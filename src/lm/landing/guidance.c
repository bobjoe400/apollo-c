/**********************************************************************
 * FILE NAME: guidance.c
 *
 * PURPOSE:
 *   LUNAR_LANDING_GUIDANCE_EQUATIONS.agc (Luminary 099,
 *   pp. 798-828): the P63/P64 quadratic guidance law, the P65 velocity
 *   nulling law, the guidance-frame state, the time-to-go root finder
 *   (ROOTPSRS, by Allan Klumpp), the window vector logic, and the guidance
 *   loop displays. R13 (GUILDENSTERN), the redesignation equations, P66/P67
 *   and the redesignator trap are in lunar_landing.c.
 *
 *   Flight sequence tables (p. 798), indexed by WCHPHASE:
 *     WCHPHASE  NEWPHASE  PREGUIDE  WHATGUID  AFTRGUID  WHATEXIT  WHATDISP
 *     -1 IGNALG TTFINCR   CALCRGVG  TTF/8CL   CGCALC    EXGSUB    --
 *      0 BRAK   TTFINCR   RGVGCALC  TTF/8CL   CGCALC    EXBRAK    P63DISPS
 *      1 APPR   STARTP64  REDESIG   TTF/8CL   CGCALC    EXNORM    P64DISPS
 *      2 VERT   P65START  RGVGCALC  VERTGUID  STEER?    --        VERTDISP
 *   The AGC's "INDEX WCHPHASE / TCF table" jumps become switch statements.
 *
 *   Frames and scales: R, V, LAND, UNFC/2 are stable-member (the landing
 *   REFSMMAT has X up at the site, Z downrange); RGU, VGU are in the guidance
 *   frame whose rows are CG. R and LAND 2^24 m, V 2^7 m/cs, ANGTERM 2^9 m/cs,
 *   VGU 2^10 m/cs, accelerations 2^-4 m/cs^2, TTF/8 2^14 cs (its high word is
 *   TTF/8 in centiseconds; it is negative before the target).
 *
 * AGC SOURCE:
 *   Luminary099 LUNAR_LANDING_GUIDANCE_EQUATIONS.agc, pp. 798-828
 *
 * EXTERNAL VARIABLES:
 *   module static ldg_tabl_ttf (TABLTTF); agc_lm_ldg(), agc_lm_pad(),
 *   agc_lm_thr() (landing, pad-load and throttle erasables);
 *   agc_guid_state() (R, V, GDT/2, XNBPIP, MASS)
 *
 * EXTERNAL REFERENCES:
 *   agc_lm_throttle, agc_lm_findcduw, agc_lm_dap_stoprate,
 *   agc_dsky_display, agc_phase_change, agc_lm_fastchng,
 *   agc_rtb_normunit, core interpretive arithmetic (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ROOTPSRS gives up after eight passes (NO_CONVERGE: outputs not to
 *   be trusted, as in the AGC); no alarms raised here
 **********************************************************************/
#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

/* ---- constants (pp. 827-828; CONTROLLED_CONSTANTS) -------------------- */
#define LDG_HIGHESTF \
    ((agc_dp_t) 284785)                       /* 2DEC 4.34546769 B-12: max DPS thrust, 10^4 N */
#define LDG_GSCALE     ((agc_dp_t) 13107200)  /* 2DEC 100 B-11              */
#define LDG_3_8DP      ((agc_dp_t) 100663296) /* 2DEC .375                  */
#define LDG_3_4DP      ((agc_dp_t) 201326592) /* 2DEC .750                  */
#define LDG_TRIMACCL   ((agc_dp_t) 2406094)   /* 2DEC* 3.50132708E-5 B+8    */
#define LDG_PROJMAX    866                    /* DEC .42262 B-3: sin(25 deg)/8 */
#define LDG_PROJMIN    530                    /* DEC .25882 B-3: sin(15 deg)/8 */
#define LDG_HALFDEG    46                     /* 1/2DEG DEC .00278 */
#define LDG_180DEGS    180                    /* 180DEGS DEC +180 */
#define LDG_TSCALINV   8                      /* TSCALINV = BIT4 */
#define LDG_TREDESCL   (-1311)                /* TREDESCL DEC -.08                      */
#define LDG_DEC103     103                    /* -DEC103                                */
#define LDG_DEC99      99                     /* +DEC99                                 */
#define LDG_BIT8       128                    /* fractional precision for TTF (2^-7)    */
#define LDG_POSMAX     16383                  /* POSMAX                                  */
#define LDG_TTF_DEGREE 3                      /* TABLTTF cubic: A(0)..A(3)              */

/* TABLTTF, the TTF/8 cubic coefficients A(0)..A(3) (p. 807). */
static agc_dp_t ldg_tabl_ttf[LDG_TTF_DEGREE + 1]; /* TABLTTF A(0)..A(3), 2^C */

/**********************************************************************
 * TARGTDEX (LUNAR_LANDING_GUIDANCE_EQUATIONS.agc p. 801): target-parameter
 * index for a phase (IGNALG and BRAKQUAD use the braking set, APPRQUAD
 * the approach set)
 *
 * RETURNS: int -- 0 braking targets, 1 approach targets
 **********************************************************************/
static int ldg_tgt(void)
{
    agc_sp_t ph = agc_lm_ldg()->wchphase; /* WCHPHASE, -1..2, unscaled */

    return ((ph == AGC_LM_PHASE_APPRQUAD) ? 1 : 0); /* approach set */
}

/**********************************************************************
 * AGC: none (AD POSMAX / TS / CS / AD idiom, QUADGUID p. 808) --
 * max(x, 0) (overflow on TS skips CS)
 *
 * ARGUMENTS:
 *   x  int32_t  I    A-register sum, scale 2^S
 *
 * RETURNS: agc_sp_t -- max(x, 0), at most POSMAX, scale 2^S
 **********************************************************************/
static agc_sp_t ldg_posonly(int32_t x)
{
    int32_t r = (x > 0) ? x : 0; /* max(x, 0), scale 2^S */

    if (r > LDG_POSMAX)
    {
        r = LDG_POSMAX;
    }

    return ((agc_sp_t) r);
}

/**********************************************************************
 * AGC: none (TS of an A-register sum, QUADGUID p. 808) -- the word TS
 * stores: an overflowing A is stored overflow-corrected (wrapped modulo
 * 2^14). The AGC's TS would also skip the next instruction; QUADGUID
 * never overflows there on valid data, which the assertion checks.
 *
 * ARGUMENTS:
 *   x  int32_t  I    A-register sum, |x| < 2^15, scale 2^S
 *
 * RETURNS: agc_sp_t -- the stored word, scale 2^S
 **********************************************************************/
static agc_sp_t ldg_wrap(int32_t x)
{
    int32_t r = x; /* stored word, scale of x */

    if (!AGC_ASSERT(r <= LDG_POSMAX && r >= -LDG_POSMAX))
    {
        r %= 16384; /* overflow-corrected */
    }

    return ((agc_sp_t) r);
}

/**********************************************************************
 * AGC: none (oracle finding F9, ROOTPSRS pp. 823-825) -- a DP held as
 * its value and the high word the AGC actually stores: DAS adds the low
 * and the high words separately, carrying the low-word overflow, and
 * never re-signs the pair, so the high word a later CA, AD or DV reads
 * can be one off the canonical split. A high word that cannot belong
 * to v (|low| >= 2^14) is ignored.
 *
 * ARGUMENTS:
 *   v   agc_dp_t  I    DP value, scale 2^S
 *   hi  agc_sp_t  I    high word the AGC stored, 2^S
 *
 * RETURNS: agc_sp_t -- the high word a CA of v reads, 2^S
 **********************************************************************/
static agc_sp_t ldg_raw_high(agc_dp_t v, agc_sp_t hi)
{
    int32_t lo = v - (int32_t) hi * 16384; /* low word for hi, 2^S-14 */

    if (lo >= 16384 || lo <= -16384)
    {
        return (agc_dp_high(v)); /* not a split of v: canonical    */
    }

    return (hi);
}

/**********************************************************************
 * AGC: none (DAS instruction, ROOTPSRS pp. 823-825) -- DAS b into the
 * pair (a, *a_hi); returns the new value, *a_hi the new high word. b
 * is a canonical (sign-agreeing) DP.
 *
 * ARGUMENTS:
 *   a     agc_dp_t    I    augend value, scale 2^S
 *   a_hi  agc_sp_t *  I/O  augend's stored high word; the sum's
 *   b     agc_dp_t    I    addend, canonical DP, scale 2^S
 *
 * RETURNS: agc_dp_t -- a + b, scale 2^S
 **********************************************************************/
static agc_dp_t ldg_das(agc_dp_t a, agc_sp_t *a_hi, agc_dp_t b)
{
    agc_sp_t       hi = ldg_raw_high(a, *a_hi); /* stored major word, 2^S */
    agc_dp_words_t w  = {hi, (agc_sp_t) (a - (int32_t) hi * 16384)};

    w     = agc_lm_das(w, agc_dp_to_words(b));
    *a_hi = w.hi;

    return (agc_dp_from_words(w));
}

/**********************************************************************
 * DMPSUB (INTERPRETER.agc p. 1031) on the stored words: MPAC (a, b) times
 * the DP (c, d) as the AGC forms it, minor x minor (high word only),
 * major x minor, minor x major, then major x major plus the net overflow.
 * Returns (MPAC, MPAC+1); POLY never reads the MPAC+2 left behind.
 *
 * ARGUMENTS:
 *   m  agc_dp_words_t  I    MPAC words, scale 2^M
 *   x  agc_dp_words_t  I    operand words, scale 2^X
 *
 * RETURNS: agc_dp_words_t -- (MPAC, MPAC+1), scale 2^(M+X)
 **********************************************************************/
static agc_dp_words_t ldg_dmpsub(agc_dp_words_t m, agc_dp_words_t x)
{
    int            ovf = 0; /* net overflow of the middle sums (A)    */
    agc_dp_words_t mid = {0, agc_dp_high(agc_sp_mul(m.lo, x.lo))};

    mid                = agc_lm_das(mid, agc_dp_to_words(agc_sp_mul(m.hi, x.lo))); /* AD */
    mid                = agc_dp_words_das(mid,
                                          agc_dp_to_words(agc_sp_mul(m.lo, x.hi)),
                                          &ovf);
    agc_dp_words_t top = {(agc_sp_t) ovf, mid.hi};                                 /* XCH MPAC       */

    return (agc_lm_das(top, agc_dp_to_words(agc_sp_mul(m.hi, x.hi))));
}

/**********************************************************************
 * POWRSERS (INTERPRETER.agc p. 1035): coef[degree] x^degree + ... +
 * coef[0] by Horner's rule, DMPSUB by the stored words of x and DAS of
 * each coefficient ("user's responsibility to assure no overflow").
 *
 * ARGUMENTS:
 *   x       agc_dp_words_t    I    argument words (as stored), 2^X
 *   coef    const agc_dp_t *  I    coef[0..degree], scale of the result
 *   degree  int               I    polynomial degree, 1..LDG_TTF_DEGREE
 *
 * RETURNS: agc_dp_t -- value of the series, scale of coef
 **********************************************************************/
static agc_dp_t ldg_powrsers(agc_dp_words_t x, const agc_dp_t *coef, int degree)
{
    agc_dp_words_t m = agc_dp_to_words(coef[degree]); /* A(N) to MPAC   */

    for (int i = degree - 1; i >= 0; i--)
    { /* POLY2 / POLYLOOP                                            */
        m = agc_lm_das(ldg_dmpsub(m, x), agc_dp_to_words(coef[i]));
    }

    return (agc_dp_from_words(m));
}

/**********************************************************************
 * AGC: none (CA TTF/8, TDISPSET/QUADGUID pp. 808, 825) -- the high word
 * of TTF/8 as stored (see ldg_raw_high)
 *
 * RETURNS: agc_sp_t -- TTF/8 high word, cs (scale 2^14 cs)
 **********************************************************************/
agc_sp_t agc_lm_ttf8_high(void)
{
    const agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */

    agc_sp_t hi = ldg_raw_high(l->ttf_8, l->ttf_8_hi); /* stored high word of TTF/8, scale 2^14 cs */

    return (hi);
}

/**********************************************************************
 * TDISPSET (pp. 825-826): TTFDISP = TTF, and TREDES, the seconds left for
 * redesignation, which reaches zero two passes before TCGFAPPR:
 * TREDES = max(0, min(99, -.08 (TCGFAPPR + TTF/8) - 4)).
 **********************************************************************/
void agc_lm_tdispset(void)
{
    agc_lm_ldg_t *l  = agc_lm_ldg();       /* landing erasables */
    agc_sp_t      t8 = agc_lm_ttf8_high(); /* CA TTF/8, cs (scale 2^14 cs) */

    l->ttfdisp = (agc_dp_t) t8 * LDG_TSCALINV;                         /* MP TSCALINV  */
    agc_sp_t a = agc_lm_ad(agc_lm_pad()->target[1].tcgf, t8);          /* TCGFAPPR + TTF/8, cs (scale 2^14 cs) */
    int32_t  x = (int32_t) agc_lm_mp_hi(a, LDG_TREDESCL) - LDG_DEC103; /* -.08 (TCGFAPPR + TTF/8) - 103, unscaled s */
    int32_t  y = (x < 0) ? x : 0;                                      /* AD NEGMAX    */

    l->tredes = ldg_posonly(y + LDG_DEC99);
}

/**********************************************************************
 * TTFINCR (pp. 802-804): TTF/8 += (TPIP - TPIPOLD)/8, and the landing site
 * is carried along by the lunar rotation:
 *   LAND = /LAND/ UNIT(LAND - LAND (TPIP - TPIPOLD) x WM).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_ttfincr(agc_interp_t *ip)
{
    agc_lm_ldg_t *l  = agc_lm_ldg();                                                  /* landing erasables */
    agc_dp_t      dt = agc_dp_shl_round(ip, agc_dp_sub(ip, l->tpip, l->tpipold), 11); /* TPIP - TPIPOLD, rescaled to 2^17 cs */
    agc_vec_t     w  = agc_vec_cross(ip, agc_vec_scale(ip, l->land, dt), l->wm);      /* LAND dt x WM, 2^24 m */
    int           sh = 0;                                                             /* NORMUNIT shift count (unused) */

    w           = agc_rtb_normunit(ip, agc_vec_sub(ip, l->land, w), &sh, NULL, NULL);
    l->landtemp = agc_vec_shl(ip, agc_vec_scale(ip, w, l->land_mag), 1);
    l->ttf_8tmp = ldg_das(l->ttf_8tmp, &l->ttf_8tmp_hi, dt); /* DAS     */
    agc_lm_fastchng();
    l->ttf_8    = l->ttf_8tmp;
    l->ttf_8_hi = l->ttf_8tmp_hi;
    l->land     = l->landtemp;
    agc_lm_tdispset();
    agc_lm_fastchng();                                       /* since REDESIG may change LANDTEMP */

    if (!AGC_ASSERT(sh >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CALCRGVG (p. 806): in IGNALG, V from the integration output plus the
 * trim correction term computed last pass and left in UNFC/2.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_calcrgvg(agc_interp_t *ip)
{
    agc_mat_t refsmmat = agc_erase_refsmmat();                              /* REFSMMAT, half unit */
    agc_vec_t vatt1    = agc_integ_shared_ctx()->out.vatt1;                 /* 2^5 m/cs   */
    agc_vec_t v        = agc_vec_shr(agc_mat_vec(ip, &refsmmat, vatt1), 1); /* V at TPIP in SM, 2^7 m/cs */

    agc_guid_state()->lm.v = agc_vec_add(ip, v, agc_lm_ldg()->unfc_2);

    if (!AGC_ASSERT(agc_integ_shared_ctx() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_rgvgcalc(ip));
}

/**********************************************************************
 * RGVGCALC (pp. 805-807): state in guidance coordinates, VHORIZ for P65,
 * RANGEDSP, and the LPD look angle LOOKANGL (degrees) for P64:
 *   ANGTERM = V + R x WM, VGU = CG ANGTERM, RGU = CG (R - LAND),
 *   LOOKANGL = ARCSIN(UNIT(R - LAND) . XNBPIP).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_rgvgcalc(agc_interp_t *ip)
{
    agc_lm_ldg_t        *l = agc_lm_ldg();          /* landing erasables */
    const agc_guid_lm_t *g = &agc_guid_state()->lm; /* LM guidance state */

    l->angterm   = agc_vec_shr(agc_vec_add(ip, agc_vec_cross(ip, g->r, l->wm), g->v), 2);
    l->vgu       = agc_mat_vec(ip, &l->cg, l->angterm);       /* ANGTERM double sized */
    agc_vec_t hz = agc_vec_make(0, l->vgu.c[2], l->vgu.c[1]); /* (0, VGU z, VGU y), 2^10 m/cs */

    l->vhoriz    = agc_dp_shl(ip, agc_vec_abval(ip, hz), 3); /* 2^7 m/cs */
    agc_vec_t rl = agc_vec_sub(ip, g->r, l->land);           /* R - LAND, 2^24 m */

    l->rgu       = agc_vec_shl(ip, agc_mat_vec(ip, &l->cg, rl), 1);
    l->rangedsp  = agc_vec_abval(ip, l->rgu);
    int       sh = 0;                                           /* NORMUNIT shift count (unused) */
    agc_vec_t u  = agc_rtb_normunit(ip, rl, &sh, NULL, NULL);   /* UNIT(R - LAND), half unit 2^1 */
    agc_dp_t  s4 = agc_vec_dot(ip, u, g->xnbpip.row[0]);        /* sin(look angle)/4, 2^2 */
    agc_sp_t  s2 = agc_lm_ad(agc_dp_high(s4), agc_dp_high(s4)); /* DOUBLE: sin/2, scale 2^1 */
    agc_sp_t  a  = agc_lm_sparcsin_doubled(s2);                 /* SPARCSIN -1: look angle, half-revolutions 2^0 */

    l->lookangl = agc_lm_mp_hi(agc_lm_ad(a, LDG_HALFDEG), LDG_180DEGS);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ROOTPSRS (pp. 822-825, Allan Klumpp): one root of the power series
 * coef[degree] x^degree + ... + coef[0] by Newton's method from `guess`.
 * Stops when |DX| <= |precroot * guess|; "Klumpp says give up after eight
 * passes": NO_CONVERGE then, outputs not to be trusted. *passes = MODE.
 * *root_hi: on entry the guess's stored high word, on exit the root's as
 * the DAS ROOTPS chain leaves it (TDISPSET and QUADGUID read it alone).
 *
 * ARGUMENTS:
 *   coef      const agc_dp_t *  I    coef[0..degree] of the series, 2^C
 *   degree    int               I    degree, 1..LDG_TTF_DEGREE
 *   guess     agc_dp_t          I    starting root, scale 2^X
 *   precroot  agc_sp_t          I    relative precision, e.g. BIT8 (2^-7)
 *   root      agc_dp_t *        O    the root, scale 2^X
 *   root_hi   agc_sp_t *        I/O  stored high word of guess / root
 *   passes    int *             O    Newton passes made (MODE), 1..8
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_rootpsrs(const agc_dp_t *coef, int degree, agc_dp_t guess, agc_sp_t precroot, agc_dp_t *root, agc_sp_t *root_hi, int *passes)
{
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);

    /* ROOTPS = the guess as stored (DCA MPAC); SHORTMP by PRECROOT keeps
     * MPAC, MPAC+1 of the words' products. DEVIATION: the stale MPAC+2
     * product, which can only carry one LSB into DXCRIT, is ignored.   */
    agc_dp_words_t x = {ldg_raw_high(guess, *root_hi), 0}; /* ROOTPS */

    x.lo            = (agc_sp_t) (guess - (int32_t) x.hi * 16384);
    agc_dp_t gp     = agc_sp_mul(x.hi, precroot) + agc_dp_high(agc_sp_mul(x.lo, precroot)); /* guess precroot, high and low products, 2^X */
    agc_dp_t dxcrit = agc_dp_abs(gp);                                                       /* DXCRIT = |guess precroot|, 2^X */
    agc_dp_t der[AGC_LM_ROOT_MAX_DEGREE];                                                   /* DERCOFN: i coef[i], scale 2^C */

    for (int i = 1; i <= degree; i++)
    { /* DERCLOOP   */
        der[i - 1] = agc_dp_mul_int(coef[i], (agc_sp_t) i); /* DMPNSUB    */
    }

    for (int mode = 0; mode <= AGC_LM_ROOT_MAX_PASSES; mode++)
    { /* ROOTLOOP   */
        agc_dp_t d   = ldg_powrsers(x, der, degree - 1); /* derivative at x, scale 2^C */
        agc_dp_t f   = ldg_powrsers(x, coef, degree);    /* residual at x, scale 2^C */
        agc_dp_t mdx = agc_dp_div(&ip, f, d);            /* -DX = residual/derivative, 2^X */

        x = agc_lm_das(x, agc_dp_to_words(-mdx)); /* DCS MPAC, DAS ROOTPS */

        if (mode == AGC_LM_ROOT_MAX_PASSES)
        {
            break; /* BADROOT: MODE has reached BIT4 (8)               */
        }

        if (agc_dp_abs(mdx) <= dxcrit)
        { /* CCS on ABS(DX) - DXCRIT: not positive -> ROOTSTOR          */
            *root    = agc_dp_from_words(x);
            *root_hi = x.hi;
            *passes  = mode + 1;

            return (AGC_STATUS_OK);
        }
    }

    *root    = agc_dp_from_words(x);
    *root_hi = x.hi;
    *passes  = AGC_LM_ROOT_MAX_PASSES;

    return (AGC_STATUS_NO_CONVERGE);
} /* agc_lm_rootpsrs */

/**********************************************************************
 * TTF/8CL (pp. 807-808): the time-to-go cubic
 *   8 JDG z (TTF/8)^3 + 6 ADG z (TTF/8)^2 + (6 VGU z + 18 VDG z)/8 TTF/8
 *   + 3/8 (RDG z - RGU z) = 0, solved for TTF/8 from last pass's value.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_ttf8cl(agc_interp_t *ip)
{
    agc_lm_ldg_t          *l = agc_lm_ldg();                     /* landing erasables */
    const agc_lm_target_t *t = &agc_lm_pad()->target[ldg_tgt()]; /* landing target set */

    ldg_tabl_ttf[3]      = t->jfg_star;                                                                                 /* A(3) = 8 JDG z   */
    ldg_tabl_ttf[2]      = t->afg_star;                                                                                 /* A(2) = 6 ADG z   */
    ldg_tabl_ttf[1]      = agc_dp_add(ip, agc_dp_mul_trunc(l->vgu.c[2], LDG_3_4DP), t->vfg_star);
    ldg_tabl_ttf[0]      = agc_dp_mul_trunc(agc_dp_sub(ip, t->rfg.c[2], l->rgu.c[2]), LDG_3_8DP);
    agc_dp_t     root    = 0;                                                                                           /* new TTF/8, scale 2^14 cs */
    agc_sp_t     root_hi = agc_lm_ttf8_high();                                                                          /* DCA TTF/8 keeps the pair, 2^14 cs */
    int          passes  = 0;                                                                                           /* Newton passes (MODE) */
    agc_status_t st      = agc_lm_rootpsrs(ldg_tabl_ttf, LDG_TTF_DEGREE, l->ttf_8, LDG_BIT8, &root, &root_hi, &passes); /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (AGC_STATUS_NO_CONVERGE); /* WHATALM          */
    }

    l->ttf_8    = root; /* corrected TTF/8  */
    l->ttf_8_hi = root_hi;
    agc_lm_tdispset();

    if (!AGC_ASSERT(passes >= 1 && passes <= AGC_LM_ROOT_MAX_PASSES))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * QUADGUID coefficients (pp. 808-809). BUF = -(lag-diminished TTF)/TTF,
 * the lag being LEADTIME (negative); the coefficients of VGU, RDG-RGU,
 * VDG and ADG follow from BUF and BUF^2 + BUF by repeated addition.
 * DEVIATION: the AGC stores them as single words over stale low words
 * (MPAC+1, 27D, 29D, 31D); here the low words are zero.
 *
 * ARGUMENTS:
 *   c  agc_sp_t 4  O    coefficients of VGU, RDG-RGU, VDG, ADG,
 *                       scale 2^0 (single words)
 **********************************************************************/
static void ldg_quadcoef(agc_sp_t c[4])
{
    agc_sp_t t8 = agc_lm_ttf8_high(); /* CS TTF/8, cs (scale 2^14 cs) */

    if (!AGC_ASSERT(t8 < 0))
    {
        t8 = -1;
    }

    agc_sp_t x   = ldg_posonly(-(int32_t) t8 + agc_lm_pad()->leadtime); /* lag-diminished -TTF/8, >= 0, cs 2^14 */
    agc_sp_t buf = agc_lm_dv((agc_dp_t) x * 16384, t8);                 /* ZL, DV TTF/8: -ratio, scale 2^0 */
    agc_sp_t sq  = agc_lm_mp_hi(buf, buf);                              /* SQUARE: ratio^2, scale 2^0 */
    int32_t  b1  = (int32_t) sq + buf;                                  /* BUF^2 - ratio */
    int32_t  cv  = (int32_t) sq + b1;                                   /* VGU          */
    int32_t  cr  = cv + b1;                                             /* RDG - RGU    */
    int32_t  cd  = cr + b1;                                             /* VDG          */
    int32_t  ca  = cd + buf + LDG_POSMAX + b1 + b1;                     /* ADG          */

    c[0] = ldg_wrap(cv);
    c[1] = ldg_wrap(cr);
    c[2] = ldg_wrap(cd);
    c[3] = ldg_wrap(ca);
}

/**********************************************************************
 * QUADGUID (pp. 808-809), the main guidance equation as programmed:
 *   ACG = 3/4 (1/4 (RDG - RG)/(TTF/8) + VDG + VG)/(TTF/8) + ADG
 * with each term weighted by its lag coefficient.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_quadguid(agc_interp_t *ip)
{
    agc_lm_ldg_t          *l = agc_lm_ldg();                     /* landing erasables */
    const agc_lm_target_t *t = &agc_lm_pad()->target[ldg_tgt()]; /* landing target set */
    agc_sp_t               c[4];                                 /* lag coefficients, scale 2^0 */

    ldg_quadcoef(c);
    agc_vec_t v1 = agc_vec_scale(ip, l->vgu, agc_dp_from_sp(c[0]));            /* VGU term, 2^10 m/cs */
    agc_vec_t v2 = agc_vec_scale(ip, t->vfg, agc_dp_from_sp(c[2]));            /* VDG term, 2^10 m/cs */
    agc_vec_t a  = agc_vec_div(ip, agc_vec_sub(ip, t->rfg, l->rgu), l->ttf_8); /* (RDG - RGU)/(TTF/8), 2^10 m/cs */

    a = agc_vec_scale(ip, agc_vec_shr(a, 2), agc_dp_from_sp(c[1]));
    a = agc_vec_add(ip, agc_vec_add(ip, a, v2), v1);
    a = agc_vec_scale(ip, agc_vec_div(ip, a, l->ttf_8), LDG_3_4DP);
    a = agc_vec_add(ip, agc_vec_scale(ip, t->afg, agc_dp_from_sp(c[3])), a);

    if (!AGC_ASSERT(l->ttf_8 != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_afccalc(ip, a));
}

/**********************************************************************
 * AFCCALC1 .. AFCCLEND (pp. 809-810): thrust acceleration command
 * UNFC/2 = CG^T ACG - GDT/2 / GSCALE (stable member) and /AFC/ for the
 * throttle; then the pass counter. VERTGUID enters here too.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  I/O  interpreter state (OVFIND)
 *   acg  agc_vec_t       I    acceleration command in guidance axes,
 *                             2^-4 m/cs^2
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_afccalc(agc_interp_t *ip, agc_vec_t acg)
{
    agc_lm_ldg_t *l = agc_lm_ldg();                                             /* landing erasables */
    agc_vec_t     a = agc_vec_shl(ip, agc_vec_mat(ip, acg, &l->cg), 1);         /* VXM CG, VSL1: ACG in SM, 2^-4 m/cs^2 */
    agc_vec_t     g = agc_vec_div(ip, agc_guid_state()->avg.gdt_2, LDG_GSCALE); /* GDT/2 / GSCALE, 2^-4 m/cs^2 */

    l->unfc_2 = agc_vec_sub(ip, a, g);        /* need not be unitized       */
    l->afc    = agc_vec_abval(ip, l->unfc_2); /* AFCCALC2: |AFC| for THROTTLE */

    /* AFCCALC2 / AFCCALC3 (pp. 809-810): limit the downrange component so the
     * horizontal demand fits within the maximum thrust:
     *   AMAXHORIZ = SQRT((HIGHESTF/MASS)^2 - A1^2 - A0^2),
     *   UNFC z >= -AMAXHORIZ.                                              */
    agc_dp_t mass = agc_guid_state()->lm.mass; /* 2^16 kg      */

    if (!AGC_ASSERT(mass > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* DDV DSQ leaves a TP; the two DSUs change MPAC, MPAC+1 only and
     * SQRT reads all three words (MPAC+2 of the DSQ takes part).      */
    agc_tp_t amax = agc_dp_sq_tp(agc_dp_div(ip, LDG_HIGHESTF, mass)); /* (HIGHESTF/MASS)^2 as a TP, 2^-8 */
    agc_dp_t a1sq = agc_dp_sq(l->unfc_2.c[1]);                        /* A1^2, popped first, 2^-8 */
    agc_dp_t a0sq = agc_dp_sq(l->unfc_2.c[0]);                        /* A0^2, 2^-8 */

    amax = agc_tp_add(ip, amax, agc_tp_from_dp(agc_dp_neg(a1sq)));
    amax = agc_tp_add(ip, amax, agc_tp_from_dp(agc_dp_neg(a0sq)));

    if (agc_tp_to_dp(amax) < 0)
    {
        amax = 0; /* ZEROVECS     */
    }

    agc_dp_t s = agc_dp_add(ip, agc_tp_sqrt(amax), l->unfc_2.c[2]); /* AMAXHORIZ + UNFC z, 2^-4 m/cs^2 */

    if (s < 0)
    {
        l->unfc_2.c[2] = agc_dp_sub(ip, l->unfc_2.c[2], s); /* BDSU    */
    }

    agc_lm_fastchng();
    l->wchphold = l->wchphase;              /* for phase switching logic  */
    l->flpass0  = agc_lm_ad(l->flpass0, 1); /* INCR FLPASS0               */

    return (AGC_STATUS_OK);
} /* agc_lm_afccalc */

/**********************************************************************
 * CGCALC (pp. 810-811): erect the guidance-to-stable-member matrix, but
 * only while TTF/8 lies between TCGFBRAK and TCGIBRAK (or the approach
 * pair). Row 0 is UNIT(LAND); row 1 is normal to LAND and to the look-
 * ahead point LAND + TTF/8 GAIN ANGTERM - R ("numero mysterioso").
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_cgcalc(agc_interp_t *ip)
{
    agc_lm_ldg_t          *l  = agc_lm_ldg();                     /* landing erasables */
    const agc_lm_target_t *t  = &agc_lm_pad()->target[ldg_tgt()]; /* landing target set */
    agc_sp_t               t8 = agc_lm_ttf8_high();               /* CA TTF/8, cs (scale 2^14 cs) */
    int32_t                ai = (int32_t) t->tcgi + t8;           /* TCGI + TTF/8, cs */
    int32_t                af = (int32_t) t->tcgf + t8;           /* TCGF + TTF/8, cs */

    if (ai > 0 && af < 0) /* TCF EXTLOGIC     */
    {
        const agc_guid_lm_t *g = &agc_guid_state()->lm; /* LM guidance state */

        l->cg.row[0] = agc_vec_unit(ip, l->land, NULL, NULL);
        agc_dp_t  k  = agc_dp_mul_trunc(l->ttf_8, t->gain);                        /* DMP* GAINBRAK: TTF/8 GAIN, 2^14 cs */
        agc_vec_t w  = agc_vec_add(ip, agc_vec_scale(ip, l->angterm, k), l->land); /* look-ahead point, 2^24 m */
        int       sh = 0;                                                          /* NORMUNIT shift count (unused) */

        w            = agc_rtb_normunit(ip, agc_vec_sub(ip, w, g->r), &sh, NULL, NULL);
        w            = agc_rtb_normunit(ip, agc_vec_cross(ip, w, l->land), &sh, NULL, NULL);
        l->cg.row[1] = w;
        l->cg.row[2] = agc_vec_shl(ip, agc_vec_cross(ip, l->cg.row[0], w), 1);
    }

    return (AGC_STATUS_OK);
} /* agc_lm_cgcalc */

/**********************************************************************
 * EXTLOGIC (p. 811): decide how to exit and whether to switch phases.
 * TENDBRAK, TENDAPPR are indexed by WCHPHASE; for IGNALG (-1) the AGC
 * reads the word before TENDBRAK, which is ZOOMTIME. Returns the phase
 * whose WHATEXIT entry is taken.
 *
 * RETURNS: agc_sp_t -- phase whose WHATEXIT entry is taken
 *          (AGC_LM_PHASE_*)
 **********************************************************************/
static agc_sp_t ldg_extlogic(void)
{
    agc_lm_ldg_t       *l    = agc_lm_ldg();         /* landing erasables */
    const agc_lm_pad_t *p    = agc_lm_pad();         /* pad loads */
    agc_sp_t            tend = agc_erase_zoomtime(); /* TENDBRAK/TENDAPPR (ZOOMTIME), cs 2^14 */

    if (l->wchphase == AGC_LM_PHASE_BRAKQUAD)
    {
        tend = p->tendbrak;
    }
    else if (l->wchphase == AGC_LM_PHASE_APPRQUAD)
    {
        tend = p->tendappr;
    }

    if ((int32_t) tend + agc_lm_ttf8_high() <= 0)
    {
        return (l->wchphase); /* BZMF WHATEXIT    */
    }

    agc_lm_fastchng();
    l->wchphase = (agc_sp_t) (l->wchphold + 1); /* next phase       */
    l->flpass0  = 0;

    return (l->wchphold);
}

/**********************************************************************
 * RATESTOP (p. 813): unless in attitude hold, a STOPRATE.
 **********************************************************************/
static void ldg_ratestop(void)
{
    if (!agc_lm_chan_bit_present(AGC_CHAN_CHAN31, AGC_CH031_ATTHOLD))
    {
        agc_lm_dap_stoprate();
    }
}

/**********************************************************************
 * STEER? / EXVERT / EXOVFLOW / RATESTOP / GDUMP1 (pp. 813-814): with
 * STEERSW up and no overflow anywhere in guidance, THROTTLE and FINDCDUW;
 * then the guidance loop displays.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_steer_exit(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (!agc_flag_test(AGC_FLAG_STEERSW))
    {
        ldg_ratestop(); /* no outputs       */
    }
    else if (ip->ovfind != 0)
    {
        agc_alarm(AGC_ALARM_01410); /* non-abortive     */
        ldg_ratestop();
    }
    else
    {
        agc_status_t st = agc_lm_throttle(); /* GDUMP1           */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        const agc_status_t st_cduw = agc_lm_findcduw_from_unfc(ip); /* step status */

        if (st_cduw != AGC_STATUS_OK)
        {
            return (st_cduw);
        }
    }

    return (agc_lm_dispexit(ctx, true));
}

/**********************************************************************
 * GODSPR/GOPERF4R requests of the landing displays (pp. 814-815) -- a
 * display request with the landing continuations
 *
 * ARGUMENTS:
 *   ctx   const agc_job_ctx_t *  I    executive job context
 *   kind  agc_dsky_disp_t        I    GODSPR, GOPERF4R or GOFLASHR request
 *   noun  uint8_t                I    noun of the V06 display (60, 63, 64)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ldg_display(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint8_t noun)
{
    const agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH,
                                  AGC_JOB_APPROACH_PHASE_PROCEED,
                                  AGC_JOB_APPROACH_PHASE_DISPLAY};

    return (agc_dsky_display(ctx, kind, agc_dsky_vn(6, noun), &cont));
}

/**********************************************************************
 * P64DISPS .. REDES-OK (pp. 814-815): V06N64, flashing until the crew
 * proceeds to permit redesignations; static once REDFLAG is set or TREDES
 * has run out (RED-OVER clears REDFLAG).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_approach_phase_display(agc_job_ctx_t *ctx)
{
    if (agc_lm_ldg()->tredes == 0)
    {
        agc_flag_clear(AGC_FLAG_REDFLAG); /* RED-OVER         */

        return (ldg_display(ctx, AGC_DSKY_REGODSPR, 64));
    }

    if (agc_flag_test(AGC_FLAG_REDFLAG))
    {
        return (ldg_display(ctx, AGC_DSKY_REGODSPR, 64)); /* REDES-OK     */
    }

    return (ldg_display(ctx, AGC_DSKY_REFLASHR, 64)); /* TCF ENDLLJOB; answers start jobs */
}

/**********************************************************************
 * P64CEED (p. 815): PROCEED on V06N64 enables the redesignation logic.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_approach_phase_proceed(void)
{
    agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */

    l->elincr1 = 0;
    l->azincr1 = 0;
    agc_flag_set(AGC_FLAG_REDFLAG);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * DISPEXIT (p. 814): kill group 3 (the next guidance cycle restores the
 * displays); unless FLUNDISP, the phase display: V06N63 (P63), P64DISPS,
 * V06N60 (P65-P67). DISPEXIT +3 enters with kill_group3 false.
 *
 * ARGUMENTS:
 *   ctx          const agc_job_ctx_t *  I    executive job context
 *   kill_group3  bool                   I    false: DISPEXIT +3, no PHASCHNG
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_dispexit(const agc_job_ctx_t *ctx, bool kill_group3)
{
    if (kill_group3)
    {
        agc_phase_kill(3); /* DXCH -PHASE3     */
    }

    if (agc_flag_test(AGC_FLAG_FLUNDISP))
    {
        return (AGC_STATUS_OK); /* ENDLLJOB         */
    }

    agc_sp_t ph = agc_lm_ldg()->wchphold; /* WCHPHOLD, unscaled */

    if (ph == AGC_LM_PHASE_APPRQUAD)
    {
        agc_job_ctx_t c = *ctx; /* job context copy */

        return (agc_lm_job_approach_phase_display(&c));
    }

    uint8_t noun = (ph == AGC_LM_PHASE_VERTICAL) ? 60 : 63; /* VERTDISP */

    return (ldg_display(ctx, AGC_DSKY_REGODSPR, noun));
}

/**********************************************************************
 * P63DISPS -2 (p. 814): restart point that kills group 5, then V06N63.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_braking_phase_display(agc_job_ctx_t *ctx)
{
    agc_lm_phaschng(00035);

    return (ldg_display(ctx, AGC_DSKY_REGODSPR, 63));
}

/**********************************************************************
 * VERTGUID (p. 815): P67 (WCHVERT > 0), P66 (+0), or P65:
 *   ACG = (V2FG - VGU) / TAUVERT, then AFCCALC1 and STEER?.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_vertguid(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */

    if (l->wchvert > 0)
    {
        return (agc_lm_manual_landing_vertical(ctx, ip));
    }

    if (l->wchvert == 0)
    {
        return (agc_lm_rate_of_descent_vertical(ctx)); /* POSTJUMP         */
    }

    const agc_lm_pad_t *p   = agc_lm_pad();                                                  /* P65VERT          */
    agc_vec_t           acg = agc_vec_div(ip, agc_vec_sub(ip, p->v2fg, l->vgu), p->tauvert); /* (V2FG - VGU)/TAUVERT, 2^-4 m/cs^2 */
    agc_status_t        st  = agc_lm_afccalc(ip, acg);                                       /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_steer_exit(ctx, ip)); /* AFTRGUID: STEER? */
}

/**********************************************************************
 * STARTP64 (p. 802): P64, TTF/8 augmented by DELTTFAP, RUPT10 enabled,
 * redesignation flag initialised; continues to TTFINCR.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_start_approach_phase(void)
{
    agc_status_t  st = agc_phase_newmode(64); /* NEWMODEX         */
    agc_lm_ldg_t *l  = agc_lm_ldg();          /* landing erasables */

    l->ttf_8tmp_hi = ldg_raw_high(l->ttf_8tmp, l->ttf_8tmp_hi);
    l->ttf_8tmp    = (agc_dp_t) (l->ttf_8tmp + (agc_dp_t) agc_lm_pad()->delttfap * 16384);
    l->ttf_8tmp_hi = (agc_sp_t) (l->ttf_8tmp_hi + agc_lm_pad()->delttfap);

    /* CA DELTTFAP, ADS TTF/8TMP: the high word only                     */
    agc_chan_or(AGC_CHAN_CHAN13, AGC_CH013_RSTTRAP31A); /* BIT12 */
    agc_flag_clear(AGC_FLAG_REDFLAG);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * P65START (p. 802): P65, WCHVERT = -2, X-axis override permitted.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_start_velocity_nulling(void)
{
    agc_status_t st = agc_phase_newmode(65); /* step status */

    agc_lm_ldg()->wchvert = -2;
    agc_flag_clear(AGC_FLAG_XOVINFLG);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(agc_lm_ldg()->wchvert < 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GUILDRET +2 .. BRSPOT2 (pp. 802-804): new times, phase start (when the
 * pass counter is zero), TTFINCR, and the pre-guidance computations.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ldg_front(agc_interp_t *ip)
{
    agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */

    l->tpipold      = l->tpip;
    agc_lm_fastchng();
    l->tpip         = agc_guid_state()->avg.piptime1;
    l->ttf_8tmp     = l->ttf_8;      /* DCA TTF/8, DXCH TTF/8TMP */
    l->ttf_8tmp_hi  = l->ttf_8_hi;
    agc_status_t st = AGC_STATUS_OK; /* step status */

    if (l->flpass0 <= 0)
    { /* BRSPOT1 NEWPHASE */
        if (l->wchphase == AGC_LM_PHASE_APPRQUAD)
        {
            st = agc_lm_start_approach_phase();
        }
        else if (l->wchphase == AGC_LM_PHASE_VERTICAL)
        {
            st = agc_lm_start_velocity_nulling();
        }
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_lm_ttfincr(ip);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (l->wchphase == AGC_LM_PHASE_IGNALG)
    { /* BRSPOT2 PREGUIDE */
        return (agc_lm_calcrgvg(ip));
    }

    if (l->wchphase == AGC_LM_PHASE_APPRQUAD)
    {
        return (agc_lm_redesig(ip));
    }

    return (agc_lm_rgvgcalc(ip));
} /* ldg_front */

/**********************************************************************
 * WHATGUID .. WHATEXIT for the quadratic phases (pp. 807-813).
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t ldg_quadratic(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_ldg_t *l  = agc_lm_ldg();      /* landing erasables */
    agc_status_t  st = agc_lm_ttf8cl(ip); /* step status */

    if (st == AGC_STATUS_NO_CONVERGE)
    { /* WHATALM          */
        if (l->wchphase == AGC_LM_PHASE_IGNALG)
        {
            return (agc_poodoo(AGC_ALARM_01406)); /* 1406P00          */
        }

        agc_alarm(AGC_ALARM_01406); /* 1406ALM          */
        ldg_ratestop();

        return (agc_lm_dispexit(ctx, true));
    }

    st = agc_lm_quadguid(ip);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_lm_cgcalc(ip); /* AFTRGUID         */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_sp_t ex = ldg_extlogic(); /* phase of the WHATEXIT entry, unscaled */

    if (ex == AGC_LM_PHASE_IGNALG)
    {
        /* back to IGNALG   */
        /* EXGSUB (pp. 811-812): IGNALG exit. Trim velocity correction term
         * UNFC/2 = UNIT(UNFC/2) ZOOMTIME TRIMACCL (2^7 m/cs), used by CALCRGVG. */
        agc_lm_ldg_t *l_exgsub = agc_lm_ldg();                                            /* landing erasables */
        int           sh       = 0;                                                       /* NORMUNIT shift count (unused) */
        agc_vec_t     u        = agc_rtb_normunit(ip, l_exgsub->unfc_2, &sh, NULL, NULL); /* UNIT(UNFC/2), half unit 2^1 */

        /* VXSC ZOOMTIME takes the DP (ZOOMTIME, TENDBRAK).                  */
        agc_dp_t zt = agc_dp_pack(agc_erase_zoomtime(), agc_lm_pad()->tendbrak); /* ZOOMTIME, cs (scale 2^14 cs) */

        l_exgsub->unfc_2 = agc_vec_scale(ip, agc_vec_scale(ip, u, zt), LDG_TRIMACCL);

        if (!AGC_ASSERT(sh >= 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        if (!AGC_ASSERT(zt >= 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        return (AGC_STATUS_OK);
    }

    if (ex == AGC_LM_PHASE_BRAKQUAD)
    {
        l->unwc_2 = agc_guid_state()->avg.unitr; /* EXBRAK: UNIT/R/  */
    }
    else
    {
        /* EXNORM (pp. 812-813): window vector between UNIT(LAND - R) and CG row 2
         * according to PROJ = (UNIT(LAND - R) x XNBPIP) . CG row 1, 1/8 size:
         *   UNWC/2 = max(PROJMAX - PROJ, 0) CG row 2
         *          + max(PROJ - PROJMIN, 0) UNIT(LAND - R). */
        agc_lm_ldg_t        *l_exnorm  = agc_lm_ldg();          /* landing erasables */
        const agc_guid_lm_t *g         = &agc_guid_state()->lm; /* LM guidance state */
        int                  sh_exnorm = 0;                     /* NORMUNIT shift count (unused) */

        l_exnorm->unwc_2 = agc_rtb_normunit(ip, agc_vec_sub(ip, l_exnorm->land, g->r), &sh_exnorm, NULL, NULL);
        agc_vec_t yw     = agc_vec_cross(ip, l_exnorm->unwc_2, g->xnbpip.row[0]); /* UNIT(LAND - R) x XNBPIP, 2^2 */
        agc_dp_t  proj   = agc_vec_dot(ip, yw, l_exnorm->cg.row[1]);              /* PROJ, scale 2^3 */
        agc_sp_t  ph     = agc_dp_high(proj);                                     /* PROJ high word, 2^3 */
        agc_sp_t  buf    = ldg_posonly((int32_t) LDG_PROJMAX - ph);               /* max(PROJMAX - PROJ, 0), 2^3 */
        agc_sp_t  buf1   = ldg_posonly((int32_t) ph - LDG_PROJMIN);               /* max(PROJ - PROJMIN, 0), 2^3 */

        for (int i = 2; i >= 0; i--)
        { /* UNWCLOOP, bound 3 */
            agc_dp_t a = agc_sp_mul(buf, agc_dp_high(l_exnorm->cg.row[2].c[i])); /* CG row 2 part, 2^4 */
            agc_dp_t b = agc_sp_mul(agc_dp_high(l_exnorm->unwc_2.c[i]), buf1);   /* UNIT(LAND - R) part, 2^4 */

            l_exnorm->unwc_2.c[i] = (agc_dp_t) (a + b); /* DXCH, MP, DAS    */
        }
    }

    return (agc_lm_steer_exit(ctx, ip));
} /* ldg_quadratic */

/**********************************************************************
 * One landing guidance pass from the given entry (p. 799):
 * GUILDRET (from LUNLAND via R13), QGUIDSUB (?GUIDSUB: GUILDRET +2, first
 * IGNALG pass), GUIDSUB (later IGNALG passes: CALCRGVG), VERTGUID.
 *
 * ARGUMENTS:
 *   ctx    const agc_job_ctx_t *  I    executive job context
 *   entry  int                    I    AGC_LM_ENTRY_* : GUILDRET, QGUIDSUB,
 *                                      GUIDSUB or VERTGUID
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_guidpass(const agc_job_ctx_t *ctx, int entry)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(entry >= AGC_LM_ENTRY_GUILDRET && entry <= AGC_LM_ENTRY_VERTGUID))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */
    agc_status_t  st;               /* step status */

    if (entry == AGC_LM_ENTRY_VERTGUID)
    {
        return (agc_lm_vertguid(ctx, &ip));
    }

    if (entry == AGC_LM_ENTRY_GUIDSUB)
    {
        st = agc_lm_calcrgvg(&ip); /* skip TTFINCR     */
    }
    else
    {
        if (entry == AGC_LM_ENTRY_GUILDRET)
        {
            l->rodcount = 0;
        }

        st = ldg_front(&ip);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (l->wchphase == AGC_LM_PHASE_VERTICAL)
    { /* BRSPOT3 WHATGUID */
        return (agc_lm_vertguid(ctx, &ip));
    }

    return (ldg_quadratic(ctx, &ip));
} /* agc_lm_guidpass */

/**********************************************************************
 * LUNLAND (p. 799): the normal entry from SERVOUT, every 2 s. Group 5
 * keeps only the PIPA task; group 3 protects guidance at priority 21,
 * just higher than SERVICER's. Then R13 decides the path.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_lunland(agc_job_ctx_t *ctx)
{
    agc_lm_phaschng(00035);
    const agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_LUNLAND, AGC_TASK_NONE, 021, false, 0U };

    agc_phase_change_var(05023, &v);
    int          entry = AGC_LM_ENTRY_GUILDRET;       /* entry chosen by R13 */
    agc_status_t st    = agc_lm_guildenstern(&entry); /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_guidpass(ctx, entry));
}
