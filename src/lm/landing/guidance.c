/*
 * guidance.c -- LUNAR_LANDING_GUIDANCE_EQUATIONS.agc (Luminary 099,
 * pp. 798-828): the P63/P64 quadratic guidance law, the P65 velocity
 * nulling law, the guidance-frame state, the time-to-go root finder
 * (ROOTPSRS, by Allan Klumpp), the window vector logic, and the guidance
 * loop displays. R13 (GUILDENSTERN), the redesignation equations, P66/P67
 * and the redesignator trap are in lunar_landing.c.
 *
 * Flight sequence tables (p. 798), indexed by WCHPHASE:
 *   WCHPHASE  NEWPHASE  PREGUIDE  WHATGUID  AFTRGUID  WHATEXIT  WHATDISP
 *   -1 IGNALG TTFINCR   CALCRGVG  TTF/8CL   CGCALC    EXGSUB    --
 *    0 BRAK   TTFINCR   RGVGCALC  TTF/8CL   CGCALC    EXBRAK    P63DISPS
 *    1 APPR   STARTP64  REDESIG   TTF/8CL   CGCALC    EXNORM    P64DISPS
 *    2 VERT   P65START  RGVGCALC  VERTGUID  STEER?    --        VERTDISP
 * The AGC's "INDEX WCHPHASE / TCF table" jumps become switch statements.
 *
 * Frames and scales: R, V, LAND, UNFC/2 are stable-member (the landing
 * REFSMMAT has X up at the site, Z downrange); RGU, VGU are in the guidance
 * frame whose rows are CG. R and LAND 2^24 m, V 2^7 m/cs, ANGTERM 2^9 m/cs,
 * VGU 2^10 m/cs, accelerations 2^-4 m/cs^2, TTF/8 2^14 cs (its high word is
 * TTF/8 in centiseconds; it is negative before the target).
 */
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
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
    ((agc_dp_t) 284785)                       /* 2DEC 4.34546769 B-12: max DPS
                                                 thrust, 10^4 N             */
#define LDG_GSCALE     ((agc_dp_t) 13107200)  /* 2DEC 100 B-11              */
#define LDG_3_8DP      ((agc_dp_t) 100663296) /* 2DEC .375                  */
#define LDG_3_4DP      ((agc_dp_t) 201326592) /* 2DEC .750                  */
#define LDG_TRIMACCL   ((agc_dp_t) 2406094) /* 2DEC* 3.50132708E-5 B+8    */
#define LDG_PROJMAX    866                  /* DEC .42262 B-3: sin(25 deg)/8          */
#define LDG_PROJMIN    530                  /* DEC .25882 B-3: sin(15 deg)/8          */
#define LDG_HALFDEG    46                   /* 1/2DEG DEC .00278                      */
#define LDG_180DEGS    180                  /* 180DEGS DEC +180                       */
#define LDG_TSCALINV   8                    /* TSCALINV = BIT4                        */
#define LDG_TREDESCL   (-1311) /* TREDESCL DEC -.08                      */
#define LDG_DEC103     103     /* -DEC103                                */
#define LDG_DEC99      99      /* +DEC99                                 */
#define LDG_BIT8       128     /* fractional precision for TTF (2^-7)    */
#define LDG_POSMAX     16383
#define LDG_TTF_DEGREE 3

/* TABLTTF, the TTF/8 cubic coefficients A(0)..A(3) (p. 807). */
static agc_dp_t ldg_tabl_ttf[LDG_TTF_DEGREE + 1];

/* Target-parameter index for a phase (TARGTDEX: IGNALG and BRAKQUAD use
 * the braking set, APPRQUAD the approach set).                          */
static int ldg_tgt(void)
{
    agc_sp_t ph = agc_lm_ldg()->wchphase;

    if (!AGC_ASSERT(ph >= AGC_LM_PHASE_IGNALG && ph <= AGC_LM_PHASE_VERTICAL))
    {
        return (0); /* no valid result */
    }

    return ((ph == AGC_LM_PHASE_APPRQUAD) ? 1 : 0);
}

/* AD POSMAX / TS / CS / AD idiom: max(x, 0) (overflow on TS skips CS). */
static agc_sp_t ldg_posonly(int32_t x)
{
    if (!AGC_ASSERT(x >= -2 * LDG_POSMAX && x <= 2 * LDG_POSMAX))
    {
        return (0); /* no valid result */
    }

    int32_t r = (x > 0) ? x : 0;

    if (r > LDG_POSMAX)
    {
        r = LDG_POSMAX;
    }

    return ((agc_sp_t) r);
}

/* A word sum kept in the 16-bit A register, saturated when stored. */
static agc_sp_t ldg_sat(int32_t x)
{
    int32_t r = x;

    if (!AGC_ASSERT(r <= LDG_POSMAX && r >= -LDG_POSMAX))
    {
        r = (r > 0) ? LDG_POSMAX : -LDG_POSMAX;
    }

    if (!AGC_ASSERT(r >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) r);
}

/* A DP held as its value and the high word the AGC actually stores: DAS
 * adds the low and the high words separately, carrying the low-word
 * overflow, and never re-signs the pair, so the high word a later CA,
 * AD or DV reads can be one off the canonical split (oracle finding F9).
 * A high word that cannot belong to v (|low| >= 2^14) is ignored.       */
static agc_sp_t ldg_raw_high(agc_dp_t v, agc_sp_t hi)
{
    int32_t lo = v - (int32_t) hi * 16384;

    if (lo >= 16384 || lo <= -16384)
    {
        return (agc_dp_high(v)); /* not a split of v: canonical    */
    }

    if (!AGC_ASSERT(hi >= AGC_SP_MIN))
    {
        return (agc_dp_high(v));
    }

    return (hi);
}

/* DAS b into the pair (a, *a_hi); returns the new value, *a_hi the new
 * high word. b is a canonical (sign-agreeing) DP.                        */
static agc_dp_t ldg_das(agc_dp_t a, agc_sp_t *a_hi, agc_dp_t b)
{
    if (!AGC_ASSERT(a_hi != NULL))
    {
        return ((agc_dp_t) (a + b));
    }

    agc_sp_t hi = ldg_raw_high(a, *a_hi);
    int32_t  lo = (a - (int32_t) hi * 16384) + (b % 16384);
    int32_t  h  = (int32_t) hi + b / 16384;

    if (lo >= 16384)
    { /* low-word overflow carries into the high word */
        lo -= 16384;
        h++;
    }
    else if (lo <= -16384)
    {
        lo += 16384;
        h--;
    }

    if (!AGC_ASSERT(h >= AGC_SP_MIN && h <= -AGC_SP_MIN))
    {
        *a_hi = agc_dp_high((agc_dp_t) (a + b));
        return ((agc_dp_t) (a + b));
    }

    *a_hi = (agc_sp_t) h;

    return ((agc_dp_t) (h * 16384 + lo));
}

/* CA TTF/8: the high word of TTF/8 as stored (see ldg_raw_high).       */
agc_sp_t agc_lm_ttf8_high(void)
{
    const agc_lm_ldg_t *l = agc_lm_ldg();

    if (!AGC_ASSERT(l != NULL))
    {
        return (0);
    }

    agc_sp_t hi = ldg_raw_high(l->ttf_8, l->ttf_8_hi);

    if (!AGC_ASSERT(hi <= 0 || l->ttf_8 > 0))
    {
        return (agc_dp_high(l->ttf_8));
    }

    return (hi);
}

/* TDISPSET (pp. 825-826): TTFDISP = TTF, and TREDES, the seconds left for
 * redesignation, which reaches zero two passes before TCGFAPPR:
 * TREDES = max(0, min(99, -.08 (TCGFAPPR + TTF/8) - 4)).                */
void agc_lm_tdispset(void)
{
    agc_lm_ldg_t *l  = agc_lm_ldg();
    agc_sp_t      t8 = agc_lm_ttf8_high(); /* CA TTF/8        */

    l->ttfdisp = (agc_dp_t) t8 * LDG_TSCALINV; /* MP TSCALINV  */
    agc_sp_t a = agc_lm_ad(agc_lm_pad()->target[1].tcgf, t8);
    int32_t  x = (int32_t) agc_lm_mp_hi(a, LDG_TREDESCL) - LDG_DEC103;
    int32_t  y = (x < 0) ? x : 0;       /* AD NEGMAX    */

    l->tredes = ldg_posonly(y + LDG_DEC99);

    if (!AGC_ASSERT(l->tredes >= 0 && l->tredes <= LDG_DEC99))
    {
        return;
    }

    if (!AGC_ASSERT(l->ttfdisp <= 0 || t8 > 0))
    {
        return;
    }
}

/* TTFINCR (pp. 802-804): TTF/8 += (TPIP - TPIPOLD)/8, and the landing site
 * is carried along by the lunar rotation:
 *   LAND = /LAND/ UNIT(LAND - LAND (TPIP - TPIPOLD) x WM).              */
agc_status_t agc_lm_ttfincr(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t *l  = agc_lm_ldg();
    agc_dp_t      dt = agc_dp_shl_round(ip, agc_dp_sub(ip, l->tpip,
                                                       l->tpipold), 11);
    agc_vec_t w  = agc_vec_cross(ip, agc_vec_scale(ip, l->land, dt), l->wm);
    int       sh = 0;

    w = agc_rtb_normunit(ip, agc_vec_sub(ip, l->land,
                                         w), &sh, NULL, NULL);
    l->landtemp = agc_vec_shl(ip, agc_vec_scale(ip, w, l->land_mag), 1);
    l->ttf_8tmp = ldg_das(l->ttf_8tmp, &l->ttf_8tmp_hi, dt); /* DAS     */
    agc_lm_fastchng();
    l->ttf_8    = l->ttf_8tmp;
    l->ttf_8_hi = l->ttf_8tmp_hi;
    l->land  = l->landtemp;
    agc_lm_tdispset();
    agc_lm_fastchng(); /* since REDESIG may change LANDTEMP */

    if (!AGC_ASSERT(sh >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* CALCRGVG (p. 806): in IGNALG, V from the integration output plus the
 * trim correction term computed last pass and left in UNFC/2.          */
agc_status_t agc_lm_calcrgvg(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_vec_t vatt1    = agc_integ_shared_ctx()->out.vatt1;     /* 2^5 m/cs   */
    agc_vec_t v        = agc_vec_shr(agc_mat_vec(ip, &refsmmat, vatt1), 1);

    agc_guid_state()->lm.v = agc_vec_add(ip, v, agc_lm_ldg()->unfc_2);

    if (!AGC_ASSERT(agc_integ_shared_ctx() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_rgvgcalc(ip));
}

/* RGVGCALC (pp. 805-807): state in guidance coordinates, VHORIZ for P65,
 * RANGEDSP, and the LPD look angle LOOKANGL (degrees) for P64:
 *   ANGTERM = V + R x WM, VGU = CG ANGTERM, RGU = CG (R - LAND),
 *   LOOKANGL = ARCSIN(UNIT(R - LAND) . XNBPIP).                         */
agc_status_t agc_lm_rgvgcalc(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t        *l = agc_lm_ldg();
    const agc_guid_lm_t *g = &agc_guid_state()->lm;

    l->angterm =
        agc_vec_shr(agc_vec_add(ip, agc_vec_cross(ip, g->r, l->wm), g->v), 2);
    l->vgu = agc_mat_vec(ip, &l->cg, l->angterm); /* ANGTERM double sized */
    agc_vec_t hz = agc_vec_make(0, l->vgu.c[2], l->vgu.c[1]);

    l->vhoriz = agc_dp_shl(ip, agc_vec_abval(ip, hz), 3); /* 2^7 m/cs */
    agc_vec_t rl = agc_vec_sub(ip, g->r, l->land);

    l->rgu      = agc_vec_shl(ip, agc_mat_vec(ip, &l->cg, rl), 1);
    l->rangedsp = agc_vec_abval(ip, l->rgu);
    int       sh = 0;
    agc_vec_t u  = agc_rtb_normunit(ip, rl, &sh, NULL, NULL);
    agc_dp_t  s4 = agc_vec_dot(ip, u, g->xnbpip.row[0]);        /* sin/4     */
    agc_sp_t  s2 = agc_lm_ad(agc_dp_high(s4), agc_dp_high(s4)); /* DOUBLE */
    agc_sp_t  a  = agc_lm_sparcsin_m1(s2); /* SPARCSIN -1 */

    l->lookangl = agc_lm_mp_hi(agc_lm_ad(a, LDG_HALFDEG), LDG_180DEGS);

    if (!AGC_ASSERT(l->lookangl >= -LDG_180DEGS && l->lookangl <= LDG_180DEGS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* ROOTPSRS (pp. 822-825, Allan Klumpp): one root of the power series
 * coef[degree] x^degree + ... + coef[0] by Newton's method from `guess`.
 * Stops when |DX| <= |precroot * guess|; "Klumpp says give up after eight
 * passes": NO_CONVERGE then, outputs not to be trusted. *passes = MODE.
 * *root_hi: on entry the guess's stored high word, on exit the root's as
 * the DAS ROOTPS chain leaves it (TDISPSET and QUADGUID read it alone). */
agc_status_t agc_lm_rootpsrs(const agc_dp_t *coef, int degree, agc_dp_t guess,
                             agc_sp_t precroot, agc_dp_t *root,
                             agc_sp_t *root_hi, int *passes)
{
    if (!AGC_ASSERT(coef != NULL && root != NULL && passes != NULL) ||
        !AGC_ASSERT(root_hi != NULL) ||
        !AGC_ASSERT(degree >= 2 && degree <= AGC_LM_ROOT_MAX_DEGREE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_dp_t dxcrit = agc_dp_abs(agc_tp_to_dp(
                                     agc_tp_mul_sp(agc_tp_from_dp(guess),
                                                   precroot))); /* SHORTMP, ABS */
    agc_dp_t der[AGC_LM_ROOT_MAX_DEGREE];                 /* DERCOFN    */

    for (int i = 1; i <= degree; i++)
    {                                                       /* DERCLOOP   */
        der[i - 1] = agc_dp_mul_int(coef[i], (agc_sp_t) i); /* DMPNSUB    */
    }

    agc_dp_t x = guess; /* ROOTPS     */

    for (int mode = 0; mode <= AGC_LM_ROOT_MAX_PASSES; mode++)
    {
        agc_dp_t d   = agc_dp_poly(x, der, degree - 1); /* POWRSERS   */
        agc_dp_t f   = agc_dp_poly(x, coef, degree);
        agc_dp_t mdx = agc_dp_div(&ip, f, d);   /* -DX        */

        x = ldg_das(x, root_hi, -mdx); /* DCS MPAC, DAS ROOTPS */

        if (mode == AGC_LM_ROOT_MAX_PASSES)
        {
            break; /* BADROOT    */
        }

        if (agc_dp_high(agc_dp_abs(mdx)) <= agc_dp_high(dxcrit))
        { /* ROOTSTOR   */
            *root   = x;
            *passes = mode + 1;

            return (AGC_STATUS_OK);
        }
    }

    *root   = x;
    *passes = AGC_LM_ROOT_MAX_PASSES;

    return (AGC_STATUS_NO_CONVERGE);
}

/* TTF/8CL (pp. 807-808): the time-to-go cubic
 *   8 JDG z (TTF/8)^3 + 6 ADG z (TTF/8)^2 + (6 VGU z + 18 VDG z)/8 TTF/8
 *   + 3/8 (RDG z - RGU z) = 0, solved for TTF/8 from last pass's value. */
agc_status_t agc_lm_ttf8cl(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t          *l = agc_lm_ldg();
    const agc_lm_target_t *t = &agc_lm_pad()->target[ldg_tgt()];

    ldg_tabl_ttf[3] = t->jfg_star; /* A(3) = 8 JDG z   */
    ldg_tabl_ttf[2] = t->afg_star; /* A(2) = 6 ADG z   */
    ldg_tabl_ttf[1] =
        agc_dp_add(ip, agc_dp_mul_trunc(l->vgu.c[2], LDG_3_4DP), t->vfg_star);
    ldg_tabl_ttf[0] =
        agc_dp_mul_trunc(agc_dp_sub(ip, t->rfg.c[2], l->rgu.c[2]), LDG_3_8DP);
    agc_dp_t     root    = 0;
    agc_sp_t     root_hi = agc_lm_ttf8_high(); /* DCA TTF/8 keeps the pair */
    int          passes  = 0;
    agc_status_t st      = agc_lm_rootpsrs(ldg_tabl_ttf, LDG_TTF_DEGREE,
                                           l->ttf_8, LDG_BIT8, &root,
                                           &root_hi, &passes);

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

/* QUADGUID coefficients (pp. 808-809). BUF = -(lag-diminished TTF)/TTF,
 * the lag being LEADTIME (negative); the coefficients of VGU, RDG-RGU,
 * VDG and ADG follow from BUF and BUF^2 + BUF by repeated addition.
 * DEVIATION: the AGC stores them as single words over stale low words
 * (MPAC+1, 27D, 29D, 31D); here the low words are zero.               */
static void ldg_quadcoef(agc_sp_t c[4])
{
    agc_sp_t t8 = agc_lm_ttf8_high(); /* CS TTF/8 */

    if (!AGC_ASSERT(t8 < 0))
    {
        t8 = -1;
    }

    agc_sp_t x   = ldg_posonly(-(int32_t) t8 + agc_lm_pad()->leadtime);
    agc_sp_t buf = agc_lm_dv((agc_dp_t) x * 16384, t8); /* ZL, DV TTF/8 */
    agc_sp_t sq  = agc_lm_mp_hi(buf, buf);              /* SQUARE       */
    int32_t  b1  = (int32_t) sq + buf;                  /* BUF^2 - ratio */
    int32_t  cv  = (int32_t) sq + b1;                   /* VGU          */
    int32_t  cr  = cv + b1;                             /* RDG - RGU    */
    int32_t  cd  = cr + b1;                             /* VDG          */
    int32_t  ca  = cd + buf + LDG_POSMAX + b1 + b1;     /* ADG          */

    c[0] = ldg_sat(cv);
    c[1] = ldg_sat(cr);
    c[2] = ldg_sat(cd);
    c[3] = ldg_sat(ca);

    if (!AGC_ASSERT(buf <= 0))
    {
        return;
    }
}

/* QUADGUID (pp. 808-809), the main guidance equation as programmed:
 *   ACG = 3/4 (1/4 (RDG - RG)/(TTF/8) + VDG + VG)/(TTF/8) + ADG
 * with each term weighted by its lag coefficient.                      */
agc_status_t agc_lm_quadguid(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t          *l = agc_lm_ldg();
    const agc_lm_target_t *t = &agc_lm_pad()->target[ldg_tgt()];
    agc_sp_t               c[4];

    ldg_quadcoef(c);
    agc_vec_t v1 = agc_vec_scale(ip, l->vgu, agc_dp_from_sp(c[0]));
    agc_vec_t v2 = agc_vec_scale(ip, t->vfg, agc_dp_from_sp(c[2]));
    agc_vec_t a  = agc_vec_div(ip, agc_vec_sub(ip, t->rfg, l->rgu), l->ttf_8);

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

/* AFCCALC1 .. AFCCLEND (pp. 809-810): thrust acceleration command
 * UNFC/2 = CG^T ACG - GDT/2 / GSCALE (stable member) and /AFC/ for the
 * throttle; then the pass counter. VERTGUID enters here too.          */
agc_status_t agc_lm_afccalc(agc_interp_t *ip, agc_vec_t acg)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t *l = agc_lm_ldg();
    agc_vec_t     a = agc_vec_shl(ip, agc_vec_mat(ip, acg, &l->cg), 1); /* VXM */
    agc_vec_t     g = agc_vec_div(ip, agc_guid_state()->avg.gdt_2, LDG_GSCALE);

    l->unfc_2 = agc_vec_sub(ip, a, g);   /* need not be unitized       */
    l->afc    = agc_vec_abval(ip, l->unfc_2); /* AFCCALC2: |AFC| for THROTTLE */
    /* AFCCALC2 / AFCCALC3 (pp. 809-810): limit the downrange component so the
     * horizontal demand fits within the maximum thrust:
     *   AMAXHORIZ = SQRT((HIGHESTF/MASS)^2 - A1^2 - A0^2), UNFC z >= -AMAXHORIZ. */
    agc_dp_t mass = agc_guid_state()->lm.mass; /* 2^16 kg      */

    if (!AGC_ASSERT(mass > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t amax = agc_dp_sq(ip, agc_dp_div(ip, LDG_HIGHESTF, mass));

    amax = agc_dp_sub(ip, agc_dp_sub(ip, amax, agc_dp_sq(ip, l->unfc_2.c[1])),
                      agc_dp_sq(ip, l->unfc_2.c[0]));

    if (amax < 0)
    {
        amax = 0; /* ZEROVECS     */
    }

    agc_dp_t s = agc_dp_add(ip, agc_dp_sqrt(ip, amax), l->unfc_2.c[2]);

    if (s < 0)
    {
        l->unfc_2.c[2] = agc_dp_sub(ip, l->unfc_2.c[2], s); /* BDSU    */
    }

    if (!AGC_ASSERT(l->unfc_2.c[2] >= AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_fastchng();
    l->wchphold = l->wchphase;              /* for phase switching logic  */
    l->flpass0  = agc_lm_ad(l->flpass0, 1); /* INCR FLPASS0               */

    if (!AGC_ASSERT(l->afc >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* CGCALC (pp. 810-811): erect the guidance-to-stable-member matrix, but
 * only while TTF/8 lies between TCGFBRAK and TCGIBRAK (or the approach
 * pair). Row 0 is UNIT(LAND); row 1 is normal to LAND and to the look-
 * ahead point LAND + TTF/8 GAIN ANGTERM - R ("numero mysterioso").    */
agc_status_t agc_lm_cgcalc(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t          *l  = agc_lm_ldg();
    const agc_lm_target_t *t  = &agc_lm_pad()->target[ldg_tgt()];
    agc_sp_t               t8 = agc_lm_ttf8_high();
    int32_t                ai = (int32_t) t->tcgi + t8;
    int32_t                af = (int32_t) t->tcgf + t8;

    if (!(ai > 0 && af < 0))
    {
        return (AGC_STATUS_OK); /* TCF EXTLOGIC     */
    }

    const agc_guid_lm_t *g = &agc_guid_state()->lm;

    l->cg.row[0] = agc_vec_unit(ip, l->land, NULL, NULL);
    agc_dp_t  k  = agc_dp_mul_trunc(l->ttf_8, t->gain); /* DMP* GAINBRAK  */
    agc_vec_t w  = agc_vec_add(ip, agc_vec_scale(ip, l->angterm, k), l->land);
    int       sh = 0;

    w = agc_rtb_normunit(ip, agc_vec_sub(ip, w,
                                         g->r), &sh, NULL, NULL);
    w = agc_rtb_normunit(ip, agc_vec_cross(ip, w,
                                           l->land), &sh, NULL,
                         NULL);
    l->cg.row[1] = w;
    l->cg.row[2] = agc_vec_shl(ip, agc_vec_cross(ip, l->cg.row[0], w), 1);

    if (!AGC_ASSERT(sh >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* EXTLOGIC (p. 811): decide how to exit and whether to switch phases.
 * TENDBRAK, TENDAPPR are indexed by WCHPHASE; for IGNALG (-1) the AGC
 * reads the word before TENDBRAK, which is ZOOMTIME. Returns the phase
 * whose WHATEXIT entry is taken.                                       */
static agc_sp_t ldg_extlogic(void)
{
    agc_lm_ldg_t       *l    = agc_lm_ldg();
    const agc_lm_pad_t *p    = agc_lm_pad();
    agc_sp_t            tend = agc_erase_zoomtime();

    if (l->wchphase == AGC_LM_PHASE_BRAKQUAD)
    {
        tend = p->tendbrak;
    }
    else if (l->wchphase == AGC_LM_PHASE_APPRQUAD)
    {
        tend = p->tendappr;
    }

    if (!AGC_ASSERT(l->wchphase < AGC_LM_PHASE_VERTICAL))
    {
        return (0); /* no valid result */
    }

    if ((int32_t) tend + agc_lm_ttf8_high() <= 0)
    {
        return (l->wchphase); /* BZMF WHATEXIT    */
    }

    agc_lm_fastchng();
    l->wchphase = (agc_sp_t) (l->wchphold + 1); /* next phase       */
    l->flpass0  = 0;

    if (!AGC_ASSERT(l->wchphase <= AGC_LM_PHASE_VERTICAL))
    {
        return (0); /* no valid result */
    }

    return (l->wchphold);
}

/* EXGSUB (pp. 811-812): IGNALG exit. Trim velocity correction term
 * UNFC/2 = UNIT(UNFC/2) ZOOMTIME TRIMACCL (2^7 m/cs), used by CALCRGVG. */
static agc_status_t ldg_exgsub(agc_interp_t *ip)
{
    agc_lm_ldg_t *l  = agc_lm_ldg();
    int           sh = 0;
    agc_vec_t     u  = agc_rtb_normunit(ip, l->unfc_2, &sh, NULL, NULL);
    /* VXSC ZOOMTIME takes the DP (ZOOMTIME, TENDBRAK).                  */
    agc_dp_t zt =
        agc_dp_pack(agc_erase_zoomtime(), agc_lm_pad()->tendbrak);

    l->unfc_2 = agc_vec_scale(ip, agc_vec_scale(ip, u, zt), LDG_TRIMACCL);

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

/* EXNORM (pp. 812-813): window vector between UNIT(LAND - R) and CG row 2
 * according to PROJ = (UNIT(LAND - R) x XNBPIP) . CG row 1, 1/8 size:
 *   UNWC/2 = max(PROJMAX - PROJ, 0) CG row 2
 *          + max(PROJ - PROJMIN, 0) UNIT(LAND - R).                    */
static void ldg_exnorm(agc_interp_t *ip)
{
    agc_lm_ldg_t        *l  = agc_lm_ldg();
    const agc_guid_lm_t *g  = &agc_guid_state()->lm;
    int                  sh = 0;

    l->unwc_2 =
        agc_rtb_normunit(ip, agc_vec_sub(ip, l->land, g->r), &sh, NULL, NULL);
    agc_dp_t proj = agc_vec_dot(
        ip, agc_vec_cross(ip, l->unwc_2, g->xnbpip.row[0]), l->cg.row[1]);
    agc_sp_t ph   = agc_dp_high(proj);
    agc_sp_t buf  = ldg_posonly((int32_t) LDG_PROJMAX - ph);
    agc_sp_t buf1 = ldg_posonly((int32_t) ph - LDG_PROJMIN);

    for (int i = 2; i >= 0; i--)
    { /* UNWCLOOP, bound 3 */
        agc_dp_t a = agc_sp_mul(buf, agc_dp_high(l->cg.row[2].c[i]));
        agc_dp_t b = agc_sp_mul(agc_dp_high(l->unwc_2.c[i]), buf1);

        l->unwc_2.c[i] = (agc_dp_t) (a + b); /* DXCH, MP, DAS    */
    }

    if (!AGC_ASSERT(buf >= 0 && buf1 >= 0))
    {
        return;
    }

    if (!AGC_ASSERT(sh >= 0))
    {
        return;
    }
}

/* RATESTOP (p. 813): unless in attitude hold, a STOPRATE. */
static void ldg_ratestop(void)
{
    if (!agc_lm_chan_bit_present(AGC_CHAN_CHAN31, AGC_CH031_ATTHOLD))
    {
        agc_lm_dap_stoprate();
    }

    if (!AGC_ASSERT(agc_lm_ldg()->wchphold >= AGC_LM_PHASE_IGNALG))
    {
        return;
    }
}

/* STEER? / EXVERT / EXOVFLOW / RATESTOP / GDUMP1 (pp. 813-814): with
 * STEERSW up and no overflow anywhere in guidance, THROTTLE and FINDCDUW;
 * then the guidance loop displays.                                      */
agc_status_t agc_lm_steer_exit(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

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

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        const agc_status_t st_cduw = agc_lm_findcduw_m2(ip);

        if (!AGC_ASSERT(st_cduw == AGC_STATUS_OK))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (agc_lm_dispexit(ctx, true));
}

/* A display request with the landing continuations. */
static agc_status_t ldg_display(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                                uint8_t noun)
{
    const agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH,
                                  AGC_JOB_APPROACH_PHASE_PROCEED,
                                  AGC_JOB_APPROACH_PHASE_DISPLAY};

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(noun == 60 || noun == 63 || noun == 64))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, kind, agc_dsky_vn(6, noun), &cont));
}

/* P64DISPS .. REDES-OK (pp. 814-815): V06N64, flashing until the crew
 * proceeds to permit redesignations; static once REDFLAG is set or TREDES
 * has run out (RED-OVER clears REDFLAG).                               */
agc_status_t agc_lm_job_approach_phase_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_lm_ldg()->tredes == 0)
    {
        agc_flag_clear(AGC_FLAG_REDFLAG); /* RED-OVER         */

        return (ldg_display(ctx, AGC_DSKY_REGODSPR, 64));
    }

    if (agc_flag_test(AGC_FLAG_REDFLAG))
    {
        return (ldg_display(ctx, AGC_DSKY_REGODSPR, 64)); /* REDES-OK     */
    }

    agc_status_t st = ldg_display(ctx, AGC_DSKY_REFLASHR, 64);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st); /* TCF ENDLLJOB; answers start jobs */
}

/* P64CEED (p. 815): PROCEED on V06N64 enables the redesignation logic. */
agc_status_t agc_lm_job_approach_phase_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t *l = agc_lm_ldg();

    l->elincr1 = 0;
    l->azincr1 = 0;
    agc_flag_set(AGC_FLAG_REDFLAG);

    return (AGC_STATUS_OK);
}

/* DISPEXIT (p. 814): kill group 3 (the next guidance cycle restores the
 * displays); unless FLUNDISP, the phase display: V06N63 (P63), P64DISPS,
 * V06N60 (P65-P67). DISPEXIT +3 enters with kill_group3 false.         */
agc_status_t agc_lm_dispexit(const agc_job_ctx_t *ctx, bool kill_group3)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (kill_group3)
    {
        agc_phase_kill(3); /* DXCH -PHASE3     */
    }

    if (agc_flag_test(AGC_FLAG_FLUNDISP))
    {
        return (AGC_STATUS_OK); /* ENDLLJOB         */
    }

    agc_sp_t ph = agc_lm_ldg()->wchphold;

    if (!AGC_ASSERT(ph >= AGC_LM_PHASE_BRAKQUAD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (ph == AGC_LM_PHASE_APPRQUAD)
    {
        agc_job_ctx_t c = *ctx;

        return (agc_lm_job_approach_phase_display(&c));
    }

    uint8_t noun = (ph == AGC_LM_PHASE_VERTICAL) ? 60 : 63; /* VERTDISP */

    return (ldg_display(ctx, AGC_DSKY_REGODSPR, noun));
}

/* P63DISPS -2 (p. 814): restart point that kills group 5, then V06N63. */
agc_status_t agc_lm_job_braking_phase_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_phaschng(00035);

    return (ldg_display(ctx, AGC_DSKY_REGODSPR, 63));
}

/* VERTGUID (p. 815): P67 (WCHVERT > 0), P66 (+0), or P65:
 *   ACG = (V2FG - VGU) / TAUVERT, then AFCCALC1 and STEER?.            */
agc_status_t agc_lm_vertguid(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL && ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t *l = agc_lm_ldg();

    if (l->wchvert > 0)
    {
        return (agc_lm_manual_landing_vertical(ctx, ip));
    }

    if (l->wchvert == 0)
    {
        return (agc_lm_rate_of_descent_vertical(ctx)); /* POSTJUMP         */
    }

    const agc_lm_pad_t *p   = agc_lm_pad(); /* P65VERT          */
    agc_vec_t           acg =
        agc_vec_div(ip, agc_vec_sub(ip, p->v2fg, l->vgu), p->tauvert);
    agc_status_t st = agc_lm_afccalc(ip, acg);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_steer_exit(ctx, ip)); /* AFTRGUID: STEER? */
}

/* STARTP64 (p. 802): P64, TTF/8 augmented by DELTTFAP, RUPT10 enabled,
 * redesignation flag initialised; continues to TTFINCR.               */
agc_status_t agc_lm_start_approach_phase(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t  st = agc_phase_newmode(64); /* NEWMODEX         */
    agc_lm_ldg_t *l  = agc_lm_ldg();

    l->ttf_8tmp_hi = ldg_raw_high(l->ttf_8tmp, l->ttf_8tmp_hi);
    l->ttf_8tmp    = (agc_dp_t) (l->ttf_8tmp +
                                 (agc_dp_t) agc_lm_pad()->delttfap * 16384);
    l->ttf_8tmp_hi = (agc_sp_t) (l->ttf_8tmp_hi + agc_lm_pad()->delttfap);
    /* CA DELTTFAP, ADS TTF/8TMP: the high word only                     */
    agc_chan_or(AGC_CHAN_CHAN13, AGC_CH013_RSTTRAP31A); /* BIT12 */
    agc_flag_clear(AGC_FLAG_REDFLAG);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* P65START (p. 802): P65, WCHVERT = -2, X-axis override permitted. */
agc_status_t agc_lm_start_velocity_nulling(void)
{
    agc_status_t st = agc_phase_newmode(65);

    agc_lm_ldg()->wchvert = -2;
    agc_flag_clear(AGC_FLAG_XOVINFLG);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_lm_ldg()->wchvert < 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* GUILDRET +2 .. BRSPOT2 (pp. 802-804): new times, phase start (when the
 * pass counter is zero), TTFINCR, and the pre-guidance computations.  */
static agc_status_t ldg_front(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t *l = agc_lm_ldg();

    l->tpipold = l->tpip;
    agc_lm_fastchng();
    l->tpip     = agc_guid_state()->avg.piptime1;
    l->ttf_8tmp    = l->ttf_8; /* DCA TTF/8, DXCH TTF/8TMP */
    l->ttf_8tmp_hi = l->ttf_8_hi;
    agc_status_t st = AGC_STATUS_OK;

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

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
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
}

/* WHATGUID .. WHATEXIT for the quadratic phases (pp. 807-813). */
static agc_status_t ldg_quadratic(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_ldg_t *l  = agc_lm_ldg();
    agc_status_t  st = agc_lm_ttf8cl(ip);

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

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_sp_t ex = ldg_extlogic();

    if (ex == AGC_LM_PHASE_IGNALG)
    {
        return (ldg_exgsub(ip)); /* back to IGNALG   */
    }

    if (ex == AGC_LM_PHASE_BRAKQUAD)
    {
        l->unwc_2 = agc_guid_state()->avg.unitr; /* EXBRAK: UNIT/R/  */
    }
    else
    {
        ldg_exnorm(ip);
    }

    return (agc_lm_steer_exit(ctx, ip));
}

/* One landing guidance pass from the given entry (p. 799):
 * GUILDRET (from LUNLAND via R13), QGUIDSUB (?GUIDSUB: GUILDRET +2, first
 * IGNALG pass), GUIDSUB (later IGNALG passes: CALCRGVG), VERTGUID.      */
agc_status_t agc_lm_guidpass(const agc_job_ctx_t *ctx, int entry)
{
    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(
            entry >= AGC_LM_ENTRY_GUILDRET && entry <= AGC_LM_ENTRY_VERTGUID))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_lm_ldg_t *l = agc_lm_ldg();
    agc_status_t  st;

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
}

/* LUNLAND (p. 799): the normal entry from SERVOUT, every 2 s. Group 5
 * keeps only the PIPA task; group 3 protects guidance at priority 21,
 * just higher than SERVICER's. Then R13 decides the path.             */
agc_status_t agc_lm_job_lunland(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_phaschng(00035);
    const agc_phase_var_t v = {
        AGC_RESTART_JOB, AGC_JOB_LUNLAND, AGC_TASK_NONE, 021, false, 0U
    };

    agc_phase_change_var(05023, &v);
    int          entry = AGC_LM_ENTRY_GUILDRET;
    agc_status_t st    = agc_lm_guildenstern(ctx, &entry);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_guidpass(ctx, entry));
}
