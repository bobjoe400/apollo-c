/*
 * gts.c -- the descent-engine trim gimbal control system
 * (TRIM_GIMBAL_CONTROL_SYSTEM.agc, Luminary 099 pp.1472-1484): the GTS
 * attitude/rate/acceleration control law, GTSQRT / ROOTCYCL (Craig Work),
 * ACDT+C12 and TIMEGMBL.
 *
 * Drive-setting algorithm, per axis (Q, R), from the AGC comments:
 *     DEL     = SGN(OMEGA + ALPHA ABS(ALPHA) / (2K))
 *     NEGUSUM = ERROR K^2 + ALPHA (DEL OMEGA + ALPHA^2/(3K))
 *               + DEL K^(1/2) (DEL OMEGA + ALPHA^2/(2K))^(3/2)
 *     DRIVE   = -SGN(NEGUSUM);  NEGU = SGN(NEGUSUM)
 * ERROR (QDIFF = QERROR) at pi, OMEGA (EDOTQ) at pi/4 rad/s, ALPHA (AOS)
 * at pi/4 rad/s^2 here, K (KQ, KRDAP) at pi/2^8 rad/s^3; NEGUSUM DP at
 * 4 pi^2.
 *
 * The GTS temporaries overlay DAPTEMP1.. (KCENTRAL = SHFTFLAG, ACENTRAL =
 * ININDEX, A2CNTRAL = SCRATCH/HALFARG); each is dead before its overlay
 * is written, so they are separate locals here.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"
#include "internal.h"

#define GT_BGIM     007400u /* BGIM: channel 12 bits 9-12              */
#define GT_GIMQ     001400u /* GMBLBITA: bits 9,10 (Q drive)           */
#define GT_GIMR     006000u /* GMBLBITB: bits 11,12 (R drive)          */
#define GT_BIT9     0000400u
#define GT_BIT10    0001000u
#define GT_BIT11    0002000u
#define GT_BIT12    0004000u
#define GT_NEG1_3   ((agc_sp_t) -5461) /* NEG1/3 DEC -.33333               */
#define GT_ROOTHALF ((agc_sp_t) 11585) /* ROOTHALF 26501: sqrt(1/2)        */
#define GT_POINT6   ((agc_sp_t) 9830)  /* OCT23146: decimal .6             */
#define GT_10_1024  ((agc_sp_t) 160)   /* OCT00240: decimal 10/1024        */
#define GT_OCT31    ((agc_sp_t) 25)    /* OCT31: 2.5 s at 0.1 s           */
#define GT_TWO      ((agc_sp_t) 2)
#define GT_SKIPOFF  ((agc_sp_t) 4)

/* DP x SP as the GTS codes it (MP high, DXCH, MP low, ADS with carry).   */
static lm_dp_pair_t dp_x_sp(lm_dp_pair_t x, agc_sp_t k)
{
    lm_dp_pair_t hi = lm_dap_mp(x.hi, k);
    lm_dp_pair_t lo = {0, lm_dap_mphi(x.lo, k)};
    lm_dp_pair_t r  = lm_dap_das(hi, lo);

    if (!AGC_ASSERT(r.hi >= LM_NEGMAX && r.hi <= LM_POSMAX))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    return (r);
}

static lm_dp_pair_t dp_neg(lm_dp_pair_t x) /* DCS                  */
{
    lm_dp_pair_t r = {(agc_sp_t) -x.hi, (agc_sp_t) -x.lo};

    if (!AGC_ASSERT(r.hi >= LM_NEGMAX && r.lo >= LM_NEGMAX))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    return (r);
}

/* Sign of a DP pair, high word first (CCS high, then CCS low).          */
static agc_sp_t dp_sign(lm_dp_pair_t x)
{
    agc_sp_t w = (x.hi != 0) ? x.hi : x.lo;

    if (!AGC_ASSERT(x.hi >= LM_NEGMAX && x.lo >= LM_NEGMAX))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) ((w > 0) ? 1 : ((w < 0) ? -1 : 0)));
}

/* ACDT+C12 (pp.1476-1477): Q(R)ACCDOT = -NEGU x ACCDOT(magnitude); the
 * drive bits of channel 12 from NEGUQ, NEGUR; clear the CALLGMBL request.
 * Called with interrupts inhibited.                                     */
agc_status_t lm_dap_gimbal_drive_update(void)
{
    lm_dap_t *d = lm_dap_state();

    if (!AGC_ASSERT(d->neguq >= -1 && d->neguq <= 1) ||
        !AGC_ASSERT(d->negur >= -1 && d->negur <= 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    d->qaccdot = lm_dap_mp((agc_sp_t) -d->neguq, d->accdotq).lo;
    d->raccdot = lm_dap_mp((agc_sp_t) -d->negur, d->accdotr).lo;
    uint16_t bits = (d->neguq > 0) ? GT_BIT10 : ((d->neguq < 0) ? GT_BIT9 : 0u);

    bits =
        (uint16_t) (bits + ((d->negur > 0) ? GT_BIT12 :
                            ((d->negur < 0) ? GT_BIT11 : 0u)));
    uint16_t c12 =
        (uint16_t) (agc_chan_read(012) & (uint16_t) ~GT_BGIM & 077777u);

    agc_chan_write(012, (uint16_t) (c12 + bits));
    d->rcsflags = (uint16_t) (d->rcsflags & ~LM_RCS_CALLGMBL);

    return (AGC_STATUS_OK);
}

/* ROOTCYCL (p.1484): one Newton iteration x' = x/2 + (ARG/2)/x.          */
static agc_sp_t rootcycl(agc_sp_t x, agc_sp_t halfarg)
{
    if (!AGC_ASSERT(x > 0))
    {
        return (0);
    }

    lm_dp_pair_t n = {halfarg, 0};
    agc_sp_t     q = lm_dap_dv(n, x);         /* (ARG/X)/2      */

    return (lm_dap_sat((int32_t) (x / 2) + q)); /* SR: X/2        */
}

/* GTSQRT (pp.1481-1482): 14-bit square root of the DP *fn; the root must
 * be shifted right by SHFTFLAG + ININDEX/2. Zero or negative -> 0.      */
agc_sp_t lm_dap_gtsqrt(lm_dp_pair_t *fn, agc_sp_t *shftflag, agc_sp_t *inindex)
{
    if (!AGC_ASSERT(fn != NULL && shftflag != NULL) ||
        !AGC_ASSERT(inindex != NULL))
    {
        return (0);
    }

    *shftflag = 0;

    if (fn->hi < 0 || (fn->hi == 0 && fn->lo <= 0))
    {
        return (0); /* ZEROOT         */
    }

    if (fn->hi == 0)
    { /* ZEROHIGH       */
        fn->hi    = fn->lo;
        fn->lo    = 0;
        *shftflag = 7;
    }

    *inindex = 12;     /* TWELVE         */
    agc_sp_t a = fn->hi; /* SCALSTRT       */

    for (int i = 0; i < 6; i++)
    { /* SCALLOOP       */
        agc_sp_t ref = (agc_sp_t) (1 << (14 - *inindex));

        if ((int32_t) ref - fn->hi > 0)
        {                            /* arg < reference */
            a = lm_dap_dv(*fn, ref); /* HALFARG        */
            break;
        }

        *inindex = (agc_sp_t) (*inindex - 2); /* MULBUSH        */

        if (*inindex <= 0)
        {
            break;
        }
    }

    agc_sp_t half = lm_dap_mphi(a, (agc_sp_t) 8192); /* SCALDONE       */
    agc_sp_t x    = ((half & 4096) != 0) ? LM_POSMAX : GT_ROOTHALF;

    for (int k = 0; k < 3; k++)
    {
        x = rootcycl(x, half);
    }

    if (!AGC_ASSERT(*inindex >= 0 && *inindex <= 12))
    {
        return (0);
    }

    return (x);
}

/* GTS working cells for one axis (GTSTEMPS).                             */
typedef struct gts_w
{
    lm_dp_pair_t k2theta; /* K2THETA: NEGUSUM, DP at 4 pi^2           */
    lm_dp_pair_t k2;      /* K2CNTRAL, DP at 16 pi                    */
    lm_dp_pair_t a2;      /* A2CNTRAL, DP at 16 pi                    */
    lm_dp_pair_t fn;      /* FUNCTION, DP                             */
    agc_sp_t     k;       /* KCENTRAL at pi/2^8                       */
    agc_sp_t     omega;   /* WCENTRAL at pi/4                         */
    agc_sp_t     alpha;   /* ACENTRAL at pi/4                         */
    agc_sp_t     del;     /* DEL: +1, 0, -1                           */
} gts_w_t;

/* GOQTRIMG .. GTSQAXIS (p.1473): AOS rescaled to pi/4 (limited), rate,
 * K of the axis (0 = Q, 1 = R).                                          */
static void gts_inputs(const lm_dap_t *d, int i, gts_w_t *w)
{
    agc_sp_t     aos = (i == 0) ? d->aosq : d->aosr; /* at pi/2        */
    lm_dp_pair_t p   = lm_dap_mp(aos, (agc_sp_t) 2); /* MP BIT2        */

    w->alpha = (p.hi == 0) ? p.lo : ((p.hi > 0) ? LM_POSMAX : LM_NEGMAX);
    w->omega = (i == 0) ? d->edotq : d->edotr;
    w->k     = (i == 0) ? d->kq : d->krdap;

    if (!AGC_ASSERT(w->k >= 0))
    {
        return;
    }

    if (!AGC_ASSERT(i == 0 || i == 1))
    {
        return;
    }
}

/* ALGORTHM (pp.1473-1474): K2THETA = K ERROR rescaled to 4 pi^2, and
 * A2CNTRAL = ALPHA^2/(2K) at 16 pi (POSMAX,POSMAX if K <= ALPHA^2/2).   */
static void gts_first_terms(const lm_dap_t *d, int i, gts_w_t *w)
{
    agc_sp_t     err  = (i == 0) ? d->qerror : d->rerror; /* QDIFF, RDIFF   */
    lm_dp_pair_t ke   = lm_dap_mp(err, w->k);             /* at pi^2/2^8    */
    lm_dp_pair_t hi16 = lm_dap_mp(ke.hi, (agc_sp_t) 16);  /* MP BIT5        */

    w->k2theta.hi = hi16.hi;
    w->k2theta.lo =
        lm_dap_wrap((int32_t) hi16.lo + lm_dap_mphi(ke.lo, (agc_sp_t) 16));
    agc_sp_t     half = lm_dap_mphi((agc_sp_t) -w->alpha, (agc_sp_t) 8192);
    lm_dp_pair_t sq   = lm_dap_mp(half, w->alpha); /* -ALPHA^2/2     */

    if ((int32_t) sq.hi + w->k <= 0)
    { /* HUGEQUOT       */
        w->a2.hi = LM_POSMAX;
        w->a2.lo = LM_POSMAX;

        return;
    }

    lm_dp_pair_t num = dp_neg(sq); /* ALPHA^2/2      */
    lm_dp_pair_t q1  = lm_dap_dvqr(num, w->k);
    lm_dp_pair_t rem = {q1.lo, 0}; /* LXCH 7         */

    w->a2.hi = q1.hi;
    w->a2.lo = lm_dap_dv(rem, w->k);

    if (!AGC_ASSERT(w->a2.hi >= 0 && w->a2.lo >= 0))
    {
        return;
    }
}

/* HAVEQUOT .. FUNCT3 (pp.1474-1475): DEL, K2CNTRAL = DEL OMEGA +
 * ALPHA^2/(2K), and the ALPHA (DEL OMEGA + ALPHA^2/(3K)) term of
 * NEGUSUM.                                                              */
static void gts_middle_terms(gts_w_t *w)
{
    w->k2 = lm_dap_mp(w->omega, (agc_sp_t) 256); /* OMEGA at 16 pi */
    lm_dp_pair_t f = (w->alpha > 0) ? lm_dap_das(w->k2, w->a2) :
                     lm_dap_das(w->k2, dp_neg(w->a2));

    w->del = dp_sign(f);

    if (w->del == 0)
    { /* DEFUNCT        */
        w->k2.hi = 0;
        w->k2.lo = 0;
    }
    else if (w->del < 0)
    { /* NEGFNCT2       */
        w->k2 = dp_neg(w->k2);
    }

    w->k2      = lm_dap_das(w->k2, w->a2);  /* FUNCT2         */
    w->a2      = dp_x_sp(w->a2, GT_NEG1_3); /* -ALPHA^2/(6K)  */
    w->a2      = lm_dap_das(w->a2, w->k2);
    w->k2theta = lm_dap_das(w->k2theta, dp_x_sp(w->a2, w->alpha));

    if (!AGC_ASSERT(w->del >= -1 && w->del <= 1))
    {
        return;
    }

    if (!AGC_ASSERT(w->k2theta.hi >= LM_NEGMAX))
    {
        return;
    }
}

/* GETROOT .. ADDITIN (pp.1475-1483): add DEL K^(1/2) (K2CNTRAL)^(3/2)
 * using GTSQRT of K K2CNTRAL and the shift it reports.                  */
static void gts_root_term(gts_w_t *w)
{
    w->fn = dp_x_sp(w->k2, w->k); /* at pi^2/16     */

    if (w->del == 0)
    {
        return; /* NEGUSUM        */
    }

    if (w->del < 0)
    {
        w->k2 = dp_neg(w->k2); /* NEGATE         */
    }

    agc_sp_t shft    = 0;
    agc_sp_t inindex = 0;
    agc_sp_t root    = lm_dap_gtsqrt(&w->fn, &shft, &inindex);

    w->k2 = dp_x_sp(w->k2, root); /* PRODUCT        */
    shft  =
        (agc_sp_t) (shft + lm_dap_mphi(inindex, (agc_sp_t) 8192)); /* DOSHIFT */

    if (!AGC_ASSERT(shft >= 0 && shft <= 14))
    {
        return;
    }

    if (shft != 0)
    {
        w->k2 = dp_x_sp(w->k2, (agc_sp_t) (1 << (14 - shft))); /* 2^-S     */
    }

    w->k2theta = lm_dap_das(w->k2theta, w->k2); /* ADDITIN        */
}

/* NEGUSUM .. LOUPE (pp.1483-1484): new drive NEGU = SGN(NEGUSUM); a
 * reversal first stops the drive (zero-drive pause); any change asks
 * PAXIS to run ACDT+C12 (CALLGMBL).                                     */
static agc_status_t gts_drive(lm_dap_t *d, int i, agc_sp_t newneg)
{
    agc_sp_t old = (i == 0) ? d->neguq : d->negur;

    if (i == 0)
    {
        d->neguq = newneg;
    }
    else
    {
        d->negur = newneg;
    }

    agc_sp_t prod = (agc_sp_t) (old * newneg);

    if (prod > 0)
    {
        return (AGC_STATUS_OK); /* LOUPE          */
    }

    if (prod < 0)
    { /* REVERSAL       */
        if (i == 0)
        {
            d->qaccdot = 0;
        }
        else
        {
            d->raccdot = 0;
        }

        agc_chan_and(
            012, (uint16_t) (~((i == 0) ? GT_GIMQ : GT_GIMR) & 077777u));
    }

    d->rcsflags = (uint16_t) (d->rcsflags | LM_RCS_CALLGMBL); /* ZEROLOUP  */

    if (!AGC_ASSERT(newneg >= -1 && newneg <= 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* GTS (pp.1472-1484): trim gimbal control of Q and R this pass. The next
 * pass is RCS (COTROLER = -1) and not a skip; the drive timers are set
 * to 200 ms against runaway. Ends the jask (CLOSEOUT).                  */
agc_status_t lm_dap_gts(void)
{
    lm_dap_t *d = lm_dap_state();

    d->cotroler = -1;
    d->skip[0]  = GT_SKIPOFF;
    d->skip[1]  = GT_SKIPOFF;
    d->ingts    = GT_TWO;
    d->qgimtimr = GT_TWO;
    d->rgimtimr = GT_TWO;
    /* DEVIATION: SAVESR (the editing shift register) is not modelled.   */
    agc_status_t st = AGC_STATUS_OK;

    for (int i = 1; i >= 0; i--)
    { /* R, then Q      */
        gts_w_t w = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, 0, 0, 0, 0};

        gts_inputs(d, i, &w);
        agc_sp_t newneg = 0;

        if (w.k != 0)
        { /* else POSDRIVE +1 */
            gts_first_terms(d, i, &w);
            gts_middle_terms(&w);
            gts_root_term(&w);
            newneg = dp_sign(w.k2theta);
        }

        agc_status_t s = gts_drive(d, i, newneg);

        st = (s != AGC_STATUS_OK) ? s : st;
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t sc = lm_dap_closeout();

    return ((st != AGC_STATUS_OK) ? st : sc);
}

/* TIMEGMBL, one axis (pp.1478-1480): drive time ABS(.4 AOS / ACCDOT) in
 * 0.1 s, drive direction NEGU = SGN(AOS). Zero ALPHA, ACCDOT or time:
 * drive off now. Over 2 s (or overflow): drive 2.5 s and withdraw GTS
 * permission (ALLOWGTS = 0) until 1/ACCS approves it again.             */
static void timegmbl_axis(lm_dap_t *d, int i)
{
    agc_sp_t accdot    = (i == 0) ? d->accdotq : d->accdotr; /* at pi/2^7    */
    agc_sp_t negaos    = (agc_sp_t) -((i == 0) ? d->aosq : d->aosr);
    agc_sp_t negu      = 0;
    agc_sp_t timer     = 0;
    bool     set_timer = false;

    if (accdot > 0 && negaos != 0)
    {
        int32_t v = (int32_t) lm_dap_mphi(negaos, GT_POINT6) +
                    negaos;                            /* -.4 AOS at pi/8 */

        negu = (agc_sp_t) ((v > 0) ? -1 : 1); /* SGN(AOS)       */
        agc_sp_t absa = (agc_sp_t) ((v > 0) ? v : -v);

        set_timer = true;

        if (lm_dap_ovf(v) ||
            (int32_t) lm_dap_mphi(accdot, (agc_sp_t) 2048) - absa <= 0)
        {
            timer       = GT_OCT31; /* NOTALLOW       */
            d->allowgts = 0;
        }
        else
        {
            timer = lm_dap_dv(lm_dap_mp(absa, GT_10_1024), accdot);

            if (timer == 0)
            {
                negu      = 0; /* TGOFFNOW       */
                set_timer = false;
            }
        }
    }

    if (i == 0)
    {
        d->neguq    = negu;
        d->qgimtimr = set_timer ? timer : d->qgimtimr;
    }
    else
    {
        d->negur    = negu;
        d->rgimtimr = set_timer ? timer : d->rgimtimr;
    }

    if (!AGC_ASSERT(negu >= -1 && negu <= 1))
    {
        return;
    }

    if (!AGC_ASSERT(timer >= 0))
    {
        return;
    }
}

/* TIMEGMBL (Craig Work, pp.1478-1480): null the offset acceleration with
 * a timed trim gimbal drive on each axis, then ACDT+C12. Called with
 * interrupts inhibited.                                                 */
agc_status_t agc_lm_dap_timegmbl(void)
{
    lm_dap_t *d = lm_dap_state();

    d->allowgts = 1;
    timegmbl_axis(d, 1); /* R axis first   */
    timegmbl_axis(d, 0);

    if (!AGC_ASSERT(d->allowgts == 0 || d->allowgts == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(d->neguq >= -1 && d->negur >= -1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (lm_dap_gimbal_drive_update());
}
