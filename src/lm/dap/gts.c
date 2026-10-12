/**********************************************************************
 * FILE NAME: gts.c
 *
 * PURPOSE:
 *   The descent-engine trim gimbal control system
 *   (TRIM_GIMBAL_CONTROL_SYSTEM.agc, Luminary 099 pp.1472-1484): the GTS
 *   attitude/rate/acceleration control law, GTSQRT / ROOTCYCL (Craig
 *   Work), ACDT+C12 and TIMEGMBL.
 *
 * AGC SOURCE:
 *   Luminary099 TRIM_GIMBAL_CONTROL_SYSTEM.agc pp. 1472-1484
 *     (GTS, GTSQRT, ROOTCYCL, GTSGO+DN, ACDT+C12, TIMEGMBL, DRIVEON)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_chan_and                WAND channel
 *   agc_chan_read               READ channel
 *   agc_chan_write              WRITE channel
 *   lm_dap_closeout             CLOSEOUT / ENDJASK (qrjets.c)
 *   lm_dap_das                  DAS (arith.c)
 *   lm_dap_dv                   DV quotient (arith.c)
 *   lm_dap_dvqr                 DV quotient and remainder (arith.c)
 *   lm_dap_mp                   MP: A,L product (arith.c)
 *   lm_dap_mphi                 A after MP (arith.c)
 *   lm_dap_ovf                  TS overflow-skip test (arith.c)
 *   lm_dap_sat                  OVERSUB / LIMITS (arith.c)
 *   lm_dap_state                DAP erasables (state.c)
 *   lm_dap_wrap                 overflow-corrected store, TS/ADS/XCH (arith.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. AGC_STATUS_BAD_ARG when an internal assertion fails.
 *
 * NOTES:
 *   Drive-setting algorithm, per axis (Q, R), from the AGC comments:
 *       DEL     = SGN(OMEGA + ALPHA ABS(ALPHA) / (2K))
 *       NEGUSUM = ERROR K^2 + ALPHA (DEL OMEGA + ALPHA^2/(3K))
 *                 + DEL K^(1/2) (DEL OMEGA + ALPHA^2/(2K))^(3/2)
 *       DRIVE   = -SGN(NEGUSUM);  NEGU = SGN(NEGUSUM)
 *   ERROR (QDIFF = QERROR) at pi, OMEGA (EDOTQ) at pi/4 rad/s, ALPHA (AOS)
 *   at pi/4 rad/s^2 here, K (KQ, KRDAP) at pi/2^8 rad/s^3; NEGUSUM DP at
 *   4 pi^2.
 *
 *   The GTS temporaries overlay DAPTEMP1.. (KCENTRAL = SHFTFLAG, ACENTRAL =
 *   ININDEX, A2CNTRAL = SCRATCH/HALFARG); each is dead before its overlay
 *   is written, so they are separate locals here.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"
#include "internal.h"

#define GT_BGIM     007400u            /* BGIM: channel 12 bits 9-12              */
#define GT_GIMQ     001400u            /* GMBLBITA: bits 9,10 (Q drive)           */
#define GT_GIMR     006000u            /* GMBLBITB: bits 11,12 (R drive)          */
#define GT_BIT9     0000400u           /* -Q drive (NEGUQ < 0) */
#define GT_BIT10    0001000u           /* +Q drive (NEGUQ > 0) */
#define GT_BIT11    0002000u           /* -R drive (NEGUR < 0) */
#define GT_BIT12    0004000u           /* +R drive (NEGUR > 0) */
#define GT_NEG1_3   ((agc_sp_t) -5461) /* NEG1/3 DEC -.33333               */
#define GT_ROOTHALF ((agc_sp_t) 11585) /* ROOTHALF 26501: sqrt(1/2)        */
#define GT_POINT6   ((agc_sp_t) 9830)  /* OCT23146: decimal .6             */
#define GT_10_1024  ((agc_sp_t) 160)   /* OCT00240: decimal 10/1024        */
#define GT_OCT31    ((agc_sp_t) 25)    /* OCT31: 2.5 s at 0.1 s           */
#define GT_TWO      ((agc_sp_t) 2)     /* TWO: INGTS, gimbal timers (0.1 s counts) */
#define GT_SKIPOFF  ((agc_sp_t) 4)     /* SKIPU/V: skip 4 TJETLAW passes */

/* ACDT+C12: channel 12 drive bits for NEGU = -1, 0, +1 (Q: bits 9/10,
 * R: bits 11/12).                                                         */
static const uint16_t q_drive[3] = {GT_BIT9, 0u, GT_BIT10};
static const uint16_t r_drive[3] = {GT_BIT11, 0u, GT_BIT12};

/**********************************************************************
 * DP x SP as the GTS codes it (MP high, DXCH, MP low, ADS with carry).
 * AGC: none (helper)
 *
 * FUNCTION NAME: dp_x_sp
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   x         agc_dp_words_t  I    DP multiplicand
 *   k         agc_sp_t      I    SP multiplier
 *
 * RETURNS: agc_dp_words_t -- product, DP
 **********************************************************************/
static agc_dp_words_t dp_x_sp(agc_dp_words_t x, agc_sp_t k)
{
    agc_dp_words_t hi = lm_dap_mp(x.hi, k); /* x.hi k, same scale as x k */
    agc_dp_words_t lo = {0, lm_dap_mphi(x.lo, k)};
    agc_dp_words_t r  = lm_dap_das(hi, lo); /* x k with the minor product carried */

    return (r);
}

/**********************************************************************
 * DCS (p.1474): two-word complement.
 *
 * FUNCTION NAME: dp_neg
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   x         agc_dp_words_t  I    DP pair to complement (DCS)
 *
 * RETURNS: agc_dp_words_t -- -x
 **********************************************************************/
static agc_dp_words_t dp_neg(agc_dp_words_t x) /* DCS                  */
{
    agc_dp_words_t r = {(agc_sp_t) -x.hi, (agc_sp_t) -x.lo};

    return (r);
}

/**********************************************************************
 * Sign of a DP pair, high word first (CCS high, then CCS low).
 * AGC: none (helper)
 *
 * FUNCTION NAME: dp_sign
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   x         agc_dp_words_t  I    DP pair
 *
 * RETURNS: agc_sp_t -- -1, 0 or +1
 **********************************************************************/
static agc_sp_t dp_sign(agc_dp_words_t x)
{
    agc_sp_t w = (x.hi != 0) ? x.hi : x.lo; /* sign word: major, or minor when major is 0; unscaled (only its sign is used) */

    agc_sp_t sgn = 0; /* +-0: no sign (unscaled) */

    if (w > 0)
    {
        sgn = 1;
    }
    else if (w < 0)
    {
        sgn = -1;
    }

    return (sgn);
}

/**********************************************************************
 * ACDT+C12 (pp.1476-1477): Q(R)ACCDOT = -NEGU x ACCDOT(magnitude); the
 * drive bits of channel 12 from NEGUQ, NEGUR; clear the CALLGMBL request.
 * Called with interrupts inhibited.
 *
 * FUNCTION NAME: lm_dap_gimbal_drive_update
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_gimbal_drive_update(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->qaccdot    = lm_dap_mp((agc_sp_t) -d->neguq, d->accdotq).lo;
    d->raccdot    = lm_dap_mp((agc_sp_t) -d->negur, d->accdotr).lo;
    uint16_t bits = (uint16_t) (q_drive[d->neguq + 1] + r_drive[d->negur + 1]); /* channel 12 drive bits */

    uint16_t c12 = (uint16_t) (agc_chan_read(012) & (uint16_t) ~GT_BGIM & 077777u); /* channel 12 with the gimbal drive bits cleared */

    agc_chan_write(012, (uint16_t) (c12 + bits));
    d->rcsflags = (uint16_t) (d->rcsflags & ~LM_RCS_CALLGMBL);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ROOTCYCL (p.1484): one Newton iteration x' = x/2 + (ARG/2)/x.
 *
 * FUNCTION NAME: rootcycl
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   x         agc_sp_t  I    current root estimate X, scaled 1, > 0
 *   halfarg   agc_sp_t  I    ARG/2 (HALFARG), scaled 1
 *
 * RETURNS: agc_sp_t -- next estimate (X + ARG/X)/2, scaled 1
 **********************************************************************/
static agc_sp_t rootcycl(agc_sp_t x, agc_sp_t halfarg)
{
    agc_dp_words_t n = {halfarg, 0};
    agc_sp_t       q = lm_dap_dv(n, x); /* (ARG/X)/2, scaled 1 (fraction) */

    return (lm_dap_sat((int32_t) (x / 2) + q)); /* SR: X/2; guess >= root, so the sum cannot overflow (sat = wrap) */
}

/**********************************************************************
 * GTSQRT (pp.1481-1482): 14-bit square root of the DP *fn; the root must
 * be shifted right by SHFTFLAG + ININDEX/2. Zero or negative -> 0.
 *
 * FUNCTION NAME: lm_dap_gtsqrt
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------
 *   fn        agc_dp_words_t *  I/O  FUNCTION, DP argument; words swapped by
 *                                  ZEROHIGH
 *   shftflag  agc_sp_t *      O    SHFTFLAG: 7 when the major word was zero
 *   inindex   agc_sp_t *      O    ININDEX: twice the scaling shift
 *
 * RETURNS: agc_sp_t -- square root before the reported shift,
 *          scaled 1
 **********************************************************************/
agc_sp_t lm_dap_gtsqrt(agc_dp_words_t *fn, agc_sp_t *shftflag, agc_sp_t *inindex)
{
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

    *inindex   = 12;     /* TWELVE         */
    agc_sp_t a = fn->hi; /* SCALSTRT: the word to root, scaled 1 (fraction) */

    for (int i = 0; i < 6; i++)
    { /* SCALLOOP       */
        agc_sp_t ref = (agc_sp_t) (1 << (14 - *inindex)); /* reference 2^-ININDEX, scaled 1 (fraction) */

        if ((int32_t) ref - fn->hi > 0)
        { /* arg < reference */
            a = lm_dap_dv(*fn, ref); /* HALFARG        */
            break;
        }

        *inindex = (agc_sp_t) (*inindex - 2); /* MULBUSH        */

        if (*inindex <= 0)
        {
            break;
        }
    }

    agc_sp_t half = lm_dap_mphi(a, (agc_sp_t) 8192);                /* SCALDONE: HALFARG = ARG/2, scaled 1 (fraction) */
    agc_sp_t x    = ((half & 4096) != 0) ? LM_POSMAX : GT_ROOTHALF; /* first guess: 1 or sqrt(1/2); scaled 1 (fraction) */

    for (int k = 0; k < 3; k++)
    {
        x = rootcycl(x, half);
    }

    return (x);
} /* lm_dap_gtsqrt */

/* GTS working cells for one axis (GTSTEMPS).                             */
typedef struct gts_w
{
    agc_dp_words_t k2theta; /* K2THETA: NEGUSUM, DP at 4 pi^2           */
    agc_dp_words_t k2;      /* K2CNTRAL, DP at 16 pi                    */
    agc_dp_words_t a2;      /* A2CNTRAL, DP at 16 pi                    */
    agc_dp_words_t fn;      /* FUNCTION, DP                             */
    agc_sp_t       k;       /* KCENTRAL at pi/2^8                       */
    agc_sp_t       omega;   /* WCENTRAL = EDOT, scaled pi/4 rad/s */
    agc_sp_t       alpha;   /* ACENTRAL = AOS x 2, scaled pi/4 rad/s^2 */
    agc_sp_t       del;     /* DEL = SGN(K2CNTRAL): +1, 0, -1 (unscaled) */
} gts_w_t;

/**********************************************************************
 * GOQTRIMG .. GTSQAXIS (p.1473): AOS rescaled to pi/4 (limited), rate,
 * K of the axis (0 = Q, 1 = R).
 *
 * FUNCTION NAME: gts_inputs
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   i         int               I    axis: 0 = Q, 1 = R (QRCNTR / QRNDXER /
 *                                    2)
 *   w         gts_w_t *         I/O  working cells of this computation
 *
 * RETURNS: none
 **********************************************************************/
static void gts_inputs(const lm_dap_t *d, int i, gts_w_t *w)
{
    agc_sp_t       aos = (i == 0) ? d->aosq : d->aosr; /* AOSQ or AOSR, scaled pi/2 rad/s^2 */
    agc_dp_words_t p   = lm_dap_mp(aos, (agc_sp_t) 2); /* MP BIT2        */

    w->alpha = p.lo; /* AOS x 2 at pi/4 */

    if (p.hi != 0)
    { /* overflow: limit */
        w->alpha = (p.hi > 0) ? LM_POSMAX : LM_NEGMAX; /* POSMAX or NEGMAX */
    }

    w->omega = (i == 0) ? d->edotq : d->edotr; /* EDOTQ or EDOTR */
    w->k     = (i == 0) ? d->kq : d->krdap;    /* KQ or KRDAP */
}

/**********************************************************************
 * ALGORTHM (pp.1473-1474): K2THETA = K ERROR rescaled to 4 pi^2, and
 * A2CNTRAL = ALPHA^2/(2K) at 16 pi (POSMAX,POSMAX if K <= ALPHA^2/2).
 *
 * FUNCTION NAME: gts_first_terms
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   i         int               I    axis: 0 = Q, 1 = R (QRCNTR / QRNDXER /
 *                                    2)
 *   w         gts_w_t *         I/O  working cells of this computation
 *
 * RETURNS: none
 **********************************************************************/
static void gts_first_terms(const lm_dap_t *d, int i, gts_w_t *w)
{
    agc_sp_t       err  = (i == 0) ? d->qerror : d->rerror; /* QDIFF or RDIFF, scaled pi rad */
    agc_dp_words_t ke   = lm_dap_mp(err, w->k);             /* at pi^2/2^8    */
    agc_dp_words_t hi16 = lm_dap_mp(ke.hi, (agc_sp_t) 16);  /* MP BIT5        */

    w->k2theta.hi       = hi16.hi;
    w->k2theta.lo       = lm_dap_wrap((int32_t) hi16.lo + lm_dap_mphi(ke.lo, (agc_sp_t) 16));
    agc_sp_t       half = lm_dap_mphi((agc_sp_t) -w->alpha, (agc_sp_t) 8192); /* -ALPHA/2, scaled pi/4 rad/s^2 */
    agc_dp_words_t sq   = lm_dap_mp(half, w->alpha);                          /* -ALPHA^2/2     */

    if ((int32_t) sq.hi + w->k <= 0)
    { /* HUGEQUOT       */
        w->a2.hi = LM_POSMAX;
        w->a2.lo = LM_POSMAX;
    }
    else
    {
        agc_dp_words_t num = dp_neg(sq);             /* ALPHA^2/2      */
        agc_dp_words_t q1  = lm_dap_dvqr(num, w->k); /* (ALPHA^2/2)/K: quotient and remainder, DP at 16 pi */
        agc_dp_words_t rem = {q1.lo, 0};             /* LXCH 7         */

        w->a2.hi = q1.hi;
        w->a2.lo = lm_dap_dv(rem, w->k);
    }
}

/**********************************************************************
 * NEGUSUM .. LOUPE (pp.1483-1484): new drive NEGU = SGN(NEGUSUM); a
 * reversal first stops the drive (zero-drive pause); any change asks
 * PAXIS to run ACDT+C12 (CALLGMBL).
 *
 * FUNCTION NAME: gts_drive
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   i         int         I    axis: 0 = Q, 1 = R (QRCNTR / QRNDXER / 2)
 *   newneg    agc_sp_t    I    new NEGU = SGN(NEGUSUM): -1, 0, +1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t gts_drive(lm_dap_t *d, int i, agc_sp_t newneg)
{
    agc_sp_t old = (i == 0) ? d->neguq : d->negur; /* previous NEGU of the axis, unscaled sign */

    if (i == 0)
    {
        d->neguq = newneg;
    }
    else
    {
        d->negur = newneg;
    }

    agc_sp_t prod = (agc_sp_t) (old * newneg); /* sign of NEGU old x new (unscaled) */

    if (prod <= 0) /* LOUPE          */
    {
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

            agc_chan_and(012, (uint16_t) (~((i == 0) ? GT_GIMQ : GT_GIMR) & 077777u)); /* drive bits of this axis */
        }

        d->rcsflags = (uint16_t) (d->rcsflags | LM_RCS_CALLGMBL); /* ZEROLOUP  */
    }

    return (AGC_STATUS_OK);
} /* gts_drive */

/**********************************************************************
 * GTS (pp.1472-1484): trim gimbal control of Q and R this pass. The next
 * pass is RCS (COTROLER = -1) and not a skip; the drive timers are set
 * to 200 ms against runaway. Ends the jask (CLOSEOUT).
 *
 * FUNCTION NAME: lm_dap_gts
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_gts(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->cotroler = -1;
    d->skip[0]  = GT_SKIPOFF;
    d->skip[1]  = GT_SKIPOFF;
    d->ingts    = GT_TWO;
    d->qgimtimr = GT_TWO;
    d->rgimtimr = GT_TWO;

    /* DEVIATION: SAVESR (the editing shift register) is not modelled.   */
    agc_status_t st = AGC_STATUS_OK; /* first failing status of the two axes */

    for (int i = 1; i >= 0; i--)
    { /* R, then Q      */
        gts_w_t w = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, 0, 0, 0, 0};

        gts_inputs(d, i, &w);
        agc_sp_t newneg = 0; /* new NEGU, unscaled sign (0 when K = 0: BZF POSDRIVE +1 stores A = 0) */

        if (w.k != 0)
        { /* else POSDRIVE +1 */
            gts_first_terms(d, i, &w);

            /* HAVEQUOT .. FUNCT3 (pp.1474-1475): DEL, K2CNTRAL = DEL OMEGA +
             * ALPHA^2/(2K), and the ALPHA (DEL OMEGA + ALPHA^2/(3K)) term of
             * NEGUSUM. */
            w.k2             = lm_dap_mp(w.omega, (agc_sp_t) 256);                                      /* OMEGA at 16 pi */
            agc_dp_words_t f = (w.alpha > 0) ? lm_dap_das(w.k2, w.a2) : lm_dap_das(w.k2, dp_neg(w.a2)); /* DEL OMEGA + SGN(ALPHA) ALPHA^2/(2K), DP at 16 pi */

            w.del = dp_sign(f);

            if (w.del == 0)
            { /* DEFUNCT        */
                w.k2.hi = 0;
                w.k2.lo = 0;
            }
            else if (w.del < 0)
            { /* NEGFNCT2       */
                w.k2 = dp_neg(w.k2);
            }

            w.k2      = lm_dap_das(w.k2, w.a2);   /* FUNCT2         */
            w.a2      = dp_x_sp(w.a2, GT_NEG1_3); /* -ALPHA^2/(6K)  */
            w.a2      = lm_dap_das(w.a2, w.k2);
            w.k2theta = lm_dap_das(w.k2theta, dp_x_sp(w.a2, w.alpha));

            /* GETROOT .. ADDITIN (pp.1475-1483): add DEL K^(1/2) (K2CNTRAL)^(3/2)
             * using GTSQRT of K K2CNTRAL and the shift it reports. */
            w.fn = dp_x_sp(w.k2, w.k); /* at pi^2/16     */

            if (w.del != 0) /* NEGUSUM        */
            {
                if (w.del < 0)
                {
                    w.k2 = dp_neg(w.k2); /* NEGATE         */
                }

                agc_sp_t shft    = 0;                                     /* SHFTFLAG, unscaled shift count */
                agc_sp_t inindex = 0;                                     /* ININDEX, twice the scaling shift (unscaled) */
                agc_sp_t root    = lm_dap_gtsqrt(&w.fn, &shft, &inindex); /* sqrt(FUNCTION) before the shift, scaled 1 (fraction) */

                w.k2 = dp_x_sp(w.k2, root);                                       /* PRODUCT        */
                shft = (agc_sp_t) (shft + lm_dap_mphi(inindex, (agc_sp_t) 8192)); /* DOSHIFT */

                if (shft != 0)
                {
                    w.k2 = dp_x_sp(w.k2, (agc_sp_t) (1 << (14 - shft))); /* 2^-S     */
                }

                w.k2theta = lm_dap_das(w.k2theta, w.k2); /* ADDITIN        */
            }

            newneg = dp_sign(w.k2theta);
        }

        agc_status_t s = gts_drive(d, i, newneg); /* status of this axis */

        st = (s != AGC_STATUS_OK) ? s : st; /* a failing axis status replaces st */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_status_t sc = lm_dap_closeout(); /* CLOSEOUT status */

    return ((st != AGC_STATUS_OK) ? st : sc); /* first failure wins */
} /* lm_dap_gts */

/**********************************************************************
 * TIMEGMBL, one axis (pp.1478-1480): drive time ABS(.4 AOS / ACCDOT) in
 * 0.1 s, drive direction NEGU = SGN(AOS). Zero ALPHA, ACCDOT or time:
 * drive off now. Over 2 s (or overflow): drive 2.5 s and withdraw GTS
 * permission (ALLOWGTS = 0) until 1/ACCS approves it again.
 *
 * FUNCTION NAME: timegmbl_axis
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   i         int         I    axis: 0 = Q, 1 = R (QRCNTR / QRNDXER / 2)
 *
 * RETURNS: none
 **********************************************************************/
static void timegmbl_axis(lm_dap_t *d, int i)
{
    agc_sp_t accdot    = (i == 0) ? d->accdotq : d->accdotr;         /* at pi/2^7    */
    agc_sp_t negaos    = (agc_sp_t) -((i == 0) ? d->aosq : d->aosr); /* -AOSQ or -AOSR, scaled pi/2 rad/s^2 */
    agc_sp_t negu      = 0;                                          /* new NEGU, unscaled sign */
    agc_sp_t timer     = 0;                                          /* drive time, scaled 0.1 s per count */
    bool     set_timer = false;                                      /* timer word to be stored */

    if (accdot > 0 && negaos != 0)
    {
        int32_t v = (int32_t) lm_dap_mphi(negaos, GT_POINT6) + negaos; /* -.4 AOS at pi/8 */

        negu          = (agc_sp_t) ((v > 0) ? -1 : 1); /* SGN(AOS)       */
        agc_sp_t absa = (agc_sp_t) ((v > 0) ? v : -v); /* |.4 AOS|, scaled pi/8 rad/s^2 */

        set_timer = true;

        if (lm_dap_ovf(v) || (int32_t) lm_dap_mphi(accdot, (agc_sp_t) 2048) - absa <= 0)
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
        d->qgimtimr = set_timer ? timer : d->qgimtimr; /* keep the timer when the drive stops now */
    }
    else
    {
        d->negur    = negu;
        d->rgimtimr = set_timer ? timer : d->rgimtimr; /* keep the timer when the drive stops now */
    }
} /* timegmbl_axis */

/**********************************************************************
 * TIMEGMBL (Craig Work, pp.1478-1480): null the offset acceleration with
 * a timed trim gimbal drive on each axis, then ACDT+C12. Called with
 * interrupts inhibited.
 *
 * FUNCTION NAME: agc_lm_dap_timegmbl
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_timegmbl(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->allowgts = 1;
    timegmbl_axis(d, 1); /* R axis first   */
    timegmbl_axis(d, 0);

    return (lm_dap_gimbal_drive_update());
}
