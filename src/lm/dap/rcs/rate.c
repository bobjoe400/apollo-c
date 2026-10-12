/**********************************************************************
 * FILE NAME: rate.c
 *
 * PURPOSE:
 *   LM DAP state estimation: jet-torque feedback (KALMAN_FILTER.agc
 *   pp.1470-1471, RATELOOP/ROTORQUE), the rate derivation from CDU
 *   differences (P-AXIS_RCS_AUTOPILOT.agc pp.1422-1427, BACKP/SUBDIVDE)
 *   and the offset-acceleration estimator (p.1427, WORKTIME).
 *
 * AGC SOURCE:
 *   Luminary099 KALMAN_FILTER.agc pp. 1470-1471 (RATELOOP, ROTORQUE),
 *     P-AXIS_RCS_AUTOPILOT.agc pp. 1422-1427 (BACKP, SUBDIVDE, WORKTIME)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *   M11           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m11())
 *   M21           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m21())
 *   M22           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m22())
 *   M31           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m31())
 *   M32           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m32())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_flag_test               test a flag bit (flags.h)
 *   lm_dap_ccs_dim              CCS diminished magnitude (arith.c)
 *   lm_dap_das                  DAS (arith.c)
 *   lm_dap_dv                   DV quotient (arith.c)
 *   lm_dap_mp                   MP: A,L product (arith.c)
 *   lm_dap_mphi                 A after MP (arith.c)
 *   lm_dap_msu                  MSU (arith.c)
 *   lm_dap_sat                  OVERSUB / LIMITS (arith.c)
 *   lm_dap_state                DAP erasables (state.c)
 *   lm_dap_wrap                 overflow-corrected store, TS/ADS/XCH (arith.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. AGC_STATUS_BAD_ARG when an internal assertion fails.
 *
 * NOTES:
 *   OMEGA = OMEGA + JETRATE + AOSTERM (+ TRAPED/NTRAPS when TRAPED is big):
 *   the predicted rate is propagated with the commanded jet torque and the
 *   offset acceleration, and corrected by the accumulated difference between
 *   the CDU-measured and predicted rates once it exceeds the trap size.
 **********************************************************************/
#include "../internal.h"
#include "agc/core/assertion.h"

#define RT_100MST6  160               /* -100MST6 = -160: 100 ms in T6 counts */
#define RT_100MS    ((agc_sp_t) 1638) /* -100MS = DEC -.1 (74631)            */
#define RT_ELEVEN   ((agc_sp_t) 11)   /* ELEVEN: 10.24 "plus"                */
#define RT_BIT10    ((agc_sp_t) 512)  /* rescale torque to 32 jet-s          */
#define RT_1_40     ((agc_sp_t) 410)  /* 1/40 = DEC .025 (00632)             */
#define RT_BIT14    ((agc_sp_t) 8192) /* .5: MP BIT14 halves */
#define RT_OCT630   408               /* -OCT630 = 77147                     */
#define RT_FIVE     ((agc_sp_t) 5)    /* FIVE: AOS gain numerator, unscaled integer */
#define RT_CALLCODE ((agc_sp_t) 26)   /* 00032: .1 at 2^6                    */
#define RT_200MS    ((agc_sp_t) 3277) /* 200MS = DEC .2 (06315)              */

/**********************************************************************
 * RATELOOP + ROTORQUE (pp.1470-1471): JETRATEQ, JETRATER from the U,V
 * torques; BACKP then forms JETRATE for P. Rates at pi/4 rad/s.
 *
 * FUNCTION NAME: lm_dap_rateloop
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_rateloop(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    for (int i = 2; i >= 0; i--)
    { /* DAPTEMP6 = 2, 1, 0     */
        /* RATELOOP (p.1470): the torque each axis' jets applied during the last
         * 100 ms, signed, at 1 jet-second (DAPTEMP1..3), and the downlink torque
         * sums. Long firings are reduced by 100 ms; short ones are zeroed.       */
        agc_sp_t tj     = d->tj[i]; /* TJP/TJU/TJV, scaled 0.625 ms per count */
        agc_sp_t torque = 0;        /* jet-seconds this pass, scaled 1 jet-s */

        if (tj != 0)
        {
            if ((int32_t) lm_dap_ccs_dim(tj) - RT_100MST6 > 0)
            { /* > 100 ms */
                tj       = (agc_sp_t) ((tj > 0) ? tj - RT_100MST6 : tj + RT_100MST6); /* firing time left after this 100 ms */
                d->tj[i] = tj;
                torque   = (tj > 0) ? RT_100MS : (agc_sp_t) -RT_100MS;                /* 100 ms with the firing's sign */
            }
            else
            { /* SMALLTJU */
                d->tj[i] = 0;
                torque   = lm_dap_mp(tj, RT_ELEVEN).lo; /* CA L      */
            }
        }

        torque        = lm_dap_mp(torque, d->nojets[i]).lo; /* LOOPRATE: x NO.xJETS */
        d->torque[i]  = torque;
        agc_sp_t t32  = lm_dap_mphi(torque, RT_BIT10);      /* torque rescaled to 32 jet-s (~2 jet-ms per bit) */
        int      slot = 2 * i;                              /* DOWNTORK pair of the axis (index) */

        if (t32 <= 0)
        { /* NEGTORK               */
            t32 = (agc_sp_t) -t32;
            slot++;
        }

        d->downtork[slot] = lm_dap_wrap((int32_t) d->downtork[slot] + t32); /* overflows */
    } /* end DAPTEMP6 = 2, 1, 0 */

    int32_t usum = (int32_t) d->torque[1] + d->torque[2]; /* U + V torque, scaled 1 jet-s */
    int32_t udif = (int32_t) d->torque[1] - d->torque[2]; /* U - V torque, scaled 1 jet-s */

    d->jetrate[2] = lm_dap_mphi(lm_dap_wrap(usum), d->jacc[2]); /* JETRATER */
    d->jetrate[1] = lm_dap_mphi(lm_dap_wrap(udif), d->jacc[1]); /* JETRATEQ */

    return (AGC_STATUS_OK);
} /* lm_dap_rateloop */

/**********************************************************************
 * SUBDIVDE +3 (p.1422): measured angle increment (DP at pi, also left in
 * DAPTEMP5,6) divided by 1/40 to a rate at pi/4; POSMAX/NEGMAX (45 deg/s)
 * if the divide would overflow.
 *
 * FUNCTION NAME: measured_increment
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   om        agc_dp_words_t  I    CDU difference as a DP pair, scaled pi
 *
 * RETURNS: agc_sp_t -- increment major word, scaled pi
 **********************************************************************/
static agc_sp_t measured_increment(agc_dp_words_t om)
{
    if (om.hi != 0 && (int32_t) lm_dap_ccs_dim(om.hi) - RT_OCT630 > 0)
    {
        return ((om.hi > 0) ? LM_POSMAX : LM_NEGMAX); /* 45 deg/s with the sign of the difference */
    }

    agc_sp_t q = lm_dap_dv(om, RT_1_40); /* DIVIDER: rate = increment x 40, scaled pi/4 rad/s */

    return (q);
}

/**********************************************************************
 * One axis of the rate derivation (pp.1423-1425): TRAPED += measured -
 * predicted rate (limited), DxERROR += angle increment - commanded rate
 * x 0.1 s. The DP error is given as its two words.
 *
 * FUNCTION NAME: derive_axis
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   inc       agc_dp_words_t  I    measured angle increment, DP scaled pi
 *   omega     agc_sp_t      I    OMEGAx, rate estimate scaled pi/4 rad/s
 *   traped    agc_sp_t *    I/O  TRAPEDx, accumulated difference scaled pi
 *   err_hi    agc_sp_t *    I/O  DxERROR major word, scaled pi
 *   err_lo    agc_sp_t *    I/O  DxERROR minor word
 *   last      agc_sp_t      I    xLAST, commanded manual rate scaled 45 deg/s
 *
 * RETURNS: none
 **********************************************************************/
static void derive_axis(agc_dp_words_t inc, agc_sp_t omega, agc_sp_t *traped, agc_sp_t *err_hi, agc_sp_t *err_lo, agc_sp_t last)
{
    agc_sp_t rate = measured_increment(inc); /* CDU-measured rate, scaled pi/4 rad/s */

    *traped          = lm_dap_sat((int32_t) *traped + rate - omega);
    agc_dp_words_t e = {*err_hi, *err_lo};

    e       = lm_dap_das(e, inc);
    e       = lm_dap_das(e, lm_dap_mp((agc_sp_t) -last, RT_1_40)); /* manual mode */
    *err_hi = e.hi;
    *err_lo = e.lo;
}

/**********************************************************************
 * Trap correction for one axis (pp.1425-1427): when |TRAPED| exceeds the
 * trap size, OMEGA += TRAPED/NTRAPS and NTRAPS restarts at OMEGAN. Returns
 * the correction (DAPTEMP1/2) and the old count, or false if no trap.
 *
 * FUNCTION NAME: trap_axis
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   traped    agc_sp_t *  I/O  TRAPEDx, scaled pi
 *   ntraps    agc_sp_t *  I/O  NxTRAPS, count of passes since the last trap
 *   omega     agc_sp_t *  I/O  OMEGAx, scaled pi/4 rad/s
 *   omegan    agc_sp_t    I    LM/DKOMEGAN rate gain, scaled 1
 *   trap      agc_sp_t    I    LM/DKTRAP threshold, scaled pi
 *   corr      agc_sp_t *  O    rate correction, scaled pi/4 rad/s
 *   oldn      agc_sp_t *  O    NxTRAPS before the reset
 *
 * RETURNS: bool -- true when a trap correction was made
 **********************************************************************/
static bool trap_axis(agc_sp_t *traped, agc_sp_t *ntraps, agc_sp_t *omega, agc_sp_t omegan, agc_sp_t trap, agc_sp_t *corr, agc_sp_t *oldn)
{
    bool big = (*traped != 0) && ((int32_t) lm_dap_ccs_dim(*traped) + trap > 0); /* |TRAPED| beyond the trap size */

    if (big)
    {
        agc_dp_words_t n = {0, *traped};

        *corr   = lm_dap_dv(n, *ntraps);
        *traped = 0;
        *omega  = lm_dap_sat((int32_t) *omega + *corr);
        *oldn   = *ntraps;
        *ntraps = omegan;
    }

    *ntraps = lm_dap_wrap((int32_t) *ntraps + 1); /* INCR NxTRAPS     */

    return (big);
}

/**********************************************************************
 * Offset update from a trap correction (p.1426): AOS += 5 corr / (N+KAOS).
 *
 * FUNCTION NAME: aos_update
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   aos       agc_sp_t  I    AOSx major word, scaled pi/2 rad/s^2
 *   corr      agc_sp_t  I    rate correction from trap_axis, scaled pi/4
 *                            rad/s
 *   oldn      agc_sp_t  I    passes since the last trap
 *   kaosn     agc_sp_t  I    LM/DKKAOSN offset gain
 *
 * RETURNS: agc_sp_t -- new AOS major word, scaled pi/2 rad/s^2
 **********************************************************************/
static agc_sp_t aos_update(agc_sp_t aos, agc_sp_t corr, agc_sp_t oldn, agc_sp_t kaosn)
{
    agc_sp_t div = lm_dap_wrap((int32_t) oldn + kaosn); /* old NxTRAPS + KAOSN, unscaled count */

    agc_sp_t q = lm_dap_dv(lm_dap_mp(corr, RT_FIVE), div); /* 5 x correction / count: AOS change, scaled pi/2 rad/s^2 */

    return (lm_dap_wrap((int32_t) aos + q)); /* ADS AOSQ          */
}

/**********************************************************************
 * BACKP (pp.1423-1427): the whole rate derivation.
 *
 * FUNCTION NAME: lm_dap_backp
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_backp(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    /* BACKP (pp.1423-1425), first part: JETRATE, the jet terms of TRAPED and
     * the CDU-measured increments for P, Q, R.                               */
    d->jetrate[0] = lm_dap_mphi(d->torque[0], d->jacc[0]); /* JETRATE */
    agc_sp_t xdif = lm_dap_msu(d->dapcdu[0], d->oldxforp); /* CDUX - OLDXFORP, scaled pi rad */

    d->oldxforp = d->dapcdu[0];
    d->trapedp  = lm_dap_wrap((int32_t) d->trapedp + lm_dap_mphi((agc_sp_t) -d->jetrate[0], RT_BIT14));
    agc_sp_t jq = lm_dap_wrap((int32_t) d->jetrate[1] + d->aosqterm); /* JETRATEQ + AOSQTERM, scaled pi/4 rad/s */
    agc_sp_t jr = lm_dap_wrap((int32_t) d->jetrate[2] + d->aosrterm); /* JETRATER + AOSRTERM, scaled pi/4 rad/s */

    d->trapedq    = lm_dap_wrap((int32_t) d->trapedq + lm_dap_mphi(jq, (agc_sp_t) -RT_BIT14));
    d->trapedr    = lm_dap_wrap((int32_t) d->trapedr + lm_dap_mphi(jr, (agc_sp_t) -RT_BIT14));
    agc_sp_t ydif = lm_dap_msu(d->dapcdu[1], d->oldyforp); /* CDUY - OLDYFORP, scaled pi rad */

    d->oldyforp        = d->dapcdu[1];
    agc_dp_words_t inc = lm_dap_mp(ydif, agc_erase_m11()); /* M11 dCDUY (+ dCDUX below): P increment, DP scaled pi rad */

    inc.hi        = lm_dap_wrap((int32_t) inc.hi + xdif);  /* M13 = 1 */
    derive_axis(inc, d->omegap, &d->trapedp, &d->dxerror, &d->dxerror_lo, d->plast);
    agc_sp_t zdif = lm_dap_msu(d->dapcdu[2], d->oldzforq); /* CDUZ - OLDZFORQ, scaled pi rad */

    d->oldzforq = d->dapcdu[2];
    inc         = lm_dap_das(lm_dap_mp(agc_erase_m21(), ydif), lm_dap_mp(agc_erase_m22(), zdif));
    derive_axis(inc, d->omegaq, &d->trapedq, &d->qerror, &d->dyerror_lo, d->qlast);
    inc         = lm_dap_das(lm_dap_mp(agc_erase_m31(), ydif), lm_dap_mp(agc_erase_m32(), zdif));
    derive_axis(inc, d->omegar, &d->trapedr, &d->rerror, &d->dzerror_lo, d->rlast);

    bool     docked = agc_flag_test(AGC_FLAG_CSMDKFLG);                                            /* CSMDKFLG: docked gains */
    agc_sp_t omegan = docked ? d->dkomegan : d->lmomegan;                                          /* DAPTREG4: OMEGAN gain, unscaled count */
    agc_sp_t kaosn  = docked ? d->dkkaosn : d->lmkaosn;                                            /* DAPTREG5: KAOSN gain, unscaled count */
    agc_sp_t trap   = docked ? d->dktrap : d->lmtrap;                                              /* DAPTREG6: trap size, scaled 4.5 deg */
    agc_sp_t corr   = 0;                                                                           /* TRAPED/NTRAPS correction, scaled pi/4 rad/s */
    agc_sp_t oldn   = 0;                                                                           /* NxTRAPS before the reset, unscaled count */
    bool     ptrap  = trap_axis(&d->trapedp, &d->nptraps, &d->omegap, omegan, trap, &corr, &oldn); /* P trap corrected this pass */

    if (!AGC_ASSERT(!ptrap || d->trapedp == 0)) /* no P offset   */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    d->omegap = lm_dap_sat((int32_t) d->omegap + d->jetrate[0]); /* P-RATE */

    if (trap_axis(&d->trapedq, &d->nqtraps, &d->omegaq, omegan, trap, &corr, &oldn))
    {
        d->aosq = aos_update(d->aosq, corr, oldn, kaosn);
    }

    d->omegaq = lm_dap_sat((int32_t) d->omegaq + d->jetrate[1] + d->aosqterm);

    if (trap_axis(&d->trapedr, &d->nrtraps, &d->omegar, omegan, trap, &corr, &oldn))
    {
        d->aosr = aos_update(d->aosr, corr, oldn, kaosn);
    }

    d->omegar = lm_dap_sat((int32_t) d->omegar + d->jetrate[2] + d->aosrterm);

    return (AGC_STATUS_OK);
} /* lm_dap_backp */

/**********************************************************************
 * Offset estimator (p.1427). Drifting flight: zero the offsets (major
 * words only, as the AGC). Powered: AOS += .1 ACCDOT (gimbal jerk) each
 * pass; AOSTERM = .2 AOS (the 100 ms rate change at pi/4).
 *
 * FUNCTION NAME: lm_dap_offset_est
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_offset_est(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    if (agc_flag_test(AGC_FLAG_DRIFTDFL))
    {
        d->alphaq   = 0;
        d->alphar   = 0;
        d->aosqterm = 0;
        d->aosrterm = 0;
        d->aosq     = 0;
        d->aosr     = 0;

        return (AGC_STATUS_OK);
    }

    agc_dp_words_t q = {d->aosq, d->aosq_lo}; /* WORKTIME */

    q                = lm_dap_das(q, lm_dap_mp(d->qaccdot, RT_CALLCODE));
    d->aosq          = q.hi;
    d->aosq_lo       = q.lo;
    d->alphaq        = q.hi;
    d->aosqterm      = lm_dap_mphi(q.hi, RT_200MS);
    agc_dp_words_t r = {d->aosr, d->aosr_lo};

    r           = lm_dap_das(r, lm_dap_mp(d->raccdot, RT_CALLCODE));
    d->aosr     = r.hi;
    d->aosr_lo  = r.lo;
    d->alphar   = r.hi;
    d->aosrterm = lm_dap_mphi(r.hi, RT_200MS);

    if (!AGC_ASSERT(d->alphaq == d->aosq))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* lm_dap_offset_est */
