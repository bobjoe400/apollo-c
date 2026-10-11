/**********************************************************************
 * FILE NAME: tjetlaw.c
 *
 * PURPOSE:
 *   TJETLAW (TJET_LAW.agc, Luminary 099 pp.1460-1469), designed by R. D.
 *   Goss and P. S. Weissman: the jet-firing time for one axis (AXISCTR
 *   -1 = P, 0 = U, 1 = V) from the attitude error E and error rate EDOT,
 *   using the phase-plane switch curves that 1/ACCONT stores in the
 *   48-word BLOCKTOP table.
 *
 * AGC SOURCE:
 *   Luminary099 TJET_LAW.agc pp. 1460-1469
 *     (TJETLAW, FINELAW zones 1-5, FORMULA1-3, CHKMINTJ, RUFLAW1-3)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   lm_dap_ccs_dim              CCS diminished magnitude (arith.c)
 *   lm_dap_das                  DAS (arith.c)
 *   lm_dap_ddoubl               DDOUBL (arith.c)
 *   lm_dap_dpval                DP pair value (arith.c)
 *   lm_dap_dv                   DV quotient (arith.c)
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
 *   Negative EDOT is reflected into the upper half plane (E, EDOT negated,
 *   ROTSENSE negative, indexers moved by one). Small errors use FINELAW
 *   (zones 1-5); errors beyond pi/16 or rates beyond pi/32 rad/s use the
 *   rough laws RUFLAW1-3. The result, scaled at 4 s inside the law, is
 *   multiplied by ROTSENSE (+-.390625 = 4/10.24) to give TJP/TJU/TJV in
 *   TIME6 counts with the rotation sign.
 *
 *   Inputs : E (at pi), EDOT (at pi/4 rad/s), AXISCTR, SENSETYP, TJx.
 *   Outputs: TJx (T6 counts), NUMBERT (U,V jet policy), FIREFCT (at pi/4).
 **********************************************************************/
#include "../internal.h"
#include "agc/core/assert.h"

#define TJ_SENSOR   6400              /* SENSOR 14400: 4 s / 10.24 s               */
#define TJ_M3DEG    (-1092)           /* -3DEG  DEC -.06667 at 45 deg              */
#define TJ_M0112A8  (-23)             /* -.0112A8 DEC -.00141: -.01125 s^2 at 8    */
#define TJ_1AT4     410               /* .1AT4  DEC .025: 0.1 s at 4               */
#define TJ_1AT2     819               /* .1AT2  DEC .05: 0.1 s at 2                */
#define TJ_0375AT4  154               /* .0375AT4 DEC .00938                       */
#define TJ_025AT2   205               /* -(-.025AT2) DEC .0125: .025 s at 2        */
#define TJ_025AT4   102               /* -(-.025AT4) DEC .00625                    */
#define TJ_M05AT2   (-410)            /* -.05AT2 DEC -.025                         */
#define TJ_M15AT2   (-1229)           /* -.15AT2 DEC -.075                         */
#define TJ_00375HI  7                 /* .00375A8 2DEC .00375 B-3, major word      */
#define TJ_00375LO  11141             /*                          minor word       */
#define TJ_MTJMAX   (-614)            /* -TJMAX DEC -.0375: 150 ms at 4 s          */
#define TJ_TJMIN    82                /* TJMIN DEC .005: 20 ms at 4 s              */
#define TJ_RUFRATE  2366              /* RUFRATE DEC .1444: 6.5 deg/s at pi/4      */
#define TJ_FULLTIME 1024              /* BIT11: 250 ms at 4 s                      */
#define TJ_MINIMP   32                /* BIT6: one-jet minimum impulse (zone 3)    */
#define TJ_M1_8     (-2048)           /* -1/8                                      */
#define TJ_BIT13    ((agc_sp_t) 4096) /* .25: SCALEE pi -> pi/4; EDOT^2 to pi^2/2^8 */
#define TJ_BIT14    ((agc_sp_t) 8192) /* .5: MP BIT14 rescales TJET 2 s -> 4 s */
#define TJ_BIT11    ((agc_sp_t) 1024) /* 1/16: RUFLAW3 distance from pi/4 to 4 pi */
#define TJ_FLAT     6                 /* FLAT = BLOCKTOP +6                        */
#define TJ_ZONE3LIM 7                 /* ZONE3LIM = BLOCKTOP +7                    */

/* AXISDIFF (p.1469): BLOCKTOP offset of the axis block relative to U.    */
static const agc_sp_t axisdiff[3] = {-16, 0, 16};

/* TJETLAW working registers (DAPTEMP1-6, DAPTREG1-4).                    */
typedef struct tjl
{
    agc_sp_t     adr1;     /* ADRSDIF1: axis + sign-of-EDOT indexer (unscaled index) */
    agc_sp_t     adr2;     /* ADRSDIF2: ADRSDIF1 + max-jet / sense (unscaled index) */
    agc_sp_t     rotsense; /* ROTSENSE +-SENSOR: 4 s -> T6 scale and sign, scaled 10.24/4 */
    agc_sp_t     edotsq;   /* EDOTSQ at pi^2/2^8                         */
    agc_sp_t     ttoaxis;  /* TTOAXIS, scaled 4 s */
    lm_dp_pair_t hh;       /* HH, DP at 8 s^2                            */
} tjl_t;

/**********************************************************************
 * A BLOCKTOP parameter: base label offset (from 1/ANET1) + indexer.
 * AGC: none (helper)
 *
 * FUNCTION NAME: blk
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   label     int               I    BLOCKTOP label offset from 1/ANET1
 *                                    (LM_BK_*)
 *   idx       agc_sp_t          I    ADRSDIF indexer: axis and sense
 *
 * RETURNS: agc_sp_t -- the BLOCKTOP word
 **********************************************************************/
static agc_sp_t blk(const lm_dap_t *d, int label, agc_sp_t idx)
{
    int i = LM_BLK_U + label + (int) idx; /* BLOCKTOP index */

    if (!AGC_ASSERT(i >= 0 && i < LM_BLOCK_N))
    {
        return (0);
    }

    if (!AGC_ASSERT(d->block[i] >= LM_NEGMAX))
    {
        return (0);
    }

    return (d->block[i]);
}

/**********************************************************************
 * CHKMINTJ (p.1467): computed times under 20 ms become zero.
 *
 * FUNCTION NAME: chkmintj
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  --------------------
 *   a         int32_t  I    computed jet time, scaled 4 s
 *
 * RETURNS: agc_sp_t -- jet time, scaled 4 s (0 below TJMIN)
 **********************************************************************/
static agc_sp_t chkmintj(int32_t a)
{
    if (!AGC_ASSERT(a >= -3 * LM_POSMAX && a <= 3 * LM_POSMAX))
    {
        return (0); /* no valid result */
    }

    if (a - TJ_TJMIN <= 0)
    {
        return (0); /* COASTTJ        */
    }

    agc_sp_t r = lm_dap_wrap(a); /* CHKMINTJ result, scaled 4 s */

    if (!AGC_ASSERT(r > 0))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * Z123COMP (p.1465): reverse ROTSENSE, time to axis at 4 s. True if it
 * exceeds 150 ms (go to FULLTIME).
 *
 * FUNCTION NAME: zone_axis_time
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         tjl_t *           I/O  working cells of this computation
 *
 * RETURNS: bool -- true for full-time firing
 **********************************************************************/
static bool zone_axis_time(const lm_dap_t *d, tjl_t *w)
{
    w->rotsense = (agc_sp_t) -w->rotsense;
    w->ttoaxis  = lm_dap_mphi(d->edot, blk(d, LM_BK_1ANET1, w->adr2));

    if (!AGC_ASSERT(w->rotsense != 0))
    {
        return (false);
    }

    if (!AGC_ASSERT(w->ttoaxis >= LM_NEGMAX))
    {
        return (false);
    }

    return ((int32_t) w->ttoaxis + TJ_MTJMAX > 0);
}

/**********************************************************************
 * ZONE2,3 (p.1466): fire to the axis, or a minimum impulse when close.
 *
 * FUNCTION NAME: zone_fire_to_axis
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         const tjl_t *     I/O  working cells of this computation
 *
 * RETURNS: agc_sp_t -- jet time, scaled 4 s
 **********************************************************************/
static agc_sp_t zone_fire_to_axis(const lm_dap_t *d, const tjl_t *w)
{
    if (!AGC_ASSERT(d->block[TJ_ZONE3LIM] >= 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(d->edot >= 0))
    {
        return (0); /* no valid result */
    }

    if ((int32_t) w->ttoaxis - d->block[TJ_ZONE3LIM] > 0)
    {
        return (w->ttoaxis); /* ZONE2          */
    }

    return ((d->edot > 0) ? (agc_sp_t) TJ_MINIMP : 0); /* ZONE3          */
}

/**********************************************************************
 * ZONE1 (pp.1466-1467): beyond the switch curve. H = 4 (FIREFCT -
 * AXISDIST) ACCFCTZ1; TJET = H/.1 + TTOAXIS + .0375 or H/.025 + TTOAXIS.
 *
 * FUNCTION NAME: zone_beyond_switch
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         tjl_t *           I/O  working cells of this computation
 *
 * RETURNS: agc_sp_t -- jet time, scaled 4 s
 **********************************************************************/
static agc_sp_t zone_beyond_switch(const lm_dap_t *d, tjl_t *w)
{
    agc_sp_t     x = lm_dap_wrap((int32_t) d->firefct - blk(d, LM_BK_AXISDIST, w->adr1)); /* FIREFCT - AXISDIST (CAE FIREFCT, SU AXISDIST), scaled pi/4 rad */
    lm_dp_pair_t p = lm_dap_mp(x, blk(d, LM_BK_ACCFCTZ1, w->adr2));                       /* (..) x ACCFCTZ1: H before DDOUBL, DP */

    w->hh      = lm_dap_ddoubl(lm_dap_ddoubl(p));
    agc_sp_t t = lm_dap_wrap((int32_t) w->ttoaxis + TJ_MTJMAX); /* TTOAXIS - TJMAX, scaled 4 s */

    if ((int32_t) lm_dap_mphi(t, t) - w->hh.hi <= 0)
    {
        return (TJ_FULLTIME);
    }

    if (!AGC_ASSERT(w->hh.hi >= LM_NEGMAX))
    {
        return (0); /* no valid result */
    }

    if ((int32_t) w->hh.hi - 2 <= 0)
    { /* FORMULA3       */
        agc_sp_t q3 = lm_dap_dv(w->hh, TJ_025AT2); /* HH / .25 s: FORMULA3 quotient, scaled 4 s */

        return (chkmintj((int32_t) q3 + w->ttoaxis));
    }

    agc_sp_t q = lm_dap_dv(w->hh, TJ_1AT2); /* HH / .1 s: quotient, scaled 4 s */

    if (!AGC_ASSERT(q >= LM_NEGMAX))
    {
        return (0); /* no valid result */
    }

    return (lm_dap_wrap((int32_t) q + w->ttoaxis + TJ_0375AT4));
}

/**********************************************************************
 * ZONE5 (pp.1463-1465): between the coast and fire curves; TJET from
 * H = 4 f ACCFCTZ5 and TTOAXIS = 1/ANET2 EDOT by FORMULA1/2 or FULLTIME.
 *
 * FUNCTION NAME: zone_between_curves
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         tjl_t *           I/O  working cells of this computation
 *   f         agc_sp_t          I    distance below the coast curve, scaled
 *                                    pi/4
 *
 * RETURNS: agc_sp_t -- jet time, scaled 4 s
 **********************************************************************/
static agc_sp_t zone_between_curves(const lm_dap_t *d, tjl_t *w, agc_sp_t f)
{
    if (!AGC_ASSERT(w->rotsense != 0))
    { /* CCSHOLE        */
        return (0);
    }

    if (w->rotsense < 0)
    {
        w->adr2 = (agc_sp_t) (w->adr2 - 2);
    }

    w->hh      = lm_dap_ddoubl(lm_dap_ddoubl(lm_dap_mp(f, blk(d, LM_BK_ACCFCTZ5, w->adr2))));
    w->ttoaxis = lm_dap_mphi(blk(d, LM_BK_1ANET2, w->adr2), d->edot);
    int32_t t1 = (int32_t) lm_dap_mphi(w->ttoaxis, TJ_M05AT2) + w->hh.hi - 2; /* H - .05 TTOAXIS - .00125, scaled 8 s^2 */

    if (t1 <= 0)
    { /* FORMULA1       */
        agc_sp_t q1 = lm_dap_dv(w->hh, lm_dap_wrap((int32_t) w->ttoaxis + TJ_025AT4)); /* HH / (TTOAXIS - .025): FORMULA1, scaled 2 s */

        return (chkmintj(lm_dap_mphi(q1, TJ_BIT14)));
    }

    int32_t t2 = (int32_t) lm_dap_mphi(w->ttoaxis, TJ_M15AT2) + w->hh.hi + TJ_M0112A8; /* H - .15 TTOAXIS - .01125, scaled 8 s^2 */

    if (t2 > 0)
    {
        return (TJ_FULLTIME);
    }

    lm_dp_pair_t k = {TJ_00375HI, TJ_00375LO}; /* FORMULA2       */

    w->hh       = lm_dap_das(w->hh, k);
    agc_sp_t q2 = lm_dap_dv(w->hh, lm_dap_wrap((int32_t) w->ttoaxis + TJ_1AT4)); /* (HH + .00375) / (TTOAXIS + .1): FORMULA2, scaled 2 s */

    if (!AGC_ASSERT(q2 >= LM_NEGMAX))
    {
        return (0); /* no valid result */
    }

    return (lm_dap_mphi(q2, TJ_BIT14));
}

/**********************************************************************
 * ZONE4 / JETSON / DRIFT/ON (p.1463): the coast region, but jets already
 * firing toward the target stay on if the target (axis strip when
 * drifting, parabola when powered) can still be reached.
 *
 * FUNCTION NAME: zone_coast_jets_on
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         tjl_t *           I/O  working cells of this computation
 *
 * RETURNS: agc_sp_t -- jet time, scaled 4 s
 **********************************************************************/
static agc_sp_t zone_coast_jets_on(const lm_dap_t *d, tjl_t *w)
{
    agc_sp_t tj = d->tj[d->axisctr + 1]; /* TJ of the axis now, scaled 0.625 ms per count */

    if (lm_dap_mphi((agc_sp_t) -tj, w->rotsense) <= 0)
    {
        return (0); /* COASTTJ        */
    }

    if (!AGC_ASSERT(d->block[TJ_FLAT] >= 0))
    {
        return (0); /* no valid result */
    }

    if (d->block[TJ_FLAT] > 0)
    { /* DRIFT/ON       */
        int32_t s = -2 * (int32_t) blk(d, LM_BK_FIREDB, w->adr1) + d->firefct; /* FIREFCT - 2 FIREDB: target strip reachable, scaled pi/4 rad */

        if (s > 0)
        {
            return (0);
        }

        return (zone_axis_time(d, w) ? (agc_sp_t) TJ_FULLTIME : zone_fire_to_axis(d, w)); /* ZONE2,3 time, or FULLTIME */
    }

    if ((int32_t) blk(d, LM_BK_AXISDIST, w->adr1) - d->firefct <= 0)
    {
        return (0);
    }

    if (!AGC_ASSERT(w->rotsense != 0))
    {
        return (0); /* no valid result */
    }

    return (zone_axis_time(d, w) ? (agc_sp_t) TJ_FULLTIME : zone_beyond_switch(d, w)); /* ZONE1 time, or FULLTIME */
}

/**********************************************************************
 * FINELAW (pp.1462-1466): EDOTSQ, max-jet decision, FIREFCT and zones.
 *
 * FUNCTION NAME: finelaw
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         tjl_t *     I/O  working cells of this computation
 *
 * RETURNS: agc_sp_t -- jet time, scaled 4 s
 **********************************************************************/
static agc_sp_t finelaw(lm_dap_t *d, tjl_t *w)
{
    w->edotsq    = lm_dap_mphi(lm_dap_mphi(d->edot, d->edot), TJ_BIT13);
    int32_t  err = (d->e != 0) ? (int32_t) lm_dap_ccs_dim(d->e) + TJ_M3DEG : 0; /* ERRTEST */
    agc_sp_t n;                                                                 /* NUMBERT for TJCALC, unscaled code */

    if (err - blk(d, LM_BK_FIREDB, w->adr1) > 0 || d->sensetyp <= 0)
    {
        w->adr2 = (agc_sp_t) (w->adr2 + 2); /* MAXJETS        */
        n       = 4;
    }
    else
    {
        n = (agc_sp_t) (d->sensetyp - 1); /* SENSTEST       */
    }

    d->numbert = n;                                                                                                                  /* TJCALC         */
    int32_t f  = (int32_t) lm_dap_mphi((agc_sp_t) -w->edotsq, blk(d, LM_BK_1ANET1, w->adr2)) + blk(d, LM_BK_FIREDB, w->adr1) - d->e; /* -.5 EDOT^2/ACC + FIREDB - E in A, scaled pi/4 rad */

    d->firefct = lm_dap_sat(f); /* -E-.5EDOT^2/ACC-DB; TS FIREFCT overflow (and its skip) unreachable: the rough law takes large E, EDOT, so sat = wrap here */

    if (!AGC_ASSERT(d->numbert >= 0 && d->numbert <= 4))
    {
        return (0); /* no valid result */
    }

    if (d->firefct <= 0)
    { /* ZON1,2,3       */
        if (zone_axis_time(d, w))
        {
            return (TJ_FULLTIME);
        }

        if ((int32_t) d->firefct + d->block[TJ_FLAT] <= 0)
        {
            return (zone_beyond_switch(d, w));
        }

        return (zone_fire_to_axis(d, w));
    }

    int32_t g = (int32_t) lm_dap_mphi(blk(d, LM_BK_1ACOAST, w->adr1), w->edotsq) + d->e + blk(d, LM_BK_COASTDB, w->adr1); /* ZONE4,5        */

    if (!AGC_ASSERT(g >= -3 * LM_POSMAX && g <= 3 * LM_POSMAX))
    {
        return (0); /* no valid result */
    }

    if (g > 0)
    {
        return (zone_coast_jets_on(d, w)); /* ZONE4          */
    }

    /* ZONE5: TS L of an overflowed A would store the wrapped word and skip;
     * unreachable (rough law takes large E, EDOT), so sat = wrap here.     */
    return (zone_between_curves(d, w, lm_dap_sat(g)));
} /* finelaw */

/**********************************************************************
 * RUFSETUP (p.1469): optionally reverse ROTSENSE; require max jets.
 *
 * FUNCTION NAME: rufsetup
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         tjl_t *     I/O  working cells of this computation
 *   reverse   bool        I    true at RUFSETUP (reverse ROTSENSE), false at
 *                              +2
 *
 * RETURNS: none
 **********************************************************************/
static void rufsetup(lm_dap_t *d, tjl_t *w, bool reverse)
{
    if (reverse)
    {
        w->rotsense = (agc_sp_t) -w->rotsense;
    }

    d->numbert = 4;         /* 2 jets U,V     */
    d->firefct = LM_NEGMAX; /* 4 jets P       */

    if (!AGC_ASSERT(w->rotsense != 0))
    {
        return;
    }

    if (!AGC_ASSERT(d->numbert == 4))
    {
        return;
    }
}

/**********************************************************************
 * RUFLAW12 (p.1468): TJET = rate change / 2-jet acceleration, at 32 s,
 * FULLTIME beyond 4 s, then rescaled to 4 s (x 8). The three DAS A
 * double the whole A,L product, so carries from the minor word count.
 *
 * FUNCTION NAME: rough_law_jet_time
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         const tjl_t *     I/O  working cells of this computation
 *   a         agc_sp_t          I    desired rate change, scaled pi/4 rad/s
 *
 * RETURNS: agc_sp_t -- jet time, scaled 4 s
 **********************************************************************/
static agc_sp_t rough_law_jet_time(const lm_dap_t *d, const tjl_t *w, agc_sp_t a)
{
    if (!AGC_ASSERT(a >= 0))
    {
        return (0); /* RUFLAW12 gets the magnitude of the rate change */
    }

    lm_dp_pair_t p = lm_dap_mp(a, blk(d, LM_BK_1ANET1 + 2, w->adr2)); /* desired change x 1/ANET (2 jets): TJET, DP scaled 32 s */
    int32_t      x = (int32_t) p.hi + TJ_M1_8;                        /* TJET - 4 s, scaled 32 s */

    if (x >= 0)
    {
        return (TJ_FULLTIME);
    }

    int32_t t = (8 * lm_dap_dpval(p)) / 16384; /* DAS A x 3      */

    if (!AGC_ASSERT(t >= 0 && t <= LM_POSMAX))
    {
        return (0); /* no valid result */
    }

    return (chkmintj(t));
}

/**********************************************************************
 * RUFLAW1 / SMALRATE / RUFLAW2 (p.1468): |E| >= pi/16. Fire to (or
 * against) a 6.5 deg/s target rate.
 *
 * FUNCTION NAME: rough_law_large_error
 *
 * ARGUMENTS:
 *   Argument        Type        I/O  Description (scale)
 *   --------------  ----------  ---  --------------------
 *   d               lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w               tjl_t *     I/O  working cells of this computation
 *   negative_error  bool        I    E more negative than -pi/16 (RUFLAW1)
 *
 * RETURNS: agc_sp_t -- jet time, scaled 4 s
 **********************************************************************/
static agc_sp_t rough_law_large_error(lm_dap_t *d, tjl_t *w, bool negative_error)
{
    if (!AGC_ASSERT(d->edot >= 0))
    {
        return (0); /* NEGEDOT has reflected EDOT */
    }

    if (!negative_error)
    { /* RUFLAW2        */
        rufsetup(d, w, true);
        int32_t v = (int32_t) TJ_RUFRATE + d->edot; /* RUFRATE + EDOT in A, scaled pi/4 rad/s */

        return (lm_dap_ovf(v) ? (agc_sp_t) TJ_FULLTIME : rough_law_jet_time(d, w, (agc_sp_t) v)); /* TS A overflow skip: FULLTIME */
    }

    d->edot = lm_dap_wrap((int32_t) d->edot - TJ_RUFRATE); /* RUFLAW1: ADS   */

    if (d->edot > 0)
    {
        rufsetup(d, w, true);

        return (rough_law_jet_time(d, w, d->edot));
    }

    rufsetup(d, w, false); /* SMALRATE       */

    if (w->rotsense > 0)
    { /* CCS ROTSENSE: +1 to the 1/ANET of the proper sense */
        w->adr2 = (agc_sp_t) (w->adr2 + 1);
    }
    else if (w->rotsense < 0)
    {
        w->adr2 = (agc_sp_t) (w->adr2 - 1);
    }

    if (!AGC_ASSERT(d->edot <= 0))
    {
        return (0); /* no valid result */
    }

    return (rough_law_jet_time(d, w, (agc_sp_t) -d->edot));
}

/**********************************************************************
 * RUFLAW3 (p.1469): |EDOT| >= pi/32 rad/s, E at pi/4. Fire full time if
 * above the switch curve 1/ANET EDOT^2 + E - FIREDB = 0, else coast.
 *
 * FUNCTION NAME: rough_law_large_rate
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   w         tjl_t *     I/O  working cells of this computation
 *
 * RETURNS: agc_sp_t -- jet time: FULLTIME or 0, scaled 4 s
 **********************************************************************/
static agc_sp_t rough_law_large_rate(lm_dap_t *d, tjl_t *w)
{
    if (!AGC_ASSERT(d->edot > 0))
    {
        return (0); /* EDOT >= pi/32 after NEGEDOT */
    }

    rufsetup(d, w, true);
    agc_sp_t dist = lm_dap_wrap((int32_t) d->e - blk(d, LM_BK_FIREDB, w->adr1)); /* E - FIREDB, scaled pi/4 rad */
    agc_sp_t t    = lm_dap_mphi(dist, TJ_BIT11);                                 /* distance from the switch curve, scaled 4 pi rad */
    agc_sp_t sq   = lm_dap_mphi(d->edot, d->edot);                               /* EDOT^2, scaled pi^2/16 rad^2/s^2 */

    d->edot   = t;                                                                /* XCH EDOT       */
    int32_t s = (int32_t) lm_dap_mphi(sq, blk(d, LM_BK_1ANET1 + 2, w->adr1)) + t; /* 1/ANET1 EDOT^2 + E - FIREDB: above the curve, scaled 4 pi rad */

    if (!AGC_ASSERT(s >= -2 * LM_POSMAX && s <= 2 * LM_POSMAX))
    {
        return (0); /* no valid result */
    }

    return ((s <= 0) ? 0 : (agc_sp_t) TJ_FULLTIME); /* below the switch curve: coast */
}

/**********************************************************************
 * RETURNTJ (pp.1464-1465): apply ROTSENSE (sign, 4 s -> 10.24 s); call
 * for max jets if 1/ACCS forced them for this sense (ACCSWU, ACCSWV).
 *
 * FUNCTION NAME: returntj
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  --------------------
 *   d         lm_dap_t *     I/O  DAP erasables (lm_dap_state())
 *   w         const tjl_t *  I/O  working cells of this computation
 *   a         agc_sp_t       I    jet time, scaled 4 s
 *
 * RETURNS: none
 **********************************************************************/
static void returntj(lm_dap_t *d, const tjl_t *w, agc_sp_t a)
{
    int axis = d->axisctr; /* AXISCTR: -1 P, 0 U, 1 V */

    d->tj[axis + 1] = lm_dap_mphi(a, w->rotsense);

    /* DEVIATION: for the P axis the AGC indexes ACCSWU -1 (= AOSRTERM)
     * and may set NUMBERT = 4; PJETSLEC overwrites NUMBERT for P, so the
     * test is made for U and V only.                                     */
    if (axis >= 0 && lm_dap_mp(d->tj[axis + 1], d->block[axis]).lo > 0)
    {
        d->numbert = 4;
    }

    if (!AGC_ASSERT(axis >= -1 && axis <= 1))
    {
        return;
    }

    if (!AGC_ASSERT(d->numbert >= 0 && d->numbert <= 4))
    {
        return;
    }
}

/**********************************************************************
 * TJETLAW (pp.1461-1462): entry, half-plane reflection and law choice.
 *
 * FUNCTION NAME: lm_dap_tjetlaw
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_tjetlaw(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    if (!AGC_ASSERT(d->axisctr >= -1 && d->axisctr <= 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    tjl_t w = {axisdiff[d->axisctr + 1], 0, TJ_SENSOR, 0, 0, {0, 0}};

    if (d->edot <= 0)
    { /* NEGEDOT        */
        d->e       = (agc_sp_t) -d->e;
        d->edot    = (agc_sp_t) -d->edot;
        w.adr1     = (agc_sp_t) (w.adr1 + 1);
        w.rotsense = (agc_sp_t) -TJ_SENSOR;
    }

    w.adr2          = w.adr1;
    agc_sp_t     a;                                   /* jet time from the chosen law, scaled 4 s */
    lm_dp_pair_t pe = lm_dap_mp(d->e, (agc_sp_t) 16); /* MP BIT5        */

    if (pe.hi != 0)
    {
        a = rough_law_large_error(d, &w, pe.hi < 0); /* RUFLAW1 / 2    */
    }
    else
    {
        d->e            = lm_dap_mphi(pe.lo, TJ_BIT13);     /* SCALEE: at pi/4 */
        lm_dp_pair_t pd = lm_dap_mp(d->edot, (agc_sp_t) 8); /* MP BIT4       */

        if (pd.hi != 0)
        {
            a = rough_law_large_rate(d, &w);
        }
        else
        {
            d->edot = pd.lo; /* at pi/32       */
            a       = finelaw(d, &w);
        }
    }

    if (!AGC_ASSERT(w.rotsense == TJ_SENSOR || w.rotsense == -TJ_SENSOR))
    {
        return (AGC_STATUS_BAD_ARG); /* ROTSENSE is +-SENSOR */
    }

    returntj(d, &w, a);

    return (AGC_STATUS_OK);
} /* lm_dap_tjetlaw */
