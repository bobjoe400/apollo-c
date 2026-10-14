/**********************************************************************
 * FILE NAME: accs.c
 *
 * PURPOSE:
 *   1/ACCS, the interface between the guidance programs and the DAP
 *   (AOSTASK_AND_AOSJOB.agc, Luminary 099 pp.1485-1496; written by Bob
 *   Covelli and Mike Houston, last modified Feb. 14 1969 by G. Kalan).
 *
 * AGC SOURCE:
 *   Luminary099 AOSTASK_AND_AOSJOB.agc pp. 1485-1496
 *     (1/ACCS, 1/ACCSET, 1/ACCFIX, 1/ACCJOB, MASSFIX, COMMEQS, DOCKED,
 *     SPSLOOP1/2, LM alone ascent/descent fits, GTS gimbal terms)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *   CSMMASS       agc_sp_t  I    B16 kg         CSM mass (agc_erase_csmmass())
 *   LEMMASS       agc_sp_t  I/O  B16 kg         LM mass (agc_erase_lemmass())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_chan_read               READ channel
 *   agc_exec_novac              NOVAC: schedule a job
 *   agc_flag_set                set a flag bit (flags.h)
 *   agc_flag_test               test a flag bit (flags.h)
 *   agc_guid_state              guidance erasables (MASS, ABDELV ...)
 *   agc_inhint                  INHINT
 *   agc_lm_dap_timegmbl         TIMEGMBL: trim-gimbal null drive (gts.c)
 *   agc_relint                  RELINT
 *   lm_dap_dv                   DV quotient (arith.c)
 *   lm_dap_dvovsub              DVOVSUB: limited divide (arith.c)
 *   lm_dap_dvqr                 DV quotient and remainder (arith.c)
 *   lm_dap_mp                   MP: A,L product (arith.c)
 *   lm_dap_mphi                 A after MP (arith.c)
 *   lm_dap_state                DAP erasables (state.c)
 *   lm_dap_switch_curves        1/ACCONT (accont.c)
 *   lm_dap_wrap                 overflow-corrected store, TS/ADS/XCH (arith.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. AGC_STATUS_BAD_ARG when an internal assertion fails;
 *   the status of NOVAC / 1/ACCONT is passed through.
 *
 * NOTES:
 *   Whenever the mass, deadband, configuration (ascent / descent / docked)
 *   or jet masks change, and at fresh start / restart, 1/ACCS computes the
 *   one-jet accelerations 1JACC, 1JACCQ, 1JACCR as functions of mass, the
 *   U,V resolution coefficients COEFFQ, COEFFR and 1JACCU, 1JACCV; in the
 *   descent case also the engine moment arm L,PVT-CG and the gimbal jerk
 *   terms ACCDOTQ/R, KQ, KRDAP, QACCDOT/RACCDOT; then 1/ACCONT (accont.c)
 *   repositions the TJETLAW switch curves and ACCSOKAY is set.
 *
 *   Scales: MASS DP B16 kg (high word used), LEMMASS / CSMMASS B16 kg,
 *   ABDELV B13 cm/s^2, 1JACC.. pi/4 rad/s^2, 1JACCU,V pi/2 rad/s^2,
 *   L,PVT-CG 8 ft, ACCDOTQ/R pi/2^7 rad/s^3, KQ / KRDAP pi/2^8 rad/s^3.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"
#include "internal.h"

#define AC_LOASCENT ((agc_sp_t) 550)   /* DEC 2200 B-16: min ascent LM kg  */
#define AC_HIDESCNT ((agc_sp_t) 3825)  /* DEC 15300 B-16: max descent LM   */
#define AC_LODESCNT ((agc_sp_t) 438)   /* DEC 1750 B-16: min descent stage */
#define AC_BIT10    ((agc_sp_t) 512)   /* 1.4 deg/s at 45: 4-jet P limit   */
#define AC_GFACTM   ((agc_sp_t) 0337)  /* 979.24/2.20462 at B+15           */
#define AC_TORKJET1 ((agc_sp_t) 616)   /* DEC .03757: 550/.2 at 2^16 64/180 */
#define AC_DGBF     ((agc_sp_t) 9830)  /* DEC 0.6: .3 scaled at 1/2        */
#define AC_035356   ((agc_sp_t) 5793)  /* DEC 0.35356: .70711 at 2         */
#define AC_07071    ((agc_sp_t) 11585) /* DEC .70711                       */
#define AC_EPSMAX   ((agc_sp_t) 6925)  /* -(-EPSMAX) DEC .42265            */
#define AC_BIT14    ((agc_sp_t) 8192)  /* .5: MP BIT14 halves (pi/4 -> pi/2 scale) */
#define AC_1JACCCON ((agc_sp_t) 0167)  /* pi/4 x 2^16 rad/s^2 kg           */
#define AC_TORQ_LO  ((agc_sp_t) 8428)  /* TORQCONS 2DEC .51443 B-14: 500 lb-ft */
#define AC_PRIO27   ((agc_prio_t) 027) /* 1/ACCJOB priority 27 */

/* Inertia fit tables (pp.1493-1494), row k = 0 L,PVT-CG descent,
 * 1..3 P,Q,R descent, 4..6 P,Q,R ascent. 1JACC(k) = A / (LEMMASS + C) + B.
 * A is 2DEC (major, minor word); B, C are DEC.                            */
static const agc_sp_t inercona_hi[7] = {672, 97, 24, 17, 107, 58, 93};
static const agc_sp_t inercona_lo[7] = {9547, 3854, 8881, 2038, 3657, 10307, 4937};

/* .0410511917 .0059347674 .0014979264 .0010451889 .0065443852 .0035784354 .0056946631 */
static const agc_sp_t inerconb[7] = {2540, 49, 308, 350, 1, 2668, 153};

/* .155044 .002989 .018791 .021345 .000032 .162862 .009312                 */
static const agc_sp_t inerconc[7] = {-413, 143, -1117, -1082, -113, 42, -387};

/* -.025233 .008721 -.068163 -.066027 -.006923 .002588 -.023608           */

/* COEFF (p.1496): docked inertia (0..5) and CG (6..11) fits, in the order
 * C, F, B, A, E, D. X = CSMMASS, Y = LEMMASS:
 *   value = C X Y + F + (A X + D) X + (B Y + E) Y.                        */
static const agc_sp_t coeff[12] = {
    3198, -87, -2895,
    -608, 1143, 421,    /* .19518 -.00529 -.17670 -.03709 .06974 .02569 */
    3293, 2222, 12403,
    -6085, -10341, 6747 /* .20096 .13564 .75704 -.37142 -.63117 .41179  */
};

/* GIMBLBTS (p.1494): channel 12 trim gimbal drive bits, Q then R:
 * any motion, positive rotation.                                          */
static const uint16_t gimblbts[4] = {001400u, 001000u, 006000u, 004000u};

/**********************************************************************
 * AGC: none (helper) -- major word of MASS (1/ACCS reads it).
 *
 * FUNCTION NAME: mass_hi
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- MASS major word, scaled 2^16 kg
 **********************************************************************/
static agc_sp_t mass_hi(void)
{
    agc_dp_t m = agc_erase_mass(); /* MASS (DP), scaled 2^16 kg */

    return ((agc_sp_t) (m / 16384)); /* major word of MASS          */
}

/**********************************************************************
 * 1/ACCS entry .. MASSFIX (pp.1486-1487): LEMMASS from MASS, kept inside
 * the ascent / descent limits (which catches staging); a violated limit
 * also corrects MASS (low word zeroed). Sets -2JETLIM. Returns descent.
 *
 * FUNCTION NAME: lem_mass
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   docked    bool        I    CSMDOCKD: subtract CSMMASS from MASS
 *
 * RETURNS: bool -- true in descent (APSFLAG clear)
 **********************************************************************/
static bool lem_mass(lm_dap_t *d, bool docked)
{
    agc_sp_t csm    = docked ? agc_erase_csmmass() : 0;                              /* CSMMASS when docked, scaled 2^16 kg */
    agc_sp_t lem    = (agc_sp_t) (mass_hi() - csm);                                  /* LEMMASS candidate, scaled 2^16 kg */
    bool     ascent = agc_flag_test(AGC_FLAG_APSFLAG);                               /* APSFLAG: ascent stage */
    agc_sp_t hi     = ascent ? d->hiascent : AC_HIDESCNT;                            /* upper LEMMASS limit, scaled 2^16 kg */
    agc_sp_t lo     = ascent ? AC_LOASCENT : (agc_sp_t) (AC_LODESCNT + d->hiascent); /* lower LEMMASS limit, scaled 2^16 kg */

    d->neg_2jetlim = ascent ? LM_NEGMAX : (agc_sp_t) -AC_BIT10; /* ascent: no 4-jet limit; descent: 1.4 deg/s */

    if ((int32_t) hi - lem <= 0)
    {
        lem = hi; /* MASSFIX: high  */
    }
    else if ((int32_t) lo - lem > 0)
    {
        lem = lo; /* MASSFIX: low   */
    }
    else
    {
        agc_erase_set_lemmass(lem); /* F(MASS)        */

        return (!ascent);
    }

    agc_erase_set_lemmass(lem);
    agc_erase_set_mass((agc_dp_t) ((int32_t) (csm + lem) * 16384));

    return (!ascent);
}

/**********************************************************************
 * STCTR1 (p.1488): one row of the inertia fit; the DV remainder is left
 * in *rem (the AGC's L, read again by COMMEQS).
 *
 * FUNCTION NAME: fit_row
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   k         int         I    row of INERCONA..C: 0 L,PVT-CG, 1-3 descent
 *                              P,Q,R, 4-6 ascent
 *   rem       agc_sp_t *  O    DV remainder (the AGC L register)
 *
 * RETURNS: agc_sp_t -- 1JACCx (scaled pi/4 rad/s^2) or L,PVT-CG
 *          (scaled 8 ft)
 **********************************************************************/
static agc_sp_t fit_row(int k, agc_sp_t *rem)
{
    agc_dp_words_t a   = {inercona_hi[k], inercona_lo[k]};
    agc_sp_t       div = (agc_sp_t) (agc_erase_lemmass() + inerconc[k]); /* LEMMASS + C, scaled 2^16 kg */
    agc_dp_words_t q   = lm_dap_dvqr(a, div);                            /* (A,L) / (LEMMASS + C): quotient and remainder, scale per row */

    *rem = q.lo;

    return (lm_dap_wrap((int32_t) q.hi + inerconb[k])); /* TS 1JACC */
}

/**********************************************************************
 * SPSCONT .. DOCKTEST (pp.1490-1491): gimbal gains KQ, KRDAP = .3 ACCDOT,
 * the signed jerk terms from the channel 12 gimbal drive bits, and the
 * GTS permission (TIMEGMBL when the DAP runs, GTS is usable and not in
 * control). The docked case then bypasses 1/ACCONT.
 *
 * FUNCTION NAME: spscont
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   accdotq   agc_sp_t    I    ACCDOTQ, scaled pi/2^7 rad/s^3
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t spscont(lm_dap_t *d, agc_sp_t accdotq)
{
    d->accdotq   = accdotq;
    d->kq        = lm_dap_mphi(d->accdotq, AC_DGBF);
    d->krdap     = lm_dap_mphi(d->accdotr, AC_DGBF);
    uint16_t c12 = agc_chan_read(012); /* channel 12: trim gimbal drive bits */

    for (int i = 1; i >= 0; i--)
    { /* R, then Q      */
        agc_sp_t acc = (i == 0) ? d->accdotq : d->accdotr; /* ACCDOTQ or ACCDOTR, scaled pi/2^7 rad/s^3 */
        agc_sp_t j   = 0;                                  /* ZACCDOT: gimbal not driving, scaled pi/2^7 rad/s^3 */

        if ((c12 & gimblbts[2 * i]) != 0u)
        {
            j = ((c12 & gimblbts[2 * i + 1]) != 0u) ? (agc_sp_t) -acc : acc; /* sign of the drive direction */
        }

        if (i == 0)
        {
            d->qaccdot = j;
        }
        else
        {
            d->raccdot = j;
        }
    }

    if (agc_flag_test(AGC_FLAG_USEQRFLG) || d->t5adr != LM_T5_PAXIS)
    {
        d->allowgts = 0; /* DOWNGTS        */
        d->ingts    = 0;
    }
    else if (d->ingts <= 0)
    {
        agc_lm_dap_timegmbl(); /* null offset    */
    }

    if (!AGC_ASSERT(d->kq >= 0 || d->accdotq < 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* spscont */

/**********************************************************************
 * SPSLOOP1/2 (pp.1494-1495): the docked quadratic fit in X = CSMMASS,
 * Y = LEMMASS starting at COEFF row `base` (0 inertia, 6 CG).
 *
 * FUNCTION NAME: docked_fit
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  --------------------
 *   base      int   I    first COEFF row: 0 inertia, 6 CG
 *
 * RETURNS: agc_sp_t -- inertia (scaled 2^38 kg cm^2) or CG x
 *          DELDOT (scaled 4 pi rad cm/s)
 **********************************************************************/
static agc_sp_t docked_fit(int base)
{
    agc_sp_t        x  = agc_erase_csmmass();                                                /* X = CSMMASS, scaled 2^16 kg */
    agc_sp_t        y  = agc_erase_lemmass();                                                /* Y = LEMMASS, scaled 2^16 kg */
    const agc_sp_t *c  = &coeff[base];                                                       /* this fit's six coefficients (C F A B D E) */
    agc_sp_t        m  = lm_dap_wrap((int32_t) lm_dap_mphi(lm_dap_mphi(c[0], y), x) + c[1]); /* MPAC = C X Y + F, scale per fit (inertia 2^38 kg cm^2 or CG) */
    agc_sp_t        lx = lm_dap_wrap((int32_t) lm_dap_mphi(c[3], x) + c[5]);                 /* A X + D, scale per fit */

    m           = lm_dap_wrap((int32_t) m + lm_dap_mphi(lx, x));      /* ADS     */
    agc_sp_t ly = lm_dap_wrap((int32_t) lm_dap_mphi(c[2], y) + c[4]); /* B Y + E, scale per fit */

    m = lm_dap_wrap((int32_t) m + lm_dap_mphi(ly, y));

    return (m);
}

/**********************************************************************
 * DOCKED (pp.1494-1496): CSM-docked jet accelerations. Inertia at
 * 2^38 kg cm^2, CG x DELDOT at 4 pi rad-cm/s; 1JACC = 1JACCCON/MASS;
 * 1JACCQ = 1JACCR = TORQCONS / inertia; U,V axes made orthogonal; the
 * inverse net accelerations set to POSMAX (1.4 deg/s^2).
 *
 * FUNCTION NAME: docked
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t docked(lm_dap_t *d)
{
    /* The inertia pass ends with DIM COEFCTR taking 1 to -0 (ones'
     * complement 1 + -1), so CCS COEFCTR takes its fourth branch over
     * the TORQCONS words to CA MPAC (saves the inertia in MPAC +1); the
     * CG pass leaves COEFCTR = 6 and exits. Traced in the rope.          */
    agc_sp_t       inertia = docked_fit(0); /* docked inertia, scaled 2^38 kg cm^2 */
    agc_sp_t       cg      = docked_fit(6); /* CG x DELDOT, scaled 4 pi rad cm/s */
    agc_dp_words_t con     = {AC_1JACCCON, 0};

    d->jacc[0]              = lm_dap_dvovsub(con, mass_hi());
    d->block[2]             = LM_POSMAX; /* 1/ANETP        */
    d->block[LM_BLK_U + 2]  = LM_POSMAX; /* 1/ANET2 +1, +2 */
    d->block[LM_BLK_U + 3]  = LM_POSMAX;
    d->block[LM_BLK_U + 18] = LM_POSMAX; /* +17D, +18D     */
    d->block[LM_BLK_U + 19] = LM_POSMAX;
    agc_dp_words_t torq     = {0, AC_TORQ_LO};

    if (!AGC_ASSERT(inertia > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    d->jacc[1] = lm_dap_dv(torq, inertia);
    d->jacc[2] = d->jacc[1];
    d->coeffq  = (agc_sp_t) -AC_07071;
    d->coeffr  = AC_07071;

    agc_sp_t mc = lm_dap_mphi(mass_hi(), cg); /* MASS x CG DELDOT, scale per the DOCKED fit (MPAC) */

    d->accdotr = lm_dap_dvovsub(lm_dap_mp(mc, agc_erase_abdelv()), inertia);

    return (spscont(d, d->accdotr)); /* TCF SPSCONT    */
}

/**********************************************************************
 * F(MASS) .. COMMEQS for the LM alone (pp.1487-1489): the jet
 * accelerations, then (descent) L,PVT-CG and LRESC, or (ascent) GTS off.
 *
 * FUNCTION NAME: lm_alone
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   descent   bool        I    true for the descent coefficient set
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t lm_alone(lm_dap_t *d, bool descent)
{
    agc_sp_t rem  = 0;               /* remainder L of the last fit row, scaled 2^-14 of the row */
    int      base = descent ? 1 : 4; /* first COEFF row: 1 descent, 4 ascent (index) */

    if (!AGC_ASSERT(agc_erase_lemmass() > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int j = 2; j >= 0; j--)
    { /* R, Q, P        */
        d->jacc[j] = fit_row(base + j, &rem);
    }

    /* COMMEQS (pp.1488-1489): coupling EPSILON = 1 - Imin/Imax, limited to
     * .42265, and the U,V resolution coefficients
     *   IR > IQ: COEFFR = .707(1 + .5 EPS), COEFFQ = -(1 - EPS) COEFFR
     *   IQ > IR: COEFFQ = -.707(1 + .5 EPS), COEFFR = (1 - EPS)(-COEFFQ). */
    agc_dp_words_t diff = {lm_dap_wrap((int32_t) d->jacc[1] - d->jacc[2]), rem};

    if (diff.hi > 0)
    {
        agc_sp_t eps = lm_dap_dv(diff, d->jacc[1]); /* EPSILON = 1 - IQ/IR, unscaled fraction */

        eps       = ((int32_t) eps - AC_EPSMAX > 0) ? AC_EPSMAX : eps; /* GOODEPS1 */
        d->coeffr = lm_dap_wrap((int32_t) lm_dap_mphi(eps, AC_035356) + AC_07071);
        d->coeffq = lm_dap_mphi(lm_dap_wrap((int32_t) eps - LM_POSMAX), d->coeffr);
    }
    else
    { /* BIGIQ    */
        agc_sp_t neps = lm_dap_dv(diff, d->jacc[2]); /* -EPSILON = IR/IQ - 1, unscaled fraction */

        neps      = ((int32_t) -neps - AC_EPSMAX > 0) ? (agc_sp_t) -AC_EPSMAX : neps; /* limited to -.42265 */
        d->coeffq = lm_dap_wrap((int32_t) lm_dap_mphi(neps, AC_035356) - AC_07071);
        d->coeffr = lm_dap_mphi(lm_dap_wrap((int32_t) -neps + LM_NEGMAX), d->coeffq);
    }

    /* JACCUV (p.1489): 1JACCU = 1JACCV = -COEFFQ 1JACCQ + COEFFR 1JACCR,
     * rescaled from pi/4 to pi/2. */
    agc_sp_t u = lm_dap_mphi((agc_sp_t) -d->coeffq, d->jacc[1]);                /* -COEFFQ 1JACCQ, scaled pi/4 rad/s^2 */
    agc_sp_t a = lm_dap_wrap((int32_t) lm_dap_mphi(d->coeffr, d->jacc[2]) + u); /* 1JACCU before halving, scaled pi/4 rad/s^2 */

    d->jacc[3] = lm_dap_mphi(a, AC_BIT14);
    d->jacc[4] = d->jacc[3];

    /* DEVIATION: the AGC tells descent from ascent here by CCS on the
     * index MPAC, which counts down 6, 4, 2 to -0 (ones'-complement
     * 2 + -2) in descent and stays positive in ascent.                  */
    if (!descent)
    {
        d->allowgts = 0; /* ascent         */
        d->ingts    = 0; /* 1/ACCONT -1    */

        return (AGC_STATUS_OK);
    }

    /* DEVIATION: with a LEMMASS far below the descent range (only the
     * zero erasable before the pad load) this row overflows, and TS
     * 1JACC -1 skips back into STCTR with the index running below the
     * tables; the AGC's values are then meaningless anyway (a later
     * 1/ACCS with real masses replaces them). The C stores the row.     */
    d->l_pvt_cg = fit_row(0, &rem);

    /* LRESC (pp.1489-1490): descent jerk ACCDOTx = (ABDELV MASS / GFACTM)
     * L,PVT-CG 1JACCx / TORKJET1, i.e. engine thrust x moment arm divided by
     * the jet torque, times the jet acceleration. */
    agc_sp_t abdelv = agc_erase_abdelv();                                      /* ABDELV, scaled 2^13 cm/s^2 */
    agc_sp_t f      = lm_dap_dvovsub(lm_dap_mp(abdelv, mass_hi()), AC_GFACTM); /* thrust T = ABDELV MASS / GFACTM, scaled 2^14 lb */
    agc_sp_t t      = lm_dap_mphi(f, d->l_pvt_cg);                             /* torque T x L,PVT-CG, scaled 2^17 lb-ft */

    d->accdotr = lm_dap_dvovsub(lm_dap_mp(t, d->jacc[2]), AC_TORKJET1);
    agc_sp_t q = lm_dap_dvovsub(lm_dap_mp(t, d->jacc[1]), AC_TORKJET1); /* ACCDOTQ, scaled pi/2^7 rad/s^3 */

    if (!AGC_ASSERT(abdelv >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (spscont(d, q));
} /* lm_alone */

/**********************************************************************
 * 1/ACCS (pp.1486-1491), called by BANKCALL or from the NOVAC jobs.
 *
 * FUNCTION NAME: agc_lm_dap_jet_accel_update
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_jet_accel_update(void)
{
    lm_dap_t *d    = lm_dap_state();                   /* DAP erasables */
    bool      dock = agc_flag_test(AGC_FLAG_CSMDKFLG); /* DOCKTEMP       */

    agc_inhint();
    bool descent = lem_mass(d, dock); /* APSFLAG clear */

    agc_relint();
    agc_status_t st = dock ? docked(d) : lm_alone(d, descent); /* DOCKED or LM-alone fits */

    if (!dock)
    {
        agc_status_t sc = lm_dap_switch_curves(); /* 1/ACCONT       */

        st = (st != AGC_STATUS_OK) ? st : sc; /* first failure wins */
    }

    agc_inhint();                    /* 1/ACCRET       */
    agc_flag_set(AGC_FLAG_ACCOKFLG); /* data good      */
    agc_relint();

    if (!AGC_ASSERT(d->jacc[0] >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * 1/ACCJOB (p.1486): NOVAC job form of 1/ACCS (CALLACCS, RCSMONIT).
 *
 * FUNCTION NAME: agc_job_jet_accel_update
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  --------------------
 *   ctx       agc_job_ctx_t *  I    job context from the executive
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_job_jet_accel_update(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id == AGC_JOB_JET_ACCEL_UPDATE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_dap_jet_accel_update()); /* then ENDOFJOB  */
}

/**********************************************************************
 * 1/ACCSET (p.1486): entry from fresh start / restart (DAPIDLER): null
 * the offset estimates for 1/ACCS and the downlist, then 1/ACCJOB. Only
 * the major words of AOSQ, AOSR are zeroed (TS AOSQ, TS AOSR).
 *
 * FUNCTION NAME: agc_job_jet_accel_reset
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  --------------------
 *   ctx       agc_job_ctx_t *  I    job context from the executive
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_job_jet_accel_reset(agc_job_ctx_t *ctx)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->aosq   = 0;
    d->aosr   = 0;
    d->alphaq = 0;
    d->alphar = 0;

    if (!AGC_ASSERT(ctx->id == AGC_JOB_JET_ACCEL_RESET))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_dap_jet_accel_update());
}

/**********************************************************************
 * RCSMONIT / CALLACCS helper: set up 1/ACCJOB (1/ACCFIX, p.192).
 *
 * FUNCTION NAME: lm_dap_accfix
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_accfix(void)
{
    agc_status_t st = agc_exec_novac(AC_PRIO27, AGC_JOB_JET_ACCEL_UPDATE); /* NOVAC 1/ACCS */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    _Static_assert(AC_PRIO27 == 027, "AGC constant");

    return (st);
}
