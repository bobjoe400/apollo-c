/**********************************************************************
 * FILE NAME: free_fall.c
 *
 * PURPOSE:
 *   TIME OF FREE FALL (R. R. Bairnsfather): conic parameters (TFFCONIC),
 *   perigee and apogee radii (TFFRP/RA), and the time of free fall to a
 *   radius (CALCTFF) or to perigee (CALCTPER) with T(X), the general
 *   conic, the |delta E| >= 90 deg ellipse (TFFEL1) and ENDTFF.
 *
 * AGC SOURCE:
 *   Comanche055 TIME_OF_FREE_FALL.agc pp. 1373-1388 (TFFCONIC, TFFCONMU,
 *     TFFRP/RA, CALCTPER, CALCTFF, TFFXTEST, TFFEL1, ENDTFF, T(X))
 *   Luminary099 TIME_OF_FREE_FALL.agc (identical; its 1/RTMU lives in
 *     CONTROLLED_CONSTANTS.agc)
 *
 * EXTERNAL VARIABLES:
 *   none (the push-list quantities are fields of the caller's agc_tff_t)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dp_mul_tp         DMP keeping MPAC+2 (interp.h)
 *   agc_tp_shift/sqrt     SL* and SR*, SQRT on the TP MPAC (interp.h)
 *   agc_vec_sq_tp         VSQ to TP (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms.  No free fall (no conic, overflow) returns NEARONE
 *   (AGC_TFF_NEARONE); AGC_STATUS_BAD_ARG on NULL or bad inputs.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   The TFF routines never know the origin: the caller supplies RONE,
 *   VONE and 1/sqrt(MU) at earth scale (E: 2^29 m, 2^7 m/cs, 2^-17) or
 *   moon scale (M: 2^27 m, 2^5 m/cs, 2^-14).  Free-fall times are cs at
 *   scale 2^28.  NR and NA are the positive normalization counts (the AGC
 *   keeps -NR in X1, -NA in X2; its "SL* k,1" shifts by k + NR).
 *
 * NOTES:
 *   Interpretive DMP never rounds and DAD touches only the top two
 *   words; the TP chains are kept so RPER and RAPO are bit-exact.
 *   TFFCONIC, CALCTFF/CALCTPER and TFFXTEST run on the MPAC words
 *   (interp_words.h): DAD leaves ALFA, NUM and DEN with words that may
 *   disagree in sign, and SQRT, DDV and DMP read them as stored, so
 *   every output is bit-exact (TFFALFA is kept as words).
 *   VGAMCALC and TFF/TRIG, which use these results, are in P61-P67.agc.
 *
 * ALGORITHM:
 *   H = R x V, P = H.H/MU, ALFA = 2/R - V.V/MU; Z from the inbound or
 *   outbound form of the universal variable; X = ALFA Z^2; TFF from the
 *   T(X) series, or TFFEL1 when Z is large.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/interp_words.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* PI/16 2DEC 3.141592653 B-4, oct 06220 37553: pi scale 2^4 (2 pi at 2^5). */
static const agc_dp_t PI_16 = 52707179; /* pi, scale 2^4  */

/* DP2(-3) 2DEC 1 B-3, oct 04000 00000: 1 scale 2^3.                       */
static const agc_dp_t DP2_M3 = 33554432; /* 1, scale 2^3   */

/* DP2(-4) 2DEC 1 B-4, oct 02000 00000: 1 scale 2^4.                       */
static const agc_dp_t DP2_M4 = 16777216; /* 1, scale 2^4   */

/* DP(-22) 2OCT 0000000100: 2^-22.  LIM(-22) 2OCT 3777737700 = 1 - 2^-22;
 * DAD LIM(-22) overflows exactly when |DEN| >= DP(-22).                   */
static const agc_dp_t DP_M22 = 64; /* 2^-22, 2^0     */

/* 1/RTMU 2DEC* .5005750271 E-5 B17*, oct 24775 30424: modified earth
 * 1/sqrt(MU) (MU = 3.990815471E10 m^3/cs^2), scale 2^-17.  Exported for
 * callers through agc_tff_t.rtmu; kept here for reference.                */
#define TFF_1_RTMU ((agc_dp_t) 176124180) /* 1/sqrt(MU), 2^-17 */

/* T(X) coefficients A0..A5, 2DEC (scale 2^0), p. 1387 (Chebyshev fit of
 * 1/3 - X/5 + X^2/7 - ..., max deviation 2E-5 over (-.08, 1)).            */
#define TFF_POLY_N 6 /* A0..A5 */
static const agc_dp_t TFF_POLY[TFF_POLY_N] = { /* A0..A5, scale 2^0 */
    89478485,  /* A0  3.333333333E-1   oct 12525 12525 */
    -53682236, /* A1 -1.999819135E-1   oct 71463 57703 */
    38068133,  /* A2  1.418148467E-1   oct 04423 17645 */
    -27195464, /* A3 -1.01310997E-1    oct 74604 43667 */
    15056558,  /* A4  5.609004986E-2   oct 01626 37256 */
    -4123590,  /* A5 -1.536156925E-2   oct 77404 52071 */
};

/**********************************************************************
 * T(X) (TIME_OF_FREE_FALL.agc p. 1387, label T(X)) via POLY
 *   (INTERPRETER.agc p. 1139) -- Horner sum A5 X^5 + ... + A0
 *
 * FUNCTION NAME: agc_tff_poly
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   x         agc_dp_t        I    X, scale 2^0
 *
 * RETURNS: agc_dp_t -- T(X), scale 2^0.  DMPSUB truncates each product
 *          and DAS adds without an overflow check ("user's
 *          responsibility").
 **********************************************************************/
agc_dp_t agc_tff_poly(agc_dp_t x)
{
    agc_dp_t  acc = TFF_POLY[TFF_POLY_N - 1]; /* Horner sum, scale 2^0 */
    agc_ovf_t ovf = 0;                        /* DAS overflow, ignored */

    for (int i = TFF_POLY_N - 2; i >= 0; i--)
    {
        agc_dp_t prod = agc_tp_to_dp(agc_dp_mul_tp(acc, x)); /* 2^0 */

        acc = agc_dp_add_raw(prod, TFF_POLY[i], &ovf);
    }

    return (acc);
}

/**********************************************************************
 * TFFCONIC / TFFCONMU (TIME_OF_FREE_FALL.agc pp. 1375-1377, labels
 *   TFFCONIC, TFFCONMU) -- conic parameters
 *   H = RN x VN, LCP = H.H / MU, ALFA = 2/RN - VN.VN / MU
 *
 * FUNCTION NAME: agc_tff_conic
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   tff       agc_tff_t *     I/O  in: rone, vone, rtmu; out: vone_p,
 *                                  rmag1, nrmag, nr, tffnp, tffvsq,
 *                                  tffalfa, tffrtalf, na, tff1_alf
 *                                  (scales in the type, auxiliary.h)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (always)
 **********************************************************************/
agc_status_t agc_tff_conic(agc_interp_t *ip, agc_tff_t *tff)
{
    agc_dp_t  mag = 0;                                       /* |RN|, E 2^29 m */
    agc_vec_t ur  = agc_vec_unit(ip, tff->rone, &mag, NULL); /* UR, 2^1 */

    tff->rmag1  = mag;
    tff->nrmag  = agc_dp_norm(mag, &tff->nr);                                        /* NORM X1    */
    tff->vone_p = agc_vec_scale(ip, tff->vone, tff->rtmu);                           /* VONE'      */
    agc_vec_t h = agc_vec_cross(ip, agc_vec_scale(ip, tff->vone_p, tff->nrmag), ur); /* H/sqrt(MU), E 2^(20-NR) */

    tff->tffnp     = agc_vec_sq(ip, agc_vec_shl(ip, h, 1));      /* VSL1 VSQ   */
    agc_dp_t two_r = agc_dp_div(ip, AGC_DP_QUARTER, tff->nrmag); /* 2/RMAG, E 2^-(26-NR) */

    /* VSQ DCOMP keeps MPAC+2 ("KEEP MPAC+2 HONEST FOR SQRT"); SR*, DAD
     * (top two words only), SL*, ABS SQRT, NORM and DSQ SIGN all run on
     * the MPAC words, whose signs may disagree after the DAD.             */
    agc_tp_words_t m = agc_mpac_dcomp(agc_mpac_vsq(ip, tff->vone_p)); /* -V^2/MU, 2^-20 */

    tff->tffvsq  = agc_dp_from_words(agc_mpac_store(m));        /* STORE TFFVSQ */
    m            = agc_mpac_shift(ip, m, 6 - tff->nr);          /* SR* 0-6,1 */
    m            = agc_mpac_dad(ip, m, agc_dp_to_words(two_r)); /* DAD 2/RMAG */
    tff->tffalfa = agc_mpac_store(m);                           /* ALFA as stored, 2^-26+NR */
    m            = agc_mpac_shift(ip, m, tff->nr - 6);          /* SL* 0-6,1 */

    const agc_dp_words_t alfa20 = agc_mpac_store(m); /* PUSH, E 2^-20 */

    m             = agc_mpac_norm(agc_mpac_sqrt(agc_mpac_abs(m)), &tff->na); /* ABS SQRT NORM X2 */
    tff->tffrtalf = agc_dp_from_words(agc_mpac_store(m));                    /* E 2^(10+NA) */
    m             = agc_mpac_sign(agc_mpac_dsq(m), alfa20);                  /* DSQ SIGN, 2^-(20+2NA) */

    /* BZE tests all three words; BDDV divides by the top two.            */
    tff->tff1_alf = 0;

    if ((m.hi != 0) || (m.mid != 0) || (m.lo != 0))
    {
        tff->tff1_alf = agc_dp_from_words(agc_mpac_store(agc_mpac_bddv(ip, m, agc_dp_to_words(AGC_DP_QUARTER))));
    }

    return (AGC_STATUS_OK);
} /* agc_tff_conic */

/**********************************************************************
 * TFFRP/RA (TIME_OF_FREE_FALL.agc pp. 1378-1379, label TFFRP/RA)
 *   -- RP = P / (1 + E), RA = (1 + E) / ALFA, E^2 = 1 - P ALFA
 *
 * FUNCTION NAME: agc_tff_rp_ra
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   tff       agc_tff_t *     I/O  in: TFFCONIC outputs; out: rper,
 *                                  rapo (E 2^29 m, M 2^27 m)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL or
 *          bad NR/NA.  RA = NEARONE (POSMAX tag) for a hyperbola,
 *          parabola or out-of-scale RA.
 **********************************************************************/
agc_status_t agc_tff_rp_ra(agc_interp_t *ip, agc_tff_t *tff)
{
    if (!AGC_ASSERT((tff->nr >= 0) && (tff->nr <= AGC_DP_BITS) && (tff->na >= 0) && (tff->na <= AGC_DP_BITS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* DMP, SR* and DCOMP on the TP; DAD adds to its top two words; ABS
     * and SQRT see MPAC+2 ("DCOMP GIVES VALID TP RESULT FOR SQRT").       */
    agc_tp_t ap = agc_tp_from_words(agc_mpac_dmp(agc_mpac_dload(tff->tffalfa), agc_dp_to_words(tff->tffnp))); /* DLOAD TFFALFA DMP TFFNP, 2^(12-NR) */

    ap          = agc_tp_shift(ip, ap, 8 - tff->nr);           /* -> 2^4   */
    agc_tp_t e2 = agc_tp_add(ip, -ap, agc_tp_from_dp(DP2_M4)); /* E^2, 2^4 */

    if (e2 < 0)
    {
        e2 = -e2; /* ABS */
    }

    agc_dp_t onepe = agc_dp_add(ip, agc_tp_sqrt(e2), AGC_DP_QUARTER);   /* 1+E, scale 2^2 */
    agc_tp_t rp    = agc_tp_from_dp(agc_dp_div(ip, tff->tffnp, onepe)); /* P/(1+E), 2^(36-2NR) */

    rp          = agc_tp_shift(ip, rp, -tff->nr);                  /* SR* 0,1  */
    tff->rper   = agc_tp_to_dp(agc_tp_shift(ip, rp, 7 - tff->nr)); /* 0-7,1 */
    agc_tp_t ra = agc_dp_mul_tp(onepe, tff->tff1_alf);             /* 2^(24+2NA) */

    agc_interp_clear_ovf(ip);    /* BOVB TCDANZIG */
    tff->rapo = AGC_TFF_NEARONE; /* MAXRA    */

    if (ra != 0)
    { /* BZE MAXRA */
        ra = agc_tp_shift(ip, ra, tff->na - 5); /* SL* 0-5,2 */
        ra = agc_tp_shift(ip, ra, tff->na);     /* SL* 0,2   */

        if (!agc_interp_test_ovf(ip) && (ra >= 0))
        {
            tff->rapo = agc_tp_to_dp(ra);
        }
    }

    return (AGC_STATUS_OK);
} /* agc_tff_rp_ra */

/**********************************************************************
 * ENDTFF (TIME_OF_FREE_FALL.agc p. 1384, label ENDTFF)
 *   -- TFF = (TFF sqrt(MU)) / sqrt(MU)
 *
 * FUNCTION NAME: tff_end
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ip        agc_interp_t *     I    interpreter state (OVFIND test)
 *   tff       const agc_tff_t *  I    rtmu, 1/sqrt(MU) E 2^-17
 *   tff_rtmu  agc_dp_words_t     I    TFF sqrt(MU) as MPAC holds it,
 *                                     E 2^45
 *   t_cs      agc_dp_t *         O    TFF, cs scale 2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (NEARONE in t_cs on overflow)
 **********************************************************************/
static agc_status_t tff_end(agc_interp_t *ip, const agc_tff_t *tff, agc_dp_words_t tff_rtmu, agc_dp_t *t_cs)
{
    /* DMP TFF/RTMU: the caller takes the top two words (truncation).     */
    *t_cs = agc_dp_from_words(agc_mpac_store(agc_mpac_dmp(agc_mpac_dload(tff_rtmu), agc_dp_to_words(tff->rtmu)))); /* TFF, 2^28 cs */

    if (agc_interp_test_ovf(ip))
    {
        *t_cs = AGC_TFF_NEARONE; /* BOV MAXTFF  */
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * TFFEL1 (TIME_OF_FREE_FALL.agc pp. 1385-1386, label TFFEL1)
 *   -- ellipse with |delta E| >= 90 deg, X = 1/(ALFA Z^2),
 *   TFF = (pi/sqrt(ALFA) - Q2 + Q1 + 2(X T(X) - 1)/(ALFA Z))
 *         / (ALFA sqrt(MU))
 *
 * FUNCTION NAME: tff_el1
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   tff       agc_tff_t *     I/O  conic data; out: tffdelq, tffx,
 *                                  tfftem
 *   dn        agc_dp_t        I    D/N = 1/Z, E 2^-11, M 2^-10
 *   q2        agc_dp_t        I    Q2, E 2^16, M 2^15
 *   t_cs      agc_dp_t *      O    TFF, cs scale 2^28
 *
 * RETURNS: agc_status_t -- the status of ENDTFF, tff_end() (always
 *          AGC_STATUS_OK)
 **********************************************************************/
static agc_status_t tff_el1(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t dn, agc_dp_t q2, agc_dp_t *t_cs)
{
    int na = tff->na; /* NA, normalization count */

    tff->tffdelq = agc_dp_sub(ip, q2, tff->tffq1); /* Q2-Q1    */

    /* DMP SL* 0,2 / PUSH: 1/(ALFA Z), TP shifted, pushed as DP.           */
    agc_dp_t inv_az = agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(dn, tff->tff1_alf), na)); /* 1/(ALFA Z), E 2^(11+NA) */

    /* DMP SL* 0,2 BOVB SIGNMPAC: X = 1/(ALFA Z^2); on overflow (X = 1.0)
     * SIGNMPAC loads +-DPPOSMAX with the sign of the overflow.            */
    agc_tp_t xp = agc_dp_mul_tp(inv_az, dn); /* X before SL*, 2^-NA */

    agc_interp_clear_ovf(ip);
    agc_tp_t xs = agc_tp_shift(ip, xp, na); /* X, scale 2^0 */
    agc_dp_t x  = agc_tp_to_dp(xs);         /* X, scale 2^0 */

    if (agc_interp_test_ovf(ip))
    {
        x = AGC_DP_MAX; /* SIGNMPAC        */

        if (xp < 0)
        {
            x = -AGC_DP_MAX;
        }
    }

    tff->tffx = x;

    /* RTB DMP T(X) X, SR3 DSU DP2(-3), DMP (1/ALFA Z), PUSH.               */
    agc_tp_t xt   = agc_tp_shift(ip, agc_dp_mul_tp(agc_tff_poly(x), x), -3);            /* X T(X), scale 2^3 */
    agc_dp_t term = agc_dp_mul_trunc(agc_dp_sub(ip, agc_tp_to_dp(xt), DP2_M3), inv_az); /* (X T(X) - 1)/(ALFA Z), 2^(14+NA) */

    /* DLOAD DMP D/N RMAG1, SL2, DAD TFFQ1: sign of SDELF.                 */
    agc_tp_t sgt = agc_dp_mul_tp(dn, tff->rmag1); /* D/N RMAG1, 2^14 */

    sgt         = agc_tp_shift(ip, sgt, 2);                      /* SL2, scale 2^16  */
    agc_dp_t sg = agc_dp_add(ip, agc_tp_to_dp(sgt), tff->tffq1); /* 2^16 */

    /* DLOAD TFFNP, DMP SL* 1,2, SIGN, SL* 0,2: P/ALFA for TFF/TRIG.       */
    agc_tp_t pa = agc_tp_shift(ip, agc_dp_mul_tp(tff->tffnp, tff->tff1_alf), 1 + na); /* P/ALFA, E 2^(59-2NR) after the second SL* */

    if (sg < 0)
    {
        pa = -pa; /* SIGN TFFTEM                       */
    }

    tff->tfftem = agc_tp_to_dp(agc_tp_shift(ip, pa, na)); /* P/ALFA   */

    /* DLOAD 1/ALFA, SQRT DMP PI/16, DAD term, SL* 0-1,2, DSU Q2-Q1,
     * DMP 1/ALFA SL* 0-3,2 SL* 0-4,2: TFF sqrt(MU) scale 2^45 (E).        */
    agc_tp_t st = agc_dp_mul_tp(agc_dp_sqrt(tff->tff1_alf), PI_16); /* running sum, TP, E 2^(15+NA) .. 2^45 */

    st         = agc_tp_add(ip, st, agc_tp_from_dp(term));
    st         = agc_tp_shift(ip, st, na - 1);
    agc_dp_t s = agc_dp_sub(ip, agc_tp_to_dp(st), tff->tffdelq); /* 2^16 */

    st = agc_dp_mul_tp(s, tff->tff1_alf);
    st = agc_tp_shift(ip, st, na - 3);
    s  = agc_tp_to_dp(agc_tp_shift(ip, st, na - 4));

    return (tff_end(ip, tff, agc_dp_to_words(s), t_cs));
} /* tff_el1 */

/**********************************************************************
 * TFFXTEST continuation (TIME_OF_FREE_FALL.agc pp. 1383-1384) -- the
 *   general conic, alfa Z Z < 1:  X = ALFA Z^2,
 *   TFF = (RTERM + RN - 2 Z^2 T(X)) Z / sqrt(MU)
 *
 * FUNCTION NAME: tff_general
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   tff       agc_tff_t *     I/O  conic data; out: tffx, tfftem
 *   num       agc_dp_words_t  I    numerator of Z (TFFX), E 2^29 / 2^16
 *   den       agc_dp_words_t  I    denominator of Z (PDL+0), E 2^16 /
 *                                  2^3
 *   t_cs      agc_dp_t *      O    TFF, cs scale 2^28
 *
 * RETURNS: agc_status_t -- status of ENDTFF, AGC_STATUS_BAD_ARG when
 *          TFFRTALF = 0 on the period correction
 **********************************************************************/
static agc_status_t tff_general(agc_interp_t *ip, agc_tff_t *tff, agc_dp_words_t num, agc_dp_words_t den, agc_dp_t *t_cs)
{
    /* DDV, STORE TFFTEM, PUSH DSQ PUSH DMP TFFNP SL 5 SIGN TFFTEM.       */
    agc_tp_words_t       m   = agc_mpac_ddv(ip, agc_mpac_dload(num), den); /* Z, E 2^13 */
    const agc_dp_words_t z   = agc_mpac_store(m);                          /* PDL+0 */
    const agc_dp_words_t zsq = agc_mpac_store(agc_mpac_dsq(m));            /* Z^2, PDL+2, E 2^26 */

    m           = agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(zsq), agc_dp_to_words(tff->tffnp)), 5); /* P Z^2, E 2^59 */
    tff->tfftem = agc_dp_from_words(agc_mpac_store(agc_mpac_sign(m, z)));                                /* SIGN Z */

    /* DLOAD ZSQ, DMP TFFALFA, SL* 0,1: X = ALFA Z^2.                      */
    tff->tffx = agc_dp_from_words(agc_mpac_store(agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(zsq), tff->tffalfa), tff->nr)));

    /* RTB DMP T(X) Z^2, SR2, BDSU RTERM, DAD RMAG1, DMP Z, SR3.            */
    m = agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(agc_tff_poly(tff->tffx))), zsq), -2); /* Z^2 T(X), E 2^28 */
    m = agc_mpac_bdsu(ip, m, agc_dp_to_words(tff->rterm));                                                   /* RTERM - Z^2 T(X), E 2^29 m */
    m = agc_mpac_dad(ip, m, agc_dp_to_words(tff->rmag1));                                                    /* + RN */
    m = agc_mpac_shift(ip, agc_mpac_dmp(m, z), -3);                                                          /* TFF sqrt(MU), E 2^45 */

    agc_dp_words_t tsm = agc_mpac_store(m); /* TFF sqrt(MU), E 2^45 */

    if (agc_tp_from_words(m) < 0)
    { /* BPL ENDTFF   */
        bool gone_past = (agc_dp_sign(agc_dp_from_words(tsm), tff->tffq1) >= 0); /* SIGN TFFQ1 */

        if (!gone_past && (tff->tff1_alf > 0))
        { /* ellipse      */
            /* Orbital-period correction for a negative TFF on an ellipse:
             * TFF + 2 pi / (ALFA sqrt(ALFA)), scale 2^45 (E), 2^42 (M).   */
            if (!AGC_ASSERT(tff->tffrtalf != 0))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            /* DMP PI/16 DDV TFFRTALF (top words), SL* SL* SL* on the TP. */
            agc_dp_t pia = agc_dp_mul_trunc(tff->tff1_alf, PI_16);             /* pi/ALFA, E 2^(26+2NA) */
            agc_tp_t per = agc_tp_from_dp(agc_dp_div(ip, pia, tff->tffrtalf)); /* period term, E 2^45 after the shifts */

            per = agc_tp_shift(ip, per, tff->na - 4);
            per = agc_tp_shift(ip, per, tff->na - 4);
            per = agc_tp_shift(ip, per, tff->na);
            tsm = agc_dp_to_words(agc_dp_add(ip, agc_tp_to_dp(per), agc_dp_from_words(tsm))); /* DAD PDL+0 */
        } /* else NEGTFF  */
    }

    return (tff_end(ip, tff, tsm, t_cs));
} /* tff_general */

/**********************************************************************
 * SAVEDEN .. TFFXTEST (TIME_OF_FREE_FALL.agc pp. 1383-1384, labels
 *   SAVEDEN, TFFXTEST, TFFELL) -- indeterminacy test and choice of
 *   equation
 *
 * FUNCTION NAME: tff_z
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   tff       agc_tff_t *     I/O  conic data; out: tffx, tfftem
 *   num       agc_dp_words_t  I    numerator of Z as stored (TFFX),
 *                                  E 2^29 / 2^16
 *   den       agc_dp_words_t  I    denominator of Z as pushed (PDL+0),
 *                                  E 2^16 / 2^3
 *   q2        agc_dp_t        I    Q2, E 2^16, M 2^15
 *   t_cs      agc_dp_t *      O    TFF, cs scale 2^28 (0: perigee of a
 *                                  parabola/hyperbola)
 *
 * RETURNS: agc_status_t -- status of ENDTFF / TFFEL1 / the general
 *          conic
 **********************************************************************/
static agc_status_t tff_z(agc_interp_t *ip, agc_tff_t *tff, agc_dp_words_t num, agc_dp_words_t den, agc_dp_t q2, agc_dp_t *t_cs)
{
    const agc_dp_words_t lim_m22 = {AGC_SP_MAX, 16320}; /* LIM(-22) 2OCT 3777737700: 1 - 2^-22 */
    const agc_dp_words_t dp_m22  = {0, DP_M22};         /* DP(-22) 2OCT 0000000100: 2^-22 */

    tff->tffx = agc_dp_from_words(num);

    /* PUSH ABS, DAD LIM(-22) BOV TFFXTEST: overflows iff |DEN| >= 2^-22;
     * the BOV also sees any OVFIND left by the Q1/DEN arithmetic.        */
    agc_tp_words_t m = agc_mpac_dad(ip, agc_mpac_abs(agc_mpac_dload(den)), lim_m22); /* |DEN| + LIM(-22) */

    if (!agc_interp_test_ovf(ip))
    { /* DEN = 0: Z indeterminate   */
        if (agc_dp_from_words(tff->tffalfa) > 0)
        {
            return (tff_el1(ip, tff, 0, q2, t_cs)); /* assume 90 deg, D/N = 0 */
        }

        *t_cs = 0; /* DUMPTFF1: perigee of parab/hyp   */

        return (AGC_STATUS_OK);
    }

    /* TFFXTEST: DAD DP(-22) restores |DEN| (the overflow undone), PDDL
     * TFFX, DMP TFFRTALF, SR* 0-3,2, DDV |DEN|: only OVFIND is used.     */
    const agc_dp_words_t absden = agc_mpac_store(agc_mpac_dad(ip, m, dp_m22)); /* |DEN|, PDL+2 */

    m = agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(num), agc_dp_to_words(tff->tffrtalf)), 3 - tff->na); /* NUM sqrt(ALFA) */
    m = agc_mpac_ddv(ip, m, absden);

    if (agc_interp_test_ovf(ip))
    { /* BOV TFFELL: SL2 BDDV DEN, D/N  */
        agc_dp_t dn = agc_dp_from_words(agc_mpac_store(agc_mpac_bddv(ip, agc_mpac_shift(ip, agc_mpac_dload(num), 2), den))); /* 2^-11 */

        return (tff_el1(ip, tff, dn, q2, t_cs));
    }

    return (tff_general(ip, tff, num, den, t_cs));
} /* tff_z */

/**********************************************************************
 * CALCTPER / CALCTFF (TIME_OF_FREE_FALL.agc pp. 1380-1383, labels
 *   CALCTPER, CALCTFF) -- common part
 *   Q2 = -sqrt(RTERM (2 - RTERM ALFA) - LCP)   (0 for CALCTPER)
 *   Q1 = RN.VN / sqrt(MU)
 *   inbound (Q1 < 0): Z = (RTERM - RN) / (Q2 + Q1)
 *   outbound:         Z = (Q2 - Q1) / (2 - ALFA (RTERM + RN))
 *
 * FUNCTION NAME: tff_calc
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   tff       agc_tff_t *     I/O  conic data; out: rterm, nrterm,
 *                                  tfftem, tffq1 and the TFF debris
 *   rterm     agc_dp_t        I    RTERM, E 2^29 m, M 2^27 m
 *   tper      bool            I    TFFSW: true for CALCTPER (Q2 = 0)
 *   t_cs      agc_dp_t *      O    TFF, cs scale 2^28; NEARONE when no
 *                                  conic reaches RTERM
 *
 * RETURNS: agc_status_t -- status of TFFXTEST, AGC_STATUS_BAD_ARG on
 *          a normalization count out of range
 **********************************************************************/
static agc_status_t tff_calc(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t rterm, bool tper, agc_dp_t *t_cs)
{
    if (!AGC_ASSERT((tff->nr >= 0) && (tff->nr <= AGC_DP_BITS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int nr = tff->nr; /* NR, normalization count */

    tff->rterm  = rterm;
    tff->nrterm = agc_navaux_dp_shift(ip, rterm, nr); /* SL* 0,1 */

    /* On the MPAC words: DMP TFFALFA, BDSU TFF1/4, PUSH (2 - ALFA RTERM,
     * 2^3), DMP NRTERM, PDDL TFFNP SR* 0-6,1, DCOMP, DAD, SR* 0,1 ("DUE TO
     * SHIFTS, KEEP PRECISION FOR SQRT"); BMN and SQRT see all three words. */
    agc_tp_words_t       m     = agc_mpac_bdsu(ip, agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(tff->nrterm)), tff->tffalfa), agc_dp_to_words(AGC_DP_QUARTER)); /* 2 - ALFA RTERM, 2^3 */
    const agc_dp_words_t two_m = agc_mpac_store(m);                                                                                                            /* PDL+0, 2^3 */
    const agc_dp_words_t rt2   = agc_mpac_store(agc_mpac_dmp(m, agc_dp_to_words(tff->nrterm)));                                                                /* PDL+2, E 2^(32-NR) */

    m = agc_mpac_dcomp(agc_mpac_shift(ip, agc_mpac_dload(agc_dp_to_words(tff->tffnp)), 6 - nr)); /* -P */
    m = agc_mpac_shift(ip, agc_mpac_dad(ip, m, rt2), -nr);                                       /* Q2^2, E 2^32 */

    if (tper)
    {
        m = agc_mpac_dload(agc_dp_to_words(0)); /* TFFZEROS: Q2 = 0  */
    }

    if (agc_tp_from_words(m) < 0)
    {
        *t_cs = AGC_TFF_NEARONE; /* MAXTFF1: no conic */

        return (AGC_STATUS_OK);
    }

    const agc_dp_words_t q2 = agc_mpac_store(agc_mpac_dcomp(agc_mpac_sqrt(m))); /* Q2, E 2^16 */

    agc_interp_clear_ovf(ip); /* BOVB TCDANZIG     */
    tff->tfftem = agc_dp_from_words(q2);

    /* VLOAD VONE' DOT RONE SL3 STORE TFFQ1; BMN INBOUND.                  */
    m          = agc_mpac_shift(ip, agc_mpac_dot(ip, tff->vone_p, tff->rone), 3); /* Q1, 2^16 */
    tff->tffq1 = agc_dp_from_words(agc_mpac_store(m));

    if (tff->tffq1 < 0)
    { /* INBOUND: DLOAD RTERM DSU RMAG1; DLOAD TFFTEM DAD TFFQ1  */
        const agc_dp_words_t num = agc_mpac_store(agc_mpac_dsu(ip, agc_mpac_dload(agc_dp_to_words(rterm)), agc_dp_to_words(tff->rmag1))); /* NUM of Z (TFFX) */

        return (tff_z(ip, tff, num, agc_mpac_store(agc_mpac_dad(ip, agc_mpac_dload(q2), agc_mpac_store(m))), tff->tfftem, t_cs));
    }

    /* BDSU TFFTEM: NUM = Q2 - Q1; DLOAD TFFALFA DMP NRMAG BDSU (PDL+0).   */
    const agc_dp_words_t num = agc_mpac_store(agc_mpac_bdsu(ip, m, q2));                                                                          /* NUM of Z (TFFX) */
    const agc_dp_words_t den = agc_mpac_store(agc_mpac_bdsu(ip, agc_mpac_dmp(agc_mpac_dload(tff->tffalfa), agc_dp_to_words(tff->nrmag)), two_m)); /* 2^3 */

    return (tff_z(ip, tff, num, den, tff->tfftem, t_cs));
} /* tff_calc */

/**********************************************************************
 * CALCTFF (TIME_OF_FREE_FALL.agc p. 1382, label CALCTFF)
 *   -- time of free fall to radius RTERM (inbound side)
 *
 * FUNCTION NAME: agc_tff_calctff
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   tff       agc_tff_t *     I/O  TFFCONIC data; out: TFF debris
 *   rterm     agc_dp_t        I    RTERM, E 2^29 m, M 2^27 m, >= 0
 *   t_cs      agc_dp_t *      O    TFF, cs scale 2^28 (NEARONE: none)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG
 **********************************************************************/
agc_status_t agc_tff_calctff(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t rterm, agc_dp_t *t_cs)
{
    if (!AGC_ASSERT(rterm >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (tff_calc(ip, tff, rterm, false, t_cs)); /* CLEAR TFFSW     */
}

/**********************************************************************
 * CALCTPER (TIME_OF_FREE_FALL.agc p. 1382, label CALCTPER)
 *   -- time to perigee
 *
 * FUNCTION NAME: agc_tff_calctper
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   tff       agc_tff_t *     I/O  TFFCONIC data; out: TFF debris
 *   rper      agc_dp_t        I    RPER from TFFRP/RA, E 2^29 m, >= 0
 *   t_cs      agc_dp_t *      O    time to perigee, cs scale 2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG
 **********************************************************************/
agc_status_t agc_tff_calctper(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t rper, agc_dp_t *t_cs)
{
    if (!AGC_ASSERT(rper >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (tff_calc(ip, tff, rper, true, t_cs)); /* SET TFFSW       */
}
