/*
 * free_fall.c -- TIME OF FREE FALL
 * (Comanche055 TIME_OF_FREE_FALL.agc pp. 1373-1388, R. R. Bairnsfather;
 * Luminary099 code identical, its 1/RTMU lives in CONTROLLED_CONSTANTS).
 *
 * The TFF routines never know the origin: the caller supplies RONE, VONE
 * and 1/sqrt(MU) at earth scale (E: 2^29 m, 2^7 m/cs, 2^-17) or moon scale
 * (M: 2^27 m, 2^5 m/cs, 2^-14).  Free-fall times are returned in cs at
 * scale 2^28.  The AGC push-list quantities are fields of agc_tff_t; NR
 * and NA are the positive normalization counts (the AGC keeps -NR in X1,
 * -NA in X2, and its indexed shift "SL* k,1" shifts by k + NR).
 *
 * VGAMCALC and TFF/TRIG, which use these results, are in P61-P67.agc.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* PI/16 2DEC 3.141592653 B-4, oct 06220 37553: pi scale 2^4 (2 pi at 2^5). */
static const agc_dp_t PI_16 = 52707179;
/* DP2(-3) 2DEC 1 B-3, oct 04000 00000: 1 scale 2^3.                       */
static const agc_dp_t DP2_M3 = 33554432;
/* DP2(-4) 2DEC 1 B-4, oct 02000 00000: 1 scale 2^4.                       */
static const agc_dp_t DP2_M4 = 16777216;
/* DP(-22) 2OCT 0000000100: 2^-22.  LIM(-22) 2OCT 3777737700 = 1 - 2^-22;
 * DAD LIM(-22) overflows exactly when |DEN| >= DP(-22).                   */
static const agc_dp_t DP_M22 = 64;
/* 1/RTMU 2DEC* .5005750271 E-5 B17*, oct 24775 30424: modified earth
 * 1/sqrt(MU) (MU = 3.990815471E10 m^3/cs^2), scale 2^-17.  Exported for
 * callers through agc_tff_t.rtmu; kept here for reference.                */
#define TFF_1_RTMU ((agc_dp_t) 176124180)

/* T(X) coefficients A0..A5, 2DEC (scale 2^0), p. 1387 (Chebyshev fit of
 * 1/3 - X/5 + X^2/7 - ..., max deviation 2E-5 over (-.08, 1)).            */
#define TFF_POLY_N 6
static const agc_dp_t TFF_POLY[TFF_POLY_N] = {
    89478485,  /* A0  3.333333333E-1   oct 12525 12525 */
    -53682236, /* A1 -1.999819135E-1   oct 71463 57703 */
    38068133,  /* A2  1.418148467E-1   oct 04423 17645 */
    -27195464, /* A3 -1.01310997E-1    oct 74604 43667 */
    15056558,  /* A4  5.609004986E-2   oct 01626 37256 */
    -4123590,  /* A5 -1.536156925E-2   oct 77404 52071 */
};

/*
 * T(X) (TIME_OF_FREE_FALL.agc p. 1387, label T(X)) via POLY
 * (INTERPRETER.agc p. 1139): Horner sum A5 X^5 + ... + A0, DMPSUB
 * (truncated product) and DAS (no overflow check, "user's responsibility").
 */
agc_dp_t agc_tff_poly(agc_interp_t *ip, agc_dp_t x)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(x)))
    {
        return (0);
    }

    agc_dp_t  acc = TFF_POLY[TFF_POLY_N - 1];
    agc_ovf_t ovf = 0;

    for (int i = TFF_POLY_N - 2; i >= 0; i--)
    {
        agc_dp_t prod = agc_tp_to_dp(agc_dp_mul_tp(acc, x));

        acc = agc_dp_add_raw(prod, TFF_POLY[i], &ovf);
    }

    return (acc);
}

/*
 * TFFCONIC / TFFCONMU (TIME_OF_FREE_FALL.agc pp. 1375-1377)
 *   H = RN x VN, LCP = H.H / MU, ALFA = 2/RN - VN.VN / MU
 * Caller sets tff->rone, vone, rtmu (TFFCONMU: rtmu already set).
 */
agc_status_t agc_tff_conic(agc_interp_t *ip, agc_tff_t *tff)
{
    if (!AGC_ASSERT((ip != NULL) && (tff != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_vec_ok(tff->rone) &&
                    agc_navaux_vec_ok(tff->vone)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t  mag = 0;
    agc_vec_t ur  = agc_vec_unit(ip, tff->rone, &mag, NULL); /* UR/2, 36D  */

    tff->rmag1  = mag;
    tff->nrmag  = agc_dp_norm(mag, &tff->nr);                /* NORM X1    */
    tff->vone_p = agc_vec_scale(ip, tff->vone, tff->rtmu); /* VONE'      */
    agc_vec_t h =
        agc_vec_cross(ip, agc_vec_scale(ip, tff->vone_p, tff->nrmag), ur);

    tff->tffnp = agc_vec_sq(ip, agc_vec_shl(ip, h, 1)); /* VSL1 VSQ   */
    agc_dp_t two_r = agc_dp_div(ip, AGC_DP_QUARTER, tff->nrmag); /* 2/RMAG */

    tff->tffvsq = agc_dp_neg(agc_vec_sq(ip, tff->vone_p)); /* VSQ DCOMP  */
    agc_dp_t vsq =
        agc_navaux_dp_shift(ip, tff->tffvsq, 6 - tff->nr); /* SR* 0-6,1 */

    tff->tffalfa = agc_dp_add(ip, vsq, two_r);          /* ALFA       */
    agc_dp_t alfa20 = agc_navaux_dp_shift(ip, tff->tffalfa, tff->nr - 6);

    tff->tffrtalf = agc_dp_norm(agc_dp_sqrt(ip, agc_dp_abs(alfa20)), &tff->na);
    agc_dp_t a =
        agc_dp_sign(agc_dp_sq(ip, tff->tffrtalf), alfa20); /* DSQ SIGN */

    tff->tff1_alf = (a == 0) ? 0 : agc_dp_div(ip, AGC_DP_QUARTER, a);

    return (AGC_STATUS_OK);
}

/*
 * TFFRP/RA (TIME_OF_FREE_FALL.agc pp. 1378-1379)
 *   RP = P / (1 + E),  RA = (1 + E) / ALFA,  E^2 = 1 - P ALFA.
 * RA = NEARONE (POSMAX tag) for hyperbola, parabola or out-of-scale RA.
 */
agc_status_t agc_tff_rp_ra(agc_interp_t *ip, agc_tff_t *tff)
{
    if (!AGC_ASSERT((ip != NULL) && (tff != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((tff->nr >= 0) && (tff->nr <= AGC_DP_BITS) &&
                    (tff->na >= 0) && (tff->na <= AGC_DP_BITS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t ap = agc_dp_mul(ip, tff->tffalfa, tff->tffnp); /* (-12+NR) */

    ap = agc_navaux_dp_shift(ip, ap, 8 - tff->nr); /* -> 2^4   */
    agc_dp_t e2    = agc_dp_abs(agc_dp_add(ip, agc_dp_neg(ap), DP2_M4));
    agc_dp_t onepe =
        agc_dp_add(ip, agc_dp_sqrt(ip, e2), AGC_DP_QUARTER); /* 1+E 2^2 */
    agc_dp_t rp = agc_dp_div(ip, tff->tffnp, onepe);

    rp        = agc_navaux_dp_shift(ip, rp, -tff->nr);      /* SR* 0,1  */
    tff->rper = agc_navaux_dp_shift(ip, rp, 7 - tff->nr); /* SR* 0-7,1 */
    agc_dp_t ra = agc_dp_mul(ip, onepe, tff->tff1_alf);

    agc_interp_clear_ovf(ip); /* BOVB TCDANZIG */
    tff->rapo = AGC_TFF_NEARONE;    /* MAXRA    */

    if (ra != 0)
    {
        ra = agc_navaux_dp_shift(ip, ra, tff->na - 5); /* SL* 0-5,2 */
        ra = agc_navaux_dp_shift(ip, ra, tff->na);     /* SL* 0,2  */

        if (!agc_interp_test_ovf(ip) && (ra >= 0))
        {
            tff->rapo = ra;
        }
    }

    return (AGC_STATUS_OK);
}

/* ENDTFF: TFF = (TFF sqrt(MU)) / sqrt(MU), cs 2^28; NEARONE on overflow. */
static agc_status_t tff_end(agc_interp_t *ip, const agc_tff_t *tff,
                            agc_dp_t tff_rtmu, agc_dp_t *t_cs)
{
    if (!AGC_ASSERT((ip != NULL) && (tff != NULL) && (t_cs != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t t = agc_dp_mul(ip, tff_rtmu, tff->rtmu); /* DMP TFF/RTMU      */

    *t_cs = agc_interp_test_ovf(ip) ? AGC_TFF_NEARONE : t; /* BOV MAXTFF  */

    return (AGC_STATUS_OK);
}

/*
 * TFFEL1 (TIME_OF_FREE_FALL.agc pp. 1385-1386): ellipse with
 * |delta E| >= 90 deg, X = 1/(ALFA Z^2),
 * TFF = (pi/sqrt(ALFA) - Q2 + Q1 + 2(X T(X) - 1)/(ALFA Z)) / (ALFA sqrt(MU)).
 * dn = D/N = 1/Z scale 2^-11 (E) / 2^-10 (M); q2 = Q2.
 */
static agc_status_t tff_el1(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t dn,
                            agc_dp_t q2, agc_dp_t *t_cs)
{
    if (!AGC_ASSERT((ip != NULL) && (tff != NULL) && (t_cs != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int na = tff->na;

    tff->tffdelq = agc_dp_sub(ip, q2, tff->tffq1); /* Q2-Q1    */
    agc_dp_t inv_az =
        agc_navaux_dp_shift(ip, agc_dp_mul(ip, dn, tff->tff1_alf), na);
    agc_dp_t x = agc_navaux_dp_shift(ip, agc_dp_mul(ip, inv_az, dn), na);

    /* DEVIATION: BOVB SIGNMPAC is the saturation agc_dp_shl already gives. */
    agc_interp_clear_ovf(ip);
    tff->tffx = x;
    agc_dp_t xt   = agc_dp_shr(agc_dp_mul(ip, agc_tff_poly(ip, x), x), 3);
    agc_dp_t term = agc_dp_mul(ip, agc_dp_sub(ip, xt, DP2_M3), inv_az);
    agc_dp_t sg   =
        agc_dp_add(ip, agc_dp_shl(ip, agc_dp_mul(ip, dn, tff->rmag1), 2),
                   tff->tffq1); /* sign SDELF */
    agc_dp_t pa = agc_navaux_dp_shift(
        ip, agc_dp_mul(ip, tff->tffnp, tff->tff1_alf), 1 + na);

    tff->tfftem = agc_navaux_dp_shift(ip, agc_dp_sign(pa, sg), na); /* P/ALFA */
    agc_dp_t s = agc_dp_add(
        ip, agc_dp_mul(ip, agc_dp_sqrt(ip, tff->tff1_alf), PI_16), term);

    s = agc_dp_sub(ip, agc_navaux_dp_shift(ip, s, na - 1), tff->tffdelq);
    s = agc_navaux_dp_shift(ip, agc_dp_mul(ip, s, tff->tff1_alf), na - 3);
    s = agc_navaux_dp_shift(ip, s, na - 4); /* TFF sqrt(MU) 2^45   */

    return (tff_end(ip, tff, s, t_cs));
}

/* SAVEDEN .. TFFXTEST (p. 1383): indeterminacy test, choice of equation. */
static agc_status_t tff_z(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t num,
                          agc_dp_t den, agc_dp_t q2, agc_dp_t *t_cs)
{
    if (!AGC_ASSERT((ip != NULL) && (tff != NULL) && (t_cs != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(num) && agc_navaux_dp_ok(den)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    tff->tffx = num;
    agc_dp_t absden = agc_dp_abs(den);
    /* DAD LIM(-22) BOV TFFXTEST: overflows iff |DEN| >= 2^-22; the BOV also
     * sees any OVFIND left by the Q1/DEN arithmetic, and clears it.       */
    bool ovf_prior = agc_interp_test_ovf(ip);

    if (!ovf_prior && (absden < DP_M22))
    { /* DEN = 0: Z indeterminate   */
        if (tff->tffalfa > 0)
        {
            return (tff_el1(ip, tff, 0, q2, t_cs)); /* assume 90 deg, D/N = 0 */
        }

        *t_cs = 0; /* DUMPTFF1: perigee of parab/hyp   */

        return (AGC_STATUS_OK);
    }

    agc_dp_t zra = agc_dp_mul(ip, num, tff->tffrtalf);        /* TFFXTEST */

    zra = agc_navaux_dp_shift(ip, zra, 3 - tff->na); /* SR* 0-3,2 */
    agc_dp_div_ovf_only(ip, zra, absden); /* Z sqrt(ALFA): only OVFIND matters */

    if (agc_interp_test_ovf(ip))
    { /* BOV TFFELL                       */
        agc_dp_t dn = agc_dp_div(ip, den, agc_dp_shl(ip, num, 2)); /* D/N   */

        return (tff_el1(ip, tff, dn, q2, t_cs));
    }

    /* General conic (TFFXTEST continuation, p. 1383-1384), alfa Z Z < 1:
     * X = ALFA Z^2, TFF = (RTERM + RN - 2 Z^2 T(X)) Z / sqrt(MU).            */
    agc_dp_t z   = agc_dp_div(ip, num, den); /* Z (-13)  */
    agc_dp_t zsq = agc_dp_sq(ip, z);         /* (-26)    */

    tff->tfftem =
        agc_dp_sign(agc_dp_shl(ip, agc_dp_mul(ip, zsq, tff->tffnp), 5), z);
    tff->tffx =
        agc_navaux_dp_shift(ip, agc_dp_mul(ip, zsq, tff->tffalfa), tff->nr);
    agc_dp_t tz =
        agc_dp_shr(agc_dp_mul(ip, agc_tff_poly(ip, tff->tffx), zsq), 2);
    agc_dp_t s   = agc_dp_add(ip, agc_dp_sub(ip, tff->rterm, tz), tff->rmag1);
    agc_dp_t tsm = agc_dp_shr(agc_dp_mul(ip, s, z), 3); /* TFF sqrt(MU) */

    if (tsm < 0)
    {                                                       /* BPL ENDTFF   */
        bool gone_past = agc_dp_sign(tsm, tff->tffq1) >= 0; /* SIGN TFFQ1 */

        if (!gone_past && (tff->tff1_alf > 0))
        { /* ellipse      */
            /* Orbital-period correction for a negative TFF on an ellipse:
             * TFF + 2 pi / (ALFA sqrt(ALFA)), result scale 2^45 (E) / 2^42 (M).      */
            if (!AGC_ASSERT(tff->tffrtalf != 0))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            agc_dp_t per =
                agc_dp_div(ip, agc_dp_mul(ip, tff->tff1_alf,
                                          PI_16), tff->tffrtalf);

            per = agc_navaux_dp_shift(ip, per, tff->na - 4);
            per = agc_navaux_dp_shift(ip, per, tff->na - 4);
            per = agc_navaux_dp_shift(ip, per, tff->na);
            tsm = agc_dp_add(ip, per, tsm);
        } /* else NEGTFF  */
    }

    return (tff_end(ip, tff, tsm, t_cs));
}

/*
 * CALCTPER / CALCTFF (TIME_OF_FREE_FALL.agc pp. 1380-1383)
 *   Q2 = -sqrt(RTERM (2 - RTERM ALFA) - LCP)   (0 for CALCTPER)
 *   Q1 = RN.VN / sqrt(MU)
 *   inbound (Q1 < 0): Z = (RTERM - RN) / (Q2 + Q1)
 *   outbound:         Z = (Q2 - Q1) / (2 - ALFA (RTERM + RN))
 */
static agc_status_t tff_calc(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t rterm,
                             bool tper, agc_dp_t *t_cs)
{
    if (!AGC_ASSERT((ip != NULL) && (tff != NULL) && (t_cs != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(rterm) && (tff->nr >= 0) &&
                    (tff->nr <= AGC_DP_BITS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int nr = tff->nr;

    tff->rterm  = rterm;
    tff->nrterm = agc_navaux_dp_shift(ip, rterm, nr); /* SL* 0,1 */
    agc_dp_t two_m = agc_dp_sub(
        ip, AGC_DP_QUARTER,
        agc_dp_mul(ip, tff->nrterm, tff->tffalfa)); /* 2-ALFA RT, 2^3 */
    agc_dp_t rt2  = agc_dp_mul(ip, two_m, tff->nrterm);
    agc_dp_t p    = agc_navaux_dp_shift(ip, tff->tffnp, 6 - nr); /* SR* 0-6,1 */
    agc_dp_t q2sq =
        agc_navaux_dp_shift(ip, agc_dp_add(ip, agc_dp_neg(p), rt2), -nr);

    if (tper)
    {
        q2sq = 0; /* TFFZEROS: Q2 = 0  */
    }

    if (q2sq < 0)
    {
        *t_cs = AGC_TFF_NEARONE; /* MAXTFF1: no conic */

        return (AGC_STATUS_OK);
    }

    agc_dp_t q2 = agc_dp_neg(agc_dp_sqrt(ip, q2sq)); /* Q2 (-16), inbound */

    agc_interp_clear_ovf(ip);                  /* BOVB TCDANZIG     */
    tff->tfftem = q2;
    tff->tffq1  = agc_dp_shl(ip, agc_vec_dot(ip, tff->vone_p, tff->rone), 3);

    if (tff->tffq1 < 0)
    { /* INBOUND           */
        return (tff_z(ip, tff, agc_dp_sub(ip, rterm, tff->rmag1),
                      agc_dp_add(ip, q2, tff->tffq1), q2, t_cs));
    }

    agc_dp_t den =
        agc_dp_sub(ip, two_m, agc_dp_mul(ip, tff->tffalfa, tff->nrmag));

    return (tff_z(ip, tff, agc_dp_sub(ip, q2, tff->tffq1), den, q2, t_cs));
}

/* CALCTFF (p. 1382): time of free fall to radius RTERM (inbound side). */
agc_status_t agc_tff_calctff(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t rterm,
                             agc_dp_t *t_cs)
{
    if (!AGC_ASSERT((ip != NULL) && (tff != NULL) && (t_cs != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(rterm >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (tff_calc(ip, tff, rterm, false, t_cs)); /* CLEAR TFFSW     */
}

/* CALCTPER (p. 1382): time to perigee, RPER from TFFRP/RA. */
agc_status_t agc_tff_calctper(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t rper,
                              agc_dp_t *t_cs)
{
    if (!AGC_ASSERT((ip != NULL) && (tff != NULL) && (t_cs != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(rper >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (tff_calc(ip, tff, rper, true, t_cs)); /* SET TFFSW       */
}
