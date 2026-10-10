/*
 * return_to_earth_conic.c -- P37_P70.agc (Comanche055): the conic part of P37.
 *
 *   INVC109 .. INVC125 (pp. 905-906) unit vectors at TIG (after CSMPREC)
 *   RTE299 tail, RTE317 .. RTE390 (pp. 891-895) the conic iteration:
 *       "INITIALLY A CONIC TRAJECTORY IS DETERMINED" -- RCON is iterated to
 *       the 400,000 ft radius over the Fischer ellipsoid at the conic's
 *       reentry point, and X(T2) to the desired (or V2-computed) reentry
 *       flight path angle; at most 9 passes, alarm 605.
 *   RTE505           (p. 895) entry near perigee or apogee (PHI2)
 *   TMRAD100         (p. 901) TIMERAD caller
 *   RTEDISP          (pp. 902-904) display values and the landing site
 *   AUGEKUGL         (P30-P37.agc pp. 644-647) range and time from 300,000
 *                    ft to the landing site. Local copy: no other lane has
 *                    translated it (DELRSPL, its other user, is a stub).
 *
 * Earth only (P37 alarms 612 otherwise): CONICX1 = 2, the conic subroutines
 * run with X1 = -2 (AGC_CONIC_EARTH).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "agc/core/assert.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/nav/conic.h"
#include "agc/nav/auxiliary.h"
#include "return_to_earth_internal.h"

/* Constants (P37_P70.agc pp. 931-933, words read from the Comanche055 rope). */
static const agc_dp_t ONE_B2   = 67108864;   /* 1RTEB2   10000 00000  1.0 B-2            */
static const agc_dp_t EPC1RTE  = 134172094;  /* EPC1RTE  17775 06676  .99966 B-1 (CFPA)  */
static const agc_dp_t EPC2RTE  = 50;         /* EPC2RTE  00000 00062  100. B-29 m        */
static const agc_dp_t EPC3RTE  = 268435;     /* EPC3RTE  00020 14223  .001 (X(T2))       */
static const agc_dp_t K1RTE    = 3500000;    /* K1RTE    00325 23740  7. E6 B-29 m       */
static const agc_dp_t K2RTE    = 3247500;    /* K2RTE    00306 06614  6495000. B-29 m    */
static const agc_dp_t K3RTE    = -16387985;  /* K3RTE    76027 70156  -.06105 (X(T2))    */
static const agc_dp_t K4RTE    = -28059558;  /* K4RTE    74517 54131  -.10453 (X(T2))    */
static const agc_dp_t C4RTE    = -17466608;  /* C4RTE    75725 75417  -6.986643 E7 B-30 m */
static const agc_dp_t E3RTE    = 60960;      /* E3RTE    00003 27040  121920. B-29 m (400 kft) */
static const agc_dp_t M30480   = 15240;      /* 30480RTE 00000 35610  30480. B-29 m (100 kft) */
static const agc_sp_t SGNRDOT_DOWN = -0x2000;/* AXC,2 OCT 20000: SGNRDOT = -.5 (descending) */

/* RTE317 POLY: MAMAX2 = C0 + C1 R + C2 R^2 + C3 R^3, B31 (then SL1 -> B30). */
static const agc_dp_t MAMAX2_POLY[4] = {
    22625054,      /* 02544 35436  181000434. B-31       */
    101190198,     /* 14040 05066  1.50785145 B-2        */
    -234184679,    /* 44052 60030  -6.49993057 E-9 B27   */
    188967471      /* 26415 25057  9.76938926 E-18 B56   */
};
/* RTE369 POLY: X(T2), = D1 + D2 V2 + D3 V2^2 + D4 V2^3 (D1 = RTED1 pad load). */
static const agc_dp_t VGAMMA_POLY[4] = {
    0,             /* 00000 00000  0                     */
    -209425917,    /* 47021 65002  -4.8760771 E-2 B4     */
    249696210,     /* 35610 07722  4.5419476 E-4 B11     */
    -100751681     /* 63772 63276  -1.4317675 E-6 B18    */
};

/* AUGEKUGL table (P30-P37.agc p. 646; Comanche055 bank 32 words).
 * K = (V - VREF) Y + C, V m/cs B7; K1 revs^2 x 360 (range x gamma), K2 revs. */
typedef struct rte_kfit {
    agc_dp_t vref;        /* V(...)  reference speed, m/cs B7           */
    agc_dp_t slope;       /* YK1K2.. slope, B7* per m/cs                */
    agc_dp_t c;           /* CK1K2.. value at VREF                      */
} rte_kfit_t;
static const rte_kfit_t K1_BELOW_21K = { 2556848,   0,        189866   }; /* V(400), 0, 5500 */
static const rte_kfit_t K1_21K_28K   = { 178979340, -642218,  82850    }; /* V(28K), -.443, 2400 */
static const rte_kfit_t K1_ABOVE_28K = { 19176358,  0,        82850    }; /* V(3K), 0, 2400 */
static const rte_kfit_t K2_BELOW_24K = { 153410863, 38359106, -2386093 }; /* V(24K), .001225, -3.2 */
static const rte_kfit_t K2_24K_28_4K = { 178979340, 32879233, 745654   }; /* 28K, .00105, 1 */
static const rte_kfit_t K2_ABOVE_28_4K = { 204547817, 8924363, 1789570 }; /* V(32K), .000285, 2.4 */
static const agc_dp_t V21K     = 134217728;  /* V(21K)   64.000 B-7 m/cs  */
static const agc_dp_t V3K      = 19176358;   /* V(3K)    9.144 B-7        */
static const agc_dp_t V4K      = 25568477;   /* V(4K)    12.192 B-7       */
static const agc_dp_t V400     = 2556848;    /* V(400)   1.2192 B-7       */
static const agc_dp_t V26K     = 166195102;  /* V(26K)   79.248 B-7       */
static const agc_dp_t MAXPHIC  = 24855239;   /* MAXPHIC  .09259298 rev (2000 NM) */
static const agc_dp_t TLESS26  = 445427;     /* TLESS26  5.70146688 E7 B-35 (8660 PHI/V) */
static const agc_dp_t TGR26CON = 720000;     /* TGR26CON 7.2 E5 B-28 cs/rev (PHI/3) */

/* INVC100 after CSMPREC .. INVC125 (P37_P70.agc pp. 905-906): R(T1)/ V(T1)/
 * T1 from RATT VATT TAT; alarm 612 if not earth-centred (P(T1) = X2 != 0);
 * UR1/, R(T1), UV1/, CFPA = UV1/.UR1/ (B1); the orbit normal N/ (UR1 x UV1,
 * or (0,0,1) when nearly rectilinear), turned north (RETROFLG set if it
 * had to be flipped); UH/ = UNIT(N/ x UR1/).                               */
agc_status_t rte_initial_vectors(agc_interp_t *ip, rte_erasable_t *rs,
                                 agc_vec_t ratt, agc_vec_t vatt, agc_dp_t tat, int x2)
{
    agc_vec_t n;

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT((x2 == 0) || (x2 == 2))) {
        return AGC_STATUS_BAD_ARG;
    }
    rs->r_t1v = ratt;
    rs->p_t1 = (agc_sp_t)x2;                                      /* SXA,2 P(T1)   */
    rs->v_t1v = vatt;
    rs->t1 = tat;
    if (rs->p_t1 != 0) {
        rs->alarm_code = RTE_ALARM_MOON_SPHERE;                   /* INVC107: 612  */
        return AGC_STATUS_ALARM;
    }
    rs->ur1 = agc_vec_unit(ip, rs->r_t1v, &rs->r_t1, NULL);       /* UR1/, 36D     */
    rs->uv1 = agc_vec_unit(ip, rs->v_t1v, NULL, NULL);            /* UV1/          */
    rs->cfpa = agc_dp_shl(ip, agc_vec_dot(ip, rs->uv1, rs->ur1), 1);
    if (agc_dp_sub(ip, agc_dp_abs(rs->cfpa), EPC1RTE) >= 0) {
        n = agc_vec_make(0, 0, ONE_B2);                           /* N/ = (0,0,1)  */
    } else {
        n = agc_vec_cross(ip, rs->ur1, rs->uv1);                  /* INVC115       */
    }
    agc_flag_clear(AGC_FLAG_RETROFLG);                            /* INVC120       */
    if (n.c[2] < 0) {
        n = agc_vec_neg(n);                                       /* RETROGRADE    */
        agc_flag_set(AGC_FLAG_RETROFLG);
    }
    rs->uh = agc_vec_unit(ip, agc_vec_cross(ip, n, rs->ur1), NULL, NULL); /* INVC125 */
    (void)AGC_ASSERT(rs->r_t1 > 0);
    return AGC_STATUS_OK;
}

/* RTE299 after INVC100 (p. 892): CLEAR SLOWFLG; a negative RTEDVD asks for
 * a transearth slow-down, accepted (SLOWFLG) only beyond K1RTE (7000 km).
 * MAMAX1 = C4RTE.                                                         */
agc_status_t rte_conic_setup(agc_interp_t *ip, rte_erasable_t *rs)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->r_t1 > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    rs->mamax1 = C4RTE;                                           /* STCALL MAMAX1 */
    agc_flag_clear(AGC_FLAG_SLOWFLG);
    if (rs->rtedvd < 0) {
        rs->rtedvd = agc_dp_abs(rs->rtedvd);
        if (agc_dp_sub(ip, rs->r_t1, K1RTE) >= 0) {
            agc_flag_set(AGC_FLAG_SLOWFLG);
        }
    }
    return AGC_STATUS_OK;
}

/* TMRAD100 (p. 901): TIMERAD from RVEC, VVEC to RDESIRED on the descending
 * leg (SGNRDOT -.5), RVSW clear: R(T2)/, V(T2)/ and T12 (also T, 30D).
 * A TIMERAD failure is POODOO 00607 inside the conic subroutines.         */
agc_status_t rte_time_radius(agc_interp_t *ip, rte_erasable_t *rs, agc_vec_t rvec,
                             agc_vec_t vvec, agc_dp_t rdesired)
{
    agc_timerad_in_t     in;
    agc_conic_time_out_t out;

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rdesired > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_flag_clear(AGC_FLAG_RVSW);
    in.rvec = rvec;
    in.vvec = vvec;
    in.rdesired = rdesired;
    in.sgnrdot = SGNRDOT_DOWN;
    in.rvsw = false;
    in.body = AGC_CONIC_EARTH;                                    /* LXC,1 CONICX1 */
    agc_status_t st = agc_conic_timerad(ip, &in, &out);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rs->v_t2v = out.vnew;                                         /* V(T2)/        */
    rs->r_t2v = out.rnew;                                         /* R(T2)/        */
    rs->t12 = out.t;                                              /* T12           */
    rs->conic_t = out.t;                                          /* T (30D)       */
    return AGC_STATUS_OK;
}

/* RTE317 .. RTE350 (pp. 892-893): MAMAX2 from R(T1), NN1A = -9, RCON = K2,
 * and the first X(T2): COT(90 deg - GAMMA) of the input angle, else K3RTE
 * (R(T1) < K1RTE) or K4RTE.                                               */
static agc_status_t conic_start(agc_interp_t *ip, rte_erasable_t *rs)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->r_t1 > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    rs->mamax2 = agc_dp_shl(ip, agc_dp_poly(rs->r_t1, MAMAX2_POLY, 3), 1);
    rs->nn1a = -RTE_CONIC_PASSES;                                 /* M9RTEB28      */
    rs->rcon = K2RTE;                                             /* RTE320        */
    if (rs->rtegam2d != 0) {
        agc_dp_t a = agc_dp_sub(ip, ONE_B2, rs->rtegam2d);        /* 1RTEB2 BDSU   */
        rs->x_t2 = agc_dp_div(ip, agc_dp_cos(ip, a), agc_dp_sin(ip, a)); /* BDDV  */
    } else if (agc_dp_sub(ip, rs->r_t1, K1RTE) < 0) {             /* RTE340        */
        rs->x_t2 = K3RTE;                                         /* RTE350        */
    } else {
        rs->x_t2 = K4RTE;
    }
    return AGC_STATUS_OK;
}

/* RTE373 .. RTE390 (pp. 893-895): one correction of RCON and X(T2).
 * NOT_FOUND: converged (P37E). xd is the X(T2) wanted for this conic.     */
static agc_status_t conic_correct(agc_interp_t *ip, rte_erasable_t *rs, agc_dp_t xd)
{
    int n1 = 0;
    agc_dp_t dx;

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->nn1a < 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t xerr = agc_dp_sub(ip, xd, rs->x_t2);                 /* X(T2)ERR 00D  */
    agc_vec_t ur2 = agc_vec_unit(ip, rs->r_t2v, NULL, NULL);      /* ALPHAV        */
    agc_dp_t rcon1 = agc_dp_add(ip, agc_latlong_geterad(ip, ur2.c[2]), E3RTE);
    if ((agc_dp_sub(ip, agc_dp_abs(agc_dp_sub(ip, rcon1, rs->rcon)), EPC2RTE) < 0) &&
        (agc_dp_sub(ip, agc_dp_abs(xerr), EPC3RTE) < 0)) {
        return AGC_STATUS_NOT_FOUND;                              /* RTE374: P37E  */
    }
    if (rs->nn1a + 1 >= 0) {                                      /* RTE375        */
        rs->alarm_code = RTE_ALARM_ITERATIONS;
        return AGC_STATUS_ALARM;
    }
    rs->nn1a += 1;                                                /* RTE380        */
    if (rs->nn1a == 1 - RTE_CONIC_PASSES) {
        dx = xerr;                                                /* RTE385        */
    } else {
        agc_dp_t z1 = agc_dp_norm(agc_dp_sub(ip, xerr, rs->drcon), &n1);
        agc_dp_t q = agc_dp_div(ip, agc_dp_sub(ip, rs->rpre_prev, rs->x_t2), z1);
        dx = agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(q, xerr), n1)); /* SL* 0,1 */
    }
    rs->rcon = rcon1;                                             /* RTE390        */
    if (agc_interp_test_ovf(ip)) {
        return AGC_STATUS_OK;                                     /* BOV RTE360    */
    }
    rs->drcon = xerr;                                             /* X(T2)ERR,     */
    rs->rpre_prev = rs->x_t2;                                     /* X(T2)PRI      */
    rs->x_t2 = agc_dp_add(ip, dx, rs->x_t2);
    return AGC_STATUS_OK;
}

/* RTE317 .. RTE390 (pp. 892-895): the conic iteration. Each pass: V2T100,
 * TMRAD100 to RCON, T2 = T1 + T12, the wanted X(T2) (the input angle, or
 * RTE369: POLY in V(T2) + RTED1, SL3), then conic_correct. Returns OK at
 * P37E, ALARM (605) or the V2T100/TIMERAD abort.                          */
agc_status_t rte_conic_search(agc_interp_t *ip, rte_erasable_t *rs, agc_dp_t rted1)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(rs != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = conic_start(ip, rs);                        /* RTE317        */
    for (int pass = 0; (st == AGC_STATUS_OK) && (pass <= RTE_CONIC_PASSES); pass++) {
        st = rte_post_impulse_velocity(ip, rs);                   /* RTE360        */
        if (st == AGC_STATUS_OK) {                                /* RTE367        */
            st = rte_time_radius(ip, rs, rs->r_t1v, rs->v2_t1v, rs->rcon);
        }
        if (st != AGC_STATUS_OK) {
            return st;
        }
        rs->t2 = agc_dp_add(ip, rs->t12, rs->t1);
        agc_dp_t xd = rs->x_t2;                                   /* RTE372        */
        if (rs->rtegam2d == 0) {                                  /* RTE369        */
            agc_dp_t p = agc_dp_poly(agc_vec_abval(ip, rs->v_t2v), VGAMMA_POLY, 3);
            xd = agc_dp_shl(ip, agc_dp_add(ip, p, rted1), 3);
        }
        st = conic_correct(ip, rs, xd);
    }
    if (st == AGC_STATUS_NOT_FOUND) {
        return AGC_STATUS_OK;                                     /* P37E          */
    }
    if (st == AGC_STATUS_OK) {
        rs->alarm_code = RTE_ALARM_ITERATIONS;   /* cannot happen: NN1A bounds it */
        st = AGC_STATUS_ALARM;
    }
    return st;
}

/* RTE505 .. RTE515 (p. 895): RCON > PCON BETA1 -> entry near apogee
 * (PHI2 = +1), else near perigee (PHI2 = -1), B2.                         */
agc_status_t rte_entry_side(agc_interp_t *ip, rte_erasable_t *rs)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->rcon > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t d = agc_dp_sub(ip, rs->rcon, agc_dp_mul_trunc(rs->pcon, rs->beta1));
    rs->phi2 = (d > 0) ? ONE_B2 : agc_dp_neg(ONE_B2);
    return AGC_STATUS_OK;
}

/* AUGEKUGL (P30-P37.agc pp. 644-647, local copy): from the speed V (m/cs B7)
 * and |GAMMA| (revs) at 300,000 ft, the range to the landing site
 *   PHIE = K1 / (|GAMMA| - K2), limited to MAXPHIC (2000 NM), revs,
 * and the time TE = PHIE TLESS26 / V (V < 26,000 fps) or PHIE TGR26CON,
 * cs B28. K1 and K2 are linear fits in V selected by speed band.
 * DEVIATION: the AGC walks the K1/K2 table with index registers (X1, S1,
 * TIX); here the bands select the same table rows explicitly. The speed is
 * compared by the same chain of DSUs (21K, +3K, +4K, +400 fps).           */
agc_status_t rte_augekugl(agc_interp_t *ip, agc_dp_t v, agc_dp_t abs_gamma,
                          agc_dp_t *phie, agc_dp_t *te)
{
    const rte_kfit_t *k1 = &K1_ABOVE_28K;
    const rte_kfit_t *k2 = &K2_ABOVE_28_4K;

    if (!AGC_ASSERT((ip != NULL) && (phie != NULL) && (te != NULL)) || !AGC_ASSERT(v > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_dp_t d21 = agc_dp_sub(ip, v, V21K);                 /* DSU V(21K)    */
    const agc_dp_t d24 = agc_dp_sub(ip, d21, V3K);                /* DSU V(3K)     */
    const agc_dp_t d28 = agc_dp_sub(ip, d24, V4K);                /* DSU V(4K)     */
    const agc_dp_t d284 = agc_dp_sub(ip, d28, V400);              /* DSU V(400)    */
    if (d21 < 0) {
        k1 = &K1_BELOW_21K;
        k2 = &K2_BELOW_24K;
    } else if (d24 < 0) {
        k1 = &K1_21K_28K;
        k2 = &K2_BELOW_24K;
    } else if (d28 < 0) {
        k1 = &K1_21K_28K;
        k2 = &K2_24K_28_4K;
    } else if (d284 < 0) {
        k2 = &K2_24K_28_4K;
    }
    agc_dp_t c1 = agc_dp_add(ip, agc_dp_mul_trunc(agc_dp_sub(ip, v, k1->vref), k1->slope), k1->c);
    agc_dp_t c2 = agc_dp_add(ip, agc_dp_mul_trunc(agc_dp_sub(ip, v, k2->vref), k2->slope), k2->c);
    *phie = agc_dp_div(ip, c1, agc_dp_sub(ip, abs_gamma, c2));    /* DSU BDDV      */
    if (agc_interp_test_ovf(ip) || (*phie < 0) || (agc_dp_sub(ip, *phie, MAXPHIC) >= 0)) {
        *phie = MAXPHIC;                                          /* MAXPHI        */
    }
    if (agc_dp_sub(ip, v, V26K) >= 0) {                           /* PHICALC       */
        *te = agc_dp_mul_trunc(TGR26CON, *phie);                  /* TGR26         */
    } else {
        *te = agc_dp_mul_trunc(agc_dp_div(ip, TLESS26, v), *phie);
    }
    return AGC_STATUS_OK;
}

/* RTEDISP .. RTD18 (pp. 902-903): VPRED, T3TOT4, GAMMAEI at R(T2) and the
 * delta V in local vertical coordinates (X forward, Z down; X negated for
 * a retrograde orbit).                                                    */
static agc_status_t display_entry(agc_interp_t *ip, const rte_erasable_t *rs,
                                  agc_dp_t tig, rte_display_t *out)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t uv2 = agc_vec_unit(ip, rs->v_t2v, &out->vpred, NULL); /* VPRED 36D   */
    out->t3tot4 = agc_dp_sub(ip, rs->t2, tig);                    /* T21           */
    agc_vec_t ur2 = agc_vec_unit(ip, rs->r_t2v, NULL, NULL);
    agc_dp_t c = agc_dp_shl(ip, agc_vec_dot(ip, ur2, uv2), 1);    /* DOT SL1       */
    out->gammaei = agc_dp_sub(ip, ONE_B2, agc_dp_acos(ip, c));    /* ARCCOS BDSU   */
    agc_vec_t dvv = agc_vec_sub(ip, rs->v2_t1v, rs->v_t1v);
    agc_dp_t z = agc_dp_neg(agc_vec_dot(ip, dvv, rs->ur1));       /* DOT DCOMP     */
    agc_dp_t x = agc_vec_dot(ip, dvv, rs->uh);
    out->delvlvc = agc_vec_shl(ip, agc_vec_make(x, 0, z), 1);     /* VDEF VSL1     */
    if (agc_flag_test(AGC_FLAG_RETROFLG)) {
        out->delvlvc.c[0] = agc_dp_neg(out->delvlvc.c[0]);        /* RETROGRADE    */
    }
    out->vgdisp = agc_vec_abval(ip, out->delvlvc);                /* RTD18         */
    return AGC_STATUS_OK;
}

/* RTEDISP landing site (pp. 903-904): TMRAD100 down to 300,000 ft (R3, V3,
 * T23), GAMMAE = ARCSIN(UR3.UV3), AUGEKUGL (PHIE, TE), T(LS) = T2 + T23 +
 * TE; PHIE replaced by the P37RANGE pad load if non-zero (PCR 261);
 * ALPHAV = UR3 COS(PHIE) + UH3 SIN(PHIE), UH3 = UNIT(UR3 X UV3 X UR3);
 * LAT-LONG (ERADFLAG, LUNAFLAG clear) at T(LS).                           */
static agc_status_t display_landing_site(agc_interp_t *ip, rte_erasable_t *rs,
                                         agc_sp_t range_override, rte_display_t *out)
{
    agc_dp_t phie = 0;
    agc_dp_t te = 0;
    agc_dp_t v3 = 0;

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t rdes = agc_dp_sub(ip, agc_vec_abval(ip, rs->r_t2v), M30480);
    agc_status_t st = rte_time_radius(ip, rs, rs->r_t2v, rs->v_t2v, rdes);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_vec_t ur3 = agc_vec_unit(ip, rs->r_t2v, NULL, NULL);
    agc_vec_t uv3 = agc_vec_unit(ip, rs->v_t2v, &v3, NULL);
    agc_dp_t gammae = agc_dp_asin(ip, agc_dp_shl(ip, agc_vec_dot(ip, ur3, uv3), 1));
    st = rte_augekugl(ip, v3, agc_dp_abs(gammae), &phie, &te);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_dp_t tls = agc_dp_add(ip, agc_dp_add(ip, te, rs->t12), rs->t2); /* T(LS)  */
    if (range_override != 0) {
        phie = agc_dp_from_sp(range_override);                    /* P37RANGE      */
    }
    agc_vec_t uh3 = agc_vec_cross(ip, agc_vec_cross(ip, ur3, uv3), ur3);
    uh3 = agc_vec_unit(ip, uh3, NULL, NULL);
    agc_vec_t alphav = agc_vec_add(ip, agc_vec_scale(ip, ur3, agc_dp_cos(ip, phie)),
                                   agc_vec_scale(ip, uh3, agc_dp_sin(ip, phie)));
    agc_flag_clear(AGC_FLAG_ERADFLAG);
    agc_flag_clear(AGC_FLAG_LUNAFLAG);
    agc_navaux_padload_t pad;
    agc_latlong_t ll;
    const agc_latlong_opt_t opt = { false, false };
    st = agc_navaux_get_padload(&pad);
    if (st == AGC_STATUS_OK) {
        st = agc_latlong_from_r(ip, &pad, opt, alphav, tls, &ll); /* LAT-LONG      */
    }
    if (st == AGC_STATUS_OK) {
        out->lat_spl = ll.lat;
        out->lng_spl = ll.lon;
    }
    return st;
}

/* RTEDISP (P37_P70.agc pp. 902-904): "OUTPUT FOR DISPLAY IS CONVERTED TO
 * PROPER UNITS ... LANDING SITE COMPUTATION ... IS INCLUDED". Leaves
 * R(T2)/ V(T2)/ T12 at 300,000 ft, as the AGC does.                      */
agc_status_t rte_display_values(agc_interp_t *ip, rte_erasable_t *rs, agc_dp_t tig,
                                agc_sp_t range_override, rte_display_t *out)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = display_entry(ip, rs, tig, out);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return display_landing_site(ip, rs, range_override, out);
}
