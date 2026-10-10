/*
 * dps_burn_rasteer.c -- RASTEER1, the Luminary aimpoint steering (P40-P47.agc,
 * Luminary099 pp. 776-780). For a Lambert-targeted burn (XDELVFLG clear) it
 * regenerates the required velocity VR at RN for reaching RTARG at TPASS4
 * on the conic whose mu/a INITVEL last found, without iterating:
 *
 *   s = (r1 + r2 + c)/2, c = |RTARG - RN|, IC = UNIT(RTARG - RN), IR = UNIT/R/
 *   A = GEOMSGN sqrt((s - r1)(2 mu - (s - c) mu/a) / (c r1))
 *   B = sgn(tm - t) sqrt((s - r2)(2 mu - s mu/a) / (c r1))
 *   VR = A UNIT(IC + IR) + B UNIT(IC - IR)
 *
 * where tm = (s^1.5 / sqrt(2 mu)) (pi/2 -+ (g - sin g cos g)), sin g =
 * sqrt((s - c)/s), is the minimum-energy transfer time that selects the
 * branch of B. Near 180 degrees (NORMSW) UNIT(IC - IR) or UNIT(IC + IR) is
 * replaced by the perpendicular built from UN (180MESS / NEGPROD). Then
 * DELVEET3 = VR - VN1 plus, in the earth sphere, the oblateness term
 * -(mu/R^2)(T - GOBLTIME) UNITGOBL. The formulas were checked against an
 * exact Keplerian orbit (docs/modules/guid-powered.md, Tests written).
 *
 * Scales: Rb = body position scale (2^29 m earth, 2^27 moon, RTX2 = 0 / 2);
 * MU/A 2^14 (earth) / 2^10 (moon) m^2/cs^2, MUASTEER mu 2^42 / 2^36 m^3/cs^2,
 * A and B 2^7 / 2^5 m/cs, VIPRIME and DELVEET3 2^7 m/cs.
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "agc/agc_guid.h"
#include "guid_pending.h"
#include "agc/agc_sysmisc.h"

/* 2PI+3: 2DEC 3.141592653 B-2 (pi at 2^2).                                 */
static const agc_dp_t PI_B2 = 210828714;
/* EARTHMU: 2DEC* -3.986032 E10 B-36* (m^3/cs^2).                            */
static const agc_dp_t EARTHMU = -155704375;

/* The RASTEER1 working cells (the AGC's RMAG, IC, R1C, SS, MUSCALE are
 * kept in agc_guid_state()->lm as well; 30D/32D are locals here).        */
typedef struct rs_work {
    agc_dp_t  c;          /* 30D  |RTARG - RN| at Rb                       */
    agc_dp_t  r1c;        /* R1C  c r1 2^(k1+k2) at Rb^2                   */
    agc_sp_t  muscale;    /* MUSCALE -(k1+k2)                              */
    agc_dp_t  ss;         /* SS   (r1 + r2 + c)/2 at Rb                    */
    agc_dp_t  rmag;       /* RMAG r1 at Rb                                 */
    agc_vec_t ic;         /* IC   half unit                                */
    agc_dp_t  mu_a, muasteer, rtmag;
    agc_sp_t  geomsgn;
} rs_work_t;

/* SL* n: left for n > 0 (overflow to OVFIND), truncating right for n < 0. */
static agc_dp_t dp_shift(agc_interp_t *ip, agc_dp_t v, int n)
{
    (void)AGC_ASSERT(n > -64 && n < 64);
    if (n >= 0) {
        return agc_dp_shl(ip, v, n);
    }
    return agc_dp_shr(v, -n);
}

/* RASTEER1 .. SS (p.776-777): RMAG, IC, 30D, R1C with its normalization
 * MUSCALE, and SS. NORMUNX1 shifts a very short C left 13 (X1).           */
static agc_status_t rs_geometry(agc_interp_t *ip, rs_work_t *w)
{
    const agc_rendz_t *rz = agc_rendz_erasable();
    int rtx2 = rz->body.rtx2;
    if (!AGC_ASSERT(rtx2 == 0 || rtx2 == 2)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t rn = agc_erase_rn();
    w->rmag = dp_shift(ip, agc_vec_abval(ip, rn), rtx2);      /* LXC,2 SL*      */
    int n = 0;
    agc_dp_t cabs = 0;
    w->ic = agc_rtb_normunit(ip, agc_vec_sub(ip, rz->rtarg, rn), &n, &cabs, NULL);
    w->c = dp_shift(ip, cabs, rtx2 - n);                      /* XAD,2 SL* X1   */
    int k1 = 0;
    int k2 = 0;
    agc_dp_t cn = agc_dp_norm(w->c, &k1);                     /* NORM X2        */
    agc_tp_t r1c = agc_tp_norm(agc_dp_mul_tp(cn, w->rmag), &k2); /* DMP NORM X1 */
    w->r1c = agc_tp_to_dp(r1c);
    w->muscale = (agc_sp_t)(-(k1 + k2));                      /* SXA,2 MUSCALE  */
    agc_dp_t sum = agc_dp_add(ip, agc_dp_shr(w->rtmag, 1), agc_dp_shr(w->rmag, 1));
    w->ss = agc_dp_add(ip, sum, agc_dp_shr(w->c, 1));         /* STORE SS       */
    if (!AGC_ASSERT(w->r1c != 0 && w->ss > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* One velocity component (p.777-778): with L = SS - 30D (for A) or SS
 * (for B) and rk = RMAG or RTMAG:
 *   sqrt( (SS - rk) (MUASTEER - L MU/A) / R1C ), rescaled by MUSCALE.     */
static agc_dp_t rs_speed(agc_interp_t *ip, const rs_work_t *w, agc_dp_t len, agc_dp_t rk)
{
    agc_dp_t energy = agc_dp_sub(ip, w->muasteer, agc_dp_mul_trunc(len, w->mu_a)); /* BDSU */
    int k = 0;
    agc_dp_t d = agc_dp_norm(agc_dp_sub(ip, w->ss, rk), &k);  /* NORM X1        */
    (void)AGC_ASSERT(d > 0);
    d = agc_dp_shr(d, 1);                                     /* SR1            */
    agc_dp_t q = agc_dp_div(ip, d, w->r1c);                   /* DDV R1C        */
    agc_tp_t p = agc_dp_mul_tp(q, energy);                    /* DMP (pushed)   */
    p = agc_tp_shift(ip, p, 1 - (int)w->muscale - k);         /* XSU,2 SL* 1,2  */
    return agc_tp_sqrt(ip, p);
}

/* The minimum-energy transfer time less the time to go to the aimpoint
 * (p.778): tm - (TPASS4 - PIPTIME), cs 2^28. Its sign is that of B.       */
static agc_dp_t rs_time_error(agc_interp_t *ip, const rs_work_t *w)
{
    const agc_rendz_t *rz = agc_rendz_erasable();
    agc_dp_t x = agc_dp_sqrt(ip, agc_dp_div(ip, agc_dp_sub(ip, w->ss, w->c), w->ss));
    agc_dp_t g = agc_dp_asin(ip, agc_dp_shr(x, 1));           /* SR1 ASIN, rev  */
    agc_dp_t g8 = agc_dp_mul_trunc(g, PI_B2);                 /* gamma/8 (rad)  */
    agc_dp_t y = agc_dp_sqrt(ip, agc_dp_div(ip, w->c, w->ss));
    (void)agc_interp_test_ovf(ip);                            /* BOV +1         */
    agc_dp_t t = agc_dp_sub(ip, g8, agc_dp_shr(agc_dp_mul_trunc(y, x), 3)); /* BDSU */
    t = agc_dp_sign(t, agc_dp_from_sp(w->geomsgn));
    agc_dp_t q = agc_dp_sub(ip, agc_dp_shr(PI_B2, 2), t);     /* SR2 DSU        */
    q = agc_dp_mul_trunc(q, w->ss);                           /* DMP SS         */
    agc_dp_t num = agc_dp_mul_trunc(agc_dp_sqrt(ip, agc_dp_shr(w->ss, 3)), q);
    agc_dp_t den = agc_dp_sqrt(ip, agc_dp_shl(ip, w->muasteer, 3)); /* SL3 SQRT */
    (void)AGC_ASSERT(den > 0);
    agc_dp_t tm = agc_dp_div(ip, num, den);                   /* BDDV           */
    return agc_dp_add(ip, agc_dp_sub(ip, tm, rz->tpass4), agc_erase_piptime());
}

/* UNIT(sign(GEOMSGN) (v x UN)) for 180MESS / NEGPROD.                      */
static agc_vec_t rs_normal(agc_interp_t *ip, agc_vec_t v, agc_sp_t geomsgn)
{
    agc_vec_t n = agc_vec_cross(ip, v, agc_erase_un());       /* VXV UN         */
    if (geomsgn < 0) {
        n = agc_vec_neg(n);                                   /* SIGN GEOMSGN   */
    }
    (void)AGC_ASSERT(geomsgn != 0);
    return agc_vec_unit(ip, n, NULL, NULL);
}

/* VR (p.778-779): the main formula, or 180MESS (IC.IR >= 0: replace
 * UNIT(IC - IR) by UNIT(sign(-u+ x UN))) or NEGPROD (IC.IR < 0: replace
 * UNIT(IC + IR) by UNIT(sign(u- x UN))). GETVRVG2 rescales to 2^7.      */
static agc_vec_t rs_vr(agc_interp_t *ip, const rs_work_t *w, agc_dp_t a, agc_dp_t b)
{
    agc_vec_t ir = agc_guid_state()->avg.unitr;               /* UNIT/R/        */
    agc_vec_t vr;
    if (!agc_flag_test(AGC_FLAG_NORMSW)) {
        agc_vec_t vb = agc_vec_scale(ip, agc_vec_unit(ip, agc_vec_sub(ip, w->ic, ir),
                                                      NULL, NULL), b);
        agc_vec_t up = agc_vec_unit(ip, agc_vec_add(ip, w->ic, ir), NULL, NULL);
        vr = agc_vec_add(ip, agc_vec_scale(ip, up, a), vb);   /* GETVRVG1       */
    } else if (agc_vec_dot(ip, w->ic, ir) >= 0) {             /* 180MESS        */
        agc_vec_t up = agc_vec_unit(ip, agc_vec_add(ip, agc_vec_shr(ir, 1),
                                                    agc_vec_shr(w->ic, 1)), NULL, NULL);
        agc_vec_t vb = agc_vec_scale(ip, rs_normal(ip, agc_vec_neg(up), w->geomsgn), b);
        vr = agc_vec_add(ip, agc_vec_scale(ip, up, a), vb);
    } else {                                                  /* NEGPROD        */
        agc_vec_t um = agc_vec_unit(ip, agc_vec_sub(ip, agc_vec_shr(w->ic, 1),
                                                    agc_vec_shr(ir, 1)), NULL, NULL);
        agc_vec_t va = agc_vec_scale(ip, rs_normal(ip, um, w->geomsgn), a);
        vr = agc_vec_add(ip, agc_vec_scale(ip, um, b), va);
    }
    int n = (int)agc_rendz_erasable()->body.rtx2 - 1;              /* VSR* 0 -1,2    */
    return (n < 0) ? agc_vec_shl(ip, vr, -n) : agc_vec_shr(vr, n);
}

/* ASTREND .. NOGOBL (p.779-780): DELVEET3 = VR - VN1, plus in the earth
 * sphere (RTX2 = 0) G(OBL) = -(MU/R^2) UNITGOBL (PIPTIME - GOBLTIME).     */
static agc_vec_t rs_vg(agc_interp_t *ip, agc_vec_t vr)
{
    const agc_guid_t *g = agc_guid_state();
    agc_vec_t vg = agc_vec_sub(ip, vr, g->avg.vn1);           /* VSU VN1        */
    if (agc_rendz_erasable()->body.rtx2 != 0) {
        return vg;                                            /* NOGOBL         */
    }
    agc_dp_t rsq = 0;
    (void)agc_vec_unit(ip, agc_erase_rn(), NULL, &rsq);       /* 34D = |RN|^2   */
    (void)AGC_ASSERT(rsq > 0);
    agc_dp_t dt = agc_dp_sub(ip, agc_erase_piptime(), g->lm.gobltime);
    agc_dp_t k = agc_dp_div(ip, agc_dp_mul_trunc(dt, EARTHMU), rsq); /* 2^6     */
    return agc_vec_add(ip, agc_vec_scale(ip, g->avg.gobl1_2, k), vg);
}

/* RASTEER1 (Luminary p.776-780): VIPRIME and DELVEET3 for S40.8, which
 * continues at VGAIN* with DELVEET3 (agc_lm_s40_8).                      */
agc_status_t agc_lm_rasteer1(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_guid_lm_t *lm = &agc_guid_state()->lm;
    rs_work_t w = { 0 };
    w.mu_a = rz->mu_a;
    w.muasteer = rz->muasteer;
    w.rtmag = rz->rtmag;
    w.geomsgn = agc_erase_geomsgn();
    if (!AGC_ASSERT(w.muasteer > 0 && w.geomsgn != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = rs_geometry(ip, &w);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_dp_t a = rs_speed(ip, &w, agc_dp_sub(ip, w.ss, w.c), w.rmag);
    a = agc_dp_sign(a, agc_dp_from_sp(w.geomsgn));            /* STORE 32D      */
    agc_dp_t b = rs_speed(ip, &w, w.ss, w.rtmag);             /* -B (NO SIGN)   */
    b = agc_dp_sign(b, rs_time_error(ip, &w));                /* B WITH SIGN    */
    lm->rmag = w.rmag;
    lm->ic = w.ic;
    lm->r1c = w.r1c;
    lm->ss = w.ss;
    lm->muscale = w.muscale;
    rz->viprime = rs_vr(ip, &w, a, b);                        /* STORE VIPRIME  */
    rz->delveet3 = rs_vg(ip, rz->viprime);                    /* STORE DELVEET3 */
    return AGC_STATUS_OK;
}
