/*
 * maneuver_calc.c -- KALCMANU set-up mathematics: direction cosine
 * matrices from CDU angles and back, the incremental rotation matrix DEL,
 * and the maneuver axis/angle computation of KALCMAN3.
 *
 * Source: Comanche055 ANGLFIND.agc pp.399-411 and Luminary099
 *         ATTITUDE_MANEUVER_ROUTINE.agc pp.351-363 (diffed).  The
 *         mathematics is the same in both; differences:
 *   * LM KALCMAN3 first refuses final attitudes with |CPSI| >= 70 deg
 *     (TOOBADF, alarm 00401).
 *   * MXM3/TRNSPSPD: CM forms M1 x (columns of M2) then transposes, LM
 *     forms (rows of M1) x M2; both give M1 M2 (agc_mat_mul).
 *   * LM calls DEL "KEL".
 *
 * Scaling: S/C-to-SM matrices (MIS, MFS) are scaled 2 (elements / 2);
 * MFI = TMIS MFS is scaled 4; DEL is scaled 1; angles are revolutions
 * (scale 2^0, "scaled 2 pi" in the AGC comments); COF is half-unit.
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "internal.h"
#include "pending.h"

/* MINANG = 0.25 deg and MAXANG = 170 deg, in revolutions.  Luminary has
 * them as 2DEC.  Comanche writes single-word DEC constants, so DSU reads
 * MINANG with MAXANG as its low word and MAXANG with SD (DEC .433015):
 *   CM MINANG = 11, 7737  (.00069375, .472222)
 *   CM MAXANG = 7737, 7095 (.472222, .433015)                            */
#define MINANG_LM ((agc_dp_t)186227)     /* 2DEC 0.00069375               */
#define MAXANG_LM ((agc_dp_t)126761187)  /* 2DEC 0.472222222              */
#define MINANG_CM ((agc_dp_t)((11L << 14) + 7737L))
#define MAXANG_CM ((agc_dp_t)((7737L << 14) + 7095L))
/* LOCKANGL DEC .388889 = 70 deg at half-revolution scale.  DSU reads two
 * words, so the low half is the next word in fixed memory: the READCDUK
 * instruction CA CDUZ (octal 30034).                                     */
#define LOCKANGL ((agc_dp_t)((6372L << 14) + 030034L))
#define CDU_MASK 077777u

static bool vec_ok(agc_vec_t v)
{
    bool ok = true;
    for (int i = 0; i < AGC_VEC_N; i++) {
        ok = ok && v.c[i] >= AGC_DP_MIN && v.c[i] <= AGC_DP_MAX;
    }
    return ok;
}

/* BOVB SIGNMPAC: on overflow replace MPAC by DPPOSMAX with its sign.     */
static agc_dp_t bovb_signmpac(agc_interp_t *ip, agc_dp_t v)
{
    (void)AGC_ASSERT(ip != NULL);
    if (agc_interp_test_ovf(ip)) {
        v = agc_rtb_signmpac(v);
    }
    (void)AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX);
    return v;
}

/* CDUTODCM (ANGLFIND p.405-407): S/C to SM direction cosines from the CDU
 * angles PHI = OGA (X), THETA = IGA (Y), PSI = MGA (Z); scale 2.         */
agc_status_t agc_kalc_cdutodcm(agc_interp_t *ip, const uint16_t cdu[3], agc_mat_t *m)
{
    if (!AGC_ASSERT(ip != NULL && cdu != NULL && m != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t s[3];
    agc_dp_t c[3];
    for (int i = 0; i < 3; i++) {                  /* LOOPSIN: X, Y, Z    */
        if (!AGC_ASSERT(cdu[i] <= CDU_MASK)) {
            return AGC_STATUS_BAD_ARG;
        }
        agc_dp_t a = agc_rtb_cdulogic(cdu[i]);
        s[i] = agc_dp_sin(ip, a);                  /* sin/2               */
        c[i] = agc_dp_cos(ip, a);                  /* cos/2               */
    }
    const agc_dp_t sph = s[0], cph = c[0], sth = s[1], cth = c[1];
    const agc_dp_t sps = s[2], cps = c[2];
    agc_dp_t pd6 = agc_dp_mul(ip, sth, sph);       /* sin(th)sin(ph) /4   */
    agc_dp_t pd7 = agc_dp_mul(ip, cph, sth);       /* cos(ph)sin(th) /4   */
    m->row[0].c[0] = agc_dp_shl(ip, agc_dp_mul(ip, cth, cps), 1);
    agc_dp_t t = agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_mul(ip, cth, sps), cph), 1);
    m->row[0].c[1] = agc_dp_shl(ip, agc_dp_sub(ip, pd6, t), 1);
    t = agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_mul(ip, cth, sps), sph), 1);
    m->row[0].c[2] = agc_dp_shl(ip, agc_dp_add(ip, t, pd7), 1);
    m->row[1].c[0] = sps;
    m->row[1].c[1] = agc_dp_shl(ip, agc_dp_mul(ip, cps, cph), 1);
    m->row[1].c[2] = agc_dp_shl(ip, agc_dp_neg(agc_dp_mul(ip, cps, sph)), 1);
    m->row[2].c[0] = agc_dp_shl(ip, agc_dp_neg(agc_dp_mul(ip, sth, cps)), 1);
    t = agc_dp_shl(ip, agc_dp_mul(ip, pd7, sps), 1);          /* C7       */
    m->row[2].c[1] = agc_dp_shl(ip, agc_dp_add(ip, t, agc_dp_mul(ip, cth, sph)), 1);
    t = agc_dp_shl(ip, agc_dp_mul(ip, pd6, sps), 1);          /* C8       */
    m->row[2].c[2] = agc_dp_shl(ip, agc_dp_sub(ip, agc_dp_mul(ip, cth, cph), t), 1);
    return AGC_STATUS_OK;
}

/* Quadrant correction of DCMTOCDU: when the cosine term is negative the
 * angle is +-.5 - angle (SUHALFA / SUHALFAP).                            */
static agc_dp_t dcm_quadrant(agc_interp_t *ip, agc_dp_t ang, agc_dp_t costerm)
{
    (void)AGC_ASSERT(ip != NULL);
    if (costerm >= 0) {
        return ang;                                 /* OKTHETA / OKPHI    */
    }
    agc_dp_t n = agc_dp_neg(ang);
    agc_dp_t r = (n >= 0) ? agc_dp_sub(ip, n, AGC_DP_HALF)
                          : agc_dp_add(ip, n, AGC_DP_HALF);
    (void)AGC_ASSERT(r >= AGC_DP_MIN && r <= AGC_DP_MAX);
    return r;
}

/* DCMTOCDU (ANGLFIND p.409-410): CDU angles (revolutions) from an S/C to
 * SM matrix scaled 2.  cos(MGA), scale 1, is left in *cos_mga (S1).      */
agc_status_t agc_kalc_dcmtocdu(agc_interp_t *ip, const agc_mat_t *m,
                               agc_vec_t *angles, agc_dp_t *cos_mga)
{
    if (!AGC_ASSERT(ip != NULL && m != NULL && angles != NULL && cos_mga != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(vec_ok(m->row[1]) && vec_ok(m->row[2]))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t psi = agc_dp_asin(ip, m->row[1].c[0]);           /* C3       */
    agc_dp_t cpsi = agc_dp_shl(ip, agc_dp_cos(ip, psi), 1);
    cpsi = bovb_signmpac(ip, cpsi);                           /* S1       */
    agc_dp_t q = agc_dp_div(ip, agc_dp_neg(m->row[2].c[0]), cpsi); /* -C6 */
    agc_dp_t theta = dcm_quadrant(ip, agc_dp_asin(ip, q), m->row[0].c[0]);
    q = agc_dp_div(ip, agc_dp_neg(m->row[1].c[2]), cpsi);          /* -C5 */
    agc_dp_t phi = dcm_quadrant(ip, agc_dp_asin(ip, q), m->row[1].c[1]);
    *angles = agc_vec_make(phi, theta, psi);                  /* VECOFANG */
    *cos_mga = cpsi;
    return AGC_STATUS_OK;
}

/* One diagonal element of DEL: u u (1 - cos A) + cos A, scale 1.         */
static agc_dp_t del_diag(agc_interp_t *ip, agc_dp_t u, agc_dp_t omc2, agc_dp_t cos8)
{
    agc_dp_t d = agc_dp_mul(ip, agc_dp_sq(ip, u), omc2);
    d = agc_dp_shl(ip, agc_dp_add(ip, d, cos8), 3);           /* SL3      */
    return bovb_signmpac(ip, d);
}

/* A pair of off-diagonal elements: (ui uj (1-cos A) +- uk sin A), scale 1. */
static void del_pair(agc_interp_t *ip, agc_dp_t ui, agc_dp_t uj, agc_dp_t uk,
                     agc_dp_t omc2, agc_dp_t sin2, agc_dp_t out[2])
{
    agc_dp_t pd6 = agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_mul(ip, ui, uj), omc2), 1);
    agc_dp_t pd8 = agc_dp_mul(ip, uk, sin2);                  /* $ 4      */
    out[0] = bovb_signmpac(ip, agc_dp_shl(ip, agc_dp_add(ip, pd8, pd6), 2));
    out[1] = bovb_signmpac(ip, agc_dp_shl(ip, agc_dp_sub(ip, pd6, pd8), 2));
    (void)AGC_ASSERT(out[0] >= AGC_DP_MIN && out[1] >= AGC_DP_MIN);
}

/* DELCOMP (ANGLFIND p.407-409):
 *   DEL = I cos(A) + U U^T (1 - cos(A)) + U* sin(A), scale 1,
 * U half-unit along the rotation axis, A in revolutions.                 */
agc_status_t agc_kalc_delcomp(agc_interp_t *ip, agc_vec_t u, agc_dp_t a, agc_mat_t *del)
{
    if (!AGC_ASSERT(ip != NULL && del != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(vec_ok(u) && a >= AGC_DP_MIN && a <= AGC_DP_MAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t sin2 = agc_dp_sin(ip, a);                        /* PD0 $2   */
    agc_dp_t cos2 = agc_dp_cos(ip, a);
    agc_dp_t cos8 = agc_dp_shr(cos2, 2);                      /* PD2 $8   */
    agc_dp_t omc2 = bovb_signmpac(ip, agc_dp_sub(ip, AGC_DP_HALF, cos2)); /* PD4 */
    del->row[0].c[0] = del_diag(ip, u.c[0], omc2, cos8);
    del->row[1].c[1] = del_diag(ip, u.c[1], omc2, cos8);
    del->row[2].c[2] = del_diag(ip, u.c[2], omc2, cos8);
    agc_dp_t pr[2];
    del_pair(ip, u.c[0], u.c[1], u.c[2], omc2, sin2, pr);
    del->row[1].c[0] = pr[0];                                 /* DEL +6   */
    del->row[0].c[1] = pr[1];                                 /* DEL +2   */
    del_pair(ip, u.c[0], u.c[2], u.c[1], omc2, sin2, pr);
    del->row[0].c[2] = pr[0];                                 /* DEL +4   */
    del->row[2].c[0] = pr[1];                                 /* DEL +12D */
    del_pair(ip, u.c[1], u.c[2], u.c[0], omc2, sin2, pr);
    del->row[2].c[1] = pr[0];                                 /* DEL +14D */
    del->row[1].c[2] = pr[1];                                 /* DEL +10D */
    return AGC_STATUS_OK;
}

/* ALTCALC (p.400-401): for AM > 170 deg the axis comes from the
 * symmetric part MFISYM = (MFI + TMFI)/2 (scale 4):
 *   COFi = SQRT((MFISYMii - CAM/2) / (1/2 - CAM)), then UNIT.            */
static agc_vec_t altcalc(agc_interp_t *ip, const agc_kalc_setup_t *k, agc_mat_t *sym)
{
    agc_mat_t t = agc_mat_transpose(&k->mfi);
    for (int i = 0; i < 3; i++) {
        sym->row[i] = agc_vec_shr(agc_vec_add(ip, k->mfi.row[i], t.row[i]), 1);
    }
    agc_dp_t pd0 = agc_dp_shr(k->cam, 1);                     /* CAM $4   */
    agc_dp_t pd2 = bovb_signmpac(ip, agc_dp_sub(ip, AGC_DP_HALF, k->cam));
    agc_dp_t cof[3];
    for (int i = 2; i >= 0; i--) {                            /* Z, Y, X  */
        agc_dp_t r = agc_dp_div(ip, agc_dp_sub(ip, sym->row[i].c[i], pd0), pd2);
        cof[i] = agc_dp_sqrt(ip, r);                          /* $ ROOT 2 */
    }
    (void)AGC_ASSERT(pd2 >= 0);
    return agc_vec_unit(ip, agc_vec_make(cof[0], cof[1], cof[2]), NULL, NULL);
}

/* COFMAXGO / METHOD1-3 (p.401-403): fix the signs of the ALTCALC axis
 * from the largest component: its sign from COFSKEW, the others from the
 * off-diagonal terms of MFISYM.                                          */
static agc_vec_t cofmaxgo(agc_vec_t cof, agc_vec_t skew, const agc_mat_t *sym)
{
    int mx;
    if (cof.c[0] - cof.c[1] < 0) {          /* components are positive  */
        mx = (cof.c[1] - cof.c[2] < 0) ? 2 : 1;               /* COMP12   */
    } else {
        mx = (cof.c[0] - cof.c[2] < 0) ? 2 : 0;
    }
    (void)AGC_ASSERT(mx >= 0 && mx < 3);
    if (skew.c[mx] < 0) {                                     /* UnPOS    */
        cof = agc_vec_neg(cof);
    }
    for (int j = 0; j < 3; j++) {                             /* OKUnm    */
        int r = (j < mx) ? j : mx;
        int c = (j < mx) ? mx : j;
        if (j != mx && sym->row[r].c[c] < 0) {
            cof.c[j] = agc_dp_neg(cof.c[j]);
        }
    }
    (void)AGC_ASSERT(vec_ok(cof));
    return cof;
}

/* SLOAD of a CDU word: the AGC reads it as a ones'-complement SP.        */
static agc_dp_t sload_cdu(uint16_t p)
{
    (void)AGC_ASSERT(p <= CDU_MASK);
    long v = (p & 040000u) ? -(long)(~p & 037777u) : (long)p;
    (void)AGC_ASSERT(v >= -16383L && v <= 16383L);
    return (agc_dp_t)(v * 16384L);
}

/* MFI, its transpose, COFSKEW and CAM (SECAD, p.399-400).                */
static void secad(agc_interp_t *ip, agc_kalc_setup_t *k)
{
    agc_mat_t tmis = agc_mat_transpose(&k->mis);              /* TRANSPOS */
    k->mfi = agc_mat_mul(ip, &tmis, &k->mfs);                 /* MXM3 $4  */
    const agc_mat_t *f = &k->mfi;
    k->cofskew = agc_vec_make(agc_dp_sub(ip, f->row[2].c[1], f->row[1].c[2]),
                              agc_dp_sub(ip, f->row[0].c[2], f->row[2].c[0]),
                              agc_dp_sub(ip, f->row[1].c[0], f->row[0].c[1]));
    agc_dp_t cam = agc_dp_add(ip, f->row[0].c[0], f->row[2].c[2]);
    cam = agc_dp_sub(ip, cam, AGC_DP_QUARTER);
    k->cam = agc_dp_add(ip, cam, f->row[1].c[1]);   /* (trace - 1)/4      */
    k->am = agc_dp_acos(ip, k->cam);                /* revolutions        */
    (void)AGC_ASSERT(k->am >= 0);
}

/* KALCMAN3 .. SECAD .. CHECKMAX / ALTCALC / COFMAXGO (pp.399-403): from
 * the initial CDU angles bcdu and the final CPHI, CTHETA, CPSI compute
 * MIS, MFS, MFI, the maneuver angle AM and axis COF.                     */
agc_status_t agc_kalc_compute(agc_interp_t *ip, const uint16_t bcdu[3],
                              const uint16_t cphi[3], agc_kalc_setup_t *out)
{
    if (!AGC_ASSERT(ip != NULL && bcdu != NULL && cphi != NULL && out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    out->kcase = AGC_KALC_NORMAL;
    out->cof = agc_vec_zero();
    if (agc_state_vehicle() == AGC_VEHICLE_LM
        && agc_dp_sub(ip, agc_dp_abs(sload_cdu(cphi[2])), LOCKANGL) >= 0) {
        out->kcase = AGC_KALC_TOOBAD;                         /* TOOBADF  */
        return AGC_STATUS_OK;
    }
    agc_status_t st = agc_kalc_cdutodcm(ip, bcdu, &out->mis);   /* MIS    */
    if (st == AGC_STATUS_OK) {
        st = agc_kalc_cdutodcm(ip, cphi, &out->mfs);            /* MFS    */
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    secad(ip, out);
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM);
    if (agc_dp_sub(ip, out->am, lm ? MINANG_LM : MINANG_CM) < 0) {
        out->kcase = AGC_KALC_SMALL;          /* less than 0.25 deg       */
        return AGC_STATUS_OK;
    }
    if (agc_dp_sub(ip, out->am, lm ? MAXANG_LM : MAXANG_CM) < 0) { /* CHECKMAX */
        out->cof = agc_vec_unit(ip, out->cofskew, NULL, NULL);
        return AGC_STATUS_OK;
    }
    out->kcase = AGC_KALC_LARGE;                              /* ALTCALC  */
    agc_mat_t sym;
    agc_vec_t cof = altcalc(ip, out, &sym);
    out->cof = cofmaxgo(cof, out->cofskew, &sym);
    return AGC_STATUS_OK;
}
