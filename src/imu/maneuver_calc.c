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
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "internal.h"

/* MINANG = 0.25 deg and MAXANG = 170 deg, in revolutions.  Luminary has
 * them as 2DEC.  Comanche writes single-word DEC constants, so DSU reads
 * MINANG with MAXANG as its low word and MAXANG with SD (DEC .433015):
 *   CM MINANG = 11, 7737  (.00069375, .472222)
 *   CM MAXANG = 7737, 7095 (.472222, .433015)                            */
#define MINANG_LM ((agc_dp_t) 186227)    /* 2DEC 0.00069375               */
#define MAXANG_LM ((agc_dp_t) 126761187) /* 2DEC 0.472222222              */
#define MINANG_CM ((agc_dp_t) ((11L << 14) + 7737L))
#define MAXANG_CM ((agc_dp_t) ((7737L << 14) + 7095L))
/* LOCKANGL DEC .388889 = 70 deg at half-revolution scale.  DSU reads two
 * words, so the low half is the next word in fixed memory: the READCDUK
 * instruction CA CDUZ (octal 30034).                                     */
#define LOCKANGL ((agc_dp_t) ((6372L << 14) + 030034L))
#define CDU_MASK 077777u

static bool vec_ok(agc_vec_t v)
{
    bool ok = true;

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        ok = ok && v.c[i] >= AGC_DP_MIN && v.c[i] <= AGC_DP_MAX;
    }

    return (ok);
}

/* BOVB SIGNMPAC: on overflow replace MPAC by DPPOSMAX with its sign.     */
static agc_dp_t bovb_signmpac(agc_interp_t *ip, agc_dp_t v)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0); /* no valid result */
    }

    if (agc_interp_test_ovf(ip))
    {
        v = agc_rtb_signmpac(v);
    }

    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    return (v);
}

/* DMP then DAD then SL n, then STORE: the TP product keeps MPAC+2, the
 * DAD adds to MPAC, MPAC+1 only, the shift moves MPAC+2's top bits up,
 * and the STORE truncates to two words.                                 */
static agc_dp_t dmp_dad_shl(agc_interp_t *ip, agc_dp_t a, agc_dp_t b,
                            agc_dp_t add, int n)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(n >= 0 && n <= 3))
    {
        return (0); /* no valid result */
    }

    agc_tp_t m = agc_dp_mul_tp(a, b);              /* DMP: TP MPAC      */

    m = agc_tp_add(ip, m, agc_tp_from_dp(add));    /* DAD: MPAC+2 kept  */

    return (agc_tp_to_dp(agc_tp_shift(ip, m, n))); /* SLn, STORE        */
}

/* CDUTODCM (ANGLFIND p.405-407): S/C to SM direction cosines from the CDU
 * angles PHI = OGA (X), THETA = IGA (Y), PSI = MGA (Z); scale 2.         */
agc_status_t agc_kalc_cdutodcm(agc_interp_t *ip, const uint16_t cdu[3],
                               agc_mat_t *m)
{
    if (!AGC_ASSERT(ip != NULL && cdu != NULL && m != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t s[3];
    agc_dp_t c[3];

    for (int i = 0; i < 3; i++)
    { /* LOOPSIN: X, Y, Z    */
        if (!AGC_ASSERT(cdu[i] <= CDU_MASK))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_dp_t a = agc_rtb_cdulogic(cdu[i]);

        s[i] = agc_dp_sin(ip, a); /* sin/2               */
        c[i] = agc_dp_cos(ip, a); /* cos/2               */
    }

    const agc_dp_t sph = s[0], cph = c[0], sth = s[1], cth = c[1];
    const agc_dp_t sps = s[2], cps = c[2];
    /* PD12 = sin(th)sin(ph), PD14 = cos(ph)sin(th): DMP then PDDL keeps
     * two words; scale 4.                                                */
    const agc_dp_t pd12 = agc_dp_mul_trunc(sth, sph);
    const agc_dp_t pd14 = agc_dp_mul_trunc(cph, sth);

    m->row[0].c[0] = dmp_dad_shl(ip, cth, cps, 0, 1); /* C0: DMP SL1    */
    /* C1: cos(th) DMP sin(ps) DMP cos(ph) SL1, BDSU PD12, SL1.  The BDSU
     * leaves MPAC+2 of the product; its top bit is lost here (DEVIATION:
     * see docs/modules/review-imu.md F5).                               */
    agc_dp_t t = dmp_dad_shl(ip, agc_dp_mul_trunc(cth, sps), cph, 0, 1);

    m->row[0].c[1] = agc_dp_shl(ip, agc_dp_sub(ip, pd12, t), 1);
    /* C2: cos(th) DMP sin(ps) DMP sin(ph) SL1, DAD PD14, SL1            */
    agc_tp_t tp = agc_tp_shift(
        ip, agc_dp_mul_tp(agc_dp_mul_trunc(cth, sps), sph), 1);

    tp             = agc_tp_add(ip, tp, agc_tp_from_dp(pd14));
    m->row[0].c[2] = agc_tp_to_dp(agc_tp_shift(ip, tp, 1));
    m->row[1].c[0] = sps;                              /* C3: DLOAD 8D  */
    m->row[1].c[1] = dmp_dad_shl(ip, cps, cph, 0, 1); /* C4: DMP SL1   */
    m->row[1].c[2] =
        agc_dp_neg(dmp_dad_shl(ip, cps, sph, 0, 1)); /* C5: DMP DCOMP SL1 */
    m->row[2].c[0] =
        agc_dp_neg(dmp_dad_shl(ip, sth, cps, 0, 1)); /* C6: DMP DCOMP SL1 */
    /* C7: (PD14 DMP sin(ps) SL1) pushed; cos(th) DMP sin(ph), DAD, SL1  */
    t              = dmp_dad_shl(ip, pd14, sps, 0, 1);
    m->row[2].c[1] = dmp_dad_shl(ip, cth, sph, t, 1);
    /* C8: (PD12 DMP sin(ps) SL1) pushed; cos(th) DMP cos(ph), DSU, SL1  */
    t              = dmp_dad_shl(ip, pd12, sps, 0, 1);
    m->row[2].c[2] = dmp_dad_shl(ip, cth, cph, agc_dp_neg(t), 1);

    return (AGC_STATUS_OK);
}

/* Quadrant correction of DCMTOCDU: when the cosine term is negative the
 * angle is +-.5 - angle (SUHALFA / SUHALFAP).                            */
static agc_dp_t dcm_quadrant(agc_interp_t *ip, agc_dp_t ang, agc_dp_t costerm)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0); /* no valid result */
    }

    if (costerm >= 0)
    {
        return (ang); /* OKTHETA / OKPHI    */
    }

    agc_dp_t n = agc_dp_neg(ang);
    agc_dp_t r = (n >= 0) ? agc_dp_sub(ip, n, AGC_DP_HALF) :
                 agc_dp_add(ip, n, AGC_DP_HALF);

    if (!AGC_ASSERT(r >= AGC_DP_MIN && r <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/* DCMTOCDU (ANGLFIND p.409-410): CDU angles (revolutions) from an S/C to
 * SM matrix scaled 2.  cos(MGA), scale 1, is left in *cos_mga (S1).      */
agc_status_t agc_kalc_dcmtocdu(agc_interp_t *ip, const agc_mat_t *m,
                               agc_vec_t *angles, agc_dp_t *cos_mga)
{
    if (!AGC_ASSERT(ip != NULL && m != NULL && angles != NULL &&
                    cos_mga != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(vec_ok(m->row[1]) && vec_ok(m->row[2])))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t psi  = agc_dp_asin(ip, m->row[1].c[0]);  /* C3       */
    agc_dp_t cpsi = agc_dp_shl(ip, agc_dp_cos(ip, psi), 1);

    cpsi = bovb_signmpac(ip, cpsi); /* S1       */
    agc_dp_t q     = agc_dp_div(ip, agc_dp_neg(m->row[2].c[0]), cpsi); /* -C6 */
    agc_dp_t theta = dcm_quadrant(ip, agc_dp_asin(ip, q), m->row[0].c[0]);

    q = agc_dp_div(ip, agc_dp_neg(m->row[1].c[2]), cpsi); /* -C5 */
    agc_dp_t phi = dcm_quadrant(ip, agc_dp_asin(ip, q), m->row[1].c[1]);

    *angles  = agc_vec_make(phi, theta, psi);       /* VECOFANG */
    *cos_mga = cpsi;

    return (AGC_STATUS_OK);
}

/* One diagonal element of DEL: u u (1 - cos A) + cos A, scale 1.
 * PDDL COF, DSQ, DMP 4, DAD 2, SL3, BOVB SIGNMPAC (p.407).              */
static agc_dp_t del_diag(agc_interp_t *ip, agc_dp_t u, agc_dp_t omc2,
                         agc_dp_t cos8)
{
    agc_dp_t uu = agc_tp_to_dp(agc_dp_sq_tp(u)); /* DSQ, then DMP reads 2 words */

    return (bovb_signmpac(ip, dmp_dad_shl(ip, uu, omc2, cos8, 3)));
}

/* A pair of off-diagonal elements (p.408-409):
 *   DLOAD ui, DMP uj, DMP 4, SL1, PDDL uk  -> PD6 = ui uj (1-cos A)  $4
 *   DMP 0, PUSH, DAD 6, SL2, BOVB, STODL   -> out[0] = (PD8 + PD6) SL2
 *   (pop PD8), BDSU (pop PD6), SL2, BOVB   -> out[1] = (PD6 - PD8) SL2
 * PD8 = uk sin A $4.  Scale 1.                                          */
static void del_pair(agc_interp_t *ip, agc_dp_t ui, agc_dp_t uj, agc_dp_t uk,
                     agc_dp_t omc2, agc_dp_t sin2, agc_dp_t out[2])
{
    agc_dp_t uij = agc_dp_mul_trunc(ui, uj); /* DMP, next DMP reads 2 words */
    agc_dp_t pd6 = dmp_dad_shl(ip, uij, omc2, 0, 1);         /* SL1, PDDL */
    agc_dp_t pd8 = agc_dp_mul_trunc(uk, sin2);                /* PUSH      */

    out[0] = bovb_signmpac(ip, dmp_dad_shl(ip, uk, sin2, pd6, 2));
    out[1] = bovb_signmpac(ip, agc_dp_shl(ip, agc_dp_sub(ip, pd6, pd8), 2));

    if (!AGC_ASSERT(out[0] >= AGC_DP_MIN && out[1] >= AGC_DP_MIN))
    {
        return;
    }
}

/* DELCOMP (ANGLFIND p.407-409):
 *   DEL = I cos(A) + U U^T (1 - cos(A)) + U* sin(A), scale 1,
 * U half-unit along the rotation axis, A in revolutions.                 */
agc_status_t agc_kalc_delcomp(agc_interp_t *ip, agc_vec_t u, agc_dp_t a,
                              agc_mat_t *del)
{
    if (!AGC_ASSERT(ip != NULL && del != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(vec_ok(u) && a >= AGC_DP_MIN && a <= AGC_DP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t sin2 = agc_dp_sin(ip, a); /* PD0 $2   */
    agc_dp_t cos2 = agc_dp_cos(ip, a);
    agc_dp_t cos8 = agc_dp_shr(cos2, 2); /* PD2 $8   */
    agc_dp_t omc2 =
        bovb_signmpac(ip, agc_dp_sub(ip, AGC_DP_HALF, cos2)); /* PD4 */

    del->row[0].c[0] = del_diag(ip, u.c[0], omc2, cos8);
    del->row[1].c[1] = del_diag(ip, u.c[1], omc2, cos8);
    del->row[2].c[2] = del_diag(ip, u.c[2], omc2, cos8);
    agc_dp_t pr[2];

    del_pair(ip, u.c[0], u.c[1], u.c[2], omc2, sin2, pr);
    del->row[1].c[0] = pr[0]; /* DEL +6   */
    del->row[0].c[1] = pr[1]; /* DEL +2   */
    del_pair(ip, u.c[0], u.c[2], u.c[1], omc2, sin2, pr);
    del->row[0].c[2] = pr[0]; /* DEL +4   */
    del->row[2].c[0] = pr[1]; /* DEL +12D */
    del_pair(ip, u.c[1], u.c[2], u.c[0], omc2, sin2, pr);
    del->row[2].c[1] = pr[0]; /* DEL +14D */
    del->row[1].c[2] = pr[1]; /* DEL +10D */

    return (AGC_STATUS_OK);
}

/* COFMAXGO / METHOD1-3 (p.401-403): fix the signs of the ALTCALC axis
 * from the largest component: its sign from COFSKEW, the others from the
 * off-diagonal terms of MFISYM.                                          */
static agc_vec_t cofmaxgo(agc_vec_t cof, agc_vec_t skew, const agc_mat_t *sym)
{
    int mx;

    if (cof.c[0] - cof.c[1] < 0)
    {                                           /* components are positive  */
        mx = (cof.c[1] - cof.c[2] < 0) ? 2 : 1; /* COMP12   */
    }
    else
    {
        mx = (cof.c[0] - cof.c[2] < 0) ? 2 : 0;
    }

    if (!AGC_ASSERT(mx >= 0 && mx < 3))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (skew.c[mx] < 0)
    { /* UnPOS    */
        cof = agc_vec_neg(cof);
    }

    for (int j = 0; j < 3; j++)
    { /* OKUnm    */
        int r = (j < mx) ? j : mx;
        int c = (j < mx) ? mx : j;

        if (j != mx && sym->row[r].c[c] < 0)
        {
            cof.c[j] = agc_dp_neg(cof.c[j]);
        }
    }

    if (!AGC_ASSERT(vec_ok(cof)))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (cof);
}

/* SLOAD of a CDU word: the AGC reads it as a ones'-complement SP.        */
static agc_dp_t sload_cdu(uint16_t p)
{
    if (!AGC_ASSERT(p <= CDU_MASK))
    {
        return (0); /* no valid result */
    }

    long v = (p & 040000u) ? -(long) (~p & 037777u) : (long) p;

    if (!AGC_ASSERT(v >= -16383L && v <= 16383L))
    {
        return (0); /* no valid result */
    }

    return ((agc_dp_t) (v * 16384L));
}

/* KALCMAN3 .. SECAD .. CHECKMAX / ALTCALC / COFMAXGO (pp.399-403): from
 * the initial CDU angles bcdu and the final CPHI, CTHETA, CPSI compute
 * MIS, MFS, MFI, the maneuver angle AM and axis COF.                     */
agc_status_t agc_kalc_compute(agc_interp_t *ip, const uint16_t bcdu[3],
                              const uint16_t cphi[3], agc_kalc_setup_t *out)
{
    if (!AGC_ASSERT(ip != NULL && bcdu != NULL && cphi != NULL && out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    out->kcase = AGC_KALC_NORMAL;
    out->cof   = agc_vec_zero();

    if (agc_state_vehicle() == AGC_VEHICLE_LM &&
        agc_dp_sub(ip, agc_dp_abs(sload_cdu(cphi[2])), LOCKANGL) >= 0)
    {
        out->kcase = AGC_KALC_TOOBAD; /* TOOBADF  */

        return (AGC_STATUS_OK);
    }

    agc_status_t st = agc_kalc_cdutodcm(ip, bcdu, &out->mis); /* MIS    */

    if (st == AGC_STATUS_OK)
    {
        st = agc_kalc_cdutodcm(ip, cphi, &out->mfs); /* MFS    */
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    /* MFI, its transpose, COFSKEW and CAM (SECAD, p.399-400).                */
    agc_mat_t tmis = agc_mat_transpose(&out->mis);      /* TRANSPOS */

    out->mfi = agc_mat_mul(ip, &tmis, &out->mfs); /* MXM3 $4  */
    const agc_mat_t *f = &out->mfi;

    out->cofskew = agc_vec_make(agc_dp_sub(ip, f->row[2].c[1], f->row[1].c[2]),
                                agc_dp_sub(ip, f->row[0].c[2], f->row[2].c[0]),
                                agc_dp_sub(ip, f->row[1].c[0], f->row[0].c[1]));
    agc_dp_t cam = agc_dp_add(ip, f->row[0].c[0], f->row[2].c[2]);

    cam      = agc_dp_sub(ip, cam, AGC_DP_QUARTER);
    out->cam = agc_dp_add(ip, cam, f->row[1].c[1]); /* (trace - 1)/4      */
    out->am  = agc_dp_acos(ip, out->cam);                   /* revolutions        */

    if (!AGC_ASSERT(out->am >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM);

    if (agc_dp_sub(ip, out->am, lm ? MINANG_LM : MINANG_CM) < 0)
    {
        out->kcase = AGC_KALC_SMALL; /* less than 0.25 deg       */

        return (AGC_STATUS_OK);
    }

    if (agc_dp_sub(ip, out->am, lm ? MAXANG_LM : MAXANG_CM) < 0)
    { /* CHECKMAX */
        out->cof = agc_vec_unit(ip, out->cofskew, NULL, NULL);

        return (AGC_STATUS_OK);
    }

    out->kcase = AGC_KALC_LARGE; /* ALTCALC  */
    agc_mat_t sym;
    /* ALTCALC (p.400-401): for AM > 170 deg the axis comes from the
     * symmetric part MFISYM = (MFI + TMFI)/2 (scale 4):
     *   COFi = SQRT((MFISYMii - CAM/2) / (1/2 - CAM)), then UNIT.            */
    agc_mat_t t = agc_mat_transpose(&out->mfi);

    for (int i = 0; i < 3; i++)
    {
        sym.row[i] = agc_vec_shr(agc_vec_add(ip, out->mfi.row[i], t.row[i]), 1);
    }

    agc_dp_t pd0 = agc_dp_shr(out->cam, 1); /* CAM $4   */
    agc_dp_t pd2 = bovb_signmpac(ip, agc_dp_sub(ip, AGC_DP_HALF, out->cam));
    agc_dp_t cof_altcalc[3];

    for (int i = 2; i >= 0; i--)
    { /* Z, Y, X  */
        agc_dp_t r = agc_dp_div(ip, agc_dp_sub(ip, sym.row[i].c[i], pd0), pd2);

        cof_altcalc[i] = agc_dp_sqrt(ip, r); /* $ ROOT 2 */
    }

    if (!AGC_ASSERT(pd2 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t cof =
        agc_vec_unit(ip, agc_vec_make(cof_altcalc[0], cof_altcalc[1],
                                      cof_altcalc[2]), NULL, NULL);

    out->cof = cofmaxgo(cof, out->cofskew, &sym);

    return (AGC_STATUS_OK);
}
