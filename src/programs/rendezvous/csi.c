/*
 * csi.c -- concentric flight plan targeting, both vehicles:
 * CSI/A .. CSINEXT4, SCNDSOL, ALMXIT (the CSI iteration), ADVANCE, ROTATE,
 * INTINT2C, INTINT3P, CDHMVR, S32/33.X and the matrix part of S32/33.1.
 *
 * Source: Comanche055 P32-P33_P72-P73.agc pp.663-682 (Luminary099
 * P32-P35_P72-P75.agc carries the same text; the differences are names of
 * equal constants: TMIN = 600SEC, TWOPI = 2PISC SL2, 1DPB2 = 1RTEB2,
 * INITST = INITST1, P30ZERO = ZEROVEC).
 *
 * The push-down list of the AGC is kept in named locals (the AGC cell is
 * given in the comment). Address-less push-down operations follow the AGC
 * interpreter: a load with no address pops; PDDL/PDVL with no address
 * exchanges MPAC with the top of the list.
 *
 * Scales: r m 2^29, v m/cs 2^7, t cs 2^28, angles revolutions, unit
 * vectors 2^1, RTMU 2^36. "Body scale" = 2^27 / 2^5 about the moon.
 */
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "internal.h"

/* ---- constants (P32-P33 p.663, values as 2^28 fractions) --------------- */
#define CSI_INITST    ((agc_dp_t)63921)     /* .03048 B-7: 10 fps            */
#define CSI_DVMAX1    ((agc_dp_t)6392119)   /* 3.0480 B-7: 1000 fps          */
#define CSI_DVMAX2    ((agc_dp_t)6321806)   /* 3.014472 B-7: 989 fps         */
#define CSI_1DPB2     ((agc_dp_t)67108864)  /* 1.0 B-2 (CM 1RTEB2 / 1DPB2)   */
#define CSI_1DPB28    ((agc_dp_t)1)         /* 2DEC 1: one count at B28      */
#define CSI_LOOPMX    ((agc_dp_t)16)        /* LOOPMX 2DEC 16                */
#define CSI_EPSILN1   ((agc_dp_t)639)       /* .0003048 B-7: .1 fps          */
#define CSI_FIFPSDP   ((agc_dp_t)-319606)   /* -.1524 B-7: 50 fps            */
#define CSI_DELMAX1   ((agc_dp_t)1278424)   /* .6096 B-7: 200 fps            */
#define CSI_PMINE     ((agc_dp_t)78710)     /* 157420 B-29: 84 NM            */
#define CSI_PMINM     ((agc_dp_t)5334)      /* 10668 B-29: 35000 ft          */
#define CSI_NICKELDP  ((agc_dp_t)44745)     /* .021336 B-7: 7 fps            */
#define CSI_ONETHTH   ((agc_dp_t)3355)      /* .0001 B-3                     */
#define CSI_DP1_4     AGC_DP_QUARTER        /* DP1/4 (LM DP1/4TH)            */
#define CSI_MAX_PASSES 200                  /* bound on CSI/B..CSI/B1 passes */

/* CSI labels: the targets of the AGC branches inside CSI/A. */
typedef enum csi_label {
    CSI_L_B = 0,      /* CSI/B: new cycle, first DELVCSI estimate          */
    CSI_L_B1,         /* CSI/B1: count the iteration                       */
    CSI_L_B2,         /* CSI/B2: limit DELVCSI, then CSI/B23 trial         */
    CSI_L_SCNDSOL,    /* SCNDSOL: second cycle or alarm                    */
    CSI_L_ALMXITA,    /* ALMXITA: alarm with X2                            */
    CSI_L_ALMXIT,     /* ALMXIT: alarm with the stored CSIALRM             */
    CSI_L_DONE,       /* CSINEXT4 BPL P32/P72C: solution found             */
    CSI_L_ERROR       /* a called routine failed / INTSTALL sleep          */
} csi_label_t;

/* Working cells of one CSI/A call (push list, index registers). */
typedef struct csi_work {
    agc_dp_t     gamma;   /* 0D GAMMA, revolutions 2^1                     */
    agc_dp_t     slope;   /* 4D (GAM-GAMPREV)/(DV-DVPREV), 2^(-6+N1)       */
    int          x2;      /* X2 = alarm index for SCNDSOL / ALMXITA         */
    agc_job_id_t resume;  /* INTSTALL continuation                         */
    agc_status_t st;      /* status when CSI_L_ERROR                       */
} csi_work_t;

/* ---- flag helpers ------------------------------------------------------- */
static bool csi_f(rz_vflag_t f)
{
    (void)AGC_ASSERT(f >= RZ_VF_CSI_F1 && f <= RZ_VF_CSI_F3B);
    return rz_vflag_test(f);
}

static void csi_set(rz_vflag_t f, bool on)
{
    (void)AGC_ASSERT(f >= RZ_VF_CSI_F1 && f <= RZ_VF_CSI_F3B);
    agc_status_t st = rz_vflag_set(f, on);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
}

/* CSI/A first lines and SCNDSOL: F1 = 0, F2 = 1, F3A F3B = 0 1 (first
 * cycle) or 0 0 (second cycle).                                           */
static void csi_set_cycle(bool first_cycle)
{
    csi_set(RZ_VF_CSI_F1, false);
    csi_set(RZ_VF_CSI_F2, true);
    csi_set(RZ_VF_CSI_F3A, false);
    csi_set(RZ_VF_CSI_F3B, first_cycle);
}

/* ---- ROTATE (p.676): project v into the passive orbit plane, keep |v| --- */
agc_status_t agc_rendz_rotate(agc_interp_t *ip, agc_vec_t up1, agc_vec_t v,
                              agc_vec_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    /* PUSH PUSH; DOT VXSC UP1 UP1; VSL2 BVSU; UNIT PDVL; ABVAL VXSC; VSL1 */
    agc_vec_t proj = agc_vec_scale(ip, up1, agc_vec_dot(ip, v, up1));
    agc_vec_t inplane = agc_vec_sub(ip, v, agc_vec_shl(ip, proj, 2));
    agc_vec_t u = rz_unit(ip, inplane);
    agc_dp_t mag = rz_abval(ip, v);
    *out = agc_vec_shl(ip, agc_vec_scale(ip, u, mag), 1);
    (void)AGC_ASSERT(mag >= 0);
    return AGC_STATUS_OK;
}

/* ---- ADVANCE (p.675) -----------------------------------------------------
 * TDEC1 = TIG; PRECSET; XDELVFLG set; passive state at TIG into the
 * 1, 2 and 3 cells; UP1 = unit(unit(RP) x VP); active state at TIG
 * (RTIG, VTIG) rotated into the passive plane as RACT1/2, VACT1/2.     */
agc_status_t agc_rendz_advance(agc_interp_t *ip, agc_rendz_t *rz,
                               agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = rz_precset(ip, rz, agc_erase_tig(), resume);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    (void)agc_flag_set(AGC_FLAG_XDELVFLG);
    rz->vpass2 = rz->vpass3;
    rz->vpass1 = rz->vpass3;
    rz->rpass2 = rz->rpass3;
    rz->rpass1 = rz->rpass3;
    rz->up1 = rz_unit(ip, agc_vec_cross(ip, rz_unit(ip, rz->rpass1),
                                        rz->vpass1));
    rz->rtig = rz->ract3;
    st = agc_rendz_rotate(ip, rz->up1, rz->ract3, &rz->ract2);
    rz->ract1 = rz->ract2;
    rz->vtig = rz->vact3;
    if (st == AGC_STATUS_OK) {
        st = agc_rendz_rotate(ip, rz->up1, rz->vact3, &rz->vact2);
    }
    rz->vact1 = rz->vact2;
    return st;
}

/* ---- CSI/B (p.665): first estimate of the CSI horizontal delta V ---------
 * DELVCSI = sqrt(2 MU / ((1 + RA1/RP3) RA1)) - VA1 . UH1, DELDV = 10 fps.
 * (Velocity at RA1 of an ellipse with apocenter RP3, minus the present
 * horizontal velocity.)                                                   */
static void csi_b(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_dp_t ra1 = rz_abval(ip, rz->ract1);                  /* 0D B29     */
    int x2 = 0;
    agc_dp_t d2 = agc_dp_shr(rz_norm(ra1, &x2), 1);          /* 2D         */
    int x1 = 0;
    agc_dp_t rp3n = rz_norm(rz_abval(ip, rz->rpass3), &x1);
    agc_dp_t q = agc_dp_div(ip, d2, rp3n);                   /* BDDV       */
    x2 = x2 - x1;                                            /* XSU,2 X1   */
    q = rz_sr_star(ip, q, 1, x2);                            /* RA1/RP3 B2 */
    q = agc_dp_mul_trunc(agc_dp_add(ip, q, CSI_1DPB2), ra1); /* B31        */
    q = rz_norm(q, &x1);
    agc_dp_t vr = agc_dp_div(ip, agc_dp_shr(rz->body.rtmu, 1), q); /* 2MU/ */
    vr = agc_dp_sqrt(ip, rz_sl_star(ip, vr, -7, x1));        /* B7  0D     */
    agc_vec_t uh1 = rz_unit(ip, agc_vec_cross(ip, rz->up1,
                                              rz_unit(ip, rz->ract1)));
    agc_dp_t vh = agc_dp_shl(ip, agc_vec_dot(ip, uh1, rz->vact1), 1);
    rz->delvcsi = agc_dp_sub(ip, vr, vh);                    /* BDSU       */
    rz->deldv = CSI_INITST;
    (void)AGC_ASSERT(rz->deldv > 0);
}

/* ---- CSI/B1 (p.666): LOOPCT + 1; 16 passes -> SCNDSOL with X2 = 6 ------- */
static csi_label_t csi_b1(agc_rendz_t *rz, csi_work_t *w)
{
    (void)AGC_ASSERT(rz->loopct >= 0);
    rz->loopct = (agc_sp_t)(rz->loopct + 1);                 /* + 1DPB28   */
    if ((agc_dp_t)rz->loopct - CSI_LOOPMX >= 0) {
        w->x2 = 6;
        return CSI_L_SCNDSOL;
    }
    return CSI_L_B2;
}

/* ---- CSI/B2, CSI/B22 (p.666): limit |DELVCSI| to 989 fps once -----------
 * Returns false (SCNDSOL, X2 = 7) when the limit was already applied or
 * in the 50 fps stage of the second cycle.                                */
static bool csi_b2(agc_interp_t *ip, agc_rendz_t *rz, csi_work_t *w)
{
    if (agc_dp_sub(ip, agc_dp_abs(rz->delvcsi), CSI_DVMAX1) < 0) {
        return true;                                         /* CSI/B23    */
    }
    w->x2 = 7;
    if (csi_f(RZ_VF_CSI_F1)) {
        return false;
    }
    if (csi_f(RZ_VF_CSI_F3A) && csi_f(RZ_VF_CSI_F3B)) {
        return false;
    }
    csi_set(RZ_VF_CSI_F1, true);                            /* CSI/B22    */
    rz->delvcsi = agc_dp_sign(CSI_DVMAX2, rz->delvcsi);
    (void)AGC_ASSERT(rz->delvcsi != 0);
    return true;
}

/* VECSHIFT + TIMETHET (time only) of the post-CSI state (RACT1, v). */
static agc_status_t csi_timethet(agc_interp_t *ip, agc_rendz_t *rz,
                                 agc_vec_t v, agc_dp_t snth, agc_dp_t csth,
                                 agc_dp_t *t)
{
    agc_conic_time_out_t o;
    agc_status_t st = rz_timethet_rv(ip, rz, rz->ract1, v, snth, csth, true,
                                     &o);
    *t = o.t;
    (void)AGC_ASSERT(st != AGC_STATUS_OK || o.t >= AGC_DP_MIN);
    return st;
}

/* CSI/B23 .. CSI/B23D (p.667): DELVEET1 = DELVCSI UH1, VACT4, HAFPA1 =
 * half period (TIMETHET through 359.99 deg), POSTCSI; *ellipse is true
 * when the post-CSI orbit is not to be treated as circular.              */
static agc_status_t csi_b23(agc_interp_t *ip, agc_rendz_t *rz,
                            agc_rendz_periapo_t *pa, bool *ellipse)
{
    agc_vec_t uh1 = rz_unit(ip, agc_vec_cross(ip, rz->up1,
                                              rz_unit(ip, rz->ract1)));
    rz->delveet1 = agc_vec_shl(ip, agc_vec_scale(ip, uh1, rz->delvcsi), 1);
    rz->vact4 = agc_vec_add(ip, rz->delveet1, rz->vact1);
    (void)agc_interp_test_ovf(ip);                           /* BOV        */
    agc_dp_t t = 0;
    agc_status_t st = csi_timethet(ip, rz, rz->vact4, RZ_SN359, RZ_CS359, &t);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz->hafpa1 = agc_dp_shr(t, 1);                           /* SR1        */
    st = agc_rendz_periapo1(ip, rz, rz->ract1, rz->vact4, pa);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz->postcsi = agc_rendz_shiftr1(rz, pa->hperi);
    *ellipse = (rz->centang == 0)
               && (agc_dp_sub(ip, pa->ecc, CSI_ONETHTH) >= 0);
    return AGC_STATUS_OK;
}

/* (P/MU)^.5 / R1 part of the apsis-angle computation (p.668), B-31 (moon
 * SL3 more).                                                             */
static agc_dp_t csi_sqrt_p_mu(agc_interp_t *ip, const agc_rendz_t *rz,
                              agc_dp_t r1n, const agc_rendz_periapo_t *pa)
{
    agc_dp_t a = agc_dp_div(ip, agc_dp_shr(rz->body.rtsr1mu, 1), r1n); /* 2D */
    agc_dp_t b = agc_rendz_shiftr1(rz, agc_dp_mul_trunc(pa->p, pa->r1));
    b = agc_dp_shl(ip, agc_dp_shl(ip, b, 4), 1);             /* SL4 SL1    */
    b = agc_dp_mul_trunc(agc_dp_sqrt(ip, b), a);             /* SQRT DMP   */
    if (agc_flag_test(AGC_FLAG_CMOONFLG)) {
        b = agc_dp_shl(ip, b, 3);                            /* SL3        */
    }
    (void)AGC_ASSERT(b >= AGC_DP_MIN);
    return b;
}

/* Non-circular post-CSI orbit (p.668-669): RDOTV test (7 fps), then the
 * time T from RACT1 to the apsis by TIMETHET of the reversed state;
 * *pop = the value DSU'd at NTP/2 (T or HAFPA1 - T).                    */
static agc_status_t csi_apsis(agc_interp_t *ip, agc_rendz_t *rz,
                              const agc_rendz_periapo_t *pa, bool *circl,
                              agc_dp_t *pop)
{
    int x1 = 0;
    agc_dp_t r1n = rz_norm(agc_rendz_shiftr1(rz, pa->r1), &x1);   /* 2D */
    agc_dp_t rdv = agc_vec_dot(ip, rz->ract1, rz->vact4);
    agc_dp_t q = agc_dp_div(ip, agc_dp_abs(rdv), r1n);       /* B7         */
    q = agc_dp_sub(ip, rz_sl_star(ip, q, 0, x1), CSI_NICKELDP);
    *circl = (q < 0);
    if (*circl) {
        return AGC_STATUS_OK;
    }
    agc_dp_t d14 = agc_dp_sub(ip, agc_dp_shl(ip, pa->p, 2), CSI_1DPB2);
    agc_dp_t s = csi_sqrt_p_mu(ip, rz, r1n, pa);             /* 2D         */
    rz->rdotv = agc_vec_dot(ip, rz->ract1, rz->vact4);       /* CSI/B3     */
    int x2 = 0;
    agc_dp_t d12 = agc_dp_mul_trunc(rz_norm(agc_dp_abs(rz->rdotv), &x2), s);
    x1 = x1 - x2;                                            /* XSU,1 X2   */
    d12 = rz_sl_star(ip, d12, 3, x1);
    agc_vec_t sc = rz_unit(ip, agc_vec_make(d12, d14, 0));   /* 12D..16D   */
    agc_vec_t v = agc_vec_neg(rz_vsign(rz->vact4, rz->rdotv)); /* SIGN VCOMP */
    agc_dp_t t = 0;
    agc_status_t st = csi_timethet(ip, rz, v, sc.c[0], sc.c[1], &t);
    if (st == AGC_STATUS_OK) {
        *pop = (rz->rdotv >= 0) ? t : agc_dp_sub(ip, rz->hafpa1, t);
    }
    return st;
}

/* NTP/2 (p.669): TCDH = NN HAFPA1 - pop + TCSI; TTPI - TCDH < 0 ->
 * SCNDSOL with X2 = 5 (returns false).                                  */
static bool csi_ntp2(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t pop,
                     csi_work_t *w)
{
    agc_dp_t nn = agc_dp_from_sp(rz->nn);                    /* DLOAD NN   */
    agc_dp_t t = agc_dp_shl(ip, agc_dp_mul_trunc(nn, rz->hafpa1), 14);
    t = agc_dp_add(ip, agc_dp_sub(ip, t, pop), rz->tcsi);
    rz->tcdh = t;
    w->x2 = 5;
    (void)AGC_ASSERT(rz->tcdh >= AGC_DP_MIN);
    return agc_dp_sub(ip, agc_erase_ttpi(), rz->tcdh) >= 0;
}

/* CSINEXT1 .. CSINEXT2 head (p.669-670): states at CDH (conic), CDHMVR,
 * POSTCDH, active state at TTPI (conic) into RACT3/VACT3.               */
static agc_status_t csi_cdh_tpi(agc_interp_t *ip, agc_rendz_t *rz,
                                agc_job_id_t resume)
{
    agc_vec_t r, v;
    agc_status_t st = rz_intint(ip, rz, rz->ract1, rz->vact4, rz->tcsi,
                                rz->tcdh, true, resume, &r, &v);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz->ract2 = r;
    rz->vact2 = v;
    st = rz_intint(ip, rz, rz->rpass1, rz->vpass1, rz->tcsi, rz->tcdh, true,
                   resume, &r, &v);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz->rpass2 = r;
    rz->vpass2 = v;
    st = agc_rendz_cdhmvr(ip, rz);
    agc_rendz_periapo_t pa;
    if (st == AGC_STATUS_OK) {
        st = agc_rendz_periapo1(ip, rz, rz->ract2, rz->vact3, &pa);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz->postcdh = agc_rendz_shiftr1(rz, pa.hperi);
    st = rz_intint(ip, rz, rz->ract2, rz->vact3, rz->tcdh, agc_erase_ttpi(),
                   true, resume, &r, &v);
    if (st == AGC_STATUS_OK) {
        rz_store_active(rz, r, v);                           /* ACTIVE     */
    }
    return st;
}

/* U = sin(E) URA3 + cos(E) UH3 (B1): the desired line of sight at TPI. */
static agc_vec_t csi_los_unit(agc_interp_t *ip, const agc_rendz_t *rz)
{
    agc_dp_t sine = agc_dp_sin(ip, rz->elev);                /* 6D         */
    agc_vec_t ura3 = rz_unit(ip, rz->ract3);                 /* 0D         */
    agc_vec_t uh3 = rz_unit(ip, agc_vec_cross(ip, rz->up1, ura3));
    agc_vec_t c = agc_vec_scale(ip, uh3, agc_dp_cos(ip, rz->elev)); /* 18D */
    agc_vec_t u = agc_vec_add(ip, agc_vec_scale(ip, ura3, sine), c);
    return agc_vec_shl(ip, u, 1);
}

/* CSINEXT2 (p.670-671): TEMP1 = U.RA3, TEMP2 = TEMP1^2 - RA3^2 + RP3^2
 * (TP). Negative TEMP2 (no intersection of the LOS with the passive
 * radius): LOOPCT = 1 -> ALMXITA (X2 = 1), else halve DELDV and retry.
 * Otherwise *k = the root of smaller magnitude, K1 or K2.               */
static csi_label_t csi_k(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t u,
                         agc_dp_t *k, csi_work_t *w)
{
    agc_dp_t temp1 = agc_dp_shl(ip, agc_vec_dot(ip, u, rz->ract3), 1); /* 6D */
    agc_tp_t t2 = agc_dp_sq_tp(temp1);                                 /* 8D */
    agc_tp_t ra = -agc_vec_dot_tp(ip, rz->ract3, rz->ract3);           /* 11D */
    agc_tp_t rp = agc_vec_dot_tp(ip, rz->rpass3, rz->rpass3);
    agc_tp_t temp2 = agc_tp_add(ip, agc_tp_add(ip, rp, ra), t2);
    if (temp2 < 0) {
        w->x2 = 1;
        if ((agc_dp_t)rz->loopct - CSI_1DPB28 == 0) {
            return CSI_L_ALMXITA;
        }
        rz->deldv = agc_dp_shr(rz->deldv, 1);
        rz->delvcsi = agc_dp_sub(ip, rz->dvprev, rz->deldv);
        return CSI_L_B1;
    }
    agc_dp_t temp3 = agc_tp_sqrt(ip, temp2);                 /* K10RK2 B29 */
    agc_dp_t k2 = agc_dp_sub(ip, agc_dp_neg(temp3), temp1);  /* 10D        */
    agc_dp_t k1 = agc_dp_sub(ip, temp3, temp1);              /* 12D        */
    agc_dp_t d = agc_dp_sub(ip, agc_dp_abs(k2), agc_dp_abs(k1));
    *k = (d < 0) ? k2 : k1;
    return CSI_L_DONE;
}

/* K2. (p.671): GAMMA = SIGN(TEMP) ARCCOS(UNIT(RA3 + K U) . URP3), B1. */
static agc_dp_t csi_gamma(agc_interp_t *ip, const agc_rendz_t *rz,
                          agc_vec_t u, agc_dp_t k)
{
    agc_vec_t ku = agc_vec_shl(ip, agc_vec_scale(ip, u, k), 1);
    agc_vec_t uv = rz_unit(ip, agc_vec_add(ip, ku, rz->ract3));   /* 0D */
    agc_vec_t urp3 = rz_unit(ip, rz->rpass3);                     /* 6D */
    agc_vec_t uvp3 = rz_unit(ip, rz->vpass3);
    agc_vec_t a = agc_vec_cross(ip, uvp3, urp3);                  /* 12D */
    agc_dp_t temp = agc_vec_dot(ip, agc_vec_cross(ip, urp3, uv), a);
    agc_dp_t c = agc_dp_shl(ip, agc_vec_dot(ip, urp3, uv), 1);
    agc_dp_t g = agc_dp_sign(agc_dp_acos(ip, c), temp);
    (void)AGC_ASSERT(g >= -AGC_DP_HALF && g <= AGC_DP_HALF);
    return agc_dp_shr(g, 1);                                      /* SR1 */
}

/* FRSTPAS (p.672): GAMPREV = GAMMA, DVPREV = DELVCSI, step by DELDV. */
static csi_label_t csi_frstpas(agc_interp_t *ip, agc_rendz_t *rz,
                               const csi_work_t *w)
{
    rz->gamprev = w->gamma;
    rz->dvprev = rz->delvcsi;
    rz->delvcsi = agc_dp_sub(ip, rz->delvcsi, rz->deldv);    /* CSINEXT3   */
    csi_set(RZ_VF_CSI_F2, false);
    (void)AGC_ASSERT(!csi_f(RZ_VF_CSI_F2));
    return CSI_L_B1;
}

/* FIFTYFPS (p.673): DELDV = 50 fps signed by slope and GAMPREV; next
 * stage "11"; back to CSI/B2 without counting.                          */
static csi_label_t csi_fiftyfps(agc_interp_t *ip, agc_rendz_t *rz,
                                const csi_work_t *w)
{
    agc_dp_t d = agc_dp_sign(agc_dp_sign(CSI_FIFPSDP, w->slope),
                             rz->gamprev);
    rz->deldv = d;
    rz->delvcsi = agc_dp_add(ip, agc_dp_neg(d), rz->delvcsi);
    csi_set(RZ_VF_CSI_F3B, true);
    csi_set(RZ_VF_CSI_F3A, true);
    rz->gamprev = w->gamma;
    return CSI_L_B2;
}

/* NEWTN, CSISTEP (p.673): Newton step DELDV = GAMMA / slope, limited to
 * 200 fps; converged (< .1 fps) -> CSI/SOL (returned as CSI_L_DONE).    */
static csi_label_t csi_newton(agc_interp_t *ip, agc_rendz_t *rz,
                              const csi_work_t *w, int x1)
{
    int x2 = 0;
    agc_dp_t sn = rz_norm(w->slope, &x2);
    agc_dp_t d = agc_dp_div(ip, w->gamma, sn);               /* BDDV 00D   */
    x1 = x1 - x2;                                            /* XSU,1 X2   */
    rz->deldv = rz_sr_star(ip, d, 0, x1);
    rz->gamprev = w->gamma;
    agc_dp_t ad = agc_dp_abs(rz->deldv);
    if (agc_dp_sub(ip, ad, CSI_EPSILN1) < 0) {
        return CSI_L_DONE;                                   /* CSI/SOL    */
    }
    if (agc_dp_sub(ip, ad, CSI_DELMAX1) >= 0) {
        rz->deldv = agc_dp_sign(CSI_DELMAX1, rz->deldv);
    }
    rz->delvcsi = agc_dp_sub(ip, rz->delvcsi, rz->deldv);    /* CSISTEP    */
    return CSI_L_B1;
}

/* Gamma bookkeeping after K2. (p.671-673): first pass of a cycle, the
 * slope, the cycle stage logic (F3A F3B), Newton iteration.             */
static csi_label_t csi_iterate(agc_interp_t *ip, agc_rendz_t *rz,
                               csi_work_t *w)
{
    if (csi_f(RZ_VF_CSI_F2)) {
        return csi_frstpas(ip, rz, w);
    }
    agc_dp_t dg = agc_dp_sub(ip, w->gamma, rz->gamprev);      /* 2D        */
    int x1 = 0;
    agc_dp_t ddv = rz_norm(agc_dp_sub(ip, rz->delvcsi, rz->dvprev), &x1);
    w->slope = agc_dp_div(ip, dg, ddv);                      /* 4D         */
    rz->dvprev = rz->delvcsi;
    bool f3a = csi_f(RZ_VF_CSI_F3A);
    bool f3b = csi_f(RZ_VF_CSI_F3B);
    if (f3a && f3b) {                        /* "11": end of the 50 fps stage */
        if (agc_dp_mul_trunc(dg, rz->gamprev) >= 0) {
            return csi_fiftyfps(ip, rz, w);
        }
        rz->deldv = agc_dp_sign(CSI_INITST, rz->deldv);       /* INITST1   */
        csi_set(RZ_VF_CSI_F3A, true);
        csi_set(RZ_VF_CSI_F3B, false);
        return csi_frstpas(ip, rz, w);
    }
    if (!f3a && !f3b) {                      /* THRDCHK "00" -> FIFTYFPS    */
        return csi_fiftyfps(ip, rz, w);
    }
    return csi_newton(ip, rz, w, x1);        /* THRDCHK "10" or "01"        */
}

/* CSI/SOL, CSINEXT4 (p.674): perigee and spacing tests of the solution.
 * Returns CSI_L_DONE (go to P32/P72C) or CSI_L_SCNDSOL with X2 set.     */
static csi_label_t csi_sol(agc_interp_t *ip, agc_rendz_t *rz, csi_work_t *w)
{
    agc_dp_t pmin = (rz->body.rtx1 == -10) ? CSI_PMINM : CSI_PMINE;
    w->x2 = 2;
    if (agc_dp_sub(ip, rz->postcsi, pmin) < 0) {
        return CSI_L_SCNDSOL;
    }
    w->x2 = 3;
    if (agc_dp_sub(ip, rz->postcdh, pmin) < 0) {
        return CSI_L_SCNDSOL;
    }
    rz->t1tot2 = agc_dp_sub(ip, rz->tcdh, rz->tcsi);
    w->x2 = 4;
    if (agc_dp_sub(ip, rz->t1tot2, RZ_600SEC) < 0) {
        return CSI_L_SCNDSOL;
    }
    w->x2 = 5;
    rz->t2tot3 = agc_dp_sub(ip, agc_erase_ttpi(), rz->tcdh);
    if (agc_dp_sub(ip, rz->t2tot3, RZ_600SEC) >= 0) {
        return CSI_L_DONE;
    }
    return CSI_L_SCNDSOL;
}

/* SCNDSOL (p.674): in the first cycle ("01") start the second cycle at
 * CSI/B with CSIALRM = X2; otherwise ALMXIT.                            */
static csi_label_t csi_scndsol(agc_rendz_t *rz, const csi_work_t *w)
{
    if (csi_f(RZ_VF_CSI_F3A) || !csi_f(RZ_VF_CSI_F3B)) {
        return CSI_L_ALMXIT;
    }
    rz->csialrm = (agc_sp_t)w->x2;
    csi_set_cycle(false);
    rz->loopct = 0;
    (void)AGC_ASSERT(rz->csialrm >= 1 && rz->csialrm <= 7);
    return CSI_L_B;
}

/* One trial at the current DELVCSI: CSI/B23 .. K2. and the gamma logic. */
static csi_label_t csi_trial(agc_interp_t *ip, agc_rendz_t *rz, csi_work_t *w)
{
    agc_rendz_periapo_t pa;
    bool ellipse = false;
    w->st = csi_b23(ip, rz, &pa, &ellipse);
    bool circl = !ellipse;
    agc_dp_t pop = 0;                                        /* CIRCL: 0D  */
    if (w->st == AGC_STATUS_OK && ellipse) {
        w->st = csi_apsis(ip, rz, &pa, &circl, &pop);
        if (circl) {
            pop = 0;
        }
    }
    if (w->st != AGC_STATUS_OK) {
        return CSI_L_ERROR;
    }
    if (!csi_ntp2(ip, rz, pop, w)) {
        return CSI_L_SCNDSOL;
    }
    w->st = csi_cdh_tpi(ip, rz, w->resume);
    if (w->st != AGC_STATUS_OK) {
        return CSI_L_ERROR;
    }
    agc_vec_t u = csi_los_unit(ip, rz);
    agc_dp_t k = 0;
    csi_label_t l = csi_k(ip, rz, u, &k, w);
    if (l != CSI_L_DONE) {
        return l;
    }
    w->gamma = csi_gamma(ip, rz, u, k);
    l = csi_iterate(ip, rz, w);
    return (l == CSI_L_DONE) ? csi_sol(ip, rz, w) : l;
}

/* ---- CSI/A (p.663-674) ----------------------------------------------------
 * Iterates DELVCSI so that the CDH and TPI conditions are met. OK: the
 * solution (DELVEET1, DELVCSI, TCDH, T1TOT2, T2TOT3, POSTCSI, POSTCDH)
 * is set and the program continues at P32/P72C. ALARM: *alarm is the
 * octal code 00600 + CSIALRM - 1 for ALMXIT.                            */
agc_status_t agc_rendz_csi_a(agc_interp_t *ip, agc_rendz_t *rz,
                             agc_sp_t *alarm, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(alarm != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    csi_work_t w = { 0, 0, 0, resume, AGC_STATUS_OK };
    csi_set_cycle(true);
    rz->loopct = 0;
    rz->csialrm = 0;
    *alarm = 0;
    csi_label_t l = CSI_L_B;
    for (int pass = 0; pass < CSI_MAX_PASSES; pass++) {
        switch (l) {
        case CSI_L_B:       csi_b(ip, rz); l = CSI_L_B1;             break;
        case CSI_L_B1:      l = csi_b1(rz, &w);                      break;
        case CSI_L_B2:
            l = csi_b2(ip, rz, &w) ? csi_trial(ip, rz, &w) : CSI_L_SCNDSOL;
            break;
        case CSI_L_SCNDSOL: l = csi_scndsol(rz, &w);                 break;
        case CSI_L_ALMXITA:
            rz->csialrm = (agc_sp_t)w.x2;
            l = CSI_L_ALMXIT;
            break;
        case CSI_L_ALMXIT:
            *alarm = (agc_sp_t)(AGC_ALARM_CSI_00600 + rz->csialrm - 1);
            return AGC_STATUS_ALARM;
        case CSI_L_DONE:    return AGC_STATUS_OK;
        case CSI_L_ERROR:
        default:            return (w.st != AGC_STATUS_OK) ? w.st
                                                           : AGC_STATUS_BAD_ARG;
        }
    }
    return AGC_STATUS_NO_CONVERGE;  /* DEVIATION: the AGC has no pass bound */
}

/* ---- CDHMVR (p.680-682) ----------------------------------------------------
 * Constant delta height maneuver at CDH: passive advanced to the radial
 * line of RACT2 (TIMETHET), DIFFALT, the passive semimajor axis, then the
 * active velocity VACT3 giving the same apsidal geometry; DELVEET2.      */
static agc_status_t cdh_passive(agc_interp_t *ip, agc_rendz_t *rz,
                                agc_conic_time_out_t *o)
{
    rz->unvec = rz_unit(ip, rz->ract2);                      /* UNVEC      */
    agc_dp_t c = agc_vec_dot(ip, rz_unit(ip, rz->rpass2), rz->unvec); /* 6D */
    agc_dp_t csth = agc_dp_shl(ip, c, 1);
    agc_dp_t s2 = agc_dp_sub(ip, agc_dp_shr(CSI_DP1_4, 2), agc_dp_sq(ip, c));
    agc_dp_t sn = agc_dp_shl(ip, agc_dp_sqrt(ip, s2), 1);    /* 0D         */
    agc_vec_t n = agc_vec_cross(ip, agc_vec_neg(rz->ract2), rz->rpass2);
    agc_dp_t snth = agc_dp_sign(sn, agc_vec_dot(ip, n, rz->up1));
    agc_status_t st = rz_timethet_rv(ip, rz, rz->rpass2, rz->vpass2, snth,
                                     csth, false, o);
    (void)AGC_ASSERT(st != AGC_STATUS_OK || o->r1 >= 0);
    return st;
}

/* Vertical (B7) and horizontal (B7) velocity of the active vehicle. */
static void cdh_speeds(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t vpv,
                       agc_dp_t a_p, agc_dp_t a_a, agc_dp_t ra,
                       agc_dp_t *vav, agc_dp_t *vah)
{
    agc_dp_t q = agc_dp_div(ip, agc_dp_shr(a_p, 2), a_a);    /* 6D  B2     */
    agc_dp_t p = agc_dp_mul(ip, agc_dp_sqrt(ip, q), q);      /* DMPR       */
    *vav = agc_dp_shl_round(ip, agc_dp_mul_trunc(p, vpv), 3); /* 8D B7     */
    int x1 = 0;
    agc_dp_t rn = rz_norm(ra, &x1);
    agc_dp_t t2mu = agc_dp_div(ip, agc_dp_shr(rz->body.rtmu, 1), rn);
    t2mu = rz_sl_star(ip, t2mu, -5, x1);                     /* 10D B14    */
    int x2 = 0;
    agc_dp_t an = rz_norm(a_a, &x2);
    agc_dp_t mua = agc_dp_div(ip, agc_dp_shr(rz->body.rtmu, 1), an);
    mua = rz_sl_star(ip, mua, -6, x2);
    agc_dp_t e = agc_dp_sub(ip, t2mu, mua);                  /* BDSU       */
    *vah = agc_dp_sqrt(ip, agc_dp_sub(ip, e, agc_dp_sq(ip, *vav)));
}

agc_status_t agc_rendz_cdhmvr(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_conic_time_out_t o;
    agc_status_t st = cdh_passive(ip, rz, &o);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    int x2 = rz->body.rtx2;
    agc_vec_t vp = rz_vshift(ip, o.vnew, -x2);               /* VSL* 0,2   */
    agc_dp_t vpv = agc_dp_shl_round(ip, agc_vec_dot(ip, vp, rz->unvec), 1);
    agc_dp_t rp = rz_shift(ip, rz_abval(ip, o.rnew), -x2);   /* 2D -> MPAC */
    agc_dp_t ra = rz_abval(ip, rz->ract2);                   /* 2D         */
    rz->diffalt = agc_dp_sub(ip, rp, ra);
    int x1 = 0;
    agc_dp_t r1an = rz_norm(o.r1a, &x1);                     /* 4D         */
    agc_dp_t r1 = agc_dp_shr_round(agc_rendz_shiftr1(rz, o.r1), 1);
    agc_dp_t a_p = rz_sl_star(ip, agc_dp_div(ip, r1, r1an), -5, x1);
    agc_dp_t a_a = agc_dp_sub(ip, a_p, rz->diffalt);         /* 4D A SUB A */
    agc_dp_t vav = 0;
    agc_dp_t vah = 0;
    cdh_speeds(ip, rz, vpv, a_p, a_a, ra, &vav, &vah);
    agc_vec_t h = agc_vec_scale(ip, rz_unit(ip, agc_vec_cross(ip, rz->up1,
                                                              rz->unvec)),
                                vah);
    agc_vec_t v = agc_vec_add(ip, agc_vec_scale(ip, rz->unvec, vav), h);
    rz->vact3 = agc_vec_shl(ip, v, 1);
    rz->delveet2 = agc_vec_sub(ip, rz->vact3, rz->vact2);
    return AGC_STATUS_OK;
}

/* ---- S32/33.X (p.679): rows -URA1 x UP1 (VSL1), -UP1, -URA1 ------------- */
agc_status_t agc_rendz_csi_cdh_lv_matrix(agc_interp_t *ip, const agc_rendz_t *rz,
                                agc_mat_t *m)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(m != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    m->row[1] = agc_vec_neg(rz->up1);                        /* 6D         */
    m->row[2] = agc_vec_neg(rz_unit(ip, rz->ract1));         /* 12D        */
    m->row[0] = agc_vec_shl(ip, agc_vec_cross(ip, m->row[2], rz->up1), 1);
    return AGC_STATUS_OK;
}

/* ---- S32/33.1 tail (p.678): DELVSIN = M^T DELVLVC (VXM, VSL1), DELVSAB.
 * The AGC returns DELVSIN in MPAC; the program stores it as DELVEET1/2. */
agc_status_t agc_rendz_csi_cdh_dv_to_reference(agc_interp_t *ip, agc_rendz_t *rz,
                                agc_vec_t *delvsin)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(delvsin != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_mat_t m;
    agc_status_t st = agc_rendz_csi_cdh_lv_matrix(ip, rz, &m);
    rz->delvsin = agc_vec_shl(ip, agc_vec_mat(ip, rz->delvlvc, &m), 1);
    rz->delvsab = rz_abval(ip, rz->delvsin);
    *delvsin = rz->delvsin;
    return st;
}
