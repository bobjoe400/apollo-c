/*
 * measurement_update_lm.c -- LSR22.3 / LSR22.4, LM only (P20-P25.agc,
 * Luminary099 pp.574-587): the rendezvous radar measurement incorporation
 * of R22LEM, with GETULC, RADARANG, LGCUPDTE, WLINIT and INTGRCAL.
 *
 * "This routine computes the B-vectors and delta Q for each of the
 * quantities measured by the rendezvous radar (range, range rate, shaft
 * and trunnion angles). The routine calls the INCORP1 and INCORP2
 * routines which compute the deviations and correct the state vector. If
 * the computed state vector deviations exceed the maximum permitted, the
 * routine returns to R22LEM to display the deviations." LSR22.4 is the
 * lunar surface entry: range and range rate only, 6x6 W, CSM updated.
 *
 * Both state vectors are first integrated to the mark time (the updated
 * vehicle with its W matrix); the four measurements then run in order,
 * each one INCORP1, the N49 check (R22LEM96, an asynchronous display),
 * INCORP2.
 *
 * Scales ("B-n" in the AGC comments = scaled 2^n): ULC half unit; RLC
 * body scale (earth 2^29, moon 2^27; X2 = SCALSHFT = 2 earth, 0 moon);
 * DELTAQ, BVECTOR and VARIANCE (TP, B-40) as noted per measurement.
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/guidance/guidance.h"
#include "agc/nav/auxiliary.h"
#include "agc/io/service_internal.h"
#include "../../programs/navigation/internal.h"
#include "../radar/radar.h"

#define MU_IMUVAR   ((agc_dp_t)1099512)    /* IMUVAR 2DEC E-6 B12: rad^2  */
#define MU_2PI_8    ((agc_dp_t)210828714)  /* 2PI/8 2DEC 3.141592653 B-2  */
#define MU_STEPS    6                      /* bound on the measurement loop */

typedef enum mu_step {
    MU_RANGE = 1, MU_RATE = 2, MU_SHAFT = 3, MU_TRUNNION = 4, MU_END = 5
} mu_step_t;

/* LSR22.3 erasables (ULC, RXZ, SINTHETA, MX MY MZ, SCALSHFT) and the
 * INCORP1 result kept across the noun 49 display.                     */
typedef struct mu_state {
    mu_step_t    step;       /* the measurement in work (WHCHREAD)        */
    int          x2;         /* SCALSHFT: 2 earth, 0 moon                 */
    agc_vec_t    ulc;        /* ULC: unit LM-to-CSM, half unit            */
    agc_dp_t     rlc;        /* 36D: |CSM - LM|, body scale               */
    agc_vec_t    mx, my, mz; /* MX MY MZ: NB axes in reference, half unit */
    agc_dp_t     rxz;        /* RXZ: RLC cos theta, normalized            */
    int          rxz_x1;     /* X1 after NORM of RXZ (-shifts)            */
    agc_dp_t     sintheta;   /* SINTHETA: -ULC . MY, 2^1                  */
    agc_incorp_t inc;        /* BVECTOR DELTAQ VARIANCE .. DELTAX         */
} mu_state_t;

static mu_state_t mu;

/* ---- shifts with an index register (count = addr - C(X)) ----------- */
static agc_dp_t mu_srr_star(agc_interp_t *ip, agc_dp_t a, int count)
{
    (void)AGC_ASSERT(count >= -40 && count <= 40);
    return (count >= 0) ? agc_dp_shr_round(a, count) : agc_dp_shl_round(ip, a, -count);
}

static agc_dp_t mu_slr_star(agc_interp_t *ip, agc_dp_t a, int count)
{
    (void)AGC_ASSERT(count >= -40 && count <= 40);
    return (count >= 0) ? agc_dp_shl_round(ip, a, count) : agc_dp_shr_round(a, -count);
}

/* SR* on the TP MPAC: right by count (left when negative).             */
static agc_tp_t mu_tp_sr(agc_interp_t *ip, agc_tp_t v, int count)
{
    (void)AGC_ASSERT(count >= -60 && count <= 60);
    return agc_tp_shift(ip, v, -count);
}

/* B0 complemented when the LM is the vehicle being corrected (BON
 * VEHUPFLG +1), or when the CSM is (BOFF VEHUPFLG +1).                  */
static agc_vec_t mu_sign(agc_vec_t b, bool comp_if_csm)
{
    bool csm = agc_flag_test(AGC_FLAG_VEHUPFLG);
    (void)AGC_ASSERT(b.c[0] >= AGC_DP_MIN);
    return (csm == comp_if_csm) ? agc_vec_neg(b) : b;
}

/* ---- INTGRCAL / WLINIT (p.584) ---------------------------------------- */
/* INTGRCAL: TDEC1 = MKTIME, INTEGRV (which releases INTSTALL).          */
static agc_status_t mu_intgrcal(agc_interp_t *ip)
{
    agc_status_t st = agc_integ_integrv(ip, agc_integ_shared_ctx(),
                                        lmnav_state()->mktime);
    rz_phase(2, 04022);                                       /* GRP2PC   */
    return st;
}

/* WLINIT: W = 0 except the diagonal: position WRENDPOS (surface WSURFPOS)
 * SR 5 (to B-19), velocity WRENDVEL (WSURFVEL), shaft and trunnion bias
 * WSHAFT, WTRUN; RENDWFLG on, MARKCTR = 0.                              */
static void mu_wlinit(void)
{
    agc_incorp_w_t *w = &agc_integ_shared_ctx()->w;
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
    bool surf = agc_flag_test(AGC_FLAG_SURFFLAG);
    for (int b = 0; b < AGC_INCORP_BLKS; b++) {
        for (int m = 0; m < AGC_INCORP_ROWS; m++) {
            w->blk[b][m] = agc_vec_zero();
        }
    }
    agc_dp_t pos = agc_dp_shr(agc_dp_from_sp(surf ? p->wsurfpos : p->wrendpos), 5);
    agc_dp_t vel = agc_dp_from_sp(surf ? p->wsurfvel : p->wrendvel);
    for (int k = 0; k < 3; k++) {
        w->blk[0][k].c[k] = pos;                              /* W +0,8,16 */
        w->blk[1][3 + k].c[k] = vel;                          /* W +72,80,88 */
    }
    w->blk[2][6].c[0] = agc_dp_from_sp(p->wshaft);            /* W +144   */
    w->blk[2][7].c[1] = agc_dp_from_sp(p->wtrun);             /* W +152   */
    rz_flag_put(AGC_FLAG_RENDWFLG, true);
    agc_erase_set_trkmkcnt(0);
}

/* ---- the two integrations (LSR22.3 pp.575-576, LSR22.4 p.583) ---------
 * CSM corrected: LM to MKTIME, then the CSM with W; LM corrected: the
 * other way round. STATEFLG: the results become the permanent states.  */
static agc_status_t mu_integrate(agc_interp_t *ip, agc_job_id_t resume)
{
    bool csm = agc_flag_test(AGC_FLAG_VEHUPFLG);
    agc_status_t st = agc_integ_install(0, resume);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_flag_put(AGC_FLAG_VINTFLAG, !csm);    /* the other vehicle first */
    agc_status_t sf = agc_integ_setiflgs();
    (void)AGC_ASSERT(sf == AGC_STATUS_OK);
    st = mu_intgrcal(ip);
    if (st == AGC_STATUS_OK) {
        st = agc_integ_install(0, resume);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    bool w = agc_flag_test(AGC_FLAG_RENDWFLG);
    rz_flag_put(AGC_FLAG_DIM0FLAG, w);                       /* NOTWCSM  */
    if (w) {
        rz_flag_put(AGC_FLAG_D6OR9FLG, true);
    }
    rz_flag_put(AGC_FLAG_VINTFLAG, csm);
    rz_flag_put(AGC_FLAG_INTYPFLG, false);
    rz_flag_put(AGC_FLAG_STATEFLG, true);
    return mu_intgrcal(ip);
}

/* LSR22.4: the LM (from the landing site) and the CSM, 6x6 W; the first
 * mark initializes W (INITWMX6) and integrates the CSM without it.     */
static agc_status_t mu_integrate_surface(agc_interp_t *ip, agc_job_id_t resume)
{
    agc_status_t st = agc_integ_install(0, resume);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_flag_put(AGC_FLAG_STATEFLG, true);
    rz_flag_put(AGC_FLAG_VINTFLAG, false);
    st = mu_intgrcal(ip);
    rz_flag_put(AGC_FLAG_DMENFLG, false);           /* 6x6 for INCORP  */
    if (st == AGC_STATUS_OK) {
        st = agc_integ_install(0, resume);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (agc_erase_trkmkcnt() == 0) {                          /* INITWMX6 */
        mu_wlinit();
        rz_flag_put(AGC_FLAG_VINTFLAG, true);
        agc_status_t sf = agc_integ_setiflgs();
        (void)AGC_ASSERT(sf == AGC_STATUS_OK);
        return mu_intgrcal(ip);
    }
    rz_flag_put(AGC_FLAG_D6OR9FLG, false);
    rz_flag_put(AGC_FLAG_DIM0FLAG, true);
    rz_flag_put(AGC_FLAG_VINTFLAG, true);
    rz_flag_put(AGC_FLAG_INTYPFLG, false);
    return mu_intgrcal(ip);
}

/* ---- GETULC / RADARANG (pp.585-586) ----------------------------------- */
/* GETULC: CSM - LM from the conic states plus deviations (TDELTAV shifted
 * from 2^22 / 2^18 to the body scale), unitized with NORMUNIT; RLC.     */
static agc_status_t mu_getulc(agc_interp_t *ip)
{
    agc_integ_sv_t l;
    agc_integ_sv_t c;
    agc_status_t st = agc_integ_perm_get(AGC_VEHICLE_LM, &l);
    if (st == AGC_STATUS_OK) {
        st = agc_integ_perm_get(AGC_VEHICLE_CSM, &c);
    }
    int k = 9 - mu.x2;                                        /* VSR* 9D,2 */
    agc_vec_t rl = agc_vec_add(ip, agc_vec_shr(l.deltav, k), l.rcv);
    agc_vec_t rc = agc_vec_add(ip, agc_vec_shr(c.deltav, k), c.rcv);
    int x1 = 0;
    agc_dp_t mag = 0;
    mu.ulc = agc_rtb_normunit(ip, agc_vec_sub(ip, rc, rl), &x1, &mag, NULL);
    mu.rlc = rz_shift(ip, mag, -x1);                          /* SL* 0,1  */
    return st;
}

/* RADARANG: SINTHETA = -ULC . MY (SL1R); RXZ = RLC cos(theta), NORM.   */
static agc_status_t mu_radarang(agc_interp_t *ip)
{
    agc_status_t st = mu_getulc(ip);
    agc_dp_t s = agc_vec_dot(ip, agc_vec_neg(mu.ulc), mu.my);
    mu.sintheta = agc_dp_shl_round(ip, s, 1);
    agc_dp_t c = agc_dp_sqrt(ip, agc_dp_sub(ip, AGC_DP_QUARTER,
                                            agc_dp_sq(ip, mu.sintheta)));
    c = agc_dp_shl(ip, agc_dp_mul_trunc(c, mu.rlc), 1);
    mu.rxz = rz_norm(c, &mu.rxz_x1);
    return st;
}

/* ---- the measurements (RANGEBQ .. TRUNBQ pp.576-582) ------------------ */
/* Range: B0 = ULC (complemented if the LM is corrected), DELTAQ = RM -
 * RLC, VARIANCE = max(RLC^2 RANGEVAR, RVARMIN) (TP B-40).              */
static agc_status_t mu_range(agc_interp_t *ip)
{
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
    agc_tp_t vmin = (agc_tp_t)p->rvarmin;          /* SR 28, TPMODE: B-40 */
    agc_status_t st = mu_getulc(ip);
    mu.inc.bvector[0] = mu_sign(mu.ulc, false);
    mu.inc.bvector[1] = agc_vec_zero();
    mu.inc.bvector[2] = agc_vec_zero();
    agc_dp_t r = agc_dp_sub(ip, lmnav_state()->rm, mu_srr_star(ip, mu.rlc, 2 - mu.x2));
    mu.inc.deltaq = mu_slr_star(ip, r, 2 - mu.x2);           /* B-29/B-27 */
    int x1 = 0;
    agc_dp_t n = rz_norm(mu.rlc, &x1);
    agc_tp_t v = agc_dp_mul_tp(agc_dp_sq(ip, n), p->rangevar);
    v = mu_tp_sr(ip, v, -2 - x1);                             /* SR* 0-2,1 */
    v = mu_tp_sr(ip, v, 0 - x1);
    v = mu_tp_sr(ip, v, 0 - mu.x2);
    v = mu_tp_sr(ip, v, 0 - mu.x2);
    mu.inc.variance = (vmin - v < 0) ? v : vmin;              /* QOK      */
    return st;
}

/* Range rate: B1 = ULC RLC (B-24/B-22), B0 = (ULC x VLC) x ULC (B-5),
 * DELTAQ = (RDOTM - ULC . VLC) RLC (B-33), VARIANCE = max(RDOT^2 RATEVAR,
 * VVARMIN) RLC^2; B0, B1, DELTAQ normalized together (the smaller shift
 * of B0 and B1).                                                       */
static agc_status_t mu_rate_vectors(agc_interp_t *ip, agc_vec_t *vlc)
{
    agc_integ_sv_t l;
    agc_integ_sv_t c;
    agc_status_t st = mu_getulc(ip);
    if (st == AGC_STATUS_OK) {
        st = agc_integ_perm_get(AGC_VEHICLE_LM, &l);
    }
    if (st == AGC_STATUS_OK) {
        st = agc_integ_perm_get(AGC_VEHICLE_CSM, &c);
    }
    mu.rlc = rz_sr_star(ip, mu.rlc, -4, mu.x2);               /* to B-23  */
    mu.inc.bvector[1] = agc_vec_scale(ip, mu_sign(mu.ulc, false), mu.rlc);
    int k = 6 - mu.x2;                                        /* VSR* 6,2 */
    agc_vec_t vl = agc_vec_add(ip, agc_vec_shr(l.nuv, k), l.vcv);
    agc_vec_t vc = agc_vec_add(ip, agc_vec_shr(c.nuv, k), c.vcv);
    *vlc = agc_vec_sub(ip, vc, vl);
    return st;
}

static agc_status_t mu_rate(agc_interp_t *ip)
{
    agc_vec_t vlc;
    agc_status_t st = mu_rate_vectors(ip, &vlc);
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
    agc_dp_t rdot = agc_vec_dot(ip, mu.ulc, vlc);             /* B-8/B-6  */
    agc_dp_t r8 = mu_srr_star(ip, rdot, 2 - mu.x2);           /* B-8      */
    agc_dp_t var = agc_dp_mul(ip, agc_dp_sq(ip, r8), p->ratevar);
    agc_dp_t vmin = agc_dp_shr(agc_dp_from_sp(p->vvarmin), 16);   /* B-4 */
    var = (agc_dp_sub(ip, vmin, var) < 0) ? var : vmin;       /* VOK      */
    agc_dp_t dq = agc_dp_sub(ip, mu_slr_star(ip, lmnav_state()->rdotm, -1 - mu.x2),
                             agc_dp_shr(rdot, 2));      /* PD12 SR2 */
    dq = agc_dp_mul(ip, dq, mu.rlc);                          /* B-33     */
    agc_vec_t b0 = agc_vec_cross(ip, agc_vec_cross(ip, mu.ulc, vlc), mu.ulc);
    b0 = rz_vshift(ip, mu_sign(b0, false), 2 + mu.x2);        /* VSR* 0-2,2 */
    mu.inc.bvector[0] = b0;
    mu.inc.bvector[2] = agc_vec_zero();
    int s0 = 0;
    int s1 = 0;
    (void)rz_norm(agc_vec_abval(ip, b0), &s0);                /* 20D      */
    (void)rz_norm(agc_vec_abval(ip, mu.inc.bvector[1]), &s1); /* 22D      */
    int x1 = (s1 - s0 < 0) ? s0 : s1;                         /* VOK1/VOK2 */
    mu.inc.bvector[0] = rz_vshift(ip, b0, -x1);               /* VSL* 0,1 */
    mu.inc.bvector[1] = rz_vshift(ip, mu.inc.bvector[1], -x1);
    mu.inc.deltaq = rz_shift(ip, dq, -x1);
    agc_dp_t rl = rz_shift(ip, mu.rlc, -1 - x1);              /* SL* 0-1,1 */
    agc_tp_t v = agc_dp_mul_tp(agc_dp_sq(ip, rl), var);
    mu.inc.variance = agc_tp_shift(ip, v, 4);                 /* SL4, TP  */
    return st;
}

/* MXMYMZ (p.579): the NB unit axes at the mark (CDUs AIG AMG AOG) in
 * reference axes: M = (UNIT NB ->SM) x REFSMMAT, VSL1.                 */
static agc_status_t mu_mxmymz(agc_interp_t *ip)
{
    lmnav_state_t *n = lmnav_state();
    agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_cdutrig_t trig;
    uint16_t yzx[3] = { n->aig, n->amg, n->aog };
    agc_vec_t ax[3] = { agc_vec_make(AGC_DP_HALF, 0, 0),
                        agc_vec_make(0, AGC_DP_HALF, 0),
                        agc_vec_make(0, 0, AGC_DP_HALF) };
    agc_status_t st = agc_guid_trg_nbsm(&trig, yzx, &ax[0]);      /* UNITX */
    for (int i = 1; i < 3 && st == AGC_STATUS_OK; i++) {
        st = agc_guid_nbsm(&trig, &ax[i]);                        /* *NBSM* */
    }
    mu.mx = agc_vec_shl(ip, agc_vec_mat(ip, ax[0], &refsmmat), 1);
    mu.my = agc_vec_shl(ip, agc_vec_mat(ip, ax[1], &refsmmat), 1);
    mu.mz = agc_vec_shl(ip, agc_vec_mat(ip, ax[2], &refsmmat), 1);
    return st;
}

/* Angle measurement common part: B2 has RXZ (SR* 0-2,1, SRR* 0,2) in
 * component `bias` (shaft 0, trunnion 1); VARIANCE = (VAR + IMUVAR)
 * RXZ^2 (B-40); DELTAQ = (measured - computed angle (radians) - X789
 * bias) RXZ, un-normalized.                                            */
static void mu_angle(agc_interp_t *ip, int bias, agc_sp_t var, agc_dp_t dangle)
{
    int nx = -mu.rxz_x1;                                      /* shifts   */
    agc_vec_t b2 = agc_vec_zero();
    b2.c[bias] = mu_srr_star(ip, rz_sr_star(ip, mu.rxz, -2, mu.rxz_x1), 0 - mu.x2);
    mu.inc.bvector[1] = agc_vec_zero();
    mu.inc.bvector[2] = b2;
    agc_dp_t v = agc_dp_add(ip, agc_dp_from_sp(var), MU_IMUVAR);
    agc_tp_t t = agc_tp_shift(ip, agc_dp_mul_tp(v, mu.rxz), -nx);  /* SRR* 0,1 */
    t = agc_dp_mul_tp(agc_tp_round(ip, t), mu.rxz);
    t = mu_tp_sr(ip, t, nx - 2);                              /* SR* 0-2,1 */
    t = mu_tp_sr(ip, t, 0 - mu.x2);
    mu.inc.variance = mu_tp_sr(ip, t, 0 - mu.x2);
    agc_dp_t xb = agc_dp_shl(ip, agc_erase_x789().c[bias], 3);    /* SL3 */
    xb = mu_srr_star(ip, xb, 0 - mu.x2);         /* from B-5/B-3 to B0  */
    agc_tp_t dq = agc_dp_mul_tp(agc_dp_sub(ip, dangle, xb), mu.rxz);
    mu.inc.deltaq = agc_tp_round(ip, agc_tp_shift(ip, dq, -nx));  /* SRR* 0,1 */
}

/* Shaft (SHAFTBQ p.580): angle = ARCTAN(ULC . MX, ULC . MZ); B0 =
 * UNIT(ULC x MY), complemented if the CSM is corrected.                */
static agc_status_t mu_shaft(agc_interp_t *ip)
{
    agc_status_t st = mu_mxmymz(ip);
    if (st == AGC_STATUS_OK) {
        st = mu_radarang(ip);
    }
    agc_dp_t sinth = agc_dp_shl(ip, agc_vec_dot(ip, mu.ulc, mu.mx), 1);
    agc_dp_t costh = agc_dp_shl(ip, agc_vec_dot(ip, mu.ulc, mu.mz), 1);
    agc_dp_t a = agc_latlong_arctan(ip, sinth, costh);
    agc_dp_t d = agc_dp_sub(ip, lmnav_state()->rrshaft, a);
    d = agc_dp_shl_round(ip, agc_dp_mul_trunc(d, MU_2PI_8), 3);   /* SL3R */
    agc_vec_t b0 = agc_vec_shl(ip, agc_vec_cross(ip, mu.ulc, mu.my), 1);
    mu.inc.bvector[0] = mu_sign(agc_vec_unit(ip, b0, NULL, NULL), true);
    mu_angle(ip, 0, agc_rendz_nav_pad()->shaftvar, d);
    return st;
}

/* Trunnion (TRUNBQ p.581): angle = ASIN(SINTHETA); B0 = (ULC x MY) x
 * ULC, complemented if the CSM is corrected.                           */
static agc_status_t mu_trunnion(agc_interp_t *ip)
{
    agc_status_t st = mu_radarang(ip);
    agc_vec_t b0 = agc_vec_shl(ip, agc_vec_cross(ip, mu.ulc, mu.my), 1);
    b0 = agc_vec_shl(ip, agc_vec_cross(ip, b0, mu.ulc), 1);
    mu.inc.bvector[0] = mu_sign(b0, true);
    agc_dp_t d = agc_dp_sub(ip, lmnav_state()->rrtrun, agc_dp_asin(ip, mu.sintheta));
    d = agc_dp_shl_round(ip, agc_dp_mul_trunc(d, MU_2PI_8), 3);
    mu_angle(ip, 1, agc_rendz_nav_pad()->trunvar, d);
    return st;
}

/* ---- LGCUPDTE / ASTOK (p.586) ------------------------------------------ */
/* INCORP1, then the deviations for noun 49 (R22DISP: |dR| B-29, |dV|
 * B-7). True when they exceed RMAX or VMAX (R22LEM96).                 */
static bool mu_incorp1(agc_interp_t *ip, agc_status_t *st)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    lmnav_state_t *n = lmnav_state();
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
    mu.inc.dim9 = agc_flag_test(AGC_FLAG_DMENFLG);
    *st = agc_incorp1(ip, &ctx->w, &mu.inc);
    n->r22disp[1] = mu_srr_star(ip, agc_vec_abval(ip, mu.inc.deltax[1]), 2 - mu.x2);
    n->r22disp[0] = mu_srr_star(ip, agc_vec_abval(ip, mu.inc.deltax[0]), 2 - mu.x2);
    agc_dp_t rmax = agc_dp_shr(agc_dp_from_sp(p->rmax), 10);
    if (agc_dp_sub(ip, rmax, n->r22disp[0]) < 0) {
        return true;
    }
    return agc_dp_sub(ip, agc_dp_from_sp(p->vmax), n->r22disp[1]) < 0;
}

/* INCORP2 of the waiting measurement (MOONTHIS = LMOONFLG).           */
static agc_status_t mu_incorp2(agc_interp_t *ip)
{
    return rz_incorp2(ip, &mu.inc, agc_flag_test(AGC_FLAG_LMOONFLG));
}

/* ---- the measurement sequence ------------------------------------------- */
/* One measurement: its B vectors, DELTAQ, VARIANCE.                     */
static agc_status_t mu_compute(agc_interp_t *ip, mu_step_t step)
{
    lmnav_state()->whchread = (agc_sp_t)step;                 /* WHCHREAD */
    rz_phase(2, 04022);                                       /* GRP2PC   */
    switch (step) {
    case MU_RANGE:    return mu_range(ip);
    case MU_RATE:     return mu_rate(ip);
    case MU_SHAFT:    return mu_shaft(ip);
    case MU_TRUNNION: return mu_trunnion(ip);
    case MU_END:
    default:
        (void)AGC_ASSERT(false);
        return AGC_STATUS_BAD_ARG;
    }
}

/* The next measurement after an incorporated one: on the surface the
 * angles are not done (RENDEND after range rate).                     */
static mu_step_t mu_next(mu_step_t step)
{
    if (step == MU_RATE && agc_flag_test(AGC_FLAG_SURFFLAG)) {
        return MU_END;
    }
    return (step >= MU_TRUNNION) ? MU_END : (mu_step_t)((int)step + 1);
}

/* Measurements from mu.step on: each INCORP1; too large a change stops
 * here for the noun 49 display (R22LEM96); else INCORP2 (ASTOK). At the
 * end R22LEM93.                                                          */
static agc_status_t mu_run(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    for (int i = 0; i < MU_STEPS; i++) {
        if (mu.step == MU_END) {
            return agc_job_rendezvous_tracking_data_done(ctx);    /* RENDEND */
        }
        agc_status_t st = mu_compute(ip, mu.step);
        bool big = false;
        if (st == AGC_STATUS_OK) {
            big = mu_incorp1(ip, &st);
        }
        if (st != AGC_STATUS_OK) {
            return st;
        }
        if (big) {
            return lmnav_n49_display();                       /* R22LEM96 */
        }
        st = mu_incorp2(ip);                                  /* ASTOK    */
        if (st != AGC_STATUS_OK) {
            return st;
        }
        rz_phase(2, 04022);
        mu.step = mu_next(mu.step);
    }
    (void)AGC_ASSERT(false);
    return AGC_STATUS_NO_CONVERGE;
}

/* ASTOK after the crew accepted the noun 49 display (R22LEM7): INCORP2
 * of the waiting measurement, then the rest.                           */
agc_status_t lmnav_incorporate_accepted(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(mu.step != MU_END)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = mu_incorp2(&ip);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_phase(2, 04022);
    mu.step = mu_next(mu.step);
    return mu_run(ctx, &ip);
}

/* LSR22.3 / LSR22.4: states to the mark time; W reinitialized if it was
 * invalidated (MARKTEST); then (RANGEBQ) R65 once unless on the surface,
 * and the measurements.                                                  */
agc_status_t agc_job_rendezvous_tracking_data_incorporate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);                                       /* GRP2PC   */
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st;
    if (agc_flag_test(AGC_FLAG_SURFFLAG)) {
        st = mu_integrate_surface(&ip, AGC_JOB_RENDEZVOUS_TRACKING_DATA_INCORPORATE);
    } else {
        rz_flag_put(AGC_FLAG_DMENFLG, true);
        st = mu_integrate(&ip, AGC_JOB_RENDEZVOUS_TRACKING_DATA_INCORPORATE);
        if (st == AGC_STATUS_OK && !agc_flag_test(AGC_FLAG_RENDWFLG)) {
            mu_wlinit();                                      /* MARKTEST */
        }
    }
    if (st != AGC_STATUS_OK) {
        return st;                        /* BUSY: asleep in INTSTALL  */
    }
    if (agc_flag_test(AGC_FLAG_SURFFLAG)) {
        return agc_job_rendezvous_tracking_data_measure(ctx);
    }
    lmnav_state()->r65cntr = 0;
    return lmnav_tracking_attitude(ctx, false, AGC_JOB_RENDEZVOUS_TRACKING_DATA_MEASURE);
}

/* RANGEBQ1: SCALSHFT (0 moon, 2 earth), then the measurements.          */
agc_status_t agc_job_rendezvous_tracking_data_measure(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    mu.x2 = agc_flag_test(AGC_FLAG_LMOONFLG) ? 0 : 2;
    rz_phase(2, 04022);
    mu.step = MU_RANGE;
    agc_interp_t ip;
    agc_interp_init(&ip);
    return mu_run(ctx, &ip);
}
