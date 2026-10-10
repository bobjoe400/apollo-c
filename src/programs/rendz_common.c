/*
 * rendz_common.c -- rendezvous erasables and the shared rendezvous
 * subroutines of P34-35_P74-75.agc (Comanche pp.469-500, Luminary
 * equivalent): SELECTMU, VECSHIFT, SHIFTR1, PERIAPO/PERIAPO1/SETRAD,
 * PRECSET/ACTIVE/PASSIVE, INTINT, S34/35.1, LOMAT, MIDGIM, AVFLAGA/P,
 * P20FLGON, plus the numeric helpers used by every rendezvous file.
 *
 * Scales (from the AGC text): r m 2^29, v m/cs 2^7, t cs 2^28, angles
 * revolutions 2^0, unit vectors 2^1, mu 2^36. "Body scale" is 2^27/2^5
 * when RTX2 = 2 (moon).
 */
#include "agc/agc_assert.h"
#include "agc/agc_flags_common.h"
#include "agc/agc_guid.h"
#include "rendz_internal.h"

/* The rendezvous erasables (E-bank storage of ERASABLE_ASSIGNMENTS). */
static agc_rendz_t rendz_erasable;

/* MUTABLE (CONIC_SUBROUTINES p.1307, LM CONTROLLED_CONSTANTS):
 * MUE   2DEC* 3.986032 E10 B-36*    -> 155704375
 * 1/SQRT(MUE) .50087529 E-5 B+17*   -> 176229826
 * MUM   2DEC 4.902778 E8 B-30       -> 122569450
 * 1/SQRT(MUM) .45162595 E-4 B+14*   -> 198627193 */
static const agc_dp_t mutable_mue = 155704375;
static const agc_dp_t mutable_rtmue_inv = 176229826;
static const agc_dp_t mutable_mum = 122569450;
static const agc_dp_t mutable_rtmum_inv = 198627193;

/* MUTABLE entry for the primary body: MUE 2^36 (earth) or MUM 2^30 (moon),
 * i.e. DLOAD* MUTABLE -2,1 with X1 = RTX1.                              */
agc_dp_t rz_mutable_mu(bool moon)
{
    agc_dp_t mu = moon ? mutable_mum : mutable_mue;
    (void)AGC_ASSERT(mu > 0);
    return mu;
}

agc_rendz_t *agc_rendz_erasable(void)
{
    (void)AGC_ASSERT(rendz_erasable.body.rtx1 <= 0);
    (void)AGC_ASSERT(rendz_erasable.body.rtx2 >= 0);
    return &rendz_erasable;
}

agc_rendz_t *rz_state(void)
{
    return agc_rendz_erasable();
}

/* TRKMKCNT (B(1)PRM): rendezvous tracking mark count (P20, AVETOMID). */
agc_sp_t agc_erase_trkmkcnt(void)
{
    (void)AGC_ASSERT(rendz_erasable.trkmkcnt >= 0);
    return rendz_erasable.trkmkcnt;
}

void agc_erase_set_trkmkcnt(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= 0 && v <= AGC_SP_MAX)) {
        return;
    }
    rendz_erasable.trkmkcnt = v;
}

/* Fresh start: all rendezvous erasables zero, earth selected. */
void agc_rendz_reset(agc_rendz_t *rz)
{
    if (!AGC_ASSERT(rz != NULL)) {
        return;
    }
    static const agc_rendz_t zero;
    *rz = zero;
    (void)agc_rendz_selectmu(rz, false);
    (void)AGC_ASSERT(rz->body.rtx1 == -2);
}

/* ---- vehicle-dependent flags ------------------------------------------- */
bool rz_is_csm(void)
{
    agc_vehicle_t v = agc_state_vehicle();
    (void)AGC_ASSERT(v == AGC_VEHICLE_CSM || v == AGC_VEHICLE_LM);
    return v == AGC_VEHICLE_CSM;
}

agc_flag_id_t rz_vflag(rz_vflag_t f)
{
    if (!AGC_ASSERT((int)f >= 0 && f < RZ_VF_COUNT)) {
        return rz_vflag_csm(RZ_VF_ITSWICH);
    }
    return rz_is_csm() ? rz_vflag_csm(f) : rz_vflag_lm(f);
}

/* A flag that does not exist in this vehicle reads as clear. */
bool rz_vflag_test(rz_vflag_t f)
{
    (void)AGC_ASSERT((int)f >= 0 && f < RZ_VF_COUNT);
    agc_flag_id_t id = rz_vflag(f);
    return (id != 0U) && agc_flag_test(id);
}

agc_status_t rz_flag_put(agc_flag_id_t f, bool on)
{
    (void)AGC_ASSERT(f != 0U);
    return on ? agc_flag_set(f) : agc_flag_clear(f);
}

agc_status_t rz_vflag_set(rz_vflag_t f, bool on)
{
    if (!AGC_ASSERT((int)f >= 0 && f < RZ_VF_COUNT)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_flag_id_t id = rz_vflag(f);
    if (!AGC_ASSERT(id != 0U)) {
        return AGC_STATUS_BAD_ARG;          /* not a flag of this vehicle */
    }
    return rz_flag_put(id, on);
}

/* MOONTHIS EQUALS CMOONFLG (CM) / LMOONFLG (LM), TAGS_FOR_RELATIVE_SETLOC */
bool rz_moonthis(void)
{
    agc_flag_id_t f = rz_is_csm() ? AGC_FLAG_CMOONFLG : AGC_FLAG_LMOONFLG;
    (void)AGC_ASSERT(f != 0U);
    return agc_flag_test(f);
}

/* ---- numeric helpers ---------------------------------------------------- */
/* UNIT: half-unit vector (scale 2^1) */
agc_vec_t rz_unit(agc_interp_t *ip, agc_vec_t v)
{
    (void)AGC_ASSERT(ip != NULL);
    (void)AGC_ASSERT(v.c[0] != AGC_DP_MIN);
    return agc_vec_unit(ip, v, NULL, NULL);
}

/* SIGN on a vector: complement v when s is negative. */
agc_vec_t rz_vsign(agc_vec_t v, agc_dp_t s)
{
    (void)AGC_ASSERT(s >= AGC_DP_MIN && s <= AGC_DP_MAX);
    (void)AGC_ASSERT(v.c[0] >= AGC_DP_MIN);
    return (s < 0) ? agc_vec_neg(v) : v;
}

/* ABVAL */
agc_dp_t rz_abval(agc_interp_t *ip, agc_vec_t v)
{
    (void)AGC_ASSERT(ip != NULL);
    agc_dp_t a = agc_vec_abval(ip, v);
    (void)AGC_ASSERT(a >= 0);
    return a;
}

/* NORM X: left-justify; *xreg = the AGC index register, -(left shifts). */
agc_dp_t rz_norm(agc_dp_t a, int *xreg)
{
    if (!AGC_ASSERT(xreg != NULL)) {
        return a;
    }
    if (a == 0) {
        *xreg = 0;
        return 0;
    }
    int n = 0;
    agc_dp_t r = agc_dp_norm(a, &n);
    (void)AGC_ASSERT(n >= 0 && n <= AGC_DP_BITS);
    *xreg = -n;
    return r;
}

/* SL k (k > 0) or SR -k (k < 0): the AGC SL* and SR* with an index.     */
agc_dp_t rz_shift(agc_interp_t *ip, agc_dp_t a, int k)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(k >= -60 && k <= 60)) {
        return a;
    }
    if (k > 0) {
        return agc_dp_shl(ip, a, k);
    }
    if (k < 0) {
        return agc_dp_shr(a, -k);
    }
    return a;
}

/* SL* addr,X: shift count = addr - C(X); a negative count shifts right. */
agc_dp_t rz_sl_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg)
{
    (void)AGC_ASSERT(addr >= -30 && addr <= 30);
    (void)AGC_ASSERT(xreg >= -60 && xreg <= 60);
    return rz_shift(ip, a, addr - xreg);
}

/* SR* addr,X: shift right by addr - C(X); a negative count shifts left. */
agc_dp_t rz_sr_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg)
{
    (void)AGC_ASSERT(addr >= -30 && addr <= 30);
    (void)AGC_ASSERT(xreg >= -60 && xreg <= 60);
    return rz_shift(ip, a, xreg - addr);
}

agc_vec_t rz_vshift(agc_interp_t *ip, agc_vec_t v, int k)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(k >= -60 && k <= 60)) {
        return v;
    }
    if (k > 0) {
        return agc_vec_shl(ip, v, k);
    }
    if (k < 0) {
        return agc_vec_shr(v, -k);
    }
    return v;
}

/* VECSHIFT / VSR* 0,2 (P34-35 p.499): earth scale -> body scale. With
 * X2 = RTX2 = 2 (moon) the count 0 - X2 is negative: a left shift by 2. */
agc_vec_t agc_rendz_vec_to_body(agc_interp_t *ip, const agc_rendz_t *rz,
                                agc_vec_t v)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL)) {
        return v;
    }
    (void)AGC_ASSERT(rz->body.rtx2 == 0 || rz->body.rtx2 == 2);
    return rz_vshift(ip, v, rz->body.rtx2);
}

/* VSL* 0,2: body scale -> earth scale (moon: shift right 2). */
agc_vec_t rz_vec_from_body(agc_interp_t *ip, const agc_rendz_t *rz,
                           agc_vec_t v)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL)) {
        return v;
    }
    (void)AGC_ASSERT(rz->body.rtx2 == 0 || rz->body.rtx2 == 2);
    return rz_vshift(ip, v, -(int)rz->body.rtx2);
}

/* SHIFTR1 (P34-35 p.500): SL* 0,2 -- a body-scale scalar to 2^29. */
agc_dp_t agc_rendz_shiftr1(const agc_rendz_t *rz, agc_dp_t a)
{
    if (!AGC_ASSERT(rz != NULL)) {
        return a;
    }
    (void)AGC_ASSERT(rz->body.rtx2 == 0 || rz->body.rtx2 == 2);
    return agc_dp_shr(a, rz->body.rtx2);
}

/* ---- SELECTMU (P34-35 p.494) --------------------------------------------
 * Choose mu and the scale indices for the primary body of the CSM state
 * (CMOONFLG). RTMU is always at 2^36 (the lunar MUM, 2^30, is shifted
 * right 6). The AGC then clears FINALFLG and goes to VN1645; the program
 * drivers do the display (rz_ttogo_display) themselves.                         */
agc_status_t agc_rendz_selectmu(agc_rendz_t *rz, bool cmoonflg)
{
    if (!AGC_ASSERT(rz != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz->body.moon = cmoonflg;
    if (cmoonflg) {
        rz->body.rtx1 = -10;
        rz->body.rtx2 = 2;
        rz->body.rtsr1mu = mutable_rtmum_inv;          /* 2^14 */
        rz->body.rtmu = agc_dp_shr(mutable_mum, 6);    /* 2^30 -> 2^36 */
    } else {
        rz->body.rtx1 = -2;
        rz->body.rtx2 = 0;
        rz->body.rtsr1mu = mutable_rtmue_inv;          /* 2^17 */
        rz->body.rtmu = mutable_mue;                   /* 2^36 */
    }
    (void)AGC_ASSERT(rz->body.rtmu > 0);
    return AGC_STATUS_OK;
}

/* ---- AVFLAGA / AVFLAGP / P20FLGON (CM P30-P37 p.~, LM P32-P35) ---------
 * AVFLAG reset = CSM active, set = LM active. "This vehicle active" is
 * AVFLAGA. The CM AVFLAGA also sets ECSTEER = 1 (BIT13).                  */
agc_status_t rz_avflag(bool this_active)
{
    bool csm = rz_is_csm();
    bool lm_active = csm ? !this_active : this_active;
    agc_status_t st = rz_flag_put(AGC_FLAG_AVFLAG, lm_active);
    if (csm && this_active) {
        /* CAF BIT13, TS ECSTEER: "SET ECSTEER = 1" (high word only) */
        agc_burn_t *b = &agc_guid_state()->burn;
        b->ecsteer = agc_dp_pack((agc_sp_t)010000, agc_dp_low(b->ecsteer));
    }
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_AVFLAG) == lm_active);
    return st;
}

agc_status_t rz_p20flgon(void)
{
    agc_status_t st = agc_flag_set(AGC_FLAG_TRACKFLG);
    if (st == AGC_STATUS_OK) {
        st = agc_flag_set(AGC_FLAG_UPDATFLG);
    }
    (void)AGC_ASSERT(st != AGC_STATUS_OK || agc_flag_test(AGC_FLAG_UPDATFLG));
    return st;
}

/* ---- PRECSET / LEMSTORE / CSMSTORE / ACTIVE / PASSIVE (p.498) ---------- */
void rz_store_active(agc_rendz_t *rz, agc_vec_t r, agc_vec_t v)
{
    if (!AGC_ASSERT(rz != NULL)) {
        return;
    }
    rz->ract3 = r;                          /* ACTIVE: STOVL RACT3        */
    rz->vact3 = v;                          /*         STORE VACT3        */
    (void)AGC_ASSERT(rz->body.rtx2 >= 0);
}

void rz_store_passive(agc_rendz_t *rz, agc_vec_t r, agc_vec_t v)
{
    if (!AGC_ASSERT(rz != NULL)) {
        return;
    }
    rz->rpass3 = r;                         /* PASSIVE: STOVL RPASS3      */
    rz->vpass3 = v;                         /*          STORE VPASS3      */
    (void)AGC_ASSERT(rz->body.rtx2 >= 0);
}

/* PRECSET: TDEC2 = TDEC1; LEMPREC, LEMSTORE (AVFLAG set = LM active);
 * TDEC1 = TDEC2; CSMPREC, CSMSTORE. RATT/VATT are at earth scale.     */
agc_status_t rz_precset(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t tdec1,
                        agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(tdec1 >= 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    bool lm_active = agc_flag_test(AGC_FLAG_AVFLAG);
    agc_integ_out_t o;
    rz->tdec2 = tdec1;                                        /* TDEC2   */
    agc_status_t st = rz_integ(ip, RZ_LEMPREC, tdec1, resume, &o);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (lm_active) {                                          /* LEMSTORE */
        rz_store_active(rz, o.ratt, o.vatt);
    } else {
        rz_store_passive(rz, o.ratt, o.vatt);
    }
    st = rz_integ(ip, RZ_CSMPREC, rz->tdec2, resume, &o);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (lm_active) {                                          /* CSMSTORE */
        rz_store_passive(rz, o.ratt, o.vatt);
    } else {
        rz_store_active(rz, o.ratt, o.vatt);
    }
    return AGC_STATUS_OK;
}

/* ---- INTINT (p.479) ------------------------------------------------------
 * Push list on entry: V, R, T1, T2, conic indicator (non-zero = conic).
 * INTSTALL; INTYPFLG per indicator; TDEC1 = T2; MOONFLAG = CMOONFLG;
 * TET = T1; RCV, VCV = VSR* 0,2 (body scale); INTEGRVS; RATT, VATT out.
 * GOINT (p.479) is INTINT with T2 = TTPI + NOMTPI... called by S33/34.1. */
agc_status_t rz_intint(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r,
                       agc_vec_t v, agc_dp_t t1, agc_dp_t t2, bool conic,
                       agc_job_id_t resume, agc_vec_t *ratt, agc_vec_t *vatt)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL)
        || !AGC_ASSERT(ratt != NULL && vatt != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_integrvs_in_t in;
    in.conic = conic;
    in.tdec1 = t2;
    in.moon = agc_flag_test(AGC_FLAG_CMOONFLG);
    in.tet = t1;
    in.rcv = agc_rendz_vec_to_body(ip, rz, r);
    in.vcv = agc_rendz_vec_to_body(ip, rz, v);
    agc_integ_out_t o;
    agc_status_t st = rz_integrvs(ip, &in, resume, &o);
    if (st == AGC_STATUS_OK) {
        *ratt = o.ratt;
        *vatt = o.vatt;
    }
    return st;
}

/* TIMETHET call set-up used throughout P32-P39 (VECSHIFT, X1 = RTX1). */
agc_status_t rz_timethet_rv(agc_interp_t *ip, const agc_rendz_t *rz,
                            agc_vec_t r, agc_vec_t v, agc_dp_t snth,
                            agc_dp_t csth, bool rvsw,
                            agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_timetheta_in_t in;
    in.rvec = agc_rendz_vec_to_body(ip, rz, r);
    in.vvec = agc_rendz_vec_to_body(ip, rz, v);
    in.snth = snth;
    in.csth = csth;
    in.rvsw = rvsw;
    in.body = rz_body(rz);
    (void)rz_flag_put(AGC_FLAG_RVSW, rvsw);
    return rz_timethet(ip, &in, out);
}

/* ---- PERIAPO1 / PERIAPO / SETRAD (p.495-497) ----------------------------
 * PERIAPO1 shifts the earth-scale state in the push list to body scale;
 * SETRAD gives XXXALT = RPAD (2^29) about the earth, |RLS| (2^27) about
 * the moon; APSIDES gives the apsides. Altitudes = radii - XXXALT, all at
 * body scale (callers apply SHIFTR1).                                    */
agc_status_t agc_rendz_periapo1(agc_interp_t *ip, const agc_rendz_t *rz,
                                agc_vec_t r, agc_vec_t v,
                                agc_rendz_periapo_t *out)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t vvec = agc_rendz_vec_to_body(ip, rz, v);
    agc_vec_t rvec = agc_rendz_vec_to_body(ip, rz, r);
    return agc_rendz_periapo(ip, rz, rvec, vvec, out);
}

/* PERIAPO (p.496): RVEC, VVEC already at body scale. */
agc_status_t agc_rendz_periapo(agc_interp_t *ip, const agc_rendz_t *rz,
                               agc_vec_t rvec, agc_vec_t vvec,
                               agc_rendz_periapo_t *out)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_conic_body_t body = rz_body(rz);
    /* SETRAD: RPAD (2^29) about the earth, |RLS| (2^27) about the moon */
    out->xxxalt = (body == AGC_CONIC_MOON) ? rz_abval(ip, rz_rls())
                                           : RZ_RPAD_B29;
    agc_apsides_out_t aps;
    agc_status_t st = rz_apsides(ip, rvec, vvec, body, &aps);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    out->rapo = aps.rapo;                                          /* 2D */
    out->rperi = aps.rperi;                                        /* 6D */
    out->ecc = aps.ecc;
    out->r1 = aps.r1;
    out->p = aps.p;
    out->hapo = agc_dp_sub(ip, out->rapo, out->xxxalt);           /* 4D */
    out->hperi = agc_dp_sub(ip, out->rperi, out->xxxalt);         /* 8D */
    return AGC_STATUS_OK;
}

/* ---- S34/35.1 (p.474): unit normal and line of sight at T3 ------------- */
agc_status_t agc_rendz_los_and_normal(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz->ulos = rz_unit(ip, agc_vec_sub(ip, rz->rpass3, rz->ract3));
    rz->unrm = rz_unit(ip, agc_vec_cross(ip, rz->ract3, rz->vact3));
    (void)AGC_ASSERT(rz->unrm.c[0] != AGC_DP_MIN);
    return AGC_STATUS_OK;
}

/* ---- LOMAT (p.479): LV matrix rows X = Z x (-Y)... from UNRM, RACT3 ----
 * Y = -UNRM, Z = -UNIT(RACT3), X = (Z x UNRM) VSL1. Rows at 2^1.         */
agc_status_t agc_rendz_lomat(agc_interp_t *ip, const agc_rendz_t *rz,
                             agc_mat_t *m)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(m != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    m->row[1] = agc_vec_neg(rz->unrm);
    m->row[2] = agc_vec_neg(rz_unit(ip, rz->ract3));
    m->row[0] = agc_vec_shl(ip, agc_vec_cross(ip, m->row[2], rz->unrm), 1);
    return AGC_STATUS_OK;
}

/* ---- GET.LVC (MIDGIM p.493): delta V to local vertical coordinates -----
 * Z = U(-R), Y = U(U(-R) x V), X = U(Y x Z); DELVLVC = M dV, VSL1.      */
agc_status_t agc_rendz_get_lvc(agc_interp_t *ip, agc_vec_t rinit,
                               agc_vec_t vinit, agc_vec_t dv,
                               agc_vec_t *delvlvc)
{
    if (!AGC_ASSERT(ip != NULL && delvlvc != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_mat_t m;
    m.row[2] = agc_vec_neg(rz_unit(ip, rinit));                    /* 18D */
    m.row[1] = rz_unit(ip, agc_vec_cross(ip, m.row[2], vinit));    /* 12D */
    m.row[0] = rz_unit(ip, agc_vec_cross(ip, m.row[1], m.row[2])); /*  6D */
    *delvlvc = agc_vec_shl(ip, agc_mat_vec(ip, &m, dv), 1);
    (void)AGC_ASSERT(delvlvc->c[0] != AGC_DP_MIN);
    return agc_flag_set(AGC_FLAG_MGLVFLAG);
}

/* ---- GET+MGA (MIDGIM p.493): +MGA = ARCSIN(UNIT(dV) . Ysm) in 0..1 rev */
agc_status_t agc_rendz_get_mga(agc_interp_t *ip, agc_vec_t dv,
                               const agc_mat_t *refsmmat, agc_dp_t *mga)
{
    if (!AGC_ASSERT(ip != NULL && refsmmat != NULL) || !AGC_ASSERT(mga != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    /* HALFREV 2DEC 1 B-1 */
    static const agc_dp_t halfrev = AGC_DP_HALF;
    agc_dp_t s = agc_dp_shl(ip, agc_vec_dot(ip, rz_unit(ip, dv),
                                            refsmmat->row[1]), 1);
    agc_dp_t a = agc_dp_asin(ip, s);
    if (a < 0) {                    /* convert -MGA to +MGA: add one rev */
        a = agc_dp_add(ip, agc_dp_add(ip, a, halfrev), halfrev);
    }
    *mga = a;
    return agc_flag_clear(AGC_FLAG_MGLVFLAG);
}
