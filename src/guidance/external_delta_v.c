/*
 * p30.c -- P30-P37.agc (Comanche055 pp. 635-648) and P30_P37.agc
 * (Luminary099 pp. 614-617): P30 external delta-V targeting (both), P31
 * general Lambert aimpoint targeting (Comanche; the Luminary P31 is in the
 * rendezvous programs), the subroutines S30.1 and S31.1, MIDGIM and the
 * display sequence P30/P31, CNTNUP30, PARAM30 (the Comanche-only REFTEST,
 * DISP45, END45 and COMPTGO, which use Comanche flags, are in
 * src/csm/burns/external_delta_v.c). DELRSPL (the V82 splash-error continuation) is a stub.
 *
 * The targeting results go to the rendezvous erasables (agc_rendz_t,
 * agc_programs.h): RTIG VTIG m 2^29 / m/cs 2^7, DELVSIN DELVLVC m/cs 2^7,
 * RTARG m 2^29, TPASS4 DELLT4 cs 2^28, +MGA rev 2^0; HAPO, HPER (m 2^29)
 * and VGDISP (m/cs 2^7) to agc_guid_state()->burn.
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/pending.h"

#define GRP4 4

/* EP4(45)H 2DEC .125, EP4(10)H 2DEC .027777777: 45 and 10 degrees (rev). */
static const agc_dp_t EP4_45 = 33554432;
static const agc_dp_t EP4_10 = 7456540;

/* Store a precision-update result in the rendezvous erasables: X1, X2 of
 * the integrator into RTX1, RTX2 (SXA,1 / SXA,2).                         */
static void store_rtx(agc_rendz_t *rz, const agc_guid_prec_t *p)
{
    rz->body.rtx1 = p->rtx1;
    rz->body.rtx2 = p->rtx2;
    rz->body.moon = (p->rtx2 != 0);
    (void)AGC_ASSERT(p->rtx2 == 0 || p->rtx2 == 2);
}

/* PERIAPO1 / SHIFTR1 / MAXCHK for HPER and HAPO (both S30.1, S31.1):
 * apogee and perigee altitudes of (r, v), limited to 9999.9 NM, m 2^29.   */
static agc_status_t apo_peri(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t r, agc_vec_t v)
{
    agc_burn_t *b = &agc_guid_state()->burn;
    agc_rendz_periapo_t pa;
    agc_status_t st = agc_rendz_periapo1(ip, rz, r, v, &pa);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    b->hper = agc_orbit_params_maxchk(ip, agc_rendz_shiftr1(rz, pa.hperi)); /* HPER B+29 */
    b->hapo = agc_orbit_params_maxchk(ip, agc_rendz_shiftr1(rz, pa.hapo));  /* HAPO B+29 */
    (void)AGC_ASSERT(rz->body.rtx2 == 0 || rz->body.rtx2 == 2);
    return AGC_STATUS_OK;
}

/* S30.1 Comanche (p.639-640): state at TIG by THISPREC, the local-vertical
 * matrix from LOMAT (UNRM = UNIT(RTIG x VTIG), RACT3 = RTIG), DELVSIN =
 * DELVSLV VXM matrix VSL1, VGDISP = |DELVSIN|, apogee/perigee.           */
static agc_status_t external_delta_v_vectors_csm(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_guid_prec_t p;
    agc_status_t st = agc_guid_prec_update(ip, AGC_VEHICLE_CSM, agc_erase_tig(),
                                           AGC_JOB_NONE, &p);   /* THISPREC, TDEC1 = TIG */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    store_rtx(rz, &p);
    rz->vtig = p.vatt;
    rz->rtig = p.ratt;
    rz->ract3 = p.ratt;
    rz->unrm = agc_vec_unit(ip, agc_vec_cross(ip, rz->rtig, rz->vtig), NULL, NULL);
    agc_mat_t m;
    st = agc_rendz_lomat(ip, rz, &m);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz->delvsin = agc_vec_shl(ip, agc_vec_mat(ip, agc_erase_delvslv(), &m), 1);
    agc_guid_state()->burn.vgdisp = agc_vec_abval(ip, rz->delvsin); /* MAG DELV */
    return apo_peri(ip, rz, rz->rtig, agc_vec_add(ip, rz->delvsin, rz->vtig));
}

/* S30.1 Luminary (p.616-617): LEMPREC; the LV axes at TIG built inline:
 * Z = -UNIT(RTIG), Y = UNIT(VTIG x RTIG), X = VSL1(Y x Z); DELVSIN =
 * DELVSLV VXM (X Y Z) VSL1, DELVSAB = |DELVSIN|, apogee/perigee.          */
static agc_status_t external_delta_v_vectors_lm(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_guid_prec_t p;
    agc_status_t st = agc_guid_prec_update(ip, AGC_VEHICLE_LM, agc_erase_tig(),
                                           AGC_JOB_NONE, &p);   /* LEMPREC */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    store_rtx(rz, &p);
    rz->rtig = p.ratt;
    agc_mat_t m;
    m.row[2] = agc_vec_neg(agc_vec_unit(ip, rz->rtig, NULL, NULL)); /* ZRF/LV */
    rz->vtig = p.vatt;
    m.row[1] = agc_vec_unit(ip, agc_vec_cross(ip, rz->vtig, rz->rtig), NULL, NULL);
    m.row[0] = agc_vec_shl(ip, agc_vec_cross(ip, m.row[1], m.row[2]), 1);
    rz->delvsin = agc_vec_shl(ip, agc_vec_mat(ip, agc_erase_delvslv(), &m), 1);
    rz->delvsab = agc_vec_abval(ip, rz->delvsin);              /* DELTA V MAG. */
    return apo_peri(ip, rz, rz->rtig, agc_vec_add(ip, rz->vtig, rz->delvsin));
}

/* S30.1 (CM p.639, LM p.616): external delta-V targeting at TIG.          */
agc_status_t agc_guid_external_delta_v_vectors(agc_interp_t *ip, agc_vehicle_t veh)
{
    if (!AGC_ASSERT(ip != NULL) ||
        !AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_rendz_t *rz = agc_rendz_erasable();
    if (!AGC_ASSERT(rz != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return (veh == AGC_VEHICLE_CSM) ? external_delta_v_vectors_csm(ip, rz) : external_delta_v_vectors_lm(ip, rz);
}

/* MIDGIM (Comanche P34-35 p.493): with AVFLAG equal to COMPUTER (both set
 * in the CMC for an LM-active burn) GET.LVC: DELVLVC = dv in the local
 * vertical axes of RINIT, VINIT; otherwise GET+MGA: +MGA.                 */
agc_status_t agc_guid_midgim(agc_interp_t *ip, agc_vec_t dv)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_rendz_t *rz = agc_rendz_erasable();
    bool av = agc_flag_test(AGC_FLAG_AVFLAG);
    bool cmc = (agc_state_vehicle() == AGC_VEHICLE_CSM);  /* COMPUTER flag    */
    if (av == cmc) {
        return agc_rendz_get_lvc(ip, rz->rinit, rz->vinit, dv, &rz->delvlvc);
    }
    agc_mat_t refsmmat = agc_erase_refsmmat();
    return agc_rendz_get_mga(ip, dv, &refsmmat, &rz->plus_mga);
}

/* AGAIN (Comanche P40-P47 p.711): THISPREC to TDEC1, RTIG = RINIT = RATT,
 * VTIG = VINIT = VATT, then INITVEL with NUMIT 0 and EPSILON4 45 degrees
 * (NORMSW: 165-195 degree transfer) or 10 degrees.                        */
static agc_status_t again_at(agc_interp_t *ip, agc_dp_t tdec1)
{
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_guid_prec_t p;
    agc_status_t st = agc_guid_prec_update(ip, agc_state_vehicle(), tdec1,
                                           AGC_JOB_NONE, &p);   /* THISPREC */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    store_rtx(rz, &p);
    rz->rtig = p.ratt;
    rz->rinit = p.ratt;
    rz->vtig = p.vatt;
    rz->vinit = p.vatt;
    agc_dp_t e4 = agc_flag_test(AGC_FLAG_NORMSW) ? EP4_45 : EP4_10;
    (void)AGC_ASSERT(e4 > 0);
    return agc_rendz_initvel(ip, rz, 0, e4, agc_guid_nav_resume());
}

/* AGAIN with TDEC1 as set by the caller (S31.1: TIG; S40.1B: TIG - 2 s). */
agc_status_t agc_guid_again(agc_interp_t *ip, agc_dp_t tdec1)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(tdec1 >= 0);
    return again_at(ip, tdec1);
}

/* S31.1 (Comanche p.641-642): Lambert to RTARG from TIG; apogee/perigee of
 * (RTIG, VIPRIME); DELVLVC from DELVEET3 (SET AVFLAG; MIDGIM -> GET.LVC);
 * VGDISP = |DELVLVC|; TPASS4 = TIG + DELLT4.                              */
agc_status_t agc_guid_lambert_aimpoint_vectors(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_status_t st = agc_guid_again(ip, agc_erase_tig());   /* TDEC1 = TIG */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    st = apo_peri(ip, rz, rz->rtig, rz->viprime);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    (void)agc_flag_set(AGC_FLAG_AVFLAG);
    st = agc_guid_midgim(ip, rz->delveet3);              /* DELVLVC B7 FOR DISPLAY */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_guid_state()->burn.vgdisp = agc_vec_abval(ip, rz->delvlvc);
    rz->tpass4 = agc_dp_add(ip, rz->dellt4, agc_erase_tig()); /* FOR S40.1     */
    return AGC_STATUS_OK;
}

/* DELRSPL (Comanche p.643-647): splash-down error for V82 during P11.
 * Needs VGAMCALC, TFF/TRIG and LALOTORV of other lanes.                   */
agc_status_t agc_guid_delrspl(agc_interp_t *ip, agc_dp_t hdist)
{
    if (!AGC_ASSERT(ip != NULL) ||
        !AGC_ASSERT(hdist >= AGC_DP_MIN)) {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_UNIMPLEMENTED;
}

/* ---------------------------------------------------------------------- */
/* Program sequencing                                                      */
/* ---------------------------------------------------------------------- */

/* A flashing display whose terminate goes to P00 (GOTOPOOH).             */
static agc_status_t flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                          uint8_t verb, uint8_t noun, agc_job_id_t proceed,
                          agc_job_id_t enter)
{
    agc_dsky_cont_t cont = { AGC_JOB_GOTOPOOH, proceed, enter };
    (void)AGC_ASSERT(proceed != AGC_JOB_NONE);
    (void)AGC_ASSERT(enter != AGC_JOB_NONE);
    return agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont);
}

/* P30 (CM p.637 / P30/P31 p.637; LM p.614): set UPDATFLG and TRACKFLG,
 * flash V06N33 (TIG). Comanche uses GOFLASHR and protects 4.1.            */
static agc_status_t ignition_time_entry(agc_job_ctx_t *ctx, agc_job_id_t self,
                                  agc_job_id_t next)
{
    (void)agc_flag_set(AGC_FLAG_UPDATFLG);
    (void)agc_flag_set(AGC_FLAG_TRACKFLG);
    if (agc_state_vehicle() == AGC_VEHICLE_LM) {
        return flash(ctx, AGC_DSKY_GOFLASH, 6, 33, next, self); /* VNPOOH      */
    }
    agc_status_t st = flash(ctx, AGC_DSKY_GOFLASHR, 6, 33, next, self);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_phase_change(GRP4, (uint16_t)000014);        /* 4.1 ENDOFJOB  */
}

agc_status_t agc_job_external_delta_v(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return ignition_time_entry(ctx, AGC_JOB_EXTERNAL_DELTA_V, AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_DV);
}

/* CNTNUP30 (CM p.637) / P30N33+2 (LM): flash V06N81 (DELVSLV).            */
agc_status_t agc_job_external_delta_v_display_dv(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return flash(ctx, AGC_DSKY_GOFLASH, 6, 81, AGC_JOB_EXTERNAL_DELTA_V_COMPUTE, AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_DV);
}

/* P30 after N81 PROCEED (CM p.636, LM p.614): S30.1, then PARAM30.        */
agc_status_t agc_job_external_delta_v_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    (void)agc_flag_clear(AGC_FLAG_UPDATFLG);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_guid_nav_set_resume(ctx->id);
    agc_status_t st = agc_guid_external_delta_v_vectors(&ip, agc_state_vehicle());
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (agc_state_vehicle() == AGC_VEHICLE_LM) {
        (void)agc_flag_set(AGC_FLAG_UPDATFLG);
    }
    return agc_job_external_delta_v_display_parameters(ctx);
}

/* PARAM30 (CM p.637, LM p.614): flash V06N42 (HAPO, HPER, VGDISP). CM:
 * PROCEED -> REFTEST; LM: PROCEED -> XDELVFLG set, REVN1645.              */
agc_status_t agc_job_external_delta_v_display_parameters(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    agc_job_id_t pro = (agc_state_vehicle() == AGC_VEHICLE_LM) ? AGC_JOB_EXTERNAL_DELTA_V_ACCEPT
                                                               : AGC_JOB_EXTERNAL_DELTA_V_REFTEST;
    return flash(ctx, AGC_DSKY_GOFLASH, 6, 42, pro, AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_PARAMETERS);
}

/* LM PARAM30 PROCEED (p.614): SETGO XDELVFLG REVN1645 (TRKMKCNT, TTOGO,
 * +MGA display of the rendezvous programs).                               */
agc_status_t agc_job_external_delta_v_accept(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    (void)agc_flag_set(AGC_FLAG_XDELVFLG);              /* EXTERNAL DELTA-V      */
    return agc_exec_job_yield(AGC_JOB_MARKS_TFI_MGA_MONITOR);
}

/* P31 (CM p.637): as P30, but S31.1 before V06N81.                         */
agc_status_t agc_job_lambert_aimpoint_guidance(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return ignition_time_entry(ctx, AGC_JOB_LAMBERT_AIMPOINT_GUIDANCE, AGC_JOB_LAMBERT_AIMPOINT_COMPUTE);
}

/* P31 after V06N33 PROCEED (CM p.637): UPDATFLG and NORMSW reset, S31.1,
 * then CNTNUP30 (V06N81) and PARAM30.                                     */
agc_status_t agc_job_lambert_aimpoint_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    (void)agc_flag_clear(AGC_FLAG_UPDATFLG);
    (void)agc_flag_clear(AGC_FLAG_NORMSW);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_guid_nav_set_resume(ctx->id);
    agc_status_t st = agc_guid_lambert_aimpoint_vectors(&ip);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_job_lambert_aimpoint_display_dv(ctx);
}

/* CNTNUP30 for P31: V06N81 (DELVLVC), PROCEED -> PARAM30.                */
agc_status_t agc_job_lambert_aimpoint_display_dv(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return flash(ctx, AGC_DSKY_GOFLASH, 6, 81, AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_PARAMETERS, AGC_JOB_LAMBERT_AIMPOINT_DISPLAY_DV);
}
