/*
 * csi_cdh_targeting.c -- the CSI and CDH targeting programs, both
 * vehicles:
 *   P32 / P72  coelliptic sequence initiation (CSI) targeting
 *   P33 / P73  constant delta height (CDH) targeting
 * Comanche055 P32-P33_P72-P73.agc pp.652-662, Luminary099
 * P32-P35_P72-P75.agc (same program text; the LM computes TCSI at the
 * apocenter when the crew loads none, and keeps NN on entry).
 *
 * Each AGC program is one job that sleeps on its flashing displays; here
 * every return from a display is a continuation job (see the job ids in
 * ext.h) and the program state lives in the rendezvous erasables.
 *
 * Scales: r m 2^29, v m/cs 2^7, t cs 2^28.
 */
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "internal.h"

#define CSI_MOD_LOOP_MAX 800    /* > 2^28 cs / 60 min: bound of the modulo
                                 * loops P32/P72E, P33/P73E, P33/P73F    */

/* Job prologue: interpreter state and the rendezvous erasables. */
static agc_rendz_t *csi_begin(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    (void)AGC_ASSERT(ctx != NULL);
    agc_interp_init(ip);
    agc_rendz_t *rz = rz_state();
    (void)AGC_ASSERT(rz->body.rtx1 == -2 || rz->body.rtx1 == -10);
    return rz;
}

/* STORE X; DSU 60MIN; BPL (repeat): X modulo one hour (P32/P72E). */
static agc_dp_t csi_mod_hour(agc_interp_t *ip, agc_dp_t x)
{
    agc_dp_t v = x;
    for (int i = 0; i < CSI_MOD_LOOP_MAX; i++) {
        agc_dp_t d = agc_dp_sub(ip, v, RZ_60MIN);
        if (d < 0) {
            break;
        }
        v = d;
    }
    (void)AGC_ASSERT(v < RZ_60MIN);
    return v;
}

/* SELECTMU (p.494) then its GOTO VN1645, which returns to the caller. */
static agc_status_t csi_selectmu(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                                 agc_rendz_t *rz, agc_job_id_t next)
{
    agc_status_t st = agc_rendz_selectmu(rz, agc_flag_test(AGC_FLAG_CMOONFLG));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_flag_clear(AGC_FLAG_FINALFLG);
    return rz_ttogo_display(ctx, ip, rz, next);
}

/* ======================= P32 / P72 (pp.652-654) ========================== */
/* P32 (this vehicle active) / P72 (other vehicle active): AVFLAGA/P,
 * P32STRT: CENTANG = 0, then P32/P72A.                                   */
static agc_status_t csi_start(agc_job_ctx_t *ctx, bool this_active)
{
    rz_avflag(this_active);
    rz_state()->centang = 0;
    return agc_job_csi_input(ctx);
}

/* P32 CSI TARGETING (P32-P33_P72-P73.agc p.652, label P32) */
agc_status_t agc_job_csi_targeting(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return csi_start(ctx, true);
}

/* P72 CSI TARGETING, OTHER VEHICLE ACTIVE (p.652, label P72) */
agc_status_t agc_job_csi_targeting_other_vehicle(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return csi_start(ctx, false);
}

/* P32/P72A (p.652): P20FLGON; NN = 0 (CM) / NN+1 = TCSI = 0 (LM);
 * V06N11 TCSI.                                                          */
agc_status_t agc_job_csi_input(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_rendz_t *rz = rz_state();
    rz_tracking_flags_on();
    if (rz_is_csm()) {
        rz->nn = 0;
    } else {
        rz->tcsi = 0;     /* TS NN +1: the low word of NN is not modelled */
    }
    return rz_vnpooh(ctx, 6U, 11U, AGC_JOB_CSI_AFTER_TCSI);
}

/* LM VN0611 (p.~): TCSI <= 0 -> TCSI = time of the next apocenter of the
 * LM state (PRECSET at TETLEM, TIMERAD to DPPOSMAX), V06N11 again.     */
static agc_status_t csi_lm_auto_tcsi(agc_job_ctx_t *ctx, agc_interp_t *ip,
                                     agc_rendz_t *rz)
{
    agc_status_t st = rz_precset(ip, rz, agc_erase_tetlem(),
                                 AGC_JOB_CSI_AFTER_TCSI);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_timerad_in_t in;
    in.rvec = agc_rendz_vec_to_body(ip, rz, rz->ract3);
    in.vvec = agc_rendz_vec_to_body(ip, rz, rz->vact3);
    in.rdesired = RZ_DPPOSMAX;             /* beyond the apocenter      */
    in.sgnrdot = AGC_SP_HALF;   /* not set by the AGC; unused at APSESW */
    in.rvsw = true;
    in.body = rz_body(rz);
    rz_flag_put(AGC_FLAG_RVSW, true);
    agc_conic_time_out_t o;
    st = rz_timerad(ip, &in, &o);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz->tcsi = agc_dp_add(ip, o.t, rz->tdec2);
    return rz_vnpooh(ctx, 6U, 11U, AGC_JOB_CSI_AFTER_TCSI);
}

/* After V06N11: (LM: automatic TCSI) V06N55 NN, ELEV. */
agc_status_t agc_job_csi_after_tcsi(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    if (!rz_is_csm() && agc_dp_neg(rz->tcsi) >= 0) {
        return csi_lm_auto_tcsi(ctx, &ip, rz);
    }
    return rz_vnpooh(ctx, 6U, 55U, AGC_JOB_CSI_AFTER_NN_ELEV);
}

/* After V06N55: V06N37 TTPI. */
agc_status_t agc_job_csi_after_nn_elev(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(rz_state()->nn >= 0);
    return rz_vnpooh(ctx, 6U, 37U, AGC_JOB_CSI_AFTER_TTPI);
}

/* After V06N37: TIG = TCSI, SELECTMU (V16N45), then P32/P72B. */
agc_status_t agc_job_csi_after_ttpi(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    agc_erase_set_tig(rz->tcsi);
    return csi_selectmu(ctx, &ip, rz, AGC_JOB_CSI_COMPUTE);
}

/* ALMXIT (p.652): VARALARM 00600 + CSIALRM - 1, V05N09: PROCEED shows it
 * again ("TC -4"), RECYCLE goes back to P32/P72A.                       */
static agc_status_t csi_almxit(agc_job_ctx_t *ctx, agc_sp_t code)
{
    agc_status_t st = agc_exec_varalarm((agc_alarm_code_t)code);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_ALARM);
    return agc_job_csi_alarm_display(ctx);
}

agc_status_t agc_job_csi_alarm_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return rz_flash(ctx, AGC_DSKY_GOFLASH, 5U, 9U, AGC_JOB_GOTOPOOH,
                    AGC_JOB_CSI_ALARM_DISPLAY, AGC_JOB_CSI_INPUT);
}

/* P32/P72C .. P32/P72F (pp.653-654): UPDATFLG unless final, the times
 * modulo one hour, V06N75.                                              */
static agc_status_t csi_results(agc_job_ctx_t *ctx, agc_interp_t *ip,
                                agc_rendz_t *rz)
{
    if (!agc_flag_test(AGC_FLAG_FINALFLG)) {
        agc_flag_set(AGC_FLAG_UPDATFLG);
    }
    rz->t1tot2 = csi_mod_hour(ip, rz->t1tot2);              /* P32/P72E  */
    rz->t2tot3 = csi_mod_hour(ip, rz->t2tot3);              /* P32/P72F  */
    return rz_vnpooh(ctx, 6U, 75U, AGC_JOB_CSI_AFTER_RESULTS);
}

/* P32/P72B (p.653): ADVANCE; passive TCSI -> TTPI (conic) as PASSIVE;
 * CSI/A; ALMXIT on failure.                                              */
agc_status_t agc_job_csi_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    agc_status_t st = agc_rendz_advance(&ip, rz, AGC_JOB_CSI_COMPUTE);
    agc_vec_t r, v;
    if (st == AGC_STATUS_OK) {
        st = rz_intint(&ip, rz, rz->rpass1, rz->vpass1, rz->tcsi,
                       agc_erase_ttpi(), true, AGC_JOB_CSI_COMPUTE, &r, &v);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_store_passive(rz, r, v);
    agc_sp_t alarm = 0;
    st = agc_rendz_csi_a(&ip, rz, &alarm, AGC_JOB_CSI_COMPUTE);
    if (st == AGC_STATUS_ALARM) {
        return csi_almxit(ctx, alarm);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return csi_results(ctx, &ip, rz);
}

/* After V06N75: S32/33.1 with DELVEET1 -> DISDVLVC V06N81. */
agc_status_t agc_job_csi_after_results(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    return rz_disdvlvc(ctx, &ip, rz, rz->delveet1, 81U,
                       AGC_JOB_CSI_AFTER_DV_LV);
}

/* After V06N81: S32/33.1 tail (DELVEET1 = DELVSIN), RACT1 = RACT2,
 * DISDVLVC of DELVEET2 on V06N82.                                       */
agc_status_t agc_job_csi_after_dv_lv(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    agc_vec_t dv;
    agc_status_t st = agc_rendz_csi_cdh_dv_to_reference(&ip, rz, &dv);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    rz->delveet1 = dv;
    rz->ract1 = rz->ract2;
    return rz_disdvlvc(ctx, &ip, rz, rz->delveet2, 82U,
                       AGC_JOB_CSI_AFTER_CDH_DV);
}

/* After V06N82: TTPIO = TTPI, VN1645, back to P32/P72B. */
agc_status_t agc_job_csi_after_cdh_dv(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    rz->ttpio = agc_erase_ttpi();
    return rz_ttogo_display(ctx, &ip, rz, AGC_JOB_CSI_COMPUTE);
}

/* ======================= P33 / P73 (pp.658-660) ========================== */
static agc_status_t cdh_start(agc_job_ctx_t *ctx, bool this_active)
{
    rz_avflag(this_active);
    return agc_job_cdh_input(ctx);
}

/* P33 CDH TARGETING (P32-P33_P72-P73.agc p.658, label P33) */
agc_status_t agc_job_cdh_targeting(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return cdh_start(ctx, true);
}

/* P73 CDH TARGETING, OTHER VEHICLE ACTIVE (p.658, label P73) */
agc_status_t agc_job_cdh_targeting_other_vehicle(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return cdh_start(ctx, false);
}

/* P33/P73A: P20FLGON, V06N13 TCDH. */
agc_status_t agc_job_cdh_input(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_tracking_flags_on();
    return rz_vnpooh(ctx, 6U, 13U, AGC_JOB_CDH_AFTER_TCDH);
}

/* After V06N13: TTPI = TTPIO, TIG = TCDH, SELECTMU, then P33/P73B. */
agc_status_t agc_job_cdh_after_tcdh(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    agc_erase_set_ttpi(rz->ttpio);
    agc_erase_set_tig(rz->tcdh);
    return csi_selectmu(ctx, &ip, rz, AGC_JOB_CDH_COMPUTE);
}

/* P33/P73C .. P33/P73F (p.659): UPDATFLG unless final; TTPI += NOMTPI;
 * T1TOT2 = (TTPI - TCDH) mod 1 h; T2TOT3 = (TTPI - TTPIO) mod 1 h
 * (signed); V06N75.                                                     */
static agc_status_t cdh_results(agc_job_ctx_t *ctx, agc_interp_t *ip,
                                agc_rendz_t *rz)
{
    if (!agc_flag_test(AGC_FLAG_FINALFLG)) {
        agc_flag_set(AGC_FLAG_UPDATFLG);
    }
    agc_dp_t ttpi = agc_dp_add(ip, rz->nomtpi, agc_erase_ttpi());
    agc_erase_set_ttpi(ttpi);
    agc_dp_t x = agc_dp_sub(ip, ttpi, rz->tcdh);
    if (x >= 0) {
        x = csi_mod_hour(ip, x);              /* negative x is kept      */
    }
    rz->t1tot2 = x;
    agc_dp_t y = agc_dp_sub(ip, ttpi, rz->ttpio);
    rz->t2tot3 = agc_dp_sign(csi_mod_hour(ip, agc_dp_abs(y)), y);
    return rz_vnpooh(ctx, 6U, 75U, AGC_JOB_CDH_AFTER_RESULTS);
}

/* S33/34.1 result: OK -> P33/P73C; no solution -> ALARM 611, V05N09. */
static agc_status_t cdh_after_search(agc_job_ctx_t *ctx, agc_interp_t *ip,
                                     agc_rendz_t *rz, agc_status_t st)
{
    if (st == AGC_STATUS_ALARM) {
        return rz_alarm_flash(ctx, (agc_alarm_code_t)AGC_ALARM_00611,
                              AGC_JOB_CDH_ALARM_PROCEED, AGC_JOB_CDH_INPUT);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return cdh_results(ctx, ip, rz);
}

/* P33/P73B (p.658): ADVANCE, CDHMVR, active and passive TCDH -> TTPI
 * (precision, INTINT3P), NOMTPI = 0, ITSWICH, S33/34.1.                 */
agc_status_t agc_job_cdh_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    agc_status_t st = agc_rendz_advance(&ip, rz, AGC_JOB_CDH_COMPUTE);
    if (st == AGC_STATUS_OK) {
        st = agc_rendz_cdhmvr(&ip, rz);
    }
    agc_vec_t r, v;
    if (st == AGC_STATUS_OK) {
        st = rz_intint(&ip, rz, rz->ract2, rz->vact3, rz->tcdh,
                       agc_erase_ttpi(), false, AGC_JOB_CDH_COMPUTE, &r, &v);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_store_active(rz, r, v);
    st = rz_intint(&ip, rz, rz->rpass2, rz->vpass2, rz->tcdh,
                   agc_erase_ttpi(), false, AGC_JOB_CDH_COMPUTE, &r, &v);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_store_passive(rz, r, v);
    rz->nomtpi = 0;
    rz_vflag_set(RZ_VF_ITSWICH, true);
    st = agc_rendz_tpi_time_search(&ip, rz, AGC_JOB_CDH_TIME_SEARCH);
    return cdh_after_search(ctx, &ip, rz, st);
}

/* S33/34.1 resumed after an INTSTALL sleep. */
agc_status_t agc_job_cdh_time_search(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    agc_status_t st = agc_rendz_tpi_time_search_cont(&ip, rz,
                                                     AGC_JOB_CDH_TIME_SEARCH);
    return cdh_after_search(ctx, &ip, rz, st);
}

/* 611 PROCEED (p.659): NOMTPI = 0, P33/P73C. */
agc_status_t agc_job_cdh_alarm_proceed(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    rz->nomtpi = 0;
    return cdh_results(ctx, &ip, rz);
}

/* After V06N75: S32/33.1 with DELVEET2 -> DISDVLVC V06N81. */
agc_status_t agc_job_cdh_after_results(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    return rz_disdvlvc(ctx, &ip, rz, rz->delveet2, 81U,
                       AGC_JOB_CDH_AFTER_DV_LV);
}

/* After V06N81: DELVEET2 = DELVSIN, VN1645, back to P33/P73B. */
agc_status_t agc_job_cdh_after_dv_lv(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = csi_begin(ctx, &ip);
    agc_vec_t dv;
    agc_status_t st = agc_rendz_csi_cdh_dv_to_reference(&ip, rz, &dv);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    rz->delveet2 = dv;
    return rz_ttogo_display(ctx, &ip, rz, AGC_JOB_CDH_COMPUTE);
}
