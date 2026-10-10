/*
 * tpi_tpm_targeting.c -- the TPI and TPM targeting programs, both
 * vehicles (P34-35_P74-75.agc, Comanche055 pp.462-467, Luminary099 the
 * same text):
 *   P34 / P74  transfer phase initiation (TPI) targeting
 *   P35 / P75  transfer phase midcourse (TPM) targeting
 * Vehicle difference: on P34/P74A entry the LM sets CENTANG = 130 deg
 * (130DEG) and NN = 0 before DISPLAYE; the CM only zeroes NN.
 *
 * Each AGC program is one job that sleeps on its flashing displays; every
 * return from a display (and every resumption after an INTSTALL sleep) is
 * a continuation job (ids in ext.h), the program state lives in the
 * rendezvous erasables.
 *
 * Scales: r m 2^29, v m/cs 2^7, t cs 2^28, angles revolutions 2^0.
 */
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "../navigation/internal.h"

/* 130DEG 2DEC .3611111111 (LM P34-35 p.684): 130 degrees in revolutions */
#define TPI_130DEG   ((agc_dp_t)96935026)

/* Job prologue: interpreter state and the rendezvous erasables. */
static agc_rendz_t *tpi_begin(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(ip != NULL);
    agc_interp_init(ip);
    return rz_state();
}

/* SELECTMU (p.494): primary body, CLEAR FINALFLG, GOTO VN1645, which
 * returns into the program at next.                                     */
static agc_status_t tpi_selectmu(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                                 agc_rendz_t *rz, agc_job_id_t next)
{
    agc_status_t st = agc_rendz_selectmu(rz, agc_flag_test(AGC_FLAG_CMOONFLG));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_flag_clear(AGC_FLAG_FINALFLG);
    return rz_ttogo_display(ctx, ip, rz, next);
}

/* ======================= P34 / P74 (pp.462-465) ========================== */
/* P34 TPI TARGETING (P34-35_P74-75.agc p.462, label P34): AVFLAGA. */
agc_status_t agc_job_tpi_targeting(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    rz_avflag(true);
    return agc_job_tpi_input(ctx);
}

/* P74 TPI TARGETING, OTHER VEHICLE ACTIVE (p.462, label P74): AVFLAGP. */
agc_status_t agc_job_tpi_targeting_other_vehicle(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    rz_avflag(false);
    return agc_job_tpi_input(ctx);
}

/* P34/P74A (p.462): P20FLGON (UPDATFLG, TRACKFLG), V06N37 TTPI. */
agc_status_t agc_job_tpi_input(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_tracking_flags_on();
    return rz_vnpooh(ctx, 6U, 37U, AGC_JOB_TPI_AFTER_TTPI);
}

/* After V06N37: NN = 0 (LM also CENTANG = 130 deg), DISPLAYE V06N55. */
agc_status_t agc_job_tpi_after_ttpi(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_rendz_t *rz = rz_state();
    if (!rz_is_csm()) {
        rz->centang = TPI_130DEG;                 /* DCA 130DEG, DXCH     */
    }
    rz->nn = 0;                                   /* SSP NN 0 / TS NN     */
    (void)AGC_ASSERT(rz->centang >= 0);
    return rz_displaye(ctx, AGC_JOB_TPI_AFTER_ELEV);
}

/* After DISPLAYE: CLEAR ETPIFLAG, TIG = TTPI; ELEV non-zero -> SET
 * ETPIFLAG (the crew gave an elevation: find TTPI for it). P34/P74B:
 * SELECTMU, VN1645, then P34/P74C.                                      */
agc_status_t agc_job_tpi_after_elev(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    agc_flag_clear(AGC_FLAG_ETPIFLAG);
    agc_erase_set_tig(agc_erase_ttpi());
    if (rz->elev != 0) {                          /* BZE P34/P74B         */
        agc_flag_set(AGC_FLAG_ETPIFLAG);
    }
    return tpi_selectmu(ctx, &ip, rz, AGC_JOB_TPI_CYCLE);
}

/* P34/P74C (p.463): NOMTPI = 0; SET ITSWICH; ETPIFLAG clear -> CLEAR
 * ITSWICH (only compute the elevation at the given TTPI). INTLOOP.     */
agc_status_t agc_job_tpi_cycle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_rendz_t *rz = rz_state();
    rz_vflag_set(RZ_VF_ITSWICH, true);
    if (!agc_flag_test(AGC_FLAG_ETPIFLAG)) {      /* BON ETPIFLAG SWCHSET */
        rz_vflag_set(RZ_VF_ITSWICH, false);
    }
    rz->nomtpi = 0;                               /* SWCHSET STORE NOMTPI */
    (void)AGC_ASSERT(rz->nomtpi == 0);
    return agc_job_tpi_intloop(ctx);
}

/* SWCHCLR (p.463), after S33/34.1 succeeded: ITSWICH was set -> clear it
 * and go round INTLOOP once more (the elevation at the new TTPI);
 * ETPIFLAG set -> V06N37 (the new TTPI), else DISPLAYE; then P34/P74E. */
static agc_status_t tpi_after_search(agc_job_ctx_t *ctx, agc_status_t st)
{
    if (st == AGC_STATUS_ALARM) {                 /* BZE fails: 611       */
        return agc_job_tpi_alarm(ctx);
    }
    if (st != AGC_STATUS_OK) {
        return st;                                /* INTSTALL sleep       */
    }
    if (rz_vflag_test(RZ_VF_ITSWICH)) {           /* BONCLR ITSWICH       */
        rz_vflag_set(RZ_VF_ITSWICH, false);
        return rz_continue(AGC_JOB_TPI_INTLOOP);
    }
    if (agc_flag_test(AGC_FLAG_ETPIFLAG)) {       /* P34/P74D             */
        return rz_vnpooh(ctx, 6U, 37U, AGC_JOB_TPI_COMPUTE);
    }
    return rz_displaye(ctx, AGC_JOB_TPI_COMPUTE);
}

/* INTLOOP (p.463): TDEC1 = TTPI + NOMTPI, PRECSET (both vehicles), then
 * S33/34.1 (TPI time search / elevation).                               */
agc_status_t agc_job_tpi_intloop(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    agc_dp_t tdec1 = agc_dp_add(&ip, agc_erase_ttpi(), rz->nomtpi);
    agc_status_t st = rz_precset(&ip, rz, tdec1, AGC_JOB_TPI_INTLOOP);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    st = agc_rendz_tpi_time_search(&ip, rz, AGC_JOB_TPI_TIME_SEARCH);
    return tpi_after_search(ctx, st);
}

/* S33/34.1 resumed after an INTSTALL sleep. */
agc_status_t agc_job_tpi_time_search(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    agc_status_t st = agc_rendz_tpi_time_search_cont(&ip, rz,
                                                     AGC_JOB_TPI_TIME_SEARCH);
    return tpi_after_search(ctx, st);
}

/* TC ALARM OCT 611, V05N09 (p.463): TERMINATE -> P00, PROCEED ->
 * P34/P74A, V32 ("TC -7") -> the alarm and display again.              */
agc_status_t agc_job_tpi_alarm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_alarm_flash(ctx, (agc_alarm_code_t)AGC_ALARM_00611,
                          AGC_JOB_TPI_INPUT, AGC_JOB_TPI_ALARM);
}

/* P34/P74E (pp.464-465): the TPI solution (agc_rendz_tpi_compute), V06N58
 * (POSTTPI, DELVTPI, DELVTPF).                                          */
agc_status_t agc_job_tpi_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    agc_status_t st = agc_rendz_tpi_compute(&ip, rz, AGC_JOB_TPI_COMPUTE);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return rz_vnpooh(ctx, 6U, 58U, AGC_JOB_TPI_AFTER_RESULTS);
}

/* After V06N58: S34/35.5 (final-pass displays), then VN1645. */
agc_status_t agc_job_tpi_after_results(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    return rz_tpi_final_displays(ctx, &ip, rz, AGC_JOB_TPI_AFTER_FINAL);
}

/* After S34/35.5: VN1645, then GOTO P34/P74C (next cycle). */
agc_status_t agc_job_tpi_after_final(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    return rz_ttogo_display(ctx, &ip, rz, AGC_JOB_TPI_CYCLE);
}

/* ======================= P35 / P75 (pp.466-467) ========================== */
/* P35/P75A: KT = the TIG increment, P20FLGON, SELECTMU (VN1645), then
 * P35/P75B.                                                             */
static agc_status_t tpm_start(agc_job_ctx_t *ctx, bool this_active)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    rz_avflag(this_active);
    rz->kt = this_active ? rz->atiginc : rz->ptiginc;  /* DCA xTIGINC */
    rz_tracking_flags_on();
    return tpi_selectmu(ctx, &ip, rz, AGC_JOB_TPM_COMPUTE);
}

/* P35 TPM TARGETING (p.466, label P35): AVFLAGA, KT = ATIGINC. */
agc_status_t agc_job_tpm_targeting(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return tpm_start(ctx, true);
}

/* P75 TPM TARGETING, OTHER VEHICLE ACTIVE (p.466, label P75): AVFLAGP,
 * KT = PTIGINC.                                                         */
agc_status_t agc_job_tpm_targeting_other_vehicle(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return tpm_start(ctx, false);
}

/* P35/P75B (p.466): TSTRT = now; TIG = INTIME = TDEC1 = TSTRT + KT. */
agc_status_t agc_job_tpm_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    rz->tstrt = rz_loadtime();                     /* RTB LOADTIME        */
    agc_dp_t tig = agc_dp_add(&ip, rz->tstrt, rz->kt);
    agc_erase_set_tig(tig);
    rz->intime = tig;                              /* FOR INITVEL         */
    (void)AGC_ASSERT(rz->tstrt >= 0);
    return agc_job_tpm_advance(ctx);
}

/* P35/P75B continued: PRECSET to TIG (both vehicles), S34/35.1 (LOS and
 * normal), S34/35.2 (delta V), S34/35.5. Resumed here (not at LOADTIME)
 * after an INTSTALL sleep, so TIG is not moved by a sleep.
 * DEVIATION: the AGC restarts INTSTALL inside PRECSET; the C job redoes
 * the step from PRECSET with the TIG already stored.                   */
agc_status_t agc_job_tpm_advance(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    agc_status_t st = rz_precset(&ip, rz, rz->intime, AGC_JOB_TPM_ADVANCE);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    st = agc_rendz_los_and_normal(&ip, rz);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_rendz_tpi_required_velocity(&ip, rz, AGC_JOB_TPM_ADVANCE);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return rz_tpi_final_displays(ctx, &ip, rz, AGC_JOB_TPM_AFTER_FINAL);
}

/* After S34/35.5: VN1645, then GOTO P35/P75B (next cycle). */
agc_status_t agc_job_tpm_after_final(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = tpi_begin(ctx, &ip);
    return rz_ttogo_display(ctx, &ip, rz, AGC_JOB_TPM_COMPUTE);
}
