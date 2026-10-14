/**********************************************************************
 * FILE NAME: tpi_tpm_targeting.c
 *
 * PURPOSE:
 *   The TPI and TPM targeting programs, both vehicles:
 *     P34 / P74  transfer phase initiation (TPI) targeting
 *     P35 / P75  transfer phase midcourse (TPM) targeting
 *   Each AGC program is one job that sleeps on its flashing displays;
 *   every return from a display (and every resumption after an INTSTALL
 *   sleep) is a continuation job (ids in pending.h); the program state
 *   lives in the rendezvous erasables.
 *
 * AGC SOURCE:
 *   Comanche055 P34-35_P74-75.agc pp. 462-467 (P34, P74, P34/P74A ..
 *   P34/P74E, SWCHCLR, INTLOOP, P35, P75, P35/P75A, P35/P75B)
 *   Luminary099 P34-35_P74-75.agc (differs: on P34/P74A entry the LM
 *   sets CENTANG = 130 deg (130DEG) and NN = 0 before DISPLAYE; the CM
 *   only zeroes NN)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  ----------------------------
 *   TTPI       agc_dp_t   I/O  cs 2^28    TPI time (N37)
 *   TIG        agc_dp_t   O    cs 2^28    = TTPI (P34), now + KT (P35)
 *   ETPIFLAG FINALFLG  flags    I/O  -    elevation given / final pass
 *   ITSWICH    rz flag    I/O  -          iterate TTPI
 *   ELEV CENTANG NN NOMTPI KT TSTRT INTIME (agc_rendz_t) I/O  program
 *              state (revolutions 2^0, cs 2^28)
 *
 * EXTERNAL REFERENCES:
 *   Name                              Description
 *   --------------------------------  --------------------------------
 *   agc_rendz_tpi_time_search         S33/34.1
 *   agc_rendz_tpi_compute             P34/P74E: the TPI solution
 *   agc_rendz_tpi_required_velocity   S34/35.2
 *   rz_tpi_final_displays             S34/35.5
 *   rz_vnpooh rz_displaye rz_alarm_flash rz_ttogo_display  displays
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   00611 (V05N09) when S33/34.1 finds no TPI time. Jobs return
 *   AGC_STATUS_BAD_ARG for a NULL ctx or a failed invariant, else the
 *   status of the display or integration call that failed (INTSTALL
 *   sleep).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of the major mode. DEVIATION: the AGC restarts INTSTALL inside
 *   PRECSET; P35 resumes at agc_job_tpm_advance with TIG already stored.
 *
 * NOTES:
 *   Scales: r m 2^29, v m/cs 2^7, t cs 2^28, angles revolutions 2^0.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/io/service.h"
#include "internal.h"

/* 130DEG 2DEC .3611111111 (LM P34-35 p.684): 130 degrees in revolutions */
#define TPI_130DEG ((agc_dp_t) 96935026) /* 130 deg, revolutions 2^0 */

/**********************************************************************
 * Job prologue: interpreter state and the rendezvous erasables.
 * AGC: none (job prologue; the AGC uses fixed erasables)
 *
 * FUNCTION NAME: tpi_begin
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_rendz_t * -- the rendezvous erasables (never NULL)
 **********************************************************************/
static agc_rendz_t *tpi_begin(agc_interp_t *ip)
{
    agc_interp_init(ip);

    return (agc_rendz_erasable());
}

/* ======================= P34 / P74 (pp.462-465) ========================== */
/**********************************************************************
 * P34 TPI TARGETING (P34-35_P74-75.agc p.462, label P34): AVFLAGA.
 *
 * FUNCTION NAME: agc_job_tpi_targeting
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_targeting(agc_job_ctx_t *ctx)
{
    rz_avflag(true);

    return (agc_job_tpi_input(ctx));
}

/**********************************************************************
 * P74 TPI TARGETING, OTHER VEHICLE ACTIVE (p.462, label P74): AVFLAGP.
 *
 * FUNCTION NAME: agc_job_tpi_targeting_other_vehicle
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_targeting_other_vehicle(agc_job_ctx_t *ctx)
{
    rz_avflag(false);

    return (agc_job_tpi_input(ctx));
}

/**********************************************************************
 * P34/P74A (p.462): P20FLGON (UPDATFLG, TRACKFLG), V06N37 TTPI.
 *
 * FUNCTION NAME: agc_job_tpi_input
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_input(agc_job_ctx_t *ctx)
{
    rz_tracking_flags_on();

    return (rz_vnpooh(ctx, 6U, 37U, AGC_JOB_TPI_AFTER_TTPI));
}

/**********************************************************************
 * After V06N37: NN = 0 (LM also CENTANG = 130 deg), DISPLAYE V06N55.
 * AGC: P34/P74A after V06N37 (P34-35_P74-75.agc p.462)
 *
 * FUNCTION NAME: agc_job_tpi_after_ttpi
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_after_ttpi(agc_job_ctx_t *ctx)
{
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    if (!rz_is_csm())
    {
        rz->centang = TPI_130DEG; /* DCA 130DEG, DXCH     */
    }

    rz->nn = 0; /* SSP NN 0 / TS NN     */

    if (!AGC_ASSERT(rz->centang >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_displaye(ctx, AGC_JOB_TPI_AFTER_ELEV));
}

/**********************************************************************
 * After DISPLAYE: CLEAR ETPIFLAG, TIG = TTPI; ELEV non-zero -> SET
 * ETPIFLAG (the crew gave an elevation: find TTPI for it). P34/P74B:
 * SELECTMU, VN1645, then P34/P74C.
 * AGC: P34/P74A, P34/P74B (P34-35_P74-75.agc p.462)
 *
 * FUNCTION NAME: agc_job_tpi_after_elev
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_after_elev(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */

    agc_flag_clear(AGC_FLAG_ETPIFLAG);
    agc_erase_set_tig(agc_erase_ttpi());

    if (rz->elev != 0)
    { /* BZE P34/P74B         */
        agc_flag_set(AGC_FLAG_ETPIFLAG);
    }

    return (rz_selectmu_display(ctx, &ip, rz, AGC_JOB_TPI_CYCLE));
}

/**********************************************************************
 * P34/P74C (p.463): NOMTPI = 0; SET ITSWICH; ETPIFLAG clear -> CLEAR
 * ITSWICH (only compute the elevation at the given TTPI). INTLOOP.
 *
 * FUNCTION NAME: agc_job_tpi_cycle
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_cycle(agc_job_ctx_t *ctx)
{
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    rz_vflag_set(RZ_VF_ITSWICH, true);

    if (!agc_flag_test(AGC_FLAG_ETPIFLAG))
    { /* BON ETPIFLAG SWCHSET */
        rz_vflag_set(RZ_VF_ITSWICH, false);
    }

    rz->nomtpi = 0; /* SWCHSET STORE NOMTPI */

    if (!AGC_ASSERT(rz->nomtpi == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_tpi_intloop(ctx));
}

/**********************************************************************
 * SWCHCLR (p.463), after S33/34.1 succeeded: ITSWICH was set -> clear it
 * and go round INTLOOP once more (the elevation at the new TTPI);
 * ETPIFLAG set -> V06N37 (the new TTPI), else DISPLAYE; then P34/P74E.
 * AGC: SWCHCLR, P34/P74D (P34-35_P74-75.agc p.463)
 *
 * FUNCTION NAME: tpi_after_search
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   st        agc_status_t     I    status of S33/34.1 (OK, ALARM, or an
 *                                   integration status)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t tpi_after_search(agc_job_ctx_t *ctx, agc_status_t st)
{
    if (st == AGC_STATUS_ALARM)
    { /* BZE fails: 611       */
        return (agc_job_tpi_alarm(ctx));
    }

    if (st != AGC_STATUS_OK)
    {
        return (st); /* INTSTALL sleep       */
    }

    if (rz_vflag_test(RZ_VF_ITSWICH))
    { /* BONCLR ITSWICH       */
        rz_vflag_set(RZ_VF_ITSWICH, false);

        return (rz_continue(AGC_JOB_TPI_INTLOOP));
    }

    if (agc_flag_test(AGC_FLAG_ETPIFLAG))
    { /* P34/P74D             */
        return (rz_vnpooh(ctx, 6U, 37U, AGC_JOB_TPI_COMPUTE));
    }

    return (rz_displaye(ctx, AGC_JOB_TPI_COMPUTE));
}

/**********************************************************************
 * INTLOOP (p.463): TDEC1 = TTPI + NOMTPI, PRECSET (both vehicles), then
 * S33/34.1 (TPI time search / elevation).
 *
 * FUNCTION NAME: agc_job_tpi_intloop
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_intloop(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */
    agc_dp_t     tdec1;               /* cs 2^28 */

    tdec1 = agc_dp_add(&ip, agc_erase_ttpi(), rz->nomtpi);
    agc_status_t st; /* PRECSET */

    st = rz_precset(&ip, rz, tdec1, AGC_JOB_TPI_INTLOOP);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_rendz_tpi_time_search(&ip, rz, AGC_JOB_TPI_TIME_SEARCH);

    return (tpi_after_search(ctx, st));
}

/**********************************************************************
 * S33/34.1 resumed after an INTSTALL sleep.
 * AGC: S33/34.1 (P34-35_P74-75.agc p.469)
 *
 * FUNCTION NAME: agc_job_tpi_time_search
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_time_search(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */
    agc_status_t st;                  /* S33/34.1 */

    st = agc_rendz_tpi_time_search_cont(&ip, rz, AGC_JOB_TPI_TIME_SEARCH);

    return (tpi_after_search(ctx, st));
}

/**********************************************************************
 * TC ALARM OCT 611, V05N09 (p.463): TERMINATE -> P00, PROCEED ->
 * P34/P74A, V32 ("TC -7") -> the alarm and display again.
 *
 * FUNCTION NAME: agc_job_tpi_alarm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_alarm(agc_job_ctx_t *ctx)
{
    return (rz_alarm_flash(ctx, (agc_alarm_code_t) AGC_ALARM_00611, AGC_JOB_TPI_INPUT, AGC_JOB_TPI_ALARM));
}

/**********************************************************************
 * P34/P74E (pp.464-465): the TPI solution (agc_rendz_tpi_compute), V06N58
 * (POSTTPI, DELVTPI, DELVTPF).
 *
 * FUNCTION NAME: agc_job_tpi_compute
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */
    agc_status_t st;                  /* P34/P74E */

    st = agc_rendz_tpi_compute(&ip, rz, AGC_JOB_TPI_COMPUTE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rz_vnpooh(ctx, 6U, 58U, AGC_JOB_TPI_AFTER_RESULTS));
}

/**********************************************************************
 * After V06N58: S34/35.5 (final-pass displays), then VN1645.
 * AGC: P34/P74E after V06N58 (P34-35_P74-75.agc p.465)
 *
 * FUNCTION NAME: agc_job_tpi_after_results
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_after_results(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */

    return (rz_tpi_final_displays(ctx, &ip, rz, AGC_JOB_TPI_AFTER_FINAL));
}

/**********************************************************************
 * After S34/35.5: VN1645, then GOTO P34/P74C (next cycle).
 * AGC: P34/P74E "GOTO P34/P74C" (P34-35_P74-75.agc p.465)
 *
 * FUNCTION NAME: agc_job_tpi_after_final
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_after_final(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */

    return (rz_ttogo_display(ctx, &ip, rz, AGC_JOB_TPI_CYCLE));
}

/* ======================= P35 / P75 (pp.466-467) ========================== */
/**********************************************************************
 * P35/P75A: KT = the TIG increment, P20FLGON, SELECTMU (VN1645), then
 * P35/P75B.
 * AGC: P35/P75A (P34-35_P74-75.agc p.466)
 *
 * FUNCTION NAME: tpm_start
 *
 * ARGUMENTS:
 *   Argument     Type             I/O  Description (scale)
 *   -----------  ---------------  ---  ----------------------------
 *   ctx          agc_job_ctx_t *  I    job context (checked, not used)
 *   this_active  bool             I    true: this vehicle is the active one
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t tpm_start(agc_job_ctx_t *ctx, bool this_active)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */

    rz_avflag(this_active);
    rz->kt = this_active ? rz->atiginc : rz->ptiginc; /* DCA xTIGINC */
    rz_tracking_flags_on();

    return (rz_selectmu_display(ctx, &ip, rz, AGC_JOB_TPM_COMPUTE));
}

/**********************************************************************
 * P35 TPM TARGETING (p.466, label P35): AVFLAGA, KT = ATIGINC.
 *
 * FUNCTION NAME: agc_job_tpm_targeting
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpm_targeting(agc_job_ctx_t *ctx)
{
    return (tpm_start(ctx, true));
}

/**********************************************************************
 * P75 TPM TARGETING, OTHER VEHICLE ACTIVE (p.466, label P75): AVFLAGP,
 * KT = PTIGINC.
 *
 * FUNCTION NAME: agc_job_tpm_targeting_other_vehicle
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpm_targeting_other_vehicle(agc_job_ctx_t *ctx)
{
    return (tpm_start(ctx, false));
}

/**********************************************************************
 * P35/P75B (p.466): TSTRT = now; TIG = INTIME = TDEC1 = TSTRT + KT.
 *
 * FUNCTION NAME: agc_job_tpm_compute
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpm_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */

    rz->tstrt    = agc_rtb_loadtime();                 /* RTB LOADTIME        */
    agc_dp_t tig = agc_dp_add(&ip, rz->tstrt, rz->kt); /* TSTRT + KT, cs 2^28 */

    agc_erase_set_tig(tig);
    rz->intime = tig; /* FOR INITVEL         */

    if (!AGC_ASSERT(rz->tstrt >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_tpm_advance(ctx));
}

/**********************************************************************
 * P35/P75B continued: PRECSET to TIG (both vehicles), S34/35.1 (LOS and
 * normal), S34/35.2 (delta V), S34/35.5. Resumed here (not at LOADTIME)
 * after an INTSTALL sleep, so TIG is not moved by a sleep.
 * DEVIATION: the AGC restarts INTSTALL inside PRECSET; the C job redoes
 * the step from PRECSET with the TIG already stored.
 * AGC: P35/P75B (P34-35_P74-75.agc p.466)
 *
 * FUNCTION NAME: agc_job_tpm_advance
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpm_advance(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */
    agc_status_t st;                  /* PRECSET */

    st = rz_precset(&ip, rz, rz->intime, AGC_JOB_TPM_ADVANCE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_rendz_los_and_normal(&ip, rz);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_rendz_tpi_required_velocity(&ip, rz, AGC_JOB_TPM_ADVANCE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rz_tpi_final_displays(ctx, &ip, rz, AGC_JOB_TPM_AFTER_FINAL));
}

/**********************************************************************
 * After S34/35.5: VN1645, then GOTO P35/P75B (next cycle).
 * AGC: P35/P75B "GOTO P35/P75B" (P34-35_P74-75.agc p.467)
 *
 * FUNCTION NAME: agc_job_tpm_after_final
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpm_after_final(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = tpi_begin(&ip); /* rendezvous erasables */

    return (rz_ttogo_display(ctx, &ip, rz, AGC_JOB_TPM_COMPUTE));
}
