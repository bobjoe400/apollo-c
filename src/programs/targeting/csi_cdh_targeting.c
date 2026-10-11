/**********************************************************************
 * FILE NAME: csi_cdh_targeting.c
 *
 * PURPOSE:
 *   The CSI and CDH targeting programs, both vehicles:
 *     P32 / P72  coelliptic sequence initiation (CSI) targeting
 *     P33 / P73  constant delta height (CDH) targeting
 *   Each AGC program is one job that sleeps on its flashing displays;
 *   here every return from a display is a continuation job (job ids in
 *   pending.h) and the program state lives in the rendezvous erasables.
 *
 * AGC SOURCE:
 *   Comanche055 P32-P33_P72-P73.agc pp. 652-662 (P32, P72, P32/P72A ..
 *   P32/P72F, ALMXIT, P33, P73, P33/P73A .. P33/P73F)
 *   Luminary099 P32-P35_P72-P75.agc (same program text; the LM computes
 *   TCSI at the apocenter when the crew loads none, and keeps NN)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  ----------------------------
 *   TIG        agc_dp_t   O    cs 2^28    = TCSI (P32) / TCDH (P33)
 *   TTPI       agc_dp_t   I/O  cs 2^28    TPI time (N37)
 *   TETLEM     agc_dp_t   I    cs 2^28    LM state time (auto TCSI)
 *   FINALFLG UPDATFLG RVSW  flags  I/O -  display / update control
 *   TCSI TCDH NN ELEV TTPIO NOMTPI T1TOT2 T2TOT3 DELVEET1 DELVEET2
 *              (agc_rendz_t)  I/O        program state (N11 N13 N55
 *                                        N37 N75 N81 N82)
 *
 * EXTERNAL REFERENCES:
 *   Name                         Description
 *   ---------------------------  -------------------------------------
 *   agc_rendz_advance            ADVANCE: states at TIG
 *   agc_rendz_csi_a              CSI/A: the CSI iteration
 *   agc_rendz_cdhmvr             CDHMVR: the CDH maneuver
 *   agc_rendz_tpi_time_search    S33/34.1: new TTPI for ELEV
 *   rz_vnpooh rz_flash           VNPOOH / GOFLASH display requests
 *   rz_disdvlvc rz_ttogo_display DISDVLVC, VN1645
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   00600 .. 00606 (ALMXIT, VARALARM) for no CSI solution; 00611 when
 *   S33/34.1 finds no TPI time. Both show V05N09. Jobs return
 *   AGC_STATUS_BAD_ARG for a NULL ctx, else the status of the display
 *   or integration call that failed (INTSTALL sleep).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of the major mode; at most CSI_MOD_LOOP_MAX passes in the
 *   modulo-one-hour loops.
 *
 * NOTES:
 *   Scales: r m 2^29, v m/cs 2^7, t cs 2^28.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"

/* Bound of the modulo-one-hour loops P32/P72E, P33/P73E, P33/P73F:
 * more than 2^28 cs / 60 min passes.                                    */
#define CSI_MOD_LOOP_MAX 800 /* passes, > 2^28 cs / 60 min */

/**********************************************************************
 * Job prologue: interpreter state and the rendezvous erasables.
 * AGC: none (job prologue; the AGC uses fixed erasables)
 *
 * FUNCTION NAME: csi_begin
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_rendz_t * -- the rendezvous erasables (never NULL)
 **********************************************************************/
static agc_rendz_t *csi_begin(agc_interp_t *ip)
{
    agc_interp_init(ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    return (rz);
}

/**********************************************************************
 * STORE X; DSU 60MIN; BPL (repeat): X modulo one hour (P32/P72E).
 * AGC: P32/P72E "STORE X; DSU 60MIN; BPL" (P32-P33_P72-P73.agc p.654)
 *
 * FUNCTION NAME: csi_mod_hour
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   x         agc_dp_t        I    time, cs 2^28, not negative
 *
 * RETURNS: agc_dp_t -- X modulo one hour, cs 2^28 (0 when the invariant
 *          fails)
 **********************************************************************/
static agc_dp_t csi_mod_hour(agc_interp_t *ip, agc_dp_t x)
{
    agc_dp_t v = x; /* remainder, cs 2^28 */

    for (int i = 0; i < CSI_MOD_LOOP_MAX; i++)
    {
        agc_dp_t d = agc_dp_sub(ip, v, RZ_60MIN); /* DSU 60MIN, cs 2^28 */

        if (d < 0)
        {
            break;
        }

        v = d;
    }

    if (!AGC_ASSERT(v < RZ_60MIN))
    {
        return (0); /* no valid result */
    }

    return (v);
}

/**********************************************************************
 * SELECTMU (p.494) then its GOTO VN1645, which returns to the caller.
 * AGC: SELECTMU (P34-35_P74-75.agc p.494), VN1645
 *
 * FUNCTION NAME: csi_selectmu
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *          I/O  rendezvous erasables
 *   next      agc_job_id_t           I    job started next
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t csi_selectmu(const agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t next)
{
    agc_status_t st; /* SELECTMU */

    st = agc_rendz_selectmu(rz, agc_flag_test(AGC_FLAG_CMOONFLG));

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_FINALFLG);

    return (rz_ttogo_display(ctx, ip, rz, next));
}

/* ======================= P32 / P72 (pp.652-654) ========================== */
/**********************************************************************
 * P32 (this vehicle active) / P72 (other vehicle active): AVFLAGA/P,
 * P32STRT: CENTANG = 0, then P32/P72A.
 * AGC: P32STRT (P32-P33_P72-P73.agc p.652)
 *
 * FUNCTION NAME: csi_start
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
static agc_status_t csi_start(agc_job_ctx_t *ctx, bool this_active)
{
    rz_avflag(this_active);
    agc_rendz_erasable()->centang = 0;

    return (agc_job_csi_input(ctx));
}

/**********************************************************************
 * P32 CSI TARGETING (P32-P33_P72-P73.agc p.652, label P32)
 *
 * FUNCTION NAME: agc_job_csi_targeting
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
agc_status_t agc_job_csi_targeting(agc_job_ctx_t *ctx)
{
    return (csi_start(ctx, true));
}

/**********************************************************************
 * P72 CSI TARGETING, OTHER VEHICLE ACTIVE (p.652, label P72)
 *
 * FUNCTION NAME: agc_job_csi_targeting_other_vehicle
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
agc_status_t agc_job_csi_targeting_other_vehicle(agc_job_ctx_t *ctx)
{
    return (csi_start(ctx, false));
}

/**********************************************************************
 * P32/P72A (p.652): P20FLGON; NN = 0 (CM) / NN+1 = TCSI = 0 (LM);
 * V06N11 TCSI.
 *
 * FUNCTION NAME: agc_job_csi_input
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
agc_status_t agc_job_csi_input(agc_job_ctx_t *ctx)
{
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    rz_tracking_flags_on();

    if (rz_is_csm())
    {
        rz->nn = 0;
    }
    else
    {
        rz->tcsi = 0; /* TS NN +1: the low word of NN is not modelled */
    }

    return (rz_vnpooh(ctx, 6U, 11U, AGC_JOB_CSI_AFTER_TCSI));
}

/**********************************************************************
 * LM VN0611 (p.~): TCSI <= 0 -> TCSI = time of the next apocenter of the
 * LM state (PRECSET at TETLEM, TIMERAD to DPPOSMAX), V06N11 again.
 * AGC: VN0611 (P32-P35_P72-P75.agc p.621)
 *
 * FUNCTION NAME: csi_lm_auto_tcsi
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   ip        agc_interp_t *   I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *    I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t csi_lm_auto_tcsi(agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_status_t st; /* PRECSET */

    st = rz_precset(ip, rz, agc_erase_tetlem(), AGC_JOB_CSI_AFTER_TCSI);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_timerad_in_t in; /* TIMERAD inputs */

    in.rvec     = agc_rendz_vec_to_body(ip, rz, rz->ract3);
    in.vvec     = agc_rendz_vec_to_body(ip, rz, rz->vact3);
    in.rdesired = RZ_DPPOSMAX; /* beyond the apocenter      */
    in.sgnrdot  = AGC_SP_HALF; /* not set by the AGC; unused at APSESW */
    in.rvsw     = true;
    in.body     = agc_conic_body_of_x1(rz->body.rtx1);
    rz_flag_put(AGC_FLAG_RVSW, true);
    agc_conic_time_out_t o;    /* conic result */

    st = agc_conic_timerad(ip, &in, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->tcsi = agc_dp_add(ip, o.t, rz->tdec2);

    return (rz_vnpooh(ctx, 6U, 11U, AGC_JOB_CSI_AFTER_TCSI));
}

/**********************************************************************
 * After V06N11: (LM: automatic TCSI) V06N55 NN, ELEV.
 * AGC: P32/P72A after V06N11, LM VN0611 (P32-P35_P72-P75.agc)
 *
 * FUNCTION NAME: agc_job_csi_after_tcsi
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
agc_status_t agc_job_csi_after_tcsi(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */

    if (!rz_is_csm() && agc_dp_neg(rz->tcsi) >= 0)
    {
        return (csi_lm_auto_tcsi(ctx, &ip, rz));
    }

    return (rz_vnpooh(ctx, 6U, 55U, AGC_JOB_CSI_AFTER_NN_ELEV));
}

/**********************************************************************
 * After V06N55: V06N37 TTPI.
 * AGC: P32/P72A after V06N55 (P32-P33_P72-P73.agc p.652)
 *
 * FUNCTION NAME: agc_job_csi_after_nn_elev
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
agc_status_t agc_job_csi_after_nn_elev(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_rendz_erasable()->nn >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_vnpooh(ctx, 6U, 37U, AGC_JOB_CSI_AFTER_TTPI));
}

/**********************************************************************
 * After V06N37: TIG = TCSI, SELECTMU (V16N45), then P32/P72B.
 * AGC: P32/P72A after V06N37 (P32-P33_P72-P73.agc p.652)
 *
 * FUNCTION NAME: agc_job_csi_after_ttpi
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
agc_status_t agc_job_csi_after_ttpi(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */

    agc_erase_set_tig(rz->tcsi);

    return (csi_selectmu(ctx, &ip, rz, AGC_JOB_CSI_COMPUTE));
}

/**********************************************************************
 * ALMXIT (p.652): VARALARM 00600 + CSIALRM - 1, V05N09: PROCEED shows it
 * again ("TC -4"), RECYCLE goes back to P32/P72A.
 * AGC: ALMXIT (P32-P33_P72-P73.agc p.652)
 *
 * FUNCTION NAME: csi_almxit
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   code      agc_sp_t         I    alarm code 00600 .. 00606
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t csi_almxit(agc_job_ctx_t *ctx, agc_sp_t code)
{
    if (!AGC_ASSERT(code >= AGC_ALARM_CSI_00600 && code <= 00606))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_exec_varalarm((agc_alarm_code_t) code);

    return (agc_job_csi_alarm_display(ctx));
}

/**********************************************************************
 * ALMXIT display: V05N09 flashing; PROCEED shows it again ("TC -4"),
 * RECYCLE returns to P32/P72A, TERMINATE to GOTOPOOH.
 * AGC: ALMXIT (P32-P33_P72-P73.agc p.652)
 *
 * FUNCTION NAME: agc_job_csi_alarm_display
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
agc_status_t agc_job_csi_alarm_display(agc_job_ctx_t *ctx)
{
    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 5U, 9U, AGC_JOB_GOTOPOOH, AGC_JOB_CSI_ALARM_DISPLAY, AGC_JOB_CSI_INPUT));
}

/**********************************************************************
 * P32/P72C .. P32/P72F (pp.653-654): UPDATFLG unless final, the times
 * modulo one hour, V06N75.
 * AGC: P32/P72C .. P32/P72F (P32-P33_P72-P73.agc pp.653-654)
 *
 * FUNCTION NAME: csi_results
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   ip        agc_interp_t *   I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *    I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t csi_results(agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz)
{
    if (!agc_flag_test(AGC_FLAG_FINALFLG))
    {
        agc_flag_set(AGC_FLAG_UPDATFLG);
    }

    rz->t1tot2 = csi_mod_hour(ip, rz->t1tot2); /* P32/P72E  */
    rz->t2tot3 = csi_mod_hour(ip, rz->t2tot3); /* P32/P72F  */

    return (rz_vnpooh(ctx, 6U, 75U, AGC_JOB_CSI_AFTER_RESULTS));
}

/**********************************************************************
 * P32/P72B (p.653): ADVANCE; passive TCSI -> TTPI (conic) as PASSIVE;
 * CSI/A; ALMXIT on failure.
 *
 * FUNCTION NAME: agc_job_csi_compute
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
agc_status_t agc_job_csi_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */
    agc_status_t st;                  /* ADVANCE */

    st = agc_rendz_advance(&ip, rz, AGC_JOB_CSI_COMPUTE);
    agc_vec_t r; /* position, m 2^29 */
    agc_vec_t v; /* velocity, m/cs 2^7 */

    if (st == AGC_STATUS_OK)
    {
        st = rz_intint(&ip, rz, rz->rpass1, rz->vpass1, rz->tcsi, agc_erase_ttpi(), true, AGC_JOB_CSI_COMPUTE, &r, &v);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_store_passive(rz, r, v);
    agc_sp_t alarm = 0; /* ALMXIT code, unscaled octal */

    st = agc_rendz_csi_a(&ip, rz, &alarm, AGC_JOB_CSI_COMPUTE);

    if (st == AGC_STATUS_ALARM)
    {
        return (csi_almxit(ctx, alarm));
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (csi_results(ctx, &ip, rz));
}

/**********************************************************************
 * After V06N75: S32/33.1 with DELVEET1 -> DISDVLVC V06N81.
 * AGC: P32/P72D after V06N75 (P32-P33_P72-P73.agc p.653)
 *
 * FUNCTION NAME: agc_job_csi_after_results
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
agc_status_t agc_job_csi_after_results(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */

    return (rz_disdvlvc(ctx, &ip, rz, rz->delveet1, 81U, AGC_JOB_CSI_AFTER_DV_LV));
}

/**********************************************************************
 * After V06N81: S32/33.1 tail (DELVEET1 = DELVSIN), RACT1 = RACT2,
 * DISDVLVC of DELVEET2 on V06N82.
 * AGC: P32/P72D after V06N81 (P32-P33_P72-P73.agc p.653)
 *
 * FUNCTION NAME: agc_job_csi_after_dv_lv
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
agc_status_t agc_job_csi_after_dv_lv(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */
    agc_vec_t    dv;                  /* DELVSIN, m/cs 2^7 */
    agc_status_t st;                  /* S32/33.1 */

    st = agc_rendz_csi_cdh_dv_to_reference(&ip, rz, &dv);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->delveet1 = dv;
    rz->ract1    = rz->ract2;

    return (rz_disdvlvc(ctx, &ip, rz, rz->delveet2, 82U, AGC_JOB_CSI_AFTER_CDH_DV));
}

/**********************************************************************
 * After V06N82: TTPIO = TTPI, VN1645, back to P32/P72B.
 * AGC: P32/P72D after V06N82 (P32-P33_P72-P73.agc p.653)
 *
 * FUNCTION NAME: agc_job_csi_after_cdh_dv
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
agc_status_t agc_job_csi_after_cdh_dv(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */

    rz->ttpio = agc_erase_ttpi();

    return (rz_ttogo_display(ctx, &ip, rz, AGC_JOB_CSI_COMPUTE));
}

/**********************************************************************
 * P33 / P73 start: AVFLAGA (this vehicle active) or AVFLAGP, then P33/P73A.
 * AGC: P33, P73 (P32-P33_P72-P73.agc p.658)
 *
 * FUNCTION NAME: cdh_start
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
static agc_status_t cdh_start(agc_job_ctx_t *ctx, bool this_active)
{
    rz_avflag(this_active);

    return (agc_job_cdh_input(ctx));
}

/**********************************************************************
 * P33 CDH TARGETING (P32-P33_P72-P73.agc p.658, label P33)
 *
 * FUNCTION NAME: agc_job_cdh_targeting
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
agc_status_t agc_job_cdh_targeting(agc_job_ctx_t *ctx)
{
    return (cdh_start(ctx, true));
}

/**********************************************************************
 * P73 CDH TARGETING, OTHER VEHICLE ACTIVE (p.658, label P73)
 *
 * FUNCTION NAME: agc_job_cdh_targeting_other_vehicle
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
agc_status_t agc_job_cdh_targeting_other_vehicle(agc_job_ctx_t *ctx)
{
    return (cdh_start(ctx, false));
}

/**********************************************************************
 * P33/P73A: P20FLGON, V06N13 TCDH.
 * AGC: P33/P73A (P32-P33_P72-P73.agc p.658)
 *
 * FUNCTION NAME: agc_job_cdh_input
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
agc_status_t agc_job_cdh_input(agc_job_ctx_t *ctx)
{
    rz_tracking_flags_on();

    return (rz_vnpooh(ctx, 6U, 13U, AGC_JOB_CDH_AFTER_TCDH));
}

/**********************************************************************
 * After V06N13: TTPI = TTPIO, TIG = TCDH, SELECTMU, then P33/P73B.
 * AGC: P33/P73A after V06N13 (P32-P33_P72-P73.agc p.658)
 *
 * FUNCTION NAME: agc_job_cdh_after_tcdh
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
agc_status_t agc_job_cdh_after_tcdh(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */

    agc_erase_set_ttpi(rz->ttpio);
    agc_erase_set_tig(rz->tcdh);

    return (csi_selectmu(ctx, &ip, rz, AGC_JOB_CDH_COMPUTE));
}

/**********************************************************************
 * P33/P73C .. P33/P73F (p.659): UPDATFLG unless final; TTPI += NOMTPI;
 * T1TOT2 = (TTPI - TCDH) mod 1 h; T2TOT3 = (TTPI - TTPIO) mod 1 h
 * (signed); V06N75.
 * AGC: P33/P73C .. P33/P73F (P32-P33_P72-P73.agc p.659)
 *
 * FUNCTION NAME: cdh_results
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   ip        agc_interp_t *   I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *    I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t cdh_results(agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz)
{
    if (!agc_flag_test(AGC_FLAG_FINALFLG))
    {
        agc_flag_set(AGC_FLAG_UPDATFLG);
    }

    agc_dp_t ttpi = agc_dp_add(ip, rz->nomtpi, agc_erase_ttpi()); /* cs 2^28 */

    agc_erase_set_ttpi(ttpi);
    agc_dp_t x = agc_dp_sub(ip, ttpi, rz->tcdh); /* T1TOT2, cs 2^28 */

    if (x >= 0)
    {
        x = csi_mod_hour(ip, x); /* negative x is kept      */
    }

    rz->t1tot2 = x;
    agc_dp_t y = agc_dp_sub(ip, ttpi, rz->ttpio); /* T2TOT3, cs 2^28 */

    rz->t2tot3 = agc_dp_sign(csi_mod_hour(ip, agc_dp_abs(y)), y);

    return (rz_vnpooh(ctx, 6U, 75U, AGC_JOB_CDH_AFTER_RESULTS));
}

/**********************************************************************
 * S33/34.1 result: OK -> P33/P73C; no solution -> ALARM 611, V05N09.
 * AGC: P33/P73B "TC ALARM OCT 611" (P32-P33_P72-P73.agc p.658)
 *
 * FUNCTION NAME: cdh_after_search
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   ip        agc_interp_t *   I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *    I/O  rendezvous erasables
 *   st        agc_status_t     I    status of S33/34.1 (OK, ALARM, or an
 *                                   integration status)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t cdh_after_search(agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz, agc_status_t st)
{
    if (st == AGC_STATUS_ALARM)
    {
        return (rz_alarm_flash(ctx, (agc_alarm_code_t) AGC_ALARM_00611, AGC_JOB_CDH_ALARM_PROCEED, AGC_JOB_CDH_INPUT));
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (cdh_results(ctx, ip, rz));
}

/**********************************************************************
 * P33/P73B (p.658): ADVANCE, CDHMVR, active and passive TCDH -> TTPI
 * (precision, INTINT3P), NOMTPI = 0, ITSWICH, S33/34.1.
 *
 * FUNCTION NAME: agc_job_cdh_compute
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
agc_status_t agc_job_cdh_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */
    agc_status_t st;                  /* ADVANCE */

    st = agc_rendz_advance(&ip, rz, AGC_JOB_CDH_COMPUTE);

    if (st == AGC_STATUS_OK)
    {
        st = agc_rendz_cdhmvr(&ip, rz);
    }

    agc_vec_t r; /* position, m 2^29 */
    agc_vec_t v; /* velocity, m/cs 2^7 */

    if (st == AGC_STATUS_OK)
    {
        st = rz_intint(&ip, rz, rz->ract2, rz->vact3, rz->tcdh, agc_erase_ttpi(), false, AGC_JOB_CDH_COMPUTE, &r, &v);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_store_active(rz, r, v);
    st = rz_intint(&ip, rz, rz->rpass2, rz->vpass2, rz->tcdh, agc_erase_ttpi(), false, AGC_JOB_CDH_COMPUTE, &r, &v);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_store_passive(rz, r, v);
    rz->nomtpi = 0;
    rz_vflag_set(RZ_VF_ITSWICH, true);
    st         = agc_rendz_tpi_time_search(&ip, rz, AGC_JOB_CDH_TIME_SEARCH);

    return (cdh_after_search(ctx, &ip, rz, st));
} /* agc_job_cdh_compute */

/**********************************************************************
 * S33/34.1 resumed after an INTSTALL sleep.
 * AGC: S33/34.1 (P34-35_P74-75.agc p.469)
 *
 * FUNCTION NAME: agc_job_cdh_time_search
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
agc_status_t agc_job_cdh_time_search(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */
    agc_status_t st;                  /* S33/34.1 */

    st = agc_rendz_tpi_time_search_cont(&ip, rz, AGC_JOB_CDH_TIME_SEARCH);

    return (cdh_after_search(ctx, &ip, rz, st));
}

/**********************************************************************
 * 611 PROCEED (p.659): NOMTPI = 0, P33/P73C.
 * AGC: P33/P73B 611 PROCEED (P32-P33_P72-P73.agc p.659)
 *
 * FUNCTION NAME: agc_job_cdh_alarm_proceed
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
agc_status_t agc_job_cdh_alarm_proceed(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */

    rz->nomtpi = 0;

    return (cdh_results(ctx, &ip, rz));
}

/**********************************************************************
 * After V06N75: S32/33.1 with DELVEET2 -> DISDVLVC V06N81.
 * AGC: P33/P73D after V06N75 (P32-P33_P72-P73.agc p.659)
 *
 * FUNCTION NAME: agc_job_cdh_after_results
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
agc_status_t agc_job_cdh_after_results(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */

    return (rz_disdvlvc(ctx, &ip, rz, rz->delveet2, 81U, AGC_JOB_CDH_AFTER_DV_LV));
}

/**********************************************************************
 * After V06N81: DELVEET2 = DELVSIN, VN1645, back to P33/P73B.
 * AGC: P33/P73D after V06N81 (P32-P33_P72-P73.agc p.660)
 *
 * FUNCTION NAME: agc_job_cdh_after_dv_lv
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
agc_status_t agc_job_cdh_after_dv_lv(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = csi_begin(&ip); /* rendezvous erasables */
    agc_vec_t    dv;                  /* DELVSIN, m/cs 2^7 */
    agc_status_t st;                  /* S32/33.1 */

    st = agc_rendz_csi_cdh_dv_to_reference(&ip, rz, &dv);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->delveet2 = dv;

    return (rz_ttogo_display(ctx, &ip, rz, AGC_JOB_CDH_COMPUTE));
}
