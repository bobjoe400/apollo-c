/**********************************************************************
 * FILE NAME: ignition.c
 *
 * PURPOSE:
 *   The CSM burn programs from ignition to their end: IGNITION,
 *   IMPULCHK, PREPTVC, DOTVCON, DOSTRULL/STEERULL/ULAGEOFF, IMPLBURN,
 *   IMPLCONT, ENGINOFF, DOSPSOFF, DOTVCRCS, SPSOFF/MASSBACK, TVCZAP,
 *   POSTBURN, P40RCS, P40MINDB, TIGNOW, POST41, P47CSM, TIGON, P47BODY,
 *   P47BOD.
 *
 * AGC SOURCE:
 *   Comanche055 P40-P47.agc pp. 686-699
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   TIG, TGO      agc_dp_t   I/O  cs 2^28      ignition time, time to go
 *   TEVENT        agc_dp_t   O    cs 2^28      event time for the downlink
 *   CSMMASS       agc_sp_t   I/O  kg 2^16      mass (MASSBACK)
 *   CDUX          uint16_t   I    CDU counts   roll reference (OGAD)
 *   P40TMP        agc_dp_t   I/O  cs 2^28      delay to PREREAD (P47)
 *   OPTIND        agc_sp_t   O    -            optics zero allowed
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_csm_dap_*         TVC and RCS DAP start/stop (csm/dap.h)
 *   agc_integ_midtoav_now    MIDTOAV2 (P47, integration.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None raised here; IMU status, integration, display, executive and
 *   waitlist failures come back as their status.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only. IGNITION .. DOTVCRCS run as waitlist tasks (T5 handed
 *   between the TVC and RCS DAPs), POSTBURN .. P47 as jobs. Restart
 *   groups 3, 4, 5 and 6.
 *
 * NOTES:
 *   MASSBACK restores the part of the pre-decremented mass that did not
 *   flow (V97VCNTR EMDOT, low word, times 1 s).
 *
 * ALGORITHM:
 *   As in the listing.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

#define DOTVCON_DELAY  40  /* DEC 40: 0.4 s for thrust build-up         */
#define DOSTRULL_DELAY 160 /* DEC 160: 0.4 + 1.6 = 2 s to ullage off    */
#define TAILOFF_DELAY  250 /* DEC 250: 2.5 s for SPS tail-off            */
#define TWO_SECS       200 /* 2SECS, cs */
#define CS_PER_SEC     100 /* 1SEC DEC 100                               */

/**********************************************************************
 * FLAGWRD6 bits 15, 14 = 1, 0: the TVC DAP has T5.
 * AGC: none (the FLAGWRD6 bit test the listing repeats, p.700-703).
 *
 * FUNCTION NAME: agc_csm_tvc_on
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when the TVC DAP owns T5
 **********************************************************************/
bool agc_csm_tvc_on(void)
{
    uint16_t w = agc_flagword_get(agc_flag_word(AGC_FLAG_DAPBIT1)); /* FLAGWRD6 (DAPBIT1, DAPBIT2) */

    return ((w & AGC_CSM_T5BITS) == AGC_CSM_T5_TVC);
}

/**********************************************************************
 * PREPTVC (p.695): T5 bits reset, RCS DAP killed (T5LOC = T5IDLOC), 4.3,
 * DOTVCON in 0.4 s.
 *
 * FUNCTION NAME: agc_csm_preptvc
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_csm_preptvc(void)
{
    agc_flag_clear(AGC_FLAG_DAPBIT1);       /* RESET T5 BITS          */
    agc_flag_clear(AGC_FLAG_DAPBIT2);
    agc_rupt_set_t5_task(AGC_TASK_T5IDLOC); /* KILL RCS               */
    agc_phase_newphase(4, 3, true);         /* 4.3 = DOTVCON (40CS)   */

    return (agc_waitlist_fixdelay(DOTVCON_DELAY, AGC_TASK_DOTVCON));
}

/**********************************************************************
 * IGNITION (p.695): OGAD (roll reference), TEVENT, ENGONFLG, SPS on; an
 * impulsive burn sets up the engine-off task, otherwise STRULLSW is set
 * for steering at DOSTRULL; then PREPTVC.
 *
 * FUNCTION NAME: agc_task_ignition_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_task_ignition_csm(void)
{
    agc_csm_dap_com()->ogad = agc_erase_cdux(); /* ROLL DAP REF.    */
    agc_erase_set_tevent((agc_dp_t) agc_time_now_cs());
    agc_flag_set(AGC_FLAG_ENGONFLG);
    agc_chan_or(011, AGC_CSM_SPSON);            /* SPSON                  */

    if (agc_flag_test(AGC_FLAG_IMPULSW))
    { /* IMPULCHK               */
        agc_flag_clear(AGC_FLAG_STRULLSW); /* IMPLBURN               */
        agc_erase_set_tig(agc_erase_tgo() + (agc_dp_t) agc_time_now_cs());
        agc_phase_var_t v = { AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_IMPLCONT, 0, true, 0U };

        agc_phase_change2((uint16_t) 040153, (uint16_t) 007014, &v);
        agc_status_t st = agc_waitlist_call((uint16_t) agc_dp_low(agc_erase_tgo()), AGC_TASK_ENGINOFF); /* status of agc_waitlist_call */

        return ((st != AGC_STATUS_OK) ? st : agc_task_implcont()); /* IMPLCONT if the WAITLIST call worked */
    }

    agc_flag_set(AGC_FLAG_STRULLSW); /* NON-IMPULSIVE          */

    return (agc_csm_preptvc());
}

/**********************************************************************
 * IMPLCONT (p.697): IMPULSW reset (ENGINOFF is set up), V97VCNTR = 0 in
 * case ENGINOFF comes before TVCDAPON, then PREPTVC.
 *
 * FUNCTION NAME: agc_task_implcont
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_implcont(void)
{
    agc_flag_clear(AGC_FLAG_IMPULSW);
    agc_csm_dap_tvc()->engine_fail_vcntr = 0;

    return (agc_csm_preptvc());
}

/**********************************************************************
 * DOTVCON (p.696-697): TVC takes T5: TVCPHASE = -1 (FRESHDAP), TVCEXPHS =
 * 0, T5 bits = 10, 6.3 (CLOKTASK), RCS errors copied for the TVC DAP,
 * 4.5, TIME5 = POSMAX and T5LOC = TVCDAPON, DOSTRULL in 1.6 s.
 *
 * FUNCTION NAME: agc_task_dotvcon
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_dotvcon(void)
{
    agc_csm_tvc_t *tvc = agc_csm_dap_tvc(); /* TVC DAP erasables */
    agc_csm_rcs_t *rcs = agc_csm_dap_rcs(); /* RCS DAP erasables (ERRORX..) */

    tvc->tvcphase   = -1;                        /* CS BIT1                */
    tvc->tvcexphs   = 0;
    agc_flag_set(AGC_FLAG_DAPBIT1);              /* BITS 15,14 = 10        */
    agc_flag_clear(AGC_FLAG_DAPBIT2);
    agc_phase_newphase(6, 3, false);             /* 6.3 = CLOKTASK         */
    tvc->errbtmp.hi = (agc_sp_t) -rcs->error[1]; /* DCS ERRORY: each word */
    tvc->errbtmp.lo = (agc_sp_t) -rcs->error[2]; /* complemented as is    */
    agc_phase_newphase(4, 5, true);              /* 4.5 = DOSTRULL         */
    agc_rupt_set_time(5, (uint16_t) AGC_SP_MAX); /* START RIGHT AWAY   */
    agc_rupt_set_t5_task(AGC_TASK_TVCDAPON);     /* KILLS RCS DAP          */

    return (agc_waitlist_fixdelay(DOSTRULL_DELAY, AGC_TASK_DOSTRULL));
}

/**********************************************************************
 * DOSTRULL / STEERULL / ULAGEOFF (p.696): steering on for a non-impulsive
 * burn, ullage off (channel 5 = 0), group 4 off.
 *
 * FUNCTION NAME: agc_task_dostrull
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_task_dostrull(void)
{
    if (agc_flag_test(AGC_FLAG_STRULLSW))
    {
        agc_flag_set(AGC_FLAG_STEERSW); /* STEERULL               */
    }

    agc_chan_write(05, 0U); /* ULAGEOFF: ZERO CHAN5   */

    agc_phase_kill(4);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ENGINOFF (p.697-698): MASSTMP = CSMMASS (copy cycle for MASSBACK), group
 * 3 off and 4.63, then DOSPSOFF.
 *
 * FUNCTION NAME: agc_task_enginoff_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_enginoff_csm(void)
{
    agc_csm_dap_tvc()->masstmp = agc_erase_csmmass();
    agc_phase_change2((uint16_t) 000003, (uint16_t) 040634, NULL);

    return (agc_task_dospsoff());
}

/**********************************************************************
 * DOSPSOFF (p.698): SPS shutdown, 4.27, DOTVCRCS after 2.5 s of tail-off.
 *
 * FUNCTION NAME: agc_task_dospsoff
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_dospsoff(void)
{
    agc_csm_spsoff();
    agc_phase_newphase(4, 027, true); /* 4.27 = DOTVCRCS        */

    return (agc_waitlist_fixdelay(TAILOFF_DELAY, AGC_TASK_DOTVCRCS));
}

/**********************************************************************
 * DOTVCRCS (p.698): wide deadband, RCS DAP on (kills the TVC DAPs), mass
 * properties, TVCZAP, 4.35 and the POSTBURN job (PRIO12).
 *
 * FUNCTION NAME: agc_task_dotvcrcs
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_dotvcrcs(void)
{
    agc_inhint();
    agc_csm_dap_setmaxdb();
    agc_status_t st = agc_csm_dap_rcsdapon(); /* status of agc_csm_dap_rcsdapon */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_relint();
    agc_csm_dap_massprop();
    agc_inhint();
    agc_csm_tvczap();
    agc_relint();
    agc_phase_change((uint16_t) 000354); /* 4.35 = POSTBURN        */

    return (agc_exec_novac(AGC_PRIO12, AGC_JOB_POSTBURN));
}

/**********************************************************************
 * SPSOFF (p.698): TEVENT, ENGONFLG reset, SPS off, SIV cutoff, MASSBACK.
 *
 * FUNCTION NAME: agc_csm_spsoff
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_csm_spsoff(void)
{
    agc_erase_set_tevent((agc_dp_t) agc_time_now_cs());
    agc_flag_clear(AGC_FLAG_ENGONFLG);   /* RESTARTS WILL SHUT DOWN */
    agc_chan_and(011, (uint16_t) ~AGC_CSM_SPSON);
    agc_chan_or(012, AGC_CSM_SIVCUTOFF); /* SIV CUTOFF COMMAND     */

    /* MASSBACK (p.702-703): restore the part of the pre-decremented mass not
     * flowed: CSMMASS = MASSTMP + (V97VCNTR EMDOT, low word) * 100 cs (high
     * word); the trims follow DELPBAR/DELYBAR once the TVC DAP is running.   */
    agc_csm_tvc_t    *tvc  = agc_csm_dap_tvc();                                       /* TVC DAP erasables */
    agc_csm_dapcom_t *com  = agc_csm_dap_com();                                       /* CSM DAP common erasables */
    int32_t           p    = (int32_t) tvc->engine_fail_vcntr * (int32_t) com->emdot; /* MP EMDOT */
    int32_t           lo   = p % 16384;                                               /* LXCH A   */
    int32_t           corr = (lo * CS_PER_SEC) / 16384;                               /* MP 1SEC  */

    agc_erase_set_csmmass((agc_sp_t) (tvc->masstmp + corr));

    if ((tvc->tvcphase + 1 != 0) && agc_csm_tvc_on()) /* else BTRIMR */
    {
        com->pactoff = tvc->delpbar.hi; /* CA DELPBAR (high word) BESTTRIM */
        com->yactoff = tvc->delybar.hi;
    }
}

/**********************************************************************
 * TVCZAP (p.699, under INHINT): TVC and optics error counters off, optics
 * DAC re-engaged, OPTIND = -1 (optics zero allowed), NVWORD1 = 0, TIMRFLAG
 * reset (stops CLOKTASK).
 *
 * FUNCTION NAME: agc_csm_tvczap
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_csm_tvczap(void)
{
    agc_chan_and(012, (uint16_t) ~AGC_CSM_TVCOPT);
    agc_erase_set_optind(-1); /* CS BIT1                */
    agc_guid_state()->csm.nvword1 = 0;
    agc_flag_clear(AGC_FLAG_TIMRFLAG);
}

/**********************************************************************
 * A flashing display: terminate -> POST41.
 * AGC: none (helper for the flashing displays of P40/P41/P47,
 * pp.692-698).
 *
 * FUNCTION NAME: flash_post41
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *   kind      agc_dsky_disp_t  I    display kind (GOFLASH, GOFLASHR, ...)
 *   verb      uint8_t          I    verb, decimal as keyed
 *   noun      uint8_t          I    noun, decimal as keyed
 *   pro       agc_job_id_t     I    job run on PROCEED
 *   enter     agc_job_id_t     I    job run on ENTER
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t flash_post41(agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint8_t verb, uint8_t noun, agc_job_id_t pro, agc_job_id_t enter)
{
    agc_dsky_cont_t cont = {AGC_JOB_POST41, pro, enter};

    return (agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont));
}

/**********************************************************************
 * POSTBURN (p.686): V16N40 final values; PROCEED -> P40RCS (trim with RCS).
 *
 * FUNCTION NAME: agc_job_postburn_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_postburn_csm(agc_job_ctx_t *ctx)
{
    return (flash_post41(ctx, AGC_DSKY_REFLASH, 16, 40, AGC_JOB_SPS_BURN_RCS_TRIM, AGC_JOB_POSTBURN));
}

/**********************************************************************
 * P40RCS (p.686): AVEGEXIT = CALCN85, wait 2 s for it, then P40MINDB.
 *
 * FUNCTION NAME: agc_job_sps_burn_rcs_trim
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_sps_burn_rcs_trim(void)
{
    agc_guid_state()->avg.avgexit = AGC_JOB_CALC_VG_BODY; /* ACADN85                */

    return (agc_svc_delayjob(TWO_SECS, AGC_JOB_SPS_BURN_MIN_DEADBAND));
}

/**********************************************************************
 * P40MINDB (p.687): narrow deadband, then TIGNOW.
 *
 * FUNCTION NAME: agc_job_sps_burn_min_deadband
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_sps_burn_min_deadband(agc_job_ctx_t *ctx)
{
    agc_inhint();
    agc_csm_dap_setmindb();
    agc_relint();

    return (agc_job_tignow_csm(ctx));
}

/**********************************************************************
 * TIGNOW (p.687): 4 type C (PRIO20, here), V16N85 for an RCS trim.
 *
 * FUNCTION NAME: agc_job_tignow_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_tignow_csm(agc_job_ctx_t *ctx)
{
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_TIGNOW, AGC_TASK_NONE, (int16_t) AGC_PRIO20, false, 0U};

    agc_phase_change_var((uint16_t) 005024, &v);

    return (flash_post41(ctx, AGC_DSKY_REFLASH, 16, 85, AGC_JOB_POST41, AGC_JOB_TIGNOW));
}

/**********************************************************************
 * POST41 (p.687): AVEGEXIT = SERVEXIT, then P00.
 *
 * FUNCTION NAME: agc_job_post41
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_post41(void)
{
    agc_guid_state()->avg.avgexit = AGC_JOB_SERVEXIT; /* SERVCADR               */

    return (agc_exec_gotopooh());
}

/**********************************************************************
 * P47CSM (p.691): IMU status, MIDTOAV2 (time to PREREAD < 1 s), TIGON.
 *
 * FUNCTION NAME: agc_job_thrust_monitor_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_job_thrust_monitor_csm(agc_job_ctx_t *ctx)
{
    agc_status_t sw = agc_imu_status_check(); /* status of agc_imu_status_check */

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_integ_midtoav_out_t mo; /* MIDTOAV results */
    agc_interp_t            ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_integ_midtoav_now(&ip, agc_integ_shared_ctx(), ctx->id, &mo); /* status of agc_integ_midtoav_now */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_store_midtoav(&mo);
    agc_dp_t dt = mo.dt; /* MIDTOAV2 delay to PREREAD, cs 2^28 */

    agc_erase_set_p40tmp((agc_dp_t) agc_dp_low(dt)); /* CA MPAC +1 TS P40TMP */
    st = agc_waitlist_call((uint16_t) agc_erase_p40tmp(), AGC_TASK_TIGON);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change((uint16_t) 040574);

    return (AGC_STATUS_OK); /* 4.57 = TIGON           */
}

/**********************************************************************
 * TIGON (p.691): AVEGEXIT = CALCN83, P47BODY job, 4.2 and PRECHECK.
 *
 * FUNCTION NAME: agc_task_tigon
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_tigon(void)
{
    agc_guid_state()->avg.avgexit = AGC_JOB_CALC_DELTA_V_BODY;
    agc_status_t st               = agc_exec_novac(AGC_PRIO30, AGC_JOB_THRUST_MONITOR_RESET); /* status of agc_exec_novac */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_newphase(4, 2, true); /* 4.2 = PRECHECK, P47BODY */

    return (agc_task_precheck());
}

/**********************************************************************
 * P47BODY / P47BOD / P47/DSP (p.692-693): clear DELVIMU and DELVCTL, lower
 * the priority below CALCN83, 4 type C (PRIO15), V16N83 (ENTER: clear).
 *
 * FUNCTION NAME: agc_job_thrust_monitor_reset_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_thrust_monitor_reset_csm(agc_job_ctx_t *ctx)
{
    agc_burn_t *b = &agc_guid_state()->burn; /* burn erasables */

    b->delvimu = agc_vec_zero();
    b->delvctl = agc_vec_zero();
    agc_exec_prio_change(AGC_PRIO15);

    return (agc_job_thrust_monitor_display_csm(ctx));
}

/**********************************************************************
 * P47BOD (p.692): 5.2 with this job at PRIO15, V16N83 flash: PROCEED
 * and TERMINATE go to P00, ENTER to P47BODY (reset).
 *
 * FUNCTION NAME: agc_job_thrust_monitor_display_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_thrust_monitor_display_csm(agc_job_ctx_t *ctx)
{
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_THRUST_MONITOR_DISPLAY, AGC_TASK_NONE, (int16_t) AGC_PRIO15, false, 0U};

    agc_phase_change_var((uint16_t) 005024, &v);
    agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH, AGC_JOB_GOTOPOOH, AGC_JOB_THRUST_MONITOR_RESET};

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(16, 83), &cont));
}
