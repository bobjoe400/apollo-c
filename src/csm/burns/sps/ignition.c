/*
 * ignition.c -- P40-P47.agc, Comanche055 pp. 696-704: from ignition to
 * the end of the burn program: IGNITION, IMPULCHK, PREPTVC, DOTVCON,
 * DOSTRULL/STEERULL/ULAGEOFF, IMPLBURN, IMPLCONT, ENGINOFF, DOSPSOFF,
 * DOTVCRCS, SPSOFF/MASSBACK, TVCZAP, POSTBURN, P40RCS, P40MINDB, TIGNOW,
 * POST41, P41/DSP, P47CSM, TIGON, P47BODY, P47BOD.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

#define DOTVCON_DELAY  40  /* DEC 40: 0.4 s for thrust build-up         */
#define DOSTRULL_DELAY 160 /* DEC 160: 0.4 + 1.6 = 2 s to ullage off    */
#define TAILOFF_DELAY  250 /* DEC 250: 2.5 s for SPS tail-off            */
#define TWO_SECS       200
#define CS_PER_SEC     100 /* 1SEC DEC 100                               */

/* FLAGWRD6 bits 15, 14 = 1, 0: the TVC DAP has T5.                         */
bool agc_csm_tvc_on(void)
{
    uint16_t w = agc_flagword_get(agc_flag_word(AGC_FLAG_DAPBIT1));

    if (!AGC_ASSERT(agc_flag_word(AGC_FLAG_DAPBIT1) == 6))
    {
        return (false);
    }

    return ((w & AGC_CSM_T5BITS) == AGC_CSM_T5_TVC);
}

/* PREPTVC (p.700): T5 bits reset, RCS DAP killed (T5LOC = T5IDLOC), 4.3,
 * DOTVCON in 0.4 s.                                                        */
agc_status_t agc_csm_preptvc(void)
{
    agc_flag_clear(AGC_FLAG_DAPBIT1); /* RESET T5 BITS          */
    agc_flag_clear(AGC_FLAG_DAPBIT2);
    agc_rupt_set_t5_task(AGC_TASK_T5IDLOC); /* KILL RCS               */
    agc_phase_newphase(4, 3, true);         /* 4.3 = DOTVCON (40CS)   */

    if (!AGC_ASSERT(!agc_csm_tvc_on()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_waitlist_fixdelay(DOTVCON_DELAY, AGC_TASK_DOTVCON));
}

/* IGNITION (p.700): OGAD (roll reference), TEVENT, ENGONFLG, SPS on; an
 * impulsive burn sets up the engine-off task, otherwise STRULLSW is set
 * for steering at DOSTRULL; then PREPTVC.                                 */
agc_status_t agc_task_ignition_csm(void)
{
    agc_csm_dap_com()->ogad = agc_erase_cdux(); /* ROLL DAP REF.    */
    agc_erase_set_tevent((agc_dp_t) agc_time_now_cs());
    agc_flag_set(AGC_FLAG_ENGONFLG);
    agc_chan_or(011, AGC_CSM_SPSON); /* SPSON                  */

    if (agc_flag_test(AGC_FLAG_IMPULSW))
    {                                             /* IMPULCHK               */
        agc_flag_clear(AGC_FLAG_STRULLSW); /* IMPLBURN               */
        agc_erase_set_tig(agc_erase_tgo() + (agc_dp_t) agc_time_now_cs());
        agc_phase_var_t v = {
            AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_IMPLCONT, 0, true, 0U
        };

        agc_phase_change2((uint16_t) 040153, (uint16_t) 007014, &v);
        agc_status_t st = agc_waitlist_call(
            (uint16_t) agc_dp_low(agc_erase_tgo()), AGC_TASK_ENGINOFF);

        return ((st != AGC_STATUS_OK) ? st : agc_task_implcont());
    }

    agc_flag_set(AGC_FLAG_STRULLSW); /* NON-IMPULSIVE          */

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_ENGONFLG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_csm_preptvc());
}

/* IMPLCONT (p.701): IMPULSW reset (ENGINOFF is set up), V97VCNTR = 0 in
 * case ENGINOFF comes before TVCDAPON, then PREPTVC.                      */
agc_status_t agc_task_implcont(void)
{
    agc_flag_clear(AGC_FLAG_IMPULSW);
    agc_csm_dap_tvc()->engine_fail_vcntr = 0;

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_IMPULSW)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_csm_preptvc());
}

/* DOTVCON (p.700-701): TVC takes T5: TVCPHASE = -1 (FRESHDAP), TVCEXPHS =
 * 0, T5 bits = 10, 6.3 (CLOKTASK), RCS errors copied for the TVC DAP,
 * 4.5, TIME5 = POSMAX and T5LOC = TVCDAPON, DOSTRULL in 1.6 s.            */
agc_status_t agc_task_dotvcon(void)
{
    agc_csm_tvc_t *tvc = agc_csm_dap_tvc();
    agc_csm_rcs_t *rcs = agc_csm_dap_rcs();

    if (!AGC_ASSERT(tvc != NULL && rcs != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    tvc->tvcphase = -1; /* CS BIT1                */
    tvc->tvcexphs = 0;
    agc_flag_set(AGC_FLAG_DAPBIT1); /* BITS 15,14 = 10        */
    agc_flag_clear(AGC_FLAG_DAPBIT2);
    agc_phase_newphase(6, 3, false); /* 6.3 = CLOKTASK         */
    tvc->errbtmp =
        agc_dp_neg(agc_dp_pack(rcs->error[1], rcs->error[2])); /* DCS ERRORY */
    agc_phase_newphase(4, 5, true);          /* 4.5 = DOSTRULL         */
    agc_erase_set_time5(AGC_SP_MAX);         /* START RIGHT AWAY       */
    agc_rupt_set_t5_task(AGC_TASK_TVCDAPON); /* KILLS RCS DAP          */

    if (!AGC_ASSERT(agc_csm_tvc_on()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_waitlist_fixdelay(DOSTRULL_DELAY, AGC_TASK_DOSTRULL));
}

/* DOSTRULL / STEERULL / ULAGEOFF (p.701): steering on for a non-impulsive
 * burn, ullage off (channel 5 = 0), group 4 off.                          */
agc_status_t agc_task_dostrull(void)
{
    if (agc_flag_test(AGC_FLAG_STRULLSW))
    {
        agc_flag_set(AGC_FLAG_STEERSW); /* STEERULL               */
    }

    agc_chan_write(05, 0U); /* ULAGEOFF: ZERO CHAN5   */

    if (!AGC_ASSERT(agc_chan_read(05) == 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_kill(4);

    return (AGC_STATUS_OK);
}

/* ENGINOFF (p.701-702): MASSTMP = CSMMASS (copy cycle for MASSBACK), group
 * 3 off and 4.63, then DOSPSOFF.                                          */
agc_status_t agc_task_enginoff_csm(void)
{
    agc_csm_dap_tvc()->masstmp = agc_erase_csmmass();
    agc_phase_change2((uint16_t) 000003, (uint16_t) 040634, NULL);

    if (!AGC_ASSERT(agc_csm_dap_tvc()->masstmp == agc_erase_csmmass()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_dospsoff());
}

/* DOSPSOFF (p.702): SPS shutdown, 4.27, DOTVCRCS after 2.5 s of tail-off. */
agc_status_t agc_task_dospsoff(void)
{
    agc_csm_spsoff();
    agc_phase_newphase(4, 027, true); /* 4.27 = DOTVCRCS        */

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_ENGONFLG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_waitlist_fixdelay(TAILOFF_DELAY, AGC_TASK_DOTVCRCS));
}

/* DOTVCRCS (p.702): wide deadband, RCS DAP on (kills the TVC DAPs), mass
 * properties, TVCZAP, 4.35 and the POSTBURN job (PRIO12).                 */
agc_status_t agc_task_dotvcrcs(void)
{
    agc_inhint();
    agc_csm_setmaxdb();
    agc_status_t st = agc_csm_dap_rcsdapon();

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
    agc_phase_change(4, (uint16_t) 000354); /* 4.35 = POSTBURN        */

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_TIMRFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_novac(AGC_PRIO12, AGC_JOB_POSTBURN));
}

/* MASSBACK (p.702-703): restore the part of the pre-decremented mass not
 * flowed: CSMMASS = MASSTMP + (V97VCNTR EMDOT, low word) * 100 cs (high
 * word); the trims follow DELPBAR/DELYBAR once the TVC DAP is running.   */
static void massback(void)
{
    agc_csm_tvc_t    *tvc = agc_csm_dap_tvc();
    agc_csm_dapcom_t *com = agc_csm_dap_com();
    int32_t           p   =
        (int32_t) tvc->engine_fail_vcntr * (int32_t) com->emdot; /* MP EMDOT */
    int32_t lo   = p % 16384;                                    /* LXCH A   */
    int32_t corr = (lo * CS_PER_SEC) / 16384;                    /* MP 1SEC  */

    agc_erase_set_csmmass((agc_sp_t) (tvc->masstmp + corr));

    if (tvc->tvcphase + 1 == 0 || !agc_csm_tvc_on())
    {
        return; /* BTRIMR                 */
    }

    com->pactoff = agc_dp_high(tvc->delpbar); /* BESTTRIM               */
    com->yactoff = agc_dp_high(tvc->delybar);

    if (!AGC_ASSERT(corr >= -CS_PER_SEC && corr <= CS_PER_SEC))
    {
        return;
    }
}

/* SPSOFF (p.702): TEVENT, ENGONFLG reset, SPS off, SIV cutoff, MASSBACK.  */
void agc_csm_spsoff(void)
{
    agc_erase_set_tevent((agc_dp_t) agc_time_now_cs());
    agc_flag_clear(AGC_FLAG_ENGONFLG); /* RESTARTS WILL SHUT DOWN */
    agc_chan_and(011, (uint16_t) ~AGC_CSM_SPSON);
    agc_chan_or(012, AGC_CSM_SIVCUTOFF); /* SIV CUTOFF COMMAND     */
    massback();

    if (!AGC_ASSERT((agc_chan_read(011) & AGC_CSM_SPSON) == 0U))
    {
        return;
    }
}

/* TVCZAP (p.703, under INHINT): TVC and optics error counters off, optics
 * DAC re-engaged, OPTIND = -1 (optics zero allowed), NVWORD1 = 0, TIMRFLAG
 * reset (stops CLOKTASK).                                                 */
void agc_csm_tvczap(void)
{
    agc_chan_and(012, (uint16_t) ~AGC_CSM_TVCOPT);
    agc_erase_set_optind(-1); /* CS BIT1                */
    agc_guid_state()->csm.nvword1 = 0;
    agc_flag_clear(AGC_FLAG_TIMRFLAG);

    if (!AGC_ASSERT(agc_guid_state()->csm.nvword1 == 0))
    {
        return;
    }
}

/* A flashing display: terminate -> POST41.                                 */
static agc_status_t flash_post41(agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                                 uint8_t verb, uint8_t noun, agc_job_id_t pro,
                                 agc_job_id_t enter)
{
    agc_dsky_cont_t cont = {AGC_JOB_POST41, pro, enter};

    if (!AGC_ASSERT(pro != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont));
}

/* POSTBURN (p.695): V16N40 final values; PROCEED -> P40RCS (trim with RCS).*/
agc_status_t agc_job_postburn_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (flash_post41(ctx, AGC_DSKY_REFLASH, 16, 40,
                         AGC_JOB_SPS_BURN_RCS_TRIM, AGC_JOB_POSTBURN));
}

/* P40RCS (p.695): AVEGEXIT = CALCN85, wait 2 s for it, then P40MINDB.     */
agc_status_t agc_job_sps_burn_rcs_trim(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_state()->avg.avgexit =
        AGC_JOB_CALC_VG_BODY; /* ACADN85                */

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_svc_delayjob(TWO_SECS, AGC_JOB_SPS_BURN_MIN_DEADBAND));
}

/* P40MINDB (p.695): narrow deadband, then TIGNOW.                         */
agc_status_t agc_job_sps_burn_min_deadband(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_csm_setmindb();
    agc_relint();

    return (agc_job_tignow_csm(ctx));
}

/* TIGNOW (p.695): 4 type C (PRIO20, here), V16N85 for an RCS trim.         */
agc_status_t agc_job_tignow_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_TIGNOW, AGC_TASK_NONE,
                         (int16_t) AGC_PRIO20, false, 0U};

    agc_phase_change_var((uint16_t) 005024, &v);

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (flash_post41(ctx, AGC_DSKY_REFLASH, 16, 85, AGC_JOB_POST41,
                         AGC_JOB_TIGNOW));
}

/* POST41 (p.696): AVEGEXIT = SERVEXIT, then P00.                           */
agc_status_t agc_job_post41(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_state()->avg.avgexit =
        AGC_JOB_SERVEXIT; /* SERVCADR               */

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_gotopooh());
}

/* P41/DSP (p.697): V06N85 by the CLOCKJOB (VGTIG in control axes), CALCN85
 * as AVEGEXIT, 6.3 and 4.23, then P40S/RS.                                */
agc_status_t agc_csm_rcs_burn_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t *g = agc_guid_state();

    g->csm.nvword1 = agc_csm_vn(6, 85); /* V06N85B                */
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_csm_convert_vg_body(&ip); /* VGTIG IN CTRL COORDS   */
    g->avg.avgexit = AGC_JOB_CALC_VG_BODY;
    agc_phase_change2((uint16_t) 040036, (uint16_t) 000234, NULL);

    return (agc_csm_preburn_start_clock(ctx));
}

/* P47CSM (p.698): IMU status, MIDTOAV2 (time to PREREAD < 1 s), TIGON.    */
agc_status_t agc_job_thrust_monitor_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t  *g  = agc_guid_state();
    agc_status_t sw = agc_imu_status_check();

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_dp_t     dt = 0;
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_status_t st = agc_guid_midtoav2(&ip, ctx->id, &dt);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g->csm.preread_delay =
        (agc_dp_t) agc_dp_low(dt); /* CA MPAC +1 TS P40TMP   */
    st = agc_waitlist_call((uint16_t) g->csm.preread_delay, AGC_TASK_TIGON);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change(4, (uint16_t) 040574);

    return (AGC_STATUS_OK); /* 4.57 = TIGON           */
}

/* TIGON (p.698): AVEGEXIT = CALCN83, P47BODY job, 4.2 and PRECHECK.       */
agc_status_t agc_task_tigon(void)
{
    agc_guid_state()->avg.avgexit = AGC_JOB_CALC_DELTA_V_BODY;
    agc_status_t st = agc_exec_novac(AGC_PRIO30, AGC_JOB_THRUST_MONITOR_RESET);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_newphase(4, 2, true); /* 4.2 = PRECHECK, P47BODY */

    return (agc_task_precheck());
}

/* P47BODY / P47BOD / P47/DSP (p.698-699): clear DELVIMU and DELVCTL, lower
 * the priority below CALCN83, 4 type C (PRIO15), V16N83 (ENTER: clear).    */
agc_status_t agc_job_thrust_monitor_reset_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_burn_t *b = &agc_guid_state()->burn;

    b->delvimu = agc_vec_zero();
    b->delvctl = agc_vec_zero();
    agc_exec_prio_change(AGC_PRIO15);

    return (agc_job_thrust_monitor_display_csm(ctx));
}

agc_status_t agc_job_thrust_monitor_display_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_THRUST_MONITOR_DISPLAY,
                         AGC_TASK_NONE, (int16_t) AGC_PRIO15,
                         false, 0U};

    agc_phase_change_var((uint16_t) 005024, &v);
    agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH, AGC_JOB_GOTOPOOH,
                            AGC_JOB_THRUST_MONITOR_RESET};

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(16, 83),
                             &cont));
}
