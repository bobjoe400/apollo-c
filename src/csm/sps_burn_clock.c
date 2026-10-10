/*
 * sps_burn_clock.c -- P40-P47.agc, Comanche055 pp. 704-709 and 729-734:
 * the once-a-second CLOKTASK/CLOCKJOB display (TTOGO, V06N40, V99 engine
 * enable request, V97 engine-fail), DYNDISP (dynamic N85 in P41), the
 * V99 and V97 responses (V99E/T/P, V97T/P/E and their tasks, QUICKIGN,
 * V99FLASH), S40.6 (gimbal drive test and trim, PRE40.6) and DAPFIG (V46).
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "agc/agc_guid.h"
#include "sps_burn_internal.h"

#define ONE_SEC        100
#define TAILOFF_DELAY  250
#define ENGINE_FAIL_DELAY      200
#define QUICKIGN_DELAY 30
#define GIMBAL_STEP    200   /* 2 s per gimbal drive step (GPI check)      */
#define GIMTRIM_DELAY  400   /* 4 s before the trim                       */
#define ACT_P2DEG      84    /* +2ACTDEG DEC +84: 2 deg at 85.41"/bit      */
#define ACT_M4DEG      (-168)/* -4ACTDEG                                   */

/* CLOKTASK (p.704-705): with TIMRFLAG, TTOGO = TIME - TIG every second and
 * the CLOCKJOB (or DYNDISP for V06N85) run when NVWORD1 is non-zero;
 * without it, group 6 off and stop.                                       */
agc_status_t agc_task_cloktask_csm(void)
{
    agc_guid_t *g = agc_guid_state();
    if (!agc_flag_test(AGC_FLAG_TIMRFLAG)) {
        return agc_phase_kill(6);                     /* KILL RESTART           */
    }
    agc_erase_set_ttogo((agc_dp_t)agc_time_now_cs() - agc_erase_tig()); /* CLOCKON */
    agc_status_t st = agc_waitlist_call(ONE_SEC, AGC_TASK_CLOKTASK);    /* SETCLOCK */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (g->csm.nvword1 == agc_csm_vn(6, 85)) {
        (void)agc_exec_findvac(AGC_PRIO27, AGC_JOB_DYNDISP);   /* SETUPDYN      */
    } else if (g->csm.nvword1 != 0) {
        (void)agc_exec_novac(AGC_PRIO27, AGC_JOB_CLOCKJOB);
    } else {
        (void)AGC_ASSERT(g->csm.nvword1 == 0);
    }
    return agc_phase_set_tbase(6);                    /* SETTB6                 */
}

/* CKNVWRD1 (p.705-706): +VN: show NVWORD1 (REGODSP); V99 flash: engine on
 * enable request; V97 flash: engine failed. The flashing paste is passed
 * as the verb of the CLOCPLAY VN (the AGC pastes it from NVWORD1).        */
static agc_status_t cknvwrd1(agc_job_ctx_t *ctx)
{
    agc_sp_t nv = agc_guid_state()->csm.nvword1;
    agc_relint();
    if (nv > 0) {
        return agc_dsky_display(ctx, AGC_DSKY_REGODSP, (uint16_t)nv, NULL); /* NOFLASH */
    }
    if (nv == 0) {
        return AGC_STATUS_OK;                         /* ENDOFJOB               */
    }
    if (nv == AGC_CSM_NV_ENGINE_FAIL_PROCEDURE) {                       /* FAILDSP                */
        agc_dsky_cont_t c97 = { AGC_JOB_ENGINE_FAIL_TERMINATE, AGC_JOB_ENGINE_FAIL_PROCEED, AGC_JOB_ENGINE_FAIL_REIGNITE };
        return agc_dsky_display(ctx, AGC_DSKY_CLOCPLAY, agc_dsky_vn(97, 40), &c97);
    }
    agc_dsky_cont_t c99 = { AGC_JOB_ENGINE_ENABLE_TERMINATE, AGC_JOB_ENGINE_ENABLE_PROCEED, AGC_JOB_ENGINE_ENABLE_BYPASS };   /* ENGREQST */
    (void)AGC_ASSERT(nv == AGC_CSM_NV_ENGINE_ENABLE_REQUEST);
    return agc_dsky_display(ctx, AGC_DSKY_CLOCPLAY, agc_dsky_vn(99, 40), &c99);
}

/* CLOCKJOB (p.705): fresh CDU trig for the displays, then CKNVWRD1.       */
agc_status_t agc_job_clockjob(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    const uint16_t cdu[3] = { agc_erase_cduy(), agc_erase_cduz(), agc_erase_cdux() };
    (void)agc_guid_quictrig(&g->trig, cdu);
    agc_inhint();
    return cknvwrd1(ctx);
}

/* DYNDISP (p.705): update N85 for the dynamic V06N85 of P41.               */
agc_status_t agc_job_dyndisp(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    (void)agc_csm_convert_vg_body(&ip);
    agc_inhint();
    return cknvwrd1(ctx);
}

/* V99E / V99T (p.706): 6 off, 4 type C at PRIO27; TVCZAP; then P40RCS
 * (V16N85 post-burn) or POST41 (P00).                                     */
static agc_status_t engine_enable_finish(agc_job_id_t next)
{
    agc_phase_var_t v = { AGC_RESTART_JOB, next, AGC_TASK_NONE, (int16_t)AGC_PRIO27,
                          false, 0U };
    (void)agc_phase_change2((uint16_t)000006, (uint16_t)005024, &v);
    agc_inhint();
    (void)agc_csm_tvczap();                           /* WIPE OUT TVC, CLOKTASK */
    agc_relint();
    (void)AGC_ASSERT(!agc_flag_test(AGC_FLAG_TIMRFLAG));
    return agc_exec_job_yield(next);
}

agc_status_t agc_job_engine_enable_bypass(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return engine_enable_finish(AGC_JOB_SPS_BURN_RCS_TRIM);
}

agc_status_t agc_job_engine_enable_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return engine_enable_finish(AGC_JOB_POST41);
}

/* V99P (p.706-707): engine enable. If TIG-0 has come (IGNFLAG) ignite in
 * 10 ms; in any case clear the V99 (V06N40). A second V99P (restart) only
 * clears it.                                                              */
agc_status_t agc_job_engine_enable_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    if (!agc_flag_test(AGC_FLAG_ASTNFLAG)) {          /* ASTNV99P               */
        (void)agc_flag_set(AGC_FLAG_ASTNFLAG);
        if (agc_flag_test(AGC_FLAG_IGNFLAG)) {        /* ENDV99PI               */
            (void)agc_waitlist_call(1, AGC_TASK_IGNITION);
        }
    }
    agc_guid_state()->csm.nvword1 = agc_csm_vn(6, 40);  /* V99P/TIG             */
    agc_relint();
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_ASTNFLAG));
    return AGC_STATUS_OK;
}

/* V97T (p.707): engine failed, terminate: 6 off, 4.67 V97TTASK now.       */
agc_status_t agc_job_engine_fail_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_phase_change2((uint16_t)000006, (uint16_t)040674, NULL);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return agc_waitlist_call(1, AGC_TASK_ENGINE_FAIL_SHUTDOWN);   /* TWIDDLE                */
}

/* V97TTASK (p.707): CLOCKJOB off, MASSTMP = CSMMASS + 3 s of flow (not lost
 * because the thrust failed), SPSOFF, 4.71, V97TRCS after 2.5 s.          */
agc_status_t agc_task_engine_fail_shutdown(void)
{
    agc_guid_state()->csm.nvword1 = 0;                /* DISABLE CLOCKJOB       */
    agc_csm_dap_tvc()->masstmp = (agc_sp_t)(AGC_CSM_3MDOT + agc_erase_csmmass());
    agc_phase_var_t v = { AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_ENGINE_FAIL_SHUTDOWN_RESTART, 0,
                          true, 0U };
    (void)agc_phase_change_var((uint16_t)005014, &v);
    (void)AGC_ASSERT(agc_guid_state()->csm.nvword1 == 0);
    return agc_task_engine_fail_shutdown_restart();
}

agc_status_t agc_task_engine_fail_shutdown_restart(void)
{
    (void)agc_csm_spsoff();
    (void)agc_phase_change(4, (uint16_t)000714);      /* 4.71 = V97TRCS         */
    (void)AGC_ASSERT(!agc_flag_test(AGC_FLAG_ENGONFLG));
    return agc_waitlist_fixdelay(TAILOFF_DELAY, AGC_TASK_ENGINE_FAIL_RCS_DAP);
}

/* V97TRCS (p.707): RCS DAP on in 0.6 s, V99T (TVCZAP, POST41).            */
agc_status_t agc_task_engine_fail_rcs_dap(void)
{
    agc_inhint();
    (void)agc_csm_dap_rcsdapon();
    agc_relint();
    (void)AGC_ASSERT(agc_guid_state()->csm.nvword1 == 0);
    return agc_exec_novac(AGC_PRIO27, AGC_JOB_ENGINE_ENABLE_TERMINATE);
}

/* V97P (p.707): proceed: 4.73 V97PTASK now.                               */
agc_status_t agc_job_engine_fail_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_phase_change(4, (uint16_t)040734);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return agc_waitlist_call(1, AGC_TASK_ENGINE_FAIL_RESUME);
}

/* V97PTASK (p.707-708): mass updates again, V06N40, IDLEFAIL (one R40
 * bypass) and STEERSW set, 4.13, R40ENABL in 2 s.                         */
agc_status_t agc_task_engine_fail_resume(void)
{
    agc_csm_tvc_t *tvc = agc_csm_dap_tvc();
    tvc->vcntr = tvc->engine_fail_vcntr;
    agc_guid_state()->csm.nvword1 = agc_csm_vn(6, 40);
    (void)agc_flag_set(AGC_FLAG_IDLEFAIL);
    (void)agc_flag_set(AGC_FLAG_STEERSW);
    (void)agc_phase_change(4, (uint16_t)000134);      /* 4.13 = R40ENABL        */
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_STEERSW));
    return agc_waitlist_fixdelay(ENGINE_FAIL_DELAY, AGC_TASK_ENGINE_FAIL_REENABLE);
}

/* R40ENABL (p.708): IDLEFAIL reset (R40 enabled again), group 4 off.      */
agc_status_t agc_task_engine_fail_reenable(void)
{
    (void)agc_flag_clear(AGC_FLAG_IDLEFAIL);
    (void)AGC_ASSERT(!agc_flag_test(AGC_FLAG_IDLEFAIL));
    return agc_phase_kill(4);
}

/* V97E (p.708): re-ignite: 4.53 V97ETASK in 10 ms.                         */
agc_status_t agc_job_engine_fail_reignite(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_phase_change(4, (uint16_t)040534);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return agc_waitlist_call(1, AGC_TASK_ENGINE_FAIL_REIGNITE_PREP);
}

/* V97ETASK / SPSOFF97 (p.708): R1 of V06N40 forced to 59x59 (TIG high word
 * = -OCT24), V06N40, MASSTMP, 4.75, SPSOFF, 4.11, V97E40.6 after 2.5 s.   */
agc_status_t agc_task_engine_fail_reignite_prep(void)
{
    agc_erase_set_tig(agc_dp_pack((agc_sp_t)-024, 0));    /* CS OCT24 TS TIG   */
    agc_guid_state()->csm.nvword1 = agc_csm_vn(6, 40);
    agc_csm_dap_tvc()->masstmp = (agc_sp_t)(AGC_CSM_3MDOT + agc_erase_csmmass());
    (void)agc_phase_change(4, (uint16_t)000754);      /* 4.75 = SPSOFF97        */
    (void)AGC_ASSERT(agc_erase_tig() < 0);
    return agc_task_spsoff97();
}

agc_status_t agc_task_spsoff97(void)
{
    (void)agc_csm_spsoff();
    (void)agc_phase_change(4, (uint16_t)000114);      /* 4.11 = V97E40.6        */
    (void)AGC_ASSERT(!agc_flag_test(AGC_FLAG_ENGONFLG));
    return agc_waitlist_fixdelay(TAILOFF_DELAY, AGC_TASK_ENGINE_FAIL_REIGNITE_TRIM);
}

/* V97E40.6 / QUICKIGN (p.708-709): PRE40.6 (trim) now, RCS DAP on with the
 * narrow deadband left, 6.2 and a type C task; ASTNFLAG reset and IGNFLAG
 * set for an immediate V99 response; V99FLASH in 0.3 s.                    */
agc_status_t agc_task_engine_fail_reignite_trim(void)
{
    (void)agc_waitlist_call(1, AGC_TASK_GIMBAL_TRIM_RESTART);
    agc_inhint();
    (void)agc_csm_dap_rcsdapon();
    agc_relint();
    agc_phase_var_t v = { AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_QUICKIGN, 0, true, 0U };
    (void)agc_phase_change2((uint16_t)000026, (uint16_t)005014, &v);
    (void)AGC_ASSERT(v.task == AGC_TASK_QUICKIGN);
    return agc_task_quickign();
}

agc_status_t agc_task_quickign(void)
{
    (void)agc_flag_clear(AGC_FLAG_ASTNFLAG);          /* CS PRIO14 MASK         */
    (void)agc_flag_set(AGC_FLAG_IGNFLAG);             /* AD BIT13               */
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_IGNFLAG));
    return agc_waitlist_fixdelay(QUICKIGN_DELAY, AGC_TASK_ENGINE_ENABLE_FLASH);
}

/* V99FLASH (p.709): V99 flash, 4.77 (TIG-0 now) and 3.3, S40.13 job.      */
agc_status_t agc_task_engine_enable_flash(void)
{
    agc_guid_state()->csm.nvword1 = AGC_CSM_NV_ENGINE_ENABLE_REQUEST;
    (void)agc_phase_change2((uint16_t)040774, (uint16_t)000033, NULL);
    (void)AGC_ASSERT(agc_guid_state()->csm.nvword1 < 0);
    return agc_exec_findvac(AGC_PRIO20, AGC_JOB_TIMEBURN);
}

/* PRE40.6 (p.730): restart entry of S40.6, not while TVC has T5 (REDOTVC
 * restarts the actuators then): trim only, CNTR = +1 (restart entry).     */
agc_status_t agc_task_gimbal_trim_restart(void)
{
    if (agc_csm_tvc_on()) {
        return AGC_STATUS_OK;                         /* TVC: TASKOVER          */
    }
    agc_guid_state()->csm.mrkrtmp = -1;
    agc_csm_dap_tvc()->cntr = 1;
    (void)AGC_ASSERT(agc_csm_dap_tvc()->cntr == 1);
    return agc_task_gimbal_test_trim();
}

/* S40.6 (p.730): optics inhibited, optics error counters disabled, TVC
 * enable and optics DAC disengaged (channel 12 bits 8, 11); 60 ms relay
 * latching.                                                               */
agc_status_t agc_task_gimbal_test_trim(void)
{
    agc_erase_set_optind(-1);                         /* CS ZERO (-0 modelled -1) */
    (void)agc_chan_and(012, (uint16_t)~(uint16_t)02); /* WAND CHAN12 ~BIT2      */
    (void)agc_chan_or(012, AGC_CSM_TVCOPTEN);         /* OCT02200               */
    (void)AGC_ASSERT((agc_chan_read(012) & AGC_CSM_TVCOPTEN) == AGC_CSM_TVCOPTEN);
    return agc_waitlist_fixdelay(6, AGC_TASK_GIMBAL_TEST_RELAYS);
}

/* S40.6 after 60 ms (p.730): optics error counters enabled, 20 ms.        */
agc_status_t agc_task_gimbal_test_relays(void)
{
    (void)agc_chan_or(012, (uint16_t)02);             /* WOR CHAN12 BIT2        */
    (void)AGC_ASSERT((agc_chan_read(012) & 02U) != 0U);
    return agc_waitlist_fixdelay(2, AGC_TASK_GIMBAL_TEST_RSTRTST);
}

/* OUTPUT (p.731): one gimbal increment into TVCPITCH (step < 3) or TVCYAW,
 * counter released (channel 14 bit 11 or 12), 2 s for the GPI check.     */
static agc_status_t gimbal_output(int step)
{
    static const agc_sp_t inc[3] = { ACT_P2DEG, ACT_M4DEG, ACT_P2DEG };
    bool yaw = (step >= 3);
    if (yaw) {
        agc_erase_set_tvcyaw(inc[step - 3]);
    } else {
        agc_erase_set_tvcpitch(inc[step]);
    }
    (void)agc_chan_or(014, yaw ? (uint16_t)04000 : (uint16_t)02000);
    (void)AGC_ASSERT(step >= 0 && step < 6);
    return agc_waitlist_fixdelay(GIMBAL_STEP, AGC_TASK_GIMBAL_TEST_STEP);
}

/* RSTRTST (p.730-731): restart entry (CNTR > 0) trims at once; trim only
 * waits 4 s; else the drive test: +2, -4, +2 degrees in pitch, then yaw.  */
agc_status_t agc_task_gimbal_test_rstrtst(void)
{
    agc_guid_t *g = agc_guid_state();
    agc_csm_tvc_t *tvc = agc_csm_dap_tvc();
    if (tvc->cntr > 0) {
        g->csm.gimbal_test_step = 7;                          /* GIMTRIM +2             */
        return agc_task_gimbal_test_step();
    }
    tvc->cntr = g->csm.mrkrtmp;                       /* SAVES CNTR FOR RESTARTS */
    if (g->csm.mrkrtmp <= 0) {
        g->csm.gimbal_test_step = 7;                          /* GIMTRIM: WAIT 4 S      */
        return agc_waitlist_fixdelay(GIMTRIM_DELAY, AGC_TASK_GIMBAL_TEST_STEP);
    }
    g->csm.gimbal_test_step = 0;                              /* GDTSETUP: PITCH        */
    (void)AGC_ASSERT(g->csm.mrkrtmp > 0);
    return gimbal_output(0);
}

/* The return point of OUTPUT / GIMTRIM (TEMPR60): next drive step, or
 * after the six steps the 4 s wait, then the trims PACTOFF/YACTOFF out and
 * the counters released (channel 14 bits 11, 12: PRIO6).                  */
agc_status_t agc_task_gimbal_test_step(void)
{
    agc_guid_t *g = agc_guid_state();
    if (g->csm.gimbal_test_step < 5) {
        g->csm.gimbal_test_step++;
        return gimbal_output(g->csm.gimbal_test_step);
    }
    if (g->csm.gimbal_test_step == 5) {
        g->csm.gimbal_test_step = 7;                          /* GIMTRIM                */
        return agc_waitlist_fixdelay(GIMTRIM_DELAY, AGC_TASK_GIMBAL_TEST_STEP);
    }
    const agc_csm_dapcom_t *com = agc_csm_dap_com();
    agc_erase_set_tvcpitch(com->pactoff);             /* CS ZERO AD PACTOFF     */
    agc_erase_set_tvcyaw(com->yactoff);
    (void)AGC_ASSERT(g->csm.gimbal_test_step == 7);
    return agc_chan_or(014, (uint16_t)06000);         /* RELEASE THE COUNTERS   */
}

/* DAPFIG (Comanche p.734, from V46 STABLISH): SIVB takeover off; by the
 * configuration in DAPDATR1 bits 14-13: CM alone -> no DAP (T5 idle, jets
 * off, KALCMANU killed), CSM or CSM/LM -> RCS DAP, Saturn -> SATSTKON;
 * exit through GOPIN.                                                     */
agc_status_t agc_job_dapfig(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_chan_and(012, (uint16_t)~(uint16_t)AGC_CH012_SIVBTAKEOVER);
    unsigned cfg = ((unsigned)agc_erase_dapdatr1() >> 12) & 3U;   /* MP BIT3     */
    if (cfg == 3U) {
        return agc_csm_satstkon(ctx);
    }
    agc_inhint();
    if (cfg == 0U) {                                  /* NODAPUP                */
        (void)agc_rupt_set_t5_task(AGC_TASK_T5IDLOC);
        (void)agc_flag_clear(AGC_FLAG_DAPBIT1);
        (void)agc_flag_clear(AGC_FLAG_DAPBIT2);
        (void)agc_csm_dap_zerojet();
        agc_csm_dap_rcs()->holdflag = 1;              /* KILL KALCMANU JOB      */
    } else {
        (void)agc_csm_dap_rcsdapon();                 /* RCSDAPUP               */
    }
    agc_relint();
    return agc_dsky_gopin(ctx);                       /* ENDFIG                 */
}
