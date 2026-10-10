/*
 * clock.c -- BURN, BABY, BURN (Luminary099 pp. 742-748): the countdown
 * clock (STCLOK1/2/3, CLOKTASK, CLOKJOB and its displays), STOPCLOK,
 * NULLCLOK, the crew responses to the V99 engine-enable request (*PROCEED,
 * *ENTER, GOPOST, GOCUTOFF, IGNITE), to the V97 engine-fail display
 * (COMFAIL, COMFAIL1-4) and to V06N61 (ASTNRETN), REP40ALM, and P40AUTO.
 * The ignition tables and the TIG chain are in burnbaby.c.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

#define ONE_SEC      100 /* 1SEC                                       */
#define LOW5         037
#define DVMON_PASSES 4 /* COMFAIL: CAF FOUR / TS DVCNTR              */

/* STCLOK3 (p.743-744): DT = TIG - TIME (positive: we passed the 45 s
 * check); CLOKTASK in (DT mod 1 s) + 20 ms so that it runs just after each
 * whole second before TIG. The AGC divides only the low 5 bits of the
 * upper word with the lower word (DT mod 2^19 cs, i.e. 87 min) by 1SEC;
 * that is kept. 6.3 (CLOKTASK) and 4.2 type C (PRIO13) at the caller.   */
agc_status_t ign_start_clock(agc_job_id_t restart_job)
{
    agc_dp_t dt = agc_erase_tig() - (agc_dp_t) agc_time_now_cs(); /* TPAGREE */
    (void) AGC_ASSERT(dt > 0);
    int32_t hi  = (int32_t) agc_dp_high(dt) & LOW5; /* MASK LOW5              */
    int32_t lo  = (int32_t) agc_dp_low(dt);
    int32_t rem = ((hi << AGC_SP_BITS) + lo) % ONE_SEC; /* DV, CA L         */
    agc_inhint();
    (void) agc_waitlist_call((uint16_t) (rem + 2), AGC_TASK_CLOKTASK);
    agc_phase_var_t v = {AGC_RESTART_JOB,      restart_job, AGC_TASK_NONE,
                         (int16_t) AGC_PRIO13, false,       0U};
    agc_status_t    st =
        agc_phase_change2((uint16_t) 040036, (uint16_t) 005024, &v);
    agc_relint();
    (void) AGC_ASSERT(rem >= 0 && rem < ONE_SEC);
    return st;
}

/* STCLOK2 (p.743): DISPDEX = dispdex, STCLOK3, return to ret_job.         */
agc_status_t agc_lm_stclok2(const agc_job_ctx_t *ctx, agc_sp_t dispdex,
                            agc_job_id_t ret_job)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(dispdex <= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_state()->lm.dispdex = dispdex;
    (void) ign_start_clock(ret_job);
    return agc_exec_job_yield(ret_job); /* CA TBASE4 / BANKJUMP   */
}

/* STCLOK1 (p.743): DISPDEX = 0 (clock running, nothing displayed), then
 * STCLOK3. Only COMFAIL calls it, so COMFAIL is the 4.2 restart point.
 * DEVIATION: the AGC restarts after the call (COMFAIL +3); restarting
 * COMFAIL from the top repeats only idempotent flag settings.             */
agc_status_t agc_lm_stclok1(void)
{
    agc_guid_state()->lm.dispdex = 0;
    (void) AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM);
    return ign_start_clock(AGC_JOB_COMFAIL);
}

/* CLOKTASK (p.744): TBASE6 for the 6.3 restart; DISPDEX positive kills the
 * clock (KILLCLOK: group 6 off); otherwise CLOKJOB at PRIO27 and again in
 * one second.                                                             */
agc_status_t agc_task_cloktask_lm(void)
{
    (void) agc_phase_set_tbase(6);
    if (agc_guid_state()->lm.dispdex > 0)
    {
        return agc_phase_kill(6); /* KILLCLOK               */
    }
    (void) agc_exec_novac(AGC_PRIO27, AGC_JOB_CLOCKJOB);
    (void) AGC_ASSERT(agc_guid_state()->lm.dispdex <= 0);
    return agc_waitlist_fixdelay(ONE_SEC, AGC_TASK_CLOKTASK);
}

/* -13 (p.745): V99Nxx "PLEASE ENABLE ENGINE", flashing (V99 pasted by
 * NVWORD1 = -BIT9; here the paste is the verb of the CLOCPLAY VN, as in
 * the CSM clock). Also V99RECYC, the P63 *ENTER.                          */
static agc_status_t engine_enable_display(const agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t c99 = {AGC_JOB_STOPCLOK,
                                 AGC_JOB_CLOKJOB_ENGINE_ENABLE,
                                 AGC_JOB_CLOKJOB_ENGINE_ENABLE_BYPASS};
    (void) agc_dsky_set_nvword1((uint16_t) (077777U & ~0400U)); /* CS BIT9    */
    (void) AGC_ASSERT(ign_table()->noun != 0U);
    return agc_dsky_display(ctx, AGC_DSKY_CLOCPLAY,
                            agc_dsky_vn(99, ign_table()->noun), &c99);
}

/* -35 (p.745): V97Nxx engine fail (NVWORD1 = -0 pastes V97).
 * DEVIATION: the AGC shows NVWORD +2 (the last V06 noun); the table's
 * countdown noun is the same noun in every case that reaches COMFAIL.     */
static agc_status_t engine_fail_display(const agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t c97 = {AGC_JOB_STOPCLOK, AGC_JOB_CLOKJOB_ENGINE_FAIL,
                                 AGC_JOB_CLOKJOB_ENGINE_FAIL_SHUTDOWN};
    (void) agc_dsky_set_nvword1(
        (uint16_t) 077777U); /* CS ZERO                */
    return agc_dsky_display(ctx, AGC_DSKY_CLOCPLAY,
                            agc_dsky_vn(97, ign_table()->noun), &c97);
}

/* -25 (p.745): V06N61 by ASTNCLOK, used by the crew in P63 to set the
 * event timer to TIG. ENTER (TCF -6) shows it again.
 * DEVIATION: ENTER restarts CLOKJOB, which recomputes TTOGO first.       */
static agc_status_t event_timer_display(const agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t c61 = {AGC_JOB_STOPCLOK,
                                 AGC_JOB_CLOKJOB_EVENT_TIMER_PROCEED,
                                 AGC_JOB_CLOCKJOB};
    (void) AGC_ASSERT(agc_guid_state()->lm.dispdex == -IGN_ASTNDEX);
    return agc_dsky_display(ctx, AGC_DSKY_REFLASH, agc_dsky_vn(6, 61), &c61);
}

/* CLOKJOB (p.744-746): TTOGO = TIME - TIG; DISPDEX >= 0: no display (a
 * task or higher-priority job changed it); otherwise the display -DISPDEX
 * selects: 35 V97, 25 V06N61, 17 countdown V06Nxx, 13 V99, 2 blank.     */
agc_status_t agc_job_clokjob(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_erase_set_ttogo((agc_dp_t) agc_time_now_cs() - agc_erase_tig());
    agc_inhint();
    agc_sp_t dispdex = agc_guid_state()->lm.dispdex;
    agc_relint();
    switch (-dispdex)
    {
        case IGN_VB97DEX:
            return engine_fail_display(ctx);
        case IGN_ASTNDEX:
            return event_timer_display(ctx);
        case IGN_CNTDNDEX: /* ONE-SECOND COUNTDOWN   */
            return agc_dsky_display(ctx, AGC_DSKY_REGODSP,
                                    agc_dsky_vn(6, ign_table()->noun), NULL);
        case IGN_VB99DEX:
            return engine_enable_display(ctx);
        case IGN_BLANKDEX: /* AVERAGE G IS STARTING  */
            return agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0U, NULL);
        default:
            (void) AGC_ASSERT(dispdex >= 0); /* DISPNOT / ENDOFJOB     */
            return AGC_STATUS_OK;
    }
}

/* NULLCLOK (p.746): INHINT; ullage off and ULLGTASK removed, even after a
 * restart (group 1 off); CLOKTASK killed. Returns with interrupts
 * inhibited, as in the AGC: the caller relints.                          */
agc_status_t agc_lm_nullclok(void)
{
    agc_inhint();
    (void) agc_lm_noullage();
    (void) agc_waitlist_kill(AGC_TASK_ULLGTASK);
    (void) agc_phase_change(1, (uint16_t) 000001);
    agc_guid_state()->lm.dispdex = IGN_DISPDEX_KILL;
    (void) AGC_ASSERT(!agc_flag_test(AGC_FLAG_ULLAGFLG));
    return AGC_STATUS_OK;
}

/* STOPCLOK (p.746): V34 on any clock display: NULLCLOK, then P00.        */
agc_status_t agc_job_stopclok(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    (void) agc_lm_nullclok();
    agc_relint(); /* GOTOPOOH RELINTS       */
    return agc_exec_gotopooh();
}

/* ASTNRETN (p.746): V06N61 PROCEED: 4.2 type C here, stop displaying but
 * keep the clock running, ASTNRET (P63) at PRIO13.                       */
agc_status_t agc_job_clokjob_event_timer_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_CLOKJOB_EVENT_TIMER_PROCEED,
                         AGC_TASK_NONE,   (int16_t) ctx->prio,
                         false,           0U};
    (void) agc_phase_change_var((uint16_t) 004024, &v);
    agc_guid_state()->lm.dispdex = 0;
    return agc_exec_findvac(AGC_PRIO13, AGC_JOB_ASTNRET);
}

/* *PROCEED / IGNITE (p.746-747): ASTNFLAG set; if TIG-0 has come
 * (IGNFLAG) IGNITION in 10 ms with 4.23 (immediate restart at IGNITION);
 * IGNITE1: the countdown display comes back.                              */
agc_status_t agc_job_clokjob_engine_enable(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    (void) agc_flag_set(AGC_FLAG_ASTNFLAG);
    if (agc_flag_test(AGC_FLAG_IGNFLAG))
    {
        agc_inhint();
        (void) agc_waitlist_call(1, AGC_TASK_IGNITION);
        (void) agc_phase_newphase(4, (uint16_t) 023, false);
        agc_relint();
    }
    agc_guid_state()->lm.dispdex = (agc_sp_t) -IGN_CNTDNDEX; /* IGNITE1     */
    (void) AGC_ASSERT(agc_flag_test(AGC_FLAG_ASTNFLAG));
    return AGC_STATUS_OK;
}

/* GOPOST (p.746-747): POSTBURN at PRIO12 (below CLOKJOB), DAP set up for
 * coasting flight (ALLCOAST), NULLCLOK, 4.13 (POSTBURN).                 */
static agc_status_t go_postburn(void)
{
    (void) agc_exec_findvac(AGC_PRIO12, AGC_JOB_POSTBURN);
    agc_inhint();
    (void) agc_lm_dap_allcoast();
    (void) agc_lm_nullclok();
    agc_status_t st = agc_phase_change(4, (uint16_t) 000134);
    agc_relint();
    (void) AGC_ASSERT(agc_guid_state()->lm.dispdex > 0);
    return st;
}

/* GOCUTOFF (p.747): CUTOFF at PRIO17, guidance displays allowed, coasting
 * DAP, NULLCLOK, 4.2 type C restarting CUTOFF at PRIO17 (FINDVAC).       */
static agc_status_t go_cutoff(void)
{
    (void) agc_exec_findvac(AGC_PRIO17, AGC_JOB_CUTOFF);
    (void) agc_flag_clear(AGC_FLAG_FLUNDISP);
    agc_inhint();
    (void) agc_lm_dap_allcoast();
    (void) agc_lm_nullclok();
    agc_phase_var_t v  = {AGC_RESTART_JOB,      AGC_JOB_CUTOFF, AGC_TASK_NONE,
                          (int16_t) AGC_PRIO17, false,          0U};
    agc_status_t    st = agc_phase_change_var((uint16_t) 007024, &v);
    agc_relint();
    (void) AGC_ASSERT(!agc_flag_test(AGC_FLAG_FLUNDISP));
    return st;
}

/* *ENTER (p.746): V99 answered ENTER: entry (3) -- GOPOST (P40, P42),
 * GOCUTOFF (P12, aborts) or V99RECYC (P63: ask again).                    */
agc_status_t agc_job_clokjob_engine_enable_bypass(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    switch (ign_table()->enter)
    {
        case IGN_ENTER_GOPOST:
            return go_postburn();
        case IGN_ENTER_GOCUTOFF:
            return go_cutoff();
        case IGN_ENTER_RECYCLE:
            return engine_enable_display(ctx);
        default:
            (void) AGC_ASSERT(false);
            return AGC_STATUS_BAD_ARG;
    }
}

/* COMFAIL (p.742), (15): no thrust after ignition (DVMON): DV monitor off
 * (IDLEFLAG), guidance displays suppressed (FLUNDISP), DVCNTR = 4, the
 * clock started if it is not running (STCLOK1), V97 display, group 4 off. */
agc_status_t agc_job_comfail(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    (void) agc_flag_set(AGC_FLAG_IDLEFLAG);
    (void) agc_flag_set(AGC_FLAG_FLUNDISP);
    g->lm.dvcntr = DVMON_PASSES; /* RESET DVMON            */
    if (agc_phase_get(6) == 0U)
    { /* CLOCKTASK ACTIVE?      */
        (void) agc_lm_stclok1();
    }
    g->lm.dispdex = (agc_sp_t) -IGN_VB97DEX;
    (void) AGC_ASSERT(g->lm.dispdex < 0);
    return agc_phase_change(4, (uint16_t) 000004); /* TURN OFF GROUP 4       */
}

/* COMFAIL1 (p.742): V97 PROCEED (keep going without thrust): entry (2) --
 * COMFAIL3 kills the clock (P12, P63, aborts), COMFAIL4 brings back the
 * countdown (P40, P42); DV monitor and guidance displays reconnected.    */
agc_status_t agc_job_clokjob_engine_fail(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_lm_t *lm = &agc_guid_state()->lm;
    lm->dispdex = ign_table()->comfail_kill ? (agc_sp_t) IGN_DISPDEX_KILL :
                                              (agc_sp_t) -IGN_CNTDNDEX;
    (void) agc_flag_clear(AGC_FLAG_IDLEFLAG); /* RECONNECT DV MONITOR   */
    (void) agc_flag_clear(AGC_FLAG_FLUNDISP);
    (void) AGC_ASSERT(lm->dispdex != 0);
    return AGC_STATUS_OK;
}

/* COMFAIL2 (p.742-743): V97 ENTER (shut down and try again): ZOOM restart
 * protection off (group 3), ZOOM removed if still to come, engine off
 * (ENGINOF4), DAP drift bit, the other RCS system (AORBTFLG inverted),
 * ullage on, TIG-5 in 10 ms (a new V99).
 * KILLTASK names ZOOM (= P40ZOOMA), so it removes only a ZOOM scheduled by
 * the 3.3 restart, not P40ZOOM or P63ZOOM (Luminary 099 as flown).       */
agc_status_t agc_job_clokjob_engine_fail_shutdown(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    (void) agc_phase_change(3, (uint16_t) 000003);
    agc_inhint();
    (void) agc_waitlist_kill(AGC_TASK_ZOOM);
    (void) agc_lm_enginof4();
    (void) agc_flag_set(AGC_FLAG_DRIFTDFL);
    if (agc_flag_test(AGC_FLAG_AORBTFLG))
    { /* INVFLAG                */
        (void) agc_flag_clear(AGC_FLAG_AORBTFLG);
    }
    else
    {
        (void) agc_flag_set(AGC_FLAG_AORBTFLG);
    }
    (void) agc_flag_set(AGC_FLAG_ULLAGFLG); /* TURN ON ULLAGE         */
    agc_status_t st = agc_waitlist_call(1, AGC_TASK_TIG_5);
    agc_relint();
    (void) AGC_ASSERT(!agc_flag_test(AGC_FLAG_ENGONFLG));
    return st;
}

/* REP40ALM (p.748), (14): flash V05N09 (alarm 1706, program not consistent
 * with vehicle configuration): V34 P00, V32 again, PROCEED to entry (14). */
agc_status_t agc_job_stage_alarm_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_dsky_cont_t c = {AGC_JOB_GOTOPOOH, AGC_JOB_STAGE_ALARM_PROCEED,
                               AGC_JOB_STAGE_ALARM_DISPLAY};
    (void) AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM);
    return agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(5, 9), &c);
}

/* REP40ALM PROCEED (p.748): entry (14): P42 may go on with the LM unstaged
 * (P42STAGE); P40 shows the alarm again.                                  */
agc_status_t agc_job_stage_alarm_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_job_id_t next = ign_table()->alarm_proceed;
    if (!AGC_ASSERT(next != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_exec_job_yield(next);
}

/* P40A/P (p.748): +0 from G+N,AUTO = PGNCS control and auto
 * stabilization; then on the ascent stage (APSFLAG), or in auto-throttle
 * (channel 30 bit 5 = 0, inverted input), return. Otherwise V50N25
 * R1 = 00203 "please perform checklist 203": V34 P00, PROCEED check
 * again, ENTER return anyway (GOBACK).                                     */
static agc_status_t auto_control_check(const agc_job_ctx_t *ctx,
                                       agc_job_id_t         ret_job)
{
    bool ok = (agc_imu_gn_auto() == 0);
    if (ok && !agc_flag_test(AGC_FLAG_APSFLAG))
    {
        ok = (agc_chan_read(030) & AGC_CH030_AUTOTHROT) == 0U;
    }
    if (ok)
    {
        return agc_exec_job_yield(ret_job); /* GOBACK                 */
    }
    const agc_dsky_cont_t c = {AGC_JOB_GOTOPOOH, AGC_JOB_AUTO_CONTROL_RECYCLE,
                               ret_job};
    (void) AGC_ASSERT(ret_job != AGC_JOB_NONE);
    return agc_dsky_display(ctx, AGC_DSKY_GOPERF1, 00203U, &c); /* TURNITON */
}

/* P40AUTO (p.748): save the return (TEMPR60), then P40A/P.                */
agc_status_t agc_lm_check_auto_control(const agc_job_ctx_t *ctx,
                                       agc_job_id_t         ret_job)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret_job != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_state()->lm.auto_control_recycle = ret_job;
    return auto_control_check(ctx, ret_job);
}

/* P40A/P recycle (PROCEED on V50N25): check again, same return.           */
agc_status_t agc_job_auto_control_recycle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return auto_control_check(ctx, agc_guid_state()->lm.auto_control_recycle);
}
