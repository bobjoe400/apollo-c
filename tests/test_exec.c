/*
 * test_exec.c -- exec-sched unit tests: executive priorities and core
 * sets, VAC exhaustion (1201), core-set exhaustion (1202), waitlist
 * ordering and LST1 delta arithmetic, waitlist overflow (1203), dt = 0
 * (1204), LONGCALL, sleep/wake, PRIOCHNG, PHASCHNG types A/B/C, restart
 * groups from the tables of both vehicles, GOPROG / ENEMA / GOTOPOOH,
 * the KLEANs, POODOO.
 *
 * The test supplies its own agc_dispatch_job / agc_dispatch_task /
 * agc_dispatch_rupt and exec_up_* (the exec lane's only links to other
 * lanes), so it links against src/exec and src/core objects only:
 *   src/exec/{executive,waitlist,phase_table,restart_tables,restarts,
 *   alarm,fresh_start,interrupts,exec_erasable}.c + src/core/agc_state.c,
 *   agc_channels.c, agc_erasable.c.
 */
#include <stdio.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"
#include "agc/agc_channels.h"
#include "../src/exec/exec_internal.h"
#include "../src/exec/exec_pending.h"

static int g_checks;
static int g_failures;
static int g_assert_reports;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void)file;
    (void)line;
    (void)expr;
    g_assert_reports++;
    return false;
}

#define CHECK(c) check((c), __LINE__, #c)

static void check(bool ok, int line, const char *text)
{
    g_checks++;
    if (!ok) {
        g_failures++;
        (void)printf("FAIL line %d: %s\n", line, text);
    }
}

/* ---- test doubles for dispatch.c and the up-calls --------------------- */
#define TJ_A       ((agc_job_id_t)0x0F01)
#define TJ_B       ((agc_job_id_t)0x0F02)
#define TJ_C       ((agc_job_id_t)0x0F03)
#define TJ_SLEEPER ((agc_job_id_t)0x0F10)
#define TJ_WOKEN   ((agc_job_id_t)0x0F11)
#define TJ_HOG     ((agc_job_id_t)0x0F12)
#define TJ_PRIO    ((agc_job_id_t)0x0F13)
#define TT_A       ((agc_task_id_t)0x0F41)
#define TT_B       ((agc_task_id_t)0x0F42)
#define TT_C       ((agc_task_id_t)0x0F43)
#define TT_D       ((agc_task_id_t)0x0F44)
#define TT_LONG    ((agc_task_id_t)0x0F45)
#define LOG_N 64

static agc_job_id_t  job_log[LOG_N];
static int           job_n;
static agc_task_id_t task_log[LOG_N];
static int           task_n;
static agc_status_t  prio_st1;
static agc_status_t  prio_st2;

agc_status_t agc_dispatch_job(agc_job_ctx_t *ctx)
{
    if (job_n < LOG_N) {
        job_log[job_n++] = ctx->id;
    }
    agc_status_t st = AGC_STATUS_OK;
    if (ctx->id == TJ_SLEEPER) {
        st = agc_exec_job_sleep(TJ_WOKEN);
    } else if (ctx->id == TJ_HOG) {
        for (int i = 0; i < 6 && st == AGC_STATUS_OK; i++) {
            st = agc_exec_findvac(012, TJ_C);
        }
    } else if (ctx->id == TJ_PRIO) {
        (void)agc_exec_novac(020, TJ_C);
        prio_st1 = agc_exec_prio_change(025);
        prio_st2 = agc_exec_prio_change(005);
        if (prio_st2 == AGC_STATUS_BUSY) {
            st = agc_exec_job_yield(TJ_A);
        }
    } else if (ctx->id == AGC_JOB_ENDOFJOB) {
        st = agc_job_endofjob(ctx);
    } else {
        /* plain test job: ENDOFJOB */
    }
    return st;
}

agc_status_t agc_dispatch_task(agc_task_id_t task)
{
    switch (task) {
    case AGC_TASK_ENDTASK:  return agc_task_endtask();
    case AGC_TASK_SVCT3:    return agc_task_svct3();
    case AGC_TASK_LONGCYCL: return agc_task_longcycl();
    case AGC_TASK_GETCADR:  return agc_task_getcadr();
    case AGC_TASK_TASKOVER: return agc_task_taskover();
    case AGC_TASK_T5IDLOC:  return agc_task_t5idloc();
    case AGC_TASK_NONE:
    case AGC_TASK_COUNT_PLACEHOLDER:
    default:
        break;
    }
    if (task_n < LOG_N) {
        task_log[task_n++] = task;
    }
    return AGC_STATUS_OK;
}

agc_status_t agc_dispatch_rupt(agc_rupt_id_t rupt)
{
    (void)rupt;
    return AGC_STATUS_OK;
}

static int up_calls;
agc_status_t exec_up_stoprate(void) { up_calls++; return AGC_STATUS_OK; }
agc_status_t exec_up_servidle(void) { up_calls++; return AGC_STATUS_OK; }
agc_status_t exec_up_setcoars(void) { up_calls++; return AGC_STATUS_OK; }

/* ---- helpers ----------------------------------------------------------- */
/* Empty waitlist after a fresh start: TIME3 = 37777 makes ENDTASK due in
 * 1 cs; let it run, so the list holds only ENDTASK, 81.93 s away.      */
static void wl_clean(void)
{
    (void)agc_waitlist_init();
    (void)agc_waitlist_tick(1);
    task_n = 0;
}

static bool only_dspmm_waiting(void)
{
    int active = 0;
    bool dspmm = false;
    for (int i = 0; i < agc_exec_core_sets(); i++) {
        agc_exec_core_info_t ci;
        (void)agc_exec_core_info(i, &ci);
        if (ci.state != AGC_CORE_FREE) {
            active++;
            dspmm = dspmm || ci.loc == AGC_JOB_DSPMMJB;
        }
    }
    return active == 1 && dspmm;   /* GOPROG3 MMDSPLAY: DSPMMJOB only */
}

static void reset(agc_vehicle_t v)
{
    agc_state_init(v);
    (void)agc_phase_mr_klean();
    agc_erase_set_modreg(EXEC_MODREG_NEG0);
    for (int i = 0; i < 4 && agc_exec_restart_pending(); i++) {
        (void)agc_exec_service_restart();
    }
    (void)agc_chan_set_signal(AGC_CHAN_CHAN33, 060000u, false);
    (void)agc_fresh_start_slap1();
    (void)agc_exec_init();
    wl_clean();
    job_n = 0;
    up_calls = 0;
}

static int run_all(void)
{
    int n = 0;
    for (int i = 0; i < 40 && !agc_exec_idle(); i++) {
        (void)agc_exec_run_one();
        n++;
    }
    return n;
}

static bool job_present(agc_job_id_t loc, agc_prio_t prio, int vac)
{
    for (int i = 0; i < agc_exec_core_sets(); i++) {
        agc_exec_core_info_t ci;
        (void)agc_exec_core_info(i, &ci);
        if (ci.state != AGC_CORE_FREE && ci.loc == loc && ci.prio == prio &&
            ((vac < 0) == (ci.vac_index < 0))) {
            return true;
        }
    }
    return false;
}

static bool task_entry(int i, agc_task_id_t task, int16_t lst1)
{
    int16_t d = 0;
    agc_task_id_t t = AGC_TASK_NONE;
    (void)agc_waitlist_entry(i, &d, &t);
    return t == task && d == lst1;
}

static bool task_queued(agc_task_id_t task)
{
    for (int i = 0; i < AGC_WAITLIST_MAX_TASKS; i++) {
        int16_t d = 0;
        agc_task_id_t t = AGC_TASK_NONE;
        (void)agc_waitlist_entry(i, &d, &t);
        if (t == task) {
            return true;
        }
    }
    return false;
}

/* ---- executive --------------------------------------------------------- */
static void test_priority_order(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_idle());
    CHECK(agc_exec_novac(005, TJ_A) == AGC_STATUS_OK);
    CHECK(agc_exec_findvac(020, TJ_B) == AGC_STATUS_OK);
    CHECK(agc_exec_novac(010, TJ_C) == AGC_STATUS_OK);
    CHECK(agc_exec_newjob() == 1);              /* TJ_B outranks TJ_A */
    CHECK(run_all() == 3);
    CHECK(job_n == 3);
    CHECK(job_log[0] == TJ_B && job_log[1] == TJ_C && job_log[2] == TJ_A);
    CHECK(agc_exec_idle());
    CHECK(agc_exec_core_sets() == 7);
    /* equal priority: a FINDVAC job (VAC address in the low bits) beats
     * the NOVAC job created before it                                  */
    job_n = 0;
    CHECK(agc_exec_novac(010, TJ_A) == AGC_STATUS_OK);
    CHECK(agc_exec_novac(010, TJ_C) == AGC_STATUS_OK);
    CHECK(agc_exec_findvac(010, TJ_B) == AGC_STATUS_OK);
    CHECK(run_all() == 3);
    CHECK(job_log[0] == TJ_B);
}

static void test_vac_exhaustion_1201(void)
{
    reset(AGC_VEHICLE_CSM);
    for (int i = 0; i < AGC_EXEC_MAX_VACS; i++) {
        CHECK(agc_exec_findvac(010, TJ_A) == AGC_STATUS_OK);
    }
    CHECK(agc_exec_findvac(010, TJ_A) == AGC_STATUS_BAILOUT);
    CHECK(agc_erase_failreg(0) == 01201);
    CHECK((agc_erase_dsptab(11) & 0400u) != 0U);   /* PROG lamp       */
    CHECK(agc_exec_restart_pending());
    (void)agc_exec_service_restart();              /* ENEMA           */
    CHECK(!agc_exec_restart_pending());
    CHECK(only_dspmm_waiting());                   /* tables cleared  */
    CHECK(agc_exec_findvac(010, TJ_A) == AGC_STATUS_OK);
    /* second and third alarms; the fourth marks FAILREG +2 bit 15     */
    (void)agc_alarm(01202);
    (void)agc_alarm(01203);
    (void)agc_alarm(01204);
    CHECK(agc_erase_failreg(1) == 01202);
    CHECK(agc_erase_failreg(2) == (01204 | 040000));
}

static void test_core_exhaustion_1202(void)
{
    agc_vehicle_t v[2] = { AGC_VEHICLE_CSM, AGC_VEHICLE_LM };
    int sets[2] = { 7, 8 };
    for (int k = 0; k < 2; k++) {
        reset(v[k]);
        CHECK(agc_exec_core_sets() == sets[k]);
        for (int i = 0; i < sets[k]; i++) {
            CHECK(agc_exec_novac(010, TJ_A) == AGC_STATUS_OK);
        }
        CHECK(agc_exec_novac(010, TJ_A) == AGC_STATUS_BAILOUT);
        CHECK(agc_erase_failreg(0) == 01202);
        (void)agc_exec_service_restart();
        CHECK(only_dspmm_waiting());
    }
}

static void test_sleep_wake(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_novac(010, TJ_SLEEPER) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);
    CHECK(agc_exec_idle());                        /* sleeper dormant */
    CHECK(agc_exec_job_wake(TJ_A) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_exec_job_wake(TJ_WOKEN) == AGC_STATUS_OK);
    CHECK(!agc_exec_idle());
    CHECK(run_all() == 1);
    CHECK(job_n == 2 && job_log[1] == TJ_WOKEN);
    CHECK(agc_exec_idle());
}

static void test_prio_change(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_novac(010, TJ_PRIO) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);
    CHECK(prio_st1 == AGC_STATUS_OK);              /* 25 > 20         */
    CHECK(prio_st2 == AGC_STATUS_BUSY);            /* 5 < 20          */
    CHECK(run_all() == 2);
    CHECK(job_log[1] == TJ_C && job_log[2] == TJ_A); /* C, then resumed */
}

static void test_bailout_in_job(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_novac(010, TJ_HOG) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_BAILOUT);
    CHECK(agc_erase_failreg(0) == 01201);
    CHECK(!agc_exec_restart_pending());            /* serviced        */
    CHECK(only_dspmm_waiting());                   /* MODREG -0: no P00 */
}

/* ---- waitlist ---------------------------------------------------------- */
static void test_waitlist_order(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_waitlist_call(100, TT_A) == AGC_STATUS_OK);
    CHECK(agc_waitlist_call(50, TT_B) == AGC_STATUS_OK);
    CHECK(agc_waitlist_call(200, TT_C) == AGC_STATUS_OK);
    CHECK(agc_waitlist_call(50, TT_D) == AGC_STATUS_OK);
    CHECK(agc_waitlist_time3() == 16384 - 50);     /* 1.0 - DELTA T   */
    CHECK(task_entry(0, TT_B, -49));               /* -(T1 - T) + 1   */
    CHECK(task_entry(1, TT_D, 1));                 /* same time: +1   */
    CHECK(task_entry(2, TT_A, -49));               /* -(100-50) + 1   */
    CHECK(task_entry(3, TT_C, -99));               /* -(200-100) + 1  */
    CHECK(task_entry(4, AGC_TASK_ENDTASK, -7992)); /* -(8193-200) + 1 */
    CHECK(agc_waitlist_tick(49) == AGC_STATUS_OK);
    CHECK(task_n == 0);
    CHECK(agc_waitlist_tick(1) == AGC_STATUS_OK);  /* RUPTAGN: B, D   */
    CHECK(task_n == 2 && task_log[0] == TT_B && task_log[1] == TT_D);
    CHECK(agc_waitlist_tick(50) == AGC_STATUS_OK);
    CHECK(task_n == 3 && task_log[2] == TT_A);
    CHECK(agc_waitlist_tick(100) == AGC_STATUS_OK);
    CHECK(task_n == 4 && task_log[3] == TT_C);
}

static void test_waitlist_overflow_1203(void)
{
    reset(AGC_VEHICLE_LM);
    for (int i = 0; i < AGC_WAITLIST_MAX_TASKS; i++) {
        CHECK(agc_waitlist_call((uint16_t)(10 * (i + 1)), TT_A) == AGC_STATUS_OK);
    }
    CHECK(agc_waitlist_call(5, TT_B) == AGC_STATUS_BAILOUT);
    CHECK(agc_erase_failreg(0) == 01203);
    (void)agc_exec_service_restart();
    CHECK(!task_queued(TT_A));                     /* lists re-initialised */
}

static void test_waitlist_dt0_1204(void)
{
    reset(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(0);
    CHECK(agc_waitlist_call(0, TT_A) == AGC_STATUS_POODOO);
    CHECK(agc_erase_failreg(0) == 01204);
    CHECK(agc_exec_restart_pending());
    (void)agc_exec_service_restart();              /* -> GOTOPOOH     */
    CHECK(agc_phase_get(4) == 1U);                 /* CSM 4.1         */
    CHECK(job_present(AGC_JOB_GO_IDLE_FIX, 030, -1));
}

static void test_longcall(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_waitlist_longcall(20000, TT_LONG) == AGC_STATUS_OK);
    for (int i = 0; i < 19; i++) {
        (void)agc_waitlist_tick(1000);
    }
    (void)agc_waitlist_tick(999);
    CHECK(task_n == 0);
    (void)agc_waitlist_tick(1);
    CHECK(task_n == 1 && task_log[0] == TT_LONG);
}

/* ---- phase tables and restarts ---------------------------------------- */
static void test_phase_codes(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_phase_change(4, 040064) == AGC_STATUS_OK);   /* 4.6, TBASE */
    CHECK(agc_phase_get(4) == 6U);
    CHECK(phase_group_agrees(4));
    CHECK(agc_phase_change(3, 040064) == AGC_STATUS_BAD_ARG); /* group 4 */
    agc_phase_var_t v = { AGC_RESTART_JOB, TJ_A, AGC_TASK_NONE, 023, false, 0 };
    CHECK(agc_phase_change_var(05023, &v) == AGC_STATUS_OK);  /* type C  */
    CHECK(agc_phase_get(3) == 0502U);
    CHECK(agc_phase_change2(000032, 000015, 0) == AGC_STATUS_OK);
    CHECK(agc_phase_get(2) == 3U && agc_phase_get(5) == 1U);
    CHECK(agc_phase_newphase(6, 5, true) == AGC_STATUS_OK);
    CHECK(agc_phase_get(6) == 5U && phase_group_agrees(6));
    CHECK(agc_phase_kill(6) == AGC_STATUS_OK && agc_phase_get(6) == 0U);
    CHECK(phase_group_agrees(6));
}

static void test_restart_table_task_timing(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_time_set_cs(5000) == AGC_STATUS_OK);
    CHECK(agc_phase_change(4, 040064) == AGC_STATUS_OK);   /* TBASE4 */
    agc_time_advance_cs(1000);
    CHECK(agc_restart_group(4) == AGC_STATUS_OK);
    CHECK(task_entry(0, AGC_TASK_PRECHECK, 0));    /* -0: in 1 cs     */
    CHECK(task_entry(1, AGC_TASK_TIG_5, -1494));   /* 2496-1000 = 1496 */
    wl_clean();
    agc_time_advance_cs(3000);                     /* now overdue     */
    CHECK(agc_restart_group(4) == AGC_STATUS_OK);
    CHECK(task_entry(0, AGC_TASK_PRECHECK, 0));
    CHECK(task_entry(1, AGC_TASK_TIG_5, 1));       /* also in 1 cs    */
}

static void test_restart_variable(void)
{
    reset(AGC_VEHICLE_CSM);
    agc_phase_var_t vj = { AGC_RESTART_JOB, TJ_A, AGC_TASK_NONE, 023, false, 0 };
    CHECK(agc_phase_change_var(05023, &vj) == AGC_STATUS_OK);
    CHECK(agc_restart_group(3) == AGC_STATUS_OK);
    CHECK(job_present(TJ_A, 023, 0));              /* + prio: FINDVAC */
    agc_phase_var_t vn = { AGC_RESTART_JOB, TJ_B, AGC_TASK_NONE, -015, false, 0 };
    CHECK(agc_phase_change_var(05021, &vn) == AGC_STATUS_OK);
    CHECK(agc_restart_group(1) == AGC_STATUS_OK);
    CHECK(job_present(TJ_B, 015, -1));             /* - prio: NOVAC   */
    agc_phase_var_t vt = { AGC_RESTART_TASK, AGC_JOB_NONE, TT_C, 0, true, 0 };
    CHECK(agc_phase_change_var(05015, &vt) == AGC_STATUS_OK);
    CHECK(agc_restart_group(5) == AGC_STATUS_OK);
    CHECK(task_entry(0, TT_C, 0));                 /* immediate: 1 cs */
    /* type B: job + first entry of the fixed phase (5.3 REREADAC)    */
    wl_clean();
    agc_phase_var_t vb = { AGC_RESTART_JOB, TJ_C, AGC_TASK_NONE, 015, false, 0 };
    CHECK(agc_phase_change_var(014035, &vb) == AGC_STATUS_OK);
    CHECK(agc_phase_get(5) == 01403U);
    CHECK(agc_restart_group(5) == AGC_STATUS_OK);
    CHECK(job_present(TJ_C, 015, 0));
    CHECK(task_queued(AGC_TASK_REREADAC));
    /* X.1: redisplay                                                  */
    CHECK(agc_phase_change(2, 012) == AGC_STATUS_OK);
    CHECK(agc_restart_group(2) == AGC_STATUS_OK);
    CHECK(job_present(AGC_JOB_INITDSP, 014, 0));
}

static void test_lm_tables(void)
{
    reset(AGC_VEHICLE_LM);
    CHECK(agc_phase_newphase(4, 033, false) == AGC_STATUS_OK);
    CHECK(agc_restart_group(4) == AGC_STATUS_OK);
    CHECK(job_present(AGC_JOB_GO_IDLE_FIX, 031, -1)); /* OCT 46777 NOVAC  */
    CHECK(agc_phase_newphase(1, 2, false) == AGC_STATUS_OK); /* 1.2SPOT */
    CHECK(agc_restart_group(1) == AGC_STATUS_OK);
    CHECK(job_present(AGC_JOB_ENDOFJOB, 021, 0));
    CHECK(task_queued(AGC_TASK_TASKOVER));
    CHECK(agc_phase_newphase(2, 3, false) == AGC_STATUS_OK); /* 2.3 LONGCALL */
    CHECK(agc_restart_group(2) == AGC_STATUS_OK);  /* 600 s from LONGBASE */
    CHECK(task_queued(AGC_TASK_LONGCYCL));
    CHECK(agc_phase_newphase(3, 3, false) == AGC_STATUS_OK); /* ZOOMTIME */
    CHECK(agc_restart_group(3) == AGC_STATUS_UNIMPLEMENTED);
}

static void test_kleans(void)
{
    agc_vehicle_t v[2] = { AGC_VEHICLE_CSM, AGC_VEHICLE_LM };
    for (int k = 0; k < 2; k++) {
        reset(v[k]);
        for (int g = 1; g <= AGC_RESTART_GROUPS; g++) {
            (void)agc_phase_newphase(g, 3, false);
        }
        CHECK(agc_phase_change_program_klean() == AGC_STATUS_OK);
        CHECK(agc_phase_get(3) == 0U && agc_phase_get(5) == 0U);
        CHECK(agc_phase_get(6) == 0U && agc_phase_get(2) == 3U);
        CHECK(agc_phase_get(1) == (k == 0 ? 3U : 0U));  /* LM kills 1 */
        CHECK(agc_phase_idle_klean() == AGC_STATUS_OK);
        CHECK(agc_phase_get(1) == 0U && agc_phase_get(4) == 0U);
        CHECK(agc_phase_get(2) == 3U);
        CHECK(agc_phase_mr_klean() == AGC_STATUS_OK && agc_phase_get(2) == 0U);
    }
}

static void test_goprog(void)
{
    reset(AGC_VEHICLE_CSM);
    uint16_t redo = agc_restart_redoctr();
    agc_erase_set_modreg(0);
    phase_debug_set_negative(3, 0);                /* tables disagree */
    CHECK(agc_rupt_go() == AGC_STATUS_ALARM);
    CHECK(agc_erase_failreg(0) == 01107);
    CHECK(agc_erase_modreg() == EXEC_MODREG_NEG0); /* fresh start     */
    CHECK(phase_group_agrees(3));
    CHECK(agc_restart_redoctr() == (uint16_t)(redo + 1U));
    /* no active group, MODREG = 0: P00 via GOTOPOOH                   */
    agc_erase_set_modreg(0);
    CHECK(agc_restart_goprog() == AGC_STATUS_OK);
    CHECK(job_present(AGC_JOB_DSPMMJB, 030, -1));  /* MMDSPLAY        */
    CHECK(job_present(AGC_JOB_GO_IDLE_FIX, 030, -1));
    CHECK(agc_phase_get(4) == 1U);
    /* an active group is restarted                                    */
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_phase_newphase(2, 5, false) == AGC_STATUS_OK);  /* 2.5 */
    CHECK(agc_restart_goprog() == AGC_STATUS_OK);
    CHECK(job_present(AGC_JOB_STATINT1, 005, 0));
    CHECK(agc_rupt_t5_task() == AGC_TASK_T5IDLOC);
}

static void test_poodoo_lm(void)
{
    reset(AGC_VEHICLE_LM);
    agc_erase_set_modreg(0);
    (void)agc_phase_newphase(2, 5, false);
    CHECK(agc_poodoo(01206) == AGC_STATUS_POODOO);
    CHECK(agc_erase_failreg(0) == 01206);
    CHECK(agc_phase_get(2) == 0U);                 /* MR.KLEAN        */
    (void)agc_exec_service_restart();              /* WHIMPER -> ENEMA */
    CHECK(agc_phase_get(4) == 033U);               /* GOTOPOOH 4.33   */
    CHECK(job_present(AGC_JOB_GO_IDLE_FIX, 030, -1));
    CHECK(agc_rupt_t5_task() == AGC_TASK_DAPIDLER);  /* STARTSB1      */
}

static void test_inhint(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(!agc_interrupts_inhibited());
    agc_inhint();
    agc_inhint();
    agc_relint();
    CHECK(agc_interrupts_inhibited());
    agc_relint();
    CHECK(!agc_interrupts_inhibited());
}

int main(void)
{
    test_priority_order();
    test_vac_exhaustion_1201();
    test_core_exhaustion_1202();
    test_sleep_wake();
    test_prio_change();
    test_bailout_in_job();
    test_waitlist_order();
    test_waitlist_overflow_1203();
    test_waitlist_dt0_1204();
    test_longcall();
    test_phase_codes();
    test_restart_table_task_timing();
    test_restart_variable();
    test_lm_tables();
    test_kleans();
    test_goprog();
    test_poodoo_lm();
    test_inhint();
    (void)printf("test_exec: %d checks, %d failures, %d assertion reports\n",
                 g_checks, g_failures, g_assert_reports);
    return g_failures == 0 ? 0 : 1;
}
