/**********************************************************************
 * FILE NAME: exec.c
 *
 * PURPOSE:
 *   exec-sched unit tests: executive priorities and core sets, VAC
 *   exhaustion (1201), core-set exhaustion (1202), sleep/wake, PRIOCHNG,
 *   waitlist ordering and LST1 delta arithmetic, WAIT2 inside a RUPTAGN
 *   chain, waitlist overflow (1203), zero delay (1204), LONGCALL,
 *   PHASCHNG types A/B/C and the return-address restart point, restart
 *   groups from the tables of both vehicles, GOPROG / ENEMA / GOTOPOOH,
 *   the KLEANs, POODOO, the interpreter alarms 1301 / 1302.
 *
 * AGC SOURCE:
 *   none (tests of src/exec, whose prologs cite the listings)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_exec_*, agc_waitlist_*, agc_phase_*, agc_restart_*
 *                         the functions under test (exec.h, internal.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails.
 *
 * NOTES:
 *   Links against the real library (build/libagc_debug.a). The test keeps
 *   its own agc_dispatch_job / agc_dispatch_task: test doubles of the
 *   dispatch boundary, so that the executive and waitlist can be driven by
 *   test jobs (TJ_* / TT_*). src/exec/dispatch.c defines nothing else, so
 *   the linker never pulls it in; every other exec, core and up-call
 *   function is the library's. AGC_ASSERT failures are counted by this
 *   file's agc_assert_report() and printed; no check depends on them.
 **********************************************************************/
#include <stdio.h>

#include <stdint.h>

#include "../../src/exec/internal.h"
#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"

static int g_checks;         /* checks run */
static int g_failures;       /* checks failed */
static int g_assert_reports; /* AGC_ASSERT failures reported */

/* Prototypes: main() comes first (STYLE 4.6). */
static void check(bool ok, int line, const char *text);
static void wl_clean(void);
static bool only_dspmm_waiting(void);
static void reset(agc_vehicle_t v);
static int  run_all(void);
static bool job_present(agc_job_id_t loc, agc_prio_t prio, int vac);
static bool task_entry(int i, agc_task_id_t task, int16_t lst1);
static bool task_queued(agc_task_id_t task);
static void test_priority_order(void);
static void test_vac_exhaustion(void);
static void test_core_exhaustion(void);
static void test_sleep_wake(void);
static void test_prio_change(void);
static void test_bailout_in_job(void);
static void test_waitlist_order(void);
static void test_waitlist_ruptagn_insert(void);
static void test_waitlist_overflow(void);
static void test_waitlist_zero_delay(void);
static void test_waitlist_delay_bound(void);
static void test_longcall(void);
static void test_phase_codes(void);
static void test_restart_table_task_timing(void);
static void test_restart_variable(void);
static void test_restart_return_point(void);
static void test_lm_tables(void);
static void test_kleans(void);
static void test_change_program_cleanup_lm(void);
static void test_goprog(void);
static void test_poodoo_lm(void);
static void test_inhint(void);
static void test_interp_alarms(void);
static void test_self_loop_pace(void);

/**********************************************************************
 * AGC: none (test) -- run every test
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    test_priority_order();
    test_vac_exhaustion();
    test_interp_alarms();
    test_core_exhaustion();
    test_sleep_wake();
    test_prio_change();
    test_self_loop_pace();
    test_bailout_in_job();
    test_waitlist_order();
    test_waitlist_ruptagn_insert();
    test_waitlist_overflow();
    test_waitlist_zero_delay();
    test_waitlist_delay_bound();
    test_longcall();
    test_phase_codes();
    test_restart_table_task_timing();
    test_restart_variable();
    test_restart_return_point();
    test_lm_tables();
    test_kleans();
    test_change_program_cleanup_lm();
    test_goprog();
    test_poodoo_lm();
    test_inhint();
    (void) printf("test_exec: %d checks, %d failures, %d assertion reports\n", g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1); /* exit status */
}

/**********************************************************************
 * AGC: none (test) -- hook replacing core/assert.c: count the report
 *
 * ARGUMENTS:
 *   file -- source file
 *   line -- line
 *   expr -- assertion text
 *
 * RETURNS: bool -- false (go on)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) file;
    (void) line;
    (void) expr;
    g_assert_reports++;

    return (false);
}

#define CHECK(c) check((c), __LINE__, #c)

/**********************************************************************
 * AGC: none (test) -- count a check, print a failure
 *
 * ARGUMENTS:
 *   ok -- the check passed
 *   line -- line
 *   text -- the checked expression
 **********************************************************************/
static void check(bool ok, int line, const char *text)
{
    g_checks++;

    if (!ok)
    {
        g_failures++;
        (void) printf("FAIL line %d: %s\n", line, text);
    }
}

#define TJ_A       ((agc_job_id_t) 0x0F01)  /* plain test job */
#define TJ_B       ((agc_job_id_t) 0x0F02)  /* plain test job */
#define TJ_C       ((agc_job_id_t) 0x0F03)  /* plain test job */
#define TJ_SLEEPER ((agc_job_id_t) 0x0F10)  /* JOBSLEEPs, wakes as TJ_WOKEN */
#define TJ_WOKEN   ((agc_job_id_t) 0x0F11)  /* continuation of TJ_SLEEPER */
#define TJ_HOG     ((agc_job_id_t) 0x0F12)  /* claims VAC areas until 1201 */
#define TJ_PRIO    ((agc_job_id_t) 0x0F13)  /* PRIOCHNG test job */
#define TJ_ACOS    ((agc_job_id_t) 0x0F14)  /* ACOS out of range: 1301 */
#define TJ_SQRT    ((agc_job_id_t) 0x0F15)  /* SQRT of a negative: 1302 */
#define TJ_GRP2PC  ((agc_job_id_t) 0x0F16)  /* PHASCHNG OCT 04022, return address */
#define TJ_LOOP    ((agc_job_id_t) 0x0F17)  /* CHANG1 to its own LOC (a polling loop) */
#define TT_A       ((agc_task_id_t) 0x0F41) /* plain test task */
#define TT_B       ((agc_task_id_t) 0x0F42) /* plain test task */
#define TT_C       ((agc_task_id_t) 0x0F43) /* plain test task */
#define TT_D       ((agc_task_id_t) 0x0F44) /* plain test task */
#define TT_LONG    ((agc_task_id_t) 0x0F45) /* LONGCALL test task */
#define TT_QUEUER  ((agc_task_id_t) 0x0F46) /* queues TT_C in 1 cs */
#define LOG_N      64                       /* size of the dispatch logs */

static agc_job_id_t  job_log[LOG_N];  /* jobs dispatched, in order */
static int           job_n;           /* entries in job_log */
static agc_task_id_t task_log[LOG_N]; /* test tasks dispatched, in order */
static int           task_n;          /* entries in task_log */
static int           prio_nj1;        /* NEWJOB after PRIOCHNG 25 */
static int           prio_nj2;        /* NEWJOB after PRIOCHNG 5  */
static agc_dp_t      mpac_result;     /* TJ_ACOS / TJ_SQRT MPAC, 2^0 */

/**********************************************************************
 * AGC: none (test) -- test double of the job dispatch switch: runs the TJ_* test jobs
 * and the exec jobs the tests need
 *
 * ARGUMENTS:
 *   ctx -- job context (core set 0); ctx->id is the test job
 *
 * RETURNS: agc_status_t -- the job's status
 **********************************************************************/
agc_status_t agc_dispatch_job(agc_job_ctx_t *ctx)
{
    if (job_n < LOG_N)
    {
        job_log[job_n] = ctx->id;
        job_n++;
    }

    agc_status_t st = AGC_STATUS_OK; /* the job's status */

    if (ctx->id == TJ_SLEEPER)
    {
        agc_exec_job_sleep(TJ_WOKEN);
    }
    else if (ctx->id == TJ_HOG)
    {
        for (int i = 0; i < 6 && st == AGC_STATUS_OK; i++)
        {
            st = agc_exec_findvac(012, TJ_C);
        }
    }
    else if (ctx->id == TJ_PRIO)
    {
        (void) agc_exec_novac(020, TJ_C);
        agc_exec_prio_change(025);
        prio_nj1 = agc_exec_newjob();
        agc_exec_prio_change(005);
        prio_nj2 = agc_exec_newjob();

        if (prio_nj2 > 0)
        {
            st = agc_exec_job_yield(TJ_A);
        }
    }
    else if (ctx->id == TJ_ACOS || ctx->id == TJ_SQRT)
    {
        agc_interp_t ip; /* interpreter context of the job */

        agc_interp_init(&ip);
        mpac_result = (ctx->id == TJ_ACOS) ? agc_dp_acos(&ip, AGC_DP_MAX) : agc_dp_sqrt(-AGC_DP_HALF); /* ACOS of 1.0 : SQRT of a negative */
    }
    else if (ctx->id == TJ_GRP2PC)
    {
        agc_phase_set_priority(2, 026); /* CAF PRIO26 / TS PHSPRDT2 */
        agc_phase_change(04022);        /* type C job, A and D clear */
    }
    else if (ctx->id == TJ_LOOP)
    {
        st = agc_exec_job_yield(TJ_LOOP);
    }
    else if (ctx->id == AGC_JOB_ENDOFJOB)
    {
        st = agc_job_endofjob(ctx);
    }
    else
    {
        /* plain test job: ENDOFJOB */
    }

    return (st);
} /* agc_dispatch_job */

/**********************************************************************
 * AGC: none (test) -- test double of the task dispatch switch: the waitlist's own
 * tasks, then a log of the TT_* test tasks
 *
 * ARGUMENTS:
 *   task -- task id
 *
 * RETURNS: agc_status_t -- the task's status
 **********************************************************************/
agc_status_t agc_dispatch_task(agc_task_id_t task)
{
    switch ((int) task)
    {
        case AGC_TASK_ENDTASK:
            return (agc_task_endtask());
        case AGC_TASK_SVCT3:
            return (agc_task_svct3());
        case AGC_TASK_LONGCYCL:
            return (agc_task_longcycl());
        case AGC_TASK_GETCADR:
            return (agc_task_getcadr());
        case AGC_TASK_TASKOVER:
            return (agc_task_taskover());
        case AGC_TASK_T5IDLOC:
            agc_task_t5idloc();

            return (AGC_STATUS_OK);
        case AGC_TASK_NONE:
        case AGC_TASK_COUNT:
        default:
            break;
    }

    if (task_n < LOG_N)
    {
        task_log[task_n] = task;
        task_n++;
    }

    agc_status_t st = AGC_STATUS_OK; /* the task's status */

    if (task == TT_QUEUER)
    {
        st = agc_waitlist_fixdelay(1, TT_C);
    }

    return (st);
}

/**********************************************************************
 * AGC: none (test) -- empty waitlist after a fresh start: TIME3 = 37777 makes ENDTASK
 * due in 1 cs; let it run, so the list holds only ENDTASK, 81.93 s away
 *
 * ARGUMENTS: none
 **********************************************************************/
static void wl_clean(void)
{
    agc_waitlist_init();
    (void) agc_waitlist_tick(1);
    task_n = 0;
}

/**********************************************************************
 * AGC: none (test) -- GOPROG3 MMDSPLAY with MODREG -0: only DSPMMJOB is active
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when DSPMMJOB is the only active job
 **********************************************************************/
static bool only_dspmm_waiting(void)
{
    int  active = 0;     /* active core sets */
    bool dspmm  = false; /* DSPMMJOB among them */

    for (int i = 0; i < agc_exec_core_sets(); i++)
    {
        agc_exec_core_info_t ci; /* core set i */

        agc_exec_core_info(i, &ci);

        if (ci.state != AGC_CORE_FREE)
        {
            active++;
            dspmm = dspmm || ci.loc == AGC_JOB_DSPMMJB;
        }
    }

    return (active == 1 && dspmm); /* GOPROG3 MMDSPLAY: DSPMMJOB only */
}

/**********************************************************************
 * AGC: none (test) -- fresh start of vehicle v with no program and an empty waitlist
 *
 * ARGUMENTS:
 *   v -- vehicle to reset to
 **********************************************************************/
static void reset(agc_vehicle_t v)
{
    agc_state_init(v);
    agc_phase_mr_klean();
    agc_erase_set_modreg(EXEC_MODREG_NEG0);

    for (int i = 0; i < 4 && agc_exec_restart_pending(); i++)
    {
        (void) agc_exec_service_restart();
    }

    (void) agc_chan_set_signal(AGC_CHAN_CHAN33, 060000u, false);
    agc_fresh_start_slap1();
    agc_exec_init();
    wl_clean();
    job_n = 0;
}

/**********************************************************************
 * AGC: none (test) -- run jobs until the executive is idle (at most 40)
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- the number of jobs run
 **********************************************************************/
static int run_all(void)
{
    int n = 0; /* jobs run */

    for (int i = 0; i < 40 && !agc_exec_idle(); i++)
    {
        (void) agc_exec_run_one();
        n++;
    }

    return (n);
}

/**********************************************************************
 * AGC: none (test) -- a job with this LOC, priority and VAC kind is active
 *
 * ARGUMENTS:
 *   loc -- LOC of the job looked for
 *   prio -- its priority number
 *   vac -- < 0: NOVAC job, >= 0: FINDVAC job
 *
 * RETURNS: bool -- true when found
 **********************************************************************/
static bool job_present(agc_job_id_t loc, agc_prio_t prio, int vac)
{
    for (int i = 0; i < agc_exec_core_sets(); i++)
    {
        agc_exec_core_info_t ci; /* core set i */

        agc_exec_core_info(i, &ci);

        if (ci.state != AGC_CORE_FREE && ci.loc == loc && ci.prio == prio && ((vac < 0) == (ci.vac_index < 0)))
        {
            return (true);
        }
    }

    return (false);
}

/**********************************************************************
 * AGC: none (test) -- waitlist entry i holds task with LST1 delta lst1
 *
 * ARGUMENTS:
 *   i -- list entry, 0..8
 *   task -- task id
 *   lst1 -- expected delta before it, cs
 *
 * RETURNS: bool -- true when it does
 **********************************************************************/
static bool task_entry(int i, agc_task_id_t task, int16_t lst1)
{
    int16_t       d = 0;             /* LST1 delta before entry i, cs */
    agc_task_id_t t = AGC_TASK_NONE; /* LST2 entry i */

    agc_waitlist_entry(i, &d, &t);

    return (t == task && d == lst1);
}

/**********************************************************************
 * AGC: none (test) -- task is on the waitlist
 *
 * ARGUMENTS:
 *   task -- task id
 *
 * RETURNS: bool -- true when it is
 **********************************************************************/
static bool task_queued(agc_task_id_t task)
{
    for (int i = 0; i < AGC_WAITLIST_MAX_TASKS; i++)
    {
        int16_t       d = 0;             /* LST1 delta before entry i, cs */
        agc_task_id_t t = AGC_TASK_NONE; /* LST2 entry i */

        agc_waitlist_entry(i, &d, &t);

        if (t == task)
        {
            return (true);
        }
    }

    return (false);
}

/**********************************************************************
 * AGC: none (test) -- a job that runs out of VAC areas: 1201 and a restart to
 * DSPMMJOB only (MODREG -0)
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_priority_order(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_idle());
    CHECK(agc_exec_novac(005, TJ_A) == AGC_STATUS_OK);
    CHECK(agc_exec_findvac(020, TJ_B) == AGC_STATUS_OK);
    CHECK(agc_exec_novac(010, TJ_C) == AGC_STATUS_OK);
    CHECK(agc_exec_newjob() == 1); /* TJ_B outranks TJ_A */
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

/**********************************************************************
 * AGC: none (test) -- FINDVAC with every VAC area in use: BAILOUT 1201
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_vac_exhaustion(void)
{
    reset(AGC_VEHICLE_CSM);

    for (int i = 0; i < AGC_EXEC_MAX_VACS; i++)
    {
        CHECK(agc_exec_findvac(010, TJ_A) == AGC_STATUS_OK);
    }

    CHECK(agc_exec_findvac(010, TJ_A) == AGC_STATUS_BAILOUT);
    CHECK(agc_erase_failreg(0) == 01201);
    CHECK((agc_erase_dsptab(11) & 0400u) != 0U); /* PROG lamp       */
    CHECK(agc_exec_restart_pending());
    (void) agc_exec_service_restart();           /* ENEMA           */
    CHECK(!agc_exec_restart_pending());
    CHECK(only_dspmm_waiting());                 /* tables cleared  */
    CHECK(agc_exec_findvac(010, TJ_A) == AGC_STATUS_OK);

    /* second and third alarms; the fourth marks FAILREG +2 bit 15     */
    agc_alarm(01202);
    agc_alarm(01203);
    agc_alarm(01204);
    CHECK(agc_erase_failreg(1) == 01202);
    CHECK(agc_erase_failreg(2) == (01204 | 040000));
}

/**********************************************************************
 * AGC: none (test) -- NOVAC with every core set in use: BAILOUT 1202
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_core_exhaustion(void)
{
    agc_vehicle_t v[2]    = {AGC_VEHICLE_CSM, AGC_VEHICLE_LM};
    int           sets[2] = {7, 8};

    for (int k = 0; k < 2; k++)
    {
        reset(v[k]);
        CHECK(agc_exec_core_sets() == sets[k]);

        for (int i = 0; i < sets[k]; i++)
        {
            CHECK(agc_exec_novac(010, TJ_A) == AGC_STATUS_OK);
        }

        CHECK(agc_exec_novac(010, TJ_A) == AGC_STATUS_BAILOUT);
        CHECK(agc_erase_failreg(0) == 01202);
        (void) agc_exec_service_restart();
        CHECK(only_dspmm_waiting());
    }
}

/**********************************************************************
 * AGC: none (test) -- JOBSLEEP / JOBWAKE by LOC; JOBWAKE of an unknown LOC or NONE is
 * NOT_FOUND
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_sleep_wake(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_novac(010, TJ_SLEEPER) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);
    CHECK(agc_exec_idle());                                         /* sleeper dormant */
    CHECK(agc_exec_job_wake(TJ_A) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_exec_job_wake(AGC_JOB_NONE) == AGC_STATUS_NOT_FOUND); /* no sleeper holds NONE */
    CHECK(agc_exec_job_wake(TJ_WOKEN) == AGC_STATUS_OK);
    CHECK(!agc_exec_idle());
    CHECK(run_all() == 1);
    CHECK(job_n == 2 && job_log[1] == TJ_WOKEN);
    CHECK(agc_exec_idle());
}

/**********************************************************************
 * AGC: none (test) -- PRIOCHNG gives way to a higher waiting job
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_prio_change(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_novac(010, TJ_PRIO) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);
    CHECK(prio_nj1 == 0);                            /* 25 > 20: keep running */
    CHECK(prio_nj2 > 0);                             /* 5 < 20: change job    */
    CHECK(run_all() == 2);
    CHECK(job_log[1] == TJ_C && job_log[2] == TJ_A); /* C, then resumed */
}

/**********************************************************************
 * AGC: none (test) -- a job that yields (CHANG1) to its own LOC runs once
 * per centisecond (DEVIATION in agc_exec_run_one): the second pass in the
 * same cs is BUSY, the next cs runs it; a job yielding to another LOC
 * (TJ_PRIO -> TJ_A in test_prio_change) is not held.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_self_loop_pace(void)
{
    reset(AGC_VEHICLE_CSM);
    agc_time_set_cs(1000u);
    CHECK(agc_exec_novac(010, TJ_LOOP) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);        /* pass 1 */
    CHECK(agc_exec_run_one() == AGC_STATUS_BUSY);      /* same cs */
    CHECK(agc_exec_run_one() == AGC_STATUS_BUSY);
    CHECK(job_n == 1);
    agc_time_advance_cs(1u);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);        /* pass 2 in the next cs */
    CHECK(agc_exec_run_one() == AGC_STATUS_BUSY);
    CHECK(job_n == 2 && job_log[1] == TJ_LOOP);
    CHECK(agc_exec_novac(020, TJ_A) == AGC_STATUS_OK); /* a higher job still runs at once */
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);
    CHECK(job_n == 3 && job_log[2] == TJ_A);
}

/**********************************************************************
 * AGC: none (test) -- a BAILOUT inside a job runs ENEMA when the job returns
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_bailout_in_job(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_novac(010, TJ_HOG) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_BAILOUT);
    CHECK(agc_erase_failreg(0) == 01201);
    CHECK(!agc_exec_restart_pending()); /* serviced        */
    CHECK(only_dspmm_waiting());        /* MODREG -0: no P00 */
}

/**********************************************************************
 * AGC: none (test) -- WAITLIST ordering and the LST1 delta arithmetic; RUPTAGN
 *
 * ARGUMENTS: none
 **********************************************************************/
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

/**********************************************************************
 * AGC: none (test) -- B4 (sim P30, CLOKTASK vs MONREQ): two tasks due in the same
 * cs; the first queues a 1 cs task from inside the RUPTAGN chain, where
 * TIME3 holds T - T2 = 0. WAIT2's overflow branch puts the new task after
 * the second chained task, 1 cs later; the second task is not pushed back
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_waitlist_ruptagn_insert(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_waitlist_call(10, TT_QUEUER) == AGC_STATUS_OK);
    CHECK(agc_waitlist_call(10, TT_B) == AGC_STATUS_OK);
    CHECK(agc_waitlist_tick(10) == AGC_STATUS_OK);
    CHECK(task_n == 2 && task_log[0] == TT_QUEUER && task_log[1] == TT_B);
    CHECK(task_entry(0, TT_C, 0)); /* -(T1 - T) + 1: in 1 cs */
    CHECK(agc_waitlist_tick(1) == AGC_STATUS_OK);
    CHECK(task_n == 3 && task_log[2] == TT_C);
    CHECK(!task_queued(TT_B));
}

/**********************************************************************
 * AGC: none (test) -- a tenth task: BAILOUT 1203, lists re-initialised
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_waitlist_overflow(void)
{
    reset(AGC_VEHICLE_LM);

    for (int i = 0; i < AGC_WAITLIST_MAX_TASKS; i++)
    {
        CHECK(agc_waitlist_call((uint16_t) (10 * (i + 1)), TT_A) == AGC_STATUS_OK);
    }

    CHECK(agc_waitlist_call(5, TT_B) == AGC_STATUS_BAILOUT);
    CHECK(agc_erase_failreg(0) == 01203);
    (void) agc_exec_service_restart();
    CHECK(!task_queued(TT_A)); /* lists re-initialised */
}

/**********************************************************************
 * AGC: none (test) -- WAITLIST with dt = 0: POODOO 1204, then P00 via GOTOPOOH
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_waitlist_zero_delay(void)
{
    reset(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(0);
    CHECK(agc_waitlist_call(0, TT_A) == AGC_STATUS_POODOO);
    CHECK(agc_erase_failreg(0) == 01204);
    CHECK(agc_exec_restart_pending());
    (void) agc_exec_service_restart();             /* -> GOTOPOOH     */
    CHECK(agc_phase_get(4) == 1U);                 /* CSM 4.1         */
    CHECK(job_present(AGC_JOB_GO_IDLE_FIX, 030, -1));
    CHECK(!job_present(AGC_JOB_DSPMMJB, 030, -1)); /* GOP00FIX took core set 0 */
}

/**********************************************************************
 * AGC: none (test) -- WAITLIST at DTMAX (16250 cs, WARNINGS 1, C p.1221)
 * is accepted; one cs more is refused with
 * BAD_ARG (DEVIATION: the AGC documents the maximum but does not test
 * it) and leaves the list and FAILREG untouched.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_waitlist_delay_bound(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_waitlist_call(AGC_WAITLIST_MAX_DELAY, TT_A) == AGC_STATUS_OK);
    CHECK(task_queued(TT_A));

    uint16_t t3 = agc_waitlist_time3(); /* C(TIME3) before the refused call */

    CHECK(agc_waitlist_call(AGC_WAITLIST_MAX_DELAY + 1, TT_B) == AGC_STATUS_BAD_ARG);
    CHECK(!task_queued(TT_B));
    CHECK(agc_waitlist_time3() == t3);
    CHECK(agc_erase_failreg(0) == 0);
    CHECK(!agc_exec_restart_pending());
}

/**********************************************************************
 * AGC: none (test) -- LONGCALL of 200 s through LONGCYCL and GETCADR
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_longcall(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_waitlist_longcall(20000, TT_LONG) == AGC_STATUS_OK);

    for (int i = 0; i < 19; i++)
    {
        (void) agc_waitlist_tick(1000);
    }

    (void) agc_waitlist_tick(999);
    CHECK(task_n == 0);
    (void) agc_waitlist_tick(1);
    CHECK(task_n == 1 && task_log[0] == TT_LONG);
}

/**********************************************************************
 * AGC: none (test) -- PHASCHNG types A and C, 2PHSCHNG, NEWPHASE, kill
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_phase_codes(void)
{
    reset(AGC_VEHICLE_CSM);
    agc_phase_change(040064); /* 4.6, TBASE */
    CHECK(agc_phase_get(4) == 6U);
    CHECK(phase_group_agrees(4));
    CHECK(agc_phase_get(3) == 0U);
    agc_phase_var_t v = {AGC_RESTART_JOB, TJ_A, AGC_TASK_NONE, 023, false, 0};

    agc_phase_change_var(05023, &v); /* type C  */
    CHECK(agc_phase_get(3) == 0502U);
    agc_phase_change2(000032, 000015, 0);
    CHECK(agc_phase_get(2) == 3U && agc_phase_get(5) == 1U);
    agc_phase_newphase(6, 5, true);
    CHECK(agc_phase_get(6) == 5U && phase_group_agrees(6));
    agc_phase_kill(6);
    CHECK(agc_phase_get(6) == 0U);
    CHECK(phase_group_agrees(6));
}

/**********************************************************************
 * AGC: none (test) -- a table task restarts at its time from TBASEn, or in 1 cs when
 * overdue
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_restart_table_task_timing(void)
{
    reset(AGC_VEHICLE_CSM);
    agc_time_set_cs(5000);
    agc_phase_change(040064);                    /* TBASE4 */
    agc_time_advance_cs(1000);
    CHECK(agc_restart_group(4) == AGC_STATUS_OK);
    CHECK(task_entry(0, AGC_TASK_PRECHECK, 0));  /* -0: in 1 cs     */
    CHECK(task_entry(1, AGC_TASK_TIG_5, -1494)); /* 2496-1000 = 1496 */
    wl_clean();
    agc_time_advance_cs(3000);                   /* now overdue     */
    CHECK(agc_restart_group(4) == AGC_STATUS_OK);
    CHECK(task_entry(0, AGC_TASK_PRECHECK, 0));
    CHECK(task_entry(1, AGC_TASK_TIG_5, 1));     /* also in 1 cs    */
}

/**********************************************************************
 * AGC: none (test) -- type C job (FINDVAC / NOVAC) and task, type B, X.1 restarts
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_restart_variable(void)
{
    reset(AGC_VEHICLE_CSM);
    agc_phase_var_t vj = {AGC_RESTART_JOB, TJ_A, AGC_TASK_NONE, 023, false, 0};

    agc_phase_change_var(05023, &vj);
    CHECK(agc_restart_group(3) == AGC_STATUS_OK);
    CHECK(job_present(TJ_A, 023, 0)); /* + prio: FINDVAC */
    agc_phase_var_t vn = {AGC_RESTART_JOB, TJ_B, AGC_TASK_NONE, -015, false, 0};

    agc_phase_change_var(05021, &vn);
    CHECK(agc_restart_group(1) == AGC_STATUS_OK);
    CHECK(job_present(TJ_B, 015, -1)); /* - prio: NOVAC   */
    agc_phase_var_t vt = {AGC_RESTART_TASK, AGC_JOB_NONE, TT_C, 0, true, 0};

    agc_phase_change_var(05015, &vt);
    CHECK(agc_restart_group(5) == AGC_STATUS_OK);
    CHECK(task_entry(0, TT_C, 0)); /* immediate: 1 cs */

    /* type B: job + first entry of the fixed phase (5.3 REREADAC)    */
    wl_clean();
    agc_phase_var_t vb = {AGC_RESTART_JOB, TJ_C, AGC_TASK_NONE, 015, false, 0};

    agc_phase_change_var(014035, &vb);
    CHECK(agc_phase_get(5) == 01403U);
    CHECK(agc_restart_group(5) == AGC_STATUS_OK);
    CHECK(job_present(TJ_C, 015, 0));
    CHECK(task_queued(AGC_TASK_REREADAC));

    /* X.1: redisplay                                                  */
    agc_phase_change(012);
    CHECK(agc_restart_group(2) == AGC_STATUS_OK);
    CHECK(job_present(AGC_JOB_INITDSP, 014, 0));
}

/**********************************************************************
 * AGC: none (test) -- type C job phase with A clear (GRP2PC, OCT 04022): the restart
 * point is the return address, here the job segment that changed the
 * phase; the priority is the preset PHSPRDT2
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_restart_return_point(void)
{
    reset(AGC_VEHICLE_LM);
    CHECK(agc_exec_novac(020, TJ_GRP2PC) == AGC_STATUS_OK);
    CHECK(run_all() == 1);
    CHECK(agc_phase_get(2) == 0402U);
    CHECK(agc_restart_group(2) == AGC_STATUS_OK);
    CHECK(job_present(TJ_GRP2PC, 026, 0)); /* + prio: FINDVAC */
}

/**********************************************************************
 * AGC: none (test) -- Luminary restart tables: 4.33, 1.2SPOT, 2.3 LONGCALL, 3.3 ZOOM
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_lm_tables(void)
{
    reset(AGC_VEHICLE_LM);
    agc_phase_newphase(4, 033, false);
    CHECK(agc_restart_group(4) == AGC_STATUS_OK);
    CHECK(job_present(AGC_JOB_GO_IDLE_FIX, 031, -1)); /* OCT 46777 NOVAC  */
    agc_phase_newphase(1, 2, false);                  /* 1.2SPOT */
    CHECK(agc_restart_group(1) == AGC_STATUS_OK);
    CHECK(job_present(AGC_JOB_ENDOFJOB, 021, 0));
    CHECK(task_queued(AGC_TASK_TASKOVER));
    agc_phase_newphase(2, 3, false);                  /* 2.3 LONGCALL */
    CHECK(agc_restart_group(2) == AGC_STATUS_OK);     /* 600 s from LONGBASE */
    CHECK(task_queued(AGC_TASK_LONGCYCL));
    agc_erase_set_zoomtime(2600);                     /* ZOOMTIME pad load, 26 s */
    agc_phase_newphase(3, 3, true);                   /* 3.3: ZOOM at TBASE3 + ZOOMTIME */
    CHECK(agc_restart_group(3) == AGC_STATUS_OK);
    CHECK(task_queued(AGC_TASK_ZOOM));
}

/**********************************************************************
 * AGC: none (test) -- V37KLEAN, P00KLEAN, MR.KLEAN on both vehicles
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_kleans(void)
{
    agc_vehicle_t v[2] = {AGC_VEHICLE_CSM, AGC_VEHICLE_LM};

    for (int k = 0; k < 2; k++)
    {
        reset(v[k]);

        for (int g = 1; g <= AGC_RESTART_GROUPS; g++)
        {
            agc_phase_newphase(g, 3, false);
        }

        agc_phase_change_program_klean();
        CHECK(agc_phase_get(3) == 0U && agc_phase_get(5) == 0U);
        CHECK(agc_phase_get(6) == 0U && agc_phase_get(2) == 3U);
        CHECK(agc_phase_get(1) == (k == 0 ? 3U : 0U)); /* LM kills 1 */
        agc_phase_idle_klean();
        CHECK(agc_phase_get(1) == 0U && agc_phase_get(4) == 0U);
        CHECK(agc_phase_get(2) == 3U);
        agc_phase_mr_klean();
        CHECK(agc_phase_get(2) == 0U);
    }
}

/**********************************************************************
 * AGC: none (test) -- LM R00 after INTSTALL (FRESH_START_AND_RESTART
 * L p.229), reached as V37's CANV37 with MMNUMBER = 0 (POOH): 3AXISFLG,
 * R04FLAG, MUNFLAG and XOVINFLG down; FLGWRD11 = LRBYBIT (TS, so every
 * other R12 bit is cleared too).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_change_program_cleanup_lm(void)
{
    reset(AGC_VEHICLE_LM);
    agc_flag_set(AGC_FLAG_3AXISFLG);
    agc_flag_set(AGC_FLAG_ALARM_521_SUPPRESSED); /* R04FLAG */
    agc_flag_set(AGC_FLAG_MUNFLAG);
    agc_flag_set(AGC_FLAG_XOVINFLG);
    agc_flag_set(AGC_FLAG_LRINH);                /* another FLGWRD11 bit */
    CHECK(agc_job_change_program_continue() == AGC_STATUS_OK);
    CHECK(!agc_flag_test(AGC_FLAG_3AXISFLG));
    CHECK(!agc_flag_test(AGC_FLAG_ALARM_521_SUPPRESSED));
    CHECK(!agc_flag_test(AGC_FLAG_MUNFLAG));
    CHECK(!agc_flag_test(AGC_FLAG_XOVINFLG));
    CHECK(agc_flagword_get(agc_flag_word(AGC_FLAG_LRBYPASS)) == agc_flag_mask(AGC_FLAG_LRBYPASS));
}

/**********************************************************************
 * AGC: none (test) -- GOPROG: 1107 fresh start on a bad phase table, P00 with no
 * active group, an active group restarted
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_goprog(void)
{
    reset(AGC_VEHICLE_CSM);
    uint16_t redo = agc_restart_redoctr(); /* REDOCTR before GOPROG */

    agc_erase_set_modreg(0);
    phase_debug_set_negative(3, 0);                /* tables disagree */
    CHECK(agc_rupt_go() == AGC_STATUS_ALARM);
    CHECK(agc_erase_failreg(0) == 01107);
    CHECK(agc_erase_modreg() == EXEC_MODREG_NEG0); /* fresh start     */
    CHECK(phase_group_agrees(3));
    CHECK(agc_restart_redoctr() == (uint16_t) (redo + 1U));

    /* no active group, MODREG = 0: P00 via GOTOPOOH                   */
    agc_erase_set_modreg(0);
    CHECK(agc_restart_goprog() == AGC_STATUS_OK);
    CHECK(!job_present(AGC_JOB_DSPMMJB, 030, -1));    /* MMDSPLAY's job in core set 0 ... */
    CHECK(job_present(AGC_JOB_GO_IDLE_FIX, 030, -1)); /* ... runs on as GOP00FIX         */
    CHECK(agc_phase_get(4) == 1U);

    /* an active group is restarted                                    */
    reset(AGC_VEHICLE_CSM);
    agc_phase_newphase(2, 5, false); /* 2.5 */
    CHECK(agc_restart_goprog() == AGC_STATUS_OK);
    CHECK(job_present(AGC_JOB_STATEINT_UPDATE, 005, 0));
    CHECK(agc_rupt_t5_task() == AGC_TASK_T5IDLOC);
}

/**********************************************************************
 * AGC: none (test) -- LM POODOO: MR.KLEAN, ENEMA to GOTOPOOH 4.33, STARTSB1 DAP part
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_poodoo_lm(void)
{
    reset(AGC_VEHICLE_LM);
    agc_erase_set_modreg(0);
    agc_phase_newphase(2, 5, false);
    agc_flag_set(AGC_FLAG_ACCOKFLG);                /* DAPBOOLS ACCSOKAY */
    CHECK(agc_poodoo(01206) == AGC_STATUS_POODOO);
    CHECK(agc_erase_failreg(0) == 01206);
    CHECK(agc_phase_get(2) == 0U);                  /* MR.KLEAN        */
    (void) agc_exec_service_restart();              /* WHIMPER -> ENEMA */
    CHECK(agc_phase_get(4) == 033U);                /* GOTOPOOH 4.33   */
    CHECK(job_present(AGC_JOB_GO_IDLE_FIX, 030, -1));
    CHECK(!job_present(AGC_JOB_DSPMMJB, 030, -1));  /* GOP00FIX took core set 0: PROG stays blank */
    CHECK(agc_rupt_t5_task() == AGC_TASK_DAPIDLER); /* STARTSB1      */
    CHECK(!agc_flag_test(AGC_FLAG_ACCOKFLG));       /* STARTSB1 DAP part: 1/ACCS reruns */
}

/**********************************************************************
 * AGC: none (test) -- INHINT / RELINT nesting
 *
 * ARGUMENTS: none
 **********************************************************************/
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

/**********************************************************************
 * AGC: none (test) -- ALARM 1301 / ABORT 1302 latched by ACOS / SQRT in a job are
 * raised by the executive when the job returns
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_interp_alarms(void)
{
    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_novac(010, TJ_ACOS) == AGC_STATUS_OK);
    CHECK(run_all() == 1);
    CHECK(agc_erase_failreg(0) == 01301);
    CHECK(!agc_exec_restart_pending()); /* ALARM: the job went on     */
    CHECK(agc_interp_take_alarm() == 0);

    reset(AGC_VEHICLE_CSM);
    CHECK(agc_exec_novac(010, TJ_SQRT) == AGC_STATUS_OK);
    (void) run_all();
    CHECK(mpac_result == 0);
    CHECK(agc_erase_failreg(0) == 01302);
    CHECK(agc_interp_take_alarm() == 0);
}
