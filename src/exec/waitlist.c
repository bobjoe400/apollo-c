/**********************************************************************
 * FILE NAME: waitlist.c
 *
 * PURPOSE:
 *   The WAITLIST: task scheduling by delta time (WAITLIST, TWIDDLE,
 *   VARDELAY, FIXDELAY, WAIT2, WTLST4/5, WTLST2), the TIME3 interrupt that
 *   runs them (T3RUPT, T3RUPT2, TASKOVER), the idle task (ENDTASK, SVCT3),
 *   long delays (LONGCALL, LONGCYCL, GETCADR) and LM KILLTASK.
 *
 * AGC SOURCE:
 *   Comanche055 WAITLIST.agc pp. 1221-1235
 *   Luminary099 WAITLIST.agc pp. 1117-1132 (differs: see NOTES);
 *   Luminary099 BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc pp. 750-751
 *   (KILLTASK)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DRIFTFLG      flag      I    -              SVCT3: compensate drift
 *   IMUCADR       agc_sp_t  I    -              SVCT3: IMU in use (MODECADR)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dispatch_task     DTCB to the task's 2CADR (dispatch.c)
 *   agc_bailout           BAILOUT 1203 (alarm.c)
 *   agc_poodoo            POODOO 1204 (alarm.c)
 *   agc_exec_novac        NOVAC NBDONLY (executive.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   BAILOUT 1203 (more than 9 tasks); POODOO 1204 (zero delay);
 *   AGC_STATUS_BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Tasks run with interrupts inhibited; the sim harness drives TIME3
 *   through agc_waitlist_tick. Both vehicles.
 *
 * NOTES:
 *   Differences between the listings:
 *     - Comanche tests DT <= 0 at WAITLIST (TWIDDLE falls into it) and
 *       POODOOs 1204; Luminary tests at WAIT2, i.e. also for VARDELAY and
 *       FIXDELAY, and also tests the LONGCALL delta (LONGPOOH). Both 1204.
 *     - overflow: Comanche TC BAILOUT / OCT 1203; Luminary TC FILLED ->
 *       BAILOUT1 / OCT 01203 (records the caller). Same code.
 *
 *   The delta-time model is kept exactly (DTMAX 162.5 s, 9 tasks):
 *       C(TIME3)    = 16384 - (T1 - T)     centiseconds, T = now
 *       C(LST1 +k)  = -(T(k+2) - T(k+1)) + 1          k = 0..7
 *       C(LST2 +k)  = task k+1                        k = 0..8
 *   An empty slot holds ENDTASK in LST2 and NEG1/2 (-8192) in LST1, so with
 *   nothing scheduled ENDTASK (SVCT3) runs every 8193 cs = 81.93 s.
 *   TIME3 is a 14-bit counter that overflows at 16384 cs; the overflow is
 *   T3RUPT. TIME1/TIME2 are not advanced here (the sim harness owns them).
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define WL_TIME3_OVF   16384   /* TIME3 overflow, 2^14 cs = 163.84 s   */
#define WL_POSMAX      16383   /* POSMAX, octal 37777                  */
#define WL_NEG1_2      (-8192) /* NEG1/2: empty LST1 slot              */
#define WL_LST1_N      8       /* LST1 to LST1 +7                      */
#define WL_BIT14       8192    /* BIT14 / DPBIT14 low word, 81.92 s    */
#define WL_TIME3_BIT8  128     /* BIT8, octal 200: WAIT2 overflow test */
#define WL_SVCT3_DELAY 500     /* SVCT3X: DEC 500 (5 s)                */
#define WL_TICK_LIMIT  65536   /* bound for agc_waitlist_tick loop     */

static uint16_t      time3;                            /* TIME3              */
static int16_t       lst1[WL_LST1_N];                  /* LST1               */
static agc_task_id_t lst2[AGC_WAITLIST_MAX_TASKS];     /* LST2 (task ids)    */
static uint32_t      pending_longtime;                 /* LONGTIME B(2) cs   */
static agc_task_id_t pending_longcadr = AGC_TASK_NONE; /* LONGCADR           */

/**********************************************************************
 * STARTSB2 (FRESH_START_AND_RESTART C p.190 / L p.220): initialise the
 * waitlist delta-Ts to NEG1/2 and every LST2 entry to ENDTASK.
 *
 * ARGUMENTS: none
 **********************************************************************/
void waitlist_clear_lists(void)
{
    for (int k = 0; k < WL_LST1_N; k++)
    {
        lst1[k] = (int16_t) WL_NEG1_2;
    }

    for (int k = 0; k < AGC_WAITLIST_MAX_TASKS; k++)
    {
        lst2[k] = AGC_TASK_ENDTASK;
    }
}

/**********************************************************************
 * STARTSUB / STARTSB1 (FRESH_START_AND_RESTART.agc C p.190 / L p.220):
 * CAF POSMAX / TS TIME3, so T3RUPT comes 1 cs later.
 *
 * ARGUMENTS: none
 **********************************************************************/
void waitlist_reset_time3(void)
{
    time3 = (uint16_t) WL_POSMAX;
}

/**********************************************************************
 * Waitlist power-on: STARTSUB / STARTSB2 (FRESH_START_AND_RESTART.agc
 * C p.190 / L p.220) for TIME3, LST1 and LST2, no LONGCALL pending.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG on a failed postcondition
 **********************************************************************/
agc_status_t agc_waitlist_init(void)
{
    waitlist_reset_time3();
    waitlist_clear_lists();
    pending_longtime = 0;
    pending_longcadr = AGC_TASK_NONE;

    if (!AGC_ASSERT(lst2[0] == AGC_TASK_ENDTASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * WTLST4 + WTLST2 (C pp.1227,1230 / L pp.1123,1127): insert `task` as
 * entry n (0..8) of LST2 and `delta` as LST1 +n, pushing the later
 * entries down. The entry pushed off the end of LST2 must be ENDTASK,
 * otherwise the list is full (WTABORT -> 1203).
 *
 * ARGUMENTS:
 *   n -- insertion index, 0..8 (LST2 +2n)
 *   delta -- LST1 +n: -(T(n+1) - TD) + 1, cs (<= 1)
 *   task -- waitlist task id (AGC_TASK_*), its 2CADR
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAILOUT (1203) when the list is
 *          full; BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t wtlst4(int n, int32_t delta, agc_task_id_t task)
{
    /* DEVIATION: the AGC shifts first and checks the ejected LST2 word
     * afterwards; checking first is equivalent because the BAILOUT that
     * follows re-initialises both lists in STARTSB2.                   */
    if (lst2[AGC_WAITLIST_MAX_TASKS - 1] != AGC_TASK_ENDTASK)
    {
        return (agc_bailout(AGC_ALARM_01203)); /* NO ROOM IN THE INN  */
    }

    if (n < WL_LST1_N)
    { /* INDEX Q / TCF WTLST4 */
        if (!AGC_ASSERT(delta >= -WL_POSMAX && delta <= 1))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        for (int k = WL_LST1_N - 1; k > n; k--)
        {
            lst1[k] = lst1[k - 1]; /* XCH LST1 +k chain   */
        }

        lst1[n] = (int16_t) delta;
    }

    for (int k = AGC_WAITLIST_MAX_TASKS - 1; k > n; k--)
    {
        lst2[k] = lst2[k - 1]; /* DXCH LST2 +2k chain */
    }

    lst2[n] = task;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * WAIT2 / WTLST5 / WTLST2 (C pp.1226-1230 / L pp.1122-1127): task
 * insertion. TD = T + dt is the desired time; T(n) the time of task n.
 *
 * ARGUMENTS:
 *   dt -- delay from now, 1..16250 cs
 *   task -- waitlist task id (AGC_TASK_*), its 2CADR
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAILOUT (1203) when the list is
 *          full; BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t wait2(uint16_t dt, agc_task_id_t task)
{
    if (!AGC_ASSERT(dt >= 1 && dt <= AGC_WAITLIST_MAX_DELAY) || !AGC_ASSERT(time3 < WL_TIME3_OVF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* CS TIME3 / AD BIT8 / CCS A: 200 - C(TIME3) > 0 means TIME3 has
     * overflowed and C(TIME3) = T - T1. That happens inside a RUPTAGN
     * chain: ADS TIME3 in T3RUPT2 leaves the excess T - T2 (0 for two
     * tasks due in the same cs) while task 1 runs, and T2 is the next
     * task. A normal TIME3 is >= 134 (DTMAX 16250), so the test is
     * unambiguous. Both paths give TD - T1 + 1; 200 itself is -0 in the
     * CCS and goes straight to AD Q.                                     */
    int32_t a = (int32_t) dt - (WL_TIME3_OVF - (int32_t) time3) + 1; /* normal: -(1.0 - C(TIME3)) + 1 + DT */

    if (time3 < WL_TIME3_BIT8)
    {
        a = (int32_t) dt + (int32_t) time3 + 1; /* AD OCT40001 / CS A / AD OCT40201 / AD Q */
    }
    else if (time3 == WL_TIME3_BIT8)
    {
        a = (int32_t) dt; /* CCS -0 -> AD Q */
    }

    if (a <= 0)
    {
        /* TD before T1: new task 1. TIME3 = 1.0 - DELTA T, and the old
         * task 1 gets LST1 = -(T1 - TD) + 1, which equals a.             */
        time3 = (uint16_t) (WL_TIME3_OVF - (int32_t) dt);

        return (wtlst4(0, a, task));
    }

    a = a - 1 + lst1[0]; /* TD - T2 + 1         */

    for (int n = 1; n < AGC_WAITLIST_MAX_TASKS; n++)
    {
        if (a <= 0)
        {
            /* WTLST2: T(n) <= TD < T(n+1). LST1 +n-1 becomes
             * -(TD - T(n)) + 1 and -(T(n+1) - TD) + 1 = a is inserted.  */
            int32_t prev = (int32_t) lst1[n - 1] - a + 1; /* LST1 +n-1 after the insertion */

            if (!AGC_ASSERT(prev <= 1 && prev >= -WL_POSMAX))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            agc_status_t st = wtlst4(n, a, task); /* WTLST4 outcome */

            if (st == AGC_STATUS_OK)
            {
                lst1[n - 1] = (int16_t) prev;
            }

            return (st);
        }

        if (n < WL_LST1_N)
        {
            a = a - 1 + lst1[n]; /* TD - T(n+2) + 1     */
        }
    }

    return (agc_bailout(AGC_ALARM_01203)); /* WTABORT              */
} /* wait2 */

/**********************************************************************
 * WAITLIST / TWIDDLE (C p.1223 / L p.1119): call `task` in dt centiseconds
 * (1 <= dt <= 16250). dt = 0 is POODOO 1204 (WATLST0- / WAITPOOH).
 *
 * ARGUMENTS:
 *   delay_cs -- delay from now, centiseconds
 *   task -- waitlist task id (AGC_TASK_*), its 2CADR
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAILOUT (1203) when the list is full;
 *          AGC_STATUS_POODOO (1204) for a zero delay; BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_waitlist_call(uint16_t delay_cs, agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (delay_cs == 0)
    {
        return (agc_poodoo(AGC_ALARM_01204));
    }

    if (!AGC_ASSERT(delay_cs <= AGC_WAITLIST_MAX_DELAY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_status_t st = wait2(delay_cs, task); /* WAIT2 outcome */

    agc_relint();

    return (st);
}

/**********************************************************************
 * VARDELAY (C p.1223 / L p.1119): called from a task, which then ends;
 * the task continues as `task` after dt cs. Comanche checks dt <= 0 only
 * in WAITLIST.
 * DEVIATION: dt = 0 here POODOOs 1204 for both vehicles (Luminary
 * behaviour); in Comanche it would corrupt TIME3.
 *
 * ARGUMENTS:
 *   delay_cs -- delay from now, centiseconds
 *   task -- waitlist task id (AGC_TASK_*), its 2CADR
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAILOUT (1203) when the list is full;
 *          AGC_STATUS_POODOO (1204) for a zero delay; BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_waitlist_vardelay(uint16_t delay_cs, agc_task_id_t task)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG); /* must be in a task    */
    }

    return (agc_waitlist_call(delay_cs, task));
}

/**********************************************************************
 * FIXDELAY (C p.1223 / L p.1119): as VARDELAY with the delay in the word
 * after the call; in C the caller passes it.
 *
 * ARGUMENTS:
 *   delay_cs -- delay from now, centiseconds
 *   task -- waitlist task id (AGC_TASK_*), its 2CADR
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAILOUT (1203) when the list is full;
 *          AGC_STATUS_POODOO (1204) for a zero delay; BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_waitlist_fixdelay(uint16_t delay_cs, agc_task_id_t task)
{
    return (agc_waitlist_vardelay(delay_cs, task));
}

/**********************************************************************
 * LONGCYCL / MUCHTIME / LASTTIME (C pp.1234-1235 / L pp.1131-1132): take
 * 81.92 s off LONGTIME; if time remains, come back in 81.92 s, otherwise
 * schedule GETCADR for the remainder.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAILOUT (1203) when the list is full;
 *          AGC_STATUS_POODOO (1204) for a zero delay; BAD_ARG on a failed
 *          precondition
 **********************************************************************/
static agc_status_t longcycl(void)
{
    if (!AGC_ASSERT(pending_longcadr != AGC_TASK_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int64_t rem = (int64_t) pending_longtime - WL_BIT14; /* DCS DPBIT14 / DAS    */

    if (rem > 0)
    {
        pending_longtime = (uint32_t) rem;

        return (agc_waitlist_call((uint16_t) WL_BIT14, AGC_TASK_LONGCYCL));
    }

    rem += WL_BIT14; /* LASTTIME: ADS BIT14 */

    if (!AGC_ASSERT(rem >= 0 && rem <= WL_BIT14))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pending_longtime = (uint32_t) rem;

    return (agc_waitlist_call((uint16_t) rem, AGC_TASK_GETCADR));
}

/**********************************************************************
 * LONGCALL / LNGCALL2 (C pp.1233-1234 / L pp.1130-1131): call `task` after
 * a DP delay in centiseconds (scaled as TIME2,TIME1). Only one LONGCALL
 * can be pending (LONGTIME, LONGCADR are single registers).
 *
 * ARGUMENTS:
 *   delay_cs -- delay from now, centiseconds
 *   task -- waitlist task id (AGC_TASK_*), its 2CADR
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAILOUT (1203) when the list is full;
 *          AGC_STATUS_POODOO (1204) for a zero delay; BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_waitlist_longcall(uint32_t delay_cs, agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE) || !AGC_ASSERT(delay_cs < AGC_TIME_MODULUS_CS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (delay_cs == 0 && agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return (agc_poodoo(AGC_ALARM_01204)); /* LONGPOOH            */
    }

    agc_inhint();
    pending_longtime = delay_cs;   /* DXCH LONGTIME       */
    pending_longcadr = task;       /* DXCH LONGCADR       */
    agc_status_t st  = longcycl(); /* LONGCYCL outcome */

    agc_relint();

    return (st);
}

/**********************************************************************
 * ENDTASK -> SVCT3 / CKIMUSE / SVCT3X (C p.1225 / L p.1121): every 81.93 s
 * when the list is empty. With DRIFTFLG set start NBDONLY (IMU drift
 * compensation) unless someone is using the IMU, in which case retry in
 * 5 s.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, the FIXDELAY or NOVAC status; BAD_ARG on a
 *          failed precondition
 **********************************************************************/
static agc_status_t svct3(void)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!agc_flag_test(AGC_FLAG_DRIFTFLG))
    {
        return (AGC_STATUS_OK); /* TCF TASKOVER        */
    }

    if (agc_erase_imucadr() != 0)
    { /* CCS IMUCADR         */
        return (agc_waitlist_fixdelay(WL_SVCT3_DELAY, AGC_TASK_SVCT3));
    }

    agc_status_t st = agc_exec_novac(EXEC_PRIO35, AGC_JOB_NBDONLY); /* NOVAC NBDONLY outcome */

    return (st);
}

/* The waitlist's own tasks (called from agc_dispatch_task).            */
/**********************************************************************
 * KILLTASK / KILLTSK2 (Luminary BURN_BABY_BURN p.750-751, LM only): scan LST2 for the task
 * and put TASKOVER in its place (KILLDEAD), so the slot runs as a no-op
 * at its time. Only the first match is killed; none is no action.
 *
 * ARGUMENTS:
 *   task -- waitlist task id (AGC_TASK_*), its 2CADR
 **********************************************************************/
void agc_waitlist_kill(agc_task_id_t task)
{
    agc_inhint();
    bool done = false; /* first match replaced */

    for (int k = 0; k < AGC_WAITLIST_MAX_TASKS && !done; k++)
    {
        if (lst2[k] == task)
        {
            lst2[k] = AGC_TASK_TASKOVER; /* CA TCTSKOVR / TS LST2  */
            done    = true;
        }
    }

    agc_relint();
}

/**********************************************************************
 * ENDTASK (WAITLIST.agc C p.1225 / L p.1121): -2CADR SVCT3, the task in
 * every empty LST2 slot.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the SVCT3 status; BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_task_endtask(void)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(lst2[0] != AGC_TASK_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (svct3());
}

/**********************************************************************
 * SVCT3 as a task (WAITLIST.agc C p.1225 / L p.1121): the SVCT3X retry
 * after FIXDELAY 500.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the SVCT3 status; BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_task_svct3(void)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(lst2[0] != AGC_TASK_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (svct3());
}

/**********************************************************************
 * LONGCYCL as a task (WAITLIST.agc C p.1234 / L p.1131): another
 * 81.92 s of a LONGCALL has passed.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAILOUT (1203) when the list is full;
 *          AGC_STATUS_POODOO (1204) for a zero delay; BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_longcycl(void)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(pending_longcadr != AGC_TASK_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (longcycl());
}

/**********************************************************************
 * TASKOVER as a task (LM RESTART_TABLES 1.2SPOT dummy entry, p.239):
 * "TASKOVER" ends the task at once.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_task_taskover(void)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(time3 < WL_TIME3_OVF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GETCADR (C p.1235 / L p.1132): DXCH LONGCADR / DTCB. Handled in
 * agc_waitlist_t3rupt, which dispatches LONGCADR in its place (so that
 * dispatch never calls itself); this entry exists for completeness.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_task_getcadr(void)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(pending_longcadr != AGC_TASK_NONE && pending_longcadr != AGC_TASK_GETCADR))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * T3RUPT / T3RUPT2 / TASKOVER (C pp.1231-1232 / L pp.1128-1129): TIME3 has
 * overflowed; dispatch task 1, move the lists up, reload TIME3 from LST1,
 * and repeat while the next task is due at the same time (RUPTAGN).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the first non-OK task status, else AGC_STATUS_OK; BAD_ARG
 *          on a failed precondition
 **********************************************************************/
agc_status_t agc_waitlist_t3rupt(void)
{
    agc_status_t result  = AGC_STATUS_OK;             /* first non-OK task status */
    uint16_t     gen     = exec_restart_generation(); /* STARTSB2 count at entry */
    bool         ruptagn = true;                      /* RUPTAGN: next task due now */

    if (!AGC_ASSERT(time3 < WL_TIME3_OVF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();

    for (int pass = 0; pass < AGC_WAITLIST_MAX_TASKS && ruptagn; pass++)
    {
        int32_t a = lst1[0]; /* LST1 +0: delta to the next task */

        for (int k = 0; k < WL_LST1_N - 1; k++)
        {
            lst1[k] = lst1[k + 1];
        }

        lst1[WL_LST1_N - 1] = (int16_t) WL_NEG1_2;             /* 81.91 s for ENDTASK */
        int32_t sum         = (int32_t) time3 + a + WL_POSMAX; /* ADS TIME3      */

        ruptagn            = sum >= WL_TIME3_OVF;                             /* RUPTAGN = +1     */
        time3              = (uint16_t) (ruptagn ? sum - WL_TIME3_OVF : sum); /* overflow leaves the excess in TIME3 */
        agc_task_id_t task = lst2[0];                                         /* LST2 +0: the task to run */

        for (int k = 0; k < AGC_WAITLIST_MAX_TASKS - 1; k++)
        {
            lst2[k] = lst2[k + 1];
        }

        lst2[AGC_WAITLIST_MAX_TASKS - 1] = AGC_TASK_ENDTASK;

        if (task == AGC_TASK_GETCADR)
        {
            task = pending_longcadr; /* GETCADR: DXCH LONGCADR */
        }

        agc_status_t       st  = agc_dispatch_task(task);     /* the task's own status */
        const agc_status_t sia = exec_service_interp_alarm(); /* 1301 / 1302 raised in the task */

        if (st == AGC_STATUS_OK)
        {
            st = sia; /* POODOO 1302 ended the task */
        }

        if (st != AGC_STATUS_OK && result == AGC_STATUS_OK)
        {
            result = st;
        }

        if (exec_restart_generation() != gen || agc_exec_restart_pending())
        {
            ruptagn = false; /* restart: the RUPTAGN chain is gone */
        }
    } /* for each task due now (RUPTAGN) */

    agc_relint();

    if (agc_exec_restart_pending())
    {
        /* BAILOUT/POODOO in a task: WHIMPER -> ENEMA after the RUPT. */
        agc_status_t st = agc_exec_service_restart(); /* ENEMA outcome */

        if (st != AGC_STATUS_OK && st != AGC_STATUS_BAILOUT)
        {
            return (st); /* a BAILOUT inside ENEMA is served at the next point */
        }
    }

    return (result);
} /* agc_waitlist_t3rupt */

/**********************************************************************
 * Sim-harness clock (no AGC routine: the TIME3 counter of the hardware,
 * WAITLIST.agc C p.1231): advance TIME3 by elapsed_cs and run every
 * T3RUPT that falls due.
 *
 * ARGUMENTS:
 *   elapsed_cs -- time to advance TIME3 by, cs
 *
 * RETURNS: agc_status_t -- the first non-OK T3RUPT status, else AGC_STATUS_OK; BAD_ARG
 *          on a failed precondition
 **********************************************************************/
agc_status_t agc_waitlist_tick(uint16_t elapsed_cs)
{
    if (!AGC_ASSERT(time3 < WL_TIME3_OVF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t result = AGC_STATUS_OK;        /* first non-OK T3RUPT status */
    int32_t      left   = (int32_t) elapsed_cs; /* cs still to advance */

    for (int i = 0; i < WL_TICK_LIMIT && left > 0; i++)
    {
        int32_t to_ovf = WL_TIME3_OVF - (int32_t) time3; /* cs until TIME3 overflows */

        if (left < to_ovf)
        {
            time3 = (uint16_t) ((int32_t) time3 + left);
            left  = 0;
        }
        else
        {
            left            -= to_ovf;
            time3           = 0;                     /* overflow -> T3RUPT */
            agc_status_t st = agc_waitlist_t3rupt(); /* T3RUPT outcome */

            if (st != AGC_STATUS_OK && result == AGC_STATUS_OK)
            {
                result = st;
            }
        }
    }

    if (!AGC_ASSERT(left == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (result);
}

/**********************************************************************
 * TIME3 (WAITLIST.agc C p.1226 / L p.1122): 16384 - (T1 - T) cs.
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint16_t -- C(TIME3), cs; 0 on a failed precondition
 **********************************************************************/
uint16_t agc_waitlist_time3(void)
{
    return (time3);
}

/**********************************************************************
 * AGC: none -- Inspection: entry i (0..8) of the list. *lst1 receives LST1 +i-1, the
 * delta that precedes task i+1 (for i = 0, -(T1 - T) + 1 from TIME3).
 *
 * ARGUMENTS:
 *   i -- list entry, 0..8
 *   lst1_out -- receives the delta that precedes task i+1, cs
 *   lst2_out -- receives task i+1 (LST2 +2i)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG on a bad index or NULL pointer
 **********************************************************************/
agc_status_t agc_waitlist_entry(int i, int16_t *lst1_out, agc_task_id_t *lst2_out)
{
    if (i == 0)
    {
        *lst1_out = (int16_t) ((int32_t) time3 - WL_TIME3_OVF + 1);
    }
    else
    {
        *lst1_out = lst1[i - 1];
    }

    *lst2_out = lst2[i];

    return (AGC_STATUS_OK);
}
