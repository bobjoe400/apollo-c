/*
 * waitlist.c -- WAITLIST, TWIDDLE, VARDELAY, FIXDELAY, T3RUPT, TASKOVER,
 *               ENDTASK/SVCT3 and LONGCALL.
 *
 * Source: WAITLIST.agc, Comanche055 pp. 1221-1235, Luminary099 pp. 1117-1132.
 * Differences between the listings:
 *   - Comanche tests DT <= 0 at WAITLIST (TWIDDLE falls into it) and
 *     POODOOs 1204; Luminary tests at WAIT2, i.e. also for VARDELAY and
 *     FIXDELAY, and also tests the LONGCALL delta (LONGPOOH). Both 1204.
 *   - overflow: Comanche TC BAILOUT / OCT 1203; Luminary TC FILLED ->
 *     BAILOUT1 / OCT 01203 (records the caller). Same code.
 *
 * The delta-time model is kept exactly (DTMAX 162.5 s, 9 tasks):
 *     C(TIME3)    = 16384 - (T1 - T)     centiseconds, T = now
 *     C(LST1 +k)  = -(T(k+2) - T(k+1)) + 1          k = 0..7
 *     C(LST2 +k)  = task k+1                        k = 0..8
 * An empty slot holds ENDTASK in LST2 and NEG1/2 (-8192) in LST1, so with
 * nothing scheduled ENDTASK (SVCT3) runs every 8193 cs = 81.93 s.
 * TIME3 is a 14-bit counter that overflows at 16384 cs; the overflow is
 * T3RUPT. TIME1/TIME2 are not advanced here (the sim harness owns them).
 */
#include "agc/agc_exec.h"
#include "agc/agc_assert.h"
#include "agc/agc_state.h"
#include "agc/agc_flags_common.h"
#include "exec_internal.h"

#define WL_TIME3_OVF   16384      /* TIME3 overflow, 2^14 cs = 163.84 s   */
#define WL_POSMAX      16383      /* POSMAX, octal 37777                  */
#define WL_NEG1_2      (-8192)    /* NEG1/2: empty LST1 slot              */
#define WL_LST1_N      8          /* LST1 ... LST1 +7                     */
#define WL_BIT14       8192       /* BIT14 / DPBIT14 low word, 81.92 s    */
#define WL_SVCT3_DELAY 500        /* SVCT3X: DEC 500 (5 s)                */
#define WL_TICK_LIMIT  65536      /* bound for agc_waitlist_tick loop     */

static uint16_t      time3;                       /* TIME3              */
static int16_t       lst1[WL_LST1_N];             /* LST1               */
static agc_task_id_t lst2[AGC_WAITLIST_MAX_TASKS];/* LST2 (task ids)    */
static uint32_t      longtime;                    /* LONGTIME B(2) cs   */
static agc_task_id_t longcadr = AGC_TASK_NONE;    /* LONGCADR           */

/* STARTSB2 (FRESH_START_AND_RESTART C p.190 / L p.220): initialise the
 * waitlist delta-Ts to NEG1/2 and every LST2 entry to ENDTASK.          */
void waitlist_clear_lists(void)
{
    for (int k = 0; k < WL_LST1_N; k++) {
        lst1[k] = (int16_t)WL_NEG1_2;
    }
    for (int k = 0; k < AGC_WAITLIST_MAX_TASKS; k++) {
        lst2[k] = AGC_TASK_ENDTASK;
    }
    (void)AGC_ASSERT(lst1[0] == WL_NEG1_2);
    (void)AGC_ASSERT(lst2[AGC_WAITLIST_MAX_TASKS - 1] == AGC_TASK_ENDTASK);
}

/* STARTSUB / STARTSB1: CAF POSMAX / TS TIME3 (T3RUPT 1 cs later).      */
void waitlist_reset_time3(void)
{
    time3 = (uint16_t)WL_POSMAX;
    (void)AGC_ASSERT(time3 < WL_TIME3_OVF);
    (void)AGC_ASSERT(time3 > 0);
}

agc_status_t agc_waitlist_init(void)
{
    waitlist_reset_time3();
    waitlist_clear_lists();
    longtime = 0;
    longcadr = AGC_TASK_NONE;
    if (!AGC_ASSERT(lst2[0] == AGC_TASK_ENDTASK)) {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* WTLST4 + WTLST2 (C pp.1227,1230 / L pp.1123,1127): insert `task` as
 * entry n (0..8) of LST2 and `delta` as LST1 +n, pushing the later
 * entries down. The entry pushed off the end of LST2 must be ENDTASK,
 * otherwise the list is full (WTABORT -> 1203).                       */
static agc_status_t wtlst4(int n, int32_t delta, agc_task_id_t task)
{
    if (!AGC_ASSERT(n >= 0 && n < AGC_WAITLIST_MAX_TASKS)) {
        return AGC_STATUS_BAD_ARG;
    }
    /* DEVIATION: the AGC shifts first and checks the ejected LST2 word
     * afterwards; checking first is equivalent because the BAILOUT that
     * follows re-initialises both lists in STARTSB2.                   */
    if (lst2[AGC_WAITLIST_MAX_TASKS - 1] != AGC_TASK_ENDTASK) {
        return agc_bailout(AGC_ALARM_01203);    /* NO ROOM IN THE INN  */
    }
    if (n < WL_LST1_N) {                         /* INDEX Q / TCF WTLST4 */
        if (!AGC_ASSERT(delta >= -WL_POSMAX && delta <= 1)) {
            return AGC_STATUS_BAD_ARG;
        }
        for (int k = WL_LST1_N - 1; k > n; k--) {
            lst1[k] = lst1[k - 1];               /* XCH LST1 +k chain   */
        }
        lst1[n] = (int16_t)delta;
    }
    for (int k = AGC_WAITLIST_MAX_TASKS - 1; k > n; k--) {
        lst2[k] = lst2[k - 1];                   /* DXCH LST2 +2k chain */
    }
    lst2[n] = task;
    return AGC_STATUS_OK;
}

/* WAIT2 / WTLST5 / WTLST2 (C pp.1226-1230 / L pp.1122-1127): task
 * insertion. TD = T + dt is the desired time; T(n) the time of task n. */
static agc_status_t wait2(uint16_t dt, agc_task_id_t task)
{
    if (!AGC_ASSERT(dt >= 1 && dt <= AGC_WAITLIST_MAX_DELAY) ||
        !AGC_ASSERT(time3 < WL_TIME3_OVF)) {
        return AGC_STATUS_BAD_ARG;
    }
    /* DEVIATION: the "TIME3 overflow already happened" branch (C(TIME3)
     * < 200 octal) cannot arise: T3RUPT runs synchronously at overflow.  */
    int32_t a = (int32_t)dt - (WL_TIME3_OVF - (int32_t)time3) + 1; /* TD-T1+1 */
    if (a <= 0) {
        /* TD before T1: new task 1. TIME3 = 1.0 - DELTA T, and the old
         * task 1 gets LST1 = -(T1 - TD) + 1, which equals a.             */
        time3 = (uint16_t)(WL_TIME3_OVF - (int32_t)dt);
        return wtlst4(0, a, task);
    }
    a = a - 1 + lst1[0];                         /* TD - T2 + 1         */
    for (int n = 1; n < AGC_WAITLIST_MAX_TASKS; n++) {
        if (a <= 0) {
            /* WTLST2: T(n) <= TD < T(n+1). LST1 +n-1 becomes
             * -(TD - T(n)) + 1 and -(T(n+1) - TD) + 1 = a is inserted.  */
            int32_t prev = (int32_t)lst1[n - 1] - a + 1;
            if (!AGC_ASSERT(prev <= 1 && prev >= -WL_POSMAX)) {
                return AGC_STATUS_BAD_ARG;
            }
            agc_status_t st = wtlst4(n, a, task);
            if (st == AGC_STATUS_OK) {
                lst1[n - 1] = (int16_t)prev;
            }
            return st;
        }
        if (n < WL_LST1_N) {
            a = a - 1 + lst1[n];                 /* TD - T(n+2) + 1     */
        }
    }
    return agc_bailout(AGC_ALARM_01203);         /* WTABORT              */
}

/* WAITLIST / TWIDDLE (C p.1223 / L p.1119): call `task` in dt centiseconds
 * (1 <= dt <= 16250). dt = 0 is POODOO 1204 (WATLST0- / WAITPOOH).     */
agc_status_t agc_waitlist_call(uint16_t delay_cs, agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (delay_cs == 0) {
        return agc_poodoo(AGC_ALARM_01204);
    }
    if (!AGC_ASSERT(delay_cs <= AGC_WAITLIST_MAX_DELAY)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    agc_status_t st = wait2(delay_cs, task);
    agc_relint();
    return st;
}

/* VARDELAY (C p.1223 / L p.1119): called from a task, which then ends;
 * the task continues as `task` after dt cs. Comanche checks dt <= 0 only
 * in WAITLIST.
 * DEVIATION: dt = 0 here POODOOs 1204 for both vehicles (Luminary
 * behaviour); in Comanche it would corrupt TIME3.                      */
agc_status_t agc_waitlist_vardelay(uint16_t delay_cs, agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_interrupts_inhibited())) {
        return AGC_STATUS_BAD_ARG;              /* must be in a task    */
    }
    return agc_waitlist_call(delay_cs, task);
}

/* FIXDELAY (C p.1223 / L p.1119): as VARDELAY with the delay in the word
 * after the call; in C the caller passes it.                           */
agc_status_t agc_waitlist_fixdelay(uint16_t delay_cs, agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(delay_cs <= AGC_WAITLIST_MAX_DELAY)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_waitlist_vardelay(delay_cs, task);
}

/* LONGCYCL / MUCHTIME / LASTTIME (C pp.1234-1235 / L pp.1131-1132): take
 * 81.92 s off LONGTIME; if time remains, come back in 81.92 s, otherwise
 * schedule GETCADR for the remainder.                                  */
static agc_status_t longcycl(void)
{
    if (!AGC_ASSERT(longcadr != AGC_TASK_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    int64_t rem = (int64_t)longtime - WL_BIT14;  /* DCS DPBIT14 / DAS    */
    if (rem > 0) {
        longtime = (uint32_t)rem;
        return agc_waitlist_call((uint16_t)WL_BIT14, AGC_TASK_LONGCYCL);
    }
    rem += WL_BIT14;                             /* LASTTIME: ADS BIT14 */
    if (!AGC_ASSERT(rem >= 0 && rem <= WL_BIT14)) {
        return AGC_STATUS_BAD_ARG;
    }
    longtime = (uint32_t)rem;
    return agc_waitlist_call((uint16_t)rem, AGC_TASK_GETCADR);
}

/* LONGCALL / LNGCALL2 (C pp.1233-1234 / L pp.1130-1131): call `task` after
 * a DP delay in centiseconds (scaled as TIME2,TIME1). Only one LONGCALL
 * can be pending (LONGTIME, LONGCADR are single registers).            */
agc_status_t agc_waitlist_longcall(uint32_t delay_cs, agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE) ||
        !AGC_ASSERT(delay_cs < AGC_TIME_MODULUS_CS)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (delay_cs == 0 && agc_state_vehicle() == AGC_VEHICLE_LM) {
        return agc_poodoo(AGC_ALARM_01204);      /* LONGPOOH            */
    }
    agc_inhint();
    longtime = delay_cs;                         /* DXCH LONGTIME       */
    longcadr = task;                             /* DXCH LONGCADR       */
    agc_status_t st = longcycl();
    agc_relint();
    return st;
}

/* ENDTASK -> SVCT3 / CKIMUSE / SVCT3X (C p.1225 / L p.1121): every 81.93 s
 * when the list is empty. With DRIFTFLG set start NBDONLY (IMU drift
 * compensation) unless someone is using the IMU, in which case retry in
 * 5 s.                                                                 */
static agc_status_t svct3(void)
{
    if (!agc_flag_test(AGC_FLAG_DRIFTFLG)) {
        return AGC_STATUS_OK;                    /* TCF TASKOVER        */
    }
    if (agc_erase_imucadr() != 0) {              /* CCS IMUCADR         */
        return agc_waitlist_fixdelay(WL_SVCT3_DELAY, AGC_TASK_SVCT3);
    }
    agc_status_t st = agc_exec_novac(EXEC_PRIO35, AGC_JOB_NBDONLY);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BAILOUT);
    return st;
}

/* The waitlist's own tasks (called from agc_dispatch_task).            */
agc_status_t agc_task_endtask(void)
{
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    (void)AGC_ASSERT(lst2[0] != AGC_TASK_NONE);
    return svct3();
}

agc_status_t agc_task_svct3(void)
{
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    (void)AGC_ASSERT(lst2[0] != AGC_TASK_NONE);
    return svct3();
}

agc_status_t agc_task_longcycl(void)
{
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    (void)AGC_ASSERT(longcadr != AGC_TASK_NONE);
    return longcycl();
}

/* GETCADR (C p.1235 / L p.1132): DXCH LONGCADR / DTCB. Handled in
 * agc_waitlist_t3rupt, which dispatches LONGCADR in its place (so that
 * dispatch never calls itself); this entry exists for completeness.    */
agc_status_t agc_task_getcadr(void)
{
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    if (!AGC_ASSERT(longcadr != AGC_TASK_NONE && longcadr != AGC_TASK_GETCADR)) {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* T3RUPT / T3RUPT2 / TASKOVER (C pp.1231-1232 / L pp.1128-1129): TIME3 has
 * overflowed; dispatch task 1, move the lists up, reload TIME3 from LST1,
 * and repeat while the next task is due at the same time (RUPTAGN).    */
agc_status_t agc_waitlist_t3rupt(void)
{
    agc_status_t result = AGC_STATUS_OK;
    uint16_t gen = exec_restart_generation();
    bool ruptagn = true;
    agc_inhint();
    for (int pass = 0; pass < AGC_WAITLIST_MAX_TASKS && ruptagn; pass++) {
        int32_t a = lst1[0];
        for (int k = 0; k < WL_LST1_N - 1; k++) {
            lst1[k] = lst1[k + 1];
        }
        lst1[WL_LST1_N - 1] = (int16_t)WL_NEG1_2;   /* 81.91 s for ENDTASK */
        int32_t sum = (int32_t)time3 + a + WL_POSMAX; /* ADS TIME3      */
        ruptagn = sum >= WL_TIME3_OVF;               /* RUPTAGN = +1     */
        time3 = (uint16_t)(ruptagn ? sum - WL_TIME3_OVF : sum);
        agc_task_id_t task = lst2[0];
        for (int k = 0; k < AGC_WAITLIST_MAX_TASKS - 1; k++) {
            lst2[k] = lst2[k + 1];
        }
        lst2[AGC_WAITLIST_MAX_TASKS - 1] = AGC_TASK_ENDTASK;
        if (task == AGC_TASK_GETCADR) {
            task = longcadr;                         /* GETCADR: DXCH LONGCADR */
        }
        agc_status_t st = agc_dispatch_task(task);
        if (st != AGC_STATUS_OK && result == AGC_STATUS_OK) {
            result = st;
        }
        if (exec_restart_generation() != gen) {
            ruptagn = false;     /* restart: the RUPTAGN chain is gone */
        }
    }
    agc_relint();
    return result;
}

/* Sim-harness clock: advance TIME3 by elapsed_cs and run every T3RUPT
 * that falls due. Returns the first non-OK task status.                */
agc_status_t agc_waitlist_tick(uint16_t elapsed_cs)
{
    if (!AGC_ASSERT(time3 < WL_TIME3_OVF)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t result = AGC_STATUS_OK;
    int32_t left = (int32_t)elapsed_cs;
    for (int i = 0; i < WL_TICK_LIMIT && left > 0; i++) {
        int32_t to_ovf = WL_TIME3_OVF - (int32_t)time3;
        if (left < to_ovf) {
            time3 = (uint16_t)((int32_t)time3 + left);
            left = 0;
        } else {
            left -= to_ovf;
            time3 = 0;                               /* overflow -> T3RUPT */
            agc_status_t st = agc_waitlist_t3rupt();
            if (st != AGC_STATUS_OK && result == AGC_STATUS_OK) {
                result = st;
            }
        }
    }
    (void)AGC_ASSERT(left == 0);
    return result;
}

uint16_t agc_waitlist_time3(void)
{
    (void)AGC_ASSERT(time3 < WL_TIME3_OVF);
    (void)AGC_ASSERT(lst2[AGC_WAITLIST_MAX_TASKS - 1] != AGC_TASK_NONE);
    return time3;
}

/* Inspection: entry i (0..8) of the list. *lst1 receives LST1 +i-1, the
 * delta that precedes task i+1 (for i = 0, -(T1 - T) + 1 from TIME3).   */
agc_status_t agc_waitlist_entry(int i, int16_t *lst1_out, agc_task_id_t *lst2_out)
{
    if (!AGC_ASSERT(i >= 0 && i < AGC_WAITLIST_MAX_TASKS) ||
        !AGC_ASSERT(lst1_out != 0 && lst2_out != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (i == 0) {
        *lst1_out = (int16_t)((int32_t)time3 - WL_TIME3_OVF + 1);
    } else {
        *lst1_out = lst1[i - 1];
    }
    *lst2_out = lst2[i];
    return AGC_STATUS_OK;
}
