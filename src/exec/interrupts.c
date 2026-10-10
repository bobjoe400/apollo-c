/*
 * interrupts.c -- INHINT / RELINT and the interrupt lead-ins.
 *
 * Source: INTERRUPT_LEAD_INS.agc, Comanche055 pp. 131-132, Luminary099
 * pp. 153-154. At location 4000 the AGC has one 4-word vector per
 * interrupt; each saves A,L (DXCH ARUPT), selects the handler's bank and
 * jumps. Differences:
 *   - T6RUPT: Comanche jumps through the erasable 2CADR T6LOC, Luminary
 *     through the fixed T6ADR = DOT6RUPT (the LM DAP jet timer);
 *   - T5RUPT: Comanche first checks TIME5 (CS TIME5 / AD .5SEC / BZMF
 *     NOQBRSM: a T5RUPT with TIME5 >= 50 cs is ignored) and jumps through
 *     T5LOC; Luminary jumps through T5ADR;
 *   - RADAR RUPT: Comanche VHFREAD, Luminary RADAREAD;
 *   - RUPT10 (hand controller): Comanche RESUME +3 (not used), Luminary
 *     PITFALL.
 *
 * Here each vector is a function the sim harness calls when the counter
 * overflows or the input arrives. It runs with interrupts inhibited, as
 * an AGC interrupt does, and on the way out (RESUME) services a software
 * restart that the handler requested. Handlers owned by other lanes are
 * reached through agc_dispatch_rupt() in dispatch.c.
 *
 * INHINT / RELINT are counted, so that nested C calls (a task calling
 * WAITLIST, which itself inhibits) keep interrupts inhibited until the
 * outermost RELINT. In a single-threaded host they only document intent
 * and let services assert that they run in interrupt context.
 */
#include "agc/agc_exec.h"
#include "agc/agc_assert.h"
#include "agc/agc_state.h"
#include "exec_internal.h"

#define RUPT_MAX_DEPTH 64            /* bound on INHINT nesting          */
#define RUPT_TIME_MASK 077777u       /* 15-bit counter words             */
#define RUPT_TIME4_SB  037775u       /* STARTSUB: 37775 to TIME4         */
#define RUPT_TIME5_SB  037774u       /* STARTSUB: 37774 to TIME5         */
#define RUPT_HALF_SEC  50            /* .5SEC DEC 50 (cs)                */

static int           inhint_depth;   /* > 0: interrupts inhibited        */
static uint16_t      time4;          /* TIME4 (T4RUPT every 10-120 ms)   */
static uint16_t      time5;          /* TIME5 (T5RUPT, autopilot)        */
static uint16_t      time6;          /* TIME6 (T6RUPT, LM jet timing)    */
static agc_task_id_t t5loc = AGC_TASK_T5IDLOC; /* T5LOC / T5ADR          */
static agc_task_id_t t6loc = AGC_TASK_NONE;    /* T6LOC (CSM)            */

void agc_inhint(void)
{
    if (AGC_ASSERT(inhint_depth < RUPT_MAX_DEPTH)) {
        inhint_depth++;
    }
    (void)AGC_ASSERT(inhint_depth > 0);
}

void agc_relint(void)
{
    if (AGC_ASSERT(inhint_depth > 0)) {
        inhint_depth--;
    }
    (void)AGC_ASSERT(inhint_depth >= 0);
}

bool agc_interrupts_inhibited(void)
{
    (void)AGC_ASSERT(inhint_depth >= 0);
    (void)AGC_ASSERT(inhint_depth <= RUPT_MAX_DEPTH);
    return inhint_depth > 0;
}

/* STARTSUB / STARTSB1 (FRESH_START C p.190 / L p.220): TIME4 = 37775,
 * TIME5 = 37774 (TIME3 = 37777 is set by the waitlist).               */
void rupt_reset_timers(void)
{
    time4 = RUPT_TIME4_SB;
    time5 = RUPT_TIME5_SB;
    (void)AGC_ASSERT(time4 <= RUPT_TIME_MASK);
    (void)AGC_ASSERT(time5 <= RUPT_TIME_MASK);
}

uint16_t agc_rupt_time(int n)
{
    if (!AGC_ASSERT(n >= 4 && n <= 6)) {
        return 0;
    }
    uint16_t v = (n == 4) ? time4 : ((n == 5) ? time5 : time6);
    (void)AGC_ASSERT(v <= RUPT_TIME_MASK);
    return v;
}

agc_status_t agc_rupt_set_time(int n, uint16_t v)
{
    if (!AGC_ASSERT(n >= 4 && n <= 6) ||
        !AGC_ASSERT(v <= RUPT_TIME_MASK)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (n == 4) {
        time4 = v;
    } else if (n == 5) {
        time5 = v;
    } else {
        time6 = v;
    }
    (void)AGC_ASSERT(agc_rupt_time(n) == v);
    return AGC_STATUS_OK;
}

/* DXCH T5LOC (CSM) / T5ADR (LM): the program the next T5RUPT runs.    */
agc_status_t agc_rupt_set_t5_task(agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    t5loc = task;
    (void)AGC_ASSERT(t5loc == task);
    return AGC_STATUS_OK;
}

agc_task_id_t agc_rupt_t5_task(void)
{
    (void)AGC_ASSERT(t5loc != AGC_TASK_NONE);
    (void)AGC_ASSERT(inhint_depth >= 0);
    return t5loc;
}

/* T6LOC (CSM only; the LM T6RUPT always goes to DOT6RUPT).            */
agc_status_t agc_rupt_set_t6_task(agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE) ||
        !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM)) {
        return AGC_STATUS_BAD_ARG;
    }
    t6loc = task;
    return AGC_STATUS_OK;
}

/* RESUME: leave the interrupt; run a software restart it requested.   */
static agc_status_t rupt_exit(agc_status_t st)
{
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    agc_relint();
    if (agc_exec_restart_pending()) {
        agc_status_t rs = agc_exec_service_restart();
        (void)AGC_ASSERT(rs != AGC_STATUS_BAD_ARG);
    }
    return st;
}

/* Vector 4000: GOJAM (hardware restart) -> GOPROG.                    */
agc_status_t agc_rupt_go(void)
{
    inhint_depth = 0;                 /* GOJAM clears the RUPT state   */
    agc_inhint();
    (void)AGC_ASSERT(inhint_depth == 1);
    agc_status_t st = agc_restart_goprog();
    (void)AGC_ASSERT(inhint_depth == 1);
    return rupt_exit(st);
}

/* T6RUPT: CSM DTCB through T6LOC, LM through T6ADR = DOT6RUPT.        */
agc_status_t agc_rupt_t6(void)
{
    agc_inhint();
    agc_status_t st = AGC_STATUS_NOT_FOUND;
    if (agc_state_vehicle() == AGC_VEHICLE_LM) {
        st = agc_dispatch_rupt(AGC_RUPT_T6);
    } else if (t6loc != AGC_TASK_NONE) {
        st = agc_dispatch_task(t6loc);
    } else {
        (void)AGC_ASSERT(t6loc == AGC_TASK_NONE);  /* no T6 program   */
    }
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    return rupt_exit(st);
}

/* T5RUPT: CSM tests TIME5 then DTCB T5LOC; LM DTCB T5ADR.             */
agc_status_t agc_rupt_t5(void)
{
    agc_inhint();
    agc_status_t st = AGC_STATUS_OK;
    bool spurious = (agc_state_vehicle() == AGC_VEHICLE_CSM) &&
                    (RUPT_HALF_SEC - (int32_t)time5 <= 0);
    (void)AGC_ASSERT(time5 <= RUPT_TIME_MASK);
    if (!spurious) {                  /* CSM: BZMF NOQBRSM              */
        st = agc_dispatch_task(t5loc);
    }
    (void)AGC_ASSERT(t5loc != AGC_TASK_NONE);
    return rupt_exit(st);
}

/* T3RUPT: the waitlist.                                               */
agc_status_t agc_rupt_t3(void)
{
    agc_inhint();
    (void)AGC_ASSERT(agc_waitlist_time3() < 040000u);
    agc_status_t st = agc_waitlist_t3rupt();
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    return rupt_exit(st);
}

/* Lead-ins whose handlers belong to other lanes (dispatch.c).         */
static agc_status_t foreign_rupt(agc_rupt_id_t id)
{
    if (!AGC_ASSERT(id >= AGC_RUPT_T6 && id <= AGC_RUPT_HAND)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    agc_status_t st = agc_dispatch_rupt(id);
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    return rupt_exit(st);
}

agc_status_t agc_rupt_t4(void)       { return foreign_rupt(AGC_RUPT_T4); }
agc_status_t agc_rupt_keyrupt1(void) { return foreign_rupt(AGC_RUPT_KEY1); }
agc_status_t agc_rupt_keyrupt2(void) { return foreign_rupt(AGC_RUPT_KEY2); }
agc_status_t agc_rupt_uprupt(void)   { return foreign_rupt(AGC_RUPT_UPRUPT); }
agc_status_t agc_rupt_downrupt(void) { return foreign_rupt(AGC_RUPT_DOWNRUPT); }
agc_status_t agc_rupt_radarupt(void) { return foreign_rupt(AGC_RUPT_RADAR); }
agc_status_t agc_rupt_rupt10(void)   { return foreign_rupt(AGC_RUPT_HAND); }

/* T5IDLOC (FRESH_START C p.192): the CSM's idle T5RUPT program, every
 * 163.84 s when no autopilot uses TIME5: just resume.                 */
agc_status_t agc_task_t5idloc(void)
{
    (void)AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM);
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    return AGC_STATUS_OK;
}
