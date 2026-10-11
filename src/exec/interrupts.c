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
 * reached through agc_dispatch_rupt() at the end of this file (LAYERING:
 * like dispatch.c, the one place exec calls the handlers' layers).
 *
 * INHINT / RELINT are counted, so that nested C calls (a task calling
 * WAITLIST, which itself inhibits) keep interrupts inhibited until the
 * outermost RELINT. In a single-threaded host they only document intent
 * and let services assert that they run in interrupt context.
 */
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"
#include "internal.h"

#define RUPT_MAX_DEPTH 64      /* bound on INHINT nesting          */
#define RUPT_TIME_MASK 077777u /* 15-bit counter words             */
#define RUPT_TIME4_SB  037775u /* STARTSUB: 37775 to TIME4         */
#define RUPT_TIME5_SB  037774u /* STARTSUB: 37774 to TIME5         */
#define RUPT_HALF_SEC  50      /* .5SEC DEC 50 (cs)                */

static int           inhint_depth;             /* > 0: interrupts inhibited        */
static uint16_t      time4;                    /* TIME4 (T4RUPT every 10-120 ms)   */
static uint16_t      time5;                    /* TIME5 (T5RUPT, autopilot)        */
static uint16_t      time6;                    /* TIME6 (T6RUPT, LM jet timing)    */
static agc_task_id_t t5loc = AGC_TASK_T5IDLOC; /* T5LOC / T5ADR          */
static agc_task_id_t t6loc = AGC_TASK_NONE;    /* T6LOC (CSM)            */

void agc_inhint(void)
{
    if (AGC_ASSERT(inhint_depth < RUPT_MAX_DEPTH))
    {
        inhint_depth++;
    }

    if (!AGC_ASSERT(inhint_depth > 0))
    {
        return;
    }
}

void agc_relint(void)
{
    if (AGC_ASSERT(inhint_depth > 0))
    {
        inhint_depth--;
    }

    if (!AGC_ASSERT(inhint_depth >= 0))
    {
        return;
    }
}

bool agc_interrupts_inhibited(void)
{
    if (!AGC_ASSERT(inhint_depth >= 0))
    {
        return (false);
    }

    if (!AGC_ASSERT(inhint_depth <= RUPT_MAX_DEPTH))
    {
        return (false);
    }

    return (inhint_depth > 0);
}

/* STARTSUB / STARTSB1 (FRESH_START C p.190 / L p.220): TIME4 = 37775,
 * TIME5 = 37774 (TIME3 = 37777 is set by the waitlist).               */
void rupt_reset_timers(void)
{
    time4 = RUPT_TIME4_SB;
    time5 = RUPT_TIME5_SB;

    if (!AGC_ASSERT(time4 <= RUPT_TIME_MASK))
    {
        return;
    }

    if (!AGC_ASSERT(time5 <= RUPT_TIME_MASK))
    {
        return;
    }
}

uint16_t agc_rupt_time(int n)
{
    if (!AGC_ASSERT(n >= 4 && n <= 6))
    {
        return (0);
    }

    uint16_t v = (n == 4) ? time4 : ((n == 5) ? time5 : time6);

    if (!AGC_ASSERT(v <= RUPT_TIME_MASK))
    {
        return (0);
    }

    return (v);
}

void agc_rupt_set_time(int n, uint16_t v)
{
    if (!AGC_ASSERT(n >= 4 && n <= 6) || !AGC_ASSERT(v <= RUPT_TIME_MASK))
    {
        return;
    }

    if (n == 4)
    {
        time4 = v;
    }
    else if (n == 5)
    {
        time5 = v;
    }
    else
    {
        time6 = v;
    }

    if (!AGC_ASSERT(agc_rupt_time(n) == v))
    {
        return;
    }
}

/* DXCH T5LOC (CSM) / T5ADR (LM): the program the next T5RUPT runs.    */
void agc_rupt_set_t5_task(agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE))
    {
        return;
    }

    t5loc = task;

    if (!AGC_ASSERT(t5loc == task))
    {
        return;
    }
}

agc_task_id_t agc_rupt_t5_task(void)
{
    if (!AGC_ASSERT(t5loc != AGC_TASK_NONE))
    {
        return (AGC_TASK_NONE);
    }

    if (!AGC_ASSERT(inhint_depth >= 0))
    {
        return (AGC_TASK_NONE);
    }

    return (t5loc);
}

/* T6LOC (CSM only; the LM T6RUPT always goes to DOT6RUPT).            */
void agc_rupt_set_t6_task(agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE) || !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return;
    }

    t6loc = task;
}

/* RESUME: leave the interrupt; run a software restart it requested.   */
static agc_status_t rupt_exit(agc_status_t st)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_relint();

    if (agc_exec_restart_pending())
    {
        agc_status_t rs = agc_exec_service_restart();

        if (!AGC_ASSERT(rs != AGC_STATUS_BAD_ARG))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (st);
}

/* Vector 4000: GOJAM (hardware restart) -> GOPROG.                    */
agc_status_t agc_rupt_go(void)
{
    inhint_depth = 0; /* GOJAM clears the RUPT state   */
    agc_inhint();

    if (!AGC_ASSERT(inhint_depth == 1))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_restart_goprog();

    if (!AGC_ASSERT(inhint_depth == 1))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    return (rupt_exit(st));
}

/* T6RUPT: CSM DTCB through T6LOC, LM through T6ADR = DOT6RUPT.        */
agc_status_t agc_rupt_t6(void)
{
    agc_inhint();
    agc_status_t st = AGC_STATUS_NOT_FOUND;

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        st = agc_dispatch_rupt(AGC_RUPT_T6);
    }
    else if (t6loc != AGC_TASK_NONE)
    {
        st = agc_dispatch_task(t6loc);
    }
    else
    {
        if (!AGC_ASSERT(t6loc == AGC_TASK_NONE)) /* no T6 program   */
        {
            agc_relint();

            return (AGC_STATUS_BAD_ARG);
        }
    }

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    return (rupt_exit(st));
}

/* T5RUPT: CSM tests TIME5 then DTCB T5LOC; LM DTCB T5ADR.             */
agc_status_t agc_rupt_t5(void)
{
    agc_inhint();
    agc_status_t st       = AGC_STATUS_OK;
    bool         spurious = (agc_state_vehicle() == AGC_VEHICLE_CSM) && (RUPT_HALF_SEC - (int32_t) time5 <= 0);

    if (!AGC_ASSERT(time5 <= RUPT_TIME_MASK))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    if (!spurious)
    { /* CSM: BZMF NOQBRSM              */
        st = agc_dispatch_task(t5loc);
    }

    if (!AGC_ASSERT(t5loc != AGC_TASK_NONE))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    return (rupt_exit(st));
}

/* T3RUPT: the waitlist.                                               */
agc_status_t agc_rupt_t3(void)
{
    agc_inhint();

    if (!AGC_ASSERT(agc_waitlist_time3() < 040000u))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_waitlist_t3rupt();

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    return (rupt_exit(st));
}

/* Lead-ins whose handlers belong to other lanes (dispatch.c).         */
static agc_status_t foreign_rupt(agc_rupt_id_t id)
{
    if (!AGC_ASSERT(id >= AGC_RUPT_T6 && id <= AGC_RUPT_HAND))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_status_t st = agc_dispatch_rupt(id);

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    return (rupt_exit(st));
}

agc_status_t agc_rupt_t4(void)
{
    return (foreign_rupt(AGC_RUPT_T4));
}

agc_status_t agc_rupt_keyrupt1(void)
{
    return (foreign_rupt(AGC_RUPT_KEY1));
}

agc_status_t agc_rupt_keyrupt2(void)
{
    return (foreign_rupt(AGC_RUPT_KEY2));
}

agc_status_t agc_rupt_uprupt(void)
{
    return (foreign_rupt(AGC_RUPT_UPRUPT));
}

agc_status_t agc_rupt_downrupt(void)
{
    return (foreign_rupt(AGC_RUPT_DOWNRUPT));
}

agc_status_t agc_rupt_radarupt(void)
{
    return (foreign_rupt(AGC_RUPT_RADAR));
}

agc_status_t agc_rupt_rupt10(void)
{
    return (foreign_rupt(AGC_RUPT_HAND));
}

/* T5IDLOC (FRESH_START C p.192): the CSM's idle T5RUPT program, every
 * 163.84 s when no autopilot uses TIME5: just resume.                 */
agc_status_t agc_task_t5idloc(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* Interrupt handlers owned by other lanes (INTERRUPT_LEAD_INS).        */
agc_status_t agc_dispatch_rupt(agc_rupt_id_t rupt)
{
    bool         lm = agc_state_vehicle() == AGC_VEHICLE_LM;
    agc_status_t st = AGC_STATUS_UNIMPLEMENTED;

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    switch (rupt)
    {
        case AGC_RUPT_T6:       /* LM DOT6RUPT; CSM goes through T6LOC     */
            st = lm ? agc_lm_dap_t6rupt() : AGC_STATUS_NOT_FOUND;
            break;
        case AGC_RUPT_T5:       /* T5LOC / T5ADR                           */
            st = agc_dispatch_task(agc_rupt_t5_task());
            break;
        case AGC_RUPT_T3:
            st = agc_waitlist_t3rupt();
            break;
        case AGC_RUPT_KEY1:
            st = agc_dsky_keyrupt1();
            break;
        case AGC_RUPT_KEY2:     /* MARKRUPT: CSM sextant, LM AOT           */
            st = lm ? agc_aotmark_markrupt() : agc_sxtmark_markrupt();
            break;
        case AGC_RUPT_HAND:     /* LM PITFALL; CSM RESUME +3 (not used)    */
            st = lm ? agc_lm_pitfall() : AGC_STATUS_OK;
            break;
        case AGC_RUPT_T4:       /* T4RUPT (T4RUPT_PROGRAM), reloads TIME4    */
            st = agc_t4rupt_rupt();
            break;
        case AGC_RUPT_UPRUPT:   /* UPRUPT: the uplinked word in INLINK      */
            st = agc_dsky_uprupt(agc_erase_inlink());
            break;
        case AGC_RUPT_DOWNRUPT: /* DODOWNTM: next downlink word pair      */
            st = agc_telem_downrupt();
            break;
        case AGC_RUPT_RADAR:    /* CSM VHFREAD (VHF range) / LM RADAREAD     */
            st = lm ? agc_rendz_radar_read() : agc_rendz_vhf_read();
            break;
        default:
            st = AGC_STATUS_BAD_ARG;
            break;
    }

    if (!AGC_ASSERT(rupt >= AGC_RUPT_T6 && rupt <= AGC_RUPT_HAND))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}
