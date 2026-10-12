/**********************************************************************
 * FILE NAME: interrupts.c
 *
 * PURPOSE:
 *   INHINT / RELINT, the TIME4 / TIME5 / TIME6 counter words, T5LOC /
 *   T5ADR and T6LOC, and the interrupt lead-ins (vector 4000 GOJAM,
 *   T6RUPT, T5RUPT, T3RUPT, T4RUPT, KEYRUPT1/2, UPRUPT, DOWNRUPT, RADAR
 *   RUPT, RUPT10) with the RESUME that services a requested restart.
 *
 * AGC SOURCE:
 *   Comanche055 INTERRUPT_LEAD_INS.agc pp. 131-132
 *   Luminary099 INTERRUPT_LEAD_INS.agc pp. 153-154 (differs: see NOTES)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   TIME4/5/6     uint16_t  I/O  cs / 1/1600 s  counter words (here)
 *   INLINK        uint16_t  I    word           UPRUPT input
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_restart_goprog    GOPROG (fresh_start.c)
 *   agc_waitlist_t3rupt   T3RUPT (waitlist.c)
 *   agc_dispatch_task     DTCB through T5LOC / T6LOC (dispatch.c)
 *   agc_exec_service_restart  ENEMA requested in the handler (alarm.c)
 *   handlers of other lanes   see agc_dispatch_rupt
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   The handlers' own statuses; BAD_ARG when a handler left interrupts
 *   enabled.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   The sim harness calls a lead-in when the counter overflows or the
 *   input arrives. Both vehicles.
 *
 * NOTES:
 *   At location 4000 the AGC has one 4-word vector per interrupt; each
 *   saves A,L (DXCH ARUPT), selects the handler's bank and jumps.
 *   Differences between the listings:
 *     - T6RUPT: Comanche jumps through the erasable 2CADR T6LOC, Luminary
 *       through the fixed T6ADR = DOT6RUPT (the LM DAP jet timer);
 *     - T5RUPT: Comanche first checks TIME5 (CS TIME5 / AD .5SEC / BZMF
 *       NOQBRSM: a T5RUPT with TIME5 >= 50 cs is ignored) and jumps through
 *       T5LOC; Luminary jumps through T5ADR;
 *     - RADAR RUPT: Comanche VHFREAD, Luminary RADAREAD;
 *     - RUPT10 (hand controller): Comanche RESUME +3 (not used), Luminary
 *       PITFALL.
 *   Each vector runs with interrupts inhibited, as an AGC interrupt
 *   does, and on the way out (RESUME) services a software restart that
 *   the handler requested. Handlers owned by other lanes are reached
 *   through agc_dispatch_rupt() (LAYERING: like dispatch.c, the one place
 *   exec calls the handlers' layers).
 *
 *   INHINT / RELINT are counted, so that nested C calls (a task calling
 *   WAITLIST, which itself inhibits) keep interrupts inhibited until the
 *   outermost RELINT. In a single-threaded host they only document intent
 *   and let services check that they run in interrupt context.
 **********************************************************************/
#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"
#include "internal.h"

#define RUPT_TIME4_SB 037775u /* STARTSUB: 37775 to TIME4         */
#define RUPT_TIME5_SB 037774u /* STARTSUB: 37774 to TIME5         */
#define RUPT_HALF_SEC 50      /* .5SEC DEC 50 (cs)                */

static int           inhint_depth;             /* > 0: interrupts inhibited        */
static uint16_t      time4;                    /* TIME4 (T4RUPT every 10-120 ms)   */
static uint16_t      time5;                    /* TIME5 (T5RUPT, autopilot)        */
static uint16_t      time6;                    /* TIME6 (T6RUPT, LM jet timing)    */
static agc_task_id_t t5loc = AGC_TASK_T5IDLOC; /* T5LOC / T5ADR          */
static agc_task_id_t t6loc = AGC_TASK_NONE;    /* T6LOC (CSM)            */

/**********************************************************************
 * AGC: none -- INHINT instruction: inhibit interrupts; counted, see the NOTES.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_inhint(void)
{
    inhint_depth++;
}

/**********************************************************************
 * AGC: none -- RELINT instruction: allow interrupts again at the outermost RELINT.
 * A RELINT with interrupts allowed has no effect, as in the AGC.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_relint(void)
{
    if (inhint_depth > 0)
    {
        inhint_depth--;
    }
}

/**********************************************************************
 * AGC: none -- true while interrupts are inhibited (INHINT, or inside
 * an interrupt).
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true while inhibited
 **********************************************************************/
bool agc_interrupts_inhibited(void)
{
    return (inhint_depth > 0);
}

/**********************************************************************
 * STARTSUB / STARTSB1 (FRESH_START_AND_RESTART C p.190 / L p.220):
 * TIME4 = 37775, TIME5 = 37774 (TIME3 = 37777 is set by the waitlist).
 *
 * ARGUMENTS: none
 **********************************************************************/
void rupt_reset_timers(void)
{
    time4 = RUPT_TIME4_SB;
    time5 = RUPT_TIME5_SB;
}

/**********************************************************************
 * AGC: none -- C(TIMEn) for n = 4, 5, 6 (counter words, ERASABLE
 * ASSIGNMENTS 0033-0035).
 *
 * ARGUMENTS:
 *   n -- counter number: 4, 5 or 6 (a constant at every caller)
 *
 * RETURNS: uint16_t -- the 15-bit counter word
 **********************************************************************/
uint16_t agc_rupt_time(int n)
{
    uint16_t v = time6; /* TIME6 unless n names TIME4 / TIME5 */

    if (n == 4)
    {
        v = time4;
    }
    else if (n == 5)
    {
        v = time5;
    }
    else
    {
        /* TIME6 */
    }

    return (v);
}

/**********************************************************************
 * AGC: none -- TS TIMEn for n = 4, 5, 6 (counter words).
 *
 * ARGUMENTS:
 *   n -- counter number: 4, 5 or 6 (a constant at every caller)
 *   v -- the 15-bit counter word
 **********************************************************************/
void agc_rupt_set_time(int n, uint16_t v)
{
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
}

/**********************************************************************
 * DXCH T5LOC (CSM) / T5ADR (LM) (INTERRUPT_LEAD_INS C p.131 / L p.153):
 * the program the next T5RUPT runs.
 *
 * ARGUMENTS:
 *   task -- the T5RUPT program (AGC_TASK_*)
 **********************************************************************/
void agc_rupt_set_t5_task(agc_task_id_t task)
{
    t5loc = task;
}

/**********************************************************************
 * C(T5LOC) (CSM) / C(T5ADR) (LM) (INTERRUPT_LEAD_INS C p.131 / L p.153).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_task_id_t -- the program the next T5RUPT runs
 **********************************************************************/
agc_task_id_t agc_rupt_t5_task(void)
{
    return (t5loc);
}

/**********************************************************************
 * TS T6LOC (INTERRUPT_LEAD_INS C p.131; CSM only, the LM T6RUPT always
 * goes to DOT6RUPT).
 *
 * ARGUMENTS:
 *   task -- the T6RUPT program (AGC_TASK_*)
 **********************************************************************/
void agc_rupt_set_t6_task(agc_task_id_t task)
{
    t6loc = task;
}

/**********************************************************************
 * RESUME (INTERRUPT_LEAD_INS / WAITLIST C p.1232 / L p.1129): leave the
 * interrupt; run a software restart that the handler requested.
 *
 * ARGUMENTS:
 *   st -- the handler's status
 *
 * RETURNS: agc_status_t -- st; BAD_ARG when the handler left interrupts
 *          enabled or the restart failed with BAD_ARG
 **********************************************************************/
static agc_status_t rupt_exit(agc_status_t st)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_relint();

    if (agc_exec_restart_pending())
    {
        agc_status_t rs = agc_exec_service_restart(); /* ENEMA outcome */

        if (!AGC_ASSERT(rs != AGC_STATUS_BAD_ARG))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (st);
}

/**********************************************************************
 * Vector 4000 (INTERRUPT_LEAD_INS C p.131 / L p.153): GOJAM, hardware
 * restart -> GOPROG.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the GOPROG status
 **********************************************************************/
agc_status_t agc_rupt_go(void)
{
    inhint_depth = 0; /* GOJAM clears the RUPT state   */
    agc_inhint();

    return (rupt_exit(agc_restart_goprog()));
}

/**********************************************************************
 * T6RUPT (INTERRUPT_LEAD_INS C p.131 / L p.153): CSM DTCB through T6LOC,
 * LM through T6ADR = DOT6RUPT.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the handler's status; NOT_FOUND for a CSM
 *          T6RUPT with no T6 program
 **********************************************************************/
agc_status_t agc_rupt_t6(void)
{
    agc_status_t st = AGC_STATUS_NOT_FOUND; /* handler outcome; CSM without T6LOC */

    agc_inhint();

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
        /* CSM, no T6 program */
    }

    return (rupt_exit(st));
}

/**********************************************************************
 * T5RUPT (INTERRUPT_LEAD_INS C p.131 / L p.153): CSM tests TIME5 (a
 * T5RUPT with TIME5 >= .5SEC is spurious: BZMF NOQBRSM), then DTCB T5LOC;
 * LM DTCB T5ADR.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the T5 program's status (OK when spurious)
 **********************************************************************/
agc_status_t agc_rupt_t5(void)
{
    agc_status_t st       = AGC_STATUS_OK;                                                                      /* T5 program outcome */
    bool         spurious = (agc_state_vehicle() == AGC_VEHICLE_CSM) && (RUPT_HALF_SEC - (int32_t) time5 <= 0); /* CS TIME5 / AD .5SEC / BZMF */

    agc_inhint();

    if (!spurious)
    {
        st = agc_dispatch_task(t5loc);
    }

    return (rupt_exit(st));
}

/**********************************************************************
 * T3RUPT (INTERRUPT_LEAD_INS C p.131 / L p.153): the waitlist.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the T3RUPT status
 **********************************************************************/
agc_status_t agc_rupt_t3(void)
{
    agc_inhint();

    return (rupt_exit(agc_waitlist_t3rupt()));
}

/**********************************************************************
 * A lead-in (INTERRUPT_LEAD_INS C pp.131-132 / L pp.153-154) whose
 * handler belongs to another lane (agc_dispatch_rupt).
 *
 * ARGUMENTS:
 *   id -- the interrupt
 *
 * RETURNS: agc_status_t -- the handler's status
 **********************************************************************/
static agc_status_t foreign_rupt(agc_rupt_id_t id)
{
    agc_inhint();

    return (rupt_exit(agc_dispatch_rupt(id)));
}

/**********************************************************************
 * T4RUPT (INTERRUPT_LEAD_INS C p.131 / L p.153).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the handler's status
 **********************************************************************/
agc_status_t agc_rupt_t4(void)
{
    return (foreign_rupt(AGC_RUPT_T4));
}

/**********************************************************************
 * KEYRUPT1 (INTERRUPT_LEAD_INS C p.131 / L p.153): main DSKY key.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the handler's status
 **********************************************************************/
agc_status_t agc_rupt_keyrupt1(void)
{
    return (foreign_rupt(AGC_RUPT_KEY1));
}

/**********************************************************************
 * KEYRUPT2 / MARKRUPT (INTERRUPT_LEAD_INS C p.131 / L p.153).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the handler's status
 **********************************************************************/
agc_status_t agc_rupt_keyrupt2(void)
{
    return (foreign_rupt(AGC_RUPT_KEY2));
}

/**********************************************************************
 * UPRUPT (INTERRUPT_LEAD_INS C p.132 / L p.154): uplink word.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the handler's status
 **********************************************************************/
agc_status_t agc_rupt_uprupt(void)
{
    return (foreign_rupt(AGC_RUPT_UPRUPT));
}

/**********************************************************************
 * DOWNRUPT (INTERRUPT_LEAD_INS C p.132 / L p.154): downlink.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the handler's status
 **********************************************************************/
agc_status_t agc_rupt_downrupt(void)
{
    return (foreign_rupt(AGC_RUPT_DOWNRUPT));
}

/**********************************************************************
 * RADAR RUPT (INTERRUPT_LEAD_INS C p.132 / L p.154).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the handler's status
 **********************************************************************/
agc_status_t agc_rupt_radarupt(void)
{
    return (foreign_rupt(AGC_RUPT_RADAR));
}

/**********************************************************************
 * RUPT10 (INTERRUPT_LEAD_INS C p.132 / L p.154): hand controller.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the handler's status
 **********************************************************************/
agc_status_t agc_rupt_rupt10(void)
{
    return (foreign_rupt(AGC_RUPT_HAND));
}

/**********************************************************************
 * T5IDLOC (FRESH_START_AND_RESTART C p.192): the CSM's idle T5RUPT
 * program, every 163.84 s when no autopilot uses TIME5: just resume.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_task_t5idloc(void)
{
    return (AGC_STATUS_OK);
}

/**********************************************************************
 * The interrupt handlers owned by other lanes (INTERRUPT_LEAD_INS C
 * pp.131-132 / L pp.153-154).
 *
 * ARGUMENTS:
 *   rupt -- the interrupt
 *
 * RETURNS: agc_status_t -- the handler's status; BAD_ARG when called
 *          with interrupts enabled or for an unknown id
 **********************************************************************/
agc_status_t agc_dispatch_rupt(agc_rupt_id_t rupt)
{
    bool         lm = agc_state_vehicle() == AGC_VEHICLE_LM; /* Luminary rope */
    agc_status_t st = AGC_STATUS_BAD_ARG;                    /* handler outcome */

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    switch (rupt)
    {
        case AGC_RUPT_T6:                                              /* LM DOT6RUPT; CSM goes through T6LOC     */
            st = lm ? agc_lm_dap_t6rupt() : AGC_STATUS_NOT_FOUND;      /* LM : CSM */
            break;
        case AGC_RUPT_T5:                                              /* T5LOC / T5ADR                           */
            st = agc_dispatch_task(agc_rupt_t5_task());
            break;
        case AGC_RUPT_T3:
            st = agc_waitlist_t3rupt();
            break;
        case AGC_RUPT_KEY1:
            st = agc_dsky_keyrupt1();
            break;
        case AGC_RUPT_KEY2:                                            /* MARKRUPT: CSM sextant, LM AOT           */
            st = lm ? agc_aotmark_markrupt() : agc_sxtmark_markrupt(); /* LM : CSM */
            break;
        case AGC_RUPT_HAND:                                            /* LM PITFALL; CSM RESUME +3 (not used)    */
            st = lm ? agc_lm_pitfall() : AGC_STATUS_OK;                /* LM : CSM */
            break;
        case AGC_RUPT_T4:                                              /* T4RUPT (T4RUPT_PROGRAM), reloads TIME4    */
            st = agc_t4rupt_rupt();
            break;
        case AGC_RUPT_UPRUPT:                                          /* UPRUPT: the uplinked word in INLINK      */
            st = agc_dsky_uprupt(agc_erase_inlink());
            break;
        case AGC_RUPT_DOWNRUPT:                                        /* DODOWNTM: next downlink word pair      */
            st = agc_telem_downrupt();
            break;
        case AGC_RUPT_RADAR:                                           /* CSM VHFREAD (VHF range) / LM RADAREAD     */
            st = lm ? agc_rendz_radar_read() : agc_rendz_vhf_read();   /* LM : CSM */
            break;
        default:
            break;
    } /* switch (rupt) */

    return (st);
} /* agc_dispatch_rupt */
