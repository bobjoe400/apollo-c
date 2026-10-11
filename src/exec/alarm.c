/**********************************************************************
 * FILE NAME: alarm.c
 *
 * PURPOSE:
 *   Alarms and aborts: ALARM, VARALARM, PRIOLARM, CURTAINS, BAILOUT,
 *   POODOO, CCSHOLE, ABORT, WHIMPER (as a deferred ENEMA request),
 *   GOPOODOO and FLAGS (LM), VAC5STOR (CSM), and the interpreter's 1301 /
 *   1302 alarms raised at job or task return.
 *
 * AGC SOURCE:
 *   Comanche055 ALARM_AND_ABORT.agc pp. 1493-1496;
 *   FRESH_START_AND_RESTART.agc p. 204 (VAC5STOR)
 *   Luminary099 ALARM_AND_ABORT.agc pp. 1381-1385 (differs: see NOTES)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   FAILREG +0..2 uint16_t  I/O  octal code     agc_erase_failreg()
 *   DSPTAB +11D   uint16_t  I/O  lamp bits      PROG lamp (bit 9, bit 15)
 *   STATEFLG, REINTFLG, NODOFLAG flags  O  -    reset by POODOO / FLAGS
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   fresh_enema           ENEMA software restart (fresh_start.c)
 *   agc_phase_mr_klean    MR.KLEAN, V37KLEAN (phase_table.c)
 *   exec_up_servidle      LM STRTIDLE: SERVIDLE (change_program/idle.c)
 *   agc_interp_take_alarm the interpreter's latched 1301 / 1302 (core)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   This file raises them: ALARM (any code), BAILOUT (software restart),
 *   POODOO (restart to P00), CCSHOLE (POODOO 1103), CURTAINS (ALARM 217).
 *   Returns AGC_STATUS_BAILOUT / AGC_STATUS_POODOO for the caller to pass
 *   up; BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   WHIMPER is deferred: ENEMA runs at the next service point (end of
 *   the job, of T3RUPT or of an interrupt lead-in).
 *
 * NOTES:
 *   Differences between the listings:
 *     - Comanche's ALARM also saves the running job's LOC and BANKSET in
 *       LOCALARM / BANKALRM, and BAILOUT, POODOO, CCSHOLE first copy the
 *       executive and phase tables into VAC5 (VAC5STOR) for debugging.
 *     - POODOO: Comanche does a BAILOUT (software restart, no P00) when
 *       average-G is on (V37FLAG), otherwise clears STATEFLG / REINTFLG /
 *       NODOFLAG, kills all groups (MR.KLEAN) and restarts, which ends in
 *       GOTOPOOH. Luminary sets restart phase 4.35 (GOPOODOO) and then, if
 *       SERVICER is running, puts it in its ground state (STRTIDLE) and goes
 *       to GOTOPOOH directly; otherwise kills all groups and restarts.
 *     - ABORT: Comanche ABORT = BAILOUT, Luminary ABORT = WHIMPER.
 *
 *   FAILREG +0..+2 hold the first two alarm codes and the latest one; a
 *   third and later alarm sets bit 15 of FAILREG +2. Only the first alarm
 *   lights the PROG lamp (DSPTAB +11D bit 9, with bit 15 = "send me").
 *
 *   WHIMPER -> ENEMA: the AGC leaves the interrupted program by faking a
 *   RESUME into ENEMA. In C the software restart is requested here and run
 *   at the next service point (agc_exec_service_restart), so ENEMA never
 *   runs inside the job or task that failed and the call graph has no
 *   cycle (ENEMA -> RESTARTS -> NOVAC -> BAILOUT).
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define AL_DSPTAB_LAMPS 11      /* DSPTAB +11D: C/S lamp word          */
#define AL_OCT40400     040400u /* bit 15 (update) + bit 9 (PROG lamp) */
#define AL_BIT15        040000u /* MULTFAIL marker in FAILREG +2       */
#define AL_POSMAX       037777u /* POSMAX: the 15-bit alarm code field */
#define AL_PHASE_4_35   035u    /* OCT35: 4.35SPOT, GOPOODOO (LM)      */

static agc_job_id_t almcadr;      /* ALMCADR: job that raised the alarm  */
static agc_job_id_t localarm;     /* LOCALARM (CSM): LOC at the alarm    */
static bool         enema_wanted; /* WHIMPER requested, not yet run      */
static uint16_t     restarts_run; /* software restarts serviced (debug)  */

/* VAC5STOR (C p.204): copy of LOC, PRIORITY of each core set, the phase
 * table and NEWJOB, for post-flight analysis of a restart.            */
static struct
{
    agc_exec_core_info_t core[AGC_EXEC_MAX_JOBS];   /* LOC, BANKSET, PRIORITY of each core set */
    uint16_t             phase[AGC_RESTART_GROUPS]; /* PHASE1 .. PHASE6 */
    int                  newjob;                    /* NEWJOB */
} vac5;

/**********************************************************************
 * VAC5STOR (FRESH_START_AND_RESTART.agc C p.204; Comanche only): copy
 * LOC, PRIORITY of each core set, the phase table and NEWJOB into VAC5
 * for post-flight analysis of a restart.
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_vac5stor(void)
{
    if (agc_state_vehicle() != AGC_VEHICLE_CSM)
    {
        return; /* Comanche only     */
    }

    for (int i = 0; i < AGC_EXEC_MAX_JOBS; i++)
    {
        agc_status_t st = agc_exec_core_info(i, &vac5.core[i]); /* copy of core set i */

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return;
        }
    }

    for (int g = 1; g <= AGC_RESTART_GROUPS; g++)
    {
        vac5.phase[g - 1] = agc_phase_get(g);
    }

    vac5.newjob = agc_exec_newjob();
    if (!AGC_ASSERT(vac5.newjob >= -1))
    {
        return;
    }

}

/**********************************************************************
 * PROGLARM (C p.1494 / L p.1382): CS DSPTAB +11D / MASK OCT40400 / ADS --
 * turn on the PROG lamp and mark the lamp word for output.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void proglarm(void)
{
    uint16_t w = agc_erase_dsptab(AL_DSPTAB_LAMPS); /* DSPTAB +11D, the C/S lamp word */

    if (!AGC_ASSERT(w <= 077777u))
    {
        return;
    }

    agc_erase_set_dsptab(AL_DSPTAB_LAMPS, (uint16_t) (w | AL_OCT40400));
}

/**********************************************************************
 * LARMENT / CHKFAIL1 / CHKFAIL2 / FAIL3 / MULTFAIL (C p.1493 / L p.1381).
 *
 * ARGUMENTS:
 *   code -- alarm code, octal as displayed (1..077777)
 **********************************************************************/
static void store_alarm(agc_alarm_code_t code)
{
    almcadr = exec_running_job_loc(); /* TS ALMCADR        */

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        localarm = almcadr; /* CA LOC/TS LOCALARM */
    }

    agc_status_t st; /* outcome of the FAILREG store */

    if (agc_erase_failreg(0) == 0U)
    {
        st = agc_erase_set_failreg(0, code); /* first alarm       */
        proglarm();
    }
    else if (agc_erase_failreg(1) == 0U)
    {
        st = agc_erase_set_failreg(1, code);
    }
    else if ((agc_erase_failreg(2) & AL_POSMAX) != 0U)
    {
        st = agc_erase_set_failreg(2, (uint16_t) (code | AL_BIT15));
    }
    else
    {
        st = agc_erase_set_failreg(2, code);
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }
}

/**********************************************************************
 * ALARM (C p.1493 / L p.1381): light PROG, record the code, return.
 *
 * ARGUMENTS:
 *   code -- alarm code, octal as displayed (1..077777)
 **********************************************************************/
void agc_alarm(agc_alarm_code_t code)
{
    if (!AGC_ASSERT(code != 0U && code <= AL_POSMAX))
    {
        return;
    }

    agc_inhint();
    store_alarm(code);
    agc_relint();

    if (!AGC_ASSERT(agc_erase_failreg(0) != 0U))
    {
        return;
    }
}

/**********************************************************************
 * VARALARM (C p.1496 / L p.1384): ALARM with the code in A.
 *
 * ARGUMENTS:
 *   code -- alarm code, octal as displayed (1..077777)
 **********************************************************************/
void agc_exec_varalarm(agc_alarm_code_t code)
{
    if (!AGC_ASSERT(code != 0U))
    {
        return;
    }

    if (!AGC_ASSERT(code <= AL_POSMAX))
    {
        return;
    }

    agc_alarm(code);
}

/**********************************************************************
 * CURTAINS (C p.1496 / L p.1383): ALARM 217.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_curtains(void)
{
    _Static_assert(AGC_ALARM_00217 <= AL_POSMAX, "AGC constant");

    if (!AGC_ASSERT(restarts_run < UINT16_MAX))
    {
        return;
    }

    agc_alarm(AGC_ALARM_00217);
}

/**********************************************************************
 * PRIOLARM (C p.1494 / L p.1382): record the alarm as ALARM does, then
 * show V05N09 as a priority display (PRIODSPR).
 * DEVIATION: the display needs the caller's job context and its three
 * astronaut continuations, which the exec layer cannot name; the caller
 * requests it with agc_dsky_display(ctx, AGC_DSKY_PRIODSPR, V05N09, ..)
 * after this returns.
 *
 * ARGUMENTS:
 *   code -- alarm code, octal as displayed (1..077777)
 **********************************************************************/
void agc_prioalarm(agc_alarm_code_t code)
{
    agc_inhint();
    store_alarm(code);
    agc_relint();

    if (!AGC_ASSERT(agc_erase_failreg(0) != 0U))
    {
        return;
    }
}

/**********************************************************************
 * WHIMPER (C p.1495 / L p.1383): software restart via ENEMA.
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_request_enema(void)
{
    if (!AGC_ASSERT(restarts_run < UINT16_MAX))
    {
        return;
    }

    enema_wanted = true;
    if (!AGC_ASSERT(enema_wanted))
    {
        return;
    }

}

/**********************************************************************
 * WHIMPER as an entry (ALARM_AND_ABORT.agc C p.1495 / L p.1383): request
 * the ENEMA software restart; it runs at the next service point.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; BAD_ARG on a failed postcondition
 **********************************************************************/
agc_status_t agc_exec_enema(void)
{
    exec_request_enema();

    if (!AGC_ASSERT(agc_exec_restart_pending()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(restarts_run < UINT16_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * WHIMPER pending (ALARM_AND_ABORT.agc C p.1495 / L p.1383): an ENEMA
 * was requested and has not run yet.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true while a software restart is pending
 **********************************************************************/
bool agc_exec_restart_pending(void)
{
    if (!AGC_ASSERT(restarts_run < UINT16_MAX))
    {
        return (false);
    }

    if (!AGC_ASSERT(agc_erase_failreg(2) <= 077777u))
    {
        return (false);
    }

    return (enema_wanted);
}

/**********************************************************************
 * Service point of the deferred WHIMPER -> ENEMA (ALARM_AND_ABORT.agc
 * C p.1495 / L p.1383; FRESH_START_AND_RESTART.agc C p.188 / L p.217):
 * run the requested ENEMA once. A BAILOUT raised while ENEMA restarts
 * the groups requests another ENEMA, serviced at the next service point
 * (the AGC would loop through ENEMA the same way).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when none was pending; else the ENEMA status
 *          (OK, BAILOUT, ALARM, NOT_FOUND, UNIMPLEMENTED); BAD_ARG otherwise
 **********************************************************************/
agc_status_t agc_exec_service_restart(void)
{
    if (!AGC_ASSERT(restarts_run < UINT16_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!enema_wanted)
    {
        return (AGC_STATUS_OK);
    }

    enema_wanted    = false;
    restarts_run    = (uint16_t) (restarts_run + 1U);
    agc_status_t st = fresh_enema(); /* ENEMA outcome */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BAILOUT || st == AGC_STATUS_ALARM || st == AGC_STATUS_NOT_FOUND || st == AGC_STATUS_UNIMPLEMENTED))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * BAILOUT / BAILOUT1 (C p.1495 / L p.1383): record the alarm and do a
 * software restart. Returns AGC_STATUS_BAILOUT; the caller returns it.
 *
 * ARGUMENTS:
 *   code -- alarm code, octal as displayed (1..077777)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAILOUT (the caller returns it); BAD_ARG on a
 *          failed precondition
 **********************************************************************/
agc_status_t agc_bailout(agc_alarm_code_t code)
{
    agc_inhint();
    exec_vac5stor();
    store_alarm(code);
    exec_request_enema(); /* OCT40400+1: WHIMPER */
    agc_relint();

    if (!AGC_ASSERT(enema_wanted))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(restarts_run < UINT16_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_BAILOUT);
}

/**********************************************************************
 * FLAGS (L p.1385) / the three DOWNFLAGs of Comanche POODOO (C p.1495):
 * reset STATEFLG, REINTFLG and NODOFLAG.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void poodoo_flags(void)
{
    agc_flag_clear(AGC_FLAG_STATEFLG);
    agc_flag_clear(AGC_FLAG_REINTFLG);
    agc_flag_clear(AGC_FLAG_NODOFLAG);

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_STATEFLG)))
    {
        return;
    }
}

/**********************************************************************
 * GOPOODOO / STRTIDLE (L p.1383): the Luminary POODOO tail, also the
 * 4.35 restart job.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_POODOO; the SERVIDLE status when it fails;
 *          BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t gopoodoo(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    poodoo_flags();
    agc_status_t st = AGC_STATUS_POODOO; /* POODOO: the program is abandoned */

    if (agc_flag_test(AGC_FLAG_SERVICER_RUNNING))
    {
        agc_relint();

        /* STRTIDLE: SERVICER to its "ground" state; SERVIDLE ends in
         * WHIMPER and the 4.33 restart (GOP00FIX) goes to P00.        */
        agc_status_t s1 = exec_up_servidle(); /* SERVIDLE outcome */

        if (!AGC_ASSERT(s1 == AGC_STATUS_OK))
        {
            return (s1);
        }

        return (st);
    }

    agc_status_t k1 = agc_phase_change_program_klean(); /* groups 1,3,5,6    */
    agc_status_t k2 = agc_phase_mr_klean();             /* 2,4,... (4 last)  */

    if (!AGC_ASSERT(k1 == AGC_STATUS_OK && k2 == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    exec_request_enema(); /* TCF WHIMPER       */
    agc_relint();

    return (st);
}

/**********************************************************************
 * The interpreter alarms of the job or task that just returned
 * (INTERPRETER.agc SQRTABRT C p.1184 / L p.1079, ACOSABRT C p.1191 /
 * L p.1086): ALARM 01301 (ARCSIN/
 * ARCCOS out of range) or POODOO 01302 (SQRT of a negative).
 * DEVIATION: the AGC raises them at the instruction; core cannot call
 * exec, so they are raised when the job or task returns.
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_service_interp_alarm(void)
{
    uint16_t code = agc_interp_take_alarm(); /* 1301, 1302 or 0 latched by the interpreter */

    if (code == 01301u)
    {
        agc_alarm((agc_alarm_code_t) 01301);
    }
    else if (code == 01302u)
    {
        agc_status_t st = agc_poodoo((agc_alarm_code_t) 01302); /* POODOO 1302 outcome */

        if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
        {
            return;
        }
    }
    else
    {
        /* none */
    }
}

/**********************************************************************
 * POODOO / POODOO1 / ABORT2 (C p.1495 / L p.1383): record the alarm and
 * abandon the current program; the restart ends in P00 (GOTOPOOH).
 *
 * ARGUMENTS:
 *   code -- alarm code, octal as displayed (1..077777)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_POODOO; AGC_STATUS_BAILOUT (Comanche with
 *          average-G on); BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_poodoo(agc_alarm_code_t code)
{
    agc_inhint();
    exec_vac5stor();
    store_alarm(code);
    agc_relint();

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        /* Comanche POODOO after BORTENT (C p.1495).                            */
        if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        if (agc_flag_test(AGC_FLAG_SERVICER_RUNNING))
        { /* average G on      */
            exec_request_enema(); /* TC WHIMPER -1     */

            return (AGC_STATUS_BAILOUT);
        }

        poodoo_flags();
        agc_status_t st = agc_phase_mr_klean(); /* MR.KLEAN outcome */

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        exec_request_enema(); /* TC WHIMPER        */
        if (!AGC_ASSERT(agc_phase_get(4) == 0U))
        {
            return (AGC_STATUS_BAD_ARG);
        }


        return (AGC_STATUS_POODOO);
    }

    agc_inhint();
    phase_set_fixed(4, AL_PHASE_4_35); /* DXCH -PHASE4      */
    agc_relint();

    if (!AGC_ASSERT(phase_group_agrees(4)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (gopoodoo());
} /* agc_poodoo */

/**********************************************************************
 * GOPOODOO as the 4.35 restart job (L RESTART_TABLES p.242).
 *
 * ARGUMENTS:
 *   ctx -- job context (core set 0); ctx->id is the job being run
 *
 * RETURNS: agc_status_t -- the GOPOODOO status (AGC_STATUS_POODOO); BAD_ARG on a
 *          failed precondition
 **********************************************************************/
agc_status_t agc_job_gopoodoo(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != 0) || !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (gopoodoo());
}

/**********************************************************************
 * CCSHOLE (C p.1496 / L p.1383): POODOO 1103, an "impossible" branch.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the POODOO 1103 status; BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_ccshole(void)
{
    if (!AGC_ASSERT(restarts_run < UINT16_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    _Static_assert(AGC_ALARM_01103 <= AL_POSMAX, "AGC constant");

    return (agc_poodoo(AGC_ALARM_01103));
}

/**********************************************************************
 * ABORT (C p.1496: EQUALS BAILOUT; L p.1384: EQUALS WHIMPER).
 * DEVIATION: Comanche's ABORT takes its code from the word after the
 * call, which this entry does not have; both vehicles do the WHIMPER.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAILOUT; BAD_ARG on a failed postcondition
 **********************************************************************/
agc_status_t agc_abort_restart(void)
{
    if (!AGC_ASSERT(restarts_run < UINT16_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    exec_request_enema();
    agc_relint();

    if (!AGC_ASSERT(enema_wanted))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_BAILOUT);
}
