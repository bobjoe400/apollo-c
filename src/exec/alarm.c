/*
 * alarm.c -- ALARM, VARALARM, PRIOLARM, CURTAINS, BAILOUT, POODOO,
 *            CCSHOLE, ABORT, WHIMPER, GOPOODOO, FLAGS, VAC5STOR.
 *
 * Source: ALARM_AND_ABORT.agc, Comanche055 pp. 1493-1496, Luminary099
 * pp. 1381-1385; VAC5STOR from FRESH_START_AND_RESTART.agc C p.204.
 * Differences between the listings:
 *   - Comanche's ALARM also saves the running job's LOC and BANKSET in
 *     LOCALARM / BANKALRM, and BAILOUT, POODOO, CCSHOLE first copy the
 *     executive and phase tables into VAC5 (VAC5STOR) for debugging.
 *   - POODOO: Comanche does a BAILOUT (software restart, no P00) when
 *     average-G is on (V37FLAG), otherwise clears STATEFLG / REINTFLG /
 *     NODOFLAG, kills all groups (MR.KLEAN) and restarts, which ends in
 *     GOTOPOOH. Luminary sets restart phase 4.35 (GOPOODOO) and then, if
 *     SERVICER is running, puts it in its ground state (STRTIDLE) and goes
 *     to GOTOPOOH directly; otherwise kills all groups and restarts.
 *   - ABORT: Comanche ABORT = BAILOUT, Luminary ABORT = WHIMPER.
 *
 * FAILREG +0..+2 hold the first two alarm codes and the latest one; a
 * third and later alarm sets bit 15 of FAILREG +2. Only the first alarm
 * lights the PROG lamp (DSPTAB +11D bit 9, with bit 15 = "send me").
 *
 * WHIMPER -> ENEMA: the AGC leaves the interrupted program by faking a
 * RESUME into ENEMA. In C the software restart is requested here and run
 * at the next service point (agc_exec_service_restart), so ENEMA never
 * runs inside the job or task that failed and the call graph has no
 * cycle (ENEMA -> RESTARTS -> NOVAC -> BAILOUT).
 */
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define AL_DSPTAB_LAMPS 11      /* DSPTAB +11D: C/S lamp word          */
#define AL_OCT40400     040400u /* bit 15 (update) + bit 9 (PROG lamp) */
#define AL_BIT15        040000u /* MULTFAIL marker in FAILREG +2       */
#define AL_POSMAX       037777u
#define AL_FAILREGS     3
#define AL_PHASE_4_35   035u /* OCT35: 4.35SPOT, GOPOODOO (LM)      */

static agc_job_id_t almcadr;      /* ALMCADR: job that raised the alarm  */
static agc_job_id_t localarm;     /* LOCALARM (CSM): LOC at the alarm    */
static bool         enema_wanted; /* WHIMPER requested, not yet run      */
static uint16_t     restarts_run; /* software restarts serviced (debug)  */

/* VAC5STOR (C p.204): copy of LOC, PRIORITY of each core set, the phase
 * table and NEWJOB, for post-flight analysis of a restart.            */
static struct
{
    agc_exec_core_info_t core[AGC_EXEC_MAX_JOBS];
    uint16_t             phase[AGC_RESTART_GROUPS];
    int                  newjob;
} vac5;

void exec_vac5stor(void)
{
    if (agc_state_vehicle() != AGC_VEHICLE_CSM)
    {
        return; /* Comanche only     */
    }
    for (int i = 0; i < AGC_EXEC_MAX_JOBS; i++)
    {
        agc_status_t st = agc_exec_core_info(i, &vac5.core[i]);
        (void) AGC_ASSERT(st == AGC_STATUS_OK);
    }
    for (int g = 1; g <= AGC_RESTART_GROUPS; g++)
    {
        vac5.phase[g - 1] = phase_word(g);
    }
    vac5.newjob = agc_exec_newjob();
    (void) AGC_ASSERT(vac5.newjob >= -1);
}

/* PROGLARM (C p.1494 / L p.1382): CS DSPTAB +11D / MASK OCT40400 / ADS --
 * turn on the PROG lamp and mark the lamp word for output.            */
static void proglarm(void)
{
    uint16_t w = agc_erase_dsptab(AL_DSPTAB_LAMPS);
    (void) AGC_ASSERT(w <= 077777u);
    agc_status_t st =
        agc_erase_set_dsptab(AL_DSPTAB_LAMPS, (uint16_t) (w | AL_OCT40400));
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
}

/* LARMENT / CHKFAIL1 / CHKFAIL2 / FAIL3 / MULTFAIL (C p.1493 / L p.1381).*/
static void store_alarm(agc_alarm_code_t code)
{
    if (!AGC_ASSERT(code != 0U && code <= AL_POSMAX))
    {
        return;
    }
    almcadr = exec_running_job_loc(); /* TS ALMCADR        */
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        localarm = almcadr; /* CA LOC/TS LOCALARM */
    }
    agc_status_t st;
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
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
}

/* ALARM (C p.1493 / L p.1381): light PROG, record the code, return.    */
void agc_alarm(agc_alarm_code_t code)
{
    if (!AGC_ASSERT(code != 0U && code <= AL_POSMAX))
    {
        return;
    }
    agc_inhint();
    store_alarm(code);
    agc_relint();
    (void) AGC_ASSERT(agc_erase_failreg(0) != 0U);
}

/* VARALARM (C p.1496 / L p.1384): ALARM with the code in A.            */
void agc_exec_varalarm(agc_alarm_code_t code)
{
    (void) AGC_ASSERT(code != 0U);
    (void) AGC_ASSERT(code <= AL_POSMAX);
    agc_alarm(code);
}

/* CURTAINS (C p.1496 / L p.1383): ALARM 217.                           */
void agc_curtains(void)
{
    (void) AGC_ASSERT(AGC_ALARM_00217 <= AL_POSMAX);
    (void) AGC_ASSERT(restarts_run < UINT16_MAX);
    agc_alarm(AGC_ALARM_00217);
}

/* PRIOLARM (C p.1494 / L p.1382): record the alarm as ALARM does, then
 * show V05N09 as a priority display (PRIODSPR).
 * DEVIATION: the display needs the caller's job context and its three
 * astronaut continuations, which the exec layer cannot name; the caller
 * requests it with agc_dsky_display(ctx, AGC_DSKY_PRIODSPR, V05N09, ..)
 * after this returns.                                 */
void agc_prioalarm(agc_alarm_code_t code)
{
    if (!AGC_ASSERT(code != 0U && code <= AL_POSMAX))
    {
        return;
    }
    agc_inhint();
    store_alarm(code);
    agc_relint();
    (void) AGC_ASSERT(agc_erase_failreg(0) != 0U);
}

/* WHIMPER (C p.1495 / L p.1383): software restart via ENEMA.           */
void exec_request_enema(void)
{
    (void) AGC_ASSERT(restarts_run < UINT16_MAX);
    enema_wanted = true;
    (void) AGC_ASSERT(enema_wanted);
}

agc_status_t agc_exec_enema(void)
{
    exec_request_enema();
    (void) AGC_ASSERT(agc_exec_restart_pending());
    (void) AGC_ASSERT(restarts_run < UINT16_MAX);
    return AGC_STATUS_OK;
}

bool agc_exec_restart_pending(void)
{
    (void) AGC_ASSERT(restarts_run < UINT16_MAX);
    (void) AGC_ASSERT(agc_erase_failreg(2) <= 077777u);
    return enema_wanted;
}

/* Service point: run the requested ENEMA once. A BAILOUT raised while
 * ENEMA restarts the groups requests another ENEMA, serviced at the next
 * service point (the AGC would loop through ENEMA the same way).       */
agc_status_t agc_exec_service_restart(void)
{
    (void) AGC_ASSERT(restarts_run < UINT16_MAX);
    if (!enema_wanted)
    {
        return AGC_STATUS_OK;
    }
    enema_wanted    = false;
    restarts_run    = (uint16_t) (restarts_run + 1U);
    agc_status_t st = fresh_enema();
    (void) AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BAILOUT ||
                      st == AGC_STATUS_ALARM || st == AGC_STATUS_NOT_FOUND ||
                      st == AGC_STATUS_UNIMPLEMENTED);
    return st;
}

/* BAILOUT / BAILOUT1 (C p.1495 / L p.1383): record the alarm and do a
 * software restart. Returns AGC_STATUS_BAILOUT; the caller returns it. */
agc_status_t agc_bailout(agc_alarm_code_t code)
{
    if (!AGC_ASSERT(code != 0U && code <= AL_POSMAX))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    exec_vac5stor();
    store_alarm(code);
    exec_request_enema(); /* OCT40400+1: WHIMPER */
    agc_relint();
    (void) AGC_ASSERT(enema_wanted);
    (void) AGC_ASSERT(restarts_run < UINT16_MAX);
    return AGC_STATUS_BAILOUT;
}

/* FLAGS (L p.1385) / the three DOWNFLAGs of Comanche POODOO (C p.1495):
 * reset STATEFLG, REINTFLG and NODOFLAG.                              */
static void poodoo_flags(void)
{
    agc_status_t a = agc_flag_clear(AGC_FLAG_STATEFLG);
    agc_status_t b = agc_flag_clear(AGC_FLAG_REINTFLG);
    agc_status_t c = agc_flag_clear(AGC_FLAG_NODOFLAG);
    (void) AGC_ASSERT(a == AGC_STATUS_OK && b == AGC_STATUS_OK);
    (void) AGC_ASSERT(c == AGC_STATUS_OK);
}

/* GOPOODOO / STRTIDLE (L p.1383): the Luminary POODOO tail, also the
 * 4.35 restart job.                                                   */
static agc_status_t gopoodoo(void)
{
    agc_inhint();
    poodoo_flags();
    agc_status_t st = AGC_STATUS_POODOO;
    if (agc_flag_test(AGC_FLAG_SERVICER_RUNNING))
    {
        agc_relint();
        /* STRTIDLE: SERVICER to its "ground" state, then GOTOPOOH.    */
        agc_status_t s1 = exec_up_servidle();
        (void) AGC_ASSERT(s1 == AGC_STATUS_OK ||
                          s1 == AGC_STATUS_UNIMPLEMENTED);
        agc_status_t s2 = agc_exec_gotopooh();
        (void) AGC_ASSERT(s2 == AGC_STATUS_OK || s2 == AGC_STATUS_BAILOUT);
        return st;
    }
    agc_status_t k1 = agc_phase_change_program_klean(); /* groups 1,3,5,6    */
    agc_status_t k2 = agc_phase_mr_klean();             /* 2,4,... (4 last)  */
    (void) AGC_ASSERT(k1 == AGC_STATUS_OK && k2 == AGC_STATUS_OK);
    exec_request_enema(); /* TCF WHIMPER       */
    agc_relint();
    return st;
}

/* Comanche POODOO after BORTENT (C p.1495).                            */
static agc_status_t poodoo_csm(void)
{
    (void) AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM);
    if (agc_flag_test(AGC_FLAG_SERVICER_RUNNING))
    {                         /* average G on      */
        exec_request_enema(); /* TC WHIMPER -1     */
        return AGC_STATUS_BAILOUT;
    }
    poodoo_flags();
    agc_status_t st = agc_phase_mr_klean();
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    exec_request_enema(); /* TC WHIMPER        */
    (void) AGC_ASSERT(phase_word(4) == 0U);
    return AGC_STATUS_POODOO;
}

/* POODOO / POODOO1 / ABORT2 (C p.1495 / L p.1383): record the alarm and
 * abandon the current program; the restart ends in P00 (GOTOPOOH).    */
agc_status_t agc_poodoo(agc_alarm_code_t code)
{
    if (!AGC_ASSERT(code != 0U && code <= AL_POSMAX))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    exec_vac5stor();
    store_alarm(code);
    agc_relint();
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        return poodoo_csm();
    }
    agc_inhint();
    phase_set_fixed(4, AL_PHASE_4_35); /* DXCH -PHASE4      */
    agc_relint();
    (void) AGC_ASSERT(phase_group_agrees(4));
    return gopoodoo();
}

/* GOPOODOO as the 4.35 restart job (L RESTART_TABLES p.242).          */
agc_status_t agc_job_gopoodoo(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != 0) ||
        !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return gopoodoo();
}

/* CCSHOLE (C p.1496 / L p.1383): POODOO 1103, an "impossible" branch. */
agc_status_t agc_ccshole(void)
{
    (void) AGC_ASSERT(restarts_run < UINT16_MAX);
    (void) AGC_ASSERT(AGC_ALARM_01103 <= AL_POSMAX);
    return agc_poodoo(AGC_ALARM_01103);
}

/* ABORT (C p.1496: EQUALS BAILOUT; L p.1384: EQUALS WHIMPER).
 * DEVIATION: Comanche's ABORT takes its code from the word after the
 * call, which this entry does not have; both vehicles do the WHIMPER. */
agc_status_t agc_abort_restart(void)
{
    agc_inhint();
    exec_request_enema();
    agc_relint();
    (void) AGC_ASSERT(enema_wanted);
    return AGC_STATUS_BAILOUT;
}
