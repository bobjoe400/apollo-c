/**********************************************************************
 * FILE NAME: restarts.c
 *
 * PURPOSE:
 *   RESTARTS: re-establish the jobs, tasks and LONGCALLs that the phase
 *   table says were in progress, and the NXTRST loop of GOPROG3 that
 *   restarts every active group.
 *
 * AGC SOURCE:
 *   Comanche055 RESTARTS_ROUTINE.agc pp. 1414-1419;
 *   FRESH_START_AND_RESTART.agc p. 189 (NXTRST)
 *   Luminary099 RESTARTS_ROUTINE.agc pp. 1303-1308 (identical program);
 *   FRESH_START_AND_RESTART.agc p. 218 (NXTRST)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   PHASEn        uint16_t  I    phase word     agc_phase_get()
 *   PHSNAMEn/PRDTn  store   I    -              phase_var_get()
 *   TBASEn        uint16_t  I    cs (TIME1)     phase_tbase()
 *   LONGBASE      uint32_t  I    cs (TIME2,1)   phase_longbase()
 *   TIME2,TIME1   uint32_t  I    cs             agc_time_now_cs()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   rst_table_lookup      the fixed restart tables (restart_tables.c)
 *   agc_exec_findvac      FINDVAC / NOVAC of a restarted job
 *   agc_waitlist_call     WAITLIST / LONGCALL of a restarted task
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   BAILOUT 1201 / 1202 / 1203 through FINDVAC, NOVAC and WAITLIST;
 *   AGC_STATUS_BAD_ARG for a restart point that was never stored (job
 *   AGC_JOB_NONE, PHSPRDTn = 0, task AGC_TASK_NONE) or a DEC 0 table
 *   placeholder.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called from GOPROG3 with interrupts inhibited. Both vehicles.
 *
 * NOTES:
 *   For an active group n with PHASEn = p:
 *     p = 1           the last display is restarted: FINDVAC PRIO14 INITDSP;
 *     p odd  >= 3     the single odd-table entry is restarted;
 *     p even >= 2     both even-table entries are restarted;
 *     type C          the variable PHSNAMEn is restarted as a job (PHSPRDTn
 *                     > 0 FINDVAC, < 0 NOVAC), a task or a LONGCALL
 *                     according to CJW;
 *     type B          the variable job, then the first (or only) table entry
 *                     of phase p & 177.
 *   A task restarts at the time it would have run: its delta time is
 *   measured from TBASEn (FINDTIME), and if that time has passed it starts
 *   in 1 cs. A LONGCALL is measured from LONGBASE the same way.
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define RS_VARBITS 01400u             /* OCT1400: type B (01000) or C (0400)    */
#define RS_TYPEB   01000u             /* bit 10 of PHASEn: type B               */
#define RS_CJW     07u                /* CJW bits of a type C PHASEn            */
#define RS_LOW7    0177u              /* OCT177 = LOW7: table phase of type B   */
#define RS_TIME1   037777u            /* TIME1 wraps at 2^14 cs                 */
#define RS_PRIO14  ((agc_prio_t) 014) /* X.1: FINDVAC PRIO14 INITDSP            */
#define RS_SOON_CS 1                  /* IMEDIATE: AD ONE -> 1 cs               */

/**********************************************************************
 * FINDTIME / IMEDIATE (C pp.1415-1416 / L pp.1304-1305): a task due dt
 * cs after TBASEn is called max(1, dt - (TIME1 - TBASEn)) cs from now
 * (TIME1 elapsed time modulo 2^14 cs, as the AGC's single-word
 * subtraction gives).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   dt -- delta time from TBASEn, cs
 *
 * RETURNS: uint16_t -- delay from now, cs (>= 1)
 **********************************************************************/
static uint16_t findtime(int group, int32_t dt)
{
    uint32_t now     = agc_time_now_cs() & RS_TIME1;                     /* TIME1, cs              */
    uint32_t elapsed = (now - (uint32_t) phase_tbase(group)) & RS_TIME1; /* TIME1 - TBASEn, cs     */
    int32_t  left    = dt - (int32_t) elapsed;                           /* time still to wait, cs */

    if (left < RS_SOON_CS)
    {
        left = RS_SOON_CS; /* overdue: in 1 cs  */
    }

    return ((uint16_t) left);
}

/**********************************************************************
 * ITSAJOB / CHKNOVAC / ITSNOVAC (C p.1416 / L p.1305): positive
 * priority FINDVAC, negative NOVAC with the complemented priority.
 *
 * ARGUMENTS:
 *   prdt -- PHSPRDTn / PRDTTAB: +prio FINDVAC, -prio NOVAC
 *   job -- job id (AGC_JOB_*): PHSNAMEn / the table 2CADR
 *
 * RETURNS: agc_status_t -- the FINDVAC / NOVAC status; BAD_ARG for a
 *          restart point that was never stored (prdt 0 or job NONE)
 **********************************************************************/
static agc_status_t start_job(int16_t prdt, agc_job_id_t job)
{
    if (!AGC_ASSERT(prdt != 0) || !AGC_ASSERT(job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st; /* FINDVAC / NOVAC outcome */

    if (prdt > 0)
    {
        st = agc_exec_findvac((agc_prio_t) prdt, job);
    }
    else
    {
        st = agc_exec_novac((agc_prio_t) (-prdt), job);
    }

    return (st);
}

/**********************************************************************
 * ITSLGCL2 / LONGCLCL (C pp.1417-1418 / L pp.1306-1307): LONGTIME = dt -
 * (TIME2,TIME1 - LONGBASE); if time is left, LONGCALL for the rest,
 * otherwise the task is overdue and is put on the waitlist in 1 cs.
 *
 * ARGUMENTS:
 *   dt_cs -- DP delta time from LONGBASE, cs
 *   task -- waitlist task id (AGC_TASK_*), its 2CADR
 *
 * RETURNS: agc_status_t -- the LONGCALL / WAITLIST status; BAD_ARG for a
 *          restart point that was never stored (task NONE)
 **********************************************************************/
static agc_status_t start_long(uint32_t dt_cs, agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE) || !AGC_ASSERT(dt_cs < AGC_TIME_MODULUS_CS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint32_t     elapsed = (agc_time_now_cs() - phase_longbase()) & (AGC_TIME_MODULUS_CS - 1U); /* TIME2,1 - LONGBASE, cs */
    int64_t      left    = (int64_t) dt_cs - (int64_t) elapsed;                                 /* LONGTIME, cs           */
    agc_status_t st;                                                                            /* LONGCALL / WAITLIST outcome */

    if (left > 0)
    {
        st = agc_waitlist_longcall((uint32_t) left, task);
    }
    else
    {
        st = agc_waitlist_call(RS_SOON_CS, task); /* IMEDIATE -3      */
    }

    return (st);
}

/**********************************************************************
 * TIMETEST / ITSINDIR / ITSWTLST (C pp.1415, 1418 / L pp.1304, 1307): a
 * table task, immediately, after a direct delta time or after one read
 * from the erasable the table names, measured from TBASEn.
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   e -- the table entry (a task)
 *
 * RETURNS: agc_status_t -- the WAITLIST status, or the erasable read's
 **********************************************************************/
static agc_status_t start_table_task(int group, const rst_entry_t *e)
{
    uint16_t     delay = RS_SOON_CS;    /* -0: IMEDIATE      */
    agc_status_t st    = AGC_STATUS_OK; /* erasable read, then WAITLIST outcome */

    if (e->dtsrc == RST_DT_DIRECT)
    {
        delay = findtime(group, e->prdt);
    }
    else if (e->dtsrc == RST_DT_INDIRECT)
    {
        int16_t dt = 0; /* C(-GENADR), cs */

        st    = rst_read_sp(e->src, &dt);
        delay = findtime(group, dt);
    }
    else
    {
        /* RST_DT_IMMEDIATE: in 1 cs */
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_waitlist_call(delay, e->task);
    }

    return (st);
}

/**********************************************************************
 * CONTBL2 (C p.1417 / L p.1306): restart one table entry.
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   e -- the table entry
 *
 * RETURNS: agc_status_t -- the restart's status; BAD_ARG for a DEC 0
 *          placeholder entry
 **********************************************************************/
static agc_status_t restart_entry(int group, const rst_entry_t *e)
{
    agc_status_t st = AGC_STATUS_BAD_ARG; /* restart outcome */
    uint32_t     dt = 0;                  /* LONGCALL delta time, cs */

    switch (e->kind)
    {
        case AGC_RESTART_JOB:
            st = start_job(e->prdt, e->job); /* ITSAJOB2          */
            break;
        case AGC_RESTART_TASK:
            st = start_table_task(group, e);
            break;
        case AGC_RESTART_LONGCALL:           /* ITSLGCL1          */
            st = rst_read_dp(e->src, &dt);

            if (st == AGC_STATUS_OK)
            {
                st = start_long(dt, e->task);
            }

            break;
        case AGC_RESTART_NONE:
        default:

            /* DEVIATION: a "DEC 0" placeholder; the AGC would start a job at
             * address 0. Nothing is restarted: BAD_ARG.                    */
            break;
    }

    return (st);
}

/**********************************************************************
 * GETPART2 / ITSATBL / ITSEVEN / PHSPART2 (C pp.1414-1419 / L pp.1303-
 * 1308): the fixed part of a phase. Both entries of an even phase are
 * restarted unless first_only (type B: TEMPSWCH = RTRNCADR).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   phase -- the table phase, 1..0177
 *   first_only -- type B: only the first entry of an even phase
 *
 * RETURNS: agc_status_t -- the first failing restart status, else OK;
 *          BAD_ARG for phase 0 (a CCS hole)
 **********************************************************************/
static agc_status_t restart_table(int group, int phase, bool first_only)
{
    if (!AGC_ASSERT(phase >= 1 && phase <= (int) RS_LOW7))
    {
        return (AGC_STATUS_BAD_ARG); /* phase 0: CCS hole */
    }

    rst_entry_t  e;  /* the table entry */
    agc_status_t st; /* first restart outcome */

    if (phase == 1)
    { /* X.1: last display */
        st = agc_exec_findvac(RS_PRIO14, AGC_JOB_INITDSP);
    }
    else
    {
        st = rst_table_lookup(group, phase, 0, &e);

        if (st == AGC_STATUS_OK)
        {
            st = restart_entry(group, &e);
        }

        if ((phase % 2) == 0 && !first_only)
        {
            agc_status_t st2 = rst_table_lookup(group, phase, 1, &e); /* second entry outcome */

            if (st2 == AGC_STATUS_OK)
            {
                st2 = restart_entry(group, &e);
            }

            if (st == AGC_STATUS_OK)
            {
                st = st2;
            }
        }
    }

    return (st);
} /* restart_table */

/**********************************************************************
 * ITSAVAR (C p.1414 / L p.1303): a type C phase -- the job, task or
 * LONGCALL stored in PHSNAMEn, as CJW names it ((CJW) - 2 > 0 LONGCALL,
 * 0 job, < 0 task).
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *   tempphs -- C(PHASEn)
 *   s -- the stored PHSNAMEn / PHSPRDTn
 *
 * RETURNS: agc_status_t -- the restart's status
 **********************************************************************/
static agc_status_t restart_variable(int group, uint16_t tempphs, const phase_var_store_t *s)
{
    int          cjw = (int) (tempphs & RS_CJW) - 2; /* AD MINUS2 / CCS   */
    agc_status_t st;                                 /* restart outcome */

    if (cjw > 0)
    { /* ITSLNGCL          */
        st = start_long(s->long_dt_cs, s->task);
    }
    else if (cjw == 0)
    { /* ITSAJOB           */
        st = start_job(s->prdt, s->job);
    }
    else
    {
        uint16_t delay = RS_SOON_CS; /* ITSAWAIT: -0 immediate */

        if (!s->immediate)
        {
            delay = findtime(group, s->prdt);
        }

        st = agc_waitlist_call(delay, s->task);
    }

    return (st);
}

/**********************************************************************
 * RESTARTS (C p.1414 / L p.1303): restart group `group` from its phase.
 *
 * ARGUMENTS:
 *   group -- restart group, 1..6
 *
 * RETURNS: agc_status_t -- the restart's status; NOT_FOUND when the group
 *          is inactive; BAD_ARG on a bad group
 **********************************************************************/
agc_status_t agc_restart_group(int group)
{
    if (!AGC_ASSERT(group >= 1 && group <= AGC_RESTART_GROUPS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t     tempphs = agc_phase_get(group); /* TEMPPHS = C(PHASEn) */
    agc_status_t st      = AGC_STATUS_NOT_FOUND; /* restart outcome; group inactive */

    if (tempphs == 0U)
    {
        /* inactive group */
    }
    else if ((tempphs & RS_VARBITS) == 0U)
    {
        st = restart_table(group, (int) tempphs, false);
    }
    else
    {
        phase_var_store_t s; /* PHSNAMEn / PHSPRDTn */

        phase_var_get(group, &s);

        if ((tempphs & RS_TYPEB) == 0U)
        {
            st = restart_variable(group, tempphs, &s);
        }
        else
        {
            st               = start_job(s.prdt, s.job);                              /* ITSLIKEB          */
            agc_status_t st2 = restart_table(group, (int) (tempphs & RS_LOW7), true); /* fixed part outcome */

            if (st == AGC_STATUS_OK)
            {
                st = st2;
            }
        }
    }

    return (st);
} /* agc_restart_group */

/**********************************************************************
 * NXTRST / PACTIVE / PINACT (FRESH_START_AND_RESTART.agc C p.189 /
 * L p.218): restart every active group, group 6 first. The loop goes on
 * after a failure.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the first failing restart status, else OK;
 *          NOT_FOUND when no group was active
 **********************************************************************/
agc_status_t agc_phase_restart_all(void)
{
    agc_status_t result = AGC_STATUS_OK; /* first failing restart status */
    bool         any    = false;         /* INCR MPAC +6: a group was active */

    for (int g = AGC_RESTART_GROUPS; g >= 1; g--)
    {
        if (agc_phase_get(g) != 0U)
        { /* CCS PHASEn: PNZ   */
            agc_status_t st = agc_restart_group(g); /* this group's outcome */

            any = true;

            if (st != AGC_STATUS_OK && result == AGC_STATUS_OK)
            {
                result = st;
            }
        }
    }

    if (!any)
    {
        result = AGC_STATUS_NOT_FOUND;
    }

    return (result);
}
