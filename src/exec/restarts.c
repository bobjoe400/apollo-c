/*
 * restarts.c -- RESTARTS: re-establish the jobs, tasks and LONGCALLs that
 *               the phase table says were in progress.
 *
 * Source: RESTARTS_ROUTINE.agc, Comanche055 pp. 1414-1419, Luminary099
 * pp. 1303-1308 (identical programs), and the NXTRST loop of GOPROG3
 * (FRESH_START_AND_RESTART C p.189 / L p.218).
 *
 * For an active group n with PHASEn = p:
 *   p = 1           the last display is restarted: FINDVAC PRIO14 INITDSP;
 *   p odd  >= 3     the single odd-table entry is restarted;
 *   p even >= 2     both even-table entries are restarted;
 *   type C variable PHSNAMEn is restarted as a job (PHSPRDTn > 0 FINDVAC,
 *                   < 0 NOVAC), a task or a LONGCALL according to CJW;
 *   type B          the variable job, then the first (or only) table entry
 *                   of phase p & 177.
 * A task restarts at the time it would have run: its delta time is
 * measured from TBASEn (FINDTIME), and if that time has passed it starts
 * in 1 cs. A LONGCALL is measured from LONGBASE the same way.
 */
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/exec/pending.h"
#include "internal.h"

#define RS_VARBITS 01400u /* OCT1400: type B (01000) or C (0400)    */
#define RS_TYPEB   01000u
#define RS_LOW7    0177u   /* OCT177 = LOW7: table phase of type B   */
#define RS_TIME1   037777u /* TIME1 wraps at 2^14 cs                 */
#define RS_PRIO14  ((agc_prio_t) 014)
#define RS_SOON_CS 1 /* IMEDIATE: AD ONE -> 1 cs               */

/* FINDTIME / IMEDIATE (C pp.1415-1416): a task due dt cs after TBASEn
 * is called max(1, dt - (TIME1 - TBASEn)) cs from now (TIME1 elapsed
 * time modulo 2^14 cs, as the AGC's single-word subtraction gives).   */
static uint16_t findtime(int group, int32_t dt)
{
    if (!AGC_ASSERT(dt >= 0 && dt <= (int32_t) RS_TIME1))
    {
        return RS_SOON_CS;
    }
    uint32_t now     = agc_time_now_cs() & RS_TIME1;
    uint32_t elapsed = (now - (uint32_t) phase_tbase(group)) & RS_TIME1;
    int32_t  left    = dt - (int32_t) elapsed;
    if (left < RS_SOON_CS)
    {
        left = RS_SOON_CS; /* overdue: in 1 cs  */
    }
    if (!AGC_ASSERT(left <= (int32_t) RS_TIME1))
    {
        return RS_SOON_CS;
    }
    return (uint16_t) left;
}

/* ITSAJOB / CHKNOVAC / ITSNOVAC (C p.1416): positive priority FINDVAC,
 * negative NOVAC with the complemented priority.                      */
static agc_status_t start_job(int16_t prdt, agc_job_id_t job)
{
    if (!AGC_ASSERT(prdt != 0) || !AGC_ASSERT(job != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (prdt > 0)
    {
        return agc_exec_findvac((agc_prio_t) prdt, job);
    }
    return agc_exec_novac((agc_prio_t) (-prdt), job);
}

/* ITSLGCL2 / LONGCLCL (C pp.1417-1418): LONGTIME = dt - (TIME2,TIME1 -
 * LONGBASE); if time is left, LONGCALL for the rest, otherwise the task
 * is overdue and is put on the waitlist in 1 cs.                      */
static agc_status_t start_long(uint32_t dt_cs, agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE) ||
        !AGC_ASSERT(dt_cs < AGC_TIME_MODULUS_CS))
    {
        return AGC_STATUS_BAD_ARG;
    }
    uint32_t elapsed =
        (agc_time_now_cs() - phase_longbase()) & (AGC_TIME_MODULUS_CS - 1U);
    int64_t left = (int64_t) dt_cs - (int64_t) elapsed;
    if (left > 0)
    {
        return agc_waitlist_longcall((uint32_t) left, task);
    }
    return agc_waitlist_call(RS_SOON_CS, task); /* IMEDIATE -3      */
}

/* TIMETEST / ITSINDIR / ITSWTLST (C pp.1415, 1418): a table task.     */
static agc_status_t start_table_task(int group, const rst_entry_t *e)
{
    if (!AGC_ASSERT(e != 0) || !AGC_ASSERT(e->task != AGC_TASK_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    uint16_t delay = RS_SOON_CS; /* -0: IMEDIATE      */
    if (e->dtsrc == RST_DT_DIRECT)
    {
        delay = findtime(group, e->prdt);
    }
    else if (e->dtsrc == RST_DT_INDIRECT)
    {
        int16_t      dt = 0;
        agc_status_t st = rst_read_sp(e->src, &dt);
        if (st != AGC_STATUS_OK)
        {
            return st;
        }
        delay = findtime(group, dt);
    }
    else
    {
        if (!AGC_ASSERT(e->dtsrc == RST_DT_IMMEDIATE))
        {
            return AGC_STATUS_BAD_ARG;
        }
    }
    return agc_waitlist_call(delay, e->task);
}

/* CONTBL2 (C p.1417): restart one table entry.                        */
static agc_status_t restart_entry(int group, const rst_entry_t *e)
{
    if (!AGC_ASSERT(e != 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = AGC_STATUS_NOT_FOUND;
    uint32_t     dt = 0;
    switch (e->kind)
    {
        case AGC_RESTART_JOB:
            st = start_job(e->prdt, e->job); /* ITSAJOB2          */
            break;
        case AGC_RESTART_TASK:
            st = start_table_task(group, e);
            break;
        case AGC_RESTART_LONGCALL: /* ITSLGCL1          */
            st = rst_read_dp(e->src, &dt);
            if (st == AGC_STATUS_OK)
            {
                st = start_long(dt, e->task);
            }
            break;
        case AGC_RESTART_NONE:
        default:
            /* DEVIATION: a "DEC 0" placeholder; the AGC would start a job at
         * address 0. Nothing is restarted.                             */
            if (!AGC_ASSERT(e->kind != AGC_RESTART_NONE))
            {
                return AGC_STATUS_BAD_ARG;
            }
            break;
    }
    return st;
}

/* GETPART2 / ITSATBL / ITSEVEN / PHSPART2 (C pp.1414-1419): the fixed
 * part of a phase. first_only: type B takes only the first entry of an
 * even phase (TEMPSWCH = RTRNCADR).                                   */
static agc_status_t restart_table(int group, int phase, bool first_only)
{
    if (!AGC_ASSERT(phase >= 1 && phase <= (int) RS_LOW7))
    {
        return AGC_STATUS_BAD_ARG; /* phase 0: CCS hole */
    }
    if (phase == 1)
    { /* X.1: last display */
        return agc_exec_findvac(RS_PRIO14, AGC_JOB_INITDSP);
    }
    rst_entry_t  e;
    agc_status_t st = rst_table_lookup(group, phase, 0, &e);
    if (st == AGC_STATUS_OK)
    {
        st = restart_entry(group, &e);
    }
    if ((phase % 2) == 0 && !first_only)
    {
        agc_status_t st2 = rst_table_lookup(group, phase, 1, &e);
        if (st2 == AGC_STATUS_OK)
        {
            st2 = restart_entry(group, &e);
        }
        st = (st == AGC_STATUS_OK) ? st2 : st;
    }
    return st;
}

/* ITSAVAR (C p.1414): type C -- job, task or LONGCALL from PHSNAMEn.   */
static agc_status_t restart_type_c(int group, uint16_t tempphs,
                                   const phase_var_store_t *s)
{
    if (!AGC_ASSERT(s != 0) || !AGC_ASSERT((tempphs & RS_TYPEB) == 0U))
    {
        return AGC_STATUS_BAD_ARG;
    }
    int cjw = (int) (tempphs & 07U) - 2; /* AD MINUS2 / CCS   */
    if (cjw > 0)
    { /* ITSLNGCL          */
        return start_long(s->long_dt_cs, s->task);
    }
    if (cjw == 0)
    { /* ITSAJOB           */
        return start_job(s->prdt, s->job);
    }
    uint16_t delay = RS_SOON_CS; /* ITSAWAIT          */
    if (!s->immediate)
    {
        delay = findtime(group, s->prdt);
    }
    return agc_waitlist_call(delay, s->task);
}

/* RESTARTS (C p.1414 / L p.1303): restart group `group` from its phase. */
agc_status_t agc_restart_group(int group)
{
    if (!AGC_ASSERT(group >= 1 && group <= AGC_RESTART_GROUPS))
    {
        return AGC_STATUS_BAD_ARG;
    }
    uint16_t tempphs = phase_word(group);
    if (tempphs == 0U)
    {
        return AGC_STATUS_NOT_FOUND; /* group inactive    */
    }
    if ((tempphs & RS_VARBITS) == 0U)
    {
        return restart_table(group, (int) tempphs, false);
    }
    phase_var_store_t s;
    agc_status_t      st = phase_var_get(group, &s);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if ((tempphs & RS_TYPEB) == 0U)
    {
        return restart_type_c(group, tempphs, &s);
    }
    st               = start_job(s.prdt, s.job); /* ITSLIKEB          */
    agc_status_t st2 = restart_table(group, (int) (tempphs & RS_LOW7), true);
    return (st == AGC_STATUS_OK) ? st2 : st;
}

/* NXTRST / PACTIVE / PINACT (FRESH_START C p.189 / L p.218): restart
 * every active group, group 6 first. NOT_FOUND if none was active.
 * The first failing restart status is returned; the loop goes on.     */
agc_status_t agc_phase_restart_all(void)
{
    agc_status_t result = AGC_STATUS_OK;
    bool         any    = false;
    for (int g = AGC_RESTART_GROUPS; g >= 1; g--)
    {
        if (phase_word(g) != 0U)
        {                           /* CCS PHASEn: PNZ   */
            any             = true; /* INCR MPAC +6      */
            agc_status_t st = agc_restart_group(g);
            if (st != AGC_STATUS_OK && result == AGC_STATUS_OK)
            {
                result = st;
            }
        }
    }
    if (!AGC_ASSERT(result != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return any ? result : AGC_STATUS_NOT_FOUND;
}
