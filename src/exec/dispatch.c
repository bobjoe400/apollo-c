/*
 * dispatch.c -- the one switch from agc_job_id_t to the job functions,
 *               the one from agc_task_id_t to the task functions, and
 *               the interrupt handlers of other lanes.
 *
 * This file replaces the AGC's 2CADRs (rule 9: no function pointers).
 * It is the single place where src/exec/ calls into every other layer.
 *
 * INTEGRATION: only the ids that are members of agc_jobs.h today have a
 * case. The provisional ids of include/agc/agc_exec_pending.h and of the other
 * lanes' *_pending.h headers fall to `default` (NOT_FOUND) until the
 * lead adds them to agc_jobs.h and here.
 */
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/programs/alignment.h"

/* The job in core set 0 is started (or resumed) at ctx->id.           */
agc_status_t agc_dispatch_job(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_NOT_FOUND;

    switch (ctx->id)
    {
        case AGC_JOB_DUMMY:
            st = AGC_STATUS_OK;
            break;
        case AGC_JOB_CHARIN:
            st = agc_job_charin(ctx);
            break;
        case AGC_JOB_GOTOPOOH:
            st = agc_job_gotopooh(ctx);
            break;
        case AGC_JOB_GO_IDLE_FIX:
            st = agc_job_go_idle_fix(ctx);
            break;
        case AGC_JOB_GO_IDLE_FIX_REFLASH:
            st = agc_job_go_idle_fix_reflash(ctx);
            break;
        case AGC_JOB_GOPOODOO:
            st = agc_job_gopoodoo(ctx);
            break;
        case AGC_JOB_ENDOFJOB:
            st = agc_job_endofjob(ctx);
            break;
        case AGC_JOB_GOPROG:
            /* registry placeholder: no AGC job of this name (GOPROG is the
             * GOJAM restart entry, agc_rupt_go).                          */
            st = AGC_STATUS_UNIMPLEMENTED;
            break;
        case AGC_JOB_NONE:
        case AGC_JOB_COUNT_PLACEHOLDER:

            if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            st = AGC_STATUS_BAD_ARG;
            break;
        default:
            st = AGC_STATUS_NOT_FOUND; /* provisional id, not integrated  */
            break;
    }

    return (st);
}

/* The task at the head of LST2 (or T5LOC / T6LOC) is called.          */
agc_status_t agc_dispatch_task(agc_task_id_t task)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_NOT_FOUND;

    switch (task)
    {
        case AGC_TASK_ENDTASK:
            st = agc_task_endtask();
            break;
        case AGC_TASK_SVCT3:
            st = agc_task_svct3();
            break;
        case AGC_TASK_LONGCYCL:
            st = agc_task_longcycl();
            break;
        case AGC_TASK_GETCADR:
            st = agc_task_getcadr();
            break;
        case AGC_TASK_T5IDLOC:
            st = agc_task_t5idloc();
            break;
        case AGC_TASK_TASKOVER:
            st = agc_task_taskover();
            break;
        case AGC_TASK_NONE:
        case AGC_TASK_COUNT_PLACEHOLDER:

            if (!AGC_ASSERT(task != AGC_TASK_NONE))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            st = AGC_STATUS_BAD_ARG;
            break;
        default:
            st = AGC_STATUS_NOT_FOUND; /* provisional id, not integrated  */
            break;
    }

    return (st);
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
        case AGC_RUPT_T6: /* LM DOT6RUPT; CSM goes through T6LOC     */
            st = lm ? agc_lm_dap_t6rupt() : AGC_STATUS_NOT_FOUND;
            break;
        case AGC_RUPT_T5: /* T5LOC / T5ADR                           */
            st = agc_dispatch_task(agc_rupt_t5_task());
            break;
        case AGC_RUPT_T3:
            st = agc_waitlist_t3rupt();
            break;
        case AGC_RUPT_KEY1:
            st = agc_dsky_keyrupt1();
            break;
        case AGC_RUPT_KEY2: /* MARKRUPT: CSM sextant, LM AOT           */
            st = lm ? agc_aotmark_markrupt() : agc_sxtmark_markrupt();
            break;
        case AGC_RUPT_HAND: /* LM PITFALL; CSM RESUME +3 (not used)    */
            st = lm ? agc_lm_pitfall() : AGC_STATUS_OK;
            break;
        case AGC_RUPT_T4:       /* T4RUPT: sys-misc, header not public     */
        case AGC_RUPT_UPRUPT:   /* UPRUPT: needs INLINK (io-pinball)       */
        case AGC_RUPT_DOWNRUPT: /* DODOWNTM: sys-misc, header not public   */
        case AGC_RUPT_RADAR:    /* CSM VHFREAD / LM RADAREAD: unpublished  */
        default:
            st = AGC_STATUS_UNIMPLEMENTED;
            break;
    }

    if (!AGC_ASSERT(rupt >= AGC_RUPT_T6 && rupt <= AGC_RUPT_HAND))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}
