/*
 * agc_jobs.h -- registry of executive jobs and waitlist tasks.
 *
 * OWNED BY THE LEAD. Subagents list the jobs/tasks they need in their module
 * note under "Requested jobs/tasks"; the lead adds them here and to the
 * dispatch switch in src/exec/dispatch.c during integration.
 *
 * Until then, modules reference the symbols below; unknown ones are added
 * at integration time.
 */
#ifndef AGC_JOBS_H
#define AGC_JOBS_H

typedef enum agc_job_id {
    AGC_JOB_NONE = 0,
    AGC_JOB_DUMMY,             /* DUMMYJOB                                 */
    AGC_JOB_CHARIN,            /* pinball keyboard job                     */
    AGC_JOB_GOPROG,            /* fresh-start program selection            */
    /* exec-sched (src/exec/) */
    AGC_JOB_GOTOPOOH,          /* GOTOPOOH as a job (FRESH_START_AND_RESTART) */
    AGC_JOB_GOP00FIX,          /* GOP00FIX: flash V37 N99 (LM 4.33 restart)   */
    AGC_JOB_GOPOODOO,          /* GOPOODOO: POODOO tail (LM 4.35 restart)     */
    AGC_JOB_ENDOFJOB,          /* a job at ENDOFJOB (LM 1.2SPOT placeholder)  */
    AGC_JOB_COUNT_PLACEHOLDER  /* integration fills in the rest            */
} agc_job_id_t;

typedef enum agc_task_id {
    AGC_TASK_NONE = 0,
    AGC_TASK_ENDTASK,          /* ENDTASK sentinel                         */
    /* exec-sched (src/exec/) */
    AGC_TASK_SVCT3,            /* SVCT3 re-entry after SVCT3X FIXDELAY     */
    AGC_TASK_LONGCYCL,         /* LONGCALL 81.92 s cycle                   */
    AGC_TASK_GETCADR,          /* LONGCALL final dispatch of LONGCADR      */
    AGC_TASK_T5IDLOC,          /* CSM T5RUPT idler (every 163.84 s)        */
    AGC_TASK_TASKOVER,         /* a task at TASKOVER (LM 1.2SPOT placeholder) */
    AGC_TASK_COUNT_PLACEHOLDER
} agc_task_id_t;

#endif /* AGC_JOBS_H */
