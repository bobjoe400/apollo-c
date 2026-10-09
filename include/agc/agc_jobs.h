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
    AGC_JOB_COUNT_PLACEHOLDER  /* integration fills in the rest            */
} agc_job_id_t;

typedef enum agc_task_id {
    AGC_TASK_NONE = 0,
    AGC_TASK_ENDTASK,          /* ENDTASK sentinel                         */
    AGC_TASK_COUNT_PLACEHOLDER
} agc_task_id_t;

#endif /* AGC_JOBS_H */
