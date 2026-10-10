/*
 * pending.h -- placeholder job/task ids for the nav-integ lane until
 * the lead adds them to include/agc/exec/jobs.h (see docs/modules/nav-integ.md
 * "Requested jobs-tasks"). Values are in a range no other lane uses (0x0F).
 */
#ifndef INTEG_PENDING_H
#define INTEG_PENDING_H

#include "agc/exec/exec.h"

/* STATINT1: FINDVAC PRIO5 from the STATEINT task (p. 1313).              */
#define AGC_JOB_STATINT1 ((agc_job_id_t) 0x0F01)
/* STATINT1 resumed at INTSTALL +1 after JOBSLEEP.                         */
#define AGC_JOB_STATINT1_STALL ((agc_job_id_t) 0x0F02)
/* STATEUP resumed at its second INTSTALL +1 (LM state) after JOBSLEEP.   */
#define AGC_JOB_STATEUP_STALL ((agc_job_id_t) 0x0F03)
/* STATEINT: LONGCALL 600SECS from ENDINT (p. 1314).                       */
#define AGC_TASK_STATEINT ((agc_task_id_t) 0x0F81)

/* FINDVAC priority PRIO5 (octal 05000 >> 9).                              */
#define AGC_INTEG_PRIO5 ((agc_prio_t) 5)

#endif /* INTEG_PENDING_H */
