/*
 * agc_exec.h -- EXECUTIVE, WAITLIST, restart and alarm services.
 *
 * Jobs and tasks are identified by enum (no function pointers, rule 9) and
 * dispatched by the switch in src/exec/dispatch.c. The enums live in
 * agc_jobs.h, which the lead integrates from each module's note.
 */
#ifndef AGC_EXEC_H
#define AGC_EXEC_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_jobs.h"

#define AGC_EXEC_MAX_JOBS      8   /* core sets (7 user + 1 dummy)          */
#define AGC_EXEC_MAX_VACS      5   /* VAC1USE..VAC5USE                      */
#define AGC_WAITLIST_MAX_TASKS 9   /* LST1 +0..+7 and the one in TIME3      */
#define AGC_WAITLIST_MAX_DELAY 16250 /* centiseconds, 162.5 s                */
#define AGC_VAC_WORDS          44  /* 43D + MPAC etc.; spill area size      */

typedef uint16_t agc_prio_t;       /* PRIO1..PRIO37 (octal) >> 9            */
typedef uint16_t agc_alarm_code_t; /* octal alarm code, e.g. 01201          */

typedef struct agc_job_ctx {
    agc_job_id_t id;
    agc_prio_t   prio;
    bool         has_vac;
    int          vac_index;        /* -1 when !has_vac                      */
    agc_sp_t     arg;              /* word passed in A/L on entry, if any   */
} agc_job_ctx_t;

/* Executive (EXECUTIVE.agc pp. 1208-1220) */
agc_status_t agc_exec_init(void);
agc_status_t agc_exec_novac(agc_prio_t prio, agc_job_id_t job);   /* NOVAC   */
agc_status_t agc_exec_findvac(agc_prio_t prio, agc_job_id_t job); /* FINDVAC */
agc_status_t agc_exec_job_sleep(agc_job_id_t wake_as);           /* JOBSLEEP*/
agc_status_t agc_exec_job_wake(agc_job_id_t job);                /* JOBWAKE */
agc_status_t agc_exec_prio_change(agc_prio_t new_prio);          /* PRIOCHNG*/
agc_status_t agc_exec_run_one(void);     /* run highest-priority ready job  */
bool         agc_exec_idle(void);        /* DUMMYJOB would run              */

/* Waitlist (WAITLIST.agc pp. 1221-1235). Delay in centiseconds.           */
agc_status_t agc_waitlist_init(void);
agc_status_t agc_waitlist_call(uint16_t delay_cs, agc_task_id_t task); /* WAITLIST/TWIDDLE */
agc_status_t agc_waitlist_vardelay(uint16_t delay_cs, agc_task_id_t task); /* VARDELAY (delay in A, 2cadr in L/Q) */
agc_status_t agc_waitlist_longcall(uint32_t delay_cs, agc_task_id_t task); /* LONGCALL (DP delay) */
agc_status_t agc_waitlist_tick(uint16_t elapsed_cs); /* T3RUPT; runs due tasks */
agc_status_t agc_waitlist_fixdelay(uint16_t delay_cs, agc_task_id_t task); /* FIXDELAY */

/* Interrupt inhibit (INHINT/RELINT) -- counted, no-op in single thread.    */
void agc_inhint(void);
void agc_relint(void);
bool agc_interrupts_inhibited(void);

/* Alarms and aborts (ALARM_AND_ABORT.agc pp. 1493-1496).                   */
agc_status_t agc_alarm(agc_alarm_code_t code);    /* ALARM: light, store   */
agc_status_t agc_prioalarm(agc_alarm_code_t code);/* PRIOLARM              */
agc_status_t agc_bailout(agc_alarm_code_t code);  /* BAILOUT -> restart    */
agc_status_t agc_poodoo(agc_alarm_code_t code);   /* POODOO  -> P00        */
agc_status_t agc_abort_restart(void);             /* ABORT                 */

/* Phase table maintenance (PHASE_TABLE_MAINTENANCE.agc pp. 1404-1413).    */
#define AGC_RESTART_GROUPS 6
agc_status_t agc_phase_change(int group, uint16_t phase_code);   /* PHASCHNG */
agc_status_t agc_phase_new(int group, agc_job_id_t job, agc_prio_t prio); /* NEWPHASE */
agc_status_t agc_phase_restart_all(void);                        /* RESTART  */

#endif /* AGC_EXEC_H */
