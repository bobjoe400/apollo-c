/**********************************************************************
 * FILE NAME: exec.h
 *
 * PURPOSE:
 *   EXECUTIVE, WAITLIST, phase tables, restarts, alarms and aborts,
 *   fresh start, interrupt lead-ins: the public contract of src/exec/.
 *   Jobs and tasks are identified by enum (no function pointers, rule 9)
 *   and dispatched by the switches in src/exec/dispatch.c; the enums live
 *   in agc/exec/jobs.h, generated from tools/registry.tsv.
 *
 * AGC SOURCE:
 *   EXECUTIVE.agc, WAITLIST.agc, PHASE_TABLE_MAINTENANCE.agc,
 *   RESTARTS_ROUTINE.agc, RESTART_TABLES.agc, FRESH_START_AND_RESTART.agc,
 *   ALARM_AND_ABORT.agc, INTERRUPT_LEAD_INS.agc (Comanche055, Luminary099)
 *
 * REQUIRES:
 *   agc/core/state.h, agc/core/status.h, agc/core/word.h, agc/exec/jobs.h
 *
 * GLOBAL VARIABLES:
 *   none (the executive, waitlist and phase tables are reached through
 *   the functions below)
 **********************************************************************/
#ifndef AGC_EXEC_H
#define AGC_EXEC_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/jobs.h"

#define AGC_EXEC_MAX_JOBS      8     /* core sets (7 user + 1 dummy)          */
#define AGC_EXEC_MAX_VACS      5     /* VAC1USE..VAC5USE                      */
#define AGC_WAITLIST_MAX_TASKS 9     /* LST1 +0..+7 and the one in TIME3      */
#define AGC_WAITLIST_MAX_DELAY 16250 /* centiseconds, 162.5 s                */
#define AGC_VAC_WORDS          44    /* 43D + MPAC etc.; spill area size      */

typedef uint16_t agc_prio_t;       /* PRIO1..PRIO37 (octal) >> 9            */
typedef uint16_t agc_alarm_code_t; /* octal alarm code, e.g. 01201          */

typedef struct agc_job_ctx
{
    agc_job_id_t id;        /* LOC: the job being run                */
    agc_prio_t   prio;      /* priority number (PRIOnn >> 9)         */
    bool         has_vac;   /* FINDVAC job (a VAC area is claimed)   */
    int          vac_index; /* -1 when !has_vac                      */
    agc_sp_t     arg;       /* word passed in A/L on entry, if any; raw SP (B0) */
} agc_job_ctx_t;

/* Priorities: agc_prio_t = PRIOnn (octal) >> 9, i.e. the "nn" in octal.     */
#define AGC_PRIO5  ((agc_prio_t) 005) /* PRIO5 */
#define AGC_PRIO6  ((agc_prio_t) 006) /* PRIO6 */
#define AGC_PRIO10 ((agc_prio_t) 010) /* PRIO10 */
#define AGC_PRIO12 ((agc_prio_t) 012) /* PRIO12 */
#define AGC_PRIO13 ((agc_prio_t) 013) /* PRIO13 */
#define AGC_PRIO14 ((agc_prio_t) 014) /* PRIO14 */
#define AGC_PRIO15 ((agc_prio_t) 015) /* PRIO15 */
#define AGC_PRIO16 ((agc_prio_t) 016) /* PRIO16 */
#define AGC_PRIO17 ((agc_prio_t) 017) /* PRIO17 */
#define AGC_PRIO20 ((agc_prio_t) 020) /* PRIO20 */
#define AGC_PRIO21 ((agc_prio_t) 021) /* PRIO21 */
#define AGC_PRIO22 ((agc_prio_t) 022) /* PRIO22 */
#define AGC_PRIO23 ((agc_prio_t) 023) /* PRIO23 */
#define AGC_PRIO25 ((agc_prio_t) 025) /* PRIO25 */
#define AGC_PRIO27 ((agc_prio_t) 027) /* PRIO27 */
#define AGC_PRIO30 ((agc_prio_t) 030) /* PRIO30 */
#define AGC_PRIO31 ((agc_prio_t) 031) /* PRIO31 */
#define AGC_PRIO32 ((agc_prio_t) 032) /* PRIO32 */

/* Executive (EXECUTIVE.agc pp. 1208-1220) */
void         agc_exec_init(void);
agc_status_t agc_exec_novac(agc_prio_t prio, agc_job_id_t job);   /* NOVAC   */
agc_status_t agc_exec_findvac(agc_prio_t prio, agc_job_id_t job); /* FINDVAC */
void         agc_exec_job_sleep(agc_job_id_t wake_as);            /* JOBSLEEP*/
agc_status_t agc_exec_job_wake(agc_job_id_t job);                 /* JOBWAKE */

/* JOBWAKE where the caller does not care whether the job is still asleep:
 * the AGC's JOBWAKE2 just "EXIT[S] IF SLEEPING JOB NOT FOUND" (LOCCTR -1). */
agc_status_t agc_exec_job_wake_if_asleep(agc_job_id_t job);

/* JOBWAKE the job sleeping at `loc`, then INDEX LOCCTR / TS LOC: it
 * resumes at `new_loc` (pinball RECAL2, XCHNYLOC). NOT_FOUND: no job
 * sleeps at loc.                                                          */
agc_status_t agc_exec_job_wake_as(agc_job_id_t loc, agc_job_id_t new_loc);

/* MARKOK/MKVAC (marking system): the first free VACnUSE is marked in use
 * (the CCS VACnUSE loop of FINDVAC without a job). FULL when none (the
 * caller BAILOUTs 1207). Release stores the VAC address back (TS VACnUSE). */
agc_status_t agc_exec_vac_claim(int *vac_index);
agc_status_t agc_exec_vac_release(int vac_index);
void         agc_exec_prio_change(agc_prio_t new_prio);                /* PRIOCHNG*/
agc_status_t agc_exec_jump(const agc_job_ctx_t *ctx, agc_job_id_t id); /* TC/POSTJUMP to a job body */
agc_status_t agc_exec_run_one(void);                                   /* run highest-priority ready job  */
bool         agc_exec_idle(void);                                      /* DUMMYJOB would run              */

/* Waitlist (WAITLIST.agc pp. 1221-1235). Delay in centiseconds.           */
void         agc_waitlist_init(void);
agc_status_t agc_waitlist_call(uint16_t delay_cs, agc_task_id_t task);     /* WAITLIST/TWIDDLE */
agc_status_t agc_waitlist_vardelay(uint16_t delay_cs, agc_task_id_t task); /* VARDELAY (delay in A, 2cadr in L/Q) */
agc_status_t agc_waitlist_longcall(uint32_t delay_cs, agc_task_id_t task); /* LONGCALL (DP delay) */
agc_status_t agc_waitlist_tick(uint16_t elapsed_cs);                       /* T3RUPT; runs due tasks */
agc_status_t agc_waitlist_fixdelay(uint16_t delay_cs, agc_task_id_t task); /* FIXDELAY */

/* KILLTASK (BURN_BABY_BURN p.750-751): the first LST2 entry naming `task`
 * becomes TASKOVER (the time slot stays); no action when there is none.  */
void agc_waitlist_kill(agc_task_id_t task);

/* Interrupt inhibit (INHINT/RELINT) -- counted, no-op in single thread.    */
void agc_inhint(void);
void agc_relint(void);
bool agc_interrupts_inhibited(void);

/* Alarms and aborts (ALARM_AND_ABORT.agc pp. 1493-1496).                   */
void         agc_alarm(agc_alarm_code_t code);     /* ALARM: light, store   */
void         agc_prioalarm(agc_alarm_code_t code); /* PRIOLARM              */
agc_status_t agc_bailout(agc_alarm_code_t code);   /* BAILOUT -> restart    */
agc_status_t agc_poodoo(agc_alarm_code_t code);    /* POODOO  -> P00        */
agc_job_id_t agc_alarm_almcadr(void);              /* ALMCADR: alarming job */
agc_status_t agc_abort_restart(void);              /* ABORT                 */

/* Phase table maintenance (PHASE_TABLE_MAINTENANCE.agc pp. 1404-1413).    */
#define AGC_RESTART_GROUPS 6                                              /* restart groups 1..6 (PHASE1..PHASE6) */
void         agc_phase_change(uint16_t phase_code);                       /* PHASCHNG: GGG in the code */
agc_status_t agc_phase_new(int group, agc_job_id_t job, agc_prio_t prio); /* NEWPHASE */

/* CS TIME1 / TS TBASEn without a phase change (e.g. TBASE2 for R10,R11). */
void agc_phase_set_tbase(int group);

/* CAF PRIOnn / TS PHSPRDTn without a phase change: the priority a later
 * type B/C PHASCHNG of `group` with D clear keeps (FINDVAC restart).     */
void         agc_phase_set_priority(int group, agc_prio_t prio);
agc_status_t agc_phase_restart_all(void); /* RESTART  */

/* ======================================================================
 * Additions by exec-sched (see docs/modules/exec-sched.md).
 * ====================================================================== */

/* Executive core sets: Comanche055 has 7 (NO.CORES = 6), Luminary099 has
 * 8 (NO.CORES = 7, EJSCAN also tests PRIORITY +84D). DUMMYJOB uses none.  */
#define AGC_EXEC_CORE_SETS_CSM 7 /* Comanche: NO.CORES = 6, + 1 */
#define AGC_EXEC_CORE_SETS_LM  8 /* Luminary: NO.CORES = 7, + 1 */

/* Alarm / abort codes raised by the exec lane (octal, as in the listing). */
#define AGC_ALARM_00217 \
    ((agc_alarm_code_t) 00217) /* CURTAINS                 */
#define AGC_ALARM_01103 \
    ((agc_alarm_code_t) 01103) /* unused CCS branch (POODOO)*/
#define AGC_ALARM_01107 \
    ((agc_alarm_code_t) 01107) /* phase table failure      */
#define AGC_ALARM_01201 \
    ((agc_alarm_code_t) 01201) /* exec overflow: no VAC    */
#define AGC_ALARM_01202 \
    ((agc_alarm_code_t) 01202) /* exec overflow: no core set */
#define AGC_ALARM_01203 \
    ((agc_alarm_code_t) 01203) /* waitlist overflow        */
#define AGC_ALARM_01204 \
    ((agc_alarm_code_t) 01204) /* waitlist dt <= 0 (POODOO)*/
#define AGC_ALARM_01520 \
    ((agc_alarm_code_t) 01520) /* V37 not permitted now    */

/* Executive: pre-emption point and job-creation details.                  */
agc_status_t agc_exec_job_yield(agc_job_id_t resume_as); /* CHANG1/CHANG2 */
agc_status_t agc_exec_set_new_job_arg(agc_sp_t arg);     /* INDEX LOCCTR / TS MPAC */
agc_status_t agc_exec_gotopooh(void);                    /* GOTOPOOH from inside a job      */

typedef enum agc_core_state
{
    AGC_CORE_FREE = 0, /* PRIORITY = -0                            */
    AGC_CORE_ACTIVE,   /* PRIORITY positive                        */
    AGC_CORE_SLEEPING  /* PRIORITY negative (JOBSLEEP)             */
} agc_core_state_t;

typedef struct agc_exec_core_info
{
    agc_core_state_t state;     /* free, active or sleeping (PRIORITY sign) */
    agc_prio_t       prio;      /* priority number (PRIOnn >> 9)            */
    int              vac_index; /* 0..4, or -1 for a NOVAC job              */
    agc_job_id_t     loc;       /* LOC: where the job starts / resumes      */
} agc_exec_core_info_t;

int  agc_exec_core_sets(void); /* 7 CSM, 8 LM            */
void agc_exec_core_info(int core_set, agc_exec_core_info_t *out);
int  agc_exec_newjob(void);    /* NEWJOB: -1 idle (-0), 0 run set 0, k change */

/* Job/task dispatch (src/exec/dispatch.c, the one switch each).          */
agc_status_t agc_dispatch_job(agc_job_ctx_t *ctx);
agc_status_t agc_dispatch_task(agc_task_id_t task);

/* Noun data held above the io layer (src/exec/nouns.c): read (write =
 * false) or store the erasable behind a noun data id (agc_noun_data_t).
 * Value convention of include/agc/io/nouns.h. NOT_FOUND: no owner.        */
agc_status_t agc_dispatch_noun(uint16_t data_id, bool write, agc_dp_t *value);

/* Waitlist: T3RUPT entry for the interrupt lead-in (TIME3 just overflowed).*/
agc_status_t agc_waitlist_t3rupt(void);
uint16_t     agc_waitlist_time3(void);                                      /* C(TIME3)             */
void         agc_waitlist_entry(int i, int16_t *lst1, agc_task_id_t *lst2); /* LST1+i-1 (i>=1) / LST2+2i */

/* Phase tables. Codes are the AGC octal words that follow TC PHASCHNG:
 *   type A  TL0 00P PPP PPP GGG    type B  TL1 DAP PPP PPP GGG
 *   type C  TL0 1AD XXX CJW GGG                                           */
typedef enum agc_restart_kind
{
    AGC_RESTART_NONE = 0,
    AGC_RESTART_JOB,     /* NOVAC (priority < 0) or FINDVAC (> 0)    */
    AGC_RESTART_TASK,    /* WAITLIST                                 */
    AGC_RESTART_LONGCALL /* LONGCALL from LONGBASE                   */
} agc_restart_kind_t;

/* The restart point of a variable (type B/C) phase. The AGC takes it from
 * the return address of PHASCHNG or a 2CADR after the code word; in C it is
 * named explicitly.                                                       */
typedef struct agc_phase_var
{
    agc_restart_kind_t kind;       /* job, task or LONGCALL                    */
    agc_job_id_t       job;        /* kind == JOB                              */
    agc_task_id_t      task;       /* kind == TASK or LONGCALL                 */
    int16_t            prio_or_dt; /* D bit set: job +prio FINDVAC / -prio NOVAC (priority number); task delta time cs */
    bool               immediate;  /* task: PHSPRDT = -0, restart immediately  */
    uint32_t           long_dt_cs; /* longcall: DP delta time, cs              */
} agc_phase_var_t;

void         agc_phase_change_var(uint16_t code, const agc_phase_var_t *v);                 /* PHASCHNG type B/C */
void         agc_phase_change2(uint16_t code_a, uint16_t code_b, const agc_phase_var_t *v); /* 2PHSCHNG */
void         agc_phase_newphase(int group, uint16_t phase, bool set_tbase);                 /* NEWPHASE */
uint16_t     agc_phase_get(int group);                                                      /* C(PHASEn)            */
uint16_t     agc_phase_table_word(int group, bool negative);                                /* -PHASEn / PHASEn word (address map) */
void         agc_phase_table_set_word(int group, bool negative, uint16_t w);
uint16_t     agc_phase_tbase_word(int group);                                               /* TBASEn, TIME1 bits */
void         agc_phase_set_tbase_word(int group, uint16_t w);
uint32_t     agc_phase_longbase(void);                                                      /* LONGBASE, cs     */
void         agc_phase_set_longbase(uint32_t t_cs);
void         agc_phase_mr_klean(void);                                                      /* MR.KLEAN             */
void         agc_phase_idle_klean(void);                                                    /* P00KLEAN             */
void         agc_phase_change_program_klean(void);                                          /* V37KLEAN             */
agc_status_t agc_phase_newmode(agc_sp_t mode);                                              /* NEWMODEA/NEWMODEX    */
bool         agc_phase_checkmm(agc_sp_t mode);                                              /* CHECKMM              */
agc_status_t agc_restart_group(int group);                                                  /* RESTARTS (one group) */

/* Alarms.                                                                 */
void         agc_curtains(void); /* CURTAINS: ALARM 217  */
agc_status_t agc_ccshole(void);  /* CCSHOLE: POODOO 1103 */

/* Fresh start and restart (FRESH_START_AND_RESTART.agc).                  */
void         agc_fresh_start_slap1(void);        /* SLAP1 (V36)          */
void         agc_fresh_dofstart(void);           /* DOFSTART             */
void         agc_power_on(agc_vehicle_t v);      /* host: state, lanes, exec */
agc_status_t agc_restart_goprog(void);           /* GOPROG (GOJAM)       */
uint16_t     agc_restart_redoctr(void);          /* REDOCTR              */
agc_status_t agc_change_program(uint8_t new_mm); /* V37 (MMCHANG)      */

/* Interrupt lead-ins (INTERRUPT_LEAD_INS.agc), one per vector, for the sim
 * harness. Each runs with interrupts inhibited, like the AGC's RUPT.       */
typedef enum agc_rupt_id
{
    AGC_RUPT_T6 = 1,   /* T6RUPT   -> T6ADR / T6LOC               */
    AGC_RUPT_T5,       /* T5RUPT   -> T5ADR / T5LOC (autopilot)   */
    AGC_RUPT_T3,       /* T3RUPT   -> waitlist                    */
    AGC_RUPT_T4,       /* T4RUPT                                  */
    AGC_RUPT_KEY1,     /* KEYRUPT1 (main DSKY)                    */
    AGC_RUPT_KEY2,     /* KEYRUPT2 (MARKRUPT / nav DSKY)          */
    AGC_RUPT_UPRUPT,   /* UPRUPT                                  */
    AGC_RUPT_DOWNRUPT, /* DOWNRUPT (DODOWNTM)                     */
    AGC_RUPT_RADAR,    /* RADAR RUPT (LM RADAREAD, CSM VHFREAD)   */
    AGC_RUPT_HAND      /* RUPT10 (LM PITFALL, CSM RESUME)         */
} agc_rupt_id_t;

agc_status_t  agc_rupt_go(void);                        /* GOJAM -> GOPROG      */
agc_status_t  agc_rupt_t6(void);
agc_status_t  agc_rupt_t5(void);
agc_status_t  agc_rupt_t3(void);
agc_status_t  agc_rupt_t4(void);
agc_status_t  agc_rupt_keyrupt1(void);
agc_status_t  agc_rupt_keyrupt2(void);
agc_status_t  agc_rupt_uprupt(void);
agc_status_t  agc_rupt_downrupt(void);
agc_status_t  agc_rupt_radarupt(void);
agc_status_t  agc_rupt_rupt10(void);
void          agc_rupt_set_t5_task(agc_task_id_t task); /* DXCH T5ADR/T5LOC */
void          agc_rupt_set_t6_task(agc_task_id_t task); /* T6LOC (CSM)      */
agc_task_id_t agc_rupt_t5_task(void);

/* ---- added by exec-sched (second pass) -------------------------------- */
/* Software restart requests. BAILOUT, POODOO and ABORT end in WHIMPER ->
 * ENEMA; in C the request is recorded and ENEMA runs at the next service
 * point (end of the running job, end of T3RUPT, end of an interrupt
 * lead-in, or the sim harness), so that ENEMA never runs inside the job
 * or task that asked for it and the call graph stays acyclic.          */
bool         agc_exec_restart_pending(void);
agc_status_t agc_exec_service_restart(void);           /* runs ENEMA if requested */
void         agc_exec_enema(void);                     /* request ENEMA (GOPROG2)  */
void         agc_exec_varalarm(agc_alarm_code_t code); /* VARALARM      */
void         agc_phase_kill(int group);                /* DXCH -PHASEn with NEG0 */

/* Jobs and tasks defined in src/exec/ (called from dispatch.c).         */
agc_status_t agc_job_gotopooh(agc_job_ctx_t *ctx);            /* GOTOPOOH          */
agc_status_t agc_job_go_idle_fix(agc_job_ctx_t *ctx);         /* GOP00FIX          */
agc_status_t agc_job_gopoodoo(agc_job_ctx_t *ctx);            /* GOPOODOO          */
agc_status_t agc_job_go_idle_fix_reflash(agc_job_ctx_t *ctx); /* GOP00FIX -3 */
agc_status_t agc_job_change_program_continue(void);           /* CANV37   */
agc_status_t agc_job_change_program_start(void);              /* V37XEQ   */
agc_status_t agc_job_change_program_jets_off(void);           /* R00TOP00 +DELAYJOB (CSM) */
agc_status_t agc_job_change_program_return(void);             /* V37RET (LM) */
agc_status_t agc_job_endofjob(agc_job_ctx_t *ctx);            /* ENDOFJOB as a job */
agc_status_t agc_task_endtask(void);                          /* ENDTASK / SVCT3   */
agc_status_t agc_task_svct3(void);                            /* SVCT3 (retry)     */
agc_status_t agc_task_longcycl(void);                         /* LONGCYCL          */
agc_status_t agc_task_getcadr(void);                          /* GETCADR           */
void         agc_task_t5idloc(void);                          /* T5IDLOC (CSM)     */
agc_status_t agc_task_taskover(void);                         /* TASKOVER as a task*/

/* MODECADR = IMUCADR, OPTCADR, RADCADR (MODE SWITCHING ERASABLE): the
 * waiting-job words cleared by STARTSB2 and tested by SVCT3. Raw SP word
 * (+0 none, positive = a job is waiting; IMU lane defines the rest).    */
#define AGC_MODECADR_N 3          /* IMUCADR, OPTCADR, RADCADR */
agc_sp_t agc_erase_modecadr(int dev);
void     agc_erase_set_modecadr(int dev, agc_sp_t v);
agc_sp_t agc_erase_imucadr(void); /* = MODECADR +0         */

/* TIME4, TIME5, TIME6 (exec-private counters, agc_state.h): the sim
 * harness advances them and calls the T4/T5/T6 lead-ins on overflow.
 * n = 4, 5 or 6; values are the 15-bit counter words.                 */
uint16_t agc_rupt_time(int n);
void     agc_rupt_set_time(int n, uint16_t v);

#endif /* AGC_EXEC_H */
