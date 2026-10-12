/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Declarations shared only between the files of src/exec/ (executive,
 *   waitlist, phase tables, restarts, alarms, fresh start, interrupt
 *   lead-ins, change_program/). Not part of the public contract.
 *
 * AGC SOURCE:
 *   as the files that define them (see each declaration)
 *
 * REQUIRES:
 *   agc/exec/exec.h
 *
 * GLOBAL VARIABLES:
 *   none
 **********************************************************************/
#ifndef AGC_EXEC_INTERNAL_H
#define AGC_EXEC_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/exec/exec.h"

/* ---- priorities used inside the exec lane (PRIOnn octal >> 9) --------- */
#define EXEC_PRIO14 ((agc_prio_t) 014) /* INITDSP display restart        */
#define EXEC_PRIO30 ((agc_prio_t) 030) /* DSPMMJOB, RESTREG              */
#define EXEC_PRIO35 ((agc_prio_t) 035) /* NBDONLY drift compensation     */

/* ---- executive (executive.c) ------------------------------------------ */
/* Bumped by every STARTSB2 (fresh start or software restart). A caller
 * that sees it change knows the job/task tables were re-initialised.     */
uint16_t exec_restart_generation(void);
void     exec_clear_tables(void);     /* STARTSB2: core sets, VACs  */
int      exec_running_job_prio(void); /* prio of core set 0, -1 none */

/* ---- waitlist (waitlist.c) -------------------------------------------- */
void waitlist_clear_lists(void); /* STARTSB2: LST1/LST2 only   */
void waitlist_reset_time3(void); /* STARTSB1: TIME3 = POSMAX   */

/* ---- phase tables (phase_table.c) ------------------------------------- */
typedef struct phase_var_store
{
    agc_restart_kind_t kind;       /* what PHSNAMEn names                    */
    agc_job_id_t       job;        /* PHSNAMEn when it names a job           */
    agc_task_id_t      task;       /* PHSNAMEn when it names a task/longcall */
    int16_t            prdt;       /* PHSPRDTn: +prio FINDVAC / -prio NOVAC, or delta time in cs for a task */
    bool               immediate;  /* PHSPRDTn = -0: immediate task       */
    uint32_t           long_dt_cs; /* DP delta time for a variable LONGCALL (C(-GENADR) at the time of the phase change) */
} phase_var_store_t;

bool     phase_group_agrees(int group);                          /* PHASEn vs -PHASEn       */
void     phase_var_get(int group, phase_var_store_t *out);
uint16_t phase_tbase(int group);                                 /* TIME1 at TBASEn set     */
uint32_t phase_longbase(void);                                   /* TIME2,TIME1 at LONGBASE */
void     phase_kill(int group);                                  /* -PHASEn,PHASEn = -0,+0  */
void     phase_set_fixed(int group, uint16_t phase);             /* both words  */
void     phase_debug_set_negative(int group, uint16_t neg_word); /* tests */

/* ---- restart tables (restart_tables.c) -------------------------------- */
typedef enum rst_dtsrc
{
    RST_DT_DIRECT = 0, /* PRDTTAB holds the delta time (cs)        */
    RST_DT_IMMEDIATE,  /* PRDTTAB = -0 (OCT 77777): restart now    */
    RST_DT_INDIRECT,   /* PRDTTAB = -GENADR of an erasable         */
    RST_DT_LONG        /* LONGCALL: GENADR of a DP delta time      */
} rst_dtsrc_t;

typedef enum rst_erasable
{
    /* erasables named by indirect table words  */
    RST_E_NONE = 0,
    RST_E_SAVET_30,      /* LM  SAVET-30 B(2)                        */
    RST_E_600SECS,       /* both: 600SECS 2DEC 60000 (fixed const)   */
    RST_E_ZOOMTIME,      /* LM  ZOOMTIME B(1)                        */
    RST_E_TGO_1,         /* both: TGO +1 (low word of TGO B(2))      */
    RST_E_PREREAD_DELAY, /* CSM P40TMP B(2)                          */
    RST_E_S61DT,         /* CSM S61DT B(1)                           */
    RST_E_CM_GYMDT       /* CSM CM/GYMDT 1P                          */
} rst_erasable_t;

typedef struct rst_entry
{ /* one 3-word restart table entry          */
    agc_restart_kind_t kind;  /* NONE for the DEC 0 placeholders         */
    int16_t            prdt;  /* job: +prio FINDVAC, -prio NOVAC; task DIRECT: delta time cs */
    rst_dtsrc_t        dtsrc; /* where a task's delta time comes from   */
    rst_erasable_t     src;   /* for INDIRECT / LONG                     */
    agc_job_id_t       job;   /* kind JOB: its 2CADR                     */
    agc_task_id_t      task;  /* kind TASK / LONGCALL: its 2CADR         */
} rst_entry_t;

/* Look up entry `part` (0 or 1; 1 only for even phases) of phase `phase`
 * (>= 2) of restart group `group` (1..6) in the vehicle's tables.      */
agc_status_t rst_table_lookup(int group, int phase, int part, rst_entry_t *out);

/* Resolve an erasable named by a table word. SP read = the single word
 * the AGC fetches with NDX A / CA 1; DP read = DCA of the pair (cs).   */
agc_status_t rst_read_sp(rst_erasable_t src, int16_t *out);
agc_status_t rst_read_dp(rst_erasable_t src, uint32_t *out);

/* ---- fresh start (fresh_start.c) -------------------------------------- */
agc_status_t fresh_enema(void);    /* ENEMA software restart   */
agc_status_t fresh_mmdsplay(void); /* MMDSPLAY/SETUPDSP        */

/* ---- executive (executive.c), second pass ---------------------------- */
bool         exec_in_job(void);          /* a job body is executing   */
agc_job_id_t exec_running_job_loc(void); /* LOC of core set 0 (LOCALARM) */

/* ---- software restart request (alarm.c) ------------------------------ */
void         exec_request_enema(void);        /* WHIMPER: ENEMA at next service */
agc_status_t exec_service_interp_alarm(void); /* ALARM 1301 / ABORT 1302 of SQRT etc */

/* ---- MODECADR (erasable.c) -------------------------------------- */
void exec_clear_modecadr(void); /* STARTSB2: IMU/OPT/RAD CADR = 0 */

/* ---- interrupt lead-ins (interrupts.c) ------------------------------ */
void rupt_reset_timers(void); /* STARTSUB/STARTSB1: TIME4,5 */

/* ---- VAC5STOR (alarm.c) ---------------------------------------------- */
void exec_vac5stor(void); /* CSM debug copy of tables  */

/* ---- MODREG = -0 ----------------------------------------------------- */
/* agc_sp_t has no -0; the lanes agree that a negative MODREG means -0
 * (no program; the PROG lights are blanked). See io-pinball DSPMMJB.  */
#define EXEC_MODREG_NEG0 ((agc_sp_t) -1) /* MODREG = -0: no program */

/* ---- up-calls into higher layers (change_program/idle.c) --------------
 * The AGC restart code calls routines that apollo-c places above exec
 * (DAP, IMU, SERVICER). These few calls are collected in
 * change_program/, the layering exception next to dispatch.c.         */
void         exec_up_stoprate(void);       /* CSM GOPROG3 / INITSUB: STOPRATE */
agc_status_t exec_up_servidle(void);       /* LM STRTIDLE: SERVIDLE          */
void         exec_up_setcoars(void);       /* CSM GOPROG: SETCOARS (NO ATT)  */
agc_status_t exec_up_startsb2(void);       /* STARTSB2: other modules' resets */
agc_status_t exec_up_dofstart(bool slap1); /* DOFSTART: other modules' resets */
agc_status_t exec_up_goprog_optics(void);  /* CSM GOPROG: OPTMODES       */

void exec_initsub_csm(void); /* INITSUB / RAKE (CSM)           */

/* ---- V37 (change_program/sequence.c, csm.c, lm.c) -------------------------
 * One selectable major mode: FCADRMM1 (job), PREMM1 (priority; the EBANK
 * field has no meaning in C) and DNLADMM1 (downlist code).            */
typedef struct exec_cp_mode
{
    uint8_t      mm;     /* major mode number                         */
    agc_prio_t   prio;   /* PREMM1 bits 15-11                         */
    int          dnlist; /* DNLADMM1: DNLSTCOD for the new program    */
    agc_job_id_t job;    /* FCADRMM1: the program's first job          */
} exec_cp_mode_t;

uint8_t               exec_cp_mmnumber(void); /* MMNUMBER                   */
const exec_cp_mode_t *exec_cp_modes_csm(int *n);
const exec_cp_mode_t *exec_cp_modes_lm(int *n);

/* CSM: R00TOP00 (engine on or TVC DAP): BUSY while the job waits.      */
agc_status_t exec_cp_engine_off_to_idle(void);
agc_status_t exec_cp_after_engine_off(void); /* R00TOP00 tail: ISITP00 */

/* LM: SETUP70 / SETUP71 (P70 / P71 in the V37 job).                   */
agc_status_t exec_cp_setup_abort(bool aps);

/* ISSERVON: SERVICER asked to stop (AVEGFLAG down; LM OUTROUTE).      */
void exec_cp_stop_servicer_csm(void);
void exec_cp_stop_servicer_lm(void);

/* CANV37 after INTSTALL: outbits and flags of each vehicle.           */
void exec_cp_cleanup_csm(void);
void exec_cp_cleanup_lm(void);

/* POOH / NOUVEAU flag work (FLAGWRD0, FLAGWRD1, CLRADMOD, ...).        */
void         exec_cp_pooh_flags_csm(void);
agc_status_t exec_cp_pooh_flags_lm(void);
void         exec_cp_nouveau_flags_csm(void);
agc_status_t exec_cp_nouveau_flags_lm(void);

/* RENDV00: groups 1/2 for rendezvous; true = STATQUO (keep P20, GOMOD). */
bool exec_cp_rendezvous_csm(uint8_t mm);
bool exec_cp_rendezvous_lm(uint8_t mm);

#endif /* AGC_EXEC_INTERNAL_H */
