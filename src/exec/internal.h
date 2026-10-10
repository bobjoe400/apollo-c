/*
 * internal.h -- declarations shared only between the files of
 * src/exec/ (executive, waitlist, phase tables, restarts, alarms, fresh
 * start, interrupt lead-ins). Not part of the public contract.
 */
#ifndef AGC_EXEC_INTERNAL_H
#define AGC_EXEC_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_exec.h"

/* ---- priorities used inside the exec lane (PRIOnn octal >> 9) --------- */
#define EXEC_PRIO14 ((agc_prio_t)014)   /* INITDSP display restart        */
#define EXEC_PRIO30 ((agc_prio_t)030)   /* DSPMMJOB, RESTREG              */
#define EXEC_PRIO35 ((agc_prio_t)035)   /* NBDONLY drift compensation     */

/* ---- executive (executive.c) ------------------------------------------ */
/* Bumped by every STARTSB2 (fresh start or software restart). A caller
 * that sees it change knows the job/task tables were re-initialised.     */
uint16_t exec_restart_generation(void);
void     exec_clear_tables(void);          /* STARTSB2: core sets, VACs  */
int      exec_running_job_prio(void);      /* prio of core set 0, -1 none */

/* ---- waitlist (waitlist.c) -------------------------------------------- */
void     waitlist_clear_lists(void);       /* STARTSB2: LST1/LST2 only   */
void     waitlist_reset_time3(void);       /* STARTSB1: TIME3 = POSMAX   */

/* ---- phase tables (phase_table.c) ------------------------------------- */
typedef struct phase_var_store {
    agc_restart_kind_t kind;   /* what PHSNAMEn names                    */
    agc_job_id_t       job;    /* PHSNAMEn when it names a job           */
    agc_task_id_t      task;   /* PHSNAMEn when it names a task/longcall */
    int16_t            prdt;   /* PHSPRDTn: +prio FINDVAC / -prio NOVAC,
                                  or delta time in cs for a task          */
    bool               immediate; /* PHSPRDTn = -0: immediate task       */
    uint32_t           long_dt_cs; /* DP delta time for a variable
                                      LONGCALL (C(-GENADR) at the time of
                                      the phase change)                   */
} phase_var_store_t;

bool         phase_group_agrees(int group);  /* PHASEn vs -PHASEn       */
uint16_t     phase_word(int group);          /* PHASEn (with type bits) */
agc_status_t phase_var_get(int group, phase_var_store_t *out);
uint16_t     phase_tbase(int group);         /* TIME1 at TBASEn set     */
uint32_t     phase_longbase(void);           /* TIME2,TIME1 at LONGBASE */
void         phase_kill(int group);          /* -PHASEn,PHASEn = -0,+0  */
void         phase_set_fixed(int group, uint16_t phase); /* both words  */
void         phase_debug_set_negative(int group, uint16_t neg_word); /* tests */

/* ---- restart tables (restart_tables.c) -------------------------------- */
typedef enum rst_dtsrc {
    RST_DT_DIRECT = 0,       /* PRDTTAB holds the delta time (cs)        */
    RST_DT_IMMEDIATE,        /* PRDTTAB = -0 (OCT 77777): restart now    */
    RST_DT_INDIRECT,         /* PRDTTAB = -GENADR of an erasable         */
    RST_DT_LONG              /* LONGCALL: GENADR of a DP delta time      */
} rst_dtsrc_t;

typedef enum rst_erasable {  /* erasables named by indirect table words  */
    RST_E_NONE = 0,
    RST_E_SAVET_30,          /* LM  SAVET-30 B(2)                        */
    RST_E_600SECS,           /* both: 600SECS 2DEC 60000 (fixed const)   */
    RST_E_ZOOMTIME,          /* LM  ZOOMTIME B(1)                        */
    RST_E_TGO_1,             /* both: TGO +1 (low word of TGO B(2))      */
    RST_E_PREREAD_DELAY,            /* CSM P40TMP B(2)                          */
    RST_E_S61DT,             /* CSM S61DT B(1)                           */
    RST_E_CM_GYMDT           /* CSM CM/GYMDT 1P                          */
} rst_erasable_t;

typedef struct rst_entry {    /* one 3-word restart table entry          */
    agc_restart_kind_t kind;  /* NONE for the DEC 0 placeholders         */
    int16_t        prdt;      /* job: +prio FINDVAC, -prio NOVAC;
                                 task DIRECT: delta time cs             */
    rst_dtsrc_t    dtsrc;
    rst_erasable_t src;       /* for INDIRECT / LONG                     */
    agc_job_id_t   job;
    agc_task_id_t  task;
} rst_entry_t;

/* Look up entry `part` (0 or 1; 1 only for even phases) of phase `phase`
 * (>= 2) of restart group `group` (1..6) in the vehicle's tables.      */
agc_status_t rst_table_lookup(int group, int phase, int part, rst_entry_t *out);
/* Resolve an erasable named by a table word. SP read = the single word
 * the AGC fetches with NDX A / CA 1; DP read = DCA of the pair (cs).   */
agc_status_t rst_read_sp(rst_erasable_t src, int16_t *out);
agc_status_t rst_read_dp(rst_erasable_t src, uint32_t *out);

/* ---- fresh start (fresh_start.c) -------------------------------------- */
agc_status_t fresh_enema(void);              /* ENEMA software restart   */
agc_status_t fresh_mmdsplay(void);           /* MMDSPLAY/SETUPDSP        */

/* ---- executive (executive.c), second pass ---------------------------- */
bool     exec_in_job(void);                /* a job body is executing   */
agc_job_id_t exec_running_job_loc(void);   /* LOC of core set 0 (LOCALARM) */

/* ---- software restart request (alarm.c) ------------------------------ */
void     exec_request_enema(void);         /* WHIMPER: ENEMA at next service */

/* ---- MODECADR (erasable.c) -------------------------------------- */
void     exec_clear_modecadr(void);        /* STARTSB2: IMU/OPT/RAD CADR = 0 */

/* ---- interrupt lead-ins (interrupts.c) ------------------------------ */
void     rupt_reset_timers(void);          /* STARTSUB/STARTSB1: TIME4,5 */

/* ---- VAC5STOR (alarm.c) ---------------------------------------------- */
void     exec_vac5stor(void);              /* CSM debug copy of tables  */

/* ---- MODREG = -0 ----------------------------------------------------- */
/* agc_sp_t has no -0; the lanes agree that a negative MODREG means -0
 * (no program; the PROG lights are blanked). See io-pinball DSPMMJB.  */
#define EXEC_MODREG_NEG0 ((agc_sp_t)-1)

/* ---- up-calls into higher layers (change_program_idle.c) -------------------------
 * The AGC restart code calls routines that apollo-c places above exec
 * (DAP, IMU, SERVICER). These few calls are collected in change_program_idle.c, the
 * layering exception next to dispatch.c; unit tests replace them.     */
agc_status_t exec_up_stoprate(void);       /* CSM GOPROG3 / INITSUB: STOPRATE */
agc_status_t exec_up_servidle(void);       /* LM STRTIDLE: SERVIDLE          */
agc_status_t exec_up_setcoars(void);       /* CSM GOPROG: SETCOARS (NO ATT)  */

#endif /* AGC_EXEC_INTERNAL_H */
