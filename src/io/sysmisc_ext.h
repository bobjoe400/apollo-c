/*
 * sysmisc_ext.h -- TEMPORARY shim: symbols the sys-misc lane calls that other
 * lanes or the core do not export yet.  Every item here is listed under
 * "Requested core additions" / "Requested jobs-tasks" in
 * docs/modules/sys-misc.md.  At integration: move each prototype into its
 * owner's header, add the job/task ids to agc_jobs.h, and delete the
 * corresponding line here (the AGC_JOB_/AGC_TASK_ placeholders below are
 * object-like macros so that they compile today; they must be removed when
 * agc_jobs.h gains the real enum members).
 */
#ifndef AGC_SYSMISC_EXT_H
#define AGC_SYSMISC_EXT_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"

/* ---- requested from the imu lane (IMU_MODE_SWITCHING_ROUTINES.agc) ------ */
agc_status_t agc_imu_zeroicdu(void);     /* ZEROICDU: CDUX/Y/Z = 0          */
agc_status_t agc_imu_noattoff(void);     /* NOATTOFF: NO ATT lamp off       */
agc_status_t agc_imu_setcoars(void);     /* SETCOARS (GLOCKMON, > 85 deg)   */
agc_status_t agc_imu_imubad(void);       /* IMUBAD: failed turn-on to user  */
agc_status_t agc_imu_clear_cmds(void);   /* -0 into CDUXCMD..CDUZCMD,GYROCMD */
agc_sp_t     agc_erase_modecadr(int dev);/* MODECADR +0..+2 (IMU/OPT/RAD)   */

/* ---- requested erasable accessors (state-flags / owners) -------------- */
uint16_t     agc_erase_optmodes(void);   /* OPTMODES (CM), bit word         */
void         agc_erase_set_optmodes(uint16_t v);
agc_sp_t     agc_erase_optind(void);     /* OPTIND (CM): -1 none, 0, 1      */
void         agc_erase_set_optind(agc_sp_t v);
uint16_t     agc_erase_radmodes(void);   /* RADMODES (LM), bit word         */
void         agc_erase_set_radmodes(uint16_t v);
uint16_t     agc_erase_extvbact(void);   /* EXTVBACT (pinball/marks)        */
void         agc_erase_set_extvbact(uint16_t v);
uint16_t     agc_erase_dapbools(void);   /* DAPBOOLS (LM DAP)               */
void         agc_erase_set_dapbools(uint16_t v);
agc_status_t agc_erase_read_ecadr(uint16_t ecadr, uint16_t *word);
agc_status_t agc_erase_write_ecadr(uint16_t ecadr, uint16_t word);
/* LM DAP gimbal<->pilot matrix elements, SP scaled at 1 (dap-lm owner).   */
void         agc_erase_set_m11(agc_sp_t v);
void         agc_erase_set_m21(agc_sp_t v);
void         agc_erase_set_m22(agc_sp_t v);
void         agc_erase_set_m31(agc_sp_t v);
void         agc_erase_set_m32(agc_sp_t v);
void         agc_erase_set_cosmg(agc_sp_t v);

/* ---- requested from prog-rendz (Luminary P20-P25.agc) ----------------- */
bool         agc_rr_limchk(uint16_t cdut, uint16_t cdus);  /* RRLIMCHK: in limits */
agc_status_t agc_rr_settrkf(void);                         /* SETTRKF        */

/* ---- requested from dap-lm (RCS_FAILURE_MONITOR.agc) ------------------- */
agc_status_t agc_dap_rcsmonit(void);                       /* RCSMONIT/RCSMON */

/* ---- requested from io-pinball ---------------------------------------- */
agc_status_t agc_dsky_falton(void);      /* FALTON: OPR ERR light on        */

/* ---- job / task placeholders (lead adds them to agc_jobs.h) ------------ */
/* tasks owned by sys-misc (dispatched to agc_t4_task / agc_svc_delay_wake /
 * agc_extverb_task)                                                        */
#define AGC_TASK_ENDTNON   ((agc_task_id_t)0x301)  /* T4RUPT ENDTNON (90 s)  */
#define AGC_TASK_UNZ2      ((agc_task_id_t)0x302)  /* T4RUPT UNZ2            */
#define AGC_TASK_ISSUP     ((agc_task_id_t)0x303)  /* T4RUPT ISSUP (VARDELAY)*/
#define AGC_TASK_ENDZOPT   ((agc_task_id_t)0x304)  /* T4RUPT ENDZOPT (CM)    */
#define AGC_TASK_ENDZOPT2  ((agc_task_id_t)0x305)  /* ENDZOPT after VARDELAY */
#define AGC_TASK_WAKER     ((agc_task_id_t)0x306)  /* DELAYJOB WAKER         */
#define AGC_TASK_ATTCK1    ((agc_task_id_t)0x307)  /* V43 ATTCK1 (CM)        */
#define AGC_TASK_ATTCK2    ((agc_task_id_t)0x308)  /* V43 ATTCK2 (LM)        */
/* tasks owned by other lanes                                               */
#define AGC_TASK_CA_ECE    ((agc_task_id_t)0x310)  /* imu: CA+ECE            */
#define AGC_TASK_PFAILOK   ((agc_task_id_t)0x311)  /* imu: PFAILOK           */
#define AGC_TASK_RRTURNON  ((agc_task_id_t)0x312)  /* prog-rendz: RRTURNON   */
#define AGC_TASK_DORREPOS  ((agc_task_id_t)0x313)  /* prog-rendz: DORREPOS   */
#define AGC_TASK_R31CALL   ((agc_task_id_t)0x314)  /* prog-rendz: R31CALL LM */
/* jobs owned by sys-misc (dispatched to agc_extverb_job)                   */
#define AGC_JOB_XV_IMUCOARK2 ((agc_job_id_t)0x301) /* V41 after V25N22 load   */
#define AGC_JOB_XV_IMUFINEK2 ((agc_job_id_t)0x302) /* V42 after V25N93 load   */
#define AGC_JOB_XV_FINEK2    ((agc_job_id_t)0x303) /* V42 after IMUFINE stall */
#define AGC_JOB_XV_ENDEXT    ((agc_job_id_t)0x304) /* terminate: ENDEXT       */
#define AGC_JOB_XV_ALINTIME2 ((agc_job_id_t)0x305) /* V55 after V25N24 load   */
#define AGC_JOB_XV_ATTCK_LD  ((agc_job_id_t)0x306) /* V43 after V25N22 load   */
#define AGC_JOB_XV_SHOWSUM2  ((agc_job_id_t)0x307) /* V91 next bank (V33)     */
#define AGC_JOB_XV_GOPIN     ((agc_job_id_t)0x308) /* stall return -> GOPIN   */
#define AGC_JOB_P27_PART2    ((agc_job_id_t)0x310) /* UPPART2 (after V7x)     */
#define AGC_JOB_P27_DATA     ((agc_job_id_t)0x311) /* V21N01 data/enter       */
#define AGC_JOB_P27_PROCEED  ((agc_job_id_t)0x312) /* V21N0x V33              */
#define AGC_JOB_P27_TERM     ((agc_job_id_t)0x313) /* V21N0x V34 -> UPOUT+1   */
#define AGC_JOB_UPJOB        ((agc_job_id_t)0x314) /* UPJOB (FINDVAC)         */
#define AGC_JOB_REDO         ((agc_job_id_t)0x320) /* P07 REDO (LM, V92)      */
/* jobs owned by other lanes, started by extended verbs                     */
#define AGC_JOB_PROCKEY      ((agc_job_id_t)0x330) /* pinball PROCKEY         */
#define AGC_JOB_R62DISP      ((agc_job_id_t)0x331) /* imu R62DISP (V49)       */
#define AGC_JOB_SBANDANT     ((agc_job_id_t)0x332) /* prog-align S-band (V64) */
#define AGC_JOB_V89CALL      ((agc_job_id_t)0x333) /* R63 (V89)               */
#define AGC_JOB_R36          ((agc_job_id_t)0x334) /* R36 (V90)               */
#define AGC_JOB_V67CALL      ((agc_job_id_t)0x335) /* W-matrix display (V67)  */
#define AGC_JOB_R31CALL      ((agc_job_id_t)0x336) /* R31 (V83, CM)           */
#define AGC_JOB_R21CSM       ((agc_job_id_t)0x337) /* R21 (V57, CM)           */
#define AGC_JOB_R23CSM       ((agc_job_id_t)0x338) /* R23 (V54, CM)           */
#define AGC_JOB_V82CALL      ((agc_job_id_t)0x339) /* R30 (V82)               */
#define AGC_JOB_AGSINIT      ((agc_job_id_t)0x33A) /* R47 (V47, LM)           */
#define AGC_JOB_ATTACHIT     ((agc_job_id_t)0x33B) /* V66                     */
#define AGC_JOB_LMTOCM       ((agc_job_id_t)0x33C) /* V47 (CM)                */
#define AGC_JOB_COMPVER      ((agc_job_id_t)0x33D) /* V65 (CM prelaunch)      */
#define AGC_JOB_AZMTHCG1     ((agc_job_id_t)0x33E) /* V78 (CM prelaunch)      */
#define AGC_JOB_STRKTSTI     ((agc_job_id_t)0x33F) /* V68 (CM TVC)            */
#define AGC_JOB_RRLOSDSP     ((agc_job_id_t)0x340) /* V85 (LM)                */
#define AGC_JOB_DAPDISP1     ((agc_job_id_t)0x341) /* V48 R03 (dap lanes)     */

#endif /* AGC_SYSMISC_EXT_H */
