/*
 * imu_pending.h -- interfaces the imu lane uses that other lanes own and had
 * not published in include/agc when this module was written.
 *
 * INTEGRATION: every declaration below is an ASSUMED interface. When the
 * owning header appears, delete the matching block here and include that
 * header instead (or adapt the call sites). Each block names its owner. The
 * same list is in docs/modules/imu.md under "Requested core additions".
 *
 * Job/task identifiers: object-like constants AGC_JOB_<LABEL> /
 * AGC_TASK_<LABEL> with provisional values in the 0x0B00 range; the lead
 * replaces them with real agc_jobs.h members and dispatch.c cases.
 */
#ifndef IMU_PENDING_H
#define IMU_PENDING_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"

/* ==== sys-misc: RTB op codes (src/io/rtb_ops.c, header not yet public) == */
agc_dp_t     agc_rtb_cdulogic(uint16_t cdu_pattern);          /* CDULOGIC  */
uint16_t     agc_rtb_1sto2s(agc_dp_t angle_rev);              /* 1STO2S    */
agc_status_t agc_rtb_v1sto2s(agc_vec_t angles_rev, uint16_t out[3]); /* V1STO2S */
agc_dp_t     agc_rtb_signmpac(agc_dp_t mpac);                 /* SIGNMPAC  */
agc_dp_t     agc_rtb_loadtime(void);                          /* LOADTIME  */

/* ==== sys-misc: T4RUPT_PROGRAM.agc ======================================= */
agc_status_t agc_t4_setissw(void);           /* SETISSW (IMUFAIL/ICDUFAIL)  */

/* ==== exec-sched ========================================================== */
agc_status_t agc_exec_varalarm(agc_alarm_code_t code); /* VARALARM          */
agc_status_t agc_exec_linus(agc_job_id_t restart_job); /* LINUS + PHASCHNG 1.7 */

/* ==== guid-powered: PIPASR +3 (read PIPAs into DELV without scaling) ===== */
agc_status_t agc_pfs_pipasr_noscale(void);

/* ==== prog-rendz: GOTOV56 (terminate P20 via V56) ======================== */
agc_status_t agc_rendz_gotov56(void);

/* ==== dap-csm: RCS DAP erasables written by KALCMANU (Comanche) ========== */
void     agc_erase_set_cduxd(int axis, agc_sp_t v); /* CDUXD/YD/ZD high word, half-rev */
agc_sp_t agc_erase_cduxd(int axis);
void     agc_erase_set_delcdux(int axis, agc_dp_t v); /* DELCDUX/Y/Z (CM DP)   */
void     agc_erase_set_wbody(int axis, agc_dp_t v);   /* WBODY, WBODY1, WBODY2 */
void     agc_erase_set_bias(int axis, agc_sp_t v);    /* BIAS, BIAS1, BIAS2    */
uint16_t agc_erase_rcsflags(void);                    /* RCSFLAGS              */
void     agc_erase_set_rcsflags(uint16_t v);
agc_sp_t agc_erase_holdflag(void);                    /* HOLDFLAG              */
void     agc_erase_set_holdflag(agc_sp_t v);

/* ==== dap-lm: LM DAP erasables written by KALCMANU (Luminary) ============ */
void     agc_erase_set_deldcdu(int axis, agc_sp_t v); /* DELDCDU, 1, 2 (SP)    */
void     agc_erase_set_omegapd(int axis, agc_sp_t v); /* OMEGAPD/QD/RD pi/4 r/s */
void     agc_erase_set_delperor(int axis, agc_sp_t v);/* DELPEROR/QEROR/REROR  */
agc_sp_t agc_erase_1jacc(int axis);                   /* 1JACC, 1JACCQ, 1JACCR */
agc_status_t agc_lm_dap_zatteror(void);               /* ZATTEROR              */

/* ==== provisional job / task identifiers (0x0B00 range) ================== */
#define AGC_JOB_1_GYRO          ((agc_job_id_t)0x0B01) /* 1/GYRO             */
#define AGC_JOB_KALCMAN3        ((agc_job_id_t)0x0B02) /* KALCMAN3           */
#define AGC_JOB_NEWDELHI        ((agc_job_id_t)0x0B03) /* NEWDELHI           */
#define AGC_JOB_GCOMP1          ((agc_job_id_t)0x0B04) /* GCOMP1 (after IMUSTALL) */
#define AGC_JOB_R60_REDOMANC    ((agc_job_id_t)0x0B05) /* R60 PROCEED        */
#define AGC_JOB_R60_TOMANV      ((agc_job_id_t)0x0B06) /* R60 V50N18 ENTER/V34 */
#define AGC_JOB_R60_ENDMANUV    ((agc_job_id_t)0x0B07) /* R60 after GOMANUR   */
#define AGC_JOB_R62RESP         ((agc_job_id_t)0x0B08) /* R62 V06N22 response */
#define AGC_JOB_R60_TERM        ((agc_job_id_t)0x0B09) /* R60 V34 (R61TEST)   */
#define AGC_JOB_R62_TERM        ((agc_job_id_t)0x0B0A) /* R62 V34 -> ENDEXT   */
#define AGC_JOB_R62_ENTER       ((agc_job_id_t)0x0B0B) /* R62 ENTER -> redisplay */
#define AGC_JOB_R60_RELINUS     ((agc_job_id_t)0x0B0C) /* RELINUS restart     */
#define AGC_JOB_R60_DONE        ((agc_job_id_t)0x0B0D) /* R62 return from R60 */
#define AGC_JOB_1_GYRO_PULSE    ((agc_job_id_t)0x0B0E) /* 1/GYRO after GYROBUSY */

#define AGC_TASK_IMUZERO2       ((agc_task_id_t)0x0B41)
#define AGC_TASK_IMUZERO3       ((agc_task_id_t)0x0B42)
#define AGC_TASK_COARS          ((agc_task_id_t)0x0B43)
#define AGC_TASK_COARS2         ((agc_task_id_t)0x0B44)
#define AGC_TASK_CHKCORS        ((agc_task_id_t)0x0B45)
#define AGC_TASK_IMUFINED       ((agc_task_id_t)0x0B46)
#define AGC_TASK_IFAILOK        ((agc_task_id_t)0x0B47)
#define AGC_TASK_PFAILOK        ((agc_task_id_t)0x0B48)
#define AGC_TASK_STRTGYRO       ((agc_task_id_t)0x0B49)
#define AGC_TASK_8192AUG        ((agc_task_id_t)0x0B4A)
#define AGC_TASK_UPDTCALL       ((agc_task_id_t)0x0B4B)
#define AGC_TASK_MANUSTOP       ((agc_task_id_t)0x0B4C)
#define AGC_TASK_ENDMANU        ((agc_task_id_t)0x0B4D) /* ENDMANU / GOODMANU */

#endif /* IMU_PENDING_H */
