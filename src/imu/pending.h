/*
 * pending.h -- interfaces the imu lane uses that other lanes own and had
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

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"

/* ==== sys-misc: RTB op codes (src/io/service/rtb_ops.c, header not yet public) == */
agc_dp_t     agc_rtb_cdulogic(uint16_t cdu_pattern); /* CDULOGIC  */
uint16_t     agc_rtb_1sto2s(agc_dp_t angle_rev);     /* 1STO2S    */
agc_status_t agc_rtb_v1sto2s(agc_vec_t angles_rev,
                             uint16_t  out[3]); /* V1STO2S */
agc_dp_t agc_rtb_signmpac(agc_dp_t mpac);  /* SIGNMPAC  */
agc_dp_t agc_rtb_loadtime(void);           /* LOADTIME  */

/* ==== sys-misc: T4RUPT_PROGRAM.agc ======================================= */
agc_status_t agc_t4_setissw(void); /* SETISSW (IMUFAIL/ICDUFAIL)  */

/* ==== sys-misc / prog-rendz: OPTIND (CM optics mode index) ============== */
void agc_erase_set_optind(agc_sp_t v); /* OPTIND: -1 none, 0, 1       */

/* ==== guid-powered: PIPASR +3 (read PIPAs into DELV without scaling) ===== */
void agc_pfs_pipasr_noscale(void);

/* ==== prog-rendz: GOTOV56 (terminate P20 via V56) ======================== */
agc_status_t agc_rendz_goto_terminate_tracking(void);

/* ==== dap-lm: interfaces not in agc_lm_dap.h ============================ */
agc_sp_t agc_erase_1jaccq(void); /* 1JACCQ: 1-jet Q accel, at pi/4     */
agc_sp_t agc_erase_1jaccr(void); /* 1JACCR: 1-jet R accel, at pi/4     */

/* ==== provisional job / task identifiers (0x0B00 range) ================== */
#define AGC_JOB_GYRO_COMMAND_PULSE \
    ((agc_job_id_t) 0x0B01)                      /* 1/GYRO             */
#define AGC_JOB_KALCMAN3 ((agc_job_id_t) 0x0B02) /* KALCMAN3           */
#define AGC_JOB_NEWDELHI ((agc_job_id_t) 0x0B03) /* NEWDELHI           */
#define AGC_JOB_GCOMP1   ((agc_job_id_t) 0x0B04) /* GCOMP1 (after IMUSTALL) */
#define AGC_JOB_REDOMANC ((agc_job_id_t) 0x0B05) /* R60 V50N18 PROCEED  */
#define AGC_JOB_ATTITUDE_MANEUVER_ENTER \
    ((agc_job_id_t) 0x0B06)                      /* R60 V50N18 ENTER    */
#define AGC_JOB_ENDMANUV ((agc_job_id_t) 0x0B07) /* R60 after GOMANUR   */
#define AGC_JOB_ATTITUDE_MANEUVER_TERMINATE \
    ((agc_job_id_t) 0x0B09) /* R60 V34 (R61TEST)   */
#define AGC_JOB_CREW_DEFINED_MANEUVER_TERMINATE \
    ((agc_job_id_t) 0x0B0A) /* R62 V34 / end -> ENDEXT */
#define AGC_JOB_CREW_DEFINED_MANEUVER_GOMOVE \
    ((agc_job_id_t) 0x0B0B) /* R62 PROCEED (GOMOVE) */
#define AGC_JOB_GYRO_COMMAND_RETRY \
    ((agc_job_id_t) 0x0B0E) /* 1/GYRO after GYROBUSY */
/* Same names and values as the other lanes' shims (exec_pending.h,
 * sysmisc_ext.h), so the lead can merge them.                             */
#define AGC_JOB_RELINUS ((agc_job_id_t) 0x0E11) /* CSM 1.7, LM 2.13    */
#define AGC_JOB_CREW_DEFINED_MANEUVER_DISPLAY \
    ((agc_job_id_t) 0x331) /* R62DISP (V49)       */

#define AGC_TASK_IMUZERO2 ((agc_task_id_t) 0x0B41)
#define AGC_TASK_IMUZERO3 ((agc_task_id_t) 0x0B42)
#define AGC_TASK_COARS    ((agc_task_id_t) 0x0B43)
#define AGC_TASK_COARS2   ((agc_task_id_t) 0x0B44)
#define AGC_TASK_CHKCORS  ((agc_task_id_t) 0x0B45)
#define AGC_TASK_IMUFINED ((agc_task_id_t) 0x0B46)
#define AGC_TASK_IFAILOK  ((agc_task_id_t) 0x0B47)
#define AGC_TASK_PFAILOK  ((agc_task_id_t) 0x0B48)
#define AGC_TASK_STRTGYRO ((agc_task_id_t) 0x0B49)
#define AGC_TASK_8192AUG  ((agc_task_id_t) 0x0B4A)
#define AGC_TASK_UPDTCALL ((agc_task_id_t) 0x0B4B)
#define AGC_TASK_MANUSTOP ((agc_task_id_t) 0x0B4C)
#define AGC_TASK_ENDMANU  ((agc_task_id_t) 0x0B4D) /* ENDMANU / GOODMANU */

#endif /* IMU_PENDING_H */
