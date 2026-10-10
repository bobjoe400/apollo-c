/*
 * sysmisc_ext.h -- TEMPORARY shim: symbols the sys-misc lane calls that other
 * lanes or the core do not export yet, and provisional job/task ids.  Every
 * item here is listed under "Requested core additions" / "Requested
 * jobs-tasks" in docs/modules/sys-misc.md.  At integration: move each
 * prototype into its owner's header, add the job/task ids to agc_jobs.h and
 * the dispatch switch, and delete the line here.  The AGC_JOB_/AGC_TASK_
 * placeholders are object-like macros so that they compile today; where
 * another lane's shim already names the same entry the same value is used.
 */
#ifndef AGC_SYSMISC_EXT_H
#define AGC_SYSMISC_EXT_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/exec/exec.h"

/* ---- requested from prog-rendz (P20-P25.agc) ---------------------------- */
bool         agc_rr_limchk(uint16_t cdut, uint16_t cdus);  /* RRLIMCHK: true = in limits */
agc_status_t agc_rr_settrkf(void);                         /* SETTRKF: tracker lamp  */
agc_status_t agc_rendz_radar_read(void);   /* LM RADAREAD (radar RUPT reader)      */
agc_status_t agc_rendz_vhf_read(void);     /* CSM VHFREAD (VHF range RUPT reader)  */

/* ---- requested from the owners of the downlinked erasables ------------- */
/* Read the SP word at offset off of a downlist item that this lane does
 * not hold (see telemetry.c); NOT_FOUND until the owner provides it.      */

/* ---- job / task placeholders (lead adds them to agc_jobs.h) ------------ */
/* tasks owned by sys-misc (dispatched to agc_t4_task / agc_svc_delay_wake /
 * agc_extverb_task)                                                        */
#define AGC_TASK_ENDTNON   ((agc_task_id_t)0x301)  /* T4RUPT ENDTNON (90 s)  */
#define AGC_TASK_UNZ2      ((agc_task_id_t)0x302)  /* T4RUPT UNZ2            */
#define AGC_TASK_ISSUP     ((agc_task_id_t)0x303)  /* T4RUPT ISSUP (VARDELAY)*/
#define AGC_TASK_ENDZOPT   ((agc_task_id_t)0x304)  /* T4RUPT ENDZOPT (CM)    */
#define AGC_TASK_ENDZOPT2  ((agc_task_id_t)0x305)  /* ENDZOPT after VARDELAY */
#define AGC_TASK_WAKER     ((agc_task_id_t)0x306)  /* DELAYJOB WAKER         */
#define AGC_TASK_ATTCK     ((agc_task_id_t)0x307)  /* V43 ATTCK2/ATTCK3 loop */
#define AGC_TASK_ALINTIME  ((agc_task_id_t)0x308)  /* reserved               */
/* tasks owned by other lanes (values as in their shims)                    */
#define AGC_TASK_CA_ECE    ((agc_task_id_t)0x0EAF) /* imu: CA+ECE            */
#define AGC_TASK_PFAILOK   ((agc_task_id_t)0x0B48) /* imu: PFAILOK           */
#define AGC_TASK_RRTURNON  ((agc_task_id_t)0x312)  /* prog-rendz: RRTURNON   */
#define AGC_TASK_DORREPOS  ((agc_task_id_t)0x313)  /* prog-rendz: DORREPOS   */
/* jobs owned by sys-misc (dispatched to agc_extverb_job /
 * agc_uplink_update_job / agc_systems_test_job)                            */
#define AGC_JOB_XV_ENDEXT    ((agc_job_id_t)0x304) /* terminate: ENDEXT       */
#define AGC_JOB_XV_GOPIN     ((agc_job_id_t)0x308) /* stall return -> GOPIN   */
#define AGC_JOB_XV_RESUME    ((agc_job_id_t)0x309) /* proceed/enter return    */
#define AGC_JOB_UPLINK_UPDATE_RESUME ((agc_job_id_t)0x310) /* after a V21N01 */
#define AGC_JOB_UPLINK_UPDATE_TERM   ((agc_job_id_t)0x313) /* V34 -> UPOUT+1 */
#define AGC_JOB_TIMEDIDR     ((agc_job_id_t)0x0E16) /* restart 6.5 (exec shim) */
#define AGC_JOB_SYSTEMS_TEST_REDO ((agc_job_id_t)0x320) /* P07 REDO (LM, V92) */

#endif /* AGC_SYSMISC_EXT_H */
