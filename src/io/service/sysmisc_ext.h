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

/* ---- job / task placeholders (lead adds them to agc_jobs.h) ------------ */
/* tasks owned by sys-misc (dispatched to agc_t4_task / agc_svc_delay_wake /
 * agc_extverb_task)                                                        */
#define AGC_TASK_ENDTNON   ((agc_task_id_t)0x301)  /* T4RUPT ENDTNON (90 s)  */
#define AGC_TASK_UNZ2      ((agc_task_id_t)0x302)  /* T4RUPT UNZ2            */
#define AGC_TASK_ISSUP     ((agc_task_id_t)0x303)  /* T4RUPT ISSUP (VARDELAY)*/
#define AGC_TASK_ENDZOPT   ((agc_task_id_t)0x304)  /* T4RUPT ENDZOPT (CM)    */
#define AGC_TASK_ENDZOPT2  ((agc_task_id_t)0x305)  /* ENDZOPT after VARDELAY */
#define AGC_TASK_WAKER     ((agc_task_id_t)0x306)  /* DELAYJOB WAKER         */
/* tasks owned by other lanes (values as in their shims)                    */
#define AGC_TASK_CA_ECE    ((agc_task_id_t)0x0EAF) /* imu: CA+ECE            */
#define AGC_TASK_PFAILOK   ((agc_task_id_t)0x0B48) /* imu: PFAILOK           */
#define AGC_TASK_RRTURNON  ((agc_task_id_t)0x312)  /* prog-rendz: RRTURNON   */
#define AGC_TASK_DORREPOS  ((agc_task_id_t)0x313)  /* prog-rendz: DORREPOS   */
/* (the extended-verb and P27 job ids are in src/io/verbs/pending.h)      */

#endif /* AGC_SYSMISC_EXT_H */
