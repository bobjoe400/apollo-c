/*
 * entry_pending.h -- PENDING core symbols used by the guid-entry lane.
 *
 * Every declaration here is a request to another lane (see
 * docs/modules/guid-entry.md, "Requested core additions" and "Requested
 * jobs-tasks"). Nothing here is DEFINED by this lane. At integration this
 * file is deleted: the job/task ids move to agc_jobs.h, the services to the
 * owners' public headers.
 *
 * Already available and therefore NOT repeated here: all flags
 * (agc_flags_csm.h), channel bits (agc_channels.h), DSKY (agc_dsky.h),
 * lat/long (agc_navaux.h), CALCGA/CALCGTA/ARCTRIG (agc_imu.h), NEEDLER
 * (agc_csm_dap.h), UNITR/UNITW/DELVREF/AVGEXIT (agc_guid_state()->avg),
 * TLIFTOFF (= TEVENT, agc_erase_tevent()).
 */
#ifndef AGC_ENTRY_PENDING_H
#define AGC_ENTRY_PENDING_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_state.h"
#include "agc/agc_jobs.h"
#include "agc/agc_exec.h"
/* Provisional ids the exec lane already names for this lane's restart
 * points: AGC_JOB_P67_1 S61_1A PRE_HUNT P65_1 MATRXJOB REP11 REP11A
 * NORMLIZE, AGC_TASK_SETJTAG WAKEP62 S61_1C ATERTASK READGYMB REDOSAT.   */
#include "../exec/exec_pending.h"
/* sys-misc services (LOADTIME, DELAYJOB, UPFLAG, telemetry list select). */
#include "../io/sysmisc.h"

/* ---- jobs and tasks not yet in agc_jobs.h (0x0C00 provisional range) ---- */
#define AGC_JOB_SERVEXIT   ((agc_job_id_t)0x0C00) /* SERVEXIT as AVGEXIT target */
#define AGC_JOB_CM_POSE    ((agc_job_id_t)0x0C01) /* CM/POSE (AVGEXIT in entry) */
#define AGC_JOB_REDOPOSE   ((agc_job_id_t)0x0C02) /* REDOPOSE group 5 restart   */
#define AGC_JOB_P61        ((agc_job_id_t)0x0C03) /* P61 (V37)                  */
#define AGC_JOB_P61_DSP    ((agc_job_id_t)0x0C04) /* P61 V06N61 (S61.1 return)  */
#define AGC_JOB_P61_4      ((agc_job_id_t)0x0C05) /* P61.4                      */
#define AGC_JOB_P61_1      ((agc_job_id_t)0x0C06) /* P61.1  V06N60              */
#define AGC_JOB_P61_2      ((agc_job_id_t)0x0C07) /* P61.2  V06N63              */
#define AGC_JOB_P61_3      ((agc_job_id_t)0x0C08) /* P61.3  PHASCHNG, ENDOFJOB  */
#define AGC_JOB_P62_NEWMODE ((agc_job_id_t)0x0C09) /* P62 -3: NEWMODEX 62       */
#define AGC_JOB_P62        ((agc_job_id_t)0x0C0A) /* P62 (V37)                  */
#define AGC_JOB_P62_S61RET ((agc_job_id_t)0x0C0B) /* P62 after S61.1            */
#define AGC_JOB_P62_2      ((agc_job_id_t)0x0C0C) /* P62.2  GOPERF1R 41         */
#define AGC_JOB_P62_2B     ((agc_job_id_t)0x0C0D) /* P62.2 PROCEED: CM/DAPON    */
#define AGC_JOB_CM_DAPON_WAIT ((agc_job_id_t)0x0C0E) /* NOTYET after DELAYJOB   */
#define AGC_JOB_P62_1      ((agc_job_id_t)0x0C0F) /* P62.1  V06N61              */
#define AGC_JOB_P62_1B     ((agc_job_id_t)0x0C10) /* P62.1 PROCEED              */
#define AGC_JOB_P63        ((agc_job_id_t)0x0C11) /* P63 (NOVAC from WAKEP62)   */
#define AGC_JOB_P65_1B     ((agc_job_id_t)0x0C12) /* P65.1 PROCEED              */
#define AGC_JOB_P67_1B     ((agc_job_id_t)0x0C13) /* P67.1 PROCEED              */
#define AGC_JOB_S61_1_RET  ((agc_job_id_t)0x0C14) /* RETRN1 after DELAYJOB      */
#define AGC_JOB_S61_1_MID  ((agc_job_id_t)0x0C15) /* S61.1 after MIDTOAV2 BUSY  */
#define AGC_JOB_P37        ((agc_job_id_t)0x0C16) /* P37 (V37)                  */
#define AGC_JOB_P11        ((agc_job_id_t)0x0C17) /* P11                        */
#define AGC_JOB_VHHDOT     ((agc_job_id_t)0x0C18) /* VHHDOT (AVGEXIT in P11)    */
#define AGC_JOB_ATERJOB    ((agc_job_id_t)0x0C19) /* ATERJOB                    */
#define AGC_TASK_JETCALL   ((agc_task_id_t)0x0C80) /* JETCALL                   */
#define AGC_TASK_ATTRATES  ((agc_task_id_t)0x0C81) /* ATTRATES (T5RUPT)         */
#define AGC_TASK_READACCS  ((agc_task_id_t)0x0C82) /* READACCS (guid-powered)   */
#define AGC_TASK_SATSTICK  ((agc_task_id_t)0x0C83) /* SATSTICK (T5RUPT)         */

/* Priorities: PRIOnn octal (agc_prio_t = PRIOnn >> 9, see agc_exec.h).     */
#define AGC_ENTRY_PRIO1  ((agc_prio_t)001)
#define AGC_ENTRY_PRIO13 ((agc_prio_t)013)
#define AGC_ENTRY_PRIO16 ((agc_prio_t)016)
#define AGC_ENTRY_PRIO20 ((agc_prio_t)020)

/* ---- telemetry (sys-misc) -------------------------------------------- */
#define AGC_ENTRY_DNLSTCOD_ENTRY   1   /* DNLSTCOD 1: ENTRY/UPDATE list     */
#define AGC_ENTRY_DNLSTCOD_POWERED 3   /* POWDNCOD = THREE                  */

#endif /* AGC_ENTRY_PENDING_H */
