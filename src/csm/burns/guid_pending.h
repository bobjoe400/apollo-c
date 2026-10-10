/*
 * guid_pending.h -- Comanche-only symbols guid-powered needs from other
 * lanes that do not exist yet (docs/modules/guid-powered.md, "Requested core
 * additions"). Delete each block when the owner's header provides it.
 */
#ifndef GUID_PENDING_CSM_H
#define GUID_PENDING_CSM_H

#include "agc/core/flags_csm.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/guidance/pending.h"

/* ---- entry DAP angle registers (guid-entry) ------------------------------ */
agc_sp_t agc_erase_aog(void); /* AOG outer gimbal, /180 2's comp */
agc_sp_t agc_erase_aig(void); /* AIG inner gimbal               */
agc_sp_t agc_erase_amg(void); /* AMG middle gimbal              */
void     agc_erase_set_cm_gymdt(agc_sp_t v); /* CM/GYMDT entry DAP gyro dt    */

/* ---- marks and optics (prog-rendz / imu / sys-misc) ---------------------- */
void agc_erase_set_vhfcnt(agc_sp_t v); /* VHFCNT VHF range mark count   */
void agc_erase_set_mrkbuf2(int index, agc_sp_t v); /* MRKBUF2 +index     */
void agc_erase_set_optind(agc_sp_t v); /* OPTIND optics mode (T4RUPT)   */

/* ---- TVC output counters and T5 (dap-csm / exec) ------------------------- */
void     agc_erase_set_tvcpitch(agc_sp_t v); /* TVCPITCH counter, 85.41"/bit */
void     agc_erase_set_tvcyaw(agc_sp_t v);   /* TVCYAW counter               */
agc_sp_t agc_erase_time5(void);              /* TIME5 counter                */
void     agc_erase_set_time5(agc_sp_t v);

/* ---- other lanes' routines called from the CSM burn programs ------------- */
agc_status_t agc_csm_velocity_altitude_rate(
    agc_interp_t *ip); /* S11.1 V I, H, HDOT (P11, guid-entry) */
agc_status_t agc_csm_satstkon(
    const agc_job_ctx_t *ctx); /* SATSTKON (P11 / V46, guid-entry) */
agc_status_t agc_dsky_gopin(
    const agc_job_ctx_t *ctx); /* GOPIN (extended verb exit, pinball) */

/* ---- T5 entry of the TVC DAP (dap-csm; lead adds to agc_jobs.h) --------- */
#define AGC_TASK_TVCDAPON \
    ((agc_task_id_t) (AGC_TASK_GUID_BASE + 60)) /* TVCDAPON (dap-csm) */
/* ---- CSM P40 continuation jobs (lead adds to agc_jobs.h) ----------------- */
#define AGC_JOB_PREBURN_START_CLOCK \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 100)) /* after DELAYJOB of TRIM */
#define AGC_JOB_CROSS_PRODUCT_RESTART \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 101)) /* S40.8 after 5.3 B   */
#define AGC_JOB_UPDATEVG_RESTART \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 102)) /* UPDATEVG after 5.3 B */
#define AGC_JOB_SURELY9_RESTART \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 103)) /* SURELY.9 after 2PHSCHNG */
#define AGC_JOB_STEERING_RESTART \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 104)) /* STEERING after ENGINOFF set */
#define AGC_JOB_CALC_DELTA_V_RESTART \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 105)) /* CALCN83 after 5.3 B */
#define AGC_JOB_COPY40_9 \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 106)) /* COPY40.9 (1.4 C)    */
#define AGC_JOB_ENDLAMB \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 107)) /* ENDLAMB (1.5 C)     */
#define AGC_JOB_ENGINE_ENABLE_BYPASS_RESTART \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 108)) /* V99EJOB (4 C)       */
#define AGC_JOB_ENGINE_ENABLE_TERMINATE_RESTART \
    ((agc_job_id_t) (AGC_JOB_GUID_BASE + 109)) /* V99TJOB (4 C)       */
#define AGC_TASK_ENGINE_FAIL_SHUTDOWN_RESTART \
    ((agc_task_id_t) (AGC_TASK_GUID_BASE + 61)) /* V97TTASK after 5.1 C */
#define AGC_TASK_QUICKIGN \
    ((agc_task_id_t) (AGC_TASK_GUID_BASE + 62)) /* QUICKIGN (C task)  */

#endif /* GUID_PENDING_CSM_H */
