/*
 * lm_dap_pending.h -- shared symbols the LM DAP needs that other lanes have
 * not published yet (see docs/agents/_common.md naming convention).
 *
 * The DAPBOOLS flags (FLGWRD13), NEEDLFLG, SNUFFER and APSFLAG now come from
 * agc_flags_lm.h; CDUX..Z, IMODES33, LEMMASS, CSMMASS and THETAD (= CPHI,
 * CTHETA, CPSI) from agc_state.h; MASS and ABDELV from agc_guid.h
 * (agc_guid_state()->lm); CDUXCMD..CDUZCMD from agc_imu.h.
 *
 * INTEGRATION: when the owners publish the accessors below, delete them
 * here; the names follow the convention, so no DAP source file changes.
 * The job and task ids move to agc_jobs.h (lead).
 */
#ifndef LM_DAP_PENDING_H
#define LM_DAP_PENDING_H

#include <stdint.h>
#include "agc/agc_word.h"
#include "agc/agc_state.h"
#include "agc/agc_jobs.h"
#include "agc/agc_flags_lm.h"

/* ---- job and task ids (agc_jobs.h, integrated by the lead) ------------- */
#define AGC_JOB_1_ACCJOB  ((agc_job_id_t)0x0D01)   /* 1/ACCJOB, PRIO27 NOVAC  */
#define AGC_JOB_1_ACCSET  ((agc_job_id_t)0x0D02)   /* 1/ACCSET, PRIO27 NOVAC  */
#define AGC_TASK_DAPIDLER ((agc_task_id_t)0x0D81)  /* T5ADR = DAPIDLER        */
#define AGC_TASK_PAXIS    ((agc_task_id_t)0x0D82)  /* T5ADR = PAXIS           */

/* ---- erasables owned by other lanes ------------------------------------ */
/* Gimbal-to-pilot-axis matrix, SP scaled at 1, written by T4RUPT (sys-misc,
 * GPMATRIX/DAPT4S in src/lm/t4rupt_lm.c). M13 = 1 identically.            */
agc_sp_t agc_erase_m11(void);
agc_sp_t agc_erase_m21(void);
agc_sp_t agc_erase_m31(void);
agc_sp_t agc_erase_m22(void);
agc_sp_t agc_erase_m32(void);
/* Rotational hand controller counters P-RHCCTR, Q-RHCCTR, R-RHCCTR
 * (counter cells 042-044, state-flags / sim harness), SP counts.          */
agc_sp_t agc_erase_p_rhcctr(void);
agc_sp_t agc_erase_q_rhcctr(void);
agc_sp_t agc_erase_r_rhcctr(void);
void     agc_erase_set_p_rhcctr(agc_sp_t v);
void     agc_erase_set_q_rhcctr(agc_sp_t v);
void     agc_erase_set_r_rhcctr(agc_sp_t v);

#endif /* LM_DAP_PENDING_H */
