/*
 * guid_pending_lm.h -- Luminary-only symbols guid-powered needs from other
 * lanes that do not exist yet (docs/modules/guid-powered.md, "Requested
 * core additions"). Delete each block when the owner's header provides it.
 */
#ifndef GUID_PENDING_LM_H
#define GUID_PENDING_LM_H

#include "../guidance/guid_pending.h"
#include "agc/agc_flags_lm.h"
#include "agc/agc_lm_dap.h"
#include "agc/agc_lm_landing.h"

/* ---- counters (state-flags / exec) ---------------------------------------- */
agc_sp_t agc_erase_time5(void);              /* TIME5 counter (T5RUPT)       */
void     agc_erase_set_time5(agc_sp_t v);
void     agc_erase_set_thrust(agc_sp_t pulses); /* THRUST counter 055 (landing_pending.h) */

/* ---- radar routines (prog-rendz, P20-P25.agc pp. ~2727-3070, 4487, 5115) -- */
agc_status_t agc_radar_lralt(void);          /* LRALT: one LR altitude sample   */
agc_status_t agc_radar_lrvel(agc_sp_t beam2);/* LRVEL: 5 velocity samples, A = 2*VSELECT */
agc_status_t agc_radar_lrpos2(void);         /* LRPOS2: drive LR antenna to POS2 */
agc_dp_t     agc_radar_samplsum(void);       /* SAMPLSUM: last reading (DP)     */
agc_status_t agc_radar_r12lites(void);       /* R12LITES: LR lamps              */
agc_status_t agc_radar_r29(void);            /* R29: RR designate (from SERVICER) */

/* ---- landing (guid-landing, P70-P71.agc p. ~829) -------------------------- */
agc_status_t agc_lm_r10r11a(void);           /* R10,R11A (LRHTASK continues there) */

/* ---- LAMBERT / INITVEL erasables RASTEER1 reads (nav-conic / prog-rendz).
 * RTMAG, MU/A, MUASTEER are already agc_rendz_t fields.                     */
agc_sp_t  agc_erase_geomsgn(void);   /* GEOMSGN SP +.5 short way, -.5 long way */
agc_vec_t agc_erase_un(void);        /* UN unit normal to the transfer plane 2^1 */

/* ---- LM continuation jobs (lead adds to agc_jobs.h) --------------------- */
#define AGC_JOB_P42STAGE   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 110)) /* P42STAGE (REP40ALM PROCEED) */
#define AGC_JOB_IGNITE     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 111)) /* *PROCEED / IGNITE */
#define AGC_JOB_STAR_ENTER ((agc_job_id_t)(AGC_JOB_GUID_BASE + 112)) /* *ENTER     */
#define AGC_JOB_COMFAIL1   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 113)) /* COMFAIL1 (V97 PROCEED) */
#define AGC_JOB_COMFAIL2   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 114)) /* COMFAIL2 (V97 ENTER) */
#define AGC_JOB_ASTNRETN   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 115)) /* ASTNRETN (V06N61 PROCEED) */
#define AGC_JOB_GOPOST     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 116)) /* GOPOST    */
#define AGC_JOB_BRNBB      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 117)) /* B*RNB*B* after P40AUTO */
#define AGC_TASK_REDO4_2   ((agc_task_id_t)(AGC_TASK_GUID_BASE + 63)) /* REDO4.2  */

#endif /* GUID_PENDING_LM_H */
