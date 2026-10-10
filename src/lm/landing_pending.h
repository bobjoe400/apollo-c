/*
 * landing_pending.h -- guid-landing lane: the shared symbols this lane uses
 * that NO header in include/agc provides yet (checked 2026-10-09 against
 * agc_state.h, agc_exec.h, agc_flags_lm.h, agc_channels.h, agc_dsky.h,
 * agc_guid.h, agc_imu.h, agc_navaux.h, agc_align.h).
 *
 * Names follow docs/agents/_common.md; where another lane's pending header
 * already declares the same symbol (src/imu/imu_pending.h) the identical
 * prototype is repeated here. INTEGRATION: delete each block once the
 * owning lane's header provides it. Nothing here is defined in src/lm/.
 */
#ifndef LANDING_PENDING_H
#define LANDING_PENDING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_state.h"
#include "agc/agc_exec.h"
#include "agc/agc_channels.h"
#include "agc/agc_flags_lm.h"
#include "agc/agc_dsky.h"
#include "agc/agc_guid.h"
#include "agc/agc_imu.h"
#include "agc/agc_navaux.h"
#include "agc/agc_integ.h"
#include "agc/agc_lm_dap.h"
#include "agc/agc_lm_landing.h"

/* ---- jobs / tasks (lead registers them in agc_jobs.h + dispatch.c) ------
 * GOABORT, P70A, P71A, R10,R11, ZOOM come from src/exec/exec_pending.h
 * (the restart tables name them). The rest are provisional ids in this
 * lane's private range 0x0F00 (jobs) / 0x0F80 (tasks), clear of exec's.  */
#include "../exec/exec_pending.h"
#define AGC_JOB_P63LM      ((agc_job_id_t)0x0F01) /* P63LM                    */
#define AGC_JOB_ASTNCLOK   ((agc_job_id_t)0x0F02) /* IGNALGRT/ASTNCLOK tail    */
#define AGC_JOB_ASTNRET    ((agc_job_id_t)0x0F03) /* ASTNRET (STCLOK2 return)  */
#define AGC_JOB_P63SPOT2   ((agc_job_id_t)0x0F04) /* P63SPOT2 (R51 ENTER)      */
#define AGC_JOB_P63SPOT3   ((agc_job_id_t)0x0F05) /* P63SPOT3 (after R60)      */
#define AGC_JOB_P63SPOT4   ((agc_job_id_t)0x0F06) /* P63SPOT4 (CODE 500 ENTER) */
#define AGC_JOB_P64CEED    ((agc_job_id_t)0x0F07) /* P64CEED (V06N64 PROCEED)  */
#define AGC_JOB_P64DISPS   ((agc_job_id_t)0x0F08) /* P64DISPS (V06N64 ENTER)   */
#define AGC_JOB_RODCOMP    ((agc_job_id_t)0x0F09) /* RODCOMP (FINDVAC PRIO22)  */
#define AGC_JOB_LANDJUNK   ((agc_job_id_t)0x0F0A) /* LANDJUNK (P68)            */
#define AGC_JOB_LANDJNK2   ((agc_job_id_t)0x0F0B) /* LANDJUNK V06N43 PROCEED   */
#define AGC_JOB_P12LM      ((agc_job_id_t)0x0F0D) /* P12LM                     */
#define AGC_JOB_P12LM2     ((agc_job_id_t)0x0F0E) /* P12LM after V06N33A       */
#define AGC_JOB_NEWLOAD    ((agc_job_id_t)0x0F0F) /* NEWLOAD (V06N76 flash)    */
#define AGC_JOB_NEWLOAD2   ((agc_job_id_t)0x0F10) /* NEWLOAD PROCEED           */
#define AGC_JOB_CUTOFF     ((agc_job_id_t)0x0F11) /* CUTOFF (FINDVAC PRIO17)   */
#define AGC_JOB_CUTOFF1    ((agc_job_id_t)0x0F12) /* CUTOFF1 (V16N63 PROCEED)  */
#define AGC_JOB_CUTOFF2    ((agc_job_id_t)0x0F13) /* CUTOFF2                   */
#define AGC_JOB_TERMASC    ((agc_job_id_t)0x0F14) /* TERMASC                   */
#define AGC_JOB_LUNLAND    ((agc_job_id_t)0x0F15) /* LUNLAND (AVGEXIT)         */
#define AGC_JOB_ATMAG      ((agc_job_id_t)0x0F16) /* ATMAG (AVGEXIT)           */
#define AGC_JOB_SERVEXIT   ((agc_job_id_t)0x0F17) /* SERVEXIT (AVGEXIT)        */
#define AGC_JOB_P70        ((agc_job_id_t)0x0F18) /* P70 (V37)                 */
#define AGC_JOB_P71        ((agc_job_id_t)0x0F19) /* P71 (V37)                 */
#define AGC_JOB_ENDLLJOB   ((agc_job_id_t)0x0F1A) /* display "terminate" = end */
#define AGC_TASK_RODTASK   ((agc_task_id_t)0x0F81) /* RODTASK                  */
#define AGC_TASK_REDESMON  ((agc_task_id_t)0x0F82) /* REDESMON (PITFALL)       */
#define AGC_TASK_ENGOFF1   ((agc_task_id_t)0x0F83) /* ENGOFF1                  */
#define AGC_TASK_INTLZE    ((agc_task_id_t)0x0F85) /* INTLZE (R10)             */
#define AGC_TASK_SPEEDRN2  ((agc_task_id_t)0x0F86) /* SPEEDRUN after VARDELAY  */

/* ---- alarm codes raised by this lane (octal) ---------------------------- */
#define AGC_ALARM_00401 ((agc_alarm_code_t)00401) /* FINDCDUW: gimbal lock     */
#define AGC_ALARM_00402 ((agc_alarm_code_t)00402) /* FINDCDUW: NORMUNIT ovf    */
#define AGC_ALARM_01406 ((agc_alarm_code_t)01406) /* TTF/8 no convergence      */
#define AGC_ALARM_01410 ((agc_alarm_code_t)01410) /* overflow in guidance      */
#define AGC_ALARM_01412 ((agc_alarm_code_t)01412) /* IGNALG NIGNLOOP exhausted */

/* ---- erasables / counters with no accessor yet ---------------------------
 * Owners: THRUST/ALTM counters and CDUTCMD/CDUSCMD (state-flags: hardware
 * counter cells 053-060), TEMX/Y/Z (guid-powered, SERVICER temporaries
 * reused by R10).                                                        */
void     agc_erase_set_thrust(agc_sp_t pulses);     /* THRUST counter 055   */
void     agc_erase_set_altm(agc_sp_t v);            /* ALTM counter 060     */
void     agc_erase_set_cdutcmd(int which, agc_sp_t v); /* 0 CDUTCMD 053, 1 CDUSCMD 054 */

/* ---- dap-lm: the other two 1-jet accelerations (agc_lm_dap.h has P) ---- */
agc_sp_t agc_erase_1jaccq(void);                    /* 1JACCQ pi/4 r/s^2    */
agc_sp_t agc_erase_1jaccr(void);                    /* 1JACCR pi/4 r/s^2    */

/* ---- prog-rendz: R29/R12 radar helpers (RADAR_LEADIN / R10 lamps) ------- */
agc_status_t agc_rendz_flip(uint16_t lamp_bit);     /* FLIP (R10 LR lamps)  */

#endif /* LANDING_PENDING_H */
