/*
 * landing_pending.h -- guid-landing lane: the shared symbols this lane uses
 * that NO header in include/agc provides yet (re-checked 2026-10-10, all
 * wave-1 lanes landed, against every include/agc header).
 *
 * Names follow docs/agents/_common.md; where another lane's pending header
 * already declares the same symbol (src/imu/pending.h) the identical
 * prototype is repeated here. INTEGRATION: delete each block once the
 * owning lane's header provides it. Nothing here is defined in src/lm/.
 */
#ifndef LANDING_PENDING_H
#define LANDING_PENDING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/io/dsky.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/programs/rendezvous.h"

/* ---- jobs / tasks (lead registers them in agc_jobs.h + dispatch.c) ------
 * GOABORT, P70A, P71A, R10,R11, ZOOM come from include/agc/agc_exec_pending.h
 * (the restart tables name them). The rest are provisional ids in this
 * lane's private range 0x0F00 (jobs) / 0x0F80 (tasks), clear of exec's.  */
#include "pending.h"
#define AGC_JOB_BRAKING_PHASE      ((agc_job_id_t)0x0F01) /* P63LM                    */
#define AGC_JOB_ASTNCLOK   ((agc_job_id_t)0x0F02) /* IGNALGRT/ASTNCLOK tail    */
#define AGC_JOB_ASTNRET    ((agc_job_id_t)0x0F03) /* ASTNRET (STCLOK2 return)  */
#define AGC_JOB_BRAKING_PHASE_BURN_ATTITUDE   ((agc_job_id_t)0x0F04) /* P63SPOT2 (R51 ENTER)      */
#define AGC_JOB_BRAKING_PHASE_ANTENNA_CHECK   ((agc_job_id_t)0x0F05) /* P63SPOT3 (after R60)      */
#define AGC_JOB_BRAKING_PHASE_RADAR_INIT   ((agc_job_id_t)0x0F06) /* P63SPOT4 (CODE 500 ENTER) */
#define AGC_JOB_APPROACH_PHASE_PROCEED    ((agc_job_id_t)0x0F07) /* P64CEED (V06N64 PROCEED)  */
#define AGC_JOB_APPROACH_PHASE_DISPLAY   ((agc_job_id_t)0x0F08) /* P64DISPS (V06N64 ENTER)   */
#define AGC_JOB_RODCOMP    ((agc_job_id_t)0x0F09) /* RODCOMP (FINDVAC PRIO22)  */
#define AGC_JOB_LANDJUNK   ((agc_job_id_t)0x0F0A) /* LANDJUNK (P68)            */
#define AGC_JOB_LANDJNK2   ((agc_job_id_t)0x0F0B) /* LANDJUNK V06N43 PROCEED   */
#define AGC_JOB_POWERED_ASCENT      ((agc_job_id_t)0x0F0D) /* P12LM                     */
#define AGC_JOB_POWERED_ASCENT_PROCEED     ((agc_job_id_t)0x0F0E) /* P12LM after V06N33A       */
#define AGC_JOB_NEWLOAD    ((agc_job_id_t)0x0F0F) /* NEWLOAD (V06N76 flash)    */
#define AGC_JOB_NEWLOAD2   ((agc_job_id_t)0x0F10) /* NEWLOAD PROCEED           */
#define AGC_JOB_CUTOFF     ((agc_job_id_t)0x0F11) /* CUTOFF (FINDVAC PRIO17)   */
#define AGC_JOB_CUTOFF1    ((agc_job_id_t)0x0F12) /* CUTOFF1 (V16N63 PROCEED)  */
#define AGC_JOB_CUTOFF2    ((agc_job_id_t)0x0F13) /* CUTOFF2                   */
#define AGC_JOB_TERMASC    ((agc_job_id_t)0x0F14) /* TERMASC                   */
#define AGC_JOB_LUNLAND    ((agc_job_id_t)0x0F15) /* LUNLAND (AVGEXIT)         */
#define AGC_JOB_ATMAG      ((agc_job_id_t)0x0F16) /* ATMAG (AVGEXIT)           */
#define AGC_JOB_SERVEXIT   ((agc_job_id_t)0x0F17) /* SERVEXIT (AVGEXIT)        */
#define AGC_JOB_DPS_ABORT        ((agc_job_id_t)0x0F18) /* P70 (V37)                 */
#define AGC_JOB_APS_ABORT        ((agc_job_id_t)0x0F19) /* P71 (V37)                 */
#define AGC_JOB_ENDLLJOB   ((agc_job_id_t)0x0F1A) /* display "terminate" = end */
#define AGC_JOB_IGNALGP    ((agc_job_id_t)0x0F1B) /* IGNALG after LEMPREC wait */
#define AGC_JOB_IGNALOOP   ((agc_job_id_t)0x0F1C) /* IGNALOOP after INTSTALL  */
#define AGC_JOB_LANDJNKF   ((agc_job_id_t)0x0F1D) /* LANDJUNK V06N43 re-flash */
#define AGC_JOB_IGNALGRT   ((agc_job_id_t)0x0F1E) /* IGNALGRT (4.2 restart)   */
#define AGC_JOB_ABORT_AUTO_CONTROL_CHECK    ((agc_job_id_t)0x0F1F) /* UPTHROT1 -3 (P40AUTO)    */
#define AGC_JOB_UPTHROT1   ((agc_job_id_t)0x0F20) /* THROTUP, UPTHROT1        */
#define AGC_JOB_APS_ABORT_OLDTIME     ((agc_job_id_t)0x0F21) /* P71 OLDTIME restart      */
#define AGC_JOB_POWERED_ASCENT_FLASH   ((agc_job_id_t)0x0F22) /* P12 V06N33 re-flash      */
#define AGC_JOB_POWERED_ASCENT_AT_IGNITION     ((agc_job_id_t)0x0F23) /* P12LMB after LEMPREC wait */
#define AGC_JOB_CUTOFF5    ((agc_job_id_t)0x0F24) /* CUTOFF -5 (V16N63 again) */
#define AGC_JOB_POWERED_ASCENT_BURN    ((agc_job_id_t)0x0F25) /* P12RET after PHASCHNG    */
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
void     agc_erase_set_altm(uint16_t word);         /* ALTM counter 060, raw 15-bit word (bit 15 = sign/hi) */
void     agc_erase_set_cdutcmd(int which, agc_sp_t v); /* 0 CDUTCMD 053, 1 CDUSCMD 054 */

/* ---- guid-powered: TEMX, TEMY, TEMZ (PIPA counts READACCS left for P66) - */
agc_sp_t agc_erase_tem(int axis);                   /* TEMX/Y/Z, axis 0..2  */
void     agc_erase_set_tem(int axis, agc_sp_t v);

/* ---- prog-align: REFMF for P57 after P68 (R51P63 is agc_job_lm_fine_align_k in
 * agc_align.h, entered with agc_align_ram()->qmaj set) ------------------ */
agc_status_t agc_align_refmf(agc_interp_t *ip);     /* REFMF (YNBSAV, ZNBSAV, ATTFLAG) */

/* ---- nav-aux / io-nouns: LAT, LONG, ALT erasables (noun 43) ------------- */
agc_status_t agc_navaux_set_latlong(const agc_latlong_t *ll);

/* ---- prog-rendz: R29/R12 radar helpers (RADAR_LEADIN / R10 lamps) ------- */
agc_status_t agc_rendz_flip(uint16_t lamp_bit);     /* FLIP (R10 LR lamps)  */
agc_status_t agc_rendz_clradmod(void);              /* CLRADMOD (RADMODES for R29) */

/* ---- sys-misc (T4RUPT): gimbal-to-pilot matrix elements (R10 meters) ---- */
agc_sp_t agc_erase_m22(void);                       /* M22 = sin(AOG), at 1 */
agc_sp_t agc_erase_m32(void);                       /* M32 = cos(AOG), at 1 */

#endif /* LANDING_PENDING_H */
