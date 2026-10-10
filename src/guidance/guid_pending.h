/*
 * guid_pending.h -- shared symbols guid-powered needs from other lanes that
 * do not exist yet (see docs/modules/guid-powered.md, "Requested core
 * additions" and "Requested jobs-tasks"). Names follow the convention of
 * docs/agents/_common.md. At integration, delete each block whose real
 * declaration exists and include that header instead; a duplicate or
 * conflicting definition error points at the block to remove.
 *
 * Vehicle-only requests are in src/csm/guid_pending_csm.h and
 * src/lm/guid_pending_lm.h, which include this file.
 */
#ifndef GUID_PENDING_H
#define GUID_PENDING_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_jobs.h"
#include "agc/agc_state.h"
#include "agc/agc_channels.h"
#include "agc/agc_dsky.h"
#include "agc/agc_imu.h"
#include "agc/agc_programs.h"
#include "agc/agc_guid.h"

/* ---- executive / service routines --------------------------------------- */
/* DELAYJOB (sys-misc, src/io/sysmisc.h): sleep this job delay_cs, then
 * continue as wake_as.                                                     */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as);
/* KILLTASK (BURN_BABY_BURN p.750, exec lane): remove the first waitlist
 * entry for task; no action if none. Leaves interrupts inhibited.          */
agc_status_t agc_waitlist_kill(agc_task_id_t task);
/* CANV37 (FRESH_START_AND_RESTART, exec lane): continue the V37 major-mode
 * change after the average-G loop has been stopped (POSTJUMP CANV37).      */
agc_status_t agc_exec_canv37(const agc_job_ctx_t *ctx);
/* TBASEn: CS TIME1 / TS TBASEn without a phase change (exec lane).         */
agc_status_t agc_phase_set_tbase(int group);

/* ---- navigation: precision update and average-G hand-over (nav-integ) --- */
agc_status_t agc_integ_thisprec(agc_dp_t tdec1, agc_guid_prec_t *out); /* THISPREC */
agc_status_t agc_integ_csmprec(agc_dp_t tdec1, agc_guid_prec_t *out);  /* CSMPREC  */
agc_status_t agc_integ_lemprec(agc_dp_t tdec1, agc_guid_prec_t *out);  /* LEMPREC  */
/* MIDTOAV1: integrate RN VN to TDEC1 (or to now if TDEC1 is past), leave
 * RN1 VN1 PIPTIME1 (agc_guid_state()->avg); *dt = time from now to that
 * PIPTIME1 (cs, 2^28); *late = the integration ran past TDEC1 (the AGC's
 * return to L+2).                                                          */
agc_status_t agc_integ_midtoav1(agc_dp_t tdec1, agc_dp_t *dt, bool *late);
agc_status_t agc_integ_midtoav2(agc_dp_t *dt);                  /* MIDTOAV2 */
agc_status_t agc_integ_avetomid(void);                          /* AVETOMID */

/* ---- rendezvous targeting utilities (prog-rendz) ------------------------ */
/* HAVEGUES: INITVEL without setting GUESSW (use the COGA guess).            */
agc_status_t agc_rendz_havegues(agc_interp_t *ip, agc_rendz_t *rz,
                                agc_sp_t numit, agc_dp_t e4);

/* ---- job and task ids (lead adds them to agc_jobs.h) -------------------- */
#define AGC_JOB_GUID_BASE 0x400
#define AGC_JOB_SERVICER      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 1))  /* SERVICER  */
#define AGC_JOB_NORMLIZE      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 2))  /* NORMLIZE  */
#define AGC_JOB_SERVEXIT      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 3))  /* SERVEXIT  */
#define AGC_JOB_AVGEND        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 4))  /* AVGEND    */
#define AGC_JOB_LASTBIAS      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 5))  /* LASTBIAS (imu) */
#define AGC_JOB_STEERING      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 6))  /* STEERING  */
#define AGC_JOB_CALCN85       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 7))  /* CALCN85   */
#define AGC_JOB_CALCN83       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 8))  /* CALCN83   */
#define AGC_JOB_S40_9         ((agc_job_id_t)(AGC_JOB_GUID_BASE + 9))  /* S40.9     */
#define AGC_JOB_S40_13        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 10)) /* S40.13    */
#define AGC_JOB_P30           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 11)) /* P30       */
#define AGC_JOB_P31           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 12)) /* P31       */
#define AGC_JOB_P30_N81       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 13)) /* P30 after N33 PRO */
#define AGC_JOB_P30_CALC      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 14)) /* P30 after N81 PRO */
#define AGC_JOB_P30_PARAM     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 15)) /* PARAM30   */
#define AGC_JOB_P30_REFTEST   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 16)) /* REFTEST   */
#define AGC_JOB_P30_END45     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 17)) /* END45     */
#define AGC_JOB_P31_CALC      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 18)) /* P31 after N33 PRO */
#define AGC_JOB_P31_N81       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 19)) /* P31 after N81 PRO */
#define AGC_JOB_REVN1645      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 20)) /* REVN1645 (prog-rendz) */
#define AGC_JOB_P30_DISP45    ((agc_job_id_t)(AGC_JOB_GUID_BASE + 21)) /* DISP45    */
#define AGC_JOB_POSTBURN      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 22)) /* POSTBURN  */
#define AGC_JOB_TIGNOW        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 23)) /* TIGNOW    */
#define AGC_JOB_POST41        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 24)) /* POST41 / TERM40 */
#define AGC_JOB_CLOCKJOB      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 25)) /* CLOCKJOB / CLOKJOB */
#define AGC_JOB_P40           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 26)) /* P40CSM / P40LM */
#define AGC_JOB_P41           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 27)) /* P41CSM / P41LM */
#define AGC_JOB_P42           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 28)) /* P42LM     */
#define AGC_JOB_P47           ((agc_job_id_t)(AGC_JOB_GUID_BASE + 29)) /* P47CSM / P47LM */
#define AGC_JOB_P47BODY       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 30)) /* P47BODY   */
#define AGC_JOB_P40_AFTER_R60 ((agc_job_id_t)(AGC_JOB_GUID_BASE + 31)) /* return from R60 */
#define AGC_JOB_P40_TRIMONLY  ((agc_job_id_t)(AGC_JOB_GUID_BASE + 33)) /* TRIMONLY  */
#define AGC_JOB_P40_TSTTRIM   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 34)) /* TST,TRIM  */
#define AGC_JOB_P40S_SV       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 35)) /* P40S/SV   */
#define AGC_JOB_P40BLNKR      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 36)) /* P40BLNKR  */
#define AGC_JOB_P40RCS        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 37)) /* P40RCS    */
#define AGC_JOB_P40MINDB      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 38)) /* P40MINDB  */
#define AGC_JOB_DYNDISP       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 39)) /* DYNDISP / DYNMDISP */
#define AGC_JOB_V99E          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 40)) /* V99E      */
#define AGC_JOB_V99T          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 41)) /* V99T      */
#define AGC_JOB_V99P          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 42)) /* V99P      */
#define AGC_JOB_V97T          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 43)) /* V97T      */
#define AGC_JOB_V97P          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 44)) /* V97P      */
#define AGC_JOB_V97E          ((agc_job_id_t)(AGC_JOB_GUID_BASE + 45)) /* V97E      */
#define AGC_JOB_COMFAIL       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 47)) /* COMFAIL (LM) */
#define AGC_JOB_LRHJOB        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 48)) /* LRHJOB    */
#define AGC_JOB_LRVJOB        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 49)) /* LRVJOB    */
#define AGC_JOB_HIGATJOB      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 50)) /* HIGATJOB  */
#define AGC_JOB_LRHJOB_DONE   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 51)) /* LRHJOB after RADSTALL */
#define AGC_JOB_LRVJOB_DONE   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 52)) /* LRVJOB after RADSTALL */
#define AGC_JOB_HIGATJOB_DONE ((agc_job_id_t)(AGC_JOB_GUID_BASE + 53)) /* HIGATJOB after RADSTALL */
#define AGC_JOB_POSALARM_PRO  ((agc_job_id_t)(AGC_JOB_GUID_BASE + 54)) /* POSALARM PROCEED */
#define AGC_JOB_P41BLANK      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 55)) /* P41BLANK  */
#define AGC_JOB_TIG_30A       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 56)) /* TIG-30A   */
#define AGC_JOB_BURNBABY      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 57)) /* BURNBABY  */
#define AGC_JOB_P40AUTO_RET   ((agc_job_id_t)(AGC_JOB_GUID_BASE + 58)) /* P40A/P recycle */
#define AGC_JOB_CUTOFF        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 59)) /* CUTOFF (guid-landing) */
#define AGC_JOB_ASTNRET       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 60)) /* ASTNRET (guid-landing) */
#define AGC_JOB_CLOK_V97      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 61)) /* CLOKJOB V97 responses */
#define AGC_JOB_CLOK_V99      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 62)) /* CLOKJOB V99 responses */
#define AGC_JOB_CLOK_N61      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 63)) /* CLOKJOB V06N61 PROCEED */
#define AGC_JOB_TRIMDONE      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 64)) /* TRIMDONE (R03, sys-misc) */
#define AGC_JOB_P40_ALARM     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 65)) /* REP40ALM  */
#define AGC_JOB_P40_ALARM_PRO ((agc_job_id_t)(AGC_JOB_GUID_BASE + 66)) /* REP40ALM PROCEED */
#define AGC_JOB_P47BOD        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 68)) /* P47BOD    */
#define AGC_JOB_STOPCLOK      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 69)) /* STOPCLOK  */
#define AGC_JOB_REDO40_9      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 70)) /* REDO40.9  */
#define AGC_JOB_REDO6_7       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 71)) /* REDO6.7 / BLNKWAIT */
#define AGC_JOB_CLOK_V97E     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 72)) /* V97 ENTER (COMFAIL2) */
#define AGC_JOB_CLOK_V99E     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 73)) /* V99 ENTER (*ENTER)  */
#define AGC_JOB_SERVIDLE      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 74)) /* SERVIDLE  */
#define AGC_JOB_DAPFIG        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 75)) /* DAPFIG    */
#define AGC_JOB_P30_ALL_DONE  ((agc_job_id_t)(AGC_JOB_GUID_BASE + 76)) /* P30/P31 N42 PROCEED (LM) */
#define AGC_JOB_TERM40        ((agc_job_id_t)(AGC_JOB_GUID_BASE + 77)) /* TERM40 (LM) */
#define AGC_JOB_P41_AFTER_R60 ((agc_job_id_t)(AGC_JOB_GUID_BASE + 78)) /* P41LM after R60LEM */


#define AGC_JOB_LUNLAND       ((agc_job_id_t)(AGC_JOB_GUID_BASE + 81)) /* LUNLAND (guid-landing) */
#define AGC_JOB_P63DISPS      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 82)) /* P63DISPS -2 (guid-landing) */
#define AGC_JOB_ATMAG         ((agc_job_id_t)(AGC_JOB_GUID_BASE + 83)) /* ATMAG (guid-landing) */
#define AGC_JOB_POSECADR      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 84)) /* CM/POSE (guid-entry) */
#define AGC_JOB_P11SC         ((agc_job_id_t)(AGC_JOB_GUID_BASE + 85)) /* P11 AVGEXIT (guid-entry) */
#define AGC_JOB_SERV_DVTOTUP  ((agc_job_id_t)(AGC_JOB_GUID_BASE + 86)) /* CM DVTOTUP / LM GETABVAL restart */
#define AGC_JOB_SERV_AVERAGEG ((agc_job_id_t)(AGC_JOB_GUID_BASE + 87)) /* AVERAGEG restart (redo CALCRVG) */
#define AGC_JOB_SERV_COPY     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 88)) /* copy cycle restart  */
#define AGC_JOB_SERV_EXIT     ((agc_job_id_t)(AGC_JOB_GUID_BASE + 89)) /* go to AVGEXIT restart */
#define AGC_JOB_AVETOMID      ((agc_job_id_t)(AGC_JOB_GUID_BASE + 90)) /* AVGEND 2.2 restart (AVETOMID) */
#define AGC_JOB_P40_DSKY_DONE AGC_JOB_ENDOFJOB

#define AGC_TASK_GUID_BASE 0x400
#define AGC_TASK_READACCS     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 1))  /* READACCS  */
#define AGC_TASK_PREREAD      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 2))  /* PREREAD   */
#define AGC_TASK_ONMNITOR     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 3))  /* ONMNITOR  */
#define AGC_TASK_QUIKREAD     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 4))  /* QUIKREAD  */
#define AGC_TASK_SETJTAG      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 5))  /* SETJTAG (guid-entry) */
#define AGC_TASK_CLOKTASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 6))  /* CLOKTASK  */
#define AGC_TASK_TIGBLNK      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 7))  /* TIGBLNK   */
#define AGC_TASK_TIGAVEG      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 8))  /* TIGAVEG   */
#define AGC_TASK_TIG_5        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 9))  /* TIG-5     */
#define AGC_TASK_TIG_0        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 10)) /* TIG-0     */
#define AGC_TASK_IGNITION     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 11)) /* IGNITION  */
#define AGC_TASK_DOTVCON      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 12)) /* DOTVCON   */
#define AGC_TASK_DOSTRULL     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 13)) /* DOSTRULL  */
#define AGC_TASK_ENGINOFF     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 14)) /* ENGINOFF / ENGOFTSK */
#define AGC_TASK_DOTVCRCS     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 15)) /* DOTVCRCS  */
#define AGC_TASK_TTG_0        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 16)) /* TTG/0     */
#define AGC_TASK_TIGON        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 17)) /* TIGON / STARTP47 */
#define AGC_TASK_S40_6        ((agc_task_id_t)(AGC_TASK_GUID_BASE + 18)) /* S40.6     */
#define AGC_TASK_PRE40_6      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 19)) /* PRE40.6   */
#define AGC_TASK_S40_6_B      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 20)) /* S40.6 after 60 ms */
#define AGC_TASK_S40_6_C      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 21)) /* RSTRTST   */
#define AGC_TASK_S40_6_OUT    ((agc_task_id_t)(AGC_TASK_GUID_BASE + 22)) /* OUTPUT return / GIMTRIM */
#define AGC_TASK_V97TTASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 24)) /* V97TTASK  */
#define AGC_TASK_V97TRCS      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 25)) /* V97TRCS   */
#define AGC_TASK_V97PTASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 26)) /* V97PTASK  */
#define AGC_TASK_R40ENABL     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 27)) /* R40ENABL  */
#define AGC_TASK_V97ETASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 28)) /* V97ETASK  */
#define AGC_TASK_V97E40_6     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 29)) /* V97E40.6  */
#define AGC_TASK_V99FLASH     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 30)) /* V99FLASH  */
#define AGC_TASK_TIG_35       ((agc_task_id_t)(AGC_TASK_GUID_BASE + 31)) /* TIG-35    */
#define AGC_TASK_TIG_30       ((agc_task_id_t)(AGC_TASK_GUID_BASE + 32)) /* TIG-30    */
#define AGC_TASK_TIG_30_1     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 33)) /* TIG-30.1  */
#define AGC_TASK_ULLGTASK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 34)) /* ULLGTASK  */
#define AGC_TASK_ULLAGOFF     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 35)) /* ULLAGOFF  */
#define AGC_TASK_P63ZOOM      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 36)) /* P63ZOOM   */
#define AGC_TASK_P40ZOOM      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 37)) /* P40ZOOM   */
#define AGC_TASK_TIGTASK      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 38)) /* TIGTASK   */
#define AGC_TASK_COASTSET     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 39)) /* COASTSET  */
#define AGC_TASK_IMPLBURN2    ((agc_task_id_t)(AGC_TASK_GUID_BASE + 40)) /* IMPLBURN after 0.5 s */
#define AGC_TASK_TRIMGIMB     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 41)) /* TRIMGIMB  */
#define AGC_TASK_TRIMGIMB2    ((agc_task_id_t)(AGC_TASK_GUID_BASE + 42)) /* TRIMGIMB after 60 s */
#define AGC_TASK_ROLLOFF      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 43)) /* roll off (VARDELAY) */
#define AGC_TASK_PITCHOFF     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 44)) /* PITCHOFF  */
#define AGC_TASK_LRHTASK      ((agc_task_id_t)(AGC_TASK_GUID_BASE + 45)) /* LRHTASK   */
#define AGC_TASK_RDGIMS       ((agc_task_id_t)(AGC_TASK_GUID_BASE + 46)) /* RDGIMS    */
#define AGC_TASK_R10R11       ((agc_task_id_t)(AGC_TASK_GUID_BASE + 47)) /* R10,R11 (guid-landing) */
#define AGC_TASK_IMPLCONT     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 48)) /* IMPLCONT  */
#define AGC_TASK_REREADAC     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 53)) /* REREADAC  */
#define AGC_TASK_DOSPSOFF     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 54)) /* DOSPSOFF  */
#define AGC_TASK_SPSOFF97     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 55)) /* SPSOFF97  */
#define AGC_TASK_PRECHECK     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 56)) /* PRECHECK / REDO4.2 */
#define AGC_TASK_BIBIBIAS     ((agc_task_id_t)(AGC_TASK_GUID_BASE + 57)) /* BIBIBIAS  */
#define AGC_TASK_T5IDLOC_GUID AGC_TASK_T5IDLOC                           /* T5IDLOC   */

/* Priorities: agc_prio_t = PRIOnn (octal) >> 9, i.e. the "nn" in octal.     */
#define AGC_PRIO5  ((agc_prio_t)005)
#define AGC_PRIO6  ((agc_prio_t)006)
#define AGC_PRIO10 ((agc_prio_t)010)
#define AGC_PRIO12 ((agc_prio_t)012)
#define AGC_PRIO13 ((agc_prio_t)013)
#define AGC_PRIO14 ((agc_prio_t)014)
#define AGC_PRIO15 ((agc_prio_t)015)
#define AGC_PRIO16 ((agc_prio_t)016)
#define AGC_PRIO17 ((agc_prio_t)017)
#define AGC_PRIO20 ((agc_prio_t)020)
#define AGC_PRIO21 ((agc_prio_t)021)
#define AGC_PRIO22 ((agc_prio_t)022)
#define AGC_PRIO23 ((agc_prio_t)023)
#define AGC_PRIO25 ((agc_prio_t)025)
#define AGC_PRIO27 ((agc_prio_t)027)
#define AGC_PRIO30 ((agc_prio_t)030)
#define AGC_PRIO31 ((agc_prio_t)031)
#define AGC_PRIO32 ((agc_prio_t)032)

/* Alarm codes raised by this lane (octal, as in the listings).             */
#define AGC_ALARM_00205 ((agc_alarm_code_t)00205) /* CM: PIPA saturated        */
#define AGC_ALARM_00511 ((agc_alarm_code_t)00511) /* LM: LR not in position    */
#define AGC_ALARM_00523 ((agc_alarm_code_t)00523) /* LM: LR repositioning failed */
#define AGC_ALARM_01407 ((agc_alarm_code_t)01407) /* VG increasing (thrust > 90 deg off) */
#define AGC_ALARM_01706 ((agc_alarm_code_t)01706) /* LM: P40/P42 vs. stage config */

#endif /* GUID_PENDING_H */
