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

/* ---- jobs / tasks (lead registers them in agc_jobs.h + dispatch.c) ------
 * Provisional ids in this lane's private range 0x0E00 / 0x0E40.           */
#define AGC_JOB_P63LM      ((agc_job_id_t)0x0E01) /* P63LM                    */
#define AGC_JOB_ASTNCLOK   ((agc_job_id_t)0x0E02) /* IGNALGRT/ASTNCLOK tail    */
#define AGC_JOB_ASTNRET    ((agc_job_id_t)0x0E03) /* ASTNRET (STCLOK2 return)  */
#define AGC_JOB_P63SPOT2   ((agc_job_id_t)0x0E04) /* P63SPOT2 (R51 ENTER)      */
#define AGC_JOB_P63SPOT3   ((agc_job_id_t)0x0E05) /* P63SPOT3 (after R60)      */
#define AGC_JOB_P63SPOT4   ((agc_job_id_t)0x0E06) /* P63SPOT4 (CODE 500 ENTER) */
#define AGC_JOB_P64CEED    ((agc_job_id_t)0x0E07) /* P64CEED (V06N64 PROCEED)  */
#define AGC_JOB_P64DISPS   ((agc_job_id_t)0x0E08) /* P64DISPS (V06N64 ENTER)   */
#define AGC_JOB_RODCOMP    ((agc_job_id_t)0x0E09) /* RODCOMP (FINDVAC PRIO22)  */
#define AGC_JOB_LANDJUNK   ((agc_job_id_t)0x0E0A) /* LANDJUNK (P68)            */
#define AGC_JOB_LANDJNK2   ((agc_job_id_t)0x0E0B) /* LANDJUNK V06N43 PROCEED   */
#define AGC_JOB_GOABORT    ((agc_job_id_t)0x0E0C) /* GOABORT (4.3 restart)     */
#define AGC_JOB_P12LM      ((agc_job_id_t)0x0E0D) /* P12LM                     */
#define AGC_JOB_P12LM2     ((agc_job_id_t)0x0E0E) /* P12LM after V06N33A       */
#define AGC_JOB_NEWLOAD    ((agc_job_id_t)0x0E0F) /* NEWLOAD (V06N76 flash)    */
#define AGC_JOB_NEWLOAD2   ((agc_job_id_t)0x0E10) /* NEWLOAD PROCEED           */
#define AGC_JOB_CUTOFF     ((agc_job_id_t)0x0E11) /* CUTOFF (FINDVAC PRIO17)   */
#define AGC_JOB_CUTOFF1    ((agc_job_id_t)0x0E12) /* CUTOFF1 (V16N63 PROCEED)  */
#define AGC_JOB_CUTOFF2    ((agc_job_id_t)0x0E13) /* CUTOFF2                   */
#define AGC_JOB_TERMASC    ((agc_job_id_t)0x0E14) /* TERMASC                   */
#define AGC_JOB_LUNLAND    ((agc_job_id_t)0x0E15) /* LUNLAND (AVGEXIT)         */
#define AGC_JOB_ATMAG      ((agc_job_id_t)0x0E16) /* ATMAG (AVGEXIT)           */
#define AGC_JOB_SERVEXIT   ((agc_job_id_t)0x0E17) /* SERVEXIT (AVGEXIT)        */
#define AGC_JOB_P70        ((agc_job_id_t)0x0E18) /* P70 (V37)                 */
#define AGC_JOB_P71        ((agc_job_id_t)0x0E19) /* P71 (V37)                 */
#define AGC_JOB_ENDLLJOB   ((agc_job_id_t)0x0E1A) /* display "terminate" = end */
#define AGC_TASK_RODTASK   ((agc_task_id_t)0x0E41) /* RODTASK                  */
#define AGC_TASK_REDESMON  ((agc_task_id_t)0x0E42) /* REDESMON (PITFALL)       */
#define AGC_TASK_ENGOFF1   ((agc_task_id_t)0x0E43) /* ENGOFF1                  */
#define AGC_TASK_R10_R11   ((agc_task_id_t)0x0E44) /* R10,R11 (every .5 s)     */
#define AGC_TASK_INTLZE    ((agc_task_id_t)0x0E45) /* INTLZE (R10)             */
#define AGC_TASK_SPEEDRN2  ((agc_task_id_t)0x0E46) /* SPEEDRUN after VARDELAY  */

/* ---- alarm codes raised by this lane (octal) ---------------------------- */
#define AGC_ALARM_00401 ((agc_alarm_code_t)00401) /* FINDCDUW: gimbal lock     */
#define AGC_ALARM_00402 ((agc_alarm_code_t)00402) /* FINDCDUW: NORMUNIT ovf    */
#define AGC_ALARM_01406 ((agc_alarm_code_t)01406) /* TTF/8 no convergence      */
#define AGC_ALARM_01410 ((agc_alarm_code_t)01410) /* overflow in guidance      */
#define AGC_ALARM_01412 ((agc_alarm_code_t)01412) /* IGNALG NIGNLOOP exhausted */

/* ---- DAPBOOLS (= FLAGWRD13) bit masks; flags exist, masks do not -------- */
#define LM_DAPBOOLS_XOVINHIB 0x0100u   /* BIT9  XOVINHIB                     */
#define LM_DAPBOOLS_CSMDOCKD 0x1000u   /* BIT13 CSMDOCKD                     */
#define LM_DAPBOOLS_DAPBITS  0x01A0u   /* DAPBITS OCT 00640 (bits 6, 8, 9)   */
#define LM_FLAGWRD_DAPBOOLS  13
#define LM_FLAGWRD_FLGWRD11  11
#define LM_FLGWRD11_LRBYBIT  0x4000u   /* LRBYBIT = BIT15                    */

/* ---- erasables / counters with no accessor yet (state-flags, dap-lm) ---- */
void     agc_erase_set_thrust(agc_sp_t pulses);     /* THRUST counter 055   */
void     agc_erase_set_altm(agc_sp_t v);            /* ALTM counter 060     */
void     agc_erase_set_cdutcmd(int which, agc_sp_t v); /* 0 CDUTCMD 053, 1 CDUSCMD 054 */
void     agc_erase_set_time5(agc_sp_t v);           /* TIME5 (T5RUPT)       */
agc_sp_t agc_erase_tem(int axis);                   /* TEMX/Y/Z             */
void     agc_erase_set_tem(int axis, agc_sp_t v);
agc_sp_t agc_erase_cduxd(int axis);                 /* CDUXD/YD/ZD (dap)    */
void     agc_erase_set_delcdux(int axis, agc_dp_t v); /* DELCDUX/Y/Z; LM: SP in high word */
void     agc_erase_set_omegapd(int axis, agc_sp_t v); /* OMEGAPD/QD/RD pi/4 r/s */
void     agc_erase_set_delperor(int axis, agc_sp_t v);/* DELPEROR/QEROR/REROR  */
agc_sp_t agc_erase_1jacc(int axis);                 /* 1JACC/Q/R pi/4 r/s^2 */

/* ---- dap-lm: DAP_INTERFACE_SUBROUTINES ---------------------------------- */
agc_status_t agc_lm_dap_zatteror(void);             /* ZATTEROR             */
agc_status_t agc_lm_dap_setmaxdb(void);             /* SETMAXDB             */
agc_status_t agc_lm_dap_setmindb(void);             /* SETMINDB             */
agc_status_t agc_lm_dap_restordb(void);             /* RESTORDB             */
agc_status_t agc_lm_dap_pflitedb(void);             /* PFLITEDB             */

/* ---- exec-sched / io-pinball / prog-rendz / prog-align ------------------ */
agc_status_t agc_exec_enema(void);                  /* ENEMA                */
agc_status_t agc_dsky_falton(void);                 /* FALTON               */
agc_status_t agc_rendz_flip(uint16_t lamp_bit);     /* FLIP (R10 LR lamps)  */
agc_status_t agc_rendz_clradmod(void);              /* CLRADMOD             */
agc_status_t agc_align_refmf(agc_interp_t *ip);     /* REFMF (P57 init)     */
agc_status_t agc_align_set_gsav(agc_vec_t v);       /* GSAV                 */
agc_status_t agc_imu_r51p63(const agc_job_ctx_t *ctx, agc_job_id_t on_proceed,
                            agc_job_id_t on_enter); /* R51 called from P63  */

/* ---- nav-integ: precision integration (INTEGRATION_INITIALIZATION) ------ */
typedef struct lm_prec_out {
    agc_vec_t ratt;    /* RATT  2^29 m (earth scaling, any primary)          */
    agc_vec_t vatt;    /* VATT  2^7 m/cs                                     */
    agc_vec_t ratt1;   /* RATT1 2^27 m moon / 2^29 earth                     */
    agc_vec_t vatt1;   /* VATT1 2^5 m/cs moon / 2^7 earth                    */
    agc_dp_t  tat;     /* TAT   2^28 cs                                      */
} lm_prec_out_t;
agc_status_t agc_integ_lemprec(agc_dp_t tdec1, lm_prec_out_t *out);  /* LEMPREC */
agc_status_t agc_integ_integrvs(agc_vec_t rcv, agc_vec_t vcv, agc_dp_t tet,
                                bool moon, agc_dp_t tdec1,
                                lm_prec_out_t *out);  /* INTSTALL+INTEGRVS, conic flag clear */

/* ---- guid-powered: SERVICER (LM) and BURN_BABY_BURN ---------------------- */
/* MUNGRAV: UNIT/R/ -> avg.unitr, returns GDT1/2 (2^7 m/cs); r at 2^24 m.  */
agc_vec_t    agc_lm_mungrav(agc_interp_t *ip, agc_vec_t r);
agc_status_t agc_lm_setpos1(void);                  /* SETPOS1              */
agc_status_t agc_lm_lrhtask(void);                  /* LRHTASK              */
agc_status_t agc_lm_burnbaby(const agc_job_ctx_t *ctx); /* BURNBABY (WHICH set) */
agc_status_t agc_lm_stclok2(const agc_job_ctx_t *ctx, agc_sp_t astndex,
                            agc_job_id_t ret_job); /* STCLOK2              */
agc_status_t agc_lm_enginof2(void);                 /* ENGINOF2             */
agc_status_t agc_lm_p40auto(const agc_job_ctx_t *ctx, agc_job_id_t ret_job); /* P40AUTO */
#define LM_WHICH_P12TABLE 0                         /* WHICH = P12TABLE     */
#define LM_WHICH_P63TABLE 1                         /* WHICH = P63TABLE     */
#define LM_WHICH_ABRTABLE 2                         /* WHICH = ABRTABLE     */

#endif /* LANDING_PENDING_H */
