/*
 * agc_lm_dap.h -- Luminary 099 LM digital autopilot (owner: dap-lm agent).
 *
 * The RCS reaction-jet autopilot, the descent-engine trim gimbal control,
 * the 1/ACCS control-authority computation, the TIME6 jet-timing list and
 * the RCS failure monitor. Sources: DAPIDLER_PROGRAM, P-AXIS_RCS_AUTOPILOT,
 * Q_R-AXIS_RCS_AUTOPILOT, TJET_LAW, KALMAN_FILTER, TRIM_GIMBAL_CONTROL_SYSTEM,
 * AOSTASK_AND_AOSJOB, DAP_INTERFACE_SUBROUTINES, T6-RUPT_PROGRAMS,
 * RCS_FAILURE_MONITOR, SPS_BACK-UP_RCS_CONTROL (Luminary 099 pp. 1403-1506,
 * 191-192).
 *
 * Scale conventions (as in the AGC):
 *   CDU angles             uint16_t, 15-bit two's complement, scaled at pi rad
 *   attitude errors        agc_sp_t scaled at pi rad
 *   body rates             agc_sp_t scaled at pi/4 rad/s
 *   offset accelerations   agc_dp_t scaled at pi/2 rad/s^2 (AOSQ, AOSR)
 *   deadband DB            agc_sp_t scaled at 45 degrees (pi/4 rad)
 */
#ifndef AGC_LM_DAP_H
#define AGC_LM_DAP_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_exec.h"

/* ---- fresh start / restart (FRESH_START_AND_RESTART.agc pp. 212-219) -- */
agc_status_t agc_lm_dap_fresh_start(void);  /* DAP erasables on fresh start   */
agc_status_t agc_lm_dap_restart(void);      /* STARTSB1 DAP part (both)        */

/* ---- alarm codes (octal, as in the listing) ----------------------------- */
#define AGC_ALARM_02000 ((agc_alarm_code_t)02000) /* BAILOUT: DAP pass overrun  */
#define AGC_ALARM_02001 ((agc_alarm_code_t)02001) /* no Y,Z translation policy  */
#define AGC_ALARM_02002 ((agc_alarm_code_t)02002) /* no X translation policy    */
#define AGC_ALARM_02003 ((agc_alarm_code_t)02003) /* no P rotation policy       */
#define AGC_ALARM_02004 ((agc_alarm_code_t)02004) /* no U,V rotation policy     */

/* ---- interrupts --------------------------------------------------------- */
agc_status_t agc_lm_dap_t5rupt(void);       /* T5RUPT via T5ADR (DAPIDLER/PAXIS) */
agc_status_t agc_lm_dap_t6rupt(void);       /* DOT6RUPT                          */
agc_status_t agc_lm_dap_t6jobchk(void);     /* T6JOBCHK                          */
void         agc_lm_dap_time6_count(uint16_t counts); /* sim: TIME6 DINC pulses */
bool         agc_lm_dap_t6_pending(void);   /* TIME6 = -0: a T6RUPT is due       */
bool         agc_lm_dap_running(void);      /* T5ADR = PAXIS                     */
agc_status_t agc_lm_dap_rcsmon(void);       /* RCSMONIT, from T4RUPT every 480 ms */
/* T5ADR programs as exec tasks (agc_rupt_set_t5_task / dispatch.c).        */
agc_status_t agc_task_dapidler(void);       /* AGC_TASK_DAPIDLER: DAPIDLER     */
agc_status_t agc_task_paxis(void);          /* AGC_TASK_PAXIS: PAXIS           */

/* ---- 1/ACCS (AOSTASK_AND_AOSJOB.agc pp. 1485-1506) ---------------------- */
agc_status_t agc_lm_dap_1_accs(void);              /* BANKCALL 1/ACCS        */
agc_status_t agc_job_1_accjob(agc_job_ctx_t *ctx); /* NOVAC job 1/ACCJOB     */
agc_status_t agc_job_1_accset(agc_job_ctx_t *ctx); /* NOVAC job 1/ACCSET     */

/* ---- DAP interface subroutines (pp. 1406-1409); call with INHINT ------- */
agc_status_t agc_lm_dap_setmaxdb(void);     /* DB = 5.0 deg                    */
agc_status_t agc_lm_dap_setmindb(void);     /* DB = 0.3 deg                    */
agc_status_t agc_lm_dap_restordb(void);     /* DB per DBSELECT                 */
agc_status_t agc_lm_dap_pflitedb(void);     /* DB = 1.0 deg, zero errors       */
agc_status_t agc_lm_dap_zatteror(void);     /* CDUxD = CDUx, STOPRATE          */
agc_status_t agc_lm_dap_stoprate(void);     /* zero commanded rates and lags   */
agc_status_t agc_lm_dap_allcoast(void);     /* coasting-flight DAP init        */
agc_status_t agc_lm_dap_timegmbl(void);     /* TIMEGMBL trim-gimbal null drive */
agc_status_t agc_lm_dap_needles_init(void); /* set RCSFLAGS bit 3 (FDAI init)  */

/* ROT-TOUV: (q, r) -> non-orthogonal (u, v) using COEFFQ/COEFFR.            */
agc_status_t agc_lm_dap_rot_touv(agc_sp_t q, agc_sp_t r,
                                 agc_sp_t *u, agc_sp_t *v);

/* ---- erasables owned by the LM DAP and shared with other modules ------- */
/* KALCMANU / FINDCDUW interface (ERASABLE_ASSIGNMENTS p. 134)              */
uint16_t agc_erase_cduxd(void);             /* desired CDU X, 2's comp at pi */
uint16_t agc_erase_cduyd(void);
uint16_t agc_erase_cduzd(void);
void     agc_erase_set_cduxd(uint16_t v);
void     agc_erase_set_cduyd(uint16_t v);
void     agc_erase_set_cduzd(uint16_t v);
agc_sp_t agc_erase_delcdux(void);           /* -desired 100 ms CDU increment, at pi */
agc_sp_t agc_erase_delcduy(void);
agc_sp_t agc_erase_delcduz(void);
void     agc_erase_set_delcdux(agc_sp_t v);
void     agc_erase_set_delcduy(agc_sp_t v);
void     agc_erase_set_delcduz(agc_sp_t v);
agc_sp_t agc_erase_omegapd(void);           /* desired body rates, at pi/4     */
agc_sp_t agc_erase_omegaqd(void);
agc_sp_t agc_erase_omegard(void);
void     agc_erase_set_omegapd(agc_sp_t v);
void     agc_erase_set_omegaqd(agc_sp_t v);
void     agc_erase_set_omegard(agc_sp_t v);
agc_sp_t agc_erase_delperor(void);          /* KALCMANU command lags, at pi    */
agc_sp_t agc_erase_delqeror(void);
agc_sp_t agc_erase_delreror(void);
void     agc_erase_set_delperor(agc_sp_t v);
void     agc_erase_set_delqeror(agc_sp_t v);
void     agc_erase_set_delreror(agc_sp_t v);
/* estimator outputs                                                        */
agc_sp_t agc_erase_omegap(void);            /* body rates, at pi/4             */
agc_sp_t agc_erase_omegaq(void);
agc_sp_t agc_erase_omegar(void);
agc_dp_t agc_erase_aosq(void);              /* offset accel, DP at pi/2        */
agc_dp_t agc_erase_aosr(void);
void     agc_erase_set_aosq(agc_dp_t v);
void     agc_erase_set_aosr(agc_dp_t v);
agc_sp_t agc_erase_alphaq(void);            /* downlink copies of AOSQ/R       */
agc_sp_t agc_erase_alphar(void);
agc_sp_t agc_erase_perror(void);            /* attitude errors, at pi          */
agc_sp_t agc_erase_qerror(void);
agc_sp_t agc_erase_rerror(void);
agc_sp_t agc_erase_1jacc(void);             /* 1-jet P accel, at pi/4          */
agc_sp_t agc_erase_1jaccq(void);            /* 1-jet Q accel, at pi/4          */
agc_sp_t agc_erase_1jaccr(void);            /* 1-jet R accel, at pi/4          */
agc_sp_t agc_erase_kq(void);                /* GTS jerk gain, at pi/2^8        */
/* crew/pad-load parameters                                                 */
agc_sp_t agc_erase_db(void);                /* deadband, at 45 deg             */
void     agc_erase_set_db(agc_sp_t v);
agc_sp_t agc_erase_stiksens(void);          /* RHC sensitivity                 */
void     agc_erase_set_stiksens(agc_sp_t v);
agc_sp_t agc_erase_neg_ratedb(void);        /* -RATEDB, rate deadband (neg)    */
void     agc_erase_set_neg_ratedb(agc_sp_t v);
agc_sp_t agc_erase_hiascent(void);          /* max ascent mass, B16 kg         */
void     agc_erase_set_hiascent(agc_sp_t v);
uint16_t agc_erase_ch5mask(void);           /* failed-jet masks, chan formats  */
uint16_t agc_erase_ch6mask(void);

#endif /* AGC_LM_DAP_H */
