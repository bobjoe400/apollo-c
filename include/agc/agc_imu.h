/*
 * agc_imu.h -- IMU interfaces (imu lane).
 *
 * Sources (CM = Comanche055, LM = Luminary099):
 *   IMU_MODE_SWITCHING_ROUTINES.agc   CM pp.1420-1448, LM pp.1309-1337
 *   IMU_COMPENSATION_PACKAGE.agc      CM pp.297-306,   LM pp.326-337
 *   INFLIGHT_ALIGNMENT_ROUTINES.agc   CM pp.1355-1363, LM pp.1250-1258
 *   GIMBAL_LOCK_AVOIDANCE.agc, KALCMANU_STEERING.agc,
 *   ANGLFIND.agc (CM) / ATTITUDE_MANEUVER_ROUTINE.agc (LM), R60_62.agc
 *   IMU_CALIBRATION_AND_ALIGNMENT.agc (CM prelaunch, src/csm/)
 *
 * Conventions used throughout this header:
 *   * CDU angles (CDUX/Y/Z, THETAD = CPHI/CTHETA/CPSI, NCDU, BCDU) are the
 *     raw 15-bit two's-complement patterns (uint16_t, 2^15 = 1 revolution,
 *     bit 15 = -180 deg), exactly as agc_state.h holds them.  Index order
 *     is X (outer), Y (inner), Z (middle).
 *   * Interpretive angles are DP revolutions, scale 2^0 (agc_dp_t raw/2^28).
 *   * "Half-unit" vectors are unit vectors scaled 2 (each component /2).
 *   * Gyro torquing commands are DP revolutions, scale 2^0; one gyro pulse
 *     is 2^-21 revolution (0.618 arc-second), i.e. 128 LSB of agc_dp_t.
 */
#ifndef AGC_IMU_H
#define AGC_IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "agc_status.h"
#include "agc_word.h"
#include "agc_interp.h"
#include "agc_exec.h"

/* ======================================================================= */
/* Mode switching (IMU_MODE_SWITCHING_ROUTINES.agc)                        */
/* ======================================================================= */

/* Device index used by GOODEND/BADEND/STALL: C(A) on arrival, index into
 * MODECADR (IMUCADR, OPTCADR, RADCADR).                                    */
typedef enum agc_imu_device {
    AGC_IMU_DEV_IMU    = 0,          /* IMUSTALL                           */
    AGC_IMU_DEV_OPTICS = 1,          /* OPTSTALL (CM) / AOTSTALL (LM)      */
    AGC_IMU_DEV_RADAR  = 2,          /* RADSTALL                           */
    AGC_IMU_DEV_COUNT  = 3
} agc_imu_device_t;

/* Result of STALL (the AGC returned to L+2 bad, L+3 good, or slept).      */
typedef enum agc_imu_stall_result {
    AGC_IMU_STALL_GOOD = 0,          /* MODECADR was -1: return to L+3     */
    AGC_IMU_STALL_BAD,               /* MODECADR was -0: return to L+2     */
    AGC_IMU_STALL_SLEEPING           /* MODECADR was +0: job put to sleep  */
} agc_imu_stall_result_t;

/* Gyro command buffers: the ECADR that arrives in A at IMUPULSE.  Each is
 * three DP commands in X, Y, Z order (OGC, IGC, MGC for the OGC buffer).
 * IMUPULSE leaves the unsent fraction of each command in its buffer.      */
typedef enum agc_imu_gyro_buf {
    AGC_IMU_GBUF_OGC    = 0,         /* OGC, IGC, MGC  (CALCGTA, P52, P53) */
    AGC_IMU_GBUF_GCOMP  = 1,         /* GCOMP  (compensation, 1/GYRO)      */
    AGC_IMU_GBUF_ERCOMP = 2,         /* ERCOMP (CM prelaunch earth rate)   */
    AGC_IMU_GBUF_COUNT  = 3
} agc_imu_gyro_buf_t;

agc_status_t agc_imu_mode_init(void);                    /* restart state  */

agc_status_t agc_imu_zero(void);                         /* IMUZERO        */
agc_status_t agc_imu_coarse(const uint16_t thetad[3]);   /* IMUCOARS       */
agc_status_t agc_imu_fine(void);                         /* IMUFINE/IMUFIN20 */
agc_status_t agc_imu_pipuse(void);                       /* PIPUSE         */
agc_status_t agc_imu_pipuse1(void);                      /* PIPUSE1        */
agc_status_t agc_imu_pipfree(void);                      /* PIPFREE        */
agc_status_t agc_imu_rndrefdr(void);                     /* RNDREFDR       */

/* IMUPULSE with an explicit buffer.  If the gyros are busy and wake_as is
 * not AGC_JOB_NONE the calling job is put to sleep (GYROBUSY) and wake_as is
 * woken when the gyros free up; it must call agc_imu_pulse_buf again.
 * Returns AGC_STATUS_BUSY in that case, AGC_STATUS_OK when torquing has
 * started, AGC_STATUS_ALARM if the IMU is caged (CAGETSTJ).               */
agc_status_t agc_imu_pulse_buf(agc_imu_gyro_buf_t buf, agc_job_id_t wake_as);
/* IMUPULSE on the OGC buffer after loading it with gyro[0..2] (X,Y,Z).    */
agc_status_t agc_imu_pulse(const agc_dp_t gyro[3]);
agc_status_t agc_imu_gyro_buf_get(agc_imu_gyro_buf_t buf, agc_dp_t out[3]);
agc_status_t agc_imu_gyro_buf_set(agc_imu_gyro_buf_t buf, const agc_dp_t in[3]);
bool         agc_imu_gyros_busy(void);                   /* LGYRO != 0     */
uint16_t     agc_imu_gyrocmd(void);       /* GYROCMD counter (pulses)      */
agc_sp_t     agc_imu_cducmd(int axis);    /* CDUXCMD..CDUZCMD (pulses)     */

/* STALL: IMUSTALL / OPTSTALL / RADSTALL.  On SLEEPING the caller must
 * return; job wake_as is woken by GOODEND/BADEND and then reads the outcome
 * with agc_imu_stall_outcome().                                           */
agc_status_t agc_imu_stall_dev(agc_imu_device_t dev, agc_job_id_t wake_as,
                               agc_imu_stall_result_t *result);
agc_status_t agc_imu_stall(agc_job_id_t wake_as, agc_imu_stall_result_t *result);
agc_imu_stall_result_t agc_imu_stall_outcome(agc_imu_device_t dev);

agc_status_t agc_imu_goodend(agc_imu_device_t dev);      /* GOODEND        */
agc_status_t agc_imu_badend(agc_imu_device_t dev);       /* BADEND         */
agc_status_t agc_imu_endimu(void);                       /* ENDIMU         */

agc_status_t agc_imu_imubad(void);                       /* IMUBAD         */
agc_status_t agc_imu_r02both(void);                      /* R02BOTH        */
agc_status_t agc_imu_r02zero(void);                      /* R02ZERO        */
agc_status_t agc_imu_p06(agc_job_ctx_t *ctx);            /* P06 (stub)     */
agc_status_t agc_imu_postand(agc_job_ctx_t *ctx);        /* POSTAND (stub) */

/* Subroutines also used by T4RUPT (sys-misc).                             */
agc_status_t agc_imu_zeroicdu(void);     /* ZEROICDU: CDUX/Y/Z = 0         */
agc_status_t agc_imu_noattoff(void);     /* NOATTOFF: NO ATT lamp off      */
agc_status_t agc_imu_setcoars(void);     /* SETCOARS                       */
agc_status_t agc_imu_fin20(void);        /* IMUFIN20 (= IMUFINE)           */
agc_status_t agc_imu_clear_cmds(void);   /* -0 into CDUXCMD..Z, GYROCMD    */
/* MODECADR storage is exec's (agc_erase_modecadr in agc_exec.h); the
 * IMU lane's encoding: +0 free, -1 complete good, -2 complete failed
 * (stands in for -0), positive = id of the stalled job.                  */
/* Simulator side of the hardware command counters: the harness drains
 * GYROCMD and CDUXCMD..CDUZCMD as it emits the pulses.                    */
agc_status_t agc_imu_hw_set_gyrocmd(uint16_t pulses);
agc_status_t agc_imu_hw_set_cducmd(int axis, agc_sp_t pulses);

/* Waitlist tasks of the mode-switching routines (dispatch.c).             */
agc_status_t agc_task_imuzero2(void);
agc_status_t agc_task_imuzero3(void);
agc_status_t agc_task_coars(void);
agc_status_t agc_task_coars2(void);
agc_status_t agc_task_chkcors(void);
agc_status_t agc_task_imufined(void);
agc_status_t agc_task_ifailok(void);
agc_status_t agc_task_pfailok(void);
agc_status_t agc_task_strtgyro(void);
agc_status_t agc_task_8192aug(void);
agc_status_t agc_task_ca_ece(void);      /* CA+ECE (T4RUPT)                */

/* ======================================================================= */
/* Compensation (IMU_COMPENSATION_PACKAGE.agc)                             */
/* ======================================================================= */

/* Pad-loaded compensation parameters (B(1) erasables).                    */
typedef struct agc_imu_comp_params {
    agc_sp_t pipabias[3]; /* PBIASX..Z (PIPA pulses)/cs, 2^-5 LM, 2^-8 CM  */
    agc_sp_t pipascf[3];  /* PIPASCFX..Z (ppm) x 2^-9                      */
    agc_sp_t nbd[3];      /* NBDX..Z (gyro pulses)/cs x 2^-5               */
    agc_sp_t adia[3];     /* ADIAX..Z (gyro pulses)/(PIPA pulse) 2^-6 LM / 2^-3 CM */
    agc_sp_t adsra[3];    /* ADSRAX..Z same scale as ADIA                  */
} agc_imu_comp_params_t;

agc_status_t agc_imu_comp_load(const agc_imu_comp_params_t *p);
agc_status_t agc_imu_comp_get(agc_imu_comp_params_t *p);
agc_sp_t     agc_imu_gcompsw(void);                      /* GCOMPSW        */
void         agc_imu_set_gcompsw(agc_sp_t v);
agc_sp_t     agc_imu_1_pipadt(void);   /* 1/PIPADT, cs x 2^8 (or TIME1)   */
void         agc_imu_set_1_pipadt(agc_sp_t v);

/* 1/PIPA: compensate the PIPA counts DELV (pulses x 2^14, X Y Z) in place
 * for scale factor and bias, then IRIGCOMP the gyros.                     */
agc_status_t agc_imu_one_over_pipa(agc_vec_t *delv);
/* IRIGCOMP: gyro compensation from compensated DELV (pulses x 2^14).     */
agc_status_t agc_imu_comp_irig(agc_vec_t delv);
agc_status_t agc_imu_gcompzer(void);                     /* GCOMPZER       */
agc_status_t agc_job_1_gyro(agc_job_ctx_t *ctx);         /* 1/GYRO         */
agc_status_t agc_job_1_gyro_pulse(agc_job_ctx_t *ctx);   /* 1/GYRO retry   */
agc_status_t agc_job_gcomp1(agc_job_ctx_t *ctx);         /* GCOMP1         */
agc_status_t agc_job_nbdonly(agc_job_ctx_t *ctx);        /* NBDONLY        */
agc_status_t agc_job_lastbias(agc_job_ctx_t *ctx);       /* LASTBIAS       */

/* ======================================================================= */
/* Inflight alignment (INFLIGHT_ALIGNMENT_ROUTINES.agc, CALCSMSC)          */
/* ======================================================================= */

/* ARCTRIG: angle from sin/4 and cos/4 -> revolutions in [-.5, .5).        */
agc_dp_t     agc_imu_arctrig(agc_interp_t *ip, agc_dp_t sinth, agc_dp_t costh);
/* CALCGTA: dc rows = XDC, YDC, ZDC (half-unit) -> gyro[] = OGC, IGC, MGC
 * (revolutions).  Also loads the OGC gyro buffer.                         */
agc_status_t agc_imu_calcgta(agc_interp_t *ip, const agc_mat_t *dc, agc_dp_t gyro[3]);
/* CALCGA: nb rows = XNB, YNB, ZNB; sm rows = XSM, YSM, ZSM (half-unit,
 * same reference frame) -> thetad[] = OGC, IGC, MGC as CDU patterns.
 * Stores THETAD unless (CM) CPHIFLAG was set; alarm 00401 + GLOKFAIL when
 * the middle gimbal exceeds 60 deg.                                      */
agc_status_t agc_imu_calcga(agc_interp_t *ip, const agc_mat_t *nb,
                            const agc_mat_t *sm, uint16_t thetad[3]);
/* AXISGEN: a[0],a[1] = star 1, 2 in frame A; b[0],b[1] in frame B (half-
 * unit) -> rows XDC, YDC, ZDC = axes of A referred to B (half-unit).      */
agc_status_t agc_imu_axisgen(agc_interp_t *ip, const agc_vec_t a[2],
                             const agc_vec_t b[2], agc_mat_t *a_wrt_b);
/* CALCSMSC: sin/cos of CDU angles (X Y Z order, each scaled 2, i.e. sin/2)
 * -> rows XNB, YNB, ZNB in stable-member axes (half-unit).                 */
agc_status_t agc_imu_calcsmsc(agc_interp_t *ip, agc_vec_t sincdu,
                              agc_vec_t coscdu, agc_mat_t *nb);

/* ======================================================================= */
/* Attitude maneuvers: KALCMANU (ANGLFIND / ATTITUDE_MANEUVER_ROUTINE,      */
/* KALCMANU_STEERING, GIMBAL_LOCK_AVOIDANCE)                               */
/* ======================================================================= */

/* CDUTODCM: CDU patterns (X Y Z) -> S/C to SM matrix, elements scale 2.   */
agc_status_t agc_kalc_cdutodcm(agc_interp_t *ip, const uint16_t cdu[3], agc_mat_t *m);
/* DCMTOCDU: matrix (scale 2) -> gimbal angles (X Y Z) in revolutions;
 * *cos_mga = cos(middle gimbal), scale 1 (left in S1 by the AGC).          */
agc_status_t agc_kalc_dcmtocdu(agc_interp_t *ip, const agc_mat_t *m,
                               agc_vec_t *angles, agc_dp_t *cos_mga);
/* DELCOMP: rotation of angle a (revolutions) about half-unit axis u ->
 * DEL, elements scale 1.                                                  */
agc_status_t agc_kalc_delcomp(agc_interp_t *ip, agc_vec_t u, agc_dp_t a, agc_mat_t *del);

/* Result of the KALCMAN3 set-up computation (no executive side effects). */
typedef enum agc_kalc_case {
    AGC_KALC_SMALL = 0,         /* AM < .25 deg: go straight to hold        */
    AGC_KALC_NORMAL,            /* COF from the skew-symmetric part of MFI  */
    AGC_KALC_LARGE,             /* AM > 170 deg: COF from MFISYM (ALTCALC)  */
    AGC_KALC_TOOBAD             /* LM: |CPSI| > 70 deg (TOOBADF, alarm 401) */
} agc_kalc_case_t;

typedef struct agc_kalc_setup {
    agc_kalc_case_t kcase;
    agc_mat_t mis;              /* initial S/C -> SM, scale 2              */
    agc_mat_t mfs;              /* final S/C -> SM, scale 2                */
    agc_mat_t mfi;              /* final -> initial S/C, scale 4           */
    agc_vec_t cofskew;          /* skew part of MFI, scale 4               */
    agc_dp_t  cam;              /* cos(AM)/2                               */
    agc_dp_t  am;               /* maneuver angle, revolutions             */
    agc_vec_t cof;              /* maneuver axis, half-unit                */
} agc_kalc_setup_t;

/* KALCMAN3 .. CHECKMAX/ALTCALC/COFMAXGO: pure computation from the initial
 * (bcdu) and final (cphi = CPHI, CTHETA, CPSI) CDU patterns.              */
agc_status_t agc_kalc_compute(agc_interp_t *ip, const uint16_t bcdu[3],
                              const uint16_t cphi[3], agc_kalc_setup_t *out);

agc_status_t agc_kalc_set_rateindx(agc_sp_t idx);   /* RATEINDX 0,2,4,6   */
agc_sp_t     agc_kalc_rateindx(void);
bool         agc_kalc_active(void);                 /* ATTCADR != 0        */

/* GOMANUR: start KALCMAN3 for the calling job; on completion job wake_as
 * is woken (ATTSTALL-like); agc_kalc_outcome() gives good/bad.            */
agc_status_t agc_imu_gomanur(const agc_job_ctx_t *ctx, agc_job_id_t wake_as);
agc_imu_stall_result_t agc_kalc_outcome(void);

agc_status_t agc_job_kalcman3(agc_job_ctx_t *ctx);  /* KALCMAN3            */
agc_status_t agc_job_newdelhi(agc_job_ctx_t *ctx);  /* NEWDELHI            */
agc_status_t agc_task_updtcall(void);               /* UPDTCALL            */
agc_status_t agc_task_manustop(void);               /* MANUSTOP            */
agc_status_t agc_task_endmanu(void);                /* ENDMANU / GOODMANU  */
agc_status_t agc_kalc_stoprate(void);               /* STOPRATE            */
agc_status_t agc_kalc_zeroerror(void);              /* ZEROERROR (CM)      */

/* ======================================================================= */
/* R60 / R62 / VECPOINT / BALLANGS (R60_62.agc)                            */
/* ======================================================================= */

/* VECPOINT: point S/C axis scaxis (half-unit, S/C axes) along pointvsm
 * (half-unit, SM axes) from the present CDU attitude -> gimbal[] patterns. */
agc_status_t agc_imu_vecpoint(agc_interp_t *ip, agc_vec_t pointvsm,
                              agc_vec_t scaxis, uint16_t gimbal[3]);
/* VECPNT1 (LM): as VECPOINT but from the given initial CDU patterns.      */
agc_status_t agc_imu_vecpnt1(agc_interp_t *ip, const uint16_t cdu0[3],
                             agc_vec_t pointvsm, agc_vec_t scaxis, uint16_t gimbal[3]);
/* BALLANGS (LM): FDAI ball angles for CPHI.. -> fdai[] patterns.          */
agc_status_t agc_imu_ballangs(agc_interp_t *ip, const uint16_t gimbal[3], uint16_t fdai[3]);

agc_status_t agc_imu_set_scaxis(agc_vec_t v);       /* SCAXIS, half-unit   */
uint16_t     agc_erase_fdai(int axis);  /* FDAIX..Z (LM) from BALLANGS, CDU pattern */
agc_status_t agc_imu_set_pointvsm(agc_vec_t v);     /* POINTVSM, half-unit */

/* G+N,AUTO / ISITAUTO (LM R60_62.agc): +0 when the LM is in G&N and auto
 * stabilization, positive otherwise.                                     */
agc_sp_t     agc_imu_isitauto(void);
agc_sp_t     agc_imu_gn_auto(void);

/* R60CSM / R60LEM: called by a program job; on exit (ENDMANU1) job
 * ret_job is run in the same job (the AGC's BANKJUMP to TEMPR60).        */
agc_status_t agc_imu_r60(const agc_job_ctx_t *ctx, agc_job_id_t ret_job);
agc_status_t agc_job_r60_redomanc(agc_job_ctx_t *ctx); /* V50N18 PROCEED   */
agc_status_t agc_job_r60_enter(agc_job_ctx_t *ctx);    /* V50N18 ENTER     */
agc_status_t agc_job_r60_term(agc_job_ctx_t *ctx);     /* V34: R61TEST     */
agc_status_t agc_job_r60_endmanuv(agc_job_ctx_t *ctx); /* after GOMANUR    */
agc_status_t agc_job_relinus(agc_job_ctx_t *ctx);      /* RELINUS restart  */
agc_status_t agc_job_r62disp(agc_job_ctx_t *ctx);      /* R62DISP (V49)    */
agc_status_t agc_job_r62_gomove(agc_job_ctx_t *ctx);   /* V06N22 PROCEED   */
agc_status_t agc_job_r62_term(agc_job_ctx_t *ctx);     /* V34 / done: ENDEXT */

/* ======================================================================= */
/* CM prelaunch alignment pieces (IMU_CALIBRATION_AND_ALIGNMENT.agc,       */
/* src/csm/prelaunch_align.c)                                              */
/* ======================================================================= */

/* FINETIME: consistent read of HISCALAR/LOSCALAR; returns with interrupts
 * inhibited, as the AGC does.                                            */
agc_status_t agc_csm_finetime(uint16_t *hiscalar, uint16_t *loscalar);
/* POSN17C: XSM, YSM, ZSM for the compass position (Z down, X downrange)
 * from azimuth (revolutions from north, clockwise); rows half-unit.      */
agc_status_t agc_csm_prelaunch_posn17c(agc_interp_t *ip, agc_dp_t azimuth,
                                       agc_mat_t *xsm);
/* ERTHRVSE: ERVECTOR = (sin LAT, -cos LAT, 0) OMEG/MS; TMARK = now;
 * ERCOMP1 = 0.  latitude in revolutions.                                 */
agc_status_t agc_csm_prelaunch_erthrvse(agc_interp_t *ip, agc_dp_t latitude);
/* EARTHR: add the earth rate since TMARK, resolved by xsm, to ERCOMP1
 * and torque the gyros with it (PULSEIMU on the ERCOMP buffer).          */
agc_status_t agc_csm_prelaunch_earthr(agc_interp_t *ip, const agc_mat_t *xsm);
agc_vec_t    agc_csm_prelaunch_ervector(void);           /* ERVECTOR       */
/* Not translated (stubs returning AGC_STATUS_UNIMPLEMENTED).             */
agc_status_t agc_csm_prelaunch_imutest(agc_job_ctx_t *ctx);  /* IMUTEST    */
agc_status_t agc_csm_prelaunch_estims(agc_job_ctx_t *ctx);   /* ESTIMS     */
agc_status_t agc_csm_prelaunch_alwaysg(agc_job_ctx_t *ctx);  /* ALWAYSG    */
agc_status_t agc_csm_prelaunch_restaier(agc_job_ctx_t *ctx); /* RESTAIER   */
agc_status_t agc_csm_prelaunch_chkcomed(agc_job_ctx_t *ctx); /* CHKCOMED   */
agc_status_t agc_csm_prelaunch_tar_eref(agc_job_ctx_t *ctx); /* TAR/EREF   */
agc_status_t agc_csm_prelaunch_gcompver(agc_job_ctx_t *ctx); /* GCOMPVER   */
agc_status_t agc_csm_prelaunch_optdata(agc_job_ctx_t *ctx);  /* OPTDATA    */
agc_status_t agc_csm_prelaunch_latazchk(agc_job_ctx_t *ctx); /* LATAZCHK   */
agc_status_t agc_csm_prelaunch_azmthcg1(agc_job_ctx_t *ctx); /* AZMTHCG1   */
agc_status_t agc_csm_prelaunch_targdrve(agc_job_ctx_t *ctx); /* TARGDRVE   */

#endif /* AGC_IMU_H */
