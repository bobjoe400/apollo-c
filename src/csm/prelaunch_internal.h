/**********************************************************************
 * FILE NAME: prelaunch_internal.h
 *
 * PURPOSE:
 *   Declarations shared by the three files of the Comanche prelaunch
 *   alignment (src/csm/prelaunch_align.c, prelaunch_gyrocompass.c,
 *   prelaunch_verify.c): the AINLA erasable overlay and the subroutines
 *   one file calls in another.  Not part of the public API.
 *
 * AGC SOURCE:
 *   Comanche055 IMU_CALIBRATION_AND_ALIGNMENT.agc pp. 423-455;
 *   ERASABLE_ASSIGNMENTS.agc pp. 98-100 (AINLA overlay)
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   Name          Type               Scale   Description
 *   ------------  -----------------  ------  -------------------------
 *   none (the overlay is reached through prelaunch_ram(); each member
 *   carries its scale)
 **********************************************************************/
#ifndef CSM_PRELAUNCH_INTERNAL_H
#define CSM_PRELAUNCH_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"

/* THETAX1 .. INTVEC1 +5: the words STOREDTA copies to RESTARPT.          */
typedef struct prelaunch_filter
{
    agc_vec_t thetax1;     /* THETAX1: ERCOMP1 at LASTGTS, rev 2^0      */
    agc_vec_t thetan1;     /* THETAN1: compass/erection torque, rev 2^0 */
    agc_dp_t  fildelv1[2]; /* FILDELV1 +0,+4: filtered Y,Z PIPA B14     */
    agc_dp_t  intvec1[2];  /* INTVEC1 +0,+4: sum of FILDELV1, B14       */
} prelaunch_filter_t;

/* The AINLA overlay ("OPTIMUM CALIB. AND ALIGNMENT", EBANK 5) and the
 * other prelaunch erasables (LATITUDE, AZIMUTH, TAZEL1, ...).            */
typedef struct prelaunch_ram
{
    prelaunch_filter_t filt;      /* THETAX1 .. INTVEC1                   */
    prelaunch_filter_t restarpt;  /* RESTARPT: STOREDTA copy of filt      */
    agc_dp_t           latitude;  /* LATITUDE of the pad, rev 2^0         */
    agc_dp_t           azimuth;   /* AZIMUTH of ZNB from north, rev 2^0   */
    agc_dp_t           launchaz;  /* LAUNCHAZ = LUNCHAZ1, rev 2^0         */
    agc_dp_t           newazmth;  /* NEWAZMTH = NEWAZ1, rev 2^0           */
    agc_dp_t           oldazmth;  /* OLDAZMTH, rev 2^0                    */
    agc_dp_t           perfdlay;  /* PERFDLAY: delay to ESTIMS, cs 2^28   */
    agc_dp_t           alk;       /* ALK: TMARK kept by LASTGTS, cs 2^28  */
    agc_dp_t           dpipa[2];  /* DPIPAY, DPIPAZ: level PIPA, B14      */
    agc_dp_t           wango;     /* WANGO: sin LAT (perf test), 2^0      */
    agc_dp_t           wangi;     /* WANGI: -cos LAT (perf test), 2^0     */
    agc_vec_t          losvec;    /* LOSVEC: marked target 1, B1         */
    agc_sp_t           geocomp1;  /* GEOCOMP1: 1 compass, 0 IMU test, B0  */
    agc_sp_t           lengthot;  /* LENGTHOT: passes left, count B0      */
    agc_sp_t           onesecxt1; /* 1SECXT1: sample period, cs B0        */
    agc_sp_t           gtswtlt1;  /* GTSWTLT1: TIME1 at the sample, cs B0 */
    agc_sp_t           erectim1;  /* ERECTIM1: erection cycles left, B0   */
    agc_sp_t           geosave1;  /* GEOSAVE1: restart copy, count B0     */
    agc_sp_t           premtrxc;  /* PREMTRXC = PREMTRX1, count B0        */
    agc_sp_t           altim;     /* ALTIM: gain counter (perf test), B0  */
    agc_sp_t           altims;    /* ALTIMS: ALTIM - 1 or 0, B0           */
    agc_sp_t           alx1s;     /* ALX1S: gain index (perf test), B0    */
    agc_sp_t           cmpx1;     /* CMPX1: index (perf test), B0         */
    agc_sp_t           ndxctr;    /* NDXCTR: 1 if gimbal lock, B0         */
    agc_sp_t           run;       /* RUN: 0 target 1, 2 target 2, B0      */
    agc_sp_t           targ;      /* TARG1/2: 0 target 1, 6 target 2, B0  */
    uint16_t           tazel[4];  /* TAZEL1: AZ1 (CDU), EL1 (90 deg = 1),
                                   * AZ2, EL2; raw 15-bit words          */
} prelaunch_ram_t;

#define PRELAUNCH_ALARM_DRIFT  ((agc_alarm_code_t) 01600) /* overflow  */
#define PRELAUNCH_ALARM_TORQUE ((agc_alarm_code_t) 01601) /* bad IMU   */
#define PRELAUNCH_ALARM_OPTICS ((agc_alarm_code_t) 01602) /* bad optics */
#define PRELAUNCH_MM_INIT      ((agc_sp_t) 1)             /* P01 initialization     */
#define PRELAUNCH_MM_COMPASS   ((agc_sp_t) 2)             /* P02 gyrocompassing     */
#define PRELAUNCH_MM_VERIFY    ((agc_sp_t) 3)             /* P03 optical verify     */
#define PRELAUNCH_MM_IMU_TEST  ((agc_sp_t) 7)             /* P07 systems test       */
#define PRELAUNCH_PRIO_FILTER  ((agc_prio_t) 022)         /* PRIO22 (ALFLT)     */

prelaunch_ram_t *prelaunch_ram(void);

/* NBPOSPL (p.423-424): XNB, YNB, ZNB from AZIMUTH, half-unit.          */
agc_status_t prelaunch_nbpospl(agc_interp_t *ip);
/* IMUSTLLG after a stall: *done when good; SOMERR2 when bad.           */
agc_status_t prelaunch_imustall(agc_job_id_t wake_as, bool *done);
agc_status_t prelaunch_stall_outcome(bool *good);
/* EARTHR* (p.430): EARTHR then IMUSTLLG.  busy_wake: the job to run
 * again when the gyros are busy (GYROBUSY); stall_wake: the job woken
 * when the torquing ends.                                              */
agc_status_t prelaunch_earthr_star(agc_interp_t *ip, agc_job_id_t busy_wake,
                                   agc_job_id_t stall_wake, bool *done);
agc_status_t prelaunch_set_tmark(agc_dp_t t);  /* TMARK, cs 2^28   */
agc_dp_t     prelaunch_tmark(void);            /* TMARK, cs 2^28   */
/* SOMERR2 (p.442): alarm 1601, group 5 off, ENDTEST1.                  */
agc_status_t prelaunch_somerr2(void);
/* ENDTEST1 (p.427): IMUSE down, MODREG -0, MKRELEAS, ENDEXT.           */
agc_status_t prelaunch_endtest1(void);
/* POSN17C on NEWAZ1 into the alignment XSM.                            */
agc_status_t prelaunch_posn17c(agc_interp_t *ip);
/* ESTIMS (p.434) in the calling job.                                   */
agc_status_t prelaunch_estims(agc_job_ctx_t *ctx);
/* SETNBPOS +1 (p.449): MKRELEAS, then OPTDATA in the calling job.      */
agc_status_t prelaunch_verify_start(agc_job_ctx_t *ctx);

#endif /* CSM_PRELAUNCH_INTERNAL_H */
