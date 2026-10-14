/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Declarations shared between the imu lane's own files (the .c files
 *   in src/imu): the vehicle-dependent flag map, the IMODES30/IMODES33
 *   and ISS lamp bits, and the mode-switching helpers gyro.c uses.  Not
 *   part of the public API.
 *
 * AGC SOURCE:
 *   IMU_MODE_SWITCHING_ROUTINES.agc pp. 1420-1442 (bit assignments,
 *   CAGETEST, CAGETSTJ, MODEEXIT); ERASABLE_ASSIGNMENTS.agc (IMODES30,
 *   IMODES33, DSPTAB)
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   Name          Type      Scale          Description
 *   ------------  --------  -------------  -------------------------
 *   none (the IMODES words are reached through accessors)
 **********************************************************************/
#ifndef IMU_INTERNAL_H
#define IMU_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/imu/imu.h"

/* Flags whose flagword position differs between Comanche and Luminary, or
 * that exist in one vehicle only.  imu_vflag() returns the id for the
 * current vehicle (vflags_csm.c / vflags_lm.c each include only
 * their own flag header), or 0 when the flag does not exist there.        */
typedef enum imu_vflag
{
    IMU_VF_CPHIFLAG = 0,          /* CSM FLAGWRD0 bit 15: CALCGA output to CPHIX */
    IMU_VF_AUTR1FLG,              /* LM FLAGWRD13 bit 1: KALCMANU rate, low bit  */
    IMU_VF_AUTR2FLG,              /* LM FLAGWRD13 bit 2: KALCMANU rate, high bit */
    IMU_VF_NEEDLFLG,              /* CSM FLAGWRD0 bit 9 / LM FLAGWRD0 bit 4      */
    IMU_VF_TRACKING_USE_MANEUVER, /* CSM FLAGWRD5 bit 4                          */
    IMU_VF_CALCMAN3,              /* LM FLAGWRD2 bit 3: KALCMANU gimbal-lock skip */
    IMU_VF_TARG1FLG,              /* CSM FLAGWRD1 bit 10: R52 target flag        */
    IMU_VF_COUNT
} imu_vflag_t;

agc_flag_id_t imu_vflag(imu_vflag_t f);
agc_flag_id_t imu_vflag_csm(imu_vflag_t f);
agc_flag_id_t imu_vflag_lm(imu_vflag_t f);
bool          imu_vflag_test(imu_vflag_t f); /* false if not in vehicle */
void          imu_vflag_set(imu_vflag_t f);

/* IMODES30 bits used by mode switching (IMU_MODE_SWITCHING_ROUTINES).    */
#define IM30_PIPNOTUSED  0x0001u /* BIT1: program not using PIPAs (PIPFREE) */
#define IM30_ICDUFAILINH 0x0004u /* BIT3: ICDU fail inhibit                 */
#define IM30_IMUFAILINH  0x0008u /* BIT4: IMU fail inhibit                  */
#define IM30_PIPFAILINH  0x0010u /* BIT5: PIPA fail inhibit (turn-on)       */
#define IM30_CAGE        0x0020u /* BIT6: IMU being caged                   */
#define IM30_NOTOPER     0x0100u /* BIT9: 1 = IMU operate NOT present       */
#define IM30_PIPOK       0x0200u /* BIT10: 0 = PIPA fail present            */
#define IM30_IMUOK       0x1000u /* BIT13: 0 = IMU fail present             */

/* IMODES33 bits.                                                          */
#define IM33_DAPDIS  0x0030u /* SUPER011, BITS5-6: DAP auto/hold off    */
#define IM33_DAPDIS6 0x0020u /* BIT6: DAP auto/hold off (coarse align)  */
#define IM33_PIPOK   0x1000u /* BIT13: 0 = PIPA fail present            */

/* DSPTAB +11D lamp bits.                                                  */
#define ISS_LAMP_NO_ATT      0x0008u /* BIT4: NO ATT lamp                       */
#define ISS_LAMP_GIMBAL_LOCK 0x0020u /* BIT6: gimbal lock lamp                  */
#define ISS_LAMPS_CHANGED    0x4000u /* BIT15: word changed / to be sent        */

/* Mode-switching helpers shared by mode.c and gyro.c.             */
bool         imu_caged(void);    /* IMODES30 BIT6                     */
agc_status_t imu_cagetstj(void); /* CAGETSTJ: OK, or BUSY if caged    */
void         imu_modeexit(void); /* MODEEXIT: RELINT                  */

/* One gyro pulse = 2^-21 revolution = 128 LSB of an agc_dp_t (2^28/rev). */
#define IMU_DP_PER_GYRO_PULSE 128 /* 2^28 / 2^21: DP LSB per pulse */

/* ---- moved from the public headers (used only in imu/) ---- */

/* agc/imu/imu.h */
void         agc_imu_pipa_fail_enable(void); /* PIPUSE1        */
bool         agc_kalc_active(void);          /* ATTCADR != 0        */
agc_status_t agc_imu_gomanur(const agc_job_ctx_t *ctx, agc_job_id_t wake_as);
agc_sp_t     agc_imu_isitauto(void);

#endif /* IMU_INTERNAL_H */
