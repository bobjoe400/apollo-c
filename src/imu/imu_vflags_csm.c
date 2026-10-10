/*
 * imu_vflags_csm.c -- Comanche055 ids of the imu lane's vehicle-specific
 * flags (see imu_internal.h imu_vflag_t).  Includes only agc_flags_csm.h.
 */
#include "agc/agc_assert.h"
#include "agc/agc_flags_csm.h"
#include "imu_internal.h"

/* imu_vflag_csm -- CSM flag id, or 0 when the flag is LM-only.           */
agc_flag_id_t imu_vflag_csm(imu_vflag_t f)
{
    static const agc_flag_id_t ids[IMU_VF_COUNT] = {
        AGC_FLAG_CPHIFLAG,   /* FLAGWRD0 bit 15 (000D)                    */
        0,                   /* AUTR1FLG: LM only                         */
        0,                   /* AUTR2FLG: LM only                         */
        AGC_FLAG_NEEDLFLG,   /* FLAGWRD0 bit 9 (006D)                     */
        AGC_FLAG_ATTITUDE_MANEUVER_FLAG,    /* FLAGWRD5 bit 4 (086D)                     */
        0,                   /* CALCMAN3: LM only                         */
        AGC_FLAG_TARG1FLG    /* FLAGWRD1 bit 10 (020D)                    */
    };
    if (!AGC_ASSERT((int)f >= 0 && f < IMU_VF_COUNT)) {
        return 0;
    }
    (void)AGC_ASSERT(ids[IMU_VF_CPHIFLAG] != 0U);
    return ids[f];
}
