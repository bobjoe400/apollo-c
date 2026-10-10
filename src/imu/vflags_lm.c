/*
 * vflags_lm.c -- Luminary099 ids of the imu lane's vehicle-specific
 * flags (see internal.h imu_vflag_t).  Includes only agc_flags_lm.h.
 */
#include "agc/core/assert.h"
#include "agc/core/flags_lm.h"
#include "internal.h"

/* imu_vflag_lm -- LM flag id, or 0 when the flag is CSM-only.            */
agc_flag_id_t imu_vflag_lm(imu_vflag_t f)
{
    static const agc_flag_id_t ids[IMU_VF_COUNT] = {
        0,                 /* CPHIFLAG: CSM only                        */
        AGC_FLAG_AUTR1FLG, /* FLAGWRD13 bit 1 (209D)                    */
        AGC_FLAG_AUTR2FLG, /* FLAGWRD13 bit 2 (208D)                    */
        AGC_FLAG_NEEDLFLG, /* FLAGWRD0 bit 4 (011D)                     */
        0,                 /* R60FLAG: CSM only                         */
        AGC_FLAG_CALCMAN3, /* FLAGWRD2 bit 3 (042D)                     */
        0                  /* TARG1FLG: CSM only                        */
    };
    if (!AGC_ASSERT((int) f >= 0 && f < IMU_VF_COUNT))
    {
        return 0;
    }
    (void) AGC_ASSERT(ids[IMU_VF_AUTR1FLG] != 0U);
    return ids[f];
}
