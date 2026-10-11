/**********************************************************************
 * FILE NAME: vflags_csm.c
 *
 * PURPOSE:
 *   Comanche055 ids of the imu lane's vehicle-specific flags; includes
 *   only agc/core/flags_csm.h.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc flag words
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_flag_test/set     flagword access (agc/core/flags_common.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   A flag absent from the vehicle reads false and cannot be set.
 *
 * NOTES:
 *   Flags whose flagword position differs between Comanche and
 *   Luminary, or that exist in one vehicle only (internal.h).
 *
 * ALGORITHM:
 *   Table lookup.
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/flags_csm.h"
#include "internal.h"

/**********************************************************************
 * AGC: none (Comanche055 flag table) -- CSM id of a vehicle-dependent
 * flag
 *
 * ARGUMENTS:
 *   f  imu_vflag_t  I    vehicle-dependent flag
 *
 * RETURNS: agc_flag_id_t -- CSM flag id, or 0 when the flag is LM-only
 **********************************************************************/
agc_flag_id_t imu_vflag_csm(imu_vflag_t f)
{
    static const agc_flag_id_t ids[IMU_VF_COUNT] = {
        AGC_FLAG_CPHIFLAG, /* FLAGWRD0 bit 15 (000D)                    */
        0,                 /* AUTR1FLG: LM only                         */
        0,                 /* AUTR2FLG: LM only                         */
        AGC_FLAG_NEEDLFLG, /* FLAGWRD0 bit 9 (006D)                     */
        AGC_FLAG_TRACKING_USE_MANEUVER, /* FLAGWRD5 bit 4 (086D)                     */
        0,                /* CALCMAN3: LM only                         */
        AGC_FLAG_TARG1FLG /* FLAGWRD1 bit 10 (020D)                    */
    };

    if (!AGC_ASSERT((int) f >= 0 && f < IMU_VF_COUNT))
    {
        return (0);
    }

    if (!AGC_ASSERT(ids[IMU_VF_CPHIFLAG] != 0U))
    {
        return (0);
    }

    return (ids[f]);
}
