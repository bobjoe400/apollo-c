/**********************************************************************
 * FILE NAME: vflags_lm.c
 *
 * PURPOSE:
 *   Luminary099 ids of the imu lane's vehicle-specific flags; includes
 *   only agc/core/flags_lm.h.
 *
 * AGC SOURCE:
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc flag words
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
#include "agc/core/assertion.h"
#include "agc/core/flags_lm.h"
#include "internal.h"

/**********************************************************************
 * AGC: none (Luminary099 flag table) -- LM id of a vehicle-dependent
 * flag
 *
 * ARGUMENTS:
 *   f  imu_vflag_t  I    vehicle-dependent flag
 *
 * RETURNS: agc_flag_id_t -- LM flag id, or 0 when the flag is CSM-only
 **********************************************************************/
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

    return (ids[f]);
}
