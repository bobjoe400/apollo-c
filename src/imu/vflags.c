/**********************************************************************
 * FILE NAME: vflags.c
 *
 * PURPOSE:
 *   Vehicle dispatch for the imu lane's vehicle-specific flags.
 *
 * AGC SOURCE:
 *   ERASABLE_ASSIGNMENTS.agc flag words (CM and LM)
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
#include "agc/core/state.h"
#include "internal.h"

/**********************************************************************
 * AGC: none (vehicle dispatch) -- the flag id for the vehicle being
 * flown
 *
 * ARGUMENTS:
 *   f  imu_vflag_t  I    vehicle-dependent flag
 *
 * RETURNS: agc_flag_id_t -- flag id, or 0 when the flag does not exist
 *          there
 **********************************************************************/
agc_flag_id_t imu_vflag(imu_vflag_t f)
{
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM); /* LM rope */

    return (lm ? imu_vflag_lm(f) : imu_vflag_csm(f)); /* by vehicle */
}

/**********************************************************************
 * AGC: none (BON/BOFF) -- test a vehicle-dependent flag
 *
 * ARGUMENTS:
 *   f  imu_vflag_t  I    vehicle-dependent flag
 *
 * RETURNS: bool -- true when the flag exists and is set
 **********************************************************************/
bool imu_vflag_test(imu_vflag_t f)
{
    agc_flag_id_t id = imu_vflag(f); /* id on this vehicle */

    if (!AGC_ASSERT(id == 0U || agc_flag_valid(id)))
    {
        return (false);
    }

    return ((id != 0U) && agc_flag_test(id));
}

/**********************************************************************
 * AGC: none (UPFLAG / SET) -- set a vehicle-dependent flag
 *
 * ARGUMENTS:
 *   f  imu_vflag_t  I    vehicle-dependent flag (must exist on this
 *                        vehicle)
 **********************************************************************/
void imu_vflag_set(imu_vflag_t f)
{
    agc_flag_id_t id = imu_vflag(f); /* id on this vehicle */

    if (!AGC_ASSERT(id != 0U))
    {
        return;
    }

    if (!AGC_ASSERT(agc_flag_valid(id)))
    {
        return;
    }

    agc_flag_set(id);
}
