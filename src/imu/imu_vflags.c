/*
 * imu_vflags.c -- vehicle dispatch for the imu lane's vehicle-specific
 * flags (imu_internal.h).  A flag absent from the vehicle reads false and
 * cannot be set.
 */
#include "agc/agc_assert.h"
#include "agc/agc_state.h"
#include "imu_internal.h"

agc_flag_id_t imu_vflag(imu_vflag_t f)
{
    if (!AGC_ASSERT((int)f >= 0 && f < IMU_VF_COUNT)) {
        return 0;
    }
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM);
    (void)AGC_ASSERT(lm || agc_state_vehicle() == AGC_VEHICLE_CSM);
    return lm ? imu_vflag_lm(f) : imu_vflag_csm(f);
}

bool imu_vflag_test(imu_vflag_t f)
{
    agc_flag_id_t id = imu_vflag(f);
    (void)AGC_ASSERT((int)f >= 0 && f < IMU_VF_COUNT);
    (void)AGC_ASSERT(id == 0U || agc_flag_valid(id));
    return (id != 0U) && agc_flag_test(id);
}

agc_status_t imu_vflag_set(imu_vflag_t f)
{
    agc_flag_id_t id = imu_vflag(f);
    if (!AGC_ASSERT(id != 0U)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_flag_valid(id));
    return agc_flag_set(id);
}

agc_status_t imu_vflag_clear(imu_vflag_t f)
{
    agc_flag_id_t id = imu_vflag(f);
    if (!AGC_ASSERT(id != 0U)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_flag_valid(id));
    return agc_flag_clear(id);
}
