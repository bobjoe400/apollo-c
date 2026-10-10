/*
 * flags_csm.c -- the Comanche055-only flags touched by the
 * integration package (they exist only in agc_flags_csm.h):
 *   V96ONFLG  cleared by STATINT1 (INTEGRATION_INITIALIZATION p. 1313)
 *   AMOONFLG  read by SETCOAST (p. 1327), set by MIDTOAV1/2 (p. 1330):
 *             the sphere of influence of the RN, VN state of AVERAGEG.
 */
#include "agc/core/assert.h"
#include "agc/core/flags_csm.h"
#include "internal.h"

void agc_integ_csm_resume_idle_integration(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return;
    }
    agc_integ_flag(AGC_FLAG_IDLE_INTEGRATION_INHIBITED, false);
    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_IDLE_INTEGRATION_INHIBITED)))
    {
        return;
    }
}

bool agc_integ_csm_amoonflg(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return false;
    }
    if (!AGC_ASSERT(agc_flag_valid(AGC_FLAG_AMOONFLG)))
    {
        return false;
    }
    return agc_flag_test(AGC_FLAG_AMOONFLG);
}

void agc_integ_csm_set_amoonflg(bool on)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return;
    }
    agc_integ_flag(AGC_FLAG_AMOONFLG, on);
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_AMOONFLG) == on))
    {
        return;
    }
}
