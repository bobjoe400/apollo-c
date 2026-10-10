/*
 * flags_lm.c -- Luminary099 ids of the rendezvous flags whose
 * position differs between the vehicles, or which exist in one vehicle
 * only (id 0 = absent), see internal.h rz_vflag.
 * Source: Luminary099/FLAGWORD_ASSIGNMENTS.agc via agc_flags_lm.h.
 */
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "internal.h"

/* rz_vflag_lm -- LM flag id for a vehicle-dependent rendezvous flag. */
agc_flag_id_t rz_vflag_lm(rz_vflag_t f)
{
    static const agc_flag_id_t ids[RZ_VF_COUNT] = {
        AGC_FLAG_ITSWICH /* ITSWICH */,
        AGC_FLAG_AVEGFLAG /* AVEGFLAG */,
        AGC_FLAG_ORBIT_DISPLAY_MOON /* V82EMFLG */,
        AGC_FLAG_GROUND_TRACK_BASE_VECTORS /* P21FLAG */,
        AGC_FLAG_S32_1F1 /* S32_1F1 */,
        AGC_FLAG_S32_1F2 /* S32_1F2 */,
        AGC_FLAG_S32_1F3A /* S32_1F3A */,
        AGC_FLAG_S32_1F3B /* S32_1F3B */,
        AGC_FLAG_NTARGFLG /* NTARGFLG */,
        0U /* COMPUTER */,
        0U /* AMOONFLG */,
        0U /* R31FLAG */,
        AGC_FLAG_SERVICER_RUNNING /* V37FLAG */,
        AGC_FLAG_MUNFLAG /* MUNFLAG */,
        0U /* TIMRFLAG */
    };

    if (!AGC_ASSERT((int) f >= 0 && f < RZ_VF_COUNT))
    {
        return (0U);
    }

    if (!AGC_ASSERT(ids[f] == 0U || agc_flag_bit(ids[f]) >= 1))
    {
        return (0U);
    }

    return (ids[f]);
}
