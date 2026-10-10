/*
 * rendz_flags_lm.c -- Luminary099 ids of the rendezvous flags whose
 * position differs between the vehicles (see rendz_internal.h rz_vflag).
 * Source: Luminary099/FLAGWORD_ASSIGNMENTS.agc via agc_flags_lm.h.
 */
#include "agc/agc_assert.h"
#include "agc/agc_flags_lm.h"
#include "rendz_internal.h"

/* rz_vflag_lm -- LM flag id for a vehicle-dependent rendezvous flag. */
agc_flag_id_t rz_vflag_lm(rz_vflag_t f)
{
    static const agc_flag_id_t ids[RZ_VF_COUNT] = {
        AGC_FLAG_ITSWICH,    /* 105D */
        AGC_FLAG_AVEGFLAG,   /* 115D */
        AGC_FLAG_V82EMFLG,   /* 118D */
        AGC_FLAG_P21FLAG     /* 004D */
    };
    if (!AGC_ASSERT((int)f >= 0 && f < RZ_VF_COUNT)) {
        return ids[0];
    }
    (void)AGC_ASSERT(ids[f] != 0U);
    return ids[f];
}
