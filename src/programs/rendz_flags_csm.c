/*
 * rendz_flags_csm.c -- Comanche055 ids of the rendezvous flags whose
 * position differs between the vehicles (see rendz_internal.h rz_vflag).
 * Source: Comanche055/ERASABLE_ASSIGNMENTS.agc via agc_flags_csm.h.
 */
#include "agc/agc_assert.h"
#include "agc/agc_flags_csm.h"
#include "rendz_internal.h"

/* rz_vflag_csm -- CSM flag id for a vehicle-dependent rendezvous flag. */
agc_flag_id_t rz_vflag_csm(rz_vflag_t f)
{
    static const agc_flag_id_t ids[RZ_VF_COUNT] = {
        AGC_FLAG_ITSWICH,    /* 106D */
        AGC_FLAG_AVEGFLAG,   /* 029D */
        AGC_FLAG_V82EMFLG,   /* 137D */
        AGC_FLAG_P21FLAG     /* 033D */
    };
    if (!AGC_ASSERT((int)f >= 0 && f < RZ_VF_COUNT)) {
        return ids[0];
    }
    (void)AGC_ASSERT(ids[f] != 0U);
    return ids[f];
}
