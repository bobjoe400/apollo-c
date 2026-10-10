/*
 * svdwn.c -- SVDWN1 / SVDWN2: the downlink copy of a state vector, from
 * CSM_GEOMETRY.agc (Comanche055 p. 289-290) and LEM_GEOMETRY.agc
 * (Luminary099 p. 320-321).  Owner: prog-align; kept in src/nav/ because
 * the integration package (ATOPCSM / ATOPLEM) is its caller, so nav does
 * not depend on src/programs/.  Declared in include/agc/programs/alignment.h.
 *
 * The two programs exchange the names: SVDWN1 is the CSM state in both
 * (this vehicle in Comanche, the other one in Luminary), so "this vehicle"
 * is SVDWN1 in the CM and SVDWN2 in the LM.  The body is the same.
 */
#include <stdbool.h>
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/core/flags_common.h"
#include "agc/programs/alignment.h"

#define SVDWN_R_SHIFT 7     /* TDELTAV E 2^22 / M 2^18 -> RCV scale        */
#define SVDWN_V_SHIFT 4     /* TNUV    E 2^3  / M 2^-1 -> VCV scale        */

/* R-OTHER, V-OTHER (B(6)PRM, downlink only), m B29 and m/cs B7. */
static agc_vec_t r_other;
static agc_vec_t v_other;

/*
 * SVDWN1 (CM) / SVDWN2 (LM) for this vehicle, SVDWN2 (CM) / SVDWN1 (LM)
 * for the other one:
 *   R = (TDELTAV SR(7+X2) + RCV) SR X2     -> m B29
 *   V = (TNUV    SR(4+X2) + VCV) SR X2     -> m/cs B7
 * x2 = X2: 0 for an earth-centred state (B29/B7 inputs), 2 for the moon
 * (B27/B5).  This vehicle: RN, VN and PIPTIME = TET, unless AVEMIDSW is
 * set (AVETOMID doing the W-matrix integration: BOF AVEMIDSW +1, RVQ).
 * The other vehicle: R-OTHER, V-OTHER.  VSL* by a negative count is a
 * rounding right shift, as agc_vec_shr is.
 */
agc_status_t agc_align_svdwn(agc_interp_t *ip, bool this_vehicle, int x2,
                             agc_vec_t rcv, agc_vec_t vcv, agc_vec_t tdeltav,
                             agc_vec_t tnuv, agc_dp_t tet)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT((x2 == 0) || (x2 == 2))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (this_vehicle && agc_flag_test(AGC_FLAG_AVEMIDSW)) {
        return AGC_STATUS_OK;                         /* BOF AVEMIDSW; RVQ */
    }
    const agc_vec_t r = agc_vec_shr(agc_vec_add(ip, agc_vec_shr(tdeltav,
                                                SVDWN_R_SHIFT + x2), rcv), x2);
    const agc_vec_t v = agc_vec_shr(agc_vec_add(ip, agc_vec_shr(tnuv,
                                                SVDWN_V_SHIFT + x2), vcv), x2);
    if (this_vehicle) {
        agc_erase_set_rn(r);                          /* STOVL RN          */
        agc_erase_set_vn(v);                          /* STODL VN          */
        agc_erase_set_piptime(tet);                   /* STORE PIPTIME     */
        return AGC_STATUS_OK;
    }
    r_other = r;                                      /* STOVL R-OTHER     */
    v_other = v;                                      /* STORE V-OTHER     */
    return AGC_STATUS_OK;
}

/* R-OTHER (downlink): other vehicle position, m B29. */
agc_vec_t agc_align_r_other(void)
{
    (void)AGC_ASSERT(r_other.c[0] >= AGC_DP_MIN);
    (void)AGC_ASSERT(r_other.c[0] <= AGC_DP_MAX);
    return r_other;
}

/* V-OTHER (downlink): other vehicle velocity, m/cs B7. */
agc_vec_t agc_align_v_other(void)
{
    (void)AGC_ASSERT(v_other.c[0] >= AGC_DP_MIN);
    (void)AGC_ASSERT(v_other.c[0] <= AGC_DP_MAX);
    return v_other;
}
