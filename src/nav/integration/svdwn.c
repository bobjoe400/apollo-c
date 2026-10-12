/**********************************************************************
 * FILE NAME: svdwn.c
 *
 * PURPOSE:
 *   SVDWN1 / SVDWN2: the downlink copy of a state vector (RN, VN,
 *   PIPTIME for this vehicle; R-OTHER, V-OTHER for the other one).
 *   Owner: prog-align; kept in src/nav/ because the integration package
 *   (ATOPCSM / ATOPLEM) is its caller, so nav does not depend on
 *   src/programs/.
 *
 * AGC SOURCE:
 *   Comanche055 CSM_GEOMETRY.agc pp. 289-290 (SVDWN1, SVDWN2)
 *   Luminary099 LEM_GEOMETRY.agc pp. 320-321 (same body, names swapped)
 *
 * EXTERNAL VARIABLES:
 *   Name      Type       I/O  Scale        Description
 *   --------  ---------  ---  -----------  ----------------------------
 *   RN        agc_vec_t  O    2^29 m       this vehicle's position
 *   VN        agc_vec_t  O    2^7 m/cs     this vehicle's velocity
 *   PIPTIME   agc_dp_t   O    2^28 cs      time of RN, VN
 *   AVEMIDSW  flag       I    -            AVETOMID: keep RN, VN
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  -----------------------------------------
 *   agc_vec_shr / vec_add  VSL* (right) / VAD (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles.  The two programs exchange the names: SVDWN1 is the
 *   CSM state in both (this vehicle in Comanche, the other one in
 *   Luminary), so "this vehicle" is SVDWN1 in the CM and SVDWN2 in the
 *   LM.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/nav/integration.h"

#define SVDWN_R_SHIFT 7 /* TDELTAV E 2^22 / M 2^18 -> RCV scale        */
#define SVDWN_V_SHIFT 4 /* TNUV    E 2^3  / M 2^-1 -> VCV scale        */

/* R-OTHER, V-OTHER (B(6)PRM, downlink only), m B29 and m/cs B7. */
static agc_vec_t r_other; /* R-OTHER, m 2^29    */
static agc_vec_t v_other; /* V-OTHER, m/cs 2^7  */

/**********************************************************************
 * SVDWN1 / SVDWN2 (CSM_GEOMETRY.agc C pp. 289-290, LEM_GEOMETRY.agc
 * L pp. 320-321) -- downlink copy of a state vector:
 *   R = (TDELTAV SR(7+X2) + RCV) SR X2     -> m 2^29
 *   V = (TNUV    SR(4+X2) + VCV) SR X2     -> m/cs 2^7
 * This vehicle: RN, VN and PIPTIME = TET, unless AVEMIDSW is set
 * (AVETOMID doing the W-matrix integration: BOF AVEMIDSW +1, RVQ).  The
 * other vehicle: R-OTHER, V-OTHER.  VSL* by a negative count is a
 * rounding right shift, as agc_vec_shr is.  X2 = AGC_INTEG_X2_ORIGEX
 * (after an A-PCHK origin change) shifts everything out: zero vectors.
 *
 * FUNCTION NAME: agc_integ_svdwn
 *
 * ARGUMENTS:
 *   Argument      Type            I/O  Description (scale)
 *   ------------  --------------  ---  -------------------------------
 *   ip            agc_interp_t *  I/O  interpretive state (OVFIND)
 *   this_vehicle  bool            I    RN/VN/PIPTIME, else R/V-OTHER
 *   x2            int             I    X2: 0 earth (2^29), 2 moon
 *                                      (2^27), or AGC_INTEG_X2_ORIGEX
 *   rcv           agc_vec_t       I    RCV, m 2^29 (moon 2^27)
 *   vcv           agc_vec_t       I    VCV, m/cs 2^7 (moon 2^5)
 *   tdeltav       agc_vec_t       I    TDELTAV, m 2^22 (moon 2^18)
 *   tnuv          agc_vec_t       I    TNUV, m/cs 2^3 (moon 2^-1)
 *   tet           agc_dp_t        I    TET, cs 2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (it cannot fail)
 **********************************************************************/
agc_status_t agc_integ_svdwn(agc_interp_t *ip, bool this_vehicle, int x2, agc_vec_t rcv, agc_vec_t vcv, agc_vec_t tdeltav, agc_vec_t tnuv, agc_dp_t tet)
{
    if (!(this_vehicle && agc_flag_test(AGC_FLAG_AVEMIDSW))) /* BOF AVEMIDSW; RVQ */
    {
        agc_vec_t r = agc_vec_zero(); /* X2 = ORIGEX: shifted out, m B29    */
        agc_vec_t v = agc_vec_zero(); /* X2 = ORIGEX: shifted out, m/cs B7  */

        if (x2 != AGC_INTEG_X2_ORIGEX)
        {
            r = agc_vec_shr(agc_vec_add(ip, agc_vec_shr(tdeltav, SVDWN_R_SHIFT + x2), rcv), x2);
            v = agc_vec_shr(agc_vec_add(ip, agc_vec_shr(tnuv, SVDWN_V_SHIFT + x2), vcv), x2);
        }

        if (this_vehicle)
        {
            agc_erase_set_rn(r);        /* STOVL RN          */
            agc_erase_set_vn(v);        /* STODL VN          */
            agc_erase_set_piptime(tet); /* STORE PIPTIME     */
        }
        else
        {
            r_other = r; /* STOVL R-OTHER     */
            v_other = v; /* STORE V-OTHER     */
        }
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * R-OTHER (CSM_GEOMETRY.agc C p. 290) -- downlink: the other vehicle's
 * position
 *
 * FUNCTION NAME: agc_integ_r_other
 *
 * RETURNS: agc_vec_t -- R-OTHER, m 2^29
 **********************************************************************/
agc_vec_t agc_integ_r_other(void)
{
    return (r_other);
}

/**********************************************************************
 * V-OTHER (CSM_GEOMETRY.agc C p. 290) -- downlink: the other vehicle's
 * velocity
 *
 * FUNCTION NAME: agc_integ_v_other
 *
 * RETURNS: agc_vec_t -- V-OTHER, m/cs 2^7
 **********************************************************************/
agc_vec_t agc_integ_v_other(void)
{
    return (v_other);
}
