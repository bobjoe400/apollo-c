/**********************************************************************
 * FILE NAME: setitctr_lm.c
 *
 * PURPOSE:
 *   SETITCTR, the Luminary099-only entry of ORBITAL_INTEGRATION.agc: the
 *   LAMBERT iteration count by AVEGFLAG (it needs the LM flag word).
 *
 * AGC SOURCE:
 *   Luminary099 ORBITAL_INTEGRATION.agc p. 1245 (SETITCTR); not in
 *   Comanche055
 *
 * EXTERNAL VARIABLES:
 *   Name      Type  I/O  Scale  Description
 *   --------  ----  ---  -----  ---------------------------------------
 *   AVEGFLAG  flag  I    -      AVERAGEG running
 *
 * EXTERNAL REFERENCES:
 *   Name               Description
 *   -----------------  -----------------------------------------
 *   agc_conic_lambert  LAMBERT (nav/conic.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none here; LAMBERT's status is returned
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/flags_lm.h"
#include "agc/nav/integration.h"

/**********************************************************************
 * SETITCTR (ORBITAL_INTEGRATION.agc L p. 1245) -- ITERCTR = 20D, or 5
 * while AVEGFLAG (AVERAGEG running), then LAMBERT
 *
 * FUNCTION NAME: agc_integ_setitctr
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  -------------------------------
 *   ip        agc_interp_t *       I/O  interpretive state (OVFIND)
 *   in        agc_lambert_in_t *   I    LAMBERT inputs; iterctr is
 *                                       replaced in a copy
 *   out       agc_lambert_out_t *  O    LAMBERT outputs
 *
 * RETURNS: agc_status_t -- LAMBERT's status
 **********************************************************************/
agc_status_t agc_integ_setitctr(agc_interp_t *ip, const agc_lambert_in_t *in, agc_lambert_out_t *out)
{
    agc_lambert_in_t li = *in; /* the caller's inputs with ITERCTR set */

    li.iterctr = agc_flag_test(AGC_FLAG_AVEGFLAG) ? (agc_sp_t) AGC_CONIC_LAMBERT_ITERCTR_AVEG : (agc_sp_t) AGC_CONIC_LAMBERT_ITERCTR; /* SSP 5; SSP 20D */

    return (agc_conic_lambert(ip, &li, out));
}
