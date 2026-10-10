/*
 * setitctr_lm.c -- Luminary099-only part of ORBITAL_INTEGRATION.agc
 * (p. 1245): SETITCTR, which needs the LM position of AVEGFLAG.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_lm.h"
#include "agc/nav/integration.h"

/* SETITCTR: ITERCTR = 20D, or 5 while AVEGFLAG (AVERAGEG running), then
 * LAMBERT (agent nav-conic).  The caller's input is copied with iterctr
 * replaced.                                                             */
agc_status_t agc_integ_setitctr(agc_interp_t *ip, const agc_lambert_in_t *in,
                                agc_lambert_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(in != NULL) ||
        !AGC_ASSERT(out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_lambert_in_t li = *in;

    li.iterctr = agc_flag_test(AGC_FLAG_AVEGFLAG) ?
                 (agc_sp_t) AGC_CONIC_LAMBERT_ITERCTR_AVEG              /* SSP 5   */
                 :
                 (agc_sp_t) AGC_CONIC_LAMBERT_ITERCTR;              /* SSP 20D */

    return agc_conic_lambert(ip, &li, out);
}
