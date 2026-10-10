/**********************************************************************
 * FILE NAME: function_prolog.c
 *
 * PURPOSE:
 *   Template for function prologs (guide 5.1) adapted to apollo-c. The
 *   functions below are an illustration, not flight code.
 *
 * AGC SOURCE:
 *   none (template)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/status.h"
#include "agc/core/word.h"

/* Normally in the module header; here so the template compiles alone. */
agc_status_t agc_template_limit_magnitude(agc_dp_t value, agc_dp_t limit,
                                          agc_dp_t *out);

/**********************************************************************
 * AGC: none (helper for LIMIT) -- true when |value| exceeds limit
 *
 * FUNCTION NAME: magnitude_exceeds
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   value     agc_dp_t    I    quantity under test, 2^28 cs
 *   limit     agc_dp_t    I    magnitude limit, 2^28 cs, > 0
 *
 * RETURNS: bool -- true when |value| > limit (predicate name, 5.1)
 **********************************************************************/
static bool magnitude_exceeds(agc_dp_t value, agc_dp_t limit)
{
    bool exceeds = false;  /* result of the comparison */

    (void) AGC_ASSERT(limit > 0);
    (void) AGC_ASSERT(value >= AGC_DP_MIN);
    if ((value > limit) || (value < -limit))
    {
        exceeds = true;
    }
    return (exceeds);
}

/**********************************************************************
 * LIMIT (EXAMPLE.agc, pp. 100-101, label LIMIT) -- clamp a DP quantity
 *
 * FUNCTION NAME: agc_template_limit_magnitude
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   value     agc_dp_t    I    quantity to limit, 2^28 cs
 *   limit     agc_dp_t    I    magnitude limit, 2^28 cs, > 0
 *   out       agc_dp_t *  O    value limited to [-limit, +limit],
 *                              2^28 cs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when out
 *          is NULL or limit is not positive
 **********************************************************************/
agc_status_t agc_template_limit_magnitude(agc_dp_t value, agc_dp_t limit,
                                          agc_dp_t *out)
{
    agc_status_t status = AGC_STATUS_OK;  /* returned to the caller      */
    agc_dp_t     limited = value;         /* value after LIMIT, 2^28 cs  */

    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(limit > 0))
    {
        status = AGC_STATUS_BAD_ARG;
    }
    else
    {
        /*
         * Clamp to the limit, keeping the sign of the input, as the AGC
         * does before storing the result.
         */
        if (magnitude_exceeds(value, limit))
        {
            limited = (value > 0) ? limit : -limit;  /* sign of input */
        }
        *out = limited;
    }
    return (status);
}
