/**********************************************************************
 * FILE NAME: assert.c
 *
 * PURPOSE:
 *   Host-side reporting hook for AGC_ASSERT (rule 5): a failed assertion
 *   is printed when AGC_HOST_DEBUG is defined (test builds).
 *
 * AGC SOURCE:
 *   none (host-side infrastructure; the AGC has no assertions)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   fprintf  the diagnostic line (standard library, stderr)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none (it only reports; the caller takes its own error path)
 **********************************************************************/
#include <stdio.h>

#include "agc/core/assertion.h"

/**********************************************************************
 * AGC: none (host-side hook of AGC_ASSERT) -- print a failed assertion
 *   to stderr
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   file      const char *  I    source file of the assertion
 *                                (__FILE__)
 *   line      int           I    source line of the assertion
 *                                (__LINE__)
 *   expr      const char *  I    text of the failed expression
 *
 * RETURNS: bool -- false, so the AGC_ASSERT expression is false
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) fprintf(stderr, "%s:%d: assertion '%s' failed\n", file, line, expr); /* diagnostic only: the caller already takes its error path */

    return (false);
}
