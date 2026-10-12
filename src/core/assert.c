/* agc_assert.c -- host-side reporting hook for AGC_ASSERT (rule 5). */
#include <stdio.h>

#include "agc/core/assert.h"

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) fprintf(stderr, "%s:%d: assertion '%s' failed\n", file, line, expr); /* diagnostic only: the caller already takes its error path */

    return (false);
}
