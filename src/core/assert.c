/* agc_assert.c -- host-side reporting hook for AGC_ASSERT (rule 5). */
#include <stdio.h>

#include "agc/core/assert.h"

bool agc_assert_report(const char *file, int line, const char *expr)
{
    /* Return value of fprintf is irrelevant to recovery: the caller
     * already takes its error path. (rule 7, documented ignore) */
    (void) fprintf(stderr, "%s:%d: assertion '%s' failed\n", file, line, expr);

    return false;
}
