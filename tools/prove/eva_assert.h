/*
 * eva_assert.h -- force-included by `make prove` (Frama-C EVA only, never
 * by the build).  It takes the place of include/agc/core/assert.h: each
 * AGC_ASSERT(e) becomes a call of agc_eva_check whose ACSL precondition is
 * the asserted condition, so EVA reports one property per assertion site
 * (VALID: the condition holds on every path EVA explores; UNKNOWN /
 * INVALID: it may fail) at the file:line of the AGC_ASSERT.
 */
#define AGC_ASSERT_H
#include <stdbool.h>

bool agc_assert_report(const char *file, int line, const char *expr);

/*@ requires agc_assert: c != 0;
    assigns \nothing;
    ensures \result == (c != 0);
 */
int agc_eva_check(int c);

#define AGC_ASSERT(e) agc_eva_check((e) ? 1 : 0)
