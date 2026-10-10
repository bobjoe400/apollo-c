/*
 * agc_assert.h -- side-effect-free assertion (Power of Ten rule 5).
 *
 * Usage:
 *     if (!AGC_ASSERT(x >= 0)) { return AGC_STATUS_BAD_ARG; }
 *
 * AGC_ASSERT evaluates to a bool and never aborts the program. When
 * AGC_HOST_DEBUG is defined (test builds) a failed assertion is also
 * reported through agc_assert_report() so unit tests can see it.
 */
#ifndef AGC_ASSERT_H
#define AGC_ASSERT_H

#include <stdbool.h>

bool agc_assert_report(const char *file, int line, const char *expr);

#ifdef AGC_HOST_DEBUG
#define AGC_ASSERT(e) ((e) ? true : agc_assert_report(__FILE__, __LINE__, #e))
#else
#define AGC_ASSERT(e) ((e) ? true : false)
#endif

#endif /* AGC_ASSERT_H */
