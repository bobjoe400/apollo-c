/*
 * cgast_assert.h -- force-included by tools/cgast.py (libclang analysis
 * only, never by the build).  It replaces the AGC_ASSERT macro with a
 * function declaration so that the asserted expression keeps its real
 * source extents (inside a macro argument libclang collapses them), and
 * guards compare by AST.
 */
#define AGC_ASSERT_H
#include <stdbool.h>
bool AGC_ASSERT(bool e);
bool agc_assert_report(const char *file, int line, const char *expr);
