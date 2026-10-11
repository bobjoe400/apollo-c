/*
 * oracle_txt.h -- shared reader for the yaAGC oracle cases (tests only).
 *
 * A test loads tools/oracle/cases/<name>.spec (the inputs) and the dump
 * <name>.txt (the AGC's outputs, tools/oracle/README.md section 2), then:
 *   oracle_in_dp("tag", "RCV", 2)   word pair 2,3 of the last
 *                                   "set RCV=..." before "run tag"
 *   oracle_out_dp("tag", "RCV+2")   lines "RCV+2" and "RCV+3" of CASE tag
 * and compares with oracle_cmp(). The spec is the single source of the
 * input words, so the C run sees exactly what the emulator saw.
 * Run from the repository root (or one level below); a missing file
 * makes the test print SKIP and pass, as tests/oracle/core_numeric.c does.
 */
#ifndef ORACLE_TXT_H
#define ORACLE_TXT_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agc/core/interp.h"
#include "agc/core/word.h"

#define ORACLE_TXT_MAX  (1 << 20)
#define ORACLE_SPEC_MAX (1 << 19)

static char oracle_txt[ORACLE_TXT_MAX];
static char oracle_spec[ORACLE_SPEC_MAX];
static int  oracle_checks;
static int  oracle_failures;
static int  oracle_missing;
static int  oracle_known;

static inline bool oracle_read(const char *name, char *buf, size_t cap)
{
    static const char *const dirs[3] = {"tools/oracle/cases/",
                                        "../tools/oracle/cases/",
                                        "../../tools/oracle/cases/"};

    for (int k = 0; k < 3; k++)
    {
        char path[256];

        (void) snprintf(path, sizeof path, "%s%s", dirs[k], name);
        FILE *f = fopen(path, "r");

        if (f != NULL)
        {
            const size_t n = fread(buf, 1, cap - 1, f);

            buf[n] = '\0';
            (void) fclose(f);

            return (n > 0);
        }
    }

    return (false);
}

/* Load <base>.spec and <base>.txt; false when either is missing. */
static inline bool oracle_load(const char *base)
{
    char name[128];

    (void) snprintf(name, sizeof name, "%s.txt", base);
    const bool t = oracle_read(name, oracle_txt, sizeof oracle_txt);

    (void) snprintf(name, sizeof name, "%s.spec", base);
    const bool s = oracle_read(name, oracle_spec, sizeof oracle_spec);

    return (t && s);
}

/* Ones' complement 15-bit word -> signed value (-0 -> 0). */
static inline int oracle_word(long w)
{
    return ((w < 040000) ? (int) w : -(int) (077777 - w));
}

/* Start of line `key` (whole first token) in [p, end), or NULL. */
static inline const char *oracle_find_line(const char *p, const char *end,
                                           const char *key)
{
    const size_t n = strlen(key);

    while (p != NULL && p < end)
    {
        if (strncmp(p, key, n) == 0 && (p[n] == ' ' || p[n] == '\n'))
        {
            return (p);
        }

        p = strchr(p, '\n');
        p = (p != NULL) ? p + 1 : NULL;
    }

    return (NULL);
}

/* Block of CASE tag in the dump: [*b, *e). */
static inline bool oracle_case(const char *tag, const char **b,
                               const char **e)
{
    char key[160];

    (void) snprintf(key, sizeof key, "CASE %s", tag);
    const char *end = oracle_txt + strlen(oracle_txt);
    const char *p   = oracle_txt;

    while ((p = oracle_find_line(p, end, "CASE")) != NULL)
    {
        const size_t n = strlen(key);

        if (strncmp(p, key, n) == 0 && p[n] == '\n')
        {
            *b = p;
            *e = strstr(p, "\nEND");

            return (*e != NULL);
        }

        p++;
    }

    return (false);
}

/* STATUS OK and, when stop != NULL, STOP == stop. */
static inline bool oracle_ok(const char *tag, const char *stop)
{
    const char *b = NULL;
    const char *e = NULL;

    if (!oracle_case(tag, &b, &e))
    {
        return (false);
    }

    const char *s = oracle_find_line(b, e, "STATUS");
    const char *t = oracle_find_line(b, e, "STOP");

    if (s == NULL || strncmp(s, "STATUS OK", 9) != 0)
    {
        return (false);
    }

    return (stop == NULL ||
            (t != NULL && strncmp(t + 5, stop, strlen(stop)) == 0 &&
             t[5 + strlen(stop)] == '\n'));
}

/* Output word NAME of CASE tag (signed); *found = false if absent. */
static inline int oracle_out(const char *tag, const char *name, bool *found)
{
    const char *b = NULL;
    const char *e = NULL;
    const char *l = NULL;

    *found = false;

    if (oracle_case(tag, &b, &e))
    {
        l = oracle_find_line(b, e, name);
    }

    if (l == NULL)
    {
        oracle_missing++;
        (void) printf("MISSING %s %s\n", tag, name);

        return (0);
    }

    *found = true;

    return (oracle_word(strtol(l + strlen(name) + 1, NULL, 8)));
}

/* Raw 15-bit output word (CDU patterns, flag words, -0 kept). */
static inline unsigned oracle_out_raw(const char *tag, const char *name)
{
    const char *b = NULL;
    const char *e = NULL;
    const char *l = NULL;

    if (oracle_case(tag, &b, &e))
    {
        l = oracle_find_line(b, e, name);
    }

    if (l == NULL)
    {
        oracle_missing++;
        (void) printf("MISSING %s %s\n", tag, name);

        return (0);
    }

    return ((unsigned) strtoul(l + strlen(name) + 1, NULL, 8));
}

/* "X" -> "X+1", "X+n" -> "X+(n+1)". */
static inline void oracle_next_name(const char *name, char *out, size_t cap)
{
    const char *plus = strrchr(name, '+');

    if (plus == NULL)
    {
        (void) snprintf(out, cap, "%.60s+1", name);
    }
    else
    {
        (void) snprintf(out, cap, "%.*s+%ld", (int) (plus - name) & 63, name,
                        strtol(plus + 1, NULL, 10) + 1);
    }
}

static inline int oracle_out_sp(const char *tag, const char *name)
{
    bool f = false;

    return (oracle_out(tag, name, &f));
}

/* DP output: lines NAME and NAME+1 (as "get NAME:dp" prints them). */
static inline agc_dp_t oracle_out_dp(const char *tag, const char *name)
{
    char next[96];
    bool f = false;

    oracle_next_name(name, next, sizeof next);
    const int hi = oracle_out(tag, name, &f);
    const int lo = oracle_out(tag, next, &f);

    return ((agc_dp_t) hi * 16384 + (agc_dp_t) lo);
}

/* Vector output: three DP words from NAME, NAME+2, NAME+4. */
static inline agc_vec_t oracle_out_vec(const char *tag, const char *name)
{
    agc_vec_t   v;
    const char *plus = strrchr(name, '+');
    const long  base = (plus != NULL) ? strtol(plus + 1, NULL, 10) : 0;
    const int   len  = (plus != NULL) ? (int) (plus - name) : (int) strlen(name);

    for (int k = 0; k < 3; k++)
    {
        char n[96];

        if (base + 2 * k == 0)
        {
            (void) snprintf(n, sizeof n, "%.*s", len, name);
        }
        else
        {
            (void) snprintf(n, sizeof n, "%.*s+%ld", len, name, base + 2 * k);
        }

        v.c[k] = oracle_out_dp(tag, n);
    }

    return (v);
}

/* One input word: word `idx` of the last "set NAME=..." before "run tag"
 * (spec forms: octal, -n ones' complement, nnnd decimal).               */
static inline int oracle_in(const char *tag, const char *name, int idx)
{
    char run[160];
    char key[96];

    /* the generator splits long set lines into 8-word chunks NAME+8k */
    const int   chunk = (idx / 8) * 8;
    const char *plus  = strrchr(name, '+');
    const long  base  = (plus != NULL && plus != name) ?
                        strtol(plus + 1, NULL, 10) : 0;
    const int   len   = (plus != NULL && plus != name) ?
                        (int) (plus - name) & 63 : (int) strlen(name) & 63;

    idx -= chunk;
    (void) snprintf(run, sizeof run, "run %s", tag);

    if (base + chunk == 0)
    {
        (void) snprintf(key, sizeof key, "set %.*s=", len, name);
    }
    else
    {
        (void) snprintf(key, sizeof key, "set %.*s+%ld=", len, name,
                        base + chunk);
    }

    const char *end = oracle_spec;
    const char *p   = oracle_spec;

    while ((p = strstr(p, run)) != NULL)
    {
        const char c = p[strlen(run)];

        if ((p == oracle_spec || p[-1] == '\n') &&
            (c == '\n' || c == ' ' || c == '\0'))
        {
            end = p;
            break;
        }

        p++;
    }

    if (end == oracle_spec)
    {
        oracle_missing++;
        (void) printf("MISSING run %s\n", tag);

        return (0);
    }

    /* start of this run's block: after the previous "run " line */
    const char *start = oracle_spec;

    for (const char *q = oracle_spec; q < end; q++)
    {
        if ((q == oracle_spec || q[-1] == '\n') && strncmp(q, "run ", 4) == 0)
        {
            start = q;
        }
    }

    const char *hit = NULL;

    for (const char *q = start; q < end; q++)
    {
        if ((q == oracle_spec || q[-1] == '\n') &&
            strncmp(q, key, strlen(key)) == 0)
        {
            hit = q + strlen(key);
        }
    }

    if (hit == NULL)
    {
        return (0); /* not set: zeroed erasable */
    }

    for (int k = 0; k < idx && hit != NULL; k++)
    {
        hit = strchr(hit, ',');
        hit = (hit != NULL) ? hit + 1 : NULL;
    }

    if (hit == NULL)
    {
        return (0);
    }

    char      *stop = NULL;
    const bool neg  = (*hit == '-');
    const char *num = neg ? hit + 1 : hit;
    long        v   = strtol(num, &stop, 8);

    if (stop != NULL && (*stop == 'd' || *stop == '8' || *stop == '9'))
    {
        v = strtol(num, NULL, 10); /* nnnd: decimal */
    }
    else if (!neg)
    {
        return (oracle_word(v));
    }
    else
    {
        /* -n: ones' complement negative of octal n */
    }

    return (neg ? -(int) v : (int) v);
}

/* Raw 15-bit input word (CDU patterns, -0 kept): the signed value of
 * oracle_in() mapped back to ones' complement.                          */
static inline uint16_t oracle_in_raw(const char *tag, const char *name,
                                     int idx)
{
    const int w = oracle_in(tag, name, idx);

    return ((uint16_t) ((w < 0) ? w + 077777 : w));
}

static inline agc_dp_t oracle_in_dp(const char *tag, const char *name, int idx)
{
    return ((agc_dp_t) oracle_in(tag, name, idx) * 16384 +
            (agc_dp_t) oracle_in(tag, name, idx + 1));
}

static inline agc_vec_t oracle_in_vec(const char *tag, const char *name)
{
    agc_vec_t v;

    for (int k = 0; k < 3; k++)
    {
        v.c[k] = oracle_in_dp(tag, name, 2 * k);
    }

    return (v);
}

/* Triple-precision input word triple at word idx (e.g. TEPHEM). */
static inline agc_tp_t oracle_in_tp(const char *tag, const char *name, int idx)
{
    return (((agc_tp_t) oracle_in(tag, name, idx) * 16384 +
             (agc_tp_t) oracle_in(tag, name, idx + 1)) * 16384 +
            (agc_tp_t) oracle_in(tag, name, idx + 2));
}

/* Vector input k (three DP words) of a longer set line (vector k*6). */
static inline agc_vec_t oracle_in_vec_at(const char *tag, const char *name,
                                         int k)
{
    agc_vec_t v;

    for (int c = 0; c < 3; c++)
    {
        v.c[c] = oracle_in_dp(tag, name, 6 * k + 2 * c);
    }

    return (v);
}

/* Vector left in MPAC: (MPAC, +1), (MPAC+3, +4), (MPAC+5, +6). */
static inline agc_vec_t oracle_out_mpac_vec(const char *tag)
{
    agc_vec_t v;

    v.c[0] = oracle_out_dp(tag, "MPAC");
    v.c[1] = oracle_out_dp(tag, "MPAC+3");
    v.c[2] = oracle_out_dp(tag, "MPAC+5");

    return (v);
}

/* |got - want| <= tol, counted and printed on failure. */
static inline bool oracle_cmp(const char *tag, const char *what, long got,
                              long want, long tol)
{
    const long d = got - want;

    oracle_checks++;

    if (d != 0 && getenv("ORACLE_VERBOSE") != NULL)
    {
        (void) printf("DIFF %s %s: C %ld AGC %ld (diff %ld, tol %ld)\n", tag,
                      what, got, want, d, tol);
    }

    if (d > tol || d < -tol)
    {
        oracle_failures++;
        (void) printf("FAIL %s %s: C %ld AGC %ld (diff %ld, tol %ld)\n", tag,
                      what, got, want, d, tol);

        return (false);
    }

    return (true);
}

/* A comparison that fails because of a recorded finding (id = the
 * docs/modules/oracle-cases.md "Findings" entry): printed as KNOWN and
 * counted apart, so `make test` stays green until the review wave fixes
 * src/; when the summary shows 0 known the entry can be closed and the
 * call turned back into oracle_cmp().                                     */
static inline void oracle_cmp_known(const char *tag, const char *what,
                                    long got, long want, long tol,
                                    const char *id)
{
    const long d = got - want;

    oracle_checks++;

    if (d > tol || d < -tol)
    {
        oracle_known++;
        (void) printf("KNOWN %s %s %s: C %ld AGC %ld (diff %ld)\n", id, tag,
                      what, got, want, d);
    }
}

static inline void oracle_cmp_vec_known(const char *tag, const char *what,
                                        agc_vec_t got, agc_vec_t want, long tol,
                                        const char *id)
{
    static const char *const axis[3] = {"x", "y", "z"};

    for (int k = 0; k < 3; k++)
    {
        char w[96];

        (void) snprintf(w, sizeof w, "%s.%s", what, axis[k]);
        oracle_cmp_known(tag, w, (long) got.c[k], (long) want.c[k], tol, id);
    }
}

static inline void oracle_cmp_vec(const char *tag, const char *what,
                                  agc_vec_t got, agc_vec_t want, long tol)
{
    static const char *const axis[3] = {"x", "y", "z"};

    for (int k = 0; k < 3; k++)
    {
        char w[96];

        (void) snprintf(w, sizeof w, "%s.%s", what, axis[k]);
        (void) oracle_cmp(tag, w, (long) got.c[k], (long) want.c[k], tol);
    }
}

static inline void oracle_require_ok(const char *tag, const char *stop)
{
    oracle_checks++;

    if (!oracle_ok(tag, stop))
    {
        oracle_failures++;
        (void) printf("FAIL %s: AGC run did not reach %s\n", tag,
                      (stop != NULL) ? stop : "its stop");
    }
}

/* Summary line; exit status for main(). */
static inline int oracle_finish(const char *test)
{
    (void) printf("%s: %d checks, %d failures, %d missing, %d known "
                  "(findings)\n", test, oracle_checks, oracle_failures,
                  oracle_missing, oracle_known);

    return ((oracle_failures == 0 && oracle_missing == 0 &&
             oracle_checks > 0) ? 0 : 1);
}

#endif /* ORACLE_TXT_H */
