/**********************************************************************
 * FILE NAME: oracle_txt.h
 *
 * PURPOSE:
 *   Shared reader for the yaAGC oracle cases (tests only). A test loads
 *   tools/oracle/cases/<name>.spec (the inputs) and the dump <name>.txt
 *   (the AGC's outputs, tools/oracle/README.md section 2), then:
 *     oracle_in_dp("tag", "RCV", 2)   word pair 2,3 of the last
 *                                     "set RCV=..." before "run tag"
 *     oracle_out_dp("tag", "RCV+2")   lines "RCV+2" and "RCV+3" of CASE tag
 *   and compares with oracle_cmp(). The spec is the single source of the
 *   input words, so the C run sees exactly what the emulator saw.
 *
 * AGC SOURCE:
 *   none (the oracle files are produced by running the Comanche055 and
 *   Luminary099 ropes in yaAGC; tools/oracle/README.md)
 *
 * REQUIRES:
 *   <stdbool.h>, <stdio.h>, <stdlib.h>, <string.h>, "agc/core/interp.h",
 *   "agc/core/word.h"
 *
 * GLOBAL VARIABLES:
 *   Name                Type    Scale        Description
 *   ------------------  ------  -----------  --------------------------
 *   oracle_txt          char[]  text         the loaded dump
 *   oracle_spec         char[]  text         the loaded spec
 *   oracle_checks       int     count        comparisons made
 *   oracle_failures     int     count        comparisons that failed
 *   oracle_missing      int     count        entries not found
 *   oracle_known        int     count        misses on recorded findings
 *   oracle_in_negzero   bool    flag         last oracle_in word was -0
 *
 * NOTES:
 *   Run from the repository root (or one level below); a missing file
 *   makes the test print SKIP and pass, as tests/oracle/core_numeric.c
 *   does.
 **********************************************************************/
#ifndef ORACLE_TXT_H
#define ORACLE_TXT_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agc/core/interp.h"
#include "agc/core/word.h"

#define ORACLE_TXT_MAX  (1 << 20) /* capacity of the yaAGC dump text, bytes */
#define ORACLE_SPEC_MAX (1 << 19) /* capacity of the case spec text, bytes */

static char oracle_txt[ORACLE_TXT_MAX];   /* the loaded dump <name>.txt (AGC outputs) */
static char oracle_spec[ORACLE_SPEC_MAX]; /* the loaded spec <name>.spec (AGC inputs) */
static int  oracle_checks;                /* comparisons made, count */
static int  oracle_failures;              /* comparisons that failed, count */
static int  oracle_missing;               /* dump or spec entries not found, count */
static int  oracle_known;                 /* comparisons that failed on a recorded finding, count */
static bool oracle_in_negzero;            /* last oracle_in() word was 77777 (-0) */

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read one
 *   oracle file into a buffer, trying the three relative locations of
 *   tools/oracle/cases
 *
 * FUNCTION NAME: oracle_read
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   name      const char *  I    file name below tools/oracle/cases
 *   buf       char *        O    receives the NUL-terminated text
 *   cap       size_t        I    buffer size, bytes
 *
 * RETURNS: bool -- true when a non-empty file was read
 **********************************************************************/
static inline bool oracle_read(const char *name, char *buf, size_t cap)
{
    static const char *const dirs[3] = {"tools/oracle/cases/",
                                        "../tools/oracle/cases/",
                                        "../../tools/oracle/cases/"};

    for (int k = 0; k < 3; k++)
    {
        char path[256]; /* candidate file name */

        (void) snprintf(path, sizeof (path), "%s%s", dirs[k], name);
        FILE *f = fopen(path, "r");

        if (f != NULL)
        {
            const size_t n = fread(buf, 1, cap - 1, f); /* bytes read, count */

            buf[n] = '\0';
            (void) fclose(f);

            return (n > 0);
        }
    }

    return (false);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- load
 *   <base>.spec and <base>.txt
 *
 * FUNCTION NAME: oracle_load
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   base      const char *  I    case file stem
 *
 * RETURNS: bool -- true when both files were read
 **********************************************************************/
static inline bool oracle_load(const char *base)
{
    char name[128]; /* dump or spec file name */

    (void) snprintf(name, sizeof (name), "%s.txt", base);
    const bool t = oracle_read(name, oracle_txt, sizeof (oracle_txt)); /* dump loaded */

    (void) snprintf(name, sizeof (name), "%s.spec", base);
    const bool s = oracle_read(name, oracle_spec, sizeof (oracle_spec)); /* spec loaded */

    return (t && s);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- ones'
 *   complement 15-bit word to signed value (-0 gives 0)
 *
 * FUNCTION NAME: oracle_word
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   w         long int  I    15-bit ones' complement word
 *
 * RETURNS: int -- the signed value, unscaled (int)
 **********************************************************************/
static inline int oracle_word(long int w)
{
    return ((w < 040000) ? (int) w : -(int) (077777 - w)); /* positive: as is; ones' complement negative: minus the complement */
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- find the
 *   line that starts with key as its whole first token
 *
 * FUNCTION NAME: oracle_find_line
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   p         const char *  I    where to start; NULL gives NULL
 *   end       const char *  I    end of the text to search
 *   key       const char *  I    first token to match
 *
 * RETURNS: const char * -- start of the line, or NULL
 **********************************************************************/
static inline const char *oracle_find_line(const char *p, const char *end, const char *key)
{
    const size_t n = strlen(key); /* length of the key, characters */

    while (p != NULL && p < end)
    {
        if (strncmp(p, key, n) == 0 && (p[n] == ' ' || p[n] == '\n'))
        {
            return (p);
        }

        p = strchr(p, '\n');
        p = (p != NULL) ? p + 1 : NULL; /* next line, or NULL at the end of the text */
    }

    return (NULL);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- locate the
 *   block of CASE tag in the dump
 *
 * FUNCTION NAME: oracle_case
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   tag       const char *   I    CASE tag of the run in the dump and
 *                                 spec
 *   b         const char **  O    start of the block
 *   e         const char **  O    end of the block (its END line)
 *
 * RETURNS: bool -- true when the block and its END line were found
 **********************************************************************/
static inline bool oracle_case(const char *tag, const char **b, const char **e)
{
    char key[160]; /* CASE line to find */

    (void) snprintf(key, sizeof (key), "CASE %s", tag);
    const char *end = oracle_txt + strlen(oracle_txt); /* end of the dump text */
    const char *p   = oracle_txt;                      /* scan position */

    p = oracle_find_line(p, end, "CASE");

    while (p != NULL)
    {
        const size_t n = strlen(key); /* length of the CASE line, characters */

        if (strncmp(p, key, n) == 0 && p[n] == '\n')
        {
            *b = p;
            *e = strstr(p, "\nEND");

            return (*e != NULL);
        }

        p++;
        p = oracle_find_line(p, end, "CASE");
    }

    return (false);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- check
 *   STATUS OK and, when stop is given, STOP == stop
 *
 * FUNCTION NAME: oracle_ok
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   stop      const char *  I    expected STOP label, or NULL for any
 *
 * RETURNS: bool -- true when the run finished as expected
 **********************************************************************/
static inline bool oracle_ok(const char *tag, const char *stop)
{
    const char *b = NULL; /* start of the CASE block */
    const char *e = NULL; /* end of the CASE block */

    if (!oracle_case(tag, &b, &e))
    {
        return (false);
    }

    const char *s = oracle_find_line(b, e, "STATUS"); /* STATUS line of the block */
    const char *t = oracle_find_line(b, e, "STOP");   /* STOP line of the block */

    if (s == NULL || strncmp(s, "STATUS OK", 9) != 0)
    {
        return (false);
    }

    return (stop == NULL || (t != NULL && strncmp(t + 5, stop, strlen(stop)) == 0 && t[5 + strlen(stop)] == '\n'));
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read one
 *   output word of CASE tag, signed
 *
 * FUNCTION NAME: oracle_out
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *   found     bool *        O    false when the word is absent from the
 *                                dump
 *
 * RETURNS: int -- the signed word, unscaled (int); 0 when absent
 **********************************************************************/
static inline int oracle_out(const char *tag, const char *name, bool *found)
{
    const char *b = NULL; /* start of the CASE block */
    const char *e = NULL; /* end of the CASE block */
    const char *l = NULL; /* line of the requested word */

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

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read one
 *   raw 15-bit output word (CDU patterns, flag words, -0 kept)
 *
 * FUNCTION NAME: oracle_out_raw
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *
 * RETURNS: unsigned int -- the raw word; 0 when absent
 **********************************************************************/
static inline unsigned int oracle_out_raw(const char *tag, const char *name)
{
    const char *b = NULL; /* start of the CASE block */
    const char *e = NULL; /* end of the CASE block */
    const char *l = NULL; /* line of the requested word */

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

    return ((unsigned int) strtoul(l + strlen(name) + 1, NULL, 8));
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- name of
 *   the next word: "X" gives "X+1", "X+n" gives "X+(n+1)"
 *
 * FUNCTION NAME: oracle_next_name
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   name      const char *  I    erasable name, optionally with +offset
 *   out       char *        O    receives the next name
 *   cap       size_t        I    size of out, bytes
 *
 * RETURNS: none
 **********************************************************************/
static inline void oracle_next_name(const char *name, char *out, size_t cap)
{
    const char *plus = strrchr(name, '+'); /* last + of the name, or NULL */

    if (plus == NULL)
    {
        (void) snprintf(out, cap, "%.60s+1", name);
    }
    else
    {
        (void) snprintf(out, cap, "%.*s+%ld", (int) (plus - name) & 63, name, strtol(plus + 1, NULL, 10) + 1);
    }
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read an SP
 *   output word of CASE tag
 *
 * FUNCTION NAME: oracle_out_sp
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *
 * RETURNS: int -- the signed word, unscaled (int)
 **********************************************************************/
static inline int oracle_out_sp(const char *tag, const char *name)
{
    bool f = false; /* found flag, not used by the caller */

    return (oracle_out(tag, name, &f));
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read a DP
 *   output: lines NAME and NAME+1
 *
 * FUNCTION NAME: oracle_out_dp
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *
 * RETURNS: agc_dp_t -- DP value, high word * 2^14 + low word
 **********************************************************************/
static inline agc_dp_t oracle_out_dp(const char *tag, const char *name)
{
    char next[96];  /* name of the low word */
    bool f = false; /* found flag, not used by the caller */

    oracle_next_name(name, next, sizeof (next));
    const int hi = oracle_out(tag, name, &f); /* high word (signed) */
    const int lo = oracle_out(tag, next, &f); /* low word (signed) */

    return ((agc_dp_t) hi * 16384 + (agc_dp_t) lo);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read a
 *   vector output: DP words at NAME, NAME+2, NAME+4
 *
 * FUNCTION NAME: oracle_out_vec
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *
 * RETURNS: agc_vec_t -- the vector, each component a DP value at the
 *          scale of the dump
 **********************************************************************/
static inline agc_vec_t oracle_out_vec(const char *tag, const char *name)
{
    agc_vec_t      v;                                                                /* the vector, scale of the dump words */
    const char    *plus = strrchr(name, '+');                                        /* last + of the name, or NULL */
    const long int base = (plus != NULL) ? strtol(plus + 1, NULL, 10) : 0;           /* offset after the +, or 0 when the name has none */
    const int      len  = (plus != NULL) ? (int) (plus - name) : (int) strlen(name); /* length of the name without its +offset */

    for (int k = 0; k < 3; k++)
    {
        char n[96]; /* name of one component */

        if (base + 2 * k == 0)
        {
            (void) snprintf(n, sizeof (n), "%.*s", len, name);
        }
        else
        {
            (void) snprintf(n, sizeof (n), "%.*s+%ld", len, name, base + 2 * k);
        }

        v.c[k] = oracle_out_dp(tag, n);
    }

    return (v);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read one
 *   input word: word idx of the last 'set NAME=...' before 'run tag'
 *   (octal, -n ones' complement, nnnd decimal)
 *
 * FUNCTION NAME: oracle_in
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *   idx       int           I    word index within the set line, from 0
 *
 * RETURNS: int -- the signed word, unscaled (int); 0 when the word is
 *          not set
 **********************************************************************/
static inline int oracle_in(const char *tag, const char *name, int idx)
{
    char run[160]; /* the run line to find */
    char key[96];  /* the set line to find */

    /* the generator splits long set lines into 8-word chunks NAME+8k */
    const int      chunk = (idx / 8) * 8;                                                                       /* first word of the 8-word chunk holding idx */
    const char    *plus  = strrchr(name, '+');                                                                  /* last + of the name, or NULL */
    const long int base  = (plus != NULL && plus != name) ? strtol(plus + 1, NULL, 10) : 0;                     /* offset after the +, or 0 when the name has none */
    const int      len   = (plus != NULL && plus != name) ? (int) (plus - name) & 63 : (int) strlen(name) & 63; /* length of the name without its +offset */

    idx -= chunk;
    (void) snprintf(run, sizeof (run), "run %s", tag);

    if (base + chunk == 0)
    {
        (void) snprintf(key, sizeof (key), "set %.*s=", len, name);
    }
    else
    {
        (void) snprintf(key, sizeof (key), "set %.*s+%ld=", len, name, base + chunk);
    }

    const char *end = oracle_spec; /* start of the wanted run line */
    const char *p   = oracle_spec; /* scan position */

    p = strstr(p, run);

    while (p != NULL)
    {
        const char c = p[strlen(run)]; /* character after the run tag */

        if ((p == oracle_spec || p[-1] == '\n') && (c == '\n' || c == ' ' || c == '\0'))
        {
            end = p;
            break;
        }

        p++;
        p = strstr(p, run);
    }

    if (end == oracle_spec)
    {
        oracle_missing++;
        (void) printf("MISSING run %s\n", tag);

        return (0);
    }

    /* start of this run's block: after the previous "run " line */
    const char *start = oracle_spec; /* start of this run's block */

    for (const char *q = oracle_spec; q < end; q++)
    {
        if ((q == oracle_spec || q[-1] == '\n') && strncmp(q, "run ", 4) == 0)
        {
            start = q;
        }
    }

    const char *hit = NULL; /* first word of the last matching set line */

    for (const char *q = start; q < end; q++)
    {
        if ((q == oracle_spec || q[-1] == '\n') && strncmp(q, key, strlen(key)) == 0)
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
        hit = (hit != NULL) ? hit + 1 : NULL; /* past the next comma, or NULL when there is none */
    }

    if (hit == NULL)
    {
        return (0);
    }

    char       *stop = NULL;                  /* end of the number */
    const bool  neg  = (*hit == '-');         /* ones' complement negative form (-n) */
    const char *num  = neg ? hit + 1 : hit;   /* first digit, past the sign if any */
    long int    v    = strtol(num, &stop, 8); /* word value, octal by default */

    if (stop != NULL && (*stop == 'd' || *stop == '8' || *stop == '9'))
    {
        v = strtol(num, NULL, 10); /* nnnd: decimal */
    }
    else if (!neg)
    {
        oracle_in_negzero = (v == 077777);

        return (oracle_word(v));
    }
    else
    {
        /* -n: ones' complement negative of octal n */
    }

    return (neg ? -(int) v : (int) v); /* negated for -n, else as read (decimal) */
} /* oracle_in */

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read a raw
 *   15-bit input word (CDU patterns, -0 kept): the signed value of
 *   oracle_in mapped back to ones' complement
 *
 * FUNCTION NAME: oracle_in_raw
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *   idx       int           I    word index within the set line, from 0
 *
 * RETURNS: uint16_t -- the raw 15-bit word
 **********************************************************************/
static inline uint16_t oracle_in_raw(const char *tag, const char *name, int idx)
{
    oracle_in_negzero = false;
    const int w       = oracle_in(tag, name, idx); /* signed value of the word */

    if (w == 0 && oracle_in_negzero)
    {
        return (077777); /* -0 */
    }

    return ((uint16_t) ((w < 0) ? w + 077777 : w)); /* negative values map to ones' complement */
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read a DP
 *   input from words idx and idx+1
 *
 * FUNCTION NAME: oracle_in_dp
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *   idx       int           I    index of the high word
 *
 * RETURNS: agc_dp_t -- DP value, high word * 2^14 + low word
 **********************************************************************/
static inline agc_dp_t oracle_in_dp(const char *tag, const char *name, int idx)
{
    return ((agc_dp_t) oracle_in(tag, name, idx) * 16384 + (agc_dp_t) oracle_in(tag, name, idx + 1));
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read a
 *   vector input of three DP words
 *
 * FUNCTION NAME: oracle_in_vec
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *
 * RETURNS: agc_vec_t -- the vector, each component a DP value at the
 *          scale of the spec
 **********************************************************************/
static inline agc_vec_t oracle_in_vec(const char *tag, const char *name)
{
    agc_vec_t v; /* the vector, scale of the dump words */

    for (int k = 0; k < 3; k++)
    {
        v.c[k] = oracle_in_dp(tag, name, 2 * k);
    }

    return (v);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read a
 *   triple-precision input from words idx..idx+2 (e.g. TEPHEM)
 *
 * FUNCTION NAME: oracle_in_tp
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *   idx       int           I    index of the first word
 *
 * RETURNS: agc_tp_t -- TP value, the three words weighted 2^28, 2^14,
 *          1
 **********************************************************************/
static inline agc_tp_t oracle_in_tp(const char *tag, const char *name, int idx)
{
    return (((agc_tp_t) oracle_in(tag, name, idx) * 16384 + (agc_tp_t) oracle_in(tag, name, idx + 1)) * 16384 + (agc_tp_t) oracle_in(tag, name, idx + 2));
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read
 *   vector input k (three DP words) of a longer set line, at word 6k
 *
 * FUNCTION NAME: oracle_in_vec_at
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   name      const char *  I    erasable name, optionally with +offset
 *   k         int           I    vector number within the set line,
 *                                from 0
 *
 * RETURNS: agc_vec_t -- the vector, each component a DP value at the
 *          scale of the spec
 **********************************************************************/
static inline agc_vec_t oracle_in_vec_at(const char *tag, const char *name, int k)
{
    agc_vec_t v; /* the vector, scale of the dump words */

    for (int c = 0; c < 3; c++)
    {
        v.c[c] = oracle_in_dp(tag, name, 6 * k + 2 * c);
    }

    return (v);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- read the
 *   vector left in MPAC: (MPAC, +1), (MPAC+3, +4), (MPAC+5, +6)
 *
 * FUNCTION NAME: oracle_out_mpac_vec
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *
 * RETURNS: agc_vec_t -- the vector, each component a DP value at the
 *          scale of the dump
 **********************************************************************/
static inline agc_vec_t oracle_out_mpac_vec(const char *tag)
{
    agc_vec_t v; /* the vector, scale of the dump words */

    v.c[0] = oracle_out_dp(tag, "MPAC");
    v.c[1] = oracle_out_dp(tag, "MPAC+3");
    v.c[2] = oracle_out_dp(tag, "MPAC+5");

    return (v);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- compare
 *   |got - want| with tol, counting the check and printing a failure
 *
 * FUNCTION NAME: oracle_cmp
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   what      const char *  I    label of the compared quantity
 *   got       long int      I    the C result, in the unit of the
 *                                quantity
 *   want      long int      I    the AGC result, same unit
 *   tol       long int      I    allowed difference, same unit
 *
 * RETURNS: bool -- true when within tolerance
 **********************************************************************/
static inline bool oracle_cmp(const char *tag, const char *what, long int got, long int want, long int tol)
{
    const long int d = got - want; /* C minus AGC */

    oracle_checks++;

    if (d != 0 && getenv("ORACLE_VERBOSE") != NULL)
    {
        (void) printf("DIFF %s %s: C %ld AGC %ld (diff %ld, tol %ld)\n", tag, what, got, want, d, tol);
    }

    if (d > tol || d < -tol)
    {
        oracle_failures++;
        (void) printf("FAIL %s %s: C %ld AGC %ld (diff %ld, tol %ld)\n", tag, what, got, want, d, tol);

        return (false);
    }

    return (true);
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- compare as
 *   oracle_cmp, but a miss caused by a recorded finding is printed as
 *   KNOWN and counted apart
 *
 * FUNCTION NAME: oracle_cmp_known
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   what      const char *  I    label of the compared quantity
 *   got       long int      I    the C result, in the unit of the
 *                                quantity
 *   want      long int      I    the AGC result, same unit
 *   tol       long int      I    allowed difference, same unit
 *   id        const char *  I    entry of docs/modules/oracle-cases.md
 *                                Findings
 *
 * RETURNS: none
 **********************************************************************/
static inline void oracle_cmp_known(const char *tag, const char *what, long int got, long int want, long int tol, const char *id)
{
    const long int d = got - want; /* C minus AGC */

    oracle_checks++;

    if (d > tol || d < -tol)
    {
        oracle_known++;
        (void) printf("KNOWN %s %s %s: C %ld AGC %ld (diff %ld)\n", id, tag, what, got, want, d);
    }
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) --
 *   oracle_cmp_known on the three components of a vector
 *
 * FUNCTION NAME: oracle_cmp_vec_known
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   what      const char *  I    label of the vector
 *   got       agc_vec_t     I    the C vector
 *   want      agc_vec_t     I    the AGC vector, same scale
 *   tol       long int      I    allowed difference per component
 *   id        const char *  I    entry of docs/modules/oracle-cases.md
 *                                Findings
 *
 * RETURNS: none
 **********************************************************************/
static inline void oracle_cmp_vec_known(const char *tag, const char *what, agc_vec_t got, agc_vec_t want, long int tol, const char *id)
{
    static const char *const axis[3] = {"x", "y", "z"};

    for (int k = 0; k < 3; k++)
    {
        char w[96]; /* component label */

        (void) snprintf(w, sizeof (w), "%s.%s", what, axis[k]);
        oracle_cmp_known(tag, w, (long int) got.c[k], (long int) want.c[k], tol, id);
    }
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- oracle_cmp
 *   on the three components of a vector
 *
 * FUNCTION NAME: oracle_cmp_vec
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   what      const char *  I    label of the vector
 *   got       agc_vec_t     I    the C vector
 *   want      agc_vec_t     I    the AGC vector, same scale
 *   tol       long int      I    allowed difference per component
 *
 * RETURNS: none
 **********************************************************************/
static inline void oracle_cmp_vec(const char *tag, const char *what, agc_vec_t got, agc_vec_t want, long int tol)
{
    static const char *const axis[3] = {"x", "y", "z"};

    for (int k = 0; k < 3; k++)
    {
        char w[96]; /* component label */

        (void) snprintf(w, sizeof (w), "%s.%s", what, axis[k]);
        (void) oracle_cmp(tag, w, (long int) got.c[k], (long int) want.c[k], tol);
    }
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- count a
 *   check that the AGC run reached its stop with STATUS OK, and report
 *   a failure
 *
 * FUNCTION NAME: oracle_require_ok
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   tag       const char *  I    CASE tag of the run in the dump and
 *                                spec
 *   stop      const char *  I    expected STOP label, or NULL for any
 *
 * RETURNS: none
 **********************************************************************/
static inline void oracle_require_ok(const char *tag, const char *stop)
{
    oracle_checks++;

    if (!oracle_ok(tag, stop))
    {
        oracle_failures++;
        (void) printf("FAIL %s: AGC run did not reach %s\n", tag, (stop != NULL) ? stop : "its stop"); /* the stop label, or a generic name */
    }
}

/**********************************************************************
 * AGC: none (test helper reading the yaAGC oracle files) -- print the
 *   summary line
 *
 * FUNCTION NAME: oracle_finish
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ------------------------------------
 *   test      const char *  I    test name for the summary line
 *
 * RETURNS: int -- 0 when every check passed and none is missing, else
 *          1
 **********************************************************************/
static inline int oracle_finish(const char *test)
{
    (void) printf("%s: %d checks, %d failures, %d missing, %d known " "(findings)\n", test, oracle_checks, oracle_failures, oracle_missing, oracle_known);

    return ((oracle_failures == 0 && oracle_missing == 0 && oracle_checks > 0) ? 0 : 1); /* 0 when every check passed and none is missing */
}

#endif /* ORACLE_TXT_H */
