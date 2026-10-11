/*
 * pad.c -- erasable pad load at power-on (--pad FILE).
 *
 * yaAGC starts from a core-image file; the C translation keeps every
 * erasable in its owner's typed storage, which V21N01 / V71 cannot reach
 * (src/core/ecadr.c DEVIATION). A pad file sets the few erasables a
 * scenario needs before GOJAM, through the public setters, in engineering
 * units. One item per line, `#` starts a comment:
 *
 *     refsmmat m11 m12 m13 m21 m22 m23 m31 m32 m33
 *         REFSMMAT rows (unit vectors), stored at scale 2^1 (state.h)
 *     flagword N OCTAL
 *         FLAGWRDn |= OCTAL (e.g. `flagword 3 20000` = REFSMMFLG)
 *
 * Unknown keywords and malformed lines are reported and fail start-up.
 */
#include <stdio.h>
#include <string.h>

#include "agc/core/assert.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/word.h"
#include "internal.h"

#define PAD_LINE        512
#define PAD_LINES_MAX   256  /* bound on lines read                 */
#define PAD_FLAGWORDS   14   /* FLAGWRD0 .. FLGWRD13                 */
#define PAD_MAT_N       9
#define PAD_SCALE_REFSM 1    /* REFSMMAT 'SCALE OF B1'               */
#define PAD_UNIT_MAX    1.0

/* refsmmat: nine unit-vector components, |m| <= 1.                     */
static bool pad_refsmmat(const char *args)
{
    double m[PAD_MAT_N];

    if (!AGC_ASSERT(args != NULL))
    {
        return (false);
    }

    if (sscanf(args, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &m[0], &m[1],
               &m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8]) != PAD_MAT_N)
    {
        return (false);
    }

    agc_mat_t r;

    for (int i = 0; i < PAD_MAT_N; i++)
    {
        if (m[i] > PAD_UNIT_MAX || m[i] < -PAD_UNIT_MAX)
        {
            return (false);
        }

        r.row[i / 3].c[i % 3] = agc_dp_from_double(m[i], PAD_SCALE_REFSM);
    }

    agc_erase_set_refsmmat(r);

    return (AGC_ASSERT(r.row[0].c[0] == agc_erase_refsmmat().row[0].c[0]));
}

/* flagword N OCTAL: OR the bits into FLAGWRDn.                          */
static bool pad_flagword(const char *args)
{
    int          word = -1;
    unsigned int bits = 0;

    if (!AGC_ASSERT(args != NULL) || sscanf(args, "%d %o", &word, &bits) != 2)
    {
        return (false);
    }

    if (word < 0 || word >= PAD_FLAGWORDS || bits > SIM_WORD_MASK)
    {
        return (false);
    }

    uint16_t v = (uint16_t) (agc_flagword_get(word) | (uint16_t) bits);

    agc_flagword_set(word, v);

    return (AGC_ASSERT((agc_flagword_get(word) & bits) == bits));
}

/* One line; true when it is blank, a comment, or applied.              */
static bool pad_line(char *buf)
{
    char *hash = strchr(buf, '#');
    char  key[32];
    int   used = 0;

    if (!AGC_ASSERT(buf != NULL))
    {
        return (false);
    }

    if (hash != NULL)
    {
        *hash = '\0';
    }

    if (sscanf(buf, "%31s%n", key, &used) != 1)
    {
        return (true); /* blank */
    }

    if (!AGC_ASSERT(used > 0 && (size_t) used <= strlen(buf)))
    {
        return (false);
    }

    if (strcmp(key, "refsmmat") == 0)
    {
        return (pad_refsmmat(&buf[used]));
    }

    if (strcmp(key, "flagword") == 0)
    {
        return (pad_flagword(&buf[used]));
    }

    return (false);
}

agc_status_t sim_pad_load(const char *path)
{
    char buf[PAD_LINE];
    bool ok = true;

    if (!AGC_ASSERT(path != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    FILE *f = fopen(path, "r");

    if (f == NULL)
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    for (int i = 1; i <= PAD_LINES_MAX && ok; i++)
    {
        if (fgets(buf, (int) sizeof(buf), f) == NULL)
        {
            break;
        }

        ok = pad_line(buf);

        if (!ok)
        {
            (void) fprintf(stderr, "agc_sim: pad %s line %d not understood\n",
                           path, i); /* diagnostic only */
        }
    }

    if (fclose(f) != 0)
    {
        ok = false;
    }

    return (ok ? AGC_STATUS_OK : AGC_STATUS_BAD_ARG);
}
