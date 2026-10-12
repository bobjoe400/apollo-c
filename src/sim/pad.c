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
 *         FLAGWRDn |= OCTAL (e.g. `flagword 3 10000` = REFSMFLG)
 *     sv csm|lm X Y Z VX VY VZ [T]
 *         permanent state vector (RRECTCSM.. / RRECTLEM..), earth sphere,
 *         m and m/s in the reference frame, at time T s (default 0):
 *         RRECT = RCV at 2^29 m, VRECT = VCV at 2^7 m/cs, TET at 2^28 cs,
 *         TDELTAV = TNUV = TC = XKEP = 0 (a freshly rectified state).
 *
 * Unknown keywords and malformed lines are reported and fail start-up.
 *
 * The same file is accepted as yaAGC's --resume core file (the launcher's
 * "custom core file" start-up option): sim_pad_is_pad() tells a pad file
 * from a yaAGC core dump, which the typed C erasable cannot take.
 */
#include <stdio.h>
#include <string.h>

#include "agc/core/assertion.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/word.h"
#include "agc/nav/integration.h"
#include "internal.h"

#define PAD_LINE        512
#define PAD_LINES_MAX   256   /* bound on lines read                 */
#define PAD_MAT_N       9
#define PAD_SCALE_REFSM 1     /* REFSMMAT 'SCALE OF B1'               */
#define PAD_UNIT_MAX    1.0
#define PAD_SCALE_R     29    /* RRECT earth sphere, m                */
#define PAD_SCALE_V     7     /* VRECT earth sphere, m/cs             */
#define PAD_SCALE_T     28    /* TET, cs                              */
#define PAD_R_MAX       5.0e8 /* < 2^29 m                          */
#define PAD_V_MAX       1.2e4 /* m/s, < 2^7 m/cs                   */
#define PAD_T_MAX       2.6e6 /* s, < 2^28 cs                      */
#define PAD_CS_PER_S    100.0

/* refsmmat: nine unit-vector components, |m| <= 1.                     */
static bool pad_refsmmat(const char *args)
{
    double m[PAD_MAT_N];

    if (sscanf(args, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8]) != PAD_MAT_N)
    {
        return (false);
    }

    agc_mat_t r;

    for (int i = 0; i < PAD_MAT_N; i++)
    {
        if (!(m[i] <= PAD_UNIT_MAX && m[i] >= -PAD_UNIT_MAX)) /* also NaN */
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

    if (sscanf(args, "%d %o", &word, &bits) != 2)
    {
        return (false);
    }

    if (word < 0 || word >= ((agc_state_vehicle() == AGC_VEHICLE_LM) ? AGC_FLAGWORDS_LM : AGC_FLAGWORDS_CSM) || bits > SIM_WORD_MASK)
    {
        return (false);
    }

    uint16_t v = (uint16_t) (agc_flagword_get(word) | (uint16_t) bits);

    agc_flagword_set(word, v);

    return (AGC_ASSERT((agc_flagword_get(word) & bits) == bits));
}

/* sv csm|lm X Y Z VX VY VZ [T]: a rectified earth-sphere state.       */
static bool pad_state_vector(const char *args)
{
    char   veh[8];
    double x[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int  n  = sscanf(args, "%7s %lf %lf %lf %lf %lf %lf %lf", veh, &x[0], &x[1], &x[2], &x[3], &x[4], &x[5], &x[6]);
    bool lm = strcmp(veh, "lm") == 0;

    if (n < 7 || (!lm && strcmp(veh, "csm") != 0) || !(x[6] >= 0.0 && x[6] <= PAD_T_MAX)) /* also NaN */
    {
        return (false);
    }

    agc_integ_sv_t sv = {0};

    for (int i = 0; i < 3; i++)
    {
        if (!(x[i] <= PAD_R_MAX && x[i] >= -PAD_R_MAX && x[i + 3] <= PAD_V_MAX && x[i + 3] >= -PAD_V_MAX)) /* also NaN */
        {
            return (false);
        }

        sv.rrect.c[i] = agc_dp_from_double(x[i], PAD_SCALE_R);
        sv.vrect.c[i] = agc_dp_from_double(x[i + 3] / PAD_CS_PER_S, PAD_SCALE_V);
    }

    sv.rcv = sv.rrect;
    sv.vcv = sv.vrect;
    sv.tet = agc_dp_from_double(x[6] * PAD_CS_PER_S, PAD_SCALE_T);

    return (agc_integ_perm_set(lm ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM, &sv) == AGC_STATUS_OK);
}

/* One line; true when it is blank, a comment, or applied.              */
static bool pad_line(char *buf)
{
    char *hash = strchr(buf, '#');
    char  key[32];
    int   used = 0;

    if (hash != NULL)
    {
        *hash = '\0';
    }

    if (sscanf(buf, "%31s%n", key, &used) != 1)
    {
        return (true); /* blank */
    }

    if (strcmp(key, "refsmmat") == 0)
    {
        return (pad_refsmmat(&buf[used]));
    }

    if (strcmp(key, "flagword") == 0)
    {
        return (pad_flagword(&buf[used]));
    }

    if (strcmp(key, "sv") == 0)
    {
        return (pad_state_vector(&buf[used]));
    }

    return (false);
}

/* True when the first item of the file at `path` is a pad keyword (a
 * yaAGC core dump starts with octal numbers instead).                  */
bool sim_pad_is_pad(const char *path)
{
    char  buf[PAD_LINE];
    char  key[32];
    bool  pad  = false;
    bool  seen = false;
    FILE *f    = fopen(path, "r");

    if (f == NULL)
    {
        return (false);
    }

    for (int i = 1; i <= PAD_LINES_MAX && !seen && fgets(buf, (int) sizeof (buf), f) != NULL; i++)
    {
        char *hash = strchr(buf, '#');

        if (hash != NULL)
        {
            *hash = '\0';
        }

        if (sscanf(buf, "%31s", key) == 1)
        {
            seen = true;
            pad  = strcmp(key, "refsmmat") == 0 || strcmp(key, "flagword") == 0 || strcmp(key, "sv") == 0;
        }
    }

    (void) fclose(f); /* read-only file: nothing to lose */

    return (pad);
}

agc_status_t sim_pad_load(const char *path)
{
    char buf[PAD_LINE];
    bool ok = true;

    FILE *f = fopen(path, "r");

    if (f == NULL)
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    for (int i = 1; i <= PAD_LINES_MAX && ok; i++)
    {
        if (fgets(buf, (int) sizeof (buf), f) == NULL)
        {
            break;
        }

        ok = pad_line(buf);

        if (!ok)
        {
            (void) fprintf(stderr, "agc_sim: pad %s line %d not understood\n", path, i); /* diagnostic only */
        }
    }

    if (fclose(f) != 0)
    {
        ok = false;
    }

    return (ok ? AGC_STATUS_OK : AGC_STATUS_BAD_ARG);
}
