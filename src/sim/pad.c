/**********************************************************************
 * FILE NAME: pad.c
 *
 * PURPOSE:
 *   Erasable pad load at power-on (--pad FILE, or --resume FILE when FILE
 *   is a pad file): the few erasables a scenario needs, set before GOJAM
 *   through the public setters, in engineering units.
 *
 * AGC SOURCE:
 *   none (harness input; the erasables are REFSMMAT, FLAGWRD0-n and the
 *   permanent state vectors RRECTCSM/RRECTLEM.. of the AGC programs)
 *
 * EXTERNAL VARIABLES:
 *   Name        Type            I/O  Scale             Description
 *   ----------  --------------  ---  ----------------  -------------------
 *   REFSMMAT    agc_mat_t       O    2^1               agc_erase_set_refsmmat
 *   FLAGWRDn    uint16_t        I/O  bits              agc_flagword_set
 *   RRECT,      agc_integ_sv_t  O    2^29 m, 2^7 m/cs, agc_integ_perm_set
 *   VRECT, TET                       2^28 cs
 *   EK1VAL,     agc_dp_t        O    2^23 N-s/100      agc_guid_state()->csm
 *   EK2VAL
 *   EK3VAL      agc_dp_t        O    2^9 N/10^4 (with  agc_guid_state()->csm
 *                                    FANG as low word)
 *   FANG        agc_sp_t        O    2^7 N/10^4        agc_guid_state()->csm
 *   EMDOT       agc_sp_t        O    2^3 kg/cs         agc_csm_dap_com()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dp_from_double    engineering value -> DP at a scale
 *   agc_integ_perm_set    permanent state vector store
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   sim_pad_load returns AGC_STATUS_NOT_FOUND when the file cannot be
 *   opened and AGC_STATUS_BAD_ARG for an unknown keyword, a malformed
 *   line or a value out of range (reported with its line number); the
 *   harness then refuses to start.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called once, after agc_power_on and before GOJAM (main.c). At most
 *   PAD_LINES_MAX lines are read.
 *
 * NOTES:
 *   yaAGC starts from a core-image file; the C translation keeps every
 *   erasable in its owner's typed storage, which V21N01 / V71 cannot
 *   reach (src/core/ecadr.c DEVIATION). One item per line, `#` starts a
 *   comment:
 *     refsmmat m11 m12 m13 m21 m22 m23 m31 m32 m33
 *         REFSMMAT rows (unit vectors), stored at scale 2^1 (state.h)
 *     flagword N OCTAL
 *         FLAGWRDn |= OCTAL (e.g. `flagword 3 10000` = REFSMFLG)
 *     sv csm|lm X Y Z VX VY VZ [T]
 *         permanent state vector, earth sphere, m and m/s in the
 *         reference frame, at time T s (default 0): RRECT = RCV at 2^29 m,
 *         VRECT = VCV at 2^7 m/cs, TET at 2^28 cs, TDELTAV = TNUV = TC =
 *         XKEP = 0 (a freshly rectified state).
 *     sps_impulse K1 K2 K3 F MDOT
 *         CSM only: the P40 pad loads S40.13 TIMEBURN reads (P40-P47
 *         C p.726, ERASABLE_ASSIGNMENTS C p.87) and EMDOT (C p.68):
 *         EK1VAL = K1 (N s, the 1-s SPS impulse) and EK2VAL = K2 (N s,
 *         signed) at 2^23 N-s/100, EK3VAL = K3 (N, SPS thrust) at
 *         2^9 N/10^4, FANG = F (N, the thrust TEMP2 uses) at 2^7 N/10^4,
 *         EMDOT = MDOT (kg/s, SPS flow) at 2^3 kg/cs. EK3VAL is one word
 *         followed by FANG, and S40.131's DDV K3VAL reads the two as a DP
 *         divisor, so the C's DP EK3VAL is stored as (EK3VAL, FANG).
 *   The same file is accepted as yaAGC's --resume core file (the
 *   launcher's "custom core file" start-up option): sim_pad_is_pad()
 *   tells a pad file from a yaAGC core dump, which the typed C erasable
 *   cannot take.
 *
 * ALGORITHM:
 *   none (line parser)
 **********************************************************************/
#include <stdio.h>
#include <string.h>

#include "agc/core/assertion.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/word.h"
#include "agc/csm/dap.h"
#include "agc/guidance/guidance.h"
#include "agc/nav/integration.h"
#include "internal.h"

#define PAD_LINE        512   /* longest line read                    */
#define PAD_LINES_MAX   256   /* bound on lines read                  */
#define PAD_KEY         32    /* keyword buffer                       */
#define PAD_MAT_N       9     /* REFSMMAT components                  */
#define PAD_SCALE_REFSM 1     /* REFSMMAT 'SCALE OF B1'               */
#define PAD_UNIT_MAX    1.0   /* |unit-vector component|              */
#define PAD_SCALE_R     29    /* RRECT earth sphere, 2^29 m           */
#define PAD_SCALE_V     7     /* VRECT earth sphere, 2^7 m/cs         */
#define PAD_SCALE_T     28    /* TET, 2^28 cs                         */
#define PAD_R_MAX       5.0e8 /* m, < 2^29 m                          */
#define PAD_V_MAX       1.2e4 /* m/s, < 2^7 m/cs                      */
#define PAD_T_MAX       2.6e6 /* s, < 2^28 cs                         */
#define PAD_CS_PER_S    100.0 /* centiseconds per second              */
#define PAD_SV_FIELDS   7     /* vehicle and six components, at least */
#define PAD_SPS_FIELDS  5     /* K1 K2 K3 F MDOT                      */
#define PAD_SCALE_EK    23    /* EK1VAL, EK2VAL: 2^23 N-s/100         */
#define PAD_SCALE_EK3   9     /* EK3VAL: 2^9 N/10^4                   */
#define PAD_SCALE_FANG  7     /* FANG: 2^7 N/10^4 (m.newton)          */
#define PAD_SCALE_EMDOT 3     /* EMDOT: 2^3 kg/cs                     */
#define PAD_NS_UNIT     100.0 /* N s per EK1VAL/EK2VAL unit           */
#define PAD_N_UNIT      1.0e4 /* N per m.newton (EK3VAL, FANG)        */
#define PAD_EK_MAX      8.3e8 /* |K1|, |K2| N s, < 2^23 x 100         */
#define PAD_EK3_MAX     5.1e6 /* K3 N, < 2^9 x 10^4                   */
#define PAD_FANG_MAX    1.2e6 /* F N, < 2^7 x 10^4                    */
#define PAD_EMDOT_MAX   790.0 /* MDOT kg/s, < 2^3 kg/cs               */

/**********************************************************************
 * AGC: none -- `refsmmat`: nine unit-vector components into REFSMMAT.
 *
 * ARGUMENTS:
 *   args  const char *  I  the text after the keyword
 *
 * RETURNS: bool -- false for a missing value or |m| > 1 (also NaN)
 **********************************************************************/
static bool pad_refsmmat(const char *args)
{
    double    m[PAD_MAT_N];                                                                                                                         /* components, unit vectors */
    agc_mat_t r;                                                                                                                                    /* REFSMMAT, 2^1 */
    bool      ok = sscanf(args, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8]) == PAD_MAT_N; /* nine values */

    for (int i = 0; i < PAD_MAT_N && ok; i++)
    {
        ok                    = m[i] <= PAD_UNIT_MAX && m[i] >= -PAD_UNIT_MAX;        /* false for NaN too */
        r.row[i / 3].c[i % 3] = agc_dp_from_double(ok ? m[i] : 0.0, PAD_SCALE_REFSM); /* 0 when refused */
    }

    if (ok)
    {
        agc_erase_set_refsmmat(r);
    }

    return (ok);
}

/**********************************************************************
 * AGC: none -- `flagword N OCTAL`: OR the bits into FLAGWRDn.
 *
 * ARGUMENTS:
 *   args  const char *  I  the text after the keyword
 *
 * RETURNS: bool -- false for a malformed line, a flagword the vehicle
 *          does not have, or bits over 15
 **********************************************************************/
static bool pad_flagword(const char *args)
{
    int          word  = -1;                                                                                             /* flagword number */
    unsigned int bits  = 0;                                                                                              /* bits to set */
    int          words = (agc_state_vehicle() == AGC_VEHICLE_LM) ? AGC_FLAGWORDS_LM : AGC_FLAGWORDS_CSM;                 /* flagwords of the vehicle */
    bool         ok    = sscanf(args, "%d %o", &word, &bits) == 2 && word >= 0 && word < words && bits <= SIM_WORD_MASK; /* well formed */

    if (ok)
    {
        agc_flagword_set(word, (uint16_t) (agc_flagword_get(word) | (uint16_t) bits));
    }

    return (ok);
}

/**********************************************************************
 * AGC: none -- `sv csm|lm X Y Z VX VY VZ [T]`: a rectified earth-sphere
 * permanent state vector.
 *
 * ARGUMENTS:
 *   args  const char *  I  the text after the keyword
 *
 * RETURNS: bool -- false for a malformed line or a value out of range
 *          (also NaN), or when the store refuses it
 **********************************************************************/
static bool pad_state_vector(const char *args)
{
    char           veh[8];                                                                                                       /* csm or lm */
    double         x[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                                                                   /* m, m/s, s */
    int            n    = sscanf(args, "%7s %lf %lf %lf %lf %lf %lf %lf", veh, &x[0], &x[1], &x[2], &x[3], &x[4], &x[5], &x[6]); /* fields read */
    bool           lm   = n >= 1 && strcmp(veh, "lm") == 0;                                                                      /* LM state vector */
    bool           ok   = n >= PAD_SV_FIELDS && (lm || strcmp(veh, "csm") == 0) && x[6] >= 0.0 && x[6] <= PAD_T_MAX;             /* well formed */
    agc_integ_sv_t sv   = {0};                                                                                                   /* RRECT 2^29 m, VRECT 2^7 m/cs, TET 2^28 cs */

    for (int i = 0; i < 3 && ok; i++)
    {
        ok            = x[i] <= PAD_R_MAX && x[i] >= -PAD_R_MAX && x[i + 3] <= PAD_V_MAX && x[i + 3] >= -PAD_V_MAX; /* false for NaN too */
        sv.rrect.c[i] = agc_dp_from_double(ok ? x[i] : 0.0, PAD_SCALE_R);                                           /* 0 when refused */
        sv.vrect.c[i] = agc_dp_from_double(ok ? x[i + 3] / PAD_CS_PER_S : 0.0, PAD_SCALE_V);                        /* 0 when refused */
    }

    sv.rcv = sv.rrect;
    sv.vcv = sv.vrect;
    sv.tet = agc_dp_from_double(ok ? x[6] * PAD_CS_PER_S : 0.0, PAD_SCALE_T); /* 0 when refused */

    if (ok)
    {
        agc_integ_perm_set(lm ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM, &sv); /* vehicle from the line */
    }

    return (ok);
}

/**********************************************************************
 * AGC: none -- `sps_impulse K1 K2 K3 F MDOT`: the CSM's P40 pad loads
 * EK1VAL, EK2VAL, EK3VAL, FANG (S40.13 TIMEBURN, P40-P47 C pp.726-727)
 * and EMDOT, in N s, N s, N, N and kg/s.
 *
 * ARGUMENTS:
 *   args  const char *  I  the text after the keyword
 *
 * RETURNS: bool -- false on the LM, for a malformed line, or a value out
 *          of its erasable's range (also NaN; K1, K3, F, MDOT >= 0)
 **********************************************************************/
static bool pad_sps_impulse(const char *args)
{
    double x[PAD_SPS_FIELDS] = {0.0, 0.0, 0.0, 0.0, 0.0};                                                                                                          /* K1 K2 K3 F MDOT */
    bool   ok                = agc_state_vehicle() == AGC_VEHICLE_CSM && sscanf(args, "%lf %lf %lf %lf %lf", &x[0], &x[1], &x[2], &x[3], &x[4]) == PAD_SPS_FIELDS; /* CSM, well formed */

    ok = ok && x[0] >= 0.0 && x[0] <= PAD_EK_MAX && x[1] >= -PAD_EK_MAX && x[1] <= PAD_EK_MAX; /* false for NaN too */
    ok = ok && x[2] >= 0.0 && x[2] <= PAD_EK3_MAX && x[3] >= 0.0 && x[3] <= PAD_FANG_MAX && x[4] >= 0.0 && x[4] <= PAD_EMDOT_MAX;

    if (ok)
    {
        agc_guid_csm_t *c    = &agc_guid_state()->csm;                                /* EK1VAL..FANG */
        agc_sp_t        fang = agc_sp_from_double(x[3] / PAD_N_UNIT, PAD_SCALE_FANG); /* m.newton 2^7 */

        c->ek1val                = agc_dp_from_double(x[0] / PAD_NS_UNIT, PAD_SCALE_EK);                    /* N-s/100 2^23 */
        c->ek2val                = agc_dp_from_double(x[1] / PAD_NS_UNIT, PAD_SCALE_EK);                    /* N-s/100 2^23 */
        c->ek3val                = agc_dp_pack(agc_sp_from_double(x[2] / PAD_N_UNIT, PAD_SCALE_EK3), fang); /* (EK3VAL, FANG) as DDV reads them */
        c->fang                  = fang;
        agc_csm_dap_com()->emdot = agc_sp_from_double(x[4] / PAD_CS_PER_S, PAD_SCALE_EMDOT);                /* kg/cs 2^3 */
    }

    return (ok);
}

/**********************************************************************
 * AGC: none -- the first word of a line, after cutting a `#` comment.
 *
 * ARGUMENTS:
 *   buf   char *  I/O  the line; the comment is cut off in place
 *   key   char *  O    the keyword, PAD_KEY bytes
 *   used  int *   O    characters up to the end of the keyword
 *
 * RETURNS: bool -- false for a blank or comment line
 **********************************************************************/
static bool pad_keyword(char *buf, char *key, int *used)
{
    char *hash = strchr(buf, '#'); /* comment start */

    if (hash != NULL)
    {
        *hash = '\0';
    }

    return (sscanf(buf, "%31s%n", key, used) == 1);
}

/**********************************************************************
 * AGC: none -- one pad line.
 *
 * ARGUMENTS:
 *   buf  char *  I/O  the line (the comment is cut off in place)
 *
 * RETURNS: bool -- true when the line is blank, a comment, or applied
 **********************************************************************/
static bool pad_line(char *buf)
{
    char key[PAD_KEY]; /* keyword */
    int  used = 0;     /* characters up to the end of the keyword */
    bool ok   = true;  /* blank or applied */

    if (!pad_keyword(buf, key, &used))
    {
        /* blank or comment */
    }
    else if (strcmp(key, "refsmmat") == 0)
    {
        ok = pad_refsmmat(&buf[used]);
    }
    else if (strcmp(key, "flagword") == 0)
    {
        ok = pad_flagword(&buf[used]);
    }
    else if (strcmp(key, "sv") == 0)
    {
        ok = pad_state_vector(&buf[used]);
    }
    else if (strcmp(key, "sps_impulse") == 0)
    {
        ok = pad_sps_impulse(&buf[used]);
    }
    else
    {
        ok = false; /* unknown keyword */
    }

    return (ok);
}

/**********************************************************************
 * AGC: none -- is the file at `path` a pad file? (its first item is a
 * pad keyword; a yaAGC core dump starts with octal numbers instead)
 *
 * ARGUMENTS:
 *   path  const char *  I  file
 *
 * RETURNS: bool -- true for a pad file; false otherwise or when the file
 *          cannot be read
 **********************************************************************/
bool sim_pad_is_pad(const char *path)
{
    char  buf[PAD_LINE];           /* one line */
    char  key[PAD_KEY];            /* keyword */
    int   used = 0;                /* end of the keyword */
    bool  pad  = false;            /* first item is a pad keyword */
    bool  seen = false;            /* an item was found */
    FILE *f    = fopen(path, "r"); /* the file */

    for (int i = 1; i <= PAD_LINES_MAX && f != NULL && !seen && fgets(buf, (int) sizeof (buf), f) != NULL; i++)
    {
        seen = pad_keyword(buf, key, &used);
        pad  = seen && (strcmp(key, "refsmmat") == 0 || strcmp(key, "flagword") == 0 || strcmp(key, "sv") == 0 || strcmp(key, "sps_impulse") == 0);
    }

    if (f != NULL)
    {
        (void) fclose(f); /* stdlib: read-only file, nothing to lose */
    }

    return (pad);
}

/**********************************************************************
 * AGC: none -- load the pad file at `path`.
 *
 * ARGUMENTS:
 *   path  const char *  I  file
 *
 * RETURNS: agc_status_t -- OK; NOT_FOUND when it cannot be opened;
 *          BAD_ARG for a line not understood (reported)
 **********************************************************************/
agc_status_t sim_pad_load(const char *path)
{
    char  buf[PAD_LINE];           /* one line */
    bool  ok   = true;             /* every line applied */
    bool  more = true;             /* lines left */
    FILE *f    = fopen(path, "r"); /* the file */

    if (f == NULL)
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    for (int i = 1; i <= PAD_LINES_MAX && ok && more; i++)
    {
        more = fgets(buf, (int) sizeof (buf), f) != NULL;
        ok   = !more || pad_line(buf);

        if (!ok)
        {
            (void) fprintf(stderr, "agc_sim: pad %s line %d not understood\n", path, i); /* stdlib diagnostic */
        }
    }

    (void) fclose(f); /* stdlib: read-only file, nothing to lose */

    return (ok ? AGC_STATUS_OK : AGC_STATUS_BAD_ARG); /* BAD_ARG: a line refused */
}
