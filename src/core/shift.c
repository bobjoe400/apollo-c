/**********************************************************************
 * FILE NAME: shift.c
 *
 * PURPOSE:
 *   Interpretive shifts at word level: right shifts with and without
 *   rounding (MPACSHR, MPACSRND, VSHRRND, GENSCR, VRIGHT2), the TP left
 *   shift TSSL and the normalising shift TSLC (NORM).
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc pp.1152-1161 ("GENERAL SHIFT": MPACSHR,
 *   MPACSRND, VSHRRND, TSSL, TSLC, VSSR, VSSL, VRIGHT2, GENSCR)
 *   Luminary099 INTERPRETER.agc pp.1047-1056 (identical)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name            Description
 *   --------------  ----------------------------------------------
 *   agc_w_mp        MP on words (mpac.c)
 *   agc_w_das       DAS on word pairs (mpac.c)
 *   agc_w_roundsub  ROUNDSUB (mpac.c)
 *   agc_w_tp_shl1   one TP left shift (mpac.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; TSSL sets OVFIND on each overflowing step.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Shift counts >= 1 for right shifts, >= 0 for TSSL; loops bounded by
 *   SHIFT_MAX_WORDS and SHIFT_MAX_LEFT.
 *
 * ALGORITHM:
 *   A right shift by n <= 13 multiplies each word by 2^(14-n) (the BIT14
 *   table entry selected with INDEX) and keeps major/minor halves: every
 *   word is truncated toward zero separately. Counts of 14 and more first
 *   move whole words (losing MPAC+2), as GENSCR does.
 **********************************************************************/
#include "agc/core/assert.h"
#include "mpac.h"

#define SHIFT_MAX_LEFT  43 /* 42 TP bits: further shifts leave 0     */
#define SHIFT_MAX_WORDS 4  /* GENSCR word-shift passes (count < 127) */

/**********************************************************************
 * MPACSHR (INTERPRETER.agc pp.1152-1153), or MPACSRND / VSHR2 (p.1153)
 * when rounding -- shift the TP MPAC right 1..14 places.
 *
 * ARGUMENTS:
 *   m      agc_w3_t  I  TP MPAC
 *   n      int       I  shift count, 1..14
 *   round  bool      I  true: MPACSRND (round half away from zero into
 *                       MPAC+1, MPAC+2 cleared); false: truncate
 *
 * RETURNS: agc_w3_t -- the shifted MPAC (scale +n)
 **********************************************************************/
agc_w3_t agc_w_mpacshr(agc_w3_t m, int n, bool round)
{
    int32_t  t;     /* BIT14 indexed by n-1: 2^(14-n), unscaled  */
    agc_w2_t lo2;   /* MPAC+1, MPAC+2 of the truncating shift    */
    agc_w2_t p;     /* MPAC, MPAC+1 of the rounding shift        */
    int      o = 0; /* DAS overflow / rounding carry, +-1 or 0   */

    if (!AGC_ASSERT((n >= 1) && (n <= 14)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    t = (int32_t) 1 << (14 - n);

    if (!AGC_ASSERT(agc_w_ok(m.m0) && agc_w_ok(m.m1) && agc_w_ok(m.m2)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    if (!round)
    {
        lo2.hi = agc_w_mp(t, m.m0).lo;                  /* DXCH MPAC        */
        lo2.lo = agc_w_mp(t, m.m2).hi;                  /* TS MPAC +2       */
        lo2    = agc_w_das(lo2, agc_w_mp(t, m.m1), &o); /* DAS MPAC +1      */
        m.m0   = agc_w_mp(t, m.m0).hi;
        m.m1   = lo2.hi;
        m.m2   = lo2.lo;

        return (m);
    }

    /* MPACSRND: residual = major(m2 t) + minor(m1 t); DOUBLE; TS MPAC +2.  */
    p.hi = 0;
    p.lo = agc_w_mp(t, m.m1).hi;                /* trial minor part */
    o    = agc_w_ovf(2 * (agc_w_mp(t, m.m2).hi + agc_w_mp(t, m.m1).lo));
    p.lo += o;                                  /* ADS MPAC +1      */
    p    = agc_w_das(p, agc_w_mp(t, m.m0), &o); /* DAS MPAC         */
    m.m0 = p.hi;
    m.m1 = p.lo;
    m.m2 = 0;

    return (m);
} /* agc_w_mpacshr */

/**********************************************************************
 * VSHRRND (INTERPRETER.agc p.1153) -- one DP vector component shifted
 * right n places and rounded.
 *
 * ARGUMENTS:
 *   p  agc_w2_t  I  component pair
 *   n  int       I  shift count, 1..14
 *
 * RETURNS: agc_w2_t -- the rounded component (scale +n)
 **********************************************************************/
agc_w2_t agc_w_vshrrnd(agc_w2_t p, int n)
{
    agc_w3_t m; /* MPAC, MPAC+1 of the component (MPAC+2 unused) */

    if (!AGC_ASSERT(agc_w_ok(p.hi) && agc_w_ok(p.lo)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT((n >= 1) && (n <= 14)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    m.m0 = p.hi;
    m.m1 = p.lo;
    m.m2 = 0; /* not referenced   */

    return (agc_w3_top(agc_w_mpacshr(m, n, true)));
}

/**********************************************************************
 * GENSCR + DOSSHFT (INTERPRETER.agc p.1160) -- scalar SR / SRR by any
 * count: whole words move first (14 places per pass), then MPACSHR.
 *
 * ARGUMENTS:
 *   m      agc_w3_t  I  TP MPAC
 *   n      int       I  shift count, >= 1
 *   round  bool      I  true for SRR (rounds), false for SR
 *
 * RETURNS: agc_w3_t -- the shifted MPAC (scale +n); 0 beyond 55 places
 **********************************************************************/
agc_w3_t agc_w_genscr(agc_w3_t m, int n, bool round)
{
    int       pass;       /* whole-word passes done */
    int       left   = n; /* MPTEMP + 1       */ /* still to shift */
    agc_ovf_t unused = 0; /* ROUNDSUB OVFIND (cannot overflow here) */

    if (!AGC_ASSERT(n >= 1))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    for (pass = 0; pass < SHIFT_MAX_WORDS; pass++)
    {
        if (left <= 13)
        { /* BZMF DOSSHFT     */
            return (agc_w_mpacshr(m, left, round));
        }

        m.m2 = m.m1; /* XCH x4: right 14 */
        m.m1 = m.m0;
        m.m0 = 0;
        left -= 14;

        if (left == 0)
        { /* MPTEMP = -0      */
            return (round ? agc_w_roundsub(m, &unused) : m); /* SRR rounds */
        }
    }

    if (!AGC_ASSERT(m.m0 == 0))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    m.m1 = 0;
    m.m2 = 0;

    return (m);
}

/**********************************************************************
 * RIGHT / VRIGHT2 + VSSR (INTERPRETER.agc p.1159) -- one vector
 * component VSR n: each whole-word step rounds (SETROUND), then VSHRRND.
 *
 * ARGUMENTS:
 *   p  agc_w2_t  I  component pair
 *   n  int       I  shift count, >= 1
 *
 * RETURNS: agc_w2_t -- the shifted, rounded component (scale +n)
 **********************************************************************/
agc_w2_t agc_w_vright(agc_w2_t p, int n)
{
    int      pass;          /* whole-word passes done */
    int      left = n;      /* places still to shift */
    int      o    = 0;      /* SETROUND carry, then DAS overflow */
    agc_w2_t rnd  = {0, 0}; /* rounding carry as a pair */

    if (!AGC_ASSERT(n >= 1))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(agc_w_ok(p.hi) && agc_w_ok(p.lo)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    for (pass = 0; pass < SHIFT_MAX_WORDS; pass++)
    {
        if (left <= 13)
        { /* BZMF VSSR        */
            return (agc_w_vshrrnd(p, left));
        }

        o      = agc_w_ovf(2 * p.lo);   /* SETROUND         */
        rnd.lo = o;
        p.lo   = p.hi;                  /* XCH MPAC; +1     */
        p.hi   = 0;
        p      = agc_w_das(p, rnd, &o); /* DAS MPAC         */
        left   -= 14;

        if (left == 0)
        { /* MPTEMP = -0      */
            return (p);
        }
    }

    p.hi = 0;
    p.lo = 0;

    return (p);
}

/**********************************************************************
 * TSSL (INTERPRETER.agc p.1154) -- shift the TP MPAC left n places one at
 * a time; OVFIND is set on each overflowing step and the
 * overflow-corrected (wrapped) result is kept, as the AGC does.
 *
 * ARGUMENTS:
 *   m       agc_w3_t     I  TP MPAC
 *   n       int          I  shift count, >= 0
 *   ovfind  agc_ovf_t *  O  OVFIND, set to +-1 on overflow
 *
 * RETURNS: agc_w3_t -- the shifted MPAC (scale -n)
 **********************************************************************/
agc_w3_t agc_w_tssl(agc_w3_t m, int n, agc_ovf_t *ovfind)
{
    int k;     /* steps done */
    int o = 0; /* overflow of one step, +-1 or 0 */

    if (!AGC_ASSERT(n >= 0))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    for (k = 0; (k < n) && (k < SHIFT_MAX_LEFT); k++)
    {
        m = agc_w_tp_shl1(m, &o);

        if ((o != 0) && (ovfind != 0))
        {
            *ovfind = (agc_ovf_t) o; /* TS OVFIND        */
        }
    }

    if (n > SHIFT_MAX_LEFT)
    { /* all bits gone    */
        m.m0 = 0;
        m.m1 = 0;
        m.m2 = 0;
    }

    return (m);
}

/**********************************************************************
 * TSLC / NORM (INTERPRETER.agc p.1157) -- shift left until the major
 * word is >= .5 in magnitude; zero gives no shift. TPAGREE comes first
 * (words of disagreeing signs are made to agree, value unchanged).
 *
 * ARGUMENTS:
 *   m      agc_w3_t  I  TP MPAC
 *   count  int *     O  left shifts done, 0..42 (the AGC stores the
 *                       complement in the index register)
 *
 * RETURNS: agc_w3_t -- the normalised MPAC
 **********************************************************************/
agc_w3_t agc_w_tslc(agc_w3_t m, int *count)
{
    int k;     /* steps done */
    int o = 0; /* overflow of one step (never: |m0| < 1/2) */

    *count = 0;

    if ((m.m0 == 0) && (m.m1 == 0) && (m.m2 == 0))
    { /* BRANCH: zero     */
        return (m);
    }

    m = agc_w_tpagree(m); /* TC TPAGREE       */

    for (k = 0; k < SHIFT_MAX_LEFT; k++)
    {
        if ((m.m0 >= AGC_W_HALF) || (m.m0 <= -AGC_W_HALF))
        {
            break; /* DOUBLE; OVSK     */
        }

        m      = agc_w_tp_shl1(m, &o); /* TSLCLOOP         */
        *count += 1;
    }

    if (!AGC_ASSERT(o == 0))
    {
        return (m);
    }

    return (m);
}
