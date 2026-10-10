/*
 * agc_shift.c -- interpretive shift routines at word level
 * (INTERPRETER.agc, Comanche055 pp.1152-1161 "GENERAL SHIFT", TSSL, TSLC,
 * VSSR, VSSL, VRIGHT2, GENSCR; Luminary099 pp.1047-1056).
 *
 * A right shift by n <= 13 multiplies each word by 2^(14-n) (the BIT14 table
 * entry selected with INDEX) and keeps major/minor halves: every word is
 * truncated toward zero separately. Counts of 14 and more first move whole
 * words (losing MPAC+2), as GENSCR does.
 */
#include "agc/core/assert.h"
#include "mpac.h"

#define SHIFT_MAX_LEFT  43 /* 42 TP bits: further shifts leave 0     */
#define SHIFT_MAX_WORDS 4  /* GENSCR word-shift passes (count < 127) */

static int32_t shift_bit(int n) /* BIT14 indexed by n-1: 2^(14-n)          */
{
    if (!AGC_ASSERT((n >= 1) && (n <= 14)))
    {
        return (0); /* no valid result */
    }

    return ((int32_t) 1 << (14 - n));
}

/* MPACSHR (p.1152-1153) or MPACSRND/VSHR2 (p.1153) for 1 <= n <= 14.        */
agc_w3_t agc_w_mpacshr(agc_w3_t m, int n, bool round)
{
    int32_t  t = shift_bit(n);
    agc_w2_t lo2;

    if (!AGC_ASSERT(t > 0))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    agc_w2_t p;
    int      o = 0;

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
    p.hi  = 0;
    p.lo  = agc_w_mp(t, m.m1).hi; /* trial minor part */
    o     = agc_w_ovf(2 * (agc_w_mp(t, m.m2).hi + agc_w_mp(t, m.m1).lo));
    p.lo += o;                                  /* ADS MPAC +1      */
    p     = agc_w_das(p, agc_w_mp(t, m.m0), &o); /* DAS MPAC         */
    m.m0  = p.hi;
    m.m1  = p.lo;
    m.m2  = 0;

    return (m);
}

/* VSHRRND (p.1153): DP vector component, shift right n and round.           */
agc_w2_t agc_w_vshrrnd(agc_w2_t p, int n)
{
    agc_w3_t m;

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

/* GENSCR (p.1160) + DOSSHFT: scalar SR/SRR by any count n >= 1.             */
agc_w3_t agc_w_genscr(agc_w3_t m, int n, bool round)
{
    int       pass;
    int       left   = n; /* MPTEMP + 1       */
    agc_ovf_t unused = 0;

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

        m.m2  = m.m1; /* XCH x4: right 14 */
        m.m1  = m.m0;
        m.m0  = 0;
        left -= 14;

        if (left == 0)
        { /* MPTEMP = -0      */
            return (round ? agc_w_roundsub(m, &unused) : m);
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

/* RIGHT/VRIGHT2 (p.1159) + VSSR: one vector component, VSR n (rounds).      */
agc_w2_t agc_w_vright(agc_w2_t p, int n)
{
    int      pass;
    int      left = n;
    int      o    = 0;
    agc_w2_t rnd  = {0, 0};

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

        o      = agc_w_ovf(2 * p.lo); /* SETROUND         */
        rnd.lo = o;
        p.lo   = p.hi; /* XCH MPAC; +1     */
        p.hi   = 0;
        p      = agc_w_das(p, rnd, &o); /* DAS MPAC         */
        left  -= 14;

        if (left == 0)
        { /* MPTEMP = -0      */
            return (p);
        }
    }

    p.hi = 0;
    p.lo = 0;

    return (p);
}

/* TSSL (p.1154): shift the TP MPAC left n places one at a time; OVFIND is
 * set on each overflowing step and the overflow-corrected (wrapped) result
 * is kept, as the AGC does.                                                 */
agc_w3_t agc_w_tssl(agc_w3_t m, int n, agc_ovf_t *ovfind)
{
    int k;
    int o = 0;

    if (!AGC_ASSERT(ovfind != 0))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

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

/* TSLC / NORM (p.1157): shift left until the major word is >= .5 in
 * magnitude; zero gives no shift. (TPAGREE first: inputs here already have
 * agreeing signs, so it changes nothing.)                                  */
agc_w3_t agc_w_tslc(agc_w3_t m, int *count)
{
    int k;
    int o = 0;

    if (!AGC_ASSERT(count != 0))
    {
        return (m);
    }

    *count = 0;

    if ((m.m0 == 0) && (m.m1 == 0) && (m.m2 == 0))
    { /* BRANCH: zero     */
        return (m);
    }

    for (k = 0; k < SHIFT_MAX_LEFT; k++)
    {
        if ((m.m0 >= AGC_W_HALF) || (m.m0 <= -AGC_W_HALF))
        {
            break; /* DOUBLE; OVSK     */
        }

        m       = agc_w_tp_shl1(m, &o); /* TSLCLOOP         */
        *count += 1;
    }

    if (!AGC_ASSERT(o == 0))
    {
        return (m);
    }

    return (m);
}
