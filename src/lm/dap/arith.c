/*
 * arith.c -- word-level AGC arithmetic used by the LM DAP.
 *
 * The DAP is basic (machine) code, not interpretive: its results depend on
 * MP leaving a sign-agreeing A,L pair, DV truncating, DAS carrying between
 * words and ADS/TS storing the overflow-corrected word. These helpers model
 * those instructions on ones'-complement values held in agc_sp_t (the
 * value, not the bit pattern; -0 is folded into +0).
 *
 * Sources: the instruction descriptions used throughout Luminary099
 * (P-AXIS_RCS_AUTOPILOT.agc p.1422 1STOTWOS / OVERSUB, AOSTASK_AND_AOSJOB.agc
 * p.1492-1493 DVOVSUB, FIXED_FIXED_CONSTANT_POOL LIMITS).
 */
#include "agc/core/assert.h"
#include "internal.h"

#define ARITH_WORD   16384  /* 2^14: one major-word unit in minor units */
#define ARITH_MAXMAG 16383  /* POSMAX                                    */
#define ARITH_TWOS   0x8000 /* 2^15: 15-bit two's complement modulus    */
#define ARITH_MASK15 0x7FFF
#define ARITH_SIGN15 0x4000

/* hi * 2^14 + lo: the value of a word pair in minor-word units.           */
int32_t lm_dap_dpval(lm_dp_pair_t x)
{
    if (!AGC_ASSERT(x.hi >= -ARITH_MAXMAG && x.hi <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(x.lo >= -ARITH_MAXMAG && x.lo <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return ((int32_t) x.hi * ARITH_WORD + (int32_t) x.lo);
}

/* Split a value into a sign-agreeing pair (what MP and DV leave).         */
lm_dp_pair_t lm_dap_dpmake(int32_t v)
{
    lm_dp_pair_t r;
    int32_t      hi = v / ARITH_WORD; /* C truncates toward zero      */

    if (!AGC_ASSERT(hi >= -ARITH_MAXMAG && hi <= ARITH_MAXMAG))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    if (hi > ARITH_MAXMAG)
    {
        hi = ARITH_MAXMAG;
    }
    else if (hi < -ARITH_MAXMAG)
    {
        hi = -ARITH_MAXMAG;
    }

    r.hi = (agc_sp_t) hi;
    r.lo = (agc_sp_t) (v - hi * ARITH_WORD);

    if (!AGC_ASSERT(r.lo > -ARITH_WORD && r.lo < ARITH_WORD))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    return (r);
}

/* MP: A,L = a * b (14 x 14 bits, exact), both words with the product sign. */
lm_dp_pair_t lm_dap_mp(agc_sp_t a, agc_sp_t b)
{
    if (!AGC_ASSERT(a >= -ARITH_MAXMAG && a <= ARITH_MAXMAG))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(b >= -ARITH_MAXMAG && b <= ARITH_MAXMAG))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    return (lm_dap_dpmake((int32_t) a * (int32_t) b));
}

agc_sp_t lm_dap_mphi(agc_sp_t a, agc_sp_t b)
{
    lm_dp_pair_t p = lm_dap_mp(a, b);

    if (!AGC_ASSERT(p.hi >= -ARITH_MAXMAG && p.hi <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return (p.hi);
}

/* DV: (A,L) / d -> quotient in A, remainder (dividend sign) in L. The AGC
 * requires |dividend| < |d| (as fractions); an improper division gives an
 * undefined result there, here it is limited to +-POSMAX and reported.
 * A dividend exactly equal in magnitude to the divisor (A = +-d, L = 0)
 * is defined: A = POSMAX (signs agree) or NEGMAX, L = the dividend's
 * major word. 1/ACCONT's INVERT relies on it (BIT9 / AMIN = 1/.03).     */
lm_dp_pair_t lm_dap_dvqr(lm_dp_pair_t n, agc_sp_t d)
{
    lm_dp_pair_t r   = {0, 0};
    int32_t      num = lm_dap_dpval(n);

    if (!AGC_ASSERT(d != 0))
    {
        r.hi = (num >= 0) ? ARITH_MAXMAG : -ARITH_MAXMAG;

        return (r);
    }

    if (num == (int32_t) d * ARITH_WORD || num == -(int32_t) d * ARITH_WORD)
    { /* |dividend| = |divisor| */
        r.hi = ((num > 0) == (d > 0)) ? ARITH_MAXMAG : -ARITH_MAXMAG;
        r.lo = (agc_sp_t) (num / ARITH_WORD);

        return (r);
    }

    int32_t q   = num / (int32_t) d;
    int32_t rem = num - q * (int32_t) d;

    if (!AGC_ASSERT(q >= -ARITH_MAXMAG && q <= ARITH_MAXMAG))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    r.hi = lm_dap_sat(q);
    r.lo = (agc_sp_t) rem;

    return (r);
}

agc_sp_t lm_dap_dv(lm_dp_pair_t n, agc_sp_t d)
{
    lm_dp_pair_t r = lm_dap_dvqr(n, d);

    if (!AGC_ASSERT(r.hi >= -ARITH_MAXMAG && r.hi <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return (r.hi);
}

/* The word stored by TS/XCH/ADS when A holds an overflowed sum: the
 * overflow-corrected value (sum -+ 2^14).                                 */
agc_sp_t lm_dap_wrap(int32_t v)
{
    if (!AGC_ASSERT(v >= -3 * ARITH_MAXMAG && v <= 3 * ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    for (int i = 0; i < 2; i++)
    { /* a 3-term sum may need two  */
        if (v > ARITH_MAXMAG)
        {
            v -= ARITH_WORD;
        }
        else if (v < -ARITH_MAXMAG)
        {
            v += ARITH_WORD;
        }
    }

    if (!AGC_ASSERT(v >= -ARITH_MAXMAG && v <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return (lm_dap_sat(v));
}

/* OVERSUB / "INDEX A, CS LIMITS": an overflowed A becomes POSMAX/NEGMAX.  */
agc_sp_t lm_dap_sat(int32_t v)
{
    agc_sp_t r;

    if (v > ARITH_MAXMAG)
    {
        r = ARITH_MAXMAG;
    }
    else if (v < -ARITH_MAXMAG)
    {
        r = -ARITH_MAXMAG;
    }
    else
    {
        r = (agc_sp_t) v;
    }

    if (!AGC_ASSERT(r >= -ARITH_MAXMAG && r <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/* "TS A" / "TS 7" skip test: does A hold an overflowed value?              */
bool lm_dap_ovf(int32_t v)
{
    if (!AGC_ASSERT(v >= -4 * ARITH_WORD && v <= 4 * ARITH_WORD))
    {
        return (false);
    }

    return ((v > ARITH_MAXMAG) || (v < -ARITH_MAXMAG));
}

/* DAS: (x) + (y) with the minor-word carry; the major word is stored
 * overflow-corrected (the AGC leaves the overflow indication in A).      */
lm_dp_pair_t lm_dap_das(lm_dp_pair_t x, lm_dp_pair_t y)
{
    int32_t lo    = (int32_t) x.lo + (int32_t) y.lo;
    int32_t carry = 0;

    if (lo > ARITH_MAXMAG)
    {
        lo   -= ARITH_WORD;
        carry = 1;
    }
    else if (lo < -ARITH_MAXMAG)
    {
        lo   += ARITH_WORD;
        carry = -1;
    }

    lm_dp_pair_t r;

    r.hi = lm_dap_wrap((int32_t) x.hi + (int32_t) y.hi + carry);
    r.lo = (agc_sp_t) lo;

    if (!AGC_ASSERT(r.lo >= -ARITH_MAXMAG && r.lo <= ARITH_MAXMAG))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    return (r);
}

/* DDOUBL (DAS A).                                                          */
lm_dp_pair_t lm_dap_ddoubl(lm_dp_pair_t x)
{
    if (!AGC_ASSERT(x.hi >= -ARITH_MAXMAG && x.hi <= ARITH_MAXMAG))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(x.lo >= -ARITH_MAXMAG && x.lo <= ARITH_MAXMAG))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    return (lm_dap_das(x, x));
}

/* MSU: a - b of two 15-bit two's-complement words (CDU angles at pi),
 * result as a ones'-complement value. A difference of exactly half a
 * revolution (two's 40000) comes out +POSMAX, as on the AGC.             */
agc_sp_t lm_dap_msu(uint16_t a, uint16_t b)
{
    if (!AGC_ASSERT(a <= ARITH_MASK15))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(b <= ARITH_MASK15))
    {
        return (0); /* no valid result */
    }

    int32_t d = ((int32_t) a - (int32_t) b) & ARITH_MASK15;
    int32_t v;

    if (d < ARITH_SIGN15)
    {
        v = d;
    }
    else if (d == ARITH_SIGN15)
    {
        v = ARITH_MAXMAG; /* 40000 - 1 = 37777               */
    }
    else
    {
        v = d - ARITH_TWOS;
    }

    return ((agc_sp_t) v);
}

/* 1STOTWOS (p.1422): a ones'-complement value back to a 15-bit two's-
 * complement word (CCS A, AD ONE / CS A).                                 */
uint16_t lm_dap_ones_to_twos(int32_t v)
{
    if (!AGC_ASSERT(v >= -ARITH_MAXMAG && v <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    int32_t w = (v >= 0) ? v : (v + ARITH_TWOS);

    if (!AGC_ASSERT(w >= 0 && w <= ARITH_MASK15))
    {
        return (0); /* no valid result */
    }

    return ((uint16_t) w);
}

/* CCS: the diminished absolute value |v| - 1 (0 for 0).                    */
agc_sp_t lm_dap_ccs_dim(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= -ARITH_MAXMAG && v <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    agc_sp_t r =
        (v > 0) ? (agc_sp_t) (v - 1) : ((v < 0) ? (agc_sp_t) (-v - 1) : 0);

    if (!AGC_ASSERT(r >= 0))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/* DIM: decrement the magnitude, stopping at zero.                          */
agc_sp_t lm_dap_dim(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= -ARITH_MAXMAG && v <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    agc_sp_t r =
        (v > 0) ? (agc_sp_t) (v - 1) : ((v < 0) ? (agc_sp_t) (v + 1) : 0);

    if (!AGC_ASSERT(r >= -ARITH_MAXMAG + 1 || r == 0))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/* DVOVSUB (AOSTASK_AND_AOSJOB.agc pp.1492-1493): DV with overflow
 * protection. Zero divisor -> POSMAX; |dividend major| >= |divisor| ->
 * POSMAX if the quotient would be positive, NEGMAX if negative.          */
agc_sp_t lm_dap_dvovsub(lm_dp_pair_t n, agc_sp_t d)
{
    if (!AGC_ASSERT(n.hi >= -ARITH_MAXMAG && n.hi <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    if (d == 0)
    {
        return (ARITH_MAXMAG); /* MAXPLUS            */
    }

    int32_t ad = (d > 0) ? d : -(int32_t) d;
    int32_t ah = (n.hi >= 0) ? n.hi : -(int32_t) n.hi;

    if (ad - ah <= 0)
    { /* MAKEMAX            */
        bool pos = (n.hi > 0) == (d > 0);

        return (pos ? ARITH_MAXMAG : (agc_sp_t) -ARITH_MAXMAG);
    }

    agc_sp_t q = lm_dap_dv(n, d);

    if (!AGC_ASSERT(q >= -ARITH_MAXMAG && q <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return (q);
}
