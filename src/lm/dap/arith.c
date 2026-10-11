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

/**********************************************************************
 * hi * 2^14 + lo: the value of a word pair in minor-word units.
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_dpval
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   x         lm_dp_pair_t  I    word pair (major, minor), unscaled
 *
 * RETURNS: int32_t -- hi x 2^14 + lo, minor-word units
 **********************************************************************/
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

/**********************************************************************
 * Split a value into a sign-agreeing pair (what MP and DV leave).
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_dpmake
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  --------------------
 *   v         int32_t  I    value in minor-word units (2^-14 of the major
 *                           word)
 *
 * RETURNS: lm_dp_pair_t -- sign-agreeing pair
 **********************************************************************/
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

/**********************************************************************
 * MP: A,L = a * b (14 x 14 bits, exact), both words with the product sign.
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_mp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   a         agc_sp_t  I    multiplicand (A), unscaled word
 *   b         agc_sp_t  I    multiplier (memory), unscaled word
 *
 * RETURNS: lm_dp_pair_t -- product A,L
 **********************************************************************/
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

/**********************************************************************
 * MP, major word only (CA x / MP y / TS z).  AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_mphi
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   a         agc_sp_t  I    multiplicand (A), unscaled word
 *   b         agc_sp_t  I    multiplier (memory), unscaled word
 *
 * RETURNS: agc_sp_t -- major word of the product
 **********************************************************************/
agc_sp_t lm_dap_mphi(agc_sp_t a, agc_sp_t b)
{
    lm_dp_pair_t p = lm_dap_mp(a, b);

    if (!AGC_ASSERT(p.hi >= -ARITH_MAXMAG && p.hi <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return (p.hi);
}

/**********************************************************************
 * A value as a sign-extended 16-bit ones'-complement register word.
 * AGC: none (helper)
 *
 * FUNCTION NAME: arith_reg16
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  --------------------
 *   v         int32_t  I    value, -POSMAX..POSMAX
 *
 * RETURNS: uint16_t -- register word
 **********************************************************************/
static uint16_t arith_reg16(int32_t v)
{
    uint16_t r = (v >= 0) ? (uint16_t) v : (uint16_t) ~(uint16_t) (-v);

    if (!AGC_ASSERT(v >= -ARITH_MAXMAG && v <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * A 16-bit register word read back as its overflow-corrected value
 * (bit 16 is the sign, bits 14-1 the magnitude; -0 folds to 0).
 * AGC: none (helper)
 *
 * FUNCTION NAME: arith_val16
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   w         uint16_t  I    16-bit register word
 *
 * RETURNS: agc_sp_t -- value of the word
 **********************************************************************/
static agc_sp_t arith_val16(uint16_t w)
{
    agc_sp_t r = ((w & 0x8000u) != 0u) ? (agc_sp_t) -(int32_t) (~w & 037777u)
                                       : (agc_sp_t) (w & 037777u);

    if (!AGC_ASSERT(r >= -ARITH_MAXMAG && r <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * 16-bit ones'-complement add with end-around carry (the adder).
 * AGC: none (helper)
 *
 * FUNCTION NAME: arith_add16
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   a         uint16_t  I    16-bit register word
 *   b         uint16_t  I    16-bit register word
 *
 * RETURNS: uint16_t -- sum word
 **********************************************************************/
static uint16_t arith_add16(uint16_t a, uint16_t b)
{
    uint32_t s = (uint32_t) a + b;

    if ((s & 0x10000u) != 0u)
    {
        s = (s + 1u) & 0xFFFFu;
    }

    if (!AGC_ASSERT(s <= 0xFFFFu))
    {
        return (0); /* no valid result */
    }

    return ((uint16_t) s);
}

/**********************************************************************
 * DV with an improper quotient (|dividend| > |divisor|, or a zero
 * divisor): the AGC hardware's divide sequence, which yields what Savage
 * and Drake call "total nonsense" but deterministically. Register-level
 * model after yaAGC SimulateDV (tools/oracle/yaAGC-src/agc_engine.c),
 * which matches the hardware for these cases.
 * AGC: none (helper)
 *
 * FUNCTION NAME: dv_improper
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   n         lm_dp_pair_t  I    dividend A,L as the registers hold it
 *   d         agc_sp_t      I    divisor word
 *
 * RETURNS: lm_dp_pair_t -- A (quotient word), L (remainder word)
 **********************************************************************/
static lm_dp_pair_t dv_improper(lm_dp_pair_t n, agc_sp_t d)
{
    uint16_t a       = arith_reg16(n.hi);
    uint16_t l       = arith_reg16(n.lo);
    uint16_t div     = arith_reg16(d);
    uint16_t dvdsign = a & 0x8000u;

    if (dvdsign == 0u)
    {
        a = (uint16_t) ~a;              /* -|A|                        */
    }

    if (a == 0xFFFFu)
    {
        dvdsign = l & 0x8000u;          /* A was 0: sign from L        */
    }

    if (dvdsign != 0u)
    {
        l = (uint16_t) ~l;
    }

    l = arith_add16(l, 040000u);

    if ((l & 0140000u) != 040000u)
    {
        a = arith_add16(a, 1u);         /* no positive overflow in L   */
    }

    uint16_t rem     = a;
    uint16_t divsign = div & 0x8000u;

    if (divsign != 0u)
    {
        div = (uint16_t) ~div;          /* |divisor|                   */
    }

    uint16_t qsign = l & 0x8000u;
    uint16_t q     = (uint16_t) (qsign | ((l & 037777u) << 1) | (qsign >> 15));

    for (int i = 0; i < 14; i++)
    {
        q = (uint16_t) (q << 1);
        uint16_t rsign = rem & 0x8000u;

        rem = (uint16_t) (rsign | ((rem & 037777u) << 1));

        if ((q & 0x8000u) == 0u)
        {
            rem |= (uint16_t) (rsign >> 15);
        }

        uint16_t sum = arith_add16(rem, div);

        if ((sum & 0x8000u) != 0u)
        {
            q  |= 1u;
            rem = sum;
        }
    }

    a = (uint16_t) (qsign | (q & 077777u));
    lm_dp_pair_t r;

    r.hi = arith_val16((dvdsign != divsign) ? (uint16_t) ~a : a);
    r.lo = arith_val16((dvdsign != 0u) ? rem : (uint16_t) ~rem);

    if (!AGC_ASSERT(r.hi >= -ARITH_MAXMAG && r.hi <= ARITH_MAXMAG))
    {
        return ((lm_dp_pair_t){0}); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * DV: (A,L) / d -> quotient in A, remainder (dividend sign) in L. The AGC
 * requires |dividend| < |d| (as fractions); an improper division (or a
 * zero divisor) gives the hardware's nonsense result (dv_improper), as
 * 1/ACCS does with the zero erasable before the pad load.
 * A dividend exactly equal in magnitude to the divisor (A = +-d, L = 0)
 * is defined: A = POSMAX (signs agree) or NEGMAX, L = the dividend's
 * major word. 1/ACCONT's INVERT relies on it (BIT9 / AMIN = 1/.03).
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_dvqr
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   n         lm_dp_pair_t  I    dividend, DP pair in A,L
 *   d         agc_sp_t      I    divisor word
 *
 * RETURNS: lm_dp_pair_t -- quotient (A) and remainder (L)
 **********************************************************************/
lm_dp_pair_t lm_dap_dvqr(lm_dp_pair_t n, agc_sp_t d)
{
    lm_dp_pair_t r   = {0, 0};
    int32_t      num = lm_dap_dpval(n);
    lm_dp_pair_t nn  = lm_dap_dpmake(num); /* sign-agreeing form  */
    int32_t      ah  = (nn.hi >= 0) ? nn.hi : -(int32_t) nn.hi;
    int32_t      ad  = (d >= 0) ? d : -(int32_t) d;

    if (num == 0)
    {
        r.hi = (d == 0) ? ARITH_MAXMAG : 0; /* 0/0: max, sign rules  */

        return (r);
    }

    if (ah > ad || (ah == ad && nn.lo != 0))
    {
        return (dv_improper(n, d)); /* the registers as they are */
    }

    if (ah == ad)
    { /* |dividend| = |divisor| */
        r.hi = ((nn.hi > 0) == (d > 0)) ? ARITH_MAXMAG : -ARITH_MAXMAG;
        r.lo = nn.hi;

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

/**********************************************************************
 * DV, quotient only (A).  AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_dv
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   n         lm_dp_pair_t  I    dividend, DP pair in A,L
 *   d         agc_sp_t      I    divisor word
 *
 * RETURNS: agc_sp_t -- quotient (A)
 **********************************************************************/
agc_sp_t lm_dap_dv(lm_dp_pair_t n, agc_sp_t d)
{
    lm_dp_pair_t r = lm_dap_dvqr(n, d);

    if (!AGC_ASSERT(r.hi >= -ARITH_MAXMAG && r.hi <= ARITH_MAXMAG))
    {
        return (0); /* no valid result */
    }

    return (r.hi);
}

/**********************************************************************
 * The word stored by TS/XCH/ADS when A holds an overflowed sum: the
 * overflow-corrected value (sum -+ 2^14).
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_wrap
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  --------------------
 *   v         int32_t  I    unlimited sum, |v| <= 3 POSMAX
 *
 * RETURNS: agc_sp_t -- overflow-corrected word
 **********************************************************************/
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

/**********************************************************************
 * OVERSUB / "INDEX A, CS LIMITS": an overflowed A becomes POSMAX/NEGMAX.
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_sat
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  --------------------
 *   v         int32_t  I    unlimited sum
 *
 * RETURNS: agc_sp_t -- v limited to POSMAX / NEGMAX
 **********************************************************************/
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

/**********************************************************************
 * "TS A" / "TS 7" skip test: does A hold an overflowed value?
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_ovf
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  --------------------
 *   v         int32_t  I    unlimited sum in A
 *
 * RETURNS: bool -- true when v exceeds one word
 **********************************************************************/
bool lm_dap_ovf(int32_t v)
{
    if (!AGC_ASSERT(v >= -4 * ARITH_WORD && v <= 4 * ARITH_WORD))
    {
        return (false);
    }

    return ((v > ARITH_MAXMAG) || (v < -ARITH_MAXMAG));
}

/**********************************************************************
 * DAS: (x) + (y) with the minor-word carry; the major word is stored
 * overflow-corrected (the AGC leaves the overflow indication in A).
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_das
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   x         lm_dp_pair_t  I    augend pair
 *   y         lm_dp_pair_t  I    addend pair
 *
 * RETURNS: lm_dp_pair_t -- sum pair (major word overflow-
 *          corrected)
 **********************************************************************/
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

/**********************************************************************
 * DDOUBL (DAS A).
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_ddoubl
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   x         lm_dp_pair_t  I    pair to double
 *
 * RETURNS: lm_dp_pair_t -- 2 x
 **********************************************************************/
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

/**********************************************************************
 * MSU: a - b of two 15-bit two's-complement words (CDU angles at pi),
 * result as a ones'-complement value. A difference of exactly half a
 * revolution (two's 40000) comes out +POSMAX, as on the AGC.
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_msu
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   a         uint16_t  I    minuend, 15-bit two's complement (CDU at pi)
 *   b         uint16_t  I    subtrahend, 15-bit two's complement
 *
 * RETURNS: agc_sp_t -- a - b, ones' complement at the same scale
 **********************************************************************/
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

/**********************************************************************
 * 1STOTWOS (p.1422): a ones'-complement value back to a 15-bit two's-
 * complement word (CCS A, AD ONE / CS A).
 *
 * FUNCTION NAME: lm_dap_ones_to_twos
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  --------------------
 *   v         int32_t  I    ones'-complement value
 *
 * RETURNS: uint16_t -- 15-bit two's-complement word
 **********************************************************************/
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

/**********************************************************************
 * CCS: the diminished absolute value |v| - 1 (0 for 0).
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_ccs_dim
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    word tested by CCS
 *
 * RETURNS: agc_sp_t -- |v| - 1, or 0
 **********************************************************************/
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

/**********************************************************************
 * DIM: decrement the magnitude, stopping at zero.
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_dim
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    counter word
 *
 * RETURNS: agc_sp_t -- v moved one count toward zero
 **********************************************************************/
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

/**********************************************************************
 * DVOVSUB (AOSTASK_AND_AOSJOB.agc pp.1492-1493): DV with overflow
 * protection. Zero divisor -> POSMAX; |dividend major| >= |divisor| ->
 * POSMAX if the quotient would be positive, NEGMAX if negative.
 *
 * FUNCTION NAME: lm_dap_dvovsub
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   n         lm_dp_pair_t  I    dividend, DP pair in A,L
 *   d         agc_sp_t      I    divisor word
 *
 * RETURNS: agc_sp_t -- quotient, or POSMAX / NEGMAX
 **********************************************************************/
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
