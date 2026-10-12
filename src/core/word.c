/**********************************************************************
 * FILE NAME: word.c
 *
 * PURPOSE:
 *   The AGC numeric model of include/agc/core/word.h: conversions for
 *   tests and the sim, SP/DP/TP word structure, and the raw arithmetic
 *   utilities (saturating, overflow through *ovf, no OVFIND).
 *
 * AGC SOURCE:
 *   Kernels from Comanche055 INTERPRETER.agc: DMPSUB p.1136, ROUNDSUB
 *   p.1137, DDV pp.1162-1170, GENSCR/MPACSHR/MPACSRND pp.1152-1160,
 *   TSLC p.1157, SIGN/DCOMP p.1151, ABS p.1181 (Luminary099 identical)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name                 Description
 *   -------------------  -------------------------------------------
 *   agc_w_split/join     DP <-> word pair (mpac.c)
 *   agc_w_dmpsub         DMPSUB (mpac.c)
 *   agc_w_roundsub       ROUNDSUB (mpac.c)
 *   agc_w_ddv            DDV (divide.c)
 *   agc_w_genscr         GENSCR (shift.c)
 *   agc_w_tslc           TSLC (shift.c)
 *   ldexp, floor         <math.h>, conversions only
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; raw operations saturate at +-POSMAX / +-DPPOSMAX and report
 *   the overflow through *ovf.
 *
 * NOTES:
 *   SP/DP/TP values are signed fractions (v / 2^14, 2^28, 2^42). A DP or
 *   TP value maps onto AGC words with agreeing signs (agc_w_split); the
 *   AGC can also hold a pair whose words disagree in sign, but such a
 *   pair has the same value and only appears inside mpac.c etc. The
 *   multiply/divide/shift kernels are the AGC's own, so the low-order
 *   bits match the machine.
 **********************************************************************/
#include <math.h>

#include "agc/core/assert.h"
#include "mpac.h"

#define WORD_SCALE   16384.0 /* 2^14                                */
#define SHIFT_DP_MAX 28      /* a DP right shift past this is 0     */

/* ---- conversions (tests, sim harness, DSKY only) ------------------------ */

/**********************************************************************
 * AGC: none (conversion helper) -- round half away from zero and clamp.
 *
 * ARGUMENTS:
 *   v    double  I  value in LSB units
 *   max  double  I  largest magnitude, > 0
 *
 * RETURNS: double -- the rounded, clamped value (0 for NaN)
 **********************************************************************/
static double clamp_round(double v, double max)
{
    double r;

    r = (v < 0.0) ? -floor(-v + 0.5) : floor(v + 0.5); /* half away from 0 */

    if (r > max)
    {
        r = max;
    }
    else if (r < -max)
    {
        r = -max;
    }
    else
    {
        /* in range */
    }

    return (r);
}

/**********************************************************************
 * AGC: none (tests, sim, DSKY) -- engineering value to SP.
 *
 * ARGUMENTS:
 *   x           double  I  value in engineering units
 *   scale_pow2  int     I  N of the scale 2^N, -64 < N < 64
 *
 * RETURNS: agc_sp_t -- round(x 2^(14-N)), clamped to +-POSMAX
 **********************************************************************/
agc_sp_t agc_sp_from_double(double x, int scale_pow2)
{
    double v = ldexp(x, AGC_SP_BITS - scale_pow2);

    return ((agc_sp_t) clamp_round(v, (double) AGC_SP_MAX));
}

/**********************************************************************
 * AGC: none (tests, sim, DSKY) -- engineering value to DP.
 *
 * ARGUMENTS:
 *   x           double  I  value in engineering units
 *   scale_pow2  int     I  N of the scale 2^N, -64 < N < 64
 *
 * RETURNS: agc_dp_t -- round(x 2^(28-N)), clamped to +-DPPOSMAX
 **********************************************************************/
agc_dp_t agc_dp_from_double(double x, int scale_pow2)
{
    double v = ldexp(x, AGC_DP_BITS - scale_pow2);

    return ((agc_dp_t) clamp_round(v, (double) AGC_DP_MAX));
}

/**********************************************************************
 * AGC: none (tests, sim, DSKY) -- SP to engineering value.
 *
 * ARGUMENTS:
 *   v           agc_sp_t  I  SP word
 *   scale_pow2  int       I  N of the scale 2^N
 *
 * RETURNS: double -- v 2^(N-14)
 **********************************************************************/
double agc_sp_to_double(agc_sp_t v, int scale_pow2)
{
    return (ldexp((double) v, scale_pow2 - AGC_SP_BITS));
}

/**********************************************************************
 * AGC: none (tests, sim, DSKY) -- DP to engineering value.
 *
 * ARGUMENTS:
 *   v           agc_dp_t  I  DP value
 *   scale_pow2  int       I  N of the scale 2^N
 *
 * RETURNS: double -- v 2^(N-28)
 **********************************************************************/
double agc_dp_to_double(agc_dp_t v, int scale_pow2)
{
    return (ldexp((double) v, scale_pow2 - AGC_DP_BITS));
}

/* ---- SP/DP/TP structure ------------------------------------------------- */

/**********************************************************************
 * AGC: none (word structure) -- the major word of a DP (MPAC).
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I  DP value, 2^-28 per unit
 *
 * RETURNS: agc_sp_t -- the sign-agreeing major word, 2^-14 per unit
 **********************************************************************/
agc_sp_t agc_dp_high(agc_dp_t v)
{
    return ((agc_sp_t) agc_w_split(v).hi);
}

/**********************************************************************
 * AGC: none (word structure) -- the minor word of a DP (MPAC+1).
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I  DP value, 2^-28 per unit
 *
 * RETURNS: agc_sp_t -- the sign-agreeing minor word (sign of v)
 **********************************************************************/
agc_sp_t agc_dp_low(agc_dp_t v)
{
    return ((agc_sp_t) agc_w_split(v).lo);
}

/**********************************************************************
 * AGC: none (word structure) -- value of an AGC word pair; the words
 * may disagree in sign. A pair beyond +-DPPOSMAX (disagreeing signs at
 * the extremes only) is clamped.
 *
 * ARGUMENTS:
 *   hi  agc_sp_t  I  major word, 2^-14 per unit
 *   lo  agc_sp_t  I  minor word, 2^-28 per unit
 *
 * RETURNS: agc_dp_t -- hi 2^14 + lo
 **********************************************************************/
agc_dp_t agc_dp_pack(agc_sp_t hi, agc_sp_t lo)
{
    int32_t v = (int32_t) hi * AGC_W_ONE + (int32_t) lo; /* pair value, 2^-28 per unit */

    if (v > AGC_DP_MAX)
    {
        v = AGC_DP_MAX;
    }
    else if (v < AGC_DP_MIN)
    {
        v = AGC_DP_MIN;
    }
    else
    {
        /* in range */
    }

    return ((agc_dp_t) v);
}

/**********************************************************************
 * AGC: none (word structure) -- SP in the major word, minor word 0.
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I  SP word
 *
 * RETURNS: agc_dp_t -- v 2^14 (same scale)
 **********************************************************************/
agc_dp_t agc_dp_from_sp(agc_sp_t v)
{
    return ((agc_dp_t) ((int32_t) v * AGC_W_ONE));
}

/**********************************************************************
 * AGC: none (word structure) -- DP into MPAC, MPAC+1 with MPAC+2 = 0.
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I  DP value
 *
 * RETURNS: agc_tp_t -- v 2^14 (same scale)
 **********************************************************************/
agc_tp_t agc_tp_from_dp(agc_dp_t v)
{
    return ((agc_tp_t) v * AGC_W_ONE);
}

/**********************************************************************
 * AGC: none (word structure) -- drop MPAC+2 of the sign-agreeing word
 * triple (truncation toward zero, as a DP store of a TP MPAC).
 *
 * ARGUMENTS:
 *   v  agc_tp_t  I  TP value
 *
 * RETURNS: agc_dp_t -- MPAC, MPAC+1
 **********************************************************************/
agc_dp_t agc_tp_to_dp(agc_tp_t v)
{
    return (agc_w_join(agc_w3_top(agc_w3_from_tp(v))));
}

/**********************************************************************
 * ROUNDSUB (INTERPRETER.agc p.1137) on the sign-agreeing word triple;
 * saturates on the (only theoretical) carry out of MPAC.
 *
 * ARGUMENTS:
 *   v  agc_tp_t  I  TP value
 *
 * RETURNS: agc_dp_t -- v rounded half away from zero to DP
 **********************************************************************/
agc_dp_t agc_tp_round_dp(agc_tp_t v)
{
    agc_ovf_t ovf = 0;                                       /* ROUNDSUB carry out of MPAC */
    agc_w3_t m = agc_w_roundsub(agc_w3_from_tp(v), &ovf); /* rounded TP MPAC */

    if (ovf != 0)
    {
        return ((ovf > 0) ? AGC_DP_MAX : AGC_DP_MIN); /* saturate */
    }

    return (agc_w_join(agc_w3_top(m)));
}

/* ---- word form <-> value ------------------------------------------------- */

/**********************************************************************
 * AGC: none (word structure) -- the words a DCA of a stored DP gives:
 * both carry the sign of the value.
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I  DP value, 2^-28 per unit
 *
 * RETURNS: agc_dp_words_t -- {hi, lo}, signs agreeing
 **********************************************************************/
agc_dp_words_t agc_dp_to_words(agc_dp_t v)
{
    agc_dp_words_t w; /* the sign-agreeing words */

    w = agc_w2_to_words(agc_w_split(v));

    return (w);
}

/**********************************************************************
 * AGC: none (word structure) -- the value of a stored word pair.
 *
 * ARGUMENTS:
 *   w  agc_dp_words_t  I  major and minor word, signs free
 *
 * RETURNS: agc_dp_t -- hi 2^14 + lo, 2^-28 per unit
 **********************************************************************/
agc_dp_t agc_dp_from_words(agc_dp_words_t w)
{
    return (agc_w_join(agc_w2_from_words(w)));
}

/**********************************************************************
 * AGC: none (word structure) -- the three words of a TP whose signs
 * agree (TLOAD of a stored TP, or a TP value made by the value API).
 *
 * ARGUMENTS:
 *   v  agc_tp_t  I  TP value, 2^-42 per unit
 *
 * RETURNS: agc_tp_words_t -- {hi, mid, lo}, signs agreeing
 **********************************************************************/
agc_tp_words_t agc_tp_to_words(agc_tp_t v)
{
    agc_tp_words_t m; /* the sign-agreeing words */

    m = agc_w3_to_words(agc_w3_from_tp(v));

    return (m);
}

/**********************************************************************
 * AGC: none (word structure) -- the value of three TP words.
 *
 * ARGUMENTS:
 *   m  agc_tp_words_t  I  MPAC, MPAC+1, MPAC+2, signs free
 *
 * RETURNS: agc_tp_t -- (hi 2^14 + mid) 2^14 + lo, 2^-42 per unit
 **********************************************************************/
agc_tp_t agc_tp_from_words(agc_tp_words_t m)
{
    return (agc_w3_value(agc_w3_from_words(m)));
}

/* ---- raw arithmetic ------------------------------------------------------ */

/**********************************************************************
 * AGC: none (raw utilities) -- clamp to +-max and report the overflow.
 *
 * ARGUMENTS:
 *   v    int64_t      I  exact result
 *   max  int32_t      I  POSMAX or DPPOSMAX, > 0
 *   ovf  agc_ovf_t *  O  +1 / -1 when clamped, else 0
 *
 * RETURNS: int32_t -- v clamped to [-max, max]
 **********************************************************************/
static int32_t saturate32(int64_t v, int32_t max, agc_ovf_t *ovf)
{
    agc_ovf_t o = 0; /* clamp indication, +-1 or 0 */

    if (v > max)
    {
        v = max;
        o = 1;
    }
    else if (v < -max)
    {
        v = -max;
        o = -1;
    }
    else
    {
        /* in range */
    }

    if (AGC_ASSERT(ovf != 0))
    {
        *ovf = o;
    }

    return ((int32_t) v);
}

/**********************************************************************
 * AGC: none (raw utility, saturating AD) -- a + b.
 *
 * ARGUMENTS:
 *   a    agc_sp_t     I  augend
 *   b    agc_sp_t     I  addend, same scale
 *   ovf  agc_ovf_t *  O  +-1 when the sum saturated, else 0
 *
 * RETURNS: agc_sp_t -- a + b clamped to +-POSMAX
 **********************************************************************/
agc_sp_t agc_sp_add(agc_sp_t a, agc_sp_t b, agc_ovf_t *ovf)
{
    return ((agc_sp_t) saturate32((int64_t) a + b, AGC_SP_MAX, ovf));
}

/**********************************************************************
 * AGC: none (basic instruction MP) -- SP x SP -> DP, exact (both
 * result words carry the product sign).
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  multiplicand
 *   b  agc_sp_t  I  multiplier; scales add
 *
 * RETURNS: agc_dp_t -- a b
 **********************************************************************/
agc_dp_t agc_sp_mul(agc_sp_t a, agc_sp_t b)
{
    return (agc_w_join(agc_w_mp(a, b)));
}

/**********************************************************************
 * AGC: none (raw utility; the AGC's DAD is agc_dp_add) -- a + b.
 *
 * ARGUMENTS:
 *   a    agc_dp_t     I  augend
 *   b    agc_dp_t     I  addend, same scale
 *   ovf  agc_ovf_t *  O  +-1 when the sum saturated, else 0
 *
 * RETURNS: agc_dp_t -- a + b clamped to +-DPPOSMAX
 **********************************************************************/
agc_dp_t agc_dp_add_raw(agc_dp_t a, agc_dp_t b, agc_ovf_t *ovf)
{
    return ((agc_dp_t) saturate32((int64_t) a + b, AGC_DP_MAX, ovf));
}

/**********************************************************************
 * AGC: none (raw utility; the AGC's DSU is agc_dp_sub) -- a - b.
 *
 * ARGUMENTS:
 *   a    agc_dp_t     I  minuend
 *   b    agc_dp_t     I  subtrahend, same scale
 *   ovf  agc_ovf_t *  O  +-1 when the difference saturated, else 0
 *
 * RETURNS: agc_dp_t -- a - b clamped to +-DPPOSMAX
 **********************************************************************/
agc_dp_t agc_dp_sub_raw(agc_dp_t a, agc_dp_t b, agc_ovf_t *ovf)
{
    return ((agc_dp_t) saturate32((int64_t) a - b, AGC_DP_MAX, ovf));
}

/**********************************************************************
 * DMP (DMPSUB, INTERPRETER.agc p.1136) -- the TP product as the AGC
 * forms it. Only the major half of minor x minor is kept, so the result
 * can differ from the exact product by less than 2^-42.
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  multiplicand
 *   b  agc_dp_t  I  multiplier; scales add
 *
 * RETURNS: agc_tp_t -- the TP MPAC
 **********************************************************************/
agc_tp_t agc_dp_mul_tp(agc_dp_t a, agc_dp_t b)
{
    return (agc_w3_value(agc_w_dmpsub(agc_w_split(a), agc_w_split(b))));
}

/**********************************************************************
 * DMPR (INTERPRETER.agc p.1149) -- DMPSUB then ROUNDSUB.
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  multiplicand
 *   b  agc_dp_t  I  multiplier; scales add
 *
 * RETURNS: agc_dp_t -- a b rounded half away from zero
 **********************************************************************/
agc_dp_t agc_dp_mul_raw(agc_dp_t a, agc_dp_t b)
{
    agc_ovf_t ovf = 0;                                            /* ROUNDSUB carry (none: |a b| < 1) */
    agc_w3_t m = agc_w_dmpsub(agc_w_split(a), agc_w_split(b)); /* DMPSUB TP product */

    m = agc_w_roundsub(m, &ovf);

    return (agc_w_join(agc_w3_top(m)));
}

/**********************************************************************
 * DDV (INTERPRETER.agc pp.1162-1170) -- a / b; +-DPPOSMAX with *ovf =
 * +1 on divide overflow (the AGC sets OVFIND to +1 whatever the
 * quotient sign). DVSIGN is not kept.
 *
 * ARGUMENTS:
 *   a    agc_dp_t     I  dividend
 *   b    agc_dp_t     I  divisor, same scale
 *   ovf  agc_ovf_t *  O  1 on overflow (|a| >= |b|, b = 0), else 0
 *
 * RETURNS: agc_dp_t -- a / b scaled 2^0
 **********************************************************************/
agc_dp_t agc_dp_div_raw(agc_dp_t a, agc_dp_t b, agc_ovf_t *ovf)
{
    int o = 0;     /* OVFIND of DDV: 1 on overflow         */
    bool dvsign = false; /* DVSIGN, not kept by the raw utility */
    agc_dp_t q;              /* quotient, scaled 2^0                */

    q = agc_w_join(agc_w3_top(agc_w_ddv(agc_w3_from_tp(agc_tp_from_dp(a)), agc_w_split(b), &o, &dvsign)));

    if (AGC_ASSERT(ovf != 0))
    {
        *ovf = (agc_ovf_t) o;
    }

    return (q);
}

/**********************************************************************
 * AGC: none (raw utility; the AGC's wrapping SL is agc_dp_shl) -- shift
 * left n, saturating.
 *
 * ARGUMENTS:
 *   a    agc_dp_t     I  value
 *   n    int          I  shift count, >= 0
 *   ovf  agc_ovf_t *  O  +-1 when the result saturated, else 0
 *
 * RETURNS: agc_dp_t -- a 2^n clamped to +-DPPOSMAX (scale -n)
 **********************************************************************/
agc_dp_t agc_dp_shl_raw(agc_dp_t a, int n, agc_ovf_t *ovf)
{
    int64_t v = a; /* a 2^k, exact */
    int k;     /* doublings done */

    for (k = 0; (k < n) && (k < SHIFT_DP_MAX + 1); k++)
    {
        v *= 2;
    }

    if ((n > SHIFT_DP_MAX) && (a != 0))
    {
        v = (a > 0) ? (int64_t) AGC_DP_MAX + 1 : (int64_t) AGC_DP_MIN - 1; /* saturate by sign */
    }

    return ((agc_dp_t) saturate32(v, AGC_DP_MAX, ovf));
}

/**********************************************************************
 * SR n (GENSCR / MPACSHR, INTERPRETER.agc pp.1152, 1160) -- truncates
 * toward zero, word by word.
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  value
 *   n  int       I  shift count, >= 0
 *
 * RETURNS: agc_dp_t -- a 2^-n truncated (scale +n)
 **********************************************************************/
agc_dp_t agc_dp_shr(agc_dp_t a, int n)
{
    agc_w3_t m; /* TP MPAC of a */

    if (n == 0)
    {
        return (a);
    }

    m = agc_w3_from_tp(agc_tp_from_dp(a));

    return (agc_w_join(agc_w3_top(agc_w_genscr(m, n, false))));
}

/**********************************************************************
 * SRR n (GENSCR / MPACSRND, INTERPRETER.agc pp.1153, 1160) -- rounds
 * half away from zero.
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  value
 *   n  int       I  shift count, >= 0
 *
 * RETURNS: agc_dp_t -- a 2^-n rounded (scale +n)
 **********************************************************************/
agc_dp_t agc_dp_shr_round(agc_dp_t a, int n)
{
    agc_w3_t m; /* TP MPAC of a */

    if (n == 0)
    {
        return (a);
    }

    m = agc_w3_from_tp(agc_tp_from_dp(a));

    return (agc_w_join(agc_w3_top(agc_w_genscr(m, n, true))));
}

/**********************************************************************
 * DCOMP (INTERPRETER.agc p.1151) -- -0 is not represented: DCOMP of 0
 * is 0.
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  value
 *
 * RETURNS: agc_dp_t -- -a
 **********************************************************************/
agc_dp_t agc_dp_neg(agc_dp_t a)
{
    return (-a);
}

/**********************************************************************
 * ABS (INTERPRETER.agc p.1181) -- BRANCH on the sign, COMP if negative.
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  value
 *
 * RETURNS: agc_dp_t -- |a|
 **********************************************************************/
agc_dp_t agc_dp_abs(agc_dp_t a)
{
    return ((a < 0) ? -a : a); /* COMP if negative */
}

/**********************************************************************
 * AGC: none (DOUBLE; TS L carry as in ROUNDSUB) -- round a DP to SP
 * precision (major word), half away from zero; the result is still a DP
 * with a zero minor word. (The interpretive ROUND, TP to DP, is
 * agc_tp_round.)
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  value
 *
 * RETURNS: agc_dp_t -- a rounded to a multiple of 2^14, clamped to POSMAX
 **********************************************************************/
agc_dp_t agc_dp_round(agc_dp_t a)
{
    agc_w2_t p = agc_w_split(a); /* MPAC, MPAC+1 of a */
    int o = 0;              /* rounding carry, +-1 or 0 */

    o = agc_w_ovf(2 * p.lo); /* DOUBLE; TS L     */
    p.hi += o;

    if (p.hi > AGC_W_POSMAX)
    {
        p.hi = AGC_W_POSMAX;
    }
    else if (p.hi < -AGC_W_POSMAX)
    {
        p.hi = -AGC_W_POSMAX;
    }
    else
    {
        /* in range */
    }

    return ((agc_dp_t) (p.hi * AGC_W_ONE));
}

/**********************************************************************
 * SIGN X (INTERPRETER.agc p.1151) -- complement MPAC when X is negative
 * non-zero.
 *
 * ARGUMENTS:
 *   a        agc_dp_t  I  MPAC
 *   sign_of  agc_dp_t  I  X (any scale)
 *
 * RETURNS: agc_dp_t -- -a if sign_of < 0, else a
 **********************************************************************/
agc_dp_t agc_dp_sign(agc_dp_t a, agc_dp_t sign_of)
{
    return ((sign_of < 0) ? -a : a); /* COMP if X < 0 */
}

/**********************************************************************
 * NORM (TSLC, INTERPRETER.agc p.1157) -- shared by agc_dp_norm and
 * agc_dp_norm_count.
 *
 * ARGUMENTS:
 *   a      agc_dp_t    I  value
 *   shift  int *       O  left shifts done (0 for a = 0); may be NULL
 *   out    agc_dp_t *  O  a left-justified (|major word| >= 1/2), or 0;
 *                         NULL when only the count is read
 **********************************************************************/
static void dp_norm_core(agc_dp_t a, int *shift, agc_dp_t *out)
{
    int count = 0;                                 /* left shifts done */
    agc_w3_t m = agc_w3_from_tp(agc_tp_from_dp(a)); /* TP MPAC of a */

    m = agc_w_tslc(m, &count);

    if (shift != 0)
    {
        *shift = count;
    }

    if (out != 0)
    {
        *out = agc_w_join(agc_w3_top(m));
    }
}

/**********************************************************************
 * NORM (TSLC, INTERPRETER.agc p.1157) on a DP.
 *
 * ARGUMENTS:
 *   a      agc_dp_t  I  value
 *   shift  int *     O  left shifts done (the AGC stores the complement,
 *                       -*shift, in the index register named)
 *
 * RETURNS: agc_dp_t -- a left-justified (|major word| >= 1/2), or 0
 **********************************************************************/
agc_dp_t agc_dp_norm(agc_dp_t a, int *shift)
{
    agc_dp_t n = 0; /* a left-justified */

    dp_norm_core(a, shift, &n);

    return (n);
}

/**********************************************************************
 * NORM X1 (TSLC, INTERPRETER.agc p.1157) when only the shift count is
 * wanted (the normalised MPAC is not used).
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  value
 *
 * RETURNS: int -- left shifts that left-justify a (0 for a = 0)
 **********************************************************************/
int agc_dp_norm_count(agc_dp_t a)
{
    int count = 0; /* left shifts done */

    dp_norm_core(a, &count, 0);

    return (count);
}

/**********************************************************************
 * AGC: none (utility) -- the sign of a DP.
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  value
 *
 * RETURNS: int -- +1, -1 or 0
 **********************************************************************/
int agc_dp_sgn(agc_dp_t a)
{
    if (a > 0)
    {
        return (1);
    }

    if (a < 0)
    {
        return (-1);
    }

    return (0);
}
