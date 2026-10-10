/*
 * agc_word.c -- the AGC numeric model of include/agc/agc_word.h.
 *
 * SP/DP/TP values are signed fractions (v / 2^14, 2^28, 2^42). A DP or TP
 * value maps onto AGC words with agreeing signs (agc_w_split): the AGC can
 * also hold a pair whose words disagree in sign, but such a pair has the
 * same value and only appears inside the routines of agc_mpac.c etc.
 *
 * The "raw" operations here follow the header contract: they saturate at
 * +-POSMAX and report overflow through *ovf; they do not touch OVFIND. The
 * multiply/divide/shift kernels are the AGC's own (DMPSUB, ROUNDSUB, DDV,
 * GENSCR, MPACSRND) so their low-order bits match the real machine.
 */
#include <math.h>
#include "mpac.h"
#include "agc/agc_assert.h"

#define WORD_SCALE   16384.0         /* 2^14                                */
#define SHIFT_DP_MAX 28              /* a DP right shift past this is 0     */

/* ---- conversions (tests, sim harness, DSKY only) ------------------------ */

static double clamp_round(double v, double max)
{
    double r;
    (void)AGC_ASSERT(max > 0.0);
    if (!AGC_ASSERT(v == v)) {                         /* NaN              */
        return 0.0;
    }
    r = (v < 0.0) ? -floor(-v + 0.5) : floor(v + 0.5); /* half away from 0 */
    if (r > max) {
        r = max;
    } else if (r < -max) {
        r = -max;
    } else {
        /* in range */
    }
    return r;
}

agc_sp_t agc_sp_from_double(double x, int scale_pow2)
{
    double v = ldexp(x, AGC_SP_BITS - scale_pow2);
    (void)AGC_ASSERT(scale_pow2 > -64);
    (void)AGC_ASSERT(scale_pow2 < 64);
    return (agc_sp_t)clamp_round(v, (double)AGC_SP_MAX);
}

agc_dp_t agc_dp_from_double(double x, int scale_pow2)
{
    double v = ldexp(x, AGC_DP_BITS - scale_pow2);
    (void)AGC_ASSERT(scale_pow2 > -64);
    (void)AGC_ASSERT(scale_pow2 < 64);
    return (agc_dp_t)clamp_round(v, (double)AGC_DP_MAX);
}

double agc_sp_to_double(agc_sp_t v, int scale_pow2)
{
    (void)AGC_ASSERT((v >= AGC_SP_MIN) && (v <= AGC_SP_MAX));
    (void)AGC_ASSERT((scale_pow2 > -64) && (scale_pow2 < 64));
    return ldexp((double)v, scale_pow2 - AGC_SP_BITS);
}

double agc_dp_to_double(agc_dp_t v, int scale_pow2)
{
    (void)AGC_ASSERT((v >= AGC_DP_MIN) && (v <= AGC_DP_MAX));
    (void)AGC_ASSERT((scale_pow2 > -64) && (scale_pow2 < 64));
    return ldexp((double)v, scale_pow2 - AGC_DP_BITS);
}

double agc_tp_to_double(agc_tp_t v, int scale_pow2)
{
    (void)AGC_ASSERT((v >= AGC_TP_MIN) && (v <= AGC_TP_MAX));
    (void)AGC_ASSERT((scale_pow2 > -64) && (scale_pow2 < 64));
    return ldexp((double)v, scale_pow2 - AGC_TP_BITS);
}

/* ---- SP/DP/TP structure ------------------------------------------------- */

agc_sp_t agc_dp_high(agc_dp_t v)
{
    (void)AGC_ASSERT(v >= AGC_DP_MIN);
    (void)AGC_ASSERT(v <= AGC_DP_MAX);
    return (agc_sp_t)agc_w_split(v).hi;
}

agc_sp_t agc_dp_low(agc_dp_t v)
{
    (void)AGC_ASSERT(v >= AGC_DP_MIN);
    (void)AGC_ASSERT(v <= AGC_DP_MAX);
    return (agc_sp_t)agc_w_split(v).lo;
}

/* Value of an AGC word pair; the words may disagree in sign. A pair whose
 * value lies outside +-DPPOSMAX (only possible with disagreeing signs at
 * the extremes) is clamped.                                                */
agc_dp_t agc_dp_pack(agc_sp_t hi, agc_sp_t lo)
{
    int32_t v = (int32_t)hi * AGC_W_ONE + (int32_t)lo;
    (void)AGC_ASSERT((hi >= AGC_SP_MIN) && (hi <= AGC_SP_MAX));
    (void)AGC_ASSERT((lo >= AGC_SP_MIN) && (lo <= AGC_SP_MAX));
    if (v > AGC_DP_MAX) {
        v = AGC_DP_MAX;
    } else if (v < AGC_DP_MIN) {
        v = AGC_DP_MIN;
    } else {
        /* in range */
    }
    return (agc_dp_t)v;
}

agc_dp_t agc_dp_from_sp(agc_sp_t v)
{
    (void)AGC_ASSERT(v >= AGC_SP_MIN);
    (void)AGC_ASSERT(v <= AGC_SP_MAX);
    return (agc_dp_t)((int32_t)v * AGC_W_ONE);
}

agc_tp_t agc_tp_from_dp(agc_dp_t v)
{
    (void)AGC_ASSERT(v >= AGC_DP_MIN);
    (void)AGC_ASSERT(v <= AGC_DP_MAX);
    return (agc_tp_t)v * AGC_W_ONE;
}

/* Drop MPAC+2 of the sign-agreeing word triple (truncation toward zero).   */
agc_dp_t agc_tp_to_dp(agc_tp_t v)
{
    (void)AGC_ASSERT(v >= AGC_TP_MIN);
    (void)AGC_ASSERT(v <= AGC_TP_MAX);
    return agc_w_join(agc_w3_top(agc_w3_from_tp(v)));
}

/* ROUNDSUB on the sign-agreeing word triple; saturates on the (only
 * theoretical) carry out of MPAC.                                          */
agc_dp_t agc_tp_round_dp(agc_tp_t v)
{
    agc_ovf_t ovf = 0;
    agc_w3_t m = agc_w_roundsub(agc_w3_from_tp(v), &ovf);
    (void)AGC_ASSERT((v >= AGC_TP_MIN) && (v <= AGC_TP_MAX));
    (void)AGC_ASSERT(m.m2 == 0);                       /* ROUNDSUB zeroes  */
    if (ovf != 0) {
        return (ovf > 0) ? AGC_DP_MAX : AGC_DP_MIN;
    }
    return agc_w_join(agc_w3_top(m));
}

/* ---- raw arithmetic ------------------------------------------------------ */

static int32_t saturate32(int64_t v, int32_t max, agc_ovf_t *ovf)
{
    agc_ovf_t o = 0;
    (void)AGC_ASSERT(max > 0);
    if (v > max) {
        v = max;
        o = 1;
    } else if (v < -max) {
        v = -max;
        o = -1;
    } else {
        /* in range */
    }
    if (AGC_ASSERT(ovf != 0)) {
        *ovf = o;
    }
    return (int32_t)v;
}

agc_sp_t agc_sp_add(agc_sp_t a, agc_sp_t b, agc_ovf_t *ovf)
{
    (void)AGC_ASSERT((a >= AGC_SP_MIN) && (a <= AGC_SP_MAX));
    (void)AGC_ASSERT((b >= AGC_SP_MIN) && (b <= AGC_SP_MAX));
    return (agc_sp_t)saturate32((int64_t)a + b, AGC_SP_MAX, ovf);
}

agc_sp_t agc_sp_sub(agc_sp_t a, agc_sp_t b, agc_ovf_t *ovf)
{
    (void)AGC_ASSERT((a >= AGC_SP_MIN) && (a <= AGC_SP_MAX));
    (void)AGC_ASSERT((b >= AGC_SP_MIN) && (b <= AGC_SP_MAX));
    return (agc_sp_t)saturate32((int64_t)a - b, AGC_SP_MAX, ovf);
}

/* MP: SP x SP -> DP, exact (both result words carry the product sign).     */
agc_dp_t agc_sp_mul(agc_sp_t a, agc_sp_t b)
{
    (void)AGC_ASSERT((a >= AGC_SP_MIN) && (a <= AGC_SP_MAX));
    (void)AGC_ASSERT((b >= AGC_SP_MIN) && (b <= AGC_SP_MAX));
    return agc_w_join(agc_w_mp(a, b));
}

agc_dp_t agc_dp_add_raw(agc_dp_t a, agc_dp_t b, agc_ovf_t *ovf)
{
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    (void)AGC_ASSERT((b >= AGC_DP_MIN) && (b <= AGC_DP_MAX));
    return (agc_dp_t)saturate32((int64_t)a + b, AGC_DP_MAX, ovf);
}

agc_dp_t agc_dp_sub_raw(agc_dp_t a, agc_dp_t b, agc_ovf_t *ovf)
{
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    (void)AGC_ASSERT((b >= AGC_DP_MIN) && (b <= AGC_DP_MAX));
    return (agc_dp_t)saturate32((int64_t)a - b, AGC_DP_MAX, ovf);
}

/* DMP (DMPSUB p.1136): TP product as the AGC forms it. Only the major half
 * of minor x minor is kept, so the result can differ from the exact
 * product by less than 2^-42.                                               */
agc_tp_t agc_dp_mul_tp(agc_dp_t a, agc_dp_t b)
{
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    (void)AGC_ASSERT((b >= AGC_DP_MIN) && (b <= AGC_DP_MAX));
    return agc_w3_value(agc_w_dmpsub(agc_w_split(a), agc_w_split(b)));
}

/* DMPR (p.1149): DMPSUB then ROUNDSUB.                                     */
agc_dp_t agc_dp_mul_raw(agc_dp_t a, agc_dp_t b)
{
    agc_ovf_t ovf = 0;
    agc_w3_t m = agc_w_dmpsub(agc_w_split(a), agc_w_split(b));
    m = agc_w_roundsub(m, &ovf);
    (void)AGC_ASSERT(ovf == 0);                        /* |a b| < 1        */
    (void)AGC_ASSERT(m.m2 == 0);
    return agc_w_join(agc_w3_top(m));
}

/* DDV (pp.1162-1170): a / b, +-DPPOSMAX with *ovf = +1 on divide overflow
 * (the AGC sets OVFIND to +1 whatever the quotient sign).                   */
agc_dp_t agc_dp_div_raw(agc_dp_t a, agc_dp_t b, agc_ovf_t *ovf)
{
    int o = 0;
    agc_dp_t q = agc_w_ddv(a, b, &o);
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    if (AGC_ASSERT(ovf != 0)) {
        *ovf = (agc_ovf_t)o;
    }
    return q;
}

/* SL n, saturating (raw contract). For the AGC's wrapping SL see
 * agc_dp_shl in agc_interp.h.                                               */
agc_dp_t agc_dp_shl_raw(agc_dp_t a, int n, agc_ovf_t *ovf)
{
    int64_t v = a;
    int k;
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    (void)AGC_ASSERT(n >= 0);
    for (k = 0; (k < n) && (k < SHIFT_DP_MAX + 1); k++) {
        v *= 2;
    }
    if ((n > SHIFT_DP_MAX) && (a != 0)) {
        v = (a > 0) ? (int64_t)AGC_DP_MAX + 1 : (int64_t)AGC_DP_MIN - 1;
    }
    return (agc_dp_t)saturate32(v, AGC_DP_MAX, ovf);
}

/* SR n (GENSCR/MPACSHR p.1160, 1152): truncates toward zero.               */
agc_dp_t agc_dp_shr(agc_dp_t a, int n)
{
    agc_w3_t m;
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    if (!AGC_ASSERT(n >= 0) || (n == 0)) {
        return a;
    }
    m = agc_w3_from_tp(agc_tp_from_dp(a));
    return agc_w_join(agc_w3_top(agc_w_genscr(m, n, false)));
}

/* SRR n (MPACSRND p.1153): rounds half away from zero.                     */
agc_dp_t agc_dp_shr_round(agc_dp_t a, int n)
{
    agc_w3_t m;
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    if (!AGC_ASSERT(n >= 0) || (n == 0)) {
        return a;
    }
    m = agc_w3_from_tp(agc_tp_from_dp(a));
    return agc_w_join(agc_w3_top(agc_w_genscr(m, n, true)));
}

/* DCOMP (p.1151). -0 is not represented: DCOMP of 0 is 0.                  */
agc_dp_t agc_dp_neg(agc_dp_t a)
{
    (void)AGC_ASSERT(a >= AGC_DP_MIN);
    (void)AGC_ASSERT(a <= AGC_DP_MAX);
    return -a;
}

/* ABS (p.1181): BRANCH on the sign, COMP if negative.                       */
agc_dp_t agc_dp_abs(agc_dp_t a)
{
    (void)AGC_ASSERT(a >= AGC_DP_MIN);
    (void)AGC_ASSERT(a <= AGC_DP_MAX);
    return (a < 0) ? -a : a;
}

/* Round a DP value to SP precision (major word), half away from zero; the
 * result is still a DP with a zero minor word. (The interpretive ROUND
 * instruction rounds the TP MPAC to DP: agc_tp_round.)                     */
agc_dp_t agc_dp_round(agc_dp_t a)
{
    agc_w2_t p = agc_w_split(a);
    int o = 0;
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    (void)agc_w_ts(2 * p.lo, &o);                      /* DOUBLE; TS L     */
    p.hi += o;
    (void)AGC_ASSERT((p.hi >= -AGC_W_ONE) && (p.hi <= AGC_W_ONE));
    if (p.hi > AGC_W_POSMAX) {
        p.hi = AGC_W_POSMAX;
    } else if (p.hi < -AGC_W_POSMAX) {
        p.hi = -AGC_W_POSMAX;
    } else {
        /* in range */
    }
    return (agc_dp_t)(p.hi * AGC_W_ONE);
}

/* SIGN X (p.1151): complement MPAC when X is negative non-zero.            */
agc_dp_t agc_dp_sign(agc_dp_t a, agc_dp_t sign_of)
{
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    (void)AGC_ASSERT((sign_of >= AGC_DP_MIN) && (sign_of <= AGC_DP_MAX));
    return (sign_of < 0) ? -a : a;
}

/* NORM (TSLC p.1157) on a DP: *shift = left shifts done (the AGC stores the
 * complement, -*shift, in the index register named in the address).      */
agc_dp_t agc_dp_norm(agc_dp_t a, int *shift)
{
    int count = 0;
    agc_w3_t m = agc_w3_from_tp(agc_tp_from_dp(a));
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    m = agc_w_tslc(m, &count);
    if (AGC_ASSERT(shift != 0)) {
        *shift = count;
    }
    return agc_w_join(agc_w3_top(m));
}

int agc_dp_sgn(agc_dp_t a)
{
    (void)AGC_ASSERT(a >= AGC_DP_MIN);
    (void)AGC_ASSERT(a <= AGC_DP_MAX);
    return (a > 0) ? 1 : ((a < 0) ? -1 : 0);
}
