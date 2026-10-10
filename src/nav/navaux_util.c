/*
 * navaux_util.c -- interpretive-behaviour helpers shared by the nav-aux
 * files (see navaux_internal.h).
 *
 * DEVIATION: agc_interp.h has no TP shift/normalize and its DP operations
 * saturate on overflow.  The helpers below therefore work on the integer
 * representation of agc_tp_t / agc_dp_t directly (value = raw / 2^42 or
 * raw / 2^28).  This is confined to this file and documented per helper.
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "navaux_internal.h"

#define NAVAUX_DP_ONE ((int64_t)1 << AGC_DP_BITS)   /* 1.0 in DP raw units */
#define NAVAUX_TP_ONE ((int64_t)1 << AGC_TP_BITS)   /* 1.0 in TP raw units */
#define NAVAUX_TP_HALF ((int64_t)1 << (AGC_TP_BITS - 1))

bool agc_navaux_dp_ok(agc_dp_t a)
{
    return (a <= AGC_DP_MAX) && (a >= AGC_DP_MIN);
}

bool agc_navaux_vec_ok(agc_vec_t v)
{
    return agc_navaux_dp_ok(v.c[0]) && agc_navaux_dp_ok(v.c[1]) &&
           agc_navaux_dp_ok(v.c[2]);
}

bool agc_navaux_tp_ok(agc_tp_t a)
{
    return (a <= AGC_TP_MAX) && (a >= AGC_TP_MIN);
}

/* SL* / SR* X,n with an indexed (signed) shift count. */
agc_dp_t agc_navaux_dp_shift(agc_interp_t *ip, agc_dp_t a, int n)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return 0;
    }
    if (!AGC_ASSERT((n <= NAVAUX_MAX_SHIFT) && (n >= -NAVAUX_MAX_SHIFT))) {
        return 0;
    }
    agc_dp_t r = a;
    if (n > 0) {
        r = agc_dp_shl(ip, a, n);
    } else if (n < 0) {
        r = agc_dp_shr(a, -n);
    }
    (void)AGC_ASSERT(agc_navaux_dp_ok(r));      /* post: in range         */
    return r;
}

/* VSL* / VSR* with an indexed (signed) shift count. */
agc_vec_t agc_navaux_vec_shift(agc_interp_t *ip, agc_vec_t v, int n)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return agc_vec_zero();
    }
    if (!AGC_ASSERT((n <= NAVAUX_MAX_SHIFT) && (n >= -NAVAUX_MAX_SHIFT))) {
        return agc_vec_zero();
    }
    agc_vec_t r = v;
    if (n > 0) {
        r = agc_vec_shl(ip, v, n);
    } else if (n < 0) {
        r = agc_vec_shr(v, -n);
    }
    (void)AGC_ASSERT(agc_navaux_vec_ok(r));     /* post: in range         */
    return r;
}

/* SR 14 of a DP time (cs, 2^28) into TP mode (cs, 2^42).  With both scales
 * the raw integer is the time in centiseconds, so the shift is exact.     */
agc_tp_t agc_navaux_tp_from_cs(agc_dp_t t_cs)
{
    if (!AGC_ASSERT(agc_navaux_dp_ok(t_cs))) {
        return 0;
    }
    agc_tp_t t = (agc_tp_t)t_cs;
    (void)AGC_ASSERT(agc_navaux_tp_ok(t));
    return t;
}

/* Inverse of agc_navaux_tp_from_cs (SL 14 of a TP time into DP). */
agc_dp_t agc_navaux_cs_from_tp(agc_tp_t t_tp)
{
    if (!AGC_ASSERT(agc_navaux_tp_ok(t_tp))) {
        return 0;
    }
    if (!AGC_ASSERT((t_tp <= AGC_DP_MAX) && (t_tp >= AGC_DP_MIN))) {
        return (t_tp > 0) ? AGC_DP_MAX : AGC_DP_MIN;
    }
    agc_dp_t t = (agc_dp_t)t_tp;
    (void)AGC_ASSERT(agc_navaux_dp_ok(t));
    return t;
}

/* DLOAD X+1 of a TP X: the two low-order words as a DP fraction (the TP
 * is taken as sign-agreed, so the low words carry the sign of the whole). */
agc_dp_t agc_navaux_tp_low_dp(agc_tp_t v)
{
    if (!AGC_ASSERT(agc_navaux_tp_ok(v))) {
        return 0;
    }
    int64_t mag = (v < 0) ? -v : v;
    int64_t low = mag % NAVAUX_DP_ONE;
    (void)AGC_ASSERT(low < NAVAUX_DP_ONE);
    return (agc_dp_t)((v < 0) ? -low : low);
}

/* NORM (TSLC) in TP mode: shift left until |v| >= 1/2. */
agc_tp_t agc_navaux_tp_norm(agc_tp_t v, int *shift)
{
    if (!AGC_ASSERT(shift != NULL)) {
        return v;
    }
    *shift = 0;
    if (!AGC_ASSERT(agc_navaux_tp_ok(v)) || (v == 0)) {
        return v;                 /* zero: shift count zero, as TSLC      */
    }
    int64_t mag = (v < 0) ? -v : v;
    for (int i = 0; (i < AGC_TP_BITS) && (mag < NAVAUX_TP_HALF); i++) {
        mag *= 2;
        *shift += 1;
    }
    (void)AGC_ASSERT(mag >= NAVAUX_TP_HALF);     /* post: normalized      */
    return (v < 0) ? -mag : mag;
}

/* SL2 of a TP; on overflow *ovf is set and v is returned unchanged (the
 * only caller, INCOR1B, discards the overflowed result).                 */
agc_tp_t agc_navaux_tp_shl2(agc_tp_t v, bool *ovf)
{
    if (!AGC_ASSERT(ovf != NULL)) {
        return v;
    }
    if (!AGC_ASSERT(agc_navaux_tp_ok(v))) {
        *ovf = true;
        return v;
    }
    int64_t mag = (v < 0) ? -v : v;
    *ovf = (mag * 4) > AGC_TP_MAX;
    (void)AGC_ASSERT(*ovf || agc_navaux_tp_ok(v * 4));
    return *ovf ? v : v * 4;
}

/* SL n of a TP angle in revolutions followed by rounding to DP, with the
 * overflow wrap the AGC gives (bits above the binary point are lost, the
 * sign is kept).  DEVIATION: models wrap, agc_dp_shl would saturate.     */
agc_dp_t agc_navaux_rev_tp_shl(agc_tp_t v, int n)
{
    if (!AGC_ASSERT(agc_navaux_tp_ok(v))) {
        return 0;
    }
    if (!AGC_ASSERT((n >= 0) && (n <= 16))) {
        return 0;
    }
    int64_t mag = (v < 0) ? -v : v;
    mag = (mag * ((int64_t)1 << n)) % NAVAUX_TP_ONE;
    (void)AGC_ASSERT(mag < NAVAUX_TP_ONE);
    return agc_tp_round_dp((v < 0) ? -mag : mag);
}

/* DAD of two angles in revolutions with overflow correction (wrap by one
 * revolution).  DEVIATION: models wrap, agc_dp_add would saturate.        */
agc_dp_t agc_navaux_rev_add(agc_dp_t a, agc_dp_t b)
{
    if (!AGC_ASSERT(agc_navaux_dp_ok(a) && agc_navaux_dp_ok(b))) {
        return 0;
    }
    int64_t s = (int64_t)a + (int64_t)b;
    if (s >= NAVAUX_DP_ONE) {
        s -= NAVAUX_DP_ONE;
    } else if (s <= -NAVAUX_DP_ONE) {
        s += NAVAUX_DP_ONE;
    }
    (void)AGC_ASSERT((s < NAVAUX_DP_ONE) && (s > -NAVAUX_DP_ONE));
    return (agc_dp_t)s;
}
