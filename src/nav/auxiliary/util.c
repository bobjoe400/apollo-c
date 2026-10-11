/*
 * util.c -- interpretive-behaviour helpers shared by the nav-aux
 * files (see internal.h).
 *
 * DEVIATION: agc_interp.h has no TP shift/normalize and its DP operations
 * saturate on overflow.  The helpers below therefore work on the integer
 * representation of agc_tp_t / agc_dp_t directly (value = raw / 2^42 or
 * raw / 2^28).  This is confined to this file and documented per helper.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "internal.h"

#define NAVAUX_DP_ONE  ((int64_t) 1 << AGC_DP_BITS) /* 1.0 in DP raw units */

bool agc_navaux_dp_ok(agc_dp_t a)
{
    return ((a <= AGC_DP_MAX) && (a >= AGC_DP_MIN));
}

bool agc_navaux_vec_ok(agc_vec_t v)
{
    return (agc_navaux_dp_ok(v.c[0]) && agc_navaux_dp_ok(v.c[1]) &&
            agc_navaux_dp_ok(v.c[2]));
}

bool agc_navaux_tp_ok(agc_tp_t a)
{
    return ((a <= AGC_TP_MAX) && (a >= AGC_TP_MIN));
}

/* SL* / SR* X,n with an indexed (signed) shift count. */
agc_dp_t agc_navaux_dp_shift(agc_interp_t *ip, agc_dp_t a, int n)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0);
    }

    if (!AGC_ASSERT((n <= NAVAUX_MAX_SHIFT) && (n >= -NAVAUX_MAX_SHIFT)))
    {
        return (0);
    }

    agc_dp_t r = a;

    if (n > 0)
    {
        r = agc_dp_shl(ip, a, n);
    }
    else if (n < 0)
    {
        r = agc_dp_shr(a, -n);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(r))) /* post: in range         */
    {
        return (0);
    }

    return (r);
}

/* VSL* / VSR* with an indexed (signed) shift count. */
agc_vec_t agc_navaux_vec_shift(agc_interp_t *ip, agc_vec_t v, int n)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (agc_vec_zero());
    }

    if (!AGC_ASSERT((n <= NAVAUX_MAX_SHIFT) && (n >= -NAVAUX_MAX_SHIFT)))
    {
        return (agc_vec_zero());
    }

    agc_vec_t r = v;

    if (n > 0)
    {
        r = agc_vec_shl(ip, v, n);
    }
    else if (n < 0)
    {
        r = agc_vec_shr(v, -n);
    }

    if (!AGC_ASSERT(agc_navaux_vec_ok(r))) /* post: in range         */
    {
        return (agc_vec_zero());
    }

    return (r);
}

/* DLOAD X+1 of a TP X: the two low-order words as a DP fraction (the TP
 * is taken as sign-agreed, so the low words carry the sign of the whole). */
agc_dp_t agc_navaux_tp_low_dp(agc_tp_t v)
{
    if (!AGC_ASSERT(agc_navaux_tp_ok(v)))
    {
        return (0);
    }

    int64_t mag = (v < 0) ? -v : v;
    int64_t low = mag % NAVAUX_DP_ONE;

    if (!AGC_ASSERT(low < NAVAUX_DP_ONE))
    {
        return (0);
    }

    return ((agc_dp_t) ((v < 0) ? -low : low));
}

