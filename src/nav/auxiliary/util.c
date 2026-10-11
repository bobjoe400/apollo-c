/**********************************************************************
 * FILE NAME: util.c
 *
 * PURPOSE:
 *   Interpretive-behaviour helpers shared by the nav-aux files (see
 *   internal.h): range predicates, shifts by an indexed (signed) count,
 *   and the low words of a TP.
 *
 * AGC SOURCE:
 *   INTERPRETER.agc (SL*, SR*, VSL*, VSR* indexed shifts; DLOAD X+1)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dp_shl/shr        SL / SR (interp.h)
 *   agc_vec_shl/shr       VSL / VSR (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none (a failed assertion returns 0 / the zero vector)
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Shift counts within +-NAVAUX_MAX_SHIFT.
 *
 * NOTES:
 *   DEVIATION: agc_navaux_tp_low_dp works on the integer representation
 *   of agc_tp_t (value = raw / 2^42) to split off the low two words.
 *
 * ALGORITHM:
 *   n > 0 shifts left (SL, overflow sets OVFIND), n < 0 shifts right by
 *   -n (SR, truncating), n = 0 leaves the value.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "internal.h"

#define NAVAUX_DP_ONE ((int64_t) 1 << AGC_DP_BITS) /* 1.0 in DP raw units */

/**********************************************************************
 * AGC: none (range predicate) -- true when a is a valid DP
 *
 * FUNCTION NAME: agc_navaux_dp_ok
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   a         agc_dp_t  I    value, any scale
 *
 * RETURNS: bool -- true when |a| <= DPPOSMAX
 **********************************************************************/
bool agc_navaux_dp_ok(agc_dp_t a)
{
    /* No AGC_ASSERT here: this is the predicate the assertions use.        */
    return ((a <= AGC_DP_MAX) && (a >= AGC_DP_MIN));
}

/**********************************************************************
 * AGC: none (range predicate) -- true when every component is a valid DP
 *
 * FUNCTION NAME: agc_navaux_vec_ok
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  ------------------------------------
 *   v         agc_vec_t  I    vector, any scale
 *
 * RETURNS: bool -- true when all three components are in range
 **********************************************************************/
bool agc_navaux_vec_ok(agc_vec_t v)
{
    /* No AGC_ASSERT here: this is the predicate the assertions use.        */
    return (agc_navaux_dp_ok(v.c[0]) && agc_navaux_dp_ok(v.c[1]) && agc_navaux_dp_ok(v.c[2]));
}

/**********************************************************************
 * AGC: none (range predicate) -- true when a is a valid TP
 *
 * FUNCTION NAME: agc_navaux_tp_ok
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   a         agc_tp_t  I    value, any scale
 *
 * RETURNS: bool -- true when |a| <= the TP maximum
 **********************************************************************/
bool agc_navaux_tp_ok(agc_tp_t a)
{
    /* No AGC_ASSERT here: this is the predicate the assertions use.        */
    return ((a <= AGC_TP_MAX) && (a >= AGC_TP_MIN));
}

/**********************************************************************
 * AGC: SL* / SR* X,n (INTERPRETER.agc) -- DP shift by an indexed count
 *
 * FUNCTION NAME: agc_navaux_dp_shift
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND on SL)
 *   a         agc_dp_t        I    value, scale 2^s
 *   n         int             I    shift count, |n| <= NAVAUX_MAX_SHIFT
 *
 * RETURNS: agc_dp_t -- a 2^n, i.e. scale 2^(s-n); 0 on bad arguments
 **********************************************************************/
agc_dp_t agc_navaux_dp_shift(agc_interp_t *ip, agc_dp_t a, int n)
{
    if (!AGC_ASSERT((n <= NAVAUX_MAX_SHIFT) && (n >= -NAVAUX_MAX_SHIFT)))
    {
        return (0);
    }

    agc_dp_t r = a; /* shifted value, scale 2^(s-n) */

    if (n > 0)
    {
        r = agc_dp_shl(ip, a, n);
    }
    else if (n < 0)
    {
        r = agc_dp_shr(a, -n);
    }

    if (!AGC_ASSERT((n >= 0) || (agc_dp_abs(r) <= agc_dp_abs(a))))
    {
        return (0); /* post: SR never grows the magnitude */
    }

    return (r);
}

/**********************************************************************
 * AGC: VSL* / VSR* X,n (INTERPRETER.agc) -- vector shift by an indexed
 *   count
 *
 * FUNCTION NAME: agc_navaux_vec_shift
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND on VSL)
 *   v         agc_vec_t       I    vector, scale 2^s
 *   n         int             I    shift count, |n| <= NAVAUX_MAX_SHIFT
 *
 * RETURNS: agc_vec_t -- v 2^n, i.e. scale 2^(s-n); zero on bad arguments
 **********************************************************************/
agc_vec_t agc_navaux_vec_shift(agc_interp_t *ip, agc_vec_t v, int n)
{
    if (!AGC_ASSERT(agc_navaux_vec_ok(v)))
    {
        return (agc_vec_zero());
    }

    if (!AGC_ASSERT((n <= NAVAUX_MAX_SHIFT) && (n >= -NAVAUX_MAX_SHIFT)))
    {
        return (agc_vec_zero());
    }

    agc_vec_t r = v; /* shifted vector, scale 2^(s-n) */

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

    if (!AGC_ASSERT((n >= 0) || ((agc_dp_abs(r.c[0]) <= agc_dp_abs(v.c[0])) && (agc_dp_abs(r.c[1]) <= agc_dp_abs(v.c[1])) && (agc_dp_abs(r.c[2]) <= agc_dp_abs(v.c[2])))))
    {
        return (agc_vec_zero()); /* post: VSR never grows a component */
    }

    return (r);
} /* agc_navaux_vec_shift */

/**********************************************************************
 * AGC: DLOAD X+1 of a TP X (INTERPRETER.agc) -- the two low-order words
 *   as a DP fraction
 *
 * FUNCTION NAME: agc_navaux_tp_low_dp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_tp_t  I    TP, scale 2^s (taken as sign-agreed)
 *
 * RETURNS: agc_dp_t -- MPAC+1, MPAC+2 of v, scale 2^(s-14), with the
 *          sign of the whole; 0 on bad input
 **********************************************************************/
agc_dp_t agc_navaux_tp_low_dp(agc_tp_t v)
{
    int64_t mag = v; /* |v|, raw TP units       */
    int64_t low = 0; /* low words, raw DP units */

    if (v < 0)
    {
        mag = -v;
    }

    low = mag % NAVAUX_DP_ONE;

    if (!AGC_ASSERT((low >= 0) && (low < NAVAUX_DP_ONE)))
    {
        return (0); /* post: a proper DP fraction */
    }

    if (v < 0)
    {
        low = -low;
    }

    if (!AGC_ASSERT((low == 0) || ((low < 0) == (v < 0))))
    {
        return (0); /* post: the low words carry the sign of v */
    }

    return ((agc_dp_t) low);
}
