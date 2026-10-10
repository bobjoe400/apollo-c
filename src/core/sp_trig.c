/*
 * agc_sp_trig.c -- SPSIN / SPCOS, SINGLE_PRECISION_SUBROUTINES.agc p.1207
 * (identical in Comanche055 and Luminary099).
 *
 * Argument in A scaled at pi (v / 2^14 * pi radians, i.e. half-revolutions);
 * result scaled at 1. The polynomial is
 *     sin(pi/2 x) ~ 2 x (C1/2 + C3/2 x^2 + C5/2 x^4),  x scaled at pi/2,
 * evaluated with single-precision MP (major halves only) exactly as the AGC.
 */
#include "agc/core/assert.h"
#include "agc/core/constants.h"
#include "mpac.h"

/* Wrap a 16-bit accumulator value like TS: corrected value, *ovf = +-1.    */
static int32_t sp_ts(int32_t a16, int *ovf)
{
    if (!AGC_ASSERT(ovf != 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT((a16 > -2 * AGC_W_ONE) && (a16 < 2 * AGC_W_ONE)))
    {
        return (0); /* no valid result */
    }

    return (agc_w_ts(a16, ovf));
}

/* ARG90: INDEX A / CS LIMITS -> +-POSMAX for the +-1 left in A.             */
static agc_sp_t sp_arg90(int a)
{
    if (!AGC_ASSERT((a == 1) || (a == -1)))
    {
        return (0); /* no valid result */
    }

    _Static_assert(AGC_K_POSMAX == AGC_W_POSMAX, "AGC constant");

    return ((agc_sp_t) ((a > 0) ? AGC_K_POSMAX : -AGC_K_POSMAX));
}

/* POLLEY: x scaled at pi/2 in TEMK.                                         */
static agc_sp_t sp_polley(int32_t x)
{
    int32_t  sq;
    int32_t  a;
    agc_w2_t p;
    agc_w2_t two;
    int      o = 0;

    if (!AGC_ASSERT(agc_w_ok(x)))
    {
        return (0); /* no valid result */
    }

    sq = agc_w_mp(x, x).hi; /* MP TEMK; TS SQ   */

    if (!AGC_ASSERT(sq >= 0))
    {
        return (0); /* no valid result */
    }

    a   = agc_w_mp(sq, AGC_K_C5_2).hi + AGC_K_C3_2; /* MP C5/2; AD C3/2 */
    a   = agc_w_mp(a, sq).hi + AGC_K_C1_2;          /* MP SQ; AD C1/2   */
    p   = agc_w_mp(a, x);                           /* MP TEMK          */
    two = agc_w_das(p, p, &o);                      /* DDOUBL           */

    if (o != 0)
    { /* TS TEMK skips    */
        return (sp_arg90(o));
    }

    if (!AGC_ASSERT(agc_w_ok(two.hi)))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) two.hi);
}

/* SPSIN body from "TS TEMK" with the (possibly overflowed) accumulator.   */
static agc_sp_t sp_sine(int32_t a16)
{
    int     o = 0;
    int32_t t = sp_ts(a16, &o); /* SPSIN: TS TEMK   */
    int32_t x;

    if (!AGC_ASSERT(agc_w_ok(t)))
    {
        return (0); /* no valid result */
    }

    if (o != 0)
    {           /* arg beyond +-pi  */
        t = -t; /* CS TEMK          */
    }

    x = sp_ts(2 * t, &o); /* SPT: DOUBLE; TS  */

    if (o == 0)
    {
        return (sp_polley(x)); /* |arg| < pi/2     */
    }

    /* XCH TEMK; INDEX TEMK; AD LIMITS; COM; AD TEMK: x -> +-1.0 - x       */
    x = (o > 0) ? (x - AGC_K_POSMAX) : (x + AGC_K_POSMAX);
    x = -x + o;

    if (!AGC_ASSERT((x >= -AGC_W_ONE) && (x <= AGC_W_ONE)))
    {
        return (0); /* no valid result */
    }

    x = sp_ts(x, &o); /* TS TEMK          */

    if (o != 0)
    {
        return (sp_arg90(o)); /* arg = +-pi/2     */
    }

    return (sp_polley(x));
}

/* SPSIN (p.1207): sin(pi a), a scaled at pi; result scaled at 1.            */
agc_sp_t agc_sp_sin(agc_sp_t a)
{
    if (!AGC_ASSERT(a >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(a <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return (sp_sine((int32_t) a));
}

/* SPCOS (p.1207): AD HALF then SPSIN, i.e. cos(pi a) = sin(pi (a + 1/2)).  */
agc_sp_t agc_sp_cos(agc_sp_t a)
{
    if (!AGC_ASSERT(a >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(a <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return (sp_sine((int32_t) a + AGC_K_HALF));
}
