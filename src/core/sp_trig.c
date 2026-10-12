/**********************************************************************
 * FILE NAME: sp_trig.c
 *
 * PURPOSE:
 *   Single-precision sine and cosine (SPSIN, SPCOS, with their tails
 *   SPT, POLLEY and ARG90), the basic-language trig used by the DAPs and
 *   the IMU routines.
 *
 * AGC SOURCE:
 *   Comanche055 SINGLE_PRECISION_SUBROUTINES.agc p.1207 (SPCOS, SPSIN,
 *   SPT, POLLEY, ARG90)
 *   Luminary099 SINGLE_PRECISION_SUBROUTINES.agc p.1207 (identical)
 *
 * EXTERNAL VARIABLES:
 *   none (TEMK and SQ are locals here)
 *
 * EXTERNAL REFERENCES:
 *   Name          Description
 *   ------------  ------------------------------------------------
 *   agc_w_mp      MP: SP x SP -> two words (mpac.h)
 *   agc_w_ts      TS with overflow skip (mpac.h)
 *   agc_w_das     DAS / DDOUBL on a word pair (mpac.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; an argument of exactly +-1 (pi/2 after DOUBLE) gives
 *   +-POSMAX through ARG90.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Argument in A scaled at pi (v / 2^14 * pi rad, half-revolutions);
 *   result scaled at 1. Both vehicles. Bit-exact with the AGC for all
 *   32,767 SP arguments (scratch oracle run, docs/modules/core-numeric.md).
 *
 * ALGORITHM:
 *   SPCOS adds HALF (pi/2) and falls into SPSIN. An argument beyond
 *   +-pi/2 is reflected (pi - x) using the overflow of DOUBLE, then
 *   sin(pi/2 x) ~ 2 x (C1/2 + C3/2 x^2 + C5/2 x^4) is evaluated with SP
 *   MP keeping only the major word of each product, exactly as the AGC.
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/constants.h"
#include "mpac.h"

/**********************************************************************
 * ARG90 (SINGLE_PRECISION_SUBROUTINES.agc p.1207) -- INDEX A / CS LIMITS
 * for the +-1 the overflowing TS left in A: +-POSMAX (sin = +-1).
 *
 * ARGUMENTS:
 *   a  int  I  overflow indicator left in A, +1 or -1
 *
 * RETURNS: agc_sp_t -- +POSMAX for +1, -POSMAX for -1 (scaled at 1)
 **********************************************************************/
static agc_sp_t sp_arg90(int a)
{
    _Static_assert(AGC_K_POSMAX == AGC_W_POSMAX, "AGC constant");

    /* LIMITS-1 = POSMAX, LIMITS+1 = BIT15 (NEGMAX)                      */
    return ((agc_sp_t) ((a > 0) ? AGC_K_POSMAX : -AGC_K_POSMAX)); /* CS  */
}

/**********************************************************************
 * POLLEY (SINGLE_PRECISION_SUBROUTINES.agc p.1207) -- the odd polynomial
 * 2 x (C1/2 + C3/2 x^2 + C5/2 x^4), each MP keeping its major word.
 *
 * ARGUMENTS:
 *   x  int32_t  I  angle in TEMK, scaled at pi/2, a valid 15-bit word
 *
 * RETURNS: agc_sp_t -- sin(pi/2 x) scaled at 1; +-POSMAX when DDOUBL
 *          overflows (ARG90)
 **********************************************************************/
static agc_sp_t sp_polley(int32_t x)
{
    int32_t sq;    /* SQ: x^2, scaled at (pi/2)^2              */
    int32_t a;     /* accumulator, scaled at 1                 */
    agc_w2_t p;     /* MP TEMK product (A, L), scaled at 1/2    */
    agc_w2_t two;   /* DDOUBL of p, scaled at 1                 */
    int o = 0; /* DDOUBL overflow, +-1 or 0                */

    sq = agc_w_mp(x, x).hi; /* MP TEMK; TS SQ   */

    a = agc_w_mp(sq, AGC_K_C5_2).hi + AGC_K_C3_2; /* MP C5/2; AD C3/2 */
    a = agc_w_mp(a, sq).hi + AGC_K_C1_2;          /* MP SQ; AD C1/2   */
    p = agc_w_mp(a, x);                           /* MP TEMK          */
    two = agc_w_das(p, p, &o);                      /* DDOUBL           */

    if (o != 0)
    { /* TS TEMK skips    */
        return (sp_arg90(o));
    }

    return ((agc_sp_t) two.hi);
}

/**********************************************************************
 * SPSIN from "TS TEMK", SPT (SINGLE_PRECISION_SUBROUTINES.agc p.1207) --
 * the shared body of SPSIN and SPCOS, entered with the accumulator as
 * SPCOS's AD HALF may leave it (overflowed, 16 bits).
 *
 * ARGUMENTS:
 *   a16  int32_t  I  accumulator, scaled at pi, |a16| < 2 (16-bit A)
 *
 * RETURNS: agc_sp_t -- sin(pi a16) scaled at 1
 **********************************************************************/
static agc_sp_t sp_sine(int32_t a16)
{
    int o = 0;                 /* overflow of the last TS, +-1 or 0 */
    int32_t t = agc_w_ts(a16, &o); /* TEMK, scaled at pi                */
    int32_t x;                     /* TEMK after SPT, scaled at pi/2    */

    if (o != 0)
    { /* arg beyond +-pi  */
        t = -t; /* CS TEMK          */
    }

    x = agc_w_ts(2 * t, &o); /* SPT: DOUBLE; TS  */

    if (o == 0)
    {
        return (sp_polley(x)); /* |arg| < pi/2     */
    }

    /* XCH TEMK; INDEX TEMK; AD LIMITS; COM; AD TEMK: x -> +-1.0 - x       */
    x = (o > 0) ? (x - AGC_K_POSMAX) : (x + AGC_K_POSMAX); /* AD LIMITS+-1 */
    x = -x + o;

    x = agc_w_ts(x, &o); /* TS TEMK          */

    if (o != 0)
    {
        return (sp_arg90(o)); /* arg = +-pi/2     */
    }

    return (sp_polley(x));
}

/**********************************************************************
 * SPSIN (SINGLE_PRECISION_SUBROUTINES.agc p.1207) -- sin(pi a).
 * Kept as the public name over sp_sine: sp_sine also takes the 16-bit
 * accumulator SPCOS forms, which an agc_sp_t cannot hold.
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  angle scaled at pi (half-revolutions), any SP word
 *
 * RETURNS: agc_sp_t -- sin(pi a) scaled at 1 (|err| <= 4.2 LSB)
 **********************************************************************/
agc_sp_t agc_sp_sin(agc_sp_t a)
{
    return (sp_sine((int32_t) a));
}

/**********************************************************************
 * SPCOS (SINGLE_PRECISION_SUBROUTINES.agc p.1207) -- AD HALF, then SPSIN:
 * cos(pi a) = sin(pi (a + 1/2)); the sum may overflow (16-bit A).
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  angle scaled at pi (half-revolutions), any SP word
 *
 * RETURNS: agc_sp_t -- cos(pi a) scaled at 1 (|err| <= 4.2 LSB)
 **********************************************************************/
agc_sp_t agc_sp_cos(agc_sp_t a)
{
    return (sp_sine((int32_t) a + AGC_K_HALF));
}
