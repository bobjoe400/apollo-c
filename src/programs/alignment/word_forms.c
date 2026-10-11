/**********************************************************************
 * FILE NAME: word_forms.c
 *
 * PURPOSE:
 *   The words an interpretive DP result is left in (MPAC, MPAC +1), for
 *   the few alignment results that 1STO2S converts to CDU counts.  The
 *   library carries DP values; the AGC carries word pairs whose signs may
 *   disagree after AD/DAS, and 1TO2SUB reads the major word after DDOUBL,
 *   so the same value converts to counts differing by one.  The yaAGC
 *   oracle (tests/oracle/align.c: RRANGLES, SXTANG) shows the AGC result.
 *
 * AGC SOURCE:
 *   Luminary099 INTERPRETER.agc pp. 1084-1086 (ARCCOS, ACOSST2, SUBTR,
 *   ASINEX), INFLIGHT_ALIGNMENT_ROUTINES.agc p. 1349 (ARCTRIG),
 *   RTB_OP_CODES.agc pp. 1397-1398 (1STO2S, 1TO2SUB); Comanche055 the
 *   same code.
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name              Description
 *   ----------------  ----------------------------------------------
 *   agc_dp_high/low   sign-agreeing split of a DP value (word.h)
 *   agc_dp_abs/sub    value arithmetic for ARCTRIG's branch test
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none (pure functions; out-of-range inputs assert and give 0)
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles.  The ACOS DMPSUB product is taken as sign-agreeing
 *   (it is: a product of positive words).  MPAC +2 is not modelled.
 *
 * NOTES:
 *   ASINEX: ARCSIN(x) = QUARTER - ARCCOS(x) by DCS MPAC, AD QUARTER: the
 *   minor word is just negated, so for x > 0 the words are (+, -).  For
 *   x < 0 SUBTR first forms HALF - ARCCOS(-x) the same way, giving
 *   (-, +) after ASINEX.  Ones' complement AD of x and -x gives -0.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/imu/imu.h"
#include "agc/programs/alignment.h"

#define WORD_MAX   16383 /* largest 14-bit magnitude                    */
#define WORD_CARRY 16384 /* 2^14: one major-word unit in minor words    */
#define QUARTER_HI 4096  /* QUARTER = .25: major word                    */
#define HALF_HI    8192  /* HALF / HIDPHALF / LODPHALF = .5: major word  */
#define CDU_MOD    32768 /* one revolution in CDU counts                 */
#define CDU_MINUS_ZERO ((uint16_t) 077777) /* 1TO2SUB of -0: 2's comp -1 */
/* QTSN45 2DEC .1768 (= sin 45 deg / 4), Luminary099 rope 05520 26075.   */
#define QTSN45 ((agc_dp_t) 47459389)

/**********************************************************************
 * AGC: none (DLOAD of a stored DP) -- sign-agreeing words of a value
 *
 * FUNCTION NAME: align_words_of
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    DP value, any scale
 *
 * RETURNS: align_words_t -- (hi, lo) with agreeing signs; a negative
 *          value below one major unit has hi = -0
 **********************************************************************/
align_words_t align_words_of(agc_dp_t v)
{
    align_words_t w; /* the pair, same scale as v */

    (void) AGC_ASSERT((v >= AGC_DP_MIN) && (v <= AGC_DP_MAX));
    w.hi          = agc_dp_high(v);
    w.lo          = agc_dp_low(v);
    w.hi_neg_zero = (v < 0) && (w.hi == 0);
    (void) AGC_ASSERT((w.lo >= -WORD_MAX) && (w.lo <= WORD_MAX));

    return (w);
}

/**********************************************************************
 * AGC: none (ones' complement AD) -- add two SP words, end-around
 *
 * FUNCTION NAME: word_add
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  ------------------------------------
 *   a, b      int32_t    I    the words, |x| <= 16383 (+carry)
 *   a_nz      bool       I    a is -0
 *   b_nz      bool       I    b is -0
 *   sum       int32_t *  O    uncorrected sum (may exceed 14 bits)
 *
 * RETURNS: bool -- true when the sum is -0 (x + -x, or -0 + -0 / +0)
 **********************************************************************/
static bool word_add(int32_t a, bool a_nz, int32_t b, bool b_nz,
                     int32_t *sum)
{
    (void) AGC_ASSERT(sum != NULL);
    (void) AGC_ASSERT((a >= -(2 * WORD_MAX)) && (a <= 2 * WORD_MAX));
    *sum = a + b;

    /* Zero is +0 only when both operands are +0. */
    return ((*sum == 0) && ((a != 0) || a_nz || (b != 0) || b_nz));
}

/**********************************************************************
 * DAS (used by DAD / DSU / BDSU) -- add two word pairs
 *
 * FUNCTION NAME: align_words_add
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   a         align_words_t  I    MPAC pair (any scale)
 *   b         align_words_t  I    operand pair, same scale
 *
 * RETURNS: align_words_t -- the sum: minor words added, their overflow
 *          carried into the major word, the major word overflow-
 *          corrected (wraps by 2^14) as the AGC stores it
 **********************************************************************/
align_words_t align_words_add(align_words_t a, align_words_t b)
{
    align_words_t r;         /* result pair              */
    int32_t       lo    = 0; /* minor sum, 2^-28 units   */
    int32_t       hi    = 0; /* major sum, 2^-14 units   */
    int32_t       carry = 0; /* minor overflow, -1/0/+1  */

    (void) word_add(a.lo, false, b.lo, false, &lo);

    if (lo > WORD_MAX)
    {
        carry = 1;
        lo   -= WORD_CARRY;
    }
    else if (lo < -WORD_MAX)
    {
        carry = -1;
        lo   += WORD_CARRY;
    }
    else
    {
        /* no minor overflow */
    }

    r.hi_neg_zero = word_add(a.hi, a.hi_neg_zero, b.hi + carry, b.hi_neg_zero,
                             &hi);

    if (hi > WORD_MAX)
    {
        hi -= WORD_CARRY; /* overflow-corrected major word */
    }
    else if (hi < -WORD_MAX)
    {
        hi += WORD_CARRY;
    }
    else
    {
        /* no major overflow */
    }

    r.hi = (agc_sp_t) hi;
    r.lo = (agc_sp_t) lo;
    (void) AGC_ASSERT((r.hi >= -WORD_MAX) && (r.hi <= WORD_MAX));

    return (r);
}

/**********************************************************************
 * DCS (and SIGN for a negative operand) -- complement both words
 *
 * FUNCTION NAME: align_words_neg
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   a         align_words_t  I    pair to complement
 *
 * RETURNS: align_words_t -- (-hi, -lo); a +0 major word becomes -0
 **********************************************************************/
align_words_t align_words_neg(align_words_t a)
{
    align_words_t r; /* complemented pair */

    (void) AGC_ASSERT((a.hi >= -WORD_MAX) && (a.hi <= WORD_MAX));
    (void) AGC_ASSERT((a.lo >= -WORD_MAX) && (a.lo <= WORD_MAX));
    r.hi          = (agc_sp_t) -a.hi;
    r.lo          = (agc_sp_t) -a.lo;
    r.hi_neg_zero = (a.hi == 0) && !a.hi_neg_zero;

    return (r);
}

/**********************************************************************
 * ASINEX (INTERPRETER.agc p. 1085) -- words of an ARCSIN result
 *
 * FUNCTION NAME: align_asin_words
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    ARCSIN value, revs (B0), |v| <= .25
 *
 * RETURNS: align_words_t -- x >= 0: QUARTER - (c_hi, c_lo) with
 *          c = .25 - v = ARCCOS(x) (agreeing words): (4096 - c_hi,
 *          -c_lo).  x < 0: (c_hi - 4096, c_lo) with c = .25 + v
 *          (= ARCCOS(-x)) after SUBTR and ASINEX.
 **********************************************************************/
align_words_t align_asin_words(agc_dp_t v)
{
    const align_words_t quarter = {QUARTER_HI, 0, false}; /* QUARTER */
    align_words_t       c;      /* ARCCOS(|x|) words, positive        */

    (void) AGC_ASSERT((v >= -(AGC_DP_HALF / 2)) && (v <= AGC_DP_HALF / 2));

    if (v >= 0)
    {
        c = align_words_of((AGC_DP_HALF / 2) - v);

        return (align_words_add(align_words_neg(c), quarter));
    }

    /* SUBTR: HALF - c, then ASINEX: QUARTER - (HALF - c). */
    c = align_words_of((AGC_DP_HALF / 2) + v);
    const align_words_t half = {HALF_HI, 0, false};      /* HALF    */
    const align_words_t acos = align_words_add(align_words_neg(c), half);

    return (align_words_add(align_words_neg(acos), quarter));
}

/**********************************************************************
 * ACOSST2 / SUBTR (INTERPRETER.agc pp. 1084-1085) -- words of ARCCOS
 *
 * FUNCTION NAME: align_acos_words
 *
 * ARGUMENTS:
 *   Argument      Type      I/O  Description (scale)
 *   ------------  --------  ---  ------------------------------------
 *   v             agc_dp_t  I    ARCCOS value, revs (B0), 0 <= v <= .5
 *   arg_negative  bool      I    the argument was negative (SUBTR)
 *
 * RETURNS: align_words_t -- the DMPSUB product (agreeing) for a
 *          non-negative argument; HALF - ARCCOS(-x) as DCS, AD HALF,
 *          i.e. (8192 - c_hi, -c_lo), for a negative one
 **********************************************************************/
align_words_t align_acos_words(agc_dp_t v, bool arg_negative)
{
    (void) AGC_ASSERT((v >= 0) && (v <= AGC_DP_HALF));

    if (!arg_negative)
    {
        return (align_words_of(v));
    }

    const align_words_t half = {HALF_HI, 0, false}; /* HALF          */
    const align_words_t c    = align_words_of(AGC_DP_HALF - v);

    (void) AGC_ASSERT(c.hi >= 0);

    return (align_words_add(align_words_neg(c), half));
}

/**********************************************************************
 * ARCTRIG (INFLIGHT_ALIGNMENT_ROUTINES.agc p. 1349) -- result words
 *
 * FUNCTION NAME: align_arctrig_words
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter (overflow indicator)
 *   sinth     agc_dp_t        I    SINTH, sin/4 (B2)
 *   costh     agc_dp_t        I    COSTH, cos/4 (B2)
 *   theta     agc_dp_t        I    agc_imu_arctrig(sinth, costh), revs
 *
 * RETURNS: align_words_t -- (-45,45): the ASIN words; (135,-135):
 *          HIDPHALF SIGN SINTH, DSU THETA; otherwise the ACOS words,
 *          SIGN SINTH (complemented when SINTH < 0)
 **********************************************************************/
align_words_t align_arctrig_words(agc_interp_t *ip, agc_dp_t sinth,
                                  agc_dp_t costh, agc_dp_t theta)
{
    (void) AGC_ASSERT(ip != NULL);
    (void) AGC_ASSERT((theta >= -AGC_DP_HALF) && (theta <= AGC_DP_HALF));

    if (agc_dp_sub(ip, agc_dp_abs(sinth), QTSN45) >= 0)
    {
        /* ACOS(2 COSTH) SIGN SINTH: theta = +-ACOS value. */
        const agc_dp_t      a = (sinth < 0) ? -theta : theta; /* ACOS  */
        const align_words_t w = align_acos_words(a, costh < 0);

        return ((sinth < 0) ? align_words_neg(w) : w);
    }

    if (costh >= 0)
    {
        return (align_asin_words(theta)); /* TRIG1: THETA = ASIN   */
    }

    /* TRIG2: theta = (.5 SIGN SINTH) - ASIN; DSU of the ASIN words. */
    const agc_dp_t      half  = (sinth < 0) ? -AGC_DP_HALF : AGC_DP_HALF;
    const align_words_t asinw = align_asin_words(half - theta);
    align_words_t       h     = {HALF_HI, 0, false}; /* HIDPHALF  */

    if (sinth < 0)
    {
        h = align_words_neg(h); /* SIGN SINTH */
    }

    return (align_words_add(h, align_words_neg(asinw)));
}

/**********************************************************************
 * 1STO2S / 1TO2SUB (RTB_OP_CODES.agc pp. 1397-1398) on the words
 *
 * FUNCTION NAME: align_words_to_cdu
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   w         align_words_t  I    angle in revs (B0) as MPAC words
 *
 * RETURNS: uint16_t -- 2's complement CDU pattern (2^-15 rev): DDOUBL,
 *          the major word (with the minor word's carry); a negative
 *          major word converted by CCS/COM (same value), -0 -> 77777;
 *          a doubling overflow wraps by one revolution (LIMITS)
 **********************************************************************/
uint16_t align_words_to_cdu(align_words_t w)
{
    int32_t a     = 2 * (int32_t) w.hi; /* DDOUBL major word, counts */
    int32_t carry = 0;                  /* from 2 x minor word       */

    (void) AGC_ASSERT((w.lo >= -WORD_MAX) && (w.lo <= WORD_MAX));

    if (2 * (int32_t) w.lo > WORD_MAX)
    {
        carry = 1;
    }
    else if (2 * (int32_t) w.lo < -WORD_MAX)
    {
        carry = -1;
    }
    else
    {
        /* no carry */
    }

    a += carry;

    if ((a == 0) && w.hi_neg_zero && (carry == 0))
    {
        return (CDU_MINUS_ZERO); /* TS MPAC of -0 */
    }

    if (a > WORD_MAX)
    {
        a -= CDU_MOD; /* LIMITS: one revolution */
    }
    else if (a < -WORD_MAX - 1)
    {
        a += CDU_MOD;
    }
    else
    {
        /* in range */
    }

    (void) AGC_ASSERT((a >= -(CDU_MOD / 2)) && (a < CDU_MOD / 2));

    return ((uint16_t) ((a < 0) ? (a + CDU_MOD) : a));
}
