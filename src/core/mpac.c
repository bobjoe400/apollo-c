/**********************************************************************
 * FILE NAME: mpac.c
 *
 * PURPOSE:
 *   Word-level model of MPAC arithmetic: the AGC instructions (MP, TS,
 *   DAS, DV, DCS) on 15-bit ones'-complement words held in int32_t, and
 *   the interpreter subroutines built from them (OVERFLOW, DMPSUB,
 *   ROUNDSUB, DSQSUB, DOTSUB, VSQSUB, POLY), instruction by instruction.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc "ARITHMETIC SUBROUTINES REQUIRED IN
 *   FIXED-FIXED" pp.1131-1140, 1154-1157, 1183 (OVERFLOW, ENDVXV,
 *   DMPSUB, ROUNDSUB, DOTSUB, ENDDOT, POLY, DSQSUB, VSQSUB, TSSL)
 *   Luminary099 INTERPRETER.agc pp.1026-1035, 1078 (identical)
 *
 * EXTERNAL VARIABLES:
 *   none (MPAC, BUF, OVFIND are passed as values / out-parameters)
 *
 * EXTERNAL REFERENCES:
 *   none outside this file and agc_assert
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; overflow is reported through OVFIND or the A out-parameter
 *   exactly as the AGC leaves it.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Words are in (-2^14, 2^14); a 16-bit accumulator value (|a| < 2^15)
 *   is allowed only where the AGC holds one in A. Negative zero is not
 *   modelled (mpac.h). Representation: see mpac.h.
 **********************************************************************/
#include "agc/core/assert.h"
#include "mpac.h"

/* ---- representation ----------------------------------------------------- */

/**********************************************************************
 * AGC: none (representation predicate) -- a valid 15-bit word.
 * No AGC_ASSERT here: this is the predicate the assertions use.
 *
 * ARGUMENTS:
 *   w  int32_t  I  candidate word, scaled 2^-14 per unit
 *
 * RETURNS: bool -- true when -2^14 < w < 2^14
 **********************************************************************/
bool agc_w_ok(int32_t w)
{
    /* No AGC_ASSERT here: this is the predicate the assertions use.        */
    return ((w > -AGC_W_ONE) && (w < AGC_W_ONE));
}

/**********************************************************************
 * AGC: none (representation) -- split a DP into the sign-agreeing word
 * pair the AGC holds after DCA of a stored DP (pp.1131ff).
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I  DP value, 2^-28 per unit, clamped to +-DPPOSMAX
 *
 * RETURNS: agc_w2_t -- {hi, lo} with lo carrying the sign of v
 **********************************************************************/
agc_w2_t agc_w_split(agc_dp_t v)
{
    agc_w2_t p = {0, 0}; /* {MPAC, MPAC+1} */

    if (!AGC_ASSERT((v >= AGC_DP_MIN) && (v <= AGC_DP_MAX)))
    {
        v = (v < 0) ? AGC_DP_MIN : AGC_DP_MAX; /* clamp to the DP range */
    }

    p.hi = v / AGC_W_ONE; /* C division truncates toward zero */
    p.lo = v % AGC_W_ONE; /* so lo carries the sign of v      */

    return (p);
}

/**********************************************************************
 * AGC: none (representation) -- the value of a word pair, whatever the
 * signs of its words.
 *
 * ARGUMENTS:
 *   p  agc_w2_t  I  word pair, each word valid
 *
 * RETURNS: agc_dp_t -- hi 2^14 + lo, 2^-28 per unit
 **********************************************************************/
agc_dp_t agc_w_join(agc_w2_t p)
{
    if (!AGC_ASSERT(agc_w_ok(p.hi)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(agc_w_ok(p.lo)))
    {
        return (0); /* no valid result */
    }

    return ((agc_dp_t) (p.hi * AGC_W_ONE + p.lo));
}

/**********************************************************************
 * AGC: none (representation) -- split a TP into MPAC, MPAC+1, MPAC+2
 * with all three words agreeing in sign.
 *
 * ARGUMENTS:
 *   v  agc_tp_t  I  TP value, 2^-42 per unit, clamped to the TP range
 *
 * RETURNS: agc_w3_t -- {m0, m1, m2}
 **********************************************************************/
agc_w3_t agc_w3_from_tp(agc_tp_t v)
{
    agc_w3_t m = {0, 0, 0};

    if (!AGC_ASSERT((v >= AGC_TP_MIN) && (v <= AGC_TP_MAX)))
    {
        v = (v < 0) ? AGC_TP_MIN : AGC_TP_MAX; /* clamp to the TP range */
    }

    m.m0 = (int32_t) (v / ((agc_tp_t) AGC_W_ONE * AGC_W_ONE));
    m.m1 = (int32_t) ((v / AGC_W_ONE) % AGC_W_ONE);
    m.m2 = (int32_t) (v % AGC_W_ONE);

    return (m);
}

/**********************************************************************
 * AGC: none (representation) -- the value of a TP MPAC.
 *
 * ARGUMENTS:
 *   m  agc_w3_t  I  MPAC, MPAC+1, MPAC+2 (signs may disagree)
 *
 * RETURNS: agc_tp_t -- (m0 2^14 + m1) 2^14 + m2, 2^-42 per unit
 **********************************************************************/
agc_tp_t agc_w3_value(agc_w3_t m)
{
    if (!AGC_ASSERT(agc_w_ok(m.m0) && agc_w_ok(m.m1)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(agc_w_ok(m.m2)))
    {
        return (0); /* no valid result */
    }

    return (((agc_tp_t) m.m0 * AGC_W_ONE + m.m1) * AGC_W_ONE + m.m2);
}

/**********************************************************************
 * AGC: none (representation) -- MPAC, MPAC+1 of a TP MPAC (the DP the
 * interpreter stores; MPAC+2 dropped, no rounding).
 *
 * ARGUMENTS:
 *   m  agc_w3_t  I  TP MPAC
 *
 * RETURNS: agc_w2_t -- {m0, m1}
 **********************************************************************/
agc_w2_t agc_w3_top(agc_w3_t m)
{
    agc_w2_t p; /* MPAC, MPAC+1 */

    if (!AGC_ASSERT(agc_w_ok(m.m0)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(agc_w_ok(m.m1)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    p.hi = m.m0;
    p.lo = m.m1;

    return (p);
}

/**********************************************************************
 * BRANCH (INTERPRETER.agc p.1130) -- CCS MPAC, MPAC+1, MPAC+2 in turn:
 * the sign of the first non-zero word.
 *
 * ARGUMENTS:
 *   m  agc_w3_t  I  TP MPAC
 *
 * RETURNS: int -- +1, 0 (all three zero) or -1
 **********************************************************************/
int agc_w_branch(agc_w3_t m)
{
    int32_t w = m.m0; /* first non-zero word, or 0 */

    if (!AGC_ASSERT(agc_w_ok(m.m0) && agc_w_ok(m.m1)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(agc_w_ok(m.m2)))
    {
        return (0); /* no valid result */
    }

    if (w == 0)
    {
        w = (m.m1 != 0) ? m.m1 : m.m2; /* CCS MPAC +1, then MPAC +2 */
    }

    return ((w > 0) - (w < 0));
}

/**********************************************************************
 * AGC: none (representation) -- a public DP word pair as model words.
 *
 * ARGUMENTS:
 *   w  agc_dp_words_t  I  the two words as stored
 *
 * RETURNS: agc_w2_t -- {hi, lo}, unchanged
 **********************************************************************/
agc_w2_t agc_w2_from_words(agc_dp_words_t w)
{
    agc_w2_t p = {w.hi, w.lo};

    if (!AGC_ASSERT(agc_w_ok(p.hi) && agc_w_ok(p.lo)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    return (p);
}

/**********************************************************************
 * AGC: none (representation) -- model words as a public DP word pair.
 *
 * ARGUMENTS:
 *   p  agc_w2_t  I  MPAC, MPAC+1
 *
 * RETURNS: agc_dp_words_t -- the same two words
 **********************************************************************/
agc_dp_words_t agc_w2_to_words(agc_w2_t p)
{
    agc_dp_words_t w = {0, 0};

    w.hi = (agc_sp_t) p.hi;
    w.lo = (agc_sp_t) p.lo;

    return (w);
}

/**********************************************************************
 * AGC: none (representation) -- a public TP word triple as model words.
 *
 * ARGUMENTS:
 *   m  agc_tp_words_t  I  MPAC, MPAC+1, MPAC+2
 *
 * RETURNS: agc_w3_t -- {m0, m1, m2}, unchanged
 **********************************************************************/
agc_w3_t agc_w3_from_words(agc_tp_words_t m)
{
    agc_w3_t r = {m.hi, m.mid, m.lo};

    if (!AGC_ASSERT(agc_w_ok(r.m0) && agc_w_ok(r.m1) && agc_w_ok(r.m2)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * AGC: none (representation) -- model words as a public TP triple.
 *
 * ARGUMENTS:
 *   m  agc_w3_t  I  MPAC, MPAC+1, MPAC+2
 *
 * RETURNS: agc_tp_words_t -- the same three words
 **********************************************************************/
agc_tp_words_t agc_w3_to_words(agc_w3_t m)
{
    agc_tp_words_t r = {0, 0, 0};

    if (!AGC_ASSERT(agc_w_ok(m.m0) && agc_w_ok(m.m1) && agc_w_ok(m.m2)))
    {
        return (r);
    }

    r.hi  = (agc_sp_t) m.m0;
    r.mid = (agc_sp_t) m.m1;
    r.lo  = (agc_sp_t) m.m2;

    return (r);
}

/* ---- single instructions ------------------------------------------------ */

/**********************************************************************
 * AGC: none (basic instruction MP) -- SP x SP
 * into A, L; both words carry the sign of the product.
 *
 * ARGUMENTS:
 *   a  int32_t  I  multiplicand word, scaled 2^-14 per unit
 *   b  int32_t  I  multiplier word, scaled 2^-14 per unit
 *
 * RETURNS: agc_w2_t -- {A, L}: a b at 2^-28 per unit
 **********************************************************************/
agc_w2_t agc_w_mp(int32_t a, int32_t b)
{
    agc_w2_t r = {0, 0};

    if (!AGC_ASSERT(agc_w_ok(a)) || !AGC_ASSERT(agc_w_ok(b)))
    {
        return (r);
    }

    r.hi = (a * b) / AGC_W_ONE;
    r.lo = (a * b) % AGC_W_ONE;

    if (!AGC_ASSERT(agc_w_ok(r.hi)))
    {
        return (r);
    }

    return (r);
}

/**********************************************************************
 * AGC: none (basic instruction TS) -- store a 16-bit accumulator: the stored word
 * is overflow-corrected and A is left +-1 on overflow (TS skips).
 *
 * ARGUMENTS:
 *   a16  int32_t  I  accumulator, |a16| < 2^15, 2^-14 per unit
 *   ovf  int *    O  overflow left in A: +1, -1 or 0
 *
 * RETURNS: int32_t -- the stored (overflow-corrected) word
 **********************************************************************/
int32_t agc_w_ts(int32_t a16, int *ovf)
{
    int o = 0; /* overflow of the TS, +-1 or 0 */

    if (!AGC_ASSERT((a16 > -2 * AGC_W_ONE) && (a16 < 2 * AGC_W_ONE)))
    {
        return (0); /* no valid result */
    }

    if (a16 >= AGC_W_ONE)
    {
        a16 -= AGC_W_ONE;
        o   = 1;
    }
    else if (a16 <= -AGC_W_ONE)
    {
        a16 += AGC_W_ONE;
        o   = -1;
    }

    if (AGC_ASSERT(ovf != 0))
    {
        *ovf = o;
    }

    return (a16);
}

/**********************************************************************
 * DOUBLE; TS L (ROUNDSUB p.1137) -- only the overflow indication TS
 * leaves in A, used as the rounding carry.
 *
 * ARGUMENTS:
 *   a16  int32_t  I  accumulator, |a16| < 2^15
 *
 * RETURNS: int -- +1 if a16 >= 2^14, -1 if a16 <= -2^14, else 0
 **********************************************************************/
int agc_w_ovf(int32_t a16)
{
    if (!AGC_ASSERT(a16 > -2 * AGC_W_ONE))
    {
        return (0); /* below the 16-bit accumulator: no carry */
    }

    if (!AGC_ASSERT(a16 < 2 * AGC_W_ONE))
    {
        return (0); /* above the 16-bit accumulator: no carry */
    }

    if (a16 >= AGC_W_ONE)
    {
        return (1);
    }

    return ((a16 <= -AGC_W_ONE) ? -1 : 0); /* negative overflow */
}

/**********************************************************************
 * AGC: none (basic instructions DAS, DDOUBL) -- (x.hi, x.lo) + (y.hi, y.lo): the
 * minor sum carries into the major word, the major word is
 * overflow-corrected; y.hi may be an uncorrected 16-bit accumulator
 * (as after DDOUBL).
 *
 * ARGUMENTS:
 *   x    agc_w2_t  I  augend pair (the erasable DAS adds into)
 *   y    agc_w2_t  I  addend pair (A, L)
 *   ovf  int *     O  net overflow DAS leaves in A: +1, -1 or 0
 *
 * RETURNS: agc_w2_t -- the stored sum; the words may disagree in sign
 **********************************************************************/
agc_w2_t agc_w_das(agc_w2_t x, agc_w2_t y, int *ovf)
{
    agc_w2_t r;         /* stored sum */
    int      carry = 0; /* minor-word overflow into the major word */

    if (!AGC_ASSERT(agc_w_ok(x.lo) && agc_w_ok(y.lo)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    r.lo = agc_w_ts(x.lo + y.lo, &carry);
    r.hi = agc_w_ts(x.hi + y.hi + carry, ovf);

    return (r);
}

/**********************************************************************
 * AGC: none (basic instruction DV) -- divide the DP accumulator (A, L) by k. The
 * AGC forces sign agreement in A, L first, so only the value matters;
 * quotient and remainder follow C's truncating division.
 *
 * ARGUMENTS:
 *   a  int32_t  I  A, major word of the dividend
 *   l  int32_t  I  L, minor word of the dividend
 *   k  int32_t  I  divisor word, |a 2^14 + l| < |k| 2^14
 *
 * RETURNS: agc_w2_t -- {quotient, remainder}; {POSMAX, 0} for k = 0
 **********************************************************************/
agc_w2_t agc_w_dv(int32_t a, int32_t l, int32_t k)
{
    agc_w2_t r        = {AGC_W_POSMAX, 0};
    int32_t  dividend = a * AGC_W_ONE + l; /* A, L value, 2^-28 per unit */

    if (!AGC_ASSERT(k != 0) || !AGC_ASSERT(agc_w_ok(l)))
    {
        return (r);
    }

    r.hi = dividend / k;
    r.lo = dividend % k;

    if (!AGC_ASSERT(agc_w_ok(r.hi)))
    {
        return (r);
    }

    return (r);
}

/**********************************************************************
 * AGC: none (basic instruction DCS) -- complement both words of a pair.
 *
 * ARGUMENTS:
 *   p  agc_w2_t  I  word pair
 *
 * RETURNS: agc_w2_t -- {-hi, -lo}
 **********************************************************************/
agc_w2_t agc_w_dcs(agc_w2_t p)
{
    agc_w2_t r; /* complemented pair */

    if (!AGC_ASSERT(agc_w_ok(p.hi)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(agc_w_ok(p.lo)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    r.hi = -p.hi;
    r.lo = -p.lo;

    return (r);
}

/* ---- interpreter subroutines -------------------------------------------- */

/**********************************************************************
 * OVERFLOW (INTERPRETER.agc p.1133) -- entered after a DAS left A = +-1:
 * adds +-1.0 to the pair as POSMAX to the major word and POSMAX+1 to the
 * minor word, which forces sign agreement; OVFIND is set only if the
 * pair still overflows. The value is left wrapped modulo 1.0.
 *
 * ARGUMENTS:
 *   m       agc_w2_t     I  MPAC pair after the DAS
 *   a       int          I  overflow the DAS left in A, +1 or -1
 *   ovfind  agc_ovf_t *  O  OVFIND, written only on a second overflow
 *
 * RETURNS: agc_w2_t -- the corrected pair (signs agree)
 **********************************************************************/
agc_w2_t agc_w_overflow(agc_w2_t m, int a, agc_ovf_t *ovfind)
{
    int32_t buf;   /* BUF: +-POSMAX, sign of the overflow */
    int     o = 0; /* overflow of each ADS, +-1 or 0 */

    buf  = (a > 0) ? AGC_W_POSMAX : -AGC_W_POSMAX; /* CS LIMITS      */ /* sign of a */
    m.lo = agc_w_ts(m.lo + buf + a, &o);           /* AUG A, ADS     */
    m.hi = agc_w_ts(m.hi + buf + o, &o);           /* AD BUF, ADS    */

    if (o != 0)
    {
        *ovfind = (agc_ovf_t) o; /* SETOVF2        */
    }

    return (m);
}

/**********************************************************************
 * DAD tail ENDVXV (INTERPRETER.agc pp.1131-1133) -- DAS MPAC, then
 * OVERFLOW if A is non-zero.
 *
 * ARGUMENTS:
 *   m       agc_w2_t     I  MPAC pair
 *   x       agc_w2_t     I  operand pair
 *   ovfind  agc_ovf_t *  O  OVFIND, set to +-1 on overflow
 *
 * RETURNS: agc_w2_t -- MPAC after the add (wrapped on overflow)
 **********************************************************************/
agc_w2_t agc_w_dad(agc_w2_t m, agc_w2_t x, agc_ovf_t *ovfind)
{
    int      o = 0;                   /* overflow DAS leaves in A */
    agc_w2_t r = agc_w_das(m, x, &o); /* MPAC after DAS */

    if (o != 0)
    {
        r = agc_w_overflow(r, o, ovfind);
    }

    return (r);
}

/**********************************************************************
 * DMPSUB (INTERPRETER.agc p.1136) -- MPAC x C(ADDRWD) as a TP. The
 * minor x minor partial product keeps only its major half; everything
 * else is exact.
 *
 * ARGUMENTS:
 *   mpac  agc_w2_t  I  MPAC, MPAC+1 (signs may disagree)
 *   x     agc_w2_t  I  the operand at ADDRWD
 *
 * RETURNS: agc_w3_t -- MPAC, MPAC+1, MPAC+2; scales add
 **********************************************************************/
agc_w3_t agc_w_dmpsub(agc_w2_t mpac, agc_w2_t x)
{
    agc_w3_t m;     /* TP product */
    agc_w2_t acc;   /* DAS accumulator pair */
    int      o = 0; /* net DAS overflow, the carry into MPAC */

    if (!AGC_ASSERT(agc_w_ok(mpac.hi) && agc_w_ok(mpac.lo)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(agc_w_ok(x.hi) && agc_w_ok(x.lo)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    acc.hi = 0;                                           /* MPAC+1 := 0    */
    acc.lo = agc_w_mp(mpac.lo, x.lo).hi;                  /* MPAC+2         */
    acc    = agc_w_das(acc, agc_w_mp(x.lo, mpac.hi), &o); /* no overflow    */
    acc    = agc_w_das(acc, agc_w_mp(mpac.lo, x.hi), &o); /* A = net ovf    */
    m.m1   = acc.hi;
    m.m2   = acc.lo;
    acc.hi = o;                                           /* XCH MPAC       */
    acc.lo = m.m1;
    acc    = agc_w_das(acc, agc_w_mp(mpac.hi, x.hi), &o); /* no overflow    */
    m.m0   = acc.hi;
    m.m1   = acc.lo;

    return (m);
}

/**********************************************************************
 * ROUNDSUB / VROUND (INTERPRETER.agc p.1137) -- round MPAC+2 into MPAC,
 * MPAC+1 (half away from zero on the sign of MPAC+2) and zero MPAC+2.
 *
 * ARGUMENTS:
 *   m       agc_w3_t     I  TP MPAC
 *   ovfind  agc_ovf_t *  O  OVFIND, set to +-1 if the carry overflows
 *
 * RETURNS: agc_w3_t -- the rounded MPAC with m2 = 0
 **********************************************************************/
agc_w3_t agc_w_roundsub(agc_w3_t m, agc_ovf_t *ovfind)
{
    int o = 0; /* rounding carry, then TS overflow */

    if (!AGC_ASSERT(agc_w_ok(m.m2)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    o    = agc_w_ovf(2 * m.m2); /* DOUBLE; TS L   */
    m.m2 = 0;

    if (o == 0)
    {
        return (m);
    }

    m.m1 = agc_w_ts(m.m1 + o, &o);

    if (o != 0)
    {
        m.m0 = agc_w_ts(m.m0 + o, &o);

        if ((o != 0) && AGC_ASSERT(ovfind != 0))
        {
            *ovfind = (agc_ovf_t) o; /* SETOVF2        */
        }
    }

    return (m);
}

/**********************************************************************
 * DSQSUB (INTERPRETER.agc p.1183) -- square of the DP MPAC as a TP
 * (minor^2 keeps its major half, the cross term is DDOUBLed).
 *
 * ARGUMENTS:
 *   p  agc_w2_t  I  MPAC, MPAC+1
 *
 * RETURNS: agc_w3_t -- the square, at twice the scale of p
 **********************************************************************/
agc_w3_t agc_w_dsqsub(agc_w2_t p)
{
    agc_w3_t m;     /* TP square */
    agc_w2_t acc;   /* DAS accumulator pair */
    agc_w2_t cross; /* 2 hi lo cross term */
    int      o = 0; /* DAS overflow, the carry into MPAC */

    if (!AGC_ASSERT(agc_w_ok(p.hi) && agc_w_ok(p.lo)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    acc.hi   = 0;
    acc.lo   = agc_w_mp(p.lo, p.lo).hi;                  /* MPAC+2         */
    cross    = agc_w_mp(p.lo, p.hi);
    cross    = agc_w_das(cross, cross, &o);              /* DDOUBL         */
    cross.hi += o * AGC_W_ONE;                           /* A keeps ovf    */
    acc      = agc_w_das(acc, cross, &o);
    m.m1     = acc.hi;
    m.m2     = acc.lo;
    acc.hi   = o;
    acc.lo   = m.m1;
    acc      = agc_w_das(acc, agc_w_mp(p.hi, p.hi), &o); /* SQUARE; DAS    */
    m.m0     = acc.hi;
    m.m1     = acc.lo;

    if (!AGC_ASSERT(o == 0))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    return (m);
}

/**********************************************************************
 * DOTSUB middle (INTERPRETER.agc p.1138) -- add a second TP partial
 * product into BUF: DAS on the minor pair, AD/AD/TS on the major word;
 * TS OVFIND on overflow (major word left overflow-corrected).
 *
 * ARGUMENTS:
 *   buf     agc_w3_t     I  BUF, BUF+1, BUF+2 (first partial product)
 *   t       agc_w3_t     I  the next partial product (MPAC)
 *   ovfind  agc_ovf_t *  O  OVFIND, set to +-1 on overflow
 *
 * RETURNS: agc_w3_t -- the accumulated BUF
 **********************************************************************/
agc_w3_t agc_w_dot_acc(agc_w3_t buf, agc_w3_t t, agc_ovf_t *ovfind)
{
    agc_w2_t lo2 = {buf.m1, buf.m2};
    agc_w2_t add = {t.m1, t.m2};
    int      o   = 0; /* minor-pair carry, then TS overflow */

    lo2    = agc_w_das(lo2, add, &o);
    buf.m1 = lo2.hi;
    buf.m2 = lo2.lo;
    buf.m0 = agc_w_ts(o + t.m0 + buf.m0, &o);

    if ((o != 0) && AGC_ASSERT(ovfind != 0))
    {
        *ovfind = (agc_ovf_t) o;
    }

    return (buf);
}

/**********************************************************************
 * ENDDOT (INTERPRETER.agc p.1138) -- final accumulation of DOTSUB /
 * VSQSUB; an overflow goes through OVERFLOW.
 *
 * ARGUMENTS:
 *   buf     agc_w3_t     I  BUF: the sum of the first two products
 *   t       agc_w3_t     I  MPAC: the third product
 *   ovfind  agc_ovf_t *  O  OVFIND, set by OVERFLOW
 *
 * RETURNS: agc_w3_t -- the TP sum in MPAC
 **********************************************************************/
static agc_w3_t end_dot(agc_w3_t buf, agc_w3_t t, agc_ovf_t *ovfind)
{
    agc_w2_t lo2 = {t.m1, t.m2};
    agc_w2_t add = {buf.m1, buf.m2};
    agc_w2_t top;     /* MPAC, MPAC+1 of the sum */
    int      o   = 0; /* minor-pair carry, then TS overflow */

    lo2  = agc_w_das(lo2, add, &o);
    t.m1 = lo2.hi;
    t.m2 = lo2.lo;
    t.m0 = agc_w_ts(o + t.m0 + buf.m0, &o);

    if (o != 0)
    {
        top  = agc_w_overflow(agc_w3_top(t), o, ovfind);
        t.m0 = top.hi;
        t.m1 = top.lo;
    }

    return (t);
}

/**********************************************************************
 * DOTSUB (INTERPRETER.agc p.1138) -- u . v as a TP, u in MPAC and v at
 * ADDRWD, accumulated x, y, z.
 *
 * ARGUMENTS:
 *   u       agc_vec_t    I  MPAC vector
 *   v       agc_vec_t    I  operand vector; scales add
 *   ovfind  agc_ovf_t *  O  OVFIND, set to +-1 on overflow
 *
 * RETURNS: agc_w3_t -- u . v (TP MPAC)
 **********************************************************************/
agc_w3_t agc_w_dotsub(agc_vec_t u, agc_vec_t v, agc_ovf_t *ovfind)
{
    agc_w3_t buf; /* BUF: x then x + y partial products */
    agc_w3_t t;   /* MPAC: the next partial product */

    buf = agc_w_dmpsub(agc_w_split(u.c[0]), agc_w_split(v.c[0]));
    t   = agc_w_dmpsub(agc_w_split(u.c[1]), agc_w_split(v.c[1]));
    buf = agc_w_dot_acc(buf, t, ovfind);
    t   = agc_w_dmpsub(agc_w_split(u.c[2]), agc_w_split(v.c[2]));
    t   = end_dot(buf, t, ovfind);

    return (t);
}

/**********************************************************************
 * VSQSUB (INTERPRETER.agc p.1183) -- v . v with DSQSUB per component.
 *
 * ARGUMENTS:
 *   v       agc_vec_t    I  MPAC vector
 *   ovfind  agc_ovf_t *  O  OVFIND, set to +-1 on overflow
 *
 * RETURNS: agc_w3_t -- |v|^2 (TP MPAC), at twice the scale of v
 **********************************************************************/
agc_w3_t agc_w_vsqsub(agc_vec_t v, agc_ovf_t *ovfind)
{
    agc_w3_t buf; /* BUF: x then x + y squares */
    agc_w3_t t;   /* MPAC: the next square */

    buf = agc_w_dsqsub(agc_w_split(v.c[0]));
    t   = agc_w_dsqsub(agc_w_split(v.c[1]));
    buf = agc_w_dot_acc(buf, t, ovfind);
    t   = agc_w_dsqsub(agc_w_split(v.c[2]));
    t   = end_dot(buf, t, ovfind);

    return (t);
}

/**********************************************************************
 * One TP shift left (TSSL p.1154, SQRTNORM p.1186, TSLCLOOP p.1157):
 * DCA MPAC+1; DAS MPAC+1; AD MPAC; AD MPAC; TS MPAC.
 *
 * ARGUMENTS:
 *   m    agc_w3_t  I  TP MPAC
 *   ovf  int *     O  overflow of the final TS MPAC: +1, -1 or 0
 *
 * RETURNS: agc_w3_t -- 2 m, major word overflow-corrected
 **********************************************************************/
agc_w3_t agc_w_tp_shl1(agc_w3_t m, int *ovf)
{
    agc_w2_t lo2   = {m.m1, m.m2};
    int      carry = 0; /* minor-pair carry into the major word */

    if (!AGC_ASSERT(agc_w_ok(m.m0)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    lo2  = agc_w_das(lo2, lo2, &carry);
    m.m1 = lo2.hi;
    m.m2 = lo2.lo;
    m.m0 = agc_w_ts(carry + m.m0 + m.m0, ovf);

    return (m);
}

/**********************************************************************
 * POLY / POWRSERS (INTERPRETER.agc pp.1139-1140) -- Horner evaluation
 * A(N) x^N + ... + A(0): MPAC = A(N), then N times DMPSUB by x (VBUF)
 * and DAS the next coefficient. POLYCOM keeps only MPAC, MPAC+1 of the
 * argument; the DAS ignores overflow ("user's responsibility").
 *
 * ARGUMENTS:
 *   x       agc_w2_t          I  argument pair
 *   coef    const agc_dp_t *  I  A(0) .. A(degree), each at the result
 *                                scale
 *   degree  int               I  N, 1..15 (POLYCNT = N-1)
 *
 * RETURNS: agc_w3_t -- the TP MPAC left by the last DMPSUB and DAS
 **********************************************************************/
agc_w3_t agc_w_poly(agc_w2_t x, const agc_dp_t *coef, int degree)
{
    agc_w3_t m = {0, 0, 0};
    agc_w2_t top;   /* MPAC, MPAC+1 (Horner accumulator) */
    int      o = 0; /* DAS overflow, ignored as in the AGC */
    int      k;     /* coefficient index, degree-1 .. 0 */

    top = agc_w_split(coef[degree]);

    for (k = degree - 1; k >= 0; k--)
    { /* bound: degree  */
        m    = agc_w_dmpsub(top, x);
        top  = agc_w_das(agc_w3_top(m), agc_w_split(coef[k]), &o);
        m.m0 = top.hi;
        m.m1 = top.lo;
    }

    return (m);
}
