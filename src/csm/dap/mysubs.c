/**********************************************************************
 * FILE NAME: mysubs.c
 *
 * PURPOSE:
 *   MYSUBS (SMALLMP, DPADD) and the word-level helpers that stand in for
 *   the basic-language idioms of the CSM autopilots: AD/ADS with
 *   overflow correction, MSU on counter words, MP upper and lower words,
 *   DV, DDOUBL, DAS on stored word pairs, the CDU counters, the T5
 *   set-up and the SPS engine-on bit. SPSIN1/2, SPCOS1/2 are EQUALS of
 *   SPSIN/SPCOS (agc_sp_sin/agc_sp_cos).
 *
 * AGC SOURCE:
 *   Comanche055 MYSUBS.agc pp. 999-1000; the basic instructions as
 *   used throughout RCS-CSM_DIGITAL_AUTOPILOT.agc, AUTOMATIC_MANEUVERS.agc
 *   and the TVC*.agc files
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   CDUX..CDUZ    uint16_t  I    CDU counts     gimbal angle counters
 *   T5LOC, TIME5  task, word  O  10 ms          T5 interrupt set-up
 *   DSALMOUT      uint16_t  I    bits           channel 11 (bit 13)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_sp_mul            exact SP x SP product (agc/core/word.h)
 *   agc_dp_words_das      DAS on stored words (agc/core/interp_words.h)
 *   agc_rupt_set_t5_task, T5LOC and TIME5
 *   agc_rupt_set_time
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none. csm_dap_set_t5 / _set_time5 return AGC_STATUS_BAD_ARG for a
 *   word that is not a 15-bit TIME5 value.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Every agc_sp_t handed to these helpers is a word value (+-16383)
 *   produced by the lane's own overflow-corrected arithmetic or read
 *   from a 15-bit erasable, so the helpers have no failure path.
 *   Ones' complement -0 is not represented (TRANSLATION_GUIDE section 1).
 *
 * NOTES:
 *   An overflowing AD/ADS/DAS keeps the overflow-corrected value (the
 *   sum -+ 1.0, the sign of the true sum kept); MSU differences two's
 *   complement counter words and returns a ones' complement result; MP
 *   keeps the upper (A) and lower (L) words of the exact 28-bit product,
 *   signs agreeing as the sign-magnitude multiplier leaves them.
 **********************************************************************/
#include "internal.h"

#include "agc/core/interp_words.h"

#define W_ONE    16384      /* 2^14: 1.0 in SP units                     */
#define W_DP_ONE (1L << 28) /* 1.0 in DP units                          */

/**********************************************************************
 * CA of a 15-bit erasable word -- ones' complement word to value.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   w  uint16_t  I  the word (bit 15 the sign), any scale
 *
 * RETURNS: agc_sp_t -- its value (both zeros read as 0), same scale
 **********************************************************************/
agc_sp_t csm_dap_word_sp(uint16_t w)
{
    uint16_t v = (uint16_t) (w & CSM_DAP_NEG0); /* 15 bits               */

    if ((v & CSM_DAP_BIT15) != 0u)
    {
        return ((agc_sp_t) (-(int) ((~(unsigned int) v) & 037777u)));
    }

    return ((agc_sp_t) v);
}

/**********************************************************************
 * TS of a value into a 15-bit erasable word -- value to ones'
 * complement word.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I  word value, any scale
 *
 * RETURNS: uint16_t -- the ones' complement word (0 -> +0)
 **********************************************************************/
uint16_t csm_dap_sp_word(agc_sp_t v)
{
    if (v < 0)
    {
        return ((uint16_t) (077777 + (int) v));
    }

    return ((uint16_t) v);
}

/**********************************************************************
 * MSU (modular subtract) -- the two's complement difference of two
 * 15-bit counter words, converted to ones' complement by the hardware.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   a  uint16_t  I  minuend counter word (CDU counts, 2^15 = 360 deg)
 *   b  uint16_t  I  subtrahend counter word, same scale
 *
 * RETURNS: agc_sp_t -- a - b as a word value, 180 deg scale
 **********************************************************************/
agc_sp_t csm_dap_msu(uint16_t a, uint16_t b)
{
    unsigned int d = ((unsigned int) a - (unsigned int) b) & 077777u; /* two's complement */

    if ((d & 040000u) != 0u)
    {
        d -= 1u; /* two's -> ones' complement                         */
    }

    return (csm_dap_word_sp((uint16_t) d));
}

/**********************************************************************
 * AD / ADS -- single precision add; on overflow the stored word is the
 * overflow-corrected sum (-+ 1.0).
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  augend, any scale
 *   b  agc_sp_t  I  addend, same scale
 *
 * RETURNS: agc_sp_t -- the stored sum, same scale
 **********************************************************************/
agc_sp_t csm_dap_ads(agc_sp_t a, agc_sp_t b)
{
    int s = (int) a + (int) b; /* 16-bit accumulator, 2^-14 units      */

    if (s > AGC_SP_MAX)
    {
        s -= W_ONE;
    }
    else if (s < AGC_SP_MIN)
    {
        s += W_ONE;
    }
    else
    {
        /* no overflow */
    }

    return ((agc_sp_t) s);
}

/**********************************************************************
 * AD; TS X; TCF +3; INDEX A; CS LIMITS -- single precision add with an
 * overflow limited to +-POSMAX.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  augend, any scale
 *   b  agc_sp_t  I  addend, same scale
 *
 * RETURNS: agc_sp_t -- the limited sum, same scale
 **********************************************************************/
agc_sp_t csm_dap_ad_sat(agc_sp_t a, agc_sp_t b)
{
    int s = (int) a + (int) b; /* 16-bit accumulator, 2^-14 units      */

    if (s > AGC_SP_MAX)
    {
        s = AGC_SP_MAX;
    }
    else if (s < AGC_SP_MIN)
    {
        s = AGC_SP_MIN;
    }
    else
    {
        /* no overflow */
    }

    return ((agc_sp_t) s);
}

/**********************************************************************
 * DDOUBL -- A,L doubled, the upper word overflow-corrected (the value
 * -+ 1.0 on overflow), for a sign-agreeing A,L such as an MP product.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  A,L as a value, any scale
 *
 * RETURNS: agc_dp_t -- 2 A,L as stored, same scale
 **********************************************************************/
agc_dp_t csm_dap_ddoubl(agc_dp_t a)
{
    int64_t s = 2 * (int64_t) a; /* DAS A, 2^-28 units                 */

    if (s > AGC_DP_MAX)
    {
        s -= W_DP_ONE;
    }
    else if (s < AGC_DP_MIN)
    {
        s += W_DP_ONE;
    }
    else
    {
        /* no overflow */
    }

    return ((agc_dp_t) s);
}

/**********************************************************************
 * MP -- the upper word (A) of the product of two SP words.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  multiplicand, scale 2^m
 *   b  agc_sp_t  I  multiplier, scale 2^n
 *
 * RETURNS: agc_sp_t -- A, scale 2^(m+n)
 **********************************************************************/
agc_sp_t csm_dap_mp_hi(agc_sp_t a, agc_sp_t b)
{
    return (agc_dp_high(agc_sp_mul(a, b)));
}

/**********************************************************************
 * MP -- the lower word (L) of the product of two SP words.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  multiplicand, scale 2^m
 *   b  agc_sp_t  I  multiplier, scale 2^n
 *
 * RETURNS: agc_sp_t -- L, scale 2^(m+n-14)
 **********************************************************************/
agc_sp_t csm_dap_mp_lo(agc_sp_t a, agc_sp_t b)
{
    return (agc_dp_low(agc_sp_mul(a, b)));
}

/**********************************************************************
 * DV -- DP dividend (A,L) by an SP divisor, SP quotient truncated
 * toward zero (yaAGC: the dividend's words are combined into one value
 * first). DEVIATION: the AGC result is undefined when |dividend| >=
 * |divisor|; here it is limited to +-POSMAX with the sign of the
 * quotient.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   num  agc_dp_t  I  dividend A,L, scale 2^m
 *   den  agc_sp_t  I  divisor, scale 2^n
 *
 * RETURNS: agc_sp_t -- quotient, scale 2^(m-n)
 **********************************************************************/
agc_sp_t csm_dap_dv(agc_dp_t num, agc_sp_t den)
{
    bool    neg = ((num < 0) != (den < 0)) && (num != 0);     /* quotient sign */
    int64_t an  = (num < 0) ? -(int64_t) num : (int64_t) num; /* ?: |A,L|, 2^-28 units */
    int64_t ad  = (den < 0) ? -(int64_t) den : (int64_t) den; /* ?: |divisor|, 2^-14 units */

    if ((ad == 0) || (an >= ad * W_ONE))
    {
        return (neg ? AGC_SP_MIN : AGC_SP_MAX); /* ?: overflow, limited */
    }

    int64_t q = an / ad; /* |quotient|, 2^-14 units                    */

    return ((agc_sp_t) (neg ? -q : q)); /* ?: signed quotient          */
}

/**********************************************************************
 * DPADD (MYSUBS.agc pp. 999-1000) -- add the DP angle A,L (180 deg
 * scale) to KMPAC; an overflow is folded back by 360 degrees (DPADD+,
 * DPADD-, DPADD2+). A sum of exactly +-180 deg leaves +-0, +-0 after the
 * DAS and the AGC stores POSMAX, POSMAX. DEVIATION: the ones' complement
 * +-0 bookkeeping of the other "upper = 0" branches is value-equivalent
 * and not modelled bit for bit.
 *
 * ARGUMENTS:
 *   kmpac  agc_dp_t  I  KMPAC, KMPAC +1, 180 deg
 *   al     agc_dp_t  I  A,L, 180 deg
 *
 * RETURNS: agc_dp_t -- the new KMPAC, 180 deg
 **********************************************************************/
agc_dp_t csm_dap_dpadd(agc_dp_t kmpac, agc_dp_t al)
{
    int64_t s = (int64_t) kmpac + (int64_t) al; /* DAS KMPAC, 2^-28 units */

    if ((s == W_DP_ONE) || (s == -W_DP_ONE))
    {
        s = AGC_DP_MAX; /* +-0, +-0: COM; AD POSMAX; CA POSMAX            */
    }
    else if (s > AGC_DP_MAX)
    {
        s -= 2 * W_DP_ONE; /* DPADD+: subtract 360 deg                   */
    }
    else if (s < AGC_DP_MIN)
    {
        s += 2 * W_DP_ONE; /* DPADD-: add 360 deg                        */
    }
    else
    {
        /* no overflow */
    }

    return ((agc_dp_t) s);
}

/**********************************************************************
 * DCS -- a stored word pair with each word complemented (the signs stay
 * as they were).
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   p  agc_dp_words_t  I  the pair, any scale
 *
 * RETURNS: agc_dp_words_t -- -p, word by word, same scale
 **********************************************************************/
agc_dp_words_t csm_dap_words_neg(agc_dp_words_t p)
{
    agc_dp_words_t n = {(agc_sp_t) -p.hi, (agc_sp_t) -p.lo}; /* complemented pair */

    return (n);
}

/**********************************************************************
 * MP -- the product of two SP words in A,L (signs agree, as the
 * hardware's sign-magnitude multiply leaves them).
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  multiplicand, scale 2^m
 *   b  agc_sp_t  I  multiplier, scale 2^n
 *
 * RETURNS: agc_dp_words_t -- A, L, scale 2^(m+n)
 **********************************************************************/
agc_dp_words_t csm_dap_words_mp(agc_sp_t a, agc_sp_t b)
{
    return (agc_dp_to_words(agc_sp_mul(a, b))); /* A (upper) and L (lower) */
}

/**********************************************************************
 * DAS K -- K,K+1 += A,L on the stored words (agc_dp_words_das: the low
 * words add first and carry into the high sum; each word keeps its
 * overflow-corrected value, the signs are not made to agree), for the
 * sites that keep the overflow the AGC leaves in A (a following DXCH
 * stores A,L = overflow, +0).
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   k   agc_dp_words_t *  IO  K, K+1, any scale
 *   al  agc_dp_words_t    I   A, L, same scale
 *
 * RETURNS: agc_sp_t -- the overflow of the high sum: +1, -1 or 0
 **********************************************************************/
agc_sp_t csm_dap_words_das_carry(agc_dp_words_t *k, agc_dp_words_t al)
{
    int ovf = 0; /* overflow indicator left in A                          */

    *k = agc_dp_words_das(*k, al, &ovf);

    return ((agc_sp_t) ovf);
}

/**********************************************************************
 * DAS K -- as csm_dap_words_das_carry where the AGC overwrites A before
 * reading it: the overflow indicator is discarded.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   k   agc_dp_words_t *  IO  K, K+1, any scale
 *   al  agc_dp_words_t    I   A, L, same scale
 **********************************************************************/
void csm_dap_words_das(agc_dp_words_t *k, agc_dp_words_t al)
{
    int ovf = 0; /* overflow indicator, not used by the AGC here          */

    *k = agc_dp_words_das(*k, al, &ovf);
}

/**********************************************************************
 * DDOUBL -- A,L added to itself (DAS A) on the stored words.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   p  agc_dp_words_t  I  A, L, any scale
 *
 * RETURNS: agc_dp_words_t -- 2 A,L as the AGC holds it, same scale
 **********************************************************************/
agc_dp_words_t csm_dap_words_ddoubl(agc_dp_words_t p)
{
    agc_dp_words_t d = p; /* doubled pair                               */

    csm_dap_words_das(&d, p);

    return (d);
}

/**********************************************************************
 * SMALLMP (MYSUBS.agc p. 999) -- one-and-a-half precision multiply on
 * the stored words: TS of the upper word of (lower word x n) into the
 * lower word of a zeroed pair, then DAS of (upper word x n). Also the
 * TVC filter's inline form of the same sequence.
 *
 * ARGUMENTS:
 *   in  agc_dp_words_t  I  KMPAC, KMPAC +1, scale 2^m
 *   n   agc_sp_t        I  multiplier (A), scale 2^n
 *
 * RETURNS: agc_dp_words_t -- KMPAC as left, scale 2^(m+n)
 **********************************************************************/
agc_dp_words_t csm_dap_words_smallmp(agc_dp_words_t in, agc_sp_t n)
{
    agc_dp_words_t r = {0, csm_dap_mp_hi(in.lo, n)}; /* AY: CAF ZERO; XCH KMPAC */

    csm_dap_words_das(&r, csm_dap_words_mp(in.hi, n)); /* AX; DAS KMPAC  */

    return (r);
}

/**********************************************************************
 * CA CDUX / CDUY / CDUZ -- a CDU counter as its raw word.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   axis  int  I  0 X, 1 Y, 2 Z
 *
 * RETURNS: uint16_t -- the counter, 2^15 counts = 360 deg
 **********************************************************************/
uint16_t csm_dap_cdu(int axis)
{
    if (axis == 1)
    {
        return (agc_erase_cduy());
    }

    if (axis == 2)
    {
        return (agc_erase_cduz());
    }

    return (agc_erase_cdux());
}

/**********************************************************************
 * EXTEND; DCA xxxLOC; DXCH T5LOC; CAF delay; TS TIME5 -- set the next
 * T5 interrupt's task and time.
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   task        agc_task_id_t  I  T5LOC
 *   time5_word  uint16_t       I  TIME5, 10 ms counts to overflow
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (always)
 **********************************************************************/
agc_status_t csm_dap_set_t5(agc_task_id_t task, uint16_t time5_word)
{
    agc_rupt_set_t5_task(task);
    agc_rupt_set_time(CSM_DAP_TIME5, time5_word);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CAF delay; TS TIME5 -- set the next T5 interrupt's time (T5LOC kept).
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS:
 *   time5_word  uint16_t  I  TIME5, 10 ms counts to overflow
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG for a word
 *          wider than 15 bits
 **********************************************************************/
agc_status_t csm_dap_set_time5(uint16_t time5_word)
{
    if (!AGC_ASSERT(time5_word <= CSM_DAP_NEG0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rupt_set_time(CSM_DAP_TIME5, time5_word);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CAF BIT13; EXTEND; RAND DSALMOUT -- the SPS engine-on output (channel
 * 11 bit 13).
 * AGC: none (the basic instruction as the lane's code uses it).
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when the engine-on bit is set
 **********************************************************************/
bool csm_dap_engine_on(void)
{
    uint16_t ch11 = agc_chan_read(AGC_CHAN_DSALMOUT); /* DSALMOUT        */

    return ((ch11 & (uint16_t) AGC_CH011_ENGINEON) != 0u);
}
