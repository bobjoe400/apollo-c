/*
 * mysubs.c -- MYSUBS (Comanche055 pp. 999-1000) and the word-level helpers
 * that stand in for the basic-language idioms of the CSM autopilots.
 *
 * MYSUBS: SMALLMP (one-and-a-half precision multiply) and DPADD (double
 * precision angle addition modulo 360 degrees). SPSIN1/2, SPCOS1/2 are
 * EQUALS of SPSIN/SPCOS (agc_sp_sin/agc_sp_cos).
 *
 * The helpers model the AGC's ones' complement registers on agc_sp_t /
 * agc_dp_t values: an overflowing AD/DAS keeps the overflow-corrected
 * value (the sum -+ 1.0), MSU differences two's complement counter words
 * and returns a ones' complement result, MP keeps the upper (A) and lower
 * (L) words of the exact 28-bit product.
 */
#include "internal.h"

#define W_ONE    16384      /* 2^14: 1.0 in SP units                     */
#define W_DP_ONE (1L << 28) /* 1.0 in DP units                          */

/* Ones' complement 15-bit word -> value. Both zeros read as 0.           */
agc_sp_t csm_dap_word_sp(uint16_t w)
{
    uint16_t v = (uint16_t) (w & CSM_DAP_NEG0);

    if (!AGC_ASSERT(v <= CSM_DAP_NEG0))
    {
        return (0); /* no valid result */
    }

    if ((v & CSM_DAP_BIT15) != 0u)
    {
        return ((agc_sp_t) (-(int) ((~(unsigned) v) & 037777u)));
    }

    return ((agc_sp_t) v);
}

/* Value -> ones' complement 15-bit word (0 -> +0).                       */
uint16_t csm_dap_sp_word(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(v <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    if (v < 0)
    {
        return ((uint16_t) (077777 + (int) v));
    }

    return ((uint16_t) v);
}

/* MSU (modular subtract): the two's complement difference of two 15-bit
 * counter words, converted to ones' complement by the hardware.          */
agc_sp_t csm_dap_msu(uint16_t a, uint16_t b)
{
    if (!AGC_ASSERT(a <= CSM_DAP_NEG0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(b <= CSM_DAP_NEG0))
    {
        return (0); /* no valid result */
    }

    unsigned d = ((unsigned) a - (unsigned) b) & 077777u;

    if ((d & 040000u) != 0u)
    {
        d -= 1u; /* two's -> ones' complement          */
    }

    return (csm_dap_word_sp((uint16_t) d));
}

/* AD / ADS: on overflow the stored word is the sum -+ 1.0.               */
agc_sp_t csm_dap_ads(agc_sp_t a, agc_sp_t b)
{
    int s = (int) a + (int) b;

    if (!AGC_ASSERT((a >= AGC_SP_MIN) && (b >= AGC_SP_MIN)))
    {
        return (0); /* no valid result */
    }

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

    if (!AGC_ASSERT((s >= AGC_SP_MIN) && (s <= AGC_SP_MAX)))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) s);
}

/* AD; TS X; TCF +3; INDEX A; CS LIMITS: overflow limits to +-POSMAX.     */
agc_sp_t csm_dap_ad_sat(agc_sp_t a, agc_sp_t b)
{
    int s = (int) a + (int) b;

    if (!AGC_ASSERT((a >= AGC_SP_MIN) && (b >= AGC_SP_MIN)))
    {
        return (0); /* no valid result */
    }

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

    if (!AGC_ASSERT((s >= AGC_SP_MIN) && (s <= AGC_SP_MAX)))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) s);
}

/* DAS: double precision add, overflow-corrected like the AGC.            */
agc_dp_t csm_dap_das(agc_dp_t a, agc_dp_t b)
{
    int64_t s = (int64_t) a + (int64_t) b;

    if (!AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT((b >= AGC_DP_MIN) && (b <= AGC_DP_MAX)))
    {
        return (0); /* no valid result */
    }

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

/* DDOUBL: A,L doubled (overflow-corrected).                             */
agc_dp_t csm_dap_ddoubl(agc_dp_t a)
{
    if (!AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX)))
    {
        return (0); /* no valid result */
    }

    return (csm_dap_das(a, a));
}

agc_sp_t csm_dap_mp_hi(agc_sp_t a, agc_sp_t b)
{
    if (!AGC_ASSERT((a >= AGC_SP_MIN) && (a <= AGC_SP_MAX)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT((b >= AGC_SP_MIN) && (b <= AGC_SP_MAX)))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_high(agc_sp_mul(a, b)));
}

agc_sp_t csm_dap_mp_lo(agc_sp_t a, agc_sp_t b)
{
    if (!AGC_ASSERT((a >= AGC_SP_MIN) && (a <= AGC_SP_MAX)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT((b >= AGC_SP_MIN) && (b <= AGC_SP_MAX)))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_low(agc_sp_mul(a, b)));
}

/* DV: DP dividend (A,L) by an SP divisor, SP quotient truncated toward
 * zero. DEVIATION: the AGC result is undefined when |dividend| >= |divisor|;
 * here it is limited to +-POSMAX with the sign of the quotient.           */
agc_sp_t csm_dap_dv(agc_dp_t num, agc_sp_t den)
{
    if (!AGC_ASSERT((num >= AGC_DP_MIN) && (num <= AGC_DP_MAX)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT((den >= AGC_SP_MIN) && (den <= AGC_SP_MAX)))
    {
        return (0); /* no valid result */
    }

    bool    neg = ((num < 0) != (den < 0)) && (num != 0);
    int64_t an  = (num < 0) ? -(int64_t) num : (int64_t) num;
    int64_t ad  = (den < 0) ? -(int64_t) den : (int64_t) den;

    if ((ad == 0) || (an >= ad * W_ONE))
    {
        return (neg ? AGC_SP_MIN : AGC_SP_MAX);
    }

    int64_t q = an / ad;

    return ((agc_sp_t) (neg ? -q : q));
}

/* SMALLMP (MYSUBS p. 999): KMPAC = A (KMPAC, KMPAC+1), the low product
 * truncated to its upper word ("ONE AND ONE HALF PRECISION").            */
agc_dp_t csm_dap_smallmp(agc_dp_t kmpac, agc_sp_t a)
{
    if (!AGC_ASSERT((kmpac >= AGC_DP_MIN) && (kmpac <= AGC_DP_MAX)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT((a >= AGC_SP_MIN) && (a <= AGC_SP_MAX)))
    {
        return (0); /* no valid result */
    }

    agc_sp_t ay = csm_dap_mp_hi(a, agc_dp_low(kmpac)); /* AY (KMPAC +1) */
    agc_dp_t ax = agc_sp_mul(a, agc_dp_high(kmpac));   /* AX            */

    return (csm_dap_das(ax, (agc_dp_t) ay));             /* AX + AY       */
}

/* DPADD (MYSUBS pp. 999-1000): add the DP angle A,L (scaled 180 deg) to
 * KMPAC; an overflow is folded back by 360 degrees (DPADD+ / DPADD-).
 * DEVIATION: the ones' complement +-0 bookkeeping of the "upper = 0"
 * branches (DPADD2+) is value-equivalent and not modelled bit for bit.   */
agc_dp_t csm_dap_dpadd(agc_dp_t kmpac, agc_dp_t al)
{
    if (!AGC_ASSERT((kmpac >= AGC_DP_MIN) && (kmpac <= AGC_DP_MAX)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT((al >= AGC_DP_MIN) && (al <= AGC_DP_MAX)))
    {
        return (0); /* no valid result */
    }

    int64_t s = (int64_t) kmpac + (int64_t) al;

    if (s > AGC_DP_MAX)
    {
        s -= 2 * W_DP_ONE; /* + overflow: subtract 360 deg       */
    }
    else if (s < AGC_DP_MIN)
    {
        s += 2 * W_DP_ONE; /* - overflow: add 360 deg            */
    }
    else
    {
        /* no overflow */
    }

    return ((agc_dp_t) s);
}

/* Split a value into the sign-agreed word pair a DCA of it would load.  */
agc_csm_dp_pair_t csm_dap_pair(agc_dp_t v)
{
    agc_csm_dp_pair_t p = {0, 0}; /* returned pair             */

    if (AGC_ASSERT((v >= AGC_DP_MIN) && (v <= AGC_DP_MAX)))
    {
        p.hi = agc_dp_high(v);
        p.lo = agc_dp_low(v);
    }

    return (p);
}

/* The value of a stored pair, hi 2^14 + lo, in the pair's scale.        */
agc_dp_t csm_dap_pair_value(agc_csm_dp_pair_t p)
{
    agc_dp_t v = 0; /* value, upper word's scale                          */

    if (AGC_ASSERT((p.hi >= AGC_SP_MIN) && (p.lo >= AGC_SP_MIN)))
    {
        v = (agc_dp_t) p.hi * W_ONE + (agc_dp_t) p.lo;
    }

    return (v);
}

/* DCS: each word complemented (the signs stay as they were).            */
agc_csm_dp_pair_t csm_dap_pair_neg(agc_csm_dp_pair_t p)
{
    agc_csm_dp_pair_t n = {0, 0}; /* complemented pair                    */

    if (AGC_ASSERT((p.hi >= AGC_SP_MIN) && (p.lo >= AGC_SP_MIN)))
    {
        n.hi = (agc_sp_t) -p.hi;
        n.lo = (agc_sp_t) -p.lo;
    }

    return (n);
}

/* MP: the product of two SP words in A,L (signs agree, as the hardware's
 * sign-magnitude multiply leaves them).                                  */
agc_csm_dp_pair_t csm_dap_pair_mp(agc_sp_t a, agc_sp_t b)
{
    agc_csm_dp_pair_t p = {0, 0}; /* A (upper) and L (lower)             */

    if (AGC_ASSERT((a >= AGC_SP_MIN) && (b >= AGC_SP_MIN)))
    {
        p = csm_dap_pair(agc_sp_mul(a, b));
    }

    return (p);
}

/* DAS K: K,K+1 += A,L. The low words add first; an overflow there
 * carries +-1 into the high sum. Each word keeps its overflow-corrected
 * value and the signs are not made to agree. Returns the overflow of
 * the high sum (+1, -1 or 0), which the AGC leaves in A.                */
agc_sp_t csm_dap_pair_das(agc_csm_dp_pair_t *k, agc_csm_dp_pair_t al)
{
    agc_sp_t ovf = 0; /* overflow indicator left in A                     */

    if (AGC_ASSERT(k != NULL) && AGC_ASSERT(al.lo >= AGC_SP_MIN))
    {
        int lo    = (int) k->lo + (int) al.lo; /* low sum, |lo| < 2^15     */
        int carry = 0;                         /* carry into the high sum */

        if (lo > AGC_SP_MAX)
        {
            lo   -= W_ONE;
            carry = 1;
        }
        else if (lo < AGC_SP_MIN)
        {
            lo   += W_ONE;
            carry = -1;
        }
        else
        {
            /* no low-word overflow */
        }

        int hi = (int) k->hi + (int) al.hi + carry; /* high sum            */

        if (hi > AGC_SP_MAX)
        {
            hi -= W_ONE;
            ovf = 1;
        }
        else if (hi < AGC_SP_MIN)
        {
            hi += W_ONE;
            ovf = -1;
        }
        else
        {
            /* no high-word overflow */
        }

        k->hi = (agc_sp_t) hi;
        k->lo = (agc_sp_t) lo;
    }

    return (ovf);
}

/* DDOUBL: A,L added to itself (DAS A), overflow-corrected.              */
agc_csm_dp_pair_t csm_dap_pair_ddoubl(agc_csm_dp_pair_t p)
{
    agc_csm_dp_pair_t d = p; /* doubled pair                           */

    (void) csm_dap_pair_das(&d, p);

    return (d);
}

/* One-and-a-half precision multiply as the TVC filter codes it: TS of
 * the high word of (lower word x n) into the lower word of a zeroed pair,
 * then DAS of (upper word x n).                                          */
agc_csm_dp_pair_t csm_dap_pair_smallmp(agc_csm_dp_pair_t in, agc_sp_t n)
{
    agc_csm_dp_pair_t r = {0, 0}; /* (0, hi(in.lo n)) + in.hi n          */

    if (AGC_ASSERT((in.lo >= AGC_SP_MIN) && (n >= AGC_SP_MIN)))
    {
        r.lo = csm_dap_mp_hi(in.lo, n);
        (void) csm_dap_pair_das(&r, csm_dap_pair_mp(in.hi, n));
    }

    return (r);
}

/* CDUX (axis 0), CDUY (1), CDUZ (2): raw counter words.                  */
uint16_t csm_dap_cdu(int axis)
{
    if (!AGC_ASSERT((axis >= 0) && (axis < AGC_CSM_AXES)))
    {
        return (0); /* no valid result */
    }

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

/* EXTEND; DCA xxxLOC; DXCH T5LOC; CAF delay; TS TIME5.                   */
agc_status_t csm_dap_set_t5(agc_task_id_t task, uint16_t time5_word)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE) ||
        !AGC_ASSERT(time5_word <= CSM_DAP_NEG0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rupt_set_t5_task(task);
    agc_rupt_set_time(CSM_DAP_TIME5, time5_word);

    return (AGC_STATUS_OK);
}

agc_status_t csm_dap_set_time5(uint16_t time5_word)
{
    if (!AGC_ASSERT(time5_word <= CSM_DAP_NEG0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rupt_set_time(CSM_DAP_TIME5, time5_word);

    return (AGC_STATUS_OK);
}

/* CAF BIT13; EXTEND; RAND DSALMOUT: SPS engine-on output bit 11.13.      */
bool csm_dap_engine_on(void)
{
    uint16_t ch11 = agc_chan_read(AGC_CHAN_DSALMOUT);

    if (!AGC_ASSERT(ch11 <= CSM_DAP_NEG0))
    {
        return (false);
    }

    return ((ch11 & (uint16_t) AGC_CH011_ENGINEON) != 0u);
}
