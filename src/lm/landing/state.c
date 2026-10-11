/**********************************************************************
 * FILE NAME: state.c
 *
 * PURPOSE:
 *   guid-landing lane: erasable storage shared by the landing, abort,
 *   ascent, throttle and display files, plus the basic-language idioms
 *   (PHASCHNG, FASTCHNG, DV, MP, AD, DAS on stored words) they all use.
 *   The AGC keeps these quantities in EBANK 5/6/7 erasables (the
 *   "unswitched for lunar landing" overlays). They are lane-private:
 *   other lanes reach them only through the accessors below.
 *
 * AGC SOURCE:
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc (landing overlays);
 *   LUNAR_LANDING_GUIDANCE_EQUATIONS.agc p. 826 (FASTCHNG)
 *
 * EXTERNAL VARIABLES:
 *   lm_pad, lm_ldg, lm_thr, lm_asc (file static, through accessors)
 *
 * EXTERNAL REFERENCES:
 *   agc_phase_change, agc_time_now_cs, agc_chan_read, agc_dp_high,
 *   agc_dp_low
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; out-of-range arguments are reported by AGC_ASSERT
 **********************************************************************/
#include <string.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/lm/landing.h"

static agc_lm_pad_t lm_pad; /* W-matrix overlay pad loads, EBANK 5     */
static agc_lm_ldg_t lm_ldg; /* landing guidance erasables, EBANK 7     */
static agc_lm_thr_t lm_thr; /* throttle erasables, EBANK 7             */
static agc_lm_asc_t lm_asc; /* ascent guidance erasables, EBANK 7      */

#define LM_FASTCHNG_CODE 05023u /* group 3 phase of LUNLAND (OCT 05023) */
#define LM_WORD          16384  /* 2^14: weight of the major word       */

/**********************************************************************
 * AGC: none (accessor) -- the W-matrix overlay pad loads (EBANK 5)
 *
 * RETURNS: agc_lm_pad_t * -- the lane's pad-load block, never NULL
 **********************************************************************/
agc_lm_pad_t *agc_lm_pad(void)
{
    if (!AGC_ASSERT(lm_pad.leadtime <= 0)) /* LEADTIME is a lead, cs  */
    {
        return (&lm_pad);
    }

    if (!AGC_ASSERT(lm_pad.taurod >= 0)) /* TAUROD time constant     */
    {
        return (&lm_pad);
    }

    return (&lm_pad);
}

/**********************************************************************
 * AGC: none (accessor) -- the landing guidance erasables (EBANK 7)
 *
 * RETURNS: agc_lm_ldg_t * -- the lane's landing block, never NULL
 **********************************************************************/
agc_lm_ldg_t *agc_lm_ldg(void)
{
    if (!AGC_ASSERT(lm_ldg.wchphase >= AGC_LM_PHASE_IGNALG && lm_ldg.wchphase <= AGC_LM_PHASE_VERTICAL)) /* WCHPHASE */
    {
        return (&lm_ldg);
    }

    if (!AGC_ASSERT(lm_ldg.flpass0 >= 0)) /* FLPASS0 counts passes      */
    {
        return (&lm_ldg);
    }

    return (&lm_ldg);
}

/**********************************************************************
 * AGC: none (accessor) -- the throttle erasables (EBANK 7)
 *
 * RETURNS: agc_lm_thr_t * -- the lane's throttle block, never NULL
 **********************************************************************/
agc_lm_thr_t *agc_lm_thr(void)
{
    if (!AGC_ASSERT(lm_thr.fcold >= AGC_SP_MIN)) /* FCOLD is a word      */
    {
        return (&lm_thr);
    }

    if (!AGC_ASSERT(lm_thr.fweight.hi >= AGC_SP_MIN && lm_thr.fweight.lo >= AGC_SP_MIN)) /* FWEIGHT words */
    {
        return (&lm_thr);
    }

    return (&lm_thr);
}

/**********************************************************************
 * AGC: none (accessor) -- the ascent and abort guidance erasables
 *
 * RETURNS: agc_lm_asc_t * -- the lane's ascent block, never NULL
 **********************************************************************/
agc_lm_asc_t *agc_lm_asc(void)
{
    if (!AGC_ASSERT(lm_asc.tbup >= 0)) /* TBUP, burn-up time, never < 0 */
    {
        return (&lm_asc);
    }

    if (!AGC_ASSERT(lm_asc.ve >= 0)) /* VE = -APSVEX/-DPSVEX, positive */
    {
        return (&lm_asc);
    }

    return (&lm_asc);
}

/**********************************************************************
 * AGC: none -- fresh start of the lane's erasables (the AGC leaves them
 * as they were; this is for tests and the sim harness)
 **********************************************************************/
void agc_lm_landing_reset(void)
{
    (void) memset(&lm_pad, 0, sizeof (lm_pad)); /* returns &lm_pad   */
    (void) memset(&lm_ldg, 0, sizeof (lm_ldg));
    (void) memset(&lm_thr, 0, sizeof (lm_thr));
    (void) memset(&lm_asc, 0, sizeof (lm_asc));
    lm_ldg.wchphase = AGC_LM_PHASE_IGNALG;

    if (!AGC_ASSERT(lm_ldg.ttf_8 == 0 && lm_thr.tthrot == 0)) /* cleared */
    {
        return;
    }

    if (!AGC_ASSERT(lm_asc.tbup == 0 && lm_pad.leadtime == 0))
    {
        return;
    }
}

/**********************************************************************
 * PHASCHNG (PHASE_TABLE_MAINTENANCE.agc) -- the low octal digit of the
 * code word is the restart group; the full code goes to the executive so
 * the restart tables see the phase the AGC would have stored
 *
 * ARGUMENTS:
 *   octal_code  uint16_t  I  PHASCHNG code word (OCT TPAGG), 15 bits
 **********************************************************************/
void agc_lm_phaschng(uint16_t octal_code)
{
    int group = (int) (octal_code & 7u); /* restart group 1..6         */

    if (!AGC_ASSERT(group >= 1 && group <= AGC_RESTART_GROUPS))
    {
        return;
    }

    if (!AGC_ASSERT(octal_code <= 077777u))
    {
        return;
    }

    agc_phase_change(group, octal_code);
}

/**********************************************************************
 * FASTCHNG (LUNAR_LANDING_GUIDANCE_EQUATIONS.agc p. 826) -- specialised
 * PHASCHNG that stores the caller's address as the group 3 restart
 * point. DEVIATION: C cannot restart in the middle of a function, so
 * group 3 is re-armed at the LUNLAND phase; a restart repeats the
 * guidance pass.
 **********************************************************************/
void agc_lm_fastchng(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM)) /* LM only */
    {
        return;
    }

    if (!AGC_ASSERT((LM_FASTCHNG_CODE & 7u) == 3u))
    {
        return; /* not a group 3 code */
    }

    agc_phase_change(3, LM_FASTCHNG_CODE);
}

/**********************************************************************
 * AGC: none (TIME1 counter) -- low 14 bits of the centisecond clock
 * (TIME1 wraps every 163.84 s into TIME2)
 *
 * RETURNS: agc_sp_t -- TIME1, cs, 0..POSMAX (unscaled)
 **********************************************************************/
agc_sp_t agc_lm_time1(void)
{
    uint32_t t  = agc_time_now_cs();        /* TIME2,TIME1, cs */
    agc_sp_t t1 = (agc_sp_t) (t & 0x3FFFu); /* TIME1, cs, unscaled */

    if (!AGC_ASSERT(t1 >= 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(t1 <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return (t1);
}

/**********************************************************************
 * AGC: none (DV instruction) -- divide the DP fraction (A,L) by the SP
 * fraction in K. With dividend v/2^28 and divisor d/2^14 the quotient
 * word is v/d, truncated toward zero as the hardware does. The AGC
 * result is meaningless when |dividend| >= |divisor|; asserted and
 * saturated here.
 *
 * ARGUMENTS:
 *   dividend  agc_dp_t  I  A,L, any DP scale 2^S
 *   divisor   agc_sp_t  I  K, scale 2^D, non-zero
 *
 * RETURNS: agc_sp_t -- quotient, scale 2^(S-D)
 **********************************************************************/
agc_sp_t agc_lm_dv(agc_dp_t dividend, agc_sp_t divisor)
{
    if (!AGC_ASSERT(divisor != 0))
    {
        return ((dividend >= 0) ? AGC_SP_MAX : AGC_SP_MIN); /* by sign */
    }

    int32_t q = dividend / (int32_t) divisor; /* quotient, 2^(S-D)   */

    if (!AGC_ASSERT(q <= AGC_SP_MAX && q >= AGC_SP_MIN))
    { /* |dividend| >= |divisor|: the hardware result is meaningless */
        q = (q > 0) ? AGC_SP_MAX : AGC_SP_MIN; /* saturate by sign   */
    }

    return ((agc_sp_t) q);
}

/**********************************************************************
 * AGC: none (MP instruction) -- product of two words, keeping the high
 * word (what most basic-language callers use)
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  multiplicand, scale 2^A
 *   b  agc_sp_t  I  multiplier, scale 2^B
 *
 * RETURNS: agc_sp_t -- high word of a*b, scale 2^(A+B), truncated
 **********************************************************************/
agc_sp_t agc_lm_mp_hi(agc_sp_t a, agc_sp_t b)
{
    int32_t p  = (int32_t) a * (int32_t) b; /* DP product, 2^-28 units */
    int32_t hi = p / LM_WORD;               /* truncate toward zero    */

    if (!AGC_ASSERT(hi <= AGC_SP_MAX && hi >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(hi * hi <= (int32_t) a * a))
    {
        return (0); /* not reached: |b| < 1 cannot enlarge |a| */
    }

    return ((agc_sp_t) hi);
}

/**********************************************************************
 * AGC: none (AD instruction, then TS/XCH/ADS of A) -- word sum as the
 * AGC stores it: an overflowing 16-bit A is overflow-corrected, i.e.
 * the sum wraps modulo 2^14 (16383 + 1 -> +0). The callers in this lane
 * never overflow on valid data; the assertion flags it if they do.
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  augend, any scale 2^S
 *   b  agc_sp_t  I  addend, same scale 2^S
 *
 * RETURNS: agc_sp_t -- a + b, scale 2^S, wrapped like the AGC
 **********************************************************************/
agc_sp_t agc_lm_ad(agc_sp_t a, agc_sp_t b)
{
    int32_t s = (int32_t) a + (int32_t) b; /* sum in the 16-bit A, 2^S */

    if (!AGC_ASSERT(s <= AGC_SP_MAX && s >= AGC_SP_MIN))
    { /* overflow-corrected store: drop the overflow bit           */
        s = (s > 0) ? s - LM_WORD : s + LM_WORD; /* by overflow sign */
    }

    if (!AGC_ASSERT(s >= AGC_SP_MIN && s <= AGC_SP_MAX)) /* one word */
    {
        return (0); /* not reached: one correction fits a word */
    }

    return ((agc_sp_t) s);
}

/**********************************************************************
 * AGC: none -- sign-agreeing split of a DP value (what MP, DMP and
 * TPAGREE store)
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I  DP value, any scale 2^S
 *
 * RETURNS: agc_lm_words_t -- the two words, scale 2^S
 **********************************************************************/
agc_lm_words_t agc_lm_words(agc_dp_t v)
{
    agc_lm_words_t w = {agc_dp_high(v), agc_dp_low(v)}; /* 2^S       */

    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        w.hi = 0;
        w.lo = 0;
    }
    else if (!AGC_ASSERT(agc_lm_words_value(w) == v))
    {
        w.lo = (agc_sp_t) (v - (int32_t) w.hi * LM_WORD); /* exact */
    }

    return (w);
}

/**********************************************************************
 * AGC: none -- value of a stored word pair, whatever the signs
 *
 * ARGUMENTS:
 *   w  agc_lm_words_t  I  major and minor word, scale 2^S
 *
 * RETURNS: agc_dp_t -- hi * 2^14 + lo, scale 2^S
 **********************************************************************/
agc_dp_t agc_lm_words_value(agc_lm_words_t w)
{
    if (!AGC_ASSERT(w.hi >= AGC_SP_MIN && w.lo >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(w.hi <= AGC_SP_MAX && w.lo <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return ((agc_dp_t) w.hi * LM_WORD + (agc_dp_t) w.lo);
}

/**********************************************************************
 * AGC: none (DAS instruction) -- add the low words, carry a low-word
 * overflow (+-1) into the high sum, never re-sign the pair
 *
 * ARGUMENTS:
 *   x    agc_lm_words_t  I  augend words, scale 2^S
 *   y    agc_lm_words_t  I  addend words, same scale
 *   ovf  int *           O  net overflow DAS leaves in A (+1, 0, -1)
 *
 * RETURNS: agc_lm_words_t -- sum words, the high word overflow-corrected
 **********************************************************************/
agc_lm_words_t agc_lm_das_ovf(agc_lm_words_t x, agc_lm_words_t y, int *ovf)
{
    int32_t lo = (int32_t) x.lo + (int32_t) y.lo; /* minor sum, 2^S-14 */
    int32_t hi = (int32_t) x.hi + (int32_t) y.hi; /* major sum, 2^S    */
    int     a  = 0;                               /* net overflow      */

    if (!AGC_ASSERT(ovf != NULL))
    {
        return (x);
    }

    if (lo > AGC_SP_MAX)
    {
        lo -= LM_WORD;
        hi += 1;
    }
    else if (lo < AGC_SP_MIN)
    {
        lo += LM_WORD;
        hi -= 1;
    }

    if (hi > AGC_SP_MAX)
    {
        hi -= LM_WORD;
        a  = 1;
    }
    else if (hi < AGC_SP_MIN)
    {
        hi += LM_WORD;
        a  = -1;
    }

    *ovf             = a;
    agc_lm_words_t r = {(agc_sp_t) hi, (agc_sp_t) lo}; /* result, 2^S */

    if (!AGC_ASSERT(lo >= AGC_SP_MIN && lo <= AGC_SP_MAX))
    {
        return (x); /* not reached: one carry corrects a word sum */
    }

    if (!AGC_ASSERT((int64_t) agc_lm_words_value(r) + (int64_t) a * LM_WORD * LM_WORD == (int64_t) agc_lm_words_value(x) + agc_lm_words_value(y)))
    {
        return (x); /* not reached: DAS is exact up to the overflow */
    }

    return (r);
} /* agc_lm_das_ovf */

/**********************************************************************
 * AGC: none (DAS instruction) -- DAS where the caller guarantees that
 * the sum does not overflow (asserted)
 *
 * ARGUMENTS:
 *   x  agc_lm_words_t  I  augend words, scale 2^S
 *   y  agc_lm_words_t  I  addend words, same scale
 *
 * RETURNS: agc_lm_words_t -- sum words, scale 2^S
 **********************************************************************/
agc_lm_words_t agc_lm_das(agc_lm_words_t x, agc_lm_words_t y)
{
    int            ovf = 0;                          /* net overflow  */
    agc_lm_words_t r   = agc_lm_das_ovf(x, y, &ovf); /* sum, 2^S      */

    if (!AGC_ASSERT(ovf == 0))
    {
        return (r); /* overflow-corrected words, as TS stores them */
    }

    return (r);
}

/**********************************************************************
 * AGC: none (channels 30-33 are inverted) -- true when a discrete is
 * present, i.e. its bit reads 0
 *
 * ARGUMENTS:
 *   chan  int       I  channel 030..033
 *   bit   uint16_t  I  bit mask of the discrete, non-zero
 *
 * RETURNS: bool -- true when the signal is present
 **********************************************************************/
bool agc_lm_chan_bit_present(int chan, uint16_t bit)
{
    if (!AGC_ASSERT(chan >= AGC_CHAN_CHAN30 && chan <= AGC_CHAN_CHAN33))
    {
        return (false);
    }

    if (!AGC_ASSERT(bit != 0u))
    {
        return (false);
    }

    return ((agc_chan_read(chan) & bit) == 0u);
}
