/**********************************************************************
 * FILE NAME: throttle.c
 *
 * PURPOSE:
 *   The descent engine throttle law. Guidance asks for an acceleration
 *   /AFC/; THROTTLE converts it and the measured acceleration into forces
 *   in throttle pulse units (FC desired, FP present), applies the
 *   10%..94% region logic with its "throttle-up" to the fixed full-thrust
 *   region, and sends PIF = FC - FP (plus an offset PIFPSET) to the
 *   THRUST counter. It finally computes FWEIGHT, the correction to FP for
 *   the thrust change that happens part-way through the next PIPA
 *   interval.
 *
 *   Forces are DP with the high word in pulses (value / 2^14 = pulses;
 *   BITPERF = 0.0797959872 pulse per newton). FP, FCODD, PIF and FWEIGHT
 *   are kept as the AGC's two words (agc_lm_words_t): DAS leaves them
 *   sign-disagreeing and the code then reads the high word alone (CA PIF,
 *   AD FP, MP PIF), which is one pulse off the high word of the value
 *   (oracle finding F9). Clock differences are taken in ones' complement
 *   (CS / AD / MASK), as the AGC does.
 *
 * AGC SOURCE:
 *   Luminary099 THROTTLE_CONTROL_ROUTINES.agc, pp. 793-797 (bank 31)
 *
 * EXTERNAL VARIABLES:
 *   lane state through agc_lm_thr(), agc_lm_ldg(), agc_lm_pad();
 *   MASS and ABDELV through agc_guid_state(); THRUST counter, MODREG,
 *   PIPTIME through the core erasable accessors
 *
 * EXTERNAL REFERENCES:
 *   agc_lm_fastchng, agc_lm_das, agc_lm_dv, agc_lm_ad, agc_lm_time1,
 *   agc_dp_mul_tp, agc_sp_mul, agc_chan_or, agc_erase_set_thrust
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; out-of-range inputs are reported by AGC_ASSERT and refused
 *   with AGC_STATUS_BAD_ARG
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/guidance/guidance.h"
#include "agc/lm/landing.h"

/* SCALEFAC 2DEC* +7.97959872E+2 B-16* (CONTROLLED_CONSTANTS): pulses per
 * (kg m/cs^2) = 1E4 * BITPERF. /AF/CNST DEC .13107 (p. 797): ABDELV
 * (2^14 cm/s per 2 s) -> 2^-4 m/cs^2.                                    */
#define THR_SCALEFAC ((agc_dp_t) 3268444) /* SCALEFAC, scale 2^16     */
#define THR_AF_CNST  ((agc_sp_t) 2147)    /* /AF/CNST, scale 2^0      */
/* 2SECS / 4SECS: twice the guidance period of P66 / of P63 and P64.     */
#define THR_TWOSECS    200    /* 2SECS, cs                            */
#define THR_FOURSECS   400    /* 4SECS, cs                            */
#define THR_FRATE_BIT6 32     /* BIT6: FRATE, pulses per cs           */
#define THR_LOW8       0377   /* LOW8 mask                            */
#define THR_POSMAX     037777 /* POSMAX mask                          */
#define THR_A_ONES     0xFFFF /* -0 in the 16-bit A register          */

#define THR_RATE_OF_DESCENT_LANDING 66 /* DEC66: MODREG of P66 */
/* L after TPAGREE is TPAGREE's return address, the CA MPAC at 31,2410 in
 * MASSMULT (assembled rope; verified with the oracle probe): it becomes
 * the low word of an overflowed MASSMULT result.                          */
#define THR_MASSMULT_L 02410 /* address word left in L by TPAGREE    */

/**********************************************************************
 * AGC: none (CS b / AD a / AD c idiom) -- the 16-bit ones'-complement A
 * register after a - b + c, as the following MASK sees it
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I  clock word (TIME1), cs, 0..POSMAX
 *   b  agc_sp_t  I  clock word subtracted (TTHROT, PIPTIME +1), cs
 *   c  agc_sp_t  I  non-negative offset added afterwards (THROTLAG), cs
 *
 * RETURNS: int32_t -- the A register bits, 0..0xFFFF: a positive sum as
 *          itself, zero as -0 (0xFFFF), a negative sum -n as 0xFFFF - n
 **********************************************************************/
static int32_t thr_csad(agc_sp_t a, agc_sp_t b, agc_sp_t c)
{
    int32_t sum = (int32_t) a - (int32_t) b + (int32_t) c; /* cs     */

    if (!AGC_ASSERT(a >= 0 && b >= 0) || !AGC_ASSERT(c >= 0))
    {
        return (0); /* no valid result */
    }

    /* x + ~x is -0 in ones' complement, so a zero sum is all ones. */
    return ((sum > 0) ? sum : THR_A_ONES + sum); /* ones' complement */
}

/**********************************************************************
 * MASSMULT (THROTTLE_CONTROL_ROUTINES.agc p. 796) -- force from an
 * acceleration: DMP MASS, DMP SCALEFAC, TPAGREE; returns (MPAC+1,
 * MPAC+2), or (POSMAX, L) when the triple's top word is non-zero
 *
 * ARGUMENTS:
 *   accel  agc_dp_t  I  acceleration, 2^-4 m/cs^2, >= 0
 *
 * RETURNS: agc_lm_words_t -- force, high word in pulses (2^14 pulses)
 **********************************************************************/
static agc_lm_words_t thr_massmult(agc_dp_t accel)
{
    agc_lm_words_t force = {AGC_SP_MAX, THR_MASSMULT_L}; /* overflow */
    agc_dp_t       mass  = /* MASS, 2^16 kg          */
                           agc_guid_state()->lm.mass;

    if (!AGC_ASSERT(mass >= 0) || !AGC_ASSERT(accel >= 0))
    {
        return (agc_lm_words(0));
    }

    /* First DMP leaves a TP in MPAC; the second DMP uses only MPAC,+1. */
    agc_dp_t ma  = agc_tp_to_dp(agc_dp_mul_tp(accel, mass)); /* 2^12 */
    agc_tp_t f   = agc_dp_mul_tp(ma, THR_SCALEFAC); /* TP, 2^28 pulses   */
    int64_t  top = f / ((int64_t) 1 << 28);        /* MPAC after TPAGREE */

    if (top == 0)
    {
        force = agc_lm_words((agc_dp_t) f); /* DXCH MPAC +1          */
    }

    return (force);
}

/**********************************************************************
 * WHERETO .. FLATOUT2 (THROTTLE_CONTROL_ROUTINES.agc pp. 793-794) --
 * choose between the throttleable region and the fixed (full throttle)
 * region; may replace FCODD
 *
 * ARGUMENTS:
 *   t    agc_lm_thr_t *        I/O  throttle erasables (FCODD may change)
 *   pad  const agc_lm_pad_t *  I    LOWCRIT, HIGHCRIT, pulses
 *
 * RETURNS: agc_sp_t -- PIFPSET, pulses: 0, FP - FMAXODD or FEXTRA
 **********************************************************************/
static agc_sp_t thr_region(agc_lm_thr_t *t, const agc_lm_pad_t *pad)
{
    agc_sp_t lowcrit  = pad->lowcrit; /* L*WCR*T, pulses, unscaled */
    agc_sp_t highcrit = pad->highcrit; /* H*GHCR*T, pulses, unscaled */
    agc_sp_t fc       = t->fcodd.hi; /* AD FCODD, pulses, unscaled */

    if (!AGC_ASSERT(lowcrit >= 0 && highcrit >= 0) ||
        !AGC_ASSERT(lowcrit <= highcrit))
    {
        return (0);
    }

    /* PIFPSET = -0 initially; DEVIATION: modelled as +0.               */
    if (t->fcold > highcrit)
    {
        if (fc <= lowcrit)
        { /* FCOMPSET, note 2: back below 55%: PIFPSET = FP - FMAXODD   */
            return (agc_lm_ad(t->fp.hi, (agc_sp_t) -AGC_LM_FMAXODD));
        }

        /* Note 1: stay at full throttle with FC = FP so PIF is zero.
         * DEVIATION: the low word is L = BUF +1 (H*GHCR*T), scratch
         * shared with other routines; modelled as 0.                     */
        t->fcodd.hi = t->fp.hi; /* CA FP, FLATOUT1: DXCH FCODD          */
        t->fcodd.lo = 0;

        return ((agc_sp_t) AGC_LM_FEXTRA);
    }

    if (fc <= highcrit)
    {
        return (0); /* to DOPIF  */
    }

    /* LOWFCOLD: throttle-up into the fixed region (same L deviation).  */
    t->fcodd.hi = (agc_sp_t) AGC_LM_FMAXPOS;
    t->fcodd.lo = 0;

    return ((agc_sp_t) AGC_LM_FEXTRA);
}

/**********************************************************************
 * DOIT (THROTTLE_CONTROL_ROUTINES.agc p. 795) -- send the high word of
 * PIF plus PIFPSET to the THRUST counter, enable the thrust drive
 * (channel 14 bit 4) and note the time of throttling
 *
 * ARGUMENTS:
 *   t  agc_lm_thr_t *  I/O  throttle erasables (PSEUDO55, TTHROT set)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when
 *          PIFPSET is out of range
 **********************************************************************/
static agc_status_t thr_doit(agc_lm_thr_t *t)
{
    if (!AGC_ASSERT(t->pifpset >= -AGC_LM_FEXTRA) ||
        !AGC_ASSERT(t->pifpset <= AGC_LM_FEXTRA))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_sp_t cmd = agc_lm_ad(t->pif.hi, t->pifpset); /* pulses, unscaled */

    t->pseudo55 = cmd;
    agc_erase_set_thrust(cmd);
    agc_chan_or(AGC_CHAN_CHAN14, AGC_CH014_THRUSTACT);
    t->tthrot = agc_lm_time1();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * FWEIGHT computation, DOIT +7 .. THDUMP (THROTTLE_CONTROL_ROUTINES.agc
 * pp. 795-796):
 *              PIF (PPROCESS + TL)     PIF |PIF|
 *   FWEIGHT =  ------------------- + --------------
 *                    PGUID           2 PGUID FRATE
 * PGUID is 1 s in P66 (time base THISTPIP) and 2 s otherwise (PIPTIME).
 * The AGC doubles only the high word of the first term (DOUBLE); kept.
 *
 * ARGUMENTS:
 *   t  agc_lm_thr_t *  I/O  throttle erasables (FWEIGHT set)
 **********************************************************************/
static void thr_fweight(agc_lm_thr_t *t)
{
    agc_sp_t buf = agc_dp_low(agc_erase_piptime()); /* cs, unscaled */
    agc_sp_t q   = THR_FOURSECS; /* 2 PGUID, cs, unscaled */

    if (agc_erase_modreg() == THR_RATE_OF_DESCENT_LANDING)
    { /* FWCOMP: P66, one-second guidance from THISTPIP               */
        buf = agc_dp_low(agc_lm_ldg()->thistpip);
        q   = THR_TWOSECS;
    }

    agc_sp_t buf1 = (agc_sp_t) (q * THR_FRATE_BIT6); /* pulses, unscaled */
    int32_t  dtp  = thr_csad(agc_lm_time1(), buf, AGC_LM_THROTLAG) &
                    THR_LOW8; /* PPROCESS + TL, cs ("small and positive")  */

    if (!AGC_ASSERT(buf1 > 0) || !AGC_ASSERT(dtp >= 0 && dtp <= THR_LOW8))
    {
        return;
    }

    agc_sp_t ratio = agc_lm_dv((agc_dp_t) dtp * 16384, q); /* scale 2^0 */
    agc_sp_t pif   = t->pif.hi; /* MP PIF, pulses, unscaled */
    agc_dp_t first = agc_sp_mul(ratio, pif); /* 2^14 pulses            */

    /* DOUBLE (high word only), DXCH FWEIGHT                          */
    t->fweight.hi = (agc_sp_t) (2 * agc_dp_high(first));
    t->fweight.lo = agc_dp_low(first);
    agc_sp_t       abspif = (agc_sp_t) ((pif < 0) ? -pif : pif); /* unscaled */
    agc_lm_words_t second = {
        agc_lm_dv(agc_sp_mul(abspif, pif), buf1), 0
    };                                                /* DV BUF +1, ZL */

    t->fweight = agc_lm_das(t->fweight, second); /* DAS FWEIGHT        */
}

/**********************************************************************
 * THROTTLE +3 / AFDUMP .. THDUMP (THROTTLE_CONTROL_ROUTINES.agc pp.
 * 793-796) -- the throttle law, entered with the present acceleration
 * (the THROTTLE +3 entry used by P66 RODCOMP)
 *
 * ARGUMENTS:
 *   accel  agc_dp_t  I  present acceleration, 2^-4 m/cs^2, >= 0
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a
 *          negative acceleration or /AFC/
 **********************************************************************/
agc_status_t agc_lm_throttle_measured(agc_dp_t accel)
{
    agc_lm_thr_t       *t   = agc_lm_thr(); /* throttle erasables  */
    const agc_lm_pad_t *pad = agc_lm_pad(); /* LOWCRIT, HIGHCRIT   */

    if (!AGC_ASSERT(accel >= 0) || !AGC_ASSERT(agc_lm_ldg()->afc >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    t->fp = thr_massmult(accel); /* FP = present thrust                */
    agc_lm_words_t fcfull = thr_massmult(agc_lm_ldg()->afc); /* 2^14 */

    t->fc    = agc_dp_pack(fcfull.hi, agc_dp_low(t->fc)); /* TS FC   */
    t->fcodd = fcfull;                                     /* DXCH    */

    /* Less than 3 s since the last throttling: augment FP by FWEIGHT.
     * (Assumes a FLATOUT within 80 s before the first THROTTLE call.)  */
    int32_t since = /* cs since the last throttling */
                    thr_csad(agc_lm_time1(), t->tthrot, 0) & THR_POSMAX;

    if (AGC_LM_3SECS - since > 0)
    {
        t->fp = agc_lm_das(t->fp, t->fweight); /* DAS FP               */
    }

    t->pifpset = thr_region(t, pad);

    /* DOPIF: PIF = FCODD - FP by DAS, "never equals +0".               */
    agc_lm_fastchng();
    t->fcold = t->fcodd.hi;
    agc_lm_words_t minus_fp = {(agc_sp_t) -t->fp.hi,
                               (agc_sp_t) -t->fp.lo}; /* DCS FP        */

    t->pif = agc_lm_das(t->fcodd, minus_fp);
    agc_status_t st = thr_doit(t); /* DOIT status        */

    if (st == AGC_STATUS_OK)
    {
        thr_fweight(t); /* for use next pass                          */
    }

    return (st);
} /* agc_lm_throttle_measured */

/**********************************************************************
 * THROTTLE (THROTTLE_CONTROL_ROUTINES.agc p. 793) -- the throttle law
 * with the present acceleration from ABDELV, "saving SERVICER trouble":
 * ABDELV * /AF/CNST -> 2^-4 m/cs^2
 *
 * RETURNS: agc_status_t -- status of agc_lm_throttle_measured, or
 *          AGC_STATUS_BAD_ARG when ABDELV is negative
 **********************************************************************/
agc_status_t agc_lm_throttle(void)
{
    agc_sp_t abdelv = agc_guid_state()->lm.abdelv; /* 2^14 cm/s / 2 s */

    if (!AGC_ASSERT(abdelv >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t accel = agc_sp_mul(abdelv, THR_AF_CNST); /* 2^-4 m/cs^2  */

    if (!AGC_ASSERT(accel >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_throttle_measured(accel));
}

/**********************************************************************
 * WHATOUT (THROTTLE_CONTROL_ROUTINES.agc p. 796) -- command a fixed
 * pulse count through PIFPSET so that FWEIGHT comes out zero; FCOLD and
 * the PIF high word become -0 (DEVIATION: +0, same pulses and FWEIGHT)
 *
 * ARGUMENTS:
 *   pulses  agc_sp_t  I  pulses to send, |pulses| <= FEXTRA
 *
 * RETURNS: agc_status_t -- status of DOIT, or AGC_STATUS_BAD_ARG
 **********************************************************************/
static agc_status_t thr_whatout(agc_sp_t pulses)
{
    agc_lm_thr_t *t = agc_lm_thr(); /* throttle erasables             */

    if (!AGC_ASSERT(pulses >= -AGC_LM_FEXTRA) ||
        !AGC_ASSERT(pulses <= AGC_LM_FEXTRA))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    t->pifpset = pulses;
    t->fcold   = 0;
    t->pif.hi  = 0;                /* TS PIF: the high word only      */
    agc_status_t st = thr_doit(t); /* DOIT status                     */

    if (st == AGC_STATUS_OK)
    {
        thr_fweight(t);
    }

    return (st);
}

/**********************************************************************
 * FLATOUT (THROTTLE_CONTROL_ROUTINES.agc p. 796) -- throttle the descent
 * engine up to full: WHATOUT with BIT13 = 4096 pulses
 *
 * RETURNS: agc_status_t -- status of WHATOUT
 **********************************************************************/
agc_status_t agc_lm_throttle_flatout(void)
{
    agc_status_t st = thr_whatout((agc_sp_t) AGC_LM_FEXTRA); /* status */

    if (!AGC_ASSERT(st == AGC_STATUS_OK) ||
        !AGC_ASSERT(agc_lm_thr()->pifpset == AGC_LM_FEXTRA))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}
