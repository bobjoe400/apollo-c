/*
 * throttle.c -- THROTTLE_CONTROL_ROUTINES.agc (Luminary 099, pp. 793-797).
 *
 * The descent engine throttle law. Guidance asks for an acceleration
 * /AFC/; THROTTLE converts it and the measured acceleration into forces in
 * throttle pulse units (FC desired, FP present), applies the 10%..94% region
 * logic with its "throttle-up" to the fixed full-thrust region, and sends
 * the difference PIF = FC - FP (plus an offset PIFPSET) to the THRUST
 * counter. It finally computes FWEIGHT, the correction to FP that accounts
 * for the thrust change happening part-way through the next PIPA interval.
 *
 * Units: forces are DP with the high word in pulses (2^14 pulses), i.e. the
 * stored value / 2^14 = pulses. BITPERF = 0.0797959872 pulse per newton.
 */
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

/* SCALEFAC 2DEC* +7.97959872E+2 B-16* (CONTROLLED_CONSTANTS): pulses per
 * (kg m/cs^2) = 1E4 * BITPERF, scale 2^16.                                */
#define THR_SCALEFAC ((agc_dp_t) 3268444)
/* /AF/CNST DEC .13107: ABDELV (2^14 cm/s per 2 s) -> 2^-4 m/cs^2.         */
#define THR_AF_CNST                 ((agc_sp_t) 2147)
#define THR_TWOSECS                 200 /* 2SECS, P66 guidance period x 2 (cs)     */
#define THR_FOURSECS                400 /* 4SECS, P63/P64 guidance period x 2 (cs) */
#define THR_FRATE_BIT6              32 /* BIT6: FRATE, 32 pulses per centisecond  */
#define THR_LOW8                    0377 /* LOW8                                    */
#define THR_RATE_OF_DESCENT_LANDING 66

/* DP built from a high and low word without touching core semantics.     */
static agc_dp_t thr_pack(int32_t hi, int32_t lo)
{
    if (!AGC_ASSERT(hi <= AGC_SP_MAX && hi >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(lo <= AGC_SP_MAX && lo >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    return ((agc_dp_t) (hi * 16384 + lo));
}

static agc_sp_t thr_hi(agc_dp_t v)
{
    int32_t hi = v / 16384; /* sign-agreeing DP pair     */

    if (!AGC_ASSERT(hi <= AGC_SP_MAX && hi >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) hi);
}

static agc_sp_t thr_lo(agc_dp_t v)
{
    int32_t lo = v % 16384;

    if (!AGC_ASSERT(lo <= AGC_SP_MAX && lo >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) lo);
}

/* MASSMULT (p. 796): scale an acceleration arriving in A,L in units of
 * 2^-4 m/cs^2 to a force in pulse units: DMP MASS, DMP SCALEFAC, TPAGREE.
 * If the triple's top word is non-zero the force is beyond one word of
 * pulses and POSMAX is returned (DEVIATION: low word returned as 0; the AGC
 * leaves whatever L held). Otherwise the result is (MPAC+1, MPAC+2).     */
agc_dp_t agc_lm_massmult(agc_dp_t accel)
{
    agc_dp_t mass = agc_guid_state()->lm.mass; /* MASS, 2^16 kg          */

    if (!AGC_ASSERT(mass >= 0))
    {
        return (0);
    }

    if (!AGC_ASSERT(accel <= AGC_DP_MAX && accel >= AGC_DP_MIN))
    {
        return (0);
    }

    /* First DMP leaves a TP in MPAC; the second DMP uses only MPAC,+1.   */
    agc_dp_t ma  = agc_tp_to_dp(agc_dp_mul_tp(accel, mass));
    agc_tp_t f   = agc_dp_mul_tp(ma, THR_SCALEFAC); /* TP, 2^28 pulses       */
    int64_t  top = f / ((int64_t) 1 << 28);

    if (top != 0)
    {
        return (thr_pack(AGC_SP_MAX, 0));
    }

    return ((agc_dp_t) f); /* (MPAC+1,MPAC+2): /2^14 = pulses */
}

/* WHERETO .. FLATOUT2 (pp. 793-794): choose between the throttleable
 * region and the fixed (full throttle) region. Returns PIFPSET and may
 * replace FCODD. FCOLD > HIGHCRIT means we were at full throttle.       */
static agc_sp_t thr_region(agc_lm_thr_t *t, const agc_lm_pad_t *pad)
{
    agc_sp_t lowcrit  = pad->lowcrit;  /* L*WCR*T, pad loaded LOWCRIT    */
    agc_sp_t highcrit = pad->highcrit; /* H*GHCR*T, pad loaded HIGHCRIT  */
    agc_sp_t fc       = thr_hi(t->fcodd);

    if (!AGC_ASSERT(lowcrit >= 0 && highcrit >= 0))
    {
        return (0);
    }

    /* PIFPSET = -0 initially; DEVIATION: modelled as +0.                 */
    if (t->fcold > highcrit)
    {
        if (fc <= lowcrit)
        {
            /* FCOMPSET, note 2: returning below 55%: PIFPSET = FP-FMAXODD */
            return (agc_lm_ad(thr_hi(t->fp), (agc_sp_t) -AGC_LM_FMAXODD));
        }

        /* Note 1: stay at full throttle; FC = FP so PIF is zero.
         * DEVIATION: the AGC's low word is whatever L held; we use 0.    */
        t->fcodd = thr_pack(thr_hi(t->fp), 0);

        return ((agc_sp_t) AGC_LM_FEXTRA); /* FLATOUT1   */
    }

    if (fc <= highcrit)
    {
        return (0); /* to DOPIF   */
    }

    /* LOWFCOLD: throttle-up into the fixed region.                       */
    t->fcodd = thr_pack(AGC_LM_FMAXPOS, 0);

    return ((agc_sp_t) AGC_LM_FEXTRA);
}

/* DOIT (p. 795): send PIF + PIFPSET to the THRUST counter, enable the
 * thrust drive (channel 14 bit 4) and note the time of throttling.      */
static agc_status_t thr_doit(agc_lm_thr_t *t)
{
    if (!AGC_ASSERT(t->pifpset >= -AGC_LM_FEXTRA &&
                    t->pifpset <= AGC_LM_FEXTRA))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_sp_t cmd = agc_lm_ad(thr_hi(t->pif), t->pifpset);

    t->pseudo55 = cmd;
    agc_erase_set_thrust(cmd);
    agc_chan_or(AGC_CHAN_CHAN14, AGC_CH014_THRUSTACT);
    t->tthrot = agc_lm_time1();

    return (AGC_STATUS_OK);
}

/* FWEIGHT computation (pp. 795-796):
 *              PIF (PPROCESS + TL)     PIF |PIF|
 *   FWEIGHT =  ------------------- + --------------
 *                    PGUID           2 PGUID FRATE
 * PGUID is 1 s in P66 (time base THISTPIP) and 2 s otherwise (PIPTIME).
 * The AGC doubles only the high word of the first term (DOUBLE); kept.  */
static void thr_fweight(agc_lm_thr_t *t)
{
    agc_sp_t buf; /* low word of time of last PIPA read  */
    agc_sp_t q;   /* 2 PGUID in cs                       */

    if (agc_erase_modreg() == THR_RATE_OF_DESCENT_LANDING)
    {
        buf = thr_lo(agc_lm_ldg()->thistpip);
        q   = THR_TWOSECS;
    }
    else
    {
        buf = thr_lo(agc_erase_piptime());
        q   = THR_FOURSECS;
    }

    agc_sp_t buf1 = (agc_sp_t) (q * THR_FRATE_BIT6); /* 2 PGUID FRATE    */
    int32_t  dtp  = ((int32_t) agc_lm_time1() - buf + AGC_LM_THROTLAG) &
                    THR_LOW8;
    agc_sp_t ratio = agc_lm_dv((agc_dp_t) (dtp * 16384), q);
    agc_sp_t pif   = thr_hi(t->pif);
    agc_dp_t first = agc_sp_mul(ratio, pif); /* MP PIF           */

    t->fweight = thr_pack(2 * (int32_t) thr_hi(first), thr_lo(first));
    agc_sp_t abspif = (agc_sp_t) ((pif < 0) ? -pif : pif);
    agc_dp_t sq     = agc_sp_mul(abspif, pif); /* PIF |PIF|        */
    agc_sp_t second = agc_lm_dv(sq, buf1);

    t->fweight = (agc_dp_t) (t->fweight + (agc_dp_t) second * 16384); /* DAS */

    if (!AGC_ASSERT(buf1 > 0))
    {
        return;
    }

    if (!AGC_ASSERT(dtp >= 0 && dtp <= THR_LOW8))
    {
        return;
    }
}

/* AFDUMP .. THDUMP (pp. 793-796), entered with the present acceleration
 * in units of 2^-4 m/cs^2 (THROTTLE +3 entry used by P66 RODCOMP).      */
agc_status_t agc_lm_throttle_measured(agc_dp_t accel)
{
    agc_lm_thr_t       *t   = agc_lm_thr();
    const agc_lm_pad_t *pad = agc_lm_pad();

    if (!AGC_ASSERT(accel >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_lm_ldg()->afc >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    t->fp = agc_lm_massmult(accel); /* FP present thrust   */
    agc_dp_t fcfull = agc_lm_massmult(agc_lm_ldg()->afc);

    t->fc    = thr_pack(thr_hi(fcfull), thr_lo(t->fc)); /* TS FC: high only  */
    t->fcodd = fcfull; /* FCODD               */

    /* Less than 3 s since the last throttling: augment FP by FWEIGHT.
     * (Assumes a FLATOUT within 80 s before the first THROTTLE call.)   */
    int32_t since = ((int32_t) agc_lm_time1() - t->tthrot) & AGC_SP_MAX;

    if (AGC_LM_3SECS - since > 0)
    {
        t->fp = (agc_dp_t) (t->fp + t->fweight); /* DAS FP              */
    }

    t->pifpset = thr_region(t, pad);

    /* DOPIF */
    agc_lm_fastchng();
    t->fcold = thr_hi(t->fcodd);
    t->pif   = (agc_dp_t) (t->fcodd - t->fp); /* PIF = FC - FP       */
    agc_status_t st = thr_doit(t);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    thr_fweight(t); /* for use next pass   */

    return (AGC_STATUS_OK);
}

/* THROTTLE (p. 793): present acceleration from ABDELV, saving SERVICER
 * the trouble: ABDELV * /AF/CNST -> 2^-4 m/cs^2.                        */
agc_status_t agc_lm_throttle(void)
{
    agc_sp_t abdelv = agc_guid_state()->lm.abdelv; /* CA ABDELV     */

    if (!AGC_ASSERT(abdelv >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t accel = agc_sp_mul(abdelv, THR_AF_CNST); /* MP /AF/CNST   */

    if (!AGC_ASSERT(accel >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_throttle_measured(accel));
}

/* WHATOUT (p. 796): command a fixed pulse count through PIFPSET so that
 * FWEIGHT comes out zero; FCOLD and the PIF high word become -0
 * (DEVIATION: +0). FLATOUT is WHATOUT with BIT13 = 4096 pulses.        */
agc_status_t agc_lm_throttle_whatout(agc_sp_t pulses)
{
    agc_lm_thr_t *t = agc_lm_thr();

    if (!AGC_ASSERT(pulses >= -AGC_LM_FEXTRA && pulses <= AGC_LM_FEXTRA))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    t->pifpset = pulses;
    t->fcold   = 0;
    t->pif     = thr_pack(0, thr_lo(t->pif));      /* TS PIF        */
    agc_status_t st = thr_doit(t);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    thr_fweight(t);

    return (AGC_STATUS_OK);
}

/* FLATOUT (p. 796): throttle the descent engine up to full. */
agc_status_t agc_lm_throttle_flatout(void)
{
    agc_status_t st = agc_lm_throttle_whatout((agc_sp_t) AGC_LM_FEXTRA);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_lm_thr()->pifpset == AGC_LM_FEXTRA))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}
