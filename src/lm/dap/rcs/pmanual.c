/*
 * pmanual.c -- P-axis state logic, minimum-impulse mode and the manual
 * rate-command mode (P-AXIS_RCS_AUTOPILOT.agc, Luminary 099 pp.1430-1436,
 * "MANUAL RATE COMMAND MODE" by Robert F. Stengel).
 *
 * Rate command has two laws: direct rate (jets fired at once when the stick
 * command changes by more than the breakout level; rate error nulled to the
 * target deadband) and pseudo-auto (body-fixed attitude and rate errors fed
 * to TJETLAW). Control moves from direct rate to pseudo-auto when the target
 * deadband is reached or after 4 s (TCP counts 40 DAP passes).
 *
 * Every path ends in a jet decision (PJETSLEC, JETSOFF or PURGENCY); the
 * caller (PAXIS) then runs QRAXIS.
 */
#include "../internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"

#define PM_CH31_ATTHOLD 0x1000u /* BIT13 of 31 = 0: attitude hold        */
#define PM_CH31_DETENT  0x4000u /* BIT15 of 31 = 0: RHC out of detent    */
#define PM_CH31_PLUSX   0x0004u /* BIT3 of 31 = 0: +X (+P) min impulse   */
#define PM_CH31_MINUSX  0x0008u /* BIT4 of 31 = 0: -X (-P) min impulse   */
#define PM_OCT14        0x000Cu /* OCT14: bits 3,4                       */
#define PM_BITS9_11     02400u  /* BITS9,11 (= EBANK5)                   */
#define PM_PQRBIT       074777u /* PQRBIT: clears RCSFLAGS bits 10,11    */
#define PM_BITS8_9      000600u /* BITS8,9: chan 13 RHC counter enable   */
#define PM_MINTJ        ((agc_sp_t) 10) /* TEN: < 14 ms, raised by PJETSLEC */
#define PM_LINRATP      46 /* LINRATP DEC 46: linear stick term     */
#define PM_40CYC        ((agc_sp_t) 40) /* 40CYC OCT 50: 4 s of DAP passes  */
#define PM_25_32        ((agc_sp_t) 12800) /* 25/32 = PRIO31 (DEC .78125)   */
#define PM_1ANETP       2     /* 1/ANETP = BLOCKTOP +2                 */
#define PM_DPWORD       16384 /* 2^14                                  */

/* ZEROENBL (p.1434): save the Q,R hand-controller counts in SAVEHAND,
 * zero the three counters and enable them (channel 13 bits 8,9).         */
agc_status_t lm_dap_zeroenbl(void)
{
    lm_dap_t *d    = lm_dap_state();
    d->savehand[0] = agc_erase_q_rhcctr();
    d->savehand[1] = agc_erase_r_rhcctr();
    agc_erase_set_p_rhcctr(0);
    agc_erase_set_q_rhcctr(0);
    agc_erase_set_r_rhcctr(0);
    agc_chan_or(013, PM_BITS8_9);
    agc_status_t st = AGC_STATUS_OK;
    if (!AGC_ASSERT(agc_erase_p_rhcctr() == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* Minimum impulse mode, P axis (p.1431): one 14 ms pulse per deflection of
 * the THC X-axis (+X/-X bits 3,4 of channel 31). OLDPMIN holds the gate
 * until the controller is released.                                       */
agc_status_t lm_dap_pminimp(void)
{
    lm_dap_t *d   = lm_dap_state();
    d->perror     = 0;
    d->cduxd      = agc_erase_cdux();
    uint16_t ch31 = agc_chan_read(031);
    if (!AGC_ASSERT(d->oldpmin >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (d->oldpmin > 0)
    { /* CHECKP         */
        d->oldpmin = (agc_sp_t) ((uint16_t) ~ch31 & PM_OCT14);
        return lm_dap_jetsoff();
    }
    if ((ch31 & PM_CH31_PLUSX) == 0u)
    { /* FIREP: +XMIN   */
        d->tj[0] = PM_MINTJ;
    }
    else if ((ch31 & PM_CH31_MINUSX) == 0u)
    { /* -XMIN          */
        d->tj[0] = (agc_sp_t) -PM_MINTJ;
    }
    else
    {
        return lm_dap_jetsoff();
    }
    d->oldpmin = 1;
    if (!AGC_ASSERT(d->tj[0] != 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return lm_dap_pjetslec_pair();
}

/* STATE LOGIC (pp.1430-1431): attitude hold uses minimum impulse (PULSES)
 * or rate command; auto uses PURGENCY when X-axis override is inhibited
 * (XOVINHIB, visibility phase), otherwise checks the stick (DETENTCK).   */
agc_status_t lm_dap_pstate(void)
{
    bool atthold = (agc_chan_read(031) & PM_CH31_ATTHOLD) == 0u;
    if (!AGC_ASSERT(agc_flag_valid(AGC_FLAG_PULSEFLG)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_flag_valid(AGC_FLAG_XOVINFLG)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!atthold)
    {
        if (agc_flag_test(AGC_FLAG_XOVINFLG))
        {
            return lm_dap_purgency(); /* attitude steer */
        }
        return lm_dap_detentck();
    }
    if (agc_flag_test(AGC_FLAG_PULSEFLG))
    { /* MANMODE        */
        return lm_dap_pminimp();
    }
    return lm_dap_detentck();
}

/* JUSTOUT -1 / JUSTOUT (pp.1433-1434): first manual pass. Rate command on
 * (OURRCBIT), zero the DP attitude errors, the commanded rates and the
 * Q,R counters, clear PBIT/QRBIT, set the just-in bit, enable counters.  */
static agc_status_t justout(lm_dap_t *d)
{
    d->perror       = 0;
    agc_flag_set(AGC_FLAG_OURRCFLG);
    d->dxerror      = 0;
    d->dxerror_lo   = 0;
    d->qerror       = 0; /* DYERROR                        */
    d->dyerror_lo   = 0;
    d->rerror       = 0; /* DZERROR                        */
    d->dzerror_lo   = 0;
    d->plast        = 0;
    d->qlast        = 0;
    d->rlast        = 0;
    agc_erase_set_q_rhcctr(0);
    agc_erase_set_r_rhcctr(0);
    d->rcsflags     = (uint16_t) (d->rcsflags & PM_PQRBIT);
    d->rcsflags     = (uint16_t) (d->rcsflags | LM_RCS_JUSTIN);
    agc_status_t sz = lm_dap_zeroenbl();
    if (!AGC_ASSERT(sz == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT((d->rcsflags & LM_RCS_PBIT) == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return lm_dap_jetsoff();
}

/* RATEDONE (p.1433): manual command and damping finished in all axes.
 * Rate command off; desired attitude = present (CDUXD only in auto,
 * X-axis override; all three via ZATTEROR in attitude hold).            */
static agc_status_t ratedone(lm_dap_t *d)
{
    agc_inhint();
    agc_flag_clear(AGC_FLAG_OURRCFLG);
    agc_status_t sf = AGC_STATUS_OK;
    agc_status_t sz = AGC_STATUS_OK;
    if ((agc_chan_read(031) & PM_CH31_ATTHOLD) != 0u)
    {
        d->cduxd = agc_erase_cdux(); /* X-axis override */
    }
    else
    {
        agc_lm_dap_zatteror();
        sz = AGC_STATUS_OK;
    }
    agc_relint();
    if (!AGC_ASSERT(sf == AGC_STATUS_OK && sz == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_OURRCFLG)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return lm_dap_purgency();
}

/* PEGI tail (pp.1435-1436): direct-rate jet time. TJP = 25/32 x 2 x
 * (-EDOTP x 1/ANETP), OVERSUB-limited; 4 jets if the rate error exceeds
 * the 2-jet limit, otherwise 2 jets for twice the time.                 */
static agc_status_t pegi_fire(lm_dap_t *d, agc_sp_t absedotp)
{
    lm_dp_pair_t p     = lm_dap_mp((agc_sp_t) -d->edot, d->block[PM_1ANETP]);
    int32_t      twice = 2 * lm_dap_dpval(p);           /* DAS A          */
    agc_sp_t     a     = lm_dap_sat(twice / PM_DPWORD); /* OVERSUB        */
    d->tj[0]           = lm_dap_mphi(a, PM_25_32);      /* T6 counts      */
    if (!AGC_ASSERT(d->neg_2jetlim <= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(absedotp >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if ((int32_t) absedotp + d->neg_2jetlim > 0)
    {
        return lm_dap_pjetslec(6); /* four jets      */
    }
    d->tj[0] = lm_dap_wrap(2 * (int32_t) d->tj[0]); /* ADS TJP        */
    return lm_dap_pjetslec_pair();
}

/* PEGI (p.1435): direct rate control. Zero the P attitude error; stay in
 * direct rate (PBIT) while the rate error is outside the target deadband
 * and TCP has not run out, then fire against the rate error.            */
static agc_status_t pegi(lm_dap_t *d)
{
    d->cduxd          = agc_erase_cdux();
    d->dxerror        = 0;
    d->dxerror_lo     = 0;
    d->perror         = 0;
    agc_sp_t absedotp = lm_dap_ccs_dim(d->edot); /* ABSEDOTP       */
    bool     outside = ((int32_t) absedotp + d->neg_ratedb > 0) && (d->tcp > 0);
    if (outside)
    {
        d->rcsflags = (uint16_t) (d->rcsflags | LM_RCS_PBIT);
    }
    else
    { /* LAST           */
        d->rcsflags = (uint16_t) (d->rcsflags & ~LM_RCS_PBIT);
    }
    if (!AGC_ASSERT(d->neg_ratedb <= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(d->tcp >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return pegi_fire(d, absedotp);
}

/* RATERROR (pp.1434-1435): linear/quadratic stick scaling of P-RHCCTR to
 * the commanded rate PLAST (at 45 deg/s), EDOTP = OMEGAP - PLAST. A
 * command change beyond the breakout level starts direct rate (TCP = 4 s);
 * otherwise direct rate continues (PBIT) or pseudo-auto runs TJETLAW on
 * DXERROR.                                                               */
static agc_status_t raterror(lm_dap_t *d)
{
    d->cduxd     = agc_erase_cdux();
    agc_sp_t cnt = agc_erase_p_rhcctr();
    if (!AGC_ASSERT(cnt >= LM_NEGMAX && cnt <= LM_POSMAX))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_sp_t k   = lm_dap_sat(4 * (int32_t) lm_dap_ccs_dim(cnt) + PM_LINRATP);
    agc_sp_t lin = lm_dap_mp(k, cnt).lo; /* CA L           */
    agc_sp_t cmd = lm_dap_mphi(lin, d->stiksens);
    agc_sp_t change = lm_dap_wrap((int32_t) cmd - d->plast); /* DAPTEMP1    */
    d->plast        = cmd;
    agc_status_t sz = lm_dap_zeroenbl();
    d->edot = lm_dap_wrap((int32_t) d->omegap - d->plast); /* EDOTP       */
    if (!AGC_ASSERT(sz == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (change != 0 && (int32_t) lm_dap_ccs_dim(change) + d->neg_ratedb > 0)
    {
        d->tcp = PM_40CYC;
        return pegi(d);
    }
    if ((d->rcsflags & LM_RCS_PBIT) != 0u)
    {
        return pegi(d);
    }
    d->e      = d->dxerror; /* pseudo-auto    */
    d->perror = d->dxerror;
    return lm_dap_purgency_law();
}

/* DETENTCK / RUTH / RATEDAMP / RHCMOVED (pp.1432-1434): the rate-command
 * mode logic driven by the RHC detent switch (channel 31 bit 15).        */
agc_status_t lm_dap_detentck(void)
{
    lm_dap_t *d = lm_dap_state();
    d->ch31temp = agc_chan_read(031);
    bool ourrc  = agc_flag_test(AGC_FLAG_OURRCFLG);
    if ((d->ch31temp & PM_CH31_DETENT) == 0u)
    { /* RHCMOVED       */
        return ourrc ? raterror(d) : justout(d);
    }
    if (!ourrc)
    {
        return lm_dap_purgency();
    }
    if ((d->rcsflags & LM_RCS_JUSTIN) != 0u)
    { /* just in detent */
        if ((agc_chan_read(031) & PM_CH31_ATTHOLD) != 0u)
        { /* auto      */
            d->rcsflags = (uint16_t) (d->rcsflags & ~PM_BITS9_11);
        }
    }
    else if ((d->rcsflags & (LM_RCS_PBIT | LM_RCS_QRBIT)) == 0u)
    { /* RUTH */
        return ratedone(d);
    }
    agc_erase_set_p_rhcctr(0); /* RATEDAMP       */
    if (!AGC_ASSERT(d->ch31temp <= 077777u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_erase_p_rhcctr() == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return raterror(d);
}
