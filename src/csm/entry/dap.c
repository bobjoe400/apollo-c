/*
 * dap.c -- CM_ENTRY_DIGITAL_AUTOPILOT.agc (Comanche055,
 * pp. 1063-1092): the CM entry DAP.
 *
 *  * READGYMB (.1 s waitlist task): gimbal angle differences, BODYRATE
 *    (P, Q, R x TCDU/180), then via T5RUPT ATTRATES: ALFA, BETA, ROLL
 *    integration, the atmospheric (rate damping) or extra-atmospheric
 *    (attitude hold) pitch/yaw jets on channel 5;
 *  * every 2 s (SETJTAG from READACCS) the predictive roll system CM/RCS:
 *    phase-plane solution TON1/TOFF/TON2 executed by JETCALL waitlist
 *    tasks on channel 6;
 *  * CM/FDAI: roll (and pitch/yaw) attitude error needles via NEEDLER.
 *
 * All angles are SP 2s complement /180 (angle/180 deg x 2^14) and rates
 * are x TCDU/180 (TCDU = .1 s), as in the AGC. The AGC arithmetic is
 * ones' complement basic code; it is reproduced with int32 sums and the
 * MP / DV high-word semantics. DEVIATION: ones' complement edge cases
 * (-0 tags, one-LSB differences of overflow correction, the SR register's
 * rounding) are modelled in 2s complement; see docs/modules/guid-entry.md.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"

/* ---- module erasables (EBANK6, "ANGLE REGISTERS FOR ENTRY DAPS") ------ */
static agc_sp_t aog, aig, amg; /* AOG AIG AMG gimbal /180       */
static agc_sp_t m_delaog, m_delaig, m_delamg; /* -DELAOG.. 1s comp diffs   */
static agc_sp_t oldelp, oldelq, oldelr; /* previous body-rate increments */
static agc_sp_t prel, qrel, rrel;       /* P Q R x TCDU/180              */
static agc_sp_t betadot, phidot;        /* BETADOT, PHIDOT x TCDU/180    */
static agc_sp_t jetag;                  /* JETAG: +1 roll due, 0, -1 mode+1 */
static agc_sp_t paxerr1;                /* PAXERR1 integrated roll err/360 */
static agc_sp_t ak, ak1, ak2;           /* AK (= PAXERR) AK1 AK2 needles */
static agc_sp_t qaxerr, raxerr;         /* QAXERR RAXERR errors /180     */
static agc_sp_t tused;                  /* TUSED: -TIME1 at SETJTAG      */
static agc_sp_t errorz;                 /* ERRORZ initial roll error (TM)*/
static agc_sp_t mvt_180e;               /* -VT/180E (edit)               */
static agc_sp_t vdt_180;                /* VDT/180 drift velocity        */
static bool     needler_init;           /* RCSFLAGS BIT3: init NEEDLER   */
static agc_sp_t sw_ndx;                 /* SW/NDX TM alternation/index   */
static agc_sp_t cmtmtime;               /* CMTMTIME (UPBUFF)             */
static agc_sp_t endbuf[3 * 5];          /* UPBUFF +2..+16: P Q R x 5     */

/* JETCALL list: TON1/T1BITS, TOFF/TBITS, TON2/T2BITS (cs; >0 call
 * WAITLIST, +0 bits apply for the rest of T, <0 processed).               */
#define DAP_JETLIST 3
static agc_sp_t jet_time[DAP_JETLIST];
static uint16_t jet_bits[DAP_JETLIST];
static uint16_t nujet; /* NUJET roll jet bits to write  */

/* ---- SP helpers ----------------------------------------------------- */

/* ANGOVCOR / CORANGOV (pp. 1065, 888): wrap an angle/180 sum. */
agc_sp_t entry_angle_wrap(int32_t sum)
{
    int32_t s = sum;

    if (!AGC_ASSERT(s > -3 * 16384 && s < 3 * 16384))
    {
        return (0); /* no valid result */
    }

    if (s >= 16384)
    {
        s -= 32768;
    }
    else if (s < -16384)
    {
        s += 32768;
    }

    if (!AGC_ASSERT(s >= -16384 && s <= 16383))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) s);
}

/* TS / ADS / XCH of an overflowed A: memory gets SG and bits 1-14, so a
 * sum past +-1.0 comes back by 1.0 (the ones' complement end-around LSB
 * is not modelled).                                                      */
agc_sp_t entry_sp_store_ovf(int32_t sum)
{
    int32_t s = sum;

    if (!AGC_ASSERT(s > -2 * 16384 && s < 2 * 16384))
    {
        return (0); /* no valid result */
    }

    if (s >= 16384)
    {
        s -= 16384;
    }
    else if (s <= -16384)
    {
        s += 16384;
    }

    return ((agc_sp_t) s);
}

/* AD whose result the AGC keeps without an overflow test: saturate.     */
agc_sp_t entry_sp_sat(int32_t sum)
{
    if (!AGC_ASSERT(sum > -65536 && sum < 65536))
    {
        return (0); /* no valid result */
    }

    if (sum > (int32_t) AGC_SP_MAX)
    {
        return (AGC_SP_MAX);
    }

    if (sum < (int32_t) AGC_SP_MIN)
    {
        return (AGC_SP_MIN);
    }

    if (!AGC_ASSERT(sum >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) sum);
}

/* MP: high word of the SP x SP product (truncated toward zero).        */
agc_sp_t entry_sp_mp(agc_sp_t a, agc_sp_t b)
{
    agc_sp_t x = (a < AGC_SP_MIN) ? AGC_SP_MIN : a; /* -16384 = -1.0    */
    agc_sp_t y = (b < AGC_SP_MIN) ? AGC_SP_MIN : b;

    if (!AGC_ASSERT(x >= AGC_SP_MIN && y >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(x <= AGC_SP_MAX && y <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_high(agc_sp_mul(x, y)));
}

/* SP angle from a raw 15-bit CDU counter (2s complement /180).          */
static agc_sp_t cdu_sp(uint16_t raw)
{
    int32_t w = (int32_t) (raw & 077777u);

    if (w >= 16384)
    {
        w -= 32768;
    }

    if (!AGC_ASSERT(w >= -16384 && w <= 16383))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) w);
}

/* Dead-zone test (2D/SDZ, 3DDZ, 4D/SDZ): the AGC adds LIM - 1 to |x|
 * and tests for overflow, so the tag is nonzero for |x| >= 16385 - LIM.  */
static int dap_dz(agc_sp_t x, agc_sp_t lim)
{
    int32_t thr = 16385 - (int32_t) lim;

    if (!AGC_ASSERT(thr > 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(lim > 0))
    {
        return (0); /* no valid result */
    }

    if ((int32_t) x >= thr)
    {
        return (1);
    }

    return (((int32_t) x <= -thr) ? -1 : 0);
}

/* 2D/SDZ (p. 1076): rate tag, +1 above the 2 deg/s dead zone.           */
static int dap_dz2(agc_sp_t rate)
{
    _Static_assert(AGC_ENTRY_YDOTLIM > 0, "AGC constant");

    if (!AGC_ASSERT(rate >= -16384))
    {
        return (0); /* no valid result */
    }

    return (dap_dz(rate, AGC_ENTRY_YDOTLIM));
}

/* BIASEDZ / 3DDZ / 4D/SDZ / 2D/SDZ (p. 1076): extra-atmospheric dead
 * zone. The error is biased by -SGN(rate) CM/BIAS and tested against the
 * 3.6 deg YAWLIM; a rate above 4 deg/s overrides the error bit.          */
static int dap_biasedz(agc_sp_t rate, agc_sp_t err)
{
    int32_t e = (int32_t) err;
    int     etag;

    if (!AGC_ASSERT(rate >= -16384 && err >= -16384))
    {
        return (0); /* no valid result */
    }

    if (rate > 0)
    {
        e -= AGC_ENTRY_CM_BIAS;
    }
    else if (rate < 0)
    {
        e += AGC_ENTRY_CM_BIAS;
    }

    etag = -dap_dz(entry_sp_sat(e), AGC_ENTRY_YAWLIM); /* 3DDZ: COM   */

    if (dap_dz(rate, AGC_ENTRY_4D_SLIM) != 0)
    {
        etag = 0; /* ZL         */
    }

    /* EXTEND / ROR LCHAN of two ones' complement tags (+1, -1, 0): opposite
     * tags OR to -0, which indexes the zero (no jet) entry.                 */
    int rate_tag = dap_dz2(rate);

    if (!AGC_ASSERT(rate_tag >= -1 && rate_tag <= 1))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(etag >= -1 && etag <= 1))
    {
        return (0); /* no valid result */
    }

    if (rate_tag == 0)
    {
        return (etag);
    }

    if (etag == 0 || etag == rate_tag)
    {
        return (rate_tag);
    }

    return (0);
}

/* YJETCODE / P/RJCODE tables (p. 1080), index -1, 0, +1.                */
uint16_t agc_entry_dap_yaw_jet_bits(int jndx)
{
    static const uint16_t yjetcode[3] = {00120, 00000, 00240};

    if (!AGC_ASSERT(jndx >= -1 && jndx <= 1))
    {
        return (0);
    }

    if (!AGC_ASSERT(yjetcode[1] == 0))
    {
        return (0);
    }

    return (yjetcode[jndx + 1]);
}

uint16_t agc_entry_dap_roll_jet_bits(int jndx)
{
    static const uint16_t prjcode[3] = {00005, 00000, 00012};

    if (!AGC_ASSERT(jndx >= -1 && jndx <= 1))
    {
        return (0);
    }

    if (!AGC_ASSERT(prjcode[1] == 0))
    {
        return (0);
    }

    return (prjcode[jndx + 1]);
}

/* ---- accessors for other lanes ---------------------------------------- */
agc_sp_t agc_erase_aog(void)
{
    if (!AGC_ASSERT(aog >= -16384))
    {
        return (0); /* no valid result */
    }

    return (aog);
}

agc_sp_t agc_erase_aig(void)
{
    if (!AGC_ASSERT(aig >= -16384))
    {
        return (0); /* no valid result */
    }

    return (aig);
}

agc_sp_t agc_erase_amg(void)
{
    if (!AGC_ASSERT(amg >= -16384))
    {
        return (0); /* no valid result */
    }

    return (amg);
}

/* FLUSHJET (p. 1065): all jets off (channels 6 and 5).                   */
void agc_entry_dap_flushjet(void)
{
    agc_chan_write(AGC_CHAN_ROLLJETS, 0);
    agc_chan_write(AGC_CHAN_PYJETS, 0);
}

/* ======================================================================
 * READGYMB / BODYRATE (pp. 1063-1072)
 * ==================================================================== */

/* CM/GYMIC (p. 1064): first pass: GYMDIFSW set, no rates yet.            */
static void dap_gymic(bool set_gymdifsw)
{
    if (set_gymdifsw)
    {
        agc_flag_set(AGC_FLAG_GYMDIFSW);
    }

    jetag                     = 0;
    oldelp                    = 0;
    oldelq                    = 0;
    oldelr                    = 0;
    agc_entry_state()->gamdot = 0; /* no gimbal, no gamma diff  */

    if (!AGC_ASSERT(jetag == 0))
    {
        return;
    }

    if (!AGC_ASSERT(oldelp == 0))
    {
        return;
    }
}

/* RATEAVG (p. 1065): DELV(EST) = DELV + (DELV - OLDELV)/2.               */
static agc_sp_t dap_rateavg(agc_sp_t old, agc_sp_t now)
{
    agc_sp_t d = entry_sp_sat((int32_t) now - (int32_t) old);

    if (!AGC_ASSERT(old >= -16384))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(now >= -16384))
    {
        return (0); /* no valid result */
    }

    return (entry_sp_sat((int32_t) entry_sp_mp(d,
                                               AGC_SP_HALF) + (int32_t) now));
}

/* BODYRATE roll and the GAMDOT correction (pp. 1071-1072):
 *   P = ODOT + IDOT SM; with GAMDOT: P -= SR GAMDOT SIN(TRIM),
 *   Q -= CR GAMDOT, R += SR GAMDOT COS(TRIM).                             */
static void dap_bodyrate_p(void)
{
    agc_entry_state_t *es  = agc_entry_state();
    agc_dp_t           pr  = agc_sp_mul(agc_sp_sin(amg), m_delaig);
    agc_sp_t           rnd = (agc_dp_low(pr) >= 8192) ?
                             1 :
                             ((agc_dp_low(pr) <= -8192) ? -1 : 0);     /* DDOUBL */
    agc_sp_t jet;
    agc_sp_t msr;

    if (!AGC_ASSERT(es != NULL))
    {
        return;
    }

    jet = entry_sp_sat(
        -((int32_t) rnd + (int32_t) m_delaog + (int32_t) agc_dp_high(pr)));
    prel   = dap_rateavg(oldelp, jet);
    oldelp = jet;

    if (es->gamdot == 0)
    {
        return; /* NOGAMDUT  */
    }

    msr  = entry_sp_mp(agc_sp_sin((agc_sp_t) (-es->roll_180)), es->gamdot);
    prel = entry_sp_sat((int32_t) prel + entry_sp_mp(msr, AGC_ENTRY_SINTRIM));
    qrel = entry_sp_sat(
        (int32_t) qrel +
        entry_sp_mp((agc_sp_t) (-agc_sp_cos(es->roll_180)), es->gamdot));
    rrel = entry_sp_sat((int32_t) rrel +
                        entry_sp_mp((agc_sp_t) (-msr), AGC_ENTRY_COSTRIM));

    if (!AGC_ASSERT(prel >= -16384))
    {
        return;
    }
}

/* BODYRATE end (p. 1072): with CMDAPARM set, ATTRATES via T5RUPT soon.  */
static agc_status_t dap_bodyrate(void)
{
    /* BODYRATE pitch and yaw (pp. 1070-1071):
     *   Q = IDOT CO CM + MDOT SO,   R = -IDOT CM SO + MDOT CO.               */
    agc_sp_t  cosm = agc_sp_cos(amg);
    agc_sp_t  sino = agc_sp_sin(aog);
    agc_sp_t  coso = agc_sp_cos(aog);
    agc_dp_t  j;
    agc_sp_t  jet;
    agc_ovf_t ovf = 0;

    if (!AGC_ASSERT(amg >= -16384 && aog >= -16384))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    j = agc_sp_mul((agc_sp_t) (-m_delamg), sino);
    j = agc_dp_add_raw(
        j, agc_sp_mul((agc_sp_t) (-m_delaig), entry_sp_mp(coso, cosm)), &ovf);
    jet    = agc_dp_high(j);
    qrel   = dap_rateavg(oldelq, jet);
    oldelq = jet;
    j      = agc_sp_mul((agc_sp_t) (-m_delamg), coso);
    j      = agc_dp_add_raw(j, agc_sp_mul(m_delaig, entry_sp_mp(sino,
                                                                cosm)), &ovf);
    jet    = agc_dp_high(j);
    rrel   = dap_rateavg(oldelr, jet);
    oldelr = jet;

    if (!AGC_ASSERT(ovf >= -1 && ovf <= 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    dap_bodyrate_p();

    if (!agc_flag_test(AGC_FLAG_CMDAPARM))
    {
        return (AGC_STATUS_OK); /* STBYDUMP          */
    }

    agc_rupt_set_time(5, 037777); /* TIME5 = POSMAX    */

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_CMDAPARM)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rupt_set_t5_task(AGC_TASK_ATTRATES);

    return (AGC_STATUS_OK); /* T5LOC = ATTRATES  */
}

/* READGYMB / READGYM1 / DOBRATE? (pp. 1063-1064): .1 s task.             */
agc_status_t agc_task_readgymb(void)
{
    unsigned sel;

    agc_erase_set_cm_gymdt(
        entry_sp_sat((int32_t) agc_erase_cm_gymdt() + 10)); /* ADS CM/GYMDT */

    if ((agc_erase_imodes33() & 040u) != 0u)
    { /* not fine align */
        agc_flag_clear(AGC_FLAG_GYMDIFSW);
        agc_entry_dap_flushjet();
        dap_gymic(false);

        return (agc_waitlist_call(10, AGC_TASK_READGYMB)); /* DOBRATE1, JETEM 0 */
    }

    /* XCH AOG / MSU AOG: modular (2s complement) difference, so a gimbal
     * passing 180 deg gives the small step, not an overflow.            */
    m_delaog = entry_angle_wrap((int32_t) aog - (int32_t) cdu_sp(agc_erase_cdux()));
    aog      = cdu_sp(agc_erase_cdux());
    m_delaig = entry_angle_wrap((int32_t) aig - (int32_t) cdu_sp(agc_erase_cduy()));
    aig      = cdu_sp(agc_erase_cduy());
    m_delamg = entry_angle_wrap((int32_t) amg - (int32_t) cdu_sp(agc_erase_cduz()));
    amg      = cdu_sp(agc_erase_cduz());
    sel      = (agc_flag_test(AGC_FLAG_CM_DSTBY) ? 0u : 2u) |
               (agc_flag_test(AGC_FLAG_GYMDIFSW) ? 0u : 1u);

    if (!AGC_ASSERT(sel <= 3u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (sel >= 2u)
    { /* CM/DSTBY off */
        agc_entry_dap_flushjet();
        agc_phase_change(6, 000006);

        return (AGC_STATUS_OK); /* group 6 off  */
    }

    if (sel == 1u)
    {
        dap_gymic(true); /* first pass   */

        return (agc_waitlist_call(10, AGC_TASK_READGYMB));
    }

    agc_status_t st = agc_waitlist_call(10, AGC_TASK_READGYMB); /* DOBRATE      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (dap_bodyrate());
}

/* ======================================================================
 * ATTRATES and the pitch/yaw DAPs (pp. 1073-1079)
 * ==================================================================== */

/* EXDAP1 (p. 1078): yaw axis with the error in RAXERR (sign set by the
 * caller), rate RREL or (mode +0) BETADOT; pitch on QAXERR, QREL.        */
static uint16_t dap_exdap1(agc_entry_state_t *es, agc_sp_t yaw_err)
{
    agc_sp_t rate;
    uint16_t bits;

    if (!AGC_ASSERT(es != NULL))
    {
        return (0); /* no valid result */
    }

    raxerr = yaw_err;
    ak2    = yaw_err;
    rate   = (es->cmdapmod == AGC_ENTRY_DAPMOD_PLUS0) ? betadot : rrel;
    bits   = agc_entry_dap_yaw_jet_bits(dap_biasedz(rate, yaw_err));
    bits   = (uint16_t) (bits +
                         agc_entry_dap_roll_jet_bits(dap_biasedz(qrel,
                                                                 qaxerr)));

    if (!AGC_ASSERT(bits <= 0377u))
    {
        return (0); /* no valid result */
    }

    return (bits);
}

/* EXDAP2 (pp. 1078-1079): |cos alfa| <= .707, CMDAPMOD = +1. Beta is
 * controlled with the roll jets, roll held with the yaw jets.            */
static uint16_t dap_exdap2(agc_entry_state_t *es)
{
    agc_sp_t err;
    agc_sp_t rate;
    int      tag;

    es->cmdapmod = AGC_ENTRY_DAPMOD_PLUS1;
    jetag        = -1; /* roll jets now from here      */

    if (es->wake_separation_interlock > 0)
    {
        es->wake_separation_interlock = 0;
    }

    err =
        (es->salfa > 0) ? (agc_sp_t) (-raxerr) : ((es->salfa < 0) ? raxerr : 0);
    paxerr1 = entry_sp_mp(err, AGC_SP_HALF); /* error/360   */
    rate    = (es->salfa > 0) ? (agc_sp_t) (-betadot) :
              ((es->salfa < 0) ? betadot : 0);
    tag = dap_biasedz(rate, err);
    agc_chan_write(AGC_CHAN_ROLLJETS, agc_entry_dap_roll_jet_bits(tag));
    err = entry_angle_wrap((int32_t) es->rollhold -
                           (int32_t) es->roll_180); /* MSU ROLL/180  */
    err = (es->salfa > 0) ? err : ((es->salfa < 0) ? (agc_sp_t) (-err) : 0);

    if (!AGC_ASSERT(es->cmdapmod == AGC_ENTRY_DAPMOD_PLUS1))
    {
        return (0); /* no valid result */
    }

    return (dap_exdap1(es, err));
}

/* EXDAP / EXDAP4 / EXDAP3 (pp. 1077-1078): extra-atmospheric DAP. With
 * |cos alfa| > .707 the first pass with P63FLAG = +0 calls WAKEP62 in
 * NSEC (65 deg at 3 deg/s).                                              */
static uint16_t dap_exdap(agc_entry_state_t *es)
{
    es->cmdapmod = AGC_ENTRY_DAPMOD_PLUS0;

    if (dap_dz(es->calfa, AGC_ENTRY_C45LIM) == 0)
    {
        return (dap_exdap2(es));
    }

    if (es->calfa > 0 && es->wake_separation_interlock == 0)
    {
        agc_phase_change(4, 040334); /* 4.33 WAKEP62 */
        es->wake_separation_interlock = -1; /* single pass  */

        if (agc_waitlist_call(AGC_ENTRY_NSEC, AGC_TASK_WAKE_CM_SM_SEPARATION) !=
            AGC_STATUS_OK)
        {
            /* WAITLIST overflow: BAILOUT 1203 is recorded and the restart
             * pending; command no pitch/yaw jets on this pass.          */
            return (0);
        }
    }

    if (jetag < 0)
    { /* EXDAP4       */
        agc_chan_write(AGC_CHAN_ROLLJETS, 0);
        jetag = 0;
    }

    if (es->calfa > 0)
    { /* EXDAP3       */
        return (dap_exdap1(es, raxerr));
    }

    es->cmdapmod = AGC_ENTRY_DAPMOD_MINUS0; /* damping only */

    if (!AGC_ASSERT(es->calfa <= 0))
    {
        return (0); /* no valid result */
    }

    return (dap_exdap1(es, (agc_sp_t) (-raxerr)));
}

/* CM/FDAI, CM/FDAIR, CM/DUMPR, CMTMFILE, CM/END (pp. 1090-1091): roll
 * error integration between 2 s updates, the needles on alternate passes
 * and the body-rate TM file (UPBUFF) on the others.                      */
static agc_status_t dap_fdai(bool integrate)
{
    int idx;

    if (integrate)
    { /* CM/FDAI       */
        agc_sp_t d = entry_sp_mp(
            entry_sp_mp((agc_sp_t) (-phidot), agc_entry_state()->calfa),
            AGC_SP_HALF);

        paxerr1 = entry_sp_store_ovf((int32_t) paxerr1 + d); /* OVFL OK */
    }

    ak     = entry_sp_mp(AGC_SP_HALF, paxerr1); /* PAXERR        */
    sw_ndx = (agc_sp_t) (-sw_ndx);              /* CM/DUMPR      */

    if (sw_ndx > 0)
    {
        bool init = needler_init;

        needler_init = false;
        agc_rupt_set_t5_task(AGC_TASK_T5IDLOC); /* CM/END        */

        return (agc_dap_needler(ak, ak1, ak2, init));
    }

    if (sw_ndx + 3 > 0)
    { /* CMTMFILE      */
        cmtmtime = agc_erase_time1();
        sw_ndx   = -13;
    }
    else
    {
        sw_ndx = (agc_sp_t) (sw_ndx + 3);
    }

    idx = (13 + (int) sw_ndx) / 3; /* -13..-1 -> 0..4 */

    if (AGC_ASSERT(idx >= 0 && idx < 5))
    { /* SAVENDX        */
        endbuf[3 * idx]     = prel;
        endbuf[3 * idx + 1] = qrel;
        endbuf[3 * idx + 2] = rrel;
    }

    agc_rupt_set_t5_task(AGC_TASK_T5IDLOC);

    return (AGC_STATUS_OK); /* CM/END        */
}

/* ======================================================================
 * Predictive roll system CM/RCS (pp. 1081-1089)
 * ==================================================================== */

/* SETJTAG (p. 1081): 2 s task 1.2 s after PIPA time: roll update due.   */
agc_status_t agc_task_setjtag(void)
{
    tused = agc_erase_time1(); /* CS TIME1 / TS TUSED (sign kept
                                           positive here, see TIMETST)   */
    jetag = 1;

    if (!AGC_ASSERT(tused >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(jetag == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(1, 000001);

    return (AGC_STATUS_OK);
}

/* GETON2 / GETOFF / TOFFOVFL / GETOFF2 / TIMSCAL (pp. 1085-1086).       */
static void pp_getoff(agc_sp_t lcx, agc_sp_t mvt, agc_sp_t vdt,
                      agc_entry_roll_cmd_t *o)
{
    int32_t num;

    if (!AGC_ASSERT(o != NULL))
    {
        return;
    }

    o->ton2 =
        entry_sp_sat(2 * (int32_t) entry_sp_mp(entry_sp_sat(2 * (int32_t) vdt),
                                               AGC_ENTRY_180_8ATT));

    if (o->ton2 < 0)
    {
        o->ton2  = (agc_sp_t) (-o->ton2);
        o->jndx1 = o->jndx;
    }

    if (vdt == 0)
    {
        o->toff = AGC_ENTRY_2JETT; /* TOFFOVFL  */
    }
    else
    {
        num = (int32_t) entry_sp_mp((agc_sp_t) (-o->ton2), vdt) +
              entry_sp_mp(entry_sp_sat((int32_t) mvt - vdt), o->ton1) + lcx;
        num = entry_sp_sat(num);

        if ((num < 0 ? -num : num) >=
            (vdt < 0 ? -(int32_t) vdt : (int32_t) vdt))
        {
            o->toff = AGC_ENTRY_2JETT; /* DV ovfl   */
        }
        else
        {
            o->toff =
                entry_sp_mp((agc_sp_t) ((num * 16384) / vdt), AGC_ENTRY_2JETT);
        }
    }

    o->ton1 = entry_sp_mp(AGC_ENTRY_4JETT, o->ton1); /* cs        */
    o->ton2 = entry_sp_mp(AGC_ENTRY_4JETT, o->ton2);

    if (!AGC_ASSERT(o->ton2 >= 0))
    {
        return;
    }
}

/* MAXVTEST / MAXVTIM1 / GETON1 / OVRLINE (pp. 1084-1085): first burn to
 * the drift velocity VDT (or the limit VM), and its duration TON1.        */
static void pp_maxvtest(agc_sp_t lcx, agc_sp_t mvt, agc_sp_t vsq,
                        agc_entry_roll_cmd_t *o, agc_sp_t *vdt)
{
    agc_sp_t xd;
    bool     qpos = true; /* Q tag from TC GETON1   */

    if (!AGC_ASSERT(o != NULL && vdt != NULL))
    {
        return;
    }

    o->jndx1 = -o->jndx;
    xd       = entry_sp_sat((int32_t) vsq + lcx - AGC_ENTRY_XS_360); /* XD - XS */

    if ((int32_t) xd + AGC_ENTRY_MVM_360K >= 0)
    {
        qpos = false;                            /* MAXVTIM1: ZQ    */
        *vdt = (agc_sp_t) (-AGC_ENTRY_MVM_360K); /* VMT/180         */
    }
    else
    {
        agc_dp_t pr = agc_sp_mul(xd, AGC_ENTRY_KTRCS);
        int32_t  rd =
            (agc_dp_low(pr) >= 8192) ? 1 : ((agc_dp_low(pr) <= -8192) ? -1 : 0);

        *vdt = entry_sp_sat(2 * (int32_t) agc_dp_high(pr) + rd); /* DDOUBL */
    }

    o->ton1 = entry_sp_mp(entry_sp_sat(2 * ((int32_t) *vdt + mvt)),
                          AGC_ENTRY_180_8ATT);

    if (o->ton1 > 0)
    {
        return; /* GETON2          */
    }

    if (qpos)
    { /* OVRLINE1        */
        *vdt    = (agc_sp_t) (-mvt);
        o->ton1 = 0;

        return;
    }

    o->jndx = o->jndx1; /* MAXVTIM2        */
    o->ton1 = (agc_sp_t) (-o->ton1);
}

/* Phase plane of CM/RCS from WHICHALF on (pp. 1083-1086). lcx: X/360;
 * mvt2: -2 V T/180 as first stored in -VT/180. *vdt gets VDT/180.       */
static void dap_phase_plane(agc_sp_t lcx_in, agc_sp_t mvt2,
                            agc_entry_roll_cmd_t *o, agc_sp_t *vdt)
{
    agc_sp_t mvt = (agc_sp_t) (mvt2 / 2); /* TS SR; XCH -VT/180 */
    /* CA SR rewrites SR and so shifts it again (the editing registers
     * edit on every write): SR = -VT/360 for the DZ2 test.             */
    agc_sp_t sr  = (agc_sp_t) (mvt / 2);
    agc_sp_t lcx = lcx_in;
    agc_sp_t vsq = entry_sp_mp(entry_sp_mp(mvt2, mvt), AGC_ENTRY_180_8ATT);
    int32_t  l   = (int32_t) AGC_ENTRY_XS_360 - 2 * (int32_t) vsq; /* BUFLIM */
    int32_t  t   =
        ((mvt > 0) ? -l : ((mvt < 0) ? l : 0)) + lcx + AGC_ENTRY_XS_360;
    bool dz = false;

    o->jndx = -1;

    if (t <= 0)
    { /* REFLECT          */
        mvt     = (agc_sp_t) (-mvt);
        sr      = (agc_sp_t) (mvt / 2);
        lcx     = (agc_sp_t) (-lcx);
        o->jndx = 1;
    }
    else if (2 * (int32_t) AGC_ENTRY_XS_360 - t > 0)
    { /* buffer zone */
        if (vsq - AGC_ENTRY_VSQMIN <= 0)
        {
            o->in_deadzone = true;

            return;
        }

        o->jndx1 = -o->jndx;
        *vdt     = (agc_sp_t) (-mvt); /* OVRLINE1         */
        o->ton1  = 0;
        pp_getoff(lcx, mvt, *vdt, o);

        return;
    }

    if (vsq - AGC_ENTRY_VSQMIN <= 0)
    { /* DZ1 -> DZ2       */
        dz = ((int32_t) lcx - AGC_ENTRY_XMIN_360 + sr <= 0);
    }

    o->in_deadzone = dz;

    if (!AGC_ASSERT(o->jndx == 1 || o->jndx == -1))
    {
        return;
    }

    if (!dz)
    {
        pp_maxvtest(lcx, mvt, vsq, o, vdt);
        pp_getoff(lcx, mvt, *vdt, o);
    }
}

/* agc_entry_dap_roll_phase_plane -- pure phase-plane solution.           */
agc_status_t agc_entry_dap_roll_phase_plane(agc_sp_t lcx_360, agc_sp_t mvt_180,
                                            agc_entry_roll_cmd_t *out)
{
    agc_sp_t vdt = 0;

    if (!AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(lcx_360 >= -16384 && mvt_180 >= -16384))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    out->ton1        = 0;
    out->toff        = 0;
    out->ton2        = 0;
    out->jndx        = -1;
    out->jndx1       = 1;
    out->in_deadzone = false;
    dap_phase_plane(lcx_360, mvt_180, out, &vdt);

    return (AGC_STATUS_OK);
}

/* JETCALL1 .. JETACTN (pp. 1088-1089): walk the interval list from
 * `start`; a positive interval gets a WAITLIST call to JETCALL and its
 * bits fire now, +0 means its bits apply for the rest of T.               */
static agc_status_t dap_jetlist(int start)
{
    int k;

    if (!AGC_ASSERT(start >= 0 && start < DAP_JETLIST))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    nujet = 0;

    for (k = start; k < DAP_JETLIST; k++)
    {
        agc_sp_t v = jet_time[k];

        jet_time[k] = (k == 0) ? 0 : -1; /* DXCH: processed  */

        if (v > 0)
        {
            nujet = (uint16_t) (nujet + jet_bits[k]); /* ADS NUJET        */
            agc_status_t st = agc_waitlist_call((uint16_t) v, AGC_TASK_JETCALL);

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }

            break;
        }

        if (v == 0)
        {
            nujet = jet_bits[k]; /* LXCH NUJET       */
        }
    }

    if (!AGC_ASSERT(nujet <= 0377u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_chan_write(AGC_CHAN_ROLLJETS, nujet);

    return (AGC_STATUS_OK); /* JETACTN          */
}

/* TIMETST (pp. 1087-1088): keep only intervals > 2 cs that start within
 * the time left in the 2 s period; the one that runs past T is set to +0. */
static agc_status_t dap_timetst(const agc_entry_roll_cmd_t *o)
{
    const agc_sp_t times[DAP_JETLIST] = {o->ton1, o->toff, o->ton2};
    const int      ndx[DAP_JETLIST]   = {o->jndx, 0, o->jndx1};
    int32_t        used               =
        (int32_t) (((uint32_t) agc_erase_time1() - (uint32_t) tused) &
                   037777u) +
        AGC_ENTRY_T_M3;
    int  k;
    bool past = false;

    if (!AGC_ASSERT(used < 16384))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (k = 0; k < DAP_JETLIST; k++)
    {
        jet_bits[k] = (k == 1) ? 0u : agc_entry_dap_roll_jet_bits(ndx[k]);

        if (past || times[k] - 2 <= 0)
        {
            jet_time[k] = -1; /* TIMETST1/2/5     */
            continue;
        }

        used       += times[k];
        jet_time[k] = times[k];

        if (used > 0)
        { /* runs past T      */
            jet_time[k] = 0;
            past        = true;
        }
    }

    return (dap_jetlist(0));
}

/* JETCALL (p. 1089): waitlist entry; quench the roll jets if the DAP is
 * on standby, else continue with the next interval.                      */
agc_status_t agc_task_jetcall(void)
{
    if (!agc_flag_test(AGC_FLAG_CM_DSTBY))
    {
        agc_chan_write(AGC_CHAN_ROLLJETS, 0);

        return (AGC_STATUS_OK); /* JETACTN +1       */
    }

    if (!AGC_ASSERT(jet_time[0] <= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_CM_DSTBY)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dap_jetlist(1)); /* JETCALL3         */
}

/* DZCALL (p. 1089): in the dead zone: ROLLHOLD = ROLL (mode -0) or the
 * roll command, roll jets off.                                           */
static void rcs_dzcall(agc_entry_state_t *es)
{
    if (!AGC_ASSERT(es != NULL))
    {
        return;
    }

    if (es->cmdapmod == AGC_ENTRY_DAPMOD_MINUS0)
    {
        es->rollhold = es->rolltm;
    }
    else
    {
        es->rollhold = entry_angle_wrap(2 * (int32_t) agc_dp_high(es->rollc));
    }

    agc_chan_write(AGC_CHAN_ROLLJETS, 0);
    vdt_180 = 0;

    if (!AGC_ASSERT(vdt_180 == 0))
    {
        return;
    }
}

/* CM/RCS .. COMPAT (pp. 1081-1083): roll error X and rate for the phase
 * plane; then TIMETST/JETCALL or DZCALL; exit through CM/FDAIR.          */
static agc_status_t dap_cm_rcs(agc_entry_state_t *es)
{
    agc_entry_roll_cmd_t o     = {0, 0, 0, -1, 1, false};
    int32_t              v     = -40 * (int32_t) prel;                /* -2T/TCDU P */
    agc_sp_t             mvt2  = entry_sp_sat(v / 16384 + v % 16384); /* MP, AD L   */
    agc_sp_t             sr    = (agc_sp_t) (-(int32_t) es->roll_180 / 2); /* TS SR      */
    agc_sp_t             rollc = agc_dp_high(es->rollc);
    agc_sp_t             lcx;

    if (!agc_flag_test(AGC_FLAG_LATSW))
    {
        agc_flag_set(AGC_FLAG_LATSW); /* over the top */
        lcx = entry_sp_sat((int32_t) rollc + sr);
    }
    else
    {
        /* GETLCX / TRTAGXPI (p. 1082): X/360 = RCOM/360 - R/360 in (-1, 1), then
         * the shortest angular path for a single-jet stop from rate -VT/180.     */
        agc_sp_t calfa = es->calfa;

        int32_t  rcom       = (rollc >= 0) ? rollc : (int32_t) rollc + 16384;
        int32_t  rneg       = (sr > 0) ? (int32_t) sr - 16384 : sr;
        int32_t  lcx_getlcx = rcom + rneg;
        agc_dp_t p          =
            agc_sp_mul(entry_sp_sat(mvt2 < 0 ? -(int32_t) mvt2 : mvt2), mvt2);
        agc_dp_t p2 = agc_sp_mul(agc_dp_high(p), AGC_ENTRY_180_8ATT); /* 1/16A1 */
        int32_t  q;
        int32_t  t;

        if (!AGC_ASSERT(calfa != 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        q = (int32_t) (((int64_t) agc_dp_high(p2) * 16384 + agc_dp_low(p2)) /
                       calfa);
        q = entry_sp_sat(q); /* DV CALFA    */
        t =
            ((lcx_getlcx >
              0) ? 8192 : ((lcx_getlcx < 0) ? -8192 : 0)) + lcx_getlcx + q;

        if (t >= 16384)
        {
            lcx_getlcx -= 16384; /* shift -2 pi */
        }
        else if (t <= -16384)
        {
            lcx_getlcx += 16384;
        }

        if (!AGC_ASSERT(lcx_getlcx > -32768 && lcx_getlcx < 32768))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        lcx = entry_sp_sat(lcx_getlcx);
    }

    lcx = entry_sp_mp(lcx, es->calfa); /* COMPAT     */

    if (es->cmdapmod == AGC_ENTRY_DAPMOD_PLUS1)
    {
        jetag = 0; /* DZCALL1    */

        return (dap_fdai(false));
    }

    if (es->cmdapmod == AGC_ENTRY_DAPMOD_MINUS0)
    {
        lcx = 0; /* damping    */
    }

    errorz   = lcx;
    paxerr1  = lcx;
    mvt_180e = (agc_sp_t) (mvt2 / 2);
    dap_phase_plane(lcx, mvt2, &o, &vdt_180);
    jetag = 0;

    if (o.in_deadzone)
    {
        rcs_dzcall(es);

        return (dap_fdai(false));
    }

    sw_ndx = 1;
    agc_status_t st = dap_timetst(&o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(jetag == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dap_fdai(false)); /* ROLLDUMP   */
}

/* ATTRATES (pp. 1073-1075): T5RUPT continuation of BODYRATE.             */
agc_status_t agc_task_attrates(void)
{
    agc_entry_state_t *es = agc_entry_state();
    uint16_t           bits;

    if (!AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* ATTRATES (p. 1073-1074): integrate ALFA, BETA, ROLL:
     *   CB PHIDOT = CA P + SA R, BETADOT = -SA P + CA R,
     *   ALFADOT = Q + SB PHIDOT.                                              */
    agc_sp_t ca;
    agc_sp_t sa;

    es->alfa_180 = entry_angle_wrap((int32_t) qrel + (int32_t) es->alfa_180);
    ca           = agc_sp_cos(es->alfa_180);
    sa           = agc_sp_sin(es->alfa_180);
    es->calfa    = ca;
    es->salfa    = sa;
    phidot       =
        entry_sp_sat((int32_t) entry_sp_mp(ca, prel) + entry_sp_mp(sa, rrel));
    betadot = entry_sp_sat((int32_t) entry_sp_mp(ca, rrel) +
                           entry_sp_mp((agc_sp_t) (-sa), prel));
    es->beta_180 = entry_sp_sat((int32_t) es->beta_180 + betadot);
    es->alfa_180 = entry_angle_wrap(
        (int32_t) entry_sp_mp(agc_sp_sin(es->beta_180), phidot) + es->alfa_180);
    ak1 = entry_angle_wrap(
        (int32_t) es->alfacom - (int32_t) es->alfa_180);
    qaxerr       = ak1;
    es->roll_180 = entry_angle_wrap((int32_t) phidot + es->roll_180);
    es->rolltm   = es->roll_180;
    raxerr       = entry_sp_sat((int32_t) es->betacom - (int32_t) es->beta_180);

    if (!AGC_ASSERT(es->calfa >= -16384))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_flag_test(AGC_FLAG__05GSW))
    {
        /* Atmospheric DAP (p. 1074): rate damping, yaw coordinated with roll:
         * yaw rate error = R - P TAN(ALFA) ~ R - P SIN(-20 deg).                 */
        agc_sp_t yerr;
        uint16_t bits_yaw;

        es->cmdapmod = AGC_ENTRY_DAPMOD_MINUS1;
        ak1          = -1; /* zero pitch, yaw needles    */
        ak2          = -1;
        yerr         = entry_sp_sat(
            (int32_t) entry_sp_mp((agc_sp_t) (-prel),
                                  AGC_ENTRY_SINTRIM) + rrel);
        bits_yaw = agc_entry_dap_yaw_jet_bits(dap_dz2(yerr));
        bits_yaw =
            (uint16_t) (bits_yaw + agc_entry_dap_roll_jet_bits(dap_dz2(qrel)));

        if (!AGC_ASSERT(bits_yaw <= 0377u))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        bits = bits_yaw;
    }
    else
    {
        bits = dap_exdap(es);
    }

    agc_chan_write(AGC_CHAN_PYJETS, bits); /* EXDAPIN */

    if (!AGC_ASSERT(jetag >= -1 && jetag <= 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (jetag > 0)
    {
        return (dap_cm_rcs(es));
    }

    return (dap_fdai(jetag == 0)); /* -1: CM/FDAIR -1 (no add) */
}

/* ======================================================================
 * Initialisation: CM/DAPON, CM/DAPIC (pp. 1066-1069)
 * ==================================================================== */

/* CM/DAPON +4 .. (p. 1066-1067): once CM/POSE has formed one GAMA
 * difference, activate the entry DAP in attitude hold, arm it, set
 * NODOFLAG, and continue at P62.1.                                        */
agc_status_t agc_job_cm_dapon_wait(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!agc_flag_test(AGC_FLAG_GAMDIFSW))
    {                                                       /* NOTYET     */
        return (agc_svc_delayjob(50, AGC_JOB_CM_DAPON_WAIT)); /* .5SEC     */
    }

    needler_init                  = true; /* CS ONE / TS RCSFLAGS (BIT3) */
    es->wake_separation_interlock = -1;   /* no WAKEP62 before HEADSUP    */
    jetag                         = 0;
    paxerr1                       = 0; /* needles zero until update    */
    agc_inhint();
    es->alfacom  = es->alfa_180; /* attitude hold until P62.1    */
    es->betacom  = es->beta_180;
    es->rollhold = es->roll_180;
    es->rollc    = agc_dp_pack(entry_sp_mp(es->roll_180, AGC_SP_HALF),
                               agc_dp_low(es->rollc)); /* ROLL/360  */
    agc_flag_set(AGC_FLAG_CMDAPARM);
    agc_flag_set(AGC_FLAG_NODOFLAG); /* no V37 now */
    agc_relint();

    return (agc_job_cm_sm_separation_target_display(ctx)); /* POSTJUMP   */
}

/* CM/DAPON (p. 1066): RCS DAP off (DAPBIT1/2, T5LOC/T6LOC idle), jets
 * flushed, DAPDATR1 configuration bits 13-15 cleared.                     */
agc_status_t agc_job_cm_dapon(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_DAPBIT1);
    agc_flag_clear(AGC_FLAG_DAPBIT2);
    agc_rupt_set_t5_task(AGC_TASK_T5IDLOC); /* T5IDLER1   */
    agc_rupt_set_t6_task(AGC_TASK_T5IDLOC);
    agc_entry_dap_flushjet();
    agc_erase_set_dapdatr1((uint16_t) (agc_erase_dapdatr1() & ~070000u));

    if (!AGC_ASSERT((agc_erase_dapdatr1() & 070000u) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_cm_dapon_wait(ctx));
}

/* CM/DAP2C (p. 1068): time from PIPTIME to the next READGYMB slot, 5 cs
 * after a multiple of .1 s from PIPTIME; never 0.                         */
static agc_sp_t dapic_first_dt(void)
{
    int32_t delta = (int32_t) (((uint32_t) agc_erase_time1() -
                                (uint32_t) agc_dp_low(agc_erase_piptime())) &
                               037777u);
    int32_t x = delta - 5;
    int     n;

    for (n = 0; n < 1700 && x > 0; n++)
    { /* AD -CDUT+1, TCF -2    */
        x -= 10;
    }

    if (!AGC_ASSERT(x <= 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(x > -10))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) ((x == 0) ? 1 : -x)); /* AD ONE: no zero dt    */
}

/* CM/DAPIC (pp. 1068-1069): start READGYMB in phase with PIPTIME, set
 * CM/DSTBY and LATSW (clear GYMDIFSW .. ENTRYDSP), restart group 6 and
 * group 4 at P62.2, continue at P62.2.                                    */
agc_status_t agc_entry_dap_ic(agc_job_ctx_t *ctx)
{
    static const agc_flag_id_t clr[] = {
        /* CM/SWIC1 = 016017  */
        AGC_FLAG_GYMDIFSW, AGC_FLAG_CM_DSTBY, AGC_FLAG__05GSW, AGC_FLAG_LATSW,
        AGC_FLAG_GAMDIFSW, AGC_FLAG_CMDAPARM, AGC_FLAG_ENTRYDSP
    };
    agc_phase_var_t            v =
    {AGC_RESTART_JOB, AGC_JOB_CM_SM_SEPARATION_REQUEST,
     AGC_TASK_NONE, (int16_t) AGC_ENTRY_PRIO13,
     false, 0U};
    unsigned i;

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_erase_set_cm_gymdt(dapic_first_dt());
    agc_status_t st =
        agc_waitlist_call((uint16_t) agc_erase_cm_gymdt(), AGC_TASK_READGYMB);

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    for (i = 0; i < (sizeof clr / sizeof clr[0]); i++)
    {
        agc_flag_clear(clr[i]);
    }

    agc_flag_set(AGC_FLAG_CM_DSTBY); /* CM/SWIC2 = 012     */
    agc_flag_set(AGC_FLAG_LATSW);
    agc_entry_state()->beta_180 = 0; /* no ovfl correction */
    sw_ndx                      = 1;
    agc_relint();
    agc_phase_change2(040116, 005024, &v); /* 6.11, 4.2 P62.2    */

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_CM_DSTBY)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_cm_sm_separation_request(ctx)); /* POSTJUMP P62.2     */
}
