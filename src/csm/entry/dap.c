/**********************************************************************
 * FILE NAME: dap.c
 *
 * PURPOSE:
 *   The CM entry digital autopilot. READGYMB (.1 s waitlist task): gimbal
 *   angle differences, BODYRATE (P, Q, R x TCDU/180), then via T5RUPT
 *   ATTRATES: ALFA, BETA, ROLL integration and the atmospheric (rate
 *   damping) or extra-atmospheric (attitude hold) pitch/yaw jets on
 *   channel 5. Every 2 s (SETJTAG from READACCS) the predictive roll
 *   system CM/RCS: the phase-plane solution TON1/TOFF/TON2 executed by
 *   JETCALL waitlist tasks on channel 6. CM/FDAI: roll (and pitch/yaw)
 *   attitude error needles via NEEDLER, and the body-rate TM file.
 *   CM/DAPON and CM/DAPIC start it.
 *
 * AGC SOURCE:
 *   Comanche055 CM_ENTRY_DIGITAL_AUTOPILOT.agc pp. 1063-1092
 *   Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   CDUX..CDUZ    uint16_t   I    CDU pattern    gimbal angles
 *   TIME1, PIPTIME           I    cs             timing of READGYMB
 *   CM/GYMDT      agc_sp_t   IO   cs             READGYMB restart delay
 *   DAPDATR1      uint16_t   IO   bits           configuration (CM/DAPON)
 *   channels 5, 6            O    bits           pitch/yaw, roll jets
 *   CM/FLAGS      flags      IO   -              CM/DSTBY GYMDIFSW LATSW ...
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_waitlist_call     READGYMB, JETCALL, WAKEP62 tasks
 *   agc_rupt_set_t5_task  T5RUPT: ATTRATES, T5IDLOC
 *   agc_dap_needler       NEEDLER (FDAI needles)
 *   agc_sp_sin / agc_sp_cos  SPSIN / SPCOS
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none of its own; a WAITLIST overflow (BAILOUT 1203) is recorded by
 *   the executive and the pass commands no jets.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only; tasks and T5RUPT run with interrupts inhibited. All angles
 *   are SP 2s complement /180 (angle/180 deg x 2^14) and rates are
 *   x TCDU/180 (TCDU = .1 s), as in the AGC.
 *
 * NOTES:
 *   The AGC arithmetic is ones' complement basic code; it is reproduced
 *   with int32 sums and the MP / DV high-word semantics. DEVIATION: ones'
 *   complement edge cases (-0 tags, one-LSB differences of overflow
 *   correction) are modelled in 2s complement (docs/modules/guid-entry.md).
 *   Modular MSU (READGYM1, EXDAP2), "OVFL OK" stores (CM/FDAI) and the SR
 *   editing register (phase plane: SR = -VT/360) follow the listing
 *   (review-guid-entry). DEVIATION: TUSED keeps +TIME1 (see SETJTAG).
 **********************************************************************/
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
static agc_sp_t aog;      /* AOG outer gimbal /180, B0       */
static agc_sp_t aig;      /* AIG inner gimbal /180, B0       */
static agc_sp_t amg;      /* AMG middle gimbal /180, B0      */
static agc_sp_t m_delaog; /* -DELAOG 1s comp difference, B0  */
static agc_sp_t m_delaig; /* -DELAIG, B0                     */
static agc_sp_t m_delamg; /* -DELAMG, B0                     */
static agc_sp_t oldelp;   /* previous P increment, B0        */
static agc_sp_t oldelq;   /* previous Q increment, B0        */
static agc_sp_t oldelr;   /* previous R increment, B0        */
static agc_sp_t prel;     /* P x TCDU/180, B0                */
static agc_sp_t qrel;     /* Q x TCDU/180, B0                */
static agc_sp_t rrel;     /* R x TCDU/180, B0                */
static agc_sp_t betadot;  /* BETADOT x TCDU/180, B0          */
static agc_sp_t phidot;   /* PHIDOT x TCDU/180, B0           */
static agc_sp_t jetag;    /* JETAG +1 roll due, 0, -1, unscaled */
static agc_sp_t paxerr1;  /* PAXERR1 roll error/360, B0      */
static agc_sp_t ak;       /* AK (= PAXERR) needle, B0        */
static agc_sp_t ak1;      /* AK1 needle, B0                  */
static agc_sp_t ak2;      /* AK2 needle, B0                  */
static agc_sp_t qaxerr;   /* QAXERR error /180, B0           */
static agc_sp_t raxerr;   /* RAXERR error /180, B0           */
static agc_sp_t tused;    /* TUSED +TIME1 at SETJTAG, cs B14 */
static agc_sp_t errorz;   /* ERRORZ roll error (TM), B0      */
static agc_sp_t mvt_180e; /* -VT/180E (edit), B0             */
static agc_sp_t vdt_180;       /* VDT/180 drift velocity, B0      */
static bool     needler_init;  /* RCSFLAGS BIT3: init NEEDLER   */
static agc_sp_t sw_ndx;        /* SW/NDX TM index, unscaled     */
static agc_sp_t cmtmtime;      /* CMTMTIME (UPBUFF), cs B14     */
static agc_sp_t endbuf[3 * 5]; /* UPBUFF +2..+16: P Q R x 5     */

/* JETCALL list: TON1/T1BITS, TOFF/TBITS, TON2/T2BITS (cs; >0 call
 * WAITLIST, +0 bits apply for the rest of T, <0 processed).               */
#define DAP_JETLIST 3 /* three intervals per 2 s update */
static agc_sp_t jet_time[DAP_JETLIST]; /* JETCALL intervals, cs B14 */
static uint16_t jet_bits[DAP_JETLIST]; /* JETCALL jet bits */
static uint16_t nujet; /* NUJET roll jet bits to write  */

/* ---- SP helpers ----------------------------------------------------- */

/**********************************************************************
 * ANGOVCOR / CORANGOV (CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1065,
 * CM_BODY_ATTITUDE.agc p. 888): wrap an angle/180 sum (at most one
 * overflow) back into [-1, 1), i.e. modulo 360 deg.
 *
 * FUNCTION NAME: entry_angle_wrap
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  ---------------------------------------------
 *   sum       int32_t  I    the sum of angles /180, SP units (|sum| < 3 x
 *                           16384)
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- the angle /180, B0
 **********************************************************************/
agc_sp_t entry_angle_wrap(int32_t sum)
{
    int32_t s = sum; /* working sum, SP units */

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

/**********************************************************************
 * AGC: none (TS / ADS / XCH of an overflowed A) -- memory gets SG and bits
 * 1-14, so a sum past +-1.0 comes back by 1.0 (the ones' complement
 * end-around LSB is not modelled).
 *
 * FUNCTION NAME: entry_sp_store_ovf
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  ---------------------------------------------
 *   sum       int32_t  I    the overflowed sum, SP units (|sum| < 2 x
 *                           16384)
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- the stored word, B0
 **********************************************************************/
agc_sp_t entry_sp_store_ovf(int32_t sum)
{
    int32_t s = sum; /* working sum, SP units */

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

    if (!AGC_ASSERT(s > -16384 && s < 16384))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) s);
}

/**********************************************************************
 * AGC: none (AD whose result the AGC keeps without an overflow test) --
 * saturate to an SP word.
 *
 * FUNCTION NAME: entry_sp_sat
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  ---------------------------------------------
 *   sum       int32_t  I    the sum, SP units
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- the sum limited to [-16383, 16383], B0
 **********************************************************************/
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

/**********************************************************************
 * AGC: none (EXTEND MP) -- high word of the SP x SP product (truncated
 * toward zero).
 *
 * FUNCTION NAME: entry_sp_mp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   a         agc_sp_t  I    multiplicand, B0
 *   b         agc_sp_t  I    multiplier, B0
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- the high word of a x b, B0
 **********************************************************************/
agc_sp_t entry_sp_mp(agc_sp_t a, agc_sp_t b)
{
    agc_sp_t x = (a < AGC_SP_MIN) ? AGC_SP_MIN : a; /* a limited to -1.0, B0 */
    agc_sp_t y = (b < AGC_SP_MIN) ? AGC_SP_MIN : b; /* b limited, B0 */

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

/**********************************************************************
 * AGC: none (DDOUBL after EXTEND MP) -- the top bit of the product's low
 * word carries into A when the pair is doubled.
 *
 * FUNCTION NAME: dap_low_carry
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   pr        agc_dp_t  I    MP product (A, L), B0
 *
 * RETURNS: int32_t -- int32_t -- +1, -1 or 0 added to the doubled high word
 **********************************************************************/
static int32_t dap_low_carry(agc_dp_t pr)
{
    int32_t lo    = (int32_t) agc_dp_low(pr); /* L after MP, 2^-28 */
    int32_t carry = 0;                       /* +1, 0 or -1 into A */

    if (!AGC_ASSERT(lo > -16384 && lo < 16384))
    {
        return (0); /* no valid result */
    }

    if (lo >= 8192)
    {
        carry = 1;
    }
    else if (lo <= -8192)
    {
        carry = -1;
    }

    if (!AGC_ASSERT(carry >= -1 && carry <= 1))
    {
        return (0); /* no valid result */
    }

    return (carry);
}

/**********************************************************************
 * AGC: none (CCS on a sign word) -- v when s > 0, -v when s < 0, 0 when s =
 * 0.
 *
 * FUNCTION NAME: dap_signed
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  ---------------------------------------------
 *   s         int32_t  I    sign word (SALFA, -VT/180, ...), any scale
 *   v         int32_t  I    value, SP units
 *
 * RETURNS: int32_t -- int32_t -- v with the sign of s (0 for s = 0)
 **********************************************************************/
static int32_t dap_signed(int32_t s, int32_t v)
{
    int32_t r = 0; /* v with the sign of s, SP units */

    if (!AGC_ASSERT(v > -65536 && v < 65536))
    {
        return (0); /* no valid result */
    }

    if (s > 0)
    {
        r = v;
    }
    else if (s < 0)
    {
        r = -v;
    }

    if (!AGC_ASSERT(r == v || r == -v || r == 0))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * AGC: none (CDU counter as an SP angle, p. 1063) -- SP angle /180 from a
 * raw 15-bit CDU counter (2s complement).
 *
 * FUNCTION NAME: cdu_sp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   raw       uint16_t  I    CDU counter pattern
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- the gimbal angle /180, B0
 **********************************************************************/
static agc_sp_t cdu_sp(uint16_t raw)
{
    int32_t w = (int32_t) (raw & 077777u); /* counter value, SP units */

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

/**********************************************************************
 * Dead-zone test (2D/SDZ, 3DDZ, 4D/SDZ, CM_ENTRY_DIGITAL_AUTOPILOT.agc p.
 * 1076): the AGC adds LIM - 1 to |x| and tests for overflow, so the tag is
 * nonzero for |x| >= 16385 - LIM.
 *
 * FUNCTION NAME: dap_dz
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   x         agc_sp_t  I    quantity tested, B0
 *   lim       agc_sp_t  I    dead-zone constant (LIM), B0
 *
 * RETURNS: int -- int -- +1 above, -1 below the dead zone, 0 inside
 **********************************************************************/
static int dap_dz(agc_sp_t x, agc_sp_t lim)
{
    int32_t thr = 16385 - (int32_t) lim; /* threshold 16385 - LIM, SP units */

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

    return (((int32_t) x <= -thr) ? -1 : 0); /* below */
}

/**********************************************************************
 * 2D/SDZ (p. 1076): rate tag, +1 above the 2 deg/s dead zone.
 *
 * FUNCTION NAME: dap_rate_deadzone
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   rate      agc_sp_t  I    body rate x TCDU/180, B0
 *
 * RETURNS: int -- int -- +1, -1 outside the 2 deg/s dead zone, 0 inside
 **********************************************************************/
static int dap_rate_deadzone(agc_sp_t rate)
{
    _Static_assert(AGC_ENTRY_YDOTLIM > 0, "AGC constant");

    if (!AGC_ASSERT(rate >= -16384))
    {
        return (0); /* no valid result */
    }

    return (dap_dz(rate, AGC_ENTRY_YDOTLIM));
}

/**********************************************************************
 * BIASEDZ / 3DDZ / 4D/SDZ / 2D/SDZ (p. 1076): extra-atmospheric dead zone.
 * The error is biased by -SGN(rate) CM/BIAS and tested against the 3.6 deg
 * YAWLIM; a rate above 4 deg/s overrides the error bit.
 *
 * FUNCTION NAME: dap_biasedz
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   rate      agc_sp_t  I    body rate x TCDU/180, B0
 *   err       agc_sp_t  I    attitude error /180, B0
 *
 * RETURNS: int -- int -- the jet tag +1, -1 or 0 (YJETCODE / P/RJCODE
 *          index)
 **********************************************************************/
static int dap_biasedz(agc_sp_t rate, agc_sp_t err)
{
    int32_t e = (int32_t) err; /* biased error, SP units */
    int     etag; /* error tag */

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
    int rate_tag = dap_rate_deadzone(rate); /* rate tag */

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
} /* dap_biasedz */

/**********************************************************************
 * YJETCODE / P/RJCODE tables (p. 1080), index -1, 0, +1.
 *
 * FUNCTION NAME: agc_entry_dap_yaw_jet_bits
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------------
 *   jndx      int   I    jet tag -1, 0, +1
 *
 * RETURNS: uint16_t -- uint16_t -- channel 5 yaw jet bits
 **********************************************************************/
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

/**********************************************************************
 * P/RJCODE table (CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1080), index -1, 0, +1.
 *
 * FUNCTION NAME: agc_entry_dap_roll_jet_bits
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------------
 *   jndx      int   I    jet tag -1, 0, +1
 *
 * RETURNS: uint16_t -- uint16_t -- channel 5/6 jet bits
 **********************************************************************/
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

/**********************************************************************
 * AGC: none (AOG erasable, CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1063) --
 * accessor for other lanes.
 *
 * FUNCTION NAME: agc_erase_aog
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- AOG outer gimbal angle /180, B0
 **********************************************************************/
agc_sp_t agc_erase_aog(void)
{
    if (!AGC_ASSERT(aog >= -16384))
    {
        return (0); /* no valid result */
    }

    return (aog);
}

/**********************************************************************
 * AGC: none (AIG erasable, CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1063) --
 * accessor for other lanes.
 *
 * FUNCTION NAME: agc_erase_aig
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- AIG inner gimbal angle /180, B0
 **********************************************************************/
agc_sp_t agc_erase_aig(void)
{
    if (!AGC_ASSERT(aig >= -16384))
    {
        return (0); /* no valid result */
    }

    return (aig);
}

/**********************************************************************
 * AGC: none (AMG erasable, CM_ENTRY_DIGITAL_AUTOPILOT.agc p. 1063) --
 * accessor for other lanes.
 *
 * FUNCTION NAME: agc_erase_amg
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- AMG middle gimbal angle /180, B0
 **********************************************************************/
agc_sp_t agc_erase_amg(void)
{
    if (!AGC_ASSERT(amg >= -16384))
    {
        return (0); /* no valid result */
    }

    return (amg);
}

/**********************************************************************
 * FLUSHJET (p. 1065): all jets off (channels 6 and 5).
 *
 * FUNCTION NAME: agc_entry_dap_flushjet
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_entry_dap_flushjet(void)
{
    agc_chan_write(AGC_CHAN_ROLLJETS, 0);
    agc_chan_write(AGC_CHAN_PYJETS, 0);

    if (!AGC_ASSERT(agc_chan_read(AGC_CHAN_ROLLJETS) == 0u))
    {
        return;
    }

    if (!AGC_ASSERT(agc_chan_read(AGC_CHAN_PYJETS) == 0u))
    {
        return;
    }
}

/* ======================================================================
 * READGYMB / BODYRATE (pp. 1063-1072)
 * ==================================================================== */

/**********************************************************************
 * CM/GYMIC (p. 1064): first pass: GYMDIFSW set, no rates yet.
 *
 * FUNCTION NAME: dap_gymic
 *
 * ARGUMENTS:
 *   Argument      Type  I/O  Description (scale)
 *   ------------  ----  ---  --------------------------------------------
 *   set_gymdifsw  bool  I    true: set GYMDIFSW (first pass)
 *
 * RETURNS: none
 **********************************************************************/
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

/**********************************************************************
 * RATEAVG (p. 1065): DELV(EST) = DELV + (DELV - OLDELV)/2.
 *
 * FUNCTION NAME: dap_rateavg
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   old       agc_sp_t  I    previous increment, B0
 *   now       agc_sp_t  I    this increment, B0
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- DELV(EST), B0
 **********************************************************************/
static agc_sp_t dap_rateavg(agc_sp_t old, agc_sp_t now)
{
    agc_sp_t d = /* now - old, B0 */
                 entry_sp_sat((int32_t) now - (int32_t) old);

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

/**********************************************************************
 * BODYRATE roll and the GAMDOT correction (pp. 1071-1072): P = ODOT + IDOT
 * SM; with GAMDOT: P -= SR GAMDOT SIN(TRIM), Q -= CR GAMDOT, R += SR GAMDOT
 * COS(TRIM).
 *
 * FUNCTION NAME: dap_bodyrate_p
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void dap_bodyrate_p(void)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */
    agc_dp_t           pr =  /* SIN(AMG) x -DELAIG, B0 */
                            agc_sp_mul(agc_sp_sin(amg), m_delaig);
    agc_sp_t rnd = /* DDOUBL carry, B0 */
                   (agc_sp_t) dap_low_carry(pr);
    agc_sp_t jet; /* P increment, B0 */
    agc_sp_t msr; /* -SIN(ROLL) GAMDOT, B0 */

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

/**********************************************************************
 * BODYRATE end (p. 1072): with CMDAPARM set, ATTRATES via T5RUPT soon.
 *
 * FUNCTION NAME: dap_bodyrate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t dap_bodyrate(void)
{
    /* BODYRATE pitch and yaw (pp. 1070-1071):
     *   Q = IDOT CO CM + MDOT SO,   R = -IDOT CM SO + MDOT CO.               */
    agc_sp_t  cosm = agc_sp_cos(amg); /* COS AMG, B0 */
    agc_sp_t  sino = agc_sp_sin(aog); /* SIN AOG, B0 */
    agc_sp_t  coso = agc_sp_cos(aog); /* COS AOG, B0 */
    agc_dp_t  j; /* DP sum, B0 */
    agc_sp_t  jet; /* increment, B0 */
    agc_ovf_t ovf = 0; /* DAS overflow */

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
} /* dap_bodyrate */

/**********************************************************************
 * READGYMB / READGYM1 / DOBRATE? (pp. 1063-1064): .1 s task.
 *
 * FUNCTION NAME: agc_task_readgymb
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_task_readgymb(void)
{
    unsigned int sel = 0u; /* CM/DSTBY, GYMDIFSW branch */

    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_cm_gymdt(
        entry_sp_sat((int32_t) agc_erase_cm_gymdt() + 10)); /* ADS CM/GYMDT */

    if ((agc_erase_imodes33() & 040u) != 0u)
    { /* not fine align */
        agc_flag_clear(AGC_FLAG_GYMDIFSW);
        agc_entry_dap_flushjet();
        dap_gymic(false);

        /* DOBRATE1 with JETEM = 0: next READGYMB in .1 s               */
        return (agc_waitlist_call(10, AGC_TASK_READGYMB));
    }

    /* XCH AOG / MSU AOG: modular (2s complement) difference, so a gimbal
     * passing 180 deg gives the small step, not an overflow.            */
    m_delaog =
        entry_angle_wrap((int32_t) aog - (int32_t) cdu_sp(agc_erase_cdux()));
    aog      = cdu_sp(agc_erase_cdux());
    m_delaig =
        entry_angle_wrap((int32_t) aig - (int32_t) cdu_sp(agc_erase_cduy()));
    aig      = cdu_sp(agc_erase_cduy());
    m_delamg =
        entry_angle_wrap((int32_t) amg - (int32_t) cdu_sp(agc_erase_cduz()));
    amg = cdu_sp(agc_erase_cduz());

    if (!agc_flag_test(AGC_FLAG_CM_DSTBY))
    {
        sel |= 2u; /* DAP active        */
    }

    if (!agc_flag_test(AGC_FLAG_GYMDIFSW))
    {
        sel |= 1u; /* first pass        */
    }

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

    agc_status_t st = agc_waitlist_call(10, AGC_TASK_READGYMB); /* DOBRATE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (dap_bodyrate());
} /* agc_task_readgymb */

/* ======================================================================
 * ATTRATES and the pitch/yaw DAPs (pp. 1073-1079)
 * ==================================================================== */

/**********************************************************************
 * EXDAP1 (p. 1078): yaw axis with the error in RAXERR (sign set by the
 * caller), rate RREL or (mode +0) BETADOT; pitch on QAXERR, QREL.
 *
 * FUNCTION NAME: dap_exdap_yaw_pitch_jets
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *   yaw_err   agc_sp_t             I    yaw (or roll-hold) error /180, B0
 *
 * RETURNS: uint16_t -- uint16_t -- channel 5 pitch/yaw jet bits
 **********************************************************************/
static uint16_t dap_exdap_yaw_pitch_jets(agc_entry_state_t *es,
                                         agc_sp_t           yaw_err)
{
    agc_sp_t rate; /* yaw rate, B0 */
    uint16_t bits; /* jet bits */

    if (!AGC_ASSERT(es != NULL))
    {
        return (0); /* no valid result */
    }

    raxerr = yaw_err;
    ak2    = yaw_err;
    rate   = (es->cmdapmod == AGC_ENTRY_DAPMOD_PLUS0) ? /* mode +0 */
             betadot : rrel;
    bits = agc_entry_dap_yaw_jet_bits(dap_biasedz(rate, yaw_err));
    bits = (uint16_t) (bits +
                       agc_entry_dap_roll_jet_bits(dap_biasedz(qrel,
                                                               qaxerr)));

    if (!AGC_ASSERT(bits <= 0377u))
    {
        return (0); /* no valid result */
    }

    return (bits);
}

/**********************************************************************
 * EXDAP2 (pp. 1078-1079): |cos alfa| <= .707, CMDAPMOD = +1. Beta is
 * controlled with the roll jets, roll held with the yaw jets.
 *
 * FUNCTION NAME: dap_exdap_beta_roll_hold
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: uint16_t -- uint16_t -- channel 5 jet bits (EXDAP1)
 **********************************************************************/
static uint16_t dap_exdap_beta_roll_hold(agc_entry_state_t *es)
{
    agc_sp_t err; /* error /180, B0 */
    agc_sp_t rate; /* rate, B0 */
    int      tag; /* jet tag */

    if (!AGC_ASSERT(es != NULL))
    {
        return (0); /* no valid result */
    }

    es->cmdapmod = AGC_ENTRY_DAPMOD_PLUS1;
    jetag        = -1; /* roll jets now from here      */

    if (es->wake_separation_interlock > 0)
    {
        es->wake_separation_interlock = 0;
    }

    err     = (agc_sp_t) dap_signed(es->salfa, -(int32_t) raxerr);
    paxerr1 = entry_sp_mp(err, AGC_SP_HALF); /* error/360   */
    rate    = (agc_sp_t) dap_signed(es->salfa, -(int32_t) betadot);
    tag     = dap_biasedz(rate, err);
    agc_chan_write(AGC_CHAN_ROLLJETS, agc_entry_dap_roll_jet_bits(tag));
    err = entry_angle_wrap((int32_t) es->rollhold -
                           (int32_t) es->roll_180); /* MSU ROLL/180  */
    err = (agc_sp_t) dap_signed(es->salfa, err);

    if (!AGC_ASSERT(es->cmdapmod == AGC_ENTRY_DAPMOD_PLUS1))
    {
        return (0); /* no valid result */
    }

    return (dap_exdap_yaw_pitch_jets(es, err));
}

/**********************************************************************
 * EXDAP / EXDAP4 / EXDAP3 (pp. 1077-1078): extra-atmospheric DAP. With |cos
 * alfa| > .707 the first pass with P63FLAG = +0 calls WAKEP62 in NSEC (65
 * deg at 3 deg/s).
 *
 * FUNCTION NAME: dap_exdap
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: uint16_t -- uint16_t -- channel 5 jet bits (0 when WAITLIST
 *          overflowed)
 **********************************************************************/
static uint16_t dap_exdap(agc_entry_state_t *es)
{
    if (!AGC_ASSERT(es != NULL))
    {
        return (0); /* no valid result */
    }

    es->cmdapmod = AGC_ENTRY_DAPMOD_PLUS0;

    if (dap_dz(es->calfa, AGC_ENTRY_C45LIM) == 0)
    {
        return (dap_exdap_beta_roll_hold(es));
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
        return (dap_exdap_yaw_pitch_jets(es, raxerr));
    }

    es->cmdapmod = AGC_ENTRY_DAPMOD_MINUS0; /* damping only */

    if (!AGC_ASSERT(es->calfa <= 0))
    {
        return (0); /* no valid result */
    }

    return (dap_exdap_yaw_pitch_jets(es, (agc_sp_t) (-raxerr)));
} /* dap_exdap */

/**********************************************************************
 * CM/FDAI, CM/FDAIR, CM/DUMPR, CMTMFILE, CM/END (pp. 1090-1091): roll error
 * integration between 2 s updates, the needles on alternate passes and the
 * body-rate TM file (UPBUFF) on the others.
 *
 * FUNCTION NAME: dap_fdai
 *
 * ARGUMENTS:
 *   Argument   Type  I/O  Description (scale)
 *   ---------  ----  ---  -----------------------------------------------
 *   integrate  bool  I    true: CM/FDAI (integrate PHIDOT); false: CM/FDAIR
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t dap_fdai(bool integrate)
{
    int idx; /* TM file slot */

    if (integrate)
    { /* CM/FDAI       */
        agc_sp_t d = entry_sp_mp(
            entry_sp_mp((agc_sp_t) (-phidot), agc_entry_state()->calfa),
            AGC_SP_HALF); /* integration step, B0 */

        paxerr1 = entry_sp_store_ovf((int32_t) paxerr1 + d); /* OVFL OK */
    }

    ak     = entry_sp_mp(AGC_SP_HALF, paxerr1); /* PAXERR        */
    sw_ndx = (agc_sp_t) (-sw_ndx);              /* CM/DUMPR      */

    if (sw_ndx > 0)
    {
        bool init = needler_init; /* NEEDLER init pass */

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

    if (!AGC_ASSERT(sw_ndx >= -13 && sw_ndx <= -1)) /* TM file index */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* CM/END        */
} /* dap_fdai */

/* ======================================================================
 * Predictive roll system CM/RCS (pp. 1081-1089)
 * ==================================================================== */

/**********************************************************************
 * SETJTAG (p. 1081): 2 s task 1.2 s after PIPA time: roll update due.
 *
 * FUNCTION NAME: agc_task_setjtag
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_task_setjtag(void)
{
    /* DEVIATION: AGC stores -TIME1 (CS TIME1); C keeps +TIME1 and
     * TIMETST forms (TIME1 - TUSED) mod 2^14, the same value the AGC gets
     * from -OLDTIME1 + TIME1 + 2 POS1/2 with overflow (exhaustive model:
     * review-guid-entry note). TUSED is DAP-private, not downlinked.     */
    tused = agc_erase_time1(); /* SETJTAG: TUSED       */
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

/**********************************************************************
 * GETON2 / GETOFF / TOFFOVFL / GETOFF2 / TIMSCAL (pp. 1085-1086).
 *
 * FUNCTION NAME: pp_getoff
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------------
 *   lcx       agc_sp_t                I    X/360 roll error, B0
 *   mvt       agc_sp_t                I    -VT/180 roll rate, B0
 *   vdt       agc_sp_t                I    drift velocity VDT/180, B0
 *   o         agc_entry_roll_cmd_t *  IO   TON1 TOFF TON2 (cs) and jet
 *                                          indices
 *
 * RETURNS: none
 **********************************************************************/
static void pp_getoff(agc_sp_t lcx, agc_sp_t mvt, agc_sp_t vdt,
                      agc_entry_roll_cmd_t *o)
{
    int32_t num; /* TOFF numerator, B0 */

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

        if (dap_signed(num, num) >= dap_signed(vdt, vdt)) /* |num|, |vdt| */
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
} /* pp_getoff */

/**********************************************************************
 * MAXVTEST / MAXVTIM1 / GETON1 / OVRLINE (pp. 1084-1085): first burn to the
 * drift velocity VDT (or the limit VM), and its duration TON1.
 *
 * FUNCTION NAME: pp_maxvtest
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------------
 *   lcx       agc_sp_t                I    X/360 roll error, B0
 *   mvt       agc_sp_t                I    -VT/180 roll rate, B0
 *   vsq       agc_sp_t                I    V^2 term, B0
 *   o         agc_entry_roll_cmd_t *  IO   TON1 and jet indices
 *   vdt       agc_sp_t *              O    drift velocity VDT/180, B0
 *
 * RETURNS: none
 **********************************************************************/
static void pp_maxvtest(agc_sp_t lcx, agc_sp_t mvt, agc_sp_t vsq,
                        agc_entry_roll_cmd_t *o, agc_sp_t *vdt)
{
    agc_sp_t xd; /* XD - XS, B0 */
    bool     qpos = true; /* Q tag from TC GETON1   */

    if (!AGC_ASSERT(o != NULL && vdt != NULL))
    {
        return;
    }

    o->jndx1 = -o->jndx;
    xd       = entry_sp_sat((int32_t) vsq + lcx - AGC_ENTRY_XS_360); /* XD-XS */

    if ((int32_t) xd + AGC_ENTRY_MVM_360K >= 0)
    {
        qpos = false;                            /* MAXVTIM1: ZQ    */
        *vdt = (agc_sp_t) (-AGC_ENTRY_MVM_360K); /* VMT/180         */
    }
    else
    {
        agc_dp_t pr = agc_sp_mul(xd, AGC_ENTRY_KTRCS); /* XD KTRCS, B0 */
        int32_t  rd = dap_low_carry(pr); /* DDOUBL carry */

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

    if (!AGC_ASSERT(o->ton1 >= 0))
    {
        return;
    }
} /* pp_maxvtest */

/**********************************************************************
 * Phase plane of CM/RCS from WHICHALF on (pp. 1083-1086). lcx: X/360; mvt2:
 * -2 V T/180 as first stored in -VT/180. *vdt gets VDT/180.
 *
 * FUNCTION NAME: dap_phase_plane
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------------
 *   lcx_in    agc_sp_t                I    X/360 roll error, B0
 *   mvt2      agc_sp_t                I    -2 V T/180, B0
 *   o         agc_entry_roll_cmd_t *  O    TON1 TOFF TON2 (cs) and jet
 *                                          indices
 *   vdt       agc_sp_t *              O    drift velocity VDT/180, B0
 *
 * RETURNS: none
 **********************************************************************/
static void dap_phase_plane(agc_sp_t lcx_in, agc_sp_t mvt2,
                            agc_entry_roll_cmd_t *o, agc_sp_t *vdt)
{
    agc_sp_t mvt = (agc_sp_t) (mvt2 / 2); /* -VT/180, B0 */
    /* CA SR rewrites SR and so shifts it again (the editing registers
     * edit on every write): SR = -VT/360 for the DZ2 test.             */
    agc_sp_t sr  = (agc_sp_t) (mvt / 2); /* SR register: -VT/360, B0 */
    agc_sp_t lcx = lcx_in; /* X/360, B0 */
    agc_sp_t vsq = /* V^2 term, B0 */
                   entry_sp_mp(entry_sp_mp(mvt2, mvt), AGC_ENTRY_180_8ATT);
    int32_t l = (int32_t) AGC_ENTRY_XS_360 - 2 * (int32_t) vsq; /* BUFLIM */
    int32_t t = /* reflection test, B0 */
                dap_signed(mvt, -l) + lcx + AGC_ENTRY_XS_360;
    bool dz = false; /* in the dead zone */

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

    if (!AGC_ASSERT(o->ton1 >= 0 && o->ton2 >= 0)) /* cs, never negative */
    {
        return;
    }
} /* dap_phase_plane */

/**********************************************************************
 * CM/RCS phase plane (CM_ENTRY_DIGITAL_AUTOPILOT.agc pp. 1081-1086), entry
 * for tests: from X/360 and -VT/180 the jet times TON1, TOFF, TON2 and the
 * jet indices.
 *
 * FUNCTION NAME: agc_entry_dap_roll_phase_plane
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------------
 *   lcx_360   agc_sp_t                I    roll error X/360, B0
 *   mvt_180   agc_sp_t                I    -VT/180 roll rate, B0
 *   out       agc_entry_roll_cmd_t *  O    TON1 TOFF TON2 (cs) and the jet
 *                                          indices
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when out is
 *          NULL
 **********************************************************************/
agc_status_t agc_entry_dap_roll_phase_plane(agc_sp_t lcx_360, agc_sp_t mvt_180,
                                            agc_entry_roll_cmd_t *out)
{
    agc_sp_t vdt = 0; /* drift velocity, B0 */

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

/**********************************************************************
 * JETCALL1 .. JETACTN (pp. 1088-1089): walk the interval list from `start`;
 * a positive interval gets a WAITLIST call to JETCALL and its bits fire
 * now, +0 means its bits apply for the rest of T.
 *
 * FUNCTION NAME: dap_jetlist
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------------
 *   start     int   I    first list slot (0..2)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t dap_jetlist(int start)
{
    int k; /* slot */

    if (!AGC_ASSERT(start >= 0 && start < DAP_JETLIST))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    nujet = 0;

    for (k = start; k < DAP_JETLIST; k++)
    {
        agc_sp_t v = jet_time[k]; /* interval, cs B14 */

        jet_time[k] = (k == 0) ? 0 : -1; /* DXCH: processed  */

        if (v > 0)
        {
            nujet = (uint16_t) (nujet + jet_bits[k]); /* ADS NUJET        */
            agc_status_t st = /* WAITLIST status */
                              agc_waitlist_call((uint16_t) v, AGC_TASK_JETCALL);

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
} /* dap_jetlist */

/**********************************************************************
 * TIMETST (pp. 1087-1088): keep only intervals > 2 cs that start within the
 * time left in the 2 s period; the one that runs past T is set to +0.
 *
 * FUNCTION NAME: dap_timetst
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ------------------------
 *   o         const agc_entry_roll_cmd_t *  I    TON1 TOFF TON2 and the jet
 *                                                indices
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t dap_timetst(const agc_entry_roll_cmd_t *o)
{
    const agc_sp_t times[DAP_JETLIST] = {o->ton1, o->toff, o->ton2};
    const int      ndx[DAP_JETLIST]   = {o->jndx, 0, o->jndx1};
    int32_t        used               =
        (int32_t) (((uint32_t) agc_erase_time1() - (uint32_t) tused) &
                   037777u) +
        AGC_ENTRY_T_M3; /* TUSED, cs B14 */
    int  k; /* slot */
    bool past = false; /* runs past T */

    if (!AGC_ASSERT(used < 16384))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(o->jndx >= -1 && o->jndx <= 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (k = 0; k < DAP_JETLIST; k++)
    {
        jet_bits[k] = (k == 1) ? /* TOFF: none */
                      0u : agc_entry_dap_roll_jet_bits(ndx[k]);

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
} /* dap_timetst */

/**********************************************************************
 * JETCALL (p. 1089): waitlist entry; quench the roll jets if the DAP is on
 * standby, else continue with the next interval.
 *
 * FUNCTION NAME: agc_task_jetcall
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
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

/**********************************************************************
 * DZCALL (p. 1089): in the dead zone: ROLLHOLD = ROLL (mode -0) or the roll
 * command, roll jets off.
 *
 * FUNCTION NAME: rcs_dzcall
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: none
 **********************************************************************/
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

/**********************************************************************
 * CM/RCS .. COMPAT (pp. 1081-1083): roll error X and rate for the phase
 * plane; then TIMETST/JETCALL or DZCALL; exit through CM/FDAIR.
 *
 * FUNCTION NAME: dap_cm_rcs
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t dap_cm_rcs(agc_entry_state_t *es)
{
    agc_entry_roll_cmd_t o    = {0, 0, 0, -1, 1, false};
    int32_t              v    = -40 * (int32_t) prel; /* -2T/TCDU P */
    agc_sp_t             mvt2 =                       /* -2VT/180, B0 */
                                entry_sp_sat(v / 16384 + v % 16384);
    agc_sp_t sr = /* SR register, B0 */
                  (agc_sp_t) (-(int32_t) es->roll_180 / 2);
    agc_sp_t rollc = agc_dp_high(es->rollc); /* ROLLC/360, B0 */
    agc_sp_t lcx; /* X/360, B0 */

    if (!agc_flag_test(AGC_FLAG_LATSW))
    {
        agc_flag_set(AGC_FLAG_LATSW); /* over the top */
        lcx = entry_sp_sat((int32_t) rollc + sr);
    }
    else
    {
        /* GETLCX / TRTAGXPI (p. 1082): X/360 = RCOM/360 - R/360 in (-1, 1),
         * then the shortest angular path for a single-jet stop from rate
         * -VT/180.                                                       */
        agc_sp_t calfa = es->calfa; /* COS ALFA, B0 */

        int32_t rcom = (rollc >= 0) ? /* RCOM >= 0 */
                       rollc : (int32_t) rollc + 16384;
        int32_t  rneg       = (sr > 0) ? (int32_t) sr - 16384 : sr; /* -R/360 */
        int32_t  lcx_getlcx = rcom + rneg; /* X/360, B0 */
        agc_dp_t p          = /* V^2 product, B0 */
                              agc_sp_mul(entry_sp_sat(dap_signed(mvt2,
                                                                 mvt2)), mvt2);
        agc_dp_t p2 = /* 1/16A1 term, B0 */
                      agc_sp_mul(agc_dp_high(p), AGC_ENTRY_180_8ATT);
        int32_t q; /* stop angle, B0 */
        int32_t t; /* path test, B0 */

        if (!AGC_ASSERT(calfa != 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        q = (int32_t) (((int64_t) agc_dp_high(p2) * 16384 + agc_dp_low(p2)) /
                       calfa);
        q = entry_sp_sat(q); /* DV CALFA    */
        t = dap_signed(lcx_getlcx, 8192) + lcx_getlcx + q;

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
    } /* end GETLCX */

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
    agc_status_t st = dap_timetst(&o); /* TIMETST status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(jetag == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dap_fdai(false)); /* ROLLDUMP   */
} /* dap_cm_rcs */

/**********************************************************************
 * ATTRATES (pp. 1073-1075): T5RUPT continuation of BODYRATE.
 *
 * FUNCTION NAME: agc_task_attrates
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_task_attrates(void)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */
    uint16_t           bits; /* jet bits */

    if (!AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* ATTRATES (p. 1073-1074): integrate ALFA, BETA, ROLL:
     *   CB PHIDOT = CA P + SA R, BETADOT = -SA P + CA R,
     *   ALFADOT = Q + SB PHIDOT. */
    agc_sp_t ca; /* COS ALFA, B0 */
    agc_sp_t sa; /* SIN ALFA, B0 */

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
         * yaw rate error = R - P TAN(ALFA) ~ R - P SIN(-20 deg). */
        agc_sp_t yerr; /* yaw error, B0 */
        uint16_t bits_yaw; /* yaw jet bits */

        es->cmdapmod = AGC_ENTRY_DAPMOD_MINUS1;
        ak1          = -1; /* zero pitch, yaw needles    */
        ak2          = -1;
        yerr         = entry_sp_sat(
            (int32_t) entry_sp_mp((agc_sp_t) (-prel),
                                  AGC_ENTRY_SINTRIM) + rrel);
        bits_yaw = agc_entry_dap_yaw_jet_bits(dap_rate_deadzone(yerr));
        bits_yaw =
            (uint16_t) (bits_yaw +
                        agc_entry_dap_roll_jet_bits(dap_rate_deadzone(qrel)));

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
} /* agc_task_attrates */

/* ======================================================================
 * Initialisation: CM/DAPON, CM/DAPIC (pp. 1066-1069)
 * ==================================================================== */

/**********************************************************************
 * CM/DAPON +4 .. (p. 1066-1067): once CM/POSE has formed one GAMA
 * difference, activate the entry DAP in attitude hold, arm it, set
 * NODOFLAG, and continue at P62.1.
 *
 * FUNCTION NAME: agc_job_cm_dapon_wait
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_cm_dapon_wait(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

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

/**********************************************************************
 * CM/DAPON (p. 1066): RCS DAP off (DAPBIT1/2, T5LOC/T6LOC idle), jets
 * flushed, DAPDATR1 configuration bits 13-15 cleared.
 *
 * FUNCTION NAME: agc_job_cm_dapon
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
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

/**********************************************************************
 * CM/DAP2C (p. 1068): time from PIPTIME to the next READGYMB slot, 5 cs
 * after a multiple of .1 s from PIPTIME; never 0.
 *
 * FUNCTION NAME: dapic_first_dt
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- delay to the first READGYMB, cs (1..10,
 *          never 0)
 **********************************************************************/
static agc_sp_t dapic_first_dt(void)
{
    int32_t delta = (int32_t) (((uint32_t) agc_erase_time1() -
                                (uint32_t) agc_dp_low(agc_erase_piptime())) &
                               037777u); /* TIME1 - PIPTIME, cs B14 */
    int32_t x = delta - 5; /* JETEM - 5, cs */
    int     n; /* loop bound */

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

/**********************************************************************
 * CM/DAPIC (pp. 1068-1069): start READGYMB in phase with PIPTIME, set
 * CM/DSTBY and LATSW (clear GYMDIFSW .. ENTRYDSP), restart group 6 and
 * group 4 at P62.2, continue at P62.2.
 *
 * FUNCTION NAME: agc_entry_dap_ic
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
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
    unsigned int i; /* loop index */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_erase_set_cm_gymdt(dapic_first_dt());
    const uint16_t dt = (uint16_t) agc_erase_cm_gymdt(); /* CM/GYMDT, cs */
    agc_status_t   st = agc_waitlist_call(dt, AGC_TASK_READGYMB); /* WAITLIST */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    for (i = 0; i < (sizeof (clr) / sizeof (clr[0])); i++)
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
} /* agc_entry_dap_ic */
