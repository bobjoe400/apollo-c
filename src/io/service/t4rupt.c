/**********************************************************************
 * FILE NAME: t4rupt.c
 *
 * PURPOSE:
 *   T4RUPT, the 120 ms interrupt common to both vehicles: DSKY relay
 *   output (CDRVE, DSPOUT, HANG20, QUIKDSP), the PRO key (PROCEEDE), the
 *   T4JUMP activity table and the ISS monitors (IMUMON, TNONTEST,
 *   C33TEST, GLOCKMON, SETISSW, CAGESUB, LAMPTEST) with their waitlist
 *   tasks (ENDTNON, UNZ2, ISSUP).
 *
 * AGC SOURCE:
 *   Comanche055 T4RUPT_PROGRAM.agc pp.133-169 (page numbers in the
 *   function prologs are Comanche's)
 *   Luminary099 T4RUPT_PROGRAM.agc pp.155-189 (same code 22 pages on;
 *   differs: T4JUMP table, ENDTNON/OPONLY flagword, ISSUP DRIFTFLG,
 *   ITURNON2 RRINIT, IMUCAGE engine-off)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name         Type      I/O  Scale       Description
 *   -----------  --------  ---  ----------  ---------------------------
 *   DSPTAB +11D  uint16_t  I/O  bits        lamp relay word
 *   IMODES30/33  uint16_t  I/O  bits        ISS / channel 33 status
 *   CDUZ         uint16_t  I    2's comp pi middle gimbal angle
 *   RADMODES     uint16_t  O    bits        LM: RRINIT at ISS turn-on
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  -------------------------------------------
 *   agc_dsky_dspout       DSPOUT / DSPOUTSB (io-pinball)
 *   agc_t4_optics_*       OPTDRIVE / OPTMON (t4rupt_csm.c)
 *   agc_t4_rr_monitor     RRAUTCHK (t4rupt_lm.c); agc_lm_dap_rcsmon
 *   agc_imu_*             NOATTOFF, ZEROICDU, SETCOARS, RNDREFDR, IMUBAD
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarms 00207 (turn-on request dropped), 00212 (PIPA fail), 00213
 *   (turn-on without operate), 00214 (ISS off while in use), 01105 /
 *   01106 (down/uplink too fast); VARALARM 07777/03777/14777 (ISS
 *   warning).  BAD_ARG on out-of-range words.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in interrupt context (agc_t4rupt_rupt), both vehicles.
 *
 * NOTES:
 *   DEVIATION (DSRUPTSW): the AGC packs the phase counter (7..0) and
 *   the quick-display sequence into one ones'-complement word (negative
 *   = quick passes pending: magnitude bit 14 = "relays written last
 *   pass", magnitude / BIT9 = passes left, low bits = phase).  Here the
 *   three fields are kept separately (phase, quick_left, quick_wrote);
 *   the sequencing is the same.
 *   DEVIATION (TIME4): agc_t4rupt() returns the delay to the next
 *   T4RUPT in centiseconds; agc_t4rupt_rupt() writes it into TIME4 as
 *   the AGC words 20MRUPT (37776, 2 cs) and 120MRUPT (16372, 12 cs) do.
 *   The NODSPY path's repeated "ADS TIME4" is modelled as its intended
 *   effect, 20 ms per skipped quick pass.
 *
 * ALGORITHM:
 *   Each pass: CDRVE sends the lamp word or one DSKY relay word; after
 *   a relay word the next five T4RUPTs come 20 ms apart (QUIKDSP),
 *   alternating relays-off and a further relay word.  PROCEEDE edge-
 *   detects channel 32 bit 14.  T4JUMP runs one activity per phase:
 *     CSM  OPTDRIVE, OPTMON, IMUMON, -, (repeat)
 *     LM   RCSMONIT, RRAUTCHK, IMUMON, DAPT4S, (repeat)
 *   IMUMON -> TNONTEST -> C33TEST -> GLOCKMON (every 480 ms).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"

#define BIT1   000001u /* BIT1 */
#define BIT2   000002u /* BIT2 */
#define BIT4   000010u /* BIT4 */
#define BIT5   000020u /* BIT5 */
#define BIT6   000040u /* BIT6 */
#define BIT7   000100u /* BIT7 */
#define BIT8   000200u /* BIT8 */
#define BIT9   000400u /* BIT9 */
#define BIT10  001000u /* BIT10 */
#define BIT11  002000u /* BIT11 */
#define BIT13  010000u /* BIT13 */
#define BIT14  020000u /* BIT14 */
#define BIT15  040000u /* BIT15 */
#define WORD15 077777u /* one AGC word, 15 bits */
#define LOW11  003777u /* LOW11 */

#define T4_PHASES       8       /* RUPTREG1 7..0, T4JUMP table          */
#define T4_QUICK_PASSES 5       /* HANG20: 11,14,9 = 22400 -> 5 x BIT9  */
#define T4_FLUSH_PASSES 20      /* SLAP1: -PRIO12 = -12000 -> 20 x BIT9 */
#define T4_20MS_CS      2       /* 20MRUPT  OCT 37776                   */
#define T4_120MS_CS     12      /* 120MRUPT DEC 16372                   */
#define T4_TIME4_OVF    040000u /* TIME4 overflows at 2^14             */
#define RELTAB11        060000u /* RELTAB11: relay word 12 code        */

#define IM30_RDMSK      076400u /* 30RDMSK: bits 15,14,13,12,11,9      */
#define IM33_RDMSK      016000u /* 33RDMSK = PRIO16: bits 13,12,11     */
#define OCT15           000015u /* fail inhibit bits 4,3,1             */
#define OCT15000        015000u /* fail bits 13,12,10 (PRIO15)         */
#define OCT54           000054u /* OCT54: caging, IMU/ICDU fail inhibits */
#define OCT75           000075u /* OCT75: cage/turn-on flags, inhibits */
#define OCT272          000272u /* OCT272: chan 12 knocked down on cage */
#define OCT300          000300u /* BITS7&8                             */
#define OCT1720         001720u /* OCT1720: bits 10,9,8,7,6,4 */
#define OCT740          000740u /* OCT740: gyro select bits of chan 14 */
#define OCT77000        077000u /* OCT77000: chan 14 pulse train bits */
#define OC40010         040010u /* OC40010: NO ATT lamp + send bit */
#define BIT15_6         040040u /* BIT15+6 (CM) / BITS6&15 (LM)        */
#define BITS4_5         000030u /* BITS4&5: coarse align, ICDU zero */
#define PRIO30          030000u /* PRIO30: chan 11 bits 13, 12 */
#define RRINIT          000102u /* LM RRINIT, RADMODES at turn-on      */
#define NINETY_SECS_CS  9000u   /* 90SECS DEC 9000                      */
#define FOUR_SECS_CS    400u    /* 4SECS                                */
#define UNZ2_DELAY_CS   32u     /* BIT6: "wait 300 ms for AGS"          */
#define ISSUP_DELAY_CS  1024u   /* BIT11: wait 10 s for the counters    */
#define CA_ECE_DELAY_CS 6u      /* SIX: error counters in 60 ms         */
#define MINUS_70DEG     6371    /* -70DEGS DEC -.38888 (half-revs 2^14) */
#define MINUS_15DEG     1365    /* -15DEGS DEC -.08333                  */
#define CDU_SIGN        040000u /* sign bit of a CDU counter */
#define LAMPWORD        11 /* DSPTAB +11D                          */

/* Alarm codes raised here (octal, as in the listing).                     */
#define ALARM_ISS_DELAY_FAIL \
    ((agc_alarm_code_t) 00207) /* turn-on req dropped */
#define ALARM_PIPA_FAIL      ((agc_alarm_code_t) 00212) /* PIPA fail      */
#define ALARM_TURNON_NO_OP   ((agc_alarm_code_t) 00213) /* no operate     */
#define ALARM_ISS_OFF_IN_USE ((agc_alarm_code_t) 00214) /* off with IMUSE */
#define ALARM_DNLINK_FAST    ((agc_alarm_code_t) 01105) /* downlink too fast */
#define ALARM_UPLINK_FAST    ((agc_alarm_code_t) 01106) /* uplink too fast */

/* DSRUPTSW, see the DEVIATION above.                                      */
static int  t4_phase;       /* RUPTREG1 / low part of DSRUPTSW, 0..7     */
static int  t4_quick_left;  /* quick (20 ms) passes still to come        */
static bool t4_quick_wrote; /* last quick pass wrote a relay word        */

/**********************************************************************
 * AGC: none (range of DSRUPTSW, see the DEVIATION above)
 *
 * RETURNS: true when the phase and the quick-pass count are in range
 **********************************************************************/
static bool t4_seq_ok(void)
{
    return ((t4_phase >= 0) && (t4_phase < T4_PHASES) &&
            (t4_quick_left >= 0) && (t4_quick_left <= T4_FLUSH_PASSES));
}

/* ---- small helpers -------------------------------------------------- */

/**********************************************************************
 * AGC: none (CS bits / EXTEND / WAND chan) -- clear bits of an output
 * channel
 *
 * ARGUMENTS:
 *   chan  int       I   channel number
 *   bits  uint16_t  I   bits to clear, 15 bits
 **********************************************************************/
static void chan_clear(int chan, uint16_t bits)
{
    if (!AGC_ASSERT(bits <= WORD15))
    {
        return;
    }

    agc_chan_and(chan, (uint16_t) (~(uint32_t) bits & WORD15));

    if (!AGC_ASSERT((agc_chan_read(chan) & bits) == 0u))
    {
        return;
    }
}

/**********************************************************************
 * AGC: none (TS DSPTAB +11D) -- store the lamp relay word; bit 15 set
 * means "send at the next CDRVE"
 *
 * ARGUMENTS:
 *   w  uint16_t  I   new DSPTAB +11D word, 15 bits
 **********************************************************************/
static void set_lampword(uint16_t w)
{
    if (!AGC_ASSERT(w <= WORD15))
    {
        return;
    }

    agc_erase_set_dsptab(LAMPWORD, w);

    if (!AGC_ASSERT(agc_erase_dsptab(LAMPWORD) == w))
    {
        return;
    }
}

/**********************************************************************
 * AGC: none (CAF dt / TC WAITLIST / 2CADR task) -- start a T4RUPT task
 *
 * ARGUMENTS:
 *   dt    uint16_t       I   delay, cs (> 0)
 *   task  agc_task_id_t  I   the waitlist task
 **********************************************************************/
static void call_wait(uint16_t dt, agc_task_id_t task)
{
    agc_status_t st = AGC_STATUS_OK; /* WAITLIST result */

    if (!AGC_ASSERT(dt > 0u) ||

        !AGC_ASSERT(task != AGC_TASK_NONE))
    {
        return;
    }

    st = agc_waitlist_call(dt, task);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return;
    }
}

/**********************************************************************
 * LAMPTEST (p.159): true when a lamp test is in progress
 * (IMODES33 bit 1); the AGC returns to L+1 then, L+2 otherwise.
 *
 * RETURNS: true when a lamp test is in progress (AGC L+1)
 **********************************************************************/
bool agc_t4_lamptest(void)
{
    uint16_t m = agc_erase_imodes33(); /* IMODES33 bits */

    if (!AGC_ASSERT(m <= WORD15))
    {
        return (false);
    }

    return ((m & BIT1) != 0u);
}

/**********************************************************************
 * CDRVE (T4RUPT_PROGRAM.agc p.134 LM, p.134 CM) -- send the DSPTAB +11D
 * lamp relay word if it is pending
 *
 * CCS DSPTAB +11D: positive or +0 goes to DSPOUT; a negative word is
 * replaced positively and its LOW11 bits are written to OUT0 with relay
 * code 12 (RELTAB11).
 *
 * RETURNS: true when the lamp word was sent (go to HANG20), false for
 *          DSPOUT
 **********************************************************************/
static bool t4_cdrve(void)
{
    uint16_t w = agc_erase_dsptab(LAMPWORD); /* DSPTAB +11D, lamp relay word */

    if (!AGC_ASSERT(w <= WORD15))
    {
        return (false);
    }

    if ((w & BIT15) == 0u)
    {
        return (false); /* CCS: + -> TC DSPOUT          */
    }

    /* XCH / MASK LOW11 / TS: replace positively.  For -0 CCS skips the
     * XCH and masks its own +0, so no lamps are sent.                   */
    uint16_t lamps =
        (w == WORD15) ? 0u : (uint16_t) (w & LOW11); /* -0: no lamps */

    set_lampword(lamps);
    agc_chan_write((int) AGC_CHAN_OUT0, (uint16_t) (lamps + RELTAB11));

    if (!AGC_ASSERT((agc_erase_dsptab(LAMPWORD) & BIT15) == 0u))
    {
        return (false); /* not replaced positively */
    }

    return (true);
}

/**********************************************************************
 * HANG20 (p.135): five quick passes follow, TIME4 = 20 ms.
 *
 * RETURNS: the TIME4 delay, 2 cs (20MRUPT)
 **********************************************************************/
static uint16_t t4_hang20(void)
{
    if (!AGC_ASSERT(t4_quick_left == 0)) /* only from a normal pass */
    {
        return (0); /* no valid result */
    }

    t4_quick_left  = T4_QUICK_PASSES;
    t4_quick_wrote = true;

    if (!AGC_ASSERT(t4_phase >= 0 && t4_phase < T4_PHASES))
    {
        return (0); /* no valid result */
    }

    return (T4_20MS_CS);
}

/**********************************************************************
 * CDRVE / DSPOUT / NODSPOUT (pp.134-137): returns the TIME4 delay.
 *
 * RETURNS: the TIME4 delay in cs: 2 after a relay word (HANG20), 12 after
 *          NODSPOUT (120MRUPT), 0 on a pinball error
 **********************************************************************/
static uint16_t t4_display_pass(void)
{
    if (!AGC_ASSERT(t4_seq_ok()) ||
        !AGC_ASSERT(t4_quick_left == 0))
    {
        return (0); /* no valid result */
    }

    if (t4_cdrve())
    {
        return (t4_hang20());
    }

    uint16_t     relay = 0; /* relay word written (unused) */
    agc_status_t st    = agc_dsky_dspout(&relay); /* DSPOUT (io-pinball)      */

    if (st == AGC_STATUS_OK)
    {
        return (t4_hang20());
    }

    if (!AGC_ASSERT(st == AGC_STATUS_NOT_FOUND))
    {
        return (0); /* no valid result */
    }

    agc_chan_write((int) AGC_CHAN_OUT0, 0); /* NODSPOUT: relays off     */

    return (T4_120MS_CS);
}

/**********************************************************************
 * QUIKDSP / QUIKOFF / NODSPY / SYNCT4 (pp.137-138).
 *
 * RETURNS: the TIME4 delay in cs: 2 per quick pass, or 2 x the passes
 *          left when nothing was displayed (SYNCT4)
 **********************************************************************/
static uint16_t t4_quikdsp(void)
{
    if (!AGC_ASSERT(t4_quick_left > 0 && t4_quick_left <= T4_FLUSH_PASSES))
    {
        return (0); /* no valid result */
    }

    if (t4_quick_wrote)
    { /* QUIKOFF                 */
        agc_chan_write((int) AGC_CHAN_OUT0, 0);
        t4_quick_wrote = false;
        t4_quick_left--;

        return (T4_20MS_CS);
    }

    uint16_t relay = 0; /* relay word written (unused) */
    /* DEVIATION: QUIKDSP goes to DSPOUTSB on NOUT alone; the pinball
     * entry also requires DSKYFLAG, so a quick sequence stops early if
     * DSKYFLAG drops in the 100 ms it lasts (DSPOUTSB is pinball's).     */
    agc_status_t st = agc_dsky_dspout(&relay); /* CCS NOUT / DSPOUTSB */

    if (st == AGC_STATUS_OK)
    {
        t4_quick_wrote = true;
        t4_quick_left--;

        return (T4_20MS_CS);
    }

    agc_chan_write((int) AGC_CHAN_OUT0, 0); /* NODSPY                  */
    uint16_t dt = (uint16_t) (T4_20MS_CS * t4_quick_left); /* SYNCT4 loop  */

    t4_quick_left = 0;

    if (!AGC_ASSERT(dt <= T4_20MS_CS * T4_FLUSH_PASSES))
    {
        return (0); /* no valid result */
    }

    return (dt);
}

/**********************************************************************
 * PROCEEDE (p.136): the PRO key, channel 32 bit 14 compared with IMODES33
 * bit 14 (0 = pressed, inverted input).
 **********************************************************************/
static void t4_proceede(void)
{
    uint16_t im33 = agc_erase_imodes33(); /* IMODES33 bits */
    uint16_t ch32 = /* inverted inputs, bit 14 = PRO */
                    agc_chan_read((int) AGC_CHAN_CHAN32);

    if (!AGC_ASSERT(im33 <= WORD15 && ch32 <= WORD15))
    {
        return;
    }

    if (((im33 ^ ch32) & BIT14) == 0u)
    {
        return; /* no change               */
    }

    im33 = (uint16_t) (im33 ^ BIT14);
    agc_erase_set_imodes33(im33);

    if (!AGC_ASSERT(((agc_erase_imodes33() ^ ch32) & BIT14) == 0u))
    {
        return;
    }

    if ((im33 & BIT14) == 0u)
    { /* was off -- now on       */
        agc_status_t st = agc_dsky_prockey_request(); /* NOVAC PROCKEY   */

        if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
        {
            return;
        }
    }
}

/**********************************************************************
 * T4JUMP (p.137): the once-per-0.96 s activity selected by the phase.
 *
 * ARGUMENTS:
 *   phase  int  I   T4RUPT phase, 7..0 (RUPTREG1)
 *
 * RETURNS: the status of the activity run (AGC_STATUS_OK when none)
 **********************************************************************/
static agc_status_t t4_jump(int phase)
{
    if (!AGC_ASSERT(phase >= 0 && phase < T4_PHASES))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int slot = phase & 3; /* table repeats after 4   */

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM ||
                    agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (slot == 2)
    {
        return (agc_t4_imumon());
    }

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        if (slot == 0)
        {
            return (agc_t4_optics_drive()); /* OPTTEST -> OPTDRIVE     */
        }

        if (slot == 1)
        {
            return (agc_t4_optics_monitor()); /* OPTMON */
        }

        return (AGC_STATUS_OK); /* slot 3: TCF RESUME */
    }

    if (slot == 0)
    {
        /* TC RCSMONIT: RCSMON uses Q as a temporary and leaves through
         * RCSMONEX = RESUME on every path (RCS_FAILURE_MONITOR p.190), so
         * the TC never returns and RRAUTCHK is not run in this slot.     */
        return (agc_lm_dap_rcsmon());
    }

    if (slot == 1)
    {
        return (agc_t4_rr_monitor()); /* TCF RRAUTCHK */
    }

    return (agc_t4_gimbal_matrix()); /* slot 3: TCF DAPT4S */
} /* t4_jump */

/**********************************************************************
 * T4RUPT / NORMT4 (p.133): one interrupt.  *next_cs = TIME4 delay.
 *
 * ARGUMENTS:
 *   next_cs  uint16_t *  O   delay to the next T4RUPT, cs (2 or 12, or 2
 *                            x passes)
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG on a NULL pointer, or the T4JUMP
 *          activity's status
 **********************************************************************/
agc_status_t agc_t4rupt(uint16_t *next_cs)
{
    if (!AGC_ASSERT(next_cs != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (t4_quick_left > 0)
    {
        *next_cs = t4_quikdsp();

        return (AGC_STATUS_OK);
    }

    t4_phase = (t4_phase > 0) ? (t4_phase - 1) : (T4_PHASES - 1); /* CCS  */
    *next_cs = t4_display_pass();
    t4_proceede();

    if (!AGC_ASSERT(*next_cs >= T4_20MS_CS) ||

        !AGC_ASSERT(t4_seq_ok()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (t4_jump(t4_phase));
}

/**********************************************************************
 * T4RUPT entry (T4RUPT_PROGRAM.agc p.133 CM, p.155 LM; vector from
 * INTERRUPT_LEAD_INS) -- run T4RUPT and reload TIME4 so it overflows
 * after the returned delay (the AGC's TS TIME4 of 20MRUPT / 120MRUPT)
 *
 * RETURNS: the status of agc_t4rupt, or BAD_ARG for a delay outside TIME4
 **********************************************************************/
agc_status_t agc_t4rupt_rupt(void)
{
    uint16_t     dt = 0;              /* TIME4 delay, cs */
    agc_status_t st = AGC_STATUS_OK;  /* T4RUPT result   */

    if (!AGC_ASSERT(t4_seq_ok()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    st = agc_t4rupt(&dt);

    if (!AGC_ASSERT(dt > 0u && dt < T4_TIME4_OVF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rupt_set_time(4, (uint16_t) (T4_TIME4_OVF - dt));

    return (st);
}

/**********************************************************************
 * STARTSUB (FRESH_START C p.191): TS DSRUPTSW with +0.
 *
 * RETURNS: AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_t4_startsub_reset(void)
{
    t4_phase       = 0;
    t4_quick_left  = 0;
    t4_quick_wrote = false;

    if (!AGC_ASSERT(t4_quick_left == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SLAP1 (FRESH_START C p.183): CS PRIO12 / TS DSRUPTSW -- twenty quick
 * passes to flush the freshly blanked display (magnitude bit 14 clear:
 * the first pass tries to display).
 *
 * RETURNS: AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_t4_fresh_start_flush(void)
{
    t4_phase       = 0;
    t4_quick_left  = T4_FLUSH_PASSES;
    t4_quick_wrote = false;

    if (!AGC_ASSERT(t4_quick_left == T4_FLUSH_PASSES))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* ======================================================================
 * IMU monitors (pp.139-159)
 * ==================================================================== */

/**********************************************************************
 * SETISSW (p.157): ISS warning lamp (channel 11 bit 1) from IMODES30 fail
 * bits 13, 12, 10 (0 = failure, inverted inputs) and their inhibits 4, 3,
 * 1.  On a failure VARALARM is given the fail bits minus one -- the famous
 * 07777 (IMU fail), 03777 (ICDU fail), 14777 (PIPA fail) codes.
 *
 * RETURNS: AGC_STATUS_ALARM when the ISS warning was turned on (VARALARM
 *          given), AGC_STATUS_OK otherwise, BAD_ARG on a bad IMODES30
 **********************************************************************/
agc_status_t agc_t4_setissw(void)
{
    uint16_t im30 = agc_erase_imodes30(); /* IMODES30 bits */

    if (!AGC_ASSERT(im30 <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t inhib = (uint16_t) ((im30 & OCT15) << 9); /* MP BIT10        */
    uint16_t fail  = /* 0 = failure, bits 13,12,10 */
                     (uint16_t) (~(uint32_t) (im30 | inhib) & OCT15000);

    if (fail != 0u)
    { /* ISSWON          */
        agc_exec_varalarm((agc_alarm_code_t) (fail - 1u));
        agc_status_t st = AGC_STATUS_ALARM; /* VARALARM given */

        agc_chan_or((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT1);

        if (!AGC_ASSERT((agc_chan_read((int) AGC_CHAN_DSALMOUT) & BIT1) != 0u))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        return (st);
    }

    if (!agc_t4_lamptest())
    { /* ISSWOFF         */
        chan_clear((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT1);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CAGESUB2 (p.158): flags for caging / turn-on; inhibit ISS warnings and
 * the DAP auto and hold modes.
 *
 * RETURNS: AGC_STATUS_OK, or BAD_ARG if the flags did not stick
 **********************************************************************/
agc_status_t agc_t4_cagesub2(void)
{
    agc_erase_set_imodes30((uint16_t) (agc_erase_imodes30() | OCT75));
    agc_erase_set_imodes33((uint16_t) (agc_erase_imodes33() | BIT6));

    if (!AGC_ASSERT((agc_erase_imodes30() & OCT75) == OCT75) ||
        !AGC_ASSERT((agc_erase_imodes33() & BIT6) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CAGESUB1 (p.158): NO ATT lamp on (DSPTAB +11D bit 4, bit 15 = send).
 *
 * RETURNS: the CAGESUB2 status
 **********************************************************************/
static agc_status_t t4_cagesub1(void)
{
    uint16_t w = agc_erase_dsptab(LAMPWORD); /* DSPTAB +11D */

    if (!AGC_ASSERT(w <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    set_lampword((uint16_t) (w | OC40010)); /* CS / MASK / ADS = OR */

    if (!AGC_ASSERT((agc_erase_dsptab(LAMPWORD) & OC40010) == OC40010))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_t4_cagesub2());
}

/**********************************************************************
 * CAGESUB (p.158): outbits for turn-on or cage: error counter disabled,
 * delay-complete removed, zero + coarse sent.
 *
 * RETURNS: the CAGESUB2 status, or BAD_ARG if channel 12 did not take the
 *          bits
 **********************************************************************/
agc_status_t agc_t4_cagesub(void)
{
    chan_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT15_6);
    agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BITS4_5);

    if (!AGC_ASSERT((agc_chan_read((int) AGC_CHAN_CHAN12) & BITS4_5) ==
                    BITS4_5) ||
        !AGC_ASSERT((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT15_6) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (t4_cagesub1());
}

/**********************************************************************
 * TLIM (p.149): temp caution lamp (channel 11 bit 4) follows channel 30
 * bit 15 (IMODES30 negative = temperature out of limits).
 **********************************************************************/
static void t4_tlim(void)
{
    uint16_t im30 = agc_erase_imodes30(); /* IMODES30 bits */

    if (!AGC_ASSERT(im30 <= WORD15))
    {
        return;
    }

    if ((im30 & BIT15) != 0u)
    {
        agc_chan_or((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT4);

        if (!AGC_ASSERT((agc_chan_read((int) AGC_CHAN_DSALMOUT) & BIT4) != 0u))
        {
            return;
        }
    }
    else if (!agc_t4_lamptest())
    { /* TEMPOK                  */
        chan_clear((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT4);
    }
}

/**********************************************************************
 * ITURNON2 (p.151): request ISS initialisation after one more sample.
 **********************************************************************/
static void t4_iturnon2(void)
{
    agc_erase_set_imodes30((uint16_t) (agc_erase_imodes30() | BIT7));

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_erase_set_radmodes((uint16_t) RRINIT); /* LM: CAF RRINIT     */
    }

    if (!AGC_ASSERT((agc_erase_imodes30() & BIT7) != 0u) ||
        !AGC_ASSERT((agc_state_vehicle() != AGC_VEHICLE_LM) ||
                    (agc_erase_radmodes() == RRINIT)))
    {
        return;
    }
}

/**********************************************************************
 * ITURNON (p.150): channel 30 bit 14 (ISS turn-on request) changed.
 **********************************************************************/
static void t4_iturnon(void)
{
    uint16_t im30 = agc_erase_imodes30(); /* IMODES30 bits */

    if (!AGC_ASSERT(im30 <= WORD15))
    {
        return;
    }

    if ((im30 & BIT2) != 0u)
    {
        return; /* failed delay: wait for ENDTNON   */
    }

    if ((im30 & BIT14) == 0u)
    {
        t4_iturnon2(); /* just on                          */

        return;
    }

    if ((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT15) != 0u)
    {
        return; /* delay complete already sent      */
    }

    agc_erase_set_imodes30((uint16_t) (im30 + BIT2));
    agc_alarm(ALARM_ISS_DELAY_FAIL);

    if (!AGC_ASSERT((agc_erase_imodes30() & BIT2) != 0u))
    {
        return;
    }
}

/**********************************************************************
 * ISSZERO (p.144): zero the ICDUs; UNZ2 follows in 320 ms.
 **********************************************************************/
static void t4_isszero(void)
{
    if (!AGC_ASSERT(agc_erase_imodes30() <= WORD15))
    {
        return;
    }

    agc_imu_noattoff();
    agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT5);
    agc_imu_zeroicdu();
    call_wait(UNZ2_DELAY_CS, AGC_TASK_UNZ2);

    if (!AGC_ASSERT((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT5) != 0u))
    {
        return; /* ISS CDU zero not sent */
    }
}

/**********************************************************************
 * IMUCAGE (p.152): channel 30 bit 11 changed.  Returns true when the
 * remaining channel 30 changes are abandoned (ISSZERO exits to C33TEST).
 *
 * ARGUMENTS:
 *   newbit  uint16_t  I   new channel 30 bit 11 (nonzero = cage button
 *                         released)
 *
 * RETURNS: true when the scan ends (ISSZERO exits to C33TEST)
 **********************************************************************/
static bool t4_imucage(uint16_t newbit)
{
    if (newbit != 0u)
    { /* button released: ISSZERO       */
        t4_isszero();

        return (true);
    }

    chan_clear((int) AGC_CHAN_CHAN14, (uint16_t) OCT77000); /* pulse trains */
    chan_clear((int) AGC_CHAN_CHAN12, (uint16_t) OCT272);

    /* Engine off on an IMU cage: CM WAND DSALMOUT bit 13; LM clears ENGONFLG
     * and forces channel 11 bit 14 = 1, bit 13 = 0.                       */
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        chan_clear((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT13);
    }
    else
    {
        agc_flag_clear((agc_flag_id_t) AGC_FLAG_ENGONFLG);
        uint16_t ch11 =
            agc_chan_read((int) AGC_CHAN_DSALMOUT); /* DSALMOUT bits */

        agc_chan_write((int) AGC_CHAN_DSALMOUT,
                       (uint16_t) ((ch11 & ~(uint32_t) PRIO30) + BIT14));
    }

    agc_status_t st = t4_cagesub1(); /* CAGESUB1 result */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (false);
    }

    agc_imu_rndrefdr(); /* track, REFSMMAT, drift flags    */
    st = agc_imu_clear_cmds(); /* -0 into CDUXCMD..Z, GYROCMD     */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (false);
    }

    chan_clear((int) AGC_CHAN_CHAN14, (uint16_t) OCT740); /* deselect gyros */

    return (false);
} /* t4_imucage */

/**********************************************************************
 * IMUOP (p.154): channel 30 bit 9 (ISS operate) changed.
 *
 * ARGUMENTS:
 *   newbit  uint16_t  I   new channel 30 bit 9 (0 = ISS operate on)
 **********************************************************************/
static void t4_imuop(uint16_t newbit)
{
    if (newbit == 0u)
    { /* IMUOP2       */
        if ((agc_erase_imodes30() & BIT2) == 0u)
        {
            t4_iturnon2();
        }

        return;
    }

    /* disable DAP  */
    agc_erase_set_imodes33((uint16_t) (agc_erase_imodes33() | BIT6));
    agc_imu_rndrefdr();
    bool in_use = /* IMUSE before the knock-down */
                  agc_flag_test((agc_flag_id_t) AGC_FLAG_IMUSE);

    agc_flag_clear((agc_flag_id_t) AGC_FLAG_RNDVZFLG);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_IMUSE);

    if (in_use)
    {
        agc_alarm(ALARM_ISS_OFF_IN_USE);
    }

    if (!AGC_ASSERT(!agc_flag_test((agc_flag_id_t) AGC_FLAG_IMUSE)) ||
        !AGC_ASSERT((agc_erase_imodes33() & BIT6) != 0u))
    {
        return;
    }
}

/**********************************************************************
 * IFAILJMP (p.159): one changed channel 30 bit.  bit is the AGC bit
 * number.  Returns true when the scan must stop (ISSZERO).
 *
 * ARGUMENTS:
 *   bit     int       I   AGC bit number of the changed bit, 9..14
 *   newbit  uint16_t  I   its new value in IMODES30 (masked, not shifted)
 *
 * RETURNS: true when the scan must stop (ISSZERO), or on a bad bit number
 **********************************************************************/
static bool t4_ifailjmp(int bit, uint16_t newbit)
{
    if (!AGC_ASSERT(bit >= 9 && bit <= 14))
    {
        return (true);
    }

    switch (bit)
    {
        case 14:
            t4_iturnon();

            return (false);
        case 13: /* IMUFAIL  EQUALS SETISSW         */
        case 12: /* ICDUFAIL EQUALS SETISSW         */
        {
            agc_status_t st = agc_t4_setissw(); /* SETISSW result */

            if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
            {
                return (true);
            }
        }

            return (false);
        case 11:
            return (t4_imucage(newbit));
        case 9:
            t4_imuop(newbit);

            return (false);
        default: /* bit 10 is not sampled here      */
            return (false);
    }
}

/**********************************************************************
 * IMUMON (p.139): process changes of channel 30 bits 15,14,13,12,11,9,
 * bit 15 first then from bit 14 down (NXTIFBIT), then TNONTEST.
 *
 * RETURNS: the status of TNONTEST or C33TEST (the rest of the chain),
 *          BAD_ARG on a bad IMODES30
 **********************************************************************/
agc_status_t agc_t4_imumon(void)
{
    uint16_t im30    = agc_erase_imodes30(); /* IMODES30 bits */
    uint16_t changes = /* changed bits of chan 30 */
                       (uint16_t) ((im30 ^
                                    agc_chan_read((int) AGC_CHAN_CHAN30)) &
                                   IM30_RDMSK);

    if (!AGC_ASSERT(im30 <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (changes != 0u)
    {
        agc_erase_set_imodes30((uint16_t) (im30 ^ changes));

        if (!AGC_ASSERT(((agc_erase_imodes30() ^
                          agc_chan_read((int) AGC_CHAN_CHAN30)) &
                         IM30_RDMSK) == 0u))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        if ((changes & BIT15) != 0u)
        {
            t4_tlim();
        }

        bool stop = false; /* ISSZERO ends the scan */

        for (int bit = 14; bit >= 9 && !stop; bit--)
        {
            uint16_t mask = (uint16_t) (1u << (bit - 1)); /* bit under test */

            if ((changes & mask) != 0u)
            {
                stop =
                    t4_ifailjmp(bit, (uint16_t) (agc_erase_imodes30() & mask));
            }
        }

        if (stop)
        {
            return (agc_t4_c33test());
        }
    }

    return (agc_t4_tnontest());
} /* agc_t4_imumon */

/**********************************************************************
 * PROCTNON (p.142): act on the turn-on request one sample after it came.
 **********************************************************************/
static void t4_proctnon(void)
{
    uint16_t im30 = /* IMODES30 less BITS7&8 */
                    (uint16_t) (agc_erase_imodes30() & ~(uint32_t) OCT300);

    agc_erase_set_imodes30(im30);

    if ((im30 & BIT14) != 0u)
    { /* OPONLY: operate only */
        if ((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT4) != 0u ||
            agc_flag_test((agc_flag_id_t) AGC_FLAG_IMUSE))
        {
            return; /* coarse align (gimbal lock) / in use */
        }

        agc_status_t st_cage = agc_t4_cagesub2(); /* CAGESUB2 result */

        if (!AGC_ASSERT(st_cage == AGC_STATUS_OK))
        {
            return;
        }

        t4_isszero();

        return;
    }

    if ((im30 & BIT9) != 0u)
    {
        agc_alarm(ALARM_TURNON_NO_OP); /* turn-on, no operate  */
    }

    agc_status_t st = agc_t4_cagesub(); /* CAGESUB result */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }

    call_wait((uint16_t) NINETY_SECS_CS, AGC_TASK_ENDTNON);
}

/**********************************************************************
 * TNONTEST (p.141): IMODES30 bit 7 = turn-on seen, bit 8 = one sample
 * waited.
 *
 * RETURNS: the C33TEST status (the rest of the chain), BAD_ARG on a bad
 *          IMODES30
 **********************************************************************/
agc_status_t agc_t4_tnontest(void)
{
    uint16_t im30 = agc_erase_imodes30(); /* IMODES30 bits */

    if (!AGC_ASSERT(im30 <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((im30 & BIT7) != 0u)
    {
        if ((im30 & BIT8) != 0u)
        {
            t4_proctnon();
        }
        else
        {
            agc_erase_set_imodes30((uint16_t) (im30 + BIT8));
        }
    }

    /* bit 8 (one sample waited) is only ever set together with bit 7 */
    if (!AGC_ASSERT(((agc_erase_imodes30() & BIT8) == 0u) ||
                    ((agc_erase_imodes30() & BIT7) != 0u)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_t4_c33test());
}

/**********************************************************************
 * C33JMP: PIPFAIL (p.155), DNTMFAST / UPTMFAST (p.156).
 *
 * ARGUMENTS:
 *   bit     int       I   AGC bit number of the changed channel 33 bit,
 *                         11..13
 *   newbit  uint16_t  I   its new value in IMODES33 (masked, not shifted)
 **********************************************************************/
static void t4_c33jmp(int bit, uint16_t newbit)
{
    if (!AGC_ASSERT(bit >= 11 && bit <= 13))
    {
        return;
    }

    if (bit == 12 || bit == 11)
    {
        if (newbit == 0u)
        {
            agc_alarm((bit == 12) ? ALARM_DNLINK_FAST : /* DNTMFAST */
                      ALARM_UPLINK_FAST);                /* UPTMFAST */
        }

        return;
    }

    /* PIPFAIL: IMODES30 bit 10 follows channel 33 bit 13.                 */
    uint16_t pip  = (newbit != 0u) ? BIT10 : 0u; /* CCS A / CAF BIT10 */
    uint16_t im30 = /* bit 10 = the new PIPA bit */
                    (uint16_t) ((agc_erase_imodes30() & ~(uint32_t) BIT10) |
                                pip);

    agc_erase_set_imodes30(im30);
    agc_status_t sst = agc_t4_setissw(); /* SETISSW result */

    if (!AGC_ASSERT(sst != AGC_STATUS_BAD_ARG))
    {
        return;
    }

    im30 = agc_erase_imodes30();

    if ((im30 & BIT1) != 0u && (im30 & OCT1720) == 0u)
    {
        agc_alarm(ALARM_PIPA_FAIL); /* no ISS warning, operating, not caged */
    }
}

/**********************************************************************
 * C33TEST (p.145): channel 33 flip-flops 13,12,11 (read, then reset by
 * the WAND write pulse), changes processed from bit 13 down.
 *
 * RETURNS: the GLOCKMON status, BAD_ARG on a bad IMODES33
 **********************************************************************/
agc_status_t agc_t4_c33test(void)
{
    uint16_t now = /* chan 33 flip-flop bits */
                   (uint16_t) (agc_chan_read((int) AGC_CHAN_CHAN33) &
                               IM33_RDMSK);

    agc_chan_and((int) AGC_CHAN_CHAN33, IM33_RDMSK); /* WAND: reset FFs */
    uint16_t im33    = agc_erase_imodes33(); /* IMODES33 bits */
    uint16_t changes =
        (uint16_t) ((im33 & IM33_RDMSK) ^ now); /* changed bits of chan 33 */

    if (!AGC_ASSERT(im33 <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (changes != 0u)
    {
        agc_erase_set_imodes33((uint16_t) (im33 ^ changes));

        if (!AGC_ASSERT((agc_erase_imodes33() & IM33_RDMSK) == now))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        for (int bit = 13; bit >= 11; bit--)
        {
            uint16_t mask = (uint16_t) (1u << (bit - 1)); /* bit under test */

            if ((changes & mask) != 0u)
            {
                t4_c33jmp(bit, (uint16_t) (agc_erase_imodes33() & mask));
            }
        }
    }

    return (agc_t4_glockmon());
}

/**********************************************************************
 * SETGLOCK / GLINVERT / GLAMPTST (p.148): make the gimbal lock lamp
 * (DSPTAB +11D bit 6) agree with want_on.
 *
 * ARGUMENTS:
 *   want_on  bool  I   true to light the gimbal lock lamp
 **********************************************************************/
static void t4_setglock(bool want_on)
{
    uint16_t w     = agc_erase_dsptab(LAMPWORD); /* DSPTAB +11D */
    bool     is_on = (w & BIT6) != 0u; /* gimbal lock lamp lit */

    if (!AGC_ASSERT(w <= WORD15))
    {
        return;
    }

    if (is_on == want_on)
    {
        return; /* GLOCKOK          */
    }

    if (is_on && agc_t4_lamptest())
    {
        return; /* lamp test        */
    }

    if (!is_on && (agc_erase_imodes30() & BIT6) != 0u)
    {
        return; /* IMU being caged  */
    }

    /* GLINVERT: CS / MASK BIT6 / AD BIT15 / XCH / MASK OCT37737 / ADS */
    set_lampword((uint16_t) (((w ^ BIT6) & ~(uint32_t) BIT15) | BIT15));

    if (!AGC_ASSERT(((agc_erase_dsptab(LAMPWORD) & BIT6) != 0u) == want_on))
    {
        return;
    }
}

/**********************************************************************
 * GLOCKMON (p.147): middle gimbal angle (CDUZ) above 70 deg -> gimbal lock
 * lamp; above 85 deg -> coarse align (SETCOARS) to stop gimbal runaway.
 *
 * RETURNS: AGC_STATUS_OK (GLOCKOK = RESUME), BAD_ARG on a bad CDUZ word
 **********************************************************************/
agc_status_t agc_t4_glockmon(void)
{
    /* Magnitude of a CDU counter word as CCS sees it (ones' complement).     */
    uint16_t cdu = agc_erase_cduz(); /* middle gimbal CDU, 2s comp */

    if (!AGC_ASSERT(cdu <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* |CDUZ| as CCS sees it (ones' complement), 2^14 = pi */
    int mag = ((cdu & CDU_SIGN) != 0u) ? (int) (WORD15 - cdu) : /* < 0 */
              (int) cdu;

    if (!AGC_ASSERT(mag >= 0 && mag <= 037777))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool lamp = false; /* desired lamp state */

    if (mag > 0 && (mag - 1) - MINUS_70DEG > 0)
    {                /* GLOCKCHK        */
        lamp = true; /* NOGIMRUN        */

        if ((mag - 1) - MINUS_70DEG - MINUS_15DEG > 0 &&
            (agc_chan_read((int) AGC_CHAN_CHAN12) & BIT4) == 0u)
        {
            agc_imu_setcoars();
            call_wait(CA_ECE_DELAY_CS, AGC_TASK_CA_ECE);
        }
    }

    t4_setglock(lamp);

    if (!AGC_ASSERT(mag >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* ---- the T4RUPT waitlist tasks ---------------------------------------- */

/**********************************************************************
 * UNZ2 (p.143): remove zero and coarse; ISSUP in 10.24 s.
 *
 * RETURNS: the VARDELAY status (ISSUP next)
 **********************************************************************/
static agc_status_t t4_unz2(void)
{
    agc_imu_zeroicdu();
    chan_clear((int) AGC_CHAN_CHAN12, (uint16_t) BITS4_5);
    agc_status_t st = /* VARDELAY result */
                      agc_waitlist_vardelay((uint16_t) ISSUP_DELAY_CS,
                                            AGC_TASK_ISSUP);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG) ||
        !AGC_ASSERT((agc_chan_read((int) AGC_CHAN_CHAN12) & BITS4_5) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * ENDTNON (p.142): end of the 90 s ISS turn-on delay.
 *
 * RETURNS: AGC_STATUS_OK (TASKOVER), the VARDELAY status (RETNON, UNZ2)
 *          or the IMUBAD status (failed turn-on)
 **********************************************************************/
static agc_status_t t4_endtnon(void)
{
    uint16_t old = agc_erase_imodes30(); /* IMODES30 before ENDTNON */

    if (!AGC_ASSERT(old <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_imodes30((uint16_t) (old & ~(uint32_t) BIT2));

    if (!AGC_ASSERT((agc_erase_imodes30() & BIT2) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((old & BIT2) == 0u)
    {                                                     /* ENDTNON2         */
        /* delay complete  */
        agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT15);
        agc_imu_noattoff();

        return (t4_unz2());
    }

    if ((old & BIT14) == 0u)
    { /* RETNON           */
        return (agc_waitlist_vardelay((uint16_t) NINETY_SECS_CS,
                                      AGC_TASK_ENDTNON));
    }

    if (!agc_flag_test((agc_flag_id_t) AGC_FLAG_IMUSE))
    {
        return (AGC_STATUS_OK); /* TASKOVER         */
    }

    return (agc_imu_imubad()); /* failed turn-on   */
}

/**********************************************************************
 * ISSUP (p.143): caging and fail inhibits removed, DAP enabled; LM also
 * starts free-fall IMU compensation (DRIFTFLG, 1/PIPADT = TIME1).
 *
 * RETURNS: AGC_STATUS_OK (TASKOVER), BAD_ARG when SETISSW fails
 **********************************************************************/
static agc_status_t t4_issup(void)
{
    agc_erase_set_imodes30((uint16_t) (agc_erase_imodes30() &
                                       ~(uint32_t) OCT54));
    agc_erase_set_imodes33((uint16_t) (agc_erase_imodes33() &
                                       ~(uint32_t) BIT6));

    if (!AGC_ASSERT((agc_erase_imodes30() & OCT54) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM &&
        !agc_flag_test((agc_flag_id_t) AGC_FLAG_DRIFTFLG))
    {
        agc_flag_set((agc_flag_id_t) AGC_FLAG_DRIFTFLG);
        agc_imu_set_pipa_interval(agc_erase_time1());
    }

    agc_status_t sst = agc_t4_setissw(); /* SETISSW result */

    if (!AGC_ASSERT(sst != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    chan_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT15);
    call_wait((uint16_t) FOUR_SECS_CS, AGC_TASK_PFAILOK);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (exec's task dispatch for the T4RUPT tasks ENDTNON, UNZ2,
 * ISSUP p.142-143 and the CM optics tasks ENDZOPT, ENDZOPT2)
 *
 * ARGUMENTS:
 *   task  agc_task_id_t  I   the waitlist task that came due
 *
 * RETURNS: the task's status, NOT_FOUND for a task that is not a T4RUPT
 *          one
 **********************************************************************/
agc_status_t agc_t4_task(agc_task_id_t task)
{
    if (!AGC_ASSERT(task != AGC_TASK_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (task == AGC_TASK_ENDTNON)
    {
        return (t4_endtnon());
    }

    if (task == AGC_TASK_UNZ2)
    {
        return (t4_unz2());
    }

    if (task == AGC_TASK_ISSUP)
    {
        return (t4_issup());
    }

    if (task == AGC_TASK_ENDZOPT || task == AGC_TASK_ENDZOPT2)
    {
        if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        return (agc_t4_optics_task(task));
    }

    return (AGC_STATUS_NOT_FOUND);
}
