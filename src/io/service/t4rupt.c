/*
 * t4rupt.c -- T4RUPT_PROGRAM.agc, the part common to both vehicles.
 *
 * Sources: Comanche055 T4RUPT_PROGRAM.agc pp. 133-159,
 *          Luminary099 T4RUPT_PROGRAM.agc pp. 155-181.
 *
 * T4RUPT comes every 120 ms (TIME4). Each pass:
 *   CDRVE    -- send the DSPTAB +11D lamp relay word or one DSKY relay word
 *               (DSPOUT is io-pinball's agc_dsky_dspout());
 *   HANG20   -- if a relay word was sent, the next five T4RUPTs come 20 ms
 *               apart (QUIKDSP), alternating relays-off and a further relay
 *               word, so the DSKY is refreshed quickly;
 *   PROCEEDE -- the PRO button (channel 32 bit 14), edge detected;
 *   T4JUMP   -- one of eight activities by the phase counter (0.96 s cycle):
 *               CSM  OPTDRIVE, OPTMON, IMUMON, -, OPTDRIVE, OPTMON, IMUMON, -
 *               LM   RCSMONIT, RRAUTCHK, IMUMON, DAPT4S, (repeat)
 *   IMUMON -> TNONTEST -> C33TEST -> GLOCKMON  (the ISS monitors, 480 ms).
 * The optics monitor/drive (CSM) are in src/io/service/t4rupt_csm.c, the radar
 * monitor and gimbal-to-pilot matrix (LM) in src/io/service/t4rupt_lm.c.
 *
 * DEVIATION (DSRUPTSW): the AGC packs the phase counter (7..0) and the
 * quick-display sequence into one ones'-complement word (negative =
 * quick passes pending: magnitude bit 14 = "relays written last pass",
 * magnitude / BIT9 = passes left, low bits = phase).  Here the three
 * fields are kept separately (phase, quick_left, quick_wrote); the
 * sequencing is the same.
 *
 * DEVIATION (TIME4): agc_t4rupt() returns the delay to the next T4RUPT in
 * centiseconds; agc_t4rupt_rupt() writes it into TIME4 as the AGC words
 * 20MRUPT (37776, 2 cs) and 120MRUPT (16372, 12 cs) do.  The NODSPY path's
 * repeated "ADS TIME4" is modelled as its intended effect, 20 ms per
 * skipped quick pass.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service_internal.h"
#include "agc/lm/dap.h"
#include "sysmisc_ext.h"

#define BIT1   000001u
#define BIT2   000002u
#define BIT4   000010u
#define BIT5   000020u
#define BIT6   000040u
#define BIT7   000100u
#define BIT8   000200u
#define BIT9   000400u
#define BIT10  001000u
#define BIT11  002000u
#define BIT13  010000u
#define BIT14  020000u
#define BIT15  040000u
#define WORD15 077777u
#define LOW11  003777u

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
#define OCT54           000054u
#define OCT75           000075u
#define OCT272          000272u
#define OCT300          000300u /* BITS7&8                             */
#define OCT1720         001720u
#define OCT740          000740u
#define OCT77000        077000u
#define OC40010         040010u
#define BIT15_6         040040u /* BIT15+6 (CM) / BITS6&15 (LM)        */
#define BITS4_5         000030u
#define PRIO30          030000u
#define RRINIT          000102u /* LM RRINIT, RADMODES at turn-on      */
#define NINETY_SECS_CS  9000u   /* 90SECS DEC 9000                      */
#define FOUR_SECS_CS    400u    /* 4SECS                                */
#define UNZ2_DELAY_CS   32u     /* BIT6: "wait 300 ms for AGS"          */
#define ISSUP_DELAY_CS  1024u   /* BIT11: wait 10 s for the counters    */
#define CA_ECE_DELAY_CS 6u      /* SIX: error counters in 60 ms         */
#define MINUS_70DEG     6371    /* -70DEGS DEC -.38888 (half-revs 2^14) */
#define MINUS_15DEG     1365    /* -15DEGS DEC -.08333                  */
#define CDU_SIGN        040000u
#define LAMPWORD        11 /* DSPTAB +11D                          */

/* Alarm codes raised here (octal, as in the listing).                     */
#define ALARM_ISS_DELAY_FAIL \
    ((agc_alarm_code_t) 00207) /* turn-on req dropped */
#define ALARM_PIPA_FAIL      ((agc_alarm_code_t) 00212)
#define ALARM_TURNON_NO_OP   ((agc_alarm_code_t) 00213)
#define ALARM_ISS_OFF_IN_USE ((agc_alarm_code_t) 00214)
#define ALARM_DNLINK_FAST    ((agc_alarm_code_t) 01105)
#define ALARM_UPLINK_FAST    ((agc_alarm_code_t) 01106)

/* DSRUPTSW, see the DEVIATION above.                                      */
static int  t4_phase;       /* RUPTREG1 / low part of DSRUPTSW, 0..7     */
static int  t4_quick_left;  /* quick (20 ms) passes still to come        */
static bool t4_quick_wrote; /* last quick pass wrote a relay word        */

/* ---- small helpers -------------------------------------------------- */

static uint16_t w15(uint32_t v)
{
    (void) AGC_ASSERT(v <= 0xFFFFFFu);
    return (uint16_t) (v & WORD15);
}

static void chan_or(int chan, uint16_t mask)
{
    agc_status_t st = agc_chan_or(chan, mask);
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    (void) AGC_ASSERT(mask <= WORD15);
}

static void chan_clear(int chan, uint16_t bits) /* WAND with ~bits  */
{
    agc_status_t st = agc_chan_and(chan, w15(~(uint32_t) bits));
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    (void) AGC_ASSERT(bits <= WORD15);
}

static void raise_alarm(agc_alarm_code_t code)
{
    agc_status_t st = agc_alarm(code);
    (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    (void) AGC_ASSERT(code != 0u);
}

static void set_lampword(uint16_t w)
{
    agc_status_t st = agc_erase_set_dsptab(LAMPWORD, w);
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    (void) AGC_ASSERT(w <= WORD15);
}

static void im30_set(uint16_t v)
{
    (void) AGC_ASSERT(v <= WORD15);
    agc_erase_set_imodes30(v);
}

static void im33_set(uint16_t v)
{
    (void) AGC_ASSERT(v <= WORD15);
    agc_erase_set_imodes33(v);
}

static void call_wait(uint16_t dt, agc_task_id_t task)
{
    agc_status_t st = agc_waitlist_call(dt, task);
    (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    (void) AGC_ASSERT(dt > 0u);
}

/* ---- LAMPTEST (p.159): true when a lamp test is in progress
 * (IMODES33 bit 1); the AGC returns to L+1 then, L+2 otherwise.           */
bool agc_t4_lamptest(void)
{
    uint16_t m = agc_erase_imodes33();
    (void) AGC_ASSERT(m <= WORD15);
    return (m & BIT1) != 0u;
}

/* ---- DSPTAB +11D lamp relay word, CDRVE (p.134) ------------------------ */

/* CDRVE: true when the lamp word was pending (negative) and has been sent. */
static bool t4_cdrve(void)
{
    uint16_t w = agc_erase_dsptab(LAMPWORD);
    (void) AGC_ASSERT(w <= WORD15);
    if ((w & BIT15) == 0u)
    {
        return false; /* CCS: + -> TC DSPOUT          */
    }
    uint16_t lamps = (uint16_t) (w & LOW11);
    set_lampword(lamps); /* replace positively           */
    agc_status_t st =
        agc_chan_write((int) AGC_CHAN_OUT0, (uint16_t) (lamps + RELTAB11));
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    return true;
}

/* HANG20 (p.135): five quick passes follow, TIME4 = 20 ms.               */
static uint16_t t4_hang20(void)
{
    t4_quick_left  = T4_QUICK_PASSES;
    t4_quick_wrote = true;
    (void) AGC_ASSERT(t4_phase >= 0 && t4_phase < T4_PHASES);
    return T4_20MS_CS;
}

/* CDRVE / DSPOUT / NODSPOUT (pp.134-137): returns the TIME4 delay.        */
static uint16_t t4_display_pass(void)
{
    if (t4_cdrve())
    {
        return t4_hang20();
    }
    uint16_t     relay = 0;
    agc_status_t st    = agc_dsky_dspout(&relay); /* DSPOUT (io-pinball)      */
    if (st == AGC_STATUS_OK)
    {
        return t4_hang20();
    }
    (void) AGC_ASSERT(st == AGC_STATUS_NOT_FOUND);
    st = agc_chan_write((int) AGC_CHAN_OUT0, 0); /* NODSPOUT: relays off     */
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    return T4_120MS_CS;
}

/* QUIKDSP / QUIKOFF / NODSPY / SYNCT4 (pp.137-138).                      */
static uint16_t t4_quikdsp(void)
{
    (void) AGC_ASSERT(t4_quick_left > 0 && t4_quick_left <= T4_FLUSH_PASSES);
    if (t4_quick_wrote)
    { /* QUIKOFF                 */
        agc_status_t st = agc_chan_write((int) AGC_CHAN_OUT0, 0);
        (void) AGC_ASSERT(st == AGC_STATUS_OK);
        t4_quick_wrote = false;
        t4_quick_left--;
        return T4_20MS_CS;
    }
    uint16_t     relay = 0;
    agc_status_t st    = agc_dsky_dspout(&relay); /* CCS NOUT / DSPOUTSB     */
    if (st == AGC_STATUS_OK)
    {
        t4_quick_wrote = true;
        t4_quick_left--;
        return T4_20MS_CS;
    }
    st = agc_chan_write((int) AGC_CHAN_OUT0, 0); /* NODSPY                  */
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    uint16_t dt   = (uint16_t) (T4_20MS_CS * t4_quick_left); /* SYNCT4 loop  */
    t4_quick_left = 0;
    return dt;
}

/* PROCEEDE (p.136): the PRO key, channel 32 bit 14 compared with IMODES33
 * bit 14 (0 = pressed, inverted input).                                   */
static void t4_proceede(void)
{
    uint16_t im33 = agc_erase_imodes33();
    uint16_t ch32 = agc_chan_read((int) AGC_CHAN_CHAN32);
    (void) AGC_ASSERT(im33 <= WORD15 && ch32 <= WORD15);
    if (((im33 ^ ch32) & BIT14) == 0u)
    {
        return; /* no change               */
    }
    im33 = (uint16_t) (im33 ^ BIT14);
    im33_set(im33);
    if ((im33 & BIT14) == 0u)
    { /* was off -- now on       */
        agc_status_t st = agc_dsky_prockey_request(); /* NOVAC PROCKEY   */
        (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    }
}

/* T4JUMP (p.137): the once-per-0.96 s activity selected by the phase.     */
static agc_status_t t4_jump(int phase)
{
    if (!AGC_ASSERT(phase >= 0 && phase < T4_PHASES))
    {
        return AGC_STATUS_BAD_ARG;
    }
    int slot = phase & 3; /* table repeats after 4   */
    if (slot == 2)
    {
        return agc_t4_imumon();
    }
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        if (slot == 0)
        {
            return agc_t4_optics_drive(); /* OPTTEST -> OPTDRIVE     */
        }
        return (slot == 1) ? agc_t4_optics_monitor() : AGC_STATUS_OK;
    }
    if (slot == 0)
    {
        return agc_lm_dap_rcsmon(); /* RCSMONIT (dap-lm)       */
    }
    return (slot == 1) ? agc_t4_rr_monitor() : agc_t4_gimbal_matrix();
}

/* T4RUPT / NORMT4 (p.133): one interrupt.  *next_cs = TIME4 delay.       */
agc_status_t agc_t4rupt(uint16_t *next_cs)
{
    if (!AGC_ASSERT(next_cs != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (t4_quick_left > 0)
    {
        *next_cs = t4_quikdsp();
        return AGC_STATUS_OK;
    }
    t4_phase = (t4_phase > 0) ? (t4_phase - 1) : (T4_PHASES - 1); /* CCS  */
    *next_cs = t4_display_pass();
    t4_proceede();
    (void) AGC_ASSERT(*next_cs >= T4_20MS_CS);
    return t4_jump(t4_phase);
}

/* Interrupt-vector entry for exec's agc_dispatch_rupt(AGC_RUPT_T4): runs
 * T4RUPT and reloads TIME4 so that it overflows after the delay.          */
agc_status_t agc_t4rupt_rupt(void)
{
    uint16_t     dt = 0;
    agc_status_t st = agc_t4rupt(&dt);
    if (!AGC_ASSERT(dt > 0u && dt < T4_TIME4_OVF))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st4 = agc_rupt_set_time(4, (uint16_t) (T4_TIME4_OVF - dt));
    (void) AGC_ASSERT(st4 == AGC_STATUS_OK);
    return st;
}

/* STARTSUB (FRESH_START C p.191): TS DSRUPTSW with +0.                   */
agc_status_t agc_t4_startsub_reset(void)
{
    t4_phase       = 0;
    t4_quick_left  = 0;
    t4_quick_wrote = false;
    (void) AGC_ASSERT(t4_quick_left == 0);
    return AGC_STATUS_OK;
}

/* SLAP1 (FRESH_START C p.183): CS PRIO12 / TS DSRUPTSW -- twenty quick
 * passes to flush the freshly blanked display (magnitude bit 14 clear:
 * the first pass tries to display).                                      */
agc_status_t agc_t4_fresh_start_flush(void)
{
    t4_phase       = 0;
    t4_quick_left  = T4_FLUSH_PASSES;
    t4_quick_wrote = false;
    (void) AGC_ASSERT(t4_quick_left == T4_FLUSH_PASSES);
    return AGC_STATUS_OK;
}

/* ======================================================================
 * IMU monitors (pp.139-159)
 * ==================================================================== */

/* SETISSW (p.157): ISS warning lamp (channel 11 bit 1) from IMODES30 fail
 * bits 13, 12, 10 (0 = failure, inverted inputs) and their inhibits 4, 3,
 * 1.  On a failure VARALARM is given the fail bits minus one -- the famous
 * 07777 (IMU fail), 03777 (ICDU fail), 14777 (PIPA fail) codes.           */
agc_status_t agc_t4_setissw(void)
{
    uint16_t im30 = agc_erase_imodes30();
    (void) AGC_ASSERT(im30 <= WORD15);
    uint16_t inhib = (uint16_t) ((im30 & OCT15) << 9); /* MP BIT10        */
    uint16_t fail  = (uint16_t) (~(uint32_t) (im30 | inhib) & OCT15000);
    if (fail != 0u)
    { /* ISSWON          */
        agc_status_t st = agc_exec_varalarm((agc_alarm_code_t) (fail - 1u));
        (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
        chan_or((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT1);
        return st;
    }
    if (!agc_t4_lamptest())
    { /* ISSWOFF         */
        chan_clear((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT1);
    }
    return AGC_STATUS_OK;
}

/* CAGESUB2 (p.158): flags for caging / turn-on; inhibit ISS warnings and
 * the DAP auto and hold modes.                                            */
agc_status_t agc_t4_cagesub2(void)
{
    im30_set((uint16_t) (agc_erase_imodes30() | OCT75));
    im33_set((uint16_t) (agc_erase_imodes33() | BIT6));
    (void) AGC_ASSERT((agc_erase_imodes30() & OCT75) == OCT75);
    return AGC_STATUS_OK;
}

/* CAGESUB1 (p.158): NO ATT lamp on (DSPTAB +11D bit 4, bit 15 = send).    */
static agc_status_t t4_cagesub1(void)
{
    set_lampword((uint16_t) (agc_erase_dsptab(LAMPWORD) | OC40010));
    return agc_t4_cagesub2();
}

/* CAGESUB (p.158): outbits for turn-on or cage: error counter disabled,
 * delay-complete removed, zero + coarse sent.                             */
agc_status_t agc_t4_cagesub(void)
{
    chan_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT15_6);
    chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BITS4_5);
    (void) AGC_ASSERT((agc_chan_read((int) AGC_CHAN_CHAN12) & BITS4_5) ==
                      BITS4_5);
    return t4_cagesub1();
}

/* TLIM (p.149): temp caution lamp (channel 11 bit 4) follows channel 30
 * bit 15 (IMODES30 negative = temperature out of limits).                 */
static void t4_tlim(void)
{
    uint16_t im30 = agc_erase_imodes30();
    (void) AGC_ASSERT(im30 <= WORD15);
    if ((im30 & BIT15) != 0u)
    {
        chan_or((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT4);
    }
    else if (!agc_t4_lamptest())
    { /* TEMPOK                  */
        chan_clear((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT4);
    }
}

/* ITURNON2 (p.151): request ISS initialisation after one more sample.     */
static void t4_iturnon2(void)
{
    im30_set((uint16_t) (agc_erase_imodes30() | BIT7));
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_erase_set_radmodes((uint16_t) RRINIT); /* LM: CAF RRINIT     */
    }
    (void) AGC_ASSERT((agc_erase_imodes30() & BIT7) != 0u);
}

/* ITURNON (p.150): channel 30 bit 14 (ISS turn-on request) changed.       */
static void t4_iturnon(void)
{
    uint16_t im30 = agc_erase_imodes30();
    (void) AGC_ASSERT(im30 <= WORD15);
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
    im30_set((uint16_t) (im30 + BIT2));
    raise_alarm(ALARM_ISS_DELAY_FAIL);
}

/* ISSZERO (p.144): zero the ICDUs; UNZ2 follows in 320 ms.                */
static void t4_isszero(void)
{
    agc_status_t st = agc_imu_noattoff();
    (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT5);
    st = agc_imu_zeroicdu();
    (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    call_wait(UNZ2_DELAY_CS, AGC_TASK_UNZ2);
}

/* Engine off on an IMU cage: CM WAND DSALMOUT bit 13; LM clears ENGONFLG
 * and forces channel 11 bit 14 = 1, bit 13 = 0.                           */
static void t4_cage_engine_off(void)
{
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        chan_clear((int) AGC_CHAN_DSALMOUT, (uint16_t) BIT13);
        return;
    }
    agc_status_t st = agc_flag_clear((agc_flag_id_t) AGC_FLAG_ENGONFLG);
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    uint16_t ch11 = agc_chan_read((int) AGC_CHAN_DSALMOUT);
    st            = agc_chan_write((int) AGC_CHAN_DSALMOUT,
                                   (uint16_t) ((ch11 & ~(uint32_t) PRIO30) + BIT14));
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
}

/* IMUCAGE (p.152): channel 30 bit 11 changed.  Returns true when the
 * remaining channel 30 changes are abandoned (ISSZERO exits to C33TEST).   */
static bool t4_imucage(uint16_t newbit)
{
    if (newbit != 0u)
    { /* button released: ISSZERO       */
        t4_isszero();
        return true;
    }
    chan_clear((int) AGC_CHAN_CHAN14, (uint16_t) OCT77000); /* pulse trains */
    chan_clear((int) AGC_CHAN_CHAN12, (uint16_t) OCT272);
    t4_cage_engine_off();
    agc_status_t st = t4_cagesub1();
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_imu_rndrefdr(); /* track, REFSMMAT, drift flags    */
    (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    st = agc_imu_clear_cmds(); /* -0 into CDUXCMD..Z, GYROCMD     */
    (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    chan_clear((int) AGC_CHAN_CHAN14, (uint16_t) OCT740); /* deselect gyros */
    return false;
}

/* IMUOP (p.154): channel 30 bit 9 (ISS operate) changed.                  */
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
    im33_set((uint16_t) (agc_erase_imodes33() | BIT6)); /* disable DAP  */
    agc_status_t st = agc_imu_rndrefdr();
    (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    bool in_use = agc_flag_test((agc_flag_id_t) AGC_FLAG_IMUSE);
    st          = agc_flag_clear((agc_flag_id_t) AGC_FLAG_RNDVZFLG);
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_flag_clear((agc_flag_id_t) AGC_FLAG_IMUSE);
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    if (in_use)
    {
        raise_alarm(ALARM_ISS_OFF_IN_USE);
    }
}

/* IFAILJMP (p.159): one changed channel 30 bit.  bit is the AGC bit
 * number.  Returns true when the scan must stop (ISSZERO).                */
static bool t4_ifailjmp(int bit, uint16_t newbit)
{
    if (!AGC_ASSERT(bit >= 9 && bit <= 14))
    {
        return true;
    }
    switch (bit)
    {
        case 14:
            t4_iturnon();
            return false;
        case 13: /* IMUFAIL  EQUALS SETISSW         */
        case 12: /* ICDUFAIL EQUALS SETISSW         */
        {
            agc_status_t st = agc_t4_setissw();
            (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
        }
            return false;
        case 11:
            return t4_imucage(newbit);
        case 9:
            t4_imuop(newbit);
            return false;
        default: /* bit 10 is not sampled here      */
            return false;
    }
}

/* IMUMON (p.139): process changes of channel 30 bits 15,14,13,12,11,9,
 * bit 15 first then from bit 14 down (NXTIFBIT), then TNONTEST.           */
agc_status_t agc_t4_imumon(void)
{
    uint16_t im30 = agc_erase_imodes30();
    uint16_t changes =
        (uint16_t) ((im30 ^ agc_chan_read((int) AGC_CHAN_CHAN30)) & IM30_RDMSK);
    (void) AGC_ASSERT(im30 <= WORD15);
    if (changes != 0u)
    {
        im30_set((uint16_t) (im30 ^ changes));
        if ((changes & BIT15) != 0u)
        {
            t4_tlim();
        }
        bool stop = false;
        for (int bit = 14; bit >= 9 && !stop; bit--)
        {
            uint16_t mask = (uint16_t) (1u << (bit - 1));
            if ((changes & mask) != 0u)
            {
                stop =
                    t4_ifailjmp(bit, (uint16_t) (agc_erase_imodes30() & mask));
            }
        }
        if (stop)
        {
            return agc_t4_c33test();
        }
    }
    return agc_t4_tnontest();
}

/* PROCTNON (p.142): act on the turn-on request one sample after it came. */
static void t4_proctnon(void)
{
    uint16_t im30 = (uint16_t) (agc_erase_imodes30() & ~(uint32_t) OCT300);
    im30_set(im30);
    if ((im30 & BIT14) != 0u)
    { /* OPONLY: operate only */
        if ((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT4) != 0u ||
            agc_flag_test((agc_flag_id_t) AGC_FLAG_IMUSE))
        {
            return; /* coarse align (gimbal lock) / in use */
        }
        agc_status_t st = agc_t4_cagesub2();
        (void) AGC_ASSERT(st == AGC_STATUS_OK);
        t4_isszero();
        return;
    }
    if ((im30 & BIT9) != 0u)
    {
        raise_alarm(ALARM_TURNON_NO_OP); /* turn-on, no operate  */
    }
    agc_status_t st = agc_t4_cagesub();
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    call_wait((uint16_t) NINETY_SECS_CS, AGC_TASK_ENDTNON);
}

/* TNONTEST (p.141): IMODES30 bit 7 = turn-on seen, bit 8 = one sample
 * waited.                                                                 */
agc_status_t agc_t4_tnontest(void)
{
    uint16_t im30 = agc_erase_imodes30();
    (void) AGC_ASSERT(im30 <= WORD15);
    if ((im30 & BIT7) != 0u)
    {
        if ((im30 & BIT8) != 0u)
        {
            t4_proctnon();
        }
        else
        {
            im30_set((uint16_t) (im30 + BIT8));
        }
    }
    return agc_t4_c33test();
}

/* C33JMP: PIPFAIL (p.155), DNTMFAST / UPTMFAST (p.156).                   */
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
            raise_alarm(bit == 12 ? ALARM_DNLINK_FAST : ALARM_UPLINK_FAST);
        }
        return;
    }
    /* PIPFAIL: IMODES30 bit 10 follows channel 33 bit 13.                 */
    uint16_t im30 = (uint16_t) ((agc_erase_imodes30() & ~(uint32_t) BIT10) |
                                (newbit != 0u ? BIT10 : 0u));
    im30_set(im30);
    agc_status_t sst = agc_t4_setissw();
    (void) AGC_ASSERT(sst != AGC_STATUS_BAD_ARG);
    im30 = agc_erase_imodes30();
    if ((im30 & BIT1) != 0u && (im30 & OCT1720) == 0u)
    {
        raise_alarm(ALARM_PIPA_FAIL); /* no ISS warning, operating, not caged */
    }
}

/* C33TEST (p.145): channel 33 flip-flops 13,12,11 (read, then reset by
 * the WAND write pulse), changes processed from bit 13 down.              */
agc_status_t agc_t4_c33test(void)
{
    uint16_t now =
        (uint16_t) (agc_chan_read((int) AGC_CHAN_CHAN33) & IM33_RDMSK);
    chan_clear((int) AGC_CHAN_CHAN33, 0u); /* WAND CHAN33: reset FFs   */
    uint16_t im33    = agc_erase_imodes33();
    uint16_t changes = (uint16_t) ((im33 & IM33_RDMSK) ^ now);
    (void) AGC_ASSERT(im33 <= WORD15);
    if (changes != 0u)
    {
        im33_set((uint16_t) (im33 ^ changes));
        for (int bit = 13; bit >= 11; bit--)
        {
            uint16_t mask = (uint16_t) (1u << (bit - 1));
            if ((changes & mask) != 0u)
            {
                t4_c33jmp(bit, (uint16_t) (agc_erase_imodes33() & mask));
            }
        }
    }
    return agc_t4_glockmon();
}

/* Magnitude of a CDU counter word as CCS sees it (ones' complement).     */
static int cdu_ones_magnitude(uint16_t cdu)
{
    (void) AGC_ASSERT(cdu <= WORD15);
    int v = ((cdu & CDU_SIGN) != 0u) ? (int) (WORD15 - cdu) : (int) cdu;
    (void) AGC_ASSERT(v >= 0 && v <= 037777);
    return v;
}

/* SETGLOCK / GLINVERT / GLAMPTST (p.148): make the gimbal lock lamp
 * (DSPTAB +11D bit 6) agree with want_on.                                 */
static void t4_setglock(bool want_on)
{
    uint16_t w     = agc_erase_dsptab(LAMPWORD);
    bool     is_on = (w & BIT6) != 0u;
    (void) AGC_ASSERT(w <= WORD15);
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
    set_lampword((uint16_t) (((w ^ BIT6) & ~(uint32_t) BIT15) | BIT15));
}

/* GLOCKMON (p.147): middle gimbal angle (CDUZ) above 70 deg -> gimbal lock
 * lamp; above 85 deg -> coarse align (SETCOARS) to stop gimbal runaway.   */
agc_status_t agc_t4_glockmon(void)
{
    int  mag  = cdu_ones_magnitude(agc_erase_cduz());
    bool lamp = false;
    if (mag > 0 && (mag - 1) - MINUS_70DEG > 0)
    {                /* GLOCKCHK        */
        lamp = true; /* NOGIMRUN        */
        if ((mag - 1) - MINUS_70DEG - MINUS_15DEG > 0 &&
            (agc_chan_read((int) AGC_CHAN_CHAN12) & BIT4) == 0u)
        {
            agc_status_t st = agc_imu_setcoars();
            (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
            call_wait(CA_ECE_DELAY_CS, AGC_TASK_CA_ECE);
        }
    }
    t4_setglock(lamp);
    (void) AGC_ASSERT(mag >= 0);
    return AGC_STATUS_OK;
}

/* ---- the T4RUPT waitlist tasks ---------------------------------------- */

/* UNZ2 (p.143): remove zero and coarse; ISSUP in 10.24 s.                 */
static agc_status_t t4_unz2(void)
{
    agc_status_t st = agc_imu_zeroicdu();
    (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    chan_clear((int) AGC_CHAN_CHAN12, (uint16_t) BITS4_5);
    st = agc_waitlist_vardelay((uint16_t) ISSUP_DELAY_CS, AGC_TASK_ISSUP);
    (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    return st;
}

/* ENDTNON (p.142): end of the 90 s ISS turn-on delay.                     */
static agc_status_t t4_endtnon(void)
{
    uint16_t old = agc_erase_imodes30();
    im30_set((uint16_t) (old & ~(uint32_t) BIT2));
    if ((old & BIT2) == 0u)
    {                                                     /* ENDTNON2         */
        chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT15); /* delay complete  */
        agc_status_t st = agc_imu_noattoff();
        (void) AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
        return t4_unz2();
    }
    if ((old & BIT14) == 0u)
    { /* RETNON           */
        return agc_waitlist_vardelay((uint16_t) NINETY_SECS_CS,
                                     AGC_TASK_ENDTNON);
    }
    if (!agc_flag_test((agc_flag_id_t) AGC_FLAG_IMUSE))
    {
        return AGC_STATUS_OK; /* TASKOVER         */
    }
    return agc_imu_imubad(); /* failed turn-on   */
}

/* ISSUP (p.143): caging and fail inhibits removed, DAP enabled; LM also
 * starts free-fall IMU compensation (DRIFTFLG, 1/PIPADT = TIME1).         */
static agc_status_t t4_issup(void)
{
    im30_set((uint16_t) (agc_erase_imodes30() & ~(uint32_t) OCT54));
    im33_set((uint16_t) (agc_erase_imodes33() & ~(uint32_t) BIT6));
    if (agc_state_vehicle() == AGC_VEHICLE_LM &&
        !agc_flag_test((agc_flag_id_t) AGC_FLAG_DRIFTFLG))
    {
        agc_status_t st = agc_flag_set((agc_flag_id_t) AGC_FLAG_DRIFTFLG);
        (void) AGC_ASSERT(st == AGC_STATUS_OK);
        agc_imu_set_pipa_interval(agc_erase_time1());
    }
    agc_status_t sst = agc_t4_setissw();
    (void) AGC_ASSERT(sst != AGC_STATUS_BAD_ARG);
    chan_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT15);
    call_wait((uint16_t) FOUR_SECS_CS, AGC_TASK_PFAILOK);
    return AGC_STATUS_OK;
}

/* Task dispatch for the T4RUPT tasks (called from exec dispatch.c).      */
agc_status_t agc_t4_task(agc_task_id_t task)
{
    (void) AGC_ASSERT(task != AGC_TASK_NONE);
    if (task == AGC_TASK_ENDTNON)
    {
        return t4_endtnon();
    }
    if (task == AGC_TASK_UNZ2)
    {
        return t4_unz2();
    }
    if (task == AGC_TASK_ISSUP)
    {
        return t4_issup();
    }
    if (task == AGC_TASK_ENDZOPT || task == AGC_TASK_ENDZOPT2)
    {
        if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
        {
            return AGC_STATUS_BAD_ARG;
        }
        return agc_t4_optics_task(task);
    }
    return AGC_STATUS_NOT_FOUND;
}
