/*
 * moding.c -- rendezvous radar (RR) moding, LM only (P20-P25.agc,
 * Luminary099 pp.521-537, 563): MAGSUB, RRLIMCHK, SETTRKF / FLIP,
 * RRTURNON / RRZEROSB, DORREPOS / REPOSRPT / SETRRECR, REMODE / RMODINV,
 * RRTONLY / RRSONLY / RR1AXIS, RROUT, RRZERO, ENDRADAR / RGOODEND /
 * RDBADEND, CLRADMOD (p.516).
 *
 * The AGC runs these as waitlist tasks chained by FIXDELAY, with RRRET
 * holding the return address of the subroutines that wait (RRZEROSB,
 * RR1AXIS). Here each FIXDELAY continuation is its own task and RRRET is
 * an enum naming the caller's next step.
 *
 * Angles are CDU words (lm/radar/radar.h); one revolution = 2^15 counts and the
 * AGC treats them as ones' complement SP numbers scaled in half
 * revolutions (16384 = 180 degrees). Gyro commands are SP pulses.
 */
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "agc/io/service_internal.h"
#include "radar.h"

/* ---- constants (pp.523, 528, 532, 534, 536) ----------------------------
 * DEC values are SP fractions of 180 degrees (x 16384).                  */
#define RR_LIM_T1   (-5006)  /* DEC -.30555: 55 degrees               */
#define RR_LIM_S1   (-5871)  /* DEC -.35833: 64.5 degrees             */
#define RR_5_5DEGS  501      /* 5.5DEGS DEC .03056                    */
#define RR_82DEGS   7464     /* 82DEGS DEC .45556                     */
#define RR_LIM_S2   (-5188)  /* DEC -.31667: 57 degrees               */
#define RR_LIM_T2   (-11378) /* DEC -.69444: 125 degrees              */
#define RR_20_5DEGS 1866     /* 20.5DEGS DEC .11389                   */
#define RR_LIM_90   (-8192)  /* DEC -.5: 90 degrees (mode test)       */
#define RR_1DEG     (-91)    /* DEC -.00555: 1 degree                 */
#define RR_SPGAIN \
    9677                     /* RRSPGAIN DEC .59062: null .7 of the
                              * error in .5 s                          */
#define RR_LIMIT     (-384)  /* -RRLIMIT DEC -384 pulses               */
#define RR_13_7_2    010102u /* 13,7,2: zeroing, CDU OK, not in auto  */
#define RR_BIT11_1   002001u /* BIT11+1: reposition, turn-on          */
#define RR_BIT10_15  041000u /* BIT10+15: designate, continuous des.  */
#define RR_TRKF_LAMP 000200u /* DSPTAB +11D BIT8: tracker fail lamp   */
#define RR_DSPTAB_11 11
/* Single-axis targets, CDU words: BIT15 / NEGMAX = 180 degrees, CS HALF
 * = -90, -45DEGSR = 13,14,15 (octal 70000), -50DEGSR DEC -.27778,
 * -50DEGSR + -80DEGSR = -130 degrees.                                   */
#define RR_W_180  040000u
#define RR_W_M90  057777u
#define RR_W_M45  070000u
#define RR_W_M50  ((uint16_t) (077777 - 4551))
#define RR_W_M130 ((uint16_t) (077777 - 11833))
/* Waits, centiseconds.                                                  */
#define RR_DT_20MS    2U
#define RR_DT_HALFSEC 50U
#define RR_DT_1SEC    100U
#define RR_DT_10SEC   1000U
/* Bound on the RR1AXIS steps run in one task (REMODE has five).         */
#define RR_AXIS_MAX_STEPS 12
#define RR_ALARM_NOT_AUTO ((agc_alarm_code_t) 00510)

/* Where RRZEROSB returns (RRRET): RRTURNON or RRZ2 (RRZERO).            */
typedef enum rr_zero_return
{
    RR_ZERO_FOR_TURNON = 0,
    RR_ZERO_FOR_RRZERO
} rr_zero_return_t;

/* RR1AXIS state: RDES, RRINDEX and the client step (RRRET).             */
typedef struct rr_axis
{
    rr_axis_client_t client;
    int              step;
    int              rrindex; /* 0 trunnion (CDUT), 1 shaft (CDUS)      */
    uint16_t         rdes;    /* RDES: desired angle, CDU word          */
} rr_axis_t;

static rr_state_t       rr_designate_state;
static rr_axis_t        rr_axis     = {RR_AXIS_REPOSITION, 0, 0, 0U};
static rr_zero_return_t rr_zero_ret = RR_ZERO_FOR_TURNON;

rr_state_t *rr_state(void)
{
    return &rr_designate_state;
}

/* ---- word helpers ----------------------------------------------------- */
int rr_ones(uint16_t w)
{
    uint16_t v = (uint16_t) (w & RR_WORD_MASK);
    int      r = ((v & 040000u) != 0u) ? -(int) (RR_WORD_MASK - v) : (int) v;

    if (!AGC_ASSERT(r >= AGC_SP_MIN && r <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }

    return r;
}

uint16_t rr_word(int v)
{
    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return 0U;
    }

    return (v >= 0) ? (uint16_t) v : (uint16_t) ((int) RR_WORD_MASK + v);
}

/* MAGSUB (p.521): |a| + k <= 0 returns to L+3 (within), else L+2.     */
bool rr_magsub(int a, int k)
{
    if (!AGC_ASSERT(k < 0))
    {
        return false;
    }

    int m = (a < 0) ? -a : a;

    return (m + k) <= 0;
}

/* MSU (p.534): two's complement difference of the words a - k, as a
 * ones' complement number. DEVIATION: -180 degrees exactly (-16384) is
 * taken as -16383.                                                      */
static int rr_msu(uint16_t a, uint16_t k)
{
    int d = ((int) (a & RR_WORD_MASK) - (int) (k & RR_WORD_MASK)) & 077777;

    if ((d & 040000) != 0)
    {
        d -= 0100000;
    }

    if (!AGC_ASSERT(d >= -16384 && d <= 16383))
    {
        return 0; /* no valid result */
    }

    return (d < AGC_SP_MIN) ? AGC_SP_MIN : d;
}

/* EXTEND MP: upper word of the SP product.                              */
static int rr_mp(int a, int b)
{
    if (!AGC_ASSERT(a >= AGC_SP_MIN && a <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }

    if (!AGC_ASSERT(b >= AGC_SP_MIN && b <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }

    return agc_dp_high(agc_sp_mul((agc_sp_t) a, (agc_sp_t) b));
}

void rr_chan_or(int chan, uint16_t bits)
{
    agc_chan_or(chan, bits);

    if (!AGC_ASSERT(bits <= RR_WORD_MASK))
    {
        return;
    }
}

void rr_chan_clear(int chan, uint16_t bits)
{
    agc_chan_and(chan, (uint16_t) (~(uint32_t) bits & RR_WORD_MASK));

    if (!AGC_ASSERT(bits <= RR_WORD_MASK))
    {
        return;
    }
}

void rr_radmodes_set(uint16_t bits)
{
    if (!AGC_ASSERT(bits <= RR_WORD_MASK))
    {
        return;
    }

    agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() | bits));
}

void rr_radmodes_clear(uint16_t bits)
{
    if (!AGC_ASSERT(bits <= RR_WORD_MASK))
    {
        return;
    }

    agc_erase_set_radmodes(
        (uint16_t) (agc_erase_radmodes() & ~bits & RR_WORD_MASK));
}

bool rr_radmodes_test(uint16_t bits)
{
    if (!AGC_ASSERT(bits != 0U && bits <= RR_WORD_MASK))
    {
        return false;
    }

    return (agc_erase_radmodes() & bits) != 0U;
}

bool rr_mode2(void)
{
    return rr_radmodes_test(RR_ANTENBIT);
}

/* RMODINV (p.532): invert the antenna mode bit.                         */
void rr_mode_invert(void)
{
    uint16_t m = agc_erase_radmodes();

    agc_erase_set_radmodes((uint16_t) (m ^ RR_ANTENBIT));

    if (!AGC_ASSERT(((agc_erase_radmodes() ^ m) & RR_ANTENBIT) != 0U))
    {
        return;
    }
}

/* ---- RRLIMCHK (pp.522-523) ---------------------------------------------
 * Mode 1: |T| <= 55 and |S + 5.5| <= 64.5 degrees (S from -70 to +59).
 * Mode 2: |S + 82| <= 57 (S from -139 to -25) and |T| > 125 degrees.   */
bool rr_limits_ok(uint16_t t, uint16_t s)
{
    int tv = rr_ones(t);
    int sv = rr_ones(s);

    if (!rr_mode2())
    {
        return rr_magsub(tv, RR_LIM_T1) &&
               rr_magsub(sv + RR_5_5DEGS, RR_LIM_S1);
    }

    if (!rr_magsub(sv + RR_82DEGS, RR_LIM_S2))
    {
        return false;
    }

    return !rr_magsub(tv, RR_LIM_T2);
}

/* RRLIMCHK as called by the T4RUPT gimbal monitor (sys-misc) on CDUT,
 * CDUS: true = within the limits of the present mode (L+3).            */
bool agc_rr_limchk(uint16_t cdut, uint16_t cdus)
{
    if (!AGC_ASSERT(cdut <= RR_WORD_MASK && cdus <= RR_WORD_MASK))
    {
        return false;
    }

    return rr_limits_ok(cdut, cdus);
}

/* ---- lamps (SETTRKF, TRKFLON, FLIP pp.524-525) ------------------------ */
/* FLIP: invert the lamp bit in DSPTAB +11D and mark the word for output
 * (bit 15).                                                             */
void agc_rendz_flip(uint16_t lamp_bit)
{
    if (!AGC_ASSERT(lamp_bit != 0U && lamp_bit <= RR_WORD_MASK))
    {
        return;
    }

    uint16_t w = agc_erase_dsptab(RR_DSPTAB_11);

    w = (uint16_t) (((w ^ lamp_bit) & 037777u) | 040000u); /* POSMAX, BIT15 */
    agc_erase_set_dsptab(RR_DSPTAB_11, w);
}

/* TRKFLON +1: the "half add" of the desired and present lamp states;
 * FLIP only when they differ.                                           */
void rr_lamp_put(uint16_t lamp, bool on)
{
    bool now = (agc_erase_dsptab(RR_DSPTAB_11) & lamp) != 0U;

    if (now == on)
    {
        return; /* NO CHANGE      */
    }

    agc_rendz_flip(lamp);
}

/* RRTRKF: the tracker fail lamp is on when the RR CDU failed in auto mode
 * while not being zeroed (RADMODES 13, 7, 2 all zero), or RR data failed
 * (RADMODES bit 4).                                                     */
static void rr_trkf(void)
{
    uint16_t m  = agc_erase_radmodes();
    bool     on = ((m & RR_13_7_2) == 0U) || ((m & RR_RRDATABT) != 0U);

    rr_lamp_put(RR_TRKF_LAMP, on);
}

/* SETTRKF (p.524): no action during a lamp test (IMODES33 bit 1).        */
agc_status_t agc_rr_settrkf(void)
{
    if ((agc_erase_imodes33() & 1U) != 0U)
    {
        return AGC_STATUS_OK;
    }

    rr_trkf();

    if (!AGC_ASSERT(agc_erase_radmodes() <= RR_WORD_MASK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* ---- end of a radar operation (ENDRADAR p.563) ------------------------
 * DEVIATION: the "-2" entries that set RUPTAGN for a caller outside
 * waitlist control are the same as the plain entries here.            */
agc_status_t rr_goodend(void)
{
    agc_status_t st = agc_imu_goodend(AGC_IMU_DEV_RADAR);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_NOT_FOUND))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
}

agc_status_t rr_badend(void)
{
    agc_status_t st = agc_imu_badend(AGC_IMU_DEV_RADAR);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_NOT_FOUND))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
}

/* ENDRADAR: good end unless the RR CDU failed.                         */
agc_status_t rr_endradar(void)
{
    return rr_radmodes_test(RR_RCDUFBIT) ? rr_goodend() : rr_badend();
}

/* RGOODEND as a waitlist task (RADNOOP's 1 cs call).                    */
agc_status_t agc_task_rr_goodend(void)
{
    return rr_goodend();
}

/* RADNOOP (p.561): no further action requested; good end in 10 ms.     */
agc_status_t rr_radnoop(void)
{
    agc_status_t st = agc_waitlist_call(1U, AGC_TASK_RR_GOODEND);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* CLRADMOD (p.516): clear designate and continuous designate, disable
 * the RR error counters. The AGC's caller is inhibited (the user will
 * RELINT); here the routine inhibits around its own update.            */
void agc_rendz_clradmod(void)
{
    agc_inhint();
    rr_radmodes_clear(RR_BIT10_15);
    rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_ENB_ECTR);
    agc_relint();

    if (!AGC_ASSERT(!rr_radmodes_test(RR_BIT10_15)))
    {
        return;
    }
}

/* SETRRECR (p.530): enable the RR error counters and clear the last
 * commands, unless the counters are already enabled.                  */
void rr_setrrecr(void)
{
    if ((agc_chan_read(AGC_CHAN_CHAN12) & RR_CH12_ENB_ECTR) != 0U)
    {
        return;
    }

    rr_designate_state.lastcmd[0] = 0;
    rr_designate_state.lastcmd[1] = 0;
    rr_chan_or(AGC_CHAN_CHAN12, RR_CH12_ENB_ECTR);
}

/* RROUT (pp.535-536): limit TANG (trunnion), TANG +1 (shaft) to 384
 * pulses, count out the change from LASTYCMD / LASTXCMD into CDUTCMD /
 * CDUSCMD, and drive both CDUs. DEVIATION: the AGC adds -0 so that a
 * zero change is -0 in the counter; this model has no -0.              */
void rr_rrout(void)
{
    rr_state_t *s = &rr_designate_state;

    for (int i = 1; i >= 0; i--)
    { /* shaft, then trunnion   */
        int cmd = s->tang[i];

        if (!rr_magsub(cmd, RR_LIMIT))
        {
            cmd = (cmd > 0) ? -RR_LIMIT : RR_LIMIT; /* RROUTLIM   */
        }

        int delta = cmd - s->lastcmd[i];

        s->lastcmd[i] = (agc_sp_t) cmd;
        agc_erase_set_cdutcmd(i, (agc_sp_t) delta);
    }

    rr_chan_or(AGC_CHAN_CHAN14, RR_CH14_DRIVE_TS);
}

/* ---- RRTURNON / RRZEROSB (pp.526-528) ----------------------------------
 * RRZEROSB: RR CDU zero discrete on (chan 12 bit 1), 20 ms, CDUT = CDUS =
 * 0, discrete off, 10 s for the CDUs to repeat the real angles, zeroing
 * flag off, antenna mode from the trunnion angle (mode 2 when |T| > 90
 * degrees), SETTRKF, then back to the caller (RRRET).                  */
static agc_status_t rr_zero_begin(rr_zero_return_t ret)
{
    rr_zero_ret = ret;
    rr_chan_or(AGC_CHAN_CHAN12, RR_CH12_ZERO_CDU);
    agc_status_t st =
        agc_waitlist_fixdelay(RR_DT_20MS, AGC_TASK_RR_ZERO_COUNTERS);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* RRZEROSB after 20 ms: zero the counters, remove the zero discrete.    */
agc_status_t agc_task_rr_zero_counters(void)
{
    agc_erase_set_cdut(0U);
    agc_erase_set_cdus(0U);
    rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_ZERO_CDU);
    agc_status_t st =
        agc_waitlist_fixdelay(RR_DT_10SEC, AGC_TASK_RR_ZERO_SETTLED);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* RRZEROSB after 10 s: mode, tracker lamp, return to RRRET.             */
agc_status_t agc_task_rr_zero_settled(void)
{
    rr_radmodes_clear(RR_RCDU0BIT);

    if (rr_magsub(rr_ones(agc_erase_cdut()), RR_LIM_90))
    {
        rr_radmodes_clear(RR_ANTENBIT); /* mode 1  */
    }
    else
    {
        rr_radmodes_set(RR_ANTENBIT); /* mode 2  */
    }

    agc_status_t st = agc_rr_settrkf();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (rr_zero_ret == RR_ZERO_FOR_RRZERO)
    {
        return rr_endradar(); /* RRZ2    */
    }

    /* RRTURNON: wait 1 s before removing the turn-on flag, so that a
     * monitor reposition will not alarm.                                */
    st = agc_waitlist_fixdelay(RR_DT_1SEC, AGC_TASK_RR_TURNON_DONE);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* RRTURNON (p.526): waitlist task from RRAUTCHK (T4RUPT) when the RR
 * power-on-auto discrete appears.                                       */
agc_status_t agc_task_rrturnon(void)
{
    if (!AGC_ASSERT(rr_radmodes_test(RR_TURNONBT)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return rr_zero_begin(RR_ZERO_FOR_TURNON);
}

agc_status_t agc_task_rr_turnon_done(void)
{
    rr_radmodes_clear(RR_TURNONBT);

    if (!AGC_ASSERT(!rr_radmodes_test(RR_TURNONBT)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK; /* TASKOVER */
}

/* RRZ2 (p.537): RRZEROSB, then ENDRADAR.                                */
agc_status_t agc_task_rr_zero(void)
{
    if (!AGC_ASSERT(rr_radmodes_test(RR_RCDU0BIT)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return rr_zero_begin(RR_ZERO_FOR_RRZERO);
}

/* RRZERO (p.537): zero the RR CDUs and find the antenna mode; the caller
 * then waits in RADSTALL. Nothing to do during a monitor reposition or
 * turn-on (RADNOOP); alarm 510 when the auto discrete is absent.       */
agc_status_t agc_rr_zero(void)
{
    if (rr_radmodes_test(RR_BIT11_1))
    {
        return rr_radnoop();
    }

    agc_inhint();
    rr_radmodes_set(RR_RCDU0BIT);
    agc_status_t st = agc_waitlist_call(1U, AGC_TASK_RR_ZERO);

    if (st == AGC_STATUS_OK && rr_radmodes_test(RR_AUTOMBIT))
    {
        st = rz_alarm(RR_ALARM_NOT_AUTO);
        st = (st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st;
    }

    agc_relint(); /* ROADBACK */

    return st;
}

/* ---- single-axis drives (RRTONLY, RRSONLY, RR1AXIS pp.533-534) -------- */
/* The step-th single-axis target of a client: trunnion (rrindex 0) or
 * shaft (1) and the desired CDU word. False: the client is done.       */
static bool rr_axis_target(rr_axis_client_t c, int step, int *rrindex,
                           uint16_t *rdes)
{
    bool             m2             = rr_mode2();
    static const int remode_axis[5] = {0, 1, 0, 0, 1};

    *rrindex = 0;

    switch (c)
    {
        case RR_AXIS_REPOSITION: /* DORREPOS: T to 0 (180), S to 0 (-90) */
            *rrindex = step;
            *rdes    =
                (step == 0) ? (m2 ? RR_W_180 : 0U) : (m2 ? RR_W_M90 : 0U);

            return step <= 1;
        case RR_AXIS_REMODE: /* REMODE pp.531-532                    */

            if (step < 0 || step > 4)
            {
                return false;
            }

            *rrindex = remode_axis[step];
            *rdes    = (step == 0) ? (m2 ? RR_W_180 : 0U) /* T 0 (180)  */
                       :
                       (step == 1) ? RR_W_M45 /* S -45      */
                       :
                       (step == 2) ? (m2 ? RR_W_M50 : RR_W_M130) /* T -130(-50)*/
                       :
                       (step == 3) ? (m2 ? 0U : RR_W_180) /* T -180 (0) */
                       :
                       (m2 ? 0U : RR_W_M90);               /* S -90 (0)  */

            return true;
        case RR_AXIS_PREPOSITION_R29: /* PREPOS29: T to -180 (NEGMAX)          */
            *rdes = RR_W_180;

            return step == 0;
        case RR_AXIS_CLIENT_COUNT:
        default:
            return false; /* unreachable: cannot happen */
    }
}

/* Outcome of one RR1AX2 pass.                                           */
typedef enum rr_axis_outcome
{
    RR_AXIS_WAITING = 0, /* command sent, NXTRR1AX in .5 s          */
    RR_AXIS_DONE,        /* within 1 degree or out of auto: RRRET   */
    RR_AXIS_ABANDON      /* remode and reposition both on: REPOSRPT */
} rr_axis_outcome_t;

/* RR1AX2: error RDES - CDU (MSU), command .59 x error to the one axis,
 * RROUT, again in .5 s.                                                 */
static rr_axis_outcome_t rr_axis_pass(agc_status_t *st)
{
    *st = AGC_STATUS_OK;
    uint16_t m = agc_erase_radmodes();

    if ((~m & (RR_REMODBIT | RR_REPOSBIT)) == 0U)
    {
        return RR_AXIS_ABANDON;
    }

    uint16_t cdu = (rr_axis.rrindex == 0) ? agc_erase_cdut() : agc_erase_cdus();
    int      err = rr_msu(rr_axis.rdes, cdu); /* ITEMP1  */
    int      cmd = rr_mp(err, RR_SPGAIN);     /* L       */

    if (rr_magsub(err, RR_1DEG) || (m & RR_AUTOMBIT) != 0U)
    {
        return RR_AXIS_DONE;
    }

    rr_designate_state.tang[0] = (agc_sp_t) ((rr_axis.rrindex == 0) ? cmd : 0);
    rr_designate_state.tang[1] = (agc_sp_t) ((rr_axis.rrindex == 0) ? 0 : cmd);
    rr_rrout();
    *st = agc_waitlist_fixdelay(RR_DT_HALFSEC, AGC_TASK_RR_ONE_AXIS);

    return RR_AXIS_WAITING;
}

/* REPOSRPT (p.530): reposition over; designate if someone is waiting
 * (BEGDES: a remode first when requested), else drop the error counter
 * enable. True when a remode axis must run next.                       */
static bool rr_reposrpt(agc_status_t *st)
{
    *st = AGC_STATUS_OK;
    rr_radmodes_clear(RR_REPOSBIT);

    if (!rr_radmodes_test(RR_DESIGBIT))
    {
        rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_ENB_ECTR);

        return false; /* TASKOVER */
    }

    if (!rr_radmodes_test(RR_REMODBIT))
    {
        *st = rr_stdesig(); /* BEGDES   */

        return false;
    }

    rr_axis.client = RR_AXIS_REMODE; /* REMODE   */
    rr_axis.step   = 0;

    return rr_axis_target(RR_AXIS_REMODE, 0, &rr_axis.rrindex, &rr_axis.rdes);
}

/* End of REMODE (p.532): new mode, remode flag off; back to the
 * designate (STDESIG) or to the caller waiting in RADSTALL.             */
static agc_status_t rr_remode_done(void)
{
    rr_mode_invert(); /* RMODINV  */
    rr_radmodes_clear(RR_REMODBIT);

    if (rr_radmodes_test(RR_DESIGBIT))
    {
        return rr_stdesig();
    }

    return rr_goodend();
}

/* A client's single-axis drive is done: next target, or the client's
 * own ending. True when another axis must run.                          */
static bool rr_axis_advance(agc_status_t *st)
{
    *st = AGC_STATUS_OK;
    rr_axis.step++;

    if (rr_axis_target(rr_axis.client, rr_axis.step, &rr_axis.rrindex,
                       &rr_axis.rdes))
    {
        return true;
    }

    switch (rr_axis.client)
    {
        case RR_AXIS_REPOSITION:
            return rr_reposrpt(st);
        case RR_AXIS_REMODE:
            *st = rr_remode_done();

            return false;
        case RR_AXIS_PREPOSITION_R29: /* PREPOS29: repositioning off */
            rr_radmodes_clear(RR_REPOSBIT);

            return false;
        case RR_AXIS_CLIENT_COUNT:
        default:
            return false; /* unreachable: cannot happen */
    }
}

/* Run single-axis drives until one has to wait .5 s (the AGC's chain of
 * RR1AXIS returns, as a bounded loop).                                  */
static agc_status_t rr_axis_run(void)
{
    agc_status_t st = AGC_STATUS_OK;

    for (int i = 0; i < RR_AXIS_MAX_STEPS; i++)
    {
        rr_axis_outcome_t o    = rr_axis_pass(&st);
        bool              more = false;

        if (o == RR_AXIS_WAITING)
        {
            return st;
        }

        if (o == RR_AXIS_ABANDON)
        {
            more = rr_reposrpt(&st);
        }
        else
        {
            more = rr_axis_advance(&st);
        }

        if (!more)
        {
            return st;
        }
    }

    return AGC_STATUS_NO_CONVERGE; /* step bound exhausted */
}

agc_status_t rr_axis_start(rr_axis_client_t client, int step, int rrindex,
                           uint16_t rdes)
{
    if (!AGC_ASSERT((int) client >= 0 && client < RR_AXIS_CLIENT_COUNT) ||
        !AGC_ASSERT(rrindex == 0 || rrindex == 1))
    {
        return AGC_STATUS_BAD_ARG;
    }

    rr_axis.client  = client;
    rr_axis.step    = step;
    rr_axis.rrindex = rrindex;
    rr_axis.rdes    = (uint16_t) (rdes & RR_WORD_MASK);

    return rr_axis_run();
}

/* NXTRR1AX: the next .5 s pass of the single-axis drive.               */
agc_status_t agc_task_rr_one_axis(void)
{
    if (!AGC_ASSERT(rr_axis.client < RR_AXIS_CLIENT_COUNT))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return rr_axis_run();
}

/* ---- DORREPOS (pp.529-530) -------------------------------------------
 * Waitlist task from the T4RUPT gimbal monitor: drive the RR to a safe
 * position (T 0, S 0 in mode 1; T 180, S -90 in mode 2).              */
agc_status_t agc_task_dorrepos(void)
{
    rr_setrrecr();
    agc_status_t st =
        agc_waitlist_fixdelay(RR_DT_20MS, AGC_TASK_RR_REPOSITION_DRIVE);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

agc_status_t agc_task_rr_reposition_drive(void)
{
    int      idx  = 0;
    uint16_t rdes = 0U;
    bool     ok   = rr_axis_target(RR_AXIS_REPOSITION, 0, &idx, &rdes);

    if (!AGC_ASSERT(ok))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return rr_axis_start(RR_AXIS_REPOSITION, 0, idx, rdes);
}

/* ---- REMODE (pp.531-532): T to 0 (180), S to -45, T to -130 (-50), T to
 * -180 (0), S to -90 (0), new mode; then STDESIG or RGOODEND.          */
agc_status_t rr_remode_start(void)
{
    int      idx  = 0;
    uint16_t rdes = 0U;
    bool     ok   = rr_axis_target(RR_AXIS_REMODE, 0, &idx, &rdes);

    if (!AGC_ASSERT(ok))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return rr_axis_start(RR_AXIS_REMODE, 0, idx, rdes);
}

/* REMODE as a waitlist task (R29REM0J: "remode must run as a task").    */
agc_status_t agc_task_rr_remode(void)
{
    if (!AGC_ASSERT(rr_radmodes_test(RR_REMODBIT)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return rr_remode_start();
}
