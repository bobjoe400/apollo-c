/**********************************************************************
 * FILE NAME: moding.c
 *
 * PURPOSE:
 *   Rendezvous radar (RR) moding, LM only: MAGSUB, RRLIMCHK, SETTRKF /
 *   FLIP, RRTURNON / RRZEROSB, DORREPOS / REPOSRPT / SETRRECR, REMODE /
 *   RMODINV, RRTONLY / RRSONLY / RR1AXIS, RROUT, RRZERO, ENDRADAR /
 *   RGOODEND / RDBADEND, CLRADMOD. The AGC runs these as waitlist tasks
 *   chained by FIXDELAY, with RRRET holding the return address of the
 *   subroutines that wait (RRZEROSB, RR1AXIS). Here each FIXDELAY
 *   continuation is its own task and RRRET is an enum naming the
 *   caller's next step.
 *
 * AGC SOURCE:
 *   Luminary099 P20-P25.agc pp. 521-537 (MAGSUB, RRLIMCHK, SETTRKF,
 *   TRKFLON, FLIP, RRTURNON, RRZEROSB, DORREPOS, REPOSRPT, SETRRECR,
 *   REMODE, RMODINV, RRTONLY, RRSONLY, RR1AXIS, RR1AX2, NXTRR1AX, RROUT,
 *   RRZERO, RRZ2), p. 563 (ENDRADAR, RGOODEND, RDBADEND), p. 516
 *   (CLRADMOD); LM only
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  ----------------------------
 *   RADMODES   word       I/O  -          RR mode and status bits
 *   CDUT CDUS  words      I/O  CDU        RR trunnion and shaft angles
 *   CDUTCMD CDUSCMD agc_sp_t O pulses     gyro command counters
 *   TANG       agc_sp_t   I/O  pulses     commanded gyro pulses
 *   LASTYCMD LASTXCMD (rr_state_t) I/O pulses  last commands sent
 *   DSPTAB +11D word      I/O  -          tracker fail lamp
 *   IMODES33   word       I    -          lamp test
 *   CHAN12 CHAN14  channels O  -          zero, error counters, drive
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ----------------------------------------------
 *   agc_imu_goodend       GOODEND / BADEND of RADSTALL
 *   agc_waitlist_fixdelay FIXDELAY, WAITLIST
 *   rr_stdesig            STDESIG: designate after a remode
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   00510 (RRZERO with the RR out of auto mode). A failed RR CDU ends a
 *   radar operation with BADEND. AGC_STATUS_NO_CONVERGE when more than
 *   RR_AXIS_MAX_STEPS single-axis drives finish in one task (cannot
 *   happen: REMODE has five).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Waitlist tasks and their subroutines (interrupts inhibited).
 *
 * NOTES:
 *   Angles are CDU words (lm/radar/radar.h); one revolution = 2^15
 *   counts and the AGC treats them as ones' complement SP numbers scaled
 *   in half revolutions (16384 = 180 degrees). Gyro commands are SP
 *   pulses. DEVIATION (RROUT): the AGC adds -0 so that a zero change is
 *   -0 in the counter; this model has no -0.
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
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

/* RRSPGAIN DEC .59062: null .7 of the error in .5 s                   */
#define RR_SPGAIN    9677    /* RRSPGAIN, 2^0                          */
#define RR_LIMIT     (-384)  /* -RRLIMIT DEC -384 pulses               */
#define RR_13_7_2    010102u /* 13,7,2: zeroing, CDU OK, not in auto  */
#define RR_BIT11_1   002001u /* BIT11+1: reposition, turn-on          */
#define RR_BIT10_15  041000u /* BIT10+15: designate, continuous des.  */
#define RR_TRKF_LAMP 000200u /* DSPTAB +11D BIT8: tracker fail lamp   */
#define RR_DSPTAB_11 11      /* DSPTAB +11D */

/* Single-axis targets, CDU words: BIT15 / NEGMAX = 180 degrees, CS HALF
 * = -90, -45DEGSR = 13,14,15 (octal 70000), -50DEGSR DEC -.27778,
 * -50DEGSR + -80DEGSR = -130 degrees.                                   */
#define RR_W_180  040000u                       /* 180 deg (NEGMAX) */
#define RR_W_M90  057777u                       /* -90 deg (CS HALF) */
#define RR_W_M45  070000u                       /* -45 deg */
#define RR_W_M50  ((uint16_t) (077777 - 4551))  /* -50 deg */
#define RR_W_M130 ((uint16_t) (077777 - 11833)) /* -130 deg */

/* Waits, centiseconds.                                                  */
#define RR_DT_20MS    2U    /* 20 ms, cs */
#define RR_DT_HALFSEC 50U   /* .5 s, cs */
#define RR_DT_1SEC    100U  /* 1 s, cs */
#define RR_DT_10SEC   1000U /* 10 s, cs */

/* Bound on the RR1AXIS steps run in one task (REMODE has five).         */
#define RR_AXIS_MAX_STEPS 12                         /* drives per task */
#define RR_ALARM_NOT_AUTO ((agc_alarm_code_t) 00510) /* RR not in auto */

/* Where RRZEROSB returns (RRRET): RRTURNON or RRZ2 (RRZERO).            */
typedef enum rr_zero_return
{
    RR_ZERO_FOR_TURNON = 0,
    RR_ZERO_FOR_RRZERO
} rr_zero_return_t;

/* RR1AXIS state: RDES, RRINDEX and the client step (RRRET).             */
typedef struct rr_axis
{
    rr_axis_client_t client;  /* RRRET: the caller */
    int              step;    /* the caller's step */
    int              rrindex; /* 0 trunnion (CDUT), 1 shaft (CDUS)      */
    uint16_t         rdes;    /* RDES: desired angle, CDU word          */
} rr_axis_t;

static rr_state_t       rr_designate_state;               /* radar erasables */
static rr_axis_t        rr_axis     = {RR_AXIS_REPOSITION, 0, 0, 0U};
static rr_zero_return_t rr_zero_ret = RR_ZERO_FOR_TURNON; /* RRRET */

/**********************************************************************
 * The RR designate erasables (RRTARGET, LOSVEL, MLOSV, MODEA/B, TANGNB,
 * DESCOUNT, LOSCOUNT, LASTYCMD/LASTXCMD).
 * AGC: ERASABLE_ASSIGNMENTS.agc (P20-P25 erasables)
 *
 * FUNCTION NAME: rr_state
 *
 * ARGUMENTS: none
 *
 * RETURNS: rr_state_t * -- the radar erasables (never NULL)
 **********************************************************************/
rr_state_t *rr_state(void)
{
    return (&rr_designate_state);
}

/**********************************************************************
 * word helpers
 * AGC: none (word conversion)
 *
 * FUNCTION NAME: rr_ones
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   w         uint16_t  I    raw CDU word
 *
 * RETURNS: int -- the ones' complement value, half revolutions 2^-14 per
 *          count
 **********************************************************************/
int rr_ones(uint16_t w)
{
    uint16_t v = (uint16_t) (w & RR_WORD_MASK); /* 15 bits */
    int      r = (int) v;                       /* signed value */

    if ((v & 040000u) != 0u)
    {
        r = -(int) (RR_WORD_MASK - v); /* negative word */
    }

    return (r);
}

/**********************************************************************
 * A signed SP value as a raw 15-bit ones' complement word.
 * AGC: none (word conversion)
 *
 * FUNCTION NAME: rr_word
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   v         int   I    the value
 *
 * RETURNS: uint16_t -- the raw word (0 when out of range)
 **********************************************************************/
uint16_t rr_word(int v)
{
    if (v < 0)
    {
        return ((uint16_t) ((int) RR_WORD_MASK + v)); /* ones' complement */
    }

    return ((uint16_t) v);
}

/**********************************************************************
 * MAGSUB (p.521): |a| + k <= 0 returns to L+3 (within), else L+2.
 * AGC: MAGSUB (P20-P25.agc p.521)
 *
 * FUNCTION NAME: rr_magsub
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   a         int   I    the value, CDU units
 *   k         int   I    minus the limit (negative), CDU units
 *
 * RETURNS: bool -- true when |a| + k <= 0 (within the limit, L+3)
 **********************************************************************/
bool rr_magsub(int a, int k)
{
    int m = (a < 0) ? -a : a; /* |a| */

    return ((m + k) <= 0);
}

/**********************************************************************
 * MSU (p.534): two's complement difference of the words a - k, as a
 * ones' complement number. DEVIATION: -180 degrees exactly (-16384) is
 * taken as -16383.
 * AGC: MSU (P20-P25.agc p.534)
 *
 * FUNCTION NAME: rr_msu
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   a         uint16_t  I    CDU word
 *   k         uint16_t  I    CDU word subtracted
 *
 * RETURNS: int -- a - k as a ones' complement number, CDU units
 **********************************************************************/
static int rr_msu(uint16_t a, uint16_t k)
{
    int d; /* MSU */

    d = ((int) (a & RR_WORD_MASK) - (int) (k & RR_WORD_MASK)) & 077777;

    if ((d & 040000) != 0)
    {
        d -= 0100000;
    }

    return ((d < AGC_SP_MIN) ? AGC_SP_MIN : d); /* -16384 -> -16383 */
}

/**********************************************************************
 * EXTEND MP: upper word of the SP product.
 * AGC: "EXTEND MP" (P20-P25.agc p.534)
 *
 * FUNCTION NAME: rr_mp
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   a         int   I    SP factor
 *   b         int   I    SP factor
 *
 * RETURNS: int -- the upper word of a b
 **********************************************************************/
static int rr_mp(int a, int b)
{
    return (agc_dp_high(agc_sp_mul((agc_sp_t) a, (agc_sp_t) b)));
}

/**********************************************************************
 * Clear bits of a channel (the AGC's CS / WAND).
 * AGC: none (the WAND channel instruction)
 *
 * FUNCTION NAME: rr_chan_clear
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   chan      int       I    channel
 *   bits      uint16_t  I    bits to clear
 **********************************************************************/
void rr_chan_clear(int chan, uint16_t bits)
{
    agc_chan_and(chan, (uint16_t) (~(uint32_t) bits & RR_WORD_MASK));
}

/**********************************************************************
 * Set RADMODES bits.
 * AGC: "CS RADMODES / MASK / ... TS RADMODES" (P20-P25.agc)
 *
 * FUNCTION NAME: rr_radmodes_set
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   bits      uint16_t  I    bits to set
 **********************************************************************/
void rr_radmodes_set(uint16_t bits)
{
    agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() | bits));
}

/**********************************************************************
 * Clear RADMODES bits.
 * AGC: "CS bits / MASK RADMODES / TS RADMODES" (P20-P25.agc)
 *
 * FUNCTION NAME: rr_radmodes_clear
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   bits      uint16_t  I    bits to clear
 **********************************************************************/
void rr_radmodes_clear(uint16_t bits)
{
    agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() & ~bits & RR_WORD_MASK));
}

/**********************************************************************
 * Test RADMODES bits.
 * AGC: "CA bits / MASK RADMODES" (P20-P25.agc)
 *
 * FUNCTION NAME: rr_radmodes_test
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   bits      uint16_t  I    bits to test (not 0)
 *
 * RETURNS: bool -- true when any of the bits is set
 **********************************************************************/
bool rr_radmodes_test(uint16_t bits)
{
    return ((agc_erase_radmodes() & bits) != 0U);
}

/**********************************************************************
 * The RR antenna is in mode 2 (RADMODES ANTENBIT).
 * AGC: "CA ANTENBIT / MASK RADMODES" (P20-P25.agc)
 *
 * FUNCTION NAME: rr_mode2
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true in mode 2
 **********************************************************************/
bool rr_mode2(void)
{
    return (rr_radmodes_test(RR_ANTENBIT));
}

/**********************************************************************
 * RMODINV (p.532): invert the antenna mode bit.
 *
 * FUNCTION NAME: rr_mode_invert
 *
 * ARGUMENTS: none
 **********************************************************************/
void rr_mode_invert(void)
{
    uint16_t m = agc_erase_radmodes(); /* RADMODES */

    agc_erase_set_radmodes((uint16_t) (m ^ RR_ANTENBIT));
}

/**********************************************************************
 * RRLIMCHK (pp.522-523)
 * Mode 1: |T| <= 55 and |S + 5.5| <= 64.5 degrees (S from -70 to +59).
 * Mode 2: |S + 82| <= 57 (S from -139 to -25) and |T| > 125 degrees.
 * True = within the limits of the present mode (L+3). Also called by
 * the T4RUPT gimbal monitor (sys-misc) on CDUT, CDUS.
 * AGC: RRLIMCHK (P20-P25.agc pp.522-523)
 *
 * FUNCTION NAME: agc_rr_limchk
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   t         uint16_t  I    trunnion CDU word
 *   s         uint16_t  I    shaft CDU word
 *
 * RETURNS: bool -- true when within the limits of the present mode (L+3)
 **********************************************************************/
bool agc_rr_limchk(uint16_t t, uint16_t s)
{
    int tv = rr_ones(t); /* trunnion, CDU units */
    int sv = rr_ones(s); /* shaft, CDU units */

    if (!rr_mode2())
    {
        return (rr_magsub(tv, RR_LIM_T1) && rr_magsub(sv + RR_5_5DEGS, RR_LIM_S1));
    }

    if (!rr_magsub(sv + RR_82DEGS, RR_LIM_S2))
    {
        return (false);
    }

    return (!rr_magsub(tv, RR_LIM_T2));
}

/* ---- lamps (SETTRKF, TRKFLON, FLIP pp.524-525) ------------------------ */
/**********************************************************************
 * FLIP: invert the lamp bit in DSPTAB +11D and mark the word for output
 * (bit 15).
 * AGC: FLIP (P20-P25.agc p.525)
 *
 * FUNCTION NAME: agc_rendz_flip
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   lamp_bit  uint16_t  I    lamp bit in DSPTAB +11D
 **********************************************************************/
void agc_rendz_flip(uint16_t lamp_bit)
{
    uint16_t w = agc_erase_dsptab(RR_DSPTAB_11); /* DSPTAB +11D */

    w = (uint16_t) (((w ^ lamp_bit) & 037777u) | 040000u); /* POSMAX, BIT15 */
    agc_erase_set_dsptab(RR_DSPTAB_11, w);
}

/**********************************************************************
 * TRKFLON +1: the "half add" of the desired and present lamp states;
 * FLIP only when they differ.
 * AGC: TRKFLON +1 (P20-P25.agc p.525)
 *
 * FUNCTION NAME: rr_lamp_put
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   lamp      uint16_t  I    lamp bit in DSPTAB +11D
 *   on        bool      I    the desired lamp state
 **********************************************************************/
void rr_lamp_put(uint16_t lamp, bool on)
{
    bool now = (agc_erase_dsptab(RR_DSPTAB_11) & lamp) != 0U; /* lamp on now */

    if (now != on) /* NO CHANGE      */
    {
        agc_rendz_flip(lamp);
    }
}

/**********************************************************************
 * RRTRKF: the tracker fail lamp is on when the RR CDU failed in auto mode
 * while not being zeroed (RADMODES 13, 7, 2 all zero), or RR data failed
 * (RADMODES bit 4).
 * AGC: RRTRKF (P20-P25.agc p.524)
 *
 * FUNCTION NAME: rr_trkf
 *
 * ARGUMENTS: none
 **********************************************************************/
static void rr_trkf(void)
{
    uint16_t m = agc_erase_radmodes(); /* RADMODES */
    bool     on;                       /* lamp wanted */

    on = ((m & RR_13_7_2) == 0U) || ((m & RR_RRDATABT) != 0U);

    rr_lamp_put(RR_TRKF_LAMP, on);
}

/**********************************************************************
 * SETTRKF (p.524): no action during a lamp test (IMODES33 bit 1).
 * AGC: SETTRKF (P20-P25.agc p.524)
 *
 * FUNCTION NAME: agc_rr_settrkf
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_rr_settrkf(void)
{
    if ((agc_erase_imodes33() & 1U) == 0U)
    {
        rr_trkf(); /* no lamp test (IMODES33 bit 1) */
    }
}

/**********************************************************************
 * end of a radar operation (ENDRADAR p.563)
 * RGOODEND: also a waitlist task (RADNOOP's 1 cs call).
 * DEVIATION: the "-2" entries that set RUPTAGN for a caller outside
 * waitlist control are the same as the plain entries here.
 * AGC: RGOODEND (P20-P25.agc p.563)
 *
 * FUNCTION NAME: agc_task_rr_goodend
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (also when nobody waits),
 *          AGC_STATUS_BAD_ARG for an unexpected GOODEND status
 **********************************************************************/
agc_status_t agc_task_rr_goodend(void)
{
    agc_status_t st = agc_imu_goodend(AGC_IMU_DEV_RADAR); /* GOODEND */

    if (st == AGC_STATUS_NOT_FOUND)
    {
        st = AGC_STATUS_OK; /* nobody waits in RADSTALL */
    }

    return (st);
}

/**********************************************************************
 * RDBADEND: bad end of a radar operation for the job waiting in RADSTALL.
 * AGC: RDBADEND (P20-P25.agc p.563)
 *
 * FUNCTION NAME: rr_badend
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (also when nobody waits),
 *          AGC_STATUS_BAD_ARG for an unexpected BADEND status
 **********************************************************************/
agc_status_t rr_badend(void)
{
    agc_status_t st = agc_imu_badend(AGC_IMU_DEV_RADAR); /* BADEND */

    if (st == AGC_STATUS_NOT_FOUND)
    {
        st = AGC_STATUS_OK; /* nobody waits in RADSTALL */
    }

    return (st);
}

/**********************************************************************
 * ENDRADAR: good end unless the RR CDU failed.
 * AGC: ENDRADAR (P20-P25.agc p.563)
 *
 * FUNCTION NAME: rr_endradar
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of RGOODEND or RDBADEND
 **********************************************************************/
agc_status_t rr_endradar(void)
{
    if (rr_radmodes_test(RR_RCDUFBIT))
    {
        return (agc_task_rr_goodend()); /* RR CDU good */
    }

    return (rr_badend());
}

/**********************************************************************
 * RADNOOP (p.561): no further action requested; good end in 10 ms.
 * AGC: RADNOOP (P20-P25.agc p.561)
 *
 * FUNCTION NAME: rr_radnoop
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the waitlist status; AGC_STATUS_BAD_ARG on a
 *          failed precondition
 **********************************************************************/
agc_status_t rr_radnoop(void)
{
    agc_status_t st = agc_waitlist_call(1U, AGC_TASK_RR_GOODEND); /* WAITLIST */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * CLRADMOD (p.516): clear designate and continuous designate, disable
 * the RR error counters. The AGC's caller is inhibited (the user will
 * RELINT); here the routine inhibits around its own update.
 * AGC: CLRADMOD (P20-P25.agc p.516)
 *
 * FUNCTION NAME: agc_rendz_clradmod
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_rendz_clradmod(void)
{
    agc_inhint();
    rr_radmodes_clear(RR_BIT10_15);
    rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_ENB_ECTR);
    agc_relint();
}

/**********************************************************************
 * SETRRECR (p.530): enable the RR error counters and clear the last
 * commands, unless the counters are already enabled.
 * AGC: SETRRECR (P20-P25.agc p.530)
 *
 * FUNCTION NAME: rr_setrrecr
 *
 * ARGUMENTS: none
 **********************************************************************/
void rr_setrrecr(void)
{
    if ((agc_chan_read(AGC_CHAN_CHAN12) & RR_CH12_ENB_ECTR) == 0U)
    {
        rr_designate_state.lastcmd[0] = 0;
        rr_designate_state.lastcmd[1] = 0;
        agc_chan_or(AGC_CHAN_CHAN12, RR_CH12_ENB_ECTR);
    }
}

/**********************************************************************
 * RROUT (pp.535-536): limit TANG (trunnion), TANG +1 (shaft) to 384
 * pulses, count out the change from LASTYCMD / LASTXCMD into CDUTCMD /
 * CDUSCMD, and drive both CDUs. DEVIATION: the AGC adds -0 so that a
 * zero change is -0 in the counter; this model has no -0.
 * AGC: RROUT (P20-P25.agc pp.535-536)
 *
 * FUNCTION NAME: rr_rrout
 *
 * ARGUMENTS: none
 **********************************************************************/
void rr_rrout(void)
{
    rr_state_t *s = &rr_designate_state; /* radar erasables */

    for (int i = 1; i >= 0; i--)
    { /* shaft, then trunnion   */
        int cmd = agc_erase_tang(i); /* TANG, pulses */

        if (!rr_magsub(cmd, RR_LIMIT))
        {
            cmd = (cmd > 0) ? -RR_LIMIT : RR_LIMIT; /* RROUTLIM   */
        }

        int delta = cmd - s->lastcmd[i]; /* pulses to send */

        s->lastcmd[i] = (agc_sp_t) cmd;
        agc_erase_set_cdutcmd(i, (agc_sp_t) delta);
    }

    agc_chan_or(AGC_CHAN_CHAN14, RR_CH14_DRIVE_TS);
}

/**********************************************************************
 * RRTURNON / RRZEROSB (pp.526-528)
 * RRZEROSB: RR CDU zero discrete on (chan 12 bit 1), 20 ms, CDUT = CDUS =
 * 0, discrete off, 10 s for the CDUs to repeat the real angles, zeroing
 * flag off, antenna mode from the trunnion angle (mode 2 when |T| > 90
 * degrees), SETTRKF, then back to the caller (RRRET).
 * AGC: RRZEROSB (P20-P25.agc p.527)
 *
 * FUNCTION NAME: rr_zero_begin
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------
 *   ret       rr_zero_return_t  I    RRRET: RRTURNON or RRZ2
 *
 * RETURNS: agc_status_t -- the FIXDELAY status; AGC_STATUS_BAD_ARG on a
 *          failed precondition
 **********************************************************************/
static agc_status_t rr_zero_begin(rr_zero_return_t ret)
{
    rr_zero_ret = ret;
    agc_chan_or(AGC_CHAN_CHAN12, RR_CH12_ZERO_CDU);
    agc_status_t st; /* 20 ms */

    st = agc_waitlist_fixdelay(RR_DT_20MS, AGC_TASK_RR_ZERO_COUNTERS);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * RRZEROSB after 20 ms: zero the counters, remove the zero discrete.
 * AGC: RRZEROSB after 20 ms (P20-P25.agc p.527)
 *
 * FUNCTION NAME: agc_task_rr_zero_counters
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or
 *          rescheduled itself, else the status of the waitlist or
 *          executive call that failed; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_rr_zero_counters(void)
{
    agc_erase_set_cdut(0U);
    agc_erase_set_cdus(0U);
    rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_ZERO_CDU);
    agc_status_t st = agc_waitlist_fixdelay(RR_DT_10SEC, AGC_TASK_RR_ZERO_SETTLED); /* 10 s */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * RRZEROSB after 10 s: mode, tracker lamp, return to RRRET.
 * AGC: RRZEROSB after 10 s (P20-P25.agc p.528)
 *
 * FUNCTION NAME: agc_task_rr_zero_settled
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
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

    agc_rr_settrkf(); /* SETTRKF */

    if (rr_zero_ret == RR_ZERO_FOR_RRZERO)
    {
        return (rr_endradar()); /* RRZ2    */
    }

    /* RRTURNON: wait 1 s before removing the turn-on flag, so that a
     * monitor reposition will not alarm.                                */
    return (agc_waitlist_fixdelay(RR_DT_1SEC, AGC_TASK_RR_TURNON_DONE));
}

/**********************************************************************
 * RRTURNON (p.526): waitlist task from RRAUTCHK (T4RUPT) when the RR
 * power-on-auto discrete appears.
 * AGC: RRTURNON (P20-P25.agc p.526)
 *
 * FUNCTION NAME: agc_task_rrturnon
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or
 *          rescheduled itself, else the status of the waitlist or
 *          executive call that failed; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_rrturnon(void)
{
    if (!AGC_ASSERT(rr_radmodes_test(RR_TURNONBT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_zero_begin(RR_ZERO_FOR_TURNON));
}

/**********************************************************************
 * RRTURNON after 1 s: the turn-on flag off.
 * AGC: RRTURNON (P20-P25.agc p.526)
 *
 * FUNCTION NAME: agc_task_rr_turnon_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or
 *          rescheduled itself, else the status of the waitlist or
 *          executive call that failed; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_rr_turnon_done(void)
{
    rr_radmodes_clear(RR_TURNONBT);

    if (!AGC_ASSERT(!rr_radmodes_test(RR_TURNONBT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* TASKOVER */
}

/**********************************************************************
 * RRZ2 (p.537): RRZEROSB, then ENDRADAR.
 * AGC: RRZ2 (P20-P25.agc p.537)
 *
 * FUNCTION NAME: agc_task_rr_zero
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or
 *          rescheduled itself, else the status of the waitlist or
 *          executive call that failed; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_rr_zero(void)
{
    if (!AGC_ASSERT(rr_radmodes_test(RR_RCDU0BIT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_zero_begin(RR_ZERO_FOR_RRZERO));
}

/**********************************************************************
 * RRZERO (p.537): zero the RR CDUs and find the antenna mode; the caller
 * then waits in RADSTALL. Nothing to do during a monitor reposition or
 * turn-on (RADNOOP); alarm 510 when the auto discrete is absent.
 * AGC: RRZERO (P20-P25.agc p.537)
 *
 * FUNCTION NAME: agc_rr_zero
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the waitlist status
 **********************************************************************/
agc_status_t agc_rr_zero(void)
{
    if (rr_radmodes_test(RR_BIT11_1))
    {
        return (rr_radnoop());
    }

    agc_inhint();
    rr_radmodes_set(RR_RCDU0BIT);
    agc_status_t st = agc_waitlist_call(1U, AGC_TASK_RR_ZERO); /* RRZ2 */

    if (st == AGC_STATUS_OK && rr_radmodes_test(RR_AUTOMBIT))
    {
        agc_alarm(RR_ALARM_NOT_AUTO);
    }

    agc_relint(); /* ROADBACK */

    return (st);
}

/* ---- single-axis drives (RRTONLY, RRSONLY, RR1AXIS pp.533-534) -------- */
/**********************************************************************
 * The step-th single-axis target of a client: trunnion (rrindex 0) or
 * shaft (1) and the desired CDU word. False: the client is done.
 * AGC: DORREPOS, REMODE, PREPOS29 target lists (P20-P25.agc pp.529-532,
 * 597)
 *
 * FUNCTION NAME: rr_axis_target
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------
 *   c         rr_axis_client_t  I    the client
 *   step      int               I    its step
 *   rrindex   int *             O    0 trunnion, 1 shaft
 *   rdes      uint16_t *        O    RDES: desired CDU word
 *
 * RETURNS: bool -- true while the client has a target at this step
 **********************************************************************/
static bool rr_axis_target(rr_axis_client_t c, int step, int *rrindex, uint16_t *rdes)
{
    /* REMODE: T 0 (180), S -45, T -130 (-50), T -180 (0), S -90 (0);
     * the targets of the present mode 1 (mode 2) and their axes.        */
    static const int      remode_axis[5] = {0, 1, 0, 0, 1};
    static const uint16_t remode_m1[5]   = { 0U, RR_W_M45, RR_W_M130, RR_W_180, RR_W_M90 };
    static const uint16_t remode_m2[5]   = {RR_W_180, RR_W_M45, RR_W_M50, 0U, 0U};

    /* DORREPOS: T to 0 (180), S to 0 (-90) in mode 1 (mode 2).           */
    static const uint16_t repos_m2[2] = {RR_W_180, RR_W_M90};
    bool                  m2          = rr_mode2(); /* mode 2 now */

    *rrindex = 0;

    switch (c)
    {
        case RR_AXIS_REPOSITION:

            if (step < 0 || step > 1)
            {
                return (false);
            }

            *rrindex = step;
            *rdes    = m2 ? repos_m2[step] : 0U; /* mode 1: 0 */

            return (true);
        case RR_AXIS_REMODE: /* REMODE pp.531-532                    */

            if (step < 0 || step > 4)
            {
                return (false);
            }

            *rrindex = remode_axis[step];
            *rdes    = m2 ? remode_m2[step] : remode_m1[step]; /* by mode */

            return (true);
        case RR_AXIS_PREPOSITION_POWERED: /* PREPOS29: T to -180 */
            *rdes = RR_W_180;

            return (step == 0);
        case RR_AXIS_CLIENT_COUNT:
        default:
            return (false); /* unreachable: cannot happen */
    }
} /* rr_axis_target */

/* Outcome of one RR1AX2 pass.                                           */
typedef enum rr_axis_outcome
{
    RR_AXIS_WAITING = 0, /* command sent, NXTRR1AX in .5 s          */
    RR_AXIS_DONE,        /* within 1 degree or out of auto: RRRET   */
    RR_AXIS_ABANDON      /* remode and reposition both on: REPOSRPT */
} rr_axis_outcome_t;

/**********************************************************************
 * RR1AX2: error RDES - CDU (MSU), command .59 x error to the one axis,
 * RROUT, again in .5 s.
 * AGC: RR1AX2 (P20-P25.agc p.534)
 *
 * FUNCTION NAME: rr_axis_pass
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   st        agc_status_t *  O    status of the step
 *
 * RETURNS: rr_axis_outcome_t -- WAITING (command sent), DONE (within 1 deg
 *          or out of auto), ABANDON (remode and reposition both on)
 **********************************************************************/
static rr_axis_outcome_t rr_axis_pass(agc_status_t *st)
{
    *st        = AGC_STATUS_OK;
    uint16_t m = agc_erase_radmodes(); /* RADMODES */

    if ((~m & (RR_REMODBIT | RR_REPOSBIT)) == 0U)
    {
        return (RR_AXIS_ABANDON);
    }

    uint16_t cdu = agc_erase_cdut(); /* trunnion (RRINDEX 0) */

    if (rr_axis.rrindex != 0)
    {
        cdu = agc_erase_cdus(); /* shaft */
    }

    int err = rr_msu(rr_axis.rdes, cdu); /* ITEMP1  */
    int cmd = rr_mp(err, RR_SPGAIN);     /* L       */

    if (rr_magsub(err, RR_1DEG) || (m & RR_AUTOMBIT) != 0U)
    {
        return (RR_AXIS_DONE);
    }

    agc_erase_set_tang(0, 0); /* the other axis gets no command */
    agc_erase_set_tang(1, 0);
    agc_erase_set_tang(rr_axis.rrindex, (agc_sp_t) cmd);
    rr_rrout();
    *st = agc_waitlist_fixdelay(RR_DT_HALFSEC, AGC_TASK_RR_ONE_AXIS);

    return (RR_AXIS_WAITING);
}

/**********************************************************************
 * REPOSRPT (p.530): reposition over; designate if someone is waiting
 * (BEGDES: a remode first when requested), else drop the error counter
 * enable. True when a remode axis must run next.
 * AGC: REPOSRPT (P20-P25.agc p.530)
 *
 * FUNCTION NAME: rr_reposrpt
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   st        agc_status_t *  O    status of the step
 *
 * RETURNS: bool -- true when a remode axis must run next
 **********************************************************************/
static bool rr_reposrpt(agc_status_t *st)
{
    *st = AGC_STATUS_OK;
    rr_radmodes_clear(RR_REPOSBIT);

    if (!rr_radmodes_test(RR_DESIGBIT))
    {
        rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_ENB_ECTR);

        return (false); /* TASKOVER */
    }

    if (!rr_radmodes_test(RR_REMODBIT))
    {
        *st = rr_stdesig(); /* BEGDES   */

        return (false);
    }

    rr_axis.client = RR_AXIS_REMODE; /* REMODE   */
    rr_axis.step   = 0;

    return (rr_axis_target(RR_AXIS_REMODE, 0, &rr_axis.rrindex, &rr_axis.rdes));
}

/**********************************************************************
 * A client's single-axis drive is done: next target, or the client's
 * own ending. True when another axis must run.
 * AGC: RRRET returns of RR1AXIS (P20-P25.agc pp.530-532)
 *
 * FUNCTION NAME: rr_axis_advance
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   st        agc_status_t *  O    status of the step
 *
 * RETURNS: bool -- true when another axis must run
 **********************************************************************/
static bool rr_axis_advance(agc_status_t *st)
{
    *st = AGC_STATUS_OK;
    rr_axis.step++;

    if (rr_axis_target(rr_axis.client, rr_axis.step, &rr_axis.rrindex, &rr_axis.rdes))
    {
        return (true);
    }

    switch (rr_axis.client)
    {
        case RR_AXIS_REPOSITION:
            return (rr_reposrpt(st));
        case RR_AXIS_REMODE:
            /* End of REMODE (p.532): new mode, remode flag off; back to the
             * designate (STDESIG) or to the caller waiting in RADSTALL.     */
            rr_mode_invert(); /* RMODINV  */
            rr_radmodes_clear(RR_REMODBIT);

            if (rr_radmodes_test(RR_DESIGBIT))
            {
                *st = rr_stdesig(); /* designate waits */
            }
            else
            {
                *st = agc_task_rr_goodend(); /* RADSTALL caller */
            }

            return (false);
        case RR_AXIS_PREPOSITION_POWERED: /* PREPOS29: repositioning off */
            rr_radmodes_clear(RR_REPOSBIT);

            return (false);
        case RR_AXIS_CLIENT_COUNT:
        default:
            return (false); /* unreachable: cannot happen */
    }
}

/**********************************************************************
 * Run single-axis drives until one has to wait .5 s (the AGC's chain of
 * RR1AXIS returns, as a bounded loop).
 * AGC: RR1AXIS chain (P20-P25.agc pp.533-534)
 *
 * FUNCTION NAME: rr_axis_run
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed;
 *          AGC_STATUS_NO_CONVERGE when its step bound is exhausted
 **********************************************************************/
static agc_status_t rr_axis_run(void)
{
    agc_status_t st = AGC_STATUS_OK; /* status of the last step */

    for (int i = 0; i < RR_AXIS_MAX_STEPS; i++)
    {
        rr_axis_outcome_t o    = rr_axis_pass(&st); /* RR1AX2 */
        bool              more = false;             /* another axis */

        if (o == RR_AXIS_WAITING)
        {
            return (st);
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
            return (st);
        }
    }

    return (AGC_STATUS_NO_CONVERGE); /* step bound exhausted */
}

/**********************************************************************
 * Start a client's single-axis drives at a step (RRTONLY / RRSONLY with
 * RRRET).
 * AGC: RRTONLY, RRSONLY, RR1AXIS (P20-P25.agc pp.533-534)
 *
 * FUNCTION NAME: rr_axis_start
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------
 *   client    rr_axis_client_t  I    the client
 *   step      int               I    its first step
 *   rrindex   int               I    0 trunnion, 1 shaft
 *   rdes      uint16_t          I    RDES: desired CDU word
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the last step
 **********************************************************************/
agc_status_t rr_axis_start(rr_axis_client_t client, int step, int rrindex, uint16_t rdes)
{
    if (!AGC_ASSERT((int) client >= 0 && client < RR_AXIS_CLIENT_COUNT) || !AGC_ASSERT(rrindex == 0 || rrindex == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rr_axis.client  = client;
    rr_axis.step    = step;
    rr_axis.rrindex = rrindex;
    rr_axis.rdes    = (uint16_t) (rdes & RR_WORD_MASK);

    return (rr_axis_run());
}

/**********************************************************************
 * NXTRR1AX: the next .5 s pass of the single-axis drive.
 * AGC: NXTRR1AX (P20-P25.agc p.534)
 *
 * FUNCTION NAME: agc_task_rr_one_axis
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or
 *          rescheduled itself, else the status of the waitlist or
 *          executive call that failed; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_rr_one_axis(void)
{
    if (!AGC_ASSERT(rr_axis.client < RR_AXIS_CLIENT_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_axis_run());
}

/**********************************************************************
 * DORREPOS (pp.529-530)
 * Waitlist task from the T4RUPT gimbal monitor: drive the RR to a safe
 * position (T 0, S 0 in mode 1; T 180, S -90 in mode 2).
 * AGC: DORREPOS (P20-P25.agc p.529)
 *
 * FUNCTION NAME: agc_task_dorrepos
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or
 *          rescheduled itself, else the status of the waitlist or
 *          executive call that failed; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_dorrepos(void)
{
    rr_setrrecr();
    agc_status_t st; /* 20 ms */

    st = agc_waitlist_fixdelay(RR_DT_20MS, AGC_TASK_RR_REPOSITION_DRIVE);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * DORREPOS after 20 ms: the single-axis drives to the safe position.
 * AGC: DORREPOS (P20-P25.agc p.529)
 *
 * FUNCTION NAME: agc_task_rr_reposition_drive
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or
 *          rescheduled itself, else the status of the waitlist or
 *          executive call that failed; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_rr_reposition_drive(void)
{
    int      idx  = 0;  /* RRINDEX */
    uint16_t rdes = 0U; /* RDES */
    bool     ok;        /* first target */

    ok = rr_axis_target(RR_AXIS_REPOSITION, 0, &idx, &rdes);

    if (!AGC_ASSERT(ok))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_axis_start(RR_AXIS_REPOSITION, 0, idx, rdes));
}

/**********************************************************************
 * REMODE (pp.531-532): T to 0 (180), S to -45, T to -130 (-50), T to
 * -180 (0), S to -90 (0), new mode; then STDESIG or RGOODEND.
 * AGC: REMODE (P20-P25.agc pp.531-532)
 *
 * FUNCTION NAME: rr_remode_start
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the status of the first
 *          drive step; AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t rr_remode_start(void)
{
    int      idx  = 0;  /* RRINDEX */
    uint16_t rdes = 0U; /* RDES */
    bool     ok;        /* first target */

    ok = rr_axis_target(RR_AXIS_REMODE, 0, &idx, &rdes);

    if (!AGC_ASSERT(ok))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_axis_start(RR_AXIS_REMODE, 0, idx, rdes));
}

/**********************************************************************
 * REMODE as a waitlist task (R29REM0J: "remode must run as a task").
 * AGC: REMODE (P20-P25.agc p.531)
 *
 * FUNCTION NAME: agc_task_rr_remode
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or
 *          rescheduled itself, else the status of the waitlist or
 *          executive call that failed; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_rr_remode(void)
{
    if (!AGC_ASSERT(rr_radmodes_test(RR_REMODBIT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_remode_start());
}
