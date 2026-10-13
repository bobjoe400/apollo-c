/**********************************************************************
 * FILE NAME: t4rupt_lm.c
 *
 * PURPOSE:
 *   The LM-only part of T4RUPT: the rendezvous radar inbit and gimbal
 *   limit monitors (RRAUTCHK, RRCDUCHK, RRGIMON / MONREPOS) and the
 *   gimbal-to-pilot axis matrix for the LM DAP (GPMATRIX = DAPT4S); and
 *   the RR moding primitives the monitor shares with the radar programs
 *   (MAGSUB, RRLIMCHK, SETTRKF / RRTRKF, TRKFLON +1, FLIP).
 *
 * AGC SOURCE:
 *   Luminary099 T4RUPT_PROGRAM.agc pp.182-189  (RRAUTCHK, RRCDUCHK,
 *   RRGIMON, MONREPOS, GPMATRIX); P20-P25.agc pp.521-525 (MAGSUB,
 *   RRLIMCHK, SETTRKF, RRTRKF, TRKFLON, FLIP); Comanche055 has no
 *   counterpart.
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name      Type      I/O  Scale        Description
 *   --------  --------  ---  -----------  ----------------------------
 *   RADMODES  uint16_t  I/O  bits         FLGWRD12, radar mode bits
 *   CDUX/Z    uint16_t  I    2's comp pi  outer / middle gimbal CDU
 *   CDUT/S    uint16_t  I    2's comp pi  RR trunnion / shaft CDU
 *   M11..M32  agc_sp_t  O    1            DAP matrix (owned here)
 *   DSPTAB+11 uint16_t  I/O  bits         tracker fail lamp (FLIP)
 *   IMODES33  uint16_t  I    bits         lamp test (SETTRKF)
 *
 * EXTERNAL REFERENCES:
 *   Name               Description
 *   -----------------  ----------------------------------------------
 *   agc_waitlist_call  WAITLIST: RRTURNON (10 ms), DORREPOS (20 ms)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarm 00515 (RR CDU fail while RNDVZFLG, P20/P22).  BAD_ARG when the
 *   waitlist rejects a call.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only; runs inside T4RUPT (agc_t4rupt, t4rupt.c).  T4JUMP runs
 *   RRAUTCHK at phases 1 and 5 (falling into GPMATRIX through NORRGMON)
 *   and DAPT4S at phases 3 and 7, so GPMATRIX runs 4 times a second.
 *
 * NOTES:
 *   RADMODES EQUALS FLGWRD12 (FLAGWORD_ASSIGNMENTS p.85): the accessors
 *   below name the flagword, there is no second copy.
 *
 * ALGORITHM:
 *         | sin MG            0        1 |
 *   M   = | cos MG cos OG     sin OG   0 |     (AOG = CDUX, AMG = CDUZ)
 *    GP   | -cos MG sin OG    cos OG   0 |
 *   SPSIN/SPCOS of the raw CDU words (read as ones' complement, scaled
 *   at pi), the products by EXTEND MP (upper word).
 **********************************************************************/
#include <stdlib.h>

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"

#define RADMODES_WORD     12                         /* RADMODES EQUALS FLGWRD12          */
#define AUTOMBIT          000002u                    /* BIT2: RR not in auto (inverted)   */
#define RCDUFBIT          000100u                    /* BIT7: RR CDU OK                   */
#define REPOSBIT          002000u                    /* BIT11: reposition in progress     */
#define WORD15            077777u                    /* one AGC word, 15 bits             */
#define BIT6              000040u                    /* channel 33 bit 6: LR position 2   */
#define RMODINIT          000102u                    /* RMODINIT: bits 7 and 2            */
#define OCT05776          005776u                    /* keeps all but CDES/REMODE/REPOS.. */
#define OCT10001          010001u                    /* RRCDUZRO and TURNON bits          */
#define OCT32002          032002u                    /* remode, zeroing, monitor, no auto */
#define OCT20002          020002u                    /* tracker and error counter enable  */
#define CDU_SIGN          040000u                    /* sign bit of a CDU counter         */
#define RRTURNON_DELAY_CS 1u                         /* CAF ONE: RRTURNON in 10 ms        */
#define DORREPOS_DELAY_CS 2u                         /* CAF TWO: DORREPOS in 20 ms        */
#define ALARM_RR_CDU_FAIL ((agc_alarm_code_t) 00515) /* in P20/P22     */
#define ANTENBIT          004000u                    /* BIT12 ANTENFLG: RR antenna in mode 2 */
#define RR_RRDATABT       000010u                    /* BIT4 RRDATAFL: RR data fail       */
#define RR_13_7_2         010102u                    /* 13,7,2: zeroing, CDU OK, not in auto */
#define RR_TRKF_LAMP      000200u                    /* DSPTAB +11D BIT8: tracker fail lamp */
#define RR_DSPTAB_11      11                         /* DSPTAB +11D                       */

/* RRLIMCHK limits (p.523): DEC fractions of 180 degrees (x 16384).      */
#define RR_LIM_T1  (-5006)  /* DEC -.30555: 55 degrees               */
#define RR_LIM_S1  (-5871)  /* DEC -.35833: 64.5 degrees             */
#define RR_5_5DEGS 501      /* 5.5DEGS DEC .03056                    */
#define RR_82DEGS  7464     /* 82DEGS DEC .45556                     */
#define RR_LIM_S2  (-5188)  /* DEC -.31667: 57 degrees               */
#define RR_LIM_T2  (-11378) /* DEC -.69444: 125 degrees              */

static agc_sp_t m11;   /* M11 = SIN(MG), scaled at 1             */
static agc_sp_t m21;   /* M21 = COS(OG)COS(MG), scaled at 1      */
static agc_sp_t m22;   /* M22 = SIN(OG) (also MR22), scaled at 1 */
static agc_sp_t m31;   /* M31 = -SIN(OG)COS(MG), scaled at 1     */
static agc_sp_t m32;   /* M32 = COS(OG) (also MR23), scaled at 1 */
static agc_sp_t cosmg; /* COSMG = COS(MG), scaled at 1           */

/**********************************************************************
 * RADMODES (ERASABLE_ASSIGNMENTS; EQUALS FLGWRD12, FLAGWORD_ASSIGNMENTS
 * p.85) -- the radar mode bit word, read
 *
 * RETURNS: uint16_t -- RADMODES (15 bits)
 **********************************************************************/
uint16_t agc_erase_radmodes(void)
{
    return (agc_flagword_get(RADMODES_WORD));
}

/**********************************************************************
 * RADMODES (FLGWRD12, FLAGWORD_ASSIGNMENTS p.85) -- the radar mode bit
 * word, written (TS RADMODES: every caller forms a 15-bit word)
 *
 * ARGUMENTS:
 *   v  uint16_t  I   new RADMODES, 15 bits
 **********************************************************************/
void agc_erase_set_radmodes(uint16_t v)
{
    agc_flagword_set(RADMODES_WORD, v);
}

/**********************************************************************
 * STARTSUB (FRESH_START_AND_RESTART.agc L p.219): CAF BIT6, RAND CHAN33,
 * AD RMODINIT, TS RADMODES -- on every fresh start and hardware restart
 * the radar modes restart from RMODINIT (RR not in auto, bit 2; RR CDU
 * OK, bit 7) plus the LR position bit as channel 33 reads it.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_t4_radmodes_init(void)
{
    agc_erase_set_radmodes((uint16_t) ((agc_chan_read((int) AGC_CHAN_CHAN33) & BIT6) + RMODINIT));
}

/**********************************************************************
 * M11 (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- SIN(MG), read by the DAP
 *
 * RETURNS: M11, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m11(void)
{
    return (m11);
}

/**********************************************************************
 * M21 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- COS(OG)COS(MG)
 *
 * RETURNS: M21, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m21(void)
{
    return (m21);
}

/**********************************************************************
 * M22 (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- SIN(OG), also MR22
 *
 * RETURNS: M22, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m22(void)
{
    return (m22);
}

/**********************************************************************
 * M31 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- -SIN(OG)COS(MG)
 *
 * RETURNS: M31, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m31(void)
{
    return (m31);
}

/**********************************************************************
 * M32 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- COS(OG), also MR23
 *
 * RETURNS: M32, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m32(void)
{
    return (m32);
}

/**********************************************************************
 * COSMG (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- COS(MG), a factor only
 *
 * RETURNS: COSMG, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_cosmg(void)
{
    return (cosmg);
}

/**********************************************************************
 * AGC: none (CAE CDUZ / CAE CDUX in GPMATRIX, p.188) -- a CDU counter as
 * SPSIN sees it
 *
 * The counter is a two's-complement angle; CAE loads the raw word and
 * SPSIN/SPCOS treat it as ones' complement, so a negative angle reads one
 * LSB (2^-14 pi) low in magnitude, as in the AGC.
 *
 * ARGUMENTS:
 *   cdu  uint16_t  I   CDU counter word, 15 bits, 2's complement, 2^0 pi
 *
 * RETURNS: the same word as a ones'-complement SP value, scaled at pi
 **********************************************************************/
static agc_sp_t cdu_sp(uint16_t cdu)
{
    int v = ((cdu & CDU_SIGN) != 0u) ? -(int) (WORD15 - cdu) : (int) cdu; /* ones' complement, LSB 2^-14 pi */

    return ((agc_sp_t) v);
}

/**********************************************************************
 * AGC: none (EXTEND / MP in GPMATRIX, p.189) -- upper word of an SP x SP
 * product
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I   multiplicand, scaled at 1
 *   b  agc_sp_t  I   multiplier, scaled at 1
 *
 * RETURNS: the high word of a*b (what MP leaves in A), scaled at 1
 **********************************************************************/
static agc_sp_t mp_hi(agc_sp_t a, agc_sp_t b)
{
    agc_dp_t p = agc_sp_mul(a, b); /* A,L product, scaled at 1 */

    return (agc_dp_high(p));
}

/**********************************************************************
 * GPMATRIX = DAPT4S = NORRGMON (T4RUPT_PROGRAM.agc pp.188-189) -- the SP
 * gimbal-to-pilot matrix elements for the LM DAP
 *
 * (ENDDAPT4 = RESUME.)
 **********************************************************************/
void agc_t4_gimbal_matrix(void)
{
    agc_sp_t mg = cdu_sp(agc_erase_cduz()); /* AMG, scaled at pi */
    agc_sp_t og = cdu_sp(agc_erase_cdux()); /* AOG, scaled at pi */

    m11   = agc_sp_sin(mg);                /* SIN(MG)          */
    cosmg = agc_sp_cos(mg);                /* COS(MG)          */
    m22   = agc_sp_sin(og);                /* SIN(OG)          */
    m31   = mp_hi((agc_sp_t) -m22, cosmg); /* CS M22, MP COSMG */
    m32   = agc_sp_cos(og);                /* COS(OG)          */
    m21   = mp_hi(m32, cosmg);             /* MP COSMG         */
}

/* ---- RR moding primitives shared with the radar routines --------------
 * MAGSUB, RRLIMCHK and the tracker-fail lamp (SETTRKF, TRKFLON, FLIP) are
 * P20-P25 routines (Luminary099 P20-P25.agc pp.521-525). T4RUPT's RR
 * monitor below is their lowest caller, so they live with RADMODES here;
 * the radar programs (src/lm/radar) call them through io/service.h.     */

/**********************************************************************
 * MAGSUB (p.521): |a| + k <= 0 returns to L+3 (within), else L+2.
 * AGC: MAGSUB (P20-P25.agc p.521)
 *
 * FUNCTION NAME: agc_rr_magsub
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   a         int   I    the value, CDU units
 *   k         int   I    minus the limit (negative), CDU units
 *
 * RETURNS: bool -- true when |a| + k <= 0 (within the limit, L+3)
 **********************************************************************/
bool agc_rr_magsub(int a, int k)
{
    int m = (a < 0) ? -a : a; /* |a| */

    return ((m + k) <= 0);
}

/**********************************************************************
 * RRLIMCHK (pp.522-523)
 * Mode 1: |T| <= 55 and |S + 5.5| <= 64.5 degrees (S from -70 to +59).
 * Mode 2: |S + 82| <= 57 (S from -139 to -25) and |T| > 125 degrees.
 * True = within the limits of the present mode (L+3). Also called by
 * the T4RUPT gimbal monitor (RRGIMON, below) on CDUT, CDUS.
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
    int tv = (int) cdu_sp((uint16_t) (t & WORD15)); /* trunnion, CDU units */
    int sv = (int) cdu_sp((uint16_t) (s & WORD15)); /* shaft, CDU units */

    if ((agc_erase_radmodes() & ANTENBIT) == 0u) /* mode 1 (ANTENFLG clear) */
    {
        return (agc_rr_magsub(tv, RR_LIM_T1) && agc_rr_magsub(sv + RR_5_5DEGS, RR_LIM_S1));
    }

    if (!agc_rr_magsub(sv + RR_82DEGS, RR_LIM_S2))
    {
        return (false);
    }

    return (!agc_rr_magsub(tv, RR_LIM_T2));
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
 * FUNCTION NAME: agc_rr_lamp_put
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   lamp      uint16_t  I    lamp bit in DSPTAB +11D
 *   on        bool      I    the desired lamp state
 **********************************************************************/
void agc_rr_lamp_put(uint16_t lamp, bool on)
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

    agc_rr_lamp_put(RR_TRKF_LAMP, on);
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
 * AGC: none (CS bits / EXTEND / WAND chan, pp.184 and 187) -- clear bits
 * of an output channel
 *
 * ARGUMENTS:
 *   chan  int       I   channel number (CHAN12 here)
 *   bits  uint16_t  I   bits to clear, 15 bits
 **********************************************************************/
static void ch_clear(int chan, uint16_t bits)
{
    agc_chan_and(chan, (uint16_t) (~(uint32_t) bits & WORD15));
}

/**********************************************************************
 * RRGIMON / MONREPOS (T4RUPT_PROGRAM.agc pp.186-187) -- RR gimbal limit
 * monitor (R25)
 *
 * Skipped when NORRMON is set, when SERVICER runs with MUNFLAG, or while
 * RADMODES shows remode, CDU zeroing, monitor or RR not in auto (bits 14,
 * 13, 11, 2).  Out of limits: set REPOSBIT, disable the tracker and the
 * error counters (channel 12 bits 14, 2), DORREPOS in 20 ms.
 *
 * RETURNS: AGC_STATUS_OK, or the WAITLIST status of DORREPOS
 **********************************************************************/
static agc_status_t t4_rrgimon(void)
{
    if (agc_flag_test((agc_flag_id_t) AGC_FLAG_NORRMON) || (agc_flag_test((agc_flag_id_t) AGC_FLAG_AVEGFLAG) && agc_flag_test((agc_flag_id_t) AGC_FLAG_MUNFLAG)) || ((agc_erase_radmodes() & OCT32002) != 0u))
    {
        return (AGC_STATUS_OK); /* TCF NORRGMON */
    }

    if (agc_rr_limchk(agc_erase_cdut(), agc_erase_cdus()))
    {
        return (AGC_STATUS_OK); /* angles in limits (RRLIMCHK L+3) */
    }

    agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() + REPOSBIT));
    ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) OCT20002);

    return (agc_waitlist_call(DORREPOS_DELAY_CS, AGC_TASK_DORREPOS));
}

/**********************************************************************
 * RRCDUCHK (T4RUPT_PROGRAM.agc pp.184-185) -- RR CDU fail monitor
 * (channel 30 bit 7)
 *
 * A change of the fail bit while the RR is in auto is copied into
 * RADMODES bit 7; a failure during P20/P22 (RNDVZFLG) is alarm 00515;
 * SETTRKF then updates the tracker fail lamp.
 *
 * RETURNS: true to go on to RRGIMON, false when the AGC goes straight to
 *          NORRGMON (bit changed with the RR not in auto)
 **********************************************************************/
static bool t4_rrcduchk(void)
{
    uint16_t chg = (uint16_t) ((agc_erase_radmodes() ^ agc_chan_read((int) AGC_CHAN_CHAN30)) & RCDUFBIT); /* changed */

    if (chg != 0u) /* BZF RRGIMON */
    {
        if ((agc_erase_radmodes() & AUTOMBIT) != 0u)
        {
            return (false); /* not in auto: TCF NORRGMON */
        }

        agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() ^ RCDUFBIT));

        if (((agc_erase_radmodes() & RCDUFBIT) == 0u) && agc_flag_test((agc_flag_id_t) AGC_FLAG_RNDVZFLG))
        {
            agc_alarm(ALARM_RR_CDU_FAIL);
        }

        agc_rr_settrkf(); /* TRKFLCDU */
    }

    return (true);
}

/**********************************************************************
 * RRAUTCHK (T4RUPT_PROGRAM.agc pp.182-183) -- RR power-on-auto monitor
 * (channel 33 bit 2), then RRCDUCHK, RRGIMON and NORRGMON = GPMATRIX
 *
 * A change of the auto bit updates RADMODES and clears continuous
 * designate, remode, reposition, CDU zero and turn-on (MASK OCT05776).
 * Just switched on: set RRCDUZRO and TURNON and start RRTURNON in 10 ms,
 * skipping the two monitors this pass.  Switched off: disable the RR CDU
 * error counters (channel 12 bit 2, RRCDUCHK -3).
 *
 * RETURNS: AGC_STATUS_OK or the WAITLIST status
 **********************************************************************/
agc_status_t agc_t4_rr_monitor(void)
{
    agc_status_t st  = AGC_STATUS_OK;                                                                         /* WAITLIST  */
    uint16_t     chg = (uint16_t) ((agc_erase_radmodes() ^ agc_chan_read((int) AGC_CHAN_CHAN33)) & AUTOMBIT); /* changed */

    if (chg != 0u)
    {
        /* LXCH RADMODES / RXOR LCHAN / MASK OCT05776 / TS RADMODES */
        agc_erase_set_radmodes((uint16_t) ((agc_erase_radmodes() ^ chg) & OCT05776));

        if ((agc_erase_radmodes() & AUTOMBIT) == 0u)
        {
            /* just on: CA OCT10001 / ADS RADMODES */
            agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() + OCT10001));
            st = agc_waitlist_call(RRTURNON_DELAY_CS, AGC_TASK_RRTURNON);

            if (st != AGC_STATUS_OK)
            {
                return (st); /* waitlist full: BAILOUT */
            }

            agc_t4_gimbal_matrix(); /* TCF NORRGMON */

            return (AGC_STATUS_OK);
        }

        ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) AUTOMBIT);
    }

    if (t4_rrcduchk())
    {
        st = t4_rrgimon();
    }

    if (st == AGC_STATUS_OK)
    {
        agc_t4_gimbal_matrix(); /* NORRGMON */
    }

    return (st);
} /* agc_t4_rr_monitor */
