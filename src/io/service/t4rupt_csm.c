/**********************************************************************
 * FILE NAME: t4rupt_csm.c
 *
 * PURPOSE:
 *   The CSM-only part of T4RUPT: the optics monitor and zero routines
 *   (OPTMON, PROCESSW and its switch-transition branches, ENDZOPT,
 *   OCDUFTST) and the optics CDU driving program (OPTDRIVE).
 *
 * AGC SOURCE:
 *   Comanche055 T4RUPT_PROGRAM.agc pp.160-169  (OPTICS MONITORING AND
 *   ZERO ROUTINES, OPTICS CDU DRIVING PROGRAM); Luminary099 has none.
 *
 * EXTERNAL VARIABLES:  (AGC erasables; the first group is owned here)
 *   Name          Type      I/O  Scale        Description
 *   ------------  --------  ---  -----------  --------------------------
 *   OPTMODES      uint16_t  I/O  bits         optics mode bits
 *   OPTIND        agc_sp_t  I/O  -0,-1,0,1    coarse optics state
 *   SWSAMPLE      int       I/O  sign         optics mode switch
 *   DESOPMOD      int       I/O  sign         previous switch setting
 *   WTOPTION      int       I/O  480 ms       zero-optics return option
 *   ZOPTCNT       int       I/O  480 ms       32-sample wait
 *   ZONE          int       I/O  2^14 = pi    shaft stop monitor
 *   COMMANDO      int [2]   I/O  pulses       drive commands T, S
 *   DESOPTT/S     uint16_t  I    2's comp pi  desired optics (align)
 *   CDUT/S        uint16_t  I/O  2's comp pi  optics CDUs
 *   CDUTCMD/SCMD  agc_sp_t  O    pulses       drive out-counters
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
 *   agc_align_ram          DESOPTT / DESOPTS (prog-align)
 *   agc_erase_set_cdutcmd  CDUTCMD / CDUSCMD out-counters (core)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarm 00116 (switch moved while zero optics processing), 00120
 *   (optics not zeroed since fresh start).  BAD_ARG on out-of-range
 *   state.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only; runs in T4RUPT (T4JUMP phases 0, 4 OPTDRIVE; 1, 5 OPTMON)
 *   and as the waitlist tasks ENDZOPT / ENDZOPT2.
 *
 * NOTES:
 *   OPTIND: -1 no coarse optics, 0 / 1 coarse optics working (also the
 *   trunnion/shaft index in OPTDRIVE), -0 optics inhibited (TVC
 *   takeover, P40 S40.6).  DEVIATION: -0 is AGC_OPTIND_NEG0 (-2).
 *   SWSAMPLE / DESOPMOD: the AGC stores BIT5 - 1 = 17 octal for CSC,
 *   0 for manual, -1 for zero optics, and only ever tests the sign; here
 *   CSC is +1.
 *   The listing's "ZOPFIN1" labels are typos for ZOPFINI (p.161), and
 *   its header comment gives the L+1/L+2 returns reversed: the code
 *   returns L+2 when zero optics is processing.
 *
 * ALGORITHM:
 *   OPTMON (every 480 ms) watches the optics CDU fail bit (channel 30
 *   bit 7) and the optics mode switch (channel 33 bits 4, 5: ZERO OPTICS
 *   / MANUAL / CMC) and sequences optics zeroing (a 32-sample wait, 200
 *   ms of ZERO OCDU, 200 ms for the CDUs to resynchronise).  OPTDRIVE
 *   drives the trunnion and shaft CDUs towards DESOPTT / DESOPTS with at
 *   most 165 pulses per pass, keeping the shaft away from its stops.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"

#define BIT1         000001u    /* BIT1: ZERO OCDU / ZOPT task working     */
#define BIT2         000002u    /* BIT2: OCDU error counters / fail inhib  */
#define BIT5         000020u    /* BIT5: optics switch at CMC (inverted)   */
#define BIT7         000100u    /* BIT7: OCDU fail (channel 30, inverted)  */
#define BIT8         000200u    /* BIT8: OPTICS CDU FAIL lamp (DSPTAB+11D) */
#define BIT9         000400u    /* BIT9: coarse optics since fresh start   */
#define BIT10        001000u    /* BIT10: optics zeroed since fresh start  */
#define BIT15        040000u    /* BIT15: sign / lamp word "send" bit      */
#define POSMAX       037777u    /* POSMAX                                  */
#define WORD15       077777u    /* one AGC word, 15 bits                   */
#define OCTHIRTY     000030u    /* BITS4&5: optics mode switch bits        */
#define OPT_SIX      000006u    /* SIX: ZOPT processing + OCDUFAIL inhibit */
#define OPT_SEVEN    000007u    /* SEVEN: the three zero optics bits       */
#define OCT11_12     006000u    /* 11,12 = PRIO6: drive CDU S and T        */
#define OPTINITF     000130u    /* OPTINITF: fresh start OPTMODES          */
#define OPTINITR     000430u    /* OPTINITR: OPTMODES bits kept on restart */
#define ZOPT_SAMPLES 32         /* OCT40: 32-sample wait                   */
#define WTOPT_RETURN 11         /* OCT13: 5 s return option (480 ms each)  */
#define ZOPT_HOLD_CS 20u        /* 200MS = OCT24, cs                       */
#define DEG20_CDU    7199       /* 20DEGS DEC 7199, 2^14 = pi              */
#define MAXPLS       165        /* -MAXPLS (DEC -165, was -80), pulses     */
#define DEG45_CDU    4096       /* -(13,14,15) + 1: 45 deg, 2^14 = pi      */
#define DEG90_CDU    8192       /* -NEG1/2: 90 deg, 2^14 = pi              */
#define CDU_SIGN     040000u    /* sign bit of a CDU counter               */
#define LAMPWORD     11         /* DSPTAB +11D                             */

#define ALARM_ZOPT_SWITCH    ((agc_alarm_code_t) 00116) /* moved in ZOPT */
#define ALARM_OPT_NOT_ZEROED ((agc_alarm_code_t) 00120) /* not zeroed    */

static uint16_t optmodes = OPTINITF; /* OPTMODES bits                     */
static agc_sp_t optind   = -1;       /* OPTIND, unscaled (NEG0 = -0)      */
static int      swsample;            /* SWSAMPLE: +1 CMC, 0 man, -1 zero  */
static int      desopmod;            /* DESOPMOD: previous SWSAMPLE       */
static int      wtoption;            /* WTOPTION countdown, 480 ms passes */
static int      zoptcnt;             /* ZOPTCNT, 480 ms samples           */
static int      zone;                /* ZONE, CDUS ones', 2^14 = pi       */
static int      commando[2];         /* COMMANDO T, S, CDU pulses         */

/**********************************************************************
 * AGC: none (ranges of the optics erasables, p.160-169)
 *
 * RETURNS: true when every optics erasable owned here is in range
 **********************************************************************/
static bool optics_ok(void)
{
    return ((optmodes <= WORD15) &&
            (optind >= AGC_OPTIND_NEG0) && (optind <= 1) &&
            (swsample >= -1) && (swsample <= 1) &&
            (desopmod >= -1) && (desopmod <= 1) &&
            (wtoption >= 0) && (wtoption <= WTOPT_RETURN) &&
            (zoptcnt >= 0) && (zoptcnt <= ZOPT_SAMPLES) &&
            (zone >= -(int) POSMAX) && (zone <= (int) POSMAX));
}

/**********************************************************************
 * OPTMODES (ERASABLE_ASSIGNMENTS.agc, CM) -- the optics mode bit word, read
 *
 * RETURNS: OPTMODES, 15 bits (0 in an LM build)
 **********************************************************************/
uint16_t agc_erase_optmodes(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(optmodes <= WORD15))
    {
        return (0); /* no valid result */
    }

    return (optmodes);
}

/**********************************************************************
 * OPTMODES (ERASABLE_ASSIGNMENTS.agc, CM) -- the optics mode bit word,
 * written
 *
 * ARGUMENTS:
 *   v  uint16_t  I   new OPTMODES, 15 bits
 **********************************************************************/
void agc_erase_set_optmodes(uint16_t v)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return;
    }

    if (AGC_ASSERT(v <= WORD15))
    {
        optmodes = v;
    }
}

/**********************************************************************
 * OPTIND (ERASABLE_ASSIGNMENTS.agc, CM) -- coarse optics state, read
 *
 * RETURNS: OPTIND: -1, 0, 1, or AGC_OPTIND_NEG0 for -0
 **********************************************************************/
agc_sp_t agc_erase_optind(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (-1); /* no coarse optics */
    }

    if (!AGC_ASSERT(optind >= AGC_OPTIND_NEG0 && optind <= 1))
    {
        return (-1); /* no coarse optics */
    }

    return (optind);
}

/**********************************************************************
 * OPTIND (ERASABLE_ASSIGNMENTS.agc, CM) -- coarse optics state, written
 * (V41 N91 COARSE OPTICS, P40 S40.6 TVC takeover)
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I   -1, 0, 1, or AGC_OPTIND_NEG0 for -0
 **********************************************************************/
void agc_erase_set_optind(agc_sp_t v)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return;
    }

    if (AGC_ASSERT(v >= AGC_OPTIND_NEG0 && v <= 1))
    {
        optind = v;
    }
}

/**********************************************************************
 * SWSAMPLE (ERASABLE_ASSIGNMENTS, CM; set in OPTMON p.160) -- the optics
 * mode switch as last sampled (read by V41 N91 and R52)
 *
 * RETURNS: +1 computer (CSC), 0 manual, -1 zero optics
 **********************************************************************/
agc_sp_t agc_erase_swsample(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (0); /* manual: no computer control */
    }

    if (!AGC_ASSERT(swsample >= -1 && swsample <= 1))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) swsample);
}

/**********************************************************************
 * DOFSTART (FRESH_START_AND_RESTART.agc p.184, CM: OPTIND = -1,
 * OPTMODES = OPTINITF, WTOPTION = 0) / GOPROG (p.188: OPTMODES =
 * (OPTMODES & OPTINITR) + BIT7) -- optics state at fresh start / restart
 *
 * ARGUMENTS:
 *   fresh  bool  I   true for a fresh start, false for a restart
 *
 * RETURNS: AGC_STATUS_OK, or BAD_ARG if the state is out of range
 **********************************************************************/
agc_status_t agc_t4_optics_reset(bool fresh)
{
    if (fresh)
    {
        optind   = -1;
        optmodes = OPTINITF;
        wtoption = 0;
    }
    else
    {
        optmodes = (uint16_t) ((optmodes & OPTINITR) + BIT7);
    }

    if (!AGC_ASSERT(optmodes <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(optics_ok()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (CS bits / EXTEND / WAND chan) -- clear bits of an output
 * channel
 *
 * ARGUMENTS:
 *   chan  int       I   channel number
 *   bits  uint16_t  I   bits to clear, 15 bits
 **********************************************************************/
static void ch_clear(int chan, uint16_t bits)
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
 * AGC: none (CA / CCS of a CDU counter) -- a CDU counter word read as
 * the AGC's ones' complement SP value
 *
 * ARGUMENTS:
 *   cdu  uint16_t  I   CDU counter word, 15 bits, 2's complement, 2^0 pi
 *
 * RETURNS: the word's ones' complement value, 2^14 = pi
 **********************************************************************/
static int cdu_ones(uint16_t cdu)
{
    int v = 0; /* ones' complement value, 2^14 = pi */

    if (!AGC_ASSERT(cdu <= WORD15))
    {
        return (0); /* no valid result */
    }

    v = ((cdu & CDU_SIGN) != 0u) ? -(int) (WORD15 - cdu) : /* negative */
        (int) cdu;

    if (!AGC_ASSERT(v >= -(int) POSMAX && v <= (int) POSMAX))
    {
        return (0); /* no valid result */
    }

    return (v);
}

/**********************************************************************
 * AGC: none (the CCS magnitude used before AD 13,14,15 / AD NEG1/2)
 *
 * ARGUMENTS:
 *   v  int  I   ones' complement value, |v| <= 2^15
 *
 * RETURNS: |v|
 **********************************************************************/
static int iabs(int v)
{
    int a = 0; /* |v| */

    if (!AGC_ASSERT(v > -0100000 && v < 0100000))
    {
        return (0); /* no valid result */
    }

    a = (v < 0) ? -v : v; /* magnitude */

    if (!AGC_ASSERT(a >= 0))
    {
        return (0); /* no valid result */
    }

    return (a);
}

/**********************************************************************
 * ZOPFINI ("ZOPFIN1" in the listing) / ZOPFIN2 / ZOPFIN3 (p.161) --
 * where zero optics stands
 *
 * RETURNS: 1 = ENDZOPT task working (the AGC RESUMEs at once), 2 = zero
 *          optics processing (L+2), 0 = not processing (L+1)
 **********************************************************************/
static int t4_zopfini(void)
{
    if (!AGC_ASSERT(optmodes <= WORD15))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(optics_ok()))
    {
        return (0); /* no valid result */
    }

    if ((optmodes & BIT1) != 0u)
    {
        return (1);
    }

    return (((optmodes & 000004u) != 0u) ? 2 : 0); /* BIT3: processing */
}

/**********************************************************************
 * CANZOPT (p.161) -- cancel zero optics: clear the processing and
 * OCDUFAIL-inhibit bits, make sure ZERO OCDU (channel 12 bit 1) is off
 **********************************************************************/
static void t4_canzopt(void)
{
    optmodes = (uint16_t) (optmodes & ~(uint32_t) OPT_SIX);
    ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT1);

    if (!AGC_ASSERT((optmodes & OPT_SIX) == 0u))
    {
        return;
    }

    if (!AGC_ASSERT((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT1) == 0u))
    {
        return;
    }
}

/**********************************************************************
 * INITZOPT (p.164) -- zero optics processing, OCDUFAIL inhibited, no
 * return option
 **********************************************************************/
static void t4_initzopt(void)
{
    wtoption = 0;
    optmodes = (uint16_t) (optmodes | OPT_SIX);

    if (!AGC_ASSERT((optmodes & OPT_SIX) == OPT_SIX))
    {
        return;
    }

    if (!AGC_ASSERT(wtoption == 0))
    {
        return;
    }
}

/**********************************************************************
 * SETDESMD (p.163) -- remember the current switch setting, then RESUME
 *
 * RETURNS: AGC_STATUS_OK, or BAD_ARG on an out-of-range state
 **********************************************************************/
static agc_status_t t4_setdesmd(void)
{
    if (!AGC_ASSERT(optics_ok()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    desopmod = swsample;

    if (!AGC_ASSERT(desopmod >= -1 && desopmod <= 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * MANTOCSC (p.162), and MANTOCSC +3 = COARSLOK when full is false --
 * switch now at CMC: resume coarse optics if it was cancelled
 *
 * ARGUMENTS:
 *   full  bool  I   true at MANTOCSC (cancel WTOPTION and ZOPTCNT),
 *                   false at MANTOCSC +3
 *
 * RETURNS: the SETDESMD status
 **********************************************************************/
static agc_status_t t4_mantocsc(bool full)
{
    if (!AGC_ASSERT(optics_ok()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (full)
    {
        wtoption = 0;
        zoptcnt  = 0;
    }

    if ((optmodes & BIT9) != 0u)
    {
        optind = 1; /* COARSLOK L+2: set coarse optics working */
        agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT2);
    }

    if (!AGC_ASSERT(((optmodes & BIT9) == 0u) || (optind == 1)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (t4_setdesmd());
}

/**********************************************************************
 * CANCOARS (p.163) -- switch left CMC: kill coarse optics, disable the
 * OCDU error counters, set the return-to-coarse bit
 *
 * RETURNS: the SETDESMD status
 **********************************************************************/
static agc_status_t t4_cancoars(void)
{
    optind = -1;
    ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT2);
    optmodes = (uint16_t) (optmodes | BIT9); /* CS / MASK BIT9 / ADS */

    if (!AGC_ASSERT(optind == -1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((optmodes & BIT9) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (t4_setdesmd());
}

/**********************************************************************
 * SETZOEND (p.163) -- send ZERO OCDU and hold it 200 ms (task ENDZOPT)
 *
 * RETURNS: the SETDESMD status, or BAD_ARG when the waitlist refuses
 **********************************************************************/
static agc_status_t t4_setzoend(void)
{
    agc_status_t st = AGC_STATUS_OK; /* WAITLIST result */

    agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT1);
    st = agc_waitlist_call(ZOPT_HOLD_CS, AGC_TASK_ENDZOPT);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    optmodes = (uint16_t) (optmodes | BIT1); /* ZOPT task working */

    if (!AGC_ASSERT((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT1) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (t4_setdesmd());
}

/**********************************************************************
 * ZOPTDES / ZTOMAN / ZOPALARM / ZTOCSC (p.161) -- previous switch
 * setting was zero optics
 *
 * RETURNS: AGC_STATUS_OK (RESUME), or the status of the branch taken
 **********************************************************************/
static agc_status_t t4_zoptdes(void)
{
    int z = t4_zopfini(); /* ZOPFINI result */

    if (!AGC_ASSERT(desopmod < 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(z >= 0 && z <= 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (z == 1)
    {
        return (AGC_STATUS_OK); /* ENDZOPT working: TC RESUME */
    }

    if (swsample < 0)
    {
        if (z == 0)
        {
            return (t4_setdesmd()); /* ZOPT not processing */
        }

        if (zoptcnt > 0)
        {
            zoptcnt--; /* SETCNT: 32-sample wait not over */

            return (t4_setdesmd());
        }

        return (t4_setzoend());
    }

    if (swsample == 0)
    {
        if (z == 0)
        {
            return (t4_setdesmd()); /* ZTOMAN: ZOPT done */
        }

        agc_alarm(ALARM_ZOPT_SWITCH); /* ZOPALARM */
        wtoption = WTOPT_RETURN;
        t4_canzopt();

        return (t4_setdesmd());
    }

    if (z == 0)
    {
        return (t4_mantocsc(false)); /* ZTOCSC: no ZOPT */
    }

    agc_alarm(ALARM_ZOPT_SWITCH);
    t4_canzopt();

    return (t4_mantocsc(true));
} /* t4_zoptdes */

/**********************************************************************
 * MANUDES / MANTOMAN / OPTZERO (p.162) -- previous switch setting was
 * manual
 *
 * RETURNS: the status of the branch taken
 **********************************************************************/
static agc_status_t t4_manudes(void)
{
    if (!AGC_ASSERT(wtoption >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(desopmod == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (swsample > 0)
    {
        return (t4_mantocsc(true));
    }

    if (swsample == 0)
    {
        if (wtoption > 0)
        {
            wtoption--; /* MANTOMAN: count the return option down */
        }

        return (t4_setdesmd());
    }

    if (wtoption > 0)
    {
        t4_initzopt(); /* 5 s return good: continue ZOPT */

        return (t4_setdesmd());
    }

    t4_initzopt(); /* OPTZERO: start anew */
    zoptcnt = ZOPT_SAMPLES;

    return (t4_setdesmd());
}

/**********************************************************************
 * CSCDES / CSCTOZOP / CSCTOMAN (p.162) -- previous switch setting was
 * CMC
 *
 * RETURNS: the status of the branch taken
 **********************************************************************/
static agc_status_t t4_cscdes(void)
{
    if (!AGC_ASSERT(desopmod > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(optics_ok()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (swsample > 0)
    {
        return (t4_setdesmd());
    }

    if (swsample < 0)
    {
        zoptcnt = ZOPT_SAMPLES; /* CSCTOZOP, then on into CSCTOMAN */
        t4_initzopt();
    }

    if (optind >= 0)
    {
        return (t4_cancoars()); /* coarse optics working */
    }

    return (t4_setdesmd());
}

/**********************************************************************
 * OCDUFTST / OPFAILON / OPFAILOF / SETOFF (p.165) -- the OPTICS CDU
 * FAIL lamp, DSPTAB +11D bit 8, follows channel 30 bit 7 (0 = fail)
 * unless OPTMODES bit 2 inhibits it or a lamp test runs
 **********************************************************************/
static void t4_ocduftst(void)
{
    uint16_t w       = agc_erase_dsptab(LAMPWORD); /* DSPTAB +11D      */
    bool     lamp_on = (w & BIT8) != 0u;            /* lamp lit now     */
    bool     want_on = false;                       /* desired state    */

    if (!AGC_ASSERT(w <= WORD15))
    {
        return;
    }

    if ((agc_chan_read((int) AGC_CHAN_CHAN30) & BIT7) != 0u)
    {
        if ((agc_erase_imodes33() & BIT1) != 0u)
        {
            return; /* OPFAILOF: lamp test in progress */
        }

        want_on = false;
    }
    else
    {
        if ((optmodes & BIT2) != 0u)
        {
            return; /* OCDUFAIL inhibited */
        }

        want_on = true;
    }

    if (want_on != lamp_on)
    {
        /* SETOFF: RXOR the bit, MASK POSMAX, AD BIT15 (send) */
        agc_erase_set_dsptab(
            LAMPWORD, (uint16_t) (((w ^ BIT8) & POSMAX) + BIT15));

        if (!AGC_ASSERT((agc_erase_dsptab(LAMPWORD) & BIT15) != 0u))
        {
            return;
        }
    }
}

/**********************************************************************
 * OPTMON / 33OPTMON / SETSAMP / PROCESSW (p.160) -- the optics monitor,
 * every 480 ms
 *
 * RETURNS: AGC_STATUS_OK (RESUME), or the status of the branch taken
 **********************************************************************/
agc_status_t agc_t4_optics_monitor(void)
{
    uint16_t chg     = 0; /* RUPTREG1: changed inbits             */
    uint16_t present = 0; /* switch bits now set (inverted input) */

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    chg = (uint16_t) ((optmodes ^ agc_chan_read((int) AGC_CHAN_CHAN30)) &
                      BIT7);

    if (chg != 0u)
    {
        t4_ocduftst();
    }

    if (optind == AGC_OPTIND_NEG0)
    {
        return (AGC_STATUS_OK); /* 33OPTMON: bypass if TVC takeover */
    }

    chg = (uint16_t) (chg + ((optmodes ^ agc_chan_read((int) AGC_CHAN_CHAN33)) &
                             OCTHIRTY));
    optmodes = (uint16_t) (optmodes ^ chg);
    present  = (uint16_t) (~(uint32_t) optmodes & OCTHIRTY);

    if (!AGC_ASSERT(((optmodes ^ agc_chan_read((int) AGC_CHAN_CHAN33)) &
                     OCTHIRTY) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (present == 0u)
    {
        swsample = 0; /* manual */
    }
    else
    {
        swsample = ((present & BIT5) != 0u) ? 1 : -1; /* CSC / ZOPTICS */
    }

    if (desopmod > 0)
    {
        return (t4_cscdes());
    }

    if (desopmod == 0)
    {
        return (t4_manudes());
    }

    return (t4_zoptdes());
} /* agc_t4_optics_monitor */

/**********************************************************************
 * ENDZOPT / ZEROPCDU (pp.163-164) -- the zero optics end task, and its
 * continuation ENDZOPT2 after VARDELAY
 *
 * ENDZOPT zeroes CDUS and ZONE, sets CDUT to -20 deg, drops ZERO OCDU
 * and waits 200 ms for the CDUs to resynchronise; ENDZOPT2 shows the
 * optics zeroed, enables OCDUFAIL and checks the fail bit.
 *
 * ARGUMENTS:
 *   task  agc_task_id_t  I   AGC_TASK_ENDZOPT or AGC_TASK_ENDZOPT2
 *
 * RETURNS: the VARDELAY status (ENDZOPT), AGC_STATUS_OK (ENDZOPT2,
 *          TASKOVER), BAD_ARG for another task
 **********************************************************************/
agc_status_t agc_t4_optics_task(agc_task_id_t task)
{
    if (task == AGC_TASK_ENDZOPT)
    {
        agc_erase_set_cdus(0); /* ZEROPCDU */
        zone = 0;
        agc_erase_set_cdut((uint16_t) (WORD15 - DEG20_CDU)); /* CS 20DEGS */
        ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT1);

        if (!AGC_ASSERT(agc_erase_cdus() == 0u))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        return (agc_waitlist_vardelay(ZOPT_HOLD_CS, AGC_TASK_ENDZOPT2));
    }

    if (!AGC_ASSERT(task == AGC_TASK_ENDZOPT2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    optmodes = (uint16_t) ((optmodes | BIT10) & ~(uint32_t) OPT_SEVEN);
    t4_ocduftst(); /* OCDUFAIL after enable */

    if (!AGC_ASSERT((optmodes & OPT_SEVEN) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GETOPCMD (p.167) -- drive command from the desired and actual CDU
 *
 * MSU gives the difference modulo 2^15; MP BIT13 divides it by 4 and
 * the DOUBLE of the low word rounds (half away from zero).
 *
 * ARGUMENTS:
 *   desired  uint16_t  I   DESOPTT / DESOPTS, 2's complement, 2^0 pi
 *   actual   uint16_t  I   CDUT / CDUS, 2's complement, 2^0 pi
 *
 * RETURNS: the command, CDU pulses (1 pulse = 4 CDU counts)
 **********************************************************************/
static int t4_getopcmd(uint16_t desired, uint16_t actual)
{
    int diff = (int) desired - (int) actual; /* MSU, counts (mod 2^15) */
    int q    = 0;                            /* MP BIT13 high word     */
    int r    = 0;                            /* low word remainder     */

    if (!AGC_ASSERT(desired <= WORD15 && actual <= WORD15))
    {
        return (0); /* no valid result */
    }

    if (diff >= 040000)
    {
        diff -= 0100000;
    }
    else if (diff < -040000)
    {
        diff += 0100000;
    }

    q = diff / 4;
    r = diff % 4;

    if (r >= 2)
    {
        q++; /* DOUBLE overflowed: ADS L */
    }
    else if (r <= -2)
    {
        q--;
    }

    if (!AGC_ASSERT(q >= -010000 && q <= 010000))
    {
        return (0); /* no valid result */
    }

    return (q);
}

/**********************************************************************
 * OPT2 / GETOPCMD / STORCMD / TRUNCMD and the shaft stop avoidance
 * (pp.167-168) -- fill COMMANDO
 *
 * Shaft first (OPTIND 1), then trunnion: a trunnion difference that
 * overflows (TS Q skips) commands +-POSMAX.  With the shaft beyond 90
 * deg in a ZONE whose sign agrees with the command and DESOPTS within
 * 90 deg, the shaft command is reversed to keep off the stops.
 **********************************************************************/
static void t4_opt_commands(void)
{
    const agc_align_ram_t *ar    = agc_align_ram(); /* DESOPTT, DESOPTS */
    int                    tdiff = 0; /* DESOPTT - CDUT, ones', 2^14 = pi */
    int                    cdus  = 0; /* CDUS, ones', 2^14 = pi           */

    if (!AGC_ASSERT(ar != NULL))
    {
        return;
    }

    commando[1] = t4_getopcmd(ar->desopts, agc_erase_cdus()); /* shaft */
    tdiff       = cdu_ones(ar->desoptt) - cdu_ones(agc_erase_cdut());

    if ((tdiff > (int) POSMAX) || (tdiff < -(int) POSMAX))
    {
        /* TRUNCMD: TS Q overflow -> CCS A / CA POSMAX or CS POSMAX */
        commando[0] = (tdiff > 0) ? (int) POSMAX : -(int) POSMAX; /* sign */
    }
    else
    {
        commando[0] = t4_getopcmd(ar->desoptt, agc_erase_cdut());
    }

    if (!AGC_ASSERT(iabs(commando[0]) <= (int) POSMAX))
    {
        return;
    }

    cdus = cdu_ones(agc_erase_cdus());

    if ((iabs(cdus) - 1 - DEG90_CDU <= 0) || (zone == 0))
    {
        return; /* CMDSETUP: within 90 deg or ZONE zero */
    }

    if ((zone < 0) != (commando[1] < 0))
    {
        return; /* sign of ZONE differs from the command */
    }

    if (iabs(cdu_ones(ar->desopts)) - 1 - DEG90_CDU <= 0)
    {
        commando[1] = -commando[1]; /* reverse the regular command */
    }

    if (!AGC_ASSERT(iabs(commando[1]) <= 010000))
    {
        return;
    }
} /* t4_opt_commands */

/**********************************************************************
 * CMDSETUP / POSOPCMD / NEGOPCMD / DELOPCMD / NEXTOPT / SENDOCMD
 * (pp.168-169) -- limit each command to MAXPLS pulses, store the out-
 * counters (shaft, then trunnion) and start the drive if any is nonzero
 **********************************************************************/
static void t4_opt_send(void)
{
    int sends = 0; /* ITEMP1 send indicator */

    for (int i = 1; i >= 0; i--)
    {
        int c = commando[i]; /* command, CDU pulses */

        if (c > MAXPLS)
        {
            c = MAXPLS;
        }
        else if (c < -MAXPLS)
        {
            c = -MAXPLS;
        }

        if (c != 0)
        {
            sends++; /* INCR ITEMP1 (a zero command skips it) */
        }

        agc_erase_set_cdutcmd(i, (agc_sp_t) c); /* CDUTCMD / CDUSCMD */
    }

    optind = 0;

    if (sends > 0)
    {
        agc_chan_or((int) AGC_CHAN_CHAN14, (uint16_t) OCT11_12);
    }

    if (!AGC_ASSERT(sends <= 2))
    {
        return;
    }

    if (!AGC_ASSERT(iabs(agc_erase_cdutcmd(0)) <= MAXPLS &&
                    iabs(agc_erase_cdutcmd(1)) <= MAXPLS))
    {
        return;
    }
} /* t4_opt_send */

/**********************************************************************
 * OPTDRIVE / OZONE / CONTDRVE / SETBIT (pp.166-167) -- the optics CDU
 * drive, every 480 ms
 *
 * ZONE takes the sign of CDUS when the shaft first passes 45 deg and is
 * zero inside 45 deg.  Coarse optics with the switch at CMC: alarm 120
 * if the optics were never zeroed, enable the error counters (one pass)
 * and then drive.
 *
 * RETURNS: AGC_STATUS_OK (RESUME), BAD_ARG on an out-of-range state
 **********************************************************************/
agc_status_t agc_t4_optics_drive(void)
{
    int cdus = cdu_ones(agc_erase_cdus()); /* CDUS, ones', 2^14 = pi */

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (iabs(cdus) <= DEG45_CDU)
    {
        zone = 0; /* OZONE */
    }
    else if (zone == 0)
    {
        zone = cdus;
    }

    if (!AGC_ASSERT(iabs(zone) <= (int) POSMAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((optind < 0) || (swsample <= 0))
    {
        return (AGC_STATUS_OK); /* no coarse optics / switch not at CMC */
    }

    if ((optmodes & BIT10) == 0u)
    {
        agc_alarm(ALARM_OPT_NOT_ZEROED);
    }

    if ((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT2) == 0u)
    {
        agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT2); /* SETBIT */

        return (AGC_STATUS_OK);
    }

    t4_opt_commands();
    t4_opt_send();

    return (AGC_STATUS_OK);
} /* agc_t4_optics_drive */
