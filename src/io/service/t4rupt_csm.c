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
 *   DESOPTT/S     uint16_t  I/O  2's comp pi  desired optics (written by
 *                                             R52, V41 N91, P03)
 *   CDUT/S        uint16_t  I/O  2's comp pi  optics CDUs
 *   CDUTCMD/SCMD  agc_sp_t  O    pulses       drive out-counters
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
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

#include "agc/core/channels.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"

#define BIT1         000001u /* BIT1: ZERO OCDU / ZOPT task working     */
#define BIT2         000002u /* BIT2: OCDU error counters / fail inhib  */
#define BIT5         000020u /* BIT5: optics switch at CMC (inverted)   */
#define BIT7         000100u /* BIT7: OCDU fail (channel 30, inverted)  */
#define BIT8         000200u /* BIT8: OPTICS CDU FAIL lamp (DSPTAB+11D) */
#define BIT9         000400u /* BIT9: coarse optics since fresh start   */
#define BIT10        001000u /* BIT10: optics zeroed since fresh start  */
#define BIT15        040000u /* BIT15: sign / lamp word "send" bit      */
#define POSMAX       037777u /* POSMAX                                  */
#define WORD15       077777u /* one AGC word, 15 bits                   */
#define OCTHIRTY     000030u /* BITS4&5: optics mode switch bits        */
#define OPT_SIX      000006u /* SIX: ZOPT processing + OCDUFAIL inhibit */
#define OPT_SEVEN    000007u /* SEVEN: the three zero optics bits       */
#define OCT11_12     006000u /* 11,12 = PRIO6: drive CDU S and T        */
#define OPTINITF     000130u /* OPTINITF: fresh start OPTMODES          */
#define OPTINITR     000430u /* OPTINITR: OPTMODES bits kept on restart */
#define ZOPT_SAMPLES 32      /* OCT40: 32-sample wait                   */
#define WTOPT_RETURN 11      /* OCT13: 5 s return option (480 ms each)  */
#define ZOPT_HOLD_CS 20u     /* 200MS = OCT24, cs                       */
#define DEG20_CDU    7199    /* 20DEGS DEC 7199, 2^14 = pi              */
#define MAXPLS       165     /* -MAXPLS (DEC -165, was -80), pulses     */
#define DEG45_CDU    4096    /* -(13,14,15) + 1: 45 deg, 2^14 = pi      */
#define DEG90_CDU    8192    /* -NEG1/2: 90 deg, 2^14 = pi              */
#define CDU_SIGN     040000u /* sign bit of a CDU counter               */
#define LAMPWORD     11      /* DSPTAB +11D                             */

#define ALARM_ZOPT_SWITCH    ((agc_alarm_code_t) 00116) /* moved in ZOPT */
#define ALARM_OPT_NOT_ZEROED ((agc_alarm_code_t) 00120) /* not zeroed    */

static uint16_t optmodes = OPTINITF; /* OPTMODES bits                     */
static agc_sp_t optind   = -1;       /* OPTIND, unscaled (NEG0 = -0)      */
static int      swsample;            /* SWSAMPLE: +1 CMC, 0 man, -1 zero  */
static uint16_t desoptt;             /* DESOPTT: desired trunnion, CDU word */
static uint16_t desopts;             /* DESOPTS: desired shaft, CDU word  */
static int      desopmod;            /* DESOPMOD: previous SWSAMPLE       */
static int      wtoption;            /* WTOPTION countdown, 480 ms passes */
static int      zoptcnt;             /* ZOPTCNT, 480 ms samples           */
static int      zone;                /* ZONE, CDUS ones', 2^14 = pi       */
static int      commando[2];         /* COMMANDO T, S, CDU pulses         */

/**********************************************************************
 * OPTMODES (ERASABLE_ASSIGNMENTS.agc, CM) -- the optics mode bit word, read
 *
 * RETURNS: OPTMODES, 15 bits (0 in an LM build)
 **********************************************************************/
uint16_t agc_erase_optmodes(void)
{
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
    optmodes = v; /* every caller ORs / masks a 15-bit OPTMODES */
}

/**********************************************************************
 * OPTIND (ERASABLE_ASSIGNMENTS.agc, CM) -- coarse optics state, read
 *
 * RETURNS: OPTIND: -1, 0, 1, or AGC_OPTIND_NEG0 for -0
 **********************************************************************/
agc_sp_t agc_erase_optind(void)
{
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
    optind = v; /* every caller stores a constant state */
}

/**********************************************************************
 * DESOPTT (ERASABLE_ASSIGNMENTS.agc, CM) -- the trunnion angle OPTDRIVE
 * drives to, read
 *
 * RETURNS: DESOPTT, CDU word (2^15 = 1 rev)
 **********************************************************************/
uint16_t agc_erase_desoptt(void)
{
    return (desoptt);
}

/**********************************************************************
 * DESOPTT (ERASABLE_ASSIGNMENTS.agc, CM) -- the trunnion angle OPTDRIVE
 * drives to, written (R52, V41 N91, P03 RETARG)
 *
 * ARGUMENTS:
 *   v  uint16_t  I   desired trunnion, CDU word
 **********************************************************************/
void agc_erase_set_desoptt(uint16_t v)
{
    desoptt = v; /* a CDU word (PAC, CDUT) */
}

/**********************************************************************
 * DESOPTS (ERASABLE_ASSIGNMENTS.agc, CM) -- the shaft angle OPTDRIVE
 * drives to, read
 *
 * RETURNS: DESOPTS, CDU word (2^15 = 1 rev)
 **********************************************************************/
uint16_t agc_erase_desopts(void)
{
    return (desopts);
}

/**********************************************************************
 * DESOPTS (ERASABLE_ASSIGNMENTS.agc, CM) -- the shaft angle OPTDRIVE
 * drives to, written (R52, V41 N91, P03 RETARG)
 *
 * ARGUMENTS:
 *   v  uint16_t  I   desired shaft, CDU word
 **********************************************************************/
void agc_erase_set_desopts(uint16_t v)
{
    desopts = v; /* a CDU word (SAC, CDUS) */
}

/**********************************************************************
 * SWSAMPLE (ERASABLE_ASSIGNMENTS, CM; set in OPTMON p.160) -- the optics
 * mode switch as last sampled (read by V41 N91 and R52)
 *
 * RETURNS: +1 computer (CSC), 0 manual, -1 zero optics
 **********************************************************************/
agc_sp_t agc_erase_swsample(void)
{
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
 **********************************************************************/
void agc_t4_optics_reset(bool fresh)
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
    agc_chan_and(chan, (uint16_t) (~(uint32_t) bits & WORD15));
}

/**********************************************************************
 * OZONE / CMDSETUP / TRUNCMD (T4RUPT_PROGRAM.agc pp.166-168) -- CA of a
 * CDU counter: the two's complement word read as a ones' complement SP
 *
 * ARGUMENTS:
 *   cdu  uint16_t  I   CDU counter word, 15 bits, 2's complement, 2^0 pi
 *
 * RETURNS: the word's ones' complement value, 2^14 = pi
 **********************************************************************/
static int cdu_ones(uint16_t cdu)
{
    return (((cdu & CDU_SIGN) != 0u) ? -(int) (WORD15 - cdu) : (int) cdu); /* negative: -(77777 - cdu) */
}

/**********************************************************************
 * OZONE / CMDSETUP (T4RUPT_PROGRAM.agc pp.166-168) -- the magnitude CCS
 * forms before AD 13,14,15 / AD NEG1/2
 *
 * ARGUMENTS:
 *   v  int  I   ones' complement value, |v| <= 2^15
 *
 * RETURNS: |v|
 **********************************************************************/
static int iabs(int v)
{
    return ((v < 0) ? -v : v); /* CCS: |v| */
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
}

/**********************************************************************
 * INITZOPT (p.164) -- zero optics processing, OCDUFAIL inhibited, no
 * return option
 **********************************************************************/
static void t4_initzopt(void)
{
    wtoption = 0;
    optmodes = (uint16_t) (optmodes | OPT_SIX);
}

/**********************************************************************
 * SETDESMD (p.163) -- remember the current switch setting, then RESUME
 **********************************************************************/
static void t4_setdesmd(void)
{
    desopmod = swsample; /* -1, 0 or +1 (OPTMON's sample) */
}

/**********************************************************************
 * MANTOCSC (p.162), and MANTOCSC +3 = COARSLOK when full is false --
 * switch now at CMC: resume coarse optics if it was cancelled
 *
 * ARGUMENTS:
 *   full  bool  I   true at MANTOCSC (cancel WTOPTION and ZOPTCNT),
 *                   false at MANTOCSC +3
 **********************************************************************/
static void t4_mantocsc(bool full)
{
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

    t4_setdesmd();
}

/**********************************************************************
 * CANCOARS (p.163) -- switch left CMC: kill coarse optics, disable the
 * OCDU error counters, set the return-to-coarse bit
 **********************************************************************/
static void t4_cancoars(void)
{
    optind   = -1;
    ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT2);
    optmodes = (uint16_t) (optmodes | BIT9); /* CS / MASK BIT9 / ADS */
    t4_setdesmd();
}

/**********************************************************************
 * ZOPTDES / ZTOMAN / ZOPALARM / ZTOCSC (p.161) -- previous switch
 * setting was zero optics
 *
 * RETURNS: AGC_STATUS_OK (RESUME), or the status of the branch taken
 **********************************************************************/
static agc_status_t t4_zoptdes(void)
{
    int          z  = t4_zopfini();  /* ZOPFINI: 0 idle, 1 ENDZOPT working, 2 processing */
    agc_status_t st = AGC_STATUS_OK; /* SETZOEND WAITLIST */

    if (z == 1)
    {
        return (AGC_STATUS_OK); /* ENDZOPT working: TC RESUME */
    }

    if (swsample < 0)
    {
        if (z == 2 && zoptcnt > 0)
        {
            zoptcnt--; /* SETCNT: 32-sample wait not over */
        }
        else if (z == 2)
        {
            /* SETZOEND (p.163): send ZERO OCDU and hold it 200 ms (task
             * ENDZOPT); a full waitlist is the AGC's BAILOUT 1203.            */
            agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT1);
            st = agc_waitlist_call(ZOPT_HOLD_CS, AGC_TASK_ENDZOPT);

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }

            optmodes = (uint16_t) (optmodes | BIT1); /* ZOPT task working */
        }

        t4_setdesmd(); /* z == 0: ZOPT not processing */
    }
    else if (swsample == 0)
    {
        if (z != 0)
        {
            agc_alarm(ALARM_ZOPT_SWITCH); /* ZOPALARM */
            wtoption = WTOPT_RETURN;
            t4_canzopt();
        }

        t4_setdesmd(); /* ZTOMAN */
    }
    else if (z == 0)
    {
        t4_mantocsc(false); /* ZTOCSC: no ZOPT */
    }
    else
    {
        agc_alarm(ALARM_ZOPT_SWITCH);
        t4_canzopt();
        t4_mantocsc(true);
    }

    return (st);
} /* t4_zoptdes */

/**********************************************************************
 * OCDUFTST / OPFAILON / OPFAILOF / SETOFF (p.165) -- the OPTICS CDU
 * FAIL lamp, DSPTAB +11D bit 8, follows channel 30 bit 7 (0 = fail)
 * unless OPTMODES bit 2 inhibits it or a lamp test runs
 **********************************************************************/
static void t4_ocduftst(void)
{
    uint16_t w       = agc_erase_dsptab(LAMPWORD); /* DSPTAB +11D      */
    bool     lamp_on = (w & BIT8) != 0u;           /* lamp lit now     */
    bool     want_on = false;                      /* desired state    */

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
        agc_erase_set_dsptab(LAMPWORD, (uint16_t) (((w ^ BIT8) & POSMAX) + BIT15));
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

    chg = (uint16_t) ((optmodes ^ agc_chan_read((int) AGC_CHAN_CHAN30)) & BIT7);

    if (chg != 0u)
    {
        t4_ocduftst();
    }

    if (optind == AGC_OPTIND_NEG0)
    {
        return (AGC_STATUS_OK); /* 33OPTMON: bypass if TVC takeover */
    }

    chg      = (uint16_t) (chg + ((optmodes ^ agc_chan_read((int) AGC_CHAN_CHAN33)) & OCTHIRTY));
    optmodes = (uint16_t) (optmodes ^ chg);
    present  = (uint16_t) (~(uint32_t) optmodes & OCTHIRTY);

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
        /* CSCDES / CSCTOZOP / CSCTOMAN (p.162): the previous setting was CMC. */
        if (swsample < 0)
        {
            zoptcnt = ZOPT_SAMPLES; /* CSCTOZOP, then on into CSCTOMAN */
            t4_initzopt();
        }

        if (swsample <= 0 && optind >= 0)
        {
            t4_cancoars(); /* coarse optics working */
        }
        else
        {
            t4_setdesmd();
        }

        return (AGC_STATUS_OK);
    }

    if (desopmod < 0)
    {
        return (t4_zoptdes());
    }

    /* MANUDES / MANTOMAN / OPTZERO (p.162): the previous setting was manual. */
    if (swsample > 0)
    {
        t4_mantocsc(true);
    }
    else if (swsample == 0)
    {
        if (wtoption > 0)
        {
            wtoption--; /* MANTOMAN: count the return option down */
        }

        t4_setdesmd();
    }
    else
    {
        if (wtoption <= 0)
        {
            zoptcnt = ZOPT_SAMPLES; /* OPTZERO: start anew (INITZOPT clears WTOPTION) */
        }

        t4_initzopt(); /* with WTOPTION: 5 s return good, continue ZOPT */
        t4_setdesmd();
    }

    return (AGC_STATUS_OK);
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
 *          TASKOVER)
 **********************************************************************/
agc_status_t agc_t4_optics_task(agc_task_id_t task)
{
    if (task == AGC_TASK_ENDZOPT)
    {
        agc_erase_set_cdus(0);                               /* ZEROPCDU */
        zone = 0;
        agc_erase_set_cdut((uint16_t) (WORD15 - DEG20_CDU)); /* CS 20DEGS */
        ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT1);

        return (agc_waitlist_vardelay(ZOPT_HOLD_CS, AGC_TASK_ENDZOPT2));
    }

    /* ENDZOPT2 (the dispatcher passes only these two ids).                */
    optmodes = (uint16_t) ((optmodes | BIT10) & ~(uint32_t) OPT_SEVEN);
    t4_ocduftst(); /* OCDUFAIL after enable */

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

    return (q);
}

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
 * (RESUME.)
 **********************************************************************/
void agc_t4_optics_drive(void)
{
    int cdus = cdu_ones(agc_erase_cdus()); /* CDUS, ones', 2^14 = pi */

    if (iabs(cdus) <= DEG45_CDU)
    {
        zone = 0; /* OZONE */
    }
    else if (zone == 0)
    {
        zone = cdus;
    }

    if (!((optind < 0) || (swsample <= 0))) /* no coarse optics / switch not at CMC */
    {
        if ((optmodes & BIT10) == 0u)
        {
            agc_alarm(ALARM_OPT_NOT_ZEROED);
        }

        if ((agc_chan_read((int) AGC_CHAN_CHAN12) & BIT2) == 0u)
        {
            agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT2); /* SETBIT */
        }
        else
        {
            /* OPT2 / GETOPCMD / STORCMD / TRUNCMD and the shaft stop avoidance
             * (pp.167-168): shaft first (OPTIND 1), then trunnion; a trunnion
             * difference that overflows (TS Q skips) commands +-POSMAX.  With the
             * shaft beyond 90 deg in a ZONE whose sign agrees with the command
             * and DESOPTS within 90 deg, the shaft command is reversed to keep
             * off the stops.                                                     */
            int tdiff         = 0; /* DESOPTT - CDUT, ones', 2^14 = pi */
            int cdus_commands = 0; /* CDUS, ones', 2^14 = pi           */

            commando[1] = t4_getopcmd(agc_erase_desopts(), agc_erase_cdus()); /* shaft */
            tdiff       = cdu_ones(agc_erase_desoptt()) - cdu_ones(agc_erase_cdut());

            if ((tdiff > (int) POSMAX) || (tdiff < -(int) POSMAX))
            {
                /* TRUNCMD: TS Q overflow -> CCS A / CA POSMAX or CS POSMAX */
                commando[0] = (tdiff > 0) ? (int) POSMAX : -(int) POSMAX; /* sign */
            }
            else
            {
                commando[0] = t4_getopcmd(agc_erase_desoptt(), agc_erase_cdut());
            }

            cdus_commands = cdu_ones(agc_erase_cdus());

            if (!((iabs(cdus_commands) - 1 - DEG90_CDU <= 0) || (zone == 0))) /* CMDSETUP: within 90 deg or ZONE zero */
            {
                if ((zone < 0) == (commando[1] < 0)) /* ZONE sign agrees with the command */
                {
                    if (iabs(cdu_ones(agc_erase_desopts())) - 1 - DEG90_CDU <= 0)
                    {
                        commando[1] = -commando[1]; /* reverse the regular command */
                    }
                }
            }

            t4_opt_send();
        }
    } /* if (!((optind < 0) || (swsample <= 0))) */
} /* agc_t4_optics_drive */
