/**********************************************************************
 * FILE NAME: engine.c
 *
 * PURPOSE:
 *   The yaAGC engine API that NASSP (Project Apollo - NASSP 9, an Orbiter
 *   add-on) compiles into its CSM and LM vessels, implemented over the C
 *   flight software (libagc) and the src/sim clock / interrupt driver,
 *   so NASSP's C++ (ApolloGuidance, CSMcomputer, LEMcomputer, the IMU,
 *   CDU, DSKY, telemetry, MCC and RTCC code) runs unchanged on our AGC.
 *   It replaces NASSP's yaAGC sources (src_sys/yaAGC) in the vessel DLLs and is
 *   compiled against NASSP's own src_sys/yaAGC/agc_engine.h (the agc_t
 *   struct NASSP's C++ reads and writes), which is not part of this
 *   repository. Host-only: not in libagc, not in the Linux build.
 *
 * AGC SOURCE:
 *   none (host shim). Interface reference: NASSP 9 (5470bf1)
 *   src_sys/yaAGC/agc_engine.c / agc_engine_init.c and
 *   src_sys/apolloguidance.cpp; the study is
 *   docs/overview/nassp_integration.md.
 *
 * EXTERNAL VARIABLES:
 *   agc_t (NASSP's struct, one per vessel; this DLL's vessel is the one
 *   most recently passed in): Erasable[][] is a MIRROR of the typed
 *   erasable, InputChannel[] of the channels, InterruptRequests[] of the
 *   pending interrupts.
 *
 * EXTERNAL REFERENCES:
 *   Name                 Description
 *   -------------------  ------------------------------------------
 *   ChannelOutput        NASSP: an output channel or pulse train
 *   RequestRadarData     NASSP: store RNRAD at the end of a radar cycle
 *   sim_clock_count      one 1/3200 s scaler count (src/sim/clock.c)
 *   sim_input            an input channel (src/sim/channels.c)
 *   sim_counter_increment  PINC/MINC/... on a counter (src/sim/counters.c)
 *   agc_erase_read/write_ecadr  the erasable by address (src/core/ecadr.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   agc_load_binfile returns 1 for a rope other than Comanche055 /
 *   Luminary099 or an unreadable rope file (NASSP ignores the value; the
 *   computer then never starts and its DSKY stays dark).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   One AGC per DLL (libagc keeps its state in static storage): one CMC
 *   in Saturn5.dll / Saturn1B.dll, one LGC in LEM.dll. Single thread
 *   (NASSP serialises agc_engine under agcCycleMutex).
 *
 * NOTES:
 *   Timing: NASSP calls agc_engine once per memory cycle (11.71875 us);
 *   every 80/3 calls (yaAGC's SCALER_OVERFLOW / SCALER_DIVIDER) is one
 *   scaler count, and only then does the shim do work: import what
 *   NASSP wrote (input channels, the erasable mirror against its shadow
 *   copy, interrupt requests), run one clock count, export (registers
 *   and counters every count, the display words (MODREG, FAILREG,
 *   DSPTAB) every centisecond, the whole erasable every 0.1 s, all
 *   channels, the pending interrupts). NASSP's stores are imported the
 *   same way: counters every count, the whole mirror every centisecond.
 *   NASSP's channel numbering: optics / TVC drives on 0140 (CDUSCMD) and
 *   0141 (CDUTCMD), LM thrust 0142, altitude meter 0143 (agc_engine.c);
 *   src/sim sends optics on 0171 / 0172 (yaAGC), so they are renumbered.
 *   DOWNRUPT is raised by NASSP's PCM telemetry, not by the clock.
 *   DEVIATION (power): NASSP's power-off code writes Z = 04000 into the
 *   mirror; the next step after power returns is a GOJAM (agc_rupt_go),
 *   which is the hardware's power-on.
 *   DEVIATION (save / restore): a scenario's AGC_BEGIN block restores the
 *   mapped erasables (src/core/ecadr.c) and the counters; the executive,
 *   waitlist and restart tables have no address, so the computer restarts
 *   (GOJAM -> GOPROG) with the navigation state kept.
 **********************************************************************/
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "agc_engine.h"

#include "../sim/internal.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/erasable_map/address_map.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"

#define NS_SCALER_STEP   3                    /* yaAGC SCALER_DIVIDER, per MCT          */
#define NS_SCALER_COUNT  80                   /* yaAGC SCALER_OVERFLOW: one 1/3200 s    */
#define NS_COUNTS_CS     32                   /* scaler counts per centisecond          */
#define NS_EXPORT_COUNTS 320                  /* full erasable export: every 0.1 s      */
#define NS_BEAT_CS       6000u                /* trace heartbeat: every minute of AGC time */
#define NS_SLOW_CLOCKS   (CLOCKS_PER_SEC / 4) /* a scaler count slower than 0.25 s CPU */
#define NS_ERASABLE      04000                /* erasable words, E0-E7                  */
#define NS_BANK          0400                 /* words per erasable bank                */
#define NS_REG_Z         05                   /* Z register                             */
#define NS_Z_POWER       04000                /* Z after NASSP's power-off reset        */
#define NS_REG_FIRST     024                  /* first counter register (TIME2)         */
#define NS_REG_LAST      060                  /* last counter register (ALTM)           */
#define NS_FLAG_FIRST    074                  /* first flagword (STATE)                 */
#define NS_ADDR_FAILREG  0375                 /* FAILREG x3 (both ropes)                */
#define NS_ADDR_MODREG   01011                /* MODREG (both ropes)                    */
#define NS_ADDR_DSPTAB   01023                /* DSPTAB x12 (both ropes)                */
#define NS_WORD          077777u              /* one AGC word                           */
#define NS_SIGN          040000u              /* ones'-complement sign                  */
#define NS_MAG           037777u              /* ones'-complement magnitude             */
#define NS_TWOS_NEG      0100000u             /* bit 16: a two's-complement int16 < 0   */
#define NS_CHANNELS      040                  /* AGC channels 0-037                     */
#define NS_RUPTS         10                   /* yaAGC interrupt numbers 1-10           */
#define NS_CH_OPT_SHAFT  0140                 /* NASSP: CDUSCMD (TVC pitch / shaft)     */
#define NS_CH_OPT_TRUN   0141                 /* NASSP: CDUTCMD (TVC yaw / trunnion)    */
#define NS_CH_THRUST     0142                 /* NASSP: LM THRUST drive                 */
#define NS_CH_ALTM       0143                 /* NASSP: LM altitude meter drive         */
#define NS_CH14_THRUST   000010u              /* channel 14 bit 4: thrust drive         */
#define NS_CH14_ALTM     000004u              /* channel 14 bit 3: altitude meter drive */
#define NS_CH163_RESTART 000200u              /* 0163 bit 8: RESTART lamp               */
#define NS_CH163_STBY    000400u              /* 0163 bit 9: STBY lamp                  */
#define NS_RADM_LRPOS    000040u              /* RADMODES bit 6: LR position 2 wanted   */

/* Input channels NASSP writes (keys, discretes, inverted 030-033).       */
static const int k_inputs[] = {015, 016, 030, 031, 032, 033};

static agc_t   *g_agc;                    /* NASSP's struct for this DLL's vessel */
static bool     g_loaded;                 /* rope loaded, libagc powered on       */
static bool     g_started;                /* first GOJAM done                     */
static int      g_scaler;                 /* yaAGC ScalerCounter                  */
static int      g_count;                  /* scaler counts into the centisecond   */
static int      g_export;                 /* scaler counts since the full export  */
static int16_t  g_shadow[NS_ERASABLE];    /* mirror as last exported              */
static int16_t  g_chan_seen[NS_CHANNELS]; /* InputChannel as last exported        */
static FILE    *g_trace;                  /* apollo-c-agc-<vehicle>.log sidecar   */
static uint32_t g_keys;                   /* key codes taken (KEYRUPT requested)   */
static uint32_t g_errors;                 /* refused stores / inputs, GOPROG BAD_ARG */

/* ---- words ---------------------------------------------------------------- */

/**********************************************************************
 * A signed value as the 15-bit ones'-complement word NASSP reads.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: word_of_sp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   v         agc_sp_t  I    the value (scale of the register it goes to)
 *
 * RETURNS: int16_t -- the AGC word (bit 15 the sign)
 **********************************************************************/
static int16_t word_of_sp(agc_sp_t v)
{
    return ((int16_t) ((v < 0) ? (~(uint16_t) (-(int32_t) v) & NS_WORD) : (uint16_t) v)); /* negative: ones'-complement of the magnitude */
}

/**********************************************************************
 * A mirror cell as NASSP wrote it: a 15-bit ones'-complement word, or a
 * plain negative int16 (LEMcomputer stores the RHC counts as int).
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: sp_of_cell
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------------------------
 *   c         int16_t  I    the cell
 *
 * RETURNS: agc_sp_t -- the signed value (scale of the register)
 **********************************************************************/
static agc_sp_t sp_of_cell(int16_t c)
{
    uint16_t w = (uint16_t) c; /* the bits */
    agc_sp_t v = 0;            /* the value, the register's scale (unscaled count) */

    if ((w & NS_TWOS_NEG) != 0u)
    {
        v = (agc_sp_t) c;
    }
    else if ((w & NS_SIGN) != 0u)
    {
        v = (agc_sp_t) -(int32_t) ((uint16_t) ~w & NS_MAG);
    }
    else
    {
        v = (agc_sp_t) w;
    }

    return (v);
}

/**********************************************************************
 * The mirror cell of a flat erasable address (bank = addr / 0400).
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: cell
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------------------------
 *   addr      int   I    flat erasable address, 0-03777
 *
 * RETURNS: int16_t * -- the cell in NASSP's agc_t
 **********************************************************************/
static int16_t *cell(int addr)
{
    return (&g_agc->Erasable[addr / NS_BANK][addr % NS_BANK]);
}

/* ---- counters and registers (024-060) ------------------------------------- */

/**********************************************************************
 * Counter and timer registers 024-040 as words: TIME1-6, CDUs, optics, PIPAs.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: reg_get
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------------------------
 *   addr      int   I    register address
 *
 * RETURNS: int16_t -- the register word (0 for an address reg_get2 handles)
 **********************************************************************/
static int16_t reg_get(int addr)
{
    int16_t w = 0; /* the register word */

    switch (addr)
    {
        case RegTIME2:
            w = word_of_sp(agc_erase_time2());
            break;
        case RegTIME1:
            w = word_of_sp(agc_erase_time1());
            break;
        case RegTIME3:
            w = (int16_t) agc_waitlist_time3();
            break;
        case RegTIME4:
        case RegTIME5:
        case RegTIME6:
            w = (int16_t) agc_rupt_time(addr - RegTIME4 + 4);
            break;
        case RegCDUX:
            w = (int16_t) agc_erase_cdux();
            break;
        case RegCDUY:
            w = (int16_t) agc_erase_cduy();
            break;
        case RegCDUZ:
            w = (int16_t) agc_erase_cduz();
            break;
        case RegOPTY:
            w = (int16_t) agc_erase_cdut();
            break;
        case RegOPTX:
            w = (int16_t) agc_erase_cdus();
            break;
        case RegPIPAX:
            w = word_of_sp(agc_erase_pipax());
            break;
        case RegPIPAY:
            w = word_of_sp(agc_erase_pipay());
            break;
        case RegPIPAZ:
            w = word_of_sp(agc_erase_pipaz());
            break;
        default:
            w = 0; /* the rest: reg_get2 */
            break;
    } /* switch */

    return (w);
} /* reg_get */

/**********************************************************************
 * Registers 041-060 as words (RHC, INLINK, RNRAD, GYROCTR, CDU and optics
 * drives, THRUST, ALTM); the others through reg_get.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: reg_get2
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------------------------
 *   addr      int   I    register address, 024-060
 *
 * RETURNS: int16_t -- the register word
 **********************************************************************/
static int16_t reg_get2(int addr)
{
    int16_t w = 0; /* the register word */

    switch (addr)
    {
        case RegRHCP:
            w = word_of_sp(agc_erase_q_rhcctr());
            break;
        case RegRHCY:
            w = word_of_sp(agc_erase_p_rhcctr());
            break;
        case RegRHCR:
            w = word_of_sp(agc_erase_r_rhcctr());
            break;
        case RegINLINK:
            w = (int16_t) agc_erase_inlink();
            break;
        case RegRNRAD:
            w = (int16_t) agc_erase_rnrad();
            break;
        case RegGYROCTR:
            w = (int16_t) agc_imu_gyrocmd();
            break;
        case RegCDUXCMD:
        case RegCDUYCMD:
        case RegCDUZCMD:
            w = word_of_sp(agc_imu_cducmd(addr - RegCDUXCMD));
            break;
        case RegOPTYCMD:
        case RegOPTXCMD:
            w = word_of_sp(agc_erase_cdutcmd(addr - RegOPTYCMD));
            break;
        case 055:
            w = word_of_sp(agc_erase_thrust());
            break;
        case RegALTM:
            w = (int16_t) agc_erase_altm();
            break;
        default:
            w = reg_get(addr);
            break;
    }

    return (w);
} /* reg_get2 */

/**********************************************************************
 * NASSP wrote register addr: store it in the typed counter (TIME1/2 as one
 * clock, CDUs, optics, RHC, INLINK, RNRAD, PIPAs; timers 3-6 and drives
 * are the AGC's own). NASSP's PIPA pulses come through
 * UnprogrammedIncrement, which keeps the cell and its shadow, so a PIPA
 * cell changes here only at a scenario load.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: reg_set
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------------------------
 *   addr      int      I    register address, 024-060
 *   c         int16_t  I    the cell NASSP wrote
 **********************************************************************/
static void reg_set(int addr, int16_t c)
{
    uint16_t w = (uint16_t) ((uint16_t) c & NS_WORD); /* 15-bit word */

    switch (addr)
    {
        case RegTIME2:
        case RegTIME1:
        {
            uint32_t hi = (uint32_t) (uint16_t) *cell(RegTIME2) & NS_MAG; /* TIME2 */
            uint32_t lo = (uint32_t) (uint16_t) *cell(RegTIME1) & NS_MAG; /* TIME1 */

            agc_time_set_cs((hi << 14) | lo);
            break;
        }
        case RegCDUX:
            agc_erase_set_cdux(w);
            break;
        case RegCDUY:
            agc_erase_set_cduy(w);
            break;
        case RegCDUZ:
            agc_erase_set_cduz(w);
            break;
        case RegOPTY:
            agc_erase_set_cdut(w);
            break;
        case RegOPTX:
            agc_erase_set_cdus(w);
            break;
        case RegRHCP:
            agc_erase_set_q_rhcctr(sp_of_cell(c));
            break;
        case RegRHCY:
            agc_erase_set_p_rhcctr(sp_of_cell(c));
            break;
        case RegRHCR:
            agc_erase_set_r_rhcctr(sp_of_cell(c));
            break;
        case RegINLINK:
            agc_erase_set_inlink(w);
            break;
        case RegRNRAD:
            agc_erase_set_rnrad(w);
            break;
        case RegPIPAX:
            agc_erase_set_pipax(sp_of_cell(c)); /* a load: the counts since the last READACCS */
            break;
        case RegPIPAY:
            agc_erase_set_pipay(sp_of_cell(c));
            break;
        case RegPIPAZ:
            agc_erase_set_pipaz(sp_of_cell(c));
            break;
        default:
            break;                              /* timers 3-6, drives: the AGC's own */
    } /* switch */
} /* reg_set */

/* ---- the mirror ------------------------------------------------------------ */

/**********************************************************************
 * Registers 024-060 into the mirror and its shadow; Z reads 0.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: export_registers
 *
 * ARGUMENTS: none
 **********************************************************************/
static void export_registers(void)
{
    for (int a = NS_REG_FIRST; a <= NS_REG_LAST; a++)
    {
        *cell(a)    = reg_get2(a);
        g_shadow[a] = *cell(a);
    }

    *cell(NS_REG_Z)    = 0;
    g_shadow[NS_REG_Z] = 0;
}

/**********************************************************************
 * The whole erasable (flagwords up) into the mirror through the address
 * map, then the registers.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: export_erasable
 *
 * ARGUMENTS: none
 **********************************************************************/
static void export_erasable(void)
{
    for (int a = NS_FLAG_FIRST; a < NS_ERASABLE; a++)
    {
        uint16_t w = 0; /* the word */

        agc_erase_read_ecadr((uint16_t) a, &w);
        *cell(a)    = (int16_t) w;
        g_shadow[a] = (int16_t) w;
    }

    export_registers();
}

/**********************************************************************
 * The words a display reader polls (NASSP's Lua get_agcerasable, MFDs:
 * MODREG, FAILREG, DSPTAB) into the mirror and its shadow. NASSP's yaAGC
 * works in that memory itself, so a reader sees a display at once; the
 * whole-mirror export every 0.1 s alone left it up to 0.1 s late.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: export_display
 *
 * ARGUMENTS: none
 **********************************************************************/
static void export_display(void)
{
    static const int first[3] = {NS_ADDR_FAILREG, NS_ADDR_MODREG, NS_ADDR_DSPTAB}; /* first addresses */
    static const int words[3] = {3, 1, 12};                                        /* their lengths   */

    for (int b = 0; b < 3; b++)
    {
        for (int a = first[b]; a < first[b] + words[b]; a++)
        {
            uint16_t w = 0; /* the word */

            agc_erase_read_ecadr((uint16_t) a, &w);
            *cell(a)    = (int16_t) w;
            g_shadow[a] = (int16_t) w;
        }
    }
}

/**********************************************************************
 * What NASSP wrote into the mirror since the last export: the registers
 * and counters (024-060) at every count, the rest when all.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: import_erasable
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------------------------
 *   all       bool  I    scan the whole erasable, not only the registers
 *
 * RETURNS: bool -- true when NASSP wrote Z = 04000 (power came back)
 **********************************************************************/
static bool import_erasable(bool all)
{
    bool power_cycle = false;                               /* Z = 04000 written: power came back */
    int  last        = all ? NS_ERASABLE : NS_REG_LAST + 1; /* addresses scanned */

    for (int a = 0; a < last; a++)
    {
        int16_t c = *cell(a); /* the cell now */

        if (c == g_shadow[a])
        {
            continue;
        }

        if (a == NS_REG_Z)
        {
            power_cycle = (c == NS_Z_POWER);
        }
        else if (a >= NS_REG_FIRST && a <= NS_REG_LAST)
        {
            reg_set(a, c);
        }
        else if (a >= NS_FLAG_FIRST)
        {
            agc_status_t st = agc_erase_write_ecadr((uint16_t) a, (uint16_t) ((uint16_t) c & NS_WORD)); /* store */

            g_errors += (st == AGC_STATUS_OK) ? 0u : 1u; /* refused (e.g. UPSVFLAG out of range): the old value stays */
        }
        else
        {
            /* central and special registers: no C equivalent */
        }

        g_shadow[a] = c;
    }

    return (power_cycle);
}

static void trace(const char *what, double cpu_s); /* the sidecar line (below) */

/* ---- channels and interrupts ----------------------------------------------- */

/**********************************************************************
 * Input channels NASSP changed (keys, discretes) into src/sim.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: import_channels
 *
 * ARGUMENTS: none
 **********************************************************************/
static void import_channels(void)
{
    int n = (int) (sizeof (k_inputs) / sizeof (k_inputs[0])); /* input channels */

    for (int i = 0; i < n; i++)
    {
        int c = k_inputs[i]; /* the channel */

        if (g_agc->InputChannel[c] != g_chan_seen[c])
        {
            int16_t  was = g_chan_seen[c];                                           /* the value last exported */
            uint16_t v   = (uint16_t) ((uint16_t) g_agc->InputChannel[c] & NS_WORD); /* the new value */

            if ((c == 015 || c == 016) && !g_started)
            {
                /* A scenario load leaves the last key code in channel 015 (016); NASSP's
                 * yaAGC takes a key only from its interrupt request, so the load is no
                 * keystroke (a KEYRUPT here lit OPR ERR at every load). */
                g_errors += (agc_chan_set_input(c, v) == AGC_STATUS_OK) ? 0u : 1u; /* refused: a shim error */
            }
            else if (sim_input((uint16_t) c, v, NS_WORD))
            {
                g_keys++; /* 015 / 016: KEYRUPT1 / 2 requested */
            }
            else
            {
                /* not a key channel, or no interrupt asked */
            }

            if (c == 033)
            {
                char what[96]; /* the trace line */

                (void) snprintf(what, sizeof (what), "chan 33 input %05o -> %05o, stored %05o", (unsigned int) (uint16_t) was & NS_WORD, (unsigned int) (uint16_t) g_agc->InputChannel[c] & NS_WORD, (unsigned int) agc_chan_read(033)); /* stdlib: host trace */
                trace(what, 0.0);
            }
        }
    }
}

/**********************************************************************
 * All channels 0-037 into InputChannel[] (NASSP reads outputs there too)
 * and the superbank into OutputChannel7.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: export_channels
 *
 * ARGUMENTS: none
 **********************************************************************/
static void export_channels(void)
{
    for (int c = 0; c < NS_CHANNELS; c++)
    {
        g_agc->InputChannel[c] = (int16_t) agc_chan_read(c);
        g_chan_seen[c]         = g_agc->InputChannel[c];
    }

    g_agc->OutputChannel7 = g_agc->InputChannel[AGC_CHAN_SUPERBNK];
}

/**********************************************************************
 * Interrupts NASSP requested (KEYRUPT, UPRUPT, DOWNRUPT, ...) into src/sim.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: import_rupts
 *
 * ARGUMENTS: none
 **********************************************************************/
static void import_rupts(void)
{
    for (int i = 1; i <= NS_RUPTS; i++)
    {
        if (g_agc->InterruptRequests[i] != 0 && !sim_irq_pending((sim_irq_t) (i - 1)))
        {
            sim_irq_request((sim_irq_t) (i - 1));
        }
    }
}

/**********************************************************************
 * The pending interrupts back into InterruptRequests[]; no ISR in progress.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: export_rupts
 *
 * ARGUMENTS: none
 **********************************************************************/
static void export_rupts(void)
{
    for (int i = 1; i <= NS_RUPTS; i++)
    {
        g_agc->InterruptRequests[i] = sim_irq_pending((sim_irq_t) (i - 1)) ? 1 : 0; /* pending in src/sim: requested */
    }

    g_agc->InterruptRequests[0] = 0;
    g_agc->InIsr                = 0;
}

/**********************************************************************
 * LM drives NASSP adds to yaAGC: THRUST (055) on 0142, ALTM (060) on 0143,
 * sent when channel 14 enables them, then cleared.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: lm_drives
 *
 * ARGUMENTS: none
 **********************************************************************/
static void lm_drives(void)
{
    uint16_t c14 = agc_chan_read((int) AGC_CHAN_CHAN14); /* channel 14 */

    if ((c14 & NS_CH14_THRUST) != 0u && agc_erase_thrust() != 0)
    {
        ChannelOutput(g_agc, NS_CH_THRUST, (int) (uint16_t) word_of_sp(agc_erase_thrust()));
        agc_erase_set_thrust(0);
    }

    if ((c14 & NS_CH14_ALTM) != 0u && agc_erase_altm() != 0u)
    {
        ChannelOutput(g_agc, NS_CH_ALTM, (int) agc_erase_altm());
        agc_erase_set_altm(0);
    }
}

/* ---- trace (watchdog) --------------------------------------------------------- */

/**********************************************************************
 * One line in the apollo-c-agc-<vehicle>.log watchdog sidecar.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: trace
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   what      const char *  I    the event
 *   cpu_s     double        I    host CPU seconds, s
 **********************************************************************/
static void trace(const char *what, double cpu_s)
{
    sim_stats_t st; /* clock statistics */

    sim_clock_stats(&st);

    if (g_trace != NULL)
    {
        (void) fprintf(g_trace, "cs %lu MODREG %d jobs %lu restarts %lu busy %lu errors %lu shim-errors %lu keys %lu cpu %.3f s: %s\n", (unsigned long int) st.cs, (int) agc_erase_modreg(), (unsigned long int) st.jobs, (unsigned long int) st.restarts, (unsigned long int) st.busy_ticks, (unsigned long int) st.errors, (unsigned long int) g_errors, (unsigned long int) g_keys, cpu_s, what); /* stdlib: host trace */
        (void) fflush(g_trace);                                                                                                                                                                                                                                                                                                                                                                      /* stdlib: a hang must leave the line on disk */
    }
}

static agc_exec_core_info_t g_sets[8]; /* the core sets before this count, for the alarm trace */

/**********************************************************************
 * One sidecar line when FAILREG changes (an alarm): the three codes and
 * every core set's priority and job (agc_job_id_t, 0 free), so an alarm
 * seen in NASSP can be tied to the jobs that were running or stalled.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: trace_alarms
 *
 * ARGUMENTS: none
 **********************************************************************/
static void trace_alarms(void)
{
    static uint16_t last[3];         /* FAILREG as last traced */
    static agc_sp_t radcadr = 0;     /* RADCADR as last traced, a job id (B0) */
    bool            changed = false; /* FAILREG differs from the last trace */

    if (agc_erase_modecadr(2) != radcadr)
    {
        char what[96]; /* the trace line */

        radcadr = agc_erase_modecadr(2);
        (void) snprintf(what, sizeof (what), "RADCADR %d, chan 13 %05o, chan 33 %05o", (int) radcadr, (unsigned int) agc_chan_read(013), (unsigned int) agc_chan_read(033)); /* stdlib: host trace */
        trace(what, 0.0);
    }

    for (int i = 0; i < 3; i++)
    {
        changed = changed || agc_erase_failreg(i) != last[i];
        last[i] = agc_erase_failreg(i);
    }

    if (changed)
    {
        char   what[400];                                                                                                                                                                      /* the trace line */
        size_t n = (size_t) snprintf(what, sizeof (what), "alarm FAILREG %05o %05o %05o; core sets before the count", (unsigned int) last[0], (unsigned int) last[1], (unsigned int) last[2]); /* stdlib: host trace */

        for (int c = 0; c < agc_exec_core_sets() && c < 8 && n < sizeof (what); c++)
        {
            agc_exec_core_info_t j = g_sets[c];                                                                                  /* core set c before the count */
            n                      += (size_t) snprintf(what + n, sizeof (what) - n, " %d:%d/%d", c, (int) j.prio, (int) j.loc); /* stdlib: host trace */
        }

        trace(what, 0.0);
    }
}

/* ---- one scaler count ------------------------------------------------------ */

/**********************************************************************
 * GOJAM -> GOPROG: power on, or power back after NASSP's power-off.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: power_up
 *
 * ARGUMENTS: none
 **********************************************************************/
static void power_up(void)
{
    agc_status_t st = agc_rupt_go(); /* GOJAM -> GOPROG */

    g_errors  += (st == AGC_STATUS_OK) ? 0u : 1u; /* a refused GOJAM counts as a shim error */
    g_started = true;
    trace("GOJAM (power on / power cycle)", 0.0);
}

/**********************************************************************
 * One 1/3200 s scaler count: import, GOJAM when due, the clock count, LM
 * drives, export.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: one_count
 *
 * ARGUMENTS: none
 **********************************************************************/
static void one_count(void)
{
    bool cycle = import_erasable(g_count == 0); /* NASSP's stores; power cycle */

    import_channels();

    if (!g_started || cycle)
    {
        /* A scenario load (not a power-on: NASSP's power-off leaves Z = 04000) resumes a
         * running LGC through a restart, where NASSP's yaAGC simply continues. STARTSUB
         * then re-reads RADMODES bit 6, the LR position the AGC wants, from channel 33,
         * and an LR whose breaker is still open reports no position: the AGC would want
         * position 2 and alarm 00522 on every LR read of P63 once the LR comes on in
         * position 1 (flight bug 5). The saved bit is kept, as yaAGC keeps it. */
        bool     resume = !g_started && !cycle && agc_state_vehicle() == AGC_VEHICLE_LM; /* LGC scenario load */
        uint16_t lrpos  = (uint16_t) (agc_erase_radmodes() & NS_RADM_LRPOS);             /* as the image has it */

        power_up();

        if (resume)
        {
            agc_erase_set_radmodes((uint16_t) ((agc_erase_radmodes() & ~NS_RADM_LRPOS) | lrpos));
        }
    }

    import_rupts();

    for (int c = 0; c < agc_exec_core_sets() && c < 8; c++)
    {
        agc_exec_core_info(c, &g_sets[c]);
    }

    clock_t c0 = clock(); /* CPU time before the count */

    sim_clock_count();

    clock_t dc = clock() - c0; /* CPU time of the count */

    if (dc > NS_SLOW_CLOCKS)
    {
        trace("slow scaler count", (double) dc / (double) CLOCKS_PER_SEC);
    }

    if (g_count == 0 && (sim_clock_cs() % NS_BEAT_CS) == 0u)
    {
        trace("heartbeat", (double) clock() / (double) CLOCKS_PER_SEC);
    }

    trace_alarms();
    lm_drives();

    g_count = (g_count + 1) % NS_COUNTS_CS;
    g_export++;

    if (g_export >= NS_EXPORT_COUNTS)
    {
        g_export = 0;
        export_erasable();
    }
    else
    {
        export_registers();

        if (g_count == 0)
        {
            export_display(); /* each centisecond */
        }
    }

    export_channels();
    export_rupts();
    g_agc->Standby = sim_clock_standby() ? 1u : 0u; /* STBY lamp / standby state for NASSP */
} /* one_count */

/* ---- the yaAGC API ----------------------------------------------------------- */

/**********************************************************************
 * yaAGC agc_engine_init: power-on input channels, Z = 04000, RESTART lamp;
 * NASSP passes no rope here (agc_load_binfile follows).
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: agc_engine_init
 *
 * ARGUMENTS:
 *   Argument       Type          I/O  Description (scale)
 *   -------------  ------------  ---  -------------------------------------
 *   State          agc_t *       I/O  NASSP's struct for this vessel
 *   RomImage       const char *  I    unused (NULL from NASSP)
 *   CoreDump       const char *  I    unused
 *   AllOrErasable  int           I    unused
 *
 * RETURNS: int -- 0
 **********************************************************************/
int agc_engine_init(agc_t *State, const char *RomImage, const char *CoreDump, int AllOrErasable)
{
    (void) RomImage;
    (void) CoreDump;
    (void) AllOrErasable;
    g_agc = State;

    for (int c = 0; c < NUM_CHANNELS; c++)
    {
        State->InputChannel[c] = 0;
    }

    State->InputChannel[030] = 037777; /* yaAGC power-on inputs */
    State->InputChannel[031] = 077777;
    State->InputChannel[032] = 077777;
    State->InputChannel[033] = 077777;
    State->Erasable[0][RegZ] = 04000;
    State->RestartLight      = 1;
    State->DskyChannel163    = NS_CH163_RESTART;

    return (0);
}

/**********************************************************************
 * yaAGC agc_load_binfile: the rope names the vehicle; libagc, the clock
 * and the channels are powered on.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: agc_load_binfile
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   State     agc_t *       I/O  NASSP's struct for this vessel
 *   RomImage  const char *  I    rope file (Comanche055 / Luminary099)
 *
 * RETURNS: int -- 0 when loaded, 1 for another rope or an unreadable file
 **********************************************************************/
int agc_load_binfile(agc_t *State, const char *RomImage)
{
    agc_vehicle_t v  = AGC_VEHICLE_CSM; /* from the rope's name */
    int           rc = 1;               /* refused */

    g_agc = State;

    if (RomImage != NULL && sim_rope_vehicle(RomImage, &v) == AGC_STATUS_OK)
    {
        agc_power_on(v);

        if (g_trace == NULL)
        {
            g_trace   = fopen((v == AGC_VEHICLE_LM) ? "apollo-c-agc-lm.log" : "apollo-c-agc-csm.log", "w"); /* NULL: no trace */
            FILE *err = freopen("apollo-c-agc-stderr.log", "a", stderr);                                    /* src/sim's diagnostics (BAD_ARG job / interrupt) on disk, not in NASSP's lost console */

            if (err == NULL)
            {
                trace("stderr not redirected: src/sim diagnostics go to the console", 0.0);
            }
        }

        rc        = (sim_rope_load(RomImage) == AGC_STATUS_OK) ? 0 : 1; /* rope read: 0 */
        sim_clock_init();
        rc        = (sim_channels_init() == AGC_STATUS_OK) ? rc : 1;    /* channels ready: keep rc */
        (void) memset(g_shadow, 0, sizeof (g_shadow));                  /* stdlib: every non-zero mirror word is a store */
        (void) memset(g_chan_seen, 0, sizeof (g_chan_seen));            /* stdlib */
        g_loaded  = (rc == 0);
        g_started = false;
        g_scaler  = 0;
        g_count   = 0;
        g_export  = NS_EXPORT_COUNTS;                                   /* export at the first count */
    }

    return (rc);
}

/**********************************************************************
 * yaAGC agc_engine: one memory cycle (11.71875 us); every 80/3 calls one
 * scaler count (one_count).
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: agc_engine
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------------------------
 *   State     agc_t *  I/O  NASSP's struct for this vessel
 *
 * RETURNS: int -- 0
 **********************************************************************/
int agc_engine(agc_t *State)
{
    g_agc = State;
    State->CycleCounter++;

    if (g_loaded)
    {
        g_scaler += NS_SCALER_STEP;

        if (g_scaler >= NS_SCALER_COUNT)
        {
            g_scaler -= NS_SCALER_COUNT;
            one_count();
        }
    }

    return (0);
}

/**********************************************************************
 * yaAGC WriteIO: NASSP stores an input channel; taken at the next count.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: WriteIO
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------------------------
 *   State     agc_t *  I/O  NASSP's struct
 *   Address   int      I    channel, 0-0777
 *   Value     int      I    the channel word
 **********************************************************************/
void WriteIO(agc_t *State, int Address, int Value)
{
    if (Address >= 0 && Address < NUM_CHANNELS)
    {
        State->InputChannel[Address] = (int16_t) Value;
    }
}

/**********************************************************************
 * yaAGC CpuWriteIO: as WriteIO.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: CpuWriteIO
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------------------------
 *   State     agc_t *  I/O  NASSP's struct
 *   Address   int      I    channel
 *   Value     int      I    the channel word
 **********************************************************************/
void CpuWriteIO(agc_t *State, int Address, int Value)
{
    WriteIO(State, Address, Value);
}

/**********************************************************************
 * yaAGC ReadIO: a channel as last exported.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: ReadIO
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------------------------
 *   State     agc_t *  I    NASSP's struct
 *   Address   int      I    channel
 *
 * RETURNS: int -- the channel word (0 outside 0-0777)
 **********************************************************************/
int ReadIO(agc_t *State, int Address)
{
    return ((Address >= 0 && Address < NUM_CHANNELS) ? (int) State->InputChannel[Address] : 0); /* outside 0-0777: 0 */
}

/**********************************************************************
 * yaAGC UnprogrammedIncrement: a counter pulse (PIPA PINC/MINC, CDU, ...).
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: UnprogrammedIncrement
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------------------------
 *   State     agc_t *  I/O  NASSP's struct
 *   Counter   int      I    counter address (NASSP may add 0200)
 *   IncType   int      I    yaAGC increment type (PINC, MINC, PCDU, ...)
 **********************************************************************/
void UnprogrammedIncrement(agc_t *State, int Counter, int IncType)
{
    int addr = Counter & 0177; /* SetInputChannel passes 0200 | address */

    g_agc = State;

    if (g_loaded && sim_counter_increment((uint16_t) addr, (uint16_t) IncType) == AGC_STATUS_OK)
    {
        *cell(addr)    = reg_get2(addr);
        g_shadow[addr] = *cell(addr);
    }
}

/**********************************************************************
 * yaAGC MakeCoreDump: there is no yaAGC core image of the typed erasable.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: MakeCoreDump
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   State     agc_t *       I    unused
 *   CoreDump  const char *  I    unused
 **********************************************************************/
void MakeCoreDump(agc_t *State, const char *CoreDump)
{
    (void) State;
    (void) CoreDump;
}

/* ---- host hooks of src/sim (internal.h), as socket.c provides for agc_sim -- */

/**********************************************************************
 * src/sim host hook: an output channel to NASSP (ChannelOutput), optics
 * drives renumbered to NASSP's 0140/0141, the DSKY lamps into DskyChannel163.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: sim_socket_send
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   chan      uint16_t  I    channel as src/sim numbers it
 *   word      uint16_t  I    the channel word
 **********************************************************************/
void sim_socket_send(uint16_t chan, uint16_t word)
{
    int c = (int) chan; /* NASSP's channel number */

    if (chan == SIM_CH_OPT_T_DRV)
    {
        c = NS_CH_OPT_TRUN;
    }
    else if (chan == SIM_CH_OPT_S_DRV)
    {
        c = NS_CH_OPT_SHAFT;
    }
    else
    {
        /* same numbering as yaAGC */
    }

    if (chan == SIM_CH_DSKY_LAMPS)
    {
        g_agc->DskyChannel163 = word;
        g_agc->RestartLight   = ((word & NS_CH163_RESTART) != 0u) ? 1u : 0u; /* RESTART lamp bit */
    }

    ChannelOutput(g_agc, c, (int) word);
}

/**********************************************************************
 * src/sim host hook: NASSP's PCM telemetry raises DOWNRUPT.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: sim_host_downrupt_external
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true
 **********************************************************************/
bool sim_host_downrupt_external(void)
{
    return (true);
}

/**********************************************************************
 * src/sim host hook: NASSP stores RNRAD at the end of a radar cycle.
 * AGC: none (host shim; NASSP 9 5470bf1 src_sys/yaAGC/agc_engine.c)
 *
 * FUNCTION NAME: sim_host_radar_read
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_host_radar_read(void)
{
    RequestRadarData(g_agc);
    agc_erase_set_rnrad((uint16_t) ((uint16_t) *cell(RegRNRAD) & NS_WORD));
    g_shadow[RegRNRAD] = *cell(RegRNRAD);
}
