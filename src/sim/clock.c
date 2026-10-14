/**********************************************************************
 * FILE NAME: clock.c
 *
 * PURPOSE:
 *   The harness's software clock and interrupt driver: the 3200 Hz
 *   scaler, the counter-timers TIME1-TIME6 it drives, the interrupt
 *   requests they raise, the SBY/PRO standby circuit, and the executive
 *   between interrupts. One call of sim_clock_tick() is one centisecond
 *   of AGC time.
 *
 * AGC SOURCE:
 *   No AGC routine: the hardware yaAGC models in virtualagc
 *   yaAGC/agc_engine.c (agc_engine: scaler, counter-timers, HANDRUPT,
 *   RADARUPT, standby; CounterPINC, CounterDINC). Vector order of the
 *   interrupts: INTERRUPT_LEAD_INS C p.131 / L p.153.
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale     Description
 *   ------------  --------  ---  --------  --------------------------
 *   TIME1-TIME3   uint16_t  I/O  cs        agc_time_advance_cs,
 *                                          agc_waitlist_tick
 *   TIME4-TIME6   uint16_t  I/O  cs, 1/1600 s  agc_rupt_time
 *   channel 3, 4  uint16_t  O    1/3200 s  HISCALAR, LOSCALAR
 *   channel 13    uint16_t  I/O  bits      T6 enable, radar, traps,
 *                                          standby enable
 *   channel 31-32 uint16_t  I    bits      HANDRUPT trap inputs, PRO
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_rupt_t6 .. _rupt10  interrupt lead-ins (src/exec/interrupts.c)
 *   agc_waitlist_tick     PINC TIME3; T3RUPT on overflow (RUPTAGN)
 *   agc_exec_run_one      one job of the executive (EXECUTIVE C p.1220)
 *   agc_rupt_go           GOJAM -> GOPROG (leaving standby)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A handler or job returning AGC_STATUS_BAD_ARG is counted in
 *   sim_stats_t.errors and reported on stderr; the run goes on (the
 *   harness exits 1 at the end). No AGC alarm is raised here.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Single thread. The executive is bounded per call (SIM_JOBS_PER_CALL);
 *   the unbounded loop of rule 2 is in main.c.
 *
 * NOTES:
 *   Timing follows yaAGC's scaler (agc_engine.c, "Here we take care of
 *   counter-timers"): one centisecond is 32 scaler counts, starting at
 *   the TIME1 phase; at LOSCALAR & 037 = 020 TIME1/TIME2 and TIME3
 *   (PINC; TIME3 overflow -> T3RUPT); at 000 TIME5 (-> T5RUPT) and the
 *   radar gate counter while channel 13 bit 4 is set; at 010 TIME4 (->
 *   T4RUPT); at every odd count TIME6 (DINC, 1600 per second) while
 *   channel 13 bit 15 is set, the DINC at +-0 (ZOUT) requesting T6RUPT
 *   and resetting bit 15; at 036 the radar cycle completes when the gate
 *   counter is 9 (RADARUPT, bit 4 reset); the HANDRUPT traps are tested
 *   at every count. Pending interrupts are taken after each count in
 *   vector order (T6 T5 T3 T4 KEY1 KEY2 UP DOWN RADAR HAND), each through
 *   its lead-in, and stay pending while INHINT is in force; the
 *   executive runs after each count that took an interrupt and at the
 *   end of the centisecond.
 *   Standby (yaAGC agc_engine.c, SbyPressed): PRO (channel 32 bit 14 =
 *   0) seen at the scaler's 1.28 s mark (LOSCALAR & 07777 = 0) and still
 *   held 0.64 s later (04000), with standby enabled (channel 13 bit 11,
 *   set by P06), puts the computer in standby: a GOJAM that clears
 *   channels 5, 6, 010-014, 034, 035, every interrupt request and the
 *   HANDRUPT traps; the STBY lamp and EL-off bit (channels.c); only the
 *   scaler runs. PRO held the same way again leaves standby: the
 *   computer starts at 4000 (GOJAM -> GOPROG).
 *   DEVIATION (timing): a C job runs to completion or to its CHANG1 /
 *   JOBSLEEP point and an interrupt handler runs whole, so interrupts are
 *   taken between jobs and between scaler counts, never inside one.
 *   DEVIATION (DOWNRUPT): requested every 20 ms at the TIME1 phase of
 *   every other centisecond, as the downlink telemetry's end pulses do in
 *   the hardware; yaAGC instead times it 20 ms after the program has
 *   written both 034 and 035 (CpuWriteIO), which stops the downlink after
 *   a GOJAM until the program writes them again.
 *   DEVIATION (hardware alarms): the Night Watchman, Rupt Lock, TC Trap
 *   and parity alarms (yaAGC TriggeredAlarm) are not modelled, so only
 *   power-on lights RESTART, and the AGC warning filter is not modelled.
 *
 * ALGORITHM:
 *   sim_clock_tick: 32 x clock_step (scaler count, standby circuit,
 *   counters, traps, interrupts, executive if one was taken); executive;
 *   channels.c tick (lamps, drives); broadcast the changed outputs.
 **********************************************************************/
#include <stddef.h>
#include <stdio.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"
#include "internal.h"

#define SIM_JOBS_PER_CALL 64          /* executive bound per call (clock_exec) */
#define SIM_OVF           040000u     /* 15-bit counter overflow (PINC)         */
#define SIM_NEG0          077777u     /* ones'-complement -0                    */
#define SIM_TIME3_LAST    037777u     /* the next PINC overflows TIME3          */
#define SIM_DOWN_CS       2u          /* DOWNRUPT every 20 ms                   */
#define SIM_RADAR_GATES   9           /* radar cycle: 9 TIME5 phases            */
#define SIM_CH13_RADAR    000010u     /* bit 4: radar activity                  */
#define SIM_CH13_TRAPS    034000u     /* bits 12-14: HANDRUPT trap enables      */
#define SIM_CH13_T31A     004000u     /* bit 12: trap 31A (channel 31 b1-6)     */
#define SIM_CH13_T31B     010000u     /* bit 13: trap 31B (channel 31 b7-12)    */
#define SIM_CH13_T32      020000u     /* bit 14: trap 32 (channel 32 b1-10)     */
#define SIM_CH13_T6ENB    040000u     /* bit 15: TIME6 enable                   */
#define SIM_CH13_SBYENB   002000u     /* bit 11: standby enabled (P06)          */
#define SIM_CH31_ROT      000077u     /* channel 31 bits 1-6 (trap 31A)         */
#define SIM_CH31_XLT      007700u     /* channel 31 bits 7-12 (trap 31B)        */
#define SIM_CH32_TRAP     001777u     /* channel 32 bits 1-10 (trap 32)         */
#define SIM_CH32_PRO      020000u     /* channel 32 bit 14: PRO, 0 = pressed    */
#define SIM_TRAPS         3           /* traps 31A, 31B, 32                     */
#define SIM_SCALER_CS     32u         /* scaler counts (1/3200 s) per cs        */
#define SIM_PHASE_MASK    037u        /* LOSCALAR bits 1-5: the 10 ms phase     */
#define SIM_PHASE_TIME1   020u        /* TIME1, TIME2, TIME3                    */
#define SIM_PHASE_TIME5   000u        /* TIME5, radar gate (+5 ms)              */
#define SIM_PHASE_TIME4   010u        /* TIME4 (+7.5 ms)                        */
#define SIM_PHASE_RADAR   036u        /* radar cycle completion                 */
#define SIM_SCALER_LO     040000u     /* LOSCALAR wraps into HISCALAR           */
#define SIM_SCALER_MOD    0x10000000u /* 28 bits: HISCALAR, LOSCALAR            */
#define SIM_SBY_PHASE     07777u      /* 1.28 s standby period                  */
#define SIM_SBY_SAMPLE    0u          /* PRO sampled                            */
#define SIM_SBY_DECIDE    04000u      /* 0.64 s later: standby in or out        */

static uint32_t    g_cs;                 /* centiseconds since power-on            */
static uint32_t    g_count;              /* scaler counts into this centisecond    */
static bool        g_irq[SIM_IRQ_COUNT]; /* interrupt requests                     */
static int         g_radar_gate;         /* radar gate counter, TIME5 phases       */
static bool        g_trap[SIM_TRAPS];    /* HANDRUPT traps 31A 31B 32 armed        */
static sim_stats_t g_stats;              /* run statistics                         */
static uint32_t    g_scaler;             /* the next scaler count, 28 bits         */
static bool        g_standby;            /* yaAGC Standby                          */
static bool        g_sby_pressed;        /* yaAGC SbyPressed: PRO at the 1.28 s mark */
static bool        g_sby_still;          /* yaAGC SbyStillPressed                  */

/**********************************************************************
 * AGC: none -- power-on state of the clock (yaAGC agc_engine_init:
 * scaler, interrupt requests, standby flags cleared).
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_clock_init(void)
{
    g_cs          = 0;
    g_count       = 0;
    g_radar_gate  = 0;
    g_scaler      = SIM_PHASE_TIME1; /* the first count is a TIME1 phase */
    g_standby     = false;
    g_sby_pressed = false;
    g_sby_still   = false;

    for (int i = 0; i < (int) SIM_IRQ_COUNT; i++)
    {
        g_irq[i]         = false;
        g_stats.rupts[i] = 0;
    }

    for (int i = 0; i < SIM_TRAPS; i++)
    {
        g_trap[i] = false;
    }

    g_stats.cs         = 0;
    g_stats.jobs       = 0;
    g_stats.restarts   = 0;
    g_stats.busy_ticks = 0;
    g_stats.errors     = 0;
}

/**********************************************************************
 * AGC: none -- set an interrupt request (yaAGC InterruptRequests[]).
 *
 * ARGUMENTS:
 *   irq  sim_irq_t  I  the interrupt
 **********************************************************************/
void sim_irq_request(sim_irq_t irq)
{
    g_irq[irq] = true;
}

/**********************************************************************
 * AGC: none -- centiseconds run since power-on.
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint32_t -- centiseconds
 **********************************************************************/
uint32_t sim_clock_cs(void)
{
    return (g_cs);
}

/**********************************************************************
 * AGC: none -- the computer is in standby (only the scaler runs).
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true in standby
 **********************************************************************/
bool sim_clock_standby(void)
{
    return (g_standby);
}

/**********************************************************************
 * AGC: none -- a copy of the run statistics.
 *
 * ARGUMENTS:
 *   out  sim_stats_t *  O  the statistics
 **********************************************************************/
void sim_clock_stats(sim_stats_t *out)
{
    *out = g_stats;
}

/**********************************************************************
 * AGC: none -- yaAGC CpuWriteIO(013): a write of channel 13 bits 12-14
 * arms the HANDRUPT traps, and the bits cannot be read back. Called
 * before every sample of the output channels (channels.c), so neither
 * the program nor a peripheral sees them.
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_clock_arm_traps(void)
{
    uint16_t c13 = agc_chan_read((int) AGC_CHAN_CHAN13); /* channel 13 */

    if ((c13 & SIM_CH13_TRAPS) != 0u)
    {
        g_trap[0] = g_trap[0] || ((c13 & SIM_CH13_T31A) != 0u);
        g_trap[1] = g_trap[1] || ((c13 & SIM_CH13_T31B) != 0u);
        g_trap[2] = g_trap[2] || ((c13 & SIM_CH13_T32) != 0u);
        agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (~SIM_CH13_TRAPS & SIM_WORD_MASK));
    }
}

/**********************************************************************
 * AGC: none -- yaAGC CounterPINC of TIME4 (n = 4) or TIME5 (n = 5):
 * 37777 overflows to +0 and requests the interrupt.
 *
 * ARGUMENTS:
 *   n    int        I  4 or 5
 *   irq  sim_irq_t  I  the interrupt the overflow requests
 **********************************************************************/
static void time_pinc(int n, sim_irq_t irq)
{
    uint16_t v = (uint16_t) (agc_rupt_time(n) + 1u); /* TIME4 / TIME5 + 1 */

    if (v >= SIM_OVF)
    {
        v = 0;
        sim_irq_request(irq);
    }

    agc_rupt_set_time(n, v);
}

/**********************************************************************
 * AGC: none -- yaAGC CounterDINC of TIME6: a positive count steps toward
 * +0 (+1 -> -0, ones' complement), a negative one toward -0; at +-0 the
 * DINC emits ZOUT instead, which requests T6RUPT, and the hardware resets
 * channel 13 bit 15 (yaAGC: CpuWriteIO(013, ch13 & 037777)).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void time6_dinc(void)
{
    uint16_t v    = agc_rupt_time(6); /* TIME6, 1/1600 s */
    uint16_t next = v;                /* TIME6 after the DINC */

    if (v == 0u || v == SIM_NEG0)
    {
        sim_irq_request(SIM_IRQ_T6);
        agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (~SIM_CH13_T6ENB & SIM_WORD_MASK));
    }
    else if ((v & SIM_OVF) != 0u)
    {
        next = (uint16_t) (v + 1u); /* negative: toward -0 */
    }
    else
    {
        next = (v == 1u) ? (uint16_t) SIM_NEG0 : (uint16_t) (v - 1u); /* +1 - 1 = -0 in the adder */
    }

    agc_rupt_set_time(6, next);
}

/**********************************************************************
 * AGC: none -- yaAGC HANDRUPT check: an armed trap fires (and disarms)
 * when its inputs leave all-ones: channel 31 bits 1-6 (31A), 7-12 (31B),
 * channel 32 bits 1-10 (32).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void hand_traps(void)
{
    uint16_t c31  = agc_chan_read((int) AGC_CHAN_CHAN31);                  /* channel 31 */
    uint16_t c32  = agc_chan_read((int) AGC_CHAN_CHAN32);                  /* channel 32 */
    bool     hit0 = g_trap[0] && ((c31 & SIM_CH31_ROT) != SIM_CH31_ROT);   /* trap 31A fires */
    bool     hit1 = g_trap[1] && ((c31 & SIM_CH31_XLT) != SIM_CH31_XLT);   /* trap 31B fires */
    bool     hit2 = g_trap[2] && ((c32 & SIM_CH32_TRAP) != SIM_CH32_TRAP); /* trap 32 fires */

    if (hit0 || hit1 || hit2)
    {
        g_trap[0] = g_trap[0] && !hit0;
        g_trap[1] = g_trap[1] && !hit1;
        g_trap[2] = g_trap[2] && !hit2;
        sim_irq_request(SIM_IRQ_HAND);
    }
}

/**********************************************************************
 * AGC: none -- channels 4 (LOSCALAR, 14 bits) and 3 (HISCALAR, the carry
 * out of 4) from the scaler (yaAGC ChanSCALER1 / ChanSCALER2).
 *
 * ARGUMENTS:
 *   scaler  uint32_t  I  scaler count, 1/3200 s, 28 bits
 **********************************************************************/
static void scaler_set(uint32_t scaler)
{
    agc_status_t lo = agc_chan_set_input((int) AGC_CHAN_LOSCALAR, (uint16_t) (scaler % SIM_SCALER_LO));                   /* input channel: OK */
    agc_status_t hi = agc_chan_set_input((int) AGC_CHAN_HISCALAR, (uint16_t) ((scaler / SIM_SCALER_LO) % SIM_SCALER_LO)); /* input channel: OK */

    g_stats.errors += (lo == AGC_STATUS_OK && hi == AGC_STATUS_OK) ? 0u : 1u; /* a refused input is a harness error */
}

/**********************************************************************
 * AGC: none -- the GOJAM on entering standby (yaAGC TriggeredAlarm with
 * Standby set): channels 5, 6, 010-014, 034, 035 cleared, every
 * interrupt request and HANDRUPT trap dropped; RESTART is not lit.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void standby_gojam(void)
{
    static const int k_cleared[] = {005, 006, 010, 011, 012, 013, 014, 034, 035};      /* cleared channels */
    int              n           = (int) (sizeof (k_cleared) / sizeof (k_cleared[0])); /* their number */

    for (int i = 0; i < n; i++)
    {
        agc_chan_write(k_cleared[i], 0);
    }

    for (int i = 0; i < (int) SIM_IRQ_COUNT; i++)
    {
        g_irq[i] = false;
    }

    for (int i = 0; i < SIM_TRAPS; i++)
    {
        g_trap[i] = false;
    }

    sim_channels_force(AGC_CHAN_OUT0);
    sim_channels_sync();
}

/**********************************************************************
 * AGC: none -- the SBY/PRO circuit at one scaler count (yaAGC
 * agc_engine.c: SbyPressed, SbyStillPressed, the 04000 / 00000 phases of
 * channel 4).
 *
 * ARGUMENTS:
 *   scaler  uint32_t  I  scaler count, 1/3200 s
 **********************************************************************/
static void standby_check(uint32_t scaler)
{
    bool     pro   = (agc_chan_read((int) AGC_CHAN_CHAN32) & SIM_CH32_PRO) == 0u; /* PRO held */
    uint32_t phase = scaler & SIM_SBY_PHASE;                                      /* 1.28 s phase */

    if (!pro)
    {
        g_sby_pressed = false;
        g_sby_still   = false;
    }

    if (phase == SIM_SBY_SAMPLE && pro)
    {
        g_sby_pressed = true;
    }
    else if (phase == SIM_SBY_DECIDE && g_sby_pressed)
    {
        bool enabled = (agc_chan_read((int) AGC_CHAN_CHAN13) & SIM_CH13_SBYENB) != 0u; /* P06 enabled it */

        if (!g_standby && enabled)
        {
            g_standby   = true;
            g_sby_still = true;
            standby_gojam();
        }
        else if (g_standby && !g_sby_still)
        {
            g_standby      = false;
            g_stats.errors += (agc_rupt_go() == AGC_STATUS_BAD_ARG) ? 1u : 0u; /* the CPU starts at 4000 */
            sim_channels_sync();
        }
        else
        {
            /* not enabled, or PRO not yet released since standby began */
        }
    }
    else
    {
        /* not a standby phase */
    }
}

/**********************************************************************
 * AGC: none -- the TIME1 phase: TIME1/TIME2, TIME3, and DOWNRUPT every
 * other centisecond.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void time1_phase(void)
{
    agc_time_advance_cs(1);

    if (agc_waitlist_time3() >= SIM_TIME3_LAST)
    {
        sim_irq_request(SIM_IRQ_T3); /* the overflowing PINC is done by rupt_t3 */
    }
    else
    {
        g_stats.errors += (agc_waitlist_tick(1) == AGC_STATUS_OK) ? 0u : 1u; /* PINC TIME3, no overflow */
    }

    if ((g_cs % SIM_DOWN_CS) == 0u && !sim_host_downrupt_external())
    {
        sim_irq_request(SIM_IRQ_DOWNLINK);
    }
}

/**********************************************************************
 * AGC: none -- the counter-timers at one scaler count (yaAGC agc_engine,
 * "Here we take care of counter-timers").
 *
 * ARGUMENTS:
 *   scaler  uint32_t  I  scaler count, 1/3200 s
 **********************************************************************/
static void clock_counters(uint32_t scaler)
{
    uint32_t phase = scaler & SIM_PHASE_MASK;              /* 10 ms phase */
    uint16_t c13   = agc_chan_read((int) AGC_CHAN_CHAN13); /* channel 13 */

    if (phase == SIM_PHASE_TIME1)
    {
        time1_phase();
    }
    else if (phase == SIM_PHASE_TIME5)
    {
        time_pinc(5, SIM_IRQ_T5);
        g_radar_gate += ((c13 & SIM_CH13_RADAR) != 0u) ? 1 : 0; /* gate counted while bit 4 is set */
    }
    else if (phase == SIM_PHASE_TIME4)
    {
        time_pinc(4, SIM_IRQ_T4);
    }
    else
    {
        /* no counter-timer at this phase */
    }

    if ((scaler & 1u) != 0u && (c13 & SIM_CH13_T6ENB) != 0u)
    {
        time6_dinc(); /* 1/1600 s, 1/3200 s after TIME1 */
    }

    hand_traps();

    if (g_radar_gate == SIM_RADAR_GATES && phase == SIM_PHASE_RADAR)
    { /* radar cycle complete: RNRAD holds what the radar shifted in */
        g_radar_gate = 0;
        agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (~SIM_CH13_RADAR & SIM_WORD_MASK));
        sim_host_radar_read(); /* the host shifts the radar's reading into RNRAD */
        sim_irq_request(SIM_IRQ_RADAR);
    }
}

/**********************************************************************
 * T3RUPT lead-in (INTERRUPT_LEAD_INS C p.131 / L p.153) with the TIME3
 * PINC that overflowed: agc_waitlist_tick(1) sets TIME3 to +0 and runs
 * T3RUPT (WAITLIST C p.1231 / L p.1127, RUPTAGN chain), then RESUME's
 * restart service as agc_rupt_t3 does.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the T3RUPT status; BAD_ARG when a restart
 *          failed or the handler left interrupts inhibited
 **********************************************************************/
static agc_status_t rupt_t3(void)
{
    agc_inhint();
    agc_status_t st = agc_waitlist_tick(1); /* T3RUPT outcome */

    agc_relint();

    if (agc_exec_restart_pending())
    {
        g_stats.restarts++;
        st = (agc_exec_service_restart() == AGC_STATUS_BAD_ARG) ? AGC_STATUS_BAD_ARG : st; /* ENEMA failure wins */
    }

    if (agc_interrupts_inhibited())
    {
        st = AGC_STATUS_BAD_ARG; /* a task left INHINT in force */
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- take one interrupt through its lead-in
 * (src/exec/interrupts.c, INTERRUPT_LEAD_INS C p.131 / L p.153).
 *
 * ARGUMENTS:
 *   irq  sim_irq_t  I  the interrupt, < SIM_IRQ_COUNT
 *
 * RETURNS: agc_status_t -- the handler's status
 **********************************************************************/
static agc_status_t take_rupt(sim_irq_t irq)
{
    agc_status_t st = AGC_STATUS_OK; /* handler outcome */

    switch (irq)
    {
        case SIM_IRQ_T6:
            st = agc_rupt_t6();
            break;
        case SIM_IRQ_T5:
            st = agc_rupt_t5();
            break;
        case SIM_IRQ_T3:
            st = rupt_t3();
            break;
        case SIM_IRQ_T4:
            st = agc_rupt_t4();
            break;
        case SIM_IRQ_KEY1:
            st = agc_rupt_keyrupt1();
            break;
        case SIM_IRQ_KEY2:
            st = agc_rupt_keyrupt2();
            break;
        case SIM_IRQ_UPLINK:
            st = agc_rupt_uprupt();
            break;
        case SIM_IRQ_DOWNLINK:
            st = agc_rupt_downrupt();
            break;
        case SIM_IRQ_RADAR:
            st = agc_rupt_radarupt();
            break;
        case SIM_IRQ_HAND:
        case SIM_IRQ_COUNT:
        default:
            st = agc_rupt_rupt10(); /* HANDRUPT; the loop never passes COUNT */
            break;
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- the pending interrupts, highest priority first, each
 * followed by the broadcast of what it changed; held under INHINT.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when an interrupt was taken
 **********************************************************************/
static bool clock_rupts(void)
{
    bool taken = false; /* an interrupt ran */

    for (int i = 0; i < (int) SIM_IRQ_COUNT; i++)
    {
        if (!g_irq[i] || agc_interrupts_inhibited())
        {
            continue;
        }

        g_irq[i] = false;
        g_stats.rupts[i]++;
        taken    = true;

        if (take_rupt((sim_irq_t) i) == AGC_STATUS_BAD_ARG)
        {
            g_stats.errors++;
            (void) fprintf(stderr, "agc_sim: %lu cs: interrupt %d returned BAD_ARG\n", (unsigned long int) g_cs, i); /* stdlib diagnostic */
        }

        sim_channels_sync();

        if (i == (int) SIM_IRQ_T4)
        {
            sim_channels_force(AGC_CHAN_OUT0); /* every T4RUPT writes 010 */
        }
        else if (i == (int) SIM_IRQ_DOWNLINK)
        {
            sim_channels_force(AGC_CHAN_DNTM1);
            sim_channels_force(AGC_CHAN_DNTM2);
        }
        else
        {
            /* outputs already sampled */
        }
    }

    return (taken);
}

/**********************************************************************
 * AGC: none -- the executive until DUMMYJOB would run (EXECUTIVE C
 * p.1220, NEWJOB), at most SIM_JOBS_PER_CALL jobs.
 * The bound (rule 2) exists because a C job segment takes no AGC time:
 * a job that polls (CHECKG1, IMU_PERFORMANCE_TEST_2 L p.378: sample
 * PIPAX, CHANG1, sample again until a pulse comes) never lets the
 * executive reach DUMMYJOB, and without a bound the centisecond would
 * never end. 64 is above anything the AGC could finish in the time one
 * call stands for (the next call is at most one scaler count, 1/3200 s,
 * later): an empty job costs NOVAC's core-set scan, ENDOFJOB's EJSCAN
 * over the 7 core sets and CHANJOB, well over 50 MCT (EXECUTIVE C
 * pp.1208-1220), so the AGC's 853 MCT per centisecond complete fewer
 * than 17 jobs per cs. A burst cut at 64 therefore still runs earlier
 * than in the AGC; only a job that never ends reaches the bound, and
 * sim_stats_t.busy_ticks counts those calls (the AGC's processor is
 * then 100% busy too). systems_test_lm: all 1,817 hits are CHECKG1
 * polling for the first and the last PIPA pulse (cs 15746-16004 and
 * 45195-46004).
 *
 * ARGUMENTS:
 *   idle_pass  bool  I  run one pass of the idle loop (DUMMYJOB ->
 *                       ADVAN -> SELFRET) when no job is ready; once
 *                       per centisecond, so ROPECHK sums a bank per cs
 **********************************************************************/
static void clock_exec(bool idle_pass)
{
    int n = 0; /* jobs run */

    for (; n < SIM_JOBS_PER_CALL; n++)
    {
        if (agc_exec_restart_pending())
        {
            g_stats.restarts++;
            g_stats.errors += (agc_exec_service_restart() == AGC_STATUS_BAD_ARG) ? 1u : 0u; /* ENEMA failure */
        }

        agc_exec_core_info_t job;                     /* the job about to run, for the diagnostic */
        int                  set = agc_exec_newjob(); /* NEWJOB: core set, -1 idle */

        agc_exec_core_info((set > 0) ? set : 0, &job); /* set 0 when idle or running */
        agc_status_t st = agc_exec_run_one();          /* the job's outcome */

        if (st == AGC_STATUS_BUSY)
        {
            break; /* a self-rescheduled loop already ran this cs: the AGC is busy with it, no idle pass */
        }

        if (st == AGC_STATUS_NOT_FOUND)
        {
            if (idle_pass)
            {
                agc_selfcheck_idle_pass();
            }

            break; /* idle */
        }

        g_stats.jobs++;

        if (st == AGC_STATUS_BAD_ARG)
        {
            g_stats.errors++;
            (void) fprintf(stderr, "agc_sim: %lu cs: job %d returned BAD_ARG\n", (unsigned long int) g_cs, (int) job.loc); /* stdlib diagnostic */
        }

        if (!AGC_ASSERT(!agc_interrupts_inhibited()))
        { /* a job left INHINT in force: every interrupt stays held, as in the AGC */
            g_stats.errors++;
            (void) fprintf(stderr, "agc_sim: %lu cs: job %d left interrupts inhibited\n", (unsigned long int) g_cs, (int) job.loc); /* stdlib diagnostic */
        }

        sim_channels_sync();
    }

    g_stats.busy_ticks += (n == SIM_JOBS_PER_CALL) ? 1u : 0u; /* the bound was hit */
} /* clock_exec */

/**********************************************************************
 * AGC: none -- one scaler count: channels 3/4, the standby circuit, and
 * outside standby the counter-timers, traps, interrupts and, after an
 * interrupt, the executive.
 *
 * ARGUMENTS:
 *   scaler  uint32_t  I  scaler count, 1/3200 s
 **********************************************************************/
static void clock_step(uint32_t scaler)
{
    scaler_set(scaler);
    standby_check(scaler);

    if (!g_standby)
    {
        sim_clock_arm_traps();
        clock_counters(scaler);

        if (clock_rupts())
        {
            clock_exec(false);
        }
    }
}

/**********************************************************************
 * AGC: none -- whether an interrupt request is still waiting to be taken
 * (yaAGC InterruptRequests[n] != 0).
 *
 * ARGUMENTS:
 *   irq  sim_irq_t  I  the request
 *
 * RETURNS: bool -- requested and not yet taken
 **********************************************************************/
bool sim_irq_pending(sim_irq_t irq)
{
    return (g_irq[irq]);
}

/**********************************************************************
 * AGC: none -- one scaler count (1/3200 s); the 32nd count of a
 * centisecond also runs the executive's idle pass and the peripheral
 * outputs (channels.c). Hosts that step the AGC finer than a centisecond
 * (src/nassp) call this; sim_clock_tick is 32 of them.
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_clock_count(void)
{
    clock_step(g_scaler);
    g_scaler = (g_scaler + 1u) % SIM_SCALER_MOD;
    g_count++;

    if (g_count == SIM_SCALER_CS)
    {
        g_count = 0;

        if (!g_standby)
        {
            clock_exec(true);
        }

        g_stats.errors += (sim_channels_tick(g_cs) == AGC_STATUS_OK) ? 0u : 1u; /* a CDU drive count the IMU refused */
        sim_channels_sync();
        g_cs++;
        g_stats.cs     = g_cs;
    }
}

/**********************************************************************
 * AGC: none -- one centisecond: 32 scaler counts, the executive, the
 * peripheral outputs (channels.c).
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_clock_tick(void)
{
    for (uint32_t k = 0; k < SIM_SCALER_CS; k++)
    {
        sim_clock_count();
    }
}
