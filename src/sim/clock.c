/*
 * clock.c -- the software clock and the interrupt driver of the harness.
 *
 * One call of sim_clock_tick() is one centisecond of AGC time. The counter
 * timing follows yaAGC (agc_engine.c, the 1/1600 s scaler) and the AGC
 * hardware it models:
 *   +0 ms    TIME1/TIME2 (PINC), TIME3 (PINC; overflow -> T3RUPT);
 *   +5 ms    TIME5 (overflow -> T5RUPT); radar gate counter when channel
 *            13 bit 4 is set (9 counts -> RADARUPT, bit 4 reset);
 *   +7.5 ms  TIME4 (overflow -> T4RUPT);
 *   1/1600 s TIME6 (DINC) while channel 13 bit 15 enables it; the DINC at
 *            +-0 requests T6RUPT and resets channel 13 bit 15;
 *   20 ms    DOWNRUPT (the downlink word-pair clock);
 *   HANDRUPT when a trap armed by channel 13 bits 12-14 sees channel 31
 *            bits 1-6 / 7-12 or channel 32 bits 1-10 leave all-ones.
 * Pending interrupts are then taken in the AGC's priority order (vector
 * order at 4000, INTERRUPT_LEAD_INS CM p.131 / LM p.153): T6 T5 T3 T4
 * KEY1 KEY2 UP DOWN RADAR HAND, each through its lead-in in
 * src/exec/interrupts.c. A request stays pending while INHINT is in force.
 * Then the executive runs jobs until DUMMYJOB would run (EXECUTIVE C
 * p.1220), bounded per centisecond.
 *
 * DEVIATION (timing): a C job runs to completion or to its CHANG1/JOBSLEEP
 * point, so interrupts are taken between jobs, never inside one; within a
 * centisecond the counter phases above keep their order only.
 */
#include <stddef.h>
#include <stdio.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"
#include "internal.h"

#define SIM_JOBS_PER_CS 64      /* executive bound per centisecond      */
#define SIM_T6_PER_CS   16      /* TIME6 counts at 1600 per second      */
#define SIM_OVF         040000u /* 15-bit counter overflow              */
#define SIM_NEG0        077777u /* ones'-complement -0                  */
#define SIM_TIME3_LAST  037777u /* the next PINC overflows TIME3        */
#define SIM_DOWN_CS     2u      /* DOWNRUPT every 20 ms                 */
#define SIM_RADAR_GATES 9       /* radar cycle: 9 TIME5 phases          */
#define SIM_CH13_RADAR  000010u /* radar activity                       */
#define SIM_CH13_TRAPS  034000u /* reset trap 31A, 31B, 32 (bits 12-14) */
#define SIM_CH13_T31A   004000u
#define SIM_CH13_T31B   010000u
#define SIM_CH13_T32    020000u
#define SIM_CH13_T6ENB  040000u
#define SIM_CH31_ROT    000077u
#define SIM_CH31_XLT    007700u
#define SIM_CH32_TRAP   001777u

static uint32_t    g_cs;                 /* centiseconds since power-on */
static bool        g_irq[SIM_IRQ_COUNT]; /* interrupt requests          */
static int         g_radar_gate;         /* radar gate counter          */
static bool        g_trap[3];            /* HANDRUPT traps 31A 31B 32   */
static sim_stats_t g_stats;

agc_status_t sim_clock_init(void)
{
    g_cs         = 0;
    g_radar_gate = 0;

    for (int i = 0; i < (int) SIM_IRQ_COUNT; i++)
    {
        g_irq[i]         = false;
        g_stats.rupts[i] = 0;
    }

    for (int i = 0; i < 3; i++)
    {
        g_trap[i] = false;
    }

    g_stats.cs         = 0;
    g_stats.jobs       = 0;
    g_stats.restarts   = 0;
    g_stats.busy_ticks = 0;
    g_stats.errors     = 0;

    if (!AGC_ASSERT(!g_irq[SIM_IRQ_T6]) || !AGC_ASSERT(g_stats.jobs == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

void sim_irq_request(sim_irq_t irq)
{
    g_irq[irq] = true;
}

uint32_t sim_clock_cs(void)
{
    return (g_cs);
}

void sim_clock_stats(sim_stats_t *out)
{
    *out = g_stats;
}

/* PINC of TIME4 (n = 4) or TIME5 (n = 5); overflow requests the rupt.   */
static void time_pinc(int n, sim_irq_t irq)
{
    uint16_t v = (uint16_t) (agc_rupt_time(n) + 1u);

    if (v >= SIM_OVF)
    {
        v = 0;
        sim_irq_request(irq);
    }

    agc_rupt_set_time(n, v);
}

/* TIME6: DINC 16 times per centisecond while channel 13 bit 15 is set.
 * Ones' complement: +1 -> -0; the DINC at +-0 is the ZOUT that requests
 * T6RUPT, and the hardware then resets channel 13 bit 15.               */
static void time6_dinc(void)
{
    for (int i = 0; i < SIM_T6_PER_CS; i++)
    {
        if ((agc_chan_read((int) AGC_CHAN_CHAN13) & SIM_CH13_T6ENB) == 0u)
        {
            return;
        }

        uint16_t v = agc_rupt_time(6);

        if (v == 0u || v == SIM_NEG0)
        {
            sim_irq_request(SIM_IRQ_T6);
            agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (~SIM_CH13_T6ENB & SIM_WORD_MASK));

            return;
        }

        uint16_t next = (v == 1u) ? (uint16_t) SIM_NEG0 : ((v & SIM_OVF) != 0u) ? (uint16_t) (v + 1u) : (uint16_t) (v - 1u);

        agc_rupt_set_time(6, (uint16_t) (next & SIM_WORD_MASK));
    }
}

/* Radar gate (yaAGC: counted with TIME5 while channel 13 bit 4 is set). */
static void radar_gate(void)
{
    if ((agc_chan_read((int) AGC_CHAN_CHAN13) & SIM_CH13_RADAR) == 0u)
    {
        g_radar_gate = 0;
    }
    else
    {
        g_radar_gate++;

        if (g_radar_gate >= SIM_RADAR_GATES)
        { /* cycle complete: RNRAD holds what the radar shifted in    */
            g_radar_gate = 0;
            agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (~SIM_CH13_RADAR & SIM_WORD_MASK));
            sim_irq_request(SIM_IRQ_RADAR);
        }
    }
}

/* HANDRUPT traps: channel 13 bits 12-14 arm them (and read back as 0).  */
static void hand_traps(void)
{
    uint16_t c13 = agc_chan_read((int) AGC_CHAN_CHAN13);

    if ((c13 & SIM_CH13_TRAPS) != 0u)
    {
        g_trap[0] = g_trap[0] || ((c13 & SIM_CH13_T31A) != 0u);
        g_trap[1] = g_trap[1] || ((c13 & SIM_CH13_T31B) != 0u);
        g_trap[2] = g_trap[2] || ((c13 & SIM_CH13_T32) != 0u);
        agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (~SIM_CH13_TRAPS & SIM_WORD_MASK));
    }

    uint16_t c31  = agc_chan_read((int) AGC_CHAN_CHAN31);
    uint16_t c32  = agc_chan_read((int) AGC_CHAN_CHAN32);
    bool     hit0 = g_trap[0] && ((c31 & SIM_CH31_ROT) != SIM_CH31_ROT);
    bool     hit1 = g_trap[1] && ((c31 & SIM_CH31_XLT) != SIM_CH31_XLT);
    bool     hit2 = g_trap[2] && ((c32 & SIM_CH32_TRAP) != SIM_CH32_TRAP);

    if (hit0 || hit1 || hit2)
    {
        g_trap[0] = g_trap[0] && !hit0;
        g_trap[1] = g_trap[1] && !hit1;
        g_trap[2] = g_trap[2] && !hit2;
        sim_irq_request(SIM_IRQ_HAND);
    }
}

/* The counter phases of one centisecond.                               */
static void clock_counters(void)
{
    agc_time_advance_cs(1); /* TIME1/TIME2 */

    if (agc_waitlist_time3() >= SIM_TIME3_LAST)
    {
        sim_irq_request(SIM_IRQ_T3); /* the PINC is done by the T3 rupt */
    }
    else
    {
        agc_status_t st = agc_waitlist_tick(1); /* PINC TIME3, no overflow */

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            g_stats.errors++;
        }
    }

    time_pinc(5, SIM_IRQ_T5);
    radar_gate();
    time_pinc(4, SIM_IRQ_T4);
    time6_dinc();
    hand_traps();

    if ((g_cs % SIM_DOWN_CS) == 0u)
    {
        sim_irq_request(SIM_IRQ_DOWNLINK);
    }

    if (!AGC_ASSERT(agc_rupt_time(4) < SIM_OVF && agc_rupt_time(5) < SIM_OVF))
    {
        g_stats.errors++;
    }
}

/* T3RUPT: the TIME3 PINC that overflows, then the waitlist (T3RUPT, C
 * p.1231), as the lead-in does, and the RESUME restart service.        */
static agc_status_t rupt_t3(void)
{
    agc_inhint();
    agc_status_t st = agc_waitlist_tick(1);

    agc_relint();

    if (agc_exec_restart_pending())
    {
        agc_status_t rs = agc_exec_service_restart();

        g_stats.restarts++;

        if (!AGC_ASSERT(rs != AGC_STATUS_BAD_ARG))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    if (!AGC_ASSERT(!agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* Take one interrupt through its lead-in (src/exec/interrupts.c).      */
static agc_status_t take_rupt(sim_irq_t irq)
{
    switch (irq)
    {
        case SIM_IRQ_T6:
            return (agc_rupt_t6());
        case SIM_IRQ_T5:
            return (agc_rupt_t5());
        case SIM_IRQ_T3:
            return (rupt_t3());
        case SIM_IRQ_T4:
            return (agc_rupt_t4());
        case SIM_IRQ_KEY1:
            return (agc_rupt_keyrupt1());
        case SIM_IRQ_KEY2:
            return (agc_rupt_keyrupt2());
        case SIM_IRQ_UPLINK:
            return (agc_rupt_uprupt());
        case SIM_IRQ_DOWNLINK:
            return (agc_rupt_downrupt());
        case SIM_IRQ_RADAR:
            return (agc_rupt_radarupt());
        case SIM_IRQ_HAND:
            return (agc_rupt_rupt10());
        case SIM_IRQ_COUNT:
        default:
            break;
    }

    if (!AGC_ASSERT(irq < SIM_IRQ_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_NOT_FOUND);
}

/* The pending interrupts, highest priority first; held under INHINT.   */
static void clock_rupts(void)
{
    for (int i = 0; i < (int) SIM_IRQ_COUNT; i++)
    {
        if (!g_irq[i] || agc_interrupts_inhibited())
        {
            continue;
        }

        g_irq[i]        = false;
        g_stats.rupts[i]++;
        agc_status_t st = take_rupt((sim_irq_t) i);

        if (st == AGC_STATUS_BAD_ARG)
        {
            g_stats.errors++;
            (void) fprintf(stderr, "agc_sim: %lu cs: interrupt %d returned " "BAD_ARG\n", (unsigned long) g_cs, i); /* diagnostic */
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
    }

    if (!AGC_ASSERT(!agc_interrupts_inhibited()))
    {
        g_stats.errors++;
    }
}

/* The executive until DUMMYJOB (EXECUTIVE C p.1220), bounded.          */
static void clock_exec(void)
{
    int n = 0;

    for (; n < SIM_JOBS_PER_CS; n++)
    {
        if (agc_exec_restart_pending())
        {
            agc_status_t rs = agc_exec_service_restart();

            g_stats.restarts++;
            g_stats.errors += (rs == AGC_STATUS_BAD_ARG) ? 1u : 0u;
        }

        agc_exec_core_info_t job;
        int                  set = agc_exec_newjob(); /* NEWJOB: the next */
        agc_exec_core_info((set > 0) ? set : 0, &job);
        agc_status_t         st  = agc_exec_run_one();

        if (st == AGC_STATUS_NOT_FOUND)
        {
            agc_selfcheck_idle_pass(); /* DUMMYJOB -> ADVAN -> SELFRET */
            break;                     /* idle */
        }

        g_stats.jobs++;

        if (st == AGC_STATUS_BAD_ARG)
        {
            g_stats.errors++;
            (void) fprintf(stderr, "agc_sim: %lu cs: job %d returned BAD_ARG\n", /* diagnostic */
                           (unsigned long) g_cs,
                           (int) job.loc);
        }

        sim_channels_sync();
    }

    g_stats.busy_ticks += (n == SIM_JOBS_PER_CS) ? 1u : 0u;

    if (!AGC_ASSERT(n <= SIM_JOBS_PER_CS))
    {
        g_stats.errors++;
    }
}

/* One centisecond: counters, interrupts, executive, peripheral outputs. */
void sim_clock_tick(void)
{
    clock_counters();
    clock_rupts();
    clock_exec();

    if (sim_channels_tick(g_cs) != AGC_STATUS_OK)
    {
        g_stats.errors++; /* a CDU drive count the IMU refused */
    }

    sim_channels_sync();
    g_cs++;
    g_stats.cs = g_cs;

    if (!AGC_ASSERT(g_stats.cs == g_cs))
    {
        g_stats.errors++;
    }
}
