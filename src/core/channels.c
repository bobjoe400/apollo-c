/*
 * agc_channels.c -- the 15-bit I/O channel image.
 *
 * Output channels (1, 2, 5-7, 010-014, 034, 035) are plain 15-bit latches
 * written by WRITE / WOR / WAND. Input channels (3, 4, 015, 016, 030-033) are
 * set only by the simulator through agc_chan_set_input() /
 * agc_chan_set_signal(); a program write to one of them is a program error
 * (AGC_STATUS_BAD_ARG), with one exception below.
 *
 * Channels 030-033 are inverted-logic inputs: the image holds exactly what
 * READ returns, a ZERO bit meaning "signal present"; power-up value 077777
 * (nothing present). Channel 033 bits 11-15 are flip-flops (UPLINK TOO FAST,
 * DOWNLINK TOO FAST, PIPA FAIL, REPEATED ALARMS, OSCILLATOR STOPPED) that the
 * hardware resets on ANY write pulse to channel 033: the AGC does this with
 * WAND CHAN33 (T4RUPT C33TEST, FRESH_START_AND_RESTART). The model: WRITE,
 * WOR or WAND on channel 033 returns bits 11-15 of the image to the
 * inactive state (1) and leaves bits 1-10 alone. The simulator re-asserts a
 * flip-flop with agc_chan_set_signal() when its condition persists.
 *
 * DEVIATION: no channel 0 (indexed-operation-only in the AGC) and no
 * L/Q register aliasing beyond storing the 15-bit value.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"

static uint16_t g_chan[AGC_CHAN_COUNT];

static bool chan_valid(int chan)
{
    return (chan >= 1) && (chan < AGC_CHAN_COUNT);
}

static bool chan_inverted(int chan)
{
    if (!AGC_ASSERT(chan >= 0))
    {
        return false;
    }

    return (chan >= AGC_CHAN_FIRST_INVERTED) &&
           (chan <= AGC_CHAN_LAST_INVERTED);
}

/* Channels a program may write: L, Q, outputs, and 033 (flip-flop reset). */
static bool chan_writable(int chan)
{
    if (!AGC_ASSERT(chan >= 0))
    {
        return false;
    }

    if (!AGC_ASSERT(chan < AGC_CHAN_COUNT))
    {
        return false;
    }

    switch (chan)
    {
        case AGC_CHAN_L:
        case AGC_CHAN_Q:
        case AGC_CHAN_PYJETS:
        case AGC_CHAN_ROLLJETS:
        case AGC_CHAN_SUPERBNK:
        case AGC_CHAN_OUT0:
        case AGC_CHAN_DSALMOUT:
        case AGC_CHAN_CHAN12:
        case AGC_CHAN_CHAN13:
        case AGC_CHAN_CHAN14:
        case AGC_CHAN_CHAN33:
        case AGC_CHAN_DNTM1:
        case AGC_CHAN_DNTM2:
            return true;
        default:
            return false;
    }
}

/* Inputs the simulator may drive. */
static bool chan_input(int chan)
{
    if (!AGC_ASSERT(chan >= 0))
    {
        return false;
    }

    if (!AGC_ASSERT(chan < AGC_CHAN_COUNT))
    {
        return false;
    }

    switch (chan)
    {
        case AGC_CHAN_HISCALAR:
        case AGC_CHAN_LOSCALAR:
        case AGC_CHAN_MNKEYIN:
        case AGC_CHAN_NAVKEYIN:
        case AGC_CHAN_CHAN30:
        case AGC_CHAN_CHAN31:
        case AGC_CHAN_CHAN32:
        case AGC_CHAN_CHAN33:
            return true;
        default:
            return false;
    }
}

void agc_chan_reset(void)
{
    for (int i = 0; i < AGC_CHAN_COUNT; i++)
    {
        g_chan[i] = chan_inverted(i) ? (uint16_t) AGC_CHAN_MASK : 0u;
    }

    if (!AGC_ASSERT(g_chan[AGC_CHAN_CHAN30] == AGC_CHAN_MASK))
    {
        return;
    }
}

uint16_t agc_chan_read(int chan)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return 0;
    }

    if (!AGC_ASSERT(g_chan[chan] <= AGC_CHAN_MASK))
    {
        return 0;
    }

    return g_chan[chan];
}

/* Channel 033 write pulse: flip-flop bits 11-15 return to inactive (1). */
static void reset_flipflops(void)
{
    if (!AGC_ASSERT(g_chan[AGC_CHAN_CHAN33] <= AGC_CHAN_MASK))
    {
        return;
    }

    g_chan[AGC_CHAN_CHAN33] =
        (uint16_t) (g_chan[AGC_CHAN_CHAN33] | AGC_CH033_FLIPFLOP_MASK);
}

void agc_chan_write(int chan, uint16_t value)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return;
    }

    if (!AGC_ASSERT(value <= AGC_CHAN_MASK))
    {
        return;
    }

    if (!AGC_ASSERT(chan_writable(chan)))
    {
        return;
    }

    if (chan == AGC_CHAN_CHAN33)
    {
        reset_flipflops();
    }
    else
    {
        g_chan[chan] = value;

        if (!AGC_ASSERT(g_chan[chan] == value))
        {
            return;
        }
    }
}

void agc_chan_or(int chan, uint16_t mask)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return;
    }

    if (!AGC_ASSERT(mask <= AGC_CHAN_MASK))
    {
        return;
    }

    if (!AGC_ASSERT(chan_writable(chan)))
    {
        return;
    }

    if (chan == AGC_CHAN_CHAN33)
    {
        reset_flipflops();
    }
    else
    {
        g_chan[chan] = (uint16_t) (g_chan[chan] | mask);

        if (!AGC_ASSERT((g_chan[chan] & mask) == mask))
        {
            return;
        }
    }
}

void agc_chan_and(int chan, uint16_t mask)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return;
    }

    if (!AGC_ASSERT(mask <= AGC_CHAN_MASK))
    {
        return;
    }

    if (!AGC_ASSERT(chan_writable(chan)))
    {
        return;
    }

    if (chan == AGC_CHAN_CHAN33)
    {
        reset_flipflops();
    }
    else
    {
        g_chan[chan] = (uint16_t) (g_chan[chan] & mask);

        if (!AGC_ASSERT((g_chan[chan] & (uint16_t) ~mask) == 0u))
        {
            return;
        }
    }
}

agc_status_t agc_chan_set_input(int chan, uint16_t value)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(value <= AGC_CHAN_MASK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!chan_input(chan))
    {
        return AGC_STATUS_BAD_ARG;
    }

    g_chan[chan] = value;

    if (!AGC_ASSERT(g_chan[chan] == value))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

agc_status_t agc_chan_set_signal(int chan, uint16_t mask, bool present)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(mask <= AGC_CHAN_MASK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!chan_input(chan))
    {
        return AGC_STATUS_BAD_ARG;
    }

    /* Inverted channels: signal present = bit cleared. */
    if (present == chan_inverted(chan))
    {
        g_chan[chan] = (uint16_t) (g_chan[chan] & ~mask);
    }
    else
    {
        g_chan[chan] = (uint16_t) (g_chan[chan] | mask);
    }

    if (!AGC_ASSERT(g_chan[chan] <= AGC_CHAN_MASK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}
