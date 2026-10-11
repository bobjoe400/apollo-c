/**********************************************************************
 * FILE NAME: channels.c
 *
 * PURPOSE:
 *   The 15-bit I/O channel image behind READ, WRITE, WOR, WAND (ROR and
 *   RXOR are reads combined by the caller), plus the simulator's input
 *   side.
 *
 * AGC SOURCE:
 *   Luminary099 INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc pp. 54-59;
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc p. 41 (channel names);
 *   Comanche055 T4RUPT_PROGRAM.agc p. 145 (C33TEST: WAND CHAN33 resets
 *   the channel 33 flip-flops)
 *
 * EXTERNAL VARIABLES:  (module-static)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   g_chan        uint16_t  I/O  bit words      channel 0..037 image
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No AGC alarms. A program write to an input channel, a bad channel
 *   number or a value wider than 15 bits is reported by AGC_ASSERT and
 *   ignored; agc_chan_set_input/agc_chan_set_signal return
 *   AGC_STATUS_BAD_ARG for a channel that is not an input.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Output channels (1, 2, 5-7, 010-014, 034, 035) are plain latches.
 *   Input channels (3, 4, 015, 016, 030-033) are set only by the
 *   simulator. Channels 030-033 are inverted: the image holds what READ
 *   returns, ZERO = signal present, power-up 077777.
 *
 * NOTES:
 *   Channel 033 bits 11-15 are flip-flops (uplink too fast, downlink too
 *   fast, PIPA fail, repeated alarms, oscillator stopped) that any write
 *   pulse to channel 033 resets; the AGC does it with WAND CHAN33
 *   (C33TEST, FRESH_START_AND_RESTART). Model: WRITE, WOR or WAND on 033
 *   returns bits 11-15 to the inactive state (1) and leaves bits 1-10.
 *   DEVIATION: no channel 0 (indexed-operation-only in the AGC) and no
 *   L/Q register aliasing beyond the 15-bit latch; superbank channel 7
 *   "not reset by restart" is not modelled (agc_chan_reset is power-up).
 *
 * ALGORITHM:
 *   none (storage with masking)
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"

static uint16_t g_chan[AGC_CHAN_COUNT]; /* channel image, 15-bit words */

/**********************************************************************
 * AGC: none (helper) -- true for a modelled channel number 1..037
 *
 * FUNCTION NAME: chan_valid
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   chan      int   I    channel number (octal in the AGC)
 *
 * RETURNS: bool -- true when 1 <= chan < AGC_CHAN_COUNT
 **********************************************************************/
static bool chan_valid(int chan)
{
    return ((chan >= 1) && (chan < AGC_CHAN_COUNT));
}

/**********************************************************************
 * AGC: none (helper; INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS L p. 54:
 *      channels 30-33 are inverted)
 *
 * FUNCTION NAME: chan_inverted
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   chan      int   I    channel number
 *
 * RETURNS: bool -- true for the inverted-logic inputs 030..033
 **********************************************************************/
static bool chan_inverted(int chan)
{
    if (!AGC_ASSERT(chan >= 0))
    {
        return (false);
    }

    return ((chan >= AGC_CHAN_FIRST_INVERTED) &&
            (chan <= AGC_CHAN_LAST_INVERTED));
}

/**********************************************************************
 * AGC: none (helper) -- channels a program may write: L, Q, the
 *      outputs, and 033 (flip-flop reset pulse)
 *
 * FUNCTION NAME: chan_writable
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   chan      int   I    channel number
 *
 * RETURNS: bool -- true when WRITE/WOR/WAND may address chan
 **********************************************************************/
static bool chan_writable(int chan)
{
    if (!AGC_ASSERT(chan >= 0))
    {
        return (false);
    }

    if (!AGC_ASSERT(chan < AGC_CHAN_COUNT))
    {
        return (false);
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
            return (true);
        default:
            return (false);
    }
}

/**********************************************************************
 * AGC: none (helper) -- inputs the simulator may drive
 *
 * FUNCTION NAME: chan_input
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   chan      int   I    channel number
 *
 * RETURNS: bool -- true for channels 3, 4, 015, 016, 030-033
 **********************************************************************/
static bool chan_input(int chan)
{
    if (!AGC_ASSERT(chan >= 0))
    {
        return (false);
    }

    if (!AGC_ASSERT(chan < AGC_CHAN_COUNT))
    {
        return (false);
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
            return (true);
        default:
            return (false);
    }
}

/**********************************************************************
 * AGC: none (power-up channel image: outputs 0, inverted inputs 077777)
 *
 * FUNCTION NAME: agc_chan_reset
 **********************************************************************/
void agc_chan_reset(void)
{
    for (int i = 0; i < AGC_CHAN_COUNT; i++)
    {
        g_chan[i] = 0u;

        if (chan_inverted(i))
        {
            g_chan[i] = (uint16_t) AGC_CHAN_MASK; /* no signal present */
        }
    }

    if (!AGC_ASSERT(g_chan[AGC_CHAN_CHAN30] == AGC_CHAN_MASK))
    {
        return;
    }
}

/**********************************************************************
 * AGC: none (machine instruction READ; also the read half of ROR,
 *      RXOR)
 *
 * FUNCTION NAME: agc_chan_read
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   chan      int   I    channel number 1..037
 *
 * RETURNS: uint16_t -- the 15-bit channel word (0 for a bad channel)
 **********************************************************************/
uint16_t agc_chan_read(int chan)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return (0);
    }

    if (!AGC_ASSERT(g_chan[chan] <= AGC_CHAN_MASK))
    {
        return (0);
    }

    return (g_chan[chan]);
}

/**********************************************************************
 * AGC: WAND CHAN33 (T4RUPT_PROGRAM.agc C p. 145, C33TEST) -- a write
 *      pulse to channel 033 returns flip-flop bits 11-15 to inactive (1)
 *
 * FUNCTION NAME: reset_flipflops
 **********************************************************************/
static void reset_flipflops(void)
{
    if (!AGC_ASSERT(g_chan[AGC_CHAN_CHAN33] <= AGC_CHAN_MASK))
    {
        return;
    }

    g_chan[AGC_CHAN_CHAN33] =
        (uint16_t) (g_chan[AGC_CHAN_CHAN33] | AGC_CH033_FLIPFLOP_MASK);
}

/**********************************************************************
 * AGC: none (machine instruction WRITE)
 *
 * FUNCTION NAME: agc_chan_write
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ---------------------------------------
 *   chan      int       I    writable channel (see chan_writable)
 *   value     uint16_t  I    15-bit word to latch
 **********************************************************************/
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
    }
}

/**********************************************************************
 * AGC: none (machine instruction WOR)
 *
 * FUNCTION NAME: agc_chan_or
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ---------------------------------------
 *   chan      int       I    writable channel
 *   mask      uint16_t  I    15-bit mask of bits to set
 **********************************************************************/
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

/**********************************************************************
 * AGC: none (machine instruction WAND)
 *
 * FUNCTION NAME: agc_chan_and
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ---------------------------------------
 *   chan      int       I    writable channel
 *   mask      uint16_t  I    15-bit mask of bits to keep
 **********************************************************************/
void agc_chan_and(int chan, uint16_t mask)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return;
    }

    /* WAND operates on 15-bit words. A caller writing ~BIT on a 15-bit
     * value sets bit 16 in a uint16_t; the AGC never sees that bit, so
     * neither do we. */
    mask = (uint16_t) (mask & AGC_CHAN_MASK);

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

/**********************************************************************
 * AGC: none (simulator: the hardware drives an input channel)
 *
 * FUNCTION NAME: agc_chan_set_input
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ---------------------------------------
 *   chan      int       I    input channel 3, 4, 015, 016, 030-033
 *   value     uint16_t  I    raw word as READ returns it (inverted
 *                            sense on 030-033)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          channel that is not an input or a value over 15 bits
 **********************************************************************/
agc_status_t agc_chan_set_input(int chan, uint16_t value)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(value <= AGC_CHAN_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!chan_input(chan))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    g_chan[chan] = value;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (simulator: one discrete signal on an input channel)
 *
 * FUNCTION NAME: agc_chan_set_signal
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ---------------------------------------
 *   chan      int       I    input channel
 *   mask      uint16_t  I    bit(s) of the signal (AGC_CH0nn_*)
 *   present   bool      I    true = signal present; the inversion of
 *                            channels 030-033 is applied here
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          channel that is not an input or a mask over 15 bits
 **********************************************************************/
agc_status_t agc_chan_set_signal(int chan, uint16_t mask, bool present)
{
    if (!AGC_ASSERT(chan_valid(chan)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(mask <= AGC_CHAN_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!chan_input(chan))
    {
        return (AGC_STATUS_BAD_ARG);
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
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}
