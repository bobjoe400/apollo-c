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
 *   g_wlog        struct    I/O  words          the last WRITE/WOR/WAND
 *                                                (channel, word after it)
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

#include "agc/core/assertion.h"
#include "agc/core/channels.h"

/* The program's channel writes in order, for the simulator's peripheral
 * packets (yaAGC sends one per WRITE, WOR, WAND: ChannelOutput).        */
typedef struct chan_wlog
{
    uint16_t chan[AGC_CHAN_WLOG_MAX];  /* channel written            */
    uint16_t value[AGC_CHAN_WLOG_MAX]; /* its word after the write    */
    int      head;                     /* oldest unread entry         */
    int      size;                     /* unread entries              */
} chan_wlog_t;

static uint16_t    g_chan[AGC_CHAN_COUNT]; /* channel image, 15-bit words */
static chan_wlog_t g_wlog;                 /* writes not yet read          */

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
    return ((chan >= AGC_CHAN_FIRST_INVERTED) && (chan <= AGC_CHAN_LAST_INVERTED));
}

/**********************************************************************
 * AGC: none (helper) -- channels a program may write: L, Q, the
 *      outputs, and 033 (flip-flop reset pulse)
 *
 * FUNCTION NAME: agc_chan_writable
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   chan      int   I    channel number
 *
 * RETURNS: bool -- true when WRITE/WOR/WAND may address chan
 **********************************************************************/
bool agc_chan_writable(int chan)
{
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
    g_wlog.head = 0;
    g_wlog.size = 0;

    for (int i = 0; i < AGC_CHAN_COUNT; i++)
    {
        g_chan[i] = 0u;

        if (chan_inverted(i))
        {
            g_chan[i] = (uint16_t) AGC_CHAN_MASK; /* no signal present */
        }
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
    return (g_chan[chan]);
}

/**********************************************************************
 * AGC: none (simulator: a write pulse on the channel) -- record a
 *      WRITE, WOR or WAND with the channel's word after it; when the
 *      ring is full the oldest entry is dropped (a host that does not
 *      read the log loses nothing it uses).
 *
 * FUNCTION NAME: wlog_add
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   chan      int   I    channel written
 **********************************************************************/
static void wlog_add(int chan)
{
    if (g_wlog.size == AGC_CHAN_WLOG_MAX)
    {
        g_wlog.head = (g_wlog.head + 1) % AGC_CHAN_WLOG_MAX; /* drop the oldest */
        g_wlog.size--;
    }

    int i = (g_wlog.head + g_wlog.size) % AGC_CHAN_WLOG_MAX; /* the new entry */

    g_wlog.chan[i]  = (uint16_t) chan;
    g_wlog.value[i] = g_chan[chan];
    g_wlog.size++;
}

/**********************************************************************
 * AGC: none (simulator: yaAGC ChannelOutput) -- the oldest program
 *      write not yet read: a peripheral sees every WRITE, WOR and WAND
 *      as a packet, in order, even one that leaves the word unchanged
 *      (the LM Simulator fires its jets from those repeats).
 *
 * FUNCTION NAME: agc_chan_next_write
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------------------------
 *   chan      uint16_t *  O    channel written
 *   value     uint16_t *  O    its 15-bit word after the write
 *
 * RETURNS: bool -- false when every write has been read
 **********************************************************************/
bool agc_chan_next_write(uint16_t *chan, uint16_t *value)
{
    bool any = g_wlog.size > 0; /* a write is waiting */

    if (any)
    {
        *chan       = g_wlog.chan[g_wlog.head];
        *value      = g_wlog.value[g_wlog.head];
        g_wlog.head = (g_wlog.head + 1) % AGC_CHAN_WLOG_MAX;
        g_wlog.size--;
    }

    return (any);
}

/**********************************************************************
 * AGC: WAND CHAN33 (T4RUPT_PROGRAM.agc C p. 145, C33TEST) -- a write
 *      pulse to channel 033 returns flip-flop bits 11-15 to inactive (1)
 *
 * FUNCTION NAME: reset_flipflops
 **********************************************************************/
static void reset_flipflops(void)
{
    g_chan[AGC_CHAN_CHAN33] = (uint16_t) (g_chan[AGC_CHAN_CHAN33] | AGC_CH033_FLIPFLOP_MASK);
}

/**********************************************************************
 * AGC: none (machine instruction WRITE)
 *
 * FUNCTION NAME: agc_chan_write
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ---------------------------------------
 *   chan      int       I    writable channel (see agc_chan_writable)
 *   value     uint16_t  I    15-bit word to latch
 **********************************************************************/
void agc_chan_write(int chan, uint16_t value)
{
    if (chan == AGC_CHAN_CHAN33)
    {
        reset_flipflops();
    }
    else
    {
        g_chan[chan] = value;
    }

    wlog_add(chan);
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
    if (chan == AGC_CHAN_CHAN33)
    {
        reset_flipflops();
    }
    else
    {
        g_chan[chan] = (uint16_t) (g_chan[chan] | mask);
    }

    wlog_add(chan);
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
    /* WAND operates on 15-bit words. A caller writing ~BIT on a 15-bit
     * value sets bit 16 in a uint16_t; the AGC never sees that bit, so
     * neither do we. */
    mask = (uint16_t) (mask & AGC_CHAN_MASK);

    if (chan == AGC_CHAN_CHAN33)
    {
        reset_flipflops();
    }
    else
    {
        g_chan[chan] = (uint16_t) (g_chan[chan] & mask);
    }

    wlog_add(chan);
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

    return (AGC_STATUS_OK);
}
