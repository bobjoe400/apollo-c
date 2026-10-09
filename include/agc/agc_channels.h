/*
 * agc_channels.h -- I/O channel image.
 *
 * The AGC has 15-bit I/O channels 1..35 (octal). READ/WRITE/ROR/WOR/WAND/
 * RXOR map onto the functions below. Bit-name constants are maintained by
 * the state-flags agent from INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc (LM)
 * and the CM erasable comments; that agent MAY edit this file.
 */
#ifndef AGC_CHANNELS_H
#define AGC_CHANNELS_H

#include <stdint.h>
#include "agc_status.h"

#define AGC_CHAN_COUNT 040           /* channels 0..037                     */

enum agc_channel_number {
    AGC_CHAN_L        = 001,
    AGC_CHAN_Q        = 002,
    AGC_CHAN_HISCALAR = 003,
    AGC_CHAN_LOSCALAR = 004,
    AGC_CHAN_PYJETS   = 005,
    AGC_CHAN_ROLLJETS = 006,
    AGC_CHAN_SUPERBNK = 007,
    AGC_CHAN_OUT0     = 010,         /* DSKY display relays               */
    AGC_CHAN_DSALMOUT = 011,         /* DSKY lamps                        */
    AGC_CHAN_CHAN12   = 012,         /* IMU / optics moding               */
    AGC_CHAN_CHAN13   = 013,
    AGC_CHAN_CHAN14   = 014,         /* CDU drive, gyro, thrust           */
    AGC_CHAN_MNKEYIN  = 015,         /* main DSKY keycode                 */
    AGC_CHAN_NAVKEYIN = 016,         /* nav DSKY keycode / mark buttons   */
    AGC_CHAN_CHAN30   = 030,
    AGC_CHAN_CHAN31   = 031,
    AGC_CHAN_CHAN32   = 032,
    AGC_CHAN_CHAN33   = 033,
    AGC_CHAN_DNTM1    = 034,         /* downlink word 1                   */
    AGC_CHAN_DNTM2    = 035
};

uint16_t     agc_chan_read(int chan);                   /* READ          */
agc_status_t agc_chan_write(int chan, uint16_t value);  /* WRITE         */
agc_status_t agc_chan_or(int chan, uint16_t mask);      /* WOR           */
agc_status_t agc_chan_and(int chan, uint16_t mask);     /* WAND          */
agc_status_t agc_chan_set_input(int chan, uint16_t value); /* sim side   */
void         agc_chan_reset(void);

#endif /* AGC_CHANNELS_H */
