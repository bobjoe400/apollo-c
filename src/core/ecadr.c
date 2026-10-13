/*
 * ecadr.c -- the raw image of the AGC's 15-bit address space (0000-77777:
 * erasable E0..E7 at 0000-3777, then the fixed-memory pseudo addresses,
 * which hold no rope here). Erasable access by address (ECADR) is
 * src/erasable_map/address_map.c, which maps the flagwords and the named
 * erasables onto their typed storage and keeps every word as last stored
 * here; this file is only the image, so core depends on nothing above it.
 */
#include <stdint.h>

#include "agc/core/state.h"

#define EC_WORDS    0100000u /* address 00000 .. 77777 */
#define EC_ADDR_MAX 077777u  /* a 15-bit address       */

static uint16_t g_image[EC_WORDS];

/* Fresh start: every word +0.                                            */
void agc_erase_reset_ecadr(void)
{
    for (uint32_t i = 0; i < EC_WORDS; i++)
    {
        g_image[i] = 0;
    }
}

/* The word stored at a 15-bit address (bit 16 of ecadr ignored).         */
void agc_erase_raw_read(uint16_t ecadr, uint16_t *word)
{
    *word = g_image[ecadr & EC_ADDR_MAX];
}

/* Store a word at a 15-bit address (bit 16 of ecadr ignored).            */
void agc_erase_raw_write(uint16_t ecadr, uint16_t word)
{
    g_image[ecadr & EC_ADDR_MAX] = word;
}
