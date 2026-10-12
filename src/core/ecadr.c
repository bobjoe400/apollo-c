/*
 * ecadr.c -- erasable access by AGC address (ECADR), for the machine-address
 * nouns of pinball (nouns 01-03, 07, 15: V01/V21/V25 of any erasable) and
 * the uplink update program (V71/V72).
 *
 * The C translation keeps each erasable in its owner's typed storage, not
 * in an addressable array. The flagwords (STATE +0.., ECADR 074 in both
 * ropes, so FLAGWRD0..FLGWRD13 = 074..0111) map onto agc_flagword_get/set,
 * which is what noun 07 (flag-bit set/reset) needs. Every other address
 * reads and writes a raw image of the 15-bit address space (0000-77777:
 * erasable E0..E7 at 0000-3777, then the fixed-memory pseudo addresses,
 * which hold no rope here and simply keep what was stored).
 * DEVIATION: a store through an address other than a flagword does not
 * reach the typed erasable of the same name (no address map yet); the raw
 * image keeps the value so a later V01/V05 of the same address reads it.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"

#define EC_WORDS    0100000u /* address 00000 .. 77777                  */
#define EC_STATE    0074u    /* STATE = FLAGWRD0 (both ropes, unswitched)   */
#define EC_WORD_MAX 077777u

static uint16_t g_image[EC_WORDS];

static bool is_flagword(uint16_t ecadr)
{
    uint16_t n = (agc_state_vehicle() == AGC_VEHICLE_LM) ? (uint16_t) AGC_FLAGWORDS_LM : (uint16_t) AGC_FLAGWORDS_CSM;

    return ((ecadr >= EC_STATE) && (ecadr < EC_STATE + n));
}

void agc_erase_reset_ecadr(void)
{
    for (uint32_t i = 0; i < EC_WORDS; i++)
    {
        g_image[i] = 0;
    }
}

/* Read the erasable word at ECADR (raw 15-bit word).                     */
agc_status_t agc_erase_read_ecadr(uint16_t ecadr, uint16_t *word)
{
    if (is_flagword(ecadr))
    {
        *word = (uint16_t) (agc_flagword_get((int) (ecadr - EC_STATE)) & EC_WORD_MAX);
    }
    else
    {
        *word = g_image[ecadr];
    }

    return (AGC_STATUS_OK);
}

/* Store a raw 15-bit word at ECADR.                                      */
agc_status_t agc_erase_write_ecadr(uint16_t ecadr, uint16_t word)
{
    if (!AGC_ASSERT(word <= EC_WORD_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (is_flagword(ecadr))
    {
        agc_flagword_set((int) (ecadr - EC_STATE), word);
    }
    else
    {
        g_image[ecadr] = word;
    }

    return (AGC_STATUS_OK);
}
