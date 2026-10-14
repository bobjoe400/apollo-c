/**********************************************************************
 * FILE NAME: address_map.h
 *
 * PURPOSE:
 *   Erasable access by AGC address (ECADR): the machine-address nouns of
 *   pinball, the uplink update program (V71/V72), the downlink erasable
 *   dump, and hosts that see the AGC as addressable memory (src/nassp).
 *   The flagwords and the named erasables of both ropes (state vectors,
 *   REFSMMAT, TEPHEM, pad loads, ...) reach their typed storage; every
 *   other address keeps a raw word (core's image, agc/core/state.h).
 *
 * AGC SOURCE:
 *   Comanche055 / Luminary099 ERASABLE_ASSIGNMENTS (addresses from the
 *   assembly listings).
 *
 * REQUIRES:
 *   none (includes status.h)
 *
 * GLOBAL VARIABLES:
 *   none (src/erasable_map/address_map.c reaches the owners' storage
 *   through their accessors)
 **********************************************************************/
#ifndef AGC_ERASABLE_MAP_ADDRESS_MAP_H
#define AGC_ERASABLE_MAP_ADDRESS_MAP_H

#include <stdint.h>

#include "agc/core/status.h"

/* Read the erasable word at ECADR (raw 15-bit word).                      */
void agc_erase_read_ecadr(uint16_t ecadr, uint16_t *word);

/* Store a raw 15-bit word at ECADR; BAD_ARG for a word above 077777 or a
 * value the owner refuses (e.g. UPSVFLAG outside -2..2).                  */
agc_status_t agc_erase_write_ecadr(uint16_t ecadr, uint16_t word);

/* Shadow words the map of the rope flown uses; the map keeps each entry's
 * typed value as last seen in core's shadow store, which holds
 * AGC_EC_SHADOW_WORDS (agc/core/state.h).                                 */
uint32_t agc_erase_map_words(void);

#endif /* AGC_ERASABLE_MAP_ADDRESS_MAP_H */
