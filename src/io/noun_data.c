/*
 * noun_data.c -- noun table lookup (LODNNTAB) and data id resolution.
 *
 * agc_noun_lookup*() fetch one row of the flat noun tables generated from
 * PINBALL_NOUN_TABLES.agc (noun_tables_csm.c / noun_tables_lm.c).
 *
 * agc_noun_read() / agc_noun_write() resolve a data id to the erasable it
 * names through ONE switch (noun_data_access) that calls the accessors in
 * include/agc/agc_state.h and agc_channels.h. Erasables core does not expose
 * yet return AGC_STATUS_UNIMPLEMENTED; the full list, with AGC scales and the
 * accessors requested, is in docs/modules/io-nouns.md "Requested core
 * additions". To wire one up, add a case here -- nothing else changes.
 *
 * Value convention (see agc_nouns.h): an SP erasable travels in the high word
 * of the agc_dp_t, a DP erasable as the agc_dp_t itself.
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "agc/agc_channels.h"
#include "agc/agc_nouns.h"
#include "agc/agc_state.h"

/* LODNNTAB (PINBALL_NOUN_TABLES.agc p. 270): row for one noun of one vehicle. */
agc_status_t agc_noun_lookup_vehicle(agc_vehicle_t veh, uint8_t noun,
                                     agc_noun_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(noun < AGC_NOUN_COUNT)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_noun_t *row = (veh == AGC_VEHICLE_CSM) ? &agc_noun_table_csm[noun]
                                                      : &agc_noun_table_lm[noun];
    if (!AGC_ASSERT(row->noun == noun) || !AGC_ASSERT(row->ncomp <= AGC_NOUN_MAX_COMP)) {
        return AGC_STATUS_BAD_ARG;              /* table corrupted */
    }
    *out = *row;
    return (row->ncomp == 0) ? AGC_STATUS_NOT_FOUND : AGC_STATUS_OK;
}

/* Row for a noun of the vehicle selected by agc_state_init(). */
agc_status_t agc_noun_lookup(uint8_t noun, agc_noun_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(noun < AGC_NOUN_COUNT)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_noun_lookup_vehicle(agc_state_vehicle(), noun, out);
}

/* 15-bit AGC channel word (1s complement) as an SP value. */
static agc_sp_t noun_word15_to_sp(uint16_t w)
{
    if (!AGC_ASSERT(w <= 077777u)) {
        return 0;
    }
    if (w >= 040000u) {
        return (agc_sp_t)(-(int)(077777u - w));
    }
    return (agc_sp_t)w;
}

/* Channel 33 (LM noun 66 component 2, scale TT): input only, no load. */
static agc_status_t noun_access_chan33(bool write, agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(!write)) {
        return AGC_STATUS_BAD_ARG;
    }
    *value = agc_dp_from_sp(noun_word15_to_sp(agc_chan_read((int)AGC_CHAN_CHAN33)));
    return AGC_STATUS_OK;
}

/* The one data id -> erasable switch. */
static agc_status_t noun_data_access(uint16_t data_id, bool write, agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT) ||
        !AGC_ASSERT(data_id != AGC_DATA_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    switch (data_id) {
    case AGC_DATA_SPECIFY_ADDR:       /* nouns 01-03, 10, 15: the pinball */
    case AGC_DATA_SPECIFY_CHANNEL:    /* module supplies the address      */
    case AGC_DATA_INCREMENT_ADDR:
        return AGC_STATUS_NOT_FOUND;
    case AGC_DATA_CHANNEL33:
        return noun_access_chan33(write, value);
    case AGC_DATA_TIME2:              /* C(TIME2,TIME1), cs, scale 2^28   */
        if (write) {
            if (!AGC_ASSERT(*value >= 0)) {
                return AGC_STATUS_BAD_ARG;
            }
            agc_time_set_cs((uint32_t)*value);
        } else {
            *value = (agc_dp_t)agc_time_now_cs();
        }
        return AGC_STATUS_OK;
    case AGC_DATA_TIG:                /* TIG, cs, scale 2^28              */
        if (write) {
            agc_erase_set_tig(*value);
        } else {
            *value = agc_erase_tig();
        }
        return AGC_STATUS_OK;
    default:                          /* not yet exposed by core          */
        return AGC_STATUS_UNIMPLEMENTED;
    }
}

/* Read the erasable behind a data id. */
agc_status_t agc_noun_read(uint16_t data_id, agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT)) {
        return AGC_STATUS_BAD_ARG;
    }
    return noun_data_access(data_id, false, value);
}

/* Store a value into the erasable behind a data id. */
agc_status_t agc_noun_write(uint16_t data_id, agc_dp_t value)
{
    agc_dp_t v = value;
    if (!AGC_ASSERT(data_id < AGC_DATA_COUNT) ||
        !AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    return noun_data_access(data_id, true, &v);
}
