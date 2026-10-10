/*
 * noun_data.c -- noun table lookup (LODNNTAB) and data id resolution.
 *
 * agc_noun_lookup*() fetch one row of the flat noun tables generated from
 * PINBALL_NOUN_TABLES.agc (noun_tables_csm.c / noun_tables_lm.c).
 *
 * agc_noun_read() / agc_noun_write() resolve a data id to the erasable it
 * names through ONE dispatch switch (noun_data_access); the per-family helpers
 * above it hold the calls to the accessors in agc_state.h / agc_channels.h
 * (split only to keep every function within the 60-line rule). Erasables core does not expose
 * yet return AGC_STATUS_UNIMPLEMENTED; the full list, with AGC scales and the
 * accessors requested, is in docs/modules/io-nouns.md "Requested core
 * additions". To wire one up, add a case here -- nothing else changes.
 *
 * Value convention (see agc_nouns.h): an SP erasable travels in the high word
 * of the agc_dp_t, a DP erasable as the agc_dp_t itself.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/io/nouns.h"

/* LODNNTAB (PINBALL_NOUN_TABLES.agc p. 270): row for one noun of one vehicle. */
agc_status_t agc_noun_lookup_vehicle(agc_vehicle_t veh, uint8_t noun,
                                     agc_noun_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(noun < AGC_NOUN_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_noun_t *row = (veh == AGC_VEHICLE_CSM) ?
                            &agc_noun_table_csm[noun] :
                            &agc_noun_table_lm[noun];

    if (!AGC_ASSERT(row->noun == noun) ||
        !AGC_ASSERT(row->ncomp <= AGC_NOUN_MAX_COMP))
    {
        return (AGC_STATUS_BAD_ARG); /* table corrupted */
    }

    *out = *row;

    return ((row->ncomp == 0) ? AGC_STATUS_NOT_FOUND : AGC_STATUS_OK);
}

/* Row for a noun of the vehicle selected by agc_state_init(). */
agc_status_t agc_noun_lookup(uint8_t noun, agc_noun_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(noun < AGC_NOUN_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_noun_lookup_vehicle(agc_state_vehicle(), noun, out));
}

/* Channel 33 (LM noun 66 component 2, scale TT): input only, no load. The
 * 15-bit image is passed raw (the LR position routine tests bits 6 and 7). */
static agc_status_t noun_access_chan33(bool write, agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(!write))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *value = (agc_dp_t) (agc_chan_read((int) AGC_CHAN_CHAN33) & 077777u);

    return (AGC_STATUS_OK);
}

/* DP times in centiseconds, scale 2^28: TIME2, TIG, TTOGO, TTPI. */
static agc_status_t noun_access_time(uint16_t data_id, bool write,
                                     agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT) ||
        !AGC_ASSERT(!write ||
                    (*value >= 0 && *value < (agc_dp_t) AGC_TIME_MODULUS_CS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    switch (data_id)
    {
        case AGC_DATA_TIME2: /* C(TIME2,TIME1)                   */

            if (write)
            {
                agc_time_set_cs((uint32_t) *value);

                return (AGC_STATUS_OK);
            }

            *value = (agc_dp_t) agc_time_now_cs();

            return (AGC_STATUS_OK);
        case AGC_DATA_TIG:

            if (write)
            {
                agc_erase_set_tig(*value);
            }
            else
            {
                *value = agc_erase_tig();
            }

            return (AGC_STATUS_OK);
        case AGC_DATA_TTOGO:

            if (write)
            {
                agc_erase_set_ttogo(*value);
            }
            else
            {
                *value = agc_erase_ttogo();
            }

            return (AGC_STATUS_OK);
        case AGC_DATA_TTPI:

            if (write)
            {
                agc_erase_set_ttpi(*value);
            }
            else
            {
                *value = agc_erase_ttpi();
            }

            return (AGC_STATUS_OK);
        default:
            return (AGC_STATUS_BAD_ARG);
    }
}

/* SP words (ones' complement, SP in the high word): PIPAs, masses. */
static agc_status_t noun_access_sp(uint16_t data_id, bool write,
                                   agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_sp_t w = write ? agc_dp_high(*value) : 0;
    agc_sp_t       r = 0;

    switch (data_id)
    {
        case AGC_DATA_PIPAX:

            if (write)
            {
                agc_erase_set_pipax(w);
            }
            else
            {
                r = agc_erase_pipax();
            }

            break;
        case AGC_DATA_PIPAY:

            if (write)
            {
                agc_erase_set_pipay(w);
            }
            else
            {
                r = agc_erase_pipay();
            }

            break;
        case AGC_DATA_PIPAZ:

            if (write)
            {
                agc_erase_set_pipaz(w);
            }
            else
            {
                r = agc_erase_pipaz();
            }

            break;
        case AGC_DATA_CSMMASS:

            if (write)
            {
                agc_erase_set_csmmass(w);
            }
            else
            {
                r = agc_erase_csmmass();
            }

            break;
        case AGC_DATA_LEMMASS:

            if (write)
            {
                agc_erase_set_lemmass(w);
            }
            else
            {
                r = agc_erase_lemmass();
            }

            break;
        default:
            return (AGC_STATUS_BAD_ARG);
    }

    if (!write)
    {
        if (!AGC_ASSERT(r >= AGC_SP_MIN && r <= AGC_SP_MAX))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        *value = agc_dp_from_sp(r);
    }

    return (AGC_STATUS_OK);
}

/* Raw 15-bit words, read side: CDUs, DAP words, alarm codes, THETAD. */
static agc_status_t noun_raw_read(uint16_t data_id, agc_dp_t *value)
{
    uint16_t w = 0;

    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    switch (data_id)
    {
        case AGC_DATA_CDUX:
            w = agc_erase_cdux();
            break;
        case AGC_DATA_CDUY:
            w = agc_erase_cduy();
            break;
        case AGC_DATA_CDUZ:
            w = agc_erase_cduz();
            break;
        case AGC_DATA_CDUS:
            w = agc_erase_cdus();
            break;
        case AGC_DATA_CDUT:
            w = agc_erase_cdut();
            break;
        case AGC_DATA_DAPDATR1:
            w = agc_erase_dapdatr1();
            break;
        case AGC_DATA_DAPDATR2:
            w = agc_erase_dapdatr2();
            break;
        case AGC_DATA_FAILREG:
            w = agc_erase_failreg(0);
            break;
        case AGC_DATA_FAILREG_P1:
            w = agc_erase_failreg(1);
            break;
        case AGC_DATA_FAILREG_P2:
            w = agc_erase_failreg(2);
            break;
        case AGC_DATA_THETAD:
            w = agc_erase_thetad(0);
            break;
        case AGC_DATA_THETAD_P1:
            w = agc_erase_thetad(1);
            break;
        case AGC_DATA_THETAD_P2:
            w = agc_erase_thetad(2);
            break;
        default:
            return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(w <= 077777u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *value = (agc_dp_t) w;

    return (AGC_STATUS_OK);
}

/* Raw 15-bit words, write side. */
static agc_status_t noun_raw_write(uint16_t data_id, agc_dp_t value)
{
    if (!AGC_ASSERT(value >= 0 && value <= 077777) ||
        !AGC_ASSERT(data_id < AGC_DATA_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const uint16_t w = (uint16_t) value;

    switch (data_id)
    {
        case AGC_DATA_CDUX:
            agc_erase_set_cdux(w);

            return (AGC_STATUS_OK);
        case AGC_DATA_CDUY:
            agc_erase_set_cduy(w);

            return (AGC_STATUS_OK);
        case AGC_DATA_CDUZ:
            agc_erase_set_cduz(w);

            return (AGC_STATUS_OK);
        case AGC_DATA_CDUS:
            agc_erase_set_cdus(w);

            return (AGC_STATUS_OK);
        case AGC_DATA_CDUT:
            agc_erase_set_cdut(w);

            return (AGC_STATUS_OK);
        case AGC_DATA_DAPDATR1:
            agc_erase_set_dapdatr1(w);

            return (AGC_STATUS_OK);
        case AGC_DATA_DAPDATR2:
            agc_erase_set_dapdatr2(w);

            return (AGC_STATUS_OK);
        case AGC_DATA_FAILREG:
            return (agc_erase_set_failreg(0, w));
        case AGC_DATA_FAILREG_P1:
            return (agc_erase_set_failreg(1, w));
        case AGC_DATA_FAILREG_P2:
            return (agc_erase_set_failreg(2, w));
        case AGC_DATA_THETAD:
            agc_erase_set_thetad(0, w);

            return (AGC_STATUS_OK);
        case AGC_DATA_THETAD_P1:
            agc_erase_set_thetad(1, w);

            return (AGC_STATUS_OK);
        case AGC_DATA_THETAD_P2:
            agc_erase_set_thetad(2, w);

            return (AGC_STATUS_OK);
        default:
            return (AGC_STATUS_BAD_ARG);
    }
}

/* The one data id -> erasable dispatch. Ids not listed are not yet exposed by core. */
static agc_status_t noun_data_access(uint16_t data_id, bool write,
                                     agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT) ||
        !AGC_ASSERT(data_id != AGC_DATA_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    switch (data_id)
    {
        case AGC_DATA_SPECIFY_ADDR:    /* nouns 01-03, 10, 15: the pinball */
        case AGC_DATA_SPECIFY_CHANNEL: /* module supplies the address      */
        case AGC_DATA_INCREMENT_ADDR:
            return (AGC_STATUS_NOT_FOUND);
        case AGC_DATA_CHANNEL33:
            return (noun_access_chan33(write, value));
        case AGC_DATA_TIME2:
        case AGC_DATA_TIG:
        case AGC_DATA_TTOGO:
        case AGC_DATA_TTPI:
            return (noun_access_time(data_id, write, value));
        case AGC_DATA_PIPAX:
        case AGC_DATA_PIPAY:
        case AGC_DATA_PIPAZ:
        case AGC_DATA_CSMMASS:
        case AGC_DATA_LEMMASS:
            return (noun_access_sp(data_id, write, value));
        case AGC_DATA_CDUX:
        case AGC_DATA_CDUY:
        case AGC_DATA_CDUZ:
        case AGC_DATA_CDUS:
        case AGC_DATA_CDUT:
        case AGC_DATA_DAPDATR1:
        case AGC_DATA_DAPDATR2:
        case AGC_DATA_FAILREG:
        case AGC_DATA_FAILREG_P1:
        case AGC_DATA_FAILREG_P2:
        case AGC_DATA_THETAD:
        case AGC_DATA_THETAD_P1:
        case AGC_DATA_THETAD_P2:
            return (write ? noun_raw_write(data_id, *value) :
                    noun_raw_read(data_id, value));
        default: /* not yet exposed by core          */
            return (AGC_STATUS_UNIMPLEMENTED);
    }
}

/* Read the erasable behind a data id. */
agc_status_t agc_noun_read(uint16_t data_id, agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (noun_data_access(data_id, false, value));
}

/* Store a value into the erasable behind a data id. */
agc_status_t agc_noun_write(uint16_t data_id, agc_dp_t value)
{
    agc_dp_t v = value;

    if (!AGC_ASSERT(data_id < AGC_DATA_COUNT) ||
        !AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (noun_data_access(data_id, true, &v));
}
