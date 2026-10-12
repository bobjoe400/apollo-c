/**********************************************************************
 * FILE NAME: data.c
 *
 * PURPOSE:
 *   Noun table lookup (LODNNTAB) and the data id -> erasable resolution
 *   behind agc_noun_read() / agc_noun_write().
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_NOUN_TABLES.agc p. 270 (LODNNTAB; the NNADTAB and
 *   IDADDTAB ECADRs become data ids)
 *   Luminary099 PINBALL_NOUN_TABLES.agc p. 304 (identical)
 *
 * EXTERNAL VARIABLES:
 *   Name             Type      I/O  Scale        Description
 *   ---------------  --------  ---  -----------  ------------------------
 *   TIME2,TIME1      agc_dp_t  I/O  cs 2^28      clock (noun 36, 65)
 *   TIG, TTOGO, TTPI agc_dp_t  I/O  cs 2^28      signed DP times
 *   PIPAX..Z         agc_sp_t  I/O  pulses       noun 21
 *   CSMMASS, LEMMASS agc_sp_t  I/O  2^16 kg      noun 47
 *   CDUX..Z, CDUS/T  uint16_t  I/O  2^-15 rev    raw angle words
 *   DAPDATR1/2, FAILREG, THETAD  uint16_t  I/O   raw words
 *   SAMPTIME, T3TOT4 agc_dp_t  I/O  cs 2^28
 *   RR-AZ, RR-ELEV   agc_dp_t  I/O  rev
 *   OPTION1..3       agc_sp_t  I/O  octal        noun 06
 *   channel 33       uint16_t  I    bits         LM noun 66
 *
 * EXTERNAL REFERENCES:
 *   Name               Description
 *   -----------------  -----------------------------------------------
 *   agc_erase_*        core erasable accessors (state.h)
 *   agc_dispatch_noun  every other id (src/exec/nouns.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   NOT_FOUND for the pinball-resolved ids (nouns 01-03, 10, 15) and ids
 *   no layer holds; BAD_ARG on a failed precondition.
 *
 * NOTES:
 *   One dispatch switch (noun_data_access); per-family helpers hold the
 *   accessor calls only to keep every function within the 60-line rule.
 *   Value convention (nouns.h): an SP erasable travels in the high word of
 *   the agc_dp_t, a DP erasable as the agc_dp_t itself, a raw word as the
 *   unsigned 15-bit integer.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/nouns.h"
#include "agc/io/service.h"

/**********************************************************************
 * LODNNTAB (PINBALL_NOUN_TABLES.agc p. 270): row for one noun of one vehicle.
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  -------------------
 *   veh       agc_vehicle_t  I    vehicle whose tables are used (CSM or
 *                                 LM)
 *   noun      uint8_t        I    noun number 00..99 (NOUNREG)
 *   out       agc_noun_t *   O    the noun table row
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND
 **********************************************************************/
agc_status_t agc_noun_lookup_vehicle(agc_vehicle_t veh, uint8_t noun, agc_noun_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(noun < AGC_NOUN_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_noun_t *row = (veh == AGC_VEHICLE_CSM) ? &agc_noun_table_csm[noun] : &agc_noun_table_lm[noun]; /* vehicle; the vehicle's table row */

    if (!AGC_ASSERT(row->noun == noun) || !AGC_ASSERT(row->ncomp <= AGC_NOUN_MAX_COMP))
    {
        return (AGC_STATUS_BAD_ARG); /* table corrupted */
    }

    *out = *row;

    return ((row->ncomp == 0) ? AGC_STATUS_NOT_FOUND : AGC_STATUS_OK); /* spare noun */
}

/**********************************************************************
 * AGC: LODNNTAB (PINBALL_NOUN_TABLES.agc, CM p. 270)
 *
 * Row for a noun of the vehicle selected by agc_state_init().
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------
 *   noun      uint8_t       I    noun number 00..99 (NOUNREG)
 *   out       agc_noun_t *  O    the noun table row
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_noun_lookup(uint8_t noun, agc_noun_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(noun < AGC_NOUN_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_noun_lookup_vehicle(agc_state_vehicle(), noun, out));
}

/**********************************************************************
 * AGC: none (data layer: TIME2, TIG, TTOGO, TTPI)
 *
 * DP times in centiseconds, scale 2^28: TIME2, TIG, TTOGO, TTPI. TIME2 is
 * the clock (0 .. modulus); TIG, TTOGO and TTPI are signed DP erasables
 * (TTOGO = TIME2 - TIG is negative before ignition, shown "-" by M/SOUT).
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   data_id   uint16_t    I    agc_noun_data_t id of the erasable
 *                              (nouns.h)
 *   write     bool        I    true to store *value, false to read into
 *                              it
 *   value     agc_dp_t *  I/O  value read or written (nouns.h value
 *                              convention)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t noun_access_time(uint16_t data_id, bool write, agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT) || !AGC_ASSERT(!write || data_id != AGC_DATA_TIME2 || (*value >= 0 && *value < (agc_dp_t) AGC_TIME_MODULUS_CS)))
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
    } /* end switch (data_id) */
} /* noun_access_time */

/**********************************************************************
 * AGC: none (data layer)
 *
 * SP words (ones' complement, SP in the high word): PIPAs, masses.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   data_id   uint16_t    I    agc_noun_data_t id of the erasable
 *                              (nouns.h)
 *   write     bool        I    true to store *value, false to read into
 *                              it
 *   value     agc_dp_t *  I/O  value read or written (nouns.h value
 *                              convention)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t noun_access_sp(uint16_t data_id, bool write, agc_dp_t *value)
{
    const agc_sp_t w = write ? agc_dp_high(*value) : 0; /* SP scale */
    agc_sp_t       r = 0;                               /* SP word read, SP scale */

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
    } /* end switch (data_id) */

    if (!write)
    {
        *value = agc_dp_from_sp(r);
    }

    return (AGC_STATUS_OK);
} /* noun_access_sp */

/**********************************************************************
 * AGC: none (data layer)
 *
 * Raw 15-bit words, read side: CDUs, DAP words, alarm codes, THETAD.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   data_id   uint16_t    I    agc_noun_data_t id of the erasable
 *                              (nouns.h)
 *   value     agc_dp_t *  I/O  value read or written (nouns.h value
 *                              convention)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t noun_raw_read(uint16_t data_id, agc_dp_t *value)
{
    uint16_t w = 0; /* word to store, unscaled */

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
    } /* end switch (data_id) */

    *value = (agc_dp_t) w;

    return (AGC_STATUS_OK);
} /* noun_raw_read */

/**********************************************************************
 * AGC: none (data layer)
 *
 * Raw 15-bit words, write side.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   data_id   uint16_t  I    agc_noun_data_t id of the erasable (nouns.h)
 *   value     agc_dp_t  I    value to store (nouns.h value convention)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t noun_raw_write(uint16_t data_id, agc_dp_t value)
{
    if (!AGC_ASSERT(value >= 0 && value <= 077777) || !AGC_ASSERT(data_id < AGC_DATA_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const uint16_t w = (uint16_t) value; /* word to store, unscaled */

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
            agc_erase_set_failreg(0, w);

            return (AGC_STATUS_OK);
        case AGC_DATA_FAILREG_P1:
            agc_erase_set_failreg(1, w);

            return (AGC_STATUS_OK);
        case AGC_DATA_FAILREG_P2:
            agc_erase_set_failreg(2, w);

            return (AGC_STATUS_OK);
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
    } /* end switch (data_id) */
} /* noun_raw_write */

/**********************************************************************
 * AGC: none (data layer)
 *
 * Core DP erasables kept outside erasable.c: SAMPTIME, T3TOT4 (cs 2^28),
 * RR-AZ, RR-ELEV (revs); SMODE (self-check mode, raw word).
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   id        uint16_t    I    agc_noun_data_t id
 *   write     bool        I    true to store *value, false to read into
 *                              it
 *   v         agc_dp_t *  I/O  value read or written
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND; else the status of the routine it calls
 **********************************************************************/
static agc_status_t noun_access_core2(uint16_t id, bool write, agc_dp_t *v)
{
    switch (id)
    {
        case AGC_DATA_SAMPTIME:

            if (write)
            {
                agc_erase_set_samptime(*v);
            }

            *v = agc_erase_samptime();
            break;
        case AGC_DATA_T3TOT4:

            if (write)
            {
                agc_erase_set_t3tot4(*v);
            }

            *v = agc_erase_t3tot4();
            break;
        case AGC_DATA_RR_AZ:

            if (write)
            {
                agc_erase_set_rr_az(*v);
            }

            *v = agc_erase_rr_az();
            break;
        case AGC_DATA_RR_ELEV:

            if (write)
            {
                agc_erase_set_rr_elev(*v);
            }

            *v = agc_erase_rr_elev();
            break;
        case AGC_DATA_SMODE:

            if (write)
            {
                return (agc_selfcheck_set_smode((uint16_t) (*v & 077777)));
            }

            *v = (agc_dp_t) agc_selfcheck_smode();
            break;
        default:
            return (AGC_STATUS_NOT_FOUND);
    } /* end switch (id) */

    return (AGC_STATUS_OK);
} /* noun_access_core2 */

/**********************************************************************
 * AGC: none (data layer: NNADTAB / IDADDTAB ECADRs)
 *
 * The one data id -> erasable dispatch: core erasables here, the rest
 * through agc_dispatch_noun (src/exec/nouns.c).
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   data_id   uint16_t    I    agc_noun_data_t id of the erasable
 *                              (nouns.h)
 *   write     bool        I    true to store *value, false to read into
 *                              it
 *   value     agc_dp_t *  I/O  value read or written (nouns.h value
 *                              convention)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND; else the status of the routine it calls
 **********************************************************************/
static agc_status_t noun_data_access(uint16_t data_id, bool write, agc_dp_t *value)
{
    if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT) || !AGC_ASSERT(data_id != AGC_DATA_NONE))
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
            /* Channel 33 (LM noun 66 component 2, scale TT): input only, no
             * load. The 15-bit image is passed raw (the LR position routine
             * tests bits 6 and 7). */
        {
            if (!AGC_ASSERT(value != NULL) || !AGC_ASSERT(!write))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            *value = (agc_dp_t) (agc_chan_read((int) AGC_CHAN_CHAN33) & 077777u);

            return (AGC_STATUS_OK);
        }
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
            return (write ? noun_raw_write(data_id, *value) : noun_raw_read(data_id, value)); /* store / fetch */
        case AGC_DATA_OPTION1:
        case AGC_DATA_OPTION2:
        case AGC_DATA_OPTION3:
            /* OPTION1-3 (noun 06, raw octal words; OPTION1 +k). */
        {
            int k = (int) data_id - (int) AGC_DATA_OPTION1; /* component */

            if (write)
            {
                agc_erase_set_option1(k, (agc_sp_t) (*value & 037777));
            }

            *value = (agc_dp_t) ((uint16_t) agc_erase_option1(k) & 077777u);

            return (AGC_STATUS_OK);
        }
        case AGC_DATA_SAMPTIME:
        case AGC_DATA_T3TOT4:
        case AGC_DATA_RR_AZ:
        case AGC_DATA_RR_ELEV:
        case AGC_DATA_SMODE:
            return (noun_access_core2(data_id, write, value));
        default: /* held by a higher layer               */
            return (agc_dispatch_noun(data_id, write, value));
    } /* end switch (data_id) */
} /* noun_data_access */

/**********************************************************************
 * AGC: none (data layer)
 *
 * Read the erasable behind a data id.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   data_id   uint16_t    I    agc_noun_data_t id of the erasable
 *                              (nouns.h)
 *   value     agc_dp_t *  I/O  value read or written (nouns.h value
 *                              convention)
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_noun_read(uint16_t data_id, agc_dp_t *value)
{
    return (noun_data_access(data_id, false, value));
}

/**********************************************************************
 * AGC: none (data layer)
 *
 * Store a value into the erasable behind a data id.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   data_id   uint16_t  I    agc_noun_data_t id of the erasable (nouns.h)
 *   value     agc_dp_t  I    value to store (nouns.h value convention)
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_noun_write(uint16_t data_id, agc_dp_t value)
{
    agc_dp_t v = value; /* value, scale of the id */

    if (!AGC_ASSERT(data_id < AGC_DATA_COUNT) || !AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (noun_data_access(data_id, true, &v));
}
