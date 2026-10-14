/**********************************************************************
 * FILE NAME: noun_data.c
 *
 * PURPOSE:
 *   The noun data of the rendezvous erasables, for agc_noun_read /
 *   agc_noun_write (io-nouns, src/io/dsky/nouns/data.c):
 *   agc_rendz_noun_read / agc_rendz_noun_write resolve the AGC_DATA_*
 *   ids of the erasables this lane owns.
 *
 * AGC SOURCE:
 *   Comanche055 / Luminary099 PINBALL_NOUN_TABLES.agc (the noun address
 *   tables that name these erasables) and ERASABLE_ASSIGNMENTS.agc.
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  ------------------------------
 *   agc_rendz_t fields    I/O  per field  DP and vector erasables
 *   RSTACK     agc_dp_t   I    radar      LM radar test samples
 *   LANDMARK HORIZON TANG word I/O -       raw words (rendezvous_erasables.c)
 *   PRAXIS CPHIX     word I    CDU        CM R63 attitudes (rendezvous_erasables.c)
 *   MRKBUF1 +3/+5    word I    CDU        mark buffer words
 *
 * EXTERNAL REFERENCES:
 *   Name                       Description
 *   -------------------------  -----------------------------------------
 *   agc_rendz_erasable         the rendezvous erasables (rendezvous_erasables.c)
 *   agc_erase_rstack/tang/...  storage accessors (rendezvous_erasables.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; ids not owned here return AGC_STATUS_NOT_FOUND.
 *
 * NOTES:
 *   Value representation (as src/io/dsky/nouns/data.c): DP erasables as
 *   the raw DP; SP erasables in the upper word (agc_dp_from_sp); raw
 *   words (octal, CDU angles) as the 15-bit word.
 **********************************************************************/
#include <stddef.h>

#include "agc/io/nouns.h"
#include "internal.h"

/**********************************************************************
 * The DP erasables of agc_rendz_t (and DSPTEM-free values).
 * AGC: PINBALL_NOUN_TABLES.agc (noun addresses of these erasables)
 *
 * FUNCTION NAME: nd_dp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   id        uint16_t  I    AGC_DATA_* id
 *
 * RETURNS: agc_dp_t * -- the agc_rendz_t DP field the id names, NULL when
 *          it is not one
 **********************************************************************/
static agc_dp_t *nd_dp(uint16_t id)
{
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    switch (id)
    {
        case AGC_DATA_ACTCENT:
            return (&rz->actcent);
        case AGC_DATA_CENTANG:
            return (&rz->centang);
        case AGC_DATA_DELHITE:
            return (&rz->delhite);
        case AGC_DATA_DELTAR:
            return (&rz->deltar);
        case AGC_DATA_DELVTPF:
            return (&rz->delvtpf);
        case AGC_DATA_DELVTPI:
            return (&rz->delvtpi);
        case AGC_DATA_DIFFALT:
            return (&rz->diffalt);
        case AGC_DATA_ELEV:
            return (&rz->elev);
        case AGC_DATA_HAPOX:
            return (&rz->hapox);
        case AGC_DATA_HPERX:
            return (&rz->hperx);
        case AGC_DATA_NEG_TPER:
            return (&rz->neg_tper);
        case AGC_DATA_PLUS_MGA:
            return (&rz->plus_mga);
        case AGC_DATA_POSTTPI:
            return (&rz->posttpi);
        case AGC_DATA_RANGE:
        case AGC_DATA_WWPOS:  /* WWPOS EQUALS RANGE   */
            return (&rz->range);
        case AGC_DATA_RRATE:
        case AGC_DATA_WWVEL:  /* WWVEL EQUALS RRATE   */
            return (&rz->rrate);
        case AGC_DATA_RTHETA:
        case AGC_DATA_WWBIAS: /* WWBIAS EQUALS RTHETA */
            return (&rz->rtheta);
        case AGC_DATA_T1TOT2:
            return (&rz->t1tot2);
        case AGC_DATA_T2TOT3:
            return (&rz->t2tot3);
        case AGC_DATA_TCDH:
            return (&rz->tcdh);
        case AGC_DATA_TCSI:
            return (&rz->tcsi);
        case AGC_DATA_TFF:
            return (&rz->tff);
        case AGC_DATA_THETZERO:
            return (&rz->thetzero);
        case AGC_DATA_GROUND_TRACK_DETERMINATION_ALT:
            return (&rz->ground_track_alt);
        case AGC_DATA_GROUND_TRACK_DETERMINATION_GAM:
            return (&rz->ground_track_gamma);
        case AGC_DATA_GROUND_TRACK_DETERMINATION_VEL:
            return (&rz->ground_track_vel);
        default:
            return (NULL);
    } /* switch (id) */
} /* nd_dp */

/**********************************************************************
 * Vector erasables: (id of the first component) -> vector, component.
 * AGC: PINBALL_NOUN_TABLES.agc (noun addresses DELVLVC, DELVOV, DVLOS)
 *
 * FUNCTION NAME: nd_vec
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   id        uint16_t  I    AGC_DATA_* id of a vector component
 *   k         int *     O    component index 0..2 (0 when the id is not a
 *                            vector)
 *
 * RETURNS: agc_vec_t * -- the agc_rendz_t vector the id names, NULL when it
 *          is not one
 **********************************************************************/
static agc_vec_t *nd_vec(uint16_t id, int *k)
{
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    *k = 0;

    switch (id)
    {
        case AGC_DATA_DELVLVC_P4:
            *k = 2;

            return (&rz->delvlvc);
        case AGC_DATA_DELVLVC_P2:
            *k = 1;

            return (&rz->delvlvc);
        case AGC_DATA_DELVLVC:
            return (&rz->delvlvc);
        case AGC_DATA_DELVOV_P4:
            *k = 2;

            return (&rz->delvov);
        case AGC_DATA_DELVOV_P2:
            *k = 1;

            return (&rz->delvov);
        case AGC_DATA_DELVOV:
            return (&rz->delvov);
        case AGC_DATA_DVLOS_P4:
            *k = 2;

            return (&rz->dvlos);
        case AGC_DATA_DVLOS_P2:
            *k = 1;

            return (&rz->dvlos);
        case AGC_DATA_DVLOS:
            return (&rz->dvlos);
        default:
            return (NULL);
    }
}

/**********************************************************************
 * Read-only values held by the radar, mark and display modules.
 * AGC: RADAR_LEADIN_ROUTINES.agc p.490 (RSTACK, LM R04 nouns)
 *
 * FUNCTION NAME: nd_other
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   id        uint16_t    I    AGC_DATA_* id
 *   v         agc_dp_t *  O    the value, DP as stored
 *
 * RETURNS: bool -- true when the id is one of these read-only values
 **********************************************************************/
static bool nd_other(uint16_t id, agc_dp_t *v)
{
    switch (id)
    {
        case AGC_DATA_RSTACK:
            *v = agc_erase_rstack(0);

            return (true);
        case AGC_DATA_RSTACK_P2:
            *v = agc_erase_rstack(1);

            return (true);
        case AGC_DATA_RSTACK_P4:
            *v = agc_erase_rstack(2);

            return (true);
        case AGC_DATA_RSTACK_P6:
            *v = agc_erase_rstack(3);

            return (true);
        case AGC_DATA_NN:
            *v = agc_dp_from_sp(agc_rendz_erasable()->nn);

            return (true);
        case AGC_DATA_NN1:
            *v = agc_dp_from_sp(agc_rendz_erasable()->nn1);

            return (true);
        case AGC_DATA_WWOPT: /* WWOPT EQUALS RTHETA (high word)       */
            *v = agc_dp_from_sp(agc_dp_high(agc_rendz_erasable()->rtheta));

            return (true);
        case AGC_DATA_TRKMKCNT:
            *v = agc_dp_from_sp(agc_erase_trkmkcnt());

            return (true);
        case AGC_DATA_VHFCNT:
            *v = agc_dp_pack(agc_erase_vhfcnt(), agc_erase_trkmkcnt());

            return (true);
        default:
            return (false);
    }
} /* nd_other */

/**********************************************************************
 * Raw words: LANDMARK, HORIZON, MRKBUF1 +3/+5, TANG, PRAXIS, CPHIX.
 * AGC: PINBALL_NOUN_TABLES.agc (nouns 70, 71, 72, 95, 96, MRKBUF1)
 *
 * FUNCTION NAME: nd_raw
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   id        uint16_t    I    AGC_DATA_* id
 *   w         uint16_t *  O    the 15-bit word
 *
 * RETURNS: bool -- true when the id is one of these raw words
 **********************************************************************/
static bool nd_raw(uint16_t id, uint16_t *w)
{
    switch (id)
    {
        case AGC_DATA_LANDMARK:
            *w = rz_word_of(agc_erase_landmark());

            return (true);
        case AGC_DATA_HORIZON:
            *w = agc_erase_horizon();

            return (true);
        case AGC_DATA_MRKBUF1_P3:
            *w = agc_erase_mrkbuf(1, 3);

            return (true);
        case AGC_DATA_MRKBUF1_P5:
            *w = agc_erase_mrkbuf(1, 5);

            return (true);
        case AGC_DATA_TANG:
            *w = rz_word_of(agc_erase_tang(0));

            return (true);
        case AGC_DATA_TANG_P1:
            *w = rz_word_of(agc_erase_tang(1));

            return (true);
        case AGC_DATA_PRAXIS:
            *w = agc_rendz_tracking_angles(true, 0);

            return (true);
        case AGC_DATA_PRAXIS_P1:
            *w = agc_rendz_tracking_angles(true, 1);

            return (true);
        case AGC_DATA_PRAXIS_P2:
            *w = agc_rendz_tracking_angles(true, 2);

            return (true);
        case AGC_DATA_CPHIX:
            *w = agc_rendz_tracking_angles(false, 0);

            return (true);
        case AGC_DATA_CPHIX_P1:
            *w = agc_rendz_tracking_angles(false, 1);

            return (true);
        case AGC_DATA_CPHIX_P2:
            *w = agc_rendz_tracking_angles(false, 2);

            return (true);
        default:
            return (false);
    } /* switch (id) */
} /* nd_raw */

/**********************************************************************
 * Noun read of a rendezvous erasable (agc_noun_read's lookup).
 * AGC: PINBALL_NOUN_TABLES.agc (noun address tables)
 *
 * FUNCTION NAME: agc_rendz_noun_read
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   data_id   uint16_t    I    AGC_DATA_* id, < AGC_DATA_COUNT
 *   value     agc_dp_t *  O    the value (DP raw, SP in the upper word,
 *                              words raw)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_NOT_FOUND for an id not
 *          owned here
 **********************************************************************/
agc_status_t agc_rendz_noun_read(uint16_t data_id, agc_dp_t *value)
{
    agc_dp_t  *p  = nd_dp(data_id);      /* DP field, or NULL */
    int        k  = 0;                   /* vector component */
    agc_vec_t *vp = nd_vec(data_id, &k); /* vector, or NULL */
    uint16_t   w  = 0U;                  /* raw word */

    if (p != NULL)
    {
        *value = *p;
    }
    else if (vp != NULL)
    {
        *value = vp->c[k];
    }
    else if (nd_raw(data_id, &w))
    {
        *value = (agc_dp_t) w;
    }
    else if (!nd_other(data_id, value))
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * Loadable values: the DP erasables and vectors, LANDMARK, TANG, WWOPT.
 * AGC: PINBALL_NOUN_TABLES.agc (loadable nouns)
 *
 * FUNCTION NAME: agc_rendz_noun_write
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   data_id   uint16_t  I    AGC_DATA_* id, < AGC_DATA_COUNT
 *   value     agc_dp_t  I    the loaded value, same representation as the
 *                            read
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_NOT_FOUND for an id not
 *          loadable here
 **********************************************************************/
agc_status_t agc_rendz_noun_write(uint16_t data_id, agc_dp_t value)
{
    agc_dp_t  *p  = nd_dp(data_id);      /* DP field, or NULL */
    int        k  = 0;                   /* vector component */
    agc_vec_t *vp = nd_vec(data_id, &k); /* vector, or NULL */

    if (p != NULL)
    {
        *p = value;
    }
    else if (vp != NULL)
    {
        vp->c[k] = value;
    }
    else if (data_id == AGC_DATA_WWOPT)
    {
        agc_rendz_erasable()->rtheta = agc_dp_from_sp(agc_dp_high(value));
    }
    else if (data_id == AGC_DATA_LANDMARK)
    {
        agc_erase_set_landmark((agc_sp_t) rz_word_value((uint16_t) (value & 077777)));
    }
    else if (data_id == AGC_DATA_HORIZON)
    {
        agc_erase_set_horizon((uint16_t) (value & 077777));
    }
    else if (data_id == AGC_DATA_TANG || data_id == AGC_DATA_TANG_P1)
    {
        agc_erase_set_tang((data_id == AGC_DATA_TANG) ? 0 : 1, (agc_sp_t) rz_word_value((uint16_t) (value & 077777))); /* TANG or TANG +1 */
    }
    else
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    return (AGC_STATUS_OK);
} /* agc_rendz_noun_write */
