/**********************************************************************
 * FILE NAME: nouns.c
 *
 * PURPOSE:
 *   Noun data owned above the io layer: the erasables the noun tables
 *   name that live in the guidance, program and vehicle modules' state
 *   blocks.
 *
 * AGC SOURCE:
 *   Comanche055 / Luminary099 PINBALL_NOUN_TABLES.agc (the erasable
 *   addresses of the noun tables); ERASABLE_ASSIGNMENTS.agc (equivalences,
 *   e.g. LM RDOT = HDOTDISP)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   noun erasables  various I/O  per noun       through the owners' state blocks
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_guid_state, agc_entry_state, agc_lm_ldg, agc_lm_asc, agc_align_ram
 *                         owners' state blocks
 *   agc_navaux_get_latlong / set   LAT, LONG, ALT (noun 43)
 *   agc_*_rendz_noun_*, agc_return_to_earth_noun   owners' noun accessors
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   NOT_FOUND for an id no owner holds; the owners' statuses; BAD_ARG
 *   for an id out of range.
 *
 * NOTES:
 *   The noun tables (PINBALL_NOUN_TABLES.agc) name erasables by address; in
 *   the AGC every erasable is reachable from pinball. In C most of them are
 *   fields of the state blocks of the guidance, program and vehicle modules.
 *   agc_noun_read / agc_noun_write (src/io/dsky/nouns/data.c) resolve the
 *   core erasables themselves and hand every other data id here.
 *
 *   LAYERING: like dispatch.c, this file is the documented place where
 *   src/exec/ reaches the higher layers (here: their public state blocks
 *   and noun accessors), so that io never includes a higher header.
 *
 *   Value convention (include/agc/io/nouns.h): DP erasables as the DP, SP
 *   erasables in the high word (agc_dp_from_sp), raw words (octal, CDU
 *   angles) as the 15-bit word. A vector component is one DP.
 *
 *   Noun 08 (ALMCADR +0..+2): R1 is the job that raised the last alarm
 *   (the AGC shows its return address), R2 +0 (the AGC shows the bank;
 *   DEVIATION: one flat address space), R3 ERCOUNT.  LM AOTCODE +1/+2 (N70/N71 R2, R3) are
 *   SINCDUY (docs/modules/review-prog-align.md).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/nouns.h"
#include "agc/io/service.h"
#include "agc/lm/landing.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

/**********************************************************************
 * AGC: none -- guidance (SERVICER, P30-P47) DP erasables named by the
 * noun tables (PINBALL_NOUN_TABLES.agc).
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *
 * RETURNS: agc_dp_t * -- the erasable, or NULL when this owner does not hold the id
 **********************************************************************/
static agc_dp_t *noun_dp_guidance(uint16_t id)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    switch (id)
    {
        case AGC_DATA_HAPO:
            return (&g->burn.hapo);
        case AGC_DATA_HPER:
            return (&g->burn.hper);
        case AGC_DATA_VGDISP:
            return (&g->burn.vgdisp);
        case AGC_DATA_DVTOTAL:
            return (&g->avg.dvtotal);
        case AGC_DATA_ABVEL:
            return (&g->lm.abvel);
        case AGC_DATA_HCALC:
            return (&g->lm.hcalc);
        case AGC_DATA_HDOTDISP:
            return (&g->lm.hdotdisp);
        case AGC_DATA_RSP_RREC:
            return (&g->csm.rsp_rrec);
        case AGC_DATA_DELVIMU:
            return (&g->burn.delvimu.c[0]);
        case AGC_DATA_DELVIMU_P2:
            return (&g->burn.delvimu.c[1]);
        case AGC_DATA_DELVIMU_P4:
            return (&g->burn.delvimu.c[2]);
        case AGC_DATA_VGBODY:
            return (&g->burn.vgbody.c[0]);
        case AGC_DATA_VGBODY_P2:
            return (&g->burn.vgbody.c[1]);
        case AGC_DATA_VGBODY_P4:
            return (&g->burn.vgbody.c[2]);
        default:
            return (NULL);
    }
}

/**********************************************************************
 * AGC: none -- CSM entry (P61-P67) DP erasables named by the noun
 * tables (PINBALL_NOUN_TABLES.agc).
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *
 * RETURNS: agc_dp_t * -- the erasable, or NULL when this owner does not hold the id
 **********************************************************************/
static agc_dp_t *noun_dp_entry(uint16_t id)
{
    agc_entry_state_t *e = agc_entry_state(); /* entry erasables */

    switch (id)
    {
        case AGC_DATA_D:
            return (&e->d);
        case AGC_DATA_DNRNGERR:
            return (&e->dnrngerr);
        case AGC_DATA_GAMMAEI:
            return (&e->gammaei);
        case AGC_DATA_GMAX:
            return (&e->gmax);
        case AGC_DATA_LAT_SPL:
            return (&e->lat_spl);
        case AGC_DATA_LNG_SPL:
            return (&e->lng_spl);
        case AGC_DATA_Q7:
            return (&e->q7);
        case AGC_DATA_ROLLC:
            return (&e->rollc);
        case AGC_DATA_RTGO:
            return (&e->rtgo);
        case AGC_DATA_RANGE_TO_TARGET:
            return (&e->range_to_target);
        case AGC_DATA_RANGE_TO_SPLASH: /* RTGON64 EQUALS RTGO          */
            return (&e->rtgo);
        case AGC_DATA_XRNGERR:         /* XRNGERR = LATANG                     */
            return (&e->latang);
        case AGC_DATA_TTE:
            return (&e->tte);
        case AGC_DATA_VIO:
            return (&e->vio);
        case AGC_DATA_VL:
            return (&e->vl);
        case AGC_DATA_VMAGI:
            return (&e->vmagi);
        case AGC_DATA_VPRED:
            return (&e->vpred);
        case AGC_DATA_RDOT:
            return (&e->rdot);
        default:
            return (NULL);
    }
} /* noun_dp_entry */

/**********************************************************************
 * AGC: none -- LM landing (P63-P68) and ascent (P12, P70-P71) DP
 * erasables named by the noun tables (PINBALL_NOUN_TABLES.agc).
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *
 * RETURNS: agc_dp_t * -- the erasable, or NULL when this owner does not hold the id
 **********************************************************************/
static agc_dp_t *noun_dp_landing(uint16_t id)
{
    agc_lm_ldg_t *l = agc_lm_ldg(); /* landing erasables */
    agc_lm_asc_t *a = agc_lm_asc(); /* ascent erasables */

    switch (id)
    {
        case AGC_DATA_DELTAH:
            return (&l->deltah);
        case AGC_DATA_HCALC1:
            return (&l->hcalc1);
        case AGC_DATA_OUTOFPLN:
            return (&l->outofpln);
        case AGC_DATA_RANGEDSP:
            return (&l->rangedsp);
        case AGC_DATA_TTFDISP:
            return (&l->ttfdisp);
        case AGC_DATA_VHORIZ:
            return (&l->vhoriz);
        case AGC_DATA_RDOTD:
            return (&a->rdotd);
        case AGC_DATA_ZDOTD:
            return (&a->zdotd);
        case AGC_DATA_YDOT:
            return (&a->ydot);
        case AGC_DATA_RDOT:
            return (&agc_guid_state()->lm.hdotdisp); /* RDOT EQUALS HDOTDISP (LM ERASABLE_ASSIGNMENTS) */
        case AGC_DATA_XRANGE:
            return (&a->xrange);
        case AGC_DATA_YAW:
            return (&a->yaw);
        case AGC_DATA_PITCH:
            return (&a->pitch);
        default:
            return (NULL);
    }
}

/**********************************************************************
 * AGC: none -- alignment (P51-P57, R52-R56) DP erasables and vector
 * components named by the noun tables (PINBALL_NOUN_TABLES.agc).
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *
 * RETURNS: agc_dp_t * -- the erasable, or NULL when this owner does not hold the id
 **********************************************************************/
static agc_dp_t *noun_dp_alignment(uint16_t id)
{
    agc_align_ram_t *r = agc_align_ram(); /* alignment erasables */

    switch (id)
    {
        case AGC_DATA_OGC:
            return (&r->ogc[0]);
        case AGC_DATA_OGC_P2:
            return (&r->ogc[1]);
        case AGC_DATA_OGC_P4:
            return (&r->ogc[2]);
        case AGC_DATA_LANDLAT:
            return (&r->landlat);
        case AGC_DATA_LANDLONG:
            return (&r->landlong);
        case AGC_DATA_LANDALT:
            return (&r->landalt);
        case AGC_DATA_STARAD:
            return (&r->starad[0].c[0]);
        case AGC_DATA_STARAD_P2:
            return (&r->starad[0].c[1]);
        case AGC_DATA_STARAD_P4:
            return (&r->starad[0].c[2]);
        case AGC_DATA_STARSAV3:
            return (&r->starsav3.c[0]);
        case AGC_DATA_STARSAV3_P2:
            return (&r->starsav3.c[1]);
        case AGC_DATA_STARSAV3_P4:
            return (&r->starsav3.c[2]);
        default:
            return (NULL);
    }
}

/**********************************************************************
 * AGC: none -- the DP erasable behind an id, by owner and vehicle.
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *   lm -- true for the Luminary rope
 *
 * RETURNS: agc_dp_t * -- the erasable, or NULL when this owner does not hold the id
 **********************************************************************/
static agc_dp_t *noun_dp(uint16_t id, bool lm)
{
    agc_dp_t *p = noun_dp_guidance(id); /* DP erasable, NULL none */

    if (p == NULL)
    {
        p = lm ? noun_dp_landing(id) : noun_dp_entry(id); /* LM landing : CSM entry */
    }

    if (p == NULL)
    {
        p = noun_dp_alignment(id);
    }

    return (p);
}

/**********************************************************************
 * AGC: none -- SP erasables (value in the high word).
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *
 * RETURNS: agc_sp_t * -- the erasable, or NULL when none holds the id
 **********************************************************************/
static agc_sp_t *noun_sp(uint16_t id)
{
    switch (id)
    {
        case AGC_DATA_HEADSUP:
            return (&agc_entry_state()->headsup);
        case AGC_DATA_PITTIME:
            return (&agc_guid_state()->lm.pittime);
        case AGC_DATA_ROLLTIME:
            return (&agc_guid_state()->lm.rolltime);
        case AGC_DATA_PACTOFF:
            return (&agc_csm_dap_com()->pactoff);
        case AGC_DATA_YACTOFF:
            return (&agc_csm_dap_com()->yactoff);
        case AGC_DATA_POSCODE:
            return (&agc_align_ram()->poscode);
        default:
            return (NULL);
    }
}

/**********************************************************************
 * AGC: none -- raw 15-bit words (CDU angles, octal codes) held as
 * uint16_t.
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *
 * RETURNS: uint16_t * -- the word, or NULL when none holds the id
 **********************************************************************/
static uint16_t *noun_word(uint16_t id)
{
    agc_align_ram_t *r = agc_align_ram(); /* alignment erasables */

    switch (id)
    {
        case AGC_DATA_AZ:
            return (&r->az);
        case AGC_DATA_EL:
            return (&r->el);
        case AGC_DATA_CURSOR:
            return (&r->cursor);
        case AGC_DATA_SPIRAL:
            return (&r->spiral);
        case AGC_DATA_SAC:
            return (&r->sac);
        case AGC_DATA_PAC:
            return (&r->pac);
        default:
            return (NULL);
    }
}

/**********************************************************************
 * AGC: none -- Two-integer words (scale TWO_INT: high word XX, low word YY). Nouns 45
 * (CM) and 64 (LM) are "no load" (SFINTABR: 2INTOUT CANT BE LOADED), so
 * only the sim and tests store them: VHFCNT,+1 (TRKMKCNT) and FUNNYDSP =
 * TREDES, LOOKANGL.
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *   v -- the value, at the noun's scale
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
static agc_status_t noun_two_int_write(uint16_t id, agc_dp_t v)
{
    if (id == AGC_DATA_VHFCNT)
    {
        agc_erase_set_vhfcnt(agc_dp_high(v));
        agc_erase_set_trkmkcnt(agc_dp_low(v));
    }
    else
    {
        agc_lm_ldg()->tredes   = agc_dp_high(v);
        agc_lm_ldg()->lookangl = agc_dp_low(v);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- read-only values computed by functions of their owners.
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *   lm -- true for the Luminary rope
 *   v -- the value, at the noun's scale
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; NOT_FOUND for any other id
 **********************************************************************/
static agc_status_t noun_derived(uint16_t id, bool lm, agc_dp_t *v)
{
    switch (id)
    {
        case AGC_DATA_FDAIX:
        case AGC_DATA_FDAIY:
        case AGC_DATA_FDAIZ: /* BALLANGS output (LM N18)            */
            *v = (agc_dp_t) agc_erase_fdai((int) (id - AGC_DATA_FDAIX));

            return (AGC_STATUS_OK);
        case AGC_DATA_ALPHASB:                                  /* LM N51: PITCHANG; CM N51: RHOSB      */
        case AGC_DATA_RHOSB:
            *v = lm ? agc_sband_pitchang() : agc_sband_rhosb(); /* LM PITCHANG : CM RHOSB */

            return (AGC_STATUS_OK);
        case AGC_DATA_BETASB:                                   /* LM N51: YAWANG; CM N51: GAMMASB      */
        case AGC_DATA_GAMMASB:
            *v = lm ? agc_sband_yawang() : agc_sband_gammasb(); /* LM YAWANG : CM GAMMASB */

            return (AGC_STATUS_OK);
        case AGC_DATA_TET: /* time of the state being integrated    */
            *v = agc_integ_shared_ctx()->sv.tet;

            return (AGC_STATUS_OK);
        case AGC_DATA_FUNNYDSP: /* LM N64: TREDES, LOOKANGL (DELTAH +2) */
            *v = agc_dp_pack(agc_lm_ldg()->tredes, agc_lm_ldg()->lookangl);

            return (AGC_STATUS_OK);
        case AGC_DATA_ALMCADR: /* N08 R1: the job that raised the alarm   */
            *v = (agc_dp_t) agc_alarm_almcadr();

            return (AGC_STATUS_OK);
        case AGC_DATA_ALMCADR_P1: /* N08 R2: bank of the alarm (DEVIATION: +0, one flat address space) */
            *v = 0;

            return (AGC_STATUS_OK);
        case AGC_DATA_ALMCADR_P2: /* N08 R3: ERCOUNT                        */
            *v = (agc_dp_t) agc_selfcheck_error_count();

            return (AGC_STATUS_OK);
        default:
            return (AGC_STATUS_NOT_FOUND);
    }
}

/**********************************************************************
 * AGC: none -- a signed SP word as the raw 15-bit ones'-complement
 * word (SP raw).
 *
 * ARGUMENTS:
 *   w -- the SP word
 *
 * RETURNS: agc_dp_t -- the raw word, 0..077777
 **********************************************************************/
static agc_dp_t noun_raw_word(agc_sp_t w)
{
    return ((agc_dp_t) ((w >= 0) ? w : (077777 + w))); /* negative: 077777 + w (ones' complement) */
}

/**********************************************************************
 * AGC: none -- the raw 15-bit word back as a signed SP (07777x
 * negative).
 *
 * ARGUMENTS:
 *   raw -- the raw 15-bit word (bits above 15 ignored)
 *
 * RETURNS: agc_sp_t -- the signed SP word
 **********************************************************************/
static agc_sp_t noun_signed_word(agc_dp_t raw)
{
    const agc_sp_t w = (agc_sp_t) (raw & 077777); /* raw word, unsigned, B0 */

    return ((w > AGC_SP_MAX) ? (agc_sp_t) (w - 077777) : w); /* 07777x: negative */
}

/**********************************************************************
 * STARCODE (CSM N70/N71 R1), AOTCODE = STARCODE (LM N70/N71 R1), and the
 * LM's AOTCODE +1/+2 (R2, R3).  Luminary ERASABLE_ASSIGNMENTS p. 37:
 * AOTCODE = STARCODE, and STARALGN follows it, so AOTCODE +1/+2 are the
 * two words of SINCDU = SINCDUY (sin(CDUY)/2, CDUTRIG scratch); the C home
 * of SINCDU is the guidance block.  NOT_FOUND for any other id.
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *   write -- true: store *value
 *   value -- the value, at the noun's scale (read: receives it)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; NOT_FOUND for any other id
 **********************************************************************/
static agc_status_t noun_star_code(uint16_t id, bool write, agc_dp_t *value)
{
    agc_sp_t *code  = &agc_align_ram()->starcode;                 /* STARCODE/AOTCODE */
    agc_dp_t *sin_y = &agc_guid_state()->trig.sincdu[AGC_TRIG_Y]; /* SINCDUY, 2^1 */

    switch (id)
    {
        case AGC_DATA_STARCODE:
        case AGC_DATA_AOTCODE:
            *code  = write ? noun_signed_word(*value) : *code; /* store or keep */
            *value = noun_raw_word(*code);

            return (AGC_STATUS_OK);
        case AGC_DATA_AOTCODE_P1:                                                                /* AOTCODE +1: SINCDUY high word     */
            *sin_y = write ? agc_dp_pack(noun_signed_word(*value), agc_dp_low(*sin_y)) : *sin_y; /* store R2 or keep */
            *value = noun_raw_word(agc_dp_high(*sin_y));

            return (AGC_STATUS_OK);
        case AGC_DATA_AOTCODE_P2:                                                                 /* AOTCODE +2: SINCDUY low word      */
            *sin_y = write ? agc_dp_pack(agc_dp_high(*sin_y), noun_signed_word(*value)) : *sin_y; /* store R3 or keep */
            *value = noun_raw_word(agc_dp_low(*sin_y));

            return (AGC_STATUS_OK);
        default:
            return (AGC_STATUS_NOT_FOUND);
    }
}

/**********************************************************************
 * AGC: none -- VMAGI, HDOT, ALTI of the P11 monitor (N62, S11.1 "CALC.
 * VI, H, HDOT FOR NOUN 62"), computed by its owner on every read.
 *
 * ARGUMENTS:
 *   id -- AGC_DATA_VMAGI, AGC_DATA_HDOT or AGC_DATA_ALTI
 *   value -- receives the value, at the noun's scale
 *
 * RETURNS: agc_status_t -- the readout's status
 **********************************************************************/
static agc_status_t noun_p11_readout(uint16_t id, agc_dp_t *value)
{
    agc_dp_t     vmagi = 0;                                                               /* VMAGI, 2^7 m/cs */
    agc_dp_t     hdot  = 0;                                                               /* HDOT, 2^7 m/cs  */
    agc_dp_t     alti  = 0;                                                               /* ALTI, 2^29 m    */
    agc_status_t st    = agc_earth_orbit_insertion_monitor_readout(&vmagi, &hdot, &alti); /* readout outcome */

    if (st == AGC_STATUS_OK)
    {
        *value = vmagi;

        if (id == AGC_DATA_HDOT)
        {
            *value = hdot;
        }
        else if (id == AGC_DATA_ALTI)
        {
            *value = alti;
        }
        else
        {
            /* VMAGI */
        }
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- LAT, LONG, ALT (noun 43): the nav-aux store.
 *
 * ARGUMENTS:
 *   id -- AGC_DATA_LAT, AGC_DATA_LONG or AGC_DATA_ALT
 *   write -- true: store *value
 *   value -- the value, at the noun's scale (read: receives it)
 *
 * RETURNS: agc_status_t -- the nav-aux store's status
 **********************************************************************/
static agc_status_t noun_latlong(uint16_t id, bool write, agc_dp_t *value)
{
    agc_latlong_t ll;                               /* LAT, LONG, ALT */
    agc_status_t  st = agc_navaux_get_latlong(&ll); /* store outcome */
    agc_dp_t     *f  = &ll.alt;                     /* the component named by id */

    if (id == AGC_DATA_LAT)
    {
        f = &ll.lat;
    }
    else if (id == AGC_DATA_LONG)
    {
        f = &ll.lon;
    }
    else
    {
        /* ALT */
    }

    if (st == AGC_STATUS_OK && write)
    {
        *f = *value;
        st = agc_navaux_set_latlong(&ll);
    }
    else if (st == AGC_STATUS_OK)
    {
        *value = *f;
    }
    else
    {
        /* the store's failure is returned */
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- the ids resolved through functions of their owners: the
 * P11 readout, LAT/LONG/ALT, STARCODE/AOTCODE, P37, the rendezvous
 * programs, the two-integer words and the derived values.
 *
 * ARGUMENTS:
 *   id -- noun data id (AGC_DATA_*)
 *   lm -- true for the Luminary rope
 *   write -- true: store *value
 *   value -- the value, at the noun's scale (read: receives it)
 *
 * RETURNS: agc_status_t -- the owner's status; NOT_FOUND when no owner
 *          holds the id
 **********************************************************************/
static agc_status_t noun_by_function(uint16_t id, bool lm, bool write, agc_dp_t *value)
{
    bool         p11 = !lm && agc_erase_modreg() == 11; /* P11 monitor running */
    agc_status_t st  = AGC_STATUS_NOT_FOUND;            /* owner outcome */

    if ((id == AGC_DATA_HDOT || id == AGC_DATA_ALTI || (id == AGC_DATA_VMAGI && p11)) && !write)
    {
        st = noun_p11_readout(id, value);
    }
    else if (id == AGC_DATA_LAT || id == AGC_DATA_LONG || id == AGC_DATA_ALT)
    {
        st = noun_latlong(id, write, value);
    }
    else
    {
        st = noun_star_code(id, write, value);

        if (st == AGC_STATUS_NOT_FOUND && !lm)
        {
            st = agc_return_to_earth_noun(id, write, value);
        }

        if (st == AGC_STATUS_NOT_FOUND && write)
        {
            st = agc_rendz_noun_write(id, *value);
        }
        else if (st == AGC_STATUS_NOT_FOUND)
        {
            st = agc_rendz_noun_read(id, value);
        }
        else
        {
            /* found above */
        }

        if (st == AGC_STATUS_NOT_FOUND && !write)
        {
            st = lm ? agc_lm_rendz_noun_read(id, value) : agc_csm_rendz_noun_read(id, value); /* LM : CSM */
        }

        if (st == AGC_STATUS_NOT_FOUND && write && (id == AGC_DATA_VHFCNT || id == AGC_DATA_FUNNYDSP))
        {
            st = noun_two_int_write(id, *value);
        }

        if (st == AGC_STATUS_NOT_FOUND && !write)
        {
            st = noun_derived(id, lm, value);
        }
    }

    return (st);
} /* noun_by_function */

/**********************************************************************
 * AGC: none -- read (write = false) or store the erasable behind a data
 * id that core does not hold (the noun tables' erasable addresses,
 * PINBALL_NOUN_TABLES.agc).
 *
 * ARGUMENTS:
 *   data_id -- noun data id (AGC_DATA_*)
 *   write -- true: store *value
 *   value -- the value, at the noun's scale (read: receives it)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; NOT_FOUND when no owner holds
 *          the id; the owner's status; BAD_ARG for an id out of range
 **********************************************************************/
agc_status_t agc_dispatch_noun(uint16_t data_id, bool write, agc_dp_t *value)
{
    if (!AGC_ASSERT(data_id > AGC_DATA_NONE && data_id < AGC_DATA_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool         lm = agc_state_vehicle() == AGC_VEHICLE_LM; /* Luminary rope */
    agc_dp_t    *dp = NULL;                                  /* DP erasable, NULL none */
    agc_sp_t    *sp = noun_sp(data_id);                      /* SP erasable, NULL none */
    uint16_t    *w  = noun_word(data_id);                    /* raw word, NULL none */
    agc_status_t st = AGC_STATUS_OK;                         /* outcome */

    if (data_id != AGC_DATA_VMAGI || lm || agc_erase_modreg() != 11)
    {
        dp = noun_dp(data_id, lm); /* VMAGI in P11 is the readout's */
    }

    if (dp != NULL)
    {
        *dp    = write ? *value : *dp; /* store or keep */
        *value = *dp;
    }
    else if (sp != NULL)
    {
        *sp    = write ? agc_dp_high(*value) : *sp; /* store or keep */
        *value = agc_dp_from_sp(*sp);
    }
    else if (w != NULL)
    {
        *w     = write ? (uint16_t) (*value & 077777) : *w; /* store or keep */
        *value = (agc_dp_t) *w;
    }
    else
    {
        st = noun_by_function(data_id, lm, write, value);
    }

    return (st);
} /* agc_dispatch_noun */
