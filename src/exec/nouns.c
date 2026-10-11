/*
 * nouns.c -- noun data owned above the io layer.
 *
 * The noun tables (PINBALL_NOUN_TABLES.agc) name erasables by address; in
 * the AGC every erasable is reachable from pinball. In C most of them are
 * fields of the state blocks of the guidance, program and vehicle modules.
 * agc_noun_read / agc_noun_write (src/io/dsky/nouns/data.c) resolve the
 * core erasables themselves and hand every other data id here.
 *
 * LAYERING: like dispatch.c, this file is the documented place where
 * src/exec/ reaches the higher layers (here: their public state blocks
 * and noun accessors), so that io never includes a higher header.
 *
 * Value convention (include/agc/io/nouns.h): DP erasables as the DP, SP
 * erasables in the high word (agc_dp_from_sp), raw words (octal, CDU
 * angles) as the 15-bit word. A vector component is one DP.
 *
 * Erasables with no owner in the C translation (ALMCADR +0..+2: the AGC
 * alarm return address, bank and ERCOUNT) stay NOT_FOUND; see
 * docs/modules/integrate.md.  LM AOTCODE +1/+2 (N70/N71 R2, R3) are
 * SINCDUY (docs/modules/review-prog-align.md).
 */
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/nouns.h"
#include "agc/lm/landing.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

/* Guidance (SERVICER, P30-P47) DP erasables.                            */
static agc_dp_t *noun_dp_guidance(uint16_t id)
{
    agc_guid_t *g = agc_guid_state();

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

/* CSM entry (P61-P67) DP erasables.                                     */
static agc_dp_t *noun_dp_entry(uint16_t id)
{
    agc_entry_state_t *e = agc_entry_state();

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
}

/* LM landing (P63-P68) and ascent (P12, P70-P71) DP erasables.          */
static agc_dp_t *noun_dp_landing(uint16_t id)
{
    agc_lm_ldg_t *l = agc_lm_ldg();
    agc_lm_asc_t *a = agc_lm_asc();

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
            return (&a->rdot);
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

/* Alignment (P51-P57, R52-R56) DP erasables and vector components.      */
static agc_dp_t *noun_dp_alignment(uint16_t id)
{
    agc_align_ram_t *r = agc_align_ram();

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

/* The DP erasable behind an id, by owner and vehicle (NULL: none).      */
static agc_dp_t *noun_dp(uint16_t id, bool lm)
{
    agc_dp_t *p = noun_dp_guidance(id);

    if (p == NULL)
    {
        p = lm ? noun_dp_landing(id) : noun_dp_entry(id);
    }

    if (p == NULL)
    {
        p = noun_dp_alignment(id);
    }

    return (p);
}

/* SP erasables (value in the high word).                                */
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

/* Raw 15-bit words (CDU angles, octal codes) held as uint16_t.          */
static uint16_t *noun_word(uint16_t id)
{
    agc_align_ram_t *r = agc_align_ram();

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

/* Two-integer words (scale TWO_INT: high word XX, low word YY). Nouns 45
 * (CM) and 64 (LM) are "no load" (SFINTABR: 2INTOUT CANT BE LOADED), so
 * only the sim and tests store them: VHFCNT,+1 (TRKMKCNT) and FUNNYDSP =
 * TREDES, LOOKANGL.                                                       */
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

/* Read-only values computed by functions of their owners.               */
static agc_status_t noun_derived(uint16_t id, bool lm, agc_dp_t *v)
{
    switch (id)
    {
        case AGC_DATA_FDAIX:
        case AGC_DATA_FDAIY:
        case AGC_DATA_FDAIZ: /* BALLANGS output (LM N18)            */
            *v = (agc_dp_t) agc_erase_fdai((int) (id - AGC_DATA_FDAIX));

            return (AGC_STATUS_OK);
        case AGC_DATA_ALPHASB: /* LM N51: PITCHANG; CM N51: RHOSB      */
        case AGC_DATA_RHOSB:
            *v = lm ? agc_sband_pitchang() : agc_sband_rhosb();

            return (AGC_STATUS_OK);
        case AGC_DATA_BETASB: /* LM N51: YAWANG; CM N51: GAMMASB      */
        case AGC_DATA_GAMMASB:
            *v = lm ? agc_sband_yawang() : agc_sband_gammasb();

            return (AGC_STATUS_OK);
        case AGC_DATA_TET: /* time of the state being integrated    */
            *v = agc_integ_shared_ctx()->sv.tet;

            return (AGC_STATUS_OK);
        case AGC_DATA_FUNNYDSP: /* LM N64: TREDES, LOOKANGL (DELTAH +2) */
            *v = agc_dp_pack(agc_lm_ldg()->tredes, agc_lm_ldg()->lookangl);

            return (AGC_STATUS_OK);
        default:
            return (AGC_STATUS_NOT_FOUND);
    }
}

/* A signed SP word as the raw 15-bit ones'-complement word (SP raw).    */
static agc_dp_t noun_raw_word(agc_sp_t w)
{
    return ((agc_dp_t) ((w >= 0) ? w : (077777 + w)));
}

/* The raw 15-bit word back as a signed SP (07777x negative).            */
static agc_sp_t noun_signed_word(agc_dp_t raw)
{
    const agc_sp_t w = (agc_sp_t) (raw & 077777); /* raw word, unsigned */

    return ((w > AGC_SP_MAX) ? (agc_sp_t) (w - 077777) : w);
}

/*
 * STARCODE (CSM N70/N71 R1), AOTCODE = STARCODE (LM N70/N71 R1), and the
 * LM's AOTCODE +1/+2 (R2, R3).  Luminary ERASABLE_ASSIGNMENTS p. 37:
 * AOTCODE = STARCODE, and STARALGN follows it, so AOTCODE +1/+2 are the
 * two words of SINCDU = SINCDUY (sin(CDUY)/2, CDUTRIG scratch); the C home
 * of SINCDU is the guidance block.  NOT_FOUND for any other id.
 */
static agc_status_t noun_star_code(uint16_t id, bool write, agc_dp_t *value)
{
    agc_sp_t *code  = &agc_align_ram()->starcode;                 /* STARCODE/AOTCODE */
    agc_dp_t *sin_y = &agc_guid_state()->trig.sincdu[AGC_TRIG_Y]; /* SINCDUY, 2^1 */

    switch (id)
    {
        case AGC_DATA_STARCODE:
        case AGC_DATA_AOTCODE:
            *code  = write ? noun_signed_word(*value) : *code;
            *value = noun_raw_word(*code);

            return (AGC_STATUS_OK);
        case AGC_DATA_AOTCODE_P1: /* AOTCODE +1: SINCDUY high word     */
            *sin_y = write ? agc_dp_pack(noun_signed_word(*value), agc_dp_low(*sin_y)) : *sin_y;
            *value = noun_raw_word(agc_dp_high(*sin_y));

            return (AGC_STATUS_OK);
        case AGC_DATA_AOTCODE_P2: /* AOTCODE +2: SINCDUY low word      */
            *sin_y = write ? agc_dp_pack(agc_dp_high(*sin_y), noun_signed_word(*value)) : *sin_y;
            *value = noun_raw_word(agc_dp_low(*sin_y));

            return (AGC_STATUS_OK);
        default:
            return (AGC_STATUS_NOT_FOUND);
    }
}

/* Read (write = false) or store the erasable behind a data id that core
 * does not hold. NOT_FOUND: no owner; BAD_ARG: bad arguments.           */
agc_status_t agc_dispatch_noun(uint16_t data_id, bool write, agc_dp_t *value)
{
    if (!AGC_ASSERT(data_id > AGC_DATA_NONE && data_id < AGC_DATA_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool      lm = agc_state_vehicle() == AGC_VEHICLE_LM;
    agc_dp_t *dp = (data_id == AGC_DATA_VMAGI && !lm && agc_erase_modreg() == 11) ? NULL : noun_dp(data_id, lm);
    agc_sp_t *sp = noun_sp(data_id);
    uint16_t *w  = noun_word(data_id);

    if (dp != NULL)
    {
        *dp    = write ? *value : *dp;
        *value = *dp;
    }
    else if (sp != NULL)
    {
        *sp    = write ? agc_dp_high(*value) : *sp;
        *value = agc_dp_from_sp(*sp);
    }
    else if (w != NULL)
    {
        *w     = write ? (uint16_t) (*value & 077777) : *w;
        *value = (agc_dp_t) *w;
    }
    else
    {
        /* Ids resolved through functions: the P11 readout, LAT/LONG/ALT, P37,
         * the rendezvous programs.                                              */
        bool p11 = !lm && agc_erase_modreg() == 11;

        if ((data_id == AGC_DATA_HDOT || data_id == AGC_DATA_ALTI || (data_id == AGC_DATA_VMAGI && p11)) && !write)
        {
            /* VMAGI, HDOT, ALTI of the P11 monitor (N62).                           */
            {
                agc_dp_t vmagi = 0;
                agc_dp_t hdot  = 0;
                agc_dp_t alti  = 0;

                agc_status_t st = agc_earth_orbit_insertion_monitor_readout(&vmagi, &hdot, &alti);

                if (st != AGC_STATUS_OK)
                {
                    return (st);
                }

                *value = (data_id == AGC_DATA_HDOT) ? hdot : (data_id == AGC_DATA_ALTI) ? alti : vmagi;

                return (AGC_STATUS_OK);
            }
        }

        if (data_id == AGC_DATA_LAT || data_id == AGC_DATA_LONG || data_id == AGC_DATA_ALT)
        {
            /* LAT, LONG, ALT (noun 43): the nav-aux store.                          */
            {
                agc_latlong_t ll;
                agc_status_t  st = agc_navaux_get_latlong(&ll);

                if (st != AGC_STATUS_OK)
                {
                    return (st);
                }

                agc_dp_t *f = (data_id == AGC_DATA_LAT) ? &ll.lat : (data_id == AGC_DATA_LONG) ? &ll.lon : &ll.alt;

                if (!write)
                {
                    *value = *f;

                    return (AGC_STATUS_OK);
                }

                *f = *value;

                return (agc_navaux_set_latlong(&ll));
            }
        }

        agc_status_t st = noun_star_code(data_id, write, value);

        if (st != AGC_STATUS_NOT_FOUND)
        {
            return (st);
        }

        st = lm ? AGC_STATUS_NOT_FOUND : agc_return_to_earth_noun(data_id, write, value);

        if (st == AGC_STATUS_NOT_FOUND)
        {
            st = write ? agc_rendz_noun_write(data_id, *value) : agc_rendz_noun_read(data_id, value);
        }

        if (st == AGC_STATUS_NOT_FOUND && !write)
        {
            st = lm ? agc_lm_rendz_noun_read(data_id, value) : agc_csm_rendz_noun_read(data_id, value);
        }

        if (st == AGC_STATUS_NOT_FOUND && write && (data_id == AGC_DATA_VHFCNT || data_id == AGC_DATA_FUNNYDSP))
        {
            st = noun_two_int_write(data_id, *value);
        }

        return ((st == AGC_STATUS_NOT_FOUND && !write) ? noun_derived(data_id, lm, value) : st);
    }

    return (AGC_STATUS_OK);
}
