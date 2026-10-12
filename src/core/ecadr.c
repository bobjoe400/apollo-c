/*
 * ecadr.c -- erasable access by AGC address (ECADR), for the machine-address
 * nouns of pinball (nouns 01-03, 07, 15: V01/V21/V25 of any erasable), the
 * uplink update program (V71/V72), the downlink erasable dump, and hosts
 * that see the AGC as an addressable memory (src/nassp: NASSP reads and
 * pad-loads erasable by address).
 *
 * The C translation keeps each erasable in its owner's typed storage, not
 * in an addressable array. Three kinds of address:
 *   - the flagwords (STATE +0.., ECADR 074 in both ropes, so FLAGWRD0..
 *     FLGWRD13 = 074..0111) map onto agc_flagword_get/set;
 *   - the named erasables in the address map below (k_map_csm / k_map_lm,
 *     addresses from the Comanche055 / Luminary099 assembly listings)
 *     map onto their typed accessors: REFSMMAT, TEPHEM, the state vectors,
 *     the uplink buffer (UPSVFLAG, RRECT, VRECT, TET), times, masses, DAP
 *     words, display registers;
 *   - every other address reads and writes a raw image of the 15-bit
 *     address space (0000-77777: erasable E0..E7 at 0000-3777, then the
 *     fixed-memory pseudo addresses, which hold no rope here and simply
 *     keep what was stored).
 * For a mapped erasable the raw image keeps the words last stored, so a
 * DP or TP quantity loaded one word at a time (V71 stores from the last
 * word to the first) keeps the words exactly as given, signs free to
 * disagree, and the typed value is rebuilt from all its words after each
 * store. When the program has changed the typed value since, the words
 * are refreshed from it (sign-agreeing) before a read or a store.
 * DEVIATION: an address that is neither a flagword nor in the map does not
 * reach any typed erasable of the same name; the raw image keeps the value
 * so a later V01/V05 of the same address reads it.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/io/service.h"
#include "agc/nav/integration.h"

#define EC_WORDS     0100000u /* address 00000 .. 77777                  */
#define EC_STATE     0074u    /* STATE = FLAGWRD0 (both ropes, unswitched)   */
#define EC_WORD_MAX  077777u
#define EC_SIGN      040000u  /* ones'-complement sign bit                */
#define EC_MAG       037777u  /* ones'-complement magnitude               */
#define EC_MAX_WORDS 18       /* longest mapped erasable: REFSMMAT        */
#define EC_DP_WORDS  2        /* words in a DP element                    */
#define EC_TP_WORDS  3        /* words in a TP element                    */

/* Named erasables with a typed home.                                      */
typedef enum ec_item
{
    EC_MODREG = 0, /* SP   MODREG                            */
    EC_CSMMASS,    /* SP   CSMMASS, kg 2^16                  */
    EC_LEMMASS,    /* SP   LEMMASS, kg 2^16                  */
    EC_OPTION1,    /* SP x3 OPTION1..3 (CSM OPTION1..2)      */
    EC_TRKMKCNT,   /* SP   TRKMKCNT                          */
    EC_CDUCHKWD,   /* SP   CDUCHKWD (CSM)                    */
    EC_P37RANGE,   /* SP   P37RANGE (CSM)                    */
    EC_ZOOMTIME,   /* SP   ZOOMTIME (LM)                     */
    EC_S61DT,      /* SP   S61DT (CSM)                       */
    EC_CM_GYMDT,   /* SP   CM/GYMDT (CSM)                    */
    EC_UPSVFLAG,   /* SP   UPSVFLAG                          */
    EC_DAPDATR1,   /* RAW  DAPDATR1                          */
    EC_DAPDATR2,   /* RAW  DAPDATR2 (CSM)                    */
    EC_MARKSTAT,   /* RAW  MARKSTAT                          */
    EC_IMODES30,   /* RAW  IMODES30                          */
    EC_IMODES33,   /* RAW  IMODES33                          */
    EC_DSPTAB,     /* RAW x12 DSPTAB                         */
    EC_FAILREG,    /* RAW x3 FAILREG                         */
    EC_THETAD,     /* RAW x3 THETAD                          */
    EC_AOTAZ,      /* RAW x6 AOTAZ (LM)                      */
    EC_AOTEL,      /* RAW x6 AOTEL (LM)                      */
    EC_MARKDOWN,   /* RAW x7 MARKDOWN (CSM)                  */
    EC_MARK2DWN,   /* RAW x7 MARK2DWN (CSM)                  */
    EC_PIPTIME,    /* DP   PIPTIME, cs 2^28                  */
    EC_TIG,        /* DP   TIG                               */
    EC_TTOGO,      /* DP   TTOGO                             */
    EC_TGO,        /* DP   TGO                               */
    EC_TTPI,       /* DP   TTPI                              */
    EC_TEVENT,     /* DP   TEVENT                            */
    EC_TETCSM,     /* DP   TETCSM                            */
    EC_TETLEM,     /* DP   TETLEM                            */
    EC_T3TOT4,     /* DP   T3TOT4 (CSM)                      */
    EC_RR_AZ,      /* DP   RR-AZ (LM)                        */
    EC_RR_ELEV,    /* DP   RR-ELEV (LM)                      */
    EC_RTED1,      /* DP   RTED1 (CSM)                       */
    EC_TLAND,      /* DP   TLAND (LM)                        */
    EC_P40TMP,     /* DP   P40TMP (CSM)                      */
    EC_SAVET_30,   /* DP   SAVET-30 (LM)                     */
    EC_TET,        /* DP   TET (uplink / integration buffer) */
    EC_RN,         /* VEC  RN                                */
    EC_VN,         /* VEC  VN                                */
    EC_RRECTCSM,   /* VEC  RRECTCSM                          */
    EC_VRECTCSM,   /* VEC  VRECTCSM                          */
    EC_RRECTLEM,   /* VEC  RRECTLEM                          */
    EC_VRECTLEM,   /* VEC  VRECTLEM                          */
    EC_DELV,       /* VEC  DELV                              */
    EC_VGTIG,      /* VEC  VGTIG                             */
    EC_RLS,        /* VEC  RLS                               */
    EC_RRECT,      /* VEC  RRECT (uplink / integration)      */
    EC_VRECT,      /* VEC  VRECT (uplink / integration)      */
    EC_REFSMMAT,   /* MAT  REFSMMAT, 2^1                     */
    EC_TEPHEM      /* TP   TEPHEM, cs 2^42                   */
} ec_item_t;

/* How an item's words group into values.                                 */
typedef enum ec_layout
{
    EC_SP = 0, /* one signed word per element               */
    EC_RAW,    /* one bit-pattern word per element          */
    EC_DP,     /* two words per element                     */
    EC_TP      /* three words                               */
} ec_layout_t;

typedef struct ec_entry
{
    uint16_t  ecadr; /* first address              */
    int       words; /* number of words            */
    ec_item_t item;  /* the typed erasable         */
} ec_entry_t;

/* Comanche055 addresses (ERASABLE_ASSIGNMENTS, from the listing).        */
static const ec_entry_t k_map_csm[] = {
    {00375u, 3, EC_FAILREG},   {01011u, 1, EC_MODREG},    {01023u, 12, EC_DSPTAB},   {01126u, 1, EC_TRKMKCNT},
    {01131u, 2, EC_OPTION1},   {01155u, 3, EC_THETAD},    {01162u, 6, EC_DELV},      {01170u, 6, EC_RN},
    {01176u, 6, EC_VN},        {01204u, 2, EC_PIPTIME},   {01320u, 1, EC_IMODES30},  {01321u, 1, EC_IMODES33},
    {01330u, 1, EC_MARKSTAT},  {01336u, 2, EC_TEVENT},    {01341u, 1, EC_CDUCHKWD},  {01351u, 2, EC_RTED1},
    {01501u, 1, EC_UPSVFLAG},  {01502u, 6, EC_RRECT},     {01510u, 6, EC_VRECT},     {01516u, 2, EC_TET},
    {01554u, 6, EC_RRECTCSM},  {01562u, 6, EC_VRECTCSM},  {01570u, 2, EC_TETCSM},    {01626u, 6, EC_RRECTLEM},
    {01634u, 6, EC_VRECTLEM},  {01642u, 2, EC_TETLEM},    {01706u, 3, EC_TEPHEM},    {01735u, 18, EC_REFSMMAT},
    {02025u, 6, EC_RLS},       {02640u, 2, EC_T3TOT4},    {03066u, 1, EC_DAPDATR1},  {03067u, 1, EC_DAPDATR2},
    {03073u, 1, EC_LEMMASS},   {03074u, 1, EC_CSMMASS},   {03325u, 1, EC_CM_GYMDT},  {03374u, 1, EC_S61DT},
    {03376u, 1, EC_P37RANGE},  {03412u, 2, EC_TIG},       {03427u, 2, EC_TGO},       {03502u, 7, EC_MARK2DWN},
    {03660u, 2, EC_TTOGO},     {03662u, 2, EC_TTPI},      {03672u, 2, EC_P40TMP},    {03674u, 7, EC_MARKDOWN},
    {03720u, 6, EC_VGTIG}};

/* Luminary099 addresses (ERASABLE_ASSIGNMENTS, from the listing).        */
static const ec_entry_t k_map_lm[] = {
    {00321u, 3, EC_THETAD},    {00324u, 6, EC_DELV},      {00375u, 3, EC_FAILREG},   {01011u, 1, EC_MODREG},
    {01023u, 12, EC_DSPTAB},   {01144u, 3, EC_OPTION1},   {01220u, 6, EC_RN},        {01226u, 6, EC_VN},
    {01234u, 2, EC_PIPTIME},   {01302u, 1, EC_IMODES30},  {01303u, 1, EC_IMODES33},  {01312u, 1, EC_MARKSTAT},
    {01331u, 1, EC_LEMMASS},   {01332u, 1, EC_CSMMASS},   {01343u, 1, EC_DAPDATR1},  {01344u, 2, EC_TEVENT},
    {01501u, 1, EC_UPSVFLAG},  {01502u, 6, EC_RRECT},     {01510u, 6, EC_VRECT},     {01516u, 2, EC_TET},
    {01554u, 6, EC_RRECTCSM},  {01562u, 6, EC_VRECTCSM},  {01570u, 2, EC_TETCSM},    {01626u, 6, EC_RRECTLEM},
    {01634u, 6, EC_VRECTLEM},  {01642u, 2, EC_TETLEM},    {01706u, 3, EC_TEPHEM},    {01733u, 18, EC_REFSMMAT},
    {02022u, 6, EC_RLS},       {02200u, 2, EC_RR_AZ},     {02202u, 2, EC_RR_ELEV},   {02400u, 2, EC_TLAND},
    {03404u, 6, EC_AOTAZ},     {03412u, 6, EC_AOTEL},     {03422u, 1, EC_ZOOMTIME},  {03441u, 2, EC_TIG},
    {03453u, 2, EC_TTOGO},     {03462u, 1, EC_TRKMKCNT},  {03477u, 2, EC_SAVET_30},  {03516u, 2, EC_TGO},
    {03635u, 2, EC_TTPI},      {03700u, 6, EC_VGTIG}};

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

/* ---- word <-> value ------------------------------------------------------ */

static uint16_t word_of_sp(agc_sp_t v)
{
    return ((v < 0) ? (uint16_t) (~(uint16_t) (-(int32_t) v) & EC_WORD_MAX) : (uint16_t) v);
}

static agc_sp_t sp_of_word(uint16_t w)
{
    return (((w & EC_SIGN) != 0u) ? (agc_sp_t) -(int32_t) ((uint16_t) ~w & EC_MAG) : (agc_sp_t) w);
}

static void dp_to(agc_dp_t v, uint16_t *w)
{
    agc_dp_words_t d = agc_dp_to_words(v); /* sign-agreeing pair */

    w[0] = word_of_sp(d.hi);
    w[1] = word_of_sp(d.lo);
}

static agc_dp_t dp_of(const uint16_t *w)
{
    agc_dp_words_t d = {sp_of_word(w[0]), sp_of_word(w[1])}; /* signs free */

    return (agc_dp_from_words(d));
}

static void vec_to(agc_vec_t v, uint16_t *w)
{
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        dp_to(v.c[i], &w[EC_DP_WORDS * i]);
    }
}

static agc_vec_t vec_of(const uint16_t *w)
{
    agc_vec_t v; /* the vector */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        v.c[i] = dp_of(&w[EC_DP_WORDS * i]);
    }

    return (v);
}

/* ---- the items ----------------------------------------------------------- */

static ec_layout_t item_layout(ec_item_t item)
{
    ec_layout_t l = EC_DP; /* DP, VEC, MAT */

    if (item <= EC_UPSVFLAG)
    {
        l = EC_SP;
    }
    else if (item <= EC_MARK2DWN)
    {
        l = EC_RAW;
    }
    else if (item == EC_TEPHEM)
    {
        l = EC_TP;
    }
    else
    {
        /* DP elements */
    }

    return (l);
}

/* SP items: the value of element k.                                       */
static agc_sp_t sp_get(ec_item_t item, int k)
{
    agc_sp_t v = 0; /* the value */

    switch ((int) item)
    {
        case EC_MODREG:
            v = agc_erase_modreg();
            break;
        case EC_CSMMASS:
            v = agc_erase_csmmass();
            break;
        case EC_LEMMASS:
            v = agc_erase_lemmass();
            break;
        case EC_OPTION1:
            v = agc_erase_option1(k);
            break;
        case EC_TRKMKCNT:
            v = agc_erase_trkmkcnt();
            break;
        case EC_CDUCHKWD:
            v = agc_erase_cduchkwd();
            break;
        case EC_P37RANGE:
            v = agc_erase_return_to_earth_range();
            break;
        case EC_ZOOMTIME:
            v = agc_erase_zoomtime();
            break;
        case EC_S61DT:
            v = agc_erase_s61dt();
            break;
        case EC_CM_GYMDT:
            v = agc_erase_cm_gymdt();
            break;
        default:
            v = agc_erase_upsvflag(); /* EC_UPSVFLAG: the last SP item */
            break;
    }

    return (v);
}

static agc_status_t sp_set(ec_item_t item, int k, agc_sp_t v)
{
    agc_status_t st = AGC_STATUS_OK; /* outcome */

    switch ((int) item)
    {
        case EC_MODREG:
            agc_erase_set_modreg(v);
            break;
        case EC_CSMMASS:
            agc_erase_set_csmmass(v);
            break;
        case EC_LEMMASS:
            agc_erase_set_lemmass(v);
            break;
        case EC_OPTION1:
            agc_erase_set_option1(k, v);
            break;
        case EC_TRKMKCNT:
            agc_erase_set_trkmkcnt(v);
            break;
        case EC_CDUCHKWD:
            agc_erase_set_cduchkwd(v);
            break;
        case EC_P37RANGE:
            agc_erase_set_return_to_earth_range(v);
            break;
        case EC_ZOOMTIME:
            agc_erase_set_zoomtime(v);
            break;
        case EC_S61DT:
            agc_erase_set_s61dt(v);
            break;
        case EC_CM_GYMDT:
            agc_erase_set_cm_gymdt(v);
            break;
        default:
            st = agc_erase_set_upsvflag(v); /* EC_UPSVFLAG */
            break;
    }

    return (st);
}

/* RAW items: the word of element k.                                       */
static uint16_t raw_get(ec_item_t item, int k)
{
    uint16_t w = 0; /* the word */

    switch ((int) item)
    {
        case EC_DAPDATR1:
            w = agc_erase_dapdatr1();
            break;
        case EC_DAPDATR2:
            w = agc_erase_dapdatr2();
            break;
        case EC_MARKSTAT:
            w = agc_erase_markstat();
            break;
        case EC_IMODES30:
            w = agc_erase_imodes30();
            break;
        case EC_IMODES33:
            w = agc_erase_imodes33();
            break;
        case EC_DSPTAB:
            w = agc_erase_dsptab(k);
            break;
        case EC_FAILREG:
            w = agc_erase_failreg(k);
            break;
        case EC_THETAD:
            w = agc_erase_thetad(k);
            break;
        case EC_AOTAZ:
            w = agc_erase_aotaz(k + 1);
            break;
        case EC_AOTEL:
            w = agc_erase_aotel(k + 1);
            break;
        case EC_MARKDOWN:
            w = agc_erase_markdown(k);
            break;
        default:
            w = agc_erase_mark2dwn(k); /* EC_MARK2DWN */
            break;
    }

    return ((uint16_t) (w & EC_WORD_MAX));
}

static void raw_set(ec_item_t item, int k, uint16_t w)
{
    switch ((int) item)
    {
        case EC_DAPDATR1:
            agc_erase_set_dapdatr1(w);
            break;
        case EC_DAPDATR2:
            agc_erase_set_dapdatr2(w);
            break;
        case EC_MARKSTAT:
            agc_erase_set_markstat(w);
            break;
        case EC_IMODES30:
            agc_erase_set_imodes30(w);
            break;
        case EC_IMODES33:
            agc_erase_set_imodes33(w);
            break;
        case EC_DSPTAB:
            agc_erase_set_dsptab(k, w);
            break;
        case EC_FAILREG:
            agc_erase_set_failreg(k, w);
            break;
        case EC_THETAD:
            agc_erase_set_thetad(k, w);
            break;
        case EC_AOTAZ:
            agc_erase_set_aotaz(k + 1, w);
            break;
        case EC_AOTEL:
            agc_erase_set_aotel(k + 1, w);
            break;
        case EC_MARKDOWN:
            agc_erase_set_markdown(k, w);
            break;
        default:
            agc_erase_set_mark2dwn(k, w); /* EC_MARK2DWN */
            break;
    }
}

/* DP items (one element).                                                 */
static agc_dp_t dp_get(ec_item_t item)
{
    agc_dp_t v = 0; /* the value */

    switch ((int) item)
    {
        case EC_PIPTIME:
            v = agc_erase_piptime();
            break;
        case EC_TIG:
            v = agc_erase_tig();
            break;
        case EC_TTOGO:
            v = agc_erase_ttogo();
            break;
        case EC_TGO:
            v = agc_erase_tgo();
            break;
        case EC_TTPI:
            v = agc_erase_ttpi();
            break;
        case EC_TEVENT:
            v = agc_erase_tevent();
            break;
        case EC_TETCSM:
            v = agc_erase_tetcsm();
            break;
        case EC_TETLEM:
            v = agc_erase_tetlem();
            break;
        case EC_T3TOT4:
            v = agc_erase_t3tot4();
            break;
        case EC_RR_AZ:
            v = agc_erase_rr_az();
            break;
        case EC_RR_ELEV:
            v = agc_erase_rr_elev();
            break;
        case EC_RTED1:
            v = agc_erase_rted1();
            break;
        case EC_TLAND:
            v = agc_erase_tland();
            break;
        case EC_P40TMP:
            v = agc_erase_p40tmp();
            break;
        case EC_SAVET_30:
            v = agc_erase_savet_30();
            break;
        default:
            v = agc_integ_shared_ctx()->sv.tet; /* EC_TET */
            break;
    }

    return (v);
}

static void dp_set(ec_item_t item, agc_dp_t v)
{
    switch ((int) item)
    {
        case EC_PIPTIME:
            agc_erase_set_piptime(v);
            break;
        case EC_TIG:
            agc_erase_set_tig(v);
            break;
        case EC_TTOGO:
            agc_erase_set_ttogo(v);
            break;
        case EC_TGO:
            agc_erase_set_tgo(v);
            break;
        case EC_TTPI:
            agc_erase_set_ttpi(v);
            break;
        case EC_TEVENT:
            agc_erase_set_tevent(v);
            break;
        case EC_TETCSM:
            agc_erase_set_tetcsm(v);
            break;
        case EC_TETLEM:
            agc_erase_set_tetlem(v);
            break;
        case EC_T3TOT4:
            agc_erase_set_t3tot4(v);
            break;
        case EC_RR_AZ:
            agc_erase_set_rr_az(v);
            break;
        case EC_RR_ELEV:
            agc_erase_set_rr_elev(v);
            break;
        case EC_RTED1:
            agc_erase_set_rted1(v);
            break;
        case EC_TLAND:
            agc_erase_set_tland(v);
            break;
        case EC_P40TMP:
            agc_erase_set_p40tmp(v);
            break;
        case EC_SAVET_30:
            agc_erase_set_savet_30(v);
            break;
        default:
            agc_integ_shared_ctx()->sv.tet = v; /* EC_TET */
            break;
    }
}

/* VEC items.                                                              */
static agc_vec_t vec_get(ec_item_t item)
{
    agc_vec_t v; /* the vector */

    switch ((int) item)
    {
        case EC_RN:
            v = agc_erase_rn();
            break;
        case EC_VN:
            v = agc_erase_vn();
            break;
        case EC_RRECTCSM:
            v = agc_erase_rrectcsm();
            break;
        case EC_VRECTCSM:
            v = agc_erase_vrectcsm();
            break;
        case EC_RRECTLEM:
            v = agc_erase_rrectlem();
            break;
        case EC_VRECTLEM:
            v = agc_erase_vrectlem();
            break;
        case EC_DELV:
            v = agc_erase_delv();
            break;
        case EC_VGTIG:
            v = agc_erase_vgtig();
            break;
        case EC_RLS:
            v = agc_erase_rls();
            break;
        case EC_RRECT:
            v = agc_integ_shared_ctx()->sv.rrect;
            break;
        default:
            v = agc_integ_shared_ctx()->sv.vrect; /* EC_VRECT */
            break;
    }

    return (v);
}

static void vec_set(ec_item_t item, agc_vec_t v)
{
    switch ((int) item)
    {
        case EC_RN:
            agc_erase_set_rn(v);
            break;
        case EC_VN:
            agc_erase_set_vn(v);
            break;
        case EC_RRECTCSM:
            agc_erase_set_rrectcsm(v);
            break;
        case EC_VRECTCSM:
            agc_erase_set_vrectcsm(v);
            break;
        case EC_RRECTLEM:
            agc_erase_set_rrectlem(v);
            break;
        case EC_VRECTLEM:
            agc_erase_set_vrectlem(v);
            break;
        case EC_DELV:
            agc_erase_set_delv(v);
            break;
        case EC_VGTIG:
            agc_erase_set_vgtig(v);
            break;
        case EC_RLS:
            agc_erase_set_rls(v);
            break;
        case EC_RRECT:
            agc_integ_shared_ctx()->sv.rrect = v;
            break;
        default:
            agc_integ_shared_ctx()->sv.vrect = v; /* EC_VRECT */
            break;
    }
}

/* The typed value of an item as sign-agreeing words.                      */
static void item_words(const ec_entry_t *e, uint16_t *w)
{
    ec_layout_t l = item_layout(e->item); /* element layout */

    if (l == EC_SP)
    {
        for (int k = 0; k < e->words; k++)
        {
            w[k] = word_of_sp(sp_get(e->item, k));
        }
    }
    else if (l == EC_RAW)
    {
        for (int k = 0; k < e->words; k++)
        {
            w[k] = raw_get(e->item, k);
        }
    }
    else if (l == EC_TP)
    {
        agc_tp_words_t t = agc_tp_to_words(agc_erase_tephem()); /* TEPHEM */

        w[0] = word_of_sp(t.hi);
        w[1] = word_of_sp(t.mid);
        w[2] = word_of_sp(t.lo);
    }
    else if (e->item == EC_REFSMMAT)
    {
        agc_mat_t m = agc_erase_refsmmat(); /* rows */

        for (int r = 0; r < AGC_VEC_N; r++)
        {
            vec_to(m.row[r], &w[EC_DP_WORDS * AGC_VEC_N * r]);
        }
    }
    else if (e->words == EC_DP_WORDS)
    {
        dp_to(dp_get(e->item), w);
    }
    else
    {
        vec_to(vec_get(e->item), w);
    }
}

/* Store an item from its words (signs free to disagree).                  */
static agc_status_t item_store(const ec_entry_t *e, const uint16_t *w)
{
    ec_layout_t  l  = item_layout(e->item); /* element layout */
    agc_status_t st = AGC_STATUS_OK;        /* first refusal */

    if (l == EC_SP)
    {
        for (int k = 0; k < e->words && st == AGC_STATUS_OK; k++)
        {
            st = sp_set(e->item, k, sp_of_word(w[k]));
        }
    }
    else if (l == EC_RAW)
    {
        for (int k = 0; k < e->words; k++)
        {
            raw_set(e->item, k, w[k]);
        }
    }
    else if (l == EC_TP)
    {
        agc_tp_words_t t = {sp_of_word(w[0]), sp_of_word(w[1]), sp_of_word(w[2])}; /* TEPHEM words */

        agc_erase_set_tephem(agc_tp_from_words(t));
    }
    else if (e->item == EC_REFSMMAT)
    {
        agc_mat_t m; /* rows */

        for (int r = 0; r < AGC_VEC_N; r++)
        {
            m.row[r] = vec_of(&w[EC_DP_WORDS * AGC_VEC_N * r]);
        }

        agc_erase_set_refsmmat(m);
    }
    else if (e->words == EC_DP_WORDS)
    {
        dp_set(e->item, dp_of(w));
    }
    else
    {
        vec_set(e->item, vec_of(w));
    }

    return (st);
}

/* Whether two word images of an item hold the same values.                */
static bool same_values(const ec_entry_t *e, const uint16_t *a, const uint16_t *b)
{
    ec_layout_t l    = item_layout(e->item); /* element layout */
    bool        same = true;                 /* no element differs */

    if (l == EC_SP)
    {
        for (int k = 0; k < e->words; k++)
        {
            same = same && sp_of_word(a[k]) == sp_of_word(b[k]);
        }
    }
    else if (l == EC_RAW)
    {
        for (int k = 0; k < e->words; k++)
        {
            same = same && a[k] == b[k];
        }
    }
    else if (l == EC_TP)
    {
        agc_tp_words_t ta = {sp_of_word(a[0]), sp_of_word(a[1]), sp_of_word(a[2])}; /* a */
        agc_tp_words_t tb = {sp_of_word(b[0]), sp_of_word(b[1]), sp_of_word(b[2])}; /* b */

        same = agc_tp_from_words(ta) == agc_tp_from_words(tb);
    }
    else
    {
        for (int k = 0; k < e->words; k += EC_DP_WORDS)
        {
            same = same && dp_of(&a[k]) == dp_of(&b[k]);
        }
    }

    return (same);
}

/* The map entry holding ecadr, or NULL.                                   */
static const ec_entry_t *map_find(uint16_t ecadr)
{
    bool              lm  = agc_state_vehicle() == AGC_VEHICLE_LM;                                            /* which map */
    const ec_entry_t *map = lm ? k_map_lm : k_map_csm;                                                        /* the map */
    size_t            n   = lm ? sizeof (k_map_lm) / sizeof (k_map_lm[0]) : sizeof (k_map_csm) / sizeof (k_map_csm[0]); /* entries */
    const ec_entry_t *hit = NULL;                                                                             /* found */

    for (size_t i = 0; i < n && hit == NULL; i++)
    {
        if (ecadr >= map[i].ecadr && ecadr < map[i].ecadr + (uint16_t) map[i].words)
        {
            hit = &map[i];
        }
    }

    return (hit);
}

/* Bring the raw words of a mapped item up to date with its typed value.   */
static void refresh(const ec_entry_t *e)
{
    uint16_t w[EC_MAX_WORDS]; /* typed value as words */

    item_words(e, w);

    if (!same_values(e, &g_image[e->ecadr], w))
    {
        for (int k = 0; k < e->words; k++)
        {
            g_image[e->ecadr + (uint16_t) k] = w[k];
        }
    }
}

/* Read the erasable word at ECADR (raw 15-bit word).                     */
void agc_erase_read_ecadr(uint16_t ecadr, uint16_t *word)
{
    const ec_entry_t *e = map_find(ecadr); /* named erasable */

    if (is_flagword(ecadr))
    {
        *word = (uint16_t) (agc_flagword_get((int) (ecadr - EC_STATE)) & EC_WORD_MAX);
    }
    else if (e != NULL)
    {
        refresh(e);
        *word = g_image[ecadr];
    }
    else
    {
        *word = g_image[ecadr];
    }
}

/* Store a raw 15-bit word at ECADR.                                      */
agc_status_t agc_erase_write_ecadr(uint16_t ecadr, uint16_t word)
{
    const ec_entry_t *e  = map_find(ecadr); /* named erasable */
    agc_status_t      st = AGC_STATUS_OK;   /* outcome */

    if (!AGC_ASSERT(word <= EC_WORD_MAX))
    {
        st = AGC_STATUS_BAD_ARG;
    }
    else if (is_flagword(ecadr))
    {
        agc_flagword_set((int) (ecadr - EC_STATE), word);
    }
    else if (e != NULL)
    {
        refresh(e);
        g_image[ecadr] = word;
        st             = item_store(e, &g_image[e->ecadr]);
    }
    else
    {
        g_image[ecadr] = word;
    }

    return (st);
}
