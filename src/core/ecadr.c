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
 *     the uplink buffer (UPSVFLAG and the RRECT..XKEP block) and the two
 *     permanent state vector blocks (RRECTCSM.., RRECTLEM..), the external dV
 *     (DELVSLV) and desired REFSMMAT (XSMD), LAT/LONG/ALT, LAT(SPL)/LNG(SPL), times, masses, DAP
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
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define EC_WORDS     0100000u /* address 00000 .. 77777                  */
#define EC_STATE     0074u    /* STATE = FLAGWRD0 (both ropes, unswitched)   */
#define EC_WORD_MAX  077777u
#define EC_SIGN      040000u  /* ones'-complement sign bit                */
#define EC_MAG       037777u  /* ones'-complement magnitude               */
#define EC_MAX_WORDS 60       /* longest mapped erasable: VECOEM          */
#define EC_SV_DPS    21       /* DP values in a state vector block         */
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
    EC_IMUCOMP,    /* SP x15 PBIASX PIPASCFX .. ADSRAZ (IMU compensation pad load) */
    EC_GCOMPSW,    /* SP   GCOMPSW                           */
    EC_LUNARCOEF,  /* SP x2 E3J22R2M, E32C31RM (lunar potential) */
    EC_EMDOT,      /* SP   EMDOT SPS flow (CSM pad load)     */
    EC_DVTHRESH,   /* SP   DVTHRESH (CSM)                    */
    EC_FANG,       /* SP   FANG SPS thrust (CSM)             */
    EC_LADPAD,     /* SP   LADPAD (CSM entry)                */
    EC_LODPAD,     /* SP   LODPAD (CSM entry)                */
    EC_ALFAPAD,    /* SP   ALFAPAD (CSM entry)               */
    EC_ETDECAY,    /* SP   ETDECAY (CSM)                     */
    EC_ESTROKER,   /* SP   ESTROKER (CSM TVC)                */
    EC_EKPRIME,    /* SP x2 EKPRIME (CSM TVC)                */
    EC_EKTLX_I,    /* SP x3 EKTLX/I (CSM TVC)                */
    EC_EREPFRAC,   /* SP x2 EREPFRAC (CSM TVC)               */
    EC_PACTOFF,    /* SP   PACTOFF (CSM)                     */
    EC_YACTOFF,    /* SP   YACTOFF (CSM)                     */
    EC_HBN10,      /* SP x15 HBN10 .. HBD32 (CSM TVC filter) */
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
    EC_T3TOT4,     /* DP   T3TOT4 (CSM)                      */
    EC_RR_AZ,      /* DP   RR-AZ (LM)                        */
    EC_RR_ELEV,    /* DP   RR-ELEV (LM)                      */
    EC_RTED1,      /* DP   RTED1 (CSM)                       */
    EC_TLAND,      /* DP   TLAND (LM)                        */
    EC_P40TMP,     /* DP   P40TMP (CSM)                      */
    EC_SAVET_30,   /* DP   SAVET-30 (LM)                     */
    EC_LAT,        /* DP   LAT, revs 2^0 (noun 43)           */
    EC_LONG,       /* DP   LONG, revs 2^0                    */
    EC_ALT,        /* DP   ALT, m 2^29                       */
    EC_AZO,        /* DP   AZO, revs (launch pad load)       */
    EC_NEG_AYO,    /* DP   -AYO, rad                         */
    EC_AXO,        /* DP   AXO, rad                          */
    EC_504LM,      /* DP x3 504LM, rad                       */
    EC_VECOEM,     /* DP x30 VECOEM lunar ephemeris (CSM)    */
    EC_RESO,       /* DP x3 RESO (CSM)                       */
    EC_VESO,       /* DP x3 VESO (CSM)                       */
    EC_OMEGAES,    /* DP   OMEGAES (CSM)                     */
    EC_EK1VAL,     /* DP   EK1VAL (CSM)                      */
    EC_EK2VAL,     /* DP   EK2VAL (CSM)                      */
    EC_EK3VAL,     /* DP   EK3VAL (CSM)                      */
    EC_EMSALT,     /* DP   EMSALT (CSM entry)                */
    EC_LAT_SPL,    /* DP   LAT(SPL) entry target, revs (CSM) */
    EC_LNG_SPL,    /* DP   LNG(SPL) entry target, revs (CSM) */
    EC_RN,         /* VEC  RN                                */
    EC_VN,         /* VEC  VN                                */
    EC_DELV,       /* VEC  DELV                              */
    EC_VGTIG,      /* VEC  VGTIG                             */
    EC_RLS,        /* VEC  RLS                               */
    EC_DELVSLV,    /* VEC  DELVSLV = DELVLVC (external dV)   */
    EC_SV_TEMP,    /* SV   RRECT..XKEP (uplink / integration buffer, 42 words) */
    EC_SV_CSM,     /* SV   RRECTCSM..XKEPCSM (42 words)       */
    EC_SV_LEM,     /* SV   RRECTLEM..XKEPLEM (42 words)       */
    EC_XSMD,       /* MAT  XSMD.. desired REFSMMAT, 2^1      */
    EC_REFSMMAT,   /* MAT  REFSMMAT, 2^1                     */
    EC_TEPHEM,     /* TP   TEPHEM, cs 2^42                   */
    EC_TIMEMO      /* TP   TIMEMO, cs 2^42 (CSM)             */
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
static const ec_entry_t k_map_csm[] = { {00306u, 18, EC_XSMD}, {00375u, 3, EC_FAILREG}, {01011u, 1, EC_MODREG}, {01023u, 12, EC_DSPTAB}, {01103u, 2, EC_LAT}, {01105u, 2, EC_LONG}, {01107u, 2, EC_ALT}, {01126u, 1, EC_TRKMKCNT}, {01131u, 2, EC_OPTION1}, {01155u, 3, EC_THETAD}, {01162u, 6, EC_DELV}, {01170u, 6, EC_RN}, {01176u, 6, EC_VN}, {01204u, 2, EC_PIPTIME}, {01320u, 1, EC_IMODES30}, {01321u, 1, EC_IMODES33}, {01330u, 1, EC_MARKSTAT}, {01336u, 2, EC_TEVENT}, {01341u, 1, EC_CDUCHKWD}, {01351u, 2, EC_RTED1}, {01501u, 1, EC_UPSVFLAG}, {01502u, 42, EC_SV_TEMP}, {01554u, 42, EC_SV_CSM}, {01626u, 42, EC_SV_LEM}, {01706u, 3, EC_TEPHEM}, {01735u, 18, EC_REFSMMAT}, {02025u, 6, EC_RLS}, {02640u, 2, EC_T3TOT4}, {03066u, 1, EC_DAPDATR1}, {03067u, 1, EC_DAPDATR2}, {03073u, 1, EC_LEMMASS}, {03074u, 1, EC_CSMMASS}, {03325u, 1, EC_CM_GYMDT}, {03374u, 1, EC_S61DT}, {03376u, 1, EC_P37RANGE}, {03400u, 2, EC_LAT_SPL}, {03402u, 2, EC_LNG_SPL}, {03404u, 6, EC_DELVSLV}, {03412u, 2, EC_TIG}, {03427u, 2, EC_TGO}, {03502u, 7, EC_MARK2DWN}, {03660u, 2, EC_TTOGO}, {03662u, 2, EC_TTPI}, {03672u, 2, EC_P40TMP}, {03674u, 7, EC_MARKDOWN}, {03720u, 6, EC_VGTIG}, {01452u, 15, EC_IMUCOMP}, {01477u, 1, EC_GCOMPSW}, {01711u, 2, EC_AZO}, {01713u, 2, EC_NEG_AYO}, {01715u, 2, EC_AXO}, {02011u, 6, EC_504LM}, {02033u, 3, EC_TIMEMO}, {02036u, 60, EC_VECOEM}, {02132u, 6, EC_RESO}, {02140u, 6, EC_VESO}, {02146u, 2, EC_OMEGAES}, {00110u, 1, EC_EMDOT}, {01353u, 1, EC_DVTHRESH}, {01767u, 2, EC_EK1VAL}, {01771u, 2, EC_EK2VAL}, {01773u, 2, EC_EK3VAL}, {01774u, 1, EC_FANG}, {02017u, 2, EC_EMSALT}, {03007u, 1, EC_LADPAD}, {03010u, 1, EC_LODPAD}, {03011u, 1, EC_ALFAPAD}, {03012u, 1, EC_ETDECAY}, {03013u, 1, EC_ESTROKER}, {03014u, 2, EC_EKPRIME}, {03016u, 3, EC_EKTLX_I}, {03021u, 2, EC_EREPFRAC}, {03023u, 1, EC_PACTOFF}, {03024u, 1, EC_YACTOFF}, {03025u, 15, EC_HBN10}, {01776u, 2, EC_LUNARCOEF}};

/* Luminary099 addresses (ERASABLE_ASSIGNMENTS, from the listing).        */
static const ec_entry_t k_map_lm[] = { {00321u, 3, EC_THETAD}, {00324u, 6, EC_DELV}, {00375u, 3, EC_FAILREG}, {01011u, 1, EC_MODREG}, {01023u, 12, EC_DSPTAB}, {01120u, 2, EC_LAT}, {01122u, 2, EC_LONG}, {01124u, 2, EC_ALT}, {01144u, 3, EC_OPTION1}, {01220u, 6, EC_RN}, {01226u, 6, EC_VN}, {01234u, 2, EC_PIPTIME}, {01302u, 1, EC_IMODES30}, {01303u, 1, EC_IMODES33}, {01312u, 1, EC_MARKSTAT}, {01331u, 1, EC_LEMMASS}, {01332u, 1, EC_CSMMASS}, {01343u, 1, EC_DAPDATR1}, {01344u, 2, EC_TEVENT}, {01501u, 1, EC_UPSVFLAG}, {01502u, 42, EC_SV_TEMP}, {01554u, 42, EC_SV_CSM}, {01626u, 42, EC_SV_LEM}, {01706u, 3, EC_TEPHEM}, {01733u, 18, EC_REFSMMAT}, {02022u, 6, EC_RLS}, {02200u, 2, EC_RR_AZ}, {02202u, 2, EC_RR_ELEV}, {02400u, 2, EC_TLAND}, {03404u, 6, EC_AOTAZ}, {03412u, 6, EC_AOTEL}, {03422u, 1, EC_ZOOMTIME}, {03433u, 6, EC_DELVSLV}, {03441u, 2, EC_TIG}, {03453u, 2, EC_TTOGO}, {03462u, 1, EC_TRKMKCNT}, {03477u, 2, EC_SAVET_30}, {03516u, 2, EC_TGO}, {03606u, 18, EC_XSMD}, {03635u, 2, EC_TTPI}, {03700u, 6, EC_VGTIG}, {01452u, 15, EC_IMUCOMP}, {01477u, 1, EC_GCOMPSW}, {01711u, 2, EC_AZO}, {01713u, 2, EC_NEG_AYO}, {01715u, 2, EC_AXO}, {02012u, 6, EC_504LM}, {01352u, 2, EC_LUNARCOEF}};

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
    else if (item == EC_TEPHEM || item == EC_TIMEMO)
    {
        l = EC_TP;
    }
    else
    {
        /* DP elements */
    }

    return (l);
}

/* IMU compensation pad load, element k in erasable order: PBIASX
 * PIPASCFX PBIASY PIPASCFY PBIASZ PIPASCFZ NBDX NBDY NBDZ ADIAX ADIAY
 * ADIAZ ADSRAX ADSRAY ADSRAZ.                                             */
static agc_sp_t *comp_field(agc_imu_comp_params_t *p, int k)
{
    agc_sp_t *f = &p->adsra[k - 12]; /* ADSRAX..Z */

    if (k < 6)
    {
        f = ((k % 2) == 0) ? &p->pipabias[k / 2] : &p->pipascf[k / 2];
    }
    else if (k < 9)
    {
        f = &p->nbd[k - 6];
    }
    else if (k < 12)
    {
        f = &p->adia[k - 9];
    }
    else
    {
        /* ADSRA */
    }

    return (f);
}

static agc_sp_t comp_get(int k)
{
    agc_imu_comp_params_t p; /* the pad load */

    agc_imu_comp_get(&p);

    return (*comp_field(&p, k));
}

static agc_status_t comp_set(int k, agc_sp_t v)
{
    agc_imu_comp_params_t p; /* the pad load */

    agc_imu_comp_get(&p);
    *comp_field(&p, k) = v;

    return (agc_imu_comp_load(&p));
}

/* E3J22R2M (k = 0), E32C31RM (k = 1).                                    */
static agc_sp_t lunar_coef(int k)
{
    agc_sp_t j22 = 0; /* E3J22R2M */
    agc_sp_t c31 = 0; /* E32C31RM */

    agc_integ_get_lunar_coeffs(&j22, &c31);

    return ((k == 0) ? j22 : c31);
}

static void set_lunar_coef(int k, agc_sp_t v)
{
    agc_sp_t j22 = 0; /* E3J22R2M */
    agc_sp_t c31 = 0; /* E32C31RM */

    agc_integ_get_lunar_coeffs(&j22, &c31);
    agc_integ_set_lunar_coeffs((k == 0) ? v : j22, (k == 1) ? v : c31);
}

/* SP pad loads held in module structs: element k of the item.            */
static bool is_sp_field(ec_item_t item)
{
    return (item >= EC_EMDOT && item <= EC_HBN10);
}

static agc_sp_t *sp_field(ec_item_t item, int k)
{
    agc_csm_dapcom_t  *dc = agc_csm_dap_com();      /* TVC / RCS common */
    agc_guid_csm_t    *gc = &agc_guid_state()->csm; /* Comanche guidance */
    agc_entry_state_t *es = agc_entry_state();      /* entry */
    agc_sp_t          *f  = &dc->hbn10[k];          /* EC_HBN10 */

    switch ((int) item)
    {
        case EC_EMDOT:
            f = &dc->emdot;
            break;
        case EC_DVTHRESH:
            f = &gc->dvthresh;
            break;
        case EC_FANG:
            f = &gc->fang;
            break;
        case EC_LADPAD:
            f = &es->ladpad;
            break;
        case EC_LODPAD:
            f = &es->lodpad;
            break;
        case EC_ALFAPAD:
            f = &es->alfapad;
            break;
        case EC_ETDECAY:
            f = &gc->etdecay;
            break;
        case EC_ESTROKER:
            f = &dc->estroker;
            break;
        case EC_EKPRIME:
            f = &dc->ekprime[k];
            break;
        case EC_EKTLX_I:
            f = &dc->ektlx_i[k];
            break;
        case EC_EREPFRAC:
            f = &dc->erepfrac[k];
            break;
        case EC_PACTOFF:
            f = &dc->pactoff;
            break;
        case EC_YACTOFF:
            f = &dc->yactoff;
            break;
        default:
            break; /* EC_HBN10 */
    }

    return (f);
}

/* DP pad loads held in module structs.                                    */
static bool is_dp_field(ec_item_t item)
{
    return (item >= EC_EK1VAL && item <= EC_EMSALT);
}

static agc_dp_t *dp_field(ec_item_t item)
{
    agc_guid_csm_t *gc = &agc_guid_state()->csm;     /* Comanche guidance */
    agc_dp_t       *f  = &agc_entry_state()->emsalt; /* EC_EMSALT */

    if (item == EC_EK1VAL)
    {
        f = &gc->ek1val;
    }
    else if (item == EC_EK2VAL)
    {
        f = &gc->ek2val;
    }
    else if (item == EC_EK3VAL)
    {
        f = &gc->ek3val;
    }
    else
    {
        /* EMSALT */
    }

    return (f);
}

/* SP items: the value of element k.                                       */
static agc_sp_t sp_get(ec_item_t item, int k)
{
    agc_sp_t v = is_sp_field(item) ? *sp_field(item, k) : 0; /* the value */

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
        case EC_IMUCOMP:
            v = comp_get(k);
            break;
        case EC_GCOMPSW:
            v = agc_imu_gcompsw();
            break;
        case EC_LUNARCOEF:
            v = lunar_coef(k);
            break;
        case EC_UPSVFLAG:
            v = agc_erase_upsvflag();
            break;
        default:
            break; /* the struct fields: read above */
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
        case EC_IMUCOMP:
            st                 = comp_set(k, v);
            break;
        case EC_GCOMPSW:
            st                 = agc_imu_set_gcompsw(v);
            break;
        case EC_LUNARCOEF:
            set_lunar_coef(k, v);
            break;
        case EC_UPSVFLAG:
            st                 = agc_erase_set_upsvflag(v);
            break;
        default:
            *sp_field(item, k) = v; /* the struct fields */
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
        default:
            v = agc_erase_savet_30(); /* EC_SAVET_30 */
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
        default:
            agc_erase_set_savet_30(v); /* EC_SAVET_30 */
            break;
    }
}

/* LAT, LONG, ALT (one store, agc_navaux_*_latlong).                      */
static agc_dp_t *latlong_field(agc_latlong_t *ll, ec_item_t item)
{
    agc_dp_t *f = &ll->alt; /* EC_ALT */

    if (item == EC_LAT)
    {
        f = &ll->lat;
    }
    else if (item == EC_LONG)
    {
        f = &ll->lon;
    }
    else
    {
        /* ALT */
    }

    return (f);
}

static bool is_latlong(ec_item_t item)
{
    return (item == EC_LAT || item == EC_LONG || item == EC_ALT);
}

/* TIMEMO (the ephemeris' centre time).                                    */
static agc_tp_t tp_timemo(void)
{
    agc_ephem_csm_t eph; /* the ephemeris */

    agc_ephem_get_csm_data(&eph);

    return (eph.timemo);
}

static void tp_set_timemo(agc_tp_t t)
{
    agc_ephem_csm_t eph; /* the ephemeris */

    agc_ephem_get_csm_data(&eph);
    eph.timemo = t;
    agc_ephem_set_csm_data(&eph);
}

/* Launch pad load (AZO, -AYO, AXO, 504LM) and the CSM ephemeris
 * (VECOEM, RESO, VESO, OMEGAES): DP values of the item, in erasable order. */
static bool is_group(ec_item_t item)
{
    return (item >= EC_AZO && item <= EC_OMEGAES);
}

static void group_vec(agc_vec_t v, agc_dp_t *d)
{
    for (int k = 0; k < AGC_VEC_N; k++)
    {
        d[k] = v.c[k];
    }
}

static void group_get(ec_item_t item, agc_dp_t *d)
{
    agc_navaux_padload_t pad; /* AZO .. 504LM */
    agc_ephem_csm_t      eph; /* VECOEM .. OMEGAES */

    agc_navaux_get_padload(&pad);
    agc_ephem_get_csm_data(&eph);
    d[0] = (item == EC_AZO) ? pad.azo : (item == EC_NEG_AYO) ? pad.neg_ayo : (item == EC_AXO) ? pad.axo : eph.omegaes; /* the DP items */

    if (item == EC_504LM)
    {
        group_vec(pad.lm504, d); /* 3 DPs */
    }
    else if (item == EC_RESO || item == EC_VESO)
    {
        group_vec((item == EC_RESO) ? eph.reso : eph.veso, d); /* 3 DPs */
    }
    else if (item == EC_VECOEM)
    {
        for (int i = 0; i < AGC_EPHEM_NCOEF; i++)
        {
            group_vec(eph.vecoem[i], &d[AGC_VEC_N * i]); /* 3 DPs each */
        }
    }
    else
    {
        /* one DP, set above */
    }
}

static agc_vec_t group_of(const agc_dp_t *d)
{
    agc_vec_t v; /* the vector */

    for (int k = 0; k < AGC_VEC_N; k++)
    {
        v.c[k] = d[k];
    }

    return (v);
}

static void group_put(ec_item_t item, const agc_dp_t *d)
{
    agc_navaux_padload_t pad; /* AZO .. 504LM */
    agc_ephem_csm_t      eph; /* VECOEM .. OMEGAES */

    agc_navaux_get_padload(&pad);
    agc_ephem_get_csm_data(&eph);
    pad.azo     = (item == EC_AZO) ? d[0] : pad.azo;
    pad.neg_ayo = (item == EC_NEG_AYO) ? d[0] : pad.neg_ayo;
    pad.axo     = (item == EC_AXO) ? d[0] : pad.axo;
    pad.lm504   = (item == EC_504LM) ? group_of(d) : pad.lm504;
    eph.reso    = (item == EC_RESO) ? group_of(d) : eph.reso;
    eph.veso    = (item == EC_VESO) ? group_of(d) : eph.veso;
    eph.omegaes = (item == EC_OMEGAES) ? d[0] : eph.omegaes;

    for (int i = 0; i < AGC_EPHEM_NCOEF && item == EC_VECOEM; i++)
    {
        eph.vecoem[i] = group_of(&d[AGC_VEC_N * i]);
    }

    if (item >= EC_VECOEM)
    {
        agc_ephem_set_csm_data(&eph);
    }
    else
    {
        agc_navaux_set_padload(&pad);
    }
}

/* LAT(SPL), LNG(SPL): the CSM entry target (agc_entry_state).            */
static bool is_splash(ec_item_t item)
{
    return (item == EC_LAT_SPL || item == EC_LNG_SPL);
}

static agc_dp_t *splash_field(ec_item_t item)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    return ((item == EC_LAT_SPL) ? &es->lat_spl : &es->lng_spl);
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
        case EC_DELV:
            v = agc_erase_delv();
            break;
        case EC_VGTIG:
            v = agc_erase_vgtig();
            break;
        case EC_RLS:
            v = agc_erase_rls();
            break;
        default:
            v = agc_rendz_erasable()->delvlvc; /* EC_DELVSLV */
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
        case EC_DELV:
            agc_erase_set_delv(v);
            break;
        case EC_VGTIG:
            agc_erase_set_vgtig(v);
            break;
        case EC_RLS:
            agc_erase_set_rls(v);
            break;
        default:
            agc_rendz_erasable()->delvlvc = v; /* EC_DELVSLV */
            break;
    }
}

/* State vector blocks (RRECT, VRECT, TET, DELTAV, NUV, RCV, VCV, TC, XKEP:
 * the 42-word layout of the temporary and the two permanent sets).       */
static void sv_dps(const agc_integ_sv_t *sv, agc_dp_t *d)
{
    const agc_vec_t *v[6]  = {&sv->rrect, &sv->vrect, &sv->deltav, &sv->nuv, &sv->rcv, &sv->vcv}; /* the vectors */
    const int        at[6] = {0, 3, 7, 10, 13, 16};                                               /* their first DP */

    for (int i = 0; i < 6; i++)
    {
        for (int k = 0; k < AGC_VEC_N; k++)
        {
            d[at[i] + k] = v[i]->c[k];
        }
    }

    d[6]  = sv->tet;
    d[19] = sv->tc;
    d[20] = sv->xkep;
}

static void sv_of_dps(const agc_dp_t *d, agc_integ_sv_t *sv)
{
    agc_vec_t *v[6]  = {&sv->rrect, &sv->vrect, &sv->deltav, &sv->nuv, &sv->rcv, &sv->vcv}; /* the vectors */
    const int  at[6] = {0, 3, 7, 10, 13, 16};                                               /* their first DP */

    for (int i = 0; i < 6; i++)
    {
        for (int k = 0; k < AGC_VEC_N; k++)
        {
            v[i]->c[k] = d[at[i] + k];
        }
    }

    sv->tet  = d[6];
    sv->tc   = d[19];
    sv->xkep = d[20];
}

static bool is_sv(ec_item_t item)
{
    return (item == EC_SV_TEMP || item == EC_SV_CSM || item == EC_SV_LEM);
}

static void sv_get(ec_item_t item, agc_integ_sv_t *sv)
{
    if (item == EC_SV_TEMP)
    {
        *sv = agc_integ_shared_ctx()->sv;
    }
    else
    {
        agc_integ_perm_get((item == EC_SV_CSM) ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM, sv);
    }
}

static void sv_put(ec_item_t item, const agc_integ_sv_t *sv)
{
    if (item == EC_SV_TEMP)
    {
        agc_integ_shared_ctx()->sv = *sv;
    }
    else
    {
        agc_integ_perm_set((item == EC_SV_CSM) ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM, sv);
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
        agc_tp_words_t t = agc_tp_to_words((e->item == EC_TIMEMO) ? tp_timemo() : agc_erase_tephem()); /* TEPHEM / TIMEMO */

        w[0] = word_of_sp(t.hi);
        w[1] = word_of_sp(t.mid);
        w[2] = word_of_sp(t.lo);
    }
    else if (e->item == EC_REFSMMAT || e->item == EC_XSMD)
    {
        agc_mat_t m = (e->item == EC_XSMD) ? agc_align_ram()->xsmd : agc_erase_refsmmat(); /* rows */

        for (int r = 0; r < AGC_VEC_N; r++)
        {
            vec_to(m.row[r], &w[EC_DP_WORDS * AGC_VEC_N * r]);
        }
    }
    else if (is_latlong(e->item))
    {
        agc_latlong_t ll; /* LAT, LONG, ALT */

        agc_navaux_get_latlong(&ll);
        dp_to(*latlong_field(&ll, e->item), w);
    }
    else if (is_splash(e->item))
    {
        dp_to(*splash_field(e->item), w);
    }
    else if (is_dp_field(e->item))
    {
        dp_to(*dp_field(e->item), w);
    }
    else if (is_group(e->item))
    {
        agc_dp_t d[EC_MAX_WORDS / EC_DP_WORDS]; /* the DP values */

        group_get(e->item, d);

        for (int k = 0; k < e->words / EC_DP_WORDS; k++)
        {
            dp_to(d[k], &w[EC_DP_WORDS * k]);
        }
    }
    else if (is_sv(e->item))
    {
        agc_integ_sv_t sv;           /* the block */
        agc_dp_t       d[EC_SV_DPS]; /* its DP values */

        sv_get(e->item, &sv);
        sv_dps(&sv, d);

        for (int k = 0; k < EC_SV_DPS; k++)
        {
            dp_to(d[k], &w[EC_DP_WORDS * k]);
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
        agc_tp_words_t t = {sp_of_word(w[0]), sp_of_word(w[1]), sp_of_word(w[2])}; /* TEPHEM / TIMEMO words */

        if (e->item == EC_TIMEMO)
        {
            tp_set_timemo(agc_tp_from_words(t));
        }
        else
        {
            agc_erase_set_tephem(agc_tp_from_words(t));
        }
    }
    else if (e->item == EC_REFSMMAT || e->item == EC_XSMD)
    {
        agc_mat_t m; /* rows */

        for (int r = 0; r < AGC_VEC_N; r++)
        {
            m.row[r] = vec_of(&w[EC_DP_WORDS * AGC_VEC_N * r]);
        }

        if (e->item == EC_XSMD)
        {
            agc_align_ram()->xsmd = m;
        }
        else
        {
            agc_erase_set_refsmmat(m);
        }
    }
    else if (is_latlong(e->item))
    {
        agc_latlong_t ll; /* LAT, LONG, ALT */

        agc_navaux_get_latlong(&ll);
        *latlong_field(&ll, e->item) = dp_of(w);
        agc_navaux_set_latlong(&ll);
    }
    else if (is_splash(e->item))
    {
        *splash_field(e->item) = dp_of(w);
    }
    else if (is_dp_field(e->item))
    {
        *dp_field(e->item) = dp_of(w);
    }
    else if (is_group(e->item))
    {
        agc_dp_t d[EC_MAX_WORDS / EC_DP_WORDS]; /* the DP values */

        for (int k = 0; k < e->words / EC_DP_WORDS; k++)
        {
            d[k] = dp_of(&w[EC_DP_WORDS * k]);
        }

        group_put(e->item, d);
    }
    else if (is_splash(e->item))
    {
        *splash_field(e->item) = dp_of(w);
    }
    else if (is_dp_field(e->item))
    {
        *dp_field(e->item) = dp_of(w);
    }
    else if (is_group(e->item))
    {
        agc_dp_t d[EC_MAX_WORDS / EC_DP_WORDS]; /* the DP values */

        for (int k = 0; k < e->words / EC_DP_WORDS; k++)
        {
            d[k] = dp_of(&w[EC_DP_WORDS * k]);
        }

        group_put(e->item, d);
    }
    else if (is_sv(e->item))
    {
        agc_integ_sv_t sv;           /* the block */
        agc_dp_t       d[EC_SV_DPS]; /* its DP values */

        for (int k = 0; k < EC_SV_DPS; k++)
        {
            d[k] = dp_of(&w[EC_DP_WORDS * k]);
        }

        sv_of_dps(d, &sv);
        sv_put(e->item, &sv);
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

/* Words per element of a layout.                                         */
static int element_words(ec_layout_t l)
{
    int n = EC_DP_WORDS; /* DP */

    if (l == EC_SP || l == EC_RAW)
    {
        n = 1;
    }
    else if (l == EC_TP)
    {
        n = EC_TP_WORDS;
    }
    else
    {
        /* DP elements */
    }

    return (n);
}

/* Whether two word images of one element hold the same value.            */
static bool same_element(ec_layout_t l, const uint16_t *a, const uint16_t *b)
{
    bool same = a[0] == b[0]; /* RAW: the bits */

    if (l == EC_SP)
    {
        same = sp_of_word(a[0]) == sp_of_word(b[0]);
    }
    else if (l == EC_TP)
    {
        agc_tp_words_t ta = {sp_of_word(a[0]), sp_of_word(a[1]), sp_of_word(a[2])}; /* a */
        agc_tp_words_t tb = {sp_of_word(b[0]), sp_of_word(b[1]), sp_of_word(b[2])}; /* b */

        same = agc_tp_from_words(ta) == agc_tp_from_words(tb);
    }
    else if (l == EC_DP)
    {
        same = dp_of(a) == dp_of(b);
    }
    else
    {
        /* RAW */
    }

    return (same);
}

/* The map entry holding ecadr, or NULL.                                   */
static const ec_entry_t *map_find(uint16_t ecadr)
{
    bool              lm  = agc_state_vehicle() == AGC_VEHICLE_LM;                                                      /* which map */
    const ec_entry_t *map = lm ? k_map_lm : k_map_csm;                                                                  /* the map */
    size_t            n   = lm ? sizeof (k_map_lm) / sizeof (k_map_lm[0]) : sizeof (k_map_csm) / sizeof (k_map_csm[0]); /* entries */
    const ec_entry_t *hit = NULL;                                                                                       /* found */

    for (size_t i = 0; i < n && hit == NULL; i++)
    {
        if (ecadr >= map[i].ecadr && ecadr < map[i].ecadr + (uint16_t) map[i].words)
        {
            hit = &map[i];
        }
    }

    return (hit);
}

/* Bring the raw words of a mapped item up to date with its typed value,
 * element by element (an unchanged element keeps its words as stored).   */
static void refresh(const ec_entry_t *e)
{
    uint16_t w[EC_MAX_WORDS]; /* typed value as words */

    ec_layout_t l = item_layout(e->item); /* element layout */
    int         n = element_words(l);     /* words per element */

    item_words(e, w);

    for (int k = 0; k + n <= e->words; k += n)
    { /* only the elements whose value changed take the sign-agreeing words */
        if (!same_element(l, &g_image[e->ecadr + (uint16_t) k], &w[k]))
        {
            for (int j = k; j < k + n; j++)
            {
                g_image[e->ecadr + (uint16_t) j] = w[j];
            }
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
