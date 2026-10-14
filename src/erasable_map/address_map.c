/**********************************************************************
 * FILE NAME: address_map.c
 *
 * PURPOSE:
 *   Erasable access by AGC address (ECADR), for the machine-address
 *   nouns of pinball (nouns 01-03, 07, 15: V01/V21/V25 of any
 *   erasable), the uplink update program (V71/V72), the downlink
 *   erasable dump, and hosts that see the AGC as an addressable memory
 *   (src/nassp: NASSP reads and pad-loads erasable by address). The C
 *   translation keeps each erasable in its owner's typed storage, not in
 *   an addressable array; this file maps addresses onto that storage.
 *
 * AGC SOURCE:
 *   none (the addresses are Comanche055 / Luminary099
 *   ERASABLE_ASSIGNMENTS.agc, from the assembly listings)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type        I/O  Scale          Description
 *   ------------  ----------  ---  -------------  ----------------------
 *   FLAGWRD0-13   uint16_t    I/O  bits           agc_flagword_get/set
 *   (k_map_csm,   per item    I/O  each item's    REFSMMAT, TEPHEM, state
 *    k_map_lm)                     own scale      vectors, pad loads, ...
 *                                                 through their owners'
 *                                                 accessors
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_erase_raw_read    the raw word image (src/core/ecadr.c)
 *   agc_erase_raw_write   store into the raw word image
 *   agc_integ_perm_get    the permanent state vector sets (nav)
 *   agc_lm_pad            the LM landing / abort pad loads
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   agc_erase_write_ecadr returns a setter's status when the typed
 *   owner refuses the value (e.g. UPSVFLAG out of range); the old value
 *   stays.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   The vehicle (agc_state_vehicle) selects the CSM or the LM map.
 *
 * NOTES:
 *   Three kinds of address:
 *   - the flagwords (STATE +0.., ECADR 074 in both ropes, so FLAGWRD0..
 *     FLGWRD13 = 074..0111) map onto agc_flagword_get/set;
 *   - the named erasables in the address map below (k_map_csm / k_map_lm)
 *     map onto their typed accessors: REFSMMAT, TEPHEM, the state vectors,
 *     the uplink buffer (UPSVFLAG and the RRECT..XKEP block) and the two
 *     permanent state vector blocks (RRECTCSM.., RRECTLEM..), the external
 *     dV (DELVSLV) and desired REFSMMAT (XSMD), LAT/LONG/ALT, LAT(SPL)/
 *     LNG(SPL), the LM landing and abort pad loads, times, masses, DAP
 *     words, display registers;
 *   - every other address reads and writes a raw image of the 15-bit
 *     address space (0000-77777: erasable E0..E7 at 0000-3777, then the
 *     fixed-memory pseudo addresses, which hold no rope here and simply
 *     keep what was stored).
 *   For a mapped erasable the raw image keeps the words last stored, so a
 *   DP or TP quantity loaded one word at a time (V71 stores from the last
 *   word to the first) keeps the words exactly as given, signs free to
 *   disagree, and the typed value is rebuilt from all its words after each
 *   store. When the program has changed the typed value since, the words
 *   are refreshed from it (sign-agreeing) before a read or a store.
 *   DEVIATION: an address that is neither a flagword nor in the map does
 *   not reach any typed erasable of the same name; the raw image keeps the
 *   value so a later V01/V05 of the same address reads it.
 *   Layering: the map reaches typed storage in every subsystem (core to
 *   csm/lm), so it sits above them all; the raw image itself is core's
 *   (src/core/ecadr.c, agc_erase_raw_read / agc_erase_raw_write).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/erasable_map/address_map.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define EC_STATE     0074u   /* STATE = FLAGWRD0 (both ropes, unswitched)   */
#define EC_WORD_MAX  077777u /* one AGC word (15 bits)                  */
#define EC_SIGN      040000u /* ones'-complement sign bit                */
#define EC_MAG       037777u /* ones'-complement magnitude               */
#define EC_MAX_WORDS 60      /* longest mapped erasable: VECOEM          */
#define EC_SV_DPS    21      /* DP values in a state vector block         */
#define EC_DP_WORDS  2       /* words in a DP element                    */
#define EC_TP_WORDS  3       /* words in a TP element                    */

/* Named erasables with a typed home.                                      */
typedef enum ec_item
{
    EC_MODREG = 0,   /* SP   MODREG                            */
    EC_CSMMASS,      /* SP   CSMMASS, kg 2^16                  */
    EC_LEMMASS,      /* SP   LEMMASS, kg 2^16                  */
    EC_OPTION1,      /* SP x3 OPTION1..3 (CSM OPTION1..2)      */
    EC_TRKMKCNT,     /* SP   TRKMKCNT                          */
    EC_CDUCHKWD,     /* SP   CDUCHKWD (CSM)                    */
    EC_P37RANGE,     /* SP   P37RANGE (CSM)                    */
    EC_ZOOMTIME,     /* SP   ZOOMTIME (LM)                     */
    EC_S61DT,        /* SP   S61DT (CSM)                       */
    EC_CM_GYMDT,     /* SP   CM/GYMDT (CSM)                    */
    EC_IMUCOMP,      /* SP x15 PBIASX PIPASCFX .. ADSRAZ (IMU compensation pad load) */
    EC_GCOMPSW,      /* SP   GCOMPSW                           */
    EC_LUNARCOEF,    /* SP x2 E3J22R2M, E32C31RM (lunar potential) */
    EC_EMDOT,        /* SP   EMDOT SPS flow (CSM pad load)     */
    EC_DVTHRESH,     /* SP   DVTHRESH (CSM)                    */
    EC_FANG,         /* SP   FANG SPS thrust (CSM)             */
    EC_LADPAD,       /* SP   LADPAD (CSM entry)                */
    EC_LODPAD,       /* SP   LODPAD (CSM entry)                */
    EC_ALFAPAD,      /* SP   ALFAPAD (CSM entry)               */
    EC_ETDECAY,      /* SP   ETDECAY (CSM)                     */
    EC_ESTROKER,     /* SP   ESTROKER (CSM TVC)                */
    EC_EKPRIME,      /* SP x2 EKPRIME (CSM TVC)                */
    EC_EKTLX_I,      /* SP x3 EKTLX/I (CSM TVC)                */
    EC_EREPFRAC,     /* SP x2 EREPFRAC (CSM TVC)               */
    EC_PACTOFF,      /* SP   PACTOFF (CSM)                     */
    EC_YACTOFF,      /* SP   YACTOFF (CSM)                     */
    EC_HBN10,        /* SP x15 HBN10 .. HBD32 (CSM TVC filter) */
    EC_CSM_INERTIA,  /* SP x3 IXX, IAVG, IAVG/TLX (CSM MASSPROP, DAPs) */
    EC_CSM_HOLDFLAG, /* SP  HOLDFLAG (CSM RCS DAP)              */
    EC_LM_HIASCENT,  /* SP   HIASCENT (LM DAP pad load)        */
    EC_LM_ROLLPIT,   /* SP x2 ROLLTIME, PITTIME (LM)           */
    EC_LM_DAPGAINS,  /* SP x7 DKTRAP DKOMEGAN DKKAOSN LMTRAP LMOMEGAN LMKAOSN DKDB (LM) */
    EC_LM_IGNAOS,    /* SP x2 IGNAOSQ, IGNAOSR (LM)            */
    EC_LM_SKALSKAL,  /* SP   SKALSKAL (LM landing radar)       */
    EC_LM_LRPAD,     /* SP x13 LRALPHA LRBETA1 LRALPHA2 LRBETA2 LRVMAX LRVF LRWVZ/Y/X LRWVFZ/Y/X LRWVFF (LM) */
    EC_LM_LRHMAX,    /* SP x2 LRHMAX, LRWH (LM)                */
    EC_LM_RPCRT,     /* SP x2 RPCRTIME, RPCRTQSW (LM)          */
    EC_LM_NAVPAD,    /* SP x10 WRENDPOS WRENDVEL WSHAFT WTRUN RMAX VMAX WSURFPOS WSURFVEL SHAFTVAR TRUNVAR (LM) */
    EC_LM_VARMIN,    /* SP x2 RVARMIN, VVARMIN (LM)            */
    EC_LM_BRK_SP,    /* SP x2 TCGFBRAK, TCGIBRAK (LM)           */
    EC_LM_APP_SP,    /* SP x2 TCGFAPPR, TCGIAPPR (LM)           */
    EC_LM_CRIT,      /* SP x2 LOWCRIT, HIGHCRIT (LM)            */
    EC_LM_RODSCALE,  /* SP  RODSCALE (LM)                       */
    EC_LM_TEND,      /* SP x4 TENDBRAK TENDAPPR DELTTFAP LEADTIME (LM) */
    EC_UPSVFLAG,     /* SP   UPSVFLAG                          */
    EC_DAPDATR1,     /* RAW  DAPDATR1                          */
    EC_DAPDATR2,     /* RAW  DAPDATR2 (CSM)                    */
    EC_MARKSTAT,     /* RAW  MARKSTAT                          */
    EC_IMODES30,     /* RAW  IMODES30                          */
    EC_IMODES33,     /* RAW  IMODES33                          */
    EC_DSPTAB,       /* RAW x12 DSPTAB                         */
    EC_FAILREG,      /* RAW x3 FAILREG                         */
    EC_THETAD,       /* RAW x3 THETAD                          */
    EC_AOTAZ,        /* RAW x6 AOTAZ (LM)                      */
    EC_AOTEL,        /* RAW x6 AOTEL (LM)                      */
    EC_MARKDOWN,     /* RAW x7 MARKDOWN (CSM)                  */
    EC_PHASES,       /* RAW x12 -PHASE1 PHASE1 .. -PHASE6 PHASE6 */
    EC_TBASES,       /* RAW x12 TBASE1 PHSPRDT1 .. (PHSPRDTn not kept) */
    EC_LONGBASE,     /* RAW x2 LONGBASE (TIME2, TIME1)         */
    EC_LM_ASC_WRD,   /* RAW x16 Y DRDOT DYDOT DZDOT PCONS YCONS PRATE YRATE (LM ascent) */
    EC_LM_ASC_GT,    /* RAW x2 TBUP or GEFF as stored (LM ascent): k 0 hi, 1 lo */
    EC_LM_ASC_GEFF,  /* RAW x2 GEFF as stored (LM ascent)      */
    EC_CSM_THETADX,  /* RAW x3 THETADX..Z hold reference (CSM RCS DAP) */
    EC_MARK2DWN,     /* RAW x7 MARK2DWN (CSM)                  */
    EC_PIPTIME,      /* DP   PIPTIME, cs 2^28                  */
    EC_TIG,          /* DP   TIG                               */
    EC_TTOGO,        /* DP   TTOGO                             */
    EC_TGO,          /* DP   TGO                               */
    EC_TTPI,         /* DP   TTPI                              */
    EC_TEVENT,       /* DP   TEVENT                            */
    EC_T3TOT4,       /* DP   T3TOT4 (CSM)                      */
    EC_RR_AZ,        /* DP   RR-AZ (LM)                        */
    EC_RR_ELEV,      /* DP   RR-ELEV (LM)                      */
    EC_RTED1,        /* DP   RTED1 (CSM)                       */
    EC_TLAND,        /* DP   TLAND (LM)                        */
    EC_P40TMP,       /* DP   P40TMP (CSM)                      */
    EC_SAVET_30,     /* DP   SAVET-30 (LM)                     */
    EC_LM_MASS,      /* DP   MASS, kg 2^16 (LM)                */
    EC_LAT,          /* DP   LAT, revs 2^0 (noun 43)           */
    EC_LONG,         /* DP   LONG, revs 2^0                    */
    EC_ALT,          /* DP   ALT, m 2^29                       */
    EC_AZO,          /* DP   AZO, revs (launch pad load)       */
    EC_NEG_AYO,      /* DP   -AYO, rad                         */
    EC_AXO,          /* DP   AXO, rad                          */
    EC_504LM,        /* DP x3 504LM, rad                       */
    EC_VECOEM,       /* DP x30 VECOEM lunar ephemeris (CSM)    */
    EC_RESO,         /* DP x3 RESO (CSM)                       */
    EC_VESO,         /* DP x3 VESO (CSM)                       */
    EC_OMEGAES,      /* DP   OMEGAES (CSM)                     */
    EC_LM_BRK_DP,    /* DP x13 RBRFG VBRFG ABRFG VBRFG* ABRFG* JBRFG* GAINBRAK (LM) */
    EC_LM_APP_DP,    /* DP x13 RAPFG .. GAINAPPR (LM)          */
    EC_LM_IGN,       /* DP x6 VIGN RIGNX RIGNZ KIGNX/B4 KIGNY/B8 KIGNV/B4 (LM) */
    EC_LM_V2FG,      /* DP x4 V2FG, TAUVERT (LM)               */
    EC_LM_ABORT,     /* DP x17 TAUROD LAG/TAU MINFORCE MAXFORCE ABTCOF VMIN YLIM ABTRDOT COSTHET1/2 (LM) */
    EC_LM_ASC_ENG,   /* DP x3 AT VE TTO (LM ascent engine data) */
    EC_LM_ASC_TGT,   /* DP x4 RDOTD YDOTD ZDOTD /R/MAG (LM ascent) */
    EC_LM_ASC_ACC,   /* DP x3 ATY ATR ATP (LM ascent thrust components) */
    EC_LM_ASC_RCO,   /* DP x2 RCO YCO (LM ascent targets)       */
    EC_CSM_DELCDU,   /* DP x3 DELCDUX..Z (CSM RCS DAP steering) */
    EC_CSM_CDUD,     /* DP x3 CDUXD..ZD (CSM RCS DAP steering)  */
    EC_EK1VAL,       /* DP   EK1VAL (CSM)                      */
    EC_EK2VAL,       /* DP   EK2VAL (CSM)                      */
    EC_EK3VAL,       /* DP   EK3VAL (CSM)                      */
    EC_LM_RADSKAL,   /* DP   RADSKAL (LM landing radar)        */
    EC_LM_DELQFIX,   /* DP   DELQFIX (LM landing radar)        */
    EC_LM_TNEWA,     /* DP   TNEWA (LM)                        */
    EC_LM_RANGEVAR,  /* DP   RANGEVAR (LM RR variance)         */
    EC_LM_RATEVAR,   /* DP   RATEVAR (LM RR variance)          */
    EC_LM_ATIGINC,   /* DP   ATIGINC (LM)                      */
    EC_LM_PTIGINC,   /* DP   PTIGINC (LM)                      */
    EC_LM_HDOTDISP,  /* DP   HDOTDISP = RDOT (LM SERVICER, ascent) */
    EC_EMSALT,       /* DP   EMSALT (CSM entry)                */
    EC_LAT_SPL,      /* DP   LAT(SPL) entry target, revs (CSM) */
    EC_LNG_SPL,      /* DP   LNG(SPL) entry target, revs (CSM) */
    EC_RN,           /* VEC  RN                                */
    EC_VN,           /* VEC  VN                                */
    EC_DELV,         /* VEC  DELV                              */
    EC_VGTIG,        /* VEC  VGTIG                             */
    EC_RLS,          /* VEC  RLS                               */
    EC_DELVSLV,      /* VEC  DELVSLV = DELVLVC (external dV)   */
    EC_X789,         /* VEC  X789 (W-matrix states 7-9)         */
    EC_LM_UNFC,      /* VEC  UNFC/2 thrust command (LM)        */
    EC_LM_UNWC,      /* VEC  UNWC/2 window command (LM)        */
    EC_LM_VGVECT,    /* VEC  VGVECT ascent velocity to be gained (LM) */
    EC_LM_LAXIS,     /* VEC  LAXIS ascent cross-range axis (LM) */
    EC_LM_QAXIS,     /* VEC  QAXIS = UHYP (LM)                 */
    EC_LM_ZAXIS1,    /* VEC  ZAXIS1 = UHZP (LM)                */
    EC_UNITW,        /* VEC  UNITW Earth polar axis (CM; -AYO, AXO its first words) */
    EC_SV_TEMP,      /* SV   RRECT..XKEP (uplink / integration buffer, 42 words) */
    EC_SV_CSM,       /* SV   RRECTCSM..XKEPCSM (42 words)       */
    EC_SV_LEM,       /* SV   RRECTLEM..XKEPLEM (42 words)       */
    EC_XSMD,         /* MAT  XSMD.. desired REFSMMAT, 2^1      */
    EC_REFSMMAT,     /* MAT  REFSMMAT, 2^1                     */
    EC_TEPHEM,       /* TP   TEPHEM, cs 2^42                   */
    EC_TIMEMO        /* TP   TIMEMO, cs 2^42 (CSM)             */
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

/* Comanche055 addresses (ERASABLE_ASSIGNMENTS, from the listing). The
 * first entry holding an address wins: FANG (1774) precedes EK3VAL, whose
 * DP K3VAL (S40.13 DDV) is EK3VAL with FANG as its low word, so a FANG
 * pad load reaches both (sp_set).                                         */
static const ec_entry_t k_map_csm[] = { {00306u, 18, EC_XSMD}, {00375u, 3, EC_FAILREG}, {01011u, 1, EC_MODREG}, {01023u, 12, EC_DSPTAB}, {01103u, 2, EC_LAT}, {01105u, 2, EC_LONG}, {01107u, 2, EC_ALT}, {01126u, 1, EC_TRKMKCNT}, {01131u, 2, EC_OPTION1}, {01155u, 3, EC_THETAD}, {01162u, 6, EC_DELV}, {01170u, 6, EC_RN}, {01176u, 6, EC_VN}, {01204u, 2, EC_PIPTIME}, {01320u, 1, EC_IMODES30}, {01321u, 1, EC_IMODES33}, {01330u, 1, EC_MARKSTAT}, {01336u, 2, EC_TEVENT}, {01341u, 1, EC_CDUCHKWD}, {01351u, 2, EC_RTED1}, {01501u, 1, EC_UPSVFLAG}, {01502u, 42, EC_SV_TEMP}, {01554u, 42, EC_SV_CSM}, {01626u, 42, EC_SV_LEM}, {01706u, 3, EC_TEPHEM}, {01735u, 18, EC_REFSMMAT}, {02025u, 6, EC_RLS}, {02640u, 2, EC_T3TOT4}, {03066u, 1, EC_DAPDATR1}, {03067u, 1, EC_DAPDATR2}, {03073u, 1, EC_LEMMASS}, {03074u, 1, EC_CSMMASS}, {03325u, 1, EC_CM_GYMDT}, {03374u, 1, EC_S61DT}, {03376u, 1, EC_P37RANGE}, {03400u, 2, EC_LAT_SPL}, {03402u, 2, EC_LNG_SPL}, {03404u, 6, EC_DELVSLV}, {03412u, 2, EC_TIG}, {03427u, 2, EC_TGO}, {03502u, 7, EC_MARK2DWN}, {03660u, 2, EC_TTOGO}, {03662u, 2, EC_TTPI}, {03672u, 2, EC_P40TMP}, {03674u, 7, EC_MARKDOWN}, {03720u, 6, EC_VGTIG}, {01452u, 15, EC_IMUCOMP}, {01477u, 1, EC_GCOMPSW}, {01711u, 2, EC_AZO}, {01713u, 6, EC_UNITW}, {01713u, 2, EC_NEG_AYO}, {01715u, 2, EC_AXO}, {02011u, 6, EC_504LM}, {02033u, 3, EC_TIMEMO}, {02036u, 60, EC_VECOEM}, {02132u, 6, EC_RESO}, {02140u, 6, EC_VESO}, {02146u, 2, EC_OMEGAES}, {00110u, 1, EC_EMDOT}, {01353u, 1, EC_DVTHRESH}, {01767u, 2, EC_EK1VAL}, {01771u, 2, EC_EK2VAL}, {01774u, 1, EC_FANG}, {01773u, 2, EC_EK3VAL}, {02017u, 2, EC_EMSALT}, {03007u, 1, EC_LADPAD}, {03010u, 1, EC_LODPAD}, {03011u, 1, EC_ALFAPAD}, {03012u, 1, EC_ETDECAY}, {03013u, 1, EC_ESTROKER}, {03014u, 2, EC_EKPRIME}, {03016u, 3, EC_EKTLX_I}, {03021u, 2, EC_EREPFRAC}, {03023u, 1, EC_PACTOFF}, {03024u, 1, EC_YACTOFF}, {03025u, 15, EC_HBN10}, {01776u, 2, EC_LUNARCOEF}, {00752u, 12, EC_PHASES}, {01053u, 12, EC_TBASES}, {01135u, 2, EC_LONGBASE}, {03070u, 3, EC_CSM_INERTIA}, {01332u, 1, EC_CSM_HOLDFLAG}, {03172u, 3, EC_CSM_THETADX}, {03175u, 6, EC_CSM_DELCDU}, {03246u, 6, EC_CSM_CDUD}};

/* Luminary099 addresses (ERASABLE_ASSIGNMENTS, from the listing).        */
static const ec_entry_t k_map_lm[] = { {00321u, 3, EC_THETAD}, {00324u, 6, EC_DELV}, {00375u, 3, EC_FAILREG}, {01011u, 1, EC_MODREG}, {01023u, 12, EC_DSPTAB}, {01120u, 2, EC_LAT}, {01122u, 2, EC_LONG}, {01124u, 2, EC_ALT}, {01144u, 3, EC_OPTION1}, {01220u, 6, EC_RN}, {01226u, 6, EC_VN}, {01234u, 2, EC_PIPTIME}, {01302u, 1, EC_IMODES30}, {01303u, 1, EC_IMODES33}, {01312u, 1, EC_MARKSTAT}, {01331u, 1, EC_LEMMASS}, {01332u, 1, EC_CSMMASS}, {01343u, 1, EC_DAPDATR1}, {01344u, 2, EC_TEVENT}, {01501u, 1, EC_UPSVFLAG}, {01502u, 42, EC_SV_TEMP}, {01554u, 42, EC_SV_CSM}, {01626u, 42, EC_SV_LEM}, {01706u, 3, EC_TEPHEM}, {01733u, 18, EC_REFSMMAT}, {02022u, 6, EC_RLS}, {02200u, 2, EC_RR_AZ}, {02202u, 2, EC_RR_ELEV}, {02400u, 2, EC_TLAND}, {03404u, 6, EC_AOTAZ}, {03412u, 6, EC_AOTEL}, {03422u, 1, EC_ZOOMTIME}, {03433u, 6, EC_DELVSLV}, {03441u, 2, EC_TIG}, {03453u, 2, EC_TTOGO}, {03462u, 1, EC_TRKMKCNT}, {03477u, 2, EC_SAVET_30}, {03516u, 2, EC_TGO}, {03606u, 18, EC_XSMD}, {03635u, 2, EC_TTPI}, {03700u, 6, EC_VGTIG}, {01452u, 15, EC_IMUCOMP}, {01477u, 1, EC_GCOMPSW}, {01711u, 2, EC_AZO}, {01713u, 2, EC_NEG_AYO}, {01715u, 2, EC_AXO}, {02012u, 6, EC_504LM}, {01352u, 2, EC_LUNARCOEF}, {02402u, 26, EC_LM_BRK_DP}, {02434u, 2, EC_LM_BRK_SP}, {02436u, 26, EC_LM_APP_DP}, {02470u, 2, EC_LM_APP_SP}, {02472u, 12, EC_LM_IGN}, {02506u, 2, EC_LM_CRIT}, {02510u, 8, EC_LM_V2FG}, {02537u, 1, EC_LM_RODSCALE}, {02540u, 34, EC_LM_ABORT}, {03423u, 4, EC_LM_TEND}, {03000u, 1, EC_LM_HIASCENT}, {03001u, 2, EC_LM_ROLLPIT}, {03003u, 7, EC_LM_DAPGAINS}, {03012u, 2, EC_LM_IGNAOS}, {01354u, 2, EC_LM_RADSKAL}, {01356u, 1, EC_LM_SKALSKAL}, {02520u, 2, EC_LM_DELQFIX}, {02522u, 13, EC_LM_LRPAD}, {03420u, 2, EC_LM_LRHMAX}, {03427u, 2, EC_LM_RPCRT}, {03431u, 2, EC_LM_TNEWA}, {01244u, 2, EC_LM_MASS}, {01700u, 6, EC_X789}, {01770u, 2, EC_LM_RANGEVAR}, {01772u, 2, EC_LM_RATEVAR}, {01774u, 2, EC_LM_VARMIN}, {02000u, 10, EC_LM_NAVPAD}, {03400u, 2, EC_LM_ATIGINC}, {03402u, 2, EC_LM_PTIGINC}, {03253u, 6, EC_LM_UNFC}, {03261u, 6, EC_LM_UNWC}, {00752u, 12, EC_PHASES}, {01053u, 12, EC_TBASES}, {01151u, 2, EC_LONGBASE}, {02262u, 6, EC_LM_ASC_ENG}, {02272u, 8, EC_LM_ASC_TGT}, {02354u, 6, EC_LM_ASC_ACC}, {03630u, 4, EC_LM_ASC_RCO}, {03645u, 6, EC_LM_VGVECT}, {02334u, 16, EC_LM_ASC_WRD}, {02270u, 2, EC_LM_ASC_GT}, {02314u, 2, EC_LM_ASC_GEFF}, {03473u, 2, EC_LM_HDOTDISP}, {02302u, 6, EC_LM_LAXIS}, {03717u, 6, EC_LM_QAXIS}, {03725u, 6, EC_LM_ZAXIS1}};

static void vec_set_more(ec_item_t item, agc_vec_t v); /* the remaining VEC items (vec_set) */

/**********************************************************************
 * Whether an ECADR is one of the flagwords (STATE + 0..13).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: is_flagword
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   ecadr     uint16_t  I    erasable address
 *
 * RETURNS: bool -- true for FLAGWRD0-13
 **********************************************************************/
static bool is_flagword(uint16_t ecadr)
{
    uint16_t n = (agc_state_vehicle() == AGC_VEHICLE_LM) ? (uint16_t) AGC_FLAGWORDS_LM : (uint16_t) AGC_FLAGWORDS_CSM; /* flagwords of this rope (LM or CSM) */

    return ((ecadr >= EC_STATE) && (ecadr < EC_STATE + n));
}

/* ---- word <-> value ------------------------------------------------------ */

/**********************************************************************
 * A signed value as its 15-bit ones'-complement word.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: word_of_sp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   v         agc_sp_t  I    the value (the erasable's own scale)
 *
 * RETURNS: uint16_t -- the word
 **********************************************************************/
static uint16_t word_of_sp(agc_sp_t v)
{
    return ((v < 0) ? (uint16_t) (~(uint16_t) (-(int32_t) v) & EC_WORD_MAX) : (uint16_t) v); /* negative: complement of the magnitude */
}

/**********************************************************************
 * A 15-bit ones'-complement word as a signed value (-0 reads 0).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: sp_of_word
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   w         uint16_t  I    the word
 *
 * RETURNS: agc_sp_t -- the value (the erasable's scale)
 **********************************************************************/
static agc_sp_t sp_of_word(uint16_t w)
{
    return (((w & EC_SIGN) != 0u) ? (agc_sp_t) -(int32_t) ((uint16_t) ~w & EC_MAG) : (agc_sp_t) w); /* sign bit: minus the complement */
}

/**********************************************************************
 * A DP value as its two sign-agreeing words.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: dp_to
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------------------------
 *   v         agc_dp_t    I    the value (the erasable's scale)
 *   w         uint16_t *  O    the high and low words
 **********************************************************************/
static void dp_to(agc_dp_t v, uint16_t *w)
{
    agc_dp_words_t d = agc_dp_to_words(v); /* sign-agreeing pair */

    w[0] = word_of_sp(d.hi);
    w[1] = word_of_sp(d.lo);
}

/**********************************************************************
 * Two words (signs free to disagree) as a DP value.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: dp_of
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  -------------------------------------
 *   w         const uint16_t *  I    the high and low words
 *
 * RETURNS: agc_dp_t -- the value (the erasable's scale)
 **********************************************************************/
static agc_dp_t dp_of(const uint16_t *w)
{
    agc_dp_words_t d = {sp_of_word(w[0]), sp_of_word(w[1])}; /* signs free */

    return (agc_dp_from_words(d));
}

/**********************************************************************
 * A vector as its six words (three DP, x y z).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: vec_to
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------------------------
 *   v         agc_vec_t   I    the vector (the erasable's scale)
 *   w         uint16_t *  O    six words
 **********************************************************************/
static void vec_to(agc_vec_t v, uint16_t *w)
{
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        dp_to(v.c[i], &w[EC_DP_WORDS * i]);
    }
}

/**********************************************************************
 * Six words as a vector (three DP, x y z).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: vec_of
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  -------------------------------------
 *   w         const uint16_t *  I    six words
 *
 * RETURNS: agc_vec_t -- the vector (the erasable's scale)
 **********************************************************************/
static agc_vec_t vec_of(const uint16_t *w)
{
    agc_vec_t v; /* the vector, the erasable's scale */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        v.c[i] = dp_of(&w[EC_DP_WORDS * i]);
    }

    return (v);
}

/* ---- the items ----------------------------------------------------------- */

/**********************************************************************
 * The element layout of an item: SP, RAW, TP or DP (DP, VEC, MAT, groups).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: item_layout
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: ec_layout_t -- the layout
 **********************************************************************/
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

/**********************************************************************
 * IMU compensation pad load, element k in erasable order: PBIASX PIPASCFX
 * PBIASY PIPASCFY PBIASZ PIPASCFZ NBDX NBDY NBDZ ADIAX ADIAY ADIAZ ADSRAX
 * ADSRAY ADSRAZ.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: comp_field
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -------------------------------------
 *   p         agc_imu_comp_params_t *  I    the compensation parameters
 *   k         int                      I    element of the item, from 0
 *
 * RETURNS: agc_sp_t * -- the field of element k
 **********************************************************************/
static agc_sp_t *comp_field(agc_imu_comp_params_t *p, int k)
{
    agc_sp_t *f = &p->adsra[k - 12]; /* ADSRAX..Z */

    if (k < 6)
    {
        f = ((k % 2) == 0) ? &p->pipabias[k / 2] : &p->pipascf[k / 2]; /* PBIAS, PIPASCF alternate */
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

/**********************************************************************
 * IMU compensation element k (comp_field order).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: comp_get
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------------------------
 *   k         int   I    element of the item, from 0
 *
 * RETURNS: agc_sp_t -- the value (the pad load's scale)
 **********************************************************************/
static agc_sp_t comp_get(int k)
{
    agc_imu_comp_params_t p; /* the pad load */

    agc_imu_comp_get(&p);

    return (*comp_field(&p, k));
}

/**********************************************************************
 * Store IMU compensation element k (comp_field order).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: comp_set
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   k         int       I    element of the item, from 0
 *   v         agc_sp_t  I    the value (the pad load's scale)
 *
 * RETURNS: agc_status_t -- the status of agc_imu_comp_set
 **********************************************************************/
static agc_status_t comp_set(int k, agc_sp_t v)
{
    agc_imu_comp_params_t p; /* the pad load */

    agc_imu_comp_get(&p);
    *comp_field(&p, k) = v;

    return (agc_imu_comp_load(&p));
}

/**********************************************************************
 * E3J22R2M (k = 0), E32C31RM (k = 1): the lunar potential pad loads.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: lunar_coef
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------------------------
 *   k         int   I    element of the item, from 0
 *
 * RETURNS: agc_sp_t -- the coefficient (scale per ERASABLE_ASSIGNMENTS)
 **********************************************************************/
static agc_sp_t lunar_coef(int k)
{
    agc_sp_t j22 = 0; /* E3J22R2M, pad-load scale (ERASABLE_ASSIGNMENTS) */
    agc_sp_t c31 = 0; /* E32C31RM, pad-load scale (ERASABLE_ASSIGNMENTS) */

    agc_integ_get_lunar_coeffs(&j22, &c31);

    return ((k == 0) ? j22 : c31); /* E3J22R2M or E32C31RM */
}

/**********************************************************************
 * Store E3J22R2M (k = 0) or E32C31RM (k = 1).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: set_lunar_coef
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   k         int       I    element of the item, from 0
 *   v         agc_sp_t  I    the coefficient (scale per ERASABLE_ASSIGNMENTS)
 **********************************************************************/
static void set_lunar_coef(int k, agc_sp_t v)
{
    agc_sp_t j22 = 0; /* E3J22R2M, pad-load scale (ERASABLE_ASSIGNMENTS) */
    agc_sp_t c31 = 0; /* E32C31RM, pad-load scale (ERASABLE_ASSIGNMENTS) */

    agc_integ_get_lunar_coeffs(&j22, &c31);

    if (k == 0)
    {
        j22 = v;
    }
    else
    {
        c31 = v;
    }

    agc_integ_set_lunar_coeffs(j22, c31);
}

/**********************************************************************
 * LM landing pad loads (agc_lm_pad): SP element k.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: lm_sp_field
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    element of the item, from 0
 *
 * RETURNS: agc_sp_t * -- the field
 **********************************************************************/
static agc_sp_t *lm_sp_field(ec_item_t item, int k)
{
    agc_lm_pad_t *pad  = agc_lm_pad();                                                     /* the landing pad loads */
    agc_sp_t     *f    = &pad->rodscale;                                                   /* EC_LM_RODSCALE */
    agc_sp_t     *t[4] = {&pad->tendbrak, &pad->tendappr, &pad->delttfap, &pad->leadtime}; /* EC_LM_TEND */

    if (item == EC_LM_BRK_SP || item == EC_LM_APP_SP)
    {
        int i = (item == EC_LM_BRK_SP) ? 0 : 1; /* braking, approach */

        f = (k == 0) ? &pad->target[i].tcgf : &pad->target[i].tcgi; /* TCGF then TCGI */
    }
    else if (item == EC_LM_CRIT)
    {
        f = (k == 0) ? &pad->lowcrit : &pad->highcrit; /* LOWCRIT then HIGHCRIT */
    }
    else if (item == EC_LM_TEND)
    {
        f = t[k];
    }
    else
    {
        /* RODSCALE */
    }

    return (f);
}

/**********************************************************************
 * LM landing target DP element k: RFG (0-2), VFG (3-5), AFG (6-8), VFG*,
 * AFG*, JFG*, GAIN (9-12).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: lm_target_dp
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -------------------------------------
 *   t         agc_lm_target_t *  I    braking or approach target
 *   k         int                I    element of the item, from 0
 *
 * RETURNS: agc_dp_t * -- the field
 **********************************************************************/
static agc_dp_t *lm_target_dp(agc_lm_target_t *t, int k)
{
    agc_dp_t *f = &t->gain; /* 12 */

    if (k < 3)
    {
        f = &t->rfg.c[k];
    }
    else if (k < 6)
    {
        f = &t->vfg.c[k - 3];
    }
    else if (k < 9)
    {
        f = &t->afg.c[k - 6];
    }
    else if (k == 9)
    {
        f = &t->vfg_star;
    }
    else if (k == 10)
    {
        f = &t->afg_star;
    }
    else if (k == 11)
    {
        f = &t->jfg_star;
    }
    else
    {
        /* GAIN */
    }

    return (f);
}

/**********************************************************************
 * LM landing / abort pad DP element k of the DP-list items.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: lm_dp_field
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    element of the item, from 0
 *
 * RETURNS: agc_dp_t * -- the field
 **********************************************************************/
static agc_dp_t *lm_dp_field(ec_item_t item, int k)
{
    agc_lm_pad_t *pad    = agc_lm_pad();                                                                                                                         /* the landing pad loads */
    agc_dp_t     *ign[6] = {&pad->vign, &pad->rignx, &pad->rignz, &pad->kignx_b4, &pad->kigny_b8, &pad->kignv_b4};                                               /* EC_LM_IGN */
    agc_dp_t     *abt[9] = {&pad->taurod, &pad->lag_tau, &pad->minforce, &pad->maxforce, &pad->vmin, &pad->ylim, &pad->abtrdot, &pad->costhet1, &pad->costhet2}; /* EC_LM_ABORT less ABTCOF */
    agc_dp_t     *f      = NULL;                                                                                                                                 /* the element */

    agc_lm_asc_t *a      = agc_lm_asc();                                 /* the ascent erasables */
    agc_dp_t     *eng[3] = {&a->at, &a->ve, &a->tto};                    /* EC_LM_ASC_ENG */
    agc_dp_t     *tgt[4] = {&a->rdotd, &a->ydotd, &a->zdotd, &a->r_mag}; /* EC_LM_ASC_TGT */
    agc_dp_t     *acc[3] = {&a->aty, &a->atr, &a->atp};                  /* EC_LM_ASC_ACC */
    agc_dp_t     *rco[2] = {&a->rco, &a->yco};                           /* EC_LM_ASC_RCO */

    if (item == EC_CSM_DELCDU || item == EC_CSM_CDUD)
    {
        f = (item == EC_CSM_CDUD) ? &agc_csm_dap_rcs()->cdud[k] : &agc_csm_dap_rcs()->delcdu[k]; /* ?: CDUXD.., DELCDUX.. */
    }
    else if (item == EC_LM_ASC_ENG)
    {
        f = eng[k];
    }
    else if (item == EC_LM_ASC_TGT)
    {
        f = tgt[k];
    }
    else if (item == EC_LM_ASC_ACC)
    {
        f = acc[k];
    }
    else if (item == EC_LM_ASC_RCO)
    {
        f = rco[k];
    }
    else if (item == EC_LM_BRK_DP || item == EC_LM_APP_DP)
    {
        f = lm_target_dp(&pad->target[(item == EC_LM_BRK_DP) ? 0 : 1], k); /* braking 0, approach 1 */
    }
    else if (item == EC_LM_IGN)
    {
        f = ign[k];
    }
    else if (item == EC_LM_V2FG)
    {
        f = (k < 3) ? &pad->v2fg.c[k] : &pad->tauvert; /* V2FG x y z, then TAUVERT */
    }
    else if (k < 4)
    {
        f = abt[k]; /* EC_LM_ABORT: TAUROD .. MAXFORCE */
    }
    else if (k < 12)
    {
        f = &pad->abtcof[k - 4]; /* ABTCOF */
    }
    else
    {
        f = abt[k - 8]; /* VMIN .. COSTHET2 */
    }

    return (f);
} /* lm_dp_field */

/**********************************************************************
 * Whether an item is an LM landing / abort DP list.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: is_lm_dp
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: bool -- true for those items
 **********************************************************************/
static bool is_lm_dp(ec_item_t item)
{
    return (item >= EC_LM_BRK_DP && item <= EC_CSM_CDUD);
}

/**********************************************************************
 * The LM pad loads restored by address beyond the landing pad: the DAP
 * (HIASCENT, DKTRAP .. DKDB), ROLLTIME/PITTIME, IGNAOSQ/R, the landing
 * radar (SKALSKAL, LRALPHA .. LRWVFF, LRHMAX/LRWH, RPCRTIME/RPCRTQSW) and
 * the rendezvous navigation pad (WRENDPOS .. TRUNVAR, RVARMIN, VVARMIN):
 * SP element k of the item, in erasable order.
 * AGC: none (address map; addresses from Luminary099 ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: lm_pad_field
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    element of the item, from 0
 *
 * RETURNS: agc_sp_t * -- the field
 **********************************************************************/
static agc_sp_t *lm_pad_field(ec_item_t item, int k)
{
    agc_guid_lm_t       *g      = &agc_guid_state()->lm;                                                                                                                                                             /* Luminary guidance */
    agc_rendz_nav_pad_t *nav    = agc_rendz_nav_pad();                                                                                                                                                               /* rendezvous navigation pad */
    agc_sp_t            *lr[13] = {&g->lralpha[0], &g->lrbeta[0], &g->lralpha[1], &g->lrbeta[1], &g->lrvmax, &g->lrvf, &g->lrwv[0], &g->lrwv[1], &g->lrwv[2], &g->lrwvf[0], &g->lrwvf[1], &g->lrwvf[2], &g->lrwvff}; /* LRALPHA .. LRWVFF */
    agc_sp_t            *np[10] = {&nav->wrendpos, &nav->wrendvel, &nav->wshaft, &nav->wtrun, &nav->rmax, &nav->vmax, &nav->wsurfpos, &nav->wsurfvel, &nav->shaftvar, &nav->trunvar};                                /* WRENDPOS .. TRUNVAR */
    agc_sp_t            *f      = &g->skalskal;                                                                                                                                                                      /* EC_LM_SKALSKAL */

    switch ((int) item)
    {
        case EC_LM_HIASCENT:
            f = agc_lm_dap_padload(0);
            break;
        case EC_LM_ROLLPIT:
            f = (k == 0) ? &g->rolltime : &g->pittime;    /* ROLLTIME then PITTIME */
            break;
        case EC_LM_DAPGAINS:
            f = agc_lm_dap_padload(k + 1);
            break;
        case EC_LM_IGNAOS:
            f = (k == 0) ? &g->ignaosq : &g->ignaosr;     /* IGNAOSQ then IGNAOSR */
            break;
        case EC_LM_LRPAD:
            f = lr[k];
            break;
        case EC_LM_LRHMAX:
            f = (k == 0) ? &g->lrhmax : &g->lrwh;         /* LRHMAX then LRWH */
            break;
        case EC_LM_RPCRT:
            f = (k == 0) ? &g->rpcrtime : &g->rpcrtqsw;   /* RPCRTIME then RPCRTQSW */
            break;
        case EC_LM_NAVPAD:
            f = np[k];
            break;
        case EC_LM_VARMIN:
            f = (k == 0) ? &nav->rvarmin : &nav->vvarmin; /* RVARMIN then VVARMIN */
            break;
        default:
            break;                                        /* EC_LM_SKALSKAL */
    }

    return (f);
} /* lm_pad_field */

/**********************************************************************
 * SP pad loads held in module structs (sp_field).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: is_sp_field
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: bool -- true for those items
 **********************************************************************/
static bool is_sp_field(ec_item_t item)
{
    return (item >= EC_EMDOT && item <= EC_LM_TEND);
}

/**********************************************************************
 * The CSM inertia words, element k: IXX, IAVG, IAVG/TLX (MASSPROP).
 * AGC: none (address map; addresses from Comanche055 ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: csm_inertia_field
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  -------------------------------------
 *   dc        agc_csm_dapcom_t *  I    the RCS/TVC common storage
 *   k         int                 I    element of the item, from 0
 *
 * RETURNS: agc_sp_t * -- the field
 **********************************************************************/
static agc_sp_t *csm_inertia_field(agc_csm_dapcom_t *dc, int k)
{
    agc_sp_t *f = &dc->iavg_tlx; /* k 2: IAVG/TLX */

    if (k == 0)
    {
        f = &dc->ixx;
    }
    else if (k == 1)
    {
        f = &dc->iavg;
    }
    else
    {
        /* IAVG/TLX */
    }

    return (f);
}

/**********************************************************************
 * The struct field of SP pad-load element k.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: sp_field
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    element of the item, from 0
 *
 * RETURNS: agc_sp_t * -- the field
 **********************************************************************/
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
        case EC_HBN10:
            break;
        case EC_CSM_INERTIA:
            f = csm_inertia_field(dc, k);
            break;
        case EC_CSM_HOLDFLAG:
            f = &agc_csm_dap_rcs()->holdflag;
            break;
        default:
            f = (item >= EC_LM_BRK_SP) ? lm_sp_field(item, k) : lm_pad_field(item, k); /* LM landing pad (EC_LM_BRK_SP ..) or LM DAP / LR / nav pad */
            break;
    } /* switch */

    return (f);
} /* sp_field */

/**********************************************************************
 * DP pad loads held in module structs (dp_field).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: is_dp_field
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: bool -- true for those items
 **********************************************************************/
static bool is_dp_field(ec_item_t item)
{
    return (item >= EC_EK1VAL && item <= EC_EMSALT);
}

/**********************************************************************
 * The struct field of a DP pad load (EK1VAL, EK2VAL, EK3VAL, EMSALT).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: dp_field
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: agc_dp_t * -- the field
 **********************************************************************/
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
    else if (item == EC_LM_RADSKAL)
    {
        f = &agc_guid_state()->lm.radskal;
    }
    else if (item == EC_LM_DELQFIX)
    {
        f = &agc_guid_state()->lm.delqfix;
    }
    else if (item == EC_LM_TNEWA)
    {
        f = &agc_guid_state()->lm.tnewa;
    }
    else if (item == EC_LM_HDOTDISP)
    {
        f = &agc_guid_state()->lm.hdotdisp;
    }
    else if (item == EC_LM_RANGEVAR)
    {
        f = &agc_rendz_nav_pad()->rangevar;
    }
    else if (item == EC_LM_RATEVAR)
    {
        f = &agc_rendz_nav_pad()->ratevar;
    }
    else if (item == EC_LM_ATIGINC)
    {
        f = &agc_rendz_erasable()->atiginc;
    }
    else if (item == EC_LM_PTIGINC)
    {
        f = &agc_rendz_erasable()->ptiginc;
    }
    else
    {
        /* EMSALT */
    }

    return (f);
} /* dp_field */

/**********************************************************************
 * SP items: the value of element k.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: sp_get
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    element of the item, from 0
 *
 * RETURNS: agc_sp_t -- the value (the erasable's scale)
 **********************************************************************/
static agc_sp_t sp_get(ec_item_t item, int k)
{
    agc_sp_t v = is_sp_field(item) ? *sp_field(item, k) : 0; /* the value, the erasable's scale */

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
    } /* switch */

    return (v);
} /* sp_get */

/**********************************************************************
 * SP items: store element k (FANG also sets K3VAL's low word).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: sp_set
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    element of the item, from 0
 *   v         agc_sp_t   I    the value (the erasable's scale)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the setter's status (refused value)
 **********************************************************************/
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
            st                           = comp_set(k, v);
            break;
        case EC_GCOMPSW:
            st                           = agc_imu_set_gcompsw(v);
            break;
        case EC_LUNARCOEF:
            set_lunar_coef(k, v);
            break;
        case EC_UPSVFLAG:
            st                           = agc_erase_set_upsvflag(v);
            break;
        case EC_FANG:
            agc_guid_state()->csm.fang   = v;
            agc_guid_state()->csm.ek3val = agc_dp_pack(agc_dp_high(agc_guid_state()->csm.ek3val), v); /* K3VAL = (EK3VAL, FANG) as DDV reads them */
            break;
        default:
            *sp_field(item, k)           = v;                                                         /* the struct fields */
            break;
    } /* switch */

    return (st);
} /* sp_set */

/**********************************************************************
 * The ascent guidance's words between Y and YRATE (ERASABLE_ASSIGNMENTS
 * L: Y, DRDOT, DYDOT, DZDOT, PCONS, YCONS, PRATE, YRATE), word k as
 * stored: the DSU results keep their words, the DPs give their words.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: asc_words
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    EC_LM_ASC_WRD, EC_LM_ASC_GT (TBUP) or EC_LM_ASC_GEFF
 *   k         int        I    word of the item, from 0
 *
 * RETURNS: agc_dp_words_t * -- the word pair holding word k, or NULL for
 *          the DP items (Y, PRATE, YRATE: see asc_dp)
 **********************************************************************/
static agc_dp_words_t *asc_words(ec_item_t item, int k)
{
    agc_lm_asc_t   *a    = agc_lm_asc();                                                              /* ascent erasables */
    agc_dp_words_t *w[8] = {NULL, &a->drdot, &a->dydot, &a->dzdot, &a->pcons, &a->ycons, NULL, NULL}; /* the stored pairs */
    agc_dp_words_t *p    = w[(k / 2) % 8];                                                            /* pair k / 2 */

    if (item == EC_LM_ASC_GT)
    {
        p = &a->tbup;
    }
    else if (item == EC_LM_ASC_GEFF)
    {
        p = &a->geff;
    }
    else
    {
        /* EC_LM_ASC_WRD */
    }

    return (p);
}

/**********************************************************************
 * The DPs among the ascent words (Y, PRATE, YRATE), as asc_words.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: asc_dp
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------------------------
 *   k         int   I    word of EC_LM_ASC_WRD, from 0
 *
 * RETURNS: agc_dp_t * -- the DP holding word k
 **********************************************************************/
static agc_dp_t *asc_dp(int k)
{
    agc_lm_asc_t *a = agc_lm_asc(); /* ascent erasables */
    agc_dp_t     *d = &a->y;        /* Y at words 0, 1 */

    if (k / 2 == 6)
    {
        d = &a->prate;
    }
    else if (k / 2 == 7)
    {
        d = &a->yrate;
    }
    else
    {
        /* Y */
    }

    return (d);
}

/**********************************************************************
 * Word k of an ascent words item (EC_LM_ASC_WRD, _GT, _GEFF), as stored.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: asc_word_get
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    word of the item, from 0
 *
 * RETURNS: uint16_t -- the word
 **********************************************************************/
static uint16_t asc_word_get(ec_item_t item, int k)
{
    agc_dp_words_t d; /* the word pair holding word k */

    if (asc_words(item, k) != NULL)
    {
        d = *asc_words(item, k);
    }
    else
    {
        d = agc_dp_to_words(*asc_dp(k));
    }

    return (word_of_sp(((k % 2) == 0) ? d.hi : d.lo)); /* major : minor word */
}

/**********************************************************************
 * Store word k of an ascent words item, as asc_word_get reads it.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: asc_word_set
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    word of the item, from 0
 *   w         uint16_t   I    the word
 **********************************************************************/
static void asc_word_set(ec_item_t item, int k, uint16_t w)
{
    agc_dp_words_t *p = asc_words(item, k);                             /* a stored pair, or NULL for a DP */
    agc_dp_words_t  d = (p != NULL) ? *p : agc_dp_to_words(*asc_dp(k)); /* the pair now */

    if ((k % 2) == 0)
    {
        d.hi = sp_of_word(w);
    }
    else
    {
        d.lo = sp_of_word(w);
    }

    if (p != NULL)
    {
        *p = d;
    }
    else
    {
        *asc_dp(k) = agc_dp_from_words(d);
    }
}

/**********************************************************************
 * RAW items: the word of element k.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: raw_get
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    element of the item, from 0
 *
 * RETURNS: uint16_t -- the word
 **********************************************************************/
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
        case EC_CSM_THETADX:
            w = agc_csm_dap_rcs()->thetad[k];
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
        case EC_PHASES:
            w = agc_phase_table_word(k / 2 + 1, (k % 2) == 0);                                           /* -PHASEn at even k */
            break;
        case EC_TBASES:
            w = ((k % 2) == 0) ? agc_phase_tbase_word(k / 2 + 1) : 0u;                                   /* PHSPRDTn: not kept, 0 */
            break;
        case EC_LONGBASE:
            w = (uint16_t) ((k == 0) ? (agc_phase_longbase() >> 14) : (agc_phase_longbase() & 037777u)); /* TIME2, TIME1 */
            break;
        case EC_LM_ASC_WRD:
        case EC_LM_ASC_GT:
        case EC_LM_ASC_GEFF:
            w = asc_word_get(item, k);
            break;
        default:
            w = agc_erase_mark2dwn(k);                                                                   /* EC_MARK2DWN */
            break;
    } /* item */

    return ((uint16_t) (w & EC_WORD_MAX));
} /* raw_get */

/**********************************************************************
 * RAW items: store the word of element k.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: raw_set
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   k         int        I    element of the item, from 0
 *   w         uint16_t   I    the word
 **********************************************************************/
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
        case EC_CSM_THETADX:
            agc_csm_dap_rcs()->thetad[k] = w;
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
        case EC_PHASES:
            agc_phase_table_set_word(k / 2 + 1, (k % 2) == 0, w); /* -PHASEn at even k */
            break;
        case EC_TBASES:

            if ((k % 2) == 0)
            {
                agc_phase_set_tbase_word(k / 2 + 1, w);
            }
            else
            {
                /* PHSPRDTn belongs to PHSNAMEn, a rope address with no C job id: not kept */
            }

            break;
        case EC_LM_ASC_WRD:
        case EC_LM_ASC_GT:
        case EC_LM_ASC_GEFF:
            asc_word_set(item, k, w);
            break;
        case EC_LONGBASE:
            agc_phase_set_longbase((k == 0) ? (((uint32_t) w << 14) | (agc_phase_longbase() & 037777u)) : ((agc_phase_longbase() & ~(uint32_t) 037777u) | w)); /* k 0: TIME2 word, k 1: TIME1 word */
            break;
        default:
            agc_erase_set_mark2dwn(k, w);                                                                                                                      /* EC_MARK2DWN */
            break;
    } /* item */
} /* raw_set */

/**********************************************************************
 * DP items (one element): the value.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: dp_get
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: agc_dp_t -- the value (the erasable's scale)
 **********************************************************************/
static agc_dp_t dp_get(ec_item_t item)
{
    agc_dp_t v = 0; /* the value, the erasable's scale */

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
        case EC_LM_MASS:
            v = agc_erase_mass();
            break;
        default:
            v = agc_erase_savet_30(); /* EC_SAVET_30 */
            break;
    } /* switch */

    return (v);
} /* dp_get */

/**********************************************************************
 * DP items (one element): store the value.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: dp_set
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   v         agc_dp_t   I    the value (the erasable's scale)
 **********************************************************************/
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
        case EC_LM_MASS:
            agc_erase_set_mass(v);
            break;
        default:
            agc_erase_set_savet_30(v); /* EC_SAVET_30 */
            break;
    } /* switch */
} /* dp_set */

/**********************************************************************
 * LAT, LONG, ALT (one store, agc_navaux_*_latlong): the field.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: latlong_field
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ll        agc_latlong_t *  I    the latitude / longitude / altitude set
 *   item      ec_item_t        I    the mapped erasable
 *
 * RETURNS: agc_dp_t * -- the field
 **********************************************************************/
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

/**********************************************************************
 * Whether an item is LAT, LONG or ALT.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: is_latlong
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: bool -- true for those items
 **********************************************************************/
static bool is_latlong(ec_item_t item)
{
    return (item == EC_LAT || item == EC_LONG || item == EC_ALT);
}

/**********************************************************************
 * TIMEMO, the ephemeris' centre time.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: tp_timemo
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_tp_t -- cs, scale 2^42
 **********************************************************************/
static agc_tp_t tp_timemo(void)
{
    agc_ephem_csm_t eph; /* the ephemeris */

    agc_ephem_get_csm_data(&eph);

    return (eph.timemo);
}

/**********************************************************************
 * Store TIMEMO.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: tp_set_timemo
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   t         agc_tp_t  I    cs, scale 2^42
 **********************************************************************/
static void tp_set_timemo(agc_tp_t t)
{
    agc_ephem_csm_t eph; /* the ephemeris */

    agc_ephem_get_csm_data(&eph);
    eph.timemo = t;
    agc_ephem_set_csm_data(&eph);
}

/**********************************************************************
 * Launch pad load (AZO, -AYO, AXO, 504LM) and the CSM ephemeris (VECOEM,
 * RESO, VESO, OMEGAES): DP values of the item, in erasable order.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: is_group
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: bool -- true for those items
 **********************************************************************/
static bool is_group(ec_item_t item)
{
    return (item >= EC_AZO && item <= EC_OMEGAES);
}

/**********************************************************************
 * A vector into three DP elements.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: group_vec
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------------------------
 *   v         agc_vec_t   I    the vector (the item's scale)
 *   d         agc_dp_t *  O    x, y, z
 **********************************************************************/
static void group_vec(agc_vec_t v, agc_dp_t *d)
{
    for (int k = 0; k < AGC_VEC_N; k++)
    {
        d[k] = v.c[k];
    }
}

/**********************************************************************
 * A group item as DP values in erasable order.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: group_get
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------------------------
 *   item      ec_item_t   I    the mapped erasable
 *   d         agc_dp_t *  O    the values (the item's scale)
 **********************************************************************/
static void group_get(ec_item_t item, agc_dp_t *d)
{
    agc_navaux_padload_t pad; /* AZO .. 504LM */
    agc_ephem_csm_t      eph; /* VECOEM .. OMEGAES */

    agc_navaux_get_padload(&pad);
    agc_ephem_get_csm_data(&eph);
    d[0] = eph.omegaes; /* the DP items: OMEGAES unless one below */

    if (item == EC_AZO)
    {
        d[0] = pad.azo;
    }
    else if (item == EC_NEG_AYO)
    {
        d[0] = pad.neg_ayo;
    }
    else if (item == EC_AXO)
    {
        d[0] = pad.axo;
    }
    else
    {
        /* OMEGAES, or a vector item filled below */
    }

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
} /* group_get */

/**********************************************************************
 * Three DP elements as a vector.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: group_of
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  -------------------------------------
 *   d         const agc_dp_t *  I    x, y, z (the item's scale)
 *
 * RETURNS: agc_vec_t -- the vector
 **********************************************************************/
static agc_vec_t group_of(const agc_dp_t *d)
{
    agc_vec_t v; /* the vector, the erasable's scale */

    for (int k = 0; k < AGC_VEC_N; k++)
    {
        v.c[k] = d[k];
    }

    return (v);
}

/**********************************************************************
 * Store a group item from DP values in erasable order.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: group_put
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  -------------------------------------
 *   item      ec_item_t         I    the mapped erasable
 *   d         const agc_dp_t *  I    the values (the item's scale)
 **********************************************************************/
static void group_put(ec_item_t item, const agc_dp_t *d)
{
    agc_navaux_padload_t pad; /* AZO .. 504LM */
    agc_ephem_csm_t      eph; /* VECOEM .. OMEGAES */

    agc_navaux_get_padload(&pad);
    agc_ephem_get_csm_data(&eph);
    pad.azo     = (item == EC_AZO) ? d[0] : pad.azo;            /* AZO when it is the item */
    pad.neg_ayo = (item == EC_NEG_AYO) ? d[0] : pad.neg_ayo;    /* -AYO when it is the item */
    pad.axo     = (item == EC_AXO) ? d[0] : pad.axo;            /* AXO when it is the item */
    pad.lm504   = (item == EC_504LM) ? group_of(d) : pad.lm504; /* 504LM when it is the item */
    eph.reso    = (item == EC_RESO) ? group_of(d) : eph.reso;   /* RESO when it is the item */
    eph.veso    = (item == EC_VESO) ? group_of(d) : eph.veso;   /* VESO when it is the item */
    eph.omegaes = (item == EC_OMEGAES) ? d[0] : eph.omegaes;    /* OMEGAES when it is the item */

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

/**********************************************************************
 * LAT(SPL), LNG(SPL): the CSM entry target (agc_entry_state).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: is_splash
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: bool -- true for those items
 **********************************************************************/
static bool is_splash(ec_item_t item)
{
    return (item == EC_LAT_SPL || item == EC_LNG_SPL);
}

/**********************************************************************
 * The field of LAT(SPL) or LNG(SPL).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: splash_field
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: agc_dp_t * -- the field
 **********************************************************************/
static agc_dp_t *splash_field(ec_item_t item)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    return ((item == EC_LAT_SPL) ? &es->lat_spl : &es->lng_spl); /* LAT(SPL) or LNG(SPL) */
}

/**********************************************************************
 * VEC items: the vector.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: vec_get
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: agc_vec_t -- the vector (the erasable's scale)
 **********************************************************************/
static agc_vec_t vec_get(ec_item_t item)
{
    agc_vec_t v; /* the vector, the erasable's scale */

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
        case EC_X789:
            v = agc_erase_x789();
            break;
        case EC_LM_UNFC:
            v = agc_lm_ldg()->unfc_2;
            break;
        case EC_LM_UNWC:
            v = agc_lm_ldg()->unwc_2;
            break;
        case EC_LM_VGVECT:
            v = agc_lm_asc()->vgvect;
            break;
        case EC_LM_LAXIS:
            v = agc_lm_asc()->laxis;
            break;
        case EC_LM_QAXIS:
            v = agc_guid_state()->lm.uhyp;     /* QAXIS EQUALS UHYP */
            break;
        case EC_LM_ZAXIS1:
            v = agc_guid_state()->lm.uhzp;     /* ZAXIS1 EQUALS UHZP */
            break;
        case EC_UNITW:
            v = agc_guid_state()->avg.unitw;
            break;
        default:
            v = agc_rendz_erasable()->delvlvc; /* EC_DELVSLV */
            break;
    } /* switch */

    return (v);
} /* vec_get */

/**********************************************************************
 * VEC items: store the vector.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: vec_set
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   v         agc_vec_t  I    the vector (the erasable's scale)
 **********************************************************************/
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
        case EC_X789:
            agc_erase_set_x789(v);
            break;
        case EC_LM_UNFC:
            agc_lm_ldg()->unfc_2          = v;
            break;
        case EC_LM_UNWC:
            agc_lm_ldg()->unwc_2          = v;
            break;
        case EC_DELVSLV:
            agc_rendz_erasable()->delvlvc = v;
            break;
        default:
            vec_set_more(item, v);
            break;
    }
}

/**********************************************************************
 * VEC items vec_set leaves: the LM ascent's VGVECT, LAXIS, QAXIS (= UHYP),
 * ZAXIS1 (= UHZP) and the CM's UNITW, whose first four words are -AYO and
 * AXO (EQUALS UNITW, UNITW +2: the launch pad loads see the same words).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: vec_set_more
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *   v         agc_vec_t  I    the vector (the erasable's scale)
 **********************************************************************/
static void vec_set_more(ec_item_t item, agc_vec_t v)
{
    agc_navaux_padload_t pad; /* -AYO, AXO */

    switch ((int) item)
    {
        case EC_LM_VGVECT:
            agc_lm_asc()->vgvect        = v;
            break;
        case EC_LM_LAXIS:
            agc_lm_asc()->laxis         = v;
            break;
        case EC_LM_QAXIS:
            agc_guid_state()->lm.uhyp   = v;      /* QAXIS EQUALS UHYP */
            break;
        case EC_LM_ZAXIS1:
            agc_guid_state()->lm.uhzp   = v;      /* ZAXIS1 EQUALS UHZP */
            break;
        case EC_UNITW:
            agc_guid_state()->avg.unitw = v;
            agc_navaux_get_padload(&pad);
            pad.neg_ayo                 = v.c[0]; /* -AYO EQUALS UNITW    */
            pad.axo                     = v.c[1]; /* AXO EQUALS UNITW +2  */
            agc_navaux_set_padload(&pad);
            break;
        default:
            break;                                /* every VEC item is listed above or in vec_set */
    } /* switch */
} /* vec_set_more */

/**********************************************************************
 * State vector blocks (RRECT, VRECT, TET, DELTAV, NUV, RCV, VCV, TC, XKEP:
 * the 42-word layout of the temporary and the two permanent sets) as 21 DP.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: sv_dps
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  -------------------------------------
 *   sv        const agc_integ_sv_t *  I    the state vector set
 *   d         agc_dp_t *              O    21 DP values in erasable order (each its scale)
 **********************************************************************/
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

/**********************************************************************
 * 21 DP values in erasable order as a state vector set.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: sv_of_dps
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  -------------------------------------
 *   d         const agc_dp_t *  I    21 DP values (each its scale)
 *   sv        agc_integ_sv_t *  O    the state vector set
 **********************************************************************/
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

/**********************************************************************
 * Whether an item is a 42-word state vector block.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: is_sv
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   item      ec_item_t  I    the mapped erasable
 *
 * RETURNS: bool -- true for those items
 **********************************************************************/
static bool is_sv(ec_item_t item)
{
    return (item == EC_SV_TEMP || item == EC_SV_CSM || item == EC_SV_LEM);
}

/**********************************************************************
 * The state vector set of a block item (temporary, CSM, LM).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: sv_get
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  -------------------------------------
 *   item      ec_item_t         I    the mapped erasable
 *   sv        agc_integ_sv_t *  O    the set
 **********************************************************************/
static void sv_get(ec_item_t item, agc_integ_sv_t *sv)
{
    if (item == EC_SV_TEMP)
    {
        *sv = agc_integ_shared_ctx()->sv;
    }
    else
    {
        agc_integ_perm_get((item == EC_SV_CSM) ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM, sv); /* CSM or LM permanent set */
    }
}

/**********************************************************************
 * Store the state vector set of a block item.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: sv_put
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  -------------------------------------
 *   item      ec_item_t               I    the mapped erasable
 *   sv        const agc_integ_sv_t *  I    the set
 **********************************************************************/
static void sv_put(ec_item_t item, const agc_integ_sv_t *sv)
{
    if (item == EC_SV_TEMP)
    {
        agc_integ_shared_ctx()->sv = *sv;
    }
    else
    {
        agc_integ_perm_set((item == EC_SV_CSM) ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM, sv); /* CSM or LM permanent set */
    }
}

/**********************************************************************
 * The typed value of an item as sign-agreeing words.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: item_words
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  -------------------------------------
 *   e         const ec_entry_t *  I    the map entry
 *   w         uint16_t *          O    its words (e->words of them)
 **********************************************************************/
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
    else if (is_lm_dp(e->item))
    {
        for (int k = 0; k < e->words / EC_DP_WORDS; k++)
        {
            dp_to(*lm_dp_field(e->item, k), &w[EC_DP_WORDS * k]);
        }
    }
    else if (is_group(e->item))
    {
        agc_dp_t d[EC_MAX_WORDS / EC_DP_WORDS]; /* the DP values, each its erasable's scale */

        group_get(e->item, d);

        for (int k = 0; k < e->words / EC_DP_WORDS; k++)
        {
            dp_to(d[k], &w[EC_DP_WORDS * k]);
        }
    }
    else if (is_sv(e->item))
    {
        agc_integ_sv_t sv;           /* the block */
        agc_dp_t       d[EC_SV_DPS]; /* its DP values, each its erasable's scale */

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
} /* item_words */

/**********************************************************************
 * Store an item from its words (signs free to disagree).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: item_store
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  -------------------------------------
 *   e         const ec_entry_t *  I    the map entry
 *   w         const uint16_t *    I    its words
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or a setter's status (refused value)
 **********************************************************************/
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
    else if (is_lm_dp(e->item))
    {
        for (int k = 0; k < e->words / EC_DP_WORDS; k++)
        {
            *lm_dp_field(e->item, k) = dp_of(&w[EC_DP_WORDS * k]);
        }
    }
    else if (is_group(e->item))
    {
        agc_dp_t d[EC_MAX_WORDS / EC_DP_WORDS]; /* the DP values, each its erasable's scale */

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
    else if (is_lm_dp(e->item))
    {
        for (int k = 0; k < e->words / EC_DP_WORDS; k++)
        {
            *lm_dp_field(e->item, k) = dp_of(&w[EC_DP_WORDS * k]);
        }
    }
    else if (is_group(e->item))
    {
        agc_dp_t d[EC_MAX_WORDS / EC_DP_WORDS]; /* the DP values, each its erasable's scale */

        for (int k = 0; k < e->words / EC_DP_WORDS; k++)
        {
            d[k] = dp_of(&w[EC_DP_WORDS * k]);
        }

        group_put(e->item, d);
    }
    else if (is_sv(e->item))
    {
        agc_integ_sv_t sv;           /* the block */
        agc_dp_t       d[EC_SV_DPS]; /* its DP values, each its erasable's scale */

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
} /* item_store */

/**********************************************************************
 * Words per element of a layout.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: element_words
 *
 * ARGUMENTS:
 *   Argument  Type         I/O  Description (scale)
 *   --------  -----------  ---  -------------------------------------
 *   l         ec_layout_t  I    the layout
 *
 * RETURNS: int -- 1, 2 or 3
 **********************************************************************/
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

/**********************************************************************
 * Whether two word images of one element hold the same value.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: same_element
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  -------------------------------------
 *   l         ec_layout_t       I    the layout
 *   a         const uint16_t *  I    first image
 *   b         const uint16_t *  I    second image
 *
 * RETURNS: bool -- true for the same value
 **********************************************************************/
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

/**********************************************************************
 * The map entry holding ecadr (the first that does), or NULL.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: map_find
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   ecadr     uint16_t  I    erasable address
 *
 * RETURNS: const ec_entry_t * -- the entry, or NULL
 **********************************************************************/
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

/**********************************************************************
 * Bring the raw words of a mapped item up to date with its typed value,
 * element by element (an unchanged element keeps its words as stored).
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: refresh
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  -------------------------------------
 *   e         const ec_entry_t *  I    the map entry
 **********************************************************************/
static void refresh(const ec_entry_t *e)
{
    uint16_t w[EC_MAX_WORDS]; /* typed value as words */

    ec_layout_t l = item_layout(e->item); /* element layout */
    int         n = element_words(l);     /* words per element */

    item_words(e, w);

    for (int k = 0; k + n <= e->words; k += n)
    { /* only the elements whose value changed take the sign-agreeing words */
        uint16_t raw[EC_TP_WORDS]; /* the element as stored */

        for (int j = 0; j < n; j++)
        {
            agc_erase_raw_read((uint16_t) (e->ecadr + (uint16_t) (k + j)), &raw[j]);
        }

        if (!same_element(l, raw, &w[k]))
        {
            for (int j = k; j < k + n; j++)
            {
                agc_erase_raw_write((uint16_t) (e->ecadr + (uint16_t) j), w[j]);
            }
        }
    }
}

/**********************************************************************
 * Read the erasable word at ECADR (raw 15-bit word): a flagword, a
 * mapped erasable (refreshed from its typed value), or the raw image.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: agc_erase_read_ecadr
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------------------------
 *   ecadr     uint16_t    I    address, 15 bits
 *   word      uint16_t *  O    the word
 **********************************************************************/
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
        agc_erase_raw_read(ecadr, word);
    }
    else
    {
        agc_erase_raw_read(ecadr, word);
    }
}

/**********************************************************************
 * Store a raw 15-bit word at ECADR: a flagword, a mapped erasable
 * (typed value rebuilt from all its words), or the raw image.
 * AGC: none (address map; addresses from ERASABLE_ASSIGNMENTS.agc of both ropes)
 *
 * FUNCTION NAME: agc_erase_write_ecadr
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------------------------
 *   ecadr     uint16_t  I    address, 15 bits
 *   word      uint16_t  I    the word
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG for a word
 *          above 15 bits; or the setter's status (refused value; the old
 *          value stays)
 **********************************************************************/
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
        uint16_t w[EC_MAX_WORDS]; /* the item's words as stored */

        refresh(e);
        agc_erase_raw_write(ecadr, word);

        for (int k = 0; k < e->words; k++)
        {
            agc_erase_raw_read((uint16_t) (e->ecadr + (uint16_t) k), &w[k]);
        }

        st = item_store(e, w);
    }
    else
    {
        agc_erase_raw_write(ecadr, word);
    }

    return (st);
}
