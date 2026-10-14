/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of the erasable map
 *   (src/erasable_map): the named erasables, their element layouts, the
 *   word conversions, and the field items address_map_fields.c holds.
 *
 * AGC SOURCE:
 *   none (addresses from Comanche055 / Luminary099 ERASABLE_ASSIGNMENTS)
 *
 * REQUIRES:
 *   the agc/ public headers included below
 *
 * GLOBAL VARIABLES:
 *   none
 **********************************************************************/
#ifndef ERASABLE_MAP_INTERNAL_H
#define ERASABLE_MAP_INTERNAL_H

#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/status.h"

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
    EC_RTX,          /* SP x2 RTX1, RTX2 (conic body: Earth / Moon) */
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
    EC_PIPTIME1,     /* DP   PIPTIME1 average-G start time, cs 2^28 */
    EC_LM_LAND_MAG,  /* DP   /LAND/ site radius, m 2^24 (LM)   */
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
    EC_RN1,          /* VEC  RN1 average-G start position      */
    EC_VN1,          /* VEC  VN1 average-G start velocity      */
    EC_GDT1_2,       /* VEC  GDT1/2 gravity of RN1             */
    EC_GDT_2,        /* VEC  GDT/2 gravity of RN               */
    EC_LM_R_CSM,     /* VEC  R(CSM) MUNRVG CSM position, SM (LM) */
    EC_LM_V_CSM,     /* VEC  V(CSM) MUNRVG CSM velocity, SM (LM) */
    EC_SV_TEMP,      /* SV   RRECT..XKEP (uplink / integration buffer, 42 words) */
    EC_SV_CSM,       /* SV   RRECTCSM..XKEPCSM (42 words)       */
    EC_SV_LEM,       /* SV   RRECTLEM..XKEPLEM (42 words)       */
    EC_XSMD,         /* MAT  XSMD.. desired REFSMMAT, 2^1      */
    EC_REFSMMAT,     /* MAT  REFSMMAT, 2^1                     */
    EC_TEPHEM,       /* TP   TEPHEM, cs 2^42                   */
    EC_TIMEMO,       /* TP   TIMEMO, cs 2^42 (CSM)             */

    /* Fields (address_map_fields.c): the named erasables the downlists
     * send, held in their owners' storage. Layout per item there.       */
    EC_F_TIME,       /* RAW x2 TIME2, TIME1                    */
    EC_F_COUNTERS,   /* RAW x8 CDUX CDUY CDUZ CDUT CDUS PIPAX PIPAY PIPAZ */
    EC_F_UPLINK,     /* RAW x24 COMPNUMB UPOLDMOD UPVERB UPCOUNT UPBUFF +0..+19D */
    EC_F_REDOCTR,    /* RAW  REDOCTR                           */
    EC_F_BESTI,      /* SP x2 BESTI, BESTJ                     */
    EC_F_OGC,        /* DP x3 OGC, IGC, MGC                    */
    EC_F_RTARG,      /* DP x3 RTARG                            */
    EC_F_DELVEET1,   /* DP x3 DELVEET1                         */
    EC_F_DELVEET2,   /* DP x3 DELVEET2                         */
    EC_F_DELVEET3,   /* DP x3 DELVEET3                         */
    EC_F_DELVSIN,    /* DP x3 DELVSIN (CSM: = DELVEET3)        */
    EC_F_DELLT4,     /* DP   DELLT4                            */
    EC_F_ELEV,       /* DP   ELEV                              */
    EC_F_CENTANG,    /* DP   CENTANG                           */
    EC_F_DELTAR,     /* DP   DELTAR                            */
    EC_F_DIFFALT,    /* DP   DIFFALT                           */
    EC_F_TCDH,       /* DP   TCDH                              */
    EC_F_TCSI,       /* DP   TCSI                              */
    EC_F_TPASS4,     /* DP   TPASS4                            */
    EC_F_DELVTPF,    /* DP   DELVTPF                           */
    EC_F_NN,         /* SP   NN (LM)                           */
    EC_F_OPTMODES,   /* RAW  OPTMODES (CSM)                    */
    EC_F_VHFCNT,     /* SP   VHFCNT (CSM)                      */
    EC_F_VHFTIME,    /* DP   VHFTIME (CSM)                     */
    EC_F_RANGE,      /* DP x3 RANGE, RRATE, RTHETA (CSM)       */
    EC_F_HAPOX,      /* DP x2 HAPOX, HPERX (CSM)               */
    EC_F_LAUNCHAZ,   /* DP   LAUNCHAZ (CSM)                    */
    EC_F_8NN,        /* SP   8NN (CSM)                         */
    EC_F_LANDMARK,   /* SP   LANDMARK (CSM)                    */
    EC_F_SVMRKDAT,   /* RAW x36 SVMRKDAT (CSM)                 */
    EC_F_R_OTHER,    /* DP x3 R-OTHER (CSM)                    */
    EC_F_V_OTHER,    /* DP x3 V-OTHER (CSM)                    */
    EC_F_AK,         /* SP x3 AK, AK1, AK2 (CSM)               */
    EC_F_RCSFLAGS,   /* RAW  RCSFLAGS (CSM)                    */
    EC_F_WBODY,      /* DP x3 WBODY (CSM RCS DAP)              */
    EC_F_OMEGAC,     /* DP x3 OMEGAC (CSM TVC DAP, = WBODY)    */
    EC_F_ADOT,       /* DP x3 ADOT as stored (CSM RCS DAP)     */
    EC_F_OMEGAB,     /* DP x3 OMEGAB as stored (CSM TVC, = ADOT) */
    EC_F_ERRORX,     /* SP x3 ERRORX, ERRORY, ERRORZ (CSM)     */
    EC_F_PCMD,       /* SP x2 PCMD, YCMD (CSM)                 */
    EC_F_ALFA_180,   /* SP x2 ALFA/180, BETA/180 (CSM)         */
    EC_F_CMDAPMOD,   /* SP   CMDAPMOD (CSM)                    */
    EC_F_PREL,       /* SP x3 PREL, QREL, RREL (CSM)           */
    EC_F_PAXERR1,    /* SP   PAXERR1 (CSM)                     */
    EC_F_ROLLTM,     /* SP   ROLLTM (CSM)                      */
    EC_F_ROLLC,      /* DP   ROLLC (CSM)                       */
    EC_F_ECSTEER,    /* DP   ECSTEER (CSM)                     */
    EC_F_CSTEER,     /* DP   CSTEER (CSM)                      */
    EC_F_L_D1,       /* DP   L/D1 (CSM)                        */
    EC_F_LATANG,     /* DP x3 LATANG, RDOT, THETAH (CSM)       */
    EC_F_VIO,        /* DP x2 VIO, TTE (CSM)                   */
    EC_F_VPRED,      /* DP x2 VPRED, GAMMAEI (CSM)             */
    EC_F_LASTCMD,    /* SP x2 LASTYCMD, LASTXCMD (LM)          */
    EC_F_DNRADAR,    /* RAW x7 DNRRANGE DNRRDOT DNINDEX DNLRVELX..Z DNLRALT (LM) */
    EC_F_AGSK,       /* DP   AGSK (LM)                         */
    EC_F_AGSBUFF,    /* RAW x14 AGSBUFF (LM)                   */
    EC_F_GSAV,       /* DP x3 GSAV (LM)                        */
    EC_F_YNBSAV,     /* DP x6 YNBSAV, ZNBSAV (LM)              */
    EC_F_STARSAV1,   /* DP x3 STARSAV1                         */
    EC_F_STARSAV2,   /* DP x3 STARSAV2                         */
    EC_F_TALIGN,     /* DP   TALIGN (LM)                       */
    EC_F_AOTCODE,    /* SP   AOTCODE = STARCODE (LM)           */
    EC_F_LRCDUDL,    /* RAW x3 LRXCDUDL, LRYCDUDL, LRZCDUDL (LM) */
    EC_F_LRVTIMDL,   /* DP   LRVTIMDL (LM)                     */
    EC_F_RGU,        /* DP x3 RGU (LM)                         */
    EC_F_VGU,        /* DP x3 VGU (LM)                         */
    EC_F_LAND,       /* DP x3 LAND (LM)                        */
    EC_F_TTF_8,      /* DP   TTF/8 (LM)                        */
    EC_F_DELTAH,     /* DP   DELTAH (LM)                       */
    EC_F_PSEUDO55,   /* SP   PSEUDO55 (LM)                     */
    EC_F_FC,         /* DP   FC (LM)                           */
    EC_F_OMEGAP,     /* SP x3 OMEGAP, OMEGAQ, OMEGAR (LM)      */
    EC_F_ALPHAQ,     /* SP x2 ALPHAQ, ALPHAR (LM)              */
    EC_F_OMEGAPD,    /* SP x3 OMEGAPD, OMEGAQD, OMEGARD (LM)   */
    EC_F_CDUXD,      /* RAW x3 CDUXD, CDUYD, CDUZD (LM)        */
    EC_F_DOWNTORK,   /* SP x6 POSTORKP NEGTORKP POSTORKU NEGTORKU POSTORKV NEGTORKV (LM) */
    EC_F_RR_GIMBALS, /* RAW x3 AIG, AMG, AOG of R29 (LM)       */
    EC_F_LR_GIMBALS, /* SP x3 AIG, AMG, AOG of the LR mark (LM) */
    EC_F_RR_TANGNB,  /* RAW x2 TANGNB of R29 (LM)              */
    EC_F_RR_MKTIME,  /* DP   MKTIME of R29 (LM)                */
    EC_F_NAV_MKTIME, /* DP   MKTIME of the RR mark (LRS22.1, LM) */
    EC_F_LR_MKTIME,  /* DP   MKTIME of the LR mark (LM)        */
    EC_F_RM,         /* DP   RM, RR range of the mark (LM)     */
    EC_F_RANGRDOT,   /* RAW x2 RANGRDOT (LM)                   */
    EC_F_VSELECT,    /* SP   VSELECT (LM)                      */
    EC_F_VMEAS,      /* DP   VMEAS (LM)                        */
    EC_F_HMEAS,      /* DP   HMEAS (LM)                        */
    EC_F_DNLSTCOD,   /* RAW x2 DNLSTCOD (the list code), DUMPCNT */
    EC_F_AGSWORD,    /* RAW  AGSWORD, the saved list code (LM) */
    EC_F_OPTIND,     /* SP   OPTIND, -0 = TVC takeover (CSM)   */
    EC_F_OPTZONE,    /* SP   ZONE (CSM optics monitor)         */
    EC_F_OPTMON,     /* SP x4 SWSAMPLE DESOPMOD WTOPTION ZOPTCNT (CSM) */
    EC_F_DESOPT,     /* RAW x2 DESOPTT, DESOPTS (CSM)          */
    EC_F_GCOMP,      /* DP x3 GCOMP gyro compensation, pulses 2^14 */
    EC_F_PIPADT      /* SP   1/PIPADT, cs 2^8 or a TIME1 snapshot */
} ec_item_t;

/* How an item's words group into values.                                 */
typedef enum ec_layout
{
    EC_SP = 0, /* one signed word per element               */
    EC_RAW,    /* one bit-pattern word per element          */
    EC_DP,     /* two words per element                     */
    EC_TP      /* three words                               */
} ec_layout_t;

/* A word and its value (address_map.c).                                  */
uint16_t ec_word_of_sp(agc_sp_t v);         /* signed value -> ones' complement word */
agc_sp_t ec_sp_of_word(uint16_t w);         /* word -> value (-0 reads 0)            */
void     ec_dp_to(agc_dp_t v, uint16_t *w); /* DP -> two sign-agreeing words         */
agc_dp_t ec_dp_of(const uint16_t *w);       /* two words (signs free) -> DP          */

/* The field items (address_map_fields.c): EC_F_TIME and after.           */
ec_layout_t  ec_field_layout(ec_item_t item);
void         ec_field_words(ec_item_t item, int words, uint16_t *w);
agc_status_t ec_field_store(ec_item_t item, int words, const uint16_t *w);

#endif /* ERASABLE_MAP_INTERNAL_H */
