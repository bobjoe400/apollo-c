/**********************************************************************
 * FILE NAME: downlink_lists_lm.c
 *
 * PURPOSE:
 *   The LM downlists as data: DNTABLE (control list per DNLSTCOD) and the
 *   sublists the control lists point at, read by the down-telemetry
 *   program (src/io/telemetry/down_telemetry.c).
 *
 * AGC SOURCE:
 *   Luminary099 DOWNLINK_LISTS.agc pp.193-205
 *
 * EXTERNAL VARIABLES:
 *   none (const tables)
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; a list code or sublist id out of range is refused (BAD_ARG).
 *
 * NOTES:
 *   Same entry encoding as src/io/telemetry/downlink_lists_csm.c:
 *     { op, neg, ref }
 *     op     1..6 = 1DNADR..6DNADR (n DP words), AGC_DN_OP_CHAN (DNCHAN),
 *            AGC_DN_OP_PTR (DNPTR, ref = sublist id below)
 *     neg    the AGC wrote the entry negated: end of list, or (first entry of
 *            a sublist) a snapshot sublist
 *     ref    the ECADR of the first word (RN +2 -> ADR_RN + 2u, UPBUFF +8D
 *            -> ADR_UPBUFF + 8u; Luminary099 ERASABLE_ASSIGNMENTS) /
 *            channel number / sublist id
 *
 *   The AGC's EQUALS aliases (LMCSTA01 EQUALS LMORBM01 etc.) become
 *   references to the same sublist id.  LMDSAS08 is a plain
 *   (non-snapshot) sublist that sends the 11-word snapshot buffer of
 *   LMDSAS07. UNKNOWN and SPARE both equal ERASZERO (register 7, always
 *   zero).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/io/service.h"

/* Sublist ids (the AGC labels that are not EQUALS aliases).              */
enum
{
    LMORBM01 = 0,
    LMORBM02,
    LMORBM03,
    LMORBM04,
    LMORBM05,
    LMORBM06,
    LMCSTA06,
    LMCSTA07,
    LMREND07,
    LMDSAS07,
    LMDSAS08,
    LM_SUB_COUNT
};

/* The erasables the lists name: ECADR (EEE RRRRRRRR) from the rope's
 * symbol table (yaYUL), bank and address as the listing gives them.      */
#define ADR_SPARE    00007u /* 0007       SPARE */
#define ADR_TIME2    00024u /* 0024       TIME2 */
#define ADR_CDUX     00032u /* 0032       CDUX */
#define ADR_CDUS     00036u /* 0036       CDUS */
#define ADR_STATE    00074u /* 0074       STATE */
#define ADR_RADMODES 00110u /* 0110       RADMODES */
#define ADR_LASTYCMD 00112u /* 0112       LASTYCMD */
#define ADR_REDOCTR  00320u /* 0320       REDOCTR */
#define ADR_DELV     00324u /* 0324       DELV */
#define ADR_DNTMBUFF 00340u /* 0340       DNTMBUFF */
#define ADR_CADRFLSH 00372u /* 0372       CADRFLSH */
#define ADR_AOTCODE  00735u /* 0735       AOTCODE */
#define ADR_DSPTAB   01023u /* 1023       DSPTAB */
#define ADR_COMPNUMB 01170u /* 1170       COMPNUMB */
#define ADR_UPBUFF   01174u /* 1174       UPBUFF */
#define ADR_RN       01220u /* 1220       RN */
#define ADR_VN       01226u /* 1226       VN */
#define ADR_PIPTIME  01234u /* 1234       PIPTIME */
#define ADR_IMODES30 01302u /* 1302       IMODES30 */
#define ADR_LEMMASS  01331u /* 1331       LEMMASS */
#define ADR_DNRRANGE 01333u /* 1333       DNRRANGE */
#define ADR_DNLRVELX 01336u /* 1336       DNLRVELX */
#define ADR_DNLRVELZ 01340u /* 1340       DNLRVELZ */
#define ADR_TEVENT   01344u /* 1344       TEVENT */
#define ADR_RSBBQ    01432u /* E3,1432    RSBBQ */
#define ADR_T_OTHER  01570u /* E3,1570    T-OTHER */
#define ADR_X789     01700u /* E3,1700    X789 */
#define ADR_R_OTHER  01717u /* E3,1717    R-OTHER */
#define ADR_V_OTHER  01725u /* E3,1725    V-OTHER */
#define ADR_REFSMMAT 01733u /* E3,1733    REFSMMAT */
#define ADR_TCDH     01776u /* E3,1776    TCDH */
#define ADR_AGSK     02020u /* E4,1420    AGSK */
#define ADR_RLS      02022u /* E4,1422    RLS */
#define ADR_AGSBUFF  02200u /* E4,1600    AGSBUFF */
#define ADR_GSAV     02230u /* E4,1630    GSAV */
#define ADR_YNBSAV   02236u /* E4,1636    YNBSAV */
#define ADR_ELEV     02256u /* E4,1656    ELEV */
#define ADR_AT       02262u /* E4,1662    AT */
#define ADR_DELVEET1 02266u /* E4,1666    DELVEET1 */
#define ADR_DELVEET2 02274u /* E4,1674    DELVEET2 */
#define ADR_DELTAR   02302u /* E4,1702    DELTAR */
#define ADR_LRXCDUDL 02334u /* E4,1734    LRXCDUDL */
#define ADR_LRZCDUDL 02336u /* E4,1736    LRZCDUDL */
#define ADR_LRVTIMDL 02337u /* E4,1737    LRVTIMDL */
#define ADR_DELVTPF  02347u /* E4,1747    DELVTPF */
#define ADR_DELVEET3 02365u /* E4,1765    DELVEET3 */
#define ADR_TLAND    02400u /* E5,1400    TLAND */
#define ADR_RGU      02636u /* E5,1636    RGU */
#define ADR_OGC      02737u /* E5,1737    OGC */
#define ADR_BESTI    02755u /* E5,1755    BESTI */
#define ADR_STARSAV1 02760u /* E5,1760    STARSAV1 */
#define ADR_STARSAV2 02766u /* E5,1766    STARSAV2 */
#define ADR_TALIGN   02774u /* E5,1774    TALIGN */
#define ADR_OMEGAP   03021u /* E6,1421    OMEGAP */
#define ADR_ALPHAQ   03024u /* E6,1424    ALPHAQ */
#define ADR_POSTORKP 03113u /* E6,1513    POSTORKP */
#define ADR_POSTORKU 03115u /* E6,1515    POSTORKU */
#define ADR_CDUXD    03235u /* E6,1635    CDUXD */
#define ADR_OMEGAPD  03243u /* E6,1643    OMEGAPD */
#define ADR_UNFC_2   03253u /* E6,1653    UNFC/2 */
#define ADR_DELVSLV  03433u /* E7,1433    DELVSLV */
#define ADR_TIG      03441u /* E7,1441    TIG */
#define ADR_RTARG    03443u /* E7,1443    RTARG */
#define ADR_DELLT4   03451u /* E7,1451    DELLT4 */
#define ADR_TTOGO    03453u /* E7,1453    TTOGO */
#define ADR_AIG      03457u /* E7,1457    AIG */
#define ADR_AOG      03461u /* E7,1461    AOG */
#define ADR_NN       03466u /* E7,1466    NN */
#define ADR_TGO      03516u /* E7,1516    TGO */
#define ADR_PIPTIME1 03560u /* E7,1560    PIPTIME1 */
#define ADR_DIFFALT  03577u /* E7,1577    DIFFALT */
#define ADR_PSEUDO55 03614u /* E7,1614    PSEUDO55 */
#define ADR_FC       03615u /* E7,1615    FC */
#define ADR_CENTANG  03620u /* E7,1620    CENTANG */
#define ADR_VGU      03626u /* E7,1626    VGU */
#define ADR_TPASS4   03630u /* E7,1630    TPASS4 */
#define ADR_TCSI     03633u /* E7,1633    TCSI */
#define ADR_LAND     03634u /* E7,1634    LAND */
#define ADR_TTPI     03635u /* E7,1635    TTPI */
#define ADR_TTF_8    03642u /* E7,1642    TTF/8 */
#define ADR_VGVECT   03645u /* E7,1645    VGVECT */
#define ADR_VSELECT  03651u /* E7,1651    VSELECT */
#define ADR_VMEAS    03652u /* E7,1652    VMEAS */
#define ADR_HMEAS    03654u /* E7,1654    HMEAS */
#define ADR_DELTAH   03664u /* E7,1664    DELTAH */
#define ADR_VGTIG    03700u /* E7,1700    VGTIG */
#define ADR_TANGNB   03752u /* E7,1752    TANGNB */
#define ADR_MKTIME   03754u /* E7,1754    MKTIME */
#define ADR_RM       03756u /* E7,1756    RM */
#define ADR_RANGRDOT 03760u /* E7,1760    RANGRDOT */

#define PTR AGC_DN_OP_PTR  /* DNPTR: a sublist        */
#define CHN AGC_DN_OP_CHAN /* DNCHAN: channels n, n+1 */

/* ---------------- LM ORBITAL MANEUVERS LIST (LMORBMDL), p.194 ---------- */
static const agc_dn_entry_t lmorbmdl[] = {
    {PTR, false, LMORBM01},   /* COLLECT SNAPSHOT          */
    {6, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {1, false, ADR_DELLT4},   /* DELLT4,+1                 */
    {3, false, ADR_RTARG},    /* RTARG,+1...+5             */
    {1, false, ADR_ELEV},     /* ELEV,+1                   */
    {1, false, ADR_TEVENT},   /* TEVENT,+1                 */
    {6, false, ADR_REFSMMAT}, /* REFSMMAT +0...+11D        */
    {1, false, ADR_TCSI},     /* TCSI,+1                   */
    {3, false, ADR_DELVEET1}, /* DELVEET1 +0...+5          */
    {3, false, ADR_VGTIG},    /* VGTIG +0...+5             */
    {1, false, ADR_DNLRVELZ}, /* DNLRVELZ,DNLRALT          */
    {1, false, ADR_TPASS4},   /* TPASS4,+1                 */
    {PTR, false, LMORBM02},   /* COMMON DATA               */
    {1, false, ADR_TIME2},    /* TIME2/1                   */
    {PTR, false, LMORBM03},   /* COLLECT SNAPSHOT          */
    {6, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04},   /* COMMON DATA               */
    {2, false, ADR_POSTORKU}, /* POSTORKU,NEGTORKU,..V     */
    {1, false, ADR_SPARE},    /* SPARE */
    {1, false, ADR_TCDH},     /* TCDH,+1                   */
    {3, false, ADR_DELVEET2}, /* DELVEET2 +0...+5          */
    {1, false, ADR_TTPI},     /* TTPI,+1                   */
    {3, false, ADR_DELVEET3}, /* DELVEET3 +0...+5          */
    {1, false, ADR_DNRRANGE}, /* DNRRANGE,DNRRDOT          */
    {2, false, ADR_DNLRVELX}, /* DNLRVELX,Y,Z,DNLRALT      */
    {1, false, ADR_DIFFALT},  /* DIFFALT,+1                */
    {1, false, ADR_LEMMASS},  /* LEMMASS,CSMMASS           */
    {1, false, ADR_IMODES30}, /* IMODES30,IMODES33         */
    {1, false, ADR_TIG},      /* TIG,+1                    */
    {PTR, false, LMORBM05},   /* COMMON DATA               */
    {PTR, false, LMORBM06},   /* COMMON DATA               */
    {1, false, ADR_SPARE},    /* FORMERLY PIF              */
    {1, true, ADR_TGO}        /* TGO,+1                    */
};

/* ---------------- sublists, pp.194-195 -------------------------------- */
static const agc_dn_entry_t lmorbm01[] = {
    /* SNAPSHOT                   */
    {1, true, ADR_R_OTHER + 2u},  /* R-OTHER +2,+3             */
    {1, false, ADR_R_OTHER + 4u}, /* R-OTHER +4,+5             */
    {1, false, ADR_V_OTHER},      /* V-OTHER,+1                */
    {1, false, ADR_V_OTHER + 2u}, /* V-OTHER +2,+3             */
    {1, false, ADR_V_OTHER + 4u}, /* V-OTHER +4,+5             */
    {1, false, ADR_T_OTHER},      /* T-OTHER,+1                */
    {1, true, ADR_R_OTHER}        /* R-OTHER +0,+1             */
};

static const agc_dn_entry_t lmorbm02[] = {
    /* COMMON DATA                */
    {2, false, ADR_REDOCTR}, /* REDOCTR,THETAD,+1,+2      */
    {1, false, ADR_RSBBQ},   /* RSBBQ,+1                  */
    {2, false, ADR_OMEGAP},  /* OMEGAP,Q,R,GARBAGE        */
    {2, false, ADR_CDUXD},   /* CDUXD,CDUYD,CDUZD,GARBAGE */
    {2, false, ADR_CDUX},    /* CDUX,CDUY,CDUZ,CDUT       */
    {6, false, ADR_STATE},   /* STATE +0...+11D           */
    {6, true, ADR_DSPTAB}    /* DSPTAB TABLES             */
};

static const agc_dn_entry_t lmorbm03[] = {
    /* SNAPSHOT                   */
    {1, true, ADR_RN + 2u},  /* RN +2,+3                  */
    {1, false, ADR_RN + 4u}, /* RN +4,+5                  */
    {1, false, ADR_VN},      /* VN,+1                     */
    {1, false, ADR_VN + 2u}, /* VN +2,+3                  */
    {1, false, ADR_VN + 4u}, /* VN +4,+5                  */
    {1, false, ADR_PIPTIME}, /* PIPTIME,+1                */
    {1, true, ADR_RN}        /* RN,+1                     */
};

static const agc_dn_entry_t lmorbm04[] = {
    /* COMMON DATA                */
    {2, false, ADR_OMEGAPD},  /* OMEGAPD,QD,RD,GARBAGE     */
    {3, false, ADR_CADRFLSH}, /* CADRFLSH,+1,+2,FAILREG..  */
    {1, true, ADR_RADMODES}   /* RADMODES,DAPBOOLS         */
};

static const agc_dn_entry_t lmorbm05[] = {
    /* COMMON DATA                */
    {2, false, ADR_OMEGAP},   /* OMEGAP,Q,R,GARBAGE        */
    {2, false, ADR_CDUXD},    /* CDUXD,CDUYD,CDUZD,GARBAGE */
    {2, false, ADR_CDUX},     /* CDUX,CDUY,CDUZ,CDUT       */
    {1, false, ADR_ALPHAQ},   /* ALPHAQ,ALPHAR             */
    {1, false, ADR_POSTORKP}, /* POSTORKP,NEGTORKP         */
    {CHN, false, 011},        /* CHANNELS 11,12            */
    {CHN, false, 013},        /* CHANNELS 13,14            */
    {CHN, false, 030},        /* CHANNELS 30,31            */
    {CHN, true, 032}          /* CHANNELS 32,33            */
};

static const agc_dn_entry_t lmorbm06[] = {
    /* COMMON DATA                */
    {1, false, ADR_PIPTIME1}, /* PIPTIME,+1                */
    {3, true, ADR_DELV}       /* DELV +0...+5              */
};

/* ---------------- LM COAST AND ALIGNMENT LIST (LMCSTADL), p.196 -------- */
static const agc_dn_entry_t lmcstadl[] = {
    {PTR, false, LMORBM01},   /* LMCSTA01: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {1, false, ADR_AGSK},     /* AGSK,+1                   */
    {1, false, ADR_TALIGN},   /* TALIGN,+1                 */
    {2, false, ADR_POSTORKU}, /* POSTORKU,NEGTORKU,..V     */
    {1, false, ADR_DNRRANGE}, /* DNRRANGE,DNRRDOT          */
    {1, false, ADR_TEVENT},   /* TEVENT,+1                 */
    {6, false, ADR_REFSMMAT}, /* REFSMMAT +0...+11D        */
    {1, false, ADR_AOTCODE},  /* AOTCODE,GARBAGE           */
    {3, false, ADR_RLS},      /* RLS +0...+5               */
    {2, false, ADR_DNLRVELX}, /* DNLRVELX,Y,Z,DNLRALT      */
    {PTR, false, LMCSTA06},   /* COMMON DATA               */
    {PTR, false, LMORBM02},   /* LMCSTA02: COMMON DATA     */
    {1, false, ADR_TIME2},    /* TIME2/1                   */
    {PTR, false, LMORBM03},   /* LMCSTA03: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04},   /* LMCSTA04: COMMON DATA     */
    {PTR, false, LMCSTA07},   /* COMMON DATA               */
    {2, false, ADR_DNLRVELX}, /* DNLRVELX,Y,Z,DNLRALT      */
    {2, false, ADR_CDUS},     /* CDUS,PIPAX,PIPAY,PIPAZ    */
    {1, false, ADR_LASTYCMD}, /* LASTYCMD,LASTXCMD         */
    {1, false, ADR_LEMMASS},  /* LEMMASS,CSMMASS           */
    {1, false, ADR_IMODES30}, /* IMODES30,IMODES33         */
    {1, false, ADR_TIG},      /* TIG,+1                    */
    {PTR, false, LMORBM05},   /* LMCSTA05: COMMON DATA     */
    {6, true, ADR_DSPTAB}     /* DSPTAB +0...+11D TABLE    */
};

static const agc_dn_entry_t lmcsta06[] = {
    /* COMMON DATA, p.196         */
    {2, false, ADR_X789},   /* X789 +0...+3              */
    {1, true, ADR_LASTYCMD} /* LASTYCMD,LASTXCMD         */
};

static const agc_dn_entry_t lmcsta07[] = {
    /* COMMON DATA                */
    {3, false, ADR_OGC},      /* OGC,+1,IGC,+1,MGC,+1      */
    {1, false, ADR_BESTI},    /* BESTI,BESTJ               */
    {3, false, ADR_STARSAV1}, /* STARSAV1 +0...+5          */
    {3, true, ADR_STARSAV2}   /* STARSAV2 +0...+5          */
};

/* ---------------- LM RENDEZVOUS AND PRETHRUST LIST (LMRENDDL), p.198 --- */
static const agc_dn_entry_t lmrenddl[] = {
    {PTR, false, LMORBM01},   /* LMREND01: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {PTR, false, LMREND07},   /* COLLECT SNAPSHOT          */
    {4, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {1, false, ADR_DELLT4},   /* DELLT4,+1                 */
    {3, false, ADR_RTARG},    /* RTARG +0...+5             */
    {3, false, ADR_DELVSLV},  /* DELVSLV +0...+5           */
    {1, false, ADR_TCSI},     /* TCSI,+1                   */
    {3, false, ADR_DELVEET1}, /* DELVEET +0...+5           */
    {1, false, ADR_SPARE},    /* SPARE */
    {1, false, ADR_TPASS4},   /* TPASS4,+1                 */
    {PTR, false, LMCSTA06},   /* LMREND06: COMMON DATA     */
    {PTR, false, LMORBM02},   /* LMREND02: COMMON DATA     */
    {1, false, ADR_TIME2},    /* TIME2/1                   */
    {PTR, false, LMORBM03},   /* LMREND03: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04},   /* LMREND04: COMMON DATA     */
    {2, false, ADR_POSTORKU}, /* POSTORKU,NEGTORKU,..V     */
    {1, false, ADR_SPARE},    /* SPARE */
    {1, false, ADR_TCDH},     /* TCDH,+1                   */
    {3, false, ADR_DELVEET2}, /* DELVEET2 +0...+5          */
    {1, false, ADR_TTPI},     /* TTPI,+1                   */
    {3, false, ADR_DELVEET3}, /* DELVEET3 +0...+5          */
    {1, false, ADR_ELEV},     /* ELEV,+1                   */
    {2, false, ADR_CDUS},     /* CDUS,PIPAX,PIPAY,PIPAZ    */
    {1, false, ADR_LASTYCMD}, /* LASTYCMD,LASTXCMD         */
    {1, false, ADR_LEMMASS},  /* LEMMASS,CSMMASS           */
    {1, false, ADR_IMODES30}, /* IMODES30,IMODES33         */
    {1, false, ADR_TIG},      /* TIG,+1                    */
    {PTR, false, LMORBM05},   /* LMREND05: COMMON DATA     */
    {1, false, ADR_DELTAR},   /* DELTAR,+1                 */
    {1, false, ADR_CENTANG},  /* CENTANG,+1                */
    {1, false, ADR_NN},       /* NN,+1                     */
    {1, false, ADR_DIFFALT},  /* DIFFALT,+1                */
    {1, false, ADR_DELVTPF},  /* DELVTPF,+1                */
    {1, true, ADR_SPARE}      /* SPARE */
};

static const agc_dn_entry_t lmrend07[] = {
    /* SNAPSHOT, p.199            */
    {1, true, ADR_AIG},     /* AIG,AMG                   */
    {1, false, ADR_AOG},    /* AOG,TRKMKCNT              */
    {1, false, ADR_TANGNB}, /* TANGNB,+1                 */
    {1, false, ADR_MKTIME}, /* MKTIME,+1                 */
    {1, true, ADR_RANGRDOT} /* DNRRANGE,DNRRDOT          */
};

/* ---------------- LM DESCENT AND ASCENT LIST (LMDSASDL), p.200 --------- */
static const agc_dn_entry_t lmdsasdl[] = {
    {PTR, false, LMDSAS07},   /* COLLECT SNAPSHOT          */
    {PTR, false, LMDSAS08},   /* SEND SNAPSHOT             */
    {1, false, ADR_TEVENT},   /* TEVENT,+1                 */
    {3, false, ADR_UNFC_2},   /* UNFC/2 +0...+5            */
    {3, false, ADR_VGVECT},   /* VGVECT +0...+5            */
    {1, false, ADR_TTF_8},    /* TTF/8,+1                  */
    {1, false, ADR_DELTAH},   /* DELTAH,+1                 */
    {3, false, ADR_RLS},      /* RLS +0...+5               */
    {1, false, ADR_SPARE},    /* SPARE */
    {PTR, false, LMCSTA06},   /* LMDSAS09: COMMON DATA     */
    {PTR, false, LMORBM02},   /* LMDSAS02: COMMON DATA     */
    {1, false, ADR_TIME2},    /* TIME2/1                   */
    {PTR, false, LMORBM03},   /* LMDSAS03: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04},   /* LMDSAS04: COMMON DATA     */
    {2, false, ADR_POSTORKU}, /* POSTORKU,NEGTORKU,..V     */
    {3, false, ADR_RGU},      /* RGU +0...+5               */
    {3, false, ADR_VGU},      /* VGU +0...+5               */
    {3, false, ADR_LAND},     /* LAND +0...+5              */
    {1, false, ADR_AT},       /* AT,+1                     */
    {1, false, ADR_TLAND},    /* TLAND,+1                  */
    {1, false, ADR_FC},       /* FC,GARBAGE                */
    {1, false, ADR_LASTYCMD}, /* LASTYCMD,LASTXCMD         */
    {1, false, ADR_LEMMASS},  /* LEMMASS,CSMMASS           */
    {1, false, ADR_IMODES30}, /* IMODES30,IMODES33         */
    {1, false, ADR_TIG},      /* TIG,+1                    */
    {PTR, false, LMORBM05},   /* LMDSAS05: COMMON DATA     */
    {PTR, false, LMORBM06},   /* LMDSAS06: COMMON DATA     */
    {1, false, ADR_PSEUDO55}, /* PSEUDO55,GARBAGE          */
    {1, true, ADR_TTOGO}      /* TTOGO,+1                  */
};

static const agc_dn_entry_t lmdsas07[] = {
    /* SNAPSHOT, p.200            */
    {1, true, ADR_LRZCDUDL},  /* LRZCDUDL,GARBAGE          */
    {1, false, ADR_VSELECT},  /* VSELECT,GARBAGE           */
    {1, false, ADR_LRVTIMDL}, /* LRVTIMDL,+1               */
    {1, false, ADR_VMEAS},    /* VMEAS,+1                  */
    {1, false, ADR_MKTIME},   /* MKTIME,+1                 */
    {1, false, ADR_HMEAS},    /* HMEAS,+1                  */
    {1, false, ADR_RM},       /* RM,+1                     */
    {1, false, ADR_AIG},      /* AIG,AMG                   */
    {1, false, ADR_AOG},      /* AOG,TRKMKCNT              */
    {1, false, ADR_TANGNB},   /* TANGNB,+1                 */
    {1, false, ADR_MKTIME},   /* MKTIME,+1                 */
    {1, true, ADR_LRXCDUDL}   /* LRXCDUDL,LRYCDUDL         */
};

static const agc_dn_entry_t lmdsas08[] = { /* SEND SNAPSHOT              */
    {6, false, ADR_DNTMBUFF},     /* SEND SNAPSHOT */
    {5, true, ADR_DNTMBUFF + 12u} /* SEND SNAPSHOT */
};

/* ---------------- LM LUNAR SURFACE ALIGN LIST (LMLSALDL), p.202 -------- */
static const agc_dn_entry_t lmlsaldl[] = {
    {PTR, false, LMORBM01},   /* LMLSAL01: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {PTR, false, LMREND07},   /* LMLSAL07: COLLECT SNAPSHOT */
    {4, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {1, false, ADR_TALIGN},   /* TALIGN,+1                 */
    {6, false, ADR_REFSMMAT}, /* REFSMMAT +0...+11D        */
    {6, false, ADR_YNBSAV},   /* YNBSAV +0..+5,SNBSAV +0..+5 */
    {PTR, false, LMCSTA06},   /* LMLSAL08: COMMON DATA     */
    {PTR, false, LMORBM02},   /* LMLSAL02: COMMON DATA     */
    {1, false, ADR_TIME2},    /* TIME2/1                   */
    {PTR, false, LMORBM03},   /* LMLSAL03: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04},   /* LMLSAL04: COMMON DATA     */
    {PTR, false, LMCSTA07},   /* LMLSAL09: COMMON DATA     */
    {3, false, ADR_GSAV},     /* GSAV +0...+5              */
    {1, false, ADR_AGSK},     /* AGSK,+1                   */
    {1, false, ADR_LASTYCMD}, /* LASTYCMD,LASTXCMD         */
    {1, false, ADR_LEMMASS},  /* LEMMASS,CSMMASS           */
    {1, false, ADR_IMODES30}, /* IMODES30,IMODES33         */
    {1, false, ADR_TIG},      /* TIG,+1                    */
    {PTR, false, LMORBM05},   /* LMLSAL05: COMMON DATA     */
    {PTR, false, LMORBM06},   /* LMLSAL06: COMMON DATA     */
    {1, false, ADR_SPARE}, {1, true, ADR_SPARE}
};

/* ---------------- LM AGS INITIALIZATION / UPDATE LIST (LMAGSIDL), p.204 */
static const agc_dn_entry_t lmagsidl[] = {
    {3, false, ADR_AGSBUFF},       /* AGSBUFF +0...+5           */
    {1, false, ADR_AGSBUFF + 12u}, /* AGSBUFF +12D,GARBAGE      */
    {3, false, ADR_AGSBUFF + 1u},  /* AGSBUFF +1...+6           */
    {1, false, ADR_AGSBUFF + 13u}, /* AGSBUFF +13D,GARBAGE      */
    {3, false, ADR_AGSBUFF + 6u},  /* AGSBUFF +6...+11          */
    {1, false, ADR_AGSBUFF + 12u}, /* AGSBUFF +12D,GARBAGE      */
    {3, false, ADR_AGSBUFF + 7u},  /* AGSBUFF +7...+12D         */
    {1, false, ADR_AGSBUFF + 13u}, /* AGSBUFF +13D,GARBAGE      */
    {6, false, ADR_COMPNUMB},      /* COMPNUMB..UPCOUNT,UPBUFF +0..+7 */
    {6, false, ADR_UPBUFF + 8u},   /* UPBUFF +8D...+19D         */
    {PTR, false, LMORBM02},        /* LMAGSI02: COMMON DATA     */
    {1, false, ADR_TIME2},         /* TIME2/1                   */
    {PTR, false, LMORBM03},        /* LMAGSI03: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF},      /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04},        /* LMAGSI04: COMMON DATA     */
    {2, false, ADR_POSTORKU},      /* POSTORKU,NEGTORKU,..V     */
    {1, false, ADR_SPARE},         /* SPARE */
    {1, false, ADR_SPARE},         /* SPARE */
    {1, false, ADR_AGSK},          /* AGSK,+1                   */
    {6, false, ADR_UPBUFF},        /* UPBUFF +0...+11D          */
    {4, false, ADR_UPBUFF + 12u},  /* UPBUFF +12D...+19D        */
    {1, false, ADR_LEMMASS},       /* LEMMASS,CSMMASS           */
    {1, false, ADR_IMODES30},      /* IMODES30,IMODES33         */
    {1, false, ADR_SPARE},         /* SPARE */
    {PTR, false, LMORBM05},        /* LMAGSI05: COMMON DATA     */
    {6, true, ADR_DSPTAB}          /* DSPTAB +0...+11D          */
};

#undef PTR
#undef CHN

/* DNTABLE (p.204): index = DNLSTCOD.                                      */
static const agc_dn_list_t dntable[AGC_DNLIST_LM_COUNT] = {
    {lmcstadl, (uint8_t) (sizeof (lmcstadl) / sizeof (lmcstadl[0])), false}, /* 0 COAST/ALIGN (NOMDNLST) */
    {lmagsidl, (uint8_t) (sizeof (lmagsidl) / sizeof (lmagsidl[0])), false}, /* 1 AGS INIT/UPDATE (UPDNLIST) */
    {lmrenddl, (uint8_t) (sizeof (lmrenddl) / sizeof (lmrenddl[0])), false}, /* 2 RENDEZVOUS/PRETHRUST */
    {lmorbmdl, (uint8_t) (sizeof (lmorbmdl) / sizeof (lmorbmdl[0])), false}, /* 3 ORBITAL MANEUVERS */
    {lmdsasdl, (uint8_t) (sizeof (lmdsasdl) / sizeof (lmdsasdl[0])), false}, /* 4 DESCENT/ASCENT */
    {lmlsaldl, (uint8_t) (sizeof (lmlsaldl) / sizeof (lmlsaldl[0])), false}  /* 5 SURFACE ALIGN */
};

static const agc_dn_list_t sublists[LM_SUB_COUNT] = {
    {lmorbm01, (uint8_t) (sizeof (lmorbm01) / sizeof (lmorbm01[0])), true},
    {lmorbm02, (uint8_t) (sizeof (lmorbm02) / sizeof (lmorbm02[0])), false},
    {lmorbm03, (uint8_t) (sizeof (lmorbm03) / sizeof (lmorbm03[0])), true},
    {lmorbm04, (uint8_t) (sizeof (lmorbm04) / sizeof (lmorbm04[0])), false},
    {lmorbm05, (uint8_t) (sizeof (lmorbm05) / sizeof (lmorbm05[0])), false},
    {lmorbm06, (uint8_t) (sizeof (lmorbm06) / sizeof (lmorbm06[0])), false},
    {lmcsta06, (uint8_t) (sizeof (lmcsta06) / sizeof (lmcsta06[0])), false},
    {lmcsta07, (uint8_t) (sizeof (lmcsta07) / sizeof (lmcsta07[0])), false},
    {lmrend07, (uint8_t) (sizeof (lmrend07) / sizeof (lmrend07[0])), true},
    {lmdsas07, (uint8_t) (sizeof (lmdsas07) / sizeof (lmdsas07[0])), true},
    {lmdsas08, (uint8_t) (sizeof (lmdsas08) / sizeof (lmdsas08[0])), false}
};

/**********************************************************************
 * DNTABLE (Luminary099 DOWNLINK_LISTS.agc pp.193-205) -- INDEX DNLSTCOD,
 * CA DNTABLE: the control list of a downlist
 *
 * ARGUMENTS:
 *   code  int              I   DNLSTCOD, 0..AGC_DNLIST_LM_COUNT-1
 *   out   agc_dn_list_t *  O   the list
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for a code outside DNTABLE (DNLSTCOD
 *          comes from erasable: V21, AGSWORD)
 **********************************************************************/
agc_status_t agc_dnlist_lm_control(int code, agc_dn_list_t *out)
{
    if (!AGC_ASSERT(code >= 0 && code < AGC_DNLIST_LM_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *out = dntable[code];

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * DNPTR (Luminary099 DOWNLINK_LISTS.agc pp.193-205) -- the sublist a
 * DNPTR entry names
 *
 * ARGUMENTS:
 *   sub  int              I   sublist id (enum above)
 *   out  agc_dn_list_t *  O   the list
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for an id outside the sublist table
 **********************************************************************/
agc_status_t agc_dnlist_lm_sublist(int sub, agc_dn_list_t *out)
{
    if (!AGC_ASSERT(sub >= 0 && sub < LM_SUB_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *out = sublists[sub];

    return (AGC_STATUS_OK);
}
