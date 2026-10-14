/**********************************************************************
 * FILE NAME: downlink_lists_csm.c
 *
 * PURPOSE:
 *   The CSM downlists as data: DNTABLE (control list per DNLSTCOD) and the
 *   sublists the control lists point at, read by the down-telemetry
 *   program (src/io/telemetry/down_telemetry.c).
 *
 * AGC SOURCE:
 *   Comanche055 DOWNLINK_LISTS.agc pp.170-180
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
 *   Each AGC list word becomes one agc_dn_entry_t, the rope's DNADR:
 *     { op, neg, ref }
 *     op     1..6 = 1DNADR..6DNADR (n DP words), AGC_DN_OP_CHAN (DNCHAN),
 *            AGC_DN_OP_PTR (DNPTR, ref = sublist id below)
 *     neg    the AGC wrote the entry negated (-1DNADR, -DNCHAN ...): end of
 *            list, or (first entry of a sublist) a snapshot sublist
 *     ref    the ECADR of the first word (RN +2 -> ADR_RN + 2u; STATE +10D
 *            -> ADR_STATE + 10u) / channel number / sublist id
 *   The ADR_ addresses are the rope's (Comanche055 ERASABLE_ASSIGNMENTS);
 *   SPARE is ERASZERO (register 7, always +0).
 *
 *   Downlist rules (from the AGC header):
 *   1. END OF A LIST = -XDNADR (X = 1 TO 6), -DNPTR, OR -DNCHAN.
 *   2. SNAPSHOT SUBLIST = LIST WHICH STARTS WITH A -1DNADR.
 *   3. SNAPSHOT SUBLIST CAN ONLY CONTAIN 1DNADRS.
 *   4. TIME2 1DNADR MUST BE LOCATED IN THE CONTROL LIST OF A DOWNLIST.
 *   6. ... IN THE SNAPSHOT SUBLISTS THE DATA REPRESENTED BY THE FIRST 11
 *      1DNADRS IS PRESERVED (IN ORDER) IN DNTMBUFF AND SENT BY THE NEXT 11
 *      DOWNRUPTS.
 *
 *   The "(28-29)"-style numbers in the AGC comments are DP word
 *   positions in the 100-word downlist; word 1 is the ID/sync pair sent
 *   by NEWLIST. The EQUALS aliases (CMCSTA01 EQUALS CMPOWE01 etc.)
 *   become references to the same sublist id.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/io/service.h"

/* Sublist ids (the AGC labels).                                           */
enum
{
    CMPOWE01 = 0,
    CMPOWE02,
    CMPOWE03,
    CMPOWE04,
    CMPOWE05,
    CMPOWE06,
    CMPOWE07,
    CMENTR05,
    CM_LANDMARK_SNAPSHOT,
    CM_SUB_COUNT
};

/* The erasables the lists name: ECADR (EEE RRRRRRRR) from the rope's
 * symbol table (yaYUL), bank and address as the listing gives them.      */
#define ADR_SPARE    00007u /* 0007       SPARE */
#define ADR_TIME2    00024u /* 0024       TIME2 */
#define ADR_CDUX     00032u /* 0032       CDUX */
#define ADR_CDUZ     00034u /* 0034       CDUZ */
#define ADR_CDUS     00036u /* 0036       CDUS */
#define ADR_STATE    00074u /* 0074       STATE */
#define ADR_COMPNUMB 00300u /* 0300       COMPNUMB */
#define ADR_BESTI    00302u /* 0302       BESTI */
#define ADR_UPBUFF   00304u /* 0304       UPBUFF */
#define ADR_DNTMBUFF 00340u /* 0340       DNTMBUFF */
#define ADR_CADRFLSH 00372u /* 0372       CADRFLSH */
#define ADR_DSPTAB   01023u /* 1023       DSPTAB */
#define ADR_LAT      01103u /* 1103       LAT */
#define ADR_LONG     01105u /* 1105       LONG */
#define ADR_ALT      01107u /* 1107       ALT */
#define ADR_VHFCNT   01125u /* 1125       VHFCNT */
#define ADR_VHFTIME  01151u /* 1151       VHFTIME */
#define ADR_REDOCTR  01154u /* 1154       REDOCTR */
#define ADR_DELV     01162u /* 1162       DELV */
#define ADR_RN       01170u /* 1170       RN */
#define ADR_VN       01176u /* 1176       VN */
#define ADR_PIPTIME  01204u /* 1204       PIPTIME */
#define ADR_PIPTIME1 01245u /* 1245       PIPTIME1 */
#define ADR_IMODES30 01320u /* 1320       IMODES30 */
#define ADR_OPTMODES 01331u /* 1331       OPTMODES */
#define ADR_TEVENT   01336u /* 1336       TEVENT */
#define ADR_RSBBQ    01432u /* E3,1432    RSBBQ */
#define ADR_T_OTHER  01642u /* E3,1642    T-OTHER */
#define ADR_R_OTHER  01721u /* E3,1721    R-OTHER */
#define ADR_V_OTHER  01727u /* E3,1727    V-OTHER */
#define ADR_REFSMMAT 01735u /* E3,1735    REFSMMAT */
#define ADR_RLS      02025u /* E4,1425    RLS */
#define ADR_RANGE    02320u /* E4,1720    RANGE */
#define ADR_RTHETA   02324u /* E4,1724    RTHETA */
#define ADR_HAPOX    02351u /* E4,1751    HAPOX */
#define ADR_DELVEET1 02444u /* E5,1444    DELVEET1 */
#define ADR_DELTAR   02610u /* E5,1610    DELTAR */
#define ADR_DIFFALT  02632u /* E5,1632    DIFFALT */
#define ADR_LAUNCHAZ 02633u /* E5,1633    LAUNCHAZ */
#define ADR_TCDH     02634u /* E5,1634    TCDH */
#define ADR_DELVTPF  02636u /* E5,1636    DELVTPF */
#define ADR_8NN      02747u /* E5,1747    8NN */
#define ADR_LANDMARK 02751u /* E5,1751    LANDMARK */
#define ADR_OGC      02757u /* E5,1757    OGC */
#define ADR_PACTOFF  03023u /* E6,1423    PACTOFF */
#define ADR_DAPDATR1 03066u /* E6,1466    DAPDATR1 */
#define ADR_LEMMASS  03073u /* E6,1473    LEMMASS */
#define ADR_AK       03076u /* E6,1476    AK */
#define ADR_WBODY    03125u /* E6,1525    WBODY */
#define ADR_ADOT     03133u /* E6,1533    ADOT */
#define ADR_ERRORX   03167u /* E6,1567    ERRORX */
#define ADR_THETADX  03172u /* E6,1572    THETADX */
#define ADR_PCMD     03223u /* E6,1623    PCMD */
#define ADR_ALFA_180 03265u /* E6,1665    ALFA/180 */
#define ADR_CMDAPMOD 03300u /* E6,1700    CMDAPMOD */
#define ADR_PAXERR1  03313u /* E6,1713    PAXERR1 */
#define ADR_ROLLTM   03314u /* E6,1714    ROLLTM */
#define ADR_LAT_SPL  03400u /* E7,1400    LAT(SPL) */
#define ADR_DELVSLV  03404u /* E7,1404    DELVSLV */
#define ADR_TIG      03412u /* E7,1412    TIG */
#define ADR_RTARG    03414u /* E7,1414    RTARG */
#define ADR_DELLT4   03422u /* E7,1422    DELLT4 */
#define ADR_ECSTEER  03424u /* E7,1424    ECSTEER */
#define ADR_TGO      03427u /* E7,1427    TGO */
#define ADR_MARK2DWN 03502u /* E7,1502    MARK2DWN */
#define ADR_SVMRKDAT 03537u /* E7,1537    SVMRKDAT */
#define ADR_L_D1     03635u /* E7,1635    L/D1 */
#define ADR_DELVEET3 03645u /* E7,1645    DELVEET3 */
#define ADR_TPASS4   03655u /* E7,1655    TPASS4 */
#define ADR_TTPI     03662u /* E7,1662    TTPI */
#define ADR_DELVEET2 03666u /* E7,1666    DELVEET2 */
#define ADR_MARKDOWN 03674u /* E7,1674    MARKDOWN */
#define ADR_LATANG   03675u /* E7,1675    LATANG */
#define ADR_CSTEER   03702u /* E7,1702    CSTEER */
#define ADR_VGTIG    03720u /* E7,1720    VGTIG */
#define ADR_VIO      03724u /* E7,1724    VIO */
#define ADR_TTE      03726u /* E7,1726    TTE */
#define ADR_ELEV     03743u /* E7,1743    ELEV */
#define ADR_CENTANG  03753u /* E7,1753    CENTANG */
#define ADR_VPRED    03766u /* E7,1766    VPRED */
#define ADR_GAMMAEI  03770u /* E7,1770    GAMMAEI */

#define PTR AGC_DN_OP_PTR  /* DNPTR: a sublist        */
#define CHN AGC_DN_OP_CHAN /* DNCHAN: channels n, n+1 */

/* ---------------- CSM POWERED FLIGHT DOWNLIST (CMPOWEDL), p.171 -------- */
static const agc_dn_entry_t cmpowedl[] = {
    {PTR, false, CMPOWE01},      /* COLLECT SNAPSHOT          */
    {6, false, ADR_DNTMBUFF},    /* SEND SNAPSHOT             */
    {PTR, false, CMPOWE02},      /* COLLECT SECOND SNAPSHOT   */
    {4, false, ADR_DNTMBUFF},    /* SEND SNAPSHOT             */
    {PTR, false, CMPOWE03},      /* COMMON DATA               */
    {1, false, ADR_TIG},         /* TIG,+1                    */
    {1, false, ADR_DELLT4},      /* DELLT4,+1                 */
    {3, false, ADR_RTARG},       /* RTARG,+1,+2,...+5         */
    {1, false, ADR_TGO},         /* TGO,+1                    */
    {1, false, ADR_PIPTIME1},    /* PIPTIME1,+1               */
    {3, false, ADR_DELV},        /* DELV,+1,...+4,+5          */
    {1, false, ADR_PACTOFF},     /* PACTOFF,YACTOFF           */
    {1, false, ADR_PCMD},        /* PCMD,YCMD                 */
    {1, false, ADR_CSTEER},      /* CSTEER,+1                 */
    {3, false, ADR_DELVEET1},    /* CSI DELTA V (31-33)       */
    {6, false, ADR_REFSMMAT},    /* REFSMMAT,+1,...+10,+11    */
    {PTR, false, CMPOWE04},      /* COMMON DATA               */
    {1, false, ADR_TIME2},       /* TIME2,TIME1               */
    {PTR, false, CMPOWE05},      /* COLLECT SNAPSHOT          */
    {6, false, ADR_DNTMBUFF},    /* SEND SNAPSHOT             */
    {PTR, false, CMPOWE02},      /* COLLECT SNAPSHOT          */
    {4, false, ADR_DNTMBUFF},    /* SEND SNAPSHOT             */
    {PTR, false, CMPOWE03},      /* CMPOWE03: COMMON DATA */
    {PTR, false, CMPOWE06},      /* COMMON DATA               */
    {1, false, ADR_ELEV},        /* ELEV,+1                   */
    {1, false, ADR_CENTANG},     /* CENTANG,+1                */
    {1, false, ADR_DELTAR},      /* DELTAR,+1                 */
    {1, false, ADR_STATE + 10u}, /* FLAGWRDS 10 AND 11        */
    {1, false, ADR_TEVENT},      /* TEVENT,+1                 */
    {1, false, ADR_PCMD},        /* PCMD,YCMD                 */
    {1, false, ADR_OPTMODES},    /* OPTMODES,HOLDFLAG         */
    {PTR, false, CMPOWE07},      /* COMMON DATA               */
    {3, false, ADR_VGTIG},       /* VGTIG,+1,...+4,+5         */
    {3, true, ADR_DELVEET2}      /* CDH DELTA V (98-100)      */
};

/* ---------------- sublists, pp.171-172 -------------------------------- */
static const agc_dn_entry_t cmpowe01[] = {
    /* SNAPSHOT DATA              */
    {1, true, ADR_RN + 2u},  /* RN+2,+3                   */
    {1, false, ADR_RN + 4u}, /* RN+4,+5                   */
    {1, false, ADR_VN},      /* VN,+1                     */
    {1, false, ADR_VN + 2u}, /* VN+2,+3                   */
    {1, false, ADR_VN + 4u}, /* VN+4,+5                   */
    {1, false, ADR_PIPTIME}, /* PIPTIME,+1                */
    {1, true, ADR_RN}        /* RN,+1                     */
};

static const agc_dn_entry_t cmpowe02[] = {
    /* SNAPSHOT DATA              */
    {1, true, ADR_CDUZ},       /* CDUZ,CDUT                 */
    {1, false, ADR_ADOT},      /* ADOT,+1/OGARATE,+1        */
    {1, false, ADR_ADOT + 2u}, /* ADOT+2,+3/OMEGAB+2,+3     */
    {1, false, ADR_ADOT + 4u}, /* ADOT+4,+5/OMEGAB+4,+5     */
    {1, true, ADR_CDUX}        /* CDUX,CDUY                 */
};

static const agc_dn_entry_t cmpowe03[] = {
    /* COMMON DATA                */
    {2, false, ADR_AK},    /* AK,AK1,AK2,RCSFLAGS       */
    {2, true, ADR_THETADX} /* THETADX,Y,Z,GARBAGE       */
};

static const agc_dn_entry_t cmpowe04[] = {
    /* COMMON DATA                */
    {5, false, ADR_STATE}, /* FLAGWRD0 THRU FLAGWRD9    */
    {6, true, ADR_DSPTAB}  /* DISPLAY TABLES            */
};

static const agc_dn_entry_t cmpowe05[] = {
    /* SNAPSHOT DATA              */
    {1, true, ADR_R_OTHER + 2u},  /* R-OTHER+2,+3              */
    {1, false, ADR_R_OTHER + 4u}, /* R-OTHER+4,+5              */
    {1, false, ADR_V_OTHER},      /* V-OTHER,+1                */
    {1, false, ADR_V_OTHER + 2u}, /* V-OTHER+2,+3              */
    {1, false, ADR_V_OTHER + 4u}, /* V-OTHER+4,+5              */
    {1, false, ADR_T_OTHER},      /* T-OTHER,+1                */
    {1, true, ADR_R_OTHER}        /* R-OTHER,+1                */
};

static const agc_dn_entry_t cmpowe06[] = {
    /* COMMON DATA                */
    {1, false, ADR_RSBBQ},    /* RSBBQ,+1                  */
    {3, false, ADR_CADRFLSH}, /* CADRFLSH,+1,+2,FAILREG..  */
    {2, true, ADR_CDUS}       /* CDUS,PIPAX,PIPAY,PIPAZ    */
};

static const agc_dn_entry_t cmpowe07[] = {
    /* COMMON DATA                */
    {1, false, ADR_LEMMASS},  /* LEMMASS,CSMMASS           */
    {1, false, ADR_DAPDATR1}, /* DAPDATR1,DAPDATR2         */
    {2, false, ADR_ERRORX},   /* ERRORX,Y,Z,GARBAGE        */
    {3, false, ADR_WBODY},    /* WBODY,...+5/OMEGAC,...+5  */
    {2, false, ADR_REDOCTR},  /* REDOCTR,THETAD,+1,+2      */
    {1, false, ADR_IMODES30}, /* IMODES30,IMODES33         */
    {CHN, false, 011},        /* CHANNELS 11,12            */
    {CHN, false, 013},        /* CHANNELS 13,14            */
    {CHN, false, 030},        /* CHANNELS 30,31            */
    {CHN, true, 032}          /* CHANNELS 32,33            */
};

/* ---------------- CSM COAST AND ALIGNMENT DOWNLIST (CMCSTADL), p.173 --- */
static const agc_dn_entry_t cmcstadl[] = {
    {PTR, false, CMPOWE01},      /* CMCSTA01: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF},    /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02},      /* CMCSTA02                  */
    {4, false, ADR_DNTMBUFF},    /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03},      /* CMCSTA03                  */
    {1, false, ADR_TIG},         /* TIG,+1                    */
    {1, false, ADR_BESTI},       /* BESTI,BESTJ               */
    {4, false, ADR_MARKDOWN},    /* MARKDOWN,+1,...+6,GARBAGE */
    {4, false, ADR_MARK2DWN},    /* MARK2DWN,+1,...+6         */
    {2, false, ADR_HAPOX},       /* APOGEE, PERIGEE (28-29)   */
    {1, false, ADR_PACTOFF},     /* PACTOFF, YACTOFF (30)     */
    {3, false, ADR_VGTIG},       /* VGTIG,...+5               */
    {6, false, ADR_REFSMMAT},    /* REFSMMAT,+1,...+10,+11    */
    {PTR, false, CMPOWE04},      /* CMCSTA04                  */
    {1, false, ADR_TIME2},       /* TIME2,TIME1               */
    {PTR, false, CMPOWE05},      /* CMCSTA05                  */
    {6, false, ADR_DNTMBUFF},    /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02},      /* CMCSTA02: COLLECT SNAPSHOT */
    {4, false, ADR_DNTMBUFF},    /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03},      /* CMCSTA03: COMMON DATA */
    {PTR, false, CMPOWE06},      /* CMCSTA06                  */
    {3, false, ADR_OGC},         /* OGC,+1,IGC,+1,MGC,+1      */
    {1, false, ADR_STATE + 10u}, /* FLAGWRDS 10 AND 11        */
    {1, false, ADR_TEVENT},      /* TEVENT,+1                 */
    {1, false, ADR_LAUNCHAZ},    /* LAUNCHAZ,+1               */
    {1, false, ADR_OPTMODES},    /* OPTMODES,HOLDFLAG         */
    {PTR, false, CMPOWE07},      /* CMCSTA07                  */
    {6, true, ADR_DSPTAB}        /* DISPLAY TABLES            */
};

/* ---------------- CSM RENDEZVOUS AND PRETHRUST LIST (CMRENDDL), p.175 -- */
static const agc_dn_entry_t cmrenddl[] = {
    {PTR, false, CMPOWE01},    /* CMREND01: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF},  /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02},    /* CMREND02: COLLECT SNAPSHOT */
    {4, false, ADR_DNTMBUFF},  /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03},    /* CMREND03: COMMON DATA */
    {1, false, ADR_TIG},       /* TIG,+1                    */
    {1, false, ADR_DELLT4},    /* DELLT4,+1                 */
    {3, false, ADR_RTARG},     /* RTARG,+1,...+4,+5         */
    {1, false, ADR_VHFTIME},   /* VHFTIME,+1                */
    {4, false, ADR_MARKDOWN},  /* MARKTIME,YCDU,SCDU,...,RM */
    {1, false, ADR_VHFCNT},    /* VHFCNT,+1                 */
    {1, false, ADR_TTPI},      /* TTPI,+1                   */
    {1, false, ADR_ECSTEER},   /* ECSTEER,+1                */
    {1, false, ADR_DELVTPF},   /* DELVTPF,+1                */
    {2, false, ADR_TCDH},      /* CDH AND CSI TIME (32-33)  */
    {1, false, ADR_TPASS4},    /* TPASS4,+1                 */
    {3, false, ADR_DELVSLV},   /* DELVSLV,+1...+4,+5        */
    {2, false, ADR_RANGE},     /* RANGE,+1,RRATE,+1         */
    {PTR, false, CMPOWE04},    /* CMREND04: COMMON DATA */
    {1, false, ADR_TIME2},     /* TIME2,TIME1               */
    {PTR, false, CMPOWE05},    /* CMREND05: COLLECT SNAPSHOT */
    {6, false, ADR_DNTMBUFF},  /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02},    /* CMREND02: COLLECT SNAPSHOT */
    {4, false, ADR_DNTMBUFF},  /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03},    /* CMREND03: COMMON DATA */
    {PTR, false, CMPOWE06},    /* CMREND06: COMMON DATA */
    {1, false, ADR_DIFFALT},   /* CDH DELTA ALTITUDE        */
    {1, false, ADR_CENTANG},   /* CENTANG,+1                */
    {1, false, ADR_DELTAR},    /* DELTAR,+1                 */
    {3, false, ADR_DELVEET3},  /* DELVEET3,+1,...+4,+5      */
    {1, false, ADR_OPTMODES},  /* OPTMODES,HOLDFLAG         */
    {PTR, false, CMPOWE07},    /* CMREND07: COMMON DATA */
    {1, false, ADR_RTHETA},    /* RTHETA,+1                 */
    {2, false, ADR_LAT_SPL},   /* LAT(SPL),LNG(SPL),+1      */
    {2, false, ADR_VPRED},     /* VPRED,+1,GAMMAEI,+1       */
    {1, true, ADR_STATE + 10u} /* FLAGWRDS 10 AND 11        */
};

/* ---------------- CSM ENTRY AND UPDATE DOWNLIST (CMENTRDL), p.177 ------ */
static const agc_dn_entry_t cmentrdl[] = {
    {PTR, false, CMPOWE01},       /* CMENTR01                  */
    {6, false, ADR_DNTMBUFF},     /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02},       /* CMENTR02                  */
    {4, false, ADR_DNTMBUFF},     /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03},       /* CMENTR03                  */
    {2, false, ADR_CMDAPMOD},     /* CMDAPMOD,PREL,QREL,RREL   */
    {1, false, ADR_L_D1},         /* L/D1,+1                   */
    {6, false, ADR_UPBUFF},       /* UPBUFF,+1,...+10,+11      */
    {4, false, ADR_UPBUFF + 12u}, /* UPBUFF+12,13,...+18,+19D  */
    {2, false, ADR_COMPNUMB},     /* COMPNUMB,UPOLDMOD,UPVERB,UPCOUNT */
    {1, false, ADR_PAXERR1},      /* PAXERR1,ROLLTM            */
    {3, false, ADR_LATANG},       /* LATANG,+1,RDOT,+1,THETAH,+1 */
    {2, false, ADR_LAT_SPL},      /* LAT(SPL),+1,LNG(SPL),+1   */
    {1, false, ADR_ALFA_180},     /* ALFA/180,BETA/180         */
    {PTR, false, CMPOWE04},       /* CMENTR04                  */
    {1, false, ADR_TIME2},        /* TIME2,TIME1               */
    {PTR, false, CMENTR05},       /* COLLECT SNAPSHOT          */
    {6, false, ADR_DNTMBUFF},     /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02},       /* CMENTR02: COLLECT SNAPSHOT */
    {4, false, ADR_DNTMBUFF},     /* SEND SNAPSHOT */
    {2, false, ADR_AK},           /* AK,AK1,AK2,RCSFLAGS       */
    {3, false, ADR_ERRORX},       /* ERRORX/Y/Z,THETADX/Y/Z    */
    {2, false, ADR_CMDAPMOD},     /* CMDAPMOD,PREL,QREL,RREL   */
    {6, false, ADR_UPBUFF},       /* UPBUFF+0,...+11D          */
    {4, false, ADR_UPBUFF + 12u}, /* UPBUFF+12,...+19D         */
    {1, false, ADR_LEMMASS},      /* LEMMASS,CSMMASS           */
    {1, false, ADR_DAPDATR1},     /* DAPDATR1,DAPDATR2         */
    {1, false, ADR_ROLLTM},       /* ROLLTM,ROLLC              */
    {1, false, ADR_OPTMODES},     /* OPTMODES,HOLDFLAG         */
    {3, false, ADR_WBODY},        /* WBODY,...+5/OMEGAC,...+5  */
    {2, false, ADR_REDOCTR},      /* REDOCTR,THETAD+0,+1,+2    */
    {1, false, ADR_IMODES30},     /* IMODES30,IMODES33         */
    {CHN, false, 011},            /* CHANNELS 11,12            */
    {CHN, false, 013},            /* CHANNELS 13,14            */
    {CHN, false, 030},            /* CHANNELS 30,31            */
    {CHN, false, 032},            /* CHANNELS 32,33            */
    {1, false, ADR_RSBBQ},        /* RSBBQ,+1                  */
    {3, false, ADR_CADRFLSH},     /* CADRFLSH,+1,+2,FAILREG,.. */
    {1, false, ADR_STATE + 10u},  /* FLAGWRDS 10 AND 11        */
    {1, true, ADR_GAMMAEI}        /* GAMMAEI,+1                */
};

static const agc_dn_entry_t cmentr05[] = {
    /* SNAPSHOT DATA, p.178       */
    {1, true, ADR_DELV},       /* DELV,+1                   */
    {1, false, ADR_DELV + 2u}, /* DELV+2,+3                 */
    {1, false, ADR_DELV + 4u}, /* DELV+4,+5                 */
    {1, false, ADR_TTE},       /* TTE,+1                    */
    {1, false, ADR_VIO},       /* VIO,+1                    */
    {1, false, ADR_VPRED},     /* VPRED,+1                  */
    {1, true, ADR_PIPTIME1}    /* PIPTIME1,+1               */
};

/* ---------------- P22 DOWNLIST (CMPG22DL), p.179 ---------------------- */
static const agc_dn_entry_t cmpg22dl[] = {
    {PTR, false, CMPOWE01},             /* CMPG2201                  */
    {6, false, ADR_DNTMBUFF},           /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02},             /* CMPG2202                  */
    {4, false, ADR_DNTMBUFF},           /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03},             /* CMPG2203                  */
    {6, false, ADR_SVMRKDAT},           /* LANDING SITE MARK DATA    */
    {6, false, ADR_SVMRKDAT + 12u},     /* SVMRKDAT+0...+34          */
    {6, false, ADR_SVMRKDAT + 24u},     /* LANDING SITE MARK DATA    */
    {1, false, ADR_LANDMARK},           /* LANDMARK,GARBAGE          */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_SPARE},              /* SPARE */
    {PTR, false, CMPOWE04},             /* CMPG2204                  */
    {1, false, ADR_TIME2},              /* TIME2,TIME1               */
    {PTR, false, CM_LANDMARK_SNAPSHOT}, /* COLLECT SNAPSHOT          */
    {2, false, ADR_DNTMBUFF},           /* SEND SNAPSHOT             */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_SPARE},              /* SPARE */
    {PTR, false, CMPOWE02},             /* CMPG2202: COLLECT SNAPSHOT */
    {4, false, ADR_DNTMBUFF},           /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03},             /* CMPG2203: COMMON DATA */
    {PTR, false, CMPOWE06},             /* CMPG2206                  */
    {1, false, ADR_8NN},                /* 8NN,GARBAGE               */
    {1, false, ADR_STATE + 10u},        /* FLAGWRDS 10 AND 11        */
    {3, false, ADR_RLS},                /* RLS,+1,...+4,+5           */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_OPTMODES},           /* OPTMODES,HOLDFLAG         */
    {PTR, false, CMPOWE07},             /* CMPG2207                  */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, false, ADR_SPARE},              /* SPARE */
    {1, true, ADR_SPARE}                /* SPARE */
};

static const agc_dn_entry_t cm_landmark_snapshot[] = {
    /* SNAPSHOT DATA, p.180       */
    {1, true, ADR_LONG}, /* LONG,+1                   */
    {1, false, ADR_ALT}, /* ALT,+1                    */
    {1, true, ADR_LAT}   /* LAT,+1                    */
};

#undef PTR
#undef CHN

/* DNTABLE (p.180): index = DNLSTCOD.                                      */
static const agc_dn_list_t dntable[AGC_DNLIST_CSM_COUNT] = {
    {cmcstadl, (uint8_t) (sizeof (cmcstadl) / sizeof (cmcstadl[0])), false}, /* 0 NOMDNLST */
    {cmentrdl, (uint8_t) (sizeof (cmentrdl) / sizeof (cmentrdl[0])), false}, /* 1 UPDNLIST */
    {cmrenddl, (uint8_t) (sizeof (cmrenddl) / sizeof (cmrenddl[0])), false}, /* 2          */
    {cmpowedl, (uint8_t) (sizeof (cmpowedl) / sizeof (cmpowedl[0])), false}, /* 3          */
    {cmpg22dl, (uint8_t) (sizeof (cmpg22dl) / sizeof (cmpg22dl[0])), false}  /* 4          */
};

static const agc_dn_list_t sublists[CM_SUB_COUNT] = {
    {cmpowe01, (uint8_t) (sizeof (cmpowe01) / sizeof (cmpowe01[0])), true},
    {cmpowe02, (uint8_t) (sizeof (cmpowe02) / sizeof (cmpowe02[0])), true},
    {cmpowe03, (uint8_t) (sizeof (cmpowe03) / sizeof (cmpowe03[0])), false},
    {cmpowe04, (uint8_t) (sizeof (cmpowe04) / sizeof (cmpowe04[0])), false},
    {cmpowe05, (uint8_t) (sizeof (cmpowe05) / sizeof (cmpowe05[0])), true},
    {cmpowe06, (uint8_t) (sizeof (cmpowe06) / sizeof (cmpowe06[0])), false},
    {cmpowe07, (uint8_t) (sizeof (cmpowe07) / sizeof (cmpowe07[0])), false},
    {cmentr05, (uint8_t) (sizeof (cmentr05) / sizeof (cmentr05[0])), true},
    {cm_landmark_snapshot, (uint8_t) (sizeof (cm_landmark_snapshot) / sizeof (cm_landmark_snapshot[0])), true}
};

/**********************************************************************
 * DNTABLE (Comanche055 DOWNLINK_LISTS.agc pp.170-180) -- INDEX DNLSTCOD,
 * CA DNTABLE: the control list of a downlist
 *
 * ARGUMENTS:
 *   code  int              I   DNLSTCOD, 0..AGC_DNLIST_CSM_COUNT-1
 *   out   agc_dn_list_t *  O   the list
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for a code outside DNTABLE (DNLSTCOD
 *          comes from erasable: V21, AGSWORD)
 **********************************************************************/
void agc_dnlist_csm_control(int code, agc_dn_list_t *out)
{
    *out = dntable[code];
}

/**********************************************************************
 * DNPTR (Comanche055 DOWNLINK_LISTS.agc pp.170-180) -- the sublist a
 * DNPTR entry names
 *
 * ARGUMENTS:
 *   sub  int              I   sublist id (enum above)
 *   out  agc_dn_list_t *  O   the list
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for an id outside the sublist table
 **********************************************************************/
agc_status_t agc_dnlist_csm_sublist(int sub, agc_dn_list_t *out)
{
    if (!AGC_ASSERT(sub >= 0 && sub < CM_SUB_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *out = sublists[sub];

    return (AGC_STATUS_OK);
}
