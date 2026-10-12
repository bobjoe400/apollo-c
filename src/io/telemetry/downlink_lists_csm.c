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
 *   Each AGC list word becomes one agc_dn_entry_t:
 *     { op, neg, ref, offset }
 *     op     1..6 = 1DNADR..6DNADR (n DP words), AGC_DN_OP_CHAN (DNCHAN),
 *            AGC_DN_OP_PTR (DNPTR, ref = sublist id below)
 *     neg    the AGC wrote the entry negated (-1DNADR, -DNCHAN ...): end of
 *            list, or (first entry of a sublist) a snapshot sublist
 *     ref    agc_dn_item_t / channel number / sublist id
 *     offset SP-word offset (RN +2 -> 2; STATE +10D -> 10)
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

#include "agc/core/assert.h"
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

#define PTR AGC_DN_OP_PTR  /* DNPTR: a sublist        */
#define CHN AGC_DN_OP_CHAN /* DNCHAN: channels n, n+1 */

/* ---------------- CSM POWERED FLIGHT DOWNLIST (CMPOWEDL), p.171 -------- */
static const agc_dn_entry_t cmpowedl[] = {
    {PTR, false, CMPOWE01, 0},      /* COLLECT SNAPSHOT          */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, CMPOWE02, 0},      /* COLLECT SECOND SNAPSHOT   */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, CMPOWE03, 0},      /* COMMON DATA               */
    {1, false, AGC_DN_TIG, 0},      /* TIG,+1                    */
    {1, false, AGC_DN_DELLT4, 0},   /* DELLT4,+1                 */
    {3, false, AGC_DN_RTARG, 0},    /* RTARG,+1,+2,...+5         */
    {1, false, AGC_DN_TGO, 0},      /* TGO,+1                    */
    {1, false, AGC_DN_PIPTIME1, 0}, /* PIPTIME1,+1               */
    {3, false, AGC_DN_DELV, 0},     /* DELV,+1,...+4,+5          */
    {1, false, AGC_DN_PACTOFF, 0},  /* PACTOFF,YACTOFF           */
    {1, false, AGC_DN_PCMD, 0},     /* PCMD,YCMD                 */
    {1, false, AGC_DN_CSTEER, 0},   /* CSTEER,+1                 */
    {3, false, AGC_DN_DELVEET1, 0}, /* CSI DELTA V (31-33)       */
    {6, false, AGC_DN_REFSMMAT, 0}, /* REFSMMAT,+1,...+10,+11    */
    {PTR, false, CMPOWE04, 0},      /* COMMON DATA               */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2,TIME1               */
    {PTR, false, CMPOWE05, 0},      /* COLLECT SNAPSHOT          */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, CMPOWE02, 0},      /* COLLECT SNAPSHOT          */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, CMPOWE03, 0},      /* CMPOWE03: COMMON DATA */
    {PTR, false, CMPOWE06, 0},      /* COMMON DATA               */
    {1, false, AGC_DN_ELEV, 0},     /* ELEV,+1                   */
    {1, false, AGC_DN_CENTANG, 0},  /* CENTANG,+1                */
    {1, false, AGC_DN_DELTAR, 0},   /* DELTAR,+1                 */
    {1, false, AGC_DN_STATE, 10},   /* FLAGWRDS 10 AND 11        */
    {1, false, AGC_DN_TEVENT, 0},   /* TEVENT,+1                 */
    {1, false, AGC_DN_PCMD, 0},     /* PCMD,YCMD                 */
    {1, false, AGC_DN_OPTMODES, 0}, /* OPTMODES,HOLDFLAG         */
    {PTR, false, CMPOWE07, 0},      /* COMMON DATA               */
    {3, false, AGC_DN_VGTIG, 0},    /* VGTIG,+1,...+4,+5         */
    {3, true, AGC_DN_DELVEET2, 0}   /* CDH DELTA V (98-100)      */
};

/* ---------------- sublists, pp.171-172 -------------------------------- */
static const agc_dn_entry_t cmpowe01[] = {
    /* SNAPSHOT DATA              */
    {1, true, AGC_DN_RN, 2},       /* RN+2,+3                   */
    {1, false, AGC_DN_RN, 4},      /* RN+4,+5                   */
    {1, false, AGC_DN_VN, 0},      /* VN,+1                     */
    {1, false, AGC_DN_VN, 2},      /* VN+2,+3                   */
    {1, false, AGC_DN_VN, 4},      /* VN+4,+5                   */
    {1, false, AGC_DN_PIPTIME, 0}, /* PIPTIME,+1                */
    {1, true, AGC_DN_RN, 0}        /* RN,+1                     */
};

static const agc_dn_entry_t cmpowe02[] = {
    /* SNAPSHOT DATA              */
    {1, true, AGC_DN_CDUZ, 0},  /* CDUZ,CDUT                 */
    {1, false, AGC_DN_ADOT, 0}, /* ADOT,+1/OGARATE,+1        */
    {1, false, AGC_DN_ADOT, 2}, /* ADOT+2,+3/OMEGAB+2,+3     */
    {1, false, AGC_DN_ADOT, 4}, /* ADOT+4,+5/OMEGAB+4,+5     */
    {1, true, AGC_DN_CDUX, 0}   /* CDUX,CDUY                 */
};

static const agc_dn_entry_t cmpowe03[] = {
    /* COMMON DATA                */
    {2, false, AGC_DN_AK, 0},    /* AK,AK1,AK2,RCSFLAGS       */
    {2, true, AGC_DN_THETADX, 0} /* THETADX,Y,Z,GARBAGE       */
};

static const agc_dn_entry_t cmpowe04[] = {
    /* COMMON DATA                */
    {5, false, AGC_DN_STATE, 0}, /* FLAGWRD0 THRU FLAGWRD9    */
    {6, true, AGC_DN_DSPTAB, 0}  /* DISPLAY TABLES            */
};

static const agc_dn_entry_t cmpowe05[] = {
    /* SNAPSHOT DATA              */
    {1, true, AGC_DN_R_OTHER, 2},  /* R-OTHER+2,+3              */
    {1, false, AGC_DN_R_OTHER, 4}, /* R-OTHER+4,+5              */
    {1, false, AGC_DN_V_OTHER, 0}, /* V-OTHER,+1                */
    {1, false, AGC_DN_V_OTHER, 2}, /* V-OTHER+2,+3              */
    {1, false, AGC_DN_V_OTHER, 4}, /* V-OTHER+4,+5              */
    {1, false, AGC_DN_T_OTHER, 0}, /* T-OTHER,+1                */
    {1, true, AGC_DN_R_OTHER, 0}   /* R-OTHER,+1                */
};

static const agc_dn_entry_t cmpowe06[] = {
    /* COMMON DATA                */
    {1, false, AGC_DN_RSBBQ, 0},    /* RSBBQ,+1                  */
    {3, false, AGC_DN_CADRFLSH, 0}, /* CADRFLSH,+1,+2,FAILREG..  */
    {2, true, AGC_DN_CDUS, 0}       /* CDUS,PIPAX,PIPAY,PIPAZ    */
};

static const agc_dn_entry_t cmpowe07[] = {
    /* COMMON DATA                */
    {1, false, AGC_DN_LEMMASS, 0},  /* LEMMASS,CSMMASS           */
    {1, false, AGC_DN_DAPDATR1, 0}, /* DAPDATR1,DAPDATR2         */
    {2, false, AGC_DN_ERRORX, 0},   /* ERRORX,Y,Z,GARBAGE        */
    {3, false, AGC_DN_WBODY, 0},    /* WBODY,...+5/OMEGAC,...+5  */
    {2, false, AGC_DN_REDOCTR, 0},  /* REDOCTR,THETAD,+1,+2      */
    {1, false, AGC_DN_IMODES30, 0}, /* IMODES30,IMODES33         */
    {CHN, false, 011, 0},           /* CHANNELS 11,12            */
    {CHN, false, 013, 0},           /* CHANNELS 13,14            */
    {CHN, false, 030, 0},           /* CHANNELS 30,31            */
    {CHN, true, 032, 0}             /* CHANNELS 32,33            */
};

/* ---------------- CSM COAST AND ALIGNMENT DOWNLIST (CMCSTADL), p.173 --- */
static const agc_dn_entry_t cmcstadl[] = {
    {PTR, false, CMPOWE01, 0},      /* CMCSTA01: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02, 0},      /* CMCSTA02                  */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03, 0},      /* CMCSTA03                  */
    {1, false, AGC_DN_TIG, 0},      /* TIG,+1                    */
    {1, false, AGC_DN_BESTI, 0},    /* BESTI,BESTJ               */
    {4, false, AGC_DN_MARKDOWN, 0}, /* MARKDOWN,+1,...+6,GARBAGE */
    {4, false, AGC_DN_MARK2DWN, 0}, /* MARK2DWN,+1,...+6         */
    {2, false, AGC_DN_HAPOX, 0},    /* APOGEE, PERIGEE (28-29)   */
    {1, false, AGC_DN_PACTOFF, 0},  /* PACTOFF, YACTOFF (30)     */
    {3, false, AGC_DN_VGTIG, 0},    /* VGTIG,...+5               */
    {6, false, AGC_DN_REFSMMAT, 0}, /* REFSMMAT,+1,...+10,+11    */
    {PTR, false, CMPOWE04, 0},      /* CMCSTA04                  */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2,TIME1               */
    {PTR, false, CMPOWE05, 0},      /* CMCSTA05                  */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02, 0},      /* CMCSTA02: COLLECT SNAPSHOT */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03, 0},      /* CMCSTA03: COMMON DATA */
    {PTR, false, CMPOWE06, 0},      /* CMCSTA06                  */
    {3, false, AGC_DN_OGC, 0},      /* OGC,+1,IGC,+1,MGC,+1      */
    {1, false, AGC_DN_STATE, 10},   /* FLAGWRDS 10 AND 11        */
    {1, false, AGC_DN_TEVENT, 0},   /* TEVENT,+1                 */
    {1, false, AGC_DN_LAUNCHAZ, 0}, /* LAUNCHAZ,+1               */
    {1, false, AGC_DN_OPTMODES, 0}, /* OPTMODES,HOLDFLAG         */
    {PTR, false, CMPOWE07, 0},      /* CMCSTA07                  */
    {6, true, AGC_DN_DSPTAB, 0}     /* DISPLAY TABLES            */
};

/* ---------------- CSM RENDEZVOUS AND PRETHRUST LIST (CMRENDDL), p.175 -- */
static const agc_dn_entry_t cmrenddl[] = {
    {PTR, false, CMPOWE01, 0},      /* CMREND01: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02, 0},      /* CMREND02: COLLECT SNAPSHOT */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03, 0},      /* CMREND03: COMMON DATA */
    {1, false, AGC_DN_TIG, 0},      /* TIG,+1                    */
    {1, false, AGC_DN_DELLT4, 0},   /* DELLT4,+1                 */
    {3, false, AGC_DN_RTARG, 0},    /* RTARG,+1,...+4,+5         */
    {1, false, AGC_DN_VHFTIME, 0},  /* VHFTIME,+1                */
    {4, false, AGC_DN_MARKDOWN, 0}, /* MARKTIME,YCDU,SCDU,...,RM */
    {1, false, AGC_DN_VHFCNT, 0},   /* VHFCNT,+1                 */
    {1, false, AGC_DN_TTPI, 0},     /* TTPI,+1                   */
    {1, false, AGC_DN_ECSTEER, 0},  /* ECSTEER,+1                */
    {1, false, AGC_DN_DELVTPF, 0},  /* DELVTPF,+1                */
    {2, false, AGC_DN_TCDH, 0},     /* CDH AND CSI TIME (32-33)  */
    {1, false, AGC_DN_TPASS4, 0},   /* TPASS4,+1                 */
    {3, false, AGC_DN_DELVSLV, 0},  /* DELVSLV,+1...+4,+5        */
    {2, false, AGC_DN_RANGE, 0},    /* RANGE,+1,RRATE,+1         */
    {PTR, false, CMPOWE04, 0},      /* CMREND04: COMMON DATA */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2,TIME1               */
    {PTR, false, CMPOWE05, 0},      /* CMREND05: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02, 0},      /* CMREND02: COLLECT SNAPSHOT */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03, 0},      /* CMREND03: COMMON DATA */
    {PTR, false, CMPOWE06, 0},      /* CMREND06: COMMON DATA */
    {1, false, AGC_DN_DIFFALT, 0},  /* CDH DELTA ALTITUDE        */
    {1, false, AGC_DN_CENTANG, 0},  /* CENTANG,+1                */
    {1, false, AGC_DN_DELTAR, 0},   /* DELTAR,+1                 */
    {3, false, AGC_DN_DELVEET3, 0}, /* DELVEET3,+1,...+4,+5      */
    {1, false, AGC_DN_OPTMODES, 0}, /* OPTMODES,HOLDFLAG         */
    {PTR, false, CMPOWE07, 0},      /* CMREND07: COMMON DATA */
    {1, false, AGC_DN_RTHETA, 0},   /* RTHETA,+1                 */
    {2, false, AGC_DN_LAT_SPL, 0},  /* LAT(SPL),LNG(SPL),+1      */
    {2, false, AGC_DN_VPRED, 0},    /* VPRED,+1,GAMMAEI,+1       */
    {1, true, AGC_DN_STATE, 10}     /* FLAGWRDS 10 AND 11        */
};

/* ---------------- CSM ENTRY AND UPDATE DOWNLIST (CMENTRDL), p.177 ------ */
static const agc_dn_entry_t cmentrdl[] = {
    {PTR, false, CMPOWE01, 0},      /* CMENTR01                  */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02, 0},      /* CMENTR02                  */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03, 0},      /* CMENTR03                  */
    {2, false, AGC_DN_CMDAPMOD, 0}, /* CMDAPMOD,PREL,QREL,RREL   */
    {1, false, AGC_DN_L_D1, 0},     /* L/D1,+1                   */
    {6, false, AGC_DN_UPBUFF, 0},   /* UPBUFF,+1,...+10,+11      */
    {4, false, AGC_DN_UPBUFF, 12},  /* UPBUFF+12,13,...+18,+19D  */
    {2, false, AGC_DN_COMPNUMB, 0}, /* COMPNUMB,UPOLDMOD,UPVERB,UPCOUNT */
    {1, false, AGC_DN_PAXERR1, 0},  /* PAXERR1,ROLLTM            */
    {3, false, AGC_DN_LATANG, 0},   /* LATANG,+1,RDOT,+1,THETAH,+1 */
    {2, false, AGC_DN_LAT_SPL, 0},  /* LAT(SPL),+1,LNG(SPL),+1   */
    {1, false, AGC_DN_ALFA_180, 0}, /* ALFA/180,BETA/180         */
    {PTR, false, CMPOWE04, 0},      /* CMENTR04                  */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2,TIME1               */
    {PTR, false, CMENTR05, 0},      /* COLLECT SNAPSHOT          */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02, 0},      /* CMENTR02: COLLECT SNAPSHOT */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT */
    {2, false, AGC_DN_AK, 0},       /* AK,AK1,AK2,RCSFLAGS       */
    {3, false, AGC_DN_ERRORX, 0},   /* ERRORX/Y/Z,THETADX/Y/Z    */
    {2, false, AGC_DN_CMDAPMOD, 0}, /* CMDAPMOD,PREL,QREL,RREL   */
    {6, false, AGC_DN_UPBUFF, 0},   /* UPBUFF+0,...+11D          */
    {4, false, AGC_DN_UPBUFF, 12},  /* UPBUFF+12,...+19D         */
    {1, false, AGC_DN_LEMMASS, 0},  /* LEMMASS,CSMMASS           */
    {1, false, AGC_DN_DAPDATR1, 0}, /* DAPDATR1,DAPDATR2         */
    {1, false, AGC_DN_ROLLTM, 0},   /* ROLLTM,ROLLC              */
    {1, false, AGC_DN_OPTMODES, 0}, /* OPTMODES,HOLDFLAG         */
    {3, false, AGC_DN_WBODY, 0},    /* WBODY,...+5/OMEGAC,...+5  */
    {2, false, AGC_DN_REDOCTR, 0},  /* REDOCTR,THETAD+0,+1,+2    */
    {1, false, AGC_DN_IMODES30, 0}, /* IMODES30,IMODES33         */
    {CHN, false, 011, 0},           /* CHANNELS 11,12            */
    {CHN, false, 013, 0},           /* CHANNELS 13,14            */
    {CHN, false, 030, 0},           /* CHANNELS 30,31            */
    {CHN, false, 032, 0},           /* CHANNELS 32,33            */
    {1, false, AGC_DN_RSBBQ, 0},    /* RSBBQ,+1                  */
    {3, false, AGC_DN_CADRFLSH, 0}, /* CADRFLSH,+1,+2,FAILREG,.. */
    {1, false, AGC_DN_STATE, 10},   /* FLAGWRDS 10 AND 11        */
    {1, true, AGC_DN_GAMMAEI, 0}    /* GAMMAEI,+1                */
};

static const agc_dn_entry_t cmentr05[] = {
    /* SNAPSHOT DATA, p.178       */
    {1, true, AGC_DN_DELV, 0},    /* DELV,+1                   */
    {1, false, AGC_DN_DELV, 2},   /* DELV+2,+3                 */
    {1, false, AGC_DN_DELV, 4},   /* DELV+4,+5                 */
    {1, false, AGC_DN_TTE, 0},    /* TTE,+1                    */
    {1, false, AGC_DN_VIO, 0},    /* VIO,+1                    */
    {1, false, AGC_DN_VPRED, 0},  /* VPRED,+1                  */
    {1, true, AGC_DN_PIPTIME1, 0} /* PIPTIME1,+1               */
};

/* ---------------- P22 DOWNLIST (CMPG22DL), p.179 ---------------------- */
static const agc_dn_entry_t cmpg22dl[] = {
    {PTR, false, CMPOWE01, 0},             /* CMPG2201                  */
    {6, false, AGC_DN_DNTMBUFF, 0},        /* SEND SNAPSHOT */
    {PTR, false, CMPOWE02, 0},             /* CMPG2202                  */
    {4, false, AGC_DN_DNTMBUFF, 0},        /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03, 0},             /* CMPG2203                  */
    {6, false, AGC_DN_SVMRKDAT, 0},        /* LANDING SITE MARK DATA    */
    {6, false, AGC_DN_SVMRKDAT, 12},       /* SVMRKDAT+0...+34          */
    {6, false, AGC_DN_SVMRKDAT, 24},       /* LANDING SITE MARK DATA    */
    {1, false, AGC_DN_LANDMARK, 0},        /* LANDMARK,GARBAGE          */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {PTR, false, CMPOWE04, 0},             /* CMPG2204                  */
    {1, false, AGC_DN_TIME2, 0},           /* TIME2,TIME1               */
    {PTR, false, CM_LANDMARK_SNAPSHOT, 0}, /* COLLECT SNAPSHOT          */
    {2, false, AGC_DN_DNTMBUFF, 0},        /* SEND SNAPSHOT             */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {PTR, false, CMPOWE02, 0},             /* CMPG2202: COLLECT SNAPSHOT */
    {4, false, AGC_DN_DNTMBUFF, 0},        /* SEND SNAPSHOT */
    {PTR, false, CMPOWE03, 0},             /* CMPG2203: COMMON DATA */
    {PTR, false, CMPOWE06, 0},             /* CMPG2206                  */
    {1, false, AGC_DN_8NN, 0},             /* 8NN,GARBAGE               */
    {1, false, AGC_DN_STATE, 10},          /* FLAGWRDS 10 AND 11        */
    {3, false, AGC_DN_RLS, 0},             /* RLS,+1,...+4,+5           */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_OPTMODES, 0},        /* OPTMODES,HOLDFLAG         */
    {PTR, false, CMPOWE07, 0},             /* CMPG2207                  */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, false, AGC_DN_SPARE, 0},           /* SPARE */
    {1, true, AGC_DN_SPARE, 0}             /* SPARE */
};

static const agc_dn_entry_t cm_landmark_snapshot[] = {
    /* SNAPSHOT DATA, p.180       */
    {1, true, AGC_DN_LONG, 0}, /* LONG,+1                   */
    {1, false, AGC_DN_ALT, 0}, /* ALT,+1                    */
    {1, true, AGC_DN_LAT, 0}   /* LAT,+1                    */
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
agc_status_t agc_dnlist_csm_control(int code, agc_dn_list_t *out)
{
    if (!AGC_ASSERT(code >= 0 && code < AGC_DNLIST_CSM_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *out = dntable[code];

    return (AGC_STATUS_OK);
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
