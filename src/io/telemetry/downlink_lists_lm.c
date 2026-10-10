/*
 * downlink_lists_lm.c -- Luminary099 DOWNLINK_LISTS.agc (pp. 193-205).
 *
 * Same entry encoding as src/io/downlink_lists_csm.c:
 *   { op, neg, ref, offset }
 *   op     1..6 = 1DNADR..6DNADR (n DP words), AGC_DN_OP_CHAN (DNCHAN),
 *          AGC_DN_OP_PTR (DNPTR, ref = sublist id below)
 *   neg    the AGC wrote the entry negated: end of list, or (first entry of
 *          a sublist) a snapshot sublist
 *   ref    agc_dn_item_t / channel number / sublist id
 *   offset SP-word offset from the item (RN +2 -> 2, UPBUFF +8D -> 8)
 *
 * The AGC's EQUALS aliases (LMCSTA01 EQUALS LMORBM01 etc.) become
 * references to the same sublist id.  LMDSAS08 is a plain (non-snapshot)
 * sublist that sends the 11-word snapshot buffer of LMDSAS07.
 * UNKNOWN and SPARE both equal ERASZERO (register 7, always zero).
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/io/service_internal.h"

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

#define PTR AGC_DN_OP_PTR
#define CHN AGC_DN_OP_CHAN

/* ---------------- LM ORBITAL MANEUVERS LIST (LMORBMDL), p.194 ---------- */
static const agc_dn_entry_t lmorbmdl[] = {
    {PTR, false, LMORBM01, 0},      /* COLLECT SNAPSHOT          */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {1, false, AGC_DN_DELLT4, 0},   /* DELLT4,+1                 */
    {3, false, AGC_DN_RTARG, 0},    /* RTARG,+1...+5             */
    {1, false, AGC_DN_ELEV, 0},     /* ELEV,+1                   */
    {1, false, AGC_DN_TEVENT, 0},   /* TEVENT,+1                 */
    {6, false, AGC_DN_REFSMMAT, 0}, /* REFSMMAT +0...+11D        */
    {1, false, AGC_DN_TCSI, 0},     /* TCSI,+1                   */
    {3, false, AGC_DN_DELVEET1, 0}, /* DELVEET1 +0...+5          */
    {3, false, AGC_DN_VGTIG, 0},    /* VGTIG +0...+5             */
    {1, false, AGC_DN_DNLRVELZ, 0}, /* DNLRVELZ,DNLRALT          */
    {1, false, AGC_DN_TPASS4, 0},   /* TPASS4,+1                 */
    {PTR, false, LMORBM02, 0},      /* COMMON DATA               */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2/1                   */
    {PTR, false, LMORBM03, 0},      /* COLLECT SNAPSHOT          */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04, 0},      /* COMMON DATA               */
    {2, false, AGC_DN_POSTORKU, 0}, /* POSTORKU,NEGTORKU,..V     */
    {1, false, AGC_DN_SPARE, 0},
    {1, false, AGC_DN_TCDH, 0},     /* TCDH,+1                   */
    {3, false, AGC_DN_DELVEET2, 0}, /* DELVEET2 +0...+5          */
    {1, false, AGC_DN_TTPI, 0},     /* TTPI,+1                   */
    {3, false, AGC_DN_DELVEET3, 0}, /* DELVEET3 +0...+5          */
    {1, false, AGC_DN_DNRRANGE, 0}, /* DNRRANGE,DNRRDOT          */
    {2, false, AGC_DN_DNLRVELX, 0}, /* DNLRVELX,Y,Z,DNLRALT      */
    {1, false, AGC_DN_DIFFALT, 0},  /* DIFFALT,+1                */
    {1, false, AGC_DN_LEMMASS, 0},  /* LEMMASS,CSMMASS           */
    {1, false, AGC_DN_IMODES30, 0}, /* IMODES30,IMODES33         */
    {1, false, AGC_DN_TIG, 0},      /* TIG,+1                    */
    {PTR, false, LMORBM05, 0},      /* COMMON DATA               */
    {PTR, false, LMORBM06, 0},      /* COMMON DATA               */
    {1, false, AGC_DN_SPARE, 0},    /* FORMERLY PIF              */
    {1, true, AGC_DN_TGO, 0}        /* TGO,+1                    */
};

/* ---------------- sublists, pp.194-195 -------------------------------- */
static const agc_dn_entry_t lmorbm01[] = {
    /* SNAPSHOT                   */
    {1, true, AGC_DN_R_OTHER, 2},  /* R-OTHER +2,+3             */
    {1, false, AGC_DN_R_OTHER, 4}, /* R-OTHER +4,+5             */
    {1, false, AGC_DN_V_OTHER, 0}, /* V-OTHER,+1                */
    {1, false, AGC_DN_V_OTHER, 2}, /* V-OTHER +2,+3             */
    {1, false, AGC_DN_V_OTHER, 4}, /* V-OTHER +4,+5             */
    {1, false, AGC_DN_T_OTHER, 0}, /* T-OTHER,+1                */
    {1, true, AGC_DN_R_OTHER, 0}   /* R-OTHER +0,+1             */
};
static const agc_dn_entry_t lmorbm02[] = {
    /* COMMON DATA                */
    {2, false, AGC_DN_REDOCTR, 0}, /* REDOCTR,THETAD,+1,+2      */
    {1, false, AGC_DN_RSBBQ, 0},   /* RSBBQ,+1                  */
    {2, false, AGC_DN_OMEGAP, 0},  /* OMEGAP,Q,R,GARBAGE        */
    {2, false, AGC_DN_CDUXD, 0},   /* CDUXD,CDUYD,CDUZD,GARBAGE */
    {2, false, AGC_DN_CDUX, 0},    /* CDUX,CDUY,CDUZ,CDUT       */
    {6, false, AGC_DN_STATE, 0},   /* STATE +0...+11D           */
    {6, true, AGC_DN_DSPTAB, 0}    /* DSPTAB TABLES             */
};
static const agc_dn_entry_t lmorbm03[] = {
    /* SNAPSHOT                   */
    {1, true, AGC_DN_RN, 2},       /* RN +2,+3                  */
    {1, false, AGC_DN_RN, 4},      /* RN +4,+5                  */
    {1, false, AGC_DN_VN, 0},      /* VN,+1                     */
    {1, false, AGC_DN_VN, 2},      /* VN +2,+3                  */
    {1, false, AGC_DN_VN, 4},      /* VN +4,+5                  */
    {1, false, AGC_DN_PIPTIME, 0}, /* PIPTIME,+1                */
    {1, true, AGC_DN_RN, 0}        /* RN,+1                     */
};
static const agc_dn_entry_t lmorbm04[] = {
    /* COMMON DATA                */
    {2, false, AGC_DN_OMEGAPD, 0},  /* OMEGAPD,QD,RD,GARBAGE     */
    {3, false, AGC_DN_CADRFLSH, 0}, /* CADRFLSH,+1,+2,FAILREG..  */
    {1, true, AGC_DN_RADMODES, 0}   /* RADMODES,DAPBOOLS         */
};
static const agc_dn_entry_t lmorbm05[] = {
    /* COMMON DATA                */
    {2, false, AGC_DN_OMEGAP, 0},   /* OMEGAP,Q,R,GARBAGE        */
    {2, false, AGC_DN_CDUXD, 0},    /* CDUXD,CDUYD,CDUZD,GARBAGE */
    {2, false, AGC_DN_CDUX, 0},     /* CDUX,CDUY,CDUZ,CDUT       */
    {1, false, AGC_DN_ALPHAQ, 0},   /* ALPHAQ,ALPHAR             */
    {1, false, AGC_DN_POSTORKP, 0}, /* POSTORKP,NEGTORKP         */
    {CHN, false, 011, 0},           /* CHANNELS 11,12            */
    {CHN, false, 013, 0},           /* CHANNELS 13,14            */
    {CHN, false, 030, 0},           /* CHANNELS 30,31            */
    {CHN, true, 032, 0}             /* CHANNELS 32,33            */
};
static const agc_dn_entry_t lmorbm06[] = {
    /* COMMON DATA                */
    {1, false, AGC_DN_PIPTIME1, 0}, /* PIPTIME,+1                */
    {3, true, AGC_DN_DELV, 0}       /* DELV +0...+5              */
};

/* ---------------- LM COAST AND ALIGNMENT LIST (LMCSTADL), p.196 -------- */
static const agc_dn_entry_t lmcstadl[] = {
    {PTR, false, LMORBM01, 0},      /* LMCSTA01: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {1, false, AGC_DN_AGSK, 0},     /* AGSK,+1                   */
    {1, false, AGC_DN_TALIGN, 0},   /* TALIGN,+1                 */
    {2, false, AGC_DN_POSTORKU, 0}, /* POSTORKU,NEGTORKU,..V     */
    {1, false, AGC_DN_DNRRANGE, 0}, /* DNRRANGE,DNRRDOT          */
    {1, false, AGC_DN_TEVENT, 0},   /* TEVENT,+1                 */
    {6, false, AGC_DN_REFSMMAT, 0}, /* REFSMMAT +0...+11D        */
    {1, false, AGC_DN_AOTCODE, 0},  /* AOTCODE,GARBAGE           */
    {3, false, AGC_DN_RLS, 0},      /* RLS +0...+5               */
    {2, false, AGC_DN_DNLRVELX, 0}, /* DNLRVELX,Y,Z,DNLRALT      */
    {PTR, false, LMCSTA06, 0},      /* COMMON DATA               */
    {PTR, false, LMORBM02, 0},      /* LMCSTA02: COMMON DATA     */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2/1                   */
    {PTR, false, LMORBM03, 0},      /* LMCSTA03: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04, 0},      /* LMCSTA04: COMMON DATA     */
    {PTR, false, LMCSTA07, 0},      /* COMMON DATA               */
    {2, false, AGC_DN_DNLRVELX, 0}, /* DNLRVELX,Y,Z,DNLRALT      */
    {2, false, AGC_DN_CDUS, 0},     /* CDUS,PIPAX,PIPAY,PIPAZ    */
    {1, false, AGC_DN_LASTYCMD, 0}, /* LASTYCMD,LASTXCMD         */
    {1, false, AGC_DN_LEMMASS, 0},  /* LEMMASS,CSMMASS           */
    {1, false, AGC_DN_IMODES30, 0}, /* IMODES30,IMODES33         */
    {1, false, AGC_DN_TIG, 0},      /* TIG,+1                    */
    {PTR, false, LMORBM05, 0},      /* LMCSTA05: COMMON DATA     */
    {6, true, AGC_DN_DSPTAB, 0}     /* DSPTAB +0...+11D TABLE    */
};
static const agc_dn_entry_t lmcsta06[] = {
    /* COMMON DATA, p.196         */
    {2, false, AGC_DN_X789, 0},   /* X789 +0...+3              */
    {1, true, AGC_DN_LASTYCMD, 0} /* LASTYCMD,LASTXCMD         */
};
static const agc_dn_entry_t lmcsta07[] = {
    /* COMMON DATA                */
    {3, false, AGC_DN_OGC, 0},      /* OGC,+1,IGC,+1,MGC,+1      */
    {1, false, AGC_DN_BESTI, 0},    /* BESTI,BESTJ               */
    {3, false, AGC_DN_STARSAV1, 0}, /* STARSAV1 +0...+5          */
    {3, true, AGC_DN_STARSAV2, 0}   /* STARSAV2 +0...+5          */
};

/* ---------------- LM RENDEZVOUS AND PRETHRUST LIST (LMRENDDL), p.198 --- */
static const agc_dn_entry_t lmrenddl[] = {
    {PTR, false, LMORBM01, 0},      /* LMREND01: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, LMREND07, 0},      /* COLLECT SNAPSHOT          */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {1, false, AGC_DN_DELLT4, 0},   /* DELLT4,+1                 */
    {3, false, AGC_DN_RTARG, 0},    /* RTARG +0...+5             */
    {3, false, AGC_DN_DELVSLV, 0},  /* DELVSLV +0...+5           */
    {1, false, AGC_DN_TCSI, 0},     /* TCSI,+1                   */
    {3, false, AGC_DN_DELVEET1, 0}, /* DELVEET +0...+5           */
    {1, false, AGC_DN_SPARE, 0},
    {1, false, AGC_DN_TPASS4, 0},   /* TPASS4,+1                 */
    {PTR, false, LMCSTA06, 0},      /* LMREND06: COMMON DATA     */
    {PTR, false, LMORBM02, 0},      /* LMREND02: COMMON DATA     */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2/1                   */
    {PTR, false, LMORBM03, 0},      /* LMREND03: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04, 0},      /* LMREND04: COMMON DATA     */
    {2, false, AGC_DN_POSTORKU, 0}, /* POSTORKU,NEGTORKU,..V     */
    {1, false, AGC_DN_SPARE, 0},
    {1, false, AGC_DN_TCDH, 0},     /* TCDH,+1                   */
    {3, false, AGC_DN_DELVEET2, 0}, /* DELVEET2 +0...+5          */
    {1, false, AGC_DN_TTPI, 0},     /* TTPI,+1                   */
    {3, false, AGC_DN_DELVEET3, 0}, /* DELVEET3 +0...+5          */
    {1, false, AGC_DN_ELEV, 0},     /* ELEV,+1                   */
    {2, false, AGC_DN_CDUS, 0},     /* CDUS,PIPAX,PIPAY,PIPAZ    */
    {1, false, AGC_DN_LASTYCMD, 0}, /* LASTYCMD,LASTXCMD         */
    {1, false, AGC_DN_LEMMASS, 0},  /* LEMMASS,CSMMASS           */
    {1, false, AGC_DN_IMODES30, 0}, /* IMODES30,IMODES33         */
    {1, false, AGC_DN_TIG, 0},      /* TIG,+1                    */
    {PTR, false, LMORBM05, 0},      /* LMREND05: COMMON DATA     */
    {1, false, AGC_DN_DELTAR, 0},   /* DELTAR,+1                 */
    {1, false, AGC_DN_CENTANG, 0},  /* CENTANG,+1                */
    {1, false, AGC_DN_NN, 0},       /* NN,+1                     */
    {1, false, AGC_DN_DIFFALT, 0},  /* DIFFALT,+1                */
    {1, false, AGC_DN_DELVTPF, 0},  /* DELVTPF,+1                */
    {1, true, AGC_DN_SPARE, 0}};
static const agc_dn_entry_t lmrend07[] = {
    /* SNAPSHOT, p.199            */
    {1, true, AGC_DN_AIG, 0},     /* AIG,AMG                   */
    {1, false, AGC_DN_AOG, 0},    /* AOG,TRKMKCNT              */
    {1, false, AGC_DN_TANGNB, 0}, /* TANGNB,+1                 */
    {1, false, AGC_DN_MKTIME, 0}, /* MKTIME,+1                 */
    {1, true, AGC_DN_RANGRDOT, 0} /* DNRRANGE,DNRRDOT          */
};

/* ---------------- LM DESCENT AND ASCENT LIST (LMDSASDL), p.200 --------- */
static const agc_dn_entry_t lmdsasdl[] = {
    {PTR, false, LMDSAS07, 0},    /* COLLECT SNAPSHOT          */
    {PTR, false, LMDSAS08, 0},    /* SEND SNAPSHOT             */
    {1, false, AGC_DN_TEVENT, 0}, /* TEVENT,+1                 */
    {3, false, AGC_DN_UNFC_2, 0}, /* UNFC/2 +0...+5            */
    {3, false, AGC_DN_VGVECT, 0}, /* VGVECT +0...+5            */
    {1, false, AGC_DN_TTF_8, 0},  /* TTF/8,+1                  */
    {1, false, AGC_DN_DELTAH, 0}, /* DELTAH,+1                 */
    {3, false, AGC_DN_RLS, 0},    /* RLS +0...+5               */
    {1, false, AGC_DN_SPARE, 0},
    {PTR, false, LMCSTA06, 0},      /* LMDSAS09: COMMON DATA     */
    {PTR, false, LMORBM02, 0},      /* LMDSAS02: COMMON DATA     */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2/1                   */
    {PTR, false, LMORBM03, 0},      /* LMDSAS03: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04, 0},      /* LMDSAS04: COMMON DATA     */
    {2, false, AGC_DN_POSTORKU, 0}, /* POSTORKU,NEGTORKU,..V     */
    {3, false, AGC_DN_RGU, 0},      /* RGU +0...+5               */
    {3, false, AGC_DN_VGU, 0},      /* VGU +0...+5               */
    {3, false, AGC_DN_LAND, 0},     /* LAND +0...+5              */
    {1, false, AGC_DN_AT, 0},       /* AT,+1                     */
    {1, false, AGC_DN_TLAND, 0},    /* TLAND,+1                  */
    {1, false, AGC_DN_FC, 0},       /* FC,GARBAGE                */
    {1, false, AGC_DN_LASTYCMD, 0}, /* LASTYCMD,LASTXCMD         */
    {1, false, AGC_DN_LEMMASS, 0},  /* LEMMASS,CSMMASS           */
    {1, false, AGC_DN_IMODES30, 0}, /* IMODES30,IMODES33         */
    {1, false, AGC_DN_TIG, 0},      /* TIG,+1                    */
    {PTR, false, LMORBM05, 0},      /* LMDSAS05: COMMON DATA     */
    {PTR, false, LMORBM06, 0},      /* LMDSAS06: COMMON DATA     */
    {1, false, AGC_DN_PSEUDO55, 0}, /* PSEUDO55,GARBAGE          */
    {1, true, AGC_DN_TTOGO, 0}      /* TTOGO,+1                  */
};
static const agc_dn_entry_t lmdsas07[] = {
    /* SNAPSHOT, p.200            */
    {1, true, AGC_DN_LRZCDUDL, 0},  /* LRZCDUDL,GARBAGE          */
    {1, false, AGC_DN_VSELECT, 0},  /* VSELECT,GARBAGE           */
    {1, false, AGC_DN_LRVTIMDL, 0}, /* LRVTIMDL,+1               */
    {1, false, AGC_DN_VMEAS, 0},    /* VMEAS,+1                  */
    {1, false, AGC_DN_MKTIME, 0},   /* MKTIME,+1                 */
    {1, false, AGC_DN_HMEAS, 0},    /* HMEAS,+1                  */
    {1, false, AGC_DN_RM, 0},       /* RM,+1                     */
    {1, false, AGC_DN_AIG, 0},      /* AIG,AMG                   */
    {1, false, AGC_DN_AOG, 0},      /* AOG,TRKMKCNT              */
    {1, false, AGC_DN_TANGNB, 0},   /* TANGNB,+1                 */
    {1, false, AGC_DN_MKTIME, 0},   /* MKTIME,+1                 */
    {1, true, AGC_DN_LRXCDUDL, 0}   /* LRXCDUDL,LRYCDUDL         */
};
static const agc_dn_entry_t lmdsas08[] = {/* SEND SNAPSHOT              */
                                          {6, false, AGC_DN_DNTMBUFF, 0},
                                          {5, true, AGC_DN_DNTMBUFF, 12}};

/* ---------------- LM LUNAR SURFACE ALIGN LIST (LMLSALDL), p.202 -------- */
static const agc_dn_entry_t lmlsaldl[] = {
    {PTR, false, LMORBM01, 0},      /* LMLSAL01: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, LMREND07, 0},      /* LMLSAL07: COLLECT SNAPSHOT */
    {4, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {1, false, AGC_DN_TALIGN, 0},   /* TALIGN,+1                 */
    {6, false, AGC_DN_REFSMMAT, 0}, /* REFSMMAT +0...+11D        */
    {6, false, AGC_DN_YNBSAV, 0},   /* YNBSAV +0..+5,SNBSAV +0..+5 */
    {PTR, false, LMCSTA06, 0},      /* LMLSAL08: COMMON DATA     */
    {PTR, false, LMORBM02, 0},      /* LMLSAL02: COMMON DATA     */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2/1                   */
    {PTR, false, LMORBM03, 0},      /* LMLSAL03: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04, 0},      /* LMLSAL04: COMMON DATA     */
    {PTR, false, LMCSTA07, 0},      /* LMLSAL09: COMMON DATA     */
    {3, false, AGC_DN_GSAV, 0},     /* GSAV +0...+5              */
    {1, false, AGC_DN_AGSK, 0},     /* AGSK,+1                   */
    {1, false, AGC_DN_LASTYCMD, 0}, /* LASTYCMD,LASTXCMD         */
    {1, false, AGC_DN_LEMMASS, 0},  /* LEMMASS,CSMMASS           */
    {1, false, AGC_DN_IMODES30, 0}, /* IMODES30,IMODES33         */
    {1, false, AGC_DN_TIG, 0},      /* TIG,+1                    */
    {PTR, false, LMORBM05, 0},      /* LMLSAL05: COMMON DATA     */
    {PTR, false, LMORBM06, 0},      /* LMLSAL06: COMMON DATA     */
    {1, false, AGC_DN_SPARE, 0},    {1, true, AGC_DN_SPARE, 0}};

/* ---------------- LM AGS INITIALIZATION / UPDATE LIST (LMAGSIDL), p.204 */
static const agc_dn_entry_t lmagsidl[] = {
    {3, false, AGC_DN_AGSBUFF, 0},  /* AGSBUFF +0...+5           */
    {1, false, AGC_DN_AGSBUFF, 12}, /* AGSBUFF +12D,GARBAGE      */
    {3, false, AGC_DN_AGSBUFF, 1},  /* AGSBUFF +1...+6           */
    {1, false, AGC_DN_AGSBUFF, 13}, /* AGSBUFF +13D,GARBAGE      */
    {3, false, AGC_DN_AGSBUFF, 6},  /* AGSBUFF +6...+11          */
    {1, false, AGC_DN_AGSBUFF, 12}, /* AGSBUFF +12D,GARBAGE      */
    {3, false, AGC_DN_AGSBUFF, 7},  /* AGSBUFF +7...+12D         */
    {1, false, AGC_DN_AGSBUFF, 13}, /* AGSBUFF +13D,GARBAGE      */
    {6, false, AGC_DN_COMPNUMB, 0}, /* COMPNUMB..UPCOUNT,UPBUFF +0..+7 */
    {6, false, AGC_DN_UPBUFF, 8},   /* UPBUFF +8D...+19D         */
    {PTR, false, LMORBM02, 0},      /* LMAGSI02: COMMON DATA     */
    {1, false, AGC_DN_TIME2, 0},    /* TIME2/1                   */
    {PTR, false, LMORBM03, 0},      /* LMAGSI03: COLLECT SNAPSHOT */
    {6, false, AGC_DN_DNTMBUFF, 0}, /* SEND SNAPSHOT             */
    {PTR, false, LMORBM04, 0},      /* LMAGSI04: COMMON DATA     */
    {2, false, AGC_DN_POSTORKU, 0}, /* POSTORKU,NEGTORKU,..V     */
    {1, false, AGC_DN_SPARE, 0},
    {1, false, AGC_DN_SPARE, 0},
    {1, false, AGC_DN_AGSK, 0},     /* AGSK,+1                   */
    {6, false, AGC_DN_UPBUFF, 0},   /* UPBUFF +0...+11D          */
    {4, false, AGC_DN_UPBUFF, 12},  /* UPBUFF +12D...+19D        */
    {1, false, AGC_DN_LEMMASS, 0},  /* LEMMASS,CSMMASS           */
    {1, false, AGC_DN_IMODES30, 0}, /* IMODES30,IMODES33         */
    {1, false, AGC_DN_SPARE, 0},
    {PTR, false, LMORBM05, 0},  /* LMAGSI05: COMMON DATA     */
    {6, true, AGC_DN_DSPTAB, 0} /* DSPTAB +0...+11D          */
};

#undef PTR
#undef CHN

/* DNTABLE (p.204): index = DNLSTCOD.                                      */
static const agc_dn_list_t dntable[AGC_DNLIST_LM_COUNT] = {
    {lmcstadl, (uint8_t) (sizeof lmcstadl / sizeof lmcstadl[0]),
     false}, /* 0 COAST/ALIGN (NOMDNLST) */
    {lmagsidl, (uint8_t) (sizeof lmagsidl / sizeof lmagsidl[0]),
     false}, /* 1 AGS INIT/UPDATE (UPDNLIST) */
    {lmrenddl, (uint8_t) (sizeof lmrenddl / sizeof lmrenddl[0]),
     false}, /* 2 RENDEZVOUS/PRETHRUST */
    {lmorbmdl, (uint8_t) (sizeof lmorbmdl / sizeof lmorbmdl[0]),
     false}, /* 3 ORBITAL MANEUVERS */
    {lmdsasdl, (uint8_t) (sizeof lmdsasdl / sizeof lmdsasdl[0]),
     false}, /* 4 DESCENT/ASCENT */
    {lmlsaldl, (uint8_t) (sizeof lmlsaldl / sizeof lmlsaldl[0]),
     false} /* 5 SURFACE ALIGN */
};

static const agc_dn_list_t sublists[LM_SUB_COUNT] = {
    {lmorbm01, (uint8_t) (sizeof lmorbm01 / sizeof lmorbm01[0]), true},
    {lmorbm02, (uint8_t) (sizeof lmorbm02 / sizeof lmorbm02[0]), false},
    {lmorbm03, (uint8_t) (sizeof lmorbm03 / sizeof lmorbm03[0]), true},
    {lmorbm04, (uint8_t) (sizeof lmorbm04 / sizeof lmorbm04[0]), false},
    {lmorbm05, (uint8_t) (sizeof lmorbm05 / sizeof lmorbm05[0]), false},
    {lmorbm06, (uint8_t) (sizeof lmorbm06 / sizeof lmorbm06[0]), false},
    {lmcsta06, (uint8_t) (sizeof lmcsta06 / sizeof lmcsta06[0]), false},
    {lmcsta07, (uint8_t) (sizeof lmcsta07 / sizeof lmcsta07[0]), false},
    {lmrend07, (uint8_t) (sizeof lmrend07 / sizeof lmrend07[0]), true},
    {lmdsas07, (uint8_t) (sizeof lmdsas07 / sizeof lmdsas07[0]), true},
    {lmdsas08, (uint8_t) (sizeof lmdsas08 / sizeof lmdsas08[0]), false}};

/* DNTABLE lookup: control list for downlist code CODE (DNLSTCOD).         */
agc_status_t agc_dnlist_lm_control(int code, agc_dn_list_t *out)
{
    if (!AGC_ASSERT(out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(code >= 0 && code < AGC_DNLIST_LM_COUNT))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *out = dntable[code];
    return AGC_STATUS_OK;
}

/* DNPTR target: sublist SUB.                                              */
agc_status_t agc_dnlist_lm_sublist(int sub, agc_dn_list_t *out)
{
    if (!AGC_ASSERT(out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(sub >= 0 && sub < LM_SUB_COUNT))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *out = sublists[sub];
    return AGC_STATUS_OK;
}
