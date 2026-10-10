/*
 * agc_constants.h -- AGC constant pools as C constants.
 *
 * FIXED_FIXED_CONSTANT_POOL.agc (Comanche055 pp.1200-1204, Luminary099
 * pp.1095-1099) and INTERPRETIVE_CONSTANTS.agc (Comanche055 pp.1205-1206;
 * Luminary099 file INTERPRETIVE_CONSTANT.agc pp.1100-1101). Names are the
 * AGC labels upper-cased with '/', '-', '&', '+' and ',' mapped to '_' or
 * 'P' ("plus"), a leading '-' spelled MINUS_ and a leading '.' spelled PT.
 * Every SP value is the 15-bit ones'-complement word read as a signed
 * integer (value / 2^14 as a fraction); the octal word is in the comment.
 * -0 (NEG0, octal 77777) is not representable and is omitted.
 *
 * The pool is identical for CM and LM except that Luminary099 adds OCT33
 * (DEC27) and OCT35 (DEC29) and spells DEC-6/DEC-12 as DFC-6/DFC-12.
 */
#ifndef AGC_CONSTANTS_H
#define AGC_CONSTANTS_H

#include "agc/core/interp.h"
#include "agc/core/word.h"

/* ---- FIXED_FIXED_CONSTANT_POOL.agc (SP words) ---------------------------- */
/* DPOSMAX: must precede POSMAX */
#define AGC_K_DPOSMAX ((agc_sp_t) 16383) /* DPOSMAX   37777 */
/* POSMAX: +max, 1 - 2^-14 */
#define AGC_K_POSMAX ((agc_sp_t) 16383) /* POSMAX    37777 */
/* NEG1/2: -1/2; LIMITS = NEG1/2 (used by SIN) */
#define AGC_K_NEG1_2 ((agc_sp_t) -8192) /* NEG1/2    57777 */
/* LIMITS: = NEG1/2: LIMITS-1 POSMAX, LIMITS+1 BIT15 */
#define AGC_K_LIMITS ((agc_sp_t) -8192) /* LIMITS    57777 */
/* BIT15: = NEGMAX as a 15-bit word (-37777) */
#define AGC_K_BIT15 ((agc_sp_t) -16383) /* BIT15     40000 */
/* BIT14: 1/2 */
#define AGC_K_BIT14 ((agc_sp_t) 8192) /* BIT14     20000 */
/* BIT13: 1/4 */
#define AGC_K_BIT13 ((agc_sp_t) 4096) /* BIT13     10000 */
#define AGC_K_BIT12 ((agc_sp_t) 2048) /* BIT12     04000 */
#define AGC_K_BIT11 ((agc_sp_t) 1024) /* BIT11     02000 */
#define AGC_K_BIT10 ((agc_sp_t) 512)  /* BIT10     01000 */
#define AGC_K_BIT9  ((agc_sp_t) 256)  /* BIT9      00400 */
#define AGC_K_BIT8  ((agc_sp_t) 128)  /* BIT8      00200 */
#define AGC_K_BIT7  ((agc_sp_t) 64)   /* BIT7      00100 */
#define AGC_K_BIT6  ((agc_sp_t) 32)   /* BIT6      00040 */
#define AGC_K_BIT5  ((agc_sp_t) 16)   /* BIT5      00020 */
#define AGC_K_BIT4  ((agc_sp_t) 8)    /* BIT4      00010 */
#define AGC_K_BIT3  ((agc_sp_t) 4)    /* BIT3      00004 */
#define AGC_K_BIT2  ((agc_sp_t) 2)    /* BIT2      00002 */
#define AGC_K_BIT1  ((agc_sp_t) 1)    /* BIT1      00001 */
/* ZERO: NEG0 (-0, octal 77777) precedes it; -0 is not modelled */
#define AGC_K_ZERO  ((agc_sp_t) 0) /* ZERO      00000 */
#define AGC_K_FIVE  ((agc_sp_t) 5) /* FIVE      00005 */
#define AGC_K_SEVEN ((agc_sp_t) 7) /* SEVEN     00007 */
/* ELEVEN: DEC 11 */
#define AGC_K_ELEVEN   ((agc_sp_t) 11) /* ELEVEN    00013 */
#define AGC_K_OCT15    ((agc_sp_t) 13) /* OCT15     00015 */
#define AGC_K_LOW4     ((agc_sp_t) 15) /* LOW4      00017 */
#define AGC_K_BITS4_5  ((agc_sp_t) 24) /* BITS4&5   00030 */
#define AGC_K_CALLCODE ((agc_sp_t) 26) /* CALLCODE  00032 */
/* TBUILDFX: DEC 37, DAP testing */
#define AGC_K_TBUILDFX ((agc_sp_t) 37) /* TBUILDFX  00045 */
/* TDECAYFX: DEC 38, DAP testing */
#define AGC_K_TDECAYFX ((agc_sp_t) 38) /* TDECAYFX  00046 */
#define AGC_K_OCT50    ((agc_sp_t) 40) /* OCT50     00050 */
/* DEC45: DEC 45 */
#define AGC_K_DEC45 ((agc_sp_t) 45) /* DEC45     00055 */
/* SUPER011: superbank bits 011 */
#define AGC_K_SUPER011 ((agc_sp_t) 48) /* SUPER011  00060 */
/* .5SEC: DEC 50 (centiseconds) */
#define AGC_K_PT5SEC ((agc_sp_t) 50) /* .5SEC     00062 */
/* SUPER101: superbank bits 101 */
#define AGC_K_SUPER101 ((agc_sp_t) 80) /* SUPER101  00120 */
/* SUPER110: superbank bits 110 */
#define AGC_K_SUPER110 ((agc_sp_t) 96) /* SUPER110  00140 */
/* 1SEC: DEC 100 (centiseconds) */
#define AGC_K_1SEC ((agc_sp_t) 100) /* 1SEC      00144 */
/* 2SECS: DEC 200 */
#define AGC_K_2SECS ((agc_sp_t) 200) /* 2SECS     00310 */
/* GN/CCODE: set S/C control switch to G/N */
#define AGC_K_GN_CCODE ((agc_sp_t) 257) /* GN/CCODE  00401 */
/* 3SECS: DEC 300 */
#define AGC_K_3SECS ((agc_sp_t) 300) /* 3SECS     00454 */
/* 4SECS: DEC 400 */
#define AGC_K_4SECS ((agc_sp_t) 400) /* 4SECS     00620 */
#define AGC_K_LOW9  ((agc_sp_t) 511) /* LOW9      00777 */
/* C5/2: DEC .0363551 (SPSIN) */
#define AGC_K_C5_2 ((agc_sp_t) 596) /* C5/2      01124 */
/* V05N09: VN 0509 */
#define AGC_K_ALARM_CODES_DISPLAY ((agc_sp_t) 649) /* V05N09    01211 */
#define AGC_K_OCT1400             ((agc_sp_t) 768) /* OCT1400   01400 */
/* V06N22: VN 0622 */
#define AGC_K_NEW_GIMBAL_ANGLES_DISPLAY ((agc_sp_t) 790)  /* V06N22    01426 */
#define AGC_K_BITS2_10                  ((agc_sp_t) 1022) /* BITS2-10  01776 */
#define AGC_K_LOW10                     ((agc_sp_t) 1023) /* LOW10     01777 */
/* LOW7+2K: op code mask + bank 1 FBANK */
#define AGC_K_LOW7P2K  ((agc_sp_t) 1151)  /* LOW7+2K   02177 */
#define AGC_K_EBANK5   ((agc_sp_t) 1280)  /* EBANK5    02400 */
#define AGC_K_PRIO3    ((agc_sp_t) 1536)  /* PRIO3     03000 */
#define AGC_K_EBANK7   ((agc_sp_t) 1792)  /* EBANK7    03400 */
#define AGC_K_PRIO5    ((agc_sp_t) 2560)  /* PRIO5     05000 */
#define AGC_K_PRIO6    ((agc_sp_t) 3072)  /* PRIO6     06000 */
#define AGC_K_PRIO7    ((agc_sp_t) 3584)  /* PRIO7     07000 */
#define AGC_K_PRIO11   ((agc_sp_t) 4608)  /* PRIO11    11000 */
#define AGC_K_PRIO13   ((agc_sp_t) 5632)  /* PRIO13    13000 */
#define AGC_K_PRIO14   ((agc_sp_t) 6144)  /* PRIO14    14000 */
#define AGC_K_PRIO15   ((agc_sp_t) 6656)  /* PRIO15    15000 */
#define AGC_K_PRIO16   ((agc_sp_t) 7168)  /* PRIO16    16000 */
#define AGC_K_PRIO17   ((agc_sp_t) 7680)  /* PRIO17    17000 */
#define AGC_K_OCT17770 ((agc_sp_t) 8184)  /* OCT17770  17770 */
#define AGC_K_PRIO21   ((agc_sp_t) 8704)  /* PRIO21    21000 */
#define AGC_K_PRIO22   ((agc_sp_t) 9216)  /* PRIO22    22000 */
#define AGC_K_PRIO23   ((agc_sp_t) 9728)  /* PRIO23    23000 */
#define AGC_K_PRIO24   ((agc_sp_t) 10240) /* PRIO24    24000 */
#define AGC_K_PRIO25   ((agc_sp_t) 10752) /* PRIO25    25000 */
#define AGC_K_PRIO26   ((agc_sp_t) 11264) /* PRIO26    26000 */
#define AGC_K_PRIO27   ((agc_sp_t) 11776) /* PRIO27    27000 */
#define AGC_K_PRIO31   ((agc_sp_t) 12800) /* PRIO31    31000 */
/* C1/2: DEC .7853134 (SPSIN) */
#define AGC_K_C1_2     ((agc_sp_t) 12867) /* C1/2      31103 */
#define AGC_K_PRIO32   ((agc_sp_t) 13312) /* PRIO32    32000 */
#define AGC_K_PRIO33   ((agc_sp_t) 13824) /* PRIO33    33000 */
#define AGC_K_PRIO34   ((agc_sp_t) 14336) /* PRIO34    34000 */
#define AGC_K_PRIO35   ((agc_sp_t) 14848) /* PRIO35    35000 */
#define AGC_K_PRIO36   ((agc_sp_t) 15360) /* PRIO36    36000 */
#define AGC_K_PRIO37   ((agc_sp_t) 15872) /* PRIO37    37000 */
#define AGC_K_63_64P1  ((agc_sp_t) 16129) /* 63/64+1   37401 */
#define AGC_K_OCT37766 ((agc_sp_t) 16374) /* OCT37766  37766 */
#define AGC_K_OCT37774 ((agc_sp_t) 16380) /* OCT37774  37774 */
#define AGC_K_OCT37776 ((agc_sp_t) 16382) /* OCT37776  37776 */
/* DLOADCOD: interpreter op code word */
#define AGC_K_DLOADCOD ((agc_sp_t) -16371) /* DLOADCOD  40014 */
/* DLOAD*: interpreter op code word */
#define AGC_K_DLOAD_STAR  ((agc_sp_t) -16370) /* DLOAD*    40015 */
#define AGC_K_BIT15P6     ((agc_sp_t) -16351) /* BIT15+6   40040 */
#define AGC_K_OCT40200    ((agc_sp_t) -16255) /* OCT40200  40200 */
#define AGC_K_MINUS_BIT14 ((agc_sp_t) -8192)  /* -BIT14    57777 */
/* C3/2: DEC -.3216147 (SPSIN) */
#define AGC_K_C3_2      ((agc_sp_t) -5269) /* C3/2      65552 */
#define AGC_K_13_14_15  ((agc_sp_t) -4095) /* 13,14,15  70000 */
#define AGC_K_MINUS_1_8 ((agc_sp_t) -2048) /* -1/8      73777 */
#define AGC_K_HIGH4     ((agc_sp_t) -2047) /* HIGH4     74000 */
/* -ENDERAS: DEC -2001 */
#define AGC_K_MINUS_ENDERAS ((agc_sp_t) -2001) /* -ENDERAS  74056 */
#define AGC_K_HIGH9         ((agc_sp_t) -63)   /* HIGH9     77700 */
/* NEG3: DEC -3 */
#define AGC_K_NEG3 ((agc_sp_t) -3) /* NEG3      77774 */
#define AGC_K_NEG2 ((agc_sp_t) -2) /* NEG2      77775 */
/* NEGONE: DEC -1 */
#define AGC_K_NEGONE ((agc_sp_t) -1) /* NEGONE    77776 */

/* Page 1204: DEFINED BY EQUALS (and LM-only pool entries). */
/* MINUS1: = NEG1 */
#define AGC_K_MINUS1 ((agc_sp_t) -1) /* MINUS1    77776 */
/* NEG1: = NEGONE */
#define AGC_K_NEG1 ((agc_sp_t) -1) /* NEG1      77776 */
/* ONE: = BIT1 */
#define AGC_K_ONE ((agc_sp_t) 1) /* ONE       00001 */
/* TWO: = BIT2 */
#define AGC_K_TWO ((agc_sp_t) 2) /* TWO       00002 */
/* THREE: = OCTAL3 (interpreter) */
#define AGC_K_THREE ((agc_sp_t) 3) /* THREE     00003 */
/* LOW2: = THREE */
#define AGC_K_LOW2 ((agc_sp_t) 3) /* LOW2      00003 */
/* FOUR: = BIT3 */
#define AGC_K_FOUR ((agc_sp_t) 4) /* FOUR      00004 */
/* SIX: = REVCNT (interpreter) */
#define AGC_K_SIX ((agc_sp_t) 6) /* SIX       00006 */
/* LOW3: = SEVEN */
#define AGC_K_LOW3 ((agc_sp_t) 7) /* LOW3      00007 */
/* EIGHT: = BIT4 */
#define AGC_K_EIGHT ((agc_sp_t) 8) /* EIGHT     00010 */
/* NINE: = R2D1 (pinball) */
#define AGC_K_NINE ((agc_sp_t) 9) /* NINE      00011 */
/* TEN: = BINCON (pinball) */
#define AGC_K_TEN ((agc_sp_t) 10) /* TEN       00012 */
/* NOUTCON: = ELEVEN */
#define AGC_K_NOUTCON ((agc_sp_t) 11) /* NOUTCON   00013 */
/* OCT23: = VD1 */
#define AGC_K_OCT23 ((agc_sp_t) 19) /* OCT23     00023 */
/* OCT25: = MD1 */
#define AGC_K_OCT25 ((agc_sp_t) 21) /* OCT25     00025 */
/* PRIO1: = BIT10 */
#define AGC_K_PRIO1 ((agc_sp_t) 512) /* PRIO1     01000 */
/* EBANK3: = OCT1400 */
#define AGC_K_EBANK3 ((agc_sp_t) 768) /* EBANK3    01400 */
/* PRIO2: = BIT11 */
#define AGC_K_PRIO2 ((agc_sp_t) 1024) /* PRIO2     02000 */
/* OCT120: = SUPER101 */
#define AGC_K_OCT120 ((agc_sp_t) 80) /* OCT120    00120 */
/* OCT140: = SUPER110 */
#define AGC_K_OCT140 ((agc_sp_t) 96) /* OCT140    00140 */
/* 2K: = BIT11 */
#define AGC_K_2K ((agc_sp_t) 1024) /* 2K        02000 */
/* EBANK4: = BIT11 */
#define AGC_K_EBANK4 ((agc_sp_t) 1024) /* EBANK4    02000 */
/* PRIO4: = BIT12 */
#define AGC_K_PRIO4 ((agc_sp_t) 2048) /* PRIO4     04000 */
/* EBANK6: = PRIO3 */
#define AGC_K_EBANK6 ((agc_sp_t) 1536) /* EBANK6    03000 */
/* QUARTER: = BIT13 */
#define AGC_K_QUARTER ((agc_sp_t) 4096) /* QUARTER   10000 */
/* PRIO10: = BIT13 */
#define AGC_K_PRIO10 ((agc_sp_t) 4096) /* PRIO10    10000 */
/* POS1/2: = HALF */
#define AGC_K_POS1_2 ((agc_sp_t) 8192) /* POS1/2    20000 */
/* PRIO20: = BIT14 */
#define AGC_K_PRIO20 ((agc_sp_t) 8192) /* PRIO20    20000 */
/* HALF: = BIT14 */
#define AGC_K_HALF ((agc_sp_t) 8192) /* HALF      20000 */
/* PRIO30: = CHRPRIO */
#define AGC_K_PRIO30 ((agc_sp_t) 12288) /* PRIO30    30000 */
/* BIT13-14: = PRIO30 */
#define AGC_K_BIT13_14 ((agc_sp_t) 12288) /* BIT13-14  30000 */
/* B12T14: = PRIO34 */
#define AGC_K_B12T14 ((agc_sp_t) 14336) /* B12T14    34000 */
/* NEGMAX: = BIT15, -37777 */
#define AGC_K_NEGMAX ((agc_sp_t) -16383) /* NEGMAX    40000 */
/* VLOADCOD: = BIT15 */
#define AGC_K_VLOADCOD ((agc_sp_t) -16383) /* VLOADCOD  40000 */
/* OCT60000: = RELTAB11 */
#define AGC_K_OCT60000 ((agc_sp_t) -8191) /* OCT60000  60000 */
/* OCT33: LM only (Luminary099); DEC27 = OCT33 */
#define AGC_K_LM_OCT33 ((agc_sp_t) 27) /* OCT33     00033 */
/* OCT35: LM only (Luminary099); DEC29 = OCT35 */
#define AGC_K_LM_OCT35 ((agc_sp_t) 29) /* OCT35     00035 */

/* ---- INTERPRETIVE_CONSTANTS.agc (DP words, value / 2^28) ---------------- */
#define AGC_K_DP1_4TH ((agc_dp_t) 67108864)  /* DP1/4TH 2DEC .25: 10000 00000 */
#define AGC_K_DPHALF  ((agc_dp_t) 134217728) /* DPHALF = UNITX: 20000 00000   */
#define AGC_K_HALFDP  ((agc_dp_t) 134217728) /* HALFDP = XUNIT: 20000 00000   */
#define AGC_K_DPPOSMAX \
    ((agc_dp_t) 268435455)                   /* DPPOSMAX OCT 37777, 37777     */
#define AGC_K_LODPMAX ((agc_dp_t) 268435455) /* LODPMAX 2OCT 3777737777       */
#define AGC_K_LODPMAX1 \
    ((agc_dp_t) 268435455)          /* LODPMAX1 (adjacent, same)     */
#define AGC_K_ZERODP ((agc_dp_t) 0) /* ZERODP = ZEROVEC              */
/* SP words of the "-0, -6, -12 MUST REMAIN IN THIS ORDER" table.            */
#define AGC_K_DEC_6  ((agc_sp_t) -6)  /* DEC-6 (LM: DFC-6)  77771      */
#define AGC_K_DEC_12 ((agc_sp_t) -12) /* DEC-12 (LM: DFC-12) 77763     */

/* Half-unit vectors and the zero vector: UNITX/XUNIT = (.5, 0, 0),
 * UNITY/YUNIT = (0, .5, 0), UNITZ/ZUNIT = (0, 0, .5) (the AGC overlaps
 * them in one table), ZEROVECS/ZEROVEC = (0, 0, 0). Both half-memory copies
 * (INTPRET1 and INTPRET2) have the same values.                            */
agc_vec_t agc_k_unitx(void);
agc_vec_t agc_k_unity(void);
agc_vec_t agc_k_unitz(void);
agc_vec_t agc_k_zerovec(void);

#endif /* AGC_CONSTANTS_H */
