/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private state and helpers of the Luminary 099 LM DAP, shared by the
 *   files of src/lm/dap only.
 *
 * AGC SOURCE:
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc pp. 105-136 (DAP erasables, EBANK 6)
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   none (the DAP erasables are one static lm_dap_t in state.c, reached
 *   through lm_dap_state(); each field below carries its AGC scale)
 *
 * NOTES:
 *   The AGC keeps the DAP in EBANK 6 erasables (ERASABLE_ASSIGNMENTS.agc
 *   pp. 105-136); here they are one struct, lm_dap_t, reached through
 *   lm_dap_state() (single dereference). Field names are the AGC labels in
 *   lower case; each carries the AGC scale.
 *
 *   Overlays that matter for behaviour are kept as the AGC has them:
 *     QERROR = DYERROR (high word), RERROR = DZERROR (high word): the RCS
 *     attitude errors and the manual-mode DP attitude errors share cells.
 *   Overlays that are only storage sharing inside one DAP pass (DAPTEMPn,
 *   DAPTREGn, E/EDOT = DAPTREG6/OMEGAV, URATEDIF = OMEGAU, ...) are separate
 *   fields or locals; every pass writes them before reading them, so the
 *   behaviour is the same.
 *
 *   Arithmetic follows the AGC word by word: MP, DV, DAS, MSU, CCS, the
 *   OVERSUB limiter and DVOVSUB are modelled in arith.c on ones'-complement
 *   values held in agc_sp_t (-0 is not represented; see Deviations in
 *   docs/modules/dap-lm.md).
 **********************************************************************/
#ifndef LM_DAP_INTERNAL_H
#define LM_DAP_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"

/* ---- RCSFLAGS bits (ERASABLE_ASSIGNMENTS p.105) ------------------------ */
#define LM_RCS_ZEROOR1  0x0001u /* bit 1: ALTERYZ switch                  */
#define LM_RCS_NEEDLE2  0x0002u /* bit 2: NEEDLER switch                  */
#define LM_RCS_NEEDLE3  0x0004u /* bit 3: NEEDLER initialise request      */
#define LM_RCS_DSPLYALT 0x0008u /* bit 4: 100 ms display alternation      */
#define LM_RCS_CALLGMBL 0x0010u /* bit 5: ACDT+C12 requested              */
#define LM_RCS_JUSTIN   0x0100u /* bit 9: just-in-detent                  */
#define LM_RCS_PBIT     0x0200u /* bit 10: P direct-rate control          */
#define LM_RCS_QRBIT    0x0400u /* bit 11: Q,R direct-rate control        */
#define LM_RCS_PSKIP    0x0800u /* bit 12: 1 = do P axis (0 = skip once)  */
#define LM_RCS_ACCSET   0x1000u /* bit 13: 1/ACCSET job started           */

/* ---- constants (octal value, decimal value, engineering value) --------- */
#define LM_POSMAX     ((agc_sp_t) 16383)  /* POSMAX 37777                  */
#define LM_NEGMAX     ((agc_sp_t) -16383) /* NEGMAX 40000                  */
#define LM_T6_POSMAX  037777u             /* TIME6 inactive                */
#define LM_T6_NEGZERO 077777u             /* TIME6 = -0: T6RUPT due        */
#define LM_MS100      037766u             /* MS100: TIME5 100 ms reload    */
#define LM_DAPZ_NEG   ((agc_sp_t) -10)    /* -TEN: no jask (STARTDAP)      */

/* Retained "RETJADR" bodies of the U,V jet loop (GENADR in RETJADR).      */
typedef enum lm_dap_uvbody
{
    LM_UV_TJLAW = 0, /* TJLAW +3: attitude (auto/hold)         */
    LM_UV_BACKHAND,  /* BACKHAND: manual direct rate           */
    LM_UV_MINRTN     /* MINRTN: minimum impulse                */
} lm_dap_uvbody_t;

/* T5ADR: which T5RUPT program runs next.                                  */
typedef enum lm_dap_t5adr
{
    LM_T5_DAPIDLER = 0,
    LM_T5_PAXIS
} lm_dap_t5adr_t;

#define LM_BLOCK_N 48 /* BLOCKTOP .. BLOCKTOP +47D               */
#define LM_BLK_U   16 /* 1/ANET1 = BLOCKTOP +16D (U axis block)  */

/* Offsets of the TJETLAW parameters inside one 16-word axis block
 * (relative to 1/ANET1 + AXISDIFF; ERASABLE_ASSIGNMENTS p.134).          */
#define LM_BK_1ANET1   0  /* 1/ANET1  .5/ACC 2^6/pi (+0..+3)         */
#define LM_BK_1ANET2   1  /* 1/ANET2  (= 1/ANET1 +1)                 */
#define LM_BK_1ACOAST  4  /* 1/ACOAST 2^6/pi (+4,+5)                 */
#define LM_BK_ACCFCTZ1 6  /* ACCFCTZ1 2^7/pi                          */
#define LM_BK_ACCFCTZ5 7  /* ACCFCTZ5                                 */
#define LM_BK_FIREDB   10 /* FIREDB   pi/4 (+10,+11)                 */
#define LM_BK_COASTDB  12 /* COASTDB  pi/4 (+12,+13)                 */
#define LM_BK_AXISDIST 14 /* AXISDIST pi/4 (+14,+15)                 */

typedef struct lm_dap
{
    /* permanent DAP storage (p.105) */
    uint16_t       rcsflags; /* RCSFLAGS, bits above */
    lm_dap_t5adr_t t5adr;    /* T5ADR */
    uint16_t       ch5mask;  /* CH5MASK: failed U,V jets, channel 5 format */
    uint16_t       ch6mask;  /* CH6MASK: failed P jets, channel 6 format */
    uint16_t       pvalvest; /* PVALVEST, channel 32 format */
    agc_sp_t       delperor; /* DELPEROR: KALCMANU command lag, scaled pi rad */
    agc_sp_t       delqeror; /* DELQEROR, scaled pi rad */
    agc_sp_t       delreror; /* DELREROR, scaled pi rad */
    agc_sp_t       db;       /* DB deadband, scaled pi/4 rad (45 deg) */

    /* pad loads (p.129) */
    agc_sp_t hiascent; /* HIASCENT, B16 kg */
    agc_sp_t dktrap;   /* DKTRAP: docked trap size, scaled 4.5 deg (77001 = .14 deg) */
    agc_sp_t dkomegan; /* DKOMEGAN: docked omega gain, unscaled count (10) */
    agc_sp_t dkkaosn;  /* DKKAOSN: docked AOS gain, unscaled count (60) */
    agc_sp_t lmtrap;   /* LMTRAP: LM-alone trap size, scaled 4.5 deg */
    agc_sp_t lmomegan; /* LMOMEGAN: LM-alone omega gain, unscaled count */
    agc_sp_t lmkaosn;  /* LMKAOSN: LM-alone AOS gain, unscaled count */
    agc_sp_t dkdb;     /* DKDB: deadband = pi/DKDB rad, unscaled divisor */
    agc_sp_t ignaosq;  /* IGNAOSQ pad load, scaled pi/2 rad/s^2 */
    agc_sp_t ignaosr;  /* IGNAOSR pad load, scaled pi/2 rad/s^2 */

    /* angle measurements */
    agc_sp_t omegap;   /* OMEGAP body rate, scaled pi/4 rad/s */
    agc_sp_t omegaq;   /* OMEGAQ, scaled pi/4 rad/s */
    agc_sp_t omegar;   /* OMEGAR, scaled pi/4 rad/s */
    agc_sp_t alphaq;   /* ALPHAQ: downlink copy of AOSQ, scaled pi/2 rad/s^2 */
    agc_sp_t alphar;   /* ALPHAR: downlink copy of AOSR, scaled pi/2 rad/s^2 */
    agc_sp_t omegau;   /* OMEGAU (also URATEDIF), scaled pi/4 rad/s */
    agc_sp_t omegav;   /* OMEGAV (also VRATEDIF), scaled pi/4 rad/s */
    agc_sp_t trapedp;  /* TRAPEDP: P rate residual sum, scaled pi/4 rad/s */
    agc_sp_t trapedq;  /* TRAPEDQ, scaled pi/4 rad/s */
    agc_sp_t trapedr;  /* TRAPEDR, scaled pi/4 rad/s */
    agc_sp_t nptraps;  /* NPTRAPS: P sample count, unscaled */
    agc_sp_t nqtraps;  /* NQTRAPS, unscaled count */
    agc_sp_t nrtraps;  /* NRTRAPS, unscaled count */
    agc_sp_t edotq;    /* EDOTQ = QRATEDIF: Q rate error, scaled pi/4 rad/s */
    agc_sp_t edotr;    /* EDOTR = RRATEDIF, scaled pi/4 rad/s */
    uint16_t oldxforp; /* OLDXFORP: last CDUX, 2's complement scaled pi rad */
    uint16_t oldyforp; /* OLDYFORP: last CDUY, 2's complement scaled pi rad */
    uint16_t oldzforq; /* OLDZFORQ: last CDUZ, 2's complement scaled pi rad */

    /* rate command / minimum impulse */
    uint16_t ch31temp;    /* CH31TEMP channel 31 image */
    agc_sp_t stiksens;    /* STIKSENS: quadratic RHC sensitivity, scaled 1 */
    agc_sp_t tcp;         /* TCP: P direct-rate timer, unscaled DAP passes (0.1 s) */
    agc_sp_t tcqr;        /* TCQR: Q,R direct-rate timer, unscaled DAP passes */
    agc_sp_t dxerror;     /* DXERROR major word, DP scaled pi rad */
    agc_sp_t dxerror_lo;  /* DXERROR minor word, scaled 2^-14 of the major */
    agc_sp_t qerror;      /* QERROR = DYERROR major word, scaled pi rad */
    agc_sp_t dyerror_lo;  /* DYERROR minor word, scaled 2^-14 of the major */
    agc_sp_t rerror;      /* RERROR = DZERROR major word, scaled pi rad */
    agc_sp_t dzerror_lo;  /* DZERROR minor word, scaled 2^-14 of the major */
    agc_sp_t plast;       /* PLAST: commanded P rate, scaled 45 deg/s */
    agc_sp_t qlast;       /* QLAST, scaled 45 deg/s */
    agc_sp_t rlast;       /* RLAST, scaled 45 deg/s */
    agc_sp_t oldpmin;     /* OLDPMIN: P minimum-impulse gate, unscaled */
    agc_sp_t oldqrmin;    /* OLDQRMIN: Q,R minimum-impulse gate, unscaled */
    agc_sp_t savehand[2]; /* SAVEHAND: Q,R RHC counts, unscaled */
    agc_sp_t perror;      /* PERROR, scaled pi rad */

    /* jet timing list (T6) */
    agc_sp_t        nxt6adr;     /* NXT6ADR axis index 0/4/13, unscaled */
    agc_sp_t        t6next[2];   /* T6NEXT: delta, scaled 0.625 ms per count; axis index */
    agc_sp_t        t6furtha[2]; /* T6FURTHA: delta, scaled 0.625 ms per count; axis index */
    uint16_t        nextp;       /* NEXTP: P jets after the T6RUPT (channel 6 code) */
    uint16_t        nextu;       /* NEXTU: U jets (channel 5 code) */
    uint16_t        nextv;       /* NEXTV: V jets (channel 5 code) */
    agc_sp_t        neg_2jetlim; /* -2JETLIM, scaled 45 deg/s */
    agc_sp_t        neg_ratedb;  /* -RATEDB = TARGETDB, scaled 45 deg/s */
    lm_dap_uvbody_t retjadr;     /* RETJADR */
    agc_sp_t        sensetyp;    /* SENSETYP 0, 1 (-X), 2 (+X), unscaled */

    /* trim gimbal */
    agc_sp_t neguq;    /* NEGUQ: Q drive sign -1, 0, +1 (unscaled) */
    agc_sp_t allowgts; /* ALLOWGTS (= NEGUQ +1): GTS permitted, unscaled */
    agc_sp_t negur;    /* NEGUR: R drive sign, unscaled */
    agc_sp_t kq;       /* KQ: Q jerk term, scaled pi/2^8 rad/s^3 */
    agc_sp_t axisctr;  /* AXISCTR (= SAVESR): -1 P, 0 U, 1 V (unscaled) */
    agc_sp_t krdap;    /* KRDAP: .3 ACCDOTR, scaled pi/2^8 rad/s^3 */
    agc_sp_t accdotq;  /* ACCDOTQ: Q gimbal jerk magnitude, scaled pi/2^7 rad/s^3 */
    agc_sp_t qaccdot;  /* QACCDOT: signed, scaled pi/2^7 rad/s^3 */
    agc_sp_t accdotr;  /* ACCDOTR, scaled pi/2^7 rad/s^3 */
    agc_sp_t raccdot;  /* RACCDOT: signed, scaled pi/2^7 rad/s^3 */
    agc_sp_t cotroler; /* COTROLER > 0 GTS, 0 try GTS, < 0 RCS (unscaled) */
    agc_sp_t qgimtimr; /* QGIMTIMR: Q drive timer, scaled 0.1 s per count */
    agc_sp_t ingts;    /* INGTS: current GTS control indicator, unscaled */
    agc_sp_t rgimtimr; /* RGIMTIMR: R drive timer, scaled 0.1 s per count */

    /* torque reconstruction */
    agc_sp_t downtork[6];       /* +-P +-U +-V jet torque, scaled 32 jet-s (wraps) */
    agc_sp_t nojets[3];         /* NO.PJETS, NO.UJETS, NO.VJETS, unscaled counts */
    agc_sp_t tj[3];             /* TJP, TJU, TJV: scaled 0.625 ms per count (T6) */
    agc_sp_t l_pvt_cg;          /* L,PVT-CG, scaled 8 ft */
    agc_sp_t jacc[5];           /* 1JACC,Q,R scaled pi/4 rad/s^2; 1JACCU,V scaled pi/2 */
    agc_sp_t skip[2];           /* SKIPU, SKIPV (0 = skip next pass), unscaled */
    agc_sp_t aosq;              /* AOSQ major word, DP scaled pi/2 rad/s^2 */
    agc_sp_t aosq_lo;           /* AOSQ minor word, scaled 2^-14 of the major */
    agc_sp_t aosr;              /* AOSR major word, DP scaled pi/2 rad/s^2 */
    agc_sp_t aosr_lo;           /* AOSR minor word, scaled 2^-14 of the major */
    agc_sp_t aosu;              /* AOSU, scaled pi/2 rad/s^2 */
    agc_sp_t aosv;              /* AOSV, scaled pi/2 rad/s^2 */
    agc_sp_t aosqterm;          /* AOSQTERM (.1-.05K)AOSQ, scaled pi/4 rad/s */
    agc_sp_t aosrterm;          /* AOSRTERM, scaled pi/4 rad/s */
    agc_sp_t block[LM_BLOCK_N]; /* BLOCKTOP: TJETLAW parameters, scales per LM_BK_* */
    agc_sp_t coeffq;            /* COEFFQ: ROT-TOUV coefficient, scaled 1 */
    agc_sp_t coeffr;            /* COEFFR, scaled 1 */

    /* KALCMANU interface */
    uint16_t cduxd;   /* CDUXD desired CDU, 2's complement scaled pi rad */
    uint16_t cduyd;   /* CDUYD, 2's complement scaled pi rad */
    uint16_t cduzd;   /* CDUZD, 2's complement scaled pi rad */
    agc_sp_t delcdux; /* DELCDUX: -desired 100 ms increment, scaled pi rad */
    agc_sp_t delcduy; /* DELCDUY, scaled pi rad */
    agc_sp_t delcduz; /* DELCDUZ, scaled pi rad */
    agc_sp_t omegapd; /* OMEGAPD desired rate, scaled pi/4 rad/s */
    agc_sp_t omegaqd; /* OMEGAQD, scaled pi/4 rad/s */
    agc_sp_t omegard; /* OMEGARD, scaled pi/4 rad/s */

    /* jask and docked jet inhibition */
    agc_sp_t dapzrupt;  /* DAPZRUPT > 0: jask in progress (unscaled) */
    agc_sp_t jetctr[3]; /* PJETCTR, UJETCTR, VJETCTR, unscaled DAP passes */

    /* needles */
    agc_sp_t ak[3];     /* AK, AK1, AK2, scaled 180 deg */
    agc_sp_t edrive[3]; /* EDRIVEX..Z, scaled 1800 deg */

    /* per-pass temporaries shared between routines */
    agc_sp_t e;          /* E: scaled pi rad, or pi/4 rad inside TJETLAW */
    agc_sp_t edot;       /* EDOT, scaled pi/4 rad/s */
    agc_sp_t uerror;     /* U attitude error, scaled pi rad */
    agc_sp_t verror;     /* V attitude error, scaled pi rad */
    agc_sp_t numbert;    /* NUMBERT (DAPTEMP5), unscaled jet-count code */
    agc_sp_t firefct;    /* FIREFCT (DAPTEMP3), scaled pi/4 rad */
    agc_sp_t rotindex;   /* ROTINDEX (DAPTEMP6), unscaled index */
    agc_sp_t polytemp;   /* POLYTEMP (DAPTEMP3), unscaled jet policy */
    agc_sp_t abstj;      /* ABSTJ (DAPTEMP1), scaled 0.625 ms per count */
    agc_sp_t jetrate[3]; /* JETRATE, JETRATEQ, JETRATER, scaled pi/4 rad/s */
    agc_sp_t torque[3];  /* DAPTEMP1..3: signed jet-s, scaled 1 */
    uint16_t dapcdu[3];  /* DAPTREG4..6: CDUX..Z read by PAXIS */
} lm_dap_t;

lm_dap_t *lm_dap_state(void);

/* ---- word-level arithmetic (arith.c) on agc_dp_words_t pairs ----------- */
agc_dp_words_t lm_dap_mp(agc_sp_t a, agc_sp_t b);              /* MP: A,L = a*b     */
agc_sp_t       lm_dap_mphi(agc_sp_t a, agc_sp_t b);            /* A after MP         */
agc_sp_t       lm_dap_dv(agc_dp_words_t n, agc_sp_t d);        /* DV quotient        */
agc_dp_words_t lm_dap_dvqr(agc_dp_words_t n, agc_sp_t d);      /* DV: (A, L)       */
agc_dp_words_t lm_dap_das(agc_dp_words_t x, agc_dp_words_t y); /* DAS (wraps)    */
agc_dp_words_t lm_dap_ddoubl(agc_dp_words_t x);                /* DDOUBL (wraps)     */
agc_sp_t       lm_dap_wrap(int32_t v);                         /* ADS/XCH store      */
agc_sp_t       lm_dap_sat(int32_t v);                          /* OVERSUB / LIMITS   */
bool           lm_dap_ovf(int32_t v);                          /* TS skip test       */
agc_sp_t       lm_dap_msu(uint16_t a, uint16_t b);             /* MSU: 1's at pi     */
uint16_t       lm_dap_ones_to_twos(int32_t v);                 /* MSU + 1STOTWOS     */
agc_sp_t       lm_dap_ccs_dim(agc_sp_t v);                     /* CCS: |v| - 1, >= 0 */
agc_sp_t       lm_dap_dim(agc_sp_t v);                         /* DIM               */
agc_sp_t       lm_dap_dvovsub(agc_dp_words_t n, agc_sp_t d);   /* DVOVSUB        */
int32_t        lm_dap_dpval(agc_dp_words_t x);                 /* hi*2^14 + lo       */
agc_dp_words_t lm_dap_dpmake(int32_t v);                       /* sign-agreeing pair */

/* ---- DAP interface (dapif.c) -------------------------------------------- */
agc_status_t lm_dap_callaccs(void);              /* CALLACCS: NOVAC 1/ACCJOB  */
void         lm_dap_set_t5adr(lm_dap_t5adr_t a); /* DXCH T5ADR             */

/* ---- idler and display (dapidler.c, needles.c) ------------------------- */
agc_status_t lm_dap_chekbits(bool *on); /* CHEKBITS: *on = DAP on    */
agc_status_t lm_dap_moreidle(void);     /* MOREIDLE                  */
agc_status_t lm_dap_shutdown(void);     /* SHUTDOWN + SETTIME5       */
agc_status_t lm_dap_startdap(void);     /* STARTDAP                  */
agc_status_t lm_dap_altdsply(void);     /* ALTDSPLY                  */
agc_status_t lm_dap_needler(void);      /* NEEDLER                   */

/* ---- P axis (paxis.c, pmanual.c, rate.c) -------------------------------- */
void         lm_dap_rateloop(void);             /* RATELOOP + ROTORQUE       */
agc_status_t lm_dap_backp(void);                /* BACKP rate derivation     */
agc_status_t lm_dap_offset_est(void);           /* offset estimator          */
agc_status_t lm_dap_pretimck(void);             /* PRETIMCK .. SKIPPAXS test */
agc_status_t lm_dap_chkvisfz(bool *policy);     /* CHKVISFZ: false = no policy */
agc_status_t lm_dap_pstate(void);               /* P state logic             */
agc_status_t lm_dap_purgency(void);             /* PURGENCY                  */
agc_status_t lm_dap_purgency_law(void);         /* PURGENCY +4               */
agc_status_t lm_dap_pjetslec(agc_sp_t numbert); /* PJETSLEC -1            */
agc_status_t lm_dap_pjetslec_pair(void);        /* PJETSLEC -6               */
agc_status_t lm_dap_jetsoff(void);              /* JETSOFF                   */
agc_status_t lm_dap_calcperr(void);             /* CALCPERR                  */
agc_status_t lm_dap_detentck(void);             /* DETENTCK                  */
agc_status_t lm_dap_zeroenbl(void);             /* ZEROENBL                  */

/* ---- Q,R axes (qraxis.c, qrmanual.c, qrjets.c) -------------------------- */
agc_status_t lm_dap_qraxis(void);   /* QRAXIS .. CLOSEOUT        */
void         lm_dap_qerrcalc(void); /* QERRCALC + RERRCALC       */
agc_status_t lm_dap_rcs(void);      /* RCS: Q,R RCS control      */
agc_status_t lm_dap_qrminimp(void); /* Q,R minimum impulse       */
agc_status_t lm_dap_chekstik(void); /* CHEKSTIK .. BACKHAND      */
agc_status_t lm_dap_uvloop(lm_dap_uvbody_t body, agc_sp_t first_axis);
agc_status_t lm_dap_xtrans(void);   /* XTRANS                    */
void         lm_dap_closeout(void); /* CLOSEOUT / ENDJASK        */
bool         lm_dap_selctsub(void); /* SELCTSUB: false = FAILOOP out */
agc_status_t lm_dap_stillrcs(void); /* STILLRCS / ATTSTEER       */
void         lm_dap_rot_touv(agc_sp_t q, agc_sp_t r, agc_sp_t *u, agc_sp_t *v);

/* ---- T6 (t6rupt.c) ------------------------------------------------------ */
agc_status_t lm_dap_write(agc_sp_t index, uint16_t jets); /* WRITEP/U/V    */
agc_status_t lm_dap_write_next(agc_sp_t index);           /* WRITEx -1     */
agc_status_t lm_dap_jtlst(void);                          /* JTLST (input T6FURTHA)    */
agc_sp_t     lm_dap_time6_value(void);                    /* TIME6 as a 1's value      */

/* ---- control laws ------------------------------------------------------- */
agc_status_t lm_dap_tjetlaw(void);                                                     /* TJETLAW (tjetlaw.c)       */
void         lm_dap_spsrcs(void);                                                      /* SPSRCS (spsrcs.c)         */
void         lm_dap_gts(void);                                                         /* GTS .. CLOSEOUT (gts.c)   */
void         lm_dap_gimbal_drive_update(void);                                         /* ACDT+C12 (gts.c)          */
agc_sp_t     lm_dap_gtsqrt(agc_dp_words_t *fn, agc_sp_t *shftflag, agc_sp_t *inindex); /* GTSQRT (gts.c)          */

/* ---- 1/ACCS (accs.c, accont.c) ------------------------------------------ */
agc_status_t lm_dap_switch_curves(void); /* 1/ACCONT (accont.c)       */
agc_status_t lm_dap_accfix(void);        /* 1/ACCFIX: NOVAC 1/ACCJOB  */

#endif /* LM_DAP_INTERNAL_H */
