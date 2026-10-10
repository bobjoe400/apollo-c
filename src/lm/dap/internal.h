/*
 * lm_dap_internal.h -- private state and helpers of the Luminary 099 LM DAP.
 *
 * The AGC keeps the DAP in EBANK 6 erasables (ERASABLE_ASSIGNMENTS.agc
 * pp. 105-136); here they are one struct, lm_dap_t, reached through
 * lm_dap_state() (single dereference). Field names are the AGC labels in
 * lower case; each carries the AGC scale.
 *
 * Overlays that matter for behaviour are kept as the AGC has them:
 *   QERROR = DYERROR (high word), RERROR = DZERROR (high word): the RCS
 *   attitude errors and the manual-mode DP attitude errors share cells.
 * Overlays that are only storage sharing inside one DAP pass (DAPTEMPn,
 * DAPTREGn, E/EDOT = DAPTREG6/OMEGAV, URATEDIF = OMEGAU, ...) are separate
 * fields or locals; every pass writes them before reading them, so the
 * behaviour is the same.
 *
 * Arithmetic follows the AGC word by word: MP, DV, DAS, MSU, CCS, the
 * OVERSUB limiter and DVOVSUB are modelled in arith.c on ones'-complement
 * values held in agc_sp_t (-0 is not represented; see Deviations in
 * docs/modules/dap-lm.md).
 */
#ifndef LM_DAP_INTERNAL_H
#define LM_DAP_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>
#include "agc/core/word.h"
#include "agc/core/status.h"
#include "agc/exec/exec.h"
#include "pending.h"

/* ---- RCSFLAGS bits (ERASABLE_ASSIGNMENTS p.105) ------------------------ */
#define LM_RCS_ZEROOR1   0x0001u  /* bit 1: ALTERYZ switch                  */
#define LM_RCS_NEEDLE2   0x0002u  /* bit 2: NEEDLER switch                  */
#define LM_RCS_NEEDLE3   0x0004u  /* bit 3: NEEDLER initialise request      */
#define LM_RCS_DSPLYALT  0x0008u  /* bit 4: 100 ms display alternation      */
#define LM_RCS_CALLGMBL  0x0010u  /* bit 5: ACDT+C12 requested              */
#define LM_RCS_JUSTIN    0x0100u  /* bit 9: just-in-detent                  */
#define LM_RCS_PBIT      0x0200u  /* bit 10: P direct-rate control          */
#define LM_RCS_QRBIT     0x0400u  /* bit 11: Q,R direct-rate control        */
#define LM_RCS_PSKIP     0x0800u  /* bit 12: 1 = do P axis (0 = skip once)  */
#define LM_RCS_ACCSET    0x1000u  /* bit 13: 1/ACCSET job started           */

/* ---- constants (octal value, decimal value, engineering value) --------- */
#define LM_POSMAX      ((agc_sp_t)16383)   /* POSMAX 37777                  */
#define LM_NEGMAX      ((agc_sp_t)-16383)  /* NEGMAX 40000                  */
#define LM_T6_POSMAX   037777u             /* TIME6 inactive                */
#define LM_T6_NEGZERO  077777u             /* TIME6 = -0: T6RUPT due        */
#define LM_MS100       037766u             /* MS100: TIME5 100 ms reload    */
#define LM_DAPZ_NEG    ((agc_sp_t)-10)     /* -TEN: no jask (STARTDAP)      */

/* Retained "RETJADR" bodies of the U,V jet loop (GENADR in RETJADR).      */
typedef enum lm_dap_uvbody {
    LM_UV_TJLAW = 0,             /* TJLAW +3: attitude (auto/hold)         */
    LM_UV_BACKHAND,              /* BACKHAND: manual direct rate           */
    LM_UV_MINRTN                 /* MINRTN: minimum impulse                */
} lm_dap_uvbody_t;

/* T5ADR: which T5RUPT program runs next.                                  */
typedef enum lm_dap_t5adr {
    LM_T5_DAPIDLER = 0,
    LM_T5_PAXIS
} lm_dap_t5adr_t;

#define LM_BLOCK_N 48            /* BLOCKTOP .. BLOCKTOP +47D               */
#define LM_BLK_U   16            /* 1/ANET1 = BLOCKTOP +16D (U axis block)  */

/* Offsets of the TJETLAW parameters inside one 16-word axis block
 * (relative to 1/ANET1 + AXISDIFF; ERASABLE_ASSIGNMENTS p.134).          */
#define LM_BK_1ANET1   0         /* 1/ANET1  .5/ACC 2^6/pi (+0..+3)         */
#define LM_BK_1ANET2   1         /* 1/ANET2  (= 1/ANET1 +1)                 */
#define LM_BK_1ACOAST  4         /* 1/ACOAST 2^6/pi (+4,+5)                 */
#define LM_BK_ACCFCTZ1 6         /* ACCFCTZ1 2^7/pi                          */
#define LM_BK_ACCFCTZ5 7         /* ACCFCTZ5                                 */
#define LM_BK_FIREDB   10        /* FIREDB   pi/4 (+10,+11)                 */
#define LM_BK_COASTDB  12        /* COASTDB  pi/4 (+12,+13)                 */
#define LM_BK_AXISDIST 14        /* AXISDIST pi/4 (+14,+15)                 */

typedef struct lm_dap {
    /* permanent DAP storage (p.105) */
    uint16_t  rcsflags;          /* RCSFLAGS, bits above                    */
    lm_dap_t5adr_t t5adr;        /* T5ADR                                   */
    uint16_t  ch5mask, ch6mask;  /* failed-jet masks, channel 5/6 format     */
    uint16_t  pvalvest;          /* PVALVEST, channel 32 format              */
    agc_sp_t  delperor, delqeror, delreror; /* KALCMANU command lags, at pi  */
    agc_sp_t  db;                /* DB deadband, at 45 deg                   */
    /* pad loads (p.129) */
    agc_sp_t  hiascent;          /* HIASCENT, B16 kg                         */
    agc_sp_t  dktrap, dkomegan, dkkaosn;  /* docked estimator gains         */
    agc_sp_t  lmtrap, lmomegan, lmkaosn;  /* LM-alone estimator gains       */
    agc_sp_t  dkdb;              /* DKDB: deadband = pi/DKDB rad             */
    agc_sp_t  ignaosq, ignaosr;  /* IGNAOSQ/R pad loads, at pi/2             */
    /* angle measurements */
    agc_sp_t  omegap, omegaq, omegar;   /* body rates, at pi/4 rad/s        */
    agc_sp_t  alphaq, alphar;    /* downlink copies of AOSQ/R, at pi/2       */
    agc_sp_t  omegau, omegav;    /* U,V rates (also URATEDIF/VRATEDIF)       */
    agc_sp_t  trapedp, trapedq, trapedr;  /* rate residual sums, at pi/4    */
    agc_sp_t  nptraps, nqtraps, nrtraps;  /* sample counts                  */
    agc_sp_t  edotq, edotr;      /* Q,R rate errors (QRATEDIF/RRATEDIF) pi/4  */
    uint16_t  oldxforp, oldyforp, oldzforq; /* last CDUs, 2's comp at pi    */
    /* rate command / minimum impulse */
    uint16_t  ch31temp;          /* CH31TEMP channel 31 image                */
    agc_sp_t  stiksens;          /* STIKSENS                                 */
    agc_sp_t  tcp, tcqr;         /* direct-rate timers, 0.1 s               */
    agc_sp_t  dxerror, dxerror_lo;  /* DXERROR DP at pi                     */
    agc_sp_t  qerror, dyerror_lo;   /* QERROR = DYERROR (high), DP at pi    */
    agc_sp_t  rerror, dzerror_lo;   /* RERROR = DZERROR (high), DP at pi    */
    agc_sp_t  plast, qlast, rlast;  /* commanded rates, at 45 deg/s         */
    agc_sp_t  oldpmin, oldqrmin; /* minimum-impulse gates                    */
    agc_sp_t  savehand[2];       /* SAVEHAND: Q,R RHC counts                 */
    agc_sp_t  perror;            /* PERROR at pi                             */
    /* jet timing list (T6) */
    agc_sp_t  nxt6adr;           /* NXT6ADR axis index 0/4/13                */
    agc_sp_t  t6next[2];         /* T6NEXT: delta counts, axis index         */
    agc_sp_t  t6furtha[2];       /* T6FURTHA                                 */
    uint16_t  nextp, nextu, nextv; /* jets after the T6RUPT (chan 6/5 codes) */
    agc_sp_t  neg_2jetlim;       /* -2JETLIM, at 45 deg/s                    */
    agc_sp_t  neg_ratedb;        /* -RATEDB = TARGETDB, at 45 deg/s          */
    lm_dap_uvbody_t retjadr;     /* RETJADR                                  */
    agc_sp_t  sensetyp;          /* SENSETYP 0, 1 (-X), 2 (+X)               */
    /* trim gimbal */
    agc_sp_t  neguq, allowgts, negur; /* NEGUQ, ALLOWGTS (=NEGUQ+1), NEGUR  */
    agc_sp_t  kq, axisctr, krdap;/* KQ at pi/2^8, AXISCTR (=SAVESR), KRDAP   */
    agc_sp_t  accdotq, qaccdot, accdotr, raccdot; /* jerks at pi/2^7        */
    agc_sp_t  cotroler;          /* COTROLER >0 GTS, 0 try, <0 RCS           */
    agc_sp_t  qgimtimr, ingts, rgimtimr; /* timers 0.1 s, INGTS              */
    /* torque reconstruction */
    agc_sp_t  downtork[6];       /* +-P +-U +-V jet torque, 32 jet-s (wraps) */
    agc_sp_t  nojets[3];         /* NO.PJETS, NO.UJETS, NO.VJETS             */
    agc_sp_t  tj[3];             /* TJP, TJU, TJV: T6 counts (0.625 ms)      */
    agc_sp_t  l_pvt_cg;          /* L,PVT-CG at 8 ft                         */
    agc_sp_t  jacc[5];           /* 1JACC,Q,R at pi/4; 1JACCU,V at pi/2      */
    agc_sp_t  skip[2];           /* SKIPU, SKIPV (0 = skip next pass)        */
    agc_sp_t  aosq, aosq_lo, aosr, aosr_lo; /* AOSQ/AOSR DP at pi/2         */
    agc_sp_t  aosu, aosv;        /* AOSU, AOSV at pi/2                       */
    agc_sp_t  aosqterm, aosrterm;/* (.1-.05K)AOS, at pi/4                    */
    agc_sp_t  block[LM_BLOCK_N]; /* BLOCKTOP: TJETLAW parameters (1/ACCONT)  */
    agc_sp_t  coeffq, coeffr;    /* ROT-TOUV coefficients                    */
    /* KALCMANU interface */
    uint16_t  cduxd, cduyd, cduzd; /* desired CDUs, 2's comp at pi           */
    agc_sp_t  delcdux, delcduy, delcduz; /* -desired 100 ms increments      */
    agc_sp_t  omegapd, omegaqd, omegard; /* desired rates at pi/4           */
    /* jask and docked jet inhibition */
    agc_sp_t  dapzrupt;          /* DAPZRUPT > 0: jask in progress           */
    agc_sp_t  jetctr[3];         /* PJETCTR, UJETCTR, VJETCTR                */
    /* needles */
    agc_sp_t  ak[3];             /* AK, AK1, AK2 at 180 deg                  */
    agc_sp_t  edrive[3];         /* EDRIVEX..Z at 1800 deg                   */
    /* per-pass temporaries shared between routines */
    agc_sp_t  e, edot;           /* E (pi, or pi/4 inside TJETLAW), EDOT pi/4 */
    agc_sp_t  uerror, verror;    /* U,V attitude errors at pi                */
    agc_sp_t  numbert;           /* NUMBERT (DAPTEMP5)                       */
    agc_sp_t  firefct;           /* FIREFCT (DAPTEMP3) at pi/4               */
    agc_sp_t  rotindex;          /* ROTINDEX (DAPTEMP6)                      */
    agc_sp_t  polytemp;          /* POLYTEMP (DAPTEMP3)                      */
    agc_sp_t  abstj;             /* ABSTJ (DAPTEMP1)                         */
    agc_sp_t  jetrate[3];        /* JETRATE, JETRATEQ, JETRATER at pi/4      */
    agc_sp_t  torque[3];         /* DAPTEMP1..3: signed jet-s at 1           */
    uint16_t  dapcdu[3];         /* DAPTREG4..6: CDUX..Z read by PAXIS       */
} lm_dap_t;

lm_dap_t *lm_dap_state(void);

/* ---- word-level arithmetic (arith.c) ----------------------------------- */
typedef struct lm_dp_pair {
    agc_sp_t hi;                 /* A / major word                           */
    agc_sp_t lo;                 /* L / minor word                           */
} lm_dp_pair_t;

lm_dp_pair_t lm_dap_mp(agc_sp_t a, agc_sp_t b);      /* MP: A,L = a*b     */
agc_sp_t     lm_dap_mphi(agc_sp_t a, agc_sp_t b);    /* A after MP         */
agc_sp_t     lm_dap_dv(lm_dp_pair_t n, agc_sp_t d);  /* DV quotient        */
lm_dp_pair_t lm_dap_dvqr(lm_dp_pair_t n, agc_sp_t d); /* DV: (A, L)       */
lm_dp_pair_t lm_dap_das(lm_dp_pair_t x, lm_dp_pair_t y); /* DAS (wraps)    */
lm_dp_pair_t lm_dap_ddoubl(lm_dp_pair_t x);          /* DDOUBL (wraps)     */
agc_sp_t     lm_dap_wrap(int32_t v);                 /* ADS/XCH store      */
agc_sp_t     lm_dap_sat(int32_t v);                  /* OVERSUB / LIMITS   */
bool         lm_dap_ovf(int32_t v);                  /* TS skip test       */
agc_sp_t     lm_dap_msu(uint16_t a, uint16_t b);     /* MSU: 1's at pi     */
uint16_t     lm_dap_ones_to_twos(int32_t v);               /* MSU + 1STOTWOS     */
agc_sp_t     lm_dap_ccs_dim(agc_sp_t v);             /* CCS: |v| - 1, >= 0 */
agc_sp_t     lm_dap_dim(agc_sp_t v);                 /* DIM               */
agc_sp_t     lm_dap_dvovsub(lm_dp_pair_t n, agc_sp_t d); /* DVOVSUB        */
int32_t      lm_dap_dpval(lm_dp_pair_t x);           /* hi*2^14 + lo       */
lm_dp_pair_t lm_dap_dpmake(int32_t v);               /* sign-agreeing pair */

/* ---- DAP interface (dapif.c) -------------------------------------------- */
agc_status_t lm_dap_callaccs(void);           /* CALLACCS: NOVAC 1/ACCJOB  */
agc_status_t lm_dap_set_t5adr(lm_dap_t5adr_t a); /* DXCH T5ADR             */

/* ---- idler and display (dapidler.c, needles.c) ------------------------- */
bool         lm_dap_chekbits(void);           /* CHEKBITS: true = DAP on   */
agc_status_t lm_dap_moreidle(void);           /* MOREIDLE                  */
agc_status_t lm_dap_shutdown(void);           /* SHUTDOWN + SETTIME5       */
agc_status_t lm_dap_startdap(void);           /* STARTDAP                  */
agc_status_t lm_dap_altdsply(void);           /* ALTDSPLY                  */
agc_status_t lm_dap_dsplytot(void);           /* DSPLYTOT                  */
agc_status_t lm_dap_needler(void);            /* NEEDLER                   */

/* ---- P axis (paxis.c, pmanual.c, rate.c) -------------------------------- */
agc_status_t lm_dap_paxis(void);              /* PAXIS T5RUPT program      */
agc_status_t lm_dap_rateloop(void);           /* RATELOOP + ROTORQUE       */
agc_status_t lm_dap_backp(void);              /* BACKP rate derivation     */
agc_status_t lm_dap_offset_est(void);         /* offset estimator          */
agc_status_t lm_dap_pretimck(void);           /* PRETIMCK .. SKIPPAXS test */
bool         lm_dap_chkvisfz(void);           /* CHKVISFZ: false = no policy */
agc_status_t lm_dap_pstate(void);             /* P state logic             */
agc_status_t lm_dap_purgency(void);           /* PURGENCY                  */
agc_status_t lm_dap_purgency_law(void);          /* PURGENCY +4               */
agc_status_t lm_dap_pjetslec(agc_sp_t numbert);  /* PJETSLEC -1            */
agc_status_t lm_dap_pjetslec_pair(void);          /* PJETSLEC -6               */
agc_status_t lm_dap_jetsoff(void);            /* JETSOFF                   */
agc_status_t lm_dap_calcperr(void);           /* CALCPERR                  */
agc_status_t lm_dap_detentck(void);           /* DETENTCK                  */
agc_status_t lm_dap_pminimp(void);            /* P minimum impulse         */
agc_status_t lm_dap_zeroenbl(void);           /* ZEROENBL                  */

/* ---- Q,R axes (qraxis.c, qrmanual.c, qrjets.c) -------------------------- */
agc_status_t lm_dap_qraxis(void);             /* QRAXIS .. CLOSEOUT        */
agc_status_t lm_dap_qerrcalc(void);           /* QERRCALC + RERRCALC       */
agc_status_t lm_dap_rcs(void);                /* RCS: Q,R RCS control      */
agc_status_t lm_dap_qrminimp(void);           /* Q,R minimum impulse       */
agc_status_t lm_dap_chekstik(void);           /* CHEKSTIK .. BACKHAND      */
agc_status_t lm_dap_uvloop(lm_dap_uvbody_t body, agc_sp_t first_axis);
agc_status_t lm_dap_xtrans(void);             /* XTRANS                    */
agc_status_t lm_dap_closeout(void);           /* CLOSEOUT / ENDJASK        */
bool         lm_dap_selctsub(void);           /* SELCTSUB: false = FAILOOP out */
agc_status_t lm_dap_stillrcs(void);           /* STILLRCS / ATTSTEER       */
void         lm_dap_rot_touv(agc_sp_t q, agc_sp_t r, agc_sp_t *u, agc_sp_t *v);

/* ---- T6 (t6rupt.c) ------------------------------------------------------ */
agc_status_t lm_dap_write(agc_sp_t index, uint16_t jets); /* WRITEP/U/V    */
agc_status_t lm_dap_write_next(agc_sp_t index);           /* WRITEx -1     */
agc_status_t lm_dap_jtlst(void);              /* JTLST (input T6FURTHA)    */
agc_sp_t     lm_dap_time6_value(void);        /* TIME6 as a 1's value      */

/* ---- control laws ------------------------------------------------------- */
agc_status_t lm_dap_tjetlaw(void);            /* TJETLAW (tjetlaw.c)       */
agc_status_t lm_dap_spsrcs(void);             /* SPSRCS (spsrcs.c)         */
agc_status_t lm_dap_gts(void);                /* GTS .. CLOSEOUT (gts.c)   */
agc_status_t lm_dap_acdt_c12(void);           /* ACDT+C12 (gts.c)          */
agc_sp_t     lm_dap_gtsqrt(lm_dp_pair_t *fn, agc_sp_t *shftflag,
                           agc_sp_t *inindex);  /* GTSQRT (gts.c)          */

/* ---- 1/ACCS (accs.c, accont.c) ------------------------------------------ */
agc_status_t lm_dap_switch_curves(void);           /* 1/ACCONT                  */
agc_status_t lm_dap_docked(void);             /* DOCKED inertia fit        */
agc_status_t lm_dap_spscont(agc_sp_t accdotq);/* SPSCONT .. 1/ACCRET       */

#endif /* LM_DAP_INTERNAL_H */
