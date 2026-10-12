/**********************************************************************
 * FILE NAME: mpac.h
 *
 * PURPOSE:
 *   Private word-level model of the interpreter's MPAC for the
 *   core-numeric sources only (not part of the public include/agc
 *   contract). The AGC keeps a DP number as two 15-bit ones'-complement
 *   words whose signs may disagree (hi = +3, lo = -100 means
 *   3*2^-14 - 100*2^-28); INTERPRETER.agc works on those words with MP,
 *   DAS, DV, TS ... and its low-order bits depend on how the words
 *   split, so that word arithmetic is reproduced here: each word is an
 *   int32_t in (-2^14, +2^14), a DP pair is agc_w2_t, the TP MPAC,
 *   MPAC+1, MPAC+2 is agc_w3_t. Negative zero is not modelled.
 *
 * AGC SOURCE:
 *   INTERPRETER.agc (Comanche055 pp.1131-1191): DMPSUB, ROUNDSUB,
 *   DOTSUB, DSQSUB, VSQSUB, POLY, SQRTSUB, GENDDV, MAXDV, GENSCR, TSLC
 *
 * REQUIRES:
 *   agc/core/interp.h, agc/core/word.h
 *
 * GLOBAL VARIABLES:
 *   none
 *
 * NOTES:
 *   "A" values returned through int out-parameters are the AGC
 *   accumulator overflow indication (+1, 0, -1).
 **********************************************************************/
#ifndef AGC_MPAC_H
#define AGC_MPAC_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/word.h"

#define AGC_W_ONE     16384 /* 2^14: weight of the overflow bit (1.0)  */
#define AGC_W_POSMAX  16383 /* POSMAX, octal 37777                     */
#define AGC_W_HALF    8192  /* HALF, octal 20000                       */
#define AGC_W_QUARTER 4096  /* QUARTER, octal 10000                    */

typedef struct agc_w2
{
    int32_t hi; /* major word (MPAC)                       */
    int32_t lo; /* minor word (MPAC+1)                     */
} agc_w2_t;

typedef struct agc_w3
{
    int32_t m0; /* MPAC                                    */
    int32_t m1; /* MPAC+1                                  */
    int32_t m2; /* MPAC+2                                  */
} agc_w3_t;

/* ---- representation ----------------------------------------------------- */
agc_w2_t agc_w_split(agc_dp_t v);  /* sign-agreeing words    */
agc_dp_t agc_w_join(agc_w2_t p);   /* value of a word pair   */
agc_w3_t agc_w3_from_tp(agc_tp_t v);
agc_tp_t agc_w3_value(agc_w3_t m);
agc_w2_t agc_w3_top(agc_w3_t m);   /* MPAC, MPAC+1           */
int      agc_w_branch(agc_w3_t m); /* BRANCH: sign of the first non-zero word */

/* ---- public word form (word.h) <-> model words --------------------------- */
agc_w2_t       agc_w2_from_words(agc_dp_words_t w);
agc_dp_words_t agc_w2_to_words(agc_w2_t p);
agc_w3_t       agc_w3_from_words(agc_tp_words_t m);
agc_tp_words_t agc_w3_to_words(agc_w3_t m);

/* ---- single instructions ------------------------------------------------ */
agc_w2_t agc_w_mp(int32_t a, int32_t b);              /* MP            */
int32_t  agc_w_ts(int32_t a16, int *ovf);             /* TS correction */
int      agc_w_ovf(int32_t a16);                      /* TS: overflow indication only */
agc_w2_t agc_w_das(agc_w2_t x, agc_w2_t y, int *ovf); /* DAS / DDOUBL  */
agc_w2_t agc_w_dv(int32_t a, int32_t l, int32_t k);   /* DV: {q, r}    */
agc_w2_t agc_w_dcs(agc_w2_t p);                       /* DCS           */

/* ---- interpreter subroutines -------------------------------------------- */
agc_w2_t agc_w_overflow(agc_w2_t m, int a, agc_ovf_t *ovfind);                                            /* OVERFLOW */
agc_w2_t agc_w_dad(agc_w2_t m, agc_w2_t x, agc_ovf_t *ovfind);                                            /* DAS+OVERFLOW */
agc_w3_t agc_w_dmpsub(agc_w2_t mpac, agc_w2_t x);                                                         /* DMPSUB   */
agc_w3_t agc_w_roundsub(agc_w3_t m, agc_ovf_t *ovfind);                                                   /* ROUNDSUB */
agc_w3_t agc_w_dsqsub(agc_w2_t m);                                                                        /* DSQSUB   */
agc_w3_t agc_w_dot_acc(agc_w3_t buf, agc_w3_t t, agc_ovf_t *ovfind);                                      /* DOTSUB middle */
agc_w3_t agc_w_dotsub(agc_vec_t u, agc_vec_t v, agc_ovf_t *ovfind);                                       /* DOTSUB */
agc_w3_t agc_w_dotsub_words(const agc_w2_t u[AGC_VEC_N], const agc_w2_t v[AGC_VEC_N], agc_ovf_t *ovfind); /* DOTSUB, stored words */
agc_w3_t agc_w_vsqsub(agc_vec_t v, agc_ovf_t *ovfind);                                                    /* VSQSUB   */
agc_w3_t agc_w_tp_shl1(agc_w3_t m, int *ovf);                                                             /* TSSL step */
agc_w3_t agc_w_poly(agc_w2_t x, const agc_dp_t *coef, int degree);                                        /* POLY  */

/* SQRTSUB: normalised root (>= .5) of the TP MPAC in m0, m1, with m2 the
 * MPAC+2 SQRTSUB leaves (the argument's, shifted by the normalisation);
 * *shift = MPTEMP (right shift that un-normalises it); *abort = true
 * where the AGC does POODOO 01302 (argument < -2^-14).                 */
agc_w3_t agc_w_sqrtsub(agc_w3_t m, int *shift, bool *abort);

/* DDV/BDDV on the words: MPAC / BUF, overflow -> (+-POSMAX, +-POSMAX)
 * and *ovf = +1; *dvsign_neg = DVSIGN left at -0 (quotient sign
 * negative, also for a zero dividend over a negative divisor).        */
agc_w3_t agc_w_ddv(agc_w3_t m, agc_w2_t b, int *ovf, bool *dvsign_neg);

/* GENDDV/MAXDV: positive normalised operands (divisor hi >= HALF); the
 * minor words may disagree in sign with the major words.             */
agc_w2_t agc_w_genddv(agc_w2_t ab, agc_w2_t cd, bool maxdv);

/* TPAGREE (p.1141): the TP MPAC with its three words made to agree. */
agc_w3_t agc_w_tpagree(agc_w3_t m);

/* ---- shifts (GENSHIFT family, pp.1152-1161) ----------------------------- */
/* MPACSHR / MPACSRND: TP right shift by n in 1..14 (round = MPACSRND).     */
agc_w3_t agc_w_mpacshr(agc_w3_t m, int n, bool round);

/* GENSCR: scalar SR/SRR by n >= 1 (any count; word shifts above 13).       */
agc_w3_t agc_w_genscr(agc_w3_t m, int n, bool round);

/* VSHRRND: DP right shift by n in 1..14 with rounding (vector component). */
agc_w2_t agc_w_vshrrnd(agc_w2_t p, int n);

/* RIGHT/VRIGHT2: vector-component VSR by n >= 1 (rounds, double-rounds    */
/* past 14 exactly as the AGC).                                             */
agc_w2_t agc_w_vright(agc_w2_t p, int n);

/* TSSL: scalar SL by n (wraps, OVFIND on each overflowing step).           */
agc_w3_t agc_w_tssl(agc_w3_t m, int n, agc_ovf_t *ovfind);

/* TSLC (NORM): left-justify TP MPAC, *count = left shifts done.            */
agc_w3_t agc_w_tslc(agc_w3_t m, int *count);

#endif /* AGC_MPAC_H */
