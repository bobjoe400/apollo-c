/**********************************************************************
 * FILE NAME: interp_words.h
 *
 * PURPOSE:
 *   The interpretive instructions on AGC words: MPAC as the three words
 *   MPAC, MPAC+1, MPAC+2 (agc_tp_words_t) and DP operands as the two
 *   words they are stored in (agc_dp_words_t), signs free to disagree,
 *   exactly as the AGC holds them.
 *
 * AGC SOURCE:
 *   INTERPRETER.agc (Comanche055 pp.1107-1191, Luminary099
 *   pp.1002-1086): DLOAD, STORE, DAD, DSU, BDSU, TAD, DMP, DMPR, DSQ,
 *   DDV, BDDV, SQRT, ROUND, SL, SR, SLR, SRR, NORM, DCOMP, ABS, SIGN,
 *   SIN, COS, ASIN, ACOS, DOT, VSQ.
 *
 * REQUIRES:
 *   agc/core/interp.h, agc/core/word.h
 *
 * GLOBAL VARIABLES:
 *   none (agc_interp_t is passed by the caller)
 *
 * NOTES:
 * Use these where agc_dp_t (interp.h) loses bits: a DAS, DSU or BDSU
 * leaves a pair whose words disagree in sign (R1A = 00401 40000 from
 * PARAM's BDSU, 1 - cos A = 00001 40057, TBUP after DSU), or an
 * instruction leaves MPAC+2 that the next one uses (DMP or SQRT then
 * ROUND, SR then ASIN, DOT then SL). The low bits of a following DMP,
 * DDV, SQRT, SR or ROUND depend on the words, not only the value. The
 * value API stays the common path; it is this code on sign-agreeing
 * words, so the two agree wherever the words agree.
 *
 * Each function is one interpretive instruction: it takes MPAC (and an
 * operand as stored) and returns MPAC as the AGC leaves it. A sequence
 * DLOAD X / DMP Y / SR 4 / STORE Z is
 *     m = agc_mpac_dload(x); m = agc_mpac_dmp(m, y);
 *     m = agc_mpac_shift(ip, m, -4); z = agc_mpac_store(m);
 * Scalings are those of interp.h. OVFIND, DVSIGN and the SQRT/ARCCOS
 * alarm latch (agc_interp_take_alarm) behave as in interp.h. -0 is not
 * modelled: a word the AGC leaves as -0 is +0 here.
 * Verified against yaAGC word for word: tests/oracle/core_words.c.
 **********************************************************************/
#ifndef AGC_INTERP_WORDS_H
#define AGC_INTERP_WORDS_H

#include "agc/core/interp.h"
#include "agc/core/word.h"

/* ---- load / store ------------------------------------------------------- */
agc_tp_words_t agc_mpac_dload(agc_dp_words_t x); /* DLOAD: MPAC+2 = 0           */
agc_dp_words_t agc_mpac_store(agc_tp_words_t m); /* STORE: MPAC, MPAC+1 as held */

/* ---- add (DAS into MPAC, then OVERFLOW: wraps, OVFIND = +-1) ------------ */
agc_tp_words_t agc_mpac_dad(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x);  /* DAD  */
agc_tp_words_t agc_mpac_dsu(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x);  /* DSU  */
agc_tp_words_t agc_mpac_bdsu(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x); /* BDSU: x - MPAC */
agc_tp_words_t agc_mpac_tad(agc_interp_t *ip, agc_tp_words_t m, agc_tp_words_t x);  /* TAD  */

/* ---- multiply ----------------------------------------------------------- */
agc_tp_words_t agc_mpac_dmp(agc_tp_words_t m, agc_dp_words_t x);                    /* DMP: TP product */
agc_tp_words_t agc_mpac_dmpr(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x); /* DMPR           */
agc_tp_words_t agc_mpac_dsq(agc_tp_words_t m);                                      /* DSQ: TP square  */

/* ---- divide: quotient scaled 2^0; |dividend| >= |divisor| gives
 * (+-POSMAX, +-POSMAX) and OVFIND = 1; DVSIGN kept in ip ------------------ */
agc_tp_words_t agc_mpac_ddv(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x);  /* DDV:  MPAC / x */
agc_tp_words_t agc_mpac_bddv(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x); /* BDDV: x / MPAC */

/* ---- square root: sqrt of the TP MPAC at the same binary point; MPAC+2
 * as the AGC leaves it; argument < -2^-14 latches ABORT 01302, result 0  */
agc_tp_words_t agc_mpac_sqrt(agc_tp_words_t m); /* SQRT */

/* ---- shifts and rounding ------------------------------------------------- */
agc_tp_words_t agc_mpac_round(agc_interp_t *ip, agc_tp_words_t m);              /* ROUND          */
agc_tp_words_t agc_mpac_shift(agc_interp_t *ip, agc_tp_words_t m, int n);       /* SL n, SR -n    */
agc_tp_words_t agc_mpac_shift_round(agc_interp_t *ip, agc_tp_words_t m, int n); /* SLR n, SRR -n  */

/* NORM: TPAGREE, then shift left until |MPAC| >= 1/2; *nshift = left
 * shifts (the AGC stores -*nshift at the operand address).              */
agc_tp_words_t agc_mpac_norm(agc_tp_words_t m, int *nshift);

/* ---- sign ---------------------------------------------------------------- */
agc_tp_words_t agc_mpac_dcomp(agc_tp_words_t m);                  /* DCOMP: all three words   */
agc_tp_words_t agc_mpac_abs(agc_tp_words_t m);                    /* ABS: BRANCH, then DCOMP  */
agc_tp_words_t agc_mpac_sign(agc_tp_words_t m, agc_dp_words_t x); /* SIGN: DCOMP if x < 0     */

/* ---- trigonometric (scalings as interp.h); TP MPAC out ------------------- */
agc_tp_words_t agc_mpac_sin(agc_tp_words_t m);  /* SIN                       */
agc_tp_words_t agc_mpac_cos(agc_tp_words_t m);  /* COS                       */
agc_tp_words_t agc_mpac_asin(agc_interp_t *ip, agc_tp_words_t m); /* ASIN: ALARM 01301 latched */
agc_tp_words_t agc_mpac_acos(agc_interp_t *ip, agc_tp_words_t m); /* ACOS: ALARM 01301 latched */

/* ---- vector products as the TP MPAC (OVFIND as DOT/VSQ) ------------------ */
agc_tp_words_t agc_mpac_dot(agc_interp_t *ip, agc_vec_t u, agc_vec_t v); /* DOT */
agc_tp_words_t agc_mpac_vsq(agc_interp_t *ip, agc_vec_t v);              /* VSQ */

/* ---- basic-language DAS on stored words (no OVERFLOW correction): the
 * low-word carry goes into the high word; *ovf = overflow left in A.   */
agc_dp_words_t agc_dp_words_das(agc_dp_words_t x, agc_dp_words_t y, int *ovf);

#endif /* AGC_INTERP_WORDS_H */
