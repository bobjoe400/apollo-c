/**********************************************************************
 * FILE NAME: word.h
 *
 * PURPOSE:
 *   The AGC numeric model: SP/DP/TP fixed-point types, their limits,
 *   word structure, raw (non-interpretive) arithmetic and SPSIN/SPCOS.
 *
 * AGC SOURCE:
 *   none for the types (the 15-bit ones'-complement word); SPSIN/SPCOS
 *   from SINGLE_PRECISION_SUBROUTINES.agc p.1207.
 *
 * REQUIRES:
 *   <stdbool.h>, <stdint.h>
 *
 * GLOBAL VARIABLES:
 *   none
 *
 * NOTES:
 *   The AGC is a 15-bit ones' complement machine. Every quantity is a
 *   signed fraction in [-1, +1) scaled by a documented power of two:
 *
 *     SP  agc_sp_t  int16_t  sign + 14 magnitude bits   value = v / 2^14
 *     DP  agc_dp_t  int32_t  sign + 28 magnitude bits   value = v / 2^28
 *     TP  agc_tp_t  int64_t  sign + 42 magnitude bits   value = v / 2^42
 *
 *   Negative zero is not represented (docs/TRANSLATION_GUIDE.md section
 *   1). The raw operations here do NOT track the interpreter overflow
 *   indicator; flight algorithms use the interp.h functions, which do.
 **********************************************************************/
#ifndef AGC_WORD_H
#define AGC_WORD_H

#include <stdbool.h>
#include <stdint.h>

typedef int16_t agc_sp_t;
typedef int32_t agc_dp_t;
typedef int64_t agc_tp_t;

#define AGC_SP_BITS    14 /* SP magnitude bits */
#define AGC_DP_BITS    28 /* DP magnitude bits */
#define AGC_TP_BITS    42 /* TP magnitude bits */
#define AGC_SP_MAX     ((agc_sp_t) 0x3FFF)     /* POSMAX, octal 37777     */
#define AGC_SP_MIN     ((agc_sp_t) -0x3FFF)    /* NEGMAX                   */
#define AGC_DP_MAX     ((agc_dp_t) 0x0FFFFFFF) /* DPPOSMAX                 */
#define AGC_DP_MIN     ((agc_dp_t) -0x0FFFFFFF) /* -DPPOSMAX                */
#define AGC_TP_MAX     ((agc_tp_t) 0x3FFFFFFFFFFLL) /* TP POSMAX, 3 words  */
#define AGC_TP_MIN     ((agc_tp_t) -0x3FFFFFFFFFFLL) /* -TP POSMAX         */
#define AGC_SP_HALF    ((agc_sp_t) 0x2000)     /* HALF, octal 20000        */
#define AGC_DP_HALF    ((agc_dp_t) 0x08000000) /* DPHALF                   */
#define AGC_DP_QUARTER ((agc_dp_t) 0x04000000) /* DP1/4TH                  */

/* Overflow indication: 0 none, +1 positive, -1 negative.                  */
typedef int8_t agc_ovf_t;

/* ---- conversions (tests, sim harness, DSKY only) ------------------------ */
agc_sp_t agc_sp_from_double(double x, int scale_pow2);
agc_dp_t agc_dp_from_double(double x, int scale_pow2);
double   agc_sp_to_double(agc_sp_t v, int scale_pow2);
double   agc_dp_to_double(agc_dp_t v, int scale_pow2);
double   agc_tp_to_double(agc_tp_t v, int scale_pow2);

/* ---- SP/DP/TP structure ------------------------------------------------- */
agc_sp_t agc_dp_high(agc_dp_t v);               /* upper SP word (MPAC)    */
agc_sp_t agc_dp_low(agc_dp_t v);                /* lower SP word (MPAC+1)  */
agc_dp_t agc_dp_pack(agc_sp_t hi, agc_sp_t lo); /* signed-word DP pair     */
agc_dp_t agc_dp_from_sp(agc_sp_t v);            /* SP in high word         */
agc_tp_t agc_tp_from_dp(agc_dp_t v);
agc_dp_t agc_tp_to_dp(agc_tp_t v);    /* truncate (drop MPAC+2)  */
agc_dp_t agc_tp_round_dp(agc_tp_t v); /* round (DMPR semantics)  */

/* ---- raw arithmetic: saturating, overflow reported through *ovf ---------- */
agc_sp_t agc_sp_add(agc_sp_t a, agc_sp_t b, agc_ovf_t *ovf);
agc_sp_t agc_sp_sub(agc_sp_t a, agc_sp_t b, agc_ovf_t *ovf);
agc_dp_t agc_sp_mul(agc_sp_t a, agc_sp_t b); /* MP: SP x SP -> DP     */
agc_dp_t agc_dp_add_raw(agc_dp_t a, agc_dp_t b, agc_ovf_t *ovf);
agc_dp_t agc_dp_sub_raw(agc_dp_t a, agc_dp_t b, agc_ovf_t *ovf);
agc_tp_t agc_dp_mul_tp(agc_dp_t a, agc_dp_t b);  /* DMP: exact TP product */
agc_dp_t agc_dp_mul_raw(agc_dp_t a, agc_dp_t b); /* DMPR: rounded to DP   */
agc_dp_t agc_dp_div_raw(agc_dp_t a, agc_dp_t b, agc_ovf_t *ovf); /* DDV   */
agc_dp_t agc_dp_shl_raw(agc_dp_t a, int n, agc_ovf_t *ovf); /* SL n  */
agc_dp_t agc_dp_shr(agc_dp_t a, int n);                     /* SR n (truncating)     */
agc_dp_t agc_dp_shr_round(agc_dp_t a, int n);               /* SRR n                 */
agc_dp_t agc_dp_neg(agc_dp_t a);                            /* DCOMP                 */
agc_dp_t agc_dp_abs(agc_dp_t a);                            /* ABS                   */
agc_dp_t agc_dp_round(agc_dp_t a);                          /* ROUND -> SP precision */
agc_dp_t agc_dp_sign(agc_dp_t a, agc_dp_t sign_of); /* SIGN X            */
agc_dp_t agc_dp_norm(agc_dp_t a, int *shift);       /* NORM: left-justify   */
int      agc_dp_norm_count(agc_dp_t a);             /* NORM: count only     */
int      agc_dp_sgn(agc_dp_t a);                    /* -1, 0, +1             */

/* ---- SP transcendental, SINGLE_PRECISION_SUBROUTINES.agc p.1207 -------- */
/* Argument scaled at pi (i.e. v/2^14 * pi radians); result scaled at 1.    */
agc_sp_t agc_sp_sin(agc_sp_t a); /* SPSIN                 */
agc_sp_t agc_sp_cos(agc_sp_t a); /* SPCOS                 */

#endif /* AGC_WORD_H */
