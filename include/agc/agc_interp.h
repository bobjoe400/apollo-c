/*
 * agc_interp.h -- the interpretive language as a C library.
 *
 * Each interpretive instruction of INTERPRETER.agc (pp. 1107-1199 Comanche,
 * 1002-1094 Luminary) becomes one function here. The only interpreter state
 * that survives translation is agc_interp_t: the overflow indicator OVFIND,
 * the index registers X1 X2 S1 S2 (when used as data), and the ten
 * interpretive switches not covered by a named flagword.
 *
 * Scaling conventions (verify against INTERPRETER.agc "SECTION 3/4"):
 *   SIN/COS : argument in revolutions scaled 2^0 (0.5 = pi rad);
 *             result = sin(2 pi x) / 2   (scaled at 2)
 *   ASIN/ACOS: argument = sin(theta)/2 ... result = theta / 2pi
 *   SQRT    : result = sqrt(x) scaled so that sqrt(x/2^n) uses even n
 */
#ifndef AGC_INTERP_H
#define AGC_INTERP_H

#include <stdbool.h>
#include "agc_word.h"

#define AGC_VEC_N 3
#define AGC_INTERP_SWITCHES 16

typedef struct agc_vec {
    agc_dp_t c[AGC_VEC_N];           /* components, all at one scale        */
} agc_vec_t;

typedef struct agc_mat {
    agc_vec_t row[AGC_VEC_N];        /* row-major 3x3, all at one scale     */
} agc_mat_t;

typedef struct agc_interp {
    agc_ovf_t ovfind;                /* OVFIND                              */
    agc_sp_t  x1, x2, s1, s2;        /* X1 X2 S1 S2 index/step registers    */
    bool      sw[AGC_INTERP_SWITCHES];
} agc_interp_t;

void agc_interp_init(agc_interp_t *ip);
bool agc_interp_test_ovf(agc_interp_t *ip);      /* BOV: test and clear   */

/* ---- scalar DP ---------------------------------------------------------- */
agc_dp_t agc_dp_add(agc_interp_t *ip, agc_dp_t a, agc_dp_t b);   /* DAD   */
agc_dp_t agc_dp_sub(agc_interp_t *ip, agc_dp_t a, agc_dp_t b);   /* DSU   */
agc_dp_t agc_dp_mul(agc_interp_t *ip, agc_dp_t a, agc_dp_t b);   /* DMPR  */
agc_dp_t agc_dp_div(agc_interp_t *ip, agc_dp_t a, agc_dp_t b);   /* DDV   */
agc_dp_t agc_dp_sq(agc_interp_t *ip, agc_dp_t a);                /* DSQ   */
agc_dp_t agc_dp_sqrt(agc_interp_t *ip, agc_dp_t a);              /* SQRT  */
agc_dp_t agc_dp_sin(agc_interp_t *ip, agc_dp_t a);               /* SIN   */
agc_dp_t agc_dp_cos(agc_interp_t *ip, agc_dp_t a);               /* COS   */
agc_dp_t agc_dp_asin(agc_interp_t *ip, agc_dp_t a);              /* ASIN  */
agc_dp_t agc_dp_acos(agc_interp_t *ip, agc_dp_t a);              /* ACOS  */
agc_dp_t agc_dp_shl(agc_interp_t *ip, agc_dp_t a, int n);        /* SL n  */
agc_dp_t agc_dp_shl_round(agc_interp_t *ip, agc_dp_t a, int n);  /* SLR n */
agc_tp_t agc_tp_add(agc_interp_t *ip, agc_tp_t a, agc_tp_t b);   /* TAD   */

/* ---- vector ------------------------------------------------------------- */
agc_vec_t agc_vec_make(agc_dp_t x, agc_dp_t y, agc_dp_t z);      /* VDEF  */
agc_vec_t agc_vec_zero(void);
agc_vec_t agc_vec_add(agc_interp_t *ip, agc_vec_t u, agc_vec_t v);   /* VAD  */
agc_vec_t agc_vec_sub(agc_interp_t *ip, agc_vec_t u, agc_vec_t v);   /* VSU  */
agc_vec_t agc_vec_scale(agc_interp_t *ip, agc_vec_t v, agc_dp_t s);  /* VXSC */
agc_vec_t agc_vec_div(agc_interp_t *ip, agc_vec_t v, agc_dp_t s);    /* V/SC */
agc_dp_t  agc_vec_dot(agc_interp_t *ip, agc_vec_t u, agc_vec_t v);   /* DOT  */
agc_vec_t agc_vec_cross(agc_interp_t *ip, agc_vec_t u, agc_vec_t v); /* VXV  */
agc_dp_t  agc_vec_sq(agc_interp_t *ip, agc_vec_t v);                 /* VSQ  */
agc_dp_t  agc_vec_abval(agc_interp_t *ip, agc_vec_t v);              /* ABVAL*/
/* UNIT: returns unit vector; *abval = |v| (scaled as v), *vsq = |v|^2 as
 * left at 34D/36D by the AGC. Either out pointer may be NULL.            */
agc_vec_t agc_vec_unit(agc_interp_t *ip, agc_vec_t v,
                       agc_dp_t *abval, agc_dp_t *vsq);
agc_vec_t agc_vec_proj(agc_interp_t *ip, agc_vec_t u, agc_vec_t v);  /* VPROJ*/
agc_vec_t agc_vec_shl(agc_interp_t *ip, agc_vec_t v, int n);         /* VSL  */
agc_vec_t agc_vec_shr(agc_vec_t v, int n);                           /* VSR  */
agc_vec_t agc_vec_neg(agc_vec_t v);                                  /* VCOMP*/

/* ---- matrix ------------------------------------------------------------- */
agc_mat_t agc_mat_identity_half(void);     /* rows are UNITX/Y/Z (0.5)     */
agc_vec_t agc_mat_vec(agc_interp_t *ip, const agc_mat_t *m, agc_vec_t v);  /* MXV */
agc_vec_t agc_vec_mat(agc_interp_t *ip, agc_vec_t v, const agc_mat_t *m);  /* VXM */
agc_mat_t agc_mat_transpose(const agc_mat_t *m);
agc_mat_t agc_mat_mul(agc_interp_t *ip, const agc_mat_t *a, const agc_mat_t *b);

#endif /* AGC_INTERP_H */
