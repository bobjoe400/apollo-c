/**********************************************************************
 * FILE NAME: interp.h
 *
 * PURPOSE:
 *   The interpretive language as a C library: one function per
 *   interpretive instruction of INTERPRETER.agc, plus the agc_interp_t
 *   state (OVFIND, index registers, switches, DVSIGN).
 *
 * AGC SOURCE:
 *   INTERPRETER.agc pp.1107-1199 (Comanche055), pp.1002-1094
 *   (Luminary099); the arithmetic sections pp.1131-1191.
 *
 * REQUIRES:
 *   agc/core/status.h, agc/core/word.h
 *
 * GLOBAL VARIABLES:
 *   none (agc_interp_t is passed by the caller)
 *
 * NOTES:
 * The only interpreter state that survives translation is agc_interp_t:
 * the overflow indicator OVFIND, the index registers X1 X2 S1 S2 (when
 * used as data), the ten interpretive switches not covered by a named
 * flagword, and DVSIGN (= DOTINC), whose stale value signs the DPPOSMAX
 * of an early V/SC or UNIT overflow exit (docs/modules/core-numeric.md).
 *
 * The arithmetic reproduces the AGC word by word (DMPSUB, ROUNDSUB, DDV,
 * SQRTSUB, POLY, ...), so results match the flight computer to the bit for
 * operands whose AGC word pairs agree in sign (every agc_dp_t does).
 * Where the AGC's words disagree in sign (a DAS/DSU/BDSU result re-read by
 * DMP, DDV, SR, SQRT, SIN, ...) or an instruction uses the MPAC+2 the last
 * one left (SQRT ROUND, SR1 ASIN, DOT SL1), use interp_words.h.
 * Full semantics: docs/modules/core-numeric.md. Verified scalings
 * (INTERPRETER.agc pp.1131-1191):
 *
 *   SIN/COS : argument x in revolutions scaled 2^0 (0.5 = pi rad, any value
 *             in [-1, 1) allowed); result = sin(2 pi x)/2, cos(2 pi x)/2,
 *             i.e. scaled at 2.  (Hastings 4th-order polynomial, SN1.)
 *   ASIN/ACOS: argument a = sin(theta)/2 (scaled at 2, |a| <= 1/2);
 *             result = theta / 2pi in revolutions scaled 2^0:
 *             ACOS in [0, .5], ASIN in [-.25, .25]. |a| more than one
 *             major-word LSB above 1/2 -> result 0 and ALARM 01301
 *             (agc_dp_acos_chk / agc_dp_asin_chk report it).
 *   SQRT    : result = sqrt(a) with the SAME binary point, so an argument
 *             scaled 2^2n gives a root scaled 2^n (keep n integral: shift
 *             odd-scaled arguments first). a < -2^-14 aborts (POODOO
 *             01302, agc_dp_sqrt_chk); -2^-14 <= a < 0 gives 0.
 *   DDV     : a / b, both at the same scale, quotient scaled 2^0; |a| >= |b|
 *             or b = 0 -> +-DPPOSMAX and OVFIND = 1. Low two bits of the
 *             quotient follow GENDDV's first-order approximation.
 *   DMPR    : a b (scales add), DMPSUB rounded half away from zero (ROUNDSUB).
 *   DMP     : a b truncated to the top two words: agc_dp_mul_trunc, or
 *             agc_dp_mul_tp for the full TP MPAC.
 *   DAD/DSU/VAD/VSU/SL/VSL: on overflow the AGC keeps the overflow-corrected
 *             (wrapped, value -+ 1.0) result and sets OVFIND = +-1; these
 *             functions do the same (they do NOT saturate).
 *   SR/VSR  : SR truncates toward zero; SRR, VSR and every vector right
 *             shift round half away from zero.
 *   UNIT    : half-unit vector v/(2|v|); *abval = |v| (LV, scaled as v),
 *             *vsq = |v|^2 (LVSQUARE, scaled as v^2). |v|^2 >= 1 or v = 0
 *             -> (+-DPPOSMAX, ...) and OVFIND = 1 (see module note).
 *             The square root sees only the significant words of |v|^2,
 *             so a short vector (|v| below ~2^-14 of its scale, e.g. a
 *             few km at 2^27 m) gives a coarse direction, as in the AGC;
 *             flight code normalises first there (RTB NORMUNIT).
 *   DOT/VSQ/DSQ/VXV/MXV/VXM: TP products truncated to the top two words.
 *   VXSC/VPROJ: each component rounded (VROUND). VPROJ = (u.v) v.
 **********************************************************************/
#ifndef AGC_INTERP_H
#define AGC_INTERP_H

#include <stdbool.h>

#include "agc/core/status.h"
#include "agc/core/word.h"

#define AGC_VEC_N           3  /* components per vector (x, y, z) */
#define AGC_INTERP_SWITCHES 16 /* switch bits kept in sw[]        */

typedef struct agc_vec
{
    agc_dp_t c[AGC_VEC_N]; /* components, all at one scale        */
} agc_vec_t;

typedef struct agc_mat
{
    agc_vec_t row[AGC_VEC_N]; /* row-major 3x3, all at one scale     */
} agc_mat_t;

typedef struct agc_interp
{
    agc_ovf_t ovfind;                  /* OVFIND: 0, or +-1 (sign of the overflow)    */
    agc_sp_t  x1;                      /* X1 index register, unscaled (as data)       */
    agc_sp_t  x2;                      /* X2 index register, unscaled                 */
    agc_sp_t  s1;                      /* S1 step register, unscaled                  */
    agc_sp_t  s2;                      /* S2 step register, unscaled                  */
    bool      sw[AGC_INTERP_SWITCHES]; /* interpretive switch bits    */
    bool      dvsign_neg;              /* DVSIGN (= DOTINC) holds -0: last divide gave a negative quotient (see DVOVF) */
    agc_dp_t  mpac;                    /* MPAC as left by a DAD/DDV whose result only sets OVFIND (BOV next) */
} agc_interp_t;

void agc_interp_init(agc_interp_t *ip);
bool agc_interp_test_ovf(agc_interp_t *ip);  /* BOV: test and clear   */
void agc_interp_clear_ovf(agc_interp_t *ip); /* BOV +1: clear only  */

/* ---- scalar DP ---------------------------------------------------------- */
agc_dp_t agc_dp_add(agc_interp_t *ip, agc_dp_t a, agc_dp_t b);          /* DAD   */
void     agc_dp_add_ovf_only(agc_interp_t *ip, agc_dp_t a, agc_dp_t b); /* DAD, BOV */
void     agc_dp_div_ovf_only(agc_interp_t *ip, agc_dp_t a, agc_dp_t b); /* DDV, BOV */
agc_dp_t agc_dp_sub(agc_interp_t *ip, agc_dp_t a, agc_dp_t b);          /* DSU   */
agc_dp_t agc_dp_mul(agc_interp_t *ip, agc_dp_t a, agc_dp_t b);          /* DMPR  */
agc_dp_t agc_dp_div(agc_interp_t *ip, agc_dp_t a, agc_dp_t b);          /* DDV   */
agc_dp_t agc_dp_sq(agc_interp_t *ip, agc_dp_t a);                       /* DSQ   */
agc_dp_t agc_dp_sqrt(agc_interp_t *ip, agc_dp_t a);                     /* SQRT  */
agc_dp_t agc_dp_sin(agc_interp_t *ip, agc_dp_t a);                      /* SIN   */
agc_dp_t agc_dp_cos(agc_interp_t *ip, agc_dp_t a);                      /* COS   */
agc_dp_t agc_dp_asin(agc_interp_t *ip, agc_dp_t a);                     /* ASIN  */
agc_dp_t agc_dp_acos(agc_interp_t *ip, agc_dp_t a);                     /* ACOS  */
agc_dp_t agc_dp_shl(agc_interp_t *ip, agc_dp_t a, int n);               /* SL n  */
agc_dp_t agc_dp_shl_round(agc_interp_t *ip, agc_dp_t a, int n);         /* SLR n */
agc_tp_t agc_tp_add(agc_interp_t *ip, agc_tp_t a, agc_tp_t b);          /* TAD   */

/* ---- vector ------------------------------------------------------------- */
agc_vec_t agc_vec_make(agc_dp_t x, agc_dp_t y, agc_dp_t z);          /* VDEF  */
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
agc_vec_t agc_vec_unit(agc_interp_t *ip, agc_vec_t v, agc_dp_t *abval, agc_dp_t *vsq);
agc_dp_t  agc_vec_unit_abval(agc_interp_t *ip, agc_vec_t v); /* UNIT, LV only */
agc_vec_t agc_vec_proj(agc_interp_t *ip, agc_vec_t u, agc_vec_t v); /* VPROJ*/
agc_vec_t agc_vec_shl(agc_interp_t *ip, agc_vec_t v, int n);        /* VSL  */
agc_vec_t agc_vec_shr(agc_vec_t v, int n);                          /* VSR  */
agc_vec_t agc_vec_neg(agc_vec_t v);                                 /* VCOMP*/

/* ---- matrix ------------------------------------------------------------- */
agc_mat_t agc_mat_identity_half(void);                                    /* rows are UNITX/Y/Z (0.5)     */
agc_vec_t agc_mat_vec(agc_interp_t *ip, const agc_mat_t *m, agc_vec_t v); /* MXV */
agc_vec_t agc_vec_mat(agc_interp_t *ip, agc_vec_t v, const agc_mat_t *m); /* VXM */
agc_mat_t agc_mat_transpose(const agc_mat_t *m);
agc_mat_t agc_mat_mul(agc_interp_t *ip, const agc_mat_t *a, const agc_mat_t *b);

/* ---- additional operations (core-numeric) ------------------------------- */
/* DMP followed by DP use: top two words of the DMPSUB TP, no rounding.     */
agc_dp_t agc_dp_mul_trunc(agc_dp_t a, agc_dp_t b);

/* DSQ as the full TP MPAC (DSQSUB).                                        */
agc_tp_t agc_dp_sq_tp(agc_dp_t a);

/* SQRT/ASIN/ACOS with the AGC abort/alarm reported: AGC_STATUS_POODOO for
 * SQRT of a < -2^-14 (caller does agc_poodoo(01302)), AGC_STATUS_ALARM for
 * ASIN/ACOS out of range (caller does agc_alarm(01301)); *out is the value
 * the AGC leaves in MPAC (0 for the SQRT abort).                           */
agc_status_t agc_dp_sqrt_chk(agc_dp_t a, agc_dp_t *out);

/* The alarm the plain agc_dp_sqrt / asin / acos latched since the last
 * call (01302 SQRT of a negative: ABORT; 01301 ARCSIN/ARCCOS out of
 * range: ALARM; 0 none), cleared by reading. The executive raises it
 * when the job or task returns (core cannot call exec).                   */
uint16_t     agc_interp_take_alarm(void);
agc_status_t agc_dp_asin_chk(agc_interp_t *ip, agc_dp_t a, agc_dp_t *out);
agc_status_t agc_dp_acos_chk(agc_interp_t *ip, agc_dp_t a, agc_dp_t *out);

/* SQRT of a TP MPAC (TLOAD/DMP/DSQ then SQRT): MPAC+2 takes part; the
 * MPAC+2 SQRT leaves (used by a following ROUND) is in agc_mpac_sqrt.    */
agc_dp_t agc_tp_sqrt(agc_interp_t *ip, agc_tp_t a);

/* ROUND: TP MPAC rounded to DP (ROUNDSUB), OVFIND on carry out.            */
agc_dp_t agc_tp_round(agc_interp_t *ip, agc_tp_t a);

/* SL n (n > 0, wraps, OVFIND) or SR -n (n < 0, truncating) on the TP MPAC. */
agc_tp_t agc_tp_shift(agc_interp_t *ip, agc_tp_t v, int n);

/* NORM on the TP MPAC: left-justify until |major word| >= 1/2;
 * *nshift = left shifts done (the AGC stores -*nshift in the index reg).  */
agc_tp_t agc_tp_norm(agc_tp_t v, int *nshift);

/* SHORTMP (p.1141): TP MPAC times an SP fraction, TP result.               */
agc_tp_t agc_tp_mul_sp(agc_tp_t v, agc_sp_t k);

/* DMPNSUB (p.1142): DP fraction times an SP integer n (caller keeps the
 * product below 1 in magnitude, as the AGC requires).                      */
agc_dp_t agc_dp_mul_int(agc_dp_t v, agc_sp_t n);

/* POLY / POWRSERS (p.1139): coef[degree] x^degree + ... + coef[0], all DP
 * at the same scale as the result, 1 <= degree <= 15 (no overflow check). */
agc_dp_t agc_dp_poly(agc_dp_t x, const agc_dp_t *coef, int degree);

/* DOT and VSQ as the full TP MPAC.                                         */
agc_tp_t agc_vec_dot_tp(agc_interp_t *ip, agc_vec_t u, agc_vec_t v);
agc_tp_t agc_vec_sq_tp(agc_interp_t *ip, agc_vec_t v);

#endif /* AGC_INTERP_H */
