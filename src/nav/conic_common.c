/*
 * conic_common.c -- MUTABLE, interpreter-level helpers for the conic
 * subroutines (CONIC_SUBROUTINES.agc, Comanche 055 pp. 1262-1308).
 *
 * The interpretive MPAC is triple precision: DMP leaves a TP product, and a
 * following SL/SR/NORM/TAD/ROUND operates on all three words. The core API
 * has TP multiply/add/round but no TP shift or TP NORM; those two are local
 * here (agc_conic_tp_shift_, agc_conic_tp_norm_) and are listed under
 * "Requested core additions" in docs/modules/nav-conic.md.
 */
#include "conic_internal.h"
#include "agc/agc_assert.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"
#include "agc/agc_flags_common.h"

/* MUTABLE (CONIC_SUBROUTINES.agc p. 1305; Luminary: CONTROLLED_CONSTANTS).
 * Indexed by -X1: earth X1 = -2 uses words 2..6, moon X1 = -10D words 10..14. */
static const agc_dp_t mutable_tab[8] = {
    155704375,  /* MUE        22437 16067  3.986032 E10 B-36  m^3/cs^2      */
    115696162,  /* 1/MUE      15625 21042  .25087606 E-10 B+34              */
    204442107,  /* SQRT(MUE)  30276 04773  1.99650495 E5 B-18               */
    176229826,  /* 1/SQRT(MUE) 25004 06702 .50087529 E-5 B+17               */
    122569450,  /* MUM        16471 01352  4.902778 E8 B-30                 */
    146972992,  /* 1/MUM      21412 20500  .203966 E-8 B+28                 */
    181389047,  /* SQRT(MUM)  25477 03367  2.21422176 E4 B-15               */
    198627193   /* 1/SQRT(MUM) 27533 07571 .45162595 E-4 B+14               */
};

/* VLOAD* MUTABLE,1 / STORE 1/MU: load 1/MU, ROOTMU, 1/ROOTMU into 14D-19D. */
agc_status_t agc_conic_ctx_init_(agc_conic_ctx_t *ctx, agc_conic_body_t body)
{
    if (!AGC_ASSERT(ctx != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(body == AGC_CONIC_EARTH || body == AGC_CONIC_MOON)) {
        return AGC_STATUS_BAD_ARG;
    }
    int base = (body == AGC_CONIC_EARTH) ? 1 : 5;   /* MUTABLE+2 / +10 words */
    agc_conic_ctx_t z = { 0 };
    *ctx = z;
    ctx->body        = body;
    ctx->inv_mu      = mutable_tab[base];
    ctx->root_mu     = mutable_tab[base + 1];
    ctx->inv_root_mu = mutable_tab[base + 2];
    ctx->vvec  = agc_vec_zero();
    ctx->ur1   = agc_vec_zero();
    ctx->u2    = agc_vec_zero();
    ctx->un    = agc_vec_zero();
    return AGC_STATUS_OK;
}

/* MUEARTH,2 / MUM (ORBITAL_INTEGRATION.agc p. 1354) as used by KEPPREP via
 * PBODY. Same assembled octal as the MUTABLE entries.                     */
agc_dp_t agc_conic_mu_(agc_conic_body_t body)
{
    if (!AGC_ASSERT(body == AGC_CONIC_EARTH || body == AGC_CONIC_MOON)) {
        return mutable_tab[0];
    }
    return (body == AGC_CONIC_EARTH) ? mutable_tab[0] : mutable_tab[4];
}

/* SL n (n > 0, sets OVFIND) / SR -n (truncating) on a DP MPAC. */
agc_dp_t agc_conic_shift_(agc_interp_t *ip, agc_dp_t a, int n)
{
    if (!AGC_ASSERT(ip != NULL)) { return a; }
    if (!AGC_ASSERT(n > -64 && n < 64)) { return a; }
    if (n >= 0) {
        return agc_dp_shl(ip, a, n);
    }
    return agc_dp_shr(a, -n);
}

/* SLR n / SRR -n on a DP MPAC. */
agc_dp_t agc_conic_shift_round_(agc_interp_t *ip, agc_dp_t a, int n)
{
    if (!AGC_ASSERT(ip != NULL)) { return a; }
    if (!AGC_ASSERT(n > -64 && n < 64)) { return a; }
    if (n >= 0) {
        return agc_dp_shl_round(ip, a, n);
    }
    return agc_dp_shr_round(a, -n);
}

/* DMP whose TP product is then used (stored, pushed, or fed to a DP op) as
 * DP: MPAC+2 is dropped, i.e. truncation.                                 */
agc_dp_t agc_conic_dmp_(agc_dp_t a, agc_dp_t b)
{
    (void)AGC_ASSERT(a >= AGC_DP_MIN && a <= AGC_DP_MAX);
    (void)AGC_ASSERT(b >= AGC_DP_MIN && b <= AGC_DP_MAX);
    return agc_tp_to_dp(agc_dp_mul_tp(a, b));
}

/* DMP followed by SL n / SR -n (round: SLR/SRR) on the TP product. */
agc_dp_t agc_conic_dmp_shift_(agc_interp_t *ip, agc_dp_t a, agc_dp_t b,
                              int n, bool round)
{
    if (!AGC_ASSERT(ip != NULL)) { return 0; }
    if (!AGC_ASSERT(n > -64 && n < 64)) { return 0; }
    agc_tp_t tp = agc_conic_tp_shift_(ip, agc_dp_mul_tp(a, b), n);
    return round ? agc_tp_round_dp(tp) : agc_tp_to_dp(tp);
}

/* Interim TP shift (requested core addition agc_tp_shift). Left shifts
 * saturate at TP max and set OVFIND; right shifts truncate the magnitude,
 * as the ones' complement AGC does.                                       */
agc_tp_t agc_conic_tp_shift_(agc_interp_t *ip, agc_tp_t v, int n)
{
    if (!AGC_ASSERT(ip != NULL)) { return v; }
    if (!AGC_ASSERT(n > -128 && n < 128)) { return v; }
    agc_tp_t r = v;
    if (n < 0) {
        int k = -n;
        return (k >= 62) ? 0 : (r / ((agc_tp_t)1 << k));
    }
    for (int i = 0; i < n; i++) {
        if (r > AGC_TP_MAX / 2 || r < AGC_TP_MIN / 2) {
            ip->ovfind = (r > 0) ? (agc_ovf_t)1 : (agc_ovf_t)-1;
            return (r > 0) ? AGC_TP_MAX : AGC_TP_MIN;
        }
        r *= 2;
    }
    return r;
}

/* Interim TP NORM (requested core addition agc_tp_norm): left-justify until
 * |v| >= 1/2; *nshift = number of left shifts (the AGC stores -*nshift).  */
agc_tp_t agc_conic_tp_norm_(agc_tp_t v, int *nshift)
{
    if (!AGC_ASSERT(nshift != NULL)) { return v; }
    if (!AGC_ASSERT(v >= AGC_TP_MIN && v <= AGC_TP_MAX)) { *nshift = 0; return v; }
    const agc_tp_t half = (agc_tp_t)1 << (AGC_TP_BITS - 1);
    agc_tp_t r = v;
    int n = 0;
    for (int i = 0; i < AGC_TP_BITS && r != 0; i++) {
        if (r >= half || r <= -half) {
            break;
        }
        r *= 2;
        n++;
    }
    *nshift = n;
    return r;
}

/* NORM applied to a DP MPAC (MPAC+2 empty). */
agc_dp_t agc_conic_norm_(agc_dp_t a, int *nshift)
{
    if (!AGC_ASSERT(nshift != NULL)) { return a; }
    (void)AGC_ASSERT(a >= AGC_DP_MIN && a <= AGC_DP_MAX);
    return agc_tp_to_dp(agc_conic_tp_norm_(agc_tp_from_dp(a), nshift));
}

/* DSQ NORM: square to TP, then normalize the TP (keeps MPAC+2 bits). */
agc_dp_t agc_conic_sq_norm_(agc_dp_t a, int *nshift)
{
    if (!AGC_ASSERT(nshift != NULL)) { return a; }
    (void)AGC_ASSERT(a >= AGC_DP_MIN && a <= AGC_DP_MAX);
    return agc_tp_to_dp(agc_conic_tp_norm_(agc_dp_mul_tp(a, a), nshift));
}

/*
 * POLY (INTERPRETER.agc p. 1139, basic code called through EXIT/TC POLY):
 * Horner evaluation of coef[0] + coef[1] x + ... from the top coefficient,
 * DMPSUB (MPAC+2 dropped on the next pass) then DAS of the next coefficient.
 * "User's responsibility to assure no overflow": the DAS overflow is not
 * reported (OVFIND is untouched by basic DAS).
 */
agc_dp_t agc_conic_poly_(agc_dp_t x, const agc_dp_t *coef, int ncoef)
{
    if (!AGC_ASSERT(coef != NULL)) { return 0; }
    if (!AGC_ASSERT(ncoef >= 1 && ncoef <= 10)) { return 0; }
    agc_dp_t acc = coef[ncoef - 1];
    for (int i = ncoef - 2; i >= 0; i--) {
        agc_ovf_t ovf = 0;
        acc = agc_dp_add_raw(agc_conic_dmp_(acc, x), coef[i], &ovf);
        (void)ovf;   /* DAS overflow not observed by POLY (see above) */
    }
    return acc;
}

/* SIGN X with an SP word at X (GEOMSGN, SGNRDOT): complement if negative. */
agc_dp_t agc_conic_sign_sp_(agc_dp_t a, agc_sp_t s)
{
    (void)AGC_ASSERT(s != 0);   /* +-.5 tags; +0 would test the next word */
    (void)AGC_ASSERT(a >= AGC_DP_MIN && a <= AGC_DP_MAX);
    return agc_dp_sign(a, agc_dp_from_sp(s));
}

/* SIGN with vector MPAC: complement the vector if the SP tag is negative. */
agc_vec_t agc_conic_vsign_sp_(agc_vec_t v, agc_sp_t s)
{
    (void)AGC_ASSERT(s != 0);
    (void)AGC_ASSERT(v.c[0] >= AGC_DP_MIN && v.c[0] <= AGC_DP_MAX);
    return (s < 0) ? agc_vec_neg(v) : v;
}

/* Mirror an output switch into its flagword (the AGC keeps it only there). */
agc_status_t agc_conic_mirror_flag_(agc_conic_flag_t f, bool set)
{
    agc_flag_id_t id;
    switch (f) {
    case AGC_CONIC_FLAG_SOLNSW:   id = AGC_FLAG_SOLNSW;   break;
    case AGC_CONIC_FLAG_INFINFLG: id = AGC_FLAG_INFINFLG; break;
    case AGC_CONIC_FLAG_COGAFLAG: id = AGC_FLAG_COGAFLAG; break;
    case AGC_CONIC_FLAG_APSESW:   id = AGC_FLAG_APSESW;   break;
    default:
        (void)AGC_ASSERT(false);
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(id != 0)) { return AGC_STATUS_BAD_ARG; }
    return set ? agc_flag_set(id) : agc_flag_clear(id);
}

/* ABTCONIC (p. 1304): EXIT, TC POODOO, OCT 00607. */
agc_status_t agc_conic_abtconic_(void)
{
    agc_status_t st = agc_poodoo(AGC_CONIC_ALARM_ABTCONIC);
    (void)AGC_ASSERT(AGC_CONIC_ALARM_ABTCONIC == 0607);
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    return AGC_STATUS_POODOO;
}
