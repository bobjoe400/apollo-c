/*
 * conic_common.c -- MUTABLE, interpreter-level helpers for the conic
 * subroutines (CONIC_SUBROUTINES.agc, Comanche 055 pp. 1262-1308).
 *
 * The interpretive MPAC is triple precision: DMP leaves a TP product, and a
 * following SL/SR/NORM/TAD/ROUND operates on all three words. The core API
 * has TP multiply/add/round but no TP shift or TP NORM; those two are local
 * here (agc_conic_mshift_, agc_conic_mnorm_) and are listed under
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

/* ---- interpretive MPAC as a TP accumulator (see conic_internal.h) ------- */

/* MPAC+2 of a TP accumulator: what a DP-mode STORE drops. */
static agc_tp_t mpac_low(agc_tp_t m)
{
    agc_tp_t lo = m - agc_tp_from_dp(agc_tp_to_dp(m));
    (void)AGC_ASSERT(lo > -((agc_tp_t)1 << AGC_SP_BITS));
    (void)AGC_ASSERT(lo < ((agc_tp_t)1 << AGC_SP_BITS));
    return lo;
}

/* DLOAD X: MPAC,+1 = X, MPAC+2 = 0 (INTERPRETER.agc p. 1107). */
agc_tp_t agc_conic_mload_(agc_dp_t d)
{
    (void)AGC_ASSERT(d >= AGC_DP_MIN);
    (void)AGC_ASSERT(d <= AGC_DP_MAX);
    return agc_tp_from_dp(d);
}

/* STORE X / PUSH in DP mode: MPAC,+1 only. */
agc_dp_t agc_conic_mstore_(agc_tp_t m)
{
    (void)AGC_ASSERT(m >= AGC_TP_MIN);
    (void)AGC_ASSERT(m <= AGC_TP_MAX);
    return agc_tp_to_dp(m);
}

/* DAD X (p. 1131): DAS into MPAC,+1 with OVERFLOW limiting; MPAC+2 kept. */
agc_tp_t agc_conic_mdad_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    (void)AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX);
    return agc_tp_from_dp(agc_dp_add(ip, agc_tp_to_dp(m), d)) + mpac_low(m);
}

/* DSU X (p. 1133): as DAD with the complement of X. */
agc_tp_t agc_conic_mdsu_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    (void)AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX);
    return agc_tp_from_dp(agc_dp_sub(ip, agc_tp_to_dp(m), d)) + mpac_low(m);
}

/* BDSU X (p. 1134): MPAC,+1 = X - MPAC,+1. MPAC+2 is NOT complemented by
 * the AGC (DCOM acts on A,L only), so a leftover product word keeps its
 * sign; reproduced as is.                                                 */
agc_tp_t agc_conic_mbdsu_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    (void)AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX);
    return agc_tp_from_dp(agc_dp_sub(ip, d, agc_tp_to_dp(m))) + mpac_low(m);
}

/* TAD X (p. 1135): triple precision add. */
agc_tp_t agc_conic_mtad_(agc_interp_t *ip, agc_tp_t m, agc_tp_t t)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    (void)AGC_ASSERT(t >= AGC_TP_MIN && t <= AGC_TP_MAX);
    return agc_tp_add(ip, m, t);
}

/* DMP X (DMPSUB p. 1136): MPAC,+1 times X, TP product in MPAC,+1,+2. */
agc_tp_t agc_conic_mdmp_(agc_tp_t m, agc_dp_t d)
{
    (void)AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX);
    (void)AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX);
    return agc_dp_mul_tp(agc_tp_to_dp(m), d);
}

/* DMPR X (p. 1149): DMP then ROUNDSUB. */
agc_tp_t agc_conic_mdmpr_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    (void)AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX);
    return agc_tp_from_dp(agc_dp_mul(ip, agc_tp_to_dp(m), d));
}

/* DSQ (DSQSUB p. 1180): square of MPAC,+1, TP result. */
agc_tp_t agc_conic_mdsq_(agc_tp_t m)
{
    agc_dp_t d = agc_tp_to_dp(m);
    (void)AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX);
    (void)AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX);
    return agc_dp_mul_tp(d, d);
}

/* DDV X (p. 1162): MPAC,+1 / X; quotient DP, OVFIND and limit on overflow. */
agc_tp_t agc_conic_mddv_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    (void)AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX);
    return agc_tp_from_dp(agc_dp_div(ip, agc_tp_to_dp(m), d));
}

/* BDDV X (p. 1149): X / MPAC,+1. */
agc_tp_t agc_conic_mbddv_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    (void)AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX);
    return agc_tp_from_dp(agc_dp_div(ip, d, agc_tp_to_dp(m)));
}

/* SQRT (p. 1181): square root of MPAC,+1. DEVIATION: SQRTSUB pulls MPAC+2
 * up only when MPAC is zero (argument < 2^-14); the DP root ignores it.  */
agc_tp_t agc_conic_msqrt_(agc_interp_t *ip, agc_tp_t m)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    (void)AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX);
    return agc_tp_from_dp(agc_dp_sqrt(ip, agc_tp_to_dp(m)));
}

/* SL n (n > 0) / SR -n (n < 0) on the TP MPAC (GENSHIFT p. 1158). Left
 * shifts that overflow limit the accumulator and set OVFIND; right shifts
 * truncate the magnitude, as the ones' complement AGC does.  This is the
 * interim agc_tp_shift requested from core-numeric.                      */
agc_tp_t agc_conic_mshift_(agc_interp_t *ip, agc_tp_t m, int n)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    if (!AGC_ASSERT(n > -128 && n < 128)) { return m; }
    agc_tp_t r = m;
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

/* SLR n / SRR -n: shift, then ROUNDSUB on the result (p. 1160-1161). */
agc_tp_t agc_conic_mshiftr_(agc_interp_t *ip, agc_tp_t m, int n)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    if (!AGC_ASSERT(n > -128 && n < 128)) { return m; }
    return agc_conic_mround_(ip, agc_conic_mshift_(ip, m, n));
}

/* ROUND (ROUNDSUB p. 1137): round MPAC+2 into MPAC,+1 and clear it. */
agc_tp_t agc_conic_mround_(agc_interp_t *ip, agc_tp_t m)
{
    if (!AGC_ASSERT(ip != NULL)) { return m; }
    (void)AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX);
    return agc_tp_from_dp(agc_tp_round_dp(m));
}

/* NORM X (TSLC p. 1157): left-justify the TP MPAC until |MPAC| >= 1/2;
 * *nshift = number of left shifts (the AGC stores -*nshift in X). Zero
 * is left alone with a count of 0. Interim agc_tp_norm.                  */
agc_tp_t agc_conic_mnorm_(agc_tp_t m, int *nshift)
{
    if (!AGC_ASSERT(nshift != NULL)) { return m; }
    if (!AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX)) { *nshift = 0; return m; }
    const agc_tp_t half = (agc_tp_t)1 << (AGC_TP_BITS - 1);
    agc_tp_t r = m;
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

/* ABS (p. 1183). */
agc_tp_t agc_conic_mabs_(agc_tp_t m)
{
    (void)AGC_ASSERT(m >= AGC_TP_MIN);
    (void)AGC_ASSERT(m <= AGC_TP_MAX);
    return (m < 0) ? -m : m;
}

/* DCOMP (p. 1151): complement all three words. */
agc_tp_t agc_conic_mneg_(agc_tp_t m)
{
    (void)AGC_ASSERT(m >= AGC_TP_MIN);
    (void)AGC_ASSERT(m <= AGC_TP_MAX);
    return -m;
}

/* SIGN X (p. 1151) with a DP word at X: complement if X is negative. */
agc_tp_t agc_conic_msign_(agc_tp_t m, agc_dp_t s)
{
    (void)AGC_ASSERT(s >= AGC_DP_MIN && s <= AGC_DP_MAX);
    (void)AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX);
    return (s < 0) ? -m : m;
}

/* SIGN X with an SP tag at X (GEOMSGN, SGNRDOT +-.5): the AGC tests the
 * tag word (non-zero), so the following word never matters.             */
agc_tp_t agc_conic_msign_sp_(agc_tp_t m, agc_sp_t s)
{
    (void)AGC_ASSERT(s != 0);
    (void)AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX);
    return (s < 0) ? -m : m;
}

/* BMN: branch if MPAC,+1 is negative (non-zero). */
bool agc_conic_mbmn_(agc_tp_t m)
{
    (void)AGC_ASSERT(m >= AGC_TP_MIN);
    (void)AGC_ASSERT(m <= AGC_TP_MAX);
    return agc_tp_to_dp(m) < 0;
}

/* BZE: branch if MPAC,+1 is zero. */
bool agc_conic_mbze_(agc_tp_t m)
{
    (void)AGC_ASSERT(m >= AGC_TP_MIN);
    (void)AGC_ASSERT(m <= AGC_TP_MAX);
    return agc_tp_to_dp(m) == 0;
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
        acc = agc_dp_add_raw(agc_tp_to_dp(agc_dp_mul_tp(acc, x)), coef[i], &ovf);
        (void)ovf;   /* DAS overflow not observed by POLY (see above) */
    }
    return acc;
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
