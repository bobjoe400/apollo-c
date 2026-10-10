/*
 * agc_interp_scalar.c -- scalar interpretive arithmetic
 * (INTERPRETER.agc, Comanche055 pp.1131-1142, 1149, 1152-1161, 1180, 1192;
 * Luminary099 pp.1026-1037, 1044, 1047-1056, 1075, 1087).
 *
 * Each function loads its agc_dp_t / agc_tp_t operands into AGC words
 * (agc_w_split), runs the AGC routine on the words (agc_mpac.c, agc_shift.c,
 * agc_divide.c) and returns the value of the resulting MPAC words.
 * OVFIND is set exactly where the AGC sets it and is never cleared here;
 * BOV (agc_interp_test_ovf) clears it.
 */
#include "../mpac.h"
#include "agc/core/assert.h"

#define POLY_MAX_DEGREE 15

/* ---- interpreter state --------------------------------------------------- */

void agc_interp_init(agc_interp_t *ip)
{
    int k;
    if (!AGC_ASSERT(ip != 0)) {
        return;
    }
    ip->ovfind = 0;
    ip->x1 = 0;
    ip->x2 = 0;
    ip->s1 = 0;
    ip->s2 = 0;
    for (k = 0; k < AGC_INTERP_SWITCHES; k++) {
        ip->sw[k] = false;
    }
    (void)AGC_ASSERT(ip->ovfind == 0);
}

/* BOV / BOVB (p.1196 "BOV(B)"): CCS OVFIND; branch and clear if set.       */
bool agc_interp_test_ovf(agc_interp_t *ip)
{
    bool set;
    if (!AGC_ASSERT(ip != 0)) {
        return false;
    }
    (void)AGC_ASSERT((ip->ovfind >= -1) && (ip->ovfind <= 1));
    set = (ip->ovfind != 0);
    ip->ovfind = 0;
    return set;
}

/* ---- add / subtract (p.1131-1135) ---------------------------------------- */

/* DAD: DAS MPAC; OVERFLOW if A non-zero (ENDVXV). Wraps on overflow.       */
agc_dp_t agc_dp_add(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    if (!AGC_ASSERT(ip != 0)) {
        return a;
    }
    (void)AGC_ASSERT((b >= AGC_DP_MIN) && (b <= AGC_DP_MAX));
    return agc_w_join(agc_w_dad(agc_w_split(a), agc_w_split(b), &ip->ovfind));
}

/* DSU: DCS 0 then ENDVXV. BDSU X is agc_dp_sub(ip, X, mpac).                */
agc_dp_t agc_dp_sub(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    if (!AGC_ASSERT(ip != 0)) {
        return a;
    }
    (void)AGC_ASSERT((b >= AGC_DP_MIN) && (b <= AGC_DP_MAX));
    return agc_w_join(agc_w_dad(agc_w_split(a), agc_w_split(-b), &ip->ovfind));
}

/* TAD (p.1135): minor parts by DAS, then AD AD TS on MPAC; SETOVF ->
 * OVERFLOW on MPAC,+1 when the major word overflows.                       */
agc_tp_t agc_tp_add(agc_interp_t *ip, agc_tp_t a, agc_tp_t b)
{
    agc_w3_t m = agc_w3_from_tp(a);
    agc_w3_t x = agc_w3_from_tp(b);
    agc_w2_t lo2 = { m.m1, m.m2 };
    agc_w2_t add = { x.m1, x.m2 };
    agc_w2_t top;
    int o = 0;
    if (!AGC_ASSERT(ip != 0)) {
        return a;
    }
    lo2 = agc_w_das(lo2, add, &o);                     /* DCA 1; DAS +1    */
    m.m1 = lo2.hi;
    m.m2 = lo2.lo;
    m.m0 = agc_w_ts(o + x.m0 + m.m0, &o);              /* AD 0; AD; TS     */
    if (o != 0) {
        top = agc_w_overflow(agc_w3_top(m), o, &ip->ovfind);  /* SETOVF   */
        m.m0 = top.hi;
        m.m1 = top.lo;
    }
    (void)AGC_ASSERT(agc_w_ok(m.m0));
    return agc_w3_value(m);
}

/* ---- multiply (p.1136-1137, 1141-1142, 1149, 1183) ----------------------- */

/* DMPR (p.1149): DMPSUB; ROUNDSUB (OVFIND on the rare carry out).          */
agc_dp_t agc_dp_mul(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    agc_w3_t m;
    if (!AGC_ASSERT(ip != 0)) {
        return 0;
    }
    m = agc_w_dmpsub(agc_w_split(a), agc_w_split(b));
    m = agc_w_roundsub(m, &ip->ovfind);
    (void)AGC_ASSERT(m.m2 == 0);
    return agc_w_join(agc_w3_top(m));
}

/* DMP (DMP1 p.1149) used as a DP: MPAC,+1 of the DMPSUB product.           */
agc_dp_t agc_dp_mul_trunc(agc_dp_t a, agc_dp_t b)
{
    agc_w3_t m = agc_w_dmpsub(agc_w_split(a), agc_w_split(b));
    (void)AGC_ASSERT(agc_w_ok(m.m0));
    (void)AGC_ASSERT(agc_w_ok(m.m1));
    return agc_w_join(agc_w3_top(m));
}

/* DSQ (p.1180): DSQSUB leaves the TP square; as a DP, MPAC,+1.             */
agc_dp_t agc_dp_sq(agc_interp_t *ip, agc_dp_t a)
{
    (void)AGC_ASSERT(ip != 0);
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    return agc_w_join(agc_w3_top(agc_w_dsqsub(agc_w_split(a))));
}

agc_tp_t agc_dp_sq_tp(agc_dp_t a)
{
    agc_w3_t m = agc_w_dsqsub(agc_w_split(a));
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    (void)AGC_ASSERT(m.m0 >= 0);
    return agc_w3_value(m);
}

/* SHORTMP / SHORTMP2 / DMPSUB2 (p.1141, 1136): TP x SP -> TP.              */
agc_tp_t agc_tp_mul_sp(agc_tp_t v, agc_sp_t k)
{
    agc_w3_t m = agc_w3_from_tp(v);
    agc_w2_t acc;
    int o = 0;
    (void)AGC_ASSERT((k >= AGC_SP_MIN) && (k <= AGC_SP_MAX));
    acc.hi = 0;                                        /* XCH MPAC +1      */
    acc.lo = agc_w_mp(k, m.m2).hi;                     /* TS MPAC +2       */
    acc = agc_w_das(acc, agc_w_mp(m.m1, k), &o);       /* DAS MPAC +1      */
    m.m1 = acc.hi;
    m.m2 = acc.lo;
    acc.hi = o;                                        /* XCH MPAC         */
    acc.lo = m.m1;
    acc = agc_w_das(acc, agc_w_mp(m.m0, k), &o);       /* DAS MPAC         */
    m.m0 = acc.hi;
    m.m1 = acc.lo;
    (void)AGC_ASSERT(o == 0);
    return agc_w3_value(m);
}

/* DMPNSUB (p.1142): MPAC = (major(n m1) + minor(n m0), minor(n m1)).        */
agc_dp_t agc_dp_mul_int(agc_dp_t v, agc_sp_t n)
{
    agc_w2_t m = agc_w_split(v);
    agc_w2_t lo = agc_w_mp(n, m.lo);                   /* MP MPAC +1       */
    agc_w2_t hi = agc_w_mp(n, m.hi);                   /* MP DMPNTEMP      */
    int o = 0;
    (void)AGC_ASSERT(hi.hi == 0);                      /* customer's duty  */
    lo.hi = agc_w_ts(lo.hi + hi.lo, &o);               /* CA L; ADS MPAC   */
    (void)AGC_ASSERT(o == 0);
    return agc_w_join(lo);
}

/* POLY / POWRSERS (p.1139-1140). x keeps only MPAC,+1.                      */
agc_dp_t agc_dp_poly(agc_dp_t x, const agc_dp_t *coef, int degree)
{
    if (!AGC_ASSERT(coef != 0)) {
        return 0;
    }
    if (!AGC_ASSERT((degree >= 1) && (degree <= POLY_MAX_DEGREE))) {
        return 0;
    }
    return agc_w_join(agc_w3_top(agc_w_poly(agc_w_split(x), coef, degree)));
}

/* ---- divide (p.1149, 1162-1170) ------------------------------------------ */

/* DDV: MPAC / X. BDDV X is agc_dp_div(ip, X, mpac).                         */
agc_dp_t agc_dp_div(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    int o = 0;
    agc_dp_t q;
    if (!AGC_ASSERT(ip != 0)) {
        return AGC_DP_MAX;
    }
    q = agc_w_ddv(a, b, &o);
    (void)AGC_ASSERT((o == 0) || (q == AGC_DP_MAX) || (q == AGC_DP_MIN));
    if (o != 0) {
        ip->ovfind = 1;                                /* CAF ONE; TS OVFIND */
    }
    return q;
}

/* ---- shifts and rounding (p.1137, 1152-1161) ----------------------------- */

/* SL n (TSSL p.1154). A negative count shifts right (RIGHT- p.1161).        */
agc_dp_t agc_dp_shl(agc_interp_t *ip, agc_dp_t a, int n)
{
    agc_w3_t m = agc_w3_from_tp(agc_tp_from_dp(a));
    if (!AGC_ASSERT(ip != 0)) {
        return a;
    }
    if (n < 0) {
        return agc_dp_shr(a, -n);
    }
    (void)AGC_ASSERT(n <= 127);                        /* 7-bit count      */
    return agc_w_join(agc_w3_top(agc_w_tssl(m, n, &ip->ovfind)));
}

/* SLR n: TSSL then ROUNDSUB. For a DP MPAC (MPAC+2 = 0 after DLOAD) the
 * round adds nothing, so this equals SL; kept separate for fidelity.       */
agc_dp_t agc_dp_shl_round(agc_interp_t *ip, agc_dp_t a, int n)
{
    agc_w3_t m = agc_w3_from_tp(agc_tp_from_dp(a));
    if (!AGC_ASSERT(ip != 0)) {
        return a;
    }
    if (n < 0) {
        return agc_dp_shr_round(a, -n);
    }
    (void)AGC_ASSERT(n <= 127);                        /* 7-bit count      */
    m = agc_w_tssl(m, n, &ip->ovfind);
    m = agc_w_roundsub(m, &ip->ovfind);
    return agc_w_join(agc_w3_top(m));
}

/* SL n / SR -n on the TP MPAC (TSSL, GENSCR/MPACSHR).                       */
agc_tp_t agc_tp_shift(agc_interp_t *ip, agc_tp_t v, int n)
{
    agc_w3_t m = agc_w3_from_tp(v);
    if (!AGC_ASSERT(ip != 0) || (n == 0)) {
        return v;
    }
    if (n > 0) {
        m = agc_w_tssl(m, n, &ip->ovfind);
    } else {
        m = agc_w_genscr(m, -n, false);
    }
    (void)AGC_ASSERT(agc_w_ok(m.m0));
    return agc_w3_value(m);
}

/* NORM (TSLC p.1157) on the TP MPAC.                                       */
agc_tp_t agc_tp_norm(agc_tp_t v, int *nshift)
{
    int count = 0;
    agc_w3_t m = agc_w_tslc(agc_w3_from_tp(v), &count);
    (void)AGC_ASSERT((v >= AGC_TP_MIN) && (v <= AGC_TP_MAX));
    if (AGC_ASSERT(nshift != 0)) {
        *nshift = count;
    }
    return agc_w3_value(m);
}

/* ROUND (unary op 07 -> ROUND: TC ROUNDSUB, p.1154/1137).                   */
agc_dp_t agc_tp_round(agc_interp_t *ip, agc_tp_t a)
{
    agc_w3_t m = agc_w3_from_tp(a);
    if (!AGC_ASSERT(ip != 0)) {
        return agc_tp_round_dp(a);
    }
    m = agc_w_roundsub(m, &ip->ovfind);
    (void)AGC_ASSERT(m.m2 == 0);
    return agc_w_join(agc_w3_top(m));
}
