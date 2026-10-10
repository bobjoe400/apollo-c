/*
 * agc_mpac.c -- word-level model of MPAC arithmetic (INTERPRETER.agc,
 * "ARITHMETIC SUBROUTINES REQUIRED IN FIXED-FIXED", pp. 1131-1138, 1183).
 *
 * Every function here mirrors one AGC instruction or one interpreter
 * subroutine instruction by instruction, on 15-bit ones'-complement words
 * held in int32_t. See agc_mpac.h for the representation.
 */
#include "mpac.h"
#include "agc/core/assert.h"

/* ---- representation ----------------------------------------------------- */

bool agc_w_ok(int32_t w)
{
    /* No AGC_ASSERT here: this is the predicate the assertions use.        */
    return (w > -AGC_W_ONE) && (w < AGC_W_ONE);
}

/* Split a DP value into the sign-agreeing word pair the AGC would hold.     */
agc_w2_t agc_w_split(agc_dp_t v)
{
    agc_w2_t p = { 0, 0 };
    if (!AGC_ASSERT((v >= AGC_DP_MIN) && (v <= AGC_DP_MAX))) {
        v = (v < 0) ? AGC_DP_MIN : AGC_DP_MAX;
    }
    p.hi = v / AGC_W_ONE;            /* C division truncates toward zero */
    p.lo = v % AGC_W_ONE;            /* so lo carries the sign of v      */
    (void)AGC_ASSERT(agc_w_ok(p.hi) && agc_w_ok(p.lo));
    return p;
}

agc_dp_t agc_w_join(agc_w2_t p)
{
    (void)AGC_ASSERT(agc_w_ok(p.hi));
    (void)AGC_ASSERT(agc_w_ok(p.lo));
    return (agc_dp_t)(p.hi * AGC_W_ONE + p.lo);
}

agc_w3_t agc_w3_from_tp(agc_tp_t v)
{
    agc_w3_t m = { 0, 0, 0 };
    if (!AGC_ASSERT((v >= AGC_TP_MIN) && (v <= AGC_TP_MAX))) {
        v = (v < 0) ? AGC_TP_MIN : AGC_TP_MAX;
    }
    m.m0 = (int32_t)(v / ((agc_tp_t)AGC_W_ONE * AGC_W_ONE));
    m.m1 = (int32_t)((v / AGC_W_ONE) % AGC_W_ONE);
    m.m2 = (int32_t)(v % AGC_W_ONE);
    (void)AGC_ASSERT(agc_w_ok(m.m0) && agc_w_ok(m.m1) && agc_w_ok(m.m2));
    return m;
}

agc_tp_t agc_w3_value(agc_w3_t m)
{
    (void)AGC_ASSERT(agc_w_ok(m.m0) && agc_w_ok(m.m1));
    (void)AGC_ASSERT(agc_w_ok(m.m2));
    return ((agc_tp_t)m.m0 * AGC_W_ONE + m.m1) * AGC_W_ONE + m.m2;
}

agc_w2_t agc_w3_top(agc_w3_t m)
{
    agc_w2_t p;
    (void)AGC_ASSERT(agc_w_ok(m.m0));
    (void)AGC_ASSERT(agc_w_ok(m.m1));
    p.hi = m.m0;
    p.lo = m.m1;
    return p;
}

/* ---- single instructions ------------------------------------------------ */

/* MP: SP x SP -> DP in A,L; both words carry the sign of the product.      */
agc_w2_t agc_w_mp(int32_t a, int32_t b)
{
    agc_w2_t r = { 0, 0 };
    if (!AGC_ASSERT(agc_w_ok(a)) || !AGC_ASSERT(agc_w_ok(b))) {
        return r;
    }
    r.hi = (a * b) / AGC_W_ONE;
    r.lo = (a * b) % AGC_W_ONE;
    (void)AGC_ASSERT(agc_w_ok(r.hi));
    (void)AGC_ASSERT(agc_w_ok(r.lo));
    return r;
}

/* TS (or any store to erasable): overflow-correct a 16-bit accumulator
 * value. *ovf receives the overflow indication the AGC leaves in A.       */
int32_t agc_w_ts(int32_t a16, int *ovf)
{
    int o = 0;
    (void)AGC_ASSERT((a16 > -2 * AGC_W_ONE) && (a16 < 2 * AGC_W_ONE));
    if (a16 >= AGC_W_ONE) {
        a16 -= AGC_W_ONE;
        o = 1;
    } else if (a16 <= -AGC_W_ONE) {
        a16 += AGC_W_ONE;
        o = -1;
    }
    if (AGC_ASSERT(ovf != 0)) {
        *ovf = o;
    }
    return a16;
}

/* DAS: (x.hi,x.lo) + (y.hi,y.lo); the minor sum carries into the major
 * word, the major word is overflow-corrected and *ovf gets the net
 * overflow (+1/0/-1), exactly what DAS leaves in A. y.hi may be an
 * uncorrected 16-bit accumulator (as after DDOUBL).                       */
agc_w2_t agc_w_das(agc_w2_t x, agc_w2_t y, int *ovf)
{
    agc_w2_t r;
    int carry = 0;
    (void)AGC_ASSERT(agc_w_ok(x.lo) && agc_w_ok(y.lo));
    r.lo = agc_w_ts(x.lo + y.lo, &carry);
    r.hi = agc_w_ts(x.hi + y.hi + carry, ovf);
    (void)AGC_ASSERT(agc_w_ok(r.hi));
    return r;
}

/* DV: divide the DP accumulator (A,L) by k. The AGC first forces sign
 * agreement in A,L, so only the value matters; quotient and remainder
 * follow C's truncating division. Returns {quotient, remainder}.         */
agc_w2_t agc_w_dv(int32_t a, int32_t l, int32_t k)
{
    agc_w2_t r = { AGC_W_POSMAX, 0 };
    int32_t dividend = a * AGC_W_ONE + l;
    if (!AGC_ASSERT(k != 0) || !AGC_ASSERT(agc_w_ok(l))) {
        return r;
    }
    r.hi = dividend / k;
    r.lo = dividend % k;
    (void)AGC_ASSERT(agc_w_ok(r.hi));
    return r;
}

agc_w2_t agc_w_dcs(agc_w2_t p)
{
    agc_w2_t r;
    (void)AGC_ASSERT(agc_w_ok(p.hi));
    (void)AGC_ASSERT(agc_w_ok(p.lo));
    r.hi = -p.hi;
    r.lo = -p.lo;
    return r;
}

/* ---- interpreter subroutines -------------------------------------------- */

/* OVERFLOW (p.1133): entered after a DAS left A = +-1. Adds +-1.0 to the
 * pair as POSMAX in the major word and POSMAX+1 in the minor word, which
 * forces sign agreement; OVFIND is set only if the pair still overflows.
 * The value is left overflow-corrected (i.e. wrapped modulo 1.0).          */
agc_w2_t agc_w_overflow(agc_w2_t m, int a, agc_ovf_t *ovfind)
{
    int32_t buf;
    int o = 0;
    if (!AGC_ASSERT((a == 1) || (a == -1)) || !AGC_ASSERT(ovfind != 0)) {
        return m;
    }
    buf = (a > 0) ? AGC_W_POSMAX : -AGC_W_POSMAX;      /* CS LIMITS      */
    m.lo = agc_w_ts(m.lo + buf + a, &o);                /* AUG A, ADS     */
    m.hi = agc_w_ts(m.hi + buf + o, &o);                /* AD BUF, ADS    */
    if (o != 0) {
        *ovfind = (agc_ovf_t)o;                         /* SETOVF2        */
    }
    (void)AGC_ASSERT(agc_w_ok(m.hi));
    return m;
}

/* DAD tail (ENDVXV): DAS MPAC, then OVERFLOW if A is non-zero.            */
agc_w2_t agc_w_dad(agc_w2_t m, agc_w2_t x, agc_ovf_t *ovfind)
{
    int o = 0;
    agc_w2_t r = agc_w_das(m, x, &o);
    (void)AGC_ASSERT(ovfind != 0);
    if (o != 0) {
        r = agc_w_overflow(r, o, ovfind);
    }
    (void)AGC_ASSERT(agc_w_ok(r.hi));
    return r;
}

/* DMPSUB (p.1136): MPAC x C(ADDRWD) -> TP. The minor x minor partial
 * product keeps only its major half; everything else is exact.            */
agc_w3_t agc_w_dmpsub(agc_w2_t mpac, agc_w2_t x)
{
    agc_w3_t m;
    agc_w2_t acc;
    int o = 0;
    (void)AGC_ASSERT(agc_w_ok(mpac.hi) && agc_w_ok(mpac.lo));
    (void)AGC_ASSERT(agc_w_ok(x.hi) && agc_w_ok(x.lo));
    acc.hi = 0;                                         /* MPAC+1 := 0    */
    acc.lo = agc_w_mp(mpac.lo, x.lo).hi;                /* MPAC+2         */
    acc = agc_w_das(acc, agc_w_mp(x.lo, mpac.hi), &o);  /* no overflow    */
    acc = agc_w_das(acc, agc_w_mp(mpac.lo, x.hi), &o);  /* A = net ovf    */
    m.m1 = acc.hi;
    m.m2 = acc.lo;
    acc.hi = o;                                         /* XCH MPAC       */
    acc.lo = m.m1;
    acc = agc_w_das(acc, agc_w_mp(mpac.hi, x.hi), &o);  /* no overflow    */
    m.m0 = acc.hi;
    m.m1 = acc.lo;
    return m;
}

/* ROUNDSUB / VROUND (p.1137): round MPAC+2 into MPAC,+1 (half away from
 * zero on the sign of MPAC+2), zero MPAC+2, set OVFIND on overflow.        */
agc_w3_t agc_w_roundsub(agc_w3_t m, agc_ovf_t *ovfind)
{
    int o = 0;
    (void)AGC_ASSERT(agc_w_ok(m.m2));
    (void)agc_w_ts(2 * m.m2, &o);                       /* DOUBLE; TS L   */
    m.m2 = 0;
    if (o == 0) {
        return m;
    }
    m.m1 = agc_w_ts(m.m1 + o, &o);
    if (o != 0) {
        m.m0 = agc_w_ts(m.m0 + o, &o);
        if ((o != 0) && AGC_ASSERT(ovfind != 0)) {
            *ovfind = (agc_ovf_t)o;                     /* SETOVF2        */
        }
    }
    return m;
}

/* DSQSUB (p.1183): square of the DP MPAC as a TP.                          */
agc_w3_t agc_w_dsqsub(agc_w2_t p)
{
    agc_w3_t m;
    agc_w2_t acc;
    agc_w2_t cross;
    int o = 0;
    (void)AGC_ASSERT(agc_w_ok(p.hi) && agc_w_ok(p.lo));
    acc.hi = 0;
    acc.lo = agc_w_mp(p.lo, p.lo).hi;                   /* MPAC+2         */
    cross = agc_w_mp(p.lo, p.hi);
    cross = agc_w_das(cross, cross, &o);                /* DDOUBL         */
    cross.hi += o * AGC_W_ONE;                          /* A keeps ovf    */
    acc = agc_w_das(acc, cross, &o);
    m.m1 = acc.hi;
    m.m2 = acc.lo;
    acc.hi = o;
    acc.lo = m.m1;
    acc = agc_w_das(acc, agc_w_mp(p.hi, p.hi), &o);     /* SQUARE; DAS    */
    m.m0 = acc.hi;
    m.m1 = acc.lo;
    (void)AGC_ASSERT(o == 0);
    return m;
}

/* Accumulate a second TP partial result into BUF (DOTSUB p.1138 middle):
 * DAS on the minor pair, then AD/AD/TS on the major word; TS OVFIND on
 * overflow (the major word is left overflow-corrected).                   */
agc_w3_t agc_w_dot_acc(agc_w3_t buf, agc_w3_t t, agc_ovf_t *ovfind)
{
    agc_w2_t lo2 = { buf.m1, buf.m2 };
    agc_w2_t add = { t.m1, t.m2 };
    int o = 0;
    (void)AGC_ASSERT(agc_w_ok(t.m0));
    lo2 = agc_w_das(lo2, add, &o);
    buf.m1 = lo2.hi;
    buf.m2 = lo2.lo;
    buf.m0 = agc_w_ts(o + t.m0 + buf.m0, &o);
    if ((o != 0) && AGC_ASSERT(ovfind != 0)) {
        *ovfind = (agc_ovf_t)o;
    }
    return buf;
}

/* ENDDOT (p.1138): final accumulation; overflow goes through OVERFLOW.    */
static agc_w3_t end_dot(agc_w3_t buf, agc_w3_t t, agc_ovf_t *ovfind)
{
    agc_w2_t lo2 = { t.m1, t.m2 };
    agc_w2_t add = { buf.m1, buf.m2 };
    agc_w2_t top;
    int o = 0;
    (void)AGC_ASSERT(ovfind != 0);
    lo2 = agc_w_das(lo2, add, &o);
    t.m1 = lo2.hi;
    t.m2 = lo2.lo;
    t.m0 = agc_w_ts(o + t.m0 + buf.m0, &o);
    if (o != 0) {
        top = agc_w_overflow(agc_w3_top(t), o, ovfind);
        t.m0 = top.hi;
        t.m1 = top.lo;
    }
    (void)AGC_ASSERT(agc_w_ok(t.m0));
    return t;
}

/* DOTSUB (p.1138): u . v as a TP, u in MPAC and v at ADDRWD.              */
agc_w3_t agc_w_dotsub(agc_vec_t u, agc_vec_t v, agc_ovf_t *ovfind)
{
    agc_w3_t buf;
    agc_w3_t t;
    (void)AGC_ASSERT(ovfind != 0);
    buf = agc_w_dmpsub(agc_w_split(u.c[0]), agc_w_split(v.c[0]));
    t = agc_w_dmpsub(agc_w_split(u.c[1]), agc_w_split(v.c[1]));
    buf = agc_w_dot_acc(buf, t, ovfind);
    t = agc_w_dmpsub(agc_w_split(u.c[2]), agc_w_split(v.c[2]));
    t = end_dot(buf, t, ovfind);
    (void)AGC_ASSERT(agc_w_ok(t.m0));
    return t;
}

/* VSQSUB (p.1183): v . v using DSQSUB for each component.                 */
agc_w3_t agc_w_vsqsub(agc_vec_t v, agc_ovf_t *ovfind)
{
    agc_w3_t buf;
    agc_w3_t t;
    (void)AGC_ASSERT(ovfind != 0);
    buf = agc_w_dsqsub(agc_w_split(v.c[0]));
    t = agc_w_dsqsub(agc_w_split(v.c[1]));
    buf = agc_w_dot_acc(buf, t, ovfind);
    t = agc_w_dsqsub(agc_w_split(v.c[2]));
    t = end_dot(buf, t, ovfind);
    (void)AGC_ASSERT(agc_w_ok(t.m0));
    return t;
}

/* One TP shift left (TSSL p.1154, SQRTNORM p.1186, TSLCLOOP p.1157):
 * DCA MPAC+1; DAS MPAC+1; AD MPAC; AD MPAC; TS MPAC.                      */
agc_w3_t agc_w_tp_shl1(agc_w3_t m, int *ovf)
{
    agc_w2_t lo2 = { m.m1, m.m2 };
    int carry = 0;
    (void)AGC_ASSERT(agc_w_ok(m.m0));
    lo2 = agc_w_das(lo2, lo2, &carry);
    m.m1 = lo2.hi;
    m.m2 = lo2.lo;
    m.m0 = agc_w_ts(carry + m.m0 + m.m0, ovf);
    (void)AGC_ASSERT(agc_w_ok(m.m0));
    return m;
}

/* POLY (p.1139-1140): Horner evaluation A(N) x^N + ... + A(0), x a DP pair
 * (POLYCOM keeps only MPAC,+1 of the argument), coefficients DP.
 * The DAS of each coefficient ignores overflow ("user's responsibility"). */
agc_w3_t agc_w_poly(agc_w2_t x, const agc_dp_t *coef, int degree)
{
    agc_w3_t m = { 0, 0, 0 };
    agc_w2_t top;
    int o = 0;
    int k;
    if (!AGC_ASSERT(coef != 0) || !AGC_ASSERT((degree >= 1) && (degree <= 15))) {
        return m;
    }
    top = agc_w_split(coef[degree]);
    for (k = degree - 1; k >= 0; k--) {                 /* bound: degree  */
        m = agc_w_dmpsub(top, x);
        top = agc_w_das(agc_w3_top(m), agc_w_split(coef[k]), &o);
        m.m0 = top.hi;
        m.m1 = top.lo;
    }
    (void)AGC_ASSERT(k == -1);
    (void)AGC_ASSERT(agc_w_ok(m.m0) && agc_w_ok(m.m2));
    return m;
}
