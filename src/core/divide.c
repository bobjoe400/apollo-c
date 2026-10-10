/*
 * agc_divide.c -- interpretive division and square root at word level
 * (INTERPRETER.agc, Comanche055 pp.1162-1170 DDV/BDDV/GENDDV/MAXDV and
 * pp.1184-1186 SQRTSUB; Luminary099 pp.1057-1065, 1079-1081).
 *
 * Words are 15-bit ones'-complement values held in int32_t (agc_mpac.h).
 * Places where the AGC computes x + (-x) produce -0 in the real machine;
 * they are commented where the resulting CCS/BZMF branch depends on it.
 */
#include "mpac.h"
#include "agc/agc_assert.h"

/* SQRT linear first-guess constants (INTERPRETER.agc p.1159-1174).          */
#define SQ_SLOPEHI  9640     /* DEC .5884        octal 22650                */
#define SQ_BIASHI   3434     /* DEC .4192 B-1    octal 06552                */
#define SQ_SLOPELO  13638    /* DEC .8324        octal 32506                */
#define SQ_BIASLO   2436     /* DEC .2974 B-1    octal 04604                */
#define SQ_NORM_MAX 16       /* SQRTNORM passes: 13 for any non-zero TP     */
#define DV_NORM_MAX 14       /* BUFNORM passes: divisor major part >= 1     */

/* ---- GENDDV / MAXDV ------------------------------------------------------ */

/* FINALDV (p.1169): minor part of the quotient, (a,0) / c.                  */
static int32_t finaldv(int32_t a, int32_t c)
{
    (void)AGC_ASSERT(c > 0);
    (void)AGC_ASSERT((a > -c) && (a < c));
    return agc_w_dv(a, 0, c).hi;
}

/* GENDDV (p.1167-1168): (A + sB)/(C + sD) = Q + s(R - QD)/C, s = 2^-14.
 * Dividend (A,B) positive with A < C; divisor (C,D) positive, C >= HALF.  */
static agc_dp_t genddv_words(agc_w2_t ab, agc_w2_t cd)
{
    agc_w2_t qr = agc_w_dv(ab.hi, ab.lo, cd.hi);       /* DV BUF          */
    int32_t q = qr.hi;
    int32_t acc = qr.lo + agc_w_mp(-q, cd.lo).hi;      /* CS; MP; AD      */
    (void)AGC_ASSERT(ab.hi < cd.hi);
    if (acc >= AGC_W_ONE) {                            /* OVSK: + overflow */
        acc -= cd.hi;                                  /* SU BUF          */
        q += 1;                                        /* INCR MPAC       */
    } else if (acc <= 0) {                             /* BZMF -UP        */
        if (acc == 0) {
            return (agc_dp_t)(q * AGC_W_ONE);          /* FINALDV +3      */
        }
        q -= 1;                                        /* DIM MPAC        */
        acc += cd.hi;                                  /* ENDMAXDV AD BUF */
        return (agc_dp_t)(q * AGC_W_ONE + finaldv(acc, cd.hi));
    } else {
        /* fall into +DOWN */
    }
    if (acc - cd.hi >= 0) {                            /* +DOWN: SU BUF   */
        acc -= cd.hi;
        q += 1;                                        /* +3: INCR MPAC   */
    }
    (void)AGC_ASSERT(agc_w_ok(q));
    return (agc_dp_t)(q * AGC_W_ONE + finaldv(acc, cd.hi));
}

/* GENDDV entry with the MAXDV test of MAXTEST (p.1166, 1170).               */
agc_dp_t agc_w_genddv(agc_dp_t dividend, agc_dp_t divisor, bool maxdv)
{
    agc_w2_t ab = agc_w_split(dividend);
    agc_w2_t cd = agc_w_split(divisor);
    int32_t acc;
    /* The divisor is normally >= .5; UNIT's SQRTSUB root can be one LSB
     * below .5 (BUF 17777), which the AGC divides by all the same.        */
    if (!AGC_ASSERT(dividend >= 0) || !AGC_ASSERT(cd.hi > AGC_W_QUARTER)) {
        return AGC_DP_MAX;
    }
    (void)AGC_ASSERT(ab.hi <= cd.hi);                  /* A <= C (MAXDV)  */
    if (!maxdv || (ab.hi != cd.hi)) {                  /* MAXDV: BZF +2   */
        return genddv_words(ab, cd);
    }
    /* MAXDV: (C + sB)/(C + sD) = 37777 + s(C + B - D)/C, B < D.             */
    acc = ab.lo - cd.lo + cd.hi;                       /* CS; AD; AD BUF  */
    return (agc_dp_t)(AGC_W_POSMAX * AGC_W_ONE + finaldv(acc, cd.hi));
}

/* ---- DDV / BDDV ---------------------------------------------------------- */

static agc_dp_t ddv_overflow(bool negative, int *ovf)
{
    if (!AGC_ASSERT(ovf != 0)) {
        return AGC_DP_MAX;
    }
    *ovf = 1;                                          /* CAF ONE; TS OVFIND */
    return negative ? AGC_DP_MIN : AGC_DP_MAX;         /* DVOVF: DPPOSMAX */
}

/* Normalise the divisor (BUFNORM/DVNORM) and the dividend by the same count
 * (the DDOUBL prologue indexed by DVNORMCT), then GENDDV/MAXDV.            */
static agc_dp_t ddv_normalised(agc_dp_t a, agc_dp_t b, bool maxdv)
{
    int k;
    (void)AGC_ASSERT((a > 0) && (b > 0));
    for (k = 0; (k < DV_NORM_MAX) && (agc_w_split(b).hi < AGC_W_HALF); k++) {
        b *= 2;                                        /* DCA BUF; DAS BUF */
        a *= 2;                                        /* DDOUBL           */
    }
    (void)AGC_ASSERT(agc_w_split(b).hi >= AGC_W_HALF);
    (void)AGC_ASSERT(a <= b);                          /* no DDOUBL overflow */
    return agc_w_genddv(a, b, maxdv);
}

/* DDV/BDDV (p.1162-1165): a / b with both scaled alike; quotient at 2^0.
 * Overflow (|a| >= |b|, b = 0) gives +-DPPOSMAX and *ovf = 1.             */
agc_dp_t agc_w_ddv(agc_dp_t a, agc_dp_t b, int *ovf)
{
    agc_w2_t w = agc_w_split(a);
    agc_w2_t d = agc_w_split(b);
    bool neg = false;                                  /* DVSIGN = -1      */
    agc_dp_t q;
    if (!AGC_ASSERT(ovf != 0)) {
        return AGC_DP_MAX;
    }
    *ovf = 0;
    if (d.hi == 0) {                                   /* BUFZERO          */
        if (w.hi != 0) {                               /* OVF+ / SGNDVOVF  */
            return ddv_overflow((w.hi < 0) != (d.lo < 0), ovf);
        }
        a *= AGC_W_ONE;                                /* shift both 14    */
        b *= AGC_W_ONE;
        if (b == 0) {                                  /* zero divisor     */
            return ddv_overflow(a < 0, ovf);
        }
    }
    if (b < 0) {                                       /* BUF-             */
        b = -b;
        neg = !neg;
    }
    if (a == 0) {                                      /* exit on zero     */
        return 0;
    }
    if (a < 0) {                                       /* MPAC-            */
        a = -a;
        neg = !neg;
    }
    w = agc_w_split(a);
    d = agc_w_split(b);
    if ((d.hi - w.hi < 0) || ((d.hi == w.hi) && (d.lo - w.lo <= 0))) {
        return ddv_overflow(neg, ovf);                 /* MPAC+: DVOVF     */
    }
    /* Equal majors leave -0 in A; CCS turns it into +0 for TS MAXDVSW.    */
    q = ddv_normalised(a, b, d.hi == w.hi);
    (void)AGC_ASSERT((q >= 0) && (q <= AGC_DP_MAX));
    return neg ? -q : q;
}

/* ---- SQRTSUB ------------------------------------------------------------- */

/* Shift a positive DP word pair right one (DXCH; LXCH SR; MP HALF; ...).   */
static agc_w2_t sqrt_half(agc_w2_t p)
{
    agc_w2_t r;
    (void)AGC_ASSERT(p.hi > 0);
    r.hi = agc_w_mp(p.hi, AGC_W_HALF).hi;
    r.lo = agc_w_mp(p.hi, AGC_W_HALF).lo + p.lo / 2;   /* SR edits: trunc  */
    (void)AGC_ASSERT(agc_w_ok(r.lo));
    return r;
}

/* ARGHI +4 (p.1185): two Newton steps from the linear guess x0/2.          */
static agc_w2_t sqrt_newton(agc_w2_t m, int32_t x0h)
{
    int32_t buf = x0h;
    int32_t q;
    agc_w2_t x1h;
    agc_w2_t qr;
    agc_w2_t add;
    int o = 0;
    (void)AGC_ASSERT((m.hi > 0) && (m.hi < buf));
    q = agc_w_dv(m.hi, 0, buf).hi;                     /* (MPAC/2)/(X0/2)  */
    buf += agc_w_mp(q, AGC_W_HALF).hi;                 /* X1 = ...         */
    x1h = agc_w_mp(buf, AGC_W_HALF);                   /* X1/2 as DP       */
    qr = agc_w_dv(m.hi, m.lo, buf);                    /* DV BUF           */
    add.hi = qr.hi;
    add.lo = agc_w_dv(qr.lo, 0, buf).hi;               /* minor quotient   */
    x1h = agc_w_das(x1h, add, &o);                     /* X2 = X1/2 + ..   */
    if (o != 0) {                                      /* near POSMAX      */
        x1h.hi = AGC_W_POSMAX;                         /* FIXROOT          */
        x1h.lo = AGC_W_POSMAX;
    }
    (void)AGC_ASSERT(x1h.hi > AGC_W_QUARTER);          /* ~.5 (may be 17777) */
    return x1h;
}

static agc_w2_t sqrt_arghi(agc_w2_t m)
{
    (void)AGC_ASSERT(m.hi >= AGC_W_QUARTER);
    (void)AGC_ASSERT(m.hi < AGC_W_HALF);
    return sqrt_newton(m, agc_w_mp(SQ_SLOPEHI, m.hi).hi + SQ_BIASHI);
}

static agc_w2_t sqrt_arglo(agc_w2_t m)
{
    (void)AGC_ASSERT(m.hi >= AGC_W_QUARTER / 2);
    (void)AGC_ASSERT(m.hi < AGC_W_QUARTER);
    return sqrt_newton(m, agc_w_mp(SQ_SLOPELO, m.hi).hi + SQ_BIASLO);
}

/* SMPAC+ / SRTEST / SQRTNORM / NORMTEST (p.1185-1186): MPAC major > 0.     */
static agc_w2_t sqrt_positive(agc_w3_t m, int *shift)
{
    int pass;
    int o = 0;
    (void)AGC_ASSERT((m.m0 > 0) && (shift != 0));
    if (m.m0 >= AGC_W_HALF) {                          /* >= .5            */
        return sqrt_arghi(sqrt_half(agc_w3_top(m)));
    }
    if (m.m0 >= AGC_W_QUARTER) {                       /* .25 .. .5        */
        return sqrt_arglo(sqrt_half(agc_w3_top(m)));
    }
    *shift += 1;                                       /* SQRTNORM         */
    m = agc_w_tp_shl1(m, &o);
    for (pass = 0; pass < SQ_NORM_MAX; pass++) {       /* NORMTEST (CYL)   */
        if (m.m0 >= AGC_W_QUARTER) {
            return sqrt_arghi(agc_w3_top(m));
        }
        if (m.m0 >= AGC_W_QUARTER / 2) {
            return sqrt_arglo(agc_w3_top(m));
        }
        m = agc_w_tp_shl1(m, &o);                      /* SQRTNM2          */
        *shift += 1;
        m = agc_w_tp_shl1(m, &o);
    }
    (void)AGC_ASSERT(o == 0);
    return agc_w3_top(m);
}

/* SQRTSUB (p.1184-1186): normalised root of the TP MPAC.                   */
agc_w2_t agc_w_sqrtsub(agc_w3_t m, int *shift, bool *abort)
{
    agc_w2_t zero = { 0, 0 };
    int k;
    if (!AGC_ASSERT(shift != 0) || !AGC_ASSERT(abort != 0)) {
        return zero;
    }
    *shift = 0;
    *abort = false;
    if (m.m0 < 0) {                                    /* SQRTNEG          */
        *abort = (m.m0 < -1) || (m.m1 < 0);            /* POODOO 01302     */
        return zero;                                   /* ZEROANS          */
    }
    for (k = 0; (k < 2) && (m.m0 == 0); k++) {         /* shift left 14    */
        m.m0 = m.m1;
        m.m1 = m.m2;
        m.m2 = 0;
        *shift += 7;                                   /* CAF SEVEN        */
        if (m.m0 < 0) {
            return zero;                               /* ZEROANS          */
        }
    }
    if (m.m0 == 0) {
        return zero;                                   /* SQRT(0) = 0      */
    }
    (void)AGC_ASSERT((*shift == 0) || (*shift == 7) || (*shift == 14));
    return sqrt_positive(m, shift);
}
