/*
 * agc_interp_trig.c -- interpretive SQRT, SIN, COS, ASIN, ACOS
 * (INTERPRETER.agc, Comanche055 pp.1181, 1184-1191; Luminary099
 * pp.1076, 1079-1086).
 *
 *   SIN  x  -> (1/2) sin(2 pi x)      x in revolutions, result scaled at 2
 *   COS  x  -> (1/2) cos(2 pi x)
 *   ASIN a  -> (1/2pi) arcsin(2a)     a = sin/2, result in revolutions
 *   ACOS a  -> (1/2pi) arccos(2a)
 *   SQRT a  -> sqrt(a), same binary point
 *
 * The argument reductions produce word pairs whose signs disagree (e.g.
 * hi = 1.0 - hi, lo = -lo); the polynomial and multiply steps see those
 * words exactly as the AGC does.
 */
#include "../mpac.h"
#include "agc/core/assert.h"

#define SIN_DEGREE  4
#define ACOS_DEGREE 7

/* Hastings sine coefficients, SN1 (p.1188), 2DEC words as assembled.       */
static const agc_dp_t sin_coef[SIN_DEGREE + 1] = {
    105414357,  /* +.3926990796     14441 37325   (pi/8)                   */
    -173399563, /* -.6459637111     53250 60764                            */
    85566142,   /* +.318758717      12146 21276                            */
    -20073670,  /* -.074780249      75466 71471                            */
    2602479     /* +.009694988      00236 32757                            */
};

/* Hastings arccos coefficients C(i) 2^i / (pi sqrt 2), ACOS3 (p.1190).     */
static const agc_dp_t acos_coef[ACOS_DEGREE + 1] = {
    94906264,  /* +.353553385                  13240 23630                */
    -25931778, /* -.0483017006 B+1             74721 47775                */
    21504159,  /* +.0200273085 B+2             02440 20237                */
    -24251933, /* -.0112931863 B+3             75067 70742                */
    29863406,  /* +.00695311612 B+4            03436 26756                */
    -33038431, /* -.00384617957 B+5            74037 57640                */
    25792099,  /* +.001501297736 B+6           03046 07143                */
    -9763675   /* -.000284160334 B+7           76654 42244                */
};

/* ---- SQRT ---------------------------------------------------------------- */

/* SQRT (p.1181): SQRTSUB, then un-normalise by MPTEMP (SQRTSHFT for < 14,
 * GENSCR +4 without round for longer shifts).                              */
static agc_w2_t sqrt_words(agc_w3_t m, bool *abort)
{
    int      shift = 0;
    agc_w2_t r     = agc_w_sqrtsub(m, &shift, abort);
    agc_w3_t t;
    (void) AGC_ASSERT(abort != 0);
    (void) AGC_ASSERT((shift >= 0) && (shift <= 42));
    if (shift == 0)
    {
        return r;
    }
    t.m0 = r.hi;
    t.m1 = r.lo;
    t.m2 = 0; /* CAF ZERO; TS +2  */
    return agc_w3_top(agc_w_genscr(t, shift, false));
}

agc_status_t agc_dp_sqrt_chk(agc_interp_t *ip, agc_dp_t a, agc_dp_t *out)
{
    bool     abort = false;
    agc_w2_t r;
    if (!AGC_ASSERT(ip != 0) || !AGC_ASSERT(out != 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    r = sqrt_words(agc_w3_from_tp(agc_tp_from_dp(a)), &abort);
    (void) AGC_ASSERT(r.hi >= 0);
    *out = agc_w_join(r);
    return abort ? AGC_STATUS_POODOO : AGC_STATUS_OK;
}

agc_dp_t agc_dp_sqrt(agc_interp_t *ip, agc_dp_t a)
{
    agc_dp_t r = 0;
    (void) AGC_ASSERT(ip != 0);
    /* POODOO 01302 is reported by agc_dp_sqrt_chk; here the result is 0.   */
    (void) agc_dp_sqrt_chk(ip, a, &r);
    (void) AGC_ASSERT(r >= 0);
    return r;
}

agc_dp_t agc_tp_sqrt(agc_interp_t *ip, agc_tp_t a)
{
    bool     abort = false;
    agc_w2_t r;
    (void) AGC_ASSERT(ip != 0);
    (void) AGC_ASSERT((a >= AGC_TP_MIN) && (a <= AGC_TP_MAX));
    r = sqrt_words(agc_w3_from_tp(a), &abort);
    return abort ? 0 : agc_w_join(r);
}

/* ---- SIN / COS ----------------------------------------------------------- */

/* SN1 (p.1188): x (scaled at pi, |x| <= pi/2) times the Hastings
 * polynomial in x^2, shifted left 2: (1/2) sin(x).                         */
static agc_w2_t sn1_words(agc_w2_t x)
{
    agc_w3_t sq = agc_w_dsqsub(x); /* TC DSQSUB        */
    agc_w3_t p  = agc_w_poly(agc_w3_top(sq), sin_coef, SIN_DEGREE);
    agc_w3_t r  = agc_w_dmpsub(agc_w3_top(p), x); /* DMPSUB by BUF2   */
    int      o  = 0;
    r           = agc_w_tp_shl1(r, &o); /* "NEITHER SHIFT   */
    r           = agc_w_tp_shl1(r, &o); /*  OVERFLOWS"      */
    (void) AGC_ASSERT(o == 0);
    (void) AGC_ASSERT(agc_w_ok(r.m0));
    return agc_w3_top(r);
}

/* SINE (p.1187): double the argument (now scaled at pi), reflect it into
 * [-pi/2, pi/2] with words exactly as the AGC leaves them, then SN1.       */
static agc_w2_t sine_words(agc_w2_t m)
{
    int      carry = 0;
    int      s     = 0;
    int32_t  lo    = agc_w_ts(2 * m.lo, &carry); /* DDOUBL: L        */
    int32_t  a16   = 2 * m.hi + carry;           /* DDOUBL: A        */
    agc_w2_t x;
    (void) AGC_ASSERT(agc_w_ok(m.hi) && agc_w_ok(m.lo));
    if ((a16 >= AGC_W_ONE) || (a16 <= -AGC_W_ONE))
    {               /* OVSK: lost pi    */
        a16 = -a16; /* DCOM             */
        lo  = -lo;
    }
    x.hi = agc_w_ts(a16, &s); /* DXCH MPAC        */
    x.lo = lo;
    (void) agc_w_ts(2 * x.hi, &s); /* CA; DOUBLE; TS L */
    if (s != 0)
    {                                /* |x| > pi/2       */
        x.hi = s * AGC_W_ONE - x.hi; /* +-pi - x         */
        x.lo = -x.lo;                /* CS MPAC +1       */
    }
    (void) AGC_ASSERT(agc_w_ok(x.hi));
    return sn1_words(x);
}

/* SIN: (1/2) sin(2 pi a), a in revolutions.                                 */
agc_dp_t agc_dp_sin(agc_interp_t *ip, agc_dp_t a)
{
    (void) AGC_ASSERT(ip != 0);
    (void) AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    return agc_w_join(sine_words(agc_w_split(a)));
}

/* COSINE (p.1187): cos(x) = sin(pi/2 - |x|); BRANCH, DCS if positive,
 * ADS QUARTER.                                                             */
agc_dp_t agc_dp_cos(agc_interp_t *ip, agc_dp_t a)
{
    agc_w2_t m = agc_w_split(a);
    (void) AGC_ASSERT(ip != 0);
    (void) AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    if (a > 0)
    {
        m = agc_w_dcs(m);
    }
    m.hi += AGC_W_QUARTER; /* PRESINE          */
    return agc_w_join(sine_words(m));
}

/* ---- ASIN / ACOS --------------------------------------------------------- */

/* ACOSST .. ACOS3 (p.1189-1191) for a positive argument (words m):
 * arccos(2m)/2pi = sqrt(1/2 - m) P(m). Out-of-range sets *alarm.           */
static agc_w2_t acos_words(agc_w2_t m, bool *alarm)
{
    agc_w2_t zero = {0, 0};
    agc_w2_t s;
    agc_w2_t root;
    agc_w3_t p;
    int      shift = 0;
    bool     abort = false;
    (void) AGC_ASSERT((m.hi >= 0) && (m.lo >= 0));
    if (m.hi > AGC_W_HALF)
    {                                      /* ACOSOVF          */
        *alarm = (m.hi != AGC_W_HALF + 1); /* ACOSABRT 01301   */
        return zero;                       /* ACOS=0           */
    }
    if ((m.hi == AGC_W_HALF) && (m.lo >= 0))
    { /* -0: CCS MPAC +1  */
        return zero;
    }
    s.hi = AGC_W_HALF - m.hi; /* ACOSST2: DCS; AD */
    s.lo = -m.lo;
    {
        agc_w3_t t = {s.hi, s.lo, 0};
        root       = agc_w_sqrtsub(t, &shift, &abort); /* SQRTSUB          */
    }
    (void) AGC_ASSERT(!abort && (shift <= 14));
    if (shift > 0)
    {
        root = agc_w_vshrrnd(root, shift); /* ACOSSHR          */
    }
    p = agc_w_poly(m, acos_coef, ACOS_DEGREE); /* ACOS3: POLY      */
    return agc_w3_top(agc_w_dmpsub(agc_w3_top(p), root));
}

/* ARCCOS / ARCSIN entry, BRANCH, ESCAPE/ESCAPE2 epilogues (p.1189-1190).    */
static agc_w2_t arc_words(agc_dp_t a, bool asin, bool *alarm)
{
    agc_w2_t m = agc_w_split(a);
    agc_w2_t r;
    (void) AGC_ASSERT(alarm != 0);
    if (a == 0)
    { /* ACOSZERO         */
        r.hi = AGC_W_QUARTER;
        r.lo = 0;
    }
    else
    {
        r = acos_words((a < 0) ? agc_w_dcs(m) : m, alarm);
        if (a < 0)
        { /* SUBTR            */
            r.hi = AGC_W_HALF - r.hi;
            r.lo = -r.lo;
        }
    }
    if (asin)
    { /* ASINEX           */
        r.hi = AGC_W_QUARTER - r.hi;
        r.lo = -r.lo;
    }
    (void) AGC_ASSERT(agc_w_ok(r.hi));
    return r;
}

agc_status_t agc_dp_acos_chk(agc_interp_t *ip, agc_dp_t a, agc_dp_t *out)
{
    bool alarm = false;
    if (!AGC_ASSERT(ip != 0) || !AGC_ASSERT(out != 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *out = agc_w_join(arc_words(a, false, &alarm));
    (void) AGC_ASSERT((*out >= 0) && (*out <= AGC_DP_HALF));
    return alarm ? AGC_STATUS_ALARM : AGC_STATUS_OK;
}

agc_status_t agc_dp_asin_chk(agc_interp_t *ip, agc_dp_t a, agc_dp_t *out)
{
    bool alarm = false;
    if (!AGC_ASSERT(ip != 0) || !AGC_ASSERT(out != 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *out = agc_w_join(arc_words(a, true, &alarm));
    (void) AGC_ASSERT((*out >= -AGC_DP_QUARTER) && (*out <= AGC_DP_QUARTER));
    return alarm ? AGC_STATUS_ALARM : AGC_STATUS_OK;
}

/* ACOS: alarm 01301 (if any) is reported by agc_dp_acos_chk.                */
agc_dp_t agc_dp_acos(agc_interp_t *ip, agc_dp_t a)
{
    agc_dp_t r = 0;
    (void) AGC_ASSERT(ip != 0);
    (void) agc_dp_acos_chk(ip, a, &r); /* alarm: see _chk  */
    (void) AGC_ASSERT(r >= 0);
    return r;
}

agc_dp_t agc_dp_asin(agc_interp_t *ip, agc_dp_t a)
{
    agc_dp_t r = 0;
    (void) AGC_ASSERT(ip != 0);
    (void) agc_dp_asin_chk(ip, a, &r); /* alarm: see _chk  */
    (void) AGC_ASSERT((r >= -AGC_DP_QUARTER) && (r <= AGC_DP_QUARTER));
    return r;
}
