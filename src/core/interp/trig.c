/**********************************************************************
 * FILE NAME: trig.c
 *
 * PURPOSE:
 *   Interpretive SQRT, SIN, COS, ASIN and ACOS, and the latch through
 *   which their ABORT 01302 / ALARM 01301 reach the executive.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc p.1181 (SQRT, SQRTSHFT), pp.1187-1188
 *   (SINE, COSINE, PRESINE, SN1), pp.1189-1191 (ARCSIN, ARCCOS, ACOSST,
 *   ACOSST2, ACOS3, SUBTR, ASINEX, ACOSSHR, ACOSOVF, ACOSZERO)
 *   Luminary099 INTERPRETER.agc pp.1076, 1082-1086 (values identical;
 *   SQRTABRT via POODOO1, ACOSABRT via ALARM1 with LOC)
 *
 * EXTERNAL VARIABLES:
 *   Name            Type      I/O  Scale     Description
 *   --------------  --------  ---  --------  -------------------------
 *   g_interp_alarm  uint16_t  IO   octal     alarm latched for exec
 *                                            (file static)
 *
 * EXTERNAL REFERENCES:
 *   Name           Description
 *   -------------  -----------------------------------------------
 *   agc_w_sqrtsub  SQRTSUB (divide.c)
 *   agc_w_poly     POLY (mpac.c)
 *   agc_w_dmpsub   DMPSUB (mpac.c)
 *   agc_w_genscr   GENSCR (shift.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   SQRT of a < -2^-14: ABORT 01302 (POODOO); ASIN/ACOS of |a| more than
 *   one major LSB above 1/2: ALARM 01301, result 0 (ACOS) / .25 (ASIN),
 *   negated per sign. The _chk functions return AGC_STATUS_POODOO /
 *   AGC_STATUS_ALARM; the plain ones latch the code for the executive
 *   (agc_interp_take_alarm), since core cannot call it.
 *
 * NOTES:
 *   SIN x -> (1/2) sin(2 pi x), x in revolutions, result scaled at 2;
 *   COS likewise; ASIN a -> (1/2pi) arcsin(2a), a = sin/2, result in
 *   revolutions; ACOS likewise; SQRT a -> sqrt(a) at the same binary
 *   point. The argument reductions produce word pairs whose signs
 *   disagree (hi = 1.0 - hi, lo = -lo); the polynomial and multiply
 *   steps see those words exactly as the AGC does. Coefficients were
 *   checked against the assembled words of both ropes.
 **********************************************************************/
#include "../mpac.h"
#include "agc/core/assert.h"
#include "agc/core/interp_words.h"

#define SIN_DEGREE  4 /* SN1 polynomial degree (5 coefficients) */
#define ACOS_DEGREE 7 /* ACOS3 POLY degree (DEC 6 = N-1) */

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

/**********************************************************************
 * SQRT (INTERPRETER.agc p.1181) -- SQRTSUB, then un-normalise by
 * MPTEMP: none (DANZIG: MPAC+2 as SQRTSUB left it), SQRTSHFT (MPAC+2
 * zeroed, then MPACSHR) below 14, GENSCR +4 without round beyond.
 *
 * ARGUMENTS:
 *   m      agc_w3_t  I  TP MPAC (words of any signs)
 *   abort  bool *    O  SQRTABRT (POODOO 01302) taken
 *
 * RETURNS: agc_w3_t -- MPAC after SQRT: sqrt(m) at the same binary
 *          point in m0, m1, and MPAC+2
 **********************************************************************/
static agc_w3_t sqrt_words(agc_w3_t m, bool *abort)
{
    int      shift = 0;                               /* MPTEMP: right shifts owed */
    agc_w3_t r     = agc_w_sqrtsub(m, &shift, abort); /* normalised root, MPAC+2 */

    if (!AGC_ASSERT((shift >= 0) && (shift <= 42)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    if (shift == 0)
    {
        return (r); /* CCS MPTEMP: TCF DANZIG */
    }

    r.m2 = 0; /* CAF ZERO; TS MPAC +2 */

    return (agc_w_genscr(r, shift, false));
}

/* The interpreter alarm the last SQRT / ASIN / ACOS raised (01302 ABORT,
 * 01301 ALARM), held for the executive: core cannot call agc_alarm /
 * agc_poodoo (layering), so exec raises it when the job or task that
 * did the arithmetic returns (agc_interp_take_alarm).                     */
static uint16_t g_interp_alarm; /* 01301 / 01302 / 0, octal */

/**********************************************************************
 * AGC: none (layering) -- latch ALARM 01301 / ABORT 01302 for the
 * executive; the abort wins over an alarm.
 *
 * ARGUMENTS:
 *   code  uint16_t  I  01301 or 01302
 **********************************************************************/
static void interp_flag_alarm(uint16_t code)
{
    if (g_interp_alarm != 01302u)
    { /* the abort wins over an alarm         */
        g_interp_alarm = code;
    }
}

/**********************************************************************
 * AGC: none (layering) -- the alarm the plain SQRT / ASIN / ACOS
 * latched since the last call, cleared by reading; exec raises it when
 * the job or task returns.
 *
 * RETURNS: uint16_t -- 01302 (ABORT), 01301 (ALARM) or 0
 **********************************************************************/
uint16_t agc_interp_take_alarm(void)
{
    uint16_t code = g_interp_alarm; /* latched code, 0 if none */

    g_interp_alarm = 0;

    return (code);
}

/**********************************************************************
 * SQRT (INTERPRETER.agc pp.1181, 1184-1186) with the abort reported.
 *
 * ARGUMENTS:
 *   a    agc_dp_t        I  argument at 2^2n
 *   out  agc_dp_t *      O  root at 2^n; 0 on the abort
 *
 * RETURNS: agc_status_t -- AGC_STATUS_POODOO for a < -2^-14 (caller
 *          raises POODOO 01302), else AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_dp_sqrt_chk(agc_dp_t a, agc_dp_t *out)
{
    bool     abort = false; /* SQRTABRT taken */
    agc_w2_t r;             /* root words */

    r = agc_w3_top(sqrt_words(agc_w3_from_tp(agc_tp_from_dp(a)), &abort));

    *out = agc_w_join(r);

    if (!AGC_ASSERT(!abort || (*out == 0)))
    {
        return (AGC_STATUS_BAD_ARG); /* SQRTABRT leaves no root     */
    }

    return (abort ? AGC_STATUS_POODOO : AGC_STATUS_OK); /* POODOO 01302 */
}

/**********************************************************************
 * SQRT (INTERPRETER.agc pp.1181, 1184-1186) -- the abort is latched for
 * the executive (agc_interp_take_alarm).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I  interpreter state
 *   a   agc_dp_t        I  argument at 2^2n
 *
 * RETURNS: agc_dp_t -- sqrt(a) at 2^n; 0 for -2^-14 <= a <= 0 and on
 *          the abort
 **********************************************************************/
agc_dp_t agc_dp_sqrt(agc_interp_t *ip, agc_dp_t a)
{
    agc_dp_t r = 0; /* root, scaled 2^n */

    if (agc_dp_sqrt_chk(a, &r) != AGC_STATUS_OK)
    { /* ABORT 01302 (a < -2^-14): raised by exec when the job returns */
        interp_flag_alarm(01302u);

        return (0);
    }

    return (r);
}

/**********************************************************************
 * SQRT of a TP MPAC (TLOAD / DMP / DSQ then SQRT, INTERPRETER.agc
 * p.1181): MPAC+2 takes part in the root.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I  interpreter state
 *   a   agc_tp_t        I  TP argument at 2^2n
 *
 * RETURNS: agc_dp_t -- sqrt(a) at 2^n; 0 on the abort (latched)
 **********************************************************************/
agc_dp_t agc_tp_sqrt(agc_interp_t *ip, agc_tp_t a)
{
    bool     abort = false; /* SQRTABRT taken */
    agc_w2_t r;             /* root words */

    r = agc_w3_top(sqrt_words(agc_w3_from_tp(a), &abort));

    if (abort)
    { /* ABORT 01302 (a < -2^-14): raised by exec when the job returns */
        interp_flag_alarm(01302u);

        return (0);
    }

    return (agc_w_join(r));
}

/* ---- SIN / COS ----------------------------------------------------------- */

/**********************************************************************
 * SN1 (INTERPRETER.agc p.1188) -- x (scaled at pi, |x| <= pi/2) times
 * the Hastings polynomial in x^2, shifted left 2: (1/2) sin(x).
 *
 * ARGUMENTS:
 *   x  agc_w2_t  I  reduced argument (BUF2), signs may disagree
 *
 * RETURNS: agc_w3_t -- sin(x)/2 (scaled at 2) as the TP MPAC
 **********************************************************************/
static agc_w3_t sn1_words(agc_w2_t x)
{
    agc_w3_t sq;    /* x^2 (DSQSUB), scaled at pi^2             */
    agc_w3_t p;     /* Hastings polynomial in x^2, scaled at 1  */
    agc_w3_t r;     /* x P(x^2), then 4 x P = sin(x)/2          */
    int      o = 0; /* shift overflow (none, "NEITHER SHIFT")   */

    if (!AGC_ASSERT((x.hi >= -AGC_W_HALF) && (x.hi <= AGC_W_HALF)))
    {
        return ((agc_w3_t){0}); /* argument beyond +-pi/2 */
    }

    sq = agc_w_dsqsub(x);                                  /* TC DSQSUB */
    p  = agc_w_poly(agc_w3_top(sq), sin_coef, SIN_DEGREE); /* POLY      */
    r  = agc_w_dmpsub(agc_w3_top(p), x);                   /* DMPSUB by BUF2 */
    r  = agc_w_tp_shl1(r, &o);                             /* "NEITHER SHIFT   */
    r  = agc_w_tp_shl1(r, &o);                             /*  OVERFLOWS"      */

    if (!AGC_ASSERT(o == 0))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * SINE (INTERPRETER.agc p.1187) -- double the argument (now scaled at
 * pi), reflect it into [-pi/2, pi/2] with the words exactly as the AGC
 * leaves them, then SN1.
 *
 * ARGUMENTS:
 *   m  agc_w2_t  I  argument in revolutions (MPAC, MPAC+1)
 *
 * RETURNS: agc_w3_t -- (1/2) sin(2 pi m), TP MPAC
 **********************************************************************/
static agc_w3_t sine_words(agc_w2_t m)
{
    int      carry = 0;                          /* minor-word carry of DDOUBL */
    int      s     = 0;                          /* overflow of TS / DOUBLE, +-1 or 0 */
    int32_t  lo    = agc_w_ts(2 * m.lo, &carry); /* DDOUBL: L        */
    int32_t  a16   = 2 * m.hi + carry;           /* DDOUBL: A        */
    agc_w2_t x;                                  /* reduced argument, scaled at pi */

    if (!AGC_ASSERT(agc_w_ok(m.hi) && agc_w_ok(m.lo)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    if ((a16 >= AGC_W_ONE) || (a16 <= -AGC_W_ONE))
    { /* OVSK: lost pi    */
        a16 = -a16; /* DCOM             */
        lo  = -lo;
    }

    x.hi = agc_w_ts(a16, &s);   /* DXCH MPAC        */
    x.lo = lo;
    s    = agc_w_ovf(2 * x.hi); /* CA; DOUBLE; TS L */

    if (s != 0)
    { /* |x| > pi/2       */
        x.hi = s * AGC_W_ONE - x.hi; /* +-pi - x         */
        x.lo = -x.lo;                /* CS MPAC +1       */
    }

    if (!AGC_ASSERT(agc_w_ok(x.hi)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    return (sn1_words(x));
}

/**********************************************************************
 * COSINE / PRESINE (INTERPRETER.agc p.1187) -- cos(x) = sin(pi/2 -
 * |x|): BRANCH on the three MPAC words, DCS MPAC (two words) if
 * positive, ADS QUARTER into the major word, then SINE.
 *
 * ARGUMENTS:
 *   m  agc_w3_t  I  TP MPAC, angle in revolutions
 *
 * RETURNS: agc_w3_t -- (1/2) cos(2 pi m), TP MPAC
 **********************************************************************/
static agc_w3_t cosine_words(agc_w3_t m)
{
    agc_w2_t x = agc_w3_top(m); /* MPAC, MPAC+1 */
    int      o = 0;             /* ADS overflow (none: x.hi <= 0 here) */

    if (!AGC_ASSERT(agc_w_ok(m.m2)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    if (agc_w_branch(m) > 0)
    {
        x = agc_w_dcs(x); /* EXTEND; DCS MPAC  */
    }

    x.hi = agc_w_ts(x.hi + AGC_W_QUARTER, &o); /* PRESINE: ADS MPAC */

    if (!AGC_ASSERT(o == 0))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    return (sine_words(x));
}

/**********************************************************************
 * SIN (SINE, INTERPRETER.agc p.1187).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I  interpreter state
 *   a   agc_dp_t        I  angle in revolutions, scaled 2^0
 *
 * RETURNS: agc_dp_t -- (1/2) sin(2 pi a), scaled at 2
 **********************************************************************/
agc_dp_t agc_dp_sin(agc_interp_t *ip, agc_dp_t a)
{
    return (agc_w_join(agc_w3_top(sine_words(agc_w_split(a)))));
}

/**********************************************************************
 * COS (COSINE / PRESINE, INTERPRETER.agc p.1187).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I  interpreter state
 *   a   agc_dp_t        I  angle in revolutions, scaled 2^0
 *
 * RETURNS: agc_dp_t -- (1/2) cos(2 pi a), scaled at 2
 **********************************************************************/
agc_dp_t agc_dp_cos(agc_interp_t *ip, agc_dp_t a)
{
    return (agc_w_join(agc_w3_top(cosine_words(agc_w3_from_tp(agc_tp_from_dp(a))))));
}

/* ---- ASIN / ACOS --------------------------------------------------------- */

/**********************************************************************
 * ACOSST .. ACOS3 (INTERPRETER.agc pp.1189-1191) for a positive
 * argument: arccos(2m)/2pi = sqrt(1/2 - m) P(m). ACOSST2 forms 1/2 - m
 * with DCS MPAC; AD HALF, so SQRTSUB sees the argument's MPAC+2
 * uncomplemented. Out of range (ACOSOVF) gives 0, with ACOSABRT unless
 * the major word is only HALF+1; ACOS=0 leaves MPAC+2 alone.
 *
 * ARGUMENTS:
 *   m      agc_w3_t  I  TP MPAC, positive by BRANCH (sin/2 scaled at 2)
 *   alarm  bool *    O  ACOSABRT (ALARM 01301) taken
 *
 * RETURNS: agc_w3_t -- arccos(2m)/2pi in revolutions, [0, .25], as the
 *          TP MPAC
 **********************************************************************/
static agc_w3_t acos_words(agc_w3_t m, bool *alarm)
{
    agc_w3_t t;             /* 1/2 - m, scaled at 2, with m's MPAC+2 */
    agc_w3_t root;          /* sqrt(1/2 - m), normalised */
    agc_w3_t p;             /* P(m) (POLY) */
    agc_w2_t sh;            /* root after ACOSSHR */
    int      shift = 0;     /* MPTEMP: right shifts owed */
    bool     abort = false; /* SQRTABRT (cannot happen: 1/2 - m >= 0) */

    if (!AGC_ASSERT((alarm != 0) && (agc_w_branch(m) > 0)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    if ((m.m0 > AGC_W_HALF) || ((m.m0 == AGC_W_HALF) && (m.m1 >= 0)))
    { /* ACOSOVF, or CCS MPAC +1 >= 0: ACOS=0 */
        *alarm = (m.m0 > AGC_W_HALF + 1); /* ACOSABRT 01301   */
        m.m0   = 0;
        m.m1   = 0;

        return (m);
    }

    t.m0 = AGC_W_HALF - m.m0;                /* ACOSST2: DCS; AD HALF */
    t.m1 = -m.m1;
    t.m2 = m.m2;
    root = agc_w_sqrtsub(t, &shift, &abort); /* SQRTSUB          */

    if (!AGC_ASSERT(!abort && (shift <= 14)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    if (shift > 0)
    { /* ACOSSHR: VSHRRND   */
        sh   = agc_w_vshrrnd(agc_w3_top(root), shift);
        root = (agc_w3_t){sh.hi, sh.lo, 0};
    }

    p = agc_w_poly(agc_w3_top(m), acos_coef, ACOS_DEGREE); /* ACOS3: POLY      */

    return (agc_w_dmpsub(agc_w3_top(p), agc_w3_top(root)));
} /* acos_words */

/**********************************************************************
 * ARCCOS / ARCSIN entry, BRANCH, ESCAPE / ESCAPE2 epilogues
 * (INTERPRETER.agc pp.1189-1190): ACOSZERO, DCS MPAC (two words) and
 * SUBTR for a negative argument, ASINEX for ARCSIN. The epilogues
 * change MPAC, MPAC+1 only.
 *
 * ARGUMENTS:
 *   m      agc_w3_t  I  TP MPAC, sin/2 or cos/2 scaled at 2
 *   asin   bool      I  ARCSIN (ASINEX epilogue)
 *   alarm  bool *    O  ACOSABRT taken
 *
 * RETURNS: agc_w3_t -- the angle in revolutions as the TP MPAC
 **********************************************************************/
static agc_w3_t arc_words(agc_w3_t m, bool asin, bool *alarm)
{
    int      sign = agc_w_branch(m);       /* BRANCH: sign of the first non-zero word */
    agc_w3_t r    = {AGC_W_QUARTER, 0, 0}; /* ACOSZERO: TS MPAC  */

    if (sign != 0)
    {
        if (sign < 0)
        { /* EXTEND; DCS MPAC; DXCH MPAC */
            m.m0 = -m.m0;
            m.m1 = -m.m1;
        }

        r = acos_words(m, alarm);

        if (sign < 0)
        { /* SUBTR: DCS MPAC; AD HALF */
            r.m0 = AGC_W_HALF - r.m0;
            r.m1 = -r.m1;
        }
    }

    if (asin)
    { /* ASINEX: DCS MPAC; AD QUARTER */
        r.m0 = AGC_W_QUARTER - r.m0;
        r.m1 = -r.m1;
    }

    if (!AGC_ASSERT(agc_w_ok(r.m0)))
    {
        return ((agc_w3_t){0}); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * ACOS (ARCCOS, INTERPRETER.agc pp.1189-1191) with the alarm reported.
 * TS ESCAPE leaves a positive return address in DVSIGN (= DOTINC).
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  IO DVSIGN made positive
 *   a    agc_dp_t        I  argument, cos/2 scaled at 2 (|a| <= 1/2)
 *   out  agc_dp_t *      O  the angle, revolutions in [0, .5]
 *
 * RETURNS: agc_status_t -- AGC_STATUS_ALARM when ACOSABRT was taken
 *          (caller raises ALARM 01301), else AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_dp_acos_chk(agc_interp_t *ip, agc_dp_t a, agc_dp_t *out)
{
    bool alarm = false; /* ACOSABRT taken */

    ip->dvsign_neg = false; /* TS ESCAPE (= DVSIGN): a return address */
    *out           = agc_w_join(agc_w3_top(arc_words(agc_w3_from_tp(agc_tp_from_dp(a)), false, &alarm)));

    if (!AGC_ASSERT((*out >= 0) && (*out <= AGC_DP_HALF)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (alarm ? AGC_STATUS_ALARM : AGC_STATUS_OK); /* ALARM 01301 */
}

/**********************************************************************
 * ASIN (ARCSIN, INTERPRETER.agc pp.1189-1191) with the alarm reported.
 * TS ESCAPE leaves a positive return address in DVSIGN (= DOTINC).
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  IO DVSIGN made positive
 *   a    agc_dp_t        I  argument, sin/2 scaled at 2 (|a| <= 1/2)
 *   out  agc_dp_t *      O  the angle, revolutions in [-.25, .25]
 *
 * RETURNS: agc_status_t -- AGC_STATUS_ALARM when ACOSABRT was taken
 *          (caller raises ALARM 01301), else AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_dp_asin_chk(agc_interp_t *ip, agc_dp_t a, agc_dp_t *out)
{
    bool alarm = false; /* ACOSABRT taken */

    ip->dvsign_neg = false; /* TS ESCAPE (= DVSIGN): a return address */
    *out           = agc_w_join(agc_w3_top(arc_words(agc_w3_from_tp(agc_tp_from_dp(a)), true, &alarm)));

    if (!AGC_ASSERT((*out >= -AGC_DP_QUARTER) && (*out <= AGC_DP_QUARTER)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (alarm ? AGC_STATUS_ALARM : AGC_STATUS_OK); /* ALARM 01301 */
}

/**********************************************************************
 * ACOS (ARCCOS, INTERPRETER.agc pp.1189-1191) -- ALARM 01301 latched
 * for the executive.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO DVSIGN made positive
 *   a   agc_dp_t        I  argument, cos/2 scaled at 2
 *
 * RETURNS: agc_dp_t -- arccos(2a)/2pi in revolutions, [0, .5]
 **********************************************************************/
agc_dp_t agc_dp_acos(agc_interp_t *ip, agc_dp_t a)
{
    agc_dp_t r = 0; /* angle, revolutions scaled 2^0 */

    if (agc_dp_acos_chk(ip, a, &r) != AGC_STATUS_OK)
    { /* ALARM 01301 (|a| > 1/2), raised by exec; r is the limit value */
        interp_flag_alarm(01301u);

        return (r);
    }

    return (r);
}

/**********************************************************************
 * ASIN (ARCSIN, INTERPRETER.agc pp.1189-1191) -- ALARM 01301 latched
 * for the executive.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO DVSIGN made positive
 *   a   agc_dp_t        I  argument, sin/2 scaled at 2
 *
 * RETURNS: agc_dp_t -- arcsin(2a)/2pi in revolutions, [-.25, .25]
 **********************************************************************/
agc_dp_t agc_dp_asin(agc_interp_t *ip, agc_dp_t a)
{
    agc_dp_t r = 0; /* angle, revolutions scaled 2^0 */

    if (agc_dp_asin_chk(ip, a, &r) != AGC_STATUS_OK)
    { /* ALARM 01301 (|a| > 1/2), raised by exec; r is the limit value */
        interp_flag_alarm(01301u);

        return (r);
    }

    return (r);
}

/* ---- the same on MPAC words (interp_words.h) ----------------------------- */

/**********************************************************************
 * SQRT (INTERPRETER.agc pp.1181, 1184-1186) on the TP MPAC words:
 * MPAC+2 takes part in the normalisation and is left as the AGC leaves
 * it (the argument's after no shift, MPACSHR's or GENSCR's otherwise).
 * The abort (argument < -2^-14) is latched for the executive.
 *
 * ARGUMENTS:
 *   m   agc_tp_words_t  I  MPAC, argument at 2^2n
 *
 * RETURNS: agc_tp_words_t -- MPAC after SQRT, root at 2^n; 0 on the
 *          abort
 **********************************************************************/
agc_tp_words_t agc_mpac_sqrt(agc_tp_words_t m)
{
    bool     abort = false; /* SQRTABRT taken */
    agc_w3_t r;             /* MPAC after SQRT */

    r = sqrt_words(agc_w3_from_words(m), &abort);

    if (abort)
    { /* ABORT 01302: raised by exec when the job returns */
        interp_flag_alarm(01302u);

        return ((agc_tp_words_t){0, 0, 0});
    }

    return (agc_w3_to_words(r));
}

/**********************************************************************
 * SIN (SINE, INTERPRETER.agc pp.1187-1188) on MPAC, MPAC+1 as held.
 *
 * ARGUMENTS:
 *   m   agc_tp_words_t  I  MPAC, angle in revolutions (MPAC+2 unused)
 *
 * RETURNS: agc_tp_words_t -- (1/2) sin(2 pi m) as the TP MPAC
 **********************************************************************/
agc_tp_words_t agc_mpac_sin(agc_tp_words_t m)
{
    agc_w3_t r; /* SN1 result words */

    r = sine_words(agc_w3_top(agc_w3_from_words(m)));

    return (agc_w3_to_words(r));
}

/**********************************************************************
 * COS (COSINE, INTERPRETER.agc pp.1187-1188): BRANCH on the three
 * words, then SINE on MPAC, MPAC+1.
 *
 * ARGUMENTS:
 *   m   agc_tp_words_t  I  MPAC, angle in revolutions
 *
 * RETURNS: agc_tp_words_t -- (1/2) cos(2 pi m) as the TP MPAC
 **********************************************************************/
agc_tp_words_t agc_mpac_cos(agc_tp_words_t m)
{
    agc_w3_t r; /* SN1 result words */

    r = cosine_words(agc_w3_from_words(m));

    return (agc_w3_to_words(r));
}

/**********************************************************************
 * ARCSIN / ARCCOS (INTERPRETER.agc pp.1189-1191) on the MPAC words;
 * ESCAPE (= DVSIGN) gets a return address; ALARM 01301 is latched.
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *  IO DVSIGN made positive
 *   m     agc_tp_words_t  I  MPAC, sin/2 or cos/2 scaled at 2
 *   asin  bool            I  ARCSIN (ASINEX epilogue)
 *
 * RETURNS: agc_tp_words_t -- the angle in revolutions as the TP MPAC
 **********************************************************************/
static agc_tp_words_t arc_mpac(agc_interp_t *ip, agc_tp_words_t m, bool asin)
{
    bool     alarm = false; /* ACOSABRT taken */
    agc_w3_t r;             /* angle words */

    ip->dvsign_neg = false; /* TS ESCAPE (= DVSIGN): a return address */
    r              = arc_words(agc_w3_from_words(m), asin, &alarm);

    if (alarm)
    { /* ALARM 01301, raised by exec */
        interp_flag_alarm(01301u);
    }

    if (!AGC_ASSERT(agc_w_ok(r.m0)))
    {
        return (m);
    }

    return (agc_w3_to_words(r));
}

/**********************************************************************
 * ASIN (ARCSIN, INTERPRETER.agc pp.1189-1191) on the MPAC words.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO DVSIGN made positive
 *   m   agc_tp_words_t  I  MPAC, sin/2 scaled at 2
 *
 * RETURNS: agc_tp_words_t -- arcsin(2m)/2pi in revolutions, TP MPAC
 **********************************************************************/
agc_tp_words_t agc_mpac_asin(agc_interp_t *ip, agc_tp_words_t m)
{
    if (!AGC_ASSERT((m.hi >= AGC_SP_MIN) && (m.hi <= AGC_SP_MAX)))
    {
        return (m);
    }

    return (arc_mpac(ip, m, true));
}

/**********************************************************************
 * ACOS (ARCCOS, INTERPRETER.agc pp.1189-1191) on the MPAC words.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO DVSIGN made positive
 *   m   agc_tp_words_t  I  MPAC, cos/2 scaled at 2
 *
 * RETURNS: agc_tp_words_t -- arccos(2m)/2pi in revolutions, TP MPAC
 **********************************************************************/
agc_tp_words_t agc_mpac_acos(agc_interp_t *ip, agc_tp_words_t m)
{
    if (!AGC_ASSERT((m.hi >= AGC_SP_MIN) && (m.hi <= AGC_SP_MAX)))
    {
        return (m);
    }

    return (arc_mpac(ip, m, false));
}
