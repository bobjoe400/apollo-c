/**********************************************************************
 * FILE NAME: divide.c
 *
 * PURPOSE:
 *   Interpretive division and square root at word level: DDV/BDDV with
 *   their overflow exits, the general DP divide GENDDV and its near-one
 *   case MAXDV, and the normalised square root SQRTSUB.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc pp.1162-1170 (DDV, BDDV, DDV/BDDV,
 *   BUF+, MPAC+, DVOVF, GENDDV, MAXDV, FINALDV) and pp.1184-1186
 *   (SQRTSUB, SQRTNEG, SMPAC+, ARGHI, ARGLO, SRTEST, SQRTNORM, FIXROOT)
 *   Luminary099 INTERPRETER.agc pp.1057-1065, 1079-1081 (identical
 *   values; SQRTABRT goes through POODOO1 with LOC)
 *
 * EXTERNAL VARIABLES:
 *   none (DVSIGN is returned to the caller, which keeps it in
 *   agc_interp_t)
 *
 * EXTERNAL REFERENCES:
 *   Name           Description
 *   -------------  -----------------------------------------------
 *   agc_w_dv       DV on words (mpac.c)
 *   agc_w_mp       MP on words (mpac.c)
 *   agc_w_das      DAS on word pairs (mpac.c)
 *   agc_w_tp_shl1  one TP left shift (mpac.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   DDV overflow (|a| >= |b|, b = 0): +-DPPOSMAX with OVFIND = 1.
 *   SQRTSUB of a negative argument below -2^-14 sets *abort (the AGC's
 *   POODOO 01302, raised by the caller).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Words are 15-bit ones'-complement values held in int32_t (mpac.h).
 *   Places where the AGC computes x + (-x) give -0 in the machine; they
 *   are commented where the following CCS/BZMF branch depends on it.
 **********************************************************************/
#include "agc/core/assert.h"
#include "mpac.h"

/* SQRT linear first-guess constants (INTERPRETER.agc p.1159-1174).          */
#define SQ_SLOPEHI  9640  /* DEC .5884        octal 22650                */
#define SQ_BIASHI   3434  /* DEC .4192 B-1    octal 06552                */
#define SQ_SLOPELO  13638 /* DEC .8324        octal 32506                */
#define SQ_BIASLO   2436  /* DEC .2974 B-1    octal 04604                */
#define SQ_NORM_MAX 16    /* SQRTNORM passes: 13 for any non-zero TP     */
#define DV_NORM_MAX 14    /* BUFNORM passes: divisor major part >= 1     */

/* ---- GENDDV / MAXDV ------------------------------------------------------ */

/**********************************************************************
 * FINALDV (INTERPRETER.agc p.1169) -- minor part of the quotient: the
 * remainder word over the divisor's major word, (a, 0) / c.
 *
 * ARGUMENTS:
 *   a  int32_t  I  remainder word, |a| < c
 *   c  int32_t  I  divisor major word (BUF), > 0
 *
 * RETURNS: int32_t -- the minor quotient word
 **********************************************************************/
static int32_t finaldv(int32_t a, int32_t c)
{
    return (agc_w_dv(a, 0, c).hi);
}

/**********************************************************************
 * GENDDV / MAXDV (INTERPRETER.agc pp.1166-1170) -- positive normalised
 * DP divide with MAXTEST's choice: GENDDV computes (A + sB)/(C + sD) =
 * Q + s(R - QD)/C, s = 2^-14, correcting Q by one either way ("SIGN
 * AGREEMENT IS UNNECESSARY": B and D keep whatever sign DDV left them);
 * MAXDV (equal major words, MAXDVSW = +0) gives 37777 + s(C + B - D)/C.
 *
 * ARGUMENTS:
 *   ab     agc_w2_t  I  dividend words (A, B), A >= 0, A <= C
 *   cd     agc_w2_t  I  divisor words (C, D), C >= HALF (UNIT may pass
 *                       17777)
 *   maxdv  bool      I  MAXDVSW is +0: take MAXDV when A = C
 *
 * RETURNS: agc_w2_t -- the quotient words (MPAC, MPAC+1), scaled 2^0
 **********************************************************************/
agc_w2_t agc_w_genddv(agc_w2_t ab, agc_w2_t cd, bool maxdv)
{
    agc_w2_t qr;  /* DV BUF: {Q, R}                    */
    agc_w2_t r = {0, 0};
    int32_t acc; /* dividend of the minor part        */

    if (maxdv && (ab.hi == cd.hi))
    { /* MAXDV: (C + sB)/(C + sD) = 37777 + s(C + B - D)/C, B < D */
        r.hi = AGC_W_POSMAX;
        r.lo = finaldv(ab.lo - cd.lo + cd.hi, cd.hi); /* CS; AD; AD BUF  */

        return (r);
    }

    qr = agc_w_dv(ab.hi, ab.lo, cd.hi);     /* DV BUF          */
    r.hi = qr.hi;                             /* Q, major word   */
    acc = qr.lo + agc_w_mp(-r.hi, cd.lo).hi; /* CS; MP; AD      */

    if (acc >= AGC_W_ONE)
    { /* OVSK: + overflow */
        acc -= cd.hi; /* SU BUF          */
        r.hi += 1;     /* INCR MPAC       */
    }
    else if (acc <= 0)
    { /* BZMF -UP        */
        if (acc == 0)
        {
            return (r); /* FINALDV +3: minor word 0 */
        }

        r.hi -= 1;                          /* DIM MPAC        */
        r.lo = finaldv(acc + cd.hi, cd.hi); /* ENDMAXDV AD BUF */

        return (r);
    }
    else
    {
        /* fall into +DOWN */
    }

    if (acc - cd.hi >= 0)
    { /* +DOWN: SU BUF   */
        acc -= cd.hi;
        r.hi += 1; /* +3: INCR MPAC   */
    }

    r.lo = finaldv(acc, cd.hi);

    return (r);
} /* agc_w_genddv */

/* ---- DDV / BDDV ---------------------------------------------------------- */

/**********************************************************************
 * DVOVF (INTERPRETER.agc p.1162) -- MPAC, MPAC+1 := +-POSMAX (FINALDV
 * +3 applies DVSIGN), OVFIND := 1; MPAC+2 is left alone.
 *
 * ARGUMENTS:
 *   m         agc_w3_t  I  MPAC (only MPAC+2 survives)
 *   negative  bool      I  DVSIGN is -0 (negative quotient)
 *   ovf       int *     O  OVFIND, set to 1
 *
 * RETURNS: agc_w3_t -- (+-POSMAX, +-POSMAX, MPAC+2)
 **********************************************************************/
static agc_w3_t ddv_overflow(agc_w3_t m, bool negative, int *ovf)
{
    *ovf = 1;                                       /* CAF ONE; TS OVFIND */
    m.m0 = negative ? -AGC_W_POSMAX : AGC_W_POSMAX; /* CAF POSMAX; TS MPAC */
    m.m1 = m.m0;                                    /* FINALDV +3     */

    return (m);
}

/**********************************************************************
 * BUFPOS +6 / BUFNEG (INTERPRETER.agc p.1165) -- a divisor whose major
 * word is +-1 with a minor word of the other sign is made to agree,
 * which leaves its major word 0 (BUFZERO follows). Every other divisor
 * keeps its words, disagreeing or not.
 *
 * ARGUMENTS:
 *   b  agc_w2_t  I  BUF words (divisor)
 *
 * RETURNS: agc_w2_t -- BUF after the forcing
 **********************************************************************/
static agc_w2_t ddv_unit_major(agc_w2_t b)
{
    if ((b.hi == 1) && (b.lo < 0))
    { /* CA HALF; DOUBLE; ADS BUF +1; TS BUF */
        b.hi = 0;
        b.lo += AGC_W_ONE;
    }
    else if ((b.hi == -1) && (b.lo > 0))
    { /* CS HALF; TCF BUFPOS +6           */
        b.hi = 0;
        b.lo -= AGC_W_ONE;
    }
    else
    {
        /* signs agree or |major| > 1: words kept */
    }

    return (b);
}

/**********************************************************************
 * BUFZERO (INTERPRETER.agc p.1162): the divisor's major word is 0, so
 * MPAC+2 := 0, TPAGREE, overflow unless the dividend's major word is 0,
 * then both shift left 14 and a zero divisor overflows.
 *
 * ARGUMENTS:
 *   m     agc_w3_t *  IO MPAC (dividend), shifted on return
 *   b     agc_w2_t *  IO BUF (divisor), shifted on return
 *   neg   bool *      IO DVSIGN on the overflow exits
 *   ovf   int *       O  OVFIND
 *
 * RETURNS: bool -- true when DVOVF was taken (*m holds the result)
 **********************************************************************/
static bool ddv_bufzero(agc_w3_t *m, agc_w2_t *b, bool *neg, int *ovf)
{
    m->m2 = 0;                 /* TS MPAC +2       */
    *m = agc_w_tpagree(*m); /* TC TPAGREE       */

    if (m->m0 != 0)
    { /* OVF+ / OVF+ -1: INCR DVSIGN when negative; SGNDVOVF on -BUF+1 */
        *neg = (m->m0 < 0) != (b->lo < 0);
        *m = ddv_overflow(*m, *neg, ovf);

        return (true);
    }

    m->m0 = m->m1; /* XCH BUF +1 .. XCH MPAC: shift left 14 */
    m->m1 = 0;
    b->hi = b->lo;
    b->lo = 0;

    if (b->hi == 0)
    { /* zero divisor: CS MPAC; SGNDVOVF    */
        *neg = (m->m0 < 0);
        *m = ddv_overflow(*m, *neg, ovf);

        return (true);
    }

    return (false);
}

/**********************************************************************
 * Sign agreement of a positive pair (MPAC+ p.1164): CAF HALF; DOUBLE;
 * AD MPAC +1; TS MPAC +1; CAF ZERO; AD POSMAX; ADS MPAC -- a negative
 * minor word borrows one from the major word.
 *
 * ARGUMENTS:
 *   p  agc_w2_t  I  pair with p.hi > 0, or p.hi = 0 and p.lo >= 0
 *
 * RETURNS: agc_w2_t -- the same value with both words >= 0
 **********************************************************************/
static agc_w2_t ddv_agree(agc_w2_t p)
{
    if (p.lo < 0)
    {
        p.hi -= 1;
        p.lo += AGC_W_ONE;
    }

    return (p);
}

/**********************************************************************
 * DVNORM, BUFNORM, the DDOUBL prologue and MAXTEST (INTERPRETER.agc
 * pp.1165-1166): double divisor and dividend word by word (DAS) until
 * the divisor's major word is >= HALF, then GENDDV or MAXDV.
 *
 * ARGUMENTS:
 *   a      agc_w2_t  I  dividend words (MPAC, MPAC+1), positive
 *   b      agc_w2_t  I  divisor words (BUF), major word > a's
 *                       (or equal with maxdv)
 *   maxdv  bool      I  MAXDVSW = +0 (majors were equal)
 *
 * RETURNS: agc_w2_t -- |quotient| words
 **********************************************************************/
static agc_w2_t ddv_normalise(agc_w2_t a, agc_w2_t b, bool maxdv)
{
    int k;     /* BUFNORM passes (DVNORMCT) */
    int o = 0; /* DAS overflow (none: |dividend| < |divisor|) */

    for (k = 0; (k < DV_NORM_MAX) && (b.hi < AGC_W_HALF); k++)
    {
        b = agc_w_das(b, b, &o); /* DCA BUF; DAS BUF */
        a = agc_w_das(a, a, &o); /* DDOUBL           */
    }

    return (agc_w_genddv(a, b, maxdv)); /* MAXTEST          */
}

/**********************************************************************
 * MPAC+ (INTERPRETER.agc p.1164) -- with both operands positive: if the
 * divisor's major word exceeds the dividend's by two or more, divide at
 * once (words unagreed); otherwise force sign agreement in both and
 * overflow unless the dividend is smaller (MAXDVSW = +0 for equal
 * major words).
 *
 * ARGUMENTS:
 *   a    agc_w2_t  I  dividend words, positive
 *   b    agc_w2_t  I  divisor words, positive
 *   ovf  int *     O  set to 1 for DVOVF (|a| >= |b|)
 *
 * RETURNS: agc_w2_t -- |quotient| words ({0, 0} on overflow)
 **********************************************************************/
static agc_w2_t ddv_quotient(agc_w2_t a, agc_w2_t b, int *ovf)
{
    agc_w2_t zero = {0, 0};
    bool maxdv = false; /* MAXDVSW = +0 */

    if (b.hi - a.hi - 1 <= 0)
    { /* CS MPAC; AD NEGONE; AD BUF; CCS A */
        a = ddv_agree(a);
        b = ddv_agree(b);

        if ((b.hi < a.hi) || ((b.hi == a.hi) && (b.lo <= a.lo)))
        { /* DVOVF            */
            *ovf = 1;

            return (zero);
        }

        maxdv = (b.hi == a.hi); /* TS MAXDVSW       */
    }

    return (ddv_normalise(a, b, maxdv));
}

/**********************************************************************
 * DDV / BDDV (INTERPRETER.agc pp.1162-1169) on the words: dividend in
 * MPAC, MPAC+1, divisor in BUF, BUF+1. BUFZERO, the divisor and dividend
 * sign forcing (BUF-, MPAC-), the overflow test (sign agreement only
 * when the major words are within one), normalisation, GENDDV/MAXDV and
 * FINALDV's sign. MPAC+2 is 0 after a divide, untouched by an overflow
 * or a zero dividend (except BUFZERO, which zeroes it first).
 *
 * ARGUMENTS:
 *   m           agc_w3_t  I  MPAC (dividend; MPAC+2 only carried)
 *   b           agc_w2_t  I  BUF (divisor), same scale
 *   ovf         int *     O  OVFIND: 1 on overflow, else 0
 *   dvsign_neg  bool *    O  DVSIGN left at -0 (quotient negative)
 *
 * RETURNS: agc_w3_t -- MPAC after the divide, quotient scaled 2^0;
 *          (+-POSMAX, +-POSMAX) on overflow
 **********************************************************************/
agc_w3_t agc_w_ddv(agc_w3_t m, agc_w2_t b, int *ovf, bool *dvsign_neg)
{
    bool neg = false; /* DVSIGN: -1 (false) / -0 (true) / +1 (false) */
    agc_w2_t a;           /* |dividend| words */
    agc_w2_t q;           /* |quotient| words */

    *ovf = 0;
    b = ddv_unit_major(b); /* CCS BUF          */

    if ((b.hi == 0) && ddv_bufzero(&m, &b, &neg, ovf))
    {
        *dvsign_neg = neg;

        return (m);
    }

    if (b.hi < 0)
    { /* BUF-             */
        b = agc_w_dcs(b);
        neg = !neg;
    }

    *dvsign_neg = neg;
    a = agc_w3_top(m);

    if (!((a.hi == 0) && (a.lo == 0))) /* zero dividend: TCF DANZIG */
    {
        if ((a.hi < 0) || ((a.hi == 0) && (a.lo < 0)))
        { /* MPAC-            */
            a = agc_w_dcs(a);
            neg = !neg;
            *dvsign_neg = neg;
        }

        q = ddv_quotient(a, b, ovf);

        if (*ovf != 0)
        {
            return (ddv_overflow(m, neg, ovf));
        }

        m.m0 = neg ? -q.hi : q.hi; /* FINALDV +3: DCS on DVSIGN = -0 */
        m.m1 = neg ? -q.lo : q.lo; /* DCS on -0 too */
        m.m2 = 0;                  /* TS MPAC +2       */
    }

    return (m);
} /* agc_w_ddv */

/* ---- SQRTSUB ------------------------------------------------------------- */

/**********************************************************************
 * SMPAC+ / SRTEST shift (INTERPRETER.agc pp.1185-1186) -- shift a
 * positive DP pair right one: DXCH MPAC; LXCH SR; MP HALF; DXCH MPAC;
 * XCH SR; ADS MPAC +1 (SR truncates the minor word).
 *
 * ARGUMENTS:
 *   p  agc_w2_t  I  MPAC, MPAC+1, major word > 0
 *
 * RETURNS: agc_w2_t -- p / 2
 **********************************************************************/
static agc_w2_t sqrt_half(agc_w2_t p)
{
    agc_w2_t r; /* p / 2 */

    r.hi = agc_w_mp(p.hi, AGC_W_HALF).hi;
    r.lo = agc_w_mp(p.hi, AGC_W_HALF).lo + p.lo / 2; /* SR edits: trunc  */

    return (r);
}

/**********************************************************************
 * ARGHI +4 (INTERPRETER.agc p.1185) -- two Newton steps from the linear
 * guess: X1 = X0/2 + .5 (MPAC/2)/(X0/2) in SP, then X2 = X1/2 +
 * (MPAC/2)/X1 in DP; an overflow (argument near POSMAX) goes to FIXROOT.
 *
 * ARGUMENTS:
 *   m    agc_w2_t  I  MPAC/2 (normalised argument), 0 < m.hi < x0h
 *   x0h  int32_t   I  X0/2, the linear first guess
 *
 * RETURNS: agc_w2_t -- the normalised root (major word ~.5)
 **********************************************************************/
static agc_w2_t sqrt_newton(agc_w2_t m, int32_t x0h)
{
    int32_t buf = x0h; /* BUF: X0/2, then X1 */
    int32_t q;         /* (MPAC/2)/(X0/2), SP */
    agc_w2_t x1h;       /* X1/2 as DP, then X2 */
    agc_w2_t qr;        /* DP quotient major word, remainder */
    agc_w2_t add;       /* (MPAC/2)/X1 as a DP pair */
    int o = 0;   /* DAS overflow (argument near POSMAX) */

    q = agc_w_dv(m.hi, 0, buf).hi;   /* (MPAC/2)/(X0/2)  */
    buf += agc_w_mp(q, AGC_W_HALF).hi; /* X1 = ...         */
    x1h = agc_w_mp(buf, AGC_W_HALF);   /* X1/2 as DP       */
    qr = agc_w_dv(m.hi, m.lo, buf);   /* DV BUF           */
    add.hi = qr.hi;
    add.lo = agc_w_dv(qr.lo, 0, buf).hi;  /* minor quotient   */
    x1h = agc_w_das(x1h, add, &o);     /* X2 = X1/2 + ..   */

    if (o != 0)
    { /* near POSMAX      */
        x1h.hi = AGC_W_POSMAX; /* FIXROOT          */
        x1h.lo = AGC_W_POSMAX;
    }

    return (x1h);
}

/**********************************************************************
 * ARGHI (INTERPRETER.agc p.1185) -- linear guess for an argument in
 * [.25, .5): X0/2 = (MPAC/2) SLOPEHI + BIASHI/2.
 *
 * ARGUMENTS:
 *   m  agc_w2_t  I  argument pair, major word in [QUARTER, HALF)
 *
 * RETURNS: agc_w2_t -- the normalised root from sqrt_newton
 **********************************************************************/
static agc_w2_t sqrt_arghi(agc_w2_t m)
{
    return (sqrt_newton(m, agc_w_mp(SQ_SLOPEHI, m.hi).hi + SQ_BIASHI));
}

/**********************************************************************
 * ARGLO (INTERPRETER.agc p.1186) -- linear guess for an argument in
 * [.125, .25): X0/2 = (MPAC/2) SLOPELO + BIASLO/2.
 *
 * ARGUMENTS:
 *   m  agc_w2_t  I  argument pair, major word in [QUARTER/2, QUARTER)
 *
 * RETURNS: agc_w2_t -- the normalised root from sqrt_newton
 **********************************************************************/
static agc_w2_t sqrt_arglo(agc_w2_t m)
{
    return (sqrt_newton(m, agc_w_mp(SQ_SLOPELO, m.hi).hi + SQ_BIASLO));
}

/**********************************************************************
 * SMPAC+ / SRTEST / SQRTNORM / NORMTEST (INTERPRETER.agc pp.1185-1186)
 * -- select ARGHI or ARGLO, normalising by two places at a time (one
 * right-shift count per pass). The root goes to MPAC, MPAC+1; MPAC+2
 * keeps what the normalisation left in it (the SMPAC+/SRTEST halving
 * does not touch it).
 *
 * ARGUMENTS:
 *   m      agc_w3_t  I  TP MPAC, major word > 0
 *   shift  int *     IO MPTEMP: right shifts owed to the root
 *
 * RETURNS: agc_w3_t -- the normalised root and MPAC+2
 **********************************************************************/
static agc_w3_t sqrt_positive(agc_w3_t m, int *shift)
{
    int pass;  /* NORMTEST passes (two places each) */
    int o = 0; /* TP shift overflow (never: |m0| < 1/4) */
    agc_w2_t r;     /* the root (MPAC, MPAC+1) */

    if (m.m0 >= AGC_W_QUARTER)
    { /* >= .25: halve, ARGHI (>= .5) or ARGLO */
        r = sqrt_half(agc_w3_top(m));
        r = (m.m0 >= AGC_W_HALF) ? sqrt_arghi(r) : sqrt_arglo(r); /* SMPAC+ ARGHI, SRTEST ARGLO */
    }
    else
    {
        *shift += 1; /* SQRTNORM         */
        m = agc_w_tp_shl1(m, &o);

        for (pass = 0; (pass < SQ_NORM_MAX) && (m.m0 < AGC_W_QUARTER / 2); pass++)
        { /* NORMTEST (CYL)   */
            m = agc_w_tp_shl1(m, &o); /* SQRTNM2          */
            *shift += 1;
            m = agc_w_tp_shl1(m, &o);
        }

        r = (m.m0 >= AGC_W_QUARTER) ? sqrt_arghi(agc_w3_top(m)) : sqrt_arglo(agc_w3_top(m)); /* NORMTEST: CYL */
    }

    m.m0 = r.hi;
    m.m1 = r.lo;

    return (m);
} /* sqrt_positive */

/**********************************************************************
 * SQRTSUB (INTERPRETER.agc pp.1184-1186) -- normalised square root of
 * the TP MPAC; the caller unnormalises by *shift (MPTEMP).
 *
 * ARGUMENTS:
 *   m      agc_w3_t  I  TP MPAC
 *   shift  int *     O  MPTEMP: right shifts owed (7 per whole word)
 *   abort  bool *    O  true for SQRTABRT (POODOO 01302): argument
 *                       below -2^-14 (SQRTNEG)
 *
 * RETURNS: agc_w3_t -- the root in m0, m1 (major word ~.5; 0 for a zero
 *          or small negative argument, ZEROANS) and the MPAC+2 SQRTSUB
 *          leaves in m2
 **********************************************************************/
agc_w3_t agc_w_sqrtsub(agc_w3_t m, int *shift, bool *abort)
{
    agc_w3_t zero = {0, 0, 0};
    int k; /* whole-word left shifts done */

    *shift = 0;
    *abort = false;

    if (m.m0 < 0)
    { /* SQRTNEG          */
        *abort = (m.m0 < -1) || (m.m1 < 0); /* POODOO 01302     */
        zero.m2 = m.m2;                      /* FIXROOT: MPAC, MPAC+1 only */

        return (zero); /* ZEROANS          */
    }

    for (k = 0; (k < 2) && (m.m0 == 0); k++)
    { /* shift left 14    */
        m.m0 = m.m1;
        m.m1 = m.m2;
        m.m2 = 0;
        *shift += 7; /* CAF SEVEN        */

        if (m.m0 < 0)
        {
            return (zero); /* ZEROANS          */
        }
    }

    if (m.m0 == 0)
    {
        return (zero); /* SQRT(0) = 0      */
    }

    return (sqrt_positive(m, shift));
} /* agc_w_sqrtsub */

/**********************************************************************
 * TPAGREE (INTERPRETER.agc p.1141) -- force sign agreement among the
 * three MPAC words (BRANCH picks the sign; +-1.0 is added and carried
 * through MPAC+2, MPAC+1, MPAC). The value is unchanged.
 *
 * ARGUMENTS:
 *   m  agc_w3_t  I  TP MPAC, words of any signs
 *
 * RETURNS: agc_w3_t -- the same value in sign-agreeing words
 **********************************************************************/
agc_w3_t agc_w_tpagree(agc_w3_t m)
{
    agc_tp_t v = agc_w3_value(m); /* value of the three words, 2^-42 per unit */

    return (agc_w3_from_tp(v));
}
