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
    if (!AGC_ASSERT(c > 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT((a > -c) && (a < c)))
    {
        return (0); /* no valid result */
    }

    return (agc_w_dv(a, 0, c).hi);
}

/**********************************************************************
 * GENDDV / MAXDV (INTERPRETER.agc pp.1166-1170) -- positive normalised
 * DP divide with MAXTEST's choice: GENDDV computes (A + sB)/(C + sD) =
 * Q + s(R - QD)/C, s = 2^-14, correcting Q by one either way; MAXDV
 * (equal major words, MAXDVSW = +0) gives 37777 + s(C + B - D)/C.
 *
 * ARGUMENTS:
 *   dividend  agc_dp_t  I  (A, B), >= 0, A <= C, 2^-28 per unit
 *   divisor   agc_dp_t  I  (C, D), C >= HALF (UNIT may pass 17777)
 *   maxdv     bool      I  MAXDVSW is +0: take MAXDV when A = C
 *
 * RETURNS: agc_dp_t -- the quotient, scaled 2^0 (< 1)
 **********************************************************************/
agc_dp_t agc_w_genddv(agc_dp_t dividend, agc_dp_t divisor, bool maxdv)
{
    agc_w2_t ab = agc_w_split(dividend); /* (A, B) */
    agc_w2_t cd = agc_w_split(divisor); /* (C, D) */
    int32_t  acc; /* MAXDV minor dividend C + B - D */

    /* The divisor is normally >= .5; UNIT's SQRTSUB root can be one LSB
     * below .5 (BUF 17777), which the AGC divides by all the same.        */

    if (!AGC_ASSERT(dividend >= 0) || !AGC_ASSERT(cd.hi > AGC_W_QUARTER))
    {
        return (AGC_DP_MAX);
    }

    if (!AGC_ASSERT(ab.hi <= cd.hi)) /* A <= C (MAXDV)  */
    {
        return (AGC_DP_MAX);
    }

    if (!maxdv || (ab.hi != cd.hi))
    { /* MAXDV: BZF +2   */
        /* GENDDV (p.1167-1168): (A + sB)/(C + sD) = Q + s(R - QD)/C, s = 2^-14.
         * Dividend (A,B) positive with A < C; divisor (C,D) positive, C >= HALF.  */
        agc_w2_t qr        = agc_w_dv(ab.hi, ab.lo, cd.hi); /* DV BUF          */
        int32_t  q         = qr.hi; /* Q, major quotient word */
        int32_t  acc_words = qr.lo + agc_w_mp(-q, cd.lo).hi; /* CS; MP; AD      */

        if (!AGC_ASSERT(ab.hi < cd.hi))
        {
            return (0); /* no valid result */
        }

        if (acc_words >= AGC_W_ONE)
        {                 /* OVSK: + overflow */
            acc_words -= cd.hi; /* SU BUF          */
            q         += 1; /* INCR MPAC       */
        }
        else if (acc_words <= 0)
        { /* BZMF -UP        */
            if (acc_words == 0)
            {
                return ((agc_dp_t) (q * AGC_W_ONE)); /* FINALDV +3      */
            }

            q         -= 1; /* DIM MPAC        */
            acc_words += cd.hi; /* ENDMAXDV AD BUF */

            return ((agc_dp_t) (q * AGC_W_ONE + finaldv(acc_words, cd.hi)));
        }
        else
        {
            /* fall into +DOWN */
        }

        if (acc_words - cd.hi >= 0)
        { /* +DOWN: SU BUF   */
            acc_words -= cd.hi;
            q         += 1; /* +3: INCR MPAC   */
        }

        if (!AGC_ASSERT(agc_w_ok(q)))
        {
            return (0); /* no valid result */
        }

        return ((agc_dp_t) (q * AGC_W_ONE + finaldv(acc_words, cd.hi)));
    } /* if (!maxdv || (ab.hi != cd.hi)) */

    /* MAXDV: (C + sB)/(C + sD) = 37777 + s(C + B - D)/C, B < D.             */
    acc = ab.lo - cd.lo + cd.hi; /* CS; AD; AD BUF  */

    return ((agc_dp_t) (AGC_W_POSMAX * AGC_W_ONE + finaldv(acc, cd.hi)));
} /* agc_w_genddv */

/* ---- DDV / BDDV ---------------------------------------------------------- */

/**********************************************************************
 * DVOVF (INTERPRETER.agc p.1162) -- DP MPAC := +-DPPOSMAX (FINALDV +3
 * applies DVSIGN), OVFIND := 1.
 *
 * ARGUMENTS:
 *   negative  bool   I  DVSIGN is -0 (negative quotient)
 *   ovf       int *  O  OVFIND, set to 1
 *
 * RETURNS: agc_dp_t -- +DPPOSMAX, or -DPPOSMAX when negative
 **********************************************************************/
static agc_dp_t ddv_overflow(bool negative, int *ovf)
{
    if (!AGC_ASSERT(ovf != 0))
    {
        return (AGC_DP_MAX);
    }

    *ovf = 1;                                  /* CAF ONE; TS OVFIND */

    return (negative ? AGC_DP_MIN : AGC_DP_MAX); /* DVOVF: DPPOSMAX */
}

/**********************************************************************
 * DDV / BDDV (INTERPRETER.agc pp.1162-1165) -- a / b with both scaled
 * alike, quotient scaled 2^0: BUFZERO (shift 14 when the divisor's major
 * word is 0), sign forcing (BUF-, MPAC-), the overflow test, divisor
 * normalisation (BUFNORM) and GENDDV/MAXDV.
 *
 * ARGUMENTS:
 *   a           agc_dp_t  I  dividend (MPAC)
 *   b           agc_dp_t  I  divisor (BUF), same scale as a
 *   ovf         int *     O  OVFIND: 1 on overflow, else 0
 *   dvsign_neg  bool *    O  DVSIGN left at -0 (quotient negative, also
 *                            for a zero dividend over a negative divisor)
 *
 * RETURNS: agc_dp_t -- a / b; +-DPPOSMAX on overflow (|a| >= |b|, b = 0)
 **********************************************************************/
agc_dp_t agc_w_ddv(agc_dp_t a, agc_dp_t b, int *ovf, bool *dvsign_neg)
{
    agc_w2_t w     = agc_w_split(a); /* MPAC words of the dividend   */
    agc_w2_t d     = agc_w_split(b); /* BUF words of the divisor     */
    bool     neg   = false;          /* quotient negative (DVSIGN)   */
    bool     maxdv = false;          /* MAXDVSW: equal major words   */
    agc_dp_t q;                      /* |quotient|, scaled 2^0       */
    int      k;                      /* BUFNORM doublings done       */

    if (!AGC_ASSERT((ovf != 0) && (dvsign_neg != 0)))
    {
        return (AGC_DP_MAX);
    }

    *ovf = 0;

    if (d.hi == 0)
    { /* BUFZERO          */
        if (w.hi != 0)
        { /* OVF+ / SGNDVOVF  */
            *dvsign_neg = (w.hi < 0) != (d.lo < 0);

            return (ddv_overflow(*dvsign_neg, ovf));
        }

        a *= AGC_W_ONE; /* shift both 14    */
        b *= AGC_W_ONE;

        if (b == 0)
        { /* zero divisor     */
            *dvsign_neg = a < 0;

            return (ddv_overflow(a < 0, ovf));
        }
    }

    if (b < 0)
    { /* BUF-             */
        b   = -b;
        neg = !neg;
    }

    *dvsign_neg = neg; /* -1, or -0 after BUF- */

    if (a == 0)
    { /* exit on zero     */
        return (0);
    }

    if (a < 0)
    { /* MPAC-            */
        a           = -a;
        neg         = !neg;
        *dvsign_neg = neg; /* INCR DVSIGN */
    }

    w = agc_w_split(a);
    d = agc_w_split(b);

    if ((d.hi - w.hi < 0) || ((d.hi == w.hi) && (d.lo - w.lo <= 0)))
    {
        return (ddv_overflow(neg, ovf)); /* MPAC+: DVOVF     */
    }

    /* Equal majors leave -0 in A; CCS turns it into +0 for TS MAXDVSW.    */
    maxdv = (d.hi == w.hi);

    /* Normalise the divisor (BUFNORM/DVNORM) and the dividend by the same
     * count (the DDOUBL prologue indexed by DVNORMCT), then GENDDV/MAXDV. */
    if (!AGC_ASSERT((a > 0) && (b > 0)))
    {
        return (AGC_DP_MAX);
    }

    for (k = 0; (k < DV_NORM_MAX) && (agc_w_split(b).hi < AGC_W_HALF); k++)
    {
        b *= 2; /* DCA BUF; DAS BUF */
        a *= 2; /* DDOUBL           */
    }

    if (!AGC_ASSERT(agc_w_split(b).hi >= AGC_W_HALF))
    {
        return (AGC_DP_MAX);
    }

    if (!AGC_ASSERT(a <= b)) /* no DDOUBL overflow */
    {
        return (AGC_DP_MAX);
    }

    q = agc_w_genddv(a, b, maxdv);

    if (!AGC_ASSERT((q >= 0) && (q <= AGC_DP_MAX)))
    {
        return (AGC_DP_MAX);
    }

    return (neg ? -q : q); /* DVSIGN: FINALDV +3 */
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

    if (!AGC_ASSERT(p.hi > 0))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    r.hi = agc_w_mp(p.hi, AGC_W_HALF).hi;
    r.lo = agc_w_mp(p.hi, AGC_W_HALF).lo + p.lo / 2; /* SR edits: trunc  */

    if (!AGC_ASSERT(agc_w_ok(r.lo)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

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
    int32_t  buf = x0h; /* BUF: X0/2, then X1 */
    int32_t  q; /* (MPAC/2)/(X0/2), SP */
    agc_w2_t x1h; /* X1/2 as DP, then X2 */
    agc_w2_t qr; /* DP quotient major word, remainder */
    agc_w2_t add; /* (MPAC/2)/X1 as a DP pair */
    int      o = 0; /* DAS overflow (argument near POSMAX) */

    if (!AGC_ASSERT((m.hi > 0) && (m.hi < buf)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    q      = agc_w_dv(m.hi, 0, buf).hi;  /* (MPAC/2)/(X0/2)  */
    buf   += agc_w_mp(q, AGC_W_HALF).hi; /* X1 = ...         */
    x1h    = agc_w_mp(buf, AGC_W_HALF); /* X1/2 as DP       */
    qr     = agc_w_dv(m.hi, m.lo, buf); /* DV BUF           */
    add.hi = qr.hi;
    add.lo = agc_w_dv(qr.lo, 0, buf).hi; /* minor quotient   */
    x1h    = agc_w_das(x1h, add, &o);    /* X2 = X1/2 + ..   */

    if (o != 0)
    {                          /* near POSMAX      */
        x1h.hi = AGC_W_POSMAX; /* FIXROOT          */
        x1h.lo = AGC_W_POSMAX;
    }

    if (!AGC_ASSERT(x1h.hi > AGC_W_QUARTER)) /* ~.5 (may be 17777) */
    {
        return ((agc_w2_t){0}); /* no valid result */
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
    if (!AGC_ASSERT(m.hi >= AGC_W_QUARTER))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(m.hi < AGC_W_HALF))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

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
    if (!AGC_ASSERT(m.hi >= AGC_W_QUARTER / 2))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(m.hi < AGC_W_QUARTER))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    return (sqrt_newton(m, agc_w_mp(SQ_SLOPELO, m.hi).hi + SQ_BIASLO));
}

/**********************************************************************
 * SMPAC+ / SRTEST / SQRTNORM / NORMTEST (INTERPRETER.agc pp.1185-1186)
 * -- select ARGHI or ARGLO, normalising by two places at a time (one
 * right-shift count per pass).
 *
 * ARGUMENTS:
 *   m      agc_w3_t  I  TP MPAC, major word > 0
 *   shift  int *     IO MPTEMP: right shifts owed to the root
 *
 * RETURNS: agc_w2_t -- the normalised root
 **********************************************************************/
static agc_w2_t sqrt_positive(agc_w3_t m, int *shift)
{
    int pass; /* NORMTEST passes (two places each) */
    int o = 0; /* TP shift overflow (never: |m0| < 1/4) */

    if (!AGC_ASSERT((m.m0 > 0) && (shift != 0)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    if (m.m0 >= AGC_W_HALF)
    { /* >= .5            */
        return (sqrt_arghi(sqrt_half(agc_w3_top(m))));
    }

    if (m.m0 >= AGC_W_QUARTER)
    { /* .25 .. .5        */
        return (sqrt_arglo(sqrt_half(agc_w3_top(m))));
    }

    *shift += 1; /* SQRTNORM         */
    m       = agc_w_tp_shl1(m, &o);

    for (pass = 0; pass < SQ_NORM_MAX; pass++)
    { /* NORMTEST (CYL)   */
        if (m.m0 >= AGC_W_QUARTER)
        {
            return (sqrt_arghi(agc_w3_top(m)));
        }

        if (m.m0 >= AGC_W_QUARTER / 2)
        {
            return (sqrt_arglo(agc_w3_top(m)));
        }

        m       = agc_w_tp_shl1(m, &o); /* SQRTNM2          */
        *shift += 1;
        m       = agc_w_tp_shl1(m, &o);
    }

    if (!AGC_ASSERT(o == 0))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    return (agc_w3_top(m));
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
 * RETURNS: agc_w2_t -- the root, major word ~.5; 0 for a zero or small
 *          negative argument (ZEROANS)
 **********************************************************************/
agc_w2_t agc_w_sqrtsub(agc_w3_t m, int *shift, bool *abort)
{
    agc_w2_t zero = {0, 0};
    int      k; /* whole-word left shifts done */

    if (!AGC_ASSERT(shift != 0) || !AGC_ASSERT(abort != 0))
    {
        return (zero);
    }

    *shift = 0;
    *abort = false;

    if (m.m0 < 0)
    {                                       /* SQRTNEG          */
        *abort = (m.m0 < -1) || (m.m1 < 0); /* POODOO 01302     */

        return (zero);                        /* ZEROANS          */
    }

    for (k = 0; (k < 2) && (m.m0 == 0); k++)
    { /* shift left 14    */
        m.m0    = m.m1;
        m.m1    = m.m2;
        m.m2    = 0;
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

    if (!AGC_ASSERT((*shift == 0) || (*shift == 7) || (*shift == 14)))
    {
        return (zero);
    }

    return (sqrt_positive(m, shift));
} /* agc_w_sqrtsub */
