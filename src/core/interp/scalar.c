/**********************************************************************
 * FILE NAME: scalar.c
 *
 * PURPOSE:
 *   Scalar interpretive arithmetic: interpreter state (OVFIND, BOV),
 *   DAD/DSU/TAD, DMP/DMPR/DSQ, SHORTMP, DMPNSUB, POLY, DDV/BDDV, SL/SLR,
 *   TP shifts, NORM and ROUND, each on the AGC's own word arithmetic.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc pp.1131-1142, 1149, 1152-1161, 1180,
 *   1196 (DAD, DSU, BDSU, TAD, DMPSUB, ROUNDSUB, DMP, DMPR, SHORTMP,
 *   DMPNSUB, POLY, DDV, BDDV, SL, SLR, SR, TSSL, TSLC, ROUND, DSQ, BOV)
 *   Luminary099 INTERPRETER.agc pp.1026-1037, 1044, 1047-1056, 1075,
 *   1091 (identical)
 *
 * EXTERNAL VARIABLES:
 *   none (OVFIND and DVSIGN live in the caller's agc_interp_t)
 *
 * EXTERNAL REFERENCES:
 *   Name                Description
 *   ------------------  --------------------------------------------
 *   agc_w_*             word-level MPAC model (mpac.c, shift.c,
 *                       divide.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; overflow sets OVFIND exactly where the AGC sets it and is
 *   never cleared here (BOV, agc_interp_test_ovf, clears it).
 *
 * NOTES:
 *   Each function loads its agc_dp_t / agc_tp_t operands into AGC words
 *   (agc_w_split), runs the AGC routine on the words and returns the
 *   value of the resulting MPAC words. Every function here is checked
 *   bit for bit against yaAGC (tests/oracle/core_numeric.c).
 **********************************************************************/
#include "../mpac.h"

#define POLY_MAX_DEGREE 15 /* loop bound on N; the ropes use N <= 9 */

/* ---- interpreter state --------------------------------------------------- */

/**********************************************************************
 * AGC: none (C state) -- interpreter state at fresh start: OVFIND,
 * index registers and switches clear; DVSIGN as a zeroed erasable (+0,
 * so an early DVOVF gives +DPPOSMAX).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  O  state to initialise
 **********************************************************************/
void agc_interp_init(agc_interp_t *ip)
{
    int k; /* switch index */

    ip->ovfind = 0;
    ip->mpac   = 0;
    ip->x1     = 0;
    ip->x2     = 0;
    ip->s1     = 0;
    ip->s2     = 0;

    /* DVSIGN as a zeroed erasable leaves it (+0): DVOVF gives +DPPOSMAX.  */
    ip->dvsign_neg = false;

    for (k = 0; k < AGC_INTERP_SWITCHES; k++)
    {
        ip->sw[k] = false;
    }
}

/**********************************************************************
 * BOV / BOVB (INTERPRETER.agc p.1196 "BOV(B)") -- CCS OVFIND; branch
 * and clear if set.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, cleared
 *
 * RETURNS: bool -- true when OVFIND was set (the branch is taken)
 **********************************************************************/
bool agc_interp_test_ovf(agc_interp_t *ip)
{
    bool set; /* OVFIND was set */

    set        = (ip->ovfind != 0);
    ip->ovfind = 0;

    return (set);
}

/**********************************************************************
 * BOV (INTERPRETER.agc p.1196) used only to reset OVFIND (the AGC idiom
 * "BOV +1", or a BOV whose target is the next instruction).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, cleared
 **********************************************************************/
void agc_interp_clear_ovf(agc_interp_t *ip)
{
    ip->ovfind = 0;
}

/* ---- add / subtract (p.1131-1135) ---------------------------------------- */

/**********************************************************************
 * DAD (INTERPRETER.agc pp.1131-1133) -- DAS MPAC; OVERFLOW if A is
 * non-zero (ENDVXV). Wraps on overflow (value -+ 1.0).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   a   agc_dp_t        I  MPAC
 *   b   agc_dp_t        I  operand, same scale
 *
 * RETURNS: agc_dp_t -- a + b, overflow-corrected
 **********************************************************************/
agc_dp_t agc_dp_add(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    return (agc_w_join(agc_w_dad(agc_w_split(a), agc_w_split(b), &ip->ovfind)));
}

/**********************************************************************
 * DAD then BOV (INTERPRETER.agc pp.1131-1133) with the sum never read
 * (MPAC is reloaded next): only OVFIND changes.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   a   agc_dp_t        I  MPAC
 *   b   agc_dp_t        I  operand, same scale
 **********************************************************************/
void agc_dp_add_ovf_only(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    ip->mpac = agc_dp_add(ip, a, b); /* DAD: MPAC (not read), OVFIND */
}

/**********************************************************************
 * DSU (INTERPRETER.agc pp.1131-1133) -- DCS 0 then ENDVXV. BDSU X is
 * agc_dp_sub(ip, X, mpac).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   a   agc_dp_t        I  minuend (MPAC)
 *   b   agc_dp_t        I  subtrahend, same scale
 *
 * RETURNS: agc_dp_t -- a - b, overflow-corrected
 **********************************************************************/
agc_dp_t agc_dp_sub(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    return (agc_w_join(agc_w_dad(agc_w_split(a), agc_w_split(-b), &ip->ovfind)));
}

/**********************************************************************
 * TAD (INTERPRETER.agc p.1135) -- minor parts by DAS, then AD AD TS on
 * MPAC; SETOVF -> OVERFLOW on MPAC, MPAC+1 when the major word
 * overflows.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   a   agc_tp_t        I  TP MPAC
 *   b   agc_tp_t        I  TP operand, same scale
 *
 * RETURNS: agc_tp_t -- a + b, overflow-corrected
 **********************************************************************/
agc_tp_t agc_tp_add(agc_interp_t *ip, agc_tp_t a, agc_tp_t b)
{
    agc_w3_t m   = agc_w3_from_tp(a); /* TP MPAC of a */
    agc_w3_t x   = agc_w3_from_tp(b); /* TP operand words */
    agc_w2_t lo2 = {m.m1, m.m2};
    agc_w2_t add = {x.m1, x.m2};
    agc_w2_t top;                     /* MPAC, MPAC+1 after OVERFLOW */
    int      o   = 0;                 /* DAS / TS overflow, +-1 or 0 */

    lo2  = agc_w_das(lo2, add, &o);       /* DCA 1; DAS +1    */
    m.m1 = lo2.hi;
    m.m2 = lo2.lo;
    m.m0 = agc_w_ts(o + x.m0 + m.m0, &o); /* AD 0; AD; TS     */

    if (o != 0)
    {
        top  = agc_w_overflow(agc_w3_top(m), o, &ip->ovfind); /* SETOVF   */
        m.m0 = top.hi;
        m.m1 = top.lo;
    }

    return (agc_w3_value(m));
}

/* ---- multiply (p.1136-1137, 1141-1142, 1149, 1183) ----------------------- */

/**********************************************************************
 * DMPR (INTERPRETER.agc p.1149) -- DMPSUB; ROUNDSUB (OVFIND on the rare
 * carry out).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set on a rounding carry out
 *   a   agc_dp_t        I  MPAC
 *   b   agc_dp_t        I  operand; scales add
 *
 * RETURNS: agc_dp_t -- a b rounded half away from zero
 **********************************************************************/
agc_dp_t agc_dp_mul(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    agc_w3_t m; /* DMPSUB product, then rounded */

    m = agc_w_dmpsub(agc_w_split(a), agc_w_split(b));
    m = agc_w_roundsub(m, &ip->ovfind);

    return (agc_w_join(agc_w3_top(m)));
}

/**********************************************************************
 * DMP (DMP1, INTERPRETER.agc p.1149) used as a DP -- MPAC, MPAC+1 of
 * the DMPSUB product (MPAC+2 dropped by the store).
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  MPAC
 *   b  agc_dp_t  I  operand; scales add
 *
 * RETURNS: agc_dp_t -- a b truncated to DP
 **********************************************************************/
agc_dp_t agc_dp_mul_trunc(agc_dp_t a, agc_dp_t b)
{
    agc_w3_t m = agc_w_dmpsub(agc_w_split(a), agc_w_split(b)); /* DMPSUB TP product */

    return (agc_w_join(agc_w3_top(m)));
}

/**********************************************************************
 * DSQ (INTERPRETER.agc p.1180) -- DSQSUB leaves the TP square; as a DP,
 * MPAC, MPAC+1.
 *
 * ARGUMENTS:
 *   a   agc_dp_t        I  MPAC
 *
 * RETURNS: agc_dp_t -- a^2 truncated, at twice the scale of a
 **********************************************************************/
agc_dp_t agc_dp_sq(agc_dp_t a)
{
    return (agc_w_join(agc_w3_top(agc_w_dsqsub(agc_w_split(a)))));
}

/**********************************************************************
 * DSQ (DSQSUB, INTERPRETER.agc p.1183) as the full TP MPAC.
 *
 * ARGUMENTS:
 *   a  agc_dp_t  I  MPAC
 *
 * RETURNS: agc_tp_t -- a^2, at twice the scale of a
 **********************************************************************/
agc_tp_t agc_dp_sq_tp(agc_dp_t a)
{
    agc_w3_t m = agc_w_dsqsub(agc_w_split(a)); /* DSQSUB TP square */

    return (agc_w3_value(m));
}

/**********************************************************************
 * SHORTMP / SHORTMP2 / DMPSUB2 (INTERPRETER.agc pp.1141, 1136) -- TP
 * MPAC times an SP fraction.
 *
 * ARGUMENTS:
 *   v  agc_tp_t  I  TP MPAC
 *   k  agc_sp_t  I  SP multiplier (A); scales add
 *
 * RETURNS: agc_tp_t -- v k (MPAC+2 keeps the major half of k m2)
 **********************************************************************/
agc_tp_t agc_tp_mul_sp(agc_tp_t v, agc_sp_t k)
{
    agc_w3_t m = agc_w3_from_tp(v); /* TP MPAC of v */
    agc_w2_t acc;                   /* DAS accumulator pair */
    int      o = 0;                 /* DAS overflow, the carry into MPAC */

    acc.hi = 0;                                     /* XCH MPAC +1      */
    acc.lo = agc_w_mp(k, m.m2).hi;                  /* TS MPAC +2       */
    acc    = agc_w_das(acc, agc_w_mp(m.m1, k), &o); /* DAS MPAC +1      */
    m.m1   = acc.hi;
    m.m2   = acc.lo;
    acc.hi = o;                                     /* XCH MPAC         */
    acc.lo = m.m1;
    acc    = agc_w_das(acc, agc_w_mp(m.m0, k), &o); /* DAS MPAC         */
    m.m0   = acc.hi;
    m.m1   = acc.lo;

    return (agc_w3_value(m));
}

/**********************************************************************
 * DMPNSUB (INTERPRETER.agc p.1142) -- DP fraction times an SP integer:
 * MPAC = (major(n m1) + minor(n m0), minor(n m1)).
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I  MPAC
 *   n  agc_sp_t  I  integer multiplier (A); the customer keeps |n v| < 1
 *
 * RETURNS: agc_dp_t -- n v, same scale as v
 **********************************************************************/
agc_dp_t agc_dp_mul_int(agc_dp_t v, agc_sp_t n)
{
    agc_w2_t m  = agc_w_split(v);    /* MPAC words of v */
    agc_w2_t lo = agc_w_mp(n, m.lo); /* MP MPAC +1       */
    agc_w2_t hi = agc_w_mp(n, m.hi); /* MP DMPNTEMP      */
    int      o  = 0;                 /* TS overflow (none: |n v| < 1) */

    lo.hi = agc_w_ts(lo.hi + hi.lo, &o); /* CA L; ADS MPAC   */

    return (agc_w_join(lo));
}

/**********************************************************************
 * POLY / POWRSERS (INTERPRETER.agc pp.1139-1140) -- Horner evaluation
 * coef[degree] x^degree + ... + coef[0]; x keeps only MPAC, MPAC+1.
 *
 * ARGUMENTS:
 *   x       agc_dp_t          I  argument
 *   coef    const agc_dp_t *  I  A(0) .. A(degree), at the result scale
 *   degree  int               I  N, 1..POLY_MAX_DEGREE
 *
 * RETURNS: agc_dp_t -- the polynomial value (MPAC, MPAC+1)
 **********************************************************************/
agc_dp_t agc_dp_poly(agc_dp_t x, const agc_dp_t *coef, int degree)
{
    return (agc_w_join(agc_w3_top(agc_w_poly(agc_w_split(x), coef, degree))));
}

/* ---- divide (p.1149, 1162-1170) ------------------------------------------ */

/**********************************************************************
 * DDV (INTERPRETER.agc pp.1149, 1162-1170) -- MPAC / X. BDDV X is
 * agc_dp_div(ip, X, mpac).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND (1 on overflow), DVSIGN
 *   a   agc_dp_t        I  dividend
 *   b   agc_dp_t        I  divisor, same scale
 *
 * RETURNS: agc_dp_t -- a / b scaled 2^0; +-DPPOSMAX on overflow
 **********************************************************************/
agc_dp_t agc_dp_div(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    int      o = 0; /* OVFIND of DDV: 1 on overflow */
    agc_dp_t q;     /* quotient, scaled 2^0 */

    q = agc_w_join(agc_w3_top(agc_w_ddv(agc_w3_from_tp(agc_tp_from_dp(a)), agc_w_split(b), &o, &ip->dvsign_neg)));

    if (o != 0)
    {
        ip->ovfind = 1; /* CAF ONE; TS OVFIND */
    }

    return (q);
}

/**********************************************************************
 * DDV then BOV (INTERPRETER.agc pp.1162-1170) with the quotient never
 * read: only OVFIND (and DVSIGN) change (OVFIND set when |a| >= |b|).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, DVSIGN
 *   a   agc_dp_t        I  dividend
 *   b   agc_dp_t        I  divisor, same scale
 **********************************************************************/
void agc_dp_div_ovf_only(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    ip->mpac = agc_dp_div(ip, a, b); /* DDV: MPAC (not read), OVFIND, DVSIGN */
}

/* ---- shifts and rounding (p.1137, 1152-1161) ----------------------------- */

/**********************************************************************
 * SL n (TSSL, INTERPRETER.agc p.1154) -- a negative count shifts right
 * (RIGHT-, p.1161).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set on each overflowing step
 *   a   agc_dp_t        I  MPAC
 *   n   int             I  shift count, -127..127
 *
 * RETURNS: agc_dp_t -- a 2^n, wrapped on overflow (scale -n)
 **********************************************************************/
agc_dp_t agc_dp_shl(agc_interp_t *ip, agc_dp_t a, int n)
{
    agc_w3_t m = agc_w3_from_tp(agc_tp_from_dp(a)); /* TP MPAC of a */

    if (n < 0)
    {
        return (agc_dp_shr(a, -n));
    }

    return (agc_w_join(agc_w3_top(agc_w_tssl(m, n, &ip->ovfind))));
}

/**********************************************************************
 * SLR n (TSSL then ROUNDSUB, INTERPRETER.agc pp.1154, 1137). For a DP
 * MPAC (MPAC+2 = 0 after DLOAD) the round adds nothing, so this equals
 * SL; kept separate for fidelity.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set on each overflowing step
 *   a   agc_dp_t        I  MPAC
 *   n   int             I  shift count, -127..127 (negative: SRR)
 *
 * RETURNS: agc_dp_t -- a 2^n rounded (scale -n)
 **********************************************************************/
agc_dp_t agc_dp_shl_round(agc_interp_t *ip, agc_dp_t a, int n)
{
    agc_w3_t m = agc_w3_from_tp(agc_tp_from_dp(a)); /* TP MPAC of a */

    if (n < 0)
    {
        return (agc_dp_shr_round(a, -n));
    }

    m = agc_w_tssl(m, n, &ip->ovfind);
    m = agc_w_roundsub(m, &ip->ovfind);

    return (agc_w_join(agc_w3_top(m)));
}

/**********************************************************************
 * SL n / SR -n on the TP MPAC (TSSL p.1154, GENSCR / MPACSHR p.1160).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set on an overflowing left step
 *   v   agc_tp_t        I  TP MPAC
 *   n   int             I  shift count, -127..127 (> 0 left)
 *
 * RETURNS: agc_tp_t -- v 2^n (right shifts truncate)
 **********************************************************************/
agc_tp_t agc_tp_shift(agc_interp_t *ip, agc_tp_t v, int n)
{
    agc_w3_t m = agc_w3_from_tp(v); /* TP MPAC of v */

    if (n == 0)
    {
        return (v);
    }

    if (n > 0)
    {
        m = agc_w_tssl(m, n, &ip->ovfind);
    }
    else
    {
        m = agc_w_genscr(m, -n, false);
    }

    return (agc_w3_value(m));
}

/**********************************************************************
 * NORM (TSLC, INTERPRETER.agc p.1157) on the TP MPAC.
 *
 * ARGUMENTS:
 *   v       agc_tp_t  I  TP MPAC
 *   nshift  int *     O  left shifts done (the AGC stores -nshift)
 *
 * RETURNS: agc_tp_t -- v left-justified (|major word| >= 1/2), or 0
 **********************************************************************/
agc_tp_t agc_tp_norm(agc_tp_t v, int *nshift)
{
    int      count = 0;                                     /* left shifts done */
    agc_w3_t m     = agc_w_tslc(agc_w3_from_tp(v), &count); /* normalised TP MPAC */

    *nshift = count;

    return (agc_w3_value(m));
}

/**********************************************************************
 * ROUND (unary op 07: TC ROUNDSUB, INTERPRETER.agc pp.1154, 1137).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set on a carry out of MPAC
 *   a   agc_tp_t        I  TP MPAC
 *
 * RETURNS: agc_dp_t -- a rounded half away from zero to DP
 **********************************************************************/
agc_dp_t agc_tp_round(agc_interp_t *ip, agc_tp_t a)
{
    agc_w3_t m = agc_w3_from_tp(a); /* TP MPAC of a */

    m = agc_w_roundsub(m, &ip->ovfind);

    return (agc_w_join(agc_w3_top(m)));
}
