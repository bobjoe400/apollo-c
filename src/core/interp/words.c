/**********************************************************************
 * FILE NAME: words.c
 *
 * PURPOSE:
 *   The word-level MPAC API (include/agc/core/interp_words.h): each
 *   interpretive instruction on MPAC, MPAC+1, MPAC+2 and on operands as
 *   their two stored words, with no sign agreement the AGC does not do
 *   itself. SQRT, SIN, COS, ASIN, ACOS are in trig.c.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc pp.1107-1108 (DLOAD), 1131-1141 (DAD,
 *   DSU, BDSU, TAD, DMPSUB, ROUNDSUB, TPAGREE), 1143 (DOT), 1149 (DMP,
 *   DMPR), 1151-1161 (SIGN, COMP, DCOMP, TSSL, GENSHIFT, GENSCR, TSLC),
 *   1162-1170 (DDV, BDDV), 1181 (ABS), 1183 (DSQ, VSQ)
 *   Luminary099 INTERPRETER.agc pp.1002-1003, 1026-1065, 1076-1078
 *   (identical)
 *
 * EXTERNAL VARIABLES:
 *   none (OVFIND and DVSIGN live in the caller's agc_interp_t)
 *
 * EXTERNAL REFERENCES:
 *   Name      Description
 *   --------  ------------------------------------------------------
 *   agc_w_*   word-level MPAC model (mpac.c, shift.c, divide.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; OVFIND is set where the AGC sets it, never cleared here.
 *
 * NOTES:
 *   The interp.h value functions run the same model on sign-agreeing
 *   words; these take the words as given. Checked word for word
 *   against yaAGC by tests/oracle/core_words.c.
 **********************************************************************/
#include "../mpac.h"
#include "agc/core/assert.h"
#include "agc/core/interp_words.h"

#define SHIFT_COUNT_MAX 127 /* 7-bit shift count of SL/SR/SLR/SRR */

/* ---- load / store -------------------------------------------------------- */

/**********************************************************************
 * DLOAD (INTERPRETER.agc p.1107) -- MPAC, MPAC+1 := the stored words,
 * MPAC+2 := 0.
 *
 * ARGUMENTS:
 *   x  agc_dp_words_t  I  the operand as stored
 *
 * RETURNS: agc_tp_words_t -- MPAC
 **********************************************************************/
agc_tp_words_t agc_mpac_dload(agc_dp_words_t x)
{
    agc_tp_words_t m = {x.hi, x.lo, 0};

    return (m);
}

/**********************************************************************
 * STORE (INTERPRETER.agc p.1117) in DP mode -- MPAC, MPAC+1 as they
 * are (no rounding, no sign agreement).
 *
 * ARGUMENTS:
 *   m  agc_tp_words_t  I  MPAC
 *
 * RETURNS: agc_dp_words_t -- the stored pair
 **********************************************************************/
agc_dp_words_t agc_mpac_store(agc_tp_words_t m)
{
    agc_dp_words_t x = {m.hi, m.mid};

    return (x);
}

/* ---- add ----------------------------------------------------------------- */

/**********************************************************************
 * AGC: none (C helper) -- MPAC, MPAC+1 replaced, MPAC+2 kept (the
 * DP-only instructions).
 *
 * ARGUMENTS:
 *   m    agc_w3_t  I  MPAC before
 *   top  agc_w2_t  I  new MPAC, MPAC+1
 *
 * RETURNS: agc_tp_words_t -- MPAC after
 **********************************************************************/
static agc_tp_words_t with_top(agc_w3_t m, agc_w2_t top)
{
    m.m0 = top.hi;
    m.m1 = top.lo;

    return (agc_w3_to_words(m));
}

/**********************************************************************
 * DAD (ENDVXV, INTERPRETER.agc p.1131) -- DAS MPAC; OVERFLOW if A is
 * non-zero (wraps, OVFIND = +-1). MPAC+2 is not touched.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND
 *   m   agc_tp_words_t    I  MPAC
 *   x   agc_dp_words_t    I  operand as stored
 *
 * RETURNS: agc_tp_words_t -- MPAC + x
 **********************************************************************/
agc_tp_words_t agc_mpac_dad(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x)
{
    agc_w3_t w = agc_w3_from_words(m); /* MPAC words */
    agc_w2_t top;                      /* MPAC, MPAC+1 after the add */

    top = agc_w_dad(agc_w3_top(w), agc_w2_from_words(x), &ip->ovfind);

    return (with_top(w, top));
}

/**********************************************************************
 * DSU (INTERPRETER.agc p.1133) -- DCS of the operand, then ENDVXV.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND
 *   m   agc_tp_words_t    I  MPAC
 *   x   agc_dp_words_t    I  operand as stored
 *
 * RETURNS: agc_tp_words_t -- MPAC - x
 **********************************************************************/
agc_tp_words_t agc_mpac_dsu(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x)
{
    agc_w3_t w = agc_w3_from_words(m); /* MPAC words */
    agc_w2_t top;                      /* MPAC, MPAC+1 after the add */

    top = agc_w_dad(agc_w3_top(w), agc_w_dcs(agc_w2_from_words(x)), &ip->ovfind);

    return (with_top(w, top));
}

/**********************************************************************
 * BDSU (INTERPRETER.agc p.1134) -- DCA operand; DXCH MPAC; DCOM; DAS
 * MPAC: the operand minus MPAC, word by word (R1A = D1/32 - MPAC).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND
 *   m   agc_tp_words_t    I  MPAC
 *   x   agc_dp_words_t    I  operand as stored
 *
 * RETURNS: agc_tp_words_t -- x - MPAC
 **********************************************************************/
agc_tp_words_t agc_mpac_bdsu(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x)
{
    agc_w3_t w = agc_w3_from_words(m); /* MPAC words */
    agc_w2_t top;                      /* MPAC, MPAC+1 after the add */

    top = agc_w_dad(agc_w2_from_words(x), agc_w_dcs(agc_w3_top(w)), &ip->ovfind);

    return (with_top(w, top));
}

/**********************************************************************
 * TAD (INTERPRETER.agc p.1135) -- DAS of the minor pairs into MPAC+1,
 * MPAC+2, then AD/AD/TS MPAC; OVERFLOW (SETOVF) on a carry out.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND
 *   m   agc_tp_words_t    I  MPAC
 *   x   agc_tp_words_t    I  TP operand as stored
 *
 * RETURNS: agc_tp_words_t -- MPAC + x, wrapped on overflow
 **********************************************************************/
agc_tp_words_t agc_mpac_tad(agc_interp_t *ip, agc_tp_words_t m, agc_tp_words_t x)
{
    agc_w3_t w = agc_w3_from_words(m); /* MPAC words */
    agc_w3_t t = agc_w3_from_words(x); /* operand words */
    agc_w2_t lo2 = {w.m1, w.m2};
    agc_w2_t add = {t.m1, t.m2};
    agc_w2_t top;                        /* MPAC, MPAC+1 after OVERFLOW */
    int o = 0;                    /* DAS / TS overflow */

    lo2 = agc_w_das(lo2, add, &o);       /* DCA 1; DAS MPAC +1 */
    w.m1 = lo2.hi;
    w.m2 = lo2.lo;
    w.m0 = agc_w_ts(o + t.m0 + w.m0, &o); /* AD 0; AD MPAC; TS */

    if (o != 0)
    {
        top = agc_w_overflow(agc_w3_top(w), o, &ip->ovfind); /* SETOVF */
        w.m0 = top.hi;
        w.m1 = top.lo;
    }

    return (agc_w3_to_words(w));
}

/**********************************************************************
 * AGC: none (basic instruction DAS) -- on two stored pairs: the
 * minor-word carry goes
 * into the major word; the major word is overflow-corrected and the
 * overflow left in A is returned. No sign agreement.
 *
 * ARGUMENTS:
 *   x    agc_dp_words_t  I  augend (the erasable DAS adds into)
 *   y    agc_dp_words_t  I  addend (A, L)
 *   ovf  int *           O  +1, -1 or 0
 *
 * RETURNS: agc_dp_words_t -- the stored sum
 **********************************************************************/
agc_dp_words_t agc_dp_words_das(agc_dp_words_t x, agc_dp_words_t y, int *ovf)
{
    agc_w2_t r; /* sum words */

    r = agc_w_das(agc_w2_from_words(x), agc_w2_from_words(y), ovf);

    return (agc_w2_to_words(r));
}

/* ---- multiply ------------------------------------------------------------ */

/**********************************************************************
 * DMP (DMPSUB, INTERPRETER.agc pp.1136, 1149) -- MPAC, MPAC+1 times the
 * operand words, all partial products kept: the TP MPAC.
 *
 * ARGUMENTS:
 *   m  agc_tp_words_t  I  MPAC (MPAC+2 not used)
 *   x  agc_dp_words_t  I  operand as stored; scales add
 *
 * RETURNS: agc_tp_words_t -- the TP product
 **********************************************************************/
agc_tp_words_t agc_mpac_dmp(agc_tp_words_t m, agc_dp_words_t x)
{
    agc_w3_t p = agc_w_dmpsub(agc_w3_top(agc_w3_from_words(m)), agc_w2_from_words(x)); /* DMPSUB */

    return (agc_w3_to_words(p));
}

/**********************************************************************
 * DMPR (INTERPRETER.agc p.1149) -- DMPSUB, then ROUNDSUB (MPAC+2 := 0,
 * OVFIND on a carry out).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND
 *   m   agc_tp_words_t    I  MPAC
 *   x   agc_dp_words_t    I  operand as stored; scales add
 *
 * RETURNS: agc_tp_words_t -- the rounded product
 **********************************************************************/
agc_tp_words_t agc_mpac_dmpr(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x)
{
    agc_w3_t p = agc_w3_from_words(agc_mpac_dmp(m, x)); /* DMPSUB product */

    p = agc_w_roundsub(p, &ip->ovfind);

    return (agc_w3_to_words(p));
}

/**********************************************************************
 * DSQ (DSQSUB, INTERPRETER.agc p.1183) -- MPAC, MPAC+1 squared as a TP.
 *
 * ARGUMENTS:
 *   m  agc_tp_words_t  I  MPAC (MPAC+2 not used)
 *
 * RETURNS: agc_tp_words_t -- the TP square, at twice the scale
 **********************************************************************/
agc_tp_words_t agc_mpac_dsq(agc_tp_words_t m)
{
    agc_w3_t p = agc_w_dsqsub(agc_w3_top(agc_w3_from_words(m))); /* DSQSUB */

    return (agc_w3_to_words(p));
}

/* ---- divide -------------------------------------------------------------- */

/**********************************************************************
 * DDV (INTERPRETER.agc pp.1149, 1162-1169) -- MPAC / operand on the
 * words (agc_w_ddv); OVFIND := 1 on overflow, DVSIGN kept.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND, DVSIGN
 *   m   agc_tp_words_t    I  MPAC (dividend)
 *   x   agc_dp_words_t    I  divisor as stored, same scale
 *
 * RETURNS: agc_tp_words_t -- the quotient, scaled 2^0
 **********************************************************************/
agc_tp_words_t agc_mpac_ddv(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x)
{
    int o = 0; /* OVFIND of DDV */
    agc_w3_t q;     /* MPAC after the divide */

    q = agc_w_ddv(agc_w3_from_words(m), agc_w2_from_words(x), &o, &ip->dvsign_neg);

    if (o != 0)
    {
        ip->ovfind = 1; /* CAF ONE; TS OVFIND */
    }

    return (agc_w3_to_words(q));
}

/**********************************************************************
 * BDDV (INTERPRETER.agc p.1149) -- DCA operand; DXCH MPAC; DXCH BUF:
 * the operand over MPAC, MPAC+1; MPAC+2 is carried.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND, DVSIGN
 *   m   agc_tp_words_t    I  MPAC (divisor)
 *   x   agc_dp_words_t    I  dividend as stored, same scale
 *
 * RETURNS: agc_tp_words_t -- the quotient, scaled 2^0
 **********************************************************************/
agc_tp_words_t agc_mpac_bddv(agc_interp_t *ip, agc_tp_words_t m, agc_dp_words_t x)
{
    agc_tp_words_t dividend = {x.hi, x.lo, m.lo}; /* MPAC after DXCH */
    agc_dp_words_t divisor = {m.hi, m.mid};      /* BUF            */

    return (agc_mpac_ddv(ip, dividend, divisor));
}

/* ---- shifts and rounding ------------------------------------------------- */

/**********************************************************************
 * ROUND (INTERPRETER.agc p.1154, ROUNDSUB p.1137) -- MPAC+2 rounded
 * into MPAC, MPAC+1 and cleared; OVFIND on a carry out.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND
 *   m   agc_tp_words_t    I  MPAC
 *
 * RETURNS: agc_tp_words_t -- the rounded MPAC (MPAC+2 = 0)
 **********************************************************************/
agc_tp_words_t agc_mpac_round(agc_interp_t *ip, agc_tp_words_t m)
{
    agc_w3_t w; /* rounded MPAC words */

    w = agc_w_roundsub(agc_w3_from_words(m), &ip->ovfind);

    return (agc_w3_to_words(w));
}

/**********************************************************************
 * SL n (TSSL p.1154: wraps, OVFIND on each overflowing step) or SR -n
 * (GENSCR / MPACSHR p.1160: truncating) on the TP MPAC.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND
 *   m   agc_tp_words_t    I  MPAC
 *   n   int               I  shift count, -127..127 (> 0 left)
 *
 * RETURNS: agc_tp_words_t -- MPAC 2^n
 **********************************************************************/
agc_tp_words_t agc_mpac_shift(agc_interp_t *ip, agc_tp_words_t m, int n)
{
    agc_w3_t w = agc_w3_from_words(m); /* MPAC words */

    if (n == 0)
    {
        return (m);
    }

    w = (n > 0) ? agc_w_tssl(w, n, &ip->ovfind) : agc_w_genscr(w, -n, false); /* LEFT / RIGHT */

    return (agc_w3_to_words(w));
}

/**********************************************************************
 * SLR n (GENSCL p.1161: TSSL, then ROUNDSUB) or SRR -n (GENSCR with the
 * terminal round, MPACSRND p.1153); a zero count only rounds.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *    IO OVFIND
 *   m   agc_tp_words_t    I  MPAC
 *   n   int               I  shift count, -127..127 (> 0 left)
 *
 * RETURNS: agc_tp_words_t -- MPAC 2^n rounded (MPAC+2 = 0)
 **********************************************************************/
agc_tp_words_t agc_mpac_shift_round(agc_interp_t *ip, agc_tp_words_t m, int n)
{
    agc_w3_t w = agc_w3_from_words(m); /* MPAC words */

    if (n < 0)
    {
        return (agc_w3_to_words(agc_w_genscr(w, -n, true)));
    }

    w = agc_w_tssl(w, n, &ip->ovfind);

    return (agc_w3_to_words(agc_w_roundsub(w, &ip->ovfind)));
}

/**********************************************************************
 * NORM (TSLC, INTERPRETER.agc p.1157) -- TPAGREE, then shift left until
 * |MPAC| >= 1/2 (zero: no shift).
 *
 * ARGUMENTS:
 *   m       agc_tp_words_t  I  MPAC
 *   nshift  int *           O  left shifts done (the AGC stores the
 *                              complement at the operand address)
 *
 * RETURNS: agc_tp_words_t -- the normalised MPAC
 **********************************************************************/
agc_tp_words_t agc_mpac_norm(agc_tp_words_t m, int *nshift)
{
    int count = 0;                                        /* left shifts done */
    agc_w3_t w = agc_w_tslc(agc_w3_from_words(m), &count); /* normalised words */

    *nshift = count;

    return (agc_w3_to_words(w));
}

/* ---- sign ---------------------------------------------------------------- */

/**********************************************************************
 * DCOMP / COMP (INTERPRETER.agc p.1151) -- complement MPAC, MPAC+1
 * (DCS) and MPAC+2 (CS) in scalar mode.
 *
 * ARGUMENTS:
 *   m  agc_tp_words_t  I  MPAC
 *
 * RETURNS: agc_tp_words_t -- -MPAC, word by word
 **********************************************************************/
agc_tp_words_t agc_mpac_dcomp(agc_tp_words_t m)
{
    agc_tp_words_t r; /* complemented words */

    r.hi = (agc_sp_t) -m.hi;
    r.mid = (agc_sp_t) -m.mid;
    r.lo = (agc_sp_t) -m.lo;

    return (r);
}

/**********************************************************************
 * ABS (INTERPRETER.agc p.1181) -- BRANCH on the three words; COMP when
 * the first non-zero word is negative.
 *
 * ARGUMENTS:
 *   m  agc_tp_words_t  I  MPAC
 *
 * RETURNS: agc_tp_words_t -- MPAC with its first non-zero word positive
 **********************************************************************/
agc_tp_words_t agc_mpac_abs(agc_tp_words_t m)
{
    int sign = agc_w_branch(agc_w3_from_words(m)); /* BRANCH */

    return ((sign < 0) ? agc_mpac_dcomp(m) : m); /* TCF COMP on negative */
}

/**********************************************************************
 * SIGN (INTERPRETER.agc p.1151) -- COMP when the operand is negative:
 * its major word by CCS, or its minor word when the major word is 0.
 *
 * ARGUMENTS:
 *   m  agc_tp_words_t  I  MPAC
 *   x  agc_dp_words_t  I  operand as stored
 *
 * RETURNS: agc_tp_words_t -- MPAC, complemented if x < 0
 **********************************************************************/
agc_tp_words_t agc_mpac_sign(agc_tp_words_t m, agc_dp_words_t x)
{
    bool negative = (x.hi < 0) || ((x.hi == 0) && (x.lo < 0)); /* CCS 0 / CCSL */

    return (negative ? agc_mpac_dcomp(m) : m); /* TCF COMP */
}

/* ---- vector products ----------------------------------------------------- */

/**********************************************************************
 * DOT (DOTSUB, INTERPRETER.agc pp.1138, 1143) -- u . v as the TP MPAC;
 * PREDOT leaves DOTINC (= DVSIGN) positive.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, DVSIGN
 *   u   agc_vec_t       I  MPAC vector
 *   v   agc_vec_t       I  operand; scales add
 *
 * RETURNS: agc_tp_words_t -- the TP dot product
 **********************************************************************/
agc_tp_words_t agc_mpac_dot(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    agc_w3_t m; /* DOTSUB words */

    ip->dvsign_neg = false; /* PREDOT: DOTINC = 2 */
    m = agc_w_dotsub(u, v, &ip->ovfind);

    return (agc_w3_to_words(m));
}

/**********************************************************************
 * VSQ (VSQSUB, INTERPRETER.agc p.1183) -- v . v as the TP MPAC.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND
 *   v   agc_vec_t       I  MPAC vector
 *
 * RETURNS: agc_tp_words_t -- |v|^2, at twice the scale of v
 **********************************************************************/
agc_tp_words_t agc_mpac_vsq(agc_interp_t *ip, agc_vec_t v)
{
    agc_w3_t m; /* VSQSUB words */

    m = agc_w_vsqsub(v, &ip->ovfind);

    return (agc_w3_to_words(m));
}
