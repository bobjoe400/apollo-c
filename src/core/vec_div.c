/**********************************************************************
 * FILE NAME: vec_div.c
 *
 * PURPOSE:
 *   Vector divided by scalar (V/SC) and the unit vector (UNIT): each
 *   component is divided with the GENDDV kernel of divide.c after the
 *   divisor is normalised.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc p.1150 (V/SC), pp.1171-1173 (V/SC2,
 *   V/SCDV), pp.1175-1179 (UNIT, LARGE, LARGE3, SMALL, UNITDV)
 *   Luminary099 INTERPRETER.agc pp.1045, 1066-1074 (identical)
 *
 * EXTERNAL VARIABLES:
 *   none (OVFIND and DVSIGN live in the caller's agc_interp_t)
 *
 * EXTERNAL REFERENCES:
 *   Name           Description
 *   -------------  -----------------------------------------------
 *   agc_w_genddv   GENDDV / MAXDV (divide.c)
 *   agc_w_sqrtsub  SQRTSUB (divide.c)
 *   agc_w_vsqsub   VSQSUB (mpac.c)
 *   agc_w_mpacshr  MPACSHR (shift.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Divide overflow exits through DVOVF: OVFIND = 1, +-DPPOSMAX in the
 *   MPAC slot being processed, the other slots as the AGC leaves them.
 *
 * NOTES:
 *   The sign of DPPOSMAX is DVSIGN (= DOTINC), which V/SC and UNIT do not
 *   reset before their early DVOVF exits (zero scalar major word with an
 *   overflowing component, zero scalar, |v|^2 overflow, zero vector): it
 *   is carried in agc_interp_t (dvsign_neg) from the previous DDV/BDDV,
 *   V/SC, UNIT, DOT/MXV/VXM/VPROJ or ARCSIN/ARCCOS on the same
 *   interpreter state (oracle snippets DDV+UNIT, V/SC+UNIT, ...).
 *   UNIT of a short vector (SMALL: |v| < 2^-14 of its scale) divides
 *   by a root taken from the low words of |v|^2 and is as coarse as the
 *   AGC's (bit-exact); R63 avoids it with RTB NORMUNIT.
 **********************************************************************/
#include "agc/core/assert.h"
#include "mpac.h"

#define VDIV_NORM_MAX 28 /* divisor doublings: |s| >= 2^-28         */

/**********************************************************************
 * V/SCDV (INTERPRETER.agc p.1173) -- one component by the scalar.
 *
 * ARGUMENTS:
 *   c       agc_dp_t *  IO component (MPAC); +-DPPOSMAX on DVOVF
 *   s_abs   agc_dp_t    I  BUF2: |s|, not normalised
 *   s_norm  agc_dp_t    I  BUF: |s| doubled k times (>= .5)
 *   k       int         I  DVNORMCT: doublings, 0..27
 *   sneg    bool        I  the scalar is negative (VBUF+5 = -0)
 *   dvsign  bool *      O  DVSIGN left at -0 (quotient negative)
 *
 * RETURNS: bool -- false on DVOVF (|c| >= |s|), true otherwise
 **********************************************************************/
static bool vsc_dv(agc_dp_t *c, agc_dp_t s_abs, agc_dp_t s_norm, int k, bool sneg, bool *dvsign)
{
    bool     neg = sneg;               /* DVSIGN = VBUF+5  */
    agc_dp_t a   = *c;                 /* |c| after /MPAC-, scaled as v */
    agc_w2_t w;                        /* |c| as MPAC words */
    agc_w2_t d   = agc_w_split(s_abs); /* BUF2 words */
    agc_dp_t q;                        /* |quotient|, scaled 2^0 */

    if (!AGC_ASSERT((k >= 0) && (k < VDIV_NORM_MAX)))
    {
        return (false);
    }

    *dvsign = neg;

    if (a == 0)
    {
        return (true); /* exit on zero     */
    }

    if (a < 0)
    { /* /MPAC-           */
        a       = -a;
        neg     = !neg;
        *dvsign = neg; /* INCR DVSIGN      */
    }

    w = agc_w_split(a);

    if ((d.hi - w.hi < 0) || ((d.hi == w.hi) && (d.lo - w.lo <= 0)))
    {
        *c = neg ? AGC_DP_MIN : AGC_DP_MAX; /* DVOVF            */ /* DVSIGN */

        return (false);
    }

    q  = agc_w_join(agc_w_genddv(agc_w_split(a * ((agc_dp_t) 1 << k)), agc_w_split(s_norm), d.hi == w.hi));
    *c = neg ? -q : q; /* FINALDV +3: DVSIGN */

    return (true);
}

/**********************************************************************
 * V/SC2 zero major scalar word (INTERPRETER.agc p.1171) -- shift the
 * vector and scalar left 14 (XCH pairs), testing each component's major
 * word for overflow (BZF +2 / TCF DVOVF).
 *
 * ARGUMENTS:
 *   m  agc_dp_t[3]  IO MPAC, MPAC+3, MPAC+5; shifted up to the slot
 *                      that overflowed
 *   s  agc_dp_t *   IO the scalar (BUF), shifted left 14
 *
 * RETURNS: int -- the slot that overflowed (0 also for a zero shifted
 *          scalar), or -1 when the divide can proceed
 **********************************************************************/
static int vsc_shift14(agc_dp_t m[AGC_VEC_N], agc_dp_t *s)
{
    int      k; /* component slot */
    agc_w2_t w; /* component words before the shift */

    if (!AGC_ASSERT(s != 0))
    {
        return (0); /* no valid result */
    }

    *s = agc_w_split(*s).lo * AGC_W_ONE;

    for (k = 0; k < AGC_VEC_N; k++)
    {
        w    = agc_w_split(m[k]);
        m[k] = w.lo * AGC_W_ONE; /* XCH +1; XCH      */

        if (w.hi != 0)
        {
            return (k); /* BZF +2; DVOVF    */
        }
    }

    if (!AGC_ASSERT(k == AGC_VEC_N))
    {
        return (0); /* no valid result */
    }

    return ((*s == 0) ? 0 : -1); /* zero divisor     */ /* slot 0 for DVOVF */
}

/**********************************************************************
 * VROTATEX / DVOVF exit (INTERPRETER.agc p.1172) -- the vector as MPAC,
 * MPAC+3, MPAC+5 hold it on exit.
 *
 * ARGUMENTS:
 *   m0  agc_dp_t  I  MPAC (x, or the slot DVOVF overwrote)
 *   m3  agc_dp_t  I  MPAC+3 (y)
 *   m5  agc_dp_t  I  MPAC+5 (z)
 *
 * RETURNS: agc_vec_t -- (m0, m3, m5), each scaled like the quotient
 **********************************************************************/
static agc_vec_t vsc_result(agc_dp_t m0, agc_dp_t m3, agc_dp_t m5)
{
    agc_vec_t r; /* MPAC, MPAC+3, MPAC+5, quotient scale */

    r.c[0] = m0; /* MPAC             */
    r.c[1] = m3; /* MPAC +3          */
    r.c[2] = m5; /* MPAC +5          */

    if (!AGC_ASSERT((m0 >= AGC_DP_MIN) && (m0 <= AGC_DP_MAX)))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT((m3 >= AGC_DP_MIN) && (m3 <= AGC_DP_MAX) && (m5 >= AGC_DP_MIN) && (m5 <= AGC_DP_MAX)))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * V/SC (INTERPRETER.agc p.1150, pp.1171-1173) -- v / s, each component
 * divided by the normalised scalar; quotient scale = v scale - s scale.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND (set to 1 on overflow), DVSIGN
 *   v   agc_vec_t       I  dividend vector
 *   s   agc_dp_t        I  divisor, same scale as v
 *
 * RETURNS: agc_vec_t -- v / s; on overflow (|v_i| >= |s|, s = 0) the
 *          MPAC arrangement DVOVF leaves (see FILE NOTES)
 **********************************************************************/
agc_vec_t agc_vec_div(agc_interp_t *ip, agc_vec_t v, agc_dp_t s)
{
    agc_dp_t m[AGC_VEC_N] = {v.c[0], v.c[1], v.c[2]}; /* MPAC, +3, +5 */
    agc_dp_t s_norm;                                  /* BUF: |s| 2^k, scaled as s */
    bool     sneg         = false;                    /* VBUF+5 = -0: scalar negative */
    int      k            = 0;                        /* DVNORMCT: divisor doublings */
    int      bad;                                     /* slot that overflowed in V/SC2, or -1 */

    if (!AGC_ASSERT(ip != 0))
    {
        return (v);
    }

    if (agc_w_split(s).hi == 0)
    {
        bad = vsc_shift14(m, &s);

        if (bad >= 0)
        {
            ip->ovfind = 1;

            /* DVOVF before any V/SCDV: the sign is the DVSIGN left by the
             * previous divide, DOT or ARCSIN/ARCCOS (FINALDV +3).          */
            return (vsc_result(ip->dvsign_neg ? AGC_DP_MIN : AGC_DP_MAX, m[1], m[2])); /* stale DVSIGN */
        }
    }

    if (!AGC_ASSERT(s != 0))
    {
        return (v);
    }

    if (s < 0)
    { /* /BUF-            */
        s    = -s;
        sneg = true;
    }

    for (s_norm = s; (k < VDIV_NORM_MAX) && (s_norm < AGC_DP_HALF); k++)
    {
        s_norm *= 2; /* /NORM2           */
    }

    if (!vsc_dv(&m[0], s, s_norm, k, sneg, &ip->dvsign_neg))
    {
        ip->ovfind = 1;

        return (vsc_result(m[0], m[1], m[2]));
    }

    if (!vsc_dv(&m[1], s, s_norm, k, sneg, &ip->dvsign_neg))
    { /* MPAC = y, +3 = x */
        ip->ovfind = 1;

        return (vsc_result(m[1], m[0], m[2]));
    }

    if (!vsc_dv(&m[2], s, s_norm, k, sneg, &ip->dvsign_neg))
    { /* MPAC = z, +5 = y */
        ip->ovfind = 1;

        return (vsc_result(m[2], m[0], m[1]));
    }

    return (vsc_result(m[0], m[1], m[2])); /* VROTATEX         */
} /* agc_vec_div */

/* ---- UNIT ---------------------------------------------------------------- */

/**********************************************************************
 * DVOVF exit of UNIT (INTERPRETER.agc p.1175) -- MPAC = +-DPPOSMAX,
 * MPAC+3 = 0 and MPAC+5 = the major word VSQSUB left in A after the
 * x^2 + y^2 accumulation (or its overflow +-1).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND := 1; DVSIGN (stale) signs MPAC
 *   v   agc_vec_t       I  the UNIT argument
 *
 * RETURNS: agc_vec_t -- the vector MPAC holds on the DVOVF exit
 **********************************************************************/
static agc_vec_t unit_overflow(agc_interp_t *ip, agc_vec_t v)
{
    agc_ovf_t o = 0; /* overflow of the x^2 + y^2 accumulation */
    agc_w3_t  acc;   /* BUF: x^2 + y^2 (TP) */
    int32_t   a;     /* A at the DVOVF: major word or the overflow */

    if (!AGC_ASSERT(ip != 0))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    acc = agc_w_dot_acc(agc_w_dsqsub(agc_w_split(v.c[0])), agc_w_dsqsub(agc_w_split(v.c[1])), &o);
    a   = (o != 0) ? (int32_t) o : acc.m0; /* TS BUF / TS OVFIND */ /* A */

    if (!AGC_ASSERT(agc_w_ok(a)))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    ip->ovfind = 1;

    /* DVOVF before any UNITDV: the sign is the DVSIGN left by the previous
     * divide, DOT or ARCSIN/ARCCOS (FINALDV +3).                          */
    return (vsc_result(ip->dvsign_neg ? AGC_DP_MIN : AGC_DP_MAX, 0, (agc_dp_t) (a * AGC_W_ONE))); /* stale DVSIGN */
}

/**********************************************************************
 * UNITDV (INTERPRETER.agc p.1179) -- one component: |c| shifted by the
 * pre-divide count (kdiv < 0: the "-21D" prologue, shift right 1),
 * divided by the root (MAXDVSW = -1: no MAXDV case in UNIT).
 *
 * ARGUMENTS:
 *   c       agc_dp_t  I  component (VBUF), possibly shifted left 14
 *   kdiv    int       I  pre-divide left shift, -1..13
 *   root    agc_dp_t  I  normalised root (BUF), > .25
 *   dvsign  bool *    O  DVSIGN: -0 for a negative component; not
 *                        written for a zero component (exits first)
 *
 * RETURNS: agc_dp_t -- the half-unit component, scaled 2^1
 **********************************************************************/
static agc_dp_t unit_dv(agc_dp_t c, int kdiv, agc_dp_t root, bool *dvsign)
{
    agc_dp_t a = (c < 0) ? -c : c; /* |c|, scaled as v */
    agc_dp_t q;                    /* |quotient|, scaled 2^0 */

    if (!AGC_ASSERT((kdiv >= -1) && (kdiv <= 13)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(root > AGC_DP_QUARTER)) /* normalised root  */
    {
        return (0); /* no valid result */
    }

    if (c == 0)
    {
        return (0); /* exit on zero     */
    }

    *dvsign = (c < 0);                                                            /* UMPAC- / UMPAC+  */
    a       = (kdiv < 0) ? (a / 2) : (a * ((agc_dp_t) 1 << kdiv));                /* SRDDV / left kdiv */
    q       = agc_w_join(agc_w_genddv(agc_w_split(a), agc_w_split(root), false)); /* MAXDVSW = -1 */

    return ((c < 0) ? -q : q); /* UMPAC-: complement */
}

/**********************************************************************
 * UNIT (INTERPRETER.agc pp.1175-1179) -- half-unit vector v / (2|v|),
 * with LV and LVSQUARE as the AGC leaves them in the work area.
 *
 * ARGUMENTS:
 *   ip     agc_interp_t *  IO OVFIND (1 on overflow), DVSIGN
 *   v      agc_vec_t       I  argument, any scale 2^n
 *   abval  agc_dp_t *      O  LV: |v| scaled as v (0 for a zero vector,
 *                             untouched on |v|^2 overflow); may be NULL
 *   vsq    agc_dp_t *      O  LVSQUARE: |v|^2 at 2^2n (untouched on
 *                             overflow); may be NULL
 *
 * RETURNS: agc_vec_t -- the half-unit vector, scaled 2^1; the DVOVF
 *          arrangement for |v|^2 >= 1 or v = 0
 **********************************************************************/
agc_vec_t agc_vec_unit(agc_interp_t *ip, agc_vec_t v, agc_dp_t *abval, agc_dp_t *vsq)
{
    agc_ovf_t o     = 0;         /* OVFIND of VSQSUB (saved OVFIND restored) */
    agc_w3_t  sq;                /* |v|^2 (TP MPAC) */
    agc_w2_t  root;              /* SQRTSUB: normalised root */
    agc_w3_t  lv    = {0, 0, 0}; /* LV words, unnormalised root */
    int       shift = 0;         /* MPTEMP: SQRTSUB right-shift count */
    int       kdiv;              /* pre-divide shift, -1..13 */
    int       k;                 /* component index */
    bool      abort = false;     /* SQRTABRT (cannot happen: |v|^2 >= 0) */

    if (!AGC_ASSERT(ip != 0))
    {
        return (v);
    }

    sq = agc_w_vsqsub(v, &o); /* OVFIND saved     */

    if (o != 0)
    {
        return (unit_overflow(ip, v)); /* LV not written   */
    }

    if (vsq != 0)
    {
        *vsq = agc_w_join(agc_w3_top(sq)); /* LVSQUARE         */
    }

    root = agc_w3_top(agc_w_sqrtsub(sq, &shift, &abort));

    if (!AGC_ASSERT(!abort))
    {
        return (v);
    }

    if (root.hi == 0)
    { /* input to SQRT 0  */
        if (abval != 0)
        {
            *abval = 0; /* LV = 0           */
        }

        return (unit_overflow(ip, v));
    }

    if (shift > 14)
    { /* SMALL            */
        kdiv  = shift - 15;
        lv.m1 = agc_w_mp((int32_t) 1 << (28 - shift), root.hi).hi;

        for (k = 0; k < AGC_VEC_N; k++)
        {
            if (!AGC_ASSERT(agc_w_split(v.c[k]).hi == 0))
            {
                return (v);
            }

            v.c[k] *= AGC_W_ONE; /* VBUF left 14     */
        }
    }
    else if (shift == 14)
    {
        kdiv  = 13; /* CS THIRTEEN      */
        lv.m1 = root.hi;
    }
    else if (shift > 0)
    { /* LARGE3           */
        kdiv  = shift - 1;
        lv.m0 = root.hi;
        lv.m1 = root.lo;
        lv    = agc_w_mpacshr(lv, shift, false);
    }
    else
    { /* LARGE: CS SRDDV  */
        kdiv  = -1;
        lv.m0 = root.hi;
        lv.m1 = root.lo;
    }

    if (abval != 0)
    {
        *abval = agc_w_join(agc_w3_top(lv)); /* LV               */
    }

    for (k = 0; k < AGC_VEC_N; k++)
    {
        v.c[k] = unit_dv(v.c[k], kdiv, agc_w_join(root), &ip->dvsign_neg);
    }

    return (v);
} /* agc_dp_t *vsq) */
