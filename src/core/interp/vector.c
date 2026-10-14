/**********************************************************************
 * FILE NAME: vector.c
 *
 * PURPOSE:
 *   Vector and matrix interpretive instructions: VDEF, VAD/VSU/BVSU,
 *   VCOMP, VXSC, DOT, VSQ, ABVAL, VXV, VPROJ, VSL, VSR, MXV, VXM and the
 *   MXM3 / TRANSPOS subroutines. V/SC and UNIT are in vec_div.c.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc pp.1131-1134 (VAD, VSU, BVSU), p.1138
 *   (DOTSUB), pp.1143-1148 (DOT, MXV, VXM, VXSC, VPROJ, VXV), p.1151
 *   (VCOMP), pp.1155-1159 (VSR, VSL), pp.1180-1183 (ABVAL, VSQ);
 *   ANGLFIND.agc pp.404-405 (MXM3, TRANSPOS, TRNSPSPD)
 *   Luminary099 INTERPRETER.agc pp.1026-1078 (identical);
 *   ATTITUDE_MANEUVER_ROUTINE.agc (MXM3 as VXM of the rows of M1,
 *   TRANSPOS)
 *
 * EXTERNAL VARIABLES:
 *   none (OVFIND and DVSIGN live in the caller's agc_interp_t)
 *
 * EXTERNAL REFERENCES:
 *   Name            Description
 *   --------------  ----------------------------------------------
 *   agc_w_dotsub    DOTSUB (mpac.c)
 *   agc_w_vsqsub    VSQSUB (mpac.c)
 *   agc_w_dmpsub    DMPSUB (mpac.c)
 *   agc_w_roundsub  ROUNDSUB / VROUND (mpac.c)
 *   agc_w_dad       DAS + OVERFLOW (mpac.c)
 *   agc_w_vright    VSR per component (shift.c)
 *   agc_tp_sqrt     SQRT of the TP square (trig.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; overflow sets OVFIND per component as the AGC does.
 *
 * NOTES:
 *   Component i of an agc_vec_t is the AGC's MPAC (x), MPAC+3 (y),
 *   MPAC+5 (z) or the words at X, X+2, X+4. A matrix is three row
 *   vectors at X, X+6, X+12, the AGC layout. MXM3 differs in text
 *   between the ropes (CM: MXV of the columns of M2, then TRNSPSPD; LM:
 *   VXM of the rows of M1) but every element is the same DOTSUB of a row
 *   of M1 and a column of M2 (DMPSUB is symmetric in its operands), so
 *   one C function serves both.
 **********************************************************************/
#include "../mpac.h"

#define VSHIFT_MAX 43 /* a VSL past 42 places leaves zero        */

/**********************************************************************
 * VDEF (INTERPRETER.agc p.1180) -- a vector from three DP components.
 *
 * ARGUMENTS:
 *   x  agc_dp_t  I  x component (MPAC)
 *   y  agc_dp_t  I  y component (MPAC+3), same scale
 *   z  agc_dp_t  I  z component (MPAC+5), same scale
 *
 * RETURNS: agc_vec_t -- (x, y, z)
 **********************************************************************/
agc_vec_t agc_vec_make(agc_dp_t x, agc_dp_t y, agc_dp_t z)
{
    agc_vec_t v; /* MPAC, MPAC+3, MPAC+5, scale of x */

    v.c[0] = x;
    v.c[1] = y;
    v.c[2] = z;

    return (v);
}

/**********************************************************************
 * ZEROVECS (INTERPRETIVE_CONSTANTS.agc p.1205) -- the zero vector.
 *
 * RETURNS: agc_vec_t -- (0, 0, 0), any scale
 **********************************************************************/
agc_vec_t agc_vec_zero(void)
{
    const agc_vec_t v = {{0, 0, 0}};

    return (v);
}

/* ---- add / subtract / complement ----------------------------------------- */

/**********************************************************************
 * VAD (INTERPRETER.agc p.1131) -- per component DAS, then OVERFLWY /
 * OVERFLWZ / OVERFLOW, in the AGC's order y, z, x (the last overflow
 * decides the sign in OVFIND).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   u   agc_vec_t       I  MPAC vector
 *   v   agc_vec_t       I  operand, same scale
 *
 * RETURNS: agc_vec_t -- u + v, each component overflow-corrected
 **********************************************************************/
agc_vec_t agc_vec_add(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    static const int order[AGC_VEC_N] = {1, 2, 0};
    agc_vec_t        r                = u; /* result, scale of u */
    int              k;                    /* component index */
    int              i;                    /* step in AGC order */

    for (i = 0; i < AGC_VEC_N; i++)
    {
        k      = order[i];
        r.c[k] = agc_w_join(agc_w_dad(agc_w_split(u.c[k]), agc_w_split(v.c[k]), &ip->ovfind));
    }

    return (r);
}

/**********************************************************************
 * VSU (INTERPRETER.agc p.1131) -- DCS of each component of X, then as
 * VAD. BVSU X is agc_vec_sub(ip, X, mpac).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   u   agc_vec_t       I  minuend
 *   v   agc_vec_t       I  subtrahend, same scale
 *
 * RETURNS: agc_vec_t -- u - v, each component overflow-corrected
 **********************************************************************/
agc_vec_t agc_vec_sub(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    return (agc_vec_add(ip, u, agc_vec_neg(v)));
}

/**********************************************************************
 * VCOMP (COMP, INTERPRETER.agc p.1151, vector mode).
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I  vector
 *
 * RETURNS: agc_vec_t -- -v
 **********************************************************************/
agc_vec_t agc_vec_neg(agc_vec_t v)
{
    agc_vec_t r; /* -v, scale of v */
    int       k; /* component index */

    for (k = 0; k < AGC_VEC_N; k++)
    {
        r.c[k] = -v.c[k];
    }

    return (r);
}

/* ---- products ------------------------------------------------------------ */

/**********************************************************************
 * VXSC / VVXSC (INTERPRETER.agc p.1145) -- DMPSUB of each component by
 * the scalar, VROUND each. Serves both modes (vector in MPAC times a
 * scalar, or scalar in MPAC times a vector): DMPSUB is symmetric.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set on a rounding carry out
 *   v   agc_vec_t       I  vector
 *   s   agc_dp_t        I  scalar; scales add
 *
 * RETURNS: agc_vec_t -- s v, each component rounded
 **********************************************************************/
agc_vec_t agc_vec_scale(agc_interp_t *ip, agc_vec_t v, agc_dp_t s)
{
    agc_vec_t r = v; /* result, scales of v and s add */
    agc_w3_t  m;     /* DMPSUB TP product */
    int       k;     /* component index */

    for (k = 0; k < AGC_VEC_N; k++)
    {
        m      = agc_w_dmpsub(agc_w_split(v.c[k]), agc_w_split(s));
        m      = agc_w_roundsub(m, &ip->ovfind); /* VROUND           */
        r.c[k] = agc_w_join(agc_w3_top(m));
    }

    return (r);
}

/**********************************************************************
 * DOT (INTERPRETER.agc p.1143) via PREDOT / DOTSUB -- TP result.
 * PREDOT sets DOTINC (= DVSIGN) to 2.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND (+-1 on overflow), DVSIGN positive
 *   u   agc_vec_t       I  MPAC vector
 *   v   agc_vec_t       I  operand; scales add
 *
 * RETURNS: agc_tp_t -- u . v (TP MPAC)
 **********************************************************************/
agc_tp_t agc_vec_dot_tp(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    agc_w3_t m; /* DOTSUB TP result */

    ip->dvsign_neg = false; /* PREDOT: DOTINC (= DVSIGN) = 2 */
    m              = agc_w_dotsub(u, v, &ip->ovfind);

    return (agc_w3_value(m));
}

/**********************************************************************
 * DOT (INTERPRETER.agc p.1143) as a DP: MPAC, MPAC+1 of DOTSUB (MODE
 * becomes DP). PREDOT sets DOTINC (= DVSIGN) to 2.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND (+-1 on overflow), DVSIGN positive
 *   u   agc_vec_t       I  MPAC vector
 *   v   agc_vec_t       I  operand; scales add
 *
 * RETURNS: agc_dp_t -- u . v truncated to DP
 **********************************************************************/
agc_dp_t agc_vec_dot(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    agc_w3_t m; /* DOTSUB TP result */

    ip->dvsign_neg = false; /* PREDOT: DOTINC (= DVSIGN) = 2 */
    m              = agc_w_dotsub(u, v, &ip->ovfind);

    return (agc_w_join(agc_w3_top(m)));
}

/**********************************************************************
 * VSQ (INTERPRETER.agc p.1182) -- VSQSUB, DP mode.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   v   agc_vec_t       I  vector at 2^n
 *
 * RETURNS: agc_dp_t -- |v|^2 at 2^2n, truncated to DP
 **********************************************************************/
agc_dp_t agc_vec_sq(agc_interp_t *ip, agc_vec_t v)
{
    agc_w3_t m; /* VSQSUB TP result */

    m = agc_w_vsqsub(v, &ip->ovfind);

    return (agc_w_join(agc_w3_top(m)));
}

/**********************************************************************
 * VSQ (VSQSUB, INTERPRETER.agc p.1183) as the full TP MPAC.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   v   agc_vec_t       I  vector at 2^n
 *
 * RETURNS: agc_tp_t -- |v|^2 at 2^2n
 **********************************************************************/
agc_tp_t agc_vec_sq_tp(agc_interp_t *ip, agc_vec_t v)
{
    agc_w3_t m; /* VSQSUB TP result */

    m = agc_w_vsqsub(v, &ip->ovfind);

    return (agc_w3_value(m));
}

/**********************************************************************
 * ABVAL (INTERPRETER.agc pp.1180-1181) -- VSQSUB, then SQRT of the TP
 * square.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set when |v|^2 overflows
 *   v   agc_vec_t       I  vector at 2^n
 *
 * RETURNS: agc_dp_t -- |v| at 2^n
 **********************************************************************/
agc_dp_t agc_vec_abval(agc_interp_t *ip, agc_vec_t v)
{
    agc_tp_t sq; /* |v|^2 TP, at 2^2n */

    sq = agc_vec_sq_tp(ip, v);

    return (agc_tp_sqrt(sq));
}

/**********************************************************************
 * DMPSUB (INTERPRETER.agc p.1136) for VXV -- the top two words of
 * DMPSUB(mpac, x).
 *
 * ARGUMENTS:
 *   mpac  agc_dp_t  I  MPAC operand
 *   x     agc_dp_t  I  the operand at ADDRWD; scales add
 *
 * RETURNS: agc_w2_t -- the truncated product words
 **********************************************************************/
static agc_w2_t mul_top(agc_dp_t mpac, agc_dp_t x)
{
    return (agc_w3_top(agc_w_dmpsub(agc_w_split(mpac), agc_w_split(x))));
}

/**********************************************************************
 * VXV (INTERPRETER.agc pp.1147-1148) -- MPAC x X with truncated
 * partial products, each component finished by DAS and the OVERFLOW
 * routine. MPAC = u, X = v.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   u   agc_vec_t       I  MPAC vector
 *   v   agc_vec_t       I  operand; scales add
 *
 * RETURNS: agc_vec_t -- u x v
 **********************************************************************/
agc_vec_t agc_vec_cross(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    agc_vec_t r; /* u x v, scales add */
    agc_w2_t  z; /* z: X2M1 - X1M2 */
    agc_w2_t  y; /* y: X1M3 - X3M1 */
    agc_w2_t  x; /* x: X3M2 - X2M3 */

    z = mul_top(-u.c[1], v.c[0]);                            /* -X1M2 in MPAC+5  */
    z = agc_w_dad(z, mul_top(u.c[0], v.c[1]), &ip->ovfind);  /* + X2M1    */
    y = mul_top(u.c[2], v.c[0]);                             /* X1M3 (VBUF +2)   */
    y = agc_w_dad(y, mul_top(-u.c[0], v.c[2]), &ip->ovfind); /* - X3M1    */
    x = mul_top(u.c[1], v.c[2]);                             /* X3M2             */
    x = agc_w_dad(x, mul_top(-u.c[2], v.c[1]), &ip->ovfind); /* - X2M3    */

    r.c[0] = agc_w_join(x);
    r.c[1] = agc_w_join(y);
    r.c[2] = agc_w_join(z);

    return (r);
}

/**********************************************************************
 * VPROJ (INTERPRETER.agc p.1146) -- DOT, then DVXSC of the vector at X
 * by the DP dot (MPAC, MPAC+1 words): (u . v) v, each component
 * VROUNDed. PREDOT sets DOTINC (= DVSIGN) to 2.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, DVSIGN positive
 *   u   agc_vec_t       I  MPAC vector
 *   v   agc_vec_t       I  the vector at X
 *
 * RETURNS: agc_vec_t -- (u . v) v; scales of u, v, v add
 **********************************************************************/
agc_vec_t agc_vec_proj(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    agc_vec_t r = v; /* (u . v) v, scales of u, v, v add */
    agc_w2_t  d;     /* u . v as MPAC words */
    agc_w3_t  m;     /* DMPSUB TP product */
    int       k;     /* component index */

    ip->dvsign_neg = false; /* PREDOT: DOTINC (= DVSIGN) = 2 */
    d              = agc_w3_top(agc_w_dotsub(u, v, &ip->ovfind));

    for (k = 0; k < AGC_VEC_N; k++)
    {
        m      = agc_w_dmpsub(d, agc_w_split(v.c[k])); /* DVXSC            */
        m      = agc_w_roundsub(m, &ip->ovfind);       /* VROUND           */
        r.c[k] = agc_w_join(agc_w3_top(m));
    }

    return (r);
}

/* ---- shifts -------------------------------------------------------------- */

/**********************************************************************
 * VSL n (VSSL, INTERPRETER.agc p.1156) -- one place at a time, x y z
 * each step, DAS then OVERFLOW per component; wraps and sets OVFIND
 * like the AGC. A negative n is a VSR.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, set to +-1 on overflow
 *   v   agc_vec_t       I  vector
 *   n   int             I  shift count, <= 127
 *
 * RETURNS: agc_vec_t -- v 2^n (scale -n)
 **********************************************************************/
agc_vec_t agc_vec_shl(agc_interp_t *ip, agc_vec_t v, int n)
{
    agc_w2_t  p[AGC_VEC_N]; /* component words */
    agc_vec_t r = v;        /* result, scale -n */
    int       k;            /* component index */
    int       j;            /* steps done */

    if (n < 0)
    {
        return (agc_vec_shr(v, -n));
    }

    for (k = 0; k < AGC_VEC_N; k++)
    {
        p[k] = agc_w_split(v.c[k]);
    }

    for (j = 0; (j < n) && (j < VSHIFT_MAX); j++)
    {
        for (k = 0; k < AGC_VEC_N; k++)
        {
            p[k] = agc_w_dad(p[k], p[k], &ip->ovfind);
        }
    }

    for (k = 0; k < AGC_VEC_N; k++)
    {
        r.c[k] = (n > VSHIFT_MAX) ? 0 : agc_w_join(p[k]); /* all bits gone */
    }

    return (r);
}

/**********************************************************************
 * VSR n (VSSR / VSHRRND p.1155, VRIGHT2 p.1159 of INTERPRETER.agc) --
 * always rounds; past 14 places it rounds at each whole-word step too.
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I  vector
 *   n  int        I  shift count, >= 0
 *
 * RETURNS: agc_vec_t -- v 2^-n rounded (scale +n)
 **********************************************************************/
agc_vec_t agc_vec_shr(agc_vec_t v, int n)
{
    agc_vec_t r = v; /* result, scale +n */
    int       k;     /* component index */

    if (n == 0)
    {
        return (v);
    }

    for (k = 0; k < AGC_VEC_N; k++)
    {
        r.c[k] = agc_w_join(agc_w_vright(agc_w_split(v.c[k]), n));
    }

    return (r);
}

/* ---- matrices ------------------------------------------------------------ */

/**********************************************************************
 * UNITX / UNITY / UNITZ as rows (INTERPRETIVE_CONSTANTS.agc p.1205) --
 * I/2, the half-unit identity.
 *
 * RETURNS: agc_mat_t -- I/2 (scaled 2^1)
 **********************************************************************/
agc_mat_t agc_mat_identity_half(void)
{
    agc_mat_t m; /* I/2, scaled 2^1 */
    int       i; /* row index */
    int       j; /* column index */

    for (i = 0; i < AGC_VEC_N; i++)
    {
        for (j = 0; j < AGC_VEC_N; j++)
        {
            m.row[i].c[j] = (i == j) ? AGC_DP_HALF : 0; /* diagonal */
        }
    }

    return (m);
}

/**********************************************************************
 * MXV (INTERPRETER.agc pp.1143-1144) -- component i = DOTSUB(MPAC
 * vector, row i) (DOTINC = 2).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, DVSIGN positive
 *   m   const agc_mat_t *  I  matrix (rows at X, X+6, X+12)
 *   v   agc_vec_t       I  MPAC vector; scales add
 *
 * RETURNS: agc_vec_t -- M v
 **********************************************************************/
agc_vec_t agc_mat_vec(agc_interp_t *ip, const agc_mat_t *m, agc_vec_t v)
{
    agc_vec_t r; /* M v, scales add */
    int       i; /* row index */

    for (i = 0; i < AGC_VEC_N; i++)
    {
        r.c[i] = agc_vec_dot(ip, v, m->row[i]);
    }

    return (r);
}

/**********************************************************************
 * TRANSPOS / TRNSPSPD (ANGLFIND.agc pp.404-405) -- swap rows and
 * columns.
 *
 * ARGUMENTS:
 *   m  const agc_mat_t *  I  matrix
 *
 * RETURNS: agc_mat_t -- M transposed
 **********************************************************************/
agc_mat_t agc_mat_transpose(const agc_mat_t *m)
{
    agc_mat_t t; /* transpose */
    int       i; /* row index */
    int       j; /* column index */

    for (i = 0; i < AGC_VEC_N; i++)
    {
        for (j = 0; j < AGC_VEC_N; j++)
        {
            t.row[i].c[j] = m->row[j].c[i];
        }
    }

    return (t);
}

/**********************************************************************
 * VXM (INTERPRETER.agc pp.1143-1144) -- component j = DOTSUB(MPAC
 * vector, column j) (DOTINC = 6 walks down a column).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  IO OVFIND, DVSIGN positive
 *   v   agc_vec_t       I  MPAC vector
 *   m   const agc_mat_t *  I  matrix; scales add
 *
 * RETURNS: agc_vec_t -- v^T M
 **********************************************************************/
agc_vec_t agc_vec_mat(agc_interp_t *ip, agc_vec_t v, const agc_mat_t *m)
{
    agc_vec_t r; /* v^T M, scales add */
    agc_mat_t t; /* M transposed (columns as rows) */
    int       j; /* column index */

    t = agc_mat_transpose(m);

    for (j = 0; j < AGC_VEC_N; j++)
    {
        r.c[j] = agc_vec_dot(ip, v, t.row[j]);
    }

    return (r);
}

/**********************************************************************
 * MXM3 (ANGLFIND.agc p.404 CM; ATTITUDE_MANEUVER_ROUTINE.agc LM) --
 * column j of A B = MXV(A, column j of B), the columns then transposed
 * back (TRNSPSPD); the LM's VXM of the rows gives the same words.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *     IO OVFIND, DVSIGN positive
 *   a   const agc_mat_t *  I  M1
 *   b   const agc_mat_t *  I  M2; scales add
 *
 * RETURNS: agc_mat_t -- A B
 **********************************************************************/
agc_mat_t agc_mat_mul(agc_interp_t *ip, const agc_mat_t *a, const agc_mat_t *b)
{
    agc_mat_t cols; /* columns of A B */
    agc_mat_t bt;   /* B transposed */
    int       j;    /* column index */

    bt = agc_mat_transpose(b);

    for (j = 0; j < AGC_VEC_N; j++)
    {
        cols.row[j] = agc_mat_vec(ip, a, bt.row[j]);
    }

    return (agc_mat_transpose(&cols));
}
