/*
 * agc_interp_vector.c -- vector and matrix interpretive instructions
 * (INTERPRETER.agc, Comanche055 pp.1131-1134 VAD/VSU/BVSU, 1138 DOTSUB,
 * 1143-1148 DOT/MXV/VXM/VXSC/VPROJ/VXV, 1151 VCOMP, 1155-1159 VSR/VSL,
 * 1180-1183 ABVAL/VSQ; ANGLFIND.agc p.404 MXM3/TRANSPOS).
 * V/SC and UNIT are in agc_vec_div.c.
 *
 * Component i of an agc_vec_t is the AGC's MPAC (x), MPAC+3 (y), MPAC+5 (z)
 * or the words at X, X+2, X+4. A matrix is three row vectors at X, X+6,
 * X+12, exactly the AGC layout.
 */
#include "../mpac.h"
#include "agc/core/assert.h"

#define VSHIFT_MAX 43 /* a VSL past 42 places leaves zero        */

agc_vec_t agc_vec_make(agc_dp_t x, agc_dp_t y, agc_dp_t z)
{
    agc_vec_t v;

    if (!AGC_ASSERT((x >= AGC_DP_MIN) && (x <= AGC_DP_MAX)))
    {
        return ((agc_vec_t){{0, 0, 0}}); /* no valid result */
    }

    if (!AGC_ASSERT((y >= AGC_DP_MIN) && (y <= AGC_DP_MAX)))
    {
        return ((agc_vec_t){{0, 0, 0}}); /* no valid result */
    }

    if (!AGC_ASSERT((z >= AGC_DP_MIN) && (z <= AGC_DP_MAX)))
    {
        return ((agc_vec_t){{0, 0, 0}}); /* no valid result */
    }

    v.c[0] = x;
    v.c[1] = y;
    v.c[2] = z;

    return (v);
}

agc_vec_t agc_vec_zero(void)
{
    const agc_vec_t v = {{0, 0, 0}};

    return (v);
}

/* ---- add / subtract / complement ----------------------------------------- */

/* VAD (p.1131): per component DAS, then OVERFLWY/OVERFLWZ/OVERFLOW, in the
 * AGC's order y, z, x (the last overflow decides the sign in OVFIND).     */
agc_vec_t agc_vec_add(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    static const int order[AGC_VEC_N] = {1, 2, 0};
    agc_vec_t        r                = u;
    int              k;
    int              i;

    if (!AGC_ASSERT(ip != 0))
    {
        return (u);
    }

    for (i = 0; i < AGC_VEC_N; i++)
    {
        k      = order[i];
        r.c[k] = agc_w_join(
            agc_w_dad(agc_w_split(u.c[k]), agc_w_split(v.c[k]), &ip->ovfind));
    }

    if (!AGC_ASSERT(i == AGC_VEC_N))
    {
        return (u);
    }

    return (r);
}

/* VSU (p.1131): DCS of each component of X, then as VAD. BVSU X is
 * agc_vec_sub(ip, X, mpac).                                                */
agc_vec_t agc_vec_sub(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    if (!AGC_ASSERT(ip != 0))
    {
        return (u);
    }

    if (!AGC_ASSERT(v.c[0] >= AGC_DP_MIN))
    {
        return (u);
    }

    return (agc_vec_add(ip, u, agc_vec_neg(v)));
}

/* VCOMP (COMP p.1151, vector mode).                                         */
agc_vec_t agc_vec_neg(agc_vec_t v)
{
    agc_vec_t r;
    int       k;

    for (k = 0; k < AGC_VEC_N; k++)
    {
        if (!AGC_ASSERT((v.c[k] >= AGC_DP_MIN) && (v.c[k] <= AGC_DP_MAX)))
        {
            return (agc_vec_zero()); /* no valid result */
        }

        r.c[k] = -v.c[k];
    }

    if (!AGC_ASSERT(k == AGC_VEC_N))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (r);
}

/* ---- products ------------------------------------------------------------ */

/* VVXSC (p.1145): DMPSUB of each component by the scalar, VROUND each.     */
agc_vec_t agc_vec_scale(agc_interp_t *ip, agc_vec_t v, agc_dp_t s)
{
    agc_vec_t r = v;
    agc_w3_t  m;
    int       k;

    if (!AGC_ASSERT(ip != 0))
    {
        return (v);
    }

    for (k = 0; k < AGC_VEC_N; k++)
    {
        m      = agc_w_dmpsub(agc_w_split(v.c[k]), agc_w_split(s));
        m      = agc_w_roundsub(m, &ip->ovfind); /* VROUND           */
        r.c[k] = agc_w_join(agc_w3_top(m));
    }

    if (!AGC_ASSERT(k == AGC_VEC_N))
    {
        return (v);
    }

    return (r);
}

/* DOT (p.1143) via DOTSUB: TP result.                                       */
agc_tp_t agc_vec_dot_tp(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    agc_w3_t m;

    if (!AGC_ASSERT(ip != 0))
    {
        return (0);
    }

    m = agc_w_dotsub(u, v, &ip->ovfind);

    if (!AGC_ASSERT(agc_w_ok(m.m0)))
    {
        return (0);
    }

    return (agc_w3_value(m));
}

/* DOT as a DP: MPAC,+1 of the DOTSUB result (MODE becomes DP).              */
agc_dp_t agc_vec_dot(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    agc_w3_t m;

    if (!AGC_ASSERT(ip != 0))
    {
        return (0);
    }

    m = agc_w_dotsub(u, v, &ip->ovfind);

    if (!AGC_ASSERT(agc_w_ok(m.m0)))
    {
        return (0);
    }

    return (agc_w_join(agc_w3_top(m)));
}

/* VSQ (p.1182): VSQSUB, DP mode.                                            */
agc_dp_t agc_vec_sq(agc_interp_t *ip, agc_vec_t v)
{
    agc_w3_t m;

    if (!AGC_ASSERT(ip != 0))
    {
        return (0);
    }

    m = agc_w_vsqsub(v, &ip->ovfind);

    if (!AGC_ASSERT(agc_w_ok(m.m0)))
    {
        return (0);
    }

    return (agc_w_join(agc_w3_top(m)));
}

agc_tp_t agc_vec_sq_tp(agc_interp_t *ip, agc_vec_t v)
{
    agc_w3_t m;

    if (!AGC_ASSERT(ip != 0))
    {
        return (0);
    }

    m = agc_w_vsqsub(v, &ip->ovfind);

    if (!AGC_ASSERT(agc_w_ok(m.m0)))
    {
        return (0);
    }

    return (agc_w3_value(m));
}

/* ABVAL (p.1180-1181): VSQSUB, then SQRT of the TP square.                  */
agc_dp_t agc_vec_abval(agc_interp_t *ip, agc_vec_t v)
{
    agc_tp_t sq;

    if (!AGC_ASSERT(ip != 0))
    {
        return (0);
    }

    sq = agc_vec_sq_tp(ip, v);

    if (!AGC_ASSERT(sq >= 0))
    {
        return (0);
    }

    return (agc_tp_sqrt(ip, sq));
}

/* Top two words of DMPSUB(mpac, x).                                         */
static agc_w2_t mul_top(agc_dp_t mpac, agc_dp_t x)
{
    if (!AGC_ASSERT((mpac >= AGC_DP_MIN) && (mpac <= AGC_DP_MAX)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT((x >= AGC_DP_MIN) && (x <= AGC_DP_MAX)))
    {
        return ((agc_w2_t){0}); /* no valid result */
    }

    return (agc_w3_top(agc_w_dmpsub(agc_w_split(mpac), agc_w_split(x))));
}

/* VXV (p.1147-1148): MPAC x X with truncated partial products, each
 * component finished by DAS and the OVERFLOW routine. M = u, X = v.         */
agc_vec_t agc_vec_cross(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    agc_vec_t r;
    agc_w2_t  z;
    agc_w2_t  y;
    agc_w2_t  x;

    if (!AGC_ASSERT(ip != 0))
    {
        return (agc_vec_zero());
    }

    z = mul_top(-u.c[1], v.c[0]); /* -X1M2 in MPAC+5  */
    z = agc_w_dad(z, mul_top(u.c[0], v.c[1]), &ip->ovfind); /* + X2M1    */
    y = mul_top(u.c[2], v.c[0]); /* X1M3 (VBUF +2)   */
    y = agc_w_dad(y, mul_top(-u.c[0], v.c[2]), &ip->ovfind); /* - X3M1    */
    x = mul_top(u.c[1], v.c[2]); /* X3M2             */
    x = agc_w_dad(x, mul_top(-u.c[2], v.c[1]), &ip->ovfind); /* - X2M3    */

    if (!AGC_ASSERT(agc_w_ok(x.hi) && agc_w_ok(y.hi) && agc_w_ok(z.hi)))
    {
        return (agc_vec_zero());
    }

    r.c[0] = agc_w_join(x);
    r.c[1] = agc_w_join(y);
    r.c[2] = agc_w_join(z);

    return (r);
}

/* VPROJ (p.1146): DOT, then DVXSC of the vector at X by the DP dot
 * (MPAC,+1 words): result (u.v) v, each component VROUNDed.               */
agc_vec_t agc_vec_proj(agc_interp_t *ip, agc_vec_t u, agc_vec_t v)
{
    agc_vec_t r = v;
    agc_w2_t  d;
    agc_w3_t  m;
    int       k;

    if (!AGC_ASSERT(ip != 0))
    {
        return (v);
    }

    d = agc_w3_top(agc_w_dotsub(u, v, &ip->ovfind));

    for (k = 0; k < AGC_VEC_N; k++)
    {
        m      = agc_w_dmpsub(d, agc_w_split(v.c[k])); /* DVXSC            */
        m      = agc_w_roundsub(m, &ip->ovfind);       /* VROUND           */
        r.c[k] = agc_w_join(agc_w3_top(m));
    }

    if (!AGC_ASSERT(k == AGC_VEC_N))
    {
        return (v);
    }

    return (r);
}

/* ---- shifts -------------------------------------------------------------- */

/* VSL n (VSSL p.1156): one place at a time, x y z each step, DAS then
 * OVERFLOW per component; wraps and sets OVFIND like the AGC. Negative
 * n -> VSR.                                                                */
agc_vec_t agc_vec_shl(agc_interp_t *ip, agc_vec_t v, int n)
{
    agc_w2_t  p[AGC_VEC_N];
    agc_vec_t r = v;
    int       k;
    int       j;

    if (!AGC_ASSERT(ip != 0))
    {
        return (v);
    }

    if (n < 0)
    {
        return (agc_vec_shr(v, -n));
    }

    for (k = 0; k < AGC_VEC_N; k++)
    {
        p[k] = agc_w_split(v.c[k]);
    }

    if (!AGC_ASSERT(n <= 127)) /* 7-bit count      */
    {
        return (v);
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
        r.c[k] = (n > VSHIFT_MAX) ? 0 : agc_w_join(p[k]);
    }

    return (r);
}

/* VSR n (VSSR/VSHRRND p.1155, VRIGHT2 p.1159): always rounds.              */
agc_vec_t agc_vec_shr(agc_vec_t v, int n)
{
    agc_vec_t r = v;
    int       k;

    if (!AGC_ASSERT(n >= 0) || (n == 0))
    {
        return (v);
    }

    for (k = 0; k < AGC_VEC_N; k++)
    {
        r.c[k] = agc_w_join(agc_w_vright(agc_w_split(v.c[k]), n));
    }

    if (!AGC_ASSERT(k == AGC_VEC_N))
    {
        return (v);
    }

    return (r);
}

/* ---- matrices ------------------------------------------------------------ */

/* Rows are UNITX, UNITY, UNITZ (half-unit), i.e. I/2.                       */
agc_mat_t agc_mat_identity_half(void)
{
    agc_mat_t m;
    int       i;
    int       j;

    for (i = 0; i < AGC_VEC_N; i++)
    {
        for (j = 0; j < AGC_VEC_N; j++)
        {
            m.row[i].c[j] = (i == j) ? AGC_DP_HALF : 0;
        }
    }

    if (!AGC_ASSERT(m.row[0].c[0] == AGC_DP_HALF))
    {
        return ((agc_mat_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(m.row[2].c[1] == 0))
    {
        return ((agc_mat_t){0}); /* no valid result */
    }

    return (m);
}

/* MXV (p.1143-1144): component i = DOTSUB(MPAC vector, row i).              */
agc_vec_t agc_mat_vec(agc_interp_t *ip, const agc_mat_t *m, agc_vec_t v)
{
    agc_vec_t r;
    int       i;

    if (!AGC_ASSERT(ip != 0) || !AGC_ASSERT(m != 0))
    {
        return (agc_vec_zero());
    }

    for (i = 0; i < AGC_VEC_N; i++)
    {
        r.c[i] = agc_vec_dot(ip, v, m->row[i]);
    }

    if (!AGC_ASSERT(i == AGC_VEC_N))
    {
        return (agc_vec_zero());
    }

    return (r);
}

agc_mat_t agc_mat_transpose(const agc_mat_t *m)
{
    agc_mat_t t;
    int       i;
    int       j;

    if (!AGC_ASSERT(m != 0))
    {
        return (agc_mat_identity_half());
    }

    for (i = 0; i < AGC_VEC_N; i++)
    {
        for (j = 0; j < AGC_VEC_N; j++)
        {
            t.row[i].c[j] = m->row[j].c[i];
        }
    }

    if (!AGC_ASSERT(t.row[0].c[1] == m->row[1].c[0]))
    {
        return (agc_mat_identity_half());
    }

    return (t);
}

/* VXM (p.1143-1144): component j = DOTSUB(MPAC vector, column j)
 * (DOTINC = 6 walks down a column).                                        */
agc_vec_t agc_vec_mat(agc_interp_t *ip, agc_vec_t v, const agc_mat_t *m)
{
    agc_vec_t r;
    agc_mat_t t;
    int       j;

    if (!AGC_ASSERT(ip != 0) || !AGC_ASSERT(m != 0))
    {
        return (agc_vec_zero());
    }

    t = agc_mat_transpose(m);

    for (j = 0; j < AGC_VEC_N; j++)
    {
        r.c[j] = agc_vec_dot(ip, v, t.row[j]);
    }

    if (!AGC_ASSERT(j == AGC_VEC_N))
    {
        return (agc_vec_zero());
    }

    return (r);
}

/* MXM3 (ANGLFIND.agc p.404): column j of A B = MXV(A, column j of B), the
 * columns then transposed back (TRNSPSPD).                                  */
agc_mat_t agc_mat_mul(agc_interp_t *ip, const agc_mat_t *a, const agc_mat_t *b)
{
    agc_mat_t cols;
    agc_mat_t bt;
    int       j;

    if (!AGC_ASSERT(ip != 0) || !AGC_ASSERT((a != 0) && (b != 0)))
    {
        return (agc_mat_identity_half());
    }

    bt = agc_mat_transpose(b);

    for (j = 0; j < AGC_VEC_N; j++)
    {
        cols.row[j] = agc_mat_vec(ip, a, bt.row[j]);
    }

    if (!AGC_ASSERT(j == AGC_VEC_N))
    {
        return (agc_mat_identity_half());
    }

    return (agc_mat_transpose(&cols));
}
