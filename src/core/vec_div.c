/*
 * agc_vec_div.c -- V/SC and UNIT (INTERPRETER.agc, Comanche055 pp.1150,
 * 1171-1179; Luminary099 pp.1045, 1066-1074).
 *
 * Both divide each component with the GENDDV kernel of agc_divide.c after
 * normalising the divisor. On divide overflow the AGC exits through DVOVF,
 * which writes +-DPPOSMAX into MPAC (the component slot being processed)
 * and leaves the other MPAC slots as they are at that moment; the C code
 * returns exactly that arrangement (see DEVIATION notes for the sign).
 */
#include "agc/core/assert.h"
#include "mpac.h"

#define VDIV_NORM_MAX 28 /* divisor doublings: |s| >= 2^-28         */

/* V/SCDV (p.1173): one component c by the scalar; s_abs = BUF2 (|s|, not
 * normalised), s_norm = BUF (normalised by k doublings). Returns false on
 * DVOVF, leaving +-DPPOSMAX in *c.                                         */
static bool vsc_dv(agc_dp_t *c, agc_dp_t s_abs, agc_dp_t s_norm, int k,
                   bool sneg)
{
    bool     neg = sneg; /* DVSIGN = VBUF+5  */
    agc_dp_t a   = *c;
    agc_w2_t w;
    agc_w2_t d = agc_w_split(s_abs);
    agc_dp_t q;
    (void) AGC_ASSERT((k >= 0) && (k < VDIV_NORM_MAX));
    if (a == 0)
    {
        return true; /* exit on zero     */
    }
    if (a < 0)
    { /* /MPAC-           */
        a   = -a;
        neg = !neg;
    }
    w = agc_w_split(a);
    if ((d.hi - w.hi < 0) || ((d.hi == w.hi) && (d.lo - w.lo <= 0)))
    {
        *c = neg ? AGC_DP_MIN : AGC_DP_MAX; /* DVOVF            */
        return false;
    }
    q  = agc_w_genddv(a * ((agc_dp_t) 1 << k), s_norm, d.hi == w.hi);
    *c = neg ? -q : q;
    return true;
}

/* V/SC with a zero scalar major word (V/SC2 p.1171): shift the vector and
 * scalar left 14. Returns the slot that overflowed, or -1.               */
static int vsc_shift14(agc_dp_t m[AGC_VEC_N], agc_dp_t *s)
{
    int      k;
    agc_w2_t w;
    (void) AGC_ASSERT(s != 0);
    *s = agc_w_split(*s).lo * AGC_W_ONE;
    for (k = 0; k < AGC_VEC_N; k++)
    {
        w    = agc_w_split(m[k]);
        m[k] = w.lo * AGC_W_ONE; /* XCH +1; XCH      */
        if (w.hi != 0)
        {
            return k; /* BZF +2; DVOVF    */
        }
    }
    (void) AGC_ASSERT(k == AGC_VEC_N);
    return (*s == 0) ? 0 : -1; /* zero divisor     */
}

static agc_vec_t vsc_result(agc_dp_t m0, agc_dp_t m3, agc_dp_t m5)
{
    agc_vec_t r;
    r.c[0] = m0; /* MPAC             */
    r.c[1] = m3; /* MPAC +3          */
    r.c[2] = m5; /* MPAC +5          */
    (void) AGC_ASSERT((m0 >= AGC_DP_MIN) && (m0 <= AGC_DP_MAX));
    (void) AGC_ASSERT((m3 >= AGC_DP_MIN) && (m5 <= AGC_DP_MAX));
    return r;
}

/* V/SC (p.1150, 1171-1173): v / s, quotient components scaled 2^0 relative
 * to v/s. |v_i| >= |s| -> DVOVF (OVFIND = 1).                              */
agc_vec_t agc_vec_div(agc_interp_t *ip, agc_vec_t v, agc_dp_t s)
{
    agc_dp_t m[AGC_VEC_N] = {v.c[0], v.c[1], v.c[2]};
    agc_dp_t s_norm;
    bool     sneg = false;
    int      k    = 0;
    int      bad;
    if (!AGC_ASSERT(ip != 0))
    {
        return v;
    }
    if (agc_w_split(s).hi == 0)
    {
        bad = vsc_shift14(m, &s);
        if (bad >= 0)
        {
            ip->ovfind = 1;
            /* DEVIATION: DVSIGN is stale here in the AGC; +DPPOSMAX used.  */
            return vsc_result(AGC_DP_MAX, m[1], m[2]);
        }
    }
    (void) AGC_ASSERT(s != 0);
    if (s < 0)
    { /* /BUF-            */
        s    = -s;
        sneg = true;
    }
    for (s_norm = s; (k < VDIV_NORM_MAX) && (s_norm < AGC_DP_HALF); k++)
    {
        s_norm *= 2; /* /NORM2           */
    }
    if (!vsc_dv(&m[0], s, s_norm, k, sneg))
    {
        ip->ovfind = 1;
        return vsc_result(m[0], m[1], m[2]);
    }
    if (!vsc_dv(&m[1], s, s_norm, k, sneg))
    { /* MPAC = y, +3 = x */
        ip->ovfind = 1;
        return vsc_result(m[1], m[0], m[2]);
    }
    if (!vsc_dv(&m[2], s, s_norm, k, sneg))
    { /* MPAC = z, +5 = y */
        ip->ovfind = 1;
        return vsc_result(m[2], m[0], m[1]);
    }
    return vsc_result(m[0], m[1], m[2]); /* VROTATEX         */
}

/* ---- UNIT ---------------------------------------------------------------- */

/* DVOVF exit of UNIT (p.1175): MPAC = DPPOSMAX, MPAC+3 = 0 and MPAC+5 =
 * the major word VSQSUB left in A after the x^2 + y^2 accumulation.      */
static agc_vec_t unit_overflow(agc_interp_t *ip, agc_vec_t v)
{
    agc_ovf_t o = 0;
    agc_w3_t  acc;
    int32_t   a;
    (void) AGC_ASSERT(ip != 0);
    acc = agc_w_dot_acc(agc_w_dsqsub(agc_w_split(v.c[0])),
                        agc_w_dsqsub(agc_w_split(v.c[1])), &o);
    a   = (o != 0) ? (int32_t) o : acc.m0; /* TS BUF / TS OVFIND */
    (void) AGC_ASSERT(agc_w_ok(a));
    ip->ovfind = 1;
    /* DEVIATION: DVSIGN is stale in the AGC; +DPPOSMAX used.               */
    return vsc_result(AGC_DP_MAX, 0, (agc_dp_t) (a * AGC_W_ONE));
}

/* UNITDV (p.1179) for one component: |c| shifted by the pre-divide count
 * (kdiv < 0: the "-21D" prologue, shift right 1), divided by the root.    */
static agc_dp_t unit_dv(agc_dp_t c, int kdiv, agc_dp_t root)
{
    agc_dp_t a = (c < 0) ? -c : c;
    agc_dp_t q;
    (void) AGC_ASSERT((kdiv >= -1) && (kdiv <= 13));
    (void) AGC_ASSERT(root > AGC_DP_QUARTER); /* normalised root  */
    if (c == 0)
    {
        return 0; /* exit on zero     */
    }
    a = (kdiv < 0) ? (a / 2) : (a * ((agc_dp_t) 1 << kdiv));
    q = agc_w_genddv(a, root, false); /* MAXDVSW = -1     */
    return (c < 0) ? -q : q;
}

/* UNIT (p.1175-1179): v / (2|v|), *abval = LV, *vsq = LVSQUARE.             */
agc_vec_t agc_vec_unit(agc_interp_t *ip, agc_vec_t v, agc_dp_t *abval,
                       agc_dp_t *vsq)
{
    agc_ovf_t o = 0;
    agc_w3_t  sq;
    agc_w2_t  root;
    agc_w3_t  lv    = {0, 0, 0};
    int       shift = 0;
    int       kdiv;
    int       k;
    bool      abort = false;
    if (!AGC_ASSERT(ip != 0))
    {
        return v;
    }
    sq = agc_w_vsqsub(v, &o); /* OVFIND saved     */
    if (o != 0)
    {
        return unit_overflow(ip, v); /* LV not written   */
    }
    if (vsq != 0)
    {
        *vsq = agc_w_join(agc_w3_top(sq)); /* LVSQUARE         */
    }
    root = agc_w_sqrtsub(sq, &shift, &abort);
    (void) AGC_ASSERT(!abort);
    if (root.hi == 0)
    { /* input to SQRT 0  */
        if (abval != 0)
        {
            *abval = 0; /* LV = 0           */
        }
        return unit_overflow(ip, v);
    }
    if (shift > 14)
    { /* SMALL            */
        kdiv  = shift - 15;
        lv.m1 = agc_w_mp((int32_t) 1 << (28 - shift), root.hi).hi;
        for (k = 0; k < AGC_VEC_N; k++)
        {
            (void) AGC_ASSERT(agc_w_split(v.c[k]).hi == 0);
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
        v.c[k] = unit_dv(v.c[k], kdiv, agc_w_join(root));
    }
    return v;
}
