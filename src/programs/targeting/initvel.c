/*
 * initvel.c -- INITVEL (P34-35_P74-75.agc, Comanche055 pp.486-491,
 * Luminary099 the same text plus RTMAG, SETITCTR and MU/A).
 *
 * "This subroutine computes the required initial velocity vector for a
 * trajectory of specific transfer time between specified initial and
 * target positions. The trajectory may be either conic or precision
 * depending on an input parameter (namely, number of offsets). In the
 * precision trajectory case the subroutine also computes an offset target
 * vector, to be used during pure-conic cross-product steering."
 *
 * Inside the cone of half angle E4 about the 180 degree transfer
 * direction the target is rotated into the plane of RINIT and VINIT.
 *
 * Inputs (rendezvous erasables): RINIT m 2^29, VINIT m/cs 2^7, RTARG m
 * 2^29, DELLT4 cs 2^28, INTIME cs 2^28, RTX1/RTX2. Outputs: RTARG (offset
 * target) 2^29, VIPRIME, VTPRIME, DELVEET3 m/cs 2^7. As in the AGC,
 * RINIT and VINIT are left at the primary-body scale (2^27 / 2^5).
 */
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"

/* Work cells of one INITVEL call (AGC erasables RTARG1 R1VEC R2VEC UN
 * COZY4 ITCTR VTARGTAG VTARGET and the Lambert inputs).                */
typedef struct iv_work
{
    agc_vec_t         rinit, vinit; /* body scale                              */
    agc_vec_t         rtarg1; /* RTARG1 target, body scale               */
    agc_vec_t         r2vec;  /* R2VEC (conic, offset) target, body scale */
    agc_vec_t         un;     /* UN unit(RI x VI), 2^1                    */
    agc_vec_t         viprime; /* VIPRIME body scale                       */
    agc_vec_t         vtarget; /* VTARGET body scale                       */
    agc_dp_t          cozy4;  /* COZY4: cos(E4)/4 + URT.URI/4, 2^2         */
    int               itctr;  /* ITCTR: -1, 0, 1, ...                     */
    bool              normsw; /* NORMSW: UN is a Lambert input             */
    agc_lambert_in_t  lin;
    agc_lambert_out_t lout;
} iv_work_t;

/* HAVEGUES .. INITVEL1 head (p.487): body-scale copies, COZY4, UN. */
static void iv_setup(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t e4,
                     iv_work_t *w)
{
    bool moon = (rz->body.rtx2 != 0);

    w->rtarg1 = rz->rtarg;
    rz->rtmag = rz_abval(ip, w->rtarg1); /* LM RTMAG     */
    w->rinit  = rz->rinit;
    w->vinit  = rz->vinit;

    if (moon)
    { /* B29 -> B27   */
        w->rinit  = agc_vec_shl(ip, w->rinit, 2);
        w->vinit  = agc_vec_shl(ip, w->vinit, 2);
        w->rtarg1 = agc_vec_shl(ip, w->rtarg1, 2);
        rz->rtmag = rz_abval(ip, w->rtarg1);
    }

    w->itctr = -1;
    w->cozy4 = agc_dp_shr(agc_dp_cos(ip, e4), 1); /* COSINE SR1   */
    w->r2vec = w->rtarg1;
    agc_vec_t uri = rz_unit(ip, w->rinit); /* 0D           */

    w->un    = rz_unit(ip, agc_vec_cross(ip, uri, w->vinit));
    w->cozy4 =
        agc_dp_add(ip, agc_vec_dot(ip, rz_unit(ip, w->rtarg1), uri), w->cozy4);
    w->normsw = false;

    if (!AGC_ASSERT(w->cozy4 >= AGC_DP_MIN))
    {
        return;
    }
}

/* INITVEL2 rotation (p.488): RC into the RI, VI plane, |RC| kept. */
static void iv_rotate(agc_interp_t *ip, iv_work_t *w)
{
    w->normsw = true;
    agc_dp_t  rc = rz_abval(ip, w->r2vec); /* 0D           */
    agc_vec_t p  = agc_vec_shl(ip, agc_vec_proj(ip, w->r2vec, w->un), 2);
    agc_vec_t u  = rz_unit(ip, agc_vec_sub(ip, w->r2vec, p));

    w->r2vec = agc_vec_shl(ip, agc_vec_scale(ip, u, rc), 1);

    if (w->itctr < 0)
    {
        w->rtarg1 = w->r2vec; /* first pass   */
    }

    if (!AGC_ASSERT(rc >= 0))
    {
        return;
    }
}

/* INITVEL3 .. INITVEL4 (pp.488-489): GEOMSGN. X1 = RTX1 - 8 (when the z
 * component of -(UR1 x UR2) is not negative) + 10; X1 = 0 selects the
 * long-way (GEOMSGN negative) transfer. With the earth that is a transfer
 * normal pointing to -z, with the moon one pointing to +z.             */
static agc_sp_t iv_geomsgn(agc_interp_t *ip, const agc_rendz_t *rz,
                           const iv_work_t *w)
{
    /* MUEARTH 2DEC* 3.986032 E10 B-36*: used only as a positive word */
    static const agc_dp_t muearth = 155704375;
    agc_vec_t             ur2     = rz_unit(ip, w->r2vec); /* 2D           */
    agc_vec_t             ur1     = rz_unit(ip, w->lin.r1vec); /* 8D           */
    agc_vec_t             negn    = agc_vec_neg(agc_vec_cross(ip, ur1, ur2)); /* 14D      */
    int                   x1      = rz->body.rtx1;

    if (negn.c[2] >= 0)
    { /* 18D BMN      */
        x1 -= 8;
    }

    x1 += 10;
    agc_vec_t v = (x1 != 0) ? agc_vec_neg(negn) : negn;
    agc_dp_t  d = agc_vec_dot(ip, agc_vec_cross(ip, v, ur1), ur2);
    agc_dp_t  g = (d >= 0) ? muearth : agc_dp_neg(muearth);

    if (!AGC_ASSERT(x1 >= -8 && x1 <= 8))
    {
        return 0; /* no valid result */
    }

    return agc_dp_high(g);
}

/* LAMBERT (CM) / SETITCTR (LM), p.489. "Arrived at solution is good
 * enough according to slightly wider bounds": SOLNSW is not tested.    */
static agc_status_t iv_lambert(agc_interp_t *ip, const agc_rendz_t *rz,
                               agc_sp_t numit, iv_work_t *w)
{
    w->lin.r2vec    = w->r2vec;
    w->lin.tdesired = rz->dellt4;
    w->lin.geomsgn  = iv_geomsgn(ip, rz, w);
    w->lin.normsw   = w->normsw;
    w->lin.un       = w->un;
    w->lin.vtargtag = numit;
    w->lin.body     = rz_body(rz);
    agc_status_t st = rz_lambert(ip, &w->lin, &w->lout);

    if (st == AGC_STATUS_NO_CONVERGE)
    {
        st = AGC_STATUS_OK;
    }

    w->lin.guessw = false;        /* CLEAR GUESSW       */
    w->lin.coga   = w->lout.coga; /* next guess         */
    w->viprime    = w->lout.vvec;
    w->vtarget    = w->lout.vtarget;

    return st;
}

/* INITVEL5 (pp.489-490): precision integration of RINIT, VIPRIME over
 * DELLT4; VTARGET = VATT1; *rend = RATT1 (body scale).                 */
static agc_status_t iv_integrate(agc_interp_t *ip, const agc_rendz_t *rz,
                                 iv_work_t *w, agc_job_id_t resume,
                                 agc_vec_t *rend)
{
    rz_integrvs_in_t in;

    in.moon  = (rz->body.rtx2 != 0);
    in.rcv   = w->rinit;
    in.vcv   = w->viprime;
    in.tet   = rz->intime;
    in.tdec1 = agc_dp_add(ip, rz->intime, rz->dellt4);
    in.conic = false; /* CLEAR INTYPFLG     */
    agc_integ_out_t o;
    agc_status_t    st = rz_integrvs(ip, &in, resume, &o);

    if (st == AGC_STATUS_OK)
    {
        w->vtarget = o.vatt1;
        *rend      = o.ratt1;
    }

    return st;
}

/* INITVEL7 .. INITVELX (pp.490-491): DELVEET3, VTPRIME, earth scale, LM
 * MU/A and MUASTEER, RTARG = offset target.                            */
static void iv_finish(agc_interp_t *ip, agc_rendz_t *rz, iv_work_t *w)
{
    rz->delveet3 = agc_vec_sub(ip, w->viprime, w->vinit);
    rz->vtprime  = w->vtarget;
    rz->viprime  = w->viprime;

    if (rz->body.rtx2 != 0)
    { /* VSR2        */
        rz->vtprime  = agc_vec_shr(rz->vtprime, 2);
        rz->viprime  = agc_vec_shr(rz->viprime, 2);
        w->rtarg1    = agc_vec_shr(w->rtarg1, 2);
        rz->delveet3 = agc_vec_shr(rz->delveet3, 2);
    }

    if (!rz_is_csm())
    { /* LM INITVELX */
        agc_dp_t mu = rz_mutable_mu(rz->body.rtx2 != 0);
        agc_dp_t a  = agc_dp_shr(agc_dp_mul_trunc(mu, w->lout.r1a), 1);

        rz->mu_a     = agc_dp_div(ip, a, w->lout.r1); /* MU/A        */
        rz->muasteer = agc_dp_shr(rz->mu_a, 6);       /* SR 6        */
    }

    rz->rtarg = w->rtarg1;
    rz->rinit = w->rinit; /* the AGC leaves them at body scale */
    rz->vinit = w->vinit;
}

/* COGA: cotangent of the flight path angle, Lambert's last solution;
 * an erasable in the AGC, kept between calls for HAVEGUES.            */
static agc_dp_t iv_coga = 0;

/* INITVEL / HAVEGUES body (pp.487-491). */
static agc_status_t iv_run(agc_interp_t *ip, agc_rendz_t *rz, agc_sp_t numit,
                           agc_dp_t e4, bool guessw, agc_job_id_t resume)
{
    iv_work_t w;

    iv_setup(ip, rz, e4, &w);
    w.lin.r1vec   = w.rinit;   /* R1VEC        */
    w.lin.guessw  = guessw;    /* GUESSW       */
    w.lin.coga    = iv_coga;
    w.lin.iterctr = AGC_CONIC_LAMBERT_ITERCTR;
    agc_status_t st = AGC_STATUS_OK;

    for (int pass = 0; pass <= (int) numit; pass++)
    { /* INITVEL2     */
        if (w.cozy4 < 0)
        {
            iv_rotate(ip, &w);
        }

        st       = iv_lambert(ip, rz, numit, &w);
        iv_coga  = w.lin.coga;
        rz->csth = w.lout.csth; /* debris       */
        rz->snth = w.lout.snth;

        if (st != AGC_STATUS_OK || numit == 0)
        {
            break; /* INITVEL7     */
        }

        agc_vec_t rend;

        st = iv_integrate(ip, rz, &w, resume, &rend);

        if (st != AGC_STATUS_OK)
        {
            return st;
        }

        w.itctr++; /* INCR ITCTR   */

        if (w.itctr == (int) numit)
        {
            w.rtarg1 = w.r2vec; /* INITVEL6     */
            break;
        }

        w.r2vec = agc_vec_add(ip, agc_vec_sub(ip, w.rtarg1, rend), w.r2vec);
    }

    if (st == AGC_STATUS_OK)
    {
        iv_finish(ip, rz, &w);
    }

    return st;
}

/* ---- INITVEL (pp.487-491): SET GUESSW (no COGA guess available) ---------- */
agc_status_t agc_rendz_initvel(agc_interp_t *ip, agc_rendz_t *rz,
                               agc_sp_t numit, agc_dp_t e4, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(numit >= 0 && numit < AGC_RENDZ_INITVEL_MAXITER))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return iv_run(ip, rz, numit, e4, true, resume);
}

/* ---- HAVEGUES (p.487): INITVEL using the COGA of the previous Lambert
 * solution as the first guess (S40.1 cross-product steering).           */
agc_status_t agc_rendz_havegues(agc_interp_t *ip, agc_rendz_t *rz,
                                agc_sp_t numit, agc_dp_t e4,
                                agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(numit >= 0 && numit < AGC_RENDZ_INITVEL_MAXITER))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return iv_run(ip, rz, numit, e4, false, resume);
}
