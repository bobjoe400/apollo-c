/**********************************************************************
 * FILE NAME: initvel.c
 *
 * PURPOSE:
 *   INITVEL and HAVEGUES, both vehicles: "This subroutine computes the
 *   required initial velocity vector for a trajectory of specific
 *   transfer time between specified initial and target positions. The
 *   trajectory may be either conic or precision depending on an input
 *   parameter (namely, number of offsets). In the precision trajectory
 *   case the subroutine also computes an offset target vector, to be
 *   used during pure-conic cross-product steering."  Inside the cone of
 *   half angle E4 about the 180 degree transfer direction the target is
 *   rotated into the plane of RINIT and VINIT.
 *
 * AGC SOURCE:
 *   Comanche055 P34-35_P74-75.agc pp. 486-491 (INITVEL, HAVEGUES,
 *   INITVEL1 .. INITVELX)
 *   Luminary099 P34-35_P74-75.agc (differs: RTMAG, SETITCTR, MU/A and
 *   MUASTEER at INITVELX)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale        Description
 *   ---------  ---------  ---  -----------  ----------------------------
 *   RINIT      agc_vec_t  I/O  m 2^29       initial position (left at
 *                                           the body scale, as the AGC)
 *   VINIT      agc_vec_t  I/O  m/cs 2^7     initial velocity (likewise)
 *   RTARG      agc_vec_t  I/O  m 2^29       target; offset target out
 *   DELLT4     agc_dp_t   I    cs 2^28      transfer time
 *   INTIME     agc_dp_t   I    cs 2^28      time of RINIT, VINIT
 *   RTX1 RTX2  agc_sp_t   I    -            central body (X1 / X2)
 *   VIPRIME    agc_vec_t  O    m/cs 2^7     required initial velocity
 *   VTPRIME    agc_vec_t  O    m/cs 2^7     velocity at the target
 *   DELVEET3   agc_vec_t  O    m/cs 2^7     VIPRIME - VINIT
 *   RTMAG      agc_dp_t   O    m 2^29/27    |RTARG| (LM)
 *   MU/A MUASTEER agc_dp_t O   -            LM steering constants
 *   CSTH SNTH  agc_dp_t   O    2^1          Lambert debris
 *   COGA       agc_dp_t   I/O  2^5          Lambert guess (file static)
 *
 * EXTERNAL REFERENCES:
 *   Name                 Description
 *   -------------------  ------------------------------------------
 *   agc_conic_lambert    LAMBERT (CM, ITERCTR 20)
 *   agc_integ_setitctr   SETITCTR then LAMBERT (LM)
 *   rz_integrvs          INTEGRVS: precision integration (INITVEL5)
 *   rz_unit rz_abval     UNIT / ABVAL with the AGC scale bookkeeping
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarm: "arrived at solution is good enough", SOLNSW is not
 *   tested (AGC_STATUS_NO_CONVERGE is taken as OK). AGC_STATUS_BAD_ARG
 *   for NULL pointers or NUMIT out of range; an integration status other
 *   than OK (sleep, restart) is returned to the caller.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs inside the caller's job; at most NUMIT + 1 Lambert passes.
 *
 * NOTES:
 *   Scales: r m 2^29 (moon 2^27), v m/cs 2^7 (moon 2^5), t cs 2^28.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"

/* Work cells of one INITVEL call (AGC erasables RTARG1 R1VEC R2VEC UN
 * COZY4 ITCTR VTARGTAG VTARGET and the Lambert inputs).                */
typedef struct iv_work
{
    agc_vec_t         rinit;   /* RINIT, body scale (m 2^29 / 2^27)   */
    agc_vec_t         vinit;   /* VINIT, body scale (m/cs 2^7 / 2^5)  */
    agc_vec_t         rtarg1;  /* RTARG1 target, body scale               */
    agc_vec_t         r2vec;   /* R2VEC (conic, offset) target, body scale */
    agc_vec_t         un;      /* UN unit(RI x VI), 2^1                    */
    agc_vec_t         viprime; /* VIPRIME body scale                       */
    agc_vec_t         vtarget; /* VTARGET body scale                       */
    agc_dp_t          cozy4;   /* COZY4: cos(E4)/4 + URT.URI/4, 2^2         */
    int               itctr;   /* ITCTR: -1, 0, 1, ...                     */
    bool              normsw;  /* NORMSW: UN is a Lambert input             */
    agc_lambert_in_t  lin;     /* LAMBERT inputs (R1VEC R2VEC TDESIRED) */
    agc_lambert_out_t lout;    /* LAMBERT outputs (VVEC VTARGET COGA)   */
} iv_work_t;

/**********************************************************************
 * HAVEGUES .. INITVEL1 head (p.487): body-scale copies, COZY4, UN.
 * AGC: HAVEGUES .. INITVEL1 (P34-35_P74-75.agc p.487)
 *
 * FUNCTION NAME: iv_setup
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   e4        agc_dp_t        I    E4 cone half angle, revolutions 2^0
 *   w         iv_work_t *     O    work cells: body-scale RINIT VINIT
 *                                  RTARG1, COZY4, UN
 **********************************************************************/
static void iv_setup(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t e4, iv_work_t *w)
{
    bool moon = (rz->body.rtx2 != 0); /* moon sphere: rescale */

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

    w->itctr      = -1;
    w->cozy4      = agc_dp_shr(agc_dp_cos(e4), 1); /* COSINE SR1   */
    w->r2vec      = w->rtarg1;
    agc_vec_t uri = rz_unit(ip, w->rinit);         /* URI unit RINIT, 2^1 */

    w->un     = rz_unit(ip, agc_vec_cross(ip, uri, w->vinit));
    w->cozy4  = agc_dp_add(ip, agc_vec_dot(ip, rz_unit(ip, w->rtarg1), uri), w->cozy4);
    w->normsw = false;
}

/**********************************************************************
 * INITVEL3 .. INITVEL4 (pp.488-489): GEOMSGN. X1 = RTX1 - 8 (when the z
 * component of -(UR1 x UR2) is not negative) + 10; X1 = 0 selects the
 * long-way (GEOMSGN negative) transfer. With the earth that is a transfer
 * normal pointing to -z, with the moon one pointing to +z.
 * AGC: INITVEL3 .. INITVEL4 (P34-35_P74-75.agc pp.488-489)
 *
 * FUNCTION NAME: iv_geomsgn
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   w         const iv_work_t *    I    R2VEC and the Lambert R1VEC
 *
 * RETURNS: agc_sp_t -- GEOMSGN, the high word of +-MUEARTH: positive for
 *          the short way
 **********************************************************************/
static agc_sp_t iv_geomsgn(agc_interp_t *ip, const agc_rendz_t *rz, const iv_work_t *w)
{
    /* MUEARTH 2DEC* 3.986032 E10 B-36*: used only as a positive word */
    static const agc_dp_t muearth = 155704375;                 /* MUEARTH, m^3/cs^2 2^36 */
    agc_vec_t             ur2     = rz_unit(ip, w->r2vec);     /* 2D UR2, 2^1 */
    agc_vec_t             ur1     = rz_unit(ip, w->lin.r1vec); /* 8D, 2^1 */
    agc_vec_t             negn;                                /* 14D -(UR1 x UR2), 2^2 */

    negn   = agc_vec_neg(agc_vec_cross(ip, ur1, ur2));
    int x1 = rz->body.rtx1; /* X1 = RTX1 */

    if (negn.c[2] >= 0)
    { /* 18D BMN      */
        x1 -= 8;
    }

    x1          += 10;
    agc_vec_t v = (x1 != 0) ? agc_vec_neg(negn) : negn; /* X1 = 0: long way, 2^2 */
    agc_dp_t  d;                                        /* sign test, 2^5 */

    d          = agc_vec_dot(ip, agc_vec_cross(ip, v, ur1), ur2);
    agc_dp_t g = (d >= 0) ? muearth : agc_dp_neg(muearth); /* GEOMSGN, 2^36 */

    return (agc_dp_high(g));
}

/**********************************************************************
 * LAMBERT (CONIC_SUBROUTINES p.1266); the LM calls it through SETITCTR
 * (ITERCTR 20, or 5 with AVEGFLAG), the CM directly (ITERCTR 20).
 * AGC: LAMBERT (CONIC_SUBROUTINES.agc p.1266), LM SETITCTR
 *
 * FUNCTION NAME: rz_lambert
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   in        agc_lambert_in_t *   I/O  Lambert inputs (ITERCTR set for the
 *                                       CM)
 *   out       agc_lambert_out_t *  O    Lambert solution
 *
 * RETURNS: agc_status_t -- the status of agc_conic_lambert /
 *          agc_integ_setitctr (AGC_STATUS_NO_CONVERGE when SOLNSW is set),
 *          AGC_STATUS_BAD_ARG for NULL pointers
 **********************************************************************/
static agc_status_t rz_lambert(agc_interp_t *ip, agc_lambert_in_t *in, agc_lambert_out_t *out)
{
    if (rz_is_csm())
    {
        in->iterctr = AGC_CONIC_LAMBERT_ITERCTR;

        return (agc_conic_lambert(ip, in, out));
    }

    return (agc_integ_setitctr(ip, in, out));
}

/**********************************************************************
 * LAMBERT (CM) / SETITCTR (LM), p.489. "Arrived at solution is good
 * enough according to slightly wider bounds": SOLNSW is not tested.
 * AGC: INITVEL4 "CALL LAMBERT" (P34-35_P74-75.agc p.489)
 *
 * FUNCTION NAME: iv_lambert
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   numit     agc_sp_t             I    number of offsets (VTARGTAG)
 *   w         iv_work_t *          I/O  Lambert inputs from R2VEC; VIPRIME,
 *                                       VTARGET, COGA out
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (also for a Lambert that did not
 *          converge), else the Lambert status
 **********************************************************************/
static agc_status_t iv_lambert(agc_interp_t *ip, const agc_rendz_t *rz, agc_sp_t numit, iv_work_t *w)
{
    w->lin.r2vec    = w->r2vec;
    w->lin.tdesired = rz->dellt4;
    w->lin.geomsgn  = iv_geomsgn(ip, rz, w);
    w->lin.normsw   = w->normsw;
    w->lin.un       = w->un;
    w->lin.vtargtag = numit;
    w->lin.body     = agc_conic_body_of_x1(rz->body.rtx1);
    agc_status_t st = rz_lambert(ip, &w->lin, &w->lout); /* Lambert status */

    if (st == AGC_STATUS_NO_CONVERGE)
    {
        st = AGC_STATUS_OK;
    }

    w->lin.guessw = false;        /* CLEAR GUESSW       */
    w->lin.coga   = w->lout.coga; /* next guess         */
    w->viprime    = w->lout.vvec;
    w->vtarget    = w->lout.vtarget;

    return (st);
}

/**********************************************************************
 * INITVEL7 .. INITVELX (pp.490-491): DELVEET3, VTPRIME, earth scale, LM
 * MU/A and MUASTEER, RTARG = offset target.
 * AGC: INITVEL7 .. INITVELX (P34-35_P74-75.agc pp.490-491)
 *
 * FUNCTION NAME: iv_finish
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   w         iv_work_t *     I/O  VIPRIME VINIT VTARGET RTARG1 and the
 *                                  Lambert R1 R1A
 **********************************************************************/
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
        agc_dp_t mu = rz_mutable_mu(rz->body.rtx2 != 0); /* MU, body scale */
        agc_dp_t a;                                      /* MU R1A / 2 (scale MU's x R1A's) */

        a = agc_dp_shr(agc_dp_mul_trunc(mu, w->lout.r1a), 1);

        rz->mu_a     = agc_dp_div(ip, a, w->lout.r1); /* MU/A        */
        rz->muasteer = agc_dp_shr(rz->mu_a, 6);       /* SR 6        */
    }

    rz->rtarg = w->rtarg1;
    rz->rinit = w->rinit; /* the AGC leaves them at body scale */
    rz->vinit = w->vinit;
}

/* COGA: cotangent of the flight path angle, Lambert's last solution;
 * an erasable in the AGC, kept between calls for HAVEGUES.            */
static agc_dp_t iv_coga = 0; /* COGA, 2^5 */

/**********************************************************************
 * INITVEL / HAVEGUES body (pp.487-491).
 * AGC: INITVEL / HAVEGUES body (P34-35_P74-75.agc pp.487-491)
 *
 * FUNCTION NAME: iv_run
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   numit     agc_sp_t        I    number of offsets: 0 conic, else
 *                                  precision passes
 *   e4        agc_dp_t        I    E4 cone half angle, revolutions 2^0
 *   guessw    bool            I    GUESSW: true when no COGA guess is
 *                                  available
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration status
 *          (sleep, restart)
 **********************************************************************/
static agc_status_t iv_run(agc_interp_t *ip, agc_rendz_t *rz, agc_sp_t numit, agc_dp_t e4, bool guessw, agc_job_id_t resume)
{
    iv_work_t w; /* INITVEL work cells */

    iv_setup(ip, rz, e4, &w);
    w.lin.r1vec     = w.rinit;       /* R1VEC        */
    w.lin.guessw    = guessw;        /* GUESSW       */
    w.lin.coga      = iv_coga;
    w.lin.iterctr   = AGC_CONIC_LAMBERT_ITERCTR;
    agc_status_t st = AGC_STATUS_OK; /* status of the last step */

    for (int pass = 0; pass <= (int) numit; pass++)
    { /* INITVEL2     */
        if (w.cozy4 < 0)
        {
            /**********************************************************************
             * INITVEL2 rotation (p.488): RC into the RI, VI plane, |RC| kept.
             * AGC: INITVEL2 (P34-35_P74-75.agc p.488)
             *
             * FUNCTION NAME: iv_rotate
             *
             * ARGUMENTS:
             *   Argument  Type            I/O  Description (scale)
             *   --------  --------------  ---  --------------------------------
             *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
             *   w         iv_work_t *     I/O  R2VEC rotated (RTARG1 too on the first
             *                                  pass), NORMSW set
             **********************************************************************/
            w.normsw     = true;
            agc_dp_t  rc = rz_abval(ip, w.r2vec); /* |RC|, body scale (m 2^29/27) */
            agc_vec_t p;                          /* RC . UN UN, body scale */

            p = agc_vec_shl(ip, agc_vec_proj(ip, w.r2vec, w.un), 2);
            agc_vec_t u; /* in-plane unit, 2^1 */

            u = rz_unit(ip, agc_vec_sub(ip, w.r2vec, p));

            w.r2vec = agc_vec_shl(ip, agc_vec_scale(ip, u, rc), 1);

            if (w.itctr < 0)
            {
                w.rtarg1 = w.r2vec; /* first pass   */
            }
        }

        st       = iv_lambert(ip, rz, numit, &w);
        iv_coga  = w.lin.coga;
        rz->csth = w.lout.csth; /* debris       */
        rz->snth = w.lout.snth;

        if (st != AGC_STATUS_OK || numit == 0)
        {
            break; /* INITVEL7     */
        }

        agc_vec_t rend; /* RATT1, body scale */

        /**********************************************************************
         * INITVEL5 (pp.489-490): precision integration of RINIT, VIPRIME over
         * DELLT4; VTARGET = VATT1; *rend = RATT1 (body scale).
         * AGC: INITVEL5 (P34-35_P74-75.agc pp.489-490)
         *
         * FUNCTION NAME: iv_integrate
         *
         * ARGUMENTS:
         *   Argument  Type                 I/O  Description (scale)
         *   --------  -------------------  ---  ---------------------------
         *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
         *   rz        const agc_rendz_t *  I    rendezvous erasables
         *   w         iv_work_t *          I/O  RINIT, VIPRIME in; VTARGET = VATT1
         *                                       out
         *   resume    agc_job_id_t         I    job that continues the caller
         *   rend      agc_vec_t *          O    RATT1, the integrated end position,
         *                                       body scale
         *
         * RETURNS: agc_status_t -- AGC_STATUS_OK, else the status of rz_integrvs
         *          (sleep, restart)
         **********************************************************************/
        rz_integrvs_in_t in; /* INTEGRVS inputs */

        in.moon  = (rz->body.rtx2 != 0);
        in.rcv   = w.rinit;
        in.vcv   = w.viprime;
        in.tet   = rz->intime;
        in.tdec1 = agc_dp_add(ip, rz->intime, rz->dellt4);
        in.conic = false;             /* CLEAR INTYPFLG     */
        agc_integ_out_t o;            /* integration result */
        agc_status_t    st_integrate; /* integration status */

        st_integrate = rz_integrvs(ip, &in, resume, &o);

        if (st_integrate == AGC_STATUS_OK)
        {
            w.vtarget = o.vatt1;
            rend      = o.ratt1;
        }
        st = st_integrate;

        if (st != AGC_STATUS_OK)
        {
            return (st);
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

    return (st);
} /* iv_run */

/**********************************************************************
 * INITVEL (pp.487-491): SET GUESSW (no COGA guess available)
 * AGC: INITVEL (P34-35_P74-75.agc pp.487-491)
 *
 * FUNCTION NAME: agc_rendz_initvel
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   numit     agc_sp_t        I    number of offsets (0 ..
 *                                  AGC_RENDZ_INITVEL_MAXITER - 1)
 *   e4        agc_dp_t        I    E4 cone half angle, revolutions 2^0
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status
 **********************************************************************/
agc_status_t agc_rendz_initvel(agc_interp_t *ip, agc_rendz_t *rz, agc_sp_t numit, agc_dp_t e4, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(numit >= 0 && numit < AGC_RENDZ_INITVEL_MAXITER))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (iv_run(ip, rz, numit, e4, true, resume));
}

/**********************************************************************
 * HAVEGUES (p.487): INITVEL using the COGA of the previous Lambert
 * solution as the first guess (S40.1 cross-product steering).
 * AGC: HAVEGUES (P34-35_P74-75.agc p.487)
 *
 * FUNCTION NAME: agc_rendz_havegues
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   numit     agc_sp_t        I    number of offsets (0 ..
 *                                  AGC_RENDZ_INITVEL_MAXITER - 1)
 *   e4        agc_dp_t        I    E4 cone half angle, revolutions 2^0
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status
 **********************************************************************/
agc_status_t agc_rendz_havegues(agc_interp_t *ip, agc_rendz_t *rz, agc_sp_t numit, agc_dp_t e4, agc_job_id_t resume)
{
    return (iv_run(ip, rz, numit, e4, false, resume));
}
