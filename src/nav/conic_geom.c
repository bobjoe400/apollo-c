/*
 * conic_geom.c -- the geometry and x-guess subroutines of lower rank:
 * GEOM, PARAM and GETX (WLOOP, POLYCOEF, XCOMMON, RESETX2, 360CHECK,
 * INVRSEQN, 1/WLOOP, TRUE360X, INFINITY, OVFLCLR)
 * (CONIC_SUBROUTINES.agc, Comanche 055 pp. 1289-1295, Luminary 099
 * pp. 1185-1191; identical code in both).
 *
 * GETX finds the universal variable x of the conic through a given true-
 * anomaly difference (CSTH, SNTH) from the conic parameters P, R1A, COGA
 * by the continued-fraction form of GSOP 5.5: three half-angle steps
 * W(k+1) = W(k) + sqrt(W(k)^2 + R1A) (WLOOP), or for transfers near 360
 * degrees the inverse form (INVRSEQN, 1/WLOOP), then a 7-term power series
 * (POLYCOEF), then x, xi, KEPC1 and KEPC2 for DELTIME (XCOMMON).
 */
#include "conic_internal.h"
#include "agc/agc_assert.h"

/* POLYCOEF series (p. 1293), argument R1A/W^2 at 2^0, assembled 2DEC. */
static const agc_dp_t polycoef_coef[AGC_CONIC_POLYCOEF_NCOEF] = {
    134217728,  /*  .5          20000 00000 */
    -44739270,  /* -.166666770  72525 52471 */
     26843651,  /*  .100000392  03146 15003 */
    -19166583,  /* -.071401086  75556 45210 */
     14899051,  /*  .055503292  01615 13553 */
    -12687360,  /* -.047264098  76371 63777 */
     10923767   /*  .040694204  01232 27367 */
};

/* Where GETX goes next after one of its labelled blocks. */
typedef enum getx_next {
    GETX_TO_WLOOP = 0,      /* MPAC = W0 (2^5)                             */
    GETX_TO_POLYCOEF,       /* MPAC = 1/W (2^2) or 16/W (2^6)              */
    GETX_TO_360CHECK,       /* BOV taken; MPAC holds the limited value     */
    GETX_TO_INFINITY        /* closure through infinity                    */
} getx_next_t;

/*
 * GEOM (CONIC_SUBROUTINES.agc p. 1291, labels GEOM, UNITNORM, COLINEAR,
 * HAVENORM). In: r1vec (0D), v2vec (MPAC), ctx->normsw, ctx->geomsgn,
 * ctx->un (if normsw). Out: U2, MAGVEC2 = |v2vec|, UR1, R1 = |r1vec|, UN,
 * *csth (0D) and *snth (MPAC), both 2^1.
 */
agc_status_t agc_conic_geom_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                             agc_vec_t r1vec, agc_vec_t v2vec,
                             agc_dp_t *snth, agc_dp_t *csth)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(snth != NULL && csth != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(ctx->geomsgn != 0)) { return AGC_STATUS_BAD_ARG; }
    agc_dp_t mag = 0;
    ctx->u2 = agc_vec_unit(ip, v2vec, &mag, NULL);  /* UNIT, STODL U2 / 36D */
    ctx->magvec2 = mag;                             /* STOVL MAGVEC2        */
    ctx->ur1 = agc_vec_unit(ip, r1vec, &mag, NULL); /* UNIT, STORE UR1      */
    agc_tp_t m = agc_conic_mload_(agc_vec_dot(ip, ctx->ur1, ctx->u2));
    *csth = agc_conic_mstore_(agc_conic_mshift_(ip, m, 1)); /* DOT SL1: 0D */
    ctx->r1 = mag;                                  /* STOVL R1 / UR1       */
    agc_vec_t n = agc_vec_shl(ip, agc_vec_cross(ip, ctx->ur1, ctx->u2), 1);
    if (ctx->normsw) {                              /* HAVENORM: ABVAL SIGN */
        m = agc_conic_mload_(agc_vec_abval(ip, n));
        *snth = agc_conic_mstore_(agc_conic_msign_sp_(m, ctx->geomsgn));
        return AGC_STATUS_OK;
    }
    n = agc_conic_vsign_sp_(n, ctx->geomsgn);       /* SIGN GEOMSGN, UNIT   */
    agc_dp_t sn = 0;
    agc_vec_t un = agc_vec_unit(ip, n, &sn, NULL);
    if (agc_interp_test_ovf(ip)) {                  /* BOV COLINEAR: VSR1   */
        un = agc_vec_shr(un, 1);
    }
    ctx->un = un;                                   /* UNITNORM: STODL UN   */
    *snth = agc_conic_mstore_(agc_conic_msign_sp_(agc_conic_mload_(sn),
                                                  ctx->geomsgn));
    return AGC_STATUS_OK;
}

/*
 * PARAM (CONIC_SUBROUTINES.agc pp. 1289-1290, label PARAM)
 * In: rvec (0D) 2^29/2^27, vvec (MPAC) 2^7/2^5. Out: ctx->coga 2^5,
 * r1a 2^6, p 2^4, r1, ur1, u2, un, magvec2; *sngamma (36D) 2^1.
 * Clears NORMSW and COGAFLAG. OVFIND is left set if COGA overflowed
 * (callers test it with BOV and go to COGAOVFL).
 */
agc_status_t agc_conic_param_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                              agc_vec_t rvec, agc_vec_t vvec, agc_dp_t *sngamma)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(sngamma != NULL)) { return AGC_STATUS_BAD_ARG; }
    ctx->normsw = false;                            /* CLEAR NORMSW         */
    ctx->cogaflag = false;                          /* CLEAR COGAFLAG       */
    (void)agc_conic_mirror_flag_(AGC_CONIC_FLAG_COGAFLAG, false); /* mirror only */
    ctx->geomsgn = AGC_CONIC_GEOMSGN_POS;           /* gamma < 180 deg      */
    agc_dp_t sng = 0;
    agc_dp_t csg = 0;
    agc_status_t st = agc_conic_geom_(ip, ctx, rvec, vvec, &sng, &csg);
    if (st != AGC_STATUS_OK) { return st; }
    *sngamma = sng;                                 /* STODL 36D            */
    agc_tp_t m = agc_conic_mshift_(ip, agc_conic_mload_(csg), -5);
    ctx->coga = agc_conic_mstore_(agc_conic_mddv_(ip, m, sng)); /* SR 5 DDV */
    int n = 0;                                      /* MAGVEC2 DSQ NORM X1  */
    m = agc_conic_mnorm_(agc_conic_mdsq_(agc_conic_mload_(ctx->magvec2)), &n);
    m = agc_conic_mdmpr_(ip, m, ctx->inv_mu);       /* DMPR 1/MU, DMP R1    */
    m = agc_conic_mdmp_(m, ctx->r1);
    m = agc_conic_mshiftr_(ip, m, 3 - n);           /* SRR* 0 -3,1          */
    const agc_dp_t r1v2mu = agc_conic_mstore_(m);   /* PUSH: R1 V^2/MU 2^6  */
    ctx->r1a = agc_conic_mstore_(agc_conic_mbdsu_(ip, m, AGC_CONIC_D1_32));
    m = agc_conic_mdmp_(agc_conic_mload_(r1v2mu), sng); /* DMP 36D, NORM X1 */
    m = agc_conic_mnorm_(m, &n);
    m = agc_conic_mdmp_(m, sng);                    /* DMP 36D, SR* 0 -4,1  */
    ctx->p = agc_conic_mstore_(agc_conic_mshift_(ip, m, 4 - n)); /* P 2^4  */
    return AGC_STATUS_OK;
}

/* INFINITY / OVFLCLR (p. 1295): no solution, closure through infinity.
 * SETPD 0, BOV OVFLCLR (clears OVFIND), SET INFINFLG, RVQ.               */
static agc_status_t getx_infinity(agc_interp_t *ip, agc_conic_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL)) { return AGC_STATUS_BAD_ARG; }
    (void)agc_interp_test_ovf(ip);                  /* BOV: clear OVFIND    */
    ctx->infinflg = true;
    agc_status_t st = agc_conic_mirror_flag_(AGC_CONIC_FLAG_INFINFLG, true);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return AGC_STATUS_OK;
}

/* GETX entry to WLOOP (p. 1292): W0 = (SNTH/(1-CSTH) - COGA) sqrt(P),
 * 2^5. Returns GETX_TO_360CHECK if either BOV is taken.                   */
static getx_next_t getx_first_w(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                                agc_tp_t *mpac)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(mpac != NULL);
    agc_tp_t m = agc_conic_msqrt_(ip, agc_conic_mload_(ctx->p)); /* SQRT    */
    const agc_dp_t sqrtp = agc_conic_mstore_(m);                 /* 0D 2^2  */
    m = agc_conic_mshift_(ip, agc_conic_mload_(ctx->csth), -1);  /* SR1     */
    const agc_dp_t one_m_csth = agc_conic_mstore_(
        agc_conic_mbdsu_(ip, m, AGC_CONIC_D1_4));                /* 2D 2^2  */
    m = agc_conic_mshiftr_(ip, agc_conic_mload_(ctx->snth), -6); /* SRR 6   */
    m = agc_conic_mddv_(ip, m, one_m_csth);                      /* 2^5     */
    if (agc_interp_test_ovf(ip)) { *mpac = m; return GETX_TO_360CHECK; }
    m = agc_conic_mdsu_(ip, m, ctx->coga);                       /* DSU COGA */
    m = agc_conic_mdmp_(m, sqrtp);
    m = agc_conic_mshiftr_(ip, m, 2);                            /* SL2R    */
    *mpac = m;
    return agc_interp_test_ovf(ip) ? GETX_TO_360CHECK : GETX_TO_WLOOP;
}

/* WLOOP (p. 1292): three passes of W = W + sqrt(W^2 + R1A) (2^5), then
 * BDDV D1/128 to 1/W (2^2). BOV in the loop goes to RESETX2/360CHECK.    */
static getx_next_t getx_wloop(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                              agc_tp_t *mpac)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL)) { return GETX_TO_INFINITY; }
    if (!AGC_ASSERT(mpac != NULL)) { return GETX_TO_INFINITY; }
    agc_tp_t m = *mpac;
    for (int pass = 0; pass < AGC_CONIC_GETX_PASSES; pass++) {  /* TIX,2 */
        const agc_dp_t w = agc_conic_mstore_(m);                /* PUSH 0D */
        const agc_tp_t wsq = agc_conic_mdsq_(m);                /* TLOAD MPAC */
        m = agc_conic_mshift_(ip, agc_conic_mload_(ctx->r1a), -4); /* SR4  */
        m = agc_conic_mtad_(ip, m, wsq);                        /* 2^10    */
        if (agc_conic_mbmn_(m)) { return GETX_TO_INFINITY; }
        m = agc_conic_mround_(ip, agc_conic_msqrt_(ip, m));     /* SQRT ROUND */
        m = agc_conic_mdad_(ip, m, w);                          /* DAD 0D  */
        if (agc_interp_test_ovf(ip)) { *mpac = m; return GETX_TO_360CHECK; }
    }
    m = agc_conic_mbddv_(ip, m, AGC_CONIC_D1_128);              /* 1/W 2^2 */
    *mpac = m;
    return agc_interp_test_ovf(ip) ? GETX_TO_INFINITY : GETX_TO_POLYCOEF;
}

/* 360CHECK / INVRSEQN (p. 1294): near-360-degree form. 1/W at 2^-1 from
 * SNTH / (sqrt(P) (1 + CSTH - SNTH COGA)), its square to 34D.            */
static agc_dp_t getx_invrseqn(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                              agc_dp_t *inv_w_sq)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(inv_w_sq != NULL);
    agc_tp_t m = agc_conic_msqrt_(ip, agc_conic_mload_(ctx->p));
    const agc_dp_t sqrtp = agc_conic_mstore_(m);                 /* 0D 2^2  */
    m = agc_conic_mdmp_(agc_conic_mload_(ctx->snth), ctx->coga); /* DMP SL1 */
    const agc_dp_t sn_coga = agc_conic_mstore_(agc_conic_mshift_(ip, m, 1));
    m = agc_conic_mshift_(ip, agc_conic_mload_(ctx->csth), -4);  /* SR4     */
    m = agc_conic_mdad_(ip, m, AGC_CONIC_D1_32);                 /* 1+CSTH  */
    m = agc_conic_mdsu_(ip, m, sn_coga);
    m = agc_conic_mdmp_(m, sqrtp);                               /* 2^7     */
    int n = 0;
    m = agc_conic_mnorm_(m, &n);
    m = agc_conic_mbddv_(ip, m, ctx->snth);                      /* BDDV SNTH */
    m = agc_conic_mabs_(agc_conic_mshiftr_(ip, m, n - 5));       /* SLR* 0 -5,1 */
    *inv_w_sq = agc_conic_mstore_(agc_conic_mdsq_(m));           /* 34D 2^-2 */
    return agc_conic_mstore_(m);                                 /* 0D 2^-1 */
}

/* 1/WLOOP (p. 1294): G = G + sqrt(G^2 + R1A/W^2) three times from
 * G = 1 (D1/16 at 2^4); then MPAC = (1/W)/G at 2^2 for POLYCOEF.         */
static getx_next_t getx_inv_wloop(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                                  agc_tp_t *mpac)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL)) { return GETX_TO_INFINITY; }
    if (!AGC_ASSERT(mpac != NULL)) { return GETX_TO_INFINITY; }
    agc_dp_t inv_w_sq = 0;
    const agc_dp_t inv_w = getx_invrseqn(ip, ctx, &inv_w_sq);
    agc_tp_t m = agc_conic_mload_(AGC_CONIC_D1_16);
    for (int pass = 0; pass < AGC_CONIC_GETX_PASSES; pass++) {  /* TIX,2 */
        const agc_dp_t g = agc_conic_mstore_(m);                /* PUSH 2D */
        const agc_tp_t gsq = agc_conic_mdsq_(m);                /* TPMODE  */
        m = agc_conic_mdmp_(agc_conic_mload_(ctx->r1a), inv_w_sq);
        m = agc_conic_mshift_(ip, m, -4);                       /* SR4     */
        m = agc_conic_mtad_(ip, m, gsq);                        /* 2^8     */
        if (agc_conic_mbmn_(m)) { return GETX_TO_INFINITY; }
        m = agc_conic_mdad_(ip, agc_conic_msqrt_(ip, m), g);    /* DAD 2D  */
    }
    int n = 0;
    m = agc_conic_mnorm_(m, &n);                                /* NORM X1 */
    m = agc_conic_mbddv_(ip, m, inv_w);                         /* BDDV 0D */
    *mpac = agc_conic_mshiftr_(ip, m, n - 7);                   /* SLR* 0 -7,1 */
    return GETX_TO_POLYCOEF;
}

/* POLYCOEF and TRUE360X (pp. 1292-1295): from MPAC = 1/W the series
 * value x' (0D, 2^6 equivalent), corrected to 2 pi/sqrt(R1A) - x' when
 * 360SW is set. Returns false for INFINITY.                              */
static bool getx_polycoef(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                          agc_tp_t m_in, agc_tp_t *xprime)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(xprime != NULL);
    if (agc_conic_mbmn_(m_in)) { return false; }                /* BMN     */
    const agc_dp_t pw = agc_conic_mstore_(m_in);                /* PUSH 0D */
    int n = 0;
    agc_tp_t m = agc_conic_mnorm_(agc_conic_mdsq_(m_in), &n);   /* DSQ NORM */
    m = agc_conic_mdmp_(m, ctx->r1a);
    m = agc_conic_mshiftr_(ip, m, 10 - n);                      /* SRR* 0 -10D,1 */
    m = agc_conic_mload_(agc_conic_poly_(agc_conic_mstore_(m), polycoef_coef,
                                         AGC_CONIC_POLYCOEF_NCOEF));
    m = agc_conic_mshiftr_(ip, agc_conic_mdmp_(m, pw), 1);      /* DMP SL1R */
    if (ctx->sw360) {                                           /* TRUE360X */
        const agc_dp_t xp = agc_conic_mstore_(m);
        agc_tp_t r = agc_conic_mload_(ctx->r1a);
        if (agc_conic_mbmn_(r)) { return false; }
        r = agc_conic_mnorm_(agc_conic_msqrt_(ip, r), &n);
        r = agc_conic_mbddv_(ip, r, AGC_CONIC_2PISC);           /* BDDV 2PISC */
        r = agc_conic_mshift_(ip, r, n - 3);                    /* SL* 0 -3,1 */
        m = agc_conic_mdsu_(ip, r, xp);                         /* DSU PUSH */
    }
    *xprime = m;
    return true;
}

/* XCOMMON (pp. 1293-1294): XI, X, 0D = XSQ normalized (N1), KEPC1, KEPC2,
 * and CLEAR INFINFLG.                                                    */
static void getx_xcommon(agc_interp_t *ip, agc_conic_ctx_t *ctx, agc_tp_t xprime)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(xprime >= AGC_TP_MIN && xprime <= AGC_TP_MAX);
    int n = 0;
    agc_tp_t m = agc_conic_mnorm_(agc_conic_mdsq_(xprime), &n); /* DSQ NORM */
    m = agc_conic_mdmp_(m, ctx->r1a);
    ctx->xi = agc_conic_mstore_(agc_conic_mshiftr_(ip, m, 12 - n)); /* XI 2^6 */
    m = agc_conic_mshift_(ip, agc_conic_mload_(ctx->r1), -1);   /* R1 SR1  */
    m = agc_conic_mround_(ip, agc_conic_msqrt_(ip, m));         /* SQRT ROUND */
    m = agc_conic_mshiftr_(ip, agc_conic_mdmp_(m, agc_conic_mstore_(xprime)), 4);
    ctx->x = agc_conic_mstore_(m);                              /* X 2^17  */
    m = agc_conic_mnorm_(agc_conic_mdsq_(m), &n);               /* DSQ NORM X1 */
    ctx->xsq = agc_conic_mstore_(m);                            /* 0D      */
    ctx->xsq_n1 = n;
    m = agc_conic_mshift_(ip, agc_conic_mdmp_(agc_conic_mload_(ctx->p), ctx->r1), 3);
    m = agc_conic_mdmp_(agc_conic_msqrt_(ip, m), ctx->coga);    /* SQRT DMP COGA */
    ctx->kepc1 = agc_conic_mstore_(agc_conic_mshiftr_(ip, m, 3)); /* SL3R 2^17 */
    m = agc_conic_mbdsu_(ip, agc_conic_mload_(ctx->r1a), AGC_CONIC_D1_64);
    ctx->infinflg = false;                                      /* CLEAR INFINFLG */
    (void)agc_conic_mirror_flag_(AGC_CONIC_FLAG_INFINFLG, false); /* mirror only */
    ctx->kepc2 = agc_conic_mstore_(m);                          /* KEPC2 2^6 */
}

/*
 * GETX (CONIC_SUBROUTINES.agc pp. 1292-1295, label GETX)
 * In: ctx->p (MPAC) 2^4, csth, snth 2^1, coga 2^5, r1a 2^6, r1.
 * Out: ctx->x, xi, xsq/xsq_n1, kepc1, kepc2, sw360, infinflg (INFINFLG
 * set and nothing else valid when closure through infinity is required).
 */
agc_status_t agc_conic_getx_(agc_interp_t *ip, agc_conic_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(ctx->p >= AGC_DP_MIN && ctx->p <= AGC_DP_MAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    ctx->sw360 = false;                           /* AXT,2 3; SSP S2 1; CLEAR 360SW */
    agc_tp_t m = 0;
    getx_next_t next = getx_first_w(ip, ctx, &m);
    if (next == GETX_TO_WLOOP) {
        next = getx_wloop(ip, ctx, &m);
    }
    if (next == GETX_TO_360CHECK) {               /* RESETX2 / 360CHECK   */
        ctx->sw360 = agc_conic_mbmn_(m);          /* BPL INVRSEQN / SET 360SW */
        next = getx_inv_wloop(ip, ctx, &m);
    }
    agc_tp_t xprime = 0;
    if (next != GETX_TO_POLYCOEF || !getx_polycoef(ip, ctx, m, &xprime)) {
        return getx_infinity(ip, ctx);
    }
    getx_xcommon(ip, ctx, xprime);
    return AGC_STATUS_OK;
}

/*
 * PARAM, public entry (P37 and others call PARAM directly).
 */
agc_status_t agc_conic_param(agc_interp_t *ip, agc_vec_t rvec, agc_vec_t vvec,
                             agc_conic_body_t body, agc_conic_param_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL)) { return AGC_STATUS_BAD_ARG; }
    agc_conic_ctx_t ctx;
    agc_status_t st = agc_conic_ctx_init_(&ctx, body);
    if (st != AGC_STATUS_OK) { return st; }
    st = agc_conic_param_(ip, &ctx, rvec, vvec, &out->sngamma);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) { return st; }
    out->p = ctx.p;
    out->r1a = ctx.r1a;
    out->coga = ctx.coga;
    out->r1 = ctx.r1;
    out->magvec2 = ctx.magvec2;
    out->ur1 = ctx.ur1;
    out->u2 = ctx.u2;
    out->un = ctx.un;
    return AGC_STATUS_OK;
}
