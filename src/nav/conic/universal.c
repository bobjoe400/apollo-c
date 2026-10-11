/*
 * universal.c -- the universal-variable time equation and the f and g
 * state update shared by KEPLER, LAMBERT, TIME-THETA and TIME-RADIUS:
 * DELTIME (with its S and C series), NEWSTATE, LAMENTER and the KEPLER
 * final step KEPCONVG (CONIC_SUBROUTINES.agc, Comanche 055 pp. 1281-1287,
 * Luminary 099 pp. 1177-1183; identical code in both).
 *
 * Notation (GSOP 5.5): x universal anomaly, xi = alpha x^2, S(xi) and C(xi)
 * the Stumpff series, r1/v1 the initial state, mu the central-body gravity.
 *   sqrt(mu) t = KEPC1 x^2 C + x (KEPC2 x^2 S + r1)      (DELTIME)
 *   r2 = (r1 - x^2 C) ur1 + (t - x^3 S / sqrt mu) v1     (NEWSTATE)
 *   v2 = sqrt(mu) x (xi S - 1) / r2 ur1 + (1 - x^2 C / r2) v1  (LAMENTER)
 */
#include "agc/core/assert.h"
#include "internal.h"

/* DELTIME POLY coefficients for S(XI) (2^1) and C(XI) (2^4), argument XI
 * at 2^6 (p. 1283). Assembled 2DEC values; octal in the comments.        */
static const agc_dp_t deltime_s_coef[AGC_CONIC_DELTIME_NCOEF] = {
    22369622,  /*  .083333334  02525 12526 */
    -71582793, /* -.266666684  67356 75666 */
    109078521, /*  .406349155  15001 23771 */
    -96958531, /* -.361198675  64342 43674 */
    56412581,  /*  .210153242  06563 04645 */
    -23145029, /* -.086221951  75173 52672 */
    7051481,   /*  .026268812  00656 14331 */
    -1654453,  /* -.006163316  77633 40512 */
    316040,    /*  .001177342  00023 11210 */
    -53433     /* -.000199055  77774 67506 */
};

static const agc_dp_t deltime_c_coef[AGC_CONIC_DELTIME_NCOEF] = {
    8388608,    /*  .031250001  01000 00000 */
    -44739257,  /* -.166666719  72525 52506 */
    95443679,   /*  .355555413  13301 15337 */
    -109078052, /* -.406347410  62776 54733 */
    77567671,   /*  .288962094  11176 13267 */
    -37612611,  /* -.140117894  73410 51674 */
    13219745,   /*  .049247387  01446 33641 */
    -3511652,   /* -.013081923  77451 65233 */
    753334,     /*  .002806389  00055 37266 */
    -142113     /* -.000529414  77767 52336 */
};

/*
 * DELTIME (CONIC_SUBROUTINES.agc pp. 1283-1284, label DELTIME)
 * In:  ctx->xi (MPAC) 2^6, ctx->xsq (0D) X^2 normalized 2^(34-N1)/2^(32-N1)
 *      with ctx->xsq_n1 = N1, ctx->x, kepc1, kepc2, r1, inv_root_mu.
 * Out: ctx->t (T) 2^28 cs, ctx->s_xi (S(XI)) 2^1, ctx->xsqc_xi 2^33/2^31.
 * OVFIND is left set in *ip if T overflowed (the callers' BOV).
 */
agc_status_t agc_conic_deltime_(agc_interp_t *ip, agc_conic_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->xsq_n1 >= 0 && ctx->xsq_n1 <= AGC_TP_BITS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const int n1 = ctx->xsq_n1;

    /* TC POLY ... STODL S(XI) / XI ... TC POLY: C(XI) 2^4 */
    ctx->s_xi =
        agc_conic_poly_(ctx->xi, deltime_s_coef, AGC_CONIC_DELTIME_NCOEF);
    agc_tp_t m = agc_tp_from_dp(
        agc_conic_poly_(ctx->xi, deltime_c_coef, AGC_CONIC_DELTIME_NCOEF));

    m            = agc_conic_mdmp_(m, ctx->xsq);      /* DMP 0D               */
    m            = agc_conic_mshiftr_(ip, m, 5 - n1); /* SRR* 0 -5,1          */
    ctx->xsqc_xi = agc_tp_to_dp(m);                /* XSQC(XI) 2^33        */
    m            = agc_conic_mdmp_(m, ctx->kepc1); /* DMP KEPC1, SL1       */
    m            = agc_tp_shift(ip, m, 1);
    const agc_tp_t c1_xsq_c = m;           /* 0D TP, 2^49          */

    m = agc_tp_from_dp(ctx->xsq);        /* XCH with PL          */
    m = agc_conic_mdmp_(m, ctx->s_xi);   /* DMP S(XI)            */
    m = agc_conic_mshiftr_(ip, m, 5 - n1); /* SRR* 0 -5,1          */
    m = agc_conic_mdmp_(m, ctx->kepc2);    /* DMP KEPC2, SL1       */
    m = agc_tp_shift(ip, m, 1);
    const agc_tp_t c2_xsq_s = m;        /* 3D TP, 2^35          */

    m = agc_tp_from_dp(ctx->r1); /* DLOAD R1, SR 6       */
    m = agc_tp_shift(ip, m, -6);
    m = agc_tp_add(ip, m, c2_xsq_s); /* TAD                  */
    int n3 = 0;

    m      = agc_conic_mnorm_(m, &n3);         /* NORM X1              */
    m      = agc_conic_mdmp_(m, ctx->x);       /* DMP X                */
    m      = agc_tp_shift(ip, m, 3 - n3); /* SR* 0 -3,1           */
    m      = agc_tp_add(ip, m, c1_xsq_c);      /* TAD: 2^49            */
    m      = agc_conic_mshiftr_(ip, m, 4);     /* SL4R                 */
    m      = agc_conic_mdmpr_(ip, m, ctx->inv_root_mu); /* DMPR 1/ROOTMU        */
    ctx->t = agc_tp_to_dp(m);                 /* T 2^28               */

    return (AGC_STATUS_OK);
}

/* First half of NEWSTATE / KEPCONVG (p. 1281, 1287): terminal position
 * (R1 - XSQC(XI)) UR1 + (T - X^3 S(XI) / ROOTMU) VVEC, 2^29 / 2^27.       */
static agc_vec_t newstate_position(agc_interp_t *ip, const agc_conic_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    agc_tp_t m = agc_tp_from_dp(ctx->r1); /* DLOAD R1, SR4R       */

    m = agc_conic_mshiftr_(ip, m, -4);
    m = agc_conic_mdsu_(ip, m, ctx->xsqc_xi); /* DSU XSQC(XI)         */
    agc_vec_t fpart = agc_vec_scale(ip, ctx->ur1, agc_tp_to_dp(m));

    fpart = agc_vec_shl(ip, fpart, 1); /* VSL1: 0D 2^33        */
    m     = agc_tp_from_dp(ctx->x);            /* DLOAD X, DSQ, NORM X1 */
    m     = agc_conic_mdsq_(m);
    int n1 = 0;

    m = agc_conic_mnorm_(m, &n1);
    m = agc_conic_mdmpr_(ip, m, ctx->inv_root_mu); /* DMPR 1/ROOTMU        */
    m = agc_conic_mdmpr_(ip, m, ctx->x);           /* DMPR X               */
    m = agc_conic_mdmp_(m, ctx->s_xi);             /* DMP S(XI)            */
    m = agc_conic_mshiftr_(ip, m, 7 - n1);         /* SRR* 0 -7,1: 2^28    */
    m = agc_conic_mbdsu_(ip, m, ctx->t);           /* BDSU T, SL1          */
    m = agc_tp_shift(ip, m, 1);
    agc_vec_t r2 = agc_vec_scale(ip, ctx->vvec, agc_tp_to_dp(m));

    r2 = agc_vec_shl(ip, r2, 1); /* VSL1, VAD            */
    r2 = agc_vec_add(ip, r2, fpart);

    return (agc_vec_shl(ip, r2, 4)); /* VSL4                 */
}

/* Second half of NEWSTATE (LAMENTER) / KEPCONVG: terminal velocity from the
 * terminal radius r2n = |R2| normalized (2^(29-n) / 2^(27-n)). KEPCONVG
 * uses DMPR S(XI) where LAMENTER uses DMP S(XI) (keeps MPAC+2 through the
 * DSU); round_xis selects the KEPCONVG form.                              */
static agc_vec_t lamenter_velocity(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                                   agc_dp_t r2n, int n, bool round_xis)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT(n >= 0 && n <= AGC_TP_BITS))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    agc_tp_t m = agc_tp_from_dp(ctx->xi);

    m = round_xis ? agc_conic_mdmpr_(ip, m, ctx->s_xi) :
        agc_conic_mdmp_(m, ctx->s_xi);                      /* XI S(XI) 2^7       */
    m = agc_conic_mdsu_(ip, m, AGC_CONIC_D1_128); /* DSU D1/128          */
    m = agc_conic_mdmp_(m, ctx->root_mu);         /* DMP ROOTMU, SL1R    */
    m = agc_conic_mshiftr_(ip, m, 1);
    m = agc_conic_mdmp_(m, ctx->x); /* DMP X, SLR* 0 -3,n  */
    m = agc_conic_mshiftr_(ip, m, n - 3);
    m = agc_conic_mddv_(ip, m, r2n); /* DDV R2: 2^15        */
    agc_vec_t fdot = agc_vec_scale(ip, ctx->ur1, agc_tp_to_dp(m));

    fdot = agc_vec_shl(ip, fdot, 1);       /* VSL1: 2^15          */
    m    = agc_tp_from_dp(ctx->xsqc_xi); /* XSQC(XI), SLR* 0 -4,n */
    m    = agc_conic_mshiftr_(ip, m, n - 4);
    m    = agc_conic_mddv_(ip, m, r2n);            /* DDV R2              */
    m    = agc_conic_mbdsu_(ip, m, AGC_CONIC_D1_256); /* BDSU D1/256: 2^8    */
    agc_vec_t v2 = agc_vec_scale(ip, ctx->vvec, agc_tp_to_dp(m));

    v2 = agc_vec_add(ip, v2, fdot); /* VAD, VSL8           */

    return (agc_vec_shl(ip, v2, 8));
}

/*
 * LAMENTER (CONIC_SUBROUTINES.agc p. 1287, label LAMENTER)
 * Entry with MPAC = |R2VEC| (r2mag, 2^29 / 2^27). Stores R2 (= MAGVEC2)
 * normalized, as the AGC does, and returns V2VEC 2^7 / 2^5 in *v2vec.
 */
agc_status_t agc_conic_lamenter_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                 agc_dp_t r2mag, agc_vec_t *v2vec)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(v2vec != NULL && r2mag >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int      n = 0;
    agc_tp_t m = agc_conic_mnorm_(agc_tp_from_dp(r2mag), &n); /* NORM X1 */

    ctx->magvec2 = agc_tp_to_dp(m);                          /* STODL R2 */
    *v2vec       = lamenter_velocity(ip, ctx, ctx->magvec2, n, false);

    return (AGC_STATUS_OK);
}

/*
 * NEWSTATE (CONIC_SUBROUTINES.agc p. 1287, label NEWSTATE)
 * Terminal state from the DELTIME results: *r2vec (0D) 2^29 / 2^27 m,
 * *v2vec (MPAC) 2^7 / 2^5 m/cs. Falls into LAMENTER.
 */
agc_status_t agc_conic_newstate_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                 agc_vec_t *r2vec, agc_vec_t *v2vec)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(r2vec != NULL && v2vec != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *r2vec = newstate_position(ip, ctx); /* VSL4 PUSH, ABVAL     */

    return (agc_conic_lamenter_(ip, ctx, agc_vec_abval(ip, *r2vec), v2vec));
}

/*
 * KEPCONVG (CONIC_SUBROUTINES.agc pp. 1281-1282, label KEPCONVG)
 * Same algebra as NEWSTATE with URRECT/VRECT (held in ctx->ur1/vvec), the
 * radius normalized into RCNORM with X2, and DMPR S(XI).
 */
agc_status_t agc_conic_kepconvg_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                 agc_vec_t *rcv, agc_vec_t *vcv)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(rcv != NULL && vcv != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *rcv = newstate_position(ip, ctx); /* STORE RCV            */
    int      n2 = 0;                          /* ABVAL, NORM X2       */
    agc_tp_t m  =
        agc_conic_mnorm_(agc_tp_from_dp(agc_vec_abval(ip, *rcv)), &n2);
    const agc_dp_t rcnorm = agc_tp_to_dp(m); /* STODL RCNORM         */

    *vcv = lamenter_velocity(ip, ctx, rcnorm, n2, true);

    return (AGC_STATUS_OK);
}
