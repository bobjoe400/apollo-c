/**********************************************************************
 * FILE NAME: universal.c
 *
 * PURPOSE:
 *   The universal-variable time equation and the f and g state update
 *   shared by KEPLER, LAMBERT, TIME-THETA and TIME-RADIUS: DELTIME (with
 *   its S and C series), NEWSTATE, LAMENTER and KEPCONVG, the final step
 *   of KEPLER.
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1281-1287 (DELTIME, NEWSTATE,
 *   LAMENTER, KEPCONVG)
 *   Luminary099 CONIC_SUBROUTINES.agc pp. 1177-1183 (identical)
 *
 * EXTERNAL VARIABLES:
 *   none (the conic erasables are fields of agc_conic_ctx_t)
 *
 * EXTERNAL REFERENCES:
 *   Name                 Description
 *   -------------------  -------------------------------------------
 *   agc_conic_poly_      POLY with the DELTIME coefficients (common.c)
 *   agc_conic_m*_        one interpretive op on the TP MPAC (common.c)
 *   agc_vec_*            VXSC, VSL, VAD, ABVAL (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; DELTIME leaves OVFIND set when T overflows, for the callers'
 *   BOV. AGC_STATUS_BAD_ARG only for a NULL pointer or a NORM count out
 *   of range.
 *
 * ALGORITHM:
 *   Notation (GSOP 5.5): x universal anomaly, xi = alpha x^2, S(xi) and
 *   C(xi) the Stumpff series, r1/v1 the initial state, mu the central
 *   body gravity.
 *     sqrt(mu) t = KEPC1 x^2 C + x (KEPC2 x^2 S + r1)        (DELTIME)
 *     r2 = (r1 - x^2 C) ur1 + (t - x^3 S / sqrt mu) v1       (NEWSTATE)
 *     v2 = sqrt(mu) x (xi S - 1) / r2 ur1 + (1 - x^2 C / r2) v1
 *                                                            (LAMENTER)
 *********************************************************************/
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

/**********************************************************************
 * DELTIME (CONIC_SUBROUTINES.agc pp. 1283-1284, label DELTIME) -- time of
 * flight for x: the S and C series (POLY) and the Kepler equation
 *
 * FUNCTION NAME: agc_conic_deltime_
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ----------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state; OVFIND left set
 *                                     if T overflowed (callers' BOV)
 *   ctx       agc_conic_ctx_t *  I/O  in: XI (MPAC) 2^6, XSQ (0D) x^2
 *                                     normalized 2^(34-N1) / 2^(32-N1)
 *                                     with xsq_n1 = N1, X, KEPC1, KEPC2,
 *                                     R1, 1/ROOTMU; out: T 2^28 cs,
 *                                     S(XI) 2^1, XSQC(XI) 2^33 / 2^31
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL pointer or a NORM count out of range
 *********************************************************************/
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

    const int n1 = ctx->xsq_n1; /* N1, NORM count of XSQ */

    /* TC POLY ... STODL S(XI) / XI ... TC POLY: C(XI) 2^4 */
    agc_tp_words_t m = agc_conic_poly_(agc_mpac_dload(ctx->xi), deltime_s_coef, AGC_CONIC_DELTIME_NCOEF); /* S(XI) 2^1 */

    ctx->s_xi = agc_mpac_store(m);                                                         /* STODL S(XI) XI */
    m         = agc_conic_poly_(agc_mpac_dload(ctx->xi), deltime_c_coef, AGC_CONIC_DELTIME_NCOEF); /* C(XI) 2^4 */

    m                       = agc_mpac_dmp(m, ctx->xsq);      /* DMP 0D               */
    m                       = agc_mpac_shift_round(ip, m, 5 - n1); /* SRR* 0 -5,1          */
    ctx->xsqc_xi            = agc_mpac_store(m);                   /* XSQC(XI) 2^33        */
    m                       = agc_mpac_dmp(m, ctx->kepc1);    /* DMP KEPC1, SL1       */
    m                       = agc_mpac_shift(ip, m, 1);
    const agc_tp_words_t c1_xsq_c = m;                                 /* 0D TP, 2^49          */

    m                       = agc_mpac_dload(ctx->xsq);          /* XCH with PL          */
    m                       = agc_mpac_dmp(m, ctx->s_xi);     /* DMP S(XI)            */
    m                       = agc_mpac_shift_round(ip, m, 5 - n1); /* SRR* 0 -5,1          */
    m                       = agc_mpac_dmp(m, ctx->kepc2);    /* DMP KEPC2, SL1       */
    m                       = agc_mpac_shift(ip, m, 1);
    const agc_tp_words_t c2_xsq_s = m;                                 /* 3D TP, 2^35          */

    m      = agc_mpac_dload(ctx->r1);     /* DLOAD R1, SR 6       */
    m      = agc_mpac_shift(ip, m, -6);
    m      = agc_mpac_tad(ip, m, c2_xsq_s); /* TAD                  */
    int n3 = 0;                           /* NORM X1 count */

    m      = agc_mpac_norm(m, &n3);                  /* NORM X1              */
    m      = agc_mpac_dmp(m, ctx->x);                /* DMP X                */
    m      = agc_mpac_shift(ip, m, 3 - n3);               /* SR* 0 -3,1           */
    m      = agc_mpac_tad(ip, m, c1_xsq_c);               /* TAD: 2^49            */
    m      = agc_mpac_shift_round(ip, m, 4);              /* SL4R                 */
    m      = agc_mpac_dmpr(ip, m, ctx->inv_root_mu); /* DMPR 1/ROOTMU */
    ctx->t = agc_mpac_store(m);                           /* T 2^28               */

    return (AGC_STATUS_OK);
} /* agc_conic_deltime_ */

/**********************************************************************
 * NEWSTATE / KEPCONVG first half (CONIC_SUBROUTINES.agc pp. 1281, 1287)
 * -- terminal position (R1 - XSQC(XI)) UR1 + (T - X^3 S(XI)/ROOTMU) VVEC
 *
 * FUNCTION NAME: newstate_position
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ----------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       const agc_conic_ctx_t *  I    R1, XSQC(XI), X, S(XI), T,
 *                                           UR1, VVEC, 1/ROOTMU
 *
 * RETURNS: agc_vec_t -- terminal position, 2^29 / 2^27 m
 *********************************************************************/
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

    agc_tp_words_t m = agc_mpac_dload(ctx->r1); /* DLOAD R1, SR4R: 2^33 / 2^31 */

    m               = agc_mpac_shift_round(ip, m, -4);
    m               = agc_mpac_dsu(ip, m, ctx->xsqc_xi);         /* DSU XSQC(XI)         */
    agc_vec_t fpart = agc_conic_vxsc_(ip, m, ctx->ur1);          /* VXSC UR1: (R1-XSQC) UR1, 2^34 */

    fpart  = agc_vec_shl(ip, fpart, 1); /* VSL1: 0D 2^33        */
    m      = agc_mpac_dload(ctx->x);    /* DLOAD X, DSQ, NORM X1 */
    m      = agc_mpac_dsq(m);
    int n1 = 0;                         /* NORM X1 count */

    m            = agc_mpac_norm(m, &n1);
    m            = agc_mpac_dmpr(ip, m, ctx->inv_root_mu);     /* DMPR 1/ROOTMU */
    m            = agc_mpac_dmpr(ip, m, ctx->x);               /* DMPR X               */
    m            = agc_mpac_dmp(m, ctx->s_xi);                 /* DMP S(XI)            */
    m            = agc_mpac_shift_round(ip, m, 7 - n1);             /* SRR* 0 -7,1: 2^28    */
    m            = agc_mpac_bdsu(ip, m, ctx->t);               /* BDSU T, SL1          */
    m            = agc_mpac_shift(ip, m, 1);
    agc_vec_t r2 = agc_conic_vxsc_(ip, m, ctx->vvec);           /* VXSC VRECT, 2^34 */

    r2 = agc_vec_shl(ip, r2, 1); /* VSL1, VAD            */
    r2 = agc_vec_add(ip, r2, fpart);

    return (agc_vec_shl(ip, r2, 4)); /* VSL4                 */
} /* newstate_position */

/**********************************************************************
 * LAMENTER / KEPCONVG second half (CONIC_SUBROUTINES.agc pp. 1282, 1287)
 * -- terminal velocity from the normalized terminal radius. KEPCONVG uses
 * DMPR S(XI) where LAMENTER uses DMP S(XI) (keeping MPAC+2 through the
 * DSU); round_xis selects the KEPCONVG form.
 *
 * FUNCTION NAME: lamenter_velocity
 *
 * ARGUMENTS:
 *   Argument   Type                     I/O  Description (scale)
 *   ---------  -----------------------  ---  ---------------------------
 *   ip         agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx        const agc_conic_ctx_t *  I    XI, S(XI), X, XSQC(XI),
 *                                            ROOTMU, UR1, VVEC
 *   r2n        agc_dp_words_t           I    |R2| normalized,
 *                                            2^(29-n) / 2^(27-n) m
 *   n          int                      I    NORM count of |R2|
 *   round_xis  bool                     I    true: DMPR S(XI) (KEPCONVG)
 *
 * RETURNS: agc_vec_t -- terminal velocity, 2^7 / 2^5 m/cs
 *********************************************************************/
static agc_vec_t lamenter_velocity(agc_interp_t *ip, const agc_conic_ctx_t *ctx, agc_dp_words_t r2n, int n, bool round_xis)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT(n >= 0 && n <= AGC_TP_BITS))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    agc_tp_words_t m = agc_mpac_dload(ctx->xi); /* XI 2^6, then MPAC */

    if (round_xis)
    { /* KEPCONVG       */
        m = agc_mpac_dmpr(ip, m, ctx->s_xi); /* DMPR S(XI) 2^7 */
    }
    else
    { /* LAMENTER       */
        m = agc_mpac_dmp(m, ctx->s_xi); /* DMP S(XI) 2^7  */
    }

    m              = agc_mpac_dsu(ip, m, AGC_CONIC_D1_128);     /* DSU D1/128          */
    m              = agc_mpac_dmp(m, ctx->root_mu);             /* DMP ROOTMU, SL1R    */
    m              = agc_mpac_shift_round(ip, m, 1);
    m              = agc_mpac_dmp(m, ctx->x);                   /* DMP X, SLR* 0 -3,n  */
    m              = agc_mpac_shift_round(ip, m, n - 3);
    m              = agc_mpac_ddv(ip, m, r2n);                  /* DDV R2: 2^15        */
    agc_vec_t fdot = agc_conic_vxsc_(ip, m, ctx->ur1);          /* VXSC URRECT, 2^16 */

    fdot         = agc_vec_shl(ip, fdot, 1);                      /* VSL1: 2^15          */
    m            = agc_mpac_dload(ctx->xsqc_xi);                  /* XSQC(XI), SLR* 0 -4,n */
    m            = agc_mpac_shift_round(ip, m, n - 4);
    m            = agc_mpac_ddv(ip, m, r2n);                   /* DDV R2              */
    m            = agc_mpac_bdsu(ip, m, AGC_CONIC_D1_256);     /* BDSU D1/256: 2^8    */
    agc_vec_t v2 = agc_conic_vxsc_(ip, m, ctx->vvec);            /* VXSC VRECT, 2^15 */

    v2 = agc_vec_add(ip, v2, fdot); /* VAD, VSL8           */

    return (agc_vec_shl(ip, v2, 8));
} /* lamenter_velocity */

/**********************************************************************
 * LAMENTER (CONIC_SUBROUTINES.agc p. 1287, label LAMENTER) -- terminal
 * velocity for LAMBERT's TARGETV, entered with MPAC = |R2VEC|
 *
 * FUNCTION NAME: agc_conic_lamenter_
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ----------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *  I/O  DELTIME results; R2 (= MAGVEC2) is
 *                                     left normalized, as in the AGC
 *   r2mag     agc_tp_words_t     I    MPAC = |R2VEC| (ABVAL's TP words, or
 *                                     DLOAD MAGVEC2), 2^29 / 2^27 m
 *   v2vec     agc_vec_t *        O    V2VEC, 2^7 / 2^5 m/cs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL pointer or a negative r2mag
 *********************************************************************/
agc_status_t agc_conic_lamenter_(agc_interp_t *ip, agc_conic_ctx_t *ctx, agc_tp_words_t r2mag, agc_vec_t *v2vec)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(v2vec != NULL && !agc_conic_mbmn_(r2mag)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int            n = 0;                         /* NORM X1 count */
    agc_tp_words_t m = agc_mpac_norm(r2mag, &n); /* LAMENTER NORM X1: R2 2^(29-n) */

    ctx->magvec2 = agc_mpac_store(m); /* STODL R2 */
    *v2vec       = lamenter_velocity(ip, ctx, ctx->magvec2, n, false);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * NEWSTATE (CONIC_SUBROUTINES.agc p. 1287, label NEWSTATE) -- terminal
 * state from the DELTIME results; falls into LAMENTER
 *
 * FUNCTION NAME: agc_conic_newstate_
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ----------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *  I/O  DELTIME results
 *   r2vec     agc_vec_t *        O    R2VEC (0D), 2^29 / 2^27 m
 *   v2vec     agc_vec_t *        O    V2VEC (MPAC), 2^7 / 2^5 m/cs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL pointer
 *********************************************************************/
agc_status_t agc_conic_newstate_(agc_interp_t *ip, agc_conic_ctx_t *ctx, agc_vec_t *r2vec, agc_vec_t *v2vec)
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

    agc_tp_words_t m = agc_mpac_sqrt(ip, agc_mpac_vsq(ip, *r2vec)); /* PUSH ABVAL */

    return (agc_conic_lamenter_(ip, ctx, m, v2vec));
}

/**********************************************************************
 * KEPCONVG (CONIC_SUBROUTINES.agc pp. 1281-1282, label KEPCONVG) -- the
 * NEWSTATE algebra for KEPLER with URRECT/VRECT (held in ctx->ur1/vvec),
 * the radius normalized into RCNORM with X2, and DMPR S(XI)
 *
 * FUNCTION NAME: agc_conic_kepconvg_
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ----------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *  I    KEPLER's last DELTIME results
 *   rcv       agc_vec_t *        O    RCV, 2^29 / 2^27 m
 *   vcv       agc_vec_t *        O    VCV, 2^7 / 2^5 m/cs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL pointer
 *********************************************************************/
agc_status_t agc_conic_kepconvg_(agc_interp_t *ip, agc_conic_ctx_t *ctx, agc_vec_t *rcv, agc_vec_t *vcv)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(rcv != NULL && vcv != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *rcv                  = newstate_position(ip, ctx);                                     /* STORE RCV            */
    int            n2     = 0;                                                              /* ABVAL, NORM X2 count */
    agc_tp_words_t       m      = agc_mpac_norm(agc_mpac_sqrt(ip, agc_mpac_vsq(ip, *rcv)), &n2); /* ABVAL NORM X2: |RCV| 2^(29-n2) */
    const agc_dp_words_t rcnorm = agc_mpac_store(m);                                                /* STODL RCNORM, 2^(29-n2) */

    *vcv = lamenter_velocity(ip, ctx, rcnorm, n2, true);

    return (AGC_STATUS_OK);
}
