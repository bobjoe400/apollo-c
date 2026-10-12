/**********************************************************************
 * FILE NAME: geom.c
 *
 * PURPOSE:
 *   The geometry and x-guess subroutines of lower rank: GEOM, PARAM and
 *   GETX (WLOOP, POLYCOEF, XCOMMON, RESETX2, 360CHECK, INVRSEQN, 1/WLOOP,
 *   TRUE360X, INFINITY, OVFLCLR), and the public PARAM entry.
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1289-1295 (PARAM, GEOM,
 *   UNITNORM, COLINEAR, HAVENORM, GETX and its labels)
 *   Luminary099 CONIC_SUBROUTINES.agc pp. 1185-1191 (identical)
 *
 * EXTERNAL VARIABLES:
 *   Name      Type  I/O  Scale  Description
 *   --------  ----  ---  -----  ------------------------------------
 *   INFINFLG  flag  O    -      set by INFINITY, cleared by XCOMMON
 *   COGAFLAG  flag  O    -      cleared by PARAM
 *
 * EXTERNAL REFERENCES:
 *   Name               Description
 *   -----------------  ---------------------------------------------
 *   agc_vec_unit       UNIT (interp.h)
 *   agc_conic_poly_    POLY with the POLYCOEF coefficients (common.c)
 *   agc_mpac_*         one interpretive op on the MPAC words (interp_words.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; GETX reports a conic that would have to close through infinity
 *   by INFINFLG (callers then ABTCONIC or retreat). The public PARAM
 *   returns AGC_STATUS_BAD_ARG for a NULL pointer or an unknown body.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   WLOOP and 1/WLOOP run exactly three passes (TIX,2 with X2 = 3,
 *   S2 = 1).
 *
 * ALGORITHM:
 *   GETX finds the universal variable x of the conic through a given
 *   true-anomaly difference (CSTH, SNTH) from P, R1A, COGA by the
 *   continued-fraction form of GSOP 5.5: three half-angle steps
 *   W(k+1) = W(k) + sqrt(W(k)^2 + R1A) (WLOOP), or for transfers near 360
 *   degrees the inverse form (INVRSEQN, 1/WLOOP), then a 7-term power
 *   series (POLYCOEF), then x, xi, KEPC1 and KEPC2 for DELTIME (XCOMMON).
 *********************************************************************/
#include "agc/core/assertion.h"
#include "internal.h"

/* POLYCOEF series (p. 1293), argument R1A/W^2 at 2^0, assembled 2DEC. */
static const agc_dp_t polycoef_coef[AGC_CONIC_POLYCOEF_NCOEF] = {
    134217728, /*  .5          20000 00000 */
    -44739270, /* -.166666770  72525 52471 */
    26843651,  /*  .100000392  03146 15003 */
    -19166583, /* -.071401086  75556 45210 */
    14899051,  /*  .055503292  01615 13553 */
    -12687360, /* -.047264098  76371 63777 */
    10923767   /*  .040694204  01232 27367 */
};

/* Where GETX goes next after one of its labelled blocks. */
typedef enum getx_next
{
    GETX_TO_WLOOP = 0, /* MPAC = W0 (2^5)                             */
    GETX_TO_POLYCOEF,  /* MPAC = 1/W (2^2) or 16/W (2^6)              */
    GETX_TO_360CHECK,  /* BOV taken; MPAC holds the limited value     */
    GETX_TO_INFINITY   /* closure through infinity                    */
} getx_next_t;

/**********************************************************************
 * GEOM (CONIC_SUBROUTINES.agc p. 1291, labels GEOM, UNITNORM, COLINEAR,
 * HAVENORM) -- unit vectors, radii and the angle between two vectors
 *
 * FUNCTION NAME: agc_conic_geom_
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *  I/O  NORMSW, GEOMSGN, UN (if NORMSW) in;
 *                                     U2, MAGVEC2, UR1, R1, UN out
 *   r1vec     agc_vec_t          I    0D vector, 2^29 / 2^27 m
 *   v2vec     agc_vec_t          I    MPAC vector (R2VEC or VVEC)
 *   snth      agc_dp_words_t *   O    MPAC SNTH, 2^1
 *   csth      agc_dp_words_t *   O    0D CSTH, 2^1
 *********************************************************************/
void agc_conic_geom_(agc_interp_t *ip, agc_conic_ctx_t *ctx, agc_vec_t r1vec, agc_vec_t v2vec, agc_dp_words_t *snth, agc_dp_words_t *csth)
{
    agc_dp_t mag = 0; /* 36D |vector|, its scale */

    ctx->u2          = agc_vec_unit(ip, v2vec, &mag, NULL); /* UNIT: U2, 36D */
    ctx->magvec2     = agc_dp_to_words(mag);                /* STOVL MAGVEC2 */
    ctx->ur1         = agc_vec_unit(ip, r1vec, &mag, NULL); /* UNIT: UR1     */
    agc_tp_words_t m = agc_mpac_dot(ip, ctx->ur1, ctx->u2); /* DOT: TP, 2^2 */

    *csth       = agc_mpac_store(agc_mpac_shift(ip, m, 1)); /* DOT SL1: 0D */
    ctx->r1     = agc_dp_to_words(mag);                     /* STOVL R1     */
    agc_vec_t n = agc_vec_cross(ip, ctx->ur1, ctx->u2);     /* VXV, 2^2 */

    n = agc_vec_shl(ip, n, 1); /* VSL1, 2^1 */

    if (ctx->normsw)
    { /* HAVENORM: ABVAL SIGN */
        m     = agc_mpac_sqrt(agc_mpac_vsq(ip, n));
        *snth = agc_mpac_store(agc_conic_msign_sp_(m, ctx->geomsgn));
    }
    else
    {
        /* SIGN with a vector MPAC: complement it if the SP tag is negative. */
        agc_sp_t s = ctx->geomsgn; /* GEOMSGN, SP 2^0 */

        n            = (s < 0) ? agc_vec_neg(n) : n;   /* SIGN GEOMSGN, UNIT   */
        agc_dp_t  sn = 0;                              /* |N| = SNTH magnitude, 2^1 */
        agc_vec_t un = agc_vec_unit(ip, n, &sn, NULL); /* UNIT: UN, 2^1 */

        if (agc_interp_test_ovf(ip))
        { /* BOV COLINEAR: VSR1   */
            un = agc_vec_shr(un, 1);
        }

        ctx->un = un;                                                   /* UNITNORM: STODL UN 36D */
        m       = agc_mpac_dload(agc_dp_to_words(sn));
        *snth   = agc_mpac_store(agc_conic_msign_sp_(m, ctx->geomsgn)); /* SIGN GEOMSGN */
    }
} /* agc_conic_geom_ */

/**********************************************************************
 * PARAM (CONIC_SUBROUTINES.agc pp. 1289-1290, label PARAM) -- conic
 * parameters of a state: COGA, R1A = R1 V^2/MU, P
 *
 * FUNCTION NAME: agc_conic_param_
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state; OVFIND left set
 *                                     when COGA overflowed (callers BOV
 *                                     to COGAOVFL)
 *   ctx       agc_conic_ctx_t *  I/O  COGA 2^5, R1A 2^6, P 2^4, R1, UR1,
 *                                     U2, UN, MAGVEC2 out; NORMSW and
 *                                     COGAFLAG cleared
 *   rvec      agc_vec_t          I    0D RVEC, 2^29 / 2^27 m
 *   vvec      agc_vec_t          I    MPAC VVEC, 2^7 / 2^5 m/cs
 *   sngamma   agc_dp_words_t *   O    36D sin(gamma), 2^1
 *   mpac      agc_tp_words_t *   O    MPAC at RTNPRM: P 2^4 with the
 *                                     MPAC+2 of the SR* (GETX's input)
 *********************************************************************/
void agc_conic_param_(agc_interp_t *ip, agc_conic_ctx_t *ctx, agc_vec_t rvec, agc_vec_t vvec, agc_dp_words_t *sngamma, agc_tp_words_t *mpac)
{
    ctx->normsw        = false;                             /* CLEAR NORMSW         */
    ctx->cogaflag      = false;                             /* CLEAR COGAFLAG       */
    agc_conic_mirror_flag_(AGC_CONIC_FLAG_COGAFLAG, false); /* mirror only */
    ctx->geomsgn       = AGC_CONIC_GEOMSGN_POS;             /* SSP GEOMSGN: < 180 deg */
    agc_dp_words_t sng = {0, 0};                            /* MPAC SNGA, 2^1 */
    agc_dp_words_t csg = {0, 0};                            /* 0D CSGA, 2^1   */
    agc_conic_geom_(ip, ctx, rvec, vvec, &sng, &csg);       /* CALL GEOM */

    *sngamma         = sng;                                         /* STODL 36D: 0D CSGA */
    agc_tp_words_t m = agc_mpac_shift(ip, agc_mpac_dload(csg), -5); /* SR 5, 2^6 */

    m         = agc_mpac_ddv(ip, m, sng); /* DDV 36D */
    ctx->coga = agc_mpac_store(m);        /* STOVL* COGA */
    int n     = 0;                        /* NORM X1 count */

    m = agc_mpac_norm(agc_mpac_dsq(agc_mpac_dload(ctx->magvec2)), &n); /* MAGVEC2 DSQ NORM X1 */
    m = agc_mpac_dmpr(ip, m, ctx->inv_mu);                             /* DMPR 1/MU */
    m = agc_mpac_dmp(m, ctx->r1);                                      /* DMP R1    */
    m = agc_mpac_shift_round(ip, m, 3 - n);                            /* SRR* 0 -3,1 */

    const agc_dp_words_t r1v2mu = agc_mpac_store(m); /* PUSH: 0D R1 V^2/MU, 2^6 */

    ctx->r1a = agc_mpac_store(agc_mpac_bdsu(ip, m, AGC_CONIC_D1_32)); /* BDSU D1/32 STODL R1A */
    m        = agc_mpac_dmp(agc_mpac_dload(r1v2mu), sng);             /* DMP 36D */
    m        = agc_mpac_norm(m, &n);                                  /* NORM X1 */
    m        = agc_mpac_dmp(m, sng);                                  /* DMP 36D */
    m        = agc_mpac_shift(ip, m, 4 - n);                          /* SR* 0 -4,1 */
    ctx->p   = agc_mpac_store(m);                                     /* STCALL P: P 2^4 */
    *mpac    = m;
} /* agc_conic_param_ */

/**********************************************************************
 * INFINITY, OVFLCLR (CONIC_SUBROUTINES.agc p. 1295) -- no solution,
 * closure through infinity: SETPD 0, BOV OVFLCLR, SET INFINFLG, RVQ
 *
 * FUNCTION NAME: getx_infinity
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ip        agc_interp_t *     I/O  OVFIND cleared
 *   ctx       agc_conic_ctx_t *  O    INFINFLG set
 *********************************************************************/
static void getx_infinity(agc_interp_t *ip, agc_conic_ctx_t *ctx)
{
    agc_interp_clear_ovf(ip); /* BOV: clear OVFIND    */
    ctx->infinflg = true;
    agc_conic_mirror_flag_(AGC_CONIC_FLAG_INFINFLG, true);
}

/**********************************************************************
 * GETX entry to WLOOP (CONIC_SUBROUTINES.agc p. 1292) -- W0 =
 * (SNTH/(1-CSTH) - COGA) sqrt(P)
 *
 * FUNCTION NAME: getx_first_w
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       const agc_conic_ctx_t *  I    P, CSTH, SNTH, COGA
 *   mpac      agc_tp_words_t *         I/O  in: P 2^4 as the caller left
 *                                           MPAC; out: W0 2^5 (or the
 *                                           overflowed value for 360CHECK)
 *
 * RETURNS: getx_next_t -- GETX_TO_WLOOP, or GETX_TO_360CHECK if either
 *          BOV is taken
 *********************************************************************/
static getx_next_t getx_first_w(agc_interp_t *ip, const agc_conic_ctx_t *ctx, agc_tp_words_t *mpac)
{
    agc_tp_words_t m = agc_mpac_sqrt(*mpac); /* SQRT of the TP MPAC, 2^2 */

    const agc_dp_words_t sqrtp = agc_mpac_store(m); /* PDDL: 0D SQRT P, 2^2 */

    m = agc_mpac_shift(ip, agc_mpac_dload(ctx->csth), -1); /* CSTH SR1 */

    const agc_dp_words_t one_m_csth = agc_mpac_store(agc_mpac_bdsu(ip, m, AGC_CONIC_D1_4)); /* BDSU D1/4 PDDL: 2D 2^2 */

    m = agc_mpac_shift_round(ip, agc_mpac_dload(ctx->snth), -6); /* SNTH SRR 6 */
    m = agc_mpac_ddv(ip, m, one_m_csth);                         /* DDV 2D: 2^5 */

    if (agc_interp_test_ovf(ip))
    {
        *mpac = m;

        return (GETX_TO_360CHECK);
    }

    m     = agc_mpac_dsu(ip, m, ctx->coga); /* DSU COGA */
    m     = agc_mpac_dmp(m, sqrtp);
    m     = agc_mpac_shift_round(ip, m, 2); /* SL2R    */
    *mpac = m;

    return (agc_interp_test_ovf(ip) ? GETX_TO_360CHECK : GETX_TO_WLOOP); /* BOV 360CHECK */
}

/**********************************************************************
 * WLOOP (CONIC_SUBROUTINES.agc p. 1292) -- three passes of
 * W = W + sqrt(W^2 + R1A), then BDDV D1/128 to 1/W
 *
 * FUNCTION NAME: getx_wloop
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       const agc_conic_ctx_t *  I    R1A 2^6
 *   mpac      agc_tp_words_t *               I/O  W 2^5 in; 1/W 2^2 out
 *
 * RETURNS: getx_next_t -- GETX_TO_POLYCOEF, GETX_TO_360CHECK (BOV in the
 *          loop, RESETX2) or GETX_TO_INFINITY
 *********************************************************************/
static getx_next_t getx_wloop(agc_interp_t *ip, const agc_conic_ctx_t *ctx, agc_tp_words_t *mpac)
{
    agc_tp_words_t m = *mpac; /* W, 2^5 */

    for (int pass = 0; pass < AGC_CONIC_GETX_PASSES; pass++)
    { /* TIX,2 */
        const agc_dp_words_t w   = agc_mpac_store(m); /* PUSH 0D: W, 2^5 */
        const agc_tp_words_t wsq = agc_mpac_dsq(m);   /* TLOAD MPAC: W^2, 2^10 */

        m = agc_mpac_shift(ip, agc_mpac_dload(ctx->r1a), -4); /* SR4  */
        m = agc_mpac_tad(ip, m, wsq);                         /* 2^10    */

        if (agc_conic_mbmn_(m))
        {
            return (GETX_TO_INFINITY);
        }

        m = agc_mpac_round(ip, agc_mpac_sqrt(m)); /* SQRT ROUND */
        m = agc_mpac_dad(ip, m, w);               /* DAD 0D  */

        if (agc_interp_test_ovf(ip))
        {
            *mpac = m;

            return (GETX_TO_360CHECK);
        }
    }

    m     = agc_mpac_bddv(ip, m, AGC_CONIC_D1_128); /* 1/W 2^2 */
    *mpac = m;

    return (agc_interp_test_ovf(ip) ? GETX_TO_INFINITY : GETX_TO_POLYCOEF); /* BOV INFINITY */
} /* getx_wloop */

/**********************************************************************
 * 1/WLOOP (CONIC_SUBROUTINES.agc p. 1294) -- G = G + sqrt(G^2 + R1A/W^2)
 * three times from G = 1, then MPAC = (1/W)/G for POLYCOEF
 *
 * FUNCTION NAME: getx_inv_wloop
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       const agc_conic_ctx_t *  I    P, SNTH, CSTH, COGA, R1A
 *   mpac      agc_tp_words_t *               O    (1/W)/G, 2^2
 *
 * RETURNS: getx_next_t -- GETX_TO_POLYCOEF, or GETX_TO_INFINITY when the
 *          root argument goes negative
 *********************************************************************/
static getx_next_t getx_inv_wloop(agc_interp_t *ip, const agc_conic_ctx_t *ctx, agc_tp_words_t *mpac)
{
    agc_dp_words_t inv_w_sq = AGC_CONIC_KEPZERO; /* 34D (1/W)^2, 2^-2 */

    /**********************************************************************
     * 360CHECK / INVRSEQN (CONIC_SUBROUTINES.agc p. 1294) -- near-360-degree
     * form: 1/W = SNTH / (sqrt(P) (1 + CSTH - SNTH COGA)) and its square
     *
     * FUNCTION NAME: getx_invrseqn
     *
     * ARGUMENTS:
     *   Argument  Type                     I/O  Description (scale)
     *   --------  -----------------------  ---  ---------------------------
     *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
     *   ctx       const agc_conic_ctx_t *  I    P, SNTH, CSTH, COGA
     *   inv_w_sq  agc_dp_words_t *         O    34D (1/W)^2, 2^-2
     *
     * RETURNS: agc_dp_words_t -- 0D 1/W, 2^-1
     *********************************************************************/
    agc_tp_words_t m_invrseqn = agc_mpac_sqrt(agc_mpac_dload(ctx->p)); /* DLOAD P SQRT, 2^2 */

    const agc_dp_words_t sqrtp = agc_mpac_store(m_invrseqn); /* PDDL: 0D SQRT P, 2^2 */

    m_invrseqn = agc_mpac_dmp(agc_mpac_dload(ctx->snth), ctx->coga); /* SNTH DMP COGA */

    const agc_dp_words_t sn_coga = agc_mpac_store(agc_mpac_shift(ip, m_invrseqn, 1)); /* SL1: 2D SNTH COGA, 2^5 */

    m_invrseqn     = agc_mpac_shift(ip, agc_mpac_dload(ctx->csth), -4); /* SR4     */
    m_invrseqn     = agc_mpac_dad(ip, m_invrseqn, AGC_CONIC_D1_32);     /* 1+CSTH  */
    m_invrseqn     = agc_mpac_dsu(ip, m_invrseqn, sn_coga);
    m_invrseqn     = agc_mpac_dmp(m_invrseqn, sqrtp);                   /* 2^7     */
    int n_invrseqn = 0;                                                 /* NORM X1 count */

    m_invrseqn                 = agc_mpac_norm(m_invrseqn, &n_invrseqn);               /* NORM X1   */
    m_invrseqn                 = agc_mpac_bddv(ip, m_invrseqn, ctx->snth);             /* BDDV SNTH */
    m_invrseqn                 = agc_mpac_shift_round(ip, m_invrseqn, n_invrseqn - 5); /* SLR* 0 -5,1 */
    m_invrseqn                 = agc_mpac_abs(m_invrseqn);                             /* ABS       */
    inv_w_sq                   = agc_mpac_store(agc_mpac_dsq(m_invrseqn));             /* 34D 2^-2  */
    const agc_dp_words_t inv_w = agc_mpac_store(m_invrseqn);                           /* 0D 1/W, 2^-1 */
    agc_tp_words_t       m     = agc_mpac_dload(AGC_CONIC_D1_16);                      /* STODL 34D D1/16: G = 1, 2^4 */

    for (int pass = 0; pass < AGC_CONIC_GETX_PASSES; pass++)
    { /* TIX,2 */
        const agc_dp_words_t g   = agc_mpac_store(m); /* PUSH 2D: G, 2^4 */
        const agc_tp_words_t gsq = agc_mpac_dsq(m);   /* TPMODE: G^2, 2^8 */

        m = agc_mpac_dmp(agc_mpac_dload(ctx->r1a), inv_w_sq); /* PDDL R1A DMP 34D */
        m = agc_mpac_shift(ip, m, -4);                        /* SR4     */
        m = agc_mpac_tad(ip, m, gsq);                         /* 2^8     */

        if (agc_conic_mbmn_(m))
        {
            return (GETX_TO_INFINITY);
        }

        m = agc_mpac_dad(ip, agc_mpac_sqrt(m), g); /* DAD 2D  */
    }

    int n = 0; /* NORM X1 count */

    m     = agc_mpac_norm(m, &n);               /* NORM X1 */
    m     = agc_mpac_bddv(ip, m, inv_w);        /* BDDV 0D */
    *mpac = agc_mpac_shift_round(ip, m, n - 7); /* SLR* 0 -7,1 */

    return (GETX_TO_POLYCOEF);
} /* getx_inv_wloop */

/**********************************************************************
 * POLYCOEF and TRUE360X (CONIC_SUBROUTINES.agc pp. 1292-1295) -- the
 * series value x' from MPAC = 1/W, corrected to 2 pi/sqrt(R1A) - x' when
 * 360SW is set
 *
 * FUNCTION NAME: getx_polycoef
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       const agc_conic_ctx_t *  I    R1A, 360SW
 *   m_in      agc_tp_words_t                 I    MPAC 1/W, 2^2 (or 16/W 2^6)
 *   xprime    agc_tp_words_t *               O    0D x', 2^6 equivalent
 *
 * RETURNS: bool -- false for INFINITY (1/W or R1A negative)
 *********************************************************************/
static bool getx_polycoef(agc_interp_t *ip, const agc_conic_ctx_t *ctx, agc_tp_words_t m_in, agc_tp_words_t *xprime)
{
    if (agc_conic_mbmn_(m_in))
    {
        return (false);
    } /* BMN     */

    const agc_dp_words_t pw = agc_mpac_store(m_in); /* PUSH 0D: 1/W, 2^2 */
    int                  n  = 0;                    /* NORM X1 count */
    agc_tp_words_t       m  = agc_mpac_dsq(m_in);   /* DSQ, 2^4 */

    m = agc_mpac_norm(m, &n); /* NORM X1: (1/W)^2, 2^(4-n) */

    m = agc_mpac_dmp(m, ctx->r1a);
    m = agc_mpac_shift_round(ip, m, 10 - n);                         /* SRR* 0 -10D,1 */
    m = agc_conic_poly_(m, polycoef_coef, AGC_CONIC_POLYCOEF_NCOEF); /* EXIT, TC POLY, DEC 5 */
    m = agc_mpac_shift_round(ip, agc_mpac_dmp(m, pw), 1);            /* DMP SL1R */

    if (ctx->sw360)
    { /* TRUE360X */
        const agc_dp_words_t xp = agc_mpac_store(m);        /* 0D x', 2^6 equivalent */
        agc_tp_words_t       r  = agc_mpac_dload(ctx->r1a); /* DLOAD R1A, 2^6 */

        if (agc_conic_mbmn_(r))
        {
            return (false);
        }

        r = agc_mpac_norm(agc_mpac_sqrt(r), &n);
        r = agc_mpac_bddv(ip, r, AGC_CONIC_2PISC); /* BDDV 2PISC */
        r = agc_mpac_shift(ip, r, n - 3);          /* SL* 0 -3,1 */
        m = agc_mpac_dsu(ip, r, xp);               /* DSU PUSH */
    }

    *xprime = m;

    return (true);
} /* getx_polycoef */

/**********************************************************************
 * GETX (CONIC_SUBROUTINES.agc pp. 1292-1295, label GETX) -- universal
 * variable x of the conic through (CSTH, SNTH)
 *
 * FUNCTION NAME: agc_conic_getx_
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *  I/O  P, CSTH, SNTH 2^1, COGA 2^5, R1A
 *                                     2^6, R1 in; X, XI, XSQ, KEPC1,
 *                                     KEPC2, 360SW, INFINFLG out
 *                                     (INFINFLG set: nothing else valid)
 *   mpac      agc_tp_words_t     I    MPAC = P 2^4 as the caller left it
 *                                     ("ASSUMES P (+4) IN MPAC"; SQRT
 *                                     reads MPAC+2)
 *********************************************************************/
void agc_conic_getx_(agc_interp_t *ip, agc_conic_ctx_t *ctx, agc_tp_words_t mpac)
{
    ctx->sw360          = false;                     /* AXT,2 3; SSP S2 1; CLEAR 360SW */
    agc_tp_words_t m    = mpac;                      /* MPAC: P, then W 2^5 or 1/W 2^2 */
    getx_next_t    next = getx_first_w(ip, ctx, &m); /* next GETX block */

    if (next == GETX_TO_WLOOP)
    {
        next = getx_wloop(ip, ctx, &m);
    }

    if (next == GETX_TO_360CHECK)
    { /* RESETX2 / 360CHECK   */
        ctx->sw360 = agc_conic_mbmn_(m); /* BPL INVRSEQN / SET 360SW */
        next       = getx_inv_wloop(ip, ctx, &m);
    }

    agc_tp_words_t xprime = {0, 0, 0}; /* MPAC x' (pushed to 0D), 2^6 equivalent */

    if (next != GETX_TO_POLYCOEF || !getx_polycoef(ip, ctx, m, &xprime))
    {
        getx_infinity(ip, ctx);

        return;
    }

    /**********************************************************************
     * XCOMMON (CONIC_SUBROUTINES.agc pp. 1293-1294) -- XI, X, XSQ, KEPC1,
     * KEPC2 from x', and CLEAR INFINFLG
     *
     * FUNCTION NAME: getx_xcommon
     *
     * ARGUMENTS:
     *   Argument  Type               I/O  Description (scale)
     *   --------  -----------------  ---  ---------------------------------
     *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
     *   ctx       agc_conic_ctx_t *  I/O  XI 2^6, X 2^17 / 2^16, XSQ
     *                                     2^(34-N1), KEPC1 2^17, KEPC2 2^6
     *   xprime    agc_tp_words_t           I    0D x' from POLYCOEF
     *********************************************************************/
    const agc_dp_words_t xp        = agc_mpac_store(xprime); /* 0D x' as pushed */
    int                  n         = 0;                      /* NORM X1 count */
    agc_tp_words_t       m_xcommon = agc_mpac_dsq(xprime);   /* DSQ: x'^2, 2^12 */

    m_xcommon = agc_mpac_norm(m_xcommon, &n); /* NORM X1 */

    m_xcommon     = agc_mpac_dmp(m_xcommon, ctx->r1a);                           /* DMP R1A */
    ctx->xi       = agc_mpac_store(agc_mpac_shift_round(ip, m_xcommon, 12 - n)); /* SRR* 0 -12D,1: XI 2^6 */
    m_xcommon     = agc_mpac_shift(ip, agc_mpac_dload(ctx->r1), -1);             /* R1 SR1  */
    m_xcommon     = agc_mpac_round(ip, agc_mpac_sqrt(m_xcommon));                /* SQRT ROUND */
    m_xcommon     = agc_mpac_shift_round(ip, agc_mpac_dmp(m_xcommon, xp), 4);    /* DMP 0D SL4R */
    ctx->x        = agc_mpac_store(m_xcommon);                                   /* X 2^17  */
    m_xcommon     = agc_mpac_norm(agc_mpac_dsq(m_xcommon), &n);                  /* DSQ NORM X1 */
    ctx->xsq      = agc_mpac_store(m_xcommon);                                   /* 0D      */
    ctx->xsq_n1   = n;
    m_xcommon     = agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(ctx->p), ctx->r1), 3);
    m_xcommon     = agc_mpac_sqrt(m_xcommon);                                    /* SQRT       */
    m_xcommon     = agc_mpac_dmp(m_xcommon, ctx->coga);                          /* DMP COGA   */
    ctx->kepc1    = agc_mpac_store(agc_mpac_shift_round(ip, m_xcommon, 3));      /* SL3R 2^17 */
    m_xcommon     = agc_mpac_bdsu(ip, agc_mpac_dload(ctx->r1a), AGC_CONIC_D1_64);
    ctx->infinflg = false;                                                       /* CLEAR INFINFLG */
    agc_conic_mirror_flag_(AGC_CONIC_FLAG_INFINFLG, false);                      /* mirror only */
    ctx->kepc2    = agc_mpac_store(m_xcommon);                                   /* KEPC2 2^6 */
} /* agc_conic_getx_ */

/**********************************************************************
 * PARAM (CONIC_SUBROUTINES.agc pp. 1289-1290) -- public entry for
 * callers outside the conic package (P37)
 *
 * FUNCTION NAME: agc_conic_param
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state; OVFIND
 *                                           left set on COGA overflow
 *   rvec      agc_vec_t                I    RVEC, 2^29 / 2^27 m
 *   vvec      agc_vec_t                I    VVEC, 2^7 / 2^5 m/cs
 *   body      agc_conic_body_t         I    central body
 *   out       agc_conic_param_out_t *  O    P, R1A, COGA, SNGAMMA and the
 *                                           GEOM debris
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL pointer or an unknown body
 *********************************************************************/
agc_status_t agc_conic_param(agc_interp_t *ip, agc_vec_t rvec, agc_vec_t vvec, agc_conic_body_t body, agc_conic_param_out_t *out)
{
    agc_conic_ctx_t ctx;                                  /* conic erasables */
    agc_status_t    st = agc_conic_ctx_init_(&ctx, body); /* MUTABLE,1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_words_t sngamma; /* 36D sin(gamma), 2^1 */
    agc_tp_words_t m;       /* MPAC at RTNPRM (P) */

    agc_conic_param_(ip, &ctx, rvec, vvec, &sngamma, &m);

    out->sngamma = agc_dp_from_words(sngamma);
    out->p       = agc_dp_from_words(ctx.p);
    out->r1a     = agc_dp_from_words(ctx.r1a);
    out->coga    = agc_dp_from_words(ctx.coga);
    out->r1      = agc_dp_from_words(ctx.r1);
    out->magvec2 = agc_dp_from_words(ctx.magvec2);
    out->ur1     = ctx.ur1;
    out->u2      = ctx.u2;
    out->un      = ctx.un;

    return (AGC_STATUS_OK);
}
