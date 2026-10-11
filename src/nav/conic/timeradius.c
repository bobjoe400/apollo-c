/**********************************************************************
 * FILE NAME: timeradius.c
 *
 * PURPOSE:
 *   TIME-RADIUS (TIMERAD with TERMNVEC, CIRCULAR and BADR2): time of
 *   flight from a state to a given radius. TIMERAD finds the true anomaly
 *   at RDESIRED from the eccentricity vector, cos f = (R1 P / RDESIRED -
 *   1)/ECC, builds the terminal unit vector U2 = cos f UECC + sin f (UN x
 *   UECC), and then proceeds as TIME-THETA with the angle from UR1 to U2.
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1272-1274 (description),
 *   pp. 1301-1302 (code: TIMERAD, TERMNVEC, CIRCULAR, BADR2)
 *   Luminary099 CONIC_SUBROUTINES.agc pp. 1168-1170, 1197-1198 (identical)
 *
 * EXTERNAL VARIABLES:
 *   Name      Type  I/O  Scale  Description
 *   --------  ----  ---  -----  ------------------------------------
 *   APSESW    flag  O    -      RDESIRED outside the apsides (BADR2)
 *   SOLNSW    flag  O    -      set for a circular orbit (CIRCULAR)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_conic_param_      PARAM (geom.c)
 *   agc_conic_getx_       GETX (geom.c)
 *   agc_conic_commnout_   COMMNOUT (timetheta.c)
 *   agc_conic_cogaovfl_   COGAOVFL (timetheta.c)
 *   agc_conic_abtconic_   ABTCONIC, POODOO 00607 (common.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ABTCONIC (POODOO 00607, AGC_STATUS_POODOO) for a circular orbit
 *   (UNIT of the eccentricity vector overflows; SOLNSW), for COGA
 *   overflow (COGAFLAG) and for INFINFLG from GETX. RDESIRED beyond the
 *   apsides is not an abort: APSESW is set and the apsis is used.
 *
 * ALGORITHM:
 *   See PURPOSE; the conic is then solved by GETX, DELTIME and (unless
 *   RVSW) NEWSTATE through COMMNOUT.
 *********************************************************************/
#include "agc/core/assert.h"
#include "internal.h"

/**********************************************************************
 * TIMERAD from "PDDL NORM RDESIRED" to "CLEAR APSESW", and BADR2
 * (CONIC_SUBROUTINES.agc p. 1301) -- cosine and sine of the true anomaly
 * at RDESIRED
 *
 * FUNCTION NAME: timerad_anomaly
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  ------------------------
 *   ip        agc_interp_t *            I/O  interpreter state (OVFIND)
 *   ctx       const agc_conic_ctx_t *   I    R1, P from PARAM
 *   in        const agc_timerad_in_t *  I    RDESIRED, SGNRDOT
 *   ecc       agc_dp_words_t            I    36D |ECC|, 2^3
 *   cosf      agc_dp_words_t *          O    COSF, 2^1
 *   sinf      agc_dp_words_t *          O    MPAC sin f with the sign of
 *                                            SGNRDOT, 2^1
 *
 * RETURNS: bool -- true when RDESIRED is outside the apsides (APSESW):
 *          COSF = +-.5 (LODPHALF), SINF = 0
 *********************************************************************/
static bool timerad_anomaly(agc_interp_t *ip, const agc_conic_ctx_t *ctx, const agc_timerad_in_t *in, agc_dp_words_t ecc, agc_dp_words_t *cosf, agc_dp_words_t *sinf)
{
    int                  n   = 0;                                                                                /* NORM X1 shift count */
    const agc_dp_words_t rdn = agc_mpac_store(agc_mpac_norm(agc_mpac_dload(agc_dp_to_words(in->rdesired)), &n)); /* PDDL: 2^(29-n) */
    agc_tp_words_t       m   = agc_mpac_dmp(agc_mpac_dload(ctx->r1), ctx->p);                                    /* R1 DMP P, 2^33 / 2^31 */

    m        = agc_mpac_ddv(ip, agc_mpac_shift(ip, m, n), rdn);             /* SL* 0,1 DDV */
    m        = agc_mpac_ddv(ip, agc_mpac_dsu(ip, m, AGC_CONIC_D1_16), ecc); /* DSU D1/16 DDV 36D */
    *cosf    = agc_mpac_store(m);                                           /* STORE COSF     */
    bool bad = agc_interp_test_ovf(ip);                                     /* BOV BADR2      */

    if (!bad)
    {
        m   = agc_mpac_bdsu(ip, agc_mpac_dsq(m), AGC_CONIC_D1_4);
        bad = agc_conic_mbmn_(m); /* BMN BADR2      */
    }

    if (bad)
    { /* BADR2          */
        *cosf = agc_mpac_store(agc_mpac_sign(agc_mpac_dload(AGC_CONIC_LODPHALF), *cosf)); /* DLOAD LODPHALF SIGN COSF */
        *sinf = AGC_CONIC_KEPZERO;

        return (true);
    }

    m     = agc_conic_msign_sp_(agc_mpac_sqrt(ip, m), in->sgnrdot); /* SQRT SIGN */
    *sinf = agc_mpac_store(m);

    return (false);
} /* timerad_anomaly */

/**********************************************************************
 * TERMNVEC (CONIC_SUBROUTINES.agc p. 1302) -- terminal unit vector and
 * the CSTH, SNTH of the transfer angle from UR1
 *
 * FUNCTION NAME: timerad_termnvec
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND cleared)
 *   ctx       agc_conic_ctx_t *  I/O  UN, UR1 in; CSTH, SNTH 2^1 out
 *   uecc      agc_vec_t          I    unit eccentricity vector, 2^1
 *   cosf      agc_dp_words_t     I    COSF, 2^1
 *   sinf      agc_dp_words_t     I    MPAC sin f, 2^1
 *********************************************************************/
static void timerad_termnvec(agc_interp_t *ip, agc_conic_ctx_t *ctx, agc_vec_t uecc, agc_dp_words_t cosf, agc_dp_words_t sinf)
{
    agc_vec_t w = agc_vec_shl(ip, agc_conic_vxsc_(ip, agc_mpac_dload(sinf), ctx->un), 1); /* VXSC UN VSL1, 2^1 */

    w            = agc_vec_cross(ip, w, uecc);                                          /* VXV 0D, PDVL: 2^2 */
    agc_vec_t u2 = agc_vec_add(ip, agc_conic_vxsc_(ip, agc_mpac_dload(cosf), uecc), w); /* VXSC COSF VAD, 2^2 */

    u2               = agc_vec_shl(ip, u2, 1);         /* VSL1 PUSH: U2 2^1 */
    agc_tp_words_t m = agc_mpac_dot(ip, u2, ctx->ur1); /* DOT UR1, 2^2      */

    m           = agc_mpac_ddv(ip, m, AGC_CONIC_DP1_4);                /* DDV DP1/4: limit */
    m           = agc_mpac_shift(ip, m, -1);                           /* SR1               */
    agc_interp_clear_ovf(ip);                                          /* BOV +1            */
    ctx->csth   = agc_mpac_store(m);                                   /* CSTH 2^1          */
    agc_vec_t c = agc_vec_shl(ip, agc_vec_cross(ip, ctx->ur1, u2), 1); /* VXV VSL1, 2^1 */

    m = agc_mpac_shift(ip, agc_mpac_dot(ip, c, ctx->un), 1); /* DOT SL1 */

    ctx->snth = agc_mpac_store(m); /* SNTH 2^1 */
}

/**********************************************************************
 * TIMERAD from "DLOAD D1/32" to "VSL4 UNIT" (CONIC_SUBROUTINES.agc
 * p. 1301) -- eccentricity vector ((1 - R1A) UR1 - sqrt(P (2 - R1A))
 * COGA U2) and its unit vector
 *
 * FUNCTION NAME: timerad_eccvec
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state; OVFIND
 *                                           set when UNIT overflows
 *                                           (circular orbit)
 *   ctx       const agc_conic_ctx_t *  I    P, R1A, COGA, UR1, U2
 *   ecc       agc_dp_words_t *         O    36D |ECC|, 2^3
 *
 * RETURNS: agc_vec_t -- UNIT(ECC), 2^1
 *********************************************************************/
static agc_vec_t timerad_eccvec(agc_interp_t *ip, const agc_conic_ctx_t *ctx, agc_dp_words_t *ecc)
{
    agc_tp_words_t m = agc_mpac_dsu(ip, agc_mpac_dload(AGC_CONIC_D1_32), ctx->r1a); /* MPAC, 2^6 */

    m                 = agc_mpac_sqrt(ip, agc_mpac_dmp(m, ctx->p));        /* DMP P SQRT */
    m                 = agc_mpac_shift(ip, agc_mpac_dmp(m, ctx->coga), 4); /* DMP COGA SL4 */
    const agc_vec_t a = agc_conic_vxsc_(ip, m, ctx->u2);                   /* VXSC U2 PDDL: 0D, 2^7 */

    m             = agc_mpac_dsu(ip, agc_mpac_dload(AGC_CONIC_D1_64), ctx->r1a); /* D1/64 DSU R1A */
    agc_vec_t e   = agc_conic_vxsc_(ip, m, ctx->ur1);                            /* VXSC UR1, 2^7 */
    agc_dp_t  mag = 0;                                                           /* 36D |ECC|, 2^3 */

    e = agc_vec_shl(ip, agc_vec_sub(ip, e, a), 4); /* VSU VSL4: ECC 2^3 */
    e = agc_vec_unit(ip, e, &mag, NULL);           /* UNIT */

    *ecc = agc_dp_to_words(mag);

    return (e);
}

/**********************************************************************
 * CIRCULAR (CONIC_SUBROUTINES.agc p. 1302) -- SETPD 0, SETGO SOLNSW
 * ABTCONIC: the eccentricity vector is too short to define the apsides
 *
 * FUNCTION NAME: timerad_circular
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   ctx       agc_conic_ctx_t *       I    conic erasables (debris)
 *   out       agc_conic_time_out_t *  O    debris, solnsw set
 *
 * RETURNS: agc_status_t -- AGC_STATUS_POODOO or AGC_STATUS_BAILOUT
 *          (ABTCONIC)
 *********************************************************************/
static agc_status_t timerad_circular(agc_conic_ctx_t *ctx, agc_conic_time_out_t *out)
{
    agc_conic_mirror_flag_(AGC_CONIC_FLAG_SOLNSW, true);
    agc_conic_time_out_fill_(ctx, out);
    out->solnsw = true;

    return (agc_conic_abtconic_());
}

/**********************************************************************
 * TIMERAD (CONIC_SUBROUTINES.agc pp. 1301-1302, label TIMERAD) -- time of
 * flight from a state to a radius, and the state there
 *
 * FUNCTION NAME: agc_conic_timerad
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  --------------------------
 *   ip        agc_interp_t *            I/O  interpreter state (OVFIND)
 *   in        const agc_timerad_in_t *  I    RVEC, VVEC, RDESIRED
 *                                            2^29 / 2^27 m, SGNRDOT,
 *                                            RVSW, central body
 *   out       agc_conic_time_out_t *    O    T 2^28 cs, RNEW, VNEW,
 *                                            APSESW, SOLNSW, flags and
 *                                            conic debris
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_POODOO for ABTCONIC
 *          (CIRCULAR, COGAFLAG, INFINFLG); AGC_STATUS_BAD_ARG for a NULL
 *          pointer, SGNRDOT = 0 or an unknown body
 *********************************************************************/
agc_status_t agc_conic_timerad(agc_interp_t *ip, const agc_timerad_in_t *in, agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(out != NULL && in->sgnrdot != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_conic_time_out_clear_(out);
    agc_conic_ctx_t ctx;                                      /* conic erasables */
    agc_status_t    st = agc_conic_ctx_init_(&ctx, in->body); /* MUTABLE,1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_interp_clear_ovf(ip); /* SETPD 0, BOV +1 */
    ctx.vvec = in->vvec;
    agc_dp_words_t sngamma;   /* 36D sin(gamma), 2^1 (unused) */
    agc_tp_words_t m;         /* MPAC at RTNPRM (P) */

    agc_conic_param_(ip, &ctx, in->rvec, in->vvec, &sngamma, &m);

    if (agc_interp_test_ovf(ip))
    { /* BOV COGAOVFL   */
        return (agc_conic_cogaovfl_(&ctx, out));
    }

    agc_dp_words_t  ecc  = AGC_CONIC_KEPZERO;              /* 36D |ECC| 2^3 */
    const agc_vec_t uecc = timerad_eccvec(ip, &ctx, &ecc); /* UNIT(ECC) 2^1 */

    if (agc_interp_test_ovf(ip))
    { /* BOV CIRCULAR   */
        return (timerad_circular(&ctx, out));
    }

    agc_dp_words_t cosf   = AGC_CONIC_KEPZERO;                                /* COSF, 2^1 */
    agc_dp_words_t sinf   = AGC_CONIC_KEPZERO;                                /* sin f, 2^1 */
    const bool     apsesw = timerad_anomaly(ip, &ctx, in, ecc, &cosf, &sinf); /* BADR2 taken */

    agc_conic_mirror_flag_(AGC_CONIC_FLAG_APSESW, apsesw); /* SET/CLEAR */
    timerad_termnvec(ip, &ctx, uecc, cosf, sinf);
    agc_conic_getx_(ip, &ctx, agc_mpac_dload(ctx.p));      /* DLOAD P, CALL GETX */

    agc_conic_mirror_flag_(AGC_CONIC_FLAG_SOLNSW, false);       /* CLRGO SOLNSW */
    st          = agc_conic_commnout_(ip, &ctx, in->rvsw, out); /* COMMNOUT */
    out->apsesw = apsesw;
    out->solnsw = false;

    return (st);
} /* agc_conic_timerad */
