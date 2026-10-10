/*
 * timeradius.c -- TIME-RADIUS (TIMERAD, TERMNVEC, CIRCULAR, BADR2)
 * (CONIC_SUBROUTINES.agc, Comanche 055 pp. 1272-1274 description, code
 * pp. 1301-1302; Luminary 099 pp. 1168-1170, 1197-1198; identical).
 *
 * Given a state and a radius RDESIRED (with the sign of the radial
 * velocity there, SGNRDOT), TIMERAD finds the true anomaly f at that
 * radius from the eccentricity vector (cos f = (R1 P / RDESIRED - 1)/ECC),
 * builds the terminal unit vector U2 = cos f UECC + sin f (UN x UECC), and
 * then proceeds as TIME-THETA with the angle between UR1 and U2.
 */
#include "internal.h"
#include "agc/core/assert.h"

/* TIMERAD from PARAM to UNIT (p. 1301): eccentricity vector
 * ((1 - R1A) UR1 - sqrt(P (2 - R1A)) COGA U2), 2^3. Its unit vector is
 * returned, |ECC| (36D) in *ecc.                                         */
static agc_vec_t timerad_ecc_vector(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                                    agc_dp_t *ecc)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(ecc != NULL);
    agc_tp_t m = agc_conic_mdsu_(ip, agc_conic_mload_(AGC_CONIC_D1_32), ctx->r1a);
    m = agc_conic_msqrt_(ip, agc_conic_mdmp_(m, ctx->p));   /* DMP P SQRT */
    m = agc_conic_mshift_(ip, agc_conic_mdmp_(m, ctx->coga), 4); /* DMP COGA SL4 */
    const agc_vec_t a = agc_vec_scale(ip, ctx->u2, agc_conic_mstore_(m)); /* 0D */
    m = agc_conic_mdsu_(ip, agc_conic_mload_(AGC_CONIC_D1_64), ctx->r1a);
    agc_vec_t e = agc_vec_scale(ip, ctx->ur1, agc_conic_mstore_(m)); /* VXSC UR1 */
    e = agc_vec_shl(ip, agc_vec_sub(ip, e, a), 4);          /* VSU VSL4   */
    return agc_vec_unit(ip, e, ecc, NULL);                  /* UNIT       */
}

/* TIMERAD from PDDL RDESIRED to CLEAR APSESW (p. 1301), with BADR2:
 * COSF and SINF (2^1) of the true anomaly at RDESIRED. Returns true when
 * RDESIRED is outside the apsides (APSESW): COSF = +-1, SINF = 0.        */
static bool timerad_anomaly(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                            const agc_timerad_in_t *in, agc_dp_t ecc,
                            agc_dp_t *cosf, agc_dp_t *sinf)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL && in != NULL);
    (void)AGC_ASSERT(cosf != NULL && sinf != NULL);
    int n = 0;
    const agc_dp_t rdn = agc_conic_mstore_(agc_conic_mnorm_(agc_conic_mload_(in->rdesired), &n));
    agc_tp_t m = agc_conic_mdmp_(agc_conic_mload_(ctx->r1), ctx->p);
    m = agc_conic_mddv_(ip, agc_conic_mshift_(ip, m, n), rdn); /* SL* 0,1 DDV */
    m = agc_conic_mddv_(ip, agc_conic_mdsu_(ip, m, AGC_CONIC_D1_16), ecc);
    *cosf = agc_conic_mstore_(m);                       /* STORE COSF     */
    bool bad = agc_interp_test_ovf(ip);                 /* BOV BADR2      */
    if (!bad) {
        m = agc_conic_mbdsu_(ip, agc_conic_mdsq_(m), AGC_CONIC_D1_4);
        bad = agc_conic_mbmn_(m);                       /* BMN BADR2      */
    }
    if (bad) {                                          /* BADR2          */
        *cosf = agc_conic_mstore_(agc_conic_msign_(agc_conic_mload_(AGC_CONIC_LODPHALF), *cosf));
        *sinf = AGC_CONIC_KEPZERO;
        return true;
    }
    m = agc_conic_msign_sp_(agc_conic_msqrt_(ip, m), in->sgnrdot); /* SQRT SIGN */
    *sinf = agc_conic_mstore_(m);
    return false;
}

/* TERMNVEC (p. 1302): terminal unit vector, CSTH and SNTH of the transfer
 * angle from UR1 into ctx.                                               */
static void timerad_termnvec(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                             agc_vec_t uecc, agc_dp_t cosf, agc_dp_t sinf)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(cosf >= AGC_DP_MIN && sinf >= AGC_DP_MIN);
    agc_vec_t w = agc_vec_shl(ip, agc_vec_scale(ip, ctx->un, sinf), 1); /* VXSC UN VSL1 */
    w = agc_vec_cross(ip, w, uecc);                     /* VXV 0D, PDVL   */
    agc_vec_t u2 = agc_vec_add(ip, agc_vec_scale(ip, uecc, cosf), w);
    u2 = agc_vec_shl(ip, u2, 1);                        /* VSL1 PUSH: U2  */
    agc_tp_t m = agc_vec_dot_tp(ip, u2, ctx->ur1);            /* DOT        */
    m = agc_conic_mddv_(ip, m, AGC_CONIC_DP1_4);        /* DDV DP1/4: limit */
    m = agc_conic_mshift_(ip, m, -1);                   /* SR1            */
    (void)agc_interp_test_ovf(ip);                      /* BOV +1         */
    ctx->csth = agc_conic_mstore_(m);                   /* CSTH 2^1       */
    agc_vec_t c = agc_vec_shl(ip, agc_vec_cross(ip, ctx->ur1, u2), 1);
    m = agc_conic_mshift_(ip, agc_vec_dot_tp(ip, c, ctx->un), 1); /* DOT SL1 */
    ctx->snth = agc_conic_mstore_(m);                   /* SNTH 2^1       */
}

/* CIRCULAR (p. 1302): SETPD 0, SETGO SOLNSW ABTCONIC. */
static agc_status_t timerad_circular(agc_conic_ctx_t *ctx, agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ctx != NULL && out != NULL)) { return AGC_STATUS_BAD_ARG; }
    agc_status_t st = agc_conic_mirror_flag_(AGC_CONIC_FLAG_SOLNSW, true);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_conic_time_out_fill_(ctx, out);
    out->solnsw = true;
    return agc_conic_abtconic_();
}

/*
 * TIMERAD (CONIC_SUBROUTINES.agc pp. 1301-1302, label TIMERAD).
 * See agc_nav.h for the status contract.
 */
agc_status_t agc_conic_timerad(agc_interp_t *ip, const agc_timerad_in_t *in,
                               agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(out != NULL && in->sgnrdot != 0)) { return AGC_STATUS_BAD_ARG; }
    agc_conic_time_out_clear_(out);
    agc_conic_ctx_t ctx;
    agc_status_t st = agc_conic_ctx_init_(&ctx, in->body);
    if (st != AGC_STATUS_OK) { return st; }
    (void)agc_interp_test_ovf(ip);                      /* SETPD 0, BOV +1 */
    ctx.vvec = in->vvec;
    agc_dp_t sngamma = 0;
    st = agc_conic_param_(ip, &ctx, in->rvec, in->vvec, &sngamma);
    if (st != AGC_STATUS_OK) { return st; }
    if (agc_interp_test_ovf(ip)) {                      /* BOV COGAOVFL   */
        return agc_conic_cogaovfl_(&ctx, out);
    }
    agc_dp_t ecc = 0;
    const agc_vec_t uecc = timerad_ecc_vector(ip, &ctx, &ecc);
    if (agc_interp_test_ovf(ip)) {                      /* BOV CIRCULAR   */
        return timerad_circular(&ctx, out);
    }
    agc_dp_t cosf = 0;
    agc_dp_t sinf = 0;
    const bool apsesw = timerad_anomaly(ip, &ctx, in, ecc, &cosf, &sinf);
    st = agc_conic_mirror_flag_(AGC_CONIC_FLAG_APSESW, apsesw); /* SET/CLEAR APSESW */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    timerad_termnvec(ip, &ctx, uecc, cosf, sinf);
    st = agc_conic_getx_(ip, &ctx);                     /* DLOAD P, CALL GETX */
    if (st != AGC_STATUS_OK) { return st; }
    st = agc_conic_mirror_flag_(AGC_CONIC_FLAG_SOLNSW, false); /* CLRGO SOLNSW */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_conic_commnout_(ip, &ctx, in->rvsw, out);  /* COMMNOUT       */
    out->apsesw = apsesw;
    out->solnsw = false;
    return st;
}
