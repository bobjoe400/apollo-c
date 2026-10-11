/*
 * apsides.c -- APSIDES (GETECC, INFINAPO)
 * (CONIC_SUBROUTINES.agc, Comanche 055 pp. 1275-1276 description, code
 * p. 1303; Luminary 099 pp. 1171-1172, 1199; identical).
 *
 * From a state vector: ECC = sqrt(1 - P R1A), RP = R1 P / (1 + ECC),
 * RA = 2 R1/R1A - RP; RA is DPPOSMAX (LDPOSMAX) when it overflows the
 * position scale or the conic is not an ellipse.
 */
#include "agc/core/assert.h"
#include "internal.h"

/*
 * APSIDES (CONIC_SUBROUTINES.agc p. 1303, label APSIDES).
 * out->rapo (MPAC) and out->rperi (0D) 2^29 / 2^27 m, out->ecc 2^3.
 */
agc_status_t agc_conic_apsides(agc_interp_t *ip, agc_vec_t rvec, agc_vec_t vvec,
                               agc_conic_body_t body, agc_apsides_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_conic_ctx_t ctx;
    agc_status_t    st = agc_conic_ctx_init_(&ctx, body);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_interp_clear_ovf(ip); /* SETPD 0D, BOV +1 */
    agc_dp_t sngamma = 0;

    st = agc_conic_param_(ip, &ctx, rvec, vvec, &sngamma); /* CALL PARAM  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_interp_clear_ovf(ip);       /* BOV GETECC     */
    agc_tp_t m = agc_tp_from_dp(ctx.p); /* GETECC: DMP R1A SL4 */

    m = agc_tp_shift(ip, agc_conic_mdmp_(m, ctx.r1a), 4);
    m = agc_conic_msqrt_(
        ip, agc_conic_mbdsu_(ip, m, AGC_CONIC_D1_64));          /* BDSU SQRT */
    out->ecc = agc_tp_to_dp(m); /* ECC 2^3        */
    const agc_dp_t one_p_e =
        agc_tp_to_dp(agc_conic_mdad_(ip, m, AGC_CONIC_D1_8));

    m = agc_tp_shift(ip, agc_conic_mdmp_(agc_tp_from_dp(ctx.r1), ctx.p),
                     1);
    out->rperi =
        agc_tp_to_dp(agc_conic_mddv_(ip, m, one_p_e)); /* 0D = RP */
    int            n    = 0;
    const agc_dp_t r1an =
        agc_tp_to_dp(agc_conic_mnorm_(agc_tp_from_dp(ctx.r1a), &n));

    m = agc_tp_shift(ip, agc_tp_from_dp(ctx.r1), n - 5); /* SL* 0 -5,1 */
    m = agc_conic_mdsu_(ip, agc_conic_mddv_(ip, m, r1an),
                        out->rperi); /* DDV DSU */

    if (agc_interp_test_ovf(ip) || agc_conic_mbmn_(m))
    {                                   /* BOV BMN INFINAPO */
        out->rapo = AGC_CONIC_LDPOSMAX; /* DLOAD LDPOSMAX */
    }
    else
    {
        out->rapo = agc_tp_to_dp(m);
    }

    /* Copy the PARAM debris to *out. */
    out->r1      = ctx.r1;
    out->r1a     = ctx.r1a;
    out->p       = ctx.p;
    out->coga    = ctx.coga;
    out->magvec2 = ctx.magvec2;
    out->ur1     = ctx.ur1;
    out->u2      = ctx.u2;
    out->un      = ctx.un;

    return (AGC_STATUS_OK);
}
