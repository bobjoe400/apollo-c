/**********************************************************************
 * FILE NAME: apsides.c
 *
 * PURPOSE:
 *   APSIDES (with GETECC and INFINAPO): apoapsis, periapsis and
 *   eccentricity of the conic through a state vector.
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1275-1276 (description),
 *   p. 1303 (code: APSIDES, GETECC, INFINAPO)
 *   Luminary099 CONIC_SUBROUTINES.agc pp. 1171-1172, 1199 (identical)
 *
 * EXTERNAL VARIABLES:
 *   none (the conic erasables are fields of agc_conic_ctx_t)
 *
 * EXTERNAL REFERENCES:
 *   Name                 Description
 *   -------------------  ------------------------------------------
 *   agc_conic_param_     PARAM: P, R1A, COGA, R1 from RVEC, VVEC
 *   agc_conic_m*_        one interpretive op on the TP MPAC (common.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; RA is LDPOSMAX (INFINAPO) when it overflows the position
 *   scale or the conic is not an ellipse. AGC_STATUS_BAD_ARG for a
 *   NULL pointer or an unknown body.
 *
 * ALGORITHM:
 *   ECC = sqrt(1 - P R1A), RP = R1 P / (1 + ECC), RA = 2 R1/R1A - RP.
 **********************************************************************/
#include "agc/core/assert.h"
#include "internal.h"

/**********************************************************************
 * APSIDES (CONIC_SUBROUTINES.agc p. 1303, labels APSIDES, GETECC,
 * INFINAPO) -- apsides and eccentricity of a conic
 *
 * FUNCTION NAME: agc_conic_apsides
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ------------------------------
 *   ip        agc_interp_t *      I/O  interpreter state (OVFIND)
 *   rvec      agc_vec_t           I    RVEC position, 2^29 / 2^27 m
 *   vvec      agc_vec_t           I    VVEC velocity, 2^7 / 2^5 m/cs
 *   body      agc_conic_body_t    I    central body (MUTABLE index)
 *   out       agc_apsides_out_t * O    rapo (MPAC) and rperi (0D)
 *                                      2^29 / 2^27 m, ecc 2^3, and the
 *                                      PARAM debris
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL pointer or an unknown body
 **********************************************************************/
agc_status_t agc_conic_apsides(agc_interp_t *ip, agc_vec_t rvec, agc_vec_t vvec,
                               agc_conic_body_t body, agc_apsides_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_conic_ctx_t ctx;                                    /* PARAM results */
    agc_status_t    st = agc_conic_ctx_init_(&ctx, body); /* MUTABLE,1     */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_interp_clear_ovf(ip); /* SETPD 0D, BOV +1 */
    agc_dp_t sngamma = 0;     /* 36D sin(gamma), 2^1 (unused) */

    st = agc_conic_param_(ip, &ctx, rvec, vvec, &sngamma); /* CALL PARAM  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_interp_clear_ovf(ip);           /* BOV GETECC     */
    agc_tp_t m = agc_tp_from_dp(ctx.p); /* MPAC, 2^4 then 2^3 */

    m = agc_tp_shift(ip, agc_conic_mdmp_(m, ctx.r1a), 4); /* DMP R1A SL4 */
    m = agc_conic_msqrt_(
        ip, agc_conic_mbdsu_(ip, m, AGC_CONIC_D1_64));    /* BDSU SQRT */
    out->ecc = agc_tp_to_dp(m);                           /* ECC 2^3   */
    const agc_dp_t one_p_e = agc_tp_to_dp(
        agc_conic_mdad_(ip, m, AGC_CONIC_D1_8)); /* 1 + ECC, 2^3 */

    m = agc_tp_shift(ip, agc_conic_mdmp_(agc_tp_from_dp(ctx.r1), ctx.p),
                     1);
    out->rperi =
        agc_tp_to_dp(agc_conic_mddv_(ip, m, one_p_e)); /* 0D = RP */
    int            n    = 0; /* NORM X1 shift count */
    const agc_dp_t r1an = agc_tp_to_dp(
        agc_conic_mnorm_(agc_tp_from_dp(ctx.r1a), &n)); /* R1A 2^(6-n) */

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
} /* agc_conic_apsides */
