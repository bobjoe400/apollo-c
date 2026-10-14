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
 *   scale or the conic is not an ellipse. Always AGC_STATUS_OK.
 *
 * ALGORITHM:
 *   ECC = sqrt(1 - P R1A), RP = R1 P / (1 + ECC), RA = 2 R1/R1A - RP.
 **********************************************************************/
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
 * RETURNS: none (any body other than AGC_CONIC_EARTH is the moon)
 **********************************************************************/
void agc_conic_apsides(agc_interp_t *ip, agc_vec_t rvec, agc_vec_t vvec, agc_conic_body_t body, agc_apsides_out_t *out)
{
    agc_conic_ctx_t ctx; /* PARAM results */

    agc_conic_ctx_init_(&ctx, body);

    agc_interp_clear_ovf(ip); /* SETPD 0D, BOV +1 */
    agc_dp_words_t sngamma;   /* 36D sin(gamma), 2^1 (unused) */
    agc_tp_words_t m;         /* MPAC: P 2^4 as PARAM leaves it */

    agc_conic_param_(ip, &ctx, rvec, vvec, &sngamma, &m); /* CALL PARAM  */

    agc_interp_clear_ovf(ip);                                 /* BOV GETECC     */
    m = agc_mpac_shift(ip, agc_mpac_dmp(m, ctx.r1a), 4);      /* DMP R1A SL4: 2^6 */
    m = agc_mpac_sqrt(agc_mpac_bdsu(ip, m, AGC_CONIC_D1_64)); /* BDSU D1/64 SQRT */

    const agc_dp_words_t ecc     = agc_mpac_store(m);                                   /* STORE ECC, 2^3 */
    const agc_dp_words_t one_p_e = agc_mpac_store(agc_mpac_dad(ip, m, AGC_CONIC_D1_8)); /* DAD PDDL: 1 + ECC, 2^3 */

    m = agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(ctx.r1), ctx.p), 1); /* R1 DMP P SL1 */

    const agc_dp_words_t rperi = agc_mpac_store(agc_mpac_ddv(ip, m, one_p_e));               /* DDV PDDL: 0D = RP */
    int                  n     = 0;                                                          /* NORM X1 shift count */
    const agc_dp_words_t r1an  = agc_mpac_store(agc_mpac_norm(agc_mpac_dload(ctx.r1a), &n)); /* R1A 2^(6-n) */

    m = agc_mpac_shift(ip, agc_mpac_dload(ctx.r1), n - 5);  /* SL* 0 -5,1 */
    m = agc_mpac_dsu(ip, agc_mpac_ddv(ip, m, r1an), rperi); /* DDV DSU */

    if (agc_interp_test_ovf(ip) || agc_conic_mbmn_(m))
    { /* BOV BMN INFINAPO */
        m = agc_mpac_dload(AGC_CONIC_LDPOSMAX); /* DLOAD LDPOSMAX */
    }

    /* Copy the results and the PARAM debris to *out. */
    out->rapo    = agc_dp_from_words(agc_mpac_store(m));
    out->rperi   = agc_dp_from_words(rperi);
    out->ecc     = agc_dp_from_words(ecc);
    out->r1      = agc_dp_from_words(ctx.r1);
    out->r1a     = agc_dp_from_words(ctx.r1a);
    out->p       = agc_dp_from_words(ctx.p);
    out->coga    = agc_dp_from_words(ctx.coga);
    out->magvec2 = agc_dp_from_words(ctx.magvec2);
    out->ur1     = ctx.ur1;
    out->u2      = ctx.u2;
    out->un      = ctx.un;
} /* agc_conic_apsides */
