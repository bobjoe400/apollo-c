/*
 * velocity.c -- P37_P70.agc (Comanche055, pp. 917-930): the
 * post-impulse velocity of the return-to-earth conic.
 *
 *   V2T100 .. V2T1X  (pp. 917-923) "A POST IMPULSE VELOCITY VECTOR (V2(T1))
 *                    IS COMPUTED WHICH EITHER (1) MEETS THE INPUT VELOCITY
 *                    CHANGE DESIRED (RTEDVD) IN A MINIMUM TIME OR (2) IF A
 *                    VELOCITY CHANGE ISN'T SPECIFIED (RTEDVD = 0), ... WHICH
 *                    MINIMIZES THE IMPULSE (DV) AND CONSEQUENTLY FUEL."
 *   GAMDV10 .. GAMDVX (pp. 924-927) the X(T1)-DV iterator
 *   DVCALC           (pp. 928-929) V2(T1)/, DV, PCON for a given X(T1)
 *   XT1LIM           (p. 930) bounds on the independent variable X(T1)
 *
 * The independent variable X(T1) is the cotangent of the post-impulse flight
 * path angle measured from the vertical (B5); X(T2) the same at RCON (B0).
 * The conic through R(T1) and RCON with entry cotangent X(T2) has
 *   PCON = 2 R1 (LAMBDA - 1) / (BETA1 LAMBDA^2 - 1 - X(T1)^2),
 *   LAMBDA = R1/RCON, BETA1 = 1 + X(T2)^2,
 *   V2(T1) = sqrt(MU PCON)/R1 (X(T1) UR1 + UH).
 *
 * The push-down list of V2T100 (00D..28D) is the struct rte_v2t_t. The AGC
 * normalises R(T1) and RCON and carries the difference of the two NORM
 * counts in X1 (pl->x1); every "SL* 0-7,1" etc. below uses it exactly as the
 * listing does: the count of an SL* or SR* "a,j" is a - C(Xj), and NORM leaves minus
 * the shift count in the index register.
 *
 * The module erasables (rte_state) live in this file so that the arithmetic
 * files link without the job layer (return_to_earth.c).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "internal.h"

/* Constants (P37_P70.agc pp. 931-933, words read from the Comanche055 rope). */
static const agc_dp_t ONE_B1 =
    134217728; /* 1RTEB1   20000 00000   1.0 B-1          */
static const agc_dp_t ONE_B11 =
    131072; /* 1RTEB25 +1 00010 00000 1.0 B-11       */
static const agc_dp_t ONE_B17 =
    2048; /* 1RTEB17  00000 04000   1.0 B-17         */
static const agc_dp_t TWO_B1 =
    268435455; /* 2RTEB1   37777 37777   POSMAX (2.0 B1)  */
static const agc_dp_t RTMURTE =
    204442113; /* RTMURTE  30276 05001   199650.501 B-18,
                                                sqrt(mu earth), m^1.5/cs        */
static const agc_dp_t EPC9RTE =
    8; /* EPC9RTE  00000 00010   1. B-25 (X(T1) B5) */
static const agc_dp_t EPC10RTE =
    210; /* EPC10RTE 00000 00322   .0001 B-7 m/cs   */
static const agc_dp_t K_1_1_B1 =
    147639501; /* 1.1RTEB1 21463 06315   1.1 B-1          */

/* The P37 erasables and the interpreter state of the P37 job (OVFIND is
 * carried from one routine to the next, as in the AGC's MPAC area).       */
static rte_erasable_t rte_mem; /* P37 erasables, scales in internal.h */
static agc_interp_t   rte_interp; /* P37 job interpreter state */

/**********************************************************************
 * AGC: none (P37_P70.agc erasables pp. 931-933) -- the P37 erasable set
 *
 * FUNCTION NAME: rte_state
 *
 * ARGUMENTS: none
 *
 * RETURNS: rte_erasable_t * -- the module's P37 erasables (never NULL)
 **********************************************************************/
rte_erasable_t *rte_state(void)
{
    return (&rte_mem);
}

/**********************************************************************
 * AGC: none (P37 job MPAC area) -- the interpreter state of the P37 job
 *
 * FUNCTION NAME: rte_ip
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_interp_t * -- the P37 job's interpreter state (OVFIND
 *          carried between routines)
 **********************************************************************/
agc_interp_t *rte_ip(void)
{
    return (&rte_interp);
}

/**********************************************************************
 * DSU X / BDSU X on a TP MPAC (INTERPRETER p. 1133-1134, both end in
 * ENDVXV): DCS (or DCOM) and DAS on MPAC,+1 only; MPAC+2, the low word of
 * the preceding DMP, SL* or SQRT, is left as it was, so for BDSU it keeps
 * its sign: X - (hi + lo) becomes X - hi + lo. THETA1, BETA10 and the
 * PREC155 terms are small differences of nearly equal terms, so that word
 * carries much of their precision (finding F14).
 *
 * FUNCTION NAME: rte_tp_minus
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *   m         agc_tp_t        I    TP MPAC before the DSU/BDSU, scale of x
 *   x         agc_dp_t        I    DP operand X, same scale
 *   reverse   bool            I    false: DSU (m - x); true: BDSU (x - m)
 *
 * RETURNS: agc_tp_t -- agc_tp_t -- MPAC after the instruction: the DP
 *          difference with MPAC+2 of m kept as it was
 **********************************************************************/
static agc_tp_t rte_tp_minus(agc_interp_t *ip, agc_tp_t m, agc_dp_t x,
                             bool reverse)
{
    const agc_dp_t hi  = agc_tp_to_dp(m); /* MPAC,+1 of m, scale of x */
    const agc_tp_t lo  = m - agc_tp_from_dp(hi); /* MPAC+2, 2^-28 of x scale */
    const agc_tp_t one = (agc_tp_t) 1 << AGC_SP_BITS; /* 1.0 of MPAC+2, B0 */

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT((lo > -one) && (lo < one)))
    {
        return (m); /* no valid result */
    }

    const agc_dp_t top = reverse ? /* BDSU : DSU */
                         agc_dp_sub(ip, x, hi) : agc_dp_sub(ip, hi, x);

    return (agc_tp_from_dp(top) + lo);
}

/**********************************************************************
 * DSU X on a TP MPAC (INTERPRETER.agc pp. 1133-1134)
 *
 * FUNCTION NAME: rte_tp_dsu
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *   m         agc_tp_t        I    TP MPAC, scale of x
 *   x         agc_dp_t        I    DP operand X, same scale
 *
 * RETURNS: agc_tp_t -- agc_tp_t -- m - x with MPAC+2 of m kept (see
 *          rte_tp_minus)
 **********************************************************************/
agc_tp_t rte_tp_dsu(agc_interp_t *ip, agc_tp_t m, agc_dp_t x)
{
    return (rte_tp_minus(ip, m, x, false));
}

/**********************************************************************
 * BDSU X on a TP MPAC (INTERPRETER.agc pp. 1133-1134)
 *
 * FUNCTION NAME: rte_tp_bdsu
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *   x         agc_dp_t        I    DP operand X, scale of m
 *   m         agc_tp_t        I    TP MPAC, same scale
 *
 * RETURNS: agc_tp_t -- agc_tp_t -- x - m with MPAC+2 of m kept (see
 *          rte_tp_minus)
 **********************************************************************/
agc_tp_t rte_tp_bdsu(agc_interp_t *ip, agc_dp_t x, agc_tp_t m)
{
    return (rte_tp_minus(ip, m, x, true));
}

/**********************************************************************
 * VSL* / VSR* with a computed count (INTERPRETER.agc p. 1155): n > 0 left
 * (wraps, OVFIND), n < 0 right (rounded, as every vector right shift).
 *
 * FUNCTION NAME: rte_vshift
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *   v         agc_vec_t       I    vector in MPAC, any scale
 *   n         int             I    shift count, -63..63 (left when
 *                                  positive)
 *
 * RETURNS: agc_vec_t -- agc_vec_t -- v x 2^n at the AGC's rounding
 **********************************************************************/
static agc_vec_t rte_vshift(agc_interp_t *ip, agc_vec_t v, int n)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT((n > -64) && (n < 64)))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return ((n >= 0) ? /* left : right */
            agc_vec_shl(ip, v, n) : agc_vec_shr(v, -n));
}

/**********************************************************************
 * XT1LIM / XT1LIM2 / XT1LIM3 / XT1LIM5 (P37_P70.agc p. 930): bound on the
 * independent variable X(T1), B5: BETA10 = BETA5 (MA - R(T1)) / (MA - RCON)
 * - 1            (B11, then B10) X(T1)LIM = SQRT(BETA10); POSMAX if BETA10
 * overflows; 0 if negative. MA B30 (MAMAX1 is negative: a hyperbola), RCON
 * and R(T1) B29, BETA5 at B(2 - X1) with X1 = pl->x1 left by V2T100.
 *
 * FUNCTION NAME: rte_xt1lim
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------------
 *   ip        agc_interp_t *          IO   interpreter state of the job:
 *                                          OVFIND, DVSIGN, alarms
 *   rs        const rte_erasable_t *  I    P37 erasables (RTE names, scales
 *                                          in internal.h)
 *   ma        agc_dp_t                I    semi-major axis limit MA (MAMAX1
 *                                          or MAMAX2), m B30
 *   pl        const rte_v2t_t *       I    V2T100 push list (BETA5, X1)
 *   xt1lim    agc_dp_t *              O    X(T1)LIM, B5 (POSMAX on
 *                                          overflow, 0 when BETA10 < 0)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a failed
 *          assertion
 **********************************************************************/
agc_status_t rte_xt1lim(agc_interp_t *ip, const rte_erasable_t *rs, agc_dp_t ma,
                        const rte_v2t_t *pl, agc_dp_t *xt1lim)
{
    int n2 = 0; /* NORM shift count (X2) */

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) ||
        !AGC_ASSERT((pl != NULL) && (xt1lim != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t den = /* MA - RCON/2, B30 x 2^n2 */
                   agc_dp_sub(ip, ma, agc_dp_shr(rs->rcon, 1));

    if (!AGC_ASSERT(den != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    den = agc_dp_norm(den, &n2);                       /* NORM X2      */
    agc_dp_t q = agc_dp_sub(ip, ma, agc_dp_shr(rs->r_t1, 1)); /* ratio, B3 */

    q = agc_dp_div(ip, q, den);                      /* DDV          */
    q = agc_dp_shl(ip, q, n2 - 3);                   /* SL* 0-3,2 B3 */
    agc_tp_t t = agc_dp_mul_tp(q, pl->beta5); /* DMP 28D: q BETA5, TP B11 */

    t = agc_tp_shift(ip, t, -6 - pl->x1);            /* SL* 0-6,1    */
    agc_tp_t beta10 = rte_tp_dsu(ip, t, ONE_B11);    /* DSU: B11     */

    beta10 = agc_tp_shift(ip, beta10, 1);            /* SL1 -> B10   */

    if (agc_interp_test_ovf(ip))
    {
        beta10 = agc_tp_from_dp(TWO_B1); /* XT1LIM2      */
    }
    else if (agc_tp_to_dp(beta10) < 0)
    {
        *xt1lim = 0; /* BMN XT1LIM5  */

        return (AGC_STATUS_OK);
    }

    *xt1lim = agc_tp_sqrt(ip, beta10); /* XT1LIM3, B5  */

    if (!AGC_ASSERT(*xt1lim >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* rte_xt1lim */

/**********************************************************************
 * XT1LIM (P37_P70.agc p. 930), public form kept from guid-entry: BETA5 at
 * B9, i.e. the nominal X1 = -7 of a trans-earth state (R(T1) ~ 2^7 RCON).
 *
 * FUNCTION NAME: agc_return_to_earth_xt1lim
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   ma        agc_dp_t    I    semi-major axis limit MA, m B30
 *   rcon      agc_dp_t    I    conic reentry radius RCON, m B29
 *   r_t1      agc_dp_t    I    radius at TIG R(T1), m B29
 *   beta5     agc_dp_t    I    BETA5 = BETA1 LAMBDA, B9
 *   xt1lim    agc_dp_t *  O    X(T1)LIM, B5
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a failed
 *          assertion
 **********************************************************************/
agc_status_t agc_return_to_earth_xt1lim(agc_dp_t ma, agc_dp_t rcon,
                                        agc_dp_t r_t1, agc_dp_t beta5,
                                        agc_dp_t *xt1lim)
{
    rte_erasable_t rs = {0};
    rte_v2t_t      pl = {0};
    agc_interp_t   ip; /* scratch interpreter state */

    if (!AGC_ASSERT(xt1lim != NULL) || !AGC_ASSERT(rcon > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_init(&ip);
    rs.rcon  = rcon;
    rs.r_t1  = r_t1;
    pl.beta5 = beta5;
    pl.x1    = -7;

    return (rte_xt1lim(&ip, &rs, ma, &pl, xt1lim));
}

/**********************************************************************
 * DVCALC (P37_P70.agc pp. 928-929): for the current X(T1) (B5) PCON =
 * THETA2 / (THETA1 - X(T1)^2)                             m B28 V2(T1)/ =
 * THETA3 PCON^.5 (X(T1) UR1/ + UH/)                    m/cs B7 DV =
 * ABVAL(V2(T1)/ - V(T1)/)                                   m/cs B7 THETA1
 * - X^2 is PUSHed (DP) after NORM and THETA2 is cut to DP by RTB DPMODE, so
 * DDV sees DP words as here; but PCON's SR* leaves bits in MPAC+2 and SQRT
 * reads them (verified word for word against the rope). SQRT of a negative
 * PCON aborts (POODOO 01302) as in the interpreter.
 *
 * FUNCTION NAME: rte_dv_calc
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     IO   interpreter state of the job: OVFIND,
 *                                     DVSIGN, alarms
 *   rs        rte_erasable_t *   IO   P37 erasables (RTE names, scales in
 *                                     internal.h)
 *   pl        const rte_v2t_t *  I    V2T100 push list (THETA1..3, X1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK with PCON, V2(T1)/, DV in rs;
 *          POODOO 01302 (the agc_poodoo status) when PCON is negative;
 *          AGC_STATUS_BAD_ARG on a failed assertion
 **********************************************************************/
agc_status_t rte_dv_calc(agc_interp_t *ip, rte_erasable_t *rs,
                         const rte_v2t_t *pl)
{
    int      n1   = 0; /* NORM shift count (X1) */
    int      n2   = 0; /* NORM shift count (X2) */
    agc_dp_t root = 0; /* SQRT(PCON), m^.5 B14 */

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(pl != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_tp_t t =
        agc_tp_shift(ip, agc_dp_sq_tp(rs->x_t1), -7); /* DSQ SR 7: B17 */

    t = agc_tp_add(ip, pl->theta1, -t);    /* DCOMP TAD 02D */
    agc_dp_t den = /* THETA1-X^2, B17 x 2^n1 */
                   agc_tp_to_dp(agc_tp_norm(t, &n1));
    agc_dp_t num = /* THETA2, B38 x 2^n2 */
                   agc_tp_to_dp(agc_tp_norm(pl->theta2, &n2));

    num = agc_dp_shr(num, 1);                  /* DPMODE SR1    */
    agc_dp_t q = agc_dp_div(ip, num, den); /* XSU,2 X1; DDV: B0 x 2^(n1-n2) */

    /* SR* 6,2 truncates into MPAC+2; STORE PCON keeps the top two words
     * but SQRT reads the whole TP MPAC.                                  */
    agc_tp_t pcon_tp =
        agc_tp_shift(ip, agc_tp_from_dp(q), -(6 - (n1 - n2))); /* B28 */

    rs->pcon = agc_tp_to_dp(pcon_tp); /* STORE PCON    */

    if (agc_dp_sqrt_chk(ip, rs->pcon, &root) != AGC_STATUS_OK)
    {
        rs->alarm_code = RTE_POODOO_SQRT;

        return (agc_poodoo(RTE_POODOO_SQRT));
    }

    root = agc_tp_sqrt(ip, pcon_tp); /* SQRT          */

    agc_dp_t k = /* NORM: THETA3 SQRT(PCON), x 2^n1 */
                 agc_tp_to_dp(agc_tp_norm(agc_dp_mul_tp(root, pl->theta3),
                                          &n1));
    agc_dp_t  xn = agc_dp_norm(rs->x_t1, &n2); /* NORM X2: X(T1), B5 x 2^n2 */
    agc_vec_t v  = agc_vec_scale(ip, rs->ur1, xn); /* X(T1) UR1/, B7 x 2^n */

    v = agc_vec_scale(ip, v, k);        /* XAD,2 X1; 28D */
    v = rte_vshift(ip, v, 9 - n1 - n2); /* VSR* 0-9D,2   */
    agc_vec_t h = agc_vec_scale(ip, rs->uh, k); /* UH/ VXSC 28D: B7 x 2^n1 */

    h          = rte_vshift(ip, h, 4 - n1);      /* VSR* 0-4,1    */
    rs->v2_t1v = agc_vec_add(ip, v, h);          /* V2(T1)/ B7    */
    rs->dv     = agc_vec_abval(ip, agc_vec_sub(ip, rs->v2_t1v, rs->v_t1v));

    if (!AGC_ASSERT(rs->dv >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* rte_dv_calc */

/**********************************************************************
 * GAMDV10 .. GAMDV20 (p. 925): first DVCALC, and if the bounds are closer
 * than DX(T1)MAX, DX(T1) = BETA8 SIGN(DX(T1)) / 2. Returns NOT_FOUND for
 * GAMDVX (bounds within EPC9RTE: done).
 *
 * FUNCTION NAME: gamdv_start
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *   pl        rte_v2t_t *       IO   P37 push-down list of the V2(T1) /
 *                                    TPMODE search
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK to iterate; AGC_STATUS_NOT_FOUND
 *          when the bounds are within EPC9RTE (GAMDVX: done);
 *          AGC_STATUS_BAD_ARG on a failed assertion; else the DVCALC status
 **********************************************************************/
static agc_status_t gamdv_start(agc_interp_t *ip, rte_erasable_t *rs,
                                rte_v2t_t *pl)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(pl != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = rte_dv_calc(ip, rs, pl); /* DVCALC status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t beta8 =
        agc_dp_sub(ip, pl->x_t1max, pl->x_t1min); /* BETA8 B5      */

    if (agc_interp_test_ovf(ip))
    {
        return (AGC_STATUS_OK); /* BOV GAMDV20   */
    }

    if (agc_dp_sub(ip, beta8, EPC9RTE) < 0)
    {
        return (AGC_STATUS_NOT_FOUND); /* GAMDVX        */
    }

    if (agc_dp_sub(ip, beta8, pl->dx_t1max) < 0)
    { /* GAMDV15       */
        pl->dx_t1 = agc_dp_shr(agc_dp_sign(beta8, pl->dx_t1), 1);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GAMDV35 .. GAMDV47 (pp. 926-927), time critical mode: secant step DX(T1)
 * = (X(T1) - X(T1),) DVERR / (DV - DV,), limited to DX(T1)MAX. Returns
 * NOT_FOUND for GAMDVX (|DVERR| < EPC10RTE: done).
 *
 * FUNCTION NAME: gamdv_secant
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *   pl        rte_v2t_t *       IO   P37 push-down list of the V2(T1) /
 *                                    TPMODE search
 *   x_prev    agc_dp_t          I    X(T1), of the previous step, B5
 *   dv_prev   agc_dp_t          I    DV, of the previous step, m/cs B7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK with DX(T1) set;
 *          AGC_STATUS_NOT_FOUND when |DVERR| < EPC10RTE (GAMDVX: done);
 *          AGC_STATUS_BAD_ARG on a failed assertion
 **********************************************************************/
static agc_status_t gamdv_secant(agc_interp_t *ip, rte_erasable_t *rs,
                                 rte_v2t_t *pl, agc_dp_t x_prev,
                                 agc_dp_t dv_prev)
{
    int n1 = 0; /* NORM shift count (X1) */
    int n2 = 0; /* NORM shift count (X2) */

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(pl != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t dverr = agc_dp_sub(ip, rs->rtedvd, rs->dv); /* DVERR, m/cs B7 */

    if (agc_dp_sub(ip, agc_dp_abs(dverr), EPC10RTE) < 0)
    {
        return (AGC_STATUS_NOT_FOUND); /* GAMDVX        */
    }

    ip->ovfind = 0; /* BOVB TCDANZIG */
    agc_dp_t d = /* dDV, B7 x 2^n2 */
                 agc_dp_norm(agc_dp_sub(ip, rs->dv, dv_prev), &n2);
    agc_dp_t e = agc_dp_shr(agc_dp_norm(dverr, &n1), 1); /* DVERR, B8 x 2^n1 */
    agc_dp_t q = agc_dp_div(ip, e, d); /* DDV: DVERR/dDV, B1 x 2^(n1-n2) */
    agc_tp_t t = /* dX q, TP B6 */
                 agc_dp_mul_tp(agc_dp_sub(ip, rs->x_t1, x_prev), q);

    pl->dx_t1 = agc_tp_to_dp(t);                            /* PRESERVE SIGN */
    t         = agc_tp_shift(ip, t, 1 + n2 - n1); /* XSU,1; SR* 0-1,1 */

    if (!agc_interp_test_ovf(ip))
    {
        pl->dx_t1 = agc_tp_to_dp(t);

        if (agc_dp_sub(ip, agc_dp_abs(pl->dx_t1), pl->dx_t1max) < 0)
        {
            return (AGC_STATUS_OK);
        }
    }

    pl->dx_t1 = agc_dp_sign(pl->dx_t1max, pl->dx_t1); /* GAMDV47       */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GAMDV50 .. GAMDV65 (p. 927): keep X(T1) + 1.1 DX(T1) inside the bounds
 * (halve the distance to the bound). NOT_FOUND = GAMDVX (|DX| < EPC9RTE).
 *
 * FUNCTION NAME: gamdv_bound
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *   pl        rte_v2t_t *       IO   P37 push-down list of the V2(T1) /
 *                                    TPMODE search
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK with DX(T1) kept in bounds;
 *          AGC_STATUS_NOT_FOUND when |DX(T1)| < EPC9RTE (GAMDVX: done);
 *          AGC_STATUS_BAD_ARG on a failed assertion
 **********************************************************************/
static agc_status_t gamdv_bound(agc_interp_t *ip, rte_erasable_t *rs,
                                rte_v2t_t *pl)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(pl != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_tp_t t = /* 1.1 DX(T1), B5 */
                 agc_tp_shift(ip, agc_dp_mul_tp(pl->dx_t1, K_1_1_B1), 1);
    agc_dp_t beta9 =
        agc_dp_add(ip, agc_tp_to_dp(t), rs->x_t1); /* BETA9 B5      */

    if (agc_dp_sub(ip, beta9, pl->x_t1max) >= 0)
    {
        pl->dx_t1 = agc_dp_shr(agc_dp_sub(ip, pl->x_t1max, rs->x_t1), 1);
    }
    else if (agc_dp_sub(ip, beta9, pl->x_t1min) < 0)
    { /* GAMDV60       */
        pl->dx_t1 = agc_dp_shr(agc_dp_sub(ip, pl->x_t1min, rs->x_t1), 1);
    }

    if (agc_dp_sub(ip, agc_dp_abs(pl->dx_t1), EPC9RTE) < 0)
    { /* GAMDV65       */
        return (AGC_STATUS_NOT_FOUND);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GAMDV10 .. GAMDVX (P37_P70.agc pp. 924-927): "COMPUTES A POST IMPULSE
 * VELOCITY VECTOR (V2(T1)) WHICH REQUIRES A MINIMUM DV" (F2RTE clear), or
 * which meets RTEDVD (F2RTE set). NN2 runs from -144: alarm 605 after 143
 * steps, which the AGC returns straight out of V2T100 (GOTO V2T1X).
 *
 * FUNCTION NAME: rte_min_dv_search
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *   pl        rte_v2t_t *       IO   P37 push-down list of the V2(T1) /
 *                                    TPMODE search
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK at GAMDVX; AGC_STATUS_ALARM with
 *          rs->alarm_code 605 after 143 steps; AGC_STATUS_BAD_ARG on a
 *          failed assertion; else the DVCALC status
 **********************************************************************/
static agc_status_t rte_min_dv_search(agc_interp_t *ip, rte_erasable_t *rs,
                                      rte_v2t_t *pl)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(pl != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = gamdv_start(ip, rs, pl); /* GAMDV10       */

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_NOT_FOUND) ? /* GAMDVX: OK */
                AGC_STATUS_OK : st);
    }

    for (rs->nn2 = 1 - RTE_GAMDV_PASSES; rs->nn2 < 0; rs->nn2++)
    {                                                   /* GAMDV25 */
        const agc_dp_t x_prev  = rs->x_t1; /* X(T1), B5 */
        const agc_dp_t dv_prev = rs->dv; /* DV, m/cs B7 */

        rs->x_t1 = agc_dp_add(ip, rs->x_t1, pl->dx_t1); /* GAMDV30       */
        st       = rte_dv_calc(ip, rs, pl);

        if ((st == AGC_STATUS_OK) && rs->f2rte)
        {
            st = gamdv_secant(ip, rs, pl, x_prev, dv_prev); /* GAMDV35       */
        }
        else if ((st == AGC_STATUS_OK) &&
                 (agc_dp_sub(ip, rs->dv, dv_prev) >= 0))
        {
            pl->dx_t1 =
                agc_dp_shr(agc_dp_neg(pl->dx_t1), 1); /* GAMDV32       */
        }

        if (st == AGC_STATUS_OK)
        {
            st = gamdv_bound(ip, rs, pl); /* GAMDV50       */
        }

        if (st != AGC_STATUS_OK)
        {
            return ((st == AGC_STATUS_NOT_FOUND) ? /* GAMDVX: OK */
                    AGC_STATUS_OK : st);
        }
    }

    rs->alarm_code = RTE_ALARM_ITERATIONS; /* OCT605        */

    return (AGC_STATUS_ALARM);
} /* rte_min_dv_search */

/**********************************************************************
 * V2T101 (pp. 918-919): LAMBDA, BETA1, BETA5 and the THETAs on the push
 * list. LAMBDA = R(T1)/RCON (normalised), BETA1 = 1 + X(T2)^2        B1
 * THETA1 = BETA5 LAMBDA - 1 (TP B17), THETA2 = 2 R(T1) (LAMBDA - 1) (TP
 * B38), THETA3 = MU^.5 / R(T1) (B-4).
 *
 * FUNCTION NAME: v2t_setup
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *   pl        rte_v2t_t *       IO   P37 push-down list of the V2(T1) /
 *                                    TPMODE search
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t v2t_setup(agc_interp_t *ip, rte_erasable_t *rs,
                              rte_v2t_t *pl)
{
    int n1 = 0; /* NORM shift count of RCON */
    int ns = 0; /* NORM shift count of R(T1) (S1) */
    int nm = 0; /* NORM shift count of RTMURTE */

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(pl != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rs->f2rte = false;                      /* CLEAR F2RTE   */
    agc_dp_t rcon_n = agc_dp_norm(rs->rcon, &n1); /* RCON, B29 x 2^n1 */
    agc_dp_t r1_n   = agc_dp_norm(rs->r_t1, &ns); /* R(T1), B29 x 2^ns */

    pl->lambda =
        agc_dp_div(ip, agc_dp_shr(r1_n, 1), rcon_n); /* SR1 DDV       */
    pl->x1    = ns - n1;                             /* XSU,1 S1      */
    rs->beta1 = agc_dp_add(ip, agc_dp_shr(agc_dp_sq(ip, rs->x_t2), 1), ONE_B1);
    pl->beta5 = agc_dp_mul_trunc(rs->beta1, pl->lambda); /* STORE 28D     */
    agc_tp_t t = /* BETA5 LAMBDA, TP B17 */
                 agc_dp_mul_tp(pl->beta5, pl->lambda);

    t          = agc_tp_shift(ip, t, -7 - pl->x1);        /* SL* 0-7,1     */
    t          = agc_tp_shift(ip, t, -7 - pl->x1);        /* SL* 0-7,1     */
    pl->theta1 = rte_tp_dsu(ip, t, ONE_B17);              /* DSU; TPMODE   */
    agc_dp_t a = /* LAMBDA - 1, B1 */
                 agc_dp_neg(agc_dp_shl(ip, ONE_B1, pl->x1));

    a          = agc_dp_add(ip, a, pl->lambda);              /* LAMBDA - 1    */
    pl->theta2 = agc_tp_shift(ip, agc_dp_mul_tp(a, rs->r_t1), -7 - pl->x1);
    agc_dp_t m = /* MU^.5 x 2^nm, B-1 */
                 agc_dp_shr(agc_dp_norm(RTMURTE, &nm), 1);
    agc_dp_t q = agc_dp_div(ip, m, r1_n); /* XSU,2 S1; DDV: MU^.5/R(T1), B0 */

    pl->theta3 = agc_dp_shl(ip, q, -(6 - (ns - nm)));      /* SR* 6,2 B-4   */
    agc_dp_t     lim = 0; /* X(T1)LIM from MAMAX1, B5 */
    agc_status_t st  =
        rte_xt1lim(ip, rs, rs->mamax1, pl, &lim); /* MAMAX1 = MA   */

    pl->x_t1min  = agc_dp_neg(lim);               /* X(T1)MIN      */
    pl->dx_t1max = agc_dp_shr(lim, 4);            /* DX(T1)MAX     */

    if (st == AGC_STATUS_OK)
    {
        st = rte_xt1lim(ip, rs, rs->mamax2, pl,
                        &pl->x_t1max); /* X(T1)MAX      */
    }

    return (st);
} /* v2t_setup */

/**********************************************************************
 * V2T110 .. V2T135 (pp. 920-921): during a precision iteration constrain
 * X(T1) "TO INSURE THAT ALL CONICS PASS THROUGH RCON ON THE SAME PASS
 * THROUGH X(T2)": BETA6 = BETA5 (2 - LAMBDA) - 1, X(T1)LIM = BETA6^.5.
 * *start_high: begin at X(T1)MAX (V2T145) rather than X(T1)MIN (V2T140).
 *
 * FUNCTION NAME: v2t_precision_bounds
 *
 * ARGUMENTS:
 *   Argument    Type              I/O  Description (scale)
 *   ----------  ----------------  ---  ----------------------------------
 *   ip          agc_interp_t *    IO   interpreter state of the job:
 *                                      OVFIND, DVSIGN, alarms
 *   rs          rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                      internal.h)
 *   pl          rte_v2t_t *       IO   P37 push-down list of the V2(T1) /
 *                                      TPMODE search
 *   start_high  bool *            O    true: start the search at X(T1)MAX
 *                                      (V2T145), else at X(T1)MIN (V2T140)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t v2t_precision_bounds(agc_interp_t *ip, rte_erasable_t *rs,
                                         rte_v2t_t *pl, bool *start_high)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) ||
        !AGC_ASSERT((pl != NULL) && (start_high != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t a = agc_dp_sub(ip, agc_dp_shl(ip, TWO_B1, pl->x1),
                            pl->lambda); /* SR* 0,1 DSU 00D: 2 - LAMBDA, B1 */
    agc_tp_t t = agc_dp_mul_tp(a, pl->beta5); /* DMP 28D: BETA6, TP B17 */

    t          = agc_tp_shift(ip, t, -7 - pl->x1);
    t          = agc_tp_shift(ip, t, -7 - pl->x1);
    t          = agc_tp_add(ip, t, agc_tp_from_dp(-ONE_B17)); /* TAD: BETA6 B17*/
    pl->x_t1pp = rs->x_t1;                           /* X(T1),, 00D   */

    if (t <= 0)
    { /* V2T115        */
        if (rs->phi2 >= 0)
        {
            rs->phi2 = agc_dp_neg(rs->phi2);
            rs->nn1a = RTE_PREC_OUTER; /* 10RTE         */
        }

        *start_high = (rs->x_t1 > 0); /* V2T125        */

        return (AGC_STATUS_OK);
    }

    agc_dp_t lim = agc_tp_sqrt(ip, agc_tp_shift(ip, t, 7)); /* SL 7 SQRT B5  */

    if (rs->phi2 >= 0)
    {
        pl->x_t1max = lim; /* X(T1)LIM=MAX  */
        pl->x_t1min = agc_dp_neg(lim);
        *start_high = (rs->x_t1 > 0); /* V2T125        */
    }
    else if (rs->x_t1 > 0)
    { /* V2T130        */
        pl->x_t1min = lim;
        *start_high = true;
    }
    else
    {
        pl->x_t1max = agc_dp_neg(lim); /* V2T135        */
        *start_high = false;
    }

    return (AGC_STATUS_OK);
} /* v2t_precision_bounds */

/**********************************************************************
 * V2T185 .. V2T205 (pp. 922-923): "PREVENT A LARGE CHANGE IN INDEPENDENT
 * VARIABLE DURING AN ITERATION FOR A PRECISION TRAJECTORY".
 *
 * FUNCTION NAME: v2t_limit_change
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     IO   interpreter state of the job: OVFIND,
 *                                     DVSIGN, alarms
 *   rs        rte_erasable_t *   IO   P37 erasables (RTE names, scales in
 *                                     internal.h)
 *   pl        const rte_v2t_t *  I    P37 push-down list of the V2(T1) /
 *                                     TPMODE search
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t v2t_limit_change(agc_interp_t *ip, rte_erasable_t *rs,
                                     const rte_v2t_t *pl)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(pl != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t beta7 = /* BETA7 = |X(T1) - X(T1)|, B5 */
                     agc_dp_abs(agc_dp_sub(ip, rs->x_t1, pl->x_t1pp));

    if (agc_dp_sub(ip, beta7, agc_dp_shl(ip, pl->dx_t1max, 1)) < 0)
    {
        return (AGC_STATUS_OK); /* V2T300        */
    }

    rs->x_t1 = pl->x_t1pp; /* X(T1) = X,,   */

    if (agc_dp_sub(ip, rs->x_t1, pl->x_t1max) >= 0)
    {
        rs->x_t1 = pl->x_t1max;
    }
    else if (agc_dp_sub(ip, rs->x_t1, pl->x_t1min) < 0)
    {                           /* V2T195        */
        rs->x_t1 = pl->x_t1min; /* V2T200        */
    }

    return (rte_dv_calc(ip, rs, pl)); /* V2T205        */
}

/**********************************************************************
 * V2T150 .. V2T300 (pp. 921-923) after the first iterator pass, when a DV
 * is desired (RTEDVD non-zero).
 *
 * FUNCTION NAME: v2t_desired_dv
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *   pl        rte_v2t_t *       IO   P37 push-down list of the V2(T1) /
 *                                    TPMODE search
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t v2t_desired_dv(agc_interp_t *ip, rte_erasable_t *rs,
                                   rte_v2t_t *pl)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(pl != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_OK; /* iterator status */

    if (agc_dp_sub(ip, rs->rtedvd, rs->dv) < 0)
    { /* V2T155        */
        if (rs->nn1a >= 0)
        {
            return (v2t_limit_change(ip, rs, pl)); /* V2T185        */
        }

        if ((rs->x_t1 > 0) || (rs->cfpa <= 0))
        {                         /* V2T160/165    */
            return (AGC_STATUS_OK); /* V2T300        */
        }

        rs->x_t1  = pl->x_t1max;
        pl->dx_t1 = agc_dp_neg(pl->dx_t1max);
        st        = rte_min_dv_search(ip, rs, pl);

        if ((st != AGC_STATUS_OK) || (agc_dp_sub(ip, rs->rtedvd, rs->dv) < 0))
        {
            return (st);
        }
    }

    /* V2T175 .. V2T179 (p. 922): time critical: F2RTE set; X(T1) becomes the
     * lower (SLOWFLG) or upper bound and the iterator steps away from it.     */
    rs->f2rte = true;

    if (agc_flag_test(AGC_FLAG_SLOWFLG))
    {
        pl->x_t1min = rs->x_t1;
        pl->dx_t1   = pl->dx_t1max;
    }
    else
    { /* V2T177        */
        pl->x_t1max = rs->x_t1;
        pl->dx_t1   = agc_dp_neg(pl->dx_t1max);
    }

    st = rte_min_dv_search(ip, rs, pl); /* V2T175        */

    if ((st != AGC_STATUS_OK) || (rs->nn1a < 0))
    {
        return (st);
    }

    return (v2t_limit_change(ip, rs, pl)); /* V2T185        */
} /* v2t_desired_dv */

/**********************************************************************
 * V2T100 (P37_P70.agc pp. 917-923). NN1A negative: conic; otherwise a
 * precision iteration. Returns OK (MPAC = 0), ALARM with alarm_code 605
 * (excess iterations), POODOO 00610 when RCON is negative or not below
 * R(T1) ("NO SOLUTION IF LAMBDA LESS THAN 1").
 *
 * FUNCTION NAME: rte_post_impulse_velocity
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (MPAC = 0); AGC_STATUS_ALARM with
 *          rs->alarm_code 605 (excess iterations); POODOO 00610 (the
 *          agc_poodoo status) when RCON is negative or not below R(T1);
 *          AGC_STATUS_BAD_ARG on a failed assertion
 **********************************************************************/
agc_status_t rte_post_impulse_velocity(agc_interp_t *ip, rte_erasable_t *rs)
{
    rte_v2t_t pl         = {0};
    bool      start_high = false; /* start at X(T1)MAX */

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((rs->rcon < 0) || (agc_dp_sub(ip, rs->rcon, rs->r_t1) >= 0))
    {
        rs->alarm_code = RTE_POODOO_NO_SOLUTION; /* V2TERROR      */

        return (agc_poodoo(RTE_POODOO_NO_SOLUTION));
    }

    agc_status_t st = v2t_setup(ip, rs, &pl); /* V2T100 status */

    if ((st == AGC_STATUS_OK) && (rs->nn1a >= 0))
    {
        st = v2t_precision_bounds(ip, rs, &pl, &start_high); /* V2T110        */
    }
    else
    {
        start_high = (rs->rtedvd == 0) && (rs->cfpa >= 0); /* V2T102/105    */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rs->x_t1 = start_high ? pl.x_t1max : pl.x_t1min; /* V2T145/140    */
    pl.dx_t1 = start_high ? /* down from MAX */
               agc_dp_neg(pl.dx_t1max) : pl.dx_t1max;
    st = rte_min_dv_search(ip, rs, &pl); /* V2T150        */

    if ((st != AGC_STATUS_OK) || (rs->rtedvd == 0))
    {
        return (st); /* minimum fuel  */
    }

    return (v2t_desired_dv(ip, rs, &pl));
} /* rte_post_impulse_velocity */
