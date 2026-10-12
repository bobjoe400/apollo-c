/**********************************************************************
 * FILE NAME: precision.c
 *
 * PURPOSE:
 *   P37 precision return-to-earth trajectory: PREC100 .. PRECX and the
 *   integration calls RTENCK1/2/3. "A NUMERICALLY INTEGRATED TRAJECTORY
 *   IS GENERATED WHICH FOR THE RETURN TO EARTH PROBLEM SATISFIES THE
 *   REENTRY CONSTRAINTS (RCON AND X(T2)) ACHIEVED BY THE INITIAL CONIC
 *   TRAJECTORY AND MEETS THE DVD REQUIREMENT AS CLOSELY AS POSSIBLE."
 *
 * AGC SOURCE:
 *   Comanche055 P37_P70.agc pp. 907-916   (PREC100 .. PRECX, RTENCK1-3)
 *   Luminary099: P37 not in the LM rope
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type            I/O  Scale          Description
 *   ------------  --------------  ---  -------------  ------------------
 *   integration   agc_integ_ctx_t IO   per member     INTSTALL area
 *                                                     (agc_integ_shared_ctx)
 *   INTYPFLG, MOONFLAG, RVSW flags  O  -              integration options
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_integ_install     INTSTALL (may put the job to sleep: BUSY)
 *   agc_integ_integrvs    INTEGRVS, conic or precision
 *   agc_conic_param       PARAM: P, R/A, R, COGA of a state
 *   agc_conic_timerad     TIMERAD: time to a radius on the conic
 *   rte_post_impulse_velocity  V2T100 (velocity.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   AGC_STATUS_ALARM with rs->alarm_code 605 (iterations exhausted,
 *   PREC132) or 613 (reentry angle not met, PREC220); BUSY while asleep in
 *   INTSTALL; POODOO statuses from TIMERAD and V2T100 are passed up.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the P37 job (CSM only). Outer loop NN1A: 10 passes (RCON
 *   secant); inner loop NN2: 15 time steps (TIMERAD on the osculating
 *   conic to the radius where the desired X(T2) is reached).
 *
 * NOTES:
 *   The AGC job sleeps inside INTSTALL when the integration erasables are
 *   in use. In C every integration is a resumption point: rte_precision_run
 *   returns BUSY and the resume job calls it again; rs->prec_step says
 *   which integration (RTENCK3 from TIG, or RTENCK2 by DT21) to redo, and
 *   all other iteration state is in the module erasables, as in the AGC.
 *   The PREC155 / PREC205 arithmetic keeps the TP MPAC (finding F14,
 *   docs/modules/review-guid-entry.md).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/nav/conic.h"
#include "agc/nav/integration.h"
#include "internal.h"

/* Constants (P37_P70.agc pp. 912-913, 931-932, words from the rope).       */
static const agc_dp_t ONE_B1       = 134217728;  /* 1RTEB1   20000 00000  1.0 B-1          */
static const agc_dp_t ONE_B3       = 33554432;   /* 1RTEB3   04000 00000  1.0 B-3          */
static const agc_dp_t ONE_B4       = 16777216;   /* 1RTEB4   02000 00000  1.0 B-4          */
static const agc_dp_t TWO_B1       = 268435455;  /* 2RTEB1 37777 37777 POSMAX, B1 */
static const agc_dp_t M_POINT6     = -161061274; /* M.6RTE 54631 63145 -.6, B0 */
static const agc_dp_t EPC4RTE      = 2684;       /* EPC4RTE 00000 05174 .00001, B0 */
static const agc_dp_t EPC6RTE      = 940;        /* EPC6RTE  00000 01654  .000007 B-1      */
static const agc_dp_t EPC7RTE      = 500;        /* EPC7RTE  00000 00764  1000. B-29 m     */
static const agc_dp_t EPC8RTE      = 536871;     /* EPC8RTE 00040 30447 .002, B0 */
static const agc_sp_t SGNRDOT_HALF = 0x2000;     /* .5, B0 (SP): sign of radial speed */

/**********************************************************************
 * RTENCK1 / RTENCK2 / RTENCK3 .. RTENCK3D (P37_P70.agc pp. 914-916):
 * INTSTALL; RCV, VCV, TET = r, v, tet; TDEC1; INTYPFLG (set = conic); CLEAR
 * MOONFLAG; INTEGRVS. Out: R(T2)/ = RATT, T2 = TAT, V(T2)/ = VATT (X1 =
 * -2). The conic case also leaves T (30D) = the KEPLER time (TC). BUSY:
 * INTSTALL put the job to sleep; it resumes as `resume`.
 *
 * FUNCTION NAME: rte_integrate
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *   r         agc_vec_t         I    start position R, m B29 (earth)
 *   v         agc_vec_t         I    start velocity V, m/cs B7
 *   tet       agc_dp_t          I    time of r and v, cs B28
 *   tdec1     agc_dp_t          I    time to integrate to (TDEC1), cs B28
 *   conic     bool              I    true: conic only (INTYPFLG set,
 *                                    RTENCK1); false: precision
 *   resume    agc_job_id_t      I    job that INTSTALL wakes when the
 *                                    integration erasables free
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK with R(T2)/, T2, V(T2)/ (and the
 *          KEPLER time for a conic) in rs; AGC_STATUS_BUSY when INTSTALL
 *          put the job to sleep (it resumes as resume); AGC_STATUS_BAD_ARG
 *          on a failed assertion; else the status of INTEGRVS
 **********************************************************************/
agc_status_t rte_integrate(agc_interp_t *ip, rte_erasable_t *rs, agc_vec_t r, agc_vec_t v, agc_dp_t tet, agc_dp_t tdec1, bool conic, agc_job_id_t resume)
{
    agc_integ_ctx_t *ic = agc_integ_shared_ctx(); /* INTSTALL work area */

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT((ic != NULL) && (resume != AGC_JOB_NONE)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(tdec1 != tet)) /* a real step   */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_integ_install(0, resume); /* INTSTALL      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (conic)
    {
        agc_flag_set(AGC_FLAG_INTYPFLG); /* RTENCK1       */
    }
    else
    {
        agc_flag_clear(AGC_FLAG_INTYPFLG); /* RTENCK2/3     */
    }

    ic->sv.rcv = r;
    ic->sv.vcv = v;
    ic->sv.tet = tet;
    agc_flag_clear(AGC_FLAG_MOONFLAG); /* RTENCK3D      */
    st         = agc_integ_integrvs(ip, ic, tdec1);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rs->r_t2v = ic->out.ratt; /* R(T2)/        */
    rs->t2    = ic->out.tat;  /* T2            */
    rs->v_t2v = ic->out.vatt; /* V(T2)/        */

    if (conic)
    {
        rs->conic_t = ic->sv.tc; /* T (30D)       */
    }

    return (AGC_STATUS_OK);
} /* rte_integrate */

/**********************************************************************
 * PREC120 (p. 908): DT21PR = POSMAX, NN2 = -15, next integration RTENCK3.
 *
 * FUNCTION NAME: prec_restart_inner
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *
 * RETURNS: none
 **********************************************************************/
static void prec_restart_inner(rte_erasable_t *rs)
{
    rs->dt21pr    = TWO_B1;
    rs->nn2       = -RTE_PREC_INNER; /* M15RTE        */
    rs->prec_step = RTE_PREC_FROM_TIG;
}

/**********************************************************************
 * PREC100 (p. 908): NN1A = 10, RD = RCON, then PREC120.
 *
 * FUNCTION NAME: rte_precision_start
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t rte_precision_start(rte_erasable_t *rs)
{
    if (!AGC_ASSERT(rs != NULL) || !AGC_ASSERT(rs->rcon > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rs->nn1a       = RTE_PREC_OUTER; /* 10RTE         */
    rs->rd         = rs->rcon;
    rs->prec_steps = 0;
    prec_restart_inner(rs);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * PREC140 .. PREC162 (p. 909): BETA4 (B1), the ratio of the new radius to
 * RPRE at which the desired reentry angle is reached: BETA2 = BETA1 P/R,
 * BETA3 = 1 - BETA2 R/A, BETA4 = BETA2 / (1 - PHI2 SQRT(BETA3))   (BETA3 >=
 * 0) = A/R                               (BETA3 < 0, PREC160) = RD/RPRE
 * (last pass, NN1A = 0)
 *
 * FUNCTION NAME: prec_radius_ratio
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------------
 *   ip        agc_interp_t *          IO   interpreter state of the job:
 *                                          OVFIND, DVSIGN, alarms
 *   rs        const rte_erasable_t *  I    P37 erasables (RTE names, scales
 *                                          in internal.h)
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- BETA4, the new radius over RPRE, B1
 **********************************************************************/
static agc_dp_t prec_radius_ratio(agc_interp_t *ip, const rte_erasable_t *rs)
{
    int n1 = 0; /* NORM shift count (X1) */

    if (rs->nn1a == 0)
    { /* PREC162       */
        agc_dp_t rn = agc_dp_norm(rs->rpre, &n1); /* x 2^n1, scale as input */

        return (agc_dp_shl(ip, agc_dp_div(ip, rs->rd, rn), n1 - 1));
    }

    agc_tp_t t     = agc_dp_mul_tp(agc_dp_shl(ip, rs->p_rpre, 2), rs->beta1); /* TP MPAC, B2 then B4 */
    agc_dp_t beta2 = agc_tp_to_dp(agc_tp_shift(ip, t, 1));                    /* BETA2 B2      */

    t              = agc_dp_mul_tp(agc_dp_shl(ip, rs->r_apre, 4), beta2); /* SL4 DMP 00D   */
    agc_tp_t beta3 = rte_tp_bdsu(ip, ONE_B4, t);                          /* BETA3 B4      */

    if (agc_tp_to_dp(beta3) < 0)
    { /* PREC160       */
        agc_dp_t rn = agc_dp_norm(rs->r_apre, &n1); /* x 2^n1, scale as input */

        return (agc_dp_shl(ip, agc_dp_div(ip, ONE_B1, rn), n1 - 6));
    }

    /* PREC155: SL2 SQRT DMP BDSU NORM stay on the TP MPAC; the NORM
     * brings MPAC+2 into the pushed divisor (1 - PHI2 SQRT(BETA3) can be
     * a small difference).                                               */
    agc_dp_t s   = agc_tp_sqrt(agc_tp_shift(ip, beta3, 2));                      /* B1     */
    agc_tp_t dtp = agc_dp_mul_tp(s, rs->phi2);                                   /* PHI2 SQRT(BETA3), B3 */
    agc_dp_t den = agc_tp_to_dp(agc_tp_norm(rte_tp_bdsu(ip, ONE_B3, dtp), &n1)); /* 1 - PHI2 SQRT(BETA3), B3 x 2^n1 */

    return (agc_dp_shl(ip, agc_dp_div(ip, agc_dp_shr(beta2, 1), den), n1 - 1));
} /* prec_radius_ratio */

/**********************************************************************
 * PREC130 .. PREC168 (pp. 909-910): the desired reentry angle was not
 * reached; NOT_FOUND = close enough after all (go to PREC175).
 *
 * FUNCTION NAME: prec_angle_not_met
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK after a new time step (DT21 set);
 *          AGC_STATUS_NOT_FOUND when BETA12 is within EPC6RTE (go on to
 *          PREC175); AGC_STATUS_ALARM with rs->alarm_code 605 when NN2 ran
 *          out; AGC_STATUS_BAD_ARG on a failed assertion
 **********************************************************************/
static agc_status_t prec_angle_not_met(agc_interp_t *ip, rte_erasable_t *rs)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->rpre > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (rs->nn2 >= 0)
    { /* PREC132       */
        rs->alarm_code = RTE_ALARM_ITERATIONS;

        return (AGC_STATUS_ALARM);
    }

    agc_dp_t beta4 = prec_radius_ratio(ip, rs); /* PREC140 BETA4, B1 */

    rs->beta12 = agc_dp_neg(agc_dp_sub(ip, beta4, ONE_B1)); /* PREC165       */

    if ((rs->beta12 >= 0) && (rs->x_t2pre >= 0))
    {
        rs->beta12 = agc_dp_neg(rs->beta12); /* PREC167       */
    }

    if (agc_dp_sub(ip, agc_dp_abs(rs->beta12), EPC6RTE) < 0)
    { /* PREC168       */
        return (AGC_STATUS_NOT_FOUND);
    }

    agc_tp_t rf = agc_tp_shift(ip, agc_dp_mul_tp(beta4, rs->rpre), 1); /* RF B29 */

    /* PREC170 .. PREC174 (pp. 910-911): time DT21 from R(T2)/ to the new radius
     * RF on the osculating conic (TIMERAD, time only, backwards when BETA12 <
     * 0), limited to DT21PR (forward) or -.6 DT21PR (reversal). */
    agc_dp_t rf_step = agc_tp_to_dp(rf);

    agc_timerad_in_t     in;     /* TIMERAD inputs (R, V, RF, SGNRDOT) */
    agc_conic_time_out_t out;    /* TIMERAD outputs (T, cs B28) */
    int                  n1 = 0; /* NORM shift count of DT21PR */

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rf_step > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rs->nn2         += 1;                                                         /* PREC170       */
    agc_flag_set(AGC_FLAG_RVSW);
    in.rvec         = rs->r_t2v;
    in.vvec         = (rs->beta12 < 0) ? agc_vec_neg(rs->v_t2v) : rs->v_t2v;      /* SIGN      */
    in.rdesired     = rf_step;
    in.sgnrdot      = (rs->beta12 < 0) ? SGNRDOT_HALF : (agc_sp_t) -SGNRDOT_HALF; /* backwards: +.5 */
    in.rvsw         = true;
    in.body         = AGC_CONIC_EARTH;
    agc_status_t st = agc_conic_timerad(ip, &in, &out);                           /* TIMERAD status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rs->conic_t = out.t;

    if (!AGC_ASSERT(rs->dt21pr != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t dt21   = agc_dp_sign(out.t, rs->beta12);                   /* PHI4 DT21, cs B28 */
    agc_dp_t rn     = agc_dp_norm(rs->dt21pr, &n1);                     /* DT21PR x 2^n1, cs B28 */
    agc_dp_t beta13 = agc_dp_shl(ip, agc_dp_div(ip, dt21, rn), n1 - 3); /* B3   */
    agc_dp_t beta14 = (beta13 < 0) ? M_POINT6 : TWO_B1;                 /* PREC172/3, B1 */

    if (agc_dp_sub(ip, agc_dp_div(ip, beta13, beta14), ONE_B3) >= 0)
    {
        dt21 = agc_dp_mul_trunc(beta14, rs->dt21pr); /* BETA14 DT21PR */
    }

    rs->dt21      = dt21;               /* PREC174       */
    rs->dt21pr    = dt21;
    rs->prec_step = RTE_PREC_TIME_STEP; /* RTENCK2       */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * PREC205 .. PREC207 (pp. 911-912): new RCON. First pass RD^2/RPRE; then
 * the secant S = DRCON/(RPRE' - RPRE), kept in (-4, 0) (else -4), and DRCON
 * = S RERR.
 *
 * FUNCTION NAME: prec_new_rcon
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *   rerr      agc_dp_t          I    radius error RERR = RPRE - RD, m B29
 *
 * RETURNS: none
 **********************************************************************/
static void prec_new_rcon(agc_interp_t *ip, rte_erasable_t *rs, agc_dp_t rerr)
{
    int n1 = 0; /* NORM shift count of RD^2 */
    int n2 = 0; /* NORM shift count of the divisor */

    if (rs->nn1a == RTE_PREC_OUTER)
    { /* PREC207       */
        agc_dp_t a  = agc_tp_to_dp(agc_tp_norm(agc_dp_sq_tp(rs->rd), &n1)); /* RD^2 x 2^n1, m^2 B58 */
        agc_dp_t rn = agc_dp_norm(rs->rpre, &n2);                           /* RPRE x 2^n2, m B29 */
        agc_dp_t q  = agc_dp_div(ip, agc_dp_shr(a, 1), rn);                 /* BDDV, B0 */

        rs->rcon  = agc_dp_shl(ip, q, 1 - n1 + n2); /* SR* 0-1,1     */
        rs->drcon = agc_dp_sub(ip, rs->rcon, rs->rd);
    }
    else
    {
        agc_dp_t d   = agc_dp_norm(agc_dp_sub(ip, rs->rpre_prev, rs->rpre), &n2); /* dRPRE x 2^n2, m B29 */
        agc_dp_t s   = agc_dp_shl(ip, agc_dp_div(ip, rs->drcon, d), n2 - 2);      /* S B2  */
        agc_dp_t sp2 = agc_dp_add(ip, s, ONE_B1);                                 /* S + 1, B2 */

        if (agc_interp_test_ovf(ip) || (agc_dp_sub(ip, agc_dp_abs(sp2), ONE_B1) >= 0))
        {
            s = agc_dp_neg(TWO_B1); /* PREC205M: -4  */
        }

        agc_tp_t t = agc_tp_shift(ip, agc_dp_mul_tp(s, rerr), 2); /* DMP SL2: S RERR, m B29 */

        rs->drcon = agc_tp_to_dp(t); /* PREC206       */
        rs->rcon  = agc_dp_add(ip, rs->drcon, rs->rcon);
    }
} /* prec_new_rcon */

/**********************************************************************
 * PREC175 .. PREC225 (pp. 911-913): radius check; when not met, a new RCON
 * and V2(T1)/ (PREC210: NN1A - 1, V2T100) and the next outer pass; when
 * met, the reentry angle check (613). NOT_FOUND = PRECX with 0.
 *
 * FUNCTION NAME: prec_radius_check
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK after a new RCON and V2(T1)/ (next
 *          outer pass); AGC_STATUS_NOT_FOUND when radius and X(T2) are met
 *          (PREC225, done); AGC_STATUS_ALARM with rs->alarm_code 605 or
 *          613; AGC_STATUS_BAD_ARG on a failed assertion; else the status
 *          of V2T100
 **********************************************************************/
static agc_status_t prec_radius_check(agc_interp_t *ip, rte_erasable_t *rs)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->rd > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t rerr = agc_dp_sub(ip, rs->rpre, rs->rd); /* RERR, m B29 */

    if (agc_dp_sub(ip, agc_dp_abs(rerr), EPC7RTE) < 0)
    { /* PREC220       */
        agc_dp_t e = agc_dp_abs(agc_dp_sub(ip, rs->x_t2, rs->x_t2pre)); /* |dX(T2)|, B0 */

        if (agc_dp_sub(ip, e, EPC8RTE) < 0)
        {
            return (AGC_STATUS_NOT_FOUND); /* PREC225       */
        }

        rs->alarm_code = RTE_ALARM_ENTRY_ANGLE; /* OCT613        */

        return (AGC_STATUS_ALARM);
    }

    if (rs->nn1a == 0)
    { /* PREC132       */
        rs->alarm_code = RTE_ALARM_ITERATIONS;

        return (AGC_STATUS_ALARM);
    }

    if (!AGC_ASSERT(rs->nn1a <= RTE_PREC_OUTER))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    prec_new_rcon(ip, rs, rerr);
    rs->rpre_prev   = rs->rpre;                          /* PREC210       */
    rs->nn1a        -= 1;
    agc_status_t st = rte_post_impulse_velocity(ip, rs); /* V2T100        */

    if (st == AGC_STATUS_OK)
    {
        prec_restart_inner(rs); /* BHIZ PREC120  */
    }

    return (st);
} /* prec_radius_check */

/**********************************************************************
 * PREC125 (p. 908) after each integration: PARAM on R(T2)/, V(T2)/ ->
 * P/RPRE, R/APRE, RPRE, X(T2)PRE = COGA SL5; compare with X(T2).
 *
 * FUNCTION NAME: prec_after_integration
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK to integrate again
 *          (rs->prec_step says which); AGC_STATUS_NOT_FOUND when the
 *          iteration is done; AGC_STATUS_ALARM (605, 613) or the status
 *          of PARAM / TIMERAD / V2T100 otherwise; AGC_STATUS_BAD_ARG on a
 *          failed precondition
 **********************************************************************/
static agc_status_t prec_after_integration(agc_interp_t *ip, rte_erasable_t *rs)
{
    agc_conic_param_out_t po; /* PARAM of R(T2)/, V(T2)/ */

    agc_conic_param(ip, rs->r_t2v, rs->v_t2v, AGC_CONIC_EARTH, &po);

    if (!AGC_ASSERT(po.r1 > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rs->p_rpre  = po.p;
    rs->r_apre  = po.r1a;
    rs->rpre    = po.r1;
    rs->x_t2pre = agc_dp_shl(ip, po.coga, 5);                                    /* SL 5 -> B0    */
    agc_dp_t e  = agc_dp_abs(agc_dp_add(ip, agc_dp_neg(rs->x_t2pre), rs->x_t2)); /* |dX(T2)| - EPC4, B0 */

    e = agc_dp_sub(ip, e, EPC4RTE);

    if (agc_interp_test_ovf(ip) || (e >= 0))
    { /* PREC130       */
        agc_status_t st = prec_angle_not_met(ip, rs); /* PREC130 status */

        if (st != AGC_STATUS_NOT_FOUND)
        {
            return (st);
        }
    }

    return (prec_radius_check(ip, rs)); /* PREC175       */
} /* prec_after_integration */

/**********************************************************************
 * PREC120 .. PRECX (pp. 908-913): run the precision iteration from the
 * current rs->prec_step. OK: done (PRECX with 0, PREC225); ALARM 605/613
 * (rs->alarm_code); BUSY: asleep in INTSTALL, call again as `resume`;
 * POODOO from V2T100/TIMERAD.
 *
 * FUNCTION NAME: rte_precision_run
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   ip        agc_interp_t *    IO   interpreter state of the job: OVFIND,
 *                                    DVSIGN, alarms
 *   rs        rte_erasable_t *  IO   P37 erasables (RTE names, scales in
 *                                    internal.h)
 *   resume    agc_job_id_t      I    job INTSTALL wakes after a sleep
 *                                    (calls this again)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when done (PRECX with 0, PREC225);
 *          AGC_STATUS_BUSY when asleep in INTSTALL; AGC_STATUS_ALARM with
 *          rs->alarm_code 605 or 613; AGC_STATUS_BAD_ARG on a failed
 *          assertion; else POODOO statuses from V2T100 / TIMERAD
 **********************************************************************/
agc_status_t rte_precision_run(agc_interp_t *ip, rte_erasable_t *rs, agc_job_id_t resume)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    while (rs->prec_steps < RTE_PREC_MAX_STEPS)
    {
        agc_status_t st; /* integration, then PREC125 */

        if (rs->prec_step == RTE_PREC_FROM_TIG)
        { /* RTENCK3       */
            st = rte_integrate(ip, rs, rs->r_t1v, rs->v2_t1v, rs->t1, rs->t2, false, resume);
        }
        else
        { /* RTENCK2       */
            agc_dp_t tdec1 = agc_dp_add(ip, rs->t2, rs->dt21); /* T2 + DT21, cs B28 */

            st = rte_integrate(ip, rs, rs->r_t2v, rs->v_t2v, rs->t2, tdec1, false, resume);
        }

        if (st != AGC_STATUS_OK)
        {
            return (st); /* BUSY: sleep   */
        }

        rs->prec_steps += 1;
        st             = prec_after_integration(ip, rs); /* PREC125       */

        if (st != AGC_STATUS_OK)
        {
            return ((st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st); /* done: OK */
        }
    }

    rs->alarm_code = RTE_ALARM_ITERATIONS; /* cannot happen: NN1A, NN2 bound it */

    return (AGC_STATUS_ALARM);
} /* rte_precision_run */
