/*
 * precision.c -- P37_P70.agc (Comanche055, pp. 907-916): the
 * precision return-to-earth trajectory.
 *
 *   PREC100 .. PRECX (pp. 907-913) "A NUMERICALLY INTEGRATED TRAJECTORY IS
 *       GENERATED WHICH FOR THE RETURN TO EARTH PROBLEM SATISFIES THE
 *       REENTRY CONSTRAINTS (RCON AND X(T2)) ACHIEVED BY THE INITIAL CONIC
 *       TRAJECTORY AND MEETS THE DVD REQUIREMENT AS CLOSELY AS POSSIBLE."
 *       Outer loop (NN1A, 10 passes): RCON is corrected by a secant on the
 *       radius error and V2T100 recomputes V2(T1)/. Inner loop (NN2, 15
 *       passes): the integrated trajectory is stepped in time (TIMERAD on its
 *       osculating conic) to the radius where the desired X(T2) is reached.
 *   RTENCK1/2/3 (pp. 914-916) INTSTALL + INTEGRVS (conic: INTYPFLG set).
 *
 * The AGC job sleeps inside INTSTALL when the integration erasables are in
 * use. In C every integration is a resumption point: rte_precision_run
 * returns BUSY and the resume job calls it again; rs->prec_step says which
 * integration (RTENCK3 from TIG, or RTENCK2 by DT21) to redo, and all other
 * iteration state is in the module erasables, as in the AGC.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/nav/conic.h"
#include "agc/nav/integration.h"
#include "internal.h"

/* Constants (P37_P70.agc pp. 912-913, 931-932, words from the rope).       */
static const agc_dp_t ONE_B1 =
    134217728; /* 1RTEB1   20000 00000  1.0 B-1          */
static const agc_dp_t ONE_B3 =
    33554432; /* 1RTEB3   04000 00000  1.0 B-3          */
static const agc_dp_t ONE_B4 =
    16777216; /* 1RTEB4   02000 00000  1.0 B-4          */
static const agc_dp_t TWO_B1 =
    268435455; /* 2RTEB1   37777 37777  POSMAX           */
static const agc_dp_t M_POINT6 =
    -161061274; /* M.6RTE   54631 63145  -.6              */
static const agc_dp_t EPC4RTE =
    2684; /* EPC4RTE  00000 05174  .00001 (X(T2))   */
static const agc_dp_t EPC6RTE =
    940; /* EPC6RTE  00000 01654  .000007 B-1      */
static const agc_dp_t EPC7RTE =
    500; /* EPC7RTE  00000 00764  1000. B-29 m     */
static const agc_dp_t EPC8RTE =
    536871; /* EPC8RTE  00040 30447  .002 (X(T2))     */
static const agc_sp_t SGNRDOT_HALF =
    0x2000; /* .5 (SP), sign of radial velocity       */

/* RTENCK1 / RTENCK2 / RTENCK3 .. RTENCK3D (P37_P70.agc pp. 914-916):
 * INTSTALL; RCV, VCV, TET = r, v, tet; TDEC1; INTYPFLG (set = conic);
 * CLEAR MOONFLAG; INTEGRVS. Out: R(T2)/ = RATT, T2 = TAT, V(T2)/ = VATT
 * (X1 = -2). The conic case also leaves T (30D) = the KEPLER time (TC).
 * BUSY: INTSTALL put the job to sleep; it resumes as `resume`.          */
agc_status_t rte_integrate(agc_interp_t *ip, rte_erasable_t *rs, agc_vec_t r,
                           agc_vec_t v, agc_dp_t tet, agc_dp_t tdec1,
                           bool conic, agc_job_id_t resume)
{
    agc_integ_ctx_t *ic = agc_integ_shared_ctx();

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) ||
        !AGC_ASSERT((ic != NULL) && (resume != AGC_JOB_NONE)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    (void) AGC_ASSERT(tdec1 != tet);                /* a real step   */
    agc_status_t st = agc_integ_install(0, resume); /* INTSTALL      */
    if (st != AGC_STATUS_OK)
    {
        return st;
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
    st = agc_integ_integrvs(ip, ic, tdec1);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rs->r_t2v = ic->out.ratt; /* R(T2)/        */
    rs->t2    = ic->out.tat;  /* T2            */
    rs->v_t2v = ic->out.vatt; /* V(T2)/        */
    if (conic)
    {
        rs->conic_t = ic->sv.tc; /* T (30D)       */
    }
    return AGC_STATUS_OK;
}

/* PREC120 (p. 908): DT21PR = POSMAX, NN2 = -15, next integration RTENCK3. */
static void prec_restart_inner(rte_erasable_t *rs)
{
    (void) AGC_ASSERT(rs != NULL);
    (void) AGC_ASSERT(rs->nn1a <= RTE_PREC_OUTER);
    rs->dt21pr    = TWO_B1;
    rs->nn2       = -RTE_PREC_INNER; /* M15RTE        */
    rs->prec_step = RTE_PREC_FROM_TIG;
}

/* PREC100 (p. 908): NN1A = 10, RD = RCON, then PREC120.                  */
agc_status_t rte_precision_start(rte_erasable_t *rs)
{
    if (!AGC_ASSERT(rs != NULL) || !AGC_ASSERT(rs->rcon > 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rs->nn1a       = RTE_PREC_OUTER; /* 10RTE         */
    rs->rd         = rs->rcon;
    rs->prec_steps = 0;
    prec_restart_inner(rs);
    return AGC_STATUS_OK;
}

/* PREC140 .. PREC162 (p. 909): BETA4 (B1), the ratio of the new radius to
 * RPRE at which the desired reentry angle is reached:
 *   BETA2 = BETA1 P/R, BETA3 = 1 - BETA2 R/A,
 *   BETA4 = BETA2 / (1 - PHI2 SQRT(BETA3))   (BETA3 >= 0)
 *         = A/R                               (BETA3 < 0, PREC160)
 *         = RD/RPRE                           (last pass, NN1A = 0)    */
static agc_dp_t prec_radius_ratio(agc_interp_t *ip, const rte_erasable_t *rs)
{
    int n1 = 0;

    (void) AGC_ASSERT(ip != NULL);
    (void) AGC_ASSERT(rs != NULL);
    if (rs->nn1a == 0)
    { /* PREC162       */
        agc_dp_t rn = agc_dp_norm(rs->rpre, &n1);
        return agc_dp_shl(ip, agc_dp_div(ip, rs->rd, rn), n1 - 1);
    }
    (void) AGC_ASSERT(rs->beta1 > 0); /* 1 + X(T2)^2   */
    agc_tp_t t     = agc_dp_mul_tp(agc_dp_shl(ip, rs->p_rpre, 2), rs->beta1);
    agc_dp_t beta2 = agc_tp_to_dp(agc_tp_shift(ip, t, 1));   /* BETA2 B2      */
    t = agc_dp_mul_tp(agc_dp_shl(ip, rs->r_apre, 4), beta2); /* SL4 DMP 00D   */
    agc_dp_t beta3 =
        agc_dp_sub(ip, ONE_B4, agc_tp_to_dp(t)); /* BETA3 B4      */
    if (beta3 < 0)
    { /* PREC160       */
        agc_dp_t rn = agc_dp_norm(rs->r_apre, &n1);
        return agc_dp_shl(ip, agc_dp_div(ip, ONE_B1, rn), n1 - 6);
    }
    agc_dp_t s = agc_dp_sqrt(ip, agc_dp_shl(ip, beta3, 2)); /* PREC155 B1    */
    agc_dp_t den = agc_dp_sub(ip, ONE_B3, agc_dp_mul_trunc(s, rs->phi2));
    den          = agc_dp_norm(den, &n1); /* NORM X1       */
    return agc_dp_shl(ip, agc_dp_div(ip, agc_dp_shr(beta2, 1), den), n1 - 1);
}

/* PREC170 .. PREC174 (pp. 910-911): time DT21 from R(T2)/ to the new
 * radius RF on the osculating conic (TIMERAD, time only, backwards when
 * BETA12 < 0), limited to DT21PR (forward) or -.6 DT21PR (reversal).    */
static agc_status_t prec_time_step(agc_interp_t *ip, rte_erasable_t *rs,
                                   agc_dp_t rf)
{
    agc_timerad_in_t     in;
    agc_conic_time_out_t out;
    int                  n1 = 0;

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rf > 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rs->nn2 += 1; /* PREC170       */
    agc_flag_set(AGC_FLAG_RVSW);
    in.rvec = rs->r_t2v;
    in.vvec =
        (rs->beta12 < 0) ? agc_vec_neg(rs->v_t2v) : rs->v_t2v; /* SIGN      */
    in.rdesired = rf;
    in.sgnrdot  = (rs->beta12 < 0) ? SGNRDOT_HALF : (agc_sp_t) -SGNRDOT_HALF;
    in.rvsw     = true;
    in.body     = AGC_CONIC_EARTH;
    agc_status_t st = agc_conic_timerad(ip, &in, &out);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rs->conic_t = out.t;
    (void) AGC_ASSERT(rs->dt21pr != 0);
    agc_dp_t dt21 = agc_dp_sign(out.t, rs->beta12); /* PHI4 DT21     */
    agc_dp_t rn   = agc_dp_norm(rs->dt21pr, &n1);
    agc_dp_t beta13 =
        agc_dp_shl(ip, agc_dp_div(ip, dt21, rn), n1 - 3); /* B3   */
    agc_dp_t beta14 = (beta13 < 0) ? M_POINT6 : TWO_B1;   /* PREC172/173   */
    if (agc_dp_sub(ip, agc_dp_div(ip, beta13, beta14), ONE_B3) >= 0)
    {
        dt21 = agc_dp_mul_trunc(beta14, rs->dt21pr); /* BETA14 DT21PR */
    }
    rs->dt21      = dt21; /* PREC174       */
    rs->dt21pr    = dt21;
    rs->prec_step = RTE_PREC_TIME_STEP; /* RTENCK2       */
    return AGC_STATUS_OK;
}

/* PREC130 .. PREC168 (pp. 909-910): the desired reentry angle was not
 * reached; NOT_FOUND = close enough after all (go to PREC175).           */
static agc_status_t prec_angle_not_met(agc_interp_t *ip, rte_erasable_t *rs)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->rpre > 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (rs->nn2 >= 0)
    { /* PREC132       */
        rs->alarm_code = RTE_ALARM_ITERATIONS;
        return AGC_STATUS_ALARM;
    }
    agc_dp_t beta4 = prec_radius_ratio(ip, rs);             /* PREC140       */
    rs->beta12 = agc_dp_neg(agc_dp_sub(ip, beta4, ONE_B1)); /* PREC165       */
    if ((rs->beta12 >= 0) && (rs->x_t2pre >= 0))
    {
        rs->beta12 = agc_dp_neg(rs->beta12); /* PREC167       */
    }
    if (agc_dp_sub(ip, agc_dp_abs(rs->beta12), EPC6RTE) < 0)
    { /* PREC168       */
        return AGC_STATUS_NOT_FOUND;
    }
    agc_tp_t rf =
        agc_tp_shift(ip, agc_dp_mul_tp(beta4, rs->rpre), 1); /* RF B29 */
    return prec_time_step(ip, rs, agc_tp_to_dp(rf));
}

/* PREC205 .. PREC207 (pp. 911-912): new RCON. First pass RD^2/RPRE; then
 * the secant S = DRCON/(RPRE' - RPRE), kept in (-4, 0) (else -4), and
 * DRCON = S RERR.                                                        */
static void prec_new_rcon(agc_interp_t *ip, rte_erasable_t *rs, agc_dp_t rerr)
{
    int n1 = 0;
    int n2 = 0;

    (void) AGC_ASSERT((ip != NULL) && (rs != NULL));
    (void) AGC_ASSERT(rs->nn1a > 0);
    if (rs->nn1a == RTE_PREC_OUTER)
    { /* PREC207       */
        agc_dp_t a  = agc_tp_to_dp(agc_tp_norm(agc_dp_sq_tp(rs->rd), &n1));
        agc_dp_t rn = agc_dp_norm(rs->rpre, &n2);
        agc_dp_t q  = agc_dp_div(ip, agc_dp_shr(a, 1), rn); /* BDDV          */
        rs->rcon    = agc_dp_shl(ip, q, 1 - n1 + n2);       /* SR* 0-1,1     */
        rs->drcon   = agc_dp_sub(ip, rs->rcon, rs->rd);
        return;
    }
    agc_dp_t d = agc_dp_norm(agc_dp_sub(ip, rs->rpre_prev, rs->rpre), &n2);
    agc_dp_t s =
        agc_dp_shl(ip, agc_dp_div(ip, rs->drcon, d), n2 - 2); /* S B2  */
    agc_dp_t sp2 = agc_dp_add(ip, s, ONE_B1);
    if (agc_interp_test_ovf(ip) ||
        (agc_dp_sub(ip, agc_dp_abs(sp2), ONE_B1) >= 0))
    {
        s = agc_dp_neg(TWO_B1); /* PREC205M: -4  */
    }
    agc_tp_t t =
        agc_tp_shift(ip, agc_dp_mul_tp(s, rerr), 2); /* DMP SL2       */
    rs->drcon = agc_tp_to_dp(t);                     /* PREC206       */
    rs->rcon  = agc_dp_add(ip, rs->drcon, rs->rcon);
}

/* PREC175 .. PREC225 (pp. 911-913): radius check; when not met, a new
 * RCON and V2(T1)/ (PREC210: NN1A - 1, V2T100) and the next outer pass;
 * when met, the reentry angle check (613). NOT_FOUND = PRECX with 0.    */
static agc_status_t prec_radius_check(agc_interp_t *ip, rte_erasable_t *rs)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->rd > 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t rerr = agc_dp_sub(ip, rs->rpre, rs->rd); /* RERR          */
    if (agc_dp_sub(ip, agc_dp_abs(rerr), EPC7RTE) < 0)
    { /* PREC220       */
        agc_dp_t e = agc_dp_abs(agc_dp_sub(ip, rs->x_t2, rs->x_t2pre));
        if (agc_dp_sub(ip, e, EPC8RTE) < 0)
        {
            return AGC_STATUS_NOT_FOUND; /* PREC225       */
        }
        rs->alarm_code = RTE_ALARM_ENTRY_ANGLE; /* OCT613        */
        return AGC_STATUS_ALARM;
    }
    if (rs->nn1a == 0)
    { /* PREC132       */
        rs->alarm_code = RTE_ALARM_ITERATIONS;
        return AGC_STATUS_ALARM;
    }
    (void) AGC_ASSERT(rs->nn1a <= RTE_PREC_OUTER);
    prec_new_rcon(ip, rs, rerr);
    rs->rpre_prev = rs->rpre; /* PREC210       */
    rs->nn1a -= 1;
    agc_status_t st = rte_post_impulse_velocity(ip, rs); /* V2T100        */
    if (st == AGC_STATUS_OK)
    {
        prec_restart_inner(rs); /* BHIZ PREC120  */
    }
    return st;
}

/* PREC125 (p. 908) after each integration: PARAM on R(T2)/, V(T2)/ ->
 * P/RPRE, R/APRE, RPRE, X(T2)PRE = COGA SL5; compare with X(T2).        */
static agc_status_t prec_after_integration(agc_interp_t *ip, rte_erasable_t *rs)
{
    agc_conic_param_out_t po;

    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->nn1a >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st =
        agc_conic_param(ip, rs->r_t2v, rs->v_t2v, AGC_CONIC_EARTH, &po);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    (void) AGC_ASSERT(po.r1 > 0);
    rs->p_rpre  = po.p;
    rs->r_apre  = po.r1a;
    rs->rpre    = po.r1;
    rs->x_t2pre = agc_dp_shl(ip, po.coga, 5); /* SL 5 -> B0    */
    agc_dp_t e  = agc_dp_abs(agc_dp_add(ip, agc_dp_neg(rs->x_t2pre), rs->x_t2));
    e           = agc_dp_sub(ip, e, EPC4RTE);
    if (agc_interp_test_ovf(ip) || (e >= 0))
    { /* PREC130       */
        st = prec_angle_not_met(ip, rs);
        if (st != AGC_STATUS_NOT_FOUND)
        {
            return st;
        }
    }
    return prec_radius_check(ip, rs); /* PREC175       */
}

/* PREC120 .. PRECX (pp. 908-913): run the precision iteration from the
 * current rs->prec_step. OK: done (PRECX with 0, PREC225); ALARM 605/613
 * (rs->alarm_code); BUSY: asleep in INTSTALL, call again as `resume`;
 * POODOO from V2T100/TIMERAD.                                            */
agc_status_t rte_precision_run(agc_interp_t *ip, rte_erasable_t *rs,
                               agc_job_id_t resume)
{
    if (!AGC_ASSERT((ip != NULL) && (rs != NULL)) ||
        !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    while (rs->prec_steps < RTE_PREC_MAX_STEPS)
    {
        agc_status_t st;
        if (rs->prec_step == RTE_PREC_FROM_TIG)
        { /* RTENCK3       */
            st = rte_integrate(ip, rs, rs->r_t1v, rs->v2_t1v, rs->t1, rs->t2,
                               false, resume);
        }
        else
        { /* RTENCK2       */
            agc_dp_t tdec1 = agc_dp_add(ip, rs->t2, rs->dt21);
            st = rte_integrate(ip, rs, rs->r_t2v, rs->v_t2v, rs->t2, tdec1,
                               false, resume);
        }
        if (st != AGC_STATUS_OK)
        {
            return st; /* BUSY: sleep   */
        }
        rs->prec_steps += 1;
        st = prec_after_integration(ip, rs); /* PREC125       */
        if (st != AGC_STATUS_OK)
        {
            return (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
        }
    }
    rs->alarm_code =
        RTE_ALARM_ITERATIONS; /* cannot happen: NN1A, NN2 bound it */
    return AGC_STATUS_ALARM;
}
