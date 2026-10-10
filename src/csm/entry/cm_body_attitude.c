/*
 * cm_body_attitude.c -- CM_BODY_ATTITUDE.agc (Comanche055, pp. 883-889).
 *
 * CM/POSE runs every 2 s during entry as the AVEGEXIT of SERVICER: it forms
 * the trajectory triad (UXA = -UNIT(VREL), UYA, UZA), the CM body triad
 * from the gimbal angles saved at PIPA time, the body attitude angles
 * ROLL, BETA, ALFA (/180) and GAMA, and then continues at POSEXIT
 * (P62.3, STARTENT, SCALEPOP or ENDEXIT). All vectors are half-unit in
 * reference coordinates unless noted.
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
#include "agc/csm/entry_internal.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/csm/entry_pending.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"

/* Restart storage TEMPROLL, TEMPALFA, TEMPBETA (ENTRY RESTART PROTECTION
 * STORAGE): the updated angles /180, SP 2s complement.                    */
static agc_sp_t temproll;
static agc_sp_t tempalfa;
static agc_sp_t tempbeta;

/* SP angle of a gimbal saved at PIP time as the 15-bit CDU pattern.       */
static uint16_t cdu_pattern(agc_sp_t a)
{
    (void) AGC_ASSERT(a >= -16384);
    (void) AGC_ASSERT(a <= 16383);
    return (uint16_t) ((uint16_t) a & 077777u);
}

/* CM/POSE, CM/POSE2 (p. 883): -VREL = -V KVSCALE + KWE (UNITW x UNITR),
 * UXA = UNIT(-VREL), UYA = UNIT(UNITR x UXA) while |VREL| > VQUIT (else
 * OLDUYA), UZA = UXA x UYA.                                               */
agc_status_t agc_entry_cm_pose_triad(void)
{
    agc_interp_t      *ip  = entry_ip();
    agc_entry_state_t *es  = agc_entry_state();
    agc_avg_g_t       *avg = &agc_guid_state()->avg;
    agc_vec_t          w;
    agc_vec_t          uy;
    agc_dp_t           vmag = 0;

    if (!AGC_ASSERT(es != NULL) || !AGC_ASSERT(avg != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    w         = agc_vec_scale(ip, agc_vec_cross(ip, avg->unitw, avg->unitr),
                              AGC_ENTRY_KWE);
    es->mvrel = agc_vec_add(
        ip, w, agc_vec_scale(ip, agc_erase_vn(), AGC_ENTRY_MKVSCALE));
    es->uxa = agc_vec_unit(ip, es->mvrel, &vmag, NULL); /* LXA,1 36D  */
    uy      = agc_vec_neg(agc_vec_cross(ip, es->uxa, avg->unitr));
    uy      = agc_vec_unit(ip, uy, NULL, NULL);
    /* TIX,1: branch to CM/POSE2 (new UYA) when X1 = |VREL| > S1 = SPVQUIT. */
    if (agc_dp_high(vmag) > AGC_ENTRY_SPVQUIT)
    {
        es->olduya = uy;
    }
    es->uya = es->olduya; /* CM/POSE2  */
    es->uza =
        agc_vec_shl(ip, agc_vec_neg(agc_vec_cross(ip, es->uya, es->uxa)), 1);
    (void) AGC_ASSERT(vmag >= 0);
    return AGC_STATUS_OK;
}

/* CM/TRIO part 1 (p. 884): UBY in platform coordinates /4,
 * YB = (-CO SM CI + SO SI, CO CM, CO SM SI + SO CI).                      */
static agc_vec_t trio_uby_sm(agc_interp_t *ip, agc_dp_t ci, agc_dp_t si,
                             agc_dp_t sm, agc_dp_t cm, agc_dp_t so, agc_dp_t co)
{
    agc_vec_t uby;

    (void) AGC_ASSERT(ip != NULL);
    (void) AGC_ASSERT(agc_dp_abs(co) <= AGC_DP_HALF);
    /* (-SM CI, noise, SM SI)/2 with CM/2 replacing the noise              */
    uby = agc_vec_make(
        agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_neg(sm), ci), 1), cm,
        agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_neg(sm), agc_dp_neg(si)), 1));
    uby      = agc_vec_scale(ip, uby, co); /* x CO/2 -> /4 */
    uby.c[0] = agc_dp_add(ip, agc_dp_neg(agc_dp_mul(ip, so, agc_dp_neg(si))),
                          uby.c[0]);
    uby.c[2] = agc_dp_add(ip, agc_dp_mul(ip, so, ci), uby.c[2]);
    return uby;
}

/* CM/TRIO part 2 (pp. 884-885): UBX, UBY, UBZ in reference coordinates
 * from the PIP-time gimbal angles and REFSMMAT.                           */
static void trio_body_axes(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_dp_t  aig      = agc_guid_cdulogic(cdu_pattern(es->aig_pip));
    agc_dp_t  amg      = agc_guid_cdulogic(cdu_pattern(es->amg_pip));
    agc_dp_t  aog      = agc_guid_cdulogic(cdu_pattern(es->aog_pip));
    agc_dp_t  ci       = agc_dp_cos(ip, aig);
    agc_dp_t  si       = agc_dp_sin(ip, aig);
    agc_dp_t  sm       = agc_dp_sin(ip, amg);
    agc_dp_t  cm       = agc_dp_cos(ip, amg);
    agc_vec_t ubx;

    (void) AGC_ASSERT(es != NULL);
    es->uby = trio_uby_sm(ip, ci, si, sm, cm, agc_dp_sin(ip, aog),
                          agc_dp_cos(ip, aog));
    es->uby =
        agc_vec_shl(ip, agc_vec_mat(ip, es->uby, &refsmmat), 2); /* VXM VSL2 */
    /* XB = (CM CI, SM, -CM SI)/2                                          */
    ubx = agc_vec_make(agc_dp_shl(ip, agc_dp_mul(ip, cm, ci), 1), sm,
                       agc_dp_shl(ip, agc_dp_mul(ip, cm, agc_dp_neg(si)), 1));
    es->ubx =
        agc_vec_shl(ip, agc_vec_mat(ip, ubx, &refsmmat), 1); /* VXM VSL1 */
    es->ubz = agc_vec_shl(ip, agc_vec_cross(ip, es->ubx, es->uby), 1);
    (void) AGC_ASSERT(agc_dp_abs(sm) <= AGC_DP_HALF);
}

/* CM/TRIO (pp. 884-886): body triad, then
 *   angles[0] = -(ROLL/180)/2  from ARCTRIG(UL.UYA, UL.UZA)
 *   angles[1] = -(BETA/180)/2  from ARCSIN(-UVA.UBY)
 *   angles[2] = -(ALFA/180)/2  from ARCTRIG(UL.UBX, UL.UBZ)
 *   angles[3] = (-GAMA/180)/2  from ARCCOS(UR.UZA)
 * with UL = UNIT(UXA x UBY), all valid at PIPA time.                      */
agc_status_t agc_entry_cm_trio(agc_dp_t angles[4])
{
    agc_interp_t      *ip = entry_ip();
    agc_entry_state_t *es = agc_entry_state();
    agc_vec_t          ul;
    agc_dp_t           s;
    agc_dp_t           c;

    if (!AGC_ASSERT(angles != NULL) || !AGC_ASSERT(es != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    trio_body_axes(ip, es);
    ul = agc_vec_unit(ip, agc_vec_cross(ip, es->uxa, es->uby), NULL, NULL);
    c  = agc_vec_dot(ip, ul, es->uza); /* COS(ROLL)/4  */
    s  = agc_vec_dot(ip, ul, es->uya); /* -SIN(ROLL)/4 */
    angles[0] = agc_imu_arctrig(ip, s, c);
    s         = agc_dp_shl(ip, agc_vec_dot(ip, es->uby, es->uxa), 1);
    angles[1] = agc_dp_asin(ip, s);
    s         = agc_vec_dot(ip, ul, es->ubx); /* -SIN(ALFA)/4 */
    c         = agc_vec_dot(ip, ul, es->ubz); /* COS(ALFA)/4  */
    angles[2] = agc_imu_arctrig(ip, s, c);
    c = agc_dp_shl(ip, agc_vec_dot(ip, agc_guid_state()->avg.unitr, es->uza),
                   1);
    angles[3] = agc_dp_acos(ip, c);
    return AGC_STATUS_OK;
}

/* CMTR1 GAMDOT part (p. 887): GAMA = new flight path angle; GAMDOT =
 * (GAMA - old GAMA) TCDU, set to +0 on the first pass (GAMDIFSW clear,
 * then set) or when below 5 bits (about .5 deg/s).                       */
static void atup_gamdot(agc_entry_state_t *es, agc_sp_t gama_new)
{
    agc_sp_t old = es->gama;
    int32_t  g;

    (void) AGC_ASSERT(agc_interrupts_inhibited());
    es->gama = gama_new;
    if (!agc_flag_test(AGC_FLAG_GAMDIFSW))
    {
        (void) agc_flag_set(AGC_FLAG_GAMDIFSW); /* first difference next */
        es->gamdot = 0;                         /* NOGAMDOT              */
        return;
    }
    es->gamdot = entry_sp_mp(entry_sp_sat((int32_t) es->gama - (int32_t) old),
                             AGC_ENTRY_TCDU);
    g = (es->gamdot > 0) ? -(int32_t) es->gamdot : (int32_t) es->gamdot;
    if (g + 5 > 0)
    { /* |GAMDOT| < 5 bits     */
        es->gamdot = 0;
    }
    (void) AGC_ASSERT(es->gamdot == 0 || es->gamdot >= 5 || es->gamdot <= -5);
}

/* One Euler angle: (EUL/180) = -2 MPAC, + (NOW - PIP) increment since
 * PIPA time, each sum corrected by CORANGOV.                              */
static agc_sp_t atup_angle(agc_dp_t mpac, agc_sp_t pip, agc_sp_t now, bool wrap)
{
    int32_t eul = -2 * (int32_t) agc_dp_high(mpac); /* CS MPAC, DOUBLE      */
    int32_t sum;

    (void) AGC_ASSERT(eul >= -32768 && eul <= 32768);
    (void) AGC_ASSERT(pip >= -16384);
    if (wrap)
    {
        sum = (int32_t) entry_angle_wrap(eul) - (int32_t) pip + (int32_t) now;
        return entry_angle_wrap(sum);
    }
    return entry_sp_sat(eul - (int32_t) pip +
                        (int32_t) now); /* BETA: no ovfl */
}

/* CM/ATUP, CMTR1, CMTR2 (pp. 887-888): GAMA, GAMDOT, and the body angles
 * brought from PIPA time to now; restart group 5 at REDOPOSE.             */
static void entry_cm_atup(const agc_dp_t angles[4])
{
    agc_entry_state_t *es = agc_entry_state();
    agc_phase_var_t    v  = {
            AGC_RESTART_JOB, AGC_JOB_REDOPOSE, AGC_TASK_NONE, 0, false, 0U};

    (void) AGC_ASSERT(angles != NULL);
    agc_inhint(); /* until the body angles are updated     */
    atup_gamdot(es, (agc_sp_t) (-agc_dp_high(angles[3]))); /* CS 10D */
    temproll = atup_angle(angles[0], es->roll_pip, es->roll_180, true);
    tempalfa = atup_angle(angles[2], es->alfa_pip, es->alfa_180, true);
    tempbeta = atup_angle(angles[1], es->beta_pip, es->beta_180, false);
    /* DCA REPOSADR / DXCH PHSNAME5: group 5 now restarts at REDOPOSE.     */
    (void) agc_phase_change_var(010035, &v);
    agc_relint();
    (void) AGC_ASSERT(es->gamdot == 0 || agc_flag_test(AGC_FLAG_GAMDIFSW));
}

/* SERVNOUT / NODISKY (REENTRY_CONTROL p. 874): CHANG1 if a higher job
 * waits (resume at SERVEXIT), else POSTJUMP SERVEXIT.                     */
agc_status_t entry_servnout(agc_job_ctx_t *ctx)
{
    int nj;

    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    nj = agc_exec_newjob(); /* CCS NEWJOB                   */
    agc_relint();
    if (nj > 0)
    {
        return agc_exec_job_yield(AGC_JOB_SERVEXIT); /* TC CHANG1  */
    }
    (void) AGC_ASSERT(nj <= 0);
    return agc_job_servexit(ctx);
}

/* GOTO POSEXIT (p. 888): continue the pass where the program asked.       */
static agc_status_t entry_posexit(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es   = agc_entry_state();
    agc_status_t       st   = AGC_STATUS_OK;
    bool               disp = true;

    (void) AGC_ASSERT(ctx != NULL);
    switch (es->posexit)
    {
        case AGC_ENTRY_POSEXIT_DESIRED_GIMBALS:
            st =
                agc_entry_desired_gimbal_angles(); /* QPRET = ENDEXIT             */
            break;
        case AGC_ENTRY_POSEXIT_STARTENT:
            st   = agc_entry_startent();
            disp = false; /* RTB SERVNOUT                */
            break;
        case AGC_ENTRY_POSEXIT_SCALEPOP:
            st = agc_entry_scalepop(&disp);
            break;
        case AGC_ENTRY_POSEXIT_ENDEXIT:
        case AGC_ENTRY_POSEXIT_COUNT:
        default:
            (void) AGC_ASSERT(es->posexit == AGC_ENTRY_POSEXIT_ENDEXIT);
            break;
    }
    if (st == AGC_STATUS_OK && disp)
    {
        st = entry_endexit(ctx, es); /* OVERNOUT                    */
    }
    if (st != AGC_STATUS_OK)
    {
        (void) entry_servnout(ctx); /* end the SERVICER job anyway */
        return st;
    }
    return entry_servnout(ctx);
}

/* REDOPOSE, CM/POSE3 (p. 888): store the updated angles, VMAGI = |VN|,
 * GOTO POSEXIT.                                                           */
static agc_status_t entry_redopose_tail(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    es->roll_180 = temproll;
    es->alfa_180 = tempalfa;
    es->beta_180 = tempbeta;
    agc_relint();
    es->vmagi = agc_vec_abval(entry_ip(), agc_erase_vn()); /* 2^7 m/cs */
    return entry_posexit(ctx);
}

/* CM/POSE (pp. 883-888): the entry AVEGEXIT job.                          */
agc_status_t agc_job_cm_pose(agc_job_ctx_t *ctx)
{
    agc_dp_t     angles[4] = {0, 0, 0, 0};
    agc_status_t st;

    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    st = agc_entry_cm_pose_triad();
    if (st == AGC_STATUS_OK)
    {
        st = agc_entry_cm_trio(angles);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    entry_cm_atup(angles);
    (void) AGC_ASSERT(agc_entry_state()->posexit < AGC_ENTRY_POSEXIT_COUNT);
    return entry_redopose_tail(ctx);
}

/* REDOPOSE as the group 5 restart job (p. 888).                          */
agc_status_t agc_job_redopose(agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    (void) AGC_ASSERT(agc_entry_state() != NULL);
    return entry_redopose_tail(ctx);
}

/* REFAZE10 as the group 5 restart job (REENTRY_CONTROL p. 847).          */
agc_status_t agc_job_refaze10(agc_job_ctx_t *ctx)
{
    bool         disp = false;
    agc_status_t st;

    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    st = agc_entry_refaze10(&disp);
    if (st == AGC_STATUS_OK && disp)
    {
        st = entry_endexit(ctx, agc_entry_state());
    }
    (void) AGC_ASSERT(agc_entry_state()->gotoaddr < AGC_ENTRY_GOTO_COUNT);
    return (st == AGC_STATUS_OK) ? entry_servnout(ctx) : st;
}
