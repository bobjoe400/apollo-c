/**********************************************************************
 * FILE NAME: cm_body_attitude.c
 *
 * PURPOSE:
 *   CM/POSE, run every 2 s during entry as the AVEGEXIT of SERVICER: the
 *   trajectory triad (UXA = -UNIT(VREL), UYA, UZA), the CM body triad from
 *   the gimbal angles saved at PIPA time, the body attitude angles ROLL,
 *   BETA, ALFA (/180) and GAMA, GAMDOT, then the continuation at POSEXIT
 *   (P62.3, STARTENT, SCALEPOP or ENDEXIT) and SERVNOUT.
 *
 * AGC SOURCE:
 *   Comanche055 CM_BODY_ATTITUDE.agc pp. 883-889 (CM/POSE, CM/TRIO,
 *   CM/ATUP, CMTR1, CMTR2, REDOPOSE, CM/POSE3)
 *   Comanche055 REENTRY_CONTROL.agc pp. 847, 874 (REFAZE10, SERVNOUT)
 *   Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   VN            agc_vec_t  I    m/cs B7        SERVICER velocity
 *   UNITR, UNITW  agc_vec_t  I    B1, B0         average-G unit vectors
 *   REFSMMAT      agc_mat_t  I    half-unit      stable member matrix
 *   GAMDIFSW      flag       IO   -              first GAMDOT pass
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_guid_cdulogic     CDULOGIC: CDU pattern to revs
 *   agc_imu_arctrig       ARCTRIG (ROLL, ALFA)
 *   agc_entry_scalepop    entry guidance pass (reentry.c)
 *   agc_job_servexit      SERVEXIT (SERVICER)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none of its own; a BAILOUT asked for by SERVNOUT outranks the status
 *   of the guidance pass.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only; runs in the SERVICER job. Restart group 5 at REDOPOSE once
 *   the updated angles are in TEMPROLL..TEMPBETA.
 *
 * NOTES:
 *   All vectors are half-unit in reference coordinates unless noted.
 *   DOT SL1 (BETA, GAMA) keeps the TP MPAC (entry_tp_shl).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"

/* Restart storage TEMPROLL, TEMPALFA, TEMPBETA (ENTRY RESTART PROTECTION
 * STORAGE): the updated angles /180, SP 2s complement.                    */
static agc_sp_t temproll; /* TEMPROLL ROLL/180, B0 */
static agc_sp_t tempalfa; /* TEMPALFA ALFA/180, B0 */
static agc_sp_t tempbeta; /* TEMPBETA BETA/180, B0 */

/**********************************************************************
 * AGC: none (CM_BODY_ATTITUDE.agc p. 884 AOG/PIP words) -- the 15-bit CDU
 * pattern of an SP angle saved at PIP time (2s complement /180).
 *
 * FUNCTION NAME: cdu_pattern
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   a         agc_sp_t  I    gimbal angle /180 at PIP time, SP 2s
 *                            complement, B0
 *
 * RETURNS: uint16_t -- uint16_t -- the CDU counter pattern (bits 1-15) for
 *          CDULOGIC
 **********************************************************************/
static uint16_t cdu_pattern(agc_sp_t a)
{
    return ((uint16_t) ((uint16_t) a & 077777u));
}

/**********************************************************************
 * CM/POSE, CM/POSE2 (p. 883): -VREL = -V KVSCALE + KWE (UNITW x UNITR), UXA
 * = UNIT(-VREL), UYA = UNIT(UNITR x UXA) while |VREL| > VQUIT (else
 * OLDUYA), UZA = UXA x UYA.
 *
 * FUNCTION NAME: agc_entry_cm_pose_triad
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- always AGC_STATUS_OK (no failure case)
 **********************************************************************/
agc_status_t agc_entry_cm_pose_triad(void)
{
    agc_interp_t      *ip   = entry_ip();             /* entry job interpreter state */
    agc_entry_state_t *es   = agc_entry_state();      /* entry erasables */
    agc_avg_g_t       *avg  = &agc_guid_state()->avg; /* average-G data (UNITW, UNITR) */
    agc_vec_t          w;                             /* KWE (UNITW x UNITR), m/cs B7 */
    agc_vec_t          uy;                            /* UNIT(UNITR x UXA), B1 */
    agc_dp_t           vmag = 0;                      /* |VREL|, 2VS B1 */

    w         = agc_vec_scale(ip, agc_vec_cross(ip, avg->unitw, avg->unitr), AGC_ENTRY_KWE);
    es->mvrel = agc_vec_add(ip, w, agc_vec_scale(ip, agc_erase_vn(), AGC_ENTRY_MKVSCALE));
    es->uxa   = agc_vec_unit(ip, es->mvrel, &vmag, NULL); /* LXA,1 36D  */
    uy        = agc_vec_neg(agc_vec_cross(ip, es->uxa, avg->unitr));
    uy        = agc_vec_unit(ip, uy, NULL, NULL);

    /* TIX,1: branch to CM/POSE2 (new UYA) when X1 = |VREL| > S1 = SPVQUIT. */
    if (agc_dp_high(vmag) > AGC_ENTRY_SPVQUIT)
    {
        es->olduya = uy;
    }

    es->uya = es->olduya; /* CM/POSE2  */
    es->uza = agc_vec_shl(ip, agc_vec_neg(agc_vec_cross(ip, es->uya, es->uxa)), 1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CM/TRIO (pp. 884-886): body triad, then angles[0] = -(ROLL/180)/2  from
 * ARCTRIG(UL.UYA, UL.UZA) angles[1] = -(BETA/180)/2  from ARCSIN(-UVA.UBY)
 * angles[2] = -(ALFA/180)/2  from ARCTRIG(UL.UBX, UL.UBZ) angles[3] =
 * (-GAMA/180)/2  from ARCCOS(UR.UZA) with UL = UNIT(UXA x UBY), all valid
 * at PIPA time.
 *
 * FUNCTION NAME: agc_entry_cm_trio
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   angles    agc_dp_t  O    -(ROLL, BETA, ALFA)/180/2 and (-GAMA/180)/2 at
 *                            PIPA time, revs B0
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a failed
 *          assertion
 **********************************************************************/
agc_status_t agc_entry_cm_trio(agc_dp_t angles[4])
{
    agc_interp_t      *ip = entry_ip();        /* entry job interpreter state */
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */
    agc_vec_t          ul;                     /* UL = UNIT(UXA x UBY), B1 */
    agc_dp_t           s;                      /* sine term, B2 */
    agc_dp_t           c;                      /* cosine term, B2 */

    /* CM/TRIO part 2 (pp. 884-885): UBX, UBY, UBZ in reference coordinates
     * from the PIP-time gimbal angles and REFSMMAT. */
    agc_mat_t refsmmat = agc_erase_refsmmat();                        /* REFSMMAT, half-unit rows */
    agc_dp_t  aig      = agc_guid_cdulogic(cdu_pattern(es->aig_pip)); /* AIG/PIP, revs B0 */
    agc_dp_t  amg      = agc_guid_cdulogic(cdu_pattern(es->amg_pip)); /* AMG/PIP, revs B0 */
    agc_dp_t  aog      = agc_guid_cdulogic(cdu_pattern(es->aog_pip)); /* AOG/PIP, revs B0 */
    agc_dp_t  ci       = agc_dp_cos(aig);                             /* COS AIG /2, B1 */
    agc_dp_t  si       = agc_dp_sin(aig);                             /* SIN AIG /2, B1 */
    agc_dp_t  sm       = agc_dp_sin(amg);                             /* SIN AMG /2, B1 */
    agc_dp_t  cm       = agc_dp_cos(amg);                             /* COS AMG /2, B1 */
    agc_vec_t ubx;                                                    /* XB/2 platform, B1 */

    /* CM/TRIO part 1 (p. 884): UBY in platform coordinates /4,
     * YB = (-CO SM CI + SO SI, CO CM, CO SM SI + SO CI). */
    agc_dp_t so = agc_dp_sin(aog); /* SIN AOG /2, B1 */
    agc_dp_t co = agc_dp_cos(aog); /* COS AOG /2, B1 */

    agc_vec_t uby; /* YB/4 platform, B2 */

    if (!AGC_ASSERT(agc_dp_abs(co) <= AGC_DP_HALF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* (-SM CI, noise, SM SI)/2 with CM/2 replacing the noise              */
    uby      = agc_vec_make(agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_neg(sm), ci), 1), cm, agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_neg(sm), agc_dp_neg(si)), 1));
    uby      = agc_vec_scale(ip, uby, co);                              /* x CO/2 -> /4 */
    uby.c[0] = agc_dp_add(ip, agc_dp_neg(agc_dp_mul_trunc(so, agc_dp_neg(si))), uby.c[0]);
    uby.c[2] = agc_dp_add(ip, agc_dp_mul_trunc(so, ci), uby.c[2]);
    es->uby  = uby;
    es->uby  = agc_vec_shl(ip, agc_vec_mat(ip, es->uby, &refsmmat), 2); /* VXM VSL2 */

    /* XB = (CM CI, SM, -CM SI)/2                                          */
    ubx     = agc_vec_make(agc_dp_shl(ip, agc_dp_mul(ip, cm, ci), 1), sm, agc_dp_shl(ip, agc_dp_mul(ip, cm, agc_dp_neg(si)), 1));
    es->ubx = agc_vec_shl(ip, agc_vec_mat(ip, ubx, &refsmmat), 1); /* VXM VSL1 */
    es->ubz = agc_vec_shl(ip, agc_vec_cross(ip, es->ubx, es->uby), 1);

    if (!AGC_ASSERT(agc_dp_abs(sm) <= AGC_DP_HALF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    ul        = agc_vec_unit(ip, agc_vec_cross(ip, es->uxa, es->uby), NULL, NULL);
    c         = agc_vec_dot(ip, ul, es->uza); /* COS(ROLL)/4  */
    s         = agc_vec_dot(ip, ul, es->uya); /* -SIN(ROLL)/4 */
    angles[0] = agc_imu_arctrig(ip, s, c);
    s         = entry_tp_shl(ip, agc_vec_dot_tp(ip, es->uby, es->uxa), 1);
    angles[1] = agc_dp_asin(ip, s);
    s         = agc_vec_dot(ip, ul, es->ubx); /* -SIN(ALFA)/4 */
    c         = agc_vec_dot(ip, ul, es->ubz); /* COS(ALFA)/4  */
    angles[2] = agc_imu_arctrig(ip, s, c);
    c         = entry_tp_shl(ip, agc_vec_dot_tp(ip, agc_guid_state()->avg.unitr, es->uza), 1);
    angles[3] = agc_dp_acos(ip, c);

    return (AGC_STATUS_OK);
} /* agc_entry_cm_trio */

/**********************************************************************
 * AGC: none (CM/ATUP angle step, CM_BODY_ATTITUDE.agc pp. 887-888) -- one
 * Euler angle: (EUL/180) = -2 MPAC, + (NOW - PIP) increment since PIPA
 * time, each sum corrected by CORANGOV.
 *
 * FUNCTION NAME: atup_angle
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   mpac      agc_dp_t  I    -(EUL/180)/2 from CM/TRIO (high word used),
 *                            revs B0
 *   pip       agc_sp_t  I    the body angle /180 saved at PIPA time, B0
 *   now       agc_sp_t  I    the body angle /180 now, B0
 *   wrap      bool      I    true: CORANGOV after each add (ROLL, ALFA);
 *                            false: BETA
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- the angle /180 brought to now, B0 (SP 2s
 *          complement)
 **********************************************************************/
static agc_sp_t atup_angle(agc_dp_t mpac, agc_sp_t pip, agc_sp_t now, bool wrap)
{
    int32_t eul = -2 * (int32_t) agc_dp_high(mpac); /* CS MPAC, DOUBLE      */
    int32_t sum;                                    /* EUL - PIP + NOW, /180 */

    if (wrap)
    {
        sum = (int32_t) entry_angle_wrap(eul) - (int32_t) pip + (int32_t) now;

        return (entry_angle_wrap(sum));
    }

    /* CMTR2: XCH TEMPBETA keeps SG and bits 1-14 ("OVFL NOT EXPECTED"). */
    return (entry_sp_store_ovf(eul - (int32_t) pip + (int32_t) now));
}

/**********************************************************************
 * SERVNOUT / NODISKY (REENTRY_CONTROL p. 874): CHANG1 if a higher job waits
 * (resume at SERVEXIT), else POSTJUMP SERVEXIT.
 *
 * FUNCTION NAME: entry_servnout
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the CHANG1 status (agc_exec_job_yield) when a
 *          higher job waits, else the SERVEXIT status; AGC_STATUS_BAD_ARG
 *          from the NEWJOB assertion (cannot fail after the CCS test; kept
 *          while the EVA run is in progress)
 **********************************************************************/
agc_status_t entry_servnout(void)
{
    int nj; /* NEWJOB: priority waiting */

    agc_inhint();
    nj = agc_exec_newjob(); /* CCS NEWJOB                   */
    agc_relint();

    if (nj > 0)
    {
        return (agc_exec_job_yield(AGC_JOB_SERVEXIT)); /* TC CHANG1  */
    }

    if (!AGC_ASSERT(nj <= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_servexit());
}

/**********************************************************************
 * GOTO POSEXIT (p. 888): continue the pass where the program asked.
 *
 * FUNCTION NAME: entry_posexit
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t entry_posexit(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es   = agc_entry_state(); /* entry erasables */
    agc_status_t       st   = AGC_STATUS_OK;     /* status of the POSEXIT step */
    bool               disp = true;              /* OVERNOUT display wanted */

    switch (es->posexit)
    {
        case AGC_ENTRY_POSEXIT_DESIRED_GIMBALS:
            st   = agc_entry_desired_gimbal_angles(); /* QPRET = ENDEXIT */
            break;
        case AGC_ENTRY_POSEXIT_STARTENT:
            st   = agc_entry_startent();
            disp = false;                             /* RTB SERVNOUT                */
            break;
        case AGC_ENTRY_POSEXIT_SCALEPOP:
            st   = agc_entry_scalepop(&disp);
            break;
        case AGC_ENTRY_POSEXIT_ENDEXIT:
        case AGC_ENTRY_POSEXIT_COUNT:
        default:

            if (!AGC_ASSERT(es->posexit == AGC_ENTRY_POSEXIT_ENDEXIT))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            break;
    }

    if (st == AGC_STATUS_OK && disp)
    {
        st = entry_endexit(ctx, es); /* OVERNOUT                    */
    }

    if (st != AGC_STATUS_OK)
    {
        /* end the SERVICER job anyway; a restart it asks for outranks st */
        const agc_status_t sn = entry_servnout(); /* SERVNOUT status */

        return ((sn == AGC_STATUS_BAILOUT) ? sn : st); /* restart wins */
    }

    return (entry_servnout());
} /* entry_posexit */

/**********************************************************************
 * REDOPOSE, CM/POSE3 (p. 888): store the updated angles, VMAGI = |VN|, GOTO
 * POSEXIT.
 *
 * FUNCTION NAME: entry_redopose_tail
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t entry_redopose_tail(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    es->roll_180 = temproll;
    es->alfa_180 = tempalfa;
    es->beta_180 = tempbeta;
    agc_relint();
    es->vmagi    = agc_vec_abval(entry_ip(), agc_erase_vn()); /* 2^7 m/cs */

    return (entry_posexit(ctx));
}

/**********************************************************************
 * CM/POSE (pp. 883-888): the entry AVEGEXIT job.
 *
 * FUNCTION NAME: agc_job_cm_pose
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_cm_pose(agc_job_ctx_t *ctx)
{
    agc_dp_t     angles[4] = {0, 0, 0, 0};
    agc_status_t st; /* step status */

    st = agc_entry_cm_pose_triad();

    if (st == AGC_STATUS_OK)
    {
        st = agc_entry_cm_trio(angles);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* CM/ATUP, CMTR1, CMTR2 (pp. 887-888): GAMA, GAMDOT, and the body angles
     * brought from PIPA time to now; restart group 5 at REDOPOSE. */
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */
    agc_phase_var_t    v  = { AGC_RESTART_JOB, AGC_JOB_REDOPOSE, AGC_TASK_NONE, 0, false, 0U };

    agc_inhint(); /* until the body angles are updated     */

    /* CS 10D */
    /* CMTR1 GAMDOT part (p. 887): GAMA = new flight path angle; GAMDOT = (GAMA
     * - old GAMA) TCDU, set to +0 on the first pass (GAMDIFSW clear, then set)
     * or when below 5 bits (about .5 deg/s). */
    agc_sp_t gama_new = (agc_sp_t) (-agc_dp_high(angles[3]));

    agc_sp_t old = es->gama; /* old GAMA, B0 */
    int32_t  g;              /* -|GAMDOT|, SP units */

    es->gama = gama_new;

    if (!agc_flag_test(AGC_FLAG_GAMDIFSW))
    {
        agc_flag_set(AGC_FLAG_GAMDIFSW); /* first difference next */
        es->gamdot = 0;                  /* NOGAMDOT              */
    }
    else
    {
        es->gamdot = entry_sp_mp(entry_sp_sat((int32_t) es->gama - (int32_t) old), AGC_ENTRY_TCDU);
        g          = (es->gamdot > 0) ? -(int32_t) es->gamdot : (int32_t) es->gamdot; /* -|GAMDOT| */

        if (g + 5 > 0)
        { /* |GAMDOT| < 5 bits     */
            es->gamdot = 0;
        }
    }

    temproll = atup_angle(angles[0], es->roll_pip, es->roll_180, true);
    tempalfa = atup_angle(angles[2], es->alfa_pip, es->alfa_180, true);
    tempbeta = atup_angle(angles[1], es->beta_pip, es->beta_180, false);

    /* DCA REPOSADR / DXCH PHSNAME5: group 5 now restarts at REDOPOSE.     */
    agc_phase_change_var(010035, &v);
    agc_relint();

    if (!AGC_ASSERT(agc_entry_state()->posexit < AGC_ENTRY_POSEXIT_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_redopose_tail(ctx));
}

/**********************************************************************
 * REDOPOSE as the group 5 restart job (p. 888).
 *
 * FUNCTION NAME: agc_job_redopose
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_redopose(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_redopose_tail(ctx));
}

/**********************************************************************
 * REFAZE10 as the group 5 restart job (REENTRY_CONTROL p. 847).
 *
 * FUNCTION NAME: agc_job_reestablish_entry_sequencer
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_reestablish_entry_sequencer(agc_job_ctx_t *ctx)
{
    bool         disp = false; /* OVERNOUT display wanted */
    agc_status_t st;           /* REFAZE10 status */

    st = agc_entry_reestablish_sequencer(&disp);

    if (st == AGC_STATUS_OK && disp)
    {
        st = entry_endexit(ctx, agc_entry_state());
    }

    if (!AGC_ASSERT(agc_entry_state()->gotoaddr < AGC_ENTRY_GOTO_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? entry_servnout() : st); /* SERVNOUT */
}
