/*
 * programs.c -- P61-P67.agc (Comanche055, pp. 789-818): the entry programs.
 *
 *   P61  EMS initialisation data (N61 target, N60 GMAX/VPRED/GAMMAEI,
 *        N63 RTGO/VIO/TTE), then P62
 *   P62  CM/SM separation (N41 checklist), entry DAP on, entry attitude
 *   P63  entry initialisation, waits for .05 g (STARTENT via CM/POSE)
 *   P64-P67 mode/display changes called from the steering (reentry.c)
 *   S61.1 state vector and IMU orientation check, S61.2 the N60/N63
 *   predictions, S62.3 the desired gimbal angles for N22.
 *
 * Every point where the AGC job waits for the crew (GOFLASH/GOFLASHR,
 * GOPERF1R) or sleeps (DELAYJOB, waitlist) becomes a separate job entry
 * (docs/TRANSLATION_GUIDE.md section 4). Displays use agc_dsky_display.
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
#include "agc/io/dsky.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"

/* Verb/noun codes (VN pseudo-op: verb, noun in decimal digits).          */
#define VN_ENTRY_PREDICTION_DISPLAY 6, 60
#define VN_ENTRY_TARGET_DISPLAY     6, 61
#define ENTRY_OCT41                 041 /* GOPERF1R checklist: CM/SM separation */
#define ENTRY_BIT14                 ((agc_sp_t) 020000) /* .5 rev = 180 deg (ROLLC hi word)  */

/* P61/P62 module erasables (EBANK6/7).                                   */
static agc_job_id_t imu_check_return_job =
    AGC_JOB_NONE;                      /* 60GENRET return point  */
static agc_sp_t  s61dt;                /* S61DT waitlist dt for 4.43, cs      */
static agc_dp_t  entry_preparation_mm; /* MM: PIPTIME of RONE/VONE, cs B28    */
static agc_vec_t urone;                /* URONE  UNIT(RONE)/2                 */
static agc_vec_t urh;                  /* URH    rotated UR/2 (FISHCALC)      */
static agc_tff_t
    entry_preparation_tff; /* TFF push list (RONE, VONE, ...)     */

agc_sp_t agc_erase_s61dt(void)
{
    if (!AGC_ASSERT(s61dt >= 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(s61dt <= 16383))
    {
        return (0); /* no valid result */
    }

    return (s61dt);
}

/* Request a display; "R" kinds return at once (the job goes on).        */
static agc_status_t entry_flash(agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                                uint8_t verb, uint8_t noun, agc_job_id_t term,
                                agc_job_id_t proceed, agc_job_id_t enter)
{
    const agc_dsky_cont_t cont = {term, proceed, enter};

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(verb < 100 && noun < 100))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont));
}

/* NEWMODEX (MM) and the entry display ENTRYVN used by OVERNOUT.          */
static agc_status_t entry_mode(agc_entry_state_t *es, agc_sp_t mm, uint8_t verb,
                               uint8_t noun)
{
    if (!AGC_ASSERT(es != NULL) || !AGC_ASSERT(mm >= 61 && mm <= 67))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    es->entry_verb = verb;
    es->entry_noun = noun;

    return (agc_phase_newmode(mm));
}

/* HEADSUP -> ROLLC: +1 (lift down) 180 deg, -1 (lift up) 0 deg.         */
static agc_sp_t headsup_roll(const agc_entry_state_t *es)
{
    if (!AGC_ASSERT(es != NULL))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(es->headsup >= -1 && es->headsup <= 1))
    {
        return (0); /* no valid result */
    }

    return ((es->headsup > 0) ? ENTRY_BIT14 : 0);
}

/* ======================================================================
 * P61 (pp. 789-791)
 * ==================================================================== */

/* P61 (p. 789): lock out extended verbs, HEADSUP = -1, S61.1.            */
agc_status_t agc_job_entry_preparation(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_svc_set_extvbact(020000); /* CA BIT14 TS EXTVBACT */
    es->headsup          = -1;           /* preload              */
    imu_check_return_job =
        AGC_JOB_ENTRY_PREPARATION_TARGET_DISPLAY; /* QXCH 60GENRET        */

    return (agc_job_entry_imu_check(ctx));
}

/* P61 display (p. 789): V06N61 LAT(SPL) LNG(SPL) HEADSUP, GOFLASHR;
 * the job goes on to P61.3.                                              */
agc_status_t agc_job_entry_preparation_target_display(agc_job_ctx_t *ctx)
{
    agc_status_t st =
        entry_flash(ctx, AGC_DSKY_GOFLASHR, VN_ENTRY_TARGET_DISPLAY,
                    AGC_JOB_GOTOPOOH, AGC_JOB_ENTRY_PREPARATION_COMPUTE,
                    AGC_JOB_ENTRY_PREPARATION_TARGET_DISPLAY);

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_entry_display_protect(ctx) : st);
}

/* P61.3 (p. 790): restart group 4 at the display (4.1), ENDOFJOB.        */
agc_status_t agc_job_entry_display_protect(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(4, 000014);

    return (AGC_STATUS_OK);
}

/* NEWRNVN (p. 790): RONE, VONE, URONE, UNI at PIPTIME, and STARTEN1.     */
static agc_status_t entry_preparation_newrnvn(agc_interp_t      *ip,
                                              agc_entry_state_t *es)
{
    agc_status_t st;

    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    entry_preparation_mm = agc_erase_piptime(); /* save PIPTIME in MM   */
    st                   = entry_starten1(ip, es);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    entry_preparation_tff.rone = agc_erase_rn();
    urone                      = agc_vec_unit(ip, entry_preparation_tff.rone,
                                              NULL, NULL);
    entry_preparation_tff.vone = agc_erase_vn();
    es->uni                    = agc_vec_unit(
        ip, agc_vec_cross(ip, entry_preparation_tff.vone, urone), NULL, NULL);

    if (!AGC_ASSERT(entry_preparation_mm == agc_erase_piptime()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* P61.4 (p. 790): ROLLC from HEADSUP, the target at PIPTIME (retried if
 * SERVICER updated RN, VN meanwhile), S61.2, then P61.1.                 */
agc_status_t agc_job_entry_preparation_compute(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state();
    agc_interp_t      *ip = entry_ip();
    agc_status_t       st = AGC_STATUS_OK;
    int                n;

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    es->rollc = agc_dp_pack(headsup_roll(es), 0); /* ZL DXCH ROLLC */

    for (n = 0; n < 4; n++)
    { /* DUMPP61 bound  */
        st = entry_preparation_newrnvn(ip, es);

        if (st != AGC_STATUS_OK ||
            agc_dp_sub(ip, entry_preparation_mm, agc_erase_piptime()) >= 0)
        {
            break;
        }
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_entry_ems_data();
    }

    return ((st == AGC_STATUS_OK) ?
            agc_job_entry_preparation_prediction_display(ctx) :
            st);
}

/* P61.1 (p. 790): V06N60 GMAX VPRED GAMMAEI, GOFLASH.                   */
agc_status_t agc_job_entry_preparation_prediction_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_clearmrk();

    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_flash(ctx, AGC_DSKY_GOFLASH, VN_ENTRY_PREDICTION_DISPLAY,
                        AGC_JOB_GOTOPOOH, AGC_JOB_ENTRY_PREPARATION_EMS_DISPLAY,
                        AGC_JOB_ENTRY_PREPARATION_PREDICTION_DISPLAY));
}

/* P61.2 (pp. 790-791): TTE corrected for the time spent at the display,
 * V06N63 RTGO VIO TTE, GOFLASH; PROCEED falls into P62.                  */
agc_status_t agc_job_entry_preparation_ems_display(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state();
    agc_interp_t      *ip = entry_ip();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    es->tte = agc_dp_add(
        ip, agc_dp_sub(ip, agc_rtb_loadtime(), entry_preparation_mm), es->tte1);

    return (entry_flash(ctx, AGC_DSKY_GOFLASH, 6, 63, AGC_JOB_GOTOPOOH,
                        AGC_JOB_CM_SM_SEPARATION_NEWMODE,
                        AGC_JOB_ENTRY_PREPARATION_EMS_DISPLAY));
}

/* ======================================================================
 * P62 (pp. 792-794)
 * ==================================================================== */

/* P62 -3 (p. 792): from P61: NEWMODEX 62, DNLSTCOD = 1 (entry list).    */
agc_status_t agc_job_cm_sm_separation_newmode(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_phase_newmode(62);

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (st == AGC_STATUS_OK)
    {
        agc_telem_select_list(AGC_ENTRY_DNLSTCOD_ENTRY);
        st = AGC_STATUS_OK;
    }

    if (!AGC_ASSERT(agc_erase_modreg() == 62 || st != AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_cm_sm_separation(ctx) : st);
}

/* P62 (p. 792): S61.1, then the rest of P62.                             */
agc_status_t agc_job_cm_sm_separation(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    imu_check_return_job = AGC_JOB_CM_SM_SEPARATION_AFTER_CHECK;

    if (!AGC_ASSERT(
            imu_check_return_job == AGC_JOB_CM_SM_SEPARATION_AFTER_CHECK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_entry_imu_check(ctx));
}

/* P62 after S61.1 (p. 792): POSEXIT = P62.3 (desired .05 g gimbal angles
 * without display), CM/DAPIC (which goes on to P62.2).                   */
agc_status_t agc_job_cm_sm_separation_after_check(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    es->posexit = AGC_ENTRY_POSEXIT_DESIRED_GIMBALS;

    return (agc_entry_dap_ic(ctx));
}

/* P62.2 (pp. 792-793): AVEGEXIT = CM/POSE, GOPERF1R 41 (request CM/SM
 * separation); the job goes on at P61.3.                                 */
agc_status_t agc_job_cm_sm_separation_request(agc_job_ctx_t *ctx)
{
    agc_status_t st;

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_state()->avg.avgexit = AGC_JOB_CM_POSE; /* DXCH AVEGEXIT */
    {
        const agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH,
                                      AGC_JOB_CM_SM_SEPARATION_PROCEED,
                                      AGC_JOB_CM_SM_SEPARATION_REQUEST};

        st = agc_dsky_display(ctx, AGC_DSKY_GOPERF1R, ENTRY_OCT41, &cont);
    }

    if (!AGC_ASSERT(agc_guid_state()->avg.avgexit == AGC_JOB_CM_POSE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_entry_display_protect(ctx) : st);
}

/* P62.2 +3 (p. 793): PROCEED: CM/DAPON (NODOFLAG is set there).          */
agc_status_t agc_job_cm_sm_separation_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_cm_dapon(ctx));
}

/* P62.1 (p. 793): V06N61 again (terminate and enter redisplay).          */
agc_status_t agc_job_cm_sm_separation_target_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_flash(ctx, AGC_DSKY_GOFLASH, VN_ENTRY_TARGET_DISPLAY,
                        AGC_JOB_CM_SM_SEPARATION_TARGET_DISPLAY,
                        AGC_JOB_CM_SM_SEPARATION_END_HOLD,
                        AGC_JOB_CM_SM_SEPARATION_TARGET_DISPLAY));
}

/* P62.1 PROCEED (pp. 793-794): end the attitude hold: ROLLC from
 * HEADSUP, ALFACOM = ALFAPAD, BETACOM = 0, P63FLAG = +1, N22 entry
 * display. P63 directly when CMDAPMOD is -1 or +0, else wait for WAKEP62.*/
agc_status_t agc_job_cm_sm_separation_end_hold(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state();
    agc_phase_var_t    v  = {AGC_RESTART_JOB, AGC_JOB_CM_SM_SEPARATION_END_HOLD,
                             AGC_TASK_NONE, 0,
                             false, 0U};

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change_var(004024, &v); /* ENTRYVN display  */
    es->rollc =
        agc_dp_pack(headsup_roll(es), agc_dp_low(es->rollc));           /* TS */
    es->alfacom                   = es->alfapad; /* trim, negative   */
    es->betacom                   = 0;
    es->wake_separation_interlock = 1; /* EXDAP2 -> +0     */
    es->entry_verb                = 6; /* V06N22           */
    es->entry_noun                = 22;
    agc_flag_set(AGC_FLAG_ENTRYDSP);

    if (es->cmdapmod == AGC_ENTRY_DAPMOD_MINUS1 ||
        es->cmdapmod == AGC_ENTRY_DAPMOD_PLUS0)
    {
        return (agc_job_entry_initialization(ctx));
    }

    agc_phase_change(4, 000004);

    return (AGC_STATUS_OK); /* P63.1           */
}

/* WAKEP62 (p. 794): task from the entry DAP when ALFA nears trim.        */
agc_status_t agc_task_wake_cm_sm_separation(void)
{
    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    _Static_assert(AGC_ENTRY_PRIO13 > 0, "AGC constant");

    return (agc_exec_novac(AGC_ENTRY_PRIO13, AGC_JOB_ENTRY_INITIALIZATION));
}

/* ======================================================================
 * P63 .. P67 (pp. 795-802)
 * ==================================================================== */

/* P63 (p. 795): POSEXIT = STARTENT, N64 entry display, P63FLAG = -1,
 * flush the N22 display, disable group 4, ENDOFJOB.                      */
agc_status_t agc_job_entry_initialization(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state();
    agc_status_t       st;

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    st                            = entry_mode(es, 63, 6, 64); /* G VI RTOSPLASH  */
    es->posexit                   = AGC_ENTRY_POSEXIT_STARTENT;
    es->wake_separation_interlock = -1;
    agc_phase_change(4, 000004);
    const agc_status_t sd = agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0, NULL);

    if (sd != AGC_STATUS_OK)
    {
        return (sd); /* PRIOBORT 01502 or exec overflow: job abandoned */
    }

    agc_phase_change(4, 000004); /* P63.1           */

    return (st);
}

/* P64 (p. 797): MM 64, N74 (ROLLC VI D) at .05 g.                        */
agc_status_t entry_post_0_05g(agc_entry_state_t *es)
{
    if (!AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG__05GSW)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_mode(es, 64, 6, 74));
}

/* P65 (p. 798): MM 65, start the N69 job P65.1, restart 4.55 and 5.3,
 * GOTOADDR = UPCONTRL (the caller continues there, REFAZE10).            */
agc_status_t entry_upcontrol(agc_entry_state_t *es)
{
    agc_phase_var_t v = {
        AGC_RESTART_JOB, AGC_JOB_REFAZE10, AGC_TASK_NONE, 0, false, 0U
    };
    agc_status_t    st = agc_phase_newmode(65);

    if (!AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_exec_novac(AGC_ENTRY_PRIO13, AGC_JOB_ENTRY_UPCONTROL_DISPLAY);
    }

    agc_phase_change2(000554, 010035, &v);
    es->gotoaddr = AGC_ENTRY_GOTO_UPCONTRL;

    if (!AGC_ASSERT(es->gotoaddr == AGC_ENTRY_GOTO_UPCONTRL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* P65.1 (p. 798): entry display off, V16N69 ROLLC DL VL, GOFLASHR.       */
agc_status_t agc_job_entry_upcontrol_display(agc_job_ctx_t *ctx)
{
    agc_status_t st;

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_ENTRYDSP);
    st = entry_flash(
        ctx, AGC_DSKY_GOFLASHR, 16, 69, AGC_JOB_ENTRY_UPCONTROL_DISPLAY,
        AGC_JOB_ENTRY_UPCONTROL_PROCEED, AGC_JOB_ENTRY_UPCONTROL_DISPLAY);

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_ENTRYDSP)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_entry_display_protect(ctx) : st);
}

/* P65.1 PROCEED (p. 798): entry display back on, P63.1.                  */
agc_status_t agc_job_entry_upcontrol_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_ENTRYDSP);

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_ENTRYDSP)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(4, 000004);

    return (AGC_STATUS_OK);
}

/* KILLGRP4 (p. 801): disable group 4 (HUNTEST / N69 restarts).           */
agc_status_t entry_killgrp4(void)
{
    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    _Static_assert(AGC_RESTART_GROUPS >= 4, "AGC constant");
    agc_phase_change(4, 000004);

    return (AGC_STATUS_OK);
}

/* P66 (p. 799): MM 66, N22 trim gimbal angles; P66END.                   */
agc_status_t entry_ballistic(agc_entry_state_t *es)
{
    agc_status_t st = entry_mode(es, 66, 6, 22);

    if (!AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_ENTRYDSP);

    return ((st == AGC_STATUS_OK) ? entry_killgrp4() : st);
}

/* P67 (p. 800): MM 67, N66 ROLLC XRNGERR DNRNGERR; P66END.               */
agc_status_t entry_final_phase(agc_entry_state_t *es)
{
    agc_status_t st = entry_mode(es, 67, 6, 66);

    if (!AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_ENTRYDSP);

    return ((st == AGC_STATUS_OK) ? entry_killgrp4() : st);
}

/* P67.1 (p. 800): V16N67 RTOGO LAT LONG, GOFLASH (ENTER redisplays).     */
agc_status_t agc_job_entry_final_phase_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_flash(
                ctx, AGC_DSKY_GOFLASH, 16, 67, AGC_JOB_ENTRY_FINAL_PHASE_END,
                AGC_JOB_ENTRY_FINAL_PHASE_END,
                AGC_JOB_ENTRY_FINAL_PHASE_DISPLAY));
}

/* P67.1 end (pp. 800-801): entry DAP off (CM/DSTBY, GYMDIFSW), AVEGEXIT
 * = SERVEXIT, GOTOPOOH.                                                   */
agc_status_t agc_job_entry_final_phase_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_flag_clear(AGC_FLAG_CM_DSTBY);
    agc_flag_clear(AGC_FLAG_GYMDIFSW);
    agc_relint();
    agc_guid_state()->avg.avgexit = AGC_JOB_SERVEXIT; /* SERVCAD2 */

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_CM_DSTBY)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_gotopooh());
}

/* P67.2 (p. 802): present LAT, LONG, ALT of RN at PIPTIME (pad radius,
 * earth). P67.3 (SERVNOUT) is done by the caller.                        */
agc_status_t agc_entry_final_phase_latlong(void)
{
    const agc_latlong_opt_t opt = {false, false}; /* LUNA, ERAD clear */
    agc_entry_state_t      *es  = agc_entry_state();
    agc_navaux_padload_t    pad;
    agc_latlong_t           ll;
    agc_status_t            st;

    if (!AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_ERADFLAG);
    agc_flag_clear(AGC_FLAG_LUNAFLAG);
    st = agc_navaux_get_padload(&pad);

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_from_r(entry_ip(), &pad, opt, agc_erase_rn(),
                                agc_erase_piptime(), &ll);
    }

    if (st == AGC_STATUS_OK)
    {
        es->lat = ll.lat;
        es->lng = ll.lon;
        es->alt = ll.alt;
    }

    if (!AGC_ASSERT(st != AGC_STATUS_OK || agc_dp_abs(
                        es->lat) <= AGC_DP_QUARTER))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* ======================================================================
 * S61.1 (pp. 803-805)
 * ==================================================================== */

/* RETRN1 / 60GENRET: continue the program that called S61.1.            */
static agc_status_t imu_check_return(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(imu_check_return_job ==
                    AGC_JOB_ENTRY_PREPARATION_TARGET_DISPLAY ||
                    imu_check_return_job ==
                    AGC_JOB_CM_SM_SEPARATION_AFTER_CHECK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (imu_check_return_job == AGC_JOB_CM_SM_SEPARATION_AFTER_CHECK)
    {
        return (agc_job_cm_sm_separation_after_check(ctx));
    }

    return (agc_job_entry_preparation_target_display(ctx));
}

/* S61.1 after CALRB MIDTOAV2 (p. 804): the future state vector goes to
 * RN1 VN1 PIPTIME1; S61.1C runs at that time (restart 4.43).             */
agc_status_t agc_job_entry_imu_check_extrapolated(agc_job_ctx_t *ctx)
{
    agc_integ_midtoav_out_t out;
    agc_avg_g_t            *avg = &agc_guid_state()->avg;
    agc_status_t            st;

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    st = agc_integ_midtoav2(entry_ip(), agc_integ_shared_ctx(),
                            AGC_JOB_ENTRY_IMU_CHECK_EXTRAPOLATED, &out);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* asleep in INTSTALL     */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    avg->rn1      = out.rn1; /* RN1 VN1 PIPTIME1       */
    avg->vn1      = out.vn1;
    avg->piptime1 = out.piptime1;
    s61dt         = agc_dp_low(out.dt); /* CA MPAC +1 TS S61DT    */

    if (!AGC_ASSERT(s61dt > 0))
    {
        s61dt = 1;
    }

    st = agc_waitlist_call((uint16_t) s61dt, AGC_TASK_ENTRY_START_AVERAGE_G);
    agc_phase_change(4, 040434); /* 4.43 S61.1C            */

    return (st);                          /* ENDOFJOB               */
}

/* S61.1 (p. 803): R02BOTH; with AVERAGEG on, the IMU check S61.1A, else
 * bring the state vector forward and start AVERAGEG (S61.1C).            */
agc_status_t agc_job_entry_imu_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_imu_status_check(); /* IMU status check       */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_flag_test(AGC_FLAG_AVEGFLAG))
    {
        return (agc_job_entry_imu_axis_test(ctx));
    }

    if (!AGC_ASSERT(imu_check_return_job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_entry_imu_check_extrapolated(ctx));
}

/* S61.1C (p. 804): task at the state vector time: S61.1A job (4.45),
 * AVEGEXIT = SERVEXIT, PREREAD (5.41) starts AVERAGEG.                  */
agc_status_t agc_task_entry_start_average_g(void)
{
    agc_status_t st =
        agc_exec_findvac(AGC_ENTRY_PRIO13, AGC_JOB_ENTRY_IMU_AXIS_TEST);

    agc_guid_state()->avg.avgexit = AGC_JOB_SERVEXIT; /* SERVCAD1      */
    agc_phase_change2(000454, 000415, NULL);   /* 4.45, 5.41    */

    if (!AGC_ASSERT(agc_guid_state()->avg.avgexit == AGC_JOB_SERVEXIT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_task_preread_csm()); /* POSTJUMP      */
}

/* S61.1A / S61.1B / RETRN1-3 (pp. 804-805): the IMU Y axis must be
 * within 30 deg of UNIT(VN x RN): cos/2 + C(30)LIM overflows. -Y within
 * 30 deg: alarm 01427 (IMU reversed), else 01426 (unsatisfactory); both
 * show V05N09 and wait 10 s before returning.                            */
agc_status_t agc_job_entry_imu_axis_test(agc_job_ctx_t *ctx)
{
    agc_interp_t *ip       = entry_ip();
    agc_mat_t     refsmmat = agc_erase_refsmmat();
    agc_vec_t     y;
    agc_dp_t      c;
    uint16_t      code;

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_clear_ovf(ip); /* BOVB TCDANZIG */
    y = agc_mat_vec(ip, &refsmmat,
                    agc_vec_cross(ip, agc_erase_vn(), agc_erase_rn()));
    y = agc_vec_unit(ip, y, NULL, NULL);
    c = y.c[1]; /* COS(THETA)/2  */

    if (c >= 0)
    {
        agc_dp_add_ovf_only(ip, c, AGC_ENTRY_C30LIM); /* DAD C(30)LIM */

        if (agc_interp_test_ovf(ip))
        {
            return (imu_check_return(ctx)); /* RETRN1        */
        }

        code = 01426; /* RETRN3        */
    }
    else
    {
        agc_dp_add_ovf_only(ip, agc_dp_neg(c), AGC_ENTRY_C30LIM);
        code = agc_interp_test_ovf(ip) ? 01427 : 01426; /* S61.1B        */
    }

    agc_alarm(code);
    agc_status_t st = agc_dsky_display(ctx, AGC_DSKY_GODSPR, agc_dsky_vn(5,
                                                                         9),
                                       NULL);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(code == 01426 || code == 01427))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_svc_delayjob(1000,
                             AGC_JOB_ENTRY_IMU_CHECK_RETURN)); /* 10SECS        */
}

/* RETRN1 after the 10 s DELAYJOB.                                        */
agc_status_t agc_job_entry_imu_check_return(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(imu_check_return_job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (imu_check_return(ctx));
}

/* ======================================================================
 * S61.2 (pp. 806-816)
 * ==================================================================== */

/* TFF/TRIG (p. 815): .5 COS(THETA) = .5 - |ARG/(RN RTERM (1 + X))|,
 * returns .5 SIN(THETA) with the sign of ARG.                            */
static agc_dp_t tfftrig(agc_interp_t *ip, const agc_tff_t *t, agc_dp_t *cdelf2)
{
    agc_dp_t m;

    if (!AGC_ASSERT(t != NULL && cdelf2 != NULL))
    {
        return (0); /* no valid result */
    }

    m       = agc_dp_add(ip, agc_dp_shr(t->tffx, 1), AGC_DP_HALF);
    m       = agc_dp_mul(ip, agc_dp_mul(ip, m, t->nrmag), t->nrterm);
    m       = agc_dp_div(ip, t->tfftem, m);
    *cdelf2 = agc_dp_sub(ip, AGC_DP_HALF, agc_dp_abs(m));
    m       =
        agc_dp_add(ip, agc_dp_neg(agc_dp_sq(ip, *cdelf2)), AGC_DP_QUARTER);

    return (agc_dp_sign(agc_dp_sqrt(ip, m), t->tfftem));
}

/* FISHCALC (p. 812): URH = UR CDELF + UHOR SDELF; Fischer radius at the
 * latitude of URH (GETERAD), m B29.                                      */
static agc_dp_t fishcalc(agc_interp_t *ip, agc_dp_t sdelf2, agc_dp_t cdelf2)
{
    agc_vec_t h = agc_vec_cross(ip, urone, agc_entry_state()->uni);

    if (!AGC_ASSERT(agc_dp_abs(sdelf2) <= AGC_DP_HALF))
    {
        return (0); /* no valid result */
    }

    h   = agc_vec_shl(ip, agc_vec_scale(ip, h, sdelf2), 1);
    urh = agc_vec_add(ip, h, agc_vec_scale(ip, urone, cdelf2));

    return (agc_latlong_geterad(
                ip,
                agc_dp_shl(ip,
                           agc_vec_dot(ip, urh,
                                       agc_guid_state()->avg.unitw), 1)));
}

/* VRCALC (p. 816): ACOS(URH.RT) in revs.                                 */
agc_dp_t agc_entry_vrcalc(agc_vec_t u, agc_vec_t rt)
{
    agc_interp_t *ip = entry_ip();

    if (!AGC_ASSERT(ip != NULL))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(u.c[0] >= AGC_DP_MIN))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, u, rt), 2)));
}

/* DISPTARG (p. 816): target rotated through the time to go
 * (range KTETA1 - TTE1), range angle from URH.                           */
static agc_dp_t disptarg(agc_interp_t *ip, agc_entry_state_t *es, agc_dp_t rtgo)
{
    if (!AGC_ASSERT(ip != NULL && es != NULL))
    {
        return (0); /* no valid result */
    }

    es->dtearot =
        agc_dp_sub(ip, agc_dp_mul(ip, rtgo, AGC_ENTRY_KTETA1), es->tte1);
    entry_earrot(ip, es, false); /* EARROT2       */

    if (!AGC_ASSERT(es->dtearot >= AGC_DP_MIN))
    {
        return (0); /* no valid result */
    }

    return (agc_entry_vrcalc(urh, es->rt));
}

/* PREVGAM / VGAMCALC (pp. 813-814): vis-viva speed VGAM (m/cs B7) and
 * positive flight path angle (revs) at RTERM (B29).                      */
static agc_dp_t vgamcalc(agc_interp_t *ip, agc_tff_t *t, agc_dp_t rterm,
                         agc_dp_t *vgam)
{
    agc_dp_t prod;
    agc_dp_t d;
    agc_dp_t s;
    int      sh = t->nr - 8;

    if (!AGC_ASSERT(t->nr >= 0 && t->nr < 28))
    {
        return (0); /* no valid result */
    }

    t->rterm  = rterm;
    t->nrterm = agc_dp_shl(ip, rterm, t->nr); /* SL* 0,1       */
    prod      = agc_dp_mul(ip, t->nrmag, t->nrterm);
    d         = agc_dp_sub(ip, t->nrmag, t->nrterm);
    d         = (sh >= 0) ? agc_dp_shl(ip, d, sh) : agc_dp_shr_round(d, -sh);
    s         =
        agc_dp_sqrt(ip, agc_dp_sub(ip, agc_dp_div(ip, d, prod), t->tffvsq));
    *vgam = agc_dp_div(ip, s, t->rtmu);
    d     =
        agc_dp_div(ip, agc_dp_sqrt(ip, t->tffnp), agc_dp_mul(ip, s, t->nrterm));

    if (!AGC_ASSERT(s >= 0))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_acos(ip, agc_dp_shr(d, 1)));
}

/* GMAXCALC (p. 809): with VBAR = (V - 36 kft/s)/20 kft/s,
 * GMAX = (4/(1 + 4.8 VBAR^2))(GAM - 6.05 - 2.4 VBAR^2) + 10, >= 0,
 * as 100 GMAX B-14 (L/D = .3, bank 0).                                   */
agc_status_t agc_entry_gmaxcalc(agc_dp_t vgam, agc_dp_t gam, agc_dp_t *gmax)
{
    agc_interp_t *ip = entry_ip();
    agc_dp_t      vb;
    agc_dp_t      num;
    agc_dp_t      g;

    if (!AGC_ASSERT(gmax != NULL) || !AGC_ASSERT(vgam >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    vb = agc_dp_sq(ip,
                   agc_dp_div(ip, agc_dp_sub(ip, vgam, AGC_ENTRY_KFT36_PER_S),
                              AGC_ENTRY_KFT20_PER_S));  /* B-2 */
    num = agc_dp_add(ip, agc_dp_add(ip, agc_dp_mul(ip, vb, AGC_ENTRY_KR1), gam),
                     AGC_ENTRY_M6_05DEG);
    num = agc_dp_mul(ip, num, AGC_ENTRY_KR2);
    g   =
        agc_dp_add(ip, agc_dp_div(ip, vb, AGC_ENTRY_KR4), AGC_ENTRY_ONE_16TH);
    g     = agc_dp_add(ip, agc_dp_div(ip, num, g), AGC_ENTRY_KR3);
    *gmax = (g >= 0) ? g : 0; /* HI6ZEROS     */

    return (AGC_STATUS_OK);
}

/* S61.2 first part (p. 808): TTE to EMSALT and RTGO from EMSALT.        */
static agc_status_t ems_altitude_range(agc_interp_t *ip, agc_entry_state_t *es,
                                       agc_dp_t *eradm)
{
    agc_dp_t     c = 0;
    agc_dp_t     t = 0;
    agc_dp_t     a;
    agc_status_t st;

    entry_preparation_tff.rtmu =
        (agc_dp_sub(ip, es->emsalt, AGC_ENTRY_KFT290) >= 0) ?
        AGC_ENTRY_RTMU_INV_LUNENT :
        AGC_ENTRY_RTMU_INV_EARTH;
    st = agc_tff_conic(ip, &entry_preparation_tff);

    if (st == AGC_STATUS_OK)
    {
        st = agc_tff_calctff(ip, &entry_preparation_tff, AGC_ENTRY_RTRIAL, &t);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    a      = tfftrig(ip, &entry_preparation_tff, &c);
    *eradm = fishcalc(ip, a, c);
    st     = agc_tff_calctff(ip, &entry_preparation_tff,
                             agc_dp_add(ip, *eradm, es->emsalt), &t);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    es->tte1 = agc_dp_neg(t); /* countdown     */
    es->tte  = es->tte1;
    a        = tfftrig(ip, &entry_preparation_tff, &c);
    *eradm   = fishcalc(ip, a, c);
    a        = agc_entry_vrcalc(urh, es->rt);
    a        = disptarg(ip, es, a);
    es->rtgo = disptarg(ip, es, a);

    if (!AGC_ASSERT(es->rtgo >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* S61.2 (pp. 806-811): N60 GMAX VPRED GAMMAEI and N63 RTGO VIO TTE.     */
agc_status_t agc_entry_ems_data(void)
{
    agc_interp_t      *ip    = entry_ip();
    agc_entry_state_t *es    = agc_entry_state();
    agc_dp_t           eradm = 0;
    agc_dp_t           vgam  = 0;
    agc_dp_t           c     = 0;
    agc_dp_t           t     = 0;
    agc_dp_t           g;
    agc_status_t       st = ems_altitude_range(ip, es, &eradm);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g = vgamcalc(ip, &entry_preparation_tff, entry_preparation_tff.rterm,
                 &vgam);       /* VIO */
    es->vio = agc_dp_add(
        ip, agc_dp_div(ip, AGC_ENTRY_VEMSCON, agc_dp_mul(ip, g, vgam)), vgam);
    g = vgamcalc(ip, &entry_preparation_tff,
                 agc_dp_add(ip, eradm, AGC_ENTRY_KFT300), &vgam);
    st = agc_entry_gmaxcalc(vgam, g, &es->gmax);

    if (st == AGC_STATUS_OK)
    { /* 2nd ERAD iter. */
        st = agc_tff_calctff(ip, &entry_preparation_tff,
                             agc_dp_add(ip, eradm, AGC_ENTRY_KFT400), &t);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g     = tfftrig(ip, &entry_preparation_tff, &c);
    eradm = fishcalc(ip, g, c);
    g     = vgamcalc(ip, &entry_preparation_tff,
                     agc_dp_add(ip, eradm, AGC_ENTRY_KFT400), &vgam);
    es->gammaei =
        agc_dp_pack(agc_dp_high(agc_dp_neg(g)), agc_dp_high(es->rtgo));
    es->vpred = vgam;

    if (!AGC_ASSERT(es->vpred >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* ======================================================================
 * S62.3 (pp. 817-818)
 * ==================================================================== */

/* S62.3: desired body triad for trimmed flight at ROLLC and ALFAPAD:
 *   UYD = UYA COS(ROLLC) + UZA SIN(ROLLC),
 *   UXD = UNIT(UYD x UXA) SIN(ALFATRIM) + UXA COS(ALFATRIM), UZD = UXD x UYD,
 * then CALCGA (CPHIFLAG clear: CPHI = THETAD) for the N22 display.       */
agc_status_t agc_entry_desired_gimbal_angles(void)
{
    agc_interp_t      *ip   = entry_ip();
    agc_entry_state_t *es   = agc_entry_state();
    agc_dp_t           alfa = agc_dp_shr(agc_dp_from_sp(es->alfapad), 1); /* revs */
    agc_mat_t          nb;
    agc_mat_t          sm     = agc_erase_refsmmat();
    uint16_t           cdu[3] = {0, 0, 0};
    agc_status_t       st;
    int                i;

    nb.row[1] =
        agc_vec_add(ip, agc_vec_scale(ip, es->uya, agc_dp_cos(ip, es->rollc)),
                    agc_vec_scale(ip, es->uza, agc_dp_sin(ip, es->rollc)));
    nb.row[1] = agc_vec_shl(ip, nb.row[1], 1); /* YNB */
    nb.row[0] = agc_vec_shl(ip, agc_vec_cross(ip, nb.row[1], es->uxa), 1);
    nb.row[0] =
        agc_vec_add(ip, agc_vec_scale(ip, nb.row[0], agc_dp_sin(ip, alfa)),
                    agc_vec_scale(ip, es->uxa, agc_dp_cos(ip, alfa)));
    nb.row[0] = agc_vec_shl(ip, nb.row[0], 1); /* XNB */
    nb.row[2] = agc_vec_shl(ip, agc_vec_cross(ip, nb.row[0], nb.row[1]), 1);
    agc_flag_clear(AGC_FLAG_CPHIFLAG);
    st = agc_imu_calcga(ip, &nb, &sm, cdu);

    for (i = 0; i < 3; i++)
    {
        int32_t w = (int32_t) (cdu[i] & 077777u);

        es->cphi[i] = (agc_sp_t) ((w >= 16384) ? w - 32768 : w); /* /180 */
    }

    if (!AGC_ASSERT(es->cphi[0] >= -16384))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}
