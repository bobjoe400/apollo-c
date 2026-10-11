/*
 * burn.c -- P40-P47.agc, Comanche055 pp. 693-736: the CSM thrusting
 * programs P40 (SPS), P41 (RCS) and P47 (thrust monitor): P40CSM, P41CSM,
 * P40PVA/P40SXTY (S40.1, S40.2,3, R60), P40TTOG/P40GMB (gimbal test and
 * trim), P40S/SV (time to TIG-30), TIGBLNK, TIGAVEG (TIG-30), TIG-5,
 * TIG-0, IGNITION, PREPTVC, DOTVCON, DOSTRULL, IMPLBURN, ENGINOFF,
 * DOSPSOFF, DOTVCRCS, SPSOFF/MASSBACK, TVCZAP, POSTBURN, P40RCS, TIGNOW,
 * POST41, SETMINDB/SETMAXDB, P41/DSP, P40CNV85, CALCN85, P47CSM, TIGON,
 * CALCN83, P47BODY, UPDATEVG/SETUP.9, STEERING, CLOKTASK, CLOCKJOB,
 * DYNDISP, the V97/V99 engine-fail and engine-enable responses, the
 * Comanche S40.8 and S40.9, and S40.6 (gimbal drive test / trim). S41.2,
 * S40.14 and RCSDAPON are the DAP lane's (agc_csm_dap.h). DAPFIG (V46).
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

/* FENG 2DEC 9.1188544 B-7: SPS thrust 20 500 lb in m.newton (10^4 N).     */
static const agc_dp_t FENG = 19123624;

/* FRCS2 2DEC .087437837 B-7: RCS ullage 199.6 COS 10 lb, 2 jets.         */
static const agc_dp_t FRCS2 = 183370;

/* SEC29.96 2DEC 2996, SEC24.96 DEC 2496, 18SEC, 5SEC: centiseconds.        */
static const agc_dp_t SEC29_96 = 2996;
#define SEC24_96              2496
#define SEC18                 1800
#define SEC5                  500
#define GIMBAL_TEST_CHECKLIST 0204 /* V50N25 R1 00204: gimbal drive test   */

/* NVWORD1 values that display V06N40, V16N40, V06N85, V16N85, V16N83.     */
agc_sp_t agc_csm_vn(uint8_t verb, uint8_t noun)
{
    return ((agc_sp_t) agc_dsky_vn(verb, noun));
}

/* P41/P40 (p.696): MODREG odd (41) takes the P41 exit.                    */
bool agc_csm_is_rcs_burn(void)
{
    agc_sp_t mm = agc_erase_modreg();

    if (!AGC_ASSERT(mm >= 0))
    {
        return (false);
    }

    return ((mm & 1) != 0);
}

/* P40S/F .. P40PVA .. P40SXTY (p.693-694): NOMTIG, IMU status, thrust
 * direction and preferred attitude, narrow deadband, attitude maneuver.   */
static agc_status_t burn_setup_with_thrust(agc_job_ctx_t *ctx, agc_dp_t f)
{
    agc_guid_t *g = agc_guid_state();

    g->burn.f       = f;
    g->burn.nomtig  = agc_erase_tig();        /* ORIGINAL TIG FOR S40.9 */
    agc_status_t sw = agc_imu_status_check(); /* IMU STATUS CHECK       */

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    g->burn.weight_g = agc_dp_from_sp(agc_csm_dap_com()->weight_g);
    agc_flag_set(AGC_FLAG_FIRSTFLG); /* S40.1: SET FIRSTFLG    */
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_guid_nav_set_resume(ctx->id);
    agc_status_t st = agc_guid_initial_velocity_to_gain(&ip, AGC_VEHICLE_CSM); /* VGTIG, UT       */

    if (st == AGC_STATUS_OK)
    {
        const agc_csm_dapcom_t *com = agc_csm_dap_com();

        st = agc_guid_preferred_thrust_attitude_csm(&ip, agc_flag_test(AGC_FLAG_ENG2FLAG), com->pactoff, com->yactoff);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_set(AGC_FLAG_PFRATFLG);
    agc_inhint();
    agc_csm_dap_setmindb(); /* NARROW DEADBAND        */
    agc_relint();

    return (agc_imu_attitude_maneuver(ctx, AGC_JOB_SPS_BURN_AFTER_MANEUVER)); /* R60CSM                 */
}

/* P40CSM (p.693): SPS. CSTEER = 0 for an external delta-V burn, else
 * ECSTEER; F = FENG.                                                      */
agc_status_t agc_job_sps_burn(agc_job_ctx_t *ctx)
{
    agc_guid_t *g = agc_guid_state();

    agc_flag_clear(AGC_FLAG_ENG2FLAG);
    g->burn.csteer = agc_flag_test(AGC_FLAG_XDELVFLG) ? 0 : g->burn.ecsteer;

    return (burn_setup_with_thrust(ctx, FENG));
}

/* P41CSM (p.696): RCS. CSTEER = 0; F = 2-jet (NJETSFLG) or 4-jet thrust.  */
agc_status_t agc_job_rcs_burn_csm(agc_job_ctx_t *ctx)
{
    agc_guid_t *g = agc_guid_state();

    agc_flag_set(AGC_FLAG_ENG2FLAG); /* SET FOR RCS            */
    g->burn.csteer = 0;
    agc_dp_t f     = agc_flag_test(AGC_FLAG_NJETSFLG) ? FRCS2 : 2 * FRCS2;

    if (!AGC_ASSERT(f > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (burn_setup_with_thrust(ctx, f));
}

/* P40SXTY after R60 (p.694): NBRCYCLS = -1, TIMRFLAG; P41 goes to P41/DSP,
 * P40 to P40TTOG and the gimbal test question (V50N25 204).               */
agc_status_t agc_job_sps_burn_after_maneuver(agc_job_ctx_t *ctx)
{
    agc_guid_t *g = agc_guid_state();

    g->burn.nbrcycls = -1;           /* FOR UPDATEVG           */
    agc_flag_set(AGC_FLAG_TIMRFLAG); /* ALLOW CLOCKTASK        */

    if (agc_csm_is_rcs_burn())
    {
        return (agc_csm_rcs_burn_display(ctx));
    }

    g->csm.nvword1 = agc_csm_vn(6, 40); /* P40TTOG                */
    agc_interp_t ip;

    agc_interp_init(&ip);
    g->burn.vgdisp       = agc_vec_abval(&ip, agc_erase_vgtig());
    g->avg.dvtotal       = 0;
    g->avg.avgexit       = AGC_JOB_STEERING; /* STEERADS               */
    agc_dsky_cont_t cont = {AGC_JOB_POST41, AGC_JOB_GIMBAL_TEST_AND_TRIM, AGC_JOB_GIMBAL_TRIM_ONLY};

    return (agc_dsky_display(ctx, AGC_DSKY_GOPERF1, GIMBAL_TEST_CHECKLIST, &cont)); /* P40GMB  */
}

/* TRIMONLY / TST,TRIM (p.694): S40.6 in 10 ms (MRKRTMP -1 trim only, +1
 * drive test and trim; CNTR = +0 normal entry), wait 5 or 18 s.          */
static agc_status_t trim_common(agc_job_ctx_t *ctx, agc_sp_t mrkrtmp)
{
    agc_guid_t *g = agc_guid_state();

    g->csm.mrkrtmp          = mrkrtmp;
    agc_csm_dap_tvc()->cntr = 0; /* +0 SAYS NORMAL ENTRY   */
    agc_status_t st         = agc_waitlist_call(1, AGC_TASK_GIMBAL_TEST_TRIM);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_svc_delayjob((mrkrtmp > 0) ? SEC18 : SEC5, AGC_JOB_PREBURN_START_CLOCK));
}

agc_status_t agc_job_gimbal_trim_only(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (trim_common(ctx, -1)); /* CS BIT1                */
}

agc_status_t agc_job_gimbal_test_and_trim(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (trim_common(ctx, 1)); /* CAF BIT1               */
}

/* After the DELAYJOB (p.694): 6.2 (PRE40.6 -0 cs, CLOKTASK 1 s) and 4.23
 * (P40S/SV, PRIO12), then P40S/RS.                                        */
agc_status_t agc_job_preburn_start_clock(agc_job_ctx_t *ctx)
{
    agc_phase_change2((uint16_t) 040026, (uint16_t) 000234, NULL);

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_csm_preburn_start_clock(ctx));
}

/* P40S/RS (p.694): CLOKTASK in 10 ms, then P40S/SV.                        */
agc_status_t agc_csm_preburn_start_clock(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_waitlist_call(1, AGC_TASK_CLOKTASK);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_preburn_state_vector(ctx));
}

/* P40S/SV / P40SNEWM / P40SET (p.694-695): integrate the state to 29.96 s
 * before TIG (MIDTOAV1); if that took too long, slip TIG to 29.96 s after
 * the integrated time. TIGBLNK by LONGCALL 5 s before TIG-30 (4.21).      */
agc_status_t agc_job_preburn_state_vector(agc_job_ctx_t *ctx)
{
    agc_guid_t             *g = agc_guid_state();
    agc_integ_midtoav_out_t mo;
    agc_interp_t            ip;

    agc_interp_init(&ip);
    agc_status_t st = agc_integ_midtoav1(&ip, agc_integ_shared_ctx(), agc_erase_tig() - SEC29_96, ctx->id, &mo);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_store_midtoav(&mo);
    agc_dp_t dt = mo.dt;

    if (mo.slipped)
    { /* P40SNEWM               */
        agc_erase_set_tig(g->avg.piptime1 + SEC29_96);
    }

    agc_erase_set_p40tmp(dt - SEC5); /* DELTA T TO PREREAD - 5 S */

    if (!AGC_ASSERT(agc_erase_p40tmp() >= 0))
    {
        agc_erase_set_p40tmp(1);
    }

    st = agc_waitlist_longcall((uint32_t) agc_erase_p40tmp(), AGC_TASK_TIGBLNK);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change(4, (uint16_t) 020214);

    return (AGC_STATUS_OK); /* 4.21 = TIGBLNK         */
}

/* P40BLNKR (p.695): blank the DSKY (not restart protected).               */
agc_status_t agc_job_preburn_blank_dsky(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0U, NULL));
}

/* TIGBLNK (p.697): task 35 s before TIG: TIGAVEG in 5 s, CLOCKJOB display
 * off, DSKY blanked, 4.37.                                                */
agc_status_t agc_task_tigblnk(void)
{
    agc_status_t st = agc_waitlist_call(SEC5, AGC_TASK_TIGAVEG);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_state()->csm.nvword1 = 0; /* DISABLE HERE           */
    agc_status_t sw               = agc_exec_novac(AGC_PRIO14, AGC_JOB_PREBURN_BLANK_DSKY);

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    if (!AGC_ASSERT(agc_guid_state()->csm.nvword1 == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_newphase(4, 037, true);

    return (AGC_STATUS_OK); /* 4.37 = TIGAVEG         */
}

/* PRECHECK (p.699): start the average-G loop (PREREAD) unless SERVICER is
 * already running (restart).                                              */
agc_status_t agc_task_precheck(void)
{
    if (agc_phase_get(5) != 0U)
    {
        return (AGC_STATUS_OK); /* DON'T START ANOTHER    */
    }

    if (!AGC_ASSERT(agc_phase_get(5) == 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((agc_state_vehicle() == AGC_VEHICLE_CSM) ? agc_task_preread_csm() : agc_task_preread_lm());
}

/* TIGAVEG = TIG-30 (p.699): P40: V06N40 back on, TIG-5 in 24.96 s, 4.6;
 * P41 (P41REDSP): V16N85, TTG/0 in 29.96 s, 4.4. Then PRECHECK.          */
agc_status_t agc_task_tigaveg(void)
{
    agc_guid_t  *g = agc_guid_state();
    agc_status_t st;

    if (agc_csm_is_rcs_burn())
    {
        g->csm.nvword1 = agc_csm_vn(16, 85); /* REDISPLAY NONFLASHING  */
        st             = agc_waitlist_call((uint16_t) SEC29_96, AGC_TASK_TTG_0);
        agc_phase_newphase(4, 4, true);
    }
    else
    {
        g->csm.nvword1 = agc_csm_vn(6, 40); /* UNBLANK DISPLAY        */
        st             = agc_waitlist_call(SEC24_96, AGC_TASK_TIG_5);
        agc_phase_newphase(4, 6, true);     /* 4.6 = TIG-5, PRECHECK  */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_task_precheck());
}

/* TTG/0 (p.696): P41 at TIG: TIGNOW job, CLOKTASK stopped.                */
agc_status_t agc_task_ttg_0(void)
{
    agc_status_t st = agc_exec_novac(AGC_PRIO20, AGC_JOB_TIGNOW);

    agc_flag_clear(AGC_FLAG_TIMRFLAG); /* P40CLK                 */

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_TIMRFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* TIG-5 (p.699): TIG-0 in 5 s, V99 flash, S40.13 (TIMEBURN) at PRIO20.   */
agc_status_t agc_task_tig_5_csm(void)
{
    agc_status_t st = agc_waitlist_call(SEC5, AGC_TASK_TIG_0);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_state()->csm.nvword1 = AGC_CSM_NV_ENGINE_ENABLE_REQUEST; /* WILL CAUSE V99 FLASH   */
    agc_phase_change2((uint16_t) 040074, (uint16_t) 000033, NULL);

    if (!AGC_ASSERT(agc_guid_state()->csm.nvword1 == AGC_CSM_NV_ENGINE_ENABLE_REQUEST))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_findvac(AGC_PRIO20, AGC_JOB_TIMEBURN));
}

/* TIG-0 (p.700): IGNFLAG set; ignite now if the crew has already answered
 * the V99 (ASTNFLAG), else wait for V99P.                                 */
agc_status_t agc_task_tig_0_csm(void)
{
    agc_flag_set(AGC_FLAG_IGNFLAG);

    if (!agc_flag_test(AGC_FLAG_ASTNFLAG))
    {
        return (AGC_STATUS_OK); /* WAIT FOR V99P          */
    }

    agc_guid_state()->csm.nvword1 = agc_csm_vn(6, 40); /* CLEAR THE V99         */
    agc_phase_change(4, (uint16_t) 000614);            /* 4.61 = IGNITION        */

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_IGNFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_ignition_csm());
}
