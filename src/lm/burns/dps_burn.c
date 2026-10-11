/*
 * dps_burn.c -- P40-P47.agc, Luminary099 pp. 752-784: the LM thrusting
 * programs P40 (DPS), P41 (RCS), P42 (APS) and P47 (thrust monitor):
 * P40LM, P41LM, P42LM, P40IN/P40SXT4/P41MANU (S40.1, S40.2,3, R60LEM),
 * POSTBURN, TIGNOW, TERM40, BLNKWAIT/REDO6.7/DYNMDISP, CALCN85, P47LM,
 * STARTP47, CALCN83, P47BOD, P47BODY, IMPLBURN, ENGOFTSK, ENGINOFF ..
 * ENGINOF4, COASTSET, UPDATEVG, STEERING, GETDT, the Luminary S40.8,
 * S40.9 and S40.13 and TRIMGIMB (RASTEER1 is in dps_burn_rasteer.c).
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/conic.h"
#include "agc/programs/rendezvous.h"

#define ONE_SEC   100
#define HALF_SEC  50
#define TRIM_WAIT 6000 /* DEC 6000: one minute to full + trim       */
/* TRIMGIMB channel 12 masks as the listing names them (p.781): EBANK5
 * 02400 = bits 9, 11 "+ PITCH, + ROLL"; PRIO5 05000 = bits 10, 12
 * "- PITCH, - ROLL"; BIT10 shuts off pitch, BIT12 roll. agc_channels.h
 * names these bits TRIMPITCHM/TRIMROLLM (9, 11) and TRIMPITCHP/TRIMROLLP
 * (10, 12) -- the opposite sense; see the module note, Open questions.   */
#define TRIM_PLUS  ((uint16_t) (AGC_CH012_TRIMPITCHM | AGC_CH012_TRIMROLLM))
#define TRIM_MINUS ((uint16_t) (AGC_CH012_TRIMPITCHP | AGC_CH012_TRIMROLLP))
#define TRIM_PITCH ((uint16_t) AGC_CH012_TRIMPITCHP) /* BIT10            */
#define TRIM_ROLL  ((uint16_t) AGC_CH012_TRIMROLLP)  /* BIT12            */
#define PH_5_3_B   ((uint16_t) 010035)

/* CONTROLLED_CONSTANTS (p.~): DPS and APS thrust, flow, tail-off; RCS.     */
static const agc_dp_t FDPS    = 9158263; /* 4.3670 B-7: 9817.5 lb        */
static const agc_dp_t MDOTDPS = 4966056; /* 0.1480 B-3: 32.62 lb/s       */
static const agc_dp_t DTDECAY = -38;     /* 2DEC -38 cs                  */
static const agc_dp_t FAPS    = 3265056; /* 1.5569 B-7: 3500 lb          */
static const agc_dp_t MDOTAPS = 1723020; /* 0.05135 B-3: 11.32 lb/s      */
static const agc_dp_t ATDECAY = -10;     /* 2DEC -10 cs                  */
static const agc_dp_t FRCS4   = 373125;  /* 0.17792 B-7: 400 lb          */
static const agc_dp_t FRCS2   = 186563;  /* 0.08896 B-7: 200 lb          */
/* APSVEX DEC -3030 E-2 B-5, DPSVEX DEC* -2.95588868 E+1 B-05* (m/cs 2^5). */
static const agc_sp_t APSVEX = -15514;
static const agc_sp_t DPSVEX = -15134;
/* DV monitor thresholds THRESH1 24, THRESH3 12 (DPS), THRESH2 308 (APS).  */
#define THRESH1 24
#define THRESH2 308
#define THRESH3 12
/* S40.13: K1VAL 124.55 B-23 (2800 lb-s), K2VAL 31.138 B-24 (700 lb-s),
 * K3VAL 1.5569 B-10 (FAPS), S40.136 .4671 B-9 (DPS M N), S40.136_ = B+1,
 * 4SEC(17) 400 B-17, 1SEC2D 100 B-14, 3.5SEC 350 B-13, 5SECS 500 B-14,
 * 6SEC 600 B-14, 89SECS 8900 B-14.                                         */
static const agc_dp_t K1VAL                      = 3986;
static const agc_dp_t K2VAL                      = 498;
static const agc_dp_t K3VAL                      = 408132;
static const agc_dp_t DPS_IMPULSE_THRUST         = 244895;
static const agc_dp_t DPS_IMPULSE_THRUST_SHIFTED = 250772403;
static const agc_dp_t SEC4_17                    = 819200;
static const agc_dp_t SEC1_2D                    = 1638400;
static const agc_dp_t SEC3_5                     = 11468800;
static const agc_dp_t SEC5_14                    = 8192000;
static const agc_dp_t SEC6_14                    = 9830400;
static const agc_dp_t SEC89_14                   = 145817600;
/* FOURSECS 2DEC 400 (B-28); -FOURDT is in agc_guid_s40_8_tgo_lm.          */
static const agc_dp_t FOURSECS = 400;
/* EPS1 10 deg, EPS2 35 deg (rev).                                          */
static const agc_dp_t EPS1 = 7456540;
static const agc_dp_t EPS2 = 26097892;

/* The engine constants for P40 (DPS) and P42 (APS): F, MDOT, TDECAY by a
 * vector load, VEX = -VEX(SLOAD) SR1 (m/cs 2^6).                           */
static void load_engine(bool aps)
{
    agc_guid_t *g = agc_guid_state();

    g->burn.f    = aps ? FAPS : FDPS;
    g->lm.mdot   = aps ? MDOTAPS : MDOTDPS;
    g->lm.tdecay = aps ? ATDECAY : DTDECAY;
    g->lm.vex    =
        agc_dp_shr(agc_dp_neg(agc_dp_from_sp(aps ? APSVEX : DPSVEX)), 1);

    if (!AGC_ASSERT(g->lm.vex > 0))
    {
        return;
    }
}

/* P40IN .. P40SXT4 (p.753-754): S40.1, S40.2,3; DB 1 deg (PFLITEDB); R60
 * with VECPOINT (3AXISFLG reset); then ret_job (BURNBABY / P41 rest).    */
static agc_status_t burn_program_start(agc_job_ctx_t *ctx, agc_job_id_t ret_job,
                                       bool rcs_burn)
{
    agc_guid_t *g = agc_guid_state();

    g->burn.weight_g = g->lm.mass; /* WEIGHT/G = MASS        */
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_guid_nav_set_resume(ctx->id);
    agc_status_t st = agc_guid_initial_velocity_to_gain(&ip, AGC_VEHICLE_LM);

    if (st == AGC_STATUS_OK)
    {
        st = agc_guid_preferred_thrust_attitude_lm(
            &ip); /* PREFERRED ORIENTATION  */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_inhint();

    if (rcs_burn)
    {
        agc_lm_dap_zatteror(); /* ZERO ATTITUDE ERRORS   */
        agc_status_t sj = agc_lm_dap_setmindb(); /* 0.3 DEGREE DEADBAND    */

        if (sj != AGC_STATUS_OK)
        {
            agc_relint();

            return (sj);
        }
    }
    else
    {
        agc_status_t sw = agc_lm_dap_pflitedb(); /* ONE DEGREE, ERRORS 0   */

        if (sw != AGC_STATUS_OK)
        {
            agc_relint();

            return (sw);
        }
    }

    agc_relint();
    agc_flag_clear(AGC_FLAG_3AXISFLG);       /* P41MANU: USE VECPOINT  */

    return (agc_imu_attitude_maneuver(ctx, ret_job)); /* R60LEM                 */
}

/* P40ALM (p.746): alarm 1706, then the V05N09 loop REP40ALM.              */
static agc_status_t stage_alarm(agc_job_ctx_t *ctx)
{
    agc_alarm(AGC_ALARM_01706); /* PROGRAM vs. CONFIG.    */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_stage_alarm_display(ctx));
}

/* P40LM (p.753): DPS burn. 4 type C here; WHICH = P40TABLE; APS staged ->
 * alarm 1706; IMU status; DV monitor 36 pulses (12 docked to the CSM),
 * 4 low passes; DPS constants, NOTHROTL reset; then P40IN and BURNBABY.  */
agc_status_t agc_job_dps_burn(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t     *g = agc_guid_state();
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_DPS_BURN,
                         AGC_TASK_NONE, (int16_t) AGC_PRIO20,
                         false, 0U};

    agc_phase_change_var((uint16_t) 004024, &v);
    g->lm.which = (agc_sp_t) AGC_LM_WHICH_DPS_BURN; /* P40ADRES               */

    if (agc_flag_test(AGC_FLAG_APSFLAG))
    {
        return (stage_alarm(ctx));
    }

    agc_status_t st = agc_imu_status_check();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g->lm.dvthrush =
        (agc_sp_t) ((agc_flag_test(AGC_FLAG_CSMDKFLG) ? 0 : THRESH1) + THRESH3);
    g->lm.dvcntr = 4;
    load_engine(false);
    agc_flag_clear(AGC_FLAG_NOTHROTL);

    return (burn_program_start(ctx, AGC_JOB_BURNBABY, false));
}

/* P42LM / P42STAGE (p.756): APS burn; an unstaged LM gets alarm 1706 (the
 * crew may proceed: REP40ALM (14) -> P42STAGE, entered as AGC_JOB_APS_BURN_STAGE).
 * DV monitor 308 pulses, AVFLAG set.                                      */
agc_status_t agc_job_aps_burn(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t     *g = agc_guid_state();
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_APS_BURN,
                         AGC_TASK_NONE, (int16_t) AGC_PRIO20,
                         false, 0U};

    agc_phase_change_var((uint16_t) 004024, &v);
    g->lm.which = (agc_sp_t) AGC_LM_WHICH_APS_BURN;

    if (!agc_flag_test(AGC_FLAG_APSFLAG) &&
        ctx->id != AGC_JOB_APS_BURN_STAGE_CHECKED)
    {
        return (stage_alarm(ctx));
    }

    agc_status_t st = agc_imu_status_check(); /* P42STAGE               */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g->lm.dvthrush = THRESH2;
    g->lm.dvcntr   = 4;
    agc_flag_set(AGC_FLAG_AVFLAG);
    load_engine(true);

    return (burn_program_start(ctx, AGC_JOB_BURNBABY, false));
}

/* P41LM (p.755): RCS burn: F = 2-jet (NJETSFLG) or 4-jet, S40.1, S40.2,3,
 * attitude errors zeroed, 0.3 deg deadband, R60.                          */
agc_status_t agc_job_rcs_burn_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t *g = agc_guid_state();

    g->lm.which = (agc_sp_t) AGC_LM_WHICH_RCS_BURN;
    agc_status_t st = agc_imu_status_check();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g->burn.f = agc_flag_test(AGC_FLAG_NJETSFLG) ? FRCS2 : FRCS4;

    if (!AGC_ASSERT(g->burn.f > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (burn_program_start(ctx, AGC_JOB_RCS_BURN_AFTER_MANEUVER, true));
}

/* P41 after R60 (p.755-756): VGBODY = S41.1(VGTIG) for V16N85, then
 * GODSPRET; the job continues at agc_job_rcs_burn_vg_displayed.        */
agc_status_t agc_job_rcs_burn_after_maneuver(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t  *g = agc_guid_state();
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_guid_reference_to_body(&ip, AGC_VEHICLE_LM, agc_erase_vgtig(),
                               &g->burn.vgbody);
    const agc_dsky_cont_t cont = {AGC_JOB_RCS_BURN_VG_DISPLAYED, AGC_JOB_NONE,
                                  AGC_JOB_NONE};

    return (agc_dsky_display(ctx, AGC_DSKY_GODSPRET, agc_dsky_vn(16, 85),
                             &cont)); /* CADR GODSPRET */
}

/* P41 after GODSPRET (p.756): DISPDEX positive, DYNMDISP at PRIO5, 6.7
 * (REDO6.7) and 4 type C, B*RNB*B*.                                      */
agc_status_t agc_job_rcs_burn_vg_displayed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t *g = agc_guid_state();

    g->lm.dispdex = (agc_sp_t) AGC_PRIO5; /* FOR SAFETY ONLY        */
    agc_status_t st = agc_exec_findvac(AGC_PRIO5, AGC_JOB_DYNDISP);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_RCS_BURN_AFTER_MANEUVER,
                         AGC_TASK_NONE, (int16_t) AGC_PRIO20,
                         false, 0U};

    agc_phase_change2((uint16_t) 000076, (uint16_t) 004024, &v);

    return (agc_lm_burnbaby_countdown(ctx));
}

/* REDO6.7 / BLNKWAIT (p.756): on a restart do not put up N85 while the
 * DSKY is blanked (DISPDEX = -BLANKDEX = -2) -- wait a second.            */
agc_status_t agc_job_redo6_7(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_guid_state()->lm.dispdex + 2 == 0)
    {
        return (agc_svc_delayjob(ONE_SEC, AGC_JOB_REDO6_7)); /* BLNKWAIT      */
    }

    const agc_dsky_cont_t cont = {AGC_JOB_REDO6_7_VG_DISPLAYED, AGC_JOB_NONE,
                                  AGC_JOB_NONE};

    return (agc_dsky_display(ctx, AGC_DSKY_GODSPRET, agc_dsky_vn(16, 85),
                             &cont)); /* CADR GODSPRET */
}

/* REDO6.7 after GODSPRET (p.757): PRIOCHNG to PRIO5, then DYNMDISP. A
 * higher job waiting after the change runs first (CHANG1).               */
agc_status_t agc_job_redo6_7_vg_displayed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_exec_prio_change(AGC_PRIO5);

    if (agc_exec_newjob() > 0)
    {
        return (agc_exec_job_yield(AGC_JOB_DYNDISP));
    }

    return (agc_job_dynmdisp(ctx));
}

/* DYNMDISP (p.756-757): every second VGBODY = S41.1(VGPREV) until TIG-35
 * (DISPDEX <= 0: SERVICER updates N85 from then on).                       */
agc_status_t agc_job_dynmdisp(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t *g = agc_guid_state();

    if (g->lm.dispdex <= 0)
    {
        return (AGC_STATUS_OK); /* ENDOFJOB               */
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_guid_reference_to_body(&ip, AGC_VEHICLE_LM, agc_erase_vgtig(),
                               &g->burn.vgbody);

    return (agc_svc_delayjob(ONE_SEC, AGC_JOB_DYNDISP));
}

/* POSTBURN (p.754): CLOKTASK off (DISPDEX positive), AVEGEXIT = CALCN85,
 * flash V16N40: PROCEED -> TIGNOW, V34 -> TERM40; 4.1.                    */
agc_status_t agc_job_postburn_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t *g = agc_guid_state();

    g->lm.dispdex  = 1;                          /* CA Z TS DISPDEX        */
    g->avg.avgexit = AGC_JOB_CALC_VG_BODY; /* ACADN85                */
    agc_dsky_cont_t cont = {AGC_JOB_TERM40, AGC_JOB_TIGNOW, AGC_JOB_POSTBURN};
    agc_status_t    st   =
        agc_dsky_display(ctx, AGC_DSKY_GOFLASHR, agc_dsky_vn(16, 40), &cont);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change(4, (uint16_t) 000014);

    return (AGC_STATUS_OK); /* P40PHS1                */
}

/* TIGNOW (p.754): attitude errors zeroed, 0.3 deg deadband, flash V16N85
 * (RCS trim); terminate or proceed -> TERM40; 4.1.                        */
agc_status_t agc_job_tignow_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_lm_dap_zatteror();
    agc_status_t sw = agc_lm_dap_setmindb();

    if (sw != AGC_STATUS_OK)
    {
        agc_relint();

        return (sw);
    }

    agc_relint();
    agc_dsky_cont_t cont = {AGC_JOB_TERM40, AGC_JOB_TERM40, AGC_JOB_TIGNOW};
    agc_status_t    st   =
        agc_dsky_display(ctx, AGC_DSKY_REFLASHR, agc_dsky_vn(16, 85), &cont);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change(4, (uint16_t) 000014);

    return (AGC_STATUS_OK);
}

/* TERM40 (p.754-755): AVEGEXIT = SERVEXIT, rendezvous mark counter 0,
 * CLOKTASK off, deadband restored, P00.                                   */
agc_status_t agc_job_term40(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t *g = agc_guid_state();

    g->avg.avgexit = AGC_JOB_SERVEXIT;
    agc_erase_set_trkmkcnt(0); /* ZERO RENDZVS CNTERS    */
    g->lm.dispdex = 1;
    agc_inhint();
    agc_status_t st = agc_lm_dap_restordb();

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_relint();

    return (agc_exec_gotopooh());
}

/* CALCN85 (p.757): AVEGEXIT after the burn: UPDATEVG, N85, SERVEXIT.       */
agc_status_t agc_job_calc_vg_body_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t  *g = agc_guid_state();
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_status_t st = agc_lm_updatevg(&ip);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_reference_to_body(&ip, AGC_VEHICLE_LM, agc_erase_vgtig(),
                               &g->burn.vgbody);

    return (agc_job_servexit(ctx));
}

/* P47LM (p.757-758): IMU status, MIDTOAV2, STARTP47 at the PREREAD time.  */
agc_status_t agc_job_thrust_monitor_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t sw = agc_imu_status_check();

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_integ_midtoav_out_t mo;
    agc_interp_t            ip;

    agc_interp_init(&ip);
    agc_status_t st =
        agc_integ_midtoav2(&ip, agc_integ_shared_ctx(), ctx->id, &mo);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_store_midtoav(&mo);
    agc_dp_t dt = mo.dt;

    if (!AGC_ASSERT(dt >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_waitlist_call((uint16_t) agc_dp_low(dt),
                              AGC_TASK_TIGON)); /* TWIDDLE */
}

/* STARTP47 (p.758): type C immediate, AVEGEXIT = CALCN83, P47BODY job,
 * REDO4.2 (PREREAD unless SERVICER runs).                                 */
agc_status_t agc_task_start_thrust_monitor(void)
{
    agc_phase_var_t v = {
        AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_TIGON, 0, true, 0U
    };

    agc_phase_change_var((uint16_t) 005014, &v);
    agc_guid_state()->avg.avgexit = AGC_JOB_CALC_DELTA_V_BODY;
    agc_status_t st =
        agc_exec_findvac(AGC_PRIO20, AGC_JOB_THRUST_MONITOR_RESET);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(agc_guid_state()->avg.avgexit == AGC_JOB_CALC_DELTA_V_BODY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_redo4_2());
}

/* CALCN83 (p.758): DELVSIN (temp) = DELVCTL + DELVREF, DELVIMU = S41.1 of
 * it (N83), 5.3 type B, DELVCTL = DELVSIN, SERVEXIT.                      */
agc_status_t agc_job_calc_delta_v_body_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t  *g  = agc_guid_state();
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_interp_t ip;

    agc_interp_init(&ip);
    rz->delvsin = agc_vec_add(&ip, g->burn.delvctl, g->avg.delvref);
    agc_guid_reference_to_body(&ip, AGC_VEHICLE_LM, rz->delvsin,
                               &g->burn.delvimu);
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_CALC_DELTA_V_BODY,
                         AGC_TASK_NONE, (int16_t) AGC_PRIO20,
                         false, 0U};

    agc_phase_change_var(PH_5_3_B, &v);
    g->burn.delvctl = rz->delvsin;

    return (agc_job_servexit(ctx));
}

/* P47BODY / P47BOD (p.758-759): clear N83, flash V16N83 (ENTER clears).   */
agc_status_t agc_job_thrust_monitor_reset_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_burn_t *b = &agc_guid_state()->burn;

    b->delvimu = agc_vec_zero();
    b->delvctl = agc_vec_zero();

    return (agc_job_thrust_monitor_display_lm(ctx));
}

agc_status_t agc_job_thrust_monitor_display_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH, AGC_JOB_GOTOPOOH,
                            AGC_JOB_THRUST_MONITOR_RESET};
    agc_status_t    st =
        agc_dsky_display(ctx, AGC_DSKY_GOFLASHR, agc_dsky_vn(16, 83), &cont);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change(4, (uint16_t) 000014);

    return (AGC_STATUS_OK); /* P40PHS1                */
}

/* GETDT (p.762): the engine-off delay, at least 1 cs; TGO = (0, delay).   */
static uint16_t getdt(agc_sp_t t)
{
    agc_sp_t dt = (t > 0) ? t : 1; /* CCS: AD ONE            */

    agc_erase_set_tgo((agc_dp_t) dt);

    if (!AGC_ASSERT(dt >= 1))
    {
        return (0); /* no valid result */
    }

    return ((uint16_t) dt);
}

/* IMPLBURN (p.759): engine-off task at TGO, IGNFLAG, ASTNFLAG, IMPULSW
 * reset, 4.11 (ENGOFTSK), ullage off half a second later.                 */
agc_status_t agc_task_implburn_lm(void)
{
    agc_status_t st = agc_waitlist_call(getdt(agc_dp_low(agc_erase_tgo())),
                                        AGC_TASK_ENGINOFF);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_clear(AGC_FLAG_IGNFLAG);
    agc_flag_clear(AGC_FLAG_ASTNFLAG);
    agc_flag_clear(AGC_FLAG_IMPULSW);
    agc_phase_change(4, (uint16_t) 040114); /* ENGOFTSK (ENGINOFF)    */

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_IMPULSW)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_waitlist_fixdelay(HALF_SEC, AGC_TASK_IMPLBURN2));
}

agc_status_t agc_task_implburn2(void)
{
    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_IGNFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_noullage();

    return (AGC_STATUS_OK); /* TURN OFF ULLAGE        */
}

/* ENGOFTSK (p.759): ENGINOFF as a waitlist task.                           */
agc_status_t agc_task_engoftsk(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_enginoff());
}

/* ENGINOF3 (p.760): ENGONFLG reset, engine off (DSALMOUT bits 13, 14 =
 * 0, 1), trim gimbal off (USEQRJTS), auto-throttle zeroed (THRUST =
 * -HIRTHROT, channel 14 bit 4).                                           */
void agc_lm_enginof3(void)
{
    agc_flag_clear(AGC_FLAG_ENGONFLG);
    uint16_t ch11 = agc_chan_read(011);

    ch11 = (uint16_t) ((ch11 & ~(uint16_t) (AGC_CH011_ENGINEON |
                                            AGC_CH011_ENGINEOFF)) |
                       (uint16_t) AGC_CH011_ENGINEOFF);
    agc_chan_write(011, ch11);         /* TURN OFF THE ENGINE    */
    agc_flag_set(AGC_FLAG_USEQRFLG);   /* TURN OFF TRIM GIMBAL   */
    agc_erase_set_thrust((agc_sp_t) -010000); /* CS HIRTHROT (BIT13)    */

    if (!AGC_ASSERT((agc_chan_read(011) & AGC_CH011_ENGINEON) == 0U))
    {
        return;
    }

    agc_chan_or(014, (uint16_t) AGC_CH014_THRUSTACT);
}

/* ENGINOF4 (p.760): TEVENT, then ENGINOF3.                                 */
void agc_lm_enginof4(void)
{
    agc_erase_set_tevent((agc_dp_t) agc_time_now_cs());

    if (!AGC_ASSERT(agc_erase_tevent() >= 0))
    {
        return;
    }

    agc_lm_enginof3();
}

/* ENGINOF1 (p.760): DV monitor off (IDLEFLAG), ullage off, ENGINOF4.       */
agc_status_t agc_lm_enginof1(void)
{
    agc_flag_set(AGC_FLAG_IDLEFLAG);
    agc_lm_noullage();

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_IDLEFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_lm_enginof4();

    return (AGC_STATUS_OK);
}

/* ENGINOF2 (p.760): DAP coasting set-up (COASTSET) in 10 ms, ENGINOF1.    */
agc_status_t agc_lm_enginof2(void)
{
    agc_status_t st = agc_waitlist_call(1, AGC_TASK_COASTSET);

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st != AGC_STATUS_OK) ? st : agc_lm_enginof1());
}

/* ENGINOFF (p.759): POSTBURN job (lower priority than CLOKJOB), ENGINOF2. */
agc_status_t agc_lm_enginoff(void)
{
    agc_status_t st = agc_exec_findvac(AGC_PRIO12, AGC_JOB_POSTBURN);

    _Static_assert(AGC_PRIO12 < AGC_PRIO27, "AGC constant");

    return ((st != AGC_STATUS_OK) ? st : agc_lm_enginof2());
}

/* COASTSET (p.760): ALLCOAST.                                              */
agc_status_t agc_task_coastset(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_dap_allcoast());
}

/* S40.8 Luminary (p.770-771): VG = VGPREV - DELVREF (external delta-V) or
 * from RASTEER1 (aimpoint); UNFC/2 = REFSMMAT VG VSL1; VGDISP; VGPREV =
 * VG; with STEERSW, TGO and FINDCDUW.                                     */
agc_status_t agc_lm_cross_product_steering(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t *g = agc_guid_state();

    if (!agc_flag_test(AGC_FLAG_XDELVFLG))
    {
        agc_status_t st = agc_lm_rasteer1(ip); /* GENERATE VR            */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        g->burn.vg = agc_rendz_erasable()->delveet3;
    }
    else
    {
        g->burn.vg = agc_vec_sub(ip, agc_erase_vgtig(), g->avg.delvref);
    }

    agc_mat_t refsmmat = agc_erase_refsmmat();

    agc_lm_ldg()->unfc_2 =
        agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, g->burn.vg), 1);
    g->burn.vgdisp = agc_vec_abval(ip, g->burn.vg); /* BDTOK                  */
    agc_erase_set_vgtig(g->burn.vg);                /* STOVL VGPREV           */

    if (!agc_flag_test(AGC_FLAG_STEERSW))
    {
        return (AGC_STATUS_OK);
    }

    /* The steering part of S40.8 (p.771): TGDCALC with VEX and TDECAY; TIG =
     * PIPTIME + TGO; FINDCDUW while TGO >= 4 s, else IMPULSW set, STEERSW
     * reset. A VG more than 90 degrees from the thrust gives alarm 1407 and
     * goes straight to FINDCDUW.                                               */
    agc_guid_t  *g_commands = agc_guid_state();
    agc_tgo_in_t in         = {g_commands->burn.vg, agc_vec_neg(
                                   g_commands->avg.delvref), g_commands->lm.vex,
                               g_commands->lm.tdecay};
    agc_dp_t     tgo         = 0;
    agc_status_t st_commands = agc_guid_steering_tgo_lm(ip, &in, &tgo);

    if (st_commands == AGC_STATUS_ALARM)
    {
        agc_alarm(AGC_ALARM_01407); /* ALARMIT                */

        return (agc_lm_findcduw_m2(ip));
    }

    agc_erase_set_tgo(tgo);
    agc_erase_set_tig(agc_dp_add(ip, tgo, agc_erase_piptime()));

    if (agc_dp_sub(ip, tgo, FOURSECS) >= 0)
    {
        return (agc_lm_findcduw_m2(ip)); /* FINDCDUW -2      */
    }

    agc_flag_set(AGC_FLAG_IMPULSW);
    agc_flag_clear(AGC_FLAG_STEERSW);

    return (AGC_STATUS_OK);
}

/* UPDATEVG Luminary (p.761-762): S40.8; for aimpoint burns a new Lambert
 * (S40.9, 2.17) every TNEWA after TIGSAVE (or every pass inside the
 * 165-195 degree cone, NORMSW) unless one is running (PHASE2), then
 * RINIT/VINIT/TIGSAVE for it and DELLT4 = TPASS4 - PIPTIME.               */
agc_status_t agc_lm_updatevg(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t  *g  = agc_guid_state();
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_status_t sw = agc_lm_cross_product_steering(ip); /* X-PRODUCT STEERING     */

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    if (agc_flag_test(AGC_FLAG_XDELVFLG))
    {
        return (AGC_STATUS_OK);
    }

    bool start = true;

    if (!agc_flag_test(AGC_FLAG_NORMSW))
    {
        agc_dp_t d =
            agc_dp_sub(ip, agc_dp_sub(ip, agc_erase_piptime(), g->lm.tigsave),
                       g->lm.tnewa);

        start = (d >= 0);

        if (start)
        {
            g->lm.tigsavep = agc_dp_add(ip, g->lm.tigsave, g->lm.tnewa);
        }
    }

    if (start && agc_phase_get(2) == 0U)
    { /* 180SETUP               */
        agc_status_t st = agc_exec_findvac(AGC_PRIO10, AGC_JOB_VTOGAIN);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        agc_phase_change2((uint16_t) 000172, PH_5_3_B, NULL);
        g->lm.tigsave = g->lm.tigsavep; /* ENDSTEER               */
        rz->rinit     = agc_erase_rn();
        rz->vinit     = agc_erase_vn();
    }
    else if (start)
    {
        return (AGC_STATUS_OK); /* NO.9                   */
    }

    rz->dellt4 =
        agc_dp_sub(ip, rz->tpass4, agc_erase_piptime()); /* GETRANS   */

    return (AGC_STATUS_OK);
}

/* STEERING / NSTEER (p.762): UPDATEVG; while thrusting (IMPULSW set by
 * S40.8 and the DV monitor on) stop the rates, reset IMPULSW, DV monitor
 * off, ENGOFTSK at TIG (4.11 and 5.3); otherwise SERVEXIT.                 */
agc_status_t agc_job_steering_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_status_t sw = agc_lm_updatevg(&ip);

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    if (!agc_flag_test(AGC_FLAG_IMPULSW) || agc_flag_test(AGC_FLAG_IDLEFLAG))
    {
        return (agc_job_servexit(ctx));
    }

    agc_inhint();
    agc_lm_dap_stoprate();
    agc_flag_clear(AGC_FLAG_IMPULSW);
    agc_flag_set(AGC_FLAG_IDLEFLAG); /* TURN OFF DVMON         */
    agc_dp_t     t  = agc_erase_tig() - (agc_dp_t) agc_time_now_cs();
    agc_sp_t     lo = (agc_dp_high(t) == 0) ? agc_dp_low(t) : 0;
    agc_status_t st = agc_waitlist_call(getdt(lo), AGC_TASK_ENGINOFF);

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_phase_change2((uint16_t) 040114, (uint16_t) 000035, NULL);
    agc_relint();

    return (AGC_STATUS_OK); /* ENDOFJOB               */
}

/* S40.9 Luminary (p.776): AVFLAG set (LM active), HAVEGUES with NUMIT 0
 * and EPSILON4 10 or 45 degrees (NORMSW), group 2 off.                    */
agc_status_t agc_job_vtogain_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_flag_set(AGC_FLAG_AVFLAG);
    agc_dp_t e4 =
        agc_flag_test(AGC_FLAG_NORMSW) ? agc_dp_add(&ip, EPS1, EPS2) : EPS1;
    agc_status_t st =
        agc_rendz_havegues(&ip, agc_rendz_erasable(), 0, e4, ctx->id);

    agc_phase_kill(2); /* ENDS40.9               */

    return (st);
}

/* S40.13 short-burn branch, APS (p.773-774): v = |VGTIG| less the 4 s of
 * 2-jet ullage (2^7), m = WEIGHT/G (2^16). Below K1/m (one second of APS):
 * TGO = (m v SR1 + K2)/K3; otherwise with TEMP1 = v - K1/m and TEMP2 =
 * 5 s F SR2 / (m - 3.5 s MDOT): TGO = 1 s + 5 s TEMP1/TEMP2 if TEMP1 <
 * TEMP2. Result: TGO cs at 2^14 (IMPULSW to be set), or -1 to steer.     */
static agc_dp_t timeburn_aps(agc_interp_t *ip, agc_dp_t v, agc_dp_t m)
{
    const agc_guid_t *g     = agc_guid_state();
    agc_dp_t          temp1 = agc_dp_sub(ip, v, agc_dp_div(ip, K1VAL, m)); /* BDSU 00D */

    if (temp1 < 0)
    { /* S40.131  */
        agc_dp_t mv = agc_dp_shr(agc_dp_mul_trunc(m, v), 1);

        return (agc_dp_div(ip, agc_dp_add(ip, mv, K2VAL), K3VAL));
    }

    agc_dp_t mass  = agc_dp_sub(ip, m, agc_dp_mul_trunc(g->lm.mdot, SEC3_5));
    agc_dp_t f5    = agc_dp_shr(agc_dp_mul_trunc(g->burn.f, SEC5_14), 2);
    agc_dp_t temp2 = agc_dp_div(ip, f5, mass); /* 04D      */

    if (agc_dp_sub(ip, temp1, temp2) >= 0)
    {
        return (-1); /* S40.13D  */
    }

    if (!AGC_ASSERT(temp2 > 0))
    {
        return (0); /* no valid result */
    }

    agc_dp_t r = agc_dp_div(ip, temp1, temp2); /* BDDV     */

    return (agc_dp_add(ip, agc_dp_mul_trunc(r, SEC5_14), SEC1_2D));
}

/* S40.13D (p.774-775), the burn is long enough to steer: APS TGO = m v /
 * FAPS SL2; DPS TGO = m v / (.4671 M N), NOTHROTL cleared, under 6 s an
 * impulsive burn (IMPULSW), under 95 s NOTHROTL set; a DPS quotient that
 * overflows 2^14 cs is recomputed at 2^28 (S40.130V) and stored as is.
 * Returns TGO at 2^14 (SP cs) or, for S40.130V, sets TGO itself and
 * returns -1.                                                             */
static agc_dp_t timeburn_long(agc_interp_t *ip, agc_dp_t v, agc_dp_t m,
                              bool *impulsive)
{
    agc_dp_t mv = agc_dp_mul_trunc(v, m); /* 2^23     */

    if (agc_flag_test(AGC_FLAG_APSFLAG))
    { /* APSTGO   */
        return (agc_dp_shl(ip, agc_dp_div(ip, mv, FAPS), 2));
    }

    agc_dp_t t = agc_dp_div(ip, mv, DPS_IMPULSE_THRUST);

    agc_flag_clear(AGC_FLAG_NOTHROTL);

    if (agc_interp_test_ovf(ip))
    { /* S40.130V */
        agc_erase_set_tgo(
            agc_dp_div(ip, agc_dp_shr(mv, 4), DPS_IMPULSE_THRUST_SHIFTED));

        return (-1);
    }

    agc_dp_t d = agc_dp_sub(ip, t, SEC6_14); /* S40.127  */

    if (d < 0)
    {
        *impulsive = true; /* S40.132  */

        return (t);
    }

    if (agc_dp_sub(ip, d, SEC89_14) < 0)
    { /* S40.138  */
        agc_flag_set(AGC_FLAG_NOTHROTL);
    }

    return (t); /* STORETGO */
}

/* S40.13 TIMEBURN, Luminary (p.773-775), a job: decides whether the burn
 * (VGTIG, WEIGHT/G, F, MDOT, APSFLAG) is long enough to steer. Outputs
 * IMPULSW (set: attitude hold, engine off TGO after ignition), NOTHROTL
 * (set: no throttling) and TGO (cs, 2^28: the high word of the 2^14
 * result as the low word, TPAGREE / DXCH TGO). Group 3 off at the end.   */
agc_status_t agc_job_timeburn_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_guid_t *g = agc_guid_state();
    agc_dp_t          m = g->burn.weight_g;

    if (!AGC_ASSERT(m > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_flag_clear(AGC_FLAG_IMPULSW); /* NO STEERING UNTIL FOUND  */
    agc_dp_t v   = agc_vec_abval(&ip, agc_erase_vgtig());
    agc_dp_t dvu = agc_dp_shl(
        &ip, agc_dp_div(&ip, agc_dp_mul_trunc(SEC4_17, FRCS2), m), 1);

    v = agc_dp_sub(&ip, v, dvu); /* 00D: VG LESS ULLAGE      */
    bool     impulsive = false;
    agc_dp_t t14       = -1;

    if (agc_flag_test(AGC_FLAG_APSFLAG))
    {
        agc_flag_set(AGC_FLAG_NOTHROTL);
        t14       = timeburn_aps(&ip, v, m);
        impulsive = (t14 >= 0);
    }

    if (t14 < 0)
    {
        t14 = timeburn_long(&ip, v, m, &impulsive);
    }

    if (impulsive)
    {
        agc_flag_set(AGC_FLAG_IMPULSW); /* S40.132                  */
    }

    if (t14 >= 0)
    {
        agc_erase_set_tgo(
            (agc_dp_t) agc_dp_high(t14)); /* S40.132*: TGO = (0, MPAC) */
    }

    agc_phase_change(3, (uint16_t) 000003);

    return (AGC_STATUS_OK); /* S40.134                  */
}

/* TRIMGIMB (formerly S40.6), p.781-782, a waitlist task from R03: trims
 * the DPS engine: drive to full + pitch and + roll for one minute, then
 * run - pitch for PITTIME and - roll for ROLLTIME (cs); when both are
 * off, TRIMDONE (R03) as a PRIO10 job. GMBDRVSW records "one is done".  */
agc_status_t agc_task_trimgimb(void)
{
    agc_flag_clear(AGC_FLAG_GMBDRVSW);
    agc_chan_and(012, (uint16_t) ~TRIM_MINUS); /* - PITCH, - ROLL OFF     */
    agc_chan_or(012, TRIM_PLUS);          /* + PITCH, + ROLL ON      */

    if (!AGC_ASSERT((agc_chan_read(012) & TRIM_MINUS) == 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_waitlist_fixdelay(TRIM_WAIT, AGC_TASK_TRIMGIMB2));
}

/* TRIMGIMB after the minute: + off, - on, PITCHOFF at PITTIME, roll off
 * at ROLLTIME (VARDELAY).                                                 */
agc_status_t agc_task_trimgimb2(void)
{
    const agc_guid_lm_t *lm = &agc_guid_state()->lm;

    if (!AGC_ASSERT(lm->pittime >= 0 && lm->rolltime >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_chan_and(012, (uint16_t) ~TRIM_PLUS);
    agc_chan_or(012, TRIM_MINUS);
    agc_status_t st = agc_waitlist_call((uint16_t) lm->pittime,
                                        AGC_TASK_PITCHOFF); /* TWIDDLE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_waitlist_vardelay((uint16_t) lm->rolltime, AGC_TASK_ROLLOFF));
}

/* ROLLOVER (p.781-782): the second of roll/pitch to finish starts
 * TRIMDONE; the first sets GMBDRVSW. Then TASKOVER.                       */
static agc_status_t rollover(void)
{
    if (agc_flag_test(AGC_FLAG_GMBDRVSW))
    {
        if (!AGC_ASSERT((agc_chan_read(012) & TRIM_MINUS) == 0U))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        return (agc_exec_novac(AGC_PRIO10, AGC_JOB_TRIMDONE)); /* RETURN TO R03 */
    }

    agc_flag_set(AGC_FLAG_GMBDRVSW);

    return (AGC_STATUS_OK); /* PITCHOFF +4             */
}

/* TRIMGIMB after ROLLTIME: shut off - roll (BIT12), ROLLOVER.            */
agc_status_t agc_task_rolloff(void)
{
    agc_chan_and(012, (uint16_t) ~TRIM_ROLL);

    if (!AGC_ASSERT((agc_chan_read(012) & TRIM_ROLL) == 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rollover());
}

/* PITCHOFF: shut off - pitch (BIT10), ROLLOVER.                           */
agc_status_t agc_task_pitchoff(void)
{
    agc_chan_and(012, (uint16_t) ~TRIM_PITCH);

    if (!AGC_ASSERT((agc_chan_read(012) & TRIM_PITCH) == 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rollover());
}
