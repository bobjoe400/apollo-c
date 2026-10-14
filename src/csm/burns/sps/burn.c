/**********************************************************************
 * FILE NAME: burn.c
 *
 * PURPOSE:
 *   The CSM thrusting programs from their start to ignition: P40CSM
 *   (SPS), P41CSM (RCS), P40S/F .. P40SXTY (S40.1, S40.2,3, R60), P41/DSP,
 *   P40TTOG/P40GMB with the gimbal test question (TRIMONLY, TST,TRIM),
 *   P40S/RS, P40S/SV (state to TIG-30), P40BLNKR, TIGBLNK, PRECHECK,
 *   TTG/0, TIG-5 and TIG-0.
 *
 * AGC SOURCE:
 *   Comanche055 P40-P47.agc pp. 684-695
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   MODREG        agc_sp_t   I    -            major mode (P40 or P41)
 *   TIG           agc_dp_t   I/O  cs 2^28      time of ignition
 *   VGTIG         agc_vec_t  I    m/cs 2^7     velocity to be gained
 *   P40TMP        agc_dp_t   I/O  cs 2^28      delay to PREREAD - 5 s
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_guid_initial_velocity_to_gain        S40.1 (burn_steering.c)
 *   agc_guid_preferred_thrust_attitude_csm   S40.2,3 (burn_steering.c)
 *   agc_integ_midtoav_at_time    MIDTOAV1 integration to TIG-30 (integration.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None raised here; IMU status, integration, display, executive and
 *   waitlist failures come back as their status.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only; jobs and waitlist tasks of the burn programs, restart
 *   groups 4 and 6. Pieces shared with ignition.c, steer.c and clock.c are
 *   declared in internal.h.
 *
 * NOTES:
 *   S41.2, S40.14 and RCSDAPON are the DAP lane's (csm/dap.h). The
 *   prolog page numbers are the listing's page markers; a few page
 *   references inside the bodies keep the original translation's
 *   numbering (four to nine pages ahead).
 *
 * ALGORITHM:
 *   As in the listing.
 **********************************************************************/
#include <stddef.h>

#include "agc/guidance/guidance.h"
#include "internal.h"

/* FENG 2DEC 9.1188544 B-7: SPS thrust 20 500 lb in m.newton (10^4 N).     */
static const agc_dp_t FENG = 19123624; /* FENG, SPS thrust, M.newton 2^7 */

/* FRCS2 2DEC .087437837 B-7: RCS ullage 199.6 COS 10 lb, 2 jets.         */
static const agc_dp_t FRCS2 = 183370; /* FRCS2, 2-jet RCS thrust, M.newton 2^7 */

/* SEC29.96 2DEC 2996, SEC24.96 DEC 2496, 18SEC, 5SEC: centiseconds.        */
static const agc_dp_t SEC29_96 = 2996; /* SEC29.96, cs 2^28 */
#define SEC24_96              2496     /* SEC24.96, cs */
#define SEC18                 1800     /* 18SEC, cs */
#define SEC5                  500      /* 5SEC, cs */
#define GIMBAL_TEST_CHECKLIST 0204     /* V50N25 R1 00204: gimbal drive test   */

/**********************************************************************
 * NVWORD1 values that display V06N40, V16N40, V06N85, V16N85, V16N83.
 * AGC: none (C helper: NVWORD1 holds the packed VN the CLOCKJOB shows).
 *
 * FUNCTION NAME: agc_csm_vn
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   verb      uint8_t     I    verb, decimal
 *   noun      uint8_t     I    noun, decimal
 *
 * RETURNS: agc_sp_t -- the NVWORD1 code of verb, noun (agc_dsky_vn)
 **********************************************************************/
agc_sp_t agc_csm_vn(uint8_t verb, uint8_t noun)
{
    return ((agc_sp_t) agc_dsky_vn(verb, noun));
}

/**********************************************************************
 * P41/P40 (p.688): MODREG odd (41) takes the P41 exit.
 *
 * FUNCTION NAME: agc_csm_is_rcs_burn
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true in P41 (MODREG odd), false in P40
 **********************************************************************/
bool agc_csm_is_rcs_burn(void)
{
    agc_sp_t mm = agc_erase_modreg(); /* MODREG, major mode; unscaled */

    return ((mm & 1) != 0);
}

/**********************************************************************
 * P40S/F .. P40PVA .. P40SXTY (p.684-685): NOMTIG, IMU status, thrust
 * direction and preferred attitude, narrow deadband, attitude maneuver.
 *
 * FUNCTION NAME: burn_setup_with_thrust
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *   f         agc_dp_t         I    F, engine thrust, M.newton 2^7
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t burn_setup_with_thrust(agc_job_ctx_t *ctx, agc_dp_t f)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    g->burn.f       = f;
    g->burn.nomtig  = agc_erase_tig();        /* ORIGINAL TIG FOR S40.9 */
    agc_status_t sw = agc_imu_status_check(); /* IMU STATUS CHECK       */

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    g->burn.weight_g = agc_dp_from_sp(agc_csm_dap_com()->weight_g);
    agc_flag_set(AGC_FLAG_FIRSTFLG); /* S40.1: SET FIRSTFLG    */
    agc_interp_t ip;                 /* interpreter state */

    agc_interp_init(&ip);
    agc_guid_nav_set_resume(ctx->id);
    agc_status_t st = agc_guid_initial_velocity_to_gain(&ip, AGC_VEHICLE_CSM); /* VGTIG, UT       */

    if (st == AGC_STATUS_OK)
    {
        const agc_csm_dapcom_t *com = agc_csm_dap_com(); /* CSM DAP common erasables */

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

/**********************************************************************
 * P40CSM (p.684): SPS. CSTEER = 0 for an external delta-V burn, else
 * ECSTEER; F = FENG.
 *
 * FUNCTION NAME: agc_job_sps_burn
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_sps_burn(agc_job_ctx_t *ctx)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    agc_flag_clear(AGC_FLAG_ENG2FLAG);
    g->burn.csteer = agc_flag_test(AGC_FLAG_XDELVFLG) ? 0 : g->burn.ecsteer; /* external delta-V: no aimpoint steering */

    return (burn_setup_with_thrust(ctx, FENG));
}

/**********************************************************************
 * P41CSM (p.688): RCS. CSTEER = 0; F = 2-jet (NJETSFLG) or 4-jet thrust.
 *
 * FUNCTION NAME: agc_job_rcs_burn_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_rcs_burn_csm(agc_job_ctx_t *ctx)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    agc_flag_set(AGC_FLAG_ENG2FLAG);                                       /* SET FOR RCS            */
    g->burn.csteer = 0;
    agc_dp_t f     = agc_flag_test(AGC_FLAG_NJETSFLG) ? FRCS2 : 2 * FRCS2; /* F, 2-jet or 4-jet RCS thrust, M.newton 2^7 */

    return (burn_setup_with_thrust(ctx, f));
}

/**********************************************************************
 * P41/DSP (p.688): V06N85 by the CLOCKJOB (VGTIG in control axes), CALCN85
 * as AVEGEXIT, 6.3 and 4.23, then P40S/RS.
 *
 * FUNCTION NAME: rcs_burn_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t rcs_burn_display(agc_job_ctx_t *ctx)
{
    agc_guid_t  *g = agc_guid_state(); /* guidance erasables */
    agc_interp_t ip;                   /* interpreter state */

    g->csm.nvword1 = agc_csm_vn(6, 85); /* V06N85B                */
    agc_interp_init(&ip);
    agc_csm_convert_vg_body(&ip);       /* VGTIG IN CTRL COORDS   */
    g->avg.avgexit = AGC_JOB_CALC_VG_BODY;
    agc_phase_change2((uint16_t) 040036, (uint16_t) 000234, NULL);

    return (agc_csm_preburn_start_clock(ctx));
}

/**********************************************************************
 * P40SXTY after R60 (p.684): NBRCYCLS = -1, TIMRFLAG; P41 goes to P41/DSP,
 * P40 to P40TTOG and the gimbal test question (V50N25 204).
 *
 * FUNCTION NAME: agc_job_sps_burn_after_maneuver
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_sps_burn_after_maneuver(agc_job_ctx_t *ctx)
{
    agc_guid_t  *g = agc_guid_state(); /* guidance erasables */
    agc_status_t st;                   /* status of the display path */

    g->burn.nbrcycls = -1;           /* FOR UPDATEVG           */
    agc_flag_set(AGC_FLAG_TIMRFLAG); /* ALLOW CLOCKTASK        */

    if (agc_csm_is_rcs_burn())
    {
        st = rcs_burn_display(ctx); /* P41/DSP */
    }
    else
    {
        agc_interp_t ip; /* interpreter state */

        g->csm.nvword1       = agc_csm_vn(6, 40); /* P40TTOG                */
        agc_interp_init(&ip);
        g->burn.vgdisp       = agc_vec_abval(&ip, agc_erase_vgtig());
        g->avg.dvtotal       = 0;
        g->avg.avgexit       = AGC_JOB_STEERING;  /* STEERADS               */
        agc_dsky_cont_t cont = {AGC_JOB_POST41, AGC_JOB_GIMBAL_TEST_AND_TRIM, AGC_JOB_GIMBAL_TRIM_ONLY};

        st = agc_dsky_display(ctx, AGC_DSKY_GOPERF1, GIMBAL_TEST_CHECKLIST, &cont); /* P40GMB  */
    }

    return (st);
}

/**********************************************************************
 * TRIMONLY / TST,TRIM (p.685): S40.6 in 10 ms (MRKRTMP -1 trim only, +1
 * drive test and trim; CNTR = +0 normal entry), wait 5 or 18 s.
 *
 * FUNCTION NAME: trim_common
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   mrkrtmp   agc_sp_t    I    MRKRTMP: + test and trim, - trim only
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t trim_common(agc_sp_t mrkrtmp)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    g->csm.mrkrtmp          = mrkrtmp;
    agc_csm_dap_tvc()->cntr = 0;                                               /* +0 SAYS NORMAL ENTRY   */
    agc_status_t st         = agc_waitlist_call(1, AGC_TASK_GIMBAL_TEST_TRIM); /* status of agc_waitlist_call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_svc_delayjob((mrkrtmp > 0) ? SEC18 : SEC5, AGC_JOB_PREBURN_START_CLOCK)); /* test and trim 18 s, trim only 5 s */
}

/**********************************************************************
 * TRIMONLY (p.685): ENTER on V50N25 R1 00204: no drive test, trim only
 * (MRKRTMP -1).
 *
 * FUNCTION NAME: agc_job_gimbal_trim_only
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_gimbal_trim_only(void)
{
    return (trim_common(-1)); /* CS BIT1                */
}

/**********************************************************************
 * TST,TRIM (p.690): PROCEED on V50N25 R1 00204: gimbal drive test, then
 * trim (MRKRTMP +1).
 *
 * FUNCTION NAME: agc_job_gimbal_test_and_trim
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_gimbal_test_and_trim(void)
{
    return (trim_common(1)); /* CAF BIT1               */
}

/**********************************************************************
 * After the DELAYJOB (p.694): 6.2 (PRE40.6 -0 cs, CLOKTASK 1 s) and 4.23
 * (P40S/SV, PRIO12), then P40S/RS.
 *
 * FUNCTION NAME: agc_job_preburn_start_clock
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_preburn_start_clock(agc_job_ctx_t *ctx)
{
    agc_phase_change2((uint16_t) 040026, (uint16_t) 000234, NULL);

    return (agc_csm_preburn_start_clock(ctx));
}

/**********************************************************************
 * P40S/RS (p.685): CLOKTASK in 10 ms, then P40S/SV.
 *
 * FUNCTION NAME: agc_csm_preburn_start_clock
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_csm_preburn_start_clock(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_waitlist_call(1, AGC_TASK_CLOKTASK); /* status of agc_waitlist_call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_preburn_state_vector(ctx));
}

/**********************************************************************
 * P40S/SV / P40SNEWM / P40SET (p.685-686): integrate the state to 29.96 s
 * before TIG (MIDTOAV1); if that took too long, slip TIG to 29.96 s after
 * the integrated time. TIGBLNK by LONGCALL 5 s before TIG-30 (4.21).
 *
 * FUNCTION NAME: agc_job_preburn_state_vector
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_job_preburn_state_vector(agc_job_ctx_t *ctx)
{
    agc_guid_t             *g = agc_guid_state(); /* guidance erasables */
    agc_integ_midtoav_out_t mo;                   /* MIDTOAV results */
    agc_interp_t            ip;                   /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_integ_midtoav_at_time(&ip, agc_integ_shared_ctx(), agc_erase_tig() - SEC29_96, ctx->id, &mo); /* status of agc_integ_midtoav_at_time */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_store_midtoav(&mo);
    agc_dp_t dt = mo.dt; /* MIDTOAV delay to PREREAD, cs 2^28 */

    if (mo.slipped)
    { /* P40SNEWM               */
        agc_erase_set_tig(g->avg.piptime1 + SEC29_96);
    }

    return (agc_csm_preburn_call_tigblnk(dt)); /* P40SET */
}

/**********************************************************************
 * P40SET (P40-P47.agc C p.686): P40TMP = DELTA T TO PREREAD - 5 S,
 * LONGCALL TIGBLNK, phase 4.21. MIDTOAV1 leaves at least TIMEDELT
 * (12.5 s) before TIG - 30 when it starts, but the integration runs
 * after that check, so a slow one leaves less than 5 s: the AGC then
 * calls LONGCALL with a zero or negative delay, LONGCYCL falls through
 * to LASTTIME and WAITLIST ends in WATLST0-: POODOO 1204 (WAITLIST.agc
 * C p.1221). The C passes 0 for any such delay, which LONGCALL turns
 * into the same POODOO.
 *
 * FUNCTION NAME: agc_csm_preburn_call_tigblnk
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ----------------------------------
 *   dt        agc_dp_t  I    MIDTOAV delay to PREREAD, cs 2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; the LONGCALL status (POODOO
 *          1204 for a delay <= 0, BAILOUT 1203 when the list is full)
 **********************************************************************/
agc_status_t agc_csm_preburn_call_tigblnk(agc_dp_t dt)
{
    agc_dp_t p40tmp = dt - SEC5; /* DELTA T TO PREREAD - 5 S, cs 2^28 */

    agc_erase_set_p40tmp(p40tmp);
    agc_status_t st = agc_waitlist_longcall((p40tmp > 0) ? (uint32_t) p40tmp : 0u, AGC_TASK_TIGBLNK); /* <= 0: POODOO 1204 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change((uint16_t) 020214);

    return (AGC_STATUS_OK); /* 4.21 = TIGBLNK         */
}

/**********************************************************************
 * P40BLNKR (p.686): blank the DSKY (not restart protected).
 *
 * FUNCTION NAME: agc_job_preburn_blank_dsky
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_preburn_blank_dsky(agc_job_ctx_t *ctx)
{
    return (agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0U, NULL));
}

/**********************************************************************
 * TIGBLNK (p.690): task 35 s before TIG: TIGAVEG in 5 s, CLOCKJOB display
 * off, DSKY blanked, 4.37.
 *
 * FUNCTION NAME: agc_task_tigblnk
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_task_tigblnk(void)
{
    agc_status_t st = agc_waitlist_call(SEC5, AGC_TASK_TIGAVEG); /* status of agc_waitlist_call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_state()->csm.nvword1 = 0;                                                      /* DISABLE HERE           */
    agc_status_t sw               = agc_exec_novac(AGC_PRIO14, AGC_JOB_PREBURN_BLANK_DSKY); /* status of agc_exec_novac */

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_phase_newphase(4, 037, true);

    return (AGC_STATUS_OK); /* 4.37 = TIGAVEG         */
}

/**********************************************************************
 * PRECHECK (p.693): start the average-G loop (PREREAD) unless SERVICER is
 * already running (restart).
 *
 * FUNCTION NAME: agc_task_precheck
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_task_precheck(void)
{
    if (agc_phase_get(5) != 0U)
    {
        return (AGC_STATUS_OK); /* DON'T START ANOTHER    */
    }

    return (agc_task_preread_csm()); /* PREREAD (CSM only: the registry has no LM PRECHECK) */
}

/**********************************************************************
 * TIGAVEG = TIG-30 (p.693): P40: V06N40 back on, TIG-5 in 24.96 s, 4.6;
 * P41 (P41REDSP): V16N85, TTG/0 in 29.96 s, 4.4. Then PRECHECK.
 *
 * FUNCTION NAME: agc_task_tigaveg
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_tigaveg(void)
{
    agc_guid_t  *g = agc_guid_state(); /* guidance erasables */
    agc_status_t st;                   /* status of the call */

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

/**********************************************************************
 * TTG/0 (p.688): P41 at TIG: TIGNOW job, CLOKTASK stopped.
 *
 * FUNCTION NAME: agc_task_ttg_0
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_ttg_0(void)
{
    agc_status_t st = agc_exec_novac(AGC_PRIO20, AGC_JOB_TIGNOW); /* status of agc_exec_novac */

    agc_flag_clear(AGC_FLAG_TIMRFLAG); /* P40CLK                 */

    return (st);
}

/**********************************************************************
 * TIG-5 (p.694): TIG-0 in 5 s, V99 flash, S40.13 (TIMEBURN) at PRIO20.
 *
 * FUNCTION NAME: agc_task_tig_5_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_tig_5_csm(void)
{
    agc_status_t st = agc_waitlist_call(SEC5, AGC_TASK_TIG_0); /* status of agc_waitlist_call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_state()->csm.nvword1 = AGC_CSM_NV_ENGINE_ENABLE_REQUEST; /* WILL CAUSE V99 FLASH   */
    agc_phase_change2((uint16_t) 040074, (uint16_t) 000033, NULL);

    return (agc_exec_findvac(AGC_PRIO20, AGC_JOB_TIMEBURN));
}

/**********************************************************************
 * TIG-0 (p.695): IGNFLAG set; ignite now if the crew has already answered
 * the V99 (ASTNFLAG), else wait for V99P.
 *
 * FUNCTION NAME: agc_task_tig_0_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_task_tig_0_csm(void)
{
    agc_flag_set(AGC_FLAG_IGNFLAG);

    if (!agc_flag_test(AGC_FLAG_ASTNFLAG))
    {
        return (AGC_STATUS_OK); /* WAIT FOR V99P          */
    }

    agc_guid_state()->csm.nvword1 = agc_csm_vn(6, 40); /* CLEAR THE V99         */
    agc_phase_change((uint16_t) 000614);               /* 4.61 = IGNITION        */

    return (agc_task_ignition_csm());
}
