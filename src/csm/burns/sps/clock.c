/**********************************************************************
 * FILE NAME: clock.c
 *
 * PURPOSE:
 *   The once-a-second countdown display of the CSM burn programs
 *   (CLOKTASK, CLOCKJOB: TTOGO, V06N40, the V99 engine-enable request and
 *   the V97 engine-fail flash), DYNDISP (dynamic N85 in P41), the V99 and
 *   V97 responses (V99E/T/P, V97T/P/E and their tasks, QUICKIGN, V99FLASH),
 *   S40.6 (gimbal drive test and trim, PRE40.6) and DAPFIG (V46).
 *
 * AGC SOURCE:
 *   Comanche055 P40-P47.agc pp. 703-709, 729-734
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   TIG           agc_dp_t   I/O  cs 2^28      time of ignition
 *   TTOGO         agc_dp_t   O    cs 2^28      time to go (N40)
 *   CDUX/Y/Z      uint16_t   I    CDU counts   gimbal angles (N85)
 *   CSMMASS       agc_sp_t   I    kg 2^16      mass (MASSTMP)
 *   TVCPITCH/YAW  agc_sp_t   O    gimbal bits  TVC drive outputs (S40.6)
 *   DAPDATR1      uint16_t   I    -            vehicle configuration (V46)
 *   OPTIND        agc_sp_t   O    -            optics zero allowed
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_csm_spsoff        SPSOFF (ignition.c)
 *   agc_csm_tvczap        TVCZAP (ignition.c)
 *   agc_csm_dap_rcsdapon  RCSDAPON (csm/dap.h)
 *   agc_dsky_display      the CLOCKJOB displays (dsky.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None raised here; display, executive and waitlist failures come back
 *   as their status.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only. CLOKTASK, the V97/V99 tasks and S40.6 run as waitlist
 *   tasks; CLOCKJOB, DYNDISP and the verb responses as jobs. Restart
 *   groups 4 and 6.
 *
 * NOTES:
 *   NVWORD1 (agc_guid_t csm.nvword1) holds the display the CLOCKJOB
 *   shows: a VN code, the V99 request or the V97 flash.
 *
 * ALGORITHM:
 *   As in the listing.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

#define ONE_SEC           100    /* 1SEC, cs (CLOKTASK period) */
#define TAILOFF_DELAY     250    /* 2.5 s SPS tail-off before the RCS DAP, cs */
#define ENGINE_FAIL_DELAY 200    /* 2 s before R40ENABL (V97 re-enable), cs */
#define QUICKIGN_DELAY    30     /* 0.3 s from QUICKIGN to the V99 flash, cs */
#define GIMBAL_STEP       200    /* 2 s per gimbal drive step (GPI check)      */
#define GIMTRIM_DELAY     400    /* 4 s before the trim                       */
#define ACT_P2DEG         84     /* +2ACTDEG DEC +84: 2 deg at 85.41"/bit      */
#define ACT_M4DEG         (-168) /* -4ACTDEG                                   */

/**********************************************************************
 * CLOKTASK (p.703-704): with TIMRFLAG, TTOGO = TIME - TIG every second and
 * the CLOCKJOB (or DYNDISP for V06N85) run when NVWORD1 is non-zero;
 * without it, group 6 off and stop.
 *
 * FUNCTION NAME: agc_task_cloktask_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_task_cloktask_csm(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    if (!agc_flag_test(AGC_FLAG_TIMRFLAG))
    {
        agc_phase_kill(6);

        return (AGC_STATUS_OK); /* KILL RESTART           */
    }

    agc_erase_set_ttogo((agc_dp_t) agc_time_now_cs() - agc_erase_tig()); /* CLOCKON */
    agc_status_t st = agc_waitlist_call(ONE_SEC, AGC_TASK_CLOKTASK);     /* SETCLOCK */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (g->csm.nvword1 == agc_csm_vn(6, 85))
    {
        agc_status_t sj = agc_exec_findvac(AGC_PRIO27, AGC_JOB_DYNDISP); /* SETUPDYN      */

        if (sj != AGC_STATUS_OK)
        {
            return (sj);
        }
    }
    else if (g->csm.nvword1 != 0)
    {
        agc_status_t sw = agc_exec_novac(AGC_PRIO27, AGC_JOB_CLOCKJOB); /* status of agc_exec_novac */

        if (sw != AGC_STATUS_OK)
        {
            return (sw);
        }
    }

    agc_phase_set_tbase(6);

    return (AGC_STATUS_OK); /* SETTB6                 */
}

/**********************************************************************
 * CKNVWRD1 (p.705-706): +VN: show NVWORD1 (REGODSP); V99 flash: engine on
 * enable request; V97 flash: engine failed. The flashing paste is passed
 * as the verb of the CLOCPLAY VN (the AGC pastes it from NVWORD1).
 *
 * FUNCTION NAME: cknvwrd1
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
static agc_status_t cknvwrd1(agc_job_ctx_t *ctx)
{
    agc_sp_t nv = agc_guid_state()->csm.nvword1; /* NVWORD1: VN, V99 request or V97 flash; unscaled */

    agc_relint();

    if (nv > 0)
    {
        return (agc_dsky_display(ctx, AGC_DSKY_REGODSP, (uint16_t) nv, NULL)); /* NOFLASH */
    }

    if (nv == 0)
    {
        return (AGC_STATUS_OK); /* ENDOFJOB               */
    }

    if (nv == AGC_CSM_NV_ENGINE_FAIL_PROCEDURE)
    { /* FAILDSP                */
        agc_dsky_cont_t c97 = {AGC_JOB_ENGINE_FAIL_TERMINATE,
                               AGC_JOB_ENGINE_FAIL_PROCEED,
                               AGC_JOB_ENGINE_FAIL_REIGNITE};

        return (agc_dsky_display(ctx, AGC_DSKY_CLOCPLAY, agc_dsky_vn(97, 40), &c97));
    }

    agc_dsky_cont_t c99 = {AGC_JOB_ENGINE_ENABLE_TERMINATE,
                           AGC_JOB_ENGINE_ENABLE_PROCEED,
                           AGC_JOB_ENGINE_ENABLE_BYPASS}; /* ENGREQST */

    return (agc_dsky_display(ctx, AGC_DSKY_CLOCPLAY, agc_dsky_vn(99, 40), &c99));
}

/**********************************************************************
 * CLOCKJOB (p.705): fresh CDU trig for the displays, then CKNVWRD1.
 *
 * FUNCTION NAME: agc_job_clockjob
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_clockjob(agc_job_ctx_t *ctx)
{
    agc_guid_t    *g      = agc_guid_state(); /* guidance erasables */
    const uint16_t cdu[3] = {agc_erase_cduy(), agc_erase_cduz(), agc_erase_cdux()};

    agc_guid_quictrig(&g->trig, cdu);
    agc_inhint();

    return (cknvwrd1(ctx));
}

/**********************************************************************
 * DYNDISP (p.704): update N85 for the dynamic V06N85 of P41.
 *
 * FUNCTION NAME: agc_job_dyndisp
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_dyndisp(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_csm_convert_vg_body(&ip);
    agc_inhint();

    return (cknvwrd1(ctx));
}

/**********************************************************************
 * V99E / V99T (p.705): 6 off, 4 type C at PRIO27; TVCZAP; then P40RCS
 * (V16N85 post-burn) or POST41 (P00).
 *
 * FUNCTION NAME: engine_enable_finish
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   next      agc_job_id_t  I    job that follows the display
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t engine_enable_finish(agc_job_id_t next)
{
    agc_phase_var_t v = {AGC_RESTART_JOB, next, AGC_TASK_NONE, (int16_t) AGC_PRIO27, false, 0U};

    agc_phase_change2((uint16_t) 000006, (uint16_t) 005024, &v);
    agc_inhint();
    agc_csm_tvczap(); /* WIPE OUT TVC, CLOKTASK */
    agc_relint();

    return (agc_exec_job_yield(next));
}

/**********************************************************************
 * V99E (p.705-706): ENTER on the V99 flash: the SPS is bypassed;
 * ENGINE_ENABLE finish, then P40RCS (V16N85 post-burn RCS trim).
 *
 * FUNCTION NAME: agc_job_engine_enable_bypass
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_engine_enable_bypass(void)
{
    return (engine_enable_finish(AGC_JOB_SPS_BURN_RCS_TRIM));
}

/**********************************************************************
 * V99T (p.706): TERMINATE on the V99 flash (also the V97T flow):
 * ENGINE_ENABLE finish, then POST41 (P00).
 *
 * FUNCTION NAME: agc_job_engine_enable_terminate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_engine_enable_terminate(void)
{
    return (engine_enable_finish(AGC_JOB_POST41));
}

/**********************************************************************
 * V99P (p.706-707): engine enable. If TIG-0 has come (IGNFLAG) ignite in
 * 10 ms; in any case clear the V99 (V06N40). A second V99P (restart) only
 * clears it.
 *
 * FUNCTION NAME: agc_job_engine_enable_proceed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_job_engine_enable_proceed(void)
{
    agc_inhint();

    if (!agc_flag_test(AGC_FLAG_ASTNFLAG))
    { /* ASTNV99P               */
        agc_flag_set(AGC_FLAG_ASTNFLAG);

        if (agc_flag_test(AGC_FLAG_IGNFLAG))
        { /* ENDV99PI               */
            agc_status_t st = agc_waitlist_call(1, AGC_TASK_IGNITION); /* status of agc_waitlist_call */

            if (st != AGC_STATUS_OK)
            {
                agc_relint();

                return (st);
            }
        }
    }

    agc_guid_state()->csm.nvword1 = agc_csm_vn(6, 40); /* V99P/TIG             */
    agc_relint();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * V97T (p.706): engine failed, terminate: 6 off, 4.67 V97TTASK now.
 *
 * FUNCTION NAME: agc_job_engine_fail_terminate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_engine_fail_terminate(void)
{
    agc_phase_change2((uint16_t) 000006, (uint16_t) 040674, NULL);

    return (agc_waitlist_call(1, AGC_TASK_ENGINE_FAIL_SHUTDOWN)); /* TWIDDLE                */
}

/**********************************************************************
 * V97TTASK (p.706): CLOCKJOB off, MASSTMP = CSMMASS + 3 s of flow (not lost
 * because the thrust failed), SPSOFF, 4.71, V97TRCS after 2.5 s.
 *
 * FUNCTION NAME: agc_task_engine_fail_shutdown
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_engine_fail_shutdown(void)
{
    agc_guid_state()->csm.nvword1 = 0; /* DISABLE CLOCKJOB       */
    agc_csm_dap_tvc()->masstmp    = (agc_sp_t) (AGC_CSM_3MDOT + agc_erase_csmmass());
    agc_phase_var_t v             = {AGC_RESTART_TASK,
                                     AGC_JOB_NONE,
                                     AGC_TASK_ENGINE_FAIL_SHUTDOWN_RESTART,
                                     0,
                                     true,
                                     0U};

    agc_phase_change_var((uint16_t) 005014, &v);

    return (agc_task_engine_fail_shutdown_restart());
}

/**********************************************************************
 * V97TRCS (p.707), also the 4.71 restart point: SPSOFF, then the RCS
 * DAP after the tail-off delay.
 *
 * FUNCTION NAME: agc_task_engine_fail_shutdown_restart
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_engine_fail_shutdown_restart(void)
{
    agc_csm_spsoff();
    agc_phase_change((uint16_t) 000714); /* 4.71 = V97TRCS         */

    return (agc_waitlist_fixdelay(TAILOFF_DELAY, AGC_TASK_ENGINE_FAIL_RCS_DAP));
}

/**********************************************************************
 * V97TRCS (p.707): RCS DAP on in 0.6 s, V99T (TVCZAP, POST41).
 *
 * FUNCTION NAME: agc_task_engine_fail_rcs_dap
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_engine_fail_rcs_dap(void)
{
    agc_inhint();
    agc_status_t st = agc_csm_dap_rcsdapon(); /* status of agc_csm_dap_rcsdapon */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_relint();

    return (agc_exec_novac(AGC_PRIO27, AGC_JOB_ENGINE_ENABLE_TERMINATE));
}

/**********************************************************************
 * V97P (p.707): proceed: 4.73 V97PTASK now.
 *
 * FUNCTION NAME: agc_job_engine_fail_proceed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_engine_fail_proceed(void)
{
    agc_phase_change((uint16_t) 040734);

    return (agc_waitlist_call(1, AGC_TASK_ENGINE_FAIL_RESUME));
}

/**********************************************************************
 * V97PTASK (p.707-708): mass updates again, V06N40, IDLEFAIL (one R40
 * bypass) and STEERSW set, 4.13, R40ENABL in 2 s.
 *
 * FUNCTION NAME: agc_task_engine_fail_resume
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_engine_fail_resume(void)
{
    agc_csm_tvc_t *tvc = agc_csm_dap_tvc(); /* TVC DAP erasables */

    tvc->vcntr                    = tvc->engine_fail_vcntr;
    agc_guid_state()->csm.nvword1 = agc_csm_vn(6, 40);
    agc_flag_set(AGC_FLAG_IDLEFAIL);
    agc_flag_set(AGC_FLAG_STEERSW);
    agc_phase_change((uint16_t) 000134); /* 4.13 = R40ENABL        */

    return (agc_waitlist_fixdelay(ENGINE_FAIL_DELAY, AGC_TASK_ENGINE_FAIL_REENABLE));
}

/**********************************************************************
 * R40ENABL (p.707): IDLEFAIL reset (R40 enabled again), group 4 off.
 *
 * FUNCTION NAME: agc_task_engine_fail_reenable
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_task_engine_fail_reenable(void)
{
    agc_flag_clear(AGC_FLAG_IDLEFAIL);

    agc_phase_kill(4);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * V97E (p.708): re-ignite: 4.53 V97ETASK in 10 ms.
 *
 * FUNCTION NAME: agc_job_engine_fail_reignite
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_engine_fail_reignite(void)
{
    agc_phase_change((uint16_t) 040534);

    return (agc_waitlist_call(1, AGC_TASK_ENGINE_FAIL_REIGNITE_PREP));
}

/**********************************************************************
 * V97ETASK / SPSOFF97 (p.708): R1 of V06N40 forced to 59x59 (TIG high word
 * = -OCT24), V06N40, MASSTMP, 4.75, SPSOFF, 4.11, V97E40.6 after 2.5 s.
 *
 * FUNCTION NAME: agc_task_engine_fail_reignite_prep
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_engine_fail_reignite_prep(void)
{
    agc_erase_set_tig(agc_dp_pack((agc_sp_t) -024, 0)); /* CS OCT24 TS TIG   */
    agc_guid_state()->csm.nvword1 = agc_csm_vn(6, 40);
    agc_csm_dap_tvc()->masstmp    = (agc_sp_t) (AGC_CSM_3MDOT + agc_erase_csmmass());
    agc_phase_change((uint16_t) 000754);                /* 4.75 = SPSOFF97        */

    return (agc_task_spsoff97());
}

/**********************************************************************
 * SPSOFF97 (p.708), also the 4.75 restart point: SPSOFF, 4.11, then
 * V97E40.6 (gimbal trim) after the tail-off delay.
 *
 * FUNCTION NAME: agc_task_spsoff97
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_spsoff97(void)
{
    agc_csm_spsoff();
    agc_phase_change((uint16_t) 000114); /* 4.11 = V97E40.6        */

    return (agc_waitlist_fixdelay(TAILOFF_DELAY, AGC_TASK_ENGINE_FAIL_REIGNITE_TRIM));
}

/**********************************************************************
 * V97E40.6 / QUICKIGN (p.708-709): PRE40.6 (trim) now, RCS DAP on with the
 * narrow deadband left, 6.2 and a type C task; ASTNFLAG reset and IGNFLAG
 * set for an immediate V99 response; V99FLASH in 0.3 s.
 *
 * FUNCTION NAME: agc_task_engine_fail_reignite_trim
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_engine_fail_reignite_trim(void)
{
    agc_status_t st = agc_waitlist_call(1, AGC_TASK_GIMBAL_TRIM_RESTART); /* status of agc_waitlist_call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_inhint();
    agc_status_t sw = agc_csm_dap_rcsdapon(); /* status of agc_csm_dap_rcsdapon */

    if (sw != AGC_STATUS_OK)
    {
        agc_relint();

        return (sw);
    }

    agc_relint();
    agc_phase_var_t v = { AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_QUICKIGN, 0, true, 0U };

    agc_phase_change2((uint16_t) 000026, (uint16_t) 005014, &v);

    return (agc_task_quickign());
}

/**********************************************************************
 * QUICKIGN (p.708): ASTNFLAG reset, IGNFLAG set (the next V99 PROCEED
 * ignites at once), then the V99 flash after QUICKIGN_DELAY.
 *
 * FUNCTION NAME: agc_task_quickign
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_quickign(void)
{
    agc_flag_clear(AGC_FLAG_ASTNFLAG); /* CS PRIO14 MASK         */
    agc_flag_set(AGC_FLAG_IGNFLAG);    /* AD BIT13               */

    return (agc_waitlist_fixdelay(QUICKIGN_DELAY, AGC_TASK_ENGINE_ENABLE_FLASH));
}

/**********************************************************************
 * V99FLASH (p.709): V99 flash, 4.77 (TIG-0 now) and 3.3, S40.13 job.
 *
 * FUNCTION NAME: agc_task_engine_enable_flash
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_engine_enable_flash(void)
{
    agc_guid_state()->csm.nvword1 = AGC_CSM_NV_ENGINE_ENABLE_REQUEST;
    agc_phase_change2((uint16_t) 040774, (uint16_t) 000033, NULL);

    return (agc_exec_findvac(AGC_PRIO20, AGC_JOB_TIMEBURN));
}

/**********************************************************************
 * PRE40.6 (p.729): restart entry of S40.6, not while TVC has T5 (REDOTVC
 * restarts the actuators then): trim only, CNTR = +1 (restart entry).
 *
 * FUNCTION NAME: agc_task_gimbal_trim_restart
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_task_gimbal_trim_restart(void)
{
    if (agc_csm_tvc_on())
    {
        return (AGC_STATUS_OK); /* TVC: TASKOVER          */
    }

    agc_guid_state()->csm.mrkrtmp = -1;
    agc_csm_dap_tvc()->cntr       = 1;

    return (agc_task_gimbal_test_trim());
}

/**********************************************************************
 * S40.6 (p.730): optics inhibited, optics error counters disabled, TVC
 * enable and optics DAC disengaged (channel 12 bits 8, 11); 60 ms relay
 * latching.
 *
 * FUNCTION NAME: agc_task_gimbal_test_trim
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_gimbal_test_trim(void)
{
    agc_erase_set_optind(AGC_OPTIND_NEG0);        /* CS ZERO / TS OPTIND: -0, optics reserved */
    agc_chan_and(012, (uint16_t) ~(uint16_t) 02); /* WAND CHAN12 ~BIT2      */
    agc_chan_or(012, AGC_CSM_TVCOPTEN);           /* OCT02200               */

    return (agc_waitlist_fixdelay(6, AGC_TASK_GIMBAL_TEST_RELAYS));
}

/**********************************************************************
 * S40.6 after 60 ms (p.730): optics error counters enabled, 20 ms.
 *
 * FUNCTION NAME: agc_task_gimbal_test_relays
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_gimbal_test_relays(void)
{
    agc_chan_or(012, (uint16_t) 02); /* WOR CHAN12 BIT2        */

    return (agc_waitlist_fixdelay(2, AGC_TASK_GIMBAL_TEST_RSTRTST));
}

/**********************************************************************
 * OUTPUT (p.731): one gimbal increment into TVCPITCH (step < 3) or TVCYAW,
 * counter released (channel 14 bit 11 or 12), 2 s for the GPI check.
 *
 * FUNCTION NAME: gimbal_output
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   step      int         I    gimbal drive test step (0 = first)
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t gimbal_output(int step)
{
    static const agc_sp_t inc[3] = {ACT_P2DEG, ACT_M4DEG, ACT_P2DEG};
    bool                  yaw    = (step >= 3); /* yaw step (steps 3-5) */

    if (yaw)
    {
        agc_erase_set_tvcyaw(inc[step - 3]);
    }
    else
    {
        agc_erase_set_tvcpitch(inc[step]);
    }

    agc_chan_or(014, yaw ? (uint16_t) 04000 : (uint16_t) 02000); /* channel 14 bit 12 (yaw) or 11 (pitch) */

    return (agc_waitlist_fixdelay(GIMBAL_STEP, AGC_TASK_GIMBAL_TEST_STEP));
}

/**********************************************************************
 * RSTRTST (p.730-731): restart entry (CNTR > 0) trims at once; trim only
 * waits 4 s; else the drive test: +2, -4, +2 degrees in pitch, then yaw.
 *
 * FUNCTION NAME: agc_task_gimbal_test_rstrtst
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_gimbal_test_rstrtst(void)
{
    agc_guid_t    *g   = agc_guid_state();  /* guidance erasables */
    agc_csm_tvc_t *tvc = agc_csm_dap_tvc(); /* TVC DAP erasables */

    if (tvc->cntr > 0)
    {
        g->csm.gimbal_test_step = 7; /* GIMTRIM +2             */

        return (agc_task_gimbal_test_step());
    }

    tvc->cntr = g->csm.mrkrtmp; /* SAVES CNTR FOR RESTARTS */

    if (g->csm.mrkrtmp <= 0)
    {
        g->csm.gimbal_test_step = 7; /* GIMTRIM: WAIT 4 S      */

        return (agc_waitlist_fixdelay(GIMTRIM_DELAY, AGC_TASK_GIMBAL_TEST_STEP));
    }

    g->csm.gimbal_test_step = 0; /* GDTSETUP: PITCH        */

    return (gimbal_output(0));
}

/**********************************************************************
 * The return point of OUTPUT / GIMTRIM (TEMPR60): next drive step, or
 * after the six steps the 4 s wait, then the trims PACTOFF/YACTOFF out and
 * the counters released (channel 14 bits 11, 12: PRIO6).
 * AGC: TEMPR60 return of OUTPUT / GIMTRIM (P40-P47.agc p.731).
 *
 * FUNCTION NAME: agc_task_gimbal_test_step
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_task_gimbal_test_step(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    if (g->csm.gimbal_test_step < 5)
    {
        g->csm.gimbal_test_step++;

        return (gimbal_output(g->csm.gimbal_test_step));
    }

    if (g->csm.gimbal_test_step == 5)
    {
        g->csm.gimbal_test_step = 7; /* GIMTRIM                */

        return (agc_waitlist_fixdelay(GIMTRIM_DELAY, AGC_TASK_GIMBAL_TEST_STEP));
    }

    const agc_csm_dapcom_t *com = agc_csm_dap_com(); /* CSM DAP common erasables */

    agc_erase_set_tvcpitch(com->pactoff); /* CS ZERO AD PACTOFF     */
    agc_erase_set_tvcyaw(com->yactoff);

    agc_chan_or(014, (uint16_t) 06000);

    return (AGC_STATUS_OK); /* RELEASE THE COUNTERS   */
}

/**********************************************************************
 * DAPFIG (Comanche p.734, from V46 STABLISH): SIVB takeover off; by the
 * configuration in DAPDATR1 bits 14-13: CM alone -> no DAP (T5 idle, jets
 * off, KALCMANU killed), CSM or CSM/LM -> RCS DAP, Saturn -> SATSTKON;
 * exit through GOPIN.
 *
 * FUNCTION NAME: agc_job_dapfig
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_dapfig(void)
{
    agc_chan_and(012, (uint16_t) ~(uint16_t) AGC_CH012_SIVBTAKEOVER);
    unsigned int cfg = ((unsigned int) agc_erase_dapdatr1() >> 12) & 3U; /* DAPDATR1 bits 14-13 (MP BIT3): vehicle configuration */

    if (cfg == 3U)
    {
        return (agc_csm_satstkon());
    }

    agc_inhint();

    if (cfg == 0U)
    { /* NODAPUP                */
        agc_rupt_set_t5_task(AGC_TASK_T5IDLOC);
        agc_flag_clear(AGC_FLAG_DAPBIT1);
        agc_flag_clear(AGC_FLAG_DAPBIT2);
        agc_csm_dap_zerojet();
        agc_csm_dap_rcs()->holdflag = 1; /* KILL KALCMANU JOB      */
    }
    else
    {
        agc_status_t st = agc_csm_dap_rcsdapon(); /* RCSDAPUP               */

        if (st != AGC_STATUS_OK)
        {
            agc_relint();

            return (st);
        }
    }

    agc_relint();

    return (agc_extverb_gopin()); /* ENDFIG                 */
}
