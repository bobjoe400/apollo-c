/**********************************************************************
 * FILE NAME: sighting_data_and_gyro_torquing.c
 *
 * PURPOSE:
 *   The two alignment subroutines that talk to the crew and are shared by
 *   every alignment program of both vehicles:
 *
 *   R54 / CHKSDATA  star data test (catalogue pair angle against the
 *                   marked pair angle, V06N05)
 *   R55             gyro torquing (CALCGTA, V06N93, IMUPULSE, IMUSTALL)
 *
 *   Both are interpretive subroutines entered with STQ QMIN; here the
 *   caller passes the job to continue as (QMIN), and the astronaut
 *   answers arrive as the continuation jobs below.
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 758-760 (R55, CHKSDATA)
 *   Luminary099 P51-P53.agc pp. 943-946 (R55, CHKSDATA)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   STARAD, 6D, 12D, XDC, OGC, QMIN: agc_align_ram()
 *   DSPTEM1       uint16_t  O    revs (DP)      NORMTEM1 shown in N05
 *   FREEFLAG      flag      O                   data accepted
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_imu_*             IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_phase_change      restart phase tables, CHECKMM (exec.h)
 *   agc_state_vehicle     vehicle state (state.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   The CSM and LM texts differ only in the CSM's CLEANDSP before V06N05
 *   and the R55 restart phase (4.31 CM, 4.21 LM).
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/programs/alignment.h"

#define VN_STAR_ANGLE_DIFF \
    ALIGN_VN(6, 5)     /* VB6N5  VN 605: star angle difference  */
#define VN_GYRO_TORQUE_ANGLES \
    ALIGN_VN(6, 93)    /* VN_GYRO_TORQUE_ANGLES VN 0693: gyro torque angles     */
#define PH_TORQUE_CSM \
    ((uint16_t) 00314) /* R55.2 PHASCHNG, CM: group 4 phase 31 */
#define PH_TORQUE_LM \
    ((uint16_t) 00214) /* R55.2 PHASCHNG, LM: group 4 phase 21 */

/**********************************************************************
 * AGC: VB6N5 (Comanche055 P51-P53.agc p. 761; Luminary099 P51-P53.agc p.
 *      947)
 *
 * VB6N5 flash with the three astronaut returns.
 *
 * FUNCTION NAME: chksdata_flash
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t chksdata_flash(const agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_ANGLE_DIFF, AGC_JOB_GOTOPOOH, AGC_JOB_CHKSDATA_PROCEED, AGC_JOB_CHKSDATA_ENTER));
}

/**********************************************************************
 * AGC: R55.1 (Comanche055 P51-P53.agc p. 759; Luminary099 P51-P53.agc p.
 *      945)
 *
 * R55.1 flash.
 *
 * FUNCTION NAME: gyro_angles_flash
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t gyro_angles_flash(const agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_GYRO_TORQUE_ANGLES, AGC_JOB_GOTOPOOH, AGC_JOB_GYRO_TORQUING_PULSE, AGC_JOB_GYRO_TORQUING_RETURN));
}

/**********************************************************************
 * AGC: IMUSTALL (Comanche055 IMU_MODE_SWITCHING_ROUTINES.agc p. 1441;
 *      Luminary099 IMU_MODE_SWITCHING_ROUTINES.agc p. 1330)
 *
 * After the IMUSTALL of R55.2: PHASCHNG 05024 (CM) / 04024 (LM), then
 * R55RET.
 *
 * FUNCTION NAME: gyro_torquing_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t gyro_torquing_done(void)
{
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        align_phase_job(ALIGN_PH_05024, AGC_JOB_GYRO_TORQUING_RETURN, ALIGN_PRIO13);
    }
    else
    {
        agc_phase_change(ALIGN_PH_04024);
    }

    return (align_goto(agc_align_ram()->qmin)); /* R55RET     */
}

/**********************************************************************
 * CHKSDATA alias R54 (Comanche pp. 759-760, Luminary pp. 945-946) --
 * FREEFLAG set; THETA = angle between STARAD and STARAD +6 (catalogue pair,
 * desired SM), 18D = angle between 6D and 12D (marked pair, present SM);
 * NORMTEM1 = |THETA - 18D| (revs) shown in VN_STAR_ANGLE_DIFF. PROCEED
 * (CHKSDA) returns with FREEFLAG set = accept the data; ENTER (V32) clears
 * FREEFLAG = reject; V34 terminates.
 *
 * FUNCTION NAME: agc_align_sighting_data_display
 *
 * ARGUMENTS:
 *   Argument     Type                   I/O  Description (scale)
 *   -----------  ---------------------  ---  ----------------------------
 *   ctx          const agc_job_ctx_t *  I    the running job (display
 *                                            continuations)
 *   qmin_return  agc_job_id_t           I    job to continue as (STQ QMIN)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_align_sighting_data_display(const agc_job_ctx_t *ctx, agc_job_id_t qmin_return)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->qmin = qmin_return; /* STQ QMIN  */
    agc_flag_set(AGC_FLAG_FREEFLAG);
    agc_interp_t ip;         /* interpreter state */

    agc_interp_init(&ip);
    const agc_vec_t cat[2] = {ram->starad[0], ram->starad[1]};
    const agc_vec_t obs[2] = {ram->vac6, ram->vac12};
    const agc_dp_t  diff   = agc_align_chksdata_angle(&ip, cat, obs); /* CHKSB/CHKSD: |THETA - 18D|, revs 2^0 */

    align_set_dsptem1(diff); /* NORMTEM1 = DSPTEM1    */

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        /* CAF ZERO, CLEANDSP: the job continues at the VN_STAR_ANGLE_DIFF flash.     */
        return (align_flash(ctx, AGC_DSKY_CLEANDSP, 0, AGC_JOB_CHKSDATA_FLASH, AGC_JOB_CHKSDATA_FLASH, AGC_JOB_CHKSDATA_FLASH));
    }

    return (chksdata_flash(ctx));
}

/**********************************************************************
 * AGC: CHKSDATA (Comanche055 P51-P53.agc p. 760; Luminary099 P51-P53.agc p.
 *      946)
 *
 * CHKSDATA: the VN_STAR_ANGLE_DIFF flash (after CLEANDSP in the CSM).
 *
 * FUNCTION NAME: agc_job_chksdata_flash
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_chksdata_flash(agc_job_ctx_t *ctx)
{
    return (chksdata_flash(ctx));
}

/**********************************************************************
 * AGC: CHKSDA (Comanche055 P51-P53.agc p. 761; Luminary099 P51-P53.agc p.
 *      946)
 *
 * CHKSDA (PROCEED): GOTO QMIN with FREEFLAG still set.
 *
 * FUNCTION NAME: agc_job_chksdata_proceed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_chksdata_proceed(void)
{
    return (align_goto(agc_align_ram()->qmin));
}

/**********************************************************************
 * AGC: ENTER (Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc p. 323;
 *      Luminary099 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc p. 406)
 *
 * ENTER / V32 after VN_STAR_ANGLE_DIFF: CLEAR FREEFLAG, GOTO QMIN.
 *
 * FUNCTION NAME: agc_job_chksdata_enter
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_chksdata_enter(void)
{
    agc_flag_clear(AGC_FLAG_FREEFLAG);

    return (align_goto(agc_align_ram()->qmin));
}

/**********************************************************************
 * R55 (Comanche p. 758, Luminary p. 943) -- gyro torquing: CALCGTA turns
 * XDC.. (desired SM wrt present SM, half-unit) into OGC, IGC, MGC (revs,
 * also loaded into the OGC pulse buffer); PULSEM / R55.1 flash
 * VN_GYRO_TORQUE_ANGLES.
 *
 * FUNCTION NAME: agc_align_gyro_torquing
 *
 * ARGUMENTS:
 *   Argument     Type                   I/O  Description (scale)
 *   -----------  ---------------------  ---  ----------------------------
 *   ctx          const agc_job_ctx_t *  I    the running job (display
 *                                            continuations)
 *   qmin_return  agc_job_id_t           I    job to continue as (STQ QMIN)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_align_gyro_torquing(const agc_job_ctx_t *ctx, agc_job_id_t qmin_return)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->qmin = qmin_return; /* STQ QMIN  */
    agc_interp_t ip;         /* interpreter state */

    agc_interp_init(&ip);
    agc_imu_calcgta(&ip, &ram->xdc, agc_imu_ogc()); /* CALCGTA */

    return (gyro_angles_flash(ctx));
}

/**********************************************************************
 * AGC: R55.1 (Comanche055 P51-P53.agc p. 759; Luminary099 P51-P53.agc p.
 *      945)
 *
 * R55.1: VN_GYRO_TORQUE_ANGLES -- PROCEED torques (R55.2), ENTER skips it
 * (R55RET).
 *
 * FUNCTION NAME: agc_job_gyro_torquing_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_gyro_torquing_display(agc_job_ctx_t *ctx)
{
    return (gyro_angles_flash(ctx));
}

/**********************************************************************
 * AGC: R55.2 (Comanche055 P51-P53.agc p. 759; Luminary099 P51-P53.agc p.
 *      945)
 *
 * R55.2: PHASCHNG, IMUPULSE on the OGC buffer (R55CDR ECADR OGC) and
 * IMUSTALL; a bad end is CURTAINS.  Re-entered when the gyros were busy.
 *
 * FUNCTION NAME: agc_job_gyro_torquing_pulse
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_gyro_torquing_pulse(void)
{
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* running on the CSM */

    agc_phase_change(csm ? PH_TORQUE_CSM : PH_TORQUE_LM);                               /* per vehicle */
    agc_status_t st = agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_GYRO_TORQUING_PULSE); /* IMUPULSE  */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* asleep until the gyros free  */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_GYRO_TORQUING_PULSED, &done); /* IMUSTALL */

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (gyro_torquing_done());
}

/**********************************************************************
 * AGC: IMUSTALL (Comanche055 IMU_MODE_SWITCHING_ROUTINES.agc p. 1441;
 *      Luminary099 IMU_MODE_SWITCHING_ROUTINES.agc p. 1330)
 *
 * After the IMUSTALL of R55.2: the restart phase, then R55RET.
 *
 * FUNCTION NAME: agc_job_gyro_torquing_pulsed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_gyro_torquing_pulsed(void)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_IMU); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    return (gyro_torquing_done());
}

/**********************************************************************
 * AGC: R55RET (Comanche055 P51-P53.agc p. 759; Luminary099 P51-P53.agc p.
 *      945)
 *
 * R55RET (ENTER on VN_GYRO_TORQUE_ANGLES: no torquing): GOTO QMIN.
 *
 * FUNCTION NAME: agc_job_gyro_torquing_return
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_gyro_torquing_return(void)
{
    return (align_goto(agc_align_ram()->qmin));
}
