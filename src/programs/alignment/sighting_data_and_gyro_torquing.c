/*
 * align_r54_r55.c -- the two alignment subroutines that talk to the crew
 * and are shared by every alignment program of both vehicles:
 *   R54 / CHKSDATA  star data test (angle between the catalogue pair
 *                   against the angle between the marked pair, VN_STAR_ANGLE_DIFF)
 *   R55             gyro torquing (CALCGTA, VN_GYRO_TORQUE_ANGLES, IMUPULSE, IMUSTALL)
 * Sources: P51-P53.agc, Comanche055 pp. 759-760 (R55 p. 758),
 * Luminary099 pp. 944-946.  The CSM and LM texts differ only in the CSM's
 * CLEANDSP before VN_STAR_ANGLE_DIFF and the R55 restart phase (4.31 CM, 4.21 LM).
 *
 * Both are interpretive subroutines entered with STQ QMIN; here the
 * caller passes the job to continue as (QMIN), and the astronaut answers
 * arrive as the continuation jobs below.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/programs/alignment_internal.h"

#define VN_STAR_ANGLE_DIFF \
    ALIGN_VN(6, 5) /* VB6N5  VN 605: star angle difference  */
#define VN_GYRO_TORQUE_ANGLES \
    ALIGN_VN(6, 93) /* VN_GYRO_TORQUE_ANGLES VN 0693: gyro torque angles     */
#define PH_TORQUE_CSM \
    ((uint16_t) 00314) /* R55.2 PHASCHNG, CM: group 4 phase 31 */
#define PH_TORQUE_LM \
    ((uint16_t) 00214) /* R55.2 PHASCHNG, LM: group 4 phase 21 */

/* VB6N5 flash with the three astronaut returns. */
static agc_status_t chksdata_flash(const agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    (void) AGC_ASSERT(agc_align_ram()->qmin != AGC_JOB_NONE);
    return align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_ANGLE_DIFF,
                       AGC_JOB_GOTOPOOH, AGC_JOB_CHKSDATA_PROCEED,
                       AGC_JOB_CHKSDATA_ENTER);
}

/* R55.1 flash. */
static agc_status_t gyro_angles_flash(const agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    (void) AGC_ASSERT(agc_align_ram()->qmin != AGC_JOB_NONE);
    return align_flash(ctx, AGC_DSKY_GOFLASH, VN_GYRO_TORQUE_ANGLES,
                       AGC_JOB_GOTOPOOH, AGC_JOB_GYRO_TORQUING_PULSE,
                       AGC_JOB_GYRO_TORQUING_RETURN);
}

/* After the IMUSTALL of R55.2: PHASCHNG 05024 (CM) / 04024 (LM), then
 * R55RET.                                                                 */
static agc_status_t gyro_torquing_done(void)
{
    const agc_status_t st =
        (agc_state_vehicle() == AGC_VEHICLE_CSM) ?
            align_phase_job(ALIGN_PH_05024, AGC_JOB_GYRO_TORQUING_RETURN,
                            ALIGN_PRIO13) :
            agc_phase_change(4, ALIGN_PH_04024);
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    return align_goto(agc_align_ram()->qmin); /* R55RET     */
}

/*
 * CHKSDATA alias R54 (Comanche pp. 759-760, Luminary pp. 945-946) --
 * FREEFLAG set; THETA = angle between STARAD and STARAD +6 (catalogue
 * pair, desired SM), 18D = angle between 6D and 12D (marked pair, present
 * SM); NORMTEM1 = |THETA - 18D| (revs) shown in VN_STAR_ANGLE_DIFF.
 * PROCEED (CHKSDA) returns with FREEFLAG set = accept the data; ENTER
 * (V32) clears FREEFLAG = reject; V34 terminates.
 */
agc_status_t agc_align_sighting_data_display(const agc_job_ctx_t *ctx,
                                             agc_job_id_t         qmin_return)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(qmin_return != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    ram->qmin            = qmin_return; /* STQ QMIN  */
    agc_status_t st      = agc_flag_set(AGC_FLAG_FREEFLAG);
    agc_interp_t ip;
    agc_interp_init(&ip);
    const agc_vec_t cat[2] = {ram->starad[0], ram->starad[1]};
    const agc_vec_t obs[2] = {ram->vac6, ram->vac12};
    agc_dp_t        diff   = 0;
    if (st == AGC_STATUS_OK)
    {
        st = agc_align_chksdata_angle(&ip, cat, obs, &diff); /* CHKSB/CHKSD */
    }
    if (st == AGC_STATUS_OK)
    {
        st = align_set_dsptem1(diff); /* NORMTEM1 = DSPTEM1    */
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        /* CAF ZERO, CLEANDSP: the job continues at the VN_STAR_ANGLE_DIFF flash.     */
        return align_flash(ctx, AGC_DSKY_CLEANDSP, 0, AGC_JOB_CHKSDATA_FLASH,
                           AGC_JOB_CHKSDATA_FLASH, AGC_JOB_CHKSDATA_FLASH);
    }
    return chksdata_flash(ctx);
}

/* CHKSDATA: the VN_STAR_ANGLE_DIFF flash (after CLEANDSP in the CSM). */
agc_status_t agc_job_chksdata_flash(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return chksdata_flash(ctx);
}

/* CHKSDA (PROCEED): GOTO QMIN with FREEFLAG still set. */
agc_status_t agc_job_chksdata_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_job_id_t ret = agc_align_ram()->qmin;
    (void) AGC_ASSERT(agc_flag_test(AGC_FLAG_FREEFLAG));
    return align_goto(ret);
}

/* ENTER / V32 after VN_STAR_ANGLE_DIFF: CLEAR FREEFLAG, GOTO QMIN. */
agc_status_t agc_job_chksdata_enter(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_status_t st = agc_flag_clear(AGC_FLAG_FREEFLAG);
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    return align_goto(agc_align_ram()->qmin);
}

/*
 * R55 (Comanche p. 758, Luminary p. 943) -- gyro torquing: CALCGTA turns
 * XDC.. (desired SM wrt present SM, half-unit) into OGC, IGC, MGC (revs,
 * also loaded into the OGC pulse buffer); PULSEM / R55.1 flash VN_GYRO_TORQUE_ANGLES.
 */
agc_status_t agc_align_gyro_torquing(const agc_job_ctx_t *ctx,
                                     agc_job_id_t         qmin_return)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(qmin_return != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    ram->qmin            = qmin_return; /* STQ QMIN  */
    agc_interp_t ip;
    agc_interp_init(&ip);
    const agc_status_t st =
        agc_imu_calcgta(&ip, &ram->xdc, ram->ogc); /* CALCGTA */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return gyro_angles_flash(ctx);
}

/* R55.1: VN_GYRO_TORQUE_ANGLES -- PROCEED torques (R55.2), ENTER skips it (R55RET). */
agc_status_t agc_job_gyro_torquing_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return gyro_angles_flash(ctx);
}

/*
 * R55.2: PHASCHNG, IMUPULSE on the OGC buffer (R55CDR ECADR OGC) and
 * IMUSTALL; a bad end is CURTAINS.  Re-entered when the gyros were busy.
 */
agc_status_t agc_job_gyro_torquing_pulse(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const bool   csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    agc_status_t st  = agc_phase_change(4, csm ? PH_TORQUE_CSM : PH_TORQUE_LM);
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_imu_pulse_buf(AGC_IMU_GBUF_OGC,
                           AGC_JOB_GYRO_TORQUING_PULSE); /* IMUPULSE  */
    if (st == AGC_STATUS_BUSY)
    {
        return AGC_STATUS_OK; /* asleep until the gyros free  */
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    bool done = false;
    st        = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_GYRO_TORQUING_PULSED,
                            &done); /* IMUSTALL */
    if ((st != AGC_STATUS_OK) || !done)
    {
        return (st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st;
    }
    return gyro_torquing_done();
}

/* After the IMUSTALL of R55.2: the restart phase, then R55RET. */
agc_status_t agc_job_gyro_torquing_pulsed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (align_stall_outcome(AGC_IMU_DEV_IMU) != AGC_STATUS_OK)
    {
        return AGC_STATUS_OK; /* CURTAINS done    */
    }
    return gyro_torquing_done();
}

/* R55RET (ENTER on VN_GYRO_TORQUE_ANGLES: no torquing): GOTO QMIN. */
agc_status_t agc_job_gyro_torquing_return(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_job_id_t ret = agc_align_ram()->qmin;
    (void) AGC_ASSERT(ret != AGC_JOB_NONE);
    return align_goto(ret);
}
