/**********************************************************************
 * FILE NAME: rendezvous_final_attitude.c
 *
 * PURPOSE:
 *   R63 rendezvous final attitude (V89): the LM attitude that points its
 *   +Z (or +X) axis at the CSM one minute ahead: CSMCONIC and LEMCONIC to
 *   TSTART82, POINTVSM = NORMUNIT(REFSMMAT (RCSM - RLEM)), VECPOINT and
 *   BALLANGS, then V06N18 and R60LEM on PROCEED.
 *
 * AGC SOURCE:
 *   Luminary099 R60_62.agc pp. 338-341 (V89CALL, V89RECL, V89CALL1, ALINEZ, ALINEX)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   OPTIONX, +1   agc_sp_t  I/O  unscaled       axis option (V04N12)
 *   THETAD        uint16_t  O    CDU words      CPHI .. (VECPOINT)
 *   POINTVSM, SCAXIS  agc_vec_t O half-unit     to the IMU routines
 *   3AXISFLG      flag      O                   VECPOINT
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_endmark      DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_imu_*             IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_mat_vec           matrix arithmetic of the interpreter (interp.h)
 *   agc_dp_add            DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_clear        flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_rtb_normunit      RTB operations: NORMUNIT, 1STO2S (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   NORMUNIT, not UNIT, as the AGC (finding A1).
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"

#define VN_AXIS_OPTION ALIGN_VN(4, 12)   /* VB04N12 VN 412                 */
#define VN_BALL_ANGLES ALIGN_VN(6, 18)   /* VB06N18 VN 0618                */
#define OPTION_CODE    ((agc_sp_t) 3)    /* CAF THREE: OPTIONX             */
#define AXIS_Z         ((agc_sp_t) 1)    /* OPTIONX +1 = 1: Z axis         */
#define DP1MIN         ((agc_dp_t) 6000) /* DP1MIN 2DEC 6000 cs            */

/* TSTART82 and RONE (the CSM position) across the integration waits. */
static agc_dp_t  tstart82; /* TSTART82: time of the final attitude, cs 2^28 */
static agc_vec_t rone;     /* RONE: CSM position at TSTART82, m 2^29 */

/**********************************************************************
 * AGC: V89RECL (Luminary099 R63.agc p. 339)
 *
 * R63 line of sight (V89RECL): POINTVSM = NORMUNIT(REFSMMAT (RCSM - RLEM)),
 * half-unit SM axes; both positions in the same frame and scale.
 *
 * FUNCTION NAME: agc_rendezvous_final_attitude_los
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     IO   interpreter state (OVFIND, MPAC
 *                                     words)
 *   rcsm      agc_vec_t          I    CSM position (RONE), m 2^29
 *   rlem      agc_vec_t          I    LM position (RATT), m 2^29
 *   refsmmat  const agc_mat_t *  I    REFSMMAT, half-unit rows (reference
 *                                     -> present SM)
 *   pointvsm  agc_vec_t *        O    POINTVSM: the CSM line of sight in SM
 *                                     axes, half-unit (2^1)
 *
 * RETURNS: none
 **********************************************************************/
void agc_rendezvous_final_attitude_los(agc_interp_t *ip, agc_vec_t rcsm, agc_vec_t rlem, const agc_mat_t *refsmmat, agc_vec_t *pointvsm)
{
    const agc_vec_t los   = agc_vec_sub(ip, rcsm, rlem); /* RONE - RATT, B27 m */
    int             shift = 0;                           /* NORMUNIT pre-shift: 13 or 0 */

    /* MXV RTB NORMUNIT: a LOS of a few km has zero high words at B27 (and
     * MXV halves it once more), so NORMUNIT shifts it left 13 before UNIT;
     * plain UNIT would lose ~15 mrad at 1 km. */
    *pointvsm = agc_rtb_normunit(ip, agc_mat_vec(ip, refsmmat, los), &shift, NULL, NULL);
}

/**********************************************************************
 * AGC: V89CALL (Luminary099 R63.agc p. 339)
 *
 * V89CALL: R02BOTH (IMU orientation known, else its alarm); OPTIONX = 3,
 * OPTIONX +1 = 1 (Z axis); flash V04N12.
 *
 * FUNCTION NAME: agc_job_rendezvous_final_attitude
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
agc_status_t agc_job_rendezvous_final_attitude(agc_job_ctx_t *ctx)
{
    const agc_status_t st = agc_imu_status_check(); /* R02BOTH */

    if (st == AGC_STATUS_POODOO)
    {
        return (AGC_STATUS_OK); /* alarm 220/210, P00 started */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_optionx(0, OPTION_CODE);
    agc_erase_set_optionx(1, AXIS_Z);

    return (agc_job_rendezvous_final_attitude_option(ctx));
}

/**********************************************************************
 * AGC: ENDEXT (Luminary099 DISPLAY_INTERFACE_ROUTINES.agc p. 1372)
 *
 * V04N12: V34 ENDEXT, V33 V89RECL, data in (OPTIONX +1) flashes again.
 *
 * FUNCTION NAME: agc_job_rendezvous_final_attitude_option
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
agc_status_t agc_job_rendezvous_final_attitude_option(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_AXIS_OPTION, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_END, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_COMPUTE, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_OPTION));
}

/**********************************************************************
 * AGC: LEMCONIC (Luminary099 INTEGRATION_INITIALIZATION.agc p. 1214)
 *
 * LEMCONIC to TSTART82; POINTVSM from RONE - RATT.
 *
 * FUNCTION NAME: final_attitude_lm
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
static agc_status_t final_attitude_lm(const agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_integ_out_t o;                                                                                               /* integration output (RATT, VATT) */
    agc_status_t    st = align_integ(&ip, ALIGN_LEMCONIC, tstart82, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_LM_STATE, &o); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    agc_vec_t       pointvsm = agc_vec_zero();       /* POINTVSM, half-unit (2^1) */
    const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_rendezvous_final_attitude_los(&ip, rone, o.ratt, &refsmmat, &pointvsm);

    /*
     * V89CALL1: SCAXIS = UNITZ (option 1) or UNITX (option 2); VECPOINT ->
     * CPHI (THETAD); BALLANGS -> FDAIX..Z; flash V06N18.
     */
    const bool      z_axis = (agc_erase_optionx(1) == AXIS_Z);                                           /* OPTIONX +1 = 1: Z axis */
    const agc_vec_t scaxis = z_axis ? agc_vec_make(0, 0, AGC_DP_HALF) : agc_vec_make(AGC_DP_HALF, 0, 0); /* ALINEZ / ALINEX: half-unit (2^1) */
    uint16_t        gimbal[3];                                                                           /* CPHI, CTHETA, CPSI: CDU words */
    uint16_t        fdai[3];                                                                             /* FDAIX, Y, Z: CDU words */

    agc_imu_set_pointvsm(pointvsm);
    st = agc_imu_set_scaxis(scaxis);

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_vecpoint(&ip, pointvsm, scaxis, gimbal);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_thetad(i, gimbal[i]); /* CPHI     */
    }

    agc_imu_ballangs(&ip, gimbal, fdai);

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_BALL_ANGLES, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_END, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_MANEUVER, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_COMPUTE));
} /* final_attitude_lm */

/**********************************************************************
 * AGC: CSMCONIC (Luminary099 INTEGRATION_INITIALIZATION.agc p. 1214)
 *
 * CSMCONIC to TSTART82; RONE = RATT; then the LM.
 *
 * FUNCTION NAME: final_attitude_csm
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
static agc_status_t final_attitude_csm(const agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_integ_out_t    o;                                                                                                /* integration output (RATT, VATT) */
    const agc_status_t st = align_integ(&ip, ALIGN_CSMCONIC, tstart82, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_STATE, &o); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rone = o.ratt;

    return (final_attitude_lm(ctx));
}

/**********************************************************************
 * AGC: V89RECL (Luminary099 R63.agc p. 339)
 *
 * V89RECL: TSTART82 = now + 1 minute; the two states.
 *
 * FUNCTION NAME: agc_job_rendezvous_final_attitude_compute
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
agc_status_t agc_job_rendezvous_final_attitude_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    tstart82 = agc_dp_add(&ip, align_loadtime(), DP1MIN);

    return (final_attitude_csm(ctx));
}

/**********************************************************************
 * AGC: CSMCONIC (Luminary099 INTEGRATION_INITIALIZATION.agc p. 1214)
 *
 * CSMCONIC resumed after its wait.
 *
 * FUNCTION NAME: agc_job_rendezvous_final_attitude_csm_state
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
agc_status_t agc_job_rendezvous_final_attitude_csm_state(agc_job_ctx_t *ctx)
{
    return (final_attitude_csm(ctx));
}

/**********************************************************************
 * AGC: LEMCONIC (Luminary099 INTEGRATION_INITIALIZATION.agc p. 1214)
 *
 * LEMCONIC resumed after its wait.
 *
 * FUNCTION NAME: agc_job_rendezvous_final_attitude_lm_state
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
agc_status_t agc_job_rendezvous_final_attitude_lm_state(agc_job_ctx_t *ctx)
{
    return (final_attitude_lm(ctx));
}

/**********************************************************************
 * AGC: V06N18 (Luminary099 R60_62.agc p. 475)
 *
 * V06N18 PROCEED: CLEAR 3AXISFLG; R60LEM; then ENDEXT.
 *
 * FUNCTION NAME: agc_job_rendezvous_final_attitude_maneuver
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
agc_status_t agc_job_rendezvous_final_attitude_maneuver(agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_3AXISFLG);

    return (agc_imu_attitude_maneuver(ctx, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_END));
}

/**********************************************************************
 * AGC: ENDEXT (Luminary099 DISPLAY_INTERFACE_ROUTINES.agc p. 1372)
 *
 * ENDEXT: the extended verb is over.
 *
 * FUNCTION NAME: agc_job_rendezvous_final_attitude_end
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_rendezvous_final_attitude_end(void)
{
    return (agc_dsky_endmark());
}
