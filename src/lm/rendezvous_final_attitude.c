/*
 * rendezvous_final_attitude.c -- R63 rendezvous final attitude (LM),
 * R63.agc, Luminary099 pp. 338-341 (labels V89CALL, V89RECL, ALINEX,
 * V89CALL1, ALINEZ).  Extended verb V89: the attitude that points the
 * LM's +Z (option 1) or +X (option 2) axis at the CSM one minute from
 * now; the FDAI ball angles are shown on V06N18 and, on PROCEED, R60LEM
 * flies the maneuver.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/programs/alignment_internal.h"

#define VN_AXIS_OPTION ALIGN_VN(4, 12)   /* VB04N12 VN 412                 */
#define VN_BALL_ANGLES ALIGN_VN(6, 18)   /* VB06N18 VN 0618                */
#define OPTION_CODE    ((agc_sp_t) 3)    /* CAF THREE: OPTIONX             */
#define AXIS_Z         ((agc_sp_t) 1)    /* OPTIONX +1 = 1: Z axis         */
#define DP1MIN         ((agc_dp_t) 6000) /* DP1MIN 2DEC 6000 cs            */

/* TSTART82 and RONE (the CSM position) across the integration waits. */
static agc_dp_t  tstart82;
static agc_vec_t rone;

/*
 * R63 line of sight (V89RECL): POINTVSM = NORMUNIT(REFSMMAT (RCSM -
 * RLEM)), half-unit SM axes; both positions in the same frame and scale.
 */
agc_status_t agc_rendezvous_final_attitude_los(agc_interp_t *ip, agc_vec_t rcsm,
                                               agc_vec_t        rlem,
                                               const agc_mat_t *refsmmat,
                                               agc_vec_t       *pointvsm)
{
    if (!AGC_ASSERT((ip != NULL) && (refsmmat != NULL)) ||
        !AGC_ASSERT(pointvsm != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_vec_t los = agc_vec_sub(ip, rcsm, rlem);
    *pointvsm = agc_vec_unit(ip, agc_mat_vec(ip, refsmmat, los), NULL, NULL);
    return AGC_STATUS_OK;
}

/*
 * V89CALL: R02BOTH (IMU orientation known, else its alarm); OPTIONX = 3,
 * OPTIONX +1 = 1 (Z axis); flash V04N12.
 */
agc_status_t agc_job_rendezvous_final_attitude(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_imu_status_check() != AGC_STATUS_OK)
    {
        return AGC_STATUS_OK; /* alarm 220/210, ended  */
    }
    agc_erase_set_optionx(0, OPTION_CODE);
    agc_erase_set_optionx(1, AXIS_Z);
    return agc_job_rendezvous_final_attitude_option(ctx);
}

/* V04N12: V34 ENDEXT, V33 V89RECL, data in (OPTIONX +1) flashes again. */
agc_status_t agc_job_rendezvous_final_attitude_option(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    (void) AGC_ASSERT(agc_erase_optionx(0) == OPTION_CODE);
    return align_flash(ctx, AGC_DSKY_GOFLASH, VN_AXIS_OPTION,
                       AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_END,
                       AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_COMPUTE,
                       AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_OPTION);
}

/*
 * V89CALL1: SCAXIS = UNITZ (option 1) or UNITX (option 2); VECPOINT ->
 * CPHI (THETAD); BALLANGS -> FDAIX..Z; flash V06N18.
 */
static agc_status_t final_attitude_angles(const agc_job_ctx_t *ctx,
                                          agc_vec_t            pointvsm)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    const bool      z_axis = (agc_erase_optionx(1) == AXIS_Z);
    const agc_vec_t scaxis = z_axis ?
                                 agc_vec_make(0, 0, AGC_DP_HALF) /* ALINEZ */
                                 :
                                 agc_vec_make(AGC_DP_HALF, 0, 0); /* ALINEX */
    uint16_t        gimbal[3];
    uint16_t        fdai[3];
    agc_status_t    st = agc_imu_set_pointvsm(pointvsm);
    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_set_scaxis(scaxis);
    }
    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_vecpoint(&ip, pointvsm, scaxis, gimbal);
    }
    for (int i = 0; (i < 3) && (st == AGC_STATUS_OK); i++)
    {
        st = agc_erase_set_thetad(i, gimbal[i]); /* CPHI     */
    }
    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_ballangs(&ip, gimbal, fdai);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return align_flash(ctx, AGC_DSKY_GOFLASH, VN_BALL_ANGLES,
                       AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_END,
                       AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_MANEUVER,
                       AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_COMPUTE);
}

/* LEMCONIC to TSTART82; POINTVSM from RONE - RATT. */
static agc_status_t final_attitude_lm(const agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_integ_out_t o;
    agc_status_t    st =
        align_integ(&ip, ALIGN_LEMCONIC, tstart82,
                    AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_LM_STATE, &o);
    if (st == AGC_STATUS_BUSY)
    {
        return AGC_STATUS_OK; /* INTSTALL */
    }
    agc_vec_t       pointvsm = agc_vec_zero();
    const agc_mat_t refsmmat = agc_erase_refsmmat();
    if (st == AGC_STATUS_OK)
    {
        st = agc_rendezvous_final_attitude_los(&ip, rone, o.ratt, &refsmmat,
                                               &pointvsm);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return final_attitude_angles(ctx, pointvsm);
}

/* CSMCONIC to TSTART82; RONE = RATT; then the LM. */
static agc_status_t final_attitude_csm(const agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_integ_out_t    o;
    const agc_status_t st =
        align_integ(&ip, ALIGN_CSMCONIC, tstart82,
                    AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_CSM_STATE, &o);
    if (st == AGC_STATUS_BUSY)
    {
        return AGC_STATUS_OK; /* INTSTALL */
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rone = o.ratt;
    return final_attitude_lm(ctx);
}

/* V89RECL: TSTART82 = now + 1 minute; the two states. */
agc_status_t agc_job_rendezvous_final_attitude_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    tstart82 = agc_dp_add(&ip, align_loadtime(), DP1MIN);
    (void) AGC_ASSERT(tstart82 >= 0);
    return final_attitude_csm(ctx);
}

/* CSMCONIC resumed after its wait. */
agc_status_t agc_job_rendezvous_final_attitude_csm_state(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    (void) AGC_ASSERT(tstart82 >= 0);
    return final_attitude_csm(ctx);
}

/* LEMCONIC resumed after its wait. */
agc_status_t agc_job_rendezvous_final_attitude_lm_state(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    (void) AGC_ASSERT(tstart82 >= 0);
    return final_attitude_lm(ctx);
}

/* V06N18 PROCEED: CLEAR 3AXISFLG; R60LEM; then ENDEXT. */
agc_status_t agc_job_rendezvous_final_attitude_maneuver(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_flag_clear(AGC_FLAG_3AXISFLG);
    (void) AGC_ASSERT(!agc_flag_test(AGC_FLAG_3AXISFLG));
    return agc_imu_attitude_maneuver(ctx,
                                     AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE_END);
}

/* ENDEXT: the extended verb is over. */
agc_status_t agc_job_rendezvous_final_attitude_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    (void) AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM);
    return agc_dsky_endext();
}
