/*
 * tracking_attitude_lm.c -- R61LEM PREFERRED TRACKING ATTITUDE ROUTINE and
 * R65LEM (the fine preferred tracking attitude), LM only (P20-P25.agc,
 * Luminary099 pp.517-520).
 *
 * "To compute the preferred tracking attitude of the LM to enable RR
 * tracking of the CSM and to perform the maneuver to the preferred
 * attitude." The +Z axis is pointed along the LOS 3 s ahead (VECPNT1 from
 * the desired CDUs). In auto, a LOS more than 15 degrees off +Z is an R60
 * maneuver; otherwise the desired CDUs are set directly. R65 repeats the
 * computation every 6 s, R65CNTR times. The caller continues at GENRET.
 *
 * Scales: LOS half unit; CDU words two's complement (one revolution =
 * 2^15).
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/imu/imu.h"
#include "agc/guidance/guidance.h"
#include "agc/lm/dap.h"
#include "internal.h"
#include "radar_lm.h"

#define TA_3SECONDS ((agc_dp_t)300)          /* 3SECONDS 2DEC 300         */
#define TA_COS15DEG ((agc_dp_t)129644930)    /* COS15DEG 2DEC 0.96593 B-1 */
#define TA_06SEC    600U                     /* 06SEC DEC 600             */
#define TA_250DEC   250U
#define TA_PRIO26   ((agc_prio_t)026)

/* R61LEM / R65LEM entry: GENRET, R61FLAG, then R61C+L01.                */
agc_status_t lmnav_tracking_attitude(const agc_job_ctx_t *ctx, bool r61,
                                     agc_job_id_t ret_job)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret_job != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    lmnav_state()->genret = ret_job;
    rz_flag_put(AGC_FLAG_USE_TRACKING_ATTITUDE, r61);         /* R61FLAG  */
    return rz_continue(AGC_JOB_TRACKING_ATTITUDE_COMPUTE);
}

/* R61C+L06: R61 returns now; R65 waits 6 s and goes round again while
 * R65CNTR lasts.                                                        */
static agc_status_t ta_next(void)
{
    lmnav_state_t *n = lmnav_state();
    if (agc_flag_test(AGC_FLAG_USE_TRACKING_ATTITUDE) || n->r65cntr <= 0) {
        return rz_continue(n->genret);                        /* R61C+L4  */
    }
    n->r65cntr--;
    return agc_waitlist_call(TA_06SEC, AGC_TASK_TRACKING_ATTITUDE_WAIT);
}

/* VECPNT1 from the desired CDUs (READCDUD): CPHI, CTHETA, CPSI.         */
static agc_status_t ta_angles(agc_interp_t *ip, uint16_t gimbal[3])
{
    agc_vec_t z = agc_vec_make(0, 0, AGC_DP_HALF);            /* HIUNITZ  */
    agc_status_t st = agc_imu_set_scaxis(z);
    agc_vec_t pointvsm = rr_state()->rrtarget;
    if (st == AGC_STATUS_OK) {
        st = agc_imu_set_pointvsm(pointvsm);
    }
    agc_inhint();
    uint16_t cdud[3] = { agc_erase_cduxd(), agc_erase_cduyd(), agc_erase_cduzd() };
    agc_relint();
    if (st == AGC_STATUS_OK) {
        st = agc_imu_vecpnt1(ip, cdud, pointvsm, z, gimbal);
    }
    for (int i = 0; i < 3 && st == AGC_STATUS_OK; i++) {
        st = agc_erase_set_thetad(i, gimbal[i]);              /* CPHI ..  */
    }
    return st;
}

/* R61 in auto: phi (the LOS from +Z, CDU*SMNB) over 15 degrees: R60
 * maneuver at minimum deadband; else the desired CDUs directly.        */
static agc_status_t ta_auto(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                            const uint16_t gimbal[3])
{
    agc_cdutrig_t trig;
    agc_vec_t v = rr_state()->rrtarget;
    agc_status_t st = agc_guid_cdu_smnb(&trig, &v);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (agc_dp_sub(ip, v.c[2], TA_COS15DEG) >= 0) {           /* R61LEM2  */
        agc_inhint();
        agc_erase_set_cduxd(gimbal[0]);
        agc_erase_set_cduyd(gimbal[1]);
        agc_erase_set_cduzd(gimbal[2]);
        agc_relint();
        return ta_next();
    }
    agc_inhint();                                             /* R61C+L05 */
    st = agc_lm_dap_zatteror();
    if (st == AGC_STATUS_OK) {
        st = agc_lm_dap_setmindb();             /* reduce attitude error */
    }
    agc_relint();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_flag_put(AGC_FLAG_3AXISFLG, false);
    rz_flag_put(AGC_FLAG_PDSPFLAG, true);
    return agc_imu_attitude_maneuver(ctx, AGC_JOB_TRACKING_ATTITUDE_MANEUVERED);
}

/* R61C+L01 .. R61LEM1 (p.518): LOS 3 s ahead (LPS20.1), final angles,
 * then the maneuver or the FDAI angles (not in auto: BALLANGS).        */
agc_status_t agc_job_tracking_attitude_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!agc_flag_test(AGC_FLAG_TRACKFLG)) {
        return lmnav_wait(TA_250DEC);                         /* R65WAIT  */
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    bool too_far = false;
    agc_status_t st = lmnav_los(&ip, agc_dp_add(&ip, rz_loadtime(), TA_3SECONDS),
                                AGC_JOB_TRACKING_ATTITUDE_COMPUTE, &too_far);
    if (st != AGC_STATUS_OK) {
        return st;                        /* BUSY: asleep in INTSTALL  */
    }
    if (too_far) {
        return rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_ALARM);
    }
    uint16_t gimbal[3] = { 0U, 0U, 0U };
    st = ta_angles(&ip, gimbal);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_phase(2, 04022);
    if (!agc_flag_test(AGC_FLAG_TRACKFLG)) {
        return lmnav_wait(TA_250DEC);
    }
    if (agc_imu_gn_auto() != 0) {                             /* R61C+L04 */
        uint16_t fdai[3];
        st = agc_imu_ballangs(&ip, gimbal, fdai);
        return (st != AGC_STATUS_OK) ? st : ta_next();
    }
    return ta_auto(ctx, &ip, gimbal);
}

/* After R60LEM: deadband back to the astronaut's value, priority display
 * flag off, R61C+L06.                                                   */
agc_status_t agc_job_tracking_attitude_maneuvered(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    agc_status_t st = agc_lm_dap_restordb();
    agc_relint();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_phase(2, 04022);
    rz_flag_put(AGC_FLAG_PDSPFLAG, false);
    return ta_next();
}

/* R61C+L2 (task): R61C+L01 as a PRIO26 job.                             */
agc_status_t agc_task_tracking_attitude_wait(void)
{
    (void)AGC_ASSERT(lmnav_state()->genret != AGC_JOB_NONE);
    return agc_exec_findvac(TA_PRIO26, AGC_JOB_TRACKING_ATTITUDE_COMPUTE);
}
