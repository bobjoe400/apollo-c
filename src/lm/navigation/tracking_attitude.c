/**********************************************************************
 * FILE NAME: tracking_attitude.c
 *
 * PURPOSE:
 *   R61LEM PREFERRED TRACKING ATTITUDE ROUTINE and R65LEM (the fine
 *   preferred tracking attitude), LM only: "To compute the preferred
 *   tracking attitude of the LM to enable RR tracking of the CSM and to
 *   perform the maneuver to the preferred attitude." The +Z axis is
 *   pointed along the LOS 3 s ahead (VECPNT1 from the desired CDUs). In
 *   auto, a LOS more than 15 degrees off +Z is an R60 maneuver;
 *   otherwise the desired CDUs are set directly. R65 repeats the
 *   computation every 6 s, R65CNTR times. The caller continues at GENRET.
 *
 * AGC SOURCE:
 *   Luminary099 P20-P25.agc pp. 517-520 (R61LEM, R65LEM, R61C+L01 ..
 *   R61C+L4, R65WAIT); LM only.
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale       Description
 *   ----------  ---------  ---  ----------  ----------------------------
 *   RRTARGET    agc_vec_t  I    half unit   LOS in stable member axes
 *   CDUXD..ZD   CDU word   I/O  2^15/rev    desired CDUs (READCDUD)
 *   THETAD      CDU word   O    2^15/rev    CPHI, CTHETA, CPSI
 *   R65CNTR     int        I/O  -           R65 passes left
 *   GENRET      job id     I/O  -           caller's continuation
 *   R61FLAG PDSPFLAG 3AXISFLG flags  O  -   (USE_TRACKING_ATTITUDE ...)
 *
 * EXTERNAL REFERENCES:
 *   Name                       Description
 *   -------------------------  -----------------------------------------
 *   lmnav_los                  LPS20.1: LOS and vehicle attitude
 *   agc_imu_vecpoint_from_cdus VECPNT1: final gimbal angles
 *   agc_imu_ballangs           BALLANGS: FDAI angles (not in auto)
 *   agc_imu_attitude_maneuver  R60LEM attitude maneuver
 *   agc_lm_dap_setmindb/restordb  DAP deadband (SETMINDB, RESTORDB)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none here; LPS20.1 beyond 400 NM goes to the 526 range alarm job.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the rendezvous navigation job; R61C+L2 is a waitlist task
 *   that restarts the computation as a PRIO26 job.
 *
 * NOTES:
 *   Scales: LOS half unit; CDU words two's complement (one revolution =
 *   2^15).
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "../radar/radar.h"
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/lm/dap.h"

#define TA_3SECONDS ((agc_dp_t) 300)       /* 3SECONDS 2DEC 300         */
#define TA_COS15DEG ((agc_dp_t) 129644930) /* COS15DEG 2DEC 0.96593 B-1 */
#define TA_06SEC    600U                   /* 06SEC DEC 600             */
#define TA_250DEC   250U                   /* R65WAIT delay, 2.5 s */
#define TA_PRIO26   ((agc_prio_t) 026)     /* R61C+L2 job priority */

/**********************************************************************
 * R61LEM / R65LEM entry: GENRET, R61FLAG, then R61C+L01.
 * AGC: R61LEM (P20-P25.agc p.517)
 *
 * FUNCTION NAME: lmnav_tracking_attitude
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   r61       bool                   I    true for R61LEM (once), false for
 *                                         R65LEM (repeats)
 *   ret_job   agc_job_id_t           I    GENRET: job that continues the
 *                                         caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t lmnav_tracking_attitude(bool r61, agc_job_id_t ret_job)
{
    lmnav_state()->genret = ret_job;
    rz_flag_put(AGC_FLAG_USE_TRACKING_ATTITUDE, r61); /* R61FLAG  */

    return (rz_continue(AGC_JOB_TRACKING_ATTITUDE_COMPUTE));
}

/**********************************************************************
 * R61C+L06: R61 returns now; R65 waits 6 s and goes round again while
 * R65CNTR lasts.
 * AGC: R61C+L06 (P20-P25.agc p.519)
 *
 * FUNCTION NAME: ta_next
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the caller's continuation status (rz_continue)
 *          or the status of the 6 s waitlist call
 **********************************************************************/
static agc_status_t ta_next(void)
{
    lmnav_state_t *n = lmnav_state(); /* LM navigation state */

    if (agc_flag_test(AGC_FLAG_USE_TRACKING_ATTITUDE) || n->r65cntr <= 0)
    {
        return (rz_continue(n->genret)); /* R61C+L4  */
    }

    n->r65cntr--;

    return (agc_waitlist_call(TA_06SEC, AGC_TASK_TRACKING_ATTITUDE_WAIT));
}

/**********************************************************************
 * VECPNT1 from the desired CDUs (READCDUD): CPHI, CTHETA, CPSI.
 * AGC: VECPNT1 (R60_62.agc p.481)
 *
 * FUNCTION NAME: ta_angles
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   gimbal    uint16_t        O    CPHI, CTHETA, CPSI, CDU words (2^15 per
 *                                  revolution)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t ta_angles(agc_interp_t *ip, uint16_t gimbal[3])
{
    agc_vec_t z; /* HIUNITZ, unit 2^1 */

    z                     = agc_vec_make(0, 0, AGC_DP_HALF); /* HIUNITZ  */
    agc_status_t st       = agc_imu_set_scaxis(z);           /* status */
    agc_vec_t    pointvsm = rr_state()->rrtarget;            /* RRTARGET, unit 2^1 */

    if (st == AGC_STATUS_OK)
    {
        agc_imu_set_pointvsm(pointvsm);
        st = AGC_STATUS_OK;
    }

    agc_inhint();
    uint16_t cdud[3] = {agc_erase_cduxd(), agc_erase_cduyd(), agc_erase_cduzd()};

    agc_relint();

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_vecpoint_from_cdus(ip, cdud, pointvsm, z, gimbal);
    }

    for (int i = 0; i < 3 && st == AGC_STATUS_OK; i++)
    {
        agc_erase_set_thetad(i, gimbal[i]); /* CPHI ..  */
        st = AGC_STATUS_OK;
    }

    return (st);
}

/**********************************************************************
 * R61C+L01 .. R61LEM1 (p.518): LOS 3 s ahead (LPS20.1), final angles,
 * then the maneuver or the FDAI angles (not in auto: BALLANGS).
 *
 * FUNCTION NAME: agc_job_tracking_attitude_compute
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tracking_attitude_compute(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return (lmnav_wait(TA_250DEC)); /* R65WAIT  */
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    bool         too_far = false;                                                                                                         /* LOS over 400 NM */
    agc_status_t st      = lmnav_los(&ip, agc_dp_add(&ip, agc_rtb_loadtime(), TA_3SECONDS), AGC_JOB_TRACKING_ATTITUDE_COMPUTE, &too_far); /* status */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    if (too_far)
    {
        return (rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_ALARM));
    }

    uint16_t gimbal[3] = {0U, 0U, 0U};

    st = ta_angles(&ip, gimbal);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_grp2pc(AGC_JOB_TRACKING_ATTITUDE_COMPUTE);

    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return (lmnav_wait(TA_250DEC));
    }

    if (agc_imu_gn_auto() != 0)
    { /* R61C+L04 */
        uint16_t fdai[3]; /* FDAI angles (BALLANGS) */

        agc_imu_ballangs(&ip, gimbal, fdai);

        return (ta_next()); /* R61C+L06 */
    }

    /* R61 in auto: phi (the LOS from +Z, CDU*SMNB) over 15 degrees: R60
     * maneuver at minimum deadband; else the desired CDUs directly.        */
    agc_cdutrig_t trig;                     /* CDU sines and cosines */
    agc_vec_t     v = rr_state()->rrtarget; /* LOS, then in NB axes, 2^1 */

    agc_guid_cdu_smnb(&trig, &v);

    if (agc_dp_sub(&ip, v.c[2], TA_COS15DEG) >= 0)
    { /* R61LEM2  */
        agc_inhint();
        agc_erase_set_cduxd(gimbal[0]);
        agc_erase_set_cduyd(gimbal[1]);
        agc_erase_set_cduzd(gimbal[2]);
        agc_relint();

        return (ta_next());
    }

    agc_inhint();                         /* R61C+L05 */
    agc_lm_dap_zatteror();
    agc_status_t st_auto = AGC_STATUS_OK; /* SETMINDB status */

    st_auto = agc_lm_dap_setmindb(); /* reduce attitude error */
    agc_relint();

    if (st_auto != AGC_STATUS_OK)
    {
        return (st_auto);
    }

    rz_flag_put(AGC_FLAG_3AXISFLG, false);
    rz_flag_put(AGC_FLAG_PDSPFLAG, true);

    return (agc_imu_attitude_maneuver(ctx, AGC_JOB_TRACKING_ATTITUDE_MANEUVERED));
} /* agc_job_tracking_attitude_compute */

/**********************************************************************
 * After R60LEM: deadband back to the astronaut's value, priority display
 * flag off, R61C+L06.
 * AGC: R60LEM (R60_62.agc p.473)
 *
 * FUNCTION NAME: agc_job_tracking_attitude_maneuvered
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tracking_attitude_maneuvered(void)
{
    agc_inhint();
    agc_status_t st = agc_lm_dap_restordb(); /* RESTORDB status */

    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_grp2pc(AGC_JOB_TRACKING_ATTITUDE_MANEUVERED);
    rz_flag_put(AGC_FLAG_PDSPFLAG, false);

    return (ta_next());
}

/**********************************************************************
 * R61C+L2 (task): R61C+L01 as a PRIO26 job.
 * AGC: R61C+L2 (P20-P25.agc p.519)
 *
 * FUNCTION NAME: agc_task_tracking_attitude_wait
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or
 *          rescheduled itself, else the status of the waitlist or
 *          executive call that failed; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_tracking_attitude_wait(void)
{
    if (!AGC_ASSERT(lmnav_state()->genret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_findvac(TA_PRIO26, AGC_JOB_TRACKING_ATTITUDE_COMPUTE));
}
