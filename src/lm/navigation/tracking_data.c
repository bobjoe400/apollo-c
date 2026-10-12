/**********************************************************************
 * FILE NAME: tracking_data.c
 *
 * PURPOSE:
 *   R22LEM DATA READ ROUTINE, LM only: R22LEM, LRS22.1 (data read
 *   subroutine 1), LRS22.2 (data read subroutine 2), N49DSP, R22RSTRT.
 *   "To process automatic RR mark data to update the state vector of
 *   either LM or CSM as defined in the rendezvous navigation program
 *   (P20)." LRS22.1 "reads rendezvous radar range and range-rate,
 *   trunnion and shaft angles, three CDU values and time ... compares
 *   radar LOS with LOS computed from state vectors to see if they are
 *   within three degrees". LRS22.2 "checks if the RR LOS (i.e., the
 *   radar boresight vector) is within 30 degrees of the LM +Z axis". The
 *   incorporation (LSR22.3) is in lm/navigation/measurement_update.c.
 *
 * AGC SOURCE:
 *   Luminary099 P20-P25.agc pp. 504-507 (R22LEM, R22LEM1 .. R22LEM96,
 *   R22WAIT, R22RSTRT, N49DSP), pp. 568-573 (LRS22.1, LRS22.2); LM only
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   CDUX CDUY CDUZ CDUT CDUS  words  I  CDU  angles at the mark
 *   CHAN12 CHAN33  channels I  -          track enable, auto, scale
 *   RADMODES    word       I/O  -          RRRSBIT, RCDU0BIT
 *   RM RDOTM RRTRUN RRSHAFT MKTIME RRBORSIT DTHETA N49FLAG R65CNTR
 *   (lmnav_state_t)        O    m 2^29, m/cs 2^7, rev, cs 2^28
 *   DSPTEM1     agc_dp_t   O    rev 2^0    noun 5 angle
 *   TRKMKCNT    agc_sp_t   I/O  -          marks incorporated
 *   RNDVZFLG TRACKFLG SURFFLAG NOUPFLAG UPDATFLG RNGSCFLG  flags  I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                     Description
 *   -----------------------  -------------------------------------------
 *   agc_radar_rrrdot         RRRDOT / RRRANGE readings and RADSTALL
 *   agc_lm_rrnb              RRNB: radar LOS in NB axes
 *   lmnav_los                LPS20.1: LOS from the state vectors
 *   agc_guid_trg_smnb        TRG*SMNB with the mark CDUs
 *   lmnav_tracking_attitude  R61LEM / R65LEM
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   00525 (radar LOS not within 3 degrees of the state-vector LOS).
 *   Bad readings go back to the designate (P20LEMC, error exit 1).
 *   Display, radar and integration statuses are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of P20 (LM). DEVIATION (R22LEM3): PHSPRDT2 = PRIO26 is not kept
 *   (request to exec, review-prog-rendz P3).
 *
 * NOTES:
 *   Scales: RM m 2^29, RDOTM m/cs 2^7, angles revolutions, times cs
 *   2^28, RRBORSIT half unit (NB).
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "../radar/radar.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"

/* RDOTCONV 2DEC -.0019135344 B7: RR rate count to m/cs 2^7;
 * RANGCONV 2DEC 2.859024 B-3: RR range count to m 2^29.                 */
#define TD_RDOTCONV      ((agc_dp_t) -65748541)     /* RDOTCONV, B7          */
#define TD_RANGCONV      ((agc_dp_t) 95932926)      /* RANGCONV, B-3         */
#define TD_THREEDEG      ((agc_dp_t) 2236962)       /* THREEDEG 2DEC .008333333    */
#define TD_30DEG         ((agc_dp_t) 22369621)      /* 30DEG 2DEC .083333333       */
#define TD_1500DEC       1500U                      /* 15 s, cs */
#define TD_2SECS         200U                       /* 2 s, cs */
#define TD_PRIO27        ((agc_prio_t) 027)         /* PRIO27 */
#define TD_ALARM_LOS     ((agc_alarm_code_t) 00525) /* LOS off by > 3 deg */
#define TD_CH33_RR_AUTO  000002u                    /* chan 33 bit 2 (0 = auto) */
#define TD_CH33_RR_SCALE 000004u                    /* chan 33 bit 3: RR range scale */

/**********************************************************************
 * RADSTALL for the job: sleep, or (reading already over) continue at once
 * as wake_as.
 * AGC: RADSTALL (P20-P25.agc p.568)
 *
 * FUNCTION NAME: td_stall
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------
 *   wake_as   agc_job_id_t  I    job that continues after the reading
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (asleep or continued), else the
 *          RADSTALL status
 **********************************************************************/
static agc_status_t td_stall(agc_job_id_t wake_as)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;                              /* RADSTALL result */
    agc_status_t           st  = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, wake_as, &res); /* RADSTALL */

    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return (st);
    }

    return (rz_continue(wake_as));
}

/**********************************************************************
 * The reading in RADSTALL ended good (GOODEND).
 * AGC: RADSTALL return +2 (P20-P25.agc p.568)
 *
 * FUNCTION NAME: td_good
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true for GOODEND
 **********************************************************************/
static bool td_good(void)
{
    return (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) == AGC_IMU_STALL_GOOD);
}

/**********************************************************************
 * READRDOT: one range-rate sample, wait for it.
 * AGC: READRDOT (P20-P25.agc p.568)
 *
 * FUNCTION NAME: td_read_rdot
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the radar or RADSTALL status
 **********************************************************************/
static agc_status_t td_read_rdot(void)
{
    agc_status_t st = agc_radar_rrrdot(); /* RRRDOT */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (td_stall(AGC_JOB_RENDEZVOUS_TRACKING_DATA_RANGE));
}

/* ---- R22LEM (pp.504-507) ---------------------------------------------- */
/**********************************************************************
 * R22LEM: P20 over: end; no track flag: wait 15 s; the tracker enable
 * gone (a monitor reposition by R25 clears it): back to P20LEMA; RR not
 * in auto: P20LEMB5; CDUs being zeroed: R22LEM42; else LRS22.1.
 * AGC: R22LEM, LRS22.1 (P20-P25.agc pp.504, 568)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data
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
agc_status_t agc_job_rendezvous_tracking_data(agc_job_ctx_t *ctx)
{
    agc_phase_change(04022);

    if (!agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        return (AGC_STATUS_OK); /* ENDOFJOB */
    }

    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return (lmnav_wait(TD_1500DEC)); /* R22WAIT  */
    }

    if ((agc_chan_read(AGC_CHAN_CHAN12) & RR_CH12_TRACK) == 0U)
    {
        return (agc_job_rendezvous_navigation_attitude(ctx)); /* P20LEMA  */
    }

    if ((agc_chan_read(AGC_CHAN_CHAN33) & TD_CH33_RR_AUTO) != 0U)
    {
        /* DEVIATION: P20LEMB5 is entered through P20LEMB, which checks
         * the track flag and auto discrete again before it.           */
        return (agc_job_rendezvous_navigation_acquire(ctx));
    }

    if (rr_radmodes_test(RR_RCDU0BIT))
    {
        return (agc_job_rendezvous_tracking_data_next()); /* R22LEM42 */
    }

    agc_phase_change(00152);               /* a restart reads range rate again */
    rz_flag_put(AGC_FLAG_RNGSCFLG, false); /* LRS22.1  */
    agc_inhint();
    uint16_t scale;                        /* range scale */

    scale = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & TD_CH33_RR_SCALE);

    agc_erase_set_radmodes((uint16_t) ((agc_erase_radmodes() & ~RR_RRRSBIT) | scale));
    agc_relint();

    return (td_read_rdot());
} /* agc_job_rendezvous_tracking_data */

/**********************************************************************
 * LRS22.1 after the range-rate read: mark time (the middle of the
 * reading), range-rate count, IMU CDUs, time of the CDUs and the RR CDUs
 * (inhibited); then the range read. Bad read: error exit 1 (P20LEMC).
 * AGC: LRS22.1 (P20-P25.agc p.569)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_range
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_range(void)
{
    if (!td_good())
    {
        return (agc_job_rendezvous_navigation_designate()); /* EREXIT1  */
    }

    lmnav_state_t *n = lmnav_state(); /* LM navigation erasables */

    agc_inhint();
    n->mktime       = rr_timehold();
    n->rdotmsav     = agc_radar_samplsum();
    n->aig          = agc_erase_cduy();
    n->amg          = agc_erase_cduz();
    n->aog          = agc_erase_cdux();
    n->cdutime      = agc_rtb_loadtime();
    n->tang[0]      = agc_erase_cdut();
    n->tang[1]      = agc_erase_cdus();
    agc_relint();
    agc_status_t st = agc_radar_rrrange(); /* RRRANGE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (td_stall(AGC_JOB_RENDEZVOUS_TRACKING_DATA_MARK));
}

/**********************************************************************
 * After the range read: a range scale change takes another reading;
 * another failure is error exit 1. Else convert, LOS from the state
 * vectors at the CDU time (LPS20.1) rotated to NB with the mark CDUs
 * (TRG*SMNB); within 3 degrees of the radar LOS: R22LEM2, else alarm
 * 525 (DSPTEM1 = the angle, for noun 5).
 * AGC: LRS22.1, CHEXERR (P20-P25.agc pp.570-571)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_mark
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
agc_status_t agc_job_rendezvous_tracking_data_mark(agc_job_ctx_t *ctx)
{
    if (!td_good())
    { /* CHEXERR  */
        if (agc_flag_test(AGC_FLAG_RNGSCFLG))
        {
            return (td_read_rdot()); /* scale changed: read again */
        }

        return (agc_job_rendezvous_navigation_designate());
    }

    lmnav_state_t *n = lmnav_state(); /* LM navigation erasables */
    agc_interp_t   ip;                /* interpreter state */

    agc_interp_init(&ip);

    /**********************************************************************
     * LRS22.1 conversions: RDOTM = count SL14 x RDOTCONV, RM = count x
     * RANGCONV SL2R, RRTRUN / RRSHAFT (CDULOGIC), RRBORSIT = RRNB(TANGNB),
     * and the downlink copy (RANGRDOT, TANGNB).
     * AGC: LRS22.1 conversions (P20-P25.agc p.570)
     *
     * FUNCTION NAME: td_convert
     *
     * ARGUMENTS:
     *   Argument  Type             I/O  Description (scale)
     *   --------  ---------------  ---  -------------------------------
     *   ip        agc_interp_t *   I/O  interpreter state (OVFIND)
     *   n         lmnav_state_t *  I/O  the mark data
     *
     * RETURNS: agc_status_t -- AGC_STATUS_OK, else the RRNB status
     **********************************************************************/
    rr_state_t *s = rr_state(); /* radar erasables */

    agc_inhint();
    n->rangrdot[0]       = agc_erase_dnrrange(0);
    n->rangrdot[1]       = agc_erase_dnrrange(1);
    s->tangnb[0]         = n->tang[0];
    s->tangnb[1]         = n->tang[1];
    agc_relint();
    n->rdotm             = agc_dp_mul(&ip, agc_dp_shl(&ip, n->rdotmsav, 14), TD_RDOTCONV);
    n->rrtrun            = agc_guid_cdulogic(n->tang[0]);
    n->rrshaft           = agc_guid_cdulogic(n->tang[1]);
    n->rm                = agc_dp_shl_round(&ip, agc_dp_mul_trunc(agc_radar_samplsum(), TD_RANGCONV), 2);
    agc_status_t st      = agc_lm_rrnb(&ip, n->tang[0], n->tang[1], &n->rrborsit); /* conversions */
    bool         too_far = false;                                                  /* range > 400 NM */

    if (st == AGC_STATUS_OK)
    {
        st = lmnav_los(&ip, n->cdutime, AGC_JOB_RENDEZVOUS_TRACKING_DATA_MARK, &too_far);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    if (too_far)
    {
        return (rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_ALARM));
    }

    agc_cdutrig_t trig;                          /* CDUTRIG sines / cosines */
    uint16_t      yzx[3] = {n->aig, n->amg, n->aog};
    agc_vec_t     los    = rr_state()->rrtarget; /* SV LOS, 2^1 */

    agc_guid_trg_smnb(&trig, yzx, &los);
    agc_dp_t c; /* cos, 2^1 */

    c = agc_dp_shl(&ip, agc_vec_dot(&ip, los, n->rrborsit), 1);

    n->dtheta = agc_dp_acos(&ip, c);
    rz_set_dsptem_dp(RZ_DSPTEM1, n->dtheta);

    if (agc_dp_sub(&ip, n->dtheta, TD_THREEDEG) < 0)
    { /* NORMEXIT */
        return (agc_job_rendezvous_tracking_data_accept());
    }

    return (agc_job_rendezvous_tracking_data_los_alarm(ctx));
} /* agc_job_rendezvous_tracking_data_mark */

/**********************************************************************
 * Alarm 525: TERMINATE exits P20, PROCEED shows the angle (R22LEM1),
 * ENTER is illegal (the alarm again).
 * AGC: R22LEM 525 alarm (P20-P25.agc p.505)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_los_alarm
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
agc_status_t agc_job_rendezvous_tracking_data_los_alarm(agc_job_ctx_t *ctx)
{
    return (rz_prio_alarm(ctx, TD_ALARM_LOS,
                          AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                          AGC_JOB_RENDEZVOUS_TRACKING_DATA_DTHETA,
                          AGC_JOB_RENDEZVOUS_TRACKING_DATA_LOS_ALARM));
}

/**********************************************************************
 * R22LEM1: V06N05 (delta theta): PROCEED uses the mark, ENTER designates
 * again (P20LEMC).
 * AGC: R22LEM1 (P20-P25.agc p.505)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_dtheta
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
agc_status_t agc_job_rendezvous_tracking_data_dtheta(agc_job_ctx_t *ctx)
{
    agc_phase_change(04022);

    return (rz_flash(ctx, AGC_DSKY_PRIODSP, 6U, 5U, AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE, AGC_JOB_RENDEZVOUS_TRACKING_DATA_ACCEPT, AGC_JOB_RENDEZVOUS_NAVIGATION_DESIGNATE));
}

/**********************************************************************
 * R22LEM2 / LRS22.2: on the surface straight to the update; otherwise
 * with the track flag on, the RR boresight must be within 30 degrees of
 * +Z, else R61 and wait.
 * AGC: R22LEM2, LRS22.2 (P20-P25.agc pp.505, 572)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_accept
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_accept(void)
{
    agc_phase_change(04022);

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return (agc_job_rendezvous_tracking_data_update());
    }

    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return (lmnav_wait(TD_1500DEC));
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_dp_t phi = agc_dp_acos(&ip, lmnav_state()->rrborsit.c[2]); /* rev 2^0 */

    if (agc_dp_sub(&ip, phi, TD_30DEG) < 0)
    {
        return (agc_job_rendezvous_tracking_data_update());
    }

    return (lmnav_tracking_attitude(true, AGC_JOB_RENDEZVOUS_TRACKING_DATA_WAIT));
}

/**********************************************************************
 * R22WAIT as a job (after R61).
 * AGC: R22WAIT (P20-P25.agc p.504)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_wait
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_wait(void)
{
    return (lmnav_wait(TD_1500DEC));
}

/**********************************************************************
 * R22LEM3: no update allowed (NOUPFLAG) or wanted (UPDATFLG off): the
 * next mark; else LSR22.3. DEVIATION: PHSPRDT2 = PRIO26 (the restart
 * priority) is not kept; the restart tables of the exec lane own it.
 * AGC: R22LEM3 (P20-P25.agc p.506)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_update
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_update(void)
{
    if (agc_flag_test(AGC_FLAG_NOUPFLAG) || !agc_flag_test(AGC_FLAG_UPDATFLG))
    {
        return (agc_job_rendezvous_tracking_data_next());
    }

    return (agc_job_rendezvous_tracking_data_incorporate());
}

/**********************************************************************
 * R22LEM42: next mark in 2 s on the surface; else R65 twice, then R22.
 * AGC: R22LEM42, R22LEM46 (P20-P25.agc p.506)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_next
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_next(void)
{
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    { /* R22LEM46 */
        return (agc_svc_delayjob(TD_2SECS, AGC_JOB_RENDEZVOUS_TRACKING_DATA));
    }

    lmnav_state()->r65cntr = 2;

    return (lmnav_tracking_attitude(false, AGC_JOB_RENDEZVOUS_TRACKING_DATA));
}

/**********************************************************************
 * R22LEM93 / R22LEM44: a mark was incorporated: count it; next mark in
 * 2 s on the surface, else R65 five times.
 * AGC: R22LEM93, R22LEM44 (P20-P25.agc p.506)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_done
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_done(void)
{
    agc_phase_change(04022);
    agc_erase_set_trkmkcnt((agc_sp_t) (agc_erase_trkmkcnt() + 1));

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return (agc_svc_delayjob(TD_2SECS, AGC_JOB_RENDEZVOUS_TRACKING_DATA));
    }

    lmnav_state()->r65cntr = 5;

    return (lmnav_tracking_attitude(false, AGC_JOB_RENDEZVOUS_TRACKING_DATA));
}

/* ---- noun 49 (R22LEM96 .. N49DSP) -------------------------------------- */
/**********************************************************************
 * R22LEM96: the state vector change is too large: V06N49 (R22DISP,
 * WHCHREAD) in its own job; this job polls N49FLAG.
 * AGC: R22LEM96 (P20-P25.agc p.507)
 *
 * FUNCTION NAME: lmnav_residuals_display
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the NOVAC status
 **********************************************************************/
agc_status_t lmnav_residuals_display(void)
{
    lmnav_state()->n49flag = 0;
    agc_phase_change(04022);
    agc_job_id_t j         = AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESIDUALS; /* N49DSP */
    agc_status_t st        = agc_exec_novac(TD_PRIO27, j);               /* NOVAC */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rz_continue(AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESIDUALS_WAIT));
}

/**********************************************************************
 * N49DSP: excessive state vector update; TERMINATE ends R22 and P20,
 * PROCEED N49FLAG = -1, RECYCLE N49FLAG positive.
 * AGC: N49DSP (P20-P25.agc p.507)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_residuals
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
agc_status_t agc_job_rendezvous_tracking_data_residuals(agc_job_ctx_t *ctx)
{
    return (rz_flash(ctx, AGC_DSKY_PRIODSP, 6U, 49U, AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE, AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESIDUALS_PROCEED, AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESIDUALS_RECYCLE));
}

/**********************************************************************
 * N49DSP PROCEED: N49FLAG = -1 (incorporate the mark).
 * AGC: N49DSP (P20-P25.agc p.507)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_residuals_proceed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_residuals_proceed(void)
{
    lmnav_state()->n49flag = -1;

    return (AGC_STATUS_OK); /* ENDOFJOB */
}

/**********************************************************************
 * N49DSP RECYCLE: N49FLAG positive (drop the mark).
 * AGC: N49DSP (P20-P25.agc p.507)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_residuals_recycle
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_residuals_recycle(void)
{
    lmnav_state()->n49flag = 1;

    return (AGC_STATUS_OK); /* ENDOFJOB */
}

/**********************************************************************
 * The N49FLAG loop: still open: look again (the AGC loops in the
 * interpreter); PROCEED: incorporate (R22LEM7: GRP2PC, ASTOK); RECYCLE:
 * drop the mark (surface: wait 15 s; else R65 once, then R22).
 * AGC: R22LEM96 N49FLAG loop, R22LEM7 (P20-P25.agc p.507)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_residuals_wait
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
agc_status_t agc_job_rendezvous_tracking_data_residuals_wait(agc_job_ctx_t *ctx)
{
    agc_sp_t f = lmnav_state()->n49flag; /* N49FLAG, unscaled */

    if (f == 0)
    {
        return (rz_continue(AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESIDUALS_WAIT));
    }

    if (f < 0)
    {
        agc_phase_change(04022); /* GRP2PC   */

        return (lmnav_incorporate_accepted(ctx)); /* ASTOK    */
    }

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return (lmnav_wait(TD_1500DEC));
    }

    lmnav_state()->r65cntr = 0;

    return (lmnav_tracking_attitude(false, AGC_JOB_RENDEZVOUS_TRACKING_DATA));
}

/**********************************************************************
 * R22RSTRT: after a restart during a radar read, one range-rate reading
 * that is not used, so that bad data is not incorporated.
 * AGC: R22RSTRT (P20-P25.agc p.507)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_restart
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_restart(void)
{
    agc_phase_change(00152);
    agc_status_t st = agc_radar_rrrdot(); /* RRRDOT */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (td_stall(AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESTART_READ));
}

/**********************************************************************
 * R22RSTRT after the unused reading: a bad read designates again, else
 * R22LEM.
 * AGC: R22RSTRT (P20-P25.agc p.507)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_restart_read
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
agc_status_t agc_job_rendezvous_tracking_data_restart_read(agc_job_ctx_t *ctx)
{
    if (!td_good())
    {
        return (agc_job_rendezvous_navigation_designate());
    }

    return (agc_job_rendezvous_tracking_data(ctx));
}
