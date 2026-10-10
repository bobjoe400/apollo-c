/*
 * tracking_data.c -- R22LEM DATA READ ROUTINE, LM only (P20-P25.agc,
 * Luminary099 pp.504-507, 568-573): R22LEM, LRS22.1 (data read
 * subroutine 1), LRS22.2 (data read subroutine 2), N49DSP, R22RSTRT.
 *
 * "To process automatic RR mark data to update the state vector of either
 * LM or CSM as defined in the rendezvous navigation program (P20)."
 * LRS22.1 "reads rendezvous radar range and range-rate, trunnion and
 * shaft angles, three CDU values and time ... compares radar LOS with LOS
 * computed from state vectors to see if they are within three degrees".
 * LRS22.2 "checks if the RR LOS (i.e., the radar boresight vector) is
 * within 30 degrees of the LM +Z axis". The incorporation (LSR22.3) is in
 * lm/navigation/measurement_update.c.
 *
 * Scales: RM m 2^29, RDOTM m/cs 2^7, angles revolutions, times cs 2^28,
 * RRBORSIT half unit (NB).
 */
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
#include "agc/io/service_internal.h"
#include "agc/programs/alignment.h"

#define TD_RDOTCONV \
    ((agc_dp_t) -65748541) /* RDOTCONV 2DEC -.0019135344 B7:
                                             * RR rate count to m/cs 2^7   */
#define TD_RANGCONV \
    ((agc_dp_t) 95932926)                      /* RANGCONV 2DEC 2.859024 B-3:
                                             * RR range count to m 2^29    */
#define TD_THREEDEG      ((agc_dp_t) 2236962)  /* THREEDEG 2DEC .008333333    */
#define TD_30DEG         ((agc_dp_t) 22369621) /* 30DEG 2DEC .083333333       */
#define TD_1500DEC       1500U
#define TD_2SECS         200U
#define TD_PRIO27        ((agc_prio_t) 027)
#define TD_ALARM_LOS     ((agc_alarm_code_t) 00525) /* RR LOS not within 3 deg */
#define TD_CH33_RR_AUTO  000002u /* chan 33 bit 2 (0 = auto) */
#define TD_CH33_RR_SCALE 000004u /* chan 33 bit 3: RR range scale */

static agc_status_t td_stall(agc_job_id_t wake_as)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;
    agc_status_t st = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, wake_as, &res);
    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return st;
    }
    return rz_continue(wake_as);
}

static bool td_good(void)
{
    return agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) == AGC_IMU_STALL_GOOD;
}

/* READRDOT: one range-rate sample, wait for it.                         */
static agc_status_t td_read_rdot(void)
{
    agc_status_t st = agc_radar_rrrdot();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return td_stall(AGC_JOB_RENDEZVOUS_TRACKING_DATA_RANGE);
}

/* ---- R22LEM (pp.504-507) ---------------------------------------------- */
/* R22LEM: P20 over: end; no track flag: wait 15 s; the tracker enable
 * gone (a monitor reposition by R25 clears it): back to P20LEMA; RR not
 * in auto: P20LEMB5; CDUs being zeroed: R22LEM42; else LRS22.1.        */
agc_status_t agc_job_rendezvous_tracking_data(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    if (!agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        return AGC_STATUS_OK; /* ENDOFJOB */
    }
    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return lmnav_wait(TD_1500DEC); /* R22WAIT  */
    }
    if ((agc_chan_read(AGC_CHAN_CHAN12) & RR_CH12_TRACK) == 0U)
    {
        return agc_job_rendezvous_navigation_attitude(ctx); /* P20LEMA  */
    }
    if ((agc_chan_read(AGC_CHAN_CHAN33) & TD_CH33_RR_AUTO) != 0U)
    {
        /* DEVIATION: P20LEMB5 is entered through P20LEMB, which checks
         * the track flag and auto discrete again before it.           */
        return agc_job_rendezvous_navigation_acquire(ctx);
    }
    if (rr_radmodes_test(RR_RCDU0BIT))
    {
        return agc_job_rendezvous_tracking_data_next(ctx); /* R22LEM42 */
    }
    rz_phase(2, 00152);                    /* restart reads range rate again */
    rz_flag_put(AGC_FLAG_RNGSCFLG, false); /* LRS22.1  */
    agc_inhint();
    uint16_t scale =
        (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & TD_CH33_RR_SCALE);
    agc_erase_set_radmodes(
        (uint16_t) ((agc_erase_radmodes() & ~RR_RRRSBIT) | scale));
    agc_relint();
    return td_read_rdot();
}

/* LRS22.1 after the range-rate read: mark time (the middle of the
 * reading), range-rate count, IMU CDUs, time of the CDUs and the RR CDUs
 * (inhibited); then the range read. Bad read: error exit 1 (P20LEMC).  */
agc_status_t agc_job_rendezvous_tracking_data_range(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!td_good())
    {
        return agc_job_rendezvous_navigation_designate(ctx); /* EREXIT1  */
    }
    lmnav_state_t *n = lmnav_state();
    agc_inhint();
    n->mktime   = rr_timehold();
    n->rdotmsav = agc_radar_samplsum();
    n->aig      = agc_erase_cduy();
    n->amg      = agc_erase_cduz();
    n->aog      = agc_erase_cdux();
    n->cdutime  = rz_loadtime();
    n->tang[0]  = agc_erase_cdut();
    n->tang[1]  = agc_erase_cdus();
    agc_relint();
    agc_status_t st = agc_radar_rrrange();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return td_stall(AGC_JOB_RENDEZVOUS_TRACKING_DATA_MARK);
}

/* LRS22.1 conversions: RDOTM = count SL14 x RDOTCONV, RM = count x
 * RANGCONV SL2R, RRTRUN / RRSHAFT (CDULOGIC), RRBORSIT = RRNB(TANGNB),
 * and the downlink copy (RANGRDOT, TANGNB).                            */
static agc_status_t td_convert(agc_interp_t *ip, lmnav_state_t *n)
{
    rr_state_t *s = rr_state();
    agc_inhint();
    n->rangrdot[0] = agc_erase_dnrrange(0);
    n->rangrdot[1] = agc_erase_dnrrange(1);
    s->tangnb[0]   = n->tang[0];
    s->tangnb[1]   = n->tang[1];
    agc_relint();
    n->rdotm   = agc_dp_mul(ip, agc_dp_shl(ip, n->rdotmsav, 14), TD_RDOTCONV);
    n->rrtrun  = agc_guid_cdulogic(n->tang[0]);
    n->rrshaft = agc_guid_cdulogic(n->tang[1]);
    n->rm      = agc_dp_shl_round(
             ip, agc_dp_mul_trunc(agc_radar_samplsum(), TD_RANGCONV), 2);
    return agc_lm_rrnb(ip, n->tang[0], n->tang[1], &n->rrborsit);
}

/* After the range read: a range scale change takes another reading;
 * another failure is error exit 1. Else convert, LOS from the state
 * vectors at the CDU time (LPS20.1) rotated to NB with the mark CDUs
 * (TRG*SMNB); within 3 degrees of the radar LOS: R22LEM2, else alarm
 * 525 (DSPTEM1 = the angle, for noun 5).                                */
agc_status_t agc_job_rendezvous_tracking_data_mark(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!td_good())
    { /* CHEXERR  */
        return agc_flag_test(AGC_FLAG_RNGSCFLG) ?
                   td_read_rdot() :
                   agc_job_rendezvous_navigation_designate(ctx);
    }
    lmnav_state_t *n = lmnav_state();
    agc_interp_t   ip;
    agc_interp_init(&ip);
    agc_status_t st      = td_convert(&ip, n);
    bool         too_far = false;
    if (st == AGC_STATUS_OK)
    {
        st = lmnav_los(&ip, n->cdutime, AGC_JOB_RENDEZVOUS_TRACKING_DATA_MARK,
                       &too_far);
    }
    if (st != AGC_STATUS_OK)
    {
        return st; /* BUSY: asleep in INTSTALL  */
    }
    if (too_far)
    {
        return rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_ALARM);
    }
    agc_cdutrig_t trig;
    uint16_t      yzx[3] = {n->aig, n->amg, n->aog};
    agc_vec_t     los    = rr_state()->rrtarget;
    st                   = agc_guid_trg_smnb(&trig, yzx, &los);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_dp_t c = agc_dp_shl(&ip, agc_vec_dot(&ip, los, n->rrborsit), 1);
    n->dtheta  = agc_dp_acos(&ip, c);
    st         = rz_set_dsptem_dp(RZ_DSPTEM1, n->dtheta);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (agc_dp_sub(&ip, n->dtheta, TD_THREEDEG) < 0)
    { /* NORMEXIT */
        return agc_job_rendezvous_tracking_data_accept(ctx);
    }
    return agc_job_rendezvous_tracking_data_los_alarm(ctx);
}

/* Alarm 525: TERMINATE exits P20, PROCEED shows the angle (R22LEM1),
 * ENTER is illegal (the alarm again).                                   */
agc_status_t agc_job_rendezvous_tracking_data_los_alarm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_prio_alarm(ctx, TD_ALARM_LOS,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                         AGC_JOB_RENDEZVOUS_TRACKING_DATA_DTHETA,
                         AGC_JOB_RENDEZVOUS_TRACKING_DATA_LOS_ALARM);
}

/* R22LEM1: V06N05 (delta theta): PROCEED uses the mark, ENTER designates
 * again (P20LEMC).                                                      */
agc_status_t agc_job_rendezvous_tracking_data_dtheta(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    return rz_flash(ctx, AGC_DSKY_PRIODSP, 6U, 5U,
                    AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                    AGC_JOB_RENDEZVOUS_TRACKING_DATA_ACCEPT,
                    AGC_JOB_RENDEZVOUS_NAVIGATION_DESIGNATE);
}

/* R22LEM2 / LRS22.2: on the surface straight to the update; otherwise
 * with the track flag on, the RR boresight must be within 30 degrees of
 * +Z, else R61 and wait.                                                */
agc_status_t agc_job_rendezvous_tracking_data_accept(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return agc_job_rendezvous_tracking_data_update(ctx);
    }
    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return lmnav_wait(TD_1500DEC);
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_dp_t phi =
        agc_dp_acos(&ip, lmnav_state()->rrborsit.c[2]); /* 30DEGCHK */
    if (agc_dp_sub(&ip, phi, TD_30DEG) < 0)
    {
        return agc_job_rendezvous_tracking_data_update(ctx);
    }
    return lmnav_tracking_attitude(ctx, true,
                                   AGC_JOB_RENDEZVOUS_TRACKING_DATA_WAIT);
}

/* R22WAIT as a job (after R61).                                         */
agc_status_t agc_job_rendezvous_tracking_data_wait(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return lmnav_wait(TD_1500DEC);
}

/* R22LEM3: no update allowed (NOUPFLAG) or wanted (UPDATFLG off): the
 * next mark; else LSR22.3. DEVIATION: PHSPRDT2 = PRIO26 (the restart
 * priority) is not kept; the restart tables of the exec lane own it.   */
agc_status_t agc_job_rendezvous_tracking_data_update(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_flag_test(AGC_FLAG_NOUPFLAG) || !agc_flag_test(AGC_FLAG_UPDATFLG))
    {
        return agc_job_rendezvous_tracking_data_next(ctx);
    }
    return agc_job_rendezvous_tracking_data_incorporate(ctx);
}

/* R22LEM42: next mark in 2 s on the surface; else R65 twice, then R22.  */
agc_status_t agc_job_rendezvous_tracking_data_next(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    { /* R22LEM46 */
        return rz_delayjob(TD_2SECS, AGC_JOB_RENDEZVOUS_TRACKING_DATA);
    }
    lmnav_state()->r65cntr = 2;
    return lmnav_tracking_attitude(ctx, false,
                                   AGC_JOB_RENDEZVOUS_TRACKING_DATA);
}

/* R22LEM93 / R22LEM44: a mark was incorporated: count it; next mark in
 * 2 s on the surface, else R65 five times.                              */
agc_status_t agc_job_rendezvous_tracking_data_done(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    agc_erase_set_trkmkcnt((agc_sp_t) (agc_erase_trkmkcnt() + 1));
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return rz_delayjob(TD_2SECS, AGC_JOB_RENDEZVOUS_TRACKING_DATA);
    }
    lmnav_state()->r65cntr = 5;
    return lmnav_tracking_attitude(ctx, false,
                                   AGC_JOB_RENDEZVOUS_TRACKING_DATA);
}

/* ---- noun 49 (R22LEM96 .. N49DSP) -------------------------------------- */
/* R22LEM96: the state vector change is too large: V06N49 (R22DISP,
 * WHCHREAD) in its own job; this job polls N49FLAG.                     */
agc_status_t lmnav_n49_display(void)
{
    lmnav_state()->n49flag = 0;
    rz_phase(2, 04022);
    agc_status_t st =
        agc_exec_novac(TD_PRIO27, AGC_JOB_RENDEZVOUS_TRACKING_DATA_N49);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return rz_continue(AGC_JOB_RENDEZVOUS_TRACKING_DATA_N49_WAIT);
}

/* N49DSP: excessive state vector update; TERMINATE ends R22 and P20,
 * PROCEED N49FLAG = -1, RECYCLE N49FLAG positive.                       */
agc_status_t agc_job_rendezvous_tracking_data_n49(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_flash(ctx, AGC_DSKY_PRIODSP, 6U, 49U,
                    AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                    AGC_JOB_RENDEZVOUS_TRACKING_DATA_N49_PROCEED,
                    AGC_JOB_RENDEZVOUS_TRACKING_DATA_N49_RECYCLE);
}

agc_status_t agc_job_rendezvous_tracking_data_n49_proceed(agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    lmnav_state()->n49flag = -1;
    return AGC_STATUS_OK; /* ENDOFJOB */
}

agc_status_t agc_job_rendezvous_tracking_data_n49_recycle(agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    lmnav_state()->n49flag = 1;
    return AGC_STATUS_OK; /* ENDOFJOB */
}

/* The N49FLAG loop: still open: look again (the AGC loops in the
 * interpreter); PROCEED: incorporate (R22LEM7: GRP2PC, ASTOK); RECYCLE:
 * drop the mark (surface: wait 15 s; else R65 once, then R22).        */
agc_status_t agc_job_rendezvous_tracking_data_n49_wait(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_sp_t f = lmnav_state()->n49flag;
    if (f == 0)
    {
        return rz_continue(AGC_JOB_RENDEZVOUS_TRACKING_DATA_N49_WAIT);
    }
    if (f < 0)
    {
        rz_phase(2, 04022);                     /* GRP2PC   */
        return lmnav_incorporate_accepted(ctx); /* ASTOK    */
    }
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return lmnav_wait(TD_1500DEC);
    }
    lmnav_state()->r65cntr = 0;
    return lmnav_tracking_attitude(ctx, false,
                                   AGC_JOB_RENDEZVOUS_TRACKING_DATA);
}

/* R22RSTRT: after a restart during a radar read, one range-rate reading
 * that is not used, so that bad data is not incorporated.              */
agc_status_t agc_job_rendezvous_tracking_data_restart(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 00152);
    agc_status_t st = agc_radar_rrrdot();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return td_stall(AGC_JOB_RENDEZVOUS_TRACKING_DATA_RESTART_READ);
}

agc_status_t agc_job_rendezvous_tracking_data_restart_read(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!td_good())
    {
        return agc_job_rendezvous_navigation_designate(ctx);
    }
    return agc_job_rendezvous_tracking_data(ctx);
}
