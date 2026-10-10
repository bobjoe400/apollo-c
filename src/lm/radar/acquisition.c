/*
 * acquisition.c -- rendezvous radar acquisition of the CSM, LM only
 * (P20-P25.agc, Luminary099 pp.508-516, 588-593):
 *   R21LEM   RR designate routine ("to point the rendezvous radar at the
 *            CSM until automatic acquisition of the CSM is accomplished")
 *   R23LEM   manual acquisition routine
 *   R24LEM   search routine and LRS24.1, the hexagonal search pattern
 *            about the LOS ("to acquire the CSM by a search pattern when
 *            the rendezvous radar has failed to acquire the CSM in the
 *            automatic tracking mode")
 *
 * NAMING: R21 uses the canonical stem rendezvous_sighting_mark; R23LEM
 * (LM) is "manual acquisition"; R24 is "radar search".
 *
 * Scales: LOS half unit (SM or reference axes), times cs 2^28, angles
 * revolutions, CDU words two's complement.
 */
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "agc/io/service_internal.h"
#include "agc/lm/dap.h"
#include "agc/programs/alignment.h"
#include "radar.h"

#define ACQ_MAXTRIES   60                     /* MAXTRIES DEC 60            */
#define ACQ_HALFSEC    ((agc_dp_t) 50)        /* HALFSEC 2DEC 50            */
#define ACQ_TENSEC     ((agc_dp_t) 1000)      /* TENSEC 2DEC 1000 B-28      */
#define ACQ_1_5SECS    ((agc_dp_t) 150)       /* 1.5SECS 2DEC 150           */
#define ACQ_6SECONDS   600U                   /* 6SECONDS DEC 600           */
#define ACQ_DELAY_HALF 50U                    /* .5SEC                      */
#define ACQ_SIN60      ((agc_dp_t) 232473158) /* SIN60DEG 2DEC .86603     */
#define ACQ_COS60      AGC_DP_HALF            /* COS60DEG = DPHALF          */
#define ACQ_OFFSTFAC \
    ((agc_dp_t) 15241765)          /* OFFSTFAC 2DEC 0.05678:
                                             * tangent of 3.25 degrees    */
#define ACQ_ALL1S            11111 /* ALL1S DEC 11111            */
#define ACQ_PRIO25           ((agc_prio_t) 025)
#define ACQ_PRIO26           ((agc_prio_t) 026)
#define ACQ_CHECKLIST_MANUAL 0205U /* OCT205: manual acquisition */
#define ACQ_ALARM_NO_LOCK    ((agc_alarm_code_t) 00503)
#define ACQ_ALARM_COVERAGE   ((agc_alarm_code_t) 00530)
#define ACQ_ALARM_LIMITS     ((agc_alarm_code_t) 00501)
#define ACQ_ALARM_SEARCH     ((agc_alarm_code_t) 00527)
#define ACQ_BIT7             000100u /* LINUS: priority display    */
#define ACQ_W_M180           040000u /* BIT15: trunnion -180       */
#define ACQ_W_M90            057777u /* CS HALF: shaft -90         */

/* RADSTALL for the job: sleep, or (reading already over) continue at
 * once as wake_as.                                                       */
static agc_status_t acq_stall(agc_job_id_t wake_as)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;
    agc_status_t st = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, wake_as, &res);
    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return st;
    }
    return rz_continue(wake_as);
}

static bool acq_good(void)
{
    return agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) == AGC_IMU_STALL_GOOD;
}

/* ---- R21LEM (pp.508-512) ---------------------------------------------- */
/* R21LEM: tracker off; first drive the antenna to the mode centre (mode
 * 1: T = S = 0; on the surface mode 2: T = -180, S = -90) unless already
 * in mode 2 on the surface.                                             */
agc_status_t agc_job_rendezvous_sighting_mark(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_TRACK);
    uint16_t t = 0U;
    uint16_t s = 0U;
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    { /* R21LEM5  */
        if (rr_mode2())
        {
            return agc_job_rendezvous_sighting_mark_designate_los(ctx);
        }
        t = ACQ_W_M180;
        s = ACQ_W_M90;
    }
    rr_state()->tang[0] = (agc_sp_t) rr_ones(t);
    rr_state()->tang[1] = (agc_sp_t) rr_ones(s);
    rz_flag_put(AGC_FLAG_LOKONSW, false); /* R21LEM6  */
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rr_designate_result_t res = AGC_RR_DESIGNATE_END_JOB;
    agc_status_t              st  = agc_rr_designate_nb(&ip, t, s, &res);
    if (st != AGC_STATUS_OK || res != AGC_RR_DESIGNATE_STARTED)
    {
        return st;
    }
    return acq_stall(AGC_JOB_RENDEZVOUS_SIGHTING_MARK_CENTERED);
}

/* After the mode-centre designate: bad end -> alarm 503; else R21LEM10. */
agc_status_t agc_job_rendezvous_sighting_mark_centered(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!acq_good())
    {
        return agc_job_rendezvous_sighting_mark_alarm(ctx);
    }
    return agc_job_rendezvous_sighting_mark_designate_los(ctx);
}

/* R21LEM10: LOS computed on every fourth DODES pass, 60 passes (about 45
 * seconds) to designate and lock on; then R21LEM2.                     */
agc_status_t agc_job_rendezvous_sighting_mark_designate_los(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_LOSCMFLG, true);
    rr_state()->descount = ACQ_MAXTRIES;
    return agc_job_rendezvous_sighting_mark_new_los(ctx);
}

/* R21LEM2 / R21LEM1 / R21LEM3: LOSCOUNT = 3; LOS .5 s ahead (LPS20.1);
 * lock-on wanted, gimbal monitor on; RRDESSM: no mode 2 coverage on the
 * surface -> the reposition search R21LEM4; maneuver -> P20LEMA; else
 * wait in RADSTALL for lock-on.                                          */
agc_status_t agc_job_rendezvous_sighting_mark_new_los(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rr_state()->loscount = 3;
    agc_interp_t ip;
    agc_interp_init(&ip);
    bool         too_far = false;
    agc_status_t st =
        lmnav_los(&ip, agc_dp_add(&ip, rz_loadtime(), ACQ_HALFSEC),
                  AGC_JOB_RENDEZVOUS_SIGHTING_MARK_NEW_LOS, &too_far);
    if (st != AGC_STATUS_OK)
    {
        return st; /* BUSY: asleep in INTSTALL  */
    }
    if (too_far)
    {
        return rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_ALARM);
    }
    rz_flag_put(AGC_FLAG_LOKONSW, true); /* R21LEM3  */
    rz_flag_put(AGC_FLAG_NORRMON, false);
    agc_rr_designate_result_t res = AGC_RR_DESIGNATE_END_JOB;
    st                            = agc_rr_designate_sm(&ip, &res);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    switch (res)
    {
        case AGC_RR_DESIGNATE_NO_COVERAGE:
        { /* R21LEM4  */
            lmnav_state_t *n = lmnav_state();
            n->reposcnt      = ACQ_MAXTRIES;
            rz_flag_put(AGC_FLAG_FSPASFLG, true);
            rz_flag_put(AGC_FLAG_LOSCMFLG, false);
            n->repostm =
                agc_dp_add(&ip, rz_loadtime(), ACQ_TENSEC); /* R21LEM12 */
            return rz_continue(AGC_JOB_RENDEZVOUS_SIGHTING_MARK_REPOSITION);
        }
        case AGC_RR_DESIGNATE_MANEUVER:
            return rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_ATTITUDE);
        case AGC_RR_DESIGNATE_STARTED:
            return acq_stall(AGC_JOB_RENDEZVOUS_SIGHTING_MARK_LOCKON);
        case AGC_RR_DESIGNATE_END_JOB:
        case AGC_RR_DESIGNATE_RESULT_COUNT:
        default:
            return AGC_STATUS_OK; /* ENDOFJOB */
    }
}

/* After the designate: lock-on -> R21END (LOSCMFLG off, R21DISP); not in
 * 60 tries -> alarm 503.                                                */
agc_status_t agc_job_rendezvous_sighting_mark_lockon(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!acq_good())
    {
        return agc_job_rendezvous_sighting_mark_alarm(ctx);
    }
    rz_flag_put(AGC_FLAG_LOSCMFLG, false); /* R21END   */
    return agc_job_rendezvous_sighting_mark_verify(ctx);
}

/* R21-503: priority alarm 503; TERMINATE exits P20 (V56), PROCEED is the
 * search option (R21SRCH: R24LEM), ENTER designates again (P20LEMC3).  */
agc_status_t agc_job_rendezvous_sighting_mark_alarm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_prio_alarm(
        ctx, ACQ_ALARM_NO_LOCK, AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
        AGC_JOB_RADAR_SEARCH, AGC_JOB_RENDEZVOUS_NAVIGATION_UPDATE_STATES);
}

/* R21DISP (p.511): flash V50N72 "please perform RR main lobe lock-on
 * verification" as a priority display (GOPERF2R, LINUS BIT7); PROCEED
 * continues to R22 (P20LEMWT), TERMINATE exits through V56.            */
agc_status_t agc_job_rendezvous_sighting_mark_verify(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    agc_dsky_cont_t cont;
    cont.terminate = AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE;
    cont.proceed   = AGC_JOB_RENDEZVOUS_SIGHTING_MARK_WAIT;
    cont.enter     = AGC_JOB_RENDEZVOUS_SIGHTING_MARK_VERIFY; /* TC -5    */
    agc_status_t st =
        agc_dsky_display(ctx, AGC_DSKY_GOPERF2R, agc_dsky_vn(6U, 72U), &cont);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_dsky_blanket(ACQ_BIT7); /* LINUS    */
}

/* PROCEED on R21DISP: P20LEMWT.                                          */
agc_status_t agc_job_rendezvous_sighting_mark_wait(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return lmnav_wait(250U);
}

/* ---- R21LEM4 .. R21LEM11: the LOS is outside mode 2 coverage on the
 * lunar surface: look 10 s further ahead each time (60 tries, about 600
 * s) for a LOS the RR can reach, then designate continuously to it ahead
 * of time and come back to R21LEM10 at that time.                     */
agc_status_t agc_job_rendezvous_sighting_mark_reposition(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    lmnav_state_t *n = lmnav_state();
    agc_interp_t   ip;
    agc_interp_init(&ip);
    bool         too_far = false;
    agc_status_t st =
        lmnav_los(&ip, n->repostm, /* R21LEM12 */
                  AGC_JOB_RENDEZVOUS_SIGHTING_MARK_REPOSITION, &too_far);
    if (st != AGC_STATUS_OK)
    {
        return st; /* BUSY: asleep in INTSTALL  */
    }
    if (too_far)
    {
        return rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_ALARM);
    }
    agc_rr_designate_result_t res = AGC_RR_DESIGNATE_END_JOB;
    st                            = agc_rr_designate_sm(&ip, &res);
    if (st != AGC_STATUS_OK || res == AGC_RR_DESIGNATE_MANEUVER ||
        res == AGC_RR_DESIGNATE_END_JOB)
    {
        return st; /* ENDOFJOB */
    }
    if (res == AGC_RR_DESIGNATE_STARTED)
    {
        agc_inhint();
        st = agc_waitlist_kill(AGC_TASK_RR_DESIGNATE_BEGIN); /* BEGDES   */
        agc_relint();
        if (!agc_flag_test(AGC_FLAG_FSPASFLG))
        {
            return agc_job_rendezvous_sighting_mark_ahead(ctx); /* R21LEMB */
        }
        rz_flag_put(AGC_FLAG_FSPASFLG, false); /* R21LEM7+1 */
    }
    else if (n->reposcnt > 0)
    {                  /* R21LEM13 */
        n->reposcnt--; /* R21LEM7  */
    }
    else
    {
        return agc_job_rendezvous_sighting_mark_coverage_alarm(
            ctx); /* R21LEM11 */
    }
    n->repostm = agc_dp_add(&ip, n->repostm, ACQ_TENSEC);
    (void) AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_NOT_FOUND);
    return rz_continue(AGC_JOB_RENDEZVOUS_SIGHTING_MARK_REPOSITION);
}

/* R21LEMB: state vectors to REPOSTM (UPPSV); continuous designate (no
 * lock-on, no gimbal monitor) to the RR angles of that LOS (MODEA in
 * TANG); at REPOSTM (LONGCALL) R21LEM9.                                 */
agc_status_t agc_job_rendezvous_sighting_mark_ahead(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    lmnav_state_t *n = lmnav_state();
    rr_state_t    *s = rr_state();
    rr_radmodes_set(RR_CDESBIT); /* CDESFLAG (RADMODES) */
    rz_flag_put(AGC_FLAG_LOKONSW, false);
    rz_flag_put(AGC_FLAG_NORRMON, true);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rr_designate_result_t res = AGC_RR_DESIGNATE_END_JOB;
    agc_status_t              st = agc_rr_designate_nb(&ip, rr_word(s->tang[0]),
                                                       rr_word(s->tang[1]), &res);
    if (st != AGC_STATUS_OK || res != AGC_RR_DESIGNATE_STARTED)
    {
        return st;
    }
    agc_dp_t dt = agc_dp_sub(&ip, n->repostm, rz_loadtime()); /* DELTATM */
    if (!AGC_ASSERT(dt > 0))
    {
        dt = 1;
    }
    return agc_waitlist_longcall((uint32_t) dt,
                                 AGC_TASK_RENDEZVOUS_SIGHTING_MARK_REDESIGNATE);
}

/* R21LEM9 (task): stop the designate loop (STDESIG), CLRADMOD, and R21LEM10
 * as a PRIO26 job.                                                       */
agc_status_t agc_task_rendezvous_sighting_mark_redesignate(void)
{
    agc_status_t st = agc_waitlist_kill(AGC_TASK_RR_DESIGNATE_LOOP);
    if (st != AGC_STATUS_OK && st != AGC_STATUS_NOT_FOUND)
    {
        return st;
    }
    st = agc_rendz_clradmod();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_exec_findvac(ACQ_PRIO26,
                            AGC_JOB_RENDEZVOUS_SIGHTING_MARK_DESIGNATE_LOS);
}

/* R21LEM11: alarm 530, LOS not in coverage after trying for 600 s; every
 * response exits through V56.                                           */
agc_status_t agc_job_rendezvous_sighting_mark_coverage_alarm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_prio_alarm(ctx, ACQ_ALARM_COVERAGE,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE);
}

/* ---- R23LEM manual acquisition (pp.513-514) ----------------------------
 * "To acquire the CSM by manual operation of the rendezvous radar."    */
agc_status_t agc_job_manual_acquisition(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_NORRMON, true); /* no angle monitor  */
    agc_inhint();
    agc_status_t st = agc_lm_dap_setmindb(); /* minimum deadband  */
    agc_relint();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_manual_acquisition_display(ctx);
}

/* R23LEM1: tracker on; GOPERF1 205 (manual acquisition): TERMINATE ends
 * R23 and P20, PROCEED checks the lock, ENTER does another maneuver.   */
agc_status_t agc_job_manual_acquisition_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rr_chan_or(AGC_CHAN_CHAN12, RR_CH12_TRACK);
    agc_dsky_cont_t cont;
    cont.terminate = AGC_JOB_MANUAL_ACQUISITION_TERMINATE;
    cont.proceed   = AGC_JOB_MANUAL_ACQUISITION_CHECK;
    cont.enter     = AGC_JOB_MANUAL_ACQUISITION_MANEUVER;
    return agc_dsky_display(ctx, AGC_DSKY_GOPERF1, ACQ_CHECKLIST_MANUAL, &cont);
}

/* R23LEM11: antenna within the limits of its mode: deadband restored,
 * monitor on, ACMODFLG (P20LEMB1) and back to P20LEMB; else priority
 * alarm 501 (PROCEED is illegal: the alarm again).                    */
agc_status_t agc_job_manual_acquisition_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    if (!rr_limits_ok(agc_erase_cdut(), agc_erase_cdus()))
    {
        agc_relint();
        return agc_job_manual_acquisition_limit_alarm(ctx); /* OUTOFLIM */
    }
    agc_status_t st = agc_lm_dap_restordb();
    agc_relint();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rz_flag_put(AGC_FLAG_NORRMON, false);
    rz_flag_put(AGC_FLAG_ACMODFLG, true); /* P20LEMB1 */
    return rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE);
}

agc_status_t agc_job_manual_acquisition_limit_alarm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_prio_alarm(ctx, ACQ_ALARM_LIMITS,
                         AGC_JOB_MANUAL_ACQUISITION_TERMINATE,
                         AGC_JOB_MANUAL_ACQUISITION_LIMIT_ALARM,
                         AGC_JOB_MANUAL_ACQUISITION_MANEUVER);
}

/* R23LEM2: monitor back on; exit through V56.                           */
agc_status_t agc_job_manual_acquisition_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_NORRMON, false);
    return agc_rendz_goto_terminate_tracking();
}

/* R23LEM3: R61LEM maneuver, then R23LEM1.                               */
agc_status_t agc_job_manual_acquisition_maneuver(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return lmnav_tracking_attitude(ctx, true,
                                   AGC_JOB_MANUAL_ACQUISITION_DISPLAY);
}

/* ---- R24LEM search routine (pp.515-516) -------------------------------- */
/* R24LEM: search option on; LOSCMFLG off (so DODES does not go to R21);
 * DATAGOOD and OMEGA zero; R24LEM2.                                     */
agc_status_t agc_job_radar_search(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022); /* R21SRCH  */
    rz_flag_put(AGC_FLAG_SRCHOPTN, true);
    rz_flag_put(AGC_FLAG_LOSCMFLG, false);
    lmnav_state_t *n = lmnav_state(); /* R24LEM1  */
    n->datagood      = 0;
    n->omegad        = 0;
    return agc_job_radar_search_display(ctx);
}

/* R24LEM2: V16N80 (DATAGOOD, OMEGA) as a priority display; TERMINATE
 * exits P20, PROCEED accepts the lock-on (R24END), ENTER maneuvers
 * (R24LEM3). The job goes on with the search pattern (LRS24.1).       */
agc_status_t agc_job_radar_search_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    agc_dsky_cont_t cont;
    cont.terminate = AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE;
    cont.proceed   = AGC_JOB_RADAR_SEARCH_ACCEPT;
    cont.enter     = AGC_JOB_RADAR_SEARCH_MANEUVER;
    agc_status_t st =
        agc_dsky_display(ctx, AGC_DSKY_PRIODSPR, agc_dsky_vn(16U, 80U), &cont);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    lmnav_state()->nsrchpnt = 0; /* LRS24.1  */
    return agc_job_radar_search_point(ctx);
}

/* R24END: stop the pattern (CALLDGCH), CLRADMOD, P20LEM1.               */
agc_status_t agc_job_radar_search_accept(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    agc_status_t st = agc_waitlist_kill(AGC_TASK_RADAR_SEARCH_NEXT_POINT);
    agc_relint();
    if (st != AGC_STATUS_OK && st != AGC_STATUS_NOT_FOUND)
    {
        return st;
    }
    st = agc_rendz_clradmod();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_rendezvous_navigation_range_check(ctx);
}

/* R24LEM3: stop the pattern and the .5 s designate loop, wait for it to
 * die (.5 s), then R61 unless on the surface.                           */
agc_status_t agc_job_radar_search_maneuver(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    agc_inhint();
    agc_status_t st = agc_waitlist_kill(AGC_TASK_RADAR_SEARCH_NEXT_POINT);
    agc_relint();
    if (st != AGC_STATUS_OK && st != AGC_STATUS_NOT_FOUND)
    {
        return st;
    }
    st = agc_rendz_clradmod();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return rz_delayjob(ACQ_DELAY_HALF, AGC_JOB_RADAR_SEARCH_MANEUVER_DONE);
}

agc_status_t agc_job_radar_search_maneuver_done(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return agc_job_radar_search_resume(ctx);
    }
    return lmnav_tracking_attitude(ctx, true, AGC_JOB_RADAR_SEARCH_RESUME);
}

/* R24LEM4: RADCADR zero (set by ENDRADAR when the designate stopped) so
 * that RRDESSM returns to its caller; R24LEM2 again.                   */
agc_status_t agc_job_radar_search_resume(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_erase_set_modecadr(AGC_IMU_DEV_RADAR, 0);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_radar_search_display(ctx);
}

/* ---- LRS24.1 RR search routine (pp.588-593) ---------------------------
 * "Drives the rendezvous radar in a hexagonal search pattern about the
 * LOS to the CSM ... checking for the data good discrete": point 0 is
 * the LOS, points 1-6 lie 3.25 degrees off it, 60 degrees apart, a new
 * point every 6 s.                                                      */
/* CHKSRCH: tracker on; while the search is wanted, the next point in 6 s
 * (CALLDGCH) and, unless a remode runs, the designate to this point.  */
agc_status_t agc_job_radar_search_point(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rr_chan_or(AGC_CHAN_CHAN12, RR_CH12_TRACK);
    if (!agc_flag_test(AGC_FLAG_SRCHOPTN))
    {
        return AGC_STATUS_OK; /* ENDOFJOB */
    }
    agc_inhint();
    agc_status_t st =
        agc_waitlist_call(ACQ_6SECONDS, AGC_TASK_RADAR_SEARCH_NEXT_POINT);
    agc_relint();
    if (st != AGC_STATUS_OK || rr_radmodes_test(RR_REMODBIT))
    {
        return st;
    }
    return agc_job_radar_search_los(ctx);
}

/* LRS24.11: LM and CSM to now + 1.5 s (conic): the desired LOS (reference
 * axes), VXRCM = UNIT(UNIT(V(CSM)) x R(CSM)), and V(CSM) - V(LM) in SM
 * axes.                                                                  */
static agc_status_t acq_search_states(agc_interp_t *ip, lmnav_state_t *n)
{
    agc_integ_out_t o;
    agc_status_t    st =
        rz_integ(ip, RZ_LEMCONIC, agc_dp_add(ip, rz_loadtime(), ACQ_1_5SECS),
                 AGC_JOB_RADAR_SEARCH_LOS, &o);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_vec_t rlmsrch = o.ratt; /* RLMSRCH  */
    agc_vec_t vlm     = o.vatt;
    st = rz_integ(ip, RZ_CSMCONIC, o.tat, AGC_JOB_RADAR_SEARCH_LOS, &o);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    n->losdesrd =
        agc_vec_unit(ip, agc_vec_sub(ip, o.ratt, rlmsrch), NULL, NULL);
    agc_vec_t uv = agc_vec_unit(ip, o.vatt, NULL, NULL);
    n->vxrcm     = agc_vec_unit(ip, agc_vec_cross(ip, uv, o.ratt), NULL, NULL);
    agc_mat_t refsmmat = agc_erase_refsmmat();
    n->savlemv         = agc_vec_shl(
                ip, agc_mat_vec(ip, &refsmmat, agc_vec_sub(ip, o.vatt, vlm)), 1);
    return AGC_STATUS_OK;
}

/* The pattern point N in SM axes: N = 0 the LOS (DESGLOS); N = 1 the
 * axes UX = UNIT(VXRCM x LOS), UY = UNIT(LOS x UX) (CALCXY); later points
 * turn UX, UY by 60 degrees; the point is UNIT(LOS + tan 3.25 deg UY)
 * (OFFCALC).                                                            */
static agc_vec_t acq_search_point(agc_interp_t *ip, lmnav_state_t *n)
{
    agc_mat_t refsmmat = agc_erase_refsmmat();
    if (n->nsrchpnt == 0)
    {
        return agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, n->losdesrd), 1);
    }
    if (n->nsrchpnt == 1)
    {
        n->uxvect = agc_vec_unit(ip, agc_vec_cross(ip, n->vxrcm, n->losdesrd),
                                 NULL, NULL);
        n->uyvect = agc_vec_unit(ip, agc_vec_cross(ip, n->losdesrd, n->uxvect),
                                 NULL, NULL);
    }
    else
    {
        agc_vec_t uxpr = n->uxvect;
        agc_vec_t uypr = n->uyvect;
        n->uxvect =
            agc_vec_unit(ip,
                         agc_vec_add(ip, agc_vec_scale(ip, uxpr, ACQ_COS60),
                                     agc_vec_scale(ip, uypr, ACQ_SIN60)),
                         NULL, NULL);
        n->uyvect =
            agc_vec_unit(ip,
                         agc_vec_sub(ip, agc_vec_scale(ip, uypr, ACQ_COS60),
                                     agc_vec_scale(ip, uxpr, ACQ_SIN60)),
                         NULL, NULL);
    }
    agc_vec_t p = agc_vec_add(ip, agc_vec_scale(ip, n->uyvect, ACQ_OFFSTFAC),
                              n->losdesrd);
    p           = agc_vec_unit(ip, p, NULL, NULL);
    return agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, p), 1);
}

/* OMEGCALC: OMEGA, the angle between the RR boresight (RRNB of the RR
 * CDUs) and the +Z axis, for R2 of noun 80.                             */
static agc_status_t acq_omega(agc_interp_t *ip, lmnav_state_t *n)
{
    rr_state_t *s = rr_state();
    agc_inhint();
    s->tangnb[0] = agc_erase_cdut();
    s->tangnb[1] = agc_erase_cdus();
    agc_relint();
    agc_vec_t    b  = agc_vec_zero();
    agc_status_t st = agc_lm_rrnb(ip, s->tangnb[0], s->tangnb[1], &b);
    n->omegad       = agc_dp_acos(ip, b.c[2]);
    return st;
}

/* LRS24.11 .. CONTDES2 / OMEGCALC: continuous designate to the point;
 * out of coverage or a maneuver needed: alarm 527 and the pattern stops
 * (LIMALARM).                                                            */
agc_status_t agc_job_radar_search_los(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    lmnav_state_t *n = lmnav_state();
    rr_state_t    *s = rr_state();
    agc_interp_t   ip;
    agc_interp_init(&ip);
    agc_status_t st = acq_search_states(&ip, n);
    if (st != AGC_STATUS_OK)
    {
        return st; /* BUSY: asleep in INTSTALL  */
    }
    s->rrtarget = acq_search_point(&ip, n); /* CONTDESG */
    s->losvel   = n->savlemv;
    agc_inhint();
    st = agc_waitlist_kill(AGC_TASK_RR_DESIGNATE_LOOP);
    rr_radmodes_set(RR_CDESBIT); /* CONTDES2 */
    agc_relint();
    agc_rr_designate_result_t res = AGC_RR_DESIGNATE_END_JOB;
    st                            = agc_rr_designate_sm(&ip, &res);
    if (st != AGC_STATUS_OK || res == AGC_RR_DESIGNATE_END_JOB)
    {
        return st;
    }
    if (res == AGC_RR_DESIGNATE_STARTED)
    {
        return acq_omega(&ip, n); /* ENDOFJOB */
    }
    st = rz_alarm(ACQ_ALARM_SEARCH); /* LIMALARM */
    if (st != AGC_STATUS_OK && st != AGC_STATUS_ALARM)
    {
        return st;
    }
    agc_inhint();
    st = agc_waitlist_kill(AGC_TASK_RADAR_SEARCH_NEXT_POINT);
    agc_relint();
    return (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
}

/* CALLDGCH (task): while P20 runs, DATGDCHK as a PRIO25 job.            */
agc_status_t agc_task_radar_search_next_point(void)
{
    if (!agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        return AGC_STATUS_OK; /* TASKOVER */
    }
    return agc_exec_findvac(ACQ_PRIO25, AGC_JOB_RADAR_SEARCH_DATA_CHECK);
}

/* DATGDCHK: data good: 11111 in R1 and the designate stops; else the next
 * point (after the sixth, the pattern starts again).                    */
agc_status_t agc_job_radar_search_data_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    lmnav_state_t *n = lmnav_state();
    if ((agc_chan_read(AGC_CHAN_CHAN33) & RR_CH33_RR_DGOOD) == 0U)
    {
        n->datagood = ACQ_ALL1S; /* STORE1S  */
        agc_inhint();
        agc_status_t st = agc_waitlist_kill(AGC_TASK_RR_DESIGNATE_LOOP);
        agc_relint();
        return (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
    }
    n->nsrchpnt = (n->nsrchpnt == 6) ? 0 : (agc_sp_t) (n->nsrchpnt + 1);
    return agc_job_radar_search_point(ctx);
}
