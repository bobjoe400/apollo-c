/*
 * navigation.c -- P20 RENDEZVOUS NAVIGATION, P22 LUNAR
 * SURFACE NAVIGATION (PROG22 = PROG20) and P25 PREFERRED TRACKING
 * ATTITUDE, LM only (P20-P25.agc, Luminary099 pp.492-503, 507, 564-567).
 *
 * "The purpose of this program is to control the rendezvous radar from
 * startup through acquisition and lockon to the CSM and to update either
 * the LM or CSM state vector (as specified by the astronaut by DSKY
 * entry) on the basis of the RR tracking data."
 *
 * Here: the program flow PROG20 .. P20LEMD2, the surface option of P22
 * (ORBCHGO .. ORBCHG3: the CSM orbit assumed to change before launch),
 * UPPSV (permanent state vectors to now), LPS20.1 (LOS from the state
 * vectors) and LPS20.2 (400 NM check), P25, LUNSFCHK. R21/R23/R24 are in
 * lm/radar/acquisition.c, R22 in lm/navigation/tracking_data.c, R61/R65 in
 * lm/navigation/tracking_attitude.c.
 *
 * Scales: r m 2^29 (moon-centred states in the integration at 2^27), v
 * m/cs 2^7, t cs 2^28, LOS half unit, MLOSV m 2^20.
 */
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "../radar/radar.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "agc/io/service_internal.h"

#define NAV_FHNM           ((agc_dp_t) 189644800) /* FHNM 2DEC 740800 B-20: 400 NM */
#define NAV_ONEB_2         ((agc_dp_t) 67108864) /* ONEB-2 2DEC 1.0 B-2       */
#define NAV_250DEC         250U                  /* 250DEC: 2.5 s             */
#define NAV_1500DEC        1500U                 /* 1500DEC: 15 s             */
#define NAV_60SCNDS        6000U                 /* 60SCNDS DEC 6000          */
#define NAV_MODE_P20       20                    /* OCT24 = 20 decimal        */
#define NAV_MODE_P22       22
#define NAV_PRIO26         ((agc_prio_t) 026)
#define NAV_PRIO14         ((agc_prio_t) 014)
#define NAV_OPTION_CODE    012U  /* OCT00012: option 12 (N06) */
#define NAV_CHECKLIST_AUTO 0201U /* OCT201: select RR auto    */
#define NAV_ALARM_RANGE    ((agc_alarm_code_t) 00526) /* range > 400 NM      */
#define NAV_ALARM_AUTO     ((agc_alarm_code_t) 00514) /* RR out of auto       */
#define NAV_CH33_RR_AUTO   000002u /* chan 33 bit 2 (0 = auto)  */

static lmnav_state_t lmnav;

lmnav_state_t *lmnav_state(void)
{
    return &lmnav;
}

/* ---- LPS20.1 (pp.564-565) -----------------------------------------------
 * "1) Extrapolate the LEM and CSM vectors ... 2) computes the LOS vector
 * to the CSM, converts it to stable member coordinates and stores it in
 * RRTARGET. 3) computes the magnitude of the LOS vector and stores it in
 * MLOSV." In R21 on the lunar surface (LOSCMFLG and SURFFLAG) the LM is
 * not integrated (LMPOS from UPPSV). In P20 (RNDVZFLG) the LOS is shifted
 * up 9 places for precision; an overflow means more than 400 NM.      */
agc_status_t lmnav_los(agc_interp_t *ip, agc_dp_t tdec1, agc_job_id_t resume,
                       bool *too_far)
{
    if (!AGC_ASSERT(ip != NULL && too_far != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *too_far = false;
    agc_integ_out_t o;
    agc_status_t    st;
    if (!agc_flag_test(AGC_FLAG_LOSCMFLG) || !agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        st = rz_integ(ip, RZ_LEMCONIC, tdec1, resume, &o); /* LMINT  */
        if (st != AGC_STATUS_OK)
        {
            return st;
        }
        lmnav.lmpos = o.ratt;
        lmnav.lmvel = o.vatt;
        tdec1       = o.tat;
    }
    st = rz_integ(ip, RZ_CSMCONIC, tdec1, resume, &o); /* CSMINT */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_mat_t   refsmmat = agc_erase_refsmmat();
    rr_state_t *s        = rr_state();
    agc_vec_t   dv       = agc_vec_sub(ip, o.vatt, lmnav.lmvel);
    agc_inhint(); /* DODES uses these erasables */
    st = agc_waitlist_kill(AGC_TASK_RR_DESIGNATE_LOOP);
    agc_relint();
    s->losvel     = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, dv), 1);
    agc_vec_t los = agc_vec_sub(ip, o.ratt, lmnav.lmpos);
    if (agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        rz_clear_ovf(ip); /* BOVB     */
        los = agc_vec_shl(ip, los, 9);
    }
    agc_vec_t u = agc_vec_unit(ip, los, &s->mlosv, NULL); /* NOTSHIFT */
    *too_far    = agc_interp_test_ovf(ip);
    s->rrtarget = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, u), 1);
    rz_flag_put(AGC_FLAG_RRNBSW, false);
    return (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
}

/* LPS20.2 (p.566): the LOS is 400 NM or less.                           */
bool lmnav_range_ok(void)
{
    agc_dp_t m = rr_state()->mlosv;
    (void) AGC_ASSERT(m >= 0);
    return m < NAV_FHNM;
}

/* ---- UPPSV (pp.499-501): the permanent state vectors to TDEC1, the
 * updated vehicle with its W matrix (9x9, 6x6 on the surface) when it is
 * valid, the other vehicle to the same time without; on the surface
 * LMPOS / LMVEL from the LM conic state. INTEGRV releases INTSTALL.    */
static agc_status_t nav_uppsv_one(agc_interp_t *ip, bool csm, bool with_w,
                                  agc_dp_t tdec1, agc_job_id_t resume)
{
    agc_status_t st = agc_integ_install(0, resume);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_status_t sf = agc_integ_setiflgs();
    (void) AGC_ASSERT(sf == AGC_STATUS_OK);
    if (with_w)
    {
        rz_flag_put(AGC_FLAG_DIM0FLAG, true);
        rz_flag_put(AGC_FLAG_D6OR9FLG, !agc_flag_test(AGC_FLAG_SURFFLAG));
    }
    rz_flag_put(AGC_FLAG_VINTFLAG, csm);
    st = agc_integ_integrv(ip, agc_integ_shared_ctx(), tdec1);
    rz_phase(2, 04022); /* GRP2PC   */
    return st;
}

static agc_status_t nav_uppsv(agc_interp_t *ip, agc_dp_t tdec1,
                              agc_job_id_t resume)
{
    bool w          = agc_flag_test(AGC_FLAG_RENDWFLG);
    bool csm_first  = !w || agc_flag_test(AGC_FLAG_VEHUPFLG); /* UPPSV1  */
    agc_status_t st = nav_uppsv_one(ip, csm_first, w, tdec1, resume);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_dp_t t2 = csm_first ? agc_erase_tetcsm() : agc_erase_tetlem();
    st = nav_uppsv_one(ip, !csm_first, false, t2, resume); /* UPPSV4  */
    if (st != AGC_STATUS_OK || !agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return st;
    }
    agc_integ_sv_t sv;
    st          = agc_integ_perm_get(AGC_VEHICLE_LM, &sv);
    lmnav.lmpos = agc_vec_shr(sv.rcv, 2); /* VSR2     */
    lmnav.lmvel = agc_vec_shr(sv.vcv, 2);
    return st;
}

/* ---- PROG20 / PROG22 (pp.492-495) ------------------------------------- */
/* PROG20: group 4 off, restart at PRIORITY 26; on the surface (P22)
 * choose the CSM orbit option, else the LM state is to be updated.     */
agc_status_t agc_job_rendezvous_navigation_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase_job(04U, 05022U, AGC_JOB_RENDEZVOUS_NAVIGATION, 026);
    if (!agc_flag_test(AGC_FLAG_SURFFLAG))
    {                                          /* LUNSFCHK */
        rz_flag_put(AGC_FLAG_VEHUPFLG, false); /* LM state updated */
        return agc_job_rendezvous_navigation_start(ctx);
    }
    rz_flag_put(AGC_FLAG_VEHUPFLG, true);   /* ORBCHGO: the CSM */
    agc_status_t st = rz_set_option(2, 1U); /* will not change   */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_rendezvous_navigation_orbit_option(ctx);
}

/* GOPERF4 option 12 (assumed CSM orbit option in R2).                   */
agc_status_t agc_job_rendezvous_navigation_orbit_option(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dsky_cont_t cont;
    cont.terminate = AGC_JOB_GOTOPOOH;
    cont.proceed   = AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHOSEN;
    cont.enter     = AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_OPTION;
    return agc_dsky_display(ctx, AGC_DSKY_GOPERF4, NAV_OPTION_CODE, &cont);
}

/* ORBCHG1: option 1 (orbit unchanged): PROG20A; else V06N33 flashing for
 * the estimated time of launch (TIG).                                  */
agc_status_t agc_job_rendezvous_navigation_orbit_chosen(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (rz_option(2) == 1U)
    {
        return agc_job_rendezvous_navigation_start(ctx);
    }
    return agc_job_rendezvous_navigation_launch_time(ctx);
}

agc_status_t agc_job_rendezvous_navigation_launch_time(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 33U, AGC_JOB_GOTOPOOH,
                    AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHANGE,
                    AGC_JOB_RENDEZVOUS_NAVIGATION_LAUNCH_TIME);
}

/* ORBCHG3 first part: LM (no W) and CSM (W when valid) to the launch
 * time by precision integration; R, V at the moon scale.              */
static agc_status_t nav_orbit_states(agc_interp_t *ip, agc_vec_t *rsubl,
                                     agc_vec_t *rsubc, agc_vec_t *vsubc)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    agc_status_t     st =
        agc_integ_install(0, AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHANGE);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    lmnav.lnchtm = agc_erase_tig();
    rz_flag_put(AGC_FLAG_VINTFLAG, false); /* LM       */
    rz_flag_put(AGC_FLAG_INTYPFLG, false); /* Encke    */
    rz_flag_put(AGC_FLAG_DIM0FLAG, false);
    rz_flag_put(AGC_FLAG_D6OR9FLG, false);
    st = agc_integ_integrv(ip, ctx, lmnav.lnchtm); /* INTEGRV  */
    rz_phase(2, 04022);                            /* GRP2PC   */
    if (st == AGC_STATUS_OK)
    {
        *rsubl = ctx->out.ratt1; /* RSUBL    */
        st = agc_integ_install(0, AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHANGE);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_dp_t tat = ctx->out.tat;
    bool     w   = agc_flag_test(AGC_FLAG_RENDWFLG);
    rz_flag_put(AGC_FLAG_VINTFLAG, true); /* CSM      */
    rz_flag_put(AGC_FLAG_INTYPFLG, false);
    rz_flag_put(AGC_FLAG_DIM0FLAG, w);
    if (w)
    {
        rz_flag_put(AGC_FLAG_D6OR9FLG, true);
    }
    st = agc_integ_integrv(ip, ctx, tat); /* NOWMATX  */
    rz_phase(2, 04022);
    *vsubc = ctx->out.vatt1;
    *rsubc = ctx->out.ratt1;
    return st;
}

/* ORBCHG3 (pp.493-495): the CSM orbit plane is rotated to contain the LM
 * at launch. TIMETHET gives the CSM state through the angle between
 * R(CSM) and UCSM = UNIT(R(LM) x N1), N1 = UNIT(R(CSM) x V(CSM)); the
 * velocity is turned into the plane of R(LM) and the new position; the
 * CSM permanent state becomes that state at LNCHTM - TRANSTM.          */
static agc_status_t nav_orbit_change(agc_interp_t *ip)
{
    agc_vec_t    rsubl, rsubc, vsubc;
    agc_status_t st = nav_orbit_states(ip, &rsubl, &rsubc, &vsubc);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_vec_t n1 =
        agc_vec_unit(ip, agc_vec_cross(ip, rsubc, vsubc), NULL, NULL);
    agc_vec_t ucsm = agc_vec_unit(ip, agc_vec_cross(ip, rsubl, n1), NULL, NULL);
    agc_timetheta_in_t in;
    in.csth = agc_dp_shl(
        ip, agc_vec_dot(ip, agc_vec_unit(ip, rsubc, NULL, NULL), ucsm), 1);
    in.snth =
        agc_dp_sqrt(ip, agc_dp_sub(ip, NAV_ONEB_2, agc_dp_sq(ip, in.csth)));
    in.rvec = rsubc;
    in.vvec = agc_vec_neg(vsubc);
    in.rvsw = false;
    in.body = AGC_CONIC_MOON;
    rz_flag_put(AGC_FLAG_RVSW, false);
    agc_conic_time_out_t o;
    st = rz_timethet(ip, &in, &o);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_vec_t newvel = agc_vec_neg(o.vnew);
    agc_vec_t n2 =
        agc_vec_unit(ip, agc_vec_cross(ip, o.rnew, rsubl), NULL, NULL);
    agc_vec_t dir = agc_vec_unit(ip, agc_vec_cross(ip, n2, o.rnew), NULL, NULL);
    agc_vec_t ncsmvel =
        agc_vec_shl(ip, agc_vec_scale(ip, dir, agc_vec_abval(ip, newvel)), 1);
    rz_phase(2, 04022); /* GRP2PC   */
    st = agc_integ_install(0, AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHANGE);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    ctx->sv.tet          = agc_dp_sub(ip, lmnav.lnchtm, o.t); /* TET      */
    ctx->sv.rcv          = o.rnew;
    ctx->sv.rrect        = o.rnew;
    st                   = agc_integ_minirect(ctx, ncsmvel); /* VRECT    */
    if (st == AGC_STATUS_OK)
    {
        st = agc_integ_atopcsm(ip, ctx, 2); /* X2 = 2   */
    }
    agc_status_t sw = agc_integ_intwake0();
    return (st == AGC_STATUS_OK) ? sw : st;
}

/* ORBCHG2: the orbit change, then PROG20A. An INTSTALL sleep resumes this
 * job from its start.                                                   */
agc_status_t agc_job_rendezvous_navigation_orbit_change(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = nav_orbit_change(&ip);
    if (st != AGC_STATUS_OK)
    {
        return st; /* BUSY: asleep in INTSTALL  */
    }
    return agc_job_rendezvous_navigation_start(ctx);
}

/* PROG20A (pp.495-496): R02BOTH; UPDATFLG, TRACKFLG, RNDVZFLG on; search,
 * manual acquisition, R04FLAG (alarm 521 wanted), NORRMON, LOSCMFLG off;
 * CLRADMOD; P20LEM1.                                                   */
agc_status_t agc_job_rendezvous_navigation_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_imu_status_check(); /* R02BOTH  */
    if (st != AGC_STATUS_OK)
    {
        return st; /* alarm and P00 (POODOO)     */
    }
    rz_flag_put(AGC_FLAG_UPDATFLG, true);
    rz_flag_put(AGC_FLAG_TRACKFLG, true);
    rz_flag_put(AGC_FLAG_RNDVZFLG, true);
    rz_flag_put(AGC_FLAG_SRCHOPTN, false);
    rz_flag_put(AGC_FLAG_ACMODFLG, false);
    rz_flag_put(AGC_FLAG_ALARM_521_SUPPRESSED, false);
    rz_flag_put(AGC_FLAG_NORRMON, false);
    rz_flag_put(AGC_FLAG_LOSCMFLG, false);
    st = agc_rendz_clradmod();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_rendezvous_navigation_range_check(ctx);
}

/* P20LEM1 (p.496): mark counter zero; LOS now (LPS20.1); more than 400 NM
 * (LPS20.2, or the LPS20.1 overflow): priority alarm 526.              */
agc_status_t agc_job_rendezvous_navigation_range_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    agc_erase_set_trkmkcnt(0); /* MARKCTR  */
    agc_interp_t ip;
    agc_interp_init(&ip);
    bool         too_far = false;
    agc_status_t st =
        lmnav_los(&ip, rz_loadtime(), AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_CHECK,
                  &too_far);
    if (st != AGC_STATUS_OK)
    {
        return st; /* BUSY: asleep in INTSTALL  */
    }
    if (too_far || !lmnav_range_ok())
    {
        return agc_job_rendezvous_navigation_range_alarm(ctx);
    }
    return agc_job_rendezvous_navigation_attitude(ctx);
}

/* 526ALARM: TERMINATE exits P20 through V56, PROCEED is illegal (the
 * alarm again), ENTER recycles to P20LEM1.                             */
agc_status_t agc_job_rendezvous_navigation_range_alarm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_prio_alarm(ctx, NAV_ALARM_RANGE,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_ALARM,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_CHECK);
}

/* GOTOV56 as a job (the TERMINATE return of the P20 displays).         */
agc_status_t agc_job_rendezvous_navigation_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_rendz_goto_terminate_tracking();
}

/* P20LEMA (p.496): preferred tracking attitude (R61LEM) unless on the
 * surface, then P20LEMB.                                               */
agc_status_t agc_job_rendezvous_navigation_attitude(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return agc_job_rendezvous_navigation_acquire(ctx);
    }
    return lmnav_tracking_attitude(ctx, true,
                                   AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE);
}

/* ---- P20LEMB .. P20LEMD2 (pp.496-499) ---------------------------------- */
/* P20LEMB: restart at priority 10 (a V37-requested program runs first),
 * priority back to 26; wait for the track flag; with the RR in auto mode
 * go to acquisition; otherwise ask for auto mode in P20/P22 (GOPERF1
 * 201: ENTER = manual acquisition R23), elsewhere priority alarm 514.  */
agc_status_t agc_job_rendezvous_navigation_acquire(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase_job(0U, 05022U, AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE, 010);
    agc_status_t st = agc_exec_prio_change(NAV_PRIO26); /* PRIOCHNG */
    if (st == AGC_STATUS_BUSY)
    {
        return rz_continue(AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return lmnav_wait(NAV_250DEC); /* P20LEMWT */
    }
    if ((agc_chan_read(AGC_CHAN_CHAN33) & NAV_CH33_RR_AUTO) == 0U)
    {
        return agc_job_rendezvous_navigation_cdu_zero_wait(ctx); /* B3   */
    }
    agc_sp_t mm = agc_erase_modreg(); /* P20LEMB5 */
    if (mm == NAV_MODE_P20 || mm == NAV_MODE_P22)
    { /* P20LEMB6 */
        agc_dsky_cont_t cont;
        cont.terminate = AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE;
        cont.proceed   = AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE;
        cont.enter     = AGC_JOB_RENDEZVOUS_NAVIGATION_MANUAL_REQUEST;
        return agc_dsky_display(ctx, AGC_DSKY_GOPERF1, NAV_CHECKLIST_AUTO,
                                &cont);
    }
    return rz_prio_alarm(ctx, NAV_ALARM_AUTO,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE,
                         AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE);
}

/* ENTER on the auto-mode request: manual acquisition (R23LEM), not
 * allowed on the surface (P20LEMB2: operator error light).             */
agc_status_t agc_job_rendezvous_navigation_manual_request(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return agc_job_manual_acquisition(ctx);
    }
    agc_status_t st = agc_dsky_falton(); /* FALTON   */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_rendezvous_navigation_acquire(ctx);
}

/* P20LEMB3: wait while the RR CDUs are zeroed (2.5 s DELAYJOB); after a
 * search or manual acquisition go back to auto and to the data read;
 * else update the state vectors and designate.                         */
agc_status_t agc_job_rendezvous_navigation_cdu_zero_wait(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (rr_radmodes_test(RR_RCDU0BIT))
    { /* P20LEMB4 */
        return rz_delayjob(NAV_250DEC,
                           AGC_JOB_RENDEZVOUS_NAVIGATION_CDU_ZERO_WAIT);
    }
    if (agc_flag_test(AGC_FLAG_SRCHOPTN) || agc_flag_test(AGC_FLAG_ACMODFLG))
    {
        rz_flag_put(AGC_FLAG_SRCHOPTN, false);
        rz_flag_put(AGC_FLAG_ACMODFLG, false);
        return lmnav_wait(NAV_250DEC);
    }
    return agc_job_rendezvous_navigation_update_states(ctx);
}

/* P20LEMC3: permanent state vectors to now (UPPSV), then P20LEMC.       */
agc_status_t agc_job_rendezvous_navigation_update_states(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = nav_uppsv(&ip, rz_loadtime(),
                                AGC_JOB_RENDEZVOUS_NAVIGATION_UPDATE_STATES);
    if (st != AGC_STATUS_OK)
    {
        return st; /* BUSY: asleep in INTSTALL  */
    }
    return agc_job_rendezvous_navigation_designate(ctx);
}

/* P20LEMC: P20 over (rendezvous flag off): end; no track flag: wait 15 s
 * (P20LEMD); else designate (R21LEM).                                   */
agc_status_t agc_job_rendezvous_navigation_designate(agc_job_ctx_t *ctx)
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
    { /* P20LEMD  */
        return agc_waitlist_call(NAV_1500DEC,
                                 AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_WAIT);
    }
    return agc_job_rendezvous_sighting_mark(ctx); /* P20LEMF  */
}

/* P20LEMWT: in `delay_cs` look again (P20LEMC1); the job ends, with the
 * phase 2.7 restart point only while the track flag is on.             */
agc_status_t lmnav_wait(uint16_t delay_cs)
{
    (void) AGC_ASSERT(delay_cs > 0U);
    agc_status_t st =
        agc_waitlist_call(delay_cs, AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_CHECK);
    if (st == AGC_STATUS_OK && agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        rz_phase(2, 040072); /* P20LMWT1 */
    }
    return st; /* ENDOFJOB */
}

/* P20LEMC1 (task): P20 over: stop; track flag on: the R22 data read job;
 * else look again in 15 s.                                              */
agc_status_t agc_task_rendezvous_navigation_track_check(void)
{
    if (!agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        return AGC_STATUS_OK; /* TASKOVER */
    }
    if (agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return agc_exec_findvac(NAV_PRIO26,
                                AGC_JOB_RENDEZVOUS_TRACKING_DATA_NEXT);
    }
    return agc_waitlist_fixdelay(NAV_1500DEC,
                                 AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_CHECK);
}

/* P20LEMD1 (task): track flag on: P20LEMC3 (designate); else 15 s more. */
agc_status_t agc_task_rendezvous_navigation_track_wait(void)
{
    if (agc_flag_test(AGC_FLAG_TRACKFLG))
    { /* P20LEMD2 */
        return agc_exec_findvac(NAV_PRIO26,
                                AGC_JOB_RENDEZVOUS_NAVIGATION_UPDATE_STATES);
    }
    return agc_waitlist_fixdelay(NAV_1500DEC,
                                 AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_WAIT);
}

/* ---- P25 PREFERRED TRACKING ATTITUDE (pp.502-503) -----------------------
 * "To compute the preferred tracking attitude of the LM to continuously
 * point the LM tracking beacon at the CSM and to perform the maneuver to
 * the preferred tracking attitude and continuously maintain this
 * attitude within prescribed limits."                                  */
agc_status_t agc_job_preferred_tracking_attitude(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase_job(04U, 05022U, AGC_JOB_PREFERRED_TRACKING_ATTITUDE, 026);
    agc_status_t st = agc_imu_status_check(); /* R02BOTH  */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rz_flag_put(AGC_FLAG_TRACKFLG, true);
    rz_flag_put(AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE,
                true); /* P25FLAG */
    return agc_job_preferred_tracking_attitude_check(ctx);
}

/* P25LEM1: P25 over: end; no track flag: look again in a minute; else
 * R65 (fine preferred tracking attitude, 7 passes), then here again.  */
agc_status_t agc_job_preferred_tracking_attitude_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    if (!agc_flag_test(AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE))
    {
        return AGC_STATUS_OK; /* ENDOFJOB */
    }
    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    { /* P25LMWT1 */
        return agc_waitlist_call(NAV_60SCNDS,
                                 AGC_TASK_PREFERRED_TRACKING_ATTITUDE_WAIT);
    }
    lmnav.r65cntr = 7;
    return lmnav_tracking_attitude(ctx, false,
                                   AGC_JOB_PREFERRED_TRACKING_ATTITUDE_CHECK);
}

/* P25LEM2 (task): P25LEM1 as a PRIO14 job.                              */
agc_status_t agc_task_preferred_tracking_attitude_wait(void)
{
    (void) AGC_ASSERT(lmnav.r65cntr >= 0);
    return agc_exec_findvac(NAV_PRIO14,
                            AGC_JOB_PREFERRED_TRACKING_ATTITUDE_CHECK);
}
