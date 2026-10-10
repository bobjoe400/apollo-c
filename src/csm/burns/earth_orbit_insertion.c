/*
 * earth_orbit_insertion.c -- P11.agc (Comanche055, pp. 533-550): earth orbit insertion
 * monitor, and the Saturn takeover function (SATSTKON, SATSTICK).
 *
 * At liftoff (or V75 backup) P11 zeroes the CMC clock, moves TEPHEM to
 * the liftoff time, starts AVERAGEG (PREREAD1), computes the launch
 * REFSMMAT and RN, VN at liftoff (MATRXJOB), displays N62 (VI, HDOT, H)
 * each 2 s (VHHDOT as AVGEXIT) and drives the FDAI attitude-error needles
 * from the Saturn pitch/roll programme (ATERJOB, every ~.5 s).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "agc/core/assert.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/core/channels.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/nav/auxiliary.h"
#include "agc/csm/dap.h"
#include "agc/core/flags_csm.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/csm/entry_internal.h"
#include "agc/csm/entry_pending.h"

#define INSERTION_RPAD       ((agc_dp_t)3186669)   /* RPAD 2DEC 6373338 B-29, m  */
#define INSERTION_MERTHRAT   ((agc_dp_t)-51313698) /* -ERTHRAT 2DEC* -7.292115138
                                                E-7 B18*: -earth rate rad/cs */
#define INSERTION_HALFREV    ((agc_dp_t)AGC_DP_HALF) /* DPHALF                   */
#define INSERTION_POWDNCOD   3                     /* POWDNCOD = THREE           */
#define INSERTION_2SECS      200                   /* 2SECS, cs                  */
#define INSERTION_ATERTASK_DELAY    25                    /* OCT31: .25 s DELAYJOB      */
#define INSERTION_100MST5    16374                 /* 100MST5 DEC 16374: TIME5
                                                preset for a 100 ms T5RUPT */
#define INSERTION_STIKBITS   00077u                /* STIKBITS: RHC rotation bits */

/* P11 erasables (P11 STORAGE, P11 SATURN I/F; module-private).          */
static agc_earth_orbit_insertion_monitor_padload_t insertion_pad;
static agc_dp_t  lifttemp;          /* LIFTTEMP TIME2,1 at liftoff, cs    */
static bool      lifttemp_set;      /* LIFTTEMP != -0                     */
static bool      tliftoff_set;      /* TLIFTOFF holds the time (not the
                                       REP11SA phase words)               */
static agc_tp_t  tephem1;           /* TEPHEM1 new ephemeris time, TP     */
static agc_dp_t  ogc_lo[3];         /* OGC IGC MGC at liftoff, revs       */
static agc_dp_t  insertion_vmagi, insertion_hdot, insertion_alti; /* N62 display data     */
static agc_sp_t  satsw;             /* SATSW boost takeover status        */
static agc_sp_t  biasak[3];         /* BIASAK stored AK bias              */
static agc_sp_t  spolyarg;          /* SPOLYARG TIME1 - RPSTART, cs       */

agc_status_t agc_earth_orbit_insertion_monitor_set_padload(const agc_earth_orbit_insertion_monitor_padload_t *pad)
{
    if (!AGC_ASSERT(pad != NULL) ||
        !AGC_ASSERT(pad->polynum >= 1 && pad->polynum <= AGC_EARTH_ORBIT_INSERTION_MONITOR_POLY_MAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    insertion_pad = *pad;
    return AGC_STATUS_OK;
}

/* ======================================================================
 * Liftoff (pp. 535-537)
 * ==================================================================== */

/* P11 / P11+7 (p. 535): restarts to REP11 (3.11), group 5 off; TLIFTOFF
 * = clock at liftoff, the clock zeroed, restarts to REP11A (3.13).      */
agc_status_t agc_job_earth_orbit_insertion_monitor(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_phase_change(3, 000113);            /* -PHASE3 = REP11S   */
    lifttemp_set = false;                         /* CS ZERO TS LIFTTEMP */
    (void)agc_phase_kill(5);                      /* prelaunch group 5   */
    tliftoff_set = false;                         /* REP11SA in TLIFTOFF */
    lifttemp = (agc_dp_t)agc_time_now_cs();       /* DCA TIME2           */
    lifttemp_set = true;
    (void)agc_time_set_cs(0u);                    /* zero CMC clock      */
    agc_erase_set_tevent(lifttemp);               /* TLIFTOFF            */
    tliftoff_set = true;
    (void)agc_phase_change(3, 000133);            /* REP11A-1: 3.13      */
    (void)AGC_ASSERT(agc_time_now_cs() < 100u);
    return agc_job_earth_orbit_insertion_monitor_ephemeris(ctx);
}

/* REP11A (pp. 535-536): TEPHEM1 = TEPHEM + TLIFTOFF, restart there.     */
agc_status_t agc_job_earth_orbit_insertion_monitor_ephemeris(agc_job_ctx_t *ctx)
{
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_PREREAD, AGC_TASK_NONE,
                          (int16_t)022, false, 0U };

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(tliftoff_set)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    tephem1 = agc_erase_tephem() + (agc_tp_t)agc_erase_tevent(); /* DAS/ADS */
    agc_relint();
    (void)agc_phase_change_var(005023, &v);            /* PRIO22         */
    return agc_job_earth_orbit_insertion_monitor_preread(ctx);
}

/* After the 05023 PHASCHNG (p. 536): TEPHEM = TEPHEM1, PREREAD1.        */
agc_status_t agc_job_earth_orbit_insertion_monitor_preread(agc_job_ctx_t *ctx)
{
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_SETUP, AGC_TASK_NONE,
                          (int16_t)022, false, 0U };
    agc_status_t    st;

    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    agc_erase_set_tephem(tephem1);
    agc_relint();
    st = agc_guid_preread1_csm();                 /* zero PIPs, AVERAGEG */
    (void)agc_phase_change_var(005023, &v);
    (void)AGC_ASSERT(agc_erase_tephem() == tephem1);
    return (st == AGC_STATUS_OK) ? agc_job_earth_orbit_insertion_monitor_setup(ctx) : st;
}

/* (p. 536-537): ATERTASK in .5 s, MM 11, NODOP01, powered-flight
 * downlist, CLEANDSP, restarts 4.51 / 3.7, then EARTHR (exit P11OUT ->
 * MATRXJOB in this job).                                                 */
agc_status_t agc_job_earth_orbit_insertion_monitor_setup(agc_job_ctx_t *ctx)
{
    agc_status_t st;

    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_waitlist_call(50, AGC_TASK_ATERTASK);          /* .5SEC     */
    (void)agc_phase_newmode(11);
    (void)agc_flag_set(AGC_FLAG_NO_PRELAUNCH_INITIALIZATION);
    (void)agc_telem_select_list(INSERTION_POWDNCOD);
    (void)agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0, NULL); /* V75 case  */
    (void)agc_phase_change2(040514, 000073, NULL);          /* 4.51, 3.7 */
    st = agc_csm_prelaunch_earthr_at_liftoff(entry_ip(), agc_erase_tevent());
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_NO_PRELAUNCH_INITIALIZATION));
    return (st == AGC_STATUS_OK) ? agc_job_matrxjob(ctx) : st;
}

/* REP11 (p. 539): restart of group 3 at 3.11.                           */
agc_status_t agc_job_earth_orbit_insertion_monitor_restart(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_phase_get(5) != 0u) {                 /* CCS PHASE5          */
        return AGC_STATUS_OK;                     /* ENDOFJOB            */
    }
    if (!lifttemp_set) {
        return agc_job_earth_orbit_insertion_monitor(ctx);                  /* P11+7 (clock first) */
    }
    if (!tliftoff_set) {
        if (agc_time_now_cs() >= 16384u) {        /* CCS TIME2: not yet  */
            (void)agc_time_set_cs(0u);            /* zeroed: REP11A -5   */
        }
        agc_erase_set_tevent(lifttemp);           /* REP11A -2           */
        tliftoff_set = true;
    }
    (void)agc_phase_change(3, 000133);            /* ENDREP11: 3.13      */
    (void)AGC_ASSERT(tliftoff_set);
    return agc_job_earth_orbit_insertion_monitor_ephemeris(ctx);
}

/* ======================================================================
 * MATRXJOB (pp. 537-539)
 * ==================================================================== */

/* REFSMMAT at liftoff (p. 538): Z = UNIT(down), X = UNIT(SIN(AZ) EAST
 * - COS(AZ) SOUTH), Y = Z x X; all half-unit.                            */
static agc_mat_t launch_refsmmat(agc_interp_t *ip, agc_vec_t down)
{
    agc_vec_t unitw = agc_guid_state()->avg.unitw;
    agc_mat_t m;
    agc_vec_t east;
    agc_vec_t msouth;

    (void)AGC_ASSERT(ip != NULL);
    m.row[2] = agc_vec_unit(ip, down, NULL, NULL);
    east = agc_vec_unit(ip, agc_vec_cross(ip, m.row[2], unitw), NULL, NULL);
    msouth = agc_vec_unit(ip, agc_vec_cross(ip, east, m.row[2]), NULL, NULL);
    m.row[0] = agc_vec_add(ip, agc_vec_scale(ip, msouth, agc_dp_cos(ip, insertion_pad.launchaz)),
                           agc_vec_scale(ip, east, agc_dp_sin(ip, insertion_pad.launchaz)));
    m.row[0] = agc_vec_unit(ip, m.row[0], NULL, NULL);
    m.row[1] = agc_vec_neg(agc_vec_unit(ip, agc_vec_cross(ip, m.row[0], m.row[2]),
                                        NULL, NULL));
    (void)AGC_ASSERT(m.row[2].c[0] >= AGC_DP_MIN);
    return m;
}

/* GETDOWN (p. 540): LALOTORV +5 with time 0 and GAMMA = 1: the geodetic
 * vertical at the pad, negated (VCOMP).                                  */
static agc_status_t getdown(agc_interp_t *ip, agc_vec_t *down)
{
    agc_navaux_padload_t pad;
    agc_dp_t             cl = agc_dp_cos(ip, insertion_pad.latitude);
    agc_vec_t            rp;
    agc_status_t         st = agc_navaux_get_padload(&pad);

    (void)AGC_ASSERT(down != NULL);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rp = agc_vec_make(agc_dp_mul(ip, agc_dp_cos(ip, insertion_pad.padlong), cl),
                      agc_dp_mul(ip, agc_dp_sin(ip, insertion_pad.padlong), cl),
                      agc_dp_shr(agc_dp_sin(ip, insertion_pad.latitude), 1));
    rp = agc_vec_unit(ip, rp, NULL, NULL);
    st = agc_planet_rp_to_r(ip, &pad, false, rp, 0, down);
    *down = agc_vec_neg(*down);
    return st;
}

/* MATRXJOB tail (pp. 538-539): restart 3.x here, AVGEXIT = VHHDOT,
 * 1/PIPADT = 2 s, READACCS 2 s after liftoff, NORMLIZE.                  */
static agc_status_t matrx_tail(agc_job_ctx_t *ctx)
{
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_MATRXJOB, AGC_TASK_NONE, 0,
                          false, 0U };
    int32_t         dt;

    (void)AGC_ASSERT(ctx != NULL);
    (void)agc_phase_change_var(004023, &v);
    agc_guid_state()->avg.avgexit = AGC_JOB_VHHDOT;     /* P11SCADR      */
    agc_imu_set_pipa_interval((agc_sp_t)031000);             /* PRIO31: 2 s   */
    agc_inhint();
    /* CS ZERO / TS TBASE5: READACCS restarts relative to liftoff (no exec
     * API for TBASE5 alone; the 5.2 restart below uses the present TIME). */
    dt = INSERTION_2SECS - (int32_t)agc_erase_time1();
    if (dt <= 0) {
        dt = 1;                                         /* never negative */
    }
    (void)agc_waitlist_call((uint16_t)dt, AGC_TASK_READACCS);
    agc_relint();
    (void)agc_phase_change2(000003, 000025, NULL);      /* 3 off, 5.2    */
    (void)AGC_ASSERT(dt > 0);
    return agc_job_normlize_csm(ctx);                   /* POSTJUMP      */
}

/* MATRXJOB (pp. 537-539): liftoff CDUs for the attitude-error display,
 * RN1 (LALOTORV of the pad at T = 0), VN1 = WE x RN1, REFSMMAT, SATRLRT
 * sign, REFSMFLG; AVGEXIT = VHHDOT; READACCS 2 s after liftoff; NORMLIZE. */
agc_status_t agc_job_matrxjob(agc_job_ctx_t *ctx)
{
    const agc_latlong_opt_t opt = { false, false };
    agc_interp_t           *ip = entry_ip();
    agc_avg_g_t            *avg = &agc_guid_state()->avg;
    agc_navaux_padload_t    pad;
    agc_latlong_t           ll = { insertion_pad.latitude, insertion_pad.padlong, insertion_pad.pgncsalt, 0 };
    agc_vec_t               down;
    agc_mat_t               m;
    agc_status_t            st;

    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    ogc_lo[0] = agc_guid_cdulogic(agc_erase_cdux());     /* OGC B-1: revs */
    ogc_lo[1] = agc_guid_cdulogic(agc_erase_cduy());
    ogc_lo[2] = agc_guid_cdulogic(agc_erase_cduz());
    (void)agc_flag_clear(AGC_FLAG_ERADFLAG);
    st = agc_navaux_get_padload(&pad);
    if (st == AGC_STATUS_OK) {
        st = agc_latlong_to_r(ip, &pad, opt, &ll, 0, &avg->rn1, NULL);
    }
    if (st == AGC_STATUS_OK) {
        st = getdown(ip, &down);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    m = launch_refsmmat(ip, down);
    avg->vn1 = agc_vec_shl(ip, agc_vec_scale(ip, agc_vec_cross(ip, avg->rn1, avg->unitw),
                                             INSERTION_MERTHRAT), 4);   /* B7 */
    agc_erase_set_refsmmat(m);
    insertion_pad.satrlrt = agc_dp_sign(insertion_pad.satrlrt, agc_dp_add(ip, agc_dp_sub(ip,
                                  INSERTION_HALFREV, insertion_pad.launchaz), insertion_pad.azimuth));
    (void)agc_flag_set(AGC_FLAG_REFSMFLG);
    return matrx_tail(ctx);
}

/* ======================================================================
 * N62 display (pp. 540, 546)
 * ==================================================================== */

/* S11.1 (p. 546): VMAGI, HDOT, ALTI for N62.                            */
agc_status_t agc_csm_velocity_altitude_rate(agc_interp_t *ip)
{
    agc_vec_t rn = agc_erase_rn();
    agc_vec_t vn = agc_erase_vn();
    agc_vec_t ur;
    agc_dp_t  rmag = 0;
    agc_dp_t  base = INSERTION_RPAD;

    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    insertion_vmagi = agc_vec_abval(ip, vn);                   /* 2^7 m/cs    */
    ur = agc_vec_unit(ip, rn, &rmag, NULL);
    insertion_hdot = agc_dp_shl(ip, agc_vec_dot(ip, ur, vn), 1);
    if (agc_flag_test(AGC_FLAG_AMOONFLG)) {
        base = agc_dp_shr(agc_vec_abval(ip, agc_erase_rls()), 2); /* B27->B29 */
    }
    insertion_alti = agc_dp_sub(ip, rmag, base);               /* BDSU 36D     */
    (void)AGC_ASSERT(rmag >= 0);
    return AGC_STATUS_OK;
}

agc_status_t agc_earth_orbit_insertion_monitor_readout(agc_dp_t *vmagi, agc_dp_t *hdot, agc_dp_t *alti)
{
    if (!AGC_ASSERT(vmagi != NULL && hdot != NULL) || !AGC_ASSERT(alti != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    *vmagi = insertion_vmagi;
    *hdot = insertion_hdot;
    *alti = insertion_alti;
    return AGC_STATUS_OK;
}

/* VHHDOT (p. 540): AVGEXIT each 2 s: S11.1, restart 5.3, REGODSP V06N62
 * (which ends the job).                                                  */
agc_status_t agc_job_vhhdot(agc_job_ctx_t *ctx)
{
    agc_status_t st;

    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    st = agc_csm_velocity_altitude_rate(entry_ip());
    (void)agc_phase_change(5, 000035);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return agc_dsky_display(ctx, AGC_DSKY_REGODSP, agc_dsky_vn(6, 62), NULL);
}

/* ======================================================================
 * Attitude error display (pp. 540-546)
 * ==================================================================== */

/* ATERTASK (p. 540): start ATERJOB, initialise NEEDLER, SATSW = 1.     */
agc_status_t agc_task_atertask(void)
{
    agc_status_t st = agc_exec_findvac(AGC_ENTRY_PRIO1, AGC_JOB_ATERJOB);

    (void)agc_dap_needler(0, 0, 0, true);                /* RCSFLAGS BIT3 */
    satsw = 1;                                           /* display      */
    (void)AGC_ASSERT(satsw == 1);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st != AGC_STATUS_BAD_ARG);
    return st;
}

/* AK = TGSC (DESIRED - CDU) (pp. 544-545), TGSC rows
 * (1, SIN Z, 0), (0, COS X COS Z, SIN X), (0, -SIN X COS Z, COS X), with
 * the errors ordered DTHETA (Y), DPSI (Z), DPHI (X); AK scaled 2 revs.   */
agc_status_t agc_earth_orbit_insertion_monitor_attitude_errors(const agc_dp_t desired[3],
                                     const uint16_t cdu[3], agc_sp_t ak_out[3])
{
    agc_interp_t *ip = entry_ip();
    agc_dp_t      d[3];       /* DTHETA (Y), DPSI (Z), DPHI (X): SP /180 hi */
    agc_dp_t      sx, cx, sz, cz;
    agc_dp_t      p;
    int           i;

    if (!AGC_ASSERT(desired != NULL && cdu != NULL) || !AGC_ASSERT(ak_out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    for (i = 0; i < 3; i++) {                     /* order Y Z X        */
        int k = (i + 1) % 3;                      /* desired/cdu: X Y Z */
        int32_t w = (int32_t)agc_rtb_1sto2s(desired[k]) - (int32_t)(cdu[k] & 077777u);
        d[i] = agc_dp_from_sp(entry_angle_wrap(w));       /* DELSTOR (MSU)  */
    }
    sx = agc_dp_sin(ip, agc_guid_cdulogic(cdu[0]));
    cx = agc_dp_cos(ip, agc_guid_cdulogic(cdu[0]));
    sz = agc_dp_sin(ip, agc_guid_cdulogic(cdu[2]));
    cz = agc_dp_cos(ip, agc_guid_cdulogic(cdu[2]));
    ak_out[0] = agc_dp_high(agc_dp_shr_round(agc_dp_add(ip,
                    agc_dp_shl(ip, agc_dp_mul(ip, sz, d[0]), 1), d[2]), 2));
    p = agc_dp_mul(ip, cz, d[0]);
    ak_out[1] = agc_dp_high(agc_dp_shl(ip, agc_dp_add(ip, agc_dp_mul(ip, sx, d[1]),
                    agc_dp_shl(ip, agc_dp_mul(ip, p, cx), 1)), 1));
    ak_out[2] = agc_dp_high(agc_dp_shl(ip, agc_dp_sub(ip, agc_dp_mul(ip, cx, d[1]),
                    agc_dp_shl(ip, agc_dp_mul(ip, p, sx), 1)), 1));
    (void)AGC_ASSERT(ak_out[0] >= AGC_SP_MIN);
    return AGC_STATUS_OK;
}

/* ATTDISP / ATTDISP1 / ATTDISPR (pp. 542-544): desired gimbal angles from
 * the pitch polynomial and the roll programme (CALCGTA), or the liftoff
 * attitude (NOPOLY).                                                     */
static void desired_attitude(agc_interp_t *ip, agc_dp_t desired[3])
{
    int32_t   t = (int32_t)agc_erase_time1() - insertion_pad.rpstart;
    agc_dp_t  pitch;
    agc_dp_t  roll;
    agc_mat_t dc;

    desired[0] = ogc_lo[0];                                /* NOPOLY    */
    desired[1] = ogc_lo[1];
    desired[2] = ogc_lo[2];
    if (t <= 0 || t + insertion_pad.polystop > 0 || agc_erase_time2() > 0) {
        return;
    }
    spolyarg = (agc_sp_t)t;
    pitch = agc_dp_shl(ip, agc_dp_poly(agc_dp_from_sp(spolyarg), insertion_pad.poly,
                                       insertion_pad.polynum), 5);      /* revs  */
    roll = agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_from_sp(spolyarg), insertion_pad.satrlrt), 14);
    roll = agc_dp_sub(ip, agc_dp_add(ip, agc_dp_sub(ip, roll, INSERTION_HALFREV),
                                     insertion_pad.launchaz), insertion_pad.azimuth);
    if (agc_dp_sgn(roll) * agc_dp_sgn(insertion_pad.satrlrt) > 0) {      /* RLTST */
        roll = 0;                           /* rollout done: MBDYTCTL +2    */
    }
    dc.row[0] = agc_vec_make(agc_dp_sin(ip, pitch), 0, agc_dp_neg(agc_dp_cos(ip, pitch)));
    dc.row[1] = agc_vec_make(agc_dp_neg(agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_sin(ip, roll),
                                 agc_dp_cos(ip, pitch)), 1)), agc_dp_neg(agc_dp_cos(ip, roll)),
                             agc_dp_neg(agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_sin(ip, roll),
                                 agc_dp_sin(ip, pitch)), 1)));
    dc.row[2] = agc_vec_make(agc_dp_neg(agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_cos(ip, roll),
                                 agc_dp_cos(ip, pitch)), 1)), agc_dp_sin(ip, roll),
                             agc_dp_neg(agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_cos(ip, roll),
                                 agc_dp_sin(ip, pitch)), 1)));
    for (t = 0; t < 3; t++) {
        dc.row[t] = agc_vec_unit(ip, dc.row[t], NULL, NULL);
    }
    (void)agc_imu_calcgta(ip, &dc, desired);               /* OGC IGC MGC */
}

/* ATERJOB (pp. 542-546): one display cycle, then DELAYJOB .25 s.         */
agc_status_t agc_job_aterjob(agc_job_ctx_t *ctx)
{
    agc_dp_t desired[3];
    uint16_t cdu[3];
    agc_sp_t ak[3];
    int      i;

    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_flag_test(AGC_FLAG_DAPBIT1) || agc_flag_test(AGC_FLAG_DAPBIT2)) {
        return AGC_STATUS_OK;                     /* Saturn stick: end   */
    }
    if ((agc_chan_read(030) & AGC_CH030_SCCTLSATURN) == 0u) {   /* STRSAT */
        if (satsw > 0) {
            return agc_job_atreset_after_delay(ctx) == AGC_STATUS_OK ? AGC_STATUS_OK
                                                          : AGC_STATUS_BAD_ARG;
        }
    } else if (satsw <= 0) {
        (void)agc_dap_needler(0, 0, 0, true);     /* ATRESET             */
        return agc_svc_delayjob(6, AGC_JOB_ATRESET_AFTER_DELAY);
    }
    desired_attitude(entry_ip(), desired);
    cdu[0] = agc_erase_cdux();
    cdu[1] = agc_erase_cduy();
    cdu[2] = agc_erase_cduz();
    (void)agc_earth_orbit_insertion_monitor_attitude_errors(desired, cdu, ak);
    if (satsw <= 0) {
        for (i = 0; i < 3; i++) {
            if (satsw == 0) {
                biasak[i] = (agc_sp_t)(-ak[i]);   /* AKLOAD              */
            }
            ak[i] = entry_sp_sat(4 * (int32_t)entry_sp_mp(entry_sp_sat(
                        (int32_t)ak[i] + biasak[i]), insertion_pad.satscale)); /* RESCALES */
        }
        satsw = -1;                               /* steer               */
    }
    (void)agc_dap_needler(ak[0], ak[1], ak[2], false);      /* SATOUT   */
    return agc_svc_delayjob(INSERTION_ATERTASK_DELAY, AGC_JOB_ATERJOB);  /* ATERSET  */
}

/* ATRESET +5 (p. 546): after the 60 ms wait, SATSW > 0: enable S-IVB
 * takeover (TAKEON) and SATSW = 0; else SATSW = 1 and takeover off.     */
agc_status_t agc_job_atreset_after_delay(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (satsw > 0) {
        (void)agc_chan_or(012, AGC_CH012_SIVBTAKEOVER);     /* TAKEON   */
        satsw = 0;
    } else {
        satsw = 1;
        (void)agc_chan_and(012, (uint16_t)~AGC_CH012_SIVBTAKEOVER);
    }
    (void)AGC_ASSERT(satsw == 0 || satsw == 1);
    (void)agc_dap_needler(0, 0, 0, false);                  /* SATOUT   */
    return agc_svc_delayjob(INSERTION_ATERTASK_DELAY, AGC_JOB_ATERJOB);
}

/* ======================================================================
 * Saturn takeover (pp. 547-550)
 * ==================================================================== */

/* SATSTKON (p. 549): V46 with DAPDATR1 bits 13,14: T5 to REDOSAT now,
 * FLAGWRD6 bits 15,14 (DAPBIT1, DAPBIT2), ZEROJET, exit via GOPIN.       */
agc_status_t agc_csm_satstkon(const agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    (void)agc_rupt_set_t5_task(AGC_TASK_REDOSAT);
    (void)agc_rupt_set_time(5, 037777);                      /* POSMAX   */
    (void)agc_flag_set(AGC_FLAG_DAPBIT1);
    (void)agc_flag_set(AGC_FLAG_DAPBIT2);
    (void)agc_csm_dap_zerojet();
    agc_relint();
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_DAPBIT1));
    return agc_dsky_gopin(ctx);
}

/* REDOSAT (p. 549): NEEDLER init, S-IVB takeover on, SATSTICK in 100 ms. */
agc_status_t agc_task_redosat(void)
{
    (void)agc_dap_needler(0, 0, 0, true);
    (void)agc_chan_or(012, AGC_CH012_SIVBTAKEOVER);
    (void)agc_rupt_set_t5_task(AGC_TASK_SATSTICK);
    (void)AGC_ASSERT(agc_rupt_t5_task() == AGC_TASK_SATSTICK);
    (void)AGC_ASSERT((agc_chan_read(012) & AGC_CH012_SIVBTAKEOVER) != 0u);
    return agc_rupt_set_time(5, INSERTION_100MST5);                /* 100MST5  */
}

/* SATSTICK (p. 550): every 100 ms (T5): the RHC rotation bits of channel
 * 31 (inverted logic: STIKBITS RXOR CHAN31, MASK STIKBITS) set RMANNDX,
 * PMANNDX, YMANNDX (STICKCHK, dap-csm); the indexed SATRATE pad loads
 * go to AK (roll), AK1 (pitch), AK2 (yaw) and out through NEEDLER.        */
agc_status_t agc_task_satstick(void)
{
    const agc_csm_rcs_t *r;
    uint16_t             bits;
    agc_status_t         st;

    (void)agc_rupt_set_t5_task(AGC_TASK_SATSTICK);         /* 2SATSTCK */
    (void)agc_rupt_set_time(5, INSERTION_100MST5);               /* 100MST5  */
    bits = (uint16_t)((INSERTION_STIKBITS ^ agc_chan_read(031)) & INSERTION_STIKBITS);
    st = agc_csm_dap_stickchk(bits);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    r = agc_csm_dap_rcs();
    if (!AGC_ASSERT(r->manndx[0] >= 0 && r->manndx[0] <= 3) ||
        !AGC_ASSERT(r->manndx[1] >= 0 && r->manndx[1] <= 3) ||
        !AGC_ASSERT(r->manndx[2] >= 0 && r->manndx[2] <= 3)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_dap_needler(insertion_pad.satrate[r->manndx[0]],  /* AK  roll  */
                           insertion_pad.satrate[r->manndx[1]],  /* AK1 pitch */
                           insertion_pad.satrate[r->manndx[2]],  /* AK2 yaw   */
                           false);
}
