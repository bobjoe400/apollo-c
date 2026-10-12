/**********************************************************************
 * FILE NAME: earth_orbit_insertion.c
 *
 * PURPOSE:
 *   P11, the earth orbit insertion monitor, and the Saturn takeover
 *   function (SATSTKON, REDOSAT, SATSTICK). At liftoff (or V75 backup)
 *   P11 zeroes the CMC clock, moves TEPHEM to the liftoff time, starts
 *   AVERAGEG (PREREAD1), computes the launch REFSMMAT and RN, VN at
 *   liftoff (MATRXJOB), displays N62 (VI, HDOT, H) each 2 s (VHHDOT as
 *   AVGEXIT) and drives the FDAI attitude-error needles from the Saturn
 *   pitch/roll programme (ATERJOB, every ~.5 s).
 *
 * AGC SOURCE:
 *   Comanche055 P11.agc pp. 533-550 (P11 .. REP11, MATRXJOB, GETDOWN,
 *   VHHDOT, ATERTASK, ATERJOB .. ATTDISP2, STEERSAT .. TAKEON, S11.1,
 *   DELSTOR, RLTST, NOPOLY, RESCALES, SATSTKON, REDOSAT, SATSTICK)
 *   Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   TIME2/TIME1   clock      IO   cs             zeroed at liftoff
 *   TEPHEM        agc_tp_t   IO   cs B42         ephemeris time
 *   TLIFTOFF      agc_dp_t   O    cs B28         clock at liftoff
 *   REFSMMAT      agc_mat_t  O    half-unit      launch REFSMMAT
 *   RN1, VN1      agc_vec_t  O    m B29, m/cs B7 state at liftoff
 *   CDUX..CDUZ    uint16_t   I    CDU pattern    gimbal angles
 *   channels 12, 30, 31     IO   -              S-IVB takeover, RHC
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_guid_preread1_csm PREREAD1: zero PIPs, start AVERAGEG
 *   agc_latlong_to_r      LALOTORV (pad position at T = 0)
 *   agc_imu_calcgta       CALCGTA: desired gimbal angles
 *   agc_dap_needler       NEEDLER: FDAI attitude-error needles
 *   agc_csm_dap_stickchk  STICKCHK: RHC rate indices
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none raised here; display and executive statuses are passed up.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only. Restart protected through groups 3, 4 and 5 as in the AGC
 *   (REP11, REP11A, 4.51, 5.2/5.3).
 *
 * NOTES:
 *   The pad loads arrive through agc_earth_orbit_insertion_monitor_set_
 *   padload; POLYNUM there is the polynomial degree (the AGC POLYNUM is
 *   one less, for POWRSERS). DMP SL / DOT SL1 / POWRSERS keep the TP MPAC
 *   (review-guid-entry); TAD BIASAK and RESCALES model the word-level
 *   carries and the TS overflow skip.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"

#define INSERTION_RPAD     ((agc_dp_t) 3186669)     /* RPAD 6373338 m, B29 */
#define INSERTION_MERTHRAT ((agc_dp_t) -51313698)   /* -ERTHRAT rad/cs B18 */
#define INSERTION_HALFREV  ((agc_dp_t) AGC_DP_HALF) /* DPHALF, B0          */
#define INSERTION_POWDNCOD 3                        /* POWDNCOD = THREE    */
#define INSERTION_2SECS    200                      /* 2SECS, cs           */
#define INSERTION_100MST5  16374                    /* TIME5 for 100 ms    */

#define INSERTION_ATERTASK_DELAY 25     /* OCT31: .25 s DELAYJOB              */
#define INSERTION_STIKBITS       00077u /* STIKBITS: RHC rotation bits        */

/* P11 erasables (P11 STORAGE, P11 SATURN I/F; module-private).          */
static agc_earth_orbit_insertion_monitor_padload_t insertion_pad;   /* pad loads, scales per member      */
static agc_dp_t                                    lifttemp;        /* LIFTTEMP TIME2,1 at liftoff, cs B28 */
static bool                                        lifttemp_set;    /* LIFTTEMP != -0                    */
static bool                                        tliftoff_set;    /* TLIFTOFF holds the time           */
static agc_tp_t                                    tephem1;         /* TEPHEM1 ephemeris time, cs B42    */
static agc_dp_t                                    ogc_lo[3];       /* OGC IGC MGC at liftoff, revs B0   */
static agc_dp_t                                    insertion_vmagi; /* N62 VI, m/cs B7                   */
static agc_dp_t                                    insertion_hdot;  /* N62 HDOT, m/cs B7                 */
static agc_dp_t                                    insertion_alti;  /* N62 H, m B29                      */
static agc_sp_t                                    satsw;           /* SATSW takeover status, unscaled   */
static agc_sp_t                                    biasak[3];       /* BIASAK stored AK bias, B0         */
static agc_sp_t                                    insertion_ak[3]; /* AK AK1 AK2 needle commands, B0    */
static agc_sp_t                                    spolyarg;        /* SPOLYARG TIME1 - RPSTART, cs B14  */

/**********************************************************************
 * AGC: none (pad loads used by P11.agc pp. 542-550) -- store the P11 pad
 * loads (pitch polynomial, roll programme, launch azimuth, Saturn scale and
 * rates).
 *
 * FUNCTION NAME: agc_earth_orbit_insertion_monitor_set_padload
 *
 * ARGUMENTS:
 *   Argument  Type (I/O)  Description (scale)
 *   --------  ----------  --------------------------------------------
 *   pad       const agc_earth_orbit_insertion_monitor_padload_t * (I)
 *                         the pad loads; polynum is the polynomial
 *                         degree, 1..POLY_MAX; scales per member
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when pad is
 *          NULL or polynum is out of range
 **********************************************************************/
agc_status_t agc_earth_orbit_insertion_monitor_set_padload(const agc_earth_orbit_insertion_monitor_padload_t *pad)
{
    if (!AGC_ASSERT(pad != NULL) || !AGC_ASSERT(pad->polynum >= 1 && pad->polynum <= AGC_EARTH_ORBIT_INSERTION_MONITOR_POLY_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    insertion_pad = *pad;

    return (AGC_STATUS_OK);
}

/* ======================================================================
 * Liftoff (pp. 535-537)
 * ==================================================================== */

/**********************************************************************
 * P11 / P11+7 (p. 535): restarts to REP11 (3.11), group 5 off; TLIFTOFF =
 * clock at liftoff, the clock zeroed, restarts to REP11A (3.13).
 *
 * FUNCTION NAME: agc_job_earth_orbit_insertion_monitor
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_earth_orbit_insertion_monitor(agc_job_ctx_t *ctx)
{
    agc_phase_change(000113);                    /* -PHASE3 = REP11S   */
    lifttemp_set = false;                        /* CS ZERO TS LIFTTEMP */
    agc_phase_kill(5);                           /* prelaunch group 5   */
    tliftoff_set = false;                        /* REP11SA in TLIFTOFF */
    lifttemp     = (agc_dp_t) agc_time_now_cs(); /* DCA TIME2           */
    lifttemp_set = true;
    agc_time_set_cs(0u);                         /* zero CMC clock      */
    agc_erase_set_tevent(lifttemp);              /* TLIFTOFF            */
    tliftoff_set = true;
    agc_phase_change(000133);                    /* REP11A-1: 3.13      */

    if (!AGC_ASSERT(agc_time_now_cs() < 100u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_earth_orbit_insertion_monitor_ephemeris(ctx));
}

/**********************************************************************
 * REP11A (pp. 535-536): TEPHEM1 = TEPHEM + TLIFTOFF, restart there.
 *
 * FUNCTION NAME: agc_job_earth_orbit_insertion_monitor_ephemeris
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_earth_orbit_insertion_monitor_ephemeris(agc_job_ctx_t *ctx)
{
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_PREREAD, AGC_TASK_NONE, (int16_t) 022, false, 0U };

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(tliftoff_set))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    tephem1 = agc_erase_tephem() + (agc_tp_t) agc_erase_tevent(); /* DAS/ADS */
    agc_relint();
    agc_phase_change_var(005023, &v);                             /* PRIO22         */

    return (agc_job_earth_orbit_insertion_monitor_preread(ctx));
}

/**********************************************************************
 * After the 05023 PHASCHNG (p. 536): TEPHEM = TEPHEM1, PREREAD1.
 *
 * FUNCTION NAME: agc_job_earth_orbit_insertion_monitor_preread
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_earth_orbit_insertion_monitor_preread(agc_job_ctx_t *ctx)
{
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_EARTH_ORBIT_INSERTION_MONITOR_SETUP, AGC_TASK_NONE, (int16_t) 022, false, 0U };
    agc_status_t    st; /* PREREAD1 status */

    agc_inhint();
    agc_erase_set_tephem(tephem1);
    agc_relint();
    st = agc_guid_preread1_csm(); /* zero PIPs, AVERAGEG */
    agc_phase_change_var(005023, &v);

    if (!AGC_ASSERT(agc_erase_tephem() == tephem1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_earth_orbit_insertion_monitor_setup(ctx) : st); /* go on if OK */
}

/**********************************************************************
 * (p. 536-537): ATERTASK in .5 s, MM 11, NODOP01, powered-flight downlist,
 * CLEANDSP, restarts 4.51 / 3.7, then EARTHR (exit P11OUT -> MATRXJOB in
 * this job).
 *
 * FUNCTION NAME: agc_job_earth_orbit_insertion_monitor_setup
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_earth_orbit_insertion_monitor_setup(agc_job_ctx_t *ctx)
{
    agc_status_t st; /* EARTHR status */

    agc_status_t sw = agc_waitlist_call(50, AGC_TASK_ATERTASK); /* .5SEC     */

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_status_t sj = agc_phase_newmode(11); /* NEWMODEX status */

    if (sj != AGC_STATUS_OK)
    {
        return (sj);
    }

    agc_flag_set(AGC_FLAG_NO_PRELAUNCH_INITIALIZATION);
    agc_telem_select_list(INSERTION_POWDNCOD);
    const agc_status_t sd = agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0, NULL); /* V75 case  */

    if (sd != AGC_STATUS_OK)
    {
        return (sd); /* PRIOBORT 01502 or exec overflow: job abandoned */
    }

    agc_phase_change2(040514, 000073, NULL); /* 4.51, 3.7 */
    st = agc_csm_prelaunch_earthr_at_liftoff(entry_ip(), agc_erase_tevent());

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_NO_PRELAUNCH_INITIALIZATION)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_matrxjob() : st); /* go on if OK */
} /* agc_job_earth_orbit_insertion_monitor_setup */

/**********************************************************************
 * REP11 (p. 539): restart of group 3 at 3.11.
 *
 * FUNCTION NAME: agc_job_earth_orbit_insertion_monitor_restart
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_earth_orbit_insertion_monitor_restart(agc_job_ctx_t *ctx)
{
    if (agc_phase_get(5) != 0u)
    { /* CCS PHASE5          */
        return (AGC_STATUS_OK); /* ENDOFJOB            */
    }

    if (!lifttemp_set)
    {
        return (agc_job_earth_orbit_insertion_monitor(ctx)); /* P11+7 (clock first) */
    }

    if (!tliftoff_set)
    {
        agc_dp_t liftoff = lifttemp; /* TLIFTOFF, cs B28 */

        if (agc_time_now_cs() >= 16384u)
        { /* CCS TIME2 > 0: REP11A -5 zeroes the clock and the DXCH TIME2 hands the clock read now to TLIFTOFF. */
            liftoff = (agc_dp_t) agc_time_now_cs();
            agc_time_set_cs(0u);
        }

        agc_erase_set_tevent(liftoff); /* REP11A -2           */
        tliftoff_set = true;
    }

    agc_phase_change(000133); /* ENDREP11: 3.13      */

    if (!AGC_ASSERT(tliftoff_set))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_earth_orbit_insertion_monitor_ephemeris(ctx));
}

/* ======================================================================
 * MATRXJOB (pp. 537-539)
 * ==================================================================== */

/**********************************************************************
 * MATRXJOB tail (pp. 538-539): restart 3.x here, AVGEXIT = VHHDOT, 1/PIPADT
 * = 2 s, READACCS 2 s after liftoff, NORMLIZE.
 *
 * FUNCTION NAME: matrx_tail
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t matrx_tail(void)
{
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_MATRXJOB, AGC_TASK_NONE, 0, false, 0U };
    int32_t         dt; /* READACCS delay, cs */

    agc_phase_change_var(004023, &v);
    agc_guid_state()->avg.avgexit = AGC_JOB_VHHDOT; /* P11SCADR      */
    agc_imu_set_pipa_interval((agc_sp_t) 031000);   /* PRIO31: 2 s   */
    agc_inhint();

    /* CS ZERO / TS TBASE5: READACCS restarts relative to liftoff (no exec
     * API for TBASE5 alone; the 5.2 restart below uses the present TIME). */
    dt = INSERTION_2SECS - (int32_t) agc_erase_time1();

    if (dt <= 0)
    {
        dt = 1; /* never negative */
    }

    agc_status_t st = agc_waitlist_call((uint16_t) dt, AGC_TASK_READACCS); /* WAITLIST status */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_relint();
    agc_phase_change2(000003, 000025, NULL); /* 3 off, 5.2    */

    if (!AGC_ASSERT(dt > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_normlize_csm()); /* POSTJUMP      */
} /* matrx_tail */

/**********************************************************************
 * MATRXJOB REFSMMAT (P11.agc p. 538): the launch REFSMMAT. Z = UNIT(down),
 * X = UNIT(SIN(AZ) EAST - COS(AZ) SOUTH), Y = -UNIT(X x Z); all half-unit.
 *
 * FUNCTION NAME: launch_refsmmat
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job
 *   down      agc_vec_t       I    down direction at the pad, m B29
 *   m         agc_mat_t *     O    REFSMMAT rows, half-unit (B1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a
 *          failed assertion
 **********************************************************************/
static agc_status_t launch_refsmmat(agc_interp_t *ip, agc_vec_t down, agc_mat_t *m)
{
    agc_vec_t unitw = agc_guid_state()->avg.unitw; /* UNITW polar axis, B0 */
    agc_vec_t east;                                /* EAST, B1   */
    agc_vec_t msouth;                              /* -SOUTH, B1 */

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(m != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    m->row[2] = agc_vec_unit(ip, down, NULL, NULL);
    east      = agc_vec_unit(ip, agc_vec_cross(ip, m->row[2], unitw), NULL, NULL);
    msouth    = agc_vec_unit(ip, agc_vec_cross(ip, east, m->row[2]), NULL, NULL);
    m->row[0] = agc_vec_add(ip, agc_vec_scale(ip, msouth, agc_dp_cos(insertion_pad.launchaz)), agc_vec_scale(ip, east, agc_dp_sin(insertion_pad.launchaz)));
    m->row[0] = agc_vec_unit(ip, m->row[0], NULL, NULL);
    m->row[1] = agc_vec_neg(agc_vec_unit(ip, agc_vec_cross(ip, m->row[0], m->row[2]), NULL, NULL));

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * MATRXJOB (pp. 537-539): liftoff CDUs for the attitude-error display, RN1
 * (LALOTORV of the pad at T = 0), VN1 = WE x RN1, REFSMMAT, SATRLRT sign,
 * REFSMFLG; AVGEXIT = VHHDOT; READACCS 2 s after liftoff; NORMLIZE.
 *
 * FUNCTION NAME: agc_job_matrxjob
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_matrxjob(void)
{
    const agc_latlong_opt_t opt = {false, false};
    agc_interp_t           *ip  = entry_ip();             /* entry job interpreter state */
    agc_avg_g_t            *avg = &agc_guid_state()->avg; /* average-G data (RN1, VN1, UNITW) */
    agc_navaux_padload_t    pad;                          /* pad loads (earth radius) */
    agc_latlong_t           ll  = {insertion_pad.latitude, insertion_pad.padlong, insertion_pad.pgncsalt, 0};
    agc_vec_t               down;                         /* down at the pad, m B29 */
    agc_mat_t               m;                            /* REFSMMAT to store, half-unit */
    agc_status_t            st;                           /* status */

    ogc_lo[0] = agc_guid_cdulogic(agc_erase_cdux()); /* OGC B-1: revs */
    ogc_lo[1] = agc_guid_cdulogic(agc_erase_cduy());
    ogc_lo[2] = agc_guid_cdulogic(agc_erase_cduz());
    agc_flag_clear(AGC_FLAG_ERADFLAG);
    st        = agc_navaux_get_padload(&pad);

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_to_r(ip, &pad, opt, &ll, 0, &avg->rn1, NULL);
    }

    if (st == AGC_STATUS_OK)
    {
        /**********************************************************************
         * GETDOWN (p. 540): LALOTORV +5 with time 0 and GAMMA = 1: the geodetic
         * vertical at the pad, negated (VCOMP).
         *
         * FUNCTION NAME: getdown
         *
         * ARGUMENTS:
         *   Argument  Type            I/O  Description (scale)
         *   --------  --------------  ---  --------------------------------------
         *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
         *                                  DVSIGN, alarms
         *   down      agc_vec_t *     O    the negated pad position at T = 0 (down
         *                                  direction), m B29
         *
         * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG on a failed
         *          assertion; else the pad load or RP-TO-R status
         **********************************************************************/
        agc_navaux_padload_t pad_getdown;                                       /* pad loads (earth radius) */
        agc_dp_t             cl         = agc_dp_cos(insertion_pad.latitude);   /* COS LAT /2, B1 */
        agc_vec_t            rp;                                                /* unit RP, B2 */
        agc_status_t         st_getdown = agc_navaux_get_padload(&pad_getdown); /* status */

        if (st_getdown == AGC_STATUS_OK)
        {
            /* LALOTORV +5 multiplies with DMPR (rounded); GAMMA = DPHALF (1 B-1). */
            rp         = agc_vec_make(agc_dp_mul(ip, agc_dp_cos(insertion_pad.padlong), cl), agc_dp_mul(ip, agc_dp_sin(insertion_pad.padlong), cl), agc_dp_mul(ip, INSERTION_HALFREV, agc_dp_sin(insertion_pad.latitude)));
            rp         = agc_vec_unit(ip, rp, NULL, NULL);
            st_getdown = agc_planet_rp_to_r(ip, &pad_getdown, false, rp, 0, &down);
            down       = agc_vec_neg(down);
        }

        st = st_getdown;
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = launch_refsmmat(ip, down, &m); /* REFSMMAT at liftoff */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    avg->vn1              = agc_vec_shl(ip, agc_vec_scale(ip, agc_vec_cross(ip, avg->rn1, avg->unitw), INSERTION_MERTHRAT), 4); /* B7 */
    agc_erase_set_refsmmat(m);
    insertion_pad.satrlrt = agc_dp_sign(insertion_pad.satrlrt, agc_dp_add(ip, agc_dp_sub(ip, INSERTION_HALFREV, insertion_pad.launchaz), insertion_pad.azimuth));
    agc_flag_set(AGC_FLAG_REFSMFLG);

    return (matrx_tail());
} /* agc_job_matrxjob */

/* ======================================================================
 * N62 display (pp. 540, 546)
 * ==================================================================== */

/**********************************************************************
 * S11.1 (p. 546): VMAGI, HDOT, ALTI for N62.
 *
 * FUNCTION NAME: agc_csm_velocity_altitude_rate
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *
 * RETURNS: none
 **********************************************************************/
void agc_csm_velocity_altitude_rate(agc_interp_t *ip)
{
    agc_vec_t rn   = agc_erase_rn(); /* RN, m B29 */
    agc_vec_t vn   = agc_erase_vn(); /* VN, m/cs B7 */
    agc_vec_t ur;                    /* UNIT(RN), B1 */
    agc_dp_t  rmag = 0;              /* |RN|, m B29 */
    agc_dp_t  base = INSERTION_RPAD; /* pad or landing radius, m B29 */

    insertion_vmagi = agc_vec_abval(ip, vn); /* 2^7 m/cs    */
    ur              = agc_vec_unit(ip, rn, &rmag, NULL);
    insertion_hdot  = entry_tp_shl(ip, agc_vec_dot_tp(ip, ur, vn), 1);

    if (agc_flag_test(AGC_FLAG_AMOONFLG))
    {
        base = agc_dp_shr(agc_vec_abval(ip, agc_erase_rls()), 2); /* B27->B29 */
    }

    insertion_alti = agc_dp_sub(ip, rmag, base); /* BDSU 36D     */
}

/**********************************************************************
 * AGC: none (N62 data, P11.agc p. 546) -- the S11.1 values for display or
 * test.
 *
 * FUNCTION NAME: agc_earth_orbit_insertion_monitor_readout
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   vmagi     agc_dp_t *  O    VI, inertial speed, m/cs B7
 *   hdot      agc_dp_t *  O    HDOT, rate of change of altitude, m/cs B7
 *   alti      agc_dp_t *  O    H, altitude above the pad radius, m B29
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when a
 *          pointer is NULL
 **********************************************************************/
agc_status_t agc_earth_orbit_insertion_monitor_readout(agc_dp_t *vmagi,
                                                       agc_dp_t *hdot,
                                                       agc_dp_t *alti)
{
    *vmagi = insertion_vmagi;
    *hdot  = insertion_hdot;
    *alti  = insertion_alti;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * VHHDOT (p. 540): AVGEXIT each 2 s: S11.1, restart 5.3, REGODSP V06N62
 * (which ends the job).
 *
 * FUNCTION NAME: agc_job_vhhdot
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_vhhdot(agc_job_ctx_t *ctx)
{
    agc_csm_velocity_altitude_rate(entry_ip());
    agc_phase_change(000035);

    return (agc_dsky_display(ctx, AGC_DSKY_REGODSP, agc_dsky_vn(6, 62), NULL));
}

/* ======================================================================
 * Attitude error display (pp. 540-546)
 * ==================================================================== */

/**********************************************************************
 * ATERTASK (p. 540): start ATERJOB, initialise NEEDLER, SATSW = 1.
 *
 * FUNCTION NAME: agc_task_atertask
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_task_atertask(void)
{
    agc_status_t st = agc_exec_findvac(AGC_ENTRY_PRIO1, AGC_JOB_ATERJOB); /* FINDVAC status */

    agc_status_t sw = agc_dap_needler(insertion_ak[0], insertion_ak[1], /* RCSFLAGS BIT3 */
                                      insertion_ak[2],
                                      true);

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    satsw = 1; /* display      */

    return (st);
}

/**********************************************************************
 * AK = TGSC (DESIRED - CDU) (pp. 544-545), TGSC rows (1, SIN Z, 0), (0, COS
 * X COS Z, SIN X), (0, -SIN X COS Z, COS X), with the errors ordered DTHETA
 * (Y), DPSI (Z), DPHI (X); AK scaled 2 revs.
 *
 * FUNCTION NAME: agc_earth_orbit_insertion_monitor_attitude_errors
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   desired   const agc_dp_t  I    desired OGC IGC MGC (X Y Z), revs B0
 *   cdu       const uint16_t  I    CDUX CDUY CDUZ patterns
 *   ak_out    agc_sp_t        O    AK AK1 AK2 attitude errors, 2 revs (SP
 *                                  high words)
 *
 * RETURNS: none
 **********************************************************************/
void agc_earth_orbit_insertion_monitor_attitude_errors(const agc_dp_t desired[3], const uint16_t cdu[3], agc_sp_t ak_out[3])
{
    agc_interp_t *ip = entry_ip(); /* entry job interpreter state */
    agc_dp_t      d[3];            /* DTHETA DPSI DPHI, revs B0 */
    agc_dp_t      sx;              /* SIN CDUX /2, B1 */
    agc_dp_t      cx;              /* COS CDUX /2, B1 */
    agc_dp_t      sz;              /* SIN CDUZ /2, B1 */
    agc_dp_t      cz;              /* COS CDUZ /2, B1 */
    agc_dp_t      p;               /* DMP PUSH: COS CDUZ DTHETA, B1 */
    agc_dp_t      q;               /* pushed product SL1, B1 */
    agc_tp_t      t;               /* TP MPAC, B1 */
    int           i;               /* loop index */

    for (i = 0; i < 3; i++)
    { /* order Y Z X        */
        int     k = (i + 1) % 3;                                                         /* desired/cdu: X Y Z */
        int32_t w = (int32_t) agc_rtb_1sto2s(desired[k]) - (int32_t) (cdu[k] & 077777u); /* desired - CDU (MSU), /180 */

        d[i] = agc_dp_from_sp(entry_angle_wrap(w)); /* DELSTOR (MSU)  */
    }

    sx = agc_dp_sin(agc_guid_cdulogic(cdu[0]));
    cx = agc_dp_cos(agc_guid_cdulogic(cdu[0]));
    sz = agc_dp_sin(agc_guid_cdulogic(cdu[2]));
    cz = agc_dp_cos(agc_guid_cdulogic(cdu[2]));

    /* NOPOLYM: .5(DPHI + DTHETA SIN(CDUZ)): DMP SL1, DAD, SR2 (truncating)
     * on the TP MPAC.                                                    */
    t         = agc_tp_shift(ip, agc_dp_mul_tp(sz, d[0]), 1);
    t         = agc_tp_add(ip, t, agc_tp_from_dp(d[2]));
    ak_out[0] = agc_dp_high(agc_tp_to_dp(agc_tp_shift(ip, t, -2)));

    /* ATTDISP2: COS(CDUZ) DTHETA is PUSHed (two words); each product with
     * COS/SIN(CDUX) is shifted on the TP and PUSHed; the DPSI product
     * keeps its MPAC+2 through the DAD / DSU and the last SL1.           */
    p         = agc_dp_mul_trunc(cz, d[0]);          /* DMP PUSH          */
    q         = entry_tp_shl(ip, agc_dp_mul_tp(p, cx), 1);
    t         = agc_tp_add(ip, agc_dp_mul_tp(sx, d[1]), agc_tp_from_dp(q));
    ak_out[1] = agc_dp_high(entry_tp_shl(ip, t, 1)); /* 17D       */
    q         = entry_tp_shl(ip, agc_dp_mul_tp(p, sx), 1);
    t         = agc_tp_add(ip, agc_dp_mul_tp(cx, d[1]), agc_tp_from_dp(agc_dp_neg(q)));
    ak_out[2] = agc_dp_high(entry_tp_shl(ip, t, 1)); /* 18D       */
} /* agc_earth_orbit_insertion_monitor_attitude_errors */

/**********************************************************************
 * ATTDISP / ATTDISP1 / ATTDISPR (pp. 542-544): desired gimbal angles from
 * the pitch polynomial and the roll programme (CALCGTA), or the liftoff
 * attitude (NOPOLY).
 *
 * FUNCTION NAME: desired_attitude
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *   desired   agc_dp_t        O    OGC IGC MGC from CALCGTA, or the liftoff
 *                                  angles (NOPOLY), revs B0
 *
 * RETURNS: none
 **********************************************************************/
static void desired_attitude(agc_interp_t *ip, agc_dp_t desired[3])
{
    int32_t   t = (int32_t) agc_erase_time1() - insertion_pad.rpstart; /* TIME1 - RPSTART, cs B14 */
    agc_dp_t  pitch;                                                   /* PITCH, revs B0 */
    agc_dp_t  x1;                                                      /* A1 + A2 T + ..., 32 revs B5 */
    agc_dp_t  roll;                                                    /* ROLL, revs B0 */
    agc_mat_t dc;                                                      /* TSMV rows XDC YDC ZDC, B1 */

    desired[0] = ogc_lo[0]; /* NOPOLY    */
    desired[1] = ogc_lo[1];
    desired[2] = ogc_lo[2];

    if (t <= 0 || t + insertion_pad.polystop > 0 || agc_erase_time2() > 0)
    {
        return;
    }

    spolyarg = (agc_sp_t) t;

    /* POWRSERS leaves the last DMPSUB's MPAC+2 (the DAS of A0 adds to
     * MPAC, MPAC+1 only); SL 5 shifts it up. Horner to A1, then x, A0.   */
    x1    = (insertion_pad.polynum > 1) ? agc_dp_poly(agc_dp_from_sp(spolyarg), &insertion_pad.poly[1], insertion_pad.polynum - 1) : insertion_pad.poly[1]; /* degree 1: A1 */
    pitch = entry_tp_shl(ip, agc_tp_add(ip, agc_dp_mul_tp(x1, agc_dp_from_sp(spolyarg)), agc_tp_from_dp(insertion_pad.poly[0])), 5);                        /* SL 5: 32 revs -> revs */
    roll  = entry_tp_shl(ip, agc_dp_mul_tp(agc_dp_from_sp(spolyarg), insertion_pad.satrlrt), 14);                                                           /* SLOAD DMP SL 14D: the TP low word moves up      */
    roll  = agc_dp_sub(ip, agc_dp_add(ip, agc_dp_sub(ip, roll, INSERTION_HALFREV), insertion_pad.launchaz), insertion_pad.azimuth);

    /* RLTST: CA MPAC, MP SATRLRT +1 (the low word), BZMF: rollout is
     * over only when the high word of that SP product is above +0.      */
    if (entry_sp_mp(agc_dp_high(roll), agc_dp_low(insertion_pad.satrlrt)) > 0)
    {
        roll = 0; /* rollout done: MBDYTCTL +2    */
    }

    dc.row[0] = agc_vec_make(agc_dp_sin(pitch), 0, agc_dp_neg(agc_dp_cos(pitch)));
    dc.row[1] = agc_vec_make(agc_dp_neg(entry_tp_shl(ip, agc_dp_mul_tp(agc_dp_sin(roll), agc_dp_cos(pitch)), 1)), agc_dp_neg(agc_dp_cos(roll)), agc_dp_neg(entry_tp_shl(ip, agc_dp_mul_tp(agc_dp_sin(roll), agc_dp_sin(pitch)), 1)));
    dc.row[2] = agc_vec_make(agc_dp_neg(entry_tp_shl(ip, agc_dp_mul_tp(agc_dp_cos(roll), agc_dp_cos(pitch)), 1)), agc_dp_sin(roll), agc_dp_neg(entry_tp_shl(ip, agc_dp_mul_tp(agc_dp_cos(roll), agc_dp_sin(pitch)), 1)));

    for (t = 0; t < 3; t++)
    {
        dc.row[t] = agc_vec_unit(ip, dc.row[t], NULL, NULL);
    }

    agc_imu_calcgta(ip, &dc, desired); /* OGC IGC MGC */
} /* desired_attitude */

/**********************************************************************
 * TLOAD BIASAK, TAD AK (p. 545): the three SP words added as one TP number
 * (DAS on the minor pair, carries into the next word up).
 *
 * FUNCTION NAME: insertion_tad
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   m         const agc_sp_t  I    TP MPAC words (BIASAK), B0
 *   x         const agc_sp_t  I    TP operand words (AK), B0
 *   out       agc_sp_t        O    sum words (overflow kept modulo 1 in
 *                                  each), B0
 *
 * RETURNS: none
 **********************************************************************/
static void insertion_tad(const agc_sp_t m[3], const agc_sp_t x[3], agc_sp_t out[3])
{
    int32_t sum   = (int32_t) m[2] + (int32_t) x[2];  /* word 2, 2^-28  */
    int32_t carry = (sum >= 16384) - (sum <= -16384); /* carry into the next word */

    out[2] = (agc_sp_t) (sum - carry * 16384);
    sum    = (int32_t) m[1] + (int32_t) x[1] + carry;                     /* word 1, 2^-14  */
    carry  = (sum >= 16384) - (sum <= -16384);
    out[1] = (agc_sp_t) (sum - carry * 16384);
    out[0] = entry_sp_store_ovf((int32_t) m[0] + (int32_t) x[0] + carry); /* TS: OVFIND only  */
}

/**********************************************************************
 * RESCALES with the TS AKn after it (p. 545): AK x SATSCALE x 4 (MP,
 * DDOUBL, DDOUBL). A result past +-1 is stored modulo 1 and the TS skips
 * the next CA AKn+1, so the next word is rescaled from A = +-1. DEVIATION:
 * a skip after TS AK2 would execute the CADR NEEDLER word; that third
 * overflow is stored modulo 1 without the skip.
 *
 * FUNCTION NAME: insertion_rescales
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   ak        agc_sp_t  IO   AK AK1 AK2 before and after RESCALES, B0
 *   satscale  agc_sp_t  I    SATSCALE pad load, B0
 *
 * RETURNS: none
 **********************************************************************/
static void insertion_rescales(agc_sp_t ak[3], agc_sp_t satscale)
{
    int32_t a = ak[0]; /* A for the next RESCALES    */
    int     i;         /* word index */

    for (i = 0; i < 3; i++)
    {
        int32_t h = (int32_t) (((int64_t) 4 * a * satscale) / 16384); /* A after MP, DDOUBL, DDOUBL, SP units */

        if (h > 32767)
        { /* DDOUBL past the 16-bit A: end-around */
            h -= 65535;
        }
        else if (h < -32767)
        {
            h += 65535;
        }

        ak[i] = entry_sp_store_ovf(h);

        if (h >= 16384 || h <= -16384)
        {
            a = (h > 0) ? 1 : -1; /* TS overflow: A = +-1, skip CA */
        }
        else
        {
            a = (i < 2) ? (int32_t) ak[i + 1] : 0; /* CA AKn+1 */
        }
    }
}

/**********************************************************************
 * ATERJOB (pp. 542-546): one display cycle, then DELAYJOB .25 s.
 *
 * FUNCTION NAME: agc_job_aterjob
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_aterjob(void)
{
    agc_dp_t desired[3]; /* desired gimbal angles, revs B0 */
    uint16_t cdu[3];     /* CDUX..CDUZ patterns */
    agc_sp_t ak[3];      /* AK AK1 AK2, B0 */

    if (agc_flag_test(AGC_FLAG_DAPBIT1) || agc_flag_test(AGC_FLAG_DAPBIT2))
    {
        return (AGC_STATUS_OK); /* Saturn stick: end   */
    }

    /**********************************************************************
     * ATERJOB, STRSAT, ATRESET (pp. 541, 546): which way this cycle goes.
     * Returns true when the needles are re-initialised (ATRESET).
     *
     * FUNCTION NAME: aterjob_needs_reset
     *
     * ARGUMENTS: none
     *
     * RETURNS: bool -- bool -- true when this cycle goes to ATRESET (NEEDLER
     *          init and 60 ms)
     **********************************************************************/
    bool switch_on = (agc_chan_read(030) & AGC_CH030_SCCTLSATURN) == 0u; /* S/C control of Saturn switch on */

    /* STRSAT: SATSW <= 0 -> ATTDISP, else ATRESET. Switch off: CCS
     * SATSW: > 0 -> ATTDISP, +0 or < 0 -> ATRESET.                       */
    if ((switch_on ? (satsw > 0) : (satsw <= 0)))
    { /* ATRESET: NEEDLER init, 60 ms        */
        agc_status_t sw = agc_dap_needler(insertion_ak[0], insertion_ak[1], /* NEEDLER status */
                                          insertion_ak[2],
                                          true);

        if (sw != AGC_STATUS_OK)
        {
            return (sw);
        }

        return (agc_svc_delayjob(6, AGC_JOB_ATRESET_AFTER_DELAY));
    }

    desired_attitude(entry_ip(), desired);
    cdu[0] = agc_erase_cdux();
    cdu[1] = agc_erase_cduy();
    cdu[2] = agc_erase_cduz();
    agc_earth_orbit_insertion_monitor_attitude_errors(desired, cdu, ak);

    if (satsw <= 0)
    { /* +0: AKLOAD, then STEERSAT        */
        if (satsw == 0)
        {
            biasak[0] = (agc_sp_t) (-ak[0]);
            biasak[1] = (agc_sp_t) (-ak[1]);
            biasak[2] = (agc_sp_t) (-ak[2]);
            satsw     = -1; /* steer               */
        }

        insertion_tad(biasak, ak, ak);
        insertion_rescales(ak, insertion_pad.satscale);
    }

    insertion_ak[0] = ak[0];
    insertion_ak[1] = ak[1];
    insertion_ak[2] = ak[2];
    agc_status_t st = agc_dap_needler(ak[0], ak[1], ak[2], false); /* SATOUT */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_svc_delayjob(INSERTION_ATERTASK_DELAY, AGC_JOB_ATERJOB)); /* ATERSET  */
} /* agc_job_aterjob */

/**********************************************************************
 * ATRESET +5 (p. 546): after the 60 ms wait, SATSW > 0: enable S-IVB
 * takeover (TAKEON) and SATSW = 0; else SATSW = 1 and takeover off.
 *
 * FUNCTION NAME: agc_job_atreset_after_delay
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_atreset_after_delay(void)
{
    if (satsw > 0)
    {
        agc_chan_or(012, AGC_CH012_SIVBTAKEOVER); /* TAKEON   */
        satsw = 0;
    }
    else
    {
        satsw = 1;
        agc_chan_and(012, (uint16_t) ~AGC_CH012_SIVBTAKEOVER);
    }

    if (!AGC_ASSERT(satsw == 0 || satsw == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* SATOUT: NEEDLER with the AK words as they stand.                  */
    agc_status_t st = agc_dap_needler(insertion_ak[0], insertion_ak[1], insertion_ak[2], false); /* NEEDLER status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_svc_delayjob(INSERTION_ATERTASK_DELAY, AGC_JOB_ATERJOB));
}

/* ======================================================================
 * Saturn takeover (pp. 547-550)
 * ==================================================================== */

/**********************************************************************
 * SATSTKON (p. 549): V46 with DAPDATR1 bits 13,14: T5 to REDOSAT now,
 * FLAGWRD6 bits 15,14 (DAPBIT1, DAPBIT2), ZEROJET, exit via GOPIN.
 *
 * FUNCTION NAME: agc_csm_satstkon
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_csm_satstkon(void)
{
    agc_inhint();
    agc_rupt_set_t5_task(AGC_TASK_REDOSAT);
    agc_rupt_set_time(5, 037777); /* POSMAX   */
    agc_flag_set(AGC_FLAG_DAPBIT1);
    agc_flag_set(AGC_FLAG_DAPBIT2);
    agc_csm_dap_zerojet();
    agc_relint();

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_DAPBIT1)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_gopin());
}

/**********************************************************************
 * REDOSAT (p. 549): NEEDLER init, S-IVB takeover on, SATSTICK in 100 ms.
 *
 * FUNCTION NAME: agc_task_redosat
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_task_redosat(void)
{
    agc_status_t st = agc_dap_needler(insertion_ak[0], insertion_ak[1], /* NEEDLER status */
                                      insertion_ak[2],
                                      true);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_chan_or(012, AGC_CH012_SIVBTAKEOVER);
    agc_rupt_set_t5_task(AGC_TASK_SATSTICK);

    if (!AGC_ASSERT(agc_rupt_t5_task() == AGC_TASK_SATSTICK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((agc_chan_read(012) & AGC_CH012_SIVBTAKEOVER) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rupt_set_time(5, INSERTION_100MST5);

    return (AGC_STATUS_OK); /* 100MST5  */
}

/**********************************************************************
 * SATSTICK (p. 550): every 100 ms (T5): the RHC rotation bits of channel 31
 * (inverted logic: STIKBITS RXOR CHAN31, MASK STIKBITS) set RMANNDX,
 * PMANNDX, YMANNDX (STICKCHK, dap-csm); the indexed SATRATE pad loads go to
 * AK (roll), AK1 (pitch), AK2 (yaw) and out through NEEDLER.
 *
 * FUNCTION NAME: agc_task_satstick
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_task_satstick(void)
{
    const agc_csm_rcs_t *r;    /* RCS DAP data (MANNDX) */
    uint16_t             bits; /* RHC bits, inverted */
    agc_status_t         st;   /* STICKCHK status */

    agc_rupt_set_t5_task(AGC_TASK_SATSTICK); /* 2SATSTCK */
    agc_rupt_set_time(5, INSERTION_100MST5); /* 100MST5  */
    bits = (uint16_t) ((INSERTION_STIKBITS ^ agc_chan_read(031)) & INSERTION_STIKBITS);
    st   = agc_csm_dap_stickchk(bits);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    r = agc_csm_dap_rcs();

    if (!AGC_ASSERT(r->manndx[0] >= 0 && r->manndx[0] <= 3) || !AGC_ASSERT(r->manndx[1] >= 0 && r->manndx[1] <= 3) || !AGC_ASSERT(r->manndx[2] >= 0 && r->manndx[2] <= 3))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    insertion_ak[0] = insertion_pad.satrate[r->manndx[0]]; /* AK  roll  */
    insertion_ak[1] = insertion_pad.satrate[r->manndx[1]]; /* AK1 pitch */
    insertion_ak[2] = insertion_pad.satrate[r->manndx[2]]; /* AK2 yaw   */

    return (agc_dap_needler(insertion_ak[0], insertion_ak[1], insertion_ak[2], false));
}
