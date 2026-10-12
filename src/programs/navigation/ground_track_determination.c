/**********************************************************************
 * FILE NAME: ground_track_determination.c
 *
 * PURPOSE:
 *   P21 GROUND TRACK DETERMINATION PROGRAM, both vehicles: "To provide
 *   the astronaut details of the LM or CSM ground track without the need
 *   for ground communication (requested by DSKY)." The crew picks the
 *   vehicle (option 2: 1 this, 2 other) and a time (noun 34); the state
 *   is integrated to that time and LAT, LONG and ALT are shown (noun
 *   43), with |V| and the flight path angle for noun 73. V32 recycles
 *   ten minutes later, integrating from the saved base vector.
 *
 * AGC SOURCE:
 *   Comanche055 GROUND_TRACKING_DETERMINATION_PROGRAM.agc pp. 456-459
 *   (PROG21, P21PROG1, P21CONT, P21VSAVE, P21DSP)
 *   Luminary099 GROUND_TRACKING_DETERMINATION_PROGRAM.agc (differs:
 *   VINTFLAG means "integrate the CSM", so the CM sets it for this
 *   vehicle and the LM for the other one; the LM does not keep the base
 *   vector (P21FLAG) of its own state while on the surface)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  ----------------------------
 *   OPTION2    word       I/O  -          vehicle (1 this, 2 other)
 *   DSPTEM1    agc_dp_t   I/O  cs 2^28    time (N34)
 *   LAT LONG ALT           O    rev, m     N43 values (navaux)
 *   VINTFLAG DIM0FLAG INTYPFLG MOONFLAG LUNAFLAG ERADFLAG P21FLAG
 *              flags      I/O  -          integration / display control
 *   P21TIME P21BASER P21BASEV P21ORIG P21VEL P21GAM P21ALT (agc_rendz_t)
 *              I/O  cs 2^28, body scale, m/cs 2^7, rev, m 2^29
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ----------------------------------------------
 *   agc_integ_integrv     INTEGRV: the permanent state to TDEC1
 *   agc_integ_integrvs    INTEGRVS: from the base vector
 *   agc_latlong_from_r    LAT-LONG
 *   agc_dsky_display      GOPERF4 option request
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. Jobs return AGC_STATUS_BAD_ARG for a NULL ctx, else the
 *   status of the display, integration or LAT-LONG call that failed (an
 *   INTSTALL sleep resumes the compute job from its start).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of the major mode P21.
 *
 * NOTES:
 *   Scales: RATT m 2^29, VATT m/cs 2^7, base vectors body scale, times
 *   cs 2^28, LAT LONG revolutions, P21ALT m 2^29 (altitude / 100).
 **********************************************************************/
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* K.01 2DEC .01 (p.459) */
#define GTD_K01 ((agc_dp_t) 2684355) /* K.01, .01 at 2^0 */

/* GOPERF4 option code: BIT2 = "option 2", vehicle */
#define GTD_OPTION_CODE ((uint16_t) 2) /* option 2 */

/**********************************************************************
 * PROG21 (p.457): OPTION2 = 1 (this vehicle assumed).
 * AGC: PROG21 (GROUND_TRACKING_DETERMINATION_PROGRAM.agc p.457)
 *
 * FUNCTION NAME: agc_job_ground_track_determination
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
agc_status_t agc_job_ground_track_determination(agc_job_ctx_t *ctx)
{
    agc_status_t st = rz_set_option(2, 1U); /* OPTION2 = 1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_ground_track_vehicle(ctx));
}

/**********************************************************************
 * GOPERF4 option 2 (vehicle in R2): TERMINATE -> P00, PROCEED -> time,
 * data load ("TC -5") -> the request again.
 * AGC: PROG21 GOPERF4 (GROUND_TRACKING_DETERMINATION_PROGRAM.agc p.457)
 *
 * FUNCTION NAME: agc_job_ground_track_vehicle
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
agc_status_t agc_job_ground_track_vehicle(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont; /* astronaut returns */

    cont.terminate = AGC_JOB_GOTOPOOH;
    cont.proceed   = AGC_JOB_GROUND_TRACK_TIME;
    cont.enter     = AGC_JOB_GROUND_TRACK_VEHICLE;

    return (agc_dsky_display(ctx, AGC_DSKY_GOPERF4, GTD_OPTION_CODE, &cont));
}

/**********************************************************************
 * P21PROG1 (p.457): V06N34 flashing (time in DSPTEM1).
 * AGC: P21PROG1 (GROUND_TRACKING_DETERMINATION_PROGRAM.agc p.457)
 *
 * FUNCTION NAME: agc_job_ground_track_time
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
agc_status_t agc_job_ground_track_time(agc_job_ctx_t *ctx)
{
    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 34U, AGC_JOB_GOTOPOOH, AGC_JOB_GROUND_TRACK_COMPUTE, AGC_JOB_GROUND_TRACK_TIME));
}

/**********************************************************************
 * P21VSAVE (p.458): |V| and the flight path angle.
 * AGC: P21VSAVE (GROUND_TRACKING_DETERMINATION_PROGRAM.agc p.458)
 *
 * FUNCTION NAME: agc_ground_track_determination_flight_path
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   ratt      agc_vec_t       I    position at TAT, m 2^29
 *   vatt      agc_vec_t       I    velocity at TAT, m/cs 2^7
 *   vatt1     agc_vec_t       I    VATT1, body scale (m/cs 2^7 / 2^5)
 *   x2        int             I    X2 of the integration (0 earth, 2 moon)
 *   vel       agc_dp_t *      O    |V|, m/cs 2^7
 *   gamma     agc_dp_t *      O    flight path angle, revolutions 2^0
 **********************************************************************/
void agc_ground_track_determination_flight_path(agc_interp_t *ip, agc_vec_t ratt, agc_vec_t vatt, agc_vec_t vatt1, int x2, agc_dp_t *vel, agc_dp_t *gamma)
{
    /* ABVAL SL* 0,2: count 0 - X2, so the moon-scale 2^5 speed is shifted
     * right 2 to 2^7.                                                   */
    *vel = rz_sl_star(ip, rz_abval(ip, vatt1), 0, x2);
    agc_dp_t c; /* U(R).V, m/cs 2^8 */

    c = agc_vec_dot(ip, rz_unit(ip, ratt), vatt);

    *gamma = agc_dp_asin(ip, agc_dp_div(ip, c, *vel)); /* DDV ASIN   */
}

/**********************************************************************
 * TDEC1 = DSPTEM1; INTSTALL; first pass or P21CONT; P21VSAVE; P21DSP.
 * An INTSTALL sleep resumes this job from its start.
 * AGC: P21PROG1 .. P21DSP (GROUND_TRACKING_DETERMINATION_PROGRAM.agc
 * pp.457-458)
 *
 * FUNCTION NAME: agc_job_ground_track_compute
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
agc_status_t agc_job_ground_track_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz    = agc_rendz_erasable();     /* rendezvous erasables */
    agc_dp_t     tdec1 = rz_dsptem_dp(RZ_DSPTEM1); /* TDEC1, cs 2^28 */
    agc_status_t st;                               /* INTSTALL */

    st = agc_integ_install(0, AGC_JOB_GROUND_TRACK_COMPUTE);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* asleep        */
    }

    agc_integ_ctx_t *ic = agc_integ_shared_ctx(); /* integration erasables */

    if (rz_vflag_test(RZ_VF_GROUND_TRACK))
    { /* BON P21CONT   */
        /* P21CONT (p.457-458): INTEGRVS from the saved base vector.
         * AGC: P21CONT (GROUND_TRACKING_DETERMINATION_PROGRAM.agc pp.457-458) */
        ic->sv.rcv = rz->ground_track_base_r;
        ic->sv.vcv = rz->ground_track_base_v;
        ic->sv.tet = rz->ground_track_time;
        rz_flag_put(AGC_FLAG_DIM0FLAG, false);
        rz_flag_put(AGC_FLAG_MOONFLAG, rz->ground_track_origin != 0);
        st         = agc_integ_integrvs(&ip, ic, tdec1);
    }
    else
    {
        /* First pass (P21FLAG clear): INTEGRV of the chosen vehicle's
         * permanent state, precision, VINTFLAG = integrate the CSM.  */
        bool other = ((rz_option(2) >> 1) != 0U);  /* SR1 BHIZ      */
        bool csm   = rz_is_csm() ? !other : other; /* VINTFLAG */

        rz_flag_put(AGC_FLAG_VINTFLAG, csm);
        rz_flag_put(AGC_FLAG_DIM0FLAG, false);
        rz_flag_put(AGC_FLAG_INTYPFLG, false); /* PRECISION     */
        st = agc_integ_integrv(&ip, ic, tdec1);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_integ_out_t o = ic->out; /* integration result */

    /* P21VSAVE (p.458): save the base vector, |V|, gamma, P21ORIG; P21FLAG
     * set (LM: not for its own state on the surface).
     * AGC: P21VSAVE (GROUND_TRACKING_DETERMINATION_PROGRAM.agc p.458) */
    rz->ground_track_time   = o.tat;
    rz->ground_track_base_r = o.ratt1;
    rz->ground_track_base_v = o.vatt1;

    agc_ground_track_determination_flight_path(&ip, o.ratt, o.vatt, o.vatt1, o.x2, &rz->ground_track_vel, &rz->ground_track_gamma); /* P21VSAVE: X2 0 or 2 from the integration */

    rz->ground_track_origin = (agc_sp_t) o.x2;
    bool this_vehicle       = ((rz_option(2) >> 1) == 0U); /* option 1 */

    if (rz_is_csm() || !this_vehicle || !agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        rz_vflag_set(RZ_VF_GROUND_TRACK, true);
    }

    /* P21DSP (p.458): LAT-LONG of RATT at TAT (LUNAFLAG from X2, ERADFLAG
     * clear), P21ALT = ALT x .01; V06N43 flashing.
     * AGC: P21DSP (GROUND_TRACKING_DETERMINATION_PROGRAM.agc p.458) */
    agc_latlong_opt_t opt; /* LUNAFLAG ERADFLAG */

    opt.lunar                       = (o.x2 != 0);
    opt.erad                        = false;
    rz_flag_put(AGC_FLAG_LUNAFLAG, opt.lunar);
    rz_flag_put(AGC_FLAG_ERADFLAG, false);
    agc_navaux_padload_t pad;                                       /* pad loads */
    agc_status_t         st_display = agc_navaux_get_padload(&pad); /* pad loads */
    agc_latlong_t        ll;                                        /* LAT LONG ALT */

    if (st_display == AGC_STATUS_OK)
    {
        st_display = agc_latlong_from_r(&ip, &pad, opt, o.ratt, o.tat, &ll);
    }

    if (st_display != AGC_STATUS_OK)
    {
        return (st_display);
    }

    st_display = agc_navaux_set_latlong(&ll); /* LAT LONG ALT */

    if (st_display != AGC_STATUS_OK)
    {
        return (st_display);
    }

    rz->ground_track_alt = agc_dp_mul_trunc(ll.alt, GTD_K01); /* DMP K.01 */

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 43U, AGC_JOB_GOTOPOOH, AGC_JOB_GOTOPOOH, AGC_JOB_GROUND_TRACK_RECYCLE));
} /* agc_job_ground_track_compute */

/**********************************************************************
 * V32 on V06N43 (p.459): DSPTEM1 = P21TIME + 600SEC, P21PROG1.
 * AGC: P21DSP V32 (GROUND_TRACKING_DETERMINATION_PROGRAM.agc p.459)
 *
 * FUNCTION NAME: agc_job_ground_track_recycle
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
agc_status_t agc_job_ground_track_recycle(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_dp_t t = agc_dp_add(&ip, agc_rendz_erasable()->ground_track_time, RZ_600SEC); /* cs 2^28 */
    rz_set_dsptem_dp(RZ_DSPTEM1, t);                                                  /* DSPTEM1 */

    return (agc_job_ground_track_time(ctx));
}
