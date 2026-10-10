/*
 * ground_track_determination.c -- P21 GROUND TRACK DETERMINATION PROGRAM,
 * both vehicles (GROUND_TRACKING_DETERMINATION_PROGRAM.agc, Comanche055
 * pp.456-459, Luminary099 the same text with the differences below).
 *
 * "To provide the astronaut details of the LM or CSM ground track without
 * the need for ground communication (requested by DSKY)." The crew picks
 * the vehicle (option 2: 1 this, 2 other) and a time (noun 34); the state
 * is integrated to that time and LAT, LONG and ALT are shown (noun 43),
 * with |V| and the flight path angle for noun 73. V32 recycles ten minutes
 * later, integrating from the saved base vector.
 *
 * Vehicle differences: VINTFLAG means "integrate the CSM", so the CM sets
 * it for this vehicle and the LM for the other one; the LM does not keep
 * the base vector (P21FLAG) of its own state while on the surface.
 *
 * Scales: RATT m 2^29, VATT m/cs 2^7, base vectors body scale, times cs
 * 2^28, LAT LONG revolutions, P21ALT m 2^29 (altitude / 100).
 */
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* K.01 2DEC .01 (p.459) */
#define GTD_K01 ((agc_dp_t) 2684355)
/* GOPERF4 option code: BIT2 = "option 2", vehicle */
#define GTD_OPTION_CODE ((uint16_t) 2)

/* PROG21 (p.457): OPTION2 = 1 (this vehicle assumed). */
agc_status_t agc_job_ground_track_determination(agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    agc_status_t st = rz_set_option(2, 1U);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_ground_track_vehicle(ctx);
}

/* GOPERF4 option 2 (vehicle in R2): TERMINATE -> P00, PROCEED -> time,
 * data load ("TC -5") -> the request again.                            */
agc_status_t agc_job_ground_track_vehicle(agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    agc_dsky_cont_t cont;
    cont.terminate = AGC_JOB_GOTOPOOH;
    cont.proceed   = AGC_JOB_GROUND_TRACK_TIME;
    cont.enter     = AGC_JOB_GROUND_TRACK_VEHICLE;
    return agc_dsky_display(ctx, AGC_DSKY_GOPERF4, GTD_OPTION_CODE, &cont);
}

/* P21PROG1 (p.457): V06N34 flashing (time in DSPTEM1). */
agc_status_t agc_job_ground_track_time(agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    return rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 34U, AGC_JOB_GOTOPOOH,
                    AGC_JOB_GROUND_TRACK_COMPUTE, AGC_JOB_GROUND_TRACK_TIME);
}

/* P21VSAVE (p.458): |V| and the flight path angle. */
agc_status_t agc_ground_track_determination_flight_path(
    agc_interp_t *ip, agc_vec_t ratt, agc_vec_t vatt, agc_vec_t vatt1, int x2,
    agc_dp_t *vel, agc_dp_t *gamma)
{
    if (!AGC_ASSERT(ip != NULL && vel != NULL && gamma != NULL) ||
        !AGC_ASSERT(x2 == 0 || x2 == 2))
    {
        return AGC_STATUS_BAD_ARG;
    }
    /* ABVAL SL* 0,2: count 0 - X2, so the moon-scale 2^5 speed is shifted
     * right 2 to 2^7.                                                   */
    *vel       = rz_sl_star(ip, rz_abval(ip, vatt1), 0, x2);
    agc_dp_t c = agc_vec_dot(ip, rz_unit(ip, ratt), vatt); /* U(R).V     */
    *gamma     = agc_dp_asin(ip, agc_dp_div(ip, c, *vel)); /* DDV ASIN   */
    return AGC_STATUS_OK;
}

/* First pass (P21FLAG clear): INTEGRV of the chosen vehicle's permanent
 * state, precision, VINTFLAG = integrate the CSM.                       */
static agc_status_t gtd_first_pass(agc_interp_t *ip, agc_integ_ctx_t *ic,
                                   agc_dp_t tdec1)
{
    bool other = ((rz_option(2) >> 1) != 0U); /* SR1 BHIZ      */
    bool csm   = rz_is_csm() ? !other : other;
    rz_flag_put(AGC_FLAG_VINTFLAG, csm);
    rz_flag_put(AGC_FLAG_DIM0FLAG, false);
    rz_flag_put(AGC_FLAG_INTYPFLG, false); /* PRECISION     */
    return agc_integ_integrv(ip, ic, tdec1);
}

/* P21CONT (p.457-458): INTEGRVS from the saved base vector. */
static agc_status_t gtd_from_base(agc_interp_t *ip, agc_integ_ctx_t *ic,
                                  const agc_rendz_t *rz, agc_dp_t tdec1)
{
    ic->sv.rcv = rz->ground_track_base_r;
    ic->sv.vcv = rz->ground_track_base_v;
    ic->sv.tet = rz->ground_track_time;
    rz_flag_put(AGC_FLAG_DIM0FLAG, false);
    rz_flag_put(AGC_FLAG_MOONFLAG, rz->ground_track_origin != 0);
    return agc_integ_integrvs(ip, ic, tdec1);
}

/* P21VSAVE (p.458): save the base vector, |V|, gamma, P21ORIG; P21FLAG
 * set (LM: not for its own state on the surface).                       */
static void gtd_save_base(agc_interp_t *ip, agc_rendz_t *rz,
                          const agc_integ_out_t *o)
{
    rz->ground_track_time   = o->tat;
    rz->ground_track_base_r = o->ratt1;
    rz->ground_track_base_v = o->vatt1;
    agc_status_t st         = agc_ground_track_determination_flight_path(
                ip, o->ratt, o->vatt, o->vatt1, o->x2, &rz->ground_track_vel,
                &rz->ground_track_gamma);
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    rz->ground_track_origin = (agc_sp_t) o->x2;
    bool this_vehicle       = ((rz_option(2) >> 1) == 0U);
    if (rz_is_csm() || !this_vehicle || !agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        rz_vflag_set(RZ_VF_GROUND_TRACK, true);
    }
}

/* P21DSP (p.458): LAT-LONG of RATT at TAT (LUNAFLAG from X2, ERADFLAG
 * clear), P21ALT = ALT x .01; V06N43 flashing.                          */
static agc_status_t gtd_display(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                                agc_rendz_t *rz, const agc_integ_out_t *o)
{
    agc_latlong_opt_t opt;
    opt.lunar = (o->x2 != 0);
    opt.erad  = false;
    rz_flag_put(AGC_FLAG_LUNAFLAG, opt.lunar);
    rz_flag_put(AGC_FLAG_ERADFLAG, false);
    agc_navaux_padload_t pad;
    agc_status_t         st = agc_navaux_get_padload(&pad);
    agc_latlong_t        ll;
    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_from_r(ip, &pad, opt, o->ratt, o->tat, &ll);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    st = agc_navaux_set_latlong(&ll); /* LAT LONG ALT */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    rz->ground_track_alt = agc_dp_mul(ip, ll.alt, GTD_K01); /* DMP K.01 */
    return rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 43U, AGC_JOB_GOTOPOOH,
                    AGC_JOB_GOTOPOOH, AGC_JOB_GROUND_TRACK_RECYCLE);
}

/* TDEC1 = DSPTEM1; INTSTALL; first pass or P21CONT; P21VSAVE; P21DSP.
 * An INTSTALL sleep resumes this job from its start.                   */
agc_status_t agc_job_ground_track_compute(agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz    = rz_state();
    agc_dp_t     tdec1 = rz_dsptem_dp(RZ_DSPTEM1);
    agc_status_t st    = agc_integ_install(0, AGC_JOB_GROUND_TRACK_COMPUTE);
    if (st != AGC_STATUS_OK)
    {
        return st; /* asleep        */
    }
    agc_integ_ctx_t *ic = agc_integ_shared_ctx();
    if (rz_vflag_test(RZ_VF_GROUND_TRACK))
    { /* BON P21CONT   */
        st = gtd_from_base(&ip, ic, rz, tdec1);
    }
    else
    {
        st = gtd_first_pass(&ip, ic, tdec1);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_integ_out_t o = ic->out;
    gtd_save_base(&ip, rz, &o);
    return gtd_display(ctx, &ip, rz, &o);
}

/* V32 on V06N43 (p.459): DSPTEM1 = P21TIME + 600SEC, P21PROG1. */
agc_status_t agc_job_ground_track_recycle(agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(ctx != NULL);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_dp_t     t  = agc_dp_add(&ip, rz_state()->ground_track_time, RZ_600SEC);
    agc_status_t st = rz_set_dsptem_dp(RZ_DSPTEM1, t);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_ground_track_time(ctx);
}
