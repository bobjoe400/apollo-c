/*
 * stable_orbit.c -- STABLE ORBIT RENDEZVOUS (P38/P78) and STABLE ORBIT
 * MIDCOURSE (P39/P79), both vehicles (STABLE_ORBIT.agc, Comanche055
 * pp.525-532, Luminary099 the same text).
 *
 * "P38 and P78 calculate the required delta V and other initial
 * conditions required by the AGC to (1) put the active vehicle on a
 * transfer trajectory that intercepts the passive vehicle orbit a given
 * distance, delta R, either ahead of or behind the passive vehicle and
 * (2) actually place the active vehicle in the passive vehicle orbit with
 * a delta R separation between the two vehicles." Option 1 is the SOI
 * maneuver, option 2 the SOR maneuver. "P39 and P79 calculate the
 * required delta V ... to make a midcourse correction maneuver after
 * completing the SOI maneuver but before making the SOR maneuver."
 *
 * Vehicle differences: the LM sets NN = 2 at entry (the CM after the
 * option display) and puts TINT into DSPTEM1 after the delta R display
 * (the CM before it).
 *
 * Aliases kept from ERASABLE_ASSIGNMENTS: TINT EQUALS TPASS4 and TINTSOI
 * EQUALS DELTAR (the SOR phase overwrites DELTAR with the SOI intercept
 * time). Scales: r m 2^29, v m/cs 2^7, t cs 2^28, CENTANG revolutions.
 */
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "internal.h"

/* NN = 2 offsets (SSP NN 2 / DECTWO). */
#define SO_NN_OFFSETS  ((agc_sp_t)2)
/* BLANKET bits after a GOFLASHR: R1 R2 = 3, R3 = BIT3, R2 R3 = 6.       */
#define SO_BLANK_R1R2  ((uint16_t)3)
#define SO_BLANK_R3    ((uint16_t)4)
#define SO_BLANK_R2R3  ((uint16_t)6)

/* GOFLASHR (immediate return: BLANKET, ENDOFJOB). */
static agc_status_t so_flashr(const agc_job_ctx_t *ctx, uint8_t verb,
                              uint8_t noun, uint16_t blank,
                              agc_job_id_t proceed, agc_job_id_t recycle)
{
    agc_status_t st = rz_flash(ctx, AGC_DSKY_GOFLASHR, verb, noun,
                               AGC_JOB_GOTOPOOH, proceed, recycle);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    st = agc_dsky_blanket(blank);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;                                       /* TCF ENDOFJOB     */
}

/* PREC/TT (p.531): PRECSET to TIG; passive transfer time T for CENTANG
 * (TIMETHET); S34/35.1 (unit normal).                                  */
static agc_status_t so_prec_tt(agc_interp_t *ip, agc_rendz_t *rz,
                               agc_job_id_t resume, agc_dp_t *t)
{
    agc_status_t st = rz_precset(ip, rz, agc_erase_tig(), resume);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_conic_time_out_t o;
    st = rz_timethet_rv(ip, rz, rz->rpass3, rz->vpass3,
                        agc_dp_sin(ip, rz->centang),
                        agc_dp_cos(ip, rz->centang), true, &o);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    *t = o.t;
    return agc_rendz_los_and_normal(ip, rz);
}

/* INTRPVP (p.531): precision update of the passive vehicle to tdec1
 * (AVFLAG clear = CSM active: LEMPREC; set: CSMPREC).                  */
static agc_status_t so_intrpvp(agc_interp_t *ip, agc_dp_t tdec1,
                               agc_job_id_t resume, agc_integ_out_t *o)
{
    rz_integ_kind_t k = agc_flag_test(AGC_FLAG_AVFLAG) ? RZ_CSMPREC
                                                       : RZ_LEMPREC;
    return rz_integ(ip, k, tdec1, resume, o);
}

/* ---- DELTTIME (p.528) ------------------------------------------------- */
agc_status_t agc_rendz_stable_orbit_deltt(agc_interp_t *ip, agc_rendz_t *rz,
                                          agc_vec_t ratt, agc_vec_t vatt)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t ur = rz_unit(ip, ratt);                  /* RP/(RP) 0D     */
    agc_dp_t h = rz_abval(ip, agc_vec_cross(ip, vatt, ur));
    int x1 = 0;
    agc_dp_t hn = rz_norm(h, &x1);                     /* NORM X1        */
    agc_dp_t d = agc_dp_div(ip, rz->deltar, hn);       /* DDV            */
    rz->delttime = rz_sl_star(ip, d, -7, x1);          /* SL* 0 -7,1     */
    (void)AGC_ASSERT(hn >= 0);
    return AGC_STATUS_OK;
}

/* ======================= P38 / P78 (pp.526-529) ========================== */
static agc_status_t so_start(agc_job_ctx_t *ctx, bool this_active)
{
    agc_status_t st = rz_avflag(this_active);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = rz_tracking_flags_on();                       /* P20FLGON       */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    if (!rz_is_csm()) {
        rz_state()->nn = SO_NN_OFFSETS;                /* CAF DECTWO     */
    }
    return rz_vnpooh(ctx, 6U, 33U, AGC_JOB_STABLE_ORBIT_CENTANG); /* TIG  */
}

/* P38 STABLE ORBIT RENDEZVOUS (p.526, label P38): AVFLAGA. */
agc_status_t agc_job_stable_orbit_rendezvous(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return so_start(ctx, true);
}

/* P78 STABLE ORBIT RENDEZVOUS, OTHER VEHICLE ACTIVE (label P78). */
agc_status_t agc_job_stable_orbit_rendezvous_other_vehicle(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return so_start(ctx, false);
}

/* After V06N33 (TIG): the same display again (VNDSPLY RECYCLE). */
agc_status_t agc_job_stable_orbit_tig(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return rz_vnpooh(ctx, 6U, 33U, AGC_JOB_STABLE_ORBIT_CENTANG);
}

/* V06N55 (CENTANG) GOFLASHR, R1 R2 blanked. */
agc_status_t agc_job_stable_orbit_centang(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return so_flashr(ctx, 6U, 55U, SO_BLANK_R1R2, AGC_JOB_STABLE_ORBIT_OPTION,
                     AGC_JOB_STABLE_ORBIT_CENTANG);
}

/* OPTION1 = 5, OPTION2 = 1 (SOI); V04N06 GOFLASHR, R3 blanked. */
agc_status_t agc_job_stable_orbit_option(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    rz_set_option(1, 5U);
    rz_set_option(2, 1U);
    return agc_job_stable_orbit_option_display(ctx);
}

agc_status_t agc_job_stable_orbit_option_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return so_flashr(ctx, 4U, 6U, SO_BLANK_R3,
                     AGC_JOB_STABLE_ORBIT_OPTION_CHOSEN,
                     AGC_JOB_STABLE_ORBIT_OPTION_DISPLAY);
}
