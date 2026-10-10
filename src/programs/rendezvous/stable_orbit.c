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
    rz_avflag(this_active);
    rz_tracking_flags_on();                            /* P20FLGON       */
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
    agc_status_t st = rz_set_option(1, 5U);
    if (st == AGC_STATUS_OK) {
        st = rz_set_option(2, 1U);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_job_stable_orbit_option_display(ctx);
}

agc_status_t agc_job_stable_orbit_option_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return so_flashr(ctx, 4U, 6U, SO_BLANK_R3,
                     AGC_JOB_STABLE_ORBIT_OPTION_CHOSEN,
                     AGC_JOB_STABLE_ORBIT_OPTION_DISPLAY);
}

/* After V04N06 (p.527): CM NN = 2. Option 1 (SOI, OPTN1): SET OPTNSW,
 * CLEAR UPDATFLG, PREC/TT, SET UPDATFLG, TINT = TIG + T, CM DSPTEM1 =
 * TINT, V06N57 (DELTAR). Option 2 (SOR): TINTSOI = TINT, CLEAR OPTNSW,
 * JUNCTN1. A PREC/TT sleep resumes this job from its start.           */
agc_status_t agc_job_stable_orbit_option_chosen(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();
    if (rz_is_csm()) {
        rz->nn = SO_NN_OFFSETS;                        /* SSP NN 2       */
    }
    if ((rz_option(2) >> 1) != 0U) {                   /* SR1 BHIZ: SOR  */
        rz->deltar = rz->tpass4;                       /* TINTSOI = TINT */
        rz_flag_put(AGC_FLAG_OPTNSW, false);
        return agc_job_stable_orbit_start(ctx);
    }
    rz_flag_put(AGC_FLAG_OPTNSW, true);          /* OPTN1          */
    rz_flag_put(AGC_FLAG_UPDATFLG, false);
    agc_dp_t t = 0;
    agc_status_t st = so_prec_tt(&ip, rz, AGC_JOB_STABLE_ORBIT_OPTION_CHOSEN,
                                 &t);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_flag_put(AGC_FLAG_UPDATFLG, true);
    rz->tpass4 = agc_dp_add(&ip, t, agc_erase_tig()); /* TI = TIG + TF   */
    if (rz_is_csm()) {
        st = rz_set_dsptem_dp(RZ_DSPTEM1, rz->tpass4);            /* FOR DISPLAY    */
        if (st != AGC_STATUS_OK) {
            return st;
        }
    }
    return agc_job_stable_orbit_deltar(ctx);
}

/* V06N57 (DELTAR) GOFLASHR, R2 R3 blanked. */
agc_status_t agc_job_stable_orbit_deltar(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return so_flashr(ctx, 6U, 57U, SO_BLANK_R2R3,
                     AGC_JOB_STABLE_ORBIT_INTERCEPT_TIME,
                     AGC_JOB_STABLE_ORBIT_DELTAR);
}

/* LM: DSPTEM1 = TINT; V06N34 (time of intercept), then JUNCTN1. */
agc_status_t agc_job_stable_orbit_intercept_time(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    if (!rz_is_csm()) {
        agc_status_t st = rz_set_dsptem_dp(RZ_DSPTEM1, rz_state()->tpass4); /* TINT */
        if (st != AGC_STATUS_OK) {
            return st;
        }
    }
    return rz_vnpooh(ctx, 6U, 34U, AGC_JOB_STABLE_ORBIT_START);
}

/* JUNCTN1 (p.527): CLEAR P39/79SW; SELECTMU (CLEAR FINALFLG, VN1645),
 * which returns to RECYCLE.                                            */
agc_status_t agc_job_stable_orbit_start(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();
    rz_flag_put(AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE, false);
    agc_status_t st = agc_rendz_selectmu(rz, agc_flag_test(AGC_FLAG_CMOONFLG));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    rz_flag_put(AGC_FLAG_FINALFLG, false);
    return rz_ttogo_display(ctx, &ip, rz, AGC_JOB_STABLE_ORBIT_RECYCLE);
}

/* RECYCLE .. JUNCTN2 (pp.527-528): PREC/TT; SOI: passive to TINT
 * (INTRPVP), DELTTIME; SOR: TINT = TINTSOI + T. TARGTIME = TINT -
 * DELTTIME; MAINRTNE. A sleep resumes this job from its start.        */
agc_status_t agc_job_stable_orbit_recycle(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();
    agc_dp_t t = 0;
    agc_status_t st = so_prec_tt(&ip, rz, AGC_JOB_STABLE_ORBIT_RECYCLE, &t);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (agc_flag_test(AGC_FLAG_OPTNSW)) {
        agc_integ_out_t o;
        st = so_intrpvp(&ip, rz->tpass4, AGC_JOB_STABLE_ORBIT_RECYCLE, &o);
        if (st != AGC_STATUS_OK) {
            return st;
        }
        st = agc_rendz_stable_orbit_deltt(&ip, rz, o.ratt, o.vatt);
        (void)AGC_ASSERT(st == AGC_STATUS_OK);
    } else {
        rz->tpass4 = agc_dp_add(&ip, rz->deltar, t);   /* OPTN2: TINTSOI */
    }
    rz->targtime = agc_dp_sub(&ip, rz->tpass4, rz->delttime); /* JUNCTN2 */
    return agc_job_stable_orbit_target(ctx);
}

/* MAINRTNE (pp.528-529): passive to TARGTIME (INTRPVP); INTIME = TIG;
 * S3435.25 with that target; P39: DSPLY81 (UPDATFLG unless final);
 * P38: DELVTPI DELVTPF POSTTPI, V06N58, then DSPLY81.                  */
agc_status_t agc_job_stable_orbit_target(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();
    agc_integ_out_t o;
    agc_status_t st = so_intrpvp(&ip, rz->targtime, AGC_JOB_STABLE_ORBIT_TARGET,
                                 &o);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz->intime = agc_erase_tig();
    st = agc_rendz_intercept_velocity(&ip, rz, o.ratt, o.vatt,
                                      AGC_JOB_STABLE_ORBIT_TARGET);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    bool final = agc_flag_test(AGC_FLAG_FINALFLG);
    if (!final) {
        rz_flag_put(AGC_FLAG_UPDATFLG, true);
    }
    if (agc_flag_test(AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE)) {
        return agc_job_stable_orbit_dv_lv(ctx);        /* P39P79         */
    }
    rz->delvtpi = rz_abval(&ip, rz->delveet3);         /* MAINRTN1       */
    rz->delvtpf = rz_abval(&ip, agc_vec_sub(&ip, rz->vpass4, rz->vtprime));
    agc_rendz_periapo_t pa;
    st = agc_rendz_periapo1(&ip, rz, rz->ract3, rz->viprime, &pa);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz->posttpi = agc_rendz_shiftr1(rz, pa.hperi);
    return rz_vnpooh(ctx, 6U, 58U, AGC_JOB_STABLE_ORBIT_DV_LV);  /* DSPLY58 */
}

/* DSPLY81: V06N81 (DELVLVC). */
agc_status_t agc_job_stable_orbit_dv_lv(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return rz_vnpooh(ctx, 6U, 81U, AGC_JOB_STABLE_ORBIT_DV_ACCEPTED);
}

/* After V06N81: CLEAR XDELVFLG, DELVSIN = DELVEET3, VN1645; then P39/P79B
 * (P39/79SW set) or RECYCLE.                                           */
agc_status_t agc_job_stable_orbit_dv_accepted(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();
    rz_flag_put(AGC_FLAG_XDELVFLG, false);
    rz->delvsin = rz->delveet3;
    agc_job_id_t next = agc_flag_test(AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE)
                        ? AGC_JOB_STABLE_ORBIT_MIDCOURSE_COMPUTE
                        : AGC_JOB_STABLE_ORBIT_RECYCLE;
    return rz_ttogo_display(ctx, &ip, rz, next);
}

/* ======================= P39 / P79 (pp.530-531) ========================== */
/* P39/P79A: KT = TIG increment, P20FLGON, SET P39/79SW, SELECTMU. */
static agc_status_t som_start(agc_job_ctx_t *ctx, bool this_active)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();
    rz_avflag(this_active);
    rz->kt = this_active ? rz->atiginc : rz->ptiginc;
    rz_tracking_flags_on();
    rz_flag_put(AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE, true);
    agc_status_t st = agc_rendz_selectmu(rz, agc_flag_test(AGC_FLAG_CMOONFLG));
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rz_flag_put(AGC_FLAG_FINALFLG, false);
    return rz_ttogo_display(ctx, &ip, rz, AGC_JOB_STABLE_ORBIT_MIDCOURSE_COMPUTE);
}

/* P39 STABLE ORBIT MIDCOURSE (p.530, label P39): AVFLAGA, ATIGINC. */
agc_status_t agc_job_stable_orbit_midcourse(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return som_start(ctx, true);
}

/* P79 STABLE ORBIT MIDCOURSE, OTHER VEHICLE ACTIVE (label P79). */
agc_status_t agc_job_stable_orbit_midcourse_other_vehicle(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return som_start(ctx, false);
}

/* P39/P79B (p.530): TIG = now + KT. */
agc_status_t agc_job_stable_orbit_midcourse_compute(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();
    agc_erase_set_tig(agc_dp_add(&ip, rz_loadtime(), rz->kt));
    return agc_job_stable_orbit_midcourse_advance(ctx);
}

/* P39/P79B continued: PRECSET to TIG, S34/35.1, MAINRTNE with TARGTIME
 * (saved by P38). Resumed here after an INTSTALL sleep.                */
agc_status_t agc_job_stable_orbit_midcourse_advance(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();
    agc_status_t st = rz_precset(&ip, rz, agc_erase_tig(),
                                 AGC_JOB_STABLE_ORBIT_MIDCOURSE_ADVANCE);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    st = agc_rendz_los_and_normal(&ip, rz);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return agc_job_stable_orbit_target(ctx);
}
