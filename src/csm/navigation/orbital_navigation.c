/*
 * orbital_navigation.c -- P22 ORBITAL NAVIGATION PROGRAM, CSM
 * (P20-P25.agc, Comanche055 pp.565-569): PROG22, PROG22A/B/C, CALLR52,
 * DOV5N71, P22OVER, the landmark code check (S22N7071, S22DSPPA, S22DSPP,
 * S22ABDE, DE-GR-50) and the landmark coordinates (S22LSITE, OBTAINLL,
 * P22SUBRB, N89ERRX). The mark processing S22.1 is in
 * csm/navigation/landmark_navigation.c.
 *
 * The crew identifies the landmark on noun 70 / 71 as ABCDE (octal): A =
 * 1 known, 2 unknown; B = the offset point mark (0-5); DE = 00 coordinates
 * from noun 89, 01 the landing site RLS, 50-57 advanced orbit (R52
 * ADVORB). The orbit's +MGA is shown first (noun 45).
 *
 * Scales: angles revolutions; RLS moon-fixed m 2^27; ALPHAV m 2^29.
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/imu/imu.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "../../programs/navigation/internal.h"

#define ON_PRIO7       07000u                /* B of ABCDE (offset)        */
#define ON_PRIO5       05000u                /* offset 5                   */
#define ON_A_MASK      070000u               /* 13,14,15: A of ABCDE       */
#define ON_A_KNOWN     010000u
#define ON_A_UNKNOWN   020000u
#define ON_DE_MASK     000077u               /* CS HIGH9                   */
#define ON_D_MASK      000070u               /* BIT5 + OCT50               */
#define ON_D_ADV_ORBIT 000050u               /* DE = 50-57                 */
#define ON_BLANK_R1R2  3U
#define ON_BLANK_R1R3  5U

/* P22 working cells: 8KK (the VN of S22N7071), 8KK +1 (noun 70), the
 * S22TOFF return of S22N7071 / P22SUBRB.                                 */
typedef struct on_state {
    uint8_t      noun;           /* 70 or 71                              */
    agc_job_id_t ret;            /* where the check returns               */
} on_state_t;

static on_state_t on = { 70U, AGC_JOB_GOTOPOOH };

/* PROG22: rendezvous flag off; IMU check; +MGA = ACOS |UNIT(V x R) . Y
 * (REFSMMAT)| of the CSM now (orbit plane to stable member); W invalid;
 * V06N45.                                                                */
agc_status_t agc_job_orbital_navigation_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_flag_put(AGC_FLAG_RNDVZFLG, false);
    agc_status_t st = agc_imu_status_check();                 /* R02BOTH  */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_integ_out_t o;
    st = rz_integ(&ip, RZ_CSMCONIC, rz_loadtime(), AGC_JOB_ORBITAL_NAVIGATION, &o);
    if (st != AGC_STATUS_OK) {
        return st;                        /* BUSY: asleep in INTSTALL  */
    }
    agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_vec_t n = agc_vec_unit(&ip, agc_vec_cross(&ip, o.vatt, o.ratt), NULL, NULL);
    agc_dp_t c = agc_dp_abs(agc_vec_dot(&ip, n, refsmmat.row[1]));
    rz_state()->plus_mga = agc_dp_acos(&ip, agc_dp_shl(&ip, c, 1));
    rz_flag_put(AGC_FLAG_RENDWFLG, false);
    return agc_job_orbital_navigation_orbit_display(ctx);
}

/* V06N45 (GOFLASHR, R1 R2 blanked): PROCEED: PROG22A.                    */
agc_status_t agc_job_orbital_navigation_orbit_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = rz_flash(ctx, AGC_DSKY_GOFLASHR, 6U, 45U, AGC_JOB_GOTOPOOH,
                               AGC_JOB_ORBITAL_NAVIGATION_LANDMARK,
                               AGC_JOB_ORBITAL_NAVIGATION_ORBIT_DISPLAY);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_blanket(ON_BLANK_R1R2);
}

/* PROG22A (point A): offset digit cleared; ERADFLAG; in the moon's sphere
 * the landmark code on noun 70; in earth orbit a known landmark with its
 * coordinates on noun 89; then CALLR52.                                  */
agc_status_t agc_job_orbital_navigation_landmark(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_sp_t lm = agc_erase_landmark();
    agc_erase_set_landmark((agc_sp_t)rz_word_value((uint16_t)(rz_word_of(lm) & ~ON_PRIO7)));
    rz_flag_put(AGC_FLAG_ORBITAL_NAVIGATION_MARK_TAKEN, false);  /* P22MKFLG */
    rz_flag_put(AGC_FLAG_ERADFLAG, true);
    on.ret = AGC_JOB_ORBITAL_NAVIGATION_OPTICS;
    if (agc_flag_test(AGC_FLAG_CMOONFLG)) {
        rz_flag_put(AGC_FLAG_LUNAFLAG, true);
        on.noun = 70U;                                        /* S22N7071 */
        return agc_job_orbital_navigation_code(ctx);
    }
    rz_flag_put(AGC_FLAG_LUNAFLAG, false);                    /* PROG22B  */
    rz_flag_put(AGC_FLAG_KNOWNFLG, true);
    return agc_job_orbital_navigation_landing_site(ctx);      /* P22SUBRB */
}

/* S22DSPPA: V05N70 or V05N71 (GOFLASHR, R1 R3 blanked); PROCEED checks
 * the landmark code.                                                      */
agc_status_t agc_job_orbital_navigation_code(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = rz_flash(ctx, AGC_DSKY_GOFLASHR, 5U, on.noun, AGC_JOB_GOTOPOOH,
                               AGC_JOB_ORBITAL_NAVIGATION_CODE_CHECK,
                               AGC_JOB_ORBITAL_NAVIGATION_CODE);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_blanket(ON_BLANK_R1R3);
}

/* S22DSPP: operator error light and the code display again.             */
static agc_status_t on_code_error(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_dsky_falton();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_job_orbital_navigation_code(ctx);
}

/* S22LSITE: the landing site RLS (moon fixed) in reference axes now
 * (RP-TO-R, VSR2 to B-29), and its LAT LONG ALT.                         */
static agc_status_t on_site_from_rls(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_navaux_padload_t pad;
    agc_status_t st = agc_navaux_get_padload(&pad);
    agc_dp_t t = rz_loadtime();
    agc_vec_t r = agc_vec_zero();
    if (st == AGC_STATUS_OK) {
        st = agc_planet_rp_to_r(&ip, &pad, true, pad.rls, t, &r);
    }
    agc_latlong_opt_t opt = { agc_flag_test(AGC_FLAG_LUNAFLAG),
                              agc_flag_test(AGC_FLAG_ERADFLAG) };
    agc_latlong_t ll;
    if (st == AGC_STATUS_OK) {
        st = agc_latlong_from_r(&ip, &pad, opt, agc_vec_shr(r, 2), t, &ll);
    }
    return (st == AGC_STATUS_OK) ? agc_navaux_set_latlong(&ll) : st;
}

/* S22ABDE: unknown landmark: done; known: DE = 00 coordinates from noun
 * 89 (OBTAINLL), 01 from the landing site (S22LSITE).                    */
static agc_status_t on_abde(agc_job_ctx_t *ctx, uint16_t de)
{
    if (!agc_flag_test(AGC_FLAG_KNOWNFLG)) {
        return rz_continue(on.ret);
    }
    if (de == 0U) {
        return agc_job_orbital_navigation_landing_site(ctx);  /* OBTAINLL */
    }
    agc_status_t st = on_site_from_rls();
    return (st != AGC_STATUS_OK) ? st : rz_continue(on.ret);
}

/* PROCEED on the code (p.568): B (offset) 0-5; A = 1 known (DE 00 or 01,
 * on noun 70 also 50-57: advanced orbit), A = 2 unknown; anything else
 * is an operator error.                                                   */
agc_status_t agc_job_orbital_navigation_code_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    uint16_t lm = rz_word_of(agc_erase_landmark());
    uint16_t cxoff = (uint16_t)(lm & ON_PRIO7);
    rz_landmark_set_offset((agc_sp_t)cxoff);
    if (cxoff > ON_PRIO5) {
        return on_code_error(ctx);
    }
    rz_flag_put(AGC_FLAG_KNOWNFLG, false);
    uint16_t a = (uint16_t)(lm & ON_A_MASK);
    if (a == ON_A_UNKNOWN) {
        return on_abde(ctx, 0U);
    }
    if (a != ON_A_KNOWN) {
        return on_code_error(ctx);                 /* A = 0, 3 .. 7    */
    }
    rz_flag_put(AGC_FLAG_KNOWNFLG, true);
    uint16_t de = (uint16_t)(lm & ON_DE_MASK);                /* 22SUBSCL */
    if (de <= 1U) {
        return on_abde(ctx, de);
    }
    if (on.noun != 70U || (de & ON_D_MASK) != ON_D_ADV_ORBIT) {
        return on_code_error(ctx);
    }
    return agc_job_orbital_navigation_advanced_optics(ctx);   /* DE-GR-50 */
}

/* P22SUBRB: V06N89 for the landmark coordinates (LAT, LONG/2, ALT).     */
agc_status_t agc_job_orbital_navigation_landing_site(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 89U, AGC_JOB_GOTOPOOH,
                    AGC_JOB_ORBITAL_NAVIGATION_SITE_CHECK,
                    AGC_JOB_ORBITAL_NAVIGATION_LANDING_SITE);
}

/* PROCEED on V06N89: LAT or LONG/2 beyond 90 degrees is an operator
 * error (N89ERRX); else LLASRDA and back.                                */
agc_status_t agc_job_orbital_navigation_site_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_align_ram_t *a = agc_align_ram();
    if (agc_dp_abs(a->landlat) > AGC_DP_QUARTER
        || agc_dp_abs(a->landlong) > AGC_DP_QUARTER) {
        agc_status_t st = agc_dsky_falton();
        return (st != AGC_STATUS_OK) ? st : agc_job_orbital_navigation_landing_site(ctx);
    }
    agc_status_t st = agc_rendz_llasrda();
    return (st != AGC_STATUS_OK) ? st : rz_continue(on.ret);
}

/* CALLR52 / DE-GR-50: restart at priority 13; five marks wanted; the
 * landmark as target; R52 (or ADVORB), then DOV5N71.                      */
static agc_status_t on_optics(const agc_job_ctx_t *ctx, bool advanced)
{
    rz_phase_job(04U, 05022U, advanced ? AGC_JOB_ORBITAL_NAVIGATION_ADVANCED_OPTICS
                                       : AGC_JOB_ORBITAL_NAVIGATION_OPTICS, 013);
    agc_align_ram()->markindx = 5;
    rz_flag_put(AGC_FLAG_TARG2FLG, true);
    rz_flag_put(AGC_FLAG_TARG1FLG, false);
    if (advanced) {
        return agc_csm_auto_optics_advanced_orbit(ctx, AGC_JOB_ORBITAL_NAVIGATION_MARKED);
    }
    return agc_csm_auto_optics_positioning(ctx, AGC_JOB_ORBITAL_NAVIGATION_MARKED);
}

agc_status_t agc_job_orbital_navigation_optics(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return on_optics(ctx, false);
}

agc_status_t agc_job_orbital_navigation_advanced_optics(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return on_optics(ctx, true);
}

/* DOV5N71: the landmark code again on noun 71, then PROG22C.            */
agc_status_t agc_job_orbital_navigation_marked(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    on.noun = 71U;
    on.ret = AGC_JOB_ORBITAL_NAVIGATION_PROCESS;
    return agc_job_orbital_navigation_code(ctx);
}

/* PROG22C: 8NN = the marks taken; S22.1, then P22OVER.                  */
agc_status_t agc_job_orbital_navigation_process(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return rz_landmark_marks_start(agc_sxtmark_marks_made(), AGC_JOB_ORBITAL_NAVIGATION_OVER);
}

/* P22OVER: back to point A.                                              */
agc_status_t agc_job_orbital_navigation_over(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rz_phase(2, 04022);
    return agc_job_orbital_navigation_landmark(ctx);
}

/* P22SUBRB for other programs (P23): noun 89, the caller continues as
 * ret after LLASRDA.                                                      */
agc_status_t rz_landing_site_display(agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    on.ret = ret;
    return agc_job_orbital_navigation_landing_site(ctx);
}
