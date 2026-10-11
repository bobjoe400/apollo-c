/*
 * nav_integ.c -- nav-integ against the real AGC integration (yaAGC oracle).
 *
 * Inputs and expected words: tools/oracle/cases/nav_integ.spec / .txt
 * (Comanche055). INTEGRVS from a fixed RCV/VCV/TET to TDEC1: one Nystrom
 * step (TIMESTEP, three ACCOMP stages with GAMCOMP and OBLATE, FBR3 /
 * KEPPREP / KEPLER) and multi-step runs, earth (J2..J4) and moon (J2, J3,
 * QUALITY1-3 with the E3J22R2M / E32C31RM pad loads); then RECTIFY alone.
 * ACCOMP is not a separate entry in C (static accomp()), so it is checked
 * through FV, the disturbing acceleration of the last stage.
 *
 * Tolerances (DP LSB, 2^-28 of the scale): see docs/modules/oracle-cases.md.
 */
#include <stdio.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "oracle_txt.h"

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);

    return (false);
}

#define TOL_RECT 0L /* RECTIFY: shifts and adds, bit-exact            */

typedef struct integ_case
{
    const char *tag;
    bool        moon;
    long        tol_r;  /* position words (RATT, RRECT, RATT1)          */
    long        tol_v;  /* velocity words                               */
    long        tol_fv; /* FV                                           */
} integ_case_t;

/* One step: within a few LSB (FV: GAMCOMP/OBLATE rounding). Several
 * steps: KEPLER's VCV differs in its low bits from the second step on
 * (finding F1 in docs/modules/oracle-cases.md) and the Encke state drifts:
 * velocity up to ~6000 LSB of 2^7 m/cs (3e-3 m/cs) after 1000 s.        */
static const integ_case_t CASES[] = {
    {"integ_earth_1step", false, 2, 2, 8},
    {"integ_earth2_1step", false, 2, 2, 8},
    {"integ_earth_multi", false, 64, 8192, 1024},
    {"integ_moon_1step", true, 2, 2, 8},
    {"integ_moon_multi", true, 64, 4096, 1024},
    /* TDEC1 - TET > 2^20 cs (DT/2COMP overflow, SIGNMPAC): TET, TAT and
     * MU(P) exact.  Against this yaAGC the state drifts ~12 km in 3 h
     * (F1); against the DV-patched probe (gen/dvcheck.sh) 1 LSB / 33 LSB
     * at 3 h; 2.5 km / 3 m/s at 8.8 h from KEPPREP's 1-LSB XKEPNEW
     * difference fixed in at a rectification (review-nav-integ N4).     */
    {"integ_earth_long", false, 8192, 1L << 19, 1L << 22},
};

typedef struct step_case
{
    const char *tag;
    bool        moon;
    long        tol_dv;  /* TDELTAV, TNUV                                */
    long        tol_cv;  /* RCV, VCV (KEPLER)                            */
    long        tol_x;   /* XKEP                                         */
} step_case_t;

static const step_case_t STEPS[] = {
    {"step_earth_max", false, 4, 0, 2},
    {"step_earth_2max", false, 4, 1024, 8},
    {"step_moon", true, 4, 0, 2},
};

static void fresh(void)
{
    agc_navaux_padload_t pad = {0};

    agc_state_init(AGC_VEHICLE_CSM);
    (void) agc_exec_init();
    (void) agc_waitlist_init();
    agc_erase_set_modreg(0);
    (void) agc_navaux_set_padload(&pad);
    agc_erase_set_tephem(0);
}

static void run_integ(const integ_case_t *k)
{
    agc_interp_t    ip;
    agc_integ_ctx_t ctx;
    const char     *t = k->tag;

    fresh();
    agc_interp_init(&ip);
    (void) agc_integ_ctx_init(&ctx);
    (void) agc_integ_install(0, AGC_JOB_NONE);
    (void) agc_integ_set_lunar_coeffs((agc_sp_t) oracle_in(t, "E3J22R2M", 0),
                                      (agc_sp_t) oracle_in(t, "E32C31RM", 0));
    ctx.sv.rcv = oracle_in_vec(t, "RCV");
    ctx.sv.vcv = oracle_in_vec(t, "VCV");
    ctx.sv.tet = oracle_in_dp(t, "TET", 0);
    agc_flag_put(AGC_FLAG_MOONFLAG, k->moon);
    agc_flag_put(AGC_FLAG_INTYPFLG, false);
    const agc_status_t st =
        agc_integ_integrvs(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0));

    oracle_require_ok(t, "INTWAKE");
    (void) oracle_cmp(t, "status", (long) st, (long) AGC_STATUS_OK, 0);
    (void) oracle_cmp(t, "out.valid", (long) ctx.out.valid, 1L, 0);
    oracle_cmp_vec(t, "RATT", ctx.out.ratt, oracle_out_vec(t, "VAC1"),
                   k->tol_r);
    oracle_cmp_vec(t, "VATT", ctx.out.vatt, oracle_out_vec(t, "VAC1+6"),
                   k->tol_v);
    (void) oracle_cmp(t, "TAT", ctx.out.tat, oracle_out_dp(t, "VAC1+12"), 0);
    oracle_cmp_vec(t, "RATT1", ctx.out.ratt1, oracle_out_vec(t, "VAC1+14"),
                   k->tol_r);
    oracle_cmp_vec(t, "VATT1", ctx.out.vatt1, oracle_out_vec(t, "VAC1+20"),
                   k->tol_v);
    (void) oracle_cmp(t, "MU(P)", ctx.out.mu_p, oracle_out_dp(t, "VAC1+26"),
                      0);
    oracle_cmp_vec(t, "RRECT", ctx.sv.rrect, oracle_out_vec(t, "RRECT"),
                   k->tol_r);
    oracle_cmp_vec(t, "VRECT", ctx.sv.vrect, oracle_out_vec(t, "VRECT"),
                   k->tol_v);
    (void) oracle_cmp(t, "TET", ctx.sv.tet, oracle_out_dp(t, "TET"), 0);
    oracle_cmp_vec(t, "FV", ctx.fv, oracle_out_vec(t, "FV"), k->tol_fv);
    (void) oracle_cmp(t, "MOONFLAG", agc_flag_test(AGC_FLAG_MOONFLAG),
                      (oracle_out_sp(t, "FLAGWRD0") & 04000) != 0, 0);
    (void) oracle_cmp(t, "MIDFLAG", agc_flag_test(AGC_FLAG_MIDFLAG),
                      (oracle_out_sp(t, "FLAGWRD0") & 010000) != 0, 0);
}

/* INTEGRVS with STATEFLG + VINTFLAG: the AGC stops at PHASCHNG in ANDOUT,
 * before RECTOUT; C: the permanent CSM state ATOPCSM copied there.     */
static void run_step(const step_case_t *k)
{
    agc_interp_t    ip;
    agc_integ_ctx_t ctx;
    agc_integ_sv_t  sv;
    const char     *t = k->tag;

    fresh();
    agc_interp_init(&ip);
    (void) agc_integ_ctx_init(&ctx);
    (void) agc_integ_install(0, AGC_JOB_NONE);
    (void) agc_integ_set_lunar_coeffs((agc_sp_t) oracle_in(t, "E3J22R2M", 0),
                                      (agc_sp_t) oracle_in(t, "E32C31RM", 0));
    ctx.sv.rcv = oracle_in_vec(t, "RCV");
    ctx.sv.vcv = oracle_in_vec(t, "VCV");
    ctx.sv.tet = oracle_in_dp(t, "TET", 0);
    agc_flag_put(AGC_FLAG_MOONFLAG, k->moon);
    agc_flag_put(AGC_FLAG_STATEFLG, true);
    agc_flag_put(AGC_FLAG_VINTFLAG, true);
    (void) agc_integ_integrvs(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0));
    (void) agc_integ_perm_get(AGC_VEHICLE_CSM, &sv);
    oracle_require_ok(t, "PHASCHNG");
    oracle_cmp_vec(t, "TDELTAV", sv.deltav, oracle_out_vec(t, "TDELTAV"),
                   k->tol_dv);
    oracle_cmp_vec(t, "TNUV", sv.nuv, oracle_out_vec(t, "TNUV"), k->tol_dv);
    oracle_cmp_vec(t, "RCV", sv.rcv, oracle_out_vec(t, "RCV"), k->tol_cv);
    oracle_cmp_vec(t, "VCV", sv.vcv, oracle_out_vec(t, "VCV"), k->tol_cv);
    (void) oracle_cmp(t, "TC", sv.tc, oracle_out_dp(t, "TC"), 0);
    (void) oracle_cmp(t, "XKEP", sv.xkep, oracle_out_dp(t, "XKEP"), k->tol_x);
    (void) oracle_cmp(t, "TET", sv.tet, oracle_out_dp(t, "TET"), 0);
    oracle_cmp_vec(t, "FV", ctx.fv, oracle_out_vec(t, "FV"), 8);
}

static void run_rectify(const char *t)
{
    agc_interp_t    ip;
    agc_integ_ctx_t ctx;

    fresh();
    agc_interp_init(&ip);
    (void) agc_integ_ctx_init(&ctx);
    ctx.pbody     = oracle_in(t, "PBODY", 0);
    ctx.sv.rcv    = oracle_in_vec(t, "RCV");
    ctx.sv.vcv    = oracle_in_vec(t, "VCV");
    ctx.sv.deltav = oracle_in_vec(t, "TDELTAV");
    ctx.sv.nuv    = oracle_in_vec(t, "TNUV");
    ctx.sv.tc     = oracle_in_dp(t, "TC", 0);
    ctx.sv.xkep   = oracle_in_dp(t, "XKEP", 0);
    (void) agc_integ_rectify(&ip, &ctx);
    oracle_require_ok(t, "POODOO");
    oracle_cmp_vec(t, "RCV", ctx.sv.rcv, oracle_out_vec(t, "RCV"), TOL_RECT);
    oracle_cmp_vec(t, "VCV", ctx.sv.vcv, oracle_out_vec(t, "VCV"), TOL_RECT);
    oracle_cmp_vec(t, "TDELTAV", ctx.sv.deltav, oracle_out_vec(t, "TDELTAV"),
                   0);
    oracle_cmp_vec(t, "TNUV", ctx.sv.nuv, oracle_out_vec(t, "TNUV"), 0);
    (void) oracle_cmp(t, "TC", ctx.sv.tc, oracle_out_dp(t, "TC"), 0);
    (void) oracle_cmp(t, "XKEP", ctx.sv.xkep, oracle_out_dp(t, "XKEP"), 0);
}

/* Origin change with STATEFLG + VINTFLAG (zero ephemeris: the moon at the
 * origin).  origchng_timestep: switched in TIMESTEP, SVDWN1 sees X2 = 2.
 * origchng_apchk: switched in A-PCHK, SVDWN1 sees X2 = ORIGEX and stores
 * RN = VN = 0.  RN / VN are preset to a pattern so a missing store shows. */
static void run_origchng(const char *t)
{
    agc_interp_t    ip;
    agc_integ_ctx_t ctx;
    agc_vec_t       pattern = agc_vec_zero();

    fresh();
    agc_interp_init(&ip);
    (void) agc_integ_ctx_init(&ctx);
    (void) agc_integ_install(0, AGC_JOB_NONE);
    pattern = oracle_in_vec(t, "RN");
    agc_erase_set_rn(pattern);
    agc_erase_set_vn(pattern);
    ctx.sv.rcv = oracle_in_vec(t, "RCV");
    ctx.sv.vcv = oracle_in_vec(t, "VCV");
    ctx.sv.tet = oracle_in_dp(t, "TET", 0);
    agc_flag_put(AGC_FLAG_STATEFLG, true);
    agc_flag_put(AGC_FLAG_VINTFLAG, true);
    const agc_status_t st =
        agc_integ_integrvs(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0));

    oracle_require_ok(t, "INTWAKE");
    (void) oracle_cmp(t, "status", (long) st, (long) AGC_STATUS_OK, 0);
    oracle_cmp_vec(t, "RN", agc_erase_rn(), oracle_out_vec(t, "RN"), 0);
    oracle_cmp_vec(t, "VN", agc_erase_vn(), oracle_out_vec(t, "VN"), 0);
    (void) oracle_cmp(t, "PIPTIME", agc_erase_piptime(),
                      oracle_out_dp(t, "PIPTIME"), 0);
    (void) oracle_cmp(t, "PBODY", ctx.pbody, oracle_out_sp(t, "PBODY"), 0);
    (void) oracle_cmp(t, "MOONFLAG", agc_flag_test(AGC_FLAG_MOONFLAG),
                      (oracle_out_sp(t, "FLAGWRD0") & 04000) != 0, 0);
    (void) oracle_cmp(t, "STATEFLG", agc_flag_test(AGC_FLAG_STATEFLG),
                      (oracle_out_sp(t, "FLAGWRD3") & 00020) != 0, 0);
    oracle_cmp_vec(t, "RRECT", ctx.sv.rrect, oracle_out_vec(t, "RRECT"), 2);
    oracle_cmp_vec(t, "VRECT", ctx.sv.vrect, oracle_out_vec(t, "VRECT"), 2);
    (void) oracle_cmp(t, "TET", ctx.sv.tet, oracle_out_dp(t, "TET"), 0);
}

/* Third bodies (MIDFLAG): ephemeris pad loads as nav_aux.spec, LSPOS in
 * every ACCOMP stage, CHKSWTCH at each TIMESTEP and in A-PCHK.          */
static void run_third(const char *t, bool moon, long tol_r, long tol_v,
                      long tol_fv)
{
    agc_interp_t    ip;
    agc_integ_ctx_t ctx;
    agc_ephem_csm_t e;

    fresh();
    e.timemo = oracle_in_tp(t, "TIMEMO", 0);

    for (int k = 0; k < AGC_EPHEM_NCOEF; k++)
    {
        e.vecoem[k] = oracle_in_vec_at(t, "VECOEM", k);
    }

    e.reso    = oracle_in_vec(t, "RESO");
    e.veso    = oracle_in_vec(t, "VESO");
    e.omegaes = oracle_in_dp(t, "OMEGAES", 0);
    (void) agc_ephem_set_csm_data(&e);
    agc_erase_set_tephem(oracle_in_tp(t, "TEPHEM", 0));
    agc_interp_init(&ip);
    (void) agc_integ_ctx_init(&ctx);
    (void) agc_integ_install(0, AGC_JOB_NONE);
    (void) agc_integ_set_lunar_coeffs((agc_sp_t) oracle_in(t, "E3J22R2M", 0),
                                      (agc_sp_t) oracle_in(t, "E32C31RM", 0));
    ctx.sv.rcv = oracle_in_vec(t, "RCV");
    ctx.sv.vcv = oracle_in_vec(t, "VCV");
    ctx.sv.tet = oracle_in_dp(t, "TET", 0);
    agc_flag_put(AGC_FLAG_MOONFLAG, moon);
    const agc_status_t st =
        agc_integ_integrvs(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0));

    oracle_require_ok(t, "INTWAKE");
    (void) oracle_cmp(t, "status", (long) st, (long) AGC_STATUS_OK, 0);
    oracle_cmp_vec(t, "RATT", ctx.out.ratt, oracle_out_vec(t, "VAC1"), tol_r);
    oracle_cmp_vec(t, "VATT", ctx.out.vatt, oracle_out_vec(t, "VAC1+6"),
                   tol_v);
    (void) oracle_cmp(t, "TAT", ctx.out.tat, oracle_out_dp(t, "VAC1+12"), 0);
    (void) oracle_cmp(t, "MU(P)", ctx.out.mu_p, oracle_out_dp(t, "VAC1+26"),
                      0);
    (void) oracle_cmp(t, "TET", ctx.sv.tet, oracle_out_dp(t, "TET"), 0);
    oracle_cmp_vec(t, "FV", ctx.fv, oracle_out_vec(t, "FV"), tol_fv);
    (void) oracle_cmp(t, "PBODY", ctx.pbody, oracle_out_sp(t, "PBODY"), 0);
    (void) oracle_cmp(t, "MIDFLAG", agc_flag_test(AGC_FLAG_MIDFLAG),
                      (oracle_out_sp(t, "FLAGWRD0") & 010000) != 0, 0);
}

/* W matrix: INTEGRV of the permanent CSM state with DIM0FLAG (6x6), the
 * W columns integrated with DOW.. after every state step (RELOADSV).    */
static void run_wmatrix(const char *t, long tol_w)
{
    agc_interp_t    ip;
    agc_integ_ctx_t ctx;
    agc_integ_sv_t  sv;
    char            name[16];

    fresh();
    agc_interp_init(&ip);
    (void) agc_integ_ctx_init(&ctx);
    agc_erase_set_modreg((agc_sp_t) oracle_in(t, "MODREG", 0));
    sv        = ctx.sv;
    sv.rrect  = oracle_in_vec(t, "RRECTCSM");
    sv.vrect  = oracle_in_vec(t, "RRECTCSM+6");
    sv.rcv    = oracle_in_vec(t, "RRECTCSM+26");
    sv.vcv    = oracle_in_vec(t, "RRECTCSM+32");
    (void) agc_integ_perm_set(AGC_VEHICLE_CSM, &sv);
    ctx.w.blk[0][0] = oracle_in_vec(t, "W");
    ctx.w.blk[1][3] = oracle_in_vec(t, "W+72");
    (void) agc_integ_install(0, AGC_JOB_NONE);
    agc_flag_put(AGC_FLAG_VINTFLAG, true);
    agc_flag_put(AGC_FLAG_DIM0FLAG, true);
    agc_flag_put(AGC_FLAG_PRECIFLG, true);
    const agc_status_t st =
        agc_integ_integrv(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0));

    oracle_require_ok(t, "INTWAKE");
    (void) oracle_cmp(t, "status", (long) st, (long) AGC_STATUS_OK, 0);

    oracle_cmp_vec(t, "W", ctx.w.blk[0][0], oracle_out_vec(t, "W"), tol_w);

    for (int m = 0; m < 6; m++)
    {
        if (m > 0)
        {
            (void) snprintf(name, sizeof (name), "W+%d", 6 * m);
            oracle_cmp_vec(t, name, ctx.w.blk[0][m], oracle_out_vec(t, name),
                           tol_w);
        }

        (void) snprintf(name, sizeof (name), "W+%d", 54 + (6 * m));
        oracle_cmp_vec(t, name, ctx.w.blk[1][m], oracle_out_vec(t, name),
                       tol_w);
    }

    (void) agc_integ_perm_get(AGC_VEHICLE_CSM, &sv);
    oracle_cmp_vec(t, "RRECTCSM", sv.rrect, oracle_out_vec(t, "RRECTCSM"),
                   2);
    oracle_cmp_vec(t, "VRECTCSM", sv.vrect, oracle_out_vec(t, "VRECTCSM"),
                   2);
    (void) oracle_cmp(t, "TETCSM", sv.tet, oracle_out_dp(t, "TETCSM"), 0);
}

int main(void)
{
    if (!oracle_load("nav_integ"))
    {
        (void) printf("oracle_nav_integ: SKIP (case files not found)\n");

        return (0);
    }

    for (size_t i = 0; i < sizeof (CASES) / sizeof (CASES[0]); i++)
    {
        run_integ(&CASES[i]);
    }

    for (size_t i = 0; i < sizeof (STEPS) / sizeof (STEPS[0]); i++)
    {
        run_step(&STEPS[i]);
    }

    run_rectify("rectify_earth");
    run_rectify("rectify_moon");
    run_rectify("rectify_earth_neg");
    run_origchng("origchng_timestep");
    run_origchng("origchng_apchk");
    run_third("third_earth_1step", false, 2, 2, 8);
    run_third("third_earth_multi", false, 2, 2, 8);
    run_third("third_moon_1step", true, 2, 2, 8);
    run_third("third_moon_multi", true, 2, 2, 8);
    run_third("third_sphere_entry", false, 2, 2, 8);
    run_wmatrix("wmatrix_6x6", 0);

    return (oracle_finish("oracle_nav_integ"));
}
