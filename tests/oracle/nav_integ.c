/**********************************************************************
 * FILE NAME: nav_integ.c
 *
 * PURPOSE:
 *   nav-integ against the real AGC integration (yaAGC oracle).  Inputs
 *   and expected words: tools/oracle/cases/nav_integ.spec / .txt
 *   (Comanche055).  INTEGRVS from a fixed RCV/VCV/TET to TDEC1: one
 *   Nystrom step (TIMESTEP, three ACCOMP stages with GAMCOMP and OBLATE,
 *   FBR3 / KEPPREP / KEPLER) and multi-step runs, earth (J2..J4) and moon
 *   (J2, J3, QUALITY1-3 with the E3J22R2M / E32C31RM pad loads); RECTIFY
 *   alone; origin changes; third bodies; the W matrix.  ACCOMP is not a
 *   separate entry in C, so it is checked through FV, the disturbing
 *   acceleration of the last stage.  Tolerances in DP LSB (2^-28 of the
 *   scale): see the CASES table and docs/modules/oracle-cases.md (F1).
 *
 * AGC SOURCE:
 *   Comanche055 INTEGRATION_INITIALIZATION.agc, ORBITAL_INTEGRATION.agc
 *
 * EXTERNAL VARIABLES:
 *   none (the oracle cases set the inputs)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_integ_*           the integration package (nav/integration.h)
 *   oracle_*              case file reader and comparisons (oracle_txt.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none (mismatches are counted; exit status from oracle_finish)
 **********************************************************************/
#include <stdio.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "oracle_txt.h"

#define TOL_RECT 0L /* RECTIFY: shifts and adds, bit-exact            */

typedef struct run_case
{
    const char *tag;    /* case name in nav_integ.txt */
    bool        moon;   /* MOONFLAG at the start */
    long int    tol_r;  /* position words (RATT, RRECT, RATT1)          */
    long int    tol_v;  /* velocity words                               */
    long int    tol_fv; /* FV                                           */
} run_case_t;

/* Tolerances are the residuals left by yaAGC's DV bug (F1 in
 * docs/modules/oracle-cases.md): against the DV-patched probe
 * (tools/oracle/cases/gen/dvcheck.sh) every case below agrees exactly
 * except integ_moon_1step / third_moon_1step VATT (1 LSB) and
 * integ_earth_long (1 LSB velocity), the TNUV words the Nystrom step
 * leaves (sign-disagreeing) that RECTIFY's VSR then rounds
 * (review-nav-integ N9).  KEPLER's VCV differs in its low bits from the
 * second step on with this yaAGC, and the Encke state drifts.            */
static const run_case_t CASES[] = {
    {"integ_earth_1step", false, 0, 0, 3},
    {"integ_earth2_1step", false, 0, 0, 2},
    {"integ_earth_multi", false, 32, 4609, 488},
    {"integ_moon_1step", true, 0, 1, 0},
    {"integ_moon_multi", true, 32, 2560, 436},

    /* TDEC1 - TET > 2^20 cs (DT/2COMP overflow, SIGNMPAC): TET, TAT and
     * MU(P) exact.  Against this yaAGC the state drifts ~12 km in 3 h
     * (F1); against the DV-patched probe 0 LSB position, 1 LSB velocity,
     * 288 LSB FV at 3 h.                                                    */
    {"integ_earth_long", false, 5731, 251822, 1842528},
};

typedef struct step_case
{
    const char *tag;    /* case name in nav_integ.txt */
    bool        moon;   /* MOONFLAG at the start */
    long int    tol_dv; /* TDELTAV, TNUV                                */
    long int    tol_cv; /* RCV, VCV (KEPLER)                            */
    long int    tol_x;  /* XKEP                                         */
} step_case_t;

static const step_case_t STEPS[] = {
    {"step_earth_max", false, 2, 0, 0},
    {"step_earth_2max", false, 2, 512, 0},
    {"step_moon", true, 0, 0, 0},
};

static void fresh(void);
static void run_integ(const run_case_t *k);
static void run_step(const step_case_t *k);
static void run_rectify(const char *t);
static void run_origchng(const char *t);
static void run_third(const char *t, bool moon, long int tol_r, long int tol_v, long int tol_fv);
static void run_wmatrix(const char *t, long int tol_w);

/**********************************************************************
 * AGC: none (test driver) -- loads tools/oracle/cases/nav_integ.* and runs
 * every case; prints the oracle summary
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- int the oracle summary status (0 when no check failed)
 *********************************************************************/
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
    run_third("third_earth_1step", false, 0, 0, 0);
    run_third("third_earth_multi", false, 0, 0, 0);
    run_third("third_moon_1step", true, 0, 1, 0);
    run_third("third_moon_multi", true, 0, 0, 0);
    run_third("third_sphere_entry", false, 0, 0, 0);
    run_wmatrix("wmatrix_6x6", 0);

    return (oracle_finish("oracle_nav_integ"));
}

/**********************************************************************
 * AGC: none (AGC_ASSERT hook) -- print a failed assertion
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   file      const char *  I    source file of the assertion
 *   line      int           I    its line
 *   expr      const char *  I    the asserted expression
 *
 * RETURNS: bool -- bool false (the assertion failed)
 *********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);

    return (false);
}

/**********************************************************************
 * AGC: none (test setup) -- CSM state, executive, waitlist, MODREG, pad loads
 * and TEPHEM reset
 *
 * FUNCTION NAME: fresh
 *********************************************************************/
static void fresh(void)
{
    agc_navaux_padload_t pad = {0};

    agc_state_init(AGC_VEHICLE_CSM);
    agc_exec_init();
    agc_waitlist_init();
    agc_erase_set_modreg(0);
    agc_navaux_set_padload(&pad);
    agc_erase_set_tephem(0);
}

/**********************************************************************
 * INTEGRVS (INTEGRATION_INITIALIZATION.agc p. 1319) -- one integration case to
 * INTWAKE: push list, RRECT, VRECT, TET, FV, MOONFLAG, MIDFLAG
 *
 * FUNCTION NAME: run_integ
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  --------------------------------
 *   k         const run_case_t *  I    the case and its tolerances (LSB)
 *********************************************************************/
static void run_integ(const run_case_t *k)
{
    agc_interp_t    ip;         /* interpretive state */
    agc_integ_ctx_t ctx;        /* integration erasables */
    const char     *t = k->tag; /* case name */

    fresh();
    agc_interp_init(&ip);
    agc_integ_ctx_init(&ctx);
    (void) agc_integ_install(0, AGC_JOB_NONE);
    agc_integ_set_lunar_coeffs((agc_sp_t) oracle_in(t, "E3J22R2M", 0), (agc_sp_t) oracle_in(t, "E32C31RM", 0));
    ctx.sv.rcv            = oracle_in_vec(t, "RCV");
    ctx.sv.vcv            = oracle_in_vec(t, "VCV");
    ctx.sv.tet            = oracle_in_dp(t, "TET", 0);
    agc_flag_put(AGC_FLAG_MOONFLAG, k->moon);
    agc_flag_put(AGC_FLAG_INTYPFLG, false);
    const agc_status_t st = agc_integ_integrvs(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0)); /* status of INTEGRVS */

    oracle_require_ok(t, "INTWAKE");
    (void) oracle_cmp(t, "status", (long int) st, (long int) AGC_STATUS_OK, 0);
    (void) oracle_cmp(t, "out.valid", (long int) ctx.out.valid, 1L, 0);
    oracle_cmp_vec(t, "RATT", ctx.out.ratt, oracle_out_vec(t, "VAC1"), k->tol_r);
    oracle_cmp_vec(t, "VATT", ctx.out.vatt, oracle_out_vec(t, "VAC1+6"), k->tol_v);
    (void) oracle_cmp(t, "TAT", ctx.out.tat, oracle_out_dp(t, "VAC1+12"), 0);
    oracle_cmp_vec(t, "RATT1", ctx.out.ratt1, oracle_out_vec(t, "VAC1+14"), k->tol_r);
    oracle_cmp_vec(t, "VATT1", ctx.out.vatt1, oracle_out_vec(t, "VAC1+20"), k->tol_v);
    (void) oracle_cmp(t, "MU(P)", ctx.out.mu_p, oracle_out_dp(t, "VAC1+26"), 0);
    oracle_cmp_vec(t, "RRECT", ctx.sv.rrect, oracle_out_vec(t, "RRECT"), k->tol_r);
    oracle_cmp_vec(t, "VRECT", ctx.sv.vrect, oracle_out_vec(t, "VRECT"), k->tol_v);
    (void) oracle_cmp(t, "TET", ctx.sv.tet, oracle_out_dp(t, "TET"), 0);
    oracle_cmp_vec(t, "FV", ctx.fv, oracle_out_vec(t, "FV"), k->tol_fv);
    (void) oracle_cmp(t, "MOONFLAG", agc_flag_test(AGC_FLAG_MOONFLAG), (oracle_out_sp(t, "FLAGWRD0") & 04000) != 0, 0);
    (void) oracle_cmp(t, "MIDFLAG", agc_flag_test(AGC_FLAG_MIDFLAG), (oracle_out_sp(t, "FLAGWRD0") & 010000) != 0, 0);
}

/**********************************************************************
 * INTEGRVS with STATEFLG + VINTFLAG (INTEGRATION_INITIALIZATION.agc p. 1321)
 * -- the AGC stops at PHASCHNG in ANDOUT, before RECTOUT; C: the permanent CSM
 * state ATOPCSM copied there
 *
 * FUNCTION NAME: run_step
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   k         const step_case_t *  I    the case and its tolerances (LSB)
 *********************************************************************/
static void run_step(const step_case_t *k)
{
    agc_interp_t    ip;         /* interpretive state */
    agc_integ_ctx_t ctx;        /* integration erasables */
    agc_integ_sv_t  sv;         /* the permanent CSM state after the run */
    const char     *t = k->tag; /* case name */

    fresh();
    agc_interp_init(&ip);
    agc_integ_ctx_init(&ctx);
    (void) agc_integ_install(0, AGC_JOB_NONE);
    agc_integ_set_lunar_coeffs((agc_sp_t) oracle_in(t, "E3J22R2M", 0), (agc_sp_t) oracle_in(t, "E32C31RM", 0));
    ctx.sv.rcv = oracle_in_vec(t, "RCV");
    ctx.sv.vcv = oracle_in_vec(t, "VCV");
    ctx.sv.tet = oracle_in_dp(t, "TET", 0);
    agc_flag_put(AGC_FLAG_MOONFLAG, k->moon);
    agc_flag_put(AGC_FLAG_STATEFLG, true);
    agc_flag_put(AGC_FLAG_VINTFLAG, true);
    (void) agc_integ_integrvs(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0));
    agc_integ_perm_get(AGC_VEHICLE_CSM, &sv);
    oracle_require_ok(t, "PHASCHNG");
    oracle_cmp_vec(t, "TDELTAV", sv.deltav, oracle_out_vec(t, "TDELTAV"), k->tol_dv);
    oracle_cmp_vec(t, "TNUV", sv.nuv, oracle_out_vec(t, "TNUV"), k->tol_dv);
    oracle_cmp_vec(t, "RCV", sv.rcv, oracle_out_vec(t, "RCV"), k->tol_cv);
    oracle_cmp_vec(t, "VCV", sv.vcv, oracle_out_vec(t, "VCV"), k->tol_cv);
    (void) oracle_cmp(t, "TC", sv.tc, oracle_out_dp(t, "TC"), 0);
    (void) oracle_cmp(t, "XKEP", sv.xkep, oracle_out_dp(t, "XKEP"), k->tol_x);
    (void) oracle_cmp(t, "TET", sv.tet, oracle_out_dp(t, "TET"), 0);
    oracle_cmp_vec(t, "FV", ctx.fv, oracle_out_vec(t, "FV"), 0);
}

/**********************************************************************
 * RECTIFY (ORBITAL_INTEGRATION.agc p. 1347) -- one RECTIFY case, bit exact
 *
 * FUNCTION NAME: run_rectify
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   t         const char *  I    case tag
 *********************************************************************/
static void run_rectify(const char *t)
{
    agc_interp_t    ip;  /* interpretive state */
    agc_integ_ctx_t ctx; /* integration erasables */

    fresh();
    agc_interp_init(&ip);
    agc_integ_ctx_init(&ctx);
    ctx.pbody     = oracle_in(t, "PBODY", 0);
    ctx.sv.rcv    = oracle_in_vec(t, "RCV");
    ctx.sv.vcv    = oracle_in_vec(t, "VCV");
    ctx.sv.deltav = oracle_in_vec(t, "TDELTAV");
    ctx.sv.nuv    = oracle_in_vec(t, "TNUV");
    ctx.sv.tc     = oracle_in_dp(t, "TC", 0);
    ctx.sv.xkep   = oracle_in_dp(t, "XKEP", 0);
    agc_integ_rectify(&ip, &ctx);
    oracle_require_ok(t, "POODOO");
    oracle_cmp_vec(t, "RCV", ctx.sv.rcv, oracle_out_vec(t, "RCV"), TOL_RECT);
    oracle_cmp_vec(t, "VCV", ctx.sv.vcv, oracle_out_vec(t, "VCV"), TOL_RECT);
    oracle_cmp_vec(t, "TDELTAV", ctx.sv.deltav, oracle_out_vec(t, "TDELTAV"), 0);
    oracle_cmp_vec(t, "TNUV", ctx.sv.nuv, oracle_out_vec(t, "TNUV"), 0);
    (void) oracle_cmp(t, "TC", ctx.sv.tc, oracle_out_dp(t, "TC"), 0);
    (void) oracle_cmp(t, "XKEP", ctx.sv.xkep, oracle_out_dp(t, "XKEP"), 0);
}

/**********************************************************************
 * ORIGCHNG (ORBITAL_INTEGRATION.agc p. 1346) -- origin change with STATEFLG +
 * VINTFLAG (zero ephemeris: the moon at the origin).  origchng_timestep:
 * switched in TIMESTEP, SVDWN1 sees X2 = 2.  origchng_apchk: switched in
 * A-PCHK, SVDWN1 sees X2 = ORIGEX and stores RN = VN = 0.  RN / VN are preset
 * to a pattern so a missing store shows.
 *
 * FUNCTION NAME: run_origchng
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   t         const char *  I    case tag
 *********************************************************************/
static void run_origchng(const char *t)
{
    agc_interp_t    ip;                       /* interpretive state */
    agc_integ_ctx_t ctx;                      /* integration erasables */
    agc_vec_t       pattern = agc_vec_zero(); /* RN / VN preset, m 2^29 */

    fresh();
    agc_interp_init(&ip);
    agc_integ_ctx_init(&ctx);
    (void) agc_integ_install(0, AGC_JOB_NONE);
    pattern               = oracle_in_vec(t, "RN");
    agc_erase_set_rn(pattern);
    agc_erase_set_vn(pattern);
    ctx.sv.rcv            = oracle_in_vec(t, "RCV");
    ctx.sv.vcv            = oracle_in_vec(t, "VCV");
    ctx.sv.tet            = oracle_in_dp(t, "TET", 0);
    agc_flag_put(AGC_FLAG_STATEFLG, true);
    agc_flag_put(AGC_FLAG_VINTFLAG, true);
    const agc_status_t st = agc_integ_integrvs(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0)); /* status of INTEGRVS */

    oracle_require_ok(t, "INTWAKE");
    (void) oracle_cmp(t, "status", (long int) st, (long int) AGC_STATUS_OK, 0);
    oracle_cmp_vec(t, "RN", agc_erase_rn(), oracle_out_vec(t, "RN"), 0);
    oracle_cmp_vec(t, "VN", agc_erase_vn(), oracle_out_vec(t, "VN"), 0);
    (void) oracle_cmp(t, "PIPTIME", agc_erase_piptime(), oracle_out_dp(t, "PIPTIME"), 0);
    (void) oracle_cmp(t, "PBODY", ctx.pbody, oracle_out_sp(t, "PBODY"), 0);
    (void) oracle_cmp(t, "MOONFLAG", agc_flag_test(AGC_FLAG_MOONFLAG), (oracle_out_sp(t, "FLAGWRD0") & 04000) != 0, 0);
    (void) oracle_cmp(t, "STATEFLG", agc_flag_test(AGC_FLAG_STATEFLG), (oracle_out_sp(t, "FLAGWRD3") & 00020) != 0, 0);
    oracle_cmp_vec(t, "RRECT", ctx.sv.rrect, oracle_out_vec(t, "RRECT"), 0);
    oracle_cmp_vec(t, "VRECT", ctx.sv.vrect, oracle_out_vec(t, "VRECT"), 0);
    (void) oracle_cmp(t, "TET", ctx.sv.tet, oracle_out_dp(t, "TET"), 0);
}

/**********************************************************************
 * ACCOMP third bodies (ORBITAL_INTEGRATION.agc pp. 1337-1338) -- MIDFLAG:
 * ephemeris pad loads as nav_aux.spec, LSPOS in every ACCOMP stage, CHKSWTCH
 * at each TIMESTEP and in A-PCHK
 *
 * FUNCTION NAME: run_third
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   t         const char *  I    case tag
 *   moon      bool          I    MOONFLAG at the start
 *   tol_r     long int          I    position tolerance, LSB
 *   tol_v     long int          I    velocity tolerance, LSB
 *   tol_fv    long int          I    FV tolerance, LSB
 *********************************************************************/
static void run_third(const char *t, bool moon, long int tol_r, long int tol_v, long int tol_fv)
{
    agc_interp_t    ip;  /* interpretive state */
    agc_integ_ctx_t ctx; /* integration erasables */
    agc_ephem_csm_t e;   /* CM ephemeris pad load */

    fresh();
    e.timemo = oracle_in_tp(t, "TIMEMO", 0);

    for (int k = 0; k < AGC_EPHEM_NCOEF; k++)
    {
        e.vecoem[k] = oracle_in_vec_at(t, "VECOEM", k);
    }

    e.reso                = oracle_in_vec(t, "RESO");
    e.veso                = oracle_in_vec(t, "VESO");
    e.omegaes             = oracle_in_dp(t, "OMEGAES", 0);
    agc_ephem_set_csm_data(&e);
    agc_erase_set_tephem(oracle_in_tp(t, "TEPHEM", 0));
    agc_interp_init(&ip);
    agc_integ_ctx_init(&ctx);
    (void) agc_integ_install(0, AGC_JOB_NONE);
    agc_integ_set_lunar_coeffs((agc_sp_t) oracle_in(t, "E3J22R2M", 0), (agc_sp_t) oracle_in(t, "E32C31RM", 0));
    ctx.sv.rcv            = oracle_in_vec(t, "RCV");
    ctx.sv.vcv            = oracle_in_vec(t, "VCV");
    ctx.sv.tet            = oracle_in_dp(t, "TET", 0);
    agc_flag_put(AGC_FLAG_MOONFLAG, moon);
    const agc_status_t st = agc_integ_integrvs(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0)); /* status of INTEGRVS */

    oracle_require_ok(t, "INTWAKE");
    (void) oracle_cmp(t, "status", (long int) st, (long int) AGC_STATUS_OK, 0);
    oracle_cmp_vec(t, "RATT", ctx.out.ratt, oracle_out_vec(t, "VAC1"), tol_r);
    oracle_cmp_vec(t, "VATT", ctx.out.vatt, oracle_out_vec(t, "VAC1+6"), tol_v);
    (void) oracle_cmp(t, "TAT", ctx.out.tat, oracle_out_dp(t, "VAC1+12"), 0);
    (void) oracle_cmp(t, "MU(P)", ctx.out.mu_p, oracle_out_dp(t, "VAC1+26"), 0);
    (void) oracle_cmp(t, "TET", ctx.sv.tet, oracle_out_dp(t, "TET"), 0);
    oracle_cmp_vec(t, "FV", ctx.fv, oracle_out_vec(t, "FV"), tol_fv);
    (void) oracle_cmp(t, "PBODY", ctx.pbody, oracle_out_sp(t, "PBODY"), 0);
    (void) oracle_cmp(t, "MIDFLAG", agc_flag_test(AGC_FLAG_MIDFLAG), (oracle_out_sp(t, "FLAGWRD0") & 010000) != 0, 0);
}

/**********************************************************************
 * DOW.. / NEXTCOL / RELOADSV (ORBITAL_INTEGRATION.agc pp. 1348-1352) -- W
 * matrix: INTEGRV of the permanent CSM state with DIM0FLAG (6x6), the W
 * columns integrated with DOW.. after every state step
 *
 * FUNCTION NAME: run_wmatrix
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   t         const char *  I    case tag
 *   tol_w     long int          I    W tolerance, LSB
 *********************************************************************/
static void run_wmatrix(const char *t, long int tol_w)
{
    agc_interp_t    ip;       /* interpretive state */
    agc_integ_ctx_t ctx;      /* integration erasables */
    agc_integ_sv_t  sv;       /* the permanent CSM state */
    char            name[16]; /* W column name, W+k */

    fresh();
    agc_interp_init(&ip);
    agc_integ_ctx_init(&ctx);
    agc_erase_set_modreg((agc_sp_t) oracle_in(t, "MODREG", 0));
    sv                    = ctx.sv;
    sv.rrect              = oracle_in_vec(t, "RRECTCSM");
    sv.vrect              = oracle_in_vec(t, "RRECTCSM+6");
    sv.rcv                = oracle_in_vec(t, "RRECTCSM+26");
    sv.vcv                = oracle_in_vec(t, "RRECTCSM+32");
    agc_integ_perm_set(AGC_VEHICLE_CSM, &sv);
    ctx.w.blk[0][0]       = oracle_in_vec(t, "W");
    ctx.w.blk[1][3]       = oracle_in_vec(t, "W+72");
    (void) agc_integ_install(0, AGC_JOB_NONE);
    agc_flag_put(AGC_FLAG_VINTFLAG, true);
    agc_flag_put(AGC_FLAG_DIM0FLAG, true);
    agc_flag_put(AGC_FLAG_PRECIFLG, true);
    const agc_status_t st = agc_integ_integrv(&ip, &ctx, oracle_in_dp(t, "VAC1+32", 0)); /* status of INTEGRV */

    oracle_require_ok(t, "INTWAKE");
    (void) oracle_cmp(t, "status", (long int) st, (long int) AGC_STATUS_OK, 0);

    oracle_cmp_vec(t, "W", ctx.w.blk[0][0], oracle_out_vec(t, "W"), tol_w);

    for (int m = 0; m < 6; m++)
    {
        if (m > 0)
        {
            (void) snprintf(name, sizeof (name), "W+%d", 6 * m);
            oracle_cmp_vec(t, name, ctx.w.blk[0][m], oracle_out_vec(t, name), tol_w);
        }

        (void) snprintf(name, sizeof (name), "W+%d", 54 + (6 * m));
        oracle_cmp_vec(t, name, ctx.w.blk[1][m], oracle_out_vec(t, name), tol_w);
    }

    agc_integ_perm_get(AGC_VEHICLE_CSM, &sv);
    oracle_cmp_vec(t, "RRECTCSM", sv.rrect, oracle_out_vec(t, "RRECTCSM"), 0);
    oracle_cmp_vec(t, "VRECTCSM", sv.vrect, oracle_out_vec(t, "VRECTCSM"), 0);
    (void) oracle_cmp(t, "TETCSM", sv.tet, oracle_out_dp(t, "TETCSM"), 0);
} /* run_wmatrix */
