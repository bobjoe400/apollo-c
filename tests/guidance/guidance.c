/**********************************************************************
 * FILE NAME: guidance.c
 *
 * PURPOSE:
 *   Unit tests of the guid-powered lane:
 *   1. CALCRVG (the average-G state update): with no PIPA counts and no
 *      previous gravity term, one 2-s step (PGUIDE = 200 cs) moves RN1 by
 *      VN * 2 s within the 2 m LSB of the 2^29 m scale, and VN1 = VN +
 *      GDT1/2, the half-step lunar gravity (mu dt / 2 r^2) at RN1.
 *   2. S40.8 cross-product steering: with BDT = 0 and the measured delta-V
 *      (DELVREF) along the velocity to be gained the commanded rate OMEGAC
 *      is zero; with VG at 90 degrees to the thrust it is not.
 *   3. DELRSPL (V82 splash error in P11): with the target under RONE the
 *      CANTDO path (TFF >= 0) gives RSP-RREC = 0; on a descending 200-km
 *      trajectory (perigee below 300 kft) the error is the free-fall angle
 *      plus the AUGEKUGL entry range, a positive overshoot of a few degrees
 *      to a few tens of degrees.
 *   4. S40.6 stores OPTIND = -0 (optics reserved for the TVC).
 *   5. CLOCPLAY two-part displays (B6): the CSM CLOCKJOB with NVWORD1 =
 *      -BIT9 / -0 and the LM CLOKJOB with DISPDEX = -VB99DEX show V06N40
 *      (V06Nxx) and paste the flashing V99 / V97 as the second part; the
 *      job sleeps in ENDIDLE until the crew answers, and PRO on the CSM
 *      V99 takes V99P (ASTNFLAG set, NVWORD1 back to V06N40).
 *
 * AGC SOURCE:
 *   Luminary099 SERVICER.agc pp. 880-881 (CALCRVG); Comanche055
 *   P40-P47.agc pp. 719-722 (S40.8), p. 730 (S40.6), P30-P37.agc
 *   pp. 643-647 (DELRSPL), pp. 705-706 (CLOCKJOB, ENGREQST, FAILDSP,
 *   V99P); Luminary099 BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc p. 745
 *   (CLOKJOB); DISPLAY_INTERFACE_ROUTINES.agc CM p. 1477 (1STOR2ND)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   RONE          agc_vec_t  O    2^29 m       DELRSPL position input
 *   LAT/LNG(SPL)  agc_dp_t   O    rev          splash target
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_guid_calcrvg      CALCRVG (servicer_common.c)
 *   agc_guid_steering_cross_product  XPRODUCT (burn_steering.c)
 *   agc_guid_delrspl      DELRSPL (csm/burns/external_delta_v.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when a check fails.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Host test; double arithmetic only to build inputs and judge results.
 *   Library assertion reports are printed and counted, not a pass
 *   criterion.
 *
 * NOTES:
 *   The bit-exact checks against the AGC are in tests/oracle/guidance.c.
 *
 * ALGORITHM:
 *   As listed under PURPOSE.
 **********************************************************************/
#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/rendezvous.h"

/* Library-internal entry points (src/csm/burns/sps/internal.h and
 * src/lm/burns/ignition/internal.h; the two cannot share a translation
 * unit: they include the CSM and the LM flag enums).                      */
agc_status_t agc_csm_preburn_call_tigblnk(agc_dp_t dt); /* P40SET    */
agc_status_t ign_call_tig_35(agc_dp_t dt35);            /* CALLT-35  */

#define MU_MOON 4.9027779e12 /* m^3/s^2 (MUM in the AGC)               */

static int g_checks;         /* checks run                    */
static int g_failures;       /* checks failed                 */
static int g_assert_reports; /* library assertion reports seen */

static void      check(bool ok, int line, const char *text);
static agc_mat_t identity_half(void);
static void      test_average_g_coast(void);
static agc_vec_t steer(agc_vec_t vg, agc_vec_t delvref);
static void      test_cross_product_steering(void);
static void      delrspl_case(agc_vec_t r, agc_vec_t v, bool past, double lo, double hi);
static void      test_splash_error(void);
static void      test_tvc_optics_reserved(void);
static void      run_jobs(void);
static int       sleeping_jobs(void);
static bool      dsky_shows(int verb, int noun, bool flash);
static void      test_engine_enable_request_csm(void);
static void      test_engine_fail_csm(void);
static void      test_engine_enable_request_lm(void);
static void      test_preburn_late(void);
static void      test_external_dv_vgdisp_lm(void);

/**********************************************************************
 * Run the tests.
 * AGC: none (test driver).
 *
 * FUNCTION NAME: main
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- 0 when every check passes, 1 otherwise
 **********************************************************************/
int main(void)
{
    test_average_g_coast();
    test_cross_product_steering();
    test_splash_error();
    test_tvc_optics_reserved();
    test_engine_enable_request_csm();
    test_engine_fail_csm();
    test_engine_enable_request_lm();
    test_preburn_late();
    test_external_dv_vgdisp_lm();
    (void) printf("test_guidance: %d checks, %d failures, %d assertion reports\n", g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1); /* exit status */
}

/**********************************************************************
 * The assertion hook of the library for this test: print and count the
 * report (informational, not a pass criterion).
 * AGC: none (test harness).
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   file      const char *  I    source file of the failed assertion
 *   line      int           I    line of the failed assertion
 *   expr      const char *  I    text of the failed expression
 *
 * RETURNS: bool -- false (the assertion stays failed)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d: %s\n", file, line, expr);
    g_assert_reports++;

    return (false);
}

#define CHECK(c) check((c), __LINE__, #c)

/**********************************************************************
 * Count a check; print the failing ones.
 * AGC: none (test harness).
 *
 * FUNCTION NAME: check
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   ok        bool          I    result of the check
 *   line      int           I    test line
 *   text      const char *  I    text of the check
 **********************************************************************/
static void check(bool ok, int line, const char *text)
{
    g_checks++;

    if (!ok)
    {
        g_failures++;
        (void) printf("FAIL line %d: %s\n", line, text);
    }
}

/**********************************************************************
 * REFSMMAT = identity, stored as a half-unit matrix (scale 2^1).
 * AGC: none (test fixture).
 *
 * FUNCTION NAME: identity_half
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_mat_t -- the identity, half-unit rows (2^1)
 **********************************************************************/
static agc_mat_t identity_half(void)
{
    agc_mat_t m; /* rotation matrix, half-unit rows (2^1) */

    m.row[0] = agc_vec_make(AGC_DP_HALF, 0, 0);
    m.row[1] = agc_vec_make(0, AGC_DP_HALF, 0);
    m.row[2] = agc_vec_make(0, 0, AGC_DP_HALF);

    return (m);
}

/* ---- 1. average-G with zero delta-V ----------------------------------- */

/**********************************************************************
 * Test 1: CALCRVG (SERVICER.agc p.880-881) with zero delta-V moves RN1
 * by VN dt and VN1 by the half-step lunar gravity.
 *
 * FUNCTION NAME: test_average_g_coast
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_average_g_coast(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_avg_g_t a = {0};

    a.pguide              = 200;            /* 2 s, cs at 2^28    */
    a.gdt_2               = agc_vec_zero(); /* no previous gravity */
    const double       r0 = 1.85e6;         /* m, ~110 km altitude */
    const double       vx = 16.3;           /* m/cs (1630 m/s)    */
    agc_state_vector_t sv = {agc_vec_zero(), agc_vec_zero(), 0, true};

    sv.r                     = agc_vec_make(0, agc_dp_from_double(r0, 29), 0);
    sv.v                     = agc_vec_make(agc_dp_from_double(vx, 7), 0, 0);
    const agc_mat_t refsmmat = identity_half(); /* REFSMMAT = I, half-unit rows (2^1) */

    agc_status_t st = agc_guid_calcrvg(&ip, AGC_VEHICLE_LM, true, &refsmmat, agc_vec_zero(), &sv, &a); /* status of agc_guid_calcrvg */

    CHECK(st == AGC_STATUS_OK);

    const double lsb_r = ldexp(1.0, 29 - AGC_DP_BITS); /* 2 m       */
    double       dx    = agc_dp_to_double(a.rn1.c[0], 29);
    double       dy    = agc_dp_to_double(a.rn1.c[1], 29) - r0;
    double       dz    = agc_dp_to_double(a.rn1.c[2], 29);

    CHECK(fabs(dx - vx * 200.0) <= 2.0 * lsb_r); /* r += v dt          */
    CHECK(fabs(dy) <= 2.0 * lsb_r);
    CHECK(fabs(dz) <= 2.0 * lsb_r);
    CHECK(agc_dp_to_double(a.delvref.c[0], 7) == 0.0);

    /* VN1 = VN + GDT1/2; GDT1/2 = -mu dt / (2 r1^2) along RN1.             */
    double r1     = sqrt((vx * 200.0) * (vx * 200.0) + r0 * r0);
    double g_half = MU_MOON * 2.0 / (2.0 * r1 * r1) * 0.01; /* m/cs     */
    double gy     = -g_half * (r0 / r1);
    double gx     = -g_half * (vx * 200.0 / r1);
    double vy1    = agc_dp_to_double(a.vn1.c[1], 7);
    double vx1    = agc_dp_to_double(a.vn1.c[0], 7);

    CHECK(fabs(vy1 - gy) <= 0.002 * g_half);
    CHECK(fabs(vx1 - (vx + gx)) <= 0.002 * g_half + ldexp(1.0, 7 - AGC_DP_BITS) * 4.0);
}

/* ---- 2. cross-product steering ------------------------------------------ */

/**********************************************************************
 * Run S40.8 XPRODUCT (P40-P47.agc p.719-722) with all gimbals at 0,
 * REFSMMAT = I, BDT = 0 and CSTEER = 1.
 *
 * FUNCTION NAME: steer
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   vg        agc_vec_t   I    velocity to be gained, m/cs 2^7
 *   delvref   agc_vec_t   I    measured delta-V, m/cs 2^7
 *
 * RETURNS: agc_vec_t -- OMEGAC, the commanded rates
 **********************************************************************/
static agc_vec_t steer(agc_vec_t vg, agc_vec_t delvref)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_cdutrig_t  trig   = {{0}, {0}, {0}, {0}};
    const uint16_t cdu[3] = {0U, 0U, 0U};

    agc_imu_cdtrigs(&trig, cdu);                /* all gimbals at 0   */
    const agc_mat_t refsmmat = identity_half(); /* REFSMMAT = I, half-unit rows (2^1) */
    agc_xprod_in_t  in;                         /* XPRODUCT inputs */

    in.vg            = vg;
    in.delvref       = delvref;
    in.bdt           = agc_vec_zero();
    in.csteer        = AGC_DP_HALF;                /* CSTEER = 1, 2^1    */
    in.kprimedt      = agc_dp_from_double(0.1, 0); /* arbitrary gain     */
    agc_vec_t omegac = agc_vec_zero();             /* OMEGAC, commanded rates; scale of OMEGAC */

    agc_guid_steering_cross_product(&ip, &in, &refsmmat, &trig, &omegac);

    return (omegac);
}

/**********************************************************************
 * Test 2 (P40-P47.agc p.719-722): XPRODUCT gives no rate when the thrust is along VG and a rate
 * about the third axis when VG is at 90 degrees.
 *
 * FUNCTION NAME: test_cross_product_steering
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_cross_product_steering(void)
{
    /* VG 300 m/s and 2 s of thrust (6 m/s) both along SM (1, 2, 2)/3.     */
    const double   ux  = 1.0 / 3.0;                                     /* thrust direction, x */
    const double   uy  = 2.0 / 3.0;                                     /* thrust direction, y */
    const double   uz  = 2.0 / 3.0;                                     /* thrust direction, z */
    agc_vec_t      vg  = agc_vec_make(agc_dp_from_double(3.0 * ux, 7),  /* VG 300 m/s along u, m/cs 2^7 */
                                      agc_dp_from_double(3.0 * uy, 7),
                                      agc_dp_from_double(3.0 * uz, 7));
    agc_vec_t      dv  = agc_vec_make(agc_dp_from_double(0.06 * ux, 7), /* DELVREF 6 m/s along u, m/cs 2^7 */
                                      agc_dp_from_double(0.06 * uy, 7),
                                      agc_dp_from_double(0.06 * uz, 7));
    agc_vec_t      om  = steer(vg, dv);                                 /* OMEGAC, thrust along VG; scale of OMEGAC */
    const agc_dp_t tol = 64;                                            /* a few unit-vector LSBs; LSB of OMEGAC, unscaled */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        CHECK(labs((long int) om.c[i]) <= (long int) tol);
    }

    /* Thrust along SM X, VG along SM Y: a rate about the third axis.       */
    agc_vec_t vg2 = agc_vec_make(0, agc_dp_from_double(3.0, 7), 0);                                    /* VG along SM Y, m/cs 2^7 */
    agc_vec_t dv2 = agc_vec_make(agc_dp_from_double(0.06, 7), 0, 0);                                   /* thrust along SM X, m/cs 2^7 */
    agc_vec_t om2 = steer(vg2, dv2);                                                                   /* OMEGAC, VG at 90 degrees; scale of OMEGAC */
    long int  mag = labs((long int) om2.c[0]) + labs((long int) om2.c[1]) + labs((long int) om2.c[2]); /* sum of |OMEGAC| components */

    CHECK(mag > 1000L * (long int) tol);
}

/**********************************************************************
 * DELRSPL on an earth trajectory: RONE (m 2^29), VONE (m/cs 2^7).
 * AGC: DELRSPL (P30-P37.agc pp.643-647).
 *
 * FUNCTION NAME: delrspl_case
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   r         agc_vec_t   I    RONE, m 2^29
 *   v         agc_vec_t   I    VONE, m/cs 2^7
 *   past      bool        I    call with TFF = 0 (CANTDO path)
 *   lo        double      I    lowest acceptable RSP-RREC, rev
 *   hi        double      I    highest acceptable RSP-RREC, rev
 **********************************************************************/
static void delrspl_case(agc_vec_t r, agc_vec_t v, bool past, double lo, double hi)
{
    agc_interp_t         ip;      /* interpreter state */
    agc_tff_t            tff = {0};
    agc_dp_t             t   = 0; /* TFF to 300 kft, cs 2^28 */
    agc_latlong_t        ll;      /* target latitude/longitude */
    agc_navaux_padload_t pad;     /* pad loads (earth model) */
    agc_latlong_opt_t    opt = {false, false};

    agc_state_init(AGC_VEHICLE_CSM);
    agc_interp_init(&ip);
    agc_guid_init(AGC_VEHICLE_CSM);
    agc_flag_clear(AGC_FLAG_SERVICER_RUNNING);
    agc_rendz_erasable()->rone         = r;
    agc_rendz_erasable()->tstart_orbit = 0;
    tff.rone                           = r;
    tff.vone                           = v;
    tff.rtmu                           = (agc_dp_t) 176229826;                     /* 1/RTMUE, 2^17 (R30)             */
    agc_tff_conic(&ip, &tff);
    CHECK(agc_tff_calctff(&ip, &tff, agc_dp_from_double(6373338.0 + 91440.0, 29), &t) == AGC_STATUS_OK);
    agc_navaux_get_padload(&pad);
    CHECK(agc_latlong_from_r(&ip, &pad, opt, r, 0, &ll) == AGC_STATUS_OK);
    agc_entry_state()->lat_spl         = ll.lat;                                   /* target under RONE at TIME 0 */
    agc_entry_state()->lng_spl         = ll.lon;
    CHECK(agc_guid_delrspl(&ip, &tff, past ? 0 : agc_dp_neg(t)) == AGC_STATUS_OK); /* CANTDO path at TFF 0, else the free-fall time */
    double e                           = agc_dp_to_double(agc_guid_state()->csm.rsp_rrec, 0);

    CHECK(e >= lo && e <= hi);
    CHECK(!agc_flag_test(AGC_FLAG_ERADFLAG));
}

/**********************************************************************
 * Test 3: DELRSPL (P30-P37.agc pp.643-647): the CANTDO path and a
 * descending 200-km trajectory.
 *
 * FUNCTION NAME: test_splash_error
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_splash_error(void)
{
    agc_vec_t r = agc_vec_make(agc_dp_from_double(6578000.0, 29), 0, 0);                     /* RONE, m 2^29 */
    agc_vec_t v = agc_vec_make(agc_dp_from_double(-1.0, 7), agc_dp_from_double(76.0, 7), 0); /* VONE, m/cs 2^7 */

    delrspl_case(r, v, true, -2e-4, 2e-4); /* CANTDO: -angle to target  */
    delrspl_case(r, v, false, 0.005, 0.3); /* free fall + entry range   */
}

/**********************************************************************
 * S40.6 (P40-P47.agc p.730): CS ZERO / TS OPTIND reserves the optics
 * for the TVC with OPTIND = -0 (AGC_OPTIND_NEG0), which 33OPTMON and V41
 * test; -1 would mean "coarse optics not working".
 * AGC: S40.6 (P40-P47.agc p.730)
 *
 * FUNCTION NAME: test_tvc_optics_reserved
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_tvc_optics_reserved(void)
{
    agc_power_on(AGC_VEHICLE_CSM);
    agc_inhint(); /* a task runs inside T3RUPT */
    CHECK(agc_task_gimbal_test_trim() == AGC_STATUS_OK);
    agc_relint();
    CHECK(agc_erase_optind() == AGC_OPTIND_NEG0);
}

/**********************************************************************
 * Run ready jobs until the executive is idle (bounded).
 * AGC: none (test helper).
 *
 * FUNCTION NAME: run_jobs
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void run_jobs(void)
{
    for (int i = 0; i < 100 && agc_exec_run_one() != AGC_STATUS_NOT_FOUND; i++)
    {
        /* the display jobs and what they start */
    }
}

/**********************************************************************
 * Count the core sets of sleeping jobs (JOBSLEEP: ENDIDLE waits).
 * AGC: none (test helper).
 *
 * FUNCTION NAME: sleeping_jobs
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- number of sleeping core sets
 **********************************************************************/
static int sleeping_jobs(void)
{
    int n = 0; /* sleeping core sets */

    for (int c = 0; c < agc_exec_core_sets(); c++)
    {
        agc_exec_core_info_t ci; /* core set c */

        agc_exec_core_info(c, &ci);
        n += (ci.state == AGC_CORE_SLEEPING) ? 1 : 0; /* count sleepers */
    }

    return (n);
}

/**********************************************************************
 * The DSKY shows verb / noun with the V/N flash on or off.
 * AGC: none (test helper).
 *
 * FUNCTION NAME: dsky_shows
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------------------------------
 *   verb      int   I    verb digits expected
 *   noun      int   I    noun digits expected
 *   flash     bool  I    V/N flash expected
 *
 * RETURNS: bool -- true when the DSKY shows exactly that
 **********************************************************************/
static bool dsky_shows(int verb, int noun, bool flash)
{
    agc_dsky_state_t s; /* DSKY readout */

    CHECK(agc_dsky_get_state(&s) == AGC_STATUS_OK);

    return (s.verb[0] == verb / 10 && s.verb[1] == verb % 10 && s.noun[0] == noun / 10 && s.noun[1] == noun % 10 && s.flash == flash);
}

/**********************************************************************
 * Test 5a: ENGREQST (P40-P47.agc p.705): NVWORD1 = -BIT9 makes the
 * CLOCKJOB flash V99N40 as the paste after V06N40 and sleep in ENDIDLE;
 * PRO is V99P (p.706): ASTNFLAG set, NVWORD1 = V06N40. With V99 executed
 * as a verb (B6) the display answered itself with ENTER (V99E).
 * AGC: ENGREQST, V99P (P40-P47.agc pp. 705-706)
 *
 * FUNCTION NAME: test_engine_enable_request_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_engine_enable_request_csm(void)
{
    agc_power_on(AGC_VEHICLE_CSM);
    agc_dsky_init();
    run_jobs();
    agc_guid_state()->csm.nvword1 = (agc_sp_t) -0400;                   /* -BIT9: V99 flash */
    agc_inhint();
    CHECK(agc_exec_novac(AGC_PRIO27, AGC_JOB_CLOCKJOB) == AGC_STATUS_OK);
    agc_relint();
    run_jobs();
    CHECK(dsky_shows(99, 40, true));
    CHECK(sleeping_jobs() == 1);
    CHECK(!agc_flag_test(AGC_FLAG_ASTNFLAG));
    CHECK(agc_dsky_keys("P") == AGC_STATUS_OK);
    run_jobs();
    CHECK(agc_flag_test(AGC_FLAG_ASTNFLAG));
    CHECK(agc_guid_state()->csm.nvword1 == (agc_sp_t) (6 * 0200 + 40)); /* V06N40 */
    CHECK(sleeping_jobs() == 0);
}

/**********************************************************************
 * Test 5b: FAILDSP (P40-P47.agc p.705): NVWORD1 = -0 (-1 in the C word)
 * makes the CLOCKJOB flash V97N40 as the paste after V06N40 and sleep.
 * AGC: FAILDSP (P40-P47.agc p.705)
 *
 * FUNCTION NAME: test_engine_fail_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_engine_fail_csm(void)
{
    agc_power_on(AGC_VEHICLE_CSM);
    agc_dsky_init();
    run_jobs();
    agc_guid_state()->csm.nvword1 = (agc_sp_t) -1; /* the AGC's -0: V97 flash */
    agc_inhint();
    CHECK(agc_exec_novac(AGC_PRIO27, AGC_JOB_CLOCKJOB) == AGC_STATUS_OK);
    agc_relint();
    run_jobs();
    CHECK(dsky_shows(97, 40, true));
    CHECK(sleeping_jobs() == 1);
}

/**********************************************************************
 * Test 5c: CLOKJOB -13 (BURN_BABY_BURN p.745): DISPDEX = -VB99DEX with
 * the P40 table flashes V99N40, the paste after the countdown V06N40,
 * and sleeps until the crew answers.
 * AGC: CLOKJOB -13 (Luminary099 p.745)
 *
 * FUNCTION NAME: test_engine_enable_request_lm
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_engine_enable_request_lm(void)
{
    agc_power_on(AGC_VEHICLE_LM);
    agc_dsky_init();
    run_jobs();
    agc_guid_state()->lm.which   = (agc_sp_t) AGC_LM_WHICH_DPS_BURN; /* P40TABLE: noun 40 */
    agc_guid_state()->lm.dispdex = (agc_sp_t) -013;                  /* -VB99DEX           */
    agc_inhint();
    CHECK(agc_exec_novac(AGC_PRIO27, AGC_JOB_CLOCKJOB) == AGC_STATUS_OK);
    agc_relint();
    run_jobs();
    CHECK(dsky_shows(99, 40, true));
    CHECK(sleeping_jobs() == 1);
}

/**********************************************************************
 * Test 6: P40SET (P40-P47.agc C p.686) and CALLT-35 (BURN_BABY_BURN
 * L p.734) when MIDTOAV1 finished less than 5 s before TIG - 30: the
 * LONGCALL delay is zero or negative and the AGC ends in POODOO 1204
 * (WATLST0- / LONGPOOH), not in a task 1 cs later (closed-loop-fixes,
 * document wave: burn.c's P40TMP fallback).
 * AGC: P40SET, CALLT-35
 *
 * FUNCTION NAME: test_preburn_late
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_preburn_late(void)
{
    agc_power_on(AGC_VEHICLE_CSM);
    run_jobs();
    CHECK(agc_csm_preburn_call_tigblnk(400) == AGC_STATUS_POODOO); /* P40TMP = -1 s */
    CHECK(agc_erase_failreg(0) == 01204u);
    agc_power_on(AGC_VEHICLE_CSM);
    run_jobs();
    CHECK(agc_csm_preburn_call_tigblnk(1500) == AGC_STATUS_OK);    /* P40TMP = 10 s */
    agc_power_on(AGC_VEHICLE_LM);
    run_jobs();
    CHECK(ign_call_tig_35(0) == AGC_STATUS_POODOO);                /* TIG-35 now */
    CHECK(agc_erase_failreg(0) == 01204u);
    agc_power_on(AGC_VEHICLE_LM);
    run_jobs();
    CHECK(ign_call_tig_35(-100) == AGC_STATUS_POODOO);             /* passed */
}

/**********************************************************************
 * Luminary S30.1 (P30_P37.agc pp.616-617): DELTA V MAG. goes to DELVSAB,
 * which Luminary's ERASABLE_ASSIGNMENTS makes the same erasable as VGDISP
 * (VGDISP = DELVSAB), so P30's N42 R3 shows it. NASSP DOI finding: the C
 * kept two variables and showed R3 = 0. LM state on an earth orbit at
 * TET = TIG (LEMPREC makes no step), DELVSLV = (+22, 0, -5.4) m/s LV.
 * AGC: S30.1, VGDISP = DELVSAB (Luminary099)
 *
 * FUNCTION NAME: test_external_dv_vgdisp_lm
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_external_dv_vgdisp_lm(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    agc_integ_sv_t sv = {0}; /* LM permanent state, earth */

    sv.rrect.c[0]   = agc_dp_from_double(6678137.0, 29);
    sv.vrect.c[1]   = agc_dp_from_double(77.26, 7); /* m/cs */
    sv.rcv          = sv.rrect;
    sv.vcv          = sv.vrect;
    sv.tet          = agc_dp_from_double(100000.0, 28);
    agc_integ_perm_set(AGC_VEHICLE_LM, &sv);
    agc_erase_set_tig(sv.tet);
    agc_rendz_t *rz = agc_rendz_erasable();         /* DELVLVC, DELVSAB */

    rz->delvlvc.c[0] = agc_dp_from_double(0.22, 7);
    rz->delvlvc.c[2] = agc_dp_from_double(-0.054, 7);
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_guid_external_delta_v_vectors(&ip, AGC_VEHICLE_LM); /* S30.1 */

    CHECK(st == AGC_STATUS_OK);
    CHECK(rz->delvsab > 0);
    CHECK(agc_guid_state()->burn.vgdisp == rz->delvsab);
}
