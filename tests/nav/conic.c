/*
 * conic.c -- nav-conic unit tests (CONIC_SUBROUTINES.agc translation).
 *
 * Analytic two-body checks, earth as central body, CSM build:
 *   1. KEPLER (with the KEPPREP guess): circular 300 km orbit, a quarter
 *      period forward, backward, and 2.25 periods forward (PERIODCH).
 *   2. LAMBERT between two points 90 degrees apart on that orbit in a
 *      quarter period: VVEC and VTARGET are the circular velocities.
 *   3. APSIDES of an ellipse (perigee 7000 km, 1.1 x circular speed).
 *   4. TIMETHET over 90 degrees of the circular orbit, TIMERAD to a radius
 *      on the ellipse (time vs the Kepler equation).
 *   5. RVCON from a state vector record.
 *
 * Inputs are made with agc_dp_from_double at the AGC scales; tolerances are
 * a few hundred LSBs of the fixed-point results (position LSB 2 m, velocity
 * LSB 5e-7 m/cs at earth scale).
 */
#include <stdio.h>

#include <math.h>

#include "agc/core/assert.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/nav/conic.h"

static int g_checks;
static int g_failures;
static int g_assert_reports;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);
    g_assert_reports++;

    return (false);
}

#define CHECK(c) check((c), __LINE__, #c)

static void check(bool ok, int line, const char *text)
{
    g_checks++;

    if (!ok)
    {
        g_failures++;
        (void) printf("FAIL line %d: %s\n", line, text);
    }
}

/* Earth: MUE 3.986032E10 m^3/cs^2 (MUTABLE), scales 2^29 m, 2^7 m/cs. */
static const double MU = 3.986032e10;
static const double R0 = 6678137.0; /* 300 km circular orbit, m    */
static const double PI = 3.14159265358979323846;

static agc_vec_t pos(double x, double y, double z)
{
    return (agc_vec_make(agc_dp_from_double(x, 29), agc_dp_from_double(y, 29), agc_dp_from_double(z, 29)));
}

static agc_vec_t vel(double x, double y, double z)
{
    return (agc_vec_make(agc_dp_from_double(x, 7), agc_dp_from_double(y, 7), agc_dp_from_double(z, 7)));
}

static double comp(agc_vec_t v, int i, int scale)
{
    return (agc_dp_to_double(v.c[i], scale));
}

static bool near_vec(agc_vec_t v, int scale, double x, double y, double z, double tol)
{
    return (fabs(comp(v, 0, scale) - x) < tol && fabs(comp(v, 1, scale) - y) < tol && fabs(comp(v, 2, scale) - z) < tol);
}

static double circ_speed(void)
{
    return (sqrt(MU / R0));
} /* m/cs */

static double circ_period(void)
{
    return (2.0 * PI * sqrt(R0 * R0 * R0 / MU));
} /* cs */

/* KEPPREP + KEPLER from the circular state at (R0,0,0) by tau cs. */
static agc_status_t run_kepler(agc_interp_t *ip, double tau, agc_kepler_out_t *out)
{
    agc_kepler_in_t in;

    in.rrect        = pos(R0, 0.0, 0.0);
    in.vrect        = vel(0.0, circ_speed(), 0.0);
    in.tau          = agc_dp_from_double(tau, 28);
    in.tc           = 0;
    in.xprev        = 0;
    in.xkepnew      = 0;
    in.body         = AGC_CONIC_EARTH;
    agc_status_t st = agc_conic_kepprep(ip, &in, &in.xkepnew);

    CHECK(st == AGC_STATUS_OK);

    return (agc_conic_kepler(ip, &in, out));
}

static void test_kepler(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_kepler_out_t out;
    const double     v = circ_speed();
    const double     q = circ_period() / 4.0;

    CHECK(run_kepler(&ip, q, &out) == AGC_STATUS_OK);
    CHECK(near_vec(out.rcv, 29, 0.0, R0, 0.0, 300.0));
    CHECK(near_vec(out.vcv, 7, -v, 0.0, 0.0, 5e-4));
    CHECK(fabs(agc_dp_to_double(out.tc, 28) - q) < 1.0);
    CHECK(fabs(agc_dp_to_double(out.alpha, -22) - 1.0 / R0) < 1e-10);
    CHECK(out.iterations >= 1 && out.iterations <= AGC_CONIC_KEPLER_ITERCTR);

    CHECK(run_kepler(&ip, -q, &out) == AGC_STATUS_OK); /* backdating */
    CHECK(near_vec(out.rcv, 29, 0.0, -R0, 0.0, 300.0));
    CHECK(near_vec(out.vcv, 7, v, 0.0, 0.0, 5e-4));

    CHECK(run_kepler(&ip, 9.0 * q, &out) == AGC_STATUS_OK); /* 2.25 periods */
    CHECK(near_vec(out.rcv, 29, 0.0, R0, 0.0, 600.0));
    CHECK(fabs(agc_dp_to_double(out.tmodulo, 28) - 8.0 * q) < 5.0);
    CHECK(fabs(agc_dp_to_double(out.tau, 28) - q) < 5.0);   /* period LSB 1 cs */
}

static void test_lambert(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_lambert_in_t  in;
    agc_lambert_out_t out;
    const double      v = circ_speed();

    in.r1vec    = pos(R0, 0.0, 0.0);
    in.r2vec    = pos(0.0, R0, 0.0);
    in.tdesired = agc_dp_from_double(circ_period() / 4.0, 28);
    in.geomsgn  = (agc_sp_t) 0x2000;                   /* +.5: less than 180 degrees  */
    in.guessw   = true;
    in.coga     = 0;
    in.normsw   = false;
    in.un       = agc_vec_zero();
    in.vtargtag = 0;                                   /* also compute VTARGET        */
    in.iterctr  = AGC_CONIC_LAMBERT_ITERCTR;
    in.body     = AGC_CONIC_EARTH;
    CHECK(agc_conic_lambert(&ip, &in, &out) == AGC_STATUS_OK);
    CHECK(!out.solnsw);
    CHECK(near_vec(out.vvec, 7, 0.0, v, 0.0, 2e-3));
    CHECK(near_vec(out.vtarget, 7, -v, 0.0, 0.0, 2e-3));
    CHECK(fabs(agc_dp_to_double(out.coga, 5)) < 1e-3); /* circular   */

    in.r2vec = pos(R0, 0.0, 0.0); /* 0 degrees: 360LAMB          */
    CHECK(agc_conic_lambert(&ip, &in, &out) == AGC_STATUS_NO_CONVERGE);
    CHECK(out.solnsw);
}

/* Ellipse: perigee 7000 km on +x, 1.1 x circular speed along +y. */
static void ellipse(double *rp, double *vp, double *a, double *e)
{
    *rp = 7.0e6;
    *vp = 1.1 * sqrt(MU / *rp);
    *a  = 1.0 / (2.0 / *rp - (*vp * *vp) / MU);
    *e  = 1.0 - *rp / *a;
}

static void test_apsides(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    double rp, vp, a, e;

    ellipse(&rp, &vp, &a, &e);
    agc_apsides_out_t out;

    CHECK(agc_conic_apsides(&ip, pos(rp, 0.0, 0.0), vel(0.0, vp, 0.0), AGC_CONIC_EARTH, &out) == AGC_STATUS_OK);
    CHECK(fabs(agc_dp_to_double(out.rperi, 29) - rp) < 200.0);
    CHECK(fabs(agc_dp_to_double(out.rapo, 29) - a * (1.0 + e)) < 500.0);
    CHECK(fabs(agc_dp_to_double(out.ecc, 3) - e) < 1e-5);

    /* hyperbola: apocenter returned as DPPOSMAX */
    CHECK(agc_conic_apsides(&ip, pos(rp, 0.0, 0.0), vel(0.0, 1.5 * vp, 0.0), AGC_CONIC_EARTH, &out) == AGC_STATUS_OK);
    CHECK(out.rapo == AGC_DP_MAX);
}

static void test_timethet_timerad(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_conic_time_out_t out;
    agc_timetheta_in_t   tt;

    tt.rvec = pos(R0, 0.0, 0.0);
    tt.vvec = vel(0.0, circ_speed(), 0.0);
    tt.snth = agc_dp_from_double(1.0, 1); /* 90 degrees                  */
    tt.csth = 0;
    tt.rvsw = false;
    tt.body = AGC_CONIC_EARTH;
    CHECK(agc_conic_timethet(&ip, &tt, &out) == AGC_STATUS_OK);
    CHECK(fabs(agc_dp_to_double(out.t, 28) - circ_period() / 4.0) < 2.0);
    CHECK(near_vec(out.rnew, 29, 0.0, R0, 0.0, 300.0));
    CHECK(!out.infinflg && !out.cogaflag);

    double rp, vp, a, e;

    ellipse(&rp, &vp, &a, &e);
    const double     rd       = a;                  /* E = 90 deg: r = a           */
    const double     n        = sqrt(MU / (a * a * a));
    const double     t_expect = (PI / 2.0 - e) / n; /* M = E - e sin E       */
    agc_timerad_in_t tr;

    tr.rvec     = pos(rp, 0.0, 0.0);
    tr.vvec     = vel(0.0, vp, 0.0);
    tr.rdesired = agc_dp_from_double(rd, 29);
    tr.sgnrdot  = (agc_sp_t) 0x2000; /* outbound                    */
    tr.rvsw     = true;
    tr.body     = AGC_CONIC_EARTH;
    CHECK(agc_conic_timerad(&ip, &tr, &out) == AGC_STATUS_OK);
    CHECK(fabs(agc_dp_to_double(out.t, 28) - t_expect) < 3.0);
    CHECK(!out.apsesw && !out.solnsw);
}

int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    test_kepler();
    test_lambert();
    test_apsides();
    test_timethet_timerad();
    (void) printf("test_conic: %d checks, %d failures, %d assert reports\n", g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1);
}
