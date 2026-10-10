/*
 * return_to_earth.c -- P37 return to earth (P37_P70.agc), lane p37.
 *
 * 1. Coplanar conic return: CSM 300,000 km from the earth, in the X-Y
 *    plane, desired reentry angle -6.5 deg, minimum fuel (RTEDVD = 0). The
 *    conic search (RTE317..RTE390 with V2T100/GAMDV10/DVCALC/XT1LIM and
 *    TMRAD100) must converge; a double-precision two-body model of the
 *    post-impulse conic V2(T1)/ must reach RCON (about 400,000 ft above the
 *    Fischer ellipsoid) with a flight path angle of -6.5 deg, stay in the
 *    plane, and DV must be a minimum along X(T1).
 * 2. AUGEKUGL: entry at 36,000 fps, -6.5 deg gives the K1/K2 fit range
 *    PHI = 2400 / (6.5 - K2) NM, K2 = 2.4 + .000285 (V - 32000), and
 *    TE = PHI x 720000 cs/rev above 26,000 fps.
 * 3. XT1LIM keeps its guid-entry public form (BETA5 B9).
 */
#include <stdio.h>

#include <math.h>
#include <string.h>

#include "../../src/csm/burns/return_to_earth/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/csm/entry.h"

static int g_checks;
static int g_failures;
static int g_assert_reports;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);
    g_assert_reports++;
    return false;
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

static const double MU  = 3.986032e10; /* m^3/cs^2 (RTMURTE^2)     */
static const double DEG = 1.0 / 360.0; /* revs per degree          */

static double vnorm(const double v[3])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/* Flight path angle (deg, + above horizon) of the conic r1, v at radius r
 * on the descending leg, by the vis-viva and angular-momentum relations. */
static double fpa_at_radius(const double r1[3], const double v[3], double r)
{
    double h[3] = {r1[1] * v[2] - r1[2] * v[1], r1[2] * v[0] - r1[0] * v[2],
                   r1[0] * v[1] - r1[1] * v[0]};
    double en =
        0.5 * (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) - MU / vnorm(r1);
    double vr = sqrt(2.0 * (en + MU / r));
    double c  = vnorm(h) / (r * vr);
    return -acos(c > 1.0 ? 1.0 : c) * 180.0 / 3.14159265358979;
}

static void set_vec(agc_vec_t *out, const double v[3], int scale)
{
    out->c[0] = agc_dp_from_double(v[0], scale);
    out->c[1] = agc_dp_from_double(v[1], scale);
    out->c[2] = agc_dp_from_double(v[2], scale);
}

static void get_vec(double out[3], agc_vec_t v, int scale)
{
    out[0] = agc_dp_to_double(v.c[0], scale);
    out[1] = agc_dp_to_double(v.c[1], scale);
    out[2] = agc_dp_to_double(v.c[2], scale);
}

static void test_coplanar_conic_return(void)
{
    rte_erasable_t *rs    = rte_state();
    agc_interp_t   *ip    = rte_ip();
    const double    r1[3] = {3.0e8, 0.0, 0.0}; /* m                 */
    const double    v1[3] = {-3.0, 6.0, 0.0};  /* m/cs (300, 600 m/s) */
    agc_vec_t       r, v;
    double          v2[3];

    memset(rs, 0, sizeof *rs);
    agc_interp_init(ip);
    set_vec(&r, r1, 29);
    set_vec(&v, v1, 7);
    rs->rtedvd   = 0;                                 /* minimum fuel      */
    rs->rtegam2d = agc_dp_from_double(-6.5 * DEG, 0); /* -6.5 deg          */
    CHECK(rte_initial_vectors(ip, rs, r, v, agc_dp_from_double(1.0e6, 28), 0) ==
          AGC_STATUS_OK);
    CHECK(rte_conic_setup(ip, rs) == AGC_STATUS_OK);
    CHECK(rte_conic_search(ip, rs, 0) == AGC_STATUS_OK);
    get_vec(v2, rs->v2_t1v, 7);
    double rcon = agc_dp_to_double(rs->rcon, 29);
    double gam  = fpa_at_radius(r1, v2, rcon);
    double dv   = agc_dp_to_double(rs->dv, 7);
    (void) printf(
        "conic: RCON %.0f m, V2 (%.4f %.4f %.4f) m/cs, DV %.4f m/cs, "
        "gamma %.3f deg, X(T2) %.5f, T12 %.0f s, iterations %d\n",
        rcon, v2[0], v2[1], v2[2], dv, gam, agc_dp_to_double(rs->x_t2, 0),
        agc_dp_to_double(rs->t12, 28) / 100.0, rs->nn1a + RTE_CONIC_PASSES);
    CHECK(rcon > 6.47e6 && rcon < 6.52e6);
    CHECK(fabs(gam + 6.5) < 0.05);
    CHECK(fabs(v2[2]) < 1e-6);
    CHECK(fabs(agc_dp_to_double(rs->x_t2, 0) -
               tan(-6.5 * 3.14159265358979 / 180.0)) < 2e-3);
    /* minimum DV along X(T1): a step either way costs more (double model) */
    double ur[3] = {1.0, 0.0, 0.0};
    double uh[3] = {0.0, 1.0, 0.0};
    double x     = agc_dp_to_double(rs->x_t1, 5);
    double best  = 1e30;
    for (int k = -1; k <= 1; k++)
    {
        double xk  = x + 0.002 * k;
        double lam = r1[0] / rcon;
        double b1  = 1.0 + pow(agc_dp_to_double(rs->x_t2, 0), 2);
        double p = 2.0 * r1[0] * (lam - 1.0) / (b1 * lam * lam - 1.0 - xk * xk);
        double s = sqrt(MU * p) / r1[0];
        double dvk = hypot(s * xk * ur[0] - v1[0], s * uh[1] - v1[1]);
        if (k == 0)
        {
            CHECK(fabs(dvk - dv) < 1e-3 * dv);
        }
        best = (dvk < best) ? dvk : best;
    }
    CHECK(fabs(best - dv) < 1e-3 * dv);
    CHECK(agc_flag_test(AGC_FLAG_RETROFLG) == false);
}

/* Time critical mode: RTEDVD = 5 m/cs (above the 3.64 m/cs minimum) must
 * be met within EPC10RTE, on the faster (lower X(T1) magnitude) side.    */
static void test_time_critical_return(void)
{
    rte_erasable_t *rs    = rte_state();
    agc_interp_t   *ip    = rte_ip();
    const double    r1[3] = {3.0e8, 0.0, 0.0};
    const double    v1[3] = {-3.0, 6.0, 0.0};
    agc_vec_t       r, v;
    double          v2[3];

    memset(rs, 0, sizeof *rs);
    agc_interp_init(ip);
    set_vec(&r, r1, 29);
    set_vec(&v, v1, 7);
    rs->rtedvd   = agc_dp_from_double(5.0, 7);
    rs->rtegam2d = agc_dp_from_double(-6.5 * DEG, 0);
    CHECK(rte_initial_vectors(ip, rs, r, v, agc_dp_from_double(1.0e6, 28), 0) ==
          AGC_STATUS_OK);
    CHECK(rte_conic_setup(ip, rs) == AGC_STATUS_OK);
    CHECK(rte_conic_search(ip, rs, 0) == AGC_STATUS_OK);
    get_vec(v2, rs->v2_t1v, 7);
    double rcon = agc_dp_to_double(rs->rcon, 29);
    (void) printf("time critical: DV %.5f m/cs, T12 %.0f s, gamma %.3f deg\n",
                  agc_dp_to_double(rs->dv, 7),
                  agc_dp_to_double(rs->t12, 28) / 100.0,
                  fpa_at_radius(r1, v2, rcon));
    CHECK(fabs(agc_dp_to_double(rs->dv, 7) - 5.0) < 2e-3);
    CHECK(agc_dp_to_double(rs->t12, 28) < 2.40e7); /* faster than min fuel */
    CHECK(fabs(fpa_at_radius(r1, v2, rcon) + 6.5) < 0.05);
}

static void test_augekugl(void)
{
    agc_interp_t *ip     = rte_ip();
    agc_dp_t      phie   = 0;
    agc_dp_t      te     = 0;
    double        v      = 36000.0 * 0.3048 / 100.0; /* m/cs              */
    double        k2     = 2.4 + 0.000285 * (36000.0 - 32000.0);
    double        phi_nm = 2400.0 / (6.5 - k2);

    agc_interp_init(ip);
    CHECK(rte_augekugl(ip, agc_dp_from_double(v, 7),
                       agc_dp_from_double(6.5 * DEG, 0), &phie,
                       &te) == AGC_STATUS_OK);
    (void) printf("augekugl: PHIE %.2f NM (fit %.2f), TE %.1f s\n",
                  agc_dp_to_double(phie, 0) * 21600.0, phi_nm,
                  agc_dp_to_double(te, 28) / 100.0);
    CHECK(fabs(agc_dp_to_double(phie, 0) * 21600.0 - phi_nm) < 1.0);
    CHECK(fabs(agc_dp_to_double(te, 28) - agc_dp_to_double(phie, 0) * 7.2e5) <
          2.0);
}

static void test_xt1lim(void)
{
    agc_dp_t x = 0;

    CHECK(agc_return_to_earth_xt1lim(
              agc_dp_from_double(1.4e7, 30), agc_dp_from_double(0.7e7, 29),
              agc_dp_from_double(0.7e7, 29), agc_dp_from_double(5.0, 9),
              &x) == AGC_STATUS_OK);
    CHECK(fabs(agc_dp_to_double(x, 5) - 2.0) < 1e-4);
}

int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    test_coplanar_conic_return();
    test_time_critical_return();
    test_augekugl();
    test_xt1lim();
    (void) printf(
        "test_return_to_earth: %d checks, %d failures, %d assert reports\n",
        g_checks, g_failures, g_assert_reports);
    return (g_failures == 0) ? 0 : 1;
}
