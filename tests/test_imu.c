/*
 * test_imu.c -- imu lane unit tests (CSM build unless noted).
 *
 * Covers: ARCTRIG quadrants; CALCGTA on an identity XDC/YDC/ZDC gives zero
 * gyro torquing angles; CALCGA gimbal-lock check (alarm 00401 / GLOKFAIL)
 * on a known attitude and its absence on a benign one; 1/PIPA with zero
 * bias and scale factor leaves DELV unchanged; CDUTODCM/DCMTOCDU round
 * trip; DELCOMP of a zero angle is the identity.
 *
 * AGC_ASSERT failures are counted by this file's agc_assert_report().
 */
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define PI_D 3.14159265358979323846
#include "agc/core/assert.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/core/flags_csm.h"
#include "agc/imu/imu.h"

static int g_checks;
static int g_failures;
static int g_assert_reports;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void)file;
    (void)line;
    (void)expr;
    g_assert_reports++;
    return false;
}

#define CHECK(c) check((c), __LINE__, #c)

static void check(bool ok, int line, const char *text)
{
    g_checks++;
    if (!ok) {
        g_failures++;
        (void)printf("FAIL line %d: %s\n", line, text);
    }
}

static bool near(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

static double rev(agc_dp_t v)
{
    return agc_dp_to_double(v, 0);
}

/* CDU pattern (2^15 per revolution) for an angle in degrees.             */
static uint16_t cdu_deg(double deg)
{
    long v = lround(deg / 360.0 * 32768.0);
    return (uint16_t)((unsigned long)v & 077777u);
}

static double cdu_to_deg(uint16_t p)
{
    int v = (p & 040000u) ? (int)p - 0100000 : (int)p;
    return (double)v * 360.0 / 32768.0;
}

static void test_arctrig(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    const double angs[] = { 10.0, 60.0, 120.0, 170.0, -30.0, -100.0, -160.0 };
    for (int i = 0; i < 7; i++) {
        double r = angs[i] * PI_D / 180.0;
        agc_dp_t s4 = agc_dp_from_double(sin(r) / 4.0, 0);
        agc_dp_t c4 = agc_dp_from_double(cos(r) / 4.0, 0);
        double got = rev(agc_imu_arctrig(&ip, s4, c4)) * 360.0;
        CHECK(near(got, angs[i], 0.01));
    }
}

static void test_calcgta_identity(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_mat_t dc = agc_mat_identity_half();      /* desired = present SM  */
    agc_dp_t gyro[3] = { 1, 1, 1 };
    CHECK(agc_imu_calcgta(&ip, &dc, gyro) == AGC_STATUS_OK);
    CHECK(gyro[0] == 0 && gyro[1] == 0 && gyro[2] == 0);
    agc_dp_t buf[3];
    CHECK(agc_imu_gyro_buf_get(AGC_IMU_GBUF_OGC, buf) == AGC_STATUS_OK);
    CHECK(buf[0] == 0 && buf[1] == 0 && buf[2] == 0);
}

/* Navigation base rotated by angle deg about the SM Z axis (middle gimbal):
 * XNB = (cos, sin, 0), YNB = (-sin, cos, 0), ZNB = (0, 0, 1), half-unit.  */
static agc_mat_t nb_about_z(double deg)
{
    double r = deg * PI_D / 180.0;
    agc_mat_t m;
    m.row[0] = agc_vec_make(agc_dp_from_double(cos(r) / 2, 0),
                            agc_dp_from_double(sin(r) / 2, 0), 0);
    m.row[1] = agc_vec_make(agc_dp_from_double(-sin(r) / 2, 0),
                            agc_dp_from_double(cos(r) / 2, 0), 0);
    m.row[2] = agc_vec_make(0, 0, AGC_DP_HALF);
    return m;
}

static void test_calcga_gimbal_lock(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_mat_t sm = agc_mat_identity_half();
    uint16_t thetad[3];
    (void)agc_flag_clear(AGC_FLAG_GLOKFAIL);
    (void)agc_flag_clear(AGC_FLAG_CPHIFLAG);
    agc_mat_t nb = nb_about_z(30.0);              /* MGA 30 deg: no lock  */
    CHECK(agc_imu_calcga(&ip, &nb, &sm, thetad) == AGC_STATUS_OK);
    CHECK(!agc_flag_test(AGC_FLAG_GLOKFAIL));
    CHECK(near(cdu_to_deg(thetad[2]), 30.0, 0.05));
    CHECK(near(cdu_to_deg(thetad[0]), 0.0, 0.05));
    CHECK(near(cdu_to_deg(thetad[1]), 0.0, 0.05));
    CHECK(agc_erase_thetad(2) == thetad[2]);
    nb = nb_about_z(75.0);                        /* MGA 75 deg: lock     */
    CHECK(agc_imu_calcga(&ip, &nb, &sm, thetad) == AGC_STATUS_OK);
    CHECK(agc_flag_test(AGC_FLAG_GLOKFAIL));
    CHECK(near(cdu_to_deg(thetad[2]), 75.0, 0.05));
}

static void test_pipa_zero_comp(void)
{
    agc_imu_comp_params_t p = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },
                                { 0, 0, 0 }, { 0, 0, 0 } };
    CHECK(agc_imu_comp_load(&p) == AGC_STATUS_OK);
    CHECK(agc_imu_gcompzer() == AGC_STATUS_OK);
    agc_imu_set_pipa_interval((agc_sp_t)(200 * 64));   /* 2 s, cs x 2^8        */
    agc_vec_t in = agc_vec_make(agc_dp_pack(123, 0), agc_dp_pack(-45, 0),
                                agc_dp_pack(7000, 0));
    agc_vec_t delv = in;
    CHECK(agc_imu_one_over_pipa(&delv) == AGC_STATUS_OK);
    for (int i = 0; i < 3; i++) {
        CHECK(delv.c[i] == in.c[i]);
    }
    agc_dp_t g[3];
    CHECK(agc_imu_gyro_buf_get(AGC_IMU_GBUF_GCOMP, g) == AGC_STATUS_OK);
    CHECK(g[0] == 0 && g[1] == 0 && g[2] == 0);   /* no gyro compensation */
    CHECK(agc_imu_gcompsw() == 0);
}

static void test_dcm_round_trip(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    const uint16_t cdu[3] = { cdu_deg(20.0), cdu_deg(-35.0), cdu_deg(15.0) };
    agc_mat_t m;
    CHECK(agc_kalc_cdutodcm(&ip, cdu, &m) == AGC_STATUS_OK);
    agc_vec_t ang;
    agc_dp_t cmga;
    CHECK(agc_kalc_dcmtocdu(&ip, &m, &ang, &cmga) == AGC_STATUS_OK);
    CHECK(near(rev(ang.c[0]) * 360.0, 20.0, 0.01));
    CHECK(near(rev(ang.c[1]) * 360.0, -35.0, 0.01));
    CHECK(near(rev(ang.c[2]) * 360.0, 15.0, 0.01));
    CHECK(near(agc_dp_to_double(cmga, 0), cos(15.0 * PI_D / 180.0), 1e-4));
}

static void test_delcomp_zero(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_vec_t u = agc_vec_make(0, 0, AGC_DP_HALF);
    agc_mat_t del;
    CHECK(agc_kalc_delcomp(&ip, u, 0, &del) == AGC_STATUS_OK);
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            double want = (r == c) ? 1.0 : 0.0;
            CHECK(near(agc_dp_to_double(del.row[r].c[c], 0), want, 1e-6));
        }
    }
}

static void test_gyro_pulse(void)
{
    /* Y gyro: 1000 pulses and 50/128 of a pulse; X and Z nothing.        */
    const agc_dp_t cmd[3] = { 0, 1000 * 128 + 50, 0 };
    CHECK(agc_imu_pulse(cmd) == AGC_STATUS_OK);
    CHECK(agc_imu_gyros_busy());
    CHECK(agc_task_strtgyro() == AGC_STATUS_OK);       /* phase Y          */
    CHECK(agc_imu_gyrocmd() == 1000u);                 /* +.215 pulse: 78/128 */
    agc_dp_t buf[3];
    CHECK(agc_imu_gyro_buf_get(AGC_IMU_GBUF_OGC, buf) == AGC_STATUS_OK);
    CHECK(buf[1] == 78);                               /* unsent fraction  */
    CHECK(agc_imu_gyros_busy());
    CHECK(agc_task_strtgyro() == AGC_STATUS_OK);       /* Z, X: nothing    */
    CHECK(!agc_imu_gyros_busy());
}

static void test_kalc_compute(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    const uint16_t b[3] = { 0, 0, 0 };
    const uint16_t f[3] = { 0, 0, cdu_deg(30.0) };     /* 30 deg about Z   */
    agc_kalc_setup_t k;
    CHECK(agc_kalc_compute(&ip, b, f, &k) == AGC_STATUS_OK);
    CHECK(k.kcase == AGC_KALC_NORMAL);
    CHECK(near(rev(k.am) * 360.0, 30.0, 0.01));
    CHECK(near(agc_dp_to_double(k.cof.c[2], 0), 0.5, 1e-4));
    CHECK(near(agc_dp_to_double(k.cof.c[0], 0), 0.0, 1e-4));
    const uint16_t g[3] = { cdu_deg(0.1), 0, 0 };      /* < .25 deg        */
    CHECK(agc_kalc_compute(&ip, b, g, &k) == AGC_STATUS_OK);
    CHECK(k.kcase == AGC_KALC_SMALL);
}

static void test_vecpoint(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_erase_set_cdux(0);
    agc_erase_set_cduy(0);
    agc_erase_set_cduz(0);
    double r = 30.0 * PI_D / 180.0;
    agc_vec_t pv = agc_vec_make(agc_dp_from_double(cos(r) / 2, 0), 0,
                                agc_dp_from_double(-sin(r) / 2, 0));
    agc_vec_t sx = agc_vec_make(AGC_DP_HALF, 0, 0);    /* point +X         */
    uint16_t g[3];
    CHECK(agc_imu_vecpoint(&ip, pv, sx, g) == AGC_STATUS_OK);
    CHECK(near(cdu_to_deg(g[0]), 0.0, 0.05));
    CHECK(near(cdu_to_deg(g[1]), 30.0, 0.05));
    CHECK(near(cdu_to_deg(g[2]), 0.0, 0.05));
}

int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    (void)agc_exec_init();
    (void)agc_waitlist_init();
    (void)agc_imu_mode_init();
    test_arctrig();
    test_calcgta_identity();
    test_calcga_gimbal_lock();
    test_pipa_zero_comp();
    test_dcm_round_trip();
    test_delcomp_zero();
    test_gyro_pulse();
    test_kalc_compute();
    test_vecpoint();
    (void)printf("test_imu: %d checks, %d failures, %d assertion reports\n",
                 g_checks, g_failures, g_assert_reports);
    return (g_failures == 0) ? 0 : 1;
}
