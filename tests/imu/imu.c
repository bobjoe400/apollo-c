/*
 * imu.c -- imu lane unit tests (CSM build unless noted).
 *
 * Covers: ARCTRIG quadrants; CALCGTA on an identity XDC/YDC/ZDC gives zero
 * gyro torquing angles; CALCGA gimbal-lock check (alarm 00401 / GLOKFAIL)
 * on a known attitude and its absence on a benign one; 1/PIPA with zero
 * bias and scale factor leaves DELV unchanged; CDUTODCM/DCMTOCDU round
 * trip; DELCOMP of a zero angle is the identity; P03 optical verification
 * (V65) end to end with marks taken on a perfectly aligned IMU gives gyro
 * angles near zero and adds them to ERCOMP1; PIPFREE's alarm path
 * releases its interrupt inhibit.
 *
 * AGC_ASSERT failures are counted by this file's agc_assert_report().
 */
#include <stdio.h>

#include <math.h>
#include <stdint.h>

#define PI_D 3.14159265358979323846
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"

static int g_checks;
static int g_failures;
static int g_assert_reports;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) file;
    (void) line;
    (void) expr;
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

static bool near(double a, double b, double tol)
{
    return (fabs(a - b) <= tol);
}

static double rev(agc_dp_t v)
{
    return (agc_dp_to_double(v, 0));
}

/* CDU pattern (2^15 per revolution) for an angle in degrees.             */
static uint16_t cdu_deg(double deg)
{
    long v = lround(deg / 360.0 * 32768.0);

    return ((uint16_t) ((unsigned long) v & 077777u));
}

static double cdu_to_deg(uint16_t p)
{
    int v = (p & 040000u) ? (int) p - 0100000 : (int) p;

    return ((double) v * 360.0 / 32768.0);
}

static void test_arctrig(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    const double angs[] = {10.0, 60.0, 120.0, 170.0, -30.0, -100.0, -160.0};

    for (int i = 0; i < 7; i++)
    {
        double   r   = angs[i] * PI_D / 180.0;
        agc_dp_t s4  = agc_dp_from_double(sin(r) / 4.0, 0);
        agc_dp_t c4  = agc_dp_from_double(cos(r) / 4.0, 0);
        double   got = rev(agc_imu_arctrig(&ip, s4, c4)) * 360.0;

        CHECK(near(got, angs[i], 0.01));
    }
}

static void test_calcgta_identity(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_mat_t dc      = agc_mat_identity_half(); /* desired = present SM  */
    agc_dp_t  gyro[3] = {1, 1, 1};

    agc_imu_calcgta(&ip, &dc, gyro);
    CHECK(gyro[0] == 0 && gyro[1] == 0 && gyro[2] == 0);
    agc_dp_t buf[3];

    CHECK(agc_imu_gyro_buf_get(AGC_IMU_GBUF_OGC, buf) == AGC_STATUS_OK);
    CHECK(buf[0] == 0 && buf[1] == 0 && buf[2] == 0);
}

/* Navigation base rotated by angle deg about the SM Z axis (middle gimbal):
 * XNB = (cos, sin, 0), YNB = (-sin, cos, 0), ZNB = (0, 0, 1), half-unit.  */
static agc_mat_t nb_about_z(double deg)
{
    double    r = deg * PI_D / 180.0;
    agc_mat_t m;

    m.row[0] = agc_vec_make(agc_dp_from_double(cos(r) / 2, 0),
                            agc_dp_from_double(sin(r) / 2, 0), 0);
    m.row[1] = agc_vec_make(agc_dp_from_double(-sin(r) / 2, 0),
                            agc_dp_from_double(cos(r) / 2, 0), 0);
    m.row[2] = agc_vec_make(0, 0, AGC_DP_HALF);

    return (m);
}

static void test_calcga_gimbal_lock(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_mat_t sm = agc_mat_identity_half();
    uint16_t  thetad[3];

    agc_flag_clear(AGC_FLAG_GLOKFAIL);
    agc_flag_clear(AGC_FLAG_CPHIFLAG);
    agc_mat_t nb = nb_about_z(30.0); /* MGA 30 deg: no lock  */

    CHECK(agc_imu_calcga(&ip, &nb, &sm, thetad) == AGC_STATUS_OK);
    CHECK(!agc_flag_test(AGC_FLAG_GLOKFAIL));
    CHECK(near(cdu_to_deg(thetad[2]), 30.0, 0.05));
    CHECK(near(cdu_to_deg(thetad[0]), 0.0, 0.05));
    CHECK(near(cdu_to_deg(thetad[1]), 0.0, 0.05));
    CHECK(agc_erase_thetad(2) == thetad[2]);
    nb = nb_about_z(75.0); /* MGA 75 deg: lock     */
    CHECK(agc_imu_calcga(&ip, &nb, &sm, thetad) == AGC_STATUS_OK);
    CHECK(agc_flag_test(AGC_FLAG_GLOKFAIL));
    CHECK(near(cdu_to_deg(thetad[2]), 75.0, 0.05));
}

static void test_pipa_zero_comp(void)
{
    agc_imu_comp_params_t p = {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
    };

    CHECK(agc_imu_comp_load(&p) == AGC_STATUS_OK);
    CHECK(agc_imu_gcompzer() == AGC_STATUS_OK);
    agc_imu_set_pipa_interval((agc_sp_t) (200 * 64)); /* 2 s, cs x 2^8        */
    agc_vec_t in = agc_vec_make(agc_dp_pack(123, 0), agc_dp_pack(-45, 0),
                                agc_dp_pack(7000, 0));
    agc_vec_t delv = in;

    CHECK(agc_imu_one_over_pipa(&delv) == AGC_STATUS_OK);

    for (int i = 0; i < 3; i++)
    {
        CHECK(delv.c[i] == in.c[i]);
    }

    agc_dp_t g[3];

    CHECK(agc_imu_gyro_buf_get(AGC_IMU_GBUF_GCOMP, g) == AGC_STATUS_OK);
    CHECK(g[0] == 0 && g[1] == 0 && g[2] == 0); /* no gyro compensation */
    CHECK(agc_imu_gcompsw() == 0);
}

static void test_dcm_round_trip(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    const uint16_t cdu[3] = {cdu_deg(20.0), cdu_deg(-35.0), cdu_deg(15.0)};
    agc_mat_t      m;

    CHECK(agc_kalc_cdutodcm(&ip, cdu, &m) == AGC_STATUS_OK);
    agc_vec_t ang;
    agc_dp_t  cmga;

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

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            double want = (r == c) ? 1.0 : 0.0;

            CHECK(near(agc_dp_to_double(del.row[r].c[c], 0), want, 1e-6));
        }
    }
}

static void test_gyro_pulse(void)
{
    /* Y gyro: 1000 pulses and 50/128 of a pulse; X and Z nothing.        */
    const agc_dp_t cmd[3] = {0, 1000 * 128 + 50, 0};

    CHECK(agc_imu_pulse(cmd) == AGC_STATUS_OK);
    CHECK(agc_imu_gyros_busy());
    CHECK(agc_task_strtgyro() == AGC_STATUS_OK); /* phase Y          */
    CHECK(agc_imu_gyrocmd() == 1000u);           /* +.215 pulse: 78/128 */
    agc_dp_t buf[3];

    CHECK(agc_imu_gyro_buf_get(AGC_IMU_GBUF_OGC, buf) == AGC_STATUS_OK);
    CHECK(buf[1] == 78); /* unsent fraction  */
    CHECK(agc_imu_gyros_busy());
    CHECK(agc_task_strtgyro() == AGC_STATUS_OK); /* Z, X: nothing    */
    CHECK(!agc_imu_gyros_busy());
}

static void test_kalc_compute(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    const uint16_t   b[3] = {0, 0, 0};
    const uint16_t   f[3] = {0, 0, cdu_deg(30.0)}; /* 30 deg about Z   */
    agc_kalc_setup_t k;

    CHECK(agc_kalc_compute(&ip, b, f, &k) == AGC_STATUS_OK);
    CHECK(k.kcase == AGC_KALC_NORMAL);
    CHECK(near(rev(k.am) * 360.0, 30.0, 0.01));
    CHECK(near(agc_dp_to_double(k.cof.c[2], 0), 0.5, 1e-4));
    CHECK(near(agc_dp_to_double(k.cof.c[0], 0), 0.0, 1e-4));
    const uint16_t g[3] = {cdu_deg(0.1), 0, 0}; /* < .25 deg        */

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
    double    r  = 30.0 * PI_D / 180.0;
    agc_vec_t pv = agc_vec_make(agc_dp_from_double(cos(r) / 2, 0), 0,
                                agc_dp_from_double(-sin(r) / 2, 0));
    agc_vec_t sx = agc_vec_make(AGC_DP_HALF, 0, 0); /* point +X         */
    uint16_t  g[3];

    CHECK(agc_imu_vecpoint(&ip, pv, sx, g) == AGC_STATUS_OK);
    CHECK(near(cdu_to_deg(g[0]), 0.0, 0.05));
    CHECK(near(cdu_to_deg(g[1]), 30.0, 0.05));
    CHECK(near(cdu_to_deg(g[2]), 0.0, 0.05));
}

/* LASTBIAS, CM: BANKCALL PIPUSE (Comanche p.306) zeroes the PIPAs before
 * the last bias compensation (the LM calls PIPUSE1 and keeps them).     */
static void test_lastbias_pipuse(void)
{
    agc_job_ctx_t ctx = {AGC_JOB_LASTBIAS, AGC_PRIO5, false, -1, 0};

    agc_erase_set_pipax(5);
    agc_erase_set_pipaz(-3);
    agc_imu_set_gcompsw(0);
    (void) agc_job_lastbias(&ctx);
    CHECK(agc_erase_pipax() == 0 && agc_erase_pipaz() == 0);
}

/* PIPFREE with PIPA fail present (IMODES30 bit 10 = 0): ALARM 212, and
 * the routine's INHINT is released (the AGC's second INHINT after ALARM
 * must not leave interrupts off in C).                                  */
static void test_pipfree_alarm(void)
{
    agc_erase_set_imodes30((uint16_t) (agc_erase_imodes30() & ~01000u));
    (void) agc_erase_set_failreg(0, 0);
    agc_imu_pipfree();
    CHECK(!agc_interrupts_inhibited());
    CHECK(agc_erase_failreg(0) == 00212u);
    CHECK((agc_erase_imodes30() & 1u) != 0u); /* PIPAs not in use */
}

/* P03 mark on the aligned IMU: optics at the commanded angles. */
static void p03_mark(const uint16_t imu[3], agc_job_ctx_t *ctx)
{
    const agc_align_ram_t *ar   = agc_align_ram();
    agc_job_ctx_t          crew = {AGC_JOB_ENTANSWR, AGC_PRIO5, false, -1, 0};

    agc_erase_set_cduy(imu[1]);
    agc_erase_set_cduz(imu[2]);
    agc_erase_set_cdux(imu[0]);
    agc_erase_set_cdus(ar->desopts);
    agc_erase_set_cdut(ar->desoptt);
    CHECK(agc_chan_set_signal(016, AGC_CH016_MARK, true) == AGC_STATUS_OK);
    CHECK(agc_sxtmark_markrupt() == AGC_STATUS_OK);
    CHECK(agc_chan_set_signal(016, AGC_CH016_MARK, false) == AGC_STATUS_OK);
    (void) agc_sxtmark_mark2();
    CHECK(agc_sxtmark_marks_made() == 1);
    CHECK(agc_job_entanswr(&crew) == AGC_STATUS_OK); /* V33 on V50N25 */
    CHECK(agc_task_endmarks() == AGC_STATUS_OK);      /* GOODEND        */
    CHECK(agc_job_optical_verification_marked(ctx) == AGC_STATUS_OK);
}

/* P03 (GCOMPVER .. GCOMP5) with the SM at the earth-reference frame. */
static void test_optical_verification(void)
{
    agc_job_ctx_t  ctx = {AGC_JOB_COMPVER, AGC_PRIO5, false, -1, 0};
    agc_interp_t   ip;
    uint16_t       imu[3];
    agc_dp_t       erc[3] = {0, 0, 0};
    const agc_dp_t az     = agc_dp_from_double(0.1, 0); /* 36 deg     */

    agc_interp_init(&ip);
    CHECK(agc_dsky_init() == AGC_STATUS_OK);
    CHECK(agc_csm_prelaunch_set_pad(agc_dp_from_double(0.08, 0), az, az) ==
          AGC_STATUS_OK);
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, erc);
    CHECK(agc_job_optical_verification(&ctx) == AGC_STATUS_OK);
    CHECK(agc_phase_checkmm(3));
    CHECK(agc_dsky_dsptem(2) == 1u); /* target 1 shown */
    CHECK(agc_dsky_set_dsptem(0, cdu_deg(20.0)) == AGC_STATUS_OK);
    CHECK(agc_dsky_set_dsptem(1, 0546u) == AGC_STATUS_OK); /* EL 2 deg */
    CHECK(agc_job_optical_verification_target_loaded(&ctx) == AGC_STATUS_OK);
    CHECK(agc_dsky_dsptem(2) == 2u); /* target 2 shown */
    CHECK(agc_dsky_set_dsptem(0, cdu_deg(60.0)) == AGC_STATUS_OK);
    CHECK(agc_dsky_set_dsptem(1, 01326u) == AGC_STATUS_OK); /* EL 5 deg */

    agc_align_ram_t *ar = agc_align_ram();

    ar->xsm = agc_mat_identity_half(); /* SM = earth reference         */
    CHECK(agc_imu_calcga(&ip, &ar->xnb, &ar->xsm, imu) == AGC_STATUS_OK);
    CHECK(agc_job_optical_verification_target_loaded(&ctx) == AGC_STATUS_OK);
    CHECK(ar->starcode == 1);
    p03_mark(imu, &ctx); /* target 1: then drives to target 2 */
    CHECK(ar->starcode == 2);
    p03_mark(imu, &ctx); /* target 2: AXISGEN, CALCGTA, V06N93 */

    for (int k = 0; k < 3; k++)
    {
        /* the marks carry the CDU quantisation: 2^-15 rev = 40 arcsec */
        CHECK(near(rev(ar->ogc[k]), 0.0, 5e-5));
    }

    CHECK(agc_job_optical_verification_torque_accepted(&ctx) ==
          AGC_STATUS_OK);
    CHECK(agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, erc) == AGC_STATUS_OK);
    CHECK(erc[0] == ar->ogc[0] && erc[2] == ar->ogc[2]);
    CHECK(agc_phase_checkmm(2)); /* GCOMP5: back to P02 */
}

int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    (void) agc_exec_init();
    (void) agc_waitlist_init();
    (void) agc_imu_mode_init();
    test_arctrig();
    test_calcgta_identity();
    test_calcga_gimbal_lock();
    test_pipa_zero_comp();
    test_dcm_round_trip();
    test_delcomp_zero();
    test_gyro_pulse();
    test_kalc_compute();
    test_vecpoint();
    test_pipfree_alarm();
    test_lastbias_pipuse();
    test_optical_verification();
    (void) printf("test_imu: %d checks, %d failures, %d assertion reports\n",
                  g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1);
}
