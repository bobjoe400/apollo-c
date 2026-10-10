/*
 * guidance.c -- guid-powered lane unit tests.
 *
 * 1. CALCRVG (SERVICER.agc p.880-881, the average-G state update): with
 *    no PIPA counts (zero delta-V) and no previous gravity term, one 2-s
 *    step (PGUIDE = 200 cs) moves RN1 by VN * 2 s within the 2 m LSB of
 *    the 2^29 m scale, and VN1 = VN + GDT1/2, the half-step lunar gravity
 *    (mu dt / 2 r^2) at RN1.
 * 2. S40.8 cross-product steering (P40-P47.agc p.719-722): with BDT = 0
 *    and the measured delta-V (DELVREF) along the velocity to be gained
 *    (thrust along VG), the commanded rate OMEGAC is zero; with VG at 90
 *    degrees to the thrust it is not.
 *
 * AGC_ASSERT failures are counted by this file's agc_assert_report().
 */
#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/word.h"
#include "agc/guidance/guidance.h"

#define MU_MOON 4.9027779e12 /* m^3/s^2 (MUM in the AGC)               */

static int g_checks;
static int g_failures;
static int g_assert_reports;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d: %s\n", file, line, expr);
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

/* REFSMMAT = identity, stored as a half-unit matrix (scale 2^1). */
static agc_mat_t identity_half(void)
{
    agc_mat_t m;
    m.row[0] = agc_vec_make(AGC_DP_HALF, 0, 0);
    m.row[1] = agc_vec_make(0, AGC_DP_HALF, 0);
    m.row[2] = agc_vec_make(0, 0, AGC_DP_HALF);
    return m;
}

/* ---- 1. average-G with zero delta-V ----------------------------------- */

static void test_average_g_coast(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_avg_g_t a            = {0};
    a.pguide                 = 200;            /* 2 s, cs at 2^28    */
    a.gdt_2                  = agc_vec_zero(); /* no previous gravity */
    const double       r0    = 1.85e6;         /* m, ~110 km altitude */
    const double       vx    = 16.3;           /* m/cs (1630 m/s)    */
    agc_state_vector_t sv    = {agc_vec_zero(), agc_vec_zero(), 0, true};
    sv.r                     = agc_vec_make(0, agc_dp_from_double(r0, 29), 0);
    sv.v                     = agc_vec_make(agc_dp_from_double(vx, 7), 0, 0);
    const agc_mat_t refsmmat = identity_half();

    agc_status_t st = agc_guid_calcrvg(&ip, AGC_VEHICLE_LM, true, &refsmmat,
                                       agc_vec_zero(), &sv, &a);
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
    CHECK(fabs(vx1 - (vx + gx)) <=
          0.002 * g_half + ldexp(1.0, 7 - AGC_DP_BITS) * 4.0);
}

/* ---- 2. cross-product steering ------------------------------------------ */

static agc_vec_t steer(agc_vec_t vg, agc_vec_t delvref)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_cdutrig_t  trig   = {{0}, {0}, {0}, {0}};
    const uint16_t cdu[3] = {0U, 0U, 0U};
    agc_guid_cdtrigs(&trig, cdu); /* all gimbals at 0   */
    const agc_mat_t refsmmat = identity_half();
    agc_xprod_in_t  in;
    in.vg               = vg;
    in.delvref          = delvref;
    in.bdt              = agc_vec_zero();
    in.csteer           = AGC_DP_HALF;                /* CSTEER = 1, 2^1    */
    in.kprimedt         = agc_dp_from_double(0.1, 0); /* arbitrary gain     */
    agc_vec_t    omegac = agc_vec_zero();
    agc_status_t st =
        agc_guid_steering_cross_product(&ip, &in, &refsmmat, &trig, &omegac);
    CHECK(st == AGC_STATUS_OK);
    return omegac;
}

static void test_cross_product_steering(void)
{
    /* VG 300 m/s and 2 s of thrust (6 m/s) both along SM (1, 2, 2)/3.     */
    const double   ux = 1.0 / 3.0, uy = 2.0 / 3.0, uz = 2.0 / 3.0;
    agc_vec_t      vg  = agc_vec_make(agc_dp_from_double(3.0 * ux, 7),
                                      agc_dp_from_double(3.0 * uy, 7),
                                      agc_dp_from_double(3.0 * uz, 7));
    agc_vec_t      dv  = agc_vec_make(agc_dp_from_double(0.06 * ux, 7),
                                      agc_dp_from_double(0.06 * uy, 7),
                                      agc_dp_from_double(0.06 * uz, 7));
    agc_vec_t      om  = steer(vg, dv);
    const agc_dp_t tol = 64; /* a few unit-vector LSBs */
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        CHECK(labs((long) om.c[i]) <= (long) tol);
    }

    /* Thrust along SM X, VG along SM Y: a rate about the third axis.       */
    agc_vec_t vg2 = agc_vec_make(0, agc_dp_from_double(3.0, 7), 0);
    agc_vec_t dv2 = agc_vec_make(agc_dp_from_double(0.06, 7), 0, 0);
    agc_vec_t om2 = steer(vg2, dv2);
    long      mag =
        labs((long) om2.c[0]) + labs((long) om2.c[1]) + labs((long) om2.c[2]);
    CHECK(mag > 1000L * (long) tol);
}

int main(void)
{
    test_average_g_coast();
    test_cross_product_steering();
    (void) printf(
        "test_guidance: %d checks, %d failures, %d assertion reports\n",
        g_checks, g_failures, g_assert_reports);
    return (g_failures == 0) ? 0 : 1;
}
