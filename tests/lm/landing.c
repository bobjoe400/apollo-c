/*
 * landing.c -- guid-landing lane unit tests (LM build).
 *
 * 1. THROTTLE (THROTTLE_CONTROL_ROUTINES.agc pp. 793-796): with the
 *    commanded and present thrusts both in the 10%-60% throttleable band,
 *    the pulse count sent to the THRUST counter is FC - FP in throttle
 *    pulses (BITPERF = 0.0797959872 pulse/N), PIFPSET is 0, and the
 *    fixed-region logic (HIGHCRIT) throttles up to FMAXPOS above it.
 * 2. QUADGUID (LUNAR_LANDING_GUIDANCE_EQUATIONS.agc pp. 808-810): a vehicle
 *    exactly on the target trajectory r(T) = RDG + VDG T + ADG T^2/2,
 *    v(T) = VDG + ADG T is commanded ACG = ADG (zero correction); a 10 m
 *    position offset gives the 12 dr / TTF^2 correction.
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
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/lm/landing.h"

/* INTEGRATION: the THRUST counter has no accessor in include/agc yet
 * (include/agc/agc_landing_pending.h declares it for state-flags). Until it does,
 * this test supplies it and records the last command; set
 * LANDING_TEST_FAKE_THRUST to 0 once libagc defines it.                 */
#define LANDING_TEST_FAKE_THRUST 1
#if LANDING_TEST_FAKE_THRUST
void agc_erase_set_thrust(agc_sp_t pulses);
static agc_sp_t g_thrust;

void agc_erase_set_thrust(agc_sp_t pulses)
{
    g_thrust = pulses;
}

#endif

#define BITPERF 0.0797959872 /* throttle pulses per newton            */

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

static void fresh_lm(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    (void) agc_exec_init();
    agc_lm_landing_reset();
}

/* ---- 1. throttle law ---------------------------------------------------- */

/* Acceleration a (m/s^2) as the THROTTLE input, 2^-4 m/cs^2. */
static agc_dp_t accel_dp(double a_ms2)
{
    return agc_dp_from_double(a_ms2 * 1.0e-4, -4);
}

static void test_throttle_band(void)
{
    fresh_lm();
    agc_lm_pad_t *pad = agc_lm_pad();

    pad->lowcrit  = 2853;     /* ~ 82% of FMAXPOS (test values)      */
    pad->highcrit = 3070;
    const double mass = 8000.0; /* kg                                   */

    agc_guid_state()->lm.mass = agc_dp_from_double(mass, 16);
    agc_time_set_cs(50000u);
    agc_lm_thr()->tthrot = (agc_sp_t) (agc_lm_time1() - 1000); /* > 3 s ago */

    /* Commanded 2.0 m/s^2 (16 kN, ~37% of the 43.9 kN maximum); present
     * 1.5 m/s^2 (12 kN, ~27%): both inside the 10%-60% band.          */
    const double a_cmd = 2.0;
    const double a_now = 1.5;

    agc_lm_ldg()->afc = accel_dp(a_cmd);
    agc_status_t st = agc_lm_throttle_measured(accel_dp(a_now));

    CHECK(st == AGC_STATUS_OK);

    const double        fc     = mass * a_cmd * BITPERF; /* 1276.7 pulses        */
    const double        fp     = mass * a_now * BITPERF; /*  957.6 pulses        */
    const agc_lm_thr_t *t      = agc_lm_thr();
    double              fc_got = agc_dp_to_double(t->fcodd, 14);
    double              fp_got = agc_dp_to_double(t->fp, 14);

    CHECK(fabs(fc_got - fc) < 1.0);
    CHECK(fabs(fp_got - fp) < 1.0);
    CHECK(t->pifpset == 0);
    /* Pulses sent = high word of FCODD - FP = 319 (within one pulse).   */
    int expected = (int) lround(fc - fp);

    CHECK(abs((int) t->pseudo55 - expected) <= 1);
#if LANDING_TEST_FAKE_THRUST
    CHECK(g_thrust == t->pseudo55);
#endif
    CHECK(t->fcold == agc_dp_high(t->fcodd));
    (void) printf("throttle: FC %.1f FP %.1f -> %d pulses (expect %d)\n",
                  fc_got, fp_got, (int) t->pseudo55, expected);

    /* Above HIGHCRIT from the throttleable region: throttle-up to the
     * fixed region (FCODD = FMAXPOS, PIFPSET = FEXTRA).                */
    agc_lm_thr()->tthrot = (agc_sp_t) (agc_lm_time1() - 1000);
    agc_lm_thr()->fcold  = 1000;
    agc_lm_ldg()->afc    = accel_dp(5.0); /* 3192 pulses > HIGHCRIT */
    st                   = agc_lm_throttle_measured(accel_dp(a_now));
    CHECK(st == AGC_STATUS_OK);
    CHECK(agc_dp_high(agc_lm_thr()->fcodd) == AGC_LM_FMAXPOS);
    CHECK(agc_lm_thr()->pifpset == AGC_LM_FEXTRA);
}

/* ---- 2. quadratic guidance on the target trajectory --------------------- */

#define CS_PER_S 100.0

/* Set the braking-phase target and the vehicle state at time-to-go T (s,
 * negative) on r = RDG + VDG T + ADG T^2/2 (+ dx_m on x), v = VDG + ADG T. */
static void setup_trajectory(double tgo_s, double dx_m, const double rd[3],
                             const double vd[3], const double ad[3])
{
    agc_lm_pad_t    *pad = agc_lm_pad();
    agc_lm_ldg_t    *l   = agc_lm_ldg();
    agc_lm_target_t *tg  = &pad->target[0];

    pad->leadtime = -1; /* 1 cs: essentially no lag */
    l->wchphase   = AGC_LM_PHASE_BRAKQUAD;
    double tt = tgo_s * CS_PER_S; /* cs                       */

    l->ttf_8 = agc_dp_from_double(tt / 8.0, 14);

    for (int i = 0; i < 3; i++)
    {
        double r = rd[i] + vd[i] * tgo_s + 0.5 * ad[i] * tgo_s * tgo_s;
        double v = vd[i] + ad[i] * tgo_s;

        if (i == 0)
        {
            r += dx_m;
        }

        tg->rfg.c[i] = agc_dp_from_double(rd[i], 24);
        tg->vfg.c[i] = agc_dp_from_double(vd[i] / CS_PER_S, 10);
        tg->afg.c[i] = agc_dp_from_double(ad[i] / 1.0e4, -4);
        l->rgu.c[i]  = agc_dp_from_double(r, 24);
        l->vgu.c[i]  = agc_dp_from_double(v / CS_PER_S, 10);

        /* CG = identity (half-unit rows): UNFC/2 = ACG when GDT/2 = 0. */
        for (int j = 0; j < 3; j++)
        {
            l->cg.row[i].c[j] = (i == j) ? AGC_DP_HALF : 0;
        }
    }

    agc_guid_state()->avg.gdt_2 = agc_vec_zero();
    agc_guid_state()->lm.mass   = agc_dp_from_double(8000.0, 16);
}

static void test_quadguid_on_trajectory(void)
{
    const double rd[3] = {100.0, -50.0, 200.0}; /* m                  */
    const double vd[3] = {-1.0, 0.5, 2.0};      /* m/s                */
    const double ad[3] = {0.5, 0.2, -0.3};      /* m/s^2              */
    const double tgo   = -400.0;                /* s                  */
    agc_interp_t ip;

    fresh_lm();
    setup_trajectory(tgo, 0.0, rd, vd, ad);
    agc_interp_init(&ip);
    agc_status_t st = agc_lm_quadguid(&ip);

    CHECK(st == AGC_STATUS_OK);
    double acg0[3];

    for (int i = 0; i < 3; i++)
    {
        acg0[i] = agc_dp_to_double(agc_lm_ldg()->unfc_2.c[i], -4) * 1.0e4;
        (void) printf(
            "quadguid on-trajectory: ACG[%d] = %+.5f m/s^2 (ADG %+.5f)\n", i,
            acg0[i], ad[i]);
        /* zero correction: ACG = ADG to 1% (lag coefficients of a
         * 1 cs LEADTIME and the 2^-28 word length)                    */
        CHECK(fabs(acg0[i] - ad[i]) <= 0.01 * fabs(ad[i]) + 1.0e-4);
    }

    double mag = sqrt(ad[0] * ad[0] + ad[1] * ad[1] + ad[2] * ad[2]);

    CHECK(fabs(agc_dp_to_double(agc_lm_ldg()->afc, -4) * 1.0e4 - mag) <=
          0.01 * mag);

    /* 10 m off the trajectory in x: correction 12 dx / T^2 back toward it. */
    const double dx = 10.0;

    fresh_lm();
    setup_trajectory(tgo, dx, rd, vd, ad);
    agc_interp_init(&ip);
    st = agc_lm_quadguid(&ip);
    CHECK(st == AGC_STATUS_OK);
    double acgx = agc_dp_to_double(agc_lm_ldg()->unfc_2.c[0], -4) * 1.0e4;
    double corr = -12.0 * dx / (tgo * tgo); /* m/s^2              */

    (void) printf("quadguid 10 m off: dACG x = %+.6f m/s^2 (expect %+.6f)\n",
                  acgx - acg0[0], corr);
    CHECK(fabs((acgx - acg0[0]) - corr) <= 0.05 * fabs(corr) + 2.0e-5);
}

int main(void)
{
    test_throttle_band();
    test_quadguid_on_trajectory();
    (void) printf(
        "test_landing: %d checks, %d failures, %d assertion reports\n",
        g_checks, g_failures, g_assert_reports);

    return (g_failures == 0) ? 0 : 1;
}
