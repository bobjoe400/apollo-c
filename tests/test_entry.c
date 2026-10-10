/*
 * test_entry.c -- guid-entry unit tests (Comanche entry guidance and the
 * entry DAP roll phase plane).
 *
 * 1. HUNTEST / RANGER for a nominal lunar-return entry state in the
 *    up-control phase (V = 32000 fps, RDOT = -300 fps, D = 4 g, L/D = .3)
 *    finds an up-control solution and predicts a finite, plausible range.
 * 2. The CM/RCS roll phase plane (pp. 1081-1086) fires the positive roll
 *    jets (JNDX = -1) for a positive roll error and the negative ones for
 *    a negative error, with no rate; a tiny error falls in the dead zone.
 * 3. XT1LIM (P37, p. 930) gives sqrt(BETA10) for a simple case.
 *
 * Scales: /2VS = /51532.3946 fps, /805 = /25 g, range revs = /21600 NM.
 */
#include <math.h>
#include <stdio.h>
#include "agc/agc_assert.h"
#include "agc/agc_state.h"
#include "agc/agc_csm_entry.h"

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

static const double TWO_VS_FPS = 51532.3946;   /* 2 VSAT, ft/s            */
static const double G25_FPSS   = 805.0;        /* 25 g, ft/s^2            */
static const double REV_NM     = 21600.0;      /* NM per rev of range     */

/* Nominal state at the start of HUNTEST (after KATEST, P64).             */
static void set_nominal_upcontrol_state(void)
{
    agc_entry_state_t *es = agc_entry_state();
    double             v = 32000.0 / TWO_VS_FPS;

    agc_entry_lexicon_reset();
    es->v       = agc_dp_from_double(v, 0);
    es->vsquare = agc_dp_from_double(v * v, 0);     /* (V/VS)^2 / 4 = v^2 */
    es->rdot    = agc_dp_from_double(-300.0 / TWO_VS_FPS, 0);
    es->d       = agc_dp_from_double(4.0 * 32.2 / G25_FPSS, 0);
    es->lad     = agc_dp_from_double(0.3, 0);
    es->l_d     = es->lad;                           /* lift up            */
    es->lewd    = agc_dp_from_double(0.15, 0);       /* LEWD1              */
    es->q7      = agc_dp_from_double(6.0 / G25_FPSS, 0);   /* Q7F          */
    es->q2      = agc_dp_from_double(0.3 * 500.0 / REV_NM - 1152.0 / REV_NM, 0);
    es->thetah  = agc_dp_from_double(1300.0 / REV_NM, 0);
}

/* Double-precision model of HUNTEST .. RANGER (pp. 855-862) with the
 * REENTRY_CONTROL constants, for the nominal state above; returns the
 * predicted range in revs (ASKEP + ASP1 + ASPUP + ASPDWN + ASP3).        */
static double model_range_revs(double *vl_out, double *gammal_out)
{
    const double v = 32000.0 / TWO_VS_FPS, vsq = v * v;
    const double rdot = -300.0 / TWO_VS_FPS, d = 4.0 * 32.2 / G25_FPSS;
    const double lad = 0.3, lewd = 0.15, q7 = 6.0 / G25_FPSS;
    const double q2 = 0.3 * 500.0 / REV_NM - 1152.0 / REV_NM;
    const double two_c1_hs = .0215983264, chook = 1.0 / 64.0;
    const double ch1 = .64;                 /* CH1 2DEC .32 B1 = 16/25    */
    double v1 = rdot / lad + v;                         /* RDOT < 0: LAD  */
    double a0 = (rdot * rdot / lad / two_c1_hs + d) * v1 * v1 / vsq;
    double alp = a0 * two_c1_hs / v1 / v1 / lewd;
    double fact1 = v1 / (1.0 - alp), fact2 = (alp - 1.0) * alp / a0;
    double vl = (1.0 - sqrt(fact2 * q7 + alp)) * fact1;
    double gl1 = lewd * (v1 - vl) / vl;
    double vbars = vl * vl, dvl = 0.5 - vl, vs1 = 0.5;
    double dhook, ahookdv, g, q7n = q7, cosg, e4, askep, aspup, aspdwn;

    if (v1 > 0.5) {
        dvl = v1 - vl;
        vs1 = v1;
    }
    dhook = (pow(1.0 - vs1 / fact1, 2) - alp) / fact2;
    ahookdv = dhook / 64.0 / q7 - chook;
    g = gl1 - (ahookdv + 1.0 / 16.0) * ch1 * dvl * dvl / dhook / vbars;
    if (g < 0) {                                        /* NEGAMA         */
        vl += (g * vl / 3.0) /
              (lewd / 3.0 - (ahookdv + 1.0 / 24.0) * dvl * ch1 / dhook / vl);
        q7n = (pow(1.0 - vl / fact1, 2) - alp) / fact2;
        vbars = vl * vl;
        g = 0;
    }
    gl1 += (g - gl1) * 0.5;                             /* Q19            */
    cosg = 0.5 - g * g / 4.0;
    e4 = sqrt(4.0 * (vbars - 0.5) * vbars * cosg * cosg + 1.0 / 16.0);
    askep = 2.0 * asin(vbars * cosg * g / e4) / 6.283185307179586;
    aspup = (-log(v1 * v1 * q7n / vbars / a0) / 32.0) * .00684572901 / gl1;
    aspdwn = -.0247622232 * rdot * v / a0 / lad;
    *vl_out = vl;
    *gammal_out = g;
    return askep + (vl * .167003132 + q2) + aspup + aspdwn +
           (.0349 - g) * .326388889;
}

static void test_huntest_range(void)
{
    agc_entry_hunt_result_t r = AGC_ENTRY_HUNT_CONSTD;
    agc_dp_t                gammal = 0;
    agc_dp_t                diff = 0;
    const agc_entry_state_t *es;
    double                  range_nm;
    double                  model_vl = 0;
    double                  model_g = 0;
    double                  model_nm = model_range_revs(&model_vl, &model_g) * REV_NM;

    set_nominal_upcontrol_state();
    CHECK(agc_entry_huntest_predict(&r, &gammal) == AGC_STATUS_OK);
    CHECK(r == AGC_ENTRY_HUNT_RANGER);
    es = agc_entry_state();
    CHECK(fabs(agc_dp_to_double(es->vl, 0) - model_vl) < 1e-5);  /* .5 fps */
    CHECK(fabs(agc_dp_to_double(gammal, 0) - model_g) < 1e-6);
    CHECK(agc_dp_to_double(es->vl, 0) * TWO_VS_FPS > 18000.0);  /* > VLMIN */
    CHECK(agc_entry_range_predict(gammal, &diff) == AGC_STATUS_OK);
    range_nm = (agc_dp_to_double(diff, 0) + 1300.0 / REV_NM) * REV_NM;
    (void)printf("  HUNTEST: VL %.0f fps, GAMMAL %.5f rad, range %.1f NM (model %.1f)\n",
                 agc_dp_to_double(es->vl, 0) * TWO_VS_FPS,
                 agc_dp_to_double(gammal, 0), range_nm, model_nm);
    CHECK(isfinite(range_nm));
    CHECK(range_nm > 100.0 && range_nm < 10000.0);
    CHECK(fabs(range_nm - model_nm) < 1.0);
    CHECK(es->diff == diff);
}

static void test_roll_phase_plane(void)
{
    agc_entry_roll_cmd_t o;
    agc_sp_t             err20 = agc_sp_from_double(20.0 / 360.0, 0);

    CHECK(agc_entry_dap_roll_phase_plane(err20, 0, &o) == AGC_STATUS_OK);
    CHECK(!o.in_deadzone);
    CHECK(o.jndx == -1);                       /* positive roll jets     */
    CHECK(o.ton1 > 0);
    CHECK(agc_entry_dap_roll_phase_plane((agc_sp_t)(-err20), 0, &o) == AGC_STATUS_OK);
    CHECK(!o.in_deadzone);
    CHECK(o.jndx == 1);                        /* negative roll jets     */
    CHECK(o.ton1 > 0);
    CHECK(agc_entry_dap_roll_jet_bits(-1) != agc_entry_dap_roll_jet_bits(1));
    /* 1 deg error, no rate: inside the 4 deg dead zone (XMIN).           */
    CHECK(agc_entry_dap_roll_phase_plane(agc_sp_from_double(1.0 / 360.0, 0), 0, &o)
          == AGC_STATUS_OK);
    CHECK(o.in_deadzone);
}

static void test_xt1lim(void)
{
    /* MA = 2 RCON = 2 R(T1) (B30/B29): ratio 1, BETA10 = BETA5 - 1;
     * BETA5 = 5 (B9) -> X(T1)LIM = 2 (B5).                               */
    agc_dp_t ma = agc_dp_from_double(1.4e7, 30);
    agc_dp_t rc = agc_dp_from_double(0.7e7, 29);
    agc_dp_t x = 0;

    CHECK(agc_p37_xt1lim(ma, rc, rc, agc_dp_from_double(5.0, 9), &x) == AGC_STATUS_OK);
    CHECK(fabs(agc_dp_to_double(x, 5) - 2.0) < 1e-4);
}

int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    test_huntest_range();
    test_roll_phase_plane();
    test_xt1lim();
    (void)printf("test_entry: %d checks, %d failures, %d assert reports\n",
                 g_checks, g_failures, g_assert_reports);
    return (g_failures == 0) ? 0 : 1;
}
