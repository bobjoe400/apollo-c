/**********************************************************************
 * FILE NAME: entry.c
 *
 * PURPOSE:
 *   guid-entry unit tests (Comanche entry guidance, the entry DAP roll
 *   phase plane and SP helpers, XT1LIM of P37).
 *   1. HUNTEST / RANGER for a nominal lunar-return entry state in the
 *      up-control phase (V = 32000 fps, RDOT = -300 fps, D = 4 g, L/D =
 *      0.3) finds an up-control solution and predicts a finite range that
 *      agrees with a double-precision model of the same equations.
 *   2. The CM/RCS roll phase plane fires the positive roll jets (JNDX =
 *      -1) for a positive roll error and the negative ones for a negative
 *      error, with no rate; a tiny error falls in the dead zone.
 *   3. XT1LIM gives sqrt(BETA10) for a simple case.
 *   4. READGYM1 / EXDAP2 MSU wraps an angle/180 sum through 180 deg (not
 *      a saturation), and an "OVFL OK" store (CM/FDAI PAXERR1) drops 1.0.
 *   5. SIDETRAK presets PHSPRDT5 = PRIO16, so a group 5 restart after a
 *      sidetracked HUNTEST pass starts the sequencer at PRIO16.
 *
 * AGC SOURCE:
 *   Comanche055 REENTRY_CONTROL.agc pp. 855-862 (HUNTEST, RANGER),
 *   CM_ENTRY_DIGITAL_AUTOPILOT.agc pp. 1065, 1081-1086, 1090,
 *   P37_P70.agc p. 930 (XT1LIM)
 *
 * EXTERNAL VARIABLES:
 *   none (the entry erasables through agc_entry_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_entry_huntest_predict / agc_entry_range_predict  HUNTEST, RANGER
 *   agc_entry_dap_roll_phase_plane  CM/RCS phase plane
 *   agc_return_to_earth_xt1lim      XT1LIM
 *   entry_angle_wrap / entry_sp_store_ovf  SP helpers of the DAP
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   exit status 1 when a check fails
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Scales: /2VS = /51532.3946 fps, /805 = /25 g, range revs = /21600 NM.
 **********************************************************************/
#include <stdio.h>

#include <math.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"

#define CHECK(c) check((c), __LINE__, #c) /* count and report a check */

static int g_checks;         /* checks run                        */
static int g_failures;       /* checks failed                     */
static int g_assert_reports; /* AGC_ASSERT failures reported      */

static const double TWO_VS_FPS = 51532.3946; /* 2 VSAT, ft/s            */
static const double G25_FPSS   = 805.0;      /* 25 g, ft/s^2            */
static const double REV_NM     = 21600.0;    /* NM per rev of range     */

static void   check(bool ok, int line, const char *text);
static void   set_nominal_upcontrol_state(void);
static double model_range_revs(double *vl_out, double *gammal_out);
static void   test_huntest_range(void);
static void   test_roll_phase_plane(void);
static void   test_xt1lim(void);
static void   test_angle_wrap(void);
static void   test_sidetrak_restart_priority(void);

/**********************************************************************
 * AGC: none -- run the tests; the exit status reports failures.
 *
 * FUNCTION NAME: main
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- 0 when every check passes, 1 otherwise
 **********************************************************************/
int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    test_huntest_range();
    test_roll_phase_plane();
    test_xt1lim();
    test_angle_wrap();
    test_sidetrak_restart_priority();
    (void) printf("test_entry: %d checks, %d failures, %d assert reports\n", g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1); /* exit status */
}

/**********************************************************************
 * AGC: none -- assertion hook of the library: count and continue.
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description
 *   --------  ------------  ---  ----------------------------------
 *   file      const char *  I    source file of the failed assertion
 *   line      int           I    its line
 *   expr      const char *  I    the asserted expression
 *
 * RETURNS: bool -- false (the assertion stays failed)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) file;
    (void) line;
    (void) expr;
    g_assert_reports++;

    return (false);
}

/**********************************************************************
 * AGC: none -- record one check; print the failing ones.
 *
 * FUNCTION NAME: check
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description
 *   --------  ------------  ---  ----------------------------------
 *   ok        bool          I    the checked condition
 *   line      int           I    source line of the check
 *   text      const char *  I    the checked expression
 *
 * RETURNS: none
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
 * AGC: none (state as after KATEST, P64, REENTRY_CONTROL.agc p. 853) --
 * nominal entry state at the start of HUNTEST.
 *
 * FUNCTION NAME: set_nominal_upcontrol_state
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void set_nominal_upcontrol_state(void)
{
    agc_entry_state_t *es = agc_entry_state();    /* entry erasables */
    double             v  = 32000.0 / TWO_VS_FPS; /* V, scale 2VS    */

    agc_entry_lexicon_reset();
    es->v       = agc_dp_from_double(v, 0);
    es->vsquare = agc_dp_from_double(v * v, 0);          /* (V/VS)^2 / 4 = v^2 */
    es->rdot    = agc_dp_from_double(-300.0 / TWO_VS_FPS, 0);
    es->d       = agc_dp_from_double(4.0 * 32.2 / G25_FPSS, 0);
    es->lad     = agc_dp_from_double(0.3, 0);
    es->l_d     = es->lad;                               /* lift up            */
    es->lewd    = agc_dp_from_double(0.15, 0);           /* LEWD1              */
    es->q7      = agc_dp_from_double(6.0 / G25_FPSS, 0); /* Q7F          */
    es->q2      = agc_dp_from_double((0.3 * 500.0 - 1152.0) / REV_NM, 0);
    es->thetah  = agc_dp_from_double(1300.0 / REV_NM, 0);
}

/**********************************************************************
 * HUNTEST .. RANGER (REENTRY_CONTROL.agc pp. 855-862) as a
 * double-precision model with the listing's constants, for the nominal
 * state above.
 *
 * FUNCTION NAME: model_range_revs
 *
 * ARGUMENTS:
 *   Argument    Type      I/O  Description (scale)
 *   ----------  --------  ---  --------------------------------------
 *   vl_out      double *  O    VL, scale 2VS
 *   gammal_out  double *  O    GAMMAL, rad
 *
 * RETURNS: double -- predicted range ASKEP + ASP1 + ASPUP + ASPDWN +
 *          ASP3, revs
 **********************************************************************/
static double model_range_revs(double *vl_out, double *gammal_out)
{
    const double v         = 32000.0 / TWO_VS_FPS;                                /* V, 2VS           */
    const double vsq       = v * v;                                               /* VSQ, 4           */
    const double rdot      = -300.0 / TWO_VS_FPS;                                 /* RDOT, 2VS        */
    const double d         = 4.0 * 32.2 / G25_FPSS;                               /* D, 805           */
    const double lad       = 0.3;                                                 /* LAD              */
    const double lewd      = 0.15;                                                /* LEWD             */
    const double q7        = 6.0 / G25_FPSS;                                      /* Q7, 805          */
    const double q2        = (0.3 * 500.0 - 1152.0) / REV_NM;                     /* Q2, revs */
    const double two_c1_hs = 0.0215983264;                                        /* 2C1HS            */
    const double chook     = 1.0 / 64.0;                                          /* CHOOK            */
    const double ch1       = 0.64;                                                /* CH1 2DEC 0.32 B1 = 16/25 */
    double       v1        = rdot / lad + v;                                      /* RDOT < 0: LAD, 2VS      */
    double       a0        = (rdot * rdot / lad / two_c1_hs + d) * v1 * v1 / vsq; /* A0, 805          */
    double       alp       = a0 * two_c1_hs / v1 / v1 / lewd;                     /* ALP    */
    double       fact1     = v1 / (1.0 - alp);                                    /* FACT1            */
    double       fact2     = (alp - 1.0) * alp / a0;                              /* FACT2           */
    double       vl        = (1.0 - sqrt(fact2 * q7 + alp)) * fact1;              /* VL */
    double       gl1       = lewd * (v1 - vl) / vl;                               /* GAMMAL1 */
    double       vbars     = vl * vl;                                             /* VBARS, 4 */
    double       dvl       = 0.5 - vl;                                            /* DVL, 2VS */
    double       vs1       = 0.5;                                                 /* VS1, 2VS */
    double       q7n       = q7;                                                  /* Q7 after NEGAMA, 805 */
    double       dhook;                                                           /* DHOOK, 805 */
    double       ahookdv;                                                         /* AHOOKDV, 16 */
    double       g;                                                               /* GAMMAL, rad */
    double       cosg;                                                            /* COSG/2 */
    double       e4;                                                              /* E/4 */
    double       askep;                                                           /* ASKEP, revs */
    double       aspup;                                                           /* ASPUP, revs */
    double       aspdwn;                                                          /* ASPDWN, revs */

    if (v1 > 0.5)
    {
        dvl = v1 - vl;
        vs1 = v1;
    }

    dhook   = (pow(1.0 - vs1 / fact1, 2) - alp) / fact2;
    ahookdv = dhook / 64.0 / q7 - chook;
    g       = gl1 - (ahookdv + 1.0 / 16.0) * ch1 * dvl * dvl / dhook / vbars;

    if (g < 0)
    { /* NEGAMA         */
        vl    += (g * vl / 3.0) / (lewd / 3.0 - (ahookdv + 1.0 / 24.0) * dvl * ch1 / dhook / vl);
        q7n   = (pow(1.0 - vl / fact1, 2) - alp) / fact2;
        vbars = vl * vl;
        g     = 0;
    }

    gl1         += (g - gl1) * 0.5; /* Q19            */
    cosg        = 0.5 - g * g / 4.0;
    e4          = sqrt(4.0 * (vbars - 0.5) * vbars * cosg * cosg + 1.0 / 16.0);
    askep       = 2.0 * asin(vbars * cosg * g / e4) / 6.283185307179586;
    aspup       = (-log(v1 * v1 * q7n / vbars / a0) / 32.0) * 0.00684572901 / gl1;
    aspdwn      = -0.0247622232 * rdot * v / a0 / lad;
    *vl_out     = vl;
    *gammal_out = g;

    return (askep + (vl * 0.167003132 + q2) + aspup + aspdwn + (0.0349 - g) * 0.326388889);
} /* model_range_revs */

/**********************************************************************
 * HUNTEST / RANGER (REENTRY_CONTROL.agc pp. 855-862) -- the up-control
 * solution and the predicted range against the model.
 *
 * FUNCTION NAME: test_huntest_range
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_huntest_range(void)
{
    agc_entry_hunt_result_t  r        = AGC_ENTRY_HUNT_CONSTD; /* HUNTEST exit */
    agc_dp_t                 gammal   = 0;                     /* GAMMAL, revs B0           */
    agc_dp_t                 diff     = 0;                     /* DIFF, revs B0             */
    const agc_entry_state_t *es;                               /* entry erasables           */
    double                   range_nm;                         /* predicted range, NM       */
    double                   model_vl = 0;                     /* model VL, scale 2VS       */
    double                   model_g  = 0;                     /* model GAMMAL, rad         */
    double                   model_nm;                         /* model range, NM           */

    model_nm = model_range_revs(&model_vl, &model_g) * REV_NM;
    set_nominal_upcontrol_state();
    CHECK(agc_entry_huntest_predict(&r, &gammal) == AGC_STATUS_OK);
    CHECK(r == AGC_ENTRY_HUNT_RANGER);
    es       = agc_entry_state();
    CHECK(fabs(agc_dp_to_double(es->vl, 0) - model_vl) < 1e-5); /* 0.5 fps */
    CHECK(fabs(agc_dp_to_double(gammal, 0) - model_g) < 1e-6);
    CHECK(agc_dp_to_double(es->vl, 0) * TWO_VS_FPS > 18000.0);  /* > VLMIN */
    CHECK(agc_entry_range_predict(gammal, &diff) == AGC_STATUS_OK);
    range_nm = (agc_dp_to_double(diff, 0) + 1300.0 / REV_NM) * REV_NM;
    (void) printf("  HUNTEST: VL %.0f fps, GAMMAL %.5f rad, range %.1f NM (model %.1f)\n", agc_dp_to_double(es->vl, 0) * TWO_VS_FPS, agc_dp_to_double(gammal, 0), range_nm, model_nm);
    CHECK(isfinite(range_nm));
    CHECK(range_nm > 100.0 && range_nm < 10000.0);
    CHECK(fabs(range_nm - model_nm) < 1.0);
    CHECK(es->diff == diff);
}

/**********************************************************************
 * CM/RCS phase plane (CM_ENTRY_DIGITAL_AUTOPILOT.agc pp. 1081-1086) --
 * jet direction for a +-20 deg error and the 4 deg dead zone.
 *
 * FUNCTION NAME: test_roll_phase_plane
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_roll_phase_plane(void)
{
    agc_entry_roll_cmd_t o;     /* phase-plane solution                 */
    agc_sp_t             err20; /* X/360 for 20 deg, B0 */

    err20 = agc_sp_from_double(20.0 / 360.0, 0);
    CHECK(agc_entry_dap_roll_phase_plane(err20, 0, &o) == AGC_STATUS_OK);
    CHECK(!o.in_deadzone);
    CHECK(o.jndx == -1); /* positive roll jets     */
    CHECK(o.ton1 > 0);
    CHECK(agc_entry_dap_roll_phase_plane((agc_sp_t) (-err20), 0, &o) == AGC_STATUS_OK);
    CHECK(!o.in_deadzone);
    CHECK(o.jndx == 1);  /* negative roll jets     */
    CHECK(o.ton1 > 0);
    CHECK(agc_entry_dap_roll_jet_bits(-1) != agc_entry_dap_roll_jet_bits(1));

    /* 1 deg error, no rate: inside the 4 deg dead zone (XMIN).           */
    CHECK(agc_entry_dap_roll_phase_plane(agc_sp_from_double(1.0 / 360.0, 0), 0, &o) == AGC_STATUS_OK);
    CHECK(o.in_deadzone);
}

/**********************************************************************
 * XT1LIM (P37_P70.agc p. 930) -- MA = 2 RCON = 2 R(T1) (B30/B29):
 * ratio 1, BETA10 = BETA5 - 1; BETA5 = 5 (B9) gives X(T1)LIM = 2 (B5).
 *
 * FUNCTION NAME: test_xt1lim
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_xt1lim(void)
{
    agc_dp_t ma = agc_dp_from_double(1.4e7, 30); /* MA, m B30           */
    agc_dp_t rc = agc_dp_from_double(0.7e7, 29); /* RCON = R(T1), m B29 */
    agc_dp_t x  = 0;                             /* X(T1)LIM, B5        */

    CHECK(agc_return_to_earth_xt1lim(ma, rc, rc, agc_dp_from_double(5.0, 9), &x) == AGC_STATUS_OK);
    CHECK(fabs(agc_dp_to_double(x, 5) - 2.0) < 1e-4);
}

/**********************************************************************
 * MSU / CORANGOV and "OVFL OK" stores (CM_ENTRY_DIGITAL_AUTOPILOT.agc
 * pp. 1065, 1090) -- 170 deg + 20 deg = 190 deg = -170 deg (angle/180:
 * 1.0 = 16384), and a sum past +-1.0 stored modulo 1.
 *
 * FUNCTION NAME: test_angle_wrap
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_angle_wrap(void)
{
    agc_sp_t a170 = (agc_sp_t) (170.0 / 180.0 * 16384.0); /* /180, B0 */
    agc_sp_t a20  = (agc_sp_t) (20.0 / 180.0 * 16384.0);  /* /180, B0 */
    agc_sp_t w;                                           /* 190 deg wrapped, /180 B0 */

    w = entry_angle_wrap((int32_t) a170 + (int32_t) a20);
    CHECK(w < 0);
    CHECK(w == (agc_sp_t) ((int32_t) a170 + (int32_t) a20 - 32768));
    CHECK(entry_angle_wrap(-(int32_t) a170 - (int32_t) a20) == -w);
    CHECK(entry_angle_wrap(1000) == 1000);         /* in range: unchanged  */
    CHECK(entry_sp_store_ovf(16384 + 100) == 100); /* OVFL OK store    */
    CHECK(entry_sp_store_ovf(-16384 - 100) == -100);
}

/**********************************************************************
 * SIDETRAK (REENTRY_CONTROL.agc p. 861) -- CA PRIO16 / TS PHSPRDT5 before
 * PHASCHNG OCT 00474: a group 5 restart after a sidetracked HUNTEST pass
 * starts the entry sequencer (SERVICER 5.3, a type B phase with D clear)
 * at PRIO16, one below group 4.
 *
 * FUNCTION NAME: test_sidetrak_restart_priority
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_sidetrak_restart_priority(void)
{
    agc_entry_state_t *es      = agc_entry_state(); /* entry erasables */
    bool               display = false;             /* ENDEXIT display */
    bool               found   = false;             /* restart job seen at PRIO16 */

    CHECK(agc_power_on(AGC_VEHICLE_CSM) == AGC_STATUS_OK);
    set_nominal_upcontrol_state();
    es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST;
    es->thetah   = agc_dp_from_double(2000.0 / REV_NM, 0); /* range to go far beyond the up-control range */
    es->dlewd    = agc_dp_from_double(0.01, 0);
    es->diffold  = agc_dp_from_double(0.5, 0);
    CHECK(agc_entry_reestablish_sequencer(&display) == AGC_STATUS_OK);
    CHECK(es->gotoaddr == AGC_ENTRY_GOTO_HUNTEST);         /* sidetracked: the next pass resumes HUNTEST */
    CHECK(agc_exec_init() == AGC_STATUS_OK);               /* STARTSB2: core sets free, NEWJOB -0 */
    CHECK(agc_restart_group(5) == AGC_STATUS_OK);

    for (int i = 0; i < agc_exec_core_sets(); i++)
    {
        agc_exec_core_info_t ci; /* core set i */

        agc_exec_core_info(i, &ci);
        found = found || (ci.loc == AGC_JOB_REESTABLISH_ENTRY_SEQUENCER && ci.prio == 016);
    }

    CHECK(found);
}
