/**********************************************************************
 * FILE NAME: return_to_earth.c
 *
 * PURPOSE:
 *   P37 return to earth unit tests (lane p37).
 *   1. Coplanar conic return: CSM 300,000 km from the earth, in the X-Y
 *      plane, desired reentry angle -6.5 deg, minimum fuel (RTEDVD = 0).
 *      The conic search (RTE317..RTE390 with V2T100/GAMDV10/DVCALC/XT1LIM
 *      and TMRAD100) must converge; a double-precision two-body model of
 *      the post-impulse conic V2(T1)/ must reach RCON (about 400,000 ft
 *      above the Fischer ellipsoid) with a flight path angle of -6.5 deg,
 *      stay in the plane, and DV must be a minimum along X(T1).
 *   2. Time critical mode: RTEDVD = 5 m/cs is met within EPC10RTE.
 *   3. AUGEKUGL: entry at 36,000 fps, -6.5 deg gives the K1/K2 fit range
 *      PHI = 2400 / (6.5 - K2) NM, K2 = 2.4 + .000285 (V - 32000), and
 *      TE = PHI x 720000 cs/rev above 26,000 fps.
 *   4. XT1LIM keeps its guid-entry public form (BETA5 B9).
 *
 * AGC SOURCE:
 *   Comanche055 P37_P70.agc pp. 891-930, P30-P37.agc pp. 644-647
 *
 * EXTERNAL VARIABLES:
 *   none (the P37 erasables through rte_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   rte_initial_vectors .. rte_conic_search  INVC100, RTE299 .. P37E
 *   rte_augekugl          AUGEKUGL
 *   agc_return_to_earth_xt1lim  XT1LIM
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   exit status 1 when a check fails
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   The double-precision model uses MU = RTMURTE^2 (m^3/cs^2).
 **********************************************************************/
#include <stdio.h>

#include <math.h>
#include <string.h>

#include "../../src/csm/burns/return_to_earth/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/csm/entry.h"

#define CHECK(c) check((c), __LINE__, #c) /* count and report a check */

static int g_checks;         /* checks run                        */
static int g_failures;       /* checks failed                     */
static int g_assert_reports; /* AGC_ASSERT failures reported      */

static const double MU  = 3.986032e10;      /* m^3/cs^2 (RTMURTE^2)     */
static const double DEG = 1.0 / 360.0;      /* revs per degree          */
static const double PI  = 3.14159265358979; /* pi                  */

static void   check(bool ok, int line, const char *text);
static double vnorm(const double v[3]);
static double fpa_at_radius(const double r1[3], const double v[3], double r);
static void   set_vec(agc_vec_t *out, const double v[3], int scale);
static void   get_vec(double out[3], agc_vec_t v, int scale);
static void   start_state(const double r1[3], const double v1[3], agc_dp_t rtedvd);
static double model_dv(double xk, double rcon);
static void   test_coplanar_conic_return(void);
static void   test_time_critical_return(void);
static void   test_augekugl(void);
static void   test_xt1lim(void);

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
    test_coplanar_conic_return();
    test_time_critical_return();
    test_augekugl();
    test_xt1lim();
    (void) printf("test_return_to_earth: %d checks, %d failures, %d assert reports\n", g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1); /* exit status */
}

/**********************************************************************
 * AGC: none -- assertion hook of the library: print, count, continue.
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
    (void) printf("assert %s:%d %s\n", file, line, expr);
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
 * AGC: none -- Euclidean norm of a double vector.
 *
 * FUNCTION NAME: vnorm
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description
 *   --------  --------------  ---  --------------------------------
 *   v         const double *  I    the vector (3 components)
 *
 * RETURNS: double -- |v|, units of v
 **********************************************************************/
static double vnorm(const double v[3])
{
    return (sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
}

/**********************************************************************
 * AGC: none (two-body model) -- flight path angle (deg, + above the
 * horizon) of the conic r1, v at radius r on the descending leg, by the
 * vis-viva and angular-momentum relations.
 *
 * FUNCTION NAME: fpa_at_radius
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description
 *   --------  --------------  ---  --------------------------------
 *   r1        const double *  I    position, m
 *   v         const double *  I    velocity, m/cs
 *   r         double          I    radius of interest, m
 *
 * RETURNS: double -- flight path angle at r, deg (negative: descending)
 **********************************************************************/
static double fpa_at_radius(const double r1[3], const double v[3], double r)
{
    double h[3] = {r1[1] * v[2] - r1[2] * v[1], r1[2] * v[0] - r1[0] * v[2], r1[0] * v[1] - r1[1] * v[0]}; /* r1 x v, m^2/cs   */
    double en   = 0.5 * (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) - MU / vnorm(r1);                        /* specific energy, m^2/cs^2 */
    double vr   = sqrt(2.0 * (en + MU / r));                                                               /* speed at r, m/cs        */
    double c    = vnorm(h) / (r * vr);                                                                     /* cos(gamma)              */

    if (c > 1.0)
    {
        c = 1.0;
    }

    return (-acos(c) * 180.0 / PI);
}

/**********************************************************************
 * AGC: none -- double vector to agc_vec_t at 2^scale.
 *
 * FUNCTION NAME: set_vec
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description
 *   --------  --------------  ---  --------------------------------
 *   out       agc_vec_t *     O    the AGC vector
 *   v         const double *  I    the value (3 components)
 *   scale     int             I    binary scale of out (B scale)
 *
 * RETURNS: none
 **********************************************************************/
static void set_vec(agc_vec_t *out, const double v[3], int scale)
{
    out->c[0] = agc_dp_from_double(v[0], scale);
    out->c[1] = agc_dp_from_double(v[1], scale);
    out->c[2] = agc_dp_from_double(v[2], scale);
}

/**********************************************************************
 * AGC: none -- agc_vec_t at 2^scale to a double vector.
 *
 * FUNCTION NAME: get_vec
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description
 *   --------  ---------  ---  --------------------------------
 *   out       double *   O    the value (3 components)
 *   v         agc_vec_t  I    the AGC vector
 *   scale     int        I    binary scale of v (B scale)
 *
 * RETURNS: none
 **********************************************************************/
static void get_vec(double out[3], agc_vec_t v, int scale)
{
    out[0] = agc_dp_to_double(v.c[0], scale);
    out[1] = agc_dp_to_double(v.c[1], scale);
    out[2] = agc_dp_to_double(v.c[2], scale);
}

/**********************************************************************
 * INVC100 .. RTE317 (P37_P70.agc pp. 891-892, 905-906) -- the P37 state
 * at TIG = 1e6 cs for r1, v1 and the desired DV, reentry angle -6.5 deg,
 * then the conic search.
 *
 * FUNCTION NAME: start_state
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description
 *   --------  --------------  ---  --------------------------------
 *   r1        const double *  I    position at TIG, m
 *   v1        const double *  I    velocity at TIG, m/cs
 *   rtedvd    agc_dp_t        I    RTEDVD (0: minimum fuel), m/cs B7
 *
 * RETURNS: none
 **********************************************************************/
static void start_state(const double r1[3], const double v1[3], agc_dp_t rtedvd)
{
    rte_erasable_t *rs = rte_state(); /* P37 erasables           */
    agc_interp_t   *ip = rte_ip();    /* P37 interpreter state   */
    agc_vec_t       r;                /* R(T1)/, m B29           */
    agc_vec_t       v;                /* V(T1)/, m/cs B7         */

    (void) memset(rs, 0, sizeof (*rs));               /* returns rs */
    agc_interp_init(ip);
    set_vec(&r, r1, 29);
    set_vec(&v, v1, 7);
    rs->rtedvd   = rtedvd;
    rs->rtegam2d = agc_dp_from_double(-6.5 * DEG, 0); /* -6.5 deg     */
    CHECK(rte_initial_vectors(ip, rs, r, v, agc_dp_from_double(1.0e6, 28), 0) == AGC_STATUS_OK);
    CHECK(rte_conic_setup(ip, rs) == AGC_STATUS_OK);
    CHECK(rte_conic_search(ip, rs, 0) == AGC_STATUS_OK);
}

/**********************************************************************
 * DVCALC (P37_P70.agc pp. 928-929) as a double model for the coplanar
 * case: the DV of the conic through R(T1) and RCON with X(T1) = xk.
 *
 * FUNCTION NAME: model_dv
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description
 *   --------  ------  ---  ------------------------------------------
 *   xk        double  I    X(T1), cot of the post-impulse path angle
 *   rcon      double  I    RCON, m
 *
 * RETURNS: double -- DV, m/cs
 **********************************************************************/
static double model_dv(double xk, double rcon)
{
    const rte_erasable_t *rs  = rte_state();                                               /* X(T2)     */
    const double          r1  = 3.0e8;                                                     /* R(T1), m  */
    double                lam = r1 / rcon;                                                 /* LAMBDA    */
    double                b1  = 1.0 + pow(agc_dp_to_double(rs->x_t2, 0), 2);               /* BETA1 */
    double                p   = 2.0 * r1 * (lam - 1.0) / (b1 * lam * lam - 1.0 - xk * xk); /* PCON, m */
    double                s   = sqrt(MU * p) / r1;                                         /* THETA3 PCON^.5, m/cs */

    return (hypot(s * xk + 3.0, s - 6.0)); /* |V2 - V1|, V1 = (-3, 6) */
}

/**********************************************************************
 * RTE317 .. RTE390 (P37_P70.agc pp. 892-895) -- coplanar minimum-fuel
 * conic return (see the file prolog, test 1).
 *
 * FUNCTION NAME: test_coplanar_conic_return
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_coplanar_conic_return(void)
{
    const rte_erasable_t *rs    = rte_state();       /* P37 erasables */
    const double          r1[3] = {3.0e8, 0.0, 0.0}; /* m             */
    const double          v1[3] = {-3.0, 6.0, 0.0};  /* m/cs          */
    double                v2[3];                     /* V2(T1), m/cs  */
    double                rcon;                      /* RCON, m       */
    double                gam;                       /* entry, deg    */
    double                dv;                        /* DV, m/cs      */
    double                x;                         /* X(T1)         */
    double                best  = 1e30;              /* min DV, m/cs  */
    int                   k;                         /* step          */

    start_state(r1, v1, 0); /* minimum fuel      */
    get_vec(v2, rs->v2_t1v, 7);
    rcon = agc_dp_to_double(rs->rcon, 29);
    gam  = fpa_at_radius(r1, v2, rcon);
    dv   = agc_dp_to_double(rs->dv, 7);
    (void) printf("conic: RCON %.0f m, V2 (%.4f %.4f %.4f) m/cs, DV %.4f m/cs, " "gamma %.3f deg, X(T2) %.5f, T12 %.0f s, iterations %d\n", rcon, v2[0], v2[1], v2[2], dv, gam, agc_dp_to_double(rs->x_t2, 0), agc_dp_to_double(rs->t12, 28) / 100.0, rs->nn1a + RTE_CONIC_PASSES);
    CHECK(rcon > 6.47e6 && rcon < 6.52e6);
    CHECK(fabs(gam + 6.5) < 0.05);
    CHECK(fabs(v2[2]) < 1e-6);
    CHECK(fabs(agc_dp_to_double(rs->x_t2, 0) - tan(-6.5 * PI / 180.0)) < 2e-3);

    /* minimum DV along X(T1): a step either way costs more (double model) */
    x = agc_dp_to_double(rs->x_t1, 5);
    CHECK(fabs(model_dv(x, rcon) - dv) < 1e-3 * dv);

    for (k = -1; k <= 1; k++)
    {
        best = fmin(best, model_dv(x + 0.002 * k, rcon));
    }

    CHECK(fabs(best - dv) < 1e-3 * dv);
    CHECK(agc_flag_test(AGC_FLAG_RETROFLG) == false);
}

/**********************************************************************
 * V2T175 .. V2T179 (P37_P70.agc p. 922) -- time critical mode: RTEDVD =
 * 5 m/cs (above the 3.64 m/cs minimum) is met within EPC10RTE, on the
 * faster (lower X(T1) magnitude) side.
 *
 * FUNCTION NAME: test_time_critical_return
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_time_critical_return(void)
{
    const rte_erasable_t *rs    = rte_state();       /* P37 erasables */
    const double          r1[3] = {3.0e8, 0.0, 0.0}; /* m             */
    const double          v1[3] = {-3.0, 6.0, 0.0};  /* m/cs          */
    double                v2[3];                     /* V2(T1), m/cs  */
    double                rcon;                      /* RCON, m       */

    start_state(r1, v1, agc_dp_from_double(5.0, 7));
    get_vec(v2, rs->v2_t1v, 7);
    rcon = agc_dp_to_double(rs->rcon, 29);
    (void) printf("time critical: DV %.5f m/cs, T12 %.0f s, gamma %.3f deg\n",
                  agc_dp_to_double(rs->dv, 7),
                  agc_dp_to_double(rs->t12, 28) / 100.0,
                  fpa_at_radius(r1, v2, rcon));
    CHECK(fabs(agc_dp_to_double(rs->dv, 7) - 5.0) < 2e-3);
    CHECK(agc_dp_to_double(rs->t12, 28) < 2.40e7); /* faster than min fuel */
    CHECK(fabs(fpa_at_radius(r1, v2, rcon) + 6.5) < 0.05);
}

/**********************************************************************
 * AUGEKUGL (P30-P37.agc pp. 644-647) -- range and time for 36,000 fps,
 * -6.5 deg against the K1/K2 fit.
 *
 * FUNCTION NAME: test_augekugl
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_augekugl(void)
{
    agc_interp_t *ip     = rte_ip();                             /* interpreter   */
    agc_dp_t      phie   = 0;                                    /* PHIE, revs B0 */
    agc_dp_t      te     = 0;                                    /* TE, cs B28    */
    double        v      = 36000.0 * 0.3048 / 100.0;             /* m/cs          */
    double        k2     = 2.4 + 0.000285 * (36000.0 - 32000.0); /* deg */
    double        phi_nm = 2400.0 / (6.5 - k2);                  /* fit range, NM */

    agc_interp_init(ip);
    CHECK(rte_augekugl(ip, agc_dp_from_double(v, 7), agc_dp_from_double(6.5 * DEG, 0), &phie, &te) == AGC_STATUS_OK);
    (void) printf("augekugl: PHIE %.2f NM (fit %.2f), TE %.1f s\n", agc_dp_to_double(phie, 0) * 21600.0, phi_nm, agc_dp_to_double(te, 28) / 100.0);
    CHECK(fabs(agc_dp_to_double(phie, 0) * 21600.0 - phi_nm) < 1.0);
    CHECK(fabs(agc_dp_to_double(te, 28) - agc_dp_to_double(phie, 0) * 7.2e5) < 2.0);
}

/**********************************************************************
 * XT1LIM (P37_P70.agc p. 930) -- the public form: MA = 2 RCON = 2 R(T1),
 * BETA5 = 5 (B9) gives X(T1)LIM = 2 (B5).
 *
 * FUNCTION NAME: test_xt1lim
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_xt1lim(void)
{
    agc_dp_t x = 0; /* X(T1)LIM, B5 */

    CHECK(agc_return_to_earth_xt1lim(agc_dp_from_double(1.4e7, 30), agc_dp_from_double(0.7e7, 29), agc_dp_from_double(0.7e7, 29), agc_dp_from_double(5.0, 9), &x) == AGC_STATUS_OK);
    CHECK(fabs(agc_dp_to_double(x, 5) - 2.0) < 1e-4);
}
