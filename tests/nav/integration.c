/**********************************************************************
 * FILE NAME: integration.c
 *
 * PURPOSE:
 *   nav-integ unit tests (INTEGRATION_INITIALIZATION + ORBITAL_INTEGRATION
 *   translation) against analytic motion and double-precision reference
 *   models:
 *   1. INTSTALL lock: grab, busy -> JOBSLEEP, INTWAKE releases.
 *   2. RECTIFY arithmetic (deviation folded into the conic, scales).
 *   3. Conic integration (INTEGRVS with INTYPFLG): 300 km circular orbit,
 *      10 minutes, vs analytic circular motion.
 *   4. Two-body-only Encke integration (LM build: MIDFLAG never set; earth
 *      orbit beyond RDE so no oblateness): 100000 km circular orbit,
 *      10 minutes, vs analytic circular motion. Exercises TESTLOOP, DT/2,
 *      ACCOMP/GAMCOMP with zero deviation, FBR3/KEPLER, DIFEQ, RECTOUT.
 *   5. Precision Encke integration in LEO (earth J2..J4 active, CSM below
 *      RME so no third bodies), 10 minutes, inclined orbit, vs a double
 *      RK4 reference with the J2 term (J2 mu Re^2 = J2REQSQ); J3/J4 move
 *      the vehicle by a few metres only. Also vs analytic circular motion
 *      with a loose bound (the J2 effect is ~2 km).
 *   6. W matrix (INTEGRV with DIM0FLAG): a 100 m position column is
 *      propagated by the gravity-gradient equations; compared with the
 *      two-body state transition of the reference (finite difference).
 *   7. Lunar orbit, 1 h: lunar J2, J3 and the pad-loaded J22 (E3J22R2M)
 *      and C31 (E32C31RM) terms vs a reference with the same physics
 *      (each term moves the result by 0.2-1 km; agreement ~20 m).
 *   8. Third bodies: CSM at 60000 km (MIDFLAG), 6 h, constant moon and
 *      sun ephemeris, vs a reference with J2..J4 + moon + sun.
 *   9. STATEINT / STATINT1 / STATEUP: P00 integration of both vehicles,
 *      4-time-step logic (the last partial step is left), NOINT.
 *  10. MIDTOAV1 to TDEC1, and the NOTIME slip (ALARM 1703).
 *  11. AVETOMID: W through the burn, RN VN PIPTIME into the CSM, LM along.
 *  12. INTWAKEU: P27 update into the LM, then the CSM in the moon sphere.
 *
 *   Units: positions m scale 2^29, velocities m/cs scale 2^7, time cs.
 *
 * AGC SOURCE:
 *   Comanche055 / Luminary099 INTEGRATION_INITIALIZATION.agc,
 *   ORBITAL_INTEGRATION.agc (the code under test)
 *
 * EXTERNAL VARIABLES:
 *   none (the tests set the erasables through the accessors)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_integ_*           the integration package (nav/integration.h)
 *   agc_exec_init         executive reset (exec.h)
 *   agc_navaux_*          pad loads (nav/auxiliary.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none (a failed check is counted and printed; exit status 1)
 **********************************************************************/
#include <stdio.h>

#include <math.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"

static int g_checks;         /* checks run */
static int g_failures;       /* checks failed */
static int g_assert_reports; /* AGC_ASSERT reports seen */

#define CHECK(c) check((c), __LINE__, #c)

static const double MU_E = 3.986032e10;            /* MUEARTH m^3/cs^2          */
static const double J2K  = 1.75501139e21;          /* J2REQSQ m^5/cs^2           */
static const double MU_M = 4.9027780e8;            /* MUM m^3/cs^2               */
static const double MU_S = 1.32715445e16;          /* sun m^3/cs^2               */
static const double PI   = 3.14159265358979323846; /* pi (unscaled double) */
#define TEN_MIN_CS 60000.0                         /* 10 minutes, cs */

typedef struct dvec
{
    double x, y, z;
} dvec_t;

/* ---- double reference model, RK4, time in cs --------------------------- */
typedef struct ref_model
{
    double mu;         /* primary, m^3/cs^2                            */
    double j2k;        /* J2REQSQ (J2 mu Re^2), m^5/cs^2; 0 = none     */
    double k3, k4;     /* 2J3RE/J2 (= J3 Re/J2), J4REQ/J3 (= J4 Re/J3) */
    dvec_t pole;       /* planet pole (unit), reference axes           */
    double mrow[3][3]; /* planet-fixed axes (rows, unit) for C22/C31   */
    double e22;        /* E3J22R2M real value (3 J22 R^2 mu)           */
    double e31;        /* E32C31RM real value (3/2 C31 R^3 mu)         */
    int    nbody;      /* third bodies, fixed positions                */
    dvec_t bpos[2];    /* third-body positions, m                      */
    double bmu[2];     /* their mu, m^3/cs^2                           */
} ref_model_t;

typedef struct dstate
{
    dvec_t r; /* position, m   */
    dvec_t v; /* velocity, m/cs */
} dstate_t;

static void         check(bool ok, int line, const char *text);
static dvec_t       dv(double x, double y, double z);
static agc_vec_t    to_agc(dvec_t v, int scale);
static dvec_t       from_agc(agc_vec_t v, int scale);
static double       dist(dvec_t a, dvec_t b);
static void         circular(double mu, double r, double incl, double t, dvec_t *pos, dvec_t *vel);
static double       dot3(dvec_t a, dvec_t b);
static dvec_t       axpy(double a, dvec_t x, dvec_t y);
static dvec_t       zonal(const ref_model_t *m, dvec_t r, double rr);
static dvec_t       tesseral(const ref_model_t *m, dvec_t r, double rr);
static dvec_t       accel(const ref_model_t *m, dvec_t r);
static dstate_t     deriv(const ref_model_t *m, dstate_t s, double h, dstate_t d);
static dstate_t     rk4(const ref_model_t *m, dstate_t s0, double t);
static ref_model_t  earth_model(bool j2);
static void         fresh(agc_vehicle_t veh);
static agc_status_t run_body(agc_integ_ctx_t *ctx, dvec_t r, dvec_t v, bool conic, double t1, bool moon);
static agc_status_t run_integrvs(agc_integ_ctx_t *ctx, dvec_t r, dvec_t v, bool conic, double t1);
static void         test_lock(void);
static void         test_rectify(void);
static void         test_conic_leo(void);
static void         test_encke_twobody(void);
static void         test_encke_leo(void);
static void         test_wmatrix(void);
static void         test_lunar_gravity(void);
static void         test_third_body(void);
static void         set_perm(agc_vehicle_t veh, dvec_t r, dvec_t v, agc_dp_t t);
static void         test_stateint(void);
static void         test_midtoav(void);
static void         test_avetomid(void);
static void         test_intwakeu(void);

/**********************************************************************
 * AGC: none (test driver) -- runs the twelve nav-integ unit tests and prints
 * the summary
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- int 0 when no check failed, 1 otherwise
 *********************************************************************/
int main(void)
{
    test_lock();
    test_rectify();
    test_conic_leo();
    test_encke_twobody();
    test_encke_leo();
    test_wmatrix();
    test_lunar_gravity();
    test_third_body();
    test_stateint();
    test_midtoav();
    test_avetomid();
    test_intwakeu();
    (void) printf("test_integration: %d checks, %d failures, %d assert reports\n", g_checks, g_failures, g_assert_reports);
    (void) PI;

    return ((g_failures == 0) ? 0 : 1); /* exit status */
}

/**********************************************************************
 * AGC: none (AGC_ASSERT hook) -- print and count a failed assertion
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   file      const char *  I    source file of the assertion
 *   line      int           I    its line
 *   expr      const char *  I    the asserted expression
 *
 * RETURNS: bool -- bool false (the assertion failed)
 *********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);
    g_assert_reports++;

    return (false);
}

/**********************************************************************
 * AGC: none (test helper) -- count a check and print a failure
 *
 * FUNCTION NAME: check
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   ok        bool          I    the checked condition
 *   line      int           I    test source line
 *   text      const char *  I    the condition as text
 *********************************************************************/
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
 * AGC: none (test helper) -- a double vector
 *
 * FUNCTION NAME: dv
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   x         double      I    x component
 *   y         double      I    y component
 *   z         double      I    z component
 *
 * RETURNS: dvec_t -- dvec_t (x, y, z)
 *********************************************************************/
static dvec_t dv(double x, double y, double z)
{
    dvec_t r = {x, y, z};

    return (r);
}

/**********************************************************************
 * AGC: none (test helper) -- a double vector to the AGC vector at scale
 * 2^scale
 *
 * FUNCTION NAME: to_agc
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   v         dvec_t      I    vector in SI units (m, m/cs)
 *   scale     int         I    binary scale of the result
 *
 * RETURNS: agc_vec_t -- agc_vec_t the scaled vector
 *********************************************************************/
static agc_vec_t to_agc(dvec_t v, int scale)
{
    return (agc_vec_make(agc_dp_from_double(v.x, scale),
                         agc_dp_from_double(v.y, scale),
                         agc_dp_from_double(v.z, scale)));
}

/**********************************************************************
 * AGC: none (test helper) -- an AGC vector at scale 2^scale to doubles
 *
 * FUNCTION NAME: from_agc
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   v         agc_vec_t   I    scaled vector
 *   scale     int         I    its binary scale
 *
 * RETURNS: dvec_t -- dvec_t the vector in SI units
 *********************************************************************/
static dvec_t from_agc(agc_vec_t v, int scale)
{
    return (dv(agc_dp_to_double(v.c[0], scale), agc_dp_to_double(v.c[1], scale), agc_dp_to_double(v.c[2], scale)));
}

/**********************************************************************
 * AGC: none (test helper) -- |a - b|
 *
 * FUNCTION NAME: dist
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   a         dvec_t      I    first point
 *   b         dvec_t      I    second point
 *
 * RETURNS: double -- double the distance, same unit
 *********************************************************************/
static double dist(dvec_t a, dvec_t b)
{
    const double dx = a.x - b.x; /* x difference */
    const double dy = a.y - b.y; /* y difference */
    const double dz = a.z - b.z; /* z difference */

    return (sqrt((dx * dx) + (dy * dy) + (dz * dz)));
}

/**********************************************************************
 * AGC: none (reference model) -- circular orbit in the plane of x and (y cos i
 * + z sin i)
 *
 * FUNCTION NAME: circular
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   mu        double      I    gravitational constant, m^3/cs^2
 *   r         double      I    radius, m
 *   incl      double      I    inclination, rad
 *   t         double      I    time, cs
 *   pos       dvec_t *    O    position at t, m
 *   vel       dvec_t *    O    velocity at t, m/cs
 *********************************************************************/
static void circular(double mu, double r, double incl, double t, dvec_t *pos, dvec_t *vel)
{
    const double n = sqrt(mu / (r * r * r)); /* mean motion, rad/cs */
    const double v = n * r;                  /* circular speed, m/cs */
    const double c = cos(n * t);             /* cos of the phase angle */
    const double s = sin(n * t);             /* sin of the phase angle */

    *pos = dv(r * c, r * s * cos(incl), r * s * sin(incl));
    *vel = dv(-v * s, v * c * cos(incl), v * c * sin(incl));
}

/**********************************************************************
 * AGC: none (test helper) -- a . b
 *
 * FUNCTION NAME: dot3
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   a         dvec_t      I    first vector
 *   b         dvec_t      I    second vector
 *
 * RETURNS: double -- double the dot product
 *********************************************************************/
static double dot3(dvec_t a, dvec_t b)
{
    return ((a.x * b.x) + (a.y * b.y) + (a.z * b.z));
}

/**********************************************************************
 * AGC: none (test helper) -- a x + y
 *
 * FUNCTION NAME: axpy
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   a         double      I    scalar
 *   x         dvec_t      I    vector scaled by a
 *   y         dvec_t      I    vector added
 *
 * RETURNS: dvec_t -- dvec_t the sum
 *********************************************************************/
static dvec_t axpy(double a, dvec_t x, dvec_t y)
{
    return (dv((a * x.x) + y.x, (a * x.y) + y.y, (a * x.z) + y.z));
}

/**********************************************************************
 * COMTERM (ORBITAL_INTEGRATION.agc pp. 1341-1342) reference -- zonal J2..J4 in
 * the AGC's form: (J2REQSQ/r^4) [(P3' + k3/r (P4' + k4/r P5')) u - (P2' + k3/r
 * (P3' + k4/r P4')) pole]
 *
 * FUNCTION NAME: zonal
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   m         const ref_model_t *  I    reference model (body constants)
 *   r         dvec_t               I    position, m
 *   rr        double               I    |r|, m
 *
 * RETURNS: dvec_t -- dvec_t the zonal acceleration, m/cs^2
 *********************************************************************/
static dvec_t zonal(const ref_model_t *m, dvec_t r, double rr)
{
    const dvec_t u  = dv(r.x / rr, r.y / rr, r.z / rr);                         /* unit position */
    const double c  = dot3(u, m->pole);                                         /* cos of the pole angle */
    const double p2 = 3.0 * c;                                                  /* P2' (derivative form) */
    const double p3 = ((15.0 * c * c) - 3.0) / 2.0;                             /* P3' */
    const double p4 = ((35.0 * c * c * c) - (15.0 * c)) / 2.0;                  /* P4' */
    const double p5 = ((315.0 * c * c * c * c) - (210.0 * c * c) + 15.0) / 8.0; /* P5' */
    const double f  = m->j2k / (rr * rr * rr * rr);                             /* J2REQSQ / r^4 */
    const double cu = p3 + ((m->k3 / rr) * (p4 + ((m->k4 / rr) * p5)));         /* coefficient of u */
    const double cz = p2 + ((m->k3 / rr) * (p3 + ((m->k4 / rr) * p4)));         /* coefficient of the pole */

    return (axpy(-f * cz, m->pole, dv(f * cu * u.x, f * cu * u.y, f * cu * u.z)));
}

/**********************************************************************
 * QUALITY2 / QUALITY3 (ORBITAL_INTEGRATION.agc pp. 1342-1343) reference --
 * lunar J22 and C31 terms in planet-fixed axes
 *
 * FUNCTION NAME: tesseral
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   m         const ref_model_t *  I    reference model (pad-load
 *                                       coefficients)
 *   r         dvec_t               I    position, m
 *   rr        double               I    |r|, m
 *
 * RETURNS: dvec_t -- dvec_t the acceleration, m/cs^2
 *********************************************************************/
static dvec_t tesseral(const ref_model_t *m, dvec_t r, double rr)
{
    const dvec_t u    = dv(r.x / rr, r.y / rr, r.z / rr);                                      /* unit position, reference axes */
    const double x    = (m->mrow[0][0] * u.x) + (m->mrow[0][1] * u.y) + (m->mrow[0][2] * u.z); /* planet-fixed x */
    const double y    = (m->mrow[1][0] * u.x) + (m->mrow[1][1] * u.y) + (m->mrow[1][2] * u.z); /* planet-fixed y */
    const double z    = (m->mrow[2][0] * u.x) + (m->mrow[2][1] * u.y) + (m->mrow[2][2] * u.z); /* planet-fixed z */
    const double r4   = rr * rr * rr * rr;                                                     /* r^4, m^4 */
    const double g    = m->e22 / r4;                                                           /* J22 factor */
    const double q    = 5.0 * ((x * x) - (y * y));                                             /* 5 (x^2 - y^2) */
    double       a[3] = {g *((2.0 * x) - (q * x)), g *((-2.0 * y) - (q * y)), g *(-q * z)};
    const double h    = m->e31 / (r4 * rr);                                                    /* C31 factor */
    const double w    = 5.0 * x * (1.0 - (7.0 * z * z));                                       /* 5 x (1 - 7 z^2) */

    a[0]     += h * ((w * x) + (5.0 * z * z) - 1.0);
    a[1]     += h * (w * y);
    a[2]     += h * ((w * z) + (10.0 * x * z));
    dvec_t o = dv(0.0, 0.0, 0.0); /* acceleration, planet-fixed then reference axes */

    for (int i = 0; i < 3; i++)
    {
        o = axpy(a[i], dv(m->mrow[i][0], m->mrow[i][1], m->mrow[i][2]), o);
    }

    return (o);
}

/**********************************************************************
 * AGC: none (reference model) -- central body, harmonics and third bodies
 *
 * FUNCTION NAME: accel
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   m         const ref_model_t *  I    reference model
 *   r         dvec_t               I    position, m
 *
 * RETURNS: dvec_t -- dvec_t the acceleration, m/cs^2
 *********************************************************************/
static dvec_t accel(const ref_model_t *m, dvec_t r)
{
    const double rr = sqrt(dot3(r, r));                                                                                /* |r|, m */
    dvec_t       a  = dv(-m->mu * r.x / (rr * rr * rr), -m->mu * r.y / (rr * rr * rr), -m->mu * r.z / (rr * rr * rr)); /* central-body acceleration, m/cs^2 */

    if (m->j2k != 0.0)
    {
        const dvec_t z = zonal(m, r, rr); /* zonal terms, m/cs^2 */

        a = axpy(1.0, z, a);
        a = axpy(1.0, tesseral(m, r, rr), a);
    }

    for (int i = 0; i < m->nbody; i++)
    {
        const dvec_t d  = axpy(-1.0, r, m->bpos[i]);          /* body - r  */
        const double dd = sqrt(dot3(d, d));                   /* |body - vehicle|, m */
        const double bb = sqrt(dot3(m->bpos[i], m->bpos[i])); /* |body|, m */

        a = axpy(m->bmu[i] / (dd * dd * dd), d, a);
        a = axpy(-m->bmu[i] / (bb * bb * bb), m->bpos[i], a);
    }

    return (a);
}

/**********************************************************************
 * AGC: none (reference model) -- RK4 stage: the derivative at s + h d
 *
 * FUNCTION NAME: deriv
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   m         const ref_model_t *  I    reference model
 *   s         dstate_t             I    state at the step start
 *   h         double               I    stage step, cs
 *   d         dstate_t             I    derivative of the previous stage
 *
 * RETURNS: dstate_t -- dstate_t the derivative
 *********************************************************************/
static dstate_t deriv(const ref_model_t *m, dstate_t s, double h, dstate_t d)
{
    dstate_t x; /* stage state */

    x.r = axpy(h, d.r, s.r);
    x.v = axpy(h, d.v, s.v);
    dstate_t o; /* derivative */

    o.r = x.v;
    o.v = accel(m, x.r);

    return (o);
}

/**********************************************************************
 * AGC: none (reference model) -- fixed-step RK4 from s0 over t
 *
 * FUNCTION NAME: rk4
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   m         const ref_model_t *  I    reference model
 *   s0        dstate_t             I    initial state
 *   t         double               I    interval, cs
 *
 * RETURNS: dstate_t -- dstate_t the final state
 *********************************************************************/
static dstate_t rk4(const ref_model_t *m, dstate_t s0, double t)
{
    const int      n    = 6000;  /* RK4 steps */
    const double   h    = t / n; /* step, cs */
    const dstate_t zero = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    dstate_t       s    = s0;    /* state being propagated */

    for (int i = 0; i < n; i++)
    {
        const dstate_t k1 = deriv(m, s, 0.0, zero);   /* RK4 stage 1 */
        const dstate_t k2 = deriv(m, s, h / 2.0, k1); /* RK4 stage 2 */
        const dstate_t k3 = deriv(m, s, h / 2.0, k2); /* RK4 stage 3 */
        const dstate_t k4 = deriv(m, s, h, k3);       /* RK4 stage 4 */

        s.r = axpy(h / 6.0, axpy(1.0, axpy(2.0, k2.r, k1.r), axpy(2.0, k3.r, k4.r)), s.r);
        s.v = axpy(h / 6.0, axpy(1.0, axpy(2.0, k2.v, k1.v), axpy(2.0, k3.v, k4.v)), s.v);
    }

    return (s);
}

/**********************************************************************
 * AGC: none (reference model) -- earth, optionally with the J2..J4 terms
 *
 * FUNCTION NAME: earth_model
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   j2        bool        I    include the zonal harmonics
 *
 * RETURNS: ref_model_t -- ref_model_t the model
 *********************************************************************/
static ref_model_t earth_model(bool j2)
{
    ref_model_t m = {0};

    m.mu         = MU_E;
    m.j2k        = j2 ? J2K : 0.0; /* zonal terms on or off */
    m.k3         = -0.1355426363e5;
    m.k4         = 0.4991607391e7;
    m.pole       = dv(0.0, 0.0, 1.0);
    m.mrow[0][0] = 1.0;
    m.mrow[1][1] = 1.0;
    m.mrow[2][2] = 1.0;

    return (m);
}

/**********************************************************************
 * AGC: none (test setup) -- state, executive and waitlist reset for a vehicle
 *
 * FUNCTION NAME: fresh
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------------
 *   veh       agc_vehicle_t  I    vehicle (CSM or LM)
 *********************************************************************/
static void fresh(agc_vehicle_t veh)
{
    agc_state_init(veh);
    agc_exec_init();
    CHECK(agc_waitlist_init() == AGC_STATUS_OK);
    agc_erase_set_modreg(0);
}

/**********************************************************************
 * INTEGRVS (INTEGRATION_INITIALIZATION.agc p. 1319) test driver -- from (r, v)
 * at t = 0 to t1 about the earth or the moon (the input at that body's
 * scales); result in ctx->out
 *
 * FUNCTION NAME: run_body
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  O    integration context (the result)
 *   r         dvec_t             I    position, m
 *   v         dvec_t             I    velocity, m/cs
 *   conic     bool               I    INTYPFLG: conic only
 *   t1        double             I    TDEC1, cs
 *   moon      bool               I    moon-centred
 *
 * RETURNS: agc_status_t -- agc_status_t the status of INTEGRVS
 *********************************************************************/
static agc_status_t run_body(agc_integ_ctx_t *ctx, dvec_t r, dvec_t v, bool conic, double t1, bool moon)
{
    agc_interp_t ip; /* interpretive state */

    agc_interp_init(&ip);
    CHECK(agc_integ_ctx_init(ctx) == AGC_STATUS_OK);
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    ctx->sv.rcv           = to_agc(r, moon ? 27 : 29);                   /* moon 2^27, earth 2^29 m */
    ctx->sv.vcv           = to_agc(v, moon ? 5 : 7);                     /* moon 2^5, earth 2^7 m/cs */
    ctx->sv.tet           = 0;
    agc_flag_put(AGC_FLAG_MOONFLAG, moon);
    agc_flag_put(AGC_FLAG_INTYPFLG, conic);
    const agc_status_t st = agc_integ_integrvs(&ip, ctx, (agc_dp_t) t1); /* status of INTEGRVS */

    CHECK(!agc_integ_area_busy(0)); /* INTWAKE released */

    return (st);
}

/**********************************************************************
 * INTEGRVS (INTEGRATION_INITIALIZATION.agc p. 1319) test driver -- run_body
 * about the earth
 *
 * FUNCTION NAME: run_integrvs
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  O    integration context (the result)
 *   r         dvec_t             I    position, m
 *   v         dvec_t             I    velocity, m/cs
 *   conic     bool               I    INTYPFLG: conic only
 *   t1        double             I    TDEC1, cs
 *
 * RETURNS: agc_status_t -- agc_status_t the status of INTEGRVS
 *********************************************************************/
static agc_status_t run_integrvs(agc_integ_ctx_t *ctx, dvec_t r, dvec_t v, bool conic, double t1)
{
    return (run_body(ctx, r, v, conic, t1, false));
}

/**********************************************************************
 * INTSTALL / INTWAKE (INTEGRATION_INITIALIZATION.agc pp. 1324-1325) -- test 1:
 * grab, busy -> JOBSLEEP, INTWAKE releases; the restart take-over; LM areas
 *
 * FUNCTION NAME: test_lock
 *********************************************************************/
static void test_lock(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(!agc_integ_area_busy(0));
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    CHECK(agc_integ_area_busy(0));
    CHECK(agc_flag_test(AGC_FLAG_INTFLAG)); /* RASFLAG BIT14     */

    /* busy: the caller is put to sleep (here not inside a job, so JOBSLEEP
     * only reports its assertion) and must call again when woken.       */
    CHECK(agc_integ_install(0, AGC_JOB_STATEINT_UPDATE) == AGC_STATUS_BUSY);
    CHECK(agc_integ_install(1, AGC_JOB_NONE) == AGC_STATUS_OK); /* ERASTAL1 */
    CHECK(agc_integ_area_busy(1));
    CHECK(agc_integ_eraswake(1) == AGC_STATUS_OK);
    CHECK(!agc_integ_area_busy(1));
    CHECK(agc_integ_intwake() == AGC_STATUS_OK);
    CHECK(!agc_integ_area_busy(0));
    CHECK(!agc_flag_test(AGC_FLAG_INTFLAG));

    /* a restart (GOPROG2 clears INTFLBIT) inside an integration that had
     * set REINTFLG: the rerun job segment takes the area over.          */
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    agc_flag_set(AGC_FLAG_REINTFLG);
    agc_flag_clear(AGC_FLAG_INTFLAG);
    CHECK(agc_integ_install(0, AGC_JOB_STATEINT_UPDATE) == AGC_STATUS_OK);
    CHECK(agc_flag_test(AGC_FLAG_INTFLAG) && !agc_flag_test(AGC_FLAG_REINTFLG));
    CHECK(agc_integ_intwake() == AGC_STATUS_OK);
    fresh(AGC_VEHICLE_LM);
    CHECK(agc_integ_install(1, AGC_JOB_NONE) == AGC_STATUS_BAD_ARG); /* LM: 1 area */
}

/**********************************************************************
 * RECTIFY (ORBITAL_INTEGRATION.agc p. 1347) -- test 2: the deviation folded
 * into the conic, scales
 *
 * FUNCTION NAME: test_rectify
 *********************************************************************/
static void test_rectify(void)
{
    agc_interp_t    ip;  /* interpretive state */
    agc_integ_ctx_t ctx; /* integration erasables */

    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_integ_ctx_init(&ctx) == AGC_STATUS_OK);
    ctx.sv.rcv     = to_agc(dv(7.0e6, 0.0, 0.0), 29);
    ctx.sv.vcv     = to_agc(dv(0.0, 75.0, 0.0), 7);
    ctx.sv.deltav  = to_agc(dv(1000.0, -500.0, 0.0), 22); /* E 2^22 m    */
    ctx.sv.nuv     = to_agc(dv(0.0, 0.0, 0.05), 3);       /* E 2^3 m/cs  */
    ctx.sv.tc      = 1234;
    ctx.sv.xkep    = 99;
    agc_integ_rectify(&ip, &ctx);
    const dvec_t r = from_agc(ctx.sv.rrect, 29);          /* RRECT, m */
    const dvec_t v = from_agc(ctx.sv.vrect, 7);           /* VRECT, m/cs */

    CHECK(fabs(r.x - 7001000.0) < 4.0);
    CHECK(fabs(r.y + 500.0) < 4.0);
    CHECK(fabs(v.z - 0.05) < 1e-6);
    CHECK(ctx.sv.tc == 0);
    CHECK(ctx.sv.xkep == 0);
    CHECK(agc_vec_abval(&ip, ctx.sv.deltav) == 0);
}

/**********************************************************************
 * RVCON (INTEGRATION_INITIALIZATION.agc p. 1321) -- test 3: conic integration
 * of a 300 km circular orbit over 10 minutes vs analytic motion
 *
 * FUNCTION NAME: test_conic_leo
 *********************************************************************/
static void test_conic_leo(void)
{
    agc_integ_ctx_t ctx; /* integration erasables */

    dvec_t r0; /* initial position, m        */
    dvec_t v0; /* initial velocity, m/cs     */
    dvec_t r1; /* reference position, m      */
    dvec_t v1; /* reference velocity, m/cs   */

    fresh(AGC_VEHICLE_CSM);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    circular(MU_E, 6678137.0, 0.5, TEN_MIN_CS, &r1, &v1);
    CHECK(run_integrvs(&ctx, r0, v0, true, TEN_MIN_CS) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(ctx.out.tat == (agc_dp_t) TEN_MIN_CS);
    const double dr   = dist(from_agc(ctx.out.ratt, 29), r1); /* position error, m */
    const double dvel = dist(from_agc(ctx.out.vatt, 7), v1);  /* velocity error, m/cs */

    (void) printf("conic LEO 10 min: |dr| = %.1f m, |dv| = %.3g m/cs\n", dr, dvel);
    CHECK(dr < 60.0); /* AGC KEPLER itself: 53.4 m (oracle_conic kep_leo_10min, exact) */
    CHECK(dvel < 5e-4);
    CHECK(ctx.out.x1 == -2);
    CHECK(ctx.out.x2 == 0);
}

/**********************************************************************
 * TESTLOOP .. RECTOUT (INTEGRATION_INITIALIZATION.agc pp. 1320-1324) -- test
 * 4: two-body Encke integration (LM, 100000 km circular orbit) vs analytic
 * motion
 *
 * FUNCTION NAME: test_encke_twobody
 *********************************************************************/
static void test_encke_twobody(void)
{
    agc_integ_ctx_t ctx; /* integration erasables */

    dvec_t r0; /* initial position, m        */
    dvec_t v0; /* initial velocity, m/cs     */
    dvec_t r1; /* reference position, m      */
    dvec_t v1; /* reference velocity, m/cs   */

    fresh(AGC_VEHICLE_LM);
    agc_erase_set_modreg(20);
    circular(MU_E, 1.0e8, 0.3, 0.0, &r0, &v0);
    circular(MU_E, 1.0e8, 0.3, TEN_MIN_CS, &r1, &v1);
    CHECK(run_integrvs(&ctx, r0, v0, false, TEN_MIN_CS) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(!agc_flag_test(AGC_FLAG_MIDFLAG));                  /* LM: RME = DPPOSMAX */
    const double dr   = dist(from_agc(ctx.out.ratt, 29), r1); /* position error, m */
    const double dvel = dist(from_agc(ctx.out.vatt, 7), v1);  /* velocity error, m/cs */

    (void) printf("two-body Encke 10 min: |dr| = %.1f m, |dv| = %.3g m/cs\n", dr, dvel);
    CHECK(dr < 50.0);
    CHECK(dvel < 5e-4);
    CHECK(ctx.out.tat == (agc_dp_t) TEN_MIN_CS);
}

/**********************************************************************
 * OBLATE (ORBITAL_INTEGRATION.agc pp. 1341-1342) -- test 5: precision Encke
 * integration in LEO with J2..J4 vs an RK4 reference
 *
 * FUNCTION NAME: test_encke_leo
 *********************************************************************/
static void test_encke_leo(void)
{
    agc_integ_ctx_t ctx; /* integration erasables */

    dvec_t r0; /* initial position, m        */
    dvec_t v0; /* initial velocity, m/cs     */
    dvec_t r1; /* reference position, m      */
    dvec_t v1; /* reference velocity, m/cs   */

    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(20);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    circular(MU_E, 6678137.0, 0.5, TEN_MIN_CS, &r1, &v1);
    CHECK(run_integrvs(&ctx, r0, v0, false, TEN_MIN_CS) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(!agc_flag_test(AGC_FLAG_MIDFLAG));                         /* below RME         */
    dstate_t          s0   = {r0, v0};
    const ref_model_t m    = earth_model(true);                      /* earth with J2..J4 */
    const dstate_t    ref  = rk4(&m, s0, TEN_MIN_CS);                /* reference state at 10 min */
    const dvec_t      ra   = from_agc(ctx.out.ratt, 29);             /* integrated position, m */
    const double      dr   = dist(ra, ref.r);                        /* position error vs the reference, m */
    const double      dvel = dist(from_agc(ctx.out.vatt, 7), ref.v); /* velocity error vs the reference, m/cs */
    const double      d2b  = dist(ra, r1);                           /* distance from the two-body state, m */

    (void) printf("Encke LEO 10 min: |dr| vs J2 ref = %.1f m, |dv| = %.3g m/cs, " "vs two-body %.0f m\n", dr, dvel, d2b);
    CHECK(dr < 60.0);
    CHECK(dvel < 1e-3);
    CHECK((d2b > 200.0) && (d2b < 10000.0)); /* J2 is felt        */
}

/**********************************************************************
 * DOW.. / NEXTCOL (ORBITAL_INTEGRATION.agc pp. 1348-1352) -- test 6: a W
 * column propagated by the gravity gradient vs the finite-difference state
 * transition
 *
 * FUNCTION NAME: test_wmatrix
 *********************************************************************/
static void test_wmatrix(void)
{
    agc_interp_t     ip;                           /* interpretive state */
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* the integration erasables */

    dvec_t r0; /* initial position, m        */
    dvec_t v0; /* initial velocity, m/cs     */

    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(20);
    CHECK(agc_integ_ctx_init(ctx) == AGC_STATUS_OK);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    agc_integ_sv_t sv = ctx->sv; /* permanent CSM state with W */

    sv.rrect = to_agc(r0, 29);
    sv.rcv   = sv.rrect;
    sv.vrect = to_agc(v0, 7);
    sv.vcv   = sv.vrect;
    sv.tet   = 0;
    CHECK(agc_integ_perm_set(AGC_VEHICLE_CSM, &sv) == AGC_STATUS_OK);
    agc_flag_clear(AGC_FLAG_CMOONFLG);

    /* W column 0: 100 m along x at the stored scale (YV SL3: 2^19 m).    */
    ctx->w.blk[0][0] = to_agc(dv(100.0, 0.0, 0.0), 19);
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    agc_flag_set(AGC_FLAG_VINTFLAG);
    agc_flag_clear(AGC_FLAG_INTYPFLG);
    agc_flag_set(AGC_FLAG_DIM0FLAG);
    agc_flag_clear(AGC_FLAG_D6OR9FLG);
    agc_flag_set(AGC_FLAG_PRECIFLG);
    CHECK(agc_integ_integrv(&ip, ctx, (agc_dp_t) TEN_MIN_CS) == AGC_STATUS_OK);
    dstate_t a       = {r0, v0};
    dstate_t b       = a; /* state with the 100 m offset */

    b.r.x                  += 100.0;
    const ref_model_t m    = earth_model(false);                                    /* two-body earth */
    const dstate_t    ra   = rk4(&m, a, TEN_MIN_CS);                                /* reference from the state */
    const dstate_t    rb   = rk4(&m, b, TEN_MIN_CS);                                /* reference from the offset state */
    const dvec_t      want = dv(rb.r.x - ra.r.x, rb.r.y - ra.r.y, rb.r.z - ra.r.z); /* finite-difference transition, m */
    const dvec_t      got  = from_agc(ctx->w.blk[0][0], 19);                        /* W position column, m */

    (void) printf("W column 0 after 10 min: (%.2f %.2f %.2f) want (%.2f %.2f %.2f)\n", got.x, got.y, got.z, want.x, want.y, want.z);
    CHECK(dist(got, want) < 2.0);
    CHECK(agc_integ_perm_get(AGC_VEHICLE_CSM, &sv) == AGC_STATUS_OK);
    CHECK(sv.tet == (agc_dp_t) TEN_MIN_CS); /* permanent updated */
} /* test_wmatrix */

/**********************************************************************
 * QUALITY1-3 (ORBITAL_INTEGRATION.agc pp. 1342-1343) -- test 7: lunar orbit, 1
 * h, lunar J2 J3 J22 C31 vs a reference with the same physics
 *
 * FUNCTION NAME: test_lunar_gravity
 *********************************************************************/
static void test_lunar_gravity(void)
{
    agc_interp_t         ip;  /* interpretive state */
    agc_integ_ctx_t      ctx; /* integration erasables */
    agc_navaux_padload_t pad = {0};
    agc_mat_t            mm;  /* MOONMX, planet-fixed axes */

    dvec_t       r0;           /* initial position, m        */
    dvec_t       v0;           /* initial velocity, m/cs     */
    dvec_t       r1;           /* reference position, m      */
    dvec_t       v1;           /* reference velocity, m/cs   */
    const double t = 360000.0; /* 1 hour            */

    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(20);
    agc_navaux_set_padload(&pad);

    /* E3J22R2M = 3J22R2MU (9.20479048E16 B-58), E32C31RM ~ 3/2 C31 R^3 mu
     * for C31 = 2.8E-5 at the scale B80 implied by QUALITY3.             */
    const agc_sp_t e22 = 5232; /* E3J22R2M pad load, 2^58 */
    const agc_sp_t e31 = 1465; /* E32C31RM pad load, 2^80 */

    agc_integ_set_lunar_coeffs(e22, e31);
    CHECK(agc_planet_moonmx(&ip, &pad, 180000, &mm) == AGC_STATUS_OK);
    ref_model_t m = {0};

    m.mu  = MU_M;
    m.j2k = 0.3067493316e18;
    m.k3  = -176236.02;
    m.k4  = 0.0;

    for (int i = 0; i < 3; i++)
    {
        const dvec_t row = from_agc(mm.row[i], 1); /* MOONMX row */
        const double n   = sqrt(dot3(row, row));   /* |row|, 1 for a rotation */

        m.mrow[i][0] = row.x / n;
        m.mrow[i][1] = row.y / n;
        m.mrow[i][2] = row.z / n;
    }

    m.pole             = dv(m.mrow[2][0], m.mrow[2][1], m.mrow[2][2]);
    m.e22              = agc_sp_to_double(e22, 58);
    m.e31              = agc_sp_to_double(e31, 40) * pow(2.0, 40.0); /* B80 */
    circular(MU_M, 1.85e6, 1.2, 0.0, &r0, &v0);
    circular(MU_M, 1.85e6, 1.2, t, &r1, &v1);
    CHECK(run_body(&ctx, r0, v0, false, t, true) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(ctx.out.x1 == -10);
    CHECK(ctx.out.x2 == 2);
    dstate_t       s0  = {r0, v0};
    const dstate_t ref = rk4(&m, s0, t);                             /* reference with all terms */
    ref_model_t    m2  = m;                                          /* reference model without J22 / C31 */

    m2.k3               = 0.0;
    m2.e22              = 0.0;
    m2.e31              = 0.0;
    const dstate_t ref2 = rk4(&m2, s0, t);             /* reference without J22 / C31 */
    const dvec_t   ra   = from_agc(ctx.out.ratt1, 27); /* integrated position, m */

    (void) printf("lunar 1 h: |dr| vs ref = %.1f m (J2-only ref %.1f m, " "two-body %.0f m)\n", dist(ra, ref.r), dist(ra, ref2.r), dist(ra, r1));
    CHECK(dist(ra, ref.r) < 60.0);
    CHECK(dist(ref.r, ref2.r) > 150.0); /* J3/J22/C31 felt   */
    CHECK(dist(from_agc(ctx.out.vatt1, 5), ref.v) < 2e-3);
} /* test_lunar_gravity */

/**********************************************************************
 * ACCOMP third bodies (ORBITAL_INTEGRATION.agc pp. 1337-1338) -- test 8: CSM
 * at 60000 km (MIDFLAG), 6 h, moon and sun vs a reference
 *
 * FUNCTION NAME: test_third_body
 *********************************************************************/
static void test_third_body(void)
{
    agc_integ_ctx_t      ctx; /* integration erasables */
    agc_navaux_padload_t pad = {0};
    agc_ephem_csm_t      eph = {0};

    dvec_t       r0;            /* initial position, m        */
    dvec_t       v0;            /* initial velocity, m/cs     */
    const double t = 2160000.0; /* 6 hours           */

    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(20);
    agc_erase_set_tephem(0);
    agc_navaux_set_padload(&pad);
    const dvec_t moon = dv(3.8e8, 1.0e7, -2.0e7); /* moon position, m */
    const dvec_t sun  = dv(1.5e11, 0.0, 0.0);     /* sun position, m */

    eph.vecoem[AGC_EPHEM_NCOEF - 1] = to_agc(moon, 31);  /* a0, fixed    */
    eph.reso                        = to_agc(sun, 38);
    agc_ephem_set_csm_data(&eph);
    ref_model_t m                   = earth_model(true); /* earth with J2..J4 and both bodies */

    m.nbody             = 2;
    m.bpos[0]           = moon;
    m.bmu[0]            = MU_M;
    m.bpos[1]           = sun;
    m.bmu[1]            = MU_S;
    circular(MU_E, 6.0e7, 0.4, 0.0, &r0, &v0);
    CHECK(run_integrvs(&ctx, r0, v0, false, t) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(agc_flag_test(AGC_FLAG_MIDFLAG));           /* beyond RME        */
    CHECK(!agc_flag_test(AGC_FLAG_MOONFLAG));         /* no origin change  */
    dstate_t       s0   = {r0, v0};
    const dstate_t ref  = rk4(&m, s0, t);             /* reference with the third bodies */
    ref_model_t    m2   = earth_model(true);          /* reference model without them */
    const dstate_t ref2 = rk4(&m2, s0, t);            /* reference without the third bodies */
    const dvec_t   ra   = from_agc(ctx.out.ratt, 29); /* integrated position, m */

    (void) printf("third body 6 h: |dr| vs ref = %.1f m (without moon/sun %.0f m)\n", dist(ra, ref.r), dist(ra, ref2.r));
    CHECK(dist(ra, ref.r) < 300.0);
    CHECK(dist(ref.r, ref2.r) > 1000.0);
}

/**********************************************************************
 * MOVEACSM / MOVEALEM (INTEGRATION_INITIALIZATION.agc p. 1314) test helper --
 * permanent state of veh from (r, v) at t, earth scales
 *
 * FUNCTION NAME: set_perm
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------------
 *   veh       agc_vehicle_t  I    vehicle (CSM or LM)
 *   r         dvec_t         I    position, m
 *   v         dvec_t         I    velocity, m/cs
 *   t         agc_dp_t       I    TET, cs 2^28
 *********************************************************************/
static void set_perm(agc_vehicle_t veh, dvec_t r, dvec_t v, agc_dp_t t)
{
    agc_integ_sv_t sv = {0};

    sv.rrect = to_agc(r, 29);
    sv.rcv   = sv.rrect;
    sv.vrect = to_agc(v, 7);
    sv.vcv   = sv.vrect;
    sv.tet   = t;
    CHECK(agc_integ_perm_set(veh, &sv) == AGC_STATUS_OK);
}

/**********************************************************************
 * STATEINT / STATINT1 / STATEUP (INTEGRATION_INITIALIZATION.agc p. 1313) --
 * test 9: P00 integration of both vehicles, the 4-time-step logic, NOINT
 *
 * FUNCTION NAME: test_stateint
 *********************************************************************/
static void test_stateint(void)
{
    agc_integ_sv_t c; /* permanent CSM state */
    agc_integ_sv_t l; /* permanent LM state */

    dvec_t r0; /* initial position, m        */
    dvec_t v0; /* initial velocity, m/cs     */

    fresh(AGC_VEHICLE_CSM);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    set_perm(AGC_VEHICLE_CSM, r0, v0, 0);
    set_perm(AGC_VEHICLE_LM, r0, v0, 0);
    agc_time_set_cs(300000u);
    CHECK(agc_job_stateint_update() == AGC_STATUS_OK);
    CHECK(agc_integ_perm_get(AGC_VEHICLE_CSM, &c) == AGC_STATUS_OK);
    CHECK(agc_integ_perm_get(AGC_VEHICLE_LM, &l) == AGC_STATUS_OK);
    (void) printf("STATEINT: TETCSM = %d cs, TETLEM = %d cs (now 300000)\n", (int) c.tet, (int) l.tet);
    CHECK((c.tet > 200000) && (c.tet <= 300000)); /* last step left out */
    CHECK(l.tet == c.tet);                        /* LM to TETCSM      */
    CHECK(!agc_integ_area_busy(0));
    CHECK(!agc_flag_test(AGC_FLAG_STATEFLG));
    CHECK(!agc_flag_test(AGC_FLAG_NODOFLAG));
    agc_flag_set(AGC_FLAG_QUITFLAG);              /* V96: NOINT        */
    CHECK(agc_job_stateint_update() == AGC_STATUS_OK);
    CHECK(!agc_flag_test(AGC_FLAG_QUITFLAG));
}

/**********************************************************************
 * MIDTOAV1 (INTEGRATION_INITIALIZATION.agc pp. 1329-1331) -- test 10: to
 * TDEC1, and the NOTIME slip (ALARM 1703)
 *
 * FUNCTION NAME: test_midtoav
 *********************************************************************/
static void test_midtoav(void)
{
    agc_interp_t            ip;  /* interpretive state */
    agc_integ_midtoav_out_t out; /* MIDTOAV1 outputs */

    dvec_t r0; /* initial position, m        */
    dvec_t v0; /* initial velocity, m/cs     */
    dvec_t r1; /* reference position, m      */
    dvec_t v1; /* reference velocity, m/cs   */

    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(40);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    circular(MU_E, 6678137.0, 0.5, 30000.0, &r1, &v1);
    set_perm(AGC_VEHICLE_CSM, r0, v0, 0);
    CHECK(agc_integ_ctx_init(agc_integ_shared_ctx()) == AGC_STATUS_OK);
    CHECK(agc_integ_midtoav_at_time(&ip, agc_integ_shared_ctx(), 30000, AGC_JOB_NONE, &out) == AGC_STATUS_OK);
    CHECK(out.piptime1 == 30000);
    CHECK(out.dt == 30000);
    CHECK(!out.slipped);
    CHECK(out.rtx2 == 0);
    CHECK(dist(from_agc(out.rn1, 29), r1) < 2000.0);
    CHECK(!agc_flag_test(AGC_FLAG_MIDAVFLG));
    agc_time_set_cs(100000u);
    CHECK(agc_integ_midtoav_at_time(&ip, agc_integ_shared_ctx(), 100500, AGC_JOB_NONE, &out) == AGC_STATUS_OK);
    CHECK(out.slipped);            /* ALARM 1703        */
    CHECK(out.piptime1 == 101250); /* now + TIMEDELT    */
    CHECK(out.dt == 1250);
    CHECK(!agc_integ_area_busy(0));
}

/**********************************************************************
 * AVETOMID (INTEGRATION_INITIALIZATION.agc pp. 1327-1328) -- test 11: W
 * through the burn, RN VN PIPTIME into the CSM, LM along
 *
 * FUNCTION NAME: test_avetomid
 *********************************************************************/
static void test_avetomid(void)
{
    agc_interp_t     ip;                           /* interpretive state */
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* the integration erasables */
    agc_integ_sv_t   c;                            /* permanent CSM state */
    agc_integ_sv_t   l;                            /* permanent LM state */

    dvec_t r0; /* initial position, m        */
    dvec_t v0; /* initial velocity, m/cs     */
    dvec_t r1; /* reference position, m      */
    dvec_t v1; /* reference velocity, m/cs   */

    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(40);
    CHECK(agc_integ_ctx_init(ctx) == AGC_STATUS_OK);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    circular(MU_E, 6700000.0, 0.5, 60000.0, &r1, &v1);
    set_perm(AGC_VEHICLE_CSM, r0, v0, 0);
    set_perm(AGC_VEHICLE_LM, r0, v0, 0);
    agc_erase_set_rn(to_agc(r1, 29));
    agc_erase_set_vn(to_agc(v1, 7));
    agc_erase_set_piptime(60000);
    ctx->w.blk[0][0] = to_agc(dv(100.0, 0.0, 0.0), 19);
    agc_flag_set(AGC_FLAG_ORBWFLAG);
    CHECK(agc_integ_avetomid(&ip, ctx, AGC_JOB_NONE) == AGC_STATUS_OK);
    CHECK(agc_integ_perm_get(AGC_VEHICLE_CSM, &c) == AGC_STATUS_OK);
    CHECK(agc_integ_perm_get(AGC_VEHICLE_LM, &l) == AGC_STATUS_OK);
    CHECK(c.tet == 60000);
    CHECK(dist(from_agc(c.rrect, 29), r1) < 4.0);
    CHECK(l.tet == 60000);
    CHECK(fabs(agc_dp_to_double(ctx->w.blk[0][0].c[0], 19) - 146.0) < 1.0);
    CHECK(!agc_integ_area_busy(0));
    CHECK(!agc_flag_test(AGC_FLAG_AVEMIDSW));
    CHECK(!agc_flag_test(AGC_FLAG_CMOONFLG));
}

/**********************************************************************
 * INTWAKEU (INTEGRATION_INITIALIZATION.agc pp. 1332-1333) -- test 12: P27
 * update into the LM, then the CSM in the moon sphere
 *
 * FUNCTION NAME: test_intwakeu
 *********************************************************************/
static void test_intwakeu(void)
{
    agc_interp_t     ip;                           /* interpretive state */
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* the integration erasables */
    agc_integ_sv_t   l;                            /* permanent LM state */
    agc_sp_t         u   = -1;                     /* UPSVFLAG: LM, earth sphere (integer 2^14) */

    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_integ_ctx_init(ctx) == AGC_STATUS_OK);
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    ctx->sv.rrect = to_agc(dv(7.0e6, 1.0e5, 0.0), 29);
    ctx->sv.vrect = to_agc(dv(0.0, 75.0, 1.0), 7);
    ctx->sv.tet   = 5000;
    CHECK(agc_integ_intwakeu(&ip, ctx, &u, AGC_JOB_UPLINK_UPDATE_TERMINATE) == AGC_STATUS_OK);
    CHECK(u == 0);
    CHECK(!agc_integ_area_busy(0));
    CHECK(agc_integ_perm_get(AGC_VEHICLE_LM, &l) == AGC_STATUS_OK);
    CHECK(l.tet == 5000);
    CHECK(l.rcv.c[0] == l.rrect.c[0]);
    CHECK(!agc_flag_test(AGC_FLAG_LMOONFLG));
    u             = 2;
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    CHECK(agc_integ_intwakeu(&ip, ctx, &u, AGC_JOB_UPLINK_UPDATE_TERMINATE) == AGC_STATUS_OK);
    CHECK(agc_flag_test(AGC_FLAG_CMOONFLG));
    CHECK(agc_flag_test(AGC_FLAG_MOONFLAG));

    /* PHASCHNG 04026: a group 6 restart resumes at the caller's
     * continuation (P27: UPOUT +1), at the preset PHSPRDT6.            */
    bool resumed = false; /* the terminate job was started */

    agc_phase_set_priority(6, 030);
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    CHECK(agc_integ_intwakeu(&ip, ctx, &u, AGC_JOB_UPLINK_UPDATE_TERMINATE) == AGC_STATUS_OK);
    agc_exec_init();
    CHECK(agc_restart_group(6) == AGC_STATUS_OK);

    for (int i = 0; i < agc_exec_core_sets(); i++)
    {
        agc_exec_core_info_t ci; /* core set i */

        agc_exec_core_info(i, &ci);
        resumed = resumed || (ci.loc == AGC_JOB_UPLINK_UPDATE_TERMINATE && ci.prio == 030);
    }

    CHECK(resumed);
}
