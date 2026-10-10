/*
 * integration.c -- nav-integ unit tests (INTEGRATION_INITIALIZATION +
 * ORBITAL_INTEGRATION translation).
 *
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
 * Units: positions m scale 2^29, velocities m/cs scale 2^7, time cs.
 */
#include <stdio.h>

#include <math.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"

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

static const double MU_E = 3.986032e10;   /* MUEARTH m^3/cs^2          */
static const double J2K  = 1.75501139e21; /* J2REQSQ m^5/cs^2           */
static const double MU_M = 4.9027780e8;   /* MUM m^3/cs^2               */
static const double MU_S = 1.32715445e16; /* sun m^3/cs^2               */
static const double PI   = 3.14159265358979323846;
#define TEN_MIN_CS 60000.0

typedef struct dvec
{
    double x, y, z;
} dvec_t;

static dvec_t dv(double x, double y, double z)
{
    dvec_t r = {x, y, z};
    return r;
}

static agc_vec_t to_agc(dvec_t v, int scale)
{
    return agc_vec_make(agc_dp_from_double(v.x, scale),
                        agc_dp_from_double(v.y, scale),
                        agc_dp_from_double(v.z, scale));
}

static dvec_t from_agc(agc_vec_t v, int scale)
{
    return dv(agc_dp_to_double(v.c[0], scale), agc_dp_to_double(v.c[1], scale),
              agc_dp_to_double(v.c[2], scale));
}

static double dist(dvec_t a, dvec_t b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

/* Circular orbit in the plane of x and (y cos i + z sin i).             */
static void circular(double mu, double r, double incl, double t, dvec_t *pos,
                     dvec_t *vel)
{
    const double n = sqrt(mu / (r * r * r));
    const double v = n * r;
    const double c = cos(n * t);
    const double s = sin(n * t);
    *pos           = dv(r * c, r * s * cos(incl), r * s * sin(incl));
    *vel           = dv(-v * s, v * c * cos(incl), v * c * sin(incl));
}

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
    dvec_t bpos[2];
    double bmu[2];
} ref_model_t;

typedef struct dstate
{
    dvec_t r, v;
} dstate_t;

static double dot3(dvec_t a, dvec_t b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

static dvec_t axpy(double a, dvec_t x, dvec_t y)
{
    return dv((a * x.x) + y.x, (a * x.y) + y.y, (a * x.z) + y.z);
}

/* Zonal J2..J4 in the AGC's form (COMTERM): (J2REQSQ/r^4) [(P3' + k3/r
 * (P4' + k4/r P5')) u - (P2' + k3/r (P3' + k4/r P4')) pole].            */
static dvec_t zonal(const ref_model_t *m, dvec_t r, double rr)
{
    const dvec_t u  = dv(r.x / rr, r.y / rr, r.z / rr);
    const double c  = dot3(u, m->pole);
    const double p2 = 3.0 * c;
    const double p3 = ((15.0 * c * c) - 3.0) / 2.0;
    const double p4 = ((35.0 * c * c * c) - (15.0 * c)) / 2.0;
    const double p5 = ((315.0 * c * c * c * c) - (210.0 * c * c) + 15.0) / 8.0;
    const double f  = m->j2k / (rr * rr * rr * rr);
    const double cu = p3 + ((m->k3 / rr) * (p4 + ((m->k4 / rr) * p5)));
    const double cz = p2 + ((m->k3 / rr) * (p3 + ((m->k4 / rr) * p4)));
    return axpy(-f * cz, m->pole, dv(f * cu * u.x, f * cu * u.y, f * cu * u.z));
}

/* Lunar J22 and C31 terms (QUALITY2, QUALITY3) in planet-fixed axes.     */
static dvec_t tesseral(const ref_model_t *m, dvec_t r, double rr)
{
    const dvec_t u = dv(r.x / rr, r.y / rr, r.z / rr);
    const double x =
        (m->mrow[0][0] * u.x) + (m->mrow[0][1] * u.y) + (m->mrow[0][2] * u.z);
    const double y =
        (m->mrow[1][0] * u.x) + (m->mrow[1][1] * u.y) + (m->mrow[1][2] * u.z);
    const double z =
        (m->mrow[2][0] * u.x) + (m->mrow[2][1] * u.y) + (m->mrow[2][2] * u.z);
    const double r4   = rr * rr * rr * rr;
    const double g    = m->e22 / r4;
    const double q    = 5.0 * ((x * x) - (y * y));
    double       a[3] = {g * ((2.0 * x) - (q * x)), g * ((-2.0 * y) - (q * y)),
                         g * (-q * z)};
    const double h    = m->e31 / (r4 * rr);
    const double w    = 5.0 * x * (1.0 - (7.0 * z * z));
    a[0] += h * ((w * x) + (5.0 * z * z) - 1.0);
    a[1] += h * (w * y);
    a[2] += h * ((w * z) + (10.0 * x * z));
    dvec_t o = dv(0.0, 0.0, 0.0);
    for (int i = 0; i < 3; i++)
    {
        o = axpy(a[i], dv(m->mrow[i][0], m->mrow[i][1], m->mrow[i][2]), o);
    }
    return o;
}

static dvec_t accel(const ref_model_t *m, dvec_t r)
{
    const double rr = sqrt(dot3(r, r));
    dvec_t a = dv(-m->mu * r.x / (rr * rr * rr), -m->mu * r.y / (rr * rr * rr),
                  -m->mu * r.z / (rr * rr * rr));
    if (m->j2k != 0.0)
    {
        const dvec_t z = zonal(m, r, rr);
        a              = axpy(1.0, z, a);
        a              = axpy(1.0, tesseral(m, r, rr), a);
    }
    for (int i = 0; i < m->nbody; i++)
    {
        const dvec_t d  = axpy(-1.0, r, m->bpos[i]); /* body - r  */
        const double dd = sqrt(dot3(d, d));
        const double bb = sqrt(dot3(m->bpos[i], m->bpos[i]));
        a               = axpy(m->bmu[i] / (dd * dd * dd), d, a);
        a               = axpy(-m->bmu[i] / (bb * bb * bb), m->bpos[i], a);
    }
    return a;
}

static dstate_t deriv(const ref_model_t *m, dstate_t s, double h, dstate_t d)
{
    dstate_t x;
    x.r = axpy(h, d.r, s.r);
    x.v = axpy(h, d.v, s.v);
    dstate_t o;
    o.r = x.v;
    o.v = accel(m, x.r);
    return o;
}

static dstate_t rk4(const ref_model_t *m, dstate_t s0, double t)
{
    const int      n    = 6000;
    const double   h    = t / n;
    const dstate_t zero = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    dstate_t       s    = s0;
    for (int i = 0; i < n; i++)
    {
        const dstate_t k1 = deriv(m, s, 0.0, zero);
        const dstate_t k2 = deriv(m, s, h / 2.0, k1);
        const dstate_t k3 = deriv(m, s, h / 2.0, k2);
        const dstate_t k4 = deriv(m, s, h, k3);
        s.r =
            axpy(h / 6.0,
                 axpy(1.0, axpy(2.0, k2.r, k1.r), axpy(2.0, k3.r, k4.r)), s.r);
        s.v =
            axpy(h / 6.0,
                 axpy(1.0, axpy(2.0, k2.v, k1.v), axpy(2.0, k3.v, k4.v)), s.v);
    }
    return s;
}

static ref_model_t earth_model(bool j2)
{
    ref_model_t m = {0};
    m.mu          = MU_E;
    m.j2k         = j2 ? J2K : 0.0;
    m.k3          = -.1355426363e5;
    m.k4          = .4991607391e7;
    m.pole        = dv(0.0, 0.0, 1.0);
    m.mrow[0][0]  = 1.0;
    m.mrow[1][1]  = 1.0;
    m.mrow[2][2]  = 1.0;
    return m;
}

/* ---- setup ------------------------------------------------------------ */
static void fresh(agc_vehicle_t veh)
{
    agc_state_init(veh);
    CHECK(agc_exec_init() == AGC_STATUS_OK);
    CHECK(agc_waitlist_init() == AGC_STATUS_OK);
    agc_erase_set_modreg(0);
}

/* INTEGRVS from (r, v) at t = 0 to t1 about the earth or the moon (the
 * input at that body's scales); result in ctx->out.                     */
static agc_status_t run_body(agc_integ_ctx_t *ctx, dvec_t r, dvec_t v,
                             bool conic, double t1, bool moon)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    CHECK(agc_integ_ctx_init(ctx) == AGC_STATUS_OK);
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    ctx->sv.rcv = to_agc(r, moon ? 27 : 29);
    ctx->sv.vcv = to_agc(v, moon ? 5 : 7);
    ctx->sv.tet = 0;
    agc_flag_put(AGC_FLAG_MOONFLAG, moon);
    agc_flag_put(AGC_FLAG_INTYPFLG, conic);
    const agc_status_t st = agc_integ_integrvs(&ip, ctx, (agc_dp_t) t1);
    CHECK(!agc_integ_area_busy(0)); /* INTWAKE released */
    return st;
}

static agc_status_t run_integrvs(agc_integ_ctx_t *ctx, dvec_t r, dvec_t v,
                                 bool conic, double t1)
{
    return run_body(ctx, r, v, conic, t1, false);
}

/* ---- 1. lock ---------------------------------------------------------- */
static void test_lock(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(!agc_integ_area_busy(0));
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    CHECK(agc_integ_area_busy(0));
    CHECK(agc_flag_test(AGC_FLAG_INTFLAG)); /* RASFLAG BIT14     */
    /* busy, and not called from a job: JOBSLEEP refuses, nothing queued */
    CHECK(agc_integ_install(0, (agc_job_id_t) 0x0F01) == AGC_STATUS_BAD_ARG);
    CHECK(agc_integ_install(1, AGC_JOB_NONE) == AGC_STATUS_OK); /* ERASTAL1 */
    CHECK(agc_integ_area_busy(1));
    CHECK(agc_integ_eraswake(1) == AGC_STATUS_OK);
    CHECK(!agc_integ_area_busy(1));
    CHECK(agc_integ_intwake() == AGC_STATUS_OK);
    CHECK(!agc_integ_area_busy(0));
    CHECK(!agc_flag_test(AGC_FLAG_INTFLAG));
    fresh(AGC_VEHICLE_LM);
    CHECK(agc_integ_install(1, AGC_JOB_NONE) ==
          AGC_STATUS_BAD_ARG); /* LM: 1 area */
}

/* ---- 2. RECTIFY ------------------------------------------------------- */
static void test_rectify(void)
{
    agc_interp_t    ip;
    agc_integ_ctx_t ctx;
    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_integ_ctx_init(&ctx) == AGC_STATUS_OK);
    ctx.sv.rcv    = to_agc(dv(7.0e6, 0.0, 0.0), 29);
    ctx.sv.vcv    = to_agc(dv(0.0, 75.0, 0.0), 7);
    ctx.sv.deltav = to_agc(dv(1000.0, -500.0, 0.0), 22); /* E 2^22 m    */
    ctx.sv.nuv    = to_agc(dv(0.0, 0.0, 0.05), 3);       /* E 2^3 m/cs  */
    ctx.sv.tc     = 1234;
    ctx.sv.xkep   = 99;
    CHECK(agc_integ_rectify(&ip, &ctx) == AGC_STATUS_OK);
    const dvec_t r = from_agc(ctx.sv.rrect, 29);
    const dvec_t v = from_agc(ctx.sv.vrect, 7);
    CHECK(fabs(r.x - 7001000.0) < 4.0);
    CHECK(fabs(r.y + 500.0) < 4.0);
    CHECK(fabs(v.z - 0.05) < 1e-6);
    CHECK(ctx.sv.tc == 0);
    CHECK(ctx.sv.xkep == 0);
    CHECK(agc_vec_abval(&ip, ctx.sv.deltav) == 0);
}

/* ---- 3. conic LEO ----------------------------------------------------- */
static void test_conic_leo(void)
{
    agc_integ_ctx_t ctx;
    dvec_t          r0, v0, r1, v1;
    fresh(AGC_VEHICLE_CSM);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    circular(MU_E, 6678137.0, 0.5, TEN_MIN_CS, &r1, &v1);
    CHECK(run_integrvs(&ctx, r0, v0, true, TEN_MIN_CS) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(ctx.out.tat == (agc_dp_t) TEN_MIN_CS);
    const double dr   = dist(from_agc(ctx.out.ratt, 29), r1);
    const double dvel = dist(from_agc(ctx.out.vatt, 7), v1);
    (void) printf("conic LEO 10 min: |dr| = %.1f m, |dv| = %.3g m/cs\n", dr,
                  dvel);
    CHECK(dr < 50.0);
    CHECK(dvel < 5e-4);
    CHECK(ctx.out.x1 == -2);
    CHECK(ctx.out.x2 == 0);
}

/* ---- 4. two-body Encke (LM, high earth orbit) ------------------------- */
static void test_encke_twobody(void)
{
    agc_integ_ctx_t ctx;
    dvec_t          r0, v0, r1, v1;
    fresh(AGC_VEHICLE_LM);
    agc_erase_set_modreg(20);
    circular(MU_E, 1.0e8, 0.3, 0.0, &r0, &v0);
    circular(MU_E, 1.0e8, 0.3, TEN_MIN_CS, &r1, &v1);
    CHECK(run_integrvs(&ctx, r0, v0, false, TEN_MIN_CS) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(!agc_flag_test(AGC_FLAG_MIDFLAG)); /* LM: RME = DPPOSMAX */
    const double dr   = dist(from_agc(ctx.out.ratt, 29), r1);
    const double dvel = dist(from_agc(ctx.out.vatt, 7), v1);
    (void) printf("two-body Encke 10 min: |dr| = %.1f m, |dv| = %.3g m/cs\n",
                  dr, dvel);
    CHECK(dr < 50.0);
    CHECK(dvel < 5e-4);
    CHECK(ctx.out.tat == (agc_dp_t) TEN_MIN_CS);
}

/* ---- 5. precision Encke LEO with J2 ----------------------------------- */
static void test_encke_leo(void)
{
    agc_integ_ctx_t ctx;
    dvec_t          r0, v0, r1, v1;
    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(20);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    circular(MU_E, 6678137.0, 0.5, TEN_MIN_CS, &r1, &v1);
    CHECK(run_integrvs(&ctx, r0, v0, false, TEN_MIN_CS) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(!agc_flag_test(AGC_FLAG_MIDFLAG)); /* below RME         */
    dstate_t          s0   = {r0, v0};
    const ref_model_t m    = earth_model(true);
    const dstate_t    ref  = rk4(&m, s0, TEN_MIN_CS);
    const dvec_t      ra   = from_agc(ctx.out.ratt, 29);
    const double      dr   = dist(ra, ref.r);
    const double      dvel = dist(from_agc(ctx.out.vatt, 7), ref.v);
    const double      d2b  = dist(ra, r1);
    (void) printf(
        "Encke LEO 10 min: |dr| vs J2 ref = %.1f m, |dv| = %.3g m/cs, "
        "vs two-body %.0f m\n",
        dr, dvel, d2b);
    CHECK(dr < 60.0);
    CHECK(dvel < 1e-3);
    CHECK((d2b > 200.0) && (d2b < 10000.0)); /* J2 is felt        */
}

/* ---- 6. W matrix ------------------------------------------------------ */
static void test_wmatrix(void)
{
    agc_interp_t     ip;
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    dvec_t           r0, v0;
    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(20);
    CHECK(agc_integ_ctx_init(ctx) == AGC_STATUS_OK);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    agc_integ_sv_t sv = ctx->sv;
    sv.rrect          = to_agc(r0, 29);
    sv.rcv            = sv.rrect;
    sv.vrect          = to_agc(v0, 7);
    sv.vcv            = sv.vrect;
    sv.tet            = 0;
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
    dstate_t a = {r0, v0};
    dstate_t b = a;
    b.r.x += 100.0;
    const ref_model_t m  = earth_model(false);
    const dstate_t    ra = rk4(&m, a, TEN_MIN_CS);
    const dstate_t    rb = rk4(&m, b, TEN_MIN_CS);
    const dvec_t want = dv(rb.r.x - ra.r.x, rb.r.y - ra.r.y, rb.r.z - ra.r.z);
    const dvec_t got  = from_agc(ctx->w.blk[0][0], 19);
    (void) printf(
        "W column 0 after 10 min: (%.2f %.2f %.2f) want (%.2f %.2f %.2f)\n",
        got.x, got.y, got.z, want.x, want.y, want.z);
    CHECK(dist(got, want) < 2.0);
    CHECK(agc_integ_perm_get(AGC_VEHICLE_CSM, &sv) == AGC_STATUS_OK);
    CHECK(sv.tet == (agc_dp_t) TEN_MIN_CS); /* permanent updated */
}

/* ---- 7. lunar orbit: lunar J2 J3 and the J22 / C31 pad-load terms ---- */
static void test_lunar_gravity(void)
{
    agc_interp_t         ip;
    agc_integ_ctx_t      ctx;
    agc_navaux_padload_t pad = {0};
    agc_mat_t            mm;
    dvec_t               r0, v0, r1, v1;
    const double         t = 360000.0; /* 1 hour            */
    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(20);
    CHECK(agc_navaux_set_padload(&pad) == AGC_STATUS_OK);
    /* E3J22R2M = 3J22R2MU (9.20479048E16 B-58), E32C31RM ~ 3/2 C31 R^3 mu
     * for C31 = 2.8E-5 at the scale B80 implied by QUALITY3.             */
    const agc_sp_t e22 = 5232;
    const agc_sp_t e31 = 1465;
    CHECK(agc_integ_set_lunar_coeffs(e22, e31) == AGC_STATUS_OK);
    CHECK(agc_planet_moonmx(&ip, &pad, 180000, &mm) == AGC_STATUS_OK);
    ref_model_t m = {0};
    m.mu          = MU_M;
    m.j2k         = .3067493316e18;
    m.k3          = -176236.02;
    m.k4          = 0.0;
    for (int i = 0; i < 3; i++)
    {
        const dvec_t row = from_agc(mm.row[i], 1);
        const double n   = sqrt(dot3(row, row));
        m.mrow[i][0]     = row.x / n;
        m.mrow[i][1]     = row.y / n;
        m.mrow[i][2]     = row.z / n;
    }
    m.pole = dv(m.mrow[2][0], m.mrow[2][1], m.mrow[2][2]);
    m.e22  = agc_sp_to_double(e22, 58);
    m.e31  = agc_sp_to_double(e31, 40) * pow(2.0, 40.0); /* B80 */
    circular(MU_M, 1.85e6, 1.2, 0.0, &r0, &v0);
    circular(MU_M, 1.85e6, 1.2, t, &r1, &v1);
    CHECK(run_body(&ctx, r0, v0, false, t, true) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(ctx.out.x1 == -10);
    CHECK(ctx.out.x2 == 2);
    dstate_t       s0   = {r0, v0};
    const dstate_t ref  = rk4(&m, s0, t);
    ref_model_t    m2   = m;
    m2.k3               = 0.0;
    m2.e22              = 0.0;
    m2.e31              = 0.0;
    const dstate_t ref2 = rk4(&m2, s0, t);
    const dvec_t   ra   = from_agc(ctx.out.ratt1, 27);
    (void) printf("lunar 1 h: |dr| vs ref = %.1f m (J2-only ref %.1f m, "
                  "two-body %.0f m)\n",
                  dist(ra, ref.r), dist(ra, ref2.r), dist(ra, r1));
    CHECK(dist(ra, ref.r) < 60.0);
    CHECK(dist(ref.r, ref2.r) > 150.0); /* J3/J22/C31 felt   */
    CHECK(dist(from_agc(ctx.out.vatt1, 5), ref.v) < 2e-3);
}

/* ---- 8. third bodies: CSM in a 60000 km earth orbit (MIDFLAG) --------- */
static void test_third_body(void)
{
    agc_integ_ctx_t      ctx;
    agc_navaux_padload_t pad = {0};
    agc_ephem_csm_t      eph = {0};
    dvec_t               r0, v0;
    const double         t = 2160000.0; /* 6 hours           */
    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(20);
    agc_erase_set_tephem(0);
    CHECK(agc_navaux_set_padload(&pad) == AGC_STATUS_OK);
    const dvec_t moon               = dv(3.8e8, 1.0e7, -2.0e7);
    const dvec_t sun                = dv(1.5e11, 0.0, 0.0);
    eph.vecoem[AGC_EPHEM_NCOEF - 1] = to_agc(moon, 31); /* a0, fixed    */
    eph.reso                        = to_agc(sun, 38);
    CHECK(agc_ephem_set_csm_data(&eph) == AGC_STATUS_OK);
    ref_model_t m = earth_model(true);
    m.nbody       = 2;
    m.bpos[0]     = moon;
    m.bmu[0]      = MU_M;
    m.bpos[1]     = sun;
    m.bmu[1]      = MU_S;
    circular(MU_E, 6.0e7, 0.4, 0.0, &r0, &v0);
    CHECK(run_integrvs(&ctx, r0, v0, false, t) == AGC_STATUS_OK);
    CHECK(ctx.out.valid);
    CHECK(agc_flag_test(AGC_FLAG_MIDFLAG));   /* beyond RME        */
    CHECK(!agc_flag_test(AGC_FLAG_MOONFLAG)); /* no origin change  */
    dstate_t       s0   = {r0, v0};
    const dstate_t ref  = rk4(&m, s0, t);
    ref_model_t    m2   = earth_model(true);
    const dstate_t ref2 = rk4(&m2, s0, t);
    const dvec_t   ra   = from_agc(ctx.out.ratt, 29);
    (void) printf(
        "third body 6 h: |dr| vs ref = %.1f m (without moon/sun %.0f m)\n",
        dist(ra, ref.r), dist(ra, ref2.r));
    CHECK(dist(ra, ref.r) < 300.0);
    CHECK(dist(ref.r, ref2.r) > 1000.0);
}

/* Permanent state of `veh` from (r, v) at t, earth scales.               */
static void set_perm(agc_vehicle_t veh, dvec_t r, dvec_t v, agc_dp_t t)
{
    agc_integ_sv_t sv = {0};
    sv.rrect          = to_agc(r, 29);
    sv.rcv            = sv.rrect;
    sv.vrect          = to_agc(v, 7);
    sv.vcv            = sv.vrect;
    sv.tet            = t;
    CHECK(agc_integ_perm_set(veh, &sv) == AGC_STATUS_OK);
}

/* ---- 9. STATEINT: STATINT1 job, P00 4-time-step logic ---------------- */
static void test_stateint(void)
{
    agc_job_ctx_t  jc = {0};
    agc_integ_sv_t c;
    agc_integ_sv_t l;
    dvec_t         r0, v0;
    fresh(AGC_VEHICLE_CSM);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    set_perm(AGC_VEHICLE_CSM, r0, v0, 0);
    set_perm(AGC_VEHICLE_LM, r0, v0, 0);
    agc_time_set_cs(300000u);
    CHECK(agc_job_statint1(&jc) == AGC_STATUS_OK);
    CHECK(agc_integ_perm_get(AGC_VEHICLE_CSM, &c) == AGC_STATUS_OK);
    CHECK(agc_integ_perm_get(AGC_VEHICLE_LM, &l) == AGC_STATUS_OK);
    (void) printf("STATEINT: TETCSM = %d cs, TETLEM = %d cs (now 300000)\n",
                  (int) c.tet, (int) l.tet);
    CHECK((c.tet > 200000) && (c.tet <= 300000)); /* last step left out */
    CHECK(l.tet == c.tet);                        /* LM to TETCSM      */
    CHECK(!agc_integ_area_busy(0));
    CHECK(!agc_flag_test(AGC_FLAG_STATEFLG));
    CHECK(!agc_flag_test(AGC_FLAG_NODOFLAG));
    agc_flag_set(AGC_FLAG_QUITFLAG); /* V96: NOINT        */
    CHECK(agc_job_statint1(&jc) == AGC_STATUS_OK);
    CHECK(!agc_flag_test(AGC_FLAG_QUITFLAG));
}

/* ---- 10. MIDTOAV1: to TDEC1, and the NOTIME slip ---------------------- */
static void test_midtoav(void)
{
    agc_interp_t            ip;
    agc_integ_midtoav_out_t out;
    dvec_t                  r0, v0, r1, v1;
    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    agc_erase_set_modreg(40);
    circular(MU_E, 6678137.0, 0.5, 0.0, &r0, &v0);
    circular(MU_E, 6678137.0, 0.5, 30000.0, &r1, &v1);
    set_perm(AGC_VEHICLE_CSM, r0, v0, 0);
    CHECK(agc_integ_ctx_init(agc_integ_shared_ctx()) == AGC_STATUS_OK);
    CHECK(agc_integ_midtoav1(&ip, agc_integ_shared_ctx(), 30000, AGC_JOB_NONE,
                             &out) == AGC_STATUS_OK);
    CHECK(out.piptime1 == 30000);
    CHECK(out.dt == 30000);
    CHECK(!out.slipped);
    CHECK(out.rtx2 == 0);
    CHECK(dist(from_agc(out.rn1, 29), r1) < 2000.0);
    CHECK(!agc_flag_test(AGC_FLAG_MIDAVFLG));
    agc_time_set_cs(100000u);
    CHECK(agc_integ_midtoav1(&ip, agc_integ_shared_ctx(), 100500, AGC_JOB_NONE,
                             &out) == AGC_STATUS_OK);
    CHECK(out.slipped);            /* ALARM 1703        */
    CHECK(out.piptime1 == 101250); /* now + TIMEDELT    */
    CHECK(out.dt == 1250);
    CHECK(!agc_integ_area_busy(0));
}

/* ---- 11. AVETOMID: W through the burn, RN VN into the CSM, LM along --- */
static void test_avetomid(void)
{
    agc_interp_t     ip;
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    agc_integ_sv_t   c;
    agc_integ_sv_t   l;
    dvec_t           r0, v0, r1, v1;
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

/* ---- 12. INTWAKEU: P27 update into the LM / the CSM (moon) ----------- */
static void test_intwakeu(void)
{
    agc_interp_t     ip;
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    agc_integ_sv_t   l;
    agc_sp_t         u = -1;
    agc_interp_init(&ip);
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_integ_ctx_init(ctx) == AGC_STATUS_OK);
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    ctx->sv.rrect = to_agc(dv(7.0e6, 1.0e5, 0.0), 29);
    ctx->sv.vrect = to_agc(dv(0.0, 75.0, 1.0), 7);
    ctx->sv.tet   = 5000;
    CHECK(agc_integ_intwakeu(&ip, ctx, &u) == AGC_STATUS_OK);
    CHECK(u == 0);
    CHECK(!agc_integ_area_busy(0));
    CHECK(agc_integ_perm_get(AGC_VEHICLE_LM, &l) == AGC_STATUS_OK);
    CHECK(l.tet == 5000);
    CHECK(l.rcv.c[0] == l.rrect.c[0]);
    CHECK(!agc_flag_test(AGC_FLAG_LMOONFLG));
    u = 2;
    CHECK(agc_integ_install(0, AGC_JOB_NONE) == AGC_STATUS_OK);
    CHECK(agc_integ_intwakeu(&ip, ctx, &u) == AGC_STATUS_OK);
    CHECK(agc_flag_test(AGC_FLAG_CMOONFLG));
    CHECK(agc_flag_test(AGC_FLAG_MOONFLAG));
}

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
    (void) printf(
        "test_integration: %d checks, %d failures, %d assert reports\n",
        g_checks, g_failures, g_assert_reports);
    (void) PI;
    return (g_failures == 0) ? 0 : 1;
}
