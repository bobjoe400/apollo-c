/**********************************************************************
 * FILE NAME: auxiliary.c
 *
 * PURPOSE:
 *   nav-aux unit tests: latitude/longitude, planetary orientation, time
 *   of free fall, measurement incorporation, ephemerides, star table.
 *   Expected values are computed in double precision from the textbook
 *   formulas the AGC routines implement, with the AGC's own constants
 *   (WEARTH, modified earth MU from 1/RTMU), and compared with
 *   tolerances that allow for 28-bit fixed point.  Bit-exact comparisons
 *   are in tests/oracle/nav_auxiliary.c.
 *
 * AGC SOURCE:
 *   none (tests of src/nav/auxiliary/, whose prologs cite the listings)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_latlong_*, agc_planet_*, agc_tff_*, agc_incorp_*, agc_ephem_*,
 *   agc_star_vector       the functions under test (auxiliary.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails.
 **********************************************************************/
#include <stdio.h>

#include <math.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/nav/auxiliary.h"

#define CHECK(c)              check((c), __LINE__, #c)
#define CHECK_NEAR(a, b, tol) check_near((a), (b), (tol), __LINE__, #a)

static int g_checks;         /* checks run                          */
static int g_failures;       /* checks failed                       */
static int g_assert_reports; /* AGC_ASSERT failures reported        */

static const double TWO_PI = 6.283185307179586; /* 2 pi             */

/* WEARTH 2DEC .973561595 at 2^-23 rev/cs (PLANETARY_INERTIAL_ORIENTATION). */
static const double WEARTH_REV_CS = 0.973561595 / 8388608.0; /* rev/cs */

/* Modified earth MU of TIME_OF_FREE_FALL (1/RTMU = .5005750271E-5).        */
static const double TFF_RTMU = 0.5005750271e-5; /* 1/sqrt(MU), cs/m^1.5 */

/* Prototypes: main() comes first (STYLE 4.6). */
static void                 check(bool ok, int line, const char *text);
static void                 check_near(double got, double want, double tol, int line, const char *text);
static agc_vec_t            vec_m(double x, double y, double z, int scale);
static double               vec_comp(agc_vec_t v, int i, int scale);
static double               rev_wrap(double x);
static agc_navaux_padload_t zero_pad(void);
static void                 test_latlong_equator(void);
static void                 test_latlong_round_trip(void);
static void                 test_rp_round_trip(void);
static agc_tff_t            tff_state(double r, double v);
static void                 test_tff_circular(void);
static void                 test_tff_ellipse(void);
static void                 test_tff_poly(void);
static void                 test_star_table(void);
static void                 test_incorp(void);
static void                 test_ephemerides(void);

/**********************************************************************
 * AGC: none (test driver) -- run every nav-aux unit test
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    test_latlong_equator();
    test_latlong_round_trip();
    test_rp_round_trip();
    test_tff_circular();
    test_tff_ellipse();
    test_tff_poly();
    test_star_table();
    test_incorp();
    test_ephemerides();
    (void) printf("test_navaux: %d checks, %d failures\n", g_checks, g_failures);

    if (g_failures != 0)
    {
        return (1);
    }

    return (0);
}

/**********************************************************************
 * AGC: none (test hook replacing core/assert.c)
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description
 *   --------  ------------  ---  ---------------------------------
 *   file      const char *  I    source file of the failed assertion
 *   line      int           I    source line
 *   expr      const char *  I    assertion text
 *
 * RETURNS: bool -- false (the assertion stays failed); counts reports
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
 * AGC: none (test helper) -- record one boolean check
 *
 * FUNCTION NAME: check
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description
 *   --------  ------------  ---  ---------------------------------
 *   ok        bool          I    check result
 *   line      int           I    source line of the check
 *   text      const char *  I    check text, printed on failure
 *
 * RETURNS: void
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
 * AGC: none (test helper) -- record |got - want| <= tol
 *
 * FUNCTION NAME: check_near
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description
 *   --------  ------------  ---  ---------------------------------
 *   got       double        I    computed value
 *   want      double        I    expected value
 *   tol       double        I    tolerance
 *   line      int           I    source line of the check
 *   text      const char *  I    check text, printed on failure
 *
 * RETURNS: void
 **********************************************************************/
static void check_near(double got, double want, double tol, int line, const char *text)
{
    g_checks++;

    if (!(fabs(got - want) <= tol))
    {
        g_failures++;
        (void) printf("FAIL line %d: %s = %.10g, want %.10g (tol %.3g)\n", line, text, got, want, tol);
    }
}

/**********************************************************************
 * AGC: none (test helper) -- vector from doubles
 *
 * FUNCTION NAME: vec_m
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description
 *   --------  ------  ---  ---------------------------------
 *   x, y, z   double  I    components, engineering units
 *   scale     int     I    binary scale N (value = fraction 2^N)
 *
 * RETURNS: agc_vec_t -- the vector at scale 2^scale
 **********************************************************************/
static agc_vec_t vec_m(double x, double y, double z, int scale)
{
    return (agc_vec_make(agc_dp_from_double(x, scale),
                         agc_dp_from_double(y, scale),
                         agc_dp_from_double(z, scale)));
}

/**********************************************************************
 * AGC: none (test helper) -- one vector component as a double
 *
 * FUNCTION NAME: vec_comp
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description
 *   --------  ---------  ---  ---------------------------------
 *   v         agc_vec_t  I    vector at scale 2^scale
 *   i         int        I    component, 0..2
 *   scale     int        I    binary scale N
 *
 * RETURNS: double -- component i in engineering units
 **********************************************************************/
static double vec_comp(agc_vec_t v, int i, int scale)
{
    return (agc_dp_to_double(v.c[i], scale));
}

/**********************************************************************
 * AGC: none (test helper) -- wrap an angle to [-1/2, +1/2] rev
 *
 * FUNCTION NAME: rev_wrap
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description
 *   --------  ------  ---  ---------------------------------
 *   x         double  I    angle, revolutions
 *
 * RETURNS: double -- x modulo one revolution, in [-1/2, +1/2]
 **********************************************************************/
static double rev_wrap(double x)
{
    double r = fmod(x, 1.0); /* revolutions */

    if (r > 0.5)
    {
        r -= 1.0;
    }
    else if (r < -0.5)
    {
        r += 1.0;
    }

    return (r);
}

/**********************************************************************
 * AGC: none (test helper) -- an all-zero pad load
 *
 * FUNCTION NAME: zero_pad
 *
 * RETURNS: agc_navaux_padload_t -- pad load with every field zero
 **********************************************************************/
static agc_navaux_padload_t zero_pad(void)
{
    agc_navaux_padload_t pad = {0}; /* scales in the type */

    pad.tephem = 0;
    pad.lm504  = agc_vec_zero();
    pad.rls    = agc_vec_zero();

    return (pad);
}

/**********************************************************************
 * AGC: none (test of LAT-LONG) -- equatorial point, inertial +Y at 6 h
 *   after the epoch, pad radius ERAD
 *
 * FUNCTION NAME: test_latlong_equator
 *
 * RETURNS: void
 **********************************************************************/
static void test_latlong_equator(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_navaux_padload_t pad  = zero_pad();                           /* zero pad load  */
    agc_latlong_opt_t    opt  = {false, false};                       /* earth, ERAD    */
    const double         erad = 6373338.0;                            /* ERAD, metres   */
    const double         t_cs = 2160000.0;                            /* 6 h            */
    agc_vec_t            r    = vec_m(0.0, erad + 100000.0, 0.0, 29); /* R, 2^29 m */
    agc_latlong_t        ll;                                          /* result         */

    CHECK(agc_latlong_from_r(&ip, &pad, opt, r, (agc_dp_t) t_cs, &ll) == AGC_STATUS_OK);
    CHECK_NEAR(agc_dp_to_double(ll.lat, 0), 0.0, 1e-7);
    CHECK_NEAR(agc_dp_to_double(ll.lon, 0), rev_wrap(0.25 - (WEARTH_REV_CS * t_cs)), 1e-6);
    CHECK_NEAR(agc_dp_to_double(ll.alt, 29), 100000.0, 20.0);
    CHECK_NEAR(agc_dp_to_double(ll.eradm, 29), erad, 4.0);
}

/**********************************************************************
 * AGC: none (test of LALOTORV, LAT-LONG) -- round trip on the Fischer
 *   ellipsoid at the Cape Kennedy pad
 *
 * FUNCTION NAME: test_latlong_round_trip
 *
 * RETURNS: void
 **********************************************************************/
static void test_latlong_round_trip(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_navaux_padload_t pad = zero_pad(); /* AZO set below */

    pad.azo                 = agc_dp_from_double(0.1, 0);
    agc_latlong_opt_t opt   = {false, true};                                                                                                     /* earth, Fischer ellipsoid */
    agc_latlong_t     in    = {agc_dp_from_double(28.608 / 360.0, 0), agc_dp_from_double(-80.604 / 360.0, 0), agc_dp_from_double(150.0, 29), 0}; /* pad */
    agc_vec_t         r;                                                                                                                         /* R, scale 2^29 m     */
    agc_dp_t          eradm = 0;                                                                                                                 /* ERADM, scale 2^29 m */

    CHECK(agc_latlong_to_r(&ip, &pad, opt, &in, 360000, &r, &eradm) == AGC_STATUS_OK);
    agc_latlong_t out; /* LAT-LONG of R */

    CHECK(agc_latlong_from_r(&ip, &pad, opt, r, 360000, &out) == AGC_STATUS_OK);
    CHECK_NEAR(agc_dp_to_double(out.lat, 0), agc_dp_to_double(in.lat, 0), 2e-6);
    CHECK_NEAR(agc_dp_to_double(out.lon, 0), agc_dp_to_double(in.lon, 0), 2e-6);
    CHECK_NEAR(agc_dp_to_double(out.alt, 29), 150.0, 60.0);

    /* Fischer radius at 28.6 deg lies between B and A.                   */
    CHECK(agc_dp_to_double(eradm, 29) > 6356784.0);
    CHECK(agc_dp_to_double(eradm, 29) < 6378166.0);
}

/**********************************************************************
 * AGC: none (test of R-TO-RP, RP-TO-R) -- the round trip returns the
 *   input, earth (AXO, AYO) and moon (504LM)
 *
 * FUNCTION NAME: test_rp_round_trip
 *
 * RETURNS: void
 **********************************************************************/
static void test_rp_round_trip(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_navaux_padload_t pad = zero_pad(); /* filled below */

    pad.tephem  = (agc_tp_t) 134474400;            /* ~15.56 days, cs      */
    pad.azo     = agc_dp_from_double(0.3, 0);
    pad.axo     = agc_dp_from_double(2.0e-5, 0);
    pad.neg_ayo = agc_dp_from_double(-1.5e-5, 0);
    pad.lm504   = vec_m(1.0e-4, -2.0e-4, 0.5e-4, 0);
    agc_vec_t r = vec_m(-3.1e7, 2.2e7, 1.3e7, 29); /* R, scale 2^29 m */

    for (int moon = 0; moon < 2; moon++)
    {
        agc_vec_t rp; /* RP, scale 2^29 m      */
        agc_vec_t r2; /* R again, scale 2^29 m */

        CHECK(agc_planet_r_to_rp(&ip, &pad, moon == 1, r, 1234567, &rp) == AGC_STATUS_OK);
        CHECK(agc_planet_rp_to_r(&ip, &pad, moon == 1, rp, 1234567, &r2) == AGC_STATUS_OK);

        for (int i = 0; i < AGC_VEC_N; i++)
        {
            CHECK_NEAR(vec_comp(r2, i, 29), vec_comp(r, i, 29), 30.0);
        }

        double n1 = hypot(hypot(vec_comp(r, 0, 29), vec_comp(r, 1, 29)), vec_comp(r, 2, 29));    /* |R|, m  */
        double n2 = hypot(hypot(vec_comp(rp, 0, 29), vec_comp(rp, 1, 29)), vec_comp(rp, 2, 29)); /* |RP|, m */

        CHECK_NEAR(n2, n1, 30.0); /* rotation keeps |R|  */
    }
}

/**********************************************************************
 * AGC: none (test helper) -- TFF input at earth scale
 *
 * FUNCTION NAME: tff_state
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description
 *   --------  ------  ---  ---------------------------------
 *   r         double  I    radius along +X, m
 *   v         double  I    speed along +Y, m/cs
 *
 * RETURNS: agc_tff_t -- rone (2^29 m), vone (2^7 m/cs), rtmu (2^-17)
 **********************************************************************/
static agc_tff_t tff_state(double r, double v)
{
    agc_tff_t tff = {0}; /* scales in the type */

    tff.rone = vec_m(r, 0.0, 0.0, 29);
    tff.vone = vec_m(0.0, v, 0.0, 7);
    tff.rtmu = agc_dp_from_double(TFF_RTMU, -17);

    return (tff);
}

/**********************************************************************
 * AGC: none (test of TFFCONIC, TFFRP/RA, CALCTFF) -- circular orbit:
 *   RP = RA = r; no conic reaches a lower RTERM (MAXTFF1)
 *
 * FUNCTION NAME: test_tff_circular
 *
 * RETURNS: void
 **********************************************************************/
static void test_tff_circular(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    const double mu  = 1.0 / (TFF_RTMU * TFF_RTMU); /* m^3/cs^2            */
    const double r   = 6678000.0;                   /* radius, m           */
    agc_tff_t    tff = tff_state(r, sqrt(mu / r));  /* circular speed     */

    agc_tff_conic(&ip, &tff);
    CHECK(agc_tff_rp_ra(&ip, &tff) == AGC_STATUS_OK);
    CHECK_NEAR(agc_dp_to_double(tff.rper, 29), r, 2000.0);
    CHECK_NEAR(agc_dp_to_double(tff.rapo, 29), r, 2000.0);
    agc_dp_t t     = 0;                                            /* TFF, scale 2^28 cs */
    agc_dp_t rterm = agc_dp_from_double(6373338.0 + 121920.0, 29); /* 400k ft, 2^29 m */

    CHECK(agc_tff_calctff(&ip, &tff, rterm, &t) == AGC_STATUS_OK);
    CHECK(t == AGC_TFF_NEARONE);
}

/**********************************************************************
 * AGC: none (test of CALCTFF, CALCTPER) -- from apogee of an ellipse
 *   with perigee below RTERM, against Kepler's equation
 *
 * FUNCTION NAME: test_tff_ellipse
 *
 * RETURNS: void
 **********************************************************************/
static void test_tff_ellipse(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    const double mu  = 1.0 / (TFF_RTMU * TFF_RTMU);                             /* m^3/cs^2        */
    const double ra  = 6678000.0;                                               /* apogee, m       */
    const double rpg = 6000000.0;                                               /* perigee, m      */
    const double a   = 0.5 * (ra + rpg);                                        /* semi-major, m   */
    const double e   = (ra - rpg) / (ra + rpg);                                 /* eccentricity    */
    agc_tff_t    tff = tff_state(ra, sqrt(mu * 2.0 * rpg / (ra * (ra + rpg)))); /* at apogee */

    agc_tff_conic(&ip, &tff);
    CHECK(agc_tff_rp_ra(&ip, &tff) == AGC_STATUS_OK);
    CHECK_NEAR(agc_dp_to_double(tff.rper, 29), rpg, 2000.0);
    CHECK_NEAR(agc_dp_to_double(tff.rapo, 29), ra, 2000.0);
    const double rterm = 6373338.0 + 121920.0;                                         /* m */
    double       e0    = acos((1.0 - (rterm / a)) / e);                                /* E at RTERM, out */
    double       want  = sqrt(a * a * a / mu) * ((TWO_PI / 2.0) - e0 + (e * sin(e0))); /* cs */
    agc_dp_t     t     = 0;                                                            /* TFF, scale 2^28 cs */

    CHECK(agc_tff_calctff(&ip, &tff, agc_dp_from_double(rterm, 29), &t) == AGC_STATUS_OK);
    CHECK_NEAR(agc_dp_to_double(t, 28), want, want * 2e-3);

    /* Time to perigee from apogee: half the period.                      */
    CHECK(agc_tff_calctper(&ip, &tff, tff.rper, &t) == AGC_STATUS_OK);
    want = sqrt(a * a * a / mu) * (TWO_PI / 2.0);
    CHECK_NEAR(agc_dp_to_double(t, 28), want, want * 2e-3);
}

/**********************************************************************
 * AGC: none (test of T(X)) -- T(0) = 1/3 and the series at X = 1/2
 *
 * FUNCTION NAME: test_tff_poly
 *
 * RETURNS: void
 **********************************************************************/
static void test_tff_poly(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    CHECK_NEAR(agc_dp_to_double(agc_tff_poly(0), 0), 1.0 / 3.0, 1e-8);
    double x      = 0.5;                                                                                                               /* X */
    double series = (1.0 / 3.0) - (x / 5.0) + (x * x / 7.0) - (x * x * x / 9.0) + (x * x * x * x / 11.0) - (x * x * x * x * x / 13.0); /* truncated series */

    CHECK_NEAR(agc_dp_to_double(agc_tff_poly(agc_dp_from_double(x, 0)), 0), series, 5e-3);
}

/**********************************************************************
 * AGC: none (test of the STAR TABLES) -- every vector is a half-unit
 *   vector; Polaris is near +Z; bad numbers are refused
 *
 * FUNCTION NAME: test_star_table
 *
 * RETURNS: void
 **********************************************************************/
static void test_star_table(void)
{
    for (int s = 1; s <= AGC_STAR_COUNT; s++)
    {
        agc_vec_t v; /* star vector, scale 2^0 read */

        CHECK(agc_star_vector(s, &v) == AGC_STATUS_OK);
        double x = vec_comp(v, 0, 0); /* half-unit x */
        double y = vec_comp(v, 1, 0); /* half-unit y */
        double z = vec_comp(v, 2, 0); /* half-unit z */

        CHECK_NEAR(sqrt((x * x) + (y * y) + (z * z)), 0.5, 1e-8);
    }

    agc_vec_t polaris; /* star 5, scale 2^1 */

    CHECK(agc_star_vector(5, &polaris) == AGC_STATUS_OK);
    CHECK(vec_comp(polaris, 2, 0) > 0.4999);
    agc_vec_t bad; /* not written, 2^1    */

    CHECK(agc_star_vector(0, &bad) == AGC_STATUS_BAD_ARG);
    CHECK(agc_star_vector(38, &bad) == AGC_STATUS_BAD_ARG);
    CHECK(AGC_STAR_CATLOG == 6970);
}

/**********************************************************************
 * AGC: none (test of INCORP1, INCORP2) -- W = s I (9x9), BVECTOR = h e1:
 *   scalar Kalman update on state 1
 *
 * FUNCTION NAME: test_incorp
 *
 * RETURNS: void
 **********************************************************************/
static void test_incorp(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    const double   s   = 0.25;  /* W diagonal, scale 2^0       */
    const double   h   = 0.5;   /* BVECTOR[0].x, scale 2^0     */
    const double   var = 0.01;  /* VARIANCE                    */
    const double   dq  = 0.001; /* DELTAQ                      */
    agc_incorp_w_t w;           /* W, scale 2^0                */

    for (int b = 0; b < AGC_INCORP_BLKS; b++)
    {
        for (int m = 0; m < AGC_INCORP_ROWS; m++)
        {
            w.blk[b][m] = agc_vec_zero();

            if ((m / 3) == b)
            {
                w.blk[b][m].c[m % 3] = agc_dp_from_double(s, 0);
            }
        }
    }

    agc_incorp_t inc = {0}; /* measurement, scale 2^0 */

    inc.dim9       = true;
    inc.bvector[0] = agc_vec_make(agc_dp_from_double(h, 0), 0, 0);
    inc.bvector[1] = agc_vec_zero();
    inc.bvector[2] = agc_vec_zero();
    inc.deltaq     = agc_dp_from_double(dq, 0);
    inc.variance   = (agc_tp_t) llround(var * 4398046511104.0); /* TP 2^42 */
    CHECK(agc_incorp_state_deviation(&ip, &w, &inc) == AGC_STATUS_OK);
    double a       = (s * s * h * h) + var;                     /* A = Z.Z + VARIANCE */

    /* DELTAX = OMEGA DELTAQ / A, delivered at half scale (see note).      */
    CHECK_NEAR(vec_comp(inc.deltax[0], 0, 0), 0.5 * s * s * h * dq / a, 2e-7);
    CHECK_NEAR(vec_comp(inc.deltax[0], 1, 0), 0.0, 1e-8);
    CHECK_NEAR(vec_comp(inc.deltax[1], 0, 0), 0.0, 1e-8);
    CHECK(agc_incorp_update_w(&ip, &inc, &w) == AGC_STATUS_OK);
    double gamma = 1.0 / (a + sqrt(a * var));           /* GAMMA      */
    double w00   = s - ((s * h) * (s * s * h) * gamma); /* W'(0,0)    */

    CHECK_NEAR(vec_comp(w.blk[0][0], 0, 0), w00, 1e-6);
    CHECK_NEAR(vec_comp(w.blk[0][1], 1, 0), s, 1e-8); /* untouched  */
    CHECK_NEAR(vec_comp(w.blk[2][8], 2, 0), s, 1e-8);

    agc_incorp_sv_t sv = {0}; /* integration temporaries, earth scale */

    sv.moon = false;
    agc_incorp_update_state(&ip, &inc, &sv);
    CHECK(!sv.rectify);
    CHECK(sv.tdeltav.c[0] == agc_dp_shl(&ip, inc.deltax[0].c[0], 7));
} /* test_incorp */

/**********************************************************************
 * AGC: none (test of LSPOS, LUNVEL) -- CSM ephemeris with a constant
 *   moon and a fixed sun; LM sun direction on 16 July 1969 (Apollo 11
 *   launch, TEPHEM ~ 15.564 days)
 *
 * FUNCTION NAME: test_ephemerides
 *
 * RETURNS: void
 **********************************************************************/
static void test_ephemerides(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_ephem_csm_t eph = {0}; /* CM uplink, scales in the type */

    eph.timemo = 0;

    for (int i = 0; i < AGC_EPHEM_NCOEF; i++)
    {
        eph.vecoem[i] = agc_vec_zero();
    }

    eph.vecoem[AGC_EPHEM_NCOEF - 1] = vec_m(3.8e8, 1.0e7, -2.0e7, 31); /* a0 */

    /* a1 = 10 m/cs (the moon moves ~1 km/s): a DP at 2^5 holds < 32.  */
    eph.vecoem[AGC_EPHEM_NCOEF - 2] = vec_m(10.0, 0.0, 0.0, 5); /* a1 m/cs */
    eph.reso                        = vec_m(1.5e11, 0.0, 0.0, 38);
    eph.veso                        = vec_m(0.0, 300.0, 0.0, 9);
    eph.omegaes                     = 0;
    agc_vec_t moon;                                             /* moon, scale 2^29 m */
    agc_vec_t sun;                                              /* sun, scale 2^38 m  */

    CHECK(agc_ephem_csm_lspos(&ip, &eph, 0, 0, &moon, &sun) == AGC_STATUS_OK);
    CHECK_NEAR(vec_comp(moon, 0, 29), 3.8e8, 8.0);
    CHECK_NEAR(vec_comp(moon, 2, 29), -2.0e7, 8.0);
    CHECK_NEAR(vec_comp(sun, 0, 38), 1.5e11, 4096.0);
    agc_vec_t vm; /* moon velocity, scale 2^7 m/cs */

    CHECK(agc_ephem_csm_lunvel(&ip, &eph, 0, 0, &vm) == AGC_STATUS_OK);
    CHECK_NEAR(vec_comp(vm, 0, 7), 10.0, 1e-3);

    agc_vec_t vmoon;                         /* VMOON, scale 2^1     */
    agc_vec_t vsun;                          /* VSUN, scale 2^1      */
    agc_tp_t  tephem = (agc_tp_t) 134472000; /* 15.5639 days, 2^42 cs */

    agc_ephem_lm_lspos(&ip, tephem, 0, &vmoon, &vsun);
    double sx = vec_comp(vsun, 0, 1); /* unit x */
    double sy = vec_comp(vsun, 1, 1); /* unit y */
    double sz = vec_comp(vsun, 2, 1); /* unit z */

    CHECK_NEAR(sqrt((sx * sx) + (sy * sy) + (sz * sz)), 1.0, 1e-6);
    double lam = 113.6 * TWO_PI / 360.0; /* sun longitude        */

    CHECK_NEAR(sx, cos(lam), 0.02);
    CHECK_NEAR(sy, 0.91745 * sin(lam), 0.02);
    CHECK_NEAR(sz, 0.39784 * sin(lam), 0.02);
    double mx = vec_comp(vmoon, 0, 1); /* unit x */
    double my = vec_comp(vmoon, 1, 1); /* unit y */
    double mz = vec_comp(vmoon, 2, 1); /* unit z */

    CHECK_NEAR(sqrt((mx * mx) + (my * my) + (mz * mz)), 1.0, 1e-6);
} /* test_ephemerides */
