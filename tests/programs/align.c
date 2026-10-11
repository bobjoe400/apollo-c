/*
 * align.c -- prog-align unit tests (alignment math, optics and radar
 * geometry, S-band antenna angles).
 *
 * Required by the assignment:
 *   * P52 option 3 with an identity REFSMMAT and two orthogonal star
 *     sightings (observed exactly where the catalogue puts them) gives an
 *     identity desired-SM matrix XDC, a zero CHKSDATA angle difference and
 *     zero gyro torquing angles.
 *   * S-band antenna angles for the earth directly along the +X body axis:
 *     CM pitch 0, yaw 0; LM pitch 90 deg, yaw -45 deg (antenna base is the
 *     NB rotated 45 deg about Z).
 * Also: P52 fine align with a realistic sub-degree drift and a
 * non-identity REFSMMAT, SXTNB / SXTANG round trip, RRNB / RRANGLES round trip, LOG,
 * SVDWN shifts, S52.3 axes, CHKSCODE, 1STO2S, R47 SCALEVEC rounding and
 * AGS time, R63 line of sight.
 */
#include <stdio.h>

#include <math.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/word.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/nouns.h"
#include "agc/programs/alignment.h"

#define PI_D 3.14159265358979323846

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

static agc_vec_t half_unit(double x, double y, double z)
{
    const double n = sqrt(x * x + y * y + z * z);

    return (agc_vec_make(agc_dp_from_double(x / n, 1),
                         agc_dp_from_double(y / n, 1),
                         agc_dp_from_double(z / n, 1)));
}

/* 2's complement CDU word for an angle in degrees. */
static uint16_t cdu_deg(double deg)
{
    long c = lround(deg / 360.0 * 32768.0);

    while (c < 0)
    {
        c += 32768;
    }

    return ((uint16_t) (c % 32768));
}

static double cdu_to_deg(uint16_t w)
{
    long c = (long) (w & 077777u);

    if (c >= 16384)
    {
        c -= 32768;
    }

    return ((double) c * 360.0 / 32768.0);
}

static bool mat_is_identity_half(const agc_mat_t *m, double tol)
{
    bool ok = true;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            const double want = (i == j) ? 1.0 : 0.0;

            ok = ok && near(agc_dp_to_double(m->row[i].c[j], 1), want, tol);
        }
    }

    return (ok);
}

/* P52 option 3 (REFSMMAT), R51 / R54 / AXISGEN / R55 arithmetic. */
static void test_realign_refsmmat_option_identity(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    const agc_mat_t refsmmat = agc_mat_identity_half();  /* B1 scale */
    const agc_vec_t cat1     = half_unit(1.0, 0.0, 0.0); /* star 1  */
    const agc_vec_t cat2     = half_unit(0.0, 1.0, 0.0); /* star 2  */
    agc_vec_t       a[2];                                /* STARAD, STARAD+6: catalogue in desired SM */

    a[0] = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, cat2), NULL, NULL);
    a[1] = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, cat1), NULL, NULL);
    agc_vec_t b[2]; /* 6D, 12D: STARSAV2, STARSAV1 observed (SM) */

    b[0]          = cat2;
    b[1]          = cat1;
    agc_dp_t diff = -1;

    CHECK(agc_align_chksdata_angle(&ip, a, b, &diff) == AGC_STATUS_OK);
    CHECK(near(rev(diff), 0.0, 1e-6));
    CHECK(near(rev(agc_align_ram()->theta), 0.25, 1e-5)); /* 90 deg  */
    agc_mat_t xdc;

    CHECK(agc_align_fine_solve(&ip, a, b, &xdc) == AGC_STATUS_OK);
    CHECK(mat_is_identity_half(&xdc, 1e-6));
    agc_dp_t gyro[3] = {1, 1, 1};

    agc_imu_calcgta(&ip, &xdc, gyro);

    for (int k = 0; k < 3; k++)
    {
        CHECK(near(rev(gyro[k]), 0.0, 1e-6));
    }

    /* GYCRS / GYCOARS with XSMD = REFSMMAT rows: XDC identity again. */
    agc_mat_t smd = agc_mat_identity_half();

    CHECK(agc_align_gyro_coarse_dc(&ip, &smd, &refsmmat, &xdc) == AGC_STATUS_OK);
    CHECK(mat_is_identity_half(&xdc, 1e-6));
}

/* Rotation matrices about X, Y, Z (row i = image of axis i). */
static void rot_axis(int axis, double ang, double m[3][3])
{
    const double c = cos(ang);
    const double s = sin(ang);
    const int    j = (axis + 1) % 3;
    const int    k = (axis + 2) % 3;

    for (int i = 0; i < 3; i++)
    {
        for (int n = 0; n < 3; n++)
        {
            m[i][n] = (i == n) ? 1.0 : 0.0;
        }
    }

    m[j][j] = c;
    m[j][k] = s;
    m[k][j] = -s;
    m[k][k] = c;
}

static void mat_mul_d(double a[3][3], double b[3][3], double out[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
}

/*
 * P52 fine align at realistic magnitudes: REFSMMAT a 30 deg rotation about
 * Z, the platform drifted by (0.5, -0.3, 0.2) deg about X, Y, Z, two stars
 * 52 deg apart.  XDC must reproduce the drift matrix to 1e-7 (0.02 arcsec)
 * and CALCGTA's three torquing angles must equal the same formulae in double
 * to within one fine-align gyro pulse (2^-21 rev, 0.62 arcsec).
 */
static void test_realign_realistic_drift(void)
{
    agc_interp_t ip;
    double       rz[3][3];
    double       rx[3][3];
    double       ry[3][3];
    double       t[3][3];
    double       d[3][3]; /* desired SM axes referred to the present SM */
    double       r30[3][3];
    agc_mat_t    refsmmat;

    agc_interp_init(&ip);
    rot_axis(0, 0.5 * PI_D / 180.0, rx);
    rot_axis(1, -0.3 * PI_D / 180.0, ry);
    rot_axis(2, 0.2 * PI_D / 180.0, rz);
    mat_mul_d(rx, ry, t);
    mat_mul_d(t, rz, d);
    rot_axis(2, PI_D / 6.0, r30);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            refsmmat.row[i].c[j] = agc_dp_from_double(r30[i][j], 1);
        }
    }

    const double star[2][3] = {{0.6, 0.48, 0.64}, {-0.2, 0.9, 0.3873}};
    agc_vec_t    a[2]; /* STARAD, STARAD +6: catalogue in desired SM     */
    agc_vec_t    b[2]; /* 6D, 12D: observed in the present SM            */

    for (int s = 0; s < 2; s++)
    {
        const agc_vec_t ref = half_unit(star[s][0], star[s][1], star[s][2]);
        double          ad[3];
        double          bd[3];

        a[s] = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, ref), NULL, NULL);

        for (int i = 0; i < 3; i++)
        {
            ad[i] = agc_dp_to_double(a[s].c[i], 1);
        }

        for (int j = 0; j < 3; j++)
        {
            bd[j] = d[0][j] * ad[0] + d[1][j] * ad[1] + d[2][j] * ad[2];
        }

        b[s] = half_unit(bd[0], bd[1], bd[2]);
    }

    agc_dp_t diff = -1;

    CHECK(agc_align_chksdata_angle(&ip, a, b, &diff) == AGC_STATUS_OK);
    CHECK(near(rev(diff), 0.0, 1e-6));
    agc_mat_t xdc;

    CHECK(agc_align_fine_solve(&ip, a, b, &xdc) == AGC_STATUS_OK);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            CHECK(near(agc_dp_to_double(xdc.row[i].c[j], 1), d[i][j], 1e-7));
        }
    }

    /* CALCGTA in double (INFLIGHT_ALIGNMENT_ROUTINES p.1355). */
    const double n       = hypot(d[0][2], d[0][0]);
    const double zp[3]   = {-d[0][2] / n, 0.0, d[0][0] / n};
    const double want[3] = {
        atan2(zp[0] * d[1][0] + zp[2] * d[1][2],
              zp[0] * d[2][0] + zp[2] * d[2][2]),
        atan2(zp[0], zp[2]),
        atan2(d[0][1], zp[2] * d[0][0] - zp[0] * d[0][2])
    };
    agc_dp_t     gyro[3] = {0, 0, 0};

    agc_imu_calcgta(&ip, &xdc, gyro);

    for (int k = 0; k < 3; k++)
    {
        CHECK(near(rev(gyro[k]), want[k] / (2.0 * PI_D), 0x1p-21));
        CHECK(fabs(want[k]) < 0.6 * PI_D / 180.0); /* sub-degree drift */
    }
}

/* R05: the earth along +X body. */
static void test_sband_plus_x(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_dp_t  pitch = 1;
    agc_dp_t  yaw   = 1;
    agc_vec_t r     = agc_vec_make(agc_dp_from_double(3.0e8, 29), 0, 0);

    CHECK(agc_sband_cm_angles(&ip, r, &pitch, &yaw) == AGC_STATUS_OK);
    CHECK(near(rev(pitch) * 360.0, 0.0, 0.01));
    CHECK(near(rev(yaw) * 360.0, 0.0, 0.01));

    /* CM: earth along +Y body gives yaw 90, along -Y yaw 270. */
    r = agc_vec_make(0, agc_dp_from_double(3.0e8, 29), 0);
    CHECK(agc_sband_cm_angles(&ip, r, &pitch, &yaw) == AGC_STATUS_OK);
    CHECK(near(rev(yaw) * 360.0, 90.0, 0.01));
    r = agc_vec_make(0, agc_dp_from_double(-3.0e8, 29), 0);
    CHECK(agc_sband_cm_angles(&ip, r, &pitch, &yaw) == AGC_STATUS_OK);
    CHECK(near(rev(yaw) * 360.0, 270.0, 0.01));

    /* LM: unit vector along +X body. */
    agc_vec_t ue = half_unit(1.0, 0.0, 0.0);

    CHECK(agc_sband_lm_angles(&ip, ue, &pitch, &yaw) == AGC_STATUS_OK);
    CHECK(near(rev(pitch) * 360.0, 90.0, 0.01));
    CHECK(near(rev(yaw) * 360.0, -45.0, 0.01));
}

/* SXTNB then SXTANG (NB = reference): the optics angles come back. */
static void test_sextant_round_trip(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    const double   sa_deg = 40.0;
    const double   ta_deg = 20.0; /* true trunnion angle     */
    const uint16_t shaft  = cdu_deg(sa_deg);
    const uint16_t trun   = cdu_deg(4.0 * ta_deg - 79.1015625);
    agc_vec_t      los;

    CHECK(agc_csm_sxtnb(&ip, trun, shaft, &los) == AGC_STATUS_OK);
    CHECK(near(rev(agc_csm_sxtlogic(trun)) * 360.0, ta_deg, 0.02));
    CHECK(near(agc_dp_to_double(agc_vec_abval(&ip, los), 1), 1.0, 1e-5));
    const agc_mat_t nb   = agc_mat_identity_half();
    uint16_t        sac  = 0;
    uint16_t        pac  = 0;
    bool            cult = true;

    CHECK(agc_csm_sxtang(&ip, los, &nb, &sac, &pac, &cult) == AGC_STATUS_OK);
    CHECK(!cult);
    CHECK(near(cdu_to_deg(sac), sa_deg, 0.03));
    CHECK(near(cdu_to_deg(pac), cdu_to_deg(trun), 0.1));

    /* A star along the shaft axis: SAC 270 deg, PAC = 20DEGS-. */
    agc_vec_t z1;

    CHECK(agc_csm_sxtnb(&ip, cdu_deg(-79.1015625), 0, &z1) == AGC_STATUS_OK);
    CHECK(agc_csm_sxtang(&ip, z1, &nb, &sac, &pac, &cult) == AGC_STATUS_OK);
    CHECK(sac == 060000u);
}

/* RRNB then RRANGLES: mode 1 angles come back; mode 2 = (180-T, S+180). */
static void test_rr_round_trip(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    const uint16_t t = cdu_deg(-25.0);
    const uint16_t s = cdu_deg(30.0);
    agc_vec_t      los;

    CHECK(agc_lm_rrnb(&ip, t, s, &los) == AGC_STATUS_OK);
    uint16_t ma[2]   = {0, 0};
    uint16_t mb[2]   = {0, 0};
    bool     along_y = true;

    CHECK(agc_lm_rrangles(&ip, los, false, ma, mb, &along_y) == AGC_STATUS_OK);
    CHECK(!along_y);
    CHECK(near(cdu_to_deg(ma[0]), -25.0, 0.03));
    CHECK(near(cdu_to_deg(ma[1]), 30.0, 0.03));
    CHECK(near(cdu_to_deg(mb[0]), -155.0, 0.03)); /* 205 deg        */
    CHECK(near(cdu_to_deg(mb[1]), -150.0, 0.03)); /* 210 deg        */
    CHECK(agc_lm_rrangles(&ip, half_unit(0.0, 1.0, 0.0), false, ma, mb, &along_y) == AGC_STATUS_OK);
    CHECK(along_y);
}

static void test_misc(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_dp_t l = 0;

    CHECK(agc_csm_log(&ip, agc_dp_from_double(0.5, 0), &l) == AGC_STATUS_OK);

    /* LOG is a 3-term fit, good to ~1.5e-3 in ln x; the AGC model values
     * (same polynomial in double) are .69178 for x = .5 and 1.20506 for .3. */
    CHECK(near(agc_dp_to_double(l, 5), 0.6917797, 2e-6));
    CHECK(near(agc_dp_to_double(l, 5), log(2.0), 2e-3));
    CHECK(agc_csm_log(&ip, agc_dp_from_double(0.3, 0), &l) == AGC_STATUS_OK);
    CHECK(near(agc_dp_to_double(l, 5), 1.2050592, 2e-6));
    CHECK(near(agc_dp_to_double(l, 5), -log(0.3), 2e-3));
    CHECK(agc_align_chkscode(0));
    CHECK(agc_align_chkscode(050));
    CHECK(!agc_align_chkscode(051));
    CHECK(!agc_align_chkscode(-1));
    CHECK(agc_align_ones_to_twos(agc_dp_from_double(0.25, 0)) == 020000u);
    CHECK(agc_align_ones_to_twos(agc_dp_from_double(-0.25, 0)) == 060000u);
    CHECK(near(rev(agc_align_cdulogic(060000u)), -0.25, 1e-9));

    /* SVDWN, other vehicle, moon (x2 = 2): R = (dR SR9 + RCV) SR2. */
    const agc_vec_t rcv = agc_vec_make(agc_dp_from_double(2.0e6, 27), 0, 0);
    const agc_vec_t vcv = agc_vec_make(0, agc_dp_from_double(16.0, 5), 0);
    const agc_vec_t dr  = agc_vec_make(agc_dp_from_double(1000.0, 18), 0, 0);
    const agc_vec_t dv  = agc_vec_make(0, agc_dp_from_double(0.01, -1), 0);

    CHECK(agc_integ_svdwn(&ip, false, 2, rcv, vcv, dr, dv, 0) == AGC_STATUS_OK);
    CHECK(near(agc_dp_to_double(agc_integ_r_other().c[0], 29), 2.001e6, 2.0));
    CHECK(near(agc_dp_to_double(agc_integ_v_other().c[1], 7), 16.01, 1e-3));
}

/* S52.3 axes for the CSM: Z down, Y opposite the orbital angular momentum. */
static void test_nominal_orientation_axes(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    const agc_vec_t r = agc_vec_make(agc_dp_from_double(7.0e6, 29), 0, 0);
    const agc_vec_t v = agc_vec_make(0, agc_dp_from_double(75.0, 7), 0);
    agc_mat_t       smd;

    CHECK(agc_align_nominal_orientation_axes(&ip, r, v, &smd) == AGC_STATUS_OK);
    CHECK(near(agc_dp_to_double(smd.row[2].c[0], 1), -1.0, 1e-6)); /* ZSMD */
    CHECK(near(agc_dp_to_double(smd.row[1].c[2], 1), -1.0, 1e-6)); /* YSMD */
    CHECK(near(agc_dp_to_double(smd.row[0].c[1], 1), 1.0, 1e-6));  /* XSMD */
}

/* AGS word as the 2's complement AEA reads it (15 bits). */
static long ags_read(uint16_t w)
{
    long c = (long) (w & 077777u);

    return ((c >= 16384) ? (c - 32768) : c);
}

/*
 * R47 SCALEVEC in the moon sphere: half-unit identity REFSMMAT (MXV keeps
 * the matrix's B1, so the result is half the vector); r = (1e6, 0, -1e6)
 * m (RATT1 B27), v = (1, 0, -1) m/cs (VATT1 B5).  High words: R in 512 ft
 * (6407.9 -> 6408), V in 0.5 ft/s (656.17 -> 656), rounded to nearest as
 * the AEA reads them (2's complement), so -656.17 reads -656.  AGSBUFF
 * +12D: 3600 s at B18.
 */
static void test_ags_scalevec(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    const agc_mat_t ident  = agc_mat_identity_half();
    const agc_vec_t r      = agc_vec_make(agc_dp_from_double(1.0e6, 27), 0, agc_dp_from_double(-1.0e6, 27));
    const agc_vec_t v      = agc_vec_make(agc_dp_from_double(1.0, 5), 0, agc_dp_from_double(-1.0, 5));
    uint16_t        out[6] = {0, 0, 0, 0, 0, 0};

    CHECK(agc_ags_scalevec(&ip, r, v, &ident, out) == AGC_STATUS_OK);
    CHECK(ags_read(out[0]) == 6408);  /* RX            */
    CHECK(ags_read(out[1]) == 656);   /* VX            */
    CHECK((ags_read(out[2]) == 0) && (ags_read(out[3]) == 0));
    CHECK(ags_read(out[4]) == -6408); /* RZ            */
    CHECK(ags_read(out[5]) == -656);  /* VZ            */

    /* A negative sum smaller than one LSB: the two-step AGSRND add leaves
     * -0 (77777), read -1 (-0.6 + 0.5 rounds down); -0.4 + 0.5 reads 0. */
    const double    ft_lsb = 1.0e6 / 6407.9; /* R: metres per 512 ft (x2) */
    const double    v_lsb  = 1.0 / 656.17;   /* V: m/cs per 0.5 ft/s (x2) */
    const agc_vec_t rs     = agc_vec_make(agc_dp_from_double(-0.6 * ft_lsb, 27),
                                          agc_dp_from_double(-0.4 * ft_lsb, 27),
                                          0);
    const agc_vec_t vs     = agc_vec_make(agc_dp_from_double(-0.6 * v_lsb, 5),
                                          agc_dp_from_double(-0.4 * v_lsb, 5),
                                          0);

    CHECK(agc_ags_scalevec(&ip, rs, vs, &ident, out) == AGC_STATUS_OK);
    CHECK((out[0] == 077777u) && (out[1] == 077777u)); /* -0: AEA -1     */
    CHECK((out[2] == 0u) && (out[3] == 0u));
    agc_dp_t t = 0;

    CHECK(agc_ags_time(&ip, agc_dp_from_double(400000.0, 28), agc_dp_from_double(40000.0, 28), &t) == AGC_STATUS_OK);
    CHECK(near(agc_dp_to_double(t, 18), 3600.0, 0.01));
}

/*
 * R63 at the ranges it is flown (a few hundred metres to tens of km): the
 * CSM along (3, 4, 12)/13 from the LM in lunar orbit (B27), REFSMMAT a
 * 30 deg rotation about Z (half unit).  POINTVSM must equal REFSMMAT LOS
 * to within the input quantisation (MXV leaves a 1 m LSB per component):
 * tolerance 1 m / range.  Plain UNIT (no NORMUNIT pre-shift) misses this
 * by 15x at 1 km.
 */
static void test_final_attitude_los(void)
{
    static const double range_m[] = {300.0, 1000.0, 3000.0, 10000.0, 50000.0};
    const double        c30       = cos(PI_D / 6.0);
    const double        s30       = sin(PI_D / 6.0);
    const double        dir[3]    = {3.0 / 13.0, 4.0 / 13.0, 12.0 / 13.0};
    const double        rot[3][3] = {{c30, s30, 0.0}, {-s30, c30, 0.0}, {0.0, 0.0, 1.0}};
    agc_mat_t           refsmmat;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            refsmmat.row[i].c[j] = agc_dp_from_double(rot[i][j], 1);
        }
    }

    for (int k = 0; k < 5; k++)
    {
        agc_interp_t ip;

        agc_interp_init(&ip);
        const double    rl[3] = {1.80e6, 2.0e5, -3.0e5}; /* LM, m  */
        const agc_vec_t rlem  = agc_vec_make(agc_dp_from_double(rl[0], 27),
                                             agc_dp_from_double(rl[1], 27),
                                             agc_dp_from_double(rl[2], 27));
        const agc_vec_t rcsm  = agc_vec_make(agc_dp_from_double(rl[0] + range_m[k] * dir[0], 27), agc_dp_from_double(rl[1] + range_m[k] * dir[1], 27), agc_dp_from_double(rl[2] + range_m[k] * dir[2], 27));
        agc_vec_t       los   = agc_vec_zero();
        const double    tol   = 1.0 / range_m[k];

        CHECK(agc_rendezvous_final_attitude_los(&ip, rcsm, rlem, &refsmmat, &los) == AGC_STATUS_OK);

        for (int i = 0; i < 3; i++)
        {
            const double want = rot[i][0] * dir[0] + rot[i][1] * dir[1] + rot[i][2] * dir[2];

            CHECK(near(agc_dp_to_double(los.c[i], 1), want, tol));
        }
    }
}

/*
 * N70/N71 star code: V21 loads R1 into STARCODE (CSM) / AOTCODE (LM); the
 * LM's R2/R3 (AOTCODE +1/+2) are the two words of SINCDUY, shown as raw
 * ones'-complement octal.
 */
static void test_star_code_nouns(bool lm)
{
    agc_dp_t v = 0;

    CHECK(agc_noun_write(lm ? AGC_DATA_AOTCODE : AGC_DATA_STARCODE, 0225) == AGC_STATUS_OK);
    CHECK(agc_align_ram()->starcode == 0225);
    CHECK(agc_noun_read(lm ? AGC_DATA_AOTCODE : AGC_DATA_STARCODE, &v) == AGC_STATUS_OK);
    CHECK(v == 0225);

    if (lm)
    {
        agc_guid_state()->trig.sincdu[AGC_TRIG_Y] = agc_dp_pack(-3, -5);
        CHECK(agc_noun_read(AGC_DATA_AOTCODE_P1, &v) == AGC_STATUS_OK);
        CHECK(v == 077774); /* -3 in ones' complement */
        CHECK(agc_noun_read(AGC_DATA_AOTCODE_P2, &v) == AGC_STATUS_OK);
        CHECK(v == 077772); /* -5                      */
    }
}

int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    agc_align_ram_reset();
    test_realign_refsmmat_option_identity();
    test_realign_realistic_drift();
    test_sband_plus_x();
    test_sextant_round_trip();
    test_misc();
    test_nominal_orientation_axes();
    test_star_code_nouns(false);
    agc_state_init(AGC_VEHICLE_LM);
    agc_align_ram_reset();
    test_rr_round_trip();
    test_realign_refsmmat_option_identity();
    test_ags_scalevec();
    test_final_attitude_los();
    test_star_code_nouns(true);
    (void) printf("test_align: %d checks, %d failures, %d assertion reports\n", g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1);
}
