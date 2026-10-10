/*
 * test_rendezvous.c -- prog-rendz unit tests (rendezvous programs).
 *
 *   1. R30 SR30.1 (agc_orbit_params_compute): a circular 300 km earth
 *      orbit has equal apogee and perigee radii and heights, within a few
 *      hundred LSBs; MAXCHK limits a large height to 9999.9 NM.
 *   2. CSI targeting (agc_rendz_csi_a) on a coplanar circular case: the
 *      active vehicle 15 NM below the passive one must speed up, so the
 *      CSI horizontal delta V (DELVCSI) is positive.
 *   3. SELECTMU and PERIAPO1 on the circular orbit: equal apsides.
 *
 * CSM build; inputs at the AGC scales (r m 2^29, v m/cs 2^7).
 */
#include <math.h>
#include <stdio.h>
#include "agc/core/assert.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/flags_common.h"
#include "agc/programs/rendezvous.h"
#include "../src/programs/rendezvous/ext.h"   /* provisional job ids */

static int g_checks;
static int g_failures;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void)printf("assert %s:%d %s\n", file, line, expr);
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

static const double MU = 3.986032e10;     /* MUE, m^3/cs^2 (MUTABLE)    */
static const double RPAD = 6373338.0;     /* RPAD, m                    */
static const double R0 = 6678137.0;       /* 300 km circular orbit, m   */

static agc_vec_t pos(double x, double y, double z)
{
    return agc_vec_make(agc_dp_from_double(x, 29), agc_dp_from_double(y, 29),
                        agc_dp_from_double(z, 29));
}

static agc_vec_t vel(double x, double y, double z)
{
    return agc_vec_make(agc_dp_from_double(x, 7), agc_dp_from_double(y, 7),
                        agc_dp_from_double(z, 7));
}

static agc_dp_t dp_absdiff(agc_dp_t a, agc_dp_t b)
{
    return (a > b) ? a - b : b - a;
}

/* 1. SR30.1 on a circular orbit. */
static void test_orbit_parameters_circular(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_orbit_params_in_t in;
    in.rone = pos(R0, 0.0, 0.0);
    CHECK(agc_orbit_params_select_body(false, agc_dp_from_double(RPAD, 29),
                                       &in) == AGC_STATUS_OK);
    in.vone = vel(0.0, sqrt(MU / R0), 0.0);
    agc_orbit_params_out_t out;
    agc_status_t st = agc_orbit_params_compute(&ip, &in, &out);
    CHECK(st == AGC_STATUS_OK);
    CHECK(dp_absdiff(out.rapo, out.rper) < 400);           /* < 800 m    */
    CHECK(dp_absdiff(out.hapox, out.hperx) < 400);
    agc_dp_t h = agc_dp_from_double(R0 - RPAD, 29);
    CHECK(dp_absdiff(out.hperx, h) < 400);
    CHECK(out.neg_tper <= 0);
    /* MAXCHK: 20000 NM is shown as 9999.9 NM. */
    agc_dp_t big = agc_dp_from_double(20000.0 * 1852.0, 29);
    agc_dp_t lim = agc_orbit_params_maxchk(&ip, big);
    CHECK(lim == agc_dp_from_double(9999.9 * 1852.0, 29)
          || dp_absdiff(lim, agc_dp_from_double(9999.9 * 1852.0, 29)) < 2);
    CHECK(agc_orbit_params_maxchk(&ip, h) == h);
}

/* 2. CSI on coplanar circular orbits: DELVCSI > 0. */
static void test_csi_coplanar_circular(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_rendz_reset(rz);
    CHECK(agc_rendz_selectmu(rz, false) == AGC_STATUS_OK);
    double ra = R0;
    double rp = R0 + 15.0 * 1852.0;
    double th = 0.15;                     /* passive leads by 0.15 rad   */
    rz->ract1 = pos(ra, 0.0, 0.0);
    rz->vact1 = vel(0.0, sqrt(MU / ra), 0.0);
    rz->rpass3 = pos(rp * cos(th), rp * sin(th), 0.0);
    rz->vpass3 = vel(-sqrt(MU / rp) * sin(th), sqrt(MU / rp) * cos(th), 0.0);
    rz->rpass1 = rz->rpass3;
    rz->vpass1 = rz->vpass3;
    rz->rpass2 = rz->rpass3;
    rz->vpass2 = rz->vpass3;
    rz->ract2 = rz->ract1;
    rz->vact2 = rz->vact1;
    rz->up1 = agc_vec_make(0, 0, AGC_DP_HALF);             /* +z, 2^1     */
    rz->tcsi = 0;
    rz->nn = 1;
    rz->elev = agc_dp_from_double(26.6 / 360.0, 0);
    agc_erase_set_tig(0);
    agc_erase_set_ttpi(agc_dp_from_double(2.0 * 3600.0 * 100.0, 28));
    agc_sp_t alarm = 0;
    agc_status_t st = agc_rendz_csi_a(&ip, rz, &alarm, AGC_JOB_CSI_COMPUTE);
    (void)printf("CSI status %d alarm %o DELVCSI %.3f fps\n", (int)st,
                 (unsigned)alarm,
                 (double)rz->delvcsi / 2147483648.0 * 128.0 * 100.0 / 0.3048);
    CHECK(st == AGC_STATUS_OK || st == AGC_STATUS_ALARM);
    CHECK(rz->delvcsi > 0);
}

/* 3. PERIAPO1 of the circular orbit. */
static void test_periapo_circular(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable();
    CHECK(agc_rendz_selectmu(rz, false) == AGC_STATUS_OK);
    agc_rendz_periapo_t pa;
    agc_status_t st = agc_rendz_periapo1(&ip, rz, pos(R0, 0.0, 0.0),
                                         vel(0.0, sqrt(MU / R0), 0.0), &pa);
    CHECK(st == AGC_STATUS_OK);
    CHECK(dp_absdiff(pa.rapo, pa.rperi) < 400);
}

int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    test_orbit_parameters_circular();
    test_periapo_circular();
    test_csi_coplanar_circular();
    (void)printf("test_rendezvous: %d checks, %d failures\n", g_checks,
                 g_failures);
    return (g_failures == 0) ? 0 : 1;
}
