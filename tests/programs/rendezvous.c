/**********************************************************************
 * FILE NAME: rendezvous.c
 *
 * PURPOSE:
 *   prog-rendz unit tests (rendezvous programs), CSM build; inputs at
 *   the AGC scales (r m 2^29, v m/cs 2^7):
 *     1. R30 SR30.1 (agc_orbit_params_compute): a 300 km perigee earth
 *        orbit with e = 0.0404 has the analytic apogee and perigee radii
 *        and heights within a few hundred LSBs; MAXCHK limits a large
 *        height to 9999.9 NM.
 *     2. CSI targeting (agc_rendz_csi_a) on coplanar circular orbits,
 *        the active vehicle 15 NM below the passive one: with a small
 *        lead of the passive vehicle CSI must slow the catch-up
 *        (DELVCSI > 0), with a large lead speed it up (DELVCSI < 0).
 *        States are set as P32/P72B leaves them (ADVANCE, passive INTINT
 *        to TTPI).
 *     3. SELECTMU and PERIAPO1 on the circular orbit: equal apsides.
 *     4. P23 and R22 preset PHSPRDT2 (PRIO13, PRIO26): a later group 2
 *        restart of a type C phase with D clear runs at that priority.
 *     5. GET+MGA: Comanche unitizes delta V once, Luminary twice
 *        (VLOAD UNIT / UNIT); each vehicle follows its rope.
 *     6. V82 with average G on (V82GON1) on an escape hyperbola past its
 *        pericenter: CALCTPER gives TPER < 0, SR30.1 stores -TPER > 0
 *        as is and the job goes on (NASSP TEI finding: the C refused a
 *        positive -TPER, ended the V82 job and left the DSKY blank).
 *
 * AGC SOURCE:
 *   Comanche055 R30.agc (SR30.1, MAXCHK), P32-P33_P72-P73.agc (CSI/A),
 *   P34-35_P74-75.agc (SELECTMU, PERIAPO1), P20-P25.agc (P23, R22)
 *
 * EXTERNAL VARIABLES:
 *   TIG, TTPI (agc_erase_*): set for the CSI case
 *
 * EXTERNAL REFERENCES:
 *   Name                           Description
 *   -----------------------------  -------------------------------------
 *   agc_orbit_params_compute       SR30.1
 *   agc_rendz_csi_a                CSI/A
 *   agc_rendz_periapo_body_scale   PERIAPO1
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when a check fails.
 **********************************************************************/
#include <stdio.h>

#include <math.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/rendezvous.h"

#define CHECK(c) check((c), __LINE__, #c) /* count and report a check */

static int g_checks;   /* checks run   */
static int g_failures; /* checks failed */

static const double MU   = 3.986032e10; /* MUE, m^3/cs^2 (MUTABLE)    */
static const double RPAD = 6373338.0;   /* RPAD, m                    */
static const double R0   = 6678137.0;   /* 300 km circular orbit, m   */

/* Prototypes: main() comes first (STYLE 4.6). */
static void         check(bool ok, int line, const char *text);
static agc_vec_t    pos(double x, double y, double z);
static agc_vec_t    vel(double x, double y, double z);
static agc_dp_t     dp_absdiff(agc_dp_t a, agc_dp_t b);
static void         test_orbit_parameters_eccentric(void);
static agc_status_t csi_case(double th, agc_dp_t *delvcsi);
static void         test_csi_coplanar_circular(void);
static void         test_periapo_circular(void);
static bool         group2_restarts_at(agc_job_id_t job, agc_prio_t prio);
static void         test_group2_restart_priority(void);
static agc_dp_t     mga_expected(agc_vec_t dv, agc_vec_t ysm, bool twice);
static void         test_get_mga_per_rope(void);
static void         test_v82_average_g_departing(void);
static void         test_vehicle_dispatch(void);

/**********************************************************************
 * AGC: none (test driver) -- the three tests, CSM build
 *
 * FUNCTION NAME: main
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    test_orbit_parameters_eccentric();
    test_periapo_circular();
    test_csi_coplanar_circular();
    test_group2_restart_priority();
    test_get_mga_per_rope();
    test_v82_average_g_departing();
    test_vehicle_dispatch();
    (void) printf("test_rendezvous: %d checks, %d failures\n", g_checks, g_failures);

    if (g_failures != 0)
    {
        return (1);
    }

    return (0);
}

/**********************************************************************
 * The AGC_ASSERT hook of the test: print and fail the assertion.
 * AGC: none (test support)
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   file      const char *  I    source file of the assertion
 *   line      int           I    its line
 *   expr      const char *  I    the expression text
 *
 * RETURNS: bool -- false (the assertion fails)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);

    return (false);
}

/**********************************************************************
 * Count a check; print the failing ones.
 * AGC: none (test support)
 *
 * FUNCTION NAME: check
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   ok        bool          I    the check passed
 *   line      int           I    its source line
 *   text      const char *  I    the checked expression
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
 * A position vector in meters at the AGC scale.
 * AGC: none (test support)
 *
 * FUNCTION NAME: pos
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description (scale)
 *   --------  ------  ---  ----------------------------------------------
 *   x         double  I    x, m
 *   y         double  I    y, m
 *   z         double  I    z, m
 *
 * RETURNS: agc_vec_t -- the vector, m 2^29
 **********************************************************************/
static agc_vec_t pos(double x, double y, double z)
{
    return (agc_vec_make(agc_dp_from_double(x, 29), agc_dp_from_double(y, 29), agc_dp_from_double(z, 29)));
}

/**********************************************************************
 * A velocity vector in m/cs at the AGC scale.
 * AGC: none (test support)
 *
 * FUNCTION NAME: vel
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description (scale)
 *   --------  ------  ---  ----------------------------------------------
 *   x         double  I    x, m/cs
 *   y         double  I    y, m/cs
 *   z         double  I    z, m/cs
 *
 * RETURNS: agc_vec_t -- the vector, m/cs 2^7
 **********************************************************************/
static agc_vec_t vel(double x, double y, double z)
{
    return (agc_vec_make(agc_dp_from_double(x, 7), agc_dp_from_double(y, 7), agc_dp_from_double(z, 7)));
}

/**********************************************************************
 * |a - b| of two DP words.
 * AGC: none (test support)
 *
 * FUNCTION NAME: dp_absdiff
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   a         agc_dp_t  I    first value
 *   b         agc_dp_t  I    second value, same scale
 *
 * RETURNS: agc_dp_t -- |a - b|, LSB
 **********************************************************************/
static agc_dp_t dp_absdiff(agc_dp_t a, agc_dp_t b)
{
    if (a > b)
    {
        return (a - b);
    }

    return (b - a);
}

/**********************************************************************
 * 1. SR30.1 on an eccentric orbit, state at perigee.
 * Not a circular orbit: there APSIDES takes SQRT(1/64 - P R1A 2^4) of a
 * difference of one or two LSB, and the AGC DMP sees R1A as the word
 * pair PARAM left (00401, 40000) where C holds the value (Findings P1
 * in docs/modules/review-prog-rendz.md). For R = 3339068, V = 23241
 * 00170 the AGC gives ECC 0 and RAPO - RPER = 8 LSB, C ECC 23164 and
 * 4614 LSB: a word-form effect magnified by the SQRT, not a defect of
 * SR30.1.
 * AGC: SR30.1, MAXCHK (R30.agc pp.522-523)
 *
 * FUNCTION NAME: test_orbit_parameters_eccentric
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_orbit_parameters_eccentric(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_orbit_params_in_t in; /* SR30.1 inputs */

    in.rone         = pos(R0, 0.0, 0.0);
    CHECK(agc_orbit_params_select_body(false, agc_dp_from_double(RPAD, 29), &in) == AGC_STATUS_OK);
    const double k2 = 1.0404;               /* (v / v circular)^2 */
    const double ra = R0 * k2 / (2.0 - k2); /* apogee radius, m   */

    in.vone = vel(0.0, sqrt(k2 * MU / R0), 0.0);
    agc_orbit_params_out_t out; /* SR30.1 results */
    agc_status_t           st;  /* SR30.1 status  */

    st = agc_orbit_params_compute(&ip, &in, &out);

    CHECK(st == AGC_STATUS_OK);
    CHECK(dp_absdiff(out.rapo, agc_dp_from_double(ra, 29)) < 400);
    CHECK(dp_absdiff(out.rper, agc_dp_from_double(R0, 29)) < 400);
    CHECK(dp_absdiff(out.hapox, agc_dp_from_double(ra - RPAD, 29)) < 400);
    agc_dp_t h = agc_dp_from_double(R0 - RPAD, 29); /* perigee height, 2^29 */

    CHECK(dp_absdiff(out.hperx, h) < 400);
    CHECK(out.neg_tper <= 0);

    /* MAXCHK: 20000 NM is shown as 9999.9 NM. */
    agc_dp_t big = agc_dp_from_double(20000.0 * 1852.0, 29); /* m 2^29 */
    agc_dp_t lim = agc_orbit_params_maxchk(&ip, big);        /* m 2^29 */
    agc_dp_t max = agc_dp_from_double(9999.9 * 1852.0, 29);  /* m 2^29 */

    CHECK(dp_absdiff(lim, max) < 2);
    CHECK(agc_orbit_params_maxchk(&ip, h) == h);
}

/**********************************************************************
 * 2. CSI on coplanar circular orbits, the passive vehicle 15 NM higher
 * and leading by `th` radians at TCSI, TPI two hours later at 26.6 deg
 * elevation.
 * AGC: CSI/A (P32-P33_P72-P73.agc pp.663-674)
 *
 * FUNCTION NAME: csi_case
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   th        double      I    lead of the passive vehicle, radians
 *   delvcsi   agc_dp_t *  O    DELVCSI, m/cs 2^7
 *
 * RETURNS: agc_status_t -- the CSI/A status
 **********************************************************************/
static agc_status_t csi_case(double th, agc_dp_t *delvcsi)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    agc_rendz_reset(rz);
    agc_rendz_selectmu(rz, false);
    double ra = R0;                 /* active radius, m  */
    double rp = R0 + 15.0 * 1852.0; /* passive radius, m */

    rz->ract1      = pos(ra, 0.0, 0.0);
    rz->vact1      = vel(0.0, sqrt(MU / ra), 0.0);
    double vp      = sqrt(MU / rp);          /* passive speed, m/cs */
    double ttpi_cs = 2.0 * 3600.0 * 100.0;   /* TPI two hours on   */
    double th3     = th + vp / rp * ttpi_cs; /* passive at TTPI     */

    rz->rpass1         = pos(rp * cos(th), rp * sin(th), 0.0);   /* passive at TCSI */
    rz->vpass1         = vel(-vp * sin(th), vp * cos(th), 0.0);
    rz->rpass2         = rz->rpass1;
    rz->vpass2         = rz->vpass1;
    rz->rpass3         = pos(rp * cos(th3), rp * sin(th3), 0.0); /* P32/P72B INTINT */
    rz->vpass3         = vel(-vp * sin(th3), vp * cos(th3), 0.0);
    rz->ract2          = rz->ract1;
    rz->vact2          = rz->vact1;
    rz->up1            = agc_vec_make(0, 0, AGC_DP_HALF);        /* +z, 2^1     */
    rz->tcsi           = 0;
    rz->nn             = 1;
    rz->elev           = agc_dp_from_double(26.6 / 360.0, 0);
    agc_erase_set_tig(0);
    agc_erase_set_ttpi(agc_dp_from_double(ttpi_cs, 28));
    agc_sp_t     alarm = 0;                                      /* ALMXIT code, unscaled */
    agc_status_t st;                                             /* CSI/A                 */

    st       = agc_rendz_csi_a(&ip, rz, &alarm, AGC_JOB_CSI_COMPUTE);
    (void) printf("CSI status %d alarm %o DELVCSI %.3f fps\n", (int) st,
                  (unsigned int) alarm,
                  (double) rz->delvcsi / 2147483648.0 * 128.0 * 100.0 / 0.3048);
    *delvcsi = rz->delvcsi;

    return (st);
} /* csi_case */

/**********************************************************************
 * Small lead: without a burn the active vehicle would pass the passive
 * one before TPI, so CSI raises its orbit (DELVCSI > 0). Large lead: it
 * must catch up faster, so CSI lowers it (DELVCSI < 0).
 * AGC: CSI/A (P32-P33_P72-P73.agc pp.663-674)
 *
 * FUNCTION NAME: test_csi_coplanar_circular
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_csi_coplanar_circular(void)
{
    agc_dp_t     dv = 0;                   /* DELVCSI, m/cs 2^7 */
    agc_status_t st = csi_case(0.02, &dv); /* small lead        */

    CHECK(st == AGC_STATUS_OK);
    CHECK(dv > 0);
    st = csi_case(0.15, &dv);
    CHECK(st == AGC_STATUS_OK);
    CHECK(dv < 0);
}

/**********************************************************************
 * 3. PERIAPO1 of the circular orbit. The bound is loose: the circular
 * case is ill-conditioned (see test 1); C gives apsides 9 km apart where
 * the AGC gives 16 m (the R1A word form, review-prog-rendz P1).
 * AGC: SELECTMU, PERIAPO1 (P34-35_P74-75.agc pp.494-496)
 *
 * FUNCTION NAME: test_periapo_circular
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_periapo_circular(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    agc_rendz_selectmu(rz, false);
    agc_rendz_periapo_t pa;                               /* PERIAPO1 */
    agc_vec_t           v = vel(0.0, sqrt(MU / R0), 0.0); /* m/cs 2^7 */

    agc_rendz_periapo_body_scale(&ip, rz, pos(R0, 0.0, 0.0), v, &pa);
    CHECK(dp_absdiff(pa.rapo, pa.rperi) < 8192); /* 16 km */
}

/**********************************************************************
 * Restart group 2 as GOPROG does after STARTSB2 and look for the restart
 * job at the expected priority.
 * AGC: none (test support; RESTARTS ITSLIKEB, FRESH_START_AND_RESTART)
 *
 * FUNCTION NAME: group2_restarts_at
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   job       agc_job_id_t  I    expected restart job
 *   prio      agc_prio_t    I    expected restart priority
 *
 * RETURNS: bool -- the job was started at prio
 **********************************************************************/
static bool group2_restarts_at(agc_job_id_t job, agc_prio_t prio)
{
    bool found = false; /* job seen */

    agc_exec_init(); /* STARTSB2 */
    CHECK(agc_restart_group(2) == AGC_STATUS_OK);

    for (int i = 0; i < agc_exec_core_sets(); i++)
    {
        agc_exec_core_info_t ci; /* core set i */

        agc_exec_core_info(i, &ci);
        found = found || (ci.loc == job && ci.prio == prio);
    }

    return (found);
}

/**********************************************************************
 * P23 (P20-P25.agc p.619) presets PHSPRDT2 = PRIO13 and R22 (p.576)
 * PHSPRDT2 = PRIO26: a later group 2 restart of a type C phase with D
 * clear starts its job at that priority. LM R24LEM (p.515): the GRP2PC
 * at R24LEM2, reached from R24LEM inside the same job, restarts at
 * R24LEM2 (its return address), not at R24LEM.
 * AGC: P23, R22, R24LEM (P20-P25.agc pp.515, 576, 619)
 *
 * FUNCTION NAME: test_group2_restart_priority
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_group2_restart_priority(void)
{
    const agc_phase_var_t p23 = {AGC_RESTART_JOB, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION, AGC_TASK_NONE, 0, false, 0U}; /* D clear */
    const agc_phase_var_t r22 = {AGC_RESTART_JOB, AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM, AGC_TASK_NONE, 0, false, 0U};  /* D clear */

    agc_power_on(AGC_VEHICLE_CSM);
    agc_dsky_init();
    agc_flag_set(AGC_FLAG_REFSMFLG);
    agc_exec_init();                                                         /* only the job under test */
    CHECK(agc_exec_novac(013, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);
    agc_phase_change_var(06022, &p23);                                       /* type C, 2CADR, job, group 2 */
    CHECK(group2_restarts_at(AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION, 013));
    CHECK(agc_exec_novac(026, AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);
    CHECK(group2_restarts_at(AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_CHECK, 026)); /* REND1: WAITONE */
    agc_phase_change_var(06022, &r22);
    CHECK(group2_restarts_at(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM, 026));

    agc_power_on(AGC_VEHICLE_LM);
    agc_dsky_init();
    agc_exec_init();
    agc_phase_set_priority(2, 026);
    CHECK(agc_exec_novac(026, AGC_JOB_RADAR_SEARCH) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK); /* V16N80, the search goes on */
    CHECK(group2_restarts_at(AGC_JOB_RADAR_SEARCH_DISPLAY, 026));
}

/**********************************************************************
 * GET+MGA by hand: UNIT once (Comanche) or twice (Luminary), DOT the
 * REFSMMAT Y row, SL1, ARCSIN, plus one revolution when negative.
 * AGC: GET+MGA (P34-35_P74-75.agc C line 1336 / L line 1406)
 *
 * FUNCTION NAME: mga_expected
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  ----------------------------------------
 *   dv        agc_vec_t  I    delta V, m/cs 2^7
 *   ysm       agc_vec_t  I    REFSMMAT +6, 2^1
 *   twice     bool       I    Luminary's second UNIT
 *
 * RETURNS: agc_dp_t -- +MGA, revolutions 0 .. 1
 **********************************************************************/
static agc_dp_t mga_expected(agc_vec_t dv, agc_vec_t ysm, bool twice)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_vec_t u = agc_vec_unit(&ip, dv, NULL, NULL); /* UNIT, 2^1 */

    if (twice)
    {
        u = agc_vec_unit(&ip, u, NULL, NULL);
    }

    agc_dp_t a = agc_dp_asin(&ip, agc_dp_shl(&ip, agc_vec_dot(&ip, u, ysm), 1)); /* MGA, revolutions 2^0 */

    if (a < 0)
    {
        a = agc_dp_add(&ip, agc_dp_add(&ip, a, AGC_DP_HALF), AGC_DP_HALF); /* + 1 rev */
    }

    return (a);
}

/**********************************************************************
 * GET+MGA unitizes once on the CSM and twice on the LM: for a set of
 * delta-V directions each vehicle's +MGA equals its own rope's sequence,
 * and at least one direction shows the second UNIT moving the result.
 * AGC: GET+MGA (P34-35_P74-75.agc C line 1336 / L line 1406)
 *
 * FUNCTION NAME: test_get_mga_per_rope
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_get_mga_per_rope(void)
{
    agc_mat_t refsmmat;      /* REFSMMAT, 2^1: a skewed orthonormal frame */
    bool      moved = false; /* a direction where the second UNIT matters */

    refsmmat.row[0] = agc_vec_make(agc_dp_from_double(0.6, 1), agc_dp_from_double(0.8, 1), 0);
    refsmmat.row[1] = agc_vec_make(agc_dp_from_double(-0.8, 1), agc_dp_from_double(0.6, 1), 0);
    refsmmat.row[2] = agc_vec_make(0, 0, agc_dp_from_double(1.0, 1));

    for (int i = 0; i < 64; i++)
    {
        double    t  = 0.1 * (double) i;                                          /* direction parameter */
        agc_vec_t dv = vel(3.0 * cos(t), 2.0 * sin(1.7 * t) + 0.3, 1.1 * sin(t)); /* delta V, m/cs 2^7 (vel() scales it) */
        agc_dp_t  m[2];                                                           /* +MGA CSM, LM, revolutions 2^0 */

        for (int k = 0; k < 2; k++)
        {
            agc_interp_t ip; /* interpreter state */

            agc_state_init((k == 0) ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM); /* CSM first, then LM */
            agc_interp_init(&ip);
            agc_rendz_get_mga(&ip, dv, &refsmmat, &m[k]);
            CHECK(m[k] == mga_expected(dv, refsmmat.row[1], k == 1));
        }

        moved = moved || m[0] != m[1];
    }

    CHECK(moved);
    agc_state_init(AGC_VEHICLE_CSM);
}

/**********************************************************************
 * 6. V82GON1 (R30.agc p.519) after an escape burn: RN, VN on an earth
 * hyperbola (v = 1.1 escape speed) 40 degrees past its pericenter, the
 * pericenter above HPERMIN. SKIPTPER stores -TPER > 0 as is; the job
 * must not fail (it did: the display was left blank, NASSP TEI).
 * AGC: V82GON1, SR30.1, DOTPER, SKIPTPER (R30.agc pp.519-523)
 *
 * FUNCTION NAME: test_v82_average_g_departing
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_v82_average_g_departing(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    const double rp = R0;                                    /* pericenter radius, m        */
    const double vp = 1.1 * sqrt(2.0 * MU / rp);             /* pericenter speed, m/cs  */
    const double h  = rp * vp;                               /* angular momentum, m^2/cs    */
    const double e  = h * h / (MU * rp) - 1.0;               /* eccentricity (> 1)          */
    const double nu = 40.0 * 3.14159265358979323846 / 180.0; /* true anomaly, rad */
    const double r  = h * h / MU / (1.0 + e * cos(nu));      /* radius, m      */
    const double vr = MU / h * e * sin(nu);                  /* radial speed > 0, m/cs     */
    const double vt = h / r;                                 /* transverse speed, m/cs      */

    agc_flag_clear(AGC_FLAG_AMOONFLG);
    agc_erase_set_rn(pos(r, 0.0, 0.0));
    agc_erase_set_vn(vel(vr, vt, 0.0));
    agc_job_ctx_t ctx = {AGC_JOB_ORBIT_DISPLAY_AVERAGE_G, 0, false, -1, 0}; /* the V82GON1 job */
    agc_status_t  st  = agc_job_orbit_display_average_g(&ctx);              /* SR30.1 then V82GON3 */

    CHECK(st != AGC_STATUS_BAD_ARG);
    CHECK(agc_rendz_erasable()->neg_tper > 0);
}

/**********************************************************************
 * The registry dispatches P20 / P22 (V37E20E, V37E22E) and V92 to each
 * rope's own program: the LM's PROG20 (PROG22 = PROG20) and the CSM's
 * REDO (NEWMODEX 07, IMUTEST). Before the structure lane the vehicle
 * switch for P20 / P22 sat in csm/navigation and the LM column was empty,
 * and the CSM column of V92 was empty: those jobs ended NOT_FOUND.
 * AGC: PROG20, PROG22 (Luminary099 P20-P25.agc p.492); REDO (Comanche055
 * SYSTEM_TEST_STANDARD_LEAD_INS.agc p.422)
 *
 * FUNCTION NAME: test_vehicle_dispatch
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_vehicle_dispatch(void)
{
    agc_job_ctx_t ctx = {AGC_JOB_RENDEZVOUS_NAVIGATION, 013, false, -1, 0}; /* the V37 job */

    agc_state_init(AGC_VEHICLE_LM);
    CHECK(agc_dispatch_job(&ctx) != AGC_STATUS_NOT_FOUND); /* LM PROG20 */
    ctx.id = AGC_JOB_ORBITAL_NAVIGATION;
    CHECK(agc_dispatch_job(&ctx) != AGC_STATUS_NOT_FOUND); /* LM PROG22 = PROG20 */

    agc_state_init(AGC_VEHICLE_CSM);
    ctx.id   = AGC_JOB_SYSTEMS_TEST;
    ctx.prio = 020;
    CHECK(agc_dispatch_job(&ctx) != AGC_STATUS_NOT_FOUND); /* CSM REDO */
    CHECK(agc_erase_modreg() == 07);                       /* NEWMODEX 07 */
    agc_state_init(AGC_VEHICLE_CSM);
}
