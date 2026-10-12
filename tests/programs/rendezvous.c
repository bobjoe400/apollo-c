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

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
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
    agc_status_t        st;                               /* PERIAPO1 */

    st = agc_rendz_periapo_body_scale(&ip, rz, pos(R0, 0.0, 0.0), v, &pa);
    CHECK(st == AGC_STATUS_OK);
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

    CHECK(agc_exec_init() == AGC_STATUS_OK); /* STARTSB2 */
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

    CHECK(agc_power_on(AGC_VEHICLE_CSM) == AGC_STATUS_OK);
    CHECK(agc_dsky_init() == AGC_STATUS_OK);
    agc_flag_set(AGC_FLAG_REFSMFLG);
    CHECK(agc_exec_init() == AGC_STATUS_OK);                                 /* only the job under test */
    CHECK(agc_exec_novac(013, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);
    agc_phase_change_var(06022, &p23);                                       /* type C, 2CADR, job, group 2 */
    CHECK(group2_restarts_at(AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION, 013));
    CHECK(agc_exec_novac(026, AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK);
    CHECK(group2_restarts_at(AGC_JOB_RENDEZVOUS_NAVIGATION_CSM_CHECK, 026)); /* REND1: WAITONE */
    agc_phase_change_var(06022, &r22);
    CHECK(group2_restarts_at(AGC_JOB_RENDEZVOUS_TRACKING_DATA_CSM, 026));

    CHECK(agc_power_on(AGC_VEHICLE_LM) == AGC_STATUS_OK);
    CHECK(agc_dsky_init() == AGC_STATUS_OK);
    CHECK(agc_exec_init() == AGC_STATUS_OK);
    agc_phase_set_priority(2, 026);
    CHECK(agc_exec_novac(026, AGC_JOB_RADAR_SEARCH) == AGC_STATUS_OK);
    CHECK(agc_exec_run_one() == AGC_STATUS_OK); /* V16N80, the search goes on */
    CHECK(group2_restarts_at(AGC_JOB_RADAR_SEARCH_DISPLAY, 026));
}
