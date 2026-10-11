/*
 * dap.c -- dap-lm unit tests (Luminary 099 LM digital autopilot).
 *
 *  1. 1/ACCS (AOSTASK_AND_AOSJOB pp.1485-1506), LM alone in descent at
 *     15000 kg: LEMMASS = MASS, positive one-jet accelerations, U,V
 *     coefficients of opposite sign, -2JETLIM = -BIT10, ACCSOKAY set,
 *     and the switch-curve block filled (1/ANETP > 0, deadbands = DB).
 *     A mass above HIDESCNT (15300 kg) is limited and MASS corrected.
 *  2. TJETLAW (TJET_LAW pp.1460-1469): zero error and zero rate is no
 *     firing (TJU = 0) on each axis.
 *  3. TJETLAW phase plane: a large positive error (+10 deg) at zero rate
 *     commands negative jets (TJ < 0, the correcting polarity); a large
 *     negative error commands positive jets.
 *  4. Rate estimator (BACKP / RATELOOP, the LM's "Kalman filter",
 *     pp.1423-1427, 1470-1471): with no jets and a constant P rate of
 *     10 CDU counts per 100 ms (OMEGAP = 400 at pi/4, 1.1 deg/s) the
 *     estimate converges to within 2 % in 30 DAP passes.
 *  5. RCSMON (RCS_FAILURE_MONITOR pp.190-192): closing the isolation
 *     valve of channel 32 bit 1 inhibits jets ch5 bit 2 and ch6 bit 7
 *     (5FAILTAB/6FAILTAB), reopening it clears them.
 *  6. SPSRCS (SPS_BACK-UP_RCS_CONTROL pp.1507-1510): a rate above
 *     1.73 deg/s commands -HALF, kept until the rate passes the
 *     -0.10 deg/s target; the stop sets the inhibition counter (UTIME =
 *     10 passes for U), which then inhibits a new firing.
 *  7. ROT-TOUV: with the docked orthogonal coefficients a pure +Q
 *     quantity gives +U and -V.
 *  8. 1/ACCS on the zero erasable of power-on (sim finding B2) runs to
 *     the end (status OK, ACCSOKAY set, no assertion), LEMMASS limited to
 *     LODESCNT + HIASCENT; an improper DV gives the hardware's result
 *     (the rope's BIGIQ divide in that run: 77470,00000 / 77652 ->
 *     A = 37777, L = 77652).
 */
#include <stdio.h>

#include <stdint.h>
#include <string.h>

#include "../../src/lm/dap/internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/guidance/guidance.h"
#include "agc/lm/dap.h"

static int g_checks;
static int g_failures;
static int g_assert_reports;

/**********************************************************************
 * AGC: none -- assertion hook: print and count.
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   file      const char *  I    source file of the failed assertion
 *   line      int           I    source line
 *   expr      const char *  I    text of the failed expression
 *
 * RETURNS: bool -- false (the assertion stays failed)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);
    g_assert_reports++;

    return (false);
}

#define CHECK(c) check((c), __LINE__, #c)

/**********************************************************************
 * AGC: none -- record one check.
 *
 * FUNCTION NAME: check
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  --------------------
 *   ok        bool          I    checked condition
 *   line      int           I    source line
 *   text      const char *  I    text of the condition
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

#define KG(x) ((agc_sp_t) ((x) / 4)) /* kg -> B16 SP (2^16 kg)   */

/**********************************************************************
 * Fresh start, LM alone, descent stage, given mass; then 1/ACCS.
 * AGC: none (helper)
 *
 * FUNCTION NAME: setup_descent
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  --------------------
 *   kg        int32_t  I    LM mass, kg
 *
 * RETURNS: agc_status_t -- status of 1/ACCS
 **********************************************************************/
static agc_status_t setup_descent(int32_t kg)
{
    CHECK(agc_lm_dap_fresh_start() == AGC_STATUS_OK);
    agc_flag_clear(AGC_FLAG_CSMDKFLG);
    agc_flag_clear(AGC_FLAG_APSFLAG);
    agc_flag_set(AGC_FLAG_DRIFTDFL);
    agc_erase_set_csmmass(0);
    agc_guid_state()->lm.mass   = (agc_dp_t) (KG(kg) * 16384);
    agc_guid_state()->lm.abdelv = 0;

    return (agc_lm_dap_jet_accel_update());
}

/**********************************************************************
 * AGC: none -- test 1 (1/ACCS, AOSTASK_AND_AOSJOB.agc).
 *
 * FUNCTION NAME: test_accs
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_accs(void)
{
    agc_status_t    st = setup_descent(15000);
    const lm_dap_t *d  = lm_dap_state();

    CHECK(st == AGC_STATUS_OK);
    CHECK(agc_erase_lemmass() == KG(15000));
    CHECK(d->jacc[0] > 0 && d->jacc[1] > 0 && d->jacc[2] > 0);
    CHECK(d->jacc[3] > 0 && d->jacc[3] == d->jacc[4]);
    CHECK(d->coeffq < 0 && d->coeffr > 0);
    CHECK(d->neg_2jetlim == -512);
    CHECK(agc_flag_test(AGC_FLAG_ACCOKFLG));
    CHECK(d->block[2] > 0); /* 1/ANETP                  */
    CHECK(d->block[LM_BLK_U + LM_BK_1ANET1 + 2] > 0);
    CHECK(d->block[6] > 0); /* drifting: FLAT = .8 deg  */
    CHECK(d->block[LM_BLK_U + LM_BK_FIREDB] == d->db);
    CHECK(d->l_pvt_cg > 0); /* engine arm, descent      */

    st = setup_descent(16000); /* above HIDESCNT           */
    CHECK(st == AGC_STATUS_OK);
    CHECK(agc_erase_lemmass() == 3825);
    CHECK(agc_guid_state()->lm.mass == (agc_dp_t) 3825 * 16384);
}

/**********************************************************************
 * AGC: none -- TJETLAW inputs for one axis.
 *
 * FUNCTION NAME: set_axis
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   axis      agc_sp_t    I    AXISCTR: -1 P, 0 U, 1 V
 *   e         agc_sp_t    I    attitude error E, scaled pi
 *   edot      agc_sp_t    I    error rate EDOT, scaled pi/4 rad/s
 *
 * RETURNS: none
 **********************************************************************/
static void set_axis(lm_dap_t *d, agc_sp_t axis, agc_sp_t e, agc_sp_t edot)
{
    d->axisctr      = axis;
    d->e            = e;
    d->edot         = edot;
    d->sensetyp     = 0;
    d->tj[axis + 1] = 0;
}

/**********************************************************************
 * AGC: none -- tests 2 and 3 (TJET_LAW.agc).
 *
 * FUNCTION NAME: test_tjetlaw
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_tjetlaw(void)
{
    CHECK(setup_descent(15000) == AGC_STATUS_OK);
    lm_dap_t *d = lm_dap_state();

    for (agc_sp_t axis = -1; axis <= 1; axis++)
    {
        set_axis(d, axis, 0, 0);
        CHECK(lm_dap_tjetlaw() == AGC_STATUS_OK);
        CHECK(d->tj[axis + 1] == 0);
    }

    agc_sp_t ten_deg = (agc_sp_t) (16384 * 10 / 180); /* at pi            */

    for (agc_sp_t axis = 0; axis <= 1; axis++)
    {
        set_axis(d, axis, ten_deg, 0);
        CHECK(lm_dap_tjetlaw() == AGC_STATUS_OK);
        CHECK(d->tj[axis + 1] < 0);
        set_axis(d, axis, (agc_sp_t) -ten_deg, 0);
        CHECK(lm_dap_tjetlaw() == AGC_STATUS_OK);
        CHECK(d->tj[axis + 1] > 0);
    }
}

/**********************************************************************
 * AGC: none -- test 4 (BACKP / RATELOOP).
 *
 * FUNCTION NAME: test_rate_estimator
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_rate_estimator(void)
{
    CHECK(setup_descent(15000) == AGC_STATUS_OK);
    lm_dap_t *d = lm_dap_state();

    d->nptraps = 2; /* STARTDAP                 */
    d->nqtraps = 2;
    d->nrtraps = 2;
    d->omegap  = 0;
    d->trapedp = 0;
    d->plast   = 0;

    for (int i = 0; i < 3; i++)
    {
        d->tj[i]     = 0;
        d->dapcdu[i] = 0;
    }

    d->oldxforp      = 0;
    d->oldyforp      = 0;
    d->oldzforq      = 0;
    int converged_at = -1;

    for (int pass = 1; pass <= 30; pass++)
    {
        d->dapcdu[0] = (uint16_t) ((d->dapcdu[0] + 10u) & 077777u);
        CHECK(lm_dap_rateloop() == AGC_STATUS_OK);
        CHECK(lm_dap_backp() == AGC_STATUS_OK);
        int32_t err  = (int32_t) d->omegap - 400;

        if (converged_at < 0 && err >= -8 && err <= 8)
        {
            converged_at = pass;
        }
    }

    CHECK(converged_at > 0);
    CHECK(d->omegap >= 392 && d->omegap <= 408);
    CHECK(d->omegaq == 0 && d->omegar == 0);
}

/**********************************************************************
 * AGC: none -- test 5 (RCS_FAILURE_MONITOR.agc).
 *
 * FUNCTION NAME: test_rcsmon
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_rcsmon(void)
{
    CHECK(agc_lm_dap_fresh_start() == AGC_STATUS_OK);
    const lm_dap_t *d = lm_dap_state();

    agc_chan_set_input(032, 077777u); /* all valves open (inverted) */
    CHECK(agc_lm_dap_rcsmon() == AGC_STATUS_OK);
    CHECK(d->ch5mask == 0u && d->ch6mask == 0u);
    agc_chan_set_input(032, 077776u); /* bit 1 low: valve closed  */
    CHECK(agc_lm_dap_rcsmon() == AGC_STATUS_OK);
    CHECK(d->ch5mask == 000002u);
    CHECK(d->ch6mask == 000100u);
    CHECK(d->pvalvest == 000001u);
    agc_chan_set_input(032, 077777u);
    CHECK(agc_lm_dap_rcsmon() == AGC_STATUS_OK);
    CHECK(d->ch5mask == 0u && d->ch6mask == 0u && d->pvalvest == 0u);
}

/**********************************************************************
 * AGC: none -- test 6 (SPS_BACK-UP_RCS_CONTROL.agc).
 *
 * FUNCTION NAME: test_spsrcs
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_spsrcs(void)
{
    CHECK(agc_lm_dap_fresh_start() == AGC_STATUS_OK);
    lm_dap_t *d = lm_dap_state();

    agc_flag_clear(AGC_FLAG_DRIFTDFL);
    d->jetctr[1] = 0;
    set_axis(d, 0, 0, 700);                       /* 1.92 deg/s               */
    CHECK(lm_dap_spsrcs() == AGC_STATUS_OK);
    CHECK(d->tj[1] == -8192);
    d->edot      = 0;                             /* target is -0.1 deg/s     */
    CHECK(lm_dap_spsrcs() == AGC_STATUS_OK);
    CHECK(d->tj[1] == -8192);                     /* keep firing              */
    d->edot      = -50;                           /* past target: stop        */
    CHECK(lm_dap_spsrcs() == AGC_STATUS_OK);
    CHECK(d->tj[1] == 0);
    CHECK(d->jetctr[1] == 10);
    d->e         = (agc_sp_t) (16384 * 10 / 180); /* outside deadband        */
    CHECK(lm_dap_spsrcs() == AGC_STATUS_OK);
    CHECK(d->tj[1] == 0);                         /* inhibited by counter     */
}

/**********************************************************************
 * AGC: none -- test 7 (ROT-TOUV).
 *
 * FUNCTION NAME: test_rot_touv
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_rot_touv(void)
{
    lm_dap_t *d = lm_dap_state();

    d->coeffq  = -11585;
    d->coeffr  = 11585;
    agc_sp_t u = 0;
    agc_sp_t v = 0;

    lm_dap_rot_touv(1000, 0, &u, &v);
    CHECK(u > 0 && v < 0);
    CHECK(u == -v);
}

/**********************************************************************
 * AGC: none -- test 8 (1/ACCS on zero erasable, sim B2).
 *
 * FUNCTION NAME: test_zero_erasable
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_zero_erasable(void)
{
    int reports = g_assert_reports;

    CHECK(agc_lm_dap_fresh_start() == AGC_STATUS_OK);
    (void) memset(lm_dap_state(), 0, sizeof (lm_dap_t));
    agc_flag_clear(AGC_FLAG_CSMDKFLG);
    agc_flag_clear(AGC_FLAG_APSFLAG);
    agc_flag_clear(AGC_FLAG_ACCOKFLG);
    agc_guid_state()->lm.mass   = 0;
    agc_guid_state()->lm.abdelv = 0;
    CHECK(agc_lm_dap_jet_accel_update() == AGC_STATUS_OK);
    CHECK(agc_flag_test(AGC_FLAG_ACCOKFLG));
    CHECK(agc_erase_lemmass() == 438); /* DEC 1750 B-16    */
    CHECK(g_assert_reports == reports);
    lm_dp_pair_t q              = lm_dap_dvqr((lm_dp_pair_t){-199, 0}, -85);

    CHECK(q.hi == 16383 && q.lo == -85);
}

/**********************************************************************
 * AGC: none -- run the tests.
 *
 * FUNCTION NAME: main
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- 0 when every check passed, else 1
 **********************************************************************/
int main(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    test_accs();
    test_tjetlaw();
    test_rate_estimator();
    test_rcsmon();
    test_spsrcs();
    test_rot_touv();
    test_zero_erasable();
    (void) printf("test_lm_dap: %d checks, %d failures, %d assertion reports\n", g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1);
}
