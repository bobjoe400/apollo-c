/*
 * test_csm_dap.c -- dap-csm unit tests (Comanche055 CSM RCS and TVC DAPs).
 *
 *  1. Jet selection (JET_SELECTION_LOGIC, PYTABLE p. 1042): +pitch with
 *     all quads good fires the documented pair, channel 5 bits 1 and 3
 *     (PYTABLE OCT 5125 masked by PJETS = OCT 1005: two jets), for
 *     TAU/2 T6 counts; -pitch fires bits 2 and 4; quad A failed leaves
 *     one jet (bit 1) for the whole TAU; on-times over 0.1 s are cut to
 *     .1 s with DFT = DFTMAX; +X translation uses bits 1, 4, 5, 8.
 *  2. TVC pitch DAP step response (TVCDAPS, OPTVARK p. 971): a positive
 *     body-axis error integral gives a negative actuator command (the
 *     sign change in the forward loop), and a large error is limited to
 *     ACTSAT = 253 (6 deg).
 *  3. MASSPROP / FIXCW (TVCMASSPROP) for the CSM alone: IXX = VARST0 +
 *     (CSMMASS + NEGBPW) VARST5 x 2.
 */
#include <stdio.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_csm_dap.h"
#include "agc/agc_channels.h"
#include "agc/agc_state.h"

static int g_checks;
static int g_failures;
static int g_assert_reports;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void)file;
    (void)line;
    (void)expr;
    g_assert_reports++;
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

static agc_csm_jet_in_t jet_in(agc_sp_t roll, agc_sp_t pitch, agc_sp_t yaw)
{
    agc_csm_jet_in_t in;
    in.chan31 = (uint16_t)077777;   /* THC in detent: bits read 1 (inactive) */
    in.lem_on = false;
    in.tau[0] = roll;
    in.tau[1] = pitch;
    in.tau[2] = yaw;
    in.racfail = 0;
    in.rbdfail = 0;
    in.acorbd = 1;                  /* BD roll                              */
    in.xtrans = 0;
    return in;
}

static void test_jet_select(void)
{
    agc_csm_jet_out_t out;
    agc_csm_jet_in_t in = jet_in(0, 100, 0);
    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK(out.pword1 == 01005u);                         /* 2 jets + count  */
    CHECK((out.pword1 & 017u) == (AGC_CH005_PYJET1 | AGC_CH005_PYJET3));
    CHECK(out.yword1 == 0u);
    CHECK(out.pword2 == 0u);                             /* off afterwards  */
    CHECK(out.blast[2].axis == AGC_CSM_JET_PITCH);       /* sorted last     */
    CHECK(out.blast[2].time == 50);                      /* TAU / 2 jets    */
    CHECK(out.dft[1] == 100);
    CHECK(out.tau[1] == 0);
    CHECK(out.attkalmn_xln == 0x7FFF);                   /* no translation  */

    in = jet_in(0, -100, 0);
    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK((out.pword1 & 017u) == (AGC_CH005_PYJET2 | AGC_CH005_PYJET4));
    CHECK(out.dft[1] == -100);

    in = jet_in(0, 100, 0);
    in.racfail = 1;                                      /* quad A failed   */
    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK(out.pword1 == 0401u);                          /* one jet         */
    CHECK(out.blast[2].time == 99);                      /* TAU x .99999    */

    in = jet_in(0, 1000, 0);
    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK(out.blast[2].time == 160);                     /* limited to .1 s */
    CHECK(out.dft[1] == 320);                            /* DFTMAX, 2 jets  */
    CHECK(out.tau[1] == 680);

    in = jet_in(0, 0, 0);
    in.chan31 = (uint16_t)(077777u & ~0100u);            /* +X: bit 7 low   */
    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK((uint16_t)(out.pword1 | out.yword1) == 0231u);
    CHECK(out.attkalmn_xln == -2);                       /* LM off          */
}

/* CSM/LM low-bandwidth pitch DAP (V46 switch-over coefficients).         */
static void tvc_setup(agc_sp_t perrb_hi)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_csm_dapcom_t *c = agc_csm_dap_com();
    agc_erase_set_dapdatr1((uint16_t)020000);            /* 010: CSM/LM     */
    c->ektlx_i[2] = 8000;
    c->iavg_tlx = 4000;
    c->pactoff = 0;
    c->yactoff = 0;
    t->tvcphase = 0;
    t->delpbar = 0;
    CHECK(agc_csm_tvc_preswtch() == AGC_STATUS_OK);      /* LBN10, VARK     */
    CHECK(t->vark == 7812);                              /* 8000 x 4000 x 4 */
    CHECK(t->n10[5] == 512);                             /* N20 = .03125    */
    t->t5tvcdt = 16380;
    t->perrb = agc_dp_from_sp(perrb_hi);
    t->omegac[1] = 0;
    t->pcmd = 0;
    t->stroker = 0;
    t->pcduypst = agc_erase_cduy();                      /* zero CDU rates  */
    t->pcduzpst = agc_erase_cduz();
}

static void test_tvc_step(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    tvc_setup(1000);
    CHECK(agc_task_pitchdap() == AGC_STATUS_OK);
    CHECK(t->pcmd < 0);                                  /* sign change     */
    CHECK(t->pcmd > -253);
    CHECK(t->tvcphase == 2);                             /* pitch copycycle */
    CHECK(agc_dp_high(t->perrb) == 1000);                /* integrator held */
    CHECK(agc_rupt_t5_task() != AGC_TASK_NONE);

    tvc_setup(-1000);
    CHECK(agc_task_pitchdap() == AGC_STATUS_OK);
    CHECK(t->pcmd > 0);

    tvc_setup(4000);                                     /* large error     */
    agc_csm_dap_tvc()->ptmp[0] = agc_dp_from_sp(16000);  /* filter memory   */
    CHECK(agc_task_pitchdap() == AGC_STATUS_OK);
    CHECK(t->pcmd == -253);                              /* ACTSAT limit    */
}

static void test_massprop(void)
{
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    agc_erase_set_dapdatr1((uint16_t)010000);            /* 001: CSM alone  */
    agc_erase_set_csmmass(5000);                         /* kg, B+16        */
    agc_erase_set_lemmass(0);
    CHECK(agc_csm_dap_massprop() == AGC_STATUS_OK);
    /* TEMP333 = 2 (5000 - 3851) = 2298 > 0: slopes VARST3, 4; VARST5.   */
    agc_sp_t d = 2298;
    agc_sp_t ixx = (agc_sp_t)(398 + 2 * ((394 * d) >> 14));
    CHECK(c->ixx == ixx);
    CHECK(c->weight_g == 5000);
}

int main(void)
{
    test_jet_select();
    test_tvc_step();
    test_massprop();
    (void)printf("test_csm_dap: %d checks, %d failures, %d assertion reports\n",
                 g_checks, g_failures, g_assert_reports);
    return (g_failures == 0) ? 0 : 1;
}
