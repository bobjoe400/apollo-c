/**********************************************************************
 * FILE NAME: dap.c
 *
 * PURPOSE:
 *   dap-csm unit tests (Comanche055 CSM RCS and TVC DAPs):
 *   1. Jet selection (JET_SELECTION_LOGIC, PYTABLE p. 1042): +pitch with
 *      all quads good fires the documented pair, channel 5 bits 1 and 3
 *      (PYTABLE OCT 5125 masked by PJETS = OCT 1005: two jets), for
 *      TAU/2 T6 counts; -pitch fires bits 2 and 4; quad A failed leaves
 *      one jet (bit 1) for the whole TAU; on-times over 0.1 s are cut to
 *      .1 s with DFT = DFTMAX; +X translation uses bits 1, 4, 5, 8.
 *   2. TVC pitch DAP step response (TVCDAPS, OPTVARK p. 971): a positive
 *      body-axis error integral gives a negative actuator command (the
 *      sign change in the forward loop), and a large error is limited to
 *      ACTSAT = 253 (6 deg).
 *   3. Stroke test (TVCSTROKETEST p. 982): set 1 is 10 bursts in 1/2
 *      amplitude with 4 reversals = 80 bursts returning the waveform to
 *      zero, then set 2 is loaded (REVS 5, N -6); after all four sets
 *      (80 + 72 + 100 + 112 bursts) STROKER is +0.
 *   4. Roll DAP (TVCROLLDAP pp. 989-997): a +10 deg OGA error at rest is
 *      region 6 and fires negative roll torque, jets 12 and 10 (chan 6
 *      OCT 12); inside the deadband at rest is a coast.
 *   5. MASSPROP / FIXCW (TVCMASSPROP) for the CSM alone: IXX = VARST0 +
 *      (CSMMASS + NEGBPW) VARST5 x 2.
 *   6. T6START (JET_SELECTION_LOGIC pp. 1061-1062): one T6 interrupt
 *      with all three intervals expired replaces the roll, pitch and yaw
 *      words (a REPLACE does not end the T6RUPTOR scan), and a positive
 *      interval becomes the next TIME6 wait.
 *   7. S40.14 (P40-P47 p. 735) forms signed gains from a negative IXX.
 *
 * AGC SOURCE:
 *   Comanche055 JET_SELECTION_LOGIC.agc, TVCDAPS.agc, TVCSTROKETEST.agc,
 *   TVCROLLDAP.agc, TVCMASSPROP.agc (pages above)
 *
 * EXTERNAL VARIABLES:
 *   none (the DAP blocks are reached through agc_csm_dap_rcs() /
 *   _tvc() / _com())
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_csm_jet_select    JETSLECT .. CALCDT6
 *   agc_task_pitchdap     PITCHDAP
 *   agc_task_hackwlst     HACKWLST
 *   agc_task_rolldap      ROLLDAP
 *   agc_csm_dap_massprop  MASSPROP
 *   agc_task_t6start      T6START
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   exit status 1 when a check fails
 **********************************************************************/
#include <stdio.h>

#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"

static int g_checks;         /* checks run                          */
static int g_failures;       /* checks failed                       */
static int g_assert_reports; /* library assertions reported         */

static void             check(bool ok, int line, const char *text);
static agc_csm_jet_in_t jet_in(agc_sp_t roll, agc_sp_t pitch, agc_sp_t yaw);
static void             test_jet_select(void);
static void             tvc_setup(agc_sp_t perrb_hi);
static void             test_tvc_step(void);
static int              stroke_bursts(int count, int pos);
static void             test_stroke(void);
static void             test_rolldap(void);
static void             test_massprop(void);
static void             test_t6start(void);
static void             test_rcs_gains_signed(void);

/**********************************************************************
 * AGC: none -- run every test; the exit status reports failures.
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- 0 when every check passes, 1 otherwise
 **********************************************************************/
int main(void)
{
    test_jet_select();
    test_tvc_step();
    test_stroke();
    test_rolldap();
    test_massprop();
    test_t6start();
    test_rcs_gains_signed();
    (void) printf("test_csm_dap: %d checks, %d failures, %d assertion reports\n", g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1); /* ?: exit status */
}

/**********************************************************************
 * AGC: none -- assertion hook of the library: count and continue.
 *
 * ARGUMENTS:
 *   file  const char *  I  source file of the failed assertion
 *   line  int           I  its line
 *   expr  const char *  I  the asserted expression
 *
 * RETURNS: bool -- false (the assertion stays failed)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) file;
    (void) line;
    (void) expr;
    g_assert_reports++;

    return (false);
}

#define CHECK(c) check((c), __LINE__, #c) /* record one check      */

/**********************************************************************
 * AGC: none -- record one check and print a failure.
 *
 * ARGUMENTS:
 *   ok    bool          I  the check passed
 *   line  int           I  source line of the check
 *   text  const char *  I  the checked expression
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
 * AGC: none -- jet selection inputs: THC in detent, all quads good, BD
 * roll.
 *
 * ARGUMENTS:
 *   roll   agc_sp_t  I  TAU, unscaled 1/1600 s
 *   pitch  agc_sp_t  I  TAU1, unscaled 1/1600 s
 *   yaw    agc_sp_t  I  TAU2, unscaled 1/1600 s
 *
 * RETURNS: agc_csm_jet_in_t -- the inputs
 **********************************************************************/
static agc_csm_jet_in_t jet_in(agc_sp_t roll, agc_sp_t pitch, agc_sp_t yaw)
{
    agc_csm_jet_in_t in; /* jet selection inputs                     */

    in.chan31  = (uint16_t) 077777; /* THC in detent: bits read 1 (inactive) */
    in.lem_on  = false;
    in.tau[0]  = roll;
    in.tau[1]  = pitch;
    in.tau[2]  = yaw;
    in.racfail = 0;
    in.rbdfail = 0;
    in.acorbd  = 1;                 /* BD roll                              */
    in.xtrans  = 0;

    return (in);
}

/**********************************************************************
 * JETSLECT .. CALCDT6 (JET_SELECTION_LOGIC.agc pp. 1039-1060) -- test 1.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_jet_select(void)
{
    agc_csm_jet_out_t out;                    /* jet selection outputs */
    agc_csm_jet_in_t  in = jet_in(0, 100, 0); /* +pitch, 100 counts    */

    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK(out.pword1 == 01005u);                   /* 2 jets + count  */
    CHECK((out.pword1 & 017u) == (AGC_CH005_PYJET1 | AGC_CH005_PYJET3));
    CHECK(out.yword1 == 0u);
    CHECK(out.pword2 == 0u);                       /* off afterwards  */
    CHECK(out.blast[2].axis == AGC_CSM_JET_PITCH); /* sorted last     */
    CHECK(out.blast[2].time == 50);                /* TAU / 2 jets    */
    CHECK(out.dft[1] == 100);
    CHECK(out.tau[1] == 0);
    CHECK(out.attkalmn_xln == 0x7FFF);             /* no translation  */

    in = jet_in(0, -100, 0);
    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK((out.pword1 & 017u) == (AGC_CH005_PYJET2 | AGC_CH005_PYJET4));
    CHECK(out.dft[1] == -100);

    in         = jet_in(0, 100, 0);
    in.racfail = 1;                 /* quad A failed   */
    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK(out.pword1 == 0401u);     /* one jet         */
    CHECK(out.blast[2].time == 99); /* TAU x .99999    */

    in = jet_in(0, 1000, 0);
    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK(out.blast[2].time == 160); /* limited to .1 s */
    CHECK(out.dft[1] == 320);        /* DFTMAX, 2 jets  */
    CHECK(out.tau[1] == 680);

    in        = jet_in(0, 0, 0);
    in.chan31 = (uint16_t) (077777u & ~0100u); /* +X: bit 7 low   */
    CHECK(agc_csm_jet_select(&in, &out) == AGC_STATUS_OK);
    CHECK((uint16_t) (out.pword1 | out.yword1) == 0231u);
    CHECK(out.attkalmn_xln == -2);             /* LM off          */
}

/**********************************************************************
 * PRESWTCH (TVCINITIALIZE.agc p. 941) -- set up the CSM/LM low-bandwidth
 * pitch DAP (V46 switch-over coefficients) with a body error integral.
 *
 * ARGUMENTS:
 *   perrb_hi  agc_sp_t  I  upper word of PERRB, scale B-1 rev
 **********************************************************************/
static void tvc_setup(agc_sp_t perrb_hi)
{
    agc_csm_tvc_t    *t = agc_csm_dap_tvc(); /* TVC DAP erasables       */
    agc_csm_dapcom_t *c = agc_csm_dap_com(); /* pad loads               */

    agc_erase_set_dapdatr1((uint16_t) 020000);      /* 010: CSM/LM     */
    c->ektlx_i[2] = 8000;
    c->iavg_tlx   = 4000;
    c->pactoff    = 0;
    c->yactoff    = 0;
    t->tvcphase   = 0;
    t->delpbar.hi = 0;
    t->delpbar.lo = 0;
    CHECK(agc_csm_tvc_preswtch() == AGC_STATUS_OK); /* LBN10, VARK     */
    CHECK(t->vark == 7812);                         /* 8000 x 4000 x 4 */
    CHECK(t->n10[5] == 512);                        /* N20 = .03125    */
    t->t5tvcdt    = 16380;
    t->perrb.hi   = perrb_hi;
    t->perrb.lo   = 0;
    t->omegac[1]  = 0;
    t->pcmd       = 0;
    t->stroker    = 0;
    t->pcduypst   = agc_erase_cduy();               /* zero CDU rates  */
    t->pcduzpst   = agc_erase_cduz();

    for (int i = 0; i < 6; i++)
    { /* filter memory   */
        t->ptmp[i].hi = 0;
        t->ptmp[i].lo = 0;
    }
}

/**********************************************************************
 * PITCHDAP (TVCDAPS.agc pp. 963-971) -- test 2.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_tvc_step(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    tvc_setup(1000);
    CHECK(agc_task_pitchdap() == AGC_STATUS_OK);
    CHECK(t->pcmd < 0);         /* sign change     */
    CHECK(t->pcmd > -253);
    CHECK(t->tvcphase == 2);    /* pitch copycycle */
    CHECK(t->perrb.hi == 1000); /* integrator held */
    CHECK(agc_rupt_t5_task() != AGC_TASK_NONE);

    tvc_setup(-1000);
    CHECK(agc_task_pitchdap() == AGC_STATUS_OK);
    CHECK(t->pcmd > 0);

    /* Saturation needs filter memory: the error alone gives at most
     * 4096 x N10 x N20 x N30 x VARK x 4 = 122 < ACTSAT. TMP3 (second
     * cascade) = 1000 -> DAP2 = 1125, DAP3 = 562, CMDTMP = -1072. A large
     * TMP1 would overflow DAP1 (DAS wraps as in the AGC), so use TMP3.   */
    tvc_setup(4000);        /* large error     */
    agc_csm_dap_tvc()->ptmp[2].hi = 1000;
    CHECK(agc_task_pitchdap() == AGC_STATUS_OK);
    CHECK(t->pcmd == -253); /* ACTSAT limit    */
}

/**********************************************************************
 * HACKWLST (TVCSTROKETEST.agc p. 982) -- run bursts; the waveform
 * position follows STROKER's sign.
 *
 * ARGUMENTS:
 *   count  int  I  bursts to run (stops early at STROKILL)
 *   pos    int  I  waveform position, unscaled 85.41 arcsec/bit
 *
 * RETURNS: int -- the new position, unscaled 85.41 arcsec/bit
 **********************************************************************/
static int stroke_bursts(int count, int pos)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* STROKER                    */

    for (int i = 0; (i < count) && (t->stroker != 0); i++)
    {
        pos += t->stroker;
        CHECK(agc_task_hackwlst() == AGC_STATUS_OK);
        CHECK((pos >= -50) && (pos <= 50)); /* 10 x ESTROKER   */
    }

    return (pos);
}

/**********************************************************************
 * TSTINIT .. STROKILL (TVCSTROKETEST.agc pp. 982-983) -- test 3.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_stroke(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    t->caddy   = -10;                  /* TSTINIT: -FCADDY */
    t->n       = -10;
    t->revs    = 3;                    /* FREVS           */
    t->card    = -4;                   /* -FCARD          */
    t->stroker = 5;                    /* ESTROKER        */
    int pos    = stroke_bursts(80, 0); /* set 1, unscaled 85.41 arcsec/bit */

    CHECK(pos == 0);
    CHECK(t->card == -3);
    CHECK(t->revs == 5);
    CHECK(t->caddy == -6);
    CHECK(t->stroker == 5);                   /* no sign change  */
    pos = stroke_bursts(72 + 100 + 111, pos); /* sets 2, 3, 4    */
    CHECK(t->stroker == 5);
    pos = stroke_bursts(1, pos);
    CHECK(pos == 0);
    CHECK(t->stroker == 0);                   /* STROKILL        */
}

/**********************************************************************
 * ROLLDAP (TVCROLLDAP.agc pp. 989-997) -- test 4.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_rolldap(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    t->oganow     = 0;
    t->ogapast    = 0;                               /* OGARATE 0       */
    t->one_conacc = 1000;
    t->rollfire   = 0;
    t->temreg     = 0;
    t->rollword   = 0;
    t->ogaerr     = 455;                             /* 10 deg, B+0 rev */
    CHECK(agc_task_rolldap() == AGC_STATUS_OK);
    CHECK(t->rollfire < 0);                          /* -TORQUE         */
    CHECK(t->temreg > 24);                           /* > TMINFIRE      */
    CHECK(agc_chan_read(AGC_CHAN_ROLLJETS) == 012u); /* jets 12, 10     */
    CHECK(t->rollword == 2u);
    CHECK(agc_task_roll_jets_off() == AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_ROLLJETS) == 0u);

    t->temreg = 0;
    t->ogaerr = 100;         /* inside 5 deg DB */
    CHECK(agc_task_rolldap() == AGC_STATUS_OK);
    CHECK(t->rollfire == 0); /* coast           */
    CHECK(agc_chan_read(AGC_CHAN_ROLLJETS) == 0u);
}

/**********************************************************************
 * MASSPROP, FIXCW (TVCMASSPROP.agc pp. 953-955) -- test 5.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_massprop(void)
{
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* outputs           */

    agc_erase_set_dapdatr1((uint16_t) 010000); /* 001: CSM alone  */
    agc_erase_set_csmmass(5000);               /* kg, B+16        */
    agc_erase_set_lemmass(0);
    agc_csm_dap_massprop();

    /* TEMP333 = 2 (5000 - 3851) = 2298 > 0: slopes VARST3, 4; VARST5.   */
    agc_sp_t d   = 2298;                                     /* TEMP333, scale B+15 kg   */
    agc_sp_t ixx = (agc_sp_t) (398 + 2 * ((394 * d) >> 14)); /* IXX, scale B+20 kg m^2   */

    CHECK(c->ixx == ixx);
    CHECK(c->weight_g == 5000);
}

/**********************************************************************
 * T6START, T6RUPTOR, REPLACE* (JET_SELECTION_LOGIC.agc pp. 1061-1062)
 * -- test 6.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_t6start(void)
{
    agc_csm_rcs_t    *r = agc_csm_dap_rcs(); /* jet words, BLASTs        */
    agc_csm_dapcom_t *c = agc_csm_dap_com(); /* RCSFLAGS                 */

    c->rcsflags = 1u;    /* BIT1: channels already initialised        */
    agc_chan_write(AGC_CHAN_ROLLJETS, 0u);
    agc_chan_write(AGC_CHAN_PYJETS, 0u);
    r->rword2   = 0120u; /* roll second word                            */
    r->pword2   = 01005u;
    r->yword2   = 0u;

    for (int k = 0; k < 3; k++)
    {
        r->blast[k].time = 0;
    }

    r->blast[0].axis = AGC_CSM_JET_ROLL;
    r->blast[1].axis = AGC_CSM_JET_PITCH;
    r->blast[2].axis = AGC_CSM_JET_YAW;
    agc_rupt_set_time(6, 0u);                         /* TIME6 +0: the T6RUPT happened          */
    CHECK(agc_task_t6start() == AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_ROLLJETS) == 0120u); /* REPLACER        */
    CHECK(agc_chan_read(AGC_CHAN_PYJETS) == 01005u);  /* REPLACEP, -Y    */
    CHECK(r->blast[0].time == -1);
    CHECK(r->blast[1].time == -1);
    CHECK(r->blast[2].time == -1);                    /* the scan reached BLAST2           */

    r->blast[1].time = 40;          /* pitch interval still to run               */
    r->blast[2].time = 0;
    agc_rupt_set_time(6, 0u);
    CHECK(agc_task_t6start() == AGC_STATUS_OK);
    CHECK(agc_rupt_time(6) == 40u); /* ZBLAST1; ENABT6                 */
    CHECK(r->blast[1].time == 0);
    CHECK(r->blast[2].time == 0);   /* untouched until the next T6RUPT    */
}

/**********************************************************************
 * S40.14 (P40-P47.agc p. 735) -- test 7: the RCS gains are formed with
 * the AGC's signed MP / DV whatever the inertias; a negative IXX (from
 * nonsense V48 masses) gives negative J/M and KMJ, no refusal.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_rcs_gains_signed(void)
{
    agc_csm_dapcom_t    *c = agc_csm_dap_com(); /* IXX, IAVG */
    const agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* J/M, KMJ  */

    c->ixx  = -4000; /* B+20 kg m^2 */
    c->iavg = 4000;
    agc_csm_dap_rcs_gains();
    CHECK(r->jm[0] < 0 && r->kmj[0] < 0);
    CHECK(r->jm[1] > 0 && r->kmj[1] > 0 && r->kmj[2] == r->kmj[1]);
}
