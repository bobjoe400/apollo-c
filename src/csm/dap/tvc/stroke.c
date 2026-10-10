/*
 * stroke.c -- the TVC stroke test (V68).
 * TVCSTROKETEST.agc (Comanche055 pp. 979-983): STRKTSTI, STRKCHK,
 * TSTINIT, HACK, HACKWLST, STROKILL, REVUP and the waveform parameters.
 *
 * Generates a waveform designed to excite bending: pulse bursts of
 * ESTROKER (pad load, 85.41 arcsec/bit) dumped directly into TVCPITCH, in
 * proper sign and sequence, from both TVC DAP passes and an intermediate
 * waitlist call (2 DAPs x 2 passes/DAP x 2 cs/pass = 8 cs = TVCDT). The
 * test runs in the pitch axis only and only for CSM/LM. Restarts
 * terminate it; another V68 is then required. The official waveform
 * (3 Jan 1967) has four stroke sets:
 *   set 1: 10 bursts in 1/2 amplitude, 4 reversals; set 2: 6, 6;
 *   set 3: 5, 10; set 4: 4, 14.
 * Amplitude 10 ESTROKER (1/42.15) deg, nominally 50/42.15 = 1.185 deg.
 */
#include "../internal.h"

#define HACK_DELAY ((uint16_t) 2)  /* 20MS = BIT2 (cs)                 */
#define FCADDY     ((agc_sp_t) 10) /* bursts in 1/2 amplitude, set 1    */
#define FREVS      ((agc_sp_t) 3)  /* reversals minus 1, set 1          */
#define FCARD      ((agc_sp_t) 4)  /* number of stroke sets             */

/* FCARD1..3: reversals minus 1 for sets 2..4, indexed by CARD -3..-1
 * (INDEX CARD; CAF FCARD +4); FCARD4..6: bursts in 1/2 amplitude
 * (INDEX CARD; CS FCARD +7).                                             */
static const agc_sp_t fcard_revs[3]   = {5, 9, 13};
static const agc_sp_t fcard_bursts[3] = {6, 5, 4};

/* TSTINIT (p. 982): CADDY = N = -FCADDY, REVS = FREVS, CARD = -FCARD.    */
agc_status_t csm_tvc_tstinit(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    t->caddy = (agc_sp_t) -FCADDY; /* sign change (+ to -) */
    t->n     = (agc_sp_t) -FCADDY;
    t->revs  = FREVS;
    t->card  = (agc_sp_t) -FCARD;

    if (!AGC_ASSERT(t->caddy < 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(t->card < 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK; /* TC Q                 */
}

/* STRKTSTI, STRKCHK (p. 982): job set up by V68. Start the test on the
 * next DAP pass if CSM/LM (DAPDATR1 bit 14), otherwise exit.             */
agc_status_t agc_job_strktsti(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = csm_tvc_tstinit();

    agc_inhint();

    if ((agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u)
    {
        agc_csm_dap_tvc()->stroker = agc_csm_dap_com()->estroker;
    }

    agc_relint();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st; /* ENDOFJOB             */
}

/* End of a stroke set (CCS REVS negative): the next set, or STROKILL.    */
static void stroke_next_set(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    t->card = (agc_sp_t) (t->card + 1); /* INCR CARD            */

    if (t->card >= 0)
    {                   /* all sets completed   */
        t->card    = 0; /* (AGC: -0)            */
        t->stroker = 0; /* STROKILL: +0 ends it */

        return;
    }

    int k = (int) t->card + 3; /* CARD -3, -2, -1      */

    if (!AGC_ASSERT((k >= 0) && (k < 3)))
    {
        t->stroker = 0;

        return;
    }

    t->revs  = fcard_revs[k]; /* reinitialise         */
    t->n     = (agc_sp_t) -fcard_bursts[k];
    t->caddy = t->n;

    if (!AGC_ASSERT(t->n < 0))
    {
        return;
    }
}

/* HACKWLST +3 .. REVUP (pp. 982-983): one pulse burst into TVCPITCH,
 * count down the bursts of this slope and handle the reversals.          */
static agc_status_t stroke_burst(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    if (!AGC_ASSERT((t->caddy < 0) || (t->stroker == 0))) /* +0: STROKILL */
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_erase_set_tvcpitch(csm_dap_ads(agc_erase_tvcpitch(), t->stroker));
    agc_chan_or(AGC_CHAN_CHAN14, (uint16_t) AGC_CH014_DRIVECDUS);
    t->caddy = (agc_sp_t) (t->caddy + 1); /* INCR CADDY           */

    if (t->caddy < 0)
    {
        return AGC_STATUS_OK; /* exit, on a slope     */
    }

    if (t->revs > 0)
    { /* REVUP                */
        t->revs  = (agc_sp_t) (t->revs - 1);
        t->caddy = csm_dap_ads(t->n, t->n); /* 2 x 1/2 amplitude    */
    }
    else if (t->revs == 0)
    {                    /* final reversal, set  */
        t->revs  = -1;   /* prepare for new burst*/
        t->caddy = t->n; /* just return to zero  */
    }
    else
    {
        stroke_next_set(); /* exit, end of set     */

        return AGC_STATUS_OK;
    }

    t->stroker = (agc_sp_t) -t->stroker; /* change sign of slope */

    if (!AGC_ASSERT(t->caddy <= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK; /* exit at a reversal   */
}

/* HACK (p. 982): entry in the T5 interrupt from the TVC DAPs (BUNKER =
 * return to the DAP): set up HACKWLST in 20 ms, then the burst.          */
agc_status_t csm_tvc_hack(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    if (!AGC_ASSERT(t->stroker != 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    t->bunker_task = false; /* QXCH BUNKER          */
    agc_status_t st = agc_waitlist_call(HACK_DELAY, AGC_TASK_HACKWLST);
    agc_status_t s2 = stroke_burst();

    if (!AGC_ASSERT(!t->bunker_task))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return (st != AGC_STATUS_OK) ? st : s2;
}

/* HACKWLST (p. 982): entry from the waitlist; BUNKER = TC TASKOVER.     */
agc_status_t agc_task_hackwlst(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    t->bunker_task = true;
    agc_status_t st = stroke_burst();

    if (!AGC_ASSERT(t->bunker_task))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT((t->card >= -FCARD) && (t->card <= 0)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st; /* TASKOVER             */
}
