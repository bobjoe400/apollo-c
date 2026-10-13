/**********************************************************************
 * FILE NAME: stroke.c
 *
 * PURPOSE:
 *   The TVC stroke test (V68): a waveform designed to excite bending,
 *   pulse bursts of ESTROKER dumped directly into TVCPITCH in proper sign
 *   and sequence, from both TVC DAP passes and an intermediate waitlist
 *   call (2 DAPs x 2 passes/DAP x 2 cs/pass = 8 cs = TVCDT). Pitch axis
 *   only, CSM/LM only; restarts terminate it and another V68 is then
 *   required. The official waveform (3 Jan 1967) has four stroke sets:
 *   set 1: 10 bursts in 1/2 amplitude, 4 reversals; set 2: 6, 6; set 3:
 *   5, 10; set 4: 4, 14. Amplitude 10 ESTROKER (1/42.15) deg, nominally
 *   50/42.15 = 1.185 deg. Labels: STRKTSTI, STRKCHK, TSTINIT, HACK,
 *   HACKWLST, STROKILL, REVUP; FCADDY, FREVS, FCARD, FCARD1..6.
 *
 * AGC SOURCE:
 *   Comanche055 TVCSTROKETEST.agc pp. 979-983
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   STROKER       agc_sp_t  I/O  85.41"/bit     burst, 0 = no test
 *   ESTROKER      agc_sp_t  I    85.41"/bit     pad-loaded burst size
 *   CADDY, N, REVS, CARD  agc_sp_t  I/O  unscaled  waveform counters
 *   BUNKER        bool      I/O  -              return: DAP or TASKOVER
 *   TVCPITCH      agc_sp_t  I/O  85.41"/bit     optics error counter
 *   DAPDATR1      uint16_t  I    bits           bit 14: CSM/LM
 *   CHAN14        uint16_t  O    bits           11: count release
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_waitlist_call     WAITLIST HACKWLST in 20 ms
 *   csm_dap_ads           ADS (mysubs.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC; a WAITLIST failure (1203 raised by the executive)
 *   is passed on. AGC_STATUS_BAD_ARG for a missing job context or HACK
 *   without a stroke test.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   HACK runs in the T5 interrupt from the TVC DAPs, HACKWLST as a
 *   waitlist task. STROKER -0 ("a prior V68") is not represented.
 **********************************************************************/
#include "../internal.h"

#define HACK_DELAY ((uint16_t) 2)  /* 20MS = BIT2, cs                   */
#define FCADDY     ((agc_sp_t) 10) /* bursts in 1/2 amplitude, set 1    */
#define FREVS      ((agc_sp_t) 3)  /* reversals minus 1, set 1          */
#define FCARD      ((agc_sp_t) 4)  /* number of stroke sets             */

/* FCARD1..3: reversals minus 1 for sets 2..4, indexed by CARD -3..-1
 * (INDEX CARD; CAF FCARD +4); FCARD4..6: bursts in 1/2 amplitude
 * (INDEX CARD; CS FCARD +7).                                             */
static const agc_sp_t fcard_revs[3]   = {5, 9, 13};
static const agc_sp_t fcard_bursts[3] = {6, 5, 4};

/**********************************************************************
 * STRKTSTI, STRKCHK (TVCSTROKETEST.agc p. 982) -- job set up by V68:
 * TSTINIT, then (INHINT) start the test on the next DAP pass if CSM/LM
 * (DAPDATR1 bit 14): STROKER = ESTROKER; ENDOFJOB.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (ENDOFJOB, always)
 **********************************************************************/
agc_status_t agc_job_strktsti(void)
{
    /* TSTINIT (TVCSTROKETEST.agc p. 982) -- CADDY = N = -FCADDY, REVS =
     * FREVS, CARD = -FCARD (sign changes + to -). */
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    t->caddy        = (agc_sp_t) -FCADDY; /* CS FCADDY; TS CADDY             */
    t->n            = (agc_sp_t) -FCADDY; /* TS N                            */
    t->revs         = FREVS;              /* CAF FREVS; TS REVS              */
    t->card         = (agc_sp_t) -FCARD;  /* CS FCARD; TS CARD               */
    agc_status_t st = AGC_STATUS_OK;      /* TCR TSTINIT                 */

    agc_inhint();

    if ((agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u)
    {
        agc_csm_dap_tvc()->stroker = agc_csm_dap_com()->estroker; /* CAE ESTROKER; TS STROKER */
    }

    agc_relint();

    return (st); /* ENDOFJOB                                            */
}

/**********************************************************************
 * HACKWLST +2 .. REVUP (TVCSTROKETEST.agc pp. 982-983) -- one pulse
 * burst into TVCPITCH and the count release, count down the bursts of
 * this slope (INCR CADDY: exit while on a slope) and at the end of a
 * slope reverse (REVUP: CADDY = 2N; final reversal of the set: REVS -1,
 * CADDY = N), changing the sign of STROKER; or end the set.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void stroke_burst(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    agc_erase_set_tvcpitch(csm_dap_ads(agc_erase_tvcpitch(), t->stroker)); /* CA STROKER; ADS TVCPITCH */
    agc_chan_or(AGC_CHAN_CHAN14, (uint16_t) AGC_CH014_DRIVECDUS);          /* CAF BIT11; WOR CHAN14    */
    t->caddy = (agc_sp_t) (t->caddy + 1);                                  /* INCR CADDY               */

    if (t->caddy >= 0) /* CS CADDY; BZMF: exit while on a slope                 */
    {
        if (t->revs > 0)
        { /* CCS REVS: REVUP */
            t->revs  = (agc_sp_t) (t->revs - 1); /* TS REVS                    */
            t->caddy = csm_dap_ads(t->n, t->n);  /* CA N; DOUBLE: 2 x 1/2 amplitude */
        }
        else if (t->revs == 0)
        { /* REVUP +4: final reversal, this set */
            t->revs  = -1;   /* CS ONE; TS REVS: prepare for a new burst     */
            t->caddy = t->n; /* CA N: just return to zero                    */
        }
        else
        {
            /* exit, end of set                           */
            /* HACKWLST +12 .. STROKILL (TVCSTROKETEST.agc pp. 982-983) -- end of a
             * stroke set (CCS REVS negative): INCR CARD; with all sets completed
             * STROKER = +0 (STROKILL), otherwise REVS, N, CADDY for the next set. */
            agc_csm_tvc_t *t_set = agc_csm_dap_tvc(); /* TVC DAP erasables          */

            t_set->card = (agc_sp_t) (t_set->card + 1); /* INCR CARD                    */

            if (t_set->card >= 0)
            { /* CS CARD; BZF STROKILL: all sets completed */
                t_set->card    = 0; /* (AGC: -0)                                     */
                t_set->stroker = 0; /* STROKILL: TS STROKER (+0) ends the test       */
            }
            else
            {
                int k = (int) t_set->card + 3; /* CARD -3, -2, -1 -> 0, 1, 2           */

                t_set->revs  = fcard_revs[k];               /* INDEX CARD; CAF FCARD +4 */
                t_set->n     = (agc_sp_t) -fcard_bursts[k]; /* INDEX CARD; CS FCARD +7  */
                t_set->caddy = t_set->n;                    /* TS N; TS CADDY           */
            }

            return;
        }

        t->stroker = (agc_sp_t) -t->stroker; /* CS STROKER; TS STROKER: change sign of slope */
    }
} /* stroke_burst */

/**********************************************************************
 * HACK (TVCSTROKETEST.agc p. 982) -- entry in the T5 interrupt from the
 * TVC DAPs (BUNKER = return to the DAP): WAITLIST HACKWLST in 20 ms,
 * then the burst (TCF HACKWLST +2).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the WAITLIST status; AGC_STATUS_BAD_ARG with
 *          no stroke test in progress
 **********************************************************************/
agc_status_t csm_tvc_hack(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    if (!AGC_ASSERT(t->stroker != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    t->bunker_task  = false;                                            /* QXCH BUNKER        */
    agc_status_t st = agc_waitlist_call(HACK_DELAY, AGC_TASK_HACKWLST); /* CAF 20MS; WAITLIST */

    stroke_burst();

    return (st); /* TC BUNKER: back to the DAP                          */
}

/**********************************************************************
 * HACKWLST (TVCSTROKETEST.agc p. 982) -- entry from the waitlist
 * (BUNKER = TC TASKOVER): the burst.
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_task_hackwlst(void)
{
    agc_csm_dap_tvc()->bunker_task = true; /* CAF TCTSKOVR; TS BUNKER    */
    stroke_burst();
}
