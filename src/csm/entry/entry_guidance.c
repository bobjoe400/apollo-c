/**********************************************************************
 * FILE NAME: entry_guidance.c
 *
 * PURPOSE:
 *   CM entry guidance ("Colossus 2A" entry steering of GSOP AS-278
 *   section 5.6): the per-pass driver. SCALEPOP (via CM/POSE) runs
 *   TARGETNG, then the phase GOTOADDR names (initial roll, HUNTEST,
 *   UPCONTROL, KEP2, PREDICT3), the lateral logic and roll command, and
 *   ENDEXIT's entry display; the shared entry interpreter state and the
 *   SL n on the TP MPAC the entry code uses.
 *
 * AGC SOURCE:
 *   Comanche055 REENTRY_CONTROL.agc p. 847 (SCALEPOP), pp. 873-874
 *   (ENDEXIT, OVERNOUT, NODISKY), the REFAZE10 restart; Luminary099:
 *   none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   entry erasables agc_entry_state_t IO  ENTRY_LEXICON  agc_entry_state()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_entry_targeting   TARGETNG (entry_targeting.c)
 *   entry_*_step          the phase steps (internal.h)
 *   agc_dsky_display      OVERNOUT entry display
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none of its own; a failing called routine ends the pass (ST_ERROR)
 *   and its status is returned.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only; runs in the SERVICER job (one pass per 2 s via CM/POSE).
 *
 * NOTES:
 *   Each labelled AGC section is a step function returning the next step
 *   (entry_step_t, internal.h); entry_sequencer() (entry_guidance.c) runs
 *   the chain. Values the AGC carries in MPAC or in the push-down list
 *   between labelled sections travel in entry_pass_t. Scales: /2VS, /805,
 *   revs (agc/csm/entry.h, entry_lexicon.h). Every DMP is truncating (the
 *   listing has no DMPR); DOT/DMP followed by SL, SQRT or ACOS keeps the
 *   TP MPAC (review-guid-entry F-E1).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "internal.h"

/* Interpreter state shared by the entry interpretive code (OVFIND).        */
static agc_interp_t entry_interp;               /* entry interpreter state */
static bool         entry_interp_ready = false; /* entry_interp initialised */

/**********************************************************************
 * AGC: none (the MPAC area of the entry jobs) -- the shared interpreter
 * state for the entry lane (OVFIND carried between routines as in the AGC).
 *
 * FUNCTION NAME: entry_ip
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_interp_t * -- agc_interp_t * -- the entry interpreter state
 *          (never NULL)
 **********************************************************************/
agc_interp_t *entry_ip(void)
{
    if (!entry_interp_ready)
    {
        agc_interp_init(&entry_interp);
        entry_interp_ready = true;
    }

    return (&entry_interp);
}

/**********************************************************************
 * AGC: none (SL n of the interpreter, INTERPRETER.agc p. 1154 TSSL) -- SL n
 * on the TP MPAC a DMP or DOT leaves; MPAC+2 bits move up into MPAC+1.
 *
 * FUNCTION NAME: entry_tp_shl
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *   m         agc_tp_t        I    TP MPAC, any scale
 *   n         int             I    left shift count, 0..14
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- MPAC, MPAC+1 after the shift (2^n x the
 *          input scale); overflow wraps and sets OVFIND
 **********************************************************************/
agc_dp_t entry_tp_shl(agc_interp_t *ip, agc_tp_t m, int n)
{
    return (agc_tp_to_dp(agc_tp_shift(ip, m, n)));
}

/**********************************************************************
 * GOTO GOTOADDR (p. 847): first step of the pass.
 *
 * FUNCTION NAME: entry_first_step
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   g         agc_entry_goto_t  I    GOTOADDR
 *
 * RETURNS: entry_step_t -- entry_step_t -- the step that GOTO GOTOADDR
 *          enters
 **********************************************************************/
static entry_step_t entry_first_step(agc_entry_goto_t g)
{
    static const entry_step_t map[AGC_ENTRY_GOTO_COUNT] = { ST_INITROLL, ST_HUNTEST, ST_UPCONTRL, ST_KEPLER_TEST, ST_PREFINAL, ST_FINAL_PREDICT, ST_ENDEXIT, ST_FINAL_PHASE_LATLONG };

    return (map[g]);
}

/**********************************************************************
 * AGC: none (the GOTO chain of REENTRY_CONTROL.agc pp. 847-875) -- run the
 * chain of sections from s to a terminal step (bounded).
 *
 * FUNCTION NAME: entry_sequencer
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   p         entry_pass_t *  IO   state of this guidance pass:
 *                                  interpreter, entry erasables, MPAC and
 *                                  push-down words
 *   s         entry_step_t    I    first step
 *
 * RETURNS: entry_step_t -- entry_step_t -- the terminal step (ENDEXIT,
 *          SERVNOUT, STEEROFF or ST_ERROR)
 **********************************************************************/
static entry_step_t entry_sequencer(entry_pass_t *p, entry_step_t s)
{
    int n; /* step count (bound) */

    for (n = 0; n < AGC_ENTRY_SEQ_MAX_STEPS; n++)
    {
        if (s == ST_ENDEXIT || s == ST_SERVNOUT || s == ST_ERROR)
        {
            return (s);
        }

        if (s < ST_HUNTEST)
        {
            s = entry_initial_roll_step(p, s);
        }
        else if (s < ST_GOTOUPSY)
        {
            s = entry_huntest_step(p, s);
        }
        else if (s < ST_KEP)
        {
            s = entry_upcontrol_step(p, s);
        }
        else if (s < ST_PREFINAL)
        {
            s = entry_ballistic_step(p, s);
        }
        else
        {
            s = entry_final_phase_step(p, s);
        }
    }

    return (ST_ERROR); /* unreachable: step bound exhausted */
} /* entry_sequencer */

/**********************************************************************
 * ENDEXIT / OVERNOUT / NODISKY (pp. 873-874): the entry display ENTRYVN
 * (REGODSPR, no abort if the DSKY is busy) when ENTRYDSP is set.
 *
 * FUNCTION NAME: entry_endexit
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ctx       agc_job_ctx_t *      I    job context of the running job
 *                                       (display and DELAYJOB
 *                                       continuations)
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t entry_endexit(agc_job_ctx_t *ctx, agc_entry_state_t *es)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_flag_test(AGC_FLAG_ENTRYDSP)) /* NODISKY          */
    {
        agc_status_t st = agc_dsky_display(ctx, AGC_DSKY_REGODSPR, agc_dsky_vn(es->entry_verb, es->entry_noun), NULL); /* display status */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (AGC_STATUS_OK); /* REGODSPR: never aborts here    */
}

/**********************************************************************
 * REFAZE10 (p. 847): restart 5.3, then the steering phase named by
 * GOTOADDR. *display tells the caller whether the pass ends at ENDEXIT
 * (entry display) or at SERVNOUT (no display). Also the group 5 restart
 * point (the AGC's PHASCHNG OCT 10035 return).
 *
 * FUNCTION NAME: agc_entry_reestablish_sequencer
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description (scale)
 *   --------  ------  ---  ----------------------------------------------
 *   display   bool *  O    true: the pass ends at ENDEXIT (entry display);
 *                          false: SERVNOUT
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG on a failed
 *          assertion; else the status of a called routine
 **********************************************************************/
agc_status_t agc_entry_reestablish_sequencer(bool *display)
{
    entry_pass_t    pass; /* state of this pass */
    entry_step_t    last; /* terminal step */
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_REESTABLISH_ENTRY_SEQUENCER, AGC_TASK_NONE, 0, false, 0U };

    *display       = false;
    agc_phase_change_var(010035, &v); /* SERVICER 5.3        */
    pass.ip        = entry_ip();
    pass.es        = agc_entry_state();
    pass.mpac      = 0;
    pass.pd_y      = 0;
    pass.pd_ld     = agc_dp_to_words(0);
    pass.dref      = agc_dp_to_words(0);
    pass.hunt_iter = 0;
    pass.status    = AGC_STATUS_OK;
    last           = entry_sequencer(&pass, entry_first_step(pass.es->gotoaddr));

    if (last == ST_ERROR)
    {
        return ((pass.status != AGC_STATUS_OK) ? pass.status : AGC_STATUS_BAD_ARG); /* failure first */
    }

    *display = (last == ST_ENDEXIT);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SCALEPOP (p. 847): TARGETNG, then REFAZE10.
 *
 * FUNCTION NAME: agc_entry_scalepop
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description (scale)
 *   --------  ------  ---  ----------------------------------------------
 *   display   bool *  O    true: the pass ends at ENDEXIT (entry display)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG on a failed
 *          assertion; else the TARGETNG / REFAZE10 status
 **********************************************************************/
agc_status_t agc_entry_scalepop(bool *display)
{
    *display = false;

    agc_entry_targeting();

    return (agc_entry_reestablish_sequencer(display));
}
