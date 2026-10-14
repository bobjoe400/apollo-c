/**********************************************************************
 * FILE NAME: entry_initial_roll.c
 *
 * PURPOSE:
 *   The initial roll phase of entry: INITROLL, D0EQ, INROLOUT, KATEST and
 *   the HUNTEST trigger (FOREHUNT), and the PREHUNT job that starts the
 *   range prediction.
 *
 * AGC SOURCE:
 *   Comanche055 REENTRY_CONTROL.agc pp. 853-855 (INITROLL .. KATEST,
 *   PREHUNT); Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   entry erasables agc_entry_state_t IO  ENTRY_LEXICON  agc_entry_state()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_exec_*            PREHUNT job
 *   entry_post_0_05g      P64 (programs.c)
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
#include "agc/io/service.h"
#include "internal.h"

/* ======================================================================
 * INITIAL ROLL (pp. 853-855), MM 63/64
 * ==================================================================== */

/**********************************************************************
 * INROLOUT (p. 854): BOFSET INRLSW -> LIMITL/D (set at the end for restart
 * protection). Reached only with INRLSW clear.
 *
 * FUNCTION NAME: step_inrolout
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *
 * RETURNS: entry_step_t -- the step the AGC branches to next (its label),
 *          or ST_ERROR when an assertion fails
 **********************************************************************/
static entry_step_t step_inrolout(void)
{
    bool was_set = agc_flag_test(AGC_FLAG_INRLSW); /* INRLSW before this pass */

    agc_flag_set(AGC_FLAG_INRLSW);

    return (was_set ? ST_KATEST : ST_LIMITLD); /* INRLSW */
}

/**********************************************************************
 * FOREHUNT (p. 863): initialise HUNTEST.
 *
 * FUNCTION NAME: entry_forehunt
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: none
 **********************************************************************/
static void entry_forehunt(agc_entry_state_t *es)
{
    es->diffold = 0;
    es->dlewd   = AGC_ENTRY_DLEWD0;
    es->lewd    = AGC_ENTRY_LEWD1;
}

/**********************************************************************
 * PRE-HUNT (p. 863): restart point of group 4 after SIDETRAK.
 *
 * FUNCTION NAME: agc_job_pre_hunt
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_pre_hunt(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_HIND);
    entry_forehunt(es);
    es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST;

    return (AGC_STATUS_OK); /* next SERVICER pass continues at HUNTEST */
}

/* ======================================================================
 * SEQUENCER (p. 847)
 * ==================================================================== */

/**********************************************************************
 * One labelled section of the sequencer; returns the next.  Split along the
 * AGC's own sections ("* START ..." in REENTRY_CONTROL.agc).  Dispatch
 * within the INITIAL ROLL section (p. 853), the roll held until KAT and
 * then VRTHRESH are exceeded: steps INITROLL .. INITRL1.
 *
 * FUNCTION NAME: entry_initial_roll_step
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   p         entry_pass_t *  IO   state of this guidance pass:
 *                                  interpreter, entry erasables, MPAC and
 *                                  push-down words
 *   s         entry_step_t    I    step to run (INITROLL .. INITRL1)
 *
 * RETURNS: entry_step_t -- the step the AGC branches to next (its label),
 *          or ST_ERROR when an assertion fails
 **********************************************************************/
entry_step_t entry_initial_roll_step(entry_pass_t *p, entry_step_t s)
{
    switch ((int) s)
    {
        case ST_INITROLL:
            /* INITROLL (p. 853): hold the HEADSUP roll until .05 g; at .05 g (P64) KAT
             * = min(LEQ^3 KA1 + KA2, KALIM), then D0EQ or (V < VFINAL1) KEP2. */
        {
            agc_interp_t      *ip = p->ip; /* interpreter state */
            agc_entry_state_t *es = p->es; /* entry erasables */
            agc_dp_t           kat;        /* KAT/805, B0 */

            if (agc_flag_test(AGC_FLAG_INRLSW))
            {
                return (ST_HUNT_TRIGGER);
            }

            if (!agc_flag_test(AGC_FLAG__05GSW))
            {
                return (ST_LIMITLD); /* D < .05 g             */
            }

            kat = agc_dp_mul_trunc(agc_dp_sq(es->leq), es->leq);
            kat = agc_dp_div(ip, kat, AGC_ENTRY_ONE_OVER_KA1);
            kat = agc_dp_add(ip, kat, AGC_ENTRY_KA2);

            const agc_status_t st = entry_post_0_05g(es); /* status of the post-0.05g computation */

            if (st != AGC_STATUS_OK)
            { /* RTB P64: MM 64, N74   */
                p->status = st;

                return (ST_ERROR);
            }

            es->kat = (agc_dp_sub(ip, kat, AGC_ENTRY_KALIM) < 0) ? kat : AGC_ENTRY_KALIM; /* KALIM limit */

            /* CLEAR GONEPAST: ROLLC follows HEADSUP only until P64.               */
            agc_flag_clear(AGC_FLAG_GONEPAST);

            if (agc_dp_sub(ip, es->v, AGC_ENTRY_VFINAL1) >= 0)
            {
                return (ST_D0EQ);
            }

            es->gotoaddr = AGC_ENTRY_GOTO_KEPLER; /* idle until D > .2 g   */

            return (ST_INROLOUT);
        } /* end ST_INITROLL case */
        case ST_D0EQ:
            /* D0EQ (pp. 853-854): constant drag level D0 = KA3 LEQ + KA4, C/D0 = (-4/25
             * g)/D0 B-8; L/D = LAD, or -LAD when V - VFINAL + (RDOT/V)^3 / K44 is
             * negative. */
        {
            agc_interp_t      *ip = p->ip; /* interpreter state */
            agc_entry_state_t *es = p->es; /* entry erasables */
            agc_dp_t           r;          /* LEQ/D0 ratio, B0 */
            agc_dp_t           x;          /* r^3, B0 */

            es->d0 = agc_dp_add(ip, agc_dp_mul_trunc(es->leq, AGC_ENTRY_KA3), AGC_ENTRY_KA4);

            es->c_d0 = agc_dp_div(ip, AGC_ENTRY_C001, es->d0); /* BDDV       */
            agc_interp_clear_ovf(ip);                          /* BOV +1     */
            es->l_d  = es->lad;
            r        = agc_dp_div(ip, es->rdot, es->v);        /* RDOT/V     */
            x        = agc_dp_mul_trunc(agc_dp_sq(r), r);
            x        = agc_dp_div(ip, x, AGC_ENTRY_ONE_OVER_K44);
            x        = agc_dp_sub(ip, x, AGC_ENTRY_VFINAL);
            x        = agc_dp_add(ip, x, es->v);

            if (agc_interp_test_ovf(ip) || x < 0)
            {
                return (ST_INROLOUT);
            }

            es->l_d = agc_dp_neg(es->lad);

            return (ST_INROLOUT);
        }
        case ST_INROLOUT:
            return (step_inrolout());
        case ST_KATEST:
            /* KATEST (p. 854): if KAT - D >= 0 keep the roll (LIMITL/D), else CONSTD. */
        {
            return ((agc_dp_sub(p->ip, p->es->kat, p->es->d) >= 0) ? ST_LIMITLD : ST_CONSTD); /* KAT test */
        }
        case ST_HUNT_TRIGGER:
            /* INITRL1 (p. 854): when RDOT + VRCONT >= 0 start HUNTEST, else KATEST. */
        {
            agc_entry_state_t *es = p->es; /* entry erasables */

            if (agc_dp_add(p->ip, es->rdot, AGC_ENTRY_VRCONT) < 0)
            {
                return (ST_KATEST);
            }

            entry_forehunt(es);
            es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST; /* after FOREHUNT (restarts)   */

            return (ST_HUNTEST);
        }
        default:
            break;
    } /* end switch ((int) s) */

    return (ST_ERROR); /* unreachable: the range was asserted */
} /* entry_initial_roll_step */
