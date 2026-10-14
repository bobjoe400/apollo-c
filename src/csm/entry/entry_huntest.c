/**********************************************************************
 * FILE NAME: entry_huntest.c
 *
 * PURPOSE:
 *   The range prediction of entry: HUNTEST (the reference trajectory's
 *   exit velocity and flight path angle, DHOOKYQ7) and RANGER (the range
 *   components ASKEP, ASP1, ASPUP, ASPDWN, ASP3 against THETAH), with
 *   the iteration on L/D (GETDLEWD, SIDETRAK) and the test entries.
 *
 * AGC SOURCE:
 *   Comanche055 REENTRY_CONTROL.agc pp. 855-863 (HUNTEST .. RANGER ..
 *   GETDLEWD); LOG is CSM_GEOMETRY's (agc_csm_log); Luminary099: none
 *   (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   entry erasables agc_entry_state_t IO  ENTRY_LEXICON  agc_entry_state()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_csm_log           LOG (CSM_GEOMETRY, RANGER)
 *   agc_exec_prio_change  PRIOCHNG
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
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "internal.h"

/* ======================================================================
 * HUNTEST (pp. 855-858, 862) and RANGER (pp. 859-861)
 * ==================================================================== */

/**********************************************************************
 * DHOOKYQ7 (p. 862): ((1 - x/FACT1)^2 - ALP) / FACT2, x = VS1 or VL.
 *
 * FUNCTION NAME: entry_dhookyq7
 *
 * ARGUMENTS:
 *   Argument  Type                       I/O  Description (scale)
 *   --------  -------------------------  ---  ---------------------------
 *   ip        agc_interp_t *             IO   interpreter state of the job:
 *                                             OVFIND, DVSIGN, alarms
 *   es        const agc_entry_state_t *  I    entry erasables
 *                                             (ENTRY_LEXICON names)
 *   x         agc_dp_t                   I    VS1 or VL, /2VS B1
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- DHOOK or Q7, /805 B0
 **********************************************************************/
static agc_dp_t entry_dhookyq7(agc_interp_t *ip, const agc_entry_state_t *es, agc_dp_t x)
{
    agc_dp_t m; /* MPAC, B0 */

    m = agc_dp_div(ip, agc_dp_shr(x, 1), es->fact1);           /* SR1 DDV    */
    m = agc_dp_shl(ip, agc_dp_sub(ip, AGC_ENTRY_HALVE, m), 1); /* BDSU SL1 */
    m = agc_dp_sub(ip, agc_dp_sq(m), es->alp);

    return (agc_dp_div(ip, m, es->fact2));
}

/**********************************************************************
 * HUNTEST (pp. 855-858) without the sequencing: runs from the present state
 * through HUNTEST3 and reports where the AGC continues.
 *
 * FUNCTION NAME: entry_huntest_core
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *   gammal    agc_dp_t *           O    GAMMAL found, revs B0
 *
 * RETURNS: agc_entry_hunt_result_t -- agc_entry_hunt_result_t -- where the
 *          AGC continues (RANGER, CONSTD, PREFINAL, ...)
 **********************************************************************/
static agc_entry_hunt_result_t entry_huntest_core(agc_interp_t      *ip,
                                                  agc_entry_state_t *es,
                                                  agc_dp_t          *gammal)
{
    /* HUNTEST .. V1LEAD (p. 855): A1 = D, TEM1B, V1 = V + RDOT/TEM1B, A0 =
     * (V1/V)^2 (D + RDOT^2/(2 C1 HS TEM1B)), A1 = A0 if RDOT < 0, and V1 = V1 -
     * VQUIT when L/D < 0. */
    es->a1    = es->d;
    es->tem1b = (es->rdot < 0) ? es->lad : es->lewd; /* RDOT sign */

    /* On the MPAC words: DAD leaves V1, and the DMPs leave VL, with words
     * that may disagree in sign; the DDVs, DMPs and SQRT that follow read
     * them as stored (review-guid-entry: ALP 13571613 C, 13571567 AGC).  */
    const agc_dp_words_t c1hs = agc_dp_to_words(AGC_ENTRY_TWO_C1_HS);                                                    /* 2C1HS */
    const agc_dp_words_t near = agc_dp_to_words(AGC_ENTRY_NEARONE);                                                      /* BARELY1 */
    agc_tp_words_t       m    = agc_mpac_ddv(ip, agc_mpac_dload(agc_dp_to_words(es->rdot)), agc_dp_to_words(es->tem1b)); /* A0CALC: RDOT/TEM1B */
    agc_dp_words_t       v1   = agc_mpac_store(agc_mpac_dad(ip, m, agc_dp_to_words(es->v)));                             /* V1 = V + RDOT/TEM1B, /2VS */

    /* RDOT DSQ DDV TEM1B DDV 2C1HS DAD D DMP V1 DMP V1 DDV VSQUARE: A0.   */
    m = agc_mpac_ddv(ip, agc_mpac_ddv(ip, agc_mpac_dsq(agc_mpac_dload(agc_dp_to_words(es->rdot))), agc_dp_to_words(es->tem1b)), c1hs);
    m = agc_mpac_dmp(agc_mpac_dmp(agc_mpac_dad(ip, m, agc_dp_to_words(es->d)), v1), v1);

    const agc_dp_words_t a0 = agc_mpac_store(agc_mpac_ddv(ip, m, agc_dp_to_words(es->vsquare))); /* A0, /805 B0 */

    es->a0 = agc_dp_from_words(a0);

    if (es->rdot < 0)
    {
        es->a1 = es->a0;
    }

    if (es->l_d < 0)
    { /* V1LEAD: DLOAD V1 DSU VQUIT */
        v1 = agc_mpac_store(agc_mpac_dsu(ip, agc_mpac_dload(v1), agc_dp_to_words(AGC_ENTRY_VQUIT)));
    }

    es->v1 = v1; /* STORE V1: the words as the DAD / DSU left them */

    /* HUNTEST1 (p. 856): ALP = 2 C1 HS A0/LEWD V1 V1 (DMP DDV DDV DDV).    */
    m = agc_mpac_ddv(ip, agc_mpac_dmp(agc_mpac_dload(a0), c1hs), v1);
    m = agc_mpac_ddv(ip, agc_mpac_ddv(ip, m, v1), agc_dp_to_words(es->lewd));

    const agc_dp_words_t alp = agc_mpac_store(m); /* ALP */

    /* BDSU BARELY1 BDDV V1: FACT1 = V1 / (1 - ALP).                        */
    const agc_dp_words_t fact1 = agc_mpac_store(agc_mpac_bddv(ip, agc_mpac_bdsu(ip, m, near), v1)); /* FACT1, /2VS */

    /* DLOAD ALP DSU BARELY1 DMP ALP DDV A0: FACT2 = ALP(ALP - 1)/A0.      */
    m = agc_mpac_ddv(ip, agc_mpac_dmp(agc_mpac_dsu(ip, agc_mpac_dload(alp), near), alp), a0);

    const agc_dp_words_t fact2 = agc_mpac_store(m); /* x 805 */

    /* DMP Q7 DAD ALP SQRT BDSU BARELY1 DMP FACT1: VL = FACT1 (1 - SQRT(Q7
     * FACT2 + ALP)); the DAD leaves MPAC+2 of the DMP product for SQRT.  */
    m = agc_mpac_sqrt(agc_mpac_dad(ip, agc_mpac_dmp(m, agc_dp_to_words(es->q7)), alp));
    m = agc_mpac_dmp(agc_mpac_bdsu(ip, m, near), fact1);

    const agc_dp_words_t vl = agc_mpac_store(m); /* VL, /2VS */

    /* BDSU V1 DMP LEWD DDV VL: GAMMAL1 = LEWD (V1 - VL)/VL.               */
    m = agc_mpac_ddv(ip, agc_mpac_dmp(agc_mpac_bdsu(ip, m, v1), agc_dp_to_words(es->lewd)), vl);

    es->alp     = agc_dp_from_words(alp);
    es->fact1   = agc_dp_from_words(fact1);
    es->fact2   = agc_dp_from_words(fact2);
    es->vl      = agc_dp_from_words(vl);
    es->gammal1 = agc_dp_from_words(agc_mpac_store(m));

    if (agc_dp_sub(ip, es->vl, AGC_ENTRY_VLMIN) < 0)
    {
        return (AGC_ENTRY_HUNT_PREFINAL); /* VL < VLMIN  */
    }

    es->vbars = agc_dp_sq(es->vl);
    es->dvl   = agc_dp_sub(ip, AGC_ENTRY_HALVE, es->vl); /* VSAT - VL   */

    if (es->dvl < 0)
    {
        return (AGC_ENTRY_HUNT_CONSTD); /* BECONSTD    */
    }

    es->vs1 = AGC_ENTRY_HALVE; /* VS1 = VSAT  */

    if (agc_dp_sub(ip, AGC_ENTRY_HALVE, agc_dp_from_words(es->v1)) < 0)
    {
        es->dvl = agc_dp_sub(ip, agc_dp_from_words(es->v1), es->vl); /* V1 - VL     */
        es->vs1 = agc_dp_from_words(es->v1);
    }

    /* GETDHOOK .. HUNTEST3 (pp. 857-858): DHOOK, AHOOKDV, GAMMAL, and the
     * smoothed GAMMAL1 = GAMMAL1 + Q19 (GAMMAL - GAMMAL1). */
    agc_dp_t m_getdhook; /* MPAC, scale per step */
    agc_dp_t g;          /* GAMMAL, revs B0 */

    es->dhook   = entry_dhookyq7(ip, es, es->vs1);
    m_getdhook  = agc_dp_div(ip, agc_dp_shr(es->dhook, 6), es->q7);
    es->ahookdv = agc_dp_sub(ip, m_getdhook, AGC_ENTRY_CHOOK);
    m_getdhook  = agc_dp_mul_trunc(agc_dp_add(ip, es->ahookdv, AGC_ENTRY_ONE_16TH), AGC_ENTRY_CH1);
    m_getdhook  = agc_dp_mul_trunc(agc_dp_mul_trunc(m_getdhook, es->dvl), es->dvl);
    m_getdhook  = agc_dp_div(ip, agc_dp_div(ip, m_getdhook, es->dhook), es->vbars);
    g           = agc_dp_sub(ip, es->gammal1, m_getdhook);

    if (g < 0)
    {
        /* NEGAMA (p. 862): GAMMAL < 0. Move VL so that GAMMAL = 0: VL = VL +
         * (GAMMAL VL/3) / (LEWD/3 - DVL (AHOOKDV + 1/24) CH1/DHOOK VL), Q7 =
         * DHOOKYQ7(VL), VBARS = VL^2; returns GAMMAL = 0. */
        agc_dp_t num; /* GAMMAL VL/3, B0 */
        agc_dp_t den; /* LEWD/3 - ..., B0 */

        num       = agc_dp_mul_trunc(agc_dp_mul_trunc(g, es->vl), AGC_ENTRY_ONE_THIRD);
        den       = agc_dp_add(ip, es->ahookdv, AGC_ENTRY_ONE_24TH);
        den       = agc_dp_mul_trunc(agc_dp_mul_trunc(den, es->dvl), AGC_ENTRY_CH1);
        den       = agc_dp_div(ip, agc_dp_div(ip, den, es->dhook), es->vl);
        den       = agc_dp_sub(ip, agc_dp_mul_trunc(es->lewd, AGC_ENTRY_ONE_THIRD), den);
        es->vl    = agc_dp_add(ip, agc_dp_div(ip, num, den), es->vl);
        es->q7    = entry_dhookyq7(ip, es, es->vl);
        es->vbars = agc_dp_sq(es->vl);
        g         = 0;
    }

    es->gammal  = g; /* HUNTEST3    */
    m_getdhook  = agc_dp_mul_trunc(agc_dp_sub(ip, g, es->gammal1), AGC_ENTRY_HALVE);
    es->gammal1 = agc_dp_add(ip, m_getdhook, es->gammal1);
    *gammal     = g;

    return (AGC_ENTRY_HUNT_RANGER);
} /* entry_huntest_core */

/**********************************************************************
 * RANGER (pp. 859-860): range components (revs) for GAMMAL; returns DIFF =
 * ASKEP + ASP1 + ASPUP + ASPDWN + ASP3 - THETAH. Leaves OVFIND clear.
 *
 * FUNCTION NAME: entry_ranger_core
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *   gammal    agc_dp_t             I    GAMMAL, revs B0
 *   diff      agc_dp_t *           O    DIFF = predicted range - THETAH,
 *                                       revs B0
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of LOG
 *          (AGC_STATUS_BAD_ARG for an argument that is not positive)
 **********************************************************************/
static agc_status_t entry_ranger_core(agc_interp_t *ip, agc_entry_state_t *es, agc_dp_t gammal, agc_dp_t *diff)
{
    agc_dp_t e4;  /* E/4, B2 */
    agc_dp_t m;   /* MPAC, scale per step */
    agc_dp_t sum; /* range sum, revs B0 */

    es->cosg_2 = agc_dp_sub(ip, AGC_ENTRY_HALVE, agc_dp_shr(agc_dp_sq(gammal), 2));
    m          = agc_dp_mul_trunc(agc_dp_sub(ip, es->vbars, AGC_ENTRY_HALVE), es->vbars);

    /* DMP DMP SL2 DAD SQRT: the shift, add and root see the TP MPAC.     */
    e4        = agc_tp_sqrt(agc_tp_add(ip, agc_tp_shift(ip, agc_dp_mul_tp(agc_dp_mul_trunc(m, es->cosg_2), es->cosg_2), 2), agc_tp_from_dp(AGC_ENTRY_ONE_16TH)));
    m         = agc_dp_mul_trunc(agc_dp_mul_trunc(es->vbars, es->cosg_2), gammal);
    es->askep = agc_dp_shl(ip, agc_dp_asin(ip, agc_dp_div(ip, m, e4)), 1);
    es->asp1  = agc_dp_add(ip, agc_dp_mul_trunc(es->vl, AGC_ENTRY_Q3), es->q2);
    m         = agc_dp_div(ip, agc_dp_mul_trunc(agc_dp_sq(agc_dp_from_words(es->v1)), es->q7), es->vbars);
    m         = agc_dp_div(ip, m, es->a0);

    const agc_status_t st = agc_csm_log(ip, m, &m); /* LOG: -LOG/32, B5 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    es->aspup  = agc_dp_div(ip, agc_dp_mul_trunc(m, AGC_ENTRY_C12), es->gammal1);
    m          = agc_dp_mul_trunc(agc_dp_mul_trunc(AGC_ENTRY_KC3, es->rdot), es->v);
    es->aspdwn = agc_dp_div(ip, agc_dp_div(ip, m, es->a0), es->lad);
    es->asp3   = agc_dp_mul_trunc(agc_dp_sub(ip, AGC_ENTRY_Q6, gammal), AGC_ENTRY_Q5);

    /* ASPS(TM): the high words (the AGC stores the five DPs overlapping). */
    es->asps_tm[0] = agc_dp_high(es->askep);
    es->asps_tm[1] = agc_dp_high(es->asp1);
    es->asps_tm[2] = agc_dp_high(es->aspup);
    es->asps_tm[3] = agc_dp_high(es->aspdwn);
    es->asps_tm[4] = agc_dp_high(es->asp3);
    sum            = agc_dp_add(ip, agc_dp_add(ip, es->asp3, es->aspdwn), es->aspup);
    sum            = agc_dp_add(ip, agc_dp_add(ip, sum, es->asp1), es->askep);
    sum            = agc_dp_sub(ip, sum, es->thetah);
    agc_interp_clear_ovf(ip); /* BOVB TCDANZIG */
    *diff          = sum;

    return (AGC_STATUS_OK);
} /* entry_ranger_core */

/**********************************************************************
 * HUNTEST step (p. 855): HUNTEST core, then RANGER, PREFINAL or BECONSTD.
 *
 * FUNCTION NAME: step_huntest
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   p         entry_pass_t *  IO   state of this guidance pass:
 *                                  interpreter, entry erasables, MPAC and
 *                                  push-down words
 *
 * RETURNS: entry_step_t -- the step the AGC branches to next (its label),
 *          or ST_ERROR when an assertion fails
 **********************************************************************/
static entry_step_t step_huntest(entry_pass_t *p)
{
    agc_entry_hunt_result_t r;     /* HUNTEST result */
    agc_dp_t                g = 0; /* GAMMAL, revs B0 */

    r = entry_huntest_core(p->ip, p->es, &g);

    if (r == AGC_ENTRY_HUNT_PREFINAL)
    {
        return (ST_PREFINAL);
    }

    if (r == AGC_ENTRY_HUNT_CONSTD)
    {
        return (ST_BECONSTD);
    }

    p->mpac = g;

    return (ST_RANGER);
}

/**********************************************************************
 * Dispatch within the HUNT TEST and RANGE PREDICTION sections
 * (REENTRY_CONTROL.agc pp. 855-863), the predicted range at nominal L/D
 * against the desired range: steps HUNTEST .. SIDETRAK.
 *
 * FUNCTION NAME: entry_huntest_step
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   p         entry_pass_t *  IO   state of this guidance pass:
 *                                  interpreter, entry erasables, MPAC and
 *                                  push-down words
 *   s         entry_step_t    I    step to run (HUNTEST section)
 *
 * RETURNS: entry_step_t -- the step the AGC branches to next (its label),
 *          or ST_ERROR when an assertion fails
 **********************************************************************/
entry_step_t entry_huntest_step(entry_pass_t *p, entry_step_t s)
{
    switch ((int) s)
    {
        case ST_HUNTEST:
            return (step_huntest(p));
        case ST_RANGER:
            /* RANGER step (p. 860): DIFF, then UPSY (|DIFF| < 25 NM), GETDLEWD (HIND
             * set or DIFF < 0) or DCONSTD. */
        {
            agc_entry_state_t *es = p->es; /* entry erasables */

            if (entry_ranger_core(p->ip, es, p->mpac, &es->diff) != AGC_STATUS_OK)
            {
                return (ST_ERROR);
            }

            if (agc_dp_sub(p->ip, agc_dp_abs(es->diff), AGC_ENTRY_TOL_25NM) < 0)
            {
                return (ST_GOTOUPSY);
            }

            if (agc_flag_test(AGC_FLAG_HIND))
            {
                return (ST_GETDLEWD); /* AGC "GETLEWD": GETDLEWD    */
            }

            return ((es->diff >= 0) ? ST_DCONSTD : ST_GETDLEWD); /* DIFF sign */
        }
        case ST_GETDLEWD:
            /* GETDLEWD / LWDSTORE / LEWDPTR / LEWDOVFL (pp. 860-861): secant step on
             * LEWD, DLEWD = DLEWD DIFF/(DIFFOLD - DIFF). */
        {
            agc_interp_t      *ip = p->ip; /* interpreter state */
            agc_entry_state_t *es = p->es; /* entry erasables */
            agc_dp_t           lewd;       /* LEWD, B0 */

            es->dlewd = agc_dp_div(ip, agc_dp_mul_trunc(es->dlewd, es->diff), agc_dp_sub(ip, es->diffold, es->diff));
            lewd      = agc_dp_add(ip, es->dlewd, es->lewd);

            if (lewd < 0)
            { /* LEWDPTR            */
                es->dlewd = agc_dp_neg(agc_dp_shr(es->lewd, 1));
                lewd      = agc_dp_add(ip, es->dlewd, es->lewd);
            }

            if (agc_interp_test_ovf(ip))
            { /* LEWDOVFL           */
                es->lewd = AGC_ENTRY_NEARONE;

                return (ST_DCONSTD);
            }

            es->lewd = lewd;

            return (ST_SIDETRAK);
        }
        case ST_SIDETRAK:
            /* SIDETRAK (p. 861): the iteration continues below SERVICER priority while
             * the next passes go to ENDEXIT; restart group 4 at PRE-HUNT. DEVIATION:
             * the C pass runs the whole iteration in one call (bounded by
             * AGC_ENTRY_HUNTEST_MAX_ITER); PRIOCHNG lowers the priority but no
             * pre-emption point is taken. If the bound is hit, the pass ends and the
             * next pass resumes HUNTEST from the current LEWD/DLEWD. */
        {
            agc_entry_state_t *es = p->es; /* entry erasables */

            /* Group 5 restart priority one below group 4 (SIDETRAK p. 861). */
            agc_phase_set_priority(5, AGC_ENTRY_PRIO16); /* CA PRIO16 / TS PHSPRDT5 */
            agc_phase_change(000474);                    /* 4.47 PRE-HUNT      */
            agc_exec_prio_change(AGC_ENTRY_PRIO16);      /* BUSY: see above    */
            es->gotoaddr = AGC_ENTRY_GOTO_ENDEXIT;       /* ADENDEXT           */
            es->diffold  = es->diff;
            agc_flag_set(AGC_FLAG_HIND);
            es->q7       = AGC_ENTRY_Q7F;
            p->hunt_iter++;

            if (p->hunt_iter >= AGC_ENTRY_HUNTEST_MAX_ITER)
            {
                es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST;

                return (ST_ENDEXIT);
            }

            return (ST_HUNTEST);
        }
        default:
            break;
    } /* end switch ((int) s) */

    return (ST_ERROR); /* unreachable: the range was asserted */
} /* entry_huntest_step */

/**********************************************************************
 * AGC: none (HUNTEST, REENTRY_CONTROL.agc pp. 855-858, without sequencing;
 * for tests).
 *
 * FUNCTION NAME: agc_entry_huntest_predict
 *
 * ARGUMENTS:
 *   Argument  Type                       I/O  Description (scale)
 *   --------  -------------------------  ---  ---------------------------
 *   result    agc_entry_hunt_result_t *  O    where the AGC would continue
 *   gammal    agc_dp_t *                 O    GAMMAL, revs B0
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when a
 *          pointer is NULL
 **********************************************************************/
agc_status_t agc_entry_huntest_predict(agc_entry_hunt_result_t *result, agc_dp_t *gammal)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    if (!AGC_ASSERT(es->v > 0 && es->vsquare > 0 && es->lewd > 0 && es->lad > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *gammal = 0;
    *result = entry_huntest_core(entry_ip(), es, gammal);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (RANGER, REENTRY_CONTROL.agc pp. 859-861, on the state left by
 * HUNTEST; for tests).
 *
 * FUNCTION NAME: agc_entry_range_predict
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   gammal    agc_dp_t    I    GAMMAL, revs B0
 *   diff      agc_dp_t *  O    DIFF = predicted range - THETAH, revs B0
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when the
 *          HUNTEST state is not set up or LOG's argument is not positive
 **********************************************************************/
agc_status_t agc_entry_range_predict(agc_dp_t gammal, agc_dp_t *diff)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    if (!AGC_ASSERT(es->a0 > 0 && es->gammal1 != 0 && es->vbars > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = entry_ranger_core(entry_ip(), es, gammal, diff); /* RANGER */

    if (st == AGC_STATUS_OK)
    {
        es->diff = *diff;
    }

    return (st);
}
