/**********************************************************************
 * FILE NAME: entry_upcontrol.c
 *
 * PURPOSE:
 *   The UPCONTROL phase (the up-control reference trajectory and its
 *   L/D, NEGTESTS), the constant-drag controller (DCONSTD, CONSTD) and
 *   DOWNCNTL, then the ballistic KEP2 phase (KEPLER test).
 *
 * AGC SOURCE:
 *   Comanche055 REENTRY_CONTROL.agc pp. 864-869 (UPCONTRL .. DOWNCNTL,
 *   KEP2); Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   entry erasables agc_entry_state_t IO  ENTRY_LEXICON  agc_entry_state()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_entry_vrefer      VREFER table (lexicon.c)
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
#include "internal.h"

/* ======================================================================
 * UP CONTROL (pp. 864-866), MM 65
 * ==================================================================== */

/**********************************************************************
 * UPCONTRL / VLTEST (p. 864).
 *
 * FUNCTION NAME: step_upcontrl
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
static entry_step_t step_upcontrl(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */
    agc_dp_t           m;          /* MPAC, scale per step */

    if (agc_dp_sub(ip, es->d, AGC_ENTRY_C21) >= 0)
    {
        agc_flag_set(AGC_FLAG_NOSWITCH); /* suppress lat switch */
    }

    if (agc_dp_sub(ip, es->v, agc_dp_from_words(es->v1)) >= 0)
    {
        return (ST_DOWNCNTL);
    }

    if (agc_dp_sub(ip, es->d, es->q7) < 0)
    {
        return (ST_KEP);
    }

    if (es->rdot >= 0)
    {
        return (ST_UPCONTROL_DRAG_TEST);
    }

    m = agc_dp_sub(ip, agc_dp_sub(ip, es->v, es->vl), AGC_ENTRY_C18);

    return ((m < 0) ? ST_PREFINAL : ST_UPCONTROL_DRAG_TEST); /* VLTEST */
}

/**********************************************************************
 * CONT3 (pp. 864-865): VREF = FACT1 (1 - SQRT(FACT2 D + ALP)), RDOTREF =
 * LEWD (V1 - VREF) - RDHOOK, with RDHOOK = CH1 (1 + DV AHOOKDV/DVL)
 * DV^2/(DHOOK VREF), DV = VS1 - VREF.
 *
 * FUNCTION NAME: step_upcontrol_reference
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
static entry_step_t step_upcontrol_reference(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */
    agc_dp_t           m;          /* MPAC, scale per step */
    agc_dp_t           dv;         /* VS1 - VREF, /2VS B1 */

    /* DMP FACT2, DAD ALP, SQRT on the TP MPAC (as in HUNTEST1).          */
    m           = agc_tp_sqrt(agc_tp_add(ip, agc_dp_mul_tp(es->d, es->fact2), agc_tp_from_dp(es->alp)));
    m           = agc_dp_sub(ip, AGC_ENTRY_NEARONE, m);
    es->vref    = agc_dp_mul_trunc(m, es->fact1);
    es->rdotref = agc_dp_mul_trunc(agc_dp_sub(ip, agc_dp_from_words(es->v1), es->vref), es->lewd);
    dv          = agc_dp_sub(ip, es->vs1, es->vref);

    if (dv >= 0)
    {
        m           = agc_dp_div(ip, agc_dp_mul_trunc(dv, es->ahookdv), es->dvl);
        m           = agc_dp_mul_trunc(agc_dp_add(ip, m, AGC_ENTRY_ONE_16TH), AGC_ENTRY_CH1);
        m           = agc_dp_mul_trunc(agc_dp_mul_trunc(m, dv), dv);
        m           = agc_dp_div(ip, agc_dp_div(ip, m, es->dhook), es->vref);
        es->rdotref = agc_dp_sub(ip, es->rdotref, m); /* - RDHOOK          */
    }

    return (ST_UPCONTROL_Q7MIN_TEST);
}

/**********************************************************************
 * UPCNTRL3 / NEXT1 (p. 866): delta L/D = -((RDOT - RDOTREF) F1/KB1 + V -
 * VREF) F1/KB2, F1 = FACTOR, with the nonlinear gain reduction above .1,
 * L/D = LEWD + delta.
 *
 * FUNCTION NAME: step_upcontrol_ld
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
static entry_step_t step_upcontrol_ld(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */
    agc_dp_t           dld;        /* L/D change, B0 */
    agc_dp_t           m;          /* MPAC, scale per step */

    m   = agc_dp_mul_trunc(agc_dp_sub(ip, es->rdot, es->rdotref), es->factor);
    m   = agc_dp_add(ip, agc_dp_div(ip, m, AGC_ENTRY_ONE_OVER_KB1), es->v);
    m   = agc_dp_mul_trunc(agc_dp_sub(ip, m, es->vref), es->factor);
    dld = agc_dp_div(ip, m, AGC_ENTRY_MINUS_ONE_OVER_KB2); /* /16       */

    if (agc_interp_test_ovf(ip))
    {
        p->mpac = dld;

        return (ST_GOMAXLD);
    }

    m = agc_dp_sub(ip, agc_dp_abs(dld), AGC_ENTRY_PT1_16);

    if (m >= 0)
    {
        m   = agc_dp_add(ip, agc_dp_mul_trunc(m, AGC_ENTRY_POINT1), AGC_ENTRY_PT1_16);
        dld = agc_dp_sign(m, dld);
    }

    p->mpac = agc_dp_add(ip, agc_dp_shl(ip, dld, 4), es->lewd); /* NEXT1 */

    return (ST_NEGTESTS);
} /* step_upcontrol_ld */

/**********************************************************************
 * NEGTESTS (p. 866): L/D store; above C20 (175 fpss) LATSW is cleared (roll
 * over the top) and a negative L/D is replaced by 0.
 *
 * FUNCTION NAME: step_negtests
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
static entry_step_t step_negtests(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */

    if (agc_interp_test_ovf(ip))
    {
        return (ST_GOMAXLD); /* MPAC = overflowed L/D */
    }

    es->l_d = p->mpac;

    if (agc_dp_sub(ip, es->d, AGC_ENTRY_C20) >= 0)
    {
        agc_flag_clear(AGC_FLAG_LATSW);

        if (p->mpac < 0)
        {
            es->l_d = 0;
        }
    }

    return (ST_LIMITLD);
}

/* ======================================================================
 * CONSTANT DRAG (pp. 867-868)
 * ==================================================================== */

/**********************************************************************
 * DCONSTD (p. 867): the two RANGER entries into CONSTD.
 *
 * FUNCTION NAME: step_dconstd
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
static entry_step_t step_dconstd(entry_pass_t *p)
{
    p->es->diffold = p->es->diff;
    p->es->q7      = AGC_ENTRY_Q7F;

    return (ST_BECONSTD);
}

/**********************************************************************
 * DOWNCNTL (pp. 867-868): initial part of up control (V > V1): RDTR = LAD
 * (V1 - V), push = LAD/256 + K2D (RDOT - RDTR), DREF = (V/V1)^2 A0 - (V -
 * V1)^2 LAD/(2 C1 HS).
 * On the MPAC words, with V1 as HUNTEST stored it: the DSU/DSQ chain and
 * BDDV A0 DSU leave DREF in words whose signs may disagree, and CONSTD1's
 * BDSU D DMP K1D reads them so (oracle entry L/D and L/D1 exact; with
 * the values 1-9 LSB, with V1 canonical 256 LSB: follow-up-done-lanes F3).
 *
 * FUNCTION NAME: step_downcntl
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
static entry_step_t step_downcntl(entry_pass_t *p)
{
    agc_interp_t        *ip  = p->ip;                    /* interpreter state */
    agc_entry_state_t   *es  = p->es;                    /* entry erasables */
    const agc_dp_words_t lad = agc_dp_to_words(es->lad); /* LAD, B0 */
    const agc_dp_words_t v   = agc_dp_to_words(es->v);   /* V, /2VS */

    agc_interp_clear_ovf(ip); /* BOVB TCDANZIG    */

    /* DLOAD LAD SR 8D PDDL V DSU V1 DMP LAD DAD RDOT DMP K2D DAD (pop):
     * LAD/256 + K2D (RDOT - RDTR) into PD.                               */
    const agc_dp_words_t lad8 = agc_mpac_store(agc_mpac_shift(ip, agc_mpac_dload(lad), -8));    /* LAD/256 */
    agc_tp_words_t       m    = agc_mpac_dmp(agc_mpac_dsu(ip, agc_mpac_dload(v), es->v1), lad); /* (V - V1) LAD */

    m        = agc_mpac_dmp(agc_mpac_dad(ip, m, agc_dp_to_words(es->rdot)), agc_dp_to_words(AGC_ENTRY_K2D));
    p->pd_ld = agc_mpac_store(agc_mpac_dad(ip, m, lad8));

    /* PDDL V1 DSU V DSQ DMP LAD DDV 2C1HS: (V1 - V)^2 LAD/(2 C1 HS) into
     * PD; PDDL V1 DSQ DDV VSQUARE BDDV A0 DSU (pop): DREF in MPAC.        */
    m = agc_mpac_dmp(agc_mpac_dsq(agc_mpac_dsu(ip, agc_mpac_dload(es->v1), v)), lad);

    const agc_dp_words_t q = agc_mpac_store(agc_mpac_ddv(ip, m, agc_dp_to_words(AGC_ENTRY_TWO_C1_HS))); /* PD: the (V1 - V)^2 term */

    m       = agc_mpac_ddv(ip, agc_mpac_dsq(agc_mpac_dload(es->v1)), agc_dp_to_words(es->vsquare));
    p->dref = agc_mpac_store(agc_mpac_dsu(ip, agc_mpac_bddv(ip, m, agc_dp_to_words(es->a0)), q)); /* DREF */

    return (ST_CONSTANT_DRAG_LD);
}

/* ======================================================================
 * BALLISTIC PHASE (p. 869), MM 66
 * ==================================================================== */

/**********************************************************************
 * KEP2 (p. 869): final phase once D > Q7F + KDMIN; otherwise keep (or,
 * below .05 g, zero) ROLLC and ROLLHOLD and compute the trim gimbal angles
 * (P62.3 -> S62.3, then ENDEXIT).
 *
 * FUNCTION NAME: step_kepler_test
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
static entry_step_t step_kepler_test(entry_pass_t *p)
{
    agc_entry_state_t *es = p->es; /* entry erasables */

    if (agc_dp_sub(p->ip, AGC_ENTRY_Q7FKDMIN, es->d) < 0)
    {
        return (ST_PREFINAL);
    }

    if (!agc_flag_test(AGC_FLAG__05GSW))
    {
        es->rollc    = 0; /* TLOAD 3ZEROS: ROLLC, ROLLHOLD */
        es->rollhold = 0;
    }

    const agc_status_t st = agc_entry_desired_gimbal_angles(); /* status of the desired-gimbal-angle computation */

    if (st != AGC_STATUS_OK)
    {
        p->status = st;

        return (ST_ERROR);
    }

    return (ST_ENDEXIT); /* QPRET = ENDEXIT            */
}

/**********************************************************************
 * Dispatch within the UP CONTROL section (pp. 864-868), roll during the
 * super-circular phase: steps GOTOUPSY .. DOWNCNTL.
 *
 * FUNCTION NAME: entry_upcontrol_step
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   p         entry_pass_t *  IO   state of this guidance pass:
 *                                  interpreter, entry erasables, MPAC and
 *                                  push-down words
 *   s         entry_step_t    I    step to run (UPCONTROL section)
 *
 * RETURNS: entry_step_t -- the step the AGC branches to next (its label),
 *          or ST_ERROR when an assertion fails
 **********************************************************************/
entry_step_t entry_upcontrol_step(entry_pass_t *p, entry_step_t s)
{
    switch ((int) s)
    {
        case ST_GOTOUPSY:
            /* GOTOUPSY (p. 864): RTB P65 (mode, N69 display job), GOTOADDR = UPCONTRL,
             * REFAZE10 -> UPCONTRL. */
        {
            p->status = entry_upcontrol(p->es); /* still OK on entry to a step */

            return ((p->status == AGC_STATUS_OK) ? ST_UPCONTRL : ST_ERROR); /* UPCONTRL while the step is OK, else error */
        }
        case ST_UPCONTRL:
            return (step_upcontrl(p));
        case ST_UPCONTROL_DRAG_TEST:
            /* CONT1 (p. 864): L/D = LAD while D >= A0. */
        {
            if (agc_dp_sub(p->ip, p->es->d, p->es->a0) < 0)
            {
                return (ST_UPCONTROL_REFERENCE);
            }

            p->es->l_d = p->es->lad; /* STOREL/D          */

            return (ST_LIMITLD);
        }
        case ST_UPCONTROL_REFERENCE:
            return (step_upcontrol_reference(p));
        case ST_UPCONTROL_Q7MIN_TEST:
            /* CONTINU2 (p. 865): FACTOR = (D - Q7)/(A1 - Q7) when D >= Q7MIN. */
        {
            agc_interp_t      *ip = p->ip;                                /* interpreter state */
            agc_entry_state_t *es = p->es;                                /* entry erasables */
            agc_dp_t           m  = agc_dp_sub(ip, es->d, AGC_ENTRY_KA4); /* Q7MIN test, /805 B0 */

            agc_interp_clear_ovf(ip); /* BOVB TCDANZIG     */

            if (m < 0)
            {
                return (ST_UPCONTROL_LD);
            }

            es->factor = agc_dp_div(ip, agc_dp_sub(ip, es->d, es->q7), agc_dp_sub(ip, es->a1, es->q7));

            return (ST_UPCONTROL_LD);
        }
        case ST_UPCONTROL_LD:
            return (step_upcontrol_ld(p));
        case ST_NEGTESTS:
            return (step_negtests(p));
        case ST_DCONSTD:
            return (step_dconstd(p));
        case ST_BECONSTD:
            /* BECONSTD (p. 867): GOTOADDR = HUNTEST, RTB KILLGRP4. */
        {
            p->es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST;
            entry_disable_restart_group();

            return (ST_CONSTD);
        }
        case ST_CONSTD:
            /* CONSTD (p. 867): RDOTREF = -2 HS D0/V, L/D = (C/D0 LEQ + K2D (RDOT -
             * RDOTREF) + K1D (D - D0)) 2^8. */
        {
            agc_interp_t      *ip = p->ip; /* interpreter state */
            agc_entry_state_t *es = p->es; /* entry erasables */

            agc_interp_clear_ovf(ip); /* BOVB TCDANZIG    */

            /* On the MPAC words: DLOAD LEQ DMP C/D0 PDDL 2HS DMP D0 DDV V DAD
             * RDOT DMP K2D DAD (push) PDDL D0: the K2D term into PD.      */
            const agc_dp_words_t ldc = agc_mpac_store(agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(es->leq)), agc_dp_to_words(es->c_d0))); /* LEQ C/D0, PDL */
            agc_tp_words_t       m   = agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(AGC_ENTRY_TWO_HS)), agc_dp_to_words(es->d0));          /* 2HS D0 */

            m        = agc_mpac_dad(ip, agc_mpac_ddv(ip, m, agc_dp_to_words(es->v)), agc_dp_to_words(es->rdot));
            p->pd_ld = agc_mpac_store(agc_mpac_dad(ip, agc_mpac_dmp(m, agc_dp_to_words(AGC_ENTRY_K2D)), ldc));
            p->dref  = agc_dp_to_words(es->d0); /* DREF = D0         */

            return (ST_CONSTANT_DRAG_LD);
        }
        case ST_CONSTANT_DRAG_LD:
            /* CONSTD1 (p. 867): L/D = (K1D (D - DREF) + push) SL 8, then NEGTESTS. */
        {
            agc_interp_t *ip = p->ip; /* interpreter state */

            /* BDSU D DMP K1D DAD (push) SL 8D on the MPAC words: the shift
             * moves MPAC+2 bits up.                                        */
            agc_tp_words_t m = agc_mpac_bdsu(ip, agc_mpac_dload(p->dref), agc_dp_to_words(p->es->d)); /* D - DREF, B0 */

            m       = agc_mpac_dad(ip, agc_mpac_dmp(m, agc_dp_to_words(AGC_ENTRY_K1D)), p->pd_ld);
            p->mpac = agc_dp_from_words(agc_mpac_store(agc_mpac_shift(ip, m, 8)));

            return (ST_NEGTESTS);
        }
        case ST_DOWNCNTL:
            return (step_downcntl(p));
        default:
            break;
    } /* end switch ((int) s) */

    return (ST_ERROR); /* unreachable: the range was asserted */
} /* entry_upcontrol_step */

/**********************************************************************
 * Dispatch within the BALLISTIC PHASE section (p. 869), drag monitored
 * until Q7 + 0.5: steps KEP, KEP2.
 *
 * FUNCTION NAME: entry_ballistic_step
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   p         entry_pass_t *  IO   state of this guidance pass:
 *                                  interpreter, entry erasables, MPAC and
 *                                  push-down words
 *   s         entry_step_t    I    step to run (BALLISTIC section)
 *
 * RETURNS: entry_step_t -- the step the AGC branches to next (its label),
 *          or ST_ERROR when an assertion fails
 **********************************************************************/
entry_step_t entry_ballistic_step(entry_pass_t *p, entry_step_t s)
{
    switch ((int) s)
    {
        case ST_KEP:
            /* KEP (p. 869): RTB P66, GOTOADDR = KEP2. */
        {
            const agc_status_t st = entry_ballistic(p->es); /* status of the ballistic entry */

            if (st != AGC_STATUS_OK)
            {
                p->status = st;

                return (ST_ERROR);
            }

            p->es->gotoaddr = AGC_ENTRY_GOTO_KEPLER;

            return (ST_KEPLER_TEST);
        }
        case ST_KEPLER_TEST:
            return (step_kepler_test(p));
        default:
            break;
    }

    return (ST_ERROR); /* unreachable: the range was asserted */
}
