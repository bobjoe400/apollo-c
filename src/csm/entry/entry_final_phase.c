/**********************************************************************
 * FILE NAME: entry_final_phase.c
 *
 * PURPOSE:
 *   The final phase of entry (PREFINAL, PREDICT3: L/D from the stored
 *   reference trajectory, HAVDNRNG) and the steps that end every pass:
 *   the G limiter (GLIMITER, GONEGLAD, GOMAXLD, LIMITLD), the lateral
 *   logic and the roll command ROLLC.
 *
 * AGC SOURCE:
 *   Comanche055 REENTRY_CONTROL.agc pp. 870-873 (PREFINAL, PREDICT3,
 *   HAVDNRNG, GONEGLAD, GLIMITER, LIMITLD, the lateral switch, the roll
 *   command); Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   entry erasables agc_entry_state_t IO  ENTRY_LEXICON  agc_entry_state()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   entry_post_0_05g .. entry_final_phase  P64-P67 (programs.c)
 *   agc_entry_vrefer      VREFER table (lexicon.c)
 *   agc_rtb_signmpac      SIGNMPAC (RT8_OP_CODES)
 *   agc_exec_novac        NOVAC
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
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"
#include "internal.h"

/* ======================================================================
 * FINAL PHASE (pp. 870-872), MM 67
 * ==================================================================== */

/**********************************************************************
 * PREFINAL (p. 870): RTB P67, SET EGSW, GOTOADDR = PREDICT3.
 *
 * FUNCTION NAME: step_prefinal
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
static entry_step_t step_prefinal(entry_pass_t *p)
{
    p->es->gotoaddr = AGC_ENTRY_GOTO_PREFINAL; /* restart protection */

    const agc_status_t st = entry_final_phase(p->es); /* status of the final phase */

    if (st != AGC_STATUS_OK)
    {
        p->status = st;

        return (ST_ERROR);
    }

    agc_flag_set(AGC_FLAG_EGSW);
    p->es->gotoaddr = AGC_ENTRY_GOTO_FINAL_PREDICT;

    return (ST_FINAL_PREDICT);
}

/**********************************************************************
 * BACK (p. 870): search VREFER for VREF < V from the top (JJ = 12) and
 * return V - VREF (SP, positive); CCS semantics of the AGC loop kept.
 *
 * FUNCTION NAME: predict_find_jj
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *   v_sp      agc_sp_t             I    V/2VS high word, B0
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- V - VREF(JJ), positive, B0
 **********************************************************************/
static agc_sp_t predict_find_jj(agc_entry_state_t *es, agc_sp_t v_sp)
{
    int i;       /* loop count */
    int jj = 12; /* table index */

    for (i = 0; i < 2 * AGC_ENTRY_VREFER_N; i++)
    {
        int d = (int) agc_entry_vrefer(0, jj) - (int) v_sp; /* VREF - V   */

        if (d < 0)
        {
            es->jj = jj;

            return ((agc_sp_t) (-d)); /* TEM1B      */
        }

        if (d == 0 || jj == 0)
        { /* CCS A = +0: TCF BACK with A = 0; JJ = +0 cannot occur (VQUIT) */
            if (jj == 0)
            {
                es->jj = 0;

                return (1);
            }

            jj = 0;
        }
        else
        {
            jj--;
        }
    }

    es->jj = 0;

    return (1); /* unreachable: the table always brackets V */
} /* predict_find_jj */

/**********************************************************************
 * PREDICT3 (pp. 870-871): stop steering below VQUIT, else range error
 * DNRNGERR = PREDANG - THETAH (revs), or GONEGLAD once past the target.
 *
 * FUNCTION NAME: step_final_predict
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
static entry_step_t step_final_predict(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */
    agc_dp_t           m;          /* MPAC, scale per step */

    if (agc_dp_sub(ip, es->v, AGC_ENTRY_VQUIT) < 0)
    {
        return (ST_STEEROFF);
    }

    /* BACK2 (pp. 870-871): GRAD and the six interpolated functions FX+5..FX+0 =
     * F1, F2, RDOTREF, RTOGO, -AREF, F3 (SP). */
    agc_sp_t tem1b = predict_find_jj(es, agc_dp_high(es->v)); /* TEM1B: V - VREF(JJ), positive, B0 */

    int      mm;                                                                      /* table column */
    int      jj = es->jj;                                                             /* table index */
    agc_sp_t dk = (agc_sp_t) (agc_entry_vrefer(0, jj + 1) - agc_entry_vrefer(0, jj)); /* DELTA VREF, B0 */

    /* GRAD = (V - VREF)/(V(K+1) - V(K)), DV of (A=tem1b, L=0).            */
    es->grad = (agc_sp_t) (((int32_t) tem1b * 16384) / (int32_t) dk);

    for (mm = 5; mm >= 0; mm--)
    {
        int      t   = 6 - mm;                                      /* JJ + 13 (5 - mm + 1)          */
        agc_sp_t ak  = agc_entry_vrefer(t, jj);                     /* table word, B0 */
        agc_sp_t ak1 = agc_entry_vrefer(t, jj + 1);                 /* next table word, B0 */
        agc_dp_t pr  = agc_sp_mul((agc_sp_t) (ak1 - ak), es->grad); /* product, B0 */

        es->fx[mm] = (agc_sp_t) (agc_dp_high(pr) + ak); /* AK + GRAD dA */
    }

    /* PREDICT3 table part (pp. 870-871): PREDANG = RTOGO + F1 (D - DREF)
     * + F2 (RDOTREF - RDOT) 8, as SP fixed-point (MP / DAS); returns the DP
     * MPAC the AGC passes on to the interpreter. */
    agc_ovf_t ovf = 0; /* DAS overflow */
    agc_dp_t  mpac;    /* MPAC, scale per step */
    agc_dp_t  r;       /* range, revs B0 */
    agc_sp_t  a;       /* ARCCOS argument, B0 */

    a         = (agc_sp_t) (es->fx[1] + agc_dp_high(es->d));   /* D - DREF */
    es->fx[1] = 0;                                             /* XCH FX +1 */
    mpac      = agc_sp_mul(a, es->fx[5]);                      /* F1 (D-DREF) */
    r         = agc_dp_shl_raw(agc_dp_neg(es->rdot), 3, &ovf); /* DCS x8 */
    a         = (agc_sp_t) (agc_dp_high(r) + es->fx[3]);       /* + RDOTREF   */
    r         = agc_sp_mul(a, es->fx[4]);                      /* x F2        */

    /* + RTOGO                                                            */
    r           = agc_dp_add_raw(r, agc_dp_from_sp(es->fx[2]), &ovf);
    mpac        = agc_dp_add_raw(mpac, r, &ovf); /* DAS MPAC     */
    es->predang = agc_dp_high(mpac);

    m = mpac;
    m = agc_dp_sub(ip, agc_dp_shr(m, 3), es->thetah); /* SR3 DSU      */

    if (!agc_flag_test(AGC_FLAG_GONEPAST))
    {
        if (!agc_flag_test(AGC_FLAG_GONEBY))
        {
            p->mpac = m;

            return (ST_HAVDNRNG);
        }

        es->dnrngerr = AGC_ENTRY_MAXRNG; /* 9999.9 NM   */
        agc_flag_set(AGC_FLAG_GONEPAST);
    }

    return (ST_GONEGLAD);
} /* step_final_predict */

/**********************************************************************
 * HAVDNRNG (pp. 871-872): L/D = LOD + (THETAH - PREDANG)/(DR/D L/D) 2^5.
 *
 * FUNCTION NAME: step_havdnrng
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
static entry_step_t step_havdnrng(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */
    agc_dp_t           m;          /* MPAC, scale per step */

    es->dnrngerr = p->mpac;
    m            = agc_dp_neg(p->mpac); /* short if neg, long if pos */
    agc_interp_clear_ovf(ip);           /* BOVB TCDANZIG              */

    m = agc_dp_div(ip, m, agc_dp_pack(es->fx[0], es->fx[1])); /* DDV FX    */
    m = agc_dp_shl(ip, m, 5);

    if (!agc_interp_test_ovf(ip))
    {
        m = agc_dp_add(ip, m, es->lod);
    }

    if (agc_interp_test_ovf(ip))
    {
        p->mpac = m;

        return (ST_GOMAXLD);
    }

    es->l_d = m;

    return (ST_GLIMITER);
}

/**********************************************************************
 * GONEGLAD (p. 872): L/D = -LAD ("DLOAD GONEGLAD" loads any negative).
 *
 * FUNCTION NAME: step_gonegladd
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
static entry_step_t step_gonegladd(entry_pass_t *p)
{
    p->mpac = -1;

    return (ST_GOMAXLD);
}

/**********************************************************************
 * GOMAXL/D (p. 872): L/D = LAD SIGN(MPAC), then GLIMITER.
 *
 * FUNCTION NAME: step_gomaxld
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
static entry_step_t step_gomaxld(entry_pass_t *p)
{
    p->es->l_d = agc_dp_mul_trunc(agc_rtb_signmpac(p->mpac), p->es->lad);

    return (ST_GLIMITER);
}

/**********************************************************************
 * GLIMITER (p. 872): above GMAX/2 limit the downlift so that RDOT +
 * SQRT(2HS (GMAX - D)(LEQ/GMAX + LAD) + (2HS GMAX/V)^2) >= 0.
 *
 * FUNCTION NAME: step_glimiter
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
static entry_step_t step_glimiter(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */
    agc_dp_t           m;          /* MPAC, scale per step */
    agc_dp_t           x;          /* LEQ/2 + LAD, B0 */

    m = agc_dp_sub(ip, AGC_ENTRY_GMAX_HALF, es->d);

    if (m >= 0)
    {
        return (ST_LIMITLD);
    }

    m = agc_dp_add(ip, m, AGC_ENTRY_GMAX_HALF); /* GMAX - D     */

    if (m < 0)
    {
        return (ST_GOPOSLAD);
    }

    m = agc_dp_mul_trunc(m, AGC_ENTRY_TWO_HS);
    x = agc_dp_add(ip, agc_dp_mul_trunc(es->leq, AGC_ENTRY_HALVE), es->lad);
    m = agc_dp_mul_trunc(x, m);
    x = agc_dp_div(ip, AGC_ENTRY_TWO_HS_GMAX_SQ, es->vsquare);
    m = agc_dp_sqrt(agc_dp_add(ip, x, m)); /* XLIM         */

    return ((agc_dp_add(ip, m, es->rdot) >= 0) ? ST_LIMITLD : ST_GOPOSLAD); /* sign */
}

/* ======================================================================
 * LATERAL LOGIC (pp. 873-875)
 * ==================================================================== */

/**********************************************************************
 * LIMITL/D / L350 (p. 873): lateral switching corridor Y = KLAT VSQUARE +
 * LATBIAS (halved when |L/D| >= L/DCMINR).
 *
 * FUNCTION NAME: step_limitld
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
static entry_step_t step_limitld(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */

    es->l_d1 = es->l_d;

    if (agc_flag_test(AGC_FLAG_GONEPAST))
    {
        return (ST_ROLL_COMMAND); /* no lateral past target     */
    }

    p->pd_y = agc_dp_add(ip, agc_dp_mul_trunc(es->vsquare, es->klat), AGC_ENTRY_LATBIAS);

    if (agc_dp_sub(ip, agc_dp_abs(es->l_d), es->l_dcminr) >= 0)
    {
        if (agc_dp_sign(es->latang, es->k2roll) < 0)
        {
            return (ST_LATERAL_MIN_LD);
        }

        p->pd_y = agc_dp_shr(p->pd_y, 1); /* Y = Y/2                    */
    }

    return (ST_LATERAL_SWITCH);
}

/**********************************************************************
 * L355 (p. 873): ROLLC = SIGN(K2ROLL) ACOS(L/D1 / LAD); clear NOSWITCH.
 *
 * FUNCTION NAME: step_roll_command
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
static entry_step_t step_roll_command(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */
    agc_dp_t           m;          /* MPAC, B0 */

    /* MPAC set to +-1 if overflow; the AGC has no BOV here, so OVFIND is
     * left as DDV set it.                                               */
    m         = agc_dp_div(ip, es->l_d1, es->lad);
    m         = agc_dp_acos(ip, agc_dp_shr(m, 1));
    es->rollc = agc_dp_sign(m, es->k2roll);
    agc_flag_clear(AGC_FLAG_NOSWITCH);

    return (ST_ENDEXIT);
}

/**********************************************************************
 * Dispatch within the FINAL PHASE section (pp. 870-875) with the G-limiter
 * and lateral logic, then P67.1 latitude/longitude: steps PREFINAL ..
 * STEEROFF.
 *
 * FUNCTION NAME: entry_final_phase_step
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   p         entry_pass_t *  IO   state of this guidance pass:
 *                                  interpreter, entry erasables, MPAC and
 *                                  push-down words
 *   s         entry_step_t    I    step to run (FINAL PHASE section)
 *
 * RETURNS: entry_step_t -- the step the AGC branches to next (its label),
 *          or ST_ERROR when an assertion fails
 **********************************************************************/
entry_step_t entry_final_phase_step(entry_pass_t *p, entry_step_t s)
{
    switch ((int) s)
    {
        case ST_PREFINAL:
            return (step_prefinal(p));
        case ST_FINAL_PREDICT:
            return (step_final_predict(p));
        case ST_HAVDNRNG:
            return (step_havdnrng(p));
        case ST_GONEGLAD:
            return (step_gonegladd(p));
        case ST_GOMAXLD:
            return (step_gomaxld(p));
        case ST_GLIMITER:
            return (step_glimiter(p));
        case ST_GOPOSLAD:
            /* GOPOSLAD (p. 872): L/D = LAD. */
        {
            p->es->l_d = p->es->lad;

            return (ST_LIMITLD);
        }
        case ST_LIMITLD:
            return (step_limitld(p));
        case ST_LATERAL_SWITCH:
            /* L353 (p. 873): switch K2ROLL when LATANG SIGN(K2ROLL) - Y >= 0, unless
             * NOSWITCH (BONCLR: tested and cleared). */
        {
            agc_entry_state_t *es = p->es; /* entry erasables */
            agc_dp_t           m;          /* MPAC, B0 */

            m = agc_dp_sub(p->ip, agc_dp_sign(es->latang, es->k2roll), p->pd_y);

            if (m < 0)
            {
                return (ST_ROLL_COMMAND);
            }

            if (agc_flag_test(AGC_FLAG_NOSWITCH))
            {
                agc_flag_clear(AGC_FLAG_NOSWITCH);

                return (ST_ROLL_COMMAND);
            }

            es->k2roll = agc_dp_neg(es->k2roll); /* K2ROLL = -K2ROLL           */

            return (ST_ROLL_COMMAND);
        }
        case ST_ROLL_COMMAND:
            return (step_roll_command(p));
        case ST_LATERAL_MIN_LD:
            /* L357 (p. 875): L/D1 = L/DCMINR SIGN(L/D). */
        {
            p->es->l_d1 = agc_dp_sign(p->es->l_dcminr, p->es->l_d);

            return (ST_ROLL_COMMAND);
        }
        case ST_STEEROFF:
            /* STEEROFF (p. 875): V < VQUIT. Start P67.1 (N67 flashing), restart 4.41
             * and 5.3, GOTOADDR = P67.2 and continue there. */
        {
            agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_REESTABLISH_ENTRY_SEQUENCER, AGC_TASK_NONE, 0, false, 0U };

            p->status = agc_exec_novac(AGC_ENTRY_PRIO16, AGC_JOB_ENTRY_FINAL_PHASE_DISPLAY); /* BAILOUT 1202 when no core set */

            if (p->status != AGC_STATUS_OK)
            {
                return (ST_ERROR);
            }

            agc_phase_change2(000414, 010035, &v); /* 4.41 P67.1, SERVICER 5.3 */
            p->es->gotoaddr = AGC_ENTRY_GOTO_FINAL_PHASE_LATLONG;

            return (ST_FINAL_PHASE_LATLONG);
        }
        case ST_FINAL_PHASE_LATLONG:
            /* P67.2 (P61-P67.agc p. 802): present LAT, LONG, ALT (pad radius, earth),
             * then P67.3: SERVNOUT without display. */
        {
            p->status = agc_entry_final_phase_latlong();

            return ((p->status == AGC_STATUS_OK) ? ST_SERVNOUT : ST_ERROR); /* done */
        }
        default:
            break;
    } /* end switch ((int) s) */

    return (ST_ERROR); /* unreachable: the range was asserted */
} /* entry_final_phase_step */
