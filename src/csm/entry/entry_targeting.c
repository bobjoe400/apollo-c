/**********************************************************************
 * FILE NAME: entry_targeting.c
 *
 * PURPOSE:
 *   Entry targeting: STARTENT / STARTEN1 (the first pass: target vector
 *   by LALOTORV, EARROT1) and TARGETNG (every pass: the average-G data
 *   scaled, the target vector rotated by EARROT2, range to go THETAH and
 *   the crossrange, the L/D the lateral logic needs).
 *
 * AGC SOURCE:
 *   Comanche055 REENTRY_CONTROL.agc pp. 844-852 (STARTENT, STARTEN1,
 *   TARGETNG); EARROT1/2 are CSM_GEOMETRY's (agc_csm_earrot1/2);
 *   Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   entry erasables agc_entry_state_t IO  ENTRY_LEXICON  agc_entry_state()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_latlong_to_r      LALOTORV (STARTEN1 target vector)
 *   agc_csm_earrot1/2     EARROT1, EARROT2 (CSM_GEOMETRY)
 *   agc_rtb_signmpac      SIGNMPAC (RT8_OP_CODES): K2ROLL
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
#include "agc/guidance/guidance.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "internal.h"

/**********************************************************************
 * EARROT1 / EARROT2 (CSM_GEOMETRY.agc pp. 293-294) on the entry erasables:
 * the routines themselves are agc_csm_earrot1 / agc_csm_earrot2 (the
 * alignment lane's CSM_GEOMETRY); this passes RTINIT, UNITW, RTEAST,
 * RTNORM, DTEAROT in and takes RTEAST, RTNORM, DTEAROT (OVERADAY) and RT
 * back. first = true is the EARROT1 entry (resolves RTEAST, RTNORM).
 *
 * FUNCTION NAME: entry_earrot
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *   first     bool                 I    true: EARROT1 (forms RTEAST,
 *                                       RTNORM); false: EARROT2
 *
 * RETURNS: none
 **********************************************************************/
void entry_earrot(agc_interp_t *ip, agc_entry_state_t *es, bool first)
{
    agc_csm_earrot_t e; /* the CSM_GEOMETRY frame */

    e.rtinit  = es->rtinit;
    e.unitw   = agc_guid_state()->avg.unitw; /* UNITW polar axis, B0 */
    e.rteast  = es->rteast;
    e.rtnorm  = es->rtnorm;
    e.dtearot = es->dtearot;
    e.rt      = es->rt;

    if (first)
    {
        agc_csm_earrot1(ip, &e);
    }
    else
    {
        agc_csm_earrot2(ip, &e);
    }

    es->rteast  = e.rteast;
    es->rtnorm  = e.rtnorm;
    es->dtearot = e.dtearot; /* OVERADAY */
    es->rt      = e.rt;
}

/* ======================================================================
 * TARGETING (pp. 848-852)
 * ==================================================================== */

/**********************************************************************
 * GETETA / SUBETA / SUBETA2 (p. 851): D predicted half a cycle ahead, D = D
 * + D (-RDOT/HS - 2D/V) DT/2, then the time-of-flight estimate to the
 * target in MPAC (cs B28).
 *
 * FUNCTION NAME: targ_geteta
 *
 * On the MPAC words: RDOT and D as DSTORE stored them; the DAD of the
 * pushed -RDOT/HS and the DMP D DAD D leave words that may disagree in
 * sign, and the D they store is exact only this way (oracle entry D
 * 38925744 / 38925745 with the values, follow-up-done-lanes F3).
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *   rdot      agc_dp_words_t       I    RDOT as stored, /2VS
 *   d         agc_dp_words_t       I    D as stored by DSTORE, /805 B0
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- time-of-flight estimate to the target,
 *          cs B28
 **********************************************************************/
static agc_dp_t targ_geteta(agc_interp_t *ip, agc_entry_state_t *es, agc_dp_words_t rdot, agc_dp_words_t d)
{
    /* DLOAD RDOT DDV -HSCALED PDDL (push) D DMP -KSCALE DDV V DAD (pop)
     * DMP D DAD D STORE D.                                               */
    const agc_dp_words_t t = agc_mpac_store(agc_mpac_ddv(ip, agc_mpac_dload(rdot), agc_dp_to_words(AGC_ENTRY_MINUS_HSCALED)));                   /* -RDOT/HS, B0 */
    agc_tp_words_t       m = agc_mpac_ddv(ip, agc_mpac_dmp(agc_mpac_dload(d), agc_dp_to_words(AGC_ENTRY_MINUS_KSCALE)), agc_dp_to_words(es->v)); /* -2D/V term, B0 */

    m     = agc_mpac_dad(ip, m, t);
    es->d = agc_dp_from_words(agc_mpac_store(agc_mpac_dad(ip, agc_mpac_dmp(m, d), d))); /* STORE D */

    if (!agc_flag_test(AGC_FLAG_EGSW))
    {
        return (agc_dp_mul_trunc(es->thetah, AGC_ENTRY_KTETA)); /* DMP KTETA */
    }

    if (agc_dp_sub(ip, es->v, AGC_ENTRY_FOURTH) < 0)
    { /* SUBETA: VMIN */
        agc_flag_set(AGC_FLAG_RELVELSW); /* SET RELVELSW */
    }

    return (agc_dp_div(ip, agc_dp_mul_trunc(es->thetah, AGC_ENTRY_KT1), es->v));
}

/**********************************************************************
 * L/DCALC (p. 850): measured lift-to-drag from DELVREF and UXA, L/D =
 * SQRT((|DV|^2 - (DV.UXA)^2) / (DV.UXA)^2), skipped on overflow (OVFIND
 * last cleared in EARROT2, as in the AGC).
 *
 * FUNCTION NAME: targ_ldcalc
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: none
 **********************************************************************/
static void targ_ldcalc(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_vec_t dv = agc_guid_state()->avg.delvref; /* DELVREF, pulses (scale 2^14) */
    agc_dp_t  du;                                 /* DOT SL1: DV.UXA, B1   */
    agc_dp_t  dsq;                                /* DSQ: (DV.UXA)^2, B2   */
    agc_dp_t  r;                                  /* (L/D)^2, B0           */

    du  = entry_tp_shl(ip, agc_vec_dot_tp(ip, dv, es->uxa), 1);
    dsq = agc_dp_sq(du);

    r = agc_dp_div(ip, agc_dp_sub(ip, agc_vec_sq(ip, dv), dsq), dsq);

    if (!agc_interp_test_ovf(ip))
    {
        es->l_dcalc = agc_dp_sqrt(r);
    }
}

/**********************************************************************
 * TARGETNG (pp. 848-850): called each pass by SCALEPOP (and by P61).
 * Updates the velocity data, D, the rotated target RT, LATANG, GONEBY,
 * THETAH, RTGON67, .05GSW and L/DCALC.
 *
 * FUNCTION NAME: agc_entry_targeting
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_entry_targeting(void)
{
    agc_interp_t      *ip    = entry_ip();                  /* entry job interpreter state */
    agc_entry_state_t *es    = agc_entry_state();           /* entry erasables */
    agc_vec_t          unitr = agc_guid_state()->avg.unitr; /* UNITR, B1 */
    agc_state_vector_t sv;                                  /* state vector (PIPTIME, V) */
    agc_dp_t           mpac  = 0;                           /* ETA estimate, cs B28 */

    agc_sv_get(AGC_SV_THIS, &sv);

    /* GETVEL .. DSTORE  */
    /* GETVEL / GETUNITV / DSTORE (p. 848): velocity (inertial, or relative once
     * RELVELSW is set), VSQUARE, LEQ, V, RDOT, D and UNI. */
    agc_vec_t      unitr_velocity = agc_guid_state()->avg.unitr; /* UNITR, B1        */
    agc_dp_t       abval          = 0;                           /* |V|/2VS, B1      */
    agc_dp_t       vsq            = 0;                           /* VSQ/4VS^2, B2    */
    agc_dp_words_t rdot_w;                                       /* RDOT as stored, /2VS */
    agc_dp_words_t d_w;                                          /* D as stored, /805 B0 */

    if (agc_flag_test(AGC_FLAG_RELVELSW))
    {
        es->vel = agc_vec_neg(es->mvrel); /* -VREL left by CM/POSE */
    }
    else
    {
        es->vel = agc_vec_scale(ip, sv.v, AGC_ENTRY_KVSCALE); /* V/2VS   */
    }

    es->unitv   = agc_vec_unit(ip, es->vel, &abval, &vsq);                                                                                                                       /* GETUNITV  */
    es->vsquare = vsq;                                                                                                                                                           /* VSQ/4, 34D */
    es->leq     = agc_dp_sub(ip, vsq, AGC_ENTRY_FOURTH);                                                                                                                         /* LEQ/4      */
    es->v       = abval;                                                                                                                                                         /* V/2VS, 36D */
    rdot_w      = agc_mpac_store(agc_mpac_shift(ip, agc_mpac_dot(ip, es->vel, unitr_velocity), 1));                                                                              /* DOT SL1 on the MPAC words */
    es->rdot    = agc_dp_from_words(rdot_w);
    d_w         = agc_mpac_store(agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(agc_vec_abval(ip, agc_erase_delv()))), agc_dp_to_words(AGC_ENTRY_KASCALE)), 1)); /* ABVAL DMP SL1 */
    es->d       = agc_dp_from_words(d_w);

    if (es->d == 0)
    {
        es->d = AGC_ENTRY_ONEBIT_DP; /* SETMIND          */
        d_w   = agc_dp_to_words(es->d);
    }

    es->uni = agc_vec_unit(ip, agc_vec_cross(ip, es->vel, unitr_velocity), NULL, NULL);

    if (!agc_flag_test(AGC_FLAG_RELVELSW))
    {
        mpac = targ_geteta(ip, es, rdot_w, d_w); /* else 3ZEROS          */
    }

    /* UPDATERT: DTEAROT = estimate + (PIPTIME - TIME/RTO)                 */
    es->dtearot = agc_dp_add(ip, agc_dp_sub(ip, mpac, es->time_rto), sv.t);
    entry_earrot(ip, es, false);
    es->latang  = agc_vec_dot(ip, es->rt, es->uni); /* LATANG/4 */
    agc_flag_clear(AGC_FLAG_GONEBY);

    if (agc_vec_dot(ip, agc_vec_cross(ip, es->rt, unitr), es->uni) < 0)
    {
        agc_flag_set(AGC_FLAG_GONEBY); /* have gone past target  */
    }

    /* GETANGLE / TINYTHET / THETDONE (pp. 849, 852): THETAH = ARCCOS(RT.UNITR)
     * in revs; near zero the arccos is replaced by 32 sqrt(2 (1/4 - x)) KACOS.
     * RTGON67 = -THETAH before the target is passed, +THETAH after. */
    agc_dp_t x = agc_vec_dot(ip, es->rt, agc_guid_state()->avg.unitr); /* RT.UNITR, B2 */
    agc_dp_t m = agc_dp_sub(ip, x, AGC_ENTRY_NEAR1_4);                 /* x - NEAR1/4, B2 */

    if (m >= 0)
    {
        /* TINYTHET: MPAC - 1BITDP+1 = x - 1/4; ABS; SL 13D; SQRT; x KACOS */
        m          = agc_dp_abs(agc_dp_sub(ip, m, AGC_ENTRY_ONEBIT_DP_P1));
        m          = agc_dp_sqrt(agc_dp_shl(ip, m, 13));
        es->thetah = agc_dp_mul_trunc(m, AGC_ENTRY_KACOS);
    }
    else
    {
        m          = agc_dp_shl(ip, agc_dp_add(ip, m, AGC_ENTRY_NEAR1_4), 1);
        es->thetah = agc_dp_acos(ip, m); /* THETAH/360 */
    }

    es->range_to_target = agc_flag_test(AGC_FLAG_GONEBY) ? es->thetah : agc_dp_neg(es->thetah); /* past: + */

    if (agc_dp_sub(ip, es->d, AGC_ENTRY_G05) < 0)
    {
        agc_flag_clear(AGC_FLAG__05GSW); /* NO.05G (for the DAP)   */
    }
    else
    {
        agc_flag_set(AGC_FLAG__05GSW);
        targ_ldcalc(ip, es);
    }
} /* agc_entry_targeting */

/* ======================================================================
 * INITIALISATION (pp. 844-846)
 * ==================================================================== */

/**********************************************************************
 * STARTEN1 (pp. 845-846): initial target vector RTINIT at PIPTIME from
 * LAT(SPL), LNG(SPL) (pad radius, earth), the first RT 500 s ahead, and
 * THETAH. Called by STARTENT and by P61.
 *
 * FUNCTION NAME: entry_initial_target
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t entry_initial_target(agc_interp_t *ip, agc_entry_state_t *es)
{
    const agc_latlong_opt_t opt = {false, false}; /* CLEAR LUNAFLAG, ERADFLAG */
    agc_navaux_padload_t    pad;                  /* pad loads (earth radius) */
    agc_latlong_t           ll  = {0, 0, 0, 0};
    agc_state_vector_t      sv;                   /* state vector (PIPTIME) */
    agc_vec_t               r;                    /* LALOTORV position, m B29 */
    agc_status_t            st;                   /* status */

    agc_sv_get(AGC_SV_THIS, &sv);
    agc_navaux_get_padload(&pad);

    agc_flag_clear(AGC_FLAG_ERADFLAG);                                   /* use pad radius        */
    agc_flag_clear(AGC_FLAG_LUNAFLAG);                                   /* earth lat-long        */
    ll.lat       = es->lat_spl;
    ll.lon       = es->lng_spl;                                          /* ALT = 0 (3ZEROS)      */
    es->time_rto = sv.t;                                                 /* TIME/RTO = PIPTIME    */
    st           = agc_latlong_to_r(ip, &pad, opt, &ll, sv.t, &r, NULL); /* R */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    es->rtinit  = agc_vec_unit(ip, r, NULL, NULL); /* .5 unit target  */
    es->dtearot = AGC_ENTRY_SEC500;                /* 500SEC nominal  */
    entry_earrot(ip, es, true);                    /* EARROT1 -> RT   */
    es->thetah  = agc_dp_acos(ip, entry_tp_shl(ip, agc_vec_dot_tp(ip, es->rt, agc_guid_state()->avg.unitr), 1));

    return (AGC_STATUS_OK);
} /* entry_initial_target */

/**********************************************************************
 * STARTENT (pp. 844-845), MM 63: first pass of entry guidance, entered from
 * CM/POSE. Ends with SERVNOUT (no display on the first pass); the caller
 * (CM/POSE) does the SERVNOUT.
 *
 * FUNCTION NAME: agc_entry_startent
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_entry_startent(void)
{
    agc_interp_t      *ip = entry_ip();        /* entry job interpreter state */
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */
    agc_state_vector_t sv;                     /* state vector (PIPTIME) */
    agc_status_t       st;                     /* STARTEN1 status */
    agc_dp_t           m;                      /* MPAC, scale per step */

    agc_sv_get(AGC_SV_THIS, &sv);

    /* STARTENT entry-switch initialisation (p. 844): clear ENTMASK =
     * 011774 (ENTRYDSP GONEPAST RELVELSW EGSW NOSWITCH HIND INRLSW LATSW
     * .05GSW), then set ENTRYSW = 011010 (ENTRYDSP, GONEPAST, LATSW).
     * INHINT'ed in the AGC.                                              */
    static const agc_flag_id_t clr[] = { AGC_FLAG_ENTRYDSP, AGC_FLAG_GONEPAST, AGC_FLAG_RELVELSW, AGC_FLAG_EGSW, AGC_FLAG_NOSWITCH, AGC_FLAG_HIND, AGC_FLAG_INRLSW, AGC_FLAG_LATSW, AGC_FLAG__05GSW };
    unsigned int               i; /* loop index */

    agc_inhint();

    for (i = 0; i < (sizeof (clr) / sizeof (clr[0])); i++)
    {
        agc_flag_clear(clr[i]); /* flag API cannot fail on valid ids */
    }

    agc_flag_set(AGC_FLAG_ENTRYDSP);
    agc_flag_set(AGC_FLAG_GONEPAST);
    agc_flag_set(AGC_FLAG_LATSW);
    agc_relint();

    es->lod      = agc_dp_from_sp(es->lodpad); /* SLOAD LODPAD */
    es->lad      = agc_dp_from_sp(es->ladpad);
    es->l_dcminr = agc_dp_mul_trunc(es->lad, AGC_ENTRY_COS15);
    es->klat     = agc_dp_shr(agc_dp_mul_trunc(AGC_ENTRY_ONE_12TH, es->lad), 1);
    es->q7       = AGC_ENTRY_Q7F;
    es->factor   = AGC_ENTRY_NEARONE;

    /* L/D = -LAD SGN(HEADSUP)                                             */
    es->l_d = agc_dp_neg(agc_dp_sign(es->lad, agc_dp_from_sp(es->headsup)));
    st      = entry_initial_target(ip, es);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    m            = agc_vec_dot(ip, agc_vec_unit(ip, agc_vec_cross(ip, sv.v, agc_guid_state()->avg.unitr), NULL, NULL), es->rt);
    es->latang   = m;                               /* UNI.RT /4    */
    es->k2roll   = agc_rtb_signmpac(agc_dp_neg(m)); /* -SGN(LATANG) */
    es->q2       = agc_dp_add(ip, agc_dp_mul_trunc(es->lad, AGC_ENTRY_Q21), AGC_ENTRY_Q22);
    es->gotoaddr = AGC_ENTRY_GOTO_INITROLL;
    es->posexit  = AGC_ENTRY_POSEXIT_SCALEPOP;

    return (AGC_STATUS_OK); /* RTB SERVNOUT */
} /* agc_entry_startent */
