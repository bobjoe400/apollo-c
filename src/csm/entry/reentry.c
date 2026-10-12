/**********************************************************************
 * FILE NAME: reentry.c
 *
 * PURPOSE:
 *   CM entry guidance ("Colossus 2A" entry steering of GSOP AS-278
 *   section 5.6). One call per 2 s SERVICER cycle (via CM/POSE ->
 *   POSEXIT) computes the roll command ROLLC: TARGETNG, then the phase
 *   named by GOTOADDR (INITIAL ROLL, HUNTEST/RANGER, UPCONTROL, KEPLER,
 *   PREDICT3 final phase), the lateral logic and the roll command, and
 *   the entry display exit. Also STARTENT/STARTEN1, EARROT1/2 and LOG of
 *   CSM_GEOMETRY used by entry.
 *
 * AGC SOURCE:
 *   Comanche055 REENTRY_CONTROL.agc pp. 844-881 (STARTENT .. ENDEXIT)
 *   Comanche055 CSM_GEOMETRY.agc pp. 291-294 (LOG, EARROT1, EARROT2)
 *   Comanche055 RT8_OP_CODES.agc p. 1386 (SIGNMPAC)
 *   Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   V, PIPTIME    state vector I  m/cs B7, cs    SERVICER state
 *   UNITR, UNITW, DELVREF    I    B1, B0, pulses average-G data
 *   DELV          agc_vec_t  I    pulses         PIPA increments
 *   CM/FLAGS      flags      IO   -              entry switches
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_latlong_to_r      LALOTORV (STARTEN1 target vector)
 *   agc_entry_vrefer      VREFER table (lexicon.c)
 *   entry_post_0_05g .. entry_final_phase  P64-P67 (programs.c)
 *   agc_dsky_display      OVERNOUT entry display
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none of its own; a failing called routine ends the pass (ST_ERROR)
 *   and its status is returned.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only; runs in the SERVICER job. The step chain is bounded
 *   (rule 2); HUNTEST iterations are bounded per pass as in the AGC
 *   (SIDETRAK after the limit).
 *
 * NOTES:
 *   The AGC chains interpretive sections with GOTO; here each labelled
 *   section is a step function returning the next step, and
 *   entry_sequencer() runs the chain. Values the AGC carries in MPAC or in
 *   the push-down list between labelled sections travel in entry_pass_t.
 *   Scales: /2VS, /805, revs (agc/csm/entry.h, entry_lexicon.h). Every
 *   DMP is truncating (the listing has no DMPR); DOT/DMP followed by SL,
 *   SQRT or ACOS keeps the TP MPAC (review-guid-entry F-E1).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"

/* Interpreter state shared by the entry interpretive code (OVFIND).        */
static agc_interp_t entry_interp;               /* entry interpreter state */
static bool         entry_interp_ready = false; /* entry_interp initialised */

typedef enum entry_step
{
    ST_INITROLL = 0,
    ST_D0EQ,
    ST_INROLOUT,
    ST_KATEST,
    ST_HUNT_TRIGGER,
    ST_HUNTEST,
    ST_RANGER,
    ST_GETDLEWD,
    ST_SIDETRAK,
    ST_GOTOUPSY,
    ST_UPCONTRL,
    ST_UPCONTROL_DRAG_TEST,
    ST_UPCONTROL_REFERENCE,
    ST_UPCONTROL_Q7MIN_TEST,
    ST_UPCONTROL_LD,
    ST_NEGTESTS,
    ST_DCONSTD,
    ST_BECONSTD,
    ST_CONSTD,
    ST_CONSTANT_DRAG_LD,
    ST_DOWNCNTL,
    ST_KEP,
    ST_KEPLER_TEST,
    ST_PREFINAL,
    ST_FINAL_PREDICT,
    ST_HAVDNRNG,
    ST_GONEGLAD,
    ST_GOMAXLD,
    ST_GLIMITER,
    ST_GOPOSLAD,
    ST_LIMITLD,
    ST_LATERAL_SWITCH,
    ST_ROLL_COMMAND,
    ST_LATERAL_MIN_LD,
    ST_STEEROFF,
    ST_FINAL_PHASE_LATLONG,
    ST_ENDEXIT,  /* terminal: ENDEXIT -> OVERNOUT display -> SERVNOUT   */
    ST_SERVNOUT, /* terminal: SERVNOUT without display                  */
    ST_ERROR     /* terminal: a called routine failed                  */
} entry_step_t;

typedef struct entry_pass
{
    agc_interp_t      *ip;        /* entry interpreter state (OVFIND)      */
    agc_entry_state_t *es;        /* entry erasables                       */
    agc_dp_t           mpac;      /* "in MPAC" at a step entry, step scale */
    agc_dp_t           pd_y;      /* push-down: lateral Y (L350), B0       */
    agc_dp_t           pd_ld;     /* push-down: L/D (NEGTESTS), the K2D term (CONSTD1), B0 */
    int                hunt_iter; /* HUNTEST iterations this pass          */
    agc_status_t       status;    /* first failure of a called routine     */
} entry_pass_t;

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
 * SIGNMPAC (RT8_OP_CODES.agc p. 1386): MPAC = +DPOSMAX if the high word of
 * MPAC is +0 or positive, else -DPOSMAX. DEVIATION: tested on the sign of
 * the whole DP value (no -0 / mixed-sign DP words in this model).
 *
 * FUNCTION NAME: entry_signmpac
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   x         agc_dp_t  I    MPAC, any scale
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- +DPOSMAX or -DPOSMAX, B0
 **********************************************************************/
agc_dp_t entry_signmpac(agc_dp_t x)
{
    return ((x >= 0) ? AGC_ENTRY_DPOSMAX : (agc_dp_t) (-AGC_ENTRY_DPOSMAX)); /* +0 or positive */
}

/**********************************************************************
 * LOG (CSM_GEOMETRY.agc p. 291): returns -ln(x)/32 for 0 < x < 1. x is
 * normalised to [.5, 1) by n left shifts; with y = NEARLY1 - xn the cubic
 * series gives -ln(xn)/32, and n ln(2)/32 is added (SHORTMP).
 *
 * FUNCTION NAME: entry_log
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *   x         agc_dp_t        I    argument, 0 < x < 1, B0
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- -ln(x)/32, B5
 **********************************************************************/
agc_dp_t entry_log(agc_interp_t *ip, agc_dp_t x)
{
    int      shift = 0; /* NORM shift count */
    agc_dp_t xn;        /* x normalised to [.5, 1), B0 */
    agc_dp_t y;         /* NEARLY1 - xn, B0 */
    agc_dp_t p;         /* POLY result, B5 */

    /* POLY coefficients a0..a3 (CSM_GEOMETRY p. 291)                     */
    static const agc_dp_t log_coef[4] = { 0, AGC_ENTRY_LOG_C1, AGC_ENTRY_LOG_C2, AGC_ENTRY_LOG_C3 };

    xn = agc_dp_norm(x, &shift);

    y = agc_dp_sub(ip, AGC_ENTRY_NEARONE, xn); /* BDSU NEARLY1      */
    p = agc_dp_poly(y, log_coef, 3);           /* TC POLY, DEC 2 (N-1)  */

    /* + shift count x CLOG2/32 (SHORTMP of a small positive integer)      */
    return (agc_dp_add(ip, p, (agc_dp_t) (AGC_ENTRY_CLOG2_32 * shift)));
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
 * EARROT1 / EARROT2 (CSM_GEOMETRY.agc pp. 293-294): rotate RTINIT about the
 * polar axis UNITW through WIE DTEAROT: RT = UNIT(RTINIT + RTNORM (cos WT -
 * 1) + RTEAST sin WT). first = true is the EARROT1 entry (resolves RTEAST,
 * RTNORM).
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
    agc_vec_t unitw = agc_guid_state()->avg.unitw; /* UNITW polar axis, B0 */
    agc_dp_t  wt;                                  /* WIE DTEAROT, revs B0 */
    agc_vec_t a;                                   /* RTNORM (COS WT - .5), B1 */
    agc_vec_t b;                                   /* RTEAST SIN WT, B1 */
    int       days;                                /* OVERADAY count */

    if (first)
    {
        es->rteast = agc_vec_cross(ip, unitw, es->rtinit);
        es->rtnorm = agc_vec_cross(ip, es->rteast, unitw);
    }

    agc_interp_clear_ovf(ip);                                 /* BOVB TCDANZIG  */
    wt = agc_dp_div(ip, es->dtearot, AGC_ENTRY_ONE_OVER_WIE); /* revs  */

    /* OVERADAY: remove whole sidereal days until the divide fits.        */
    for (days = 0; days < 32 && agc_interp_test_ovf(ip); days++)
    {
        agc_dp_t day = agc_dp_sign(AGC_ENTRY_ONE_OVER_WIE, es->dtearot); /* 1/WIE with DTEAROT's sign, cs B28 */

        es->dtearot = agc_dp_sub(ip, es->dtearot, day);
        wt          = agc_dp_div(ip, es->dtearot, AGC_ENTRY_ONE_OVER_WIE);
    }

    a      = agc_vec_scale(ip, es->rtnorm, agc_dp_sub(ip, agc_dp_cos(wt), AGC_ENTRY_HALVE));
    b      = agc_vec_scale(ip, es->rteast, agc_dp_sin(wt));
    a      = agc_vec_shl(ip, agc_vec_add(ip, b, a), 1); /* VAD VSL1        */
    es->rt = agc_vec_unit(ip, agc_vec_add(ip, a, es->rtinit), NULL, NULL);
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
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- time-of-flight estimate to the target,
 *          cs B28
 **********************************************************************/
static agc_dp_t targ_geteta(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_dp_t t; /* -RDOT/HS, B0 */
    agc_dp_t u; /* D term, B0 */

    t     = agc_dp_div(ip, es->rdot, AGC_ENTRY_MINUS_HSCALED); /* -RDOT/HS */
    u     = agc_dp_div(ip, agc_dp_mul_trunc(es->d, AGC_ENTRY_MINUS_KSCALE), es->v);
    u     = agc_dp_add(ip, u, t);
    es->d = agc_dp_add(ip, agc_dp_mul_trunc(u, es->d), es->d);

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
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_entry_targeting(void)
{
    agc_interp_t      *ip    = entry_ip();                  /* entry job interpreter state */
    agc_entry_state_t *es    = agc_entry_state();           /* entry erasables */
    agc_vec_t          unitr = agc_guid_state()->avg.unitr; /* UNITR, B1 */
    agc_state_vector_t sv;                                  /* state vector (PIPTIME, V) */
    agc_dp_t           mpac  = 0;                           /* ETA estimate, cs B28 */

    if (!AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_sv_get(AGC_SV_THIS, &sv);

    /* GETVEL .. DSTORE  */
    /**********************************************************************
     * GETVEL / GETUNITV / DSTORE (p. 848): velocity (inertial, or relative once
     * RELVELSW is set), VSQUARE, LEQ, V, RDOT, D and UNI.
     *
     * FUNCTION NAME: targ_velocity
     *
     * ARGUMENTS:
     *   Argument  Type                        I/O  Description (scale)
     *   --------  --------------------------  ---  --------------------------
     *   ip        agc_interp_t *              IO   interpreter state of the
     *                                              job: OVFIND, DVSIGN, alarms
     *   es        agc_entry_state_t *         IO   entry erasables
     *                                              (ENTRY_LEXICON names)
     *   sv        const agc_state_vector_t *  I    state vector this vehicle
     *                                              (V, m/cs B7)
     *
     * RETURNS: none
     **********************************************************************/
    agc_vec_t unitr_velocity = agc_guid_state()->avg.unitr; /* UNITR, B1        */
    agc_dp_t  abval          = 0;                           /* |V|/2VS, B1      */
    agc_dp_t  vsq            = 0;                           /* VSQ/4VS^2, B2    */
    agc_dp_t  dmag;                                         /* D/805, B0        */

    if (agc_flag_test(AGC_FLAG_RELVELSW))
    {
        es->vel = agc_vec_neg(es->mvrel); /* -VREL left by CM/POSE */
    }
    else
    {
        es->vel = agc_vec_scale(ip, sv.v, AGC_ENTRY_KVSCALE); /* V/2VS   */
    }

    es->unitv   = agc_vec_unit(ip, es->vel, &abval, &vsq);                                                    /* GETUNITV  */
    es->vsquare = vsq;                                                                                        /* VSQ/4, 34D */
    es->leq     = agc_dp_sub(ip, vsq, AGC_ENTRY_FOURTH);                                                      /* LEQ/4      */
    es->v       = abval;                                                                                      /* V/2VS, 36D */
    es->rdot    = entry_tp_shl(ip, agc_vec_dot_tp(ip, es->vel, unitr_velocity), 1);
    dmag        = entry_tp_shl(ip, agc_dp_mul_tp(agc_vec_abval(ip, agc_erase_delv()), AGC_ENTRY_KASCALE), 1); /* ABVAL DMP SL1  */
    es->d       = dmag;

    if (dmag == 0)
    {
        es->d = AGC_ENTRY_ONEBIT_DP; /* SETMIND          */
    }

    es->uni = agc_vec_unit(ip, agc_vec_cross(ip, es->vel, unitr_velocity), NULL, NULL);

    if (!agc_flag_test(AGC_FLAG_RELVELSW))
    {
        mpac = targ_geteta(ip, es); /* else 3ZEROS          */
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

    /**********************************************************************
     * GETANGLE / TINYTHET / THETDONE (pp. 849, 852): THETAH = ARCCOS(RT.UNITR)
     * in revs; near zero the arccos is replaced by 32 sqrt(2 (1/4 - x)) KACOS.
     * RTGON67 = -THETAH before the target is passed, +THETAH after.
     *
     * FUNCTION NAME: targ_range_angle
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

        if (!AGC_ASSERT(agc_flag_test(AGC_FLAG__05GSW)))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (AGC_STATUS_OK);
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
    st = agc_navaux_get_padload(&pad);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

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

    if (!AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

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

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_LATSW)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

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
    es->latang   = m;                             /* UNI.RT /4    */
    es->k2roll   = entry_signmpac(agc_dp_neg(m)); /* -SGN(LATANG) */
    es->q2       = agc_dp_add(ip, agc_dp_mul_trunc(es->lad, AGC_ENTRY_Q21), AGC_ENTRY_Q22);
    es->gotoaddr = AGC_ENTRY_GOTO_INITROLL;
    es->posexit  = AGC_ENTRY_POSEXIT_SCALEPOP;

    return (AGC_STATUS_OK); /* RTB SERVNOUT */
} /* agc_entry_startent */

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
    /**********************************************************************
     * HUNTEST .. V1LEAD (p. 855): A1 = D, TEM1B, V1 = V + RDOT/TEM1B, A0 =
     * (V1/V)^2 (D + RDOT^2/(2 C1 HS TEM1B)), A1 = A0 if RDOT < 0, and V1 = V1 -
     * VQUIT when L/D < 0.
     *
     * FUNCTION NAME: hunt_a0calc
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
    agc_dp_t a0; /* A0, /805 B0 */

    es->a1    = es->d;
    es->tem1b = (es->rdot < 0) ? es->lad : es->lewd; /* RDOT sign */

    es->v1 = agc_dp_add(ip, agc_dp_div(ip, es->rdot, es->tem1b), es->v);
    a0     = agc_dp_div(ip, agc_dp_sq(es->rdot), es->tem1b);
    a0     = agc_dp_add(ip, agc_dp_div(ip, a0, AGC_ENTRY_TWO_C1_HS), es->d);
    a0     = agc_dp_mul_trunc(agc_dp_mul_trunc(a0, es->v1), es->v1);
    es->a0 = agc_dp_div(ip, a0, es->vsquare);

    if (es->rdot < 0)
    {
        es->a1 = es->a0;
    }

    if (es->l_d < 0)
    { /* V1LEAD */
        es->v1 = agc_dp_sub(ip, es->v1, AGC_ENTRY_VQUIT);
    }

    /**********************************************************************
     * HUNTEST1 (pp. 856): ALP, FACT1, FACT2, VL, GAMMAL1.
     *
     * FUNCTION NAME: hunt_exit_velocity
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
    agc_dp_t m; /* MPAC, scale per step */

    m         = agc_dp_div(ip, agc_dp_mul_trunc(es->a0, AGC_ENTRY_TWO_C1_HS), es->v1);
    m         = agc_dp_div(ip, agc_dp_div(ip, m, es->v1), es->lewd);
    es->alp   = m;                         /* 2 C1 HS A0/LEWD V1 V1 */
    es->fact1 = agc_dp_div(ip, es->v1, agc_dp_sub(ip, AGC_ENTRY_NEARONE, es->alp));
    m         = agc_dp_mul_trunc(agc_dp_sub(ip, es->alp, AGC_ENTRY_NEARONE), es->alp);
    es->fact2 = agc_dp_div(ip, m, es->a0); /* ALP(ALP-1)/A0, x 805  */

    /* DMP Q7, DAD ALP, SQRT: the DAD leaves MPAC+2 of the DMP product,
     * and SQRT reads all three words.                                    */
    m           = agc_tp_sqrt(agc_tp_add(ip, agc_dp_mul_tp(es->fact2, es->q7), agc_tp_from_dp(es->alp)));
    m           = agc_dp_sub(ip, AGC_ENTRY_NEARONE, m);
    es->vl      = agc_dp_mul_trunc(m, es->fact1); /* FACT1(1-SQRT(..))     */
    m           = agc_dp_mul_trunc(agc_dp_sub(ip, es->v1, es->vl), es->lewd);
    es->gammal1 = agc_dp_div(ip, m, es->vl);      /* LEWD (V1-VL)/VL       */

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

    if (agc_dp_sub(ip, AGC_ENTRY_HALVE, es->v1) < 0)
    {
        es->dvl = agc_dp_sub(ip, es->v1, es->vl); /* V1 - VL     */
        es->vs1 = es->v1;
    }

    /**********************************************************************
     * GETDHOOK .. HUNTEST3 (pp. 857-858): DHOOK, AHOOKDV, GAMMAL, and the
     * smoothed GAMMAL1 = GAMMAL1 + Q19 (GAMMAL - GAMMAL1).
     *
     * FUNCTION NAME: hunt_getdhook
     *
     * ARGUMENTS:
     *   Argument  Type                 I/O  Description (scale)
     *   --------  -------------------  ---  ---------------------------------
     *   ip        agc_interp_t *       IO   interpreter state of the job:
     *                                       OVFIND, DVSIGN, alarms
     *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
     *                                       names)
     *
     * RETURNS: agc_dp_t -- agc_dp_t -- GAMMAL, revs B0
     **********************************************************************/
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
        /**********************************************************************
         * NEGAMA (p. 862): GAMMAL < 0. Move VL so that GAMMAL = 0: VL = VL +
         * (GAMMAL VL/3) / (LEWD/3 - DVL (AHOOKDV + 1/24) CH1/DHOOK VL), Q7 =
         * DHOOKYQ7(VL), VBARS = VL^2; returns GAMMAL = 0.
         *
         * FUNCTION NAME: hunt_negama
         *
         * ARGUMENTS:
         *   Argument  Type                 I/O  Description (scale)
         *   --------  -------------------  ---  ---------------------------------
         *   ip        agc_interp_t *       IO   interpreter state of the job:
         *                                       OVFIND, DVSIGN, alarms
         *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
         *                                       names)
         *   gammal    agc_dp_t             I    GAMMAL (< 0), revs B0
         *
         * RETURNS: agc_dp_t -- agc_dp_t -- the new GAMMAL, 0
         **********************************************************************/
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
}

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
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- DIFF = predicted range - THETAH, revs B0
 **********************************************************************/
static agc_dp_t entry_ranger_core(agc_interp_t *ip, agc_entry_state_t *es, agc_dp_t gammal)
{
    agc_dp_t e4;  /* E/4, B2 */
    agc_dp_t m;   /* MPAC, scale per step */
    agc_dp_t sum; /* range sum, revs B0 */

    es->cosg_2 = agc_dp_sub(ip, AGC_ENTRY_HALVE, agc_dp_shr(agc_dp_sq(gammal), 2));
    m          = agc_dp_mul_trunc(agc_dp_sub(ip, es->vbars, AGC_ENTRY_HALVE), es->vbars);

    /* DMP DMP SL2 DAD SQRT: the shift, add and root see the TP MPAC.     */
    e4         = agc_tp_sqrt(agc_tp_add(ip, agc_tp_shift(ip, agc_dp_mul_tp(agc_dp_mul_trunc(m, es->cosg_2), es->cosg_2), 2), agc_tp_from_dp(AGC_ENTRY_ONE_16TH)));
    m          = agc_dp_mul_trunc(agc_dp_mul_trunc(es->vbars, es->cosg_2), gammal);
    es->askep  = agc_dp_shl(ip, agc_dp_asin(ip, agc_dp_div(ip, m, e4)), 1);
    es->asp1   = agc_dp_add(ip, agc_dp_mul_trunc(es->vl, AGC_ENTRY_Q3), es->q2);
    m          = agc_dp_div(ip, agc_dp_mul_trunc(agc_dp_sq(es->v1), es->q7), es->vbars);
    m          = entry_log(ip, agc_dp_div(ip, m, es->a0)); /* -LOG/32      */
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

    return (sum);
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

    if (agc_dp_sub(ip, es->v, es->v1) >= 0)
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
    es->rdotref = agc_dp_mul_trunc(agc_dp_sub(ip, es->v1, es->vref), es->lewd);
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
    agc_interp_t      *ip = p->ip; /* interpreter state */
    agc_entry_state_t *es = p->es; /* entry erasables */
    agc_dp_t           m;          /* MPAC, scale per step */
    agc_dp_t           q;          /* DOWNCNTL term, B0 */

    agc_interp_clear_ovf(ip);             /* BOVB TCDANZIG    */
    m        = agc_dp_mul_trunc(agc_dp_sub(ip, es->v, es->v1), es->lad);
    m        = agc_dp_mul_trunc(agc_dp_add(ip, m, es->rdot), AGC_ENTRY_K2D);
    p->pd_ld = agc_dp_add(ip, m, agc_dp_shr(es->lad, 8));
    q        = agc_dp_mul_trunc(agc_dp_sq(agc_dp_sub(ip, es->v1, es->v)), es->lad);
    q        = agc_dp_div(ip, q, AGC_ENTRY_TWO_C1_HS);
    m        = agc_dp_div(ip, agc_dp_sq(es->v1), es->vsquare);
    m        = agc_dp_div(ip, es->a0, m); /* BDDV A0           */
    p->mpac  = agc_dp_sub(ip, m, q);      /* DREF              */

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

    if (agc_entry_desired_gimbal_angles() != AGC_STATUS_OK)
    {
        p->status = AGC_STATUS_BAD_ARG;

        return (ST_ERROR);
    }

    return (ST_ENDEXIT); /* QPRET = ENDEXIT            */
}

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

    if (entry_final_phase(p->es) != AGC_STATUS_OK)
    {
        p->status = AGC_STATUS_BAD_ARG;

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

    /**********************************************************************
     * BACK2 (pp. 870-871): GRAD and the six interpolated functions FX+5..FX+0 =
     * F1, F2, RDOTREF, RTOGO, -AREF, F3 (SP).
     *
     * FUNCTION NAME: predict_interp
     *
     * ARGUMENTS:
     *   Argument  Type                 I/O  Description (scale)
     *   --------  -------------------  ---  ---------------------------------
     *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
     *                                       names)
     *   tem1b     agc_sp_t             I    V - VREF(JJ), B0
     *
     * RETURNS: none
     **********************************************************************/
    agc_sp_t tem1b = predict_find_jj(es, agc_dp_high(es->v));

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
    p->es->l_d = agc_dp_mul_trunc(entry_signmpac(p->mpac), p->es->lad);

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
static entry_step_t entry_initial_roll_step(entry_pass_t *p, entry_step_t s)
{
    switch ((int) s)
    {
        case ST_INITROLL:
            /**********************************************************************
             * INITROLL (p. 853): hold the HEADSUP roll until .05 g; at .05 g (P64) KAT
             * = min(LEQ^3 KA1 + KA2, KALIM), then D0EQ or (V < VFINAL1) KEP2.
             *
             * FUNCTION NAME: step_initroll
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

            if (entry_post_0_05g(es) != AGC_STATUS_OK)
            { /* RTB P64: MM 64, N74   */
                p->status = AGC_STATUS_BAD_ARG;

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
        }
        case ST_D0EQ:
            /**********************************************************************
             * D0EQ (pp. 853-854): constant drag level D0 = KA3 LEQ + KA4, C/D0 = (-4/25
             * g)/D0 B-8; L/D = LAD, or -LAD when V - VFINAL + (RDOT/V)^3 / K44 is
             * negative.
             *
             * FUNCTION NAME: step_d0eq
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
            /**********************************************************************
             * KATEST (p. 854): if KAT - D >= 0 keep the roll (LIMITL/D), else CONSTD.
             *
             * FUNCTION NAME: step_katest
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
        {
            return ((agc_dp_sub(p->ip, p->es->kat, p->es->d) >= 0) ? ST_LIMITLD : ST_CONSTD); /* KAT test */
        }
        case ST_HUNT_TRIGGER:
            /**********************************************************************
             * INITRL1 (p. 854): when RDOT + VRCONT >= 0 start HUNTEST, else KATEST.
             *
             * FUNCTION NAME: step_hunt_trigger
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
    }

    return (ST_ERROR); /* unreachable: the range was asserted */
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
static entry_step_t entry_huntest_step(entry_pass_t *p, entry_step_t s)
{
    switch ((int) s)
    {
        case ST_HUNTEST:
            return (step_huntest(p));
        case ST_RANGER:
            /**********************************************************************
             * RANGER step (p. 860): DIFF, then UPSY (|DIFF| < 25 NM), GETDLEWD (HIND
             * set or DIFF < 0) or DCONSTD.
             *
             * FUNCTION NAME: step_ranger
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
        {
            agc_entry_state_t *es = p->es; /* entry erasables */

            es->diff = entry_ranger_core(p->ip, es, p->mpac);

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
            /**********************************************************************
             * GETDLEWD / LWDSTORE / LEWDPTR / LEWDOVFL (pp. 860-861): secant step on
             * LEWD, DLEWD = DLEWD DIFF/(DIFFOLD - DIFF).
             *
             * FUNCTION NAME: step_getdlewd
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
            /**********************************************************************
             * SIDETRAK (p. 861): the iteration continues below SERVICER priority while
             * the next passes go to ENDEXIT; restart group 4 at PRE-HUNT. DEVIATION:
             * the C pass runs the whole iteration in one call (bounded by
             * AGC_ENTRY_HUNTEST_MAX_ITER); PRIOCHNG lowers the priority but no
             * pre-emption point is taken. If the bound is hit, the pass ends and the
             * next pass resumes HUNTEST from the current LEWD/DLEWD.
             *
             * FUNCTION NAME: step_sidetrak
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
        {
            agc_entry_state_t *es = p->es; /* entry erasables */

            /* CA PRIO16 / TS PHSPRDT5: group 5 restart priority one below group 4
             * (no exec API for PHSPRDT5 alone; the restart uses the stored one).  */
            agc_phase_change(000474);               /* 4.47 PRE-HUNT      */
            agc_exec_prio_change(AGC_ENTRY_PRIO16); /* BUSY: see above    */
            es->gotoaddr = AGC_ENTRY_GOTO_ENDEXIT;  /* ADENDEXT           */
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
    }

    return (ST_ERROR); /* unreachable: the range was asserted */
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
static entry_step_t entry_upcontrol_step(entry_pass_t *p, entry_step_t s)
{
    switch ((int) s)
    {
        case ST_GOTOUPSY:
            /**********************************************************************
             * GOTOUPSY (p. 864): RTB P65 (mode, N69 display job), GOTOADDR = UPCONTRL,
             * REFAZE10 -> UPCONTRL.
             *
             * FUNCTION NAME: step_gotoupsy
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
        {
            if (entry_upcontrol(p->es) != AGC_STATUS_OK)
            {
                p->status = AGC_STATUS_BAD_ARG;

                return (ST_ERROR);
            }

            return (ST_UPCONTRL);
        }
        case ST_UPCONTRL:
            return (step_upcontrl(p));
        case ST_UPCONTROL_DRAG_TEST:
            /**********************************************************************
             * CONT1 (p. 864): L/D = LAD while D >= A0.
             *
             * FUNCTION NAME: step_upcontrol_drag_test
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
            /**********************************************************************
             * CONTINU2 (p. 865): FACTOR = (D - Q7)/(A1 - Q7) when D >= Q7MIN.
             *
             * FUNCTION NAME: step_upcontrol_q7min_test
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
            /**********************************************************************
             * BECONSTD (p. 867): GOTOADDR = HUNTEST, RTB KILLGRP4.
             *
             * FUNCTION NAME: step_beconstd
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
        {
            p->es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST;

            if (entry_disable_restart_group() != AGC_STATUS_OK)
            {
                p->status = AGC_STATUS_BAD_ARG;

                return (ST_ERROR);
            }

            return (ST_CONSTD);
        }
        case ST_CONSTD:
            /**********************************************************************
             * CONSTD (p. 867): RDOTREF = -2 HS D0/V, L/D = (C/D0 LEQ + K2D (RDOT -
             * RDOTREF) + K1D (D - D0)) 2^8.
             *
             * FUNCTION NAME: step_constd
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
        {
            agc_interp_t      *ip = p->ip; /* interpreter state */
            agc_entry_state_t *es = p->es; /* entry erasables */
            agc_dp_t           m;          /* MPAC, scale per step */

            agc_interp_clear_ovf(ip); /* BOVB TCDANZIG    */
            m        = agc_dp_mul_trunc(AGC_ENTRY_TWO_HS, es->d0);
            m        = agc_dp_add(ip, agc_dp_div(ip, m, es->v), es->rdot);
            m        = agc_dp_mul_trunc(m, AGC_ENTRY_K2D);
            p->pd_ld = agc_dp_add(ip, m, agc_dp_mul_trunc(es->leq, es->c_d0));
            p->mpac  = es->d0;        /* DREF = D0         */

            return (ST_CONSTANT_DRAG_LD);
        }
        case ST_CONSTANT_DRAG_LD:
            /**********************************************************************
             * CONSTD1 (p. 867): L/D = (K1D (D - DREF) + push) SL 8, then NEGTESTS.
             *
             * FUNCTION NAME: step_constant_drag_ld
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
        {
            agc_interp_t *ip = p->ip; /* interpreter state */
            agc_dp_t      m;          /* MPAC, B0 */

            m = agc_dp_sub(ip, p->es->d, p->mpac); /* BDSU D           */

            /* DMP K1D, DAD (push), SL 8D: the shift moves MPAC+2 bits up.        */
            p->mpac = agc_tp_to_dp(agc_tp_shift(ip, agc_tp_add(ip, agc_dp_mul_tp(m, AGC_ENTRY_K1D), agc_tp_from_dp(p->pd_ld)), 8));

            return (ST_NEGTESTS);
        }
        case ST_DOWNCNTL:
            return (step_downcntl(p));
        default:
            break;
    }

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
static entry_step_t entry_ballistic_step(entry_pass_t *p, entry_step_t s)
{
    switch ((int) s)
    {
        case ST_KEP:
            /**********************************************************************
             * KEP (p. 869): RTB P66, GOTOADDR = KEP2.
             *
             * FUNCTION NAME: step_kep
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
        {
            if (entry_ballistic(p->es) != AGC_STATUS_OK)
            {
                p->status = AGC_STATUS_BAD_ARG;

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
static entry_step_t entry_final_phase_step(entry_pass_t *p, entry_step_t s)
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
            /**********************************************************************
             * GOPOSLAD (p. 872): L/D = LAD.
             *
             * FUNCTION NAME: step_goposlad
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
        {
            p->es->l_d = p->es->lad;

            return (ST_LIMITLD);
        }
        case ST_LIMITLD:
            return (step_limitld(p));
        case ST_LATERAL_SWITCH:
            /**********************************************************************
             * L353 (p. 873): switch K2ROLL when LATANG SIGN(K2ROLL) - Y >= 0, unless
             * NOSWITCH (BONCLR: tested and cleared).
             *
             * FUNCTION NAME: step_lateral_switch
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
            /**********************************************************************
             * L357 (p. 875): L/D1 = L/DCMINR SIGN(L/D).
             *
             * FUNCTION NAME: step_lateral_min_ld
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
        {
            p->es->l_d1 = agc_dp_sign(p->es->l_dcminr, p->es->l_d);

            return (ST_ROLL_COMMAND);
        }
        case ST_STEEROFF:
            /**********************************************************************
             * STEEROFF (p. 875): V < VQUIT. Start P67.1 (N67 flashing), restart 4.41
             * and 5.3, GOTOADDR = P67.2 and continue there.
             *
             * FUNCTION NAME: step_steeroff
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
        {
            agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_REFAZE10, AGC_TASK_NONE, 0, false, 0U };

            if (agc_exec_novac(AGC_ENTRY_PRIO16, AGC_JOB_ENTRY_FINAL_PHASE_DISPLAY) != AGC_STATUS_OK)
            {
                p->status = AGC_STATUS_FULL;

                return (ST_ERROR);
            }

            agc_phase_change2(000414, 010035, &v); /* 4.41 P67.1, SERVICER 5.3 */
            p->es->gotoaddr = AGC_ENTRY_GOTO_FINAL_PHASE_LATLONG;

            return (ST_FINAL_PHASE_LATLONG);
        }
        case ST_FINAL_PHASE_LATLONG:
            /**********************************************************************
             * P67.2 (P61-P67.agc p. 802): present LAT, LONG, ALT (pad radius, earth),
             * then P67.3: SERVNOUT without display.
             *
             * FUNCTION NAME: step_final_phase_latlong
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
        {
            p->status = agc_entry_final_phase_latlong();

            return ((p->status == AGC_STATUS_OK) ? ST_SERVNOUT : ST_ERROR); /* done */
        }
        default:
            break;
    }

    return (ST_ERROR); /* unreachable: the range was asserted */
} /* entry_final_phase_step */

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
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_REFAZE10, AGC_TASK_NONE, 0, false, 0U };

    *display       = false;
    agc_phase_change_var(010035, &v); /* SERVICER 5.3        */
    pass.ip        = entry_ip();
    pass.es        = agc_entry_state();
    pass.mpac      = 0;
    pass.pd_y      = 0;
    pass.pd_ld     = 0;
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
    agc_status_t st; /* TARGETNG status */

    *display = false;
    st       = agc_entry_targeting();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_entry_reestablish_sequencer(display));
}

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
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when diff
 *          is NULL
 **********************************************************************/
agc_status_t agc_entry_range_predict(agc_dp_t gammal, agc_dp_t *diff)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    if (!AGC_ASSERT(es->a0 > 0 && es->gammal1 != 0 && es->vbars > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *diff    = entry_ranger_core(entry_ip(), es, gammal);
    es->diff = *diff;

    return (AGC_STATUS_OK);
}
