/*
 * reentry.c -- CM entry guidance: REENTRY_CONTROL.agc (Comanche055,
 * pp. 844-881), "Colossus 2A" entry steering of GSOP AS-278 section 5.6.
 *
 * One call per 2 s SERVICER cycle (via CM/POSE -> POSEXIT) computes the
 * roll command ROLLC. The AGC chains interpretive sections with GOTO; here
 * each labelled section is a step function returning the next step, and
 * entry_sequencer() runs the chain under a fixed step bound (rule 2).
 * Values the AGC carries in MPAC or in the push-down list between labelled
 * sections travel in entry_pass_t.
 *
 * Scales: see agc_csm_entry.h and entry_lexicon.h. /2VS, /805, revs.
 */
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
#include "agc/csm/entry_internal.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/csm/entry_pending.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/io/dsky.h"
#include "agc/nav/auxiliary.h"

/* Interpreter state shared by the entry interpretive code (OVFIND).        */
static agc_interp_t entry_interp;
static bool         entry_interp_ready = false;

typedef enum entry_step
{
    ST_INITROLL = 0,
    ST_D0EQ,
    ST_INROLOUT,
    ST_KATEST,
    ST_INITRL1,
    ST_HUNTEST,
    ST_RANGER,
    ST_GETDLEWD,
    ST_SIDETRAK,
    ST_GOTOUPSY,
    ST_UPCONTRL,
    ST_CONT1,
    ST_CONT3,
    ST_CONTINU2,
    ST_UPCNTRL3,
    ST_NEGTESTS,
    ST_DCONSTD,
    ST_BECONSTD,
    ST_CONSTD,
    ST_CONSTD1,
    ST_DOWNCNTL,
    ST_KEP,
    ST_KEP2,
    ST_PREFINAL,
    ST_PREDICT3,
    ST_HAVDNRNG,
    ST_GONEGLAD,
    ST_GOMAXLD,
    ST_GLIMITER,
    ST_GOPOSLAD,
    ST_LIMITLD,
    ST_L353,
    ST_L355,
    ST_L357,
    ST_STEEROFF,
    ST_FINAL_PHASE_LATLONG,
    ST_ENDEXIT,  /* terminal: ENDEXIT -> OVERNOUT display -> SERVNOUT   */
    ST_SERVNOUT, /* terminal: SERVNOUT without display                  */
    ST_ERROR     /* terminal: a called routine failed                  */
} entry_step_t;

typedef struct entry_pass
{
    agc_interp_t      *ip;
    agc_entry_state_t *es;
    agc_dp_t           mpac;      /* value "in MPAC" at a step entry       */
    agc_dp_t           pd_y;      /* push-down cell: lateral Y (L350)      */
    agc_dp_t           pd_ld;     /* push-down cell: L/D (NEGTESTS), the
                                     K2D term (CONSTD1)                     */
    int                hunt_iter; /* HUNTEST iterations this pass          */
    agc_status_t       status;    /* first failure of a called routine     */
} entry_pass_t;

/* entry_ip -- the shared interpreter state for the entry lane.            */
agc_interp_t *entry_ip(void)
{
    if (!entry_interp_ready)
    {
        agc_interp_init(&entry_interp);
        entry_interp_ready = true;
    }
    (void) AGC_ASSERT(entry_interp_ready);
    return &entry_interp;
}

/* SIGNMPAC (RT8_OP_CODES.agc p. 1386): MPAC = +DPOSMAX if the high word of
 * MPAC is +0 or positive, else -DPOSMAX. DEVIATION: tested on the sign of
 * the whole DP value (no -0 / mixed-sign DP words in this model).          */
agc_dp_t entry_signmpac(agc_dp_t x)
{
    (void) AGC_ASSERT(x >= AGC_DP_MIN);
    (void) AGC_ASSERT(x <= AGC_DP_MAX);
    return (x >= 0) ? AGC_ENTRY_DPOSMAX : (agc_dp_t) (-AGC_ENTRY_DPOSMAX);
}

/* LOG (CSM_GEOMETRY.agc p. 291): returns -ln(x)/32 for 0 < x < 1.
 * x is normalised to [.5, 1) by n left shifts; with y = NEARLY1 - xn the
 * cubic series gives -ln(xn)/32, and n ln(2)/32 is added (SHORTMP).       */
agc_dp_t entry_log(agc_interp_t *ip, agc_dp_t x)
{
    int      shift = 0;
    agc_dp_t xn;
    agc_dp_t y;
    agc_dp_t p;

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(x > 0))
    {
        return 0;
    }
    xn = agc_dp_norm(x, &shift);
    (void) AGC_ASSERT(shift >= 0 && shift <= AGC_DP_BITS);
    y = agc_dp_sub(ip, AGC_ENTRY_NEARONE, xn); /* BDSU NEARLY1      */
    p = agc_dp_mul(ip, AGC_ENTRY_LOG_C3, y);   /* POLY, N-1 = 2:    */
    p = agc_dp_mul(ip, agc_dp_add(ip, p, AGC_ENTRY_LOG_C2), y); /* a3..a0 */
    p = agc_dp_mul(ip, agc_dp_add(ip, p, AGC_ENTRY_LOG_C1), y); /* a0 = 0 */
    /* + shift count x CLOG2/32 (SHORTMP of a small positive integer)      */
    return agc_dp_add(ip, p, (agc_dp_t) (AGC_ENTRY_CLOG2_32 * shift));
}

/* EARROT1 / EARROT2 (CSM_GEOMETRY.agc pp. 293-294): rotate RTINIT about
 * the polar axis UNITW through WIE DTEAROT:
 *   RT = UNIT(RTINIT + RTNORM (cos WT - 1) + RTEAST sin WT).
 * first = true is the EARROT1 entry (resolves RTEAST, RTNORM).            */
agc_status_t entry_earrot(agc_interp_t *ip, agc_entry_state_t *es, bool first)
{
    agc_vec_t unitw = agc_guid_state()->avg.unitw;
    agc_dp_t  wt;
    agc_vec_t a;
    agc_vec_t b;
    int       days;

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(es != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (first)
    {
        es->rteast = agc_vec_cross(ip, unitw, es->rtinit);
        es->rtnorm = agc_vec_cross(ip, es->rteast, unitw);
    }
    (void) agc_interp_test_ovf(ip); /* BOVB TCDANZIG  */
    wt = agc_dp_div(ip, es->dtearot, AGC_ENTRY_ONE_OVER_WIE); /* revs  */
    /* OVERADAY: remove whole sidereal days until the divide fits.        */
    for (days = 0; days < 32 && agc_interp_test_ovf(ip); days++)
    {
        agc_dp_t day = agc_dp_sign(AGC_ENTRY_ONE_OVER_WIE, es->dtearot);
        es->dtearot  = agc_dp_sub(ip, es->dtearot, day);
        wt           = agc_dp_div(ip, es->dtearot, AGC_ENTRY_ONE_OVER_WIE);
    }
    a      = agc_vec_scale(ip, es->rtnorm,
                           agc_dp_sub(ip, agc_dp_cos(ip, wt), AGC_ENTRY_HALVE));
    b      = agc_vec_scale(ip, es->rteast, agc_dp_sin(ip, wt));
    a      = agc_vec_shl(ip, agc_vec_add(ip, b, a), 1); /* VAD VSL1        */
    es->rt = agc_vec_unit(ip, agc_vec_add(ip, a, es->rtinit), NULL, NULL);
    return AGC_STATUS_OK;
}

/* ======================================================================
 * TARGETING (pp. 848-852)
 * ==================================================================== */

/* TARGETNG part 1: GETVEL / GETUNITV / DSTORE (p. 848). Velocity
 * (inertial, or relative once RELVELSW is set), VSQUARE, LEQ, V, RDOT, D
 * and UNI.                                                                 */
static agc_status_t targ_velocity(agc_interp_t *ip, agc_entry_state_t *es,
                                  const agc_state_vector_t *sv)
{
    agc_vec_t unitr = agc_guid_state()->avg.unitr;
    agc_dp_t  abval = 0;
    agc_dp_t  vsq   = 0;
    agc_dp_t  dmag;

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(sv != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_flag_test(AGC_FLAG_RELVELSW))
    {
        es->vel = agc_vec_neg(es->mvrel); /* -VREL left by CM/POSE */
    }
    else
    {
        es->vel = agc_vec_scale(ip, sv->v, AGC_ENTRY_KVSCALE); /* V/2VS   */
    }
    es->unitv   = agc_vec_unit(ip, es->vel, &abval, &vsq); /* GETUNITV  */
    es->vsquare = vsq;                                     /* VSQ/4, 34D    */
    es->leq     = agc_dp_sub(ip, vsq, AGC_ENTRY_FOURTH);   /* LEQ/4       */
    es->v       = abval;                                   /* V/2VS, 36D    */
    es->rdot    = agc_dp_shl(ip, agc_vec_dot(ip, es->vel, unitr), 1);
    dmag =
        agc_dp_mul(ip, agc_vec_abval(ip, agc_erase_delv()), AGC_ENTRY_KASCALE);
    dmag    = agc_dp_shl(ip, dmag, 1);
    es->d   = (dmag == 0) ? AGC_ENTRY_ONEBIT_DP : dmag; /* SETMIND        */
    es->uni = agc_vec_unit(ip, agc_vec_cross(ip, es->vel, unitr), NULL,
                           NULL); /* UNI = UNIT(VxR) */
    (void) AGC_ASSERT(es->v >= 0);
    return AGC_STATUS_OK;
}

/* GETETA / SUBETA / SUBETA2 (p. 851): D predicted half a cycle ahead,
 *   D = D + D (-RDOT/HS - 2D/V) DT/2,
 * then the time-of-flight estimate to the target in MPAC (cs B28).        */
static agc_dp_t targ_geteta(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_dp_t t;
    agc_dp_t u;

    (void) AGC_ASSERT(ip != NULL);
    (void) AGC_ASSERT(es->v > 0);
    t = agc_dp_div(ip, es->rdot, AGC_ENTRY_MINUS_HSCALED); /* -RDOT/HS */
    u = agc_dp_div(ip, agc_dp_mul(ip, es->d, AGC_ENTRY_MINUS_KSCALE), es->v);
    u = agc_dp_add(ip, u, t);
    es->d = agc_dp_add(ip, agc_dp_mul(ip, u, es->d), es->d);
    if (!agc_flag_test(AGC_FLAG_EGSW))
    {
        return agc_dp_mul(ip, es->thetah, AGC_ENTRY_KTETA); /* THETAH KTETA */
    }
    if (agc_dp_sub(ip, es->v, AGC_ENTRY_FOURTH) < 0)
    {                                           /* SUBETA: VMIN */
        agc_flag_set(AGC_FLAG_RELVELSW); /* SET RELVELSW */
    }
    return agc_dp_div(ip, agc_dp_mul(ip, es->thetah, AGC_ENTRY_KT1), es->v);
}

/* GETANGLE / TINYTHET / THETDONE (pp. 849, 852): THETAH = ARCCOS(RT.UNITR)
 * in revs; near zero the arccos is replaced by 32 sqrt(2 (1/4 - x)) KACOS.
 * RTGON67 = -THETAH before the target is passed, +THETAH after.           */
static void targ_range_angle(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_dp_t x = agc_vec_dot(ip, es->rt, agc_guid_state()->avg.unitr);
    agc_dp_t m = agc_dp_sub(ip, x, AGC_ENTRY_NEAR1_4);

    (void) AGC_ASSERT(ip != NULL);
    if (m >= 0)
    {
        /* TINYTHET: MPAC - 1BITDP+1 = x - 1/4; ABS; SL 13D; SQRT; x KACOS */
        m          = agc_dp_abs(agc_dp_sub(ip, m, AGC_ENTRY_ONEBIT_DP_P1));
        m          = agc_dp_sqrt(ip, agc_dp_shl(ip, m, 13));
        es->thetah = agc_dp_mul(ip, m, AGC_ENTRY_KACOS);
    }
    else
    {
        m          = agc_dp_shl(ip, agc_dp_add(ip, m, AGC_ENTRY_NEAR1_4), 1);
        es->thetah = agc_dp_acos(ip, m); /* THETAH/360 */
    }
    (void) AGC_ASSERT(es->thetah >= 0);
    es->range_to_target =
        agc_flag_test(AGC_FLAG_GONEBY) ? es->thetah : agc_dp_neg(es->thetah);
}

/* L/DCALC (p. 850): measured lift-to-drag from DELVREF and UXA,
 *   L/D = SQRT((|DV|^2 - (DV.UXA)^2) / (DV.UXA)^2), skipped on overflow
 * (OVFIND last cleared in EARROT2, as in the AGC).                        */
static void targ_ldcalc(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_vec_t dv  = agc_guid_state()->avg.delvref;
    agc_dp_t  du  = agc_dp_shl(ip, agc_vec_dot(ip, dv, es->uxa), 1);
    agc_dp_t  dsq = agc_dp_sq(ip, du);
    agc_dp_t  r;

    (void) AGC_ASSERT(ip != NULL);
    (void) AGC_ASSERT(dsq >= 0);
    r = agc_dp_div(ip, agc_dp_sub(ip, agc_vec_sq(ip, dv), dsq), dsq);
    if (!agc_interp_test_ovf(ip))
    {
        es->l_dcalc = agc_dp_sqrt(ip, r);
    }
}

/* TARGETNG (pp. 848-850): called each pass by SCALEPOP (and by P61).
 * Updates the velocity data, D, the rotated target RT, LATANG, GONEBY,
 * THETAH, RTGON67, .05GSW and L/DCALC.                                    */
agc_status_t agc_entry_targeting(void)
{
    agc_interp_t      *ip    = entry_ip();
    agc_entry_state_t *es    = agc_entry_state();
    agc_vec_t          unitr = agc_guid_state()->avg.unitr;
    agc_state_vector_t sv;
    agc_status_t       st;
    agc_dp_t           mpac = 0;

    if (!AGC_ASSERT(es != NULL) ||
        agc_sv_get(AGC_SV_THIS, &sv) != AGC_STATUS_OK)
    {
        return AGC_STATUS_BAD_ARG;
    }
    st = targ_velocity(ip, es, &sv);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (!agc_flag_test(AGC_FLAG_RELVELSW))
    {
        mpac = targ_geteta(ip, es); /* else 3ZEROS          */
    }
    /* UPDATERT: DTEAROT = estimate + (PIPTIME - TIME/RTO)                 */
    es->dtearot = agc_dp_add(ip, agc_dp_sub(ip, mpac, es->time_rto), sv.t);
    st          = entry_earrot(ip, es, false);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    es->latang = agc_vec_dot(ip, es->rt, es->uni); /* LATANG/4 */
    agc_flag_clear(AGC_FLAG_GONEBY);
    if (agc_vec_dot(ip, agc_vec_cross(ip, es->rt, unitr), es->uni) < 0)
    {
        agc_flag_set(AGC_FLAG_GONEBY); /* have gone past target  */
    }
    targ_range_angle(ip, es);
    if (agc_dp_sub(ip, es->d, AGC_ENTRY_G05) < 0)
    {
        agc_flag_clear(AGC_FLAG__05GSW); /* NO.05G (for the DAP)   */
        return AGC_STATUS_OK;
    }
    agc_flag_set(AGC_FLAG__05GSW);
    targ_ldcalc(ip, es);
    return AGC_STATUS_OK;
}

/* ======================================================================
 * INITIALISATION (pp. 844-846)
 * ==================================================================== */

/* STARTEN1 (pp. 845-846): initial target vector RTINIT at PIPTIME from
 * LAT(SPL), LNG(SPL) (pad radius, earth), the first RT 500 s ahead, and
 * THETAH. Called by STARTENT and by P61.                                   */
agc_status_t entry_starten1(agc_interp_t *ip, agc_entry_state_t *es)
{
    const agc_latlong_opt_t opt = {false, false}; /* CLEAR LUNAFLAG, ERADFLAG */
    agc_navaux_padload_t    pad;
    agc_latlong_t           ll = {0, 0, 0, 0};
    agc_state_vector_t      sv;
    agc_vec_t               r;
    agc_status_t            st;

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(es != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    st = agc_sv_get(AGC_SV_THIS, &sv);
    if (st == AGC_STATUS_OK)
    {
        st = agc_navaux_get_padload(&pad);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_flag_clear(AGC_FLAG_ERADFLAG); /* use pad radius        */
    agc_flag_clear(AGC_FLAG_LUNAFLAG); /* earth lat-long        */
    ll.lat       = es->lat_spl;
    ll.lon       = es->lng_spl; /* ALT = 0 (3ZEROS)      */
    es->time_rto = sv.t;        /* TIME/RTO = PIPTIME    */
    st = agc_latlong_to_r(ip, &pad, opt, &ll, sv.t, &r, NULL); /* LALOTORV */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    es->rtinit  = agc_vec_unit(ip, r, NULL, NULL); /* .5 unit target  */
    es->dtearot = AGC_ENTRY_SEC500;                /* 500SEC nominal  */
    st          = entry_earrot(ip, es, true);      /* EARROT1 -> RT   */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    es->thetah = agc_dp_acos(
        ip, agc_dp_shl(ip, agc_vec_dot(ip, es->rt, agc_guid_state()->avg.unitr),
                       1));
    return AGC_STATUS_OK;
}

/* STARTENT entry-switch initialisation (p. 844): clear ENTMASK = 011774
 * (ENTRYDSP GONEPAST RELVELSW EGSW NOSWITCH HIND INRLSW LATSW .05GSW), then
 * set ENTRYSW = 011010 (ENTRYDSP, GONEPAST, LATSW). INHINT'ed in the AGC. */
static void entry_init_switches(void)
{
    static const agc_flag_id_t clr[] = {
        AGC_FLAG_ENTRYDSP, AGC_FLAG_GONEPAST, AGC_FLAG_RELVELSW,
        AGC_FLAG_EGSW,     AGC_FLAG_NOSWITCH, AGC_FLAG_HIND,
        AGC_FLAG_INRLSW,   AGC_FLAG_LATSW,    AGC_FLAG__05GSW};
    unsigned i;

    agc_inhint();
    for (i = 0; i < (sizeof clr / sizeof clr[0]); i++)
    {
        agc_flag_clear(clr[i]); /* flag API cannot fail on valid ids */
    }
    agc_flag_set(AGC_FLAG_ENTRYDSP);
    agc_flag_set(AGC_FLAG_GONEPAST);
    agc_flag_set(AGC_FLAG_LATSW);
    agc_relint();
    (void) AGC_ASSERT(agc_flag_test(AGC_FLAG_LATSW));
    (void) AGC_ASSERT(!agc_flag_test(AGC_FLAG_HIND));
}

/* STARTENT (pp. 844-845), MM 63: first pass of entry guidance, entered from
 * CM/POSE. Ends with SERVNOUT (no display on the first pass); the caller
 * (CM/POSE) does the SERVNOUT.                                             */
agc_status_t agc_entry_startent(void)
{
    agc_interp_t      *ip = entry_ip();
    agc_entry_state_t *es = agc_entry_state();
    agc_state_vector_t sv;
    agc_status_t       st;
    agc_dp_t           m;

    if (!AGC_ASSERT(es != NULL) ||
        agc_sv_get(AGC_SV_THIS, &sv) != AGC_STATUS_OK)
    {
        return AGC_STATUS_BAD_ARG;
    }
    entry_init_switches();
    es->lod      = agc_dp_from_sp(es->lodpad); /* SLOAD LODPAD */
    es->lad      = agc_dp_from_sp(es->ladpad);
    es->l_dcminr = agc_dp_mul(ip, es->lad, AGC_ENTRY_COS15);
    es->klat     = agc_dp_shr(agc_dp_mul(ip, AGC_ENTRY_ONE_12TH, es->lad), 1);
    es->q7       = AGC_ENTRY_Q7F;
    es->factor   = AGC_ENTRY_NEARONE;
    /* L/D = -LAD SGN(HEADSUP)                                             */
    es->l_d = agc_dp_neg(agc_dp_sign(es->lad, agc_dp_from_sp(es->headsup)));
    st      = entry_starten1(ip, es);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    m = agc_vec_dot(
        ip,
        agc_vec_unit(ip, agc_vec_cross(ip, sv.v, agc_guid_state()->avg.unitr),
                     NULL, NULL),
        es->rt);
    es->latang = m;                             /* UNI.RT /4    */
    es->k2roll = entry_signmpac(agc_dp_neg(m)); /* -SGN(LATANG) */
    es->q2 =
        agc_dp_add(ip, agc_dp_mul(ip, es->lad, AGC_ENTRY_Q21), AGC_ENTRY_Q22);
    es->gotoaddr = AGC_ENTRY_GOTO_INITROLL;
    es->posexit  = AGC_ENTRY_POSEXIT_SCALEPOP;
    return AGC_STATUS_OK; /* RTB SERVNOUT */
}

/* ======================================================================
 * INITIAL ROLL (pp. 853-855), MM 63/64
 * ==================================================================== */

/* INITROLL (p. 853): hold the HEADSUP roll until .05 g; at .05 g (P64)
 * KAT = min(LEQ^3 KA1 + KA2, KALIM), then D0EQ or (V < VFINAL1) KEP2.    */
static entry_step_t step_initroll(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           kat;

    (void) AGC_ASSERT(ip != NULL);
    if (agc_flag_test(AGC_FLAG_INRLSW))
    {
        return ST_INITRL1;
    }
    if (!agc_flag_test(AGC_FLAG__05GSW))
    {
        return ST_LIMITLD; /* D < .05 g             */
    }
    kat = agc_dp_mul(ip, agc_dp_sq(ip, es->leq), es->leq);
    kat = agc_dp_div(ip, kat, AGC_ENTRY_ONE_OVER_KA1);
    kat = agc_dp_add(ip, kat, AGC_ENTRY_KA2);
    if (entry_post_0_05g(es) != AGC_STATUS_OK)
    { /* RTB P64: MM 64, N74   */
        p->status = AGC_STATUS_BAD_ARG;
        return ST_ERROR;
    }
    es->kat =
        (agc_dp_sub(ip, kat, AGC_ENTRY_KALIM) < 0) ? kat : AGC_ENTRY_KALIM;
    (void) AGC_ASSERT(es->kat <= AGC_ENTRY_KALIM);
    /* CLEAR GONEPAST: ROLLC follows HEADSUP only until P64.               */
    agc_flag_clear(AGC_FLAG_GONEPAST);
    if (agc_dp_sub(ip, es->v, AGC_ENTRY_VFINAL1) >= 0)
    {
        return ST_D0EQ;
    }
    es->gotoaddr = AGC_ENTRY_GOTO_KEP2; /* idle until D > .2 g   */
    return ST_INROLOUT;
}

/* D0EQ (pp. 853-854): constant drag level D0 = KA3 LEQ + KA4,
 * C/D0 = (-4/25 g)/D0 B-8; L/D = LAD, or -LAD when
 * V - VFINAL + (RDOT/V)^3 / K44 is negative.                              */
static entry_step_t step_d0eq(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           r;
    agc_dp_t           x;

    (void) AGC_ASSERT(es->v > 0);
    es->d0 =
        agc_dp_add(ip, agc_dp_mul(ip, es->leq, AGC_ENTRY_KA3), AGC_ENTRY_KA4);
    (void) AGC_ASSERT(es->d0 > 0);
    es->c_d0 = agc_dp_div(ip, AGC_ENTRY_C001, es->d0); /* BDDV       */
    (void) agc_interp_test_ovf(ip);                    /* BOV +1     */
    es->l_d = es->lad;
    r       = agc_dp_div(ip, es->rdot, es->v); /* RDOT/V     */
    x       = agc_dp_mul(ip, agc_dp_sq(ip, r), r);
    x       = agc_dp_div(ip, x, AGC_ENTRY_ONE_OVER_K44);
    x       = agc_dp_sub(ip, x, AGC_ENTRY_VFINAL);
    x       = agc_dp_add(ip, x, es->v);
    if (agc_interp_test_ovf(ip) || x < 0)
    {
        return ST_INROLOUT;
    }
    es->l_d = agc_dp_neg(es->lad);
    return ST_INROLOUT;
}

/* INROLOUT (p. 854): BOFSET INRLSW -> LIMITL/D (set at the end for
 * restart protection). Reached only with INRLSW clear.                     */
static entry_step_t step_inrolout(entry_pass_t *p)
{
    bool was_set = agc_flag_test(AGC_FLAG_INRLSW);

    (void) AGC_ASSERT(p != NULL);
    agc_flag_set(AGC_FLAG_INRLSW);
    return was_set ? ST_KATEST : ST_LIMITLD;
}

/* KATEST (p. 854): if KAT - D >= 0 keep the roll (LIMITL/D), else CONSTD.  */
static entry_step_t step_katest(entry_pass_t *p)
{
    (void) AGC_ASSERT(p != NULL);
    (void) AGC_ASSERT(p->es->kat >= 0);
    return (agc_dp_sub(p->ip, p->es->kat, p->es->d) >= 0) ? ST_LIMITLD :
                                                            ST_CONSTD;
}

/* FOREHUNT (p. 863): initialise HUNTEST.                                  */
static void entry_forehunt(agc_entry_state_t *es)
{
    (void) AGC_ASSERT(es != NULL);
    es->diffold = 0;
    es->dlewd   = AGC_ENTRY_DLEWD0;
    es->lewd    = AGC_ENTRY_LEWD1;
    (void) AGC_ASSERT(es->lewd > 0);
}

/* INITRL1 (p. 854): when RDOT + VRCONT >= 0 start HUNTEST, else KATEST.  */
static entry_step_t step_initrl1(entry_pass_t *p)
{
    agc_entry_state_t *es = p->es;

    (void) AGC_ASSERT(p->ip != NULL);
    if (agc_dp_add(p->ip, es->rdot, AGC_ENTRY_VRCONT) < 0)
    {
        return ST_KATEST;
    }
    entry_forehunt(es);
    es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST; /* after FOREHUNT (restarts)   */
    (void) AGC_ASSERT(es->gotoaddr == AGC_ENTRY_GOTO_HUNTEST);
    return ST_HUNTEST;
}

/* ======================================================================
 * HUNTEST (pp. 855-858, 862) and RANGER (pp. 859-861)
 * ==================================================================== */

/* DHOOKYQ7 (p. 862): ((1 - x/FACT1)^2 - ALP) / FACT2, x = VS1 or VL.      */
static agc_dp_t entry_dhookyq7(agc_interp_t *ip, const agc_entry_state_t *es,
                               agc_dp_t x)
{
    agc_dp_t m;

    (void) AGC_ASSERT(es->fact1 != 0);
    (void) AGC_ASSERT(es->fact2 != 0);
    m = agc_dp_div(ip, agc_dp_shr(x, 1), es->fact1);           /* SR1 DDV    */
    m = agc_dp_shl(ip, agc_dp_sub(ip, AGC_ENTRY_HALVE, m), 1); /* BDSU SL1 */
    m = agc_dp_sub(ip, agc_dp_sq(ip, m), es->alp);
    return agc_dp_div(ip, m, es->fact2);
}

/* HUNTEST .. V1LEAD (p. 855): A1 = D, TEM1B, V1 = V + RDOT/TEM1B,
 * A0 = (V1/V)^2 (D + RDOT^2/(2 C1 HS TEM1B)), A1 = A0 if RDOT < 0, and
 * V1 = V1 - VQUIT when L/D < 0.                                            */
static void hunt_a0calc(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_dp_t a0;

    (void) AGC_ASSERT(es->vsquare > 0);
    es->a1    = es->d;
    es->tem1b = (es->rdot < 0) ? es->lad : es->lewd;
    (void) AGC_ASSERT(es->tem1b != 0);
    es->v1 = agc_dp_add(ip, agc_dp_div(ip, es->rdot, es->tem1b), es->v);
    a0     = agc_dp_div(ip, agc_dp_sq(ip, es->rdot), es->tem1b);
    a0     = agc_dp_add(ip, agc_dp_div(ip, a0, AGC_ENTRY_TWO_C1_HS), es->d);
    a0     = agc_dp_mul(ip, agc_dp_mul(ip, a0, es->v1), es->v1);
    es->a0 = agc_dp_div(ip, a0, es->vsquare);
    if (es->rdot < 0)
    {
        es->a1 = es->a0;
    }
    if (es->l_d < 0)
    { /* V1LEAD */
        es->v1 = agc_dp_sub(ip, es->v1, AGC_ENTRY_VQUIT);
    }
}

/* HUNTEST1 (pp. 856): ALP, FACT1, FACT2, VL, GAMMAL1.                     */
static void hunt_huntest1(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_dp_t m;

    (void) AGC_ASSERT(es->v1 != 0);
    (void) AGC_ASSERT(es->lewd != 0);
    m = agc_dp_div(ip, agc_dp_mul(ip, es->a0, AGC_ENTRY_TWO_C1_HS), es->v1);
    m = agc_dp_div(ip, agc_dp_div(ip, m, es->v1), es->lewd);
    es->alp = m; /* 2 C1 HS A0/LEWD V1 V1 */
    es->fact1 =
        agc_dp_div(ip, es->v1, agc_dp_sub(ip, AGC_ENTRY_NEARONE, es->alp));
    m = agc_dp_mul(ip, agc_dp_sub(ip, es->alp, AGC_ENTRY_NEARONE), es->alp);
    es->fact2   = agc_dp_div(ip, m, es->a0); /* ALP(ALP-1)/A0, x 805  */
    m           = agc_dp_add(ip, agc_dp_mul(ip, es->fact2, es->q7), es->alp);
    m           = agc_dp_sub(ip, AGC_ENTRY_NEARONE, agc_dp_sqrt(ip, m));
    es->vl      = agc_dp_mul(ip, m, es->fact1); /* FACT1(1-SQRT(..))     */
    m           = agc_dp_mul(ip, agc_dp_sub(ip, es->v1, es->vl), es->lewd);
    es->gammal1 = agc_dp_div(ip, m, es->vl); /* LEWD (V1-VL)/VL       */
}

/* NEGAMA (p. 862): GAMMAL < 0. Move VL so that GAMMAL = 0:
 *   VL = VL + (GAMMAL VL/3) / (LEWD/3 - DVL (AHOOKDV + 1/24) CH1/DHOOK VL),
 * Q7 = DHOOKYQ7(VL), VBARS = VL^2; returns GAMMAL = 0.                    */
static agc_dp_t hunt_negama(agc_interp_t *ip, agc_entry_state_t *es,
                            agc_dp_t gammal)
{
    agc_dp_t num;
    agc_dp_t den;

    (void) AGC_ASSERT(gammal < 0);
    (void) AGC_ASSERT(es->dhook != 0 && es->vl != 0);
    num = agc_dp_mul(ip, agc_dp_mul(ip, gammal, es->vl), AGC_ENTRY_ONE_THIRD);
    den = agc_dp_add(ip, es->ahookdv, AGC_ENTRY_ONE_24TH);
    den = agc_dp_mul(ip, agc_dp_mul(ip, den, es->dvl), AGC_ENTRY_CH1);
    den = agc_dp_div(ip, agc_dp_div(ip, den, es->dhook), es->vl);
    den = agc_dp_sub(ip, agc_dp_mul(ip, es->lewd, AGC_ENTRY_ONE_THIRD), den);
    es->vl    = agc_dp_add(ip, agc_dp_div(ip, num, den), es->vl);
    es->q7    = entry_dhookyq7(ip, es, es->vl);
    es->vbars = agc_dp_sq(ip, es->vl);
    return 0; /* 3ZEROS       */
}

/* GETDHOOK .. HUNTEST3 (pp. 857-858): DHOOK, AHOOKDV, GAMMAL, and the
 * smoothed GAMMAL1 = GAMMAL1 + Q19 (GAMMAL - GAMMAL1).                    */
static agc_dp_t hunt_getdhook(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_dp_t m;
    agc_dp_t g;

    (void) AGC_ASSERT(es->q7 != 0);
    (void) AGC_ASSERT(es->vbars != 0);
    es->dhook   = entry_dhookyq7(ip, es, es->vs1);
    m           = agc_dp_div(ip, agc_dp_shr(es->dhook, 6), es->q7);
    es->ahookdv = agc_dp_sub(ip, m, AGC_ENTRY_CHOOK);
    m = agc_dp_mul(ip, agc_dp_add(ip, es->ahookdv, AGC_ENTRY_ONE_16TH),
                   AGC_ENTRY_CH1);
    m = agc_dp_mul(ip, agc_dp_mul(ip, m, es->dvl), es->dvl);
    m = agc_dp_div(ip, agc_dp_div(ip, m, es->dhook), es->vbars);
    g = agc_dp_sub(ip, es->gammal1, m);
    if (g < 0)
    {
        g = hunt_negama(ip, es, g);
    }
    es->gammal = g; /* HUNTEST3    */
    m = agc_dp_mul(ip, agc_dp_sub(ip, g, es->gammal1), AGC_ENTRY_HALVE);
    es->gammal1 = agc_dp_add(ip, m, es->gammal1);
    return g;
}

/* HUNTEST (pp. 855-858) without the sequencing: runs from the present
 * state through HUNTEST3 and reports where the AGC continues.             */
static agc_entry_hunt_result_t
entry_huntest_core(agc_interp_t *ip, agc_entry_state_t *es, agc_dp_t *gammal)
{
    (void) AGC_ASSERT(ip != NULL && gammal != NULL);
    hunt_a0calc(ip, es);
    hunt_huntest1(ip, es);
    if (agc_dp_sub(ip, es->vl, AGC_ENTRY_VLMIN) < 0)
    {
        return AGC_ENTRY_HUNT_PREFINAL; /* VL < VLMIN  */
    }
    es->vbars = agc_dp_sq(ip, es->vl);
    es->dvl   = agc_dp_sub(ip, AGC_ENTRY_HALVE, es->vl); /* VSAT - VL   */
    if (es->dvl < 0)
    {
        return AGC_ENTRY_HUNT_CONSTD; /* BECONSTD    */
    }
    es->vs1 = AGC_ENTRY_HALVE; /* VS1 = VSAT  */
    if (agc_dp_sub(ip, AGC_ENTRY_HALVE, es->v1) < 0)
    {
        es->dvl = agc_dp_sub(ip, es->v1, es->vl); /* V1 - VL     */
        es->vs1 = es->v1;
    }
    *gammal = hunt_getdhook(ip, es);
    (void) AGC_ASSERT(es->vl > 0);
    return AGC_ENTRY_HUNT_RANGER;
}

/* RANGER (pp. 859-860): range components (revs) for GAMMAL; returns DIFF =
 * ASKEP + ASP1 + ASPUP + ASPDWN + ASP3 - THETAH. Leaves OVFIND clear.     */
static agc_dp_t entry_ranger_core(agc_interp_t *ip, agc_entry_state_t *es,
                                  agc_dp_t gammal)
{
    agc_dp_t e4;
    agc_dp_t m;
    agc_dp_t sum;

    (void) AGC_ASSERT(es->a0 != 0 && es->lad != 0);
    (void) AGC_ASSERT(es->gammal1 != 0 && es->vbars != 0);
    es->cosg_2 =
        agc_dp_sub(ip, AGC_ENTRY_HALVE, agc_dp_shr(agc_dp_sq(ip, gammal), 2));
    m  = agc_dp_mul(ip, agc_dp_sub(ip, es->vbars, AGC_ENTRY_HALVE), es->vbars);
    m  = agc_dp_mul(ip, agc_dp_mul(ip, m, es->cosg_2), es->cosg_2);
    e4 = agc_dp_sqrt(ip,
                     agc_dp_add(ip, agc_dp_shl(ip, m, 2), AGC_ENTRY_ONE_16TH));
    m  = agc_dp_mul(ip, agc_dp_mul(ip, es->vbars, es->cosg_2), gammal);
    es->askep  = agc_dp_shl(ip, agc_dp_asin(ip, agc_dp_div(ip, m, e4)), 1);
    es->asp1   = agc_dp_add(ip, agc_dp_mul(ip, es->vl, AGC_ENTRY_Q3), es->q2);
    m          = agc_dp_div(ip, agc_dp_mul(ip, agc_dp_sq(ip, es->v1), es->q7),
                            es->vbars);
    m          = entry_log(ip, agc_dp_div(ip, m, es->a0)); /* -LOG/32      */
    es->aspup  = agc_dp_div(ip, agc_dp_mul(ip, m, AGC_ENTRY_C12), es->gammal1);
    m          = agc_dp_mul(ip, agc_dp_mul(ip, AGC_ENTRY_KC3, es->rdot), es->v);
    es->aspdwn = agc_dp_div(ip, agc_dp_div(ip, m, es->a0), es->lad);
    es->asp3 =
        agc_dp_mul(ip, agc_dp_sub(ip, AGC_ENTRY_Q6, gammal), AGC_ENTRY_Q5);
    /* ASPS(TM): the high words (the AGC stores the five DPs overlapping). */
    es->asps_tm[0] = agc_dp_high(es->askep);
    es->asps_tm[1] = agc_dp_high(es->asp1);
    es->asps_tm[2] = agc_dp_high(es->aspup);
    es->asps_tm[3] = agc_dp_high(es->aspdwn);
    es->asps_tm[4] = agc_dp_high(es->asp3);
    sum = agc_dp_add(ip, agc_dp_add(ip, es->asp3, es->aspdwn), es->aspup);
    sum = agc_dp_add(ip, agc_dp_add(ip, sum, es->asp1), es->askep);
    sum = agc_dp_sub(ip, sum, es->thetah);
    (void) agc_interp_test_ovf(ip); /* BOVB TCDANZIG */
    return sum;
}

/* HUNTEST step (p. 855): HUNTEST core, then RANGER, PREFINAL or BECONSTD. */
static entry_step_t step_huntest(entry_pass_t *p)
{
    agc_entry_hunt_result_t r;
    agc_dp_t                g = 0;

    (void) AGC_ASSERT(p->es != NULL);
    r = entry_huntest_core(p->ip, p->es, &g);
    (void) AGC_ASSERT(r == AGC_ENTRY_HUNT_RANGER ||
                      r == AGC_ENTRY_HUNT_PREFINAL ||
                      r == AGC_ENTRY_HUNT_CONSTD);
    if (r == AGC_ENTRY_HUNT_PREFINAL)
    {
        return ST_PREFINAL;
    }
    if (r == AGC_ENTRY_HUNT_CONSTD)
    {
        return ST_BECONSTD;
    }
    p->mpac = g;
    return ST_RANGER;
}

/* RANGER step (p. 860): DIFF, then UPSY (|DIFF| < 25 NM), GETDLEWD (HIND
 * set or DIFF < 0) or DCONSTD.                                            */
static entry_step_t step_ranger(entry_pass_t *p)
{
    agc_entry_state_t *es = p->es;

    (void) AGC_ASSERT(p->ip != NULL);
    es->diff = entry_ranger_core(p->ip, es, p->mpac);
    if (agc_dp_sub(p->ip, agc_dp_abs(es->diff), AGC_ENTRY_TOL_25NM) < 0)
    {
        return ST_GOTOUPSY;
    }
    if (agc_flag_test(AGC_FLAG_HIND))
    {
        return ST_GETDLEWD; /* AGC "GETLEWD": GETDLEWD    */
    }
    (void) AGC_ASSERT(es->diff != 0);
    return (es->diff >= 0) ? ST_DCONSTD : ST_GETDLEWD;
}

/* GETDLEWD / LWDSTORE / LEWDPTR / LEWDOVFL (pp. 860-861): secant step on
 * LEWD, DLEWD = DLEWD DIFF/(DIFFOLD - DIFF).                              */
static entry_step_t step_getdlewd(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           lewd;

    (void) AGC_ASSERT(ip != NULL);
    es->dlewd = agc_dp_div(ip, agc_dp_mul(ip, es->dlewd, es->diff),
                           agc_dp_sub(ip, es->diffold, es->diff));
    lewd      = agc_dp_add(ip, es->dlewd, es->lewd);
    if (lewd < 0)
    { /* LEWDPTR            */
        es->dlewd = agc_dp_neg(agc_dp_shr(es->lewd, 1));
        lewd      = agc_dp_add(ip, es->dlewd, es->lewd);
    }
    if (agc_interp_test_ovf(ip))
    { /* LEWDOVFL           */
        es->lewd = AGC_ENTRY_NEARONE;
        return ST_DCONSTD;
    }
    es->lewd = lewd;
    (void) AGC_ASSERT(es->lewd >= 0);
    return ST_SIDETRAK;
}

/* SIDETRAK (p. 861): the iteration continues below SERVICER priority while
 * the next passes go to ENDEXIT; restart group 4 at PRE-HUNT.
 * DEVIATION: the C pass runs the whole iteration in one call (bounded by
 * AGC_ENTRY_HUNTEST_MAX_ITER); PRIOCHNG lowers the priority but no
 * pre-emption point is taken. If the bound is hit, the pass ends and the
 * next pass resumes HUNTEST from the current LEWD/DLEWD.                  */
static entry_step_t step_sidetrak(entry_pass_t *p)
{
    agc_entry_state_t *es = p->es;

    (void) AGC_ASSERT(p->hunt_iter >= 0);
    /* CA PRIO16 / TS PHSPRDT5: group 5 restart priority one below group 4
     * (no exec API for PHSPRDT5 alone; the restart uses the stored one).  */
    agc_phase_change(4, 000474);            /* 4.47 PRE-HUNT      */
    agc_exec_prio_change(AGC_ENTRY_PRIO16); /* BUSY: see above    */
    es->gotoaddr = AGC_ENTRY_GOTO_ENDEXIT;         /* ADENDEXT           */
    es->diffold  = es->diff;
    agc_flag_set(AGC_FLAG_HIND);
    es->q7 = AGC_ENTRY_Q7F;
    p->hunt_iter++;
    if (p->hunt_iter >= AGC_ENTRY_HUNTEST_MAX_ITER)
    {
        es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST;
        return ST_ENDEXIT;
    }
    return ST_HUNTEST;
}

/* PRE-HUNT (p. 863): restart point of group 4 after SIDETRAK.            */
agc_status_t agc_job_pre_hunt(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_flag_clear(AGC_FLAG_HIND);
    entry_forehunt(es);
    es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST;
    return AGC_STATUS_OK; /* next SERVICER pass continues at HUNTEST */
}

/* ======================================================================
 * UP CONTROL (pp. 864-866), MM 65
 * ==================================================================== */

/* GOTOUPSY (p. 864): RTB P65 (mode, N69 display job), GOTOADDR =
 * UPCONTRL, REFAZE10 -> UPCONTRL.                                         */
static entry_step_t step_gotoupsy(entry_pass_t *p)
{
    (void) AGC_ASSERT(p->es != NULL);
    if (entry_upcontrol(p->es) != AGC_STATUS_OK)
    {
        p->status = AGC_STATUS_BAD_ARG;
        return ST_ERROR;
    }
    (void) AGC_ASSERT(p->es->gotoaddr == AGC_ENTRY_GOTO_UPCONTRL);
    return ST_UPCONTRL;
}

/* UPCONTRL / VLTEST (p. 864).                                             */
static entry_step_t step_upcontrl(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           m;

    (void) AGC_ASSERT(ip != NULL && es != NULL);
    if (agc_dp_sub(ip, es->d, AGC_ENTRY_C21) >= 0)
    {
        agc_flag_set(AGC_FLAG_NOSWITCH); /* suppress lat switch */
    }
    if (agc_dp_sub(ip, es->v, es->v1) >= 0)
    {
        return ST_DOWNCNTL;
    }
    if (agc_dp_sub(ip, es->d, es->q7) < 0)
    {
        return ST_KEP;
    }
    if (es->rdot >= 0)
    {
        return ST_CONT1;
    }
    m = agc_dp_sub(ip, agc_dp_sub(ip, es->v, es->vl), AGC_ENTRY_C18);
    (void) AGC_ASSERT(es->vl > 0);
    return (m < 0) ? ST_PREFINAL : ST_CONT1; /* VLTEST            */
}

/* CONT1 (p. 864): L/D = LAD while D >= A0.                                */
static entry_step_t step_cont1(entry_pass_t *p)
{
    (void) AGC_ASSERT(p->es != NULL);
    (void) AGC_ASSERT(p->es->a0 > 0);
    if (agc_dp_sub(p->ip, p->es->d, p->es->a0) < 0)
    {
        return ST_CONT3;
    }
    p->es->l_d = p->es->lad; /* STOREL/D          */
    return ST_LIMITLD;
}

/* CONT3 (pp. 864-865): VREF = FACT1 (1 - SQRT(FACT2 D + ALP)),
 * RDOTREF = LEWD (V1 - VREF) - RDHOOK, with
 * RDHOOK = CH1 (1 + DV AHOOKDV/DVL) DV^2/(DHOOK VREF), DV = VS1 - VREF.  */
static entry_step_t step_cont3(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           m;
    agc_dp_t           dv;

    (void) AGC_ASSERT(es->dhook != 0 && es->dvl != 0);
    m           = agc_dp_add(ip, agc_dp_mul(ip, es->d, es->fact2), es->alp);
    m           = agc_dp_sub(ip, AGC_ENTRY_NEARONE, agc_dp_sqrt(ip, m));
    es->vref    = agc_dp_mul(ip, m, es->fact1);
    es->rdotref = agc_dp_mul(ip, agc_dp_sub(ip, es->v1, es->vref), es->lewd);
    dv          = agc_dp_sub(ip, es->vs1, es->vref);
    if (dv < 0)
    {
        return ST_CONTINU2;
    }
    (void) AGC_ASSERT(es->vref > 0);
    m = agc_dp_div(ip, agc_dp_mul(ip, dv, es->ahookdv), es->dvl);
    m = agc_dp_mul(ip, agc_dp_add(ip, m, AGC_ENTRY_ONE_16TH), AGC_ENTRY_CH1);
    m = agc_dp_mul(ip, agc_dp_mul(ip, m, dv), dv);
    m = agc_dp_div(ip, agc_dp_div(ip, m, es->dhook), es->vref);
    es->rdotref = agc_dp_sub(ip, es->rdotref, m); /* - RDHOOK          */
    return ST_CONTINU2;
}

/* CONTINU2 (p. 865): FACTOR = (D - Q7)/(A1 - Q7) when D >= Q7MIN.        */
static entry_step_t step_continu2(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           m  = agc_dp_sub(ip, es->d, AGC_ENTRY_KA4); /* Q7MIN */

    (void) AGC_ASSERT(ip != NULL);
    (void) agc_interp_test_ovf(ip); /* BOVB TCDANZIG     */
    if (m < 0)
    {
        return ST_UPCNTRL3;
    }
    (void) AGC_ASSERT(es->a1 != es->q7);
    es->factor = agc_dp_div(ip, agc_dp_sub(ip, es->d, es->q7),
                            agc_dp_sub(ip, es->a1, es->q7));
    return ST_UPCNTRL3;
}

/* UPCNTRL3 / NEXT1 (p. 866): delta L/D =
 *   -((RDOT - RDOTREF) F1/KB1 + V - VREF) F1/KB2, F1 = FACTOR,
 * with the nonlinear gain reduction above .1, L/D = LEWD + delta.         */
static entry_step_t step_upcntrl3(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           dld;
    agc_dp_t           m;

    (void) AGC_ASSERT(ip != NULL && es != NULL);
    m   = agc_dp_mul(ip, agc_dp_sub(ip, es->rdot, es->rdotref), es->factor);
    m   = agc_dp_add(ip, agc_dp_div(ip, m, AGC_ENTRY_ONE_OVER_KB1), es->v);
    m   = agc_dp_mul(ip, agc_dp_sub(ip, m, es->vref), es->factor);
    dld = agc_dp_div(ip, m, AGC_ENTRY_MINUS_ONE_OVER_KB2); /* /16       */
    if (agc_interp_test_ovf(ip))
    {
        p->mpac = dld;
        return ST_GOMAXLD;
    }
    m = agc_dp_sub(ip, agc_dp_abs(dld), AGC_ENTRY_PT1_16);
    if (m >= 0)
    {
        m   = agc_dp_add(ip, agc_dp_mul(ip, m, AGC_ENTRY_POINT1),
                         AGC_ENTRY_PT1_16);
        dld = agc_dp_sign(m, dld);
    }
    p->mpac = agc_dp_add(ip, agc_dp_shl(ip, dld, 4), es->lewd); /* NEXT1 */
    return ST_NEGTESTS;
}

/* NEGTESTS (p. 866): L/D store; above C20 (175 fpss) LATSW is cleared
 * (roll over the top) and a negative L/D is replaced by 0.               */
static entry_step_t step_negtests(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;

    (void) AGC_ASSERT(ip != NULL && es != NULL);
    if (agc_interp_test_ovf(ip))
    {
        return ST_GOMAXLD; /* MPAC = overflowed L/D */
    }
    es->l_d = p->mpac;
    if (agc_dp_sub(ip, es->d, AGC_ENTRY_C20) < 0)
    {
        return ST_LIMITLD;
    }
    agc_flag_clear(AGC_FLAG_LATSW);
    if (p->mpac < 0)
    {
        es->l_d = 0;
    }
    return ST_LIMITLD;
}

/* ======================================================================
 * CONSTANT DRAG (pp. 867-868)
 * ==================================================================== */

/* DCONSTD (p. 867): the two RANGER entries into CONSTD.                  */
static entry_step_t step_dconstd(entry_pass_t *p)
{
    (void) AGC_ASSERT(p->es != NULL);
    p->es->diffold = p->es->diff;
    p->es->q7      = AGC_ENTRY_Q7F;
    (void) AGC_ASSERT(p->es->q7 > 0);
    return ST_BECONSTD;
}

/* BECONSTD (p. 867): GOTOADDR = HUNTEST, RTB KILLGRP4.                   */
static entry_step_t step_beconstd(entry_pass_t *p)
{
    (void) AGC_ASSERT(p->es != NULL);
    p->es->gotoaddr = AGC_ENTRY_GOTO_HUNTEST;
    if (entry_killgrp4() != AGC_STATUS_OK)
    {
        p->status = AGC_STATUS_BAD_ARG;
        return ST_ERROR;
    }
    (void) AGC_ASSERT(p->es->gotoaddr == AGC_ENTRY_GOTO_HUNTEST);
    return ST_CONSTD;
}

/* CONSTD (p. 867): RDOTREF = -2 HS D0/V,
 * L/D = (C/D0 LEQ + K2D (RDOT - RDOTREF) + K1D (D - D0)) 2^8.            */
static entry_step_t step_constd(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           m;

    (void) AGC_ASSERT(es->v > 0);
    (void) agc_interp_test_ovf(ip); /* BOVB TCDANZIG    */
    m        = agc_dp_mul(ip, AGC_ENTRY_TWO_HS, es->d0);
    m        = agc_dp_add(ip, agc_dp_div(ip, m, es->v), es->rdot);
    m        = agc_dp_mul(ip, m, AGC_ENTRY_K2D);
    p->pd_ld = agc_dp_add(ip, m, agc_dp_mul(ip, es->leq, es->c_d0));
    p->mpac  = es->d0; /* DREF = D0         */
    (void) AGC_ASSERT(es->d0 > 0);
    return ST_CONSTD1;
}

/* CONSTD1 (p. 867): L/D = (K1D (D - DREF) + push) SL 8, then NEGTESTS.   */
static entry_step_t step_constd1(entry_pass_t *p)
{
    agc_interp_t *ip = p->ip;
    agc_dp_t      m;

    (void) AGC_ASSERT(ip != NULL);
    (void) AGC_ASSERT(p->es != NULL);
    m       = agc_dp_sub(ip, p->es->d, p->mpac); /* BDSU D           */
    m       = agc_dp_add(ip, agc_dp_mul(ip, m, AGC_ENTRY_K1D), p->pd_ld);
    p->mpac = agc_dp_shl(ip, m, 8);
    return ST_NEGTESTS;
}

/* DOWNCNTL (pp. 867-868): initial part of up control (V > V1):
 *   RDTR = LAD (V1 - V), push = LAD/256 + K2D (RDOT - RDTR),
 *   DREF = (V/V1)^2 A0 - (V - V1)^2 LAD/(2 C1 HS).                       */
static entry_step_t step_downcntl(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           m;
    agc_dp_t           q;

    (void) AGC_ASSERT(es->v1 != 0);
    (void) agc_interp_test_ovf(ip); /* BOVB TCDANZIG    */
    m        = agc_dp_mul(ip, agc_dp_sub(ip, es->v, es->v1), es->lad);
    m        = agc_dp_mul(ip, agc_dp_add(ip, m, es->rdot), AGC_ENTRY_K2D);
    p->pd_ld = agc_dp_add(ip, m, agc_dp_shr(es->lad, 8));
    q = agc_dp_mul(ip, agc_dp_sq(ip, agc_dp_sub(ip, es->v1, es->v)), es->lad);
    q = agc_dp_div(ip, q, AGC_ENTRY_TWO_C1_HS);
    m = agc_dp_div(ip, agc_dp_sq(ip, es->v1), es->vsquare);
    m = agc_dp_div(ip, es->a0, m);  /* BDDV A0           */
    p->mpac = agc_dp_sub(ip, m, q); /* DREF              */
    (void) AGC_ASSERT(es->vsquare > 0);
    return ST_CONSTD1;
}

/* ======================================================================
 * BALLISTIC PHASE (p. 869), MM 66
 * ==================================================================== */

/* KEP (p. 869): RTB P66, GOTOADDR = KEP2.                                */
static entry_step_t step_kep(entry_pass_t *p)
{
    (void) AGC_ASSERT(p->es != NULL);
    if (entry_ballistic(p->es) != AGC_STATUS_OK)
    {
        p->status = AGC_STATUS_BAD_ARG;
        return ST_ERROR;
    }
    p->es->gotoaddr = AGC_ENTRY_GOTO_KEP2;
    (void) AGC_ASSERT(p->es->gotoaddr == AGC_ENTRY_GOTO_KEP2);
    return ST_KEP2;
}

/* KEP2 (p. 869): final phase once D > Q7F + KDMIN; otherwise keep (or,
 * below .05 g, zero) ROLLC and ROLLHOLD and compute the trim gimbal
 * angles (P62.3 -> S62.3, then ENDEXIT).                                  */
static entry_step_t step_kep2(entry_pass_t *p)
{
    agc_entry_state_t *es = p->es;

    (void) AGC_ASSERT(p->ip != NULL);
    if (agc_dp_sub(p->ip, AGC_ENTRY_Q7FKDMIN, es->d) < 0)
    {
        return ST_PREFINAL;
    }
    if (!agc_flag_test(AGC_FLAG__05GSW))
    {
        es->rollc    = 0; /* TLOAD 3ZEROS: ROLLC, ROLLHOLD */
        es->rollhold = 0;
    }
    if (agc_entry_desired_gimbal_angles() != AGC_STATUS_OK)
    {
        p->status = AGC_STATUS_BAD_ARG;
        return ST_ERROR;
    }
    (void) AGC_ASSERT(es->gotoaddr == AGC_ENTRY_GOTO_KEP2);
    return ST_ENDEXIT; /* QPRET = ENDEXIT            */
}

/* ======================================================================
 * FINAL PHASE (pp. 870-872), MM 67
 * ==================================================================== */

/* PREFINAL (p. 870): RTB P67, SET EGSW, GOTOADDR = PREDICT3.             */
static entry_step_t step_prefinal(entry_pass_t *p)
{
    (void) AGC_ASSERT(p->es != NULL);
    p->es->gotoaddr = AGC_ENTRY_GOTO_PREFINAL; /* restart protection */
    if (entry_final_phase(p->es) != AGC_STATUS_OK)
    {
        p->status = AGC_STATUS_BAD_ARG;
        return ST_ERROR;
    }
    agc_flag_set(AGC_FLAG_EGSW);
    p->es->gotoaddr = AGC_ENTRY_GOTO_PREDICT3;
    (void) AGC_ASSERT(agc_flag_test(AGC_FLAG_EGSW));
    return ST_PREDICT3;
}

/* BACK (p. 870): search VREFER for VREF < V from the top (JJ = 12) and
 * return V - VREF (SP, positive); CCS semantics of the AGC loop kept.     */
static agc_sp_t predict_find_jj(agc_entry_state_t *es, agc_sp_t v_sp)
{
    int i;
    int jj = 12;

    (void) AGC_ASSERT(es != NULL);
    for (i = 0; i < 2 * AGC_ENTRY_VREFER_N; i++)
    {
        int d = (int) agc_entry_vrefer(0, jj) - (int) v_sp; /* VREF - V   */
        if (d < 0)
        {
            es->jj = jj;
            return (agc_sp_t) (-d); /* TEM1B      */
        }
        if (d == 0 || jj == 0)
        { /* CCS A = +0: TCF BACK with A = 0;
                                         JJ = +0 cannot occur (VQUIT)      */
            if (jj == 0)
            {
                es->jj = 0;
                return 1;
            }
            jj = 0;
        }
        else
        {
            jj--;
        }
    }
    es->jj = 0;
    (void) AGC_ASSERT(false); /* the table always brackets V      */
    return 1;
}

/* BACK2 (pp. 870-871): GRAD and the six interpolated functions
 * FX+5..FX+0 = F1, F2, RDOTREF, RTOGO, -AREF, F3 (SP).                    */
static void predict_interp(agc_entry_state_t *es, agc_sp_t tem1b)
{
    int      mm;
    int      jj = es->jj;
    agc_sp_t dk =
        (agc_sp_t) (agc_entry_vrefer(0, jj + 1) - agc_entry_vrefer(0, jj));

    (void) AGC_ASSERT(dk > tem1b && tem1b > 0);
    /* GRAD = (V - VREF)/(V(K+1) - V(K)), DV of (A=tem1b, L=0).            */
    es->grad = (agc_sp_t) (((int32_t) tem1b * 16384) / (int32_t) dk);
    for (mm = 5; mm >= 0; mm--)
    {
        int      t   = 6 - mm; /* JJ + 13 (5 - mm + 1)          */
        agc_sp_t ak  = agc_entry_vrefer(t, jj);
        agc_sp_t ak1 = agc_entry_vrefer(t, jj + 1);
        agc_dp_t pr  = agc_sp_mul((agc_sp_t) (ak1 - ak), es->grad);
        es->fx[mm]   = (agc_sp_t) (agc_dp_high(pr) + ak); /* AK + GRAD dA */
    }
    (void) AGC_ASSERT(es->jj >= 0 && es->jj < 12);
}

/* PREDICT3 table part (pp. 870-871): PREDANG = RTOGO + F1 (D - DREF)
 * + F2 (RDOTREF - RDOT) 8, as SP fixed-point (MP / DAS); returns the DP
 * MPAC the AGC passes on to the interpreter.                              */
static agc_dp_t predict_predang(agc_entry_state_t *es)
{
    agc_ovf_t ovf = 0;
    agc_dp_t  mpac;
    agc_dp_t  r;
    agc_sp_t  a;

    (void) AGC_ASSERT(es != NULL);
    a         = (agc_sp_t) (es->fx[1] + agc_dp_high(es->d)); /* D - DREF     */
    es->fx[1] = 0;                                           /* XCH FX +1    */
    mpac      = agc_sp_mul(a, es->fx[5]);                    /* F1 (D-DREF)  */
    r    = agc_dp_shl_raw(agc_dp_neg(es->rdot), 3, &ovf);    /* DCS, DDOUBLx3 */
    a    = (agc_sp_t) (agc_dp_high(r) + es->fx[3]);          /* + RDOTREF    */
    r    = agc_sp_mul(a, es->fx[4]);                         /* x F2         */
    r    = agc_dp_add_raw(r, agc_dp_from_sp(es->fx[2]), &ovf); /* + RTOGO   */
    mpac = agc_dp_add_raw(mpac, r, &ovf); /* DAS MPAC     */
    es->predang = agc_dp_high(mpac);
    (void) AGC_ASSERT(ovf >= -1 && ovf <= 1);
    return mpac;
}

/* PREDICT3 (pp. 870-871): stop steering below VQUIT, else range error
 * DNRNGERR = PREDANG - THETAH (revs), or GONEGLAD once past the target.  */
static entry_step_t step_predict3(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           m;

    (void) AGC_ASSERT(ip != NULL);
    if (agc_dp_sub(ip, es->v, AGC_ENTRY_VQUIT) < 0)
    {
        return ST_STEEROFF;
    }
    predict_interp(es, predict_find_jj(es, agc_dp_high(es->v)));
    m = predict_predang(es);
    m = agc_dp_sub(ip, agc_dp_shr(m, 3), es->thetah); /* SR3 DSU      */
    if (agc_flag_test(AGC_FLAG_GONEPAST))
    {
        return ST_GONEGLAD;
    }
    if (!agc_flag_test(AGC_FLAG_GONEBY))
    {
        p->mpac = m;
        return ST_HAVDNRNG;
    }
    es->dnrngerr = AGC_ENTRY_MAXRNG; /* 9999.9 NM   */
    agc_flag_set(AGC_FLAG_GONEPAST);
    (void) AGC_ASSERT(agc_flag_test(AGC_FLAG_GONEPAST));
    return ST_GONEGLAD;
}

/* HAVDNRNG (pp. 871-872): L/D = LOD + (THETAH - PREDANG)/(DR/D L/D) 2^5. */
static entry_step_t step_havdnrng(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           m;

    (void) AGC_ASSERT(ip != NULL);
    es->dnrngerr = p->mpac;
    m            = agc_dp_neg(p->mpac); /* short if neg, long if pos */
    (void) agc_interp_test_ovf(ip);     /* BOVB TCDANZIG              */
    (void) AGC_ASSERT(es->fx[0] != 0);
    m = agc_dp_div(ip, m, agc_dp_pack(es->fx[0], es->fx[1])); /* DDV FX    */
    m = agc_dp_shl(ip, m, 5);
    if (!agc_interp_test_ovf(ip))
    {
        m = agc_dp_add(ip, m, es->lod);
    }
    if (agc_interp_test_ovf(ip))
    {
        p->mpac = m;
        return ST_GOMAXLD;
    }
    es->l_d = m;
    return ST_GLIMITER;
}

/* GONEGLAD (p. 872): L/D = -LAD ("DLOAD GONEGLAD" loads any negative). */
static entry_step_t step_gonegladd(entry_pass_t *p)
{
    (void) AGC_ASSERT(p != NULL);
    p->mpac = -1;
    (void) AGC_ASSERT(p->mpac < 0);
    return ST_GOMAXLD;
}

/* GOMAXL/D (p. 872): L/D = LAD SIGN(MPAC), then GLIMITER.                */
static entry_step_t step_gomaxld(entry_pass_t *p)
{
    (void) AGC_ASSERT(p->es != NULL);
    p->es->l_d = agc_dp_mul(p->ip, entry_signmpac(p->mpac), p->es->lad);
    (void) AGC_ASSERT(agc_dp_abs(p->es->l_d) <= p->es->lad);
    return ST_GLIMITER;
}

/* GLIMITER (p. 872): above GMAX/2 limit the downlift so that
 *   RDOT + SQRT(2HS (GMAX - D)(LEQ/GMAX + LAD) + (2HS GMAX/V)^2) >= 0.   */
static entry_step_t step_glimiter(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           m;
    agc_dp_t           x;

    (void) AGC_ASSERT(es->vsquare > 0);
    m = agc_dp_sub(ip, AGC_ENTRY_GMAX_HALF, es->d);
    if (m >= 0)
    {
        return ST_LIMITLD;
    }
    m = agc_dp_add(ip, m, AGC_ENTRY_GMAX_HALF); /* GMAX - D     */
    if (m < 0)
    {
        return ST_GOPOSLAD;
    }
    m = agc_dp_mul(ip, m, AGC_ENTRY_TWO_HS);
    x = agc_dp_add(ip, agc_dp_mul(ip, es->leq, AGC_ENTRY_HALVE), es->lad);
    m = agc_dp_mul(ip, x, m);
    x = agc_dp_div(ip, AGC_ENTRY_TWO_HS_GMAX_SQ, es->vsquare);
    m = agc_dp_sqrt(ip, agc_dp_add(ip, x, m)); /* XLIM         */
    (void) AGC_ASSERT(m >= 0);
    return (agc_dp_add(ip, m, es->rdot) >= 0) ? ST_LIMITLD : ST_GOPOSLAD;
}

/* GOPOSLAD (p. 872): L/D = LAD.                                          */
static entry_step_t step_goposlad(entry_pass_t *p)
{
    (void) AGC_ASSERT(p->es != NULL);
    p->es->l_d = p->es->lad;
    (void) AGC_ASSERT(p->es->l_d >= 0);
    return ST_LIMITLD;
}

/* ======================================================================
 * LATERAL LOGIC (pp. 873-875)
 * ==================================================================== */

/* LIMITL/D / L350 (p. 873): lateral switching corridor
 * Y = KLAT VSQUARE + LATBIAS (halved when |L/D| >= L/DCMINR).            */
static entry_step_t step_limitld(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;

    (void) AGC_ASSERT(ip != NULL && es != NULL);
    es->l_d1 = es->l_d;
    if (agc_flag_test(AGC_FLAG_GONEPAST))
    {
        return ST_L355; /* no lateral past target     */
    }
    p->pd_y = agc_dp_add(ip, agc_dp_mul(ip, es->vsquare, es->klat),
                         AGC_ENTRY_LATBIAS);
    if (agc_dp_sub(ip, agc_dp_abs(es->l_d), es->l_dcminr) < 0)
    {
        return ST_L353;
    }
    if (agc_dp_sign(es->latang, es->k2roll) < 0)
    {
        return ST_L357;
    }
    p->pd_y = agc_dp_shr(p->pd_y, 1); /* Y = Y/2                    */
    return ST_L353;
}

/* L353 (p. 873): switch K2ROLL when LATANG SIGN(K2ROLL) - Y >= 0, unless
 * NOSWITCH (BONCLR: tested and cleared).                                 */
static entry_step_t step_l353(entry_pass_t *p)
{
    agc_entry_state_t *es = p->es;
    agc_dp_t           m;

    (void) AGC_ASSERT(p->ip != NULL);
    m = agc_dp_sub(p->ip, agc_dp_sign(es->latang, es->k2roll), p->pd_y);
    if (m < 0)
    {
        return ST_L355;
    }
    if (agc_flag_test(AGC_FLAG_NOSWITCH))
    {
        agc_flag_clear(AGC_FLAG_NOSWITCH);
        return ST_L355;
    }
    es->k2roll = agc_dp_neg(es->k2roll); /* K2ROLL = -K2ROLL           */
    (void) AGC_ASSERT(es->k2roll != 0);
    return ST_L355;
}

/* L355 (p. 873): ROLLC = SIGN(K2ROLL) ACOS(L/D1 / LAD); clear NOSWITCH. */
static entry_step_t step_l355(entry_pass_t *p)
{
    agc_interp_t      *ip = p->ip;
    agc_entry_state_t *es = p->es;
    agc_dp_t           m;

    (void) AGC_ASSERT(es->lad > 0);
    m = agc_dp_div(ip, es->l_d1, es->lad); /* +-1 on overflow            */
    (void) agc_interp_test_ovf(ip);
    m         = agc_dp_acos(ip, agc_dp_shr(m, 1));
    es->rollc = agc_dp_sign(m, es->k2roll);
    agc_flag_clear(AGC_FLAG_NOSWITCH);
    (void) AGC_ASSERT(agc_dp_abs(es->rollc) <= AGC_DP_HALF);
    return ST_ENDEXIT;
}

/* L357 (p. 875): L/D1 = L/DCMINR SIGN(L/D).                              */
static entry_step_t step_l357(entry_pass_t *p)
{
    (void) AGC_ASSERT(p->es != NULL);
    p->es->l_d1 = agc_dp_sign(p->es->l_dcminr, p->es->l_d);
    (void) AGC_ASSERT(agc_dp_abs(p->es->l_d1) == p->es->l_dcminr);
    return ST_L355;
}

/* STEEROFF (p. 875): V < VQUIT. Start P67.1 (N67 flashing), restart 4.41
 * and 5.3, GOTOADDR = P67.2 and continue there.                           */
static entry_step_t step_steeroff(entry_pass_t *p)
{
    agc_phase_var_t v = {
        AGC_RESTART_JOB, AGC_JOB_REFAZE10, AGC_TASK_NONE, 0, false, 0U};

    (void) AGC_ASSERT(p->es != NULL);
    if (agc_exec_novac(AGC_ENTRY_PRIO16, AGC_JOB_ENTRY_FINAL_PHASE_DISPLAY) !=
        AGC_STATUS_OK)
    {
        p->status = AGC_STATUS_FULL;
        return ST_ERROR;
    }
    agc_phase_change2(000414, 010035, &v); /* 4.41 P67.1, SERVICER 5.3 */
    p->es->gotoaddr = AGC_ENTRY_GOTO_FINAL_PHASE_LATLONG;
    (void) AGC_ASSERT(p->es->gotoaddr == AGC_ENTRY_GOTO_FINAL_PHASE_LATLONG);
    return ST_FINAL_PHASE_LATLONG;
}

/* P67.2 (P61-P67.agc p. 802): present LAT, LONG, ALT (pad radius, earth),
 * then P67.3: SERVNOUT without display.                                   */
static entry_step_t step_final_phase_latlong(entry_pass_t *p)
{
    p->status = agc_entry_final_phase_latlong();
    (void) AGC_ASSERT(p->es != NULL);
    return (p->status == AGC_STATUS_OK) ? ST_SERVNOUT : ST_ERROR;
}

/* ======================================================================
 * SEQUENCER (p. 847)
 * ==================================================================== */

/* One labelled section; returns the next. Split in two switches to keep
 * each function short.                                                    */
static entry_step_t entry_step_a(entry_pass_t *p, entry_step_t s)
{
    switch (s)
    {
        case ST_INITROLL:
            return step_initroll(p);
        case ST_D0EQ:
            return step_d0eq(p);
        case ST_INROLOUT:
            return step_inrolout(p);
        case ST_KATEST:
            return step_katest(p);
        case ST_INITRL1:
            return step_initrl1(p);
        case ST_HUNTEST:
            return step_huntest(p);
        case ST_RANGER:
            return step_ranger(p);
        case ST_GETDLEWD:
            return step_getdlewd(p);
        case ST_SIDETRAK:
            return step_sidetrak(p);
        case ST_GOTOUPSY:
            return step_gotoupsy(p);
        case ST_UPCONTRL:
            return step_upcontrl(p);
        case ST_CONT1:
            return step_cont1(p);
        case ST_CONT3:
            return step_cont3(p);
        case ST_CONTINU2:
            return step_continu2(p);
        case ST_UPCNTRL3:
            return step_upcntrl3(p);
        case ST_NEGTESTS:
            return step_negtests(p);
        case ST_DCONSTD:
            return step_dconstd(p);
        case ST_BECONSTD:
            return step_beconstd(p);
        case ST_CONSTD:
            return step_constd(p);
        case ST_CONSTD1:
            return step_constd1(p);
        case ST_DOWNCNTL:
            return step_downcntl(p);
        case ST_KEP:
        case ST_KEP2:
        case ST_PREFINAL:
        case ST_PREDICT3:
        case ST_HAVDNRNG:
        case ST_GONEGLAD:
        case ST_GOMAXLD:
        case ST_GLIMITER:
        case ST_GOPOSLAD:
        case ST_LIMITLD:
        case ST_L353:
        case ST_L355:
        case ST_L357:
        case ST_STEEROFF:
        case ST_FINAL_PHASE_LATLONG:
        case ST_ENDEXIT:
        case ST_SERVNOUT:
        case ST_ERROR:
        default:
            break;
    }
    (void) AGC_ASSERT(s >= ST_KEP);
    return ST_ERROR;
}

static entry_step_t entry_step_b(entry_pass_t *p, entry_step_t s)
{
    switch (s)
    {
        case ST_KEP:
            return step_kep(p);
        case ST_KEP2:
            return step_kep2(p);
        case ST_PREFINAL:
            return step_prefinal(p);
        case ST_PREDICT3:
            return step_predict3(p);
        case ST_HAVDNRNG:
            return step_havdnrng(p);
        case ST_GONEGLAD:
            return step_gonegladd(p);
        case ST_GOMAXLD:
            return step_gomaxld(p);
        case ST_GLIMITER:
            return step_glimiter(p);
        case ST_GOPOSLAD:
            return step_goposlad(p);
        case ST_LIMITLD:
            return step_limitld(p);
        case ST_L353:
            return step_l353(p);
        case ST_L355:
            return step_l355(p);
        case ST_L357:
            return step_l357(p);
        case ST_STEEROFF:
            return step_steeroff(p);
        case ST_FINAL_PHASE_LATLONG:
            return step_final_phase_latlong(p);
        case ST_INITROLL:
        case ST_D0EQ:
        case ST_INROLOUT:
        case ST_KATEST:
        case ST_INITRL1:
        case ST_HUNTEST:
        case ST_RANGER:
        case ST_GETDLEWD:
        case ST_SIDETRAK:
        case ST_GOTOUPSY:
        case ST_UPCONTRL:
        case ST_CONT1:
        case ST_CONT3:
        case ST_CONTINU2:
        case ST_UPCNTRL3:
        case ST_NEGTESTS:
        case ST_DCONSTD:
        case ST_BECONSTD:
        case ST_CONSTD:
        case ST_CONSTD1:
        case ST_DOWNCNTL:
        case ST_ENDEXIT:
        case ST_SERVNOUT:
        case ST_ERROR:
        default:
            break;
    }
    (void) AGC_ASSERT(s < ST_KEP);
    return ST_ERROR;
}

/* GOTO GOTOADDR (p. 847): first step of the pass.                        */
static entry_step_t entry_first_step(agc_entry_goto_t g)
{
    static const entry_step_t map[AGC_ENTRY_GOTO_COUNT] = {
        ST_INITROLL, ST_HUNTEST,  ST_UPCONTRL, ST_KEP2,
        ST_PREFINAL, ST_PREDICT3, ST_ENDEXIT,  ST_FINAL_PHASE_LATLONG};

    if (!AGC_ASSERT(g >= AGC_ENTRY_GOTO_INITROLL && g < AGC_ENTRY_GOTO_COUNT))
    {
        return ST_ERROR;
    }
    (void) AGC_ASSERT(map[g] <= ST_ERROR);
    return map[g];
}

/* Run the chain of sections from `s` to a terminal step (bounded).       */
static entry_step_t entry_sequencer(entry_pass_t *p, entry_step_t s)
{
    int n;

    (void) AGC_ASSERT(p != NULL);
    for (n = 0; n < AGC_ENTRY_SEQ_MAX_STEPS; n++)
    {
        if (s == ST_ENDEXIT || s == ST_SERVNOUT || s == ST_ERROR)
        {
            return s;
        }
        s = (s < ST_KEP) ? entry_step_a(p, s) : entry_step_b(p, s);
    }
    (void) AGC_ASSERT(false); /* step bound exhausted      */
    return ST_ERROR;
}

/* ENDEXIT / OVERNOUT / NODISKY (pp. 873-874): the entry display ENTRYVN
 * (REGODSPR, no abort if the DSKY is busy) when ENTRYDSP is set.          */
agc_status_t entry_endexit(agc_job_ctx_t *ctx, agc_entry_state_t *es)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!agc_flag_test(AGC_FLAG_ENTRYDSP))
    {
        return AGC_STATUS_OK; /* NODISKY          */
    }
    (void) agc_dsky_display(ctx, AGC_DSKY_REGODSPR,
                            agc_dsky_vn(es->entry_verb, es->entry_noun), NULL);
    return AGC_STATUS_OK; /* REGODSPR: never aborts here    */
}

/* REFAZE10 (p. 847): restart 5.3, then the steering phase named by
 * GOTOADDR. *display tells the caller whether the pass ends at ENDEXIT
 * (entry display) or at SERVNOUT (no display). Also the group 5 restart
 * point (the AGC's PHASCHNG OCT 10035 return).                            */
agc_status_t agc_entry_refaze10(bool *display)
{
    entry_pass_t    pass;
    entry_step_t    last;
    agc_phase_var_t v = {
        AGC_RESTART_JOB, AGC_JOB_REFAZE10, AGC_TASK_NONE, 0, false, 0U};

    if (!AGC_ASSERT(display != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *display = false;
    agc_phase_change_var(010035, &v); /* SERVICER 5.3        */
    pass.ip        = entry_ip();
    pass.es        = agc_entry_state();
    pass.mpac      = 0;
    pass.pd_y      = 0;
    pass.pd_ld     = 0;
    pass.hunt_iter = 0;
    pass.status    = AGC_STATUS_OK;
    last = entry_sequencer(&pass, entry_first_step(pass.es->gotoaddr));
    if (last == ST_ERROR)
    {
        return (pass.status != AGC_STATUS_OK) ? pass.status :
                                                AGC_STATUS_BAD_ARG;
    }
    *display = (last == ST_ENDEXIT);
    (void) AGC_ASSERT(last == ST_ENDEXIT || last == ST_SERVNOUT);
    return AGC_STATUS_OK;
}

/* SCALEPOP (p. 847): TARGETNG, then REFAZE10.                            */
agc_status_t agc_entry_scalepop(bool *display)
{
    agc_status_t st;

    if (!AGC_ASSERT(display != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *display = false;
    st       = agc_entry_targeting();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    (void) AGC_ASSERT(agc_entry_state() != NULL);
    return agc_entry_refaze10(display);
}

/* agc_entry_huntest_predict -- HUNTEST without sequencing (tests).       */
agc_status_t agc_entry_huntest_predict(agc_entry_hunt_result_t *result,
                                       agc_dp_t                *gammal)
{
    agc_entry_state_t *es = agc_entry_state();

    if (!AGC_ASSERT(result != NULL) || !AGC_ASSERT(gammal != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(es->v > 0 && es->vsquare > 0 && es->lewd > 0 &&
                    es->lad > 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *gammal = 0;
    *result = entry_huntest_core(entry_ip(), es, gammal);
    return AGC_STATUS_OK;
}

/* agc_entry_range_predict -- RANGER on the state left by HUNTEST (tests). */
agc_status_t agc_entry_range_predict(agc_dp_t gammal, agc_dp_t *diff)
{
    agc_entry_state_t *es = agc_entry_state();

    if (!AGC_ASSERT(diff != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(es->a0 > 0 && es->gammal1 != 0 && es->vbars > 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *diff    = entry_ranger_core(entry_ip(), es, gammal);
    es->diff = *diff;
    return AGC_STATUS_OK;
}
