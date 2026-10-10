/*
 * reentry.c -- CM entry guidance: REENTRY_CONTROL.agc (Comanche055,
 * pp. 844-881), "Colossus 2A" entry steering of GSOP AS-278 section 5.6.
 *
 * One call per 2 s SERVICER cycle (via CM/POSE -> POSEXIT) computes the
 * roll command ROLLC. The AGC chains interpretive sections with GOTO; here
 * each labelled section is a step function returning the next step, and
 * entry_sequencer() runs the chain under a fixed step bound (rule 2).
 * Values carried "in MPAC" or in the push-down list between labelled
 * sections travel in entry_pass_t.
 *
 * Scales: see agc_csm_entry.h and entry_lexicon.h. /2VS, /805, revs.
 */
#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_state.h"
#include "agc/agc_exec.h"
#include "agc/agc_csm_entry.h"
#include "entry_lexicon.h"
#include "entry_internal.h"
#include "entry_pending.h"

/* Interpreter state shared by the entry interpretive code (OVFIND).        */
static agc_interp_t entry_interp;
static bool         entry_interp_ready = false;

typedef enum entry_step {
    ST_INITROLL = 0, ST_D0EQ, ST_INROLOUT, ST_KATEST, ST_INITRL1,
    ST_HUNTEST, ST_RANGER, ST_GETDLEWD, ST_GOTOUPSY,
    ST_UPCONTRL, ST_CONT1, ST_CONT3, ST_CONTINU2, ST_UPCNTRL3, ST_NEGTESTS,
    ST_DCONSTD, ST_BECONSTD, ST_CONSTD, ST_CONSTD1, ST_DOWNCNTL,
    ST_KEP, ST_KEP2, ST_PREFINAL, ST_PREDICT3,
    ST_GONEGLAD, ST_GOMAXLD, ST_GLIMITER, ST_GOPOSLAD, ST_STORELD,
    ST_LIMITLD, ST_L353, ST_L355, ST_L357,
    ST_STEEROFF, ST_P67_2, ST_ENDEXIT, ST_SERVNOUT, ST_ERROR
} entry_step_t;

typedef struct entry_pass {
    agc_interp_t      *ip;
    agc_entry_state_t *es;
    agc_dp_t           mpac;      /* value "in MPAC" at a step entry       */
    agc_dp_t           pd0;       /* push-down cell carried between steps  */
    int                hunt_iter; /* HUNTEST iterations this pass          */
} entry_pass_t;

/* entry_ip -- the shared interpreter state for the entry lane.            */
agc_interp_t *entry_ip(void)
{
    if (!entry_interp_ready) {
        agc_interp_init(&entry_interp);
        entry_interp_ready = true;
    }
    (void)AGC_ASSERT(entry_interp_ready);
    return &entry_interp;
}

/* SIGNMPAC (RT8_OP_CODES.agc p. 1386): MPAC = +DPOSMAX if the high word of
 * MPAC is +0 or positive, else -DPOSMAX. DEVIATION: tested on the sign of
 * the whole DP value (no -0 / mixed-sign DP words in this model).          */
agc_dp_t entry_signmpac(agc_dp_t x)
{
    (void)AGC_ASSERT(x >= AGC_DP_MIN);
    (void)AGC_ASSERT(x <= AGC_DP_MAX);
    return (x >= 0) ? AGC_ENTRY_DPOSMAX : (agc_dp_t)(-AGC_ENTRY_DPOSMAX);
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

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(x > 0)) {
        return 0;
    }
    xn = agc_dp_norm(x, &shift);
    (void)AGC_ASSERT(shift >= 0 && shift <= AGC_DP_BITS);
    y = agc_dp_sub(ip, AGC_ENTRY_NEARONE, xn);      /* BDSU NEARLY1      */
    p = agc_dp_mul(ip, AGC_ENTRY_LOG_C3, y);        /* POLY, N-1 = 2:    */
    p = agc_dp_mul(ip, agc_dp_add(ip, p, AGC_ENTRY_LOG_C2), y); /* a3..a0 */
    p = agc_dp_mul(ip, agc_dp_add(ip, p, AGC_ENTRY_LOG_C1), y); /* a0 = 0 */
    /* + shift count x CLOG2/32 (SHORTMP of a small positive integer)      */
    return agc_dp_add(ip, p, (agc_dp_t)(AGC_ENTRY_CLOG2_32 * shift));
}

/* EARROT1 / EARROT2 (CSM_GEOMETRY.agc pp. 293-294): rotate RTINIT about
 * the polar axis UNITW through WIE DTEAROT:
 *   RT = UNIT(RTINIT + RTNORM (cos WT - 1) + RTEAST sin WT).
 * first = true is the EARROT1 entry (resolves RTEAST, RTNORM).            */
agc_status_t entry_earrot(agc_interp_t *ip, agc_entry_state_t *es, bool first)
{
    agc_dp_t  wt;
    agc_vec_t a;
    agc_vec_t b;
    int       days;

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(es != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (first) {
        es->rteast = agc_vec_cross(ip, agc_erase_unitw(), es->rtinit);
        es->rtnorm = agc_vec_cross(ip, es->rteast, agc_erase_unitw());
    }
    (void)agc_interp_test_ovf(ip);                    /* BOVB TCDANZIG  */
    wt = agc_dp_div(ip, es->dtearot, AGC_ENTRY_ONE_OVER_WIE); /* revs  */
    /* OVERADAY: remove whole sidereal days until the divide fits.        */
    for (days = 0; days < 32 && agc_interp_test_ovf(ip); days++) {
        agc_dp_t day = agc_dp_sign(AGC_ENTRY_ONE_OVER_WIE, es->dtearot);
        es->dtearot = agc_dp_sub(ip, es->dtearot, day);
        wt = agc_dp_div(ip, es->dtearot, AGC_ENTRY_ONE_OVER_WIE);
    }
    a = agc_vec_scale(ip, es->rtnorm,
                      agc_dp_sub(ip, agc_dp_cos(ip, wt), AGC_ENTRY_HALVE));
    b = agc_vec_scale(ip, es->rteast, agc_dp_sin(ip, wt));
    a = agc_vec_shl(ip, agc_vec_add(ip, b, a), 1);    /* VAD VSL1        */
    es->rt = agc_vec_unit(ip, agc_vec_add(ip, a, es->rtinit), NULL, NULL);
    return AGC_STATUS_OK;
}

/* ARCTRIG (INFLIGHT_ALIGNMENT_ROUTINES.agc p. 1357): angle in revs in
 * (-.5, .5] from sin/4 and cos/4.                                         */
agc_dp_t entry_arctrig(agc_interp_t *ip, agc_dp_t sinth4, agc_dp_t costh4)
{
    const agc_dp_t qtsn45 = (agc_dp_t)47459389; /* QTSN45 2DEC .1768 = sin45/4 */
    agc_dp_t theta;

    if (!AGC_ASSERT(ip != NULL)) {
        return 0;
    }
    (void)AGC_ASSERT(agc_dp_abs(sinth4) <= AGC_DP_QUARTER + 1);
    if (agc_dp_sub(ip, agc_dp_abs(sinth4), qtsn45) >= 0) {
        /* (45,135) or (-135,-45): X = ARCCOS(COS) with sign(SIN)          */
        theta = agc_dp_acos(ip, agc_dp_shl(ip, costh4, 1));
        return agc_dp_sign(theta, sinth4);
    }
    theta = agc_dp_asin(ip, agc_dp_shl(ip, sinth4, 1)); /* TRIG1        */
    if (costh4 >= 0) {
        return theta;                                   /* (-45,45)     */
    }
    /* TRIG2 (135,-135): X = .5 sign(SIN) - ARCSIN(SIN)                   */
    return agc_dp_sub(ip, agc_dp_sign(AGC_ENTRY_HALVE, sinth4), theta);
}

/* ======================================================================
 * TARGETING (pp. 848-852)
 * ==================================================================== */

/* TARGETNG part 1: GETVEL / GETUNITV (p. 848). Velocity (inertial, or
 * relative once RELVELSW is set), VSQUARE, LEQ, V, RDOT, D, UNI.          */
static agc_status_t targ_velocity(agc_interp_t *ip, agc_entry_state_t *es,
                                  const agc_state_vector_t *sv)
{
    agc_dp_t abval = 0;
    agc_dp_t vsq = 0;
    agc_dp_t dmag;

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(sv != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_flag_test(AGC_FLAG_RELVELSW)) {
        es->vel = agc_vec_neg(es->mvrel);       /* -VREL left by CM/POSE */
    } else {
        es->vel = agc_vec_scale(ip, sv->v, AGC_ENTRY_KVSCALE); /* V/2VS   */
    }
    es->unitv   = agc_vec_unit(ip, es->vel, &abval, &vsq); /* GETUNITV  */
    es->vsquare = vsq;                                 /* VSQ/4, 34D    */
    es->leq     = agc_dp_sub(ip, vsq, AGC_ENTRY_FOURTH); /* LEQ/4       */
    es->v       = abval;                               /* V/2VS, 36D    */
    es->rdot    = agc_dp_shl(ip, agc_vec_dot(ip, es->vel, agc_erase_unitr()), 1);
    dmag = agc_dp_mul(ip, agc_vec_abval(ip, agc_erase_delv()), AGC_ENTRY_KASCALE);
    dmag = agc_dp_shl(ip, dmag, 1);
    es->d = (dmag == 0) ? AGC_ENTRY_ONEBIT_DP : dmag;  /* SETMIND        */
    es->uni = agc_vec_unit(ip, agc_vec_cross(ip, es->vel, agc_erase_unitr()),
                           NULL, NULL);                /* UNI = UNIT(VxR) */
    (void)AGC_ASSERT(es->v >= 0);
    return AGC_STATUS_OK;
}

/* GETETA / SUBETA / SUBETA2 (p. 851): D predicted half a cycle ahead,
 *   D = D + D (-RDOT/HS - 2D/V) DT/2,
 * then the time-of-flight estimate to the target in MPAC (cs B28).        */
static agc_dp_t targ_geteta(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_dp_t t;
    agc_dp_t u;

    (void)AGC_ASSERT(ip != NULL);
    (void)AGC_ASSERT(es->v > 0);
    t = agc_dp_div(ip, es->rdot, AGC_ENTRY_MINUS_HSCALED);   /* -RDOT/HS */
    u = agc_dp_div(ip, agc_dp_mul(ip, es->d, AGC_ENTRY_MINUS_KSCALE), es->v);
    u = agc_dp_add(ip, u, t);
    es->d = agc_dp_add(ip, agc_dp_mul(ip, u, es->d), es->d);
    if (!agc_flag_test(AGC_FLAG_EGSW)) {
        return agc_dp_mul(ip, es->thetah, AGC_ENTRY_KTETA);  /* THETAH KTETA */
    }
    if (agc_dp_sub(ip, es->v, AGC_ENTRY_FOURTH) < 0) {       /* SUBETA: VMIN */
        (void)agc_flag_set(AGC_FLAG_RELVELSW);               /* SET RELVELSW */
    }
    return agc_dp_div(ip, agc_dp_mul(ip, es->thetah, AGC_ENTRY_KT1), es->v);
}

/* GETANGLE / TINYTHET / THETDONE (pp. 849, 852): THETAH = ARCCOS(RT.UNITR)
 * in revs; near zero the arccos is replaced by 32 sqrt(2 (1/4 - x)) KACOS.
 * RTGON67 = -THETAH before the target is passed, +THETAH after.           */
static void targ_range_angle(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_dp_t x = agc_vec_dot(ip, es->rt, agc_erase_unitr()); /* cos/4   */
    agc_dp_t m = agc_dp_sub(ip, x, AGC_ENTRY_NEAR1_4);

    (void)AGC_ASSERT(ip != NULL);
    if (m >= 0) {
        /* TINYTHET: MPAC - 1BITDP+1 = x - 1/4; ABS; SL 13D; SQRT; x KACOS */
        m = agc_dp_abs(agc_dp_sub(ip, m, AGC_ENTRY_ONEBIT_DP_P1));
        m = agc_dp_sqrt(ip, agc_dp_shl(ip, m, 13));
        es->thetah = agc_dp_mul(ip, m, AGC_ENTRY_KACOS);
    } else {
        m = agc_dp_shl(ip, agc_dp_add(ip, m, AGC_ENTRY_NEAR1_4), 1);
        es->thetah = agc_dp_acos(ip, m);                     /* THETAH/360 */
    }
    (void)AGC_ASSERT(es->thetah >= 0);
    es->rtgon67 = agc_flag_test(AGC_FLAG_GONEBY) ? es->thetah
                                                 : agc_dp_neg(es->thetah);
}

/* L/DCALC (p. 850): measured lift-to-drag from DELVREF and UXA,
 *   L/D = SQRT((|DV|^2 - (DV.UXA)^2) / (DV.UXA)^2), skipped on overflow
 * (OVFIND last cleared in EARROT2, as in the AGC).                        */
static void targ_ldcalc(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_vec_t dv = agc_erase_delvref();
    agc_dp_t  du = agc_dp_shl(ip, agc_vec_dot(ip, dv, es->uxa), 1);
    agc_dp_t  dsq = agc_dp_sq(ip, du);
    agc_dp_t  r;

    (void)AGC_ASSERT(ip != NULL);
    (void)AGC_ASSERT(dsq >= 0);
    r = agc_dp_div(ip, agc_dp_sub(ip, agc_vec_sq(ip, dv), dsq), dsq);
    if (!agc_interp_test_ovf(ip)) {
        es->l_dcalc = agc_dp_sqrt(ip, r);
    }
}

/* TARGETNG (pp. 848-850): called each pass by SCALEPOP (and by P61).
 * Updates the velocity data, D, the rotated target RT, LATANG, GONEBY,
 * THETAH, RTGON67, .05GSW and L/DCALC.                                    */
agc_status_t agc_entry_targeting(void)
{
    agc_interp_t      *ip = entry_ip();
    agc_entry_state_t *es = agc_entry_state();
    agc_state_vector_t sv;
    agc_status_t       st;
    agc_dp_t           mpac = 0;

    if (!AGC_ASSERT(es != NULL) || agc_sv_get(AGC_SV_THIS, &sv) != AGC_STATUS_OK) {
        return AGC_STATUS_BAD_ARG;
    }
    st = targ_velocity(ip, es, &sv);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (!agc_flag_test(AGC_FLAG_RELVELSW)) {
        mpac = targ_geteta(ip, es);             /* else 3ZEROS          */
    }
    /* UPDATERT: DTEAROT = estimate + (PIPTIME - TIME/RTO)                 */
    es->dtearot = agc_dp_add(ip, agc_dp_sub(ip, mpac, es->time_rto), sv.t);
    st = entry_earrot(ip, es, false);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    es->latang = agc_vec_dot(ip, es->rt, es->uni);           /* LATANG/4 */
    (void)agc_flag_clear(AGC_FLAG_GONEBY);
    if (agc_vec_dot(ip, agc_vec_cross(ip, es->rt, agc_erase_unitr()), es->uni) < 0) {
        (void)agc_flag_set(AGC_FLAG_GONEBY);   /* have gone past target  */
    }
    targ_range_angle(ip, es);
    if (agc_dp_sub(ip, es->d, AGC_ENTRY_G05) < 0) {
        (void)agc_flag_clear(AGC_FLAG__05GSW); /* NO.05G (for the DAP)   */
        return AGC_STATUS_OK;
    }
    (void)agc_flag_set(AGC_FLAG__05GSW);
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
    agc_state_vector_t sv;
    agc_vec_t          r;
    agc_status_t       st;

    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(es != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    st = agc_sv_get(AGC_SV_THIS, &sv);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    (void)agc_flag_clear(AGC_FLAG_ERADFLAG);   /* use pad radius        */
    (void)agc_flag_clear(AGC_FLAG_LUNAFLAG);   /* earth lat-long        */
    es->time_rto = sv.t;                       /* TIME/RTO = PIPTIME    */
    st = agc_nav_lalotorv(es->lat_spl, es->lng_spl, 0, sv.t, &r);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    es->rtinit  = agc_vec_unit(ip, r, NULL, NULL);   /* .5 unit target  */
    es->dtearot = AGC_ENTRY_SEC500;                  /* 500SEC nominal  */
    st = entry_earrot(ip, es, true);                 /* EARROT1 -> RT   */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    es->thetah = agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, es->rt,
                                                 agc_erase_unitr()), 1));
    return AGC_STATUS_OK;
}

/* STARTENT entry-switch initialisation (p. 844): clear ENTMASK = 011774
 * (ENTRYDSP GONEPAST RELVELSW EGSW NOSWITCH HIND INRLSW LATSW .05GSW), then
 * set ENTRYSW = 011010 (ENTRYDSP, GONEPAST, LATSW). INHINT'ed in the AGC. */
static void entry_init_switches(void)
{
    static const agc_flag_id_t clr[] = {
        AGC_FLAG_ENTRYDSP, AGC_FLAG_GONEPAST, AGC_FLAG_RELVELSW, AGC_FLAG_EGSW,
        AGC_FLAG_NOSWITCH, AGC_FLAG_HIND, AGC_FLAG_INRLSW, AGC_FLAG_LATSW,
        AGC_FLAG__05GSW
    };
    unsigned i;

    agc_inhint();
    for (i = 0; i < (sizeof clr / sizeof clr[0]); i++) {
        (void)agc_flag_clear(clr[i]);          /* flag API cannot fail on valid ids */
    }
    (void)agc_flag_set(AGC_FLAG_ENTRYDSP);
    (void)agc_flag_set(AGC_FLAG_GONEPAST);
    (void)agc_flag_set(AGC_FLAG_LATSW);
    agc_relint();
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_LATSW));
    (void)AGC_ASSERT(!agc_flag_test(AGC_FLAG_HIND));
}

/* STARTENT (pp. 844-845), MM 63: first pass of entry guidance, entered from
 * CM/POSE. Ends with SERVNOUT (no display on the first pass).             */
agc_status_t agc_entry_startent(void)
{
    agc_interp_t      *ip = entry_ip();
    agc_entry_state_t *es = agc_entry_state();
    agc_state_vector_t sv;
    agc_status_t       st;
    agc_dp_t           m;

    if (!AGC_ASSERT(es != NULL) || agc_sv_get(AGC_SV_THIS, &sv) != AGC_STATUS_OK) {
        return AGC_STATUS_BAD_ARG;
    }
    entry_init_switches();
    es->lod      = agc_dp_from_sp(es->lodpad);           /* SLOAD LODPAD */
    es->lad      = agc_dp_from_sp(es->ladpad);
    es->l_dcminr = agc_dp_mul(ip, es->lad, AGC_ENTRY_COS15);
    es->klat     = agc_dp_shr(agc_dp_mul(ip, AGC_ENTRY_ONE_12TH, es->lad), 1);
    es->q7       = AGC_ENTRY_Q7F;
    es->factor   = AGC_ENTRY_NEARONE;
    /* L/D = -LAD SGN(HEADSUP)                                             */
    es->l_d = agc_dp_neg(agc_dp_sign(es->lad, agc_dp_from_sp(es->headsup)));
    st = entry_starten1(ip, es);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    m = agc_vec_dot(ip, agc_vec_unit(ip, agc_vec_cross(ip, sv.v, agc_erase_unitr()),
                                     NULL, NULL), es->rt);
    es->latang = m;                                      /* UNI.RT /4    */
    es->k2roll = entry_signmpac(agc_dp_neg(m));          /* -SGN(LATANG) */
    es->q2 = agc_dp_add(ip, agc_dp_mul(ip, es->lad, AGC_ENTRY_Q21), AGC_ENTRY_Q22);
    es->gotoaddr = AGC_ENTRY_GOTO_INITROLL;
    es->posexit  = AGC_ENTRY_POSEXIT_SCALEPOP;
    return AGC_STATUS_OK;                                /* RTB SERVNOUT */
}
