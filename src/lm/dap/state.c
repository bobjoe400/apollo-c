/*
 * state.c -- LM DAP erasables, their shared accessors, and the DAP parts of
 * fresh start and restart.
 *
 * Sources: ERASABLE_ASSIGNMENTS.agc pp.105-136 (layout and scales),
 * FRESH_START_AND_RESTART.agc pp.211-213 (DOFSTRT1, DAP pad-load defaults)
 * and pp.220-221 (STARTSB1).
 */
#include "agc/agc_assert.h"
#include "agc/agc_channels.h"
#include "agc/agc_guid.h"
#include "agc/agc_lm_dap.h"
#include "agc/agc_lm_dap_internal.h"

/* Fresh-start constants (FRESH_START_AND_RESTART.agc p.218, CONTROLLED_
 * CONSTANTS FULLAPS; octal values checked in the yaYUL listing).         */
#define FS_RCSFLAGS  ((uint16_t)4)        /* FOUR: needles initialisation  */
#define FS_MAXDB     ((agc_sp_t)1820)     /* MAXDB 03434: 5 deg at 45      */
#define FS_BOOLSTRT  ((uint16_t)021312)   /* BOOLSTRT: DAPBOOLS            */
#define FS_STIKSTRT  ((agc_sp_t)13521)    /* STIKSTRT 32321: .825268       */
#define FS_RATESTRT  ((agc_sp_t)-218)     /* RATESTRT 77445                */
#define FS_FULLAPS   ((agc_sp_t)1263)     /* FULLAPS 02357: 5050 kg B16    */
#define FS_TRAP      ((agc_sp_t)-510)     /* 77001OCT: .14 deg at 4.5 deg  */
#define FS_KAOS      ((agc_sp_t)60)       /* 60DEC: 6 s alpha gain         */
#define FS_DKOMEGAN  ((agc_sp_t)10)       /* TEN: 1 s omega gain (docked)  */
#define FS_DKDB      ((agc_sp_t)128)      /* BIT8: docked DB 1.4 deg       */
#define DAPBOOLS_WORD 13                  /* DAPBOOLS = FLGWRD13           */

static lm_dap_t g_dap;

lm_dap_t *lm_dap_state(void)
{
    (void)AGC_ASSERT(g_dap.t5adr == LM_T5_DAPIDLER || g_dap.t5adr == LM_T5_PAXIS);
    (void)AGC_ASSERT(g_dap.retjadr <= LM_UV_MINRTN);
    return &g_dap;
}

/* STARTSB1 DAP part (p.220-221): done on every fresh start and restart.  */
agc_status_t agc_lm_dap_restart(void)
{
    lm_dap_t *d = lm_dap_state();
    d->rcsflags = (uint16_t)(d->rcsflags & ~LM_RCS_ACCSET); /* DAPIDLER calls 1/ACCS */
    d->t6next[0] = (agc_sp_t)LM_T6_POSMAX;   /* a pending T6RUPT is made     */
    agc_status_t st = agc_chan_and(013, LM_T6_POSMAX); /* ineffectual        */
    d->nxt6adr = 0;
    d->nextp = 0;
    agc_status_t fs = agc_flag_clear(AGC_FLAG_ACCOKFLG);  /* ACCSOKAY = 0    */
    agc_status_t ts = lm_dap_set_t5adr(LM_T5_DAPIDLER);
    (void)AGC_ASSERT(st == AGC_STATUS_OK && fs == AGC_STATUS_OK);
    return (ts != AGC_STATUS_OK) ? ts : st;
}

/* DOFSTRT1 DAP part (pp.211-213), then STARTSB1.                          */
agc_status_t agc_lm_dap_fresh_start(void)
{
    lm_dap_t *d = lm_dap_state();
    d->rcsflags = FS_RCSFLAGS;
    agc_guid_state()->lm.abdelv = 0;      /* ABDELV: DAP initialisation     */
    d->ch5mask = 0;
    d->ch6mask = 0;
    d->pvalvest = 0;                      /* RCS failure monitor           */
    agc_status_t s5 = agc_chan_write(005, 0);   /* turn off RCS jets       */
    agc_status_t s6 = agc_chan_write(006, 0);
    d->db = FS_MAXDB;
    agc_status_t sb = agc_flagword_set(DAPBOOLS_WORD, FS_BOOLSTRT);
    d->stiksens = FS_STIKSTRT;
    d->neg_ratedb = FS_RATESTRT;
    d->hiascent = FS_FULLAPS;
    d->dktrap = FS_TRAP;
    d->lmtrap = FS_TRAP;
    d->dkkaosn = FS_KAOS;
    d->lmkaosn = FS_KAOS;
    d->lmomegan = 0;                      /* unity gain                     */
    d->dkomegan = FS_DKOMEGAN;
    d->dkdb = FS_DKDB;
    if (!AGC_ASSERT(s5 == AGC_STATUS_OK && s6 == AGC_STATUS_OK) ||
        !AGC_ASSERT(sb == AGC_STATUS_OK)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_lm_dap_restart();
}

/* ---- accessors of DAP erasables shared with other modules -------------- */
uint16_t agc_erase_cduxd(void) { return lm_dap_state()->cduxd; }
uint16_t agc_erase_cduyd(void) { return lm_dap_state()->cduyd; }
uint16_t agc_erase_cduzd(void) { return lm_dap_state()->cduzd; }

void agc_erase_set_cduxd(uint16_t v)
{
    if (AGC_ASSERT(v <= 077777u)) { lm_dap_state()->cduxd = v; }
}
void agc_erase_set_cduyd(uint16_t v)
{
    if (AGC_ASSERT(v <= 077777u)) { lm_dap_state()->cduyd = v; }
}
void agc_erase_set_cduzd(uint16_t v)
{
    if (AGC_ASSERT(v <= 077777u)) { lm_dap_state()->cduzd = v; }
}

agc_sp_t agc_erase_delcdux(void) { return lm_dap_state()->delcdux; }
agc_sp_t agc_erase_delcduy(void) { return lm_dap_state()->delcduy; }
agc_sp_t agc_erase_delcduz(void) { return lm_dap_state()->delcduz; }
void agc_erase_set_delcdux(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX)) { lm_dap_state()->delcdux = v; }
}
void agc_erase_set_delcduy(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX)) { lm_dap_state()->delcduy = v; }
}
void agc_erase_set_delcduz(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX)) { lm_dap_state()->delcduz = v; }
}

agc_sp_t agc_erase_omegapd(void) { return lm_dap_state()->omegapd; }
agc_sp_t agc_erase_omegaqd(void) { return lm_dap_state()->omegaqd; }
agc_sp_t agc_erase_omegard(void) { return lm_dap_state()->omegard; }
void agc_erase_set_omegapd(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX)) { lm_dap_state()->omegapd = v; }
}
void agc_erase_set_omegaqd(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX)) { lm_dap_state()->omegaqd = v; }
}
void agc_erase_set_omegard(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX)) { lm_dap_state()->omegard = v; }
}

agc_sp_t agc_erase_delperor(void) { return lm_dap_state()->delperor; }
agc_sp_t agc_erase_delqeror(void) { return lm_dap_state()->delqeror; }
agc_sp_t agc_erase_delreror(void) { return lm_dap_state()->delreror; }
void agc_erase_set_delperor(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX)) { lm_dap_state()->delperor = v; }
}
void agc_erase_set_delqeror(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX)) { lm_dap_state()->delqeror = v; }
}
void agc_erase_set_delreror(agc_sp_t v)
{
    if (AGC_ASSERT(v >= LM_NEGMAX)) { lm_dap_state()->delreror = v; }
}

agc_sp_t agc_erase_omegap(void) { return lm_dap_state()->omegap; }
agc_sp_t agc_erase_omegaq(void) { return lm_dap_state()->omegaq; }
agc_sp_t agc_erase_omegar(void) { return lm_dap_state()->omegar; }

/* AOSQ/AOSR are DP word pairs at pi/2 (the minor word may disagree in
 * sign after DAS, as in the AGC); the accessor returns their value.      */
agc_dp_t agc_erase_aosq(void)
{
    const lm_dap_t *d = lm_dap_state();
    lm_dp_pair_t p = { d->aosq, d->aosq_lo };
    return (agc_dp_t)lm_dap_dpval(p);
}
agc_dp_t agc_erase_aosr(void)
{
    const lm_dap_t *d = lm_dap_state();
    lm_dp_pair_t p = { d->aosr, d->aosr_lo };
    return (agc_dp_t)lm_dap_dpval(p);
}
void agc_erase_set_aosq(agc_dp_t v)
{
    if (AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX)) {
        lm_dp_pair_t p = lm_dap_dpmake((int32_t)v);
        lm_dap_state()->aosq = p.hi;
        lm_dap_state()->aosq_lo = p.lo;
    }
}
void agc_erase_set_aosr(agc_dp_t v)
{
    if (AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX)) {
        lm_dp_pair_t p = lm_dap_dpmake((int32_t)v);
        lm_dap_state()->aosr = p.hi;
        lm_dap_state()->aosr_lo = p.lo;
    }
}

agc_sp_t agc_erase_alphaq(void) { return lm_dap_state()->alphaq; }
agc_sp_t agc_erase_alphar(void) { return lm_dap_state()->alphar; }
agc_sp_t agc_erase_perror(void) { return lm_dap_state()->perror; }
agc_sp_t agc_erase_qerror(void) { return lm_dap_state()->qerror; }
agc_sp_t agc_erase_rerror(void) { return lm_dap_state()->rerror; }
agc_sp_t agc_erase_1jacc(void)  { return lm_dap_state()->jacc[0]; }
agc_sp_t agc_erase_1jaccq(void) { return lm_dap_state()->jacc[1]; }
agc_sp_t agc_erase_1jaccr(void) { return lm_dap_state()->jacc[2]; }
agc_sp_t agc_erase_kq(void)     { return lm_dap_state()->kq; }

agc_sp_t agc_erase_db(void) { return lm_dap_state()->db; }
void agc_erase_set_db(agc_sp_t v)
{
    if (AGC_ASSERT(v >= 0)) { lm_dap_state()->db = v; }
}
agc_sp_t agc_erase_stiksens(void) { return lm_dap_state()->stiksens; }
void agc_erase_set_stiksens(agc_sp_t v)
{
    if (AGC_ASSERT(v >= 0)) { lm_dap_state()->stiksens = v; }
}
agc_sp_t agc_erase_neg_ratedb(void) { return lm_dap_state()->neg_ratedb; }
void agc_erase_set_neg_ratedb(agc_sp_t v)
{
    if (AGC_ASSERT(v <= 0)) { lm_dap_state()->neg_ratedb = v; }
}
agc_sp_t agc_erase_hiascent(void) { return lm_dap_state()->hiascent; }
void agc_erase_set_hiascent(agc_sp_t v)
{
    if (AGC_ASSERT(v >= 0)) { lm_dap_state()->hiascent = v; }
}
uint16_t agc_erase_ch5mask(void) { return lm_dap_state()->ch5mask; }
uint16_t agc_erase_ch6mask(void) { return lm_dap_state()->ch6mask; }

bool agc_lm_dap_running(void)
{
    const lm_dap_t *d = lm_dap_state();
    (void)AGC_ASSERT(d->t5adr <= LM_T5_PAXIS);
    return d->t5adr == LM_T5_PAXIS;
}
