/*
 * dapidler.c -- DAPIDLER_PROGRAM.agc (Luminary 099 pp.1410-1413): the
 * T5RUPT program that idles at 10 per second until the crew wants the DAP
 * and the IMU and CDUs are usable, initialises the DAP (STARTDAP) and then
 * hands T5ADR to PAXIS; and the T5RUPT dispatch through T5ADR.
 */
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"
#include "../internal.h"

#define CH31_MODE_OFF  030000u        /* BIT13-14: mode select switch OFF  */
#define IMODES33_NOIMU 000040u        /* BIT6: IMU not usable              */
#define CH30_NOPGNCS   001000u        /* BIT10 of 30 (inverted): not PGNCS */
#define MANFLAG        03021u         /* RCSFLAGS bits cleared by STARTDAP */
#define BGIM23         07400u         /* trim gimbal drive bits 9-12, ch 12*/
#define PRIO27         ((agc_prio_t)027)
#define STARTDAP_SKIP  ((agc_sp_t)4)  /* FOUR: SKIPU/SKIPV not skipping    */
#define STARTDAP_TRAPS ((agc_sp_t)2)  /* TWO: NPTRAPS, NQTRAPS, NRTRAPS    */

/* SETTIME5 (p.1412): next T5RUPT in 100 ms (TIME5 = 37766).               */
static agc_status_t settime5(void)
{
    agc_status_t st = agc_rupt_set_time(5, LM_MS100);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(agc_rupt_time(5) == LM_MS100);
    return st;
}

/* SHUTDOWN (p.1413): T5ADR = DAPIDLER, kill jet requests, jets off, trim
 * gimbal off, and come back in 100 ms.                                    */
agc_status_t lm_dap_shutdown(void)
{
    lm_dap_t *d = lm_dap_state();
    agc_status_t st = lm_dap_set_t5adr(LM_T5_DAPIDLER);
    d->nextp = 0;
    d->nextu = 0;
    d->nextv = 0;
    agc_status_t s5 = agc_chan_write(005, 0);    /* command jets off        */
    agc_status_t s6 = agc_chan_write(006, 0);
    agc_status_t s12 = agc_chan_and(012, (uint16_t)~BGIM23 & 077777u);
    if (!AGC_ASSERT(s5 == AGC_STATUS_OK && s6 == AGC_STATUS_OK) ||
        !AGC_ASSERT(s12 == AGC_STATUS_OK)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t t5 = settime5();
    return (st != AGC_STATUS_OK) ? st : t5;
}

/* MOREIDLE (p.1413): keep computing the attitude errors for the displays,
 * then SHUTDOWN.                                                          */
agc_status_t lm_dap_moreidle(void)
{
    agc_status_t sq = lm_dap_qerrcalc();
    agc_status_t sp = lm_dap_calcperr();
    (void)AGC_ASSERT(sq == AGC_STATUS_OK);
    (void)AGC_ASSERT(sp == AGC_STATUS_OK);
    return lm_dap_shutdown();
}

/* CHEKBITS / CHEKMORE (p.1410). Returns true ("RETURN") when the DAP is to
 * stay on; otherwise the idle path (MOREIDLE or SHUTDOWN) has been run and
 * the interrupt ends. On the way the attitude-error display is serviced
 * (JUMPDSP -> ALTDSPLY -> RETNMORE -> CHEKMORE).                         */
bool lm_dap_chekbits(void)
{
    lm_dap_t *d = lm_dap_state();
    uint16_t ch31 = agc_chan_read(031);
    if (((uint16_t)~ch31 & CH31_MODE_OFF) == 0u) {   /* mode switch OFF     */
        (void)lm_dap_moreidle();      /* status reported by its own asserts  */
        return false;
    }
    if ((agc_erase_imodes33() & IMODES33_NOIMU) != 0u) {
        d->rcsflags = (uint16_t)(d->rcsflags | LM_RCS_NEEDLE3); /* IMU not usable */
        (void)lm_dap_shutdown();
        return false;
    }
    agc_status_t st = lm_dap_altdsply();             /* JUMPDSP             */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    if ((agc_chan_read(030) & CH30_NOPGNCS) != 0u) { /* CHEKMORE            */
        (void)lm_dap_moreidle();
        return false;
    }
    (void)AGC_ASSERT(d->t5adr <= LM_T5_PAXIS);
    return true;
}

/* STARTDAP (pp.1411-1412), first half: zero attitude error, rates, offset
 * estimates, trim-gimbal and minimum-impulse state.                      */
static void startdap_zero(lm_dap_t *d)
{
    for (int i = 0; i < 3; i++) {
        d->tj[i] = 0;
        d->jetctr[i] = 0;             /* docked jet inhibition counters     */
    }
    d->omegap = 0;
    d->omegaq = 0;
    d->omegar = 0;
    d->trapedp = 0;
    d->trapedq = 0;
    d->trapedr = 0;
    d->aosq = 0;
    d->aosq_lo = 0;
    d->aosr = 0;
    d->aosr_lo = 0;
    d->alphaq = 0;
    d->alphar = 0;
    d->neguq = 0;
    d->negur = 0;
    d->aosqterm = 0;
    d->aosrterm = 0;
    d->qaccdot = 0;
    d->raccdot = 0;
    d->allowgts = 0;
    d->cotroler = 0;                  /* do TRYGTS on first pass            */
    d->ingts = 0;
    d->qgimtimr = 0;                  /* stop gimbal drives                 */
    d->rgimtimr = 0;
    d->oldpmin = 0;
    d->oldqrmin = 0;
    (void)AGC_ASSERT(d->tj[0] == 0 && d->aosq == 0);
    (void)AGC_ASSERT(d->cotroler == 0);
}

/* STARTDAP (pp.1411-1413), second half: old CDUs, skip and T6 list
 * initialisation, then T5ADR = PAXIS.                                    */
agc_status_t lm_dap_startdap(void)
{
    lm_dap_t *d = lm_dap_state();
    agc_status_t st = agc_lm_dap_zatteror();
    startdap_zero(d);
    d->rcsflags = (uint16_t)(d->rcsflags & ~MANFLAG);
    d->oldxforp = agc_erase_cdux();
    d->oldyforp = agc_erase_cduy();
    d->oldzforq = agc_erase_cduz();
    d->rcsflags = (uint16_t)(d->rcsflags | LM_RCS_PSKIP);
    d->skip[0] = STARTDAP_SKIP;
    d->skip[1] = STARTDAP_SKIP;
    agc_status_t s6 = agc_rupt_set_time(6, LM_T6_POSMAX);
    d->t6next[0] = (agc_sp_t)LM_T6_POSMAX;
    d->t6furtha[0] = (agc_sp_t)LM_T6_POSMAX;
    d->t6next[1] = 0;
    d->t6furtha[1] = 0;
    d->nxt6adr = 0;
    d->nextp = 0;
    d->nextu = 0;
    d->nextv = 0;
    d->dapzrupt = LM_DAPZ_NEG;        /* jask not in progress               */
    d->nptraps = STARTDAP_TRAPS;
    d->nqtraps = STARTDAP_TRAPS;
    d->nrtraps = STARTDAP_TRAPS;
    agc_status_t s5 = lm_dap_set_t5adr(LM_T5_PAXIS);
    if (!AGC_ASSERT(st == AGC_STATUS_OK && s6 == AGC_STATUS_OK) ||
        !AGC_ASSERT(s5 == AGC_STATUS_OK)) {
        return AGC_STATUS_BAD_ARG;
    }
    return settime5();
}

/* DAPIDLER (p.1411): T5RUPT entry while the DAP is idling.               */
agc_status_t agc_task_dapidler(void)
{
    lm_dap_t *d = lm_dap_state();
    (void)AGC_ASSERT(d->t5adr == LM_T5_DAPIDLER);
    if ((d->rcsflags & LM_RCS_ACCSET) == 0u) {   /* 1/ACCSET since restart? */
        d->rcsflags = (uint16_t)(d->rcsflags | LM_RCS_ACCSET);
        agc_status_t sj = agc_exec_novac(PRIO27, AGC_JOB_JET_ACCEL_RESET);
        (void)AGC_ASSERT(sj != AGC_STATUS_BAD_ARG);
    }
    if (!lm_dap_chekbits()) {                    /* CHECKUP                 */
        return AGC_STATUS_OK;
    }
    if (!agc_flag_test(AGC_FLAG_ACCOKFLG)) {     /* 1/ACCS not done: idle   */
        return lm_dap_moreidle();
    }
    return lm_dap_startdap();
}

/* T5RUPT through T5ADR (INTERRUPT_LEAD_INS: DXCH ARUPT, DTCB T5ADR).     */
agc_status_t agc_lm_dap_t5rupt(void)
{
    const lm_dap_t *d = lm_dap_state();
    if (!AGC_ASSERT(d->t5adr == LM_T5_DAPIDLER || d->t5adr == LM_T5_PAXIS)) {
        return AGC_STATUS_BAD_ARG;
    }
    return (d->t5adr == LM_T5_PAXIS) ? agc_task_paxis() : agc_task_dapidler();
}
