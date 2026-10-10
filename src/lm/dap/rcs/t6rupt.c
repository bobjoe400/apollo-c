/*
 * t6rupt.c -- the LM DAP jet-timing interrupt (T6-RUPT_PROGRAMS.agc,
 * Luminary 099 pp.1403-1405, Lowell G. Hull) and the jet list JTLST
 * (Q_R-AXIS_RCS_AUTOPILOT.agc pp.1456-1457).
 *
 * TIME6 counts down at 1600 per second (0.625 ms, "scaled at 10.24 s");
 * when it passes +0 to -0 a T6RUPT ends a jet firing. The DAP owns TIME6:
 * it is POSMAX when inactive, 1..240 when counting, -0 when a T6RUPT is
 * due. The list is exactly three long: TIME6 (first), T6NEXT (delta to the
 * second), T6FURTHA (delta to the third), each with an axis index
 * 0 = P (channel 6), 4 = U, 13 = V (channel 5 halves).
 *
 * TIME6 itself is kept by the executive (agc_rupt_time(6)); the DINC
 * pulses are applied by the sim through agc_lm_dap_time6_count().
 */
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"
#include "../internal.h"

#define T6_ENABLE   040000u   /* BIT15 of channel 13: TIME6 counting      */
#define T6_PRIO37   15872     /* PRIO37 = 37000: overflow test for > 511 */
#define T6_UBITS    000314u   /* 00314OCT: U-axis jets in channel 5      */
#define T6_IDX_P    0
#define T6_IDX_U    4
#define T6_IDX_V    13

/* TIME6 as a ones'-complement value (-0, the "rupt due" state, is 0).    */
agc_sp_t lm_dap_time6_value(void)
{
    uint16_t raw = agc_rupt_time(6);
    (void)AGC_ASSERT(raw <= LM_T6_NEGZERO);
    if (raw <= LM_T6_POSMAX) {
        return (agc_sp_t)raw;
    }
    agc_sp_t v = (agc_sp_t)((int32_t)raw - (int32_t)LM_T6_NEGZERO);
    (void)AGC_ASSERT(v <= 0);
    return v;
}

static agc_status_t set_time6(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= 0 && v <= LM_POSMAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_rupt_set_time(6, (uint16_t)v);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;
}

/* WRITEP / WRITEU / WRITEV (pp.1404-1405): P jets to channel 6; U or V
 * jets into their half of channel 5, keeping the other axis' jets.       */
agc_status_t lm_dap_write(agc_sp_t index, uint16_t jets)
{
    if (!AGC_ASSERT(jets <= 0377u)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (index == T6_IDX_P) {
        return agc_chan_write(006, jets);
    }
    uint16_t keep;
    if (index == T6_IDX_U) {
        keep = (uint16_t)(agc_chan_read(005) & (uint16_t)~T6_UBITS & 077777u);
    } else if (AGC_ASSERT(index == T6_IDX_V)) {
        keep = (uint16_t)(agc_chan_read(005) & T6_UBITS);
    } else {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_chan_write(005, (uint16_t)(keep + jets));  /* AD L            */
}

/* WRITEx -1: write the jets queued for after the firing (NEXTP/U/V).     */
agc_status_t lm_dap_write_next(agc_sp_t index)
{
    const lm_dap_t *d = lm_dap_state();
    (void)AGC_ASSERT(index == T6_IDX_P || index == T6_IDX_U || index == T6_IDX_V);
    uint16_t jets = (index == T6_IDX_P) ? d->nextp
                  : ((index == T6_IDX_U) ? d->nextu : d->nextv);
    return lm_dap_write(index, jets);
}

/* JTLST (pp.1456-1457): insert (T6FURTHA, T6FURTHA +1) -- a time from now
 * in T6 counts and an axis index -- into the three-long list.           */
agc_status_t lm_dap_jtlst(void)
{
    lm_dap_t *d = lm_dap_state();
    agc_sp_t t = d->t6furtha[0];
    agc_sp_t idx = d->t6furtha[1];
    agc_sp_t time6 = lm_dap_time6_value();
    (void)AGC_ASSERT(t > 0 && t <= LM_POSMAX);
    if ((int32_t)time6 - t > 0) {                        /* new first entry */
        d->t6furtha[0] = d->t6next[0];
        d->t6furtha[1] = d->t6next[1];
        d->t6next[0] = (agc_sp_t)(time6 - t);
        d->t6next[1] = d->nxt6adr;
        d->nxt6adr = idx;
        agc_status_t st = set_time6(t);
        agc_status_t so = agc_chan_or(013, T6_ENABLE);   /* TURNON         */
        return (st != AGC_STATUS_OK) ? st : so;
    }
    int32_t mid = (int32_t)time6 - t + d->t6next[0];     /* MIDORLST       */
    if (mid > 0) {
        d->t6furtha[0] = lm_dap_sat(mid);
        d->t6furtha[1] = d->t6next[1];
        d->t6next[0] = (agc_sp_t)(t - time6);
        d->t6next[1] = idx;
        return AGC_STATUS_OK;
    }
    d->t6furtha[0] = lm_dap_sat(-mid);                   /* LASTCHG        */
    (void)AGC_ASSERT(d->t6furtha[0] >= 0);
    return AGC_STATUS_OK;
}

/* T6JOB (p.1404): a T6RUPT is due. Shift the list up one, restart TIME6
 * if the next entry is a real time (<= 511 counts), then write the queued
 * jets of the axis whose firing just ended.                              */
static agc_status_t t6job(lm_dap_t *d)
{
    agc_status_t s13 = agc_chan_and(013, LM_T6_POSMAX);  /* disable clock  */
    agc_sp_t head = d->t6next[0];                        /* DXCH T6NEXT    */
    agc_sp_t done = d->nxt6adr;                          /* LXCH NXT6ADR   */
    d->nxt6adr = d->t6next[1];
    d->t6next[0] = d->t6furtha[0];                       /* DXCH T6FURTHA  */
    d->t6next[1] = d->t6furtha[1];
    d->t6furtha[0] = LM_POSMAX;
    d->t6furtha[1] = 0;
    agc_status_t st;
    if ((int32_t)head + T6_PRIO37 > LM_POSMAX) {         /* list empty     */
        st = set_time6(LM_POSMAX);
    } else {
        st = set_time6(head);                            /* ENABLET6       */
        agc_status_t so = agc_chan_or(013, T6_ENABLE);
        (void)AGC_ASSERT(so == AGC_STATUS_OK);
        if ((int32_t)d->t6next[0] + T6_PRIO37 > LM_POSMAX) {
            d->t6next[0] = LM_POSMAX;
        }
    }
    (void)AGC_ASSERT(s13 == AGC_STATUS_OK && st == AGC_STATUS_OK);
    return lm_dap_write_next(done);                      /* GOCH56         */
}

/* T6JOBCHK (p.1404): CCS TIME6. Counting: return. -0: a T6RUPT is due,
 * do T6JOB. +0 cannot occur (CCSHOLE). A negative non-zero TIME6 makes
 * the AGC loop on itself until a hardware restart (TC TRAP).            */
agc_status_t agc_lm_dap_t6jobchk(void)
{
    uint16_t raw = agc_rupt_time(6);
    if (raw > 0u && raw <= LM_T6_POSMAX) {
        return AGC_STATUS_OK;
    }
    if (raw == 0u) {
        return agc_ccshole();                /* CCSHOLE: POODOO 01103 */
    }
    if (!AGC_ASSERT(raw == LM_T6_NEGZERO)) {
        return AGC_STATUS_BAD_ARG;          /* DEVIATION: AGC hangs here */
    }
    return t6job(lm_dap_state());
}

/* DOT6RUPT (p.1405): the T6RUPT handler is T6JOBCHK, then RESUME.        */
agc_status_t agc_lm_dap_t6rupt(void)
{
    agc_status_t st = agc_lm_dap_t6jobchk();
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    (void)AGC_ASSERT(agc_rupt_time(6) != LM_T6_NEGZERO || st != AGC_STATUS_OK);
    return st;
}

/* Sim harness: apply `counts` DINC pulses to TIME6 while channel 13 bit
 * 15 enables it. The pulse after +0 makes -0 and requests the T6RUPT.   */
void agc_lm_dap_time6_count(uint16_t counts)
{
    uint16_t raw = agc_rupt_time(6);
    if ((agc_chan_read(013) & T6_ENABLE) == 0u || raw > LM_T6_POSMAX || counts == 0u) {
        return;                              /* idle, or rupt already due */
    }
    uint16_t next = (counts > raw) ? (uint16_t)LM_T6_NEGZERO : (uint16_t)(raw - counts);
    agc_status_t st = agc_rupt_set_time(6, next);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(next <= LM_T6_NEGZERO);
}

/* TIME6 = -0: a T6RUPT is waiting.                                       */
bool agc_lm_dap_t6_pending(void)
{
    uint16_t raw = agc_rupt_time(6);
    (void)AGC_ASSERT(raw <= LM_T6_NEGZERO);
    return raw == LM_T6_NEGZERO;
}
