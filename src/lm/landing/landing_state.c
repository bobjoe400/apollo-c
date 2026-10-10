/*
 * landing_state.c -- guid-landing lane: erasable storage shared by the
 * landing, abort, ascent, throttle and display files, plus the few
 * basic-language idioms (PHASCHNG, FASTCHNG, DV, MP, AD) they all use.
 *
 * The AGC keeps these quantities in EBANK 5/6/7 erasables (ERASABLE_
 * ASSIGNMENTS.agc pp. 121-122 and the "unswitched for lunar landing"
 * overlays). They are lane-private: other lanes reach them only through
 * the accessors below, never as extern globals.
 */
#include <string.h>
#include "agc/agc_assert.h"
#include "agc/agc_channels.h"
#include "agc/agc_exec.h"
#include "agc/agc_lm_landing.h"
#include "agc/agc_state.h"

static agc_lm_pad_t lm_pad;   /* W-matrix overlay pad loads, EBANK 5     */
static agc_lm_ldg_t lm_ldg;   /* landing guidance erasables, EBANK 7     */
static agc_lm_thr_t lm_thr;   /* throttle erasables, EBANK 7             */
static agc_lm_asc_t lm_asc;   /* ascent guidance erasables, EBANK 7      */

/* Group 3 phase LUNLAND establishes (OCT 05023, priority OCT 21000).      */
#define LM_FASTCHNG_CODE 05023u

agc_lm_pad_t *agc_lm_pad(void) { return &lm_pad; }
agc_lm_ldg_t *agc_lm_ldg(void) { return &lm_ldg; }
agc_lm_thr_t *agc_lm_thr(void) { return &lm_thr; }
agc_lm_asc_t *agc_lm_asc(void) { return &lm_asc; }

/* Fresh start of the lane's erasables (the AGC leaves them as they were;
 * this is for tests and the sim harness). */
void agc_lm_landing_reset(void)
{
    (void)memset(&lm_pad, 0, sizeof lm_pad);  /* return value is &lm_pad */
    (void)memset(&lm_ldg, 0, sizeof lm_ldg);
    (void)memset(&lm_thr, 0, sizeof lm_thr);
    (void)memset(&lm_asc, 0, sizeof lm_asc);
    lm_ldg.wchphase = AGC_LM_PHASE_IGNALG;
}

/* PHASCHNG (PHASE_TABLE_MAINTENANCE.agc): the low octal digit of the code
 * word is the restart group; the full code is handed to the executive so
 * that the restart tables see the same phase the AGC would have stored. */
agc_status_t agc_lm_phaschng(uint16_t octal_code)
{
    int group = (int)(octal_code & 7u);
    if (!AGC_ASSERT(group >= 1 && group <= AGC_RESTART_GROUPS)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(octal_code <= 077777u)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_phase_change(group, octal_code);
}

/* FASTCHNG (LUNAR_LANDING_GUIDANCE_EQUATIONS.agc p. 826): specialised
 * PHASCHNG that stores the caller's address as the group 3 restart point.
 * DEVIATION: C cannot restart in the middle of a function, so group 3 is
 * re-armed at the LUNLAND phase; a restart repeats the guidance pass. */
agc_status_t agc_lm_fastchng(void)
{
    agc_status_t st = agc_phase_change(3, LM_FASTCHNG_CODE);
    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED)) {
        return st;
    }
    return AGC_STATUS_OK;
}

/* TIME1: low 14 bits of the centisecond clock (wraps every 163.84 s). */
agc_sp_t agc_lm_time1(void)
{
    uint32_t t = agc_time_now_cs();
    agc_sp_t t1 = (agc_sp_t)(t & 0x3FFFu);
    (void)AGC_ASSERT(t1 >= 0);
    return t1;
}

/* DV: divide the DP fraction (A,L) by the SP fraction in K. With
 * dividend v/2^28 and divisor d/2^14 the quotient word is v/d, truncated
 * toward zero as the hardware does. The AGC result is meaningless when
 * |dividend| >= |divisor|; we assert and saturate instead. */
agc_sp_t agc_lm_dv(agc_dp_t dividend, agc_sp_t divisor)
{
    if (!AGC_ASSERT(divisor != 0)) {
        return (dividend >= 0) ? AGC_SP_MAX : AGC_SP_MIN;
    }
    int32_t q = dividend / (int32_t)divisor;
    if (!AGC_ASSERT(q <= AGC_SP_MAX && q >= AGC_SP_MIN)) {
        q = (q > 0) ? AGC_SP_MAX : AGC_SP_MIN;
    }
    return (agc_sp_t)q;
}

/* MP, keeping the high word (what most basic-language callers use). */
agc_sp_t agc_lm_mp_hi(agc_sp_t a, agc_sp_t b)
{
    int32_t p = (int32_t)a * (int32_t)b;     /* DP product, 2^-28 units   */
    int32_t hi = p / 16384;                  /* truncate toward zero      */
    (void)AGC_ASSERT(hi <= AGC_SP_MAX && hi >= AGC_SP_MIN);
    return (agc_sp_t)hi;
}

/* AD with the overflow saturated (the callers in this lane never overflow
 * on valid data; the assertion flags it if they do). */
agc_sp_t agc_lm_ad(agc_sp_t a, agc_sp_t b)
{
    int32_t s = (int32_t)a + (int32_t)b;
    if (!AGC_ASSERT(s <= AGC_SP_MAX && s >= AGC_SP_MIN)) {
        s = (s > 0) ? AGC_SP_MAX : AGC_SP_MIN;
    }
    return (agc_sp_t)s;
}

/* Channels 30-33 are inverted: a signal is present when its bit is 0.    */
bool agc_lm_chan_bit_present(int chan, uint16_t bit)
{
    if (!AGC_ASSERT(chan >= AGC_CHAN_CHAN30 && chan <= AGC_CHAN_CHAN33)) {
        return false;
    }
    if (!AGC_ASSERT(bit != 0u)) {
        return false;
    }
    return (agc_chan_read(chan) & bit) == 0u;
}
