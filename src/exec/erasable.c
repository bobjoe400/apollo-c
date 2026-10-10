/*
 * erasable.c -- MODECADR (IMUCADR, OPTCADR, RADCADR), the
 *                    mode-switching wait words that STARTSB2 clears.
 *
 * Source: ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING ERASABLE" (both
 * vehicles; docs/modules/state-flags.md lists exec-sched as the owner).
 * The IMU, optics and radar mode-switching routines store here the job
 * that waits for a device (STALL) or the -1 / -0 good / bad outcome;
 * SVCT3 (waitlist.c) tests IMUCADR before starting drift compensation,
 * and STARTSB2 (fresh_start.c) zeroes all three.
 */
#include "agc/exec/exec.h"
#include "agc/core/assert.h"
#include "internal.h"

static agc_sp_t modecadr[AGC_MODECADR_N];   /* IMUCADR OPTCADR RADCADR */

agc_sp_t agc_erase_modecadr(int dev)
{
    if (!AGC_ASSERT(dev >= 0 && dev < AGC_MODECADR_N)) {
        return 0;
    }
    (void)AGC_ASSERT(modecadr[dev] >= AGC_SP_MIN);
    return modecadr[dev];
}

agc_status_t agc_erase_set_modecadr(int dev, agc_sp_t v)
{
    if (!AGC_ASSERT(dev >= 0 && dev < AGC_MODECADR_N)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX);
    modecadr[dev] = v;
    return AGC_STATUS_OK;
}

agc_sp_t agc_erase_imucadr(void)
{
    (void)AGC_ASSERT(AGC_MODECADR_N == 3);
    (void)AGC_ASSERT(modecadr[0] >= AGC_SP_MIN);
    return agc_erase_modecadr(0);
}

/* STARTSB2: TS IMUCADR / TS OPTCADR / TS RADCADR (+0).               */
void exec_clear_modecadr(void)
{
    (void)AGC_ASSERT(agc_interrupts_inhibited());
    for (int d = 0; d < AGC_MODECADR_N; d++) {
        modecadr[d] = 0;
    }
    (void)AGC_ASSERT(modecadr[0] == 0);
}
