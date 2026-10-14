/**********************************************************************
 * FILE NAME: modecadr.c
 *
 * PURPOSE:
 *   MODECADR (IMUCADR, OPTCADR, RADCADR): the mode-switching wait words
 *   that STARTSB2 clears and SVCT3 tests.
 *
 * AGC SOURCE:
 *   Comanche055 / Luminary099 ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING
 *   ERASABLE" (both vehicles; docs/modules/state-flags.md lists
 *   exec-sched as the owner)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none
 *
 * NOTES:
 *   The IMU, optics and radar mode-switching routines store here the job
 *   that waits for a device (STALL) or the -1 / -0 good / bad outcome;
 *   SVCT3 (waitlist.c) tests IMUCADR before starting drift compensation,
 *   and STARTSB2 (fresh_start.c) zeroes all three.
 **********************************************************************/
#include "agc/exec/exec.h"
#include "internal.h"

static agc_sp_t modecadr[AGC_MODECADR_N]; /* IMUCADR OPTCADR RADCADR, raw SP words (B0) */

/**********************************************************************
 * C(MODECADR +dev) (ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING ERASABLE").
 *
 * ARGUMENTS:
 *   dev -- 0 IMUCADR, 1 OPTCADR, 2 RADCADR (an agc_imu_device_t
 *          constant at every caller)
 *
 * RETURNS: agc_sp_t -- the word (+0 none, positive a waiting job)
 **********************************************************************/
agc_sp_t agc_erase_modecadr(int dev)
{
    return (modecadr[dev]);
}

/**********************************************************************
 * TS MODECADR +dev (ERASABLE_ASSIGNMENTS.agc "MODE SWITCHING ERASABLE").
 *
 * ARGUMENTS:
 *   dev -- 0 IMUCADR, 1 OPTCADR, 2 RADCADR (an agc_imu_device_t
 *          constant at every caller)
 *   v -- the word to store
 **********************************************************************/
void agc_erase_set_modecadr(int dev, agc_sp_t v)
{
    modecadr[dev] = v;
}

/**********************************************************************
 * C(IMUCADR) = MODECADR +0 (ERASABLE_ASSIGNMENTS.agc; read by SVCT3).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- the word (+0: nobody uses the IMU)
 **********************************************************************/
agc_sp_t agc_erase_imucadr(void)
{
    _Static_assert(AGC_MODECADR_N == 3, "AGC constant");

    return (agc_erase_modecadr(0));
}

/**********************************************************************
 * STARTSB2 (FRESH_START_AND_RESTART C p.191 / L p.221): TS IMUCADR /
 * TS OPTCADR / TS RADCADR (+0).
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_clear_modecadr(void)
{
    for (int d = 0; d < AGC_MODECADR_N; d++)
    {
        modecadr[d] = 0;
    }
}
