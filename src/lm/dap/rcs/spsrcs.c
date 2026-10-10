/*
 * spsrcs.c -- SPSRCS, the RCS control law used when the CSM is docked
 * (SPS_BACK-UP_RCS_CONTROL.agc, Luminary 099 pp.1507-1510), called from
 * PURGENCY (P axis) and TJLAW (U, V axes) instead of TJETLAW.
 *
 * Per axis (AXISCTR -1 = P, 0 = U, 1 = V), from E (at pi) and EDOT (at
 * pi/4 rad/s): fire against rates beyond 1.73 deg/s; outside the pad-loaded
 * deadband (pi/DKDB, 1.4 deg at fresh start) fire toward zero error unless
 * already returning faster than 1.125 deg/s; keep firing until the rate
 * passes the 0.10 deg/s target (zero target in drifting flight). A jet
 * shut-off or reversal starts the jet inhibition counter (UTIME: 4 DAP
 * passes for P, 10 for U and V) during which new firings are inhibited,
 * so the docked structure is not excited.
 *
 * TJx is not a time here: +HALF / -HALF fire for the whole pass, 0 is off.
 */
#include "agc/core/assert.h"
#include "agc/lm/dap.h"
#include "../internal.h"

#define SR_HALF      ((agc_sp_t)8192)  /* HALF                            */
#define SR_RATELIM1  ((agc_sp_t)032)   /* CALLCODE 00032: 1.73 deg/s      */
#define SR_RATELIM2  ((agc_sp_t)0632)  /* RATELIM2 00632: 1.125 deg/s     */
#define SR_RATEDB1   ((agc_sp_t)045)   /* TBUILDFX 00045: 0.101 deg/s     */

/* UTIME -1, UTIME, UTIME +1 (p.1509): inhibition passes for P, U, V.     */
static const agc_sp_t utime[3] = { 4, 012, 012 };

/* SPSSTART (pp.1509-1510): the jet command for this pass.                */
static agc_sp_t spsstart(const lm_dap_t *d)
{
    agc_sp_t outer = lm_dap_mphi(d->edot, SR_RATELIM1);
    (void)AGC_ASSERT(d->dkdb >= 0);
    if (outer > 0) {
        return (agc_sp_t)-SR_HALF;                    /* NEGTHRST           */
    }
    if (outer < 0) {
        return SR_HALF;                               /* POSTHRST           */
    }
    agc_sp_t ez = lm_dap_mphi(lm_dap_wrap((int32_t)d->edot + d->e), d->dkdb);
    if (ez == 0) {
        return 0;                                     /* TJZERO: coast zone */
    }
    if (ez > 0) {
        return ((int32_t)d->edot + SR_RATELIM2 <= 0) ? 0 : (agc_sp_t)-SR_HALF;
    }
    (void)AGC_ASSERT(ez < 0);
    return ((int32_t)d->edot - SR_RATELIM2 <= 0) ? SR_HALF : 0;
}

/* POSTHRST +3 .. ZAPTJ (pp.1508-1509): compare the new command with the old
 * sense; a stop or reversal sets the inhibition counter and zeroes TJ,
 * and from jets-off a running counter inhibits firing.                   */
static void postcheck(lm_dap_t *d, int i, agc_sp_t oldsense)
{
    agc_sp_t tj = d->tj[i];
    (void)AGC_ASSERT(i >= 0 && i <= 2);
    if (oldsense != 0) {
        bool same = (oldsense > 0) ? (tj > 0) : (tj < 0);
        if (!same) {
            d->jetctr[i] = utime[i];                  /* SETCTR             */
            d->tj[i] = 0;                             /* ZAPTJ              */
        }
        return;
    }
    /* CTRCHECK: a nonzero counter inhibits (the CCS < 0 branch also
     * reaches ZAPTJ; the counter is never negative).                     */
    if (d->jetctr[i] != 0) {
        d->tj[i] = 0;
    }
    (void)AGC_ASSERT(d->jetctr[i] >= 0);
}

/* SPSRCS (p.1509): jet sense test, rate deadband test, SPSSTART.        */
agc_status_t lm_dap_spsrcs(void)
{
    lm_dap_t *d = lm_dap_state();
    if (!AGC_ASSERT(d->axisctr >= -1 && d->axisctr <= 1)) {
        return AGC_STATUS_BAD_ARG;
    }
    int i = d->axisctr + 1;
    agc_sp_t tj = d->tj[i];
    agc_sp_t oldsense = (agc_sp_t)((tj > 0) ? 1 : ((tj < 0) ? -1 : 0));
    bool restart_law = true;
    if (oldsense != 0) {                              /* PLUSFIRE / NEGFIRE */
        agc_sp_t target = agc_flag_test(AGC_FLAG_DRIFTDFL) ? 0 : SR_RATEDB1;
        agc_sp_t rate = (agc_sp_t)((oldsense > 0) ? -d->edot : d->edot);
        restart_law = ((int32_t)target + rate <= 0);  /* target passed      */
    }
    if (restart_law) {
        d->tj[i] = spsstart(d);
    }
    postcheck(d, i, oldsense);
    (void)AGC_ASSERT(d->tj[i] >= -SR_HALF && d->tj[i] <= SR_HALF);
    return AGC_STATUS_OK;
}
