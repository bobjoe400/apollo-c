/**********************************************************************
 * FILE NAME: spsrcs.c
 *
 * PURPOSE:
 *   SPSRCS, the RCS control law used when the CSM is docked (SPS_BACK-
 *   UP_RCS_CONTROL.agc, Luminary 099 pp.1507-1510), called from PURGENCY
 *   (P axis) and TJLAW (U, V axes) instead of TJETLAW.
 *
 * AGC SOURCE:
 *   Luminary099 SPS_BACK-UP_RCS_CONTROL.agc pp. 1507-1510 (SPSRCS)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_flag_test               test a flag bit (flags.h)
 *   lm_dap_mphi                 A after MP (arith.c)
 *   lm_dap_state                DAP erasables (state.c)
 *   lm_dap_wrap                 overflow-corrected store, TS/ADS/XCH (arith.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. AGC_STATUS_BAD_ARG when an internal assertion fails.
 *
 * NOTES:
 *   Per axis (AXISCTR -1 = P, 0 = U, 1 = V), from E (at pi) and EDOT (at
 *   pi/4 rad/s): fire against rates beyond 1.73 deg/s; outside the pad-loaded
 *   deadband (pi/DKDB, 1.4 deg at fresh start) fire toward zero error unless
 *   already returning faster than 1.125 deg/s; keep firing until the rate
 *   passes the 0.10 deg/s target (zero target in drifting flight). A jet
 *   shut-off or reversal starts the jet inhibition counter (UTIME: 4 DAP
 *   passes for P, 10 for U and V) during which new firings are inhibited,
 *   so the docked structure is not excited.
 *
 *   TJx is not a time here: +HALF / -HALF fire for the whole pass, 0 is off.
 **********************************************************************/
#include "../internal.h"
#include "agc/core/assert.h"
#include "agc/lm/dap.h"

#define SR_HALF     ((agc_sp_t) 8192) /* HALF                            */
#define SR_RATELIM1 ((agc_sp_t) 032)  /* CALLCODE 00032: 1.73 deg/s      */
#define SR_RATELIM2 ((agc_sp_t) 0632) /* RATELIM2 00632: 1.125 deg/s     */
#define SR_RATEDB1  ((agc_sp_t) 045)  /* TBUILDFX 00045: 0.101 deg/s     */

/* UTIME -1, UTIME, UTIME +1 (p.1509): inhibition passes for P, U, V.     */
static const agc_sp_t utime[3] = {4, 012, 012};

/**********************************************************************
 * SPSSTART (pp.1509-1510): the jet command for this pass.
 *
 * FUNCTION NAME: spsstart
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  --------------------
 *   d         const lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *
 * RETURNS: agc_sp_t -- TJ command: +HALF, -HALF or 0
 **********************************************************************/
static agc_sp_t spsstart(const lm_dap_t *d)
{
    agc_sp_t outer = lm_dap_mphi(d->edot, SR_RATELIM1); /* EDOT x CALLCODE: nonzero beyond 1.73 deg/s; unscaled test word (sign only) */

    if (outer > 0)
    {
        return ((agc_sp_t) -SR_HALF); /* NEGTHRST           */
    }

    if (outer < 0)
    {
        return (SR_HALF); /* POSTHRST           */
    }

    agc_sp_t ez = lm_dap_mphi(lm_dap_wrap((int32_t) d->edot + d->e), d->dkdb); /* (EDOT + E) DKDB: zero inside the deadband; unscaled test word (sign only) */

    if (ez == 0)
    {
        return (0); /* TJZERO: coast zone */
    }

    if (ez > 0)
    {
        return (((int32_t) d->edot + SR_RATELIM2 <= 0) ? 0 : (agc_sp_t) -SR_HALF); /* returning faster than 1.125 deg/s: coast */
    }

    if (!AGC_ASSERT(ez < 0))
    {
        return (0); /* no valid result */
    }

    return (((int32_t) d->edot - SR_RATELIM2 <= 0) ? SR_HALF : 0); /* returning faster than 1.125 deg/s: coast */
}

/**********************************************************************
 * POSTHRST +3 .. ZAPTJ (pp.1508-1509): compare the new command with the old
 * sense; a stop or reversal sets the inhibition counter and zeroes TJ,
 * and from jets-off a running counter inhibits firing.
 *
 * FUNCTION NAME: postcheck
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *   i         int         I    axis index AXISCTR + 1: 0 P, 1 U, 2 V
 *   oldsense  agc_sp_t    I    OLDSENSE: sense of the firing before this pass
 *
 * RETURNS: none
 **********************************************************************/
static void postcheck(lm_dap_t *d, int i, agc_sp_t oldsense)
{
    agc_sp_t tj = d->tj[i]; /* TJ of the axis: +-HALF or 0, scaled 0.625 ms per count */

    if (oldsense != 0)
    {
        bool same = (oldsense > 0) ? (tj > 0) : (tj < 0); /* new command has the old sense */

        if (!same)
        {
            d->jetctr[i] = utime[i]; /* SETCTR             */
            d->tj[i]     = 0;        /* ZAPTJ              */
        }

        return;
    }

    /* CTRCHECK: a nonzero counter inhibits (the CCS < 0 branch also
     * reaches ZAPTJ; the counter is never negative).                     */
    if (d->jetctr[i] != 0)
    {
        d->tj[i] = 0;
    }
}

/**********************************************************************
 * SPSRCS (p.1509): jet sense test, rate deadband test, SPSSTART.
 *
 * FUNCTION NAME: lm_dap_spsrcs
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_spsrcs(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    int      i           = d->axisctr + 1; /* axis index AXISCTR + 1 */
    agc_sp_t tj          = d->tj[i];       /* TJ of the axis, scaled 0.625 ms per count */
    agc_sp_t oldsense    = 0;              /* OLDSENSE: sense of TJ, unscaled sign */
    bool     restart_law = true;           /* run SPSSTART this pass */

    if (tj > 0)
    {
        oldsense = 1;
    }
    else if (tj < 0)
    {
        oldsense = -1;
    }

    if (oldsense != 0)
    { /* PLUSFIRE / NEGFIRE */
        agc_sp_t target = agc_flag_test(AGC_FLAG_DRIFTDFL) ? 0 : SR_RATEDB1; /* 0 in drift, else 0.1 deg/s, scaled pi/4 rad/s */
        agc_sp_t rate   = (agc_sp_t) ((oldsense > 0) ? -d->edot : d->edot);  /* rate in the firing's sense, scaled pi/4 rad/s */

        restart_law = ((int32_t) target + rate <= 0); /* target passed      */
    }

    if (restart_law)
    {
        d->tj[i] = spsstart(d);
    }

    postcheck(d, i, oldsense);

    return (AGC_STATUS_OK);
} /* lm_dap_spsrcs */
