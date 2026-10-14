/**********************************************************************
 * FILE NAME: rcsmon.c
 *
 * PURPOSE:
 *   RCSMON / RCSMONIT, the RCS failure monitor (RCS_FAILURE_MONITOR.agc,
 *   Luminary 099 pp.190-192), run by T4RUPT every 480 ms.
 *
 * AGC SOURCE:
 *   Luminary099 RCS_FAILURE_MONITOR.agc pp. 190-192 (RCSMON, RCSMONIT)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_chan_read               READ channel
 *   lm_dap_accfix               1/ACCFIX (accs.c)
 *   lm_dap_state                DAP erasables (state.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. The 1/ACCFIX status is passed through.
 *
 * NOTES:
 *   Channel 32 bits 1-8 (inverted) report the crew's jet-pair isolation
 *   valve switches. When a switch has changed since the last pass
 *   (PVALVEST), the highest-numbered changed bit is acted on: a closed
 *   valve sets the inhibit bits of its jets in CH5MASK / CH6MASK, an opened
 *   valve clears them, PVALVEST records the action, and 1/ACCJOB is set up
 *   so the TJETLAW switch curves reflect the new jet masks. One change is
 *   handled per pass; further changes are found on later passes.
 **********************************************************************/
#include "../internal.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"

#define RM_LOW8  0377u   /* LOW8: jet-fail bits of channel 32        */
#define RM_WORD  077777u /* 15-bit word (CS ZERO / RXOR = invert)    */
#define RM_NBITS 8       /* bits 1-8 of channel 32 */

/* 5FAILTAB, 6FAILTAB (p.192): channel 5 / 6 jet bits for channel 32 bits
 * 8, 7, ..., 1 (index L - 1 in the AGC).                                 */
static const uint16_t fail5[RM_NBITS] = {000040u, 000020u, 000100u, 000200u, 000010u, 000001u, 000004u, 000002u};
static const uint16_t fail6[RM_NBITS] = {000010u, 000020u, 000004u, 000200u, 000001u, 000002u, 000040u, 000100u};

/**********************************************************************
 * RCSMON (pp.191-192). Called from T4RUPT (RCSMONIT), exits by RESUME.
 *
 * FUNCTION NAME: agc_lm_dap_rcsmon
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the status of the
 *          routine it ends in (1/ACCFIX)
 **********************************************************************/
agc_status_t agc_lm_dap_rcsmon(void)
{
    lm_dap_t *d      = lm_dap_state();                                        /* DAP erasables */
    uint16_t  cur    = (uint16_t) ((RM_WORD ^ agc_chan_read(032)) & RM_LOW8); /* valve switch states (channel 32 inverted), low 8 bits */
    uint16_t  change = (uint16_t) ((cur ^ d->pvalvest) & RM_LOW8);            /* switches changed since PVALVEST */

    if (change == 0u)
    {
        return (AGC_STATUS_OK); /* no action required */
    }

    /* The DOUBLE / OVSK loop: index (0 = bit 8 .. 7 = bit 1) of the highest
     * set bit of a nonzero 8-bit change word.                                */
    int l = 0; /* index of the highest changed bit (0 = bit 8) */

    for (int i = 0; i < RM_NBITS; i++)
    {
        if ((change & (uint16_t) (0200u >> i)) != 0u)
        {
            l = i;
            break;
        }
    }

    uint16_t bit = (uint16_t) (0200u >> l); /* the changed switch bit */

    if ((d->pvalvest & bit) != 0u)
    { /* VOPENED            */
        d->ch5mask  = (uint16_t) (d->ch5mask & ~fail5[l]);
        d->ch6mask  = (uint16_t) (d->ch6mask & ~fail6[l]);
        d->pvalvest = (uint16_t) (d->pvalvest & ~bit);
    }
    else
    { /* valve just closed  */
        d->ch5mask  = (uint16_t) (d->ch5mask | fail5[l]);
        d->ch6mask  = (uint16_t) (d->ch6mask | fail6[l]);
        d->pvalvest = (uint16_t) (d->pvalvest | bit);
    }

    return (lm_dap_accfix()); /* 1/ACCFIX           */
} /* agc_lm_dap_rcsmon */
