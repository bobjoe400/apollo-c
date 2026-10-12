/**********************************************************************
 * FILE NAME: t6rupt.c
 *
 * PURPOSE:
 *   The LM DAP jet-timing interrupt (T6-RUPT_PROGRAMS.agc, Luminary 099
 *   pp.1403-1405, Lowell G. Hull) and the jet list JTLST (Q_R-
 *   AXIS_RCS_AUTOPILOT.agc pp.1456-1457).
 *
 * AGC SOURCE:
 *   Luminary099 T6-RUPT_PROGRAMS.agc pp. 1403-1405 (T6JOB, T6JOBCHK,
 *     DOT6RUPT), Q_R-AXIS_RCS_AUTOPILOT.agc pp. 1456-1457 (JTLST, MIDORLST,
 *     LASTCHG)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_ccshole                 CCSHOLE: impossible CCS branch
 *   agc_chan_and                WAND channel
 *   agc_chan_or                 WOR channel
 *   agc_chan_read               READ channel
 *   agc_chan_write              WRITE channel
 *   agc_rupt_set_time           set TIME5 / TIME6
 *   agc_rupt_time               read TIME5 / TIME6
 *   lm_dap_sat                  OVERSUB / LIMITS (arith.c)
 *   lm_dap_state                DAP erasables (state.c)
 *   lm_dap_wrap                 overflow-corrected store, TS/ADS/XCH (arith.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   CCSHOLE (POODOO 01103) on an impossible axis index.
 *   AGC_STATUS_BAD_ARG when an internal assertion fails.
 *
 * NOTES:
 *   TIME6 counts down at 1600 per second (0.625 ms, "scaled at 10.24 s");
 *   when it passes +0 to -0 a T6RUPT ends a jet firing. The DAP owns TIME6:
 *   it is POSMAX when inactive, 1..240 when counting, -0 when a T6RUPT is
 *   due. The list is exactly three long: TIME6 (first), T6NEXT (delta to the
 *   second), T6FURTHA (delta to the third), each with an axis index
 *   0 = P (channel 6), 4 = U, 13 = V (channel 5 halves).
 *
 *   TIME6 itself is kept by the executive (agc_rupt_time(6)); the DINC
 *   pulses are applied by the sim through agc_lm_dap_time6_count().
 **********************************************************************/
#include "../internal.h"
#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"

#define T6_ENABLE 040000u /* BIT15 of channel 13: TIME6 counting      */
#define T6_PRIO37 15872   /* PRIO37 = 37000: overflow test for > 511 */
#define T6_UBITS  000314u /* 00314OCT: U-axis jets in channel 5      */
#define T6_IDX_P  0       /* P axis: channel 6 */
#define T6_IDX_U  4       /* U axis: channel 5 (WRITEP +4) */
#define T6_IDX_V  13      /* V axis: channel 5 (WRITEP +13) */

/**********************************************************************
 * TIME6 as a ones'-complement value (-0, the "rupt due" state, is 0).
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_time6_value
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- TIME6, scaled 0.625 ms (0 when -0)
 **********************************************************************/
agc_sp_t lm_dap_time6_value(void)
{
    uint16_t raw = agc_rupt_time(6); /* TIME6 register (15-bit ones' complement) */

    if (raw <= LM_T6_POSMAX)
    {
        return ((agc_sp_t) raw);
    }

    agc_sp_t v = (agc_sp_t) ((int32_t) raw - (int32_t) LM_T6_NEGZERO); /* negative TIME6 value, scaled 0.625 ms per count */

    return (v);
}

/**********************************************************************
 * TIME6 = v (DXCH / TS TIME6, pp.1404, 1456).  AGC: none (helper)
 *
 * FUNCTION NAME: set_time6
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   v         agc_sp_t  I    TIME6 value, scaled 0.625 ms
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t set_time6(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= 0 && v <= LM_POSMAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rupt_set_time(6, (uint16_t) v);

    if (!AGC_ASSERT(agc_rupt_time(6) == (uint16_t) v))
    {
        return (AGC_STATUS_BAD_ARG); /* TIME6 holds the new count */
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * WRITEP / WRITEU / WRITEV (pp.1404-1405): P jets to channel 6; U or V
 * jets into their half of channel 5, keeping the other axis' jets.
 *
 * FUNCTION NAME: lm_dap_write
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   index     agc_sp_t  I    axis index: 0 P (ch 6), 4 U, 13 V (ch 5)
 *   jets      uint16_t  I    jet bits for that axis
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_write(agc_sp_t index, uint16_t jets)
{
    if (!AGC_ASSERT(jets <= 0377u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (index == T6_IDX_P)
    {
        agc_chan_write(006, jets);
    }
    else
    {
        uint16_t keep; /* the other axis' channel 5 bits */

        if (index == T6_IDX_U)
        {
            keep = (uint16_t) (agc_chan_read(005) & (uint16_t) ~T6_UBITS & 077777u);
        }
        else if (AGC_ASSERT(index == T6_IDX_V))
        {
            keep = (uint16_t) (agc_chan_read(005) & T6_UBITS);
        }
        else
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_chan_write(005, (uint16_t) (keep + jets));
    }

    return (AGC_STATUS_OK); /* AD L            */
}

/**********************************************************************
 * WRITEx -1: write the jets queued for after the firing (NEXTP/U/V).
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_write_next
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   index     agc_sp_t  I    axis index: 0 P, 4 U, 13 V
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_write_next(agc_sp_t index)
{
    const lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    if (!AGC_ASSERT(index == T6_IDX_P || index == T6_IDX_U || index == T6_IDX_V))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t jets = d->nextv; /* NEXTV for the V index */

    if (index == T6_IDX_P)
    {
        jets = d->nextp;
    }
    else if (index == T6_IDX_U)
    {
        jets = d->nextu;
    }

    if (!AGC_ASSERT(jets <= 0377u))
    {
        return (AGC_STATUS_BAD_ARG); /* jet codes are 8 bits */
    }

    return (lm_dap_write(index, jets));
}

/**********************************************************************
 * JTLST (pp.1456-1457): insert (T6FURTHA, T6FURTHA +1) -- a time from now
 * in T6 counts and an axis index -- into the three-long list.
 *
 * FUNCTION NAME: lm_dap_jtlst
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_jtlst(void)
{
    lm_dap_t *d     = lm_dap_state();       /* DAP erasables */
    agc_sp_t  t     = d->t6furtha[0];       /* new firing time, scaled 0.625 ms per count */
    agc_sp_t  idx   = d->t6furtha[1];       /* its axis index (unscaled) */
    agc_sp_t  time6 = lm_dap_time6_value(); /* TIME6 now, scaled 0.625 ms per count */

    if (!AGC_ASSERT(t > 0 && t <= LM_POSMAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((int32_t) time6 - t > 0)
    { /* new first entry */
        d->t6furtha[0]  = d->t6next[0];
        d->t6furtha[1]  = d->t6next[1];
        d->t6next[0]    = (agc_sp_t) (time6 - t);
        d->t6next[1]    = d->nxt6adr;
        d->nxt6adr      = idx;
        agc_status_t st = set_time6(t); /* TIME6 = new time */

        agc_chan_or(013, T6_ENABLE); /* TURNON         */

        return (st);
    }

    int32_t mid = (int32_t) time6 - t + d->t6next[0]; /* MIDORLST       */

    if (mid > 0)
    {
        d->t6furtha[0] = lm_dap_wrap(mid);
        d->t6furtha[1] = d->t6next[1];
        d->t6next[0]   = (agc_sp_t) (t - time6);
        d->t6next[1]   = idx;
    }
    else
    {
        d->t6furtha[0] = lm_dap_sat(-mid); /* LASTCHG        */

        if (!AGC_ASSERT(d->t6furtha[0] >= 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (AGC_STATUS_OK);
} /* lm_dap_jtlst */

/**********************************************************************
 * T6JOBCHK (p.1404): CCS TIME6. Counting: return. -0: a T6RUPT is due,
 * do T6JOB. +0 cannot occur (CCSHOLE). A negative non-zero TIME6 makes
 * the AGC loop on itself until a hardware restart (TC TRAP).
 *
 * FUNCTION NAME: agc_lm_dap_t6jobchk
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_t6jobchk(void)
{
    uint16_t raw = agc_rupt_time(6); /* TIME6 register */

    if (raw > 0u && raw <= LM_T6_POSMAX)
    {
        return (AGC_STATUS_OK);
    }

    if (raw == 0u)
    {
        return (agc_ccshole()); /* CCSHOLE: POODOO 01103 */
    }

    if (!AGC_ASSERT(raw == LM_T6_NEGZERO))
    {
        return (AGC_STATUS_BAD_ARG); /* DEVIATION: AGC hangs here */
    }

    /* T6JOB (p.1404): a T6RUPT is due. Shift the list up one, restart TIME6
     * if the next entry is a real time (<= 511 counts), then write the queued
     * jets of the axis whose firing just ended. */
    lm_dap_t *d = lm_dap_state();

    agc_chan_and(013, LM_T6_POSMAX); /* disable clock  */
    agc_sp_t head = d->t6next[0];    /* DXCH T6NEXT: delta to the next entry, scaled 0.625 ms per count */
    agc_sp_t done = d->nxt6adr;      /* LXCH NXT6ADR: axis index whose firing ends now (unscaled) */

    if (!AGC_ASSERT(done == T6_IDX_P || done == T6_IDX_U || done == T6_IDX_V))
    {
        return (AGC_STATUS_BAD_ARG); /* NXT6ADR is an axis index */
    }

    d->nxt6adr     = d->t6next[1];
    d->t6next[0]   = d->t6furtha[0]; /* DXCH T6FURTHA  */
    d->t6next[1]   = d->t6furtha[1];
    d->t6furtha[0] = LM_POSMAX;
    d->t6furtha[1] = 0;
    agc_status_t st;                 /* status of the TIME6 restart */

    if ((int32_t) head + T6_PRIO37 > LM_POSMAX)
    { /* list empty     */
        st = set_time6(LM_POSMAX);
    }
    else
    {
        st = set_time6(head); /* ENABLET6       */
        agc_chan_or(013, T6_ENABLE);

        if ((int32_t) d->t6next[0] + T6_PRIO37 > LM_POSMAX)
        {
            d->t6next[0] = LM_POSMAX;
        }
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (lm_dap_write_next(done)); /* GOCH56         */
}

/**********************************************************************
 * DOT6RUPT (p.1405): the T6RUPT handler is T6JOBCHK, then RESUME.
 *
 * FUNCTION NAME: agc_lm_dap_t6rupt
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_t6rupt(void)
{
    agc_status_t st = agc_lm_dap_t6jobchk(); /* T6JOBCHK status */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_rupt_time(6) != LM_T6_NEGZERO || st != AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * Sim harness: apply `counts` DINC pulses to TIME6 while channel 13 bit
 * 15 enables it. The pulse after +0 makes -0 and requests the T6RUPT.
 * AGC: none (helper)
 *
 * FUNCTION NAME: agc_lm_dap_time6_count
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   counts    uint16_t  I    DINC pulses since the last call
 *
 * RETURNS: none
 **********************************************************************/
void agc_lm_dap_time6_count(uint16_t counts)
{
    uint16_t raw = agc_rupt_time(6); /* TIME6 register */

    if (!((agc_chan_read(013) & T6_ENABLE) == 0u || raw > LM_T6_POSMAX || counts == 0u)) /* idle, or rupt already due */
    {
        uint16_t next = (counts > raw) ? (uint16_t) LM_T6_NEGZERO : (uint16_t) (raw - counts); /* counted past +0: -0, a T6RUPT is due */

        agc_rupt_set_time(6, next);
    }
}

/**********************************************************************
 * TIME6 = -0: a T6RUPT is waiting.
 * AGC: none (helper)
 *
 * FUNCTION NAME: agc_lm_dap_t6_pending
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when TIME6 is -0
 **********************************************************************/
bool agc_lm_dap_t6_pending(void)
{
    uint16_t raw = agc_rupt_time(6); /* TIME6 register */

    return (raw == LM_T6_NEGZERO);
}
