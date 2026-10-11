/**********************************************************************
 * FILE NAME: dapidler.c
 *
 * PURPOSE:
 *   DAPIDLER_PROGRAM.agc (Luminary 099 pp.1410-1413): the T5RUPT program
 *   that idles at 10 per second until the crew wants the DAP and the IMU
 *   and CDUs are usable, initialises the DAP (STARTDAP) and then hands
 *   T5ADR to PAXIS; and the T5RUPT dispatch through T5ADR.
 *
 * AGC SOURCE:
 *   Luminary099 DAPIDLER_PROGRAM.agc pp. 1410-1413
 *     (DAPIDLER, STARTDAP, SETTIME5, SHUTDOWN, MOREIDLE, T5RUPT dispatch)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *   CDUX          uint16_t  I    2^0 rev        CDU X angle (agc_erase_cdux())
 *   CDUY          uint16_t  I    2^0 rev        CDU Y angle (agc_erase_cduy())
 *   CDUZ          uint16_t  I    2^0 rev        CDU Z angle (agc_erase_cduz())
 *   IMODES33      uint16_t  I    bits           IMU mode bits (agc_erase_imodes33())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_chan_and                WAND channel
 *   agc_chan_read               READ channel
 *   agc_chan_write              WRITE channel
 *   agc_exec_novac              NOVAC: schedule a job
 *   agc_flag_test               test a flag bit (flags.h)
 *   agc_lm_dap_zatteror         ZATTEROR: CDUxD = CDUx, STOPRATE (dapif.c)
 *   agc_rupt_set_time           set TIME5 / TIME6
 *   agc_rupt_time               read TIME5 / TIME6
 *   agc_task_paxis              PAXIS T5RUPT (paxis.c)
 *   lm_dap_altdsply             ALTDSPLY (needles.c)
 *   lm_dap_calcperr             CALCPERR (paxis.c)
 *   lm_dap_qerrcalc             QERRCALC + RERRCALC (qraxis.c)
 *   lm_dap_set_t5adr            DXCH T5ADR (dapif.c)
 *   lm_dap_state                DAP erasables (state.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. AGC_STATUS_BAD_ARG when an internal assertion fails.
 **********************************************************************/
#include <stddef.h>

#include "../internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"

#define CH31_MODE_OFF  030000u            /* BIT13-14: mode select switch OFF  */
#define IMODES33_NOIMU 000040u            /* BIT6: IMU not usable              */
#define CH30_NOPGNCS   001000u            /* BIT10 of 30 (inverted): not PGNCS */
#define MANFLAG        03021u             /* RCSFLAGS bits cleared by STARTDAP */
#define BGIM23         07400u             /* trim gimbal drive bits 9-12, ch 12*/
#define PRIO27         ((agc_prio_t) 027) /* 1/ACCJOB (1/ACCS reset) priority 27 */
#define STARTDAP_SKIP  ((agc_sp_t) 4)     /* FOUR: SKIPU/SKIPV not skipping    */
#define STARTDAP_TRAPS ((agc_sp_t) 2)     /* TWO: NPTRAPS, NQTRAPS, NRTRAPS    */

/**********************************************************************
 * SETTIME5 (p.1412): next T5RUPT in 100 ms (TIME5 = 37766).
 *
 * FUNCTION NAME: settime5
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t settime5(void)
{
    agc_rupt_set_time(5, LM_MS100);

    if (!AGC_ASSERT(agc_rupt_time(5) == LM_MS100))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SHUTDOWN (p.1413): T5ADR = DAPIDLER, kill jet requests, jets off, trim
 * gimbal off, and come back in 100 ms.
 *
 * FUNCTION NAME: lm_dap_shutdown
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_shutdown(void)
{
    lm_dap_t    *d  = lm_dap_state();                   /* DAP erasables */
    agc_status_t st = lm_dap_set_t5adr(LM_T5_DAPIDLER); /* T5ADR = DAPIDLER */

    d->nextp        = 0;
    d->nextu        = 0;
    d->nextv        = 0;
    agc_chan_write(005, 0);       /* command jets off        */
    agc_chan_write(006, 0);
    agc_chan_and(012, (uint16_t) ~BGIM23 & 077777u);
    agc_status_t t5 = settime5(); /* next T5RUPT in 100 ms */

    return ((st != AGC_STATUS_OK) ? st : t5); /* first failure wins */
}

/**********************************************************************
 * MOREIDLE (p.1413): keep computing the attitude errors for the displays,
 * then SHUTDOWN.
 *
 * FUNCTION NAME: lm_dap_moreidle
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_moreidle(void)
{
    agc_status_t sq = lm_dap_qerrcalc(); /* QERRCALC, RERRCALC */
    agc_status_t sp = lm_dap_calcperr(); /* CALCPERR */

    if (!AGC_ASSERT(sq == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(sp == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (lm_dap_shutdown());
}

/**********************************************************************
 * CHEKBITS / CHEKMORE (p.1410). *on = true ("RETURN") when the DAP is to
 * stay on; otherwise the idle path (MOREIDLE or SHUTDOWN) has been run and
 * the interrupt ends. On the way the attitude-error display is serviced
 * (JUMPDSP -> ALTDSPLY -> RETNMORE -> CHEKMORE).
 *
 * FUNCTION NAME: lm_dap_chekbits
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description (scale)
 *   --------  ------  ---  --------------------
 *   on        bool *  O    true when the DAP is to stay on
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_chekbits(bool *on)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    if (!AGC_ASSERT(on != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *on           = false;
    uint16_t ch31 = agc_chan_read(031); /* channel 31: DAP mode and guidance control bits */

    if (((uint16_t) ~ch31 & CH31_MODE_OFF) == 0u)
    { /* mode switch OFF     */
        return (lm_dap_moreidle());
    }

    if ((agc_erase_imodes33() & IMODES33_NOIMU) != 0u)
    {
        d->rcsflags = (uint16_t) (d->rcsflags | LM_RCS_NEEDLE3); /* IMU not usable */

        return (lm_dap_shutdown());
    }

    agc_status_t st = lm_dap_altdsply(); /* JUMPDSP             */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    if ((agc_chan_read(030) & CH30_NOPGNCS) != 0u)
    { /* CHEKMORE            */
        return (lm_dap_moreidle());
    }

    if (!AGC_ASSERT(d->t5adr <= LM_T5_PAXIS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *on = true;

    return (AGC_STATUS_OK);
} /* lm_dap_chekbits */

/**********************************************************************
 * STARTDAP (pp.1411-1413): zero the attitude errors, rates, offset
 * estimates, trim-gimbal and minimum-impulse state; old CDUs, skip and
 * T6 list initialisation; then T5ADR = PAXIS.
 *
 * FUNCTION NAME: lm_dap_startdap
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_startdap(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    agc_lm_dap_zatteror();

    for (int i = 0; i < 3; i++)
    {
        d->tj[i]     = 0;
        d->jetctr[i] = 0; /* docked jet inhibition counters     */
    }

    d->omegap   = 0;
    d->omegaq   = 0;
    d->omegar   = 0;
    d->trapedp  = 0;
    d->trapedq  = 0;
    d->trapedr  = 0;
    d->aosq     = 0;
    d->aosq_lo  = 0;
    d->aosr     = 0;
    d->aosr_lo  = 0;
    d->alphaq   = 0;
    d->alphar   = 0;
    d->neguq    = 0;
    d->negur    = 0;
    d->aosqterm = 0;
    d->aosrterm = 0;
    d->qaccdot  = 0;
    d->raccdot  = 0;
    d->allowgts = 0;
    d->cotroler = 0; /* do TRYGTS on first pass            */
    d->ingts    = 0;
    d->qgimtimr = 0; /* stop gimbal drives                 */
    d->rgimtimr = 0;
    d->oldpmin  = 0;
    d->oldqrmin = 0;

    if (!AGC_ASSERT(d->tj[0] == 0 && d->aosq == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(d->cotroler == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    d->rcsflags     = (uint16_t) (d->rcsflags & ~MANFLAG);
    d->oldxforp     = agc_erase_cdux();
    d->oldyforp     = agc_erase_cduy();
    d->oldzforq     = agc_erase_cduz();
    d->rcsflags     = (uint16_t) (d->rcsflags | LM_RCS_PSKIP);
    d->skip[0]      = STARTDAP_SKIP;
    d->skip[1]      = STARTDAP_SKIP;
    agc_rupt_set_time(6, LM_T6_POSMAX);
    d->t6next[0]    = (agc_sp_t) LM_T6_POSMAX;
    d->t6furtha[0]  = (agc_sp_t) LM_T6_POSMAX;
    d->t6next[1]    = 0;
    d->t6furtha[1]  = 0;
    d->nxt6adr      = 0;
    d->nextp        = 0;
    d->nextu        = 0;
    d->nextv        = 0;
    d->dapzrupt     = LM_DAPZ_NEG;                   /* jask not in progress               */
    d->nptraps      = STARTDAP_TRAPS;
    d->nqtraps      = STARTDAP_TRAPS;
    d->nrtraps      = STARTDAP_TRAPS;
    agc_status_t s5 = lm_dap_set_t5adr(LM_T5_PAXIS); /* T5ADR = PAXIS */

    if (!AGC_ASSERT(s5 == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (settime5());
} /* lm_dap_startdap */

/**********************************************************************
 * DAPIDLER (p.1411): T5RUPT entry while the DAP is idling.
 *
 * FUNCTION NAME: agc_task_dapidler
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_task_dapidler(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    if (!AGC_ASSERT(d->t5adr == LM_T5_DAPIDLER))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((d->rcsflags & LM_RCS_ACCSET) == 0u)
    { /* 1/ACCSET since restart? */
        d->rcsflags     = (uint16_t) (d->rcsflags | LM_RCS_ACCSET);
        agc_status_t sj = agc_exec_novac(PRIO27, AGC_JOB_JET_ACCEL_RESET); /* NOVAC 1/ACCS reset job */

        if (sj != AGC_STATUS_OK)
        {
            return (sj); /* NOVAC's own BAILOUT 1202 */
        }
    }

    bool         on = false;                /* CHEKBITS: DAP may run */
    agc_status_t sc = lm_dap_chekbits(&on); /* CHECKUP                 */

    if (sc != AGC_STATUS_OK || !on)
    {
        return (sc);
    }

    if (!agc_flag_test(AGC_FLAG_ACCOKFLG))
    { /* 1/ACCS not done: idle   */
        return (lm_dap_moreidle());
    }

    return (lm_dap_startdap());
}

/**********************************************************************
 * T5RUPT through T5ADR (INTERRUPT_LEAD_INS: DXCH ARUPT, DTCB T5ADR).
 * AGC: none (helper)
 *
 * FUNCTION NAME: agc_lm_dap_t5rupt
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_t5rupt(void)
{
    const lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    if (!AGC_ASSERT(d->t5adr == LM_T5_DAPIDLER || d->t5adr == LM_T5_PAXIS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((d->t5adr == LM_T5_PAXIS) ? agc_task_paxis() : agc_task_dapidler()); /* T5ADR dispatch */
}
