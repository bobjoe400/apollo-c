/**********************************************************************
 * FILE NAME: dapif.c
 *
 * PURPOSE:
 *   DAP_INTERFACE_SUBROUTINES.agc (Luminary 099 pp.1406-1409): the
 *   subroutines through which mission programs set the DAP deadband and
 *   zero its commanded attitude and rates. All are called with
 *   interrupts inhibited (INHINT / TC IBNKCALL / RELINT in the AGC).
 *
 * AGC SOURCE:
 *   Luminary099 DAP_INTERFACE_SUBROUTINES.agc pp. 1406-1409
 *     (CALLACCS, SETMAXDB, SETMINDB, RESTORDB, PFLITEDB, ZATTEROR,
 *     STOPRATE, ALLCOAST)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *   CDUX          uint16_t  I    2^0 rev        CDU X angle (agc_erase_cdux())
 *   CDUY          uint16_t  I    2^0 rev        CDU Y angle (agc_erase_cduy())
 *   CDUZ          uint16_t  I    2^0 rev        CDU Z angle (agc_erase_cduz())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_exec_novac              NOVAC: schedule a job
 *   agc_flag_set                set a flag bit (flags.h)
 *   agc_flag_test               test a flag bit (flags.h)
 *   agc_flag_valid              flag id range check (flags.h)
 *   agc_rupt_set_t5_task        T5ADR: next T5RUPT entry
 *   lm_dap_state                DAP erasables (state.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. AGC_STATUS_BAD_ARG on an invalid flag id or a failed
 *   assertion; NOVAC status passed through.
 **********************************************************************/
#include "agc/core/assertion.h"
#include "agc/lm/dap.h"
#include "internal.h"

#define DB_NARROW ((agc_sp_t) 0155)  /* NARROWDB 00155: 0.3 deg at 45    */
#define DB_WIDE   ((agc_sp_t) 03434) /* WIDEDB 03434: 5.0 deg at 45      */
#define DB_POWER  ((agc_sp_t) 0554)  /* POWERDB .02222: 1.0 deg at 45    */
#define PRIO27    ((agc_prio_t) 027) /* PRIO27 (27000 >> 9)              */

/**********************************************************************
 * CALLACCS (p.1407): set up a NOVAC job for 1/ACCS so that the TJETLAW
 * switch curves are repositioned for the new deadband / masks.
 *
 * FUNCTION NAME: lm_dap_callaccs
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_callaccs(void)
{
    agc_status_t st = agc_exec_novac(PRIO27, AGC_JOB_JET_ACCEL_UPDATE); /* NOVAC 1/ACCS */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * DXCH T5ADR: choose the program of the next T5RUPT (DAPIDLER or PAXIS).
 * AGC: none (helper)
 *
 * FUNCTION NAME: lm_dap_set_t5adr
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------
 *   a         lm_dap_t5adr_t  I    next T5RUPT program: DAPIDLER or PAXIS
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (always)
 **********************************************************************/
agc_status_t lm_dap_set_t5adr(lm_dap_t5adr_t a)
{
    lm_dap_state()->t5adr = a;
    agc_rupt_set_t5_task((a == LM_T5_PAXIS) ? AGC_TASK_PAXIS : AGC_TASK_DAPIDLER); /* PAXIS or DAPIDLER */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SETMAXDB (p.1407): deadband 5.0 degrees, then CALLACCS.
 *
 * FUNCTION NAME: agc_lm_dap_setmaxdb
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_setmaxdb(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->db = DB_WIDE;

    if (!AGC_ASSERT(d->db > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (lm_dap_callaccs());
}

/**********************************************************************
 * SETMINDB (p.1407): deadband 0.3 degree, then CALLACCS.
 *
 * FUNCTION NAME: agc_lm_dap_setmindb
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_setmindb(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->db = DB_NARROW;

    if (!AGC_ASSERT(d->db > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (lm_dap_callaccs());
}

/**********************************************************************
 * RESTORDB (p.1407): the crew-selected deadband (DBSELECT of DAPBOOLS:
 * 1 = 5 degrees, 0 = 0.3 degree).
 *
 * FUNCTION NAME: agc_lm_dap_restordb
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_restordb(void)
{
    bool wide = agc_flag_test(AGC_FLAG_DBSELFLG); /* DBSELECT: wide deadband */

    if (!AGC_ASSERT(agc_flag_valid(AGC_FLAG_DBSELFLG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (wide ? agc_lm_dap_setmaxdb() : agc_lm_dap_setmindb()); /* 5 deg or 0.3 deg */
}

/**********************************************************************
 * PFLITEDB (p.1407): zero the attitude errors and commanded rates
 * (ZATTEROR), deadband 1.0 degree, then CALLACCS.
 *
 * FUNCTION NAME: agc_lm_dap_pflitedb
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_pflitedb(void)
{
    agc_lm_dap_zatteror();
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->db = DB_POWER;

    return (lm_dap_callaccs());
}

/**********************************************************************
 * ZATTEROR (p.1408): desired CDUs = present CDUs, then STOPRATE.
 *
 * FUNCTION NAME: agc_lm_dap_zatteror
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_lm_dap_zatteror(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->cduxd = agc_erase_cdux();
    d->cduyd = agc_erase_cduy();
    d->cduzd = agc_erase_cduz();

    agc_lm_dap_stoprate();
}

/**********************************************************************
 * STOPRATE (p.1408): zero the commanded rates, the KALCMANU increments
 * and the command lags.
 *
 * FUNCTION NAME: agc_lm_dap_stoprate
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_lm_dap_stoprate(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->omegapd  = 0;
    d->omegaqd  = 0;
    d->omegard  = 0;
    d->delcdux  = 0;
    d->delcduy  = 0;
    d->delcduz  = 0;
    d->delperor = 0;
    d->delqeror = 0;
    d->delreror = 0;
}

/**********************************************************************
 * ALLCOAST (pp.1408-1409): coasting-flight DAP initialisation, called by
 * fresh start and the engine-off routines. STOPRATE, zero the offset
 * estimates (AOS, ALPHA, AOSTERM), set DRIFTBIT, restore the deadband.
 *
 * FUNCTION NAME: agc_lm_dap_allcoast
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_allcoast(void)
{
    agc_lm_dap_stoprate();
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->aosq     = 0;
    d->aosq_lo  = 0;
    d->aosr     = 0;
    d->aosr_lo  = 0;
    d->alphaq   = 0;                 /* for downlist                   */
    d->alphar   = 0;
    d->aosqterm = 0;
    d->aosrterm = 0;
    agc_flag_set(AGC_FLAG_DRIFTDFL); /* DRIFTBIT       */

    agc_status_t st = agc_lm_dap_restordb(); /* TC RESTORDB +1 */

    if (!AGC_ASSERT(st != AGC_STATUS_OK || d->db == DB_WIDE || d->db == DB_NARROW))
    {
        return (AGC_STATUS_BAD_ARG); /* RESTORDB leaves 5 or 0.3 deg */
    }

    return (st);
}

/**********************************************************************
 * Needles initialisation (NEEDLER calling sequence, p.1417): set bit 3 of
 * RCSFLAGS to start the turn-on sequence of the FDAI error needles.
 *
 * FUNCTION NAME: agc_lm_dap_needles_init
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_lm_dap_needles_init(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->rcsflags = (uint16_t) (d->rcsflags | LM_RCS_NEEDLE3);

    if (!AGC_ASSERT((d->rcsflags & LM_RCS_NEEDLE3) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(d->rcsflags <= 077777u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}
