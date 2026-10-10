/*
 * dapif.c -- DAP_INTERFACE_SUBROUTINES.agc (Luminary 099 pp.1406-1409):
 * the subroutines through which mission programs set the DAP deadband and
 * zero its commanded attitude and rates. All are called with interrupts
 * inhibited (INHINT / TC IBNKCALL / RELINT in the AGC).
 */
#include "agc/agc_assert.h"
#include "agc/agc_lm_dap.h"
#include "lm_dap_internal.h"

#define DB_NARROW ((agc_sp_t)0155)     /* NARROWDB 00155: 0.3 deg at 45    */
#define DB_WIDE   ((agc_sp_t)03434)    /* WIDEDB 03434: 5.0 deg at 45      */
#define DB_POWER  ((agc_sp_t)0554)     /* POWERDB .02222: 1.0 deg at 45    */
#define PRIO27    ((agc_prio_t)027)    /* PRIO27 (27000 >> 9)              */

/* CALLACCS (p.1407): set up a NOVAC job for 1/ACCS so that the TJETLAW
 * switch curves are repositioned for the new deadband / masks.           */
agc_status_t lm_dap_callaccs(void)
{
    agc_status_t st = agc_exec_novac(PRIO27, AGC_JOB_1_ACCJOB);
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    (void)AGC_ASSERT(lm_dap_state()->db >= 0);
    return st;
}

/* DXCH T5ADR: choose the program of the next T5RUPT (DAPIDLER or PAXIS). */
agc_status_t lm_dap_set_t5adr(lm_dap_t5adr_t a)
{
    if (!AGC_ASSERT(a == LM_T5_DAPIDLER || a == LM_T5_PAXIS)) {
        return AGC_STATUS_BAD_ARG;
    }
    lm_dap_state()->t5adr = a;
    agc_status_t st = agc_rupt_set_t5_task((a == LM_T5_PAXIS) ? AGC_TASK_PAXIS
                                                               : AGC_TASK_DAPIDLER);
    (void)AGC_ASSERT(lm_dap_state()->t5adr == a);
    return st;
}

/* SETMAXDB (p.1407): deadband 5.0 degrees, then CALLACCS.                 */
agc_status_t agc_lm_dap_setmaxdb(void)
{
    lm_dap_t *d = lm_dap_state();
    d->db = DB_WIDE;
    (void)AGC_ASSERT(d->db > 0);
    return lm_dap_callaccs();
}

/* SETMINDB (p.1407): deadband 0.3 degree, then CALLACCS.                  */
agc_status_t agc_lm_dap_setmindb(void)
{
    lm_dap_t *d = lm_dap_state();
    d->db = DB_NARROW;
    (void)AGC_ASSERT(d->db > 0);
    return lm_dap_callaccs();
}

/* RESTORDB (p.1407): the crew-selected deadband (DBSELECT of DAPBOOLS:
 * 1 = 5 degrees, 0 = 0.3 degree).                                        */
agc_status_t agc_lm_dap_restordb(void)
{
    bool wide = agc_flag_test(AGC_FLAG_DBSELFLG);
    (void)AGC_ASSERT(agc_flag_valid(AGC_FLAG_DBSELFLG));
    return wide ? agc_lm_dap_setmaxdb() : agc_lm_dap_setmindb();
}

/* PFLITEDB (p.1407): zero the attitude errors and commanded rates
 * (ZATTEROR), deadband 1.0 degree, then CALLACCS.                        */
agc_status_t agc_lm_dap_pflitedb(void)
{
    agc_status_t st = agc_lm_dap_zatteror();
    lm_dap_t *d = lm_dap_state();
    d->db = DB_POWER;
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(d->db == DB_POWER);
    return lm_dap_callaccs();
}

/* ZATTEROR (p.1408): desired CDUs = present CDUs, then STOPRATE.          */
agc_status_t agc_lm_dap_zatteror(void)
{
    lm_dap_t *d = lm_dap_state();
    d->cduxd = agc_erase_cdux();
    d->cduyd = agc_erase_cduy();
    d->cduzd = agc_erase_cduz();
    (void)AGC_ASSERT(d->cduxd <= 077777u && d->cduyd <= 077777u);
    (void)AGC_ASSERT(d->cduzd <= 077777u);
    return agc_lm_dap_stoprate();
}

/* STOPRATE (p.1408): zero the commanded rates, the KALCMANU increments
 * and the command lags.                                                   */
agc_status_t agc_lm_dap_stoprate(void)
{
    lm_dap_t *d = lm_dap_state();
    d->omegapd = 0;
    d->omegaqd = 0;
    d->omegard = 0;
    d->delcdux = 0;
    d->delcduy = 0;
    d->delcduz = 0;
    d->delperor = 0;
    d->delqeror = 0;
    d->delreror = 0;
    (void)AGC_ASSERT(d->omegapd == 0 && d->delcdux == 0);
    (void)AGC_ASSERT(d->delperor == 0);
    return AGC_STATUS_OK;
}

/* ALLCOAST (pp.1408-1409): coasting-flight DAP initialisation, called by
 * fresh start and the engine-off routines. STOPRATE, zero the offset
 * estimates (AOS, ALPHA, AOSTERM), set DRIFTBIT, restore the deadband.  */
agc_status_t agc_lm_dap_allcoast(void)
{
    agc_status_t st = agc_lm_dap_stoprate();
    lm_dap_t *d = lm_dap_state();
    d->aosq = 0;
    d->aosq_lo = 0;
    d->aosr = 0;
    d->aosr_lo = 0;
    d->alphaq = 0;                        /* for downlist                   */
    d->alphar = 0;
    d->aosqterm = 0;
    d->aosrterm = 0;
    agc_status_t fs = agc_flag_set(AGC_FLAG_DRIFTDFL);   /* DRIFTBIT       */
    if (!AGC_ASSERT(st == AGC_STATUS_OK) || !AGC_ASSERT(fs == AGC_STATUS_OK)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_lm_dap_restordb();         /* TC RESTORDB +1                 */
}

/* Needles initialisation (NEEDLER calling sequence, p.1417): set bit 3 of
 * RCSFLAGS to start the turn-on sequence of the FDAI error needles.     */
agc_status_t agc_lm_dap_needles_init(void)
{
    lm_dap_t *d = lm_dap_state();
    d->rcsflags = (uint16_t)(d->rcsflags | LM_RCS_NEEDLE3);
    (void)AGC_ASSERT((d->rcsflags & LM_RCS_NEEDLE3) != 0u);
    (void)AGC_ASSERT(d->rcsflags <= 077777u);
    return AGC_STATUS_OK;
}
