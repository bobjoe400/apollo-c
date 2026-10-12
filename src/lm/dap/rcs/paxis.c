/**********************************************************************
 * FILE NAME: paxis.c
 *
 * PURPOSE:
 *   P-AXIS_RCS_AUTOPILOT.agc (Luminary 099 pp.1421-1441): the PAXIS
 *   T5RUPT program (every 100 ms while the DAP is on), the gimbal-drive
 *   timers, Y,Z translation (CHKVISFZ), the P-axis jet selection
 *   (PJETSLEC, SELECTP), CALCPERR and PURGENCY. The manual-mode state
 *   logic is in pmanual.c; every P-axis path ends at QRAXIS (qraxis.c).
 *
 * AGC SOURCE:
 *   Luminary099 P-AXIS_RCS_AUTOPILOT.agc pp. 1421-1441
 *     (PAXIS, CHEKBITS, gimbal-drive timers, CHKVISFZ, PJETSLEC, SELECTP,
 *     CALCPERR, PURGENCY)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAP block     lm_dap_t  I/O  per field      EBANK 6 DAP erasables (lm_dap_state())
 *   CDUX          uint16_t  I    2^0 rev        CDU X angle (agc_erase_cdux())
 *   CDUY          uint16_t  I    2^0 rev        CDU Y angle (agc_erase_cduy())
 *   CDUZ          uint16_t  I    2^0 rev        CDU Z angle (agc_erase_cduz())
 *   M11           agc_sp_t  I    2^0            gimbal matrix (agc_erase_m11())
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ------------------------------------
 *   agc_alarm                   ALARM: record an alarm code
 *   agc_bailout                 BAILOUT: abort with restart
 *   agc_chan_and                WAND channel
 *   agc_chan_read               READ channel
 *   agc_flag_invert             complement a flag bit (flags.h)
 *   agc_flag_test               test a flag bit (flags.h)
 *   agc_flag_valid              flag id range check (flags.h)
 *   agc_rupt_set_time           set TIME5 / TIME6
 *   agc_rupt_time               read TIME5 / TIME6
 *   agc_state_vehicle           CSM or LM rope
 *   lm_dap_backp                BACKP rate derivation (rate.c)
 *   lm_dap_ccs_dim              CCS diminished magnitude (arith.c)
 *   lm_dap_chekbits             CHEKBITS (dapidler.c)
 *   lm_dap_dim                  DIM (arith.c)
 *   lm_dap_gimbal_drive_update  ACDT+C12 (gts.c)
 *   lm_dap_jtlst                JTLST (t6rupt.c)
 *   lm_dap_mphi                 A after MP (arith.c)
 *   lm_dap_msu                  MSU (arith.c)
 *   lm_dap_offset_est           offset estimator (rate.c)
 *   lm_dap_ones_to_twos         MSU + 1STOTWOS (arith.c)
 *   lm_dap_pstate               P state logic (pmanual.c)
 *   lm_dap_qraxis               QRAXIS .. CLOSEOUT (qraxis.c)
 *   lm_dap_rateloop             RATELOOP + ROTORQUE (rate.c)
 *   lm_dap_spsrcs               SPSRCS (spsrcs.c)
 *   lm_dap_state                DAP erasables (state.c)
 *   lm_dap_tjetlaw              TJETLAW (tjetlaw.c)
 *   lm_dap_wrap                 overflow-corrected store, TS/ADS/XCH (arith.c)
 *   lm_dap_write                WRITEP/U/V (t6rupt.c)
 *   lm_dap_write_next           WRITEx -1 (t6rupt.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   BAILOUT 02000 when a T5RUPT finds the previous jask unfinished
 *   (DAPZRUPT > 0); alarm 02001 when no P/Y/Z jet policy is found;
 *   alarm 02003 when SELECTP finds no jets (ALARMJET).
 *   AGC_STATUS_BAD_ARG when an internal assertion fails.
 *
 * NOTES:
 *   The AGC turns the rest of the pass into a "jask" (SUPERJOB at interrupt
 *   level with interrupts enabled; DAPZRUPT > 0 while it runs). In C the
 *   pass runs to completion in this call; DAPZRUPT is kept so that a pass
 *   that did not reach CLOSEOUT is detected (BAILOUT 02000) as on the AGC.
 **********************************************************************/
#include <stddef.h>

#include "../internal.h"
#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/lm/dap.h"

#define PX_150MST6   240                /* +150MST6: 150 ms in T6 counts      */
#define PX_MINT6     22                 /* MINTIMES: 14 ms minimum firing     */
#define PX_FOURDEG   1456               /* -FOURDEG = DEC -.08888 at 45 deg   */
#define PX_160MST6   256                /* -160MST6                           */
#define PX_GIMQ      001400u            /* QGIMBITS: chan 12 bits 9,10        */
#define PX_GIMR      006000u            /* RGIMBITS: chan 12 bits 11,12       */
#define PX_TRANSBITS 007400u            /* 07400OCT: chan 31 bits 9-12        */
#define PX_BADINDEX  ((agc_sp_t) 07776) /* INDXYZ: no translation policy    */
#define PX_NTAB      14                 /* JETSALL (7) + TYPEP (7) words */
#define PX_TYPEP     7                  /* TYPEP = JETSALL +7                 */

/* JETSALL (indexed by ROTINDEX) followed by TYPEP (indexed by NUMBERT),
 * contiguous as in the AGC (p.1440): channel 6 jet codes.               */
static const uint16_t jettab[PX_NTAB] = {
    00252, 00125, 00140, 00006,
    00220, 00011, 00151, /* -P +P -Y -Z +Y +Z */
    00146, 00226, 00231, 00151,
    00132, 00245, 00377  /* -U -V +U +V 1-3 2-4 ALL */
};

/* INDXYZ (pp.1440-1441): ROTINDEX for channel 31 bits 9-12 (>> 8).        */
static const agc_sp_t indxyz[16] = { -218, 4, 2, PX_BADINDEX, 5, 9, 10, PX_BADINDEX, 3, 8, 7, PX_BADINDEX, PX_BADINDEX, PX_BADINDEX, PX_BADINDEX, PX_BADINDEX };

/**********************************************************************
 * SELECTP (p.1440): POLYTEMP = TYPEP(NUMBERT) & JETSALL(ROTINDEX); on a
 * failed jet try NUMBERT = 5, 4, ... 0. False if no policy is left.
 *
 * FUNCTION NAME: selectp
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  --------------------
 *   d         lm_dap_t *  I/O  DAP erasables (lm_dap_state())
 *
 * RETURNS: bool -- false when no unfailed policy is left
 **********************************************************************/
static bool selectp(lm_dap_t *d)
{
    for (int tempnum = 6; tempnum >= -1; tempnum--)
    { /* <= 8 tries     */
        d->polytemp = (agc_sp_t) (jettab[PX_TYPEP + d->numbert] & jettab[d->rotindex]);

        if (((uint16_t) d->polytemp & d->ch6mask) == 0u)
        {
            return (true);
        }

        if (tempnum <= 0)
        {
            return (false);
        }

        d->numbert = (agc_sp_t) (tempnum - 1); /* SELECTYZ        */
    }

    return (false);
}

/**********************************************************************
 * JETSOFF (p.1437): write NEXTP to channel 6, TJP = 0.
 *
 * FUNCTION NAME: lm_dap_jetsoff
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_jetsoff(void)
{
    lm_dap_t    *d  = lm_dap_state();       /* DAP erasables */
    agc_status_t st = lm_dap_write_next(0); /* WRITEP -1: P jets off */

    d->tj[0] = 0;

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(d->tj[0] == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * ALTSYST (p.1437): alternate the P-axis jet system (AORBSYST).
 *
 * FUNCTION NAME: altsyst
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
static agc_status_t altsyst(void)
{
    agc_flag_invert(AGC_FLAG_AORBSFLG);

    if (!AGC_ASSERT(agc_flag_valid(AGC_FLAG_AORBSFLG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * PJETSLEC (pp.1436-1437), entered at PJETSLEC -1 with NUMBERT = 4, 5
 * (two jets, system A or B) or 6 (four jets): select and fire the P jets;
 * firings under 150 ms are ended by a T6RUPT (JTLST) and the next P pass
 * is skipped (RCSFLAGS bit 12).
 *
 * FUNCTION NAME: lm_dap_pjetslec
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------
 *   numbert   agc_sp_t  I    NUMBERT: 4, 5 (2 jets, system A, B) or 6 (4
 *                            jets)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_pjetslec(agc_sp_t numbert)
{
    if (!AGC_ASSERT((numbert >= 4) && (numbert <= 6)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->numbert = numbert;

    if (d->tj[0] == 0)
    {
        return (lm_dap_jetsoff());
    }

    d->abstj    = (agc_sp_t) ((d->tj[0] > 0) ? d->tj[0] : -d->tj[0]); /* ABSTJ = |TJP| */
    d->rotindex = (d->tj[0] > 0) ? 1 : 0;                             /* +P : -P        */

    if (!selectp(d))
    {
        agc_alarm(AGC_ALARM_02003); /* ALARMJET       */

        return (lm_dap_jetsoff());
    }

    d->nojets[0]    = (d->numbert == 6) ? 4 : 2;               /* four jets for the max-jet policy */
    agc_status_t st = lm_dap_write(0, (uint16_t) d->polytemp); /* WRITEP: jets on now */

    if (PX_150MST6 - d->abstj <= 0)
    {
        return (st); /* long firing    */
    }

    if (PX_MINT6 - d->abstj > 0)
    { /* MINTIMES       */
        d->abstj = PX_MINT6;
        d->tj[0] = (agc_sp_t) ((d->rotindex != 0) ? PX_MINT6 : -PX_MINT6); /* MINTIMES with the sign of TJP */
    }

    d->t6furtha[0]  = d->abstj;
    d->t6furtha[1]  = 0;              /* P-axis index   */
    agc_status_t sj = lm_dap_jtlst(); /* JTLST: schedule the turn-off */

    d->rcsflags     = (uint16_t) (d->rcsflags & ~LM_RCS_PSKIP); /* skip next P   */
    agc_status_t sa = altsyst();                                /* ALTSYST: gimbal timers, needles */

    if (sj != AGC_STATUS_OK)
    {
        return (sj);
    }

    if (sa != AGC_STATUS_OK)
    {
        return (sa);
    }

    if (!AGC_ASSERT(d->abstj >= PX_MINT6))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
} /* lm_dap_pjetslec */

/**********************************************************************
 * PJETSLEC -6 (p.1436): two jets of the system named by AORBSYST.
 *
 * FUNCTION NAME: lm_dap_pjetslec_pair
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_pjetslec_pair(void)
{
    agc_sp_t n = agc_flag_test(AGC_FLAG_AORBSFLG) ? 5 : 4; /* NUMBERT 5 or 4: the jet pair AORBSYST selects (unscaled code) */

    if (!AGC_ASSERT(n == 4 || n == 5))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_flag_valid(AGC_FLAG_AORBSFLG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (lm_dap_pjetslec(n));
}

/**********************************************************************
 * CALCPERR (pp.1437-1438): P attitude error at pi, into E and PERROR.
 *
 * FUNCTION NAME: lm_dap_calcperr
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_calcperr(void)
{
    lm_dap_t *d = lm_dap_state();                                                       /* DAP erasables */
    agc_sp_t  e = lm_dap_mphi(lm_dap_msu(agc_erase_cduy(), d->cduyd), agc_erase_m11()); /* CDUY-CDUYD times M11, scaled pi rad */
    int32_t   x = (int32_t) lm_dap_msu(agc_erase_cdux(), d->cduxd) + d->delperor;       /* CDUX-CDUXD + DELPEROR, scaled pi rad */

    d->e      = lm_dap_wrap((int32_t) e + x); /* ADS E: may wrap */
    d->perror = d->e;                         /* eight-ball     */

    if (!AGC_ASSERT(d->cduxd <= 077777u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * PURGENCY +4 (p.1438): P control with E, EDOT set; docked: SPSRCS,
 * otherwise TJETLAW and the 2-jet / 4-jet choice (HEADTJET).
 *
 * FUNCTION NAME: lm_dap_purgency_law
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_purgency_law(void)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    d->axisctr = -1;

    if (agc_flag_test(AGC_FLAG_CSMDKFLG))
    {
        agc_status_t ss = lm_dap_spsrcs(); /* SPSRCS law for P */

        if (ss != AGC_STATUS_OK)
        {
            return (ss);
        }

        if (d->tj[0] == 0)
        { /* DKALT          */
            agc_status_t sa = altsyst();        /* ALTSYST */
            agc_status_t so = lm_dap_jetsoff(); /* JETSOFF */

            return ((sa != AGC_STATUS_OK) ? sa : so); /* first failure wins */
        }

        return (lm_dap_pjetslec_pair());
    }

    d->sensetyp     = 0;                /* HEADTJET       */
    agc_status_t st = lm_dap_tjetlaw(); /* TJETLAW for P */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if ((int32_t) -d->firefct - PX_FOURDEG <= 0)
    { /* not 4 deg past */
        return (lm_dap_pjetslec_pair());
    }

    if (d->tj[0] == 0)
    {
        return (lm_dap_jetsoff());
    }

    if ((int32_t) lm_dap_ccs_dim(d->tj[0]) - PX_160MST6 <= 0)
    {
        return (lm_dap_pjetslec_pair());
    }

    return (lm_dap_pjetslec(6)); /* four jets      */
} /* lm_dap_purgency_law */

/**********************************************************************
 * PURGENCY (p.1438): P error and rate error, then the control law.
 *
 * FUNCTION NAME: lm_dap_purgency
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_purgency(void)
{
    lm_dap_t    *d  = lm_dap_state();    /* DAP erasables */
    agc_status_t st = lm_dap_calcperr(); /* CALCPERR */

    d->edot = lm_dap_wrap((int32_t) d->omegap - d->omegapd); /* EDOTP     */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (lm_dap_purgency_law());
}

/**********************************************************************
 * CHKVISFZ / TRYUORV / ABORTYZ (pp.1429-1430): Y,Z translation from the
 * THC into NEXTP. *policy = false if SELECTP found no policy (ALARM 02003,
 * JETSOFF done): the pass then goes straight to QRAXIS.
 *
 * FUNCTION NAME: lm_dap_chkvisfz
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description (scale)
 *   --------  ------  ---  --------------------
 *   policy    bool *  O    false when SELECTP found no policy
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_chkvisfz(bool *policy)
{
    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    *policy       = true;
    uint16_t bits = (uint16_t) (~agc_chan_read(031) & PX_TRANSBITS); /* THC translation bits (inverted channel 31) */

    if (bits == 0u)
    {
        d->nextp = 0;

        return (AGC_STATUS_OK);
    }

    d->rotindex = indxyz[bits >> 8]; /* MP BIT7        */

    for (int pass = 0; pass < 2; pass++)
    { /* TRYUORV        */
        if (d->rotindex == PX_BADINDEX)
        { /* TC JETSALL -1  */
            d->rcsflags = (uint16_t) (d->rcsflags ^ LM_RCS_ZEROOR1); /* ABORTYZ +2 */
            d->nextp    = 0;

            return (AGC_STATUS_OK);
        }

        d->numbert = 6;

        if (!selectp(d))
        {
            *policy = false;
            agc_alarm(AGC_ALARM_02003);

            return (lm_dap_jetsoff());
        }

        if (d->numbert == 6 || (d->rotindex > 5 && d->numbert >= 4))
        {
            d->nextp = (uint16_t) d->polytemp;

            return (AGC_STATUS_OK);
        }

        if (d->rotindex > 5)
        {
            break; /* ABORTYZ        */
        }

        /* ALTERYZ (p.1430): flip RCSFLAGS bit 1 and move to a diagonal policy. */
        d->rcsflags = (uint16_t) (d->rcsflags ^ LM_RCS_ZEROOR1);
        d->rotindex = (agc_sp_t) (d->rotindex + 4 + (int) (d->rcsflags & LM_RCS_ZEROOR1));
    }

    agc_alarm(AGC_ALARM_02001);

    d->rcsflags = (uint16_t) (d->rcsflags ^ LM_RCS_ZEROOR1);
    d->nextp    = 0;

    if (!AGC_ASSERT(d->nextp == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* lm_dap_chkvisfz */

/**********************************************************************
 * PRETIMCK .. CHKRTIMR (pp.1428-1429): count the trim-gimbal drive timers
 * down (0.1 s per pass) and end a drive at zero; decrement the docked
 * jet-inhibition counters.
 *
 * FUNCTION NAME: lm_dap_pretimck
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t lm_dap_pretimck(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    lm_dap_t *d = lm_dap_state(); /* DAP erasables */

    if (d->qgimtimr > 0)
    {
        d->qgimtimr--; /* DECQTIMR       */
    }
    else if (d->qgimtimr == 0)
    { /* TURNOFFQ       */
        d->neguq    = 0;
        d->qaccdot  = 0;
        agc_chan_and(012, (uint16_t) ~PX_GIMQ & 077777u);
        d->qgimtimr = LM_NEGMAX;
    }

    if (d->rgimtimr > 0)
    {
        d->rgimtimr--; /* DECRTIMR       */
    }
    else if (d->rgimtimr == 0)
    { /* TURNOFFR       */
        d->negur    = 0;
        d->raccdot  = 0;
        agc_chan_and(012, (uint16_t) ~PX_GIMR & 077777u);
        d->rgimtimr = LM_NEGMAX;
    }

    for (int i = 0; i < 3; i++)
    {
        d->jetctr[i] = lm_dap_dim(d->jetctr[i]); /* DIM PJETCTR..  */
    }

    if (!AGC_ASSERT(d->jetctr[0] >= 0 && d->jetctr[1] >= 0 && d->jetctr[2] >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* lm_dap_pretimck */

/**********************************************************************
 * PAXIS (pp.1421-1428): the T5RUPT program of the running DAP.
 *
 * FUNCTION NAME: agc_task_paxis
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a
 *          failed assertion, else the status of the routine it
 *          ends in
 **********************************************************************/
agc_status_t agc_task_paxis(void)
{
    lm_dap_t *d  = lm_dap_state();   /* DAP erasables */
    uint16_t  t5 = agc_rupt_time(5); /* TIME5 */

    agc_rupt_set_time(5, (uint16_t) ((t5 + LM_MS100) & 077777u)); /* ADS TIME5 */

    if (d->dapzrupt > 0)
    { /* previous jask unfinished */
        return (agc_bailout(AGC_ALARM_02000));
    }

    bool         on = false;                /* CHEKBITS: DAP may run */
    agc_status_t sc = lm_dap_chekbits(&on); /* CHEKBITS status */

    if (sc != AGC_STATUS_OK || !on)
    {
        return (sc);
    }

    d->dapcdu[0] = agc_erase_cdux(); /* DAPTREG4..6              */
    d->dapcdu[1] = agc_erase_cduy();
    d->dapcdu[2] = agc_erase_cduz();

    /* KALCMANU interface (pp.1421-1422): CDUxD = CDUxD - DELCDUx (modular, 2's
     * complement); then count down the direct-rate timers TCP, TCQR.         */
    d->cduxd = lm_dap_ones_to_twos(lm_dap_msu(d->cduxd, (uint16_t) ((uint16_t) d->delcdux & 077777u)));
    d->cduyd = lm_dap_ones_to_twos(lm_dap_msu(d->cduyd, (uint16_t) ((uint16_t) d->delcduy & 077777u)));
    d->cduzd = lm_dap_ones_to_twos(lm_dap_msu(d->cduzd, (uint16_t) ((uint16_t) d->delcduz & 077777u)));
    d->tcp   = lm_dap_dim(d->tcp);
    d->tcqr  = lm_dap_dim(d->tcqr);

    if (!AGC_ASSERT(d->cduxd <= 077777u && d->cduyd <= 077777u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(d->cduzd <= 077777u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_OK; /* the jask's status; a failure stops the chain */

    if ((d->rcsflags & LM_RCS_CALLGMBL) != 0u)
    { /* PAXFILT       */
        st = lm_dap_gimbal_drive_update();
    }

    d->dapzrupt = 1;                                                /* SUPERJOB: jask running   */
    st          = (st == AGC_STATUS_OK) ? lm_dap_rateloop() : st;   /* RATELOOP unless an earlier step failed */
    st          = (st == AGC_STATUS_OK) ? lm_dap_backp() : st;      /* BACKP unless an earlier step failed */
    st          = (st == AGC_STATUS_OK) ? lm_dap_offset_est() : st; /* offset estimator unless an earlier step failed */
    st          = (st == AGC_STATUS_OK) ? lm_dap_pretimck() : st;   /* PRETIMCK unless an earlier step failed */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* DAPZRUPT stays positive   */
    }

    if ((d->rcsflags & LM_RCS_PSKIP) == 0u)
    {
        d->rcsflags = (uint16_t) (d->rcsflags | LM_RCS_PSKIP); /* SKIPPAXS */
    }
    else
    {
        bool         policy = true;                     /* CHKVISFZ: Y,Z translation leaves P to the state logic */
        agc_status_t st_p   = lm_dap_chkvisfz(&policy); /* CHKVISFZ status */

        if (st_p == AGC_STATUS_OK && policy)
        {
            st_p = lm_dap_pstate();
        }

        if (st_p != AGC_STATUS_OK)
        {
            return (st_p);
        }
    }

    return (lm_dap_qraxis());
} /* agc_task_paxis */
