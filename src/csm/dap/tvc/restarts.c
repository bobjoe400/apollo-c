/**********************************************************************
 * FILE NAME: restarts.c
 *
 * PURPOSE:
 *   The TVC restart package: restart-proofs the pitch and yaw DAPs, the
 *   TVC executive, the roll DAP, TVCINIT4, TVCDAPON and the CSM/LM V46
 *   switch-over. An interrupted copycycle is completed; TVC enable and
 *   the optics error counter enable are set as soon as possible, the
 *   actuators are commanded to the last commands, and the DAPs go back
 *   on the air through TVCINIT4 (or MRCLEAN when the restart came before
 *   TVCINIT4, TVCPHASE = -1). TVCPHASE = -2 repeats the switch-over. A
 *   stroke test in progress is terminated (a new V68 is required).
 *   Labels: REDOTVC, EXECPHS, TVCDAPHS, ENABL1, ENABL2, CMDSOUT, PHSCHK2,
 *   CHKSTRK, FINCOPY, TRIM/CMD, TSTINITJ, EXRSTRT, TVCCADR, TVCEXADR.
 *
 * AGC SOURCE:
 *   Comanche055 TVCRESTARTS.agc pp. 956-960 (CSM only)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   TVCPHASE,     agc_sp_t  I/O  unscaled       DAP / executive phase
 *   TVCEXPHS                                    points (TVC block)
 *   PCMD, YCMD    agc_sp_t  I/O  85.41"/bit     last actuator commands
 *   PACTOFF,      agc_sp_t  I    85.41"/bit     trims
 *   YACTOFF
 *   TVCPITCH,     agc_sp_t  O    85.41"/bit     optics error counters
 *   TVCYAW
 *   CHAN12        uint16_t  O    bits           8 TVC enable, 11 optics
 *                                               DAC, 2 error counters
 *   CHAN14        uint16_t  O    bits           11, 12 count release
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   csm_tvc_pcopy/ycopy   PCOPY +1, YCOPY +1 (daps.c)
 *   csm_tvc_mrclean       MRCLEAN (init.c)
 *   csm_tvc_init_complete TVCINIT4 (init.c)
 *   csm_tvc_swichovr_body SWICHOVR +5 (init.c)
 *   csm_tvc_exec_restart  TVCEXEC restart points (exec.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG when TVCPHASE or TVCEXPHS is out
 *   of range; WAITLIST / T5 failures are passed on.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   REDOTVC is reached by T5 from ELRSKIP after any restart that finds
 *   FLAGWRD6 bits 15,14 = 10. ENABL2, CMDSOUT are T5 tasks, EXRSTRT a
 *   waitlist task (9 cs: before PITCHDAP, after CMDSOUT). CSM only.
 *
 * NOTES:
 *   DEVIATION (CMDSOUT): the AGC writes -0 + PCMD to avoid a +0 command
 *   word; -0 is not represented, so a zero command is written as 0.
 *   STROKER -0 ("a prior V68") is not represented either (stroke.c).
 *
 * ALGORITHM:
 *   REDOTVC: TVCEXPHS > 0 -> EXRSTRT in 9 cs. TVCPHASE 1 or 3 -> finish
 *   the copycycle; -1 -> PCMD, YCMD = trims; then ENABL1 -> 60 ms ->
 *   ENABL2 -> 20 ms -> CMDSOUT -> MRCLEAN (TVCPHASE -1), or SWICHOVR +5
 *   (-2) and CHKSTRK, or CHKSTRK -> TVCINIT4.
 **********************************************************************/
#include "../internal.h"

#define EXRSTRT_DELAY ((uint16_t) 9)                         /* NINE cs: before PITCHDAP   */
#define T5_60MS       ((uint16_t) 037772)                    /* TVCEXADR: 60 ms (TIME5)    */
#define T5_20MS       ((uint16_t) 037776)                    /* OCT37776: 20 ms (TIME5)    */
#define CH12_ENABL1 \
    ((uint16_t) (AGC_CH012_TVCENABLE | CSM_DAP_BIT11))       /* BIT8 + BIT11 */
#define CH14_RELEASE \
    ((uint16_t) (AGC_CH014_DRIVECDUS | AGC_CH014_DRIVECDUT)) /* PRIO6  */

/**********************************************************************
 * ENABL1 (TVCRESTARTS.agc p. 958) -- TVC enable and optics-DAC enable
 * (channel 12 bits 8, 11), then a 60 ms (40 ms minimum) wait calling
 * ENABL2.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the T5 set-up (RESUME)
 **********************************************************************/
static agc_status_t tvcr_tvc_enable(void)
{
    agc_chan_or(AGC_CHAN_CHAN12, CH12_ENABL1);
    agc_status_t st = csm_dap_set_t5(AGC_TASK_OPTICS_COUNTER_ENABLE, T5_60MS); /* T5   */

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_TVCENABLE) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * REDOTVC, EXECPHS, TVCDAPHS, FINCOPY, TRIM/CMD (TVCRESTARTS.agc
 * pp. 957-958) -- T5 entry of the restart package.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a bad
 *          TVCPHASE, or the first WAITLIST / copycycle / T5 failure
 **********************************************************************/
agc_status_t agc_task_redotvc(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC erasables    */
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* trims            */
    agc_status_t st = AGC_STATUS_OK;     /* first failure    */
    agc_status_t s2 = AGC_STATUS_OK;     /* copycycle status */

    if (!AGC_ASSERT((t->tvcphase >= -2) && (t->tvcphase <= 3)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (t->tvcexphs > 0)
    { /* EXECPHS: the TVC executive must restart                       */
        st = agc_waitlist_call(EXRSTRT_DELAY, AGC_TASK_EXRSTRT);
    }

    /* TVCDAPHS: TVCPHASE bits 15 and 1 select the DAP restart location */
    if (t->tvcphase == 1)
    {
        s2 = csm_tvc_pcopy(true); /* FINCOPY: PCOPY +1                 */
    }
    else if (t->tvcphase == 3)
    {
        s2 = csm_tvc_ycopy(true); /* FINCOPY: YCOPY +1                 */
    }
    else if (t->tvcphase == -1)
    {
        /* TRIM/CMD: TVCDAPON initialisation not completed, e.g. P,YCMD
         * may not be set: DCA PACTOFF; DXCH PCMD                        */
        t->pcmd = c->pactoff;
        t->ycmd = c->yactoff;
    }
    else
    {
        /* 0, 2: just prepare the outcounters; -2: switch-over later    */
    }

    agc_status_t s3 = tvcr_tvc_enable(); /* ENABL1                       */

    if (st == AGC_STATUS_OK)
    {
        st = (s2 != AGC_STATUS_OK) ? s2 : s3; /* copycycle, else ENABL1 */
    }

    return (st);
} /* agc_task_redotvc */

/**********************************************************************
 * ENABL2 (TVCRESTARTS.agc p. 958) -- optics error counter enable
 * (channel 12 bit 2), then a 20 ms (4 ms minimum) wait calling CMDSOUT.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the T5 set-up (NOQRSM)
 **********************************************************************/
agc_status_t agc_task_optics_counter_enable(void)
{
    agc_chan_or(AGC_CHAN_CHAN12, (uint16_t) AGC_CH012_ENBOCDUERR);
    agc_status_t st = csm_dap_set_t5(AGC_TASK_CMDSOUT, T5_20MS); /* T5   */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(agc_rupt_t5_task() == AGC_TASK_CMDSOUT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CHKSTRK, TSTINITJ (TVCRESTARTS.agc p. 959) -- kill a stroke test in
 * progress (STROKER = +0: a new V68 is required), then restart the DAPs
 * at TVCINIT4.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of TVCINIT4
 **********************************************************************/
static agc_status_t tvcr_chkstrk(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    if (t->stroker != 0)
    {
        t->stroker = 0; /* TSTINITJ: disable the stroke test           */
    }

    agc_status_t st = csm_tvc_init_complete(); /* POSTJUMP TVCINIT4    */

    if (!AGC_ASSERT(t->stroker == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(t->tvcphase == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * CMDSOUT, PHSCHK2 (TVCRESTARTS.agc pp. 958-959) -- T5 entry: the most
 * recent actuator commands to the counters and their release (channel
 * 14 bits 11, 12), then by TVCPHASE: >= 0 CHKSTRK; -1 MRCLEAN (repeat
 * the initialisation, no return); -2 SWICHOVR +5 (repeat the V46
 * switch-over) and CHKSTRK.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the first failure of
 *          MRCLEAN, SWICHOVR +5 or TVCINIT4
 **********************************************************************/
agc_status_t agc_task_cmdsout(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables         */
    agc_status_t st = AGC_STATUS_OK;     /* switch-over status        */

    agc_erase_set_tvcpitch(t->pcmd); /* CS ZERO; AD PCMD; TS TVCPITCH  */
    agc_erase_set_tvcyaw(t->ycmd);
    agc_chan_or(AGC_CHAN_CHAN14, CH14_RELEASE);

    if (t->tvcphase == -1)
    {
        return (csm_tvc_mrclean()); /* POSTJUMP MRCLEAN                */
    }

    if (t->tvcphase == -2)
    {
        st = csm_tvc_swichovr_body(); /* IBNKCALL SWICHOVR +5          */
    }

    agc_status_t s2 = tvcr_chkstrk(); /* CHKSTRK                       */

    if (!AGC_ASSERT(t->tvcphase >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st != AGC_STATUS_OK) ? st : s2); /* first failure          */
}

/**********************************************************************
 * EXRSTRT (TVCRESTARTS.agc p. 959) -- waitlist task: restart the TVC
 * executive at the point given by TVCEXPHS (TVCEXADR +1 .. +6).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the executive restart, or
 *          AGC_STATUS_BAD_ARG for a TVCEXPHS outside 1 .. 6
 **********************************************************************/
agc_status_t agc_task_exrstrt(void)
{
    agc_sp_t phs = agc_csm_dap_tvc()->tvcexphs; /* TVCEXPHS, 1 .. 6, unscaled */

    if (!AGC_ASSERT((phs >= 1) && (phs <= 6)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (csm_tvc_exec_restart(phs));
}
