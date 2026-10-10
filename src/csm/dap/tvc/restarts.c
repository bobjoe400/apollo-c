/*
 * restarts.c -- the TVC restart package.
 * TVCRESTARTS.agc (Comanche055 pp. 956-960): REDOTVC, EXECPHS, TVCDAPHS,
 * ENABL1, ENABL2, CMDSOUT, PHSCHK2, CHKSTRK, FINCOPY, TRIM/CMD, TSTINITJ,
 * EXRSTRT and the restart tables TVCCADR / TVCEXADR.
 *
 * Restart-proofs the pitch and yaw DAPs, the TVC executive, the roll DAP,
 * TVCINIT4, TVCDAPON and the CSM/LM V46 switch-over. Quantities that must
 * be protected are kept in temporaries and copied in copycycles bracketed
 * by phase points (INCR TVCEXPHS in the executive, INCR TVCPHASE in the
 * pitch and yaw DAPs). An interrupted copycycle is completed; then TVC
 * enable and the optics error counter enable are set as soon as possible,
 * the actuators are commanded to the last commands, and the DAPs go back
 * on the air through TVCINIT4 (or MRCLEAN for a restart before TVCINIT4,
 * TVCPHASE = -1). TVCPHASE = -2 repeats the switch-over. A stroke test in
 * progress is terminated (a new V68 is required). REDOTVC is reached by
 * T5 from ELRSKIP after any restart that finds FLAGWRD6 bits 15,14 = 10.
 */
#include "../internal.h"

#define EXRSTRT_DELAY ((uint16_t)9)      /* NINE cs: before PITCHDAP, after CMDSOUT */
#define T5_60MS       ((uint16_t)037772) /* TVCEXADR: 60 ms                 */
#define T5_20MS       ((uint16_t)037776) /* OCT37776: 20 ms                 */
#define CH12_ENABL1   ((uint16_t)(AGC_CH012_TVCENABLE | CSM_DAP_BIT11))
                                         /* BIT8 TVC enable, BIT11 optics DAC enable */
#define CH14_RELEASE  ((uint16_t)(AGC_CH014_DRIVECDUS | AGC_CH014_DRIVECDUT))
                                         /* PRIO6: release counters (bits 11,12) */

/* ENABL1 (p. 958): TVC enable and optics-DAC enable, then 60 ms (at
 * least 40 ms) wait calling ENABL2.                                      */
static agc_status_t tvcr_enabl1(void)
{
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN12, CH12_ENABL1);
    if (st == AGC_STATUS_OK) {
        st = csm_dap_set_t5(AGC_TASK_ENABL2, T5_60MS);
    }
    (void)AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_TVCENABLE) != 0u);
    return st;                                     /* RESUME               */
}

/* REDOTVC, EXECPHS, TVCDAPHS, FINCOPY, TRIM/CMD (pp. 957-958).           */
agc_status_t agc_task_redotvc(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    agc_status_t st = AGC_STATUS_OK;
    if (t->tvcexphs > 0) {         /* must restart the TVC executive      */
        st = agc_waitlist_call(EXRSTRT_DELAY, AGC_TASK_EXRSTRT);
    }
    /* TVCPHASE bits 15 and 1 select the DAP restart location.           */
    if (t->tvcphase == 1) {        /* FINCOPY: pitch copycycle (PCOPY +1) */
        agc_status_t s2 = csm_tvc_pcopy(true);
        st = (st != AGC_STATUS_OK) ? st : s2;
    } else if (t->tvcphase == 3) { /* FINCOPY: yaw copycycle (YCOPY +1)   */
        agc_status_t s2 = csm_tvc_ycopy(true);
        st = (st != AGC_STATUS_OK) ? st : s2;
    } else if (t->tvcphase == -1) {
        /* TRIM/CMD: TVCDAPON initialisation not completed, e.g. P,YCMD
         * may not be set: set them to the trims.                         */
        t->pcmd = c->pactoff;
        t->ycmd = c->yactoff;
    } else {
        (void)AGC_ASSERT((t->tvcphase == 0) || (t->tvcphase == 2) ||
                         (t->tvcphase == -2));
    }
    agc_status_t s3 = tvcr_enabl1();
    return (st != AGC_STATUS_OK) ? st : s3;
}

/* ENABL2 (p. 958): optics error counter enable, 20 ms (4 ms minimum)
 * wait calling CMDSOUT.                                                  */
agc_status_t agc_task_enabl2(void)
{
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN12, (uint16_t)AGC_CH012_ENBOCDUERR);
    if (st == AGC_STATUS_OK) {
        st = csm_dap_set_t5(AGC_TASK_CMDSOUT, T5_20MS);
    }
    (void)AGC_ASSERT(agc_rupt_t5_task() == AGC_TASK_CMDSOUT);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;                                     /* NOQRSM               */
}

/* CHKSTRK, TSTINITJ (p. 959): kill a stroke test in progress (STROKER =
 * +0: a new V68 is required), then restart the DAPs at TVCINIT4.          */
static agc_status_t tvcr_chkstrk(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    if (t->stroker != 0) {
        t->stroker = 0;                            /* TSTINITJ             */
    }
    agc_status_t st = csm_tvc_init4();             /* POSTJUMP TVCINIT4    */
    (void)AGC_ASSERT(t->stroker == 0);
    (void)AGC_ASSERT(t->tvcphase == 0);
    return st;
}

/* CMDSOUT, PHSCHK2 (pp. 958-959): the most recent actuator commands to
 * the counters, release them (channel 14 bits 11, 12), then continue by
 * TVCPHASE: >= 0 CHKSTRK, -1 MRCLEAN (full initialisation, no return),
 * -2 repeat the V46 switch-over (SWICHOVR +5) and CHKSTRK.
 * DEVIATION: the AGC writes -0 + PCMD to avoid a +0 command word; -0 is
 * not represented, so a zero command is written as 0.                    */
agc_status_t agc_task_cmdsout(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_erase_set_tvcpitch(t->pcmd);
    agc_erase_set_tvcyaw(t->ycmd);
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN14, CH14_RELEASE);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (t->tvcphase == -1) {
        return csm_tvc_mrclean();                  /* repeat initialisation */
    }
    if (t->tvcphase == -2) {
        st = csm_tvc_swichovr_body();              /* repeat switch-over   */
    }
    agc_status_t s2 = tvcr_chkstrk();
    (void)AGC_ASSERT(t->tvcphase >= 0);
    return (st != AGC_STATUS_OK) ? st : s2;
}

/* EXRSTRT (p. 959): waitlist task, TVC executive restart at the point
 * given by TVCEXPHS (TVCEXADR +1 .. +6).                                 */
agc_status_t agc_task_exrstrt(void)
{
    agc_sp_t phs = agc_csm_dap_tvc()->tvcexphs;
    if (!AGC_ASSERT((phs >= 1) && (phs <= 6))) {
        return AGC_STATUS_BAD_ARG;
    }
    return csm_tvc_exec_restart(phs);
}
