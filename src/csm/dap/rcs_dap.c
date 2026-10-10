/*
 * rcs_dap.c -- T5 interrupt program of the RCS-CSM autopilot: entry,
 * phase control and initialisation.
 * RCS-CSM_DIGITAL_AUTOPILOT.agc (Comanche055 pp. 1002-1004, 1012, 1014-1016,
 * 1021-1022): REDORCS, RCSATT, SETT5, FREECHK, REINIT, FRESHDAP, REDAP,
 * ZEROT5, IMUAOK, RCSSWIT, ZEROJET, T5PHASE2, KALUPDT.
 *
 * "Because of the length of the T5 program, it has been divided into
 * three parts, T5PHASE1, T5PHASE2, and the jet selection logic, to allow
 * for the execution of other interrupts." Phase 1 (rate filter, matrix
 * timing, automatic steering increments, FDAI needles) runs every 100 ms;
 * phase 2 (manual and automatic attitude control, phase plane) 20 ms
 * later; the jet selection logic (jet_select.c) 20 ms after that.
 *
 * Every function here is the body of a T5RUPT or is called from one; the
 * AGC's "TCF RESUME" is a return of AGC_STATUS_OK.
 */
#include "csm_dap_internal.h"

/* TIME5 words: T5RUPT occurs when TIME5 overflows 2^14 (10 ms per count). */
#define T5_100MS   ((uint16_t)037766) /* OCT37766: reinitialise in 100 ms   */
#define T5_80MS    ((uint16_t)037770) /* DELTATT  80 MS (TIME5)             */
#define T5_20MS    ((uint16_t)037776) /* DELTATT2 20 MS (TIME5)             */
#define T5_200MS   ((uint16_t)037754) /* DELAY200 DEC 16364, 200 MS         */
#define T5_60MS    ((uint16_t)037772) /* T5WAIT60 DEC 16378, 6 CS           */
#define T6_14MS    ((uint16_t)23)     /* =+14MS DEC 23 (TIME6, 1/1600 s)    */
#define SLOPE_024  ((agc_sp_t)3932)   /* =.24 DEC .24 (07534): 0.6 deg/s/deg */
#define RCSINIT    ((uint16_t)000004) /* RCSINIT: needles init (BIT3)       */
#define RCSINITB   ((uint16_t)020004) /* RCSINITB: + BIT14 no rate filter   */
#define ATTKALMN_0 ((agc_sp_t)11)     /* ELEVEN: Kalman gains for 1.1 s     */
#define T5TIME_0   ((agc_sp_t)4)      /* FOUR: phase timing allowance       */

static agc_status_t rcs_sett5(void);
static agc_status_t rcs_freechk(void);
static agc_status_t rcs_zerot5(void);

/* REDORCS (p. 1002): restart of the autopilot comes here on a T5RUPT.
 * T5PHASE +0, -0 or - is reset to - (REDAP); + is left (FRESHDAP).       */
agc_status_t agc_task_redorcs(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    (void)AGC_ASSERT(r->t5phase <= AGC_CSM_T5P_INIT);
    if (r->t5phase != AGC_CSM_T5P_INIT) {
        r->t5phase = AGC_CSM_T5P_RESTART;
    }
    agc_status_t st = agc_rupt_set_t5_task(AGC_TASK_RCSATT); /* RCSLOC  */
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    return agc_task_rcsatt();                   /* TCF RCSATT +1          */
}

/* RCSATT (pp. 1002-1003): check that the G/C autopilot is fully enabled
 * (channel 31 bit 15 = 0: IMU power on and S/C control switch in CMC).   */
agc_status_t agc_task_rcsatt(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_csm_dapcom_t *c = agc_csm_dap_com();
    uint16_t ch31 = agc_chan_read(AGC_CHAN_CHAN31);
    (void)AGC_ASSERT(ch31 <= CSM_DAP_NEG0);
    if ((ch31 & (uint16_t)AGC_CH031_CMCCONTROL) == 0u) {
        return rcs_sett5();
    }
    c->rcsflags = (uint16_t)(c->rcsflags | AGC_CSM_RCSF_NORATE); /* NORATE */
    r->holdflag = AGC_SP_MAX;                   /* HOLDFLAG +             */
    for (int i = 0; i < AGC_CSM_AXES; i++) {
        r->error[i] = 0;                        /* ERRORX, Y, Z           */
    }
    if ((ch31 & (uint16_t)AGC_CH031_FREE) == 0u) {
        return rcs_sett5();                     /* free mode              */
    }
    r->t5phase = AGC_CSM_T5P_INIT;              /* TS T5PHASE (BIT14 > 0) */
    agc_status_t st = csm_dap_set_time5(T5_100MS); /* FRESHDAP in 100 ms  */
    if (st == AGC_STATUS_OK) {
        st = agc_csm_dap_zerojet();             /* jets off in 14 ms      */
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return csm_rcs_kmatrix();
}

/* SETT5 (p. 1003): CCS T5PHASE.                                          */
static agc_status_t rcs_sett5(void)
{
    const agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_status_t st = AGC_STATUS_BAD_ARG;
    switch (r->t5phase) {
    case AGC_CSM_T5P_INIT:    st = csm_rcs_freshdap(true);  break; /* +  */
    case AGC_CSM_T5P_PHASE2:  st = csm_rcs_t5phase2();      break; /* +0 */
    case AGC_CSM_T5P_RESTART: st = csm_rcs_freshdap(false); break; /* -  */
    case AGC_CSM_T5P_PHASE1:  st = csm_rcs_phase1();        break; /* -0 */
    default:
        (void)AGC_ASSERT(false);
        break;
    }
    return st;
}

/* SETT5 -0 branch and IMU STATUS CHECK (pp. 1003-1004): phase 1.         */
agc_status_t csm_rcs_phase1(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    r->t5phase = AGC_CSM_T5P_PHASE2;            /* phase 1 reset for 2    */
    r->t5time = (agc_sp_t)(agc_rupt_time(CSM_DAP_TIME5) & 037777u); /* T5TIME */
    agc_status_t st = csm_dap_set_time5(T5_20MS); /* phase 2 in 20 ms     */
    (void)AGC_ASSERT(r->t5time >= 0);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if ((agc_erase_imodes33() & CSM_DAP_BIT6) != 0u) {
        return rcs_freechk();                   /* BIT6 = 1: no IMU       */
    }
    if (!csm_rcs_rate_init_pending()) {         /* RATEFILT               */
        st = csm_rcs_ratefilt();
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return csm_rcs_kmatrix();
}

/* FREECHK (p. 1004): no attitude reference.                              */
static agc_status_t rcs_freechk(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_csm_dapcom_t *c = agc_csm_dap_com();
    c->rcsflags = (uint16_t)(c->rcsflags | AGC_CSM_RCSF_NORATE);
    r->holdflag = (agc_sp_t)CSM_DAP_BIT14;      /* stop automatic steering */
    uint16_t ch31 = agc_chan_read(AGC_CHAN_CHAN31);
    (void)AGC_ASSERT(r->holdflag > 0);
    if ((ch31 & (uint16_t)AGC_CH031_FREE) == 0u) {
        return AGC_STATUS_OK;                   /* KRESUME1: free control */
    }
    return csm_rcs_reinit();                    /* ..... TILT ............ */
}

/* RCSFLAGS BIT14: the rate filter has not been initialised.              */
bool csm_rcs_rate_init_pending(void)
{
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    (void)AGC_ASSERT(c->rcsflags <= CSM_DAP_NEG0);
    return (c->rcsflags & AGC_CSM_RCSF_NORATE) != 0u;
}

/* REINIT (p. 1012): ........ TILT LOGIC: reinitialise DAP in 200 ms.     */
agc_status_t csm_rcs_reinit(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_status_t st = csm_dap_set_time5(T5_200MS);
    r->t5phase = AGC_CSM_T5P_INIT;              /* TS T5PHASE (16364 > 0) */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;
}

/* FRESHDAP / REDAP (p. 1014): FRESHDAP resets HOLDFLAG to stop automatic
 * steering and pick up an attitude hold reference; both decode DAPDATR1/2
 * (S41.2) and IXX, IAVG (S40.14) and zero the autopilot variables.       */
agc_status_t csm_rcs_freshdap(bool fresh)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    if (fresh) {
        r->holdflag = 1;                        /* CAF ONE; TS HOLDFLAG   */
    }
    agc_status_t st = agc_csm_dap_s41_2();
    if (st == AGC_STATUS_OK) {
        st = agc_csm_dap_s40_14();
    }
    if (st == AGC_STATUS_OK) {
        st = rcs_zerot5();
    }
    (void)AGC_ASSERT(r->holdflag != 0 || !fresh);
    return st;
}

/* ZEROT5 .. RCSSWIT (pp. 1014-1015).                                     */
static agc_status_t rcs_zerot5(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_csm_dapcom_t *c = agc_csm_dap_com();
    for (int i = 0; i < AGC_CSM_AXES; i++) {    /* NO.T5VAR + 1 = 37 words */
        r->wbody[i] = 0;  r->adot[i] = 0;  r->merror[i] = 0;
        r->dft[i] = 0;    r->drho[i] = 0;  r->tau[i] = 0;
        r->bias[i] = 0;   r->error[i] = 0;
    }
    r->attsec = 0;
    agc_status_t st = agc_csm_dap_zerojet();
    r->chantemp = CSM_DAP_NEG0;                 /* minimum impulse memory */
    r->ch31temp = CSM_DAP_NEG0;                 /* RHC position memory    */
    r->slope = SLOPE_024;
    r->t5time = T5TIME_0;   /* phase 1 begins cycling 100 ms from now     */
    r->attkalmn = ATTKALMN_0;
    for (int i = 0; i < AGC_CSM_AXES; i++) {
        r->rho[i] = csm_dap_cdu(i);             /* RHO, RHO1, RHO2        */
    }
    r->t5phase = AGC_CSM_T5P_PHASE2;
    uint16_t flags = RCSINIT;                   /* IMUAOK                 */
    if ((agc_erase_imodes33() & CSM_DAP_BIT6) != 0u) {
        r->attkalmn = 0;                        /* cannot use IMU         */
        flags = RCSINITB;                       /* free control only      */
    } else {
        /* NOVAC returns after scheduling; a 1202 is raised by the exec.  */
        (void)agc_exec_novac(CSM_DAP_PRIO34, AGC_JOB_AMBGUPDT);
    }
    c->rcsflags = flags;                        /* RCSSWIT                */
    agc_status_t st2 = csm_dap_set_time5(T5_60MS); /* IMU error counter   */
    (void)AGC_ASSERT(r->attkalmn >= 0);
    return (st != AGC_STATUS_OK) ? st : st2;
}

/* ZEROJET (pp. 1015-1016): zero BLAST2, BLAST1, BLAST, YWORD2 .. RWORD1,
 * restore the branch indices and enable T6RUPT to shut the jets off in
 * 14 ms (T6START writes the zero words to channels 5 and 6).             */
agc_status_t agc_csm_dap_zerojet(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_csm_dapcom_t *c = agc_csm_dap_com();
    r->rword1 = 0u; r->rword2 = 0u;
    r->pword1 = 0u; r->pword2 = 0u;
    r->yword1 = 0u; r->yword2 = 0u;
    r->blast[0].time = 0; r->blast[0].axis = AGC_CSM_JET_ROLL;
    r->blast[1].time = 0; r->blast[1].axis = AGC_CSM_JET_PITCH;
    r->blast[2].time = 0; r->blast[2].axis = AGC_CSM_JET_YAW;
    c->rcsflags = (uint16_t)(c->rcsflags & ~(unsigned)AGC_CSM_RCSF_T6INIT);
    agc_status_t st = agc_rupt_set_t6_task(AGC_TASK_T6START); /* T6ADDR   */
    if (st == AGC_STATUS_OK) {
        st = agc_rupt_set_time(CSM_DAP_TIME6, T6_14MS);
    }
    if (st == AGC_STATUS_OK) {
        st = agc_chan_or(AGC_CHAN_CHAN13, (uint16_t)AGC_CH013_ENBT6RUPT);
    }
    (void)AGC_ASSERT((c->rcsflags & AGC_CSM_RCSF_T6INIT) == 0u);
    return st;
}

/* T5PHASE2 (p. 1016) and KALUPDT (pp. 1021-1022).                        */
agc_status_t csm_rcs_t5phase2(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    if (r->attkalmn > 0) {                      /* initialise rate estimate */
        r->attkalmn = (agc_sp_t)(r->attkalmn - 1); /* KALUPDT, 1.1 s total */
        int t = (int)T5_80MS + (int)r->t5time;  /* + delays               */
        uint16_t w = (t > AGC_SP_MAX) ? T5_20MS : (uint16_t)t; /* safety  */
        r->t5phase = AGC_CSM_T5P_PHASE1;        /* KRESUME2               */
        return csm_dap_set_time5(w);
    }
    uint16_t old = agc_rupt_time(CSM_DAP_TIME5); /* CA DELTATT2; XCH TIME5 */
    agc_status_t st = csm_dap_set_time5(T5_20MS); /* phase 3 in 20 ms     */
    r->t5time = csm_dap_ads(r->t5time, csm_dap_word_sp(old)); /* ADS T5TIME */
    r->attkalmn = ((c->rcsflags & AGC_CSM_RCSF_HIRATE) != 0u) ? -1 : 0;
    (void)AGC_ASSERT(r->attkalmn <= 0);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return csm_rcs_manual();
}
