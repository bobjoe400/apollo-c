/**********************************************************************
 * FILE NAME: t5rupt.c
 *
 * PURPOSE:
 *   The T5 interrupt program of the RCS-CSM autopilot: entry, phase
 *   control and initialisation. "Because of the length of the T5
 *   program, it has been divided into three parts, T5PHASE1, T5PHASE2,
 *   and the jet selection logic, to allow for the execution of other
 *   interrupts." Phase 1 (rate filter, matrix timing, automatic steering
 *   increments, FDAI needles) runs every 100 ms; phase 2 (manual and
 *   automatic attitude control, phase plane) 20 ms later; the jet
 *   selection logic (jet_select.c) 20 ms after that. Labels: REDORCS,
 *   RCSATT, SETT5, IMU STATUS CHECK, FREECHK, REINIT, FRESHDAP, REDAP,
 *   ZEROT5, IMUAOK, RCSSWIT, ZEROJET, T5PHASE2, KALUPDT, KRESUME2.
 *
 * AGC SOURCE:
 *   Comanche055 RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1002-1004, 1012,
 *   1014-1016, 1021-1022
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   T5PHASE       enum      I/O  -              phase of the program
 *   T5TIME        agc_sp_t  I/O  10 ms          delay compensation
 *   TIME5, T5LOC  word, task  O  10 ms          next T5 interrupt
 *   TIME6, T6LOC  word, task  O  1/1600 s       ZEROJET's T6 interrupt
 *   RCSFLAGS      uint16_t  I/O  bits           DAP flags
 *   HOLDFLAG      agc_sp_t  O    -              steering mode
 *   ATTKALMN      agc_sp_t  I/O  index          filter gain index
 *   RCS block     struct    O    per field      zeroed at turn-on
 *   IMODES33      uint16_t  I    bits           bit 6: IMU not usable
 *   CHAN31        uint16_t  I    bits           13 hold, 14 free, 15 CMC
 *   CHAN13        uint16_t  O    bits           15 T6RUPT enable
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_csm_dap_decode_dap_data  S41.2 (dapdata.c)
 *   agc_csm_dap_rcs_gains        S40.14 (dapdata.c)
 *   csm_rcs_ratefilt, _kmatrix   phase 1 (filter.c)
 *   csm_rcs_manual               phase 2 (manual.c)
 *   agc_exec_novac               NOVAC AMBGUPDT
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC; the NOVAC status (1202 raised by the executive) and
 *   the T5 set-up status are passed on.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Every function here is the body of a T5RUPT or is called from one;
 *   the AGC's "TCF RESUME" is a return of AGC_STATUS_OK. T5PHASE's four
 *   ones' complement states (+, +0, -, -0) are an enum.
 **********************************************************************/
#include "../internal.h"

/* TIME5 words: T5RUPT occurs when TIME5 overflows 2^14 (10 ms per count). */
#define T5_100MS   ((uint16_t) 037766) /* OCT37766: reinitialise in 100 ms   */
#define T5_80MS    ((uint16_t) 037770) /* DELTATT  80 MS (TIME5)             */
#define T5_20MS    ((uint16_t) 037776) /* DELTATT2 20 MS (TIME5)             */
#define T5_200MS   ((uint16_t) 037754) /* DELAY200 DEC 16364, 200 MS         */
#define T5_60MS    ((uint16_t) 037772) /* T5WAIT60 DEC 16378, 6 CS           */
#define T6_14MS    ((uint16_t) 23)     /* =+14MS DEC 23 (TIME6, 1/1600 s)    */
#define SLOPE_024  ((agc_sp_t) 3932)   /* =.24 DEC .24 (07534): 0.6 deg/s/deg, scale 450/180 */
#define RCSINIT    ((uint16_t) 000004) /* RCSINIT: needles init (BIT3)       */
#define RCSINITB   ((uint16_t) 020004) /* RCSINITB: + BIT14 no rate filter   */
#define ATTKALMN_0 ((agc_sp_t) 11)     /* ELEVEN: Kalman gains for 1.1 s     */
#define T5TIME_0   ((agc_sp_t) 4)      /* FOUR: phase timing allowance, 10 ms */

static agc_status_t rcs_sett5(void);
static agc_status_t rcs_zerot5(void);

/**********************************************************************
 * REDORCS (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1002) -- restart of the
 * autopilot on a T5RUPT: T5PHASE +0, -0 or - is reset to - (REDAP); +
 * is left (FRESHDAP). T5LOC = RCSATT; RCSATT +1.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the RCSATT status
 **********************************************************************/
agc_status_t agc_task_redorcs(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* RCS DAP erasables          */

    if (r->t5phase != AGC_CSM_T5P_INIT)
    {
        r->t5phase = AGC_CSM_T5P_RESTART; /* CS ONE; TS T5PHASE         */
    }

    agc_rupt_set_t5_task(AGC_TASK_RCSATT); /* DCA RCSLOC; DXCH T5LOC     */

    return (agc_task_rcsatt());
}

/**********************************************************************
 * RCSATT (RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1002-1003) -- if the G/C
 * autopilot is fully enabled (channel 31 bit 15 = 0: IMU power on and
 * S/C control switch in CMC) go to SETT5. Otherwise set NORATE,
 * HOLDFLAG +, zero the errors; in free mode go to SETT5, else schedule
 * the reinitialisation (FRESHDAP) in 100 ms, ZEROJET, KMATRIX.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the branch taken
 **********************************************************************/
agc_status_t agc_task_rcsatt(void)
{
    agc_csm_rcs_t    *r    = agc_csm_dap_rcs();              /* RCS DAP erasables  */
    agc_csm_dapcom_t *c    = agc_csm_dap_com();              /* RCSFLAGS           */
    uint16_t          ch31 = agc_chan_read(AGC_CHAN_CHAN31); /* CAF BIT15; RAND    */

    if ((ch31 & (uint16_t) AGC_CH031_CMCCONTROL) == 0u)
    {
        return (rcs_sett5());
    }

    c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NORATE); /* NORATE flag */
    r->holdflag = AGC_SP_MAX;                                     /* CAF POSMAX  */

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        r->error[i] = 0; /* CAF ZERO; TS ERRORX, ERRORY, ERRORZ         */
    }

    if ((ch31 & (uint16_t) AGC_CH031_FREE) == 0u)
    {
        return (rcs_sett5()); /* free mode                              */
    }

    r->t5phase      = AGC_CSM_T5P_INIT;            /* TS T5PHASE (BIT14 > 0) */
    agc_status_t st = csm_dap_set_time5(T5_100MS); /* OCT37766: FRESHDAP     */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_csm_dap_zerojet(); /* TCR ZEROJET: jets off in 14 ms             */

    return (csm_rcs_kmatrix());
}

/**********************************************************************
 * SETT5 (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1003) -- CCS T5PHASE: +
 * FRESHDAP, +0 T5PHASE2, - REDAP, -0 phase 1.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the phase run
 **********************************************************************/
static agc_status_t rcs_sett5(void)
{
    const agc_csm_rcs_t *r  = agc_csm_dap_rcs(); /* T5PHASE              */
    agc_status_t         st = AGC_STATUS_OK;     /* phase status         */

    switch (r->t5phase)
    {
        case AGC_CSM_T5P_INIT:
            st = csm_rcs_freshdap(true);           /* +: turn on autopilot        */
            break;
        case AGC_CSM_T5P_ATTITUDE_CONTROL:
            st = csm_rcs_attitude_control_phase(); /* +0: phase 2      */
            break;
        case AGC_CSM_T5P_RESTART:
            st = csm_rcs_freshdap(false);          /* -: restart autopilot       */
            break;
        case AGC_CSM_T5P_RATE_FILTER:
        default:
            st = csm_rcs_rate_filter_phase();      /* -0: phase 1            */
            break;
    }

    return (st);
}

/**********************************************************************
 * SETT5 -0 branch, IMU STATUS CHECK (RCS-CSM_DIGITAL_AUTOPILOT.agc
 * pp. 1003-1004) -- phase 1: reset for phase 2 in 20 ms (T5TIME =
 * TIME5 for the delay compensation); with the IMU usable (IMODES33
 * bit 6 = 0) the rate filter (unless not initialised), KMATRIX and the
 * needles; otherwise FREECHK.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the phase
 **********************************************************************/
agc_status_t csm_rcs_rate_filter_phase(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* RCS DAP erasables          */

    r->t5phase      = AGC_CSM_T5P_ATTITUDE_CONTROL;                        /* TS T5PHASE: phase 1 reset for 2 */
    r->t5time       = (agc_sp_t) (agc_rupt_time(CSM_DAP_TIME5) & 037777u); /* CA TIME5; TS T5TIME, 10 ms      */
    agc_status_t st = csm_dap_set_time5(T5_20MS);                          /* CAF DELTATT2; TS TIME5          */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if ((agc_erase_imodes33() & CSM_DAP_BIT6) != 0u)
    {
        /* BIT6 = 1: no IMU                     */
        /* FREECHK (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1004) -- no attitude
         * reference: set NORATE and HOLDFLAG (BIT14) to stop any automatic
         * steering and pick up the CDU angles on resumption of attitude hold;
         * in free mode provide free control only (KRESUME1), else REINIT. */
        agc_csm_rcs_t    *r_freechk = agc_csm_dap_rcs(); /* HOLDFLAG                */
        agc_csm_dapcom_t *c         = agc_csm_dap_com(); /* RCSFLAGS                */

        c->rcsflags         = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NORATE); /* CS; MASK BIT14; ADS */
        r_freechk->holdflag = (agc_sp_t) CSM_DAP_BIT14;                       /* CAF BIT14; TS HOLDFLAG */

        if ((agc_chan_read(AGC_CHAN_CHAN31) & (uint16_t) AGC_CH031_FREE) == 0u)
        {
            return (AGC_STATUS_OK); /* KRESUME1: free control only          */
        }

        return (csm_rcs_reinit()); /* ..... TILT ......                     */
    }

    if (!csm_rcs_rate_init_pending())
    {
        st = csm_rcs_ratefilt(); /* RATEFILT (BIT14 clear)              */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (csm_rcs_kmatrix());
}

/**********************************************************************
 * RCSFLAGS BIT14 (RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1004, 1025) -- the
 * rate filter has not been initialised.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true while BIT14 (NORATE) is set
 **********************************************************************/
bool csm_rcs_rate_init_pending(void)
{
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* RCSFLAGS          */

    return ((c->rcsflags & AGC_CSM_RCSF_NORATE) != 0u);
}

/**********************************************************************
 * REINIT (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1012) -- TILT logic:
 * reinitialise the DAP in 200 ms (TIME5 = T5PHASE = DELAY200, positive:
 * FRESHDAP).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the T5 set-up status
 **********************************************************************/
agc_status_t csm_rcs_reinit(void)
{
    agc_csm_rcs_t *r  = agc_csm_dap_rcs();           /* T5PHASE          */
    agc_status_t   st = csm_dap_set_time5(T5_200MS); /* CAF DELAY200; TS TIME5 */

    r->t5phase = AGC_CSM_T5P_INIT; /* TS T5PHASE (16364 > 0)            */

    return (st);
}

/**********************************************************************
 * FRESHDAP, REDAP (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1014) -- FRESHDAP
 * resets HOLDFLAG (+1) to stop automatic steering and pick up an
 * attitude hold reference; both decode DAPDATR1/2 (S41.2) and IXX, IAVG
 * (S40.14), then ZEROT5.
 *
 * ARGUMENTS:
 *   fresh  bool  I  true: FRESHDAP, false: REDAP
 *
 * RETURNS: agc_status_t -- the first failure of S41.2 or ZEROT5
 **********************************************************************/
agc_status_t csm_rcs_freshdap(bool fresh)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* HOLDFLAG                   */

    if (fresh)
    {
        r->holdflag = 1; /* CAF ONE; TS HOLDFLAG                        */
    }

    agc_status_t st = agc_csm_dap_decode_dap_data(); /* IBNKCALL S41.2  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_csm_dap_rcs_gains(); /* IBNKCALL S40.14                */

    return (rcs_zerot5());
}

/**********************************************************************
 * ZEROT5, IMUAOK, RCSSWIT (RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1014-1015)
 * -- zero the NO.T5VAR + 1 = 37 words from WBODY (WBODY, ADOT, MERRORX,
 * DFT, DRHO, ATTSEC, TAU, BIAS, ERRORX), ZEROJET, initialise the minimum
 * impulse and RHC memories, SLOPE, T5TIME, ATTKALMN (11: Kalman gains),
 * RHO = CDU, T5PHASE +0; with the IMU in fine align start the matrix
 * (NOVAC AMBGUPDT) and RCSFLAGS = RCSINIT, else ATTKALMN 0 and RCSINITB
 * (free control only); next T5RUPT in 60 ms.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the NOVAC or T5 set-up status
 **********************************************************************/
static agc_status_t rcs_zerot5(void)
{
    agc_csm_rcs_t    *r     = agc_csm_dap_rcs(); /* RCS DAP erasables    */
    agc_csm_dapcom_t *c     = agc_csm_dap_com(); /* RCSFLAGS             */
    uint16_t          flags = RCSINIT;           /* IMUAOK: CAF RCSINIT  */

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        r->wbody[i]  = 0;
        r->adot[i]   = (agc_dp_words_t){0, 0};
        r->merror[i] = (agc_dp_words_t){0, 0};
        r->dft[i]    = 0;
        r->drho[i]   = (agc_dp_words_t){0, 0};
        r->tau[i]    = 0;
        r->bias[i]   = 0;
        r->error[i]  = 0;
    }

    r->attsec   = 0;
    agc_csm_dap_zerojet();      /* TCR ZEROJET                       */
    r->chantemp = CSM_DAP_NEG0; /* CS ZERO: minimum impulse memory   */
    r->ch31temp = CSM_DAP_NEG0; /* RHC position memory               */
    r->slope    = SLOPE_024;    /* CAF =.24; TS SLOPE                */
    r->t5time   = T5TIME_0;     /* phase 1 cycles 100 ms from now    */
    r->attkalmn = ATTKALMN_0;   /* CAF ELEVEN; TS ATTKALMN           */

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        r->rho[i] = csm_dap_cdu(i); /* CA CDUX; TS RHO .. RHO2             */
    }

    r->t5phase = AGC_CSM_T5P_ATTITUDE_CONTROL; /* CAF ZERO; TS T5PHASE */

    if ((agc_erase_imodes33() & CSM_DAP_BIT6) != 0u)
    {
        r->attkalmn = 0;        /* TS ATTKALMN: cannot use IMU           */
        flags       = RCSINITB; /* free control only, no rate filter     */
    }
    else
    {
        /* IMUAOK: CAF PRIO34; TC NOVAC; 2CADR AMBGUPDT (a 1202 is raised
         * by the executive)                                              */
        agc_status_t sw = agc_exec_novac(CSM_DAP_PRIO34, AGC_JOB_AMBGUPDT); /* NOVAC status */

        if (sw != AGC_STATUS_OK)
        {
            return (sw);
        }
    }

    c->rcsflags = flags; /* RCSSWIT: TS RCSFLAGS                         */

    return (csm_dap_set_time5(T5_60MS)); /* CAF T5WAIT60; TS TIME5       */
} /* rcs_zerot5 */

/**********************************************************************
 * ZEROJET (RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1015-1016) -- zero BLAST2,
 * BLAST1, BLAST, YWORD2 .. RWORD1, restore the branch indices (BLAST1
 * +1 = 4, BLAST2 +1 = 11), clear RCSFLAGS BIT1 and enable T6RUPT to
 * shut the jets off in 14 ms (T6START writes the zero words to channels
 * 5 and 6).
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_csm_dap_zerojet(void)
{
    agc_csm_rcs_t    *r = agc_csm_dap_rcs(); /* jet words, BLASTs        */
    agc_csm_dapcom_t *c = agc_csm_dap_com(); /* RCSFLAGS                 */

    r->rword1        = 0u;
    r->rword2        = 0u;
    r->pword1        = 0u;
    r->pword2        = 0u;
    r->yword1        = 0u;
    r->yword2        = 0u;
    r->blast[0].time = 0;
    r->blast[0].axis = AGC_CSM_JET_ROLL;
    r->blast[1].time = 0;
    r->blast[1].axis = AGC_CSM_JET_PITCH;                         /* CAF FOUR; TS BLAST1 +1      */
    r->blast[2].time = 0;
    r->blast[2].axis = AGC_CSM_JET_YAW;                           /* CAF ELEVEN; TS BLAST2 +1    */
    c->rcsflags      = (uint16_t) (c->rcsflags & ~(unsigned int) AGC_CSM_RCSF_T6INIT);
    agc_rupt_set_t6_task(AGC_TASK_T6START);                       /* DCA T6ADDR; DXCH T6LOC */
    agc_rupt_set_time(CSM_DAP_TIME6, T6_14MS);                    /* CAF =+14MS; TS TIME6   */
    agc_chan_or(AGC_CHAN_CHAN13, (uint16_t) AGC_CH013_ENBT6RUPT); /* CAF BIT15; WOR CHAN13  */
}

/**********************************************************************
 * T5PHASE2, KALUPDT, KRESUME2 (RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1016,
 * 1021-1022) -- while ATTKALMN is positive the rate estimate is being
 * initialised: count it down and resume phase 1 in 1 s - 80 ms + T5TIME
 * (20 ms if that overflows). Otherwise phase 3 in 20 ms, T5TIME += the
 * old TIME5, ATTKALMN -1 for a high-rate automatic maneuver (RCSFLAGS
 * BIT15) else 0, and the manual rotation commands.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the T5 set-up or phase 2 status
 **********************************************************************/
agc_status_t csm_rcs_attitude_control_phase(void)
{
    agc_csm_rcs_t          *r = agc_csm_dap_rcs(); /* RCS DAP erasables  */
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* RCSFLAGS           */

    if (r->attkalmn > 0)
    { /* CCS ATTKALMN: KALUPDT */
        r->attkalmn = (agc_sp_t) (r->attkalmn - 1);              /* TS ATTKALMN                  */
        int      t  = (int) T5_80MS + (int) r->t5time;           /* CA DELTATT; AD T5TIME, 10 ms */
        uint16_t w  = (t > AGC_SP_MAX) ? T5_20MS : (uint16_t) t; /* ?: TS TIME5 overflow: DELTATT2 */

        r->t5phase = AGC_CSM_T5P_RATE_FILTER; /* KRESUME2: CS ZERO; TS T5PHASE */

        return (csm_dap_set_time5(w));
    }

    uint16_t     old = agc_rupt_time(CSM_DAP_TIME5); /* CA DELTATT2; XCH TIME5 */
    agc_status_t st  = csm_dap_set_time5(T5_20MS);   /* phase 3 in 20 ms       */

    r->t5time   = csm_dap_ads(r->t5time, csm_dap_word_sp(old));         /* ADS T5TIME, 10 ms */
    r->attkalmn = ((c->rcsflags & AGC_CSM_RCSF_HIRATE) != 0u) ? -1 : 0; /* ?: NOHIAUTO */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (csm_rcs_manual());
}
