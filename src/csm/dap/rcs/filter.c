/**********************************************************************
 * FILE NAME: filter.c
 *
 * PURPOSE:
 *   Phase 1 of the RCS-CSM autopilot T5 program: the attitude rate
 *   filter, the 1-s gimbal-to-body matrix update request, the automatic
 *   steering increments and the FDAI attitude error display. Labels:
 *   RATEFILT, DRHOLOOP, ADOTLOOP, KMATRIX, TENTHSEK, DCDUINCR, DELOOP,
 *   DACNDLS, FDAIDSP1, KRESUME1, FDAIDSP2, FDAITOTL, WRTN22, WRTN17,
 *   GETAKS; tables GAIN1, GAIN2.
 *
 * AGC SOURCE:
 *   Comanche055 RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1004-1009, 1021-1023
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type            I/O  Scale        Description
 *   ------------  --------------  ---  -----------  ------------------
 *   DRHO..DRHO2   agc_dp_words_t  I/O  180 deg      filter residual
 *   ADOT..ADOT2   agc_dp_words_t  I/O  450 deg/s    estimated body rate
 *   MERRORX..Z    agc_dp_words_t  I/O  180 deg      manual att. error
 *   RHO..RHO2     uint16_t        I/O  CDU counts   CDUs of last pass
 *   CDUX..CDUZ    uint16_t        I    CDU counts   gimbal angles
 *   AMGB1..AMGB8  agc_sp_t        I    1            gimbal-to-body
 *   KMJ, DFT      agc_sp_t        I    1/1.3, 1/1600 s  jet torque term
 *   ATTKALMN      agc_sp_t        I    index        filter gain index
 *   ATTSEC        agc_sp_t        I/O  0.1 s        matrix update count
 *   CDUXD, DELCDUX agc_dp_t       I/O  180 deg      steering commands
 *   THETADX..Z    uint16_t        O    180 deg      hold references
 *   AK..AK2       agc_sp_t        O    180 deg      FDAI needle errors
 *   RCSFLAGS      uint16_t        I/O  bits         BIT4 alternation
 *   NEEDLFLG, N22ORN17  flags     I    -            display mode
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   csm_dap_words_*       DAS, MP, SMALLMP on stored words (mysubs.c)
 *   csm_dap_msu, _ads,    basic-language word arithmetic (mysubs.c)
 *   _mp_hi, _dpadd
 *   csm_dap_needler       NEEDLER (needles.c)
 *   agc_exec_novac        NOVAC AMBGUPDT (PRIO34)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC; the 1202 of a full core-set table is raised by the
 *   executive. AGC_STATUS_BAD_ARG when an assertion fails.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs from the T5 interrupt every 0.1 s (T5PHASE -0), interrupts
 *   inhibited. RATEFILT is bypassed until RCSFLAGS BIT14 is cleared.
 *
 * NOTES:
 *   Rate filter equations (p. 1004):
 *     DRHO = DELRHO - (.1)ADOT     + (1 - GAIN1) DRHO
 *                             -1                     -1
 *     ADOT = ADOT    + GAIN2 DRHO + KMJ DFT
 *                -1
 *     where DELRHO = AMGB (CDU - CDU  )
 *                                   -1
 *   QUARTER (BIT13 = 1/4) times ADOT at 450 deg/s is .1 s of rate at
 *   180 deg. DRHO, ADOT and MERRORX are kept as the AGC stores them:
 *   DAS never makes the two signs agree, and DRHO's and MERRORX's upper
 *   words are later used alone (DOUBLE; MP GAIN2, SPNDXCHK).
 **********************************************************************/
#include "../internal.h"

#define QUARTER   ((agc_sp_t) 4096) /* QUARTER = BIT13 (010000): 1/4      */
#define LOW4      ((agc_sp_t) 017)  /* LOW4                               */
#define NINE      ((agc_sp_t) 9)    /* ATTSEC: 1 s matrix update cycle    */
#define GAIN_BIAS 3                 /* GAIN1 -3 .. GAIN1 +10              */
#define GAIN_N    14                /* entries in GAIN1, GAIN2            */

/* GAIN1 -3 .. +10 (p. 1021): (1 - GAIN1) multiplies DRHO. Octal, decimal: */
static const agc_sp_t gain1[GAIN_N] = {
    3460,  /* 06604 .2112 translation, LEM on                          */
    13763, /* 32703 .8400 translation 2(ZETA)WN DT                     */
    3460,  /* 06604 .2112 2 degrees/sec maneuvers                      */
    1049,  /* 02031 .0640  Kalman filter gains for initialization      */
    5210,  /* 12132 .3180  of attitude rates (ATTKALMN 1 .. 10)        */
    5656,  /* 13030 .3452 */
    6183,  /* 14047 .3774 */
    6817,  /* 15241 .4161 */
    7592,  /* 16650 .4634 */
    8557,  /* 20555 .5223 */
    9781,  /* 23065 .5970 */
    11359, /* 26137 .6933 */
    13355, /* 32053 .8151 */
    15306  /* 35712 .9342 */
};

/* GAIN2 -3 .. +10 (p. 1021), scaled 10 (DRHO is doubled twice first).    */
static const agc_sp_t gain2[GAIN_N] = {
    285,  /* 00435 .0174 translation, LEM on                          */
    5898, /* 13412 .3600 translation (WN)(WN)DT                       */
    285,  /* 00435 .0174 2 degrees/sec maneuvers                      */
    26,   /* 00032 .0016 */
    744,  /* 01350 .0454 */
    893,  /* 01575 .0545 */
    1091, /* 02103 .0666 */
    1363, /* 02523 .0832 */
    1751, /* 03327 .1069 */
    2330, /* 04432 .1422 */
    3252, /* 06264 .1985 */
    4841, /* 11351 .2955 */
    7892, /* 17324 .4817 */
    14226 /* 33622 .8683 */
};

static void rcs_delrho(agc_dp_words_t deltemp[AGC_CSM_AXES]);

/**********************************************************************
 * RATEFILT, DRHOLOOP, ADOTLOOP (RCS-CSM_DIGITAL_AUTOPILOT.agc
 * pp. 1004-1006) -- one pass of the rate filter, axes Z, Y, X:
 *   DRHO += -DRHO(upper) GAIN1;  DRHO += SMALLMP(-ADOT, QUARTER);
 *   DELTEMP = AMGB (CDU - RHO);  DRHO += DELTEMP;  MERRORX += DELTEMP;
 *   ADOT += 4 DRHO(upper) GAIN2 + KMJ DFT.
 * Every sum is a DAS on the stored words.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when
 *          ATTKALMN is outside the gain tables (the +0 gains are used)
 **********************************************************************/
agc_status_t csm_rcs_ratefilt(void)
{
    agc_csm_rcs_t *r  = agc_csm_dap_rcs();             /* RCS DAP erasables       */
    int            g  = (int) r->attkalmn + GAIN_BIAS; /* INDEX ATTKALMN  */
    agc_status_t   st = AGC_STATUS_OK;                 /* gain table status */
    agc_dp_words_t deltemp[AGC_CSM_AXES];              /* DELTEMPX..Z, scale 180 deg */

    if (!AGC_ASSERT((g >= 0) && (g < GAIN_N)))
    {
        g  = GAIN_BIAS; /* ATTKALMN out of table: use +0          */
        st = AGC_STATUS_BAD_ARG;
    }

    for (int i = AGC_CSM_AXES - 1; i >= 0; i--)
    { /* DRHOLOOP */
        /* CS DRHO; MP GAIN1; DAS DRHO: (1 - GAIN1) DRHO                  */
        csm_dap_words_das(&r->drho[i], csm_dap_words_mp((agc_sp_t) -r->drho[i].hi, gain1[g]));

        /* DCS ADOT; DXCH KMPAC; CA QUARTER; TC SMALLMP; DXCH KMPAC; DAS  */
        csm_dap_words_das(&r->drho[i], csm_dap_words_smallmp(csm_dap_words_neg(r->adot[i]), QUARTER));
    }

    rcs_delrho(deltemp);

    for (int i = AGC_CSM_AXES - 1; i >= 0; i--)
    { /* ADOTLOOP */
        csm_dap_words_das(&r->drho[i], deltemp[i]);   /* DCA DELTEMPX; DAS DRHO   */
        csm_dap_words_das(&r->merror[i], deltemp[i]); /* DAS MERRORX              */

        /* CA DRHO; DOUBLE; DOUBLE (N.B.); MP GAIN2; DAS ADOT             */
        agc_sp_t d = csm_dap_ads(r->drho[i].hi, r->drho[i].hi); /* 2 DRHO, scale 90 deg */

        d = csm_dap_ads(d, d);
        csm_dap_words_das(&r->adot[i], csm_dap_words_mp(d, gain2[g]));

        /* CA KMJ; MP DFT; DAS ADOT: torque to inertia times firing time  */
        csm_dap_words_das(&r->adot[i], csm_dap_words_mp(r->kmj[i], r->dft[i]));
    }

    return (st);
}

/**********************************************************************
 * RATEFILT, measured body rates (RCS-CSM_DIGITAL_AUTOPILOT.agc
 * p. 1005) -- DELRHO = AMGB (CDU - RHO), double precision "or will lose
 * pulses"; RHO, RHO1, RHO2 take the new CDUs (CA CDUX; XCH RHO; MSU RHO;
 * COM).
 *
 * ARGUMENTS:
 *   deltemp  agc_dp_words_t [3]  O  DELTEMPX..Z, 180 deg
 **********************************************************************/
static void rcs_delrho(agc_dp_words_t deltemp[AGC_CSM_AXES])
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* RCS DAP erasables          */
    agc_sp_t       dx[AGC_CSM_AXES];      /* CDU - RHO, scale 180 deg (T5TEMP for Y, Z) */

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        uint16_t now = csm_dap_cdu(i); /* CA CDUX; XCH RHO            */

        dx[i]     = (agc_sp_t) -csm_dap_msu(r->rho[i], now); /* MSU RHO; COM   */
        r->rho[i] = now;
    }

    /* ZL; DXCH DELTEMPX; T5TEMP = CDUY - RHO1; MP AMGB1; DAS DELTEMPX    */
    deltemp[0].hi = dx[0];
    deltemp[0].lo = 0;
    csm_dap_words_das(&deltemp[0], csm_dap_words_mp(dx[1], r->amgb1));

    deltemp[1] = csm_dap_words_mp(r->amgb4, dx[1]); /* CA AMGB4; MP T5TEMP; DXCH */
    deltemp[2] = csm_dap_words_mp(r->amgb7, dx[1]); /* CA AMGB7; MP T5TEMP; DXCH */

    /* T5TEMP = CDUZ - RHO2; MP AMGB5; DAS DELTEMPY; AMGB8; DAS DELTEMPZ   */
    csm_dap_words_das(&deltemp[1], csm_dap_words_mp(dx[2], r->amgb5));
    csm_dap_words_das(&deltemp[2], csm_dap_words_mp(r->amgb8, dx[2]));
}

/**********************************************************************
 * KMATRIX, TENTHSEK (RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1006-1007) -- a
 * 1 s update of the gimbal-to-body matrix (job AMBGUPDT, PRIO34) every
 * tenth pass; then, unless HOLDFLAG is +0, -0 or +, the automatic
 * steering increments (DCDUINCR); then the FDAI display (DACNDLS). Ends
 * phase 1 (RESUME).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, the NOVAC status (core sets
 *          full) or a failed assertion's AGC_STATUS_BAD_ARG
 **********************************************************************/
agc_status_t csm_rcs_kmatrix(void)
{
    agc_csm_rcs_t *r  = agc_csm_dap_rcs();             /* RCS DAP erasables          */
    agc_sp_t       a  = (agc_sp_t) (r->attsec & LOW4); /* CA ATTSEC; MASK LOW4, unscaled */
    agc_status_t   st = AGC_STATUS_OK;                 /* NOVAC, DCDUINCR status */

    if (a > 0)
    {
        r->attsec = (agc_sp_t) (a - 1); /* CCS A; TCF TENTHSEK         */
    }
    else
    {
        /* CAF PRIO34; TC NOVAC; 2CADR AMBGUPDT; CAF NINE                 */
        st = agc_exec_novac(CSM_DAP_PRIO34, AGC_JOB_AMBGUPDT);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        r->attsec = NINE;
    }

    if (!AGC_ASSERT((r->attsec >= 0) && (r->attsec <= NINE)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (r->holdflag < 0)
    { /* CS HOLDFLAG; BZMF DACNDLS: automatic steering only */
        /**********************************************************************
         * DCDUINCR, DELOOP (RCS-CSM_DIGITAL_AUTOPILOT.agc pp. 1007-1008) --
         * CDUXD = CDUXD + DELCDUX (DP, modulo 360 deg via DPADD), THETADX =
         * upper word of CDUXD; axes Z, Y, X.
         *
         * ARGUMENTS: none
         *
         * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when
         *          HOLDFLAG is not negative
         **********************************************************************/
        agc_csm_rcs_t *r_dcduincr = agc_csm_dap_rcs(); /* RCS DAP erasables          */

        if (!AGC_ASSERT(r_dcduincr->holdflag < 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        for (int i = AGC_CSM_AXES - 1; i >= 0; i--)
        {
            agc_dp_t kmpac = csm_dap_dpadd(r_dcduincr->cdud[i], r_dcduincr->delcdu[i]); /* DCA CDUXD; DXCH KMPAC; DCA DELCDUX; TC DPADD, scale 180 deg */

            r_dcduincr->thetad[i] = csm_dap_sp_word(agc_dp_high(kmpac)); /* TS THETADX */
            r_dcduincr->cdud[i]   = kmpac;                               /* DXCH CDUXD */
        }

        st = AGC_STATUS_OK;
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /**********************************************************************
     * DACNDLS, FDAIDSP1, KRESUME1, FDAIDSP2 (RCS-CSM_DIGITAL_AUTOPILOT.agc
     * pp. 1009, 1022) -- alternate between driving the needles (FDAIDSP1:
     * NEEDLER) and preparing AK..AK2 (FDAIDSP2). With NEEDLFLG off (V61)
     * the needles show the autopilot following errors -ERRORX..Z; with it
     * on, the total attitude errors (FDAITOTL).
     *
     * ARGUMENTS: none
     *
     * RETURNS: agc_status_t -- the NEEDLER status, or AGC_STATUS_OK
     **********************************************************************/
    const agc_csm_rcs_t *r_dacndls = agc_csm_dap_rcs(); /* RCS DAP erasables    */
    agc_csm_dapcom_t    *c         = agc_csm_dap_com(); /* AK, RCSFLAGS         */

    if ((c->rcsflags & AGC_CSM_RCSF_FDAIALT) == 0u)
    { /* FDAIDSP1: ADS RCSFLAGS (BIT4) */
        c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_FDAIALT);

        return (csm_dap_needler()); /* TC NEEDLER; KRESUME1        */
    }

    c->rcsflags = (uint16_t) (c->rcsflags & ~(unsigned int) AGC_CSM_RCSF_FDAIALT); /* FDAIDSP2 */

    if (agc_flag_test(AGC_FLAG_NEEDLFLG))
    {
        /* total attitude errors   */
        /**********************************************************************
         * FDAITOTL, WRTN22, WRTN17, GETAKS (RCS-CSM_DIGITAL_AUTOPILOT.agc
         * pp. 1022-1023) -- total attitude error with respect to N22 (THETAD,
         * N22ORN17 set: V62) or N17 (CPHIX, V63), resolved into S/C control
         * axes as a fly-to indicator: AK = AMGB (THETA - CDU), 180 deg.
         *
         * ARGUMENTS: none
         *
         * RETURNS: agc_status_t -- AGC_STATUS_OK (end phase 1)
         **********************************************************************/
        const agc_csm_rcs_t *r_fdaitotl         = agc_csm_dap_rcs();                                     /* AMGB                     */
        agc_csm_dapcom_t    *c_fdaitotl         = agc_csm_dap_com();                                     /* AK, CPHIX                */
        bool                 use_desired_angles = agc_flag_test(AGC_FLAG_ATTITUDE_ERROR_DESIRED_ANGLES); /* N22ORN17 */
        uint16_t             ref[AGC_CSM_AXES];                                                          /* WTEMP: N22 or N17 gimbal angles, raw     */

        for (int i_fdaitotl = 0; i_fdaitotl < AGC_CSM_AXES; i_fdaitotl++)
        {
            ref[i_fdaitotl] = use_desired_angles ? agc_erase_thetad(i_fdaitotl) : c_fdaitotl->cphix[i_fdaitotl]; /* ?: CPHI (WRTN22) or CPHIX (WRTN17) */
        }

        agc_sp_t t = csm_dap_msu(ref[1], agc_erase_cduy()); /* T5TEMP = CTHETA - CDUY, scale 180 deg */

        c_fdaitotl->ak[0] = csm_dap_msu(ref[0], agc_erase_cdux()); /* MSU CDUX; TS AK */
        c_fdaitotl->ak[0] = csm_dap_ads(c_fdaitotl->ak[0], csm_dap_mp_hi(t, r_fdaitotl->amgb1));
        c_fdaitotl->ak[1] = csm_dap_mp_hi(t, r_fdaitotl->amgb4);
        c_fdaitotl->ak[2] = csm_dap_mp_hi(t, r_fdaitotl->amgb7);
        t                 = csm_dap_msu(ref[2], agc_erase_cduz()); /* CPSI - CDUZ, 180 deg */
        c_fdaitotl->ak[1] = csm_dap_ads(c_fdaitotl->ak[1], csm_dap_mp_hi(t, r_fdaitotl->amgb5));
        c_fdaitotl->ak[2] = csm_dap_ads(c_fdaitotl->ak[2], csm_dap_mp_hi(t, r_fdaitotl->amgb8));
    }
    else
    {
        for (int i_dacndls = 0; i_dacndls < AGC_CSM_AXES; i_dacndls++)
        {
            c->ak[i_dacndls] = (agc_sp_t) -r_dacndls->error[i_dacndls]; /* DCS ERRORX; DXCH AK; CS ERRORZ; TS AK2 */
        }
    }

    return (AGC_STATUS_OK); /* RESUME: end phase 1                     */
}
