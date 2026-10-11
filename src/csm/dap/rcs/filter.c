/*
 * filter.c -- phase 1 of the RCS-CSM autopilot T5 program.
 * RCS-CSM_DIGITAL_AUTOPILOT.agc (Comanche055 pp. 1004-1009, 1021-1023):
 * RATEFILT (DRHOLOOP, ADOTLOOP), KMATRIX, TENTHSEK, DCDUINCR (DELOOP),
 * DACNDLS, FDAIDSP1, FDAIDSP2, FDAITOTL, WRTN22, WRTN17, GETAKS.
 *
 * Rate filter equations (p. 1004):
 *     DRHO = DELRHO - (.1)ADOT     + (1 - GAIN1) DRHO
 *                             -1                     -1
 *     ADOT = ADOT    + GAIN2 DRHO + KMJ DFT
 *                -1
 *     where DELRHO = AMGB (CDU - CDU  )
 *                                   -1
 * DRHO scaled 180 deg, ADOT 450 deg/s, CDU differences one bit = 180/2^14
 * deg, AMGB elements scaled 1, KMJ (450)(1600)/(57.3)(16384) = 1/1.3,
 * DFT in 1/1600 s.
 */
#include "../internal.h"

#define QUARTER   ((agc_sp_t) 8192) /* QUARTER = BIT13                      */
#define LOW4      ((agc_sp_t) 017)  /* LOW4                                 */
#define NINE      ((agc_sp_t) 9)    /* ATTSEC: 1 s matrix update cycle      */
#define GAIN_BIAS 3                 /* GAIN1 -3 .. GAIN1 +10                */
#define GAIN_N    14

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

static agc_status_t rcs_delrho(agc_dp_t deltemp[AGC_CSM_AXES]);
static agc_status_t rcs_dcduincr(void);
static agc_status_t rcs_dacndls(void);
static agc_status_t rcs_fdaitotl(void);

/* RATEFILT .. ADOTLOOP (pp. 1004-1006).                                  */
agc_status_t csm_rcs_ratefilt(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    int            g = (int) r->attkalmn + GAIN_BIAS;

    if (!AGC_ASSERT((g >= 0) && (g < GAIN_N)))
    {
        g = GAIN_BIAS; /* ATTKALMN out of table: use +0  */
    }

    for (int i = AGC_CSM_AXES - 1; i >= 0; i--)
    { /* DRHOLOOP   */
        agc_sp_t m = (agc_sp_t) -agc_dp_high(r->drho[i]); /* CS DRHO    */

        r->drho[i]     = csm_dap_das(r->drho[i], agc_sp_mul(m, gain1[g]));
        agc_dp_t kmpac = csm_dap_smallmp(-r->adot[i], QUARTER); /* -(.1)ADOT */

        r->drho[i] = csm_dap_das(r->drho[i], kmpac);
    }

    agc_dp_t     deltemp[AGC_CSM_AXES];
    agc_status_t st = rcs_delrho(deltemp);

    for (int i = AGC_CSM_AXES - 1; i >= 0; i--)
    { /* ADOTLOOP   */
        r->drho[i]   = csm_dap_das(r->drho[i], deltemp[i]);
        r->merror[i] = csm_dap_das(r->merror[i], deltemp[i]);
        agc_sp_t d   = agc_dp_high(r->drho[i]);

        d          = csm_dap_ads(d, d); /* DOUBLE N.B. */
        d          = csm_dap_ads(d, d); /* DOUBLE N.B. */
        r->adot[i] = csm_dap_das(r->adot[i], agc_sp_mul(d, gain2[g]));
        r->adot[i] = csm_dap_das(r->adot[i], agc_sp_mul(r->kmj[i], r->dft[i]));
    }

    if (!AGC_ASSERT(g < GAIN_N))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* Measured body rates (p. 1005): DELRHO = AMGB (CDU - RHO), kept double
 * precision "or will lose pulses". RHO, RHO1, RHO2 take the new CDUs.    */
static agc_status_t rcs_delrho(agc_dp_t deltemp[AGC_CSM_AXES])
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();

    if (!AGC_ASSERT(deltemp != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_sp_t dx[AGC_CSM_AXES];

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        uint16_t now = csm_dap_cdu(i); /* CA CDUX; XCH RHO        */

        dx[i]     = (agc_sp_t) -csm_dap_msu(r->rho[i], now); /* MSU RHO; COM   */
        r->rho[i] = now;
    }

    deltemp[0] = csm_dap_das(agc_dp_from_sp(dx[0]), agc_sp_mul(dx[1], r->amgb1));
    deltemp[1] = agc_sp_mul(r->amgb4, dx[1]);
    deltemp[2] = agc_sp_mul(r->amgb7, dx[1]);
    deltemp[1] = csm_dap_das(deltemp[1], agc_sp_mul(dx[2], r->amgb5));
    deltemp[2] = csm_dap_das(deltemp[2], agc_sp_mul(r->amgb8, dx[2]));

    if (!AGC_ASSERT(r->rho[2] <= CSM_DAP_NEG0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* KMATRIX, TENTHSEK (pp. 1006-1007): a 1 s update of the gimbal-to-body
 * matrix (job AMBGUPDT) every tenth pass; then the automatic steering
 * increments and the FDAI display. Ends phase 1 (RESUME).                */
agc_status_t csm_rcs_kmatrix(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_sp_t       a = (agc_sp_t) (r->attsec & LOW4);

    if (a > 0)
    {
        r->attsec = (agc_sp_t) (a - 1);
    }
    else
    {
        /* NOVAC returns after scheduling; a 1202 is raised by the exec.  */
        agc_status_t sw = agc_exec_novac(CSM_DAP_PRIO34, AGC_JOB_AMBGUPDT);

        if (sw != AGC_STATUS_OK)
        {
            return (sw);
        }

        r->attsec = NINE;
    }

    if (!AGC_ASSERT((r->attsec >= 0) && (r->attsec <= NINE)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_OK;

    if (r->holdflag < 0)
    { /* HOLDFLAG +0,-0,+: bypass auto cmds */
        st = rcs_dcduincr();
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rcs_dacndls());
}

/* DCDUINCR, DELOOP (pp. 1007-1008): CDUXD = CDUXD + DELCDUX (DP, modulo
 * 360 deg via DPADD); THETADX = upper word of CDUXD.                     */
static agc_status_t rcs_dcduincr(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();

    if (!AGC_ASSERT(r->holdflag < 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = AGC_CSM_AXES - 1; i >= 0; i--)
    {
        agc_dp_t kmpac = csm_dap_dpadd(r->cdud[i], r->delcdu[i]);

        r->thetad[i] = csm_dap_sp_word(agc_dp_high(kmpac));
        r->cdud[i]   = kmpac;
    }

    if (!AGC_ASSERT(r->thetad[0] <= CSM_DAP_NEG0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* DACNDLS, FDAIDSP1, FDAIDSP2 (pp. 1009, 1022): alternate between
 * driving the needles (FDAIDSP1) and preparing AK..AK2 (FDAIDSP2).
 * Mode 1 (V61, NEEDLFLG off): autopilot following errors -ERRORX..Z.    */
static agc_status_t rcs_dacndls(void)
{
    const agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_csm_dapcom_t    *c = agc_csm_dap_com();

    if ((c->rcsflags & AGC_CSM_RCSF_FDAIALT) == 0u)
    { /* FDAIDSP1    */
        c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_FDAIALT);

        return (csm_dap_needler()); /* KRESUME1: end phase 1  */
    }

    c->rcsflags = (uint16_t) (c->rcsflags & ~(unsigned) AGC_CSM_RCSF_FDAIALT);

    if (agc_flag_test(AGC_FLAG_NEEDLFLG))
    {
        return (rcs_fdaitotl()); /* total attitude errors  */
    }

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        c->ak[i] = (agc_sp_t) -r->error[i]; /* DCS ERRORX; DXCH AK    */
    }

    if (!AGC_ASSERT((c->rcsflags & AGC_CSM_RCSF_FDAIALT) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* end phase 1            */
}

/* FDAITOTL, WRTN22, WRTN17, GETAKS (pp. 1022-1023): total attitude error
 * with respect to N22 (THETAD: V62) or N17 (CPHIX: V63), resolved into
 * S/C control axes as a fly-to indicator: AK = AMGB (THETA - CDU).       */
static agc_status_t rcs_fdaitotl(void)
{
    const agc_csm_rcs_t *r                  = agc_csm_dap_rcs();
    agc_csm_dapcom_t    *c                  = agc_csm_dap_com();
    uint16_t             ref[AGC_CSM_AXES];
    bool                 use_desired_angles = agc_flag_test(AGC_FLAG_ATTITUDE_ERROR_DESIRED_ANGLES);

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        ref[i] = use_desired_angles ? agc_erase_thetad(i) : c->cphix[i]; /* CPHI.. / CPHIX.. */
    }

    c->ak[0]   = csm_dap_msu(ref[0], agc_erase_cdux());
    agc_sp_t t = csm_dap_msu(ref[1], agc_erase_cduy());

    c->ak[0] = csm_dap_ads(c->ak[0], csm_dap_mp_hi(t, r->amgb1));
    c->ak[1] = csm_dap_mp_hi(t, r->amgb4);
    c->ak[2] = csm_dap_mp_hi(t, r->amgb7);
    t        = csm_dap_msu(ref[2], agc_erase_cduz());
    c->ak[1] = csm_dap_ads(c->ak[1], csm_dap_mp_hi(t, r->amgb5));
    c->ak[2] = csm_dap_ads(c->ak[2], csm_dap_mp_hi(t, r->amgb8));

    if (!AGC_ASSERT(c->ak[2] >= AGC_SP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* end phase 1 of RCS DAP */
}
