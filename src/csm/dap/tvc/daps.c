/*
 * daps.c -- pitch and yaw thrust-vector-control autopilots.
 * TVCDAPS.agc (Comanche055 pp. 961-978): PITCHDAP, PSTROKER, PCDUDOTS,
 * RLIMTEST, PINTEGRL, PERORLIM, PFORWARD, POFFSET, PACLIM, POUT,
 * PPRECOMP, DELBARP, PCOPY, PMISC; YAWDAP .. YMISC, YCOPY; DAPINIT,
 * ERRORLIM, ACTLIM, FWDFLTR (1DAPCAS, 2DAPCAS, 3DAPCAS, OPTVARK), PRECOMP
 * (2CASFLTR, 3CASFLTR) and the autopilot constants.
 *
 * Self-perpetuating T5 tasks generating the SPS gimbal actuator commands
 * in response to the body-axis rate commands OMEGAYC, OMEGAZC of cross-
 * product steering (S40.8); without steering (impulsive burns) they hold
 * attitude about the initial directions. Each loop derives gimbal angle
 * rates, transforms to body axes, integrates the body-axis attitude error
 * with limiting, runs the generalised 6th-order filter (CSM: two cascades,
 * CSM/LM: three), limits the output, and runs the CG-offset tracker
 * filter and its minor loop. The DAPs call each other at TVCDT/2 (T5TVCDT:
 * 20 ms CSM, 40 ms CSM/LM). Commands go to the optics error counters
 * (TVCPITCH = CDUSCMD, TVCYAW = CDUTCMD), 1 ascrev = 85.41 arcsec/bit.
 *
 * DEVIATION (whole file): DP quantities are agc_dp_t values (sign-agreed
 * words). The AGC's DAS does not sign-correct, and where it overwrites or
 * reads only the upper word of such a pair (ERRORLIM, ACTLIM, PCMD, AK1)
 * the C value may differ from the AGC by one unit of the lower word's
 * weight carried into the upper word.
 */
#include "../internal.h"
#include "agc/guidance/guidance.h"

#define ACTSAT       ((agc_sp_t) 253) /* 00375: actuator limit 6 deg, 1 ascrev */
#define RECIP_ACTSAT ((agc_sp_t) 65)  /* 00101 DEC .0039525692 (1/253)        */
#define ERRLIM \
    ((agc_sp_t) 4096)               /* BIT13: filter input limit 45 deg,
                                           B-1 rev                            */
#define RECIP_ERRLIM ((agc_sp_t) 4) /* BIT3                                 */
#define RECIP_RTLIM \
    ((agc_sp_t) 77)                /* 00115 DEC 0.004715: 0 if CDU
                                           difference < 2.33 deg              */
#define ONE_M_EAT ((agc_sp_t) 163) /* 00243 1-E(-AT)                       */
#define E_AT \
    ((agc_sp_t) 16221)               /* 37535 E(-AT): AT = .01 (1/A = 4 s,
                                           T = 40 ms, or 1/A = 8 s, T = 80 ms) */
#define ROUND_HALF ((agc_sp_t) 8192) /* DOUBLE overflows: low word >= 1/2    */

typedef enum tvcd_axis
{
    TVCD_PITCH = 0,
    TVCD_YAW   = 1
} tvcd_axis_t;

/* RLIMTEST (p. 964): zero a CDU difference over 2.33 deg per sample.     */
static agc_sp_t tvcd_rlimtest(agc_sp_t d)
{
    if (!AGC_ASSERT(d >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    agc_sp_t q = csm_dap_mp_hi(d, RECIP_RTLIM);
    agc_sp_t r = (q != 0) ? (agc_sp_t) 0 : d;

    if (!AGC_ASSERT((r == 0) || (r == d)))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/* PCDUDOTS (p. 964): minus CDU rates for pitch and yaw, scaled at
 * 1/2 TVCDT rev/s (past - present).                                      */
static void tvcd_pcdudots(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    uint16_t       y = agc_erase_cduy();
    uint16_t       z = agc_erase_cduz();

    t->mcduydot = tvcd_rlimtest(csm_dap_msu(t->pcduypst, y));
    t->pcduypst = y;
    t->mcduzdot = tvcd_rlimtest(csm_dap_msu(t->pcduzpst, z));
    t->pcduzpst = z;

    if (!AGC_ASSERT(t->pcduypst == y))
    {
        return;
    }

    if (!AGC_ASSERT(t->pcduzpst == z))
    {
        return;
    }
}

/* ERRORLIM (p. 970): limit the upper word of ERRBTMP to +-45 deg.        */
static void tvcd_errorlim(void)
{
    agc_csm_tvc_t *t  = agc_csm_dap_tvc();
    agc_sp_t       hi = agc_dp_high(t->errbtmp);

    if (csm_dap_mp_hi(hi, RECIP_ERRLIM) != 0)
    {
        agc_sp_t lim = (hi > 0) ? ERRLIM : (agc_sp_t) -ERRLIM;

        t->errbtmp = agc_dp_pack(lim, agc_dp_low(t->errbtmp));
    }

    if (!AGC_ASSERT(agc_dp_high(t->errbtmp) <= ERRLIM))
    {
        return;
    }

    if (!AGC_ASSERT(agc_dp_high(t->errbtmp) >= -ERRLIM))
    {
        return;
    }
}

/* ACTLIM (p. 970): the command is rounded for the limit test (DOUBLE of
 * the low word overflows into the TS L skip); as in the AGC the rounded
 * word is not stored unless the limit (6 deg) applies, and then only the
 * upper word of CMDTMP is replaced.                                      */
static void tvcd_actlim(void)
{
    agc_csm_tvc_t *t   = agc_csm_dap_tvc();
    agc_sp_t       hi  = agc_dp_high(t->cmdtmp);
    agc_sp_t       lo  = agc_dp_low(t->cmdtmp);
    agc_sp_t       rnd = (lo >= ROUND_HALF) ?
                         (agc_sp_t) 1 :
                         ((lo <= -ROUND_HALF) ? (agc_sp_t) -1 : (agc_sp_t) 0);
    agc_sp_t a = csm_dap_ads(rnd, hi);

    if (csm_dap_mp_hi(a, RECIP_ACTSAT) != 0)
    {
        agc_sp_t lim = (hi > 0) ? ACTSAT : (agc_sp_t) -ACTSAT; /* CCS CMDTMP */

        t->cmdtmp = agc_dp_pack(lim, lo);
    }

    if (!AGC_ASSERT(agc_dp_high(t->cmdtmp) <= ACTSAT))
    {
        return;
    }

    if (!AGC_ASSERT(agc_dp_high(t->cmdtmp) >= -ACTSAT))
    {
        return;
    }
}

/* One cascade output (1DAPCAS .. 3DAPCAS, pp. 970-971): DAPn = N x input
 * (one-and-a-half precision) + TMP(2k-1). The DXCH leaves the DAS
 * overflow indicator (normally zero) in TMP(2k-1), which PRECOMP rewrites. */
static agc_dp_t tvcd_cascade_out(agc_dp_t in, agc_sp_t n, int k)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    if (!AGC_ASSERT((k == 0) || (k == 2) || (k == 4)))
    {
        return (0);
    }

    agc_dp_t out = csm_dap_das(csm_dap_smallmp(in, n), t->tmp[k]);

    t->tmp[k] = 0;

    if (!AGC_ASSERT(t->tmp[k] == 0))
    {
        return (0);
    }

    return (out);
}

/* FWDFLTR, OPTVARK (pp. 970-972): the filter's present output and the
 * variable gain: CMDTMP = -VARK x DAP3, rescaled B+3 -> B+1 ascrev (the
 * output DACs have an inherent gain of B+1 ascrev).                      */
static void tvcd_fwdfltr(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    t->delbrtmp = 0;
    t->dap1     = tvcd_cascade_out(t->errbtmp, t->n10[0], 0); /* N10      */
    t->dap2     = tvcd_cascade_out(t->dap1, t->n10[5], 2); /* N20      */

    if ((agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u)
    {                                                       /* LM on    */
        t->dap3 = tvcd_cascade_out(t->dap2, t->n10[10], 4); /* N30     */
    }
    else
    {
        t->dap3 = t->dap2;
    }

    agc_dp_t c = csm_dap_smallmp(agc_dp_neg(t->dap3), t->vark);

    t->cmdtmp = csm_dap_ddoubl(csm_dap_ddoubl(c));

    if (!AGC_ASSERT(t->vark >= 0))
    {
        return;
    }

    if (!AGC_ASSERT(t->delbrtmp == 0))
    {
        return;
    }
}

/* One cascade's storage for the next pass (PRECOMP, 2CASFLTR, 3CASFLTR,
 * pp. 972-976), coefficients from N10 + j (N_1/2, N_2, D_1/2, D_2):
 *   TMP(k)   = TMP(k+1) + 2 (N_1/2 in - D_1/2 out)
 *   TMP(k+1) = N_2 in - D_2 out                                          */
static void tvcd_cascade_pre(agc_dp_t in, agc_dp_t out, int j, int k)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    if (!AGC_ASSERT(((j == 1) || (j == 6) || (j == 11)) && (k >= 0) &&
                    (k <= 4)))
    {
        return;
    }

    agc_dp_t nout = agc_dp_neg(out);
    agc_dp_t a    = csm_dap_das(csm_dap_smallmp(in, t->n10[j]),
                                csm_dap_smallmp(nout, t->n10[j + 2]));

    t->tmp[k]     = csm_dap_das(t->tmp[k + 1], csm_dap_ddoubl(a));
    t->tmp[k + 1] = csm_dap_das(csm_dap_smallmp(nout, t->n10[j + 3]),
                                csm_dap_smallmp(in, t->n10[j + 1]));

    if (!AGC_ASSERT((t->tmp[k] >= AGC_DP_MIN) && (t->tmp[k] <= AGC_DP_MAX)))
    {
        return;
    }
}

/* PRECOMP (pp. 972-976): the third cascade only for CSM/LM (bit 13 off). */
static void tvcd_precomp(void)
{
    const agc_csm_tvc_t *t = agc_csm_dap_tvc();

    tvcd_cascade_pre(t->errbtmp, t->dap1, 1, 0); /* first     */
    tvcd_cascade_pre(t->dap1, t->dap2, 6, 2);    /* 2CASFLTR  */

    if ((agc_erase_dapdatr1() & CSM_DAP_BIT13) == 0u)
    {
        tvcd_cascade_pre(t->dap2, t->dap3, 11, 4); /* 3CASFLTR  */
    }

    if (!AGC_ASSERT(t->n10[0] != 0))
    {
        return;
    }

    if (!AGC_ASSERT(t->vark >= 0))
    {
        return;
    }
}

/* DELBARP / DELBARY (pp. 966, 969): offset-tracker filter
 * DELBRTMP = E(-AT) DELBAR + (1 - E(-AT)) CMDTMP.                        */
static void tvcd_delbar(agc_dp_t delbar)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_dp_t       d = csm_dap_smallmp(delbar, E_AT);

    t->delbrtmp = csm_dap_das(d, agc_sp_mul(agc_dp_high(t->cmdtmp), ONE_M_EAT));
    _Static_assert(E_AT + ONE_M_EAT == 16384, "AGC constant");

    if (!AGC_ASSERT(t->delbrtmp >= AGC_DP_MIN))
    {
        return;
    }
}

/* PCOPY (p. 966): pitch copycycle, called by the pitch DAP or (restart =
 * true, PCOPY +1) by the TVC restart package. TVCPHASE 1 .. 2.           */
agc_status_t csm_tvc_pcopy(bool restart)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    if (!restart)
    {
        t->tvcphase = 1; /* INCR TVCPHASE (1)    */
    }

    if (!AGC_ASSERT(t->tvcphase == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < 6; i++)
    {
        t->ptmp[i] = t->tmp[i];
    }

    agc_csm_dap_com()->ak[1] = agc_dp_high(t->errbtmp); /* AK1, B-1 rev  */
    t->perrb                 = t->errbtmp; /* rate-error integrator */
    t->pcmd                  = agc_dp_high(t->cmdtmp); /* actuator command     */
    t->delpbar               = t->delbrtmp;            /* offset tracker       */
    t->tvcphase              = 2;                      /* completed (2)        */

    if (!AGC_ASSERT(t->pcmd == agc_dp_high(t->cmdtmp)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* YCOPY (p. 969): yaw copycycle (YCOPY +1 from restarts). TVCPHASE 3 .. 0. */
agc_status_t csm_tvc_ycopy(bool restart)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    if (!restart)
    {
        t->tvcphase = 3; /* INCR TVCPHASE (3)    */
    }

    if (!AGC_ASSERT(t->tvcphase == 3))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < 6; i++)
    {
        t->ytmp[i] = t->tmp[i];
    }

    agc_csm_dap_com()->ak[2] = agc_dp_high(t->errbtmp); /* AK2, B-1 rev  */
    t->yerrb                 = t->errbtmp;
    t->ycmd                  = agc_dp_high(t->cmdtmp);
    t->delybar               = t->delbrtmp;
    t->tvcphase              = 0; /* reset TVCPHASE       */

    if (!AGC_ASSERT(t->ycmd == agc_dp_high(t->cmdtmp)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* The common body of PITCHDAP / YAWDAP after the T5 set-up and stroke
 * check: integrate, limit, filter, offset, limit, output, precompute,
 * track the offset, copycycle (pp. 964-969).                             */
static agc_status_t tvcd_dap(tvcd_axis_t ax)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    bool           p = (ax == TVCD_PITCH);
    int            a = p ? 1 : 2; /* OMEGAYC / OMEGAZC    */

    if (!AGC_ASSERT((ax == TVCD_PITCH) || (ax == TVCD_YAW)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* xINTEGRL: body-axis rate error integral, B-1 rev                   */
    t->errbtmp = csm_dap_das(p ? t->perrb : t->yerrb, t->omegac[a]);
    /* Body-axis pitch rate OMEGAYB (PINTEGRL, p. 964) or yaw rate OMEGAZB
     * (YINTEGRL, p. 968), scaled 1/2 TVCDT revs, from SIN/COSCDUX, COSCDUZ
     * (QUICTRIG, scale 2) and the minus CDU rates.                           */
    const agc_csm_tvc_t *t_rate = agc_csm_dap_tvc();
    const agc_cdutrig_t *g      = &agc_guid_state()->trig;
    agc_sp_t             cosz   = agc_dp_high(g->coscdu[AGC_TRIG_Z]);
    agc_sp_t             cosx   = agc_dp_high(g->coscdu[AGC_TRIG_X]);
    agc_sp_t             sinx   = agc_dp_high(g->sincdu[AGC_TRIG_X]);
    agc_sp_t             a_rate = (ax == TVCD_PITCH) ? (agc_sp_t) -cosz : cosz;
    agc_sp_t             b      = (ax == TVCD_PITCH) ? cosx : sinx;
    agc_sp_t             c      = (ax == TVCD_PITCH) ? sinx : cosx;
    agc_sp_t             k      =
        agc_dp_high(csm_dap_ddoubl(agc_sp_mul(a_rate, b)));
    agc_dp_t w =
        csm_dap_ddoubl(agc_sp_mul(k, t_rate->mcduydot));

    w =
        csm_dap_das(w,
                    csm_dap_ddoubl(agc_sp_mul((agc_sp_t) -t_rate->mcduzdot,
                                              c)));

    if (!AGC_ASSERT((w >= AGC_DP_MIN) && (w <= AGC_DP_MAX)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    t->omegab[a] = w;
    t->errbtmp   = csm_dap_das(t->errbtmp, agc_dp_neg(t->omegab[a]));
    tvcd_errorlim();

    for (int i = 0; i < 6; i += 2)
    { /* xFORWARD             */
        t->tmp[i] = p ? t->ptmp[i] : t->ytmp[i];
    }

    tvcd_fwdfltr();
    t->cmdtmp = csm_dap_das(t->cmdtmp, p ? t->pdeloff : t->ydeloff); /* B+0 */
    tvcd_actlim();

    if (!AGC_ASSERT(agc_dp_high(t->cmdtmp) >= -ACTSAT)) /* limited     */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* xOUT: incremental command to the error counter and count release */
    agc_sp_t inc = csm_dap_ads((agc_sp_t) -(p ? t->pcmd : t->ycmd),
                               agc_dp_high(t->cmdtmp));
    agc_status_t st;

    if (p)
    {
        agc_erase_set_tvcpitch(csm_dap_ads(agc_erase_tvcpitch(), inc));
        agc_chan_or(AGC_CHAN_CHAN14, (uint16_t) AGC_CH014_DRIVECDUS);
    }
    else
    {
        agc_erase_set_tvcyaw(csm_dap_ads(agc_erase_tvcyaw(), inc));
        agc_chan_or(AGC_CHAN_CHAN14, (uint16_t) AGC_CH014_DRIVECDUT);
        st = AGC_STATUS_OK;
    }

    for (int i = 1; i < 6; i += 2)
    { /* xPRECOMP             */
        t->tmp[i] = p ? t->ptmp[i] : t->ytmp[i];
    }

    tvcd_precomp();
    tvcd_delbar(p ? t->delpbar : t->delybar);
    agc_status_t s2 = p ? csm_tvc_pcopy(false) : csm_tvc_ycopy(false);

    return ((st != AGC_STATUS_OK) ? st : s2); /* xDAPEND: RESUME      */
}

/* PITCHDAP / YAWDAP entry (pp. 963, 967): T5 call of the other axis in
 * TVCDT/2, then HACK if a stroke test is starting or in progress.        */
static agc_status_t tvcd_entry(tvcd_axis_t ax)
{
    agc_csm_tvc_t *t    = agc_csm_dap_tvc();
    agc_task_id_t  next =
        (ax == TVCD_PITCH) ? AGC_TASK_YAWDAP : AGC_TASK_PITCHDAP;

    if (!AGC_ASSERT((ax == TVCD_PITCH) || (ax == TVCD_YAW)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = csm_dap_set_t5(next, (uint16_t) t->t5tvcdt);

    if ((st == AGC_STATUS_OK) && (t->stroker != 0))
    { /* (STRKFLG)    */
        st = csm_tvc_hack();
    }

    if (ax == TVCD_PITCH)
    {
        tvcd_pcdudots(); /* the yaw DAP uses the pitch rates  */
    }

    agc_status_t s2 = tvcd_dap(ax);

    return ((st != AGC_STATUS_OK) ? st : s2);
}

/* PITCHDAP (p. 963): T5 entry, normal or via DAPINIT.                    */
agc_status_t agc_task_pitchdap(void)
{
    if (!AGC_ASSERT(agc_csm_dap_tvc()->t5tvcdt > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = tvcd_entry(TVCD_PITCH);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* YAWDAP (p. 967): T5 entry.                                             */
agc_status_t agc_task_yawdap(void)
{
    if (!AGC_ASSERT(agc_csm_dap_tvc()->t5tvcdt > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = tvcd_entry(TVCD_YAW);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* DAPINIT (pp. 969-970): T5 entry from TVCINIT4. PITCHDAP in TVCDT
 * (T5TVCDT = POSMAX - TVCDT/2 + 1, so -1 + 2 T5TVCDT + NEGMAX), and the
 * past CDUs for the differentiators.                                     */
agc_status_t agc_task_dapinit(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    int            w = -1 + (int) t->t5tvcdt + (int) AGC_SP_MIN +
                       (int) t->t5tvcdt;

    if (!AGC_ASSERT((w > 0) && (w <= AGC_SP_MAX)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = csm_dap_set_t5(AGC_TASK_PITCHDAP, (uint16_t) w);

    t->pcduypst = agc_erase_cduy();
    t->pcduzpst = agc_erase_cduz();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st); /* NOQRSM               */
}
