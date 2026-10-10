/*
 * exec.c -- the TVC executive: a self-perpetuating waitlist task at
 * 1/2 second intervals.
 * TVCEXECUTIVE.agc (Comanche055 pp. 945-950): TVCEXEC, ROLLPREP, NEEDLEUP,
 * VARGAINS, GAINCHNG, EXECCOPY, 1SHOTCHK, REPCHEK, 1SHOTOK, TEMPSET,
 * CORSETUP, CG.CORR, CORCOPY, CNTRCOPY, TVCEXFIN.
 *
 * Prepares the roll DAP (OGANOW, the fly-to roll needle AK and the
 * fly-from phase-plane error OGAERR) and its waitlist call (3 cs delay),
 * updates the needles, updates vehicle mass and the DAP gains through
 * MASSPROP every 10 s, performs the one-shot trim correction for the
 * TMC loop 0-3 s after ignition and the repetitive updates after it.
 * The copycycles are restart-protected by the phase points TVCEXPHS 1..6
 * (INCR TVCEXPHS); EXRSTRT (restarts.c) re-enters at the phase point.
 */
#include "../internal.h"

#define HALF_SEC_CS ((uint16_t) 50)    /* .5SEC                          */
#define ROLL_DELAY  ((uint16_t) 3)     /* THREE cs: ROLLDAP task delay   */
#define NINETEEN    ((agc_sp_t) 19)    /* VD1: 20 passes = 10 s          */
#define FCORFRAC    ((agc_sp_t) 4096)  /* 10000 OCT: one-shot fraction .25 */
#define NEG_BIT14   ((agc_sp_t) -8192) /* -BIT14                         */

/* Restart points of the TVC executive (the order of TVCEXADR, p. 960).  */
typedef enum tvcx_step
{
    TVCX_EXECCOPY1 = 1, /* EXECCOPY +1   (TVCEXPHS 1)             */
    TVCX_1SHOTCHK,      /* 1SHOTCHK      (TVCEXPHS 2)             */
    TVCX_TEMPSET,       /* TEMPSET       (TVCEXPHS 3)             */
    TVCX_CORSETUP,      /* CORSETUP      (TVCEXPHS 4)             */
    TVCX_CORCOPY1,      /* CORCOPY +1    (TVCEXPHS 5)             */
    TVCX_CNTRCOPY,      /* CNTRCOPY      (TVCEXPHS 6)             */
    TVCX_REPCHEK,
    TVCX_1SHOTOK,
    TVCX_FIN /* TVCEXFIN                               */
} tvcx_step_t;

#define TVCX_MAX_STEPS 10 /* longest path: 1 .. 6 plus branches      */

/* ROLLPREP (p. 947): roll ladders, roll FDAI needle (fly-to OGA error,
 * command - measured, B-1 rev) and the phase-plane OGAERR (fly-from, B+0
 * rev); the ROLLDAP task in 3 cs. Roll DAPs restart themselves.          */
static agc_status_t tvcx_rollprep(void)
{
    agc_csm_tvc_t    *t = agc_csm_dap_tvc();
    agc_csm_dapcom_t *c = agc_csm_dap_com();

    t->ogapast = t->oganow;
    t->oganow  = agc_erase_cdux();
    c->ak[0]   = csm_dap_msu(c->ogad, t->oganow); /* AK, fly-to error     */
    t->ogaerr  = csm_dap_mp_hi(c->ak[0], NEG_BIT14);

    if (!AGC_ASSERT(t->oganow <= CSM_DAP_NEG0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_waitlist_call(ROLL_DELAY, AGC_TASK_ROLLDAP));
}

/* VARGAINS, GAINCHNG (pp. 947-948): mass and gain updates while the
 * engine is on; returns the next step (EXECCOPY +1 or 1SHOTCHK).        */
static tvcx_step_t tvcx_vargains(agc_status_t *st)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    /* Engine off, or VCNTR = -0 (V97 R40 ENGFAIL / LOTHRUST): bypass the
     * mass/gain updates. Any negative VCNTR stands for -0 here.           */
    if (!csm_dap_engine_on() || (t->vcntr < 0))
    {
        t->tvcexphs = 2;

        return (TVCX_1SHOTCHK);
    }

    if (t->vcntr > 0)
    {                                            /* not yet              */
        t->vcntrtmp = (agc_sp_t) (t->vcntr - 1); /* protect VCNTR and    */
        t->masstmp  = agc_erase_csmmass();       /* CSMMASS (impulsive)  */
    }
    else
    {                                 /* GAINCHNG             */
        agc_csm_dap_fixcw();          /* IXX, IAVG, IAVG/TLX  */
        *st = csm_tvc_gains(true);    /* 1/CONACC, VARK       */
        /* update mass for the next 10 s of burn, kg B+16                  */
        t->masstmp  = csm_dap_ads(agc_erase_csmmass(), (agc_sp_t) -t->tenmdot);
        t->vcntrtmp = NINETEEN; /* reset the counter    */
    }

    t->tvcexphs = 1; /* EXECCOPY: INCR (1)   */

    if (!AGC_ASSERT(t->vcntrtmp >= 0))
    {
        return (TVCX_FIN);
    }

    return (TVCX_EXECCOPY1);
}

/* EXECCOPY +1 (p. 948): the copycycle of mass and gain counter.         */
static tvcx_step_t tvcx_execcopy(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    if (!AGC_ASSERT(t->tvcexphs == 1))
    {
        return (TVCX_FIN);
    }

    agc_erase_set_csmmass(t->masstmp);
    t->vcntr             = t->vcntrtmp;
    t->engine_fail_vcntr =
        t->vcntrtmp; /* for ENGFAIL (R41) updates at SPSOFF  */
    t->tvcexphs = 2; /* copycycle over (2)                   */

    if (!AGC_ASSERT(t->vcntr >= 0))
    {
        return (TVCX_FIN);
    }

    return (TVCX_1SHOTCHK);
}

/* 1SHOTCHK (p. 948): time for the one-shot or the repetitive correction. */
static tvcx_step_t tvcx_1shotchk(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    if (!AGC_ASSERT(t->cntr >= AGC_SP_MIN))
    {
        return (TVCX_FIN);
    }

    if (t->cntr > 0)
    { /* not yet: count down  */
        t->cntrtmp  = (agc_sp_t) (t->cntr - 1);
        t->tvcexphs = 6; /* entry at CNTRCOPY    */

        return (TVCX_CNTRCOPY);
    }

    if (t->cntr < 0)
    {
        return (TVCX_REPCHEK); /* one-shot over        */
    }

    return (TVCX_1SHOTOK); /* now                  */
}

/* REPCHEK (p. 948): repetitive updates if REPFRAC is positive.           */
static tvcx_step_t tvcx_repchek(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();

    if (t->repfrac <= 0)
    {
        return (TVCX_FIN); /* over and out         */
    }

    t->tempdap[1] = t->repfrac; /* correction fraction  */
    t->tvcexphs   = 4;          /* entry at CORSETUP    */

    if (!AGC_ASSERT(t->tempdap[1] > 0))
    {
        return (TVCX_FIN);
    }

    return (TVCX_CORSETUP);
}

/* CORSETUP (p. 949): the correction fraction TEMPDAP is doubled for the
 * CSM alone (DAPDATR1 bit 13 shows LM off). CNTR = -1: one-shot done.    */
static void tvcx_corsetup(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_sp_t       f = t->tempdap[1];

    if ((agc_erase_dapdatr1() & CSM_DAP_BIT13) != 0u)
    {
        f = csm_dap_ads(f, t->tempdap[1]); /* LM off: 2(TEMPDAP+1) */
    }

    t->tempdap[0] = f;
    t->cntrtmp    = -1;

    if (!AGC_ASSERT(t->cntrtmp < 0))
    {
        return;
    }
}

/* CG.CORR, one axis (p. 949): ACTTMP = DELOFF + TEMPDAP x hi(4 (DELBAR -
 * DELOFF)), the TMC loop correction of the trim estimate.                */
static agc_dp_t tvcx_cgcorr(agc_dp_t deloff, agc_dp_t delbar, agc_sp_t frac)
{
    agc_dp_t d = csm_dap_ddoubl(csm_dap_ddoubl(agc_dp_neg(deloff)));

    d = csm_dap_das(d, csm_dap_ddoubl(csm_dap_ddoubl(delbar))); /* TTMP1 */

    if (!AGC_ASSERT(frac >= 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(d >= AGC_DP_MIN))
    {
        return (0); /* no valid result */
    }

    return (csm_dap_das(deloff, agc_sp_mul(agc_dp_high(d), frac)));
}

/* CORCOPY +1 (p. 950): trim estimates and trims, restart-protected.     */
static void tvcx_corcopy(void)
{
    agc_csm_tvc_t    *t = agc_csm_dap_tvc();
    agc_csm_dapcom_t *c = agc_csm_dap_com();

    if (!AGC_ASSERT(t->tvcexphs == 5))
    {
        return;
    }

    c->pactoff  = agc_dp_high(t->pacttmp);
    t->pdeloff  = t->pacttmp;
    c->yactoff  = agc_dp_high(t->yacttmp);
    t->ydeloff  = t->yacttmp;
    t->tvcexphs = 6; /* (6)                  */

    if (!AGC_ASSERT(c->pactoff >= AGC_SP_MIN))
    {
        return;
    }
}

/* One step of the TVCEXEC .. TVCEXFIN sequence after VARGAINS.          */
static tvcx_step_t tvcx_step(tvcx_step_t s)
{
    agc_csm_tvc_t *t    = agc_csm_dap_tvc();
    tvcx_step_t    next = TVCX_FIN;

    switch (s)
    {
        case TVCX_EXECCOPY1:
            next = tvcx_execcopy();
            break;
        case TVCX_1SHOTCHK:
            next = tvcx_1shotchk();
            break;
        case TVCX_REPCHEK:
            next = tvcx_repchek();
            break;
        case TVCX_1SHOTOK: /* not permitted during engine shutdown          */

            if (csm_dap_engine_on())
            {
                t->tvcexphs = 3;
                next        = TVCX_TEMPSET;
            }

            break;
        case TVCX_TEMPSET: /* correction fraction  */
            t->tempdap[1] = FCORFRAC;
            t->tvcexphs   = 4;
            next          = TVCX_CORSETUP;
            break;
        case TVCX_CORSETUP: /* .. CG.CORR           */
            tvcx_corsetup();
            t->pacttmp  = tvcx_cgcorr(t->pdeloff, t->delpbar, t->tempdap[0]);
            t->yacttmp  = tvcx_cgcorr(t->ydeloff, t->delybar, t->tempdap[0]);
            t->tvcexphs = 5; /* CORCOPY: INCR (5)    */
            next        = TVCX_CORCOPY1;
            break;
        case TVCX_CORCOPY1:
            tvcx_corcopy();
            next = TVCX_CNTRCOPY;
            break;
        case TVCX_CNTRCOPY: /* restarts OK here     */
            t->cntr = t->cntrtmp;
            break;
        case TVCX_FIN:
        default:
            break;
    }

    if (!AGC_ASSERT((next >= TVCX_EXECCOPY1) && (next <= TVCX_FIN)))
    {
        return (TVCX_FIN);
    }

    return (next);
}

/* Run the sequence from a restart point to TVCEXFIN (reset TVCEXPHS).    */
static agc_status_t tvcx_run(tvcx_step_t from)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    tvcx_step_t    s = from;

    for (int i = 0; (i < TVCX_MAX_STEPS) && (s != TVCX_FIN); i++)
    {
        s = tvcx_step(s);
    }

    if (!AGC_ASSERT(s == TVCX_FIN))
    {
        return (AGC_STATUS_NO_CONVERGE);
    }

    t->tvcexphs = 0;      /* TVCEXFIN             */

    return (AGC_STATUS_OK); /* TASKOVER             */
}

/* EXRSTRT targets (TVCRESTARTS p. 960): INDEX TVCEXPHS into TVCEXADR.    */
agc_status_t csm_tvc_exec_restart(agc_sp_t tvcexphs)
{
    if (!AGC_ASSERT((tvcexphs >= 1) && (tvcexphs <= 6)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = tvcx_run((tvcx_step_t) tvcexphs);

    if (!AGC_ASSERT(agc_csm_dap_tvc()->tvcexphs == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* TVCEXEC (p. 946): waitlist task every 1/2 s, called by TVCINIT4 and by
 * itself; terminates when FLAGWRD6 bits 15,14 no longer read 10.         */
agc_status_t agc_task_tvcexec(void)
{
    if (!csm_tvc_dap_is_tvc())
    {
        return (tvcx_run(TVCX_FIN)); /* TVCEXFIN: terminate  */
    }

    agc_status_t st   = agc_waitlist_call(HALF_SEC_CS, AGC_TASK_TVCEXEC);
    agc_status_t s2   = tvcx_rollprep();
    agc_status_t s3   = csm_dap_needler(); /* NEEDLEUP             */
    agc_status_t s4   = AGC_STATUS_OK;
    tvcx_step_t  from = tvcx_vargains(&s4);
    agc_status_t s5   = tvcx_run(from);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if ((s2 != AGC_STATUS_OK) || (s3 != AGC_STATUS_OK))
    {
        return ((s2 != AGC_STATUS_OK) ? s2 : s3);
    }

    return ((s4 != AGC_STATUS_OK) ? s4 : s5);
}
