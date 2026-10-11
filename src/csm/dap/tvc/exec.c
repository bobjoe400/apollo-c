/**********************************************************************
 * FILE NAME: exec.c
 *
 * PURPOSE:
 *   The TVC executive, a self-perpetuating waitlist task every 1/2 s:
 *   prepares the roll DAP (OGANOW, the fly-to roll needle AK, the
 *   fly-from phase-plane error OGAERR) and calls it in 3 cs, updates the
 *   needles, updates vehicle mass and the DAP gains every 10 s, and makes
 *   the one-shot trim correction (TMC loop) and the repetitive updates.
 *   Labels: TVCEXEC, ROLLPREP, NEEDLEUP, VARGAINS, GAINCHNG, EXECCOPY,
 *   1SHOTCHK, REPCHEK, 1SHOTOK, TEMPSET, CORSETUP, CG.CORR, CORCOPY,
 *   CNTRCOPY, TVCEXFIN.
 *
 * AGC SOURCE:
 *   Comanche055 TVCEXECUTIVE.agc pp. 945-950 (CSM only)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   FLAGWRD6      flags     I    bits           DAPBIT1/2 (T5 owner)
 *   CDUX          uint16_t  I    2^15 = 1 rev   outer gimbal (raw)
 *   CSMMASS       agc_sp_t  I/O  2^16 kg        vehicle mass
 *   DAPDATR1      uint16_t  I    bits           bit 13: LM off
 *   CHAN11        uint16_t  I    bits           bit 13 SPS engine on
 *   TVC block     struct    I/O  per field      agc_csm_dap_tvc()
 *   AK, PACTOFF,  agc_sp_t  O    B-1 rev,       needle, trims
 *   YACTOFF                      85.41"/bit     (agc_csm_dap_com())
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_waitlist_call     WAITLIST (TVCEXEC, ROLLDAP)
 *   csm_dap_needler       NEEDLER (needles.c)
 *   agc_csm_dap_fixcw     FIXCW, mass properties (massprop.c)
 *   csm_tvc_gains         S40.15, 1/CONACC and VARK (init.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG / AGC_STATUS_NO_CONVERGE when an
 *   assertion on TVCEXPHS or the step sequence fails.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Waitlist task. The copycycles are restart-protected by the phase
 *   points TVCEXPHS 1..6 (INCR TVCEXPHS); EXRSTRT (restarts.c) re-enters
 *   at the phase point through csm_tvc_exec_restart. CSM only.
 *
 * NOTES:
 *   The sequence from EXECCOPY +1 to TVCEXFIN is a bounded step machine
 *   (tvcx_run) so that a restart can enter it at any phase point.
 *   VCNTR: the AGC reads -0 (V97 R40 ENGFAIL, LOTHRUST) as "bypass"; any
 *   negative VCNTR stands for that -0 here (a negative VCNTR is "not
 *   used" in the AGC).
 *
 * ALGORITHM:
 *   1. Terminate (TVCEXFIN) unless FLAGWRD6 bits 15,14 read 10.
 *   2. Call itself in 1/2 s; ROLLPREP; NEEDLER.
 *   3. Engine on: every 20th pass (10 s) update IXX.., the gains and the
 *      mass for the next 10 s; copycycle.
 *   4. CNTR > 0: count down. CNTR = 0: one-shot trim correction with
 *      fraction .25 (engine on only). CNTR < 0: repetitive correction
 *      with REPFRAC if positive. The CSM alone doubles the fraction.
 **********************************************************************/
#include "../internal.h"

#define HALF_SEC_CS    ((uint16_t) 50)    /* .5SEC DEC 50: TVCEXEC period, cs     */
#define ROLL_DELAY     ((uint16_t) 3)     /* THREE: ROLLDAP task delay, cs        */
#define NINETEEN       ((agc_sp_t) 19)    /* VD1: 20 passes = 10 s, unscaled     */
#define FCORFRAC       ((agc_sp_t) 4096)  /* OCT 10000: one-shot fraction .25   */
#define NEG_BIT14      ((agc_sp_t) -8192) /* -BIT14: B-1 -> B+0 rev, sign change */
#define TVCX_MAX_STEPS 10                 /* longest path: 1 .. 6 plus branches */

/* Restart points of the TVC executive, in the order of TVCEXADR +1 ..
 * +6 (TVCRESTARTS p. 960), then the branch targets that are not
 * restart points.                                                       */
typedef enum tvcx_step
{
    TVCX_EXEC_COPY = 1,   /* EXECCOPY +1 (TVCEXPHS 1)              */
    TVCX_ONE_SHOT_CHECK,  /* 1SHOTCHK    (TVCEXPHS 2)              */
    TVCX_TEMPSET,         /* TEMPSET     (TVCEXPHS 3)              */
    TVCX_CORSETUP,        /* CORSETUP    (TVCEXPHS 4)              */
    TVCX_CORRECTION_COPY, /* CORCOPY +1  (TVCEXPHS 5)              */
    TVCX_CNTRCOPY,        /* CNTRCOPY    (TVCEXPHS 6)              */
    TVCX_REPCHEK,         /* REPCHEK                               */
    TVCX_ONE_SHOT_OK,     /* 1SHOTOK                               */
    TVCX_FIN              /* TVCEXFIN                              */
} tvcx_step_t;

/**********************************************************************
 * ROLLPREP (TVCEXECUTIVE.agc p. 946) -- roll ladders (no restart
 * protection needed, the roll DAP restarts itself), the roll FDAI
 * needle AK (fly-to OGA error, command - measured, B-1 rev), the
 * phase-plane OGAERR (fly-from, B+0 rev) and the ROLLDAP task in 3 cs.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the WAITLIST status
 **********************************************************************/
static agc_status_t tvcx_rollprep(void)
{
    agc_csm_tvc_t    *t = agc_csm_dap_tvc(); /* TVC DAP erasables       */
    agc_csm_dapcom_t *c = agc_csm_dap_com(); /* AK, OGAD                */

    t->ogapast = t->oganow;                          /* XCH OGANOW; XCH OGAPAST       */
    t->oganow  = agc_erase_cdux();
    c->ak[0]   = csm_dap_msu(c->ogad, t->oganow);    /* MSU: AK, B-1 rev */
    t->ogaerr  = csm_dap_mp_hi(c->ak[0], NEG_BIT14); /* B+0 rev       */

    if (!AGC_ASSERT(t->oganow <= CSM_DAP_NEG0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_waitlist_call(ROLL_DELAY, AGC_TASK_ROLLDAP));
}

/**********************************************************************
 * VARGAINS, GAINCHNG (TVCEXECUTIVE.agc pp. 946-947) -- mass and gain
 * updates while the engine is on, prepared for the copycycle.
 *
 * ARGUMENTS:
 *   st  agc_status_t *  O  status of S40.15 (left unchanged when the
 *                          gains are not updated)
 *
 * RETURNS: tvcx_step_t -- EXECCOPY +1, or 1SHOTCHK when the engine is
 *          off or VCNTR is -0 (TVCEXPHS = 2)
 **********************************************************************/
static tvcx_step_t tvcx_vargains(agc_status_t *st)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    if (!AGC_ASSERT(st != NULL))
    {
        return (TVCX_FIN);
    }

    if (!csm_dap_engine_on() || (t->vcntr < 0))
    {
        t->tvcexphs = 2; /* VARGAINS +5: CAF TWO; TS TVCEXPHS         */

        return (TVCX_ONE_SHOT_CHECK);
    }

    if (t->vcntr > 0)
    { /* not yet             */
        t->vcntrtmp = (agc_sp_t) (t->vcntr - 1); /* protect VCNTR and   */
        t->masstmp  = agc_erase_csmmass();       /* CSMMASS (impulsive burn)  */
    }
    else
    { /* GAINCHNG                          */
        agc_csm_dap_fixcw();                                                    /* IXX, IAVG, IAVG/TLX               */
        *st         = csm_tvc_gains(true);                                      /* 1/CONACC, VARK             */
        t->masstmp  = csm_dap_ads(agc_erase_csmmass(), (agc_sp_t) -t->tenmdot); /* 2^16 kg    */
        t->vcntrtmp = NINETEEN;                                                 /* reset the variable-gain counter      */
    }

    t->tvcexphs = (agc_sp_t) (t->tvcexphs + 1); /* EXECCOPY: INCR (1)  */

    if (!AGC_ASSERT(t->vcntrtmp >= 0))
    {
        return (TVCX_FIN);
    }

    return (TVCX_EXEC_COPY);
}

/**********************************************************************
 * EXECCOPY +1 (TVCEXECUTIVE.agc p. 947) -- the copycycle of mass and
 * gain-update counter (also V97VCNTR for the ENGFAIL mass update).
 *
 * ARGUMENTS: none
 *
 * RETURNS: tvcx_step_t -- 1SHOTCHK (TVCEXPHS = 2)
 **********************************************************************/
static tvcx_step_t tvcx_exec_copy(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    if (!AGC_ASSERT(t->tvcexphs == 1))
    {
        return (TVCX_FIN);
    }

    agc_erase_set_csmmass(t->masstmp);  /* CSMMASS, 2^16 kg              */
    t->vcntr             = t->vcntrtmp;
    t->engine_fail_vcntr = t->vcntrtmp; /* V97VCNTR (R41 at SPSOFF)    */
    t->tvcexphs          = 2;           /* INCR: copycycle over (2)    */

    if (!AGC_ASSERT(t->vcntr >= 0))
    {
        return (TVCX_FIN);
    }

    return (TVCX_ONE_SHOT_CHECK);
}

/**********************************************************************
 * 1SHOTCHK (TVCEXECUTIVE.agc p. 947) -- time for the one-shot or the
 * repetitive correction (CCS CNTR).
 *
 * ARGUMENTS: none
 *
 * RETURNS: tvcx_step_t -- CNTRCOPY (count down, TVCEXPHS = 6), 1SHOTOK
 *          (CNTR = 0) or REPCHEK (CNTR < 0, one-shot over)
 **********************************************************************/
static tvcx_step_t tvcx_one_shot_check(void)
{
    agc_csm_tvc_t *t    = agc_csm_dap_tvc(); /* TVC DAP erasables       */
    tvcx_step_t    next = TVCX_ONE_SHOT_OK;  /* CNTR = +0: now          */

    if (!AGC_ASSERT(t->cntr >= AGC_SP_MIN))
    {
        return (TVCX_FIN);
    }

    if (t->cntr > 0)
    { /* not yet: count down  */
        t->cntrtmp  = (agc_sp_t) (t->cntr - 1);
        t->tvcexphs = 6; /* entry at CNTRCOPY    */
        next        = TVCX_CNTRCOPY;
    }
    else if (t->cntr < 0)
    {
        next = TVCX_REPCHEK; /* one-shot over, on to REPCORR            */
    }
    else
    {
        /* +0: 1SHOTOK */
    }

    return (next);
}

/**********************************************************************
 * REPCHEK (TVCEXECUTIVE.agc p. 947) -- repetitive updates when REPFRAC
 * is positive: correction fraction TEMPDAP +1 = REPFRAC.
 *
 * ARGUMENTS: none
 *
 * RETURNS: tvcx_step_t -- CORSETUP (TVCEXPHS = 4) or TVCEXFIN
 **********************************************************************/
static tvcx_step_t tvcx_repchek(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    if (t->repfrac <= 0)
    {
        return (TVCX_FIN); /* BZMF: no, over and out                    */
    }

    t->tempdap[1] = t->repfrac; /* correction fraction, 2^2           */
    t->tvcexphs   = 4;          /* entry at CORSETUP                  */

    if (!AGC_ASSERT(t->tempdap[1] > 0))
    {
        return (TVCX_FIN);
    }

    return (TVCX_CORSETUP);
}

/**********************************************************************
 * CORSETUP (TVCEXECUTIVE.agc p. 948) -- the correction fraction used
 * by CG.CORR: TEMPDAP +1, doubled for the CSM alone (DAPDATR1 bit 13
 * shows LM off); CNTRTMP = -1 (one-shot done).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void tvcx_corsetup(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */
    agc_sp_t       f = t->tempdap[1];     /* TEMPDAP, fraction 2^2      */

    if ((agc_erase_dapdatr1() & CSM_DAP_BIT13) != 0u)
    {
        f = csm_dap_ads(f, t->tempdap[1]); /* LM off: 2 (TEMPDAP +1)    */
    }

    t->tempdap[0] = f;
    t->cntrtmp    = -1;

    if (!AGC_ASSERT(t->cntrtmp < 0))
    {
        return;
    }
}

/**********************************************************************
 * CG.CORR, one axis (TVCEXECUTIVE.agc p. 948) -- the TMC loop correction
 * of the trim estimate: ACTTMP = DELOFF + TEMPDAP x (upper word of
 * 4 (DELBAR - DELOFF)). TTMP1 is formed by DAS without sign agreement.
 *
 * ARGUMENTS:
 *   deloff  agc_dp_words_t  I  PDELOFF or YDELOFF, 1 ascrev
 *   delbar  agc_dp_words_t  I  DELPBAR or DELYBAR, 1 ascrev
 *   frac    agc_sp_t           I  TEMPDAP, correction fraction, 2^2
 *
 * RETURNS: agc_dp_words_t -- PACTTMP or YACTTMP, 1 ascrev
 **********************************************************************/
static agc_dp_words_t tvcx_cgcorr(agc_dp_words_t deloff, agc_dp_words_t delbar, agc_sp_t frac)
{
    agc_dp_words_t act = deloff; /* DCA xDELOFF; DXCH xACTTMP       */

    if (!AGC_ASSERT(frac >= 0))
    {
        return (act);
    }

    /* DCS xDELOFF; DDOUBL; DDOUBL; DXCH TTMP1 (4 ascrev scale)          */
    agc_dp_words_t ttmp1 = csm_dap_words_ddoubl(csm_dap_words_ddoubl(csm_dap_words_neg(deloff))); /* B+2 ascrev  */

    /* DCA DELxBAR; DDOUBL; DDOUBL; DAS TTMP1                             */
    (void) csm_dap_words_das(&ttmp1, csm_dap_words_ddoubl(csm_dap_words_ddoubl(delbar)));

    /* DCA TTMP1; MP TEMPDAP (upper word only); DAS xACTTMP               */
    (void) csm_dap_words_das(&act, csm_dap_words_mp(ttmp1.hi, frac));

    /* a word out of range: leave the trim estimate uncorrected         */
    if (!AGC_ASSERT(act.hi >= AGC_SP_MIN))
    {
        return (deloff);
    }

    return (act);
}

/**********************************************************************
 * CORCOPY +1 (TVCEXECUTIVE.agc p. 949) -- copy the new trim estimates
 * and trims (restart-protected): xACTOFF = upper word of xACTTMP,
 * xDELOFF = xACTTMP.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void tvcx_correction_copy(void)
{
    agc_csm_tvc_t    *t = agc_csm_dap_tvc(); /* TVC DAP erasables       */
    agc_csm_dapcom_t *c = agc_csm_dap_com(); /* PACTOFF, YACTOFF        */

    if (!AGC_ASSERT(t->tvcexphs == 5))
    {
        return;
    }

    c->pactoff  = t->pacttmp.hi; /* DCA PACTTMP; TS PACTOFF          */
    t->pdeloff  = t->pacttmp;    /* DXCH PDELOFF                     */
    c->yactoff  = t->yacttmp.hi;
    t->ydeloff  = t->yacttmp;
    t->tvcexphs = 6;             /* INCR (6)                         */

    if (!AGC_ASSERT(c->pactoff == t->pdeloff.hi))
    {
        return;
    }
}

/**********************************************************************
 * One step of EXECCOPY +1 .. TVCEXFIN (TVCEXECUTIVE.agc pp. 947-949):
 * 1SHOTOK (engine on: TEMPSET), TEMPSET (fraction .25), CORSETUP and
 * both CG.CORR axes, CORCOPY, CNTRCOPY.
 *
 * ARGUMENTS:
 *   s  tvcx_step_t  I  the step to run
 *
 * RETURNS: tvcx_step_t -- the next step (TVCX_FIN to finish)
 **********************************************************************/
static tvcx_step_t tvcx_step(tvcx_step_t s)
{
    agc_csm_tvc_t *t    = agc_csm_dap_tvc(); /* TVC DAP erasables       */
    tvcx_step_t    next = TVCX_FIN;          /* next step               */

    switch (s)
    {
        case TVCX_EXEC_COPY:
            next = tvcx_exec_copy();
            break;
        case TVCX_ONE_SHOT_CHECK:
            next = tvcx_one_shot_check();
            break;
        case TVCX_REPCHEK:
            next = tvcx_repchek();
            break;
        case TVCX_ONE_SHOT_OK: /* no one-shot during engine shutdown   */

            if (csm_dap_engine_on())
            {
                t->tvcexphs = 3; /* INCR TVCEXPHS (3)                   */
                next        = TVCX_TEMPSET;
            }

            break;
        case TVCX_TEMPSET:     /* one-shot correction fraction */
            t->tempdap[1] = FCORFRAC;
            t->tvcexphs   = 4; /* INCR (4)                     */
            next          = TVCX_CORSETUP;
            break;
        case TVCX_CORSETUP:
            tvcx_corsetup();
            t->pacttmp    = tvcx_cgcorr(t->pdeloff, t->delpbar, t->tempdap[0]);
            t->yacttmp    = tvcx_cgcorr(t->ydeloff, t->delybar, t->tempdap[0]);
            t->tvcexphs   = 5; /* CORCOPY: INCR (5)            */
            next          = TVCX_CORRECTION_COPY;
            break;
        case TVCX_CORRECTION_COPY:
            tvcx_correction_copy();
            next          = TVCX_CNTRCOPY;
            break;
        case TVCX_CNTRCOPY:    /* restarts OK, follows copy    */
            t->cntr       = t->cntrtmp;
            break;
        case TVCX_FIN:
        default:
            break;
    }

    return (next);
}

/**********************************************************************
 * EXECCOPY +1 .. TVCEXFIN (TVCEXECUTIVE.agc pp. 947-949) -- run the
 * step sequence from a restart point to TVCEXFIN (TVCEXPHS = 0,
 * TASKOVER).
 *
 * ARGUMENTS:
 *   from  tvcx_step_t  I  first step (a TVCEXPHS restart point,
 *                         1SHOTCHK, or TVCX_FIN to finish at once)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_NO_CONVERGE when the
 *          sequence does not reach TVCEXFIN within TVCX_MAX_STEPS
 **********************************************************************/
static agc_status_t tvcx_run(tvcx_step_t from)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */
    tvcx_step_t    s = from;              /* current step               */

    for (int i = 0; (i < TVCX_MAX_STEPS) && (s != TVCX_FIN); i++)
    {
        s = tvcx_step(s);

        if (!AGC_ASSERT((s >= TVCX_EXEC_COPY) && (s <= TVCX_FIN)))
        {
            return (AGC_STATUS_NO_CONVERGE);
        }
    }

    if (!AGC_ASSERT(s == TVCX_FIN))
    {
        return (AGC_STATUS_NO_CONVERGE);
    }

    t->tvcexphs = 0; /* TVCEXFIN: reset TVCEXPHS                       */

    return (AGC_STATUS_OK); /* TASKOVER                                */
}

/**********************************************************************
 * EXRSTRT targets (TVCRESTARTS.agc p. 959) -- INDEX TVCEXPHS into
 * TVCEXADR: re-enter the TVC executive at its restart point.
 *
 * ARGUMENTS:
 *   tvcexphs  agc_sp_t  I  TVCEXPHS, 1 .. 6
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a
 *          TVCEXPHS outside 1 .. 6
 **********************************************************************/
agc_status_t csm_tvc_exec_restart(agc_sp_t tvcexphs)
{
    if (!AGC_ASSERT((tvcexphs >= 1) && (tvcexphs <= 6)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = tvcx_run((tvcx_step_t) tvcexphs); /* to TVCEXFIN  */

    if (!AGC_ASSERT(agc_csm_dap_tvc()->tvcexphs == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * TVCEXEC (TVCEXECUTIVE.agc p. 946) -- waitlist task every 1/2 s,
 * called by TVCINIT4 and by itself; terminates (TVCEXFIN) when FLAGWRD6
 * bits 15,14 (DAPBIT1, DAPBIT2) no longer read 10.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the first failure of the
 *          WAITLIST calls, NEEDLER, S40.15 or the step sequence
 **********************************************************************/
agc_status_t agc_task_tvcexec(void)
{
    bool b1 = agc_flag_test(AGC_FLAG_DAPBIT1); /* FLAGWRD6 bit 15       */
    bool b2 = agc_flag_test(AGC_FLAG_DAPBIT2); /* FLAGWRD6 bit 14       */

    if (!AGC_ASSERT(agc_flag_valid(AGC_FLAG_DAPBIT1)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!b1 || b2)
    {
        return (tvcx_run(TVCX_FIN)); /* CS FLAGWRD6; MASK; BZMF TVCEXFIN */
    }

    agc_status_t st   = agc_waitlist_call(HALF_SEC_CS, AGC_TASK_TVCEXEC);
    agc_status_t s2   = tvcx_rollprep();    /* ROLLPREP               */
    agc_status_t s3   = csm_dap_needler();  /* NEEDLEUP               */
    agc_status_t s4   = AGC_STATUS_OK;      /* S40.15 in GAINCHNG     */
    tvcx_step_t  from = tvcx_vargains(&s4); /* VARGAINS               */
    agc_status_t s5   = tvcx_run(from);     /* .. TVCEXFIN            */
    agc_status_t r    = st;                 /* first failure          */

    if (r == AGC_STATUS_OK)
    {
        r = (s2 != AGC_STATUS_OK) ? s2 : s3; /* ROLLPREP, else NEEDLER */
    }

    if (r == AGC_STATUS_OK)
    {
        r = (s4 != AGC_STATUS_OK) ? s4 : s5; /* S40.15, else the steps */
    }

    return (r);
}
