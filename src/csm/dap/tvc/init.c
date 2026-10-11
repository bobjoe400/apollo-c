/**********************************************************************
 * FILE NAME: init.c
 *
 * PURPOSE:
 *   TVC DAP initialisation and start-up: gains, timing parameters and
 *   filter variables, the steering gain KPRIMEDT (S40.8), the
 *   initialisation call to NEEDLER, the roll DAP, TVCEXEC by waitlist and
 *   DAPINIT by T5; and the CSM/LM switch-over to the low-bandwidth
 *   coefficients and gains (V46). Labels: TVCDAPON, MRCLEAN, TVCINIT1 ..
 *   TVCINIT4, ATTINIT, NEEDLEIN, PRESWTCH, SWICHOVR, MCLEANJR, LOADCOEF,
 *   S40.15, CSMN10, LBN10.
 *
 * AGC SOURCE:
 *   Comanche055 TVCINITIALIZE.agc pp. 937-944 (CSM only)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   CSMMASS       agc_sp_t  I/O  2^16 kg        vehicle mass
 *   DAPDATR1      uint16_t  I    bits           bit 14 LM on, 13 LM off
 *   CDUX          uint16_t  I    2^15 = 1 rev   outer gimbal (raw)
 *   CHAN11        uint16_t  I    bits           bit 13 SPS engine on
 *   FLAGWRD9      flags     O    bit 15         SWTOVER
 *   TVC block     struct    I/O  per field      agc_csm_dap_tvc()
 *   pad loads     struct    I    per field      EMDOT, EKPRIME, EKTLX/I,
 *                                               EREPFRAC, HBN10, PACTOFF,
 *                                               YACTOFF (agc_csm_dap_com())
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_csm_dap_massprop  MASSPROP: IXX, IAVG/TLX (massprop.c)
 *   csm_dap_needler       NEEDLER initialisation pass (needles.c)
 *   agc_waitlist_call     WAITLIST: TVCEXEC in 1/2 s
 *   csm_dap_set_t5        T5LOC / TIME5 (mysubs.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG when an assertion on the DAP
 *   state fails; a WAITLIST failure is passed on.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   T5 tasks TVCDAPON (from DOTVCON, P40) and TVCINIT1 (from MRCLEAN);
 *   MRCLEAN and TVCINIT4 are TVC-restart entry points, SWICHOVR +5 the
 *   switch-over restart entry. ERRBTMP holds -ERRORY, -ERRORZ as two
 *   single words when TVCDAPON starts (P40 DOTVCON). CSM only.
 *
 * NOTES:
 *   LOADCOEF copies 15 words: for the CSM alone the last five come from
 *   LBN10, which follows CSMN10 in fixed memory (csmn10 below repeats
 *   them); the two-cascade CSM filter does not use them.
 *   Constants checked against the assembled Comanche055 rope.
 *
 * ALGORITHM:
 *   MRCLEAN zeroes OMEGAYC .. TTMP1 (52 words) and calls TVCINIT1 by T5
 *   in 10 ms. TVCINIT1: MASSPROP, 10-s mass loss, filter coefficients,
 *   KTLX/I, S40.15; TVCINIT2: filter period, T5TVCDT, KPRIMEDT, REPFRAC,
 *   one-shot counter; TVCINIT3: trims to trackers; ATTINIT (CSM alone):
 *   limit the RCS errors to 1.5 deg into PERRB, YERRB; NEEDLEIN; TVCINIT4:
 *   TVCPHASE = 0, OGANOW, mass copycycle, TVCEXEC, DAPINIT.
 **********************************************************************/
#include "../internal.h"

#define NZERO        51                  /* 00063: words to zero less one (odd) */
#define NZEROJR      23                  /* 00027: MCLEANJR words less one      */
#define ATTLIM       ((agc_sp_t) 136)    /* 00210 DEC 0.00833: 1.5 deg, B-1 rev */
#define RECIP_ATTLIM ((agc_sp_t) 120)    /* 00170 DEC 0.007325 (1/ATTLIM)       */
#define FKPRIMDT     ((agc_sp_t) 167)    /* 00247 DEC .0102: CSM/LM LB, pi/8    */
#define FREPFRAC     ((agc_sp_t) 154)    /* 00232 DEC .0375 B-2: CSM/LM LB, 2^2 */
#define TWOPI_M      ((agc_sp_t) 13884)  /* 33074 DEC .00331017 B+8: 2^-8/(N m)*/
#define ONETHOU      ((agc_sp_t) 2000)   /* 03720 DEC 1000 B-13: kg/cs -> kg/10 s*/
#define NINETEEN     ((agc_sp_t) 19)     /* VD1 (OCT 23): 20 TVCEXEC passes = 10 s*/
#define HALF_SEC_CS  ((uint16_t) 50)     /* .5SEC DEC 50, cs                    */
#define T5_POSMAX    ((uint16_t) 037777) /* POSMAX: next T5RUPT in 10 ms     */
#define NODES        6                   /* PTMP1..6, YTMP1..6               */

/* TCORR: one-shot correction time (TVCEXEC passes): CSM 5, CSM/LM 0.     */
static const agc_sp_t tcorr[2] = {5, 0};

/* CSMN10 (p. 943), CSM-only filter coefficients, scale 1. LOADCOEF copies
 * 15 words, so the last five (N30..D32) are the first five words of
 * LBN10 that follow CSMN10 in fixed memory; the CSM filter ignores them. */
static const agc_sp_t csmn10[AGC_CSM_N10_WORDS] = {
    16383,                             /* N10    DEC .99999                                        */
    -4176,                             /* N11/2  DEC -.2549                                        */
    963,                               /* N12    DEC .0588                                         */
    -12485,                            /* D11/2  DEC -.7620                                        */
    12206,                             /* D12    DEC .7450                                         */
    16383,                             /* N20    DEC .99999                                        */
    -7950,                             /* N21/2  DEC -.4852                                        */
    0,                                 /* N22    DEC 0                                             */
    -4411,                             /* D21/2  DEC -.2692                                        */
    0,                                 /* D22    DEC 0                                             */
    16383, -5382, -5408, -14911, 13861 /* LBN10 .. LBD12 (overrun)       */
};

/* LBN10 (p. 943), CSM/LM low-bandwidth filter coefficients, scale 1.     */
static const agc_sp_t lbn10[AGC_CSM_N10_WORDS] = {
    16383,  /* N10    DEC +.99999                                       */
    -5382,  /* N11/2  DEC -.3285                                        */
    -5408,  /* N12    DEC -.3301                                        */
    -14911, /* D11/2  DEC -.9101                                        */
    13861,  /* D12    DEC +.8460                                        */
    512,    /* N20    DEC +.03125                                       */
    0,      /* N21/2  DEC 0                                             */
    0,      /* N22    DEC 0                                             */
    -14911, /* D21/2  DEC -.9101                                        */
    13861,  /* D22    DEC +.8460                                        */
    8192,   /* N30    DEC +.5000                                        */
    -7719,  /* N31/2  DEC -.47115                                       */
    7781,   /* N32    DEC +.4749                                        */
    -15660, /* D31/2  DEC -.9558                                        */
    15355   /* D32    DEC +.9372                                        */
};

/* COEFFADR: which coefficient block LOADCOEF copies. */
typedef enum tvc_coef_set
{
    TVC_COEF_CSM = 0, /* CSMCFADR: CSMN10 (fixed)                     */
    TVC_COEF_HB,      /* HBCFADR: HBN10 (erasable pad load)           */
    TVC_COEF_LB       /* LBCFADR: LBN10 (fixed)                       */
} tvc_coef_set_t;

/**********************************************************************
 * LOADCOEF (TVCINITIALIZE.agc p. 942) -- load the DAP filter
 * coefficients N10..D32 from fixed memory (CSM, CSM/LM low bandwidth)
 * or from the erasable pad load (CSM/LM high bandwidth).
 *
 * ARGUMENTS:
 *   set  tvc_coef_set_t  I  COEFFADR: CSM, HB or LB block
 **********************************************************************/
static void tvc_loadcoef(tvc_coef_set_t set)
{
    agc_csm_tvc_t          *t = agc_csm_dap_tvc(); /* TVC DAP erasables */
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* HBN10 pad load    */

    if (!AGC_ASSERT((set == TVC_COEF_CSM) || (set == TVC_COEF_HB) || (set == TVC_COEF_LB)))
    {
        return;
    }

    for (int i = 0; i < AGC_CSM_N10_WORDS; i++)
    {
        if (set == TVC_COEF_HB)
        {
            t->n10[i] = c->hbn10[i];
        }
        else if (set == TVC_COEF_LB)
        {
            t->n10[i] = lbn10[i];
        }
        else
        {
            t->n10[i] = csmn10[i];
        }
    }

    (void) AGC_ASSERT(t->n10[0] != 0);
}

/**********************************************************************
 * S40.15 (TVCINITIALIZE.agc p. 943) -- gain computations, entered from
 * TVCINITIALIZE and TVCEXEC. full: 1/CONACC = IXX (2PI/M) x 8, scaled
 * B+9 s^2/rev (IXX at B+20 kg m^2); then (S40.15 +7, the V46 switch-over
 * entry) VARK = KTLX/I x IAVG/TLX x 4, scaled B+3 ascrev.
 *
 * ARGUMENTS:
 *   full  bool  I  true: entry at S40.15; false: entry at S40.15 +7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG when IXX or
 *          IAVG/TLX is negative
 **********************************************************************/
agc_status_t csm_tvc_gains(bool full)
{
    agc_csm_tvc_t          *t = agc_csm_dap_tvc(); /* TVC DAP erasables */
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* IXX, IAVG/TLX     */

    if (!AGC_ASSERT(c->ixx >= 0) || !AGC_ASSERT(c->iavg_tlx >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (full)
    {
        agc_dp_t p = agc_sp_mul(c->ixx, TWOPI_M); /* MP 2PI/M, B+12  */

        p             = csm_dap_ddoubl(csm_dap_ddoubl(csm_dap_ddoubl(p)));
        t->one_conacc = agc_dp_high(p); /* TS 1/CONACC, B+9 s^2/rev  */
    }

    agc_dp_t v = agc_sp_mul(t->ktlx_i, c->iavg_tlx); /* MP, B+5 ascrev */

    v       = csm_dap_ddoubl(csm_dap_ddoubl(v));
    t->vark = agc_dp_high(v); /* TS VARK, B+3 ascrev                   */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * MRCLEAN .. ENDMRC (TVCINITIALIZE.agc pp. 937-938) -- zero the 52 TVC
 * words OMEGAYC .. TTMP1 ("TVC zeroing starts here" .. "ends here", and
 * TTMP1), then a T5RUPT in 10 ms continues at TVCINIT1. TVC restarts
 * (CMDSOUT with TVCPHASE = -1) also enter here.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG when the T5
 *          set-up fails
 **********************************************************************/
agc_status_t csm_tvc_mrclean(void)
{
    agc_csm_tvc_t          *t    = agc_csm_dap_tvc(); /* TVC erasables  */
    const agc_csm_dp_pair_t zero = {0, 0};            /* +0, +0         */

    _Static_assert(NZERO % 2 == 1, "AGC constant");
    t->omegac[1] = 0;
    t->omegac[2] = 0; /* OMEGAYC, OMEGAZC                          */

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        t->omegab[i] = zero; /* OMEGAB (overlaid by OGARATE)        */
    }

    t->ogarate = 0;

    for (int i = 0; i < NODES; i++)
    {
        t->ptmp[i] = zero; /* PTMP1..6, YTMP1..6                     */
        t->ytmp[i] = zero;
    }

    t->rollfire     = 0;
    t->rollword     = 0u;
    t->temreg       = 0;
    t->stroker      = 0;
    t->perrb        = zero;
    t->yerrb        = zero;
    t->delpbar      = zero;
    t->delybar      = zero;
    t->pdeloff      = zero;
    t->ydeloff      = zero;
    t->ttmp1        = 0;
    t->cntr         = 0; /* CNTR, the loop counter, ends at +0           */
    agc_status_t st = csm_dap_set_t5(AGC_TASK_TVC_INITIALIZE, T5_POSMAX);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st); /* ENDMRC: RESUME                                     */
}

/**********************************************************************
 * TVCDAPON (TVCINITIALIZE.agc p. 937) -- T5 entry from DOTVCON (P40):
 * falls into MRCLEAN.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of MRCLEAN; AGC_STATUS_BAD_ARG when
 *          TVCPHASE is not -1
 **********************************************************************/
agc_status_t agc_task_tvcdapon(void)
{
    if (!AGC_ASSERT(agc_csm_dap_tvc()->tvcphase == -1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = csm_tvc_mrclean(); /* MRCLEAN                    */

    (void) AGC_ASSERT(st == AGC_STATUS_OK);

    return (st);
}

/**********************************************************************
 * TVCINIT2 (TVCINITIALIZE.agc pp. 938-939) -- filter period, T5TVCDT,
 * steering gain KPRIMEDT, trim-loop fraction, one-shot counter, stroke-
 * test and gain-update counters, for LM off or on.
 *
 * ARGUMENTS:
 *   lem  int  I  CNTR: 0 LM off, 1 LM on
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a bad
 *          lem or T5TVCDT
 **********************************************************************/
static agc_status_t tvc_init_filter_timing(int lem)
{
    agc_csm_tvc_t          *t = agc_csm_dap_tvc(); /* TVC DAP erasables */
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* pad loads         */

    if (!AGC_ASSERT((lem == 0) || (lem == 1)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* CS CNTR; INDEX A; CAF BIT2: TVCDT/2, scale B+14 cs: BIT2 (20 ms,
     * CSM 40 ms filter) or BIT3 (40 ms, CSM/LM 80 ms filter)            */
    agc_sp_t half_dt = (agc_sp_t) ((lem != 0) ? 4 : 2); /* ?: LM on/off */

    t->t5tvcdt = (agc_sp_t) (AGC_SP_MAX - half_dt + 1); /* COM; AD POSMAX; AD BIT1 */
    agc_flag_clear(AGC_FLAG_SWTOVER);                   /* reset SWTOVER          */

    /* KPRIMEDT = EKPRIME (100 pi/16) x TVCDT/2: LXCH A exchanges the
     * product words (14 places left); scaled pi/8                       */
    agc_sp_t ek = c->ekprime[lem]; /* EKPRIME, (100 pi)/16   */

    t->kprimedt          = agc_dp_pack(csm_dap_mp_lo(ek, half_dt), csm_dap_mp_hi(ek, half_dt));
    t->repfrac           = c->erepfrac[lem]; /* REPFRAC, 2^2             */
    t->cntr              = tcorr[lem];       /* one-shot correction time */
    t->strktime          = -1;               /* no stroke test until V68 */
    t->vcntr             = NINETEEN;         /* gain update in 10 s      */
    t->engine_fail_vcntr = NINETEEN;         /* V97VCNTR for ENGFAIL     */

    (void) AGC_ASSERT(t->t5tvcdt > 0);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * TVCINIT3 (TVCINITIALIZE.agc p. 939) -- trim values to the trim
 * trackers, output trackers, offset updates and offset-tracker filters
 * (upper words only; the lower words were zeroed by MRCLEAN).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void tvc_init_trims(void)
{
    agc_csm_tvc_t          *t = agc_csm_dap_tvc(); /* TVC DAP erasables */
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* PACTOFF, YACTOFF  */

    t->pdeloff.hi = c->pactoff; /* CAE PACTOFF; TS PDELOFF, PCMD, DELPBAR */
    t->pcmd       = c->pactoff;
    t->delpbar.hi = c->pactoff;
    t->ydeloff.hi = c->yactoff;
    t->ycmd       = c->yactoff;
    t->delybar.hi = c->yactoff;

    if (!AGC_ASSERT(t->pcmd == c->pactoff))
    {
        return;
    }

    (void) AGC_ASSERT(t->ycmd == c->yactoff);
}

/**********************************************************************
 * ATTINIT (TVCINITIALIZE.agc pp. 939-940) -- attitude-error
 * initialisation for the CSM alone (bypassed for CSM/LM): the RCS
 * attitude errors -ERRORY, -ERRORZ that DOTVCON left in the two words
 * of ERRBTMP are limited to +-1.5 deg (yaw first, then pitch) and
 * become the upper words of PERRB, YERRB (B-1 rev).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void tvc_attinit(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */
    agc_sp_t      *w[2];                  /* ERRBTMP, ERRBTMP +1        */

    if ((agc_erase_dapdatr1() & CSM_DAP_BIT13) == 0u)
    {
        return; /* CSM/LM: BZF NEEDLEIN                               */
    }

    w[0] = &t->errbtmp.hi;
    w[1] = &t->errbtmp.lo;

    for (int k = 1; k >= 0; k--)
    { /* TTMP1 = 1, then 0   */
        agc_sp_t q = csm_dap_mp_hi(*w[k], RECIP_ATTLIM); /* 0: < 1.5 deg */

        if (q > 0)
        {
            *w[k] = ATTLIM; /* |error| > 1.5 deg, and positive          */
        }
        else if (q < 0)
        {
            *w[k] = (agc_sp_t) -ATTLIM; /* |error| > 1.5 deg, negative  */
        }
        else
        {
            /* |error| less than 1.5 deg: keep it */
        }
    }

    t->perrb.hi = t->errbtmp.hi; /* CA ERRBTMP; TS PERRB                 */
    t->yerrb.hi = t->errbtmp.lo; /* CA ERRBTMP +1; TS YERRB              */

    (void) AGC_ASSERT((t->perrb.hi >= -ATTLIM) && (t->perrb.hi <= ATTLIM));
}

/**********************************************************************
 * TVCINIT1 .. NEEDLEIN (TVCINITIALIZE.agc pp. 938-940) -- T5 entry set
 * up by MRCLEAN: MASSPROP, the 10-s mass loss TENMDOT and MASSTMP, the
 * filter coefficients, KTLX/I, S40.15, TVCINIT2, TVCINIT3, ATTINIT, the
 * NEEDLER initialisation pass, then TVCINIT4.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the first failure of
 *          S40.15, TVCINIT2, NEEDLER or TVCINIT4
 **********************************************************************/
agc_status_t agc_task_tvc_initialize(void)
{
    agc_csm_tvc_t    *t = agc_csm_dap_tvc(); /* TVC DAP erasables       */
    agc_csm_dapcom_t *c = agc_csm_dap_com(); /* pad loads, RCSFLAGS     */

    agc_csm_dap_massprop(); /* IXX, IAVG/TLX for the DAP gains            */

    /* TENMDOT = EMDOT (kg/cs, B+3) x 1000: 10-s mass loss, kg B+16       */
    t->tenmdot = csm_dap_mp_hi(c->emdot, ONETHOU);
    t->masstmp = csm_dap_ads(agc_erase_csmmass(), (agc_sp_t) -t->tenmdot);
    int lem    = ((agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u) ? 1 : 0;

    /* CNTR: ?: LM on (BIT1) or off (0)                                   */

    tvc_loadcoef((lem != 0) ? TVC_COEF_HB : TVC_COEF_CSM); /* ?: LM   */
    t->ktlx_i       = c->ektlx_i[lem];                     /* 1/(8 ascrev) scale        */
    agc_status_t st = csm_tvc_gains(true);                 /* 1/CONACC, VARK           */

    if (st == AGC_STATUS_OK)
    {
        st = tvc_init_filter_timing(lem);
    }

    tvc_init_trims();
    tvc_attinit();

    /* NEEDLEIN: set RCSFLAGS bit 3 for the initialisation pass          */
    c->rcsflags     = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NEEDLE1);
    agc_status_t s2 = csm_dap_needler(); /* NEEDLER                    */

    if (st == AGC_STATUS_OK)
    {
        st = s2;
    }

    agc_status_t s3 = csm_tvc_init_complete(); /* TVCINIT4            */

    return ((st != AGC_STATUS_OK) ? st : s3); /* first failure          */
}

/**********************************************************************
 * TVCINIT4 .. ENDTVCIN (TVCINITIALIZE.agc pp. 940-941) -- TVCPHASE = 0
 * (TVCDAPON-through-NEEDLEIN initialisation finished; TVC-restart
 * entry), roll DAP OGANOW, the CSMMASS copycycle if the engine is on
 * (if it is off, ENGINOFF's MASSBACK has set the post-burn mass),
 * TVCEXEC in 0.5 s and DAPINIT after T5TVCDT.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, the WAITLIST or T5 failure, or
 *          AGC_STATUS_BAD_ARG when T5TVCDT is not positive
 **********************************************************************/
agc_status_t csm_tvc_init_complete(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    t->tvcphase = 0;
    t->oganow   = agc_erase_cdux(); /* prepare the roll DAP             */

    if (csm_dap_engine_on())
    {
        agc_erase_set_csmmass(t->masstmp); /* COPYCYCLE, 2^16 kg        */
    }

    agc_status_t st = agc_waitlist_call(HALF_SEC_CS, AGC_TASK_TVCEXEC);

    if (st == AGC_STATUS_OK)
    { /* DAPINIT after T5TVCDT: time for restart computations */
        st = csm_dap_set_t5(AGC_TASK_DAPINIT, (uint16_t) t->t5tvcdt);
    }

    if (!AGC_ASSERT(t->t5tvcdt > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st); /* ENDTVCIN: RESUME                                   */
}

/**********************************************************************
 * SWICHOVR +5 .. TC RTRNLOC (TVCINITIALIZE.agc pp. 941-942) -- restart
 * entry point with TVCPHASE and PHASETMP already set. Zero the filter
 * nodes (MCLEANJR, which leaves CNTRTMP = 1), set the switch-over flag
 * for the downlink, load the low-bandwidth DAP, steering and trim-loop
 * gains, update the trim estimates, load the low-bandwidth
 * coefficients and restore TVCPHASE.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of S40.15 +7, or AGC_STATUS_BAD_ARG
 *          when the restored TVCPHASE is out of range
 **********************************************************************/
agc_status_t csm_tvc_swichovr_body(void)
{
    agc_csm_tvc_t          *t    = agc_csm_dap_tvc(); /* TVC erasables  */
    const agc_csm_dapcom_t *c    = agc_csm_dap_com(); /* EKTLX/I +2     */
    const agc_csm_dp_pair_t zero = {0, 0};            /* +0, +0         */

    _Static_assert(NZEROJR + 1 == 2 * NODES * 2, "AGC constant");

    for (int i = 0; i < NODES; i++)
    { /* MCLEANJR: PTMP1 .. YTMP6                  */
        t->ptmp[i] = zero;
        t->ytmp[i] = zero;
    }

    t->cntrtmp      = 1;                    /* the zeroing loop's counter ends at 1           */
    agc_flag_set(AGC_FLAG_SWTOVER);
    t->ktlx_i       = c->ektlx_i[2];        /* low-bandwidth DAP gain   */
    agc_status_t st = csm_tvc_gains(false); /* S40.15 +7: VARK only     */

    t->kprimedt = agc_dp_pack(FKPRIMDT, agc_dp_low(t->kprimedt)); /* steering, pi/8 */
    t->repfrac  = FREPFRAC;                                       /* TMC loop, 2^2                        */
    t->pdeloff  = t->delpbar;                                     /* update the trim estimates            */
    t->ydeloff  = t->delybar;
    tvc_loadcoef(TVC_COEF_LB);
    t->tvcphase = t->phasetmp;                                    /* restore TVCPHASE                    */

    if (!AGC_ASSERT((t->tvcphase >= -2) && (t->tvcphase <= 3)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * PRESWTCH, SWICHOVR (TVCINITIALIZE.agc p. 941) -- entry from V46
 * (CSM/LM switch-over to the low-bandwidth DAP): INHINT, save TVCPHASE
 * in PHASETMP, TVCPHASE = -2 for the restart logic, SWICHOVR +5, exit
 * through PINBRNCH (which does the RELINT).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of SWICHOVR +5
 **********************************************************************/
agc_status_t agc_csm_tvc_preswtch(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    agc_inhint();
    t->phasetmp     = t->tvcphase;             /* save TVCPHASE                         */
    t->tvcphase     = -2;
    agc_status_t st = csm_tvc_swichovr_body(); /* SWICHOVR +5          */

    agc_relint(); /* POSTJUMP PINBRNCH                                 */

    (void) AGC_ASSERT(t->tvcphase != -2);

    return (st);
}
