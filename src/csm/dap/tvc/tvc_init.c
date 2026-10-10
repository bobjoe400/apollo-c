/*
 * tvc_init.c -- TVC DAP initialisation and start-up.
 * TVCINITIALIZE.agc (Comanche055 pp. 937-944): TVCDAPON, MRCLEAN,
 * TVCINIT1..TVCINIT4, ATTINIT, NEEDLEIN, PRESWTCH, SWICHOVR, MCLEANJR,
 * LOADCOEF, S40.15 and the filter coefficient tables.
 *
 * Performs TVC DAP initialisation (gains, timing parameters, filter
 * variables), computes the steering (S40.8) gain KPRIMEDT, makes the
 * initialisation call to NEEDLER, initialises the roll DAP, calls
 * TVCEXEC via waitlist and DAPINIT via T5, and provides for the loading
 * of the low-bandwidth coefficients and gains at the CSM/LM switch-over.
 * Called by DOTVCON (P40) with T5LOC = TVCDAPON, TIME5 = .6SECT5.
 * MRCLEAN and TVCINIT4 are TVC-restart entry points.
 */
#include "agc/agc_csm_dap_internal.h"

#define NZERO       51                 /* 00063: words to zero less one (odd) */
#define NZEROJR     23                 /* 00027: MCLEANJR words less one      */
#define ATTLIM      ((agc_sp_t)136)    /* 00210 DEC 0.00833: 1.5 deg, B-1 rev */
#define RECIP_ATTLIM ((agc_sp_t)120)   /* 00170 DEC 0.007325 (1/ATTLIM)       */
#define FKPRIMDT    ((agc_sp_t)167)    /* 00247 DEC .0102: CSM/LM LB, pi/8    */
#define FREPFRAC    ((agc_sp_t)154)    /* 00232 DEC .0375 B-2: CSM/LM LB      */
#define TWOPI_M     ((agc_sp_t)13884)  /* 33074 DEC .00331017 B+8: 1/(B+8 N M)*/
#define ONETHOU     ((agc_sp_t)2000)   /* 03720 DEC 1000 B-13: kg/cs -> kg/10 s*/
#define NINETEEN    ((agc_sp_t)19)     /* VD1 (OCT 23): 20 TVCEXEC passes=10 s*/
#define HALF_SEC_CS ((uint16_t)50)     /* .5SEC DEC 50                        */
#define T5_POSMAX   ((uint16_t)037777) /* POSMAX: next T5RUPT in 10 ms        */

/* TCORR: one-shot correction time (TVCEXEC passes): CSM 5, CSM/LM 0.     */
static const agc_sp_t tcorr[2] = { 5, 0 };

/* CSMN10 (p. 943), CSM-only filter coefficients, scale 1. LOADCOEF copies
 * 15 words, so for the CSM the last five (N30..D32) are the first five
 * words of LBN10 that follow CSMN10 in fixed memory; they are not used
 * by the CSM-only (two-cascade) filter.                                  */
static const agc_sp_t csmn10[AGC_CSM_N10_WORDS] = {
    16383,   /* N10    DEC .99999                                        */
    -4176,   /* N11/2  DEC -.2549                                        */
    963,     /* N12    DEC .0588                                         */
    -12485,  /* D11/2  DEC -.7620                                        */
    12206,   /* D12    DEC .7450                                         */
    16383,   /* N20    DEC .99999                                        */
    -7950,   /* N21/2  DEC -.4852                                        */
    0,       /* N22    DEC 0                                             */
    -4411,   /* D21/2  DEC -.2692                                        */
    0,       /* D22    DEC 0                                             */
    16383, -5382, -5408, -14911, 13861 /* LBN10 .. LBD12 (overrun)       */
};

/* LBN10 (p. 943), CSM/LM low-bandwidth filter coefficients, scale 1.     */
static const agc_sp_t lbn10[AGC_CSM_N10_WORDS] = {
    16383,   /* N10    DEC +.99999                                       */
    -5382,   /* N11/2  DEC -.3285                                        */
    -5408,   /* N12    DEC -.3301                                        */
    -14911,  /* D11/2  DEC -.9101                                        */
    13861,   /* D12    DEC +.8460                                        */
    512,     /* N20    DEC +.03125                                       */
    0,       /* N21/2  DEC 0                                             */
    0,       /* N22    DEC 0                                             */
    -14911,  /* D21/2  DEC -.9101                                        */
    13861,   /* D22    DEC +.8460                                        */
    8192,    /* N30    DEC +.5000                                        */
    -7719,   /* N31/2  DEC -.47115                                       */
    7781,    /* N32    DEC +.4749                                        */
    -15660,  /* D31/2  DEC -.9558                                        */
    15355    /* D32    DEC +.9372                                        */
};

typedef enum tvc_coef_set {     /* COEFFADR: CSMCFADR, HBCFADR, LBCFADR   */
    TVC_COEF_CSM = 0,
    TVC_COEF_HB,
    TVC_COEF_LB
} tvc_coef_set_t;

/* LOADCOEF (p. 942): load the DAP filter coefficients N10..D32 from
 * fixed memory (CSM, CSM/LM low bandwidth) or the erasable pad load
 * (CSM/LM high bandwidth).                                               */
static void tvc_loadcoef(tvc_coef_set_t set)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    (void)AGC_ASSERT((set == TVC_COEF_CSM) || (set == TVC_COEF_HB) ||
                     (set == TVC_COEF_LB));
    for (int i = 0; i < AGC_CSM_N10_WORDS; i++) {
        if (set == TVC_COEF_HB) {
            t->n10[i] = c->hbn10[i];
        } else if (set == TVC_COEF_LB) {
            t->n10[i] = lbn10[i];
        } else {
            t->n10[i] = csmn10[i];
        }
    }
    (void)AGC_ASSERT(t->n10[0] != 0);
}

/* S40.15 (p. 943): gain computations, entered from TVCINITIALIZE and
 * TVCEXEC. full: 1/CONACC = IXX (2PI/M) x 8, scaled B+9 s^2/rev (IXX at
 * B+20 kg m^2, 2PI/M at 1/(B+8 N M)); then (S40.15 +7, the V46 switch-over
 * entry) VARK = KTLX/I x IAVG/TLX x 4, scaled B+3 ascrev.                */
agc_status_t csm_tvc_gains(bool full)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    if (!AGC_ASSERT(c->ixx >= 0) || !AGC_ASSERT(c->iavg_tlx >= 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (full) {
        agc_dp_t p = agc_sp_mul(c->ixx, TWOPI_M);
        p = csm_dap_ddoubl(csm_dap_ddoubl(csm_dap_ddoubl(p)));
        t->one_conacc = agc_dp_high(p);            /* TS 1/CONACC          */
    }
    agc_dp_t v = agc_sp_mul(t->ktlx_i, c->iavg_tlx);
    v = csm_dap_ddoubl(csm_dap_ddoubl(v));
    t->vark = agc_dp_high(v);                      /* TS VARK              */
    return AGC_STATUS_OK;
}

/* MRCLEAN .. ENDMRC (pp. 937-938): zero the 52 TVC words OMEGAYC .. TTMP1
 * ("TVC zeroing starts here" .. "ends here", plus TTMP1), then a T5RUPT
 * in 10 ms continues at TVCINIT1. TVC restarts also enter here.         */
agc_status_t csm_tvc_mrclean(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    (void)AGC_ASSERT(NZERO % 2 == 1);
    t->omegac[1] = 0;  t->omegac[2] = 0;           /* OMEGAYC, OMEGAZC     */
    for (int i = 0; i < AGC_CSM_AXES; i++) {
        t->omegab[i] = 0;                          /* OMEGAB (= OGARATE)   */
    }
    t->ogarate = 0;
    for (int i = 0; i < 6; i++) {
        t->ptmp[i] = 0;                            /* PTMP1..6, YTMP1..6   */
        t->ytmp[i] = 0;
    }
    t->rollfire = 0;   t->rollword = 0u;   t->temreg = 0;  t->stroker = 0;
    t->perrb = 0;      t->yerrb = 0;       t->delpbar = 0; t->delybar = 0;
    t->pdeloff = 0;    t->ydeloff = 0;     t->ttmp1 = 0;
    t->cntr = 0;                                   /* CNTR (loop counter)  */
    agc_status_t st = csm_dap_set_t5(AGC_TASK_TVCINIT1, T5_POSMAX);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;                                     /* ENDMRC: RESUME       */
}

/* TVCDAPON (p. 937): T5 entry from DOTVCON (P40).                        */
agc_status_t agc_task_tvcdapon(void)
{
    (void)AGC_ASSERT(agc_csm_dap_tvc()->tvcphase == -1);
    agc_status_t st = csm_tvc_mrclean();
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;
}

/* TVCINIT2 (pp. 938-939): filter period, steering gain, trim-loop
 * fraction, one-shot counter, stroke-test and gain-update counters.
 * lem: 0 LM off, 1 LM on (CNTR).                                         */
static agc_status_t tvc_init2(int lem)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    if (!AGC_ASSERT((lem == 0) || (lem == 1))) {
        return AGC_STATUS_BAD_ARG;
    }
    /* TVCDT/2 in cs, scale B+14 cs: BIT2 (20 ms, CSM 40 ms filter) or
     * BIT3 (40 ms, CSM/LM 80 ms filter).                                  */
    agc_sp_t half_dt = (lem != 0) ? (agc_sp_t)4 : (agc_sp_t)2;
    t->t5tvcdt = (agc_sp_t)(AGC_SP_MAX - half_dt + 1); /* COM; AD POSMAX; AD BIT1 */
    agc_status_t st = agc_flag_clear(AGC_FLAG_SWTOVER);  /* reset SWTOVER   */
    /* KPRIMEDT = EKPRIME (100 pi/16) x TVCDT/2: the LXCH A exchanges the
     * product words, which shifts it 14 places left; scaled pi/8.        */
    agc_sp_t ek = c->ekprime[lem];
    t->kprimedt = agc_dp_pack(csm_dap_mp_lo(ek, half_dt), csm_dap_mp_hi(ek, half_dt));
    t->repfrac = c->erepfrac[lem];
    t->cntr = tcorr[lem];                          /* one-shot correction   */
    t->strktime = -1;                              /* no stroke test yet    */
    t->vcntr = NINETEEN;                           /* gain update in 10 s   */
    t->engine_fail_vcntr = NINETEEN;                        /* for ENGFAIL (R41)     */
    (void)AGC_ASSERT(t->t5tvcdt > 0);
    return st;
}

/* TVCINIT3 (p. 939): trim values to the trim trackers, output trackers,
 * offset updates and offset-tracker filters (low words zeroed by MRCLEAN). */
static void tvc_init3(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    t->pdeloff = agc_dp_pack(c->pactoff, agc_dp_low(t->pdeloff));
    t->pcmd = c->pactoff;
    t->delpbar = agc_dp_pack(c->pactoff, agc_dp_low(t->delpbar));
    t->ydeloff = agc_dp_pack(c->yactoff, agc_dp_low(t->ydeloff));
    t->ycmd = c->yactoff;
    t->delybar = agc_dp_pack(c->yactoff, agc_dp_low(t->delybar));
    (void)AGC_ASSERT(t->pcmd == c->pactoff);
    (void)AGC_ASSERT(t->ycmd == c->yactoff);
}

/* ATTINIT (pp. 939-940): attitude-error initialisation for the CSM alone
 * (bypassed for CSM/LM): the RCS attitude errors ERRORY, ERRORZ left in
 * ERRBTMP by DOTVCON are limited to +-1.5 deg and become PERRB, YERRB.   */
static void tvc_attinit(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    if ((agc_erase_dapdatr1() & CSM_DAP_BIT13) == 0u) {
        return;                                    /* CSM/LM: NEEDLEIN      */
    }
    for (int k = 1; k >= 0; k--) {                 /* TTMP1 = 1, 0          */
        agc_sp_t e = t->errbinit[k];
        agc_sp_t q = csm_dap_mp_hi(e, RECIP_ATTLIM); /* 0 if |e| < 1.5 deg  */
        if (q > 0) {
            t->errbinit[k] = ATTLIM;
        } else if (q < 0) {
            t->errbinit[k] = (agc_sp_t)-ATTLIM;
        } else {
            /* |error| less than 1.5 deg: keep it */
        }
    }
    t->perrb = agc_dp_pack(t->errbinit[0], agc_dp_low(t->perrb));
    t->yerrb = agc_dp_pack(t->errbinit[1], agc_dp_low(t->yerrb));
    (void)AGC_ASSERT((t->errbinit[0] >= -ATTLIM) && (t->errbinit[0] <= ATTLIM));
}

/* TVCINIT1 (p. 938): T5 entry set up by MRCLEAN.                         */
agc_status_t agc_task_tvcinit1(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    agc_status_t st = agc_csm_dap_massprop();      /* IXX, IAVG/TLX        */
    /* TENMDOT = EMDOT (kg/cs, B+3) x 1000: 10-s mass loss, kg B+16       */
    t->tenmdot = csm_dap_mp_hi(c->emdot, ONETHOU);
    t->masstmp = csm_dap_ads(agc_erase_csmmass(), (agc_sp_t)-t->tenmdot);
    int lem = ((agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u) ? 1 : 0;
    tvc_loadcoef((lem != 0) ? TVC_COEF_HB : TVC_COEF_CSM);
    t->ktlx_i = c->ektlx_i[lem];                   /* 1/(8 ascrev) scale    */
    agc_status_t s2 = csm_tvc_gains(true);         /* 1/CONACC, VARK        */
    agc_status_t s3 = tvc_init2(lem);
    tvc_init3();
    tvc_attinit();
    /* NEEDLEIN: set RCSFLAGS bit 3 for the initialisation pass, NEEDLER. */
    agc_csm_dapcom_t *cw = agc_csm_dap_com();
    cw->rcsflags = (uint16_t)(cw->rcsflags | AGC_CSM_RCSF_NEEDLE1);
    agc_status_t s4 = csm_dap_needler();
    if ((st != AGC_STATUS_OK) || (s2 != AGC_STATUS_OK) ||
        (s3 != AGC_STATUS_OK) || (s4 != AGC_STATUS_OK)) {
        return (st != AGC_STATUS_OK) ? st : ((s2 != AGC_STATUS_OK) ? s2 :
               ((s3 != AGC_STATUS_OK) ? s3 : s4));
    }
    return csm_tvc_init4();
}

/* TVCINIT4 .. ENDTVCIN (pp. 940-941): TVCPHASE = 0 (initialisation done;
 * possible TVC-restart entry), roll DAP OGANOW, the CSMMASS copycycle if
 * the engine is on, TVCEXEC in 0.5 s, DAPINIT after T5TVCDT.             */
agc_status_t csm_tvc_init4(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    t->tvcphase = 0;
    t->oganow = agc_erase_cdux();
    /* If the engine is already off, ENGINOFF (MASSBACK) has set the
     * post-burn CSMMASS; do not touch it.                                 */
    if (csm_dap_engine_on()) {
        agc_erase_set_csmmass(t->masstmp);         /* COPYCYCLE            */
    }
    agc_status_t st = agc_waitlist_call(HALF_SEC_CS, AGC_TASK_TVCEXEC);
    if (st == AGC_STATUS_OK) {                     /* allow time for restart */
        st = csm_dap_set_t5(AGC_TASK_DAPINIT, (uint16_t)t->t5tvcdt);
    }
    (void)AGC_ASSERT(t->t5tvcdt > 0);
    return st;                                     /* ENDTVCIN: RESUME     */
}

/* SWICHOVR +5 .. TC RTRNLOC (pp. 941-942): restart entry point with
 * TVCPHASE and PHASETMP already set. Zero the filter nodes (MCLEANJR),
 * set the switch-over flag for the downlink, load the low-bandwidth DAP,
 * steering and trim-loop gains, update the trim estimates, load the
 * low-bandwidth coefficients and restore TVCPHASE.                       */
agc_status_t csm_tvc_swichovr_body(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    (void)AGC_ASSERT(NZEROJR + 1 == 2 * 6 * 2);
    for (int i = 0; i < 6; i++) {                  /* MCLEANJR             */
        t->ptmp[i] = 0;
        t->ytmp[i] = 0;
    }
    agc_status_t st = agc_flag_set(AGC_FLAG_SWTOVER);
    t->ktlx_i = c->ektlx_i[2];                     /* low-bandwidth gains   */
    agc_status_t s2 = csm_tvc_gains(false);        /* S40.15 +7: VARK only  */
    t->kprimedt = agc_dp_pack(FKPRIMDT, agc_dp_low(t->kprimedt)); /* steering */
    t->repfrac = FREPFRAC;                         /* TMC loop             */
    t->pdeloff = t->delpbar;                       /* update trim estimates */
    t->ydeloff = t->delybar;
    tvc_loadcoef(TVC_COEF_LB);
    t->tvcphase = t->phasetmp;                     /* restore TVCPHASE     */
    (void)AGC_ASSERT((t->tvcphase >= -2) && (t->tvcphase <= 3));
    return (st != AGC_STATUS_OK) ? st : s2;
}

/* PRESWTCH, SWICHOVR (p. 941): entry from V46 (CSM/LM switch-over to the
 * low-bandwidth DAP). TVCPHASE = -2 tells the restart logic.             */
agc_status_t agc_csm_tvc_preswtch(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_inhint();
    t->phasetmp = t->tvcphase;                     /* save TVCPHASE        */
    t->tvcphase = -2;
    agc_status_t st = csm_tvc_swichovr_body();
    /* exit via PINBRNCH, which does the RELINT                            */
    agc_relint();
    (void)AGC_ASSERT(t->tvcphase != -2);
    return st;
}
