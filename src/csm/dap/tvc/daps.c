/**********************************************************************
 * FILE NAME: daps.c
 *
 * PURPOSE:
 *   The pitch and yaw thrust-vector-control autopilots: self-perpetuating
 *   T5 tasks generating the SPS gimbal actuator commands from the body-
 *   axis rate commands OMEGAYC, OMEGAZC of cross-product steering (S40.8),
 *   or holding attitude about the initial directions without steering.
 *   Labels: PITCHDAP, PSTROKER, PCDUDOTS, RLIMTEST, PINTEGRL, PERORLIM,
 *   PFORWARD, POFFSET, PACLIM, POUT, PPRECOMP, DELBARP, PCOPY, PMISC;
 *   YAWDAP .. YMISC, YCOPY; DAPINIT, ERRORLIM, ACTLIM, FWDFLTR (1DAPCAS,
 *   2DAPCAS, 3DAPCAS, OPTVARK), PRECOMP (2CASFLTR, 3CASFLTR).
 *
 * AGC SOURCE:
 *   Comanche055 TVCDAPS.agc pp. 961-978 (CSM only)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   CDUY, CDUZ    uint16_t  I    2^15 = 1 rev   gimbal angles (raw)
 *   SIN/COSCDUX,  agc_dp_t  I    2^1            QUICTRIG trig of the
 *   COSCDUZ                                     CDUs (agc_guid_state())
 *   DAPDATR1      uint16_t  I    bits           bit 14 LM on, bit 13 LM
 *                                               off (configuration)
 *   TVCPITCH,     agc_sp_t  I/O  85.41"/bit     optics error counters
 *   TVCYAW                                      CDUSCMD, CDUTCMD
 *   CHAN14        uint16_t  O    bits           bits 11, 12 count release
 *   TVC block     struct    I/O  per field      agc_csm_dap_tvc()
 *   AK1, AK2      agc_sp_t  O    B-1 rev        needle errors
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   csm_dap_pair_*        DAS, DDOUBL, MP on stored word pairs (mysubs.c)
 *   csm_dap_set_t5        T5LOC / TIME5 (mysubs.c)
 *   csm_tvc_hack          HACK, stroke test burst (stroke.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG when an assertion on the DAP
 *   state fails (the T5 call for the other axis is already made).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   T5 interrupt tasks, the two axes alternating every TVCDT/2 (20 ms
 *   CSM, 40 ms CSM/LM). The copycycles PCOPY / YCOPY are restart-
 *   protected by TVCPHASE 1..3 (restarts.c re-enters them). CSM only.
 *
 * NOTES:
 *   Every DP filter quantity is held as the AGC stores it, a pair of
 *   words whose signs may disagree (agc_dp_words_t), and every DAS,
 *   DDOUBL and MP is done on the words in the AGC's order. ERRORLIM,
 *   ACTLIM, POUT, DELBARP and the copycycle read the upper word alone,
 *   so the pair, not just its value, decides the actuator command
 *   (oracle finding F9; the pitch cases now agree bit for bit).
 *   DEVIATION: two DDOUBLs in a row that both overflow are corrected
 *   after each one (the AGC doubles the 16-bit A); this needs a filter
 *   output beyond 1/2 of full scale and does not occur within ERRLIM.
 *   FWDFLTR tests DAPDATR1 bit 14 (LM on) and PRECOMP bit 13 (LM off)
 *   exactly as the AGC does.
 *
 * ALGORITHM:
 *   Each pass: set T5 for the other axis, stroke-test burst if one is in
 *   progress, CDU rates (pitch pass only), body-axis rate, integrate the
 *   body-axis attitude error and limit it to 45 deg, the 6th-order filter
 *   (CSM two cascades, CSM/LM three) and the variable gain, add the trim
 *   offset, round and limit to 6 deg, send the increment to the error
 *   counter, precompute the filter nodes for the next pass, run the
 *   offset-tracker filter and copy everything back (copycycle).
 **********************************************************************/
#include "../internal.h"
#include "agc/guidance/guidance.h"

#define ACTSAT       ((agc_sp_t) 253)   /* 00375 DEC 253: 6 deg, 1 ascrev      */
#define RECIP_ACTSAT ((agc_sp_t) 65)    /* 00101 DEC .0039525692 (1/253)        */
#define ERRLIM       ((agc_sp_t) 4096)  /* BIT13: 45 deg filter input, B-1 rev */
#define RECIP_ERRLIM ((agc_sp_t) 4)     /* BIT3: 1/ERRLIM                      */
#define RECIP_RTLIM  ((agc_sp_t) 77)    /* 00115 DEC 0.004715: 2.33 deg test   */
#define ONE_M_EAT    ((agc_sp_t) 163)   /* 00243 1-E(-AT), AT = .01, scale 1  */
#define E_AT         ((agc_sp_t) 16221) /* 37535 E(-AT), scale 1             */
#define ROUND_HALF   ((agc_sp_t) 8192)  /* DOUBLE overflows: low word >= 1/2 */
#define NODES        6                  /* TMP1..TMP6 filter nodes           */

/* The two axes of one DAP pass (PITCHDAP / YAWDAP). */
typedef enum tvcd_axis
{
    TVCD_PITCH = 0, /* PITCHDAP: OMEGAYC, PERRB, PTMP, PCMD, TVCPITCH */
    TVCD_YAW   = 1  /* YAWDAP:   OMEGAZC, YERRB, YTMP, YCMD, TVCYAW   */
} tvcd_axis_t;

/**********************************************************************
 * RLIMTEST (TVCDAPS.agc p. 964) -- zero a CDU difference of more than
 * 2.33 deg in one sample period (a CDU transient, not a rate).
 *
 * ARGUMENTS:
 *   d  agc_sp_t  I  past - present CDU, 1 bit = 2^-15 rev
 *
 * RETURNS: agc_sp_t -- d, or 0 when .004715 d is not zero
 **********************************************************************/
static agc_sp_t tvcd_rlimtest(agc_sp_t d)
{
    agc_sp_t r = d; /* MCDUxDOT, 2^-15 rev per TVCDT/2                   */

    if (csm_dap_mp_hi(d, RECIP_RTLIM) != 0)
    {
        r = 0; /* CAF ZERO; TS TTMP1    */
    }

    return (r);
}

/**********************************************************************
 * PCDUDOTS (TVCDAPS.agc p. 964) -- minus CDU rates for pitch and yaw
 * (past - present), used by both DAPs; updates the past values.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void tvcd_pcdudots(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */
    uint16_t       y = agc_erase_cduy();  /* CDUY, raw counter word     */
    uint16_t       z = agc_erase_cduz();  /* CDUZ, raw counter word     */

    t->mcduydot = tvcd_rlimtest(csm_dap_msu(t->pcduypst, y)); /* XCH; MSU */
    t->pcduypst = y;
    t->mcduzdot = tvcd_rlimtest(csm_dap_msu(t->pcduzpst, z));
    t->pcduzpst = z;
}

/**********************************************************************
 * ERRORLIM (TVCDAPS.agc p. 970) -- limit the body-axis error input to
 * +-45 deg. Tests and overwrites the upper word of ERRBTMP only.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void tvcd_errorlim(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    if (csm_dap_mp_hi(t->errbtmp.hi, RECIP_ERRLIM) != 0)
    {
        /* CCS ERRBTMP: CAF ERRLIM (+) or CS ERRLIM (-)                    */
        t->errbtmp.hi = (t->errbtmp.hi > 0) ? ERRLIM : (agc_sp_t) -ERRLIM; /* ?: sign of the error */
    }
}

/**********************************************************************
 * ACTLIM (TVCDAPS.agc p. 970) -- round the command for the limit test
 * (DOUBLE of the lower word overflows into the TS L skip: +-1 when the
 * lower word is at least 1/2) and limit it to +-6 deg. As in the AGC
 * the rounded word is only tested; a limit overwrites the upper word.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void tvcd_actlim(void)
{
    agc_csm_tvc_t *t   = agc_csm_dap_tvc(); /* TVC DAP erasables        */
    agc_sp_t       rnd = 0;                 /* TS L overflow, scale 1 ascrev */

    if (t->cmdtmp.lo >= ROUND_HALF)
    {
        rnd = 1;
    }
    else if (t->cmdtmp.lo <= -ROUND_HALF)
    {
        rnd = -1;
    }
    else
    {
        /* CAF ZERO */
    }

    agc_sp_t a = csm_dap_ads(rnd, t->cmdtmp.hi); /* rounded, scale 1 ascrev */

    if (csm_dap_mp_hi(a, RECIP_ACTSAT) != 0)
    {
        /* CCS CMDTMP: CAF ACTSAT (+) or CS ACTSAT (-)                     */
        t->cmdtmp.hi = (t->cmdtmp.hi > 0) ? ACTSAT : (agc_sp_t) -ACTSAT; /* ?: sign of the command */
    }
}

/**********************************************************************
 * 1DAPCAS / 2DAPCAS / 3DAPCAS (TVCDAPS.agc pp. 972-973) -- one cascade's
 * present output DAPn = N x input + TMP(2k-1). The DXCH leaves the
 * overflow indicator of the first DAS (and +0) in TMP(2k-1), which
 * PRECOMP then rewrites.
 *
 * ARGUMENTS:
 *   in  agc_dp_words_t  I  cascade input (ERRBTMP, DAP1, DAP2), B-1 rev
 *   n   agc_sp_t           I  N10, N20 or N30, scale 1
 *   k   int                I  index of TMP1, TMP3 or TMP5 (0, 2, 4)
 *
 * RETURNS: agc_dp_words_t -- the cascade output, B-1 rev
 **********************************************************************/
static agc_dp_words_t tvcd_cascade_out(agc_dp_words_t in, agc_sp_t n, int k)
{
    agc_csm_tvc_t *t   = agc_csm_dap_tvc(); /* TVC DAP erasables     */
    agc_dp_words_t out = {0, 0};            /* DAPn, B-1 rev         */

    /* CAE in +1; MP N; TS DAPn +1 (DAPn zeroed by FWDFLTR); CA in; MP N;
     * DAS DAPn: the one-and-a-half precision product                     */
    out.lo              = csm_dap_mp_hi(in.lo, n);
    agc_sp_t       ovf  = csm_dap_words_das_carry(&out, csm_dap_words_mp(in.hi, n)); /* A after DAS: overflow, unscaled */
    agc_dp_words_t node = t->tmp[k];                                                 /* TMP(2k-1), B-1 rev          */

    /* DXCH TMP(2k-1): A,L = overflow, +0 go to TMP; DAS DAPn             */
    t->tmp[k].hi = ovf;
    t->tmp[k].lo = 0;
    csm_dap_words_das(&out, node);

    return (out);
}

/**********************************************************************
 * FWDFLTR, OPTVARK (TVCDAPS.agc pp. 972-973) -- the filter's present
 * output and the variable gain, with the sign change of the forward
 * loop: CMDTMP = -VARK x DAP3, rescaled from B+3 to B+1 ascrev by two
 * DDOUBLs (the output DACs have an inherent gain of B+1 ascrev).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void tvcd_fwdfltr(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    t->cmdtmp.hi   = 0;                                          /* CAF ZERO; TS DAP1..3 (rewritten), CMDTMP */
    t->delbrtmp.hi = 0;                                          /* TS DELBRTMP                              */
    t->dap1        = tvcd_cascade_out(t->errbtmp, t->n10[0], 0); /* N10  */
    t->dap2        = tvcd_cascade_out(t->dap1, t->n10[5], 2);    /* N20  */

    if ((agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u)
    { /* LM on */
        t->dap3 = tvcd_cascade_out(t->dap2, t->n10[10], 4); /* N30   */
    }
    else
    {
        t->dap3 = t->dap2; /* LM off: DCA DAP2; DXCH DAP3            */
    }

    /* CS DAP3 +1; MP VARK; TS CMDTMP +1; CS DAP3; MP VARK; DAS CMDTMP   */
    agc_dp_words_t c = csm_dap_words_smallmp(csm_dap_words_neg(t->dap3), t->vark); /* B+3 ascrev */

    t->cmdtmp = csm_dap_words_ddoubl(csm_dap_words_ddoubl(c)); /* B+1     */
}

/**********************************************************************
 * PRECOMP, 2CASFLTR, 3CASFLTR, one cascade (TVCDAPS.agc pp. 973-977) --
 * the cascade's filter nodes for the next pass, coefficients N10 + j
 * (N_1/2, N_2, D_1/2, D_2):
 *   TMP(k)   = TMP(k+1) + 2 (N_1/2 in - D_1/2 out)
 *   TMP(k+1) = (- D_2 out) + N_2 in
 *
 * ARGUMENTS:
 *   in   agc_dp_words_t  I  cascade input, B-1 rev
 *   out  agc_dp_words_t  I  cascade output, B-1 rev
 *   j    int                I  N10 index of N_1/2 (1, 6, 11)
 *   k    int                I  TMP index of the first node (0, 2, 4)
 **********************************************************************/
static void tvcd_cascade_pre(agc_dp_words_t in, agc_dp_words_t out, int j, int k)
{
    agc_csm_tvc_t *t    = agc_csm_dap_tvc();      /* TVC DAP erasables    */
    agc_dp_words_t nout = csm_dap_words_neg(out); /* CS DAPn, B-1 rev */

    /* TTMP1 = N_1/2 in, TTMP2 = -D_1/2 out; DAS TTMP1                    */
    agc_dp_words_t sum = csm_dap_words_smallmp(in, t->n10[j]); /* B-1  */

    csm_dap_words_das(&sum, csm_dap_words_smallmp(nout, t->n10[j + 2]));

    /* DDOUBL; DAS TMP(k+1); DXCH TMP(k+1); DXCH TMP(k)                   */
    agc_dp_words_t node = t->tmp[k + 1]; /* TMP(k+1), B-1 rev        */

    csm_dap_words_das(&node, csm_dap_words_ddoubl(sum));
    t->tmp[k] = node;

    /* TMP(k+1) = -D_2 out (TS TMP(k+1) +1 after TS of zero); DAS N_2 in  */
    t->tmp[k + 1] = csm_dap_words_smallmp(nout, t->n10[j + 3]);
    csm_dap_words_das(&t->tmp[k + 1], csm_dap_words_smallmp(in, t->n10[j + 1]));
}

/**********************************************************************
 * PRECOMP (TVCDAPS.agc pp. 973-977) -- precompute the nodes of each
 * cascade; the third cascade only when DAPDATR1 bit 13 (LM off) is
 * clear.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void tvcd_precomp(void)
{
    const agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables    */

    tvcd_cascade_pre(t->errbtmp, t->dap1, 1, 0); /* first cascade      */
    tvcd_cascade_pre(t->dap1, t->dap2, 6, 2);    /* 2CASFLTR           */

    if ((agc_erase_dapdatr1() & CSM_DAP_BIT13) == 0u)
    {
        tvcd_cascade_pre(t->dap2, t->dap3, 11, 4); /* 3CASFLTR         */
    }
}

/**********************************************************************
 * DELBARP / DELBARY (TVCDAPS.agc pp. 966, 969) -- the offset-tracker
 * filter DELBRTMP = E(-AT) DELBAR + (1 - E(-AT)) CMDTMP, the lower word
 * of DELBAR and the upper word of CMDTMP each multiplied alone.
 *
 * ARGUMENTS:
 *   delbar  agc_dp_words_t  I  DELPBAR or DELYBAR, 1 ascrev
 **********************************************************************/
static void tvcd_delbar(agc_dp_words_t delbar)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    _Static_assert(E_AT + ONE_M_EAT == 16384, "AGC constant");

    /* CAE DELxBAR +1; MP E(-AT); TS DELBRTMP +1 (upper word zeroed by
     * FWDFLTR); CAE DELxBAR; MP E(-AT); DAS DELBRTMP                      */
    t->delbrtmp.lo = csm_dap_mp_hi(delbar.lo, E_AT);
    csm_dap_words_das(&t->delbrtmp, csm_dap_words_mp(delbar.hi, E_AT));
    csm_dap_words_das(&t->delbrtmp, csm_dap_words_mp(t->cmdtmp.hi, ONE_M_EAT));
}

/**********************************************************************
 * PCOPY, PMISC (TVCDAPS.agc p. 966) -- the pitch copycycle, called by
 * the pitch DAP or (restart = true, PCOPY +1) by the TVC restart
 * package. TVCPHASE goes 0 -> 1 (copy in progress) -> 2 (completed).
 *
 * ARGUMENTS:
 *   restart  bool  I  true: entry at PCOPY +1 (TVCPHASE already 1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when
 *          TVCPHASE is not 1 inside the copycycle
 **********************************************************************/
agc_status_t csm_tvc_pcopy(bool restart)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    if (!restart)
    {
        t->tvcphase = (agc_sp_t) (t->tvcphase + 1); /* INCR TVCPHASE (1) */
    }

    if (!AGC_ASSERT(t->tvcphase == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < NODES; i++)
    {
        t->ptmp[i] = t->tmp[i]; /* DCA TMPn; DXCH PTMPn          */
    }

    agc_csm_dap_com()->ak[1] = t->errbtmp.hi;                /* TS AK1, B-1 rev      */
    t->perrb                 = t->errbtmp;                   /* rate-error integrator */
    t->pcmd                  = t->cmdtmp.hi;                 /* actuator command     */
    t->delpbar               = t->delbrtmp;                  /* offset tracker       */
    t->tvcphase              = (agc_sp_t) (t->tvcphase + 1); /* (2)   */

    if (!AGC_ASSERT(t->pcmd == t->cmdtmp.hi))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * YCOPY, YMISC (TVCDAPS.agc p. 969) -- the yaw copycycle (YCOPY +1 from
 * the restart package). TVCPHASE goes 2 -> 3 (copy in progress) -> 0.
 *
 * ARGUMENTS:
 *   restart  bool  I  true: entry at YCOPY +1 (TVCPHASE already 3)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG when
 *          TVCPHASE is not 3 inside the copycycle
 **********************************************************************/
agc_status_t csm_tvc_ycopy(bool restart)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    if (!restart)
    {
        t->tvcphase = (agc_sp_t) (t->tvcphase + 1); /* INCR TVCPHASE (3) */
    }

    if (!AGC_ASSERT(t->tvcphase == 3))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < NODES; i++)
    {
        t->ytmp[i] = t->tmp[i]; /* DCA TMPn; DXCH YTMPn          */
    }

    agc_csm_dap_com()->ak[2] = t->errbtmp.hi; /* TS AK2, B-1 rev      */
    t->yerrb                 = t->errbtmp;
    t->ycmd                  = t->cmdtmp.hi;
    t->delybar               = t->delbrtmp;
    t->tvcphase              = 0;             /* reset TVCPHASE                   */

    if (!AGC_ASSERT(t->ycmd == t->cmdtmp.hi))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * PINTEGRL / YINTEGRL (TVCDAPS.agc pp. 964, 968) -- the integral of the
 * body-axis rate error into ERRBTMP: xERRB + OMEGAxC - OMEGAxB, with the
 * body rate OMEGAYB or OMEGAZB (1/2 TVCDT revs) formed from the minus
 * CDU rates and SIN/COSCDUX, COSCDUZ (scale 2, upper words).
 *
 * ARGUMENTS:
 *   ax  tvcd_axis_t  I  TVCD_PITCH or TVCD_YAW
 **********************************************************************/
static void tvcd_integrate(tvcd_axis_t ax)
{
    agc_csm_tvc_t       *t    = agc_csm_dap_tvc();                  /* TVC DAP erasables */
    const agc_cdutrig_t *g    = &agc_guid_state()->trig;            /* QUICTRIG     */
    bool                 p    = (ax == TVCD_PITCH);                 /* pitch pass        */
    int                  a    = p ? 1 : 2;                          /* OMEGAYx / OMEGAZx */
    agc_sp_t             cosz = agc_dp_high(g->coscdu[AGC_TRIG_Z]); /* 2^1 */
    agc_sp_t             cosx = agc_dp_high(g->coscdu[AGC_TRIG_X]); /* 2^1 */
    agc_sp_t             sinx = agc_dp_high(g->sincdu[AGC_TRIG_X]); /* 2^1 */

    /* DCA xERRB; DXCH ERRBTMP; DCA OMEGAxC; DAS ERRBTMP (B-1 rev)        */
    t->errbtmp = p ? t->perrb : t->yerrb; /* pitch or yaw integrator   */
    csm_dap_words_das(&t->errbtmp, agc_dp_to_words(t->omegac[a]));

    /* pitch: CS COSCDUZ; MP COSCDUX; DDOUBL; MP MCDUYDOT; DDOUBL;
     *        CS MCDUZDOT; MP SINCDUX; DDOUBL; DAS OMEGAYB
     * yaw:   CAE COSCDUZ; MP SINCDUX; ...;   MP COSCDUX ... OMEGAZB      */
    agc_sp_t c1 = p ? (agc_sp_t) -cosz : cosz;                       /* ?: CS / CAE COSCDUZ, scale 2^1 */
    agc_sp_t c2 = p ? cosx : sinx;                                   /* ?: COSCDUX / SINCDUX, scale 2^1 */
    agc_sp_t k  = csm_dap_words_ddoubl(csm_dap_words_mp(c1, c2)).hi; /* MP; DDOUBL, scale 2^1 */

    t->omegab[a] = csm_dap_words_ddoubl(csm_dap_words_mp(k, t->mcduydot));                                              /* revs/s  */
    csm_dap_words_das(&t->omegab[a], csm_dap_words_ddoubl(csm_dap_words_mp((agc_sp_t) -t->mcduzdot, p ? sinx : cosx))); /* ?: axis */

    /* DCS OMEGAxB; DAS ERRBTMP                                           */
    csm_dap_words_das(&t->errbtmp, csm_dap_words_neg(t->omegab[a]));
}

/**********************************************************************
 * POFFSET .. POUT / YOFFSET .. YOUT (TVCDAPS.agc pp. 965, 968) -- add
 * the trim estimate, round and limit, and send the increment over the
 * last command to the optics error counter with its count release.
 *
 * ARGUMENTS:
 *   ax  tvcd_axis_t  I  TVCD_PITCH or TVCD_YAW
 **********************************************************************/
static void tvcd_output(tvcd_axis_t ax)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();  /* TVC DAP erasables          */
    bool           p = (ax == TVCD_PITCH); /* pitch pass                 */

    /* DCA xDELOFF; DAS CMDTMP: now scaled B+0 ascrev                     */
    csm_dap_words_das(&t->cmdtmp, p ? t->pdeloff : t->ydeloff); /* ?: pitch or yaw */
    tvcd_actlim();

    /* CS xCMD; AD CMDTMP: the increment, 85.41 arcsec/bit                */
    agc_sp_t inc = csm_dap_ads((agc_sp_t) -(p ? t->pcmd : t->ycmd), t->cmdtmp.hi); /* ?: increment, unscaled 85.41 arcsec/bit */

    if (p)
    {
        agc_erase_set_tvcpitch(csm_dap_ads(agc_erase_tvcpitch(), inc));
        agc_chan_or(AGC_CHAN_CHAN14, (uint16_t) AGC_CH014_DRIVECDUS); /* 11 */
    }
    else
    {
        agc_erase_set_tvcyaw(csm_dap_ads(agc_erase_tvcyaw(), inc));
        agc_chan_or(AGC_CHAN_CHAN14, (uint16_t) AGC_CH014_DRIVECDUT); /* 12 */
    }
}

/**********************************************************************
 * PINTEGRL .. PDAPEND / YINTEGRL .. YDAPEND (TVCDAPS.agc pp. 964-969)
 * -- the body of one DAP pass after the T5 set-up and stroke check.
 *
 * ARGUMENTS:
 *   ax  tvcd_axis_t  I  TVCD_PITCH or TVCD_YAW
 *
 * RETURNS: agc_status_t -- the copycycle's status (AGC_STATUS_OK)
 **********************************************************************/
static agc_status_t tvcd_dap(tvcd_axis_t ax)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();  /* TVC DAP erasables          */
    bool           p = (ax == TVCD_PITCH); /* pitch pass                 */

    if (!AGC_ASSERT((ax == TVCD_PITCH) || (ax == TVCD_YAW)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    tvcd_integrate(ax);
    tvcd_errorlim(); /* xERORLIM                                        */

    for (int i = 0; i < NODES; i += 2)
    { /* xFORWARD: TMP1,3,5  */
        t->tmp[i] = p ? t->ptmp[i] : t->ytmp[i]; /* ?: axis             */
    }

    tvcd_fwdfltr();
    tvcd_output(ax);

    for (int i = 1; i < NODES; i += 2)
    { /* xPRECOMP: TMP2,4,6  */
        t->tmp[i] = p ? t->ptmp[i] : t->ytmp[i]; /* ?: axis             */
    }

    tvcd_precomp();
    tvcd_delbar(p ? t->delpbar : t->delybar); /* DELBARx: ?: axis      */

    /* xCOPYCYC; xDAPEND: TCF RESUME                                      */
    return (p ? csm_tvc_pcopy(false) : csm_tvc_ycopy(false)); /* ?: axis */
}

/**********************************************************************
 * PITCHDAP / YAWDAP .. xSTROKER (TVCDAPS.agc pp. 963, 967) -- T5 call of
 * the other axis in TVCDT/2, HACK if a stroke test is starting or in
 * progress (CCS STROKER not +0), the CDU rates on the pitch pass, then
 * the DAP body.
 *
 * ARGUMENTS:
 *   ax  tvcd_axis_t  I  TVCD_PITCH or TVCD_YAW
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the first failure of the
 *          T5 set-up, HACK or the copycycle
 **********************************************************************/
static agc_status_t tvcd_entry(tvcd_axis_t ax)
{
    agc_csm_tvc_t *t    = agc_csm_dap_tvc(); /* TVC DAP erasables       */
    agc_task_id_t  next = AGC_TASK_PITCHDAP; /* T5LOC: the other axis   */

    if (!AGC_ASSERT((ax == TVCD_PITCH) || (ax == TVCD_YAW)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (ax == TVCD_PITCH)
    {
        next = AGC_TASK_YAWDAP; /* CAF YAWT5; TS T5LOC                  */
    }

    agc_status_t st = csm_dap_set_t5(next, (uint16_t) t->t5tvcdt); /* T5 status */

    if ((st == AGC_STATUS_OK) && (t->stroker != 0))
    { /* (STRKFLG)                                */
        st = csm_tvc_hack();
    }

    if (ax == TVCD_PITCH)
    {
        tvcd_pcdudots(); /* the yaw DAP uses the pitch pass's rates     */
    }

    agc_status_t s2 = tvcd_dap(ax); /* status of the DAP body          */

    return ((st != AGC_STATUS_OK) ? st : s2); /* first failure          */
}

/**********************************************************************
 * PITCHDAP (TVCDAPS.agc p. 963) -- T5 entry, normal or via DAPINIT.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a DAP
 *          state assertion
 **********************************************************************/
agc_status_t agc_task_pitchdap(void)
{
    if (!AGC_ASSERT(agc_csm_dap_tvc()->t5tvcdt > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = tvcd_entry(TVCD_PITCH); /* one pitch pass        */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (st);
}

/**********************************************************************
 * YAWDAP (TVCDAPS.agc p. 967) -- T5 entry.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a DAP
 *          state assertion
 **********************************************************************/
agc_status_t agc_task_yawdap(void)
{
    if (!AGC_ASSERT(agc_csm_dap_tvc()->t5tvcdt > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = tvcd_entry(TVCD_YAW); /* one yaw pass            */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (st);
}

/**********************************************************************
 * DAPINIT (TVCDAPS.agc pp. 969-970) -- T5 entry from TVCINIT4: PITCHDAP
 * in TVCDT (TIME5 = -1 + T5TVCDT + NEGMAX + T5TVCDT, where T5TVCDT =
 * POSMAX - TVCDT/2 + 1) and the past CDUs for the differentiators.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG when the
 *          TIME5 word is out of range
 **********************************************************************/
agc_status_t agc_task_dapinit(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();                                           /* TVC DAP erasables          */
    int            w = -1 + (int) t->t5tvcdt + (int) AGC_SP_MIN + (int) t->t5tvcdt; /* TIME5 word, 10 ms counts    */

    if (!AGC_ASSERT((w > 0) && (w <= AGC_SP_MAX)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = csm_dap_set_t5(AGC_TASK_PITCHDAP, (uint16_t) w); /* T5 status */

    t->pcduypst = agc_erase_cduy();
    t->pcduzpst = agc_erase_cduz();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (st); /* NOQRSM                                              */
}
