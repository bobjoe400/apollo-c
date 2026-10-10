/*
 * massprop.c -- CSM (and docked LM) mass properties for the DAPs.
 * TVCMASSPROP.agc (Comanche055 pp. 951-955): MASSPROP, LEMTEST, STOINST,
 * DXTEST, FIXCW, PEGGY, BOKKEP2. Programmer Melanson (Engel, Schlundt).
 *
 * MASSPROP operates in two modes: (1) if LM mass or configuration have
 * been updated, the breakpoint values and the derivatives of the outputs
 * with respect to CSM mass are computed first (MASSPROP); (2) otherwise
 * the outputs are computed from the previous breakpoints (FIXCW).
 *
 *   IXX      = VARST0 + (CSMMASS + NEGBPW) VARST5             B+20 kg m^2
 *   IAVG     = VARST1 + (CSMMASS + NEGBPW) VARST(4 or 6)      B+20 kg m^2
 *   IAVG/TLX = VARST2 + (CSMMASS + NEGBPW) VARST(3 or 7)      B+2 s^2
 * with VARSTi = INTVALUEi + LEMMASS SLOPEVALi when docked, NOLEMVALi
 * otherwise. Data from the CSM/LM Spacecraft Operational Data Book vol. 3
 * (SNA-8-D-027, March 1968, amendment 1). Masses are kg, scale B+16.
 * Called by DONOUN47 (job, R03) and TVCEXECUTIVE (task).
 */
#include "../internal.h"

#define VARST_N AGC_CSM_VARST_WORDS
#define NEGBPW \
    ((agc_sp_t) -3851)              /* 70364 DEC -15402.17 B-16: breakpoint
                                         weight 33956 lb (SPS 10000 lb)   */
#define DXITFIX  ((agc_sp_t) -1263) /* 75420 DEC -1.88275 E-5 B+12      */
#define W_ONE_SP 16384              /* 1.0 in SP units                   */

/* NOLEMVAL (p. 954), LM not docked. MASSPROP indexes ten words but the
 * table has eight; words 8, 9 are INTVALUE 0, 1 (unused for this case). */
static const agc_sp_t nolemval[VARST_N] = {
    398,       /* DEC 25445 B-20        IXX breakpoint                      */
    1366,      /* DEC 87450 B-20        IAVG breakpoint                     */
    1258,      /* DEC .30715 B-2        IAVG/TLX breakpoint                 */
    825,       /* DEC 1.22877 E-5 B+12  IAVG/TLX slope, CSMMASS > breakpoint */
    412,       /* DEC 1.6096 B-6        IAVG slope, CSMMASS > breakpoint     */
    394,       /* DEC 1.54 B-6          IXX slope                           */
    1990,      /* DEC 7.77177 B-6       IAVG slope, CSMMASS < breakpoint     */
    2325,      /* DEC 3.46458 E-5 B+12  IAVG/TLX slope, CSMMASS < breakpoint */
    420,  1992 /* (INTVALUE 0, 1: overrun)           */
};
/* INTVALUE (p. 954), LM docked: intercepts of VARST0..9.                */
static const agc_sp_t intvalue[VARST_N] = {
    420,  /* DEC 26850 B-20       */
    1992, /* DEC 127518 B-20      */
    2214, /* DEC .54059 B-2       */
    1033, /* DEC .153964 E-4 B+12 */
    -190, /* DEC -.742923 B-6     */
    394,  /* DEC 1.5398 B-6       */
    2478, /* DEC 9.68 B-6         */
    4346, /* DEC .647625 E-4 B+12 */
    -425, /* DEC -27228 B-20      IAVG decrement, LM descent stage off  */
    -846  /* DEC -.206476 B-2     IAVG/TLX decrement, descent stage off */
};
/* SLOPEVAL (pp. 954-955), LM docked: d VARSTi / d LEMMASS.               */
static const agc_sp_t slopeval[VARST_N] = {
    503,  /* DEC 1.96307 B-6      */
    7060, /* DEC 27.5774 B-6      */
    1580, /* DEC 2.3548 E-5 B+12  */
    2394, /* DEC 2.1777 E-9 B+26  */
    4379, /* DEC 1.044 E-3 B+8    */
    0,    /* DEC 0                */
    9272, /* DEC 2.21068 E-3 B+8  */
    1668, /* DEC 1.5166 E-9 B+26  */
    -329, /* DEC -1.284 B-6       */
    1342  /* DEC 2 E-5 B+12       */
};

/* DAS VARST0 +1 with A,L = VARST8, VARST9 (DXTEST): two separate SP
 * quantities added as a word pair, the low-word carry going up.          */
static void das_words(agc_sp_t *hi, agc_sp_t *lo, agc_sp_t ahi, agc_sp_t alo)
{
    int l     = (int) *lo + (int) alo;
    int carry = (l > AGC_SP_MAX) ? 1 : ((l < AGC_SP_MIN) ? -1 : 0);
    l -= carry * W_ONE_SP;
    (void) AGC_ASSERT((l >= AGC_SP_MIN) && (l <= AGC_SP_MAX));
    *lo = (agc_sp_t) l;
    *hi = csm_dap_ads(csm_dap_ads(*hi, ahi), (agc_sp_t) carry);
    (void) AGC_ASSERT(*hi >= AGC_SP_MIN);
}

/* MASSPROP, LEMTEST, LEMNO, LEMYES, STOINST, DXTEST (pp. 953-954).       */
agc_status_t agc_csm_dap_massprop(void)
{
    agc_csm_tvc_t *t   = agc_csm_dap_tvc();
    uint16_t       d1  = agc_erase_dapdatr1();
    bool           lem = (d1 & CSM_DAP_BIT13) == 0u; /* LEM attached      */
    agc_sp_t       lm2 = csm_dap_ads(agc_erase_lemmass(), agc_erase_lemmass());
    if (!AGC_ASSERT(agc_erase_lemmass() >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    for (int phi = VARST_N - 1; phi >= 0; phi--)
    { /* ten passes, PHI333 */
        if (lem)
        { /* LEMYES            */
            agc_dp_t p    = csm_dap_ddoubl(agc_sp_mul(lm2, slopeval[phi]));
            t->varst[phi] = csm_dap_ads(agc_dp_high(p), intvalue[phi]);
        }
        else
        {
            t->varst[phi] = nolemval[phi]; /* LEMNO             */
        }
    }
    if ((d1 & CSM_DAP_BIT15) != 0u)
    { /* LM descent stage off */
        das_words(&t->varst[1], &t->varst[2], t->varst[8], t->varst[9]);
        t->varst[7] = csm_dap_ads(t->varst[7], DXITFIX);
    }
    return agc_csm_dap_fixcw();
}

/* FIXCW, PEGGY, BOKKEP2, ENDMASSP (pp. 954-955): the outputs IXX, IAVG,
 * IAVG/TLX from the breakpoints, and WEIGHT/G (kg, B+16).               */
agc_status_t agc_csm_dap_fixcw(void)
{
    const agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_csm_dapcom_t    *c = agc_csm_dap_com();
    agc_sp_t             m = agc_erase_csmmass();
    if (!AGC_ASSERT(m >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_sp_t d       = csm_dap_ads(m, NEGBPW);
    agc_sp_t temp333 = csm_dap_ads(d, d);      /* DOUBLE: delta CSM wt */
    int      phi     = (temp333 > 0) ? -2 : 2; /* slope locations    */
    agc_sp_t out[3];
    for (int psi = 2; psi >= 0; psi--)
    { /* PSI333 = 2, 1, 0   */
        agc_sp_t s = csm_dap_mp_hi(t->varst[5 + phi], temp333);
        out[psi]   = csm_dap_ads(csm_dap_ads(s, s), t->varst[psi]);
        phi += (phi > 0) ? -1 : ((phi < 0) ? 1 : 0); /* DIM PHI333         */
    }
    c->ixx      = out[0];
    c->iavg     = out[1];
    c->iavg_tlx = out[2];
    agc_sp_t lm = ((agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u) ?
                      agc_erase_lemmass() :
                      0;
    c->weight_g = csm_dap_ads(lm, m); /* WEIGHT/G           */
    (void) AGC_ASSERT(phi == 0);
    return AGC_STATUS_OK; /* ENDMASSP: TC Q     */
}
