/**********************************************************************
 * FILE NAME: massprop.c
 *
 * PURPOSE:
 *   CSM (and docked LM) mass properties for the DAPs (programmer
 *   Melanson; Engel, Schlundt). MASSPROP operates in two modes: (1) if
 *   LM mass or configuration have been updated, the breakpoint values and
 *   the derivatives of the outputs with respect to CSM mass are computed
 *   first (MASSPROP); (2) otherwise the outputs are computed from the
 *   previous breakpoints (FIXCW):
 *     IXX      = VARST0 + (CSMMASS + NEGBPW) VARST5           B+20 kg m^2
 *     IAVG     = VARST1 + (CSMMASS + NEGBPW) VARST(4 or 6)    B+20 kg m^2
 *     IAVG/TLX = VARST2 + (CSMMASS + NEGBPW) VARST(3 or 7)    B+2 s^2
 *   with VARSTi = INTVALUEi + LEMMASS SLOPEVALi when docked, NOLEMVALi
 *   otherwise. Labels: MASSPROP, LEMTEST, LEMNO, LEMYES, STOINST, DXTEST,
 *   FIXCW, PEGGY, BOKKEP2, ENDMASSP.
 *
 * AGC SOURCE:
 *   Comanche055 TVCMASSPROP.agc pp. 951-955
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAPDATR1      uint16_t  I    bits           15 descent stage off,
 *                                               14 LM on, 13 LM off
 *   LEMMASS       agc_sp_t  I    B+16 kg        LM mass
 *   CSMMASS       agc_sp_t  I    B+16 kg        CSM mass
 *   VARST0..9     agc_sp_t  I/O  per table      breakpoints, slopes
 *   IXX, IAVG     agc_sp_t  O    B+20 kg m^2    inertias
 *   IAVG/TLX      agc_sp_t  O    B+2 s^2        inertia / lever arm
 *   WEIGHT/G      agc_sp_t  O    B+16 kg        vehicle mass
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   csm_dap_ads, _mp_hi,  AD/ADS, MP, DDOUBL, DAS (mysubs.c)
 *   _ddoubl, _words_das
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called by DONOUN47 (job, R03) and TVCEXECUTIVE (task). Data from
 *   the CSM/LM Spacecraft Operational Data Book vol. 3 (SNA-8-D-027,
 *   March 1968, amendment 1).
 *
 * NOTES:
 *   DEVIATION (DXTEST): the AGC's DXCH VARST0 +8D leaves A, L (the CCS
 *   residue of DAPDATR1 and the last L) in VARST8, VARST9; nothing reads
 *   them before the next MASSPROP rewrites them, so they keep the values
 *   just computed here.
 **********************************************************************/
#include "../internal.h"

#define VARST_N AGC_CSM_VARST_WORDS /* VARST0 .. VARST9                */

#define NEGBPW  ((agc_sp_t) -3851) /* 70364 DEC -15402.17 B-16 kg: breakpoint weight 33956 lb */
#define DXITFIX ((agc_sp_t) -1263) /* 75420 DEC -1.88275 E-5 B+12        */

/* NOLEMVAL (p. 954), LM not docked. MASSPROP indexes ten words but the
 * table has eight; words 8, 9 are INTVALUE 0, 1 (unused for this case). */
static const agc_sp_t nolemval[VARST_N] = {
    398,      /* DEC 25445 B-20        IXX breakpoint                      */
    1366,     /* DEC 87450 B-20        IAVG breakpoint                     */
    1258,     /* DEC .30715 B-2        IAVG/TLX breakpoint                 */
    825,      /* DEC 1.22877 E-5 B+12  IAVG/TLX slope, CSMMASS > breakpoint */
    412,      /* DEC 1.6096 B-6        IAVG slope, CSMMASS > breakpoint     */
    394,      /* DEC 1.54 B-6          IXX slope                           */
    1990,     /* DEC 7.77177 B-6       IAVG slope, CSMMASS < breakpoint     */
    2325,     /* DEC 3.46458 E-5 B+12  IAVG/TLX slope, CSMMASS < breakpoint */
    420, 1992 /* (INTVALUE 0, 1: overrun)           */
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

/**********************************************************************
 * MASSPROP, LEMTEST, LEMNO, LEMYES, STOINST, DXTEST (TVCMASSPROP.agc
 * pp. 953-954) -- the ten breakpoint and slope values VARST9 .. VARST0:
 * docked (DAPDATR1 bit 13 = 0) INTVALUE + 2 LEMMASS SLOPEVAL x 2, else
 * NOLEMVAL; with the LM descent stage off (DAPDATR1 negative) VARST1,2
 * += VARST8,9 (DAS) and VARST7 += DXITFIX; then FIXCW.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_csm_dap_massprop(void)
{
    agc_csm_tvc_t *t   = agc_csm_dap_tvc();                                         /* VARST            */
    uint16_t       d1  = agc_erase_dapdatr1();                                      /* DAPDATR1         */
    bool           lem = (d1 & CSM_DAP_BIT13) == 0u;                                /* LEM attached     */
    agc_sp_t       lm2 = csm_dap_ads(agc_erase_lemmass(), agc_erase_lemmass());     /* DOUBLE, scale B+15 kg */

    for (int phi = VARST_N - 1; phi >= 0; phi--)
    { /* ten passes, PHI333 */
        if (lem)
        { /* LEMYES: MP SLOPEVAL; DDOUBL; AD INTVALUE */
            agc_dp_t p = csm_dap_ddoubl(agc_sp_mul(lm2, slopeval[phi])); /* scale of VARSTphi */

            t->varst[phi] = csm_dap_ads(agc_dp_high(p), intvalue[phi]);
        }
        else
        {
            t->varst[phi] = nolemval[phi]; /* LEMNO: CAF NOLEMVAL     */
        }
    }

    if ((d1 & CSM_DAP_BIT15) != 0u)
    { /* DXTEST: CCS DAPDATR1 negative: LM descent stage off */
        agc_dp_words_t k = {t->varst[1], t->varst[2]}; /* VARST0 +1, +2 as a pair */

        csm_dap_words_das(&k, (agc_dp_words_t){t->varst[8], t->varst[9]}); /* DXCH VARST0 +8D; DAS VARST0 +1 */
        t->varst[1] = k.hi;
        t->varst[2] = k.lo;
        t->varst[7] = csm_dap_ads(t->varst[7], DXITFIX); /* CA DXITFIX; ADS VARST0 +7 */
    }

    agc_csm_dap_fixcw();
}

/**********************************************************************
 * FIXCW, PEGGY, BOKKEP2, ENDMASSP (TVCMASSPROP.agc pp. 954-955) -- the
 * outputs IXX, IAVG, IAVG/TLX = breakpoint + slope x 2 (CSMMASS +
 * NEGBPW), the slopes VARST5,4,3 above the breakpoint weight or
 * VARST5,6,7 below it; WEIGHT/G = CSMMASS (+ LEMMASS when DAPDATR1
 * bit 14 is set).
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_csm_dap_fixcw(void)
{
    const agc_csm_tvc_t *t = agc_csm_dap_tvc();   /* VARST                */
    agc_csm_dapcom_t    *c = agc_csm_dap_com();   /* outputs              */
    agc_sp_t             m = agc_erase_csmmass(); /* CSMMASS, scale B+16 kg */
    agc_sp_t             out[3];                  /* IXX, IAVG (scale B+20 kg m^2), IAVG/TLX (scale B+2 s^2) */

    agc_sp_t d       = csm_dap_ads(m, NEGBPW);  /* CAE CSMMASS; AD NEGBPW, scale B+16 kg */
    agc_sp_t temp333 = csm_dap_ads(d, d);       /* DOUBLE; TS TEMP333, scale B+15 kg */
    int      phi     = (temp333 > 0) ? -2 : 2; /* ?: BZMF PEGGY; CAF NEG2: slope locations */

    for (int psi = 2; psi >= 0; psi--)
    { /* PSI333 = 2, 1, 0 */
        agc_sp_t s = csm_dap_mp_hi(t->varst[5 + phi], temp333); /* INDEX PHI333; CAE VARST5; MP TEMP333, output scale / 2 */

        out[psi] = csm_dap_ads(csm_dap_ads(s, s), t->varst[psi]); /* DOUBLE; AD VARST0; TS IXX */

        if (phi > 0)
        {
            phi--; /* DIM PHI333: magnitude down by one                */
        }
        else if (phi < 0)
        {
            phi++;
        }
        else
        {
            /* DIM of zero leaves zero */
        }
    }

    c->ixx      = out[0];
    c->iavg     = out[1];
    c->iavg_tlx = out[2];
    agc_sp_t lm = ((agc_erase_dapdatr1() & CSM_DAP_BIT14) != 0u) ? agc_erase_lemmass() : 0; /* ?: MASK BIT14; CCS; CA LEMMASS, scale B+16 kg */

    c->weight_g = csm_dap_ads(lm, m); /* AD CSMMASS; TS WEIGHT/G         */
}
