/**********************************************************************
 * FILE NAME: dapdata.c
 *
 * PURPOSE:
 *   DAP data load decoding and RCS DAP start-up: SETMINDB, SETMAXDB,
 *   S41.2 (decode DAPDATR1/2), S40.14 (RCS gains), RCSUP, RCSDAPON.
 *   These live with P40-P47 in the listing but read and write only the
 *   autopilot's EBANK6 erasables; S41.2 and S40.14 are called by R03
 *   (DONOUN46/47, sys-misc) and by FRESHDAP; RCSDAPON by DAPFIG (V46),
 *   TVCDAPOF (P40) and R00TOP00 (exec).
 *
 * AGC SOURCE:
 *   Comanche055 P40-P47.agc pp. 687, 731-733, 735-736
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   DAPDATR1      uint16_t  I/O  bits           N46 R1 (below)
 *   DAPDATR2      uint16_t  I    bits           N46 R2 (below)
 *   ADB           agc_sp_t  O    180 deg        deadband
 *   THETADX..Z    uint16_t  O    CDU counts     hold references
 *   RATEINDX,     agc_sp_t  O    unscaled       S41.2 outputs
 *   XTRANS, ACORBD, RACFAIL, RBDFAIL
 *   ATTCHFLG, NJETSFLG  flags  O  -             docked, 2-jet ullage
 *   IXX, IAVG     agc_sp_t  I    2^20 kg m^2    inertias
 *   J/M, KMJ      agc_sp_t  O    see dap.h      RCS gains
 *   T5PHASE, T5LOC, TIME5, RCSFLAGS, FLAGWRD6 (DAPBIT1/2)  O  RCSDAPON
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   csm_dap_mp_hi, _dv    MP, DV (mysubs.c)
 *   csm_dap_set_t5        T5LOC, TIME5 (mysubs.c)
 *   agc_flag_*            ATTCHFLG, NJETSFLG, DAPBIT1, DAPBIT2
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG for a negative inertia or a
 *   TIME5 word outside 1 .. 037777.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   SETMINDB, SETMAXDB and RCSDAPON are called under INHINT.
 *
 * NOTES:
 *   DAPDATR1 (N46 R1): bits 15,14,13 configuration (001 CSM, 010
 *   CSM/LM, 110 CSM/LM ascent only), bit 10 quad AC / bit 7 quad BD
 *   X-translation, bit 4 deadband (0 = 0.5 deg, 1 = 5 deg), bits 2,1
 *   rate (0.05, 0.2, 0.5, 2 deg/s). DAPDATR2 (N46 R2): bit 13 AC roll,
 *   bits 10,4 quads A,C good, bits 7,1 quads B,D good.
 **********************************************************************/
#include "../internal.h"

#define MINDB      ((agc_sp_t) 46)     /* 00056 DEC 46: 0.5 deg, scale 180 deg */
#define DEC409     ((agc_sp_t) 409)    /* 00631: MAXDB - MINDB, scale 180 deg  */
#define MAXDB      ((agc_sp_t) 455)    /* 00707 DEC 455: 5 deg, scale 180 deg  */
#define CONTONE    ((agc_sp_t) 10847)  /* 25137 DEC .662034 = 2PI/M, scale 1   */
#define CONTTWO    ((agc_dp_t) 316754) /* 00023 12522 2DEC .00118, scale 1     */
#define T5_06SEC   ((uint16_t) 037704) /* 0.6SECT5: 0.6 s lets TVCEXEC die     */
#define CONFIGBITS ((uint16_t) 030000) /* PRIO30: DAPDATR1 bits 14,13          */

/**********************************************************************
 * SETMINDB (P40-P47.agc p. 687) -- minimum deadband (ADB = MINDB,
 * DAPDATR1 BIT4 clear) and attitude hold about the present CDUs
 * (THETADX..Z).
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_csm_dap_setmindb(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* THETAD, ADB                */

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        r->thetad[i] = csm_dap_cdu(i); /* CA CDUX; DCA CDUY; THETADX..Z */
    }

    r->adb = MINDB; /* CA MINDB; TS ADB                                  */
    agc_erase_set_dapdatr1((uint16_t) (agc_erase_dapdatr1() & ~(unsigned int) CSM_DAP_BIT4));
}

/**********************************************************************
 * SETMAXDB (P40-P47.agc p. 687) -- maximum deadband (ADB = MAXDB,
 * DAPDATR1 BIT4 set).
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_csm_dap_setmaxdb(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* ADB                        */

    r->adb = MAXDB; /* CA MAXDB; TS ADB                                  */
    agc_erase_set_dapdatr1((uint16_t) (agc_erase_dapdatr1() | CSM_DAP_BIT4));
}

/**********************************************************************
 * S41.2 (P40-P47.agc pp. 731-733) -- decode DAPDATR1, DAPDATR2: the
 * rate index, the docked flag ATTCHFLG (bits 14,13 = 10), the deadband,
 * the X-translation quads (XTRANS +1 BD, -1 AC, 0 both) and NJETSFLG
 * (2-jet ullage), the roll quads (ACORBD - AC, + BD) and the quad
 * failures (RACFAIL +1 A failed, -1 C failed; RBDFAIL +1 B, -1 D).
 * Called by DONOUN46 (V48) or directly by FRESHDAP.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_csm_dap_decode_dap_data(void)
{
    agc_csm_rcs_t *r  = agc_csm_dap_rcs();    /* RCS DAP erasables    */
    uint16_t       d1 = agc_erase_dapdatr1(); /* N46 R1               */
    uint16_t       d2 = agc_erase_dapdatr2(); /* N46 R2               */

    r->ratindx = (agc_sp_t) (2u * (d1 & 3u));                                     /* MASK THREE; AD A; TS RATEINDX */
    agc_inhint();
    agc_flag_put(AGC_FLAG_ATTCHFLG, (d1 & CONFIGBITS) == CSM_DAP_BIT14);          /* MASK PRIO30; AD -BIT14; BZF TOGETHER */
    agc_relint();
    r->adb     = (agc_sp_t) ((((d1 & CSM_DAP_BIT4) != 0u) ? DEC409 : 0) + MINDB); /* ?: wide or narrow DB */
    r->xtrans  = ((d1 & CSM_DAP_BIT7) != 0u) ? 1 : 0;                             /* ?: quad BD          */

    if ((d1 & CSM_DAP_BIT10) != 0u)
    {
        r->xtrans = (agc_sp_t) (r->xtrans - 1); /* quad AC: CS ONE; ADS XTRANS */
    }

    agc_inhint();
    agc_flag_put(AGC_FLAG_NJETSFLG, r->xtrans != 0); /* 2-jet ullage (AC or BD) */
    agc_relint();

    /* MASK BIT13; BZMF: COM of -1 (+1, B-D) or of BIT13 (minus, A-C)     */
    r->acorbd = ((d2 & CSM_DAP_BIT13) != 0u) ? (agc_sp_t) -(agc_sp_t) CSM_DAP_BIT13 : 1; /* ?: */

    if ((d2 & CSM_DAP_BIT10) == 0u)
    {
        r->racfail = 1; /* A failed                                      */
    }
    else
    {
        r->racfail = ((d2 & CSM_DAP_BIT4) == 0u) ? -1 : 0; /* ?: C failed */
    }

    if ((d2 & CSM_DAP_BIT7) == 0u)
    { /* BDFAIL */
        r->rbdfail = 1; /* B failed                                      */
    }
    else
    {
        r->rbdfail = ((d2 & CSM_DAP_BIT1) == 0u) ? -1 : 0; /* ?: D failed */
    }

    return (AGC_STATUS_OK);
} /* agc_csm_dap_decode_dap_data */

/**********************************************************************
 * S40.14 (P40-P47.agc p. 735) -- RCS gains from IXX, IAVG:
 * J/M = IXX (2PI/M), J/M1 = J/M2 = IAVG (2PI/M);
 * KMJ = .00118 / IXX, KMJ1 = KMJ2 = .00118 / IAVG. The AGC does not
 * check the inertias (MASSPROP forms them from the crew's V48 masses):
 * MP and DV are signed, and DV limits an overflow, so the gains are
 * always formed.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_csm_dap_rcs_gains(void)
{
    agc_csm_rcs_t          *r = agc_csm_dap_rcs(); /* J/M, KMJ           */
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* IXX, IAVG          */

    r->jm[0]  = csm_dap_mp_hi(c->ixx, CONTONE);  /* CAE IXX; MP CONTONE; TS J/M  */
    r->jm[1]  = csm_dap_mp_hi(c->iavg, CONTONE); /* TS J/M1                      */
    r->jm[2]  = r->jm[1];                        /* TS J/M2                      */
    r->kmj[0] = csm_dap_dv(CONTTWO, c->ixx);     /* DCA CONTTWO; DV IXX; TS KMJ  */
    r->kmj[1] = csm_dap_dv(CONTTWO, c->iavg);    /* DV IAVG; TS KMJ1             */
    r->kmj[2] = r->kmj[1];                       /* TS KMJ2                      */
}

/**********************************************************************
 * RCSDAPON +1 (P40-P47.agc p. 736) -- entry from R00TOP00 with the
 * TIME5 word in A: TIME5 = T5PHASE = A (+: FRESHDAP), RCSFLAGS BIT3 to
 * reinitialise the FDAI error display (in case the S/C control switch
 * is in SCS, not GNC), T5LOC = RCSATT, FLAGWRD6 bits 15,14 = 01 (T5
 * takeover by the RCS DAP: kills TVCEXEC and ROLLDAP).
 *
 * ARGUMENTS:
 *   time5_word  uint16_t  I  TIME5, 10 ms counts to overflow, > 0
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (always: csm_dap_set_t5() cannot
 *          fail)
 **********************************************************************/
agc_status_t agc_csm_dap_rcsdapon_t5(uint16_t time5_word)
{
    agc_csm_rcs_t    *r = agc_csm_dap_rcs(); /* T5PHASE                 */
    agc_csm_dapcom_t *c = agc_csm_dap_com(); /* RCSFLAGS                */

    r->t5phase      = AGC_CSM_T5P_INIT;                                /* TS T5PHASE (+)          */
    c->rcsflags     = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NEEDLE1); /* CS; MASK BIT3; ADS      */
    agc_status_t st = csm_dap_set_t5(AGC_TASK_RCSATT, time5_word);     /* DCA RCSADDR; DXCH T5LOC */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_clear(AGC_FLAG_DAPBIT1); /* CS OCT60000; MASK FLAGWRD6; AD BIT14 */
    agc_flag_set(AGC_FLAG_DAPBIT2);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * RCSDAPON (P40-P47.agc p. 736) -- RCSDAPON +1 with 0.6 s, which allows
 * TVCEXEC and ROLLDAP to die.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the RCSDAPON +1 status
 **********************************************************************/
agc_status_t agc_csm_dap_rcsdapon(void)
{
    return (agc_csm_dap_rcsdapon_t5(T5_06SEC)); /* CAF 0.6SECT5           */
}

/**********************************************************************
 * RCSUP (P40-P47.agc p. 736) -- T5 task (RCSCADR) that activates the
 * RCS DAP: TCR RCSDAPON; TCF RESUME.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the RCSDAPON status
 **********************************************************************/
agc_status_t agc_task_rcsup(void)
{
    return (agc_csm_dap_rcsdapon());
}
