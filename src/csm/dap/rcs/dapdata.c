/*
 * dapdata.c -- DAP data load decoding and RCS DAP start-up.
 * P40-P47.agc (Comanche055): SETMINDB, SETMAXDB (p. 687), S41.2
 * (pp. 731-733), S40.14 (p. 735), RCSUP and RCSDAPON (p. 736).
 *
 * These live with P40-P47 in the listing but read and write only the
 * autopilot's EBANK6 erasables; S41.2 and S40.14 are called by R03
 * (DONOUN46/47, sys-misc) and by FRESHDAP; RCSDAPON by DAPFIG (V46),
 * TVCDAPOF (P40) and R00TOP00 (exec).
 *
 * DAPDATR1 (N46 R1): bits 15,14,13 configuration (001 CSM, 010 CSM/LM,
 * 110 CSM/LM ascent only), bit 10 quad AC / bit 7 quad BD X-translation,
 * bit 4 deadband (0 = 0.5 deg, 1 = 5 deg), bits 2,1 rate (0.05, 0.2, 0.5,
 * 2 deg/s). DAPDATR2 (N46 R2): bit 13 AC roll, bits 10,4 quads A,C good,
 * bits 7,1 quads B,D good.
 */
#include "../internal.h"

#define MINDB      ((agc_sp_t) 46)     /* 00056 DEC 46: 0.5 deg, 180 deg    */
#define MAXDB      ((agc_sp_t) 455)    /* 00707 DEC 455: 5 deg              */
#define DEC409     ((agc_sp_t) 409)    /* 00631                             */
#define CONTONE    ((agc_sp_t) 10847)  /* 25137 DEC .662034 = 2PI/M         */
#define CONTTWO    ((agc_dp_t) 316754) /* 00023 12522 2DEC .00118           */
#define T5_06SEC   ((uint16_t) 037704) /* 0.6SECT5: 0.6 s lets TVCEXEC die  */
#define CONFIGBITS ((uint16_t) 030000) /* PRIO30: DAPDATR1 bits 14,13       */

/* SETMINDB (p. 687): minimum deadband, attitude hold about the present
 * CDUs. Should be called under interrupt inhibit.                        */
agc_status_t agc_csm_dap_setmindb(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        r->thetad[i] = csm_dap_cdu(i); /* THETADX, Y, Z          */
    }

    r->adb = MINDB;
    agc_erase_set_dapdatr1(
        (uint16_t) (agc_erase_dapdatr1() & ~(unsigned) CSM_DAP_BIT4));

    if (!AGC_ASSERT((agc_erase_dapdatr1() & CSM_DAP_BIT4) == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* SETMAXDB (p. 687): maximum deadband. Under interrupt inhibit.          */
agc_status_t agc_csm_dap_setmaxdb(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();

    r->adb = MAXDB;
    agc_erase_set_dapdatr1((uint16_t) (agc_erase_dapdatr1() | CSM_DAP_BIT4));

    if (!AGC_ASSERT(r->adb == MAXDB))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT((agc_erase_dapdatr1() & CSM_DAP_BIT4) != 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* S41.2 (pp. 731-732), part 1: rates, docked flag, deadband, quad
 * management for X-translation and the ullage jet flag.                  */
static agc_status_t s41_2_datr1(void)
{
    agc_csm_rcs_t *r  = agc_csm_dap_rcs();
    uint16_t       d1 = agc_erase_dapdatr1();

    r->ratindx = (agc_sp_t) (2u * (d1 & 3u)); /* MASK THREE; AD A     */
    agc_inhint();
    /* LEM attached: TOGETHER */
    agc_flag_put(AGC_FLAG_ATTCHFLG, (d1 & CONFIGBITS) == CSM_DAP_BIT14);
    agc_relint();
    r->adb    = (agc_sp_t) ((((d1 & CSM_DAP_BIT4) != 0u) ? DEC409 : 0) + MINDB);
    r->xtrans = ((d1 & CSM_DAP_BIT7) != 0u) ? 1 : 0; /* quad BD     */

    if ((d1 & CSM_DAP_BIT10) != 0u)
    {
        r->xtrans = (agc_sp_t) (r->xtrans - 1); /* quad AC     */
    }

    agc_inhint();
    agc_flag_put(AGC_FLAG_NJETSFLG, r->xtrans != 0);
    agc_relint();

    if (!AGC_ASSERT((r->adb == MINDB) || (r->adb == MAXDB)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* S41.2 (pp. 732-733), part 2: AC/BD roll and the quad failure flags.   */
static agc_status_t s41_2_datr2(void)
{
    agc_csm_rcs_t *r  = agc_csm_dap_rcs();
    uint16_t       d2 = agc_erase_dapdatr2();

    /* minus for A-C (COM of BIT13), plus for B-D                         */
    r->acorbd = ((d2 & CSM_DAP_BIT13) != 0u) ?
                (agc_sp_t) -(agc_sp_t) CSM_DAP_BIT13 :
                1;

    if ((d2 & CSM_DAP_BIT10) == 0u)
    {
        r->racfail = 1; /* A failed             */
    }
    else
    {
        r->racfail = ((d2 & CSM_DAP_BIT4) == 0u) ? -1 : 0; /* C failed     */
    }

    if ((d2 & CSM_DAP_BIT7) == 0u)
    {                   /* BDFAIL               */
        r->rbdfail = 1; /* B failed             */
    }
    else
    {
        r->rbdfail = ((d2 & CSM_DAP_BIT1) == 0u) ? -1 : 0; /* D failed     */
    }

    if (!AGC_ASSERT((r->racfail >= -1) && (r->racfail <= 1)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT((r->rbdfail >= -1) && (r->rbdfail <= 1)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* S41.2 (pp. 731-733): decode DAPDATR1, DAPDATR2. Called by DONOUN46
 * (V48) or directly by FRESHDAP.                                         */
agc_status_t agc_csm_dap_decode_dap_data(void)
{
    agc_status_t st  = s41_2_datr1();
    agc_status_t st2 = s41_2_datr2();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(st2 == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return (st != AGC_STATUS_OK) ? st : st2;
}

/* S40.14 (p. 735): RCS gains from IXX, IAVG (2^20 kg m^2):
 * J/M = IXX (2PI/M), J/M1 = J/M2 = IAVG (2PI/M);
 * KMJ = .00118 / IXX, KMJ1 = KMJ2 = .00118 / IAVG.                       */
agc_status_t agc_csm_dap_rcs_gains(void)
{
    agc_csm_rcs_t          *r = agc_csm_dap_rcs();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();

    if (!AGC_ASSERT(c->ixx >= 0) || !AGC_ASSERT(c->iavg >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    r->jm[0]  = csm_dap_mp_hi(c->ixx, CONTONE);
    r->jm[1]  = csm_dap_mp_hi(c->iavg, CONTONE);
    r->jm[2]  = r->jm[1];
    r->kmj[0] = csm_dap_dv(CONTTWO, c->ixx);
    r->kmj[1] = csm_dap_dv(CONTTWO, c->iavg);
    r->kmj[2] = r->kmj[1];

    return AGC_STATUS_OK;
}

/* RCSDAPON +1 (p. 736): entry from R00TOP00 with the TIME5 word in A.
 * Must be called under interrupt inhibit. T5PHASE + causes FRESHDAP.     */
agc_status_t agc_csm_dap_rcsdapon_t5(uint16_t time5_word)
{
    agc_csm_rcs_t    *r = agc_csm_dap_rcs();
    agc_csm_dapcom_t *c = agc_csm_dap_com();

    if (!AGC_ASSERT((time5_word > 0u) && (time5_word <= 037777u)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    r->t5phase = AGC_CSM_T5P_INIT; /* TS T5PHASE (+)        */
    /* set BIT3 to reinitialise the FDAI error display, in case the S/C
     * control switch is in SCS, not GNC                                   */
    c->rcsflags = (uint16_t) (c->rcsflags | AGC_CSM_RCSF_NEEDLE1);
    agc_status_t st = csm_dap_set_t5(AGC_TASK_RCSATT, time5_word);

    /* FLAGWRD6 bits 15,14 to 01: T5 takeover by the RCS DAP (kills
     * TVCEXEC and ROLLDAP)                                                */
    if (st == AGC_STATUS_OK)
    {
        agc_flag_clear(AGC_FLAG_DAPBIT1);
        st = AGC_STATUS_OK;
    }

    if (st == AGC_STATUS_OK)
    {
        agc_flag_set(AGC_FLAG_DAPBIT2);
        st = AGC_STATUS_OK;
    }

    return st;
}

/* RCSDAPON (p. 736): 0.6 s allows TVCEXEC/ROLLDAP to die.                */
agc_status_t agc_csm_dap_rcsdapon(void)
{
    if (!AGC_ASSERT(T5_06SEC <= 037777u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = agc_csm_dap_rcsdapon_t5(T5_06SEC);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* RCSUP (p. 736): T5 task (RCSCADR) that activates the RCS DAP.          */
agc_status_t agc_task_rcsup(void)
{
    agc_status_t st = agc_csm_dap_rcsdapon();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(agc_rupt_t5_task() == AGC_TASK_RCSATT))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st; /* RESUME                */
}
