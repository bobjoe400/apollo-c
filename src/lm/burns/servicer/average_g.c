/**********************************************************************
 * FILE NAME: average_g.c
 *
 * PURPOSE:
 *   The Luminary average-G cycle of SERVICER.agc: PREREAD, BIBIBIAS,
 *   READACCS, SERVICER (mass monitor, DV monitor), AVERAGEG with CALCRVG or
 *   the lunar-surface integrator MUNRVG/RVBOTH/MUNGRAV, the copy cycle,
 *   AVGEND, SERVIDLE, SERVEXIT and NORMLIZE, and the REREADAC restart task.
 *
 * AGC SOURCE:
 *   Luminary099 SERVICER.agc pp. 857-883
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   DELV          agc_vec_t  I/O  2^14 pulses  PIPA counts (high words)
 *   CDUX/Y/Z      uint16_t   I    CDU counts   gimbal angles at PIPA time
 *   PIPTIME       agc_dp_t   I    cs 2^28      time of RN, VN
 *   REFSMMAT      agc_mat_t  I    2^1          SM to reference
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_guid_pipasr       PIPASR (servicer_common.c)
 *   agc_guid_calcrvg      CALCRVG / CALCGRAV (servicer_common.c)
 *   agc_imu_fleshpot     FLESHPOT into XNBPIP (powered_flight.c)
 *   agc_lm_dap_jet_accel_update  1/ACCS (lm/dap.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None raised here; executive, waitlist and DAP failures come back as
 *   their status.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only. READACCS and REREADAC run as waitlist tasks under INHINT,
 *   SERVICER and NORMLIZE as jobs; restart groups 2 and 5 (QUIKFAZ5 names
 *   the C continuation job of each restart point).
 *
 * NOTES:
 *   The landing radar logic entered from MUNRETRN is in landing_radar.c;
 *   the arithmetic shared with Comanche is in src/guidance/servicer_common.c.
 *
 * ALGORITHM:
 *   Scales: R, R1S, R(CSM) SM m 2^24; V, V1S, V(CSM), G(CSM), GDT/2
 *   m/cs 2^7; PGUIDE cs 2^28; MASS, MASS1 kg 2^16; ABDELV PIPA pulses
 *   (1 cm/s each); ABDVCONV m/cs 2^5; DVTOTAL m/cs 2^7.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

#define TWO_SECS 200                    /* 2SECS, cs                                  */
#define ABORT_MONITOR_FIRST_CS \
    20                                  /* OCT24: first R10,R11 in .200 seconds       */
#define ONEPIPADT_2S \
    ((agc_sp_t) 12800)                  /* PRIO31 = OCT 31000: 2.0 s at B+8 cs */
#define TIME5_SYNC \
    ((agc_sp_t) 16377)                  /* OCT37771: PAXIS rupt 70 ms after READACCS */
#define GRP2        2                   /* restart group 2 (R10, R11) */
#define GRP5        5                   /* restart group 5 (average-G) */
#define PH_5_3_BJOB ((uint16_t) 016035) /* B: 5.3 REREADAC + job, PRIO20    */

/* KPIP DEC .0512 (SP): PIPA pulses to m/cs 2^5.                           */
static const agc_sp_t KPIP_LM = 839; /* KPIP .0512: pulses to m/cs 2^5 */

/* KPIP1 2DEC .0128: high word 209 used by CA KPIP1 / SHORTMP (2^7 m/cs).  */
static const agc_sp_t KPIP1_HI = 209; /* KPIP1 high word: pulses to m/cs 2^7 */

/* KPIP2 2DEC .0064: PIPA pulses to m/cs 2^8.                              */
static const agc_dp_t KPIP2_LM = 1717987; /* KPIP2 .0064: pulses to m/cs 2^8 */

/* APSVEX DEC -3030 E-2 B-5, DPSVEX DEC* -2.95588868 E+1 B-05*: m/cs 2^5.   */
static const agc_sp_t APSVEX = -15514; /* APSVEX, m/cs 2^5 */
static const agc_sp_t DPSVEX = -15134; /* DPSVEX, m/cs 2^5 */

/* SHIFT11 2DEC 1 B-11.                                                    */
static const agc_dp_t SHIFT11 = (agc_dp_t) 131072; /* 1 at 2^11 */

/* -MUDTMUN 2DEC* -9.8055560 E+10 B-38*: -mu(moon) * 200 cs.               */
static const agc_dp_t MUDTMUN = -(agc_dp_t) 95757383; /* -mu(moon) 200 cs, 2^38 */

/**********************************************************************
 * GNUFAZE5 / GNUTFAZ5 (p.859): new fixed phase of group 5, with or
 * without setting TBASE5.
 *
 * FUNCTION NAME: gnufaze5
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   phase     uint16_t    I    group 5 phase word
 *   tbase     bool        I    also set TBASE5
 **********************************************************************/
static void gnufaze5(uint16_t phase, bool tbase)
{
    agc_phase_newphase(GRP5, phase, tbase);
}

/**********************************************************************
 * QUIKFAZ5 (p.890): store the restart address in PHSNAME5. In C the
 * group-5 type B phase is re-entered with the continuation job named.
 *
 * FUNCTION NAME: agc_lm_serv_quikfaz5
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   resume    agc_job_id_t  I    job resumed after the phase change
 **********************************************************************/
void agc_lm_serv_quikfaz5(agc_job_id_t resume)
{
    agc_phase_var_t v = {AGC_RESTART_JOB, resume, AGC_TASK_NONE, (int16_t) AGC_PRIO20, false, 0U};

    agc_phase_change_var(PH_5_3_BJOB, &v);
}

/**********************************************************************
 * REPIP4 (p.870): guidance period PGUIDE = PIPTIME1 - PIPTIME, the CDUs at
 * PIPA time into CDUTEMP (X Y Z) and the raw pulses into PIPATMP for the
 * DAP.
 *
 * FUNCTION NAME: repip4
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   a         agc_avg_g_t *  I/O  average-G block (DELV, RN1, VN1, GDT1/2,
 *                                 PIPAGE...)
 **********************************************************************/
static void repip4(agc_avg_g_t *a)
{
    a->pguide      = a->piptime1 - agc_erase_piptime(); /* DCA PIPTIME1 DCS PIPTIME */
    a->cdutemp[0]  = agc_erase_cdux();
    a->cdutemp[1]  = agc_erase_cduy();
    a->cdutemp[2]  = agc_erase_cduz();
    agc_vec_t delv = agc_erase_delv();                  /* DELV, PIPA counts in the high words (2^14 pulses) */

    for (int i = 0; i < 3; i++)
    {
        a->pipatmp[i] = agc_dp_high(delv.c[i]);
    }
}

/**********************************************************************
 * PREREAD (p.857): 5.7 so that a restart skips LASTBIAS, then BIBIBIAS.
 *
 * FUNCTION NAME: agc_task_preread_lm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_preread_lm(void)
{
    gnufaze5(7, false);
    agc_status_t st = agc_exec_novac(AGC_PRIO21, AGC_JOB_LASTBIAS); /* status of agc_exec_novac */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_task_bibibias());
}

/**********************************************************************
 * BIBIBIAS (p.857): last free-fall reading (PIPASR +3 keeps PIPTIME1),
 * V37FLAG and AVEGFLAG set, DRIFTFLG reset, DV monitor initialised,
 * OUTROUTE = ENDOFJOB, NORMLIZE started, READACCS in 2 s.
 *
 * FUNCTION NAME: agc_task_bibibias
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_bibibias(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    agc_guid_pipasr(&g->avg, false);

    agc_flag_set(AGC_FLAG_SERVICER_RUNNING);                          /* SUPER011: BITS 5, 6    */
    agc_flag_set(AGC_FLAG_AVEGFLAG);
    agc_flag_clear(AGC_FLAG_DRIFTFLG);
    g->avg.pipage   = 4;                                              /* INITIALIZE DV MONITOR  */
    g->lm.outroute  = AGC_JOB_ENDOFJOB;                               /* ENDJBCAD               */
    agc_status_t st = agc_exec_findvac(AGC_PRIO22, AGC_JOB_NORMLIZE); /* NORMLIZE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    gnufaze5(2, true); /* 5.2SPOT                */

    return (agc_waitlist_vardelay(TWO_SECS, AGC_TASK_READACCS));
}

/**********************************************************************
 * REDO5.5 / MAKEACCS / AVEGOUT (p.858-859): restart 5.5 enters here.
 *
 * FUNCTION NAME: redo5_5
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   a         agc_avg_g_t *  I/O  average-G block (DELV, RN1, VN1, GDT1/2,
 *                                 PIPAGE...)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
static agc_status_t redo5_5(agc_avg_g_t *a)
{
    a->pipage       = 1;
    agc_status_t st = agc_exec_findvac(AGC_PRIO20, AGC_JOB_SERVICER); /* status of agc_exec_findvac */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_chan_or(011, (uint16_t) AGC_CH011_TESTCONN); /* TEST CONNECTOR ON */

    if (!agc_flag_test(AGC_FLAG_AVEGFLAG))
    { /* AVEGOUT                */
        a->avgexit = AGC_JOB_AVGEND;
        gnufaze5(4, true);

        return (AGC_STATUS_OK); /* TASKOVER, no READACCS  */
    }

    if (agc_flag_test(AGC_FLAG_MUNFLAG) && agc_phase_get(GRP2) == 0U)
    {
        /* R10,R11 set-up of REDO5.5 (p.858-859): 4x/s rate, group 2 at 2.21.
         * DEVIATION: TBASE2 is set ".05 seconds in the past" (-TIME1 + 5 - 1.0)
         * in the AGC; the exec API bases it on TIME1.                              */
        a->pipctr = 7;                        /* PIPCTR FOR 4X/SEC RATE */
        agc_phase_set_tbase(GRP2);
        agc_phase_newphase(GRP2, 021, false); /* 2.21SPOT FOR R10,R11   */

        st = agc_waitlist_call(ABORT_MONITOR_FIRST_CS, AGC_TASK_ABORT_MONITOR); /* not if PHASE2 active   */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    gnufaze5(4, true); /* MAKEACCS: GOREADAX     */

    return (agc_waitlist_vardelay(TWO_SECS, AGC_TASK_READACCS));
}

/**********************************************************************
 * PIPSDONE (p.858): 5.5 (REDO5.5), then REDO5.5.
 *
 * FUNCTION NAME: pipsdone
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   a         agc_avg_g_t *  I/O  average-G block (DELV, RN1, VN1, GDT1/2,
 *                                 PIPAGE...)
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t pipsdone(agc_avg_g_t *a)
{
    gnufaze5(5, false);

    return (redo5_5(a));
}

/**********************************************************************
 * REDO5.5 as the restart task of phase 5.5 (RESTART_TABLES p.242).
 *
 * FUNCTION NAME: agc_task_redo5_5_lm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_redo5_5_lm(void)
{
    return (redo5_5(&agc_guid_state()->avg));
}

/**********************************************************************
 * READACCS (p.858): waitlist task every 2 s.
 *
 * FUNCTION NAME: agc_task_readaccs_lm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_readaccs_lm(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    /* The READACCS-to-DAP synchronisation (p.858): nudge TIME5 by one count
     * toward OCT 37771 so that a PAXIS rupt comes about 70 ms after READACCS,
     * clear of the R10/R11 interrupts and of the downrupts.                    */
    agc_sp_t t5   = (agc_sp_t) (agc_rupt_time(5) & 037777u); /* CA TIME5; TIME5 counts, unscaled */
    agc_sp_t d    = (agc_sp_t) (t5 - TIME5_SYNC);            /* TIME5 - sync count, cs; unscaled */
    agc_sp_t step = 0;                                       /* TIME5 correction toward the sync count, unscaled */

    if (d > 0)
    {
        step = -1; /* CCS: CS ONE */
    }
    else if (d < 0)
    {
        step = 1; /* CCS: CA ONE */
    }

    agc_rupt_set_time(5, (uint16_t) (t5 + step)); /* ADS TIME5              */

    agc_guid_pipasr(&g->avg, true);

    repip4(&g->avg);

    return (pipsdone(&g->avg));
}

/**********************************************************************
 * REREADAC (p.871): restart task for group 5.
 *
 * FUNCTION NAME: agc_task_rereadac_lm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_rereadac_lm(void)
{
    agc_guid_t  *g  = agc_guid_state();                  /* guidance erasables */
    agc_status_t st = agc_guid_rereadac_resume(&g->avg); /* status of agc_guid_rereadac_resume */

    if (st == AGC_STATUS_NOT_FOUND)
    {
        return (agc_task_readaccs_lm()); /* NOT STARTED: BEGINNING */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st); /* PIPAGE out of range (agc_guid_rereadac_resume) */
    }

    repip4(&g->avg); /* Z DONE, GO DO CDUS     */

    return (pipsdone(&g->avg));
}

/**********************************************************************
 * MASSMON (p.860): off the surface, MASS1 = MASS - MASS * |DV| / |VEX|
 * with the APS or DPS exhaust velocity (VEX negative, so DV/VEX < 0).
 * DV divides the DP ABDVCONV by the SP VEX; MP uses the high word of MASS.
 *
 * FUNCTION NAME: massmon
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   lm        agc_guid_lm_t *  I/O  LM guidance erasables (MASS, MASS1)
 **********************************************************************/
static void massmon(agc_guid_lm_t *lm)
{
    if (!agc_flag_test(AGC_FLAG_SURFFLAG)) /* YES: BYPASS MASS MESS  */
    {
        agc_sp_t  vex = agc_flag_test(AGC_FLAG_APSFLAG) ? APSVEX : DPSVEX;         /* VEX of the engine, m/cs 2^5 */
        agc_ovf_t ovf = 0;                                                         /* overflow of the DAS (not tested, as in the AGC) */
        agc_dp_t  q   = agc_dp_div_raw(lm->abdvconv, agc_dp_from_sp(vex), &ovf);   /* DV Q; ABDVCONV/VEX, scale 2^0 */
        agc_dp_t  dm  = agc_sp_mul(agc_dp_high(q), agc_dp_high(agc_erase_mass())); /* MP MASS; mass decrement, kg 2^16 */

        lm->mass1 = agc_dp_add_raw(lm->mass1, dm, &ovf); /* DAS MASS1 */
    }
}

/**********************************************************************
 * TMPTOSPT (p.872): CDUSPOT (Y Z X) = CDUTEMP of the PIPA reading.
 *
 * FUNCTION NAME: agc_lm_tmptospt
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_lm_tmptospt(void)
{
    const agc_guid_t *g    = agc_guid_state(); /* guidance erasables */
    agc_cdutrig_t    *trig = agc_imu_trig();   /* CDUSPOT */

    trig->cduraw[AGC_TRIG_Y] = (agc_sp_t) (g->avg.cdutemp[1] & 077777U);
    trig->cduraw[AGC_TRIG_Z] = (agc_sp_t) (g->avg.cdutemp[2] & 077777U);
    trig->cduraw[AGC_TRIG_X] = (agc_sp_t) (g->avg.cdutemp[0] & 077777U);
}

/**********************************************************************
 * SERVICER (p.860): restart point GETABVAL, compensation, then GETABVAL.
 *
 * FUNCTION NAME: agc_job_servicer_lm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_servicer_lm(agc_job_ctx_t *ctx)
{
    agc_lm_serv_quikfaz5(AGC_JOB_SERV_DVTOTUP); /* PHASCHNG 16035 GETABVAL */

    agc_status_t st = agc_imu_set_pipa_interval(ONEPIPADT_2S); /* IN CASE LASTBIAS SKIPPED */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t delv = agc_erase_delv(); /* DELV, PIPA counts in the high words (2^14 pulses) */

    st = agc_imu_one_over_pipa(&delv); /* PIPA COMPENSATION      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_delv(delv);

    return (agc_job_serv_resume_lm(ctx)); /* falls into GETABVAL    */
}

/**********************************************************************
 * MUNGRAV (p.883): lunar gravity for a spherical moon. r: SM m 2^24.
 * UNIT leaves |r|^2 at 34D (2^48); SL 6 and BDDV -MUDTMUN (B-38) give
 * -mu dt / r^2 at 2^4, DMP SHIFT11 to 2^-7, VXSC UNIT/R/ (half unit):
 * GDT1/2 = half the 2-s gravity velocity at m/cs 2^7.
 *
 * FUNCTION NAME: agc_lm_mungrav
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   r         agc_vec_t       I    position R, m 2^24
 *
 * RETURNS: agc_vec_t -- GDT1/2, half the 2-s lunar gravity velocity,
 *          m/cs 2^7
 **********************************************************************/
agc_vec_t agc_lm_mungrav(agc_interp_t *ip, agc_vec_t r)
{
    agc_avg_g_t *a = &agc_guid_state()->avg; /* average-G erasables */

    a->unitr   = agc_vec_unit(ip, r, &a->rmag, &a->rmagsq); /* STODL UNIT/R/ 34D */
    agc_dp_t x = agc_dp_shl(ip, a->rmagsq, 6);              /* SL 6D; |R|^2 SL 6, m^2 2^42 */

    agc_dp_t k = agc_dp_mul_trunc(agc_dp_div(ip, MUDTMUN, x), SHIFT11); /* BDDV DMP; -mu dt/R^2, m/cs 2^-7 */

    a->gdt1_2 = agc_vec_scale(ip, a->unitr, k); /* VXSC UNIT/R/  */

    return (a->gdt1_2);
}

/**********************************************************************
 * RVBOTH (p.882), first part: carry the CSM state along with the LM's
 * (same spherical-moon average-G):
 *   R1S = R(CSM) + (G(CSM) + V(CSM)) * PGUIDE / SHIFT11   (SM m 2^24)
 *   V1S = GDT1/2 + V(CSM) + G(CSM)                          (m/cs 2^7)
 *
 * FUNCTION NAME: agc_lm_rvboth
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
agc_status_t agc_lm_rvboth(agc_interp_t *ip)
{
    agc_guid_t *g  = agc_guid_state();                          /* guidance erasables */
    agc_vec_t   gv = agc_vec_add(ip, g->lm.g_csm, g->lm.v_csm); /* VAD V(CSM); G(CSM) + V(CSM), m/cs 2^7 */
    agc_dp_t    dt = agc_dp_div(ip, g->avg.pguide, SHIFT11);    /* DDV SHIFT11; PGUIDE, cs 2^17 */

    g->lm.r1s     = agc_vec_add(ip, agc_vec_scale(ip, gv, dt), g->lm.r_csm);
    agc_vec_t gdt = agc_lm_mungrav(ip, g->lm.r1s); /* STCALL MUNGRAV; GDT1/2, m/cs 2^7 */

    g->lm.v1s = agc_vec_add(ip, agc_vec_add(ip, gdt, g->lm.v_csm), g->lm.g_csm);

    if (!AGC_ASSERT(dt >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * RVBOTH, second part (p.882): store the new CSM state.
 *
 * FUNCTION NAME: rvboth_store
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   g         agc_guid_t *  I/O  guidance erasables
 **********************************************************************/
static void rvboth_store(agc_guid_t *g)
{
    g->lm.g_csm = g->avg.gdt1_2; /* STOVL G(CSM)           */
    g->lm.r_csm = g->lm.r1s;     /* STOVL R(CSM)           */
    g->lm.v_csm = g->lm.v1s;     /* STORE V(CSM)           */
}

/**********************************************************************
 * MUNRVG (p.882-883): the LM's own spherical-moon average-G in SM axes:
 *   R1S = R + (V + (DELV + GDT)/2) * PGUIDE / SHIFT11
 *   V1S = V + DELV + GDT/2 + GDT1/2
 * plus ABVEL, HDOTDISP = UNIT/R/ . V1S SL1, DELVS = R1S x WM VSL2 (lunar
 * rotation, m/cs 2^5) and HCALC = |R1S| - /LAND/ (m 2^24).
 *
 * FUNCTION NAME: agc_lm_munrvg
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: none
 **********************************************************************/
void agc_lm_munrvg(agc_interp_t *ip)
{
    agc_guid_t   *g   = agc_guid_state();                              /* guidance erasables */
    agc_lm_ldg_t *ldg = agc_lm_ldg();                                  /* landing guidance erasables */
    agc_vec_t     dv8 = agc_vec_scale(ip, agc_erase_delv(), KPIP2_LM); /* 2^8 m/cs   */
    agc_vec_t     avg = agc_vec_add(ip, dv8, g->avg.gdt_2);            /* (DELV+GDT)/2; m/cs 2^7 */
    agc_vec_t     vav = agc_vec_add(ip, avg, g->lm.v);                 /* VAD V; m/cs 2^7 */
    agc_dp_t      dt  = agc_dp_div(ip, g->avg.pguide, SHIFT11);        /* PGUIDE / 2^11, cs 2^17 */

    g->lm.r1s     = agc_vec_add(ip, agc_vec_scale(ip, vav, dt), g->lm.r);
    agc_vec_t gdt = agc_lm_mungrav(ip, g->lm.r1s);                       /* GDT1/2 at R1S, m/cs 2^7 */
    agc_vec_t v1  = agc_vec_add(ip, agc_vec_add(ip, gdt, g->lm.v), avg); /* V1S before the copy, m/cs 2^7 */

    g->lm.v1s      = agc_vec_add(ip, v1, dv8);                   /* VAD VAD VAD V */
    g->lm.abvel    = agc_vec_abval(ip, g->lm.v1s);               /* SPEED         */
    g->lm.hdotdisp = agc_dp_shl(ip, agc_vec_dot(ip, g->avg.unitr, g->lm.v1s), 1);
    g->lm.delvs    = agc_vec_shl(ip, agc_vec_cross(ip, g->lm.r1s, ldg->wm), 2);
    g->lm.hcalc    = agc_dp_sub(ip, g->avg.rmag, ldg->land_mag); /* 36D DSU /LAND/ */
}

/**********************************************************************
 * GETABVAL .. AVERAGEG (p.860-861): ABDELV, mass, DVTOTAL, CDUSPOT and the
 * NB->SM matrix at PIPA time (QUICTRIG, FLESHPOT into XNBPIP), then
 * MUNRVG (MUNFLAG, via RVBOTH) or CALCRVG.
 *
 * FUNCTION NAME: getabval
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
static agc_status_t getabval(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    /* GETABVAL / MASSMON / MOONSPOT (p.860-861): |DELV| for the DV monitor
     * and mass, DVTOTAL += |DELV| * KPIP1 (SHORTMP with the SP KPIP1).        */
    {
        agc_interp_t ip; /* interpreter state */

        agc_interp_init(&ip);
        agc_dp_t abv = agc_vec_abval(&ip, agc_erase_delv()); /* VLOAD ABVAL DELV; |DELV|, scale 2^14 pulses */

        agc_erase_set_abdelv(agc_dp_high(abv));                                      /* ABDELV cm/s 2^14 */
        g->lm.abdvconv = agc_sp_mul(agc_erase_abdelv(), KPIP_LM);                    /* MP KPIP: 2^5     */
        g->lm.mass1    = agc_erase_mass();                                           /* INITIALIZE MASS1 */
        massmon(&g->lm);
        agc_dp_t  dv   = agc_tp_to_dp(agc_dp_mul_tp(abv, agc_dp_from_sp(KPIP1_HI))); /* SHORTMP; DVTOTAL increment, m/cs 2^7 */
        agc_ovf_t ovf  = 0;                                                          /* overflow of the DAS (not tested, as in the AGC) */

        g->avg.dvtotal = agc_dp_add_raw(g->avg.dvtotal, dv, &ovf); /* DAS DVTOTAL */
    }
    agc_lm_tmptospt();
    agc_cdutrig_t *trig   = agc_imu_trig(); /* CDUSPOT, SINCDU, COSCDU */
    const uint16_t cdu[3] = {(uint16_t) trig->cduraw[0], (uint16_t) trig->cduraw[1], (uint16_t) trig->cduraw[2]};

    agc_imu_quictrig(trig, cdu);
    agc_imu_fleshpot(trig, &g->lm.xnbpip);
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);

    if (agc_flag_test(AGC_FLAG_MUNFLAG))
    { /* BON MUNFLAG RVBOTH     */
        agc_status_t st = agc_lm_rvboth(&ip); /* status of agc_lm_rvboth */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        agc_lm_serv_quikfaz5(AGC_JOB_SERV_RVBOTH2);
        rvboth_store(g);
        agc_lm_serv_quikfaz5(AGC_JOB_SERV_MUNRVG);
        agc_lm_munrvg(&ip);

        return (agc_lm_serv_munretrn()); /* MUNRETRN */
    }

    agc_state_vector_t sv; /* state vector (RN, VN, PIPTIME) */

    agc_sv_get(AGC_SV_THIS, &sv);
    agc_mat_t    refsmmat = agc_erase_refsmmat();                                                                                                   /* REFSMMAT, half-unit rows (2^1) */
    agc_status_t st       = agc_guid_calcrvg(&ip, AGC_VEHICLE_LM, agc_rendz_erasable()->body.rtx2 != 0, &refsmmat, agc_erase_delv(), &sv, &g->avg); /* RTX2 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_lm_serv_quikfaz5(AGC_JOB_SERV_COPY); /* GOSERV                 */

    return (agc_lm_serv_copycycl());
} /* getabval */

/**********************************************************************
 * SERVOUT (p.863): 1/ACCS (DAP acceleration estimates), then to AVGEXIT
 * whether there is thrust or not; STEERSW tells the routine which.
 *
 * FUNCTION NAME: servout
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t servout(void)
{
    if (agc_interrupts_inhibited())
    {
        agc_relint(); /* SERVOUT RELINT         */
    }

    agc_status_t st = agc_lm_dap_jet_accel_update(); /* status of agc_lm_dap_jet_accel_update */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_lm_serv_quikfaz5(AGC_JOB_SERV_EXIT);

    return (agc_lm_avgexit());
}

/**********************************************************************
 * LOTHRUST .. DECCNTR (p.862): low thrust. After DVCNTR passes the COMFAIL
 * job (one at a time) handles the engine failure; meanwhile no steering:
 * STOPRATE zeroes the desired rates.
 *
 * FUNCTION NAME: lothrust
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t lothrust(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm; /* LM guidance erasables */

    agc_lm_serv_quikfaz5(AGC_JOB_SERV_LOTHRUST);

    if (lm->dvcntr > 0)
    { /* CCS DVCNTR: DECCNTR    */
        lm->mass1  = agc_dp_from_sp((agc_sp_t) (lm->dvcntr - 1)); /* TS DVCNTR1   */
        agc_lm_serv_quikfaz5(AGC_JOB_SERV_DECCNTR);
        lm->dvcntr = agc_dp_high(lm->mass1);
        agc_lm_dap_stoprate();                                    /* NO STEERING (INHINT)   */
        agc_flag_set(AGC_FLAG_USEQRFLG);                          /* USEJETS                */

        return (servout());
    }

    if (agc_phase_get(4) == 0U)
    { /* COMFAIL JOB ACTIVE?    */
        agc_phase_change((uint16_t) 000374);                           /* 4.37SPOT FOR COMFAIL   */
        agc_status_t st = agc_exec_novac(AGC_PRIO25, AGC_JOB_COMFAIL); /* status of agc_exec_novac */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (servout());
}

/**********************************************************************
 * DVMON (p.861-862): thrust detected when ABDELV > DVTHRUSH: STEERSW set,
 * two low-thrust passes allowed again; the trim gimbal is used unless the
 * APS is in use or the gimbal has failed (channel 32 bit 9).
 *
 * FUNCTION NAME: dvmon
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t dvmon(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm; /* LM guidance erasables */
    agc_status_t   st;                         /* status of SERVOUT or LOTHRUST */

    if (agc_erase_abdelv() - lm->dvthrush <= 0)
    {
        st = lothrust();
    }
    else
    {
        agc_flag_set(AGC_FLAG_STEERSW);
        lm->dvcntr = 1; /* DVCNTSET               */

        if (agc_flag_test(AGC_FLAG_APSFLAG) || ((agc_chan_read(032) & (uint16_t) AGC_CH032_DPSGIMBDIS) == 0U))
        {
            agc_flag_set(AGC_FLAG_USEQRFLG); /* USEJETS                */
        }
        else
        {
            agc_flag_clear(AGC_FLAG_USEQRFLG); /* USEGTS                 */
        }

        st = servout();
    }

    return (st);
}

/**********************************************************************
 * COPYCYCL (p.861): copy cycle (returns under INHINT), PIPATMP cleared,
 * STEERSW cleared before DVMON; IDLEFLAG and AUXFLAG bypass DVMON.
 *
 * FUNCTION NAME: agc_lm_serv_copycycl
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_serv_copycycl(void)
{
    agc_guid_t  *g = agc_guid_state(); /* guidance erasables */
    agc_status_t st;                   /* status of SERVOUT or DVMON */

    agc_guid_copycycle(AGC_VEHICLE_LM, AGC_COPY_ALL);
    agc_inhint(); /* RETURN UNDER INHINT    */
    g->avg.pipatmp[0] = 0;
    g->avg.pipatmp[1] = 0;
    g->avg.pipatmp[2] = 0;
    agc_flag_clear(AGC_FLAG_STEERSW);

    if (agc_flag_test(AGC_FLAG_IDLEFLAG))
    {
        agc_flag_clear(AGC_FLAG_AUXFLAG); /* NODVMON1               */
        agc_flag_set(AGC_FLAG_USEQRFLG);
        st = servout();
    }
    else if (!agc_flag_test(AGC_FLAG_AUXFLAG))
    {
        agc_flag_set(AGC_FLAG_AUXFLAG); /* NODVMON2               */
        agc_flag_set(AGC_FLAG_USEQRFLG);
        st = servout();
    }
    else
    {
        st = dvmon();
    }

    return (st);
}

/**********************************************************************
 * DCA AVGEXIT / DXCH Z (p.863).
 *
 * FUNCTION NAME: agc_lm_avgexit
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_lm_avgexit(void)
{
    agc_job_id_t exit_job = agc_guid_state()->avg.avgexit; /* AVGEXIT */

    if (!AGC_ASSERT(exit_job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_job_yield(exit_job));
}

/**********************************************************************
 * The restart points named by QUIKFAZ5 (one dispatcher, by job id).
 * AGC: QUIKFAZ5 restart points (SERVICER.agc pp.861-863, 890).
 *
 * FUNCTION NAME: agc_job_serv_resume_lm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_job_serv_resume_lm(agc_job_ctx_t *ctx)
{
    agc_guid_t  *g = agc_guid_state(); /* guidance erasables */
    agc_interp_t ip;                   /* interpreter state */

    agc_interp_init(&ip);
    const agc_job_id_t id = ctx->id; /* this restart point */

    if (id == AGC_JOB_SERV_RVBOTH2)
    {
        rvboth_store(g);
        agc_lm_serv_quikfaz5(AGC_JOB_SERV_MUNRVG);
    }

    if (id == AGC_JOB_SERV_RVBOTH2 || id == AGC_JOB_SERV_MUNRVG)
    {
        agc_lm_munrvg(&ip);

        return (agc_lm_serv_munretrn()); /* MUNRETRN */
    }

    if (id == AGC_JOB_SERV_COPY)
    {
        return (agc_lm_serv_copycycl());
    }

    if (id == AGC_JOB_SERV_EXIT)
    {
        return (agc_lm_avgexit());
    }

    if (id == AGC_JOB_SERV_COPYCYC1)
    {
        return (agc_lm_serv_copycyc1());
    }

    if (id == AGC_JOB_SERV_VMEASCHK)
    {
        return (agc_lm_serv_vmeaschk());
    }

    if (id == AGC_JOB_SERV_VALTCHK)
    {
        return (agc_lm_serv_valtchk());
    }

    if (id == AGC_JOB_SERV_RUPDATED)
    {
        return (agc_lm_serv_rupdated());
    }

    if (id == AGC_JOB_SERV_VUPDATED)
    {
        return (agc_lm_serv_vupdated());
    }

    if (id == AGC_JOB_SERV_LOTHRUST)
    {
        return (lothrust());
    }

    if (id == AGC_JOB_SERV_DECCNTR)
    {
        agc_inhint();
        g->lm.dvcntr = agc_dp_high(g->lm.mass1); /* CA DVCNTR1 TS DVCNTR   */
        agc_lm_dap_stoprate();
        agc_flag_set(AGC_FLAG_USEQRFLG);

        return (servout());
    }

    return (getabval()); /* SERVICER / GETABVAL    */
} /* agc_job_serv_resume_lm */

/**********************************************************************
 * AVGEND (p.863): the final pass. Free-fall gyro compensation, DRIFT flag,
 * PIPFREE, test connector off, group 5 off and 2.2 on for AVETOMID.
 *
 * FUNCTION NAME: agc_job_avgend_lm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_avgend_lm(agc_job_ctx_t *ctx)
{
    const agc_status_t st = agc_imu_set_pipa_interval(agc_dp_low(agc_erase_piptime())); /* CA PIPTIME +1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_set(AGC_FLAG_DRIFTFLG);
    agc_imu_pipfree();
    agc_chan_and(011, (uint16_t) ~(uint16_t) AGC_CH011_TESTCONN);
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_AVETOMID, AGC_TASK_NONE, (int16_t) AGC_PRIO20, false, 0U};

    agc_phase_change2(000005, 005022, &v); /* GROUP 5 OFF, 2 ON      */

    return (agc_job_avetomid_lm(ctx));
}

/**********************************************************************
 * AVGEND after the 2.2 restart point (p.863-864): R29 and R10 off, MUNFLAG
 * reset, AVETOMID, V37FLAG reset, then to OUTROUTE (AVERTRN).
 *
 * FUNCTION NAME: agc_job_avetomid_lm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_job_avetomid_lm(agc_job_ctx_t *ctx)
{
    agc_flag_set(AGC_FLAG_NO_POWERED_DESIGNATE); /* SHUT OFF R29           */
    agc_flag_clear(AGC_FLAG_SWANDISP);           /* SHUT OFF R10           */
    agc_flag_clear(AGC_FLAG_MUNFLAG);
    agc_interp_t ip;                             /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_integ_avetomid(&ip, agc_integ_shared_ctx(), ctx->id); /* status of agc_integ_avetomid */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_clear(AGC_FLAG_SERVICER_RUNNING);
    agc_job_id_t out = agc_guid_state()->lm.outroute; /* OUTROUTE */

    if (out == AGC_JOB_ENDOFJOB || out == AGC_JOB_NONE)
    {
        return (AGC_STATUS_OK); /* SERVEXIT +2: ENDOFJOB  */
    }

    return (agc_exec_job_yield(out)); /* BANKJUMP OUTROUTE      */
}

/**********************************************************************
 * SERVIDLE (p.865): disconnect SERVICER from all guidance, DV monitor off,
 * R12 terminated, groups 1, 3, 6 (and 2 unless MUNFLAG) off, 4.33 for
 * GOP00FIX, then a software restart (WHIMPER) while SERVICER continues in
 * a ground state. Also entered by POODOO (STRTIDLE, ALARM_AND_ABORT
 * p.1383) when SERVICER is running.
 *
 * FUNCTION NAME: agc_lm_servidle
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_lm_servidle(void)
{
    agc_guid_state()->avg.avgexit = AGC_JOB_SERVEXIT;                                     /* SVEXTADR               */
    agc_flag_set(AGC_FLAG_IDLEFLAG);
    agc_flagword_set(agc_flag_word(AGC_FLAG_LRBYPASS), agc_flag_mask(AGC_FLAG_LRBYPASS)); /* TS FLGWRD11  */
    agc_phase_kill(1);

    if (!agc_flag_test(AGC_FLAG_MUNFLAG))
    {
        agc_phase_kill(GRP2);
    }

    agc_phase_kill(3);
    agc_phase_kill(6);
    agc_phase_newphase(4, 033, false); /* 4.33SPOT FOR GOP00FIX  */

    agc_exec_enema(); /* TCF WHIMPER */
}

/**********************************************************************
 * SERVIDLE as a job (V37 to P00 while SERVICER runs).
 * AGC: SERVIDLE (SERVICER.agc p.865).
 *
 * FUNCTION NAME: agc_job_servidle
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_job_servidle(void)
{
    agc_lm_servidle();
}

/**********************************************************************
 * NORMLIZE (p.867): the first average-G pass. With MUNFLAG the SM state
 * R = REFSMMAT RN1 (VSL6, m 2^24), V = REFSMMAT VN1 (VSL1), GDT1/2 from
 * MUNGRAV and UHYP = UNIT(V(CSM) x R(CSM)); otherwise CALCGRAV(RN1).
 * Then NORMLIZ2 copies 20 words (not MASS).
 *
 * FUNCTION NAME: agc_job_normlize_lm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_job_normlize_lm(void)
{
    agc_guid_t  *g = agc_guid_state(); /* guidance erasables */
    agc_interp_t ip;                   /* interpreter state */

    agc_interp_init(&ip);

    if (agc_flag_test(AGC_FLAG_MUNFLAG))
    {
        agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows (2^1) */

        g->lm.r       = agc_mat_vec(&ip, &refsmmat, agc_vec_shl(&ip, g->avg.rn1, 6));
        g->avg.gdt1_2 = agc_lm_mungrav(&ip, g->lm.r); /* GDT1/2 */
        g->lm.v       = agc_mat_vec(&ip, &refsmmat, agc_vec_shl(&ip, g->avg.vn1, 1));
        g->lm.uhyp    = agc_vec_unit(&ip, agc_vec_cross(&ip, g->lm.v_csm, g->lm.r_csm), NULL, NULL);
        agc_phase_kill(GRP2);                         /* GROUP 2 IS OFF         */
    }
    else
    {
        agc_status_t st = agc_guid_calcgrav(&ip, AGC_VEHICLE_LM, agc_rendz_erasable()->body.rtx2 != 0, g->avg.rn1, &g->avg); /* CALCGRAV (moon: RTX2 = 2, SLOAD RTX2) */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    agc_guid_copycycle(AGC_VEHICLE_LM, AGC_COPY_RVTG);

    return (AGC_STATUS_OK); /* NORMLIZ2      */
}
