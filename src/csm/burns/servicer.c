/*
 * servicer.c -- SERVICER207.agc, Comanche055 pp. 819-836: PREREAD,
 * READACCS, ONMNITOR/QUIKREAD, SERVICER, AVERAGEG, AVGEND, SERVEXIT,
 * NORMLIZE and the REREADAC restart task. The arithmetic shared with
 * Luminary (PIPASR, CALCGRAV, CALCRVG, the copy cycle) is in
 * src/guidance/servicer_common.c.
 *
 * The cycle: a program calls PREREAD as a waitlist task 2 s before the
 * first average-G update; PREREAD reads the PIPAs for the last time in free
 * fall, starts NORMLIZE and calls READACCS in 2 s. READACCS reads the
 * PIPAs every 2 s while AVEGFLAG is set and starts the SERVICER job, which
 * compensates the readings, integrates RN/VN (CALCRVG), copies the results
 * and goes to the routine named in AVGEXIT (steering, displays). When
 * AVEGFLAG is cleared the last pass goes to AVGEND instead.
 *
 * Restart protection: group 5 (5.2 NORMLIZE, 5.3 REREADAC, 5.4 READACCS
 * and SERVICER, 5.5 REDO5.5, 5.37 CHEKAVEG). The type B phase changes
 * inside SERVICER name the C continuation job for each restart point.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "guid_pending.h"

#define MAXDELV  6398 /* -MAXDELV DEC -6398: 3200 pps for 2 s, less 1 (CCS) */
#define JTAGTIME 120  /* JTAGTIME DEC 120 cs = 1 s + T CDU (0.1 s)  */
#define TWO_SECS 200  /* 2SECS, cs                                  */
#define HALF_SEC 50   /* 0.5SEC DEC 50, cs                          */
#define ONEPIPADT_2S \
    ((agc_sp_t) 12800) /* PRIO31 = OCT 31000: 2.0 s at B+8 cs */

/* Restart groups and phases (octal codes as in the listing).              */
#define GRP1        1
#define GRP2        2
#define GRP5        5
#define PH_5_3_B    ((uint16_t) 010035) /* B: 5.3 REREADAC + job below   */
#define PH_5_3_BJOB ((uint16_t) 016035) /* B: 5.3 + 2CADR job, PRIO20    */
#define PH_5_3_A    ((uint16_t) 000035) /* A: 5.3 REREADAC only          */

/* KPIP1 2DEC 0.074880 (1 PIPA pulse = 5.85 cm/s), scale to m/cs 2^7.      */
static const agc_dp_t KPIP1_CM = (agc_dp_t) 20100447;

/* PHASCHNG type B for group 5 with a job restart point at PRIO20.          */
static void serv_phase(uint16_t code, agc_job_id_t job)
{
    agc_phase_var_t v = {AGC_RESTART_JOB, job, AGC_TASK_NONE,
                         (int16_t) AGC_PRIO20, false, 0U};

    if (!AGC_ASSERT(job != AGC_JOB_NONE))
    {
        return;
    }

    agc_phase_change_var(code, &v);
}

/* PREREAD1 (p.823): last free-fall PIPA reading and package set-up.        */
static agc_status_t preread1(agc_avg_g_t *a)
{
    agc_status_t st = agc_guid_pipasr(a, true); /* CLEAR + READ PIPS      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    a->pipage = 1;                           /* for REREADAC           */
    agc_flag_set(AGC_FLAG_AVEGFLAG);         /* SET AVEG FLAG          */
    agc_flag_clear(AGC_FLAG_DRIFTFLG);       /* KNOCK DOWN DRIFT FLAG  */
    agc_flag_set(AGC_FLAG_SERVICER_RUNNING); /* SET V37 FLAG           */
    a->dvtotal = 0;                                 /* CLEAR DVTOTAL          */

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_AVEGFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* PREREAD / REDO5.31 (p.822-823): task. The caller protects it. LASTBIAS
 * (last free-fall gyro compensation) is an unprotected NOVAC job; REREADAC
 * sets 1/PIPADT to 2 s in case it is lost.                                */
agc_status_t agc_task_preread_csm(void)
{
    agc_guid_t  *g  = agc_guid_state();
    agc_status_t sw = agc_exec_novac(AGC_PRIO21, AGC_JOB_LASTBIAS);

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_status_t st = preread1(&g->avg);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_exec_findvac(AGC_PRIO32, AGC_JOB_NORMLIZE); /* NORMLIZE job    */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_waitlist_call(TWO_SECS, AGC_TASK_READACCS);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_newphase(GRP5, 2, true);

    return (AGC_STATUS_OK); /* CS TWO NEWPHASE 5 */
}

/* Entry-DAP part of REDO5.5 (p.824-825): during entry (CM/DSTBY set) the
 * CDU and DAP angles are frozen at PIPA time for the entry DAP, and while
 * the entry DAP is armed the PIPA readings are saved every 0.5 s for TM.  */
static agc_status_t readaccs_entry(agc_avg_g_t *a)
{
    agc_guid_t        *g = agc_guid_state();
    agc_entry_state_t *e = agc_entry_state();

    if (!AGC_ASSERT(e != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* DEVIATION: CS PIPTIME1 +1 / TS TBASE6 bases group 6 on the PIPA time;
     * the exec API sets TBASE6 from TIME1, which equals it here.           */
    agc_phase_set_tbase(6);
    e->aog_pip  = agc_erase_aog(); /* DCA AOG DXCH AOG/PIP   */
    e->aig_pip  = agc_erase_aig();
    e->amg_pip  = agc_erase_amg(); /* CA AMG XCH AMG/PIP     */
    e->roll_pip = e->roll_180;     /* DCA ROLL/180           */
    e->alfa_pip = e->alfa_180;
    e->beta_pip = e->beta_180; /* CA BETA/180            */

    if (agc_flag_test(AGC_FLAG_CMDAPARM))
    { /* SAVE PIPAS EACH 0.5 SEC */
        agc_status_t sw = agc_waitlist_call(HALF_SEC, AGC_TASK_QUIKREAD);

        if (sw != AGC_STATUS_OK)
        {
            return (sw);
        }

        agc_vec_t delv = agc_erase_delv();

        for (int i = 0; i < 3; i++)
        {
            g->csm.xoldbuf[i] = g->csm.xpipbuf[i]; /* XCH XPIPBUF TS XOLDBUF */
            g->csm.xpipbuf[i] = agc_dp_high(delv.c[i]);
        }
    }

    agc_erase_set_cm_gymdt(5); /* CA FIVE TS CM/GYMDT    */
    agc_status_t st = agc_waitlist_call(JTAGTIME, AGC_TASK_SETJTAG);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_newphase(GRP1, 3, true); /* 1.3SPOT FOR SETJTAG    */

    if (!AGC_ASSERT(a->pipctr == 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_newphase(GRP5, 037, false);

    return (AGC_STATUS_OK); /* 5.37 = CHEKAVEG        */
}

/* CHEKAVEG / MAKESERV / AVEGOUT (p.825-826): recall READACCS while AVEGFLAG
 * is set, else make AVGEND the exit of this last pass; start SERVICER.    */
static agc_status_t chekaveg(agc_avg_g_t *a)
{
    if (!agc_flag_test(AGC_FLAG_AVEGFLAG))
    {
        a->avgexit = AGC_JOB_AVGEND; /* AVEGOUT: AVOUTCAD      */
    }
    else
    {
        agc_status_t st = agc_waitlist_call(TWO_SECS, AGC_TASK_READACCS);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    agc_status_t st =
        agc_exec_findvac(AGC_PRIO20, AGC_JOB_SERVICER); /* MAKESERV */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_newphase(GRP5, 4, true); /* CS FOUR NEWPHASE 5     */
    agc_chan_or(011, (uint16_t) AGC_CH011_TESTCONN);

    return (AGC_STATUS_OK); /* TEST CONNECTOR ON */
}

/* PIPSDONE / REDO5.5 (p.824): mark the reading done and continue.          */
static agc_status_t pipsdone(agc_avg_g_t *a)
{
    agc_phase_newphase(GRP5, 5, false); /* CAF FIVE DXCH -PHASE5  */
    a->pipage = 1;                             /* SHOW PIPS HAVE BEEN READ */
    a->pipctr = 2;                             /* PIPCTR FOR ONMNITOR    */

    if (agc_flag_test(AGC_FLAG_CM_DSTBY))
    {
        agc_status_t st = readaccs_entry(a);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (chekaveg(a));
}

/* READACCS (p.824): waitlist task every 2 s.                               */
agc_status_t agc_task_readaccs_csm(void)
{
    agc_guid_t  *g  = agc_guid_state();
    agc_status_t st = agc_guid_pipasr(&g->avg, true);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(g->avg.pipage == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pipsdone(&g->avg));
}

/* REREADAC (p.833): restart task for group 5. Does nothing on the last
 * pass (PHASE5 = +0); otherwise re-establishes 1/PIPADT (LASTBIAS may have
 * been lost) and either redoes READACCS or finishes it at PIPSDONE.       */
agc_status_t agc_task_rereadac_csm(void)
{
    agc_guid_t *g = agc_guid_state();

    if (agc_phase_get(GRP5) == 0U)
    {                         /* LAST PASS CHECK        */
        return (AGC_STATUS_OK); /* TASKOVER               */
    }

    agc_imu_set_pipa_interval(ONEPIPADT_2S); /* PRIO31 = 2.0 SEC B+8CS */
    agc_status_t st = agc_guid_rereadac_resume(&g->avg);

    if (st == AGC_STATUS_NOT_FOUND)
    {
        return (agc_task_readaccs_csm()); /* NOT STARTED: BEGINNING */
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (pipsdone(&g->avg));
}

/* QUIKREAD / CHKCTR (p.827): copy the PIPA counters (not zeroed) into
 * X,Y,ZPIPBUF and the previous values into X,Y,ZOLDBUF for the downlink;
 * repeat every 0.5 s while PIPCTR counts down (ONMNITOR).                 */
agc_status_t agc_task_quikread(void)
{
    agc_guid_t    *g       = agc_guid_state();
    const agc_sp_t pipa[3] = {agc_erase_pipax(), agc_erase_pipay(),
                              agc_erase_pipaz()};

    for (int i = 2; i >= 0; i--)
    { /* RUPTREG1 = 2, 1, 0     */
        g->csm.xoldbuf[i] = g->csm.xpipbuf[i];
        g->csm.xpipbuf[i] = pipa[i];
    }

    if (g->avg.pipctr > 0)
    {                    /* CCS PIPCTR             */
        g->avg.pipctr--; /* ONMNITOR: TS PIPCTR    */

        return (agc_waitlist_fixdelay(HALF_SEC, AGC_TASK_ONMNITOR));
    }

    if (!AGC_ASSERT(g->avg.pipctr == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* TASKOVER               */
}

/* ONMNITOR (p.827): the FIXDELAY return point of the 0.5 s loop.          */
agc_status_t agc_task_onmnitor(void)
{
    if (!AGC_ASSERT(agc_guid_state()->avg.pipctr >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_quikread());
}

/* NORMLIZE (p.831): copy RN1, VN1, PIPTIME1 into RN, VN, PIPTIME and
 * initialise UNITR, RMAG and GDT/2, GOBL/2 for the first AVERAGEG pass.   */
agc_status_t agc_job_normlize_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t *g = agc_guid_state();

    agc_guid_copycycle(AGC_VEHICLE_CSM, AGC_COPY_RVT); /* 14 REGS */
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_status_t st = agc_guid_calcgrav(&ip, AGC_VEHICLE_CSM,
                                        agc_flag_test(
                                            AGC_FLAG_MOONFLAG), agc_erase_rn(),
                                        &g->avg);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g->avg.gdt_2  = g->avg.gdt1_2;  /* STOVL GDT/2            */
    g->avg.gobl_2 = g->avg.gobl1_2; /* STORE GOBL/2           */

    return (AGC_STATUS_OK);           /* ENDOFJOB               */
}

/* PIPCHECK (p.828): PIPA saturation test before compensation. CCS gives
 * |DELV| - 1, so a reading of more than 6399 pulses in 2 s is bad.       */
static bool pipas_saturated(agc_vec_t delv)
{
    bool bad = false;

    for (int i = 2; i >= 0; i--)
    { /* RUPTREG1 = 2, 1, 0     */
        agc_sp_t p   = agc_dp_high(delv.c[i]);
        int      mag = (p < 0) ? -(int) p : (int) p;

        if (mag != 0 && (mag - 1 - MAXDELV) > 0)
        {
            bad = true;
            break; /* TC ALARM; TC AVERAGEG  */
        }
    }

    _Static_assert(MAXDELV == 6398, "AGC constant");

    return (bad);
}

/* SERVICER (p.828): the 2-second job. A saturated PIPA skips compensation
 * and the DVTOTAL update and goes straight to AVERAGEG.                   */
agc_status_t agc_job_servicer_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    bool bad = pipas_saturated(agc_erase_delv());

    agc_relint();

    if (bad)
    {
        agc_alarm(AGC_ALARM_00205); /* SATURATED-PIPA ALARM   */

        return (agc_job_serv_averageg_csm(ctx));
    }

    serv_phase(PH_5_3_BJOB, AGC_JOB_SERV_DVTOTUP); /* RESTART REREADAC+SERVICER */
    agc_vec_t    delv = agc_erase_delv();
    agc_status_t st   = agc_imu_one_over_pipa(&delv); /* PIPA COMPENSATION      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_delv(delv);

    return (agc_job_serv_dvtotup_csm(ctx));
}

/* DVTOTUP (p.828): DVTOTAL += |DELV| * KPIP1 (m/cs 2^7), then AVERAGEG.    */
agc_status_t agc_job_serv_dvtotup_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t  *g = agc_guid_state();
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_dp_t dv =
        agc_dp_mul(&ip, agc_vec_abval(&ip, agc_erase_delv()), KPIP1_CM);
    agc_ovf_t ovf = 0;

    g->avg.dvtotal = agc_dp_add_raw(g->avg.dvtotal, dv, &ovf); /* DAS DVTOTAL */

    if (!AGC_ASSERT(ovf == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_serv_averageg_csm(ctx));
}

/* AVERAGEG (p.828-829): CALCRVG, then the restart-protected copy.          */
agc_status_t agc_job_serv_averageg_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t *g = agc_guid_state();

    serv_phase(PH_5_3_B, AGC_JOB_SERV_AVERAGEG);
    agc_state_vector_t sv;
    agc_status_t       st = agc_sv_get(AGC_SV_THIS, &sv);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_mat_t    refsmmat = agc_erase_refsmmat();
    agc_interp_t ip;

    agc_interp_init(&ip);
    st =
        agc_guid_calcrvg(&ip, AGC_VEHICLE_CSM, agc_flag_test(AGC_FLAG_MOONFLAG),
                         &refsmmat, agc_erase_delv(), &sv, &g->avg);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_serv_copy_csm(ctx));
}

/* AVERAGEG copy (p.829): GENTRAN of 26 words RN1..GOBL1/2 under INHINT,
 * then the AVGEXIT transfer.                                               */
agc_status_t agc_job_serv_copy_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    serv_phase(PH_5_3_B, AGC_JOB_SERV_COPY);
    agc_guid_copycycle(AGC_VEHICLE_CSM, AGC_COPY_RVTG);
    serv_phase(PH_5_3_B, AGC_JOB_SERV_EXIT);

    return (agc_csm_avgexit(ctx));
}

/* DCA AVGEXIT / DXCH Z (p.829): continue this job at the routine named in
 * AVGEXIT (the C executive resumes the same job as that id).              */
agc_status_t agc_csm_avgexit(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_job_id_t exit_job = agc_guid_state()->avg.avgexit;

    if (!AGC_ASSERT(exit_job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_job_yield(exit_job));
}

/* AVGEND (p.829-830): the final pass. Free-fall gyro compensation, DRIFT
 * flag, state vector back to the integration scale (AVETOMID), mark
 * counters and mark buffer cleared, test connector off, then CANV37.      */
agc_status_t agc_job_avgend_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* CA PIPTIME +1 / TS OLDBT1 (= 1/PIPADT): time of the last reading.    */
    agc_imu_set_pipa_interval(agc_dp_low(agc_erase_piptime()));
    agc_flag_set(AGC_FLAG_DRIFTFLG);
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_AVETOMID,
                         AGC_TASK_NONE, (int16_t) AGC_PRIO20,
                         false, 0U};

    agc_phase_change2(000005, 005022, &v); /* 5 OFF, 2.2 AVETOMID    */

    return (agc_job_avetomid_csm(ctx));
}

/* AVGEND after the 2.2 restart point (p.829-830).                          */
agc_status_t agc_job_avetomid_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_status_t st =
        agc_guid_avetomid(&ip, ctx->id); /* CONVERT STATE VECTOR   */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_vhfcnt(0); /* ZERO MARK COUNTERS     */
    agc_rendz_erasable()->trkmkcnt = 0;
    agc_imu_pipfree();
    agc_erase_set_mrkbuf2(0, (agc_sp_t) -0400); /* CS BIT9: INVALIDATE    */
    agc_chan_and(011, (uint16_t) ~(uint16_t) AGC_CH011_TESTCONN);
    agc_flag_clear(AGC_FLAG_CM_DSTBY);
    agc_flag_clear(AGC_FLAG_SERVICER_RUNNING);

    if (agc_flag_test(AGC_FLAG_RNDVZFLG))
    { /* RESTORE GROUP 1 + 2    */
        agc_phase_change2(000111, 000132, NULL); /* 1.11SPOT, 2.13SPOT     */
    }

    return (agc_exec_change_program_continue(ctx)); /* POSTJUMP CANV37        */
}

/* SERVEXIT (p.830, both vehicles): leave only 5.3 (REREADAC) protection
 * and end the job.                                                         */
agc_status_t agc_job_servexit(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_guid_state()->avg.avgexit != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(GRP5, PH_5_3_A);

    return (AGC_STATUS_OK);
}
