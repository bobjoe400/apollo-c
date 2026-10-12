/**********************************************************************
 * FILE NAME: servicer.c
 *
 * PURPOSE:
 *   The Comanche average-G cycle of SERVICER207.agc: PREREAD, READACCS
 *   (with the entry-DAP part of REDO5.5), ONMNITOR/QUIKREAD, SERVICER and
 *   PIPCHECK, DVTOTUP, AVERAGEG, AVGEND, SERVEXIT, NORMLIZE, the REREADAC
 *   and CHEKAVEG restart tasks. The arithmetic shared with Luminary
 *   (PIPASR, CALCGRAV, CALCRVG, the copy cycle) is in
 *   src/guidance/servicer_common.c.
 *
 * AGC SOURCE:
 *   Comanche055 SERVICER207.agc pp. 819-836
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   DELV          agc_vec_t  I/O  2^14 pulses  PIPA counts (high words)
 *   PIPAX/Y/Z     agc_sp_t   I    pulses       PIPA counters (QUIKREAD)
 *   RN, PIPTIME   agc_vec_t  I    2^29 m, cs   state at the PIPA time
 *   REFSMMAT      agc_mat_t  I    2^1          SM to reference
 *   AIG/AMG/AOG   agc_sp_t   I    CDU counts   gimbal angles (entry DAP)
 *   CM/GYMDT      agc_sp_t   O    -            entry gyro drive count
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_guid_pipasr       PIPASR (servicer_common.c)
 *   agc_guid_calcrvg      CALCRVG / CALCGRAV (servicer_common.c)
 *   agc_guid_copycycle    GENTRAN copy cycle (servicer_common.c)
 *   agc_imu_*             PIPA compensation and the PIPA interval (imu.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 00205 (PIPA saturated, PIPCHECK). Executive and waitlist
 *   failures (BAILOUT 1201/1202, 1203) come back as their status.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only. READACCS, ONMNITOR and the restart tasks run as waitlist
 *   tasks under INHINT; SERVICER and NORMLIZE as jobs. Restart group 5
 *   (5.2 NORMLIZE, 5.3 REREADAC, 5.4 READACCS and SERVICER, 5.5 REDO5.5,
 *   5.37 CHEKAVEG); the type B phase changes name the C continuation job
 *   of each restart point.
 *
 * NOTES:
 *   The cycle: a program calls PREREAD 2 s before the first update;
 *   PREREAD reads the PIPAs for the last time in free fall, starts NORMLIZE
 *   and calls READACCS in 2 s. READACCS reads the PIPAs every 2 s while
 *   AVEGFLAG is set and starts SERVICER, which compensates the readings,
 *   integrates RN/VN (CALCRVG), copies the results and goes to the routine
 *   named in AVGEXIT. With AVEGFLAG cleared the last pass goes to AVGEND.
 *
 * ALGORITHM:
 *   As in the listing. Scales: RN m 2^29, VN and DELVREF m/cs 2^7,
 *   DVTOTAL m/cs 2^7, PIPTIME cs 2^28.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/rendezvous.h"

#define MAXDELV  6398  /* -MAXDELV DEC -6398: 3200 pps for 2 s, less 1 (CCS) */
#define JTAGTIME 120   /* JTAGTIME DEC 120 cs = 1 s + T CDU (0.1 s)  */
#define TWO_SECS 200   /* 2SECS, cs                                  */
#define HALF_SEC 50    /* 0.5SEC DEC 50, cs                          */
#define ONEPIPADT_2S \
    ((agc_sp_t) 12800) /* PRIO31 = OCT 31000: 2.0 s at B+8 cs */

/* Restart groups and phases (octal codes as in the listing).              */
#define GRP1        1                   /* restart group 1 (SETJTAG) */
#define GRP2        2                   /* restart group 2 (AVETOMID) */
#define GRP5        5                   /* restart group 5 (average-G) */
#define PH_5_3_B    ((uint16_t) 010035) /* B: 5.3 REREADAC + job below   */
#define PH_5_3_BJOB ((uint16_t) 016035) /* B: 5.3 + 2CADR job, PRIO20    */
#define PH_5_3_A    ((uint16_t) 000035) /* A: 5.3 REREADAC only          */

/* KPIP1 2DEC 0.074880 (1 PIPA pulse = 5.85 cm/s), scale to m/cs 2^7.      */
static const agc_dp_t KPIP1_CM = (agc_dp_t) 20100447; /* KPIP1 0.074880: pulses to m/cs 2^7 */

/**********************************************************************
 * PHASCHNG type B for group 5 with a job restart point at PRIO20.
 * AGC: none (helper for the type B PHASCHNG 5.3 calls, SERVICER207
 * pp.828-829).
 *
 * FUNCTION NAME: serv_phase
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   code      uint16_t      I    PHASCHNG word for group 5 (octal, as in
 *                                the listing)
 *   job       agc_job_id_t  I    restart job for the phase (2CADR in the
 *                                AGC)
 **********************************************************************/
static void serv_phase(uint16_t code, agc_job_id_t job)
{
    agc_phase_var_t v = {AGC_RESTART_JOB, job, AGC_TASK_NONE, (int16_t) AGC_PRIO20, false, 0U};

    agc_phase_change_var(code, &v);
}

/**********************************************************************
 * PREREAD1 (p.823): last free-fall PIPA reading and package set-up.
 *
 * FUNCTION NAME: preread1
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   a         agc_avg_g_t *  I/O  average-G block (DELV, RN1, VN1, GDT1/2,
 *                                 PIPAGE...)
 *
 * RETURNS: none
 **********************************************************************/
static void preread1(agc_avg_g_t *a)
{
    agc_guid_pipasr(a, true);

    a->pipage  = 1;                          /* for REREADAC           */
    agc_flag_set(AGC_FLAG_AVEGFLAG);         /* SET AVEG FLAG          */
    agc_flag_clear(AGC_FLAG_DRIFTFLG);       /* KNOCK DOWN DRIFT FLAG  */
    agc_flag_set(AGC_FLAG_SERVICER_RUNNING); /* SET V37 FLAG           */
    a->dvtotal = 0;                          /* CLEAR DVTOTAL          */
}

/**********************************************************************
 * PREREAD1 for P11 (IBNKCALL PREREAD1 at liftoff, P11 p.552).
 *
 * FUNCTION NAME: agc_guid_preread1_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_guid_preread1_csm(void)
{
    preread1(&agc_guid_state()->avg);
}

/**********************************************************************
 * PREREAD / REDO5.31 (p.822-823): task. The caller protects it. LASTBIAS
 * (last free-fall gyro compensation) is an unprotected NOVAC job; REREADAC
 * sets 1/PIPADT to 2 s in case it is lost.
 *
 * FUNCTION NAME: agc_task_preread_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_task_preread_csm(void)
{
    agc_guid_t  *g  = agc_guid_state();                             /* guidance erasables */
    agc_status_t sw = agc_exec_novac(AGC_PRIO21, AGC_JOB_LASTBIAS); /* status of agc_exec_novac */

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    preread1(&g->avg);

    agc_status_t st = agc_exec_findvac(AGC_PRIO32, AGC_JOB_NORMLIZE); /* NORMLIZE job    */

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

/**********************************************************************
 * Entry-DAP part of REDO5.5 (p.824-825): during entry (CM/DSTBY set) the
 * CDU and DAP angles are frozen at PIPA time for the entry DAP, and while
 * the entry DAP is armed the PIPA readings are saved every 0.5 s for TM.
 *
 * FUNCTION NAME: readaccs_entry
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          WAITLIST call that failed
 **********************************************************************/
static agc_status_t readaccs_entry(void)
{
    agc_guid_t        *g = agc_guid_state();  /* guidance erasables */
    agc_entry_state_t *e = agc_entry_state(); /* entry erasables */

    /* DEVIATION: CS PIPTIME1 +1 / TS TBASE6 bases group 6 on the PIPA time;
     * the exec API sets TBASE6 from TIME1, which equals it here.           */
    agc_phase_set_tbase(6);
    e->aog_pip  = agc_erase_aog(); /* DCA AOG DXCH AOG/PIP   */
    e->aig_pip  = agc_erase_aig();
    e->amg_pip  = agc_erase_amg(); /* CA AMG XCH AMG/PIP     */
    e->roll_pip = e->roll_180;     /* DCA ROLL/180           */
    e->alfa_pip = e->alfa_180;
    e->beta_pip = e->beta_180;     /* CA BETA/180            */

    if (agc_flag_test(AGC_FLAG_CMDAPARM))
    { /* SAVE PIPAS EACH 0.5 SEC */
        agc_status_t sw = agc_waitlist_call(HALF_SEC, AGC_TASK_QUIKREAD); /* status of agc_waitlist_call */

        if (sw != AGC_STATUS_OK)
        {
            return (sw);
        }

        agc_vec_t delv = agc_erase_delv(); /* DELV, PIPA counts in the high words (2^14 pulses) */

        for (int i = 0; i < 3; i++)
        {
            g->csm.xoldbuf[i] = g->csm.xpipbuf[i]; /* XCH XPIPBUF TS XOLDBUF */
            g->csm.xpipbuf[i] = agc_dp_high(delv.c[i]);
        }
    }

    agc_erase_set_cm_gymdt(5);                                       /* CA FIVE TS CM/GYMDT    */
    agc_status_t st = agc_waitlist_call(JTAGTIME, AGC_TASK_SETJTAG); /* status of agc_waitlist_call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_newphase(GRP1, 3, true); /* 1.3SPOT FOR SETJTAG    */
    agc_phase_newphase(GRP5, 037, false);

    return (AGC_STATUS_OK); /* 5.37 = CHEKAVEG        */
} /* readaccs_entry */

/**********************************************************************
 * CHEKAVEG / MAKESERV / AVEGOUT (p.825-826): recall READACCS while AVEGFLAG
 * is set, else make AVGEND the exit of this last pass; start SERVICER.
 *
 * FUNCTION NAME: chekaveg
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
static agc_status_t chekaveg(agc_avg_g_t *a)
{
    if (!agc_flag_test(AGC_FLAG_AVEGFLAG))
    {
        a->avgexit = AGC_JOB_AVGEND; /* AVEGOUT: AVOUTCAD      */
    }
    else
    {
        agc_status_t st = agc_waitlist_call(TWO_SECS, AGC_TASK_READACCS); /* status of agc_waitlist_call */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    agc_status_t st = agc_exec_findvac(AGC_PRIO20, AGC_JOB_SERVICER); /* MAKESERV */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_newphase(GRP5, 4, true); /* CS FOUR NEWPHASE 5     */
    agc_chan_or(011, (uint16_t) AGC_CH011_TESTCONN);

    return (AGC_STATUS_OK); /* TEST CONNECTOR ON */
}

/**********************************************************************
 * REDO5.5 (p.824): mark the reading done and continue (restart 5.5 enters
 * here, after PIPSDONE's phase change).
 *
 * FUNCTION NAME: redo5_5
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   a         agc_avg_g_t *  I/O  average-G block (DELV, RN1, VN1, GDT1/2,
 *                                 PIPAGE...)
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t redo5_5(agc_avg_g_t *a)
{
    a->pipage = 1; /* SHOW PIPS HAVE BEEN READ */
    a->pipctr = 2; /* PIPCTR FOR ONMNITOR    */

    if (agc_flag_test(AGC_FLAG_CM_DSTBY))
    {
        agc_status_t st = readaccs_entry(); /* status of readaccs_entry */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (chekaveg(a));
}

/**********************************************************************
 * PIPSDONE (p.824): CAF FIVE, DXCH -PHASE5, then REDO5.5.
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
    agc_phase_newphase(GRP5, 5, false); /* CAF FIVE DXCH -PHASE5  */

    return (redo5_5(a));
}

/**********************************************************************
 * REDO5.5 as the restart task of phase 5.5 (RESTART_TABLES p.217: -2CADR
 * REDO5.5, an immediate task).
 *
 * FUNCTION NAME: agc_task_redo5_5_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_redo5_5_csm(void)
{
    return (redo5_5(&agc_guid_state()->avg));
}

/**********************************************************************
 * CHEKAVEG as the restart task of phase 5.37 (RESTART_TABLES p.218).
 *
 * FUNCTION NAME: agc_task_chekaveg
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_chekaveg(void)
{
    return (chekaveg(&agc_guid_state()->avg));
}

/**********************************************************************
 * READACCS (p.824): waitlist task every 2 s.
 *
 * FUNCTION NAME: agc_task_readaccs_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_readaccs_csm(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    agc_guid_pipasr(&g->avg, true);

    return (pipsdone(&g->avg));
}

/**********************************************************************
 * REREADAC (p.833): restart task for group 5. Does nothing on the last
 * pass (PHASE5 = +0); otherwise re-establishes 1/PIPADT (LASTBIAS may have
 * been lost) and either redoes READACCS or finishes it at PIPSDONE.
 *
 * FUNCTION NAME: agc_task_rereadac_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
agc_status_t agc_task_rereadac_csm(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    if (agc_phase_get(GRP5) == 0U)
    { /* LAST PASS CHECK        */
        return (AGC_STATUS_OK); /* TASKOVER               */
    }

    agc_status_t st = agc_imu_set_pipa_interval(ONEPIPADT_2S); /* PRIO31 = 2.0 SEC B+8CS */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_guid_rereadac_resume(&g->avg); /* status of agc_guid_rereadac_resume */

    if (st == AGC_STATUS_NOT_FOUND)
    {
        return (agc_task_readaccs_csm()); /* NOT STARTED: BEGINNING */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st); /* PIPAGE out of range (agc_guid_rereadac_resume) */
    }

    return (pipsdone(&g->avg));
}

/**********************************************************************
 * QUIKREAD / CHKCTR (p.827): copy the PIPA counters (not zeroed) into
 * X,Y,ZPIPBUF and the previous values into X,Y,ZOLDBUF for the downlink;
 * repeat every 0.5 s while PIPCTR counts down (ONMNITOR).
 *
 * FUNCTION NAME: agc_task_quikread
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed); otherwise the status of the routine it
 *          continues in
 **********************************************************************/
agc_status_t agc_task_quikread(void)
{
    agc_guid_t    *g       = agc_guid_state(); /* guidance erasables */
    const agc_sp_t pipa[3] = {agc_erase_pipax(), agc_erase_pipay(), agc_erase_pipaz()};

    for (int i = 2; i >= 0; i--)
    { /* RUPTREG1 = 2, 1, 0     */
        g->csm.xoldbuf[i] = g->csm.xpipbuf[i];
        g->csm.xpipbuf[i] = pipa[i];
    }

    if (g->avg.pipctr > 0)
    { /* CCS PIPCTR             */
        g->avg.pipctr--; /* ONMNITOR: TS PIPCTR    */

        return (agc_waitlist_fixdelay(HALF_SEC, AGC_TASK_ONMNITOR));
    }

    return (AGC_STATUS_OK); /* TASKOVER               */
}

/**********************************************************************
 * ONMNITOR (p.827): the FIXDELAY return point of the 0.5 s loop.
 *
 * FUNCTION NAME: agc_task_onmnitor
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_task_onmnitor(void)
{
    if (!AGC_ASSERT(agc_guid_state()->avg.pipctr >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_quikread());
}

/**********************************************************************
 * NORMLIZE (p.831): copy RN1, VN1, PIPTIME1 into RN, VN, PIPTIME and
 * initialise UNITR, RMAG and GDT/2, GOBL/2 for the first AVERAGEG pass.
 *
 * FUNCTION NAME: agc_job_normlize_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (ENDOFJOB); otherwise the failing
 *          status of agc_guid_calcgrav() (BAD_ARG)
 **********************************************************************/
agc_status_t agc_job_normlize_csm(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    agc_guid_copycycle(AGC_VEHICLE_CSM, AGC_COPY_RVT); /* 14 REGS */
    agc_interp_t ip;                                   /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_guid_calcgrav(&ip, AGC_VEHICLE_CSM, agc_rendz_erasable()->body.rtx2 != 0, agc_erase_rn(), &g->avg); /* CALCGRAV at RN (moon: RTX2 = 2, LXC,1 SLOAD RTX2) */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g->avg.gdt_2  = g->avg.gdt1_2;  /* STOVL GDT/2            */
    g->avg.gobl_2 = g->avg.gobl1_2; /* STORE GOBL/2           */

    return (AGC_STATUS_OK); /* ENDOFJOB               */
}

/**********************************************************************
 * PIPCHECK (p.828): PIPA saturation test before compensation. CCS gives
 * |DELV| - 1, so a reading of more than 6399 pulses in 2 s is bad.
 *
 * FUNCTION NAME: pipas_saturated
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   delv      agc_vec_t   I    PIPA counts read this cycle, pulses (high
 *                              words)
 *
 * RETURNS: bool -- true when a PIPA reading exceeds MAXDELV + 1 pulses
 *          (the AGC's TC ALARM 0205 path)
 **********************************************************************/
static bool pipas_saturated(agc_vec_t delv)
{
    bool bad = false; /* a PIPA saturated */

    for (int i = 2; (i >= 0) && !bad; i--)
    { /* RUPTREG1 = 2, 1, 0     */
        agc_sp_t p   = agc_dp_high(delv.c[i]);       /* PIPA pulses, high word; scale 2^14 pulses */
        int      mag = (p < 0) ? -(int) p : (int) p; /* |DELV|, pulses          */

        bad = (mag != 0) && ((mag - 1 - MAXDELV) > 0); /* CCS; AD -MAXDELV; TC ALARM */
    }

    _Static_assert(MAXDELV == 6398, "AGC constant");

    return (bad);
}

/**********************************************************************
 * SERVICER (p.828): the 2-second job. A saturated PIPA skips compensation
 * and the DVTOTAL update and goes straight to AVERAGEG.
 *
 * FUNCTION NAME: agc_job_servicer_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the failing status of agc_imu_one_over_pipa()
 *          (BAILOUT 1202 from NOVAC, BAD_ARG); otherwise the status of the
 *          routine it continues in, agc_job_serv_dvtotup_csm() or (saturated
 *          PIPA) agc_job_serv_averageg_csm() (OK, BAD_ARG)
 **********************************************************************/
agc_status_t agc_job_servicer_csm(void)
{
    agc_inhint();

    bool bad = pipas_saturated(agc_erase_delv()); /* PIPCHECK */

    agc_relint();

    if (bad)
    {
        agc_alarm(AGC_ALARM_00205); /* SATURATED-PIPA ALARM   */

        return (agc_job_serv_averageg_csm());
    }

    serv_phase(PH_5_3_BJOB, AGC_JOB_SERV_DVTOTUP);    /* RESTART REREADAC+SERVICER */
    agc_vec_t    delv = agc_erase_delv();             /* DELV, PIPA counts in the high words (2^14 pulses) */
    agc_status_t st   = agc_imu_one_over_pipa(&delv); /* PIPA COMPENSATION      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_delv(delv);

    return (agc_job_serv_dvtotup_csm());
}

/**********************************************************************
 * DVTOTUP (p.828): DVTOTAL += |DELV| * KPIP1 (m/cs 2^7), then AVERAGEG.
 *
 * FUNCTION NAME: agc_job_serv_dvtotup_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of agc_job_serv_averageg_csm() (OK,
 *          BAD_ARG)
 **********************************************************************/
agc_status_t agc_job_serv_dvtotup_csm(void)
{
    agc_guid_t  *g = agc_guid_state(); /* guidance erasables */
    agc_interp_t ip;                   /* interpreter state */

    agc_interp_init(&ip);
    agc_dp_t  dv  = agc_dp_mul_trunc(agc_vec_abval(&ip, agc_erase_delv()), KPIP1_CM); /* |DELV| KPIP1, m/cs 2^7 */
    agc_ovf_t ovf = 0;                                                                /* overflow of the DAS (not tested, as in the AGC) */

    g->avg.dvtotal = agc_dp_add_raw(g->avg.dvtotal, dv, &ovf); /* DAS DVTOTAL */

    return (agc_job_serv_averageg_csm());
}

/**********************************************************************
 * AVERAGEG (p.828-829): CALCRVG, then the restart-protected copy.
 *
 * FUNCTION NAME: agc_job_serv_averageg_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the failing status of agc_guid_calcrvg()
 *          (BAD_ARG); otherwise the status of agc_job_serv_copy_csm() (OK,
 *          BAD_ARG)
 **********************************************************************/
agc_status_t agc_job_serv_averageg_csm(void)
{
    agc_guid_t *g = agc_guid_state(); /* guidance erasables */

    serv_phase(PH_5_3_B, AGC_JOB_SERV_AVERAGEG);
    agc_state_vector_t sv; /* state vector (RN, VN, PIPTIME) */

    agc_sv_get(AGC_SV_THIS, &sv);
    agc_mat_t    refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows (2^1) */
    agc_interp_t ip;                              /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_guid_calcrvg(&ip, AGC_VEHICLE_CSM, agc_rendz_erasable()->body.rtx2 != 0, &refsmmat, agc_erase_delv(), &sv, &g->avg); /* RTX2 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_serv_copy_csm());
}

/**********************************************************************
 * AVERAGEG copy (p.829): GENTRAN of 26 words RN1..GOBL1/2 under INHINT,
 * then the AVGEXIT transfer.
 *
 * FUNCTION NAME: agc_job_serv_copy_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of agc_csm_avgexit() (OK, BAD_ARG)
 **********************************************************************/
agc_status_t agc_job_serv_copy_csm(void)
{
    serv_phase(PH_5_3_B, AGC_JOB_SERV_COPY);
    agc_guid_copycycle(AGC_VEHICLE_CSM, AGC_COPY_RVTG);
    serv_phase(PH_5_3_B, AGC_JOB_SERV_EXIT);

    return (agc_csm_avgexit());
}

/**********************************************************************
 * DCA AVGEXIT / DXCH Z (p.829): continue this job at the routine named in
 * AVGEXIT (the C executive resumes the same job as that id).
 *
 * FUNCTION NAME: agc_csm_avgexit
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_csm_avgexit(void)
{
    agc_job_id_t exit_job = agc_guid_state()->avg.avgexit; /* AVGEXIT */

    if (!AGC_ASSERT(exit_job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_job_yield(exit_job));
}

/**********************************************************************
 * AVGEND (p.829-830): the final pass. Free-fall gyro compensation, DRIFT
 * flag, state vector back to the integration scale (AVETOMID), mark
 * counters and mark buffer cleared, test connector off, then CANV37.
 *
 * FUNCTION NAME: agc_job_avgend_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_avgend_csm(agc_job_ctx_t *ctx)
{
    /* CA PIPTIME +1 / TS OLDBT1 (= 1/PIPADT): time of the last reading.    */
    const agc_status_t st = agc_imu_set_pipa_interval(agc_dp_low(agc_erase_piptime())); /* 1/PIPADT store status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_set(AGC_FLAG_DRIFTFLG);
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_AVETOMID, AGC_TASK_NONE, (int16_t) AGC_PRIO20, false, 0U};

    agc_phase_change2(000005, 005022, &v); /* 5 OFF, 2.2 AVETOMID    */

    return (agc_job_avetomid_csm(ctx));
}

/**********************************************************************
 * AVGEND after the 2.2 restart point (p.829-830).
 *
 * FUNCTION NAME: agc_job_avetomid_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_avetomid_csm(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_integ_avetomid(&ip, agc_integ_shared_ctx(), ctx->id); /* CONVERT STATE VECTOR   */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_vhfcnt(0);                    /* ZERO MARK COUNTERS     */
    agc_erase_set_trkmkcnt(0);
    agc_imu_pipfree();
    agc_erase_set_mrkbuf2(0, (agc_sp_t) -0400); /* CS BIT9: INVALIDATE    */
    agc_chan_and(011, (uint16_t) ~(uint16_t) AGC_CH011_TESTCONN);
    agc_flag_clear(AGC_FLAG_CM_DSTBY);
    agc_flag_clear(AGC_FLAG_SERVICER_RUNNING);

    if (agc_flag_test(AGC_FLAG_RNDVZFLG))
    { /* RESTORE GROUP 1 + 2    */
        agc_phase_change2(000111, 000132, NULL); /* 1.11SPOT, 2.13SPOT     */
    }

    return (agc_job_change_program_continue()); /* POSTJUMP CANV37        */
}

/**********************************************************************
 * SERVEXIT (p.830, both vehicles): leave only 5.3 (REREADAC) protection
 * and end the job.
 *
 * FUNCTION NAME: agc_job_servexit
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when AVGEXIT
 *          holds no job
 **********************************************************************/
agc_status_t agc_job_servexit(void)
{
    if (!AGC_ASSERT(agc_guid_state()->avg.avgexit != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(PH_5_3_A);

    return (AGC_STATUS_OK);
}
