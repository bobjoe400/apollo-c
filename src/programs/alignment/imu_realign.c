/**********************************************************************
 * FILE NAME: imu_realign.c
 *
 * PURPOSE:
 *   The part of P52 (IMU realign) common to both vehicles: PROG52, the
 *   option display P52B, the time display P52T, the nominal orientation
 *   S52.3, the gimbal-angle display P52D, the coarse-align choice
 *   COARSTYP, CAL53A (coarse / fine align to XSMD) and the gyro-torque
 *   coarse align GYCRS / GYCOARS.  The differences (CM phase and flag
 *   clears in PROG52, the option-to-routine map, TALIGN in the LM, the
 *   ends of CAL53A and GYCRS) are behind agc_state_vehicle() tests.
 *
 *   OPTION2 (OPTION1 +1) holds the alignment option: 1 preferred, 2
 *   nominal, 3 REFSMMAT, 4 landing site (option 4 & 3 = 0).
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 739-744, 761 (PROG52 .. GYCRS, CAL53A)
 *   Luminary099 P51-P53.agc pp. 927-933, 944, 947 (PROG52 .. GYCOARS, CAL53A)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   OPTION1 +1    agc_sp_t  I/O  unscaled       OPTION2: the option code
 *   DSPTEM1       uint16_t  I/O  cs 2^28 (DP)   alignment time (CM)
 *   TALIGN, XSMD, XSM, XDC, OGC  I/O          agc_align_ram()
 *   REFSMMAT      agc_mat_t I/O  half-unit      agc_erase_refsmmat()
 *   PFRATFLG, REFSMFLG, DRIFTFLG, UPDATFLG, TRACKFLG flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_dsptem       DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_imu_*             IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_state_vehicle     vehicle state (state.h)
 *   agc_exec_jump         the vehicle's own P52 steps, by registry id:
 *                         AGC_JOB_IMU_REALIGN_STARS (CM P52C, LM R51),
 *                         _RECHECK (R51K), _LANDING_SITE (P52LS)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   P52T tests only the major word of DSPTEM1 (finding A15). R02BOTH
 *   POODOO ends the job; other statuses are passed up.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "internal.h"

#define PH_REALIGN ((uint16_t) 00254)    /* PROG52 (CM): group 4 phase 25    */
#define PH_OPTION_RETURN \
    ((uint16_t) 00014)                   /* P52B display return: phase 1     */
#define VN_ALIGN_TIME    ALIGN_VN(6, 34) /* VN 0634: TALIGN / time           */
#define VN_GIMBAL_ANGLES ALIGN_VN(6, 22) /* VN 00622: gimbal angles THETAD   */
#define VN_MONITOR_GIMBALS \
    ALIGN_VN(16, 20)                     /* VN 1620: monitor the gimbals     */
#define OPT_CODE_1   ((uint16_t) 1)      /* GOPERF4R with BIT1: V04N06       */
#define CHK_OCT13    ((uint16_t) 013)    /* OCT13: coarse align checklist    */
#define OPTION2_PREF ((agc_sp_t) 1)      /* BIT1: preferred                  */
#define OPTION2_NOM  ((agc_sp_t) 2)      /* BIT2: nominal                    */
#define OPTION_MASK3 3                   /* MASK THREE                       */

/**********************************************************************
 * AGC: none (vehicle test)
 *
 * True on the CSM (Comanche) rope.
 *
 * FUNCTION NAME: is_csm
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true for the CSM
 **********************************************************************/
static bool is_csm(void)
{
    return (agc_state_vehicle() == AGC_VEHICLE_CSM);
}

/**********************************************************************
 * AGC: SET1/PDT (Comanche055 P51-P53.agc p. 769)
 *
 * CM SET1/PDT, GCOMP = 0, SET DRIFTFLG; LM NCOARSE (the same plus PIPAs).
 *
 * FUNCTION NAME: drift_compensation_restart
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the SET1/PDT or NCOARSE status
 **********************************************************************/
static agc_status_t drift_compensation_restart(void)
{
    agc_status_t st; /* SET1/PDT or NCOARSE status */

    if (is_csm())
    {
        st = align_set1_pdt();
    }
    else
    {
        st = align_ncoarse();
    }

    return (st);
}

/**********************************************************************
 * AGC: OPTION2 (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 78; Luminary099
 *      ERASABLE_ASSIGNMENTS.agc p. 104)
 *
 * OPTION2 (OPTION1 +1).
 *
 * FUNCTION NAME: option2
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- OPTION2 (OPTION1 +1), the alignment option word
 **********************************************************************/
static agc_sp_t option2(void)
{
    const agc_sp_t v = agc_erase_option1(1); /* OPTION2, unscaled option code */

    return (v);
}

/**********************************************************************
 * PROG52 alias P54 (CM) / PROG52 (LM) (Comanche p. 739, Luminary p. 927):
 * CM: restart phase 4.25, clear UPDATFLG and TRACKFLG.  Both: R02BOTH;
 * OPTION2 = preferred if PFRATFLG is set, else nominal; then P52B.
 *
 * FUNCTION NAME: agc_job_imu_realign
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_imu_realign(agc_job_ctx_t *ctx)
{
    if (is_csm())
    {
        align_phase4(PH_REALIGN);
        agc_flag_clear(AGC_FLAG_UPDATFLG);
        agc_flag_clear(AGC_FLAG_TRACKFLG);
    }

    const agc_status_t st = agc_imu_status_check(); /* R02BOTH   */

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_POODOO) ? AGC_STATUS_OK : st); /* alarm 210/220, P00 started */
    }

    agc_erase_set_option1(1, agc_flag_test(AGC_FLAG_PFRATFLG) ? OPTION2_PREF : OPTION2_NOM); /* PFRATFLG: preferred, else nominal */

    return (agc_job_imu_realign_option_display(ctx));
}

/**********************************************************************
 * AGC: P52B (Comanche055 P51-P53.agc p. 739; Luminary099 P51-P53.agc p.
 *      928)
 *
 * P52B: GOPERF4R (flash V04N06, option code 00001, OPTION2); V34
 * terminates, V33 goes on with the option, ENTER (a new option) re-flashes.
 * The immediate return sets restart phase 4.1 and ends the job.
 *
 * FUNCTION NAME: agc_job_imu_realign_option_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_imu_realign_option_display(agc_job_ctx_t *ctx)
{
    const agc_status_t st = align_flash(ctx, AGC_DSKY_GOPERF4R, OPT_CODE_1, AGC_JOB_GOTOPOOH, AGC_JOB_IMU_REALIGN_OPTION, AGC_JOB_IMU_REALIGN_OPTION_DISPLAY); /* status of the call */

    if (st == AGC_STATUS_OK)
    {
        align_phase4(PH_OPTION_RETURN);
    }

    return (st); /* ENDOFJOB  */
}

/**********************************************************************
 * AGC: P52B (Comanche055 P51-P53.agc p. 739; Luminary099 P51-P53.agc p.
 *      928)
 *
 * P52B +9 (V33 on V04N06): branch on OPTION2 & 3. CM:  0 (landing site)
 * P52T, 1 P52J -> P52D, 2 P52T, 3 P52C. LM:  0 OPT4 (TLAND shown in
 * VN_ALIGN_TIME), 1 P52H -> P52D, 2 P52T, 3 P52E -> P52F (R51, then P52OUT
 * = GOTOPOOH).
 *
 * FUNCTION NAME: agc_job_imu_realign_option
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_imu_realign_option(agc_job_ctx_t *ctx)
{
    const int opt = (int) (option2() & OPTION_MASK3); /* LAT-LONG options */

    if (opt == 1)
    {
        return (agc_job_realign_gimbal_display(ctx)); /* P52J / P52H    */
    }

    if (opt == 2)
    {
        return (agc_job_realign_time(ctx));
    }

    if (is_csm())
    {
        return ((opt == 0) ? agc_job_realign_time(ctx) : agc_exec_jump(ctx, AGC_JOB_IMU_REALIGN_STARS)); /* landing site (P52T) or REFSMMAT (P52C) */
    }

    if (opt == 0)
    { /* OPT4           */
        align_set_dsptem1(agc_erase_tland());

        return (agc_job_realign_time_flash(ctx));
    }

    agc_align_ram()->qmaj = AGC_JOB_GOTOPOOH; /* P52OUT         */

    return (agc_exec_jump(ctx, AGC_JOB_IMU_REALIGN_STARS)); /* P52E, P52F: R51 */
}

/**********************************************************************
 * AGC: P52T (Comanche055 P51-P53.agc p. 739; Luminary099 P51-P53.agc p.
 *      928)
 *
 * P52T: DSPTEM1 = -0 (0 here: both tests treat +-0 alike), then
 * VN_ALIGN_TIME.
 *
 * FUNCTION NAME: agc_job_realign_time
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_realign_time(agc_job_ctx_t *ctx)
{
    align_set_dsptem1(0);

    return (agc_job_realign_time_flash(ctx));
}

/**********************************************************************
 * AGC: P52T (Comanche055 P51-P53.agc p. 739; Luminary099 P51-P53.agc p.
 *      928)
 *
 * P52T +3: flash VN_ALIGN_TIME (time of alignment); ENTER re-flashes.
 *
 * FUNCTION NAME: agc_job_realign_time_flash
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_realign_time_flash(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALIGN_TIME, AGC_JOB_GOTOPOOH, AGC_JOB_REALIGN_TIME_PROCEED, AGC_JOB_REALIGN_TIME_FLASH));
}

/**********************************************************************
 * AGC: P52T (Comanche055 P51-P53.agc p. 739; Luminary099 P51-P53.agc p.
 *      928)
 *
 * P52T +10 (V33): a zero time means now.  CM keeps the time in DSPTEM1; LM:
 * TALIGN = DSPTEM1, or TIME2/1 if it is zero or negative (P52V). Both tests
 * (DCA then BZF / BZMF) look at the major word only, so a time under 2^14
 * cs also means now.  OPTION2 bit 2 (nominal) -> S52.3, else the landing
 * site (P52LS).
 *
 * FUNCTION NAME: agc_job_realign_time_proceed
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_realign_time_proceed(agc_job_ctx_t *ctx)
{
    const agc_dp_t t       = align_dsptem1();                      /* DSPTEM1 as a DP, cs 2^28 */
    const agc_sp_t major   = align_word_to_sp(agc_dsky_dsptem(0)); /* A after DCA DSPTEM1 (major word), scale 2^14 cs per unit */
    const bool     nominal = (option2() & OPTION2_NOM) != 0;       /* OPTION2 bit 2: nominal orientation */

    if (is_csm())
    {
        if (major == 0)
        { /* BZF       */
            align_set_dsptem1(agc_rtb_loadtime());
        }

        return (nominal ? agc_job_realign_nominal_orientation(ctx) : agc_exec_jump(ctx, AGC_JOB_IMU_REALIGN_LANDING_SITE)); /* S52.3 or P52LS */
    }

    agc_align_ram()->talign = (major <= 0) ? agc_rtb_loadtime() : t; /* BZMF   */

    return (nominal ? agc_job_realign_nominal_orientation(ctx) : agc_exec_jump(ctx, AGC_JOB_IMU_REALIGN_LANDING_SITE)); /* S52.3 or P52LS */
}

/**********************************************************************
 * AGC: S52.3 (Comanche055 P51-P53.agc p. 780; Luminary099 P51-P53.agc
 *      p. 954)
 *
 * CALL S52.3 (nominal orientation at DSPTEM1 (CM) / TALIGN (LM)), then
 * P52D.  The integration may put the job to sleep in INTSTALL; it then
 * resumes here.
 *
 * FUNCTION NAME: agc_job_realign_nominal_orientation
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_realign_nominal_orientation(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();                          /* alignment erasables */
    const agc_dp_t   t   = is_csm() ? align_dsptem1() : ram->talign; /* alignment time, cs 2^28, per vehicle */
    agc_interp_t     ip;                                             /* interpreter state */

    agc_interp_init(&ip);
    const agc_status_t st = agc_align_nominal_orientation(&ip, t, AGC_JOB_REALIGN_NOMINAL_ORIENTATION, &ram->xsmd); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL sleep   */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_realign_gimbal_display(ctx));
}

/**********************************************************************
 * AGC: S52.2 (Comanche055 P51-P53.agc p. 774; Luminary099 P51-P53.agc
 *      p. 953)
 *
 * CALL S52.2 on the present CDUs: XSM = XSMD, THETAD, XNB (reference).
 *
 * FUNCTION NAME: realign_gimbal_angles
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t realign_gimbal_angles(void)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    uint16_t cdu[3]; /* CDUX, CDUY, CDUZ words */

    align_read_cdus(cdu);
    const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */
    uint16_t        thetad[3];                       /* THETAD X, Y, Z words */

    return (agc_align_gimbal_angles(&ip, &ram->xsmd, &refsmmat, cdu, &ram->xsm, &ram->xnb, thetad));
}

/**********************************************************************
 * AGC: P52D (Comanche055 P51-P53.agc p. 740; Luminary099 P51-P53.agc p.
 *      930)
 *
 * P52D: S52.2, flash VN_GIMBAL_ANGLES; V33 COARSTYP, ENTER (vehicle moved)
 * P52D.
 *
 * FUNCTION NAME: agc_job_realign_gimbal_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_realign_gimbal_display(agc_job_ctx_t *ctx)
{
    const agc_status_t st = realign_gimbal_angles(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_GIMBAL_ANGLES, AGC_JOB_GOTOPOOH, AGC_JOB_COARSTYP, AGC_JOB_REALIGN_GIMBAL_DISPLAY));
}

/**********************************************************************
 * AGC: COARSTYP (Comanche055 P51-P53.agc p. 741; Luminary099 P51-P53.agc p.
 *      931)
 *
 * COARSTYP: V50N25 R1 00013; V33 normal coarse (CAL53A), V32 gyro torque.
 *
 * FUNCTION NAME: agc_job_coarstyp
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_coarstyp(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_OCT13, AGC_JOB_GOTOPOOH, AGC_JOB_COARSE_ALIGN, AGC_JOB_GYCRS));
}

/**********************************************************************
 * CAL53A (Comanche p. 761, Luminary p. 947): a last S52.2, read the CDUs;
 * if any gimbal is more than 1 degree from THETAD (CALOOP), restart phase
 * 4.2 and COARSE, then (CM) SET1/PDT, GCOMP = 0, DRIFTFLG / (LM) NCOARSE;
 * in either case FINEONLY.
 *
 * FUNCTION NAME: agc_job_coarse_align
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_coarse_align(agc_job_ctx_t *ctx)
{
    agc_status_t st = realign_gimbal_angles(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    uint16_t cdu[3];    /* CDUX, CDUY, CDUZ words */
    uint16_t thetad[3]; /* THETAD X, Y, Z words */

    align_read_cdus(cdu); /* RDCDUS    */

    for (int i = 0; i < 3; i++)
    {
        thetad[i] = agc_erase_thetad(i);
    }

    if (!agc_align_coarse_needed(thetad, cdu))
    {
        return (agc_job_coarse_align_fineonly(ctx)); /* CALOOP1   */
    }

    align_phase_job(ALIGN_PH_04024, AGC_JOB_COARSE_ALIGN, ctx->prio); /* COARFINE */

    return (align_coarse(AGC_JOB_COARSE_ALIGN_COARSED));
}

/**********************************************************************
 * AGC: COARSE (Luminary099 P51-P53.agc p. 952)
 *
 * After COARSE in CAL53A: drift compensation restarted, then FINEONLY.
 *
 * FUNCTION NAME: agc_job_coarse_align_coarsed
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_coarse_align_coarsed(agc_job_ctx_t *ctx)
{
    const agc_status_t st = drift_compensation_restart(); /* SET1/PDT, NCOARSE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_coarse_align_fineonly(ctx));
}

/**********************************************************************
 * AGC: FINEONLY (Comanche055 P51-P53.agc p. 762; Luminary099 P51-P53.agc p.
 *      947)
 *
 * FINEONLY: REFSMMAT = XSM (MATMOVE).  CM CAL53RET: SET REFSMFLG, P52C. LM
 * COARSRET: SET REFSMFLG, CLEAR PFRATFLG, P52F (R51, then P52OUT).
 *
 * FUNCTION NAME: agc_job_coarse_align_fineonly
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_coarse_align_fineonly(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    agc_erase_set_refsmmat(ram->xsm); /* MATMOVE   */
    agc_flag_set(AGC_FLAG_REFSMFLG);

    if (is_csm())
    {
        return (agc_exec_jump(ctx, AGC_JOB_IMU_REALIGN_STARS)); /* P52C */
    }

    agc_flag_clear(AGC_FLAG_PFRATFLG);
    ram->qmaj = AGC_JOB_GOTOPOOH; /* P52OUT    */

    return (agc_exec_jump(ctx, AGC_JOB_IMU_REALIGN_STARS)); /* R51 */
}

/**********************************************************************
 * GYCRS (Comanche p. 741) / COARSTYP V32 + GYCOARS (Luminary pp. 931, 944):
 * XDC.. = UNIT(REFSMMAT XSMD..), the desired SM wrt the present SM;
 * CALCGTA; CLEAR DRIFTFLG, REFSMFLG; monitor VN_MONITOR_GIMBALS (GODSPR);
 * torque.
 *
 * FUNCTION NAME: agc_job_gycrs
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_gycrs(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */

    agc_align_gyro_coarse_dc(&ip, &ram->xsmd, &refsmmat, &ram->xdc);
    agc_imu_calcgta(&ip, &ram->xdc, agc_imu_ogc());                         /* CALCGTA   */
    agc_flag_clear(AGC_FLAG_DRIFTFLG);                                      /* flag ids are valid  */
    agc_flag_clear(AGC_FLAG_REFSMFLG);
    agc_status_t st = align_show(ctx, AGC_DSKY_GODSPR, VN_MONITOR_GIMBALS); /* GODSPR    */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_gycrs_pulse(ctx));
}

/**********************************************************************
 * AGC: GYCRS (Comanche055 P51-P53.agc p. 741)
 *
 * GYCRS: IMUPULSE (R55CDR = OGC) and IMUSTALL; resumed if gyros busy.
 *
 * FUNCTION NAME: agc_job_gycrs_pulse
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_gycrs_pulse(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_GYCRS_PULSE); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_GYCRS_PULSED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (agc_job_gycrs_torqued(ctx));
}

/**********************************************************************
 * AGC: GYCRS (Comanche055 P51-P53.agc p. 741)
 *
 * GYCRS: woken from the IMUSTALL.
 *
 * FUNCTION NAME: agc_job_gycrs_pulsed
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_gycrs_pulsed(agc_job_ctx_t *ctx)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_IMU); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    return (agc_job_gycrs_torqued(ctx));
}

/**********************************************************************
 * AGC: GYCRS (Comanche055 P51-P53.agc p. 741)
 *
 * GYCRS after torquing: phase 4.2; REFSMMAT = XSMD; CLEAR PFRATFLG, SET
 * REFSMFLG; CM SET1/PDT, GCOMP = 0, DRIFTFLG / LM NCOARSE; GOTO R51K (the
 * LM returns from R51K's ENTER to QMAJ = P52OUT).
 *
 * FUNCTION NAME: agc_job_gycrs_torqued
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_gycrs_torqued(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    align_phase_job(ALIGN_PH_04024, AGC_JOB_GYCRS_TORQUED, ctx->prio);
    agc_erase_set_refsmmat(ram->xsmd); /* MATMOVE   */
    agc_flag_clear(AGC_FLAG_PFRATFLG);
    agc_flag_set(AGC_FLAG_REFSMFLG);

    const agc_status_t st = drift_compensation_restart(); /* SET1/PDT, NCOARSE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (is_csm())
    {
        return (agc_exec_jump(ctx, AGC_JOB_IMU_REALIGN_RECHECK)); /* R51K */
    }

    ram->qmaj = AGC_JOB_GOTOPOOH; /* P52OUT    */

    return (agc_exec_jump(ctx, AGC_JOB_IMU_REALIGN_RECHECK)); /* R51K */
}
