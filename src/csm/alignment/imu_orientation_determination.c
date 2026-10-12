/**********************************************************************
 * FILE NAME: imu_orientation_determination.c
 *
 * PURPOSE:
 *   P51 IMU orientation determination and P53, its backup (P53 EQUALS
 *   P51), CSM.  With the IMU on, the crew may coarse align to zero gimbal
 *   angles (ENTER on V50N25 00015), then marks two stars (R53, or R56 in
 *   P53).  The two observed lines of sight (present SM) and the two
 *   catalogue lines of sight (reference) give REFSMMAT directly
 *   (AXISGEN).
 *
 *   STARAD = 1st observed (SM)     6D  = 1st catalogue (PLANVEC, ref)
 *   STARAD +6 = 2nd observed       12D = 2nd catalogue
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 766-769 (P51, P51A, P51AA, P51B, P51C, P51C.1, P51C.2, P51D, P51E, P51F, P51G)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   IMODES30      uint16_t  I    bits           bit 9: IMU off
 *   THETAD        uint16_t  O    CDU words      zeroed for the coarse align
 *   REFSMMAT      agc_mat_t O    half-unit      agc_erase_set_refsmmat()
 *   STARIND, MARKINDX, STARSAV1/2, PLANVEC, STARAD, 6D, 12D, XDC: agc_align_ram()
 *   REFSMFLG, TARG2FLG, FREEFLAG  flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_exec_gotopooh     executive: jobs, VAC areas, sleep / wake (exec.h)
 *   agc_imu_*             IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_phase_checkmm     restart phase tables, CHECKMM (exec.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_alarm             ALARM (exec.h)
 *   agc_csm_*             CSM optics and sighting routines (alignment.h)
 *   agc_sxtmark_mkreleas  SXTMARK routines (alignment.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarms: ALM_IMU_OFF 0210: OCT 210: IMU not operating.
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   Restart: the C restart point of P51E is P51C (the mark is taken
 *   again); the AGC restarts at P51E (MKRELEAS).
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/programs/alignment.h"

#define IMODES30_IMU_OFF       ((uint16_t) 0400)         /* BIT9: IMU not operating        */
#define ALM_IMU_OFF            ((agc_alarm_code_t) 0210) /* OCT 210: IMU not operating */
#define CHK_ACQUIRE            ((uint16_t) 015)          /* PRFMSTAQ = OCT15               */
#define PH_ORIENTATION         ((uint16_t) 05024)        /* OCT 05024, OCT 13000           */
#define PH_ORIENTATION_RESTART ((uint16_t) 00014)        /* OCT 00014                      */
#define VN_GIMBALS             ALIGN_VN(6, 22)           /* V6N22  VN 0622                 */
#define VN_COARSE_VERB         ALIGN_VN(41, 0)           /* V41K   VN 4100                 */
#define MM_BACKUP_ORIENT       ((agc_sp_t) 53)           /* CHECKMM 53                     */

/**********************************************************************
 * AGC: none (TC PHASCHNG with a type-C code, P51-P53.agc)
 *
 * PHASCHNG OCT 05024, OCT 13000: restart group 4 at the given job, PRIO13.
 *
 * FUNCTION NAME: orientation_phase
 *
 * ARGUMENTS:
 *   Argument    Type          I/O  Description (scale)
 *   ----------  ------------  ---  --------------------------------------
 *   restart_at  agc_job_id_t  I    job a restart re-enters
 *
 * RETURNS: none
 **********************************************************************/
static void orientation_phase(agc_job_id_t restart_at)
{
    align_phase_job(PH_ORIENTATION, restart_at, ALIGN_PRIO13);
}

/**********************************************************************
 * AGC: P51 (Comanche055 P51-P53.agc p. 766)
 *
 * P51 / P53: alarm 210 and P00 when the IMU is off (IMODES30 bit 9);
 * otherwise R02ZERO (SET IMUSE) and P51AA.
 *
 * FUNCTION NAME: agc_job_csm_imu_orientation_determination
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
agc_status_t agc_job_csm_imu_orientation_determination(agc_job_ctx_t *ctx)
{
    if ((agc_erase_imodes30() & IMODES30_IMU_OFF) != 0u)
    {
        agc_alarm(ALM_IMU_OFF);

        return (agc_exec_gotopooh());
    }

    const agc_status_t st = agc_imu_set_in_use(); /* R02ZERO  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_csm_orientation_acquire_display(ctx));
}

/**********************************************************************
 * AGC: P51AA (Comanche055 P51-P53.agc p. 767)
 *
 * P51AA: V50N25 R1 00015; V34 P00, V33 P51B (mark), ENTER coarse align.
 *
 * FUNCTION NAME: agc_job_csm_orientation_acquire_display
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
agc_status_t agc_job_csm_orientation_acquire_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_ACQUIRE, AGC_JOB_GOTOPOOH, AGC_JOB_CSM_ORIENTATION_MARK_SETUP, AGC_JOB_CSM_ORIENTATION_COARSE));
}

/**********************************************************************
 * AGC: P51AA (Comanche055 P51-P53.agc p. 767)
 *
 * P51AA ENTER: PHASCHNG, THETAD = 0, GODSPRET V06N22 (job continues).
 *
 * FUNCTION NAME: agc_job_csm_orientation_coarse
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
agc_status_t agc_job_csm_orientation_coarse(agc_job_ctx_t *ctx)
{
    orientation_phase(AGC_JOB_CSM_ORIENTATION_COARSE);

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_thetad(i, 0); /* P51ZERO  */
    }

    return (align_flash(ctx, AGC_DSKY_GODSPRET, VN_GIMBALS, AGC_JOB_CSM_ORIENTATION_COARSE_VERB, AGC_JOB_CSM_ORIENTATION_COARSE_VERB, AGC_JOB_CSM_ORIENTATION_COARSE_VERB));
}

/**********************************************************************
 * AGC: GODSPRET (Comanche055 DISPLAY_INTERFACE_ROUTINES.agc p. 1468)
 *
 * GODSPRET V41 (coarse align verb); then IMUCOARS and IMUSTALL.
 *
 * FUNCTION NAME: agc_job_csm_orientation_coarse_verb
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
agc_status_t agc_job_csm_orientation_coarse_verb(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GODSPRET, VN_COARSE_VERB, AGC_JOB_CSM_ORIENTATION_COARSE_START, AGC_JOB_CSM_ORIENTATION_COARSE_START, AGC_JOB_CSM_ORIENTATION_COARSE_START));
}

/**********************************************************************
 * AGC: IMUCOARS (Comanche055 IMU_MODE_SWITCHING_ROUTINES.agc p. 1423)
 *
 * IMUCOARS to THETAD (all zero) and IMUSTALL (CURTAINS on a bad end).
 *
 * FUNCTION NAME: agc_job_csm_orientation_coarse_start
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_csm_orientation_coarse_start(void)
{
    uint16_t thetad[3]; /* THETAD X, Y, Z words */

    for (int i = 0; i < 3; i++)
    {
        thetad[i] = agc_erase_thetad(i);
    }

    agc_status_t st = agc_imu_coarse(thetad); /* IMUCOARS */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_CSM_ORIENTATION_COARSE_DONE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (align_goto(AGC_JOB_CSM_ORIENTATION_COARSE_DONE));
}

/**********************************************************************
 * AGC: IMUFIN20 (Comanche055 IMU_MODE_SWITCHING_ROUTINES.agc p. 1442)
 *
 * After the coarse align: IMUFIN20 and IMUSTALL.
 *
 * FUNCTION NAME: agc_job_csm_orientation_coarse_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_csm_orientation_coarse_done(void)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_IMU); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    agc_status_t st = agc_imu_fine(); /* IMUFIN20 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_CSM_ORIENTATION_FINE_DONE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (align_goto(AGC_JOB_CSM_ORIENTATION_FINE_DONE));
}

/**********************************************************************
 * AGC: PHASCHNG (Comanche055 PHASE_TABLE_MAINTENANCE.agc p. 1409)
 *
 * Coarse align done: PHASCHNG, recycle to P51AA for the fine align.
 *
 * FUNCTION NAME: agc_job_csm_orientation_fine_done
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
agc_status_t agc_job_csm_orientation_fine_done(agc_job_ctx_t *ctx)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_IMU); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    orientation_phase(AGC_JOB_CSM_ORIENTATION_ACQUIRE_DISPLAY);

    return (agc_job_csm_orientation_acquire_display(ctx));
}

/**********************************************************************
 * AGC: P51B (Comanche055 P51-P53.agc p. 768)
 *
 * P51B: PHASCHNG 00014; STARIND = 0; SET1/PDT, GCOMP = 0, SET DRIFTFLG;
 * CLEAR TARG2FLG (star, not landmark); MARKINDX = 1; P51C.
 *
 * FUNCTION NAME: agc_job_csm_orientation_mark_setup
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
agc_status_t agc_job_csm_orientation_mark_setup(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    align_phase4(PH_ORIENTATION_RESTART);
    ram->starind  = 0;
    align_set1_pdt();
    agc_flag_clear(AGC_FLAG_TARG2FLG);
    ram->markindx = 1;

    return (agc_job_csm_orientation_mark_star(ctx));
}

/**********************************************************************
 * AGC: P51C (Comanche055 P51-P53.agc p. 768)
 *
 * P51C: PHASCHNG; P53 marks with R56, P51 with R53; both to P51C.2.
 *
 * FUNCTION NAME: agc_job_csm_orientation_mark_star
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
agc_status_t agc_job_csm_orientation_mark_star(agc_job_ctx_t *ctx)
{
    orientation_phase(AGC_JOB_CSM_ORIENTATION_MARK_STAR);

    if (agc_phase_checkmm(MM_BACKUP_ORIENT))
    {
        return (agc_csm_alternate_los_sighting_mark(ctx, AGC_JOB_CSM_ORIENTATION_MARKED));
    }

    return (agc_csm_sighting_mark(AGC_JOB_CSM_ORIENTATION_MARKED));
}

/**********************************************************************
 * AGC: P51E (Comanche055 P51-P53.agc p. 769)
 *
 * P51E: PHASCHNG; MKRELEAS; after the first star STARIND = 1 and P51C
 * again; after the second, P51F (PLANET for the second catalogue LOS).
 *
 * FUNCTION NAME: orientation_after_mark
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t orientation_after_mark(const agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    orientation_phase(AGC_JOB_CSM_ORIENTATION_MARK_STAR);
    const agc_status_t st = agc_sxtmark_mkreleas(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (ram->starind == 0)
    {
        ram->starind = 1;

        return (align_goto(AGC_JOB_CSM_ORIENTATION_MARK_STAR));
    }

    orientation_phase(AGC_JOB_CSM_ORIENTATION_SECOND_STAR);

    return (align_planet(ctx, ram->tsight, AGC_JOB_CSM_ORIENTATION_SECOND_STAR));
}

/**********************************************************************
 * AGC: P51C.2 (Comanche055 P51-P53.agc p. 768)
 *
 * P51C.2: SXTSM.  First star (P51D): STARSAV1 = LOS, PLANET at TSIGHT ->
 * PLANVEC.  Second star: STARSAV2 = LOS.  Then P51E.
 *
 * FUNCTION NAME: agc_job_csm_orientation_marked
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
agc_status_t agc_job_csm_orientation_marked(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    agc_vec_t los; /* SXTSM line of sight, present SM, half-unit (2^1) */
    agc_csm_sxtsm_marks(&ip, &los);

    if (ram->starind != 0)
    {
        ram->starsav2 = los;

        return (orientation_after_mark(ctx));
    }

    ram->starsav1 = los; /* P51D     */

    return (align_planet(ctx, ram->tsight, AGC_JOB_CSM_ORIENTATION_FIRST_STAR));
}

/**********************************************************************
 * AGC: P51D (Comanche055 P51-P53.agc p. 768)
 *
 * P51D after PLANET: PLANVEC = the first catalogue LOS; P51E.
 *
 * FUNCTION NAME: agc_job_csm_orientation_first_star
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
agc_status_t agc_job_csm_orientation_first_star(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->planvec = ram->planet_los;

    return (orientation_after_mark(ctx));
}

/**********************************************************************
 * AGC: P51F (Comanche055 P51-P53.agc p. 769)
 *
 * P51F after PLANET: 12D = 2nd catalogue, 6D = PLANVEC, STARAD = STARSAV1,
 * STARAD +6 = STARSAV2; CHKSDATA (R54).
 *
 * FUNCTION NAME: agc_job_csm_orientation_second_star
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
agc_status_t agc_job_csm_orientation_second_star(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->vac12     = ram->planet_los;
    ram->vac6      = ram->planvec;
    ram->starad[0] = ram->starsav1;
    ram->starad[1] = ram->starsav2;

    return (agc_align_sighting_data_display(ctx, AGC_JOB_CSM_ORIENTATION_DATA_CHECKED));
}

/**********************************************************************
 * AGC: CHKSDATA (Comanche055 P51-P53.agc p. 760)
 *
 * After CHKSDATA: rejected -> P51AA; accepted (P51G): AXISGEN gives
 * REFSMMAT in XDC; CLEAR REFSMFLG, MATMOVE, SET REFSMFLG, ENDP50S.
 *
 * FUNCTION NAME: agc_job_csm_orientation_data_checked
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
agc_status_t agc_job_csm_orientation_data_checked(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return (agc_job_csm_orientation_acquire_display(ctx));
    }

    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_vec_t a[2] = {ram->starad[0], ram->starad[1]};
    const agc_vec_t b[2] = {ram->vac6, ram->vac12};
    agc_status_t    st   = agc_align_fine_solve(&ip, a, b, &ram->xdc); /* AXISGEN */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_clear(AGC_FLAG_REFSMFLG);
    agc_erase_set_refsmmat(ram->xdc); /* MATMOVE  */
    agc_flag_set(AGC_FLAG_REFSMFLG);

    return (agc_exec_gotopooh()); /* ENDP50S  */
}
