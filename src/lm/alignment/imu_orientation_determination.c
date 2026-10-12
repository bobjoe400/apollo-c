/**********************************************************************
 * FILE NAME: imu_orientation_determination.c
 *
 * PURPOSE:
 *   P51 IMU orientation determination (LM) and IMUCHK.  As in the CSM: an
 *   optional coarse align to zero gimbal angles, then two AOTMARK
 *   sightings; the two observed lines of sight (present SM, averaged by
 *   AOTMARK into STARAD +6) and the two catalogue lines of sight
 *   (reference) give REFSMMAT through AXISGEN.
 *
 *   STARAD    = 1st observed (STARSAV1)   6D  = 1st catalogue (PLANVEC)
 *   STARAD +6 = 2nd observed (STARSAV2)   12D = 2nd catalogue
 *
 * AGC SOURCE:
 *   Luminary099 P51-P53.agc pp. 949-951 (P51, P51B, P51C, P51D, P51E, P51G), p. 982 (IMUCHK)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   IMODES30      uint16_t  I    bits           bit 9: IMU off
 *   THETAD        uint16_t  O    CDU words      zeroed for the coarse align
 *   REFSMMAT      agc_mat_t O    half-unit      agc_erase_set_refsmmat()
 *   STARIND, STARSAV1/2, PLANVEC, STARAD, 6D, 12D, XDC: agc_align_ram()
 *   IMUSE, REFSMFLG, FREEFLAG  flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_exec_gotopooh     executive: jobs, VAC areas, sleep / wake (exec.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_alarm             ALARM (exec.h)
 *   agc_aotmark           AOTMARK routines (alignment.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarms: ALM_IMU_OFF 0210: OCT 210: IMU not operating.
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   THETAD is zeroed on the coarse-align path as the AGC does (finding
 *   A13).
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/programs/alignment.h"

#define IMODES30_IMU_OFF       ((uint16_t) 0400)         /* BIT9                           */
#define ALM_IMU_OFF            ((agc_alarm_code_t) 0210) /* OCT 210: IMU not operating */
#define CHK_ACQUIRE            ((uint16_t) 015)          /* OCT15                          */
#define PH_ORIENTATION         ((uint16_t) 04024)        /* OCT 04024                      */
#define PH_ORIENTATION_RESTART ((uint16_t) 00014)        /* OCT 00014                      */
#define VN_GIMBALS             ALIGN_VN(6, 22)           /* V06N22                         */
#define VN_COARSE_VERB         ALIGN_VN(41, 0)           /* V41K VN 4100                   */

/**********************************************************************
 * IMUCHK (Luminary p. 982): with the IMU off (IMODES30 bit 9) alarm 210 and
 * GOTOPOOH (returns AGC_STATUS_ALARM: the caller just ends); otherwise SET
 * IMUSE.
 *
 * FUNCTION NAME: agc_lm_imu_check
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_lm_imu_check(void)
{
    if ((agc_erase_imodes30() & IMODES30_IMU_OFF) != 0u)
    {
        agc_alarm(ALM_IMU_OFF);

        const agc_status_t st = agc_exec_gotopooh(); /* status of the call */

        return ((st == AGC_STATUS_OK) ? AGC_STATUS_ALARM : st); /* GOTOPOOH started: report the alarm */
    }

    agc_flag_set(AGC_FLAG_IMUSE);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: P51 (Luminary099 P51-P53.agc p. 950)
 *
 * P51: IMUCHK, then P51 +2.
 *
 * FUNCTION NAME: agc_job_lm_imu_orientation_determination
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
agc_status_t agc_job_lm_imu_orientation_determination(agc_job_ctx_t *ctx)
{
    const agc_status_t st = agc_lm_imu_check(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (agc_job_lm_orientation_acquire_display(ctx));
}

/**********************************************************************
 * AGC: P51 (Luminary099 P51-P53.agc p. 950)
 *
 * P51 +2: V50N25 R1 00015; V33 P51B, ENTER coarse align.
 *
 * FUNCTION NAME: agc_job_lm_orientation_acquire_display
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
agc_status_t agc_job_lm_orientation_acquire_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_ACQUIRE, AGC_JOB_GOTOPOOH, AGC_JOB_LM_ORIENTATION_MARK_SETUP, AGC_JOB_LM_ORIENTATION_COARSE));
}

/**********************************************************************
 * AGC: ENTER (Luminary099 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc p. 406)
 *
 * ENTER: PHASCHNG, THETAD = 0, GODSPRET V06N22.
 *
 * FUNCTION NAME: agc_job_lm_orientation_coarse
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
agc_status_t agc_job_lm_orientation_coarse(agc_job_ctx_t *ctx)
{
    align_phase4(PH_ORIENTATION); /* PHASCHNG */

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_thetad(i, 0); /* ZERO THE GIMBALS */
    }

    return (align_flash(ctx, AGC_DSKY_GODSPRET, VN_GIMBALS, AGC_JOB_LM_ORIENTATION_COARSE_VERB, AGC_JOB_LM_ORIENTATION_COARSE_VERB, AGC_JOB_LM_ORIENTATION_COARSE_VERB));
}

/**********************************************************************
 * AGC: GODSPRET (Luminary099 DISPLAY_INTERFACE_ROUTINES.agc p. 1357)
 *
 * GODSPRET V41, then CALL COARSE.
 *
 * FUNCTION NAME: agc_job_lm_orientation_coarse_verb
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
agc_status_t agc_job_lm_orientation_coarse_verb(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GODSPRET, VN_COARSE_VERB, AGC_JOB_LM_ORIENTATION_COARSE_START, AGC_JOB_LM_ORIENTATION_COARSE_START, AGC_JOB_LM_ORIENTATION_COARSE_START));
}

/**********************************************************************
 * AGC: COARSE (Luminary099 P51-P53.agc p. 952)
 *
 * COARSE (IMUCOARS, IMUFINE and their stalls), returning to P51 +2.
 *
 * FUNCTION NAME: agc_job_lm_orientation_coarse_start
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lm_orientation_coarse_start(void)
{
    return (align_coarse(AGC_JOB_LM_ORIENTATION_COARSE_DONE));
}

/**********************************************************************
 * AGC: COARSE (Luminary099 P51-P53.agc p. 952)
 *
 * After COARSE: PHASCHNG, P51 +2.
 *
 * FUNCTION NAME: agc_job_lm_orientation_coarse_done
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
agc_status_t agc_job_lm_orientation_coarse_done(agc_job_ctx_t *ctx)
{
    align_phase4(PH_ORIENTATION);

    return (agc_job_lm_orientation_acquire_display(ctx));
}

/**********************************************************************
 * AGC: P51B (Luminary099 P51-P53.agc p. 950)
 *
 * P51B: PHASCHNG 00014; NCOARSE; STARIND = 0; P51C.
 *
 * FUNCTION NAME: agc_job_lm_orientation_mark_setup
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lm_orientation_mark_setup(void)
{
    align_phase4(PH_ORIENTATION_RESTART);
    align_ncoarse();
    agc_align_ram()->starind = 0;

    return (agc_job_lm_orientation_mark_star());
}

/**********************************************************************
 * AGC: P51C (Luminary099 P51-P53.agc p. 950)
 *
 * P51C: PHASCHNG; AOTMARK (R53); AOTSTALL.
 *
 * FUNCTION NAME: agc_job_lm_orientation_mark_star
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lm_orientation_mark_star(void)
{
    align_phase4(PH_ORIENTATION);
    agc_status_t st = agc_aotmark(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_OPTICS, AGC_JOB_LM_ORIENTATION_MARKED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (align_goto(AGC_JOB_LM_ORIENTATION_MARKED));
}

/**********************************************************************
 * AGC: AOTSTALL (Luminary099 IMU_MODE_SWITCHING_ROUTINES.agc p. 1329)
 *
 * After AOTSTALL.  First star: STARSAV1 = STARAD +6; P51D: PHASCHNG, PLANET
 * at TSIGHT -> PLANVEC.  Second star (STARSAV2 already left by AOTMARK):
 * P51E, PLANET at TSIGHT.
 *
 * FUNCTION NAME: agc_job_lm_orientation_marked
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
agc_status_t agc_job_lm_orientation_marked(agc_job_ctx_t *ctx)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_OPTICS); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    align_phase4(PH_ORIENTATION);

    if (ram->starind != 0)
    {
        return (align_planet(ctx, ram->tsight, AGC_JOB_LM_ORIENTATION_SECOND_STAR));
    }

    ram->starsav1 = ram->starad[1];

    return (align_planet(ctx, ram->tsight, AGC_JOB_LM_ORIENTATION_FIRST_STAR));
}

/**********************************************************************
 * AGC: P51D (Luminary099 P51-P53.agc p. 951)
 *
 * P51D after PLANET: PLANVEC; STARIND = 1; P51C +1.
 *
 * FUNCTION NAME: agc_job_lm_orientation_first_star
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lm_orientation_first_star(void)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->planvec = ram->planet_los;
    ram->starind = 1;

    return (agc_job_lm_orientation_mark_star());
}

/**********************************************************************
 * AGC: P51E (Luminary099 P51-P53.agc p. 951)
 *
 * P51E after PLANET: 12D, 6D, STARAD, STARAD +6; CHKSDATA.
 *
 * FUNCTION NAME: agc_job_lm_orientation_second_star
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
agc_status_t agc_job_lm_orientation_second_star(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->vac12     = ram->planet_los;
    ram->vac6      = ram->planvec;
    ram->starad[0] = ram->starsav1;
    ram->starad[1] = ram->starsav2;

    return (agc_align_sighting_data_display(ctx, AGC_JOB_LM_ORIENTATION_DATA_CHECKED));
}

/**********************************************************************
 * AGC: CHKSDATA (Luminary099 P51-P53.agc p. 946)
 *
 * After CHKSDATA: rejected -> P51 +2; accepted (P51G): AXISGEN, REFSMMAT =
 * XDC, SET REFSMFLG, GOTOPOOH.
 *
 * FUNCTION NAME: agc_job_lm_orientation_data_checked
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
agc_status_t agc_job_lm_orientation_data_checked(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return (agc_job_lm_orientation_acquire_display(ctx));
    }

    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_vec_t    a[2] = {ram->starad[0], ram->starad[1]};
    const agc_vec_t    b[2] = {ram->vac6, ram->vac12};
    const agc_status_t st   = agc_align_fine_solve(&ip, a, b, &ram->xdc); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_refsmmat(ram->xdc); /* MATMOVE  */
    agc_flag_set(AGC_FLAG_REFSMFLG);

    return (agc_exec_gotopooh()); /* FINIS    */
}
