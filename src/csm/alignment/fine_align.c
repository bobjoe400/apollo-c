/**********************************************************************
 * FILE NAME: fine_align.c
 *
 * PURPOSE:
 *   R51 fine align (CSM).  Two stars (BESTI, BESTJ from PICAPAR or keyed
 *   in on V01N70) are each sighted: R52 drives the sextant to the
 *   catalogue line of sight and R53 takes the mark (R56 in P54); SXTSM
 *   turns the mark into a present-SM line of sight.  With both stars
 *   marked, R54 checks the star angles, AXISGEN gives the desired SM wrt
 *   the present SM and R55 torques the gyros.  STARIND counts down: 1 =
 *   first star, 0 = second star.
 *
 *   Pairing for R54 / AXISGEN (as the AGC stores them):
 *     STARAD    = 2nd catalogue LOS (desired SM)   6D  = 2nd observed (SM)
 *     STARAD +6 = 1st catalogue LOS (PLANVEC)      12D = 1st observed
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 756-758 (R51, R51.2, R51.3, R51DSP, R51ST, R51A, R51K, R51.4, R51B)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   STARIND, MARKINDX, STARCODE, BESTI/J, STARSAV1/2, PLANVEC, STARAD, 6D, 12D, XDC: agc_align_ram()
 *   MODREG        agc_sp_t  I    unscaled       P54 test
 *   TARG1FLG, TARG2FLG, PFRATFLG, FREEFLAG  flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_*            DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_vec_unit          vector arithmetic of the interpreter (interp.h)
 *   agc_mat_vec           matrix arithmetic of the interpreter (interp.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_csm_*             CSM optics and sighting routines (alignment.h)
 *   agc_sxtmark_mkreleas  SXTMARK routines (alignment.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/programs/alignment.h"

#define PH_SIGHTING       ((uint16_t) 05024) /* OCT 05024, OCT 13000: R52-R53  */
#define VN_STAR_CODE      ALIGN_VN(1, 70)    /* V01N70 VN 0170                 */
#define BLANK_R2_R3       ((uint16_t) 06)    /* CAF SIX, TC BLANKET            */
#define CHK_FINE_RECHECK  ((uint16_t) 014)   /* OCT14: V50N25 00014            */
#define MM_BACKUP_REALIGN ((agc_sp_t) 54)    /* OCT66 = 54 decimal (P54)       */
#define STAR_WORDS        ((agc_sp_t) 6)     /* 1/6TH: BESTI is star x 6       */

/**********************************************************************
 * AGC: R51 (Comanche055 P51-P53.agc p. 756)
 *
 * R51 (restart 4.31 enters here): STARIND = MARKINDX = 1, then R51.3.
 * R51.3: CLEAR TARG2FLG, TARG1FLG; PHASCHNG 05024 (restart at the star code
 * display); STARCODE = BESTI +STARIND / 6.
 *
 * FUNCTION NAME: agc_job_csm_fine_align
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
agc_status_t agc_job_csm_fine_align(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->starind  = 1;
    ram->markindx = 1;

    return (agc_job_csm_fine_align_next_star(ctx));
}

/**********************************************************************
 * AGC: R51.3 (Comanche055 P51-P53.agc p. 756)
 *
 * R51.3 (also re-entered from R51.4 for the second star).
 *
 * FUNCTION NAME: agc_job_csm_fine_align_next_star
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
agc_status_t agc_job_csm_fine_align_next_star(agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_TARG2FLG);
    agc_flag_clear(AGC_FLAG_TARG1FLG);
    align_phase_job(PH_SIGHTING, AGC_JOB_CSM_FINE_ALIGN_STAR_CODE, ALIGN_PRIO13);

    return (agc_job_csm_fine_align_star_code(ctx));
}

/**********************************************************************
 * AGC: STARCODE (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 74)
 *
 * STARCODE = BESTI +STARIND x 1/6, then R51DSP.
 *
 * FUNCTION NAME: agc_job_csm_fine_align_star_code
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
agc_status_t agc_job_csm_fine_align_star_code(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram   = agc_align_ram();                               /* alignment erasables */
    const agc_sp_t   star6 = (ram->starind == 0) ? ram->besti : ram->bestj; /* BESTI +STARIND, star code x 6 (unscaled); INDEX STARIND: BESTI or BESTJ */

    ram->starcode = (agc_sp_t) (star6 / STAR_WORDS);

    return (agc_job_csm_fine_align_star_display(ctx));
}

/**********************************************************************
 * AGC: R51DSP (Comanche055 P51-P53.agc p. 756)
 *
 * R51DSP: GOFLASHR V01N70 (star code); V34 terminates, V33 goes on
 * (CHKSCODE), ENTER shows it again.  The immediate return blanks R2, R3 and
 * ends the job.
 *
 * FUNCTION NAME: agc_job_csm_fine_align_star_display
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
agc_status_t agc_job_csm_fine_align_star_display(agc_job_ctx_t *ctx)
{
    agc_status_t st = align_flash(ctx, AGC_DSKY_GOFLASHR, VN_STAR_CODE, AGC_JOB_GOTOPOOH, AGC_JOB_CSM_FINE_ALIGN_STAR_ACCEPTED, AGC_JOB_CSM_FINE_ALIGN_STAR_DISPLAY); /* status of the call */

    if (st == AGC_STATUS_OK)
    {
        st = agc_dsky_blanket(BLANK_R2_R3); /* BLANKET  */
    }

    return (st); /* ENDOFJOB */
}

/**********************************************************************
 * AGC: R51DSP (Comanche055 P51-P53.agc p. 756)
 *
 * R51DSP V33: CHKSCODE (a bad code: FALTON and R51DSP again); PLANET at the
 * present time.
 *
 * FUNCTION NAME: agc_job_csm_fine_align_star_accepted
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
agc_status_t agc_job_csm_fine_align_star_accepted(agc_job_ctx_t *ctx)
{
    if (!agc_align_chkscode(agc_align_ram()->starcode))
    {
        agc_dsky_falton();

        return (agc_job_csm_fine_align_star_display(ctx));
    }

    return (align_planet(ctx, align_loadtime(), AGC_JOB_CSM_FINE_ALIGN_STAR_LOCATED));
}

/**********************************************************************
 * AGC: R51ST (Comanche055 P51-P53.agc p. 756)
 *
 * R51ST: STARSAV1 (STARIND 1) or STARSAV2 (STARIND 0) = the catalogue LOS
 * for R52.  P54 marks with R56 (R51B); otherwise CALL R52 (which calls
 * R53), both returning to R51A.
 *
 * FUNCTION NAME: agc_job_csm_fine_align_star_located
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
agc_status_t agc_job_csm_fine_align_star_located(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    if (ram->starind != 0)
    {
        ram->starsav1 = ram->planet_los; /* R51ST    */
    }
    else
    {
        ram->starsav2 = ram->planet_los;
    }

    if (agc_erase_modreg() == MM_BACKUP_REALIGN)
    { /* R51B     */
        return (agc_csm_alternate_los_sighting_mark(ctx, AGC_JOB_CSM_FINE_ALIGN_MARKED));
    }

    return (agc_csm_auto_optics_positioning(ctx, AGC_JOB_CSM_FINE_ALIGN_MARKED));
}

/**********************************************************************
 * AGC: R51A (Comanche055 P51-P53.agc p. 757)
 *
 * R51A: STARSAV2 = SXTSM (observed LOS, SM); MKRELEAS; PLANET at TSIGHT.
 *
 * FUNCTION NAME: agc_job_csm_fine_align_marked
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
agc_status_t agc_job_csm_fine_align_marked(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_csm_sxtsm_marks(&ip, &ram->starsav2); /* status of the call */

    if (st == AGC_STATUS_OK)
    {
        st = agc_sxtmark_mkreleas();
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_planet(ctx, ram->tsight, AGC_JOB_CSM_FINE_ALIGN_STAR_AT_MARK));
}

/**********************************************************************
 * AGC: PLANET (Comanche055 P51-P53.agc p. 782)
 *
 * After PLANET at the mark time.  First star (R51.4): PLANVEC =
 * UNIT(REFSMMAT LOS), STARSAV1 = STARSAV2, STARIND = 0, R51.3.  Second
 * star: STARAD, 6D, 12D, STARAD +6 as in the file header, then R54.
 *
 * FUNCTION NAME: agc_job_csm_fine_align_star_at_mark
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
agc_status_t agc_job_csm_fine_align_star_at_mark(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_mat_t refsmmat = agc_erase_refsmmat();                                                        /* REFSMMAT, half-unit rows */
    const agc_vec_t cat      = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, ram->planet_los), NULL, NULL); /* UNIT(REFSMMAT LOS): the catalogue LOS in present SM, half-unit (2^1) */

    if (ram->starind != 0)
    { /* R51.4    */
        ram->planvec  = cat;
        ram->starsav1 = ram->starsav2;
        ram->starind  = 0;

        return (agc_job_csm_fine_align_next_star(ctx));
    }

    ram->starad[0] = cat;
    ram->vac6      = ram->starsav2;
    ram->vac12     = ram->starsav1;
    ram->starad[1] = ram->planvec;

    return (agc_align_sighting_data_display(ctx, AGC_JOB_CSM_FINE_ALIGN_DATA_CHECKED));
}

/**********************************************************************
 * AGC: R54 (Comanche055 P51-P53.agc p. 759)
 *
 * After R54: FREEFLAG clear (data rejected) -> R51K; otherwise AXISGEN
 * (XDC) and R55 (gyro torquing), then CLEAR PFRATFLG and R51K.
 *
 * FUNCTION NAME: agc_job_csm_fine_align_data_checked
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
agc_status_t agc_job_csm_fine_align_data_checked(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return (agc_job_csm_fine_align_recheck(ctx));
    }

    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_vec_t    a[2] = {ram->starad[0], ram->starad[1]};
    const agc_vec_t    b[2] = {ram->vac6, ram->vac12};
    const agc_status_t st   = agc_align_fine_solve(&ip, a, b, &ram->xdc); /* AXISGEN */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_align_gyro_torquing(ctx, AGC_JOB_CSM_FINE_ALIGN_TORQUED)); /* R55 */
}

/**********************************************************************
 * AGC: R55 (Comanche055 P51-P53.agc p. 759)
 *
 * After R55: CLEAR PFRATFLG, R51K.
 *
 * FUNCTION NAME: agc_job_csm_fine_align_torqued
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
agc_status_t agc_job_csm_fine_align_torqued(agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_PFRATFLG);

    return (agc_job_csm_fine_align_recheck(ctx));
}

/**********************************************************************
 * AGC: R51K (Comanche055 P51-P53.agc p. 757)
 *
 * R51K: V50N25 R1 00014 (fine align check): V34 terminates, PROCEED does
 * the star selection again (P52C), ENTER ends the program (ENDP50S).
 *
 * FUNCTION NAME: agc_job_csm_fine_align_recheck
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
agc_status_t agc_job_csm_fine_align_recheck(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_FINE_RECHECK, AGC_JOB_GOTOPOOH, AGC_JOB_CSM_REALIGN_STAR_SELECTION, AGC_JOB_GOTOPOOH));
}
