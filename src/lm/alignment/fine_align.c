/**********************************************************************
 * FILE NAME: fine_align.c
 *
 * PURPOSE:
 *   R51 fine align (LM).  R51 is an interpretive subroutine (STQ QMAJ);
 *   the caller puts the job to continue as in agc_align_ram()->qmaj
 *   (P52OUT, or P63's attitude maneuver for R51P63).  The crew may let
 *   the LGC choose the star pair (LOCSAM + PICAPAR, which Luminary calls
 *   R56) or key the codes in.  For each star R52 points the AOT detent at
 *   it and AOTMARK takes the marks; AOTMARK leaves the averaged
 *   present-SM LOS in STARAD +6.  STARIND counts up: 0 = first star, 1 =
 *   second.
 *
 *   STARAD    = 1st catalogue LOS (PLANVEC, SM)   6D  = 1st observed
 *   STARAD +6 = 2nd catalogue LOS (SM)            12D = 2nd observed
 *
 * AGC SOURCE:
 *   Luminary099 P51-P53.agc pp. 941-943 (R51, R51.1, R51C, R51F, R51I, R51E, R51.2, R51.3, R51K, R51P63, R51.4)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   TSIGHT, BESTI/J, STARIND, STARSAV1/2, PLANVEC, STARAD, 6D, 12D, XDC, QMAJ: agc_align_ram()
 *   REFSMMAT      agc_mat_t I    half-unit      agc_erase_refsmmat()
 *   PFRATFLG, FREEFLAG  flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_vec_unit          vector arithmetic of the interpreter (interp.h)
 *   agc_mat_vec           matrix arithmetic of the interpreter (interp.h)
 *   agc_dp_add            DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_erase_refsmmat    erasable accessors (state.h)
 *   agc_alarm             ALARM (exec.h)
 *   agc_lm_auto_optics_positioning  LM alignment and optics routines (alignment.h)
 *   agc_aotmark           AOTMARK routines (alignment.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarms: ALM_NO_STARS 0405: OCT 405: no star pair available.
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "internal.h"

static agc_status_t agc_job_lm_fine_align_next_star(agc_job_ctx_t *ctx);

#define PH_FINE_ALIGN    ((uint16_t) 04024)        /* OCT 04024                     */
#define CHK_ACQUIRE      ((uint16_t) 015)          /* OCT15: V50N25 00015           */
#define CHK_FINE_RECHECK ((uint16_t) 014)          /* OCT14: V50N25 00014           */
#define TSIGHT1_CS       ((agc_dp_t) 36000)        /* TSIGHT1 2DEC 36000: 6 min     */
#define ALM_NO_STARS     ((agc_alarm_code_t) 0405) /* OCT 405: no star pair available */
#define VN_ALARM         ALIGN_VN(5, 9)            /* VB05N09                       */

/**********************************************************************
 * AGC: R51 (Luminary099 P51-P53.agc p. 941)
 *
 * R51 (STQ QMAJ: the caller has set agc_align_ram()->qmaj).
 *
 * FUNCTION NAME: agc_job_lm_fine_align
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
agc_status_t agc_job_lm_fine_align(agc_job_ctx_t *ctx)
{
    return (agc_job_lm_fine_align_perform(ctx));
}

/**********************************************************************
 * AGC: R51.1 (Luminary099 P51-P53.agc p. 941)
 *
 * R51.1 (restart 4.21 enters here): PHASCHNG 04024, then R51C.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_perform
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
agc_status_t agc_job_lm_fine_align_perform(agc_job_ctx_t *ctx)
{
    align_phase4(PH_FINE_ALIGN);

    return (agc_job_lm_fine_align_star_acquire(ctx));
}

/**********************************************************************
 * AGC: R51C (Luminary099 P51-P53.agc p. 941)
 *
 * R51C: V50N25 R1 00015; V33 star selection, ENTER the crew's stars.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_star_acquire
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
agc_status_t agc_job_lm_fine_align_star_acquire(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_ACQUIRE, AGC_JOB_GOTOPOOH, AGC_JOB_LM_FINE_ALIGN_PICK_STARS, AGC_JOB_LM_FINE_ALIGN_FIRST_STAR));
}

/**********************************************************************
 * AGC: LOCSAM (Luminary099 P51-P53.agc p. 935)
 *
 * LOCSAM at TSIGHT, then R56 (PICAPAR): a pair found -> R51F -> R51E; none
 * -> R51I: alarm 405, V05N09 (PROCEED R51E, ENTER R51C).
 *
 * FUNCTION NAME: fine_align_pick
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
static agc_status_t fine_align_pick(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_align_locsam(&ip, ram->tsight, AGC_JOB_LM_FINE_ALIGN_STARS_LOCATED, &ram->locsam); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    uint16_t cdu[3]; /* CDUX, CDUY, CDUZ words */

    align_read_cdus(cdu);
    const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */
    bool            found    = false;                /* pair found */

    if (st == AGC_STATUS_OK)
    {
        st = agc_align_picapar(&ip, &ram->locsam, &refsmmat, cdu, &ram->besti, &ram->bestj, &found);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (found)
    {
        return (agc_job_lm_fine_align_first_star(ctx)); /* R51F     */
    }

    agc_alarm(ALM_NO_STARS); /* R51I     */

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALARM, AGC_JOB_GOTOPOOH, AGC_JOB_LM_FINE_ALIGN_FIRST_STAR, AGC_JOB_LM_FINE_ALIGN_STAR_ACQUIRE));
}

/**********************************************************************
 * AGC: R51C (Luminary099 P51-P53.agc p. 941)
 *
 * R51C V33: TSIGHT = now + TSIGHT1 (6 min to marking); LOCSAM.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_pick_stars
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
agc_status_t agc_job_lm_fine_align_pick_stars(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_align_ram()->tsight = agc_dp_add(&ip, agc_rtb_loadtime(), TSIGHT1_CS);

    return (fine_align_pick(ctx));
}

/**********************************************************************
 * AGC: LOCSAM (Luminary099 P51-P53.agc p. 935)
 *
 * LOCSAM resumed after its integration wait.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_stars_located
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
agc_status_t agc_job_lm_fine_align_stars_located(agc_job_ctx_t *ctx)
{
    return (fine_align_pick(ctx));
}

/**********************************************************************
 * AGC: R51E (Luminary099 P51-P53.agc p. 941)
 *
 * R51E: STARIND = 0, then R51.3.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_first_star
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
agc_status_t agc_job_lm_fine_align_first_star(agc_job_ctx_t *ctx)
{
    agc_align_ram()->starind = 0;

    return (agc_job_lm_fine_align_next_star(ctx));
}

/**********************************************************************
 * AGC: R51.3 (Luminary099 P51-P53.agc p. 941)
 *
 * R51.3: PHASCHNG 04024; CALL R52 (point the AOT), returning to the mark.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_next_star
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
static agc_status_t agc_job_lm_fine_align_next_star(agc_job_ctx_t *ctx)
{
    align_phase4(PH_FINE_ALIGN);

    return (agc_lm_auto_optics_positioning(ctx, AGC_JOB_LM_FINE_ALIGN_MARK));
}

/**********************************************************************
 * AGC: R52 (Luminary099 P51-P53.agc p. 955)
 *
 * After R52: AOTMARK, then AOTSTALL (CURTAINS on a bad end).
 *
 * FUNCTION NAME: agc_job_lm_fine_align_mark
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lm_fine_align_mark(void)
{
    agc_status_t st = agc_aotmark(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_OPTICS, AGC_JOB_LM_FINE_ALIGN_MARKED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (align_goto(AGC_JOB_LM_FINE_ALIGN_MARKED));
}

/**********************************************************************
 * AGC: AOTSTALL (Luminary099 IMU_MODE_SWITCHING_ROUTINES.agc p. 1329)
 *
 * After AOTSTALL.  First star (R51.4): STARSAV1 = STARAD +6 (observed),
 * PLANET at TSIGHT -> PLANVEC.  Second star: STARSAV2 = STARAD +6,
 * PHASCHNG, PLANET at TSIGHT.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_marked
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
agc_status_t agc_job_lm_fine_align_marked(agc_job_ctx_t *ctx)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_OPTICS); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    if (ram->starind == 0)
    {
        ram->starsav1 = ram->starad[1]; /* R51.4    */

        return (align_planet(ctx, ram->tsight, AGC_JOB_LM_FINE_ALIGN_FIRST_LOCATED));
    }

    ram->starsav2 = ram->starad[1];
    align_phase4(PH_FINE_ALIGN);

    return (align_planet(ctx, ram->tsight, AGC_JOB_LM_FINE_ALIGN_SECOND_LOCATED));
}

/**********************************************************************
 * AGC: R51.4 (Luminary099 P51-P53.agc p. 942)
 *
 * R51.4 after PLANET: PLANVEC = LOS (ref); STARIND = 1; R51.3.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_first_located
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
agc_status_t agc_job_lm_fine_align_first_located(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->planvec = ram->planet_los;
    ram->starind = 1;

    return (agc_job_lm_fine_align_next_star(ctx));
}

/**********************************************************************
 * AGC: PLANET (Luminary099 P51-P53.agc p. 961)
 *
 * Second star after PLANET: STARAD +6 = UNIT(REFSMMAT LOS), STARAD =
 * UNIT(REFSMMAT PLANVEC), 6D = STARSAV1, 12D = STARSAV2; R54.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_second_located
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
agc_status_t agc_job_lm_fine_align_second_located(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */

    ram->starad[1] = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, ram->planet_los), NULL, NULL);
    ram->starad[0] = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, ram->planvec), NULL, NULL);
    ram->vac6      = ram->starsav1;
    ram->vac12     = ram->starsav2;

    return (agc_align_sighting_data_display(ctx, AGC_JOB_LM_FINE_ALIGN_DATA_CHECKED));
}

/**********************************************************************
 * AGC: R54 (Luminary099 P51-P53.agc p. 945)
 *
 * After R54: rejected -> R51K; else AXISGEN and R55, CLEAR PFRATFLG.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_data_checked
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
agc_status_t agc_job_lm_fine_align_data_checked(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return (agc_job_lm_fine_align_recheck(ctx));
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

    return (agc_align_gyro_torquing(ctx, AGC_JOB_LM_FINE_ALIGN_TORQUED)); /* R55 */
}

/**********************************************************************
 * AGC: R55 (Luminary099 P51-P53.agc p. 945)
 *
 * After R55: CLEAR PFRATFLG, R51K.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_torqued
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
agc_status_t agc_job_lm_fine_align_torqued(agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_PFRATFLG);

    return (agc_job_lm_fine_align_recheck(ctx));
}

/**********************************************************************
 * AGC: R51K (Luminary099 P51-P53.agc p. 942)
 *
 * R51K / R51P63: V50N25 R1 00014 (fine align check): V34 terminates,
 * PROCEED aligns again (R51C), ENTER returns to QMAJ.
 *
 * FUNCTION NAME: agc_job_lm_fine_align_recheck
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
agc_status_t agc_job_lm_fine_align_recheck(agc_job_ctx_t *ctx)
{
    const agc_job_id_t ret = agc_align_ram()->qmaj; /* QMAJ: the job ENTER returns to */

    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_FINE_RECHECK, AGC_JOB_GOTOPOOH, AGC_JOB_LM_FINE_ALIGN_STAR_ACQUIRE, ret));
}
