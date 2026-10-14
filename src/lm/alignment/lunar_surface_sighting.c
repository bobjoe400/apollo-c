/**********************************************************************
 * FILE NAME: lunar_surface_sighting.c
 *
 * PURPOSE:
 *   The star sightings of P57: 2STARS, 1STAR, R59RET, ASTAR and R59,
 *   lunar surface star acquisition: for a catalogue star R59 finds the
 *   AOT detent whose 60 degree field contains it and the CURSOR and
 *   SPIRAL settings that put the reticle on it (shown on V06N79), then
 *   AOTMARK takes the surface marks.  STARIND: 0 = first star, 1 =
 *   second.
 *
 * AGC SOURCE:
 *   Luminary099 P51-P53.agc pp. 957-960 (R59 .. R59OUT), 967-968 (2STARS, 1STAR, R59RET, ASTAR)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   AOTCODE, BESTI/J, STARIND, STAR, SCAXIS, UXP, UYP, CURSOR, SPIRAL, POSCODE, VEC1/2, STARSAV1: agc_align_ram()
 *   AOTAZ         uint16_t  I    CDU words      detent pad loads
 *   REFSMMAT      agc_mat_t I    half-unit      agc_erase_refsmmat()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_ram         alignment routines of this lane (alignment.h)
 *   agc_imu_cdu_smnb     CDUTRIG, *NBSM* / *SMNB*, PIPASR (imu.h)
 *   agc_star_vector       star catalogue (auxiliary.h)
 *   agc_mpac_*            interpretive instructions on MPAC words (interp_words.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_mat_vec           matrix arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_test         flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_alarm             ALARM (exec.h)
 *   agc_lm_lunar_surface_surfline  LM alignment and optics routines (alignment.h)
 *   agc_aot*              AOTMARK routines (alignment.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarms: ALM_NOT_IN_AOT 0404: OCT 404: star not in any AOT field.
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/core/interp_words.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"

static agc_status_t agc_job_lunar_surface_star_position_display(agc_job_ctx_t *ctx);

#define PH_SURFACE_ALIGN ((uint16_t) 04024)        /* OCT 04024                      */
#define VN_STAR_CODE     ALIGN_VN(1, 70)           /* V01N70* VN 170                 */
#define VN_CURSOR        ALIGN_VN(6, 79)           /* V06N79 VN 679                  */
#define VN_ALARM         ALIGN_VN(5, 9)            /* VB05N09                        */
#define ALM_NOT_IN_AOT   ((agc_alarm_code_t) 0404) /* OCT 404: star not in any AOT field */
#define STAR_BITS        ((agc_sp_t) 077)          /* CS HIGH9: star bits 1-6        */
#define REVCNT           ((agc_sp_t) 6)            /* REVCNT: just 6                 */
#define DEC227           ((agc_sp_t) 227)          /* 228 and up: sun, earth, moon   */
#define POSITIONS        6                         /* AOT detents 1..6               */
#define POS_SHIFT        6                         /* MP BIT7: detent to bits 7-9    */
#define POS_MASK         ((agc_sp_t) 07)           /* SEVEN: detent bits */
#define ELEVATION_45     ((uint16_t) 010000)       /* BIT13: 45 degrees              */
#define DEG30            ((agc_dp_t) 22369621)     /* DEG30 2DEC .083333333 revs    */
#define DEG_HALF         ((agc_dp_t) 372825)       /* DEG.5 2DEC .00138888 revs     */
#define THREE_QUARTERS   ((agc_dp_t) 201326592)    /* 3/4                          */
#define ABOUTONE         ((agc_dp_t) 268435453)    /* ABOUTONE 2DEC .99999999      */

/**********************************************************************
 * AGC: 1STAR (Luminary099 P51-P53.agc p. 970)
 *
 * 1STAR / 2STARS tail: STARIND, PHASCHNG, R59.
 *
 * FUNCTION NAME: star_sighting
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *   starind   agc_sp_t         I    STARIND: 0 = first star (BESTI), 1 =
 *                                   second (BESTJ)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t star_sighting(agc_job_ctx_t *ctx, agc_sp_t starind)
{
    agc_align_ram()->starind = starind;
    align_phase4(PH_SURFACE_ALIGN);

    return (agc_job_lunar_surface_star_acquisition(ctx));
}

/**********************************************************************
 * AGC: 2STARS (Luminary099 P51-P53.agc p. 970)
 *
 * 2STARS: STARIND = 0 (first star).
 *
 * FUNCTION NAME: agc_lm_lunar_surface_two_stars
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
agc_status_t agc_lm_lunar_surface_two_stars(agc_job_ctx_t *ctx)
{
    return (star_sighting(ctx, 0));
}

/**********************************************************************
 * AGC: 1STAR (Luminary099 P51-P53.agc p. 970)
 *
 * 1STAR: STARIND = 1 (second star, or the only one of option 3).
 *
 * FUNCTION NAME: agc_lm_lunar_surface_one_star
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
agc_status_t agc_lm_lunar_surface_one_star(agc_job_ctx_t *ctx)
{
    return (star_sighting(ctx, 1));
}

/**********************************************************************
 * R59 (p. 957): without a REFSMMAT go straight to AOTMARK (R59OUT);
 * otherwise flash V01N70 (star code for the acquisition).
 *
 * FUNCTION NAME: agc_job_lunar_surface_star_acquisition
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
agc_status_t agc_job_lunar_surface_star_acquisition(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        return (agc_job_lunar_surface_star_mark());
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_CODE, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_STAR_CHOSEN, AGC_JOB_LUNAR_SURFACE_STAR_ACQUISITION));
}

/**********************************************************************
 * AGC: INCAZ (Luminary099 P51-P53.agc p. 957)
 *
 * INCAZ .. R59D for one AOT position: OANB of (AOTAZ, 45 deg); the star
 * (NB) at less than 30 deg from the optic axis is in the field.  Near the
 * centre (< .5 deg) CURSOR = SPIRAL = 0; otherwise YROT = the angle of OA x
 * STAR from OA x (OA x UNITX) (360 - it for a negative C2), CURSOR = AZ +
 * YROT, SPIRAL = 12 x separation + CURSOR.  false: not in field.
 *
 * FUNCTION NAME: star_in_position
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   star      agc_vec_t       I    star line of sight, half-unit (2^1)
 *   az        uint16_t        I    AOT azimuth, CDU word (2^15 = 1 rev)
 *
 * RETURNS: bool -- true when the star is in this detent's field (CURSOR,
 *          SPIRAL set)
 **********************************************************************/
static bool star_in_position(agc_interp_t *ip, agc_vec_t star, uint16_t az)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    agc_aot_oanb(ip, az, ELEVATION_45, &ram->scaxis, &ram->uxp, &ram->uyp);

    /* The angles stay in MPAC words: 1STO2S of YROT and of 12 SEP + YROT
     * reads the words ARCCOS, DSU, SR1 and DAD leave (word_forms.c).   */
    const agc_dp_words_t sepw = agc_mpac_store(agc_mpac_acos(ip, agc_mpac_shift(ip, agc_mpac_dot(ip, star, ram->scaxis), 1))); /* DOT SL1 ARCCOS: 24D */
    const agc_dp_t       sep  = agc_dp_from_words(sepw);                                                                       /* separation from the optic axis, revs 2^0 */

    if ((sep - DEG30) >= 0)
    {
        return (false); /* NXAX     */
    }

    if ((sep - DEG_HALF) < 0)
    {
        ram->cursor = 0; /* ZSPCR    */
        ram->spiral = 0;
    }
    else
    {
        const agc_dp_words_t spiral12 = agc_mpac_store(agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(sepw), agc_dp_to_words(THREE_QUARTERS)), 4)); /* DMP SL4: 24D */
        const agc_vec_t      ux       = agc_vec_make(AGC_DP_HALF, 0, 0);                                                                            /* UNITX, half-unit (2^1) */
        const agc_vec_t      oaux     = agc_vec_unit(ip, agc_vec_cross(ip, ram->scaxis, ux), NULL, NULL);                                           /* UNIT(OA x UNITX), half-unit (2^1) */
        const agc_vec_t      c2       = agc_vec_unit(ip, agc_vec_neg(agc_vec_cross(ip, oaux, ram->scaxis)), NULL, NULL);                            /* -UNIT(OAUX x OA), half-unit (2^1) */
        const agc_vec_t      oas      = agc_vec_unit(ip, agc_vec_cross(ip, ram->scaxis, star), NULL, NULL);                                         /* UNIT(OA x STAR), half-unit (2^1) */
        agc_dp_words_t       thet     = agc_mpac_store(agc_mpac_acos(ip, agc_mpac_shift(ip, agc_mpac_dot(ip, oas, oaux), 1)));                      /* 26D */

        if (agc_vec_dot(ip, oas, c2) < 0)
        {
            thet = agc_mpac_store(agc_mpac_dsu(ip, agc_mpac_dload(agc_dp_to_words(ABOUTONE)), thet)); /* 360-THET */
        }

        /* R59D: SLOAD SR1 QMIN, DAD 26D, PUSH; DLOAD 24D, DAD.          */
        const agc_dp_words_t az_words = {align_word_to_sp(az), 0};
        const agc_dp_words_t yrot     = agc_mpac_store(agc_mpac_dad(ip, agc_mpac_shift(ip, agc_mpac_dload(az_words), -1), thet)); /* YROT = AZ/2 + THET, MPAC words, revs 2^0 */

        ram->cursor = align_words_to_cdu(yrot); /* 1STO2S   */
        ram->spiral = align_words_to_cdu(agc_mpac_store(agc_mpac_dad(ip, agc_mpac_dload(spiral12), yrot)));
    }

    return (true);
} /* star_in_position */

/**********************************************************************
 * INCAZ .. NXAX (pp. 957-959): POSCODE = 1..6 until the AOT detent at
 * AOTAZ(POSCODE) and 45 degrees elevation sees star (NB, half-unit); CURSOR
 * and SPIRAL are set for it.  false: no detent (R59ALM).
 *
 * FUNCTION NAME: agc_lm_aot_find_detent
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   star      agc_vec_t       I    star line of sight, half-unit (2^1)
 *
 * RETURNS: bool -- true when some detent sees the star (POSCODE is that
 *          detent)
 **********************************************************************/
bool agc_lm_aot_find_detent(agc_interp_t *ip, agc_vec_t star)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    for (int pos = 1; pos <= POSITIONS; pos++)
    { /* INCAZ    */
        ram->poscode = (agc_sp_t) pos;

        if (star_in_position(ip, star, agc_erase_aotaz(pos)))
        {
            return (true);
        }
    }

    return (false);
}

/**********************************************************************
 * AGC: R59A (Luminary099 P51-P53.agc p. 957)
 *
 * R59A: BESTI +STARIND = 6 x star code; not a catalogue star (0, or the
 * sun, earth, moon) -> R59OUT.  STAR = CDU*SMNB(UNIT(REFSMMAT CATLOG));
 * POSCODE 1..6 until one sees it (79DISP); none: alarm 404 (R59ALM).
 *
 * FUNCTION NAME: agc_job_lunar_surface_star_chosen
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
agc_status_t agc_job_lunar_surface_star_chosen(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram   = agc_align_ram();                                   /* alignment erasables */
    const agc_sp_t   star6 = (agc_sp_t) ((ram->starcode & STAR_BITS) * REVCNT); /* star code x 6 (unscaled) */

    if (ram->starind == 0)
    {
        ram->besti = star6;
    }
    else
    {
        ram->bestj = star6;
    }

    if ((star6 == 0) || ((DEC227 - star6) <= 0))
    {
        return (agc_job_lunar_surface_star_mark()); /* R59OUT   */
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_vec_t       star;                                              /* catalogue star, then in nav-base axes, half-unit (2^1) */
    const agc_mat_t refsmmat = agc_erase_refsmmat();                   /* REFSMMAT, half-unit rows */
    agc_cdutrig_t   t;                                                 /* CDUTRIG sines and cosines */
    agc_status_t    st       = agc_star_vector(star6 / REVCNT, &star); /* CATLOG,1 */

    if (st == AGC_STATUS_OK)
    {
        star = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, star), NULL, NULL);
        agc_imu_cdu_smnb(&t, &star);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ram->star = star;

    if (agc_lm_aot_find_detent(&ip, star))
    {
        return (agc_job_lunar_surface_star_position_display(ctx));
    }

    agc_alarm(ALM_NOT_IN_AOT); /* R59ALM   */

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALARM, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_STAR_MARK, AGC_JOB_LUNAR_SURFACE_STAR_ACQUISITION));
} /* agc_job_lunar_surface_star_chosen */

/**********************************************************************
 * AGC: 79DISP (Luminary099 P51-P53.agc p. 959)
 *
 * 79DISP: flash V06N79 (CURSOR, SPIRAL, POSCODE); V32 back to R59.
 *
 * FUNCTION NAME: agc_job_lunar_surface_star_position_display
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
static agc_status_t agc_job_lunar_surface_star_position_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_CURSOR, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_STAR_POSITIONED, AGC_JOB_LUNAR_SURFACE_STAR_ACQUISITION));
}

/**********************************************************************
 * AGC: R59E (Luminary099 P51-P53.agc p. 960)
 *
 * R59E: AOTCODE detent bits 7-9 = POSCODE; R59OUT.
 *
 * FUNCTION NAME: agc_job_lunar_surface_star_positioned
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_surface_star_positioned(void)
{
    agc_align_ram_t *ram = agc_align_ram();                                     /* alignment erasables */
    const agc_sp_t   pos = (agc_sp_t) ((ram->poscode & POS_MASK) << POS_SHIFT); /* detent code in bits 7-9 (unscaled) */

    ram->starcode = (agc_sp_t) ((ram->starcode & STAR_BITS) + pos);

    return (agc_job_lunar_surface_star_mark());
}

/**********************************************************************
 * AGC: R59OUT (Luminary099 P51-P53.agc p. 960)
 *
 * R59OUT: AOTMARK, AOTSTALL (CURTAINS on a bad end), then R59RET.
 *
 * FUNCTION NAME: agc_job_lunar_surface_star_mark
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_surface_star_mark(void)
{
    agc_status_t st = agc_aotmark(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_OPTICS, AGC_JOB_LUNAR_SURFACE_STAR_MARKED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (align_goto(AGC_JOB_LUNAR_SURFACE_STAR_MARKED));
}

/**********************************************************************
 * AGC: R59RET (Luminary099 P51-P53.agc p. 970)
 *
 * R59RET: first star (ASTAR): STARSAV1 = STARAD +6, PLANET at TSIGHT ->
 * VEC1, then 1STAR.  Second star: PHASCHNG, PLANET -> VEC2, SURFLINE.
 *
 * FUNCTION NAME: agc_job_lunar_surface_star_marked
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
agc_status_t agc_job_lunar_surface_star_marked(agc_job_ctx_t *ctx)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_OPTICS); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    if (ram->starind == 0)
    {
        ram->starsav1 = ram->starad[1]; /* ASTAR    */

        return (align_planet(ctx, ram->tsight, AGC_JOB_LUNAR_SURFACE_FIRST_STAR));
    }

    align_phase4(PH_SURFACE_ALIGN);

    return (align_planet(ctx, ram->tsight, AGC_JOB_LUNAR_SURFACE_SECOND_STAR));
}

/**********************************************************************
 * AGC: ASTAR (Luminary099 P51-P53.agc p. 970)
 *
 * ASTAR after PLANET: VEC1 = the first catalogue LOS; 1STAR.
 *
 * FUNCTION NAME: agc_job_lunar_surface_first_star
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
agc_status_t agc_job_lunar_surface_first_star(agc_job_ctx_t *ctx)
{
    agc_align_ram()->vec1 = agc_align_ram()->planet_los;

    return (agc_lm_lunar_surface_one_star(ctx));
}

/**********************************************************************
 * AGC: R59RET (Luminary099 P51-P53.agc p. 970)
 *
 * R59RET after PLANET: VEC2 = the second catalogue LOS; SURFLINE.
 *
 * FUNCTION NAME: agc_job_lunar_surface_second_star
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
agc_status_t agc_job_lunar_surface_second_star(agc_job_ctx_t *ctx)
{
    agc_align_ram()->vec2 = agc_align_ram()->planet_los;

    return (agc_lm_lunar_surface_surfline(ctx));
}
