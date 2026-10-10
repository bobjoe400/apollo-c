/*
 * auto_optics_positioning.c -- R52 automatic optics positioning (LM),
 * P51-P53.agc, Luminary099 pp. 955-956 (labels R52, R52A, R52B, GETAZEL,
 * AZEL).
 *
 * The LM's AOT cannot be driven, so R52 turns the vehicle instead: the
 * star code (with AOT detent 2 by default) is shown on V01N70; the optic
 * axis of the chosen detent (or the AZ, EL keyed in on V06N87 for COAS
 * codes 0 and 7) becomes SCAXIS, the star's line of sight POINTVSM, and
 * R60LEM maneuvers the LM to put one on the other.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/programs/alignment_internal.h"

#define VN_STAR_CODE ALIGN_VN(1, 70)   /* V01N70 VN 0170                 */
#define VN_AZ_EL     ALIGN_VN(6, 87)   /* V06N87 VN 687                  */
#define STAR_WORDS   ((agc_sp_t) 6)    /* 1/6TH                          */
#define DETENT_2     ((agc_sp_t) 0200) /* BIT8: detent position 2        */
#define DETENT_SHIFT 6                 /* MP BIT9: bits 7-9 -> 1-3       */
#define DETENT_MASK  ((agc_sp_t) 07)
#define DETENT_COAS  ((agc_sp_t) 7)      /* code 7: COAS sighting          */
#define DETENT_BITS  ((agc_sp_t) 0700)   /* HIGH9 & a star code < 01000    */
#define ELEVATION_45 ((uint16_t) 010000) /* BIT13: 45 degrees              */

/* Detent code (bits 7-9 of STARCODE / AOTCODE). */
static agc_sp_t detent_code(void)
{
    const agc_sp_t d =
        (agc_sp_t) ((agc_align_ram()->starcode >> DETENT_SHIFT) & DETENT_MASK);
    if (!AGC_ASSERT((d >= 0) && (d <= DETENT_COAS)))
    {
        return 0; /* no valid result */
    }
    return d;
}

/*
 * R52 (Luminary p. 955): STQ SAVQR52; STARCODE = BESTI +STARIND / 6 +
 * detent 2; then R52A.
 */
agc_status_t agc_lm_auto_optics_positioning(const agc_job_ctx_t *ctx,
                                            agc_job_id_t         ret)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram    = agc_align_ram();
    ram->auto_optics_return = ret; /* SAVQR52  */
    const agc_sp_t star6    = (ram->starind == 0) ? ram->besti : ram->bestj;
    if (!AGC_ASSERT(star6 >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    ram->starcode = (agc_sp_t) ((star6 / STAR_WORDS) + DETENT_2);
    return align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_CODE, AGC_JOB_GOTOPOOH,
                       AGC_JOB_LM_AUTO_OPTICS_ORIENT,
                       AGC_JOB_LM_AUTO_OPTICS_STAR_DISPLAY);
}

/* R52A: flash V01N70; V33 R52B, ENTER (new star code) R52A. */
agc_status_t agc_job_lm_auto_optics_star_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_align_ram()->auto_optics_return != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_CODE, AGC_JOB_GOTOPOOH,
                       AGC_JOB_LM_AUTO_OPTICS_ORIENT,
                       AGC_JOB_LM_AUTO_OPTICS_STAR_DISPLAY);
}

/*
 * AZEL: OANB of (AZ, EL) -> SCAXIS (NB); PLANET at the present time;
 * POINTVSM = UNIT(REFSMMAT LOS); R60LEM.
 */
static agc_status_t orient_to_star(const agc_job_ctx_t *ctx, uint16_t az,
                                   uint16_t el)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;
    agc_interp_init(&ip);
    const agc_status_t st =
        agc_aot_oanb(&ip, az, el, &ram->scaxis, &ram->uxp, &ram->uyp);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return align_planet(ctx, align_loadtime(),
                        AGC_JOB_LM_AUTO_OPTICS_STAR_LOCATED);
}

/*
 * R52B: CLEAR 3AXISFLG (VECPOINT); detent 1..6: AZ = AOTAZ of the detent,
 * EL = 45 deg; detent 0 (COAS calibration) or 7 (COAS sighting): GETAZEL.
 */
agc_status_t agc_job_lm_auto_optics_orient(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_flag_clear(AGC_FLAG_3AXISFLG);
    const agc_sp_t d = detent_code();
    if ((d <= 0) || (d == DETENT_COAS))
    {
        return agc_job_lm_auto_optics_az_el_display(ctx); /* GETAZEL  */
    }
    return orient_to_star(ctx, agc_erase_aotaz((int) d), ELEVATION_45);
}

/* GETAZEL: flash V06N87 (AZ, EL of the sighting device). */
agc_status_t agc_job_lm_auto_optics_az_el_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_align_ram()->auto_optics_return != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return align_flash(ctx, AGC_DSKY_GOFLASH, VN_AZ_EL, AGC_JOB_GOTOPOOH,
                       AGC_JOB_LM_AUTO_OPTICS_AZ_EL_LOADED,
                       AGC_JOB_LM_AUTO_OPTICS_AZ_EL_DISPLAY);
}

/* GETAZEL V33: DCA AZ (AZ, EL in SP 2's complement), AZEL. */
agc_status_t agc_job_lm_auto_optics_az_el_loaded(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_align_ram_t *ram = agc_align_ram();
    return orient_to_star(ctx, ram->az, ram->el);
}

/* After PLANET: POINTVSM; the attitude maneuver R60LEM. */
agc_status_t agc_job_lm_auto_optics_star_located(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;
    agc_interp_init(&ip);
    const agc_mat_t refsmmat = agc_erase_refsmmat();
    ram->pointvsm            = agc_vec_unit(
                   &ip, agc_mat_vec(&ip, &refsmmat, ram->planet_los), NULL, NULL);
    agc_imu_set_pointvsm(ram->pointvsm);
    agc_status_t st = AGC_STATUS_OK;
    st = agc_imu_set_scaxis(ram->scaxis);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_imu_attitude_maneuver(ctx, AGC_JOB_LM_AUTO_OPTICS_MANEUVERED);
}

/* Back from R60LEM: COAS calibration (detent 0) recycles to R52A;
 * otherwise GOTO SAVQR52.                                                */
agc_status_t agc_job_lm_auto_optics_maneuvered(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if ((agc_align_ram()->starcode & DETENT_BITS) == 0)
    {
        return agc_job_lm_auto_optics_star_display(ctx);
    }
    return align_goto(agc_align_ram()->auto_optics_return);
}
