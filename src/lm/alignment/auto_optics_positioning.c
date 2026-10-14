/**********************************************************************
 * FILE NAME: auto_optics_positioning.c
 *
 * PURPOSE:
 *   R52 automatic optics positioning (LM).  The LM's AOT cannot be
 *   driven, so R52 turns the vehicle instead: the star code (with AOT
 *   detent 2 by default) is shown on V01N70; the optic axis of the chosen
 *   detent (or the AZ, EL keyed in on V06N87 for COAS codes 0 and 7)
 *   becomes SCAXIS, the star's line of sight POINTVSM, and R60LEM
 *   maneuvers the LM to put one on the other.
 *
 * AGC SOURCE:
 *   Luminary099 P51-P53.agc pp. 955-956 (R52, R52A, R52B, GETAZEL, AZEL)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   STARCODE, BESTI/J, SCAXIS, UXP, UYP, AZ, EL, SAVQR52: agc_align_ram()
 *   AOTAZ, AOTEL  uint16_t  I    CDU words      detent pad loads
 *   3AXISFLG      flag      O                   VECPOINT
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_ram         alignment routines of this lane (alignment.h)
 *   agc_imu_*             IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_vec_unit          vector arithmetic of the interpreter (interp.h)
 *   agc_mat_vec           matrix arithmetic of the interpreter (interp.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_clear        flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_aot_oanb          AOTMARK routines (alignment.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/programs/alignment.h"

#define VN_STAR_CODE ALIGN_VN(1, 70)     /* V01N70 VN 0170                 */
#define VN_AZ_EL     ALIGN_VN(6, 87)     /* V06N87 VN 687                  */
#define STAR_WORDS   ((agc_sp_t) 6)      /* 1/6TH                          */
#define DETENT_2     ((agc_sp_t) 0200)   /* BIT8: detent position 2        */
#define DETENT_SHIFT 6                   /* MP BIT9: bits 7-9 -> 1-3       */
#define DETENT_MASK  ((agc_sp_t) 07)     /* three detent bits */
#define DETENT_COAS  ((agc_sp_t) 7)      /* code 7: COAS sighting          */
#define DETENT_BITS  ((agc_sp_t) 0700)   /* HIGH9 & a star code < 01000    */
#define ELEVATION_45 ((uint16_t) 010000) /* BIT13: 45 degrees              */

/**********************************************************************
 * R52 (Luminary p. 955): STQ SAVQR52; STARCODE = BESTI +STARIND / 6 +
 * detent 2; then R52A.
 *
 * FUNCTION NAME: agc_lm_auto_optics_positioning
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   ret       agc_job_id_t           I    job the routine continues as when
 *                                         it ends (the AGC return address)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_lm_auto_optics_positioning(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->auto_optics_return = ret;                                           /* SAVQR52  */
    const agc_sp_t star6    = (ram->starind == 0) ? ram->besti : ram->bestj; /* BESTI +STARIND, star code x 6 (unscaled); INDEX STARIND: BESTI or BESTJ */

    ram->starcode = (agc_sp_t) ((star6 / STAR_WORDS) + DETENT_2);

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_CODE, AGC_JOB_GOTOPOOH, AGC_JOB_LM_AUTO_OPTICS_ORIENT, AGC_JOB_LM_AUTO_OPTICS_STAR_DISPLAY));
}

/**********************************************************************
 * AGC: R52A (Luminary099 P51-P53.agc p. 955)
 *
 * R52A: flash V01N70; V33 R52B, ENTER (new star code) R52A.
 *
 * FUNCTION NAME: agc_job_lm_auto_optics_star_display
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
agc_status_t agc_job_lm_auto_optics_star_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_CODE, AGC_JOB_GOTOPOOH, AGC_JOB_LM_AUTO_OPTICS_ORIENT, AGC_JOB_LM_AUTO_OPTICS_STAR_DISPLAY));
}

/**********************************************************************
 * AGC: AZEL (Luminary099 P51-P53.agc p. 956)
 *
 * AZEL: OANB of (AZ, EL) -> SCAXIS (NB); PLANET at the present time;
 * POINTVSM = UNIT(REFSMMAT LOS); R60LEM.
 *
 * FUNCTION NAME: orient_to_star
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   az        uint16_t               I    AOT azimuth, CDU word (2^15 = 1
 *                                         rev)
 *   el        uint16_t               I    AOT elevation, CDU word (2^15 = 1
 *                                         rev)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t orient_to_star(const agc_job_ctx_t *ctx, uint16_t az, uint16_t el)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    agc_aot_oanb(&ip, az, el, &ram->scaxis, &ram->uxp, &ram->uyp);

    return (align_planet(ctx, align_loadtime(), AGC_JOB_LM_AUTO_OPTICS_STAR_LOCATED));
}

/**********************************************************************
 * AGC: R52B (Luminary099 P51-P53.agc p. 955)
 *
 * R52B: CLEAR 3AXISFLG (VECPOINT); detent 1..6: AZ = AOTAZ of the detent,
 * EL = 45 deg; detent 0 (COAS calibration) or 7 (COAS sighting): GETAZEL.
 *
 * FUNCTION NAME: agc_job_lm_auto_optics_orient
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
agc_status_t agc_job_lm_auto_optics_orient(agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_3AXISFLG);

    /* Detent code (bits 7-9 of STARCODE / AOTCODE). */
    const agc_sp_t d = (agc_sp_t) ((agc_align_ram()->starcode >> DETENT_SHIFT) & DETENT_MASK); /* detent code 0..7 (STARCODE bits 7-9), unscaled */

    if ((d == 0) || (d == DETENT_COAS))
    {
        return (agc_job_lm_auto_optics_az_el_display(ctx)); /* GETAZEL  */
    }

    return (orient_to_star(ctx, agc_erase_aotaz((int) d), ELEVATION_45));
}

/**********************************************************************
 * AGC: GETAZEL (Luminary099 P51-P53.agc p. 956)
 *
 * GETAZEL: flash V06N87 (AZ, EL of the sighting device).
 *
 * FUNCTION NAME: agc_job_lm_auto_optics_az_el_display
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
agc_status_t agc_job_lm_auto_optics_az_el_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_AZ_EL, AGC_JOB_GOTOPOOH, AGC_JOB_LM_AUTO_OPTICS_AZ_EL_LOADED, AGC_JOB_LM_AUTO_OPTICS_AZ_EL_DISPLAY));
}

/**********************************************************************
 * AGC: GETAZEL (Luminary099 P51-P53.agc p. 956)
 *
 * GETAZEL V33: DCA AZ (AZ, EL in SP 2's complement), AZEL.
 *
 * FUNCTION NAME: agc_job_lm_auto_optics_az_el_loaded
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
agc_status_t agc_job_lm_auto_optics_az_el_loaded(agc_job_ctx_t *ctx)
{
    const agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    return (orient_to_star(ctx, ram->az, ram->el));
}

/**********************************************************************
 * AGC: PLANET (Luminary099 P51-P53.agc p. 961)
 *
 * After PLANET: POINTVSM; the attitude maneuver R60LEM.
 *
 * FUNCTION NAME: agc_job_lm_auto_optics_star_located
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
agc_status_t agc_job_lm_auto_optics_star_located(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */

    ram->pointvsm   = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, ram->planet_los), NULL, NULL);
    agc_imu_set_pointvsm(ram->pointvsm);
    agc_status_t st = AGC_STATUS_OK; /* status of the call */

    st = agc_imu_set_scaxis(ram->scaxis);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_imu_attitude_maneuver(ctx, AGC_JOB_LM_AUTO_OPTICS_MANEUVERED));
}

/**********************************************************************
 * AGC: R60LEM (Luminary099 R60_62.agc p. 473)
 *
 * Back from R60LEM: COAS calibration (detent 0) recycles to R52A; otherwise
 * GOTO SAVQR52.
 *
 * FUNCTION NAME: agc_job_lm_auto_optics_maneuvered
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
agc_status_t agc_job_lm_auto_optics_maneuvered(agc_job_ctx_t *ctx)
{
    if ((agc_align_ram()->starcode & DETENT_BITS) == 0)
    {
        return (agc_job_lm_auto_optics_star_display(ctx));
    }

    return (align_goto(agc_align_ram()->auto_optics_return));
}
