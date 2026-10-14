/**********************************************************************
 * FILE NAME: planet.c
 *
 * PURPOSE:
 *   PLANET as a job step: the reference line of sight of the star or body
 *   coded in STARCODE (AOTCODE in the LM) at TSIGHT.  Code 00 (planet)
 *   asks the crew for the vector with V06N88; the CSM always runs LOCSAM
 *   (CSMCONIC, which may wait in INTSTALL) and corrects for aberration.
 *   The vector math is agc_align_planet_vector() (locsam.c); this file
 *   holds the display and the waits.
 *
 *   The caller names the job to continue as (the AGC's QMIN return); the
 *   result is left in agc_align_ram()->planet_los.
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 782-784 (PLANET, NOTPLAN, CALSAM1, CORPLAN)
 *   Luminary099 P51-P53.agc pp. 961-962 (PLANET)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   STARCODE, TSIGHT, STARAD, PLANET_RET, PLANET_LOS: agc_align_ram()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_vec_zero          vector arithmetic of the interpreter (interp.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_state_vehicle     vehicle state (state.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/programs/alignment.h"
#include "internal.h"

#define VN_PLANET_VECTOR ALIGN_VN(6, 88)  /* VNPLANV VN 0688              */
#define STAR_CODE_MASK   ((agc_sp_t) 077) /* CS HIGH9: bits 1-6           */
#define STAR_WORDS       6                /* SIGHTSIX / REVCNT            */

/**********************************************************************
 * AGC: CATLOG (Comanche055 STAR_TABLES.agc p. 1393; Luminary099
 *      CONTROLLED_CONSTANTS.agc p. 51)
 *
 * Star code x 6 (the CATLOG index) from STARCODE / AOTCODE bits 1-6.
 *
 * FUNCTION NAME: star6_of_code
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_sp_t -- star code x 6 (the CATLOG index)
 **********************************************************************/
static agc_sp_t star6_of_code(void)
{
    const agc_sp_t code = (agc_sp_t) (agc_align_ram()->starcode & STAR_CODE_MASK); /* STARCODE / AOTCODE bits 1-6, unscaled */

    return ((agc_sp_t) (code * STAR_WORDS));
}

/**********************************************************************
 * AGC: NOSAM (Comanche055 P51-P53.agc p. 783)
 *
 * NOSAM .. CORPLAN: the vector, then continue as the caller asked.
 *
 * FUNCTION NAME: planet_compute
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t planet_compute(void)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_status_t st = agc_align_planet_vector(&ip, star6_of_code(), ram->starind, ram->tsight, AGC_JOB_PLANET_RESUME, &ram->planet_los); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL sleep  */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_goto(ram->planet_ret));
}

/**********************************************************************
 * AGC: ENTER (Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc p. 323;
 *      Luminary099 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc p. 406)
 *
 * The V06N88 flash: V33 takes the loaded vector; ENTER (and, in the LM,
 * V34) shows it again.
 *
 * FUNCTION NAME: planet_flash
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
static agc_status_t planet_flash(const agc_job_ctx_t *ctx)
{
    const agc_job_id_t term = (agc_state_vehicle() == AGC_VEHICLE_CSM) ? AGC_JOB_GOTOPOOH : AGC_JOB_PLANET_FLASH; /* per vehicle */

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_PLANET_VECTOR, term, AGC_JOB_PLANET_LOADED, AGC_JOB_PLANET_FLASH));
}

/**********************************************************************
 * AGC: PLANET (Comanche055 P51-P53.agc p. 782; Luminary099 P51-P53.agc p.
 *      961)
 *
 * PLANET entry: TSIGHT = t; (LM) STARAD = 0 for the N88 load; BESTI or
 * BESTJ = code x 6 (set by the vector routine); code 00 flashes V06N88.
 *
 * FUNCTION NAME: align_planet
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   t         agc_dp_t               I    time, cs 2^28
 *   ret       agc_job_id_t           I    job the routine continues as when
 *                                         it ends (the AGC return address)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t align_planet(const agc_job_ctx_t *ctx, agc_dp_t t, agc_job_id_t ret)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->tsight     = t;
    ram->planet_ret = ret;

    if (star6_of_code() != 0)
    {
        return (planet_compute()); /* NOTPLAN        */
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        ram->starad[0] = agc_vec_zero(); /* STORE STARAD   */
    }

    return (planet_flash(ctx));
}

/**********************************************************************
 * AGC: PLANET (Comanche055 P51-P53.agc p. 782; Luminary099 P51-P53.agc p.
 *      961)
 *
 * PLANET: V06N88 shown again.
 *
 * FUNCTION NAME: agc_job_planet_flash
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
agc_status_t agc_job_planet_flash(agc_job_ctx_t *ctx)
{
    return (planet_flash(ctx));
}

/**********************************************************************
 * AGC: PLANET (Comanche055 P51-P53.agc p. 782; Luminary099 P51-P53.agc p.
 *      961)
 *
 * PLANET: V33 on V06N88 (vector loaded in STARSAV3 / STARAD).
 *
 * FUNCTION NAME: agc_job_planet_loaded
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_planet_loaded(void)
{
    return (planet_compute());
}

/**********************************************************************
 * AGC: PLANET (Comanche055 P51-P53.agc p. 782; Luminary099 P51-P53.agc p.
 *      961)
 *
 * PLANET: resumed after waiting for the integration (LOCSAM).
 *
 * FUNCTION NAME: agc_job_planet_resume
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_planet_resume(void)
{
    return (planet_compute());
}
