/*
 * planet.c -- PLANET as a job step (P51-P53.agc, Comanche pp.
 * 782-784, Luminary pp. 961-962): the reference line of sight of the
 * star / body coded in STARCODE (AOTCODE in the LM) at TSIGHT.  Code 00
 * (planet) asks the crew for the vector with V06N88; the CSM always runs
 * LOCSAM (CSMCONIC, which may wait in INTSTALL) and corrects for
 * aberration.  The vector math is agc_align_planet_vector()
 * (locsam.c); this file holds the display and the waits.
 *
 * The caller names the job to continue as (the AGC's QMIN / GCTR return);
 * the result is left in agc_align_ram()->planet_los.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/programs/alignment.h"

#define VN_PLANET_VECTOR ALIGN_VN(6, 88)  /* VNPLANV VN 0688              */
#define STAR_CODE_MASK   ((agc_sp_t) 077) /* CS HIGH9: bits 1-6           */
#define STAR_WORDS       6                /* SIGHTSIX / REVCNT            */

/* Star code x 6 (the CATLOG index) from STARCODE / AOTCODE bits 1-6. */
static agc_sp_t star6_of_code(void)
{
    const agc_sp_t code = (agc_sp_t) (agc_align_ram()->starcode & STAR_CODE_MASK);

    return ((agc_sp_t) (code * STAR_WORDS));
}

/* NOSAM .. CORPLAN: the vector, then continue as the caller asked. */
static agc_status_t planet_compute(void)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);

    if (!AGC_ASSERT((ram->starind == 0) || (ram->starind == 1)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = agc_align_planet_vector(&ip, star6_of_code(), ram->starind, ram->tsight, AGC_JOB_PLANET_RESUME, &ram->planet_los);

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

/* The V06N88 flash: V33 takes the loaded vector; ENTER (and, in the LM,
 * V34) shows it again.                                                    */
static agc_status_t planet_flash(const agc_job_ctx_t *ctx)
{
    const agc_job_id_t term = (agc_state_vehicle() == AGC_VEHICLE_CSM) ? AGC_JOB_GOTOPOOH : AGC_JOB_PLANET_FLASH;

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_PLANET_VECTOR, term, AGC_JOB_PLANET_LOADED, AGC_JOB_PLANET_FLASH));
}

/*
 * PLANET entry: TSIGHT = t; (LM) STARAD = 0 for the N88 load; BESTI or
 * BESTJ = code x 6 (set by the vector routine); code 00 flashes V06N88.
 */
agc_status_t align_planet(const agc_job_ctx_t *ctx, agc_dp_t t, agc_job_id_t ret)
{
    agc_align_ram_t *ram = agc_align_ram();

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

/* PLANET: V06N88 shown again. */
agc_status_t agc_job_planet_flash(agc_job_ctx_t *ctx)
{
    return (planet_flash(ctx));
}

/* PLANET: V33 on V06N88 (vector loaded in STARSAV3 / STARAD). */
agc_status_t agc_job_planet_loaded(void)
{
    if (!AGC_ASSERT(agc_align_ram()->planet_ret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (planet_compute());
}

/* PLANET: resumed after waiting for the integration (LOCSAM). */
agc_status_t agc_job_planet_resume(void)
{
    if (!AGC_ASSERT(agc_align_ram()->planet_ret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (planet_compute());
}
