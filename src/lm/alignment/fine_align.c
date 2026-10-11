/*
 * fine_align.c -- R51 fine align (LM), P51-P53.agc, Luminary099
 * pp. 941-943 (labels R51, R51.1, R51C, R51F, R51I, R51E, R51.2, R51.3,
 * R51K, R51P63, R51.4).
 *
 * R51 is an interpretive subroutine (STQ QMAJ); the caller puts the job
 * to continue as in agc_align_ram()->qmaj (P52OUT, or P63's attitude
 * maneuver for R51P63).  The crew may let the LGC choose the star pair
 * (LOCSAM + PICAPAR, which Luminary calls R56) or key the codes in.  For
 * each star R52 points the AOT detent at it and AOTMARK takes the marks;
 * AOTMARK leaves the averaged present-SM LOS in STARAD +6.  STARIND
 * counts up: 0 = first star, 1 = second.
 *   STARAD    = 1st catalogue LOS (PLANVEC, SM)   6D  = 1st observed
 *   STARAD +6 = 2nd catalogue LOS (SM)            12D = 2nd observed
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/programs/alignment.h"

#define PH_FINE_ALIGN    ((uint16_t) 04024) /* OCT 04024                     */
#define CHK_ACQUIRE      ((uint16_t) 015)   /* OCT15: V50N25 00015           */
#define CHK_FINE_RECHECK ((uint16_t) 014)   /* OCT14: V50N25 00014           */
#define TSIGHT1_CS       ((agc_dp_t) 36000) /* TSIGHT1 2DEC 36000: 6 min     */
#define ALM_NO_STARS     ((agc_alarm_code_t) 0405)
#define VN_ALARM         ALIGN_VN(5, 9)     /* VB05N09                       */

static agc_status_t fine_align_phase(void)
{
    const agc_status_t st = align_phase4(PH_FINE_ALIGN);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* R51 (STQ QMAJ: the caller has set agc_align_ram()->qmaj). */
agc_status_t agc_job_lm_fine_align(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_align_ram()->qmaj != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_lm_fine_align_perform(ctx));
}

/* R51.1 (restart 4.21 enters here): PHASCHNG 04024, then R51C. */
agc_status_t agc_job_lm_fine_align_perform(agc_job_ctx_t *ctx)
{
    const agc_status_t st = fine_align_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_lm_fine_align_star_acquire(ctx));
}

/* R51C: V50N25 R1 00015; V33 star selection, ENTER the crew's stars. */
agc_status_t agc_job_lm_fine_align_star_acquire(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_align_ram()->qmaj != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_ACQUIRE, AGC_JOB_GOTOPOOH, AGC_JOB_LM_FINE_ALIGN_PICK_STARS, AGC_JOB_LM_FINE_ALIGN_FIRST_STAR));
}

/*
 * LOCSAM at TSIGHT, then R56 (PICAPAR): a pair found -> R51F -> R51E;
 * none -> R51I: alarm 405, V05N09 (PROCEED R51E, ENTER R51C).
 */
static agc_status_t fine_align_pick(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    agc_status_t st = agc_align_locsam(&ip, ram->tsight, AGC_JOB_LM_FINE_ALIGN_STARS_LOCATED, &ram->locsam);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    uint16_t cdu[3];

    align_read_cdus(cdu);
    const agc_mat_t refsmmat = agc_erase_refsmmat();
    bool            found    = false;

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

/* R51C V33: TSIGHT = now + TSIGHT1 (6 min to marking); LOCSAM. */
agc_status_t agc_job_lm_fine_align_pick_stars(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_align_ram()->tsight = agc_dp_add(&ip, align_loadtime(), TSIGHT1_CS);

    if (!AGC_ASSERT(agc_align_ram()->tsight >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (fine_align_pick(ctx));
}

/* LOCSAM resumed after its integration wait. */
agc_status_t agc_job_lm_fine_align_stars_located(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_align_ram()->tsight >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (fine_align_pick(ctx));
}

/* R51E: STARIND = 0, then R51.3. */
agc_status_t agc_job_lm_fine_align_first_star(agc_job_ctx_t *ctx)
{
    agc_align_ram()->starind = 0;

    return (agc_job_lm_fine_align_next_star(ctx));
}

/* R51.3: PHASCHNG 04024; CALL R52 (point the AOT), returning to the mark. */
agc_status_t agc_job_lm_fine_align_next_star(agc_job_ctx_t *ctx)
{
    const agc_status_t st = fine_align_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT((agc_align_ram()->starind == 0) || (agc_align_ram()->starind == 1)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_auto_optics_positioning(ctx, AGC_JOB_LM_FINE_ALIGN_MARK));
}

/* After R52: AOTMARK, then AOTSTALL (CURTAINS on a bad end). */
agc_status_t agc_job_lm_fine_align_mark(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_aotmark();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_OPTICS, AGC_JOB_LM_FINE_ALIGN_MARKED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    return (align_goto(AGC_JOB_LM_FINE_ALIGN_MARKED));
}

/*
 * After AOTSTALL.  First star (R51.4): STARSAV1 = STARAD +6 (observed),
 * PLANET at TSIGHT -> PLANVEC.  Second star: STARSAV2 = STARAD +6,
 * PHASCHNG, PLANET at TSIGHT.
 */
agc_status_t agc_job_lm_fine_align_marked(agc_job_ctx_t *ctx)
{
    if (align_stall_outcome(AGC_IMU_DEV_OPTICS) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK); /* CURTAINS */
    }

    agc_align_ram_t *ram = agc_align_ram();

    if (ram->starind == 0)
    {
        ram->starsav1 = ram->starad[1]; /* R51.4    */

        return (align_planet(ctx, ram->tsight, AGC_JOB_LM_FINE_ALIGN_FIRST_LOCATED));
    }

    ram->starsav2         = ram->starad[1];
    const agc_status_t st = fine_align_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_planet(ctx, ram->tsight, AGC_JOB_LM_FINE_ALIGN_SECOND_LOCATED));
}

/* R51.4 after PLANET: PLANVEC = LOS (ref); STARIND = 1; R51.3. */
agc_status_t agc_job_lm_fine_align_first_located(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();

    ram->planvec = ram->planet_los;
    ram->starind = 1;

    return (agc_job_lm_fine_align_next_star(ctx));
}

/*
 * Second star after PLANET: STARAD +6 = UNIT(REFSMMAT LOS), STARAD =
 * UNIT(REFSMMAT PLANVEC), 6D = STARSAV1, 12D = STARSAV2; R54.
 */
agc_status_t agc_job_lm_fine_align_second_located(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    const agc_mat_t refsmmat = agc_erase_refsmmat();

    ram->starad[1] = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, ram->planet_los), NULL, NULL);
    ram->starad[0] = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, ram->planvec), NULL, NULL);
    ram->vac6      = ram->starsav1;
    ram->vac12     = ram->starsav2;

    return (agc_align_sighting_data_display(ctx, AGC_JOB_LM_FINE_ALIGN_DATA_CHECKED));
}

/* After R54: rejected -> R51K; else AXISGEN and R55, CLEAR PFRATFLG. */
agc_status_t agc_job_lm_fine_align_data_checked(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return (agc_job_lm_fine_align_recheck(ctx));
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

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

/* After R55: CLEAR PFRATFLG, R51K. */
agc_status_t agc_job_lm_fine_align_torqued(agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_PFRATFLG);

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_PFRATFLG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_lm_fine_align_recheck(ctx));
}

/*
 * R51K / R51P63: V50N25 R1 00014 (fine align check): V34 terminates,
 * PROCEED aligns again (R51C), ENTER returns to QMAJ.
 */
agc_status_t agc_job_lm_fine_align_recheck(agc_job_ctx_t *ctx)
{
    const agc_job_id_t ret = agc_align_ram()->qmaj;

    if (!AGC_ASSERT(ret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_FINE_RECHECK, AGC_JOB_GOTOPOOH, AGC_JOB_LM_FINE_ALIGN_STAR_ACQUIRE, ret));
}
