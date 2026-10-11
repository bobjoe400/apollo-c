/*
 * lunar_surface_sighting.c -- the star sightings of P57: 2STARS, 1STAR,
 * R59RET, ASTAR (Luminary099 pp. 967-968) and R59, lunar surface star
 * acquisition (pp. 957-960): for a catalogue star R59 finds the AOT
 * detent whose 60 degree field contains it and the CURSOR and SPIRAL
 * settings that put the reticle on it (shown on V06N79), then AOTMARK
 * takes the surface marks.  STARIND: 0 = first star, 1 = second.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"

#define PH_SURFACE_ALIGN ((uint16_t) 04024) /* OCT 04024                      */
#define VN_STAR_CODE     ALIGN_VN(1, 70)  /* V01N70* VN 170                 */
#define VN_CURSOR        ALIGN_VN(6, 79)  /* V06N79 VN 679                  */
#define VN_ALARM         ALIGN_VN(5, 9)   /* VB05N09                        */
#define ALM_NOT_IN_AOT   ((agc_alarm_code_t) 0404)
#define STAR_BITS        ((agc_sp_t) 077) /* CS HIGH9: star bits 1-6        */
#define REVCNT           ((agc_sp_t) 6) /* REVCNT: just 6                 */
#define DEC227           ((agc_sp_t) 227) /* 228 and up: sun, earth, moon   */
#define POSITIONS        6              /* AOT detents 1..6               */
#define POS_SHIFT        6              /* MP BIT7: detent to bits 7-9    */
#define POS_MASK         ((agc_sp_t) 07)
#define ELEVATION_45     ((uint16_t) 010000) /* BIT13: 45 degrees              */
#define DEG30            ((agc_dp_t) 22369621) /* DEG30 2DEC .083333333 revs    */
#define DEG_HALF         ((agc_dp_t) 372825) /* DEG.5 2DEC .00138888 revs     */
#define THREE_QUARTERS   ((agc_dp_t) 201326592) /* 3/4                          */
#define ABOUTONE         ((agc_dp_t) 268435453) /* ABOUTONE 2DEC .99999999      */

static agc_status_t surface_align_phase(void)
{
    const agc_status_t st = align_phase4(PH_SURFACE_ALIGN);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* 1STAR / 2STARS tail: STARIND, PHASCHNG, R59. */
static agc_status_t star_sighting(agc_job_ctx_t *ctx, agc_sp_t starind)
{
    agc_align_ram()->starind = starind;
    const agc_status_t st = surface_align_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_lunar_surface_star_acquisition(ctx));
}

/* 2STARS: STARIND = 0 (first star). */
agc_status_t agc_lm_lunar_surface_two_stars(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (star_sighting(ctx, 0));
}

/* 1STAR: STARIND = 1 (second star, or the only one of option 3). */
agc_status_t agc_lm_lunar_surface_one_star(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (star_sighting(ctx, 1));
}

/*
 * R59 (p. 957): without a REFSMMAT go straight to AOTMARK (R59OUT);
 * otherwise flash V01N70 (star code for the acquisition).
 */
agc_status_t agc_job_lunar_surface_star_acquisition(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        return (agc_job_lunar_surface_star_mark(ctx));
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_CODE, AGC_JOB_GOTOPOOH,
                        AGC_JOB_LUNAR_SURFACE_STAR_CHOSEN,
                        AGC_JOB_LUNAR_SURFACE_STAR_ACQUISITION));
}

/*
 * INCAZ .. R59D for one AOT position: OANB of (AOTAZ, 45 deg); the star
 * (NB) at less than 30 deg from the optic axis is in the field.  Near the
 * centre (< .5 deg) CURSOR = SPIRAL = 0; otherwise YROT = the angle of
 * OA x STAR from OA x (OA x UNITX) (360 - it for a negative C2), CURSOR =
 * AZ + YROT, SPIRAL = 12 x separation + CURSOR.  false: not in field.
 */
static bool star_in_position(agc_interp_t *ip, agc_vec_t star, uint16_t az)
{
    agc_align_ram_t *ram = agc_align_ram();

    if (agc_aot_oanb(ip, az, ELEVATION_45, &ram->scaxis, &ram->uxp,
                     &ram->uyp) != AGC_STATUS_OK)
    {
        return (false);
    }

    const agc_dp_t sep =
        agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, star, ram->scaxis), 1));

    if ((sep - DEG30) >= 0)
    {
        return (false); /* NXAX     */
    }

    if ((sep - DEG_HALF) < 0)
    {
        ram->cursor = 0; /* ZSPCR    */
        ram->spiral = 0;

        return (true);
    }

    const agc_dp_t spiral12 =
        agc_dp_shl(ip, agc_dp_mul(ip, sep, THREE_QUARTERS), 4);
    const agc_vec_t ux   = agc_vec_make(AGC_DP_HALF, 0, 0);
    const agc_vec_t oaux =
        agc_vec_unit(ip, agc_vec_cross(ip, ram->scaxis, ux), NULL, NULL);
    const agc_vec_t c2 = agc_vec_unit(
        ip, agc_vec_neg(agc_vec_cross(ip, oaux, ram->scaxis)), NULL, NULL);
    const agc_vec_t oas =
        agc_vec_unit(ip, agc_vec_cross(ip, ram->scaxis, star), NULL, NULL);
    agc_dp_t thet =
        agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, oas, oaux), 1));

    if (agc_vec_dot(ip, oas, c2) < 0)
    {
        thet = agc_dp_sub(ip, ABOUTONE, thet); /* 360-THET */
    }

    const agc_dp_t yrot = agc_dp_add(
        ip, agc_dp_shr(agc_dp_pack(align_word_to_sp(az), 0), 1), thet);

    ram->cursor = agc_align_ones_to_twos(yrot); /* 1STO2S   */
    ram->spiral = agc_align_ones_to_twos(agc_dp_add(ip, spiral12, yrot));

    return (true);
}

/*
 * R59A: BESTI +STARIND = 6 x star code; not a catalogue star (0, or the
 * sun, earth, moon) -> R59OUT.  STAR = CDU*SMNB(UNIT(REFSMMAT CATLOG));
 * POSCODE 1..6 until one sees it (79DISP); none: alarm 404 (R59ALM).
 */
agc_status_t agc_job_lunar_surface_star_chosen(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram   = agc_align_ram();
    const agc_sp_t   star6 = (agc_sp_t) ((ram->starcode & STAR_BITS) * REVCNT);

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
        return (agc_job_lunar_surface_star_mark(ctx)); /* R59OUT   */
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_vec_t       star;
    const agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_cdutrig_t   t;
    agc_status_t    st = agc_star_vector(star6 / REVCNT, &star); /* CATLOG,1 */

    if (st == AGC_STATUS_OK)
    {
        star = agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, star), NULL, NULL);
        agc_guid_cdu_smnb(&t, &star);
        st = AGC_STATUS_OK;
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ram->star = star;

    for (int pos = 1; pos <= POSITIONS; pos++)
    { /* INCAZ    */
        ram->poscode = (agc_sp_t) pos;

        if (star_in_position(&ip, star, agc_erase_aotaz(pos)))
        {
            return (agc_job_lunar_surface_star_position_display(ctx));
        }
    }

    agc_alarm(ALM_NOT_IN_AOT); /* R59ALM   */
    const agc_status_t alm = AGC_STATUS_ALARM;

    if (alm != AGC_STATUS_OK)
    {
        return (alm);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALARM, AGC_JOB_GOTOPOOH,
                        AGC_JOB_LUNAR_SURFACE_STAR_MARK,
                        AGC_JOB_LUNAR_SURFACE_STAR_ACQUISITION));
}

/* 79DISP: flash V06N79 (CURSOR, SPIRAL, POSCODE); V32 back to R59. */
agc_status_t agc_job_lunar_surface_star_position_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((agc_align_ram()->poscode >= 1) &&
                    (agc_align_ram()->poscode <= POSITIONS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_CURSOR, AGC_JOB_GOTOPOOH,
                        AGC_JOB_LUNAR_SURFACE_STAR_POSITIONED,
                        AGC_JOB_LUNAR_SURFACE_STAR_ACQUISITION));
}

/* R59E: AOTCODE detent bits 7-9 = POSCODE; R59OUT. */
agc_status_t agc_job_lunar_surface_star_positioned(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    const agc_sp_t   pos = (agc_sp_t) ((ram->poscode & POS_MASK) << POS_SHIFT);

    ram->starcode = (agc_sp_t) ((ram->starcode & STAR_BITS) + pos);

    return (agc_job_lunar_surface_star_mark(ctx));
}

/* R59OUT: AOTMARK, AOTSTALL (CURTAINS on a bad end), then R59RET. */
agc_status_t agc_job_lunar_surface_star_mark(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_aotmark();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_OPTICS, AGC_JOB_LUNAR_SURFACE_STAR_MARKED,
                     &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    return (align_goto(AGC_JOB_LUNAR_SURFACE_STAR_MARKED));
}

/*
 * R59RET: first star (ASTAR): STARSAV1 = STARAD +6, PLANET at TSIGHT ->
 * VEC1, then 1STAR.  Second star: PHASCHNG, PLANET -> VEC2, SURFLINE.
 */
agc_status_t agc_job_lunar_surface_star_marked(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (align_stall_outcome(AGC_IMU_DEV_OPTICS) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK); /* CURTAINS */
    }

    agc_align_ram_t *ram = agc_align_ram();

    if (ram->starind == 0)
    {
        ram->starsav1 = ram->starad[1]; /* ASTAR    */

        return (align_planet(ctx, ram->tsight,
                             AGC_JOB_LUNAR_SURFACE_FIRST_STAR));
    }

    const agc_status_t st = surface_align_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_planet(ctx, ram->tsight, AGC_JOB_LUNAR_SURFACE_SECOND_STAR));
}

/* ASTAR after PLANET: VEC1 = the first catalogue LOS; 1STAR. */
agc_status_t agc_job_lunar_surface_first_star(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram()->vec1 = agc_align_ram()->planet_los;

    return (agc_lm_lunar_surface_one_star(ctx));
}

/* R59RET after PLANET: VEC2 = the second catalogue LOS; SURFLINE. */
agc_status_t agc_job_lunar_surface_second_star(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram()->vec2 = agc_align_ram()->planet_los;

    return (agc_lm_lunar_surface_surfline(ctx));
}
