/*
 * imu_orientation_determination.c -- P51 IMU orientation determination
 * (LM), P51-P53.agc, Luminary099 pp. 949-951 (labels P51, P51B, P51C,
 * P51D, P51E, P51G) and IMUCHK (p. 982).
 *
 * As in the CSM: an optional coarse align to zero gimbal angles, then
 * two AOTMARK sightings; the two observed lines of sight (present SM,
 * averaged by AOTMARK into STARAD +6) and the two catalogue lines of
 * sight (reference) give REFSMMAT through AXISGEN.
 *   STARAD    = 1st observed (STARSAV1)   6D  = 1st catalogue (PLANVEC)
 *   STARAD +6 = 2nd observed (STARSAV2)   12D = 2nd catalogue
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/programs/alignment_internal.h"

#define IMODES30_IMU_OFF ((uint16_t) 0400) /* BIT9                           */
#define ALM_IMU_OFF      ((agc_alarm_code_t) 0210)
#define CHK_ACQUIRE      ((uint16_t) 015)   /* OCT15                          */
#define PH_P51           ((uint16_t) 04024) /* OCT 04024                      */
#define PH_P51B          ((uint16_t) 00014) /* OCT 00014                      */
#define VN_GIMBALS       ALIGN_VN(6, 22)    /* V06N22                         */
#define VN_COARSE_VERB   ALIGN_VN(41, 0)    /* V41K VN 4100                   */

static agc_status_t p51_phase(void)
{
    const agc_status_t st = align_phase4(PH_P51);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/*
 * IMUCHK (Luminary p. 982): with the IMU off (IMODES30 bit 9) alarm 210
 * and GOTOPOOH (returns AGC_STATUS_ALARM: the caller just ends);
 * otherwise SET IMUSE.
 */
agc_status_t agc_lm_imu_check(void)
{
    if ((agc_erase_imodes30() & IMODES30_IMU_OFF) != 0u)
    {
        agc_alarm(ALM_IMU_OFF);
        const agc_status_t alm = AGC_STATUS_ALARM;
        if (alm != AGC_STATUS_OK)
        {
            return alm;
        }
        const agc_status_t st = agc_exec_gotopooh();
        return (st == AGC_STATUS_OK) ? AGC_STATUS_ALARM : st;
    }
    agc_flag_set(AGC_FLAG_IMUSE);
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_IMUSE)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* P51: IMUCHK, then P51 +2. */
agc_status_t agc_job_lm_imu_orientation_determination(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_status_t st = agc_lm_imu_check();
    if (st != AGC_STATUS_OK)
    {
        return (st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st;
    }
    return agc_job_lm_orientation_acquire_display(ctx);
}

/* P51 +2: V50N25 R1 00015; V33 P51B, ENTER coarse align. */
agc_status_t agc_job_lm_orientation_acquire_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return align_flash(ctx, AGC_DSKY_GOPERF1, CHK_ACQUIRE, AGC_JOB_GOTOPOOH,
                       AGC_JOB_LM_ORIENTATION_MARK_SETUP,
                       AGC_JOB_LM_ORIENTATION_COARSE);
}

/* ENTER: PHASCHNG, THETAD = 0, GODSPRET V06N22. */
agc_status_t agc_job_lm_orientation_coarse(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = p51_phase();
    for (int i = 0; (i < 3) && (st == AGC_STATUS_OK); i++)
    {
        agc_erase_set_thetad(i, 0);
        st = AGC_STATUS_OK;
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return align_flash(
        ctx, AGC_DSKY_GODSPRET, VN_GIMBALS, AGC_JOB_LM_ORIENTATION_COARSE_VERB,
        AGC_JOB_LM_ORIENTATION_COARSE_VERB, AGC_JOB_LM_ORIENTATION_COARSE_VERB);
}

/* GODSPRET V41, then CALL COARSE. */
agc_status_t agc_job_lm_orientation_coarse_verb(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_erase_thetad(0) == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return align_flash(ctx, AGC_DSKY_GODSPRET, VN_COARSE_VERB,
                       AGC_JOB_LM_ORIENTATION_COARSE_START,
                       AGC_JOB_LM_ORIENTATION_COARSE_START,
                       AGC_JOB_LM_ORIENTATION_COARSE_START);
}

/* COARSE (IMUCOARS, IMUFINE and their stalls), returning to P51 +2. */
agc_status_t agc_job_lm_orientation_coarse_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_erase_thetad(1) == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return align_coarse(AGC_JOB_LM_ORIENTATION_COARSE_DONE);
}

/* After COARSE: PHASCHNG, P51 +2. */
agc_status_t agc_job_lm_orientation_coarse_done(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_status_t st = p51_phase();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_job_lm_orientation_acquire_display(ctx);
}

/* P51B: PHASCHNG 00014; NCOARSE; STARIND = 0; P51C. */
agc_status_t agc_job_lm_orientation_mark_setup(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = align_phase4(PH_P51B);
    if (st == AGC_STATUS_OK)
    {
        st = align_ncoarse();
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_align_ram()->starind = 0;
    return agc_job_lm_orientation_mark_star(ctx);
}

/* P51C: PHASCHNG; AOTMARK (R53); AOTSTALL. */
agc_status_t agc_job_lm_orientation_mark_star(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = p51_phase();
    if (st == AGC_STATUS_OK)
    {
        st = agc_aotmark();
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    bool done = false;
    st = align_stall(AGC_IMU_DEV_OPTICS, AGC_JOB_LM_ORIENTATION_MARKED, &done);
    if ((st != AGC_STATUS_OK) || !done)
    {
        return (st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st;
    }
    return align_goto(AGC_JOB_LM_ORIENTATION_MARKED);
}

/*
 * After AOTSTALL.  First star: STARSAV1 = STARAD +6; P51D: PHASCHNG,
 * PLANET at TSIGHT -> PLANVEC.  Second star (STARSAV2 already left by
 * AOTMARK): P51E, PLANET at TSIGHT.
 */
agc_status_t agc_job_lm_orientation_marked(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (align_stall_outcome(AGC_IMU_DEV_OPTICS) != AGC_STATUS_OK)
    {
        return AGC_STATUS_OK; /* CURTAINS */
    }
    agc_align_ram_t   *ram = agc_align_ram();
    const agc_status_t st  = p51_phase();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (ram->starind != 0)
    {
        return align_planet(ctx, ram->tsight,
                            AGC_JOB_LM_ORIENTATION_SECOND_STAR);
    }
    ram->starsav1 = ram->starad[1];
    return align_planet(ctx, ram->tsight, AGC_JOB_LM_ORIENTATION_FIRST_STAR);
}

/* P51D after PLANET: PLANVEC; STARIND = 1; P51C +1. */
agc_status_t agc_job_lm_orientation_first_star(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    ram->planvec         = ram->planet_los;
    ram->starind         = 1;
    return agc_job_lm_orientation_mark_star(ctx);
}

/* P51E after PLANET: 12D, 6D, STARAD, STARAD +6; CHKSDATA. */
agc_status_t agc_job_lm_orientation_second_star(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    ram->vac12           = ram->planet_los;
    ram->vac6            = ram->planvec;
    ram->starad[0]       = ram->starsav1;
    ram->starad[1]       = ram->starsav2;
    if (!AGC_ASSERT(ram->starind == 1))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_align_sighting_data_display(ctx,
                                           AGC_JOB_LM_ORIENTATION_DATA_CHECKED);
}

/*
 * After CHKSDATA: rejected -> P51 +2; accepted (P51G): AXISGEN,
 * REFSMMAT = XDC, SET REFSMFLG, GOTOPOOH.
 */
agc_status_t agc_job_lm_orientation_data_checked(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return agc_job_lm_orientation_acquire_display(ctx);
    }
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;
    agc_interp_init(&ip);
    const agc_vec_t    a[2] = {ram->starad[0], ram->starad[1]};
    const agc_vec_t    b[2] = {ram->vac6, ram->vac12};
    const agc_status_t st   = agc_align_fine_solve(&ip, a, b, &ram->xdc);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_erase_set_refsmmat(ram->xdc); /* MATMOVE  */
    agc_flag_set(AGC_FLAG_REFSMFLG);
    return agc_exec_gotopooh(); /* FINIS    */
}
