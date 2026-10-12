/*
 * imu_orientation_determination.c -- P51 IMU orientation determination
 * and P53, its backup (P53 EQUALS P51), CSM.  P51-P53.agc, Comanche055
 * pp. 766-769 (labels P51, P51A, P51AA, P51B, P51C, P51C.1, P51C.2, P51D,
 * P51E, P51F, P51G).
 *
 * With the IMU on, the crew may coarse align to zero gimbal angles
 * (ENTER on V50N25 00015), then marks two stars (R53, or R56 in P53).
 * The two observed lines of sight (present SM) and the two catalogue
 * lines of sight (reference) give REFSMMAT directly (AXISGEN).
 *   STARAD = 1st observed (SM)     6D  = 1st catalogue (PLANVEC, ref)
 *   STARAD +6 = 2nd observed       12D = 2nd catalogue
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/programs/alignment.h"

#define IMODES30_IMU_OFF       ((uint16_t) 0400)  /* BIT9: IMU not operating        */
#define ALM_IMU_OFF            ((agc_alarm_code_t) 0210)
#define CHK_ACQUIRE            ((uint16_t) 015)   /* PRFMSTAQ = OCT15               */
#define PH_ORIENTATION         ((uint16_t) 05024) /* OCT 05024, OCT 13000           */
#define PH_ORIENTATION_RESTART ((uint16_t) 00014) /* OCT 00014                      */
#define VN_GIMBALS             ALIGN_VN(6, 22)    /* V6N22  VN 0622                 */
#define VN_COARSE_VERB         ALIGN_VN(41, 0)    /* V41K   VN 4100                 */
#define MM_BACKUP_ORIENT       ((agc_sp_t) 53)    /* CHECKMM 53                     */

static void orientation_phase(agc_job_id_t restart_at)
{
    align_phase_job(PH_ORIENTATION, restart_at, ALIGN_PRIO13);
}

/*
 * P51 / P53: alarm 210 and P00 when the IMU is off (IMODES30 bit 9);
 * otherwise R02ZERO (SET IMUSE) and P51AA.
 */
agc_status_t agc_job_csm_imu_orientation_determination(agc_job_ctx_t *ctx)
{
    if ((agc_erase_imodes30() & IMODES30_IMU_OFF) != 0u)
    {
        agc_alarm(ALM_IMU_OFF);

        return (agc_exec_gotopooh());
    }

    const agc_status_t st = agc_imu_set_in_use(); /* R02ZERO  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_csm_orientation_acquire_display(ctx));
}

/* P51AA: V50N25 R1 00015; V34 P00, V33 P51B (mark), ENTER coarse align. */
agc_status_t agc_job_csm_orientation_acquire_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_ACQUIRE, AGC_JOB_GOTOPOOH, AGC_JOB_CSM_ORIENTATION_MARK_SETUP, AGC_JOB_CSM_ORIENTATION_COARSE));
}

/* P51AA ENTER: PHASCHNG, THETAD = 0, GODSPRET V06N22 (job continues). */
agc_status_t agc_job_csm_orientation_coarse(agc_job_ctx_t *ctx)
{
    orientation_phase(AGC_JOB_CSM_ORIENTATION_COARSE);

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_thetad(i, 0); /* P51ZERO  */
    }

    return (align_flash(ctx, AGC_DSKY_GODSPRET, VN_GIMBALS, AGC_JOB_CSM_ORIENTATION_COARSE_VERB, AGC_JOB_CSM_ORIENTATION_COARSE_VERB, AGC_JOB_CSM_ORIENTATION_COARSE_VERB));
}

/* GODSPRET V41 (coarse align verb); then IMUCOARS and IMUSTALL. */
agc_status_t agc_job_csm_orientation_coarse_verb(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_erase_thetad(0) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GODSPRET, VN_COARSE_VERB, AGC_JOB_CSM_ORIENTATION_COARSE_START, AGC_JOB_CSM_ORIENTATION_COARSE_START, AGC_JOB_CSM_ORIENTATION_COARSE_START));
}

/* IMUCOARS to THETAD (all zero) and IMUSTALL (CURTAINS on a bad end). */
agc_status_t agc_job_csm_orientation_coarse_start(void)
{
    uint16_t thetad[3];

    for (int i = 0; i < 3; i++)
    {
        thetad[i] = agc_erase_thetad(i);
    }

    agc_status_t st = agc_imu_coarse(thetad); /* IMUCOARS */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_CSM_ORIENTATION_COARSE_DONE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    return (align_goto(AGC_JOB_CSM_ORIENTATION_COARSE_DONE));
}

/* After the coarse align: IMUFIN20 and IMUSTALL. */
agc_status_t agc_job_csm_orientation_coarse_done(void)
{
    if (align_stall_outcome(AGC_IMU_DEV_IMU) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK); /* CURTAINS */
    }

    agc_status_t st = agc_imu_fine(); /* IMUFIN20 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_CSM_ORIENTATION_FINE_DONE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    return (align_goto(AGC_JOB_CSM_ORIENTATION_FINE_DONE));
}

/* Coarse align done: PHASCHNG, recycle to P51AA for the fine align. */
agc_status_t agc_job_csm_orientation_fine_done(agc_job_ctx_t *ctx)
{
    if (align_stall_outcome(AGC_IMU_DEV_IMU) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK);
    }

    orientation_phase(AGC_JOB_CSM_ORIENTATION_ACQUIRE_DISPLAY);

    return (agc_job_csm_orientation_acquire_display(ctx));
}

/*
 * P51B: PHASCHNG 00014; STARIND = 0; SET1/PDT, GCOMP = 0, SET DRIFTFLG;
 * CLEAR TARG2FLG (star, not landmark); MARKINDX = 1; P51C.
 */
agc_status_t agc_job_csm_orientation_mark_setup(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_status_t     st  = align_phase4(PH_ORIENTATION_RESTART);

    ram->starind = 0;
    st           = align_set1_pdt();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_clear(AGC_FLAG_TARG2FLG);
    ram->markindx = 1;

    return (agc_job_csm_orientation_mark_star(ctx));
}

/* P51C: PHASCHNG; P53 marks with R56, P51 with R53; both to P51C.2. */
agc_status_t agc_job_csm_orientation_mark_star(agc_job_ctx_t *ctx)
{
    orientation_phase(AGC_JOB_CSM_ORIENTATION_MARK_STAR);

    if (agc_phase_checkmm(MM_BACKUP_ORIENT))
    {
        return (agc_csm_alternate_los_sighting_mark(ctx, AGC_JOB_CSM_ORIENTATION_MARKED));
    }

    return (agc_csm_sighting_mark(ctx, AGC_JOB_CSM_ORIENTATION_MARKED));
}

/*
 * P51E: PHASCHNG; MKRELEAS; after the first star STARIND = 1 and P51C
 * again; after the second, P51F (PLANET for the second catalogue LOS).
 */
static agc_status_t orientation_after_mark(const agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();

    orientation_phase(AGC_JOB_CSM_ORIENTATION_MARK_STAR);
    const agc_status_t st = agc_sxtmark_mkreleas();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (ram->starind == 0)
    {
        ram->starind = 1;

        return (align_goto(AGC_JOB_CSM_ORIENTATION_MARK_STAR));
    }

    orientation_phase(AGC_JOB_CSM_ORIENTATION_SECOND_STAR);

    return (align_planet(ctx, ram->tsight, AGC_JOB_CSM_ORIENTATION_SECOND_STAR));
}

/*
 * P51C.2: SXTSM.  First star (P51D): STARSAV1 = LOS, PLANET at TSIGHT
 * -> PLANVEC.  Second star: STARSAV2 = LOS.  Then P51E.
 */
agc_status_t agc_job_csm_orientation_marked(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    agc_vec_t          los;
    const agc_status_t st = agc_csm_sxtsm_marks(&ip, &los);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (ram->starind != 0)
    {
        ram->starsav2 = los;

        return (orientation_after_mark(ctx));
    }

    ram->starsav1 = los; /* P51D     */

    return (align_planet(ctx, ram->tsight, AGC_JOB_CSM_ORIENTATION_FIRST_STAR));
}

/* P51D after PLANET: PLANVEC = the first catalogue LOS; P51E. */
agc_status_t agc_job_csm_orientation_first_star(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();

    ram->planvec = ram->planet_los;

    if (!AGC_ASSERT(ram->starind == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (orientation_after_mark(ctx));
}

/*
 * P51F after PLANET: 12D = 2nd catalogue, 6D = PLANVEC, STARAD =
 * STARSAV1, STARAD +6 = STARSAV2; CHKSDATA (R54).
 */
agc_status_t agc_job_csm_orientation_second_star(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram();

    ram->vac12     = ram->planet_los;
    ram->vac6      = ram->planvec;
    ram->starad[0] = ram->starsav1;
    ram->starad[1] = ram->starsav2;

    if (!AGC_ASSERT(ram->starind == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_align_sighting_data_display(ctx, AGC_JOB_CSM_ORIENTATION_DATA_CHECKED));
}

/*
 * After CHKSDATA: rejected -> P51AA; accepted (P51G): AXISGEN gives
 * REFSMMAT in XDC; CLEAR REFSMFLG, MATMOVE, SET REFSMFLG, ENDP50S.
 */
agc_status_t agc_job_csm_orientation_data_checked(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return (agc_job_csm_orientation_acquire_display(ctx));
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    const agc_vec_t a[2] = {ram->starad[0], ram->starad[1]};
    const agc_vec_t b[2] = {ram->vac6, ram->vac12};
    agc_status_t    st   = agc_align_fine_solve(&ip, a, b, &ram->xdc); /* AXISGEN */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_clear(AGC_FLAG_REFSMFLG);
    agc_erase_set_refsmmat(ram->xdc); /* MATMOVE  */
    agc_flag_set(AGC_FLAG_REFSMFLG);

    return (agc_exec_gotopooh()); /* ENDP50S  */
}
