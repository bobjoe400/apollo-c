/*
 * lunar_surface_gravity.c -- the IMU work of P57 lunar surface align:
 * the gravity determination GVDETER (with LUNG / GRABGRAV / ADDGRAV and
 * GRAVEL, the 180 degree turn of the stable member about the gravity
 * vector that cancels the PIPA biases), the gyro torquing GYROTRIM /
 * JUSTTRIM and the coarse align for large corrections COATRIM / CORSIT.
 * P51-P53.agc, Luminary099 pp. 963-967, 976-977.
 *
 * LUNG and GYROTRIM are interpretive subroutines (STQ QMIN); their caller
 * names the job to continue as in agc_align_ram()->lunar_gravity_return.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/guidance/guidance.h"
#include "agc/programs/alignment_internal.h"

#define PH_P57          ((uint16_t) 04024)  /* OCT 04024                     */
#define THETA_42DEG     ((uint16_t) 07357)  /* 42DEG OCT 07357               */
#define THETA_MINUS_42  ((uint16_t) 070420) /* COM of 42DEG                  */
#define THETA_35DEG     ((uint16_t) 06211)  /* 35DEG OCT 06211               */
#define GRAVITY_SAMPLES ((agc_sp_t) -20)    /* T/2SEC DEC -20                */
#define PIPA_INTERVAL   ((agc_sp_t) 031000) /* CA PRIO31, TS 1/PIPADT        */
#define SAMPLE_DT       ((uint16_t) 200)    /* 2SECS                         */
#define VN_GRAVITY      ALIGN_VN(6, 4)      /* DISGRVER VN 0604              */
#define VN_GIMBALS      ALIGN_VN(6, 22)     /* V06N22                        */

/* The CDUTRIG of the last gravity sample (*NBSM* in GRAVEL uses it).   */
static agc_cdutrig_t lung_trig;

static agc_status_t p57_phase(void)
{
    const agc_status_t st = align_phase4(PH_P57);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/*
 * LUNG (p. 965): STQ QMIN; GACC = 0; PHASCHNG; IMUCOARS / IMUFINE to
 * THETAD (COARSE), then the sampling set-up.
 */
static agc_status_t lung(agc_job_id_t ret)
{
    agc_align_ram_t *ram = agc_align_ram();

    ram->lunar_gravity_return = ret; /* QMIN     */
    ram->gacc                 = agc_vec_zero();
    const agc_status_t st = p57_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_coarse(AGC_JOB_LUNAR_GRAVITY_COARSED));
}

/* PIPSRINE (PIPASR +3): read and zero the PIPAs into DELV. */
static agc_status_t read_pipas(void)
{
    const agc_status_t st = agc_guid_pipasr(&agc_guid_state()->avg, false);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* GREED: TWIDDLE 2SECS GRABGRAV; ENDOFJOB. */
static agc_status_t next_sample(void)
{
    const agc_status_t st =
        agc_waitlist_call(SAMPLE_DT, AGC_TASK_LUNAR_GRAVITY_SAMPLE);

    if (st != AGC_STATUS_OK)
    {
        const agc_status_t bo = agc_bailout(AGC_ALARM_01203); /* WAITLIST */

        return ((bo == AGC_STATUS_OK) ? AGC_STATUS_ALARM : bo);
    }

    return (AGC_STATUS_OK);
}

/*
 * After LUNG's coarse and fine align: GCTR = -20; 1/PIPADT = PRIO31;
 * GCOMPZER; PHASCHNG; PIPSRINE (the first read is thrown away); GREED.
 */
agc_status_t agc_job_lunar_gravity_coarsed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram()->gctr = GRAVITY_SAMPLES;
    agc_imu_set_pipa_interval(PIPA_INTERVAL);
    agc_status_t st = agc_imu_gcompzer();

    if (st == AGC_STATUS_OK)
    {
        st = p57_phase();
    }

    if (st == AGC_STATUS_OK)
    {
        st = read_pipas();
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (next_sample());
}

/* GRABGRAV task: PIPSRINE; FINDVAC PRIO13 ADDGRAV; TASKOVER. */
agc_status_t agc_task_lunar_gravity_sample(void)
{
    agc_status_t st = read_pipas();

    if (st == AGC_STATUS_OK)
    {
        st = agc_exec_findvac(ALIGN_PRIO13, AGC_JOB_LUNAR_GRAVITY_ACCUMULATE);
    }

    if (!AGC_ASSERT(agc_align_ram()->gctr < 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/*
 * ADDGRAV: 1/PIPA; GCTR + 1; GACC += DELV.  While GCTR < 0 another
 * sample (GREED); after the last, STAR = UNIT(GACC) (SM), GOUT = STAR in
 * nav-base axes (CDUTRIG, *SMNB*); PHASCHNG; GOTO QMIN.
 */
agc_status_t agc_job_lunar_gravity_accumulate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram  = agc_align_ram();
    agc_vec_t        delv = agc_erase_delv();
    agc_status_t     st   = agc_imu_one_over_pipa(&delv); /* 1/PIPA   */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_delv(delv);
    agc_interp_t ip;

    agc_interp_init(&ip);
    ram->gctr = (agc_sp_t) (ram->gctr + 1); /* INCR     */
    ram->gacc = agc_vec_add(&ip, delv, ram->gacc);

    if (ram->gctr < 0)
    {
        return (next_sample()); /* GREED    */
    }

    ram->star = agc_vec_unit(&ip, ram->gacc, NULL, NULL);
    agc_vec_t g = ram->star;

    agc_guid_cdutrig(&lung_trig);
    st = AGC_STATUS_OK;
    agc_guid_smnb(&lung_trig, &g); /* *SMNB*   */
    ram->gout = g;
    st        = p57_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_goto(ram->lunar_gravity_return)); /* QMINEXIT */
}

/*
 * GVDETER (p. 963): THETAD = (42, -42, 35) degrees; CLEAR REFSMFLG;
 * LUNG, continuing at GRAVEL.
 */
agc_status_t agc_lm_lunar_surface_gravity(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_thetad(0, THETA_42DEG);
    agc_erase_set_thetad(1, THETA_MINUS_42);
    agc_erase_set_thetad(2, THETA_35DEG);
    agc_flag_clear(AGC_FLAG_REFSMFLG);

    return (lung(AGC_JOB_LUNAR_GRAVITY_ROTATE));
}

/*
 * GRAVEL (pp. 963-964): the stable member turned 180 degrees about the
 * gravity vector u (STAR, SM):  XSM = 2 (u u^T) - I/2 (half-unit rows),
 * XNB = NBSM(UNITX, UNITY, UNITZ) with LUNG's sines and cosines;
 * CALCGA -> THETAD.  STARAD +12D = GOUT/2; LUNG again.
 */
agc_status_t agc_job_lunar_gravity_rotate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    const agc_mat_t unit = agc_mat_identity_half();
    agc_status_t    st   = AGC_STATUS_OK;

    for (int i = 0; (i < 3) && (st == AGC_STATUS_OK); i++)
    {
        agc_vec_t nb = unit.row[i];

        agc_guid_nbsm(&lung_trig, &nb); /* *NBSM*   */
        st              = AGC_STATUS_OK;
        ram->xnb.row[i] = nb;
        const agc_vec_t uu =
            agc_vec_shl(&ip, agc_vec_scale(&ip, ram->star, ram->star.c[i]), 2);

        ram->xsm.row[i] = agc_vec_sub(&ip, uu, unit.row[i]); /* VSU*     */
    }

    uint16_t thetad[3];

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_calcga(&ip, &ram->xnb, &ram->xsm, thetad);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ram->starad[2] = agc_vec_shr(ram->gout, 1); /* VSR1     */

    return (lung(AGC_JOB_LUNAR_GRAVITY_MEASURED));
}

/*
 * After the second LUNG: STARSAV1 = UNIT(GOUT/2 + STARAD +12D) (nav
 * base); DSPTEM1 = angle to the saved GSAV; CLEAR FREEFLAG; flash V06N04:
 * PROCEED keeps it, V32 recycles (SET FREEFLAG).
 */
agc_status_t agc_job_lunar_gravity_measured(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();
    agc_interp_t     ip;

    agc_interp_init(&ip);
    ram->starsav1 = agc_vec_unit(
        &ip, agc_vec_add(&ip, agc_vec_shr(ram->gout, 1), ram->starad[2]), NULL,
        NULL);
    const agc_dp_t c =
        agc_dp_shl(&ip, agc_vec_dot(&ip, ram->starsav1, ram->gsav), 1);
    const agc_status_t st = align_set_dsptem1(agc_dp_acos(&ip, c));

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_clear(AGC_FLAG_FREEFLAG);

    return (agc_job_lunar_gravity_display(ctx));
}

/* The V06N04 flash (gravity error angle). */
agc_status_t agc_job_lunar_gravity_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_GRAVITY, AGC_JOB_GOTOPOOH,
                        AGC_JOB_LUNAR_GRAVITY_ACCEPTED,
                        AGC_JOB_LUNAR_GRAVITY_RECYCLE));
}

/* V32 on V06N04: UPFLAG FREEFLAG (measure again), then PROGRAV. */
agc_status_t agc_job_lunar_gravity_recycle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_FREEFLAG);

    return (agc_job_lunar_gravity_accepted(ctx));
}

/* PROGRAV: PHASCHNG; GSAV = STARSAV1; FREEFLAG -> GVDETER, else ATTCHK. */
agc_status_t agc_job_lunar_gravity_accepted(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = p57_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_align_ram()->gsav = agc_align_ram()->starsav1;

    if (agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return (agc_lm_lunar_surface_gravity(ctx));
    }

    return (agc_job_lunar_surface_attitude_check(ctx));
}

/*
 * JUSTTRIM (p. 966): IMUFINE and IMUSTALL, IMUPULSE on OGC and IMUSTALL,
 * then GOTO QMIN.
 */
agc_status_t agc_lm_lunar_surface_just_trim(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_align_ram()->lunar_gravity_return != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_imu_fine(); /* IMUFINE  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_LUNAR_SURFACE_TRIM_PULSE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    return (align_goto(AGC_JOB_LUNAR_SURFACE_TRIM_PULSE));
}

/* JUSTTRIM after IMUFINE: IMUPULSE (GYRCDR = OGC); re-entered when the
 * gyros are busy.                                                        */
agc_status_t agc_job_lunar_surface_trim_pulse(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (align_stall_outcome(AGC_IMU_DEV_IMU) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK); /* CURTAINS */
    }

    agc_status_t st =
        agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_LUNAR_SURFACE_TRIM_RETRY);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_LUNAR_SURFACE_TRIMMED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    return (align_goto(AGC_JOB_LUNAR_SURFACE_TRIMMED));
}

/* IMUPULSE again once the gyros are free. */
agc_status_t agc_job_lunar_surface_trim_retry(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st =
        agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_LUNAR_SURFACE_TRIM_RETRY);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false;

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_LUNAR_SURFACE_TRIMMED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st);
    }

    return (align_goto(AGC_JOB_LUNAR_SURFACE_TRIMMED));
}

/* After the torquing: QMINEXIT. */
agc_status_t agc_job_lunar_surface_trimmed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (align_stall_outcome(AGC_IMU_DEV_IMU) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK);
    }

    return (align_goto(agc_align_ram()->lunar_gravity_return));
}

/*
 * GYROTRIM (p. 966): STQ QMIN; with CDUSPOT = THETAD the nav-base X and Y
 * axes in SM (TRG*NBSM, *NBSM*) go to STARAD, STARAD +6; the present
 * XNB, YNB (CDUTRIG, CALCSMSC) to 6D, 12D; AXISGEN -> XDC; CALCGTA ->
 * OGC; JUSTTRIM.
 */
static agc_status_t gyro_trim(agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    agc_align_ram_t *ram = agc_align_ram();

    ram->lunar_gravity_return = ret;
    const agc_mat_t unit   = agc_mat_identity_half();
    const uint16_t  yzx[3] = {agc_erase_thetad(1), agc_erase_thetad(2),
                              agc_erase_thetad(0)};
    agc_cdutrig_t   t;
    agc_vec_t       x  = unit.row[0];
    agc_vec_t       y  = unit.row[1];
    agc_status_t    st = agc_guid_trg_nbsm(&t, yzx, &x);

    if (st == AGC_STATUS_OK)
    {
        agc_guid_nbsm(&t, &y);
        st = AGC_STATUS_OK;
    }

    agc_interp_t ip;

    agc_interp_init(&ip);

    if (st == AGC_STATUS_OK)
    {
        st = align_nb_present(&ip, &t, &ram->xnb);
    }

    const agc_vec_t a[2] = {x, y};
    const agc_vec_t b[2] = {ram->xnb.row[0], ram->xnb.row[1]};

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_axisgen(&ip, a, b, &ram->xdc);
    }

    if (st == AGC_STATUS_OK)
    {
        agc_imu_calcgta(&ip, &ram->xdc, ram->ogc);
        st = AGC_STATUS_OK;
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_lunar_surface_just_trim(ctx));
}

/*
 * COATRIM (p. 976): XSM = XDC; CDUTRIG, CALCSMSC, CALCGA -> THETAD.  The
 * initial alignment shows the final gimbal angles (V06N22) first; then
 * CORSIT.
 */
agc_status_t agc_lm_lunar_surface_coarse_trim(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();

    ram->xsm = ram->xdc; /* MATMOVE  */
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_cdutrig_t t;
    uint16_t      thetad[3];
    agc_status_t  st = align_nb_present(&ip, &t, &ram->xnb);

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_calcga(&ip, &ram->xnb, &ram->xsm, thetad);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!agc_flag_test(AGC_FLAG_INITALGN))
    {
        return (agc_job_lunar_surface_coarse_trim_go(ctx));
    }

    return (agc_job_lunar_surface_coarse_trim_display(ctx));
}

/* COATRIM: flash V06N22; ENTER shows it again. */
agc_status_t agc_job_lunar_surface_coarse_trim_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_INITALGN)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_GIMBALS, AGC_JOB_GOTOPOOH,
                        AGC_JOB_LUNAR_SURFACE_COARSE_TRIM_GO,
                        AGC_JOB_LUNAR_SURFACE_COARSE_TRIM_DISPLAY));
}

/* CORSIT (after PHASCHNG when displayed): COARSE. */
agc_status_t agc_job_lunar_surface_coarse_trim_go(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = p57_phase();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (align_coarse(AGC_JOB_LUNAR_SURFACE_COARSE_TRIMMED));
}

/* CORSIT after COARSE: NCOARSE; GYROTRIM; GOTO SURFDISP. */
agc_status_t agc_job_lunar_surface_coarse_trimmed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_status_t st = align_ncoarse();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (gyro_trim(ctx, AGC_JOB_LUNAR_SURFACE_ALIGNED));
}
