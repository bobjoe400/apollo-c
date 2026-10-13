/**********************************************************************
 * FILE NAME: lunar_surface_gravity.c
 *
 * PURPOSE:
 *   The IMU work of P57 lunar surface align: the gravity determination
 *   GVDETER (with LUNG / GRABGRAV / ADDGRAV and GRAVEL, the 180 degree
 *   turn of the stable member about the gravity vector that cancels the
 *   PIPA biases), the gyro torquing GYROTRIM / JUSTTRIM and the coarse
 *   align for large corrections COATRIM / CORSIT.
 *
 *   LUNG and GYROTRIM are interpretive subroutines (STQ QMIN); their
 *   caller names the job to continue as in
 *   agc_align_ram()->lunar_gravity_return.
 *
 * AGC SOURCE:
 *   Luminary099 P51-P53.agc pp. 963-967 (GVDETER .. JUSTTRIM), 976-977 (COATRIM, CORSIT)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   THETAD        uint16_t  I/O  CDU words      GVDETER angles, CALCGA
 *   DELV          agc_vec_t I    PIPA 2^14      agc_erase_delv()
 *   GACC, GCTR, STAR, GOUT, GSAV, STARSAV1, XSM, XNB, XDC, OGC: agc_align_ram()
 *   REFSMFLG, FREEFLAG, INITALGN  flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_ram         alignment routines of this lane (alignment.h)
 *   agc_exec_findvac      executive: jobs, VAC areas, sleep / wake (exec.h)
 *   agc_waitlist_call     WAITLIST (exec.h)
 *   agc_imu_*             IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_guid_*            CDUTRIG, *NBSM* / *SMNB*, PIPASR (guidance.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_mat_identity_half  matrix arithmetic of the interpreter (interp.h)
 *   agc_dp_acos           DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_bailout           BAILOUT (exec.h)
 *   agc_job_lunar_surface_attitude_check  see its header
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarms: AGC_ALARM_01203.
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/guidance/guidance.h"
#include "agc/programs/alignment.h"

#define PH_SURFACE_ALIGN ((uint16_t) 04024)  /* OCT 04024                     */
#define THETA_42DEG      ((uint16_t) 07357)  /* 42DEG OCT 07357               */
#define THETA_MINUS_42   ((uint16_t) 070420) /* COM of 42DEG                  */
#define THETA_35DEG      ((uint16_t) 06211)  /* 35DEG OCT 06211               */
#define GRAVITY_SAMPLES  ((agc_sp_t) -20)    /* T/2SEC DEC -20                */
#define PIPA_INTERVAL    ((agc_sp_t) 031000) /* CA PRIO31, TS 1/PIPADT        */
#define SAMPLE_DT        ((uint16_t) 200)    /* 2SECS                         */
#define VN_GRAVITY       ALIGN_VN(6, 4)      /* DISGRVER VN 0604              */
#define VN_GIMBALS       ALIGN_VN(6, 22)     /* V06N22                        */

/* The CDUTRIG of the last gravity sample (*NBSM* in GRAVEL uses it).   */
static agc_cdutrig_t lung_trig; /* CDUTRIG of the last gravity sample (*NBSM* in GRAVEL) */

/**********************************************************************
 * LUNG (p. 965): STQ QMIN; GACC = 0; PHASCHNG; IMUCOARS / IMUFINE to THETAD
 * (COARSE), then the sampling set-up.
 *
 * FUNCTION NAME: lung
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   ret       agc_job_id_t  I    job the routine continues as when it ends
 *                                (the AGC return address)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t lung(agc_job_id_t ret)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->lunar_gravity_return = ret; /* QMIN     */
    ram->gacc                 = agc_vec_zero();
    align_phase4(PH_SURFACE_ALIGN);

    return (align_coarse(AGC_JOB_LUNAR_GRAVITY_COARSED));
}

/**********************************************************************
 * AGC: PIPSRINE (Luminary099 P51-P53.agc p. 962)
 *
 * PIPSRINE (PIPASR +3): read and zero the PIPAs into DELV.
 *
 * FUNCTION NAME: read_pipas
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void read_pipas(void)
{
    agc_guid_pipasr(&agc_guid_state()->avg, false);
}

/**********************************************************************
 * AGC: GREED (Luminary099 P51-P53.agc p. 966)
 *
 * GREED: TWIDDLE 2SECS GRABGRAV; ENDOFJOB.
 *
 * FUNCTION NAME: next_sample
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_ALARM after an alarm
 **********************************************************************/
static agc_status_t next_sample(void)
{
    const agc_status_t st = agc_waitlist_call(SAMPLE_DT, AGC_TASK_LUNAR_GRAVITY_SAMPLE); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        const agc_status_t bo = agc_bailout(AGC_ALARM_01203); /* WAITLIST */

        return ((bo == AGC_STATUS_OK) ? AGC_STATUS_ALARM : bo); /* BAILOUT done: report the alarm */
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: LUNG (Luminary099 P51-P53.agc p. 965)
 *
 * After LUNG's coarse and fine align: GCTR = -20; 1/PIPADT = PRIO31;
 * GCOMPZER; PHASCHNG; PIPSRINE (the first read is thrown away); GREED.
 *
 * FUNCTION NAME: agc_job_lunar_gravity_coarsed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_gravity_coarsed(void)
{
    agc_align_ram()->gctr = GRAVITY_SAMPLES;

    const agc_status_t st = agc_imu_set_pipa_interval(PIPA_INTERVAL); /* 1/PIPADT store status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_imu_gcompzer();
    align_phase4(PH_SURFACE_ALIGN);
    read_pipas();

    return (next_sample());
}

/**********************************************************************
 * AGC: GRABGRAV (Luminary099 P51-P53.agc p. 966)
 *
 * GRABGRAV task: PIPSRINE; FINDVAC PRIO13 ADDGRAV; TASKOVER.
 *
 * FUNCTION NAME: agc_task_lunar_gravity_sample
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_task_lunar_gravity_sample(void)
{
    read_pipas();

    return (agc_exec_findvac(ALIGN_PRIO13, AGC_JOB_LUNAR_GRAVITY_ACCUMULATE));
}

/**********************************************************************
 * AGC: ADDGRAV (Luminary099 P51-P53.agc p. 966)
 *
 * ADDGRAV: 1/PIPA; GCTR + 1; GACC += DELV.  While GCTR < 0 another sample
 * (GREED); after the last, STAR = UNIT(GACC) (SM), GOUT = STAR in nav-base
 * axes (CDUTRIG, *SMNB*); PHASCHNG; GOTO QMIN.
 *
 * FUNCTION NAME: agc_job_lunar_gravity_accumulate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_gravity_accumulate(void)
{
    agc_align_ram_t *ram  = agc_align_ram();              /* alignment erasables */
    agc_vec_t        delv = agc_erase_delv();             /* DELV after 1/PIPA, PIPA counts scale 2^14 */
    agc_status_t     st   = agc_imu_one_over_pipa(&delv); /* 1/PIPA   */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_delv(delv);
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    ram->gctr = (agc_sp_t) (ram->gctr + 1); /* INCR     */
    ram->gacc = agc_vec_add(&ip, delv, ram->gacc);

    if (ram->gctr < 0)
    {
        return (next_sample()); /* GREED    */
    }

    ram->star   = agc_vec_unit(&ip, ram->gacc, NULL, NULL);
    agc_vec_t g = ram->star; /* STAR in nav-base axes (*SMNB*), half-unit (2^1) */

    agc_imu_cdutrig(&lung_trig);
    agc_imu_smnb(&lung_trig, &g); /* *SMNB*   */
    ram->gout = g;
    align_phase4(PH_SURFACE_ALIGN);

    return (align_goto(ram->lunar_gravity_return)); /* QMINEXIT */
}

/**********************************************************************
 * GVDETER (p. 963): THETAD = (42, -42, 35) degrees; CLEAR REFSMFLG; LUNG,
 * continuing at GRAVEL.
 *
 * FUNCTION NAME: agc_lm_lunar_surface_gravity
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_lm_lunar_surface_gravity(void)
{
    agc_erase_set_thetad(0, THETA_42DEG);
    agc_erase_set_thetad(1, THETA_MINUS_42);
    agc_erase_set_thetad(2, THETA_35DEG);
    agc_flag_clear(AGC_FLAG_REFSMFLG);

    return (lung(AGC_JOB_LUNAR_GRAVITY_ROTATE));
}

/**********************************************************************
 * GRAVEL (pp. 963-964): the stable member turned 180 degrees about the
 * gravity vector u (STAR, SM):  XSM = 2 (u u^T) - I/2 (half-unit rows), XNB
 * = NBSM(UNITX, UNITY, UNITZ) with LUNG's sines and cosines; CALCGA ->
 * THETAD.  STARAD +12D = GOUT/2; LUNG again.
 *
 * FUNCTION NAME: agc_job_lunar_gravity_rotate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_gravity_rotate(void)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    const agc_mat_t unit = agc_mat_identity_half(); /* XUNIT, YUNIT, ZUNIT rows, half-unit (2^1) */

    for (int i = 0; i < 3; i++)
    {
        agc_vec_t nb = unit.row[i]; /* unit vector i in SM (*NBSM*), half-unit (2^1) */

        agc_imu_nbsm(&lung_trig, &nb);                                                           /* *NBSM*   */
        ram->xnb.row[i]    = nb;
        const agc_vec_t uu = agc_vec_shl(&ip, agc_vec_scale(&ip, ram->star, ram->star.c[i]), 2); /* STAR STAR(i) VSL2, scale 2^0 */

        ram->xsm.row[i] = agc_vec_sub(&ip, uu, unit.row[i]); /* VSU*     */
    }

    uint16_t           thetad[3];                                              /* THETAD X, Y, Z words */
    const agc_status_t st = agc_imu_calcga(&ip, &ram->xnb, &ram->xsm, thetad); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ram->starad[2] = agc_vec_shr(ram->gout, 1); /* VSR1     */

    return (lung(AGC_JOB_LUNAR_GRAVITY_MEASURED));
}

/**********************************************************************
 * AGC: LUNG (Luminary099 P51-P53.agc p. 965)
 *
 * After the second LUNG: STARSAV1 = UNIT(GOUT/2 + STARAD +12D) (nav base);
 * DSPTEM1 = angle to the saved GSAV; CLEAR FREEFLAG; flash V06N04: PROCEED
 * keeps it, V32 recycles (SET FREEFLAG).
 *
 * FUNCTION NAME: agc_job_lunar_gravity_measured
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
agc_status_t agc_job_lunar_gravity_measured(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */
    agc_interp_t     ip;                    /* interpreter state */

    agc_interp_init(&ip);
    ram->starsav1    = agc_vec_unit(&ip, agc_vec_add(&ip, agc_vec_shr(ram->gout, 1), ram->starad[2]), NULL, NULL);
    const agc_dp_t c = align_dot_shl(&ip, ram->starsav1, ram->gsav, 1); /* DOT SL1: cosine of the gravity error, scale 2^0 */
    align_set_dsptem1(agc_dp_acos(&ip, c));
    agc_flag_clear(AGC_FLAG_FREEFLAG);

    return (agc_job_lunar_gravity_display(ctx));
}

/**********************************************************************
 * AGC: GRAVEL V06N04 (Luminary099 P51-P53.agc p. 964)
 *
 * The V06N04 flash (gravity error angle).
 *
 * FUNCTION NAME: agc_job_lunar_gravity_display
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
agc_status_t agc_job_lunar_gravity_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_GRAVITY, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_GRAVITY_ACCEPTED, AGC_JOB_LUNAR_GRAVITY_RECYCLE));
}

/**********************************************************************
 * AGC: UPFLAG (Luminary099 SERVICE_ROUTINES.agc p. 1375)
 *
 * V32 on V06N04: UPFLAG FREEFLAG (measure again), then PROGRAV.
 *
 * FUNCTION NAME: agc_job_lunar_gravity_recycle
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
agc_status_t agc_job_lunar_gravity_recycle(agc_job_ctx_t *ctx)
{
    agc_flag_set(AGC_FLAG_FREEFLAG);

    return (agc_job_lunar_gravity_accepted(ctx));
}

/**********************************************************************
 * AGC: PROGRAV (Luminary099 P51-P53.agc p. 965)
 *
 * PROGRAV: PHASCHNG; GSAV = STARSAV1; FREEFLAG -> GVDETER, else ATTCHK.
 *
 * FUNCTION NAME: agc_job_lunar_gravity_accepted
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
agc_status_t agc_job_lunar_gravity_accepted(agc_job_ctx_t *ctx)
{
    align_phase4(PH_SURFACE_ALIGN);

    agc_align_ram()->gsav = agc_align_ram()->starsav1;

    if (agc_flag_test(AGC_FLAG_FREEFLAG))
    {
        return (agc_lm_lunar_surface_gravity());
    }

    return (agc_job_lunar_surface_attitude_check(ctx));
}

/**********************************************************************
 * JUSTTRIM (p. 966): IMUFINE and IMUSTALL, IMUPULSE on OGC and IMUSTALL,
 * then GOTO QMIN.
 *
 * FUNCTION NAME: agc_lm_lunar_surface_just_trim
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_lm_lunar_surface_just_trim(void)
{
    agc_status_t st = agc_imu_fine(); /* IMUFINE  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_LUNAR_SURFACE_TRIM_PULSE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (align_goto(AGC_JOB_LUNAR_SURFACE_TRIM_PULSE));
}

/**********************************************************************
 * AGC: JUSTTRIM (Luminary099 P51-P53.agc p. 968)
 *
 * JUSTTRIM after IMUFINE: IMUPULSE (GYRCDR = OGC); re-entered when the
 * gyros are busy.
 *
 * FUNCTION NAME: agc_job_lunar_surface_trim_pulse
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_surface_trim_pulse(void)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_IMU); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    agc_status_t st = agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_LUNAR_SURFACE_TRIM_RETRY); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_LUNAR_SURFACE_TRIMMED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (align_goto(AGC_JOB_LUNAR_SURFACE_TRIMMED));
}

/**********************************************************************
 * AGC: IMUPULSE (Luminary099 IMU_MODE_SWITCHING_ROUTINES.agc p. 1319)
 *
 * IMUPULSE again once the gyros are free.
 *
 * FUNCTION NAME: agc_job_lunar_surface_trim_retry
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_surface_trim_retry(void)
{
    agc_status_t st = agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_LUNAR_SURFACE_TRIM_RETRY); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_LUNAR_SURFACE_TRIMMED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (align_goto(AGC_JOB_LUNAR_SURFACE_TRIMMED));
}

/**********************************************************************
 * AGC: QMINEXIT (Luminary099 P51-P53.agc p. 966)
 *
 * After the torquing: QMINEXIT.
 *
 * FUNCTION NAME: agc_job_lunar_surface_trimmed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_surface_trimmed(void)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_IMU); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    return (align_goto(agc_align_ram()->lunar_gravity_return));
}

/**********************************************************************
 * COATRIM (p. 976): XSM = XDC; CDUTRIG, CALCSMSC, CALCGA -> THETAD.  The
 * initial alignment shows the final gimbal angles (V06N22) first; then
 * CORSIT.
 *
 * FUNCTION NAME: agc_lm_lunar_surface_coarse_trim
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
agc_status_t agc_lm_lunar_surface_coarse_trim(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->xsm = ram->xdc; /* MATMOVE  */
    agc_interp_t ip;     /* interpreter state */

    agc_interp_init(&ip);
    agc_cdutrig_t t;                                         /* CDUTRIG sines and cosines */
    uint16_t      thetad[3];                                 /* THETAD X, Y, Z words */
    agc_status_t  st = align_nb_present(&ip, &t, &ram->xnb); /* status of the call */

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
        return (agc_job_lunar_surface_coarse_trim_go());
    }

    return (agc_job_lunar_surface_coarse_trim_display(ctx));
}

/**********************************************************************
 * AGC: COATRIM (Luminary099 P51-P53.agc p. 974)
 *
 * COATRIM: flash V06N22; ENTER shows it again.
 *
 * FUNCTION NAME: agc_job_lunar_surface_coarse_trim_display
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
agc_status_t agc_job_lunar_surface_coarse_trim_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_GIMBALS, AGC_JOB_GOTOPOOH, AGC_JOB_LUNAR_SURFACE_COARSE_TRIM_GO, AGC_JOB_LUNAR_SURFACE_COARSE_TRIM_DISPLAY));
}

/**********************************************************************
 * AGC: CORSIT (Luminary099 P51-P53.agc p. 974)
 *
 * CORSIT (after PHASCHNG when displayed): COARSE.
 *
 * FUNCTION NAME: agc_job_lunar_surface_coarse_trim_go
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_surface_coarse_trim_go(void)
{
    align_phase4(PH_SURFACE_ALIGN);

    return (align_coarse(AGC_JOB_LUNAR_SURFACE_COARSE_TRIMMED));
}

/**********************************************************************
 * AGC: CORSIT (Luminary099 P51-P53.agc p. 974)
 *
 * CORSIT after COARSE: NCOARSE; GYROTRIM; GOTO SURFDISP.
 *
 * FUNCTION NAME: agc_job_lunar_surface_coarse_trimmed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_lunar_surface_coarse_trimmed(void)
{
    agc_status_t st = align_ncoarse(); /* NCOARSE, then the step statuses */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /*
     * GYROTRIM (p. 966): STQ QMIN; with CDUSPOT = THETAD the nav-base X and Y
     * axes in SM (TRG*NBSM, *NBSM*) go to STARAD, STARAD +6; the present
     * XNB, YNB (CDUTRIG, CALCSMSC) to 6D, 12D; AXISGEN -> XDC; CALCGTA ->
     * OGC; JUSTTRIM.
     */
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->lunar_gravity_return = AGC_JOB_LUNAR_SURFACE_ALIGNED;
    const agc_mat_t unit      = agc_mat_identity_half(); /* XUNIT, YUNIT rows, half-unit (2^1) */
    const uint16_t  yzx[3]    = {agc_erase_thetad(1), agc_erase_thetad(2), agc_erase_thetad(0)};
    agc_cdutrig_t   t;                                   /* CDUTRIG sines and cosines */
    agc_vec_t       x         = unit.row[0];             /* XUNIT, then TRG*NBSM: nav-base X in SM (STARAD), half-unit (2^1) */
    agc_vec_t       y         = unit.row[1];             /* YUNIT, then *NBSM*: nav-base Y in SM (STARAD +6), half-unit (2^1) */

    agc_imu_trg_nbsm(&t, yzx, &x);
    agc_imu_nbsm(&t, &y);
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    st = align_nb_present(&ip, &t, &ram->xnb); /* status of the call */

    const agc_vec_t a[2] = {x, y};
    const agc_vec_t b[2] = {ram->xnb.row[0], ram->xnb.row[1]};

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_axisgen(&ip, a, b, &ram->xdc);
    }

    if (st == AGC_STATUS_OK)
    {
        agc_imu_calcgta(&ip, &ram->xdc, agc_imu_ogc());
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_lm_lunar_surface_just_trim());
} /* agc_job_lunar_surface_coarse_trimmed */
