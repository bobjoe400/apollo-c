/**********************************************************************
 * FILE NAME: job_entries.c
 *
 * PURPOSE:
 *   P07 SYSTEMS TEST in the LM (V92): the job and task entries the
 *   registry dispatches. Each fetches the test erasables, runs the step
 *   of the IMU performance tests it names (imu_performance_test.c,
 *   drift_filter.c) and writes them back. The tests: site latitude /
 *   azimuth, coarse and fine align, the gyro drift filter, the PIPA scale
 *   test, the vertical drift test.
 *
 * AGC SOURCE:
 *   Luminary099 IMU_PERFORMANCE_TEST_2.agc p.373 (REDO), the restart and
 *   wake points of IMU_PERFORMANCE_TEST_2.agc pp.373-381 and
 *   IMU_PERFORMANCE_TESTS_4.agc pp.382-389; SYSTEM_TEST_STANDARD_LEAD_INS
 *   pp.370-372 (see NOTES). Comanche055's REDO falls into IMUTEST:
 *   src/csm/prelaunch/align.c
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors or by ECADR)
 *   Name          Type          I/O  Scale     Description
 *   ------------  ------------  ---  --------  ----------------------------
 *   W .. AINLA+99 uint16_t[]    I/O  words     the test erasables (st_words())
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   st_fetch / st_flush   the test erasables (test_erasables.c)
 *   the step functions    imu_performance_test.c, drift_filter.c
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   VARALARM 01601 (bad IMU moding, SOMERR2); CCSHOLE when the first PIPA
 *   count is +0; the test ends with ENDTEST1 (V34) or SOMERR2.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only; the jobs and tasks of P07 (V92).
 *
 * NOTES:
 *   The LM IMU performance test.  REDO sets MM 07, zeroes the ISS
 *   (GEOIMUTT) and asks for the site latitude and the azimuth of the
 *   test table (V06N41).  The platform is then coarse aligned to the
 *   navigation base (POSGMBL); in gimbal lock only a PIPA test is made
 *   (PIPACHK..PIPJOBB: count the pulses of one PIPA for 58 x 5.12 s,
 *   V06N98 shows the rate). Otherwise it is fine aligned and, after
 *   PERFDLAY, the drift filter runs once a second (ESTIMS, ALLOOP,
 *   ALFLT) until LENGTHOT passes are done; the drift angles then torque
 *   the gyros (SETUPER1), the south drift is shown (TORQUE), the PIPA
 *   test follows, then the vertical drift test (VERTDRFT, ESTIMS again
 *   with TORQNDX = POSMAX) ends with its result (VALMIS) and ENDTEST1.
 *
 *   What the rope lacks: the drift filter's calculation proper.
 *   PERFERAS (p.387) does TC ATIGINC -- "GOTO ERASABLE TO CALCULATE" --
 *   into a program at E7,1400 that "MUST BE LOADED BEFORE ANY ATTEMPT IS
 *   MADE TO RUN THE IMU PERFORMANCE TEST"; it is not in Luminary099.
 *   The filter's gains are erasable too: the transition matrix TRANSM1
 *   and the 144-word gain schedule ALFDK (E4,1400 on; ALKCG loads 12
 *   words per stage), and so are the test parameters PERFDLAY, 1SECXT,
 *   LENGTHOT, POSITON and PIPINDEX (the ground loads them with V21 N01).
 *   Everything in fixed memory is translated; the erasable words are
 *   read by address (src/core/ecadr.c), so a V21 N01 load reaches them.
 *   DEVIATION (ATIGINC): the erasable program is not run (nothing in C
 *   can execute erasable memory); PERFERAS goes on as if it had
 *   returned. DEVIATION (storage): the system test erasables (W .. AINLA
 *   +99D, E5,1400 on, 128 words) are kept as AGC words, fetched from the
 *   ECADR image when a job or task of the test starts and written back
 *   when it ends, because the code depends on their layout (DATAPL
 *   overlays ANGZ, ALKCG stores ALTIM and ALTIMS with ALDK, LOSVEC +1 is
 *   NDXCTR).  The AGC overlays the rendezvous W matrix with them; the C
 *   W matrix is separate storage.  The earth-rate commands ERCOMP live
 *   in the IMU lane's ERCOMP gyro buffer. DEVIATION (returns): QPLACE
 *   and QPLACES hold a step number (st_step_t) instead of a return
 *   address; a job resumes at the step. DEVIATION (CHECKG): the AGC
 *   catches a PIPA pulse between two reads a few microseconds apart; in
 *   C the PIPA can only change between two runs of the job, so the job
 *   yields (CHANG1) between the reads.
 *
 *   Also translated: ZEROING, OVERFFIX, ERTHRVSE / EARTHR (the
 *   same text as the CM prelaunch ones, src/csm/prelaunch/align.c).  The
 *   erasable-memory program helpers in SYSTEM_TEST_STANDARD_LEAD_INS
 *   (E/BKCALL, E/CALL, E/SWITCH, E/JOBWAK) exist only so that programs
 *   loaded into erasable memory can make bank calls; they vanish in C
 *   like the inter-bank routines (src/io/service/interbank.c).  SOMEERRR
 *   (alarm 1600) has no caller in fixed memory (the erasable program's). **********************************************************************/
#include <stddef.h>

#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "internal.h"

/**********************************************************************
 * AGC: none -- Forward: the step a QPLACE / QPLACES return goes on at.
 *
 * ARGUMENTS:
 *   step  st_step_t        I   the step from QPLACE or QPLACES
 *   ctx   agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the step's status, BAD_ARG for a word that is no step
 **********************************************************************/
agc_status_t run_step(st_step_t step, agc_job_ctx_t *ctx)
{
    switch (step)
    {
        case STEP_PIPACHK:
            return (pipachk());
        case STEP_PIPACHK_ARM:
            return (pipachk_arm());
        case STEP_PIPATASK_START:
            /* PIPACHK after CHECKG: TWIDDLE PIPATASK in 20 ms, ENDOFJOB.    */
            return (agc_waitlist_call(PIP_START_CS, AGC_TASK_SYSTEMS_TEST_PIP_TIMER));
        case STEP_PIPJOBB_CATCH:
            return (pipjobb_catch());
        case STEP_PIPJOBB_RESULT:
            return (pipjobb_result(ctx));
        case STEP_VERTDRFT:
            return (vertdrft(DEC3990));
        case STEP_GUESS1:
            return (guess1());
        case STEP_ENDTEST1:
            return (endtest1());
        case STEP_ENDOFJOB:
            return (AGC_STATUS_OK);
        case STEP_NONE:
        case STEP_COUNT:
        default:
            break;
    }

    return (AGC_STATUS_BAD_ARG); /* QPLACE / QPLACES overwritten */
}

/**********************************************************************
 * AGC: none -- The body of a job or task of the test, with the test erasables fetched.
 *
 * ARGUMENTS:
 *   e    st_entry_t       I   the job or task starting
 *   ctx  agc_job_ctx_t *  I   the running job (NULL for a task)
 *
 * RETURNS: the status of what runs; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
static agc_status_t st_body(st_entry_t e, agc_job_ctx_t *ctx)
{
    uint16_t *w    = st_words(); /* W .. AINLA +99D, the test erasables */
    bool      good = false;      /* stall over and good */

    switch (e)
    {
        case ENTRY_REDO:
            return (redo(ctx));
        case ENTRY_IMU_ZEROED:
            imu_stall_outcome(&good);

            return (good ? imuback(ctx) : AGC_STATUS_OK); /* stall over and good; else SOMERR2 */
        case ENTRY_SITE_DISPLAY:
            return (site_display(ctx));
        case ENTRY_SITE_LOADED:
            return (posgmbl());
        case ENTRY_END:
            return (endtest1());
        case ENTRY_COARSE_ALIGNED:
            imu_stall_outcome(&good);

            return (good ? coarse_aligned() : AGC_STATUS_OK); /* stall over and good; else SOMERR2 */
        case ENTRY_FINE_ALIGNED:
            imu_stall_outcome(&good);

            return (good ? fine_aligned() : AGC_STATUS_OK); /* stall over and good; else SOMERR2 */
        case ENTRY_DRIFT_START:
            return (estims());
        case ENTRY_RESUME:
            return (run_step((st_step_t) w_sp(W_QPLACE), ctx));
        case ENTRY_EARTH_RATE_RETRY:
            return (earthr_stall(agc_imu_pulse_buf(AGC_IMU_GBUF_ERCOMP, AGC_JOB_SYSTEMS_TEST_EARTH_RATE_RETRY)));
        case ENTRY_EARTH_RATE_DONE:
            imu_stall_outcome(&good);

            return (good ? run_step((st_step_t) w_sp(W_QPLACES), ctx) : AGC_STATUS_OK);       /* stall over and good; else SOMERR2 */
        case ENTRY_PIP_CATCH:
            return (pip_catch(ctx));
        case ENTRY_PIP_RESULT:
            return ((w[W_NDXCTR] == 0u) ? earthr_star(STEP_PIPJOBB_CATCH) : pipjobb_catch()); /* PIPJOBB */
        case ENTRY_FILTER:
            return (alflt(ctx));
        case ENTRY_TORQUE_RETRY:
            return (setuper1_stall(agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_SYSTEMS_TEST_TORQUE_RETRY), ctx));
        case ENTRY_TORQUED:
            imu_stall_outcome(&good);

            return (good ? geostrt4(ctx) : AGC_STATUS_OK); /* stall over and good; else SOMERR2 */
        case ENTRY_GIMBALS_ZEROED:
            imu_stall_outcome(&good);

            return (good ? show(STEP_VERTDRFT, ctx) : AGC_STATUS_OK);     /* stall over and good; else SOMERR2 */
        case ENTRY_SHOW:
            return (show1(ctx));
        case ENTRY_DRIFT_WAKE:
            return (agc_exec_job_wake(AGC_JOB_SYSTEMS_TEST_DRIFT_START)); /* GOESTIMS */
        case ENTRY_PIP_TIMER:
            return (pip_timer());
        case ENTRY_SAMPLE:
            return (alloop());
        default:
            break;
    } /* switch (e) */

    return (AGC_STATUS_BAD_ARG);
} /* st_body */

/**********************************************************************
 * AGC: none -- Run one job or task of the test on the test erasables (fetch, body,
 * write back; the body's failure is reported first).
 *
 * ARGUMENTS:
 *   e    st_entry_t       I   the job or task starting
 *   ctx  agc_job_ctx_t *  I   the running job (NULL for a task)
 *
 * RETURNS: the body's status, else the write-back status
 **********************************************************************/
static agc_status_t st_enter(st_entry_t e, agc_job_ctx_t *ctx)
{
    st_fetch();

    agc_status_t st = st_body(e, ctx); /* status of the test body */
    agc_status_t fl = st_flush();      /* write-back status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (fl);
}

/**********************************************************************
 * REDO (V92, IMU_PERFORMANCE_TEST_2.agc p.373): NEWMODEX 07, then the
 * IMU performance tests.  The CSM's REDO (falls into IMUTEST) is
 * agc_job_systems_test_csm (src/csm/prelaunch/align.c).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the V92 job
 *
 * RETURNS: the status of REDO
 **********************************************************************/
agc_status_t agc_job_systems_test(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_REDO, ctx));
}

/**********************************************************************
 * IMUBACK after IMUZERR's IMUSTALL (p.373).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the status of IMUBACK
 **********************************************************************/
agc_status_t agc_job_systems_test_imu_zeroed(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_IMU_ZEROED, ctx));
}

/**********************************************************************
 * LATAZCHK again after ENTER on V06N41 (p.373, TC -5).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display status
 **********************************************************************/
agc_status_t agc_job_systems_test_site_display(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_SITE_DISPLAY, ctx));
}

/**********************************************************************
 * LATAZCHK V33: POSGMBL (pp.373-374).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the status of POSGMBL
 **********************************************************************/
agc_status_t agc_job_systems_test_site_loaded(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_SITE_LOADED, ctx));
}

/**********************************************************************
 * ENDTEST1 (p.377): V34 on V06N41 or V06N98.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the ENDTEST1 status
 **********************************************************************/
agc_status_t agc_job_systems_test_end(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_END, ctx));
}

/**********************************************************************
 * POSGMBL after IMUSLLLG (p.374).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the status of PIPACHK or FINIMUDD
 **********************************************************************/
agc_status_t agc_job_systems_test_coarse_aligned(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_COARSE_ALIGNED, ctx));
}

/**********************************************************************
 * FINIMUDD after IMUSTALL (p.374).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the LONGCALL status
 **********************************************************************/
agc_status_t agc_job_systems_test_fine_aligned(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_FINE_ALIGNED, ctx));
}

/**********************************************************************
 * ESTICADR (p.374): the job GOESTIMS wakes runs ESTIMS (p.383).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the ESTIMS status
 **********************************************************************/
agc_status_t agc_job_systems_test_drift_start(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_DRIFT_START, ctx));
}

/**********************************************************************
 * TC QPLACE (pp.378, 381): V33 on V06N98, or a PIPA pulse caught.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the status of the step in QPLACE
 **********************************************************************/
agc_status_t agc_job_systems_test_resume(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_RESUME, ctx));
}

/**********************************************************************
 * EARTHR's PULSEIMU again after GYROBUSY (p.380).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the PULSEIMU / IMUSLLLG status
 **********************************************************************/
agc_status_t agc_job_systems_test_earth_rate_retry(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_EARTH_RATE_RETRY, ctx));
}

/**********************************************************************
 * EARTHR* after IMUSLLLG: TC QPLACES (p.380).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the status of the step in QPLACES
 **********************************************************************/
agc_status_t agc_job_systems_test_earth_rate_done(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_EARTH_RATE_DONE, ctx));
}

/**********************************************************************
 * CHECKG1 (p.378): wait for a PIPA pulse.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the CHECKG1 status
 **********************************************************************/
agc_status_t agc_job_systems_test_pip_catch(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_PIP_CATCH, ctx));
}

/**********************************************************************
 * PIPJOBB (p.375): the end of the PIPA count.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the EARTHR* or CHECKG status
 **********************************************************************/
agc_status_t agc_job_systems_test_pip_result(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_PIP_RESULT, ctx));
}

/**********************************************************************
 * ALFLT (p.385): one pass of the drift filter.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the ALFLT status
 **********************************************************************/
agc_status_t agc_job_systems_test_filter(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_FILTER, ctx));
}

/**********************************************************************
 * SETUPER1's IMUPULSE again after GYROBUSY (p.388).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the IMUPULSE / IMUSLLLG status
 **********************************************************************/
agc_status_t agc_job_systems_test_torque_retry(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_TORQUE_RETRY, ctx));
}

/**********************************************************************
 * GEOSTRT4 after SETUPER1's IMUSLLLG (p.388).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the GEOSTRT4 status
 **********************************************************************/
agc_status_t agc_job_systems_test_torqued(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_TORQUED, ctx));
}

/**********************************************************************
 * COAALIGN after IMUSTALL (p.378): SHOW, then VERTDRFT.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the woken job
 *
 * RETURNS: the SHOW status
 **********************************************************************/
agc_status_t agc_job_systems_test_gimbals_zeroed(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_GIMBALS_ZEROED, ctx));
}

/**********************************************************************
 * SHOW1 again after ENTER on V06N98 (p.381).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display status
 **********************************************************************/
agc_status_t agc_job_systems_test_show(agc_job_ctx_t *ctx)
{
    return (st_enter(ENTRY_SHOW, ctx));
}

/**********************************************************************
 * GOESTIMS (p.374): JOBWAKE ESTICADR, TASKOVER.
 *
 * RETURNS: the JOBWAKE status
 **********************************************************************/
agc_status_t agc_task_systems_test_drift_wake(void)
{
    return (st_enter(ENTRY_DRIFT_WAKE, NULL));
}

/**********************************************************************
 * PIPATASK (p.375): every 5.12 s until LENGTHOT runs out, then PIPJOBB.
 *
 * RETURNS: the TWIDDLE or FINDVAC status
 **********************************************************************/
agc_status_t agc_task_systems_test_pip_timer(void)
{
    return (st_enter(ENTRY_PIP_TIMER, NULL));
}

/**********************************************************************
 * ALLOOP (p.384): the drift test's once-per-sample task.
 *
 * RETURNS: the ALLOOP status
 **********************************************************************/
agc_status_t agc_task_systems_test_sample(void)
{
    return (st_enter(ENTRY_SAMPLE, NULL));
}
