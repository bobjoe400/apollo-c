/*
 * leadin.c -- RADAR_LEADIN_ROUTINES.agc, LM only (Luminary099
 * pp.490-491): the radar sampling loop of the R04 radar test (verb 63)
 * and R77 (verb 78).
 *
 * RADSAMP is a waitlist task that recalls itself every RSAMPDT - 1
 * centiseconds ("times normal once-per-second sampling"; +0 inserted
 * manually terminates the test) and starts the DORSAMP job, which takes
 * one sample of the radar selected by RTSTDEX (VARADAR) and keeps it in
 * RSTACK for the R04 nouns, cycling RTSTLOC.
 *
 * RSAMPDT, RTSTDEX, RTSTBASE, RTSTMAX, RTSTLOC and RFAILCNT belong to the
 * extended verbs (sys-misc, src/io/verbs/extended_verbs_lm_radar.c).
 * DEVIATION: RSTACK EQUALS RANGE in the AGC (eight words over RANGE,
 * RRATE, RTHETA); here it has its own storage.
 */
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "radar.h"

#define LEADIN_PRIO25   ((agc_prio_t) 025)
#define LEADIN_RSTACK_N 4 /* RSTACK: B(8), four DP readings          */

static agc_dp_t rstack[LEADIN_RSTACK_N];

/* RSTACK +2 index (index 0..3: DP readings for nouns 66, 67, 78).        */
agc_dp_t agc_erase_rstack(int index)
{
    if (!AGC_ASSERT(index >= 0 && index < LEADIN_RSTACK_N))
    {
        return (0);
    }

    return (rstack[index]);
}

/* RADSAMP (p.490): recall in RSAMPDT - 1 cs, start DORSAMP, and for
 * cyclic sampling RTSTDEX = RTSTLOC / 2 + RTSTBASE (0 for RR, 2 for LR). */
agc_status_t agc_task_radsamp(void)
{
    agc_sp_t dt = agc_erase_rsampdt();

    if (dt == 0)
    {
        return (AGC_STATUS_OK); /* TASKOVER */
    }

    int delay = ((dt > 0) ? dt : -dt) - 1; /* CCS      */

    if (!AGC_ASSERT(delay >= 1 && delay <= AGC_WAITLIST_MAX_DELAY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_waitlist_call((uint16_t) delay, AGC_TASK_RADSAMP);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_exec_novac(LEADIN_PRIO25, AGC_JOB_RADAR_SAMPLE);
    agc_sp_t loc = agc_erase_rtstloc();

    agc_erase_set_rtstdex((agc_sp_t) (loc / 2 + agc_erase_rtstbase()));

    return (st);
}

/* VARADAR (p.491): one sample of the radar RTSTDEX selects (RDRLOCS:
 * 0 RRRANGE, 1 RRRDOT, 2 LRVELX, 3 LRVELY, 4 LRVELZ, 5 LRALT).        */
static agc_status_t leadin_varadar(agc_sp_t dex)
{
    switch (dex)
    {
        case 0:
            return (agc_radar_rrrange());
        case 1:
            return (agc_radar_rrrdot());
        case 2:
            agc_radar_lr_velocity_beam(2, 1);

            return (AGC_STATUS_OK); /* LRVELX   */
        case 3:
            agc_radar_lr_velocity_beam(1, 1);

            return (AGC_STATUS_OK); /* LRVELY   */
        case 4:
            agc_radar_lr_velocity_beam(0, 1);

            return (AGC_STATUS_OK); /* LRVELZ   */
        case 5:
            agc_radar_lralt();

            return (AGC_STATUS_OK);
        default:
            return (AGC_STATUS_BAD_ARG); /* unreachable: cannot happen */
    }
}

/* DORSAMP (p.490): do the actual radar sample and wait in RADSTALL.     */
agc_status_t agc_job_radar_sample(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = leadin_varadar(agc_erase_rtstdex());

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;

    st = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, AGC_JOB_RADAR_SAMPLE_DONE, &res);

    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return (st);
    }

    return (rz_continue(AGC_JOB_RADAR_SAMPLE_DONE));
}

/* DORSAMP after RADSTALL / DORSAMP2: a bad reading advances RFAILCNT but
 * is accepted; RSTACK is not updated in R77; RTSTLOC cycles through the
 * DP slots up to RTSTMAX ("continuous sampling and 2N tries - gone").  */
agc_status_t agc_job_radar_sample_done(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) != AGC_IMU_STALL_GOOD)
    {
        agc_erase_set_rfailcnt((agc_sp_t) (agc_erase_rfailcnt() + 1));
    }

    agc_inhint();
    agc_sp_t loc = agc_erase_rtstloc();

    if (!agc_flag_test(AGC_FLAG_RADAR_ALARMS_SUPPRESSED) &&
        AGC_ASSERT(loc >= 0 && loc / 2 < LEADIN_RSTACK_N))
    {
        rstack[loc / 2] = agc_radar_samplsum();
    }

    agc_erase_set_rtstloc((loc == agc_erase_rtstmax()) ? 0 :
                          (agc_sp_t) (loc + 2));
    agc_relint();

    return (AGC_STATUS_OK); /* ENDOFJOB */
}
