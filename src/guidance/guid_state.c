/*
 * guid_state.c -- the erasable block owned by the powered-flight guidance
 * routines (POWERED_FLIGHT_SUBROUTINES, SERVICER207/SERVICER, P30-P47,
 * BURN_BABY_BURN). The AGC keeps these in EBANKs 3-7 under the names given
 * in agc_guid.h; here they are one static structure reached through
 * agc_guid_state(), so that the program, servicer and steering modules of
 * both vehicles share them the way the AGC routines share erasable.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"

static agc_guid_t g_guid;

/* The one guidance erasable block.                                        */
agc_guid_t *agc_guid_state(void)
{
    return (&g_guid);
}

/* Zero a vector (no interpreter needed).                                  */

/* Average-G defaults: AVGEXIT = SERVEXIT (normal exit), UNITW = earth's
 * rotation axis along reference Z as a full unit vector (CM pad load,
 * DPPOSMAX), PIPAGE = +1 (no reading in progress).                        */
static void init_avg(agc_avg_g_t *a)
{
    if (!AGC_ASSERT(a != NULL))
    {
        return;
    }

    a->avgexit = AGC_JOB_SERVEXIT;
    a->unitw   = agc_vec_make(0, 0, AGC_DP_MAX);
    a->pipage  = 1;
    a->gdt_2   = agc_vec_zero();

    if (!AGC_ASSERT(a->avgexit != AGC_JOB_NONE))
    {
        return;
    }

    if (!AGC_ASSERT(a->unitw.c[2] > 0))
    {
        return;
    }
}

/* Fresh start of the guidance erasables: everything zero, then the values
 * the AGC fresh start or the pad loads put there. The pad loads below are
 * the nominal Apollo 11 values quoted in the listings' comments; the sim
 * harness overwrites them with a real pad load.                           */
agc_status_t agc_guid_init(agc_vehicle_t veh)
{
    if (!AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_guid_t zero = {0};

    g_guid = zero;
    init_avg(&g_guid.avg);
    g_guid.burn.nbrcycls = -1; /* no S40.9 cycle under way      */

    if (veh == AGC_VEHICLE_CSM)
    {
        g_guid.csm.etdecay  = 0; /* ETDECAY pad load (cs, 2^14)    */
        g_guid.csm.dvthresh = 0; /* DVTHRESH pad load (2^-2 m/cs)  */
    }
    else
    {
        g_guid.lm.outroute = AGC_JOB_ENDOFJOB; /* ENDJBCAD: SERVEXIT +2 */
        g_guid.lm.dispdex  = 1; /* positive: CLOKTASK not running */
        g_guid.lm.vselect  = 2; /* VSELECT: X beam first          */
        g_guid.lm.which    = (agc_sp_t) AGC_LM_WHICH_DPS_BURN;
    }

    if (!AGC_ASSERT(g_guid.avg.avgexit == AGC_JOB_SERVEXIT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* The job that resumes after an INTSTALL wait when a targeting subroutine
 * (S30.1, S31.1, AGAIN, S40.1) has no job context of its own: set by the
 * job that calls it.                                                     */
static agc_job_id_t g_resume = AGC_JOB_NONE;

void agc_guid_nav_set_resume(agc_job_id_t job)
{
    if (!AGC_ASSERT(job != AGC_JOB_NONE))
    {
        return;
    }

    g_resume = job;
}

agc_job_id_t agc_guid_nav_resume(void)
{
    if (!AGC_ASSERT(g_resume != AGC_JOB_NONE))
    {
        return (AGC_JOB_NONE);
    }

    return (g_resume);
}
