/**********************************************************************
 * FILE NAME: guid_state.c
 *
 * PURPOSE:
 *   The erasable block owned by the powered-flight guidance routines
 *   (POWERED_FLIGHT_SUBROUTINES, SERVICER207/SERVICER, P30-P47,
 *   BURN_BABY_BURN). The AGC keeps these in EBANKs 3-7 under the names
 *   given in agc/guidance/guidance.h; here they are one static structure
 *   reached through agc_guid_state(), so that the program, servicer and
 *   steering modules of both vehicles share them the way the AGC
 *   routines share erasable.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc, FRESH_START_AND_RESTART.agc
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc, FRESH_START_AND_RESTART.agc
 *   (the guidance cells and their fresh-start values)
 *
 * EXTERNAL VARIABLES:
 *   none (this file owns the block; others use agc_guid_state())
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_vec_make          builds the UNITW pad value (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   AGC_STATUS_BAD_ARG from agc_guid_init for an unknown vehicle.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles; agc_guid_init runs at fresh start.
 *
 * NOTES:
 *   The resume-job cell is C-only: the AGC's INTSTALL sleeps the calling
 *   job itself, the C routines name the job to resume instead.
 *
 * ALGORITHM:
 *   Zero the block, then store the fresh-start and pad-load defaults.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"

static agc_guid_t g_guid; /* the guidance erasables (see guidance.h) */

/**********************************************************************
 * The one guidance erasable block. AGC: none (C accessor for the
 * guidance erasables of ERASABLE_ASSIGNMENTS.agc).
 *
 * FUNCTION NAME: agc_guid_state
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_guid_t * -- the block, never NULL
 **********************************************************************/
agc_guid_t *agc_guid_state(void)
{
    return (&g_guid);
}

/**********************************************************************
 * Fresh start of the guidance erasables: everything zero, then the values
 * the AGC fresh start (FRESH_START_AND_RESTART.agc) or the pad loads put
 * there. The pad loads below are the nominal Apollo 11 values quoted in
 * the listings' comments; the sim harness overwrites them with a real
 * pad load.
 *
 * FUNCTION NAME: agc_guid_init
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   veh       agc_vehicle_t  I    vehicle: AGC_VEHICLE_CSM or
 *                                 AGC_VEHICLE_LM
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (always)
 **********************************************************************/
agc_status_t agc_guid_init(agc_vehicle_t veh)
{
    agc_guid_t zero = {0}; /* all cells +0 */

    g_guid = zero;

    /* Average-G defaults: AVGEXIT = SERVEXIT (normal exit), UNITW = earth's
     * rotation axis along reference Z as a full unit vector (CM pad load,
     * DPPOSMAX), PIPAGE = +1 (no reading in progress).                        */
    agc_avg_g_t *a = &g_guid.avg; /* average-G block */

    a->avgexit = AGC_JOB_SERVEXIT;
    a->unitw   = agc_vec_make(0, 0, AGC_DP_MAX);
    a->pipage  = 1;
    a->gdt_2   = agc_vec_zero();

    g_guid.burn.nbrcycls = -1; /* no S40.9 cycle under way      */

    if (veh == AGC_VEHICLE_CSM)
    {
        g_guid.csm.etdecay  = 0; /* ETDECAY pad load (cs, 2^14)    */
        g_guid.csm.dvthresh = 0; /* DVTHRESH pad load (2^-2 m/cs)  */
    }
    else
    {
        g_guid.lm.outroute = AGC_JOB_ENDOFJOB; /* ENDJBCAD: SERVEXIT +2 */
        g_guid.lm.dispdex  = 1;                /* positive: CLOKTASK not running */
        g_guid.lm.vselect  = 2;                /* VSELECT: X beam first          */
        g_guid.lm.which    = (agc_sp_t) AGC_LM_WHICH_DPS_BURN;
    }

    return (AGC_STATUS_OK);
} /* agc_guid_init */

/* The job that resumes after an INTSTALL wait when a targeting subroutine
 * (S30.1, S31.1, AGAIN, S40.1) has no job context of its own: set by the
 * job that calls it.                                                     */
static agc_job_id_t g_resume = AGC_JOB_NONE; /* job resumed after INTSTALL */

/**********************************************************************
 * Name the job to resume when a targeting subroutine (S30.1, S31.1,
 * AGAIN, S40.1) has to wait in INTSTALL. AGC: none (the AGC sleeps the
 * calling job itself).
 *
 * FUNCTION NAME: agc_guid_nav_set_resume
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   job       agc_job_id_t  I    job to resume after the navigation glue
 **********************************************************************/
void agc_guid_nav_set_resume(agc_job_id_t job)
{
    g_resume = job;
}

/**********************************************************************
 * The job named by agc_guid_nav_set_resume. AGC: none (C-only cell).
 *
 * FUNCTION NAME: agc_guid_nav_resume
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_job_id_t -- the job, or AGC_JOB_NONE if none was named
 **********************************************************************/
agc_job_id_t agc_guid_nav_resume(void)
{
    return (g_resume);
}
