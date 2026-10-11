/**********************************************************************
 * FILE NAME: state.c
 *
 * PURPOSE:
 *   EBANK6 storage of the CSM RCS and TVC autopilots ("EXCLUSIVE RCS
 *   DAP VARIABLES", "RCS/TVC DAP COMMON STORAGE", "TVC DAP TEMPORARY
 *   VARIABLES", "RCS DAP TEMPORARY VARIABLES") and its accessors. The AGC
 *   overlays the RCS and TVC temporaries on the same words (TVCRCS,
 *   INTTEMP); here they are separate structs, which is harmless because
 *   only one autopilot owns T5 at a time.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc pp. 102-110
 *
 * EXTERNAL VARIABLES:
 *   none (static storage, rule 3; other lanes reach it through the
 *   accessors)
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none
 **********************************************************************/
#include "internal.h"

static agc_csm_rcs_t    rcs;    /* RCS DAP erasables                     */
static agc_csm_tvc_t    tvc;    /* TVC DAP erasables                     */
static agc_csm_dapcom_t dapcom; /* RCS/TVC common storage and pad loads  */

/**********************************************************************
 * AGC: none (accessor of ERASABLE_ASSIGNMENTS.agc pp. 108-110 storage)
 * -- the RCS DAP erasables.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_csm_rcs_t * -- the RCS DAP block
 **********************************************************************/
agc_csm_rcs_t *agc_csm_dap_rcs(void)
{
    return (&rcs);
}

/**********************************************************************
 * AGC: none (accessor of ERASABLE_ASSIGNMENTS.agc pp. 104-107 storage)
 * -- the TVC DAP erasables.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_csm_tvc_t * -- the TVC DAP block
 **********************************************************************/
agc_csm_tvc_t *agc_csm_dap_tvc(void)
{
    return (&tvc);
}

/**********************************************************************
 * AGC: none (accessor of ERASABLE_ASSIGNMENTS.agc pp. 102-103 storage)
 * -- the RCS/TVC common storage and pad loads.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_csm_dapcom_t * -- the common block
 **********************************************************************/
agc_csm_dapcom_t *agc_csm_dap_com(void)
{
    return (&dapcom);
}
