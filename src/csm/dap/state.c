/*
 * state.c -- EBANK6 storage of the CSM RCS and TVC autopilots
 * (ERASABLE_ASSIGNMENTS.agc pp. 102-110: "EXCLUSIVE RCS DAP VARIABLES",
 * "RCS/TVC DAP COMMON STORAGE", "TVC DAP TEMPORARY VARIABLES", "RCS DAP
 * TEMPORARY VARIABLES"). The AGC overlays the RCS and TVC temporaries on
 * the same words (TVCRCS, INTTEMP); here they are separate structs, which
 * is harmless because only one autopilot owns T5 at a time.
 *
 * Static storage (rule 3); other lanes reach it through the accessors.
 */
#include "internal.h"

static agc_csm_rcs_t    rcs;    /* RCS DAP erasables                     */
static agc_csm_tvc_t    tvc;    /* TVC DAP erasables                     */
static agc_csm_dapcom_t dapcom; /* RCS/TVC common storage and pad loads  */

agc_csm_rcs_t *agc_csm_dap_rcs(void)
{
    return &rcs;
}

agc_csm_tvc_t *agc_csm_dap_tvc(void)
{
    return &tvc;
}

agc_csm_dapcom_t *agc_csm_dap_com(void)
{
    return &dapcom;
}
