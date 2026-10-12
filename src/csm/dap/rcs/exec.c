/**********************************************************************
 * FILE NAME: exec.c
 *
 * PURPOSE:
 *   The executive program of the RCS-CSM autopilot: AMBGUPDT, the
 *   calculation of the gimbal-to-body matrix AMGB once every second (job,
 *   PRIO34, from KMATRIX and IMUAOK):
 *     AMGB = | 1   SIN(PSI)            0        |
 *            | 0   COS(PSI)COS(PHI)    SIN(PHI) |
 *            | 0  -COS(PSI)SIN(PHI)    COS(PHI) |
 *   where PSI = CDUZ and PHI = CDUX - 7.25 deg (the jet quad angular
 *   offset).
 *
 * AGC SOURCE:
 *   Comanche055 RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc pp. 1037-1038
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   FLAGWRD6      flags     I    bits           DAPBIT1, DAPBIT2
 *   CDUX, CDUZ    uint16_t  I    CDU counts     gimbal angles
 *   AMGB1..AMGB8  agc_sp_t  O    scale 1        matrix elements
 *   CAPSI         agc_sp_t  O    scale 1        cos(PSI)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_sp_sin, _cos      SPSIN1/2, SPCOS1/2 (half-revolution scale)
 *   csm_dap_msu, _mp_hi   MSU, MP (mysubs.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG for a missing job context.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Exits at once unless FLAGWRD6 bits 15,14 = 01 (RCS autopilot), to
 *   protect the TVC DAP on switch-over.
 **********************************************************************/
#include "../internal.h"

#define QUADANGL ((uint16_t) 660) /* 01224 DEC 660 = 7.25 deg, 180 deg scale */

/**********************************************************************
 * AMBGUPDT (RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc pp. 1037-1038) -- the
 * gimbal-to-body matrix from CDUZ and CDUX - 7.25 deg.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (ENDOFJOB, always)
 **********************************************************************/
agc_status_t agc_job_ambgupdt(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* RCS DAP erasables          */

    /* CA FLAGWRD6; BZMF ENDOFJOB; MASK BIT14; BZF ENDOFJOB               */
    if (!(agc_flag_test(AGC_FLAG_DAPBIT1) || !agc_flag_test(AGC_FLAG_DAPBIT2)))
    {
        agc_sp_t psi = csm_dap_word_sp(agc_erase_cduz()); /* CA CDUZ, scale 180 deg */

        r->amgb1 = agc_sp_sin(psi); /* TC SPSIN2; TS AMGB1                    */
        r->capsi = agc_sp_cos(psi); /* TC SPCOS2; TS CAPSI (gimbal lock)      */

        /* CAF QUADANGL; MSU CDUX; COM: CDUX - 7.25 DEG                        */
        agc_sp_t phi = (agc_sp_t) -csm_dap_msu(QUADANGL, agc_erase_cdux()); /* scale 180 deg */

        r->amgb8 = agc_sp_cos(phi);                               /* SPCOS1          */
        r->amgb4 = csm_dap_mp_hi(r->amgb8, r->capsi);             /* MP CAPSI        */
        r->amgb5 = agc_sp_sin(phi);                               /* SPSIN1          */
        r->amgb7 = (agc_sp_t) -csm_dap_mp_hi(r->amgb5, r->capsi); /* MP CAPSI; COM   */
    }

    return (AGC_STATUS_OK); /* ENDOFJOB                                 */
}
