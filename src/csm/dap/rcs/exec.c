/*
 * exec.c -- executive programs of the RCS-CSM autopilot.
 * RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc (Comanche055 pp. 1037-1038): AMBGUPDT.
 *
 * Calculation of AMGB once every second (job, PRIO34, from KMATRIX and
 * IMUAOK):
 *     AMGB = | 1   SIN(PSI)            0        |
 *            | 0   COS(PSI)COS(PHI)    SIN(PHI) |
 *            | 0  -COS(PSI)SIN(PHI)    COS(PHI) |
 * where PHI and PSI are CDU angles; PHI is taken 7.25 degrees from CDUX
 * for the jet quad angular offset. Elements scaled 1.
 */
#include "../internal.h"

#define QUADANGL ((uint16_t) 660) /* 01224 DEC 660 = 7.25 degrees        */

/* AMBGUPDT (pp. 1037-1038).                                              */
agc_status_t agc_job_ambgupdt(agc_job_ctx_t *ctx)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();

    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    /* FLAGWRD6 bit 15 = 0, bit 14 = 1 for the RCS autopilot; otherwise
     * exit, to protect the TVC DAP on switch-over.                        */
    if (agc_flag_test(AGC_FLAG_DAPBIT1) || !agc_flag_test(AGC_FLAG_DAPBIT2))
    {
        return AGC_STATUS_OK; /* ENDOFJOB             */
    }

    agc_sp_t psi = csm_dap_word_sp(agc_erase_cduz()); /* CA CDUZ (raw word) */

    r->amgb1 = agc_sp_sin(psi);                   /* SPSIN2               */
    r->capsi = agc_sp_cos(psi);                   /* SPCOS2: gimbal lock  */
    /* CAF QUADANGL; MSU CDUX; COM: CDUX - 7.25 DEG                        */
    agc_sp_t phi = (agc_sp_t) -csm_dap_msu(QUADANGL, agc_erase_cdux());

    r->amgb8 = agc_sp_cos(phi); /* SPCOS1               */
    r->amgb4 = csm_dap_mp_hi(r->amgb8, r->capsi);
    r->amgb5 = agc_sp_sin(phi); /* SPSIN1               */
    r->amgb7 = (agc_sp_t) -csm_dap_mp_hi(r->amgb5, r->capsi);

    if (!AGC_ASSERT(r->capsi >= AGC_SP_MIN))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK; /* ENDOFJOB             */
}
