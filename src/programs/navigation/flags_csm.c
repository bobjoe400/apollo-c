/**********************************************************************
 * FILE NAME: flags_csm.c
 *
 * PURPOSE:
 *   Comanche055 flag ids of the rendezvous flags whose position differs
 *   between the vehicles, or which exist in one vehicle only (id 0 =
 *   absent); rz_vflag (internal.h) picks this table on the CSM.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc (flag word assignments), through
 *   include/agc/core/flags_csm.h
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name          Description
 *   ------------  ------------------------------------------------
 *   agc_flag_bit  bit position of a flag id (consistency check)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none
 **********************************************************************/
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "internal.h"

/**********************************************************************
 * rz_vflag_csm -- CSM flag id for a vehicle-dependent rendezvous flag.
 * AGC: ERASABLE_ASSIGNMENTS.agc (flag words; CSM bit assignments)
 *
 * FUNCTION NAME: rz_vflag_csm
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   f         rz_vflag_t  I    vehicle-dependent rendezvous flag (RZ_VF_*)
 *
 * RETURNS: agc_flag_id_t -- the CSM flag id, 0 when the flag does not exist
 *          in Comanche055
 **********************************************************************/
agc_flag_id_t rz_vflag_csm(rz_vflag_t f)
{
    static const agc_flag_id_t ids[RZ_VF_COUNT] = {
        AGC_FLAG_ITSWICH /* ITSWICH */,
        AGC_FLAG_AVEGFLAG /* AVEGFLAG */,
        AGC_FLAG_ORBIT_DISPLAY_MOON /* V82EMFLG */,
        AGC_FLAG_GROUND_TRACK_BASE_VECTORS /* P21FLAG */,
        AGC_FLAG_S32_1F1 /* S32_1F1 */,
        AGC_FLAG_S32_1F2 /* S32_1F2 */,
        AGC_FLAG_S32_1F3A /* S32_1F3A */,
        AGC_FLAG_S32_1F3B /* S32_1F3B */,
        0U /* NTARGFLG */,
        AGC_FLAG_COMPUTER /* COMPUTER */,
        AGC_FLAG_AMOONFLG /* AMOONFLG */,
        AGC_FLAG_RENDEZVOUS_DISPLAY_THETA /* R31FLAG */,
        AGC_FLAG_SERVICER_RUNNING /* V37FLAG */,
        0U /* MUNFLAG */,
        AGC_FLAG_TIMRFLAG /* TIMRFLAG */
    };

    return (ids[f]);
}
