/**********************************************************************
 * FILE NAME: flags_csm.c
 *
 * PURPOSE:
 *   The Comanche055-only flags the integration package touches (they
 *   exist only in flags_csm.h): V96ONFLG, cleared by STATINT1, and
 *   AMOONFLG, the sphere of influence of AVERAGEG's RN, VN state.
 *
 * AGC SOURCE:
 *   Comanche055 INTEGRATION_INITIALIZATION.agc p. 1313 (STATINT1),
 *   p. 1327 (SETCOAST), p. 1330 (MIDTOAV1/2); not in Luminary099
 *
 * EXTERNAL VARIABLES:
 *   Name      Type  I/O  Scale  Description
 *   --------  ----  ---  -----  ---------------------------------------
 *   V96ONFLG  flag  O    -      idle integration inhibited by V96
 *   AMOONFLG  flag  I/O  -      RN, VN of AVERAGEG are moon-centred
 *
 * EXTERNAL REFERENCES:
 *   Name           Description
 *   -------------  ---------------------------------------------
 *   agc_flag_put   SET / CLEAR a flag (flags_common.h)
 *   agc_flag_test  BON / BOFF (flags_common.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only: the callers test the vehicle first.
 **********************************************************************/
#include "agc/core/flags_csm.h"
#include "internal.h"

/**********************************************************************
 * STATINT1 (INTEGRATION_INITIALIZATION.agc C p. 1313) -- CLEAR V96ONFLG:
 * P00 integration allowed again
 *
 * FUNCTION NAME: agc_integ_csm_resume_idle_integration
 **********************************************************************/
void agc_integ_csm_resume_idle_integration(void)
{
    agc_flag_put(AGC_FLAG_IDLE_INTEGRATION_INHIBITED, false);
}

/**********************************************************************
 * SETCOAST (INTEGRATION_INITIALIZATION.agc C p. 1327) -- read AMOONFLG
 *
 * FUNCTION NAME: agc_integ_csm_amoonflg
 *
 * RETURNS: bool -- true when AVERAGEG's RN, VN are moon-centred
 **********************************************************************/
bool agc_integ_csm_amoonflg(void)
{
    return (agc_flag_test(AGC_FLAG_AMOONFLG));
}

/**********************************************************************
 * MIDTOAV1/2 (INTEGRATION_INITIALIZATION.agc C p. 1330) -- AMOONFLG from
 * the sphere of influence of the integrated state (X2)
 *
 * FUNCTION NAME: agc_integ_csm_set_amoonflg
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  --------------------------------------------
 *   on        bool  I    true: moon sphere (X2 = 2)
 **********************************************************************/
void agc_integ_csm_set_amoonflg(bool on)
{
    agc_flag_put(AGC_FLAG_AMOONFLG, on);
}
