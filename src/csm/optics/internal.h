/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/csm/optics/: the functions
 *   one file defines and another of this folder calls (no caller outside
 *   the folder, no test).
 *
 * AGC SOURCE:
 *   see the defining files
 *
 * REQUIRES:
 *   the agc/ public headers included below
 *
 * GLOBAL VARIABLES:
 *   Name          Type      Scale          Description
 *   ------------  --------  -------------  -------------------------
 *   none
 **********************************************************************/
#ifndef CSM_OPTICS_INTERNAL_H
#define CSM_OPTICS_INTERNAL_H

#include "agc/programs/alignment.h"

/* from agc/programs/alignment.h */
agc_status_t agc_sxtmark_reserve(void); /* SXTMARK +2    */

#endif /* CSM_OPTICS_INTERNAL_H */
