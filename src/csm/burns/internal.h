/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/csm/burns/: the functions
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
#ifndef CSM_BURNS_INTERNAL_H
#define CSM_BURNS_INTERNAL_H

#include "agc/guidance/guidance.h"

/* from agc/guidance/guidance.h */
void agc_guid_preread1_csm(void); /* PREREAD1 (P11 liftoff)  */

#endif /* CSM_BURNS_INTERNAL_H */
