/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/lm/burns/: the functions
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
#ifndef LM_BURNS_INTERNAL_H
#define LM_BURNS_INTERNAL_H

#include "agc/guidance/guidance.h"

/* from agc/guidance/guidance.h */
agc_status_t agc_lm_rasteer1(agc_interp_t *ip); /* RASTEER1    */

#endif /* LM_BURNS_INTERNAL_H */
