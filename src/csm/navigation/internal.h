/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/csm/navigation/: the functions
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
#ifndef CSM_NAVIGATION_INTERNAL_H
#define CSM_NAVIGATION_INTERNAL_H

#include "agc/programs/rendezvous.h"

/* from agc/programs/rendezvous.h */
void     agc_rendz_llasrd(void);
void     agc_rendz_llasrda(void);
agc_sp_t agc_erase_r61cntr(void); /* R61CNTR (CM)       */
void     agc_erase_set_r61cntr(agc_sp_t v);

#endif /* CSM_NAVIGATION_INTERNAL_H */
