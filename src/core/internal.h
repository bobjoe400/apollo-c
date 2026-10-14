/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/core/: the functions
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
#ifndef CORE_INTERNAL_H
#define CORE_INTERNAL_H

#include "agc/core/state.h"

/* from agc/core/state.h */
void agc_erase_reset_counters(void);
void agc_erase_reset_display(void);
void agc_erase_reset_pad_loads(void);
void agc_erase_reset_restart_delays(void);

#endif /* CORE_INTERNAL_H */
