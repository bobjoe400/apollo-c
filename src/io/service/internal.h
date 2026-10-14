/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/io/service/: the functions
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
#ifndef IO_SERVICE_INTERNAL_H
#define IO_SERVICE_INTERNAL_H

#include "agc/io/service.h"

/* from agc/io/service.h */
agc_status_t agc_t4_optics_monitor(void);            /* OPTMON                   */
void         agc_t4_optics_drive(void);              /* OPTDRIVE                 */
agc_status_t agc_t4_optics_task(agc_task_id_t task); /* ENDZOPT              */
agc_status_t agc_t4_rr_monitor(void);                /* RRAUTCHK..RRGIMON+GPMATRIX */

#endif /* IO_SERVICE_INTERNAL_H */
