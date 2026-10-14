/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/guidance/: the functions
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
#ifndef GUIDANCE_INTERNAL_H
#define GUIDANCE_INTERNAL_H

#include "agc/guidance/guidance.h"

/* from agc/guidance/guidance.h */
agc_job_id_t agc_guid_nav_resume(void);
agc_status_t agc_guid_midgim(agc_interp_t *ip, agc_vec_t dv);  /* MIDGIM (CM) */
agc_status_t agc_guid_again(agc_interp_t *ip, agc_dp_t tdec1); /* AGAIN (CM) */

#endif /* GUIDANCE_INTERNAL_H */
