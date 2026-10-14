/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/programs/alignment/: the functions
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
#ifndef PROGRAMS_ALIGNMENT_INTERNAL_H
#define PROGRAMS_ALIGNMENT_INTERNAL_H

#include "agc/programs/alignment.h"

/* from agc/programs/alignment.h */
agc_status_t agc_align_gimbal_angles(agc_interp_t *ip, const agc_mat_t *smd, const agc_mat_t *refsmmat, const uint16_t cdu[3], agc_mat_t *xsm, agc_mat_t *xnb, uint16_t thetad[3]);
agc_status_t agc_align_nominal_orientation(agc_interp_t *ip, agc_dp_t talign, agc_job_id_t resume_job, agc_mat_t *smd);
bool         agc_align_coarse_needed(const uint16_t thetad[3], const uint16_t cdu[3]);
agc_status_t agc_align_planet_vector(agc_interp_t *ip, agc_sp_t star6, agc_sp_t starind, agc_dp_t time_cs, agc_job_id_t resume_job, agc_vec_t *los);

#endif /* PROGRAMS_ALIGNMENT_INTERNAL_H */
