/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/lm/radar/: the functions
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
#ifndef LM_RADAR_INTERNAL_H
#define LM_RADAR_INTERNAL_H

#include "agc/programs/rendezvous.h"

/* from agc/programs/rendezvous.h */
/* RRDESSM: designate to RRTARGET (half unit, SM; set by LPS20.1).       */
agc_status_t agc_rr_designate_sm(agc_interp_t *ip, agc_rr_designate_result_t *res);

/* RRDESNB: designate to RR gimbal angles (trunnion, shaft CDU words).   */
agc_status_t agc_rr_designate_nb(agc_interp_t *ip, uint16_t trunnion, uint16_t shaft, agc_rr_designate_result_t *res);
void         agc_radar_lr_velocity_beam(int beam, agc_sp_t nsamples); /* LRVELZ/Y/X */

#endif /* LM_RADAR_INTERNAL_H */
