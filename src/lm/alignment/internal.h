/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/lm/alignment/: the functions
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
#ifndef LM_ALIGNMENT_INTERNAL_H
#define LM_ALIGNMENT_INTERNAL_H

#include "agc/programs/alignment.h"

/* from agc/programs/alignment.h */
agc_status_t agc_lm_imu_check(void);

/* N89DISP: landing site at time t (ALPHAV, m B29) shown on V06N89; the
 * job continues as ret with the accepted site in agc_align_ram()->alphav. */
agc_status_t agc_lm_landing_site_latlong(agc_job_ctx_t *ctx, agc_dp_t t, agc_job_id_t ret);

/* R52 (LM): point the AOT detent at the star; continues as ret.          */
agc_status_t agc_lm_auto_optics_positioning(const agc_job_ctx_t *ctx, agc_job_id_t ret);
agc_status_t agc_lm_lunar_surface_gravity(void);                   /* GVDETER  */
agc_status_t agc_lm_lunar_surface_just_trim(void);                 /* JUSTTRIM */
agc_status_t agc_lm_lunar_surface_coarse_trim(agc_job_ctx_t *ctx); /* COATRIM  */
agc_status_t agc_lm_lunar_surface_two_stars(agc_job_ctx_t *ctx);   /* 2STARS   */
agc_status_t agc_lm_lunar_surface_one_star(agc_job_ctx_t *ctx);    /* 1STAR    */
agc_status_t agc_lm_lunar_surface_surfline(agc_job_ctx_t *ctx);    /* SURFLINE */
agc_status_t agc_job_lunar_surface_attitude_check(agc_job_ctx_t *ctx);

#endif /* LM_ALIGNMENT_INTERNAL_H */
