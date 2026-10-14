/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/programs/targeting/: the functions
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
#ifndef PROGRAMS_TARGETING_INTERNAL_H
#define PROGRAMS_TARGETING_INTERNAL_H

#include "agc/programs/rendezvous.h"

/* from agc/programs/rendezvous.h */
agc_status_t agc_rendz_advance(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* S32/33.1 after the V06N81 display: DELVSIN = M^T DELVLVC, DELVSAB.      */
agc_status_t agc_rendz_csi_cdh_dv_to_reference(agc_interp_t *ip,
                                               agc_rendz_t  *rz,
                                               agc_vec_t    *delvsin);
agc_status_t agc_rendz_tpi_time_search(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* The same, continued at the ADTIME/GOINT step after an INTSTALL sleep. */
agc_status_t agc_rendz_tpi_time_search_cont(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

/* S3435.25 (p.475-476): RTARG VPASS4 given -> ACTCENT, DELLT4, INITVEL,
 * DELVLVC (also P38/P39 MAINRTNE).                                     */
agc_status_t agc_rendz_intercept_velocity(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t rtarg, agc_vec_t vpass4, agc_job_id_t resume);

/* P34/P74E (pp.464-465): TPASS4 from CENTANG, S34/35.2, DELVTPI,
 * DELVTPF, POSTTPI, TIG.                                                 */
agc_status_t agc_rendz_tpi_compute(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume);

#endif /* PROGRAMS_TARGETING_INTERNAL_H */
