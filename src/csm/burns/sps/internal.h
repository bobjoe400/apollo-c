/*
 * internal.h -- pieces of the Comanche P40-P47 programs shared by
 * src/csm/burns/sps/burn.c (program start and countdown), ignition.c
 * (ignition to post-burn), steer.c (UPDATEVG, S40.8, S40.9,
 * S40.13, STEERING, N85/N83) and clock.c (CLOKTASK, CLOCKJOB,
 * V97/V99, S40.6, DAPFIG).
 */
#ifndef SPS_BURN_INTERNAL_H
#define SPS_BURN_INTERNAL_H

#include "agc/core/channels.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/rendezvous.h"

/* NVWORD1 of the CLOCKJOB: the V99 flash (-BIT9) and the V97 flash, which
 * the AGC marks with -0 (DEVIATION: -0 is not representable; -1 is used). */
#define AGC_CSM_NV_ENGINE_ENABLE_REQUEST ((agc_sp_t) -0400)
#define AGC_CSM_NV_ENGINE_FAIL_PROCEDURE ((agc_sp_t) -1)
/* FLAGWRD6 bits 15, 14 (DAPBIT1, DAPBIT2): T5 user, 10 = TVC, 01 = RCS.   */
#define AGC_CSM_T5BITS ((uint16_t) 060000)
#define AGC_CSM_T5_TVC ((uint16_t) 040000)
/* DSALMOUT (channel 11) bit 13: SPS engine on; channel 12 bit 14: SIV
 * cutoff; channel 12 bits 2, 8, 11 (OCT 02202): TVC enable / optics.     */
#define AGC_CSM_SPSON     ((uint16_t) 010000)
#define AGC_CSM_SIVCUTOFF ((uint16_t) 020000)
#define AGC_CSM_TVCOPT    ((uint16_t) 002202)
#define AGC_CSM_TVCOPTEN  ((uint16_t) 002200)
/* 3MDOT DEC 86.6175796 B-16: 3 s of SPS mass loss, kg 2^16.               */
#define AGC_CSM_3MDOT ((agc_sp_t) 22)

agc_sp_t     agc_csm_vn(uint8_t verb, uint8_t noun); /* VN as NVWORD1 */
bool         agc_csm_is_rcs_burn(void);              /* P41/P40       */
bool         agc_csm_tvc_on(void);                   /* FLAGWRD6 15,14 = 10 */
agc_status_t agc_csm_rcs_burn_display(agc_job_ctx_t *ctx); /* P41/DSP       */
agc_status_t agc_csm_preburn_start_clock(agc_job_ctx_t *ctx);        /* P40S/RS       */
void         agc_csm_convert_vg_body(agc_interp_t *ip); /* P40CNV85      */
agc_status_t agc_csm_servxt(agc_job_ctx_t *ctx);        /* SERVXT        */
agc_status_t agc_csm_preptvc(void);                     /* PREPTVC       */
agc_status_t agc_csm_vtogain_body(agc_interp_t *ip);    /* S40.9 to COPY40.9 */

#endif /* SPS_BURN_INTERNAL_H */
