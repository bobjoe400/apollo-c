/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/io/verbs/: the functions
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
#ifndef IO_VERBS_INTERNAL_H
#define IO_VERBS_INTERNAL_H

#include "agc/io/service.h"

/* from agc/io/service.h */
int          agc_extverb_noun(void);
agc_status_t agc_extverb_testxact(bool *ok);        /* TESTXACT: false = ALM/END done */
agc_status_t agc_extverb_alm_end(void);             /* ALM/END: FALTON, PINBRNCH     */
agc_status_t agc_extverb_chkpooh(bool *p00);        /* CHKPOOH: P00 (ALM/END if not) */
agc_status_t agc_extverb_goloadlv(void);            /* GOLOADLV: please-perform done */
agc_status_t agc_extverb_upflag(agc_flag_id_t f);   /* TC UPFLAG, TC GOPIN       */
agc_status_t agc_extverb_downflag(agc_flag_id_t f); /* TC DOWNFLAG, TC GOPIN     */
agc_status_t agc_extverb_schedule(agc_prio_t prio, agc_job_id_t job, bool vac);

/* Request a flashing display for the running verb (GOXDSPF = GOMARKF).    */
agc_status_t agc_extverb_flash(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t on_proceed, agc_job_id_t on_enter);
agc_status_t agc_extverb_rr_zero(void);                        /* V40 N72 */
agc_status_t agc_extverb_rr_designate(agc_job_ctx_t *ctx);     /* V41 N72 */
agc_status_t agc_extverb_lr_position_two(void);                /* V60     */
agc_status_t agc_extverb_radar_test(agc_job_ctx_t *ctx);       /* V63 R04 */
agc_status_t agc_extverb_lr_spurious_test(agc_job_ctx_t *ctx); /* V78 R77 */
agc_status_t agc_extverb_lr_spurious_test_end(void);           /* V79    */
agc_status_t agc_extverb_rr_los_display(agc_job_ctx_t *ctx);   /* V85     */
agc_status_t agc_extverb_state_vector_copy(bool from_other);   /* V66 / CSM V47 */
agc_status_t agc_extverb_lm_attck2(void);                      /* LM V43 ATTCK2 (task body)  */
agc_status_t agc_job_attitude_error_load_csm(void);
agc_status_t agc_job_attitude_error_load_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_tracking_terminate_csm(void);
agc_status_t agc_job_tracking_terminate_lm(void);
agc_status_t agc_job_dap_data_load_noun46_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun46_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_config_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_config_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun47_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun47_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_mass_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_mass_lm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun48_csm(agc_job_ctx_t *ctx);
agc_status_t agc_job_dap_data_load_noun48_lm(agc_job_ctx_t *ctx);
agc_status_t agc_extverb_csm(agc_job_ctx_t *ctx, int verb);
agc_status_t agc_extverb_lm(agc_job_ctx_t *ctx, int verb);

#endif /* IO_VERBS_INTERNAL_H */
