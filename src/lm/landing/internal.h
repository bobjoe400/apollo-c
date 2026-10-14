/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of src/lm/landing/: the functions
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
#ifndef LM_LANDING_INTERNAL_H
#define LM_LANDING_INTERNAL_H

#include "agc/lm/landing.h"

/* from agc/lm/landing.h */
void           agc_lm_fastchng(void);                           /* FASTCHNG  */
agc_sp_t       agc_lm_dv(agc_dp_t dividend, agc_sp_t divisor);  /* DV        */
agc_sp_t       agc_lm_mp_hi(agc_sp_t a, agc_sp_t b);            /* MP, high  */
agc_sp_t       agc_lm_ad(agc_sp_t a, agc_sp_t b);               /* AD, wraps */
agc_dp_words_t agc_lm_das(agc_dp_words_t x, agc_dp_words_t y);  /* DAS     */
bool           agc_lm_chan_bit_present(int chan, uint16_t bit); /* inverted ch30-33 */
agc_sp_t       agc_lm_sparcsin_doubled(agc_sp_t x);
agc_status_t   agc_lm_guidpass(const agc_job_ctx_t *ctx, int entry);
agc_status_t   agc_lm_dispexit(const agc_job_ctx_t *ctx, bool kill_group3);
agc_status_t   agc_lm_guildenstern(int *entry);                 /* R13 */
agc_status_t   agc_lm_redesig(agc_interp_t *ip);                /* REDESIG      */
agc_status_t   agc_lm_rate_of_descent_vertical(const agc_job_ctx_t *ctx);
agc_status_t   agc_lm_manual_landing_vertical(const agc_job_ctx_t *ctx, agc_interp_t *ip);
agc_sp_t       agc_lm_pipctr1(void);                            /* PIPCTR1 for LANDISP */

#endif /* LM_LANDING_INTERNAL_H */
