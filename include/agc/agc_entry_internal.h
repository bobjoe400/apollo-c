/*
 * entry_internal.h -- functions shared between the files of the guid-entry
 * lane only (src/csm/entry/reentry.c, programs.c, cm_body_attitude.c,
 * dap.c, earth_orbit_insertion.c). Not part of the public API.
 */
#ifndef AGC_ENTRY_INTERNAL_H
#define AGC_ENTRY_INTERNAL_H

#include <stdint.h>
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_csm_entry.h"

/* reentry.c */
agc_interp_t *entry_ip(void);                 /* shared interpreter state (OVFIND) */
agc_dp_t entry_signmpac(agc_dp_t x);          /* SIGNMPAC: +-DPOSMAX               */
agc_dp_t entry_log(agc_interp_t *ip, agc_dp_t x); /* LOG: -ln(x)/32               */
agc_status_t entry_earrot(agc_interp_t *ip, agc_entry_state_t *es, bool first); /* EARROT1/2 */
agc_status_t entry_endexit(agc_job_ctx_t *ctx, agc_entry_state_t *es); /* OVERNOUT */
agc_status_t entry_starten1(agc_interp_t *ip, agc_entry_state_t *es); /* STARTEN1 */

/* cm_body_attitude.c: SERVNOUT (CHANG1 if a job waits, then SERVEXIT).  */
agc_status_t entry_servnout(agc_job_ctx_t *ctx);

/* programs.c: mode/display changes called from the steering via RTB.        */
agc_status_t entry_post_0_05g(agc_entry_state_t *es);
agc_status_t entry_upcontrol(agc_entry_state_t *es);
agc_status_t entry_ballistic(agc_entry_state_t *es);
agc_status_t entry_final_phase(agc_entry_state_t *es);
agc_status_t entry_killgrp4(void);

/* dap.c */
/* ANGOVCOR / CORANGOV: an SP angle/180 sum (at most one overflow) wrapped
 * back into [-1, 1) (i.e. modulo 360 deg).                               */
agc_sp_t entry_angle_wrap(int32_t sum);
agc_sp_t entry_sp_mp(agc_sp_t a, agc_sp_t b);  /* MP: high word of a b      */
agc_sp_t entry_sp_sat(int32_t sum);            /* AD whose result is kept
                                                  (overflow not expected)  */

#endif /* AGC_ENTRY_INTERNAL_H */
