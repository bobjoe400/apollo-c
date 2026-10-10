/*
 * entry_internal.h -- functions shared between the files of the guid-entry
 * lane only (src/csm/reentry.c, p61_p67.c, cm_body_attitude.c,
 * entry_dap.c). Not part of the public API.
 */
#ifndef AGC_ENTRY_INTERNAL_H
#define AGC_ENTRY_INTERNAL_H

#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_csm_entry.h"

/* reentry.c */
agc_interp_t *entry_ip(void);                 /* shared interpreter state (OVFIND) */
agc_dp_t entry_signmpac(agc_dp_t x);          /* SIGNMPAC: +-DPOSMAX               */
agc_dp_t entry_log(agc_interp_t *ip, agc_dp_t x); /* LOG: -ln(x)/32               */
agc_status_t entry_earrot(agc_interp_t *ip, agc_entry_state_t *es, bool first); /* EARROT1/2 */
agc_status_t entry_endexit(agc_entry_state_t *es); /* ENDEXIT/OVERNOUT/SERVNOUT    */
agc_status_t entry_starten1(agc_interp_t *ip, agc_entry_state_t *es); /* STARTEN1 */
agc_dp_t entry_arctrig(agc_interp_t *ip, agc_dp_t sinth4, agc_dp_t costh4); /* ARCTRIG */

/* p61_p67.c: mode/display changes called from the steering via RTB.        */
agc_status_t entry_p64(agc_entry_state_t *es);
agc_status_t entry_p65(agc_entry_state_t *es);
agc_status_t entry_p66(agc_entry_state_t *es);
agc_status_t entry_p67(agc_entry_state_t *es);
agc_status_t entry_killgrp4(void);
agc_status_t entry_newmodex(int major_mode); /* NEWMODEX: MODREG + display    */

/* entry_dap.c */
agc_sp_t entry_angovcor(agc_sp_t a, agc_ovf_t ovf); /* ANGOVCOR / CORANGOV   */

#endif /* AGC_ENTRY_INTERNAL_H */
