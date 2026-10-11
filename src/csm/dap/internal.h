/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interfaces between the files of the CSM RCS and TVC
 *   autopilots (src/csm/dap/); not part of the public API. Word
 *   conventions used throughout the lane: CDU counters, THETADX.., OGAD,
 *   RHO and TIME5/TIME6 are raw 15-bit words (uint16_t), because the AGC
 *   mixes two's complement CDU counts with ones' complement arithmetic on
 *   them (MSU, CA/TS); DP erasables whose words the AGC uses separately
 *   are agc_dp_words_t; every other SP quantity is an agc_sp_t value.
 *   Ones' complement -0 is not represented (TRANSLATION_GUIDE section 1).
 *
 * AGC SOURCE:
 *   Comanche055 MYSUBS.agc, RCS-CSM_DIGITAL_AUTOPILOT.agc,
 *   AUTOMATIC_MANEUVERS.agc, RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc,
 *   JET_SELECTION_LOGIC.agc, TVC*.agc (pp. 938-1062)
 *
 * REQUIRES:
 *   agc/csm/dap.h and the core headers included below
 *
 * GLOBAL VARIABLES:
 *   none (the DAP blocks are reached through agc_csm_dap_rcs/_tvc/_com)
 **********************************************************************/
#ifndef CSM_DAP_INTERNAL_H
#define CSM_DAP_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/dap.h"
#include "agc/exec/exec.h"

/* ---- AGC fixed constants used by several files ------------------------- */
#define CSM_DAP_BIT1  ((uint16_t) 000001) /* BIT1 */
#define CSM_DAP_BIT2  ((uint16_t) 000002) /* BIT2 */
#define CSM_DAP_BIT3  ((uint16_t) 000004) /* BIT3 */
#define CSM_DAP_BIT4  ((uint16_t) 000010) /* BIT4 */
#define CSM_DAP_BIT6  ((uint16_t) 000040) /* BIT6 */
#define CSM_DAP_BIT7  ((uint16_t) 000100) /* BIT7 */
#define CSM_DAP_BIT10 ((uint16_t) 001000) /* BIT10 */
#define CSM_DAP_BIT11 ((uint16_t) 002000) /* BIT11 */
#define CSM_DAP_BIT12 ((uint16_t) 004000) /* BIT12 */
#define CSM_DAP_BIT13 ((uint16_t) 010000) /* BIT13 */
#define CSM_DAP_BIT14 ((uint16_t) 020000) /* BIT14 */
#define CSM_DAP_BIT15 ((uint16_t) 040000) /* BIT15 */
#define CSM_DAP_NEG0  ((uint16_t) 077777) /* ones' complement -0 word        */
#define CSM_DAP_POSMAX_W \
    ((uint16_t) 037777)                   /* TIME5 = POSMAX: rupt in 10 ms   */
#define CSM_DAP_PRIO34 ((agc_prio_t) 034) /* PRIO34 (octal 34000 >> 9)       */
#define CSM_DAP_TIME5  5                  /* TIME5 counter number (agc_rupt_set_time) */
#define CSM_DAP_TIME6  6                  /* TIME6 counter number                       */

/* ---- word arithmetic in the AGC's manner (mysubs.c) --------------------- */
agc_sp_t csm_dap_word_sp(uint16_t w);                /* 1's comp word -> value  */
uint16_t csm_dap_sp_word(agc_sp_t v);                /* value -> 1's comp word  */
agc_sp_t csm_dap_msu(uint16_t a, uint16_t b);        /* MSU: (a - b) mod 2^15   */
agc_sp_t csm_dap_ads(agc_sp_t a, agc_sp_t b);        /* AD/ADS, overflow-corrected */
agc_sp_t csm_dap_ad_sat(agc_sp_t a, agc_sp_t b);     /* AD; TS; INDEX A CS LIMITS */
agc_dp_t csm_dap_ddoubl(agc_dp_t a);                 /* DDOUBL                  */
agc_sp_t csm_dap_mp_hi(agc_sp_t a, agc_sp_t b);      /* MP, upper word (A)      */
agc_sp_t csm_dap_mp_lo(agc_sp_t a, agc_sp_t b);      /* MP, lower word (L)      */
agc_sp_t csm_dap_dv(agc_dp_t num, agc_sp_t den);     /* DV, limited to +-POSMAX */
agc_dp_t csm_dap_dpadd(agc_dp_t kmpac, agc_dp_t al); /* DPADD (mod 360 deg) */

/* ---- DP erasables as their stored words (agc_dp_words_t; mysubs.c) ---- */
agc_dp_words_t csm_dap_words_neg(agc_dp_words_t p);                           /* DCS          */
agc_dp_words_t csm_dap_words_mp(agc_sp_t a, agc_sp_t b);                      /* MP -> A,L  */
agc_sp_t       csm_dap_words_das_carry(agc_dp_words_t *k, agc_dp_words_t al); /* DAS, A kept */
void           csm_dap_words_das(agc_dp_words_t *k, agc_dp_words_t al);       /* DAS, A discarded */
agc_dp_words_t csm_dap_words_ddoubl(agc_dp_words_t p);                        /* DDOUBL    */
agc_dp_words_t csm_dap_words_smallmp(agc_dp_words_t in, agc_sp_t n);
uint16_t       csm_dap_cdu(int axis);                                         /* CDUX, CDUY, CDUZ raw    */
agc_status_t   csm_dap_set_t5(agc_task_id_t task, uint16_t time5_word);
agc_status_t   csm_dap_set_time5(uint16_t time5_word);
bool           csm_dap_engine_on(void);                                       /* channel 11 bit 13       */
agc_status_t   csm_dap_needler(void);                                         /* NEEDLER on AK..AK2      */

/* ---- RCS DAP (t5rupt.c, filter.c, manual.c, attitude.c) ---- */
agc_status_t csm_rcs_freshdap(bool fresh);         /* FRESHDAP (true) / REDAP     */
agc_status_t csm_rcs_reinit(void);                 /* REINIT ("TILT")             */
agc_status_t csm_rcs_rate_filter_phase(void);      /* SETT5 -0 branch .. RESUME   */
agc_status_t csm_rcs_kmatrix(void);                /* KMATRIX .. DACNDLS .. RESUME*/
agc_status_t csm_rcs_ratefilt(void);               /* DRHOLOOP, ADOTLOOP          */
agc_status_t csm_rcs_attitude_control_phase(void); /* T5PHASE2 .. T6PROG          */
agc_status_t csm_rcs_manual(void);                 /* MANUAL ROTATION COMMANDS    */
agc_status_t csm_rcs_ahfnorot(void);               /* AHFNOROT                    */
agc_status_t csm_rcs_jets(void);                   /* JETS .. JTIME .. T6PROG     */
agc_status_t csm_rcs_t6prog_manual(void);          /* T6PROGM: zero errors, T6PROG*/
bool         csm_rcs_rate_init_pending(void);      /* RCSFLAGS BIT14 (NORATE)     */

/* ---- TVC DAP (tvc_*.c) -------------------------------------------------- */
agc_status_t csm_tvc_mrclean(void);                   /* MRCLEAN .. ENDMRC           */
agc_status_t csm_tvc_init_complete(void);             /* TVCINIT4 .. ENDTVCIN        */
agc_status_t csm_tvc_swichovr_body(void);             /* SWICHOVR +5 .. TC RTRNLOC   */
agc_status_t csm_tvc_gains(bool full);                /* S40.15 (full) or S40.15 +7  */
agc_status_t csm_tvc_pcopy(bool restart);             /* PCOPY (restart: PCOPY +1)   */
agc_status_t csm_tvc_ycopy(bool restart);             /* YCOPY (restart: YCOPY +1)   */
agc_status_t csm_tvc_exec_restart(agc_sp_t tvcexphs); /* EXRSTRT targets    */
agc_status_t csm_tvc_tstinit(void);                   /* TSTINIT                     */
agc_status_t csm_tvc_hack(void);                      /* HACK (entry from the DAPs)  */
#endif                                                /* CSM_DAP_INTERNAL_H */
