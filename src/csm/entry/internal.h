/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interface between the files of the CM entry guidance
 *   (src/csm/entry/entry_*.c): the step of the per-pass chain, the pass
 *   state the steps share, and the phase step functions the sequencer
 *   calls.
 *
 * AGC SOURCE:
 *   Comanche055 REENTRY_CONTROL.agc pp. 844-881
 *
 * REQUIRES:
 *   the agc/ public headers included below
 *
 * GLOBAL VARIABLES:
 *   Name          Type      Scale          Description
 *   ------------  --------  -------------  -------------------------
 *   none (the entry erasables are reached through agc_entry_state())
 **********************************************************************/
#ifndef CSM_ENTRY_INTERNAL_H
#define CSM_ENTRY_INTERNAL_H

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"

/* The labelled sections of REENTRY_CONTROL, in the order the sequencer
 * dispatches them by phase (entry_sequencer, entry_guidance.c).         */
typedef enum entry_step
{
    ST_INITROLL = 0,
    ST_D0EQ,
    ST_INROLOUT,
    ST_KATEST,
    ST_HUNT_TRIGGER,
    ST_HUNTEST,
    ST_RANGER,
    ST_GETDLEWD,
    ST_SIDETRAK,
    ST_GOTOUPSY,
    ST_UPCONTRL,
    ST_UPCONTROL_DRAG_TEST,
    ST_UPCONTROL_REFERENCE,
    ST_UPCONTROL_Q7MIN_TEST,
    ST_UPCONTROL_LD,
    ST_NEGTESTS,
    ST_DCONSTD,
    ST_BECONSTD,
    ST_CONSTD,
    ST_CONSTANT_DRAG_LD,
    ST_DOWNCNTL,
    ST_KEP,
    ST_KEPLER_TEST,
    ST_PREFINAL,
    ST_FINAL_PREDICT,
    ST_HAVDNRNG,
    ST_GONEGLAD,
    ST_GOMAXLD,
    ST_GLIMITER,
    ST_GOPOSLAD,
    ST_LIMITLD,
    ST_LATERAL_SWITCH,
    ST_ROLL_COMMAND,
    ST_LATERAL_MIN_LD,
    ST_STEEROFF,
    ST_FINAL_PHASE_LATLONG,
    ST_ENDEXIT,  /* terminal: ENDEXIT -> OVERNOUT display -> SERVNOUT   */
    ST_SERVNOUT, /* terminal: SERVNOUT without display                  */
    ST_ERROR     /* terminal: a called routine failed                  */
} entry_step_t;

typedef struct entry_pass
{
    agc_interp_t      *ip;        /* entry interpreter state (OVFIND)      */
    agc_entry_state_t *es;        /* entry erasables                       */
    agc_dp_t           mpac;      /* "in MPAC" at a step entry, step scale */
    agc_dp_t           pd_y;      /* push-down: lateral Y (L350), B0       */
    agc_dp_words_t     pd_ld;     /* push-down: the K2D term (CONSTD1) as the DAD leaves it, B0 */
    agc_dp_words_t     dref;      /* MPAC at CONSTD1: DREF as stored words, /805 B0 */
    int                hunt_iter; /* HUNTEST iterations this pass          */
    agc_status_t       status;    /* first failure of a called routine     */
} entry_pass_t;

/* One phase's steps: run step s, return the step the AGC branches to.    */
entry_step_t entry_initial_roll_step(entry_pass_t *p, entry_step_t s);
entry_step_t entry_huntest_step(entry_pass_t *p, entry_step_t s);
entry_step_t entry_upcontrol_step(entry_pass_t *p, entry_step_t s);
entry_step_t entry_ballistic_step(entry_pass_t *p, entry_step_t s);
entry_step_t entry_final_phase_step(entry_pass_t *p, entry_step_t s);

/* TARGETNG (REENTRY_CONTROL p.848): every pass, before the phase step.  */
void agc_entry_targeting(void);

/* ---- moved from the public headers (used only in csm/entry/) ---- */

/* agc/csm/entry.h */
agc_status_t agc_entry_desired_gimbal_angles(void);                             /* S62.3 CPHI for N22  */
agc_status_t agc_entry_final_phase_latlong(void);                               /* P67.2 LAT LONG ALT  */
agc_status_t agc_job_cm_dapon(agc_job_ctx_t *ctx);                              /* CM/DAPON        */
agc_status_t agc_entry_dap_ic(agc_job_ctx_t *ctx);                              /* CM/DAPIC        */
void         entry_earrot(agc_interp_t *ip, agc_entry_state_t *es, bool first); /* EARROT1/2 */
agc_status_t entry_endexit(agc_job_ctx_t *ctx, agc_entry_state_t *es);          /* OVERNOUT */
agc_status_t entry_initial_target(agc_interp_t *ip, agc_entry_state_t *es);     /* STARTEN1 */
agc_status_t entry_post_0_05g(agc_entry_state_t *es);
agc_status_t entry_upcontrol(agc_entry_state_t *es);
agc_status_t entry_ballistic(agc_entry_state_t *es);
agc_status_t entry_final_phase(agc_entry_state_t *es);
void         entry_disable_restart_group(void);
agc_sp_t     entry_sp_sat(int32_t sum);                                         /* AD whose result is kept (overflow not expected) */

/* agc/csm/entry_lexicon.h */
/* VREFER (REENTRY_CONTROL pp. 876-878): table t, point k -> SP value.     */
agc_sp_t agc_entry_vrefer(int table, int k);

#endif /* CSM_ENTRY_INTERNAL_H */
