/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private interfaces between the nav-integ translation units (init.c,
 *   jobs.c, orbital_integration.c, flags_csm.c, setitctr_lm.c): the
 *   driver's next-label enum and the routines the files share.  Not part
 *   of include/agc.
 *
 * AGC SOURCE:
 *   Comanche055 INTEGRATION_INITIALIZATION.agc pp. 1309-1333,
 *   ORBITAL_INTEGRATION.agc pp. 1334-1354 (Luminary099 pp. 1203-1248)
 *
 * REQUIRES:
 *   agc/nav/integration.h
 *
 * GLOBAL VARIABLES:
 *   none
 **********************************************************************/
#ifndef INTEG_INTERNAL_H
#define INTEG_INTERNAL_H

#include <stdbool.h>

#include "agc/nav/integration.h"

/* Where the integration goes next: the AGC labels it jumps to. The driver
 * agc_integ_run() is the one switch over these (no goto, no recursion).  */
typedef enum agc_integ_next
{
    INTEG_NEXT_TESTLOOP = 0,  /* TESTLOOP                                */
    INTEG_NEXT_TIMESTEP,      /* TIMESTEP                                */
    INTEG_NEXT_APCHK,         /* A-PCHK  (then RECTOUT, INTEXIT)          */
    INTEG_NEXT_RECTOUT,       /* RECTOUT (then INTEXIT)                   */
    INTEG_NEXT_INTEXIT,       /* INTEXIT without output                   */
    INTEG_NEXT_MIDTOAV_CHECK, /* CKMID2  (MIDAVFLG set at ENDSTATE)       */
    INTEG_NEXT_WMATRIX,       /* ENDSTATE with DIM0FLAG: ATOP, AMOVED     */
    INTEG_NEXT_RELOAD,        /* RELOADSV -> INTEGRV2 -> ALOADED          */
    INTEG_NEXT_RVCON,         /* RVCON (conic: ALOADED with INTYPFLG)      */
    INTEG_NEXT_ABORT,         /* INT-ABRT (POODOO 00430 already issued)   */
    INTEG_NEXT_FAIL           /* a called routine failed: ctx->fail       */
} agc_integ_next_t;

/* Values used across files. */
#define INTEG_PBODY_EARTH  0                   /* PBODY (X2): earth primary */
#define INTEG_PBODY_MOON   2                   /* PBODY (X2): moon primary  */
#define INTEG_REINTBIT     ((uint16_t) 0x0040) /* REINTBIT = BIT7 of RASFLAG */
#define INTEG_RASFLAG_WORD 10                  /* RASFLAG = FLAGWRD10       */

/* TIMEDELT 2DEC 1250 (CM, oct 00000 02342) / 2DEC 2000 (LM, 00000 03720),
 * cs scale 2^28: computation time allowance for one MIDTOAV time step.   */
#define INTEG_TIMEDELT_CSM ((agc_dp_t) 1250) /* CM TIMEDELT, cs 2^28 */
#define INTEG_TIMEDELT_LM  ((agc_dp_t) 2000) /* LM TIMEDELT, cs 2^28 */

/* ---- init.c --------------------------------------------- */
agc_status_t     agc_integ_run(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_integ_next_t first); /* the driver */
agc_status_t     agc_integ_integrate_permanent(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_dp_t         agc_integ_loadtime(void);                                                      /* LOADTIME  */
agc_integ_next_t agc_integ_fail(agc_integ_ctx_t *ctx, agc_status_t st);                         /* keep st, INTEG_NEXT_FAIL */

/* ---- jobs.c --------------------------------------------- */
agc_integ_next_t agc_integ_midtoav_check(agc_interp_t *ip, agc_integ_ctx_t *ctx);

/* ---- orbital_integration.c ------------------------------------------ */
agc_integ_next_t agc_integ_timestep(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_integ_next_t agc_integ_wcolumns(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_status_t     agc_integ_kepprep(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_status_t     agc_integ_chkswtch(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t *test);
agc_status_t     agc_integ_origchng(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_dp_t         agc_integ_mu(int x2); /* MUEARTH - X2 (0,2,4) */
void             agc_integ_setmoon(agc_integ_ctx_t *ctx, bool moon);

/* ---- flags_csm.c (CM-only flags) ------------------------------ */
void agc_integ_csm_resume_idle_integration(void);
bool agc_integ_csm_amoonflg(void);
void agc_integ_csm_set_amoonflg(bool on);

/* FINDVAC priority PRIO5 (octal 05000 >> 9).                              */
#define AGC_INTEG_PRIO5 ((agc_prio_t) 5) /* STATINT1 priority, PRIO5 */

#endif /* INTEG_INTERNAL_H */
