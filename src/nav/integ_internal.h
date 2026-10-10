/*
 * integ_internal.h -- private interfaces between the nav-integ translation
 * units (integration_init.c, integration_jobs.c, orbital_integration.c,
 * integration_csm.c, integration_lm.c). Not part of include/agc.
 */
#ifndef INTEG_INTERNAL_H
#define INTEG_INTERNAL_H

#include <stdbool.h>
#include "agc/agc_integ.h"

/* Where the integration goes next: the AGC labels it jumps to. The driver
 * agc_integ_run() is the one switch over these (no goto, no recursion).  */
typedef enum agc_integ_next {
    INTEG_NEXT_TESTLOOP = 0,  /* TESTLOOP                                */
    INTEG_NEXT_TIMESTEP,      /* TIMESTEP                                */
    INTEG_NEXT_APCHK,         /* A-PCHK  (then RECTOUT, INTEXIT)          */
    INTEG_NEXT_RECTOUT,       /* RECTOUT (then INTEXIT)                   */
    INTEG_NEXT_INTEXIT,       /* INTEXIT without output                   */
    INTEG_NEXT_CKMID2,        /* CKMID2  (MIDAVFLG set at ENDSTATE)       */
    INTEG_NEXT_WMATRIX,       /* ENDSTATE with DIM0FLAG: ATOP, AMOVED     */
    INTEG_NEXT_RELOAD,        /* RELOADSV -> INTEGRV2 -> ALOADED          */
    INTEG_NEXT_RVCON,         /* RVCON (conic: ALOADED with INTYPFLG)      */
    INTEG_NEXT_ABORT,         /* INT-ABRT (POODOO 00430 already issued)   */
    INTEG_NEXT_FAIL           /* a called routine failed (bad argument)   */
} agc_integ_next_t;

/* Values used across files. */
#define INTEG_PBODY_EARTH 0
#define INTEG_PBODY_MOON  2
#define INTEG_REINTBIT    ((uint16_t)0x0040)  /* REINTBIT = BIT7 of RASFLAG */
#define INTEG_RASFLAG_WORD 10                 /* RASFLAG = FLAGWRD10       */
/* TIMEDELT 2DEC 1250 (CM, oct 00000 02342) / 2DEC 2000 (LM, 00000 03720),
 * cs scale 2^28: computation time allowance for one MIDTOAV time step.   */
#define INTEG_TIMEDELT_CSM ((agc_dp_t)1250)
#define INTEG_TIMEDELT_LM  ((agc_dp_t)2000)

/* ---- integration_init.c --------------------------------------------- */
agc_status_t     agc_integ_run(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                               agc_integ_next_t first);     /* the driver */
agc_status_t     agc_integ_integrv1(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_dp_t         agc_integ_loadtime(void);                  /* LOADTIME  */
bool             agc_integ_valid_ctx(const agc_integ_ctx_t *ctx);
/* SET / CLEAR a flag; the status is asserted (flag ids are constants).   */
void             agc_integ_flag(agc_flag_id_t f, bool on);

/* ---- integration_jobs.c --------------------------------------------- */
agc_integ_next_t agc_integ_ckmid2(agc_interp_t *ip, agc_integ_ctx_t *ctx);

/* ---- orbital_integration.c ------------------------------------------ */
agc_integ_next_t agc_integ_timestep(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_integ_next_t agc_integ_wcolumns(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_status_t     agc_integ_kepprep(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_status_t     agc_integ_chkswtch(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                    agc_dp_t *test);
agc_status_t     agc_integ_origchng(agc_interp_t *ip, agc_integ_ctx_t *ctx);
agc_dp_t         agc_integ_mu(int x2);          /* MUEARTH - X2 (0,2,4) */
agc_vec_t        agc_integ_vshift(agc_interp_t *ip, agc_vec_t v, int n);
agc_dp_t         agc_integ_dshift(agc_interp_t *ip, agc_dp_t a, int n);
agc_status_t     agc_integ_setmoon(agc_integ_ctx_t *ctx, bool moon);

/* ---- integration_csm.c (CM-only flags) ------------------------------ */
agc_status_t     agc_integ_csm_clear_v96onflg(void);
bool             agc_integ_csm_amoonflg(void);
agc_status_t     agc_integ_csm_set_amoonflg(bool on);

#endif /* INTEG_INTERNAL_H */
