/**********************************************************************
 * FILE NAME: targeting.c
 *
 * PURPOSE:
 *   Rendezvous targeting against the real AGC code (yaAGC oracle):
 *   INITVEL (conic, earth and moon, and one precision iteration through
 *   INTEGRVS), CDHMVR (the CDH maneuver of P32/P33) and S34/35.2 (the
 *   TPI required velocity: INTINT, ACTCENT, INITVEL, LOMAT -> DELVLVC).
 *   Tolerances (LSB of the AGC word): see docs/overview/oracle-cases.md.
 *
 * AGC SOURCE:
 *   Comanche055 P34-35_P74-75.agc, P32-P33_P72-P73.agc. Inputs and
 *   expected words: tools/oracle/cases/targeting.spec / .txt, generated
 *   by tools/oracle/cases/gen/gen_targeting.py.
 *
 * EXTERNAL VARIABLES:
 *   oracle_spec (oracle_txt.h): the loaded spec text
 *
 * EXTERNAL REFERENCES:
 *   Name                             Description
 *   -------------------------------  -----------------------------------
 *   agc_rendz_initvel                INITVEL
 *   agc_rendz_cdhmvr                 CDHMVR
 *   agc_rendz_tpi_required_velocity  S34/35.2
 *   oracle_load / oracle_cmp         case files and comparison
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any comparison fails (oracle_finish). Missing
 *   case files skip the test.
 **********************************************************************/
#include <stdio.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/rendezvous.h"
#include "oracle_txt.h"

/* Prototypes: main() comes first (STYLE 4.6). */
static agc_rendz_t *fresh(const char *t);
static void         initvel(const char *t, long int tol);
static void         cdh(const char *t, long int tol);
static void         tpi(const char *t, long int tol);

/**********************************************************************
 * AGC: none (test driver) -- the CSM case files
 *
 * FUNCTION NAME: main
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- 0 when all comparisons pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    if (!oracle_load("targeting"))
    {
        (void) printf("oracle_targeting: SKIP (case files not found)\n");

        return (0);
    }

    initvel("initvel_earth", 0);
    initvel("initvel_earth_far", 0);

    /* Tolerances = the observed residual (2026-10-12, word-exact conics
     * and integration): VACT3.x / DELVEET2.x (CDH) and DELVLVC.x (TPI)
     * keep 2-4 LSB from outside the conics, _prec 1 LSB of VTPRIME. */
    initvel("initvel_moon", 0);
    initvel("initvel_earth_prec", 1);
    cdh("cdh_earth", 2);
    cdh("cdh_earth_ell", 4);
    tpi("tpi_earth", 2);
    tpi("tpi_earth_b", 0);

    return (oracle_finish("oracle_targeting"));
}

/**********************************************************************
 * The AGC_ASSERT hook of the test: print and fail the assertion.
 * AGC: none (test support)
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   file      const char *  I    source file of the assertion
 *   line      int           I    its line
 *   expr      const char *  I    the expression text
 *
 * RETURNS: bool -- false (the assertion fails)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);

    return (false);
}

/**********************************************************************
 * A fresh CSM state for case t: executive, pad loads, integration and
 * rendezvous erasables reset; moon flags and SELECTMU from RTX1.
 * AGC: none (test support)
 *
 * FUNCTION NAME: fresh
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   t         const char *  I    case tag
 *
 * RETURNS: agc_rendz_t * -- the rendezvous erasables
 **********************************************************************/
static agc_rendz_t *fresh(const char *t)
{
    agc_navaux_padload_t pad  = {0};                            /* zero pad */
    const bool           moon = oracle_in(t, "RTX1", 0) == -10; /* moon */

    agc_state_init(AGC_VEHICLE_CSM);
    agc_exec_init();
    agc_waitlist_init();
    agc_navaux_set_padload(&pad);
    agc_integ_ctx_init(agc_integ_shared_ctx());
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    agc_rendz_reset(rz);
    agc_flag_put(AGC_FLAG_CMOONFLG, moon);
    agc_flag_put(AGC_FLAG_MOONFLAG, moon);
    agc_rendz_selectmu(rz, moon);

    return (rz);
}

/**********************************************************************
 * INITVEL of case t against the AGC words.
 * AGC: INITVEL (P34-35_P74-75.agc pp.487-491)
 *
 * FUNCTION NAME: initvel
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   t         const char *  I    case tag
 *   tol       long int      I    tolerance, LSB of the DP word
 **********************************************************************/
static void initvel(const char *t, long int tol)
{
    agc_interp_t ip;            /* interpreter state */
    agc_rendz_t *rz = fresh(t); /* rendezvous erasables */

    agc_interp_init(&ip);
    rz->rinit             = oracle_in_vec(t, "RINIT");
    rz->vinit             = oracle_in_vec(t, "VINIT");
    rz->rtarg             = oracle_in_vec(t, "RTARG");
    rz->dellt4            = oracle_in_dp(t, "DELLT4", 0);
    rz->intime            = oracle_in_dp(t, "INTIME", 0);
    const agc_status_t st = agc_rendz_initvel(&ip, rz, (agc_sp_t) oracle_in(t, "VAC1", 0), oracle_in_dp(t, "VAC1+2", 0), AGC_JOB_NONE); /* INITVEL */

    oracle_require_ok(t, "POODOO");
    (void) oracle_cmp(t, "status", st, AGC_STATUS_OK, 0);
    oracle_cmp_vec(t, "RTARG", rz->rtarg, oracle_out_vec(t, "RTARG"), tol);
    oracle_cmp_vec(t, "VIPRIME", rz->viprime, oracle_out_vec(t, "VIPRIME"), tol);
    oracle_cmp_vec(t, "VTPRIME", rz->vtprime, oracle_out_vec(t, "VTPRIME"), tol);
    oracle_cmp_vec(t, "DELVEET3", rz->delveet3, oracle_out_vec(t, "DELVEET3"), tol);
}

/**********************************************************************
 * CDHMVR of case t against the AGC words.
 * AGC: CDHMVR (P32-P33_P72-P73.agc pp.680-682)
 *
 * FUNCTION NAME: cdh
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   t         const char *  I    case tag
 *   tol       long int      I    tolerance, LSB of the DP word
 **********************************************************************/
static void cdh(const char *t, long int tol)
{
    agc_interp_t ip;            /* interpreter state */
    agc_rendz_t *rz = fresh(t); /* rendezvous erasables */

    agc_interp_init(&ip);
    rz->ract2  = oracle_in_vec(t, "RACT2");
    rz->vact2  = oracle_in_vec(t, "VACT2");
    rz->rpass2 = oracle_in_vec(t, "RPASS2");
    rz->vpass2 = oracle_in_vec(t, "VPASS2");
    rz->up1    = oracle_in_vec(t, "UP1");
    (void) agc_rendz_cdhmvr(&ip, rz);
    oracle_require_ok(t, "POODOO");

    /* F13 (DIFFALT 62 m) closed by the word-level WLOOP in nav-conic:
     * DIFFALT is exact.                                                */
    (void) oracle_cmp(t, "DIFFALT", rz->diffalt, oracle_out_dp(t, "DIFFALT"), 0);
    oracle_cmp_vec(t, "VACT3", rz->vact3, oracle_out_vec(t, "VACT3"), tol);
    oracle_cmp_vec(t, "DELVEET2", rz->delveet2, oracle_out_vec(t, "DELVEET2"), tol);
}

/**********************************************************************
 * S34/35.2 of case t against the AGC words.
 * AGC: S34/35.2 (P34-35_P74-75.agc pp.475-476)
 *
 * FUNCTION NAME: tpi
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   t         const char *  I    case tag
 *   tol       long int      I    tolerance, LSB of the DP word
 **********************************************************************/
static void tpi(const char *t, long int tol)
{
    agc_interp_t ip;            /* interpreter state */
    agc_rendz_t *rz = fresh(t); /* rendezvous erasables */

    agc_interp_init(&ip);
    rz->ract3             = oracle_in_vec(t, "RACT3");
    rz->vact3             = oracle_in_vec(t, "VACT3");
    rz->rpass3            = oracle_in_vec(t, "RPASS3");
    rz->vpass3            = oracle_in_vec(t, "VPASS3");
    rz->unrm              = oracle_in_vec(t, "UNRM");
    rz->intime            = oracle_in_dp(t, "INTIME", 0);
    rz->tpass4            = oracle_in_dp(t, "TPASS4", 0);
    rz->nn                = 0;
    const agc_status_t st = agc_rendz_tpi_required_velocity(&ip, rz, AGC_JOB_STATEINT_UPDATE); /* S34/35.2 */

    oracle_require_ok(t, "POODOO");
    (void) oracle_cmp(t, "status", st, AGC_STATUS_OK, 0);
    oracle_cmp_vec(t, "RTARG", rz->rtarg, oracle_out_vec(t, "RTARG"), tol);
    oracle_cmp_vec(t, "VPASS4", rz->vpass4, oracle_out_vec(t, "VPASS4"), tol);
    (void) oracle_cmp(t, "ACTCENT", rz->actcent, oracle_out_dp(t, "ACTCENT"), tol);
    (void) oracle_cmp(t, "DELLT4", rz->dellt4, oracle_out_dp(t, "DELLT4"), 0);
    oracle_cmp_vec(t, "VIPRIME", rz->viprime, oracle_out_vec(t, "VIPRIME"), tol);
    oracle_cmp_vec(t, "VTPRIME", rz->vtprime, oracle_out_vec(t, "VTPRIME"), tol);
    oracle_cmp_vec(t, "DELVEET3", rz->delveet3, oracle_out_vec(t, "DELVEET3"), tol);
    oracle_cmp_vec(t, "DELVLVC", rz->delvlvc, oracle_out_vec(t, "DELVLVC"), tol);
}
