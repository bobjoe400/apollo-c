/**********************************************************************
 * FILE NAME: guidance.c
 *
 * PURPOSE:
 *   The guidance lane against the real AGC code (yaAGC oracle): CALCRVG
 *   + CALCGRAV (one average-G step, earth and moon), the S40.1 external-
 *   delta-V branch (CALCTHET, MIDGIM), S40.2,3 (SPS and RCS) and DELTIME
 *   (the conic time-of-flight kernel used by the targeting conics; its C
 *   is nav-conic's agc_conic_deltime_, src/nav/conic/internal.h).
 *
 * AGC SOURCE:
 *   Comanche055 SERVICER207.agc pp. 835-836, P40-P47.agc pp. 709-716,
 *   CONIC_SUBROUTINES.agc (DELTIME); inputs and expected words in
 *   tools/oracle/cases/guidance.spec / .txt (gen_guidance.py)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   REFSMMAT      agc_mat_t  O    2^1          case input
 *   VGTIG, TIG    -          O    as stored    case inputs (S40.1)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   oracle_*              case reader and comparisons (oracle_txt.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when a check fails; SKIP when the case files are missing.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Host test; one case at a time from a fresh state.
 *
 * NOTES:
 *   Tolerances are in LSB of the AGC word (0 = bit exact); the residuals
 *   are explained at the calls in main and in docs/modules/oracle-cases.md.
 *
 * ALGORITHM:
 *   Load the case, run the C routine, compare word by word.
 **********************************************************************/
#include <stdio.h>

#include "../../src/nav/conic/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/guidance/guidance.h"
#include "agc/programs/rendezvous.h"
#include "oracle_txt.h"

static agc_mat_t mat_in(const char *t, const char *name);
static void      calcrvg(const char *t, long int tol);
static void      s40_1(const char *t, long int tol);
static void      s40_2(const char *t, long int tol);
static void      deltime(const char *t, long int tol);

/**********************************************************************
 * Run every guidance oracle case with its tolerance.
 * AGC: none (test driver).
 *
 * FUNCTION NAME: main
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- 0 when every check passes, 1 otherwise
 **********************************************************************/
int main(void)
{
    if (!oracle_load("guidance"))
    {
        (void) printf("oracle_guidance: SKIP (case files not found)\n");

        return (0);
    }

    calcrvg("calcrvg_coast", 0);
    calcrvg("calcrvg_burn", 0);
    calcrvg("calcrvg_tli", 0);
    calcrvg("calcrvg_moon", 0);
    s40_1("s40_1_posigrade", 0);
    s40_1("s40_1_plane", 0);
    s40_1("s40_1_large", 0);

    /* S40.2,3: the bell axes are word-exact (SCAXIS 0 LSB); the VXM of the
     * stored, sign-disagreeing VDEF words into the UT/Y/Z frame needs a
     * DOT on stored words (requested from core) -- 2 LSB. The radial case
     * also UNITs a short UT x UNIT(R) (UNIT at B27, core item) -- 10 LSB. */
    s40_2("s40_2_sps", 0);
    s40_2("s40_2_sps_b", 0);
    s40_2("s40_2_rcs", 0);
    s40_2("s40_2_radial", 10);
    deltime("deltime_ell", 0);
    deltime("deltime_hyp", 0);
    deltime("deltime_small", 0);

    return (oracle_finish("oracle_guidance"));
}

/**********************************************************************
 * The assertion hook of the library for this test: print the report.
 * AGC: none (test harness).
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   file      const char *  I    source file of the failed assertion
 *   line      int           I    line of the failed assertion
 *   expr      const char *  I    text of the failed expression
 *
 * RETURNS: bool -- false (the assertion stays failed)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assert %s:%d %s\n", file, line, expr);

    return (false);
}

/**********************************************************************
 * Read a 3x3 matrix input of an oracle case (rows at NAME, +6, +12).
 * AGC: none (test harness).
 *
 * FUNCTION NAME: mat_in
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   t         const char *  I    oracle case name
 *   name      const char *  I    name of the matrix in the case
 *
 * RETURNS: agc_mat_t -- the matrix, scale as stored
 **********************************************************************/
static agc_mat_t mat_in(const char *t, const char *name)
{
    agc_mat_t m; /* rotation matrix, half-unit rows (2^1) */

    for (int r = 0; r < 3; r++)
    {
        m.row[r] = oracle_in_vec_at(t, name, r);
    }

    return (m);
}

/**********************************************************************
 * CALCRVG + CALCGRAV, one average-G step (SERVICER207.agc p.835-836)
 * from the case inputs; RN1, VN1, DELVREF, UNITR, GDT1/2 and GOBL1/2
 * compared.
 *
 * FUNCTION NAME: calcrvg
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   t         const char *  I    oracle case name
 *   tol       long int      I    tolerance, LSB of the AGC word
 **********************************************************************/
static void calcrvg(const char *t, long int tol)
{
    agc_interp_t       ip;                                  /* interpreter state */
    agc_avg_g_t        a    = {0};
    agc_state_vector_t sv;                                  /* state vector (RN, VN, PIPTIME) */
    const agc_mat_t    ref  = mat_in(t, "REFSMMAT");        /* REFSMMAT, half-unit rows (2^1) */
    const bool         moon = oracle_in(t, "RTX2", 0) != 0; /* moon-centred case (RTX2 = 2) */

    agc_state_init(AGC_VEHICLE_CSM);
    agc_interp_init(&ip);
    sv.r    = oracle_in_vec(t, "RN");
    sv.v    = oracle_in_vec(t, "VN");
    sv.t    = 0;
    sv.moon = moon;
    a.gdt_2 = oracle_in_vec(t, "GDT/2");
    a.unitw = oracle_in_vec(t, "UNITW");
    (void) agc_guid_calcrvg(&ip, AGC_VEHICLE_CSM, moon, &ref, oracle_in_vec(t, "DELV"), &sv, &a);
    oracle_require_ok(t, "POODOO");
    oracle_cmp_vec(t, "DELVREF", a.delvref, oracle_out_vec(t, "DELVREF"), tol);
    oracle_cmp_vec(t, "RN1", a.rn1, oracle_out_vec(t, "RN1"), tol);
    oracle_cmp_vec(t, "UNITR", a.unitr, oracle_out_vec(t, "UNITR"), tol);

    if (moon)
    {
        oracle_cmp_vec(t, "VN1", a.vn1, oracle_out_vec(t, "VN1"), tol);
        oracle_cmp_vec(t, "GDT1/2", a.gdt1_2, oracle_out_vec(t, "GDT1/2"), tol);

        return;
    }

    /* F7 (fixed in review): the Comanche 20J / 2J words are B1.         */
    oracle_cmp_vec(t, "VN1", a.vn1, oracle_out_vec(t, "VN1"), tol);
    oracle_cmp_vec(t, "GDT1/2", a.gdt1_2, oracle_out_vec(t, "GDT1/2"), tol);
    oracle_cmp_vec(t, "GOBL1/2", a.gobl1_2, oracle_out_vec(t, "GOBL1/2"), tol);
}

/**********************************************************************
 * S40.1 external delta-V branch (P40-P47.agc p.709-710: CALCTHET,
 * MIDGIM): DELVSAB, UT, VGTIG, RINIT, VINIT, DELVLVC and AVFLAG
 * compared.
 *
 * FUNCTION NAME: s40_1
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   t         const char *  I    oracle case name
 *   tol       long int      I    tolerance, LSB of the AGC word
 **********************************************************************/
static void s40_1(const char *t, long int tol)
{
    agc_interp_t ip; /* interpreter state */

    agc_state_init(AGC_VEHICLE_CSM);
    agc_interp_init(&ip);
    (void) agc_guid_init(AGC_VEHICLE_CSM);
    agc_rendz_t *rz = agc_rendz_erasable();    /* targeting erasables */
    agc_burn_t  *b  = &agc_guid_state()->burn; /* burn erasables */

    rz->rtig    = oracle_in_vec(t, "RTIG");
    rz->vtig    = oracle_in_vec(t, "VTIG");
    rz->delvsin = oracle_in_vec(t, "DELVSIN");
    b->weight_g = oracle_in_dp(t, "WEIGHT/G", 0);
    b->f        = oracle_in_dp(t, "F", 0);
    agc_flag_put(AGC_FLAG_XDELVFLG, true);
    (void) agc_guid_initial_velocity_to_gain(&ip, AGC_VEHICLE_CSM);
    oracle_require_ok(t, "POODOO");
    (void) oracle_cmp(t, "DELVSAB", rz->delvsab, oracle_out_dp(t, "DELVSAB"), tol);
    oracle_cmp_vec(t, "UT", b->ut, oracle_out_vec(t, "UT"), tol);
    oracle_cmp_vec(t, "VGTIG", agc_erase_vgtig(), oracle_out_vec(t, "VGTIG"), tol);
    oracle_cmp_vec(t, "RINIT", rz->rinit, oracle_out_vec(t, "RINIT"), 0);
    oracle_cmp_vec(t, "VINIT", rz->vinit, oracle_out_vec(t, "VINIT"), 0);
    oracle_cmp_vec(t, "DELVLVC", rz->delvlvc, oracle_out_vec(t, "DELVLVC"), tol);
    (void) oracle_cmp(t, "AVFLAG", agc_flag_test(AGC_FLAG_AVFLAG), (oracle_out_raw(t, "FLAGWRD2") & 020) != 0, 0);
}

/**********************************************************************
 * S40.2,3 (P40-P47.agc p.714-716) for the SPS or RCS case: POINTVSM,
 * SCAXIS and XSCREF..ZSCREF compared.
 *
 * FUNCTION NAME: s40_2
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   t         const char *  I    oracle case name
 *   tol       long int      I    tolerance, LSB of the AGC word
 **********************************************************************/
static void s40_2(const char *t, long int tol)
{
    agc_interp_t ip; /* interpreter state */

    agc_state_init(AGC_VEHICLE_CSM);
    agc_interp_init(&ip);
    (void) agc_guid_init(AGC_VEHICLE_CSM);
    agc_rendz_t *rz = agc_rendz_erasable();    /* targeting erasables */
    agc_burn_t  *b  = &agc_guid_state()->burn; /* burn erasables */

    b->ut          = oracle_in_vec(t, "UT");
    rz->rtig       = oracle_in_vec(t, "RTIG");
    rz->vtig       = oracle_in_vec(t, "VTIG");
    agc_erase_set_refsmmat(mat_in(t, "REFSMMAT"));
    const bool rcs = (oracle_in(t, "FLAGWRD1", 0) & 02000) != 0; /* ENG2FLAG */

    (void) agc_guid_preferred_thrust_attitude_csm(&ip, rcs, (agc_sp_t) oracle_in(t, "PACTOFF", 0), (agc_sp_t) oracle_in(t, "YACTOFF", 0));
    oracle_require_ok(t, "POODOO");
    oracle_cmp_vec(t, "POINTVSM", b->pointvsm, oracle_out_vec(t, "POINTVSM"), tol);
    oracle_cmp_vec(t, "SCAXIS", b->scaxis, oracle_out_vec(t, "SCAXIS"), tol);
    oracle_cmp_vec(t, "XSCREF", b->xscref, oracle_out_vec(t, "XSCREF"), tol);
    oracle_cmp_vec(t, "YSCREF", b->yscref, oracle_out_vec(t, "YSCREF"), tol);
    oracle_cmp_vec(t, "ZSCREF", b->zscref, oracle_out_vec(t, "ZSCREF"), tol);
}

/**********************************************************************
 * DELTIME (CONIC_SUBROUTINES.agc, the targeting conics' time-of-flight
 * kernel): S(XI), XSQC(XI) and T compared as stored words.
 *
 * FUNCTION NAME: deltime
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------------------------
 *   t         const char *  I    oracle case name
 *   tol       long int      I    tolerance, LSB of the AGC word
 **********************************************************************/
static void deltime(const char *t, long int tol)
{
    agc_interp_t    ip; /* interpreter state */
    agc_conic_ctx_t c;  /* DELTIME context */

    agc_interp_init(&ip);
    (void) agc_conic_ctx_init_(&c, AGC_CONIC_EARTH);
    c.xsq         = agc_dp_to_words(oracle_in_dp(t, "VAC1", 0));
    c.xsq_n1      = -oracle_in(t, "VAC1+38", 0);
    c.inv_root_mu = agc_dp_to_words(oracle_in_dp(t, "VAC1+18", 0));
    c.x           = agc_dp_to_words(oracle_in_dp(t, "VAC1+20", 0));
    c.r1          = agc_dp_to_words(oracle_in_dp(t, "VAC1+32", 0));
    c.kepc1       = agc_dp_to_words(oracle_in_dp(t, "VAC1+34", 0));
    c.kepc2       = agc_dp_to_words(oracle_in_dp(t, "VAC1+36", 0));
    c.xi          = agc_dp_to_words(oracle_in_dp(t, "VAC1+24", 0));
    (void) agc_conic_deltime_(&ip, &c);
    oracle_require_ok(t, "POODOO");
    (void) oracle_cmp(t, "S(XI)", agc_dp_from_words(c.s_xi), oracle_out_dp(t, "VAC1+26"), tol);
    (void) oracle_cmp(t, "XSQC(XI)", agc_dp_from_words(c.xsqc_xi), oracle_out_dp(t, "VAC1+28"), tol);
    (void) oracle_cmp(t, "T", agc_dp_from_words(c.t), oracle_out_dp(t, "VAC1+30"), tol);
}
