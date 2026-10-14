/**********************************************************************
 * FILE NAME: orbital_navigation.c
 *
 * PURPOSE:
 *   P22 ORBITAL NAVIGATION PROGRAM, CSM: PROG22, PROG22A/B/C, CALLR52,
 *   DOV5N71, P22OVER, the landmark code check (S22N7071, S22DSPPA,
 *   S22DSPP, S22ABDE, DE-GR-50) and the landmark coordinates (S22LSITE,
 *   OBTAINLL, P22SUBRB, N89ERRX). The mark processing S22.1 is in
 *   csm/navigation/landmark_navigation.c. The crew identifies the
 *   landmark on noun 70 / 71 as ABCDE (octal): A = 1 known, 2 unknown;
 *   B = the offset point mark (0-5); DE = 00 coordinates from noun 89,
 *   01 the landing site RLS, 50-57 advanced orbit (R52 ADVORB). The
 *   orbit's +MGA is shown first (noun 45).
 *
 * AGC SOURCE:
 *   Comanche055 P20-P25.agc pp. 565-569; CSM only
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  ----------------------------
 *   LANDMARK   agc_sp_t   I/O  -          ABCDE code (nouns 70, 71)
 *   +MGA       agc_dp_t   O    rev 2^0    noun 45
 *   REFSMMAT   agc_mat_t  I    2^1        reference to SM
 *   RLS        agc_vec_t  I    m 2^27     landing site (moon fixed)
 *   LANDLAT LANDLONG  agc_dp_t I  rev     noun 89 coordinates
 *   MARKINDX   agc_sp_t   O    -          marks wanted (5)
 *   RNDVZFLG RENDWFLG P22MKFLG ERADFLAG LUNAFLAG KNOWNFLG TARG1FLG
 *   TARG2FLG CMOONFLG  flags  I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                                Description
 *   ----------------------------------  --------------------------------
 *   rz_integ                            CSMCONIC
 *   agc_planet_rp_to_r                  RP-TO-R
 *   agc_latlong_from_r                  LAT-LONG
 *   agc_csm_auto_optics_positioning     R52 (and ADVORB)
 *   rz_landmark_marks_start             S22.1
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Operator error light for a bad code or coordinates; the R02BOTH
 *   alarms. Display, integration and planetary statuses are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of P22 (CSM).
 *
 * NOTES:
 *   Scales: angles revolutions; RLS moon-fixed m 2^27; ALPHAV m 2^29.
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "internal.h"

#define ON_PRIO7       07000u  /* B of ABCDE (offset)        */
#define ON_PRIO5       05000u  /* offset 5                   */
#define ON_A_MASK      070000u /* 13,14,15: A of ABCDE       */
#define ON_A_KNOWN     010000u /* A = 1 */
#define ON_A_UNKNOWN   020000u /* A = 2 */
#define ON_DE_MASK     000077u /* CS HIGH9                   */
#define ON_D_MASK      000070u /* BIT5 + OCT50               */
#define ON_D_ADV_ORBIT 000050u /* DE = 50-57                 */
#define ON_BLANK_R1R2  3U      /* R1 R2 blanked */
#define ON_BLANK_R1R3  5U      /* R1 R3 blanked */

/* P22 working cells: 8KK (the VN of S22N7071), 8KK +1 (noun 70), the
 * S22TOFF return of S22N7071 / P22SUBRB.                                 */
typedef struct on_state
{
    uint8_t      noun; /* 70 or 71                              */
    agc_job_id_t ret;  /* where the check returns               */
} on_state_t;

static on_state_t on = {70U, AGC_JOB_GOTOPOOH};

/**********************************************************************
 * PROG22: rendezvous flag off; IMU check; +MGA = ACOS |UNIT(V x R) . Y
 * (REFSMMAT)| of the CSM now (orbit plane to stable member); W invalid;
 * V06N45.
 * AGC: PROG22 (P20-P25.agc p.565)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_csm(agc_job_ctx_t *ctx)
{
    rz_flag_put(AGC_FLAG_RNDVZFLG, false);
    agc_status_t st = agc_imu_status_check(); /* R02BOTH  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_integ_out_t o; /* integration result */

    st = rz_integ(&ip, RZ_CSMCONIC, agc_rtb_loadtime(), AGC_JOB_ORBITAL_NAVIGATION, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, 2^1 */
    agc_vec_t n;                               /* 2^1 */

    n          = agc_vec_unit(&ip, agc_vec_cross(&ip, o.vatt, o.ratt), NULL, NULL);
    agc_dp_t c = agc_dp_abs(agc_vec_dot(&ip, n, refsmmat.row[1])); /* 2^2 */

    agc_rendz_erasable()->plus_mga = agc_dp_acos(&ip, agc_dp_shl(&ip, c, 1));
    rz_flag_put(AGC_FLAG_RENDWFLG, false);

    return (agc_job_orbital_navigation_orbit_display(ctx));
}

/**********************************************************************
 * V06N45 (GOFLASHR, R1 R2 blanked): PROCEED: PROG22A.
 * AGC: PROG22 V06N45 (P20-P25.agc p.565)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_orbit_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_orbit_display(agc_job_ctx_t *ctx)
{
    agc_status_t st = rz_flash(ctx, AGC_DSKY_GOFLASHR, 6U, 45U, AGC_JOB_GOTOPOOH, AGC_JOB_ORBITAL_NAVIGATION_LANDMARK, AGC_JOB_ORBITAL_NAVIGATION_ORBIT_DISPLAY); /* V06N45 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(ON_BLANK_R1R2));
}

/**********************************************************************
 * PROG22A (point A): offset digit cleared; ERADFLAG; in the moon's sphere
 * the landmark code on noun 70; in earth orbit a known landmark with its
 * coordinates on noun 89; then CALLR52.
 * AGC: PROG22A, PROG22B (P20-P25.agc p.565)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_landmark
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_landmark(agc_job_ctx_t *ctx)
{
    agc_sp_t lm = agc_erase_landmark(); /* ABCDE, unscaled octal */

    agc_erase_set_landmark((agc_sp_t) rz_word_value((uint16_t) (rz_word_of(lm) & ~ON_PRIO7)));
    rz_flag_put(AGC_FLAG_ORBITAL_NAVIGATION_MARK_TAKEN, false); /* P22MKFLG */
    rz_flag_put(AGC_FLAG_ERADFLAG, true);
    on.ret = AGC_JOB_ORBITAL_NAVIGATION_OPTICS;

    if (agc_flag_test(AGC_FLAG_CMOONFLG))
    {
        rz_flag_put(AGC_FLAG_LUNAFLAG, true);
        on.noun = 70U; /* S22N7071 */

        return (agc_job_orbital_navigation_code(ctx));
    }

    rz_flag_put(AGC_FLAG_LUNAFLAG, false); /* PROG22B  */
    rz_flag_put(AGC_FLAG_KNOWNFLG, true);

    return (agc_job_orbital_navigation_landing_site(ctx)); /* P22SUBRB */
}

/**********************************************************************
 * S22DSPPA: V05N70 or V05N71 (GOFLASHR, R1 R3 blanked); PROCEED checks
 * the landmark code.
 * AGC: S22DSPPA (P20-P25.agc p.567)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_code
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_code(agc_job_ctx_t *ctx)
{
    agc_status_t st = rz_flash(ctx, AGC_DSKY_GOFLASHR, 5U, on.noun, AGC_JOB_GOTOPOOH, AGC_JOB_ORBITAL_NAVIGATION_CODE_CHECK, AGC_JOB_ORBITAL_NAVIGATION_CODE); /* V05N70/71 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(ON_BLANK_R1R3));
}

/**********************************************************************
 * S22DSPP: operator error light and the code display again.
 * AGC: S22DSPP (P20-P25.agc p.567)
 *
 * FUNCTION NAME: on_code_error
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t on_code_error(agc_job_ctx_t *ctx)
{
    agc_dsky_falton();

    return (agc_job_orbital_navigation_code(ctx));
}

/**********************************************************************
 * S22LSITE: the landing site RLS (moon fixed) in reference axes now
 * (RP-TO-R, VSR2 to B-29), and its LAT LONG ALT.
 * AGC: S22LSITE (P20-P25.agc p.568)
 *
 * FUNCTION NAME: on_site_from_rls
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the pad, RP-TO-R or LAT-LONG
 *          status
 **********************************************************************/
static agc_status_t on_site_from_rls(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_navaux_padload_t pad;                    /* pad loads */
    agc_dp_t             t = agc_rtb_loadtime(); /* now, cs 2^28 */
    agc_vec_t            r = agc_vec_zero();     /* RLS reference, 2^27 */

    agc_navaux_get_padload(&pad);
    agc_status_t st = agc_planet_rp_to_r(&ip, &pad, true, pad.rls, t, &r); /* status of the last step */

    agc_latlong_opt_t opt = {agc_flag_test(AGC_FLAG_LUNAFLAG), agc_flag_test(AGC_FLAG_ERADFLAG)};
    agc_latlong_t     ll; /* LAT LONG ALT */

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_from_r(&ip, &pad, opt, agc_vec_shr(r, 2), t, &ll);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_navaux_set_latlong(&ll);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * S22ABDE: unknown landmark: done; known: DE = 00 coordinates from noun
 * 89 (OBTAINLL), 01 from the landing site (S22LSITE).
 * AGC: S22ABDE, OBTAINLL (P20-P25.agc p.568)
 *
 * FUNCTION NAME: on_abde
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   de        uint16_t         I    DE of the code: 00 noun 89, 01 landing
 *                                   site
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t on_abde(agc_job_ctx_t *ctx, uint16_t de)
{
    if (!agc_flag_test(AGC_FLAG_KNOWNFLG))
    {
        return (rz_continue(on.ret));
    }

    if (de == 0U)
    {
        return (agc_job_orbital_navigation_landing_site(ctx)); /* OBTAINLL */
    }

    agc_status_t st = on_site_from_rls(); /* S22LSITE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rz_continue(on.ret));
}

/**********************************************************************
 * PROCEED on the code (p.568): B (offset) 0-5; A = 1 known (DE 00 or 01,
 * on noun 70 also 50-57: advanced orbit), A = 2 unknown; anything else
 * is an operator error.
 * AGC: S22N7071, 22SUBSCL (P20-P25.agc p.568)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_code_check
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_code_check(agc_job_ctx_t *ctx)
{
    uint16_t lm    = rz_word_of(agc_erase_landmark()); /* ABCDE word */
    uint16_t cxoff = (uint16_t) (lm & ON_PRIO7);       /* B (offset) */

    rz_landmark_set_offset((agc_sp_t) cxoff);

    if (cxoff > ON_PRIO5)
    {
        return (on_code_error(ctx));
    }

    rz_flag_put(AGC_FLAG_KNOWNFLG, false);
    uint16_t a = (uint16_t) (lm & ON_A_MASK); /* A */

    if (a == ON_A_UNKNOWN)
    {
        return (on_abde(ctx, 0U));
    }

    if (a != ON_A_KNOWN)
    {
        return (on_code_error(ctx)); /* A = 0, 3 .. 7    */
    }

    rz_flag_put(AGC_FLAG_KNOWNFLG, true);
    uint16_t de = (uint16_t) (lm & ON_DE_MASK); /* 22SUBSCL */

    if (de <= 1U)
    {
        return (on_abde(ctx, de));
    }

    if (on.noun != 70U || (de & ON_D_MASK) != ON_D_ADV_ORBIT)
    {
        return (on_code_error(ctx));
    }

    return (agc_job_orbital_navigation_advanced_optics(ctx)); /* DE-GR-50 */
} /* agc_job_orbital_navigation_code_check */

/**********************************************************************
 * P22SUBRB: V06N89 for the landmark coordinates (LAT, LONG/2, ALT).
 * AGC: P22SUBRB (P20-P25.agc p.569)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_landing_site
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_landing_site(agc_job_ctx_t *ctx)
{
    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 89U, AGC_JOB_GOTOPOOH, AGC_JOB_ORBITAL_NAVIGATION_SITE_CHECK, AGC_JOB_ORBITAL_NAVIGATION_LANDING_SITE));
}

/**********************************************************************
 * PROCEED on V06N89: LAT or LONG/2 beyond 90 degrees is an operator
 * error (N89ERRX); else LLASRDA and back.
 * AGC: P22SUBRB PROCEED, N89ERRX (P20-P25.agc p.569)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_site_check
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_site_check(agc_job_ctx_t *ctx)
{
    const agc_align_ram_t *a = agc_align_ram(); /* LANDLAT .. */

    if (agc_dp_abs(a->landlat) > AGC_DP_QUARTER || agc_dp_abs(a->landlong) > AGC_DP_QUARTER)
    {
        agc_dsky_falton(); /* N89ERRX */

        return (agc_job_orbital_navigation_landing_site(ctx));
    }

    agc_rendz_llasrda();

    return (rz_continue(on.ret));
}

/**********************************************************************
 * CALLR52 / DE-GR-50: restart at priority 13; five marks wanted; the
 * landmark as target; R52 (or ADVORB), then DOV5N71.
 * AGC: CALLR52, DE-GR-50 (P20-P25.agc p.566)
 *
 * FUNCTION NAME: on_optics
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   advanced  bool                   I    true: R52 advanced orbit (ADVORB)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t on_optics(const agc_job_ctx_t *ctx, bool advanced)
{
    agc_job_id_t restart = AGC_JOB_ORBITAL_NAVIGATION_OPTICS; /* CALLR52 */

    if (advanced)
    {
        restart = AGC_JOB_ORBITAL_NAVIGATION_ADVANCED_OPTICS; /* DE-GR-50 */
    }

    rz_phase_job(04U, 05022U, restart, 013);
    agc_align_ram()->markindx = 5;
    rz_flag_put(AGC_FLAG_TARG2FLG, true);
    rz_flag_put(AGC_FLAG_TARG1FLG, false);

    if (advanced)
    {
        return (agc_csm_auto_optics_advanced_orbit(ctx, AGC_JOB_ORBITAL_NAVIGATION_MARKED));
    }

    return (agc_csm_auto_optics_positioning(ctx, AGC_JOB_ORBITAL_NAVIGATION_MARKED));
}

/**********************************************************************
 * CALLR52: R52 to the landmark, then DOV5N71.
 * AGC: CALLR52 (P20-P25.agc p.566)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_optics
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_optics(agc_job_ctx_t *ctx)
{
    return (on_optics(ctx, false));
}

/**********************************************************************
 * DE-GR-50: R52 advanced orbit, then DOV5N71.
 * AGC: DE-GR-50 (P20-P25.agc p.568)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_advanced_optics
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_advanced_optics(agc_job_ctx_t *ctx)
{
    return (on_optics(ctx, true));
}

/**********************************************************************
 * DOV5N71: the landmark code again on noun 71, then PROG22C.
 * AGC: DOV5N71 (P20-P25.agc p.566)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_marked
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_marked(agc_job_ctx_t *ctx)
{
    on.noun = 71U;
    on.ret  = AGC_JOB_ORBITAL_NAVIGATION_PROCESS;

    return (agc_job_orbital_navigation_code(ctx));
}

/**********************************************************************
 * PROG22C: 8NN = the marks taken; S22.1, then P22OVER.
 * AGC: PROG22C (P20-P25.agc p.566)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_process
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_process(void)
{
    return (rz_landmark_marks_start(agc_sxtmark_marks_made(), AGC_JOB_ORBITAL_NAVIGATION_OVER));
}

/**********************************************************************
 * P22OVER: back to point A.
 * AGC: P22OVER (P20-P25.agc p.566)
 *
 * FUNCTION NAME: agc_job_orbital_navigation_over
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbital_navigation_over(agc_job_ctx_t *ctx)
{
    rz_grp2pc(AGC_JOB_ORBITAL_NAVIGATION_OVER);

    return (agc_job_orbital_navigation_landmark(ctx));
}

/**********************************************************************
 * P22SUBRB for other programs (P23): noun 89, the caller continues as
 * ret after LLASRDA.
 * AGC: P22SUBRB (P20-P25.agc p.569)
 *
 * FUNCTION NAME: rz_landing_site_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *   ret       agc_job_id_t     I    job that continues after LLASRDA
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_landing_site_display(agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    on.ret = ret;

    return (agc_job_orbital_navigation_landing_site(ctx));
}
