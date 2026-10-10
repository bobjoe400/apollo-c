/*
 * program.c -- P37_P70.agc (Comanche055, pp. 890-933): P37, RETURN
 * TO EARTH, the program and its displays.
 *
 * "A RETURN TO EARTH TRAJECTORY IS COMPUTED PROVIDED THE CSM IS OUTSIDE THE
 *  LUNAR SPHERE OF INFLUENCE AT THE TIME OF IGNITION. INITIALLY A CONIC
 *  TRAJECTORY IS DETERMINED AND RESULTING IGNITION AND REENTRY PARAMETERS
 *  ARE DISPLAYED TO THE ASTRONAUT. THEN IF THE ASTRONAUT SO DESIRES, A
 *  PRECISION TRAJECTORY IS DETERMINED WITH THE RESULTING IGNITION AND
 *  REENTRY PARAMETERS DISPLAYED. UPON FINAL ACCEPTANCE BY THE ASTRONAUT, THE
 *  PROGRAM COMPUTES AND STORES THE TARGET PARAMETERS FOR RETURN TO EARTH FOR
 *  USE BY SPS PROGRAM (P40) OR RCS PROGRAM (P41)."
 *
 * In Comanche the file holds only P37 (the "P70" of the title is the LM
 * abort program of Luminary; there is no Saturn-abort code here).
 *
 *   P37 .. RTE299     (pp. 891-892) V06N33 TIG, V06N60 GAMMAEI and DV
 *   RTE299 .. P37E    (pp. 891-895) INVC100 and the conic search
 *                     (conic.c, velocity.c)
 *   RTEVN, RTEDISP    (pp. 895-896, 902-904) N61 N39 N60 N81
 *   RTE505 .. RTE625  (p. 895) precision trajectory (precision.c)
 *   P37G .. P37W      (pp. 896-898) N06 SPS/RCS option, biased TIG (N33),
 *                     RTENCK1, aim point rotation, RTARG TPASS4 DELVSIN,
 *                     VN1645 (V16N45), then the crew selects P40/P41.
 *   P370ALRM/RTEALRM  (pp. 895, 900) alarm, V05N09, recycle to P37.
 *   P370GOF, P370GOFR, P37GFRB1, 37BLANK (pp. 898-899) display returns.
 *
 * Every flashing display is a job boundary (docs/TRANSLATION_GUIDE.md
 * section 4): its PROCEED, ENTER (or V32 recycle) and V34 returns are job
 * ids. Integrations (CSMPREC, RTENCK) may sleep in INTSTALL; the job then
 * returns and is resumed as the job named in the call.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "agc/core/assert.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/nav/conic.h"
#include "agc/nav/integration.h"
#include "agc/guidance/guidance.h"
#include "agc/programs/rendezvous.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_pending.h"
#include "internal.h"
#include "pending.h"

#define RTE_PHASE_NOT_RESTARTABLE ((uint16_t)04) /* PHASCHNG OCT 4: group 4 off */
#define RTE_ECSTEER       ((agc_sp_t)04000) /* AXT,1 OCT 04000; SXA,1 ECSTEER */
#define RTE_OPTION1_P37   ((agc_sp_t)7)     /* CAF SEVEN TS OPTION1          */
#define RTE_OPTION2_SPS   ((agc_sp_t)1)     /* OPTION2 1 = SPS, 2 = RCS      */
#define RTE_OPTION2_RCS   ((agc_sp_t)2)
#define RTE_BLANK_R1      ((uint16_t)1)     /* CAF ONE  TCR BLANKET          */
#define RTE_BLANK_R3      ((uint16_t)4)     /* CAF FOUR TCR 37BLANK +1       */

/* Constants (P37_P70.agc pp. 931-932, words from the Comanche055 rope).   */
static const agc_dp_t VCSPS    = 264328360;  /* VCSPS    37405 12250  31.510396 B-5 m/cs */
static const agc_dp_t VCRCS    = 227049420;  /* VCRCS    33041 37714  27.0664 B-5 m/cs   */
static const agc_dp_t MDOTRCS  = 54945;      /* MDOTRCS  00003 13241  .0016375 B-3 kg/cs */
static const agc_dp_t CSUBT    = 134217728;  /* CSUBT    20000 00000  .5                 */
static const agc_dp_t MCOS7_5  = -266138953; /* MCOS7.5  40214 45266  -.99144486         */
static const agc_dp_t MSIN7_5  = -35037857;  /* MSIN7.5  73645 56536  -.13052619         */
static const agc_dp_t MCOS22_5 = -62000506;  /* MCOS22.5 70467 71205  -.92387953 B-2     */
static const agc_dp_t THETA165 = 123032917;  /* THETA165 16525 12525  .4583333333 rev    */
static const agc_dp_t THETA210 = 156587349;  /* THETA210 22525 12525  .5833333333 rev    */
/* P37T POLY: A = (1 - E^(-DV/VC)) (B3) in x = DV/VC (B2).                  */
static const agc_dp_t BURN_POLY[3] = {
    19000,         /* 00001 05070  5.66240507 E-4 B-3   */
    131464640,     /* 17527 36700  9.79487897 E-1 B-1   */
    -208457287     /* 47114 70670  -.388281955 B1       */
};

/* P370GOF / P370GOFR (pp. 898-899): flashing display, V34 -> GOTOPOOH.    */
static agc_status_t rte_flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                              uint8_t verb, uint8_t noun, agc_job_id_t proceed,
                              agc_job_id_t enter)
{
    const agc_dsky_cont_t cont = { AGC_JOB_GOTOPOOH, proceed, enter };

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT((verb < 100U) && (noun < 100U))) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT((proceed != AGC_JOB_NONE) && (enter != AGC_JOB_NONE));
    return agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont);
}

/* P37GFRB1 / 37BLANK (p. 898): GOFLASHR, then on the immediate return blank
 * the given registers and ENDOFJOB.                                       */
static agc_status_t rte_flash_blank(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun,
                                    uint16_t blank, agc_job_id_t proceed,
                                    agc_job_id_t enter)
{
    agc_status_t st = rte_flash(ctx, AGC_DSKY_GOFLASHR, verb, noun, proceed, enter);

    (void)AGC_ASSERT((blank == RTE_BLANK_R1) || (blank == RTE_BLANK_R3));
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_blanket(blank);                               /* TCF ENDOFJOB  */
}

/* RTEALRM / P370ALRM (pp. 895, 900): VARALARM with the code, then V05N09. */
static agc_status_t rte_alarm(const agc_job_ctx_t *ctx, const rte_erasable_t *rs)
{
    if (!AGC_ASSERT((ctx != NULL) && (rs != NULL)) || !AGC_ASSERT(rs->alarm_code != 0U)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_exec_varalarm(rs->alarm_code);
    if (st != AGC_STATUS_ALARM) {
        return st;
    }
    return rte_flash(ctx, AGC_DSKY_GOFLASH, 5U, 9U, AGC_JOB_RETURN_TO_EARTH_ALARM_DISPLAY,
                     AGC_JOB_RETURN_TO_EARTH);
}

/* The result of a computation step: alarm display, or the abort status. */
static agc_status_t rte_failed(const agc_job_ctx_t *ctx, const rte_erasable_t *rs,
                               agc_status_t st)
{
    (void)AGC_ASSERT(st != AGC_STATUS_OK);
    (void)AGC_ASSERT(rs != NULL);
    return (st == AGC_STATUS_ALARM) ? rte_alarm(ctx, rs) : st;
}

/* P370ALRM after PROCEED (p. 900, TCF -4): show V05N09 again; V32 recycles
 * to P37, V34 terminates.                                                 */
agc_status_t agc_job_return_to_earth_alarm_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(rte_state()->alarm_code != 0U);
    return rte_flash(ctx, AGC_DSKY_GOFLASH, 5U, 9U, AGC_JOB_RETURN_TO_EARTH_ALARM_DISPLAY,
                     AGC_JOB_RETURN_TO_EARTH);
}

/* P37 (p. 891): not restartable (PHASCHNG 4.0); ECSTEER = 04000 (SXA
 * stores the high word only); VPRED = GAMMAEI = 0; V06N33.               */
agc_status_t agc_job_return_to_earth(agc_job_ctx_t *ctx)
{
    agc_guid_t        *g = agc_guid_state();
    agc_entry_state_t *es = agc_entry_state();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT((g != NULL) && (es != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_phase_change(4, RTE_PHASE_NOT_RESTARTABLE);
    g->burn.ecsteer = agc_dp_pack(RTE_ECSTEER, agc_dp_low(g->burn.ecsteer));
    es->vpred = 0;                                                /* ZEROVECS      */
    es->gammaei = 0;
    return agc_job_return_to_earth_tig_display(ctx);
}

/* P37 (p. 891): V06N33, TIG (SPRTETIG overlays TIG). ENTER shows it again. */
agc_status_t agc_job_return_to_earth_tig_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return rte_flash(ctx, AGC_DSKY_GOFLASH, 6U, 33U, AGC_JOB_RETURN_TO_EARTH_ENTRY_DISPLAY,
                     AGC_JOB_RETURN_TO_EARTH_TIG_DISPLAY);
}

/* P37 (p. 891): V06N60 with R1 blank: "INPUT REENTRY ANGLE IN GAMMAEI AND
 * DESIRED DELTA V" (VPRED register). PROCEED: RTE299.                    */
agc_status_t agc_job_return_to_earth_entry_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return rte_flash_blank(ctx, 6U, 60U, RTE_BLANK_R1, AGC_JOB_RETURN_TO_EARTH_CONIC_SEARCH,
                           AGC_JOB_RETURN_TO_EARTH_ENTRY_DISPLAY);
}

/* RTEVN with RTEDISP (pp. 895, 902-904): compute and store the display
 * values, then V06N61 (LAT(SPL), LNG(SPL), R3 blank). VNSTORE = where the
 * last display (N81) proceeds to.                                          */
static agc_status_t rte_show_solution(const agc_job_ctx_t *ctx, agc_interp_t *ip,
                                      rte_erasable_t *rs, agc_job_id_t vnstore)
{
    rte_display_t d = { 0 };

    if (!AGC_ASSERT((ctx != NULL) && (ip != NULL)) || !AGC_ASSERT(rs != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(vnstore != AGC_JOB_NONE);
    rs->vnstore = vnstore;                                        /* STQ VNSTORE   */
    agc_status_t st = rte_display_values(ip, rs, agc_erase_tig(),
                                         agc_erase_return_to_earth_range(), &d);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_entry_state_t *es = agc_entry_state();
    es->vpred = d.vpred;
    es->gammaei = d.gammaei;
    es->lat_spl = d.lat_spl;
    es->lng_spl = d.lng_spl;
    agc_erase_set_t3tot4(d.t3tot4);
    agc_rendz_erasable()->delvlvc = d.delvlvc;
    agc_guid_state()->burn.vgdisp = d.vgdisp;
    return rte_flash_blank(ctx, 6U, 61U, RTE_BLANK_R3, AGC_JOB_RETURN_TO_EARTH_TRANSIT_DISPLAY,
                           AGC_JOB_RETURN_TO_EARTH);
}

/* RTE299 (pp. 891-895): RTEDVD = VPRED, RTEGAM2D = GAMMAEI, INVC100
 * (CSMPREC to TIG), the conic search, then P37E: RTEVN, returning to
 * RTE505 (precision). An INTSTALL sleep resumes this job from its start. */
agc_status_t agc_job_return_to_earth_conic_search(agc_job_ctx_t *ctx)
{
    agc_interp_t      *ip = rte_ip();
    rte_erasable_t    *rs = rte_state();
    agc_integ_ctx_t   *ic = agc_integ_shared_ctx();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ic != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    ip->ovfind = 0;                                               /* SSP OVFIND 0  */
    rs->rtedvd = agc_entry_state()->vpred;
    rs->rtegam2d = agc_entry_state()->gammaei;
    agc_status_t st = agc_integ_csmprec(ip, ic, agc_erase_tig(),
                                        AGC_JOB_RETURN_TO_EARTH_CONIC_SEARCH);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;                                     /* asleep        */
    }
    if (st == AGC_STATUS_OK) {
        st = rte_initial_vectors(ip, rs, ic->out.ratt, ic->out.vatt, ic->out.tat, ic->out.x2);
    }
    if (st == AGC_STATUS_OK) {
        st = rte_conic_setup(ip, rs);
    }
    if (st == AGC_STATUS_OK) {
        st = rte_conic_search(ip, rs, agc_erase_rted1());
    }
    if (st != AGC_STATUS_OK) {
        return rte_failed(ctx, rs, st);
    }
    return rte_show_solution(ctx, ip, rs, AGC_JOB_RETURN_TO_EARTH_PRECISION); /* P37E */
}

/* RTEVN (p. 895): V06N39 T3TOT4 (TIG to 400,000 ft). V32: P37.            */
agc_status_t agc_job_return_to_earth_transit_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return rte_flash(ctx, AGC_DSKY_GOFLASH, 6U, 39U,
                     AGC_JOB_RETURN_TO_EARTH_ENTRY_CONDITIONS_DISPLAY, AGC_JOB_RETURN_TO_EARTH);
}

/* RTEVN (p. 895): V06N60, R1 blank, VPRED and GAMMAEI. V32: P37.          */
agc_status_t agc_job_return_to_earth_entry_conditions_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return rte_flash_blank(ctx, 6U, 60U, RTE_BLANK_R1, AGC_JOB_RETURN_TO_EARTH_DELTA_V_DISPLAY,
                           AGC_JOB_RETURN_TO_EARTH);
}

/* RTEVN (p. 896): V06N81 DELVLVC; PROCEED returns to VNSTORE. V32: P37.   */
agc_status_t agc_job_return_to_earth_delta_v_display(agc_job_ctx_t *ctx)
{
    const rte_erasable_t *rs = rte_state();

    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(rs->vnstore != AGC_JOB_NONE);
    return rte_flash(ctx, AGC_DSKY_GOFLASH, 6U, 81U, rs->vnstore, AGC_JOB_RETURN_TO_EARTH);
}

/* RTE505 .. RTE515 (p. 895): PHI2, then PREC100.                          */
agc_status_t agc_job_return_to_earth_precision(agc_job_ctx_t *ctx)
{
    rte_erasable_t *rs = rte_state();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(rs->rcon > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = rte_entry_side(rte_ip(), rs);
    if (st == AGC_STATUS_OK) {
        st = rte_precision_start(rs);
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_job_return_to_earth_precision_resume(ctx);
}

/* PREC120 .. PRECX and RTE625 (pp. 895, 908-913): run (or resume after an
 * INTSTALL sleep) the precision iteration; success: P37G, RTEVN returning
 * to P37N; failure: RTEALRM.                                              */
agc_status_t agc_job_return_to_earth_precision_resume(agc_job_ctx_t *ctx)
{
    agc_interp_t   *ip = rte_ip();
    rte_erasable_t *rs = rte_state();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(rs->nn1a >= 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = rte_precision_run(ip, rs, AGC_JOB_RETURN_TO_EARTH_PRECISION_RESUME);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;                                     /* asleep        */
    }
    if (st != AGC_STATUS_OK) {
        return rte_failed(ctx, rs, st);
    }
    return rte_show_solution(ctx, ip, rs, AGC_JOB_RETURN_TO_EARTH_PROPULSION_OPTION); /* P37G */
}

/* P37N (p. 896): OPTION1 = 7, OPTION2 = 1 (SPS assumed), V04N06.          */
agc_status_t agc_job_return_to_earth_propulsion_option(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    agc_erase_set_option1(0, RTE_OPTION1_P37);
    agc_erase_set_option1(1, RTE_OPTION2_SPS);
    return agc_job_return_to_earth_option_display(ctx);
}

/* P37N (p. 896): V04N06 "DISPLAY RCS OR SPS OPTION"; ENTER shows it again. */
agc_status_t agc_job_return_to_earth_option_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return rte_flash(ctx, AGC_DSKY_GOFLASH, 4U, 6U, AGC_JOB_RETURN_TO_EARTH_IGNITION_TIME,
                     AGC_JOB_RETURN_TO_EARTH_OPTION_DISPLAY);
}

/* P37N PROCEED .. P37T (pp. 896-897): the burn is centred on T1:
 *   A = 1 - E^(-DV/VC) (POLY, B3), DTB = (M0/MDOT) A (cs B28),
 *   TIG = T1 - CSUBT DTB.
 * SPS: MDOT = EMDOT, VC = VCSPS; RCS: MDOT = 2 or 4 x MDOTRCS (NJETSFLG
 * set = 2 jets), VC = VCRCS.                                              */
agc_status_t agc_job_return_to_earth_ignition_time(agc_job_ctx_t *ctx)
{
    agc_interp_t     *ip = rte_ip();
    rte_erasable_t   *rs = rte_state();
    agc_csm_dapcom_t *dc = agc_csm_dap_com();
    agc_dp_t mdot;
    agc_dp_t vc = VCSPS;
    int n1 = 0;

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(dc != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_erase_option1(1) == RTE_OPTION2_RCS) {                /* P37Q          */
        mdot = agc_flag_test(AGC_FLAG_NJETSFLG) ? MDOTRCS : agc_dp_shl(ip, MDOTRCS, 1);
        mdot = agc_dp_shl(ip, mdot, 1);                           /* P37R SL1      */
        vc = VCRCS;
    } else {
        mdot = agc_dp_from_sp(dc->emdot);                         /* SLOAD EMDOT   */
    }
    mdot = agc_dp_norm(mdot, &n1);                                /* NORM X1       */
    if (!AGC_ASSERT(mdot != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(rs->dv >= 0);
    agc_dp_t a = agc_dp_poly(agc_dp_div(ip, rs->dv, vc), BURN_POLY, 2); /* P37T  */
    agc_dp_t w = agc_dp_mul_trunc(agc_dp_from_sp(dc->weight_g), a);   /* DMP     */
    agc_dp_t dtb = agc_dp_shl(ip, agc_dp_div(ip, w, mdot), n1 - 12);  /* SL* B28 */
    agc_erase_set_tig(agc_dp_sub(ip, rs->t1, agc_dp_mul_trunc(dtb, CSUBT)));
    return agc_job_return_to_earth_biased_tig_display(ctx);
}

/* P37T (p. 897): V06N33 "DISPLAY BIASED TIG"; ENTER shows it again.       */
agc_status_t agc_job_return_to_earth_biased_tig_display(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return rte_flash(ctx, AGC_DSKY_GOFLASH, 6U, 33U, AGC_JOB_RETURN_TO_EARTH_TARGETING,
                     AGC_JOB_RETURN_TO_EARTH_BIASED_TIG_DISPLAY);
}

/* P37 after N33 .. P37V (pp. 897-898): VHFCNT = TRKMKCNT = 0; RTENCK1
 * (conic R(T1)/, V2(T1)/ to T2). If the entry point is more than 22.5 deg
 * past K/ = -UR1 cos 7.5 - UH sin 7.5, the Lambert aim point is moved to
 * 165 deg (UH.R(T2) >= 0) or 210 deg from R(T1) (TIMETHET).               */
agc_status_t agc_job_return_to_earth_targeting(agc_job_ctx_t *ctx)
{
    agc_interp_t   *ip = rte_ip();
    rte_erasable_t *rs = rte_state();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(rs->t2 != rs->t1)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_erase_set_vhfcnt(0);
    agc_erase_set_trkmkcnt(0);
    agc_status_t st = rte_integrate(ip, rs, rs->r_t1v, rs->v2_t1v, rs->t1, rs->t2, true,
                                    AGC_JOB_RETURN_TO_EARTH_TARGETING);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;                                     /* asleep        */
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    rs->rtarg = rs->r_t2v;                                        /* pushed R(T2)/ */
    agc_vec_t ur2 = agc_vec_unit(ip, rs->r_t2v, NULL, NULL);
    agc_vec_t k = agc_vec_add(ip, agc_vec_scale(ip, rs->ur1, MCOS7_5),
                              agc_vec_scale(ip, rs->uh, MSIN7_5));
    if (agc_dp_add(ip, agc_vec_dot(ip, k, ur2), MCOS22_5) >= 0) {
        agc_timetheta_in_t   in;
        agc_conic_time_out_t out;
        agc_dp_t th = (agc_vec_dot(ip, rs->uh, rs->r_t2v) < 0) ? THETA210 : THETA165;
        in.snth = agc_dp_sin(ip, th);                             /* P37V          */
        in.csth = agc_dp_cos(ip, th);
        agc_flag_clear(AGC_FLAG_RVSW);
        in.rvsw = false;
        in.rvec = rs->r_t1v;
        in.vvec = rs->v2_t1v;
        in.body = AGC_CONIC_EARTH;
        st = agc_conic_timethet(ip, &in, &out);
        if (st != AGC_STATUS_OK) {
            return st;
        }
        rs->rtarg = out.rnew;
        rs->conic_t = out.t;
    }
    return agc_job_return_to_earth_target_store(ctx);
}

/* P37W (p. 898): CLEAR XDELVFLG (Lambert aim point), NORMSW; SET FINALFLG;
 * RTARG, TPASS4 = T + T1, DELVSIN = V2(T1)/ - V(T1)/; VN1645 (V16N45),
 * which returns here on recycle.
 * DEVIATION: the AGC takes RTARG from the push-down list (VLOAD with no
 * address), so a recycle pops a lower, stale, cell; here the vector saved
 * by the targeting step is used each time.                                */
agc_status_t agc_job_return_to_earth_target_store(agc_job_ctx_t *ctx)
{
    agc_interp_t         *ip = rte_ip();
    const rte_erasable_t *rs = rte_state();
    agc_rendz_t          *rz = agc_rendz_erasable();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(rz != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_flag_clear(AGC_FLAG_XDELVFLG);
    agc_flag_clear(AGC_FLAG_NORMSW);
    agc_flag_set(AGC_FLAG_FINALFLG);
    (void)AGC_ASSERT(rs->conic_t != 0);
    rz->rtarg = rs->rtarg;
    rz->tpass4 = agc_dp_add(ip, rs->conic_t, rs->t1);
    rz->delvsin = agc_vec_sub(ip, rs->v2_t1v, rs->v_t1v);
    return agc_rendz_ttogo_display_start(ctx, ip, AGC_JOB_RETURN_TO_EARTH_TARGET_STORE);
}
