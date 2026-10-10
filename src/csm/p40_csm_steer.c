/*
 * p40_csm_steer.c -- P40-P47.agc, Comanche055 pp. 697-728: the 2-second
 * steering cycle run from AVGEXIT: UPDATEVG (with SETUP.9 / SURELY.9),
 * S40.8 (VG update, low-thrust test, TGOCALC, XPRODUCT, S40.81, LOTHRUST),
 * STEERING (engine-off set-up), CALCN85, P40CNV85, CALCN83, SERVXT, the
 * S40.9 Lambert job (ENDLAMB, FIRSTTME, COPY40.9, REDO40.9) and the
 * S40.13 TIMEBURN job.
 *
 * Restart points (type B phase changes inside the cycle) resume in the
 * continuation jobs AGC_JOB_S40_8B, UPDATEVG_B, SURELY9_B, STEERING_B and
 * CALCN83_B; CSM.UPDCALLER records which AVGEXIT routine called UPDATEVG
 * (the AGC keeps that in QTEMP1).
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "agc/agc_guid.h"
#include "p40_csm_internal.h"

#define CALLER_STEERING 1
#define CALLER_CALCN85  2
#define PH_5_3_B        ((uint16_t)010035)

/* 2VEXHUST 2DEC 63.020792 B-7: 2 (10 338.0564 ft/s), m/cs 2^7.            */
static const agc_dp_t TWOVEXHUST = 132164180;
/* FOURSEC 2DEC 400.0 B-28; DPB-9 2DEC 1 B-9.                              */
static const agc_dp_t FOURSEC = 400;
static const agc_dp_t DPB_9 = 524288;
/* 200CSHI 2DEC 200 B-12; EARTHMU 2DEC* -3.986032 E10 B-36*.               */
static const agc_dp_t CS200HI = 13107200;
static const agc_dp_t EARTHMU = -155704375;
/* EP4(45)L .125, EP4(10)L .027777777: EPSILON4 for HAVEGUES.              */
static const agc_dp_t EP4_45L = 33554432;
static const agc_dp_t EP4_10L = 7456540;

/* PHASCHNG type B for group 5 (5.3 REREADAC + this job at PRIO20).        */
static void phase_b(agc_job_id_t job)
{
    agc_phase_var_t v = { AGC_RESTART_JOB, job, AGC_TASK_NONE, (int16_t)AGC_PRIO20,
                          false, 0U };
    (void)AGC_ASSERT(job != AGC_JOB_NONE);
    (void)agc_phase_change_var(PH_5_3_B, &v);
}

/* SERVXT (p.704): POSTJUMP SERVEXIT.                                       */
agc_status_t agc_csm_servxt(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id != AGC_JOB_NONE);
    return agc_job_servexit(ctx);
}

/* S40.81 (p.722): TGO < 4 s: IMPULSW set (engine-off call), OMEGAC = 0
 * (TVC attitude hold), CNTR = POSMAX (no switch-over/CG tracking),
 * STEERSW reset.                                                          */
static agc_status_t s40_81(agc_csm_tvc_t *tvc)
{
    (void)agc_flag_set(AGC_FLAG_IMPULSW);
    tvc->omegac[0] = tvc->omegac[1] = tvc->omegac[2] = 0;   /* RATEZRO         */
    tvc->cntr = AGC_SP_MAX;
    (void)AGC_ASSERT(tvc->cntr == AGC_SP_MAX);
    return agc_flag_clear(AGC_FLAG_STEERSW);
}

/* LOTHRUST (p.722): thrust failure, unless IDLEFAIL (one bypass after V97
 * PROCEED): TVC to attitude hold, mass updating and CG tracking killed,
 * V97 flash by the CLOCKJOB, steering off.                                */
static agc_status_t lothrust(agc_csm_tvc_t *tvc)
{
    if (agc_flag_test(AGC_FLAG_IDLEFAIL)) {
        return AGC_STATUS_OK;
    }
    tvc->omegac[0] = tvc->omegac[1] = tvc->omegac[2] = 0;
    tvc->vcntr = 0;                                   /* CS ZERO: KILL MASS UPD. */
    tvc->vcntrtmp = 0;
    tvc->repfrac = 0;
    agc_guid_state()->csm.nvword1 = AGC_CSM_NV_V97;   /* ENGINE-FAIL V97 FLASH  */
    (void)AGC_ASSERT(tvc->vcntr == 0);
    return agc_flag_clear(AGC_FLAG_STEERSW);
}

/* S40.8 after its restart point (p.719-720): VGPREV = VG; with STEERSW:
 * low-thrust test against DVTHRESH, CG tracking fraction, TGOCALC (TIG =
 * PIPTIME + TGO), S40.81 when TGO < 4 s, else XPRODUCT.                    */
static agc_status_t s40_8_part2(agc_interp_t *ip)
{
    agc_guid_t *g = agc_guid_state();
    agc_csm_tvc_t *tvc = agc_csm_dap_tvc();
    agc_erase_set_vgtig(g->burn.vg);                  /* VGPREV = VG            */
    if (!agc_flag_test(AGC_FLAG_STEERSW)) {
        return AGC_STATUS_OK;                         /* SKIP TGO AND XPRODUCT  */
    }
    agc_dp_t thr = agc_dp_mul(ip, agc_dp_from_sp(g->csm.dvthresh), DPB_9);
    if (agc_dp_sub(ip, agc_vec_abval(ip, g->avg.delvref), thr) < 0) {
        return lothrust(tvc);
    }
    int lmon = ((agc_erase_dapdatr1() & 020000U) != 0U) ? 1 : 0;  /* BIT14   */
    tvc->repfrac = agc_csm_dap_com()->erepfrac[lmon];
    agc_tgo_in_t in = { g->burn.vg, agc_vec_sub(ip, g->burn.bdt, g->avg.delvref),
                        TWOVEXHUST, agc_dp_shr(agc_dp_from_sp(g->csm.etdecay), 14) };
    agc_dp_t tgo = 0;
    agc_status_t st = agc_guid_s40_8_tgo_csm(ip, &in, &tgo);
    if (st == AGC_STATUS_ALARM) {
        return agc_alarm(AGC_ALARM_01407);            /* INCRSVG                */
    }
    agc_erase_set_tgo(tgo);
    agc_erase_set_tig(agc_dp_add(ip, tgo, agc_erase_piptime()));
    if (agc_dp_sub(ip, tgo, FOURSEC) < 0) {
        return s40_81(tvc);
    }
    agc_xprod_in_t xin = { g->burn.vg, g->avg.delvref, g->burn.bdt, g->burn.csteer,
                           tvc->kprimedt };
    agc_mat_t refsmmat = agc_erase_refsmmat();
    agc_vec_t om;
    st = agc_guid_s40_8_xproduct(ip, &xin, &refsmmat, &g->trig, &om);
    for (int i = 0; i < 3; i++) {
        tvc->omegac[i] = om.c[i];                     /* OMEGACLC               */
    }
    return st;
}

/* S40.8 Comanche (p.719): VG = VGPREV + BDT - DELVREF, VGDISP = |VG|,
 * type B restart (5.3 and S40.8B), then the rest.                         */
agc_status_t agc_csm_s40_8(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    g->burn.vg = agc_vec_add(ip, agc_vec_sub(ip, g->burn.bdt, g->avg.delvref),
                             agc_erase_vgtig());       /* BVSU VAD VGPREV        */
    g->burn.vgdisp = agc_vec_abval(ip, g->burn.vg);
    phase_b(AGC_JOB_S40_8B);
    return s40_8_part2(ip);
}

/* SURELY.9 continuation (p.704): the S40.9 initial conditions RINIT VINIT
 * TNIT DELLT4 from the present state, DELVSUM and NBRCYCLS/P cleared.     */
static agc_status_t surely9_b(agc_interp_t *ip)
{
    agc_guid_t *g = agc_guid_state();
    agc_rendz_t *rz = agc_rendz_erasable();
    rz->rinit = agc_erase_rn();                       /* RADIUS VECTOR AT T1    */
    rz->vinit = agc_erase_vn();
    g->burn.tnit = agc_erase_piptime();
    rz->dellt4 = agc_dp_sub(ip, rz->tpass4, g->burn.tnit);
    g->burn.delvsum = agc_vec_zero();
    g->burn.nbrcycls = 0;
    g->burn.nbrcyclp = 0;
    (void)AGC_ASSERT(g->burn.tnit >= 0);
    return agc_csm_s40_8(ip);
}

/* SETUP.9 / SURELY.9 (p.703-704): on later passes VGPREV = VGTEMP +
 * NBRCYCLP BDT - DELVSUM (BDT times the cycle count through NORM X1 and
 * VSR* 0-14D,1); start the S40.9 job; 1.5 (REDO40.9) and 5.3.             */
static agc_status_t setup_9(agc_interp_t *ip)
{
    agc_guid_t *g = agc_guid_state();
    if (!agc_flag_test(AGC_FLAG_FIRSTFLG)) {
        int k = 0;
        agc_dp_t n = agc_dp_norm(agc_dp_from_sp(g->burn.nbrcyclp), &k);
        agc_vec_t b = agc_vec_shl(ip, agc_vec_scale(ip, g->burn.bdt, n), 14 - k);
        b = agc_vec_add(ip, b, agc_rendz_erasable()->delveet3);  /* VGTEMP      */
        agc_erase_set_vgtig(agc_vec_sub(ip, b, g->burn.delvsum)); /* VGPREV     */
    }
    agc_status_t st = agc_exec_findvac(AGC_PRIO10, AGC_JOB_S40_9);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_SURELY9_B, AGC_TASK_NONE,
                          (int16_t)AGC_PRIO20, false, 0U };
    (void)agc_phase_change2((uint16_t)000051, PH_5_3_B, &v);
    (void)AGC_ASSERT(g->burn.nbrcyclp >= 0);
    return surely9_b(ip);
}

/* UPDATEVG (p.703): external delta-V: S40.8 only; aimpoint: every cycle
 * DELVSUM += DELVREF and NBRCYCLS += 1 (restart protected), S40.9 set up
 * when NBRCYCLS < 0.                                                      */
agc_status_t agc_csm_updatevg(agc_interp_t *ip, bool *s40_9_started)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(s40_9_started != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    *s40_9_started = false;
    if (agc_flag_test(AGC_FLAG_XDELVFLG)) {
        return agc_csm_s40_8(ip);                     /* CALL40.8               */
    }
    if (g->burn.nbrcycls < 0) {
        *s40_9_started = true;
        return setup_9(ip);
    }
    g->burn.delvsump = agc_vec_add(ip, g->burn.delvsum, g->avg.delvref);
    g->burn.nbrcyclp = (agc_sp_t)(g->burn.nbrcycls + 1);
    phase_b(AGC_JOB_UPDATEVG_B);
    g->burn.nbrcycls = g->burn.nbrcyclp;
    g->burn.delvsum = g->burn.delvsump;
    return agc_csm_s40_8(ip);
}

/* The rest of STEERING after UPDATEVG (p.704): with IMPULSW and the engine
 * on, set up ENGINOFF at TIG (at least 1 cs ahead), 3.15 and 5.3.        */
static agc_status_t steering_tail(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_IMPULSW) ||
        (agc_chan_read(011) & AGC_CSM_SPSON) == 0U) {
        return agc_csm_servxt(ctx);                   /* PRE-IGNITE / POST-OFF  */
    }
    agc_inhint();
    agc_dp_t t = agc_erase_tig() - (agc_dp_t)agc_time_now_cs();   /* TPAGREE   */
    agc_sp_t lo = agc_dp_low(t);
    agc_sp_t dt = (agc_dp_high(t) == 0 && lo > 0) ? lo : 1;      /* CCS: >= 1 */
    agc_erase_set_tgo((agc_dp_t)dt);
    (void)agc_waitlist_call((uint16_t)dt, AGC_TASK_ENGINOFF);
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_STEERING_B, AGC_TASK_NONE,
                          (int16_t)AGC_PRIO20, false, 0U };
    (void)agc_phase_change2((uint16_t)040153, PH_5_3_B, &v);
    agc_relint();
    (void)AGC_ASSERT(dt >= 1);
    return agc_job_steering_b(ctx);
}

/* STEERING (p.704): the AVGEXIT of P40.                                    */
agc_status_t agc_job_steering_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_state()->csm.updcaller = CALLER_STEERING;
    agc_interp_t ip;
    agc_interp_init(&ip);
    bool s409 = false;
    agc_status_t st = agc_csm_updatevg(&ip, &s409);
    if (st != AGC_STATUS_OK && st != AGC_STATUS_ALARM) {
        return st;
    }
    return steering_tail(ctx);
}

/* STEERING after its restart point: IMPULSW reset (ENGINOFF is set up).   */
agc_status_t agc_job_steering_b(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_flag_clear(AGC_FLAG_IMPULSW);
    (void)AGC_ASSERT(!agc_flag_test(AGC_FLAG_IMPULSW));
    return agc_csm_servxt(ctx);
}

/* P40CNV85 (p.697): VGBODY = S41.1(VGPREV), VG in control axes for N85.   */
agc_status_t agc_csm_p40cnv85(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_burn_t *b = &agc_guid_state()->burn;
    (void)AGC_ASSERT(b != NULL);
    return agc_guid_s41_1(ip, AGC_VEHICLE_CSM, agc_erase_vgtig(), &b->vgbody);
}

/* CALCN85 (p.697): AVEGEXIT of P41 / post-burn: UPDATEVG, N85, SERVEXIT.  */
agc_status_t agc_job_calcn85_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_state()->csm.updcaller = CALLER_CALCN85;
    agc_interp_t ip;
    agc_interp_init(&ip);
    bool s409 = false;
    (void)agc_csm_updatevg(&ip, &s409);
    (void)agc_csm_p40cnv85(&ip);
    return agc_csm_servxt(ctx);
}

/* The caller's tail after a restart inside UPDATEVG / S40.8.               */
static agc_status_t caller_tail(agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (agc_guid_state()->csm.updcaller == CALLER_CALCN85) {
        (void)agc_csm_p40cnv85(ip);
        return agc_csm_servxt(ctx);
    }
    (void)AGC_ASSERT(agc_guid_state()->csm.updcaller == CALLER_STEERING);
    return steering_tail(ctx);
}

/* Restart entries inside the cycle: S40.8B, UPDATEVG_B, SURELY9_B.        */
agc_status_t agc_job_s40_8b_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    agc_interp_t ip;
    agc_interp_init(&ip);
    if (ctx->id == AGC_JOB_UPDATEVG_B) {
        g->burn.nbrcycls = g->burn.nbrcyclp;
        g->burn.delvsum = g->burn.delvsump;
        (void)agc_csm_s40_8(&ip);
    } else if (ctx->id == AGC_JOB_SURELY9_B) {
        (void)surely9_b(&ip);
    } else {
        (void)s40_8_part2(&ip);                       /* AGC_JOB_S40_8B         */
    }
    return caller_tail(ctx, &ip);
}

/* CALCN83 (p.698): P47 thrust monitor: DV47TEMP (= VG) = DELVCTL +
 * DELVREF, DELVIMU = S41.1(DV47TEMP), S11.1 (V I, H, HDOT for N62), then
 * DELVCTL = DV47TEMP after a 5.3 type B restart point.                    */
agc_status_t agc_job_calcn83_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    agc_interp_t ip;
    agc_interp_init(&ip);
    g->burn.vg = agc_vec_add(&ip, g->burn.delvctl, g->avg.delvref);
    (void)agc_guid_s41_1(&ip, AGC_VEHICLE_CSM, g->burn.vg, &g->burn.delvimu);
    (void)agc_csm_s11_1(&ip);
    phase_b(AGC_JOB_CALCN83_B);
    return agc_job_calcn83_b(ctx);
}

agc_status_t agc_job_calcn83_b(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    agc_inhint();
    g->burn.delvctl = g->burn.vg;                     /* GENTRAN DV47TEMP       */
    agc_relint();
    return agc_csm_servxt(ctx);
}

/* S40.9 (p.723-724): HAVEGUES (GUESSW reset: use the last COGA) with
 * NUMIT 0, EPSILON4 45 (NORMSW) or 10 degrees; 1 type C at ENDLAMB.       */
agc_status_t agc_job_s40_9_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_dp_t e4 = agc_flag_test(AGC_FLAG_NORMSW) ? EP4_45L : EP4_10L;
    (void)agc_flag_clear(AGC_FLAG_GUESSW);
    agc_status_t st = agc_rendz_havegues(&ip, agc_rendz_erasable(), 0, e4);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_ENDLAMB, AGC_TASK_NONE,
                          (int16_t)AGC_PRIO10, false, 0U };
    (void)agc_phase_change_var((uint16_t)005021, &v);
    return agc_job_endlamb(ctx);
}

/* ENDLAMB / FIRSTTME (p.724): after the first pass BDT = (VIPRIME -
 * VRPREV) 2 s / (TNIT - TNITPREV) - GDT/2 (VSL1); about the earth VGTEMP +=
 * GOBL/2 (PIPTIME - NOMTIG) EARTHMU / RN^2. Then 1 type C at COPY40.9.    */
agc_status_t agc_job_endlamb(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_interp_t ip;
    agc_interp_init(&ip);
    if (!agc_flag_test(AGC_FLAG_FIRSTFLG)) {
        agc_dp_t dtn = agc_dp_shl(&ip, agc_dp_sub(&ip, g->burn.tnit, g->burn.tnitprev), 17);
        agc_dp_t k = agc_dp_div(&ip, CS200HI, dtn);              /* BDDV 200CSHI */
        agc_vec_t dv = agc_vec_scale(&ip, agc_vec_sub(&ip, rz->viprime, g->burn.vrprev), k);
        g->burn.bdt = agc_vec_shl(&ip, agc_vec_sub(&ip, dv, g->avg.gdt_2), 1);
    }
    if (rz->body.rtx2 == 0) {                                    /* NOT MOONCASE */
        agc_dp_t rsq = 0;
        (void)agc_vec_unit(&ip, agc_erase_rn(), NULL, &rsq);    /* 34D          */
        agc_dp_t s = agc_dp_mul(&ip, agc_dp_sub(&ip, agc_erase_piptime(), g->burn.nomtig),
                                EARTHMU);
        s = agc_dp_div(&ip, s, rsq);
        rz->delveet3 = agc_vec_add(&ip, agc_vec_scale(&ip, g->avg.gobl_2, s), rz->delveet3);
    }
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_COPY40_9, AGC_TASK_NONE,
                          (int16_t)AGC_PRIO10, false, 0U };
    (void)agc_phase_change_var((uint16_t)004021, &v);
    return agc_job_copy40_9(ctx);
}

/* COPY40.9 / ENDS40.9 (p.724-725): TNITPREV = TNIT, VRPREV = VIPRIME,
 * FIRSTFLG reset, NBRCYCLS = -1 (next UPDATEVG sets up S40.9), group 1 off. */
agc_status_t agc_job_copy40_9(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    g->burn.tnitprev = g->burn.tnit;
    g->burn.vrprev = agc_rendz_erasable()->viprime;
    (void)agc_flag_clear(AGC_FLAG_FIRSTFLG);
    g->burn.nbrcycls = -1;
    (void)AGC_ASSERT(g->burn.nbrcycls == -1);
    return agc_phase_kill(1);
}

/* REDO40.9 (p.725): restart of S40.9: give up this pass, keep VGPREV.     */
agc_status_t agc_job_redo40_9(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    g->burn.delvsum = agc_vec_zero();
    g->burn.nbrcyclp = 0;
    agc_rendz_erasable()->delveet3 = agc_erase_vgtig();          /* VGTEMP      */
    g->burn.nbrcycls = -1;                                       /* ENDS40.9 -2 */
    (void)AGC_ASSERT(g->burn.nbrcyclp == 0);
    return agc_phase_kill(1);
}

/* S40.13 job (p.726-728): TIMEBURN with the present mass, then group 3 off.
 * +X translation: channel 31 bit 7 reads 0 (inverted input).              */
agc_status_t agc_job_s40_13_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    const agc_csm_dapcom_t *com = agc_csm_dap_com();
    g->burn.weight_g = agc_dp_from_sp(com->weight_g);
    bool ullage = (agc_chan_read(031) & (uint16_t)AGC_CH031_PLUSX) == 0U;
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = agc_guid_s40_13_csm(&ip, ullage, !agc_flag_test(AGC_FLAG_NJETSFLG),
                                          com->emdot, g->csm.fang);
    (void)agc_phase_change(3, (uint16_t)000003);      /* KILL GROUP 3           */
    return st;
}
