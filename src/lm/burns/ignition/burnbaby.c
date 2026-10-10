/*
 * burnbaby.c -- BURN, BABY, BURN -- MASTER IGNITION ROUTINE (Luminary099
 * BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc pp. 731-751, by Adler and
 * Eyles): everything between the 45-second pre-ignition check and TIG + 26 s
 * for P12, P40, P41, P42, P63 and the aborts: the ignition tables, BURNBABY,
 * the precomputation at P41SPOT (state integrated to TIG - 30 s), the
 * TIG-35 / TIG-30 / TIG-5 / TIG-0 waitlist chain, ullage, IGNITION and the
 * program-specific ignition tails, ZOOM (throttle-up), TIGTASK.
 * The once-a-second display (CLOKTASK/CLOKJOB), the crew responses,
 * COMFAIL, P40AUTO and REP40ALM are in clock.c.
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "agc/agc_guid.h"
#include "internal.h"

#define SEC1        100
#define SEC5        500       /* 5SEC                                      */
#define SEC4_9      490       /* 4.9SEC DEC 490                            */
#define SEC24_9     2490      /* S24.9SEC DEC 2490                         */
#define D29_9SEC    2990      /* D29.9SEC 2DEC 2990                        */
#define ULLAGE_OFF  50        /* DEC 50: ullage off 0.5 s after light-up   */
#define BIT13       010000

/* NOLI SE TANGERE (p.732-734). P41TABLE starts at entry (5) (P41ADRES =
 * P41TABLE -5); its entries (0)-(4), (13), (14) are never used.            */
static const ign_table_t IGN_TABLES[AGC_LM_WHICH_COUNT] = {
    /* P12TABLE */
    { 074, false, true, IGN_ENTER_GOCUTOFF, true, 0, AGC_JOB_SERVEXIT,
      IGN_TIG5_DISPCHNG, false, IGN_KIND_ASCENT, AGC_JOB_NONE },
    /* P40TABLE */
    { 040, false, false, IGN_ENTER_GOPOST, true, 2240, AGC_JOB_STEERING,
      IGN_TIG5_TIMEBURN, false, IGN_KIND_DPS, AGC_JOB_STAGE_ALARM_DISPLAY },
    /* P41TABLE */
    { 0, false, false, IGN_ENTER_GOPOST, false, -1, AGC_JOB_CALC_VG_BODY,
      IGN_TIG5_COMMON, true, IGN_KIND_NONE, AGC_JOB_NONE },
    /* P42TABLE */
    { 040, true, false, IGN_ENTER_GOPOST, true, 2640, AGC_JOB_STEERING,
      IGN_TIG5_TIMEBURN, false, IGN_KIND_APS, AGC_JOB_APS_BURN_STAGE_CHECKED },
    /* P63TABLE */
    { 062, false, true, IGN_ENTER_RECYCLE, false, 2240, AGC_JOB_SERVEXIT,
      IGN_TIG5_DISPCHNG, false, IGN_KIND_BRAKING, AGC_JOB_NONE },
    /* ABRTABLE: (5)-(7) are NOOPs, so B*RNB*B* falls into P40SPOT.        */
    { 063, false, true, IGN_ENTER_GOCUTOFF, true, 0, AGC_JOB_NONE,
      IGN_TIG5_DISPCHNG, false, IGN_KIND_ABORT, AGC_JOB_NONE }
};

const ign_table_t *ign_table(void)
{
    agc_sp_t w = agc_guid_state()->lm.which;
    if (!AGC_ASSERT(w >= 0 && w < (agc_sp_t)AGC_LM_WHICH_COUNT)) {
        return &IGN_TABLES[AGC_LM_WHICH_DPS_BURN];
    }
    return &IGN_TABLES[w];
}

/* NOULLAGE / ONULLAGE (p.745): ullage request bit of DAPBOOLS (ULLAGFLG).
 * Called in a task or under INHINT.                                        */
agc_status_t agc_lm_noullage(void)
{
    (void)AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM);
    return agc_flag_clear(AGC_FLAG_ULLAGFLG);
}

agc_status_t agc_lm_onullage(void)
{
    (void)AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM);
    return agc_flag_set(AGC_FLAG_ULLAGFLG);
}

/* BURNBABY (p.735): group 4 restarts here; DVTOTAL zeroed; P40AUTO (crew
 * to PGNCS auto), which returns to B*RNB*B*.                              */
agc_status_t agc_lm_burnbaby(const agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_BURNBABY, AGC_TASK_NONE,
                          (int16_t)ctx->prio, false, 0U };
    (void)agc_phase_change_var((uint16_t)004024, &v);
    agc_guid_state()->avg.dvtotal = 0;                /* EXTIRPATE JUNK         */
    return agc_lm_check_auto_control(ctx, AGC_JOB_BRNBB);
}

agc_status_t agc_job_burnbaby(agc_job_ctx_t *ctx)
{
    return agc_lm_burnbaby(ctx);
}

/* B*RNB*B* (p.735): GOBLTIME = TIG (oblateness, P70/P71), engine off
 * (ENGINOF3); (5): P40SPOT starts the countdown clock, then P41SPOT.     */
agc_status_t agc_lm_brnbb(const agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_state()->lm.gobltime = agc_erase_tig();
    agc_inhint();
    (void)agc_lm_enginof3();
    agc_relint();
    if (ign_table()->start_clock) {                   /* P40SPOT: STCLOK2       */
        (void)ign_start_clock((agc_sp_t)-IGN_CNTDNDEX, AGC_JOB_IGN_INTEGRATE);
    }
    return agc_exec_job_yield(AGC_JOB_IGN_INTEGRATE); /* P41SPOT                */
}

agc_status_t agc_job_brnbb(agc_job_ctx_t *ctx)
{
    return agc_lm_brnbb(ctx);
}

/* P41SPOT, CSM part (p.736): with MUNFLAG the CSM state at TDEC1 in SM
 * coordinates for MUNRVG: V(CSM) = REFSMMAT VATT1 VSR1 (2^7), R(CSM) =
 * REFSMMAT RATT1 VSL4 (2^24), G(CSM) = MUNGRAV; TDEC1 = TAT.              */
static agc_status_t csm_state_for_munrvg(agc_interp_t *ip, agc_job_id_t self,
                                         agc_dp_t *tdec1)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm;
    agc_guid_prec_t p;
    agc_status_t st = agc_guid_prec_update(ip, AGC_VEHICLE_CSM, *tdec1, self, &p);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_mat_t refsmmat = agc_erase_refsmmat();
    lm->v_csm = agc_vec_shr(agc_mat_vec(ip, &refsmmat, p.vatt1), 1);
    lm->r_csm = agc_mat_vec(ip, &refsmmat, agc_vec_shl(ip, p.ratt1, 4));
    lm->g_csm = agc_lm_mungrav(ip, lm->r_csm);
    *tdec1 = p.tat;                                   /* RELOAD TDEC1           */
    (void)AGC_ASSERT(p.rtx2 == 2);
    return AGC_STATUS_OK;
}

/* P41SPOT .. CALLT-35 (p.736-737), a job: TDEC1 = TIG - 29.9 s, INITCDUW,
 * the CSM state (MUNFLAG), MIDTOAV1; a slipped TIG becomes PIPTIME1 +
 * 29.9 s. TIG-35 by LONGCALL (4.25); in P63 the countdown display starts
 * now and ABVEL = |VN1|.                                                   */
agc_status_t agc_job_ign_integrate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_guid_t *g = agc_guid_state();
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_dp_t tdec1 = agc_dp_sub(&ip, agc_erase_tig(), D29_9SEC);
    (void)agc_lm_initcduw();
    agc_status_t st = AGC_STATUS_OK;
    if (agc_flag_test(AGC_FLAG_MUNFLAG)) {
        st = csm_state_for_munrvg(&ip, ctx->id, &tdec1);
    }
    agc_dp_t dt = 0;
    bool late = false;
    if (st == AGC_STATUS_OK) {
        st = agc_guid_midtoav1(&ip, tdec1, ctx->id, &dt, &late);   /* GOMIDAV */
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (late) {                                       /* TIG WAS SLIPPED        */
        agc_erase_set_tig(agc_dp_add(&ip, g->avg.piptime1, D29_9SEC));
    }
    g->lm.savet_30 = agc_dp_sub(&ip, dt, SEC5);       /* DELTA-T UNTIL TIG-35   */
    (void)AGC_ASSERT(g->lm.savet_30 >= 0);
    (void)agc_waitlist_longcall((uint32_t)((g->lm.savet_30 > 0) ? g->lm.savet_30 : 1),
                                AGC_TASK_TIG_35);
    (void)agc_phase_change(4, (uint16_t)020254);      /* 4.25SPOT               */
    if (agc_phase_checkmm(63)) {                      /* P63 DISPLAYS NOW       */
        g->lm.dispdex = (agc_sp_t)-IGN_CNTDNDEX;
        g->lm.abvel = agc_vec_abval(&ip, g->avg.vn1);
    }
    return AGC_STATUS_OK;                             /* ENDOFJOB               */
}

/* TIG-35 (p.737): TIG-30 in 5 s (4.15), DSKY blanked 5 s (average-G is
 * starting). A negative ullage time means P41: CLOKJOB is not running,
 * so blank by a job (P41BLANK) and restore the display at TIG-30 (4.9 s).*/
agc_status_t agc_task_tig_35(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm;
    (void)agc_waitlist_call(SEC5, AGC_TASK_TIG_30);
    (void)agc_phase_change(4, (uint16_t)040154);      /* 4.15SPOT               */
    lm->dispdex = (agc_sp_t)-IGN_BLANKDEX;
    if (ign_table()->ullage_cs >= 0) {                /* CS 6, BZMF TASKOVER    */
        return AGC_STATUS_OK;
    }
    (void)agc_waitlist_call(SEC4_9, AGC_TASK_RESTORE_BURN_DISPLAY);
    return agc_exec_novac(AGC_PRIO17, AGC_JOB_RCS_BURN_BLANK_DSKY);
}

/* TIG-30.1 (p.738): restore the display by a job (TIG-30A).                */
agc_status_t agc_task_restore_burn_display(void)
{
    (void)AGC_ASSERT(ign_table()->ullage_cs < 0);
    return agc_exec_novac(AGC_PRIO17, AGC_JOB_TIG_30A);
}

/* ULLGNOT .. REDO4.2 (p.739): AVEGEXIT from the table, 4.2 (restart at
 * REDO4.2 and TIG-5 at its time), group 2 cleared for Lambert, PREREAD
 * unless SERVICER is already running (PHASE5).                           */
static agc_status_t ullgnot(void)
{
    agc_guid_state()->avg.avgexit = ign_table()->avgexit;
    (void)agc_phase_newphase(4, 2, true);             /* 4.2SPOT, TBASE4        */
    (void)agc_phase_kill(2);                          /* REDO2.17               */
    return agc_task_redo4_2();
}

agc_status_t agc_task_redo4_2(void)
{
    if (agc_phase_get(5) != 0U) {                     /* SERVICER GOING?        */
        return AGC_STATUS_OK;
    }
    (void)AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM);
    return agc_task_preread_lm();
}

/* TIG-30 (p.738-739): TIG-5 in 24.9 s, countdown display on; ullage on
 * at TIG - (6) (1.3); (1) WANTAPS sets APSFLAG for P42; then ULLGNOT.    */
agc_status_t agc_task_tig_30(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm;
    const ign_table_t *t = ign_table();
    (void)agc_waitlist_call(SEC24_9, AGC_TASK_TIG_5);
    lm->dispdex = (agc_sp_t)-IGN_CNTDNDEX;            /* START UP CLOKTASK      */
    if (t->ullage_cs > 0) {
        lm->savet_30 = (agc_dp_t)t->ullage_cs;        /* FOR RESTART            */
        (void)agc_waitlist_call((uint16_t)t->ullage_cs, AGC_TASK_ULLGTASK);
        (void)agc_phase_newphase(1, 3, true);         /* 1.3SPOT, TBASE1        */
        if (t->want_aps) {
            (void)agc_flag_set(AGC_FLAG_APSFLAG);     /* WANTAPS                */
        }
    }
    return ullgnot();
}

/* ULLGTASK (p.739): ullage on (TIG-7.5 s or TIG-3.5 s), group 1 off.        */
agc_status_t agc_task_ullgtask(void)
{
    (void)agc_lm_onullage();
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_ULLAGFLG));
    return agc_phase_change(1, (uint16_t)000001);
}

/* TIG-5 (p.739-740): group 3 off, TIG-0 in 5 s, IGNFLAG and ASTNFLAG
 * reset; (11): S40.13 (P40SJUNK, 3.5) unless running; V99 request
 * (DISPCHNG); TIG-0 restart point 4.7 (COMMON).                            */
agc_status_t agc_task_tig_5_lm(void)
{
    const ign_table_t *t = ign_table();
    (void)agc_phase_kill(3);
    (void)agc_waitlist_call(SEC5, AGC_TASK_TIG_0);
    (void)agc_flag_clear(AGC_FLAG_IGNFLAG);
    (void)agc_flag_clear(AGC_FLAG_ASTNFLAG);
    if (t->tig5 == IGN_TIG5_TIMEBURN && agc_phase_get(3) == 0U) {
        (void)agc_exec_findvac(AGC_PRIO20, AGC_JOB_TIMEBURN);
        (void)agc_phase_change(3, (uint16_t)000053);  /* 3.5SPOT FOR S40.13     */
    }
    if (t->tig5 != IGN_TIG5_COMMON) {
        agc_guid_state()->lm.dispdex = (agc_sp_t)-IGN_VB99DEX;   /* DISPCHNG    */
    }
    return agc_phase_change(4, (uint16_t)040074);     /* 4.7SPOT                */
}

/* IGNYET? (p.740): the crew has answered the V99 (ASTNFLAG): IGNITION;
 * otherwise (12): TIGTASK (P41) or WAITABIT (wait for *PROCEED).          */
agc_status_t agc_task_ignyet(void)
{
    if (agc_flag_test(AGC_FLAG_ASTNFLAG)) {
        return agc_task_ignition_lm();
    }
    if (ign_table()->tigtask) {
        return agc_task_tigtask();
    }
    return agc_phase_kill(4);                         /* WAITABIT               */
}

/* TIG-0 (p.740): IGNFLAG set; in P63 throttle-up comes ZOOMTIME after
 * the nominal ignition (P63ZOOM, 3.3, type C restart here).                */
agc_status_t agc_task_tig_0_lm(void)
{
    (void)agc_flag_set(AGC_FLAG_IGNFLAG);
    if (agc_phase_checkmm(63)) {
        agc_sp_t zoom = agc_guid_state()->lm.zoomtime;
        (void)AGC_ASSERT(zoom > 0);
        (void)agc_waitlist_call((uint16_t)zoom, AGC_TASK_BRAKING_PHASE_ZOOM);
        agc_phase_var_t v = { AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_IGNYET, 0, true, 0U };
        (void)agc_phase_change2((uint16_t)040033, (uint16_t)005014, &v);
    }
    return agc_task_ignyet();
}
