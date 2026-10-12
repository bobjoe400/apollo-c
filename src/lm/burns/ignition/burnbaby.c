/*
 * burnbaby.c -- BURN, BABY, BURN -- MASTER IGNITION ROUTINE (Luminary099
 * BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc pp. 731-751, by Adler and
 * Eyles): everything between the 45-second pre-ignition check and TIG + 26 s
 * for P12, P40, P41, P42, P63 and the aborts: the ignition tables, BURNBABY,
 * the precomputation at P41SPOT (state integrated to TIG - 30 s), the
 * TIG-35 / TIG-30 / TIG-5 / TIG-0 waitlist chain, ullage, IGNITION and the
 * program-specific ignition tails, ZOOM (throttle-up), TIGTASK.
 * The once-a-second display (CLOKTASK/CLOKJOB), STCLOK, NULLCLOK, the
 * crew responses, COMFAIL, P40AUTO and REP40ALM are in clock.c.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

#define SEC1       100
#define SEC5       500  /* 5SEC                                      */
#define SEC4_9     490  /* 4.9SEC DEC 490                            */
#define SEC24_9    2490 /* S24.9SEC DEC 2490                         */
#define D29_9SEC   2990 /* D29.9SEC 2DEC 2990                        */
#define ULLAGE_OFF 50   /* DEC 50: ullage off 0.5 s after light-up   */
#define BIT13      010000

/* NOLI SE TANGERE (p.732-734). Entry (0) is VN 06xx; the noun is kept as
 * the decimal DSKY noun. P41TABLE starts at entry (5) (P41ADRES =
 * P41TABLE -5); its entries (0)-(4), (13), (14) are never used.            */
static const ign_table_t IGN_TABLES[AGC_LM_WHICH_COUNT] = {
    /* P12TABLE */
    {74, false, true, IGN_ENTER_GOCUTOFF, true, 0, AGC_JOB_SERVEXIT,
     IGN_TIG5_DISPCHNG, false, IGN_KIND_ASCENT, AGC_JOB_NONE},

    /* P40TABLE */
    {40, false, false, IGN_ENTER_GOPOST, true, 2240, AGC_JOB_STEERING,
     IGN_TIG5_TIMEBURN, false, IGN_KIND_DPS, AGC_JOB_STAGE_ALARM_DISPLAY},

    /* P41TABLE */
    {0, false, false, IGN_ENTER_GOPOST, false, -1, AGC_JOB_CALC_VG_BODY,
     IGN_TIG5_COMMON, true, IGN_KIND_NONE, AGC_JOB_NONE},

    /* P42TABLE */
    {40, true, false, IGN_ENTER_GOPOST, true, 2640, AGC_JOB_STEERING,
     IGN_TIG5_TIMEBURN, false, IGN_KIND_APS, AGC_JOB_APS_BURN_STAGE_CHECKED},

    /* P63TABLE */
    {62, false, true, IGN_ENTER_RECYCLE, false, 2240, AGC_JOB_SERVEXIT,
     IGN_TIG5_DISPCHNG, false, IGN_KIND_BRAKING, AGC_JOB_NONE},

    /* ABRTABLE (P70/P71, which do not enter BURNBABY): (5)-(7) are NOOPs. */
    {63, false, true, IGN_ENTER_GOCUTOFF, true, 0, AGC_JOB_NONE,
     IGN_TIG5_DISPCHNG, false, IGN_KIND_ABORT, AGC_JOB_NONE}
};

const ign_table_t *ign_table(void)
{
    agc_sp_t w = agc_guid_state()->lm.which;

    return (&IGN_TABLES[w]);
}

/* NOULLAGE / ONULLAGE (p.745): ullage request bit of DAPBOOLS (ULLAGFLG).
 * Called in a task or under INHINT.                                        */
void agc_lm_noullage(void)
{
    _Static_assert(AGC_FLAG_ULLAGFLG != AGC_FLAG_DRIFTDFL, "AGC constant");
    agc_flag_clear(AGC_FLAG_ULLAGFLG);
}

void agc_lm_onullage(void)
{
    agc_flag_set(AGC_FLAG_ULLAGFLG);
}

/* BURNBABY (p.735): group 4 restarts here; DVTOTAL zeroed; P40AUTO (crew
 * to PGNCS auto), which returns to B*RNB*B*.                              */
agc_status_t agc_lm_burnbaby(const agc_job_ctx_t *ctx)
{
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_BURNBABY, AGC_TASK_NONE, (int16_t) ctx->prio, false, 0U};

    agc_phase_change_var((uint16_t) 004024, &v);
    agc_guid_state()->avg.dvtotal = 0; /* EXTIRPATE JUNK         */

    return (agc_lm_check_auto_control(ctx, AGC_JOB_BURNBABY_COUNTDOWN));
}

agc_status_t agc_job_burnbaby(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx == NULL || ctx->id == AGC_JOB_BURNBABY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_burnbaby(ctx));
}

/* B*RNB*B* (p.733): GOBLTIME = TIG (oblateness, P70/P71), engine off
 * (ENGINOF3); (5): P40SPOT starts the countdown clock (STCLOK2), then
 * P41SPOT (P63SPOT = P41SPOT: in P63 CLOKTASK is already going).          */
agc_status_t agc_lm_burnbaby_countdown(void)
{
    agc_guid_state()->lm.gobltime = agc_erase_tig();
    agc_inhint();
    agc_lm_enginof3();
    agc_relint();

    if (ign_table()->start_clock)
    { /* P40SPOT: STCLOK2       */
        agc_guid_state()->lm.dispdex = (agc_sp_t) -IGN_CNTDNDEX;
        agc_status_t st              = ign_start_clock(AGC_JOB_PREBURN_STATE_VECTOR);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (agc_exec_job_yield(AGC_JOB_PREBURN_STATE_VECTOR)); /* P41SPOT         */
}

agc_status_t agc_job_burnbaby_countdown(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_burnbaby_countdown());
}

/* P41SPOT, CSM part (p.736): with MUNFLAG the CSM state at TDEC1 in SM
 * coordinates for MUNRVG: V(CSM) = REFSMMAT VATT1 VSR1 (2^7), R(CSM) =
 * REFSMMAT RATT1 VSL4 (2^24), G(CSM) = MUNGRAV; TDEC1 = TAT.              */
static agc_status_t csm_state_for_munrvg(agc_interp_t *ip, agc_job_id_t self, agc_dp_t *tdec1)
{
    agc_guid_lm_t   *lm = &agc_guid_state()->lm;
    agc_integ_ctx_t *ic = agc_integ_shared_ctx();
    agc_status_t     st = agc_integ_csmprec(ip, ic, *tdec1, self);

    if (st == AGC_STATUS_OK && !ic->out.valid)
    {
        st = AGC_STATUS_NOT_FOUND; /* no RECTOUT */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_integ_out_t *p        = &ic->out;
    agc_mat_t              refsmmat = agc_erase_refsmmat();

    lm->v_csm = agc_vec_shr(agc_mat_vec(ip, &refsmmat, p->vatt1), 1);
    lm->r_csm = agc_mat_vec(ip, &refsmmat, agc_vec_shl(ip, p->ratt1, 4));
    lm->g_csm = agc_lm_mungrav(ip, lm->r_csm);
    *tdec1    = p->tat; /* RELOAD TDEC1           */

    if (!AGC_ASSERT(p->x2 == 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* P41SPOT .. CALLT-35 (p.736-737), a job: TDEC1 = TIG - 29.9 s, INITCDUW,
 * the CSM state (MUNFLAG), MIDTOAV1; a slipped TIG becomes PIPTIME1 +
 * 29.9 s. TIG-35 by LONGCALL (4.25); in P63 the countdown display starts
 * now and ABVEL = |VN1|.                                                   */
agc_status_t agc_job_preburn_state_vector_lm(agc_job_ctx_t *ctx)
{
    agc_guid_t  *g = agc_guid_state();
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_dp_t tdec1 = agc_dp_sub(&ip, agc_erase_tig(), D29_9SEC);

    agc_lm_initcduw();
    agc_status_t st = AGC_STATUS_OK;

    if (agc_flag_test(AGC_FLAG_MUNFLAG))
    {
        st = csm_state_for_munrvg(&ip, ctx->id, &tdec1);
    }

    agc_integ_midtoav_out_t mo;

    if (st == AGC_STATUS_OK)
    {
        st = agc_integ_midtoav1(&ip, agc_integ_shared_ctx(), tdec1, ctx->id, &mo); /* GOMIDAV */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_store_midtoav(&mo);

    if (mo.slipped)
    { /* TIG WAS SLIPPED        */
        agc_erase_set_tig(agc_dp_add(&ip, g->avg.piptime1, D29_9SEC));
    }

    agc_erase_set_savet_30(agc_dp_sub(&ip, mo.dt, SEC5)); /* DELTA-T TO TIG-35 */

    if (!AGC_ASSERT(agc_erase_savet_30() >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t sw = agc_waitlist_longcall((uint32_t) ((agc_erase_savet_30() > 0) ? agc_erase_savet_30() : 1), AGC_TASK_TIG_35);

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_phase_change((uint16_t)020254); /* 4.25SPOT               */

    if (agc_phase_checkmm(63))
    { /* P63 DISPLAYS NOW       */
        g->lm.dispdex = (agc_sp_t) -IGN_CNTDNDEX;
        g->lm.abvel   = agc_vec_abval(&ip, g->avg.vn1);
    }

    return (AGC_STATUS_OK); /* ENDOFJOB               */
}

/* TIG-35 (p.737): TIG-30 in 5 s (4.15), DSKY blanked 5 s (average-G is
 * starting). A negative ullage time means P41: CLOKJOB is not running,
 * so blank by a job (P41BLANK) and restore the display at TIG-30 (4.9 s).*/
agc_status_t agc_task_tig_35(void)
{
    agc_guid_lm_t *lm = &agc_guid_state()->lm;
    agc_status_t   sw = agc_waitlist_call(SEC5, AGC_TASK_TIG_30);

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_phase_change((uint16_t)040154); /* 4.15SPOT               */
    lm->dispdex = (agc_sp_t) -IGN_BLANKDEX;

    if (ign_table()->ullage_cs >= 0)
    { /* CS 6, BZMF TASKOVER    */
        return (AGC_STATUS_OK);
    }

    agc_status_t st = agc_waitlist_call(SEC4_9, AGC_TASK_RESTORE_BURN_DISPLAY);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_exec_novac(AGC_PRIO17, AGC_JOB_RCS_BURN_BLANK_DSKY));
}

/* TIG-30.1 (p.738): restore the display by a job (TIG-30A).                */
agc_status_t agc_task_restore_burn_display(void)
{
    if (!AGC_ASSERT(ign_table()->ullage_cs < 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_novac(AGC_PRIO17, AGC_JOB_RESTORE_BURN_DISPLAY));
}

/* ULLGNOT .. REDO4.2 (p.739): AVEGEXIT from the table, 4.2 (restart at
 * REDO4.2 and TIG-5 at its time), group 2 cleared for Lambert, PREREAD
 * unless SERVICER is already running (PHASE5).                           */
static agc_status_t ullgnot(void)
{
    agc_guid_state()->avg.avgexit = ign_table()->avgexit;

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_newphase(4, 2, true); /* 4.2SPOT, TBASE4        */

    return (agc_task_redo2_17());
}

/* REDO2.17 (p.736-737, restart 2.17): clear group 2 so that LAMBERT can
 * start if needed, then REDO4.2.                                          */
agc_status_t agc_task_redo2_17(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_kill(2); /* DCA NEG0 DXCH -PHASE2  */

    return (agc_task_redo4_2());
}

agc_status_t agc_task_redo4_2(void)
{
    if (agc_phase_get(5) != 0U)
    { /* SERVICER GOING?        */
        return (AGC_STATUS_OK);
    }

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_preread_lm());
}

/* TIG-30 (p.738-739): TIG-5 in 24.9 s, countdown display on; ullage on
 * at TIG - (6) (1.3); (1) WANTAPS sets APSFLAG for P42; then ULLGNOT.    */
agc_status_t agc_task_tig_30(void)
{
    agc_guid_lm_t     *lm = &agc_guid_state()->lm;
    const ign_table_t *t  = ign_table();

    if (!AGC_ASSERT(t->ullage_cs <= SEC5 * 6))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t sw = agc_waitlist_call(SEC24_9, AGC_TASK_TIG_5);

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    lm->dispdex = (agc_sp_t) -IGN_CNTDNDEX; /* START UP CLOKTASK      */

    if (t->ullage_cs > 0)
    {
        agc_erase_set_savet_30((agc_dp_t) t->ullage_cs); /* FOR RESTART */
        agc_status_t st = agc_waitlist_call((uint16_t) t->ullage_cs, AGC_TASK_ULLGTASK);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        agc_phase_newphase(1, 3, true); /* 1.3SPOT, TBASE1        */

        if (t->want_aps)
        {
            agc_flag_set(AGC_FLAG_APSFLAG); /* WANTAPS                */
        }
    }

    return (ullgnot());
}

/* ULLGTASK (p.739): ullage on (TIG-7.5 s or TIG-3.5 s), group 1 off.        */
agc_status_t agc_task_ullgtask(void)
{
    agc_lm_onullage();

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_ULLAGFLG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change((uint16_t)000001);

    return (AGC_STATUS_OK);
}

/* TIG-5 (p.739-740): group 3 off, TIG-0 in 5 s, IGNFLAG and ASTNFLAG
 * reset; (11): S40.13 (P40SJUNK, 3.5) unless running; V99 request
 * (DISPCHNG); TIG-0 restart point 4.7 (COMMON).                            */
agc_status_t agc_task_tig_5_lm(void)
{
    const ign_table_t *t = ign_table();

    agc_phase_kill(3);
    agc_status_t sw = agc_waitlist_call(SEC5, AGC_TASK_TIG_0);

    if (sw != AGC_STATUS_OK)
    {
        return (sw);
    }

    agc_flag_clear(AGC_FLAG_IGNFLAG);
    agc_flag_clear(AGC_FLAG_ASTNFLAG);

    if (t->tig5 == IGN_TIG5_TIMEBURN && agc_phase_get(3) == 0U)
    {
        agc_status_t st = agc_exec_findvac(AGC_PRIO20, AGC_JOB_TIMEBURN);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        agc_phase_change((uint16_t)000053); /* 3.5SPOT FOR S40.13     */
    }

    if (t->tig5 != IGN_TIG5_COMMON)
    {
        agc_guid_state()->lm.dispdex = (agc_sp_t) -IGN_VB99DEX; /* DISPCHNG    */
    }

    agc_phase_change((uint16_t)040074);

    return (AGC_STATUS_OK); /* 4.7SPOT                */
}

/* IGNYET? (p.738): the crew has answered the V99 (ASTNFLAG): IGNITION;
 * otherwise (12): TIGTASK (P41) or WAITABIT (wait for *PROCEED).          */
agc_status_t agc_task_ignition_if_enabled(void)
{
    if (agc_flag_test(AGC_FLAG_ASTNFLAG))
    {
        return (agc_task_ignition_lm());
    }

    if (ign_table()->tigtask)
    {
        return (agc_task_tigtask());
    }

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_ASTNFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_kill(4);

    return (AGC_STATUS_OK); /* WAITABIT               */
}

/* TIG-0 (p.740): IGNFLAG set; in P63 throttle-up comes ZOOMTIME after
 * the nominal ignition (P63ZOOM, 3.3, type C restart here).                */
agc_status_t agc_task_tig_0_lm(void)
{
    agc_flag_set(AGC_FLAG_IGNFLAG);

    if (agc_phase_checkmm(63))
    {
        agc_sp_t zoom = agc_erase_zoomtime();

        if (!AGC_ASSERT(zoom > 0))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_status_t st = agc_waitlist_call((uint16_t) zoom, AGC_TASK_BRAKING_PHASE_ZOOM);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        agc_phase_var_t v = {AGC_RESTART_TASK,
                             AGC_JOB_NONE,
                             AGC_TASK_IGNITION_IF_ENABLED,
                             0,
                             true,
                             0U};

        agc_phase_change2((uint16_t) 040033, (uint16_t) 005014, &v);
    }

    return (agc_task_ignition_if_enabled());
}

/* P12IGN (p.740): DAP bias acceleration estimates AOSQ, AOSR = the pad
 * loads IGNAOSQ, IGNAOSR (upper words only: CA / TS), then ABRTIGN.
 * ABRTIGN: CLOKTASK killed, AVEGEXIT = ATMAG (ascent guidance), R10 on.  */
static void ascent_ignition(bool load_bias)
{
    agc_guid_t *g = agc_guid_state();

    if (load_bias)
    {
        agc_erase_set_aosq(agc_dp_pack(g->lm.ignaosq, agc_dp_low(agc_erase_aosq())));
        agc_erase_set_aosr(agc_dp_pack(g->lm.ignaosr, agc_dp_low(agc_erase_aosr())));
    }

    g->lm.dispdex  = IGN_DISPDEX_KILL; /* ABRTIGN: KILL CLOKTASK */
    g->avg.avgexit = AGC_JOB_ATMAG;    /* ATMAGADR               */
    agc_flag_set(AGC_FLAG_SWANDISP);   /* ENABLE R10             */
}

/* INDEX WHICH / TCF 13 (p.739): the program-specific part of IGNITION.   */
static agc_status_t ignition_by_program(ign_kind_t kind)
{
    switch (kind)
    {
        case IGN_KIND_BRAKING:
            /* P63IGN (p.739): burn displays (AVEGEXIT = P63DISPS -2), CLOKTASK killed,
             * LETABORT (P70/P71 allowed), SWANDISP (R10 on), DAP out of minimum-impulse
             * mode (in case of P66), TIG = TIME for P70/P71, WCHPHASE = WCHPHOLD = 0,
             * FLPASS0 = 2.                                                            */
            {
                agc_guid_t *g = agc_guid_state();

                g->avg.avgexit    = AGC_JOB_BRAKING_PHASE_DISPLAY; /* DSP2CADR               */
                g->lm.dispdex     = IGN_DISPDEX_KILL;              /* ASSASSINATE CLOKTASK   */
                agc_flag_set(AGC_FLAG_LETABORT);
                agc_flag_set(AGC_FLAG_SWANDISP);
                agc_flag_clear(AGC_FLAG_PULSEFLG);
                agc_erase_set_tig((agc_dp_t) agc_time_now_cs());   /* TIG FOR P70 AND P71    */
                agc_lm_ldg_t *ldg = agc_lm_ldg();

                ldg->wchphase = 0;
                ldg->wchphold = 0;
                ldg->flpass0  = 2;
            }

            return (agc_task_ignition_finish());
        case IGN_KIND_DPS:
            /* P40IGN (p.739-740): unless NOTHROTL, throttle-up (P40ZOOM) ZOOMTIME
             * after ignition; P63IGN1: 3.3 for the ZOOM restart and 4.1 type C,
             * immediate, at P42IGN.                                                   */
            {
                if (agc_flag_test(AGC_FLAG_NOTHROTL))
                {
                    return (agc_task_ignition_finish());
                }

                agc_sp_t zoom = agc_erase_zoomtime();

                if (!AGC_ASSERT(zoom > 0))
                {
                    return (AGC_STATUS_BAD_ARG);
                }

                agc_status_t st = agc_waitlist_call((uint16_t) zoom, AGC_TASK_DPS_BURN_ZOOM);

                if (st != AGC_STATUS_OK)
                {
                    return (st);
                }

                agc_phase_var_t v = { AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_IGNITION_FINISH, 0, true, 0U };

                agc_phase_change2((uint16_t) 040033, (uint16_t) 005014, &v);

                return (agc_task_ignition_finish());
            }
        case IGN_KIND_ASCENT:
            ascent_ignition(true);

            return (agc_task_ignition_finish());
        case IGN_KIND_ABORT:
            ascent_ignition(false);

            return (agc_task_ignition_finish());
        case IGN_KIND_APS:
            return (agc_task_ignition_finish());
        case IGN_KIND_NONE:
        default:
            /* P41 has no entry (13): it never asks V99, so ASTNFLAG stays reset
             * and IGNYET? goes to TIGTASK instead.                            */
            return (AGC_STATUS_BAD_ARG); /* unreachable: cannot happen */
    }
}

/* IGNITION (p.738-739): ENGONFLG set, engine on (DSALMOUT bits 13, 14 =
 * 1, 0), TEVENT = TIME for the downlink, TIG = TIME + TGO (TGO from
 * S40.13), guidance displays permitted (FLUNDISP reset), then (13).      */
agc_status_t agc_task_ignition_lm(void)
{
    agc_flag_set(AGC_FLAG_ENGONFLG); /* INSURE ENGONFLG IS SET */
    uint16_t ch11 = agc_chan_read(011);

    ch11         = (uint16_t) ((ch11 & ~(uint16_t) (AGC_CH011_ENGINEON | AGC_CH011_ENGINEOFF)) | (uint16_t) AGC_CH011_ENGINEON);
    agc_chan_write(011, ch11); /* TURN ON THE ENGINE     */
    agc_dp_t now = (agc_dp_t) agc_time_now_cs();

    agc_erase_set_tevent(now);
    agc_dp_t tgo = agc_erase_tgo();

    if (!AGC_ASSERT(tgo >= 0 && tgo <= AGC_DP_MAX - now))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_tig(tgo + now); /* UPDATE TIG USING TGO   */
    agc_flag_clear(AGC_FLAG_FLUNDISP);

    return (ignition_by_program(ign_table()->kind));
}

/* P42IGN (p.740): powered-flight switching curves (DRIFTDFL reset); an
 * impulsive burn (IMPULSW) goes to IMPLBURN; otherwise DVMONCON: DV monitor
 * connected (IGNFLAG, ASTNFLAG, IDLEFLAG reset), 4.5, ullage off 0.5 s
 * after light-up.                                                        */
agc_status_t agc_task_ignition_finish(void)
{
    agc_flag_clear(AGC_FLAG_DRIFTDFL);

    if (agc_flag_test(AGC_FLAG_IMPULSW))
    {
        return (agc_task_implburn_lm());
    }

    agc_flag_clear(AGC_FLAG_IGNFLAG);   /* CONNECT DVMON          */
    agc_flag_clear(AGC_FLAG_ASTNFLAG);
    agc_flag_clear(AGC_FLAG_IDLEFLAG);
    agc_phase_change((uint16_t)040054); /* 4.5SPOT                */

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_IDLEFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_waitlist_fixdelay(ULLAGE_OFF, AGC_TASK_ULLAGOFF));
}

/* ULLAGOFF (p.740): ullage off; WAITABIT: group 4 off.                   */
agc_status_t agc_task_ullagoff(void)
{
    agc_lm_noullage();

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_ULLAGFLG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_kill(4);

    return (AGC_STATUS_OK); /* WAITABIT               */
}

/* TIGTASK / TIGTASK1 (p.741), P41 at TIG: TIGNOW at PRIO16, group 6 off. */
agc_status_t agc_task_tigtask(void)
{
    if (!AGC_ASSERT(ign_table()->tigtask))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_exec_novac(AGC_PRIO16, AGC_JOB_TIGNOW);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change((uint16_t)000006);

    return (AGC_STATUS_OK); /* KILL GROUP 6           */
}

/* P63ZOOM (p.741): AVEGEXIT = LUNLAND, FLATOUT (full thrust), ZOOM.       */
agc_status_t agc_task_braking_phase_zoom(void)
{
    agc_guid_state()->avg.avgexit = AGC_JOB_LUNLAND; /* LUNLANAD               */
    agc_status_t st               = agc_lm_throttle_flatout();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(agc_guid_state()->avg.avgexit == AGC_JOB_LUNLAND))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_zoom());
}

/* P40ZOOM (p.741): THRUST = BIT13 and channel 14 bit 4 (thrust drive):
 * throttle to full, then ZOOM.                                            */
agc_status_t agc_task_dps_burn_zoom(void)
{
    agc_erase_set_thrust((agc_sp_t) BIT13);
    agc_chan_or(014, (uint16_t) AGC_CH014_THRUSTACT);

    if (!AGC_ASSERT((agc_chan_read(014) & AGC_CH014_THRUSTACT) != 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_zoom());
}

/* ZOOM = P40ZOOMA (p.741-742): group 3 off. This is also the 3.3 restart
 * point, so after a restart only the phase is cleared: the throttle-up
 * itself is not repeated (Luminary 099 as flown).                        */
agc_status_t agc_task_zoom(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change((uint16_t)000003);

    return (AGC_STATUS_OK);
}

/* P41BLANK (p.735): blank the DSKY (CLOKJOB is not running in P41).       */
agc_status_t agc_job_rcs_burn_blank_dsky(agc_job_ctx_t *ctx)
{
    return (agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0U, NULL));
}

/* TIG-30A (p.736): restore the V16N85 display (REGODSP ends the job).    */
agc_status_t agc_job_restore_burn_display(agc_job_ctx_t *ctx)
{
    return (agc_dsky_display(ctx, AGC_DSKY_REGODSP, agc_dsky_vn(16, 85), NULL));
}
