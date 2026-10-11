/*
 * target_delta_v.c -- P76 TARGET DELTA V PROGRAM, both vehicles (P76.agc,
 * Comanche055 pp.511-513, Luminary099 the same with the noted changes).
 *
 * "Upon entry by astronaut action, P76 flashes DSKY requests to the
 * astronaut to provide via DSKY (1) the delta V to be applied to the
 * other vehicle state vector and (2) the time (TIG) at which the other
 * vehicle velocity was changed by execution of a thrusting maneuver. The
 * other vehicle state vector is integrated to TIG and updated by the
 * addition of delta V (delta V having been transformed from LV to REF
 * cosys). Using INTEGRVS, the program then integrates the other vehicle
 * state vector to the state vector of this vehicle, thus insuring that
 * the W-matrix and both vehicle states correspond to the same time."
 *
 * Vehicle differences: the LM first copies DELVLVC into DELVOV (noun 84);
 * P76SUB1 sets MOONFLAG from X2 (CM) or from CMOONFLG (LM, X2 = 2 or 0)
 * and the CM calls it again after INTEGRVS; ENDP76 of the CM also clears
 * VHFCNT and invalidates MRKBUF2.
 *
 * Scales: DELVOV m/cs 2^7 (local vertical: x along the horizontal, z
 * down), RATT m 2^29, VATT m/cs 2^7, RCV VCV body scale.
 */
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"

/* PHASCHNG OCT 04024: group 4, phase 2, type A. */
#define TDV_PHASE_04024 ((uint16_t) 04024)

/* The other-vehicle state at TIG between the jobs of P76 (the AGC keeps
 * it in the push list across the INTSTALL sleeps).                      */
typedef struct tdv_work
{
    agc_vec_t ratt; /* RATT at TIG, m 2^29                            */
    agc_vec_t vnew; /* VATT + DELTA V (push cell 6), m/cs 2^7         */
    int       x2;   /* X2: 0 earth, 2 moon                            */
} tdv_work_t;

static tdv_work_t tdv_work;

/* COMPMAT (p.511-512): the LV -> reference matrix of the state (RATT,
 * VATT), rows U((R x V) x R), U(V x R), U(-R); v_new = VATT + DELVOV M.  */
agc_status_t agc_target_delta_v_lv_to_ref(agc_interp_t *ip, agc_vec_t ratt,
                                          agc_vec_t vatt, agc_vec_t dv_lv,
                                          agc_vec_t *v_new)
{
    if (!AGC_ASSERT(ip != NULL && v_new != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t m;

    m.row[2] = agc_vec_neg(rz_unit(ip, ratt));                     /* 24D */
    m.row[1] = rz_unit(ip, agc_vec_cross(ip, m.row[2], vatt));     /* 18D */
    m.row[0] = rz_unit(ip, agc_vec_cross(ip, m.row[1], m.row[2])); /* 12D */
    agc_vec_t dv = agc_vec_shl(ip, agc_vec_mat(ip, dv_lv, &m), 1); /* VSL1 */

    *v_new = agc_vec_add(ip, dv, vatt);

    if (!AGC_ASSERT(m.row[2].c[0] != AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* P76SUB1: MOONFLAG for the integration. CM: X2 = 0 clear, 2 set. LM:
 * X2 = 2 and MOONFLAG set when CMOONFLG is set, else X2 = 0, clear.    */
static void tdv_moonflag(tdv_work_t *w)
{
    if (!rz_is_csm())
    {
        w->x2 = agc_flag_test(AGC_FLAG_CMOONFLG) ? 2 : 0;
    }

    rz_flag_put(AGC_FLAG_MOONFLAG, w->x2 != 0);

    if (!AGC_ASSERT(w->x2 == 0 || w->x2 == 2))
    {
        return;
    }
}

/* P76 TARGET DELTA V (P76.agc p.512, label P76): UPFLAG TRACKFLG; LM:
 * DELVOV = DELVLVC; V06N84 flashing (TERMINATE -> ENDP76, PROCEED ->
 * V06N33, ENTER -> the display again).                                 */
agc_status_t agc_job_target_delta_v(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_TRACKFLG);

    if (!rz_is_csm())
    {
        agc_rendz_t *rz = agc_rendz_erasable();

        rz->delvov = rz->delvlvc;
    }

    return (agc_job_target_delta_v_display(ctx));
}

/* V06N84 (DELVOV) flashing; ENTER ("TC -5") shows it again. */
agc_status_t agc_job_target_delta_v_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 84U, AGC_JOB_TARGET_DELTA_V_END,
                     AGC_JOB_TARGET_DELTA_V_TIG,
                     AGC_JOB_TARGET_DELTA_V_DISPLAY));
}

/* V06N33 (TIG) flashing; PROCEED -> the update. */
agc_status_t agc_job_target_delta_v_tig(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 33U, AGC_JOB_TARGET_DELTA_V_END,
                     AGC_JOB_TARGET_DELTA_V_UPDATE,
                     AGC_JOB_TARGET_DELTA_V_TIG));
}

/* TDEC1 = TIG, OTHPREC; COMPMAT (VATT + DELVOV in reference axes). An
 * INTSTALL sleep inside OTHPREC resumes this job from its start.       */
agc_status_t agc_job_target_delta_v_update(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_integ_out_t o;
    agc_status_t    st = rz_integ(&ip, RZ_OTHPREC, agc_erase_tig(),
                                  AGC_JOB_TARGET_DELTA_V_UPDATE, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    tdv_work.ratt = o.ratt;
    tdv_work.x2   = o.x2;
    st            = agc_target_delta_v_lv_to_ref(&ip, o.ratt, o.vatt,
                                                 agc_rendz_erasable()->delvov,
                                                 &tdv_work.vnew);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_target_delta_v_integrate(ctx));
}

/* INTSTALL, P76SUB1; RCV VCV = the updated state at body scale (VSR*
 * 0,2), TET = TIG, CLEAR INTYPFLG, TDEC1 = TETTHIS; INTEGRVS.          */
agc_status_t agc_job_target_delta_v_integrate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    tdv_moonflag(&tdv_work);
    rz_integrvs_in_t in;

    in.moon = (tdv_work.x2 != 0);
    /* VSR* 0,2: count 0 - X2, i.e. shift left 2 about the moon (2^29 ->
     * 2^27, 2^7 -> 2^5).                                                */
    in.vcv   = agc_vec_shl(&ip, tdv_work.vnew, tdv_work.x2);
    in.rcv   = agc_vec_shl(&ip, tdv_work.ratt, tdv_work.x2);
    in.tet   = agc_erase_tig();
    in.conic = false;
    in.tdec1 = rz_is_csm() ? agc_erase_tetcsm() : agc_erase_tetlem();
    agc_integ_out_t o;
    agc_status_t    st =
        rz_integrvs(&ip, &in, AGC_JOB_TARGET_DELTA_V_INTEGRATE, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    tdv_work.x2 = o.x2; /* RATT1 VATT1 TAT stay in the integration
                         * output, as in the AGC erasables         */

    return (agc_job_target_delta_v_store(ctx));
}

/* INTSTALL (CM: P76SUB1 again); RRECT = RCV = RATT1, TET = TAT,
 * MINIRECT(VATT1); PHASCHNG 04024; UPFLAG REINTFLG; ATOPOTH; INTWAKE1;
 * ENDP76.                                                              */
agc_status_t agc_job_target_delta_v_store(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_integ_install(0, AGC_JOB_TARGET_DELTA_V_STORE);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* asleep        */
    }

    if (rz_is_csm())
    {
        tdv_moonflag(&tdv_work);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_integ_ctx_t *ic = agc_integ_shared_ctx();

    ic->sv.rrect = ic->out.ratt1;
    ic->sv.rcv   = ic->out.ratt1;
    ic->sv.tet   = ic->out.tat;
    agc_integ_minirect(ic, ic->out.vatt1);
    agc_phase_change(4, TDV_PHASE_04024);
    agc_flag_set(AGC_FLAG_REINTFLG);
    st = rz_is_csm() ? agc_integ_atoplem(&ip, ic, tdv_work.x2) /* ATOPOTH */
         :
         agc_integ_atopcsm(&ip, ic, tdv_work.x2);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_integ_intwake0(); /* INTWAKE1      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_target_delta_v_end(ctx));
}

/* ENDP76: MARKCTR = 0 (CM also VHFCNT = 0, MRKBUF2 = -1), GOTOPOOH. */
agc_status_t agc_job_target_delta_v_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_trkmkcnt(0); /* MARKCTR       */

    if (rz_is_csm())
    {
        agc_erase_set_vhfcnt(0);
        agc_erase_set_mrkbuf2(0, -1); /* NEGONE        */
    }

    return (agc_exec_gotopooh());
}
