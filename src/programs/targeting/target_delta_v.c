/**********************************************************************
 * FILE NAME: target_delta_v.c
 *
 * PURPOSE:
 *   P76 TARGET DELTA V PROGRAM, both vehicles: "Upon entry by astronaut
 *   action, P76 flashes DSKY requests to the astronaut to provide via
 *   DSKY (1) the delta V to be applied to the other vehicle state vector
 *   and (2) the time (TIG) at which the other vehicle velocity was
 *   changed by execution of a thrusting maneuver. The other vehicle
 *   state vector is integrated to TIG and updated by the addition of
 *   delta V (delta V having been transformed from LV to REF cosys).
 *   Using INTEGRVS, the program then integrates the other vehicle state
 *   vector to the state vector of this vehicle, thus insuring that the
 *   W-matrix and both vehicle states correspond to the same time."
 *
 * AGC SOURCE:
 *   Comanche055 P76.agc pp. 511-513 (P76, P76SUB1, COMPMAT, ENDP76)
 *   Luminary099 P76.agc (differs: DELVOV = DELVLVC first, P76SUB1 from
 *   CMOONFLG, no VHFCNT / MRKBUF2 at ENDP76)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  -------------------------------
 *   DELVOV     agc_vec_t  I/O  m/cs 2^7   delta V, local vertical (N84)
 *   TIG        agc_dp_t   I    cs 2^28    time of the other's burn (N33)
 *   TETCSM/LEM agc_dp_t   I    cs 2^28    this vehicle's state time
 *   RRECT RCV TET VCV     O    body scale other vehicle's new state
 *   TRKMKCNT VHFCNT MRKBUF2  O  -         cleared at ENDP76
 *   TRACKFLG MOONFLAG REINTFLG flags O -
 *
 * EXTERNAL REFERENCES:
 *   Name                 Description
 *   -------------------  ----------------------------------------------
 *   rz_integ (OTHPREC)   other vehicle to TIG
 *   rz_integrvs          INTEGRVS: precision integration to TETTHIS
 *   agc_integ_atopcsm/lm ATOPOTH: state into the permanent vector
 *   agc_integ_install    INTSTALL
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; INTSTALL / integration sleeps resume the job from its start.
 *
 * NOTES:
 *   Scales: DELVOV m/cs 2^7 (local vertical: x along the horizontal, z
 *   down), RATT m 2^29, VATT m/cs 2^7, RCV VCV body scale.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"

/* PHASCHNG OCT 04024: group 4, phase 2, type A. */
#define TDV_PHASE_04024 ((uint16_t) 04024) /* PHASCHNG OCT 04024 */

/* The other-vehicle state at TIG between the jobs of P76 (the AGC keeps
 * it in the push list across the INTSTALL sleeps).                      */
typedef struct tdv_work
{
    agc_vec_t ratt; /* RATT at TIG, m 2^29                            */
    agc_vec_t vnew; /* VATT + DELTA V (push cell 6), m/cs 2^7         */
    int       x2;   /* X2: 0 earth, 2 moon                            */
} tdv_work_t;

static tdv_work_t tdv_work; /* state between the jobs */

/**********************************************************************
 * COMPMAT (p.511-512): the LV -> reference matrix of the state (RATT,
 * VATT), rows U((R x V) x R), U(V x R), U(-R); v_new = VATT + DELVOV M.
 *
 * FUNCTION NAME: agc_target_delta_v_lv_to_ref
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   ratt      agc_vec_t       I    RATT, the state at TIG, m 2^29
 *   vatt      agc_vec_t       I    VATT, m/cs 2^7
 *   dv_lv     agc_vec_t       I    DELVOV, local vertical, m/cs 2^7
 *   v_new     agc_vec_t *     O    VATT + delta V in reference axes, m/cs
 *                                  2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t agc_target_delta_v_lv_to_ref(agc_interp_t *ip, agc_vec_t ratt,
                                          agc_vec_t vatt, agc_vec_t dv_lv,
                                          agc_vec_t *v_new)
{
    if (!AGC_ASSERT(ip != NULL && v_new != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t m; /* LV -> reference rows, unit 2^1 */

    m.row[2] = agc_vec_neg(rz_unit(ip, ratt));                     /* 24D */
    m.row[1] = rz_unit(ip, agc_vec_cross(ip, m.row[2], vatt));     /* 18D */
    m.row[0] = rz_unit(ip, agc_vec_cross(ip, m.row[1], m.row[2])); /* 12D */
    agc_vec_t dv = /* delta V, m/cs 2^7 */
                   agc_vec_shl(ip, agc_vec_mat(ip, dv_lv, &m), 1); /* VSL1 */

    *v_new = agc_vec_add(ip, dv, vatt);

    if (!AGC_ASSERT(m.row[2].c[0] != AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * P76SUB1: MOONFLAG for the integration. CM: X2 = 0 clear, 2 set. LM:
 * X2 = 2 and MOONFLAG set when CMOONFLG is set, else X2 = 0, clear.
 * AGC: P76SUB1 (P76.agc p.513)
 *
 * FUNCTION NAME: tdv_moonflag
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------
 *   w         tdv_work_t *  I/O  X2 (LM: set from CMOONFLG)
 **********************************************************************/
static void tdv_moonflag(tdv_work_t *w)
{
    if (!rz_is_csm())
    {
        w->x2 = agc_flag_test(AGC_FLAG_CMOONFLG) ? 2 : 0; /* moon: X2 = 2 */
    }

    rz_flag_put(AGC_FLAG_MOONFLAG, w->x2 != 0);

    if (!AGC_ASSERT(w->x2 == 0 || w->x2 == 2))
    {
        return;
    }
}

/**********************************************************************
 * P76 TARGET DELTA V (P76.agc p.512, label P76): UPFLAG TRACKFLG; LM:
 * DELVOV = DELVLVC; V06N84 flashing (TERMINATE -> ENDP76, PROCEED ->
 * V06N33, ENTER -> the display again).
 *
 * FUNCTION NAME: agc_job_target_delta_v
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
agc_status_t agc_job_target_delta_v(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_TRACKFLG);

    if (!rz_is_csm())
    {
        agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

        rz->delvov = rz->delvlvc;
    }

    return (agc_job_target_delta_v_display(ctx));
}

/**********************************************************************
 * V06N84 (DELVOV) flashing; ENTER ("TC -5") shows it again.
 * AGC: V06N84 (P76.agc p.513)
 *
 * FUNCTION NAME: agc_job_target_delta_v_display
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

/**********************************************************************
 * V06N33 (TIG) flashing; PROCEED -> the update.
 * AGC: PROCEED (DISPLAY_INTERFACE_ROUTINES.agc p.1481)
 *
 * FUNCTION NAME: agc_job_target_delta_v_tig
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

/**********************************************************************
 * TDEC1 = TIG, OTHPREC; COMPMAT (VATT + DELVOV in reference axes). An
 * INTSTALL sleep inside OTHPREC resumes this job from its start.
 * AGC: COMPMAT (P76.agc p.511)
 *
 * FUNCTION NAME: agc_job_target_delta_v_update
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
agc_status_t agc_job_target_delta_v_update(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_integ_out_t o; /* integration result */
    agc_status_t    st = rz_integ(&ip, RZ_OTHPREC, agc_erase_tig(), /* status */
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

/**********************************************************************
 * INTSTALL, P76SUB1; RCV VCV = the updated state at body scale (VSR*
 * 0,2), TET = TIG, CLEAR INTYPFLG, TDEC1 = TETTHIS; INTEGRVS.
 * AGC: INTSTALL (INTEGRATION_INITIALIZATION.agc p.1324)
 *
 * FUNCTION NAME: agc_job_target_delta_v_integrate
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
agc_status_t agc_job_target_delta_v_integrate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    tdv_moonflag(&tdv_work);
    rz_integrvs_in_t in; /* INTEGRVS inputs */

    in.moon = (tdv_work.x2 != 0);
    /* VSR* 0,2: count 0 - X2, i.e. shift left 2 about the moon (2^29 ->
     * 2^27, 2^7 -> 2^5).                                                */
    in.vcv   = agc_vec_shl(&ip, tdv_work.vnew, tdv_work.x2);
    in.rcv   = agc_vec_shl(&ip, tdv_work.ratt, tdv_work.x2);
    in.tet   = agc_erase_tig();
    in.conic = false;
    in.tdec1 =
        rz_is_csm() ? agc_erase_tetcsm() : agc_erase_tetlem(); /* TETTHIS */
    agc_integ_out_t o; /* integration result */
    agc_status_t    st = /* status */
                         rz_integrvs(&ip, &in, AGC_JOB_TARGET_DELTA_V_INTEGRATE,
                                     &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    tdv_work.x2 = o.x2; /* RATT1 VATT1 TAT stay in the integration
                         * output, as in the AGC erasables         */

    return (agc_job_target_delta_v_store(ctx));
}

/**********************************************************************
 * INTSTALL (CM: P76SUB1 again); RRECT = RCV = RATT1, TET = TAT,
 * MINIRECT(VATT1); PHASCHNG 04024; UPFLAG REINTFLG; ATOPOTH; INTWAKE1;
 * ENDP76.
 * AGC: INTSTALL (INTEGRATION_INITIALIZATION.agc p.1324)
 *
 * FUNCTION NAME: agc_job_target_delta_v_store
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
agc_status_t agc_job_target_delta_v_store(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = /* INTSTALL */
                      agc_integ_install(0, AGC_JOB_TARGET_DELTA_V_STORE);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* asleep        */
    }

    if (rz_is_csm())
    {
        tdv_moonflag(&tdv_work);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_integ_ctx_t *ic = agc_integ_shared_ctx(); /* integration erasables */

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
} /* agc_job_target_delta_v_store */

/**********************************************************************
 * ENDP76: MARKCTR = 0 (CM also VHFCNT = 0, MRKBUF2 = -1), GOTOPOOH.
 * AGC: ENDP76 (P76.agc p.513)
 *
 * FUNCTION NAME: agc_job_target_delta_v_end
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
