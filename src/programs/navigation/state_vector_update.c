/**********************************************************************
 * FILE NAME: state_vector_update.c
 *
 * PURPOSE:
 *   The state-vector work the rendezvous and navigation programs share:
 *   the integration calls (THISPREC / OTHPREC resolved to CSMPREC ..
 *   LEMCONIC, INTSTALL and the INTEGRVS set-up), the measurement update
 *   INCORP2 on the shared integration erasables, BVECTORS, and the
 *   restart phase changes these callers make (PHASCHNG type C, GRP2PC).
 *
 * AGC SOURCE:
 *   Comanche055 INTEGRATION_INITIALIZATION.agc pp.1318-1324 (CSMPREC ..
 *   INTSTALL, INTEGRVS), MEASUREMENT_INCORPORATION.agc p.1256 (INCORP2),
 *   P20-P25.agc p.576 (BVECTORS), PHASE_TABLE_MAINTENANCE.agc p.1409
 *   Luminary099 INTEGRATION_INITIALIZATION.agc pp.1214-1220,
 *   MEASUREMENT_INCORPORATION.agc p.1153, PHASE_TABLE_MAINTENANCE.agc
 *   p.1298
 *
 * EXTERNAL VARIABLES:
 *   Name       Type        I/O  Scale      Description
 *   ---------  ----------  ---  ---------  ----------------------------
 *   (integration erasables) agc_integ_ctx_t I/O  see integration.h
 *   X789       agc_vec_t   I/O  bias scales  agc_erase_x789()
 *   MOONFLAG INTYPFLG VEHUPFLG  flags    I/O  -
 *
 * EXTERNAL REFERENCES:
 *   Name                     Description
 *   -----------------------  -------------------------------------------
 *   agc_integ_csmprec ..     CSMPREC LEMPREC CSMCONIC LEMCONIC
 *   agc_integ_install        INTSTALL
 *   agc_integ_integrvs       INTEGRVS
 *   agc_incorp_update_w      INCORP2 W update
 *   agc_incorp_update_state  INCORP2 state update
 *   agc_phase_change_var     PHASCHNG / 2PHSCHNG
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms of their own; integration statuses (INTSTALL sleep) and
 *   the INCORP2 steps' statuses are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called from rendezvous and navigation jobs, both vehicles.
 **********************************************************************/
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "agc/nav/conic.h"
#include "internal.h"

/**********************************************************************
 * INCORP2: W update, then the deviation into the corrected vehicle's
 * state (MOVEP.., FAZAB, RECTIFY on overflow, MOVEA.. with SVDWN), X789,
 * INTWAKE. moonthis: CMOONFLG (CM) or LMOONFLG (LM). DEVIATION: the AGC
 * updates W row by row through HOLDW with a GRP2PC per row (FAZA, FAZB)
 * so a restart finishes the update; here the update is one call and the
 * GRP2PCs restart the running job segment. Each caller names its own
 * restart point after INCORP2 returns (rz_grp2pc), so only a restart
 * inside this call (a BAILOUT of RECTIFY or INTWAKE) reruns the segment.
 * AGC: INCORP2 (MEASUREMENT_INCORPORATION.agc p.1256)
 *
 * FUNCTION NAME: rz_incorp2
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  --------------------------
 *   ip        agc_interp_t *        I/O  interpreter state (OVFIND)
 *   inc       const agc_incorp_t *  I    INCORP1 results (deviations, W
 *                                        update terms)
 *   moonthis  bool                  I    CMOONFLG (CM) or LMOONFLG (LM)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the W, state, RECTIFY or INTWAKE
 *          step
 **********************************************************************/
agc_status_t rz_incorp2(agc_interp_t *ip, const agc_incorp_t *inc, bool moonthis)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();                 /* integration erasables */
    bool             csm = agc_flag_test(AGC_FLAG_VEHUPFLG);       /* VEHUPFLG */
    agc_vehicle_t    veh = csm ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM; /* corrected */
    agc_status_t     st  = agc_incorp_update_w(ip, inc, &ctx->w);  /* W update */

    agc_phase_change(04022);           /* FAZC     */
    agc_integ_perm_get(veh, &ctx->sv); /* MOVEP..  */

    agc_incorp_sv_t sv; /* state for the update */

    sv.moon    = moonthis; /* MOONTHIS */
    sv.rcv     = ctx->sv.rcv;
    sv.vcv     = ctx->sv.vcv;
    sv.tdeltav = ctx->sv.deltav;
    sv.tnuv    = ctx->sv.nuv;
    sv.x789    = agc_erase_x789();

    if (st == AGC_STATUS_OK)
    {
        agc_incorp_update_state(ip, inc, &sv);
    }

    ctx->sv.rcv = sv.rcv;

    ctx->sv.vcv    = sv.vcv;
    ctx->sv.deltav = sv.tdeltav;
    ctx->sv.nuv    = sv.tnuv;
    ctx->pbody     = sv.moon ? 2 : 0; /* X2 of the body */
    rz_flag_put(AGC_FLAG_MOONFLAG, sv.moon);

    if (st == AGC_STATUS_OK && sv.rectify)
    {
        agc_integ_rectify(ip, ctx); /* FAZAB2   */
    }

    agc_phase_change(04022); /* FAZAB3   */

    if (st == AGC_STATUS_OK)
    { /* MOVEA..  */
        if (csm)
        {
            agc_integ_atopcsm(ip, ctx, ctx->pbody);
        }
        else
        {
            agc_integ_atoplem(ip, ctx, ctx->pbody);
        }
    }

    if (inc->dim9)
    {
        agc_erase_set_x789(sv.x789); /* FAZAB4   */
    }

    agc_status_t sw = AGC_STATUS_OK; /* INTWAKE status */

    if (agc_integ_area_busy(0))
    {
        sw = agc_integ_intwake(); /* INTWAKE */
    }

    return ((st == AGC_STATUS_OK) ? sw : st); /* first failure */
} /* rz_incorp2 */

#define RZ_PI_4 ((agc_dp_t) 210828714) /* PI/4.0 2DEC .785398164     */

/**********************************************************************
 * BVECTORS (P20-P25 CM p.576): USTAR = UNIT(USTAR x UNIT(RCLP)) = B0,
 * DELTAQ = (ACOS(USTAR . UM) - 1/4) PI/4 |RCLP| normalized (SRR* 0-3,1),
 * B1 = B2 = 0. rclp body scale, um and ustar half unit.
 * AGC: BVECTORS (P20-P25.agc p.576)
 *
 * FUNCTION NAME: rz_bvectors
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rclp      agc_vec_t       I    RCLP, body scale
 *   um        agc_vec_t       I    UM, half unit
 *   ustar     agc_vec_t *     I/O  USTAR, half unit; B0 out
 *   inc       agc_incorp_t *  O    BVECTOR, DELTAQ
 **********************************************************************/
void rz_bvectors(agc_interp_t *ip, agc_vec_t rclp, agc_vec_t um, agc_vec_t *ustar, agc_incorp_t *inc)
{
    agc_dp_t  absr = 0;                                   /* |RCLP|, body scale */
    agc_vec_t u    = agc_vec_unit(ip, rclp, &absr, NULL); /* 26D, 2^1 */

    inc->bvector[0] = agc_vec_unit(ip, agc_vec_cross(ip, *ustar, u), NULL, NULL);
    agc_phase_change(04022); /* GRP2PC   */
    *ustar          = inc->bvector[0];
    agc_dp_t a;              /* rev 2^0 */

    a           = agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, *ustar, um), 1));
    int      x1 = 0;                                               /* NORM index, -(left shifts) */
    agc_dp_t n  = rz_norm(agc_dp_sub(ip, a, AGC_DP_QUARTER), &x1); /* 2^(X1) */
    agc_tp_t t;                                                    /* TP, scale set by SRR* */

    t = agc_dp_mul_tp(agc_dp_mul_trunc(n, RZ_PI_4), absr);

    inc->deltaq     = agc_tp_round(ip, agc_tp_shift(ip, t, -(-3 - x1))); /* SRR* 0-3,1 */
    inc->bvector[1] = agc_vec_zero();
    inc->bvector[2] = agc_vec_zero();
}

/**********************************************************************
 * THISPREC / OTHPREC: CSMPREC or LEMPREC by vehicle.
 * AGC: THISPREC / OTHPREC (P34-35_P74-75.agc), CSMPREC / LEMPREC by
 * vehicle
 *
 * FUNCTION NAME: rz_resolve_kind
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   kind      rz_integ_kind_t  I    integration kind
 *
 * RETURNS: rz_integ_kind_t -- CSMPREC, LEMPREC, CSMCONIC or LEMCONIC
 **********************************************************************/
static rz_integ_kind_t rz_resolve_kind(rz_integ_kind_t kind)
{
    bool csm = rz_is_csm(); /* CM build */

    if (kind == RZ_THISPREC)
    {
        return (csm ? RZ_CSMPREC : RZ_LEMPREC); /* THISPREC */
    }

    if (kind == RZ_OTHPREC)
    {
        return (csm ? RZ_LEMPREC : RZ_CSMPREC); /* OTHPREC */
    }

    return (kind);
}

/**********************************************************************
 * CSMPREC LEMPREC CSMCONIC LEMCONIC THISPREC OTHPREC
 * (INTEGRATION_INITIALIZATION p.1318): permanent state to TDEC1.
 * AGC: CSMPREC .. LEMCONIC (INTEGRATION_INITIALIZATION.agc p.1318)
 *
 * FUNCTION NAME: rz_integ
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   kind      rz_integ_kind_t    I    integration kind (THISPREC / OTHPREC
 *                                     resolved here)
 *   tdec1     agc_dp_t           I    TDEC1, cs 2^28
 *   resume    agc_job_id_t       I    job that continues the caller
 *   out       agc_integ_out_t *  O    integration result (RATT VATT TAT
 *                                     RATT1 VATT1 X2)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status (INTSTALL sleep)
 **********************************************************************/
agc_status_t rz_integ(agc_interp_t *ip, rz_integ_kind_t kind, agc_dp_t tdec1, agc_job_id_t resume, agc_integ_out_t *out)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* integration erasables */
    agc_status_t     st;                           /* integration status */

    switch (rz_resolve_kind(kind))
    {
        case RZ_CSMPREC:
            st = agc_integ_csmprec(ip, ctx, tdec1, resume);
            break;
        case RZ_LEMPREC:
            st = agc_integ_lemprec(ip, ctx, tdec1, resume);
            break;
        case RZ_CSMCONIC:
            st = agc_integ_csmconic(ip, ctx, tdec1, resume);
            break;
        case RZ_LEMCONIC:
            st = agc_integ_lemconic(ip, ctx, tdec1, resume);
            break;
        case RZ_THISPREC:
        case RZ_OTHPREC:
        case RZ_INTEG_KIND_COUNT:
        default:
            st = AGC_STATUS_BAD_ARG;
            break;
    }

    if (st == AGC_STATUS_OK)
    {
        *out = ctx->out;
    }

    return (st);
}

/**********************************************************************
 * INTSTALL, then the INTEGRVS set-up common to INTINT, INITVEL, R31.
 * AGC: INTSTALL, INTEGRVS (INTEGRATION_INITIALIZATION.agc pp.1319-1324)
 *
 * FUNCTION NAME: rz_integrvs
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  ----------------------
 *   ip        agc_interp_t *            I/O  interpreter state (OVFIND)
 *   in        const rz_integrvs_in_t *  I    RCV VCV TET TDEC1, conic, moon
 *   resume    agc_job_id_t              I    job that continues the caller
 *   out       agc_integ_out_t *         O    integration result
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status (INTSTALL sleep)
 **********************************************************************/
agc_status_t rz_integrvs(agc_interp_t *ip, const rz_integrvs_in_t *in, agc_job_id_t resume, agc_integ_out_t *out)
{
    agc_status_t st = agc_integ_install(0, resume); /* INTSTALL */

    if (st == AGC_STATUS_OK)
    {
        agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* integration erasables */

        rz_flag_put(AGC_FLAG_INTYPFLG, in->conic);
        rz_flag_put(AGC_FLAG_MOONFLAG, in->moon);
        ctx->sv.rcv = in->rcv;
        ctx->sv.vcv = in->vcv;
        ctx->sv.tet = in->tet;
        st          = agc_integ_integrvs(ip, ctx, in->tdec1);

        if (st == AGC_STATUS_OK)
        {
            *out = ctx->out;
        }
    }

    return (st);
}

/**********************************************************************
 * PHASCHNG / 2PHSCHNG with a type C code whose restart point is job
 * `job` at priority `prio` (the AGC's OCT 05022 / OCT 26000 pair: the
 * restart runs the job by FINDVAC). code_a = 0: a single PHASCHNG.
 * AGC: PHASCHNG, 2PHSCHNG (PHASE_TABLE_MAINTENANCE.agc p.1409)
 *
 * FUNCTION NAME: rz_phase_job
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------
 *   code_a    uint16_t      I    first code of 2PHSCHNG, 0 for a single
 *                                PHASCHNG
 *   code_c    uint16_t      I    the type C code
 *   job       agc_job_id_t  I    restart job
 *   prio      int16_t       I    its priority
 **********************************************************************/
void rz_phase_job(uint16_t code_a, uint16_t code_c, agc_job_id_t job, int16_t prio)
{
    agc_phase_var_t v; /* restart job */

    v.kind       = AGC_RESTART_JOB;
    v.job        = job;
    v.task       = AGC_TASK_NONE;
    v.prio_or_dt = prio;
    v.immediate  = false;
    v.long_dt_cs = 0U;

    if (code_a == 0U)
    {
        agc_phase_change_var(code_c, &v);
    }
    else
    {
        agc_phase_change2(code_a, code_c, &v);
    }
}

/**********************************************************************
 * GRP2PC (TC PHASCHNG, OCT 04022): group 2 phase 2, a type C job phase
 * with the A and D bits clear, so the AGC restarts at the return address
 * of PHASCHNG with the preset PHSPRDT2. C cannot name a return address;
 * plain agc_phase_change(04022) restarts the job segment that is running,
 * which is wrong when that segment did non-repeatable work (took a mark,
 * counted, incorporated) before the call or reached the call through
 * another job's body. The caller names the job whose body starts where
 * the AGC resumes; the code word, and so PHASE2, stays the AGC's.
 * AGC: GRP2PC, PHASCHNG (PHASE_TABLE_MAINTENANCE.agc C p.1409, L p.1298)
 *
 * FUNCTION NAME: rz_grp2pc
 *
 * ARGUMENTS:
 *   Argument    Type          I/O  Description (scale)
 *   ----------  ------------  ---  ----------------------------------
 *   restart_at  agc_job_id_t  I    job resuming where the AGC would
 **********************************************************************/
void rz_grp2pc(agc_job_id_t restart_at)
{
    const agc_phase_var_t v = {AGC_RESTART_JOB, restart_at, AGC_TASK_NONE, 0, false, 0U}; /* D clear: PHSPRDT2 */

    agc_phase_change_var(04022, &v);
}
