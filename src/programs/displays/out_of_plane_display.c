/**********************************************************************
 * FILE NAME: out_of_plane_display.c
 *
 * PURPOSE:
 *   R36 OUT-OF-PLANE RENDEZVOUS ROUTINE (verb 90), both vehicles: "To
 *   display at astronaut request LGC calculated rendezvous out-of-plane
 *   parameters (Y, YDOT, PSI)." Y and YDOT are the active vehicle
 *   position and velocity along the normal of the passive orbit, PSI the
 *   angle between the forward horizontal of the active vehicle and the
 *   line of sight projected into its horizontal plane. They are kept in
 *   RANGE, RRATE and RTHETA.
 *
 * AGC SOURCE:
 *   Comanche055 P34-35_P74-75.agc pp. 501-504 (R36, R36A, R36P3,
 *   R36PROG5 .. R36B)
 *   Luminary099 P34-35_P74-75.agc (differs: no vehicle choice, the CSM
 *   is always passive; PSI corrected with LODPMAX, the same value as the
 *   CM's DPPOSMAX)
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   OPTIONX     agc_sp_t   O    -          V04N12 vehicle option (CM)
 *   DSPTEMX     agc_dp_t   I/O  cs 2^28    time of event (N16)
 *   RANGE       agc_dp_t   O    m 2^29     Y (N90)
 *   RRATE       agc_dp_t   O    m/cs 2^7   YDOT (N90)
 *   RTHETA      agc_dp_t   O    rev 2^0    PSI (N90)
 *   RPASS36 OPTIONY  file static          state between the jobs
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ----------------------------------------------
 *   rz_integ              THISPREC / OTHPREC: states at the event time
 *   rz_flash              GOMARKF / GOXDSPF display requests
 *   agc_rtb_loadtime      LOADTIME: present time
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. Jobs return AGC_STATUS_BAD_ARG for a NULL ctx, else the
 *   status of the display or integration call that failed.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Extended verb jobs. DEVIATION: a sleep in an integration resumes the
 *   compute job from its start; with a zero DSPTEMX the present time is
 *   then read again.
 *
 * NOTES:
 *   Scales: r m 2^29, v m/cs 2^7, Y m 2^29, YDOT m/cs 2^7, PSI rev 2^0,
 *   t cs 2^28.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"

/* State between the jobs of R36 (the AGC's RPASS36, UNP36, OPTIONY). */
typedef struct oop_work
{
    agc_vec_t rpass36; /* RPASS36: passive position at the time, 2^29   */
    uint16_t  optiony; /* OPTIONY: CM vehicle option (1 or 2)            */
} oop_work_t;

static oop_work_t oop_work = {{{0, 0, 0}}, 1U};

/**********************************************************************
 * R36PROG5 .. R36B (pp.503-504): Y, YDOT, PSI
 * AGC: R36PROG5 .. R36B (P34-35_P74-75.agc pp.503-504)
 *
 * FUNCTION NAME: agc_rendz_out_of_plane_compute
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   rp        agc_vec_t       I    passive position, m 2^29
 *   vp        agc_vec_t       I    passive velocity, m/cs 2^7
 *   ra        agc_vec_t       I    active position, m 2^29
 *   va        agc_vec_t       I    active velocity, m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a bad
 *          argument
 **********************************************************************/
agc_status_t agc_rendz_out_of_plane_compute(agc_interp_t *ip, agc_rendz_t *rz,
                                            agc_vec_t rp, agc_vec_t vp,
                                            agc_vec_t ra, agc_vec_t va)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* UNP36 = UNIT(V x UNIT(R)) of the passive vehicle (the AGC's
     * push-down order: VATT x UNIT RATT).                               */
    agc_vec_t u; /* UNP36, 2^1 */

    u = rz_unit(ip, agc_vec_cross(ip, vp, rz_unit(ip, rp)));
    agc_vec_t los = agc_vec_sub(ip, rp, ra); /* 12D LOS, m 2^29 */

    rz->range = agc_dp_shl(ip, agc_vec_dot(ip, ra, u), 1); /* Y         */
    rz->rrate = agc_dp_shl(ip, agc_vec_dot(ip, va, u), 1); /* YDOT      */
    agc_vec_t ura = rz_unit(ip, ra); /* 18D, 2^1 */
    agc_vec_t ua; /* (URA x VA) x URA, 2^9 */

    ua = agc_vec_cross(ip, agc_vec_cross(ip, ura, va), ura);

    ua = rz_unit(ip, rz_unit(ip, agc_vec_shl(ip, ua, 2))); /* 00D       */
    agc_vec_t proj = /* m 2^29 */
                     agc_vec_shl(ip,
                                 agc_vec_scale(ip, ura,
                                               agc_vec_dot(ip, ura, los)), 2);
    agc_vec_t ul; /* 2^1 */

    ul = rz_unit(ip, rz_unit(ip, agc_vec_sub(ip, los, proj)));
    agc_dp_t c; /* SL1, cos at 2^1 */

    c = agc_dp_shl(ip, agc_vec_dot(ip, ul, ua), 1);

    rz->rtheta = agc_dp_acos(ip, c);                         /* PSI       */

    if (agc_vec_dot(ip, agc_vec_cross(ip, ul, ua), ra) < 0)
    { /* BPL       */
        rz->rtheta = agc_dp_sub(ip, RZ_DPPOSMAX, rz->rtheta);
    }

    if (!AGC_ASSERT(rz->rtheta >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* agc_rendz_out_of_plane_compute */

/**********************************************************************
 * R36 (CM, p.502): OPTIONX = 2, OPTIONX+1 = 1; V04N12 (TERMINATE ->
 * ENDEXT, PROCEED -> R36A, data load -> again). LM: straight to R36A.
 * AGC: R36 (P34-35_P74-75.agc p.502)
 *
 * FUNCTION NAME: agc_job_out_of_plane_display
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
agc_status_t agc_job_out_of_plane_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!rz_is_csm())
    {
        return (agc_job_out_of_plane_start(ctx));
    }

    agc_status_t st = rz_set_option(1, 2U); /* OPTIONX = 2 */

    if (st == AGC_STATUS_OK)
    {
        st = rz_set_option(2, 1U);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_out_of_plane_vehicle(ctx));
}

/**********************************************************************
 * V04N12 GOXDSPF (CM).
 * AGC: R36 V04N12 GOXDSPF (P34-35_P74-75.agc p.502)
 *
 * FUNCTION NAME: agc_job_out_of_plane_vehicle
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
agc_status_t agc_job_out_of_plane_vehicle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOMARKF, 4U, 12U, AGC_JOB_RENDZ_ENDEXT,
                     AGC_JOB_OUT_OF_PLANE_START, AGC_JOB_OUT_OF_PLANE_VEHICLE));
}

/**********************************************************************
 * R36A (p.502): OPTIONY = OPTIONX+1 (CM), DSPTEMX = 0 (time of event 0
 * for the first display), R36P3.
 * AGC: R36A (P34-35_P74-75.agc p.502)
 *
 * FUNCTION NAME: agc_job_out_of_plane_start
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
agc_status_t agc_job_out_of_plane_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (rz_is_csm())
    {
        oop_work.optiony = rz_option(2); /* LXCH OPTIONY   */
    }

    agc_status_t st = rz_set_dsptem_dp(RZ_DSPTEMX, 0); /* DSPTEMX = 0 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_out_of_plane_time(ctx));
}

/**********************************************************************
 * R36P3: V06N16 GOMARKF (time of event); TERMINATE -> ENDEXT, PROCEED ->
 * compute, RECYCLE -> again.
 * AGC: R36P3 (P34-35_P74-75.agc p.502)
 *
 * FUNCTION NAME: agc_job_out_of_plane_time
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
agc_status_t agc_job_out_of_plane_time(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOMARKF, 6U, 16U, AGC_JOB_RENDZ_ENDEXT,
                     AGC_JOB_OUT_OF_PLANE_COMPUTE, AGC_JOB_OUT_OF_PLANE_TIME));
}

/**********************************************************************
 * Passive (first) or active (second) vehicle integration: CM option 1
 * (CSM display) passive = OTHPREC, active = THISPREC; option 2 the
 * reverse; LM passive = OTHPREC, active = THISPREC.
 * AGC: R36 INTSTALL calls with OPTIONY (P34-35_P74-75.agc p.503)
 *
 * FUNCTION NAME: oop_kind
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   passive   bool  I    true: the passive vehicle, false: the active one
 *
 * RETURNS: rz_integ_kind_t -- RZ_OTHPREC or RZ_THISPREC
 **********************************************************************/
static rz_integ_kind_t oop_kind(bool passive)
{
    bool reverse; /* CM option 2 */

    reverse = rz_is_csm() && ((oop_work.optiony >> 1) != 0U);
    bool other = (passive != reverse); /* OTHPREC */

    return (other ? RZ_OTHPREC : RZ_THISPREC); /* other or this vehicle */
}

/**********************************************************************
 * PROCEED on V06N16 (pp.502-504): time = DSPTEMX, or now when zero;
 * passive state (RPASS36, UNP36), active state at TAT; Y YDOT PSI;
 * DSPTEMX = TAT; V06N90 (TERMINATE, PROCEED -> ENDEXT, RECYCLE ->
 * R36P3). A sleep in an integration resumes this job from its start
 * (DEVIATION: with a zero DSPTEMX the present time is then read again).
 * AGC: R36P3 PROCEED .. R36B (P34-35_P74-75.agc pp.502-504)
 *
 * FUNCTION NAME: agc_job_out_of_plane_compute
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
agc_status_t agc_job_out_of_plane_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */
    agc_dp_t     t  = rz_dsptem_dp(RZ_DSPTEMX); /* event time, cs 2^28 */

    if (t == 0)
    {
        t = agc_rtb_loadtime(); /* ENTTIM2        */
    }

    agc_integ_out_t o; /* integration result */
    agc_status_t    st = /* passive */
                         rz_integ(&ip, oop_kind(
                                      true), t, AGC_JOB_OUT_OF_PLANE_COMPUTE,
                                  &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    oop_work.rpass36 = o.ratt;
    agc_vec_t vp = o.vatt; /* VPASS, m/cs 2^7 */

    st =
        rz_integ(&ip, oop_kind(false), o.tat, AGC_JOB_OUT_OF_PLANE_COMPUTE, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_rendz_out_of_plane_compute(&ip, rz, oop_work.rpass36, vp, o.ratt,
                                        o.vatt);

    if (st == AGC_STATUS_OK)
    {
        st = rz_set_dsptem_dp(RZ_DSPTEMX, o.tat); /* SGNAGREE 30D   */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rz_flash(ctx, AGC_DSKY_GOMARKF, 6U, 90U, AGC_JOB_RENDZ_ENDEXT,
                     AGC_JOB_RENDZ_ENDEXT, AGC_JOB_OUT_OF_PLANE_TIME));
} /* agc_job_out_of_plane_compute */
