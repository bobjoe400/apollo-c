/**********************************************************************
 * FILE NAME: lambert_aimpoint_guidance_lm.c
 *
 * PURPOSE:
 *   P31 GENERAL LAMBERT AIMPOINT GUIDANCE (LM): "To accept targeting
 *   parameters obtained from a source external to the LEM and compute
 *   there from the required velocity and other initial conditions
 *   required by LM for desired maneuver. The targeting parameters are
 *   TIG (time of ignition), target vector (RTARG), and the time from TIG
 *   until the target is reached (DELLT4)."
 *
 * AGC SOURCE:
 *   Luminary099 GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc pp. 651-653 (P31,
 *   REVN1645); LM only.
 *
 * EXTERNAL VARIABLES:
 *   Name      Type       I/O  Scale        Description
 *   --------  ---------  ---  -----------  ----------------------------
 *   TIG       agc_dp_t   I    cs 2^28      time of ignition
 *   RTARG     agc_vec_t  I    m 2^29       target vector (agc_rendz_t)
 *   DELLT4    agc_dp_t   I    cs 2^28      transfer time
 *   DELVSIN   agc_vec_t  O    m/cs 2^7     required delta V (DELVEET3)
 *   VGDISP    agc_dp_t   O    m/cs 2^7     |DELVEET3| (guidance state)
 *   DELVLVC   agc_vec_t  O    m/cs 2^7     delta V, local vertical
 *   HAPO HPER agc_dp_t   O    m 2^29       post-burn apsides altitudes
 *   UPDATFLG XDELVFLG    O    -            cleared
 *   FINALFLG flag        O    -            set for VN1645
 *
 * EXTERNAL REFERENCES:
 *   Name                          Description
 *   ----------------------------  --------------------------------------
 *   rz_integ (LEMPREC)            LM state at TIG
 *   agc_rendz_initvel             INITVEL: Lambert required velocity
 *   agc_rendz_get_lvc             GET.LVC: delta V to local vertical
 *   agc_rendz_periapo_body_scale  PERIAPO1: apsides of the new orbit
 *   agc_orbit_params_maxchk       MAXCHK: 9999.9 NM display limit
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none (AGC "ALARMS OR ABORTS NONE"); an INTSTALL sleep resumes the
 *   compute job from its start.
 *
 * NOTES:
 *   Scales: r m 2^29, v m/cs 2^7, t cs 2^28.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"

/**********************************************************************
 * P31 (p.652): P20FLGON; V06N33 (TIG).
 *
 * FUNCTION NAME: agc_job_lambert_aimpoint_guidance_lm
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
agc_status_t agc_job_lambert_aimpoint_guidance_lm(agc_job_ctx_t *ctx)
{
    rz_tracking_flags_on();

    return (rz_vnpooh(ctx, 6U, 33U, AGC_JOB_LAMBERT_AIMPOINT_COMPUTE));
}

/**********************************************************************
 * HPER, HAPO of the post-burn state (RTIG, VIPRIME): PERIAPO1, SHIFTR1,
 * MAXCHK.
 * AGC: P31 "VLOAD PDVL RTIG VIPRIME" .. "STORE HAPO"
 * (GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc p.653)
 *
 * FUNCTION NAME: laim_apsides
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    RTIG, VIPRIME (post-burn state at
 *                                       TIG)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t laim_apsides(agc_interp_t *ip, const agc_rendz_t *rz)
{
    agc_rendz_periapo_t pa;                                                                    /* PERIAPO1 apsides, body scale */
    agc_status_t        st = agc_rendz_periapo_body_scale(ip, rz, rz->rtig, rz->viprime, &pa); /* status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_burn_t *b = &agc_guid_state()->burn; /* HAPO, HPER, VGDISP */

    b->hper = agc_orbit_params_maxchk(ip, agc_rendz_scalar_from_body(rz, pa.hperi));
    b->hapo = agc_orbit_params_maxchk(ip, agc_rendz_scalar_from_body(rz, pa.hapo));

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * After V06N33 (pp.652-653): CLEAR UPDATFLG; LM state to TIG (LEMPREC):
 * RTIG = RINIT, VTIG = VINIT; TPASS4 = TIG + DELLT4; RTX1 RTX2 from the
 * integration; INITVEL with no offsets and E4 = 0; DELVSIN, VGDISP,
 * CLEAR XDELVFLG, GET.LVC; HPER HAPO; V06N81. An INTSTALL sleep
 * resumes this job from its start.
 *
 * FUNCTION NAME: agc_job_lambert_aimpoint_compute_lm
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
agc_status_t agc_job_lambert_aimpoint_compute_lm(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    agc_flag_clear(AGC_FLAG_UPDATFLG);
    agc_dp_t        tig = agc_erase_tig(); /* TIG, cs 2^28 */
    agc_integ_out_t o;                     /* LEMPREC result at TIG */
    agc_status_t    st;                    /* integration status */

    st = rz_integ(&ip, RZ_LEMPREC, tig, AGC_JOB_LAMBERT_AIMPOINT_COMPUTE, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->rtig   = o.ratt;
    rz->rinit  = o.ratt;
    rz->vtig   = o.vatt;
    rz->vinit  = o.vatt;
    rz->tpass4 = agc_dp_add(&ip, rz->dellt4, tig);
    agc_rendz_selectmu(rz, o.x2 != 0); /* SXA,1 SXA,2   */
    st         = agc_rendz_initvel(&ip, rz, 0, 0, AGC_JOB_LAMBERT_AIMPOINT_COMPUTE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->delvsin                   = rz->delveet3;
    agc_guid_state()->burn.vgdisp = rz_abval(&ip, rz->delveet3);
    agc_flag_clear(AGC_FLAG_XDELVFLG);
    st                            = agc_rendz_get_lvc(&ip, rz->rinit, rz->vinit, rz->delveet3, &rz->delvlvc);

    if (st == AGC_STATUS_OK)
    {
        st = laim_apsides(&ip, rz);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rz_vnpooh(ctx, 6U, 81U, AGC_JOB_LAMBERT_AIMPOINT_APSIDES));
} /* agc_job_lambert_aimpoint_compute_lm */

/**********************************************************************
 * After V06N81: V06N42 (HAPO, HPER, VGDISP).
 * AGC: P31 "CAF V06N42 / TC VNPOOH" (GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc
 * p.653)
 *
 * FUNCTION NAME: agc_job_lambert_aimpoint_apsides
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
agc_status_t agc_job_lambert_aimpoint_apsides(agc_job_ctx_t *ctx)
{
    return (rz_vnpooh(ctx, 6U, 42U, AGC_JOB_MARKS_TFI_MGA_MONITOR));
}

/**********************************************************************
 * REVN1645 (p.653): SET FINALFLG, VN1645 (TRKMKCNT, TTOGO, +MGA), and
 * again.
 *
 * FUNCTION NAME: agc_job_marks_tfi_mga_monitor
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
agc_status_t agc_job_marks_tfi_mga_monitor(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_flag_set(AGC_FLAG_FINALFLG);

    return (rz_ttogo_display(ctx, &ip, agc_rendz_erasable(), AGC_JOB_MARKS_TFI_MGA_MONITOR));
}
