/*
 * lambert_aimpoint_guidance_lm.c -- P31 GENERAL LAMBERT AIMPOINT
 * GUIDANCE, LM only (GENERAL_LAMBERT_AIMPOINT_GUIDANCE.agc, Luminary099
 * pp.651-653).
 *
 * "To accept targeting parameters obtained from a source external to the
 * LEM and compute there from the required velocity and other initial
 * conditions required by LM for desired maneuver. The targeting
 * parameters are TIG (time of ignition), target vector (RTARG), and the
 * time from TIG until the target is reached (DELLT4)."
 *
 * Outputs: DELVSIN (= DELVEET3), VGDISP = |DELVEET3|, DELVLVC, HAPO and
 * HPER (limited to 9999.9 NM); XDELVFLG cleared for Lambert VG steering.
 * Scales: r m 2^29, v m/cs 2^7, t cs 2^28.
 */
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"

/* P31 (p.652): P20FLGON; V06N33 (TIG). */
agc_status_t agc_job_lambert_aimpoint_guidance(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    rz_tracking_flags_on();

    return rz_vnpooh(ctx, 6U, 33U, AGC_JOB_LAMBERT_AIMPOINT_COMPUTE);
}

/* HPER, HAPO of the post-burn state (RTIG, VIPRIME): PERIAPO1, SHIFTR1,
 * MAXCHK.                                                             */
static agc_status_t laim_apsides(agc_interp_t *ip, const agc_rendz_t *rz)
{
    agc_rendz_periapo_t pa;
    agc_status_t        st = agc_rendz_periapo1(ip, rz, rz->rtig, rz->viprime,
                                                &pa);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    agc_burn_t *b = &agc_guid_state()->burn;

    b->hper = agc_orbit_params_maxchk(ip, agc_rendz_shiftr1(rz, pa.hperi));
    b->hapo = agc_orbit_params_maxchk(ip, agc_rendz_shiftr1(rz, pa.hapo));

    if (!AGC_ASSERT(b->hper <= RZ_MAXNM))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* After V06N33 (pp.652-653): CLEAR UPDATFLG; LM state to TIG (LEMPREC):
 * RTIG = RINIT, VTIG = VINIT; TPASS4 = TIG + DELLT4; RTX1 RTX2 from the
 * integration; INITVEL with no offsets and E4 = 0; DELVSIN, VGDISP,
 * CLEAR XDELVFLG, GET.LVC; HPER HAPO; V06N81. An INTSTALL sleep
 * resumes this job from its start.                                    */
agc_status_t agc_job_lambert_aimpoint_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();

    agc_flag_clear(AGC_FLAG_UPDATFLG);
    agc_dp_t        tig = agc_erase_tig();
    agc_integ_out_t o;
    agc_status_t    st =
        rz_integ(&ip, RZ_LEMPREC, tig, AGC_JOB_LAMBERT_AIMPOINT_COMPUTE, &o);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    rz->rtig   = o.ratt;
    rz->rinit  = o.ratt;
    rz->vtig   = o.vatt;
    rz->vinit  = o.vatt;
    rz->tpass4 = agc_dp_add(&ip, rz->dellt4, tig);
    rz->intime = tig;
    st         = agc_rendz_selectmu(rz, o.x2 != 0); /* SXA,1 SXA,2   */

    if (st == AGC_STATUS_OK)
    {
        st = agc_rendz_initvel(&ip, rz, 0, 0, AGC_JOB_LAMBERT_AIMPOINT_COMPUTE);
    }

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    rz->delvsin                   = rz->delveet3;
    agc_guid_state()->burn.vgdisp = rz_abval(&ip, rz->delveet3);
    agc_flag_clear(AGC_FLAG_XDELVFLG);
    st = agc_rendz_get_lvc(&ip, rz->rinit, rz->vinit, rz->delveet3,
                           &rz->delvlvc);

    if (st == AGC_STATUS_OK)
    {
        st = laim_apsides(&ip, rz);
    }

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    return rz_vnpooh(ctx, 6U, 81U, AGC_JOB_LAMBERT_AIMPOINT_APSIDES);
}

/* After V06N81: V06N42 (HAPO, HPER, VGDISP). */
agc_status_t agc_job_lambert_aimpoint_apsides(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return rz_vnpooh(ctx, 6U, 42U, AGC_JOB_LAMBERT_AIMPOINT_TTOGO);
}

/* REVN1645 (p.653): SET FINALFLG, VN1645 (TRKMKCNT, TTOGO, +MGA), and
 * again.                                                               */
agc_status_t agc_job_lambert_aimpoint_ttogo(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_flag_set(AGC_FLAG_FINALFLG);

    return rz_ttogo_display(ctx, &ip, rz_state(),
                            AGC_JOB_LAMBERT_AIMPOINT_TTOGO);
}
