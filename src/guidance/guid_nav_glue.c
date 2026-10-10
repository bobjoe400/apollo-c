/*
 * guid_nav_glue.c -- the precision-integration entrances the powered-flight
 * programs use (THISPREC / CSMPREC / LEMPREC, MIDTOAV1, MIDTOAV2,
 * AVETOMID), adapted to the nav-integ interface (include/agc/nav/integration.h).
 *
 * Each call works on the shared integration context. AGC_STATUS_BUSY means
 * the integration package was in use: the calling job has been put to sleep
 * and will be resumed as `resume` (normally the caller's own job id, which
 * then repeats the call from its start, as the AGC's INTSTALL does).
 * MIDTOAV1/2 leave RN1 VN1 PIPTIME1 in the average-G block and RTX1/RTX2 in
 * the rendezvous body block, as the AGC leaves them in erasable.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/guidance/pending.h"
#include "agc/nav/integration.h"

/* The job that resumes after an INTSTALL wait when a targeting subroutine
 * (S30.1, S31.1, AGAIN, S40.1) has no job context of its own: set by the
 * job that calls it.                                                     */
static agc_job_id_t g_resume = AGC_JOB_NONE;

void agc_guid_nav_set_resume(agc_job_id_t job)
{
    if (!AGC_ASSERT(job != AGC_JOB_NONE))
    {
        return;
    }

    g_resume = job;
}

agc_job_id_t agc_guid_nav_resume(void)
{
    if (!AGC_ASSERT(g_resume != AGC_JOB_NONE))
    {
        return (AGC_JOB_NONE);
    }

    return (g_resume);
}

static agc_job_id_t resume_or_default(agc_job_id_t resume)
{
    agc_job_id_t r = (resume != AGC_JOB_NONE) ? resume : g_resume;

    if (!AGC_ASSERT(r != AGC_JOB_NONE))
    {
        return (AGC_JOB_NONE);
    }

    return (r);
}

static void store_body(const agc_integ_out_t *o)
{
    agc_rendz_t *rz = agc_rendz_erasable();

    rz->body.rtx1 = (agc_sp_t) o->x1;
    rz->body.rtx2 = (agc_sp_t) o->x2;
    rz->body.moon = (o->x2 != 0);

    if (!AGC_ASSERT(o->x2 == 0 || o->x2 == 2))
    {
        return;
    }
}

/* THISPREC (this vehicle), CSMPREC or LEMPREC to tdec1 (cs 2^28).          */
agc_status_t agc_guid_prec_update(agc_interp_t *ip, agc_vehicle_t veh,
                                  agc_dp_t tdec1, agc_job_id_t resume,
                                  agc_guid_prec_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL) ||
        !AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    agc_status_t     st  =
        (veh == AGC_VEHICLE_CSM) ?
        agc_integ_csmprec(ip, ctx, tdec1, resume_or_default(resume)) :
        agc_integ_lemprec(ip, ctx, tdec1, resume_or_default(resume));

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_integ_out_t *o = &ctx->out;

    out->ratt  = o->ratt;
    out->vatt  = o->vatt;
    out->ratt1 = o->ratt1;
    out->vatt1 = o->vatt1;
    out->tat   = o->tat;
    out->rtx1  = (agc_sp_t) o->x1;
    out->rtx2  = (agc_sp_t) o->x2;

    return (o->valid ? AGC_STATUS_OK : AGC_STATUS_NOT_FOUND); /* no RECTOUT */
}

static void store_midtoav(const agc_integ_midtoav_out_t *o, agc_dp_t *dt)
{
    agc_avg_g_t *a = &agc_guid_state()->avg;

    a->rn1      = o->rn1;
    a->vn1      = o->vn1;
    a->piptime1 = o->piptime1;
    agc_integ_out_t body = {true, o->rn1, o->vn1, o->piptime1, o->rn1,
                            o->vn1, 0, o->rtx1, o->rtx2};

    store_body(&body);
    *dt = o->dt;
}

/* MIDTOAV1 to tdec1; *late: TIG had to slip (NOTIME, alarm 1703).          */
agc_status_t agc_guid_midtoav1(agc_interp_t *ip, agc_dp_t tdec1,
                               agc_job_id_t resume, agc_dp_t *dt, bool *late)
{
    if (!AGC_ASSERT(ip != NULL && dt != NULL && late != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_midtoav_out_t o;
    agc_status_t            st = agc_integ_midtoav1(ip,
                                                    agc_integ_shared_ctx(), tdec1,
                                                    resume_or_default(
                                                        resume), &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    store_midtoav(&o, dt);
    *late = o.slipped;

    return (AGC_STATUS_OK);
}

/* MIDTOAV2: to the next PREREAD time; *dt = time from now to PIPTIME1.     */
agc_status_t agc_guid_midtoav2(agc_interp_t *ip, agc_job_id_t resume,
                               agc_dp_t *dt)
{
    if (!AGC_ASSERT(ip != NULL && dt != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_midtoav_out_t o;
    agc_status_t            st = agc_integ_midtoav2(ip, agc_integ_shared_ctx(),
                                                    resume_or_default(
                                                        resume), &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    store_midtoav(&o, dt);

    return (AGC_STATUS_OK);
}

/* AVETOMID: average-G state back into the coasting integration.            */
agc_status_t agc_guid_avetomid(agc_interp_t *ip, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_integ_avetomid(ip, agc_integ_shared_ctx(), resume));
}
