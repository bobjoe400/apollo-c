/*
 * rendz_glue.c -- the single place where the rendezvous programs call into
 * other lanes (conic, integration, time of free fall, DSKY, executive).
 *
 * Each function is the AGC call set-up of the named entry point; when the
 * owning header lands with a different signature only this file changes
 * (see rendz_ext.h).
 */
#include "agc/agc_assert.h"
#include "agc/agc_flags_common.h"
#include "rendz_internal.h"

/* TIMETHET (CONIC_SUBROUTINES p.1269) */
agc_status_t rz_timethet(agc_interp_t *ip, const agc_timethet_in_t *in,
                         agc_timethet_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_conic_timethet(ip, in, out);
}

/* TIMERAD (CONIC_SUBROUTINES p.1272) */
agc_status_t rz_timerad(agc_interp_t *ip, const agc_timerad_in_t *in,
                        agc_timethet_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_conic_timerad(ip, in, out);
}

/* LAMBERT (CONIC_SUBROUTINES p.1266) */
agc_status_t rz_lambert(agc_interp_t *ip, const agc_lambert_in_t *in,
                        agc_lambert_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_conic_lambert(ip, in, out);
}

/* APSIDES (CONIC_SUBROUTINES p.1275) */
agc_status_t rz_apsides(agc_interp_t *ip, agc_vec_t r, agc_vec_t v, bool moon,
                        agc_dp_t *rapo, agc_dp_t *rperi, agc_dp_t *ecc)
{
    if (!AGC_ASSERT(ip != NULL && rapo != NULL)
        || !AGC_ASSERT(rperi != NULL && ecc != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_conic_apsides(ip, r, v, moon, rapo, rperi, ecc);
}

/* CSMPREC LEMPREC CSMCONIC LEMCONIC THISPREC OTHPREC
 * (INTEGRATION_INITIALIZATION p.1309ff): permanent state to TDEC1. */
agc_status_t rz_integ(agc_interp_t *ip, rz_integ_kind_t kind, agc_dp_t tdec1,
                      agc_state_vector_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL)
        || !AGC_ASSERT((int)kind >= 0 && kind <= RZ_OTHPREC)) {
        return AGC_STATUS_BAD_ARG;
    }
    switch (kind) {
    case RZ_CSMPREC:  return agc_integ_csmprec(ip, tdec1, out);
    case RZ_LEMPREC:  return agc_integ_lemprec(ip, tdec1, out);
    case RZ_CSMCONIC: return agc_integ_csmconic(ip, tdec1, out);
    case RZ_LEMCONIC: return agc_integ_lemconic(ip, tdec1, out);
    case RZ_THISPREC: return agc_integ_thisprec(ip, tdec1, out);
    case RZ_OTHPREC:  return agc_integ_othprec(ip, tdec1, out);
    default:          return AGC_STATUS_BAD_ARG;
    }
}

/* INTSTALL + INTEGRVS: the AGC always reserves integration first. */
agc_status_t rz_integrvs(agc_interp_t *ip, const agc_integrvs_in_t *in,
                         agc_state_vector_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_integ_intstall();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_integ_integrvs(ip, in, out);
}

/* TFFCONMU (TIME_OF_FREE_FALL p.1375) */
agc_status_t rz_tff_conmu(agc_interp_t *ip, agc_vec_t rone, agc_vec_t vone,
                          agc_dp_t tff_rtmu, agc_tff_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL) || !AGC_ASSERT(tff_rtmu > 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_tff_conmu(ip, rone, vone, tff_rtmu, ctx);
}

/* TFFRP/RA */
agc_status_t rz_tff_rpra(agc_interp_t *ip, agc_tff_ctx_t *ctx, agc_dp_t *rapo,
                         agc_dp_t *rper)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL)
        || !AGC_ASSERT(rapo != NULL && rper != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_tff_rpra(ip, ctx, rapo, rper);
}

/* CALCTPER */
agc_status_t rz_tff_calctper(agc_interp_t *ip, agc_tff_ctx_t *ctx,
                             agc_dp_t rterm, agc_dp_t *tper)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL) || !AGC_ASSERT(tper != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_tff_calctper(ip, ctx, rterm, tper);
}

/* CALCTFF */
agc_status_t rz_tff_calctff(agc_interp_t *ip, agc_tff_ctx_t *ctx,
                            agc_dp_t rterm, agc_dp_t *tff)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL) || !AGC_ASSERT(tff != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_tff_calctff(ip, ctx, rterm, tff);
}

/* RLS: landing site, moon-fixed, m 2^27 */
agc_vec_t rz_rls(void)
{
    agc_vec_t rls = agc_erase_rls();
    (void)AGC_ASSERT(rls.c[0] != AGC_DP_MIN);
    return rls;
}

/* GOFLASH / GOFLASHR / GOMARKF / GOXDSPF / PRIODSP ... with the three
 * astronaut returns (call CADR +1 terminate, +2 proceed, +3 enter). */
agc_status_t rz_flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                      uint8_t verb, uint8_t noun, agc_job_id_t term,
                      agc_job_id_t proceed, agc_job_id_t enter)
{
    if (!AGC_ASSERT(verb < 100U && noun < 100U)
        || !AGC_ASSERT((int)kind >= 0 && kind < AGC_DSKY_DISP_COUNT)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dsky_cont_t cont;
    cont.terminate = term;
    cont.proceed = proceed;
    cont.enter = enter;
    return agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont);
}

/* GOTOPOOH */
agc_status_t rz_gotopooh(void)
{
    agc_status_t st = agc_exec_gotopooh();
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_POODOO);
    return st;
}

/* ENDEXT */
agc_status_t rz_endext(void)
{
    agc_status_t st = agc_dsky_endext();
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;
}

/* DELAYJOB: sleep delay_cs, resume as cont */
agc_status_t rz_delayjob(uint16_t delay_cs, agc_job_id_t cont)
{
    if (!AGC_ASSERT(delay_cs > 0U) || !AGC_ASSERT(cont != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_exec_delayjob(delay_cs, cont);
}

/* LOADTIME (RTB): TIME2,TIME1 as a DP centisecond count, scale 2^28 */
agc_dp_t rz_loadtime(void)
{
    uint32_t t = agc_time_now_cs();
    (void)AGC_ASSERT(t <= (uint32_t)AGC_DP_MAX);
    return (agc_dp_t)(t & (uint32_t)AGC_DP_MAX);
}
