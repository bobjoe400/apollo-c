/*
 * glue.c -- the single place where the rendezvous programs call into
 * other lanes (conic subroutines, integration, DSKY, executive, guidance
 * clock display). Each function is the AGC call set-up of the named entry.
 */
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "internal.h"

/* Conic body from RTX1: the AGC passes X1 = -2 (earth) or -10 (moon). */
agc_conic_body_t rz_body(const agc_rendz_t *rz)
{
    if (!AGC_ASSERT(rz != NULL)) {
        return AGC_CONIC_EARTH;
    }
    (void)AGC_ASSERT(rz->body.rtx1 == -2 || rz->body.rtx1 == -10);
    return (rz->body.rtx1 == -10) ? AGC_CONIC_MOON : AGC_CONIC_EARTH;
}

/* TIMETHET (CONIC_SUBROUTINES p.1269) */
agc_status_t rz_timethet(agc_interp_t *ip, const agc_timetheta_in_t *in,
                         agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_conic_timethet(ip, in, out);
}

/* TIMERAD (CONIC_SUBROUTINES p.1272) */
agc_status_t rz_timerad(agc_interp_t *ip, const agc_timerad_in_t *in,
                        agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_conic_timerad(ip, in, out);
}

/* LAMBERT (CONIC_SUBROUTINES p.1266); the LM calls it through SETITCTR
 * (ITERCTR 20, or 5 with AVEGFLAG), the CM directly (ITERCTR 20).       */
agc_status_t rz_lambert(agc_interp_t *ip, agc_lambert_in_t *in,
                        agc_lambert_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (rz_is_csm()) {
        in->iterctr = AGC_CONIC_LAMBERT_ITERCTR;
        return agc_conic_lambert(ip, in, out);
    }
    return agc_integ_setitctr(ip, in, out);
}

/* APSIDES (CONIC_SUBROUTINES p.1275) */
agc_status_t rz_apsides(agc_interp_t *ip, agc_vec_t r, agc_vec_t v,
                        agc_conic_body_t body, agc_apsides_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL)
        || !AGC_ASSERT(body == AGC_CONIC_EARTH || body == AGC_CONIC_MOON)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_conic_apsides(ip, r, v, body, out);
}

/* THISPREC / OTHPREC: CSMPREC or LEMPREC by vehicle. */
static rz_integ_kind_t rz_resolve_kind(rz_integ_kind_t kind)
{
    bool csm = rz_is_csm();
    (void)AGC_ASSERT((int)kind >= 0 && kind < RZ_INTEG_KIND_COUNT);
    if (kind == RZ_THISPREC) {
        return csm ? RZ_CSMPREC : RZ_LEMPREC;
    }
    if (kind == RZ_OTHPREC) {
        return csm ? RZ_LEMPREC : RZ_CSMPREC;
    }
    return kind;
}

/* CSMPREC LEMPREC CSMCONIC LEMCONIC THISPREC OTHPREC
 * (INTEGRATION_INITIALIZATION p.1318): permanent state to TDEC1. */
agc_status_t rz_integ(agc_interp_t *ip, rz_integ_kind_t kind, agc_dp_t tdec1,
                      agc_job_id_t resume, agc_integ_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL)
        || !AGC_ASSERT((int)kind >= 0 && kind < RZ_INTEG_KIND_COUNT)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    agc_status_t st;
    switch (rz_resolve_kind(kind)) {
    case RZ_CSMPREC:  st = agc_integ_csmprec(ip, ctx, tdec1, resume);  break;
    case RZ_LEMPREC:  st = agc_integ_lemprec(ip, ctx, tdec1, resume);  break;
    case RZ_CSMCONIC: st = agc_integ_csmconic(ip, ctx, tdec1, resume); break;
    case RZ_LEMCONIC: st = agc_integ_lemconic(ip, ctx, tdec1, resume); break;
    case RZ_THISPREC:
    case RZ_OTHPREC:
    case RZ_INTEG_KIND_COUNT:
    default:          st = AGC_STATUS_BAD_ARG;                          break;
    }
    if (st == AGC_STATUS_OK) {
        *out = ctx->out;
    }
    return st;
}

/* INTSTALL, then the INTEGRVS set-up common to INTINT, INITVEL, R31. */
agc_status_t rz_integrvs(agc_interp_t *ip, const rz_integrvs_in_t *in,
                         agc_job_id_t resume, agc_integ_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_integ_install(0, resume);              /* INTSTALL */
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    (void)rz_flag_put(AGC_FLAG_INTYPFLG, in->conic);
    (void)rz_flag_put(AGC_FLAG_MOONFLAG, in->moon);
    ctx->sv.rcv = in->rcv;
    ctx->sv.vcv = in->vcv;
    ctx->sv.tet = in->tet;
    st = agc_integ_integrvs(ip, ctx, in->tdec1);
    if (st == AGC_STATUS_OK) {
        *out = ctx->out;
    }
    return st;
}

/* RLS: landing site, moon-fixed, m 2^27 */
agc_vec_t rz_rls(void)
{
    agc_vec_t rls = agc_erase_rls();
    (void)AGC_ASSERT(rls.c[0] >= AGC_DP_MIN && rls.c[0] <= AGC_DP_MAX);
    return rls;
}

/* GOFLASH / GOFLASHR / GOMARKF / GOXDSPF ... with the three astronaut
 * returns (call CADR +1 terminate, +2 proceed, +3 enter). */
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
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY);
    return st;
}

/* DELAYJOB: sleep delay_cs, resume as cont */
agc_status_t rz_delayjob(uint16_t delay_cs, agc_job_id_t cont)
{
    if (!AGC_ASSERT(delay_cs > 0U) || !AGC_ASSERT(cont != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_svc_delayjob(delay_cs, cont);
}

/* LOADTIME (RTB): TIME2,TIME1 as a DP centisecond count, scale 2^28 */
agc_dp_t rz_loadtime(void)
{
    uint32_t t = agc_time_now_cs();
    (void)AGC_ASSERT(t < AGC_TIME_MODULUS_CS);
    return (agc_dp_t)(t & (uint32_t)AGC_DP_MAX);
}

/* TC ALARM / OCT code */
agc_status_t rz_alarm(agc_alarm_code_t code)
{
    (void)AGC_ASSERT(code != 0U);
    agc_status_t st = agc_alarm(code);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_ALARM);
    return st;
}

/* COMPTGO (CM P30-P37 p.636; LM P32-P35_P72-P75): start CLOKTASK (TTOGO
 * clock display) and protect it: 2PHSCHNG OCT 40036 (6.3SPOT CLOKTASK),
 * OCT 05024 + OCT 13000 (group 4 type C, PRIO13 job at the return). The
 * CM also sets TIMRFLAG and clears NVWORD1, and waits 1 cs; the LM clears
 * DISPDEX and waits 2 cs. DEVIATION: the CM routine of guid-powered
 * (agc_csm_comptgo) hard-wires its own P30 return job, so the rendezvous
 * programs use this copy with their own return job `cont`.             */
agc_status_t rz_comptgo(agc_job_id_t cont)
{
    if (!AGC_ASSERT(cont != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    uint16_t delay = 2U;
    if (rz_is_csm()) {
        (void)rz_vflag_set(RZ_VF_TIMRFLAG, true);
        agc_guid_state()->csm.nvword1 = 0;
        delay = 1U;
    } else {
        agc_guid_state()->lm.dispdex = 0;
    }
    agc_inhint();
    agc_status_t st = agc_waitlist_call(delay, AGC_TASK_CLOKTASK_RZ);
    agc_phase_var_t v;
    v.kind = AGC_RESTART_JOB;
    v.job = cont;
    v.task = AGC_TASK_NONE;
    v.prio_or_dt = 013;                                 /* PRIO13          */
    v.immediate = false;
    v.long_dt_cs = 0U;
    agc_status_t st2 = agc_phase_change2(040036U, 05024U, &v);
    agc_relint();
    (void)AGC_ASSERT(st2 == AGC_STATUS_OK || st2 == AGC_STATUS_BAD_ARG);
    return st;
}

/* DISPDEX = 0 (KILCLOCK, CLUPDATE). In the CM, DISPDEX EQUALS A
 * (ERASABLE_ASSIGNMENTS p.~), so "TS DISPDEX" stores nothing.        */
void rz_dispdex_zero(void)
{
    if (rz_is_csm()) {
        return;
    }
    agc_guid_state()->lm.dispdex = 0;
    (void)AGC_ASSERT(agc_guid_state()->lm.dispdex == 0);
}

/* OPTION1 = DSPTEMX = DSPTEM2 +1 (index 4), OPTION2 = +2 (index 5). */
uint16_t rz_option(int which)
{
    if (!AGC_ASSERT(which == 1 || which == 2)) {
        return 0U;
    }
    return agc_dsky_dsptem(3 + which);
}

void rz_set_option(int which, uint16_t v)
{
    if (!AGC_ASSERT(which == 1 || which == 2) || !AGC_ASSERT(v <= 077777U)) {
        return;
    }
    (void)agc_dsky_set_dsptem(3 + which, v);
}

/* A raw 15-bit ones' complement word as a signed SP value. */
static agc_sp_t rz_word_to_sp(uint16_t w)
{
    (void)AGC_ASSERT(w <= 077777U);
    if ((w & 040000U) != 0U) {
        return (agc_sp_t)(-(agc_sp_t)((uint16_t)~w & 037777U));
    }
    return (agc_sp_t)w;
}

/* A signed SP value as a raw 15-bit ones' complement word. */
static uint16_t rz_sp_to_word(agc_sp_t v)
{
    (void)AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX);
    if (v < 0) {
        return (uint16_t)(~(uint16_t)(-v) & 077777U);
    }
    return (uint16_t)v;
}

/* DSPTEM1, DSPTEM1 +1 (index 0, 1) as a DP. */
agc_dp_t rz_dsptem1_dp(void)
{
    agc_sp_t hi = rz_word_to_sp(agc_dsky_dsptem(0));
    agc_sp_t lo = rz_word_to_sp(agc_dsky_dsptem(1));
    return agc_dp_pack(hi, lo);
}

void rz_set_dsptem1_dp(agc_dp_t v)
{
    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX)) {
        return;
    }
    (void)agc_dsky_set_dsptem(0, rz_sp_to_word(agc_dp_high(v)));
    (void)agc_dsky_set_dsptem(1, rz_sp_to_word(agc_dp_low(v)));
}
