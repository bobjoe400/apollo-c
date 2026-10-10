/*
 * conic_timetheta.c -- TIME-THETA (TIMETHET), its common exit COMMNOUT and
 * the abort COGAOVFL (CONIC_SUBROUTINES.agc, Comanche 055 pp. 1269-1271
 * description, code p. 1289; Luminary 099 pp. 1165-1167, 1185; identical).
 *
 * Given a state (RVEC, VVEC) and a true-anomaly difference (SNTH, CSTH),
 * TIMETHET returns the time of flight T and, unless RVSW is set, the state
 * at the end of that arc. PARAM gives the conic parameters, GETX the
 * universal variable x, DELTIME the time and NEWSTATE the new state.
 */
#include "conic_internal.h"
#include "agc/agc_assert.h"

/* Copy the shared outputs and debris into *out. */
static void time_out_fill(const agc_conic_ctx_t *ctx, agc_conic_time_out_t *out)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(out != NULL);
    out->t = ctx->t;
    out->infinflg = ctx->infinflg;
    out->cogaflag = ctx->cogaflag;
    out->r1 = ctx->r1;
    out->r1a = ctx->r1a;
    out->p = ctx->p;
    out->coga = ctx->coga;
    out->csth = ctx->csth;
    out->snth = ctx->snth;
    out->ur1 = ctx->ur1;
    out->u2 = ctx->u2;
    out->un = ctx->un;
}

/* Clear the outputs that a call may not reach. */
static void time_out_clear(agc_conic_time_out_t *out)
{
    (void)AGC_ASSERT(out != NULL);
    agc_conic_time_out_t z = { 0 };
    *out = z;
    out->rnew = agc_vec_zero();
    out->vnew = agc_vec_zero();
    (void)AGC_ASSERT(out->t == 0);
}

/*
 * COGAOVFL (p. 1289): SETGO COGAFLAG ABTCONIC. Also used by TIMERAD.
 */
static agc_status_t time_cogaovfl(agc_conic_ctx_t *ctx, agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ctx != NULL && out != NULL)) { return AGC_STATUS_BAD_ARG; }
    ctx->cogaflag = true;
    agc_status_t st = agc_conic_mirror_flag_(AGC_CONIC_FLAG_COGAFLAG, true);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    time_out_fill(ctx, out);
    return agc_conic_abtconic_();
}

/*
 * COMMNOUT (CONIC_SUBROUTINES.agc p. 1289, label COMMNOUT)
 * After GETX: ABTCONIC if INFINFLG, else CLEAR COGAFLAG, DELTIME (T), and
 * NEWSTATE unless RVSW. Shared by TIMETHET and TIMERAD.
 */
agc_status_t agc_conic_commnout_(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                 bool rvsw, agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(out != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (ctx->infinflg) {                               /* BON INFINFLG ABTCONIC */
        time_out_fill(ctx, out);
        return agc_conic_abtconic_();
    }
    ctx->cogaflag = false;                             /* CLEAR COGAFLAG */
    (void)agc_conic_mirror_flag_(AGC_CONIC_FLAG_COGAFLAG, false); /* mirror only */
    agc_status_t st = agc_conic_deltime_(ip, ctx);     /* CALL DELTIME   */
    if (st != AGC_STATUS_OK) { return st; }
    if (!rvsw) {                                       /* BON RVSW RTNTT */
        st = agc_conic_newstate_(ip, ctx, &out->rnew, &out->vnew);
        if (st != AGC_STATUS_OK) { return st; }
    }
    time_out_fill(ctx, out);
    return AGC_STATUS_OK;
}

/*
 * TIMETHET (CONIC_SUBROUTINES.agc p. 1289, label TIMETHET).
 * See agc_nav.h for the status contract.
 */
agc_status_t agc_conic_timethet(agc_interp_t *ip, const agc_timetheta_in_t *in,
                                agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(out != NULL)) { return AGC_STATUS_BAD_ARG; }
    time_out_clear(out);
    agc_conic_ctx_t ctx;
    agc_status_t st = agc_conic_ctx_init_(&ctx, in->body);
    if (st != AGC_STATUS_OK) { return st; }
    (void)agc_interp_test_ovf(ip);                     /* SETPD 0, BOV +1 */
    ctx.vvec = in->vvec;
    agc_dp_t sngamma = 0;
    st = agc_conic_param_(ip, &ctx, in->rvec, in->vvec, &sngamma); /* CALL PARAM */
    if (st != AGC_STATUS_OK) { return st; }
    if (agc_interp_test_ovf(ip)) {                     /* BOV COGAOVFL   */
        return time_cogaovfl(&ctx, out);
    }
    ctx.snth = in->snth;
    ctx.csth = in->csth;
    st = agc_conic_getx_(ip, &ctx);                    /* CALL GETX      */
    if (st != AGC_STATUS_OK) { return st; }
    return agc_conic_commnout_(ip, &ctx, in->rvsw, out);
}

/* COGAOVFL entry for TIMERAD (conic_timeradius.c). */
agc_status_t agc_conic_cogaovfl_(agc_conic_ctx_t *ctx, agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ctx != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(out != NULL)) { return AGC_STATUS_BAD_ARG; }
    return time_cogaovfl(ctx, out);
}

/* Debris fill for TIMERAD (conic_timeradius.c). */
void agc_conic_time_out_fill_(const agc_conic_ctx_t *ctx, agc_conic_time_out_t *out)
{
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(out != NULL);
    time_out_fill(ctx, out);
}

/* Output reset for TIMERAD (conic_timeradius.c). */
void agc_conic_time_out_clear_(agc_conic_time_out_t *out)
{
    (void)AGC_ASSERT(out != NULL);
    (void)AGC_ASSERT(sizeof(*out) > 0);
    time_out_clear(out);
}
