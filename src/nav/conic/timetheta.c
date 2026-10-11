/**********************************************************************
 * FILE NAME: timetheta.c
 *
 * PURPOSE:
 *   TIME-THETA (TIMETHET), its common exit COMMNOUT (shared with TIMERAD)
 *   and the abort COGAOVFL. Given a state (RVEC, VVEC) and a true-anomaly
 *   difference (SNTH, CSTH), TIMETHET returns the time of flight T and,
 *   unless RVSW is set, the state at the end of that arc.
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1269-1271 (description),
 *   p. 1289 (code: TIMETHET, COMMNOUT, COGAOVFL)
 *   Luminary099 CONIC_SUBROUTINES.agc pp. 1165-1167, 1185 (identical)
 *
 * EXTERNAL VARIABLES:
 *   Name      Type  I/O  Scale  Description
 *   --------  ----  ---  -----  ------------------------------------
 *   COGAFLAG  flag  O    -      set by COGAOVFL, cleared by COMMNOUT
 *   INFINFLG  flag  I    -      set by GETX (INFINITY)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_conic_param_      PARAM (geom.c)
 *   agc_conic_getx_       GETX (geom.c)
 *   agc_conic_deltime_    DELTIME (universal.c)
 *   agc_conic_newstate_   NEWSTATE (universal.c)
 *   agc_conic_abtconic_   ABTCONIC, POODOO 00607 (common.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ABTCONIC (POODOO 00607, AGC_STATUS_POODOO) when COGA overflows in
 *   PARAM (COGAFLAG) or GETX finds no conic closing through the angle
 *   (INFINFLG). AGC_STATUS_BAD_ARG for a NULL pointer or unknown body.
 *
 * ALGORITHM:
 *   PARAM gives P, R1A and COGA, GETX the universal variable x, DELTIME
 *   the time and NEWSTATE the new state.
 *********************************************************************/
#include "agc/core/assert.h"
#include "internal.h"

/**********************************************************************
 * AGC: none (C output record) -- copy T, the flags and the conic debris
 * from the shared context into the caller's output record
 *
 * FUNCTION NAME: agc_conic_time_out_fill_
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   ctx       const agc_conic_ctx_t * I    conic erasables after the call
 *   out       agc_conic_time_out_t *  O    T 2^28 cs, flags, P, R1A,
 *                                          COGA, CSTH, SNTH, UR1, U2, UN
 *********************************************************************/
void agc_conic_time_out_fill_(const agc_conic_ctx_t *ctx, agc_conic_time_out_t *out)
{
    out->t        = agc_conic_value_(ctx->t);
    out->infinflg = ctx->infinflg;
    out->cogaflag = ctx->cogaflag;
    out->r1       = agc_conic_value_(ctx->r1);
    out->r1a      = agc_conic_value_(ctx->r1a);
    out->p        = agc_conic_value_(ctx->p);
    out->coga     = agc_conic_value_(ctx->coga);
    out->csth     = agc_conic_value_(ctx->csth);
    out->snth     = agc_conic_value_(ctx->snth);
    out->ur1      = ctx->ur1;
    out->u2       = ctx->u2;
    out->un       = ctx->un;
}

/**********************************************************************
 * AGC: none (C output record) -- zero the outputs a call may not reach
 * (RNEW, VNEW when RVSW is set or an abort is taken)
 *
 * FUNCTION NAME: agc_conic_time_out_clear_
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   out       agc_conic_time_out_t *  O    record set to all zero
 *********************************************************************/
void agc_conic_time_out_clear_(agc_conic_time_out_t *out)
{
    agc_conic_time_out_t z = {0}; /* all-zero record */

    *out      = z;
    out->rnew = agc_vec_zero();
    out->vnew = agc_vec_zero();
}

/**********************************************************************
 * COGAOVFL (CONIC_SUBROUTINES.agc p. 1289) -- SETGO COGAFLAG ABTCONIC:
 * COGA overflowed in PARAM (flight path too close to radial). Also the
 * COGAOVFL exit of TIMERAD.
 *
 * FUNCTION NAME: agc_conic_cogaovfl_
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   ctx       agc_conic_ctx_t *       I/O  conic erasables; COGAFLAG set
 *   out       agc_conic_time_out_t *  O    debris as ABTCONIC leaves it
 *
 * RETURNS: agc_status_t -- AGC_STATUS_POODOO or AGC_STATUS_BAILOUT
 *          (ABTCONIC)
 *********************************************************************/
agc_status_t agc_conic_cogaovfl_(agc_conic_ctx_t *ctx, agc_conic_time_out_t *out)
{
    ctx->cogaflag = true;
    agc_conic_mirror_flag_(AGC_CONIC_FLAG_COGAFLAG, true);
    agc_conic_time_out_fill_(ctx, out);

    return (agc_conic_abtconic_());
}

/**********************************************************************
 * COMMNOUT (CONIC_SUBROUTINES.agc p. 1289) -- after GETX: ABTCONIC if
 * INFINFLG, else CLEAR COGAFLAG, DELTIME (T) and NEWSTATE unless RVSW.
 * Shared by TIMETHET and TIMERAD.
 *
 * FUNCTION NAME: agc_conic_commnout_
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   ip        agc_interp_t *          I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *       I/O  conic erasables from GETX
 *   rvsw      bool                    I    RVSW: time only, no new state
 *   out       agc_conic_time_out_t *  O    T 2^28 cs; RNEW 2^29 / 2^27 m,
 *                                          VNEW 2^7 / 2^5 m/cs (not RVSW)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_POODOO /
 *          AGC_STATUS_BAILOUT (INFINFLG, ABTCONIC)
 *********************************************************************/
agc_status_t agc_conic_commnout_(agc_interp_t *ip, agc_conic_ctx_t *ctx, bool rvsw, agc_conic_time_out_t *out)
{
    if (ctx->infinflg)
    { /* BON INFINFLG ABTCONIC */
        agc_conic_time_out_fill_(ctx, out);

        return (agc_conic_abtconic_());
    }

    ctx->cogaflag = false;                                  /* CLEAR COGAFLAG */
    agc_conic_mirror_flag_(AGC_CONIC_FLAG_COGAFLAG, false); /* mirror only */
    agc_conic_deltime_(ip, ctx);                            /* CALL DELTIME */

    if (!rvsw)
    { /* BON RVSW RTNTT */
        agc_conic_newstate_(ip, ctx, &out->rnew, &out->vnew);
    }

    agc_conic_time_out_fill_(ctx, out);

    return (AGC_STATUS_OK);
} /* agc_conic_commnout_ */

/**********************************************************************
 * TIMETHET (CONIC_SUBROUTINES.agc p. 1289, label TIMETHET) -- time of
 * flight through a given true-anomaly difference, and the end state
 *
 * FUNCTION NAME: agc_conic_timethet
 *
 * ARGUMENTS:
 *   Argument  Type                       I/O  Description (scale)
 *   --------  -------------------------  ---  -------------------------
 *   ip        agc_interp_t *             I/O  interpreter state (OVFIND)
 *   in        const agc_timetheta_in_t * I    RVEC, VVEC, SNTH and CSTH
 *                                             2^1, RVSW, central body
 *   out       agc_conic_time_out_t *     O    T 2^28 cs, RNEW, VNEW,
 *                                             flags and conic debris
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_POODOO for ABTCONIC
 *          (COGAFLAG or INFINFLG set in *out); AGC_STATUS_BAD_ARG for a
 *          NULL pointer or an unknown body
 *********************************************************************/
agc_status_t agc_conic_timethet(agc_interp_t *ip, const agc_timetheta_in_t *in, agc_conic_time_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_conic_time_out_clear_(out);
    agc_conic_ctx_t ctx;                                      /* conic erasables */
    agc_status_t    st = agc_conic_ctx_init_(&ctx, in->body); /* MUTABLE,1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_interp_clear_ovf(ip); /* SETPD 0, BOV +1 */
    ctx.vvec = in->vvec;
    agc_dp_words_t sngamma;   /* 36D sin(gamma), 2^1 (unused) */
    agc_tp_words_t m;         /* MPAC: P as PARAM leaves it */

    agc_conic_param_(ip, &ctx, in->rvec, in->vvec, &sngamma, &m); /* CALL PARAM */

    if (agc_interp_test_ovf(ip))
    { /* BOV COGAOVFL   */
        return (agc_conic_cogaovfl_(&ctx, out));
    }

    ctx.snth = agc_dp_to_words(in->snth);
    ctx.csth = agc_dp_to_words(in->csth);
    agc_conic_getx_(ip, &ctx, m); /* CALL GETX      */

    return (agc_conic_commnout_(ip, &ctx, in->rvsw, out));
} /* agc_conic_timethet */
