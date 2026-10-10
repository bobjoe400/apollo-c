/*
 * jobs.c -- the jobs and transition routines around the orbital
 * integration (INTEGRATION_INITIALIZATION.agc, Comanche055 pp. 1313-1314,
 * 1327-1333; Luminary099 pp. 1206-1207, 1220-1226) and STATEUP/THISVINT
 * (FRESH_START_AND_RESTART.agc CM p. 209-210, LM p. 236-237).
 *
 *   STATEINT task / STATINT1 job / STATEUP / ENDINT / NOINT: the P00
 *       integration of both permanent state vectors every 600 s.
 *   AVETOMID: thrusting (AVERAGEG) -> coasting.
 *   MIDTOAV1 / MIDTOAV2 / CKMID2 / MID2 / TIMEINC / NOTIME: coasting ->
 *       thrusting, integrate this vehicle up to the start of AVERAGEG.
 *   INTWAKEU: end of a P27 state vector update.
 *
 * Job splitting: a job that may JOBSLEEP in INTSTALL is resumed as a
 * separate job function (AGC_JOB_..._STALL) that repeats INTSTALL +1.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/nav/integration.h"
#include "internal.h"
#include "pending.h"

/* 600SECS 2DEC 60000 (oct 00003 25140): STATEINT period, cs.            */
#define INTEG_600SECS ((uint32_t) 60000)
/* 3CSECS 2DEC 3; 5.6SECS 2DEC 560 (CM): cs scale 2^28.                   */
static const agc_dp_t K_3CSECS  = 3;
static const agc_dp_t K_5_6SECS = 560;

/* STATINT1 keeps TDEC1 in its VAC push list across the INTSTALL sleeps. */
static agc_dp_t statint1_tdec1;
static agc_dp_t stateup_tdec1;

static agc_dp_t timedelt(void)
{
    const agc_vehicle_t v = agc_state_vehicle();
    if (!AGC_ASSERT((v == AGC_VEHICLE_CSM) || (v == AGC_VEHICLE_LM)))
    {
        return 0; /* no valid result */
    }
    _Static_assert(INTEG_TIMEDELT_CSM < INTEG_TIMEDELT_LM, "AGC constant");
    return (v == AGC_VEHICLE_CSM) ? INTEG_TIMEDELT_CSM : INTEG_TIMEDELT_LM;
}

/* ---- STATEINT (p. 1313-1314) ------------------------------------------ */

/* STATEINT task: PHASCHNG OCT 00052, FINDVAC PRIO5 STATINT1, TASKOVER.  */
agc_status_t agc_task_stateint(void)
{
    agc_phase_change(2, 00052);
    const agc_status_t sf = agc_exec_findvac(AGC_INTEG_PRIO5, AGC_JOB_STATINT1);
    if (!AGC_ASSERT(sf != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return sf;
}

/* ENDINT (p. 1314): CLEAR STATEFLG, PHASCHNG OCT 20032, LONGCALL STATEINT
 * in 600 s, ENDOFJOB.                                                   */
static agc_status_t endint(void)
{
    agc_integ_flag(AGC_FLAG_STATEFLG, false);
    agc_phase_change(2, 020032);
    const agc_status_t sl =
        agc_waitlist_longcall(INTEG_600SECS, AGC_TASK_STATEINT);
    if (!AGC_ASSERT(sl != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return sl;
}

/* STATEUP second half (CM p. 210 / LM p. 236) after the INTSTALL for the
 * LM: LM permanent state (and LM W if valid) to TETCSM with PRECIFLG set
 * (4-time-step logic off), then STATEND / NO-INT: CLEAR NODOFLAG, ENDINT. */
static agc_status_t stateup_lm(agc_interp_t *ip, agc_dp_t tdec1)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    if (!AGC_ASSERT(ip != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_integ_flag(AGC_FLAG_VINTFLAG, false);
    agc_integ_setiflgs(); /* always OK       */
    if (agc_flag_test(AGC_FLAG_RENDWFLG))
    {
        agc_integ_flag(AGC_FLAG_DIM0FLAG, true);
        if (agc_state_vehicle() == AGC_VEHICLE_LM)
        {
            agc_integ_flag(AGC_FLAG_D6OR9FLG, true); /* LM: 9x9 W       */
        }
    }
    agc_integ_flag(AGC_FLAG_PRECIFLG, true);
    const agc_status_t st = agc_integ_integrv(ip, ctx, tdec1);
    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_integ_flag(AGC_FLAG_NODOFLAG, false); /* STATEND         */
    return endint();
}

/* STATEUP second INTSTALL (+1 on resumption).                            */
static agc_status_t stateup_grab_lm(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    if (!AGC_ASSERT(stateup_tdec1 >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_status_t st = agc_integ_install(0, AGC_JOB_STATEUP_STALL);
    if (st == AGC_STATUS_BUSY)
    {
        return AGC_STATUS_OK; /* asleep          */
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }
    return stateup_lm(&ip, stateup_tdec1);
}

/* STATEUP first half: CSM permanent state (and W when valid: CM ORBWFLAG
 * 6x6; LM SURFFLAG and RENDWFLG) to TDEC1 with the 4-time-step logic.    */
static agc_status_t stateup(agc_interp_t *ip, agc_dp_t tdec1)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    const bool       csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    agc_integ_flag(AGC_FLAG_VINTFLAG, true);
    const bool w = csm ? agc_flag_test(AGC_FLAG_ORBWFLAG) :
                         (agc_flag_test(AGC_FLAG_SURFFLAG) &&
                          agc_flag_test(AGC_FLAG_RENDWFLG));
    if (w)
    {
        agc_integ_flag(AGC_FLAG_DIM0FLAG, true);
    }
    agc_integ_flag(AGC_FLAG_PRECIFLG, false); /* DOINT          */
    const agc_status_t st = agc_integ_integrv(ip, ctx, tdec1);
    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        agc_integ_flag(AGC_FLAG_NODOFLAG, false); /* STATEND/NO-INT */
        return endint();
    }
    stateup_tdec1 = agc_erase_tetcsm(); /* DLOAD TETCSM   */
    return stateup_grab_lm();
}

/* STATINT1 from INTSTALL +1: SET NODOFLAG, SETIFLGS, STATEUP.            */
static agc_status_t statint1_grab(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    const agc_status_t st = agc_integ_install(0, AGC_JOB_STATINT1_STALL);
    if (st == AGC_STATUS_BUSY)
    {
        return AGC_STATUS_OK; /* asleep          */
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }
    agc_integ_flag(AGC_FLAG_NODOFLAG, true);
    agc_integ_setiflgs(); /* always OK       */
    return stateup(&ip, statint1_tdec1);
}

/* STATINT1 job (p. 1313): no integration after V96 (QUITFLAG: NOINT);
 * else TDEC1 = now, CM CLEAR V96ONFLG, INTSTALL.                         */
agc_status_t agc_job_statint1(agc_job_ctx_t *jc)
{
    if (!AGC_ASSERT(jc != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_flag_test(AGC_FLAG_QUITFLAG))
    {
        agc_phase_change(2, 00002); /* NOINT     */
        agc_integ_flag(AGC_FLAG_QUITFLAG, false);
        return AGC_STATUS_OK; /* ENDOFJOB  */
    }
    statint1_tdec1 = agc_integ_loadtime();
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        agc_integ_csm_resume_idle_integration(); /* always OK       */
    }
    return statint1_grab();
}

agc_status_t agc_job_statint1_stall(agc_job_ctx_t *jc)
{
    if (!AGC_ASSERT(jc != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(statint1_tdec1 >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return statint1_grab();
}

agc_status_t agc_job_stateup_stall(agc_job_ctx_t *jc)
{
    if (!AGC_ASSERT(jc != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(stateup_tdec1 >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return stateup_grab_lm();
}

/* ---- AVETOMID (p. 1327-1328) ------------------------------------------ */

/* TRKMKCNT (P20 tracking mark count) belongs to prog-rendz; requested as
 * agc_erase_set_trkmkcnt() (see the module note).  Kept here meanwhile. */
static agc_sp_t trkmkcnt_pending;

/* INTSTALL again later in the same call; in C no other job can run in
 * between, so BUSY here means the caller re-enters from the start.      */
static agc_status_t regrab(agc_job_id_t resume_job)
{
    const agc_status_t st = agc_integ_install(0, resume_job);
    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT((st != AGC_STATUS_OK) || agc_integ_area_busy(0)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* INT/W (p. 1328): the W matrix through the burn with the pre-thrust
 * state, to PIPTIME. CM: CSM 6x6; LM: LM 9x9. INTEGRV releases INTSTALL. */
static agc_status_t int_w(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    if (!AGC_ASSERT(agc_integ_area_busy(0)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_integ_flag(AGC_FLAG_DIM0FLAG, true);
    agc_integ_flag(AGC_FLAG_AVEMIDSW, true); /* do not clobber RN VN    */
    agc_integ_flag(AGC_FLAG_D6OR9FLG, !csm);
    agc_integ_flag(AGC_FLAG_VINTFLAG, csm);
    const agc_status_t st = agc_integ_integrv(ip, ctx, agc_erase_piptime());
    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* SETCOAST body / MOVATHIS (CM p. 1327, LM OTHERS): RN, VN (2^29, 2^7)
 * shifted to the moon scale when `moon`, PIPTIME -> RRECT RCV TET VRECT
 * VCV (MINIRECT), into this vehicle's permanent state; TRKMKCNT = 0.    */
static agc_status_t move_this(agc_interp_t *ip, agc_integ_ctx_t *ctx, bool moon)
{
    const int x2 = moon ? INTEG_PBODY_MOON : INTEG_PBODY_EARTH;
    if (!AGC_ASSERT(agc_integ_area_busy(0)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    ctx->sv.rrect = agc_integ_vshift(ip, agc_erase_rn(), x2); /* VSR* 0,2 */
    ctx->sv.rcv   = ctx->sv.rrect;
    ctx->sv.tet   = agc_erase_piptime();
    agc_integ_minirect(ctx, agc_integ_vshift(ip, agc_erase_vn(), x2));
    agc_status_t st = AGC_STATUS_OK;
    st = agc_integ_perm_set(agc_state_vehicle(), &ctx->sv);
    trkmkcnt_pending = 0; /* SSP     */
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* CM SETCOAST (p. 1327-1328): CSM permanent state from RN VN PIPTIME,
 * CMOONFLG = AMOONFLG, then the LM (unless on the surface) to PIPTIME.  */
static agc_status_t setcoast_csm(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const bool   moon = agc_integ_csm_amoonflg();
    agc_status_t st   = move_this(ip, ctx, moon);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_integ_flag(AGC_FLAG_CMOONFLG, moon);
    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return agc_integ_intwake(); /* FAZAB5        */
    }
    agc_integ_setiflgs(); /* always OK     */
    agc_integ_flag(AGC_FLAG_VINTFLAG, false);
    st = agc_integ_integrv(ip, ctx, agc_erase_piptime());
    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* LM OTHERS (Luminary p. 1220): CSM permanent state to PIPTIME, then the
 * LM permanent state from RN VN PIPTIME at the LMOONFLG scale; FAZAB5.  */
static agc_status_t others_lm(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                              agc_job_id_t resume_job)
{
    agc_integ_flag(AGC_FLAG_VINTFLAG, true);
    agc_integ_setiflgs(); /* always OK     */
    agc_status_t st = agc_integ_integrv(ip, ctx, agc_erase_piptime());
    if (st == AGC_STATUS_OK)
    {
        st = regrab(resume_job);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    st = move_this(ip, ctx, agc_flag_test(AGC_FLAG_LMOONFLG)); /* MOONTHIS */
    const agc_status_t sw = agc_integ_intwake(); /* FAZAB5        */
    return (st == AGC_STATUS_OK) ? sw : st;
}

agc_status_t agc_integ_avetomid(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                agc_job_id_t resume_job)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_integ_install(0, resume_job); /* first INTSTALL */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (agc_flag_test(AGC_FLAG_RENDWFLG) || agc_flag_test(AGC_FLAG_ORBWFLAG))
    {
        st = int_w(ip, ctx);
        if (st == AGC_STATUS_OK)
        {
            st = regrab(resume_job);
        }
        if (st != AGC_STATUS_OK)
        {
            return st;
        }
    }
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        return setcoast_csm(ip, ctx);
    }
    return others_lm(ip, ctx, resume_job);
}

/* ---- MIDTOAV1 / MIDTOAV2 (p. 1329-1331) ------------------------------- */

/* NOTIME (p. 1331): too late; CALLOC +2 return, ALARM 1703; CM T-TO-ADD
 * = TIMEDELT.  ctx is only touched by callers that hold INTSTALL.        */
static void notime(agc_integ_ctx_t *ctx, bool *slipped)
{
    agc_integ_flag(AGC_FLAG_MID1FLAG, false);
    *slipped              = true; /* INCR IRETURN1 */
    agc_alarm(AGC_INTEG_ALARM_NOTIME);
    if ((ctx != NULL) && (agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        ctx->t_to_add = INTEG_TIMEDELT_CSM;
    }
}

/* ENTMID1 (p. 1330): precision-integrate this vehicle (no W, MIDAVFLG)
 * to TDEC1; RN1 VN1 PIPTIME1 = RATT VATT TAT; MPAC = TAT - TIME2,1.
 * DEVIATION: the AGC returns with INHINT still set; here RELINT.        */
static agc_status_t entmid1(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                            agc_dp_t tdec1, agc_integ_midtoav_out_t *out)
{
    agc_integ_flag(AGC_FLAG_DIM0FLAG, false);
    agc_integ_thisvint(); /* always OK     */
    agc_integ_flag(AGC_FLAG_INTYPFLG, false);
    agc_integ_flag(AGC_FLAG_MIDAVFLG, true);
    const agc_status_t st = agc_integ_integrv(ip, ctx, tdec1);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_integ_flag(AGC_FLAG_MIDAVFLG, false);
    }
    else
    {
        agc_integ_csm_set_amoonflg(ctx->out.x2 != 0); /* always OK */
    }
    out->rtx1     = ctx->out.x1;
    out->rtx2     = ctx->out.x2;
    out->rn1      = ctx->out.ratt;
    out->vn1      = ctx->out.vatt;
    out->piptime1 = ctx->out.tat;
    agc_inhint();
    out->dt = agc_dp_sub(ip, ctx->out.tat, agc_integ_loadtime());
    agc_relint();
    out->slipped = ctx->notime;
    return AGC_STATUS_OK;
}

agc_status_t agc_integ_midtoav1(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                agc_dp_t tdec1, agc_job_id_t resume_job,
                                agc_integ_midtoav_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_valid_ctx(ctx)) ||
        !AGC_ASSERT(out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const bool   csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    agc_status_t st  = csm ? agc_integ_install(0, resume_job) : AGC_STATUS_OK;
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_integ_flag(AGC_FLAG_MID1FLAG, true);
    agc_dp_t       t1      = tdec1;
    bool           slipped = false;
    const agc_dp_t now     = agc_integ_loadtime();
    if (agc_dp_sub(ip, t1, agc_dp_add(ip, now, timedelt())) < 0)
    {
        notime(csm ? ctx : NULL, &slipped); /* ALARM 1703    */
        const agc_dp_t add = csm ? ctx->t_to_add : timedelt();
        t1 = agc_dp_add(ip, agc_integ_loadtime(), add); /* ENTMID2       */
    }
    st = csm ? AGC_STATUS_OK : agc_integ_install(0, resume_job);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    ctx->notime = slipped;
    return entmid1(ip, ctx, t1, out);
}

agc_status_t agc_integ_midtoav2(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                agc_job_id_t             resume_job,
                                agc_integ_midtoav_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_valid_ctx(ctx)) ||
        !AGC_ASSERT(out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const bool   csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    agc_status_t st  = csm ? agc_integ_install(0, resume_job) : AGC_STATUS_OK;
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    if (csm)
    {
        ctx->t_to_add = INTEG_TIMEDELT_CSM;
    }
    agc_integ_flag(AGC_FLAG_MID1FLAG, false);
    const agc_dp_t add = csm ? ctx->t_to_add : timedelt();
    const agc_dp_t t1 = agc_dp_add(ip, agc_integ_loadtime(), add); /* ENTMID2 */
    st                = csm ? AGC_STATUS_OK : agc_integ_install(0, resume_job);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    ctx->notime = false;
    return entmid1(ip, ctx, t1, out);
}

/* TIMEINC (p. 1331): TDEC = now + T-TO-ADD (CM) / TIMEDELT (LM).         */
static agc_integ_next_t timeinc(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const bool     csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    const agc_dp_t add = csm ? ctx->t_to_add : timedelt();
    if (!AGC_ASSERT(add >= 0))
    {
        return INTEG_NEXT_FAIL;
    }
    ctx->tdec = agc_dp_add(ip, agc_integ_loadtime(), add);
    if (!AGC_ASSERT(ctx->tdec >= 0))
    {
        return INTEG_NEXT_FAIL;
    }
    return INTEG_NEXT_TESTLOOP;
}

/*
 * CKMID2 / MID2 (p. 1330-1331), entered from ENDSTATE when MIDAVFLG.
 * MIDTOAV1: carry on while TDEC is at least TIMEDELT in the future, else
 * NOTIME and switch to MIDTOAV2 stepping.  MIDTOAV2: when TDEC is reached
 * (within 3 cs) stop if (CM) the state is at least 5.6 s ahead of now,
 * otherwise push TDEC on by TIMEDELT.
 */
agc_integ_next_t agc_integ_ckmid2(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_valid_ctx(ctx)))
    {
        return INTEG_NEXT_FAIL;
    }
    if (agc_flag_test(AGC_FLAG_MID1FLAG))
    {
        const agc_dp_t lim = agc_dp_add(ip, agc_integ_loadtime(), timedelt());
        if (agc_dp_sub(ip, ctx->tdec, lim) >= 0)
        {
            return INTEG_NEXT_TESTLOOP;
        }
        notime(ctx, &ctx->notime);
        return timeinc(ip, ctx);
    }
    const agc_dp_t d = agc_dp_sub(
        ip, agc_dp_abs(agc_dp_sub(ip, ctx->tdec, ctx->sv.tet)), K_3CSECS);
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return (d < 0) ? INTEG_NEXT_APCHK : timeinc(ip, ctx);
    }
    if (d >= 0)
    {
        return timeinc(ip, ctx);
    }
    const agc_dp_t ahead = agc_dp_sub(ip, ctx->sv.tet, agc_integ_loadtime());
    if (agc_dp_sub(ip, ahead, K_5_6SECS) >= 0)
    {
        return INTEG_NEXT_APCHK; /* 5.6 s to spare */
    }
    ctx->t_to_add = agc_dp_add(ip, ctx->t_to_add, INTEG_TIMEDELT_CSM);
    return timeinc(ip, ctx);
}

/* ---- INTWAKEU (p. 1332-1333) ------------------------------------------ */

/* INTWAKEU: a P27 update left RRECT VRECT (TET) in the temporary set and
 * holds INTSTALL; finish it into the CSM (+1/+2) or LM (-1/-2) permanent
 * state (2 = moon sphere), release INTSTALL, PHASCHNG OCT 04026.         */
agc_status_t agc_integ_intwakeu(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                agc_sp_t *upsvflag)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(agc_integ_valid_ctx(ctx)) ||
        !AGC_ASSERT(upsvflag != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_interrupts_inhibited())
    {
        agc_relint(); /* RELINT        */
    }
    agc_status_t   st = AGC_STATUS_OK;
    const agc_sp_t u  = *upsvflag;
    if (u != 0)
    {
        ctx->sv.rcv = ctx->sv.rrect;
        agc_integ_minirect(ctx, ctx->sv.vrect); /* RECTIFY +13D  */
        const bool moon = (u == 2) || (u == -2);       /* UPMNSVCD      */
        const int  x2   = moon ? INTEG_PBODY_MOON : INTEG_PBODY_EARTH;
        agc_integ_flag(AGC_FLAG_MOONFLAG, moon);
        if (u < 0)
        {
            st = agc_integ_atoplem(ip, ctx, x2); /* INTWAKLM      */
        }
        else
        {
            st = agc_integ_atopcsm(ip, ctx, x2);
            agc_integ_flag(AGC_FLAG_ORBWFLAG, false);
        }
        agc_integ_flag(AGC_FLAG_RENDWFLG, false); /* INTWAKEX      */
    }
    *upsvflag             = 0; /* INTWAKUP      */
    const agc_status_t sw = agc_integ_intwake0();
    agc_phase_change(6, 04026);
    return (st == AGC_STATUS_OK) ? sw : st;
}
