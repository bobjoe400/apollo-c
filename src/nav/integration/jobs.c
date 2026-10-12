/**********************************************************************
 * FILE NAME: jobs.c
 *
 * PURPOSE:
 *   The jobs and transition routines around the orbital integration:
 *   STATEINT task / STATINT1 job / STATEUP / ENDINT / NOINT (the P00
 *   integration of both permanent state vectors every 600 s), AVETOMID
 *   (thrusting -> coasting: INT/W, SETCOAST, OTHERS), MIDTOAV1 /
 *   MIDTOAV2 / CKMID2 / MID2 / TIMEINC / NOTIME (coasting -> thrusting)
 *   and INTWAKEU (end of a P27 state vector update).
 *
 * AGC SOURCE:
 *   Comanche055 INTEGRATION_INITIALIZATION.agc pp. 1313-1314,
 *   1327-1333; FRESH_START_AND_RESTART.agc pp. 209-210 (STATEUP)
 *   Luminary099 INTEGRATION_INITIALIZATION.agc pp. 1206-1207,
 *   1220-1226; FRESH_START_AND_RESTART.agc pp. 236-237 (differs: OTHERS
 *   instead of SETCOAST, no T-TO-ADD, 9x9 W)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale        Description
 *   ---------  ---------  ---  -----------  ----------------------------
 *   RN, VN     agc_vec_t  I    2^29 m, 2^7  AVERAGEG state (AVETOMID)
 *   PIPTIME    agc_dp_t   I    2^28 cs      its time
 *   TETCSM     agc_dp_t   I    2^28 cs      CSM state time (STATEUP)
 *   TRKMKCNT   agc_sp_t   O    -            cleared by MOVATHIS
 *   STATEFLG.. flags      I/O  -            integration control flags
 *
 * EXTERNAL REFERENCES:
 *   Name                    Description
 *   ----------------------  -----------------------------------------
 *   agc_exec_findvac        FINDVAC (exec.h)
 *   agc_waitlist_longcall   LONGCALL (exec.h)
 *   agc_phase_change        PHASCHNG (exec.h)
 *   agc_alarm               ALARM 1703 (NOTIME)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 01703 (NOTIME: MIDTOAV too late, TIG slipped).  Statuses of
 *   INTSTALL (BUSY: the job sleeps), FINDVAC, LONGCALL and the
 *   integration are returned to the job.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Job splitting: a job that may JOBSLEEP in INTSTALL is resumed as a
 *   separate job function (AGC_JOB_..._STALL) that repeats INTSTALL +1.
 *   A group-2 restart reruns the running job segment from its top
 *   (exec R11): STATINT1 then integrates to a later TDEC1 (now).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/exec/exec.h"
#include "agc/nav/integration.h"
#include "internal.h"

/* 600SECS 2DEC 60000 (oct 00003 25140): STATEINT period, cs.            */
#define INTEG_600SECS ((uint32_t) 60000) /* STATEINT period, cs */

/* 3CSECS 2DEC 3; 5.6SECS 2DEC 560 (CM): cs scale 2^28.                   */
static const agc_dp_t K_3CSECS  = 3;   /* 3 cs, cs 2^28 */
static const agc_dp_t K_5_6SECS = 560; /* 5.6 s, cs 2^28 */

/* STATINT1 keeps TDEC1 in its VAC push list across the INTSTALL sleeps. */
static agc_dp_t state_integration_tdec1; /* STATINT1's TDEC1 (VAC), cs 2^28 */
static agc_dp_t stateup_tdec1;           /* STATEUP's LM target, TETCSM, cs 2^28 */

/**********************************************************************
 * TIMEDELT (INTEGRATION_INITIALIZATION.agc C p. 1331, L p. 1225) --
 * computation time allowed for one MIDTOAV step
 *
 * FUNCTION NAME: timedelt
 *
 * RETURNS: agc_dp_t -- agc_dp_t TIMEDELT: 1250 (CM) or 2000 (LM), cs 2^28
 *********************************************************************/
static agc_dp_t timedelt(void)
{
    const agc_vehicle_t v = agc_state_vehicle(); /* this vehicle */

    _Static_assert(INTEG_TIMEDELT_CSM < INTEG_TIMEDELT_LM, "AGC constant");

    return ((v == AGC_VEHICLE_CSM) ? INTEG_TIMEDELT_CSM : INTEG_TIMEDELT_LM); /* CM 1250, LM 2000 cs */
}

/* ---- STATEINT (p. 1313-1314) ------------------------------------------ */

/**********************************************************************
 * STATEINT (INTEGRATION_INITIALIZATION.agc p. 1313) -- task: PHASCHNG OCT
 * 00052, FINDVAC PRIO5 STATINT1, TASKOVER
 *
 * FUNCTION NAME: agc_task_stateint
 *
 * RETURNS: agc_status_t -- the status of FINDVAC
 *********************************************************************/
agc_status_t agc_task_stateint(void)
{
    agc_phase_change(00052);

    return (agc_exec_findvac(AGC_INTEG_PRIO5, AGC_JOB_STATEINT_UPDATE));
}

/**********************************************************************
 * ENDINT (INTEGRATION_INITIALIZATION.agc p. 1314) -- CLEAR STATEFLG, PHASCHNG
 * OCT 20032, LONGCALL STATEINT in 600 s, ENDOFJOB
 *
 * FUNCTION NAME: endint
 *
 * RETURNS: agc_status_t -- the status of LONGCALL
 *********************************************************************/
static agc_status_t endint(void)
{
    agc_flag_put(AGC_FLAG_STATEFLG, false);
    agc_phase_change(020032);

    return (agc_waitlist_longcall(INTEG_600SECS, AGC_TASK_STATEINT));
}

/**********************************************************************
 * STATEUP second half (FRESH_START_AND_RESTART.agc C p. 210 / L p. 237) --
 * after the INTSTALL for the LM: LM permanent state (and LM W if valid) to
 * TETCSM with PRECIFLG set (4-time-step logic off), then STATEND / NO-INT:
 * CLEAR NODOFLAG, ENDINT
 *
 * FUNCTION NAME: stateup_lm_state
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (also when asleep in INTSTALL), or
 *                          the INTSTALL / integration / LONGCALL status
 *********************************************************************/
static agc_status_t stateup_lm_state(void)
{
    agc_interp_t ip; /* interpretive state of this job segment */

    agc_interp_init(&ip);
    agc_status_t st = agc_integ_install(0, AGC_JOB_STATEUP_STALL); /* status of INTSTALL, then INTEGRV */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* asleep          */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* STATEUP second half (CM p. 210 / LM p. 236) after the INTSTALL for the
     * LM: LM permanent state (and LM W if valid) to TETCSM with PRECIFLG set
     * (4-time-step logic off), then STATEND / NO-INT: CLEAR NODOFLAG, ENDINT. */
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* the integration erasables */

    agc_flag_put(AGC_FLAG_VINTFLAG, false);
    agc_integ_setiflgs();

    if (agc_flag_test(AGC_FLAG_RENDWFLG))
    {
        agc_flag_put(AGC_FLAG_DIM0FLAG, true);

        if (agc_state_vehicle() == AGC_VEHICLE_LM)
        {
            agc_flag_put(AGC_FLAG_D6OR9FLG, true); /* LM: 9x9 W       */
        }
    }

    agc_flag_put(AGC_FLAG_PRECIFLG, true);
    st = agc_integ_integrv(&ip, ctx, stateup_tdec1);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_put(AGC_FLAG_NODOFLAG, false); /* STATEND         */

    return (endint());
} /* stateup_lm_state */

/**********************************************************************
 * STATEUP first half (FRESH_START_AND_RESTART.agc C p. 209 / L p. 236) -- CSM
 * permanent state (and W when valid: CM ORBWFLAG 6x6; LM SURFFLAG and
 * RENDWFLG) to TDEC1 with the 4-time-step logic; on the surface STATEND, else
 * the LM half
 *
 * FUNCTION NAME: stateup
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  I/O  interpretive state: MPAC work and the
 *                                  overflow indicator
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the integration / ENDINT status
 *********************************************************************/
static agc_status_t stateup(agc_interp_t *ip)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();                   /* the integration erasables */
    const bool       csm = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* CSM: ORBWFLAG decides W */

    agc_flag_put(AGC_FLAG_VINTFLAG, true);
    const bool w = csm ? agc_flag_test(AGC_FLAG_ORBWFLAG) : (agc_flag_test(AGC_FLAG_SURFFLAG) && agc_flag_test(AGC_FLAG_RENDWFLG)); /* integrate W as well (DIM0FLAG) */

    if (w)
    {
        agc_flag_put(AGC_FLAG_DIM0FLAG, true);
    }

    agc_flag_put(AGC_FLAG_PRECIFLG, false);                                      /* DOINT          */
    const agc_status_t st = agc_integ_integrv(ip, ctx, state_integration_tdec1); /* status of INTEGRV */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        agc_flag_put(AGC_FLAG_NODOFLAG, false); /* STATEND/NO-INT */

        return (endint());
    }

    stateup_tdec1 = agc_erase_tetcsm(); /* DLOAD TETCSM   */

    return (stateup_lm_state());
}

/**********************************************************************
 * STATINT1 from INTSTALL +1 (INTEGRATION_INITIALIZATION.agc p. 1313) -- SET
 * NODOFLAG, SETIFLGS, STATEUP
 *
 * FUNCTION NAME: state_integration_grab
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (also when asleep in INTSTALL), or
 *                          the INTSTALL / STATEUP status
 *********************************************************************/
static agc_status_t state_integration_grab(void)
{
    agc_interp_t ip; /* interpretive state of this job segment */

    agc_interp_init(&ip);
    const agc_status_t st = agc_integ_install(0, AGC_JOB_STATEINT_UPDATE_STALL); /* status of INTSTALL */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* asleep          */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_put(AGC_FLAG_NODOFLAG, true);
    agc_integ_setiflgs();

    return (stateup(&ip));
}

/**********************************************************************
 * STATINT1 (INTEGRATION_INITIALIZATION.agc p. 1313) -- job: no integration
 * after V96 (QUITFLAG: NOINT); else TDEC1 = now, CM CLEAR V96ONFLG, INTSTALL
 *
 * FUNCTION NAME: agc_job_stateint_update
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of STATEUP
 *********************************************************************/
agc_status_t agc_job_stateint_update(void)
{
    if (agc_flag_test(AGC_FLAG_QUITFLAG))
    {
        agc_phase_change(00002); /* NOINT     */
        agc_flag_put(AGC_FLAG_QUITFLAG, false);

        return (AGC_STATUS_OK); /* ENDOFJOB  */
    }

    state_integration_tdec1 = agc_integ_loadtime();

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        agc_integ_csm_resume_idle_integration();
    }

    return (state_integration_grab());
}

/**********************************************************************
 * STATINT1 INTSTALL +1 (INTEGRATION_INITIALIZATION.agc p. 1313) -- STATINT1
 * resumed after INTSTALL slept
 *
 * FUNCTION NAME: agc_job_stateint_update_stall
 *
 * RETURNS: agc_status_t -- the status of state_integration_grab
 *********************************************************************/
agc_status_t agc_job_stateint_update_stall(void)
{
    return (state_integration_grab());
}

/**********************************************************************
 * STATEUP INTSTALL +1 (FRESH_START_AND_RESTART.agc C p. 210) -- STATEUP
 * resumed after its second INTSTALL slept
 *
 * FUNCTION NAME: agc_job_stateup_stall
 *
 * RETURNS: agc_status_t -- the status of stateup_lm_state
 *********************************************************************/
agc_status_t agc_job_stateup_stall(void)
{
    return (stateup_lm_state());
}

/* ---- AVETOMID (p. 1327-1328) ------------------------------------------ */

/**********************************************************************
 * INT/W (INTEGRATION_INITIALIZATION.agc p. 1328) -- the W matrix through the
 * burn with the pre-thrust state, to PIPTIME. CM: CSM 6x6; LM: LM 9x9. INTEGRV
 * releases INTSTALL.
 *
 * FUNCTION NAME: int_w
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_status_t -- the status of INTEGRV
 *********************************************************************/
static agc_status_t int_w(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* CSM: 6x6 W, VINTFLAG set */

    agc_flag_put(AGC_FLAG_DIM0FLAG, true);
    agc_flag_put(AGC_FLAG_AVEMIDSW, true); /* do not clobber RN VN    */
    agc_flag_put(AGC_FLAG_D6OR9FLG, !csm);
    agc_flag_put(AGC_FLAG_VINTFLAG, csm);

    return (agc_integ_integrv(ip, ctx, agc_erase_piptime()));
}

/**********************************************************************
 * SETCOAST body / MOVATHIS (INTEGRATION_INITIALIZATION.agc C p. 1327, L OTHERS
 * p. 1220) -- RN, VN (2^29, 2^7) shifted to the moon scale when `moon`,
 * PIPTIME -> RRECT RCV TET VRECT VCV (MINIRECT), into this vehicle's permanent
 * state; TRKMKCNT = 0
 *
 * FUNCTION NAME: move_this
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *   moon      bool               I    the state is moon-centred (2^27, 2^5)
 *
 * RETURNS: none
 *********************************************************************/
static void move_this(agc_interp_t *ip, agc_integ_ctx_t *ctx, bool moon)
{
    const int x2 = moon ? INTEG_PBODY_MOON : INTEG_PBODY_EARTH; /* X2 of RN, VN: 2 moon, 0 earth */

    ctx->sv.rrect = agc_vec_shl(ip, agc_erase_rn(), x2); /* VSR* 0,2 */
    ctx->sv.rcv   = ctx->sv.rrect;
    ctx->sv.tet   = agc_erase_piptime();
    agc_integ_minirect(ctx, agc_vec_shl(ip, agc_erase_vn(), x2));
    agc_integ_perm_set(agc_state_vehicle(), &ctx->sv);   /* MOVEACSM / MOVEALEM */
    agc_erase_set_trkmkcnt(0);                           /* SSP TRKMKCNT 0 */
}

/**********************************************************************
 * SETCOAST (INTEGRATION_INITIALIZATION.agc C pp. 1327-1328) -- CSM permanent
 * state from RN VN PIPTIME, CMOONFLG = AMOONFLG, then the LM (unless on the
 * surface) to PIPTIME
 *
 * FUNCTION NAME: setcoast
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the MOVATHIS / INTWAKE / INTEGRV
 *                          status
 *********************************************************************/
static agc_status_t setcoast(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const bool moon = agc_integ_csm_amoonflg(); /* AMOONFLG: RN, VN moon-centred */

    move_this(ip, ctx, moon); /* MOVATHIS */

    agc_flag_put(AGC_FLAG_CMOONFLG, moon);

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return (agc_integ_intwake()); /* FAZAB5        */
    }

    agc_integ_setiflgs();
    agc_flag_put(AGC_FLAG_VINTFLAG, false);

    return (agc_integ_integrv(ip, ctx, agc_erase_piptime()));
}

/**********************************************************************
 * OTHERS (INTEGRATION_INITIALIZATION.agc L p. 1220) -- CSM permanent state to
 * PIPTIME, then the LM permanent state from RN VN PIPTIME at the LMOONFLG
 * scale; FAZAB5.  The second INTSTALL: in C no other job runs in between, so
 * BUSY means the caller re-enters from the start.
 *
 * FUNCTION NAME: others
 *
 * ARGUMENTS:
 *   Argument    Type               I/O  Description (scale)
 *   ----------  -----------------  ---  ---------------------------------
 *   ip          agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                       overflow indicator
 *   ctx         agc_integ_ctx_t *  I/O  integration erasables (the set
 *                                       INTSTALL guards)
 *   resume_job  agc_job_id_t       I    job to continue as if INTSTALL is
 *                                       busy (JOBSLEEP)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the INTEGRV / INTSTALL /
 *                          MOVATHIS / INTWAKE status
 *********************************************************************/
static agc_status_t others(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_job_id_t resume_job)
{
    agc_flag_put(AGC_FLAG_VINTFLAG, true);
    agc_integ_setiflgs();
    agc_status_t st = agc_integ_integrv(ip, ctx, agc_erase_piptime()); /* status of INTEGRV, INTSTALL, MOVATHIS */

    if (st == AGC_STATUS_OK)
    {
        st = agc_integ_install(0, resume_job);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    move_this(ip, ctx, agc_flag_test(AGC_FLAG_LMOONFLG)); /* MOONTHIS */

    return (agc_integ_intwake()); /* FAZAB5   */
}

/**********************************************************************
 * AVETOMID (INTEGRATION_INITIALIZATION.agc pp. 1327-1328) -- INTSTALL; with a
 * valid W, INT/W and INTSTALL again; then SETCOAST (CM) / OTHERS (LM)
 *
 * FUNCTION NAME: agc_integ_avetomid
 *
 * ARGUMENTS:
 *   Argument    Type               I/O  Description (scale)
 *   ----------  -----------------  ---  ---------------------------------
 *   ip          agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                       overflow indicator
 *   ctx         agc_integ_ctx_t *  I/O  integration erasables (the set
 *                                       INTSTALL guards)
 *   resume_job  agc_job_id_t       I    job to continue as if INTSTALL is
 *                                       busy (JOBSLEEP)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BUSY (asleep in
 *                          INTSTALL), AGC_STATUS_BAD_ARG for a NULL ip, or a
 *                          called routine's status
 *********************************************************************/
agc_status_t agc_integ_avetomid(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_job_id_t resume_job)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_integ_install(0, resume_job); /* first INTSTALL */

    if ((st == AGC_STATUS_OK) && (agc_flag_test(AGC_FLAG_RENDWFLG) || agc_flag_test(AGC_FLAG_ORBWFLAG)))
    {
        st = int_w(ip, ctx);

        if (st == AGC_STATUS_OK)
        {
            st = agc_integ_install(0, resume_job); /* INTSTALL again */
        }
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return ((agc_state_vehicle() == AGC_VEHICLE_CSM) ? setcoast(ip, ctx) : others(ip, ctx, resume_job)); /* SETCOAST (CM) / OTHERS (LM) */
}

/* ---- MIDTOAV1 / MIDTOAV2 (p. 1329-1331) ------------------------------- */

/**********************************************************************
 * NOTIME (INTEGRATION_INITIALIZATION.agc p. 1331) -- too late: CALLOC +2
 * return, ALARM 1703; CM T-TO-ADD = TIMEDELT
 *
 * FUNCTION NAME: notime
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (T-TO-ADD); NULL
 *                                     in the LM
 *   slipped   bool *             O    set: the caller returns to CALLOC +2
 *********************************************************************/
static void notime(agc_integ_ctx_t *ctx, bool *slipped)
{
    agc_flag_put(AGC_FLAG_MID1FLAG, false);
    *slipped = true; /* INCR IRETURN1 */
    agc_alarm(AGC_INTEG_ALARM_NOTIME);

    if ((ctx != NULL) && (agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        ctx->t_to_add = INTEG_TIMEDELT_CSM;
    }
}

/**********************************************************************
 * ENTMID1 (INTEGRATION_INITIALIZATION.agc p. 1330) -- precision-integrate this
 * vehicle (no W, MIDAVFLG) to TDEC1; RN1 VN1 PIPTIME1 = RATT VATT TAT; MPAC =
 * TAT - TIME2,1.  DEVIATION: the AGC returns with INHINT still set; here
 * RELINT.
 *
 * FUNCTION NAME: midtoav_integrate
 *
 * ARGUMENTS:
 *   Argument  Type                       I/O  Description (scale)
 *   --------  -------------------------  ---  ---------------------------
 *   ip        agc_interp_t *             I/O  interpretive state: MPAC work
 *                                             and the overflow indicator
 *   ctx       agc_integ_ctx_t *          I/O  integration erasables (the set
 *                                             INTSTALL guards)
 *   tdec1     agc_dp_t                   I    TDEC1, the time to integrate
 *                                             to, cs 2^28
 *   out       agc_integ_midtoav_out_t *  O    RN1, VN1, PIPTIME1, MPAC, RTX1,
 *                                             RTX2
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of INTEGRV
 *********************************************************************/
static agc_status_t midtoav_integrate(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_integ_midtoav_out_t *out)
{
    agc_flag_put(AGC_FLAG_DIM0FLAG, false);
    agc_integ_thisvint();
    agc_flag_put(AGC_FLAG_INTYPFLG, false);
    agc_flag_put(AGC_FLAG_MIDAVFLG, true);
    const agc_status_t st = agc_integ_integrv(ip, ctx, tdec1); /* status of INTEGRV */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_flag_put(AGC_FLAG_MIDAVFLG, false);
    }
    else
    {
        agc_integ_csm_set_amoonflg(ctx->out.x2 != 0);
    }

    out->rtx1     = ctx->out.x1;
    out->rtx2     = ctx->out.x2;
    out->rn1      = ctx->out.ratt;
    out->vn1      = ctx->out.vatt;
    out->piptime1 = ctx->out.tat;
    agc_inhint();
    out->dt       = agc_dp_sub(ip, ctx->out.tat, agc_integ_loadtime());
    agc_relint();
    out->slipped  = ctx->notime;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * MIDTOAV1 (INTEGRATION_INITIALIZATION.agc pp. 1329-1330) -- coasting to
 * thrusting: integrate this vehicle to TDEC1 (TIG - 30 s), or, when too late
 * (NOTIME), to now + TIMEDELT in MIDTOAV2 steps
 *
 * FUNCTION NAME: agc_integ_midtoav_at_time
 *
 * ARGUMENTS:
 *   Argument    Type                       I/O  Description (scale)
 *   ----------  -------------------------  ---  -------------------------
 *   ip          agc_interp_t *             I/O  interpretive state: MPAC work
 *                                               and the overflow indicator
 *   ctx         agc_integ_ctx_t *          I/O  integration erasables (the
 *                                               set INTSTALL guards)
 *   tdec1       agc_dp_t                   I    TDEC1, the time to integrate
 *                                               to (TIG - 30 s), cs 2^28
 *   resume_job  agc_job_id_t               I    job to continue as if
 *                                               INTSTALL is busy (JOBSLEEP)
 *   out         agc_integ_midtoav_out_t *  O    RN1, VN1, PIPTIME1, MPAC,
 *                                               RTX1, RTX2, NOTIME
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BUSY (asleep in
 *                          INTSTALL), AGC_STATUS_BAD_ARG for a NULL pointer,
 *                          or the integration status
 *********************************************************************/
agc_status_t agc_integ_midtoav_at_time(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t tdec1, agc_job_id_t resume_job, agc_integ_midtoav_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const bool   csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);               /* CM: INTSTALL before the time check */
    agc_status_t st  = csm ? agc_integ_install(0, resume_job) : AGC_STATUS_OK; /* status of INTSTALL (CM now, LM below) */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_put(AGC_FLAG_MID1FLAG, true);
    agc_dp_t       t1      = tdec1;                /* TDEC1, cs 2^28 */
    bool           slipped = false;                /* NOTIME taken */
    const agc_dp_t now     = agc_integ_loadtime(); /* time now, cs 2^28 */

    if (agc_dp_sub(ip, t1, agc_dp_add(ip, now, timedelt())) < 0)
    {
        notime(csm ? ctx : NULL, &slipped);                    /* ALARM 1703    */
        const agc_dp_t add = csm ? ctx->t_to_add : timedelt(); /* T-TO-ADD (CM) or TIMEDELT (LM), cs 2^28 */

        t1 = agc_dp_add(ip, agc_integ_loadtime(), add); /* ENTMID2       */
    }

    st = csm ? AGC_STATUS_OK : agc_integ_install(0, resume_job); /* LM: INTSTALL after the time check */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ctx->notime = slipped;

    return (midtoav_integrate(ip, ctx, t1, out));
}

/**********************************************************************
 * MIDTOAV2 (INTEGRATION_INITIALIZATION.agc p. 1330) -- integrate this vehicle
 * to now + TIMEDELT (T-TO-ADD in the CM) in steps until the state is ahead of
 * the clock
 *
 * FUNCTION NAME: agc_integ_midtoav_now
 *
 * ARGUMENTS:
 *   Argument    Type                       I/O  Description (scale)
 *   ----------  -------------------------  ---  -------------------------
 *   ip          agc_interp_t *             I/O  interpretive state: MPAC work
 *                                               and the overflow indicator
 *   ctx         agc_integ_ctx_t *          I/O  integration erasables (the
 *                                               set INTSTALL guards)
 *   resume_job  agc_job_id_t               I    job to continue as if
 *                                               INTSTALL is busy (JOBSLEEP)
 *   out         agc_integ_midtoav_out_t *  O    RN1, VN1, PIPTIME1, MPAC,
 *                                               RTX1, RTX2
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BUSY (asleep in
 *                          INTSTALL), AGC_STATUS_BAD_ARG for a NULL pointer,
 *                          or the integration status
 *********************************************************************/
agc_status_t agc_integ_midtoav_now(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_job_id_t resume_job, agc_integ_midtoav_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const bool   csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);               /* CM: INTSTALL first */
    agc_status_t st  = csm ? agc_integ_install(0, resume_job) : AGC_STATUS_OK; /* status of INTSTALL (CM now, LM below) */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (csm)
    {
        ctx->t_to_add = INTEG_TIMEDELT_CSM;
    }

    agc_flag_put(AGC_FLAG_MID1FLAG, false);
    const agc_dp_t add = csm ? ctx->t_to_add : timedelt();          /* T-TO-ADD (CM) or TIMEDELT (LM), cs 2^28 */
    const agc_dp_t t1  = agc_dp_add(ip, agc_integ_loadtime(), add); /* ENTMID2; TDEC1 = now + step, cs 2^28 */

    st = csm ? AGC_STATUS_OK : agc_integ_install(0, resume_job); /* LM: INTSTALL now */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ctx->notime = false;

    return (midtoav_integrate(ip, ctx, t1, out));
}

/**********************************************************************
 * TIMEINC (INTEGRATION_INITIALIZATION.agc p. 1331) -- TDEC = now + T-TO-ADD
 * (CM) / TIMEDELT (LM)
 *
 * FUNCTION NAME: timeinc
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_TESTLOOP
 *********************************************************************/
static agc_integ_next_t timeinc(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const bool     csm = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* CM: T-TO-ADD, LM: TIMEDELT */
    const agc_dp_t add = csm ? ctx->t_to_add : timedelt();         /* step, cs 2^28 */

    ctx->tdec = agc_dp_add(ip, agc_integ_loadtime(), add);

    return (INTEG_NEXT_TESTLOOP);
}

/**********************************************************************
 * CKMID2 / MID2 (INTEGRATION_INITIALIZATION.agc pp. 1330-1331) -- entered from
 * ENDSTATE when MIDAVFLG.  MIDTOAV1: carry on while TDEC is at least TIMEDELT
 * in the future, else NOTIME and switch to MIDTOAV2 stepping.  MIDTOAV2: when
 * TDEC is reached (within 3 cs) stop if (CM) the state is at least 5.6 s ahead
 * of now, otherwise push TDEC on by TIMEDELT.
 *
 * FUNCTION NAME: agc_integ_midtoav_check
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_TESTLOOP (carry on) or
 *                              INTEG_NEXT_APCHK (done)
 *********************************************************************/
agc_integ_next_t agc_integ_midtoav_check(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (agc_flag_test(AGC_FLAG_MID1FLAG))
    {
        const agc_dp_t lim = agc_dp_add(ip, agc_integ_loadtime(), timedelt()); /* now + TIMEDELT, cs 2^28 */

        if (agc_dp_sub(ip, ctx->tdec, lim) >= 0)
        {
            return (INTEG_NEXT_TESTLOOP);
        }

        notime(ctx, &ctx->notime);

        return (timeinc(ip, ctx));
    }

    const agc_dp_t d = agc_dp_sub(ip, agc_dp_abs(agc_dp_sub(ip, ctx->tdec, ctx->sv.tet)), K_3CSECS); /* |TDEC - TET| - 3 cs, cs 2^28 */

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return ((d < 0) ? INTEG_NEXT_APCHK : timeinc(ip, ctx)); /* LM: done, or step on */
    }

    if (d >= 0)
    {
        return (timeinc(ip, ctx));
    }

    const agc_dp_t ahead = agc_dp_sub(ip, ctx->sv.tet, agc_integ_loadtime()); /* TET - now, cs 2^28 */

    if (agc_dp_sub(ip, ahead, K_5_6SECS) >= 0)
    {
        return (INTEG_NEXT_APCHK); /* 5.6 s to spare */
    }

    ctx->t_to_add = agc_dp_add(ip, ctx->t_to_add, INTEG_TIMEDELT_CSM);

    return (timeinc(ip, ctx));
}

/* ---- INTWAKEU (p. 1332-1333) ------------------------------------------ */

/**********************************************************************
 * INTWAKEU (INTEGRATION_INITIALIZATION.agc pp. 1332-1333) -- a P27 update left
 * RRECT VRECT (TET) in the temporary set and holds INTSTALL; finish it into
 * the CSM (+1/+2) or LM (-1/-2) permanent state (2 = moon sphere), release
 * INTSTALL, PHASCHNG OCT 04026. The AGC restarts group 6 at the return
 * address of that PHASCHNG, i.e. back in the caller (UPOUT, then UPOUT
 * +1); C names that point as the job `resume` (P27: the terminate job),
 * so a restart does not rerun the P27 segment from its top.
 *
 * FUNCTION NAME: agc_integ_intwakeu
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state: MPAC work and the
 *                                     overflow indicator
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the set INTSTALL
 *                                     guards)
 *   upsvflag  agc_sp_t *         I/O  UPSVFLAG: +1/+2 CSM, -1/-2 LM (2:
 *                                     moon), 0 none; cleared
 *   resume    agc_job_id_t       I    the caller's continuation after the
 *                                     return (group 6 restart point)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a NULL
 *                          pointer, or the ATOP / INTWAKE0 status
 *********************************************************************/
agc_status_t agc_integ_intwakeu(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_sp_t *upsvflag, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(upsvflag != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_interrupts_inhibited())
    {
        agc_relint(); /* RELINT        */
    }

    const agc_sp_t u = *upsvflag; /* UPSVFLAG, integer 2^14 */

    if (u != 0)
    {
        ctx->sv.rcv     = ctx->sv.rrect;
        agc_integ_minirect(ctx, ctx->sv.vrect);                        /* RECTIFY +13D  */
        const bool moon = (u == 2) || (u == -2);                       /* UPMNSVCD      */
        const int  x2   = moon ? INTEG_PBODY_MOON : INTEG_PBODY_EARTH; /* X2 of the update: 2 moon, 0 earth */

        agc_flag_put(AGC_FLAG_MOONFLAG, moon);

        if (u < 0)
        {
            agc_integ_atoplem(ip, ctx, x2); /* INTWAKLM      */
        }
        else
        {
            agc_integ_atopcsm(ip, ctx, x2);
            agc_flag_put(AGC_FLAG_ORBWFLAG, false);
        }

        agc_flag_put(AGC_FLAG_RENDWFLG, false); /* INTWAKEX      */
    }

    *upsvflag             = 0;                           /* INTWAKUP      */
    const agc_status_t sw = agc_integ_intwake_release(); /* status of INTWAKE0 */

    const agc_phase_var_t v = {AGC_RESTART_JOB, resume, AGC_TASK_NONE, 0, false, 0U}; /* D clear: PHSPRDT6 */

    agc_phase_change_var(04026, &v); /* TC INTWAKUQ: the return address */

    return (sw);
} /* agc_integ_intwakeu */
