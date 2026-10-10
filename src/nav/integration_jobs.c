/*
 * integration_jobs.c -- the jobs and transition routines around the orbital
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
#include "agc/agc_assert.h"
#include "agc/agc_flags_common.h"
#include "agc/agc_integ.h"
#include "integ_internal.h"
#include "integ_pending.h"

/* 600SECS 2DEC 60000 (oct 00003 25140): STATEINT period, cs.            */
#define INTEG_600SECS ((uint32_t)60000)
/* 3CSECS 2DEC 3; 5.6SECS 2DEC 560 (CM): cs scale 2^28.                   */
static const agc_dp_t K_3CSECS = 3;
static const agc_dp_t K_5_6SECS = 560;

/* STATINT1 keeps TDEC1 in its VAC push list across the INTSTALL sleeps. */
static agc_dp_t statint1_tdec1;
static agc_dp_t stateup_tdec1;

static agc_dp_t timedelt(void)
{
    const agc_vehicle_t v = agc_state_vehicle();
    (void)AGC_ASSERT((v == AGC_VEHICLE_CSM) || (v == AGC_VEHICLE_LM));
    (void)AGC_ASSERT(INTEG_TIMEDELT_CSM < INTEG_TIMEDELT_LM);
    return (v == AGC_VEHICLE_CSM) ? INTEG_TIMEDELT_CSM : INTEG_TIMEDELT_LM;
}

/* ---- STATEINT (p. 1313-1314) ------------------------------------------ */

/* STATEINT task: PHASCHNG OCT 00052, FINDVAC PRIO5 STATINT1, TASKOVER.  */
agc_status_t agc_task_stateint(void)
{
    const agc_status_t st = agc_phase_change(2, 00052);
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    const agc_status_t sf = agc_exec_findvac(AGC_INTEG_PRIO5, AGC_JOB_STATINT1);
    (void)AGC_ASSERT(sf != AGC_STATUS_BAD_ARG);
    return sf;
}

/* ENDINT (p. 1314): CLEAR STATEFLG, PHASCHNG OCT 20032, LONGCALL STATEINT
 * in 600 s, ENDOFJOB.                                                   */
static agc_status_t endint(void)
{
    agc_integ_flag(AGC_FLAG_STATEFLG, false);
    const agc_status_t st = agc_phase_change(2, 020032);
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    const agc_status_t sl = agc_waitlist_longcall(INTEG_600SECS, AGC_TASK_STATEINT);
    (void)AGC_ASSERT(sl != AGC_STATUS_BAD_ARG);
    return sl;
}

/* STATEUP second half (CM p. 210 / LM p. 236) after the INTSTALL for the
 * LM: LM permanent state (and LM W if valid) to TETCSM with PRECIFLG set
 * (4-time-step logic off), then STATEND / NO-INT: CLEAR NODOFLAG, ENDINT. */
static agc_status_t stateup_lm(agc_interp_t *ip, agc_dp_t tdec1)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    (void)AGC_ASSERT(ip != NULL);
    agc_integ_flag(AGC_FLAG_VINTFLAG, false);
    (void)agc_integ_setiflgs();                         /* always OK       */
    if (agc_flag_test(AGC_FLAG_RENDWFLG)) {
        agc_integ_flag(AGC_FLAG_DIM0FLAG, true);
        if (agc_state_vehicle() == AGC_VEHICLE_LM) {
            agc_integ_flag(AGC_FLAG_D6OR9FLG, true);    /* LM: 9x9 W       */
        }
    }
    agc_integ_flag(AGC_FLAG_PRECIFLG, true);
    const agc_status_t st = agc_integ_integrv(ip, ctx, tdec1);
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    agc_integ_flag(AGC_FLAG_NODOFLAG, false);           /* STATEND         */
    return endint();
}

/* STATEUP second INTSTALL (+1 on resumption).                            */
static agc_status_t stateup_grab_lm(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    (void)AGC_ASSERT(stateup_tdec1 >= 0);
    const agc_status_t st = agc_integ_install(0, AGC_JOB_STATEUP_STALL);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;                           /* asleep          */
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    return stateup_lm(&ip, stateup_tdec1);
}

/* STATEUP first half: CSM permanent state (and W when valid: CM ORBWFLAG
 * 6x6; LM SURFFLAG and RENDWFLG) to TDEC1 with the 4-time-step logic.    */
static agc_status_t stateup(agc_interp_t *ip, agc_dp_t tdec1)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx();
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    agc_integ_flag(AGC_FLAG_VINTFLAG, true);
    const bool w = csm ? agc_flag_test(AGC_FLAG_ORBWFLAG)
                       : (agc_flag_test(AGC_FLAG_SURFFLAG) &&
                          agc_flag_test(AGC_FLAG_RENDWFLG));
    if (w) {
        agc_integ_flag(AGC_FLAG_DIM0FLAG, true);
    }
    agc_integ_flag(AGC_FLAG_PRECIFLG, false);            /* DOINT          */
    const agc_status_t st = agc_integ_integrv(ip, ctx, tdec1);
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    if (agc_flag_test(AGC_FLAG_SURFFLAG)) {
        agc_integ_flag(AGC_FLAG_NODOFLAG, false);         /* STATEND/NO-INT */
        return endint();
    }
    stateup_tdec1 = agc_erase_tetcsm();                   /* DLOAD TETCSM   */
    return stateup_grab_lm();
}

/* STATINT1 from INTSTALL +1: SET NODOFLAG, SETIFLGS, STATEUP.            */
static agc_status_t statint1_grab(void)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    const agc_status_t st = agc_integ_install(0, AGC_JOB_STATINT1_STALL);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;                           /* asleep          */
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    agc_integ_flag(AGC_FLAG_NODOFLAG, true);
    (void)agc_integ_setiflgs();                         /* always OK       */
    return stateup(&ip, statint1_tdec1);
}

/* STATINT1 job (p. 1313): no integration after V96 (QUITFLAG: NOINT);
 * else TDEC1 = now, CM CLEAR V96ONFLG, INTSTALL.                         */
agc_status_t agc_job_statint1(agc_job_ctx_t *jc)
{
    (void)AGC_ASSERT(jc != NULL);
    if (agc_flag_test(AGC_FLAG_QUITFLAG)) {
        const agc_status_t st = agc_phase_change(2, 00002);  /* NOINT     */
        (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
        agc_integ_flag(AGC_FLAG_QUITFLAG, false);
        return AGC_STATUS_OK;                                /* ENDOFJOB  */
    }
    statint1_tdec1 = agc_integ_loadtime();
    if (agc_state_vehicle() == AGC_VEHICLE_CSM) {
        (void)agc_integ_csm_clear_v96onflg();           /* always OK       */
    }
    return statint1_grab();
}

agc_status_t agc_job_statint1_stall(agc_job_ctx_t *jc)
{
    (void)AGC_ASSERT(jc != NULL);
    (void)AGC_ASSERT(statint1_tdec1 >= 0);
    return statint1_grab();
}

agc_status_t agc_job_stateup_stall(agc_job_ctx_t *jc)
{
    (void)AGC_ASSERT(jc != NULL);
    (void)AGC_ASSERT(stateup_tdec1 >= 0);
    return stateup_grab_lm();
}
