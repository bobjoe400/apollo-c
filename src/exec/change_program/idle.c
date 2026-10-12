/**********************************************************************
 * FILE NAME: idle.c
 *
 * PURPOSE:
 *   GOP00FIX (the P00 V37N99 request), INITSUB, the host power-on, and
 *   the few calls the restart code makes into higher layers (exec_up_*).
 *
 * AGC SOURCE:
 *   Comanche055 FRESH_START_AND_RESTART.agc pp. 194-209
 *   Luminary099 FRESH_START_AND_RESTART.agc pp. 223-236
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   FLAGWRD0-11   flags     O    bits           INITSUB / RAKE (CSM)
 *   EXTVBACT      uint16_t  I/O  bits           kept across CLEARMRK +2
 *   OPTIND        agc_sp_t  O    B0             INITSUB: -1
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_*            GOP00FIX display, STARTSB2 resets (io-dsky)
 *   agc_kalc_stoprate, agc_imu_setcoars    CSM restart up-calls
 *   agc_lm_dap_fresh_start, agc_lm_servidle   LM up-calls
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   The statuses of the up-calls; BAD_ARG for a bad vehicle or
 *   priority.
 *
 * NOTES:
 *   LAYERING: this is program-level code that the AGC keeps in the fresh
 *   start log section. It calls up into io (pinball), imu and the DAPs, so
 *   like dispatch.c it is a documented exception to "exec calls only core".
 *   Everything else in src/exec/ reaches higher layers only through the
 *   exec_up_* functions here, which unit tests replace.
 *
 *   V37 itself (MMCHANG .. V37XEQ) is in sequence.c, csm.c and lm.c beside this
 *   file.
 **********************************************************************/
#include "../internal.h"
#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define VP_VERB_CHANGE_PROGRAM 37U     /* V37: the verb GOP00FIX flashes */
#define VP_GO_IDLE_NOUN        99U     /* N99: the noun GOP00FIX flashes */
#define VP_RAKE_WORDS          12      /* FLAGTABL: FLAGWRD0 .. FLAGWRD11       */
#define VP_LM_F13_CLR          000440u /* LM FLAGWRD13: XOVINFLG bit 9, ULLAGFLG bit 6 */

/**********************************************************************
 * GOPROG3 / INITSUB (FRESH_START_AND_RESTART C pp.189, 203): STOPRATE
 * (CSM only: both callers test the vehicle).
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_up_stoprate(void)
{
    agc_kalc_stoprate(); /* CSM only: both callers test the vehicle */
}

/**********************************************************************
 * STRTIDLE (L p.1383): SERVIDLE puts SERVICER into its ground state and
 * requests the software restart (WHIMPER) that goes on to GOP00FIX (4.33).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the SERVIDLE status
 **********************************************************************/
agc_status_t exec_up_servidle(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(exec_cp_mmnumber() <= 99U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_servidle());
}

/**********************************************************************
 * SETCOARS (IMU_MODE_SWITCHING_ROUTINES.agc, called from GOPROG C
 * p.187): coarse-align enable after a restart with the NO ATT lamp on
 * (CSM GOPROG).
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_up_setcoars(void)
{
    agc_imu_setcoars(); /* CSM only: called from the CSM GOPROG tail */
}

/**********************************************************************
 * INITSUB / RAKE (C p.203): STOPRATE, restore the deadband (FLAGWRD9
 * MAXDBFLG: SETMAXDB, else SETMINDB), clear the FLAGTABL bits of
 * FLAGWRD0..11, set IMPULSW, OPTIND = -1.
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_initsub_csm(void)
{
    static const uint16_t flagtabl[VP_RAKE_WORDS] = { 0, 000040u, 006000u, 0, 0, 004140u, 010000u, 016000u, 0, 042000u, 0, 0 };

    exec_up_stoprate();

    if (agc_flag_test(AGC_FLAG_MAXDBFLG))
    {
        agc_csm_dap_setmaxdb(); /* SETMAXER          */
    }
    else
    {
        agc_csm_dap_setmindb();
    }

    for (int w = VP_RAKE_WORDS - 1; w >= 0; w--)
    { /* RAKE         */
        uint16_t v = (uint16_t) (agc_flagword_get(w) & ~flagtabl[w]); /* the flagword after RAKE */

        agc_flagword_set(w, v);
    }

    agc_flag_set(AGC_FLAG_IMPULSW);
    agc_erase_set_optind(-1); /* CA NEGONE / TS OPTIND (kill CDU optics) */
}

/**********************************************************************
 * GOP00FIX (FRESH_START_AND_RESTART C p.194 / L p.223): the V37 N99
 * flash; every response re-flashes it (TCF -3 / -4 / -5 back to
 * GOP00FIX), so only a new V37 selects a program.
 *
 * ARGUMENTS:
 *   ctx -- job context (core set 0); ctx->id is the job being run
 *
 * RETURNS: agc_status_t -- the display request's status; BAD_ARG for a priority out
 *          of range
 **********************************************************************/
static agc_status_t flash_program_request(const agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_GO_IDLE_FIX_REFLASH,
                            AGC_JOB_GO_IDLE_FIX_REFLASH,
                            AGC_JOB_GO_IDLE_FIX_REFLASH};

    if (!AGC_ASSERT((ctx->prio >= 1u) && (ctx->prio <= 037u)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn((uint8_t) VP_VERB_CHANGE_PROGRAM, (uint8_t) VP_GO_IDLE_NOUN), &cont));
}

/**********************************************************************
 * GOP00FIX (C p.194 / L p.223): P00 has been requested by a restart
 * with no active group or by POODOO: flash V37 N99 until the crew
 * selects a program.
 *
 * ARGUMENTS:
 *   ctx -- job context (core set 0); ctx->id is the job being run
 *
 * RETURNS: agc_status_t -- the display request's status, or the INITSUB / ALLCOAST
 *          status; BAD_ARG for a priority out of range
 **********************************************************************/
agc_status_t agc_job_go_idle_fix(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT((ctx->prio >= 1u) && (ctx->prio <= 037u)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        exec_initsub_csm(); /* TC INITSUB        */
    }
    else
    {
        uint16_t f13 = (uint16_t) (agc_flagword_get(13) & ~VP_LM_F13_CLR); /* FLAGWRD13 without XOVINFLG, ULLAGFLG */

        agc_flagword_set(13, f13); /* XOVINFLG, ULLAGFLG */
    }

    /* TC CLEARMRK +2: XDSPFLAG down but EXTVBACT kept (CLEARMRK +0 and
     * +1 zero it). io-dsky has one CLEARMRK entry, so EXTVBACT is saved
     * around it.                                                       */
    uint16_t extvbact = agc_erase_extvbact(); /* EXTVBACT kept across CLEARMRK +2 */

    agc_dsky_clearmrk();
    agc_erase_set_extvbact(extvbact);

    if (!AGC_ASSERT(agc_erase_extvbact() == extvbact))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (flash_program_request(ctx));
}

/**********************************************************************
 * GOP00FIX -3 (FRESH_START_AND_RESTART C p.194 / L p.223): the V37 N99
 * re-flash after any response.
 *
 * ARGUMENTS:
 *   ctx -- job context (core set 0); ctx->id is the job being run
 *
 * RETURNS: agc_status_t -- the display request's status; BAD_ARG for a wrong job id
 **********************************************************************/
agc_status_t agc_job_go_idle_fix_reflash(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id == AGC_JOB_GO_IDLE_FIX_REFLASH))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (flash_program_request(ctx));
}

/**********************************************************************
 * STARTSUB's resets of the erasables other modules own (C p.190 / L
 * p.219): DNTMGOTO = DNPHASE1 (the next DOWNRUPT restarts the current
 * downlist); LM RADMODES = channel 33 bit 6 + RMODINIT.
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_up_startsub(void)
{
    agc_telem_restart();

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_t4_radmodes_init();
    }
}

/**********************************************************************
 * STARTSB2's resets of the erasables other modules own (C pp.190-192 /
 * L pp.220-222): the DSKY / pinball state (DELAYLOC .. DSPCOUNT, NOUT,
 * MONSAVE, VERBREG, NOUNREG, DSPLIST), DELAYJOB's DELAYLOC, T4RUPT's
 * DSRUPTSW, SELFRET (the self-check restarts at SELFCHK) and EXTVBACT.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the DSKY reset status; BAD_ARG when called
 *          with interrupts enabled
 **********************************************************************/
agc_status_t exec_up_startsb2(void)
{
    agc_status_t st = agc_dsky_init(); /* DSKY reset outcome */

    agc_svc_startsub_reset();  /* DELAYLOC = 0      */
    agc_t4_startsub_reset();   /* DSRUPTSW = +0     */
    agc_selfcheck_restart();   /* SELFRET = SELFCHK */
    agc_erase_set_extvbact(0); /* EXTVBACT = 0      */

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * DOFSTART's resets of the erasables other modules own (C p.183-184 /
 * L p.211-212): self-check (SMODE; ERCOUNT on SLAP1), the uplink
 * (UPSVFLAG), the downlist (DNLSTCOD, DNTMGOTO), T4RUPT DSRUPTSW (SLAP1),
 * the mark counters; CSM optics (OPTIND = -1, OPTMODES = OPTINITF); LM
 * DAP erasables (DOFSTRT1; THRUST = -0 is DOFSTART's, in fresh_start.c).
 *
 * ARGUMENTS:
 *   slap1 -- true for the V36 SLAP1 fresh start (ERCOUNT, DSRUPTSW)
 *
 * RETURNS: agc_status_t -- the LM DAP reset status, else OK
 **********************************************************************/
agc_status_t exec_up_dofstart(bool slap1)
{
    agc_status_t st = AGC_STATUS_OK; /* LM DAP reset outcome */

    agc_selfcheck_fresh_start(slap1);
    agc_uplink_update_fresh_start();
    agc_telem_fresh_start(); /* DNLSTCOD = COSTALIN */

    if (slap1)
    {
        agc_t4_fresh_start_flush(); /* DSRUPTSW = -PRIO12 */

        if (agc_state_vehicle() == AGC_VEHICLE_LM)
        {
            agc_telem_slap1(); /* DUMPCNT = BIT12 */
        }
    }

    agc_erase_set_trkmkcnt(0);

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        st = agc_lm_dap_fresh_start();
    }
    else
    {
        agc_t4_optics_reset(true);
    }

    return (st);
}

/**********************************************************************
 * GOPROG's (hardware restart) optics reset (C p.188): OPTMODES =
 * (OPTMODES & OPTINITR) + BIT7.
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_up_goprog_optics(void)
{
    agc_t4_optics_reset(false);
}

/**********************************************************************
 * LM GOPROG (FRESH_START_AND_RESTART.agc L p.217): CA AGSWORD, TS
 * DNLSTCOD -- be sure of the correct downlist after a hardware restart.
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_up_goprog_downlist(void)
{
    agc_telem_select_list((int) agc_erase_agsword());
}

/* ---- host power-on --------------------------------------------------- */

/**********************************************************************
 * AGC: none -- Power-on for the host (no AGC counterpart: the erasable memory of a
 * real AGC keeps its contents through a fresh start). Core state and
 * channels for vehicle `v`, the erasable blocks the program lanes own
 * (alignment, rendezvous, landing / ascent, entry), then the executive
 * and the waitlist. The caller (src/sim/main.c start) then loads the
 * rope, the pad and the channels and raises GOJAM through agc_rupt_go,
 * whose GOPROG takes the fresh start (DOFSTART) or the restart the
 * phase tables call for, as the AGC does at power-on.
 *
 * ARGUMENTS:
 *   v -- AGC_VEHICLE_CSM or AGC_VEHICLE_LM
 *
 * RETURNS: agc_status_t -- the executive / waitlist start status; BAD_ARG for a
 *          bad vehicle
 **********************************************************************/
agc_status_t agc_power_on(agc_vehicle_t v)
{
    if (!AGC_ASSERT(v == AGC_VEHICLE_CSM || v == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_state_init(v);
    agc_align_ram_reset();
    agc_rendz_reset(agc_rendz_erasable());

    if (v == AGC_VEHICLE_LM)
    {
        agc_lm_landing_reset();
    }
    else
    {
        agc_entry_lexicon_reset();
    }

    agc_status_t st = agc_exec_init(); /* executive / waitlist start outcome */

    if (st == AGC_STATUS_OK)
    {
        st = agc_waitlist_init();
    }

    if (!AGC_ASSERT(agc_state_vehicle() == v))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}
