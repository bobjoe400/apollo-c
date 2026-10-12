/**********************************************************************
 * FILE NAME: sequence.c
 *
 * PURPOSE:
 *   V37 (MMCHANG): change of major mode, CANV37, POOH, NOUVEAU and
 *   V37XEQ, the parts the two programs share.
 *
 * AGC SOURCE:
 *   Comanche055 FRESH_START_AND_RESTART.agc pp. 197-209 (V37 .. V37XEQC,
 *   FCADRMM1, PREMM1, DNLADMM1)
 *   Luminary099 FRESH_START_AND_RESTART.agc pp. 227-236
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   MODREG        agc_sp_t  I/O  B0 integer     the major mode
 *   IMODES30      uint16_t  I    bits           bit 6: IMU being initialised
 *   PHSPRDT2/4    priority  O    PRIOnn >> 9    POOH / V37XEQ presets
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_integ_install     INTSTALL (nav-integ)
 *   agc_dsky_releasedsp, agc_dsky_pinbrnch, agc_dsky_falton   (io-dsky)
 *   agc_telem_select_list DNLSTCOD (io-telemetry)
 *   csm.c, lm.c           the vehicle parts
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 1520 (V37 not permitted), OPERATOR ERROR (no such program),
 *   the BAILOUT of FINDVAC; the statuses of the calls.
 *
 * NOTES:
 *   Vehicle-specific steps are in csm.c and lm.c beside this file.
 *
 *   The AGC changes program by a software restart: CANV37 sets TEMPFLSH =
 *   R00AD and phase 4.1, NOUVEAU sets TEMPFLSH = V37QCAD (V37XEQ +3) and
 *   goes to GOPROG2; the 4.1 restart is INITDSP (FINDVAC PRIO14, PRIOCHNG
 *   to RESTREG = PRIO30), which jumps to TEMPFLSH -3: ROC (CANV37 again)
 *   or V37XEQ.
 *   DEVIATION: the C INITDSP re-issues the last display and cannot be
 *   pointed at TEMPFLSH, so group 4 gets type C job phases instead: CANV37
 *   restarts as itself (AGC_JOB_CHANGE_PROGRAM_CONTINUE), NOUVEAU as V37XEQ
 *   (AGC_JOB_CHANGE_PROGRAM_START), both at PRIO30, and ENEMA is requested:
 *   the software restart that follows starts V37XEQ, which starts the
 *   program (FCADRMM1), presets PHSPRDT4 and sets MODREG (NEWMODEA). P00
 *   sets MODREG and restarts with no group 4. PHASE4 therefore reads
 *   05024 (type C) where the AGC shows 4.1.
 *
 *   LAYERING: as the rest of change_program/, exec calling up (pinball,
 *   integration, telemetry, the DAPs and SERVICER); see ARCHITECTURE.md.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>

#include "../internal.h"
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/integration.h"

#define CP_BIT6       000040u             /* IMODES30 bit 6: IMU being initialised */
#define CP_DPS_ABORT  70U                 /* P70: SETUP70 */
#define CP_APS_ABORT  71U                 /* P71: SETUP71 */
#define CP_GROUP4_JOB ((uint16_t) 005024) /* type C, D, J, group 4      */
#define CP_PRIO5      ((agc_prio_t) 05)   /* PHSPRDT2 for P00 (POOH)    */

static uint8_t mmnumber; /* MMNUMBER: the requested major mode       */
static int     mindex;   /* MINDEX: its index in the mode tables       */

/**********************************************************************
 * MMNUMBER (FRESH_START_AND_RESTART C p.197 / L p.227): the major mode
 * V37 asked for.
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint8_t -- C(MMNUMBER)
 **********************************************************************/
uint8_t exec_cp_mmnumber(void)
{
    return (mmnumber);
}

/**********************************************************************
 * FCADRMM1 / PREMM1 / DNLADMM1 of the vehicle (FRESH_START_AND_RESTART
 * C pp.206-209 / L pp.234-236).
 *
 * ARGUMENTS:
 *   n -- receives the number of table rows
 *
 * RETURNS: const exec_cp_mode_t * -- the vehicle's mode table
 **********************************************************************/
static const exec_cp_mode_t *cp_modes(int *n)
{
    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? exec_cp_modes_lm(n) : exec_cp_modes_csm(n)); /* LM : CSM table */
}

/**********************************************************************
 * CANTR00 / V37BAD (C p.198 / L p.228): alarm 1520, release the DSKY.
 * V37NONO: FALTON (OPERATOR ERROR lamp), then V37BAD; NOT_FOUND.
 *
 * ARGUMENTS:
 *   alarm_1520 -- true: CANTR00 (alarm 1520), false: V37NONO (OPR ERR)
 *
 * RETURNS: agc_status_t -- ALARM (1520) or NOT_FOUND (no such program); the RELDSP /
 *          PINBRNCH status when one fails
 **********************************************************************/
static agc_status_t cp_refuse(bool alarm_1520)
{
    agc_status_t result = AGC_STATUS_NOT_FOUND; /* ALARM 1520 or NOT_FOUND */

    if (alarm_1520)
    {
        agc_alarm(AGC_ALARM_01520);
        result = AGC_STATUS_ALARM;
    }
    else
    {
        agc_dsky_falton(); /* V37NONO: TC FALTON    */
    }

    agc_status_t st = agc_dsky_releasedsp(); /* TC RELDSP outcome */

    if (st == AGC_STATUS_OK)
    {
        st = agc_dsky_pinbrnch(); /* CADR PINBRNCH     */
    }

    return ((st != AGC_STATUS_OK) ? st : result); /* a failure first, else ALARM / NOT_FOUND */
}

/**********************************************************************
 * ISITP00 / CHECKTAB / ISSERVON (C pp.197-199 / L pp.227-228).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the CANV37 or refusal status; OK when SERVICER will return
 *          to CANV37
 **********************************************************************/
static agc_status_t cp_select(void)
{
    if (mmnumber != 0U)
    {
        if (agc_flag_test(AGC_FLAG_NODOFLAG))
        {
            return (cp_refuse(true)); /* CANTR00           */
        }

        /* CHECKTAB / AGAINMM (C p.198 / L p.228): MINDEX of `mm`, -1 if none.  */
        int                   n = 0;            /* rows in the mode table */
        const exec_cp_mode_t *t = cp_modes(&n); /* the mode table */
        int                   k = -1;           /* MINDEX, -1 none */

        for (int i = 0; i < n && k < 0; i++)
        {
            k = (t[i].mm == mmnumber) ? i : -1; /* AGAINMM: this row, else go on */
        }

        mindex = k;

        if (mindex < 0)
        {
            return (cp_refuse(false)); /* V37NONO           */
        }
    }

    if (!agc_flag_test(AGC_FLAG_SERVICER_RUNNING))
    {
        return (agc_job_change_program_continue());
    }

    agc_inhint(); /* AVEGFLAG down: SERVICER returns to CANV37 */

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        exec_cp_stop_servicer_lm();
    }
    else
    {
        exec_cp_stop_servicer_csm();
    }

    agc_relint();

    return (AGC_STATUS_OK); /* ENDOFJOB          */
} /* cp_select */

/**********************************************************************
 * V37 (C p.197 / L p.227): request to change to major mode new_mm, in
 * the pinball job. Returns ALARM (1520, not permitted), NOT_FOUND (no
 * such program), or the status of the change (OK when the job ends or
 * is put to sleep until SERVICER or the integration lets it go on).
 *
 * ARGUMENTS:
 *   new_mm -- MMNUMBER, the requested major mode (0..99)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the routine it calls;
 *          BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_change_program(uint8_t new_mm)
{
    if (!AGC_ASSERT(new_mm <= 99U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    mmnumber = new_mm;                                /* TS MMNUMBER       */
    mindex   = -1;
    bool lm  = agc_state_vehicle() == AGC_VEHICLE_LM; /* Luminary rope */

    if ((agc_erase_imodes30() & CP_BIT6) != 0U)
    { /* IMU initialising  */
        return (cp_refuse(true));
    }

    if (lm && (mmnumber == CP_DPS_ABORT || mmnumber == CP_APS_ABORT))
    {
        return (exec_cp_setup_abort(mmnumber == CP_APS_ABORT));
    }

    if (!lm)
    {
        agc_status_t st = exec_cp_engine_off_to_idle(); /* engine on / TVC  */

        if (st != AGC_STATUS_NOT_FOUND)
        {
            return (st); /* waits, then P00 (agc_job_change_program_jets_off) */
        }
    }

    return (cp_select());
}

/**********************************************************************
 * R00TOP00 continuation (C p.198 after DELAYJOB 5): MMNUMBER = 0.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the ISITP00 status
 **********************************************************************/
agc_status_t exec_cp_after_engine_off(void)
{
    mmnumber = 0;

    return (cp_select()); /* ISITP00: P00, check SERVICER */
}

/**********************************************************************
 * AGC: none -- GOGOPROG: the software restart; for a new program, with group 4 set to
 * restart V37XEQ at PRIO30 (see the file header).
 *
 * ARGUMENTS:
 *   execute_immediately -- true: group 4 restarts V37XEQ (a new program)
 *
 * RETURNS: none
 **********************************************************************/
static void cp_gogoprog(bool execute_immediately)
{
    if (execute_immediately)
    {
        agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_CHANGE_PROGRAM_START, AGC_TASK_NONE, (int16_t) EXEC_PRIO30, false, 0U};

        agc_phase_change_var(CP_GROUP4_JOB, &v); /* TEMPFLSH = V37QCAD */
    }

    agc_exec_enema(); /* TC POSTJUMP CADR GOPROG2 */
}

/**********************************************************************
 * POOH (C p.200 / L p.229): P00. RELDSP, NODOFLAG off, 2.5 restart for
 * STATEINT1, the P00 downlist, groups killed, MODREG = 0, GOPROG2.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the GOPROG2 request status, or the RELDSP / SEUDOP00
 *          status that failed
 **********************************************************************/
static agc_status_t cp_pooh(void)
{
    agc_status_t st = agc_dsky_releasedsp(); /* RELDSP outcome */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_set_priority(2, CP_PRIO5); /* CAF PRIO5 / TS PHSPRDT2 */
    agc_inhint();
    agc_flag_clear(AGC_FLAG_NODOFLAG);
    agc_phase_newphase(2, 5, false);     /* 2.5: STATINT1 restart  */

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        st = exec_cp_pooh_flags_lm();
    }
    else
    {
        exec_cp_pooh_flags_csm();
    }

    agc_telem_select_list(0); /* DNLADP00 = COSTALIN */
    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change_program_klean(); /* V37KLEAN: 1, 3, 5, 6 */
    agc_phase_idle_klean();           /* P00KLEAN (group 4)  */
    agc_erase_set_modreg(0);          /* GOMOD: CA MMNUMBER TS MODREG */

    cp_gogoprog(false);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * NOUVEAU / SEUDOP00 / RENDV00 (C pp.200-202 / L pp.229-231): the new
 * program's downlist, flags, the rendezvous groups, then GOPROG2.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the GOPROG2 request status, or the SEUDOP00 status that
 *          failed; BAD_ARG when no program was selected
 **********************************************************************/
static agc_status_t cp_nouveau(void)
{
    int                   n  = 0;                                     /* rows in the mode table */
    const exec_cp_mode_t *t  = cp_modes(&n);                          /* the mode table */
    bool                  lm = agc_state_vehicle() == AGC_VEHICLE_LM; /* Luminary rope */

    if (!AGC_ASSERT(mindex >= 0 && mindex < n))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_OK; /* SEUDOP00 outcome */

    agc_inhint();

    if (lm)
    {
        st = exec_cp_nouveau_flags_lm();
    }
    else
    {
        exec_cp_nouveau_flags_csm();
    }

    agc_telem_select_list(t[mindex].dnlist); /* SEUDOP00: TS DNLSTCOD */

    if (lm)
    {
        agc_erase_set_agsword((uint16_t) t[mindex].dnlist); /* TS AGSWORD */
    }

    agc_relint();

    if (st == AGC_STATUS_OK)
    {
        bool statquo; /* STATQUO: the running P20/P25 is kept */

        agc_phase_change_program_klean(); /* V37KLEAN           */

        if (lm)
        {
            statquo = exec_cp_rendezvous_lm(mmnumber);
        }
        else
        {
            statquo = exec_cp_rendezvous_csm(mmnumber);
        }

        if (statquo)
        {
            agc_erase_set_modreg((agc_sp_t) mmnumber); /* STATQUO .. GOMOD */
        }

        cp_gogoprog(!statquo); /* else REV37 / P00FIZZ: V37XEQ */
    }

    return (st);
} /* cp_nouveau */

/**********************************************************************
 * CANV37 / R00 (C p.199 / L p.229): wait for the integration, clean up
 * outbits and flags, then POOH (P00) or NOUVEAU. Entered in the V37 job,
 * from SERVICER's AVGEND (CSM), or as LM V37RET via OUTROUTE.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the POOH / NOUVEAU status; OK while asleep in INTSTALL
 **********************************************************************/
agc_status_t agc_job_change_program_continue(void)
{
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_CHANGE_PROGRAM_CONTINUE, AGC_TASK_NONE, (int16_t) EXEC_PRIO30, false, 0U};

    agc_phase_change_var(CP_GROUP4_JOB, &v);                                 /* R00AD; PHASCHNG OCT 14 */
    agc_status_t st = agc_integ_install(0, AGC_JOB_CHANGE_PROGRAM_CONTINUE); /* INTSTALL outcome */

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st); /* asleep */
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        exec_cp_cleanup_lm();
    }
    else
    {
        exec_cp_cleanup_csm();
    }

    return ((mmnumber == 0U) ? cp_pooh() : cp_nouveau()); /* POOH : NOUVEAU */
}

/**********************************************************************
 * V37XEQ / V37XEQC (C p.202 / L p.232): after the software restart,
 * preset PHSPRDT4, start the program's first job at its priority
 * (SPVAC), MODREG = the new major mode (NEWMODEA), RELDSP, ENDOFJOB.
 * (MMTEMP and BASETEMP are PHSPRDT3 and TBASE4 in the AGC; that debris
 * is not reproduced.)
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the FINDVAC / NEWMODEA / RELDSP status; BAD_ARG when no
 *          program was selected
 **********************************************************************/
agc_status_t agc_job_change_program_start(void)
{
    int                   n = 0;            /* rows in the mode table */
    const exec_cp_mode_t *t = cp_modes(&n); /* the mode table */

    if (!AGC_ASSERT(mindex >= 0 && mindex < n))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_set_priority(4, t[mindex].prio);                         /* TS PHSPRDT4      */
    agc_status_t st = agc_exec_findvac(t[mindex].prio, t[mindex].job); /* FINDVAC outcome */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BAILOUT 1201/1202 raised inside */
    }

    st = agc_phase_newmode((agc_sp_t) t[mindex].mm); /* NEWMODEA */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_releasedsp()); /* RELDSP, ENDOFJOB */
}
