/**********************************************************************
 * FILE NAME: lm.c
 *
 * PURPOSE:
 *   The Luminary parts of V37 (change program): the FCADRMM1 / PREMM1 /
 *   DNLADMM1 table, SETUP70 / SETUP71, V37RET, and the LM flag work of
 *   CANV37 (R00), POOH, SEUDOP00, NOUVEAU and RENDV00.
 *
 * AGC SOURCE:
 *   Luminary099 FRESH_START_AND_RESTART.agc pp. 227-236
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   FLAGWRD0,1,11,13 flags  I/O  bits           rendezvous, R04, MUNFLAG ...
 *   MODREG        agc_sp_t  I    B0 integer     RENDV00 test
 *   OUTROUTE      job id    O    -              ISSERVON: V37RET
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_lm_enginof1, agc_lm_dap_allcoast   SEUDOP00 (lm)
 *   agc_rendz_clradmod    CLRADMOD (prog-rendz)
 *   agc_lm_job_aps_abort / dps_abort       P71 / P70
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   The ENGINOF1 / ALLCOAST / P70 / P71 statuses.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>

#include "../internal.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"

#define CP_PRIO13     ((agc_prio_t) 013)  /* PRIO13: program start priority (PREMM1) */
#define CP_COSTALIN   0                   /* downlist codes (DNLADMM1, L p.235-236) */
#define CP_RENDEZVU   2                   /* RENDEZVU downlist */
#define CP_ORBMANUV   3                   /* ORBMANUV downlist */
#define CP_DESASCNT   4                   /* DESASCNT downlist */
#define CP_LUNRSALN   5                   /* LUNRSALN downlist */
#define CP_PHASE_2_7  ((uint16_t) 040072) /* V37RET 2.7SPT (TBASE2)  */
#define CP_PHASE_2_11 ((uint16_t) 040112) /* V37RET 2.11SPT (TBASE2) */

/* FCADRMM1 / PREMM1 / DNLADMM1 (L pp.233-236), highest mode first.       */
static const exec_cp_mode_t cp_modes_lm[] = {
    {79, CP_PRIO13, CP_RENDEZVU, AGC_JOB_STABLE_ORBIT_MIDCOURSE_OTHER_VEHICLE},
    {78, CP_PRIO13, CP_RENDEZVU, AGC_JOB_STABLE_ORBIT_RENDEZVOUS_OTHER_VEHICLE},
    {76, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TARGET_DELTA_V},
    {75, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPM_TARGETING_OTHER_VEHICLE},
    {74, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPI_TARGETING_OTHER_VEHICLE},
    {73, CP_PRIO13, CP_RENDEZVU, AGC_JOB_CDH_TARGETING_OTHER_VEHICLE},
    {72, CP_PRIO13, CP_RENDEZVU, AGC_JOB_CSI_TARGETING_OTHER_VEHICLE},
    {68, CP_PRIO13, CP_DESASCNT, AGC_JOB_LANDING_CONFIRMATION},
    {63, CP_PRIO13, CP_DESASCNT, AGC_JOB_BRAKING_PHASE},
    {57, CP_PRIO13, CP_LUNRSALN, AGC_JOB_LUNAR_SURFACE_ALIGN},
    {52, CP_PRIO13, CP_COSTALIN, AGC_JOB_IMU_REALIGN},
    {51, CP_PRIO13, CP_COSTALIN, AGC_JOB_IMU_ORIENTATION_DETERMINATION},
    {47, CP_PRIO13, CP_ORBMANUV, AGC_JOB_THRUST_MONITOR},
    {42, CP_PRIO13, CP_ORBMANUV, AGC_JOB_APS_BURN},
    {41, CP_PRIO13, CP_ORBMANUV, AGC_JOB_RCS_BURN},
    {40, CP_PRIO13, CP_ORBMANUV, AGC_JOB_DPS_BURN},
    {39, CP_PRIO13, CP_RENDEZVU, AGC_JOB_STABLE_ORBIT_MIDCOURSE},
    {38, CP_PRIO13, CP_RENDEZVU, AGC_JOB_STABLE_ORBIT_RENDEZVOUS},
    {35, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPM_TARGETING},
    {34, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPI_TARGETING},
    {33, CP_PRIO13, CP_RENDEZVU, AGC_JOB_CDH_TARGETING},
    {32, CP_PRIO13, CP_RENDEZVU, AGC_JOB_CSI_TARGETING},
    {31, CP_PRIO13, CP_RENDEZVU, AGC_JOB_LAMBERT_AIMPOINT_GUIDANCE},
    {30, CP_PRIO13, CP_RENDEZVU, AGC_JOB_EXTERNAL_DELTA_V},
    {25, CP_PRIO13, CP_RENDEZVU, AGC_JOB_PREFERRED_TRACKING_ATTITUDE},
    {22, CP_PRIO13, CP_LUNRSALN, AGC_JOB_ORBITAL_NAVIGATION},
    {21, CP_PRIO13, CP_RENDEZVU, AGC_JOB_GROUND_TRACK_DETERMINATION},
    {20, CP_PRIO13, CP_RENDEZVU, AGC_JOB_RENDEZVOUS_NAVIGATION},
    {12, CP_PRIO13, CP_DESASCNT, AGC_JOB_POWERED_ASCENT},
    {6, CP_PRIO13, CP_COSTALIN, AGC_JOB_STANDBY}
};

/**********************************************************************
 * FCADRMM1 / PREMM1 / DNLADMM1 (FRESH_START_AND_RESTART L pp.234-236):
 * the Luminary major-mode table, highest mode first.
 *
 * ARGUMENTS:
 *   n -- receives the number of table rows
 *
 * RETURNS: const exec_cp_mode_t * -- the table
 **********************************************************************/
const exec_cp_mode_t *exec_cp_modes_lm(int *n)
{
    *n = (int) (sizeof (cp_modes_lm) / sizeof (cp_modes_lm[0]));

    return (cp_modes_lm);
}

/**********************************************************************
 * SETUP70 / SETUP71 (L p.233): DTCB to P70 / P71 in the V37 job.
 *
 * ARGUMENTS:
 *   aps -- true: P71 (APS abort), false: P70 (DPS abort)
 *
 * RETURNS: agc_status_t -- the P70 / P71 job's status
 **********************************************************************/
agc_status_t exec_cp_setup_abort(bool aps)
{
    return (aps ? agc_lm_job_aps_abort() : agc_lm_job_dps_abort()); /* SETUP71 : SETUP70 */
}

/**********************************************************************
 * ISSERVON (L p.228): DOWNFLAG AVEGFLAG, OUTROUTE = V37RET, ENDOFJOB.
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_cp_stop_servicer_lm(void)
{
    agc_flag_clear(AGC_FLAG_AVEGFLAG);
    agc_guid_state()->lm.outroute = AGC_JOB_CHANGE_PROGRAM_RETURN;
}

/**********************************************************************
 * V37RET (L p.228): group 2 for the program that keeps running: 2.7 when
 * P20 or P22 runs, 2.11 for P25, else group 2 off; then CANV37.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the CANV37 status
 **********************************************************************/
agc_status_t agc_job_change_program_return(void)
{
    if (agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        agc_phase_change(CP_PHASE_2_7); /* 2.7SPT     */
    }
    else if (agc_flag_test(AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE))
    {
        agc_phase_change(CP_PHASE_2_11); /* 2.11SPT    */
    }
    else
    {
        agc_phase_kill(2); /* 2.0SPT                     */
    }

    return (agc_job_change_program_continue());
}

/**********************************************************************
 * R00 after INTSTALL (L p.229): 3AXISFLG down, FLGWRD11 = LRBYBIT (R12),
 * R04FLAG, MUNFLAG and XOVINFLG down.
 *
 * ARGUMENTS: none
 **********************************************************************/
void exec_cp_cleanup_lm(void)
{
    agc_flag_clear(AGC_FLAG_3AXISFLG);
    agc_flagword_set(agc_flag_word(AGC_FLAG_LRBYPASS), agc_flag_mask(AGC_FLAG_LRBYPASS)); /* TS FLGWRD11 */
    agc_flag_clear(AGC_FLAG_ALARM_521_SUPPRESSED);                                        /* R04FLAG     */
    agc_flag_clear(AGC_FLAG_MUNFLAG);
    agc_flag_clear(AGC_FLAG_XOVINFLG);
}

/**********************************************************************
 * POOH / RESET22 (FRESH_START_AND_RESTART L p.229): P20, P25 and
 * IMU-in-use flags off (CS OCT700 MASK FLAGWRD0).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void cp_rendezvous_flags_off(void)
{
    agc_flag_clear(AGC_FLAG_RNDVZFLG);
    agc_flag_clear(AGC_FLAG_IMUSE);
    agc_flag_clear(AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE);
}

/**********************************************************************
 * SEUDOP00 (L p.229-230): ENGINOF1, ALLCOAST, TRACK and UPDATE off.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the ALLCOAST status
 **********************************************************************/
static agc_status_t cp_seudop00(void)
{
    agc_lm_enginof1();
    agc_status_t st = agc_lm_dap_allcoast(); /* ALLCOAST outcome; does a RESTORDB    */

    agc_flag_clear(AGC_FLAG_TRACKFLG);
    agc_flag_clear(AGC_FLAG_UPDATFLG);

    return (st);
}

/**********************************************************************
 * POOH (L p.229): CLRADMOD, the rendezvous flags, then SEUDOP00. AGSWORD
 * gets the P00 downlist code with DNLSTCOD.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the SEUDOP00 status
 **********************************************************************/
agc_status_t exec_cp_pooh_flags_lm(void)
{
    agc_rendz_clradmod();
    cp_rendezvous_flags_off();
    agc_erase_set_agsword(0); /* DNLADP00               */

    return (cp_seudop00());
}

/**********************************************************************
 * NOUVEAU (L p.231): IMUSE off unless P20 or P25 runs, then SEUDOP00.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the SEUDOP00 status
 **********************************************************************/
agc_status_t exec_cp_nouveau_flags_lm(void)
{
    if (!agc_flag_test(AGC_FLAG_RNDVZFLG) && !agc_flag_test(AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE))
    {
        agc_flag_clear(AGC_FLAG_IMUSE);
    }

    return (cp_seudop00());
}

/**********************************************************************
 * RESET22 / KILL2 (L p.230-231): rendezvous flags and RADMODES cleared
 * (P22 involved), group 2 killed.
 *
 * ARGUMENTS:
 *   clear_flags -- true: RESET22 (flags and RADMODES), false: KILL2 only
 **********************************************************************/
static void cp_reset22(bool clear_flags)
{
    if (clear_flags)
    {
        cp_rendezvous_flags_off();
        agc_rendz_clradmod();
    }

    agc_phase_kill(2); /* KILL2              */
}

/**********************************************************************
 * RENDV00 / RENDN00 / STATQUO (L p.230-231). STATQUO (true): P20 or P25
 * asked for while it runs: TRACK and UPDATE on, P00KLEAN; the caller sets
 * MODREG = mm.
 *
 * ARGUMENTS:
 *   mm -- MMNUMBER, the requested major mode
 *
 * RETURNS: bool -- true for STATQUO
 **********************************************************************/
bool exec_cp_rendezvous_lm(uint8_t mm)
{
    agc_sp_t modreg  = agc_erase_modreg();                                         /* C(MODREG), B0 integer   */
    bool     p20     = agc_flag_test(AGC_FLAG_RNDVZFLG);                           /* P20 running             */
    bool     p25     = agc_flag_test(AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE); /* P25 running             */
    bool     statquo = false;                                                      /* keep the running P20/P25 */

    if (modreg == 22 || mm == 22U || ((mm != 20U && mm != 25U) && !p20 && !p25))
    {
        cp_reset22(true); /* RESET22                */
    }
    else if (mm == 20U || mm == 25U)
    {
        statquo = (modreg != 20 && modreg != 25) && ((mm == 20U && p20) || (mm == 25U && p25));

        if (!statquo)
        {
            cp_reset22(false); /* KILL2              */
        }
    }
    else
    {
        /* P00FIZZ: P20 or P25 keeps group 2                          */
    }

    if (statquo)
    { /* STATQUO, GOMOD: P00KLEAN                 */
        agc_flag_set(AGC_FLAG_TRACKFLG);
        agc_flag_set(AGC_FLAG_UPDATFLG);
        agc_phase_idle_klean();
    }

    return (statquo);
}
