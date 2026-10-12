/*
 * csm.c -- the Comanche parts of V37: the FCADRMM1 / PREMM1 / DNLADMM1
 *          tables, R00TOP00, and the CSM flag and outbit work of CANV37,
 *          POOH, NOUVEAU and RENDV00.
 *
 * Source: FRESH_START_AND_RESTART.agc, Comanche055 pp. 197-209.
 */
#include <stdbool.h>
#include <stddef.h>

#include "../internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/state.h"
#include "agc/csm/dap.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"

#define CP_PRIO13           ((agc_prio_t) 013)
#define CP_PRIO20           ((agc_prio_t) 020)
#define CP_COSTALIN         0                   /* downlist codes (DNLADMM1)            */
#define CP_ENTRYUPD         1
#define CP_RENDEZVU         2
#define CP_POWERED          3
#define CP_ORBNAV_DNLIST    4
#define CP_BIT13            010000u             /* DSALMOUT bit 13: engine on       */
#define CP_F6_DAPBITS       060000u             /* FLAGWRD6 bits 15, 14: DAP type   */
#define CP_F6_TVC           040000u
#define CP_T5_3_1SEC        ((uint16_t) 037312) /* 3.1SEC: 2.5 + 0.6 s     */
#define CP_ENGINE_OFF_DELAY 5U                  /* DELAYJOB 5 cs           */
#define CP_CH11_CLEAR       ((uint16_t) 001400) /* caution reset, test conn */
#define CP_CH12_KEEP        ((uint16_t) 044571) /* OCT44571 WAND 12          */
#define CP_CH13_CLEAR       ((uint16_t) 000600) /* OCT600: unused bits       */
#define CP_WORD15           ((uint16_t) 077777)

/* FCADRMM1 / PREMM1 / DNLADMM1 (C pp.206-209), highest mode first. P53
 * EQUALS P51 and P54 EQUALS PROG52; GTSCPSS1 is the P01 lead-in.        */
static const exec_cp_mode_t cp_modes_csm[] = {
    {79, CP_PRIO13, CP_RENDEZVU, AGC_JOB_STABLE_ORBIT_MIDCOURSE_OTHER_VEHICLE},
    {78, CP_PRIO13, CP_RENDEZVU, AGC_JOB_STABLE_ORBIT_RENDEZVOUS_OTHER_VEHICLE},
    {77, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPI_SEARCH_OTHER_VEHICLE},
    {76, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TARGET_DELTA_V},
    {75, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPM_TARGETING_OTHER_VEHICLE},
    {74, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPI_TARGETING_OTHER_VEHICLE},
    {73, CP_PRIO13, CP_RENDEZVU, AGC_JOB_CDH_TARGETING_OTHER_VEHICLE},
    {72, CP_PRIO13, CP_RENDEZVU, AGC_JOB_CSI_TARGETING_OTHER_VEHICLE},
    {62, CP_PRIO13, CP_ENTRYUPD, AGC_JOB_CM_SM_SEPARATION},
    {61, CP_PRIO13, CP_POWERED, AGC_JOB_ENTRY_PREPARATION},
    {54, CP_PRIO13, CP_COSTALIN, AGC_JOB_IMU_REALIGN},
    {53, CP_PRIO13, CP_COSTALIN, AGC_JOB_IMU_ORIENTATION_DETERMINATION},
    {52, CP_PRIO13, CP_COSTALIN, AGC_JOB_IMU_REALIGN},
    {51, CP_PRIO13, CP_COSTALIN, AGC_JOB_IMU_ORIENTATION_DETERMINATION},
    {47, CP_PRIO13, CP_POWERED, AGC_JOB_THRUST_MONITOR},
    {41, CP_PRIO13, CP_POWERED, AGC_JOB_RCS_BURN},
    {40, CP_PRIO13, CP_POWERED, AGC_JOB_SPS_BURN},
    {39, CP_PRIO13, CP_RENDEZVU, AGC_JOB_STABLE_ORBIT_MIDCOURSE},
    {38, CP_PRIO13, CP_RENDEZVU, AGC_JOB_STABLE_ORBIT_RENDEZVOUS},
    {37, CP_PRIO13, CP_RENDEZVU, AGC_JOB_RETURN_TO_EARTH},
    {35, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPM_TARGETING},
    {34, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPI_TARGETING},
    {33, CP_PRIO13, CP_RENDEZVU, AGC_JOB_CDH_TARGETING},
    {32, CP_PRIO13, CP_RENDEZVU, AGC_JOB_CSI_TARGETING},
    {31, CP_PRIO13, CP_RENDEZVU, AGC_JOB_LAMBERT_AIMPOINT_GUIDANCE},
    {30, CP_PRIO13, CP_RENDEZVU, AGC_JOB_EXTERNAL_DELTA_V},
    {23, CP_PRIO13, CP_RENDEZVU, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION},
    {22, CP_PRIO13, CP_ORBNAV_DNLIST, AGC_JOB_ORBITAL_NAVIGATION},
    {21, CP_PRIO13, CP_RENDEZVU, AGC_JOB_GROUND_TRACK_DETERMINATION},
    {20, CP_PRIO13, CP_RENDEZVU, AGC_JOB_RENDEZVOUS_NAVIGATION},
    {17, CP_PRIO13, CP_RENDEZVU, AGC_JOB_TPI_SEARCH},
    {6, CP_PRIO13, CP_COSTALIN, AGC_JOB_STANDBY},
    {1, CP_PRIO20, CP_COSTALIN, AGC_JOB_PRELAUNCH_INITIALIZATION}
};

const exec_cp_mode_t *exec_cp_modes_csm(int *n)
{
    *n = (int) (sizeof (cp_modes_csm) / sizeof (cp_modes_csm[0]));

    return (cp_modes_csm);
}

/* R00TOP00 (C p.197-198): with the SPS engine on or the TVC DAP running,
 * V37 first stops them: SPSOFF, MASSPROP, the RCS DAP in 3.1 s (RCSDAPON
 * +1), TVCZAP, then DELAYJOB 5 cs and P00. NOT_FOUND: not needed.      */
agc_status_t exec_cp_engine_off_to_idle(void)
{
    bool engine = (agc_chan_read(AGC_CHAN_DSALMOUT) & CP_BIT13) != 0U;
    bool tvc = (agc_flagword_get(6) & CP_F6_DAPBITS) == CP_F6_TVC;

    if (!engine && !tvc)
    {
        return (AGC_STATUS_NOT_FOUND); /* ISITP00             */
    }

    agc_inhint();
    agc_csm_spsoff();
    agc_csm_dap_massprop();
    agc_status_t st = agc_csm_dap_rcsdapon_t5(CP_T5_3_1SEC);

    agc_csm_tvczap(); /* DISABLE TVC         */
    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_svc_delayjob(CP_ENGINE_OFF_DELAY, AGC_JOB_CHANGE_PROGRAM_JETS_OFF));
}

/* R00TOP00 after DELAYJOB: WRITE 5, WRITE 6 = 0 (all jets off), then
 * ISITP00 with MMNUMBER = 0.                                            */
agc_status_t agc_job_change_program_jets_off(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_chan_write((int) AGC_CHAN_PYJETS, 0);   /* WRITE 5 */
    agc_chan_write((int) AGC_CHAN_ROLLJETS, 0); /* WRITE 6 */

    return (exec_cp_after_engine_off());
}

/* ISSERVON: DOWNFLAG AVEGFLAG; SERVICER's AVGEND returns to CANV37.     */
void exec_cp_stop_servicer_csm(void)
{
    agc_flag_clear(AGC_FLAG_AVEGFLAG);
}

/* CANV37 after INTSTALL (C p.199-200): caution reset and test connector
 * off, the optics / TVC / S-IVB outbits of channel 12 except OCT44571,
 * channel 13 bits 7, 8; INITSUB; CLEARMRK; STIKFLAG, VHFRFLAG, R21MARK
 * down; the uplink activity light off (UPACTOFF).                       */
void exec_cp_cleanup_csm(void)
{
    agc_chan_and((int) AGC_CHAN_DSALMOUT, (uint16_t) (CP_WORD15 & ~CP_CH11_CLEAR));
    agc_chan_and((int) AGC_CHAN_CHAN12, CP_CH12_KEEP);
    agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (CP_WORD15 & ~CP_CH13_CLEAR));
    exec_initsub_csm();
    agc_dsky_clearmrk();
    agc_flag_clear(AGC_FLAG_STIKFLAG);
    agc_chan_and((int) AGC_CHAN_DSALMOUT, (uint16_t) (CP_WORD15 & ~(uint16_t) AGC_CH011_UPACT));
    agc_flag_clear(AGC_FLAG_VHFRFLAG);
    agc_flag_clear(AGC_FLAG_MARKRUPT_OPTION_ONE); /* R21MARK */
}

/* TRACK, TARG1 and UPDATE flags off (SEUDOP00: CS OCT01120 MASK FLAGWRD1). */
static void cp_track_flags_off(void)
{
    agc_flag_clear(AGC_FLAG_TRACKFLG);
    agc_flag_clear(AGC_FLAG_TARG1FLG);
    agc_flag_clear(AGC_FLAG_UPDATFLG);
}

/* POOH (C p.200): IMUSE and RNDVZFLG off (BIT7-8 of FLAGWRD0), SEUDOP00. */
void exec_cp_pooh_flags_csm(void)
{
    agc_flag_clear(AGC_FLAG_IMUSE);
    agc_flag_clear(AGC_FLAG_RNDVZFLG); /* KILL P20 */
    cp_track_flags_off();
}

/* NOUVEAU (C p.202): IMUSE off unless P20 runs (RNDVZFLG), SEUDOP00.      */
void exec_cp_nouveau_flags_csm(void)
{
    if (!agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        agc_flag_clear(AGC_FLAG_IMUSE);
    }

    cp_track_flags_off();
}

/* RENDV00 .. REV37 (C p.201): P20 requested while P20 runs: STATQUO (keep
 * group 2, TRACK and UPDATE on, group 4 off; true). Otherwise groups 1
 * and 2 are killed (KILL20) unless the rendezvous flag is set (false).  */
bool exec_cp_rendezvous_csm(uint8_t mm)
{
    bool rndvz = agc_flag_test(AGC_FLAG_RNDVZFLG);

    if (mm == 20U && agc_erase_modreg() != 20 && rndvz)
    { /* STATQUO                        */
        agc_flag_set(AGC_FLAG_TRACKFLG);
        agc_flag_set(AGC_FLAG_UPDATFLG);
        agc_phase_kill(4);

        return (true);
    }

    if (mm == 20U && agc_erase_modreg() == 20)
    {
        rndvz = false; /* RENDN00: P20 on top of P20 -> KILL20 */
    }

    if (!rndvz)
    { /* KILL20                         */
        agc_phase_kill(1);
        agc_phase_kill(2);
    }

    return (false);
}
