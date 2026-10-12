/**********************************************************************
 * FILE NAME: extended_verbs_lm_radar.c
 *
 * PURPOSE:
 *   The LM radar extended verbs: V40 N72 RR CDU zero, V41 N72 RR designate
 *   with the lock-on question, V44 end of continuous designate, V60 LR
 *   position 2, V63 radar test (R04), V78 / V79 LR spurious return test
 *   (R77), V85 RR line of sight display; RDRUSECK, the radar-in-use check
 *   they share.
 *
 * AGC SOURCE:
 *   Luminary099 EXTENDED_VERBS.agc pp.265-280, 298-300
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors; the R04 / R77 test words are
 *   owned here)
 *   Name        Type      I/O  Scale     Description
 *   ----------  --------  ---  --------  ---------------------------------
 *   RADMODES    uint16_t  I/O  bits      radar mode bits (FLGWRD12)
 *   OPTIONX     agc_sp_t  I/O  unscaled  V04N12 option code / answer
 *   RSAMPDT ..  agc_sp_t  I/O  unscaled  R04 / R77 sampling (RADSAMP)
 *    RFAILCNT
 *   RR-AZ, ELEV agc_dp_t  O    rev 2^0   V85 display (N56)
 *   EXTVBACT    uint16_t  I    bits      bit 5: V85 still shown
 *
 * EXTERNAL REFERENCES:
 *   Name                    Description
 *   ----------------------  ------------------------------------------
 *   agc_rr_zero, agc_radar  RRZERO, LRPOS2, RRDESNB (radar lane)
 *    _lrpos2, agc_rr_
 *    designate_gimbal
 *   agc_imu_stall_dev       RADSTALL
 *   agc_lm_rrnb             RRNBMPAC (V85)
 *   agc_svc_delayjob        DELAYJOB
 *   agc_dsky_*              GOXDSPF, GOMARKF(R), GOMARK4, BLANKET, ENDEXT
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALM/END when a radar is in use (RDRUSECK) or P20 runs (V41); ALARM
 *   00503 designate fail, 00523 LR not in position 2.
 *
 * NOTES:
 *   The radar verbs (pp. 265-280, 298-300):
 *     V40 N72  RRZEROK    zero the RR CDUs
 *     V41 N72  RRDESNBK   designate the RR to loaded gimbal angles, asking
 *                         (AURLOKON) for lock-on or continuous designate
 *     V44      RRDESEND   terminate continuous designate
 *     V60      LRPOS2K    command the landing radar to position 2
 *     V63      R04        radar test: sample the RR or LR once a second
 *     V78/V79  R77/R77END LR spurious return test, start / stop
 *     V85      VERB85     RR line of sight azimuth and elevation (RRLOSDSP)
 *   and RDRUSECK, the "is either radar in use" check they share.
 *
 *   The radar mode-switching routines called here are in P20-P25.agc and
 *   RADAR_LEADIN_ROUTINES.agc (prog-rendz): RRZERO, RRDESNB, LRPOS2,
 *   CLRADMOD and the RADSAMP sampling task
 *   (include/agc/programs/rendezvous.h,
 *   include/agc/programs/alignment.h).  RADSTALL is the imu lane's
 *   agc_imu_stall_dev(AGC_IMU_DEV_RADAR, ...).
 *
 *   DEVIATION (MAKECADR / DESRET): AURLOKON keeps its caller's return
 *   CADR in DESRET and jumps back to it after the V04N12 answer, and
 *   RRDESK2 gives up its VAC area by hand before it stalls.  Here the
 *   answer is a continuation job that runs the rest of RRDESNBK; the VAC
 *   bookkeeping vanishes (docs/TRANSLATION_GUIDE.md section 4).
 *
 *   Erasables owned here (R04/R77 test storage, also read and written by
 *   RADSAMP): RSAMPDT, RTSTDEX, RTSTBASE, RTSTMAX, RTSTLOC, RFAILCNT.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define WORD15        077777u                           /* one AGC word */
#define BIT2          000002u                           /* CHAN33: RR auto-mode discrete (inverted)   */
#define BIT3          000004u                           /* BLANKET R3                                 */
#define BIT5          000020u                           /* EXTVBACT: extended verb active             */
#define BIT14         020000u                           /* CHAN12: enable RR auto tracker             */
#define RADMODES_CDES 040000u                           /* RADMODES bit 15: continuous designate      */
#define OCT41000      041000u                           /* RADMODES bits 15, 10: designate bits       */
#define LRPOSCAL      000444u                           /* RADMODES/CHAN33 bits 9, 6, 3 (scales, pos) */
#define SIX           000006u                           /* RTSTDEX: LR when nonzero under this mask   */
#define LOCKON_OPTION ((agc_sp_t) 6)                    /* OPTIONX: V04N12 option code 6        */
#define LOCKON_NONE   ((agc_sp_t) 2)                    /* OPTIONX +1: 2 = no lock-on           */
#define RADAR_OPTION  ((agc_sp_t) 4)                    /* OPTIONX: option code (BIT3)          */
#define RADAR_RR      ((agc_sp_t) 1)                    /* OPTIONX +1 / RTSTDEX: RR             */
#define RADAR_LR      ((agc_sp_t) 2)                    /* OPTIONX +1 / RTSTDEX: LR             */
#define RR_AUTO_CHECKLIST \
    ((agc_sp_t) 00201)                                  /* 201R04: select RR auto mode    */
#define RSAMP_1SEC           ((agc_sp_t) 101)           /* 1SEC+1: sample once a second        */
#define RSAMP_250MS          ((agc_sp_t) 032)           /* 250MS+1 = CALLCODE: 4 per second    */
#define RR_SAMPLE_MAX        2                          /* RTSTMAX for the RR (and first delay, cs)   */
#define LR_SAMPLE_MAX        6                          /* RTSTMAX for the LR (and first delay, cs)   */
#define LR_BASE              ((agc_sp_t) 2)             /* RTSTBASE for the LR                  */
#define DELAY_1SEC           100u                       /* 1SEC */
#define DELAY_2SECS          200u                       /* 2SECS */
#define DELAY_BIT8           0200u                      /* 1.28 s: a pending radar RUPT               */
#define DELAY_BIT6           040u                       /* 320 ms                                     */
#define PRIO4                ((agc_prio_t) 004)         /* PRIO4 */
#define PRIO5                ((agc_prio_t) 005)         /* PRIO5 */
#define PRIO20               ((agc_prio_t) 020)         /* PRIO20 */
#define ALARM_DESIGNATE_FAIL ((agc_alarm_code_t) 00503) /* 503: designate fail */
#define ALARM_LR_POSITION    ((agc_alarm_code_t) 00523) /* 523: LR not in position 2 */
#define LM_SURFACE_NAV_MODE  22                         /* DEC22: P22 lunar surface navigation        */

/* R04 / R77 test storage (EBANK4).                                        */
static agc_sp_t rsampdt;  /* sampling interval + 1, cs 2^0; 0 stops it */
static agc_sp_t rtstdex;  /* which radar sample, unscaled */
static agc_sp_t rtstbase; /* 0 RR, 2 LR, unscaled */
static agc_sp_t rtstmax;  /* 2 RR, 6 LR, unscaled */
static agc_sp_t rtstloc;  /* storage index, unscaled */
static agc_sp_t rfailcnt; /* bad sample count, unscaled */

/**********************************************************************
 * AGC: none -- RSAMPDT (R04 / R77, read by RADSAMP) -- the sampling interval
 *
 * RETURNS: RSAMPDT: interval + 1, cs (0 stops the sampling)
 **********************************************************************/
agc_sp_t agc_erase_rsampdt(void)
{
    return (rsampdt);
}

/**********************************************************************
 * AGC: none -- RSAMPDT, written (R04Z, R04Y, R04END, R77END; RADSAMP)
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I   interval + 1, cs, or 0
 **********************************************************************/
void agc_erase_set_rsampdt(agc_sp_t v)
{
    rsampdt = v;
}

/**********************************************************************
 * AGC: none -- RTSTDEX (R04X) -- which radar is sampled
 *
 * RETURNS: RTSTDEX: 1 RR, 2 LR (test index)
 **********************************************************************/
agc_sp_t agc_erase_rtstdex(void)
{
    return (rtstdex);
}

/**********************************************************************
 * AGC: none -- RTSTDEX, written (RADSAMP)
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I   test index
 **********************************************************************/
void agc_erase_set_rtstdex(agc_sp_t v)
{
    rtstdex = v;
}

/**********************************************************************
 * AGC: none -- RTSTBASE (R04X / R04L) -- base of the sample storage
 *
 * RETURNS: 0 for the RR, 2 for the LR
 **********************************************************************/
agc_sp_t agc_erase_rtstbase(void)
{
    return (rtstbase);
}

/**********************************************************************
 * AGC: none -- RTSTMAX (R04C) -- the last sample slot
 *
 * RETURNS: 2 for the RR, 6 for the LR
 **********************************************************************/
agc_sp_t agc_erase_rtstmax(void)
{
    return (rtstmax);
}

/**********************************************************************
 * AGC: none -- RTSTLOC (R04Z, RADSAMP) -- the next sample slot
 *
 * RETURNS: RTSTLOC
 **********************************************************************/
agc_sp_t agc_erase_rtstloc(void)
{
    return (rtstloc);
}

/**********************************************************************
 * AGC: none -- RTSTLOC, written (RADSAMP)
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I   next sample slot
 **********************************************************************/
void agc_erase_set_rtstloc(agc_sp_t v)
{
    rtstloc = v;
}

/**********************************************************************
 * AGC: none -- RFAILCNT (R04Z, RADSAMP) -- bad samples counted
 *
 * RETURNS: RFAILCNT
 **********************************************************************/
agc_sp_t agc_erase_rfailcnt(void)
{
    return (rfailcnt);
}

/**********************************************************************
 * AGC: none -- RFAILCNT, written (RADSAMP)
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I   bad sample count
 **********************************************************************/
void agc_erase_set_rfailcnt(agc_sp_t v)
{
    rfailcnt = v;
}

/* ---- shared helpers ------------------------------------------------------ */

/**********************************************************************
 * RDRUSECK (p.280): ALM/END if R29 may be using the RR (NOR29FLG reset),
 * R77 is on, R12 may be using the LR (SERVICER on and LRBYPASS reset), P20
 * is tracking (TRACKFLG and RNDVZFLG) or P22 is running.
 *
 * ARGUMENTS:
 *   ok  bool *  O   true when no radar is in use (go on)
 *
 * RETURNS: AGC_STATUS_OK, or the ALM/END status
 **********************************************************************/
static agc_status_t radar_in_use_check(bool *ok)
{
    bool busy = !agc_flag_test((agc_flag_id_t) AGC_FLAG_NO_POWERED_DESIGNATE) || agc_flag_test((agc_flag_id_t) AGC_FLAG_RADAR_ALARMS_SUPPRESSED) || (agc_flag_test((agc_flag_id_t) AGC_FLAG_SERVICER_RUNNING) && !agc_flag_test((agc_flag_id_t) AGC_FLAG_LRBYPASS)) || (agc_flag_test((agc_flag_id_t) AGC_FLAG_TRACKFLG) && agc_flag_test((agc_flag_id_t) AGC_FLAG_RNDVZFLG)) || (agc_erase_modreg() == (agc_sp_t) LM_SURFACE_NAV_MODE); /* R29, R77, R12 (SERVICER, LR bypass), P20 tracking or P22 */

    *ok = !busy;

    return (busy ? agc_extverb_alm_end() : AGC_STATUS_OK); /* in use: ALM/END */
}

/**********************************************************************
 * AGC: none -- RADSTALL: true when the job must return (asleep until wake runs).
 *
 * ARGUMENTS:
 *   wake  agc_job_id_t    I   the job woken when the radar mode switch
 *                             ends
 *   st    agc_status_t *  O   the RADSTALL status
 *
 * RETURNS: true when the job must return (asleep, or a failure in *st)
 **********************************************************************/
static bool radar_stall(agc_job_id_t wake, agc_status_t *st)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING; /* STALL return */

    *st = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, wake, &res);

    return ((*st != AGC_STATUS_OK) || (res == AGC_IMU_STALL_SLEEPING));
}

/**********************************************************************
 * AGC: none -- RADMODES: clear the designate bits 15 and 10 (CS OCT41000, MASK).
 **********************************************************************/
static void clear_designate_bits(void)
{
    agc_inhint();
    agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() & ~(uint32_t) OCT41000));
    agc_relint();
}

/* ---- V40 N72, V60 ----------------------------------------------------------- */

/**********************************************************************
 * RRZEROK (V40 N72, p.265): RRZERO, RADSTALL, GOPIN on either return.
 *
 * RETURNS: the RDRUSECK, RRZERO, RADSTALL or GOPIN status
 **********************************************************************/
agc_status_t agc_extverb_rr_zero(void)
{
    bool         ok = false;                   /* no radar in use */
    agc_status_t st = radar_in_use_check(&ok); /* RDRUSECK / RRZERO / stall status */

    if (!ok)
    {
        return (st);
    }

    st = agc_rr_zero();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (radar_stall(AGC_JOB_EXTENDED_VERB_GOPIN, &st))
    {
        return (st);
    }

    return (agc_extverb_gopin());
}

/**********************************************************************
 * LRPOS2K (V60, p.265): LRPOS2, RADSTALL; alarm 523 if position 2 is not
 * indicated in time; GOPIN.
 *
 * RETURNS: the RDRUSECK, LRPOS2, RADSTALL status, or that of the check
 **********************************************************************/
agc_status_t agc_extverb_lr_position_two(void)
{
    bool         ok = false;                   /* no radar in use */
    agc_status_t st = radar_in_use_check(&ok); /* RDRUSECK / LRPOS2 / stall status */

    if (!ok)
    {
        return (st);
    }

    st = agc_radar_lrpos2();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (radar_stall(AGC_JOB_LR_POSITION_TWO_CHECK, &st))
    {
        return (st);
    }

    return (agc_job_lr_position_two_check());
}

/**********************************************************************
 * AGC: none -- LRPOS2K after RADSTALL: bad return -> LRP2ALM (alarm 523), then GOPIN.
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
agc_status_t agc_job_lr_position_two_check(void)
{
    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) == AGC_IMU_STALL_BAD)
    {
        agc_alarm(ALARM_LR_POSITION);
    }

    return (agc_extverb_gopin());
}

/* ---- V41 N72 designate, AURLOKON, V44 ---------------------------------------- */

/**********************************************************************
 * RRDESNBK (V41 N72, p.269): not while P20 runs (XACTALM); end any present
 * designation; ask for the gimbal angles (V24N73).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the extended-verb job
 *
 * RETURNS: the RDRUSECK, TESTXACT, ENDEXT or display status
 **********************************************************************/
agc_status_t agc_extverb_rr_designate(agc_job_ctx_t *ctx)
{
    bool         ok = false;                   /* no radar in use; then TESTXACT */
    agc_status_t st = radar_in_use_check(&ok); /* RDRUSECK / TESTXACT status */

    if (!ok)
    {
        return (st);
    }

    st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    if (agc_flag_test((agc_flag_id_t) AGC_FLAG_RNDVZFLG))
    {
        agc_dsky_falton(); /* XACTALM                */

        return (agc_dsky_endmark());
    }

    clear_designate_bits();

    return (agc_job_rr_designate_load(ctx));
}

/**********************************************************************
 * AGC: none -- V24N73 flashing: V34 ends the verb, V33 asks again, ENTER -> AURLOKON.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_rr_designate_load(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_EXTENDED_VERB_END, AGC_JOB_RR_DESIGNATE_LOAD, AGC_JOB_RR_LOCKON_QUERY};

    return (agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(24, 73), &cont));
}

/**********************************************************************
 * AURLOKON (p.271): option code 6, default answer 2 (no lock-on).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display status
 **********************************************************************/
agc_status_t agc_job_rr_lockon_query(agc_job_ctx_t *ctx)
{
    agc_erase_set_optionx(1, LOCKON_NONE);
    agc_erase_set_optionx(0, LOCKON_OPTION);

    return (agc_job_rr_lockon_display(ctx));
}

/**********************************************************************
 * AGC: none -- AURLOKON -5: GOMARKFR V04N12 (V34 ENDEXT, V33 answer, V32 again), then
 * BLANKET R3 and ENDOFJOB.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display or BLANKET status
 **********************************************************************/
agc_status_t agc_job_rr_lockon_display(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_EXTENDED_VERB_END, AGC_JOB_RR_LOCKON_ANSWER, AGC_JOB_RR_LOCKON_DISPLAY};
    agc_status_t    st   = agc_dsky_display(ctx, AGC_DSKY_GOMARKFR, agc_dsky_vn(4, 12), &cont); /* display status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(BIT3));
}

/**********************************************************************
 * AGC: none -- AURLOKON +5 / NOLOKON / AURLKON1: answer 1 = lock on (LOKONSW); 2 = no
 * lock-on: continuous designate (CDESFLAG) and R25 off (NORRMON).  Then
 * the rest of RRDESNBK: show V41 again and schedule RRDESK2.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_rr_lockon_answer(agc_job_ctx_t *ctx)
{
    bool no_lockon = ((uint16_t) agc_erase_optionx(1) & (uint16_t) LOCKON_NONE) != 0u; /* OPTIONX +1 = 2: continuous designate */

    agc_flag_put((agc_flag_id_t) AGC_FLAG_LOKONSW, !no_lockon);

    if (no_lockon)
    {
        agc_flag_set((agc_flag_id_t) AGC_FLAG_CDESFLAG); /* continuous designate */
        agc_flag_set((agc_flag_id_t) AGC_FLAG_NORRMON);  /* R25 off             */
    }

    return (agc_extverb_redisplay(ctx, 41, AGC_JOB_RR_DESIGNATE_SCHEDULE));
}

/**********************************************************************
 * AGC: none -- RRDESNBK after EXDSPRET: FINDVAC RRDESK2 at priority 20, TERMEXTV.
 *
 * RETURNS: the FINDVAC or ENDEXT status
 **********************************************************************/
agc_status_t agc_job_rr_designate_schedule(void)
{
    agc_status_t st = agc_extverb_schedule(PRIO20, AGC_JOB_RR_DESIGNATE_DRIVE, true); /* FINDVAC status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_endmark());
}

/**********************************************************************
 * RRDESK2 (p.269): RRDESNB, then RADSTALL for the designate to finish.
 *
 * RETURNS: the RRDESNB or RADSTALL status
 **********************************************************************/
agc_status_t agc_job_rr_designate_drive(void)
{
    bool         started = false;                             /* RRDESNB started a designate */
    agc_status_t st      = agc_rr_designate_gimbal(&started); /* RRDESNB / stall status */

    if (st != AGC_STATUS_OK || !started)
    {
        return (st); /* RRDESNB did ENDOFJOB   */
    }

    if (radar_stall(AGC_JOB_RR_DESIGNATE_RESULT, &st))
    {
        return (st);
    }

    return (agc_job_rr_designate_result());
}

/**********************************************************************
 * AGC: none -- RRDESK2 after RADSTALL: bad end (no lock-on or out of limits) -> alarm
 * 503 DESIGNATE FAIL; ENDOFJOB either way.
 *
 * RETURNS: AGC_STATUS_OK (ENDOFJOB)
 **********************************************************************/
agc_status_t agc_job_rr_designate_result(void)
{
    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) == AGC_IMU_STALL_BAD)
    {
        agc_alarm(ALARM_DESIGNATE_FAIL);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * RRDESEND (V44, p.270): only during continuous designate (RADMODES < 0):
 * clear the designate bits, CLRADMOD, wait 1 s, R25 back on, GOPIN.
 * DEVIATION: the AGC holds INHINT into DELAYJOB; here interrupts are
 * released before agc_svc_delayjob, which inhibits them itself.
 *
 * RETURNS: the GOPIN or DELAYJOB status
 **********************************************************************/
agc_status_t agc_extverb_rr_designate_end(void)
{
    if ((agc_erase_radmodes() & RADMODES_CDES) == 0u)
    {
        return (agc_extverb_gopin());
    }

    clear_designate_bits();
    agc_rendz_clradmod();

    return (agc_svc_delayjob(DELAY_1SEC, AGC_JOB_RR_DESIGNATE_END_MONITOR));
}

/**********************************************************************
 * AGC: none -- RRDESEND after the delay: DOWNFLAG NORRMON (enable R25), GOPIN.
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
agc_status_t agc_job_rr_designate_end_monitor(void)
{
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_NORRMON);

    return (agc_extverb_gopin());
}

/* ---- V85 ----------------------------------------------------------------- */

/**********************************************************************
 * VERB85 / DSPRRLOS (p.298-299): FINDVAC RRLOSDSP at priority 5, drop to
 * priority 4, monitor V16N56; every response is B5OFF.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the extended-verb job
 *
 * RETURNS: the TESTXACT, FINDVAC, display or BLANKET status
 **********************************************************************/
agc_status_t agc_extverb_rr_los_display(agc_job_ctx_t *ctx)
{
    bool         ok = false;                     /* TESTXACT got the display */
    agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT / FINDVAC / display status */

    if (!ok)
    {
        return (st);
    }

    st = agc_extverb_schedule(PRIO5, AGC_JOB_RR_LOS_DISPLAY, true);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_exec_prio_change(PRIO4);
    agc_dsky_cont_t cont = {AGC_JOB_RR_LOS_DISPLAY_STOP,
                            AGC_JOB_RR_LOS_DISPLAY_STOP,
                            AGC_JOB_RR_LOS_DISPLAY_STOP};

    st = agc_dsky_display(ctx, AGC_DSKY_GOMARKFR, agc_dsky_vn(16, 56), &cont);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(BIT3));
}

/**********************************************************************
 * AGC: none -- ARCTRIG, then BPL / DAD DPPOSMAX: an angle 0..1 rev (error 2^-28 rev).
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *  I   interpreter state (OVFIND)
 *   sin4  agc_dp_t        I   sin / 4 of the angle
 *   cos4  agc_dp_t        I   cos / 4 of the angle
 *
 * RETURNS: the angle, 0..1 revolution
 **********************************************************************/
static agc_dp_t angle_0_360(agc_interp_t *ip, agc_dp_t sin4, agc_dp_t cos4)
{
    agc_dp_t a = agc_imu_arctrig(ip, sin4, cos4); /* ARCTRIG, rev 2^0 */

    if (a < 0)
    {
        a = agc_dp_add(ip, a, AGC_DP_MAX);
    }

    return (a);
}

/**********************************************************************
 * RRLOSDSP (p.299-300): LOS in NB axes from CDUT, CDUS (RRNBMPAC); its X-Z
 * projection P.  RR-ELEV = angle (P.X, P.Z) about +Y; RR-AZ = angle of the
 * LOS out of the X-Z plane (LOS.Y, LOS.P).  Revolutions.  Again in 1 s.
 *
 * RETURNS: the RRNBMPAC or DELAYJOB status
 **********************************************************************/
agc_status_t agc_job_rr_los_display(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_vec_t los = agc_vec_zero(); /* RR LOS in NB axes, half-unit, 2^1 */
    agc_lm_rrnb(&ip, agc_erase_cdut(), agc_erase_cdus(), &los);

    agc_vec_t xz = los; /* LOS projected on the X-Z plane, half-unit, 2^1 */

    xz.c[1]               = 0;                                                                          /* HI6ZEROS into 8D       */
    xz                    = agc_vec_unit(&ip, xz, NULL, NULL);
    const agc_vec_t unitx = agc_vec_make(AGC_DP_HALF, 0, 0);                                            /* UNITX, half-unit, 2^1 */
    const agc_vec_t unity = agc_vec_make(0, AGC_DP_HALF, 0);                                            /* UNITY, half-unit, 2^1 */
    const agc_vec_t unitz = agc_vec_make(0, 0, AGC_DP_HALF);                                            /* UNITZ, half-unit, 2^1 */
    agc_dp_t        elev  = angle_0_360(&ip, agc_vec_dot(&ip, unitx, xz), agc_vec_dot(&ip, xz, unitz)); /* RR-ELEV, rev 2^0 */
    agc_dp_t        az    = angle_0_360(&ip, agc_vec_dot(&ip, los, unity), agc_vec_dot(&ip, los, xz));  /* RR-AZ, rev 2^0 */

    agc_erase_set_rr_elev(elev);
    agc_erase_set_rr_az(az);

    return (agc_svc_delayjob(DELAY_1SEC, AGC_JOB_RR_LOS_DISPLAY_NEXT));
}

/**********************************************************************
 * AGC: none -- RRLOSDSP after DELAYJOB: again while EXTVBACT bit 5 is set, else ENDEXT.
 *
 * RETURNS: the RRLOSDSP or ENDEXT status
 **********************************************************************/
agc_status_t agc_job_rr_los_display_next(void)
{
    if ((agc_erase_extvbact() & BIT5) != 0u)
    {
        return (agc_job_rr_los_display());
    }

    return (agc_dsky_endmark());
}

/**********************************************************************
 * AGC: none -- V16N56 responses: B5OFF (the RRLOSDSP job then ends itself).
 *
 * RETURNS: the B5OFF status
 **********************************************************************/
agc_status_t agc_job_rr_los_display_stop(void)
{
    agc_svc_b5off(); /* B5OFF */

    return (AGC_STATUS_OK);
}

/* ---- V63 R04, V78/V79 R77 -------------------------------------------------- */

/**********************************************************************
 * CS FLAGWRD3, MASK R04FLBIT (EXTENDED_VERBS.agc R04Z, p.276) -- R04 or R77
 *
 * RETURNS: true for R04 (R04FLAG), false for R77
 **********************************************************************/
static bool radar_test_is_self_test(void)
{
    return (agc_flag_test((agc_flag_id_t) AGC_FLAG_ALARM_521_SUPPRESSED)); /* R04FLAG */
}

/**********************************************************************
 * AGC: none -- R04C / R04C+4: (RR) enable the auto tracker; RTSTMAX; RADSAMP in
 * RTSTMAX cs (A still holds it at TC WAITLIST); R77 returns to pinball,
 * R04 shows the first sample display.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *   rr   bool             I   true: enable the RR auto tracker
 *   max  int              I   RTSTMAX (2 RR, 6 LR), also the first delay
 *                             in cs
 *
 * RETURNS: the WAITLIST status, or the GOPIN / display status
 **********************************************************************/
static agc_status_t radar_test_start(agc_job_ctx_t *ctx, bool rr, int max)
{
    agc_status_t st = AGC_STATUS_OK; /* WAITLIST status */

    if (rr)
    {
        agc_chan_or((int) AGC_CHAN_CHAN12, (uint16_t) BIT14);
    }

    rtstmax = (agc_sp_t) max;
    agc_inhint();
    st      = agc_waitlist_call((uint16_t) max, AGC_TASK_RADSAMP);
    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!radar_test_is_self_test())
    {
        return (agc_extverb_gopin()); /* R77                    */
    }

    return (agc_job_radar_test_first_display(ctx));
}

/**********************************************************************
 * R04X (p.277): LR (RTSTDEX & 6) -> R04L, base 2; RR -> base 0, R04B.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the status of R04L / R04B
 **********************************************************************/
static agc_status_t radar_test_choose(agc_job_ctx_t *ctx)
{
    if (((uint16_t) rtstdex & SIX) != 0u)
    {
        rtstbase = LR_BASE; /* R04L                   */

        return (radar_test_start(ctx, false, LR_SAMPLE_MAX));
    }

    rtstbase = 0;

    return (agc_job_radar_test_auto_check(ctx));
}

/**********************************************************************
 * R04Z (p.276): once a second, clear the counters, copy the LR range
 * scale / position and RR range scale bits of channel 33 into RADMODES.
 * R77 (R04FLAG reset) samples the LR 4 times a second (R04K); R04 asks
 * which radar (V04N12 option 4, default 1 = RR).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the status of R04K / R04A
 **********************************************************************/
static agc_status_t radar_test_setup(agc_job_ctx_t *ctx)
{
    rsampdt     = RSAMP_1SEC;
    rtstloc     = 0;
    rfailcnt    = 0;
    agc_inhint();
    uint16_t rm = (uint16_t) (agc_erase_radmodes() & ~(uint32_t) LRPOSCAL); /* RADMODES */

    rm = (uint16_t) (rm + (agc_chan_read((int) AGC_CHAN_CHAN33) & LRPOSCAL));
    agc_erase_set_radmodes(rm);
    agc_relint();

    if (!radar_test_is_self_test())
    {
        rsampdt  = RSAMP_250MS; /* R04K                   */
        rtstbase = LR_BASE;     /* R04L                   */

        return (radar_test_start(ctx, false, LR_SAMPLE_MAX));
    }

    agc_erase_set_optionx(1, RADAR_RR);
    agc_erase_set_optionx(0, RADAR_OPTION); /* R04A                   */

    return (agc_job_radar_test_option_display(ctx));
}

/**********************************************************************
 * R04 (V63, p.276): RDRUSECK, TESTXACT, R04FLAG (alarm 521 suppressed).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the extended-verb job
 *
 * RETURNS: the RDRUSECK, TESTXACT status, or that of R04Z
 **********************************************************************/
agc_status_t agc_extverb_radar_test(agc_job_ctx_t *ctx)
{
    bool         ok = false;                   /* no radar in use; then TESTXACT */
    agc_status_t st = radar_in_use_check(&ok); /* RDRUSECK / TESTXACT status */

    if (!ok)
    {
        return (st);
    }

    st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    agc_flag_put((agc_flag_id_t) AGC_FLAG_ALARM_521_SUPPRESSED, true);

    return (radar_test_setup(ctx));
}

/**********************************************************************
 * R77 (V78, p.276): RDRUSECK; not while R04 runs; R77FLAG (all radar
 * alarms and tracker fails suppressed).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the extended-verb job
 *
 * RETURNS: the RDRUSECK, ALM/END status, or that of R04Z
 **********************************************************************/
agc_status_t agc_extverb_lr_spurious_test(agc_job_ctx_t *ctx)
{
    bool         ok = false;                   /* no radar in use */
    agc_status_t st = radar_in_use_check(&ok); /* RDRUSECK status */

    if (!ok)
    {
        return (st);
    }

    if (radar_test_is_self_test())
    {
        return (agc_extverb_alm_end());
    }

    agc_flag_put((agc_flag_id_t) AGC_FLAG_RADAR_ALARMS_SUPPRESSED, true);

    return (radar_test_setup(ctx));
}

/**********************************************************************
 * AGC: none -- R04A +2: GOMARKFR V04N12 (V34 R04END, V33 select, V32 again), BLANKET.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display or BLANKET status
 **********************************************************************/
agc_status_t agc_job_radar_test_option_display(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_RADAR_TEST_END, AGC_JOB_RADAR_TEST_SELECT, AGC_JOB_RADAR_TEST_OPTION_DISPLAY};
    agc_status_t    st   = agc_dsky_display(ctx, AGC_DSKY_GOMARKFR, agc_dsky_vn(4, 12), &cont); /* display status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(BIT3));
}

/**********************************************************************
 * AGC: none -- V33 on V04N12: RTSTDEX = OPTIONX +1 (1 RR, 2 LR), R04X.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the status of R04X
 **********************************************************************/
agc_status_t agc_job_radar_test_select(agc_job_ctx_t *ctx)
{
    rtstdex = agc_erase_optionx(1);

    return (radar_test_choose(ctx));
}

/**********************************************************************
 * R04B (p.277): RR auto-mode discrete present (channel 33 bit 2 = 0) ->
 * R04C; otherwise ask for it.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the status of R04C or of the request
 **********************************************************************/
agc_status_t agc_job_radar_test_auto_check(agc_job_ctx_t *ctx)
{
    if ((agc_chan_read((int) AGC_CHAN_CHAN33) & BIT2) == 0u)
    {
        return (radar_test_start(ctx, true, RR_SAMPLE_MAX));
    }

    return (agc_job_radar_test_auto_request(ctx));
}

/**********************************************************************
 * AGC: none -- R04B +5: please perform checklist 201 (select RR auto mode), V50N25:
 * V34 R04END, V33 check again, ENTER ask again.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_radar_test_auto_request(agc_job_ctx_t *ctx)
{
    agc_erase_set_dsptem1(0, RR_AUTO_CHECKLIST);
    agc_dsky_cont_t cont = {AGC_JOB_RADAR_TEST_END,
                            AGC_JOB_RADAR_TEST_AUTO_CHECK,
                            AGC_JOB_RADAR_TEST_AUTO_REQUEST};

    return (agc_dsky_display(ctx, AGC_DSKY_GOMARK4, agc_dsky_vn(50, 25), &cont));
}

/**********************************************************************
 * R04RR / R04LR (p.278): V16N72 RR CDU angles or V16N66 LR range and
 * position, once a second; V33 -> the second display.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_radar_test_first_display(agc_job_ctx_t *ctx)
{
    bool            lr   = ((uint16_t) rtstdex & SIX) != 0u; /* LR sampled */
    agc_dsky_cont_t cont = {AGC_JOB_RADAR_TEST_END,
                            AGC_JOB_RADAR_TEST_SECOND_DISPLAY,
                            AGC_JOB_RADAR_TEST_FIRST_DISPLAY};

    return (agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(16, lr ? 66 : 72), &cont)); /* V16N66 LR / V16N72 RR */
}

/**********************************************************************
 * AGC: none -- V16N78 RR range and range rate or V16N67 LR velocities; V33 -> R04Y.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_radar_test_second_display(agc_job_ctx_t *ctx)
{
    bool            lr   = ((uint16_t) rtstdex & SIX) != 0u; /* LR sampled */
    agc_dsky_cont_t cont = {AGC_JOB_RADAR_TEST_END, AGC_JOB_RADAR_TEST_SWITCH, AGC_JOB_RADAR_TEST_FIRST_DISPLAY};

    return (agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(16, lr ? 67 : 78), &cont)); /* V16N67 LR / V16N78 RR */
}

/**********************************************************************
 * R04Y (p.278): stop sampling and wait 2 s for the last RADARUPT.
 *
 * RETURNS: the DELAYJOB status
 **********************************************************************/
agc_status_t agc_job_radar_test_switch(void)
{
    rsampdt = 0;

    return (agc_svc_delayjob(DELAY_2SECS, AGC_JOB_RADAR_TEST_SWITCH_RESUME));
}

/**********************************************************************
 * AGC: none -- R04Y after the delay: once a second again, and the other radar (RR was
 * sampled -> RTSTDEX 2 = LR; LR -> 1 = RR), R04X.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the status of R04X
 **********************************************************************/
agc_status_t agc_job_radar_test_switch_resume(agc_job_ctx_t *ctx)
{
    rsampdt = RSAMP_1SEC;
    rtstloc = 0;
    rtstdex = (((uint16_t) rtstdex & SIX) != 0u) ? RADAR_RR : RADAR_LR; /* the other radar */

    return (radar_test_choose(ctx));
}

/**********************************************************************
 * R04END (p.279): stop sampling, wait 1.28 s for a pending RUPT.
 *
 * RETURNS: the DELAYJOB status
 **********************************************************************/
agc_status_t agc_job_radar_test_end(void)
{
    rsampdt = 0;

    return (agc_svc_delayjob(DELAY_BIT8, AGC_JOB_RADAR_TEST_END_RESUME));
}

/**********************************************************************
 * AGC: none -- R04END after the delay: RR auto tracker off, R04FLAG reset, ENDEXT.
 *
 * RETURNS: the ENDEXT status
 **********************************************************************/
agc_status_t agc_job_radar_test_end_resume(void)
{
    agc_inhint();
    agc_chan_and((int) AGC_CHAN_CHAN12, (uint16_t) (WORD15 & ~BIT14));
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_ALARM_521_SUPPRESSED); /* DOWNFLAG R04FLAG */
    agc_relint();

    return (agc_dsky_endmark());
}

/**********************************************************************
 * R77END (V79, p.279): stop sampling, wait 320 ms for a pending RUPT.
 *
 * RETURNS: the DELAYJOB status
 **********************************************************************/
agc_status_t agc_extverb_lr_spurious_test_end(void)
{
    rsampdt = 0;

    return (agc_svc_delayjob(DELAY_BIT6, AGC_JOB_LR_SPURIOUS_TEST_END_RESUME));
}

/**********************************************************************
 * AGC: none -- R77END after the delay: R77FLAG reset, GOPIN.
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
agc_status_t agc_job_lr_spurious_test_end_resume(void)
{
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_RADAR_ALARMS_SUPPRESSED); /* DOWNFLAG R77FLAG */

    return (agc_extverb_gopin());
}
