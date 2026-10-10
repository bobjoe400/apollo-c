/*
 * extended_verbs_lm_radar.c -- Luminary099 EXTENDED_VERBS.agc, the radar
 * verbs (pp. 265-280, 298-300):
 *   V40 N72  RRZEROK    zero the RR CDUs
 *   V41 N72  RRDESNBK   designate the RR to loaded gimbal angles, asking
 *                       (AURLOKON) for lock-on or continuous designate
 *   V44      RRDESEND   terminate continuous designate
 *   V60      LRPOS2K    command the landing radar to position 2
 *   V63      R04        radar test: sample the RR or LR once a second
 *   V78/V79  R77/R77END LR spurious return test, start / stop
 *   V85      VERB85     RR line of sight azimuth and elevation (RRLOSDSP)
 * and RDRUSECK, the "is either radar in use" check they share.
 *
 * The radar mode-switching routines called here are in P20-P25.agc and
 * RADAR_LEADIN_ROUTINES.agc (prog-rendz): RRZERO, RRDESNB, LRPOS2, CLRADMOD
 * and the RADSAMP sampling task; their prototypes are in pending.h until
 * that lane publishes them.  RADSTALL is the imu lane's
 * agc_imu_stall_dev(AGC_IMU_DEV_RADAR, ...).
 *
 * DEVIATION (MAKECADR / DESRET): AURLOKON keeps its caller's return CADR
 * in DESRET and jumps back to it after the V04N12 answer, and RRDESK2
 * gives up its VAC area by hand before it stalls.  Here the answer is a
 * continuation job that runs the rest of RRDESNBK; the VAC bookkeeping
 * vanishes (docs/TRANSLATION_GUIDE.md section 4).
 *
 * Erasables owned here (R04/R77 test storage, also read and written by
 * RADSAMP): RSAMPDT, RTSTDEX, RTSTBASE, RTSTMAX, RTSTLOC, RFAILCNT.
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/imu/imu.h"
#include "agc/programs/alignment.h"
#include "agc/io/service_internal.h"
#include "pending.h"

#define WORD15        077777u
#define BIT2          000002u   /* CHAN33: RR auto-mode discrete (inverted)   */
#define BIT3          000004u   /* BLANKET R3                                 */
#define BIT5          000020u   /* EXTVBACT: extended verb active             */
#define BIT14         020000u   /* CHAN12: enable RR auto tracker             */
#define RADMODES_CDES 040000u   /* RADMODES bit 15: continuous designate      */
#define OCT41000      041000u   /* RADMODES bits 15, 10: designate bits       */
#define LRPOSCAL      000444u   /* RADMODES/CHAN33 bits 9, 6, 3 (scales, pos) */
#define SIX           000006u   /* RTSTDEX: LR when nonzero under this mask   */
#define LOCKON_OPTION ((agc_sp_t)6)   /* OPTIONX: V04N12 option code 6        */
#define LOCKON_NONE   ((agc_sp_t)2)   /* OPTIONX +1: 2 = no lock-on           */
#define RADAR_OPTION  ((agc_sp_t)4)   /* OPTIONX: option code (BIT3)          */
#define RADAR_RR      ((agc_sp_t)1)   /* OPTIONX +1 / RTSTDEX: RR             */
#define RADAR_LR      ((agc_sp_t)2)   /* OPTIONX +1 / RTSTDEX: LR             */
#define RR_AUTO_CHECKLIST ((agc_sp_t)00201) /* 201R04: select RR auto mode    */
#define RSAMP_1SEC    ((agc_sp_t)101)  /* 1SEC+1: sample once a second        */
#define RSAMP_250MS   ((agc_sp_t)032)  /* 250MS+1 = CALLCODE: 4 per second    */
#define RR_SAMPLE_MAX 2         /* RTSTMAX for the RR (and first delay, cs)   */
#define LR_SAMPLE_MAX 6         /* RTSTMAX for the LR (and first delay, cs)   */
#define LR_BASE       ((agc_sp_t)2)    /* RTSTBASE for the LR                  */
#define DELAY_1SEC    100u
#define DELAY_2SECS   200u
#define DELAY_BIT8    0200u     /* 1.28 s: a pending radar RUPT               */
#define DELAY_BIT6    040u      /* 320 ms                                     */
#define PRIO4         ((agc_prio_t)004)
#define PRIO5         ((agc_prio_t)005)
#define PRIO20        ((agc_prio_t)020)
#define ALARM_DESIGNATE_FAIL ((agc_alarm_code_t)00503)
#define ALARM_LR_POSITION    ((agc_alarm_code_t)00523)
#define LM_P22_MODE   22        /* DEC22: P22 lunar surface navigation        */

/* R04 / R77 test storage (EBANK4).                                        */
static agc_sp_t rsampdt;        /* sampling interval + 1 (cs); 0 stops it     */
static agc_sp_t rtstdex;        /* which radar sample                          */
static agc_sp_t rtstbase;       /* 0 RR, 2 LR                                  */
static agc_sp_t rtstmax;        /* 2 RR, 6 LR                                  */
static agc_sp_t rtstloc;        /* storage index                               */
static agc_sp_t rfailcnt;       /* bad sample count                            */

agc_sp_t agc_erase_rsampdt(void)            { return rsampdt; }
void     agc_erase_set_rsampdt(agc_sp_t v)  { rsampdt = v; }
agc_sp_t agc_erase_rtstdex(void)            { return rtstdex; }
void     agc_erase_set_rtstdex(agc_sp_t v)  { rtstdex = v; }
agc_sp_t agc_erase_rtstbase(void)           { return rtstbase; }
agc_sp_t agc_erase_rtstmax(void)            { return rtstmax; }
agc_sp_t agc_erase_rtstloc(void)            { return rtstloc; }
void     agc_erase_set_rtstloc(agc_sp_t v)  { rtstloc = v; }
agc_sp_t agc_erase_rfailcnt(void)           { return rfailcnt; }
void     agc_erase_set_rfailcnt(agc_sp_t v) { rfailcnt = v; }

/* ---- shared helpers ------------------------------------------------------ */

/* RDRUSECK (p.280): ALM/END if R29 may be using the RR (NOR29FLG reset),
 * R77 is on, R12 may be using the LR (SERVICER on and LRBYPASS reset), P20
 * is tracking (TRACKFLG and RNDVZFLG) or P22 is running.                  */
static agc_status_t radar_in_use_check(bool *ok)
{
    if (!AGC_ASSERT(ok != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    bool busy = !agc_flag_test((agc_flag_id_t)AGC_FLAG_NO_POWERED_DESIGNATE)
        || agc_flag_test((agc_flag_id_t)AGC_FLAG_RADAR_ALARMS_SUPPRESSED)
        || (agc_flag_test((agc_flag_id_t)AGC_FLAG_SERVICER_RUNNING)
            && !agc_flag_test((agc_flag_id_t)AGC_FLAG_LRBYPASS))
        || (agc_flag_test((agc_flag_id_t)AGC_FLAG_TRACKFLG)
            && agc_flag_test((agc_flag_id_t)AGC_FLAG_RNDVZFLG))
        || (agc_erase_modreg() == (agc_sp_t)LM_P22_MODE);
    *ok = !busy;
    (void)AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM);
    return busy ? agc_extverb_alm_end() : AGC_STATUS_OK;
}

/* RADSTALL: true when the job must return (asleep until wake runs).      */
static bool radar_stall(agc_job_id_t wake, agc_status_t *st)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;
    *st = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, wake, &res);
    (void)AGC_ASSERT(wake != AGC_JOB_NONE);
    return (*st != AGC_STATUS_OK) || (res == AGC_IMU_STALL_SLEEPING);
}

static agc_status_t flag_put(agc_flag_id_t f, bool on)
{
    agc_status_t st = on ? agc_flag_set(f) : agc_flag_clear(f);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;
}

/* RADMODES: clear the designate bits 15 and 10 (CS OCT41000, MASK).      */
static void clear_designate_bits(void)
{
    agc_inhint();
    agc_erase_set_radmodes((uint16_t)(agc_erase_radmodes() & ~(uint32_t)OCT41000));
    agc_relint();
    (void)AGC_ASSERT((agc_erase_radmodes() & OCT41000) == 0u);
}

/* ---- V40 N72, V60 ----------------------------------------------------------- */

/* RRZEROK (V40 N72, p.265): RRZERO, RADSTALL, GOPIN on either return.     */
agc_status_t agc_extverb_rr_zero(agc_job_ctx_t *ctx)
{
    bool ok = false;
    agc_status_t st = radar_in_use_check(&ok);
    if (!AGC_ASSERT(ctx != NULL) || !ok) {
        return st;
    }
    st = agc_rr_zero();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (radar_stall(AGC_JOB_EXTENDED_VERB_GOPIN, &st)) {
        return st;
    }
    return agc_extverb_gopin();
}

/* LRPOS2K (V60, p.265): LRPOS2, RADSTALL; alarm 523 if position 2 is not
 * indicated in time; GOPIN.                                               */
agc_status_t agc_extverb_lr_position_two(agc_job_ctx_t *ctx)
{
    bool ok = false;
    agc_status_t st = radar_in_use_check(&ok);
    if (!AGC_ASSERT(ctx != NULL) || !ok) {
        return st;
    }
    st = agc_radar_lrpos2();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (radar_stall(AGC_JOB_LR_POSITION_TWO_CHECK, &st)) {
        return st;
    }
    return agc_job_lr_position_two_check(ctx);
}

/* LRPOS2K after RADSTALL: bad return -> LRP2ALM (alarm 523), then GOPIN. */
agc_status_t agc_job_lr_position_two_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) == AGC_IMU_STALL_BAD) {
        agc_status_t st = agc_alarm(ALARM_LR_POSITION);
        (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    }
    return agc_extverb_gopin();
}

/* ---- V41 N72 designate, AURLOKON, V44 ---------------------------------------- */

/* RRDESNBK (V41 N72, p.269): not while P20 runs (XACTALM); end any present
 * designation; ask for the gimbal angles (V24N73).                       */
agc_status_t agc_extverb_rr_designate(agc_job_ctx_t *ctx)
{
    bool ok = false;
    agc_status_t st = radar_in_use_check(&ok);
    if (!AGC_ASSERT(ctx != NULL) || !ok) {
        return st;
    }
    st = agc_extverb_testxact(&ok);
    if (!ok) {
        return st;
    }
    if (agc_flag_test((agc_flag_id_t)AGC_FLAG_RNDVZFLG)) {
        st = agc_dsky_falton();                   /* XACTALM                */
        (void)AGC_ASSERT(st == AGC_STATUS_OK);
        return agc_dsky_endext();
    }
    clear_designate_bits();
    return agc_job_rr_designate_load(ctx);
}

/* V24N73 flashing: V34 ends the verb, V33 asks again, ENTER -> AURLOKON. */
agc_status_t agc_job_rr_designate_load(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dsky_cont_t cont = { AGC_JOB_EXTENDED_VERB_END, AGC_JOB_RR_DESIGNATE_LOAD,
                             AGC_JOB_RR_LOCKON_QUERY };
    return agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(24, 73), &cont);
}

/* AURLOKON (p.271): option code 6, default answer 2 (no lock-on).       */
agc_status_t agc_job_rr_lockon_query(agc_job_ctx_t *ctx)
{
    agc_erase_set_optionx(1, LOCKON_NONE);
    agc_erase_set_optionx(0, LOCKON_OPTION);
    (void)AGC_ASSERT(agc_erase_optionx(0) == LOCKON_OPTION);
    return agc_job_rr_lockon_display(ctx);
}

/* AURLOKON -5: GOMARKFR V04N12 (V34 ENDEXT, V33 answer, V32 again), then
 * BLANKET R3 and ENDOFJOB.                                               */
agc_status_t agc_job_rr_lockon_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dsky_cont_t cont = { AGC_JOB_EXTENDED_VERB_END, AGC_JOB_RR_LOCKON_ANSWER,
                             AGC_JOB_RR_LOCKON_DISPLAY };
    agc_status_t st = agc_dsky_display(ctx, AGC_DSKY_GOMARKFR, agc_dsky_vn(4, 12), &cont);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_blanket(BIT3);
}

/* AURLOKON +5 / NOLOKON / AURLKON1: answer 1 = lock on (LOKONSW); 2 = no
 * lock-on: continuous designate (CDESFLAG) and R25 off (NORRMON).  Then
 * the rest of RRDESNBK: show V41 again and schedule RRDESK2.             */
agc_status_t agc_job_rr_lockon_answer(agc_job_ctx_t *ctx)
{
    bool no_lockon = ((uint16_t)agc_erase_optionx(1) & (uint16_t)LOCKON_NONE) != 0u;
    agc_status_t st = flag_put((agc_flag_id_t)AGC_FLAG_LOKONSW, !no_lockon);
    if (no_lockon && st == AGC_STATUS_OK) {
        st = flag_put((agc_flag_id_t)AGC_FLAG_CDESFLAG, true);
    }
    if (no_lockon && st == AGC_STATUS_OK) {
        st = flag_put((agc_flag_id_t)AGC_FLAG_NORRMON, true);
    }
    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    return agc_extverb_redisplay(ctx, 41, AGC_JOB_RR_DESIGNATE_SCHEDULE);
}

/* RRDESNBK after EXDSPRET: FINDVAC RRDESK2 at priority 20, TERMEXTV.     */
agc_status_t agc_job_rr_designate_schedule(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_extverb_schedule(PRIO20, AGC_JOB_RR_DESIGNATE_DRIVE, true);
    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_endext();
}

/* RRDESK2 (p.269): RRDESNB, then RADSTALL for the designate to finish.   */
agc_status_t agc_job_rr_designate_drive(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    bool started = false;
    agc_status_t st = agc_rr_designate_gimbal(&started);
    if (st != AGC_STATUS_OK || !started) {
        return st;                                /* RRDESNB did ENDOFJOB   */
    }
    if (radar_stall(AGC_JOB_RR_DESIGNATE_RESULT, &st)) {
        return st;
    }
    return agc_job_rr_designate_result(ctx);
}

/* RRDESK2 after RADSTALL: bad end (no lock-on or out of limits) -> alarm
 * 503 DESIGNATE FAIL; ENDOFJOB either way.                               */
agc_status_t agc_job_rr_designate_result(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) != AGC_IMU_STALL_BAD) {
        return AGC_STATUS_OK;
    }
    agc_status_t st = agc_alarm(ALARM_DESIGNATE_FAIL);
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    return AGC_STATUS_OK;
}

/* RRDESEND (V44, p.270): only during continuous designate (RADMODES < 0):
 * clear the designate bits, CLRADMOD, wait 1 s, R25 back on, GOPIN.
 * DEVIATION: the AGC holds INHINT into DELAYJOB; here interrupts are
 * released before agc_svc_delayjob, which inhibits them itself.          */
agc_status_t agc_extverb_rr_designate_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if ((agc_erase_radmodes() & RADMODES_CDES) == 0u) {
        return agc_extverb_gopin();
    }
    clear_designate_bits();
    agc_status_t st = agc_rendz_clradmod();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_svc_delayjob(DELAY_1SEC, AGC_JOB_RR_DESIGNATE_END_MONITOR);
}

/* RRDESEND after the delay: DOWNFLAG NORRMON (enable R25), GOPIN.        */
agc_status_t agc_job_rr_designate_end_monitor(agc_job_ctx_t *ctx)
{
    agc_status_t st = flag_put((agc_flag_id_t)AGC_FLAG_NORRMON, false);
    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK) {
        return st;
    }
    return agc_extverb_gopin();
}

/* ---- V85 ----------------------------------------------------------------- */

/* VERB85 / DSPRRLOS (p.298-299): FINDVAC RRLOSDSP at priority 5, drop to
 * priority 4, monitor V16N56; every response is B5OFF.                   */
agc_status_t agc_extverb_rr_los_display(agc_job_ctx_t *ctx)
{
    bool ok = false;
    agc_status_t st = agc_extverb_testxact(&ok);
    if (!AGC_ASSERT(ctx != NULL) || !ok) {
        return st;
    }
    st = agc_extverb_schedule(PRIO5, AGC_JOB_RR_LOS_DISPLAY, true);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    st = agc_exec_prio_change(PRIO4);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_dsky_cont_t cont = { AGC_JOB_RR_LOS_DISPLAY_STOP, AGC_JOB_RR_LOS_DISPLAY_STOP,
                             AGC_JOB_RR_LOS_DISPLAY_STOP };
    st = agc_dsky_display(ctx, AGC_DSKY_GOMARKFR, agc_dsky_vn(16, 56), &cont);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_blanket(BIT3);
}

/* ARCTRIG, then BPL / DAD DPPOSMAX: an angle 0..1 rev (error 2^-28 rev). */
static agc_dp_t angle_0_360(agc_interp_t *ip, agc_dp_t sin4, agc_dp_t cos4)
{
    agc_dp_t a = agc_imu_arctrig(ip, sin4, cos4);
    if (a < 0) {
        a = agc_dp_add(ip, a, AGC_DP_MAX);
    }
    (void)AGC_ASSERT(a >= 0 && a <= AGC_DP_MAX);
    return a;
}

/* RRLOSDSP (p.299-300): LOS in NB axes from CDUT, CDUS (RRNBMPAC); its X-Z
 * projection P.  RR-ELEV = angle (P.X, P.Z) about +Y; RR-AZ = angle of the
 * LOS out of the X-Z plane (LOS.Y, LOS.P).  Revolutions.  Again in 1 s.  */
agc_status_t agc_job_rr_los_display(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_vec_t los = agc_vec_zero();
    agc_status_t st = agc_lm_rrnb(&ip, agc_erase_cdut(), agc_erase_cdus(), &los);
    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    agc_vec_t xz = los;
    xz.c[1] = 0;                                  /* HI6ZEROS into 8D       */
    xz = agc_vec_unit(&ip, xz, NULL, NULL);
    const agc_vec_t unitx = agc_vec_make(AGC_DP_HALF, 0, 0);
    const agc_vec_t unity = agc_vec_make(0, AGC_DP_HALF, 0);
    const agc_vec_t unitz = agc_vec_make(0, 0, AGC_DP_HALF);
    agc_dp_t elev = angle_0_360(&ip, agc_vec_dot(&ip, unitx, xz), agc_vec_dot(&ip, xz, unitz));
    agc_dp_t az = angle_0_360(&ip, agc_vec_dot(&ip, los, unity), agc_vec_dot(&ip, los, xz));
    agc_erase_set_rr_elev(elev);
    agc_erase_set_rr_az(az);
    return agc_svc_delayjob(DELAY_1SEC, AGC_JOB_RR_LOS_DISPLAY_NEXT);
}

/* RRLOSDSP after DELAYJOB: again while EXTVBACT bit 5 is set, else ENDEXT. */
agc_status_t agc_job_rr_los_display_next(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if ((agc_erase_extvbact() & BIT5) != 0u) {
        return agc_job_rr_los_display(ctx);
    }
    return agc_dsky_endext();
}

/* V16N56 responses: B5OFF (the RRLOSDSP job then ends itself).            */
agc_status_t agc_job_rr_los_display_stop(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    return agc_svc_b5off();
}

/* ---- V63 R04, V78/V79 R77 -------------------------------------------------- */

static bool radar_test_is_r04(void)
{
    return agc_flag_test((agc_flag_id_t)AGC_FLAG_ALARM_521_SUPPRESSED); /* R04FLAG */
}

/* R04C / R04C+4: (RR) enable the auto tracker; RTSTMAX; RADSAMP in
 * RTSTMAX cs (A still holds it at TC WAITLIST); R77 returns to pinball,
 * R04 shows the first sample display.                                    */
static agc_status_t radar_test_start(agc_job_ctx_t *ctx, bool rr, int max)
{
    if (!AGC_ASSERT(max == RR_SAMPLE_MAX || max == LR_SAMPLE_MAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = AGC_STATUS_OK;
    if (rr) {
        st = agc_chan_or((int)AGC_CHAN_CHAN12, (uint16_t)BIT14);
        (void)AGC_ASSERT(st == AGC_STATUS_OK);
    }
    rtstmax = (agc_sp_t)max;
    agc_inhint();
    st = agc_waitlist_call((uint16_t)max, AGC_TASK_RADSAMP);
    agc_relint();
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (!radar_test_is_r04()) {
        return agc_extverb_gopin();               /* R77                    */
    }
    return agc_job_radar_test_first_display(ctx);
}

/* R04X (p.277): LR (RTSTDEX & 6) -> R04L, base 2; RR -> base 0, R04B.     */
static agc_status_t radar_test_choose(agc_job_ctx_t *ctx)
{
    if (((uint16_t)rtstdex & SIX) != 0u) {
        rtstbase = LR_BASE;                       /* R04L                   */
        return radar_test_start(ctx, false, LR_SAMPLE_MAX);
    }
    rtstbase = 0;
    return agc_job_radar_test_auto_check(ctx);
}

/* R04Z (p.276): once a second, clear the counters, copy the LR range
 * scale / position and RR range scale bits of channel 33 into RADMODES.
 * R77 (R04FLAG reset) samples the LR 4 times a second (R04K); R04 asks
 * which radar (V04N12 option 4, default 1 = RR).                         */
static agc_status_t radar_test_setup(agc_job_ctx_t *ctx)
{
    rsampdt = RSAMP_1SEC;
    rtstloc = 0;
    rfailcnt = 0;
    agc_inhint();
    uint16_t rm = (uint16_t)(agc_erase_radmodes() & ~(uint32_t)LRPOSCAL);
    rm = (uint16_t)(rm + (agc_chan_read((int)AGC_CHAN_CHAN33) & LRPOSCAL));
    agc_erase_set_radmodes(rm);
    agc_relint();
    (void)AGC_ASSERT(rm <= WORD15);
    if (!radar_test_is_r04()) {
        rsampdt = RSAMP_250MS;                    /* R04K                   */
        rtstbase = LR_BASE;                       /* R04L                   */
        return radar_test_start(ctx, false, LR_SAMPLE_MAX);
    }
    agc_erase_set_optionx(1, RADAR_RR);
    agc_erase_set_optionx(0, RADAR_OPTION);       /* R04A                   */
    return agc_job_radar_test_option_display(ctx);
}

/* R04 (V63, p.276): RDRUSECK, TESTXACT, R04FLAG (alarm 521 suppressed). */
agc_status_t agc_extverb_radar_test(agc_job_ctx_t *ctx)
{
    bool ok = false;
    agc_status_t st = radar_in_use_check(&ok);
    if (!AGC_ASSERT(ctx != NULL) || !ok) {
        return st;
    }
    st = agc_extverb_testxact(&ok);
    if (!ok) {
        return st;
    }
    st = flag_put((agc_flag_id_t)AGC_FLAG_ALARM_521_SUPPRESSED, true);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return radar_test_setup(ctx);
}

/* R77 (V78, p.276): RDRUSECK; not while R04 runs; R77FLAG (all radar
 * alarms and tracker fails suppressed).                                  */
agc_status_t agc_extverb_lr_spurious_test(agc_job_ctx_t *ctx)
{
    bool ok = false;
    agc_status_t st = radar_in_use_check(&ok);
    if (!AGC_ASSERT(ctx != NULL) || !ok) {
        return st;
    }
    if (radar_test_is_r04()) {
        return agc_extverb_alm_end();
    }
    st = flag_put((agc_flag_id_t)AGC_FLAG_RADAR_ALARMS_SUPPRESSED, true);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return radar_test_setup(ctx);
}

/* R04A +2: GOMARKFR V04N12 (V34 R04END, V33 select, V32 again), BLANKET. */
agc_status_t agc_job_radar_test_option_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dsky_cont_t cont = { AGC_JOB_RADAR_TEST_END, AGC_JOB_RADAR_TEST_SELECT,
                             AGC_JOB_RADAR_TEST_OPTION_DISPLAY };
    agc_status_t st = agc_dsky_display(ctx, AGC_DSKY_GOMARKFR, agc_dsky_vn(4, 12), &cont);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_blanket(BIT3);
}

/* V33 on V04N12: RTSTDEX = OPTIONX +1 (1 RR, 2 LR), R04X.                */
agc_status_t agc_job_radar_test_select(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rtstdex = agc_erase_optionx(1);
    (void)AGC_ASSERT(rtstdex >= 0);
    return radar_test_choose(ctx);
}

/* R04B (p.277): RR auto-mode discrete present (channel 33 bit 2 = 0) ->
 * R04C; otherwise ask for it.                                             */
agc_status_t agc_job_radar_test_auto_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if ((agc_chan_read((int)AGC_CHAN_CHAN33) & BIT2) == 0u) {
        return radar_test_start(ctx, true, RR_SAMPLE_MAX);
    }
    return agc_job_radar_test_auto_request(ctx);
}

/* R04B +5: please perform checklist 201 (select RR auto mode), V50N25:
 * V34 R04END, V33 check again, ENTER ask again.                          */
agc_status_t agc_job_radar_test_auto_request(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_erase_set_dsptem1(0, RR_AUTO_CHECKLIST);
    agc_dsky_cont_t cont = { AGC_JOB_RADAR_TEST_END, AGC_JOB_RADAR_TEST_AUTO_CHECK,
                             AGC_JOB_RADAR_TEST_AUTO_REQUEST };
    return agc_dsky_display(ctx, AGC_DSKY_GOMARK4, agc_dsky_vn(50, 25), &cont);
}

/* R04RR / R04LR (p.278): V16N72 RR CDU angles or V16N66 LR range and
 * position, once a second; V33 -> the second display.                    */
agc_status_t agc_job_radar_test_first_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    bool lr = ((uint16_t)rtstdex & SIX) != 0u;
    agc_dsky_cont_t cont = { AGC_JOB_RADAR_TEST_END, AGC_JOB_RADAR_TEST_SECOND_DISPLAY,
                             AGC_JOB_RADAR_TEST_FIRST_DISPLAY };
    return agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(16, lr ? 66 : 72), &cont);
}

/* V16N78 RR range and range rate or V16N67 LR velocities; V33 -> R04Y.   */
agc_status_t agc_job_radar_test_second_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    bool lr = ((uint16_t)rtstdex & SIX) != 0u;
    agc_dsky_cont_t cont = { AGC_JOB_RADAR_TEST_END, AGC_JOB_RADAR_TEST_SWITCH,
                             AGC_JOB_RADAR_TEST_FIRST_DISPLAY };
    return agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(16, lr ? 67 : 78), &cont);
}

/* R04Y (p.278): stop sampling and wait 2 s for the last RADARUPT.        */
agc_status_t agc_job_radar_test_switch(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rsampdt = 0;
    return agc_svc_delayjob(DELAY_2SECS, AGC_JOB_RADAR_TEST_SWITCH_RESUME);
}

/* R04Y after the delay: once a second again, and the other radar (RR was
 * sampled -> RTSTDEX 2 = LR; LR -> 1 = RR), R04X.                         */
agc_status_t agc_job_radar_test_switch_resume(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rsampdt = RSAMP_1SEC;
    rtstloc = 0;
    rtstdex = (((uint16_t)rtstdex & SIX) != 0u) ? RADAR_RR : RADAR_LR;
    return radar_test_choose(ctx);
}

/* R04END (p.279): stop sampling, wait 1.28 s for a pending RUPT.         */
agc_status_t agc_job_radar_test_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rsampdt = 0;
    return agc_svc_delayjob(DELAY_BIT8, AGC_JOB_RADAR_TEST_END_RESUME);
}

/* R04END after the delay: RR auto tracker off, R04FLAG reset, ENDEXT.    */
agc_status_t agc_job_radar_test_end_resume(agc_job_ctx_t *ctx)
{
    agc_inhint();
    agc_status_t st = agc_chan_and((int)AGC_CHAN_CHAN12, (uint16_t)(WORD15 & ~BIT14));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = flag_put((agc_flag_id_t)AGC_FLAG_ALARM_521_SUPPRESSED, false);
    agc_relint();
    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK) {
        return st;
    }
    return agc_dsky_endext();
}

/* R77END (V79, p.279): stop sampling, wait 320 ms for a pending RUPT.    */
agc_status_t agc_extverb_lr_spurious_test_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    rsampdt = 0;
    return agc_svc_delayjob(DELAY_BIT6, AGC_JOB_LR_SPURIOUS_TEST_END_RESUME);
}

/* R77END after the delay: R77FLAG reset, GOPIN.                          */
agc_status_t agc_job_lr_spurious_test_end_resume(agc_job_ctx_t *ctx)
{
    agc_status_t st = flag_put((agc_flag_id_t)AGC_FLAG_RADAR_ALARMS_SUPPRESSED, false);
    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK) {
        return st;
    }
    return agc_extverb_gopin();
}
