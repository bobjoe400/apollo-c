/*
 * extended_verbs.c -- EXTENDED_VERBS.agc, the part common to both vehicles
 * (Comanche055 pp. 235-262, Luminary099 pp. 261-296): the verb fan
 * LST2FAN for verbs 40-99, TESTXACT, ALM/END, GOPIN, CHKPOOH, CKMODCAD,
 * OP/INERT, and the verbs that are the same in both ropes:
 *   V40 N20  zero the IMU CDUs (VBZERO / IMUZEROK)
 *   V41 N20  coarse align the IMU from the keyboard (IMUCOARK / ICORK2)
 *   V42      fine align the IMU with gyro torquing (IMUFINEK / FINEK2)
 *   V49      start an automatic attitude maneuver (CREWMANU -> R62DISP)
 *   V50 etc. please-perform verbs (GOLOADLV)
 *   V55      align the AGC clock (ALINTIME / UPDATIME)
 *   V64      S-band antenna angles (VB64 -> SBANDANT)
 *   V67      W-matrix monitor, V69 hardware restart, V74 erasable dump,
 *   V82      orbit parameters (R30), V89 final attitude (R63), V90 out of
 *   plane (R36), V91 show-banksum (SHOWSUM).
 * V70-V73 are the update program (uplink_update.c).  Everything else is
 * in extended_verbs_csm.c / extended_verbs_lm.c.
 *
 * DEVIATION (job): in the AGC GOEXTVB runs on in the CHARIN job.  Pinball
 * calls agc_extverb() without a job context, so GOEXTVB starts a job
 * (AGC_JOB_EXTENDED_VERB, CHARIN's priority 30) that runs the fan.  Each
 * place where a verb waits for the operator or a mode switch (GOXDSPF,
 * EXDSPRET, IMUSTALL) is a separate continuation job, as
 * docs/TRANSLATION_GUIDE.md section 4 prescribes.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/integration.h"
#include "agc/programs/rendezvous.h"

#define WORD15                    077777u
#define BIT6                      000040u
#define OCT24                     000024u /* EXTVBACT: bits 3 and 5                     */
#define OC24100                   024100u /* FLAGWRD4: priority displays use the DSKY   */
#define PRIO4                     ((agc_prio_t) 004)
#define PRIO5                     ((agc_prio_t) 005)
#define PRIO7                     ((agc_prio_t) 007)
#define PRIO10                    ((agc_prio_t) 010)
#define PRIO30                    ((agc_prio_t) 030)
#define NOUN_ICDU                 20      /* N20 ICDU angles                            */
#define NVSUB_BLANK_BUT_MODE_VERB (-2)    /* CS TWO, TC NVSUB              */
#define TIME_UPDATE_VERB          23      /* DEC23: load with V23 before the update     */
#define CLOCK_DELTA_WORD          4       /* DSPTEM2 +1, +2 hold the delta time         */

static uint16_t extvbact;      /* EXTVBACT                           */
static bool     sv_from_other; /* V47 (LMTOCM) rather than V66        */
static int      xv_verb;       /* VERBREG at GOEXTVB                 */
static int      xv_noun;       /* NOUNREG at GOEXTVB                 */

/* ---- EXTVBACT ---------------------------------------------------------- */

uint16_t agc_erase_extvbact(void)
{
    if (!AGC_ASSERT(extvbact <= WORD15))
    {
        return (0); /* no valid result */
    }

    return (extvbact);
}

void agc_erase_set_extvbact(uint16_t v)
{
    if (AGC_ASSERT(v <= WORD15))
    {
        extvbact = v;
    }
}

/* TS EXTVBACT by another program (P61 locks out the extended verbs).     */
void agc_svc_set_extvbact(uint16_t v)
{
    if (!AGC_ASSERT(v <= WORD15))
    {
        return;
    }

    extvbact = v;
}

bool agc_extverb_busy(void)
{
    if (!AGC_ASSERT(extvbact <= WORD15))
    {
        return (false);
    }

    return (extvbact != 0u);
}

int agc_extverb_verb(void)
{
    if (!AGC_ASSERT(xv_verb == 0 || (xv_verb >= 40 && xv_verb <= 99)))
    {
        return (0); /* no valid result */
    }

    return (xv_verb);
}

int agc_extverb_noun(void)
{
    if (!AGC_ASSERT(xv_noun >= 0 && xv_noun <= 99))
    {
        return (0); /* no valid result */
    }

    return (xv_noun);
}

/* ---- common exits ------------------------------------------------------ */

/* GOPIN: TC POSTJUMP, CADR PINBRNCH.                                      */
agc_status_t agc_extverb_gopin(void)
{
    agc_status_t st = agc_dsky_pinbrnch();

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* ALM/END: operator error light, then GOPIN.                              */
agc_status_t agc_extverb_alm_end(void)
{
    agc_dsky_falton();

    return (agc_extverb_gopin());
}

/* TESTXACT (C p.237 / L p.264): refuse (ALM/END) if an extended verb or a
 * priority display holds the DSKY; otherwise mark the extended-verb display
 * system busy (EXTVBACT bits 3, 5) and blank all but MODE and VERB.       */
agc_status_t agc_extverb_testxact(bool *ok)
{
    if (!AGC_ASSERT(ok != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *ok = false;

    if (extvbact != 0u || (agc_flagword_get(4) & OC24100) != 0u)
    {
        return (agc_extverb_alm_end());
    }

    extvbact        = OCT24; /* SETXTACT        */
    agc_status_t st = agc_dsky_nvsub_blank(NVSUB_BLANK_BUT_MODE_VERB);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *ok = true;

    return (AGC_STATUS_OK);
}

/* CHKPOOH: true if the major mode is P00 (MODREG +-0), else ALM/END.     */
bool agc_extverb_chkpooh(void)
{
    if (agc_erase_modreg() == 0)
    {
        return (true);
    }

    agc_status_t st = agc_extverb_alm_end();

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (false);
    }

    return (false);
}

/* CKMODCAD: true if nobody waits on IMU mode switching (MODECADR = 0),
 * else ALM/END.                                                           */
bool agc_extverb_ckmodcad(void)
{
    if (agc_erase_modecadr(0) == 0)
    {
        return (true);
    }

    agc_status_t st = agc_extverb_alm_end();

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (false);
    }

    return (false);
}

/* GOLOADLV (V50 and the other please-perform verbs): FLASHOFF, then the
 * same recall as a completed load (LOADLV1).                              */
agc_status_t agc_extverb_goloadlv(void)
{
    agc_chan_and((int) AGC_CHAN_DSALMOUT, (uint16_t) (WORD15 & ~BIT6));

    return (agc_dsky_loadlv());
}

/* TC UPFLAG / DOWNFLAG, ADRES f, TC GOPIN.                               */
agc_status_t agc_extverb_upflag(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t w = agc_svc_upflag(f); /* the new flagword        */

    if (!AGC_ASSERT((w & agc_flag_mask(f)) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_gopin());
}

agc_status_t agc_extverb_downflag(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t w = agc_svc_downflag(f); /* the new flagword        */

    if (!AGC_ASSERT((w & agc_flag_mask(f)) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_gopin());
}

/* CAF PRIOn, TC NOVAC/FINDVAC, 2CADR job, then ENDOFJOB (return).        */
agc_status_t agc_extverb_schedule(agc_prio_t prio, agc_job_id_t job, bool vac)
{
    if (!AGC_ASSERT(job != AGC_JOB_NONE) || !AGC_ASSERT(prio > 0u && prio <= 037u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_status_t st = vac ? agc_exec_findvac(prio, job) : agc_exec_novac(prio, job);

    agc_relint();

    return (st);
}

/* GOXDSPF (= GOMARKF): flash VERB/NOUN; V34 ends the verb (TERMEXTV).    */
agc_status_t agc_extverb_flash(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t on_proceed, agc_job_id_t on_enter)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(verb <= 99u && noun <= 99u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_cont_t cont = {AGC_JOB_EXTENDED_VERB_END, on_proceed, on_enter};

    return (agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(verb, noun), &cont));
}

/* EXDSPRET: show our own verb again (VN vv00) and continue at next.      */
agc_status_t agc_extverb_redisplay(const agc_job_ctx_t *ctx, uint8_t verb, agc_job_id_t next)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(verb >= 40u && verb <= 99u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_cont_t cont = {next, next, next};

    return (agc_dsky_display(ctx, AGC_DSKY_EXDSPRET, agc_dsky_vn(verb, 0), &cont));
}

/* IMUSTALL with the continuation wake: GOOD or BAD run it now.            */
static agc_status_t stall_then(agc_job_ctx_t *ctx, agc_job_id_t wake)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;
    agc_status_t           st  = agc_imu_stall(wake, &res);

    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (res == AGC_IMU_STALL_SLEEPING)
    {
        return (AGC_STATUS_OK); /* woken at wake           */
    }

    if (wake == AGC_JOB_EXTENDED_VERB_GOPIN)
    {
        return (agc_extverb_gopin());
    }

    return (agc_dsky_endmark());
}

/* ---- GOEXTVB ----------------------------------------------------------- */

/* GOEXTVB (PINBALL VERBFAN): verb 40-99 keyed; the display system has been
 * released.  Starts the extended-verb job (see DEVIATION above).         */
agc_status_t agc_extverb(int verb, int noun)
{
    if (!AGC_ASSERT(verb >= 40 && verb <= 99) || !AGC_ASSERT(noun >= 0 && noun <= 99))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    xv_verb = verb;
    xv_noun = noun;

    return (agc_extverb_schedule(PRIO30, AGC_JOB_EXTENDED_VERB, false));
}

/* VBZERO (V40, C p.238 / L p.265), IMU part: zero the ICDUs, then GOPIN. */
static agc_status_t verb_zero_imu(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !agc_extverb_ckmodcad())
    {
        return (AGC_STATUS_OK);
    }

    agc_status_t st = agc_imu_zero(); /* IMUZERO         */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (stall_then(ctx, AGC_JOB_EXTENDED_VERB_GOPIN)); /* both returns GOPIN */
}

/* IMUCOARK (V41 N20): load THETAD by V25N22, then ICORK2.                */
static agc_status_t verb_coarse_imu(agc_job_ctx_t *ctx)
{
    bool ok = false;

    if (!AGC_ASSERT(ctx != NULL) || !agc_extverb_ckmodcad())
    {
        return (AGC_STATUS_OK);
    }

    agc_status_t st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    return (agc_extverb_flash(ctx, 25, 22, AGC_JOB_KEYBOARD_COARSE_ALIGN, AGC_JOB_KEYBOARD_COARSE_ALIGN));
}

/* IMUFINEK (V42): load gyro torquing angles by V25N93.                   */
static agc_status_t verb_fine_align(agc_job_ctx_t *ctx)
{
    bool ok = false;

    if (!AGC_ASSERT(ctx != NULL) || !agc_extverb_ckmodcad())
    {
        return (AGC_STATUS_OK);
    }

    agc_status_t st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    return (agc_extverb_flash(ctx, 25, 93, AGC_JOB_KEYBOARD_FINE_ALIGN, AGC_JOB_KEYBOARD_FINE_ALIGN));
}

/* ALINTIME (V55): load the clock increment by V25N24 (LM: as R33 at
 * priority 7).                                                            */
static agc_status_t verb_clock_align(agc_job_ctx_t *ctx)
{
    bool         ok = false;
    agc_status_t st = agc_extverb_testxact(&ok);

    if (!AGC_ASSERT(ctx != NULL) || !ok)
    {
        return (st);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_exec_prio_change(PRIO7);
    }

    return (agc_extverb_flash(ctx, 25, 24, AGC_JOB_EXTENDED_VERB_END, AGC_JOB_CLOCK_ALIGN_UPDATE));
}

/* TESTXACT (and CHKPOOH when pooh), then schedule job at prio.           */
static agc_status_t xact_and_schedule(bool pooh, agc_prio_t prio, agc_job_id_t job)
{
    bool ok = false;

    if (pooh && !agc_extverb_chkpooh())
    {
        return (AGC_STATUS_OK);
    }

    agc_status_t st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    if (!AGC_ASSERT(job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_schedule(prio, job, true)); /* FINDVAC         */
}

/* V82PERF: R30 runs on in this job at priority 7 (V82CALL must not be a
 * FINDVAC job).                                                           */
static agc_status_t verb_orbit_parameters(agc_job_ctx_t *ctx)
{
    bool         ok = false;
    agc_status_t st = agc_extverb_testxact(&ok);

    if (!AGC_ASSERT(ctx != NULL) || !ok)
    {
        return (st);
    }

    agc_exec_prio_change(PRIO7);

    return (agc_job_orbit_parameter_display(ctx)); /* POSTJUMP V82CALL */
}

/* VERB69: force a hardware restart (the AGC does TC VERB69, a TC trap).  */
static agc_status_t verb_hardware_restart(void)
{
    agc_status_t st = agc_rupt_go();

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* V74 DNEDUMP: replace the downlist by the erasable dump.                */
static agc_status_t verb_erasable_dump(void)
{
    agc_status_t st = agc_telem_start_dump();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_gopin());
}

/* SDISPLAY / NOKILL: flash V05N01 with R1-R3 = SKEEP1-3 (sum, bank,
 * bugger word).  V34 ends, V33 sums the next bank, ENTER shows it again.  */
static agc_status_t banksum_display(const agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_cont_t cont = {AGC_JOB_SHOW_BANKSUM_END, AGC_JOB_SHOW_BANKSUM_NEXT,
                            AGC_JOB_SHOW_BANKSUM_DISPLAY};

    return (agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(5, 1), &cont));
}

/* SHOWSUM (V91, C p.258 / L p.288): P00 only; turn self-check off and sum
 * the first bank (LM: at priority 7 so other CHARINs can run).            */
static agc_status_t verb_show_banksum(agc_job_ctx_t *ctx)
{
    bool ok = false;

    if (!AGC_ASSERT(ctx != NULL) || !agc_extverb_chkpooh())
    {
        return (AGC_STATUS_OK);
    }

    agc_status_t st = agc_extverb_testxact(&ok);

    if (!ok)
    {
        return (st);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_exec_prio_change(PRIO7);
    }

    st = agc_selfcheck_showsum_start(); /* SKEEP6 = +1, SMODE = 0 */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (banksum_display(ctx));
}

/* The shared verbs; NOT_FOUND for the vehicle fans.                       */
static agc_status_t fan_shared(agc_job_ctx_t *ctx, int verb)
{
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM);

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    switch (verb)
    {
        case 40:
            return ((xv_noun == NOUN_ICDU) ? verb_zero_imu(ctx) : AGC_STATUS_NOT_FOUND);
        case 41:
            return ((xv_noun == NOUN_ICDU) ? verb_coarse_imu(ctx) : AGC_STATUS_NOT_FOUND);
        case 42:
            return (verb_fine_align(ctx));
        case 49:
            return (xact_and_schedule(true, PRIO10, AGC_JOB_CREW_DEFINED_MANEUVER_DISPLAY));
        case 50:
        case 97:
        case 99:
            return (agc_extverb_goloadlv());
        case 55:
            return (verb_clock_align(ctx));
        case 64:
            return (xact_and_schedule(true, PRIO4, lm ? AGC_JOB_SBANDANT_LM : AGC_JOB_SBANDANT_CM));
        case 66:
            return (agc_extverb_state_vector_copy(false));
        case 67:
            return (xact_and_schedule(false, PRIO5, AGC_JOB_W_MATRIX_MONITOR));
        case 69:
            return (verb_hardware_restart());
        case 70:
        case 71:
        case 72:
        case 73:
            return (agc_uplink_update_start(ctx, verb));
        case 74:
            return (verb_erasable_dump());
        case 82:
            return (verb_orbit_parameters(ctx));
        case 89:
            return (xact_and_schedule(true, PRIO10, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE));
        case 90:
            return (xact_and_schedule(false, PRIO7, AGC_JOB_OUT_OF_PLANE_DISPLAY));
        case 91:
            return (verb_show_banksum(ctx));
        default:
            break;
    }

    return (AGC_STATUS_NOT_FOUND);
}

/* LST2FAN (C p.236 / L p.262), run as the extended-verb job.             */
agc_status_t agc_job_extended_verb(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(xv_verb >= 40 && xv_verb <= 99))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = fan_shared(ctx, xv_verb);

    if (st != AGC_STATUS_NOT_FOUND)
    {
        return (st);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return (agc_extverb_lm(ctx, xv_verb));
    }

    return (agc_extverb_csm(ctx, xv_verb));
}

/* ---- continuation jobs ------------------------------------------------- */

/* ENDEXT (TERMEXTV, ENDEXTVB) as a job: release the display systems.     */
agc_status_t agc_job_extended_verb_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_dsky_endmark();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* GOPIN as a job (after an IMUSTALL in V40).                              */
agc_status_t agc_job_extended_verb_gopin(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_gopin());
}

/* ICORK2 (V41): show V41 again, then coarse align.                       */
agc_status_t agc_job_keyboard_coarse_align(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_redisplay(ctx, 41, AGC_JOB_KEYBOARD_COARSE_DRIVE));
}

/* ICORK2 +3: IMUCOARS to THETAD, IMUSTALL, ENDEXT either way.            */
agc_status_t agc_job_keyboard_coarse_drive(agc_job_ctx_t *ctx)
{
    uint16_t thetad[3];

    for (int i = 0; i < 3; i++)
    {
        thetad[i] = agc_erase_thetad(i);
    }

    agc_status_t st = agc_imu_coarse(thetad);

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (agc_dsky_endmark());
    }

    return (stall_then(ctx, AGC_JOB_EXTENDED_VERB_END));
}

/* IMUFINEK after the load: show V42 again.                                */
agc_status_t agc_job_keyboard_fine_align(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_extverb_redisplay(ctx, 42, AGC_JOB_KEYBOARD_FINE_MODE));
}

/* IMUFINE, IMUSTALL: bad -> ENDEXT, good -> FINEK2.                       */
agc_status_t agc_job_keyboard_fine_mode(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_imu_fine();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (agc_dsky_endmark());
    }

    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;

    st = agc_imu_stall(AGC_JOB_KEYBOARD_GYRO_TORQUE, &res);

    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return (st);
    }

    return (agc_job_keyboard_gyro_torque(ctx));
}

/* FINEK2: torque the gyros by the angles pinball left in OGC, IGC, MGC;
 * IMUSTALL; ENDEXT.  Entered after the fine-align stall (bad -> ENDEXT). */
agc_status_t agc_job_keyboard_gyro_torque(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || agc_imu_stall_outcome(AGC_IMU_DEV_IMU) == AGC_IMU_STALL_BAD)
    {
        return (agc_dsky_endmark());
    }

    agc_status_t st = agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_KEYBOARD_GYRO_TORQUE);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* gyros busy: woken again */
    }

    if (st != AGC_STATUS_OK)
    {
        return (agc_dsky_endmark());
    }

    return (stall_then(ctx, AGC_JOB_EXTENDED_VERB_END));
}

/* Signed value of a 15-bit ones'-complement word.                        */
static int32_t ones_value(uint16_t w)
{
    if (!AGC_ASSERT(w <= WORD15))
    {
        return (0); /* no valid result */
    }

    return (((w & 040000u) != 0u) ? -(int32_t) (WORD15 - w) : (int32_t) w);
}

/* UPDATIME (V55): only after a V23 load, add the increment in DSPTEM2 +1,
 * +2 (cs, DP) to TIME2,TIME1 (TPAGREE gives sign agreement; the counter
 * wraps at 2^28 like TIME2).                                              */
agc_status_t agc_job_clock_align_update(agc_job_ctx_t *ctx)
{
    uint8_t      verb = 0;
    uint8_t      noun = 0;
    agc_status_t st   = agc_dsky_response(&verb, &noun);

    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK || verb != TIME_UPDATE_VERB)
    {
        return (agc_dsky_endmark()); /* not a V23 data load     */
    }

    int32_t delta = ones_value(agc_dsky_dsptem(CLOCK_DELTA_WORD)) * 040000 + ones_value(agc_dsky_dsptem(CLOCK_DELTA_WORD + 1));

    if (!AGC_ASSERT(delta > -(int32_t) AGC_TIME_MODULUS_CS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    int64_t t = (int64_t) agc_time_now_cs() + (int64_t) delta;

    t  = ((t % (int64_t) AGC_TIME_MODULUS_CS) + (int64_t) AGC_TIME_MODULUS_CS) % (int64_t) AGC_TIME_MODULUS_CS;
    agc_time_set_cs((uint32_t) t);
    st = AGC_STATUS_OK;
    agc_relint();

    return (agc_dsky_endmark()); /* UPDTMEND                */
}

/* NOKILL (ENTER on the V05N01): show the same bank again.                 */
agc_status_t agc_job_show_banksum_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (banksum_display(ctx));
}

/* NXTBNK (V33): sum the next bank (after the last: bank 0 again, ENDSUMS
 * with SKEEP6 = +1 -> STSHOSUM) and show it.                              */
agc_status_t agc_job_show_banksum_next(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_selfcheck_showsum_next();

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (agc_dsky_endmark());
    }

    return (banksum_display(ctx));
}

/* V34: SKEEP1 = SELFADRS (self-check restarts from SELFCHK), ENDEXT.     */
agc_status_t agc_job_show_banksum_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_selfcheck_showsum_end();

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_endmark());
}

/* ATTACHED (V66) / LMTOCMSV (CSM V47): FINDVAC the copy at priority 10. */
agc_status_t agc_extverb_state_vector_copy(bool from_other)
{
    if (!AGC_ASSERT(!from_other || agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    sv_from_other = from_other;

    return (agc_extverb_schedule(PRIO10, AGC_JOB_STATE_VECTOR_COPY, true));
}

/* ATTACHIT / LMTOCM: with INTSTALL held, copy the 42-word permanent state
 * of one vehicle into the other's (GENTRAN OCT51 words) with its moon
 * flag (MOONTHIS / MOONOTH), refresh the temporary set (PTOACSM, PTOALEM),
 * INTWAKE, PINBRNCH.
 * DEVIATION: SVDWN1/SVDWN2 (downlink copies) are not called; the down-
 * telemetry reads the state vectors directly (down_telemetry.c).         */
agc_status_t agc_job_state_vector_copy(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_integ_install(0, AGC_JOB_STATE_VECTOR_COPY);

    if (!AGC_ASSERT(ctx != NULL) || st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* sleeps in INTSTALL      */
    }

    bool           lm    = (agc_state_vehicle() == AGC_VEHICLE_LM);
    agc_vehicle_t  self  = lm ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM;
    agc_vehicle_t  other = lm ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM;
    agc_vehicle_t  from  = sv_from_other ? other : self;
    agc_integ_sv_t sv;

    st = agc_integ_perm_get(from, &sv);

    if (st == AGC_STATUS_OK)
    {
        st = agc_integ_perm_set(sv_from_other ? self : other, &sv);
    }

    agc_flag_id_t mself = (agc_flag_id_t) (lm ? AGC_FLAG_LMOONFLG : AGC_FLAG_CMOONFLG);
    agc_flag_id_t moth  = (agc_flag_id_t) (lm ? AGC_FLAG_CMOONFLG : AGC_FLAG_LMOONFLG);
    bool          moon  = agc_flag_test(sv_from_other ? moth : mself);
    agc_flag_id_t dst   = sv_from_other ? mself : moth;

    agc_flag_put(dst, moon);
    agc_interp_t ip;

    agc_interp_init(&ip);
    bool         surface = false;
    agc_status_t sp      = lm ? agc_integ_ptoalem(&ip, agc_integ_shared_ctx(), &surface) : agc_integ_ptoacsm(agc_integ_shared_ctx());
    agc_status_t sw      = agc_integ_intwake();

    if (st != AGC_STATUS_OK || sp != AGC_STATUS_OK || sw != AGC_STATUS_OK)
    {
        return ((st != AGC_STATUS_OK) ? st : (sp != AGC_STATUS_OK) ? sp : sw);
    }

    return (agc_extverb_gopin()); /* TCPIN: RTB PINBRNCH     */
}

/* ---- jobs whose code differs by vehicle ---------------------------------- */

agc_status_t agc_job_attitude_error_load(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_attitude_error_load_lm(ctx) : agc_job_attitude_error_load_csm(ctx));
}

agc_status_t agc_job_tracking_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_tracking_terminate_lm(ctx) : agc_job_tracking_terminate_csm(ctx));
}

agc_status_t agc_job_dap_data_load_noun46(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_noun46_lm(ctx) : agc_job_dap_data_load_noun46_csm(ctx));
}

agc_status_t agc_job_dap_data_load_config(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_config_lm(ctx) : agc_job_dap_data_load_config_csm(ctx));
}

agc_status_t agc_job_dap_data_load_noun47(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_noun47_lm(ctx) : agc_job_dap_data_load_noun47_csm(ctx));
}

agc_status_t agc_job_dap_data_load_mass(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_mass_lm(ctx) : agc_job_dap_data_load_mass_csm(ctx));
}

agc_status_t agc_job_dap_data_load_noun48(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_noun48_lm(ctx) : agc_job_dap_data_load_noun48_csm(ctx));
}
