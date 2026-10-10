/*
 * change_program_idle.c -- GOP00FIX, INITSUB, V37 (MMCHANG), CANV37, and the few
 *              calls the restart code makes into higher layers.
 *
 * Source: FRESH_START_AND_RESTART.agc, Comanche055 pp. 194-209,
 * Luminary099 pp. 223-236.
 *
 * LAYERING: this is program-level code that the AGC keeps in the fresh
 * start log section. It calls up into io (pinball), imu and the DAPs, so
 * like dispatch.c it is a documented exception to "exec calls only core".
 * Everything else in src/exec/ reaches higher layers only through the
 * exec_up_* functions here, which unit tests replace.
 *
 * V37 is PARTIAL: the checks that decide whether a new program may be
 * selected are translated (IMU initialising or NODOFLAG -> alarm 1520,
 * the PREMM1 table of selectable programs -> operator error, SERVICER
 * running -> wait for it); the program change itself (CANV37 .. POOH /
 * NOUVEAU / V37XEQ) and Comanche R00TOP00 / Luminary SETUP70 are stubs.
 */
#include "agc/agc_exec.h"
#include "agc/agc_assert.h"
#include "agc/agc_state.h"
#include "agc/agc_channels.h"
#include "agc/agc_flags_common.h"
#include "agc/agc_dsky.h"
#include "agc/agc_imu.h"
#include "exec_internal.h"

#define VP_VERB_CHANGE_PROGRAM        37U
#define VP_NOUN_ENABLE_REQUEST        99U
#define VP_BIT6       000040u     /* IMODES30 bit 6: IMU being initialised */
#define VP_BIT13      010000u     /* DSALMOUT bit 13: engine on            */
#define VP_F6_DAPBITS 060000u
#define VP_F6_TVC     040000u
#define VP_RAKE_WORDS 12          /* FLAGTABL: FLAGWRD0 .. FLAGWRD11       */
#define VP_LM_F13_CLR 000440u     /* LM FLAGWRD13: XOVINFLG bit 9, ULLAGFLG bit 6 */
#define VP_DPS_ABORT        70U
#define VP_APS_ABORT        71U

/* PREMM1 (C p.207 / L p.234): major modes that V37 can select, highest
 * first, with the priority (PREMM1 bits 15-11) the program starts at.
 * The FCADRMM1 start addresses are programs of other lanes and are not
 * listed (CANV37 is a stub).                                           */
typedef struct vp_premm { uint8_t mm; uint8_t prio; } vp_premm_t;

static const vp_premm_t premm_csm[] = {
    {79, 013}, {78, 013}, {77, 013}, {76, 013}, {75, 013}, {74, 013},
    {73, 013}, {72, 013}, {62, 013}, {61, 013}, {54, 013}, {53, 013},
    {52, 013}, {51, 013}, {47, 013}, {41, 013}, {40, 013}, {39, 013},
    {38, 013}, {37, 013}, {35, 013}, {34, 013}, {33, 013}, {32, 013},
    {31, 013}, {30, 013}, {23, 013}, {22, 013}, {21, 013}, {20, 013},
    {17, 013}, {6, 013}, {1, 020} };
static const vp_premm_t premm_lm[] = {
    {79, 013}, {78, 013}, {76, 013}, {75, 013}, {74, 013}, {73, 013},
    {72, 013}, {68, 013}, {63, 013}, {57, 013}, {52, 013}, {51, 013},
    {47, 013}, {42, 013}, {41, 013}, {40, 013}, {39, 013}, {38, 013},
    {35, 013}, {34, 013}, {33, 013}, {32, 013}, {31, 013}, {30, 013},
    {25, 013}, {22, 013}, {21, 013}, {20, 013}, {12, 013}, {6, 013} };

static uint8_t mmnumber;          /* MMNUMBER: the requested major mode    */

/* ---- up-calls used by fresh_start.c and alarm.c ---------------------- */
agc_status_t exec_up_stoprate(void)
{
    (void)AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM);
    (void)AGC_ASSERT(agc_interrupts_inhibited() || mmnumber <= 99U);
    return agc_kalc_stoprate();
}

/* SERVIDLE (SERVICER, guid-powered): not yet published.               */
agc_status_t exec_up_servidle(void)
{
    (void)AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM);
    (void)AGC_ASSERT(mmnumber <= 99U);
    return AGC_STATUS_UNIMPLEMENTED;
}

/* SETCOARS (IMU mode switching, imu): not yet published.              */
agc_status_t exec_up_setcoars(void)
{
    (void)AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM);
    (void)AGC_ASSERT(mmnumber <= 99U);
    return AGC_STATUS_UNIMPLEMENTED;
}

/* INITSUB / RAKE (C p.203): STOPRATE, restore the deadband, clear the
 * FLAGTABL bits of FLAGWRD0..11, set IMPULSW.
 * PARTIAL: SETMAXDB / SETMINDB (CSM RCS DAP) are not published yet;
 * OPTIND = -1 is delegated to the optics owner.                       */
static void initsub_csm(void)
{
    static const uint16_t flagtabl[VP_RAKE_WORDS] = {
        0, 000040u, 006000u, 0, 0, 004140u, 010000u, 016000u, 0,
        042000u, 0, 0 };
    agc_status_t st = exec_up_stoprate();
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    for (int w = VP_RAKE_WORDS - 1; w >= 0; w--) {     /* RAKE         */
        uint16_t v = (uint16_t)(agc_flagword_get(w) & ~flagtabl[w]);
        st = agc_flagword_set(w, v);
        (void)AGC_ASSERT(st == AGC_STATUS_OK);
    }
    st = agc_flag_set(AGC_FLAG_IMPULSW);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
}

/* The V37 N99 flash of GOP00FIX: every response re-flashes it
 * (TCF -3 / -4 / -5 back to CAF V37N99).                              */
static agc_status_t flash_change_program_enable_request(const agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = { AGC_JOB_GOP00FIX_REFLASH,
                             AGC_JOB_GOP00FIX_REFLASH,
                             AGC_JOB_GOP00FIX_REFLASH };
    if (!AGC_ASSERT(ctx != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_dsky_display(ctx, AGC_DSKY_GOFLASH,
                            agc_dsky_vn((uint8_t)VP_VERB_CHANGE_PROGRAM, (uint8_t)VP_NOUN_ENABLE_REQUEST),
                            &cont);
}

/* GOP00FIX (C p.194 / L p.223): P00 has been requested by a restart
 * with no active group or by POODOO: flash V37 N99 until the crew
 * selects a program.                                                  */
agc_status_t agc_job_go_idle_fix(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_state_vehicle() == AGC_VEHICLE_CSM) {
        initsub_csm();                            /* TC INITSUB        */
    } else {
        uint16_t f13 = (uint16_t)(agc_flagword_get(13) & ~VP_LM_F13_CLR);
        agc_status_t s = agc_flagword_set(13, f13); /* XOVINFLG, ULLAGFLG */
        (void)AGC_ASSERT(s == AGC_STATUS_OK);
    }
    agc_status_t st = agc_dsky_clearmrk();        /* TC CLEARMRK +2    */
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    return flash_change_program_enable_request(ctx);
}

agc_status_t agc_job_go_idle_fix_reflash(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id == AGC_JOB_GOP00FIX_REFLASH);
    return flash_change_program_enable_request(ctx);
}

/* CHECKTAB / AGAINMM (C p.198 / L p.228): is `mm` a selectable mode?  */
static bool premm_has(uint8_t mm)
{
    bool lm = agc_state_vehicle() == AGC_VEHICLE_LM;
    const vp_premm_t *t = lm ? premm_lm : premm_csm;
    int n = lm ? (int)(sizeof(premm_lm) / sizeof(premm_lm[0]))
               : (int)(sizeof(premm_csm) / sizeof(premm_csm[0]));
    bool found = false;
    for (int i = 0; i < n && !found; i++) {
        found = t[i].mm == mm;
    }
    (void)AGC_ASSERT(n > 0);
    (void)AGC_ASSERT(mm <= 99U);
    return found;
}

/* CANTR00 / V37BAD (C p.198 / L p.228): alarm 1520, release the DSKY.
 * V37NONO (FALTON, operator error) is reported as NOT_FOUND for the
 * pinball caller, which owns the OPR ERR lamp.                        */
static agc_status_t change_program_refuse(bool alarm_1520)
{
    agc_status_t result = AGC_STATUS_NOT_FOUND;
    if (alarm_1520) {
        result = agc_alarm(AGC_ALARM_01520);
    }
    agc_status_t st = agc_dsky_releasedsp();       /* TC RELDSP         */
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    st = agc_dsky_pinbrnch();                      /* CADR PINBRNCH     */
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    return result;
}

/* V37 (C p.197 / L p.227): request to change to major mode new_mm.
 * Returns ALARM (1520, not permitted), NOT_FOUND (no such program),
 * BUSY (SERVICER asked to stop; V37 continues when it has) or
 * UNIMPLEMENTED (the program change itself, see file header).         */
agc_status_t agc_change_program(uint8_t new_mm)
{
    if (!AGC_ASSERT(new_mm <= 99U)) {
        return AGC_STATUS_BAD_ARG;
    }
    mmnumber = new_mm;                             /* TS MMNUMBER       */
    bool lm = agc_state_vehicle() == AGC_VEHICLE_LM;
    if ((agc_erase_imodes30() & VP_BIT6) != 0U) {  /* IMU initialising  */
        return change_program_refuse(true);
    }
    if (lm && (mmnumber == VP_DPS_ABORT || mmnumber == VP_APS_ABORT)) {
        return AGC_STATUS_UNIMPLEMENTED;           /* SETUP70 / SETUP71 */
    }
    if (!lm && ((agc_chan_read(AGC_CHAN_DSALMOUT) & VP_BIT13) != 0U ||
                (agc_flagword_get(6) & VP_F6_DAPBITS) == VP_F6_TVC)) {
        return AGC_STATUS_UNIMPLEMENTED;           /* R00TOP00          */
    }
    if (mmnumber != 0U) {
        if (agc_flag_test(AGC_FLAG_NODOFLAG)) {
            return change_program_refuse(true);               /* CANTR00           */
        }
        if (!premm_has(mmnumber)) {
            return change_program_refuse(false);              /* V37NONO           */
        }
    }
    if (agc_flag_test(AGC_FLAG_CHANGE_PROGRAM_FLAG)) {         /* ISSERVON          */
        /* DOWNFLAG AVEGFLAG (vehicle-specific bit, owned by SERVICER):
         * delegated; SERVICER returns to CANV37 when it has stopped.   */
        return AGC_STATUS_BUSY;
    }
    return AGC_STATUS_UNIMPLEMENTED;               /* CANV37            */
}

/* CANV37 (C p.199 / L p.229): STUB. Waits for integration (INTSTALL),
 * clears outbits and flags, then POOH (P00) or NOUVEAU (new program via
 * restart group 4 and GOPROG2).                                       */
agc_status_t agc_job_change_program_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(mmnumber <= 99U);
    return AGC_STATUS_UNIMPLEMENTED;
}
