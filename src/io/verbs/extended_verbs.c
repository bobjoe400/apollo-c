/**********************************************************************
 * FILE NAME: extended_verbs.c
 *
 * PURPOSE:
 *   The extended verbs both vehicles share (GOEXTVB, the LST2FAN entries
 *   that are the same in both ropes) and the common exits TESTXACT,
 *   ALM/END, GOPIN, CHKPOOH, CKMODCAD; the per-vehicle verbs are in
 *   extended_verbs_csm.c, extended_verbs_lm.c and extended_verbs_lm_radar.c,
 *   V70-V73 in uplink_update.c.
 *
 * AGC SOURCE:
 *   Comanche055 EXTENDED_VERBS.agc pp.235-262
 *   Luminary099 EXTENDED_VERBS.agc pp.261-296
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors)
 *   Name        Type      I/O  Scale     Description
 *   ----------  --------  ---  --------  ---------------------------------
 *   EXTVBACT    uint16_t  I/O  bits      extended verb activity (owned here)
 *   FLAGWRD4    uint16_t  I    bits      priority displays (OC24100)
 *   MODREG      agc_sp_t  I    unscaled  P00 test (CHKPOOH)
 *   MODECADR    agc_sp_t  I    -         IMU mode switching in use
 *   THETAD      uint16_t  I    CDU       V41 coarse align angles
 *   DSPTEM2     uint16_t  I    cs        V55 clock increment
 *   TIME2,TIME1 uint32_t  I/O  cs 2^28   V55 clock
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
 *   agc_dsky_*             NVSUB, GOXDSPF, EXDSPRET, ENDEXT, PINBRNCH
 *   agc_imu_*              IMUZERO, IMUCOARS, IMUFINE, IMUPULSE, IMUSTALL
 *   agc_exec_*             FINDVAC / NOVAC, PRIOCHNG
 *   agc_integ_*            INTSTALL, INTWAKE, PTOACSM / PTOALEM (V66, V47)
 *   agc_selfcheck_*        SHOWSUM (V91)
 *   agc_telem_start_dump   DNEDUMP (V74)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Operator error light (ALM/END) when the extended-verb display is busy,
 *   a priority display holds the DSKY, MODECADR is in use or the mode is
 *   not P00 where the verb demands it.
 *
 * NOTES:
 *   The verb fan LST2FAN for verbs 40-99, TESTXACT, ALM/END, GOPIN, CHKPOOH, CKMODCAD,
 *   OP/INERT, and the verbs that are the same in both ropes:
 *     V40 N20  zero the IMU CDUs (VBZERO / IMUZEROK)
 *     V41 N20  coarse align the IMU from the keyboard (IMUCOARK / ICORK2)
 *     V42      fine align the IMU with gyro torquing (IMUFINEK / FINEK2)
 *     V49      start an automatic attitude maneuver (CREWMANU -> R62DISP)
 *     V50 etc. please-perform verbs (GOLOADLV)
 *     V55      align the AGC clock (ALINTIME / UPDATIME)
 *     V64      S-band antenna angles (VB64 -> SBANDANT)
 *     V67      W-matrix monitor, V69 hardware restart, V74 erasable dump,
 *     V82      orbit parameters (R30), V89 final attitude (R63), V90 out of
 *     plane (R36), V91 show-banksum (SHOWSUM).
 *   V70-V73 are the update program (uplink_update.c).  Everything else is
 *   in extended_verbs_csm.c / extended_verbs_lm.c.
 *
 *   DEVIATION (job): in the AGC GOEXTVB runs on in the CHARIN job.
 *   Pinball calls agc_extverb() without a job context, so GOEXTVB starts
 *   a job (AGC_JOB_EXTENDED_VERB, CHARIN's priority 30) that runs the
 *   fan.  Each place where a verb waits for the operator or a mode
 *   switch (GOXDSPF, EXDSPRET, IMUSTALL) is a separate continuation job,
 *   as docs/TRANSLATION_GUIDE.md section 4 prescribes.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/erasable_map/address_map.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/integration.h"
#include "internal.h"

#define WORD15                    077777u            /* one AGC word */
#define BIT6                      000040u            /* DSALMOUT bit 6: flash */
#define OCT24                     000024u            /* EXTVBACT: bits 3 and 5                     */
#define ADRS1_SKEEP1              001371u            /* ADRS1: ECADR SKEEP1 (both ropes)           */
#define OC24100                   024100u            /* FLAGWRD4: priority displays use the DSKY   */
#define PRIO4                     ((agc_prio_t) 004) /* PRIO4 */
#define PRIO5                     ((agc_prio_t) 005) /* PRIO5 */
#define PRIO7                     ((agc_prio_t) 007) /* PRIO7 */
#define PRIO10                    ((agc_prio_t) 010) /* PRIO10 */
#define PRIO30                    ((agc_prio_t) 030) /* CHRPRIO */
#define NOUN_ICDU                 20                 /* N20 ICDU angles                            */
#define NVSUB_BLANK_BUT_MODE_VERB (-2)               /* CS TWO, TC NVSUB              */
#define TIME_UPDATE_VERB          23                 /* DEC23: load with V23 before the update     */
#define CLOCK_DELTA_WORD          4                  /* DSPTEM2 +1, +2 hold the delta time         */

static uint16_t extvbact;      /* EXTVBACT                           */
static bool     sv_from_other; /* V47 (LMTOCM) rather than V66        */
static int      xv_verb;       /* VERBREG at GOEXTVB                 */
static int      xv_noun;       /* NOUNREG at GOEXTVB                 */

/* ---- EXTVBACT ---------------------------------------------------------- */

/**********************************************************************
 * EXTVBACT (ERASABLE_ASSIGNMENTS; TESTXACT p.237 C / p.264 L) -- the
 * extended verb activity bits, read
 *
 * RETURNS: EXTVBACT, 15 bits
 **********************************************************************/
uint16_t agc_erase_extvbact(void)
{
    return (extvbact);
}

/**********************************************************************
 * AGC: none -- EXTVBACT, written (TESTXACT, ENDEXT, the marking routines, P61)
 *
 * ARGUMENTS:
 *   v  uint16_t  I   new EXTVBACT, 15 bits
 **********************************************************************/
void agc_erase_set_extvbact(uint16_t v)
{
    extvbact = v; /* callers store 15-bit masks */
}

/**********************************************************************
 * AGC: none -- EXTVBACT (TESTXACT CCS EXTVBACT) -- an extended verb or marking holds
 * the DSKY
 *
 * RETURNS: true when EXTVBACT is nonzero
 **********************************************************************/
bool agc_extverb_busy(void)
{
    return (extvbact != 0u);
}

/**********************************************************************
 * AGC: none -- NOUNREG at GOEXTVB (OP/INERT, RRDESNBK read it)
 *
 * RETURNS: the noun, 0..99
 **********************************************************************/
int agc_extverb_noun(void)
{
    return (xv_noun);
}

/* ---- common exits ------------------------------------------------------ */

/**********************************************************************
 * AGC: none -- GOPIN: TC POSTJUMP, CADR PINBRNCH.
 *
 * RETURNS: the PINBRNCH status
 **********************************************************************/
agc_status_t agc_extverb_gopin(void)
{
    return (agc_dsky_pinbrnch());
}

/**********************************************************************
 * AGC: none -- ALM/END: operator error light, then GOPIN.
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
agc_status_t agc_extverb_alm_end(void)
{
    agc_dsky_falton();

    return (agc_extverb_gopin());
}

/**********************************************************************
 * TESTXACT (C p.237 / L p.264): refuse (ALM/END) if an extended verb or a
 * priority display holds the DSKY; otherwise mark the extended-verb display
 * system busy (EXTVBACT bits 3, 5) and blank all but MODE and VERB.
 *
 * ARGUMENTS:
 *   ok  bool *  O   true when the extended-verb display is ours (go on)
 *
 * RETURNS: AGC_STATUS_OK, the ALM/END status when refused, or the NVSUB
 *          failure
 **********************************************************************/
agc_status_t agc_extverb_testxact(bool *ok)
{
    *ok = false;

    if (extvbact != 0u || (agc_flagword_get(4) & OC24100) != 0u)
    {
        return (agc_extverb_alm_end());
    }

    extvbact        = OCT24;                                           /* SETXTACT        */
    agc_status_t st = agc_dsky_nvsub_blank(NVSUB_BLANK_BUT_MODE_VERB); /* NVSUB status (busy is fine) */

    if (st != AGC_STATUS_OK && st != AGC_STATUS_BUSY)
    {
        return (st); /* NVSUB busy (TC +1) goes on too */
    }

    *ok = true;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- CHKPOOH: *p00 = the major mode is P00 (MODREG +-0); otherwise ALM/END,
 * whose status is returned.
 *
 * ARGUMENTS:
 *   p00  bool *  O   true in P00
 *
 * RETURNS: AGC_STATUS_OK, or the ALM/END status
 **********************************************************************/
agc_status_t agc_extverb_chkpooh(bool *p00)
{
    *p00 = (agc_erase_modreg() == 0);

    return (*p00 ? AGC_STATUS_OK : agc_extverb_alm_end()); /* not P00: ALM/END */
}

/**********************************************************************
 * AGC: none -- CKMODCAD: *free = nobody waits on IMU mode switching (MODECADR = 0);
 * otherwise ALM/END, whose status is returned.
 *
 * ARGUMENTS:
 *   free  bool *  O   true when MODECADR is free
 *
 * RETURNS: AGC_STATUS_OK, or the ALM/END status
 **********************************************************************/
static agc_status_t agc_extverb_ckmodcad(bool *free)
{
    *free = (agc_erase_modecadr(0) == 0);

    return (*free ? AGC_STATUS_OK : agc_extverb_alm_end()); /* in use: ALM/END */
}

/**********************************************************************
 * AGC: none -- GOLOADLV (V50 and the other please-perform verbs): FLASHOFF, then the
 * same recall as a completed load (LOADLV1).
 *
 * RETURNS: the LOADLV1 status
 **********************************************************************/
agc_status_t agc_extverb_goloadlv(void)
{
    agc_chan_and((int) AGC_CHAN_DSALMOUT, (uint16_t) (WORD15 & ~BIT6));

    return (agc_dsky_loadlv());
}

/**********************************************************************
 * AGC: none -- TC UPFLAG / DOWNFLAG, ADRES f, TC GOPIN.
 *
 * ARGUMENTS:
 *   f  agc_flag_id_t  I   the flag
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
agc_status_t agc_extverb_upflag(agc_flag_id_t f)
{
    agc_flag_set(f); /* UPFLAG (the flagword it leaves in A is not used) */

    return (agc_extverb_gopin());
}

/**********************************************************************
 * TC DOWNFLAG, ADRES f, TC GOPIN (EXTENDED_VERBS.agc, e.g. V58, V61)
 *
 * ARGUMENTS:
 *   f  agc_flag_id_t  I   the flag
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
agc_status_t agc_extverb_downflag(agc_flag_id_t f)
{
    agc_flag_clear(f); /* DOWNFLAG                */

    return (agc_extverb_gopin());
}

/**********************************************************************
 * AGC: none -- CAF PRIOn, TC NOVAC/FINDVAC, 2CADR job, then ENDOFJOB (return).
 *
 * ARGUMENTS:
 *   prio  agc_prio_t    I   the job's priority
 *   job   agc_job_id_t  I   the job (2CADR)
 *   vac   bool          I   true FINDVAC, false NOVAC
 *
 * RETURNS: the FINDVAC / NOVAC status (BAILOUT 1201 / 1202 when full)
 **********************************************************************/
agc_status_t agc_extverb_schedule(agc_prio_t prio, agc_job_id_t job, bool vac)
{
    agc_inhint();
    agc_status_t st = vac ? agc_exec_findvac(prio, job) : agc_exec_novac(prio, job); /* FINDVAC / NOVAC status */

    agc_relint();

    return (st);
}

/**********************************************************************
 * AGC: none -- GOXDSPF (= GOMARKF): flash VERB/NOUN; V34 ends the verb (TERMEXTV).
 *
 * ARGUMENTS:
 *   ctx         const agc_job_ctx_t *  I   the running job
 *   verb        uint8_t                I   VERB to flash
 *   noun        uint8_t                I   NOUN to flash
 *   on_proceed  agc_job_id_t           I   job for V33
 *   on_enter    agc_job_id_t           I   job for ENTER (data loaded)
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_extverb_flash(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t on_proceed, agc_job_id_t on_enter)
{
    agc_dsky_cont_t cont = {AGC_JOB_EXTENDED_VERB_END, on_proceed, on_enter};

    return (agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(verb, noun), &cont));
}

/**********************************************************************
 * AGC: none -- EXDSPRET: show our own verb again (VN vv00) and continue at next.
 *
 * ARGUMENTS:
 *   ctx   const agc_job_ctx_t *  I   the running job
 *   verb  uint8_t                I   our VERB (VN vv00)
 *   next  agc_job_id_t           I   the job that goes on
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_extverb_redisplay(const agc_job_ctx_t *ctx, uint8_t verb, agc_job_id_t next)
{
    agc_dsky_cont_t cont = {next, next, next};

    return (agc_dsky_display(ctx, AGC_DSKY_EXDSPRET, agc_dsky_vn(verb, 0), &cont));
}

/**********************************************************************
 * AGC: none -- IMUSTALL with the continuation wake: GOOD or BAD run it now.
 *
 * ARGUMENTS:
 *   wake  agc_job_id_t  I   GOPIN or ENDEXT job, run when the stall ends
 *
 * RETURNS: the IMUSTALL status, or that of GOPIN / ENDEXT when it was
 *          already over
 **********************************************************************/
static agc_status_t stall_then(agc_job_id_t wake)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;    /* STALL return */
    agc_status_t           st  = agc_imu_stall(wake, &res); /* IMUSTALL status */

    if (st != AGC_STATUS_OK)
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

/**********************************************************************
 * AGC: none -- GOEXTVB (PINBALL VERBFAN): verb 40-99 keyed; the display system has been
 * released.  Starts the extended-verb job (see DEVIATION above).
 *
 * ARGUMENTS:
 *   verb  int  I   40..99
 *   noun  int  I   0..99
 *
 * RETURNS: the NOVAC status of the extended-verb job, BAD_ARG for a verb
 *          or noun out of range
 **********************************************************************/
agc_status_t agc_extverb(int verb, int noun)
{
    xv_verb = verb;
    xv_noun = noun;

    return (agc_extverb_schedule(PRIO30, AGC_JOB_EXTENDED_VERB, false));
}

/**********************************************************************
 * GOTOV56 (P20-P25.agc C p.631 / L p.593): P20 terminates through verb
 * 56 (TRACKTRM) instead of GOTOPOOH (DCA VB56CADR, TCF SUPDXCHZ: a jump
 * into the extended verb). DEVIATION: the AGC continues the calling job
 * at TRACKTRM; here the extended verb is started as its own job and the
 * calling job ends.
 * AGC: GOTOV56 (P20-P25.agc C p.631 / L p.593)
 *
 * FUNCTION NAME: agc_extverb_goto_tracking_terminate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the extended verb request;
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_extverb_goto_tracking_terminate(void)
{
    agc_status_t st = agc_extverb(56, 0); /* V56 request */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * VBZERO (V40, C p.238 / L p.265), IMU part: zero the ICDUs, then GOPIN.
 *
 * RETURNS: the CKMODCAD, IMUZERO or stall status
 **********************************************************************/
static agc_status_t verb_zero_imu(void)
{
    bool         xv_ok = false;                        /* CKMODCAD passed */
    agc_status_t xv_st = agc_extverb_ckmodcad(&xv_ok); /* ALM/END outcome otherwise */

    if (!xv_ok)
    {
        return (xv_st);
    }

    agc_status_t st = agc_imu_zero(); /* IMUZERO         */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (stall_then(AGC_JOB_EXTENDED_VERB_GOPIN)); /* both returns GOPIN */
}

/**********************************************************************
 * AGC: none -- IMUCOARK (V41 N20): load THETAD by V25N22, then ICORK2.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the extended-verb job
 *
 * RETURNS: the CKMODCAD, TESTXACT or display status
 **********************************************************************/
static agc_status_t verb_coarse_imu(agc_job_ctx_t *ctx)
{
    bool ok = false; /* TESTXACT got the display */

    bool         xv_ok = false;                        /* CKMODCAD passed */
    agc_status_t xv_st = agc_extverb_ckmodcad(&xv_ok); /* ALM/END outcome otherwise */

    if (!xv_ok)
    {
        return (xv_st);
    }

    agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT status */

    if (!ok)
    {
        return (st);
    }

    return (agc_extverb_flash(ctx, 25, 22, AGC_JOB_KEYBOARD_COARSE_ALIGN, AGC_JOB_KEYBOARD_COARSE_ALIGN));
}

/**********************************************************************
 * AGC: none -- IMUFINEK (V42): load gyro torquing angles by V25N93.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the extended-verb job
 *
 * RETURNS: the CKMODCAD, TESTXACT or display status
 **********************************************************************/
static agc_status_t verb_fine_align(agc_job_ctx_t *ctx)
{
    bool ok = false; /* TESTXACT got the display */

    bool         xv_ok = false;                        /* CKMODCAD passed */
    agc_status_t xv_st = agc_extverb_ckmodcad(&xv_ok); /* ALM/END outcome otherwise */

    if (!xv_ok)
    {
        return (xv_st);
    }

    agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT status */

    if (!ok)
    {
        return (st);
    }

    return (agc_extverb_flash(ctx, 25, 93, AGC_JOB_KEYBOARD_FINE_ALIGN, AGC_JOB_KEYBOARD_FINE_ALIGN));
}

/**********************************************************************
 * AGC: none -- TESTXACT (and CHKPOOH when pooh), then schedule job at prio.
 *
 * ARGUMENTS:
 *   pooh  bool          I   true: CHKPOOH first
 *   prio  agc_prio_t    I   the job's priority
 *   job   agc_job_id_t  I   the job FINDVAC starts
 *
 * RETURNS: the CHKPOOH, TESTXACT or FINDVAC status
 **********************************************************************/
static agc_status_t xact_and_schedule(bool pooh, agc_prio_t prio, agc_job_id_t job)
{
    bool ok = false; /* TESTXACT got the display */

    bool         xv_ok = true;          /* CHKPOOH passed (or not asked) */
    agc_status_t xv_st = AGC_STATUS_OK; /* ALM/END outcome otherwise */

    if (pooh)
    {
        xv_st = agc_extverb_chkpooh(&xv_ok);
    }

    if (!xv_ok)
    {
        return (xv_st);
    }

    agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT status */

    if (!ok)
    {
        return (st);
    }

    return (agc_extverb_schedule(prio, job, true)); /* FINDVAC         */
}

/**********************************************************************
 * AGC: none -- V74 DNEDUMP: replace the downlist by the erasable dump.
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
static agc_status_t verb_erasable_dump(void)
{
    agc_telem_start_dump();

    return (agc_extverb_gopin());
}

/**********************************************************************
 * SDISPLAY / NOKILL (EXTENDED_VERBS.agc C p.258, L p.288): CA ADRS1 /
 * TS MPAC +2, then GOXDSPF V05N01: the machine-address noun shows the
 * three words from SKEEP1 (sum, bank, bugger word). V34 ends, V33 sums
 * the next bank, ENTER shows it again. DEVIATION: SKEEP1-3 live in the
 * self-check's storage; they are copied to the erasable image at their
 * address (1371-1373 in both ropes) for the noun to read.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
static agc_status_t banksum_display(const agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_SHOW_BANKSUM_END, AGC_JOB_SHOW_BANKSUM_NEXT, AGC_JOB_SHOW_BANKSUM_DISPLAY};
    uint16_t        w[3];                 /* SKEEP1..3 */
    agc_status_t    st   = AGC_STATUS_OK; /* image store status */

    agc_selfcheck_showsum_words(w);

    for (int i = 0; i < 3 && st == AGC_STATUS_OK; i++)
    {
        st = agc_erase_write_ecadr((uint16_t) (ADRS1_SKEEP1 + (uint16_t) i), w[i]);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_display_ecadr(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(5, 1), &cont, ADRS1_SKEEP1)); /* MPAC+2 = ADRS1 */
}

/**********************************************************************
 * SHOWSUM (V91, C p.258 / L p.288): P00 only; turn self-check off and sum
 * the first bank (LM: at priority 7 so other CHARINs can run).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the extended-verb job
 *
 * RETURNS: the CHKPOOH, TESTXACT or display status
 **********************************************************************/
static agc_status_t verb_show_banksum(agc_job_ctx_t *ctx)
{
    bool ok = false; /* TESTXACT got the display */

    bool         xv_ok = false;                       /* CHKPOOH passed */
    agc_status_t xv_st = agc_extverb_chkpooh(&xv_ok); /* ALM/END outcome otherwise */

    if (!xv_ok)
    {
        return (xv_st);
    }

    agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT status */

    if (!ok)
    {
        return (st);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_exec_prio_change(PRIO7);
    }

    agc_selfcheck_showsum_start(); /* SKEEP6 = +1, SMODE = 0 */

    return (banksum_display(ctx));
}

/**********************************************************************
 * AGC: none -- The shared verbs; NOT_FOUND for the vehicle fans.
 *
 * ARGUMENTS:
 *   ctx   agc_job_ctx_t *  I   the extended-verb job
 *   verb  int              I   40..99
 *
 * RETURNS: the verb's status, NOT_FOUND when the vehicle's fan handles it
 **********************************************************************/
static agc_status_t fan_shared(agc_job_ctx_t *ctx, int verb)
{
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM); /* LM build */

    switch (verb)
    {
        case 40:
            return ((xv_noun == NOUN_ICDU) ? verb_zero_imu() : AGC_STATUS_NOT_FOUND);      /* N20 only; other nouns: the vehicle fan */
        case 41:
            return ((xv_noun == NOUN_ICDU) ? verb_coarse_imu(ctx) : AGC_STATUS_NOT_FOUND); /* N20 only; other nouns: the vehicle fan */
        case 42:
            return (verb_fine_align(ctx));
        case 49:
            return (xact_and_schedule(true, PRIO10, AGC_JOB_CREW_DEFINED_MANEUVER_DISPLAY));
        case 50:
        case 97:
        case 99:
            return (agc_extverb_goloadlv());
        case 55:
            /* ALINTIME (V55): load the clock increment by V25N24 (LM: as R33 at
             * priority 7).                                                            */
        {
            bool         ok = false;                     /* TESTXACT got the display */
            agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT status */

            if (!ok)
            {
                return (st);
            }

            if (agc_state_vehicle() == AGC_VEHICLE_LM)
            {
                agc_exec_prio_change(PRIO7);
            }

            return (agc_extverb_flash(ctx, 25, 24, AGC_JOB_EXTENDED_VERB_END, AGC_JOB_CLOCK_ALIGN_UPDATE));
        }
        case 64:
            return (xact_and_schedule(true, PRIO4, lm ? AGC_JOB_SBANDANT_LM : AGC_JOB_SBANDANT_CM)); /* SBANDANT of the vehicle */
        case 66:
            return (agc_extverb_state_vector_copy(false));
        case 67:
            return (xact_and_schedule(false, PRIO5, AGC_JOB_W_MATRIX_MONITOR));
        case 69:
            /* VERB69: force a hardware restart (the AGC does TC VERB69, a TC trap).  */
            return (agc_rupt_go());
        case 70:
        case 71:
        case 72:
        case 73:
            return (agc_uplink_update_start(ctx, verb));
        case 74:
            return (verb_erasable_dump());
        case 82:
            /* V82PERF: R30 runs on in this job at priority 7 (V82CALL must not be a
             * FINDVAC job).                                                           */
        {
            bool         ok = false;                     /* TESTXACT got the display */
            agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT status */

            if (!ok)
            {
                return (st);
            }

            agc_exec_prio_change(PRIO7);

            return (agc_exec_jump(ctx, AGC_JOB_ORBIT_PARAMETER_DISPLAY)); /* POSTJUMP V82CALL */
        }
        case 89:
            return (xact_and_schedule(true, PRIO10, AGC_JOB_RENDEZVOUS_FINAL_ATTITUDE));
        case 90:
            return (xact_and_schedule(false, PRIO7, AGC_JOB_OUT_OF_PLANE_DISPLAY));
        case 91:
            return (verb_show_banksum(ctx));
        default:
            break;
    } /* switch (verb) */

    return (AGC_STATUS_NOT_FOUND);
} /* fan_shared */

/**********************************************************************
 * LST2FAN (C p.236 / L p.262), run as the extended-verb job.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the extended-verb job
 *
 * RETURNS: the verb's status
 **********************************************************************/
agc_status_t agc_job_extended_verb(agc_job_ctx_t *ctx)
{
    agc_status_t st = fan_shared(ctx, xv_verb); /* xv_verb 40-99 (agc_extverb) */

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

/**********************************************************************
 * AGC: none -- ENDEXT (TERMEXTV, ENDEXTVB) as a job: release the display systems.
 *
 * RETURNS: the ENDMARK status
 **********************************************************************/
agc_status_t agc_job_extended_verb_end(void)
{
    return (agc_dsky_endmark());
}

/**********************************************************************
 * AGC: none -- GOPIN as a job (after an IMUSTALL in V40).
 *
 * RETURNS: the GOPIN status
 **********************************************************************/
agc_status_t agc_job_extended_verb_gopin(void)
{
    return (agc_extverb_gopin());
}

/**********************************************************************
 * AGC: none -- ICORK2 (V41): show V41 again, then coarse align.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_keyboard_coarse_align(agc_job_ctx_t *ctx)
{
    return (agc_extverb_redisplay(ctx, 41, AGC_JOB_KEYBOARD_COARSE_DRIVE));
}

/**********************************************************************
 * AGC: none -- ICORK2 +3: IMUCOARS to THETAD, IMUSTALL, ENDEXT either way.
 *
 * RETURNS: the IMUCOARS status, or that of the stall
 **********************************************************************/
agc_status_t agc_job_keyboard_coarse_drive(void)
{
    uint16_t thetad[3]; /* THETAD, CDU patterns */

    for (int i = 0; i < 3; i++)
    {
        thetad[i] = agc_erase_thetad(i);
    }

    agc_status_t st = agc_imu_coarse(thetad); /* IMUCOARS status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (stall_then(AGC_JOB_EXTENDED_VERB_END));
}

/**********************************************************************
 * AGC: none -- IMUFINEK after the load: show V42 again.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_keyboard_fine_align(agc_job_ctx_t *ctx)
{
    return (agc_extverb_redisplay(ctx, 42, AGC_JOB_KEYBOARD_FINE_MODE));
}

/**********************************************************************
 * AGC: none -- IMUFINE, IMUSTALL: bad -> ENDEXT, good -> FINEK2.
 *
 * RETURNS: the IMUFINE or stall status, or that of FINEK2
 **********************************************************************/
agc_status_t agc_job_keyboard_fine_mode(void)
{
    agc_status_t st = agc_imu_fine(); /* IMUFINE / stall status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING; /* STALL return */

    st = agc_imu_stall(AGC_JOB_KEYBOARD_GYRO_TORQUE, &res);

    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return (st);
    }

    return (agc_job_keyboard_gyro_torque());
}

/**********************************************************************
 * AGC: none -- FINEK2: torque the gyros by the angles pinball left in OGC, IGC, MGC;
 * IMUSTALL; ENDEXT.  Entered after the fine-align stall (bad -> ENDEXT).
 *
 * RETURNS: AGC_STATUS_OK (gyros busy: woken again), the ENDEXT status, or
 *          the stall's
 **********************************************************************/
agc_status_t agc_job_keyboard_gyro_torque(void)
{
    if (agc_imu_stall_outcome(AGC_IMU_DEV_IMU) == AGC_IMU_STALL_BAD)
    {
        return (agc_dsky_endmark());
    }

    agc_status_t st = agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_KEYBOARD_GYRO_TORQUE); /* IMUPULSE status */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* gyros busy: woken again */
    }

    if (st != AGC_STATUS_OK)
    {
        return (agc_dsky_endmark());
    }

    return (stall_then(AGC_JOB_EXTENDED_VERB_END));
}

/**********************************************************************
 * AGC: none -- Signed value of a 15-bit ones'-complement word.
 *
 * ARGUMENTS:
 *   w  uint16_t  I   a 15-bit ones' complement word
 *
 * RETURNS: its value
 **********************************************************************/
static int32_t ones_value(uint16_t w)
{
    return (((w & 040000u) != 0u) ? -(int32_t) (WORD15 - w) : (int32_t) w); /* negative: -(77777 - w) */
}

/**********************************************************************
 * AGC: none -- UPDATIME (V55): only after a V23 load, add the increment in DSPTEM2 +1,
 * +2 (cs, DP) to TIME2,TIME1 (TPAGREE gives sign agreement; the counter
 * wraps at 2^28 like TIME2).
 *
 * RETURNS: the ENDEXT status
 **********************************************************************/
agc_status_t agc_job_clock_align_update(void)
{
    uint8_t verb = 0; /* the response verb (MPAC) */
    uint8_t noun = 0; /* the response noun */

    agc_dsky_response(&verb, &noun);

    if (verb != TIME_UPDATE_VERB)
    {
        return (agc_dsky_endmark()); /* not a V23 data load     */
    }

    int32_t delta = ones_value(agc_dsky_dsptem(CLOCK_DELTA_WORD)) * 040000 + ones_value(agc_dsky_dsptem(CLOCK_DELTA_WORD + 1)); /* DSPTEM2 +1, +2: increment, cs */

    agc_inhint();
    int64_t t = (int64_t) agc_time_now_cs() + (int64_t) delta; /* new clock, cs */

    t = ((t % (int64_t) AGC_TIME_MODULUS_CS) + (int64_t) AGC_TIME_MODULUS_CS) % (int64_t) AGC_TIME_MODULUS_CS;
    agc_time_set_cs((uint32_t) t);
    agc_relint();

    return (agc_dsky_endmark()); /* UPDTMEND                */
}

/**********************************************************************
 * AGC: none -- NOKILL (ENTER on the V05N01): show the same bank again.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_show_banksum_display(agc_job_ctx_t *ctx)
{
    return (banksum_display(ctx));
}

/**********************************************************************
 * AGC: none -- NXTBNK (V33): sum the next bank (after the last: bank 0 again, ENDSUMS
 * with SKEEP6 = +1 -> STSHOSUM) and show it.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t agc_job_show_banksum_next(agc_job_ctx_t *ctx)
{
    agc_selfcheck_showsum_next();

    return (banksum_display(ctx));
}

/**********************************************************************
 * AGC: none -- V34: SKEEP1 = SELFADRS (the self-check restarts from SELFCHK, as
 * SHOWSUM already arranged), ENDEXT.
 *
 * RETURNS: the ENDEXT status
 **********************************************************************/
agc_status_t agc_job_show_banksum_end(void)
{
    return (agc_dsky_endmark());
}

/**********************************************************************
 * AGC: none -- ATTACHED (V66) / LMTOCMSV (CSM V47): FINDVAC the copy at priority 10.
 *
 * ARGUMENTS:
 *   from_other  bool  I   true: V47 LMTOCM (CSM), false: V66 ATTACHED
 *
 * RETURNS: the FINDVAC status
 **********************************************************************/
agc_status_t agc_extverb_state_vector_copy(bool from_other)
{
    sv_from_other = from_other;

    return (agc_extverb_schedule(PRIO10, AGC_JOB_STATE_VECTOR_COPY, true));
}

/**********************************************************************
 * AGC: none -- ATTACHIT / LMTOCM: with INTSTALL held, copy the 42-word permanent state
 * of one vehicle into the other's (GENTRAN OCT51 words) with its moon
 * flag (MOONTHIS / MOONOTH), refresh the temporary set (PTOACSM, PTOALEM),
 * INTWAKE, PINBRNCH.
 * DEVIATION: SVDWN1/SVDWN2 (downlink copies) are not called; the down-
 * telemetry reads the state vectors directly (down_telemetry.c).
 *
 * RETURNS: AGC_STATUS_OK (asleep in INTSTALL), the first failure of
 *          INTSTALL, the copy, PTOA* or INTWAKE, else the GOPIN status
 **********************************************************************/
agc_status_t agc_job_state_vector_copy(void)
{
    agc_status_t st = agc_integ_install(0, AGC_JOB_STATE_VECTOR_COPY); /* INTSTALL / copy status */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* sleeps in INTSTALL      */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool           lm    = (agc_state_vehicle() == AGC_VEHICLE_LM); /* LM build */
    agc_vehicle_t  self  = lm ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM;   /* this vehicle */
    agc_vehicle_t  other = lm ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM;   /* the other vehicle */
    agc_vehicle_t  from  = sv_from_other ? other : self;            /* the vehicle copied from */
    agc_integ_sv_t sv;                                              /* the permanent state vector copied */

    agc_integ_perm_get(from, &sv);
    agc_integ_perm_set(sv_from_other ? self : other, &sv); /* LMTOCM: into this vehicle */

    agc_flag_id_t mself = (agc_flag_id_t) (lm ? AGC_FLAG_LMOONFLG : AGC_FLAG_CMOONFLG); /* MOONTHIS */
    agc_flag_id_t moth  = (agc_flag_id_t) (lm ? AGC_FLAG_CMOONFLG : AGC_FLAG_LMOONFLG); /* MOONOTH */
    bool          moon  = agc_flag_test(sv_from_other ? moth : mself);                  /* the source's moon flag */
    agc_flag_id_t dst   = sv_from_other ? mself : moth;                                 /* the flag that takes it */

    agc_flag_put(dst, moon);
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    bool         surface = false;         /* PTOALEM's surface answer (unused) */
    agc_status_t sp      = AGC_STATUS_OK; /* PTOALEM status */

    if (lm)
    {
        sp = agc_integ_ptoalem(&ip, agc_integ_shared_ctx(), &surface);
    }
    else
    {
        agc_integ_ptoacsm(agc_integ_shared_ctx());
    }

    agc_status_t sw = agc_integ_intwake(); /* INTWAKE status */

    /* INTWAKE runs whatever happened before it (the integration must be
     * released); the first failure is the one reported.                    */
    if (st == AGC_STATUS_OK)
    {
        st = sp;
    }

    if (st == AGC_STATUS_OK)
    {
        st = sw;
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_extverb_gopin()); /* TCPIN: RTB PINBRNCH     */
} /* agc_job_state_vector_copy */

/* ---- jobs whose code differs by vehicle ---------------------------------- */

/**********************************************************************
 * AGC: none -- V43 after the V25N22 load, by vehicle (CSM NEEDLE11 etc.,
 * LM EXDSPRET)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the vehicle routine's status
 **********************************************************************/
agc_status_t agc_job_attitude_error_load(agc_job_ctx_t *ctx)
{
    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_attitude_error_load_lm(ctx) : agc_job_attitude_error_load_csm()); /* LM : CSM */
}

/**********************************************************************
 * AGC: none -- V56 after INTSTALL, by vehicle (CSM TRACKTRM, LM TRMTRAK1)
 *
 * RETURNS: the vehicle routine's status
 **********************************************************************/
agc_status_t agc_job_tracking_terminate(void)
{
    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_tracking_terminate_lm() : agc_job_tracking_terminate_csm()); /* LM : CSM */
}

/**********************************************************************
 * AGC: none -- R03 (V48) noun 46 display, by vehicle (DAPDISP / DAPDATA1)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the vehicle routine's status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_noun46(agc_job_ctx_t *ctx)
{
    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_noun46_lm(ctx) : agc_job_dap_data_load_noun46_csm(ctx)); /* LM : CSM */
}

/**********************************************************************
 * AGC: none -- R03 noun 46 proceed, by vehicle (CSM S41.2 / LM DPDAT1)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the vehicle routine's status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_config(agc_job_ctx_t *ctx)
{
    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_config_lm(ctx) : agc_job_dap_data_load_config_csm(ctx)); /* LM : CSM */
}

/**********************************************************************
 * AGC: none -- R03 noun 47 display, by vehicle (DONOUN47 / DAPDATA2)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the vehicle routine's status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_noun47(agc_job_ctx_t *ctx)
{
    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_noun47_lm(ctx) : agc_job_dap_data_load_noun47_csm(ctx)); /* LM : CSM */
}

/**********************************************************************
 * AGC: none -- R03 noun 47 proceed, by vehicle (MASSPROP / DAPDAT2)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the vehicle routine's status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_mass(agc_job_ctx_t *ctx)
{
    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_mass_lm(ctx) : agc_job_dap_data_load_mass_csm(ctx)); /* LM : CSM */
}

/**********************************************************************
 * AGC: none -- R03 noun 48 display, by vehicle (DONOUN48 / DAPDATA3)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the vehicle routine's status
 **********************************************************************/
agc_status_t agc_job_dap_data_load_noun48(agc_job_ctx_t *ctx)
{
    return ((agc_state_vehicle() == AGC_VEHICLE_LM) ? agc_job_dap_data_load_noun48_lm(ctx) : agc_job_dap_data_load_noun48_csm(ctx)); /* LM : CSM */
}
