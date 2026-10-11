/**********************************************************************
 * FILE NAME: programs.c
 *
 * PURPOSE:
 *   The entry programs: P61 (EMS initialisation data: N61 target, N60
 *   GMAX/VPRED/GAMMAEI, N63 RTGO/VIO/TTE), P62 (CM/SM separation, N41
 *   checklist, entry DAP on, entry attitude), P63 (entry initialisation,
 *   waits for .05 g: STARTENT via CM/POSE), the P64-P67 mode and display
 *   changes called from the steering (reentry.c), S61.1 (state vector and
 *   IMU orientation check), S61.2 (the N60/N63 predictions) and S62.3
 *   (the desired gimbal angles for N22).
 *
 * AGC SOURCE:
 *   Comanche055 P61-P67.agc pp. 789-818 (P61 .. P67.3, S61.1, S61.1A-C,
 *   RETRN1-3, S61.2, FISHCALC, PREVGAM/VGAMCALC, TFF/TRIG, DISPTARG,
 *   VRCALC, GMAXCALC, S62.3)
 *   Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:  (through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   RN, VN        agc_vec_t  I    m B29, m/cs B7 SERVICER state
 *   PIPTIME       agc_dp_t   I    cs B28         time of RN, VN
 *   REFSMMAT      agc_mat_t  I    half-unit      stable member matrix
 *   AVEGEXIT      job id     O    -              SERVICER exit
 *   ENTRYDSP, CMDAPARM, NODOFLAG, AVEGFLAG ...   flags (I/O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_display      GOFLASH / GOFLASHR / GOPERF1R / GODSPR
 *   agc_tff_conic, agc_tff_calctff  TFFCONIC / CALCTFF (nav-aux)
 *   agc_latlong_geterad   GETERAD (Fischer radius)
 *   agc_integ_midtoav2    MIDTOAV2 (S61.1)
 *   agc_imu_calcga        CALCGA (S62.3)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarm 01426 (IMU unsatisfactory) or 01427 (IMU reversed) from
 *   S61.1A, then V05N09 and a 10 s DELAYJOB. Display and executive
 *   statuses are passed up.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only. DUMPP61 retries NEWRNVN at most 4 times (the AGC loops
 *   until SERVICER leaves PIPTIME alone).
 *
 * NOTES:
 *   Every point where the AGC job waits for the crew (GOFLASH, GOFLASHR,
 *   GOPERF1R) or sleeps (DELAYJOB, waitlist) is a separate job entry
 *   (docs/TRANSLATION_GUIDE.md section 4). The S61.2 arithmetic uses DMP
 *   (truncating) throughout, as the listing does; DOT SL1 / SL2 keep the
 *   TP MPAC (FISHCALC, VRCALC); VGAMCALC's SL* with a negative count is a
 *   truncating shift (review-guid-entry).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"

/* Verb/noun codes (VN pseudo-op: verb, noun in decimal digits).          */
#define VN_ENTRY_PREDICTION_DISPLAY 6, 60 /* V06N60 GMAX VPRED GAMMAEI */
#define VN_ENTRY_TARGET_DISPLAY     6, 61 /* V06N61 LAT LNG HEADSUP    */

#define ENTRY_OCT41 041                 /* GOPERF1R: CM/SM separation */
#define ENTRY_BIT14 ((agc_sp_t) 020000) /* .5 rev = 180 deg, B0       */

/* P61/P62 module erasables (EBANK6/7).                                   */
static agc_job_id_t imu_check_return_job = AGC_JOB_NONE; /* 60GENRET return point  */
static agc_dp_t     entry_preparation_mm;                /* MM: PIPTIME of RONE/VONE, cs B28    */
static agc_vec_t    urone;                               /* URONE UNIT(RONE)/2, B1 */
static agc_vec_t    urh;                                 /* URH rotated UR/2, B1 */
static agc_tff_t    entry_preparation_tff;               /* TFF push list (RONE, VONE, ...)     */

/**********************************************************************
 * AGC: none (BANKCALL GOFLASH / GOFLASHR / GOPERF1R with their three
 * returns, P61-P67.agc pp. 789-800) -- request an entry display; "R" kinds
 * return at once (the job goes on).
 *
 * FUNCTION NAME: entry_flash
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *   kind      agc_dsky_disp_t  I    display kind (GOFLASH, GOFLASHR, ...)
 *   verb      uint8_t          I    verb, decimal 0..99
 *   noun      uint8_t          I    noun, decimal 0..99
 *   term      agc_job_id_t     I    job for the terminate return (V34)
 *   proceed   agc_job_id_t     I    job for PROCEED (V33)
 *   enter     agc_job_id_t     I    job for ENTER / recycle
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a failed
 *          assertion, else the agc_dsky_display status
 **********************************************************************/
static agc_status_t entry_flash(agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint8_t verb, uint8_t noun, agc_job_id_t term, agc_job_id_t proceed, agc_job_id_t enter)
{
    const agc_dsky_cont_t cont = {term, proceed, enter};

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(verb < 100 && noun < 100))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont));
}

/**********************************************************************
 * AGC: none (TC NEWMODEX, MM n; CA VnnNnn, TS ENTRYVN; P61-P67.agc pp.
 * 795-800) -- the major mode and the entry display ENTRYVN used by
 * OVERNOUT.
 *
 * FUNCTION NAME: entry_mode
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *   mm        agc_sp_t             I    new major mode, 61..67
 *   verb      uint8_t              I    ENTRYVN verb, decimal
 *   noun      uint8_t              I    ENTRYVN noun, decimal
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a failed
 *          assertion, else the NEWMODEX status
 **********************************************************************/
static agc_status_t entry_mode(agc_entry_state_t *es, agc_sp_t mm, uint8_t verb, uint8_t noun)
{
    es->entry_verb = verb;
    es->entry_noun = noun;

    return (agc_phase_newmode(mm));
}

/**********************************************************************
 * AGC: none (CCS HEADSUP / CA BIT14, P61-P67.agc pp. 790, 793) -- ROLLC
 * from HEADSUP: +1 (lift down) 180 deg, -1 (lift up) 0 deg.
 *
 * FUNCTION NAME: headsup_roll
 *
 * ARGUMENTS:
 *   Argument  Type                       I/O  Description (scale)
 *   --------  -------------------------  ---  ---------------------------
 *   es        const agc_entry_state_t *  I    entry erasables
 *                                             (ENTRY_LEXICON names)
 *
 * RETURNS: agc_sp_t -- agc_sp_t -- ROLLC high word: BIT14 (.5 rev = 180
 *          deg) or 0, revs B0
 **********************************************************************/
static agc_sp_t headsup_roll(const agc_entry_state_t *es)
{
    if (!AGC_ASSERT(es->headsup >= -1 && es->headsup <= 1))
    {
        return (0); /* no valid result */
    }

    return ((es->headsup > 0) ? ENTRY_BIT14 : 0); /* lift down: 180 */
}

/* ======================================================================
 * P61 (pp. 789-791)
 * ==================================================================== */

/**********************************************************************
 * P61 (p. 789): lock out extended verbs, HEADSUP = -1, S61.1.
 *
 * FUNCTION NAME: agc_job_entry_preparation
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_preparation(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_svc_set_extvbact(020000);                                    /* CA BIT14 TS EXTVBACT */
    es->headsup          = -1;                                       /* preload              */
    imu_check_return_job = AGC_JOB_ENTRY_PREPARATION_TARGET_DISPLAY; /* QXCH 60GENRET        */

    return (agc_job_entry_imu_check(ctx));
}

/**********************************************************************
 * P61 display (p. 789): V06N61 LAT(SPL) LNG(SPL) HEADSUP, GOFLASHR; the job
 * goes on to P61.3.
 *
 * FUNCTION NAME: agc_job_entry_preparation_target_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_preparation_target_display(agc_job_ctx_t *ctx)
{
    agc_status_t st = entry_flash(ctx, AGC_DSKY_GOFLASHR, VN_ENTRY_TARGET_DISPLAY, AGC_JOB_GOTOPOOH, AGC_JOB_ENTRY_PREPARATION_COMPUTE, AGC_JOB_ENTRY_PREPARATION_TARGET_DISPLAY); /* GOFLASHR status */

    if (!AGC_ASSERT(imu_check_return_job != AGC_JOB_NONE)) /* P61 ran */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_entry_display_protect() : st); /* P61.3 */
}

/**********************************************************************
 * P61.3 (p. 790): restart group 4 at the display (4.1), ENDOFJOB.
 *
 * FUNCTION NAME: agc_job_entry_display_protect
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_display_protect(void)
{
    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(4, 000014);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * NEWRNVN (p. 790): RONE, VONE, URONE, UNI at PIPTIME, and STARTEN1.
 *
 * FUNCTION NAME: entry_preparation_newrnvn
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t entry_preparation_newrnvn(agc_interp_t *ip, agc_entry_state_t *es)
{
    agc_status_t st; /* STARTEN1 status */

    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    entry_preparation_mm = agc_erase_piptime(); /* save PIPTIME in MM   */
    st                   = entry_initial_target(ip, es);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    entry_preparation_tff.rone = agc_erase_rn();
    urone                      = agc_vec_unit(ip, entry_preparation_tff.rone, NULL, NULL);
    entry_preparation_tff.vone = agc_erase_vn();
    es->uni                    = agc_vec_unit(ip, agc_vec_cross(ip, entry_preparation_tff.vone, urone), NULL, NULL);

    if (!AGC_ASSERT(entry_preparation_mm == agc_erase_piptime()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * P61.4 (p. 790): ROLLC from HEADSUP, the target at PIPTIME (retried if
 * SERVICER updated RN, VN meanwhile), S61.2, then P61.1.
 *
 * FUNCTION NAME: agc_job_entry_preparation_compute
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_preparation_compute(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */
    agc_interp_t      *ip = entry_ip();        /* entry job interpreter state */
    agc_status_t       st = AGC_STATUS_OK;     /* NEWRNVN / S61.2 status */
    int                n;                      /* DUMPP61 retry count */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    es->rollc = agc_dp_pack(headsup_roll(es), 0); /* ZL DXCH ROLLC */

    for (n = 0; n < 4; n++)
    { /* DUMPP61 bound  */
        st = entry_preparation_newrnvn(ip, es);

        if (st != AGC_STATUS_OK || agc_dp_sub(ip, entry_preparation_mm, agc_erase_piptime()) >= 0)
        {
            break;
        }
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_entry_ems_data();
    }

    return ((st == AGC_STATUS_OK) ? agc_job_entry_preparation_prediction_display(ctx) : st); /* then the display */
}

/**********************************************************************
 * P61.1 (p. 790): V06N60 GMAX VPRED GAMMAEI, GOFLASH.
 *
 * FUNCTION NAME: agc_job_entry_preparation_prediction_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_preparation_prediction_display(agc_job_ctx_t *ctx)
{
    agc_dsky_clearmrk();

    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_flash(ctx, AGC_DSKY_GOFLASH, VN_ENTRY_PREDICTION_DISPLAY, AGC_JOB_GOTOPOOH, AGC_JOB_ENTRY_PREPARATION_EMS_DISPLAY, AGC_JOB_ENTRY_PREPARATION_PREDICTION_DISPLAY));
}

/**********************************************************************
 * P61.2 (pp. 790-791): TTE corrected for the time spent at the display,
 * V06N63 RTGO VIO TTE, GOFLASH; PROCEED falls into P62.
 *
 * FUNCTION NAME: agc_job_entry_preparation_ems_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_preparation_ems_display(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */
    agc_interp_t      *ip = entry_ip();        /* entry job interpreter state */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    es->tte = agc_dp_add(ip, agc_dp_sub(ip, agc_rtb_loadtime(), entry_preparation_mm), es->tte1);

    return (entry_flash(ctx, AGC_DSKY_GOFLASH, 6, 63, AGC_JOB_GOTOPOOH, AGC_JOB_CM_SM_SEPARATION_NEWMODE, AGC_JOB_ENTRY_PREPARATION_EMS_DISPLAY));
}

/* ======================================================================
 * P62 (pp. 792-794)
 * ==================================================================== */

/**********************************************************************
 * P62 -3 (p. 792): from P61: NEWMODEX 62, DNLSTCOD = 1 (entry list).
 *
 * FUNCTION NAME: agc_job_cm_sm_separation_newmode
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_cm_sm_separation_newmode(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_phase_newmode(62); /* NEWMODEX status */

    if (st == AGC_STATUS_OK)
    {
        agc_telem_select_list(AGC_ENTRY_DNLSTCOD_ENTRY);
        st = AGC_STATUS_OK;
    }

    if (!AGC_ASSERT(agc_erase_modreg() == 62 || st != AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_cm_sm_separation(ctx) : st); /* then the display */
}

/**********************************************************************
 * P62 (p. 792): S61.1, then the rest of P62.
 *
 * FUNCTION NAME: agc_job_cm_sm_separation
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_cm_sm_separation(agc_job_ctx_t *ctx)
{
    imu_check_return_job = AGC_JOB_CM_SM_SEPARATION_AFTER_CHECK;

    if (!AGC_ASSERT(imu_check_return_job == AGC_JOB_CM_SM_SEPARATION_AFTER_CHECK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_entry_imu_check(ctx));
}

/**********************************************************************
 * P62 after S61.1 (p. 792): POSEXIT = P62.3 (desired .05 g gimbal angles
 * without display), CM/DAPIC (which goes on to P62.2).
 *
 * FUNCTION NAME: agc_job_cm_sm_separation_after_check
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_cm_sm_separation_after_check(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    es->posexit = AGC_ENTRY_POSEXIT_DESIRED_GIMBALS;

    return (agc_entry_dap_ic(ctx));
}

/**********************************************************************
 * P62.2 (pp. 792-793): AVEGEXIT = CM/POSE, GOPERF1R 41 (request CM/SM
 * separation); the job goes on at P61.3.
 *
 * FUNCTION NAME: agc_job_cm_sm_separation_request
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_cm_sm_separation_request(agc_job_ctx_t *ctx)
{
    agc_status_t st; /* GOPERF1R status */

    agc_guid_state()->avg.avgexit = AGC_JOB_CM_POSE; /* DXCH AVEGEXIT */
    {
        const agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH,
                                      AGC_JOB_CM_SM_SEPARATION_PROCEED,
                                      AGC_JOB_CM_SM_SEPARATION_REQUEST};

        st = agc_dsky_display(ctx, AGC_DSKY_GOPERF1R, ENTRY_OCT41, &cont);
    }

    if (!AGC_ASSERT(agc_guid_state()->avg.avgexit == AGC_JOB_CM_POSE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_entry_display_protect() : st); /* P61.3 */
}

/**********************************************************************
 * P62.2 +3 (p. 793): PROCEED: CM/DAPON (NODOFLAG is set there).
 *
 * FUNCTION NAME: agc_job_cm_sm_separation_proceed
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_cm_sm_separation_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_cm_dapon(ctx));
}

/**********************************************************************
 * P62.1 (p. 793): V06N61 again (terminate and enter redisplay).
 *
 * FUNCTION NAME: agc_job_cm_sm_separation_target_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_cm_sm_separation_target_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_flash(ctx, AGC_DSKY_GOFLASH, VN_ENTRY_TARGET_DISPLAY, AGC_JOB_CM_SM_SEPARATION_TARGET_DISPLAY, AGC_JOB_CM_SM_SEPARATION_END_HOLD, AGC_JOB_CM_SM_SEPARATION_TARGET_DISPLAY));
}

/**********************************************************************
 * P62.1 PROCEED (pp. 793-794): end the attitude hold: ROLLC from HEADSUP,
 * ALFACOM = ALFAPAD, BETACOM = 0, P63FLAG = +1, N22 entry display. P63
 * directly when CMDAPMOD is -1 or +0, else wait for WAKEP62.
 *
 * FUNCTION NAME: agc_job_cm_sm_separation_end_hold
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_cm_sm_separation_end_hold(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */
    agc_phase_var_t    v  = {AGC_RESTART_JOB, AGC_JOB_CM_SM_SEPARATION_END_HOLD, AGC_TASK_NONE, 0, false, 0U};

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change_var(004024, &v);                                                     /* ENTRYVN display  */
    es->rollc                     = agc_dp_pack(headsup_roll(es), agc_dp_low(es->rollc)); /* TS */
    es->alfacom                   = es->alfapad;                                          /* trim, negative   */
    es->betacom                   = 0;
    es->wake_separation_interlock = 1;                                                    /* EXDAP2 -> +0     */
    es->entry_verb                = 6;                                                    /* V06N22           */
    es->entry_noun                = 22;
    agc_flag_set(AGC_FLAG_ENTRYDSP);

    if (es->cmdapmod == AGC_ENTRY_DAPMOD_MINUS1 || es->cmdapmod == AGC_ENTRY_DAPMOD_PLUS0)
    {
        return (agc_job_entry_initialization(ctx));
    }

    agc_phase_change(4, 000004);

    return (AGC_STATUS_OK); /* P63.1           */
}

/**********************************************************************
 * WAKEP62 (p. 794): task from the entry DAP when ALFA nears trim.
 *
 * FUNCTION NAME: agc_task_wake_cm_sm_separation
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_task_wake_cm_sm_separation(void)
{
    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    _Static_assert(AGC_ENTRY_PRIO13 > 0, "AGC constant");

    if (!AGC_ASSERT(agc_entry_state()->posexit < AGC_ENTRY_POSEXIT_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_novac(AGC_ENTRY_PRIO13, AGC_JOB_ENTRY_INITIALIZATION));
}

/* ======================================================================
 * P63 .. P67 (pp. 795-802)
 * ==================================================================== */

/**********************************************************************
 * P63 (p. 795): POSEXIT = STARTENT, N64 entry display, P63FLAG = -1, flush
 * the N22 display, disable group 4, ENDOFJOB.
 *
 * FUNCTION NAME: agc_job_entry_initialization
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_initialization(agc_job_ctx_t *ctx)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */
    agc_status_t       st;                     /* NEWMODEX status */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(es != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    st                            = entry_mode(es, 63, 6, 64);                         /* V06N64 */
    es->posexit                   = AGC_ENTRY_POSEXIT_STARTENT;
    es->wake_separation_interlock = -1;
    agc_phase_change(4, 000004);
    const agc_status_t sd         = agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0, NULL); /* CLEANDSP status */

    if (sd != AGC_STATUS_OK)
    {
        return (sd); /* PRIOBORT 01502 or exec overflow: job abandoned */
    }

    agc_phase_change(4, 000004); /* P63.1           */

    return (st);
}

/**********************************************************************
 * P64 (p. 797): MM 64, N74 (ROLLC VI D) at .05 g.
 *
 * FUNCTION NAME: entry_post_0_05g
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t entry_post_0_05g(agc_entry_state_t *es)
{
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG__05GSW)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_mode(es, 64, 6, 74));
}

/**********************************************************************
 * P65 (p. 798): MM 65, start the N69 job P65.1, restart 4.55 and 5.3,
 * GOTOADDR = UPCONTRL (the caller continues there, REFAZE10).
 *
 * FUNCTION NAME: entry_upcontrol
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t entry_upcontrol(agc_entry_state_t *es)
{
    agc_phase_var_t v  = { AGC_RESTART_JOB, AGC_JOB_REFAZE10, AGC_TASK_NONE, 0, false, 0U };
    agc_status_t    st = agc_phase_newmode(65); /* NEWMODEX / NOVAC status */

    if (st == AGC_STATUS_OK)
    {
        st = agc_exec_novac(AGC_ENTRY_PRIO13, AGC_JOB_ENTRY_UPCONTROL_DISPLAY);
    }

    agc_phase_change2(000554, 010035, &v);
    es->gotoaddr = AGC_ENTRY_GOTO_UPCONTRL;

    return (st);
}

/**********************************************************************
 * P65.1 (p. 798): entry display off, V16N69 ROLLC DL VL, GOFLASHR.
 *
 * FUNCTION NAME: agc_job_entry_upcontrol_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_upcontrol_display(agc_job_ctx_t *ctx)
{
    agc_status_t st; /* GOFLASHR status */

    agc_flag_clear(AGC_FLAG_ENTRYDSP);
    st = entry_flash(ctx, AGC_DSKY_GOFLASHR, 16, 69, AGC_JOB_ENTRY_UPCONTROL_DISPLAY, AGC_JOB_ENTRY_UPCONTROL_PROCEED, AGC_JOB_ENTRY_UPCONTROL_DISPLAY);

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_ENTRYDSP)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? agc_job_entry_display_protect() : st); /* P61.3 */
}

/**********************************************************************
 * P65.1 PROCEED (p. 798): entry display back on, P63.1.
 *
 * FUNCTION NAME: agc_job_entry_upcontrol_proceed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_upcontrol_proceed(void)
{
    agc_flag_set(AGC_FLAG_ENTRYDSP);

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_ENTRYDSP)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(4, 000004);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * KILLGRP4 (p. 801): disable group 4 (HUNTEST / N69 restarts).
 *
 * FUNCTION NAME: entry_disable_restart_group
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t entry_disable_restart_group(void)
{
    _Static_assert(AGC_RESTART_GROUPS >= 4, "AGC constant");
    agc_phase_change(4, 000004);

    if (!AGC_ASSERT(agc_phase_get(4) == 0u)) /* group 4 inactive */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * P66 (p. 799): MM 66, N22 trim gimbal angles; P66END.
 *
 * FUNCTION NAME: entry_ballistic
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t entry_ballistic(agc_entry_state_t *es)
{
    agc_status_t st = entry_mode(es, 66, 6, 22); /* NEWMODEX status */

    agc_flag_set(AGC_FLAG_ENTRYDSP);

    if (!AGC_ASSERT(es->entry_noun == 22)) /* ENTRYVN for OVERNOUT */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? entry_disable_restart_group() : st); /* then the display */
}

/**********************************************************************
 * P67 (p. 800): MM 67, N66 ROLLC XRNGERR DNRNGERR; P66END.
 *
 * FUNCTION NAME: entry_final_phase
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t entry_final_phase(agc_entry_state_t *es)
{
    agc_status_t st = entry_mode(es, 67, 6, 66); /* NEWMODEX status */

    agc_flag_set(AGC_FLAG_ENTRYDSP);

    if (!AGC_ASSERT(es->entry_noun == 66)) /* ENTRYVN for OVERNOUT */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? entry_disable_restart_group() : st); /* then the display */
}

/**********************************************************************
 * P67.1 (p. 800): V16N67 RTOGO LAT LONG, GOFLASH (ENTER redisplays).
 *
 * FUNCTION NAME: agc_job_entry_final_phase_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_final_phase_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_entry_state() != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (entry_flash(ctx, AGC_DSKY_GOFLASH, 16, 67, AGC_JOB_ENTRY_FINAL_PHASE_END, AGC_JOB_ENTRY_FINAL_PHASE_END, AGC_JOB_ENTRY_FINAL_PHASE_DISPLAY));
}

/**********************************************************************
 * P67.1 end (pp. 800-801): entry DAP off (CM/DSTBY, GYMDIFSW), AVEGEXIT =
 * SERVEXIT, GOTOPOOH.
 *
 * FUNCTION NAME: agc_job_entry_final_phase_end
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_final_phase_end(void)
{
    agc_inhint();
    agc_flag_clear(AGC_FLAG_CM_DSTBY);
    agc_flag_clear(AGC_FLAG_GYMDIFSW);
    agc_relint();
    agc_guid_state()->avg.avgexit = AGC_JOB_SERVEXIT; /* SERVCAD2 */

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_CM_DSTBY)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_gotopooh());
}

/**********************************************************************
 * P67.2 (p. 802): present LAT, LONG, ALT of RN at PIPTIME (pad radius,
 * earth). P67.3 (SERVNOUT) is done by the caller.
 *
 * FUNCTION NAME: agc_entry_final_phase_latlong
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_entry_final_phase_latlong(void)
{
    const agc_latlong_opt_t opt = {false, false};    /* LUNA, ERAD clear */
    agc_entry_state_t      *es  = agc_entry_state(); /* entry erasables */
    agc_navaux_padload_t    pad;                     /* pad loads (earth) */
    agc_latlong_t           ll;                      /* LAT LONG ALT of RN */
    agc_status_t            st;                      /* status */

    agc_flag_clear(AGC_FLAG_ERADFLAG);
    agc_flag_clear(AGC_FLAG_LUNAFLAG);
    st = agc_navaux_get_padload(&pad);

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_from_r(entry_ip(), &pad, opt, agc_erase_rn(), agc_erase_piptime(), &ll);
    }

    if (st == AGC_STATUS_OK)
    {
        es->lat = ll.lat;
        es->lng = ll.lon;
        es->alt = ll.alt;
    }

    return (st);
}

/* ======================================================================
 * S61.1 (pp. 803-805)
 * ==================================================================== */

/**********************************************************************
 * AGC: none (RETRN1: TC 60GENRET, P61-P67.agc p. 805) -- continue the
 * program that called S61.1.
 *
 * FUNCTION NAME: imu_check_return
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
static agc_status_t imu_check_return(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(imu_check_return_job == AGC_JOB_ENTRY_PREPARATION_TARGET_DISPLAY || imu_check_return_job == AGC_JOB_CM_SM_SEPARATION_AFTER_CHECK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (imu_check_return_job == AGC_JOB_CM_SM_SEPARATION_AFTER_CHECK)
    {
        return (agc_job_cm_sm_separation_after_check(ctx));
    }

    return (agc_job_entry_preparation_target_display(ctx));
}

/**********************************************************************
 * S61.1 after CALRB MIDTOAV2 (p. 804): the future state vector goes to RN1
 * VN1 PIPTIME1; S61.1C runs at that time (restart 4.43).
 *
 * FUNCTION NAME: agc_job_entry_imu_check_extrapolated
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_imu_check_extrapolated(void)
{
    agc_integ_midtoav_out_t out;                          /* MIDTOAV2 outputs */
    agc_avg_g_t            *avg = &agc_guid_state()->avg; /* average-G data (RN1, VN1) */
    agc_status_t            st;                           /* MIDTOAV2 / WAITLIST status */

    st = agc_integ_midtoav2(entry_ip(), agc_integ_shared_ctx(), AGC_JOB_ENTRY_IMU_CHECK_EXTRAPOLATED, &out);

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* asleep in INTSTALL     */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    avg->rn1      = out.rn1;            /* RN1 VN1 PIPTIME1       */
    avg->vn1      = out.vn1;
    avg->piptime1 = out.piptime1;
    agc_sp_t dt61 = agc_dp_low(out.dt); /* S61DT, cs B14 */

    if (!AGC_ASSERT(dt61 > 0))
    {
        dt61 = 1;
    }

    agc_erase_set_s61dt(dt61);
    st = agc_waitlist_call((uint16_t) dt61, AGC_TASK_ENTRY_START_AVERAGE_G);
    agc_phase_change(4, 040434); /* 4.43 S61.1C            */

    return (st); /* ENDOFJOB               */
}

/**********************************************************************
 * S61.1 (p. 803): R02BOTH; with AVERAGEG on, the IMU check S61.1A, else
 * bring the state vector forward and start AVERAGEG (S61.1C).
 *
 * FUNCTION NAME: agc_job_entry_imu_check
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_imu_check(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_imu_status_check(); /* IMU status check       */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_flag_test(AGC_FLAG_AVEGFLAG))
    {
        return (agc_job_entry_imu_axis_test(ctx));
    }

    if (!AGC_ASSERT(imu_check_return_job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_entry_imu_check_extrapolated());
}

/**********************************************************************
 * S61.1C (p. 804): task at the state vector time: S61.1A job (4.45),
 * AVEGEXIT = SERVEXIT, PREREAD (5.41) starts AVERAGEG.
 *
 * FUNCTION NAME: agc_task_entry_start_average_g
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_task_entry_start_average_g(void)
{
    agc_status_t st = agc_exec_findvac(AGC_ENTRY_PRIO13, AGC_JOB_ENTRY_IMU_AXIS_TEST); /* FINDVAC status */

    agc_guid_state()->avg.avgexit = AGC_JOB_SERVEXIT; /* SERVCAD1      */
    agc_phase_change2(000454, 000415, NULL);          /* 4.45, 5.41    */

    if (!AGC_ASSERT(agc_guid_state()->avg.avgexit == AGC_JOB_SERVEXIT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(imu_check_return_job != AGC_JOB_NONE)) /* S61.1 */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_task_preread_csm()); /* POSTJUMP      */
}

/**********************************************************************
 * S61.1A / S61.1B / RETRN1-3 (pp. 804-805): the IMU Y axis must be within
 * 30 deg of UNIT(VN x RN): cos/2 + C(30)LIM overflows. -Y within 30 deg:
 * alarm 01427 (IMU reversed), else 01426 (unsatisfactory); both show V05N09
 * and wait 10 s before returning.
 *
 * FUNCTION NAME: agc_job_entry_imu_axis_test
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_imu_axis_test(agc_job_ctx_t *ctx)
{
    agc_interp_t *ip       = entry_ip();           /* entry job interpreter state */
    agc_mat_t     refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit */
    agc_vec_t     y;                               /* Y axis of VN x RN, B1 */
    agc_dp_t      c;                               /* COS(THETA)/2, B1 */
    uint16_t      code;                            /* alarm 01426 or 01427 */

    agc_interp_clear_ovf(ip); /* BOVB TCDANZIG */
    y = agc_mat_vec(ip, &refsmmat, agc_vec_cross(ip, agc_erase_vn(), agc_erase_rn()));
    y = agc_vec_unit(ip, y, NULL, NULL);
    c = y.c[1];               /* COS(THETA)/2  */

    if (c >= 0)
    {
        agc_dp_add_ovf_only(ip, c, AGC_ENTRY_C30LIM); /* DAD C(30)LIM */

        if (agc_interp_test_ovf(ip))
        {
            return (imu_check_return(ctx)); /* RETRN1        */
        }

        code = 01426; /* RETRN3        */
    }
    else
    {
        agc_dp_add_ovf_only(ip, agc_dp_neg(c), AGC_ENTRY_C30LIM);
        code = agc_interp_test_ovf(ip) ? 01427 : 01426; /* S61.1B        */
    }

    agc_alarm(code);
    agc_status_t st = agc_dsky_display(ctx, AGC_DSKY_GODSPR, agc_dsky_vn(5, 9), NULL); /* GODSPR status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(code == 01426 || code == 01427))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_svc_delayjob(1000, AGC_JOB_ENTRY_IMU_CHECK_RETURN)); /* 10SECS */
} /* agc_job_entry_imu_axis_test */

/**********************************************************************
 * RETRN1 after the 10 s DELAYJOB (P61-P67.agc p. 805).
 *
 * FUNCTION NAME: agc_job_entry_imu_check_return
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    job context of the running job (display
 *                                   and DELAYJOB continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_job_entry_imu_check_return(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(imu_check_return_job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (imu_check_return(ctx));
}

/* ======================================================================
 * S61.2 (pp. 806-816)
 * ==================================================================== */

/**********************************************************************
 * TFF/TRIG (p. 815): .5 COS(THETA) = .5 - |ARG/(RN RTERM (1 + X))|, returns
 * .5 SIN(THETA) with the sign of ARG.
 *
 * FUNCTION NAME: agc_entry_tff_trig
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     IO   interpreter state of the job: OVFIND,
 *                                     DVSIGN, alarms
 *   t         const agc_tff_t *  I    TFF data left by CALCTFF (TFFX,
 *                                     TFFTEM, NRMAG, NRTERM)
 *   cdelf2    agc_dp_t *         O    .5 COS(THETA), CDELF/2, B1
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- .5 SIN(THETA) with the sign of ARG, B1
 **********************************************************************/
agc_dp_t agc_entry_tff_trig(agc_interp_t *ip, const agc_tff_t *t, agc_dp_t *cdelf2)
{
    agc_dp_t m; /* MPAC, B1 */

    m       = agc_dp_add(ip, agc_dp_shr(t->tffx, 1), AGC_DP_HALF);
    m       = agc_dp_mul_trunc(agc_dp_mul_trunc(m, t->nrmag), t->nrterm);
    m       = agc_dp_div(ip, t->tfftem, m);
    *cdelf2 = agc_dp_sub(ip, AGC_DP_HALF, agc_dp_abs(m));
    m       = agc_dp_add(ip, agc_dp_neg(agc_dp_sq(*cdelf2)), AGC_DP_QUARTER);

    return (agc_dp_sign(agc_dp_sqrt(m), t->tfftem));
}

/**********************************************************************
 * FISHCALC (p. 812): URH = UR CDELF + UHOR SDELF; Fischer radius at the
 * latitude of URH (GETERAD), m B29.
 *
 * FUNCTION NAME: fishcalc
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *   sdelf2    agc_dp_t        I    .5 SIN(THETA), B1
 *   cdelf2    agc_dp_t        I    .5 COS(THETA), B1
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- Fischer radius at the latitude of URH
 *          (ERADM), m B29
 **********************************************************************/
static agc_dp_t fishcalc(agc_interp_t *ip, agc_dp_t sdelf2, agc_dp_t cdelf2)
{
    const agc_avg_g_t *avg = &agc_guid_state()->avg;                           /* UNITW, B0 */
    agc_dp_t           sinlat;                                                 /* .5 SIN(LAT), B1 */
    agc_vec_t          h   = agc_vec_cross(ip, urone, agc_entry_state()->uni); /* UR x UNI, B1 */

    if (!AGC_ASSERT(agc_dp_abs(sdelf2) <= AGC_DP_HALF))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(agc_dp_abs(cdelf2) <= AGC_DP_HALF))
    {
        return (0); /* no valid result */
    }

    h   = agc_vec_shl(ip, agc_vec_scale(ip, h, sdelf2), 1);
    urh = agc_vec_add(ip, h, agc_vec_scale(ip, urone, cdelf2));

    /* DOT SL1 UNITW: .5 SIN(LAT) on the TP MPAC, then GETERAD.           */
    sinlat = entry_tp_shl(ip, agc_vec_dot_tp(ip, urh, avg->unitw), 1);

    return (agc_latlong_geterad(ip, sinlat));
}

/**********************************************************************
 * VRCALC (P61-P67.agc p. 816): ACOS(URH.RT) in revs.
 *
 * FUNCTION NAME: agc_entry_vrcalc
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------------
 *   u         agc_vec_t  I    URH, B1
 *   rt        agc_vec_t  I    RT target, B1
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- the range angle ACOS(u.rt), revs B0
 **********************************************************************/
agc_dp_t agc_entry_vrcalc(agc_vec_t u, agc_vec_t rt)
{
    agc_interp_t *ip = entry_ip(); /* entry job interpreter state */

    if (!AGC_ASSERT(ip != NULL))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_acos(ip, entry_tp_shl(ip, agc_vec_dot_tp(ip, u, rt), 2)));
}

/**********************************************************************
 * DISPTARG (p. 816): target rotated through the time to go (range KTETA1 -
 * TTE1), range angle from URH.
 *
 * FUNCTION NAME: disptarg
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *   rtgo      agc_dp_t             I    range-to-go estimate, revs B0
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- the range angle to the rotated target
 *          (VRCALC), revs B0
 **********************************************************************/
static agc_dp_t disptarg(agc_interp_t *ip, agc_entry_state_t *es, agc_dp_t rtgo)
{
    es->dtearot = agc_dp_sub(ip, agc_dp_mul_trunc(rtgo, AGC_ENTRY_KTETA1), es->tte1);
    entry_earrot(ip, es, false); /* EARROT2       */

    return (agc_entry_vrcalc(urh, es->rt));
}

/**********************************************************************
 * PREVGAM / VGAMCALC (pp. 813-814): vis-viva speed VGAM (m/cs B7) and
 * positive flight path angle (revs) at RTERM (B29).
 *
 * FUNCTION NAME: agc_entry_vgamcalc
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state of the job: OVFIND,
 *                                  DVSIGN, alarms
 *   t         agc_tff_t *     IO   TFF data; RTERM and NRTERM are set
 *                                  (PREVGAM)
 *   rterm     agc_dp_t        I    terminal radius RTERM, m B29
 *   vgam      agc_dp_t *      O    VGAM speed at RTERM, m/cs B7
 *
 * RETURNS: agc_dp_t -- agc_dp_t -- GAMMA/360 at RTERM, a positive number,
 *          revs B0
 **********************************************************************/
agc_dp_t agc_entry_vgamcalc(agc_interp_t *ip, agc_tff_t *t, agc_dp_t rterm, agc_dp_t *vgam)
{
    agc_dp_t prod;           /* RMAG RTERM, scale 2^(58-2NR) m^2 */
    agc_dp_t d;              /* 2(RN - RTERM), scale 2^(8-NR) */
    agc_dp_t s;              /* VGAM/RTMU, B10 */
    int      sh = t->nr - 8; /* SL* count -8 + NR */

    if (!AGC_ASSERT(t->nr >= 0 && t->nr < 28))
    {
        return (0); /* no valid result */
    }

    t->rterm  = rterm;
    t->nrterm = agc_dp_shl(ip, rterm, t->nr);                           /* SL* 0,1       */
    prod      = agc_dp_mul_trunc(t->nrmag, t->nrterm);
    d         = agc_dp_sub(ip, t->nrmag, t->nrterm);
    d         = (sh >= 0) ? agc_dp_shl(ip, d, sh) : agc_dp_shr(d, -sh); /* SL* or SR */
    s         = agc_dp_sqrt(agc_dp_sub(ip, agc_dp_div(ip, d, prod), t->tffvsq));
    *vgam     = agc_dp_div(ip, s, t->rtmu);
    d         = agc_dp_div(ip, agc_dp_sqrt(t->tffnp), agc_dp_mul_trunc(s, t->nrterm));

    return (agc_dp_acos(ip, agc_dp_shr(d, 1)));
}

/**********************************************************************
 * GMAXCALC (p. 809): with VBAR = (V - 36 kft/s)/20 kft/s, GMAX = (4/(1 +
 * 4.8 VBAR^2))(GAM - 6.05 - 2.4 VBAR^2) + 10, >= 0, as 100 GMAX B-14 (L/D =
 * .3, bank 0).
 *
 * FUNCTION NAME: agc_entry_gmaxcalc
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   vgam      agc_dp_t    I    VGAM at 300 kft, m/cs B7
 *   gam       agc_dp_t    I    GAMMA at 300 kft, positive down, revs B0
 *   gmax      agc_dp_t *  O    100 GMAX, B14 (>= 0)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a failed
 *          assertion
 **********************************************************************/
agc_status_t agc_entry_gmaxcalc(agc_dp_t vgam, agc_dp_t gam, agc_dp_t *gmax)
{
    agc_interp_t *ip = entry_ip(); /* entry job interpreter state */
    agc_dp_t      vb;              /* VBARSQ, B2 */
    agc_dp_t      num;             /* numerator, B14 */
    agc_dp_t      g;               /* GMAX denominator, then 100 GMAX, B14 */

    if (!AGC_ASSERT(gmax != NULL) || !AGC_ASSERT(vgam >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    vb    = agc_dp_sq(agc_dp_div(ip, agc_dp_sub(ip, vgam, AGC_ENTRY_KFT36_PER_S), AGC_ENTRY_KFT20_PER_S)); /* B-2 */
    num   = agc_dp_add(ip, agc_dp_add(ip, agc_dp_mul_trunc(vb, AGC_ENTRY_KR1), gam), AGC_ENTRY_M6_05DEG);
    num   = agc_dp_mul_trunc(num, AGC_ENTRY_KR2);
    g     = agc_dp_add(ip, agc_dp_div(ip, vb, AGC_ENTRY_KR4), AGC_ENTRY_ONE_16TH);
    g     = agc_dp_add(ip, agc_dp_div(ip, num, g), AGC_ENTRY_KR3);
    *gmax = (g >= 0) ? g : 0;                                                                                  /* HI6ZEROS     */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * S61.2 first part (p. 808): TTE to EMSALT and RTGO from EMSALT.
 *
 * FUNCTION NAME: ems_altitude_range
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------------
 *   ip        agc_interp_t *       IO   interpreter state of the job:
 *                                       OVFIND, DVSIGN, alarms
 *   es        agc_entry_state_t *  IO   entry erasables (ENTRY_LEXICON
 *                                       names)
 *   eradm     agc_dp_t *           O    Fischer radius at the EMSALT point,
 *                                       m B29
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a failed
 *          RTGO assertion, else the TFF status
 **********************************************************************/
static agc_status_t ems_altitude_range(agc_interp_t *ip, agc_entry_state_t *es, agc_dp_t *eradm)
{
    agc_dp_t     c = 0; /* CDELF/2, B1 */
    agc_dp_t     t = 0; /* TFF time, cs B28 */
    agc_dp_t     a;     /* SDELF/2 then angles, B1 */
    agc_status_t st;    /* TFF status */

    if (!AGC_ASSERT(ip != NULL && es != NULL) || !AGC_ASSERT(eradm != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    entry_preparation_tff.rtmu = (agc_dp_sub(ip, es->emsalt, AGC_ENTRY_KFT290) >= 0) ? AGC_ENTRY_RTMU_INV_LUNENT : AGC_ENTRY_RTMU_INV_EARTH; /* LUNENT */
    st                         = agc_tff_conic(ip, &entry_preparation_tff);

    if (st == AGC_STATUS_OK)
    {
        st = agc_tff_calctff(ip, &entry_preparation_tff, AGC_ENTRY_RTRIAL, &t);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    a      = agc_entry_tff_trig(ip, &entry_preparation_tff, &c);
    *eradm = fishcalc(ip, a, c);
    st     = agc_tff_calctff(ip, &entry_preparation_tff, agc_dp_add(ip, *eradm, es->emsalt), &t);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    es->tte1 = agc_dp_neg(t); /* countdown     */
    es->tte  = es->tte1;
    a        = agc_entry_tff_trig(ip, &entry_preparation_tff, &c);
    *eradm   = fishcalc(ip, a, c);
    a        = agc_entry_vrcalc(urh, es->rt);
    a        = disptarg(ip, es, a);
    es->rtgo = disptarg(ip, es, a);

    if (!AGC_ASSERT(es->rtgo >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* ems_altitude_range */

/**********************************************************************
 * S61.2 (pp. 806-811): N60 GMAX VPRED GAMMAEI and N63 RTGO VIO TTE.
 *
 * FUNCTION NAME: agc_entry_ems_data
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_entry_ems_data(void)
{
    agc_interp_t      *ip    = entry_ip();                         /* entry job interpreter state */
    agc_entry_state_t *es    = agc_entry_state();                  /* entry erasables */
    agc_dp_t           eradm = 0;                                  /* ERADM, m B29 */
    agc_dp_t           vgam  = 0;                                  /* VGAM, m/cs B7 */
    agc_dp_t           c     = 0;                                  /* CDELF/2, B1 */
    agc_dp_t           t     = 0;                                  /* TFF time, cs B28 */
    agc_dp_t           g;                                          /* GAMMA, revs B0 */
    agc_status_t       st    = ems_altitude_range(ip, es, &eradm); /* status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g       = agc_entry_vgamcalc(ip, &entry_preparation_tff, /* VIO */
                                 entry_preparation_tff.rterm,
                                 &vgam);
    es->vio = agc_dp_add(ip, agc_dp_div(ip, AGC_ENTRY_VEMSCON, agc_dp_mul_trunc(g, vgam)), vgam);
    g       = agc_entry_vgamcalc(ip, &entry_preparation_tff, agc_dp_add(ip, eradm, AGC_ENTRY_KFT300), &vgam);
    st      = agc_entry_gmaxcalc(vgam, g, &es->gmax);

    if (st == AGC_STATUS_OK)
    { /* 2nd ERAD iter. */
        st = agc_tff_calctff(ip, &entry_preparation_tff, agc_dp_add(ip, eradm, AGC_ENTRY_KFT400), &t);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    g           = agc_entry_tff_trig(ip, &entry_preparation_tff, &c);
    eradm       = fishcalc(ip, g, c);
    g           = agc_entry_vgamcalc(ip, &entry_preparation_tff, agc_dp_add(ip, eradm, AGC_ENTRY_KFT400), &vgam);
    es->gammaei = agc_dp_pack(agc_dp_high(agc_dp_neg(g)), agc_dp_high(es->rtgo));
    es->vpred   = vgam;

    if (!AGC_ASSERT(es->vpred >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(es->gmax >= 0)) /* HI6ZEROS floor */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* agc_entry_ems_data */

/* ======================================================================
 * S62.3 (pp. 817-818)
 * ==================================================================== */

/**********************************************************************
 * S62.3 (P61-P67.agc pp. 817-818): desired body triad for trimmed flight at
 * ROLLC and ALFAPAD, UYD = UYA COS(ROLLC) + UZA SIN(ROLLC), UXD = UNIT(UYD
 * x UXA) SIN(ALFATRIM) + UXA COS(ALFATRIM), UZD = UXD x UYD, then CALCGA
 * (CPHIFLAG clear: CPHI = THETAD) for the N22 display.
 *
 * FUNCTION NAME: agc_entry_desired_gimbal_angles
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_BAD_ARG when an input
 *          or state assertion fails; otherwise the status of the first
 *          service call that failed
 **********************************************************************/
agc_status_t agc_entry_desired_gimbal_angles(void)
{
    agc_interp_t      *ip     = entry_ip();                                 /* entry job interpreter state */
    agc_entry_state_t *es     = agc_entry_state();                          /* entry erasables */
    agc_dp_t           alfa   = agc_dp_shr(agc_dp_from_sp(es->alfapad), 1); /* ALFATRIM, revs B0 */
    agc_mat_t          nb;                                                  /* XNB YNB ZNB, B1 */
    agc_mat_t          sm     = agc_erase_refsmmat();                       /* REFSMMAT, half-unit */
    uint16_t           cdu[3] = {0, 0, 0};                                  /* CALCGA CDU patterns */
    agc_status_t       st;                                                  /* CALCGA status */
    int                i;                                                   /* loop index */

    nb.row[1] = agc_vec_add(ip, agc_vec_scale(ip, es->uya, agc_dp_cos(es->rollc)), agc_vec_scale(ip, es->uza, agc_dp_sin(es->rollc)));
    nb.row[1] = agc_vec_shl(ip, nb.row[1], 1); /* YNB */
    nb.row[0] = agc_vec_shl(ip, agc_vec_cross(ip, nb.row[1], es->uxa), 1);
    nb.row[0] = agc_vec_add(ip, agc_vec_scale(ip, nb.row[0], agc_dp_sin(alfa)), agc_vec_scale(ip, es->uxa, agc_dp_cos(alfa)));
    nb.row[0] = agc_vec_shl(ip, nb.row[0], 1); /* XNB */
    nb.row[2] = agc_vec_shl(ip, agc_vec_cross(ip, nb.row[0], nb.row[1]), 1);
    agc_flag_clear(AGC_FLAG_CPHIFLAG);
    st        = agc_imu_calcga(ip, &nb, &sm, cdu);

    for (i = 0; i < 3; i++)
    {
        int32_t w = (int32_t) (cdu[i] & 077777u); /* CDU pattern */

        es->cphi[i] = (agc_sp_t) ((w >= 16384) ? w - 32768 : w); /* /180 */
    }

    return (st);
} /* agc_entry_desired_gimbal_angles */
