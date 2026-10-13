/**********************************************************************
 * FILE NAME: common.c
 *
 * PURPOSE:
 *   The alignment erasables and the small helpers shared by every
 *   alignment and marking routine of both vehicles: the CDULOGIC / 1STO2S
 *   word formats, CDUTRIG / CALCSMSC glue, the integration call, the DSKY
 *   display calls with their astronaut returns, the IMUSTALL / OPTSTALL
 *   outcome, restart phase changes, SET1/PDT and NCOARSE, COARSE,
 *   LOADTIME and the DSPTEM1 words.
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 737-784 (SET1/PDT p. 769)
 *   Luminary099 P51-P53.agc pp. 926-983 (COARSE, NCOARSE p. 951)
 *   RTB_OP_CODES.agc pp. 1397-1398 (CDULOGIC, 1STO2S, LOADTIME)
 *   POWERED_FLIGHT_SUBROUTINES.agc (CDUTRIG)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   ALIGNMENT STORAGE  agc_align_ram_t I/O per member  the block this file owns
 *   DSPTEM1, +1   uint16_t  I/O  raw words      agc_dsky_dsptem()
 *   THETAD        uint16_t  I    CDU words      agc_erase_thetad()
 *   PIPAX..Z      agc_sp_t  O    PIPA counts    NCOARSE zeroes them
 *   TIME1/TIME2   agc_dp_t  I    cs 2^28        agc_time_now_cs()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_*            DSKY displays, astronaut returns (dsky.h)
 *   agc_exec_job_yield    executive: jobs, VAC areas, sleep / wake (exec.h)
 *   agc_imu_*             IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_guid_*            CDUTRIG, *NBSM* / *SMNB*, PIPASR (guidance.h)
 *   agc_integ_*           state vector integration (integration.h)
 *   agc_vec_make          vector arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_test_ovf   interpreter state: OVFIND (interp.h)
 *   agc_flag_set          flag words (flags_common.h)
 *   agc_phase_*           restart phase tables, CHECKMM (exec.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_curtains          CURTAINS (exec.h)
 *   agc_time_now_cs       TIME2/TIME1 (state.h)
 *   agc_inhint            INHINT / RELINT (exec.h)
 *   agc_relint            INHINT / RELINT (exec.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   agc_align_ones_to_twos is a local 1STO2S (DEVIATION: the RTB op is
 *   not public); align_stall_outcome checks that the woken job sees a
 *   finished stall.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "agc/programs/alignment.h"

#define CDU_WORD_MASK     077777u /* 15-bit CDU register                  */
#define CDU_MODULUS       0100000 /* one revolution in CDU counts         */
#define CDU_HALF_REV      16384   /* counts in half a revolution          */
#define CDU_TO_REVS_SHIFT 8192    /* 2^13: cdu count -> DP revs           */
#define WORD_SIGN         040000u /* sign bit of a 15-bit word            */
#define WORD_ONES_NEG     077777  /* 1's complement bias                  */

static agc_align_ram_t align_ram; /* the alignment erasables (one block, cleared at fresh start) */

/**********************************************************************
 * AGC: none (accessor of the ALIGNMENT STORAGE erasables,
 *      ERASABLE_ASSIGNMENTS.agc)
 *
 * Alignment erasable block (ERASABLE_ASSIGNMENTS "ALIGNMENT STORAGE").
 *
 * FUNCTION NAME: agc_align_ram
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_align_ram_t * -- the alignment erasable block, never NULL
 **********************************************************************/
agc_align_ram_t *agc_align_ram(void)
{
    return (&align_ram);
}

/**********************************************************************
 * AGC: none (fresh start: clear the alignment erasables)
 *
 * Fresh start: clear the alignment erasables and the OGC gyro buffer
 * (owned by imu since the layering lane).
 *
 * FUNCTION NAME: agc_align_ram_reset
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_align_ram_reset(void)
{
    const agc_align_ram_t zero    = {.mkvac_index = -1};
    const agc_dp_t        none[3] = {0, 0, 0}; /* OGC, IGC, MGC, rev 2^0 */

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_OGC, none); /* OGC (imu's) cleared with the alignment erasables */

    align_ram                      = zero;
    align_ram.qmin                 = AGC_JOB_NONE;
    align_ram.qmaj                 = AGC_JOB_NONE;
    align_ram.auto_optics_return   = AGC_JOB_NONE;
    align_ram.sighting_mark_return = AGC_JOB_NONE;
    align_ram.planet_ret           = AGC_JOB_NONE;
    align_ram.coarse_ret           = AGC_JOB_NONE;
}

/**********************************************************************
 * CDULOGIC (RTB_OP_CODES.agc, Luminary p. 1397) -- the SP 2's complement
 * CDU angle (half-revolutions) as a DP angle in revolutions.  The routine
 * itself is guid-powered's agc_imu_cdulogic(); this keeps the alignment
 * name used throughout the lane.
 *
 * FUNCTION NAME: agc_align_cdulogic
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   cdu       uint16_t  I    CDU words X, Y, Z (2^15 = 1 rev)
 *
 * RETURNS: agc_dp_t -- the angle, revs 2^0 (DP)
 **********************************************************************/
agc_dp_t agc_align_cdulogic(uint16_t cdu)
{
    const agc_dp_t r = agc_imu_cdulogic((uint16_t) (cdu & CDU_WORD_MASK)); /* CDULOGIC result, revs 2^0 */

    return (r);
}

/**********************************************************************
 * 1STO2S / 1TO2SUB (RTB_OP_CODES.agc, Luminary p. 1397-1398) -- convert a
 * DP angle in revolutions to a SP 2's complement angle in half revolutions.
 * The low word is dropped (truncation toward zero, as the 1's complement
 * high word is), and a doubling overflow wraps by one revolution.
 * DEVIATION: local copy of the RTB op (owner sys-misc, not public yet).
 *
 * FUNCTION NAME: agc_align_ones_to_twos
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   revs      agc_dp_t  I    angle, revs 2^0 (DP)
 *
 * RETURNS: uint16_t -- 2s complement CDU pattern, 2^15 = 1 rev
 **********************************************************************/
uint16_t agc_align_ones_to_twos(agc_dp_t revs)
{
    int32_t hv = revs / CDU_TO_REVS_SHIFT; /* major word doubled, CDU counts */

    if (hv >= CDU_HALF_REV)
    {
        hv -= CDU_MODULUS;
    }
    else if (hv < -CDU_HALF_REV)
    {
        hv += CDU_MODULUS;
    }
    else
    {
        /* no doubling overflow */
    }

    if (hv < 0)
    {
        hv += CDU_MODULUS;
    }

    return ((uint16_t) hv);
}

/**********************************************************************
 * AGC: CDUTRIG (Comanche055 POWERED_FLIGHT_SUBROUTINES.agc p. 1365;
 *      Luminary099 POWERED_FLIGHT_SUBROUTINES.agc p. 1259)
 *
 * CDUTRIG with explicit angles (POWERED_FLIGHT_SUBROUTINES CD*TR*GS): the
 * CDUSPOT order is Y, Z, X.
 *
 * FUNCTION NAME: agc_align_cdutrig
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   cdu_xyz   const uint16_t   I    CDU words X, Y, Z (2^15 = 1 rev)
 *   t         agc_cdutrig_t *  O    CDUTRIG sines and cosines (scale 2^1)
 *
 * RETURNS: none
 **********************************************************************/
void agc_align_cdutrig(const uint16_t cdu_xyz[3], agc_cdutrig_t *t)
{
    uint16_t yzx[3]; /* CDUY, CDUZ, CDUX words */

    yzx[AGC_TRIG_X] = cdu_xyz[0];
    yzx[AGC_TRIG_Y] = cdu_xyz[1];
    yzx[AGC_TRIG_Z] = cdu_xyz[2];
    agc_imu_cdtrigs(t, yzx);
}

/**********************************************************************
 * CALCSMSC (P51-P53.agc, Comanche p. 763) on the SINCDU/COSCDU left by
 * CDUTRIG: the routine is the imu lane's agc_imu_calcsmsc (X, Y, Z order).
 *
 * FUNCTION NAME: agc_align_calcsmsc
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ip        agc_interp_t *         IO   interpreter state (OVFIND, MPAC
 *                                         words)
 *   t         const agc_cdutrig_t *  I    CDUTRIG sines and cosines (scale 2^1)
 *   nb        agc_mat_t *            O    XNB, YNB, ZNB rows, half-unit
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_align_calcsmsc(agc_interp_t *ip, const agc_cdutrig_t *t, agc_mat_t *nb)
{
    const agc_vec_t s = agc_vec_make(t->sincdu[AGC_TRIG_X], t->sincdu[AGC_TRIG_Y], t->sincdu[AGC_TRIG_Z]); /* SINCDU X, Y, Z, scale 2^1 */
    const agc_vec_t c = agc_vec_make(t->coscdu[AGC_TRIG_X], t->coscdu[AGC_TRIG_Y], t->coscdu[AGC_TRIG_Z]); /* COSCDU X, Y, Z, scale 2^1 */

    return (agc_imu_calcsmsc(ip, s, c, nb));
}

/**********************************************************************
 * AGC: CDUX (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 39; Luminary099
 *      ERASABLE_ASSIGNMENTS.agc p. 92)
 *
 * The present CDUX, CDUY, CDUZ (RDCDUS reads them under INHINT).
 *
 * FUNCTION NAME: align_read_cdus
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   cdu       uint16_t  I    CDU words X, Y, Z (2^15 = 1 rev)
 *
 * RETURNS: none
 **********************************************************************/
void align_read_cdus(uint16_t cdu[3])
{
    agc_inhint();
    cdu[0] = agc_erase_cdux();
    cdu[1] = agc_erase_cduy();
    cdu[2] = agc_erase_cduz();
    agc_relint();
}

/**********************************************************************
 * AGC: CALL (Comanche055 INTERPRETER.agc p. 1128; Luminary099
 *      INTERPRETER.agc p. 1023)
 *
 * CALL CDUTRIG; CALL CALCSMSC on the present CDUs.
 *
 * FUNCTION NAME: align_nb_present
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ip        agc_interp_t *   IO   interpreter state (OVFIND, MPAC words)
 *   t         agc_cdutrig_t *  O    CDUTRIG sines and cosines (scale 2^1)
 *   nb        agc_mat_t *      O    XNB, YNB, ZNB rows, half-unit
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t align_nb_present(agc_interp_t *ip, agc_cdutrig_t *t, agc_mat_t *nb)
{
    agc_imu_cdutrig(t);

    return (agc_align_calcsmsc(ip, t, nb));
}

/**********************************************************************
 * AGC: CSMPREC (Comanche055 INTEGRATION_INITIALIZATION.agc p. 1318;
 *      Luminary099 INTEGRATION_INITIALIZATION.agc p. 1214)
 *
 * CSMPREC / LEMPREC / CSMCONIC / LEMCONIC (INTEGRATION_INITIALIZATION).
 *
 * FUNCTION NAME: align_integ
 *
 * ARGUMENTS:
 *   Argument    Type                I/O  Description (scale)
 *   ----------  ------------------  ---  --------------------------------
 *   ip          agc_interp_t *      IO   interpreter state (OVFIND, MPAC
 *                                        words)
 *   kind        align_integ_kind_t  I    CSMPREC, LEMPREC, CSMCONIC or LEMCONIC
 *   tdec1       agc_dp_t            I    TDEC1, the time to integrate to,
 *                                        cs 2^28
 *   resume_job  agc_job_id_t        I    job resumed after an integration
 *                                        wait (INTSTALL)
 *   out         agc_integ_out_t *   O    integration output (RATT, VATT, TAT, X2)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t align_integ(agc_interp_t *ip, align_integ_kind_t kind, agc_dp_t tdec1, agc_job_id_t resume_job, agc_integ_out_t *out)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* the shared integration context */
    agc_status_t     st  = AGC_STATUS_BAD_ARG;     /* status of the call */

    switch (kind)
    {
        case ALIGN_CSMPREC:
            st = agc_integ_csmprec(ip, ctx, tdec1, resume_job);
            break;
        case ALIGN_LEMPREC:
            st = agc_integ_lemprec(ip, ctx, tdec1, resume_job);
            break;
        case ALIGN_CSMCONIC:
            st = agc_integ_csmconic(ip, ctx, tdec1, resume_job);
            break;
        case ALIGN_LEMCONIC:
            st = agc_integ_lemconic(ip, ctx, tdec1, resume_job);
            break;
        default:
            break;
    }

    if (st == AGC_STATUS_OK)
    {
        *out = ctx->out;
    }

    return (st);
}

/**********************************************************************
 * AGC: GOFLASH (Comanche055 DISPLAY_INTERFACE_ROUTINES.agc p. 1469;
 *      Luminary099 DISPLAY_INTERFACE_ROUTINES.agc p. 1358)
 *
 * GOFLASH / GOFLASHR / GOMARK.. / GOPERF1 / GOPERF4R with the returns.
 *
 * FUNCTION NAME: align_flash
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   kind      agc_dsky_disp_t        I    display entry (GOFLASH, GOPERF1,
 *                                         ...)
 *   vn        uint16_t               I    verb and noun, VN packing
 *   term      agc_job_id_t           I    job for V34 (terminate)
 *   proceed   agc_job_id_t           I    job for V33 (proceed)
 *   enter     agc_job_id_t           I    job for ENTER / V32 (recycle)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t align_flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint16_t vn, agc_job_id_t term, agc_job_id_t proceed, agc_job_id_t enter)
{
    agc_dsky_cont_t cont; /* astronaut returns */

    cont.terminate = term;
    cont.proceed   = proceed;
    cont.enter     = enter;

    return (agc_dsky_display(ctx, kind, vn, &cont));
}

/**********************************************************************
 * AGC: GODSP (Comanche055 DISPLAY_INTERFACE_ROUTINES.agc p. 1468;
 *      Luminary099 DISPLAY_INTERFACE_ROUTINES.agc p. 1357)
 *
 * GODSP / GODSPR / GODSPRET / CLEANDSP / KLEENEX (no astronaut return).
 *
 * FUNCTION NAME: align_show
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   kind      agc_dsky_disp_t        I    display entry (GOFLASH, GOPERF1,
 *                                         ...)
 *   vn        uint16_t               I    verb and noun, VN packing
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t align_show(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint16_t vn)
{
    return (agc_dsky_display(ctx, kind, vn, NULL));
}

/**********************************************************************
 * AGC: none (reads the verb of the DSKY answer)
 *
 * The verb of the answer was V32 (MPAC = V32 at the third return).
 *
 * FUNCTION NAME: align_resp_recycle
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when the crew answered V32
 **********************************************************************/
bool align_resp_recycle(void)
{
    uint8_t verb = 0; /* verb of the answer */
    uint8_t noun = 0; /* noun of the answer */

    agc_dsky_response(&verb, &noun);

    return (verb == 32u);
}

/**********************************************************************
 * AGC: TC (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 85; Luminary099
 *      ERASABLE_ASSIGNMENTS.agc p. 110)
 *
 * TC / GOTO another resumption point of the running job.
 *
 * FUNCTION NAME: align_goto
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   next      agc_job_id_t  I    job to continue as
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t align_goto(agc_job_id_t next)
{
    if (!AGC_ASSERT(next != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_job_yield(next));
}

/**********************************************************************
 * AGC: IMUSTALL (Comanche055 IMU_MODE_SWITCHING_ROUTINES.agc p. 1441;
 *      Luminary099 IMU_MODE_SWITCHING_ROUTINES.agc p. 1330)
 *
 * IMUSTALL / OPTSTALL: TC CURTAINS on the bad return.
 *
 * FUNCTION NAME: align_stall
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   dev       agc_imu_device_t  I    stalled device (IMU or optics)
 *   wake_as   agc_job_id_t      I    job woken when the stalled device
 *                                    finishes
 *   done      bool *            O    true when the device had already
 *                                    finished (no sleep)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_ALARM after an alarm
 **********************************************************************/
agc_status_t align_stall(agc_imu_device_t dev, agc_job_id_t wake_as, bool *done)
{
    agc_imu_stall_result_t r  = AGC_IMU_STALL_BAD;                   /* IMUSTALL / OPTSTALL outcome */
    agc_status_t           st = agc_imu_stall_dev(dev, wake_as, &r); /* status of the call */

    *done = false;

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (r == AGC_IMU_STALL_BAD)
    {
        agc_curtains(); /* TC CURTAINS (alarm 217)  */

        return (AGC_STATUS_ALARM);
    }

    *done = (r == AGC_IMU_STALL_GOOD);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (L+2 / L+3 return of IMUSTALL / OPTSTALL as seen by the woken
 *      job)
 *
 * The outcome seen by the job woken from a stall.
 *
 * FUNCTION NAME: align_stall_outcome
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ------------------------------------
 *   dev       agc_imu_device_t  I    stalled device (IMU or optics)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_ALARM after an alarm; AGC_STATUS_BAD_ARG on a
 *          failed precondition
 **********************************************************************/
agc_status_t align_stall_outcome(agc_imu_device_t dev)
{
    const agc_imu_stall_result_t r = agc_imu_stall_outcome(dev); /* outcome seen by the woken job */

    if (!AGC_ASSERT(r != AGC_IMU_STALL_SLEEPING))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (r != AGC_IMU_STALL_GOOD)
    {
        agc_curtains();

        return (AGC_STATUS_ALARM);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: TC (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 85; Luminary099
 *      ERASABLE_ASSIGNMENTS.agc p. 110)
 *
 * TC PHASCHNG; OCT code (type C, job at the next instruction).
 *
 * FUNCTION NAME: align_phase_job
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   code      uint16_t      I    restart phase code (octal, as in TC
 *                                PHASCHNG)
 *   job       agc_job_id_t  I    job restarted from the phase
 *   prio      agc_prio_t    I    its priority
 *
 * RETURNS: none
 **********************************************************************/
void align_phase_job(uint16_t code, agc_job_id_t job, agc_prio_t prio)
{
    agc_phase_var_t v = {0};

    v.kind       = AGC_RESTART_JOB;
    v.job        = job;
    v.prio_or_dt = (int16_t) prio;
    agc_phase_change_var(code, &v);
}

/**********************************************************************
 * AGC: LOADTIME (Comanche055 RT8_OP_CODES.agc p. 1508; Luminary099
 *      RTB_OP_CODES.agc p. 1397)
 *
 * LOADTIME (RTB): TIME2, TIME1, cs B28.
 *
 * FUNCTION NAME: align_loadtime
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- TIME2, TIME1, cs 2^28
 **********************************************************************/
agc_dp_t align_loadtime(void)
{
    const uint32_t t = agc_time_now_cs(); /* CDUTRIG sines and cosines */

    const agc_dp_t d = (agc_dp_t) (t & (AGC_TIME_MODULUS_CS - 1u)); /* TIME2/TIME1, cs 2^28 */

    return (d);
}

/**********************************************************************
 * AGC: none (1s complement word to a signed SP)
 *
 * A raw 15-bit 1's complement word as a signed SP (-0 reads as 0).
 *
 * FUNCTION NAME: align_word_to_sp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   w         uint16_t  I    raw 15-bit 1s complement word
 *
 * RETURNS: agc_sp_t -- the value, -0 as 0
 **********************************************************************/
agc_sp_t align_word_to_sp(uint16_t w)
{
    const uint16_t m = (uint16_t) (w & CDU_WORD_MASK); /* the mark */

    if ((m & WORD_SIGN) != 0u)
    {
        return ((agc_sp_t) ((int32_t) m - WORD_ONES_NEG));
    }

    return ((agc_sp_t) m);
}

/**********************************************************************
 * AGC: none (signed SP to the 1s complement word)
 *
 * A signed SP as the raw 15-bit 1's complement word.
 *
 * FUNCTION NAME: align_sp_to_word
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   v         agc_sp_t  I    the value
 *
 * RETURNS: uint16_t -- the raw 15-bit word
 **********************************************************************/
uint16_t align_sp_to_word(agc_sp_t v)
{
    if (v < 0)
    {
        return ((uint16_t) ((int32_t) v + WORD_ONES_NEG));
    }

    return ((uint16_t) v);
}

/**********************************************************************
 * AGC: DSPTEM1 (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 76; Luminary099
 *      ERASABLE_ASSIGNMENTS.agc p. 102)
 *
 * DSPTEM1, DSPTEM1 +1 (pinball buffer) as a DP.
 *
 * FUNCTION NAME: align_dsptem1
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- DSPTEM1, DSPTEM1 +1 as a DP (2^28 per major word
 *          unit)
 **********************************************************************/
agc_dp_t align_dsptem1(void)
{
    const agc_sp_t hi = align_word_to_sp(agc_dsky_dsptem(0)); /* DSPTEM1, major word, scale B14 per unit */
    const agc_sp_t lo = align_word_to_sp(agc_dsky_dsptem(1)); /* DSPTEM1 +1, minor word, scale B0 per unit */

    return (agc_dp_pack(hi, lo));
}

/**********************************************************************
 * AGC: none (pinball buffer store)
 *
 * DSPTEM1, DSPTEM1 +1 = the words of a DP (the store of align_dsptem1).
 *
 * FUNCTION NAME: align_set_dsptem1
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   v         agc_dp_t  I    the DP, as many units of 2^-28 as the display
 *                            needs
 *
 * RETURNS: none
 **********************************************************************/
void align_set_dsptem1(agc_dp_t v)
{
    agc_dsky_set_dsptem(0, align_sp_to_word(agc_dp_high(v)));
    agc_dsky_set_dsptem(1, align_sp_to_word(agc_dp_low(v)));
}

/**********************************************************************
 * SET1/PDT (Comanche p. 769) with the GCOMP clear and DRIFTFLG set that
 * follow it at each call; NCOARSE (Luminary p. 951) is the same plus PIPA
 * zeroing, done by its caller.  1/PIPADT = TIME1.
 *
 * FUNCTION NAME: align_set1_pdt
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK; BAD_ARG when 1/PIPADT refuses TIME1
 **********************************************************************/
agc_status_t align_set1_pdt(void)
{
    const agc_dp_t     zero3[3] = {0, 0, 0};
    const agc_status_t st       = agc_imu_set_pipa_interval(agc_erase_time1()); /* 1/PIPADT store status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_GCOMP, zero3);
    agc_flag_set(AGC_FLAG_DRIFTFLG);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: TC (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 85; Luminary099
 *      ERASABLE_ASSIGNMENTS.agc p. 110)
 *
 * TC PHASCHNG; OCT code (type A, group 4).
 *
 * FUNCTION NAME: align_phase4
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   code      uint16_t  I    restart phase code (octal, as in TC PHASCHNG)
 *
 * RETURNS: none
 **********************************************************************/
void align_phase4(uint16_t code)
{
    agc_phase_change(code);
}

/**********************************************************************
 * NCOARSE (P51-P53.agc, Luminary p. 951) -- 1/PIPADT = TIME1, PIPAX, Y, Z =
 * -0 (0 here), GCOMP = 0, SET DRIFTFLG.
 *
 * FUNCTION NAME: align_ncoarse
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the SET1/PDT status (align_set1_pdt)
 **********************************************************************/
agc_status_t align_ncoarse(void)
{
    agc_erase_set_pipax(0);
    agc_erase_set_pipay(0);
    agc_erase_set_pipaz(0);

    return (align_set1_pdt());
}

/**********************************************************************
 * AGC: COARSE (Luminary099 P51-P53.agc p. 952)
 *
 * COARSE, second half: IMUFINE and its IMUSTALL.
 *
 * FUNCTION NAME: coarse_fine
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t coarse_fine(void)
{
    agc_status_t st = agc_imu_fine(); /* IMUFINE   */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_ALIGN_FINE_DONE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (align_goto(agc_align_ram()->coarse_ret));
}

/**********************************************************************
 * COARSE (Luminary p. 951): IMUCOARS to THETAD and its IMUSTALL.
 *
 * FUNCTION NAME: align_coarse
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------------
 *   ret       agc_job_id_t  I    job the routine continues as when it ends
 *                                (the AGC return address)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t align_coarse(agc_job_id_t ret)
{
    agc_align_ram()->coarse_ret = ret;
    uint16_t thetad[3]; /* THETAD X, Y, Z words */

    for (int i = 0; i < 3; i++)
    {
        thetad[i] = agc_erase_thetad(i);
    }

    agc_status_t st = agc_imu_coarse(thetad); /* IMUCOARS  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_IMU, AGC_JOB_ALIGN_COARSE_DONE, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* the alarm or CURTAINS ended the job */
    }

    return (coarse_fine());
}

/**********************************************************************
 * AGC: COARSE (Luminary099 P51-P53.agc p. 952)
 *
 * COARSE: woken after the IMUCOARS stall.
 *
 * FUNCTION NAME: agc_job_align_coarse_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_align_coarse_done(void)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_IMU); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    return (coarse_fine());
}

/**********************************************************************
 * AGC: COARSE (Luminary099 P51-P53.agc p. 952)
 *
 * COARSE: woken after the IMUFINE stall; continue as the caller asked.
 *
 * FUNCTION NAME: agc_job_align_fine_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_align_fine_done(void)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_IMU); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    return (align_goto(agc_align_ram()->coarse_ret));
}

/**********************************************************************
 * AGC: none (BOV / BOVB to the next instruction)
 *
 * BOV +1 / BOVB to the next instruction: the overflow indicator is tested
 * (and so cleared) but either way the code goes on.
 *
 * FUNCTION NAME: align_clear_ovf
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *
 * RETURNS: none
 **********************************************************************/
void align_clear_ovf(agc_interp_t *ip)
{
    const bool was_set = agc_interp_test_ovf(ip); /* OVFIND before the test (discarded) */

    (void) was_set; /* the overflow is discarded    */
}
