/**********************************************************************
 * FILE NAME: display.c
 *
 * PURPOSE:
 *   The display subroutines shared by the rendezvous targeting programs
 *   (P32-P35, P72-P75), both vehicles:
 *     VNPOOH     (P34-35_P74-75.agc p.484)  flash, TERM -> P00
 *     DISPLAYE   (p.483)                    V06N55 flash
 *     P3XORP7X   (p.484)
 *     VN1645     (pp.481-482)  +MGA, COMPTGO, V16N45, KILCLOCK,
 *                              N45PROC, CLUPDATE
 *     S34/35.5   (p.480; LM NTARGCHK)       final-pass N81 / N59
 *     DISDVLVC   (P32-P33_P72-P73.agc p.662)
 *     ALMXIT / 611 alarm flash (P32-P33 p.652, P34-35 p.464)
 *   The AGC subroutines put the calling job to sleep on a flashing
 *   display and return into the job when the crew answers. In C the job
 *   ends at the display and is resumed as a continuation job; the
 *   continuation of a subroutine that "returns" is stored in the
 *   rendezvous erasables (QSAVED / NORMEX / SUBEXIT), and a continuation
 *   that falls back into its caller does so through agc_exec_job_yield
 *   (the job keeps its core set and priority).
 *
 * AGC SOURCE:
 *   Comanche055 P34-35_P74-75.agc pp. 480-484, P32-P33_P72-P73.agc
 *   pp. 652, 662, P30-P37.agc p. 636 (COMPTGO)
 *   Luminary099 P32-P35_P72-P75.agc (differs: NTARGCHK in S34/35.5,
 *   COMPTGO clears DISPDEX and waits 2 cs)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type        I/O  Scale      Description
 *   ---------  ----------  ---  ---------  ----------------------------
 *   MODREG     agc_sp_t    I    -          major mode (P3x / P7x)
 *   FINALFLG UPDATFLG REFSMFLG XDELVFLG  flags  I/O  -  display control
 *   REFSMMAT   agc_mat_t   I    2^1        for +MGA (GET+MGA)
 *   DSPTEM1 DSPTEMX OPTION1 OPTION2  words  I/O  -  display temporaries
 *   DISPDEX NVWORD1  words  O   -          clock display control
 *   QSAVED NORMEX +MGA DELVLVC DVLOS GDT/2 DELVSIN (agc_rendz_t) I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                Description
 *   ------------------  ------------------------------------------------
 *   agc_dsky_display    GOFLASH / GOFLASHR / GOMARKF / PRIODSP requests
 *   agc_rendz_get_mga   GET+MGA: middle gimbal angle of the burn
 *   agc_waitlist_call   CLOKTASK start (COMPTGO)
 *   agc_phase_change2   2PHSCHNG restart protection (COMPTGO)
 *   agc_svc_delayjob    DELAYJOB 1SEC (VN1645)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   rz_alarm_flash / rz_prio_alarm raise the alarm code they are given
 *   (00600 .. 00606, 00611, radar 00501 00503 00514 00525 00526
 *   00530) and show
 *   V05N09. Functions return AGC_STATUS_BAD_ARG for NULL pointers or bad
 *   display arguments, else the status of the display call.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called from rendezvous jobs; one VNPOOH display pending at a time
 *   (vnpooh_slot is the AGC's VERBNOUN and return address).
 *
 * NOTES:
 *   DEVIATION (NTARGCHK): the AGC sums ones' complement SP words with ADS
 *   (overflow wraps); here the sum is exact.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"

/* PHASCHNG OCT 04024 (VN1645 p.482): group 4, phase 2, type A.          */
#define RZ_PHASE_04024 ((uint16_t) 04024) /* group 4 phase 2 */

/* HIGH9 (P3XORP7X): MODREG >= 64 decimal is a P7x program.             */
#define RZ_HIGH9_MODES 64 /* MODREG of P70 */

/* The display a VNPOOH redisplay (ENTER) repeats: VERBNOUN, and where
 * PROCEED continues (the return address of the VNPOOH call).          */
typedef struct rz_vnpooh_slot
{
    uint8_t      verb; /* verb of VERBNOUN                  */
    uint8_t      noun; /* noun of VERBNOUN                  */
    agc_job_id_t next; /* PROCEED continuation (return)     */
} rz_vnpooh_slot_t;

static rz_vnpooh_slot_t vnpooh_slot = {6U, 55U, AGC_JOB_GOTOPOOH}; /* last */

/**********************************************************************
 * P3XORP7X (p.484): MODREG & HIGH9 = 0 -> P3x
 * AGC: P3XORP7X (P34-35_P74-75.agc p.484)
 *
 * FUNCTION NAME: rz_active_vehicle_program
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true for a P3x program (this vehicle active), false for
 *          P7x
 **********************************************************************/
bool rz_active_vehicle_program(void)
{
    agc_sp_t mm = agc_erase_modreg(); /* MODREG, unscaled */

    return (mm < RZ_HIGH9_MODES);
}

/**********************************************************************
 * Continue the running job at another entry (the AGC falls back into the
 * caller's code at the stored return address).
 * AGC: none (the AGC falls back into the caller's code)
 *
 * FUNCTION NAME: rz_continue
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------
 *   next      agc_job_id_t  I    job started next
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for
 *          AGC_JOB_NONE or a failed yield
 **********************************************************************/
agc_status_t rz_continue(agc_job_id_t next)
{
    if (!AGC_ASSERT(next != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_exec_job_yield(next); /* yield status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (st);
}

/**********************************************************************
 * VNPOOH (p.484): CAF VNxxxx, TC VNPOOH
 * GOFLASH; TERMINATE -> GOTOPOOH, PROCEED -> return (next), ENTER ->
 * "TC -5", the same display again.
 * AGC: VNPOOH (P34-35_P74-75.agc p.484)
 *
 * FUNCTION NAME: rz_vnpooh
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   verb      uint8_t                I    verb of the display
 *   noun      uint8_t                I    noun of the display
 *   next      agc_job_id_t           I    job started next
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_vnpooh(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t next)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(next != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    vnpooh_slot.verb = verb; /* TS VERBNOUN  */
    vnpooh_slot.noun = noun;
    vnpooh_slot.next = next;

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, verb, noun, AGC_JOB_GOTOPOOH, next, AGC_JOB_RENDZ_VNPOOH));
}

/**********************************************************************
 * VNPOOH "TC -5": ENTER on a VNPOOH display shows it again.
 * AGC: VNPOOH "TC -5" (P34-35_P74-75.agc p.484)
 *
 * FUNCTION NAME: agc_job_rendz_vnpooh
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendz_vnpooh(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(vnpooh_slot.next != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_vnpooh(ctx, vnpooh_slot.verb, vnpooh_slot.noun, vnpooh_slot.next));
}

/**********************************************************************
 * DISPLAYE (p.483): V06N55 (NN/ELEV/CENTANG), return = next
 * AGC: DISPLAYE (P34-35_P74-75.agc p.483)
 *
 * FUNCTION NAME: rz_displaye
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   next      agc_job_id_t           I    job started next
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_displaye(const agc_job_ctx_t *ctx, agc_job_id_t next)
{
    return (rz_vnpooh(ctx, 6U, 55U, next));
}

/**********************************************************************
 * TC ALARM / OCT code, CAF V05N09, GOFLASH (P32 ALMXIT, P33 and P34
 * 611): TERM -> P00, PROCEED -> proceed, V32 -> recycle (the alarm and
 * display again, or the program step the AGC returns to).
 * AGC: ALMXIT (P32-P33_P72-P73.agc p.652), P34/P74C 611 (P34-35_P74-75.agc
 * p.463)
 *
 * FUNCTION NAME: rz_alarm_flash
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   code      agc_alarm_code_t       I    alarm code
 *   proceed   agc_job_id_t           I    job for PROCEED (V33)
 *   recycle   agc_job_id_t           I    job for RECYCLE (V32)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_alarm_flash(const agc_job_ctx_t *ctx, agc_alarm_code_t code, agc_job_id_t proceed, agc_job_id_t recycle)
{
    agc_alarm(code);

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 5U, 9U, AGC_JOB_GOTOPOOH, proceed, recycle));
}

/**********************************************************************
 * VN1645 (p.481)
 * +MGA = -.01 deg (not computed); -.02 deg on the final pass; when
 * REFSMFLG is set and this is a P3x program, +MGA from DELVSIN (GET+MGA).
 * Then COMPTGO (TTOGO clock), QSAVED = return, DELAYJOB 1 s, V16N45.
 * AGC: VN1645 (P34-35_P74-75.agc p.481)
 *
 * FUNCTION NAME: rz_ttogo_display
 *
 * ARGUMENTS:
 *   Argument   Type                   I/O  Description (scale)
 *   ---------  ---------------------  ---  ------------------------
 *   ctx        const agc_job_ctx_t *  I    job context of the calling job
 *   ip         agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz         agc_rendz_t *          I/O  rendezvous erasables
 *   resume_as  agc_job_id_t           I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_ttogo_display(const agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume_as)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL) || !AGC_ASSERT(rz != NULL) || !AGC_ASSERT(resume_as != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->plus_mga = RZ_DP_MINUS_01; /* MGA = -.01 */

    if (agc_flag_test(AGC_FLAG_FINALFLG))
    {
        rz->plus_mga = agc_dp_add(ip, RZ_DP_MINUS_01, RZ_DP_MINUS_01);

        if (agc_flag_test(AGC_FLAG_REFSMFLG) && rz_active_vehicle_program())
        {
            agc_mat_t    refsmmat = agc_erase_refsmmat(); /* REFSMMAT, 2^1 */
            agc_status_t st;                              /* GET+MGA */

            st = agc_rendz_get_mga(ip, rz->delvsin, &refsmmat, &rz->plus_mga);

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }
        }
    }

    agc_status_t st = rz_comptgo(AGC_JOB_RENDZ_TTOGO_DISPLAY); /* GET45: COMPTGO */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->qsaved = resume_as; /* TS QSAVED      */

    /* DELAYJOB 1SEC, then V16N45 */
    return (agc_svc_delayjob(RZ_1SEC_CS, AGC_JOB_RENDZ_TTOGO_DISPLAY));
}

/**********************************************************************
 * VN1645 after DELAYJOB: V16N45 (TRKMKCNT, TTOGO, +MGA) flashing.
 * AGC: VN1645 after DELAYJOB (P34-35_P74-75.agc p.481)
 *
 * FUNCTION NAME: agc_job_rendz_ttogo_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendz_ttogo_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_rendz_erasable()->qsaved != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 16U, 45U, AGC_JOB_RENDZ_KILCLOCK, AGC_JOB_RENDZ_TTOGO_PROCEED, AGC_JOB_RENDZ_CLUPDATE));
}

/**********************************************************************
 * KILCLOCK (p.481): DISPDEX = 0 (stop the clock display), GOTOPOOH.
 * AGC: KILCLOCK (P34-35_P74-75.agc p.481)
 *
 * FUNCTION NAME: agc_job_rendz_kilclock
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendz_kilclock(void)
{
    rz_dispdex_zero();
    agc_status_t st = agc_exec_gotopooh(); /* GOTOPOOH */

    return (st);
}

/**********************************************************************
 * CLUPDATE (p.482): DISPDEX = 0, PHASCHNG 04024, CLEAR UPDATFLG, return
 * to QSAVED for the next computation cycle.
 * AGC: CLUPDATE (P34-35_P74-75.agc p.482)
 *
 * FUNCTION NAME: agc_job_rendz_clupdate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendz_clupdate(void)
{
    rz_dispdex_zero();
    agc_phase_change(RZ_PHASE_04024);
    agc_flag_clear(AGC_FLAG_UPDATFLG);

    return (rz_continue(agc_rendz_erasable()->qsaved));
}

/**********************************************************************
 * N45PROC (p.482): FINALFLG already set -> KILCLOCK (await a new
 * program); otherwise PHASCHNG 04024, set FINALFLG, CLUPDATE.
 * AGC: N45PROC (P34-35_P74-75.agc p.482)
 *
 * FUNCTION NAME: agc_job_rendz_ttogo_proceed
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendz_ttogo_proceed(void)
{
    if (agc_flag_test(AGC_FLAG_FINALFLG))
    {
        return (agc_job_rendz_kilclock());
    }

    agc_phase_change(RZ_PHASE_04024);
    agc_flag_set(AGC_FLAG_FINALFLG); /* UPFLAG     */

    return (agc_job_rendz_clupdate());
}

/**********************************************************************
 * DISDVLVC (P32-P33 p.662)
 * DELVLVC = dv; M = S32/33.X; DELVLVC = M DELVLVC (VSL1); VNPOOH with the
 * verb/noun the caller left in X1 (V06N81 or V06N82); return = next.
 * AGC: DISDVLVC (P32-P33_P72-P73.agc p.662)
 *
 * FUNCTION NAME: rz_disdvlvc
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *          I/O  rendezvous erasables
 *   dv        agc_vec_t              I    delta V in reference axes, m/cs
 *                                         2^7
 *   noun      uint8_t                I    noun of the display (81 or 82)
 *   next      agc_job_id_t           I    job started next
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_disdvlvc(const agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t dv, uint8_t noun, agc_job_id_t next)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL) || !AGC_ASSERT(rz != NULL) || !AGC_ASSERT(noun == 81U || noun == 82U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->delvlvc     = dv;                                      /* STORE      */
    agc_mat_t    m;                                            /* matrix rows, unit 2^1 */
    agc_status_t st = agc_rendz_csi_cdh_lv_matrix(ip, rz, &m); /* S32/33.X */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->delvlvc = agc_vec_shl(ip, agc_mat_vec(ip, &m, rz->delvlvc), 1);

    return (rz_vnpooh(ctx, 6U, noun, next));
}

/**********************************************************************
 * S34/35.5 (p.480)
 * Not the final pass: set UPDATFLG, go to FLAGOFF. Final pass: save the
 * computed DELVLVC (CM in DVLOS, LM in GDT/2 after clearing NTARGFLG)
 * and show V06N81 for a crew overwrite.
 * AGC: S34/35.5 (P34-35_P74-75.agc p.480)
 *
 * FUNCTION NAME: rz_tpi_final_displays
 *
 * ARGUMENTS:
 *   Argument   Type                   I/O  Description (scale)
 *   ---------  ---------------------  ---  ------------------------
 *   ctx        const agc_job_ctx_t *  I    job context of the calling job
 *   ip         agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz         agc_rendz_t *          I/O  rendezvous erasables
 *   resume_as  agc_job_id_t           I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_tpi_final_displays(const agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume_as)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL) || !AGC_ASSERT(rz != NULL) || !AGC_ASSERT(resume_as != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->normex = resume_as; /* STQ SUBEXIT  */

    if (!agc_flag_test(AGC_FLAG_FINALFLG))
    {
        agc_flag_set(AGC_FLAG_UPDATFLG);

        return (rz_tpi_flagoff(ctx, ip, rz));
    }

    if (rz_is_csm())
    {
        rz->dvlos = rz->delvlvc; /* FLAGON (CM)  */
    }
    else
    {
        rz_vflag_set(RZ_VF_NTARGFLG, false); /* FLAGON (LM)  */
        rz->gdt_2 = rz->delvlvc;
    }

    return (rz_vnpooh(ctx, 6U, 81U, AGC_JOB_RENDZ_FINAL_DV_CHECKED));
}

/**********************************************************************
 * S34/35.5 after PROCEED on V06N81: test for a crew overwrite of the
 * computed delta V (CM: |DELVLVC - DVLOS| non-zero; LM: NTARGCHK sets
 * NTARGFLG); an overwrite is processed by S34/35.3. NOCHG: CLEAR
 * XDELVFLG, DELVSIN = DELVEET3. FLAGOFF follows.
 * AGC: S34/35.5 after V06N81, NOCHG (P34-35_P74-75.agc p.480)
 *
 * FUNCTION NAME: agc_job_rendz_final_dv_checked
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendz_final_dv_checked(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */
    bool         overwrite;                 /* crew changed DELVLVC */

    if (rz_is_csm())
    {
        overwrite = rz_abval(&ip, agc_vec_sub(&ip, rz->delvlvc, rz->dvlos)) != 0;
    }
    else
    {
        /**********************************************************************
         * LM NTARGCHK: add the differences of the six words of DELVLVC and its
         * saved copy GDT/2. DEVIATION: the AGC sums ones' complement SP words
         * with ADS (overflow wraps); here the sum is exact, which only differs
         * when the word differences overflow 14 bits and cancel.
         * AGC: NTARGCHK (Luminary099 P32-P35_P72-P75.agc)
         *
         * FUNCTION NAME: rz_ntargchk
         *
         * ARGUMENTS:
         *   Argument  Type                 I/O  Description (scale)
         *   --------  -------------------  ---  ---------------------------
         *   rz        const agc_rendz_t *  I    rendezvous erasables
         *
         * RETURNS: bool -- true when the crew changed DELVLVC (word sum non-zero)
         **********************************************************************/
        int32_t sum = 0; /* word differences */

        for (int i = 0; i < AGC_VEC_N; i++)
        {
            sum += (int32_t) agc_dp_high(rz->gdt_2.c[i]) - (int32_t) agc_dp_high(rz->delvlvc.c[i]);
            sum += (int32_t) agc_dp_low(rz->gdt_2.c[i]) - (int32_t) agc_dp_low(rz->delvlvc.c[i]);
        }

        overwrite = sum != 0;

        if (overwrite)
        {
            rz_vflag_set(RZ_VF_NTARGFLG, true); /* UPFLAG       */
        }
    }

    if (overwrite)
    {
        agc_status_t st = agc_rendz_tpi_crew_overwrite(&ip, rz, AGC_JOB_RENDZ_FINAL_DV_CHECKED); /* S34/35.3 */

        if (st != AGC_STATUS_OK)
        {
            return (st); /* INTSTALL sleep: redo step */
        }
    }

    agc_flag_clear(AGC_FLAG_XDELVFLG); /* NOCHG     */
    rz->delvsin = rz->delveet3;

    return (rz_tpi_flagoff(ctx, &ip, rz));
} /* agc_job_rendz_final_dv_checked */

/**********************************************************************
 * FLAGOFF: S34/35.4 (DVLOS from DELVEET3), V06N59, return to SUBEXIT.
 * AGC: FLAGOFF (P34-35_P74-75.agc p.480)
 *
 * FUNCTION NAME: rz_tpi_flagoff
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *          I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_tpi_flagoff(const agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL) || !AGC_ASSERT(rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rendz_tpi_los_delta_v(ip, rz); /* S34/35.4 */

    return (rz_vnpooh(ctx, 6U, 59U, rz->normex));
}

/**********************************************************************
 * VN1645 as a public entry (return to earth, guid-entry): the same as
 * rz_ttogo_display with the rendezvous erasables.
 * AGC: VN1645 (P34-35_P74-75.agc p.481)
 *
 * FUNCTION NAME: agc_rendz_ttogo_display_start
 *
 * ARGUMENTS:
 *   Argument   Type                   I/O  Description (scale)
 *   ---------  ---------------------  ---  ------------------------
 *   ctx        const agc_job_ctx_t *  I    job context of the calling job
 *   ip         agc_interp_t *         I/O  interpreter state (OVFIND)
 *   resume_as  agc_job_id_t           I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t agc_rendz_ttogo_display_start(const agc_job_ctx_t *ctx,
                                           agc_interp_t        *ip,
                                           agc_job_id_t         resume_as)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_ttogo_display(ctx, ip, agc_rendz_erasable(), resume_as));
}

/**********************************************************************
 * GOFLASH / GOFLASHR / GOMARKF / GOXDSPF ... with the three astronaut
 * returns (call CADR +1 terminate, +2 proceed, +3 enter).
 * AGC: GOFLASH, GOFLASHR, GOMARKF, GOXDSPF (DISPLAY_INTERFACE_ROUTINES.agc)
 *
 * FUNCTION NAME: rz_flash
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   kind      agc_dsky_disp_t        I    display routine (GOFLASH,
 *                                         GOFLASHR, GOMARKF, PRIODSP)
 *   verb      uint8_t                I    verb of the display
 *   noun      uint8_t                I    noun of the display
 *   term      agc_job_id_t           I    job for TERMINATE (V34)
 *   proceed   agc_job_id_t           I    job for PROCEED (V33)
 *   enter     agc_job_id_t           I    job for ENTER (data loaded)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint8_t verb, uint8_t noun, agc_job_id_t term, agc_job_id_t proceed, agc_job_id_t enter)
{
    if (!AGC_ASSERT(verb < 100U && noun < 100U) || !AGC_ASSERT((int) kind >= 0 && kind < AGC_DSKY_DISP_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_cont_t cont; /* astronaut returns */

    cont.terminate = term;
    cont.proceed   = proceed;
    cont.enter     = enter;

    return (agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont));
}

/**********************************************************************
 * PRIOLARM (DISPLAY_INTERFACE_ROUTINES): the alarm, then V05N09 as a
 * priority display with the three astronaut returns.
 * AGC: PRIOLARM (DISPLAY_INTERFACE_ROUTINES.agc)
 *
 * FUNCTION NAME: rz_prio_alarm
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   code      agc_alarm_code_t       I    alarm code
 *   term      agc_job_id_t           I    job for TERMINATE (V34)
 *   proceed   agc_job_id_t           I    job for PROCEED (V33)
 *   enter     agc_job_id_t           I    job for ENTER (data loaded)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t rz_prio_alarm(const agc_job_ctx_t *ctx, agc_alarm_code_t code, agc_job_id_t term, agc_job_id_t proceed, agc_job_id_t enter)
{
    agc_prioalarm(code);

    return (rz_flash(ctx, AGC_DSKY_PRIODSP, 5U, 9U, term, proceed, enter));
}

/**********************************************************************
 * COMPTGO (CM P30-P37 p.636; LM P32-P35_P72-P75): start CLOKTASK (TTOGO
 * clock display) and protect it: 2PHSCHNG OCT 40036 (6.3SPOT CLOKTASK),
 * OCT 05024 + OCT 13000 (group 4 type C, PRIO13 job at the return). The
 * CM also sets TIMRFLAG and clears NVWORD1, and waits 1 cs; the LM clears
 * DISPDEX and waits 2 cs. DEVIATION: the CM routine of guid-powered
 * (agc_csm_comptgo) hard-wires its own P30 return job, so the rendezvous
 * programs use this copy with their own return job `cont`.
 * AGC: COMPTGO (P30-P37.agc p.636)
 *
 * FUNCTION NAME: rz_comptgo
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------
 *   cont      agc_job_id_t  I    job restarted at the return (PRIO13, group
 *                                4)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for
 *          AGC_JOB_NONE, else the waitlist status (AGC_STATUS_FULL)
 **********************************************************************/
agc_status_t rz_comptgo(agc_job_id_t cont)
{
    uint16_t delay = 2U; /* CLOKTASK delay, cs */

    if (rz_is_csm())
    {
        rz_vflag_set(RZ_VF_TIMRFLAG, true);
        agc_guid_state()->csm.nvword1 = 0;
        delay                         = 1U;
    }
    else
    {
        agc_guid_state()->lm.dispdex = 0;
    }

    agc_inhint();
    agc_status_t st; /* WAITLIST */

    st = agc_waitlist_call(delay, AGC_TASK_CLOKTASK);
    agc_phase_var_t v; /* group 4 type C job */

    v.kind       = AGC_RESTART_JOB;
    v.job        = cont;
    v.task       = AGC_TASK_NONE;
    v.prio_or_dt = 013; /* PRIO13          */
    v.immediate  = false;
    v.long_dt_cs = 0U;
    agc_phase_change2(040036U, 05024U, &v);
    agc_relint();

    return (st);
}

/**********************************************************************
 * DISPDEX = 0 (KILCLOCK, CLUPDATE). In the CM, DISPDEX EQUALS A
 * (ERASABLE_ASSIGNMENTS p.~), so "TS DISPDEX" stores nothing.
 * AGC: KILCLOCK, CLUPDATE "TS DISPDEX" (P34-35_P74-75.agc pp.481-482)
 *
 * FUNCTION NAME: rz_dispdex_zero
 *
 * ARGUMENTS: none
 **********************************************************************/
void rz_dispdex_zero(void)
{
    if (!rz_is_csm())
    {
        agc_guid_state()->lm.dispdex = 0;
    }
}

/**********************************************************************
 * OPTION1 / OPTION2 (DSPTEMX = DSPTEM2 +1 at index 4, +2 at index 5).
 * AGC: OPTION1, OPTION2 (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rz_option
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   which     int   I    1 (OPTION1) or 2 (OPTION2)
 *
 * RETURNS: uint16_t -- the option word (0 for a bad index)
 **********************************************************************/
uint16_t rz_option(int which)
{
    return (agc_dsky_dsptem(3 + which));
}

/**********************************************************************
 * Store OPTION1 / OPTION2 for an option display.
 * AGC: OPTION1, OPTION2 (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rz_set_option
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   which     int       I    1 (OPTION1) or 2 (OPTION2)
 *   v         uint16_t  I    the option word, 15 bits
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for a bad
 *          index or word, else the DSKY status
 **********************************************************************/
agc_status_t rz_set_option(int which, uint16_t v)
{
    if (!AGC_ASSERT(which == 1 || which == 2) || !AGC_ASSERT(v <= 077777U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_set_dsptem(3 + which, v);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * A raw 15-bit ones' complement word as a signed SP value.
 * AGC: none (word conversion)
 *
 * FUNCTION NAME: rz_word_to_sp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   w         uint16_t  I    raw 15-bit ones' complement word
 *
 * RETURNS: agc_sp_t -- the signed SP value (0 for a word over 15 bits)
 **********************************************************************/
static agc_sp_t rz_word_to_sp(uint16_t w)
{
    if ((w & 040000U) != 0U)
    {
        return ((agc_sp_t) (-(agc_sp_t) ((uint16_t) ~w & 037777U)));
    }

    return ((agc_sp_t) w);
}

/**********************************************************************
 * A signed SP value as a raw 15-bit ones' complement word.
 * AGC: none (word conversion)
 *
 * FUNCTION NAME: rz_sp_to_word
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   v         agc_sp_t  I    signed SP value
 *
 * RETURNS: uint16_t -- the raw 15-bit ones' complement word
 **********************************************************************/
static uint16_t rz_sp_to_word(agc_sp_t v)
{
    if (v < 0)
    {
        return ((uint16_t) (~(uint16_t) (-v) & 077777U));
    }

    return ((uint16_t) v);
}

/**********************************************************************
 * A DP in the display temporaries: index 0 = DSPTEM1, 4 = DSPTEMX.
 * AGC: DSPTEM1, DSPTEMX (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rz_dsptem_dp
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   index     int   I    RZ_DSPTEM1 or RZ_DSPTEMX
 *
 * RETURNS: agc_dp_t -- the DP in the display temporaries (0 for a bad
 *          index)
 **********************************************************************/
agc_dp_t rz_dsptem_dp(int index)
{
    agc_sp_t hi; /* high word, 2^14 units */

    hi = rz_word_to_sp(agc_dsky_dsptem(index));
    agc_sp_t lo; /* low word, unscaled */

    lo = rz_word_to_sp(agc_dsky_dsptem(index + 1));

    return (agc_dp_pack(hi, lo));
}

/**********************************************************************
 * Store a DP in the display temporaries: index 0 = DSPTEM1, 4 = DSPTEMX.
 * AGC: DSPTEM1, DSPTEMX (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rz_set_dsptem_dp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   index     int       I    RZ_DSPTEM1 or RZ_DSPTEMX
 *   v         agc_dp_t  I    the DP (a time, cs 2^28)

 **********************************************************************/
void rz_set_dsptem_dp(int index, agc_dp_t v)
{
    agc_dsky_set_dsptem(index, rz_sp_to_word(agc_dp_high(v)));
    agc_dsky_set_dsptem(index + 1, rz_sp_to_word(agc_dp_low(v)));
}
