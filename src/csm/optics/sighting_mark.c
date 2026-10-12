/**********************************************************************
 * FILE NAME: sighting_mark.c
 *
 * PURPOSE:
 *   The CSM sighting routines of P51-P53:
 *
 *   R53    sighting mark: SXTMARK + OPTSTALL, then the star code (V01N71)
 *          is checked (CHKSCODE) and kept in BESTI
 *   R56    alternate LOS sighting mark: a backup mark from keyed-in optics
 *          angles (V06N94, V53) when the optics cannot be used (P53, P54)
 *   SXTSM  the line of sight of the first mark in stable-member axes
 *
 *   Both R53 and R56 are interpretive subroutines entered with STQ
 *   R53EXIT; here the caller names the job to continue as, kept in
 *   agc_align_ram()->sighting_mark_return, and the routine ends with
 *   TERMIFLG set (R53OUT) so that R52 stops driving the optics.
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 770-773 (SXTSM, R53 .. R53OUT), 781-782 (R56)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   MARKINDX, STARCODE, STARIND, BESTI/J, SAC, PAC, TSIGHT, the mark slots: agc_align_ram()
 *   MRKBUF1       uint16_t  I    CDU words      N94 shaft / trunnion
 *   MARKDOWN, MARK2DWN uint16_t O raw words     downlink
 *   R53FLAG, TERMIFLG  flags (O)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_*            DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_guid_trg_nbsm     CDUTRIG, *NBSM* / *SMNB*, PIPASR (guidance.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_flag_set          flag words (flags_common.h)
 *   agc_phase_checkmm     restart phase tables, CHECKMM (exec.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_inhint            INHINT / RELINT (exec.h)
 *   agc_relint            INHINT / RELINT (exec.h)
 *   agc_csm_sxtnb         CSM optics and sighting routines (alignment.h)
 *   agc_sxtmark*          SXTMARK routines (alignment.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define VN_STAR_CODE    ALIGN_VN(1, 71)  /* V01N71 VN 0171: star code      */
#define VN_ALT_ANGLES   ALIGN_VN(6, 94)  /* V06N94B VN 00694: optics angles */
#define VN_ALT_MARK     ALIGN_VN(53, 0)  /* VB53 VN 05300: alternate mark  */
#define CHK_MARK_DONE   ((uint16_t) 016) /* OCT16: checklist "mark done"   */
#define STAR_BITS       ((agc_sp_t) 077) /* CS HIGH9: star bits 1-6        */
#define SIGHTSIX        ((agc_sp_t) 6)   /* SIGHTSIX = SIX                 */
#define MARKS_MASK      ((agc_sp_t) 07)  /* LOW3: number of marks          */
#define MARK_WORDS      7                /* MKT2T1 .. MKCDUX               */
#define MRKBUF_CDUS     3                /* MRKBUF1 +3 (N94 shaft)         */
#define MRKBUF_CDUT     5                /* MRKBUF1 +5 (N94 trunnion)      */
#define MM_ORBITAL_NAV  ((agc_sp_t) 22)  /* P22                            */
#define MM_CISLUNAR_NAV ((agc_sp_t) 23)  /* P23                            */

/**********************************************************************
 * AGC: R53A (Comanche055 P51-P53.agc p. 772)
 *
 * R53A: SXTMARK for MARKINDX marks, then OPTSTALL (waking at R53A +5).
 *
 * FUNCTION NAME: sighting_mark_request
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t sighting_mark_request(void)
{
    const agc_sp_t n  = (agc_sp_t) (agc_align_ram()->markindx & MARKS_MASK); /* MARKINDX bits 1-3: marks wanted (unscaled) */
    agc_status_t   st = agc_sxtmark(n);                                      /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* BAILOUT */
    }

    bool done = false; /* device already finished */

    st = align_stall(AGC_IMU_DEV_OPTICS, AGC_JOB_SIGHTING_MARK_MARKED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* CURTAINS */
    }

    return (align_goto(AGC_JOB_SIGHTING_MARK_MARKED));
}

/**********************************************************************
 * R53 (Comanche p. 771): STQ R53EXIT, SET R53FLAG, then R53A.
 *
 * FUNCTION NAME: agc_csm_sighting_mark
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
agc_status_t agc_csm_sighting_mark(agc_job_id_t ret)
{
    agc_align_ram()->sighting_mark_return = ret; /* R53EXIT  */
    agc_flag_set(AGC_FLAG_SIGHTING_MARK_REQUESTED);

    return (sighting_mark_request());
}

/**********************************************************************
 * AGC: R53D (Comanche055 P51-P53.agc p. 773)
 *
 * R53D / R53OUT: SET TERMIFLG (R52 stops), GOTO R53EXIT.
 *
 * FUNCTION NAME: sighting_mark_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t sighting_mark_done(void)
{
    agc_flag_set(AGC_FLAG_TERMIFLG);

    return (align_goto(agc_align_ram()->sighting_mark_return));
}

/**********************************************************************
 * AGC: R53A (Comanche055 P51-P53.agc p. 772)
 *
 * R53A after OPTSTALL: QPRET (marks made) > 0 goes on to R53B; with no mark
 * the VAC area and MARKSTAT are released and SXTMARK is asked again. R53B:
 * P22 and P23 need no star code (R53D); otherwise R53C1.
 *
 * FUNCTION NAME: agc_job_sighting_mark_marked
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_sighting_mark_marked(agc_job_ctx_t *ctx)
{
    const agc_status_t woke = align_stall_outcome(AGC_IMU_DEV_OPTICS); /* outcome of the stall */

    if (woke != AGC_STATUS_OK)
    {
        return ((woke == AGC_STATUS_ALARM) ? AGC_STATUS_OK : woke); /* CURTAINS */
    }

    if (agc_sxtmark_marks_made() <= 0)
    {
        const agc_status_t st = agc_sxtmark_free_vac(); /* XCH MARKSTAT */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        return (sighting_mark_request()); /* TCF R53A */
    }

    if (agc_phase_checkmm(MM_ORBITAL_NAV) || agc_phase_checkmm(MM_CISLUNAR_NAV))
    {
        return (sighting_mark_done()); /* R53D     */
    }

    return (agc_job_sighting_mark_code_clean(ctx));
}

/**********************************************************************
 * AGC: R53C1 (Comanche055 P51-P53.agc p. 773)
 *
 * R53C1: CAF ZERO, CLEANDSP; the job continues at R53C.
 *
 * FUNCTION NAME: agc_job_sighting_mark_code_clean
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_sighting_mark_code_clean(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_CLEANDSP, 0, AGC_JOB_SIGHTING_MARK_CODE_DISPLAY, AGC_JOB_SIGHTING_MARK_CODE_DISPLAY, AGC_JOB_SIGHTING_MARK_CODE_DISPLAY));
}

/**********************************************************************
 * AGC: R53C (Comanche055 P51-P53.agc p. 773)
 *
 * R53C: flash V01N71; V34 terminates, V33 R53Z, ENTER re-flashes.
 *
 * FUNCTION NAME: agc_job_sighting_mark_code_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_sighting_mark_code_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_STAR_CODE, AGC_JOB_GOTOPOOH, AGC_JOB_SIGHTING_MARK_CODE_ENTERED, AGC_JOB_SIGHTING_MARK_CODE_DISPLAY));
}

/**********************************************************************
 * AGC: R53Z (Comanche055 P51-P53.agc p. 773)
 *
 * R53Z: CHKSCODE; a bad code lights OPR ERR and shows V01N71 again.  A good
 * one: BESTI +STARIND = 6 x (STARCODE bits 1-6), then R53D.
 *
 * FUNCTION NAME: agc_job_sighting_mark_code_entered
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_sighting_mark_code_entered(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    if (!agc_align_chkscode(ram->starcode))
    {
        agc_dsky_falton(); /* FALTON   */

        return (agc_job_sighting_mark_code_display(ctx));
    }

    const agc_sp_t star6 = (agc_sp_t) ((ram->starcode & STAR_BITS) * SIGHTSIX); /* star code x 6 (unscaled) */

    if (ram->starind == 0)
    {
        ram->besti = star6; /* INDEX STARIND */
    }
    else
    {
        ram->bestj = star6;
    }

    return (sighting_mark_done());
}

/**********************************************************************
 * R56 (Comanche p. 781): STQ R53EXIT, then flash V06N94 (the optics angles
 * of the alternate line of sight).
 *
 * FUNCTION NAME: agc_csm_alternate_los_sighting_mark
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *   ret       agc_job_id_t           I    job the routine continues as when
 *                                         it ends (the AGC return address)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_csm_alternate_los_sighting_mark(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    agc_align_ram()->sighting_mark_return = ret; /* R53EXIT  */

    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALT_ANGLES, AGC_JOB_GOTOPOOH, AGC_JOB_ALTERNATE_LOS_RESERVE, AGC_JOB_ALTERNATE_LOS_ANGLES_DISPLAY));
}

/**********************************************************************
 * AGC: R56 (Comanche055 P51-P53.agc p. 781)
 *
 * R56 ENTER on V06N94: new angles, flash again.
 *
 * FUNCTION NAME: agc_job_alternate_los_angles_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_alternate_los_angles_display(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOFLASH, VN_ALT_ANGLES, AGC_JOB_GOTOPOOH, AGC_JOB_ALTERNATE_LOS_RESERVE, AGC_JOB_ALTERNATE_LOS_ANGLES_DISPLAY));
}

/**********************************************************************
 * AGC: R56A (Comanche055 P51-P53.agc p. 781)
 *
 * R56A +2: CLEANDSP, continuing at R56A1.
 *
 * FUNCTION NAME: alternate_los_clean
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ctx       const agc_job_ctx_t *  I    the running job (display
 *                                         continuations)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t alternate_los_clean(const agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_CLEANDSP, 0, AGC_JOB_ALTERNATE_LOS_MARK_REQUEST, AGC_JOB_ALTERNATE_LOS_MARK_REQUEST, AGC_JOB_ALTERNATE_LOS_MARK_REQUEST));
}

/**********************************************************************
 * AGC: R56A (Comanche055 P51-P53.agc p. 781)
 *
 * R56A (V33 on V06N94): SXTMARK +2 (marks system and a VAC area only).
 *
 * FUNCTION NAME: agc_job_alternate_los_reserve
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_alternate_los_reserve(agc_job_ctx_t *ctx)
{
    const agc_status_t st = agc_sxtmark_reserve(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_ALARM) ? AGC_STATUS_OK : st); /* BAILOUT */
    }

    return (alternate_los_clean(ctx));
}

/**********************************************************************
 * AGC: R56A1 (Comanche055 P51-P53.agc p. 781)
 *
 * R56A1: GOMARK2 VB53 (please make the alternate mark); V33 shows it again,
 * ENTER is the mark.
 *
 * FUNCTION NAME: agc_job_alternate_los_mark_request
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_alternate_los_mark_request(agc_job_ctx_t *ctx)
{
    return (align_flash(ctx, AGC_DSKY_GOMARK2, VN_ALT_MARK, AGC_JOB_GOTOPOOH, AGC_JOB_ALTERNATE_LOS_MARK_REQUEST, AGC_JOB_ALTERNATE_LOS_MARKED));
}

/**********************************************************************
 * AGC: R56A1 (Comanche055 P51-P53.agc p. 781)
 *
 * R56A1 ENTER (a backup system mark): SAC = MRKBUF1 +3, PAC = MRKBUF1 +5
 * (the N94 angles); under INHINT the mark slot gets TIME2/1, CDUY, SAC,
 * CDUZ, PAC, CDUX.  CLEARMRK, then V50N25 R1 00016: PROCEED goes to R53C1
 * (star code), ENTER marks again (R56A +2).
 *
 * FUNCTION NAME: agc_job_alternate_los_marked
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_alternate_los_marked(agc_job_ctx_t *ctx)
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    ram->sac = agc_erase_mrkbuf(1, MRKBUF_CDUS);
    ram->pac = agc_erase_mrkbuf(1, MRKBUF_CDUT);
    agc_inhint();
    agc_align_sxt_mark_t m; /* the mark */

    m.t         = align_loadtime();
    m.cduy      = agc_erase_cduy();
    m.cdus      = ram->sac;
    m.cduz      = agc_erase_cduz();
    m.cdut      = ram->pac;
    m.cdux      = agc_erase_cdux();
    ram->sxt[0] = m;     /* INDEX MARKSTAT TS */
    agc_relint();
    agc_dsky_clearmrk(); /* CLEARMRK */

    return (align_flash(ctx, AGC_DSKY_GOPERF1, CHK_MARK_DONE, AGC_JOB_GOTOPOOH, AGC_JOB_SIGHTING_MARK_CODE_CLEAN, AGC_JOB_ALTERNATE_LOS_REMARK));
}

/**********************************************************************
 * AGC: R56A (Comanche055 P51-P53.agc p. 781)
 *
 * R56A +2 (V32 on V50N25: another mark, like a reject).
 *
 * FUNCTION NAME: agc_job_alternate_los_remark
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_alternate_los_remark(agc_job_ctx_t *ctx)
{
    return (alternate_los_clean(ctx));
}

/**********************************************************************
 * SXTSM line of sight (Comanche p. 770): SXTNB of the mark's trunnion and
 * shaft CDUs (half-unit, nav base), then NBSM with the mark's IMU CDUs
 * (CDUY, CDUZ, CDUX at words 2, 4, 6 of the mark): half-unit, SM axes.
 *
 * FUNCTION NAME: agc_csm_sxtsm
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ------------------------
 *   ip        agc_interp_t *                IO   interpreter state (OVFIND,
 *                                                MPAC words)
 *   m         const agc_align_sxt_mark_t *  I    the stored mark
 *   los_sm    agc_vec_t *                   O    line of sight in SM axes,
 *                                                half-unit (2^1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_csm_sxtsm(agc_interp_t *ip, const agc_align_sxt_mark_t *m, agc_vec_t *los_sm)
{
    agc_vec_t    v;                                            /* SXTNB line of sight (NB), then SM, half-unit (2^1) */
    agc_status_t st = agc_csm_sxtnb(ip, m->cdut, m->cdus, &v); /* SXTNB */

    if (st == AGC_STATUS_OK)
    {
        const uint16_t yzx[3] = {m->cduy, m->cduz, m->cdux};
        agc_cdutrig_t  t; /* CDUTRIG sines and cosines */

        st = agc_guid_trg_nbsm(&t, yzx, &v); /* NBSM  */

        if (st == AGC_STATUS_OK)
        {
            *los_sm = v;
        }
    }

    return (st);
}

/**********************************************************************
 * AGC: MKT2T1 (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 73)
 *
 * The mark as the 7 downlink words (MKT2T1 .. MKCDUX).
 *
 * FUNCTION NAME: mark_words
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ------------------------
 *   m         const agc_align_sxt_mark_t *  I    the stored mark
 *   w         uint16_t                      O    the 7 raw words MKT2T1 .. MKCDUX
 *
 * RETURNS: none
 **********************************************************************/
static void mark_words(const agc_align_sxt_mark_t *m, uint16_t w[MARK_WORDS])
{
    w[0] = align_sp_to_word(agc_dp_high(m->t));
    w[1] = align_sp_to_word(agc_dp_low(m->t));
    w[2] = m->cduy;
    w[3] = m->cdus;
    w[4] = m->cduz;
    w[5] = m->cdut;
    w[6] = m->cdux;
}

/**********************************************************************
 * SXTSM (Comanche p. 770) as called: TSIGHT = time of the first mark; the 7
 * mark words go to MARKDOWN (STARIND 0) or MARK2DWN (STARIND 1) for the
 * downlink; the line of sight in SM axes.
 *
 * FUNCTION NAME: agc_csm_sxtsm_marks
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   los_sm    agc_vec_t *     O    line of sight in SM axes, half-unit
 *                                  (2^1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_csm_sxtsm_marks(agc_interp_t *ip, agc_vec_t *los_sm)
{
    agc_align_ram_t           *ram = agc_align_ram(); /* alignment erasables */
    const agc_align_sxt_mark_t m   = ram->sxt[0];     /* the mark */

    ram->tsight = m.t;      /* TSIGHT   */
    uint16_t w[MARK_WORDS]; /* MKT2T1 .. MKCDUX as raw words */

    mark_words(&m, w);

    for (int i = 0; i < MARK_WORDS; i++)
    {
        if (ram->starind == 0)
        {
            agc_erase_set_markdown(i, w[i]);
        }
        else
        {
            agc_erase_set_mark2dwn(i, w[i]);
        }
    }

    return (agc_csm_sxtsm(ip, &m, los_sm));
}
