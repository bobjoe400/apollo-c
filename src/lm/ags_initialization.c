/**********************************************************************
 * FILE NAME: ags_initialization.c
 *
 * PURPOSE:
 *   R47 AGS initialization (V47): the LM and CSM states in AEA units
 *   (SCALEVEC, the AGSRND rounding to the major words), the time since
 *   the AGS clock zero AGSK, the AGS downlist for 20 s; V06N16 shows or
 *   zeroes AGSK.
 *
 * AGC SOURCE:
 *   Luminary099 AGS_INITIALIZATION.agc pp. 206-210 (AGSINIT, NEWAGS, REDSPTEM, AGSDISPK, AGSVCALC, CKSTALL, AGSEND, SCALEVEC)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   AGSBUFF       uint16_t  O    AEA words      this file (agc_ags_buffer())
 *   AGSK          agc_dp_t  I/O  cs 2^28        AGS clock zero (agc_ags_agsk())
 *   REFSMMAT      agc_mat_t I    half-unit      agc_erase_refsmmat()
 *   DSPTEMX, SAMPTIME  agc_dp_t I  cs 2^28       the keyed or mark time
 *   IMUCADR       agc_sp_t  I    unscaled       CKSTALL
 *   NODOFLAG      flag      I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_endmark      DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_svc_delayjob      service routines: DELAYJOB, flags, B5OFF (service.h)
 *   agc_imu_*             IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_mat_vec           matrix arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_init       interpreter state: OVFIND (interp.h)
 *   agc_flag_*            flag words (flags_common.h)
 *   agc_erase_*           erasable accessors (state.h)
 *   agc_alarm             ALARM (exec.h)
 *   agc_telem_select_list  downlink list selection (telemetry.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarms: ALM_NO_REFSMMAT 0220: OCT 220: no REFSMMAT.
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   A negative component under one major-word LSB gives -0 (77777), as
 *   the AGC's two-step add does (finding A3).
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define ALM_NO_REFSMMAT ((agc_alarm_code_t) 0220) /* OCT 220: no REFSMMAT */
#define VN_AGS_TIME     ALIGN_VN(6, 16)           /* V06N16 VN 0616                */
#define VN_AGS_DONE     ALIGN_VN(50, 16)          /* V50N16 VN 5016                */
#define LAGSLIST        1                         /* LAGSLIST = ONE (AGS downlist) */
#define DT_DOWNLIST     ((uint16_t) 2000)         /* 20SEC DEC 2000                */
#define DT_RETRY        ((uint16_t) 10)           /* TEN cs                        */

/* RSCALE 2DEC 3.280839 B-3 (m -> ft) and VSCALE 2DEC 3.280839 E2 B-9
 * (m/cs -> ft/s): the words assembled in the Luminary 099 rope (15077 05041,
 * 24402 25724), 30 and 15 LSB from the exact decimal conversion.          */
#define RSCALE ((agc_dp_t) 110086689) /* 3.280839 B-3, ft/m              */
#define VSCALE ((agc_dp_t) 172010452) /* 328.0839 B-9, (ft/s)/(m/cs)     */
#define TSCALE ((agc_dp_t) 26214400)  /* TSCALE 2DEC 100 B-10          */

/* AGSRND1 (2OCT 0000060000) + AGSRND2 (2OCT 0000037777): half a high-word
 * LSB, added in two steps; here as one DP addend.                        */
#define AGS_ROUND      ((agc_dp_t) 8192)   /* AGSRND1 + AGSRND2, half a major-word LSB */
#define AGSBUFF_TIME   12                  /* AGSBUFF +12D: the DP time     */
#define AGS_MINUS_ZERO ((uint16_t) 077777) /* -0 high word (AEA: -1)  */

static uint16_t agsbuff[AGC_ALIGN_AGSBUFF_WORDS]; /* AGSBUFF: RX, VX, RY, VY, RZ, VZ of LM then CSM, time (raw words) */
static agc_dp_t agsk;                             /* AGSK: AGS clock zero, cs B28  */

/**********************************************************************
 * AGC: none (accessor of AGSBUFF, Luminary099 AGS_INITIALIZATION.agc p.
 *      209)
 *
 * AGSBUFF, the 14 words sent to the AEA.
 *
 * FUNCTION NAME: agc_ags_buffer
 *
 * ARGUMENTS: none
 *
 * RETURNS: const uint16_t * -- AGSBUFF, read only
 **********************************************************************/
const uint16_t *agc_ags_buffer(void)
{
    _Static_assert(AGC_ALIGN_AGSBUFF_WORDS == 14, "AGC constant");

    return (agsbuff);
}

/**********************************************************************
 * AGC: none (accessor of AGSK)
 *
 * AGSK, the AGS clock zero.
 *
 * FUNCTION NAME: agc_ags_agsk
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- AGSK, cs 2^28
 **********************************************************************/
agc_dp_t agc_ags_agsk(void)
{
    return (agsk);
}

/**********************************************************************
 * VECSGNAG and the high words: v + half an LSB, the high word of each
 * component as the raw 15-bit word the AGS reads (2's complement).  The
 * AGC's two-step add (AGSRND1 then AGSRND2) leaves a negative sum smaller
 * than one high-word LSB as (-1, +low), which VECSGNAG turns into -0
 * (77777): the AEA reads -1, i.e. the sum rounded down.  A single add would
 * leave (+0, -low) and read 0; that is what the comment on p. 209 ("defeat
 * ALSIGNAG") is about.
 *
 * FUNCTION NAME: ags_words
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   v         agc_vec_t       I    the scaled vector (R: ft 2^23, V: ft/s
 *                                  2^13)
 *   w         uint16_t        O    the three major words as the AEA reads them
 *
 * RETURNS: none
 **********************************************************************/
static void ags_words(agc_interp_t *ip, agc_vec_t v, uint16_t w[3])
{
    for (int i = 0; i < 3; i++)
    {
        const agc_dp_t r  = agc_dp_add(ip, v.c[i], AGS_ROUND); /* component + half a major-word LSB, scale as v (2^23 ft or 2^13 ft/s) */
        const agc_sp_t hi = agc_dp_high(r);                    /* its sign-agreeing major word, scale 2^14 LSB of v */

        w[i] = align_sp_to_word(hi);

        if ((r < 0) && (hi == 0))
        {
            w[i] = AGS_MINUS_ZERO;
        }
    }
}

/**********************************************************************
 * SCALEVEC (p. 209): V = REFSMMAT VATT1 x VSCALE, VSL2; R = REFSMMAT RATT1
 * x RSCALE, VSL8; each rounded.  With the half-unit REFSMMAT (MXV keeps its
 * B1) and moon-sphere states (RATT1 m B27, VATT1 m/cs B5) the high words
 * are R in 512 ft (2^23 ft full scale) and V in 0.5 ft/s (2^13 ft/s). out =
 * RX, VX, RY, VY, RZ, VZ (the high words; VATT1's go into R's low-word
 * places).
 *
 * FUNCTION NAME: agc_ags_scalevec
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     IO   interpreter state (OVFIND, MPAC
 *                                     words)
 *   ratt1     agc_vec_t          I    RATT1, m 2^27 (moon sphere)
 *   vatt1     agc_vec_t          I    VATT1, m/cs 2^5 (moon sphere)
 *   refsmmat  const agc_mat_t *  I    REFSMMAT, half-unit rows (reference
 *                                     -> present SM)
 *   out       uint16_t           O    RX, VX, RY, VY, RZ, VZ major words
 *
 * RETURNS: none
 **********************************************************************/
void agc_ags_scalevec(agc_interp_t *ip, agc_vec_t ratt1, agc_vec_t vatt1, const agc_mat_t *refsmmat, uint16_t out[6])
{
    const agc_vec_t v = agc_vec_shl(ip, agc_vec_scale(ip, agc_mat_vec(ip, refsmmat, vatt1), VSCALE), 2); /* REFSMMAT VATT1 VSCALE VSL2, ft/s 2^13 */
    const agc_vec_t r = agc_vec_shl(ip, agc_vec_scale(ip, agc_mat_vec(ip, refsmmat, ratt1), RSCALE), 8); /* REFSMMAT RATT1 RSCALE VSL8, ft 2^23 */
    uint16_t        rw[3];                                                                               /* R high words (AEA format) */
    uint16_t        vw[3];                                                                               /* V high words (AEA format) */

    ags_words(ip, v, vw);
    ags_words(ip, r, rw);

    for (int i = 0; i < 3; i++)
    {
        out[2 * i]       = rw[i];
        out[(2 * i) + 1] = vw[i];
    }
}

/**********************************************************************
 * AGC: AGSBUFF (Luminary099 ERASABLE_ASSIGNMENTS.agc p. 117)
 *
 * AGSBUFF +12D: (TAT - AGSK) / TSCALE, seconds B18.
 *
 * FUNCTION NAME: agc_ags_time
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   tat       agc_dp_t        I    TAT, state time, cs 2^28
 *   k         agc_dp_t        I    AGSK, AGS clock zero, cs 2^28
 *
 * RETURNS: agc_dp_t -- (TAT - AGSK) / TSCALE, s 2^18
 **********************************************************************/
agc_dp_t agc_ags_time(agc_interp_t *ip, agc_dp_t tat, agc_dp_t k)
{
    return (agc_dp_div(ip, agc_dp_sub(ip, tat, k), TSCALE));
}

/**********************************************************************
 * AGC: REDSPTEM (Luminary099 AGS_INITIALIZATION.agc p. 207)
 *
 * REDSPTEM / AGSDISPK: DSPTEMX = AGSK; GOMARKF V06N16.
 *
 * FUNCTION NAME: show_agsk
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
static agc_status_t show_agsk(agc_job_ctx_t *ctx)
{
    agc_erase_set_dsptemx(agsk);

    return (align_flash(ctx, AGC_DSKY_GOMARKF, VN_AGS_TIME, AGC_JOB_AGS_INITIALIZATION_END, AGC_JOB_AGS_INITIALIZATION_COMPUTE, AGC_JOB_AGS_INITIALIZATION_NEW_ZERO));
}

/**********************************************************************
 * AGC: AGSINIT (Luminary099 AGS_INITIALIZATION.agc p. 206)
 *
 * AGSINIT: alarm 220 and ENDEXT without a REFSMMAT; else REDSPTEM.
 *
 * FUNCTION NAME: agc_job_ags_initialization
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
agc_status_t agc_job_ags_initialization(agc_job_ctx_t *ctx)
{
    if (!agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        agc_alarm(ALM_NO_REFSMMAT);

        return (agc_dsky_endmark()); /* TC ALARM returns; TC ENDEXT */
    }

    return (show_agsk(ctx));
}

/**********************************************************************
 * AGC: V06N16 (Luminary099 AGS_INITIALIZATION.agc p. 209)
 *
 * V06N16 ENTER: V32 (NEWAGS) makes the time of the ENTER keystroke
 * (SAMPTIME) the new AGSK; otherwise the AGSK keyed in (V25) is taken from
 * DSPTEMX; shown again.
 *
 * FUNCTION NAME: agc_job_ags_initialization_new_zero
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
agc_status_t agc_job_ags_initialization_new_zero(agc_job_ctx_t *ctx)
{
    agsk = align_resp_recycle() ? agc_erase_samptime() : agc_erase_dsptemx(); /* V32: SAMPTIME (mark time), else the keyed DSPTEMX */

    return (show_agsk(ctx));
}

/**********************************************************************
 * AGC: AGSVCALC (Luminary099 AGS_INITIALIZATION.agc p. 207)
 *
 * AGSVCALC: SET NODOFLAG (no V37), XDSPFLAG; EXDSPRET V06N16.
 *
 * FUNCTION NAME: agc_job_ags_initialization_compute
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
agc_status_t agc_job_ags_initialization_compute(agc_job_ctx_t *ctx)
{
    agc_flag_set(AGC_FLAG_NODOFLAG);
    agc_flag_set(AGC_FLAG_XDSPFLAG);

    return (align_flash(ctx, AGC_DSKY_EXDSPRET, VN_AGS_TIME, AGC_JOB_AGS_INITIALIZATION_LM_STATE, AGC_JOB_AGS_INITIALIZATION_LM_STATE, AGC_JOB_AGS_INITIALIZATION_LM_STATE));
}

/* LEMPREC to now; SCALEVEC -> AGSBUFF; TAT kept for the CSM. */
static agc_dp_t ags_tat; /* TAT: the state time for the CSM integration, cs 2^28 */

/**********************************************************************
 * AGC: AGSVCALC (Luminary099 AGS_INITIALIZATION.agc p. 208)
 *
 * LEMPREC to now; SCALEVEC -> AGSBUFF; TAT kept for the CSM.
 *
 * FUNCTION NAME: agc_job_ags_initialization_lm_state
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_ags_initialization_lm_state(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_integ_out_t o;                                                                                               /* integration output (RATT, VATT) */
    agc_status_t    st = align_integ(&ip, ALIGN_LEMPREC, align_loadtime(), AGC_JOB_AGS_INITIALIZATION_LM_STATE, &o); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* INTSTALL */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */

    agc_ags_scalevec(&ip, o.ratt1, o.vatt1, &refsmmat, &agsbuff[0]);
    ags_tat = o.tat;

    return (agc_job_ags_initialization_csm_state());
}

/**********************************************************************
 * AGC: CSMPREC (Luminary099 INTEGRATION_INITIALIZATION.agc p. 1214)
 *
 * CSMPREC to TAT; SCALEVEC -> AGSBUFF +6; AGSBUFF +12D = time since AGSK;
 * the AGS downlist (LAGSLIST) for 20 s.
 *
 * FUNCTION NAME: agc_job_ags_initialization_csm_state
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_ags_initialization_csm_state(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_integ_out_t o;                                                                                       /* integration output (RATT, VATT) */
    agc_status_t    st = align_integ(&ip, ALIGN_CSMPREC, ags_tat, AGC_JOB_AGS_INITIALIZATION_CSM_STATE, &o); /* status of the call */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */

    agc_ags_scalevec(&ip, o.ratt1, o.vatt1, &refsmmat, &agsbuff[6]);
    const agc_dp_t t = agc_ags_time(&ip, ags_tat, agsk); /* (TAT - AGSK) / TSCALE, s 2^18 */

    agsbuff[AGSBUFF_TIME]     = align_sp_to_word(agc_dp_high(t));
    agsbuff[AGSBUFF_TIME + 1] = align_sp_to_word(agc_dp_low(t));
    agc_telem_select_list(LAGSLIST); /* DNLSTCOD */

    return (agc_svc_delayjob(DT_DOWNLIST, AGC_JOB_AGS_INITIALIZATION_SENT));
}

/**********************************************************************
 * AGC: AGSEND (Luminary099 AGS_INITIALIZATION.agc p. 208)
 *
 * AGSEND: DOWNFLAG NODOFLAG; GOMARK3 V50N16; every answer ENDEXT.
 *
 * FUNCTION NAME: ags_end
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
static agc_status_t ags_end(const agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_NODOFLAG);

    return (align_flash(ctx, AGC_DSKY_GOMARK3, VN_AGS_DONE, AGC_JOB_AGS_INITIALIZATION_END, AGC_JOB_AGS_INITIALIZATION_END, AGC_JOB_AGS_INITIALIZATION_END));
}

/**********************************************************************
 * AGC: CKSTALL (Luminary099 AGS_INITIALIZATION.agc p. 208)
 *
 * CKSTALL: wait (0.1 s at a time) while the IMU is otherwise busy (IMUCADR
 * non-zero), then IMUZERO and IMUSTALL; either end goes to AGSEND.
 *
 * FUNCTION NAME: agc_job_ags_initialization_zero_imu
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
agc_status_t agc_job_ags_initialization_zero_imu(agc_job_ctx_t *ctx)
{
    if (agc_erase_imucadr() != 0)
    {
        return (agc_svc_delayjob(DT_RETRY, AGC_JOB_AGS_INITIALIZATION_ZERO_IMU));
    }

    agc_status_t st = agc_imu_zero(); /* IMUZERO  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_imu_stall_result_t r = AGC_IMU_STALL_BAD; /* IMUSTALL outcome */

    st = agc_imu_stall_dev(AGC_IMU_DEV_IMU, AGC_JOB_AGS_INITIALIZATION_IMU_ZEROED, &r);

    if ((st != AGC_STATUS_OK) || (r == AGC_IMU_STALL_SLEEPING))
    {
        return (st);
    }

    return (ags_end(ctx));
}

/**********************************************************************
 * AGC: AGSWORD (Luminary099 ERASABLE_ASSIGNMENTS.agc p. 107)
 *
 * After the 20 s: the old downlist (AGSWORD); IMUSE set -> AGSEND.
 *
 * FUNCTION NAME: agc_job_ags_initialization_sent
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
agc_status_t agc_job_ags_initialization_sent(agc_job_ctx_t *ctx)
{
    agc_telem_select_list((int) agc_erase_agsword());

    if (agc_flag_test(AGC_FLAG_IMUSE))
    {
        return (ags_end(ctx));
    }

    return (agc_job_ags_initialization_zero_imu(ctx));
}

/**********************************************************************
 * AGC: IMUSTALL (Luminary099 IMU_MODE_SWITCHING_ROUTINES.agc p. 1330)
 *
 * IMUSTALL woken (good or bad end): AGSEND.
 *
 * FUNCTION NAME: agc_job_ags_initialization_imu_zeroed
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
agc_status_t agc_job_ags_initialization_imu_zeroed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_imu_stall_outcome(AGC_IMU_DEV_IMU) != AGC_IMU_STALL_SLEEPING))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (ags_end(ctx));
}

/**********************************************************************
 * AGC: ENDEXT (Luminary099 DISPLAY_INTERFACE_ROUTINES.agc p. 1372)
 *
 * ENDEXT (= ENDMARK): the extended verb is over.  NODOFLAG is left as it
 * is: AGSEND has already cleared it, and V34 on the first V06N16 comes
 * before AGSVCALC sets it.
 *
 * FUNCTION NAME: agc_job_ags_initialization_end
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_ags_initialization_end(void)
{
    return (agc_dsky_endmark());
}
