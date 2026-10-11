/**********************************************************************
 * FILE NAME: stable_orbit.c
 *
 * PURPOSE:
 *   STABLE ORBIT RENDEZVOUS (P38/P78) and STABLE ORBIT MIDCOURSE
 *   (P39/P79), both vehicles. "P38 and P78 calculate the required delta
 *   V and other initial conditions required by the AGC to (1) put the
 *   active vehicle on a transfer trajectory that intercepts the passive
 *   vehicle orbit a given distance, delta R, either ahead of or behind
 *   the passive vehicle and (2) actually place the active vehicle in the
 *   passive vehicle orbit with a delta R separation between the two
 *   vehicles." Option 1 is the SOI maneuver, option 2 the SOR maneuver.
 *   "P39 and P79 calculate the required delta V ... to make a midcourse
 *   correction maneuver after completing the SOI maneuver but before
 *   making the SOR maneuver."
 *
 * AGC SOURCE:
 *   Comanche055 STABLE_ORBIT.agc pp. 525-532 (P38, P78, OPTN1, JUNCTN1,
 *   RECYCLE, JUNCTN2, MAINRTNE, DSPLY81, P39, P79, P39/P79A, P39/P79B,
 *   DELTTIME, PREC/TT, INTRPVP)
 *   Luminary099 STABLE_ORBIT.agc (same text; the LM sets NN = 2 at entry
 *   and shows TINT on V06N34 after the delta R display)
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   TIG         agc_dp_t   I/O  cs 2^28    maneuver time (N33)
 *   OPTNSW UPDATFLG FINALFLG XDELVFLG P39/79SW  flags  I/O  -  control
 *   OPTION1 OPTION2  agc_sp_t  O  -          V04N06 option code
 *   DSPTEM1     agc_dp_t   O    cs 2^28    TINT for display
 *   CENTANG DELTAR (= TINTSOI) TPASS4 (= TINT) TARGTIME DELTTIME NN KT
 *               (agc_rendz_t) I/O        program state (N55 N57 N34)
 *   DELVTPI DELVTPF POSTTPI DELVSIN (agc_rendz_t) O   N58 / N81 results
 *
 * EXTERNAL REFERENCES:
 *   Name                          Description
 *   ----------------------------  ------------------------------------
 *   rz_precset rz_integ           PRECSET, CSMPREC / LEMPREC
 *   rz_timethet_rv                TIMETHET: passive transfer time
 *   agc_rendz_los_and_normal      S34/35.1
 *   agc_rendz_intercept_velocity  S3435.25: required velocity
 *   agc_rendz_periapo_body_scale  PERIAPO1: post-maneuver apsides
 *   rz_vnpooh rz_flash            display requests
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. Jobs return AGC_STATUS_BAD_ARG for a NULL ctx, else the
 *   status of the display or integration call that failed (an INTSTALL
 *   sleep resumes the job from its start).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of the major mode. Aliases kept from ERASABLE_ASSIGNMENTS: TINT
 *   EQUALS TPASS4 and TINTSOI EQUALS DELTAR (the SOR phase overwrites
 *   DELTAR with the SOI intercept time).
 *
 * NOTES:
 *   Scales: r m 2^29, v m/cs 2^7, t cs 2^28, CENTANG revolutions.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"

/* NN = 2 offsets (SSP NN 2 / DECTWO). */
#define SO_NN_OFFSETS ((agc_sp_t) 2) /* SSP NN 2 */
/* BLANKET bits after a GOFLASHR: R1 R2 = 3, R3 = BIT3, R2 R3 = 6.       */
#define SO_BLANK_R1R2 ((uint16_t) 3) /* R1 R2 blanked */
#define SO_BLANK_R3   ((uint16_t) 4) /* R3 blanked */
#define SO_BLANK_R2R3 ((uint16_t) 6) /* R2 R3 blanked */

/**********************************************************************
 * GOFLASHR (immediate return: BLANKET, ENDOFJOB).
 * AGC: GOFLASHR, BLANKET (DISPLAY_INTERFACE_ROUTINES.agc)
 *
 * FUNCTION NAME: so_flashr
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   verb      uint8_t                I    verb of the display
 *   noun      uint8_t                I    noun of the display
 *   blank     uint16_t               I    BLANKET bits (R1 = 1, R2 = 2, R3
 *                                         = 4)
 *   proceed   agc_job_id_t           I    job for PROCEED (V33)
 *   recycle   agc_job_id_t           I    job for RECYCLE (V32)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t so_flashr(const agc_job_ctx_t *ctx, uint8_t verb,
                              uint8_t noun, uint16_t blank,
                              agc_job_id_t proceed, agc_job_id_t recycle)
{
    agc_status_t st; /* GOFLASHR */

    st = rz_flash(ctx, AGC_DSKY_GOFLASHR, verb, noun,
                  AGC_JOB_GOTOPOOH, proceed, recycle);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_dsky_blanket(blank);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st); /* TCF ENDOFJOB     */
}

/**********************************************************************
 * PREC/TT (p.531): PRECSET to TIG; passive transfer time T for CENTANG
 * (TIMETHET); S34/35.1 (unit normal).
 * AGC: PREC/TT (STABLE_ORBIT.agc p.531)
 *
 * FUNCTION NAME: so_prec_tt
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *   t         agc_dp_t *      O    passive transfer time for CENTANG, cs
 *                                  2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the PRECSET, TIMETHET or
 *          S34/35.1 status
 **********************************************************************/
static agc_status_t so_prec_tt(agc_interp_t *ip, agc_rendz_t *rz,
                               agc_job_id_t resume, agc_dp_t *t)
{
    agc_status_t st = rz_precset(ip, rz, agc_erase_tig(), resume); /* PRECSET */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_conic_time_out_t o; /* conic result */

    st = rz_timethet_rv(ip, rz, rz->rpass3, rz->vpass3,
                        agc_dp_sin(ip, rz->centang),
                        agc_dp_cos(ip, rz->centang), true, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    *t = o.t;

    return (agc_rendz_los_and_normal(ip, rz));
}

/**********************************************************************
 * INTRPVP (p.531): precision update of the passive vehicle to tdec1
 * (AVFLAG clear = CSM active: LEMPREC; set: CSMPREC).
 * AGC: INTRPVP (STABLE_ORBIT.agc p.531)
 *
 * FUNCTION NAME: so_intrpvp
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   tdec1     agc_dp_t           I    TDEC1, time to integrate to, cs 2^28
 *   resume    agc_job_id_t       I    job that continues the caller
 *   o         agc_integ_out_t *  O    passive state at TDEC1 (RATT, VATT)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration status
 *          (INTSTALL sleep)
 **********************************************************************/
static agc_status_t so_intrpvp(agc_interp_t *ip, agc_dp_t tdec1,
                               agc_job_id_t resume, agc_integ_out_t *o)
{
    rz_integ_kind_t k =
        agc_flag_test(AGC_FLAG_AVFLAG) ? RZ_CSMPREC : RZ_LEMPREC; /* passive */

    return (rz_integ(ip, k, tdec1, resume, o));
}

/**********************************************************************
 * DELTTIME (p.528)
 * AGC: DELTTIME (STABLE_ORBIT.agc p.528)
 *
 * FUNCTION NAME: agc_rendz_stable_orbit_deltt
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   ratt      agc_vec_t       I    passive position at TINT, m 2^29
 *   vatt      agc_vec_t       I    passive velocity at TINT, m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a bad
 *          argument
 **********************************************************************/
agc_status_t agc_rendz_stable_orbit_deltt(agc_interp_t *ip, agc_rendz_t *rz,
                                          agc_vec_t ratt, agc_vec_t vatt)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t ur = rz_unit(ip, ratt); /* RP/(RP) 0D, 2^1 */
    agc_dp_t  h; /* |V x UR|, m/cs 2^8 */

    h = rz_abval(ip, agc_vec_cross(ip, vatt, ur));
    int      x1 = 0;  /* NORM index, -(left shifts) */
    agc_dp_t hn = rz_norm(h, &x1); /* NORM X1, 2^(8+X1) */
    agc_dp_t d  = agc_dp_div(ip, rz->deltar, hn);  /* DDV, cs 2^28 after SL* */

    rz->delttime = rz_sl_star(ip, d, -7, x1);      /* SL* 0 -7,1     */

    if (!AGC_ASSERT(hn >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * P38 / P78 start: AVFLAGA or AVFLAGP, P20FLGON, LM NN = 2, V06N33 (TIG).
 * AGC: P38, P78 (STABLE_ORBIT.agc p.526)
 *
 * FUNCTION NAME: so_start
 *
 * ARGUMENTS:
 *   Argument     Type             I/O  Description (scale)
 *   -----------  ---------------  ---  ----------------------------
 *   ctx          agc_job_ctx_t *  I    job context (checked, not used)
 *   this_active  bool             I    true: this vehicle is the active one
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t so_start(agc_job_ctx_t *ctx, bool this_active)
{
    rz_avflag(this_active);
    rz_tracking_flags_on(); /* P20FLGON       */

    if (!rz_is_csm())
    {
        agc_rendz_erasable()->nn = SO_NN_OFFSETS; /* CAF DECTWO     */
    }

    return (rz_vnpooh(ctx, 6U, 33U, AGC_JOB_STABLE_ORBIT_CENTANG)); /* TIG  */
}

/**********************************************************************
 * P38 STABLE ORBIT RENDEZVOUS (p.526, label P38): AVFLAGA.
 *
 * FUNCTION NAME: agc_job_stable_orbit_rendezvous
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
agc_status_t agc_job_stable_orbit_rendezvous(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (so_start(ctx, true));
}

/**********************************************************************
 * P78 STABLE ORBIT RENDEZVOUS, OTHER VEHICLE ACTIVE (label P78).
 * AGC: ACTIVE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: agc_job_stable_orbit_rendezvous_other_vehicle
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
agc_status_t agc_job_stable_orbit_rendezvous_other_vehicle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (so_start(ctx, false));
}

/**********************************************************************
 * After V06N33 (TIG): the same display again (VNDSPLY RECYCLE).
 * AGC: P38 "VNDSPLY" V06N33 (STABLE_ORBIT.agc p.526)
 *
 * FUNCTION NAME: agc_job_stable_orbit_tig
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
agc_status_t agc_job_stable_orbit_tig(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_vnpooh(ctx, 6U, 33U, AGC_JOB_STABLE_ORBIT_CENTANG));
}

/**********************************************************************
 * V06N55 (CENTANG) GOFLASHR, R1 R2 blanked.
 * AGC: P38 V06N55 (STABLE_ORBIT.agc p.526)
 *
 * FUNCTION NAME: agc_job_stable_orbit_centang
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
agc_status_t agc_job_stable_orbit_centang(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (so_flashr(ctx, 6U, 55U, SO_BLANK_R1R2, AGC_JOB_STABLE_ORBIT_OPTION,
                      AGC_JOB_STABLE_ORBIT_CENTANG));
}

/**********************************************************************
 * OPTION1 = 5, OPTION2 = 1 (SOI); V04N06 GOFLASHR, R3 blanked.
 * AGC: P38 OPTION1 OPTION2 (STABLE_ORBIT.agc p.526)
 *
 * FUNCTION NAME: agc_job_stable_orbit_option
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
agc_status_t agc_job_stable_orbit_option(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = rz_set_option(1, 5U); /* OPTION1 = 5 */

    if (st == AGC_STATUS_OK)
    {
        st = rz_set_option(2, 1U);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_stable_orbit_option_display(ctx));
}

/**********************************************************************
 * V04N06 (option) GOFLASHR, R3 blanked; PROCEED chooses the option, RECYCLE
 * shows it again.
 * AGC: P38 V04N06 (STABLE_ORBIT.agc p.527)
 *
 * FUNCTION NAME: agc_job_stable_orbit_option_display
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
agc_status_t agc_job_stable_orbit_option_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (so_flashr(ctx, 4U, 6U, SO_BLANK_R3,
                      AGC_JOB_STABLE_ORBIT_OPTION_CHOSEN,
                      AGC_JOB_STABLE_ORBIT_OPTION_DISPLAY));
}

/**********************************************************************
 * After V04N06 (p.527): CM NN = 2. Option 1 (SOI, OPTN1): SET OPTNSW,
 * CLEAR UPDATFLG, PREC/TT, SET UPDATFLG, TINT = TIG + T, CM DSPTEM1 =
 * TINT, V06N57 (DELTAR). Option 2 (SOR): TINTSOI = TINT, CLEAR OPTNSW,
 * JUNCTN1. A PREC/TT sleep resumes this job from its start.
 * AGC: OPTN1 (STABLE_ORBIT.agc p.527)
 *
 * FUNCTION NAME: agc_job_stable_orbit_option_chosen
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
agc_status_t agc_job_stable_orbit_option_chosen(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    if (rz_is_csm())
    {
        rz->nn = SO_NN_OFFSETS; /* SSP NN 2       */
    }

    if ((rz_option(2) >> 1) != 0U)
    {                            /* SR1 BHIZ: SOR  */
        rz->deltar = rz->tpass4; /* TINTSOI = TINT */
        rz_flag_put(AGC_FLAG_OPTNSW, false);

        return (agc_job_stable_orbit_start(ctx));
    }

    rz_flag_put(AGC_FLAG_OPTNSW, true); /* OPTN1          */
    rz_flag_put(AGC_FLAG_UPDATFLG, false);
    agc_dp_t     t = 0;  /* transfer time, cs 2^28 */
    agc_status_t st; /* PREC/TT */

    st = so_prec_tt(&ip, rz, AGC_JOB_STABLE_ORBIT_OPTION_CHOSEN, &t);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_UPDATFLG, true);
    rz->tpass4 = agc_dp_add(&ip, t, agc_erase_tig()); /* TI = TIG + TF   */

    if (rz_is_csm())
    {
        st = rz_set_dsptem_dp(RZ_DSPTEM1, rz->tpass4); /* FOR DISPLAY    */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (agc_job_stable_orbit_deltar(ctx));
} /* agc_job_stable_orbit_option_chosen */

/**********************************************************************
 * V06N57 (DELTAR) GOFLASHR, R2 R3 blanked.
 * AGC: OPTN1 V06N57 (STABLE_ORBIT.agc p.527)
 *
 * FUNCTION NAME: agc_job_stable_orbit_deltar
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
agc_status_t agc_job_stable_orbit_deltar(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (so_flashr(ctx, 6U, 57U, SO_BLANK_R2R3,
                      AGC_JOB_STABLE_ORBIT_INTERCEPT_TIME,
                      AGC_JOB_STABLE_ORBIT_DELTAR));
}

/**********************************************************************
 * LM: DSPTEM1 = TINT; V06N34 (time of intercept), then JUNCTN1.
 * AGC: Luminary099 STABLE_ORBIT.agc V06N34
 *
 * FUNCTION NAME: agc_job_stable_orbit_intercept_time
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
agc_status_t agc_job_stable_orbit_intercept_time(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!rz_is_csm())
    {
        agc_status_t st; /* TINT */

        st = rz_set_dsptem_dp(RZ_DSPTEM1, agc_rendz_erasable()->tpass4);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (rz_vnpooh(ctx, 6U, 34U, AGC_JOB_STABLE_ORBIT_START));
}

/**********************************************************************
 * JUNCTN1 (p.527): CLEAR P39/79SW; SELECTMU (CLEAR FINALFLG, VN1645),
 * which returns to RECYCLE.
 * AGC: JUNCTN1 (STABLE_ORBIT.agc p.527)
 *
 * FUNCTION NAME: agc_job_stable_orbit_start
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
agc_status_t agc_job_stable_orbit_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    rz_flag_put(AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE, false);
    agc_status_t st; /* SELECTMU */

    st = agc_rendz_selectmu(rz, agc_flag_test(AGC_FLAG_CMOONFLG));

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_flag_put(AGC_FLAG_FINALFLG, false);

    return (rz_ttogo_display(ctx, &ip, rz, AGC_JOB_STABLE_ORBIT_RECYCLE));
}

/**********************************************************************
 * RECYCLE .. JUNCTN2 (pp.527-528): PREC/TT; SOI: passive to TINT
 * (INTRPVP), DELTTIME; SOR: TINT = TINTSOI + T. TARGTIME = TINT -
 * DELTTIME; MAINRTNE. A sleep resumes this job from its start.
 * AGC: RECYCLE .. JUNCTN2 (STABLE_ORBIT.agc pp.527-528)
 *
 * FUNCTION NAME: agc_job_stable_orbit_recycle
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
agc_status_t agc_job_stable_orbit_recycle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */
    agc_dp_t     t  = 0; /* transfer time, cs 2^28 */
    agc_status_t st; /* PREC/TT */

    st = so_prec_tt(&ip, rz, AGC_JOB_STABLE_ORBIT_RECYCLE, &t);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_flag_test(AGC_FLAG_OPTNSW))
    {
        agc_integ_out_t o; /* integration result */

        st = so_intrpvp(&ip, rz->tpass4, AGC_JOB_STABLE_ORBIT_RECYCLE, &o);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        st = agc_rendz_stable_orbit_deltt(&ip, rz, o.ratt, o.vatt);

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }
    else
    {
        rz->tpass4 = agc_dp_add(&ip, rz->deltar, t); /* OPTN2: TINTSOI */
    }

    rz->targtime = agc_dp_sub(&ip, rz->tpass4, rz->delttime); /* JUNCTN2 */

    return (agc_job_stable_orbit_target(ctx));
} /* agc_job_stable_orbit_recycle */

/**********************************************************************
 * MAINRTNE (pp.528-529): passive to TARGTIME (INTRPVP); INTIME = TIG;
 * S3435.25 with that target; P39: DSPLY81 (UPDATFLG unless final);
 * P38: DELVTPI DELVTPF POSTTPI, V06N58, then DSPLY81.
 * AGC: MAINRTNE (STABLE_ORBIT.agc pp.528-529)
 *
 * FUNCTION NAME: agc_job_stable_orbit_target
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
agc_status_t agc_job_stable_orbit_target(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t    *rz = agc_rendz_erasable(); /* rendezvous erasables */
    agc_integ_out_t o; /* integration result */
    agc_status_t    st; /* INTRPVP */

    st = so_intrpvp(&ip, rz->targtime, AGC_JOB_STABLE_ORBIT_TARGET, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->intime = agc_erase_tig();
    st         = agc_rendz_intercept_velocity(&ip, rz, o.ratt, o.vatt,
                                              AGC_JOB_STABLE_ORBIT_TARGET);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool final = agc_flag_test(AGC_FLAG_FINALFLG); /* FINALFLG */

    if (!final)
    {
        rz_flag_put(AGC_FLAG_UPDATFLG, true);
    }

    if (agc_flag_test(AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE))
    {
        return (agc_job_stable_orbit_dv_lv(ctx)); /* P39P79         */
    }

    rz->delvtpi = rz_abval(&ip, rz->delveet3); /* MAINRTN1       */
    rz->delvtpf = rz_abval(&ip, agc_vec_sub(&ip, rz->vpass4, rz->vtprime));
    agc_rendz_periapo_t pa; /* PERIAPO1 results */

    st = agc_rendz_periapo_body_scale(&ip, rz, rz->ract3, rz->viprime, &pa);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->posttpi = agc_rendz_scalar_from_body(rz, pa.hperi);

    return (rz_vnpooh(ctx, 6U, 58U, AGC_JOB_STABLE_ORBIT_DV_LV)); /* DSPLY58 */
} /* agc_job_stable_orbit_target */

/**********************************************************************
 * DSPLY81: V06N81 (DELVLVC).
 * AGC: DSPLY81 (STABLE_ORBIT.agc p.529)
 *
 * FUNCTION NAME: agc_job_stable_orbit_dv_lv
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
agc_status_t agc_job_stable_orbit_dv_lv(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_vnpooh(ctx, 6U, 81U, AGC_JOB_STABLE_ORBIT_DV_ACCEPTED));
}

/**********************************************************************
 * After V06N81: CLEAR XDELVFLG, DELVSIN = DELVEET3, VN1645; then P39/P79B
 * (P39/79SW set) or RECYCLE.
 * AGC: DSPLY81 after V06N81 (STABLE_ORBIT.agc p.529)
 *
 * FUNCTION NAME: agc_job_stable_orbit_dv_accepted
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
agc_status_t agc_job_stable_orbit_dv_accepted(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    rz_flag_put(AGC_FLAG_XDELVFLG, false);
    rz->delvsin = rz->delveet3;
    agc_job_id_t next = AGC_JOB_STABLE_ORBIT_RECYCLE; /* P38: RECYCLE */

    if (agc_flag_test(AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE))
    {
        next = AGC_JOB_STABLE_ORBIT_MIDCOURSE_COMPUTE; /* P39/P79B */
    }

    return (rz_ttogo_display(ctx, &ip, rz, next));
}

/* ======================= P39 / P79 (pp.530-531) ========================== */
/**********************************************************************
 * P39/P79A: KT = TIG increment, P20FLGON, SET P39/79SW, SELECTMU.
 * AGC: P39/P79A (STABLE_ORBIT.agc p.530)
 *
 * FUNCTION NAME: som_start
 *
 * ARGUMENTS:
 *   Argument     Type             I/O  Description (scale)
 *   -----------  ---------------  ---  ----------------------------
 *   ctx          agc_job_ctx_t *  I    job context (checked, not used)
 *   this_active  bool             I    true: this vehicle is the active one
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t som_start(agc_job_ctx_t *ctx, bool this_active)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    rz_avflag(this_active);
    rz->kt = this_active ? rz->atiginc : rz->ptiginc; /* ATIGINC / PTIGINC */
    rz_tracking_flags_on();
    rz_flag_put(AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE, true);
    agc_status_t st; /* SELECTMU */

    st = agc_rendz_selectmu(rz, agc_flag_test(AGC_FLAG_CMOONFLG));

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_FINALFLG, false);

    return (rz_ttogo_display(ctx, &ip, rz,
                             AGC_JOB_STABLE_ORBIT_MIDCOURSE_COMPUTE));
}

/**********************************************************************
 * P39 STABLE ORBIT MIDCOURSE (p.530, label P39): AVFLAGA, ATIGINC.
 *
 * FUNCTION NAME: agc_job_stable_orbit_midcourse
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
agc_status_t agc_job_stable_orbit_midcourse(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (som_start(ctx, true));
}

/**********************************************************************
 * P79 STABLE ORBIT MIDCOURSE, OTHER VEHICLE ACTIVE (label P79).
 * AGC: ACTIVE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: agc_job_stable_orbit_midcourse_other_vehicle
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
agc_status_t agc_job_stable_orbit_midcourse_other_vehicle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (som_start(ctx, false));
}

/**********************************************************************
 * P39/P79B (p.530): TIG = now + KT.
 * AGC: P39/P79B (STABLE_ORBIT.agc p.530)
 *
 * FUNCTION NAME: agc_job_stable_orbit_midcourse_compute
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
agc_status_t agc_job_stable_orbit_midcourse_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    agc_erase_set_tig(agc_dp_add(&ip, agc_rtb_loadtime(), rz->kt));

    return (agc_job_stable_orbit_midcourse_advance(ctx));
}

/**********************************************************************
 * P39/P79B continued: PRECSET to TIG, S34/35.1, MAINRTNE with TARGTIME
 * (saved by P38). Resumed here after an INTSTALL sleep.
 * AGC: P39/P79B (STABLE_ORBIT.agc p.530)
 *
 * FUNCTION NAME: agc_job_stable_orbit_midcourse_advance
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
agc_status_t agc_job_stable_orbit_midcourse_advance(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */
    agc_status_t st = rz_precset(&ip, rz, agc_erase_tig(), /* PRECSET */
                                 AGC_JOB_STABLE_ORBIT_MIDCOURSE_ADVANCE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_rendz_los_and_normal(&ip, rz);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_stable_orbit_target(ctx));
}
