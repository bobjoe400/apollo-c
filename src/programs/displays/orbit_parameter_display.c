/**********************************************************************
 * FILE NAME: orbit_parameter_display.c
 *
 * PURPOSE:
 *   R30 ORBIT PARAMETER DISPLAY (verb 82), both vehicles: "Called by
 *   verb 82 enter. Calculate and display orbital parameters." With
 *   average G off the crew picks a vehicle (V04N12: R2 = 1 this
 *   vehicle), its state is advanced to now (THISPREC / OTHPREC) and the
 *   parameters are shown once on V16N44, the clock fields ticking once
 *   per second (TICKTEST). With average G on, SR30.1 runs on the current
 *   state about once a second until the crew ends the display.
 *
 * AGC SOURCE:
 *   Comanche055 R30.agc pp. 514-523 (V82CALL, V82GOFF, V82GOFLP,
 *   V82STALL, V82GOFF1, BOTHSHIP, SPLRET1, TICKTEST, V82GON, V82GON1,
 *   V82GON3, V82REDSP, CANDEL, SR30.1, MAXCHK)
 *   Luminary099 R30.agc (same text; the moon flag is MOONTHIS, and the
 *   LM never calls DELRSPL)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  ----------------------------
 *   RN VN      agc_vec_t  I    2^29, 2^7  average-G state (V82GON1)
 *   RLS        agc_vec_t  I    m 2^27     landing site (moon pad)
 *   MODREG     agc_sp_t   I    -          P00 / P11 (CANDEL)
 *   EXTVBACT   word       I    -          bit 5: display still wanted
 *   OPTION1 OPTION2  words I/O -          V04N12 vehicle option
 *   AVEGFLAG AMOONFLG V82EMFLG  flags  I/O  -  mode / body
 *   RONE VONE HAPOX HPERX TFF -TPER TSTART82 V82FLAGS (agc_rendz_t)
 *              I/O  m 2^29, m/cs 2^7, cs 2^28  N44 parameters
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ----------------------------------------------
 *   agc_tff_conic         TFFCONMU: conic parameters
 *   agc_tff_rp_ra         TFFRP/RA: apsides
 *   agc_tff_calctper      CALCTPER: time to perigee
 *   agc_tff_calctff       CALCTFF: time to the entry altitude
 *   agc_guid_delrspl      DELRSPL: CM splash error (noun 50)
 *   rz_integ              THISPREC / OTHPREC
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms of its own (FINDVAC raises the executive's alarm when no
 *   VAC area is free). AGC_STATUS_BAD_ARG for NULL pointers; display,
 *   waitlist and integration statuses are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Extended verb jobs plus the TICKTEST waitlist task (1 s period).
 *
 * NOTES:
 *   Scales: RONE m 2^29, VONE m/cs 2^7 (SR30.1 shifts both left 2 about
 *   the moon), HAPOX HPERX m 2^29, TFF -TPER cs 2^28.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"

/* ---- constants (R30.agc p.517 and SR30.1 p.523) ------------------------- */
/* 1/RTMUM 2DEC* .45162595 E-4 B14*, 1/RTMUE 2DEC* .50087529 E-5 B17*     */
#define OPD_1_RTMUM ((agc_dp_t) 198627193) /* 1/RTMUM, B14 */
#define OPD_1_RTMUE ((agc_dp_t) 176229826) /* 1/RTMUE, B17 */

/* MINPERM 2DEC 10668 B-27 (35 kft), MINPERE 2DEC 91440 B-29 (300 kft)    */
#define OPD_MINPERM ((agc_dp_t) 21336) /* 35 kft, m 2^27 */
#define OPD_MINPERE ((agc_dp_t) 45720) /* 300 kft, m 2^29 */

/* V82FLAGS bits: BIT1 tick -TPER, BIT2 tick TFF                          */
#define OPD_TICK_TPER ((agc_sp_t) 1) /* tick -TPER */
#define OPD_TICK_TFF  ((agc_sp_t) 2) /* tick TFF */
#define OPD_TICK_MASK ((agc_sp_t) 3) /* both tick bits */

/* EXTVBACT BIT5: the extended verb display is still wanted               */
#define OPD_EXTVBACT_BIT5 ((uint16_t) 020) /* BIT5 */

/* BIT4 (80 ms): first TICKTEST; PRIO7, PRIO25                            */
#define OPD_BIT4_CS ((uint16_t) 8)     /* 80 ms, cs */
#define OPD_PRIO7   ((agc_prio_t) 07)  /* PRIO7 */
#define OPD_PRIO25  ((agc_prio_t) 025) /* PRIO25 */

/* CHECKMM DEC 11: P11 earth orbit insertion monitor                      */
#define OPD_MODE_EARTH_ORBIT_INSERTION_MONITOR ((agc_sp_t) 11) /* P11 */

/**********************************************************************
 * MAXCHK (p.523): limit a height to MAXNM (9999.9 NM), m 2^29
 * AGC: MAXCHK (R30.agc p.523)
 *
 * FUNCTION NAME: agc_orbit_params_maxchk
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   h         agc_dp_t        I    height, m 2^29
 *
 * RETURNS: agc_dp_t -- H limited to MAXNM (9999.9 NM), m 2^29
 **********************************************************************/
agc_dp_t agc_orbit_params_maxchk(agc_interp_t *ip, agc_dp_t h)
{
    agc_dp_t d = agc_dp_sub(ip, h, RZ_MAXNM); /* DSU MAXNM, m 2^29 */

    if (d >= 0)
    { /* BPL +3        */
        return (RZ_MAXNM);
    }

    agc_dp_t r = agc_dp_add(ip, d, RZ_MAXNM); /* DAD MAXNM, m 2^29 */

    return (r);
}

/**********************************************************************
 * MOONGON / EARTHGON / MOONPAD / EARTHPAD (pp.517, 519)
 * Earth: 1/RTMUE, MINPERE, RPAD (pad 37-B), V82EMFLG clear. Moon: 1/RTMUM,
 * MINPERM, |RLS| (m 2^27), V82EMFLG set. rls_mag_or_rpad is the pad or
 * landing-site radius the caller computed (RPAD or ABVAL RLS).
 * AGC: MOONGON, EARTHGON, MOONPAD, EARTHPAD (R30.agc pp.517, 519)
 *
 * FUNCTION NAME: agc_orbit_params_select_body
 *
 * ARGUMENTS:
 *   Argument         Type                     I/O  Description (scale)
 *   ---------------  -----------------------  ---  ----------------
 *   moon             bool                     I    true: moon-centred state
 *   rls_mag_or_rpad  agc_dp_t                 I    RPAD or |RLS|, m 2^29 /
 *                                                  2^27
 *   in               agc_orbit_params_in_t *  O    1/RTMU, HPERMIN, RPADTEM
 *                                                  and the body of SR30.1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a bad
 *          argument
 **********************************************************************/
agc_status_t agc_orbit_params_select_body(bool moon, agc_dp_t rls_mag_or_rpad, agc_orbit_params_in_t *in)
{
    if (!AGC_ASSERT(in != NULL) || !AGC_ASSERT(rls_mag_or_rpad >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    in->moon     = moon;
    in->tff_rtmu = moon ? OPD_1_RTMUM : OPD_1_RTMUE; /* 1/RTMU */
    in->hpermin  = moon ? OPD_MINPERM : OPD_MINPERE; /* MINPER */
    in->rpadtem  = rls_mag_or_rpad;
    rz_vflag_set(RZ_VF_ORBIT_DISPLAY_MOON, moon);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SR2R if moon centred (2^27 -> 2^29 for display), then MAXCHK.
 * AGC: STORHAPO, STORHPER "SR2R" (R30.agc p.523)
 *
 * FUNCTION NAME: opd_display_height
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   h         agc_dp_t        I    height, body scale (m 2^29 / 2^27)
 *   moon      bool            I    true: moon scale, shifted to 2^29
 *
 * RETURNS: agc_dp_t -- the display height, m 2^29, limited by MAXCHK
 **********************************************************************/
static agc_dp_t opd_display_height(agc_interp_t *ip, agc_dp_t h, bool moon)
{
    agc_dp_t x = moon ? agc_dp_shr_round(h, 2) : h; /* SR2R, m 2^29 */

    return (agc_orbit_params_maxchk(ip, x));
}

/* The TFF erasables (NRMAG, NRTERM, TFFNP, TFFVSQ, TFFX, TFFTEM...) the
 * last SR30.1 left; CM DELRSPL (CANDEL) works on them.                  */
static agc_tff_t opd_tff_debris; /* TFF erasables */

/**********************************************************************
 * SR30.1 (pp.522-523)
 * AGC: SR30.1 (R30.agc pp.522-523)
 *
 * FUNCTION NAME: opd_compute
 *
 * ARGUMENTS:
 *   Argument  Type                           I/O  Description (scale)
 *   --------  -----------------------------  ---  -----------------
 *   ip        agc_interp_t *                 I/O  interpreter state
 *                                                 (OVFIND)
 *   in        const agc_orbit_params_in_t *  I    RONE VONE, 1/RTMU,
 *                                                 HPERMIN, RPADTEM, body
 *   out       agc_orbit_params_out_t *       O    RAPO RPER HAPOX HPERX TFF
 *                                                 -TPER
 *   tff_out   agc_tff_t *                    O    the TFF erasables left
 *                                                 for DELRSPL
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the first failing TFF status
 **********************************************************************/
static agc_status_t opd_compute(agc_interp_t *ip, const agc_orbit_params_in_t *in, agc_orbit_params_out_t *out, agc_tff_t *tff_out)
{
    agc_tff_t tff = {0};

    tff.rone        = in->moon ? agc_vec_shl(ip, in->rone, 2) : in->rone; /* VSL2 */
    tff.vone        = in->moon ? agc_vec_shl(ip, in->vone, 2) : in->vone; /* VSL2 */
    tff.rtmu        = in->tff_rtmu;
    agc_status_t st = agc_tff_conic(ip, &tff);                            /* TFFCONMU      */

    if (st == AGC_STATUS_OK)
    {
        st = agc_tff_rp_ra(ip, &tff); /* TFFRP/RA      */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    out->rapo     = tff.rapo;
    out->rper     = tff.rper;
    out->hapox    = opd_display_height(ip, agc_dp_sub(ip, tff.rapo, in->rpadtem), in->moon); /* STORHAPO      */
    agc_dp_t hper = agc_dp_sub(ip, tff.rper, in->rpadtem);                                   /* MPAC +4, m 2^29 */

    out->hperx    = opd_display_height(ip, hper, in->moon); /* STORHPER      */
    out->neg_tper = 0;                                      /* HI6ZEROS      */

    if (agc_dp_sub(ip, hper, in->hpermin) >= 0)
    { /* DOTPER        */
        agc_dp_t tper = 0; /* TPER, cs 2^28 */

        st            = agc_tff_calctper(ip, &tff, tff.rper, &tper);
        out->neg_tper = agc_dp_neg(tper); /* DCOMP         */
    }

    agc_dp_t     t = 0; /* TFF, cs 2^28 */
    agc_status_t st2;   /* CALCTFF */

    st2 = agc_tff_calctff(ip, &tff, agc_dp_add(ip, in->hpermin, in->rpadtem), &t);

    out->tff = agc_dp_neg(t); /* DCOMP, TFF    */
    *tff_out = tff;

    return ((st != AGC_STATUS_OK) ? st : st2); /* first failure */
} /* opd_compute */

/**********************************************************************
 * SR30.1 on caller inputs (tests and other programs): apsides, display
 * heights, TFF and -TPER.
 * AGC: SR30.1 (R30.agc pp.522-523)
 *
 * FUNCTION NAME: agc_orbit_params_compute
 *
 * ARGUMENTS:
 *   Argument  Type                           I/O  Description (scale)
 *   --------  -----------------------------  ---  -----------------
 *   ip        agc_interp_t *                 I/O  interpreter state
 *                                                 (OVFIND)
 *   in        const agc_orbit_params_in_t *  I    RONE VONE, 1/RTMU,
 *                                                 HPERMIN, RPADTEM, body
 *   out       agc_orbit_params_out_t *       O    RAPO RPER HAPOX HPERX TFF
 *                                                 -TPER
 *
 * RETURNS: agc_status_t -- as opd_compute
 **********************************************************************/
agc_status_t agc_orbit_params_compute(agc_interp_t                *ip,
                                      const agc_orbit_params_in_t *in,
                                      agc_orbit_params_out_t      *out)
{
    agc_tff_t tff; /* TFF erasables */

    return (opd_compute(ip, in, out, &tff));
}

/**********************************************************************
 * SR30.1 on the rendezvous erasables (RONE VONE TFF/RTMU HPERMIN RPADTEM
 * V82EMFLG -> HAPOX HPERX TFF -TPER).
 * AGC: SR30.1 (R30.agc pp.522-523)
 *
 * FUNCTION NAME: opd_orbit_params
 *
 * ARGUMENTS:
 *   Argument  Type                           I/O  Description (scale)
 *   --------  -----------------------------  ---  -----------------
 *   ip        agc_interp_t *                 I/O  interpreter state
 *                                                 (OVFIND)
 *   rz        agc_rendz_t *                  I/O  rendezvous erasables
 *   in        const agc_orbit_params_in_t *  I    SR30.1 inputs (copied to
 *                                                 the erasables)
 *
 * RETURNS: agc_status_t -- the SR30.1 status, AGC_STATUS_BAD_ARG for a
 *          positive -TPER
 **********************************************************************/
static agc_status_t opd_orbit_params(agc_interp_t *ip, agc_rendz_t *rz, const agc_orbit_params_in_t *in)
{
    agc_orbit_params_out_t o;  /* SR30.1 results */
    agc_status_t           st; /* SR30.1 */

    st = opd_compute(ip, in, &o, &opd_tff_debris);

    rz->tff_rtmu                        = in->tff_rtmu;
    rz->orbit_parameter_display_hpermin = in->hpermin;
    rz->rpadtem                         = in->rpadtem;
    rz->hapox                           = o.hapox;
    rz->hperx                           = o.hperx;
    rz->tff                             = o.tff;
    rz->neg_tper                        = o.neg_tper;

    if (!AGC_ASSERT(o.neg_tper <= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * Inputs of SR30.1 for a state about the moon (moon) or the earth.
 * AGC: MOONGON, EARTHGON (R30.agc p.517)
 *
 * FUNCTION NAME: opd_input
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *            I/O  rendezvous erasables
 *   moon      bool                     I    true: moon sphere of influence
 *   in        agc_orbit_params_in_t *  O    SR30.1 inputs from RONE VONE
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a bad
 *          argument
 **********************************************************************/
static agc_status_t opd_input(agc_interp_t *ip, agc_rendz_t *rz, bool moon, agc_orbit_params_in_t *in)
{
    in->rone     = rz->rone;
    in->vone     = rz->vone;
    agc_dp_t pad = moon ? rz_abval(ip, agc_erase_rls()) : RZ_RPAD_B29; /* m 2^27 / 2^29 */

    return (agc_orbit_params_select_body(moon, pad, in));
}

/* ======================== verb 82 jobs and task ========================== */
/**********************************************************************
 * Job prologue.
 * AGC: none (job prologue; the AGC uses fixed erasables)
 *
 * FUNCTION NAME: opd_begin
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_rendz_t * -- the rendezvous erasables (never NULL)
 **********************************************************************/
static agc_rendz_t *opd_begin(agc_interp_t *ip)
{
    agc_interp_init(ip);

    return (agc_rendz_erasable());
}

/**********************************************************************
 * GOXDSPF (= GOMARKF) with the extended-verb returns.
 * AGC: GOXDSPF (R30.agc p.515)
 *
 * FUNCTION NAME: opd_flash
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   verb      uint8_t                I    verb of the display
 *   noun      uint8_t                I    noun of the display
 *   term      agc_job_id_t           I    job for TERMINATE (V34)
 *   proceed   agc_job_id_t           I    job for PROCEED (V33)
 *   enter     agc_job_id_t           I    job for ENTER (data loaded)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t opd_flash(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t term, agc_job_id_t proceed, agc_job_id_t enter)
{
    return (rz_flash(ctx, AGC_DSKY_GOMARKF, verb, noun, term, proceed, enter));
}

/**********************************************************************
 * EXTVBACT bit 5 clear: the crew has ended the verb 82 display.
 * AGC: "CAF BIT5; MASK EXTVBACT" (R30.agc p.518)
 *
 * FUNCTION NAME: opd_terminated
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when EXTVBACT bit 5 is clear (the display was
 *          ended)
 **********************************************************************/
static bool opd_terminated(void)
{
    return ((agc_erase_extvbact() & OPD_EXTVBACT_BIT5) == 0U);
}

/**********************************************************************
 * V82CALL (p.515): average G on -> V82GON, off -> V82GOFF.
 *
 * FUNCTION NAME: agc_job_orbit_parameter_display
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
agc_status_t agc_job_orbit_parameter_display(agc_job_ctx_t *ctx)
{
    if (rz_vflag_test(RZ_VF_AVEGFLAG))
    { /* V82GON        */
        agc_status_t st; /* FINDVAC */

        st = agc_exec_findvac(OPD_PRIO7, AGC_JOB_ORBIT_DISPLAY_AVERAGE_G);

        if (st != AGC_STATUS_OK)
        {
            return (st); /* FINDVAC: no VAC area (alarm) */
        }

        return (rz_continue(AGC_JOB_ORBIT_DISPLAY_MONITOR)); /* CHANG1     */
    }

    agc_status_t st = rz_set_option(1, 2U); /* OPTIONX = 2   */

    if (st == AGC_STATUS_OK)
    {
        st = rz_set_option(2, 1U); /* OPTIONX+1 = 1 */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_orbit_display_vehicle_select(ctx));
}

/**********************************************************************
 * V82GOFF (p.515): V04N12 flashing; TERMINATE -> ENDEXT, PROCEED ->
 * start, ENTER (data in, "TC -5") -> the same display again.
 *
 * FUNCTION NAME: agc_job_orbit_display_vehicle_select
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
agc_status_t agc_job_orbit_display_vehicle_select(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(rz_option(1) == 2U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (opd_flash(ctx, 4U, 12U, AGC_JOB_RENDZ_ENDEXT, AGC_JOB_ORBIT_DISPLAY_VEHICLE_SELECTED, AGC_JOB_ORBIT_DISPLAY_VEHICLE_SELECT));
}

/**********************************************************************
 * PROCEED on V04N12: WAITLIST 80 ms TICKTEST, then V82GOFLP.
 * AGC: PROCEED (DISPLAY_INTERFACE_ROUTINES.agc p.1481)
 *
 * FUNCTION NAME: agc_job_orbit_display_vehicle_selected
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
agc_status_t agc_job_orbit_display_vehicle_selected(agc_job_ctx_t *ctx)
{
    agc_status_t st; /* WAITLIST */

    st = agc_waitlist_call(OPD_BIT4_CS, AGC_TASK_TICKTEST);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_orbit_display_recycle(ctx));
}

/**********************************************************************
 * V82GOFLP (p.515), the major recycle loop: V82FLAGS = 0 (TICKTEST does
 * not tick), FINDVAC PRIO7 V82GOFF1, then V82STALL.
 *
 * FUNCTION NAME: agc_job_orbit_display_recycle
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
agc_status_t agc_job_orbit_display_recycle(agc_job_ctx_t *ctx)
{
    agc_rendz_erasable()->orbit_display_flags = 0;
    agc_status_t st; /* FINDVAC */

    st = agc_exec_findvac(OPD_PRIO7, AGC_JOB_ORBIT_DISPLAY_UPDATE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_orbit_display_stall(ctx));
}

/**********************************************************************
 * V82STALL / FLAGGON (p.516): hold V16N44 until V82GOFF1 sets a tick bit
 * (DELAYJOB 1 s per try); then V16N44: TERM, PROCEED -> B5OFF,
 * RECYCLE -> V82GOFLP.
 *
 * FUNCTION NAME: agc_job_orbit_display_stall
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
agc_status_t agc_job_orbit_display_stall(agc_job_ctx_t *ctx)
{
    if ((agc_rendz_erasable()->orbit_display_flags & OPD_TICK_MASK) == 0)
    {
        return (agc_svc_delayjob(RZ_1SEC_CS, AGC_JOB_ORBIT_DISPLAY_STALL));
    }

    return (opd_flash(ctx, 16U, 44U, AGC_JOB_RENDZ_DISPLAY_TERMINATE, AGC_JOB_RENDZ_DISPLAY_TERMINATE, AGC_JOB_ORBIT_DISPLAY_RECYCLE));
}

/**********************************************************************
 * SPLRET1 .. TICKTFF (pp.517-518): the parameters were computed at
 * TSTART82; correct the ticking field for the time since then and tell
 * TICKTEST which field to tick (-TPER when TFF was not computable,
 * otherwise TFF). ENDOFJOB.
 * AGC: SPLRET1 .. TICKTFF (R30.agc pp.517-518)
 *
 * FUNCTION NAME: opd_start_ticking
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          negative elapsed time
 **********************************************************************/
static agc_status_t opd_start_ticking(agc_interp_t *ip, agc_rendz_t *rz)
{
    rz->tstart_orbit = agc_dp_sub(ip, agc_rtb_loadtime(), rz->tstart_orbit);

    if (rz->neg_tper != 0)
    { /* TICKTPER      */
        rz->neg_tper            = agc_dp_add(ip, rz->neg_tper, rz->tstart_orbit);
        rz->orbit_display_flags = OPD_TICK_TPER;
    }
    else
    { /* TICKTFF       */
        rz->tff                 = agc_dp_add(ip, rz->tff, rz->tstart_orbit);
        rz->orbit_display_flags = OPD_TICK_TFF;
    }

    if (!AGC_ASSERT(rz->tstart_orbit >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * V82GOFF1 / BOTHSHIP (pp.516-517): TDEC1 = TSTART82 = now; THISPREC
 * (OPTIONX+1 = 1) or OTHPREC; RONE VONE = RATT VATT; earth or moon
 * parameters by the X2 the integration left; SR30.1. CM in P00: CANDEL
 * (DELRSPL) first. An INTSTALL sleep resumes this job from its start.
 * AGC: V82GOFF1, BOTHSHIP (R30.agc pp.516-517)
 *
 * FUNCTION NAME: agc_job_orbit_display_update
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbit_display_update(void)
{
    agc_interp_t ip;                       /* interpreter state */
    agc_rendz_t *rz  = opd_begin(&ip);     /* rendezvous erasables */
    agc_dp_t     now = agc_rtb_loadtime(); /* RTB LOADTIME, cs 2^28 */

    rz->tstart_orbit     = now;
    rz_integ_kind_t kind = (rz_option(2) == 1U) ? RZ_THISPREC : RZ_OTHPREC;            /* R2 = 1: this */
    agc_integ_out_t o;                                                                 /* integration result */
    agc_status_t    st   = rz_integ(&ip, kind, now, AGC_JOB_ORBIT_DISPLAY_UPDATE, &o); /* to now */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->rone = o.ratt;        /* BOTHSHIP      */
    rz->vone = o.vatt;
    agc_orbit_params_in_t in; /* SR30.1 inputs */

    st = opd_input(&ip, rz, o.x2 != 0, &in);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = opd_orbit_params(&ip, rz, &in);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (rz_is_csm() && agc_erase_modreg() == 0)
    { /* CANDEL        */
        return (agc_job_orbit_display_splash());
    }

    return (opd_start_ticking(&ip, rz));
} /* agc_job_orbit_display_update */

/**********************************************************************
 * V82GON1 (p.519), average G on: RONE VONE = RN VN; moon (CM AMOONFLG,
 * LM MOONTHIS) or earth parameters; SR30.1; CM in P11: CANDEL; then
 * V82GON3.
 *
 * FUNCTION NAME: agc_job_orbit_display_average_g
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbit_display_average_g(void)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = opd_begin(&ip); /* rendezvous erasables */

    rz->rone  = agc_erase_rn(); /* RN, VN at 2^29, 2^7 (R30 p.519) */
    rz->vone  = agc_erase_vn();
    bool moon = rz_moonthis();  /* LM MOONTHIS */

    if (rz_is_csm())
    {
        moon = rz_vflag_test(RZ_VF_AMOONFLG); /* CM AMOONFLG */
    }

    agc_orbit_params_in_t in;                                 /* SR30.1 inputs */
    agc_status_t          st = opd_input(&ip, rz, moon, &in); /* inputs */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = opd_orbit_params(&ip, rz, &in);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (rz_is_csm() && agc_phase_checkmm(OPD_MODE_EARTH_ORBIT_INSERTION_MONITOR))
    {
        return (agc_job_orbit_display_splash());
    }

    return (agc_job_orbit_display_repeat());
}

/**********************************************************************
 * V82GON3 (p.520): EXTVBACT bit 5 clear -> ENDEXT; else DELAYJOB 1 s,
 * V82GON1 again.
 *
 * FUNCTION NAME: agc_job_orbit_display_repeat
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbit_display_repeat(void)
{
    if (opd_terminated())
    {
        return (agc_dsky_endmark());
    }

    return (agc_svc_delayjob(RZ_1SEC_CS, AGC_JOB_ORBIT_DISPLAY_AVERAGE_G));
}

/**********************************************************************
 * CANDEL / SPLRET (CM p.520): INTSTALL, DELRSPL with TFF (it does the
 * INTWAKE0); in P00 -> SPLRET1 (tick set-up), otherwise V82GON3.
 * Should DELRSPL fail before its INTWAKE0, this job releases the area.
 *
 * FUNCTION NAME: agc_job_orbit_display_splash
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_orbit_display_splash(void)
{
    agc_interp_t ip;                  /* interpreter state */
    agc_rendz_t *rz = opd_begin(&ip); /* rendezvous erasables */
    agc_status_t st;                  /* INTSTALL */

    st = agc_integ_install(0, AGC_JOB_ORBIT_DISPLAY_SPLASH);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* asleep        */
    }

    st = agc_guid_delrspl(&ip, &opd_tff_debris, rz->tff);

    if (st != AGC_STATUS_OK)
    {
        st = agc_integ_intwake();

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    if (agc_erase_modreg() == 0)
    {
        return (opd_start_ticking(&ip, rz)); /* SPLRET1       */
    }

    return (agc_job_orbit_display_repeat());
}

/**********************************************************************
 * V82REDSP (p.519): V16N44 monitor; TERM, PROCEED -> B5OFF (V82GON1
 * stops), RECYCLE -> the display again.
 *
 * FUNCTION NAME: agc_job_orbit_display_monitor
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
agc_status_t agc_job_orbit_display_monitor(agc_job_ctx_t *ctx)
{
    return (opd_flash(ctx, 16U, 44U, AGC_JOB_RENDZ_DISPLAY_TERMINATE, AGC_JOB_RENDZ_DISPLAY_TERMINATE, AGC_JOB_ORBIT_DISPLAY_MONITOR));
}

/**********************************************************************
 * TICKTEST (p.518), waitlist task once a second while EXTVBACT bit 5 is
 * set: tick -TPER (V82FLAGS bit 1) or TFF (bit 2) by 1 s. When the bit
 * is clear: NOVAC PRIO25 ENDEXT (ENDEXT cannot run in a rupt).
 * AGC: TICKTEST (R30.agc p.518)
 *
 * FUNCTION NAME: agc_task_ticktest
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_ticktest(void)
{
    if (opd_terminated())
    {
        return (agc_exec_novac(OPD_PRIO25, AGC_JOB_RENDZ_ENDEXT));
    }

    agc_status_t st; /* next tick */

    st              = agc_waitlist_call(RZ_1SEC_CS, AGC_TASK_TICKTEST);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */
    agc_sp_t     bits;                      /* V82FLAGS, unscaled */

    bits = (agc_sp_t) (rz->orbit_display_flags & OPD_TICK_MASK);

    if (!AGC_ASSERT(bits != OPD_TICK_MASK)) /* never both (INDEX A)   */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (bits == OPD_TICK_TPER)
    { /* TPERTICK DAS  */
        rz->neg_tper += (agc_dp_t) RZ_1SEC_CS;
    }
    else if (bits == OPD_TICK_TFF)
    { /* TFFTICK DAS   */
        rz->tff += (agc_dp_t) RZ_1SEC_CS;
    }

    return (st);
}

/**********************************************************************
 * B5OFF (SERVICE_ROUTINES): clear EXTVBACT bit 5, ENDOFJOB. The verb 82
 * and 83 loops see the bit and end themselves.
 * AGC: B5OFF (SERVICE_ROUTINES.agc)
 *
 * FUNCTION NAME: agc_job_rendz_display_terminate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (ENDOFJOB, always)
 **********************************************************************/
agc_status_t agc_job_rendz_display_terminate(void)
{
    agc_svc_b5off(); /* B5OFF */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ENDEXT as a job (TICKTEST's NOVAC; V82GOFF TERMINATE).
 * AGC: ENDEXT (EXTENDED_VERBS.agc)
 *
 * FUNCTION NAME: agc_job_rendz_endext
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendz_endext(void)
{
    return (agc_dsky_endmark());
}
