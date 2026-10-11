/*
 * orbit_parameter_display.c -- R30 ORBIT PARAMETER DISPLAY, verb 82, both
 * vehicles (R30.agc, Comanche055 pp.514-523, Luminary099 the same text).
 *
 * "Called by verb 82 enter. Calculate and display orbital parameters."
 * With average G off the crew picks a vehicle (V04N12: R2 = 1 this
 * vehicle), its state is advanced to now (THISPREC / OTHPREC) and the
 * parameters are shown once on V16N44, the clock fields ticking once per
 * second (TICKTEST). With average G on, SR30.1 runs on the current state
 * about once a second until the crew ends the display.
 *
 * Vehicle differences: the moon flag is AMOONFLG (CM) / MOONTHIS (LM); the
 * CM calls DELRSPL (splash error, noun 50) in P00 and P11, the LM never.
 *
 * Scales: RONE m 2^29, VONE m/cs 2^7 (SR30.1 shifts both left 2 about the
 * moon), HAPOX HPERX m 2^29, TFF -TPER cs 2^28.
 */
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"

/* ---- constants (R30.agc p.517 and SR30.1 p.523) ------------------------- */
/* 1/RTMUM 2DEC* .45162595 E-4 B14*, 1/RTMUE 2DEC* .50087529 E-5 B17*     */
#define OPD_1_RTMUM ((agc_dp_t) 198627193)
#define OPD_1_RTMUE ((agc_dp_t) 176229826)
/* MINPERM 2DEC 10668 B-27 (35 kft), MINPERE 2DEC 91440 B-29 (300 kft)    */
#define OPD_MINPERM ((agc_dp_t) 21336)
#define OPD_MINPERE ((agc_dp_t) 45720)
/* V82FLAGS bits: BIT1 tick -TPER, BIT2 tick TFF                          */
#define OPD_TICK_TPER ((agc_sp_t) 1)
#define OPD_TICK_TFF  ((agc_sp_t) 2)
#define OPD_TICK_MASK ((agc_sp_t) 3)
/* EXTVBACT BIT5: the extended verb display is still wanted               */
#define OPD_EXTVBACT_BIT5 ((uint16_t) 020)
/* BIT4 (80 ms): first TICKTEST; PRIO7, PRIO25                            */
#define OPD_BIT4_CS ((uint16_t) 8)
#define OPD_PRIO7   ((agc_prio_t) 07)
#define OPD_PRIO25  ((agc_prio_t) 025)
/* CHECKMM DEC 11: P11 earth orbit insertion monitor                      */
#define OPD_MODE_EARTH_ORBIT_INSERTION_MONITOR ((agc_sp_t) 11)

/* ---- MAXCHK (p.523): limit a height to MAXNM (9999.9 NM), m 2^29 ------- */
agc_dp_t agc_orbit_params_maxchk(agc_interp_t *ip, agc_dp_t h)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0); /* no valid result */
    }

    agc_dp_t d = agc_dp_sub(ip, h, RZ_MAXNM); /* DSU MAXNM     */

    if (d >= 0)
    { /* BPL +3        */
        return (RZ_MAXNM);
    }

    agc_dp_t r = agc_dp_add(ip, d, RZ_MAXNM); /* DAD MAXNM     */

    if (!AGC_ASSERT(r < RZ_MAXNM))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/* ---- MOONGON / EARTHGON / MOONPAD / EARTHPAD (pp.517, 519) -------------
 * Earth: 1/RTMUE, MINPERE, RPAD (pad 37-B), V82EMFLG clear. Moon: 1/RTMUM,
 * MINPERM, |RLS| (m 2^27), V82EMFLG set. rls_mag_or_rpad is the pad or
 * landing-site radius the caller computed (RPAD or ABVAL RLS).        */
agc_status_t agc_orbit_params_select_body(bool moon, agc_dp_t rls_mag_or_rpad,
                                          agc_orbit_params_in_t *in)
{
    if (!AGC_ASSERT(in != NULL) || !AGC_ASSERT(rls_mag_or_rpad >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    in->moon     = moon;
    in->tff_rtmu = moon ? OPD_1_RTMUM : OPD_1_RTMUE;
    in->hpermin  = moon ? OPD_MINPERM : OPD_MINPERE;
    in->rpadtem  = rls_mag_or_rpad;
    rz_vflag_set(RZ_VF_ORBIT_DISPLAY_MOON, moon);

    return (AGC_STATUS_OK);
}

/* SR2R if moon centred (2^27 -> 2^29 for display), then MAXCHK. */
static agc_dp_t opd_display_height(agc_interp_t *ip, agc_dp_t h, bool moon)
{
    agc_dp_t x = moon ? agc_dp_shr_round(h, 2) : h;

    return (agc_orbit_params_maxchk(ip, x));
}

/* ---- SR30.1 (pp.522-523) ----------------------------------------------- */
agc_status_t agc_orbit_params_compute(agc_interp_t                *ip,
                                      const agc_orbit_params_in_t *in,
                                      agc_orbit_params_out_t      *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_tff_t tff;

    tff.rone = in->moon ? agc_vec_shl(ip, in->rone, 2) : in->rone; /* VSL2 */
    tff.vone = in->moon ? agc_vec_shl(ip, in->vone, 2) : in->vone;
    tff.rtmu = in->tff_rtmu;
    agc_status_t st = agc_tff_conic(ip, &tff); /* TFFCONMU      */

    if (st == AGC_STATUS_OK)
    {
        st = agc_tff_rp_ra(ip, &tff); /* TFFRP/RA      */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    out->rapo  = tff.rapo;
    out->rper  = tff.rper;
    out->hapox = opd_display_height(ip, agc_dp_sub(ip, tff.rapo, in->rpadtem),
                                    in->moon);              /* STORHAPO      */
    agc_dp_t hper = agc_dp_sub(ip, tff.rper, in->rpadtem);  /* MPAC +4       */

    out->hperx    = opd_display_height(ip, hper, in->moon); /* STORHPER      */
    out->neg_tper = 0;                                      /* HI6ZEROS      */

    if (agc_dp_sub(ip, hper, in->hpermin) >= 0)
    { /* DOTPER        */
        agc_dp_t tper = 0;

        st            = agc_tff_calctper(ip, &tff, tff.rper, &tper);
        out->neg_tper = agc_dp_neg(tper); /* DCOMP         */
    }

    agc_dp_t     t   = 0;
    agc_status_t st2 =
        agc_tff_calctff(ip, &tff, agc_dp_add(ip, in->hpermin, in->rpadtem), &t);

    out->tff = agc_dp_neg(t); /* DCOMP, TFF    */

    return ((st != AGC_STATUS_OK) ? st : st2);
}

/* SR30.1 on the rendezvous erasables (RONE VONE TFF/RTMU HPERMIN RPADTEM
 * V82EMFLG -> HAPOX HPERX TFF -TPER).                                  */
static agc_status_t opd_sr30_1(agc_interp_t *ip, agc_rendz_t *rz,
                               const agc_orbit_params_in_t *in)
{
    agc_orbit_params_out_t o;
    agc_status_t           st = agc_orbit_params_compute(ip, in, &o);

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

/* Inputs of SR30.1 for a state about the moon (moon) or the earth. */
static agc_status_t opd_input(agc_interp_t *ip, agc_rendz_t *rz, bool moon,
                              agc_orbit_params_in_t *in)
{
    in->rone = rz->rone;
    in->vone = rz->vone;
    agc_dp_t pad = moon ? rz_abval(ip, agc_erase_rls()) : RZ_RPAD_B29;

    return (agc_orbit_params_select_body(moon, pad, in));
}

/* ======================== verb 82 jobs and task ========================== */
/* Job prologue. */
static agc_rendz_t *opd_begin(agc_interp_t *ip)
{
    agc_interp_init(ip);

    return (rz_state());
}

/* GOXDSPF (= GOMARKF) with the extended-verb returns. */
static agc_status_t opd_flash(const agc_job_ctx_t *ctx, uint8_t verb,
                              uint8_t noun, agc_job_id_t term,
                              agc_job_id_t proceed, agc_job_id_t enter)
{
    return (rz_flash(ctx, AGC_DSKY_GOMARKF, verb, noun, term, proceed, enter));
}

/* EXTVBACT bit 5 clear: the crew has ended the verb 82 display. */
static bool opd_terminated(void)
{
    return ((agc_erase_extvbact() & OPD_EXTVBACT_BIT5) == 0U);
}

/* V82CALL (p.515): average G on -> V82GON, off -> V82GOFF. */
agc_status_t agc_job_orbit_parameter_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (rz_vflag_test(RZ_VF_AVEGFLAG))
    { /* V82GON        */
        agc_status_t st =
            agc_exec_findvac(OPD_PRIO7, AGC_JOB_ORBIT_DISPLAY_AVERAGE_G);

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

/* V82GOFF (p.515): V04N12 flashing; TERMINATE -> ENDEXT, PROCEED ->
 * start, ENTER (data in, "TC -5") -> the same display again.          */
agc_status_t agc_job_orbit_display_vehicle_select(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(rz_option(1) == 2U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (opd_flash(ctx, 4U, 12U, AGC_JOB_RENDZ_ENDEXT,
                      AGC_JOB_ORBIT_DISPLAY_VEHICLE_SELECTED,
                      AGC_JOB_ORBIT_DISPLAY_VEHICLE_SELECT));
}

/* PROCEED on V04N12: WAITLIST 80 ms TICKTEST, then V82GOFLP. */
agc_status_t agc_job_orbit_display_vehicle_selected(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_waitlist_call(OPD_BIT4_CS, AGC_TASK_TICKTEST);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_orbit_display_recycle(ctx));
}

/* V82GOFLP (p.515), the major recycle loop: V82FLAGS = 0 (TICKTEST does
 * not tick), FINDVAC PRIO7 V82GOFF1, then V82STALL.                    */
agc_status_t agc_job_orbit_display_recycle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_state()->orbit_display_flags = 0;
    agc_status_t st = agc_exec_findvac(OPD_PRIO7, AGC_JOB_ORBIT_DISPLAY_UPDATE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_orbit_display_stall(ctx));
}

/* V82STALL / FLAGGON (p.516): hold V16N44 until V82GOFF1 sets a tick bit
 * (DELAYJOB 1 s per try); then V16N44: TERM, PROCEED -> B5OFF,
 * RECYCLE -> V82GOFLP.                                                 */
agc_status_t agc_job_orbit_display_stall(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((rz_state()->orbit_display_flags & OPD_TICK_MASK) == 0)
    {
        return (agc_svc_delayjob(RZ_1SEC_CS, AGC_JOB_ORBIT_DISPLAY_STALL));
    }

    return (opd_flash(ctx, 16U, 44U, AGC_JOB_RENDZ_DISPLAY_TERMINATE,
                      AGC_JOB_RENDZ_DISPLAY_TERMINATE,
                      AGC_JOB_ORBIT_DISPLAY_RECYCLE));
}

/* SPLRET1 .. TICKTFF (pp.517-518): the parameters were computed at
 * TSTART82; correct the ticking field for the time since then and tell
 * TICKTEST which field to tick (-TPER when TFF was not computable,
 * otherwise TFF). ENDOFJOB.                                            */
static agc_status_t opd_start_ticking(agc_interp_t *ip, agc_rendz_t *rz)
{
    rz->tstart_orbit = agc_dp_sub(ip, agc_rtb_loadtime(), rz->tstart_orbit);

    if (rz->neg_tper != 0)
    { /* TICKTPER      */
        rz->neg_tper =
            agc_dp_add(ip, rz->neg_tper, rz->tstart_orbit);
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

/* V82GOFF1 / BOTHSHIP (pp.516-517): TDEC1 = TSTART82 = now; THISPREC
 * (OPTIONX+1 = 1) or OTHPREC; RONE VONE = RATT VATT; earth or moon
 * parameters by the X2 the integration left; SR30.1. CM in P00: CANDEL
 * (DELRSPL) first. An INTSTALL sleep resumes this job from its start. */
agc_status_t agc_job_orbit_display_update(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz  = opd_begin(&ip);
    agc_dp_t     now = agc_rtb_loadtime();     /* RTB LOADTIME  */

    rz->tstart_orbit = now;
    rz_integ_kind_t kind = (rz_option(2) == 1U) ? RZ_THISPREC : RZ_OTHPREC;
    agc_integ_out_t o;
    agc_status_t    st =
        rz_integ(&ip, kind, now, AGC_JOB_ORBIT_DISPLAY_UPDATE, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->rone = o.ratt; /* BOTHSHIP      */
    rz->vone = o.vatt;
    agc_orbit_params_in_t in;

    st = opd_input(&ip, rz, o.x2 != 0, &in);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = opd_sr30_1(&ip, rz, &in);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (rz_is_csm() && agc_erase_modreg() == 0)
    { /* CANDEL        */
        return (agc_job_orbit_display_splash(ctx));
    }

    return (opd_start_ticking(&ip, rz));
}

/* V82GON1 (p.519), average G on: RONE VONE = RN VN; moon (CM AMOONFLG,
 * LM MOONTHIS) or earth parameters; SR30.1; CM in P11: CANDEL; then
 * V82GON3.                                                             */
agc_status_t agc_job_orbit_display_average_g(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = opd_begin(&ip);

    rz->rone = agc_erase_rn(); /* RN, VN at 2^29, 2^7 (R30 p.519) */
    rz->vone = agc_erase_vn();
    bool moon =
        rz_is_csm() ? rz_vflag_test(RZ_VF_AMOONFLG) : rz_moonthis();
    agc_orbit_params_in_t in;
    agc_status_t          st = opd_input(&ip, rz, moon, &in);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = opd_sr30_1(&ip, rz, &in);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (rz_is_csm() &&
        agc_phase_checkmm(OPD_MODE_EARTH_ORBIT_INSERTION_MONITOR))
    {
        return (agc_job_orbit_display_splash(ctx));
    }

    return (agc_job_orbit_display_repeat(ctx));
}

/* V82GON3 (p.520): EXTVBACT bit 5 clear -> ENDEXT; else DELAYJOB 1 s,
 * V82GON1 again.                                                       */
agc_status_t agc_job_orbit_display_repeat(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (opd_terminated())
    {
        return (agc_dsky_endext());
    }

    return (agc_svc_delayjob(RZ_1SEC_CS, AGC_JOB_ORBIT_DISPLAY_AVERAGE_G));
}

/* CANDEL / SPLRET (CM p.520): INTSTALL, DELRSPL with TFF (it does the
 * INTWAKE); in P00 -> SPLRET1 (tick set-up), otherwise V82GON3.
 * DEVIATION: when DELRSPL fails (not yet translated by its lane) this
 * job does the INTWAKE itself so the integration area is released.    */
agc_status_t agc_job_orbit_display_splash(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_rendz_t *rz = opd_begin(&ip);
    agc_status_t st = agc_integ_install(0, AGC_JOB_ORBIT_DISPLAY_SPLASH);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* asleep        */
    }

    st = agc_guid_delrspl(&ip, rz->tff);

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

    return (agc_job_orbit_display_repeat(ctx));
}

/* V82REDSP (p.519): V16N44 monitor; TERM, PROCEED -> B5OFF (V82GON1
 * stops), RECYCLE -> the display again.                                */
agc_status_t agc_job_orbit_display_monitor(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (opd_flash(ctx, 16U, 44U, AGC_JOB_RENDZ_DISPLAY_TERMINATE,
                      AGC_JOB_RENDZ_DISPLAY_TERMINATE,
                      AGC_JOB_ORBIT_DISPLAY_MONITOR));
}

/* TICKTEST (p.518), waitlist task once a second while EXTVBACT bit 5 is
 * set: tick -TPER (V82FLAGS bit 1) or TFF (bit 2) by 1 s. When the bit
 * is clear: NOVAC PRIO25 ENDEXT (ENDEXT cannot run in a rupt).        */
agc_status_t agc_task_ticktest(void)
{
    if (opd_terminated())
    {
        return (agc_exec_novac(OPD_PRIO25, AGC_JOB_RENDZ_ENDEXT));
    }

    agc_status_t st   = agc_waitlist_call(RZ_1SEC_CS, AGC_TASK_TICKTEST);
    agc_rendz_t *rz   = rz_state();
    agc_sp_t     bits = (agc_sp_t) (rz->orbit_display_flags & OPD_TICK_MASK);

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

/* B5OFF (SERVICE_ROUTINES): clear EXTVBACT bit 5, ENDOFJOB. The verb 82
 * and 83 loops see the bit and end themselves.                         */
agc_status_t agc_job_rendz_display_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_svc_b5off());
}

/* ENDEXT as a job (TICKTEST's NOVAC; V82GOFF TERMINATE). */
agc_status_t agc_job_rendz_endext(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_endext());
}
