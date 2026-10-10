/*
 * rendezvous_parameter_display.c -- R31 RENDEZVOUS PARAMETER DISPLAY,
 * verb 83, both vehicles, and the CM R34 variant (R31.agc, Comanche055
 * pp.505-510, Luminary099 pp.703-708).
 *
 * A PRIO3 job extrapolates both state vectors to the present time (or to
 * PIPTIME while average G runs) and computes, in a loop, RANGE and RRATE
 * of the other vehicle and RTHETA, the angle of the vehicle axis (CM +X,
 * LM +Z; CM R34: the sextant line of sight) above the local horizontal;
 * V16N54 (CM R34: V16N53) monitors them until the crew ends it.
 *
 * The first pass saves precision "base vectors" of both vehicles; later
 * passes integrate from them (conic, or precision for a landed LM).
 * Vehicle differences (two different listings): the CM base vector of
 * the other vehicle is the LM; with average G on the CM uses RN VN and
 * integrates the LM, the LM uses RN VN and either R(CSM) V(CSM) of its
 * own servicer (MUNFLAG) or a precision CSM integration. The CM display
 * delay is a DELAYJOB loop, the LM's a FIXDELAY task.
 *
 * Scales: r m 2^29, v m/cs 2^7, RANGE m 2^29, RRATE m/cs 2^7, RTHETA
 * revolutions, base vectors body scale, times cs 2^28.
 */
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "agc/io/service_internal.h"
#include "agc/programs/alignment.h"

#define RPD_PRIO3          ((agc_prio_t) 03)
#define RPD_PRIO5          ((agc_prio_t) 05)
#define RPD_EXTVBACT_BIT5  ((uint16_t) 020)   /* display still wanted   */
#define RPD_EXTVBACT_BIT12 ((uint16_t) 04000) /* first values computed  */

/* ---- COMPDISP ----------------------------------------------------------- */
agc_status_t agc_rendezvous_parameter_display_compdisp(
    agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t rone, agc_vec_t vone,
    agc_vec_t ratt, agc_vec_t vatt, agc_vec_t axis_sm,
    const agc_mat_t *refsmmat)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(refsmmat != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t los = agc_vec_sub(ip, ratt, rone); /* 0D        */

    rz->range = rz_abval(ip, los);           /* B-29      */
    agc_vec_t ulos = rz_unit(ip, los);

    rz->rrate =
        agc_dp_shl(ip, agc_vec_dot(ip, agc_vec_sub(ip, vatt, vone), ulos),
                   1);                                 /* SL1 B-7   */
    agc_vec_t ur = rz_unit(ip, rone);                  /* 0D        */
    agc_vec_t ux = agc_vec_mat(ip, axis_sm, refsmmat); /* 6D, /4    */
    agc_vec_t p  = agc_vec_shl(ip, agc_vec_proj(ip, ux, ur), 2);
    agc_vec_t up = rz_unit(ip, agc_vec_sub(ip, ux, p)); /* 12D       */
    agc_vec_t h  = agc_vec_cross(ip, agc_vec_cross(ip, ur, vone), ur);
    agc_dp_t  s  = agc_vec_dot(ip, h, up); /* SIGN      */
    agc_dp_t  c;

    if (rz_is_csm())
    { /* VSL1 DOT  */
        c = agc_dp_shl(ip, agc_vec_dot(ip, agc_vec_shl(ip, up, 1), ux), 1);
        c = agc_dp_sign(c, s);
    }
    else
    { /* DOT SL2   */
        c = agc_dp_shl(ip, agc_dp_sign(agc_vec_dot(ip, up, ux), s), 2);
    }

    rz->rtheta = agc_dp_acos(ip, c);

    if (agc_vec_dot(ip, rz_is_csm() ? rone : ur, ux) < 0)
    { /* BPL       */
        rz->rtheta = agc_dp_sub(ip, RZ_DPPOSMAX, rz->rtheta);
    }

    if (!AGC_ASSERT(rz->range >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* The vehicle axis in stable-member axes (CDUTRIG, *NBSM*): CM +X (R31)
 * or the sextant LOS from CDUS CDUT (R34LOS, SXTNB); LM +Z.            */
static agc_status_t rpd_axis(agc_interp_t *ip, agc_vec_t *axis_sm)
{
    agc_vec_t a = agc_vec_make(0, 0, AGC_DP_HALF); /* UNITZ     */

    if (rz_is_csm())
    {
        a = agc_vec_make(AGC_DP_HALF, 0, 0); /* UNITX     */

        if (!rz_vflag_test(RZ_VF_RENDZ_DISPLAY))
        { /* R34       */
            agc_status_t st =
                agc_csm_sxtnb(ip, agc_erase_cdut(), agc_erase_cdus(), &a);

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }
        }
    }

    agc_cdutrig_t t;
    agc_status_t  st = agc_guid_cdu_nbsm(&t, &a); /* CDU*NBSM  */

    *axis_sm = a;

    return (st);
}

/* COMPDISP and its tail: EXTVBACT bit 5 clear -> ENDEXT; else set bit 12
 * (values ready) and go round again (V83 / REV83).                     */
static agc_status_t rpd_compdisp(agc_interp_t *ip, agc_rendz_t *rz,
                                 agc_vec_t ratt, agc_vec_t vatt)
{
    agc_vec_t    axis;
    agc_status_t st = rpd_axis(ip, &axis);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_mat_t refsmmat = agc_erase_refsmmat();

    st = agc_rendezvous_parameter_display_compdisp(ip, rz, rz->rone, rz->vone,
                                                   ratt, vatt, axis, &refsmmat);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    uint16_t x = agc_erase_extvbact();

    if ((x & RPD_EXTVBACT_BIT5) == 0U)
    {
        return (rz_endext()); /* YES, DIE  */
    }

    agc_svc_set_extvbact((uint16_t) (x | RPD_EXTVBACT_BIT12));

    return (rz_continue(AGC_JOB_RENDEZVOUS_PARAMS_UPDATE));
}

/* INTSTALL + INTEGRVS of a base vector to tdec1 (MOONFLAG = MOONTHIS). */
static agc_status_t rpd_from_base(agc_interp_t *ip, const agc_rendz_t *rz,
                                  bool this_base, agc_dp_t tdec1, bool conic,
                                  agc_integ_out_t *o)
{
    rz_integrvs_in_t in;

    in.rcv   = this_base ? rz->basethp : rz->baseotp;
    in.vcv   = this_base ? rz->basethv : rz->baseotv;
    in.tet   = rz->basetime;
    in.tdec1 = tdec1;
    in.moon  = rz_moonthis();
    in.conic = conic;

    return (rz_integrvs(ip, &in, AGC_JOB_RENDEZVOUS_PARAMS_UPDATE, o));
}

/* GETRVN: RONE = RN, VONE = VN; returns PIPTIME. */
static agc_dp_t rpd_getrvn(agc_rendz_t *rz)
{
    rz->rone = agc_erase_rn();
    rz->vone = agc_erase_vn();
    agc_dp_t t = agc_erase_piptime();

    if (!AGC_ASSERT(t >= 0))
    {
        return (0); /* no valid result */
    }

    return (t);
}

/* ---- CM: HAVEBASE (pp.508-510), every pass ------------------------------
 * Average G on (GETRVN5): RONE VONE = RN VN at PIPTIME; the LM by LEMPREC
 * when landed, else a precision INTEGRVS of its base vector. Off: the CM
 * base vector to now (conic, precision when the LM is landed) gives RONE
 * VONE; the LM by LEMPREC when landed, else conic from its base vector.
 * An INTSTALL sleep resumes the update job from its start.            */
static agc_status_t rpd_csm_havebase(agc_interp_t *ip, agc_rendz_t *rz)
{
    bool            surf = agc_flag_test(AGC_FLAG_SURFFLAG);
    agc_integ_out_t o;
    agc_dp_t        t;
    bool            conic = !surf;
    agc_status_t    st;

    if (rz_vflag_test(RZ_VF_CHANGE_PROGRAM))
    { /* GETRVN5       */
        t     = rpd_getrvn(rz);
        conic = false; /* CLEAR INTYPFLG */
    }
    else
    {
        st = rpd_from_base(ip, rz, true, rz_loadtime(), conic, &o);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        rz->rone = o.ratt;
        rz->vone = o.vatt;
        t        = o.tat;
    }

    if (surf)
    { /* GETRVN6       */
        st = rz_integ(ip, RZ_LEMPREC, t, AGC_JOB_RENDEZVOUS_PARAMS_UPDATE, &o);
    }
    else
    { /* OTHINT        */
        st = rpd_from_base(ip, rz, false, t, conic, &o);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rpd_compdisp(ip, rz, o.ratt, o.vatt));
}

/* ---- CM: STATEXTP (p.508), the first pass ------------------------------
 * BASETIME = now (PIPTIME, with RONE VONE = RN VN, while average G is
 * on); LM precision base vector; average G on: COMPDISP; off: CM
 * precision base vector at TAT, then HAVEBASE.                        */
static agc_status_t rpd_csm_statextp(agc_interp_t *ip, agc_rendz_t *rz)
{
    bool     aveg = rz_vflag_test(RZ_VF_CHANGE_PROGRAM); /* V37FLAG       */
    agc_dp_t t    = aveg ? rpd_getrvn(rz) : rz_loadtime();

    rz->basetime = t;
    agc_integ_out_t o;
    agc_status_t    st =
        rz_integ(ip, RZ_LEMPREC, t, AGC_JOB_RENDEZVOUS_PARAMS_START, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->baseotp = o.ratt1; /* LM base       */
    rz->baseotv = o.vatt1;

    if (aveg)
    {
        return (rpd_compdisp(ip, rz, o.ratt, o.vatt));
    }

    st = rz_integ(ip, RZ_CSMPREC, o.tat, AGC_JOB_RENDEZVOUS_PARAMS_START, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->basethp = o.ratt1; /* CM base       */
    rz->basethv = o.vatt1;

    return (rpd_csm_havebase(ip, rz));
}

/* ---- LM: GETRVN (p.706) -------------------------------------------------
 * RONE VONE = RN VN; with MUNFLAG the CSM state of the LM servicer
 * (R(CSM) V(CSM), stable member) in reference axes (VXM REFSMMAT, VSR4 /
 * VSL1); otherwise a precision integration of the CSM base vector to
 * PIPTIME. DEVIATION: the AGC raises the job to PRIO22 while it copies
 * RN VN R(CSM) V(CSM) so that they come from one SERVICER pass; the C
 * job reads them without interruption.                                */
static agc_status_t rpd_lm_getrvn(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_dp_t t = rpd_getrvn(rz);

    if (rz_vflag_test(RZ_VF_MUNFLAG))
    {
        const agc_guid_lm_t *g = &agc_guid_state()->lm;
        agc_mat_t            m = agc_erase_refsmmat();
        agc_vec_t            r = agc_vec_shr(agc_vec_mat(ip, g->r_csm, &m), 4);
        agc_vec_t            v = agc_vec_shl(ip, agc_vec_mat(ip, g->v_csm,
                                                             &m), 1);

        return (rpd_compdisp(ip, rz, r, v));
    }

    agc_integ_out_t o;
    agc_status_t    st = rpd_from_base(ip, rz, false, t, false, &o); /* GETRVN2 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rpd_compdisp(ip, rz, o.ratt, o.vatt));
}

/* ---- LM: REV83 (pp.704-705), every pass ---------------------------------
 * Average G on: GETRVN. Landed (R31SURF): LM by LEMPREC, the CSM conic
 * from its base vector (OTHCONIC). Otherwise both conic from their base
 * vectors. An INTSTALL sleep resumes the update job from its start.   */
static agc_status_t rpd_lm_rev83(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (rz_vflag_test(RZ_VF_AVEGFLAG))
    {
        return (rpd_lm_getrvn(ip, rz));
    }

    agc_integ_out_t o;
    agc_status_t    st;

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    { /* R31SURF       */
        st = rz_integ(ip, RZ_LEMPREC, rz_loadtime(),
                      AGC_JOB_RENDEZVOUS_PARAMS_UPDATE, &o);
    }
    else
    {
        st = rpd_from_base(ip, rz, true, rz_loadtime(), true, &o);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->rone = o.ratt; /* OTHCONIC      */
    rz->vone = o.vatt;
    st       = rpd_from_base(ip, rz, false, o.tat, true, &o); /* OTHINT        */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rpd_compdisp(ip, rz, o.ratt, o.vatt));
}

/* ---- LM: V83CALL (pp.703-704), the first pass ---------------------------
 * Average G on and MUNFLAG set: no base vectors (GETRVN). Average G on or
 * landed (ONEBASE): only the CSM base vector, at now. Otherwise the LM
 * precision base vector at now, the CSM's at its TAT (DOCMBASE). Then
 * REV83.                                                               */
static agc_status_t rpd_lm_v83call(agc_interp_t *ip, agc_rendz_t *rz)
{
    bool aveg = rz_vflag_test(RZ_VF_AVEGFLAG);

    if (aveg && rz_vflag_test(RZ_VF_MUNFLAG))
    { /* MUNG?         */
        return (rpd_lm_getrvn(ip, rz));
    }

    agc_integ_out_t o;
    agc_status_t    st;
    agc_dp_t        t = rz_loadtime();

    if (!aveg && !agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        st = rz_integ(ip, RZ_LEMPREC, t, AGC_JOB_RENDEZVOUS_PARAMS_START, &o);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        rz->basethp = o.ratt1; /* LM base       */
        rz->basethv = o.vatt1;
        t           = o.tat;
    }

    rz->basetime = t; /* DOCMBASE      */
    st           = rz_integ(ip, RZ_CSMPREC, t, AGC_JOB_RENDEZVOUS_PARAMS_START,
                            &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->baseotp = o.ratt1; /* CM base       */
    rz->baseotv = o.vatt1;

    return (rpd_lm_rev83(ip, rz));
}

/* ============================ jobs and task ============================== */
/* R31CALL (CM p.505, LM p.703): FINDVAC PRIO3 V83CALL; then wait (CM
 * DELAYJOB loop, LM FIXDELAY task) until EXTVBACT bit 12 shows the
 * first values, then DISPN5X.                                         */
agc_status_t agc_job_rendezvous_parameter_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st =
        agc_exec_findvac(RPD_PRIO3, AGC_JOB_RENDEZVOUS_PARAMS_START);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (rz_is_csm())
    {
        return (rz_delayjob(RZ_1SEC_CS, AGC_JOB_RENDEZVOUS_PARAMS_WAIT));
    }

    return (agc_waitlist_call(RZ_1SEC_CS, AGC_TASK_RENDEZVOUS_PARAMS_WAIT));
}

/* CM DSPDELAY: bit 12 clear -> DELAYJOB 1 s again; else DISPN5X. */
agc_status_t agc_job_rendezvous_params_wait(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((agc_erase_extvbact() & RPD_EXTVBACT_BIT12) == 0U)
    {
        return (rz_delayjob(RZ_1SEC_CS, AGC_JOB_RENDEZVOUS_PARAMS_WAIT));
    }

    return (agc_job_rendezvous_params_monitor(ctx));
}

/* LM DSPDELAY task: bit 12 clear -> FIXDELAY 100 again; else NOVAC PRIO5
 * DISPN5X; TASKOVER.                                                    */
agc_status_t agc_task_rendezvous_params_wait(void)
{
    if ((agc_erase_extvbact() & RPD_EXTVBACT_BIT12) == 0U)
    {
        return (agc_waitlist_call(RZ_1SEC_CS, AGC_TASK_RENDEZVOUS_PARAMS_WAIT));
    }

    return (agc_exec_novac(RPD_PRIO5, AGC_JOB_RENDEZVOUS_PARAMS_MONITOR));
}

/* DISPN5X: V16N54 (CM R34: V16N53) GOMARKF; TERMINATE, PROCEED -> B5OFF,
 * RECYCLE -> again.                                                    */
agc_status_t agc_job_rendezvous_params_monitor(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint8_t noun = 54U;

    if (rz_is_csm() && !rz_vflag_test(RZ_VF_RENDZ_DISPLAY))
    {
        noun = 53U; /* R34           */
    }

    return (rz_flash(
                ctx, AGC_DSKY_GOMARKF, 16U, noun,
                AGC_JOB_RENDZ_DISPLAY_TERMINATE,
                AGC_JOB_RENDZ_DISPLAY_TERMINATE,
                AGC_JOB_RENDEZVOUS_PARAMS_MONITOR));
}

/* V83CALL: the first pass (CM STATEXTP, LM V83CALL). */
agc_status_t agc_job_rendezvous_params_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();

    return (rz_is_csm() ? rpd_csm_statextp(&ip, rz) : rpd_lm_v83call(&ip, rz));
}

/* V83 / REV83: every later pass (CM HAVEBASE, LM REV83). */
agc_status_t agc_job_rendezvous_params_update(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();

    return (rz_is_csm() ? rpd_csm_havebase(&ip, rz) : rpd_lm_rev83(&ip, rz));
}
