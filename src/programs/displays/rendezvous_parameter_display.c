/**********************************************************************
 * FILE NAME: rendezvous_parameter_display.c
 *
 * PURPOSE:
 *   R31 RENDEZVOUS PARAMETER DISPLAY (verb 83), both vehicles, and the
 *   CM R34 variant. A PRIO3 job extrapolates both state vectors to the
 *   present time (or to PIPTIME while average G runs) and computes, in a
 *   loop, RANGE and RRATE of the other vehicle and RTHETA, the angle of
 *   the vehicle axis (CM +X, LM +Z; CM R34: the sextant line of sight)
 *   above the local horizontal; V16N54 (CM R34: V16N53) monitors them
 *   until the crew ends it. The first pass saves precision "base
 *   vectors" of both vehicles; later passes integrate from them (conic,
 *   or precision for a landed LM).
 *
 * AGC SOURCE:
 *   Comanche055 R31.agc pp. 505-510 (R31CALL, V83CALL, STATEXTP,
 *   HAVEBASE, GETRVN5, GETRVN6, OTHINT, COMPDISP, DSPDELAY, DISPN5X,
 *   R34LOS)
 *   Luminary099 R31.agc pp. 703-708 (differs: base vectors and average G
 *   handling (MUNFLAG: R(CSM) V(CSM) of the LM servicer), DSPDELAY is a
 *   FIXDELAY task, RTHETA from the +Z axis)
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   RN VN PIPTIME          I    2^29, 2^7, cs 2^28  average-G state
 *   R(CSM) V(CSM) agc_vec_t I   2^27, 2^5  LM servicer CSM state (SM)
 *   REFSMMAT    agc_mat_t  I    2^1        stable member to reference
 *   CDUS CDUT   words      I    -          sextant angles (CM R34)
 *   EXTVBACT    word       I/O  -          bit 5 wanted, bit 12 ready
 *   SURFFLAG AVEGFLAG MUNFLAG V37FLAG  flags  I  -  state choice
 *   RONE VONE BASETHP BASETHV BASEOTP BASEOTV BASETIME (agc_rendz_t)
 *               I/O  body scale, cs 2^28   base vectors
 *   RANGE RRATE RTHETA (agc_rendz_t) O  m 2^29, m/cs 2^7, rev  N54
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ----------------------------------------------
 *   rz_integ rz_integrvs  CSMPREC / LEMPREC, INTEGRVS from a base vector
 *   agc_csm_sxtnb         SXTNB: sextant LOS in navigation base axes
 *   agc_guid_cdu_nbsm     CDUTRIG, *NBSM*: navigation base to SM
 *   rz_flash              GOMARKF display request
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms of its own (FINDVAC raises the executive's alarm when no
 *   VAC area is free). AGC_STATUS_BAD_ARG for NULL pointers; display,
 *   waitlist and integration statuses are passed back (an INTSTALL
 *   sleep resumes the job from its start).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Extended verb jobs (PRIO3 loop) and, in the LM, waitlist tasks.
 *   DEVIATION (LM GETRVN): the AGC raises the job to PRIO22 while it
 *   copies RN VN R(CSM) V(CSM); the C job reads them without a break.
 *
 * NOTES:
 *   Scales: r m 2^29, v m/cs 2^7, RANGE m 2^29, RRATE m/cs 2^7, RTHETA
 *   revolutions, base vectors body scale, times cs 2^28.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"

#define RPD_PRIO3          ((agc_prio_t) 03)  /* V83CALL job            */
#define RPD_PRIO5          ((agc_prio_t) 05)  /* DISPN5X job (LM)       */
#define RPD_EXTVBACT_BIT5  ((uint16_t) 020)   /* display still wanted   */
#define RPD_EXTVBACT_BIT12 ((uint16_t) 04000) /* first values computed  */

/**********************************************************************
 * COMPDISP
 * AGC: COMPDISP (R31.agc p.509)
 *
 * FUNCTION NAME: agc_rendezvous_parameter_display_compdisp
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *      I/O  rendezvous erasables
 *   rone      agc_vec_t          I    this vehicle position, m 2^29
 *   vone      agc_vec_t          I    this vehicle velocity, m/cs 2^7
 *   ratt      agc_vec_t          I    other vehicle position, m 2^29
 *   vatt      agc_vec_t          I    other vehicle velocity, m/cs 2^7
 *   axis_sm   agc_vec_t          I    the vehicle axis in SM axes, unit 2^1
 *   refsmmat  const agc_mat_t *  I    REFSMMAT, 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a bad
 *          argument
 **********************************************************************/
agc_status_t agc_rendezvous_parameter_display_compdisp(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t rone, agc_vec_t vone, agc_vec_t ratt, agc_vec_t vatt, agc_vec_t axis_sm, const agc_mat_t *refsmmat)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(refsmmat != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t los = agc_vec_sub(ip, ratt, rone); /* 0D LOS, m 2^29 */

    rz->range      = rz_abval(ip, los); /* B-29      */
    agc_vec_t ulos = rz_unit(ip, los);  /* 2^1 */

    rz->rrate    = agc_dp_shl(ip, agc_vec_dot(ip, agc_vec_sub(ip, vatt, vone), ulos), 1); /* SL1 B-7   */
    agc_vec_t ur = rz_unit(ip, rone);                                                     /* 0D, 2^1 */
    agc_vec_t ux = agc_vec_mat(ip, axis_sm, refsmmat);                                    /* 6D, 2^2 */
    agc_vec_t p  = agc_vec_shl(ip, agc_vec_proj(ip, ux, ur), 2);                          /* 2^2 */
    agc_vec_t up = rz_unit(ip, agc_vec_sub(ip, ux, p));                                   /* 12D, 2^1 */
    agc_vec_t h  = agc_vec_cross(ip, agc_vec_cross(ip, ur, vone), ur);                    /* 2^9 */
    agc_dp_t  s  = agc_vec_dot(ip, h, up);                                                /* SIGN, 2^10 */
    agc_dp_t  c;                                                                          /* cos(RTHETA), 2^1 */

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

    if (agc_vec_dot(ip, rz_is_csm() ? rone : ur, ux) < 0) /* CM RONE, LM UR */
    { /* BPL       */
        rz->rtheta = agc_dp_sub(ip, RZ_DPPOSMAX, rz->rtheta);
    }

    if (!AGC_ASSERT(rz->range >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* agc_rendezvous_parameter_display_compdisp */

/**********************************************************************
 * The vehicle axis in stable-member axes (CDUTRIG, *NBSM*): CM +X (R31)
 * or the sextant LOS from CDUS CDUT (R34LOS, SXTNB); LM +Z.
 * AGC: COMPDISP CDUTRIG, *NBSM*, R34LOS (R31.agc p.509)
 *
 * FUNCTION NAME: rpd_axis
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   axis_sm   agc_vec_t *     O    the vehicle axis in SM axes, unit 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the SXTNB or NBSM status
 **********************************************************************/
static agc_status_t rpd_axis(agc_interp_t *ip, agc_vec_t *axis_sm)
{
    agc_vec_t a = agc_vec_make(0, 0, AGC_DP_HALF); /* UNITZ, 2^1 */

    if (rz_is_csm())
    {
        a = agc_vec_make(AGC_DP_HALF, 0, 0); /* UNITX     */

        if (!rz_vflag_test(RZ_VF_RENDZ_DISPLAY))
        { /* R34       */
            agc_status_t st; /* SXTNB */

            st = agc_csm_sxtnb(ip, agc_erase_cdut(), agc_erase_cdus(), &a);

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }
        }
    }

    agc_cdutrig_t t;                              /* CDUTRIG sines/cosines */
    agc_status_t  st = agc_guid_cdu_nbsm(&t, &a); /* CDU*NBSM  */

    *axis_sm = a;

    return (st);
}

/**********************************************************************
 * COMPDISP and its tail: EXTVBACT bit 5 clear -> ENDEXT; else set bit 12
 * (values ready) and go round again (V83 / REV83).
 * AGC: COMPDISP and its tail (R31.agc pp.509-510)
 *
 * FUNCTION NAME: rpd_compdisp
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   ratt      agc_vec_t       I    other vehicle position, m 2^29
 *   vatt      agc_vec_t       I    other vehicle velocity, m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (next pass queued or display
 *          ended), else the status of the call that failed
 **********************************************************************/
static agc_status_t rpd_compdisp(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t ratt, agc_vec_t vatt)
{
    agc_vec_t    axis;                     /* SM axis, 2^1 */
    agc_status_t st = rpd_axis(ip, &axis); /* axis */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, 2^1 */

    st = agc_rendezvous_parameter_display_compdisp(ip, rz, rz->rone, rz->vone, ratt, vatt, axis, &refsmmat);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    uint16_t x = agc_erase_extvbact(); /* EXTVBACT */

    if ((x & RPD_EXTVBACT_BIT5) == 0U)
    {
        return (agc_dsky_endmark()); /* YES, DIE  */
    }

    agc_svc_set_extvbact((uint16_t) (x | RPD_EXTVBACT_BIT12));

    return (rz_continue(AGC_JOB_RENDEZVOUS_PARAMS_UPDATE));
}

/**********************************************************************
 * INTSTALL + INTEGRVS of a base vector to tdec1 (MOONFLAG = MOONTHIS).
 * AGC: INTSTALL, INTEGRVS (R31.agc p.508)
 *
 * FUNCTION NAME: rpd_from_base
 *
 * ARGUMENTS:
 *   Argument   Type                 I/O  Description (scale)
 *   ---------  -------------------  ---  --------------------------
 *   ip         agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz         const agc_rendz_t *  I    rendezvous erasables
 *   this_base  bool                 I    true: this vehicle's base vector,
 *                                        false: the other's
 *   tdec1      agc_dp_t             I    TDEC1, time to integrate to, cs
 *                                        2^28
 *   conic      bool                 I    true: conic (INTYPFLG set), false:
 *                                        precision
 *   o          agc_integ_out_t *    O    the integrated state (RATT VATT
 *                                        TAT)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration status
 *          (INTSTALL sleep)
 **********************************************************************/
static agc_status_t rpd_from_base(agc_interp_t *ip, const agc_rendz_t *rz, bool this_base, agc_dp_t tdec1, bool conic, agc_integ_out_t *o)
{
    rz_integrvs_in_t in; /* INTEGRVS inputs */

    in.rcv   = this_base ? rz->basethp : rz->baseotp; /* base position */
    in.vcv   = this_base ? rz->basethv : rz->baseotv; /* base velocity */
    in.tet   = rz->basetime;
    in.tdec1 = tdec1;
    in.moon  = rz_moonthis();
    in.conic = conic;

    return (rz_integrvs(ip, &in, AGC_JOB_RENDEZVOUS_PARAMS_UPDATE, o));
}

/**********************************************************************
 * GETRVN: RONE = RN, VONE = VN; returns PIPTIME.
 * AGC: GETRVN (R31.agc p.508)
 *
 * FUNCTION NAME: rpd_getrvn
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   rz        agc_rendz_t *  I/O  rendezvous erasables
 *
 * RETURNS: agc_dp_t -- PIPTIME, cs 2^28
 **********************************************************************/
static agc_dp_t rpd_getrvn(agc_rendz_t *rz)
{
    rz->rone   = agc_erase_rn();
    rz->vone   = agc_erase_vn();
    agc_dp_t t = agc_erase_piptime(); /* PIPTIME, cs 2^28 */

    if (!AGC_ASSERT(t >= 0))
    {
        return (0); /* no valid result */
    }

    return (t);
}

/**********************************************************************
 * CM: HAVEBASE (pp.508-510), every pass
 * Average G on (GETRVN5): RONE VONE = RN VN at PIPTIME; the LM by LEMPREC
 * when landed, else a precision INTEGRVS of its base vector. Off: the CM
 * base vector to now (conic, precision when the LM is landed) gives RONE
 * VONE; the LM by LEMPREC when landed, else conic from its base vector.
 * An INTSTALL sleep resumes the update job from its start.
 * AGC: HAVEBASE (R31.agc pp.508-510)
 *
 * FUNCTION NAME: rpd_csm_havebase
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t rpd_csm_havebase(agc_interp_t *ip, agc_rendz_t *rz)
{
    bool            surf  = agc_flag_test(AGC_FLAG_SURFFLAG); /* LM landed */
    agc_integ_out_t o;                                        /* integration result */
    agc_dp_t        t;                                        /* time of RONE, cs 2^28 */
    bool            conic = !surf;                            /* conic unless landed */
    agc_status_t    st;                                       /* integration status */

    if (rz_vflag_test(RZ_VF_CHANGE_PROGRAM))
    { /* GETRVN5       */
        t     = rpd_getrvn(rz);
        conic = false; /* CLEAR INTYPFLG */
    }
    else
    {
        st = rpd_from_base(ip, rz, true, agc_rtb_loadtime(), conic, &o);

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
} /* rpd_csm_havebase */

/**********************************************************************
 * CM: STATEXTP (p.508), the first pass
 * BASETIME = now (PIPTIME, with RONE VONE = RN VN, while average G is
 * on); LM precision base vector; average G on: COMPDISP; off: CM
 * precision base vector at TAT, then HAVEBASE.
 * AGC: STATEXTP (R31.agc p.508)
 *
 * FUNCTION NAME: rpd_csm_statextp
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t rpd_csm_statextp(agc_interp_t *ip, agc_rendz_t *rz)
{
    bool     aveg = rz_vflag_test(RZ_VF_CHANGE_PROGRAM);        /* V37FLAG       */
    agc_dp_t t    = aveg ? rpd_getrvn(rz) : agc_rtb_loadtime(); /* cs 2^28 */

    rz->basetime = t;
    agc_integ_out_t o;  /* integration result */
    agc_status_t    st; /* LM */

    st = rz_integ(ip, RZ_LEMPREC, t, AGC_JOB_RENDEZVOUS_PARAMS_START, &o);

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

/**********************************************************************
 * LM: GETRVN (p.706)
 * RONE VONE = RN VN; with MUNFLAG the CSM state of the LM servicer
 * (R(CSM) V(CSM), stable member) in reference axes (VXM REFSMMAT, VSR4 /
 * VSL1); otherwise a precision integration of the CSM base vector to
 * PIPTIME. DEVIATION: the AGC raises the job to PRIO22 while it copies
 * RN VN R(CSM) V(CSM) so that they come from one SERVICER pass; the C
 * job reads them without interruption.
 * AGC: GETRVN (Luminary099 R31.agc p.706)
 *
 * FUNCTION NAME: rpd_lm_getrvn
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t rpd_lm_getrvn(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_dp_t t = rpd_getrvn(rz); /* PIPTIME, cs 2^28 */

    if (rz_vflag_test(RZ_VF_MUNFLAG))
    {
        const agc_guid_lm_t *g = &agc_guid_state()->lm; /* LM servicer */
        agc_mat_t            m = agc_erase_refsmmat();  /* REFSMMAT, 2^1 */
        agc_vec_t            r;                         /* m 2^29 */

        r = agc_vec_shr(agc_vec_mat(ip, g->r_csm, &m), 4);
        agc_vec_t v; /* m/cs 2^7 */

        v = agc_vec_shl(ip, agc_vec_mat(ip, g->v_csm, &m), 1);

        return (rpd_compdisp(ip, rz, r, v));
    }

    agc_integ_out_t o;  /* integration result */
    agc_status_t    st; /* GETRVN2: CSM base to PIPTIME */

    st = rpd_from_base(ip, rz, false, t, false, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rpd_compdisp(ip, rz, o.ratt, o.vatt));
}

/**********************************************************************
 * LM: REV83 (pp.704-705), every pass
 * Average G on: GETRVN. Landed (R31SURF): LM by LEMPREC, the CSM conic
 * from its base vector (OTHCONIC). Otherwise both conic from their base
 * vectors. An INTSTALL sleep resumes the update job from its start.
 * AGC: REV83 (Luminary099 R31.agc pp.704-705)
 *
 * FUNCTION NAME: rpd_lm_update_states
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t rpd_lm_update_states(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (rz_vflag_test(RZ_VF_AVEGFLAG))
    {
        return (rpd_lm_getrvn(ip, rz));
    }

    agc_integ_out_t o;  /* integration result */
    agc_status_t    st; /* integration status */

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    { /* R31SURF       */
        st = rz_integ(ip, RZ_LEMPREC, agc_rtb_loadtime(), AGC_JOB_RENDEZVOUS_PARAMS_UPDATE, &o);
    }
    else
    {
        st = rpd_from_base(ip, rz, true, agc_rtb_loadtime(), true, &o);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->rone = o.ratt;                                        /* OTHCONIC      */
    rz->vone = o.vatt;
    st       = rpd_from_base(ip, rz, false, o.tat, true, &o); /* OTHINT */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rpd_compdisp(ip, rz, o.ratt, o.vatt));
}

/**********************************************************************
 * LM: V83CALL (pp.703-704), the first pass
 * Average G on and MUNFLAG set: no base vectors (GETRVN). Average G on or
 * landed (ONEBASE): only the CSM base vector, at now. Otherwise the LM
 * precision base vector at now, the CSM's at its TAT (DOCMBASE). Then
 * REV83.
 * AGC: V83CALL (Luminary099 R31.agc pp.703-704)
 *
 * FUNCTION NAME: rpd_lm_compute
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t rpd_lm_compute(agc_interp_t *ip, agc_rendz_t *rz)
{
    bool aveg = rz_vflag_test(RZ_VF_AVEGFLAG); /* average G on */

    if (aveg && rz_vflag_test(RZ_VF_MUNFLAG))
    { /* MUNG?         */
        return (rpd_lm_getrvn(ip, rz));
    }

    agc_integ_out_t o;                      /* integration result */
    agc_status_t    st;                     /* integration status */
    agc_dp_t        t = agc_rtb_loadtime(); /* now, cs 2^28 */

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
    st           = rz_integ(ip, RZ_CSMPREC, t, AGC_JOB_RENDEZVOUS_PARAMS_START, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->baseotp = o.ratt1; /* CM base       */
    rz->baseotv = o.vatt1;

    return (rpd_lm_update_states(ip, rz));
}

/* ============================ jobs and task ============================== */
/**********************************************************************
 * R31CALL (CM p.505, LM p.703): FINDVAC PRIO3 V83CALL; then wait (CM
 * DELAYJOB loop, LM FIXDELAY task) until EXTVBACT bit 12 shows the
 * first values, then DISPN5X.
 * AGC: R31CALL (R31.agc p.505)
 *
 * FUNCTION NAME: agc_job_rendezvous_parameter_display
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
agc_status_t agc_job_rendezvous_parameter_display(agc_job_ctx_t *ctx)
{
    agc_status_t st; /* FINDVAC */

    st = agc_exec_findvac(RPD_PRIO3, AGC_JOB_RENDEZVOUS_PARAMS_START);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (rz_is_csm())
    {
        return (agc_svc_delayjob(RZ_1SEC_CS, AGC_JOB_RENDEZVOUS_PARAMS_WAIT));
    }

    return (agc_waitlist_call(RZ_1SEC_CS, AGC_TASK_RENDEZVOUS_PARAMS_WAIT));
}

/**********************************************************************
 * R31CALL (LM p.703) is a waitlist task (V83PERF: WAITLIST 20 ms):
 * FINDVAC PRIO3 V83CALL, then DSPDELAY (FIXDELAY 1 s) in the task.
 * AGC: R31CALL (Luminary099 R31.agc p.703)
 *
 * FUNCTION NAME: agc_task_rendezvous_parameter_display
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_rendezvous_parameter_display(void)
{
    if (!AGC_ASSERT(!rz_is_csm()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st; /* FINDVAC */

    st = agc_exec_findvac(RPD_PRIO3, AGC_JOB_RENDEZVOUS_PARAMS_START);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_waitlist_call(RZ_1SEC_CS, AGC_TASK_RENDEZVOUS_PARAMS_WAIT));
}

/**********************************************************************
 * CM DSPDELAY: bit 12 clear -> DELAYJOB 1 s again; else DISPN5X.
 * AGC: DSPDELAY (R31.agc p.505)
 *
 * FUNCTION NAME: agc_job_rendezvous_params_wait
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
agc_status_t agc_job_rendezvous_params_wait(agc_job_ctx_t *ctx)
{
    if ((agc_erase_extvbact() & RPD_EXTVBACT_BIT12) == 0U)
    {
        return (agc_svc_delayjob(RZ_1SEC_CS, AGC_JOB_RENDEZVOUS_PARAMS_WAIT));
    }

    return (agc_job_rendezvous_params_monitor(ctx));
}

/**********************************************************************
 * LM DSPDELAY task: bit 12 clear -> FIXDELAY 100 again; else NOVAC PRIO5
 * DISPN5X; TASKOVER.
 * AGC: DSPDELAY (Luminary099 R31.agc p.703)
 *
 * FUNCTION NAME: agc_task_rendezvous_params_wait
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_rendezvous_params_wait(void)
{
    if ((agc_erase_extvbact() & RPD_EXTVBACT_BIT12) == 0U)
    {
        return (agc_waitlist_call(RZ_1SEC_CS, AGC_TASK_RENDEZVOUS_PARAMS_WAIT));
    }

    return (agc_exec_novac(RPD_PRIO5, AGC_JOB_RENDEZVOUS_PARAMS_MONITOR));
}

/**********************************************************************
 * DISPN5X: V16N54 (CM R34: V16N53) GOMARKF; TERMINATE, PROCEED -> B5OFF,
 * RECYCLE -> again.
 * AGC: DISPN5X (R31.agc p.505)
 *
 * FUNCTION NAME: agc_job_rendezvous_params_monitor
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
agc_status_t agc_job_rendezvous_params_monitor(agc_job_ctx_t *ctx)
{
    uint8_t noun = 54U; /* N54 (R31) */

    if (rz_is_csm() && !rz_vflag_test(RZ_VF_RENDZ_DISPLAY))
    {
        noun = 53U; /* R34           */
    }

    return (rz_flash(ctx, AGC_DSKY_GOMARKF, 16U, noun, AGC_JOB_RENDZ_DISPLAY_TERMINATE, AGC_JOB_RENDZ_DISPLAY_TERMINATE, AGC_JOB_RENDEZVOUS_PARAMS_MONITOR));
}

/**********************************************************************
 * V83CALL: the first pass (CM STATEXTP, LM V83CALL).
 * AGC: V83CALL (R31.agc p.506)
 *
 * FUNCTION NAME: agc_job_rendezvous_params_start
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
agc_status_t agc_job_rendezvous_params_start(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    if (rz_is_csm())
    {
        return (rpd_csm_statextp(&ip, rz)); /* CM STATEXTP */
    }

    return (rpd_lm_compute(&ip, rz)); /* LM V83CALL */
}

/**********************************************************************
 * V83 / REV83: every later pass (CM HAVEBASE, LM REV83).
 * AGC: V83, REV83 (R31.agc p.508)
 *
 * FUNCTION NAME: agc_job_rendezvous_params_update
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
agc_status_t agc_job_rendezvous_params_update(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    if (rz_is_csm())
    {
        return (rpd_csm_havebase(&ip, rz)); /* CM HAVEBASE */
    }

    return (rpd_lm_update_states(&ip, rz)); /* LM REV83 */
}
