/**********************************************************************
 * FILE NAME: external_delta_v.c
 *
 * PURPOSE:
 *   P30 external delta-V targeting (both vehicles), P31 general Lambert
 *   aimpoint targeting (Comanche; the Luminary P31 is in the rendezvous
 *   programs), the subroutines S30.1 and S31.1, MIDGIM, AGAIN and the
 *   display sequence P30/P31, CNTNUP30, PARAM30. The Comanche-only
 *   REFTEST, DISP45, END45, COMPTGO and DELRSPL are in
 *   src/csm/burns/external_delta_v.c.
 *
 * AGC SOURCE:
 *   Comanche055 P30-P37.agc pp. 635-648 (P30, P31, P30/P31, CNTNUP30,
 *     PARAM30, S30.1, S31.1); P40-P47.agc p.711 (AGAIN);
 *     P34-35_P74-75.agc p.493 (MIDGIM)
 *   Luminary099 P30_P37.agc pp. 614-617 (P30, S30.1; differs: LEMPREC,
 *     LV axes built inline, no P31)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   TIG           agc_dp_t   I    cs 2^28      time of ignition
 *   REFSMMAT      agc_mat_t  I    2^1          for +MGA
 *   UPDATFLG ...  flags      O    -            P30/P31 flag set-up
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_integ_thisprec    THISPREC / LEMPREC (nav-integ)
 *   agc_rendz_lomat       LOMAT; agc_rendz_get_lvc GET.LVC (prog-rendz)
 *   agc_rendz_initvel     INITVEL (AGAIN)
 *   agc_dsky_display      GOFLASH / GOFLASHR (io-pinball)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   AGC_STATUS_NOT_FOUND when an integration ends without RECTOUT;
 *   statuses of the integration and INITVEL passed up (an INTSTALL wait
 *   resumes the job named by agc_guid_nav_set_resume).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   P30/P31 jobs; group 4 restart protection in Comanche (4.1).
 *
 * NOTES:
 *   Results go to the rendezvous erasables (agc_rendz_t): RTIG VTIG
 *   m 2^29 / m/cs 2^7, DELVSIN DELVLVC m/cs 2^7, RTARG m 2^29, TPASS4
 *   DELLT4 cs 2^28, +MGA rev 2^0; HAPO, HPER (m 2^29) and VGDISP
 *   (m/cs 2^7) to agc_guid_state()->burn.
 *
 * ALGORITHM:
 *   As in the listing.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/rendezvous.h"

#define GRP4 4 /* restart group 4 (P30/P31 display) */

/* EP4(45)H 2DEC .125, EP4(10)H 2DEC .027777777: 45 and 10 degrees (rev). */
static const agc_dp_t EP4_45 = 33554432; /* 45 deg, rev 2^0 */
static const agc_dp_t EP4_10 = 7456540;  /* 10 deg, rev 2^0 */

/**********************************************************************
 * THISPREC / LEMPREC to tdec1 for the targeting subroutines (no job of
 * their own: an INTSTALL wait resumes agc_guid_nav_resume()); X1, X2 of
 * the integrator into RTX1, RTX2 (SXA,1 / SXA,2). NOT_FOUND when the
 * integration ended without RECTOUT. The result is in the shared
 * integration context (agc_integ_shared_ctx()->out).
 * AGC: the THISPREC/LEMPREC calls of S30.1 (CM p.639, LM p.616), S31.1
 * and AGAIN (CM p.711), with the SXA,1 / SXA,2 that follow them.
 *
 * FUNCTION NAME: prec_to
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   lm_only   bool            I    integrate the LM state only (Luminary
 *                                  P30)
 *   tdec1     agc_dp_t        I    TDEC1, integration end time, cs 2^28
 *   rz        agc_rendz_t *   I/O  rendezvous/targeting erasables
 *                                  (agc_rendz_t)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed), AGC_STATUS_NOT_FOUND (no RECTOUT); otherwise the status of the routine it
 *          continues in
 **********************************************************************/
static agc_status_t prec_to(agc_interp_t *ip, bool lm_only, agc_dp_t tdec1, agc_rendz_t *rz)
{
    agc_integ_ctx_t *ic = agc_integ_shared_ctx(); /* integration area */
    agc_status_t     st;                          /* integration status */

    if (lm_only)
    {
        st = agc_integ_lemprec(ip, ic, tdec1, agc_guid_nav_resume());
    }
    else
    {
        st = agc_integ_thisprec(ip, ic, tdec1, agc_guid_nav_resume());
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (!AGC_ASSERT(ic->out.x2 == 0 || ic->out.x2 == 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->body.rtx1 = (agc_sp_t) ic->out.x1;
    rz->body.rtx2 = (agc_sp_t) ic->out.x2;
    rz->body.moon = (ic->out.x2 != 0);

    return (ic->out.valid ? AGC_STATUS_OK : AGC_STATUS_NOT_FOUND); /* RECTOUT reached; no state */
}

/**********************************************************************
 * PERIAPO1 / SHIFTR1 / MAXCHK for HPER and HAPO (both S30.1, S31.1):
 * apogee and perigee altitudes of (r, v), limited to 9999.9 NM, m 2^29.
 * AGC: the PERIAPO1 / SHIFTR1 / MAXCHK calls of S30.1 (CM p.640, LM
 * p.617) and S31.1 (CM p.641).
 *
 * FUNCTION NAME: apo_peri
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   rz        agc_rendz_t *   I/O  rendezvous/targeting erasables
 *                                  (agc_rendz_t)
 *   r         agc_vec_t       I    position at TIG, m 2^29
 *   v         agc_vec_t       I    velocity after the burn, m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed); otherwise the status of the routine it
 *          continues in
 **********************************************************************/
static agc_status_t apo_peri(agc_interp_t *ip, agc_rendz_t *rz, agc_vec_t r, agc_vec_t v)
{
    agc_burn_t         *b  = &agc_guid_state()->burn;                         /* HAPO, HPER      */
    agc_rendz_periapo_t pa;                                                   /* PERIAPO1 output */
    agc_status_t        st = agc_rendz_periapo_body_scale(ip, rz, r, v, &pa); /* status of PERIAPO1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    b->hper = agc_orbit_params_maxchk(ip, agc_rendz_scalar_from_body(rz, pa.hperi)); /* HPER B+29 */
    b->hapo = agc_orbit_params_maxchk(ip, agc_rendz_scalar_from_body(rz, pa.hapo));  /* HAPO B+29 */

    if (!AGC_ASSERT(rz->body.rtx2 == 0 || rz->body.rtx2 == 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * S30.1 Comanche (p.639-640): state at TIG by THISPREC, the local-vertical
 * matrix from LOMAT (UNRM = UNIT(RTIG x VTIG), RACT3 = RTIG), DELVSIN =
 * DELVSLV VXM matrix VSL1, VGDISP = |DELVSIN|, apogee/perigee.
 *
 * FUNCTION NAME: external_delta_v_vectors_csm
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   rz        agc_rendz_t *   I/O  rendezvous/targeting erasables
 *                                  (agc_rendz_t)
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t external_delta_v_vectors_csm(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_status_t           st = prec_to(ip, false, agc_erase_tig(), rz); /* status */
    const agc_integ_out_t *p  = &agc_integ_shared_ctx()->out;            /* RATT VATT */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* THISPREC, TDEC1 = TIG */
    }

    rz->vtig  = p->vatt;
    rz->rtig  = p->ratt;
    rz->ract3 = p->ratt;
    rz->unrm  = agc_vec_unit(ip, agc_vec_cross(ip, rz->rtig, rz->vtig), NULL, NULL);
    agc_mat_t m; /* LOMAT rows, half unit (2^1) */

    st = agc_rendz_lomat(ip, rz, &m);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->delvsin                   = agc_vec_shl(ip, agc_vec_mat(ip, rz->delvlvc, &m), 1); /* DELVSLV = DELVLVC: one vector, rz->delvlvc */
    agc_guid_state()->burn.vgdisp = agc_vec_abval(ip, rz->delvsin);                       /* MAG DELV */

    return (apo_peri(ip, rz, rz->rtig, agc_vec_add(ip, rz->delvsin, rz->vtig)));
}

/**********************************************************************
 * S30.1 Luminary (p.616-617): LEMPREC; the LV axes at TIG built inline:
 * Z = -UNIT(RTIG), Y = UNIT(VTIG x RTIG), X = VSL1(Y x Z); DELVSIN =
 * DELVSLV VXM (X Y Z) VSL1, DELVSAB = |DELVSIN|, apogee/perigee.
 *
 * FUNCTION NAME: external_delta_v_vectors_lm
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   rz        agc_rendz_t *   I/O  rendezvous/targeting erasables
 *                                  (agc_rendz_t)
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t external_delta_v_vectors_lm(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_status_t           st = prec_to(ip, true, agc_erase_tig(), rz); /* status */
    const agc_integ_out_t *p  = &agc_integ_shared_ctx()->out;           /* RATT VATT */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* LEMPREC */
    }

    rz->rtig = p->ratt;
    agc_mat_t m; /* LV axes X Y Z at TIG, half unit (2^1) */

    m.row[2]    = agc_vec_neg(agc_vec_unit(ip, rz->rtig, NULL, NULL));  /* ZRF/LV */
    rz->vtig    = p->vatt;
    m.row[1]    = agc_vec_unit(ip, agc_vec_cross(ip, rz->vtig, rz->rtig), NULL, NULL);
    m.row[0]    = agc_vec_shl(ip, agc_vec_cross(ip, m.row[1], m.row[2]), 1);
    rz->delvsin = agc_vec_shl(ip, agc_vec_mat(ip, rz->delvlvc, &m), 1); /* DELVSLV = DELVLVC: one vector, rz->delvlvc */
    rz->delvsab = agc_vec_abval(ip, rz->delvsin);                       /* DELTA V MAG. */

    return (apo_peri(ip, rz, rz->rtig, agc_vec_add(ip, rz->vtig, rz->delvsin)));
}

/**********************************************************************
 * S30.1 (CM p.639, LM p.616): external delta-V targeting at TIG.
 *
 * FUNCTION NAME: agc_guid_external_delta_v_vectors
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   veh       agc_vehicle_t   I    vehicle: AGC_VEHICLE_CSM or
 *                                  AGC_VEHICLE_LM
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_guid_external_delta_v_vectors(agc_interp_t *ip, agc_vehicle_t veh)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rendz_t *rz = agc_rendz_erasable(); /* targeting erasables */

    if (!AGC_ASSERT(rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((veh == AGC_VEHICLE_CSM) ? external_delta_v_vectors_csm(ip, rz) : external_delta_v_vectors_lm(ip, rz)); /* Comanche S30.1; Luminary S30.1 */
}

/**********************************************************************
 * MIDGIM (Comanche P34-35 p.493): with AVFLAG equal to COMPUTER (both set
 * in the CMC for an LM-active burn) GET.LVC: DELVLVC = dv in the local
 * vertical axes of RINIT, VINIT; otherwise GET+MGA: +MGA.
 *
 * FUNCTION NAME: agc_guid_midgim
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   dv        agc_vec_t       I    delta-V in reference coordinates, m/cs
 *                                  2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_guid_midgim(agc_interp_t *ip, agc_vec_t dv)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rendz_t *rz  = agc_rendz_erasable();                     /* RINIT VINIT */
    bool         av  = agc_flag_test(AGC_FLAG_AVFLAG);           /* AVFLAG      */
    bool         cmc = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* COMPUTER flag    */

    if (av == cmc)
    {
        return (agc_rendz_get_lvc(ip, rz->rinit, rz->vinit, dv, &rz->delvlvc));
    }

    agc_mat_t refsmmat = agc_erase_refsmmat(); /* half-unit rows */

    return (agc_rendz_get_mga(ip, dv, &refsmmat, &rz->plus_mga));
}

/**********************************************************************
 * AGAIN (Comanche P40-P47 p.711) with TDEC1 as set by the caller (S31.1:
 * TIG; S40.1B: TIG - 2 s).
 *
 * FUNCTION NAME: agc_guid_again
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   tdec1     agc_dp_t        I    TDEC1, time to integrate to, cs 2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_guid_again(agc_interp_t *ip, agc_dp_t tdec1)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(tdec1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* AGAIN (Comanche P40-P47 p.711): THISPREC to TDEC1, RTIG = RINIT = RATT,
     * VTIG = VINIT = VATT, then INITVEL with NUMIT 0 and EPSILON4 45 degrees
     * (NORMSW: 165-195 degree transfer) or 10 degrees.                        */
    agc_rendz_t           *rz = agc_rendz_erasable();          /* targeting */
    agc_status_t           st = prec_to(ip, false, tdec1, rz); /* status    */
    const agc_integ_out_t *p  = &agc_integ_shared_ctx()->out;  /* RATT VATT */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* THISPREC */
    }

    rz->rtig    = p->ratt;
    rz->rinit   = p->ratt;
    rz->vtig    = p->vatt;
    rz->vinit   = p->vatt;
    agc_dp_t e4 = agc_flag_test(AGC_FLAG_NORMSW) ? EP4_45 : EP4_10; /* EPSILON4, rev 2^0; NORMSW: 45 */

    if (!AGC_ASSERT(e4 > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_rendz_initvel(ip, rz, 0, e4, agc_guid_nav_resume()));
}

/**********************************************************************
 * S31.1 (Comanche p.641-642): Lambert to RTARG from TIG; apogee/perigee of
 * (RTIG, VIPRIME); DELVLVC from DELVEET3 (SET AVFLAG; MIDGIM -> GET.LVC);
 * VGDISP = |DELVLVC|; TPASS4 = TIG + DELLT4.
 *
 * FUNCTION NAME: agc_guid_lambert_aimpoint_vectors
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed); otherwise the status of the routine it
 *          continues in
 **********************************************************************/
agc_status_t agc_guid_lambert_aimpoint_vectors(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_rendz_t *rz = agc_rendz_erasable();                /* targeting erasables */
    agc_status_t st = agc_guid_again(ip, agc_erase_tig()); /* TDEC1 = TIG */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = apo_peri(ip, rz, rz->rtig, rz->viprime);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_set(AGC_FLAG_AVFLAG);
    st = agc_guid_midgim(ip, rz->delveet3); /* DELVLVC B7 FOR DISPLAY */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_state()->burn.vgdisp = agc_vec_abval(ip, rz->delvlvc);
    rz->tpass4                    = agc_dp_add(ip, rz->dellt4, agc_erase_tig()); /* FOR S40.1     */

    return (AGC_STATUS_OK);
}

/* ---------------------------------------------------------------------- */
/* Program sequencing                                                      */
/* ---------------------------------------------------------------------- */

/**********************************************************************
 * A flashing display whose terminate goes to P00 (GOTOPOOH): the
 * GOFLASH / GOFLASHR calls of P30/P31 and PARAM30 (CM p.637, LM p.614).
 *
 * FUNCTION NAME: flash
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   ctx       const agc_job_ctx_t *  I    executive context of the running
 *                                         job
 *   kind      agc_dsky_disp_t        I    display kind (GOFLASH, GOFLASHR,
 *                                         ...)
 *   verb      uint8_t                I    verb, decimal as keyed
 *   noun      uint8_t                I    noun, decimal as keyed
 *   proceed   agc_job_id_t           I    job run on PROCEED
 *   enter     agc_job_id_t           I    job run on ENTER
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
static agc_status_t flash(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind, uint8_t verb, uint8_t noun, agc_job_id_t proceed, agc_job_id_t enter)
{
    agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH, proceed, enter}; /* TERM PRO ENTER */

    if (!AGC_ASSERT(proceed != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(enter != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, kind, agc_dsky_vn(verb, noun), &cont));
}

/**********************************************************************
 * P30 (CM p.637 / P30/P31 p.637; LM p.614): set UPDATFLG and TRACKFLG,
 * flash V06N33 (TIG). Comanche uses GOFLASHR and protects 4.1.
 *
 * FUNCTION NAME: ignition_time_entry
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *   self      agc_job_id_t     I    this job, re-run on ENTER (recycle)
 *   next      agc_job_id_t     I    job run on PROCEED
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
static agc_status_t ignition_time_entry(agc_job_ctx_t *ctx, agc_job_id_t self, agc_job_id_t next)
{
    agc_flag_set(AGC_FLAG_UPDATFLG);
    agc_flag_set(AGC_FLAG_TRACKFLG);

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return (flash(ctx, AGC_DSKY_GOFLASH, 6, 33, next, self)); /* VNPOOH      */
    }

    agc_status_t st = flash(ctx, AGC_DSKY_GOFLASHR, 6, 33, next, self); /* V06N33 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_phase_change(GRP4, (uint16_t) 000014);

    return (AGC_STATUS_OK); /* 4.1 ENDOFJOB  */
}

/**********************************************************************
 * P30 EXTERNAL DELTA V (CM P30-P37.agc p.636, LM P30_P37.agc p.614):
 * P30/P31's V06N33 TIG display, continuing at CNTNUP30.
 *
 * FUNCTION NAME: agc_job_external_delta_v
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_external_delta_v(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (ignition_time_entry(ctx, AGC_JOB_EXTERNAL_DELTA_V, AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_DV));
}

/**********************************************************************
 * CNTNUP30 (CM p.637) / P30N33+2 (LM): flash V06N81 (DELVSLV).
 *
 * FUNCTION NAME: agc_job_external_delta_v_display_dv
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_external_delta_v_display_dv(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (flash(ctx, AGC_DSKY_GOFLASH, 6, 81, AGC_JOB_EXTERNAL_DELTA_V_COMPUTE, AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_DV));
}

/**********************************************************************
 * P30 after N81 PROCEED (CM p.636, LM p.614): S30.1, then PARAM30.
 *
 * FUNCTION NAME: agc_job_external_delta_v_compute
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_external_delta_v_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_UPDATFLG);
    agc_interp_t ip; /* interpreter work area of the job */

    agc_interp_init(&ip);
    agc_guid_nav_set_resume(ctx->id);
    agc_status_t st = agc_guid_external_delta_v_vectors(&ip, agc_state_vehicle()); /* status of S30.1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_flag_set(AGC_FLAG_UPDATFLG);
    }

    return (agc_job_external_delta_v_display_parameters(ctx));
}

/**********************************************************************
 * PARAM30 (CM p.637, LM p.614): flash V06N42 (HAPO, HPER, VGDISP). CM:
 * PROCEED -> REFTEST; LM: PROCEED -> XDELVFLG set, REVN1645.
 *
 * FUNCTION NAME: agc_job_external_delta_v_display_parameters
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_external_delta_v_display_parameters(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_job_id_t pro = AGC_JOB_EXTERNAL_DELTA_V_REFTEST; /* PROCEED (CM) */

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        pro = AGC_JOB_EXTERNAL_DELTA_V_ACCEPT; /* XDELVFLG, REVN1645 */
    }

    return (flash(ctx, AGC_DSKY_GOFLASH, 6, 42, pro, AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_PARAMETERS));
}

/**********************************************************************
 * LM PARAM30 PROCEED (p.614): SETGO XDELVFLG REVN1645 (TRKMKCNT, TTOGO,
 * +MGA display of the rendezvous programs).
 *
 * FUNCTION NAME: agc_job_external_delta_v_accept
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_external_delta_v_accept(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_XDELVFLG); /* EXTERNAL DELTA-V      */

    return (agc_exec_job_yield(AGC_JOB_MARKS_TFI_MGA_MONITOR));
}

/**********************************************************************
 * P31 (CM p.637): as P30, but S31.1 before V06N81.
 *
 * FUNCTION NAME: agc_job_lambert_aimpoint_guidance_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_lambert_aimpoint_guidance_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (ignition_time_entry(ctx, AGC_JOB_LAMBERT_AIMPOINT_GUIDANCE, AGC_JOB_LAMBERT_AIMPOINT_COMPUTE));
}

/**********************************************************************
 * P31 after V06N33 PROCEED (CM p.637): UPDATFLG and NORMSW reset, S31.1,
 * then CNTNUP30 (V06N81) and PARAM30.
 *
 * FUNCTION NAME: agc_job_lambert_aimpoint_compute_csm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_lambert_aimpoint_compute_csm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_clear(AGC_FLAG_UPDATFLG);
    agc_flag_clear(AGC_FLAG_NORMSW);
    agc_interp_t ip; /* interpreter work area of the job */

    agc_interp_init(&ip);
    agc_guid_nav_set_resume(ctx->id);
    agc_status_t st = agc_guid_lambert_aimpoint_vectors(&ip); /* S31.1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_lambert_aimpoint_display_dv(ctx));
}

/**********************************************************************
 * CNTNUP30 for P31 (CM P30-P37.agc p.637): V06N81 (DELVLVC), PROCEED
 * -> PARAM30.
 *
 * FUNCTION NAME: agc_job_lambert_aimpoint_display_dv
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   ctx       agc_job_ctx_t *  I    executive context of the running job
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_job_lambert_aimpoint_display_dv(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (flash(ctx, AGC_DSKY_GOFLASH, 6, 81, AGC_JOB_EXTERNAL_DELTA_V_DISPLAY_PARAMETERS, AGC_JOB_LAMBERT_AIMPOINT_DISPLAY_DV));
}
