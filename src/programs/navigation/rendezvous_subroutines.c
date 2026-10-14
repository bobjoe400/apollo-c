/**********************************************************************
 * FILE NAME: rendezvous_subroutines.c
 *
 * PURPOSE:
 *   The shared rendezvous subroutines of P34-35_P74-75.agc: SELECTMU,
 *   VECSHIFT, SHIFTR1, PERIAPO/PERIAPO1/SETRAD, PRECSET/LEMSTORE/
 *   CSMSTORE/ACTIVE/PASSIVE, INTINT, S34/35.1, LOMAT, MIDGIM (GET.LVC,
 *   GET+MGA), AVFLAGA/P, P20FLGON; with the interpretive helpers (UNIT,
 *   ABVAL, NORM, SL*, SR*) every rendezvous file uses.
 *
 * AGC SOURCE:
 *   Comanche055 P34-35_P74-75.agc pp. 469-500 (SELECTMU p.494, PERIAPO
 *   p.496, SETRAD p.497, PRECSET p.498, VECSHIFT p.499, SHIFTR1 p.500,
 *   INTINT LOMAT p.479, MIDGIM p.493, S34/35.1 p.474), P30-P37.agc p.648
 *   (AVFLAGA, AVFLAGP, P20FLGON)
 *   Luminary099 P34-35_P74-75.agc pp. 672-699 (same subroutines),
 *   P32-P35_P72-P75.agc p.629
 *
 * EXTERNAL VARIABLES:
 *   Name       Type        I/O  Scale      Description
 *   ---------  ----------  ---  ---------  ----------------------------
 *   (rendezvous erasables) agc_rendz_t I/O  see rendezvous.h
 *   AVFLAG MOONFLAG INTYPFLG TRACKFLG UPDATFLG MGLVFLAG RVSW  flags  O
 *   ECSTEER    agc_dp_t    O    -          CM AVFLAGA: BIT13
 *   RLS        agc_vec_t   I    m 2^27     SETRAD about the moon
 *
 * EXTERNAL REFERENCES:
 *   Name                     Description
 *   -----------------------  -------------------------------------------
 *   agc_conic_mu_ ..         MUTABLE (MU, 1/SQRT(MU))
 *   agc_conic_timethet       TIMETHET
 *   agc_conic_apsides        APSIDES
 *   rz_integ                 CSMPREC LEMPREC CSMCONIC LEMCONIC (state_vector_update.c)
 *   rz_ttogo_display         VN1645 (programs/displays)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. Functions return AGC_STATUS_BAD_ARG for out-of-range
 *   arguments; integration statuses (INTSTALL sleep) are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called from rendezvous and navigation jobs.
 *
 * NOTES:
 *   Scales (from the AGC text): r m 2^29, v m/cs 2^7, t cs 2^28, angles
 *   revolutions 2^0, unit vectors 2^1, mu 2^36. "Body scale" is 2^27 /
 *   2^5 when RTX2 = 2 (moon). NORM leaves -(left shifts) in the index
 *   register, so a normalized value of scale 2^S is at 2^(S + X).
 **********************************************************************/
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "agc/nav/conic.h"
#include "internal.h"

static void agc_rendz_periapo(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t rvec, agc_vec_t vvec, agc_rendz_periapo_t *out);

/* ---- numeric helpers ---------------------------------------------------- */
/**********************************************************************
 * UNIT: half-unit vector (scale 2^1)
 * AGC: UNIT (INTERPRETER.agc p.1175)
 *
 * FUNCTION NAME: rz_unit
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   v         agc_vec_t       I    vector at any scale
 *
 * RETURNS: agc_vec_t -- the half-unit vector, 2^1 (zero for a bad argument)
 **********************************************************************/
agc_vec_t rz_unit(agc_interp_t *ip, agc_vec_t v)
{
    return (agc_vec_unit(ip, v, NULL, NULL));
}

/**********************************************************************
 * ABVAL
 * AGC: ABVAL (INTERPRETER.agc p.1180)
 *
 * FUNCTION NAME: rz_abval
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   v         agc_vec_t       I    vector
 *
 * RETURNS: agc_dp_t -- |V| at the scale of V
 **********************************************************************/
agc_dp_t rz_abval(agc_interp_t *ip, agc_vec_t v)
{
    agc_dp_t a = agc_vec_abval(ip, v); /* |V|, V's scale */

    return (a);
}

/**********************************************************************
 * NORM when only the index register is wanted: -(left shifts), 0 for 0.
 * AGC: NORM (INTERPRETER.agc), index register only
 *
 * FUNCTION NAME: rz_norm_xreg
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   a         agc_dp_t  I    the value
 *
 * RETURNS: int -- -(left shifts), 0 for 0
 **********************************************************************/
int rz_norm_xreg(agc_dp_t a)
{
    if (a == 0)
    {
        return (0);
    }

    const int n = agc_dp_norm_count(a); /* left shifts */

    return (-n);
}

/**********************************************************************
 * NORM X: left-justify A; *xreg = the AGC index register, -(left shifts).
 * AGC: NORM (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_norm
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   a         agc_dp_t  I    the value, scale 2^S
 *   xreg      int *     O    -(left shifts)
 *
 * RETURNS: agc_dp_t -- the normalized value, scale 2^(S + X)
 **********************************************************************/
agc_dp_t rz_norm(agc_dp_t a, int *xreg)
{
    if (a == 0)
    {
        *xreg = 0;

        return (0);
    }

    int      n = 0;                  /* left shifts */
    agc_dp_t r = agc_dp_norm(a, &n); /* normalized, 2^(S-N) */

    *xreg = -n;

    return (r);
}

/**********************************************************************
 * SL k (k > 0) or SR -k (k < 0): the AGC SL* and SR* with an index.
 * AGC: SL, SR (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_shift
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   a         agc_dp_t        I    the value
 *   k         int             I    left shift (k > 0) or right shift (k <
 *                                  0)
 *
 * RETURNS: agc_dp_t -- the shifted value (OVFIND set on overflow)
 **********************************************************************/
agc_dp_t rz_shift(agc_interp_t *ip, agc_dp_t a, int k)
{
    if (k > 0)
    {
        return (agc_dp_shl(ip, a, k));
    }

    if (k < 0)
    {
        return (agc_dp_shr(a, -k));
    }

    return (a);
}

/**********************************************************************
 * SL* addr,X: shift count = addr - C(X); a negative count shifts right.
 * AGC: SL* (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_sl_star
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   a         agc_dp_t        I    the value
 *   addr      int             I    the address field
 *   xreg      int             I    the index register
 *
 * RETURNS: agc_dp_t -- A shifted left by addr - C(X)
 **********************************************************************/
agc_dp_t rz_sl_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg)
{
    return (rz_shift(ip, a, addr - xreg));
}

/**********************************************************************
 * SR* addr,X: shift right by addr - C(X); a negative count shifts left.
 * AGC: SR* (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_sr_star
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   a         agc_dp_t        I    the value
 *   addr      int             I    the address field
 *   xreg      int             I    the index register
 *
 * RETURNS: agc_dp_t -- A shifted right by addr - C(X)
 **********************************************************************/
agc_dp_t rz_sr_star(agc_interp_t *ip, agc_dp_t a, int addr, int xreg)
{
    return (rz_shift(ip, a, xreg - addr));
}

/**********************************************************************
 * VSL k (k > 0) or VSR -k (k < 0).
 * AGC: VSL, VSR (INTERPRETER.agc)
 *
 * FUNCTION NAME: rz_vshift
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   v         agc_vec_t       I    the vector
 *   k         int             I    left shift (k > 0) or right shift (k <
 *                                  0)
 *
 * RETURNS: agc_vec_t -- the shifted vector
 **********************************************************************/
agc_vec_t rz_vshift(agc_interp_t *ip, agc_vec_t v, int k)
{
    if (k > 0)
    {
        return (agc_vec_shl(ip, v, k));
    }

    if (k < 0)
    {
        return (agc_vec_shr(v, -k));
    }

    return (v);
}

/**********************************************************************
 * VECSHIFT / VSR* 0,2 (P34-35 p.499): earth scale -> body scale. With
 * X2 = RTX2 = 2 (moon) the count 0 - X2 is negative: a left shift by 2.
 * AGC: VECSHIFT (P34-35_P74-75.agc p.499)
 *
 * FUNCTION NAME: agc_rendz_vec_to_body
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   v         agc_vec_t            I    vector at the earth scale
 *
 * RETURNS: agc_vec_t -- V at the body scale
 **********************************************************************/
agc_vec_t agc_rendz_vec_to_body(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t v)
{
    return (rz_vshift(ip, v, rz->body.rtx2));
}

/**********************************************************************
 * VSL* 0,2: body scale -> earth scale (moon: shift right 2).
 * AGC: "VSL* 0,2" (P34-35_P74-75.agc)
 *
 * FUNCTION NAME: rz_vec_from_body
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   v         agc_vec_t            I    vector at the body scale
 *
 * RETURNS: agc_vec_t -- V at the earth scale
 **********************************************************************/
agc_vec_t rz_vec_from_body(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t v)
{
    return (rz_vshift(ip, v, -(int) rz->body.rtx2));
}

/**********************************************************************
 * SHIFTR1 (P34-35 p.500): SL* 0,2 -- a body-scale scalar to 2^29.
 * AGC: SHIFTR1 (P34-35_P74-75.agc p.500)
 *
 * FUNCTION NAME: agc_rendz_scalar_from_body
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   a         agc_dp_t             I    scalar at the body scale
 *
 * RETURNS: agc_dp_t -- A at the earth scale (m 2^29)
 **********************************************************************/
agc_dp_t agc_rendz_scalar_from_body(const agc_rendz_t *rz, agc_dp_t a)
{
    return (agc_dp_shr(a, rz->body.rtx2));
}

/**********************************************************************
 * SELECTMU (P34-35 p.494)
 * Choose mu and the scale indices for the primary body of the CSM state
 * (CMOONFLG). RTMU is always at 2^36 (the lunar MUM, 2^30, is shifted
 * right 6). The AGC then clears FINALFLG and goes to VN1645; the program
 * drivers do the display (rz_ttogo_display) themselves.
 * AGC: SELECTMU (P34-35_P74-75.agc p.494)
 *
 * FUNCTION NAME: agc_rendz_selectmu
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   rz        agc_rendz_t *  I/O  rendezvous erasables
 *   cmoonflg  bool           I    CMOONFLG: the CSM state is moon-centred
 *
 * The mu words are rope constants (MUE, MUM), so SELECTMU cannot fail.
 **********************************************************************/
void agc_rendz_selectmu(agc_rendz_t *rz, bool cmoonflg)
{
    rz->body.moon = cmoonflg;

    if (cmoonflg)
    {
        rz->body.rtx1    = -10;
        rz->body.rtx2    = 2;
        rz->body.rtsr1mu = agc_dp_from_words(agc_conic_inv_root_mu_(AGC_CONIC_MOON));       /* MUTABLE +14, 2^14 */
        rz->body.rtmu    = agc_dp_shr(agc_dp_from_words(agc_conic_mu_(AGC_CONIC_MOON)), 6); /* MUTABLE +8: 2^30 -> 2^36 */
    }
    else
    {
        rz->body.rtx1    = -2;
        rz->body.rtx2    = 0;
        rz->body.rtsr1mu = agc_dp_from_words(agc_conic_inv_root_mu_(AGC_CONIC_EARTH)); /* MUTABLE +6, 2^17 */
        rz->body.rtmu    = agc_dp_from_words(agc_conic_mu_(AGC_CONIC_EARTH));          /* MUTABLE, 2^36 */
    }
}

/**********************************************************************
 * SELECTMU through its end (P34-35_P74-75.agc p.494): the primary body
 * (agc_rendz_selectmu on CMOONFLG), CLEAR FINALFLG, GOTO VN1645, whose
 * display returns into the program at next.
 * AGC: SELECTMU, RTRNMU (P34-35_P74-75.agc p.494), VN1645
 *
 * FUNCTION NAME: rz_selectmu_display
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *          I/O  rendezvous erasables
 *   next      agc_job_id_t           I    job started next
 *
 * RETURNS: agc_status_t -- the status of the VN1645 display
 **********************************************************************/
agc_status_t rz_selectmu_display(const agc_job_ctx_t *ctx, agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t next)
{
    agc_rendz_selectmu(rz, agc_flag_test(AGC_FLAG_CMOONFLG));
    rz_flag_put(AGC_FLAG_FINALFLG, false); /* CLEAR FINALFLG */

    return (rz_ttogo_display(ctx, ip, rz, next)); /* GOTO VN1645 */
}

/**********************************************************************
 * AVFLAGA / AVFLAGP / P20FLGON (CM P30-P37 p.~, LM P32-P35)
 * AVFLAG reset = CSM active, set = LM active. "This vehicle active" is
 * AVFLAGA. The CM AVFLAGA also sets ECSTEER = 1 (BIT13).
 * AGC: AVFLAGA, AVFLAGP (P30-P37.agc p.648)
 *
 * FUNCTION NAME: rz_avflag
 *
 * ARGUMENTS:
 *   Argument     Type  I/O  Description (scale)
 *   -----------  ----  ---  ---------------------------------------
 *   this_active  bool  I    true: this vehicle is the active one
 **********************************************************************/
void rz_avflag(bool this_active)
{
    bool csm       = rz_is_csm();                      /* CM build */
    bool lm_active = csm ? !this_active : this_active; /* AVFLAG */

    rz_flag_put(AGC_FLAG_AVFLAG, lm_active);

    if (csm && this_active)
    {
        /* CAF BIT13, TS ECSTEER: "SET ECSTEER = 1" (high word only) */
        agc_burn_t *b = &agc_guid_state()->burn; /* burn erasables */

        b->ecsteer = agc_dp_pack((agc_sp_t) 010000, agc_dp_low(b->ecsteer));
    }
}

/**********************************************************************
 * P20FLGON: SET TRACKFLG, SET UPDATFLG.
 * AGC: P20FLGON (P30-P37.agc p.648)
 *
 * FUNCTION NAME: rz_tracking_flags_on
 *
 * ARGUMENTS: none
 **********************************************************************/
void rz_tracking_flags_on(void)
{
    agc_flag_set(AGC_FLAG_TRACKFLG);
    agc_flag_set(AGC_FLAG_UPDATFLG);
}

/**********************************************************************
 * PRECSET / LEMSTORE / CSMSTORE / ACTIVE / PASSIVE (p.498)
 * AGC: ACTIVE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: rz_store_active
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   rz        agc_rendz_t *  I/O  rendezvous erasables
 *   r         agc_vec_t      I    position, m 2^29
 *   v         agc_vec_t      I    velocity, m/cs 2^7
 **********************************************************************/
void rz_store_active(agc_rendz_t *rz, agc_vec_t r, agc_vec_t v)
{
    rz->ract3 = r; /* ACTIVE: STOVL RACT3        */
    rz->vact3 = v; /*         STORE VACT3        */
}

/**********************************************************************
 * PASSIVE: RPASS3 = R, VPASS3 = V.
 * AGC: PASSIVE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: rz_store_passive
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   rz        agc_rendz_t *  I/O  rendezvous erasables
 *   r         agc_vec_t      I    position, m 2^29
 *   v         agc_vec_t      I    velocity, m/cs 2^7
 **********************************************************************/
void rz_store_passive(agc_rendz_t *rz, agc_vec_t r, agc_vec_t v)
{
    rz->rpass3 = r; /* PASSIVE: STOVL RPASS3      */
    rz->vpass3 = v; /*          STORE VPASS3      */
}

/**********************************************************************
 * LEMSTORE / CSMSTORE: an integration result as the active state when
 * AVFLAG names this vehicle (set: the LM is active), else as the passive.
 * AGC: LEMSTORE, CSMSTORE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: rz_vehicle_store
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------
 *   rz        agc_rendz_t *            I/O  rendezvous erasables
 *   lm        bool                     I    true: the LM state (LEMSTORE)
 *   o         const agc_integ_out_t *  I    the integration result (RATT,
 *                                           VATT at earth scale)
 **********************************************************************/
void rz_vehicle_store(agc_rendz_t *rz, bool lm, const agc_integ_out_t *o)
{
    bool lm_active = agc_flag_test(AGC_FLAG_AVFLAG); /* AVFLAG */

    if (lm == lm_active)
    {
        rz_store_active(rz, o->ratt, o->vatt);
    }
    else
    {
        rz_store_passive(rz, o->ratt, o->vatt);
    }
}

/**********************************************************************
 * PRECSET: TDEC2 = TDEC1; LEMPREC, LEMSTORE (AVFLAG set = LM active);
 * TDEC1 = TDEC2; CSMPREC, CSMSTORE. RATT/VATT are at earth scale.
 * AGC: PRECSET, LEMSTORE, CSMSTORE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: rz_precset
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   tdec1     agc_dp_t        I    TDEC1, cs 2^28
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status (INTSTALL sleep)
 **********************************************************************/
agc_status_t rz_precset(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t tdec1, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(tdec1 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_out_t o; /* integration result */

    rz->tdec2       = tdec1;                                       /* TDEC2   */
    agc_status_t st = rz_integ(ip, RZ_LEMPREC, tdec1, resume, &o); /* LEMPREC */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_vehicle_store(rz, true, &o); /* LEMSTORE */
    st = rz_integ(ip, RZ_CSMPREC, rz->tdec2, resume, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_vehicle_store(rz, false, &o); /* CSMSTORE */

    return (AGC_STATUS_OK);
} /* rz_precset */

/**********************************************************************
 * INTINT (p.479)
 * Push list on entry: V, R, T1, T2, conic indicator (non-zero = conic).
 * INTSTALL; INTYPFLG per indicator; TDEC1 = T2; MOONFLAG = CMOONFLG;
 * TET = T1; RCV, VCV = VSR* 0,2 (body scale); INTEGRVS; RATT, VATT out.
 * GOINT (p.479) is INTINT with T2 = TTPI + NOMTPI... called by S33/34.1.
 * AGC: INTINT (P34-35_P74-75.agc p.479)
 *
 * FUNCTION NAME: rz_intint
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   r         agc_vec_t            I    position at T1, m 2^29
 *   v         agc_vec_t            I    velocity at T1, m/cs 2^7
 *   t1        agc_dp_t             I    time of the state, cs 2^28
 *   t2        agc_dp_t             I    time to integrate to, cs 2^28
 *   conic     bool                 I    true: conic, false: precision
 *   resume    agc_job_id_t         I    job that continues the caller
 *   ratt      agc_vec_t *          O    position at T2, m 2^29
 *   vatt      agc_vec_t *          O    velocity at T2, m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status
 **********************************************************************/
agc_status_t rz_intint(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r, agc_vec_t v, agc_dp_t t1, agc_dp_t t2, bool conic, agc_job_id_t resume, agc_vec_t *ratt, agc_vec_t *vatt)
{
    rz_integrvs_in_t in; /* INTEGRVS inputs */

    in.conic           = conic;
    in.tdec1           = t2;
    in.moon            = agc_flag_test(AGC_FLAG_CMOONFLG);
    in.tet             = t1;
    in.rcv             = agc_rendz_vec_to_body(ip, rz, r);
    in.vcv             = agc_rendz_vec_to_body(ip, rz, v);
    agc_integ_out_t o;                                     /* integration result */
    agc_status_t    st = rz_integrvs(ip, &in, resume, &o); /* INTEGRVS */

    if (st == AGC_STATUS_OK)
    {
        *ratt = o.ratt;
        *vatt = o.vatt;
    }

    return (st);
}

/**********************************************************************
 * TIMETHET call set-up used throughout P32-P39 (VECSHIFT, X1 = RTX1).
 * AGC: VECSHIFT, TIMETHET (CONIC_SUBROUTINES.agc p.1284)
 *
 * FUNCTION NAME: rz_timethet_rv
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ------------------------
 *   ip        agc_interp_t *          I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *     I    rendezvous erasables
 *   r         agc_vec_t               I    position, m 2^29
 *   v         agc_vec_t               I    velocity, m/cs 2^7
 *   snth      agc_dp_t                I    SNTH, 2^1
 *   csth      agc_dp_t                I    CSTH, 2^1
 *   rvsw      bool                    I    RVSW: time only
 *   out       agc_conic_time_out_t *  O    TIMETHET result
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the TIMETHET status
 **********************************************************************/
agc_status_t rz_timethet_rv(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r, agc_vec_t v, agc_dp_t snth, agc_dp_t csth, bool rvsw, agc_conic_time_out_t *out)
{
    agc_timetheta_in_t in; /* TIMETHET inputs */

    in.rvec = agc_rendz_vec_to_body(ip, rz, r);
    in.vvec = agc_rendz_vec_to_body(ip, rz, v);
    in.snth = snth;
    in.csth = csth;
    in.rvsw = rvsw;
    in.body = agc_conic_body_of_x1(rz->body.rtx1);
    rz_flag_put(AGC_FLAG_RVSW, rvsw);

    return (agc_conic_timethet(ip, &in, out));
}

/**********************************************************************
 * PERIAPO1 / PERIAPO / SETRAD (p.495-497)
 * PERIAPO1 shifts the earth-scale state in the push list to body scale;
 * SETRAD gives XXXALT = RPAD (2^29) about the earth, |RLS| (2^27) about
 * the moon; APSIDES gives the apsides. Altitudes = radii - XXXALT, all at
 * body scale (callers apply SHIFTR1).
 * AGC: PERIAPO1 (P34-35_P74-75.agc p.496)
 *
 * FUNCTION NAME: agc_rendz_periapo_body_scale
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *    I    rendezvous erasables
 *   r         agc_vec_t              I    position, m 2^29
 *   v         agc_vec_t              I    velocity, m/cs 2^7
 *   out       agc_rendz_periapo_t *  O    apsides and altitudes, body scale
 *
 * RETURNS: none
 **********************************************************************/
void agc_rendz_periapo_body_scale(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t r, agc_vec_t v, agc_rendz_periapo_t *out)
{
    agc_vec_t vvec = agc_rendz_vec_to_body(ip, rz, v); /* body scale */
    agc_vec_t rvec = agc_rendz_vec_to_body(ip, rz, r); /* body scale */

    agc_rendz_periapo(ip, rz, rvec, vvec, out);
}

/**********************************************************************
 * PERIAPO (p.496): RVEC, VVEC already at body scale.
 * AGC: PERIAPO, SETRAD (P34-35_P74-75.agc pp.496-497)
 *
 * FUNCTION NAME: agc_rendz_periapo
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *    I    rendezvous erasables
 *   rvec      agc_vec_t              I    position, body scale
 *   vvec      agc_vec_t              I    velocity, body scale
 *   out       agc_rendz_periapo_t *  O    apsides and altitudes, body scale
 *
 * RETURNS: none
 **********************************************************************/
static void agc_rendz_periapo(agc_interp_t *ip, const agc_rendz_t *rz, agc_vec_t rvec, agc_vec_t vvec, agc_rendz_periapo_t *out)
{
    agc_conic_body_t body = agc_conic_body_of_x1(rz->body.rtx1); /* X1 */

    /* SETRAD: RPAD (2^29) about the earth, |RLS| (2^27) about the moon */
    out->xxxalt = RZ_RPAD_B29; /* SETRAD: earth RPAD */

    if (body == AGC_CONIC_MOON)
    {
        out->xxxalt = rz_abval(ip, agc_erase_rls()); /* moon |RLS| */
    }

    agc_apsides_out_t aps; /* APSIDES results */

    agc_conic_apsides(ip, rvec, vvec, body, &aps);

    out->rapo  = aps.rapo;                                /* 2D */
    out->rperi = aps.rperi;                               /* 6D */
    out->ecc   = aps.ecc;
    out->r1    = aps.r1;
    out->p     = aps.p;
    out->hapo  = agc_dp_sub(ip, out->rapo, out->xxxalt);  /* 4D */
    out->hperi = agc_dp_sub(ip, out->rperi, out->xxxalt); /* 8D */
}

/**********************************************************************
 * S34/35.1 (p.474): unit normal and line of sight at T3
 * AGC: S34/35.1 (P34-35_P74-75.agc p.474)
 *
 * FUNCTION NAME: agc_rendz_los_and_normal
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
agc_status_t agc_rendz_los_and_normal(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->ulos = rz_unit(ip, agc_vec_sub(ip, rz->rpass3, rz->ract3));
    rz->unrm = rz_unit(ip, agc_vec_cross(ip, rz->ract3, rz->vact3));

    if (!AGC_ASSERT(rz->unrm.c[0] != AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * LOMAT (p.479): LV matrix rows X = Z x (-Y)... from UNRM, RACT3
 * Y = -UNRM, Z = -UNIT(RACT3), X = (Z x UNRM) VSL1. Rows at 2^1.
 * AGC: LOMAT (P34-35_P74-75.agc p.479)
 *
 * FUNCTION NAME: agc_rendz_lomat
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *   m         agc_mat_t *          O    LV matrix rows, 2^1
 *
 * RETURNS: none
 **********************************************************************/
void agc_rendz_lomat(agc_interp_t *ip, const agc_rendz_t *rz, agc_mat_t *m)
{
    m->row[1] = agc_vec_neg(rz->unrm);
    m->row[2] = agc_vec_neg(rz_unit(ip, rz->ract3));
    m->row[0] = agc_vec_shl(ip, agc_vec_cross(ip, m->row[2], rz->unrm), 1);
}

/**********************************************************************
 * GET.LVC (MIDGIM p.493): delta V to local vertical coordinates
 * Z = U(-R), Y = U(U(-R) x V), X = U(Y x Z); DELVLVC = M dV, VSL1.
 * AGC: GET.LVC (P34-35_P74-75.agc p.493)
 *
 * FUNCTION NAME: agc_rendz_get_lvc
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rinit     agc_vec_t       I    position, m 2^29
 *   vinit     agc_vec_t       I    velocity, m/cs 2^7
 *   dv        agc_vec_t       I    delta V, reference axes, m/cs 2^7
 *   delvlvc   agc_vec_t *     O    delta V, local vertical axes, m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a bad
 *          argument
 **********************************************************************/
agc_status_t agc_rendz_get_lvc(agc_interp_t *ip, agc_vec_t rinit, agc_vec_t vinit, agc_vec_t dv, agc_vec_t *delvlvc)
{
    agc_mat_t m; /* matrix rows, unit 2^1 */

    m.row[2] = agc_vec_neg(rz_unit(ip, rinit));                    /* 18D */
    m.row[1] = rz_unit(ip, agc_vec_cross(ip, m.row[2], vinit));    /* 12D */
    m.row[0] = rz_unit(ip, agc_vec_cross(ip, m.row[1], m.row[2])); /*  6D */
    *delvlvc = agc_vec_shl(ip, agc_mat_vec(ip, &m, dv), 1);

    if (!AGC_ASSERT(delvlvc->c[0] != AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_MGLVFLAG);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GET+MGA (MIDGIM p.493): +MGA = ARCSIN(UNIT(dV) . Ysm) in 0..1 rev
 * AGC: GET+MGA (P34-35_P74-75.agc C p.493, L line 1406)
 * Luminary's GET+MGA has VLOAD UNIT / UNIT: the half-unit vector is
 * unitized a second time (which can move its last bits); Comanche
 * unitizes once. The C follows each rope.
 *
 * FUNCTION NAME: agc_rendz_get_mga
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   dv        agc_vec_t          I    delta V, reference axes, m/cs 2^7
 *   refsmmat  const agc_mat_t *  I    REFSMMAT, 2^1
 *   mga       agc_dp_t *         O    +MGA, revolutions 0 .. 1
 *
 * RETURNS: none (MGLVFLAG cleared to mark a +MGA result)
 **********************************************************************/
void agc_rendz_get_mga(agc_interp_t *ip, agc_vec_t dv, const agc_mat_t *refsmmat, agc_dp_t *mga)
{
    /* HALFREV 2DEC 1 B-1 */
    static const agc_dp_t halfrev = AGC_DP_HALF; /* HALFREV, rev 2^0 */
    agc_dp_t              s;                     /* sin, 2^1 */

    agc_vec_t u = rz_unit(ip, dv); /* VLOAD UNIT: UV, 2^1 */

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        u = rz_unit(ip, u); /* Luminary: second UNIT */
    }

    s          = agc_dp_shl(ip, agc_vec_dot(ip, u, refsmmat->row[1]), 1);
    agc_dp_t a = agc_dp_asin(ip, s); /* MGA, rev 2^0 */

    if (a < 0)
    { /* convert -MGA to +MGA: add one rev */
        a = agc_dp_add(ip, agc_dp_add(ip, a, halfrev), halfrev);
    }

    *mga = a;
    agc_flag_clear(AGC_FLAG_MGLVFLAG);
}
