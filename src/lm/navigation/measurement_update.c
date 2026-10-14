/**********************************************************************
 * FILE NAME: measurement_update.c
 *
 * PURPOSE:
 *   LSR22.3 / LSR22.4, LM only: the rendezvous radar measurement
 *   incorporation of R22LEM, with GETULC, RADARANG, LGCUPDTE, WLINIT and
 *   INTGRCAL. "This routine computes the B-vectors and delta Q for each
 *   of the quantities measured by the rendezvous radar (range, range
 *   rate, shaft and trunnion angles). The routine calls the INCORP1 and
 *   INCORP2 routines which compute the deviations and correct the state
 *   vector. If the computed state vector deviations exceed the maximum
 *   permitted, the routine returns to R22LEM to display the deviations."
 *   LSR22.4 is the lunar surface entry: range and range rate only, 6x6
 *   W, CSM updated. Both state vectors are first integrated to the mark
 *   time (the updated vehicle with its W matrix); the four measurements
 *   then run in order, each one INCORP1, the N49 check (R22LEM96, an
 *   asynchronous display), INCORP2.
 *
 * AGC SOURCE:
 *   Luminary099 P20-P25.agc pp. 574-587 (LSR22.3, LSR22.4, RANGEBQ,
 *   RATEBQ, SHAFTBQ, TRUNBQ, MXMYMZ, GETULC, RADARANG, LGCUPDTE, ASTOK,
 *   WLINIT, INTGRCAL, INITWMX6, MARKTEST); LM only
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   RM RDOTM RRSHAFT RRTRUN MKTIME AIG AMG AOG (lmnav_state_t) I  mark
 *   R22DISP WHCHREAD (lmnav_state_t) O     m 2^29, m/cs 2^7  noun 49
 *   W           agc_incorp_w_t I/O  as integrated  error transition
 *   X789        agc_vec_t  I    B-5/B-3    shaft / trunnion biases
 *   REFSMMAT    agc_mat_t  I    2^1        reference to SM
 *   RANGEVAR RATEVAR RVARMIN VVARMIN SHAFTVAR TRUNVAR RMAX VMAX WRENDPOS
 *   WRENDVEL WSURFPOS WSURFVEL WSHAFT WTRUN  pad loads  I
 *   VEHUPFLG RENDWFLG SURFFLAG LMOONFLG DMENFLG STATEFLG  flags  I/O
 *   TRKMKCNT    agc_sp_t   O    -          zeroed by WLINIT
 *
 * EXTERNAL REFERENCES:
 *   Name                        Description
 *   --------------------------  ----------------------------------------
 *   agc_integ_integrv           INTEGRV (INTGRCAL)
 *   agc_incorp_state_deviation  INCORP1
 *   rz_incorp2                  INCORP2
 *   agc_rtb_normunit            NORMUNIT
 *   agc_latlong_arctan          ARCTAN
 *   lmnav_residuals_display     R22LEM96 noun 49
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. Too large a state change stops for the noun 49 display.
 *   AGC_STATUS_NO_CONVERGE when the loop bound MU_STEPS is exhausted (cannot
 *   happen: range, range rate, shaft, trunnion, then the RENDEND pass). Integration statuses (INTSTALL sleep) are passed
 *   back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of R22 (LM).
 *
 * NOTES:
 *   Scales ("B-n" in the AGC comments = scaled 2^n): ULC half unit; RLC
 *   body scale (earth 2^29, moon 2^27; X2 = SCALSHFT = 2 earth, 0 moon);
 *   DELTAQ, BVECTOR and VARIANCE (TP, B-40) as noted per measurement.
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "../radar/radar.h"
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/guidance/guidance.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"

#define MU_IMUVAR ((agc_dp_t) 1099512)   /* IMUVAR 2DEC E-6 B12: rad^2  */
#define MU_2PI_8  ((agc_dp_t) 210828714) /* 2PI/8 2DEC 3.141592653 B-2  */
#define MU_STEPS  5                      /* 4 measurements + the RENDEND pass */

typedef enum mu_step
{
    MU_RANGE    = 1,
    MU_RATE     = 2,
    MU_SHAFT    = 3,
    MU_TRUNNION = 4,
    MU_END      = 5
} mu_step_t;

/* LSR22.3 erasables (ULC, RXZ, SINTHETA, MX MY MZ, SCALSHFT) and the
 * INCORP1 result kept across the noun 49 display.                     */
typedef struct mu_state
{
    mu_step_t    step;     /* the measurement in work (WHCHREAD)        */
    int          x2;       /* SCALSHFT: 2 earth, 0 moon                 */
    agc_vec_t    ulc;      /* ULC: unit LM-to-CSM, half unit 2^1        */
    agc_dp_t     rlc;      /* 36D: |CSM - LM|, body scale               */
    agc_vec_t    mx;       /* MX: NB X axis in reference axes, 2^1      */
    agc_vec_t    my;       /* MY: NB Y axis, 2^1                        */
    agc_vec_t    mz;       /* MZ: NB Z axis, 2^1                        */
    agc_dp_t     rxz;      /* RXZ: RLC cos theta, normalized, 2^(S+X1)  */
    int          rxz_x1;   /* X1 after NORM of RXZ (-shifts)            */
    agc_dp_t     sintheta; /* SINTHETA: -ULC . MY, 2^1                  */
    agc_incorp_t inc;      /* BVECTOR DELTAQ VARIANCE .. DELTAX         */
} mu_state_t;

static mu_state_t mu; /* LSR22.3 erasables */

/**********************************************************************
 * shifts with an index register (count = addr - C(X))
 * AGC: SRR* (INTERPRETER.agc)
 *
 * FUNCTION NAME: mu_srr_star
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   a         agc_dp_t        I    the value
 *   count     int             I    right shift (left when negative)
 *
 * RETURNS: agc_dp_t -- A shifted with rounding
 **********************************************************************/
static agc_dp_t mu_srr_star(agc_interp_t *ip, agc_dp_t a, int count)
{
    if (count < 0)
    {
        return (agc_dp_shl_round(ip, a, -count)); /* negative: left */
    }

    return (agc_dp_shr_round(a, count));
}

/**********************************************************************
 * SLR* with an index: left by count (right when negative), rounded.
 * AGC: SLR* (INTERPRETER.agc)
 *
 * FUNCTION NAME: mu_slr_star
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   a         agc_dp_t        I    the value
 *   count     int             I    left shift (right when negative)
 *
 * RETURNS: agc_dp_t -- A shifted with rounding
 **********************************************************************/
static agc_dp_t mu_slr_star(agc_interp_t *ip, agc_dp_t a, int count)
{
    if (count < 0)
    {
        return (agc_dp_shr_round(a, -count)); /* negative: right */
    }

    return (agc_dp_shl_round(ip, a, count));
}

/**********************************************************************
 * SR* on the TP MPAC: right by count (left when negative).
 * AGC: SR* on the TP MPAC (INTERPRETER.agc)
 *
 * FUNCTION NAME: mu_tp_sr
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   v         agc_tp_t        I    TP value
 *   count     int             I    right shift (left when negative)
 *
 * RETURNS: agc_tp_t -- the shifted TP
 **********************************************************************/
static agc_tp_t mu_tp_sr(agc_interp_t *ip, agc_tp_t v, int count)
{
    return (agc_tp_shift(ip, v, -count));
}

/**********************************************************************
 * B0 complemented when the LM is the vehicle being corrected (BON
 * VEHUPFLG +1), or when the CSM is (BOFF VEHUPFLG +1).
 * AGC: "BON / BOFF VEHUPFLG, VCOMP" (P20-P25.agc p.577)
 *
 * FUNCTION NAME: mu_sign
 *
 * ARGUMENTS:
 *   Argument     Type       I/O  Description (scale)
 *   -----------  ---------  ---  ----------------------------------
 *   b            agc_vec_t  I    B vector
 *   comp_if_csm  bool       I    complement when the CSM (true) or the LM
 *                                (false) is corrected
 *
 * RETURNS: agc_vec_t -- B or -B
 **********************************************************************/
static agc_vec_t mu_sign(agc_vec_t b, bool comp_if_csm)
{
    bool csm = agc_flag_test(AGC_FLAG_VEHUPFLG); /* CSM corrected */

    if (csm == comp_if_csm)
    {
        return (agc_vec_neg(b)); /* VCOMP */
    }

    return (b);
}

/* ---- INTGRCAL / WLINIT (p.584) ---------------------------------------- */
/**********************************************************************
 * INTGRCAL: TDEC1 = MKTIME, INTEGRV (which releases INTSTALL).
 * AGC: INTGRCAL (P20-P25.agc p.584)
 *
 * FUNCTION NAME: mu_intgrcal
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- the INTEGRV status
 **********************************************************************/
static agc_status_t mu_intgrcal(agc_interp_t *ip)
{
    agc_status_t st; /* INTEGRV */

    st = agc_integ_integrv(ip, agc_integ_shared_ctx(), lmnav_state()->mktime);

    agc_phase_change(04022); /* GRP2PC   */

    return (st);
}

/**********************************************************************
 * WLINIT: W = 0 except the diagonal: position WRENDPOS (surface WSURFPOS)
 * SR 5 (to B-19), velocity WRENDVEL (WSURFVEL), shaft and trunnion bias
 * WSHAFT, WTRUN; RENDWFLG on, MARKCTR = 0.
 * AGC: WLINIT (P20-P25.agc p.584)
 *
 * FUNCTION NAME: mu_wlinit
 *
 * ARGUMENTS: none
 **********************************************************************/
static void mu_wlinit(void)
{
    agc_incorp_w_t            *w = &agc_integ_shared_ctx()->w; /* W matrix */
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();        /* pad loads */
    bool                       surf;                           /* on the moon */

    surf = agc_flag_test(AGC_FLAG_SURFFLAG);

    for (int b = 0; b < AGC_INCORP_BLKS; b++)
    {
        for (int m = 0; m < AGC_INCORP_ROWS; m++)
        {
            w->blk[b][m] = agc_vec_zero();
        }
    }

    agc_sp_t wpos = p->wrendpos; /* WRENDPOS, pad scale */
    agc_sp_t wvel = p->wrendvel; /* WRENDVEL, pad scale */

    if (surf)
    {
        wpos = p->wsurfpos; /* WSURFPOS on the surface */
        wvel = p->wsurfvel;
    }

    agc_dp_t pos = agc_dp_shr(agc_dp_from_sp(wpos), 5); /* SR 5, B-19 */
    agc_dp_t vel = agc_dp_from_sp(wvel);                /* pad scale */

    for (int k = 0; k < 3; k++)
    {
        w->blk[0][k].c[k]     = pos; /* W +0,8,16 */
        w->blk[1][3 + k].c[k] = vel; /* W +72,80,88 */
    }

    w->blk[2][6].c[0] = agc_dp_from_sp(p->wshaft); /* W +144   */
    w->blk[2][7].c[1] = agc_dp_from_sp(p->wtrun);  /* W +152   */
    rz_flag_put(AGC_FLAG_RENDWFLG, true);
    agc_erase_set_trkmkcnt(0);
}

/**********************************************************************
 * the two integrations (LSR22.3 pp.575-576, LSR22.4 p.583)
 * CSM corrected: LM to MKTIME, then the CSM with W; LM corrected: the
 * other way round. STATEFLG: the results become the permanent states.
 * AGC: LSR22.3 (P20-P25.agc pp.575-576)
 *
 * FUNCTION NAME: mu_integrate
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration status
 *          (INTSTALL sleep)
 **********************************************************************/
static agc_status_t mu_integrate(agc_interp_t *ip, agc_job_id_t resume)
{
    bool         csm = agc_flag_test(AGC_FLAG_VEHUPFLG); /* CSM corrected */
    agc_status_t st  = agc_integ_install(0, resume);     /* INTSTALL */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_VINTFLAG, !csm); /* the other vehicle first */
    agc_integ_setiflgs();
    st = mu_intgrcal(ip);

    if (st == AGC_STATUS_OK)
    {
        st = agc_integ_install(0, resume);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    bool w = agc_flag_test(AGC_FLAG_RENDWFLG); /* W valid */

    rz_flag_put(AGC_FLAG_DIM0FLAG, w); /* NOTWCSM  */

    if (w)
    {
        rz_flag_put(AGC_FLAG_D6OR9FLG, true);
    }

    rz_flag_put(AGC_FLAG_VINTFLAG, csm);
    rz_flag_put(AGC_FLAG_INTYPFLG, false);
    rz_flag_put(AGC_FLAG_STATEFLG, true);

    return (mu_intgrcal(ip));
}

/**********************************************************************
 * LSR22.4: the LM (from the landing site) and the CSM, 6x6 W; the first
 * mark initializes W (INITWMX6) and integrates the CSM without it.
 * AGC: LSR22.4, INITWMX6 (P20-P25.agc p.583)
 *
 * FUNCTION NAME: mu_integrate_surface
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration status
 *          (INTSTALL sleep)
 **********************************************************************/
static agc_status_t mu_integrate_surface(agc_interp_t *ip, agc_job_id_t resume)
{
    agc_status_t st = agc_integ_install(0, resume); /* INTSTALL */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_STATEFLG, true);
    rz_flag_put(AGC_FLAG_VINTFLAG, false);
    st = mu_intgrcal(ip);
    rz_flag_put(AGC_FLAG_DMENFLG, false); /* 6x6 for INCORP  */

    if (st == AGC_STATUS_OK)
    {
        st = agc_integ_install(0, resume);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_erase_trkmkcnt() == 0)
    { /* INITWMX6 */
        mu_wlinit();
        rz_flag_put(AGC_FLAG_VINTFLAG, true);
        agc_integ_setiflgs();

        return (mu_intgrcal(ip));
    }

    rz_flag_put(AGC_FLAG_D6OR9FLG, false);
    rz_flag_put(AGC_FLAG_DIM0FLAG, true);
    rz_flag_put(AGC_FLAG_VINTFLAG, true);
    rz_flag_put(AGC_FLAG_INTYPFLG, false);

    return (mu_intgrcal(ip));
}

/* ---- GETULC / RADARANG (pp.585-586) ----------------------------------- */
/**********************************************************************
 * GETULC: CSM - LM from the conic states plus deviations (TDELTAV shifted
 * from 2^22 / 2^18 to the body scale), unitized with NORMUNIT; RLC.
 * AGC: GETULC (P20-P25.agc p.585)
 *
 * FUNCTION NAME: mu_getulc
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: none
 **********************************************************************/
static void mu_getulc(agc_interp_t *ip)
{
    agc_integ_sv_t l; /* LM state */
    agc_integ_sv_t c; /* CSM state */

    agc_integ_perm_get(AGC_VEHICLE_LM, &l); /* LM state */
    agc_integ_perm_get(AGC_VEHICLE_CSM, &c);

    int       k = 9 - mu.x2; /* VSR* 9D,2 */
    agc_vec_t rl;            /* body scale */

    rl = agc_vec_add(ip, agc_vec_shr(l.deltav, k), l.rcv);
    agc_vec_t rc; /* body scale */

    rc           = agc_vec_add(ip, agc_vec_shr(c.deltav, k), c.rcv);
    int      x1  = 0; /* NORM index, -(left shifts) */
    agc_dp_t mag = 0; /* |RC - RL|, 2^(S+X1) */

    mu.ulc = agc_rtb_normunit(ip, agc_vec_sub(ip, rc, rl), &x1, &mag, NULL);
    mu.rlc = rz_shift(ip, mag, -x1); /* SL* 0,1  */
}

/**********************************************************************
 * RADARANG: SINTHETA = -ULC . MY (SL1R); RXZ = RLC cos(theta), NORM.
 * AGC: RADARANG (P20-P25.agc p.586)
 *
 * FUNCTION NAME: mu_radarang
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: none
 **********************************************************************/
static void mu_radarang(agc_interp_t *ip)
{
    mu_getulc(ip); /* GETULC */

    agc_dp_t s = agc_vec_dot(ip, agc_vec_neg(mu.ulc), mu.my); /* 2^2 */

    mu.sintheta = agc_dp_shl_round(ip, s, 1);
    agc_dp_t c  = agc_dp_sqrt(agc_dp_sub(ip, AGC_DP_QUARTER, agc_dp_sq(mu.sintheta))); /* 2^1 */

    c      = agc_dp_shl(ip, agc_dp_mul_trunc(c, mu.rlc), 1);
    mu.rxz = rz_norm(c, &mu.rxz_x1);
}

/**********************************************************************
 * the measurements (RANGEBQ .. TRUNBQ pp.576-582)
 * AGC: RATEBQ, VOK, VOK1, VOK2 (P20-P25.agc pp.577-578)
 *
 * FUNCTION NAME: mu_rate
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: none
 **********************************************************************/
static void mu_rate(agc_interp_t *ip)
{
    /* Range rate: B1 = ULC RLC (B-24/B-22), B0 = (ULC x VLC) x ULC
     * (B-5), DELTAQ = (RDOTM - ULC . VLC) RLC (B-33), VARIANCE =
     * max(RDOT^2 RATEVAR, VVARMIN) RLC^2; B0, B1, DELTAQ normalized
     * together (the smaller shift of B0 and B1).                      */
    agc_vec_t      vlc; /* VC - VL, m/cs body scale */
    agc_integ_sv_t l;   /* LM state */
    agc_integ_sv_t c;   /* CSM state */

    mu_getulc(ip); /* GETULC */
    agc_integ_perm_get(AGC_VEHICLE_LM, &l);
    agc_integ_perm_get(AGC_VEHICLE_CSM, &c);

    mu.rlc            = rz_sr_star(ip, mu.rlc, -4, mu.x2); /* to B-23  */
    mu.inc.bvector[1] = agc_vec_scale(ip, mu_sign(mu.ulc, false), mu.rlc);
    int       k       = 6 - mu.x2;                         /* VSR* 6,2 */
    agc_vec_t vl;                                          /* body scale */

    vl = agc_vec_add(ip, agc_vec_shr(l.nuv, k), l.vcv);
    agc_vec_t vc; /* body scale */

    vc = agc_vec_add(ip, agc_vec_shr(c.nuv, k), c.vcv);

    vlc                          = agc_vec_sub(ip, vc, vl);
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad(); /* pad loads */
    agc_dp_t                   rdot;                    /* B-8/B-6 */

    rdot = agc_vec_dot(ip, mu.ulc, vlc);
    agc_dp_t r8; /* B-8 */

    r8            = mu_srr_star(ip, rdot, 2 - mu.x2);
    agc_dp_t var  = agc_dp_mul(ip, agc_dp_sq(r8), p->ratevar);  /* B-4 */
    agc_dp_t vmin = agc_dp_shr(agc_dp_from_sp(p->vvarmin), 16); /* B-4 */

    if (agc_dp_sub(ip, vmin, var) >= 0)
    {
        var = vmin; /* the minimum variance (VOK otherwise) */
    }

    agc_dp_t dq = agc_dp_sub(ip, mu_slr_star(ip, lmnav_state()->rdotm, -1 - mu.x2), agc_dp_shr(rdot, 2)); /* B-10/B-8 */

    dq = agc_dp_mul(ip, dq, mu.rlc); /* B-33     */
    agc_vec_t b0;                    /* 2^4 */

    b0 = agc_vec_cross(ip, agc_vec_cross(ip, mu.ulc, vlc), mu.ulc);

    b0                = rz_vshift(ip, mu_sign(b0, false), 2 + mu.x2);       /* VSR* */
    mu.inc.bvector[0] = b0;
    mu.inc.bvector[2] = agc_vec_zero();
    int s0            = rz_norm_xreg(agc_vec_abval(ip, b0));                /* 20D      */
    int s1            = rz_norm_xreg(agc_vec_abval(ip, mu.inc.bvector[1])); /* 22D   */
    int x1            = s1;                                                 /* VOK1/VOK2: the smaller shift */

    if (s1 - s0 < 0)
    {
        x1 = s0;
    }

    mu.inc.bvector[0] = rz_vshift(ip, b0, -x1);            /* VSL* 0,1 */
    mu.inc.bvector[1] = rz_vshift(ip, mu.inc.bvector[1], -x1);
    mu.inc.deltaq     = rz_shift(ip, dq, -x1);
    agc_dp_t rl       = rz_shift(ip, mu.rlc, -1 - x1);     /* SL* 0-1,1, B-24 */
    agc_tp_t v        = agc_dp_mul_tp(agc_dp_sq(rl), var); /* TP, B-44 */

    mu.inc.variance = agc_tp_shift(ip, v, 4); /* SL4, TP  */
} /* mu_rate */

/**********************************************************************
 * Angle measurement common part: B2 has RXZ (SR* 0-2,1, SRR* 0,2) in
 * component `bias` (shaft 0, trunnion 1); VARIANCE = (VAR + IMUVAR)
 * RXZ^2 (B-40); DELTAQ = (measured - computed angle (radians) - X789
 * bias) RXZ, un-normalized.
 * AGC: SHAFTBQ, TRUNBQ common part (P20-P25.agc pp.580-582)
 *
 * FUNCTION NAME: mu_angle
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   bias      int             I    B2 component and X789 bias: 0 shaft, 1
 *                                  trunnion
 *   var       agc_sp_t        I    SHAFTVAR or TRUNVAR, rad^2 2^12 / 2^10
 *   dangle    agc_dp_t        I    measured - computed angle, radians B0
 **********************************************************************/
static void mu_angle(agc_interp_t *ip, int bias, agc_sp_t var, agc_dp_t dangle)
{
    int       nx = -mu.rxz_x1;     /* shifts   */
    agc_vec_t b2 = agc_vec_zero(); /* B2 */

    b2.c[bias]        = mu_srr_star(ip, rz_sr_star(ip, mu.rxz, -2, mu.rxz_x1), 0 - mu.x2);
    mu.inc.bvector[1] = agc_vec_zero();
    mu.inc.bvector[2] = b2;
    agc_dp_t v        = agc_dp_add(ip, agc_dp_from_sp(var), MU_IMUVAR); /* B12 */
    agc_tp_t t;                                                         /* SRR* 0,1, TP, B-40 */

    t = agc_tp_shift(ip, agc_dp_mul_tp(v, mu.rxz), -nx); /* SRR* 0,1 */

    t               = agc_dp_mul_tp(agc_tp_round(ip, t), mu.rxz);
    t               = mu_tp_sr(ip, t, nx - 2);                     /* SR* 0-2,1 */
    t               = mu_tp_sr(ip, t, 0 - mu.x2);
    mu.inc.variance = mu_tp_sr(ip, t, 0 - mu.x2);
    agc_dp_t xb     = agc_dp_shl(ip, agc_erase_x789().c[bias], 3); /* SL3, B-2/B0 */

    xb = mu_srr_star(ip, xb, 0 - mu.x2); /* from B-5/B-3 to B0  */
    agc_tp_t dq;                         /* TP, B0 */

    dq = agc_dp_mul_tp(agc_dp_sub(ip, dangle, xb), mu.rxz);

    mu.inc.deltaq = agc_tp_round(ip, agc_tp_shift(ip, dq, -nx)); /* SRR* 0,1 */
}

/* ---- LGCUPDTE / ASTOK (p.586) ------------------------------------------ */
/**********************************************************************
 * INCORP1, then the deviations for noun 49 (R22DISP: |dR| B-29, |dV|
 * B-7). True when they exceed RMAX or VMAX (R22LEM96).
 * AGC: LGCUPDTE (P20-P25.agc p.586)
 *
 * FUNCTION NAME: mu_incorp1
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   st        agc_status_t *  O    the INCORP1 status
 *
 * RETURNS: bool -- true when |dR| > RMAX or |dV| > VMAX (noun 49)
 **********************************************************************/
static bool mu_incorp1(agc_interp_t *ip, agc_status_t *st)
{
    agc_integ_ctx_t *ctx; /* integration erasables */

    ctx = agc_integ_shared_ctx();
    lmnav_state_t *n; /* LM navigation erasables */

    n                            = lmnav_state();
    const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad(); /* pad loads */

    mu.inc.dim9   = agc_flag_test(AGC_FLAG_DMENFLG);
    *st           = agc_incorp_state_deviation(ip, &ctx->w, &mu.inc);
    n->r22disp[1] = mu_srr_star(ip, agc_vec_abval(ip, mu.inc.deltax[1]), 2 - mu.x2);
    n->r22disp[0] = mu_srr_star(ip, agc_vec_abval(ip, mu.inc.deltax[0]), 2 - mu.x2);
    agc_dp_t rmax = agc_dp_shr(agc_dp_from_sp(p->rmax), 10); /* m 2^29 */

    if (agc_dp_sub(ip, rmax, n->r22disp[0]) < 0)
    {
        return (true);
    }

    return (agc_dp_sub(ip, agc_dp_from_sp(p->vmax), n->r22disp[1]) < 0);
}

/**********************************************************************
 * Range (RANGEBQ p.576): B0 = ULC (complemented if the LM is corrected),
 * DELTAQ = RM - RLC, VARIANCE = max(RLC^2 RANGEVAR, RVARMIN) (TP B-40).
 * AGC: RANGEBQ, QOK (P20-P25.agc pp.576-577)
 *
 * FUNCTION NAME: mu_range
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: none
 **********************************************************************/
static void mu_range(agc_interp_t *ip)
{
    const agc_rendz_nav_pad_t *p    = agc_rendz_nav_pad();   /* pad loads */
    agc_tp_t                   vmin = (agc_tp_t) p->rvarmin; /* B-40 */

    mu_getulc(ip); /* GETULC */

    mu.inc.bvector[0] = mu_sign(mu.ulc, false);
    mu.inc.bvector[1] = agc_vec_zero();
    mu.inc.bvector[2] = agc_vec_zero();
    agc_dp_t rlc      = mu_srr_star(ip, mu.rlc, 2 - mu.x2);     /* m 2^29 */
    agc_dp_t r        = agc_dp_sub(ip, lmnav_state()->rm, rlc); /* m 2^29 */

    mu.inc.deltaq = mu_slr_star(ip, r, 2 - mu.x2);            /* B-29/B-27 */
    int      x1   = 0;                                        /* NORM index, -(left shifts) */
    agc_dp_t n    = rz_norm(mu.rlc, &x1);                     /* 2^(S+X1) */
    agc_tp_t v    = agc_dp_mul_tp(agc_dp_sq(n), p->rangevar); /* TP, B12 */

    v               = mu_tp_sr(ip, v, -2 - x1); /* SR* 0-2,1 */
    v               = mu_tp_sr(ip, v, 0 - x1);
    v               = mu_tp_sr(ip, v, 0 - mu.x2);
    v               = mu_tp_sr(ip, v, 0 - mu.x2);
    mu.inc.variance = vmin;                     /* RVARMIN unless larger (QOK) */

    if (vmin - v < 0)
    {
        mu.inc.variance = v;
    }
}

/* ---- the measurement sequence ------------------------------------------- */
/**********************************************************************
 * One measurement: its B vectors, DELTAQ, VARIANCE.
 * AGC: RANGEBQ, RATEBQ, SHAFTBQ, TRUNBQ (P20-P25.agc pp.576-582)
 *
 * FUNCTION NAME: mu_compute
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   step      mu_step_t       I    the measurement
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the status of the step;
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t mu_compute(agc_interp_t *ip, mu_step_t step)
{
    lmnav_state()->whchread = (agc_sp_t) step;                    /* WHCHREAD */
    rz_grp2pc(AGC_JOB_RENDEZVOUS_TRACKING_DATA_MEASURE_CONTINUE); /* GRP2PC   */

    switch (step)
    {
        case MU_RANGE:
            mu_range(ip);

            return (AGC_STATUS_OK); /* RANGEBQ */
        case MU_RATE:
            mu_rate(ip);

            return (AGC_STATUS_OK);
        case MU_SHAFT:
        { /* Shaft (SHAFTBQ p.580): angle = ARCTAN(ULC . MX, ULC . MZ); B0 = UNIT(ULC x MY), complemented if the CSM is corrected. */
            /* MXMYMZ (p.579): the NB unit axes at the mark (CDUs AIG AMG AOG) in
             * reference axes: M = (UNIT NB ->SM) x REFSMMAT, VSL1.
             * AGC: MXMYMZ (P20-P25.agc p.579) */
            lmnav_state_t *n        = lmnav_state();        /* LM navigation erasables */
            agc_mat_t      refsmmat = agc_erase_refsmmat(); /* REFSMMAT, 2^1 */
            agc_cdutrig_t  trig;                            /* CDUTRIG sines / cosines */
            uint16_t       yzx[3]   = {n->aig, n->amg, n->aog};
            agc_vec_t      ax[3]    = {agc_vec_make(AGC_DP_HALF, 0, 0),
                                       agc_vec_make(0, AGC_DP_HALF, 0),
                                       agc_vec_make(0, 0, AGC_DP_HALF)};

            agc_imu_trg_nbsm(&trig, yzx, &ax[0]); /* UNITX */
            agc_imu_nbsm(&trig, &ax[1]);          /* UNITY *NBSM* */
            agc_imu_nbsm(&trig, &ax[2]);          /* UNITZ *NBSM* */

            mu.mx = agc_vec_shl(ip, agc_vec_mat(ip, ax[0], &refsmmat), 1);
            mu.my = agc_vec_shl(ip, agc_vec_mat(ip, ax[1], &refsmmat), 1);
            mu.mz = agc_vec_shl(ip, agc_vec_mat(ip, ax[2], &refsmmat), 1);
            mu_radarang(ip); /* RADARANG */

            agc_dp_t sinth; /* 2^1 */

            sinth = agc_dp_shl(ip, agc_vec_dot(ip, mu.ulc, mu.mx), 1);
            agc_dp_t costh; /* 2^1 */

            costh      = agc_dp_shl(ip, agc_vec_dot(ip, mu.ulc, mu.mz), 1);
            agc_dp_t a = agc_latlong_arctan(ip, sinth, costh); /* rev 2^0 */
            agc_dp_t d;                                        /* rev 2^0 */

            d = agc_dp_sub(ip, lmnav_state()->rrshaft, a);

            /* SL3R */
            d = agc_dp_shl_round(ip, agc_dp_mul_trunc(d, MU_2PI_8), 3);
            agc_vec_t b0; /* 2^1 */

            b0 = agc_vec_shl(ip, agc_vec_cross(ip, mu.ulc, mu.my), 1);

            mu.inc.bvector[0] = mu_sign(agc_vec_unit(ip, b0, NULL, NULL), true);
            mu_angle(ip, 0, agc_rendz_nav_pad()->shaftvar, d);

            return (AGC_STATUS_OK);
        } /* end MU_SHAFT case */
        case MU_TRUNNION:
        { /* Trunnion (TRUNBQ p.581): angle = ASIN(SINTHETA); B0 = (ULC x MY) x ULC, complemented if the CSM is corrected. */
            agc_vec_t b0; /* 2^1 */

            mu_radarang(ip); /* RADARANG */
            b0 = agc_vec_shl(ip, agc_vec_cross(ip, mu.ulc, mu.my), 1);

            b0                = agc_vec_shl(ip, agc_vec_cross(ip, b0, mu.ulc), 1);
            mu.inc.bvector[0] = mu_sign(b0, true);
            agc_dp_t d        = agc_dp_sub(ip, lmnav_state()->rrtrun, agc_dp_asin(ip, mu.sintheta)); /* rev 2^0 */

            d = agc_dp_shl_round(ip, agc_dp_mul_trunc(d, MU_2PI_8), 3);
            mu_angle(ip, 1, agc_rendz_nav_pad()->trunvar, d);

            return (AGC_STATUS_OK);
        }

        case MU_END:
        default:
            return (AGC_STATUS_BAD_ARG); /* unreachable: cannot happen */
    } /* switch (step) */
} /* mu_compute */

/**********************************************************************
 * The next measurement after an incorporated one: on the surface the
 * angles are not done (RENDEND after range rate).
 * AGC: RENDEND after RATEBQ on the surface (P20-P25.agc p.583)
 *
 * FUNCTION NAME: mu_next
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------------------------
 *   step      mu_step_t  I    the measurement just incorporated
 *
 * RETURNS: mu_step_t -- the next measurement, or MU_END
 **********************************************************************/
static mu_step_t mu_next(mu_step_t step)
{
    if (step == MU_RATE && agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return (MU_END);
    }

    if (step >= MU_TRUNNION)
    {
        return (MU_END);
    }

    return ((mu_step_t) ((int) step + 1));
}

/**********************************************************************
 * Measurements from mu.step on: each INCORP1; too large a change stops
 * here for the noun 49 display (R22LEM96); else INCORP2 (ASTOK). At the
 * end R22LEM93.
 * AGC: LSR22.3 measurement sequence (P20-P25.agc pp.576-582)
 *
 * FUNCTION NAME: mu_run
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ip        agc_interp_t *   I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the status that stopped
 *          the sequence; AGC_STATUS_NO_CONVERGE when its step bound is
 *          exhausted
 **********************************************************************/
static agc_status_t mu_run(agc_interp_t *ip)
{
    for (int i = 0; i < MU_STEPS; i++)
    {
        if (mu.step == MU_END)
        {
            return (agc_job_rendezvous_tracking_data_done()); /* RENDEND */
        }

        agc_status_t st  = mu_compute(ip, mu.step); /* B, DELTAQ */
        bool         big = false;                   /* over RMAX/VMAX */

        if (st == AGC_STATUS_OK)
        {
            big = mu_incorp1(ip, &st);
        }

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        if (big)
        {
            return (lmnav_residuals_display()); /* R22LEM96 */
        }

        st = rz_incorp2(ip, &mu.inc, agc_flag_test(AGC_FLAG_LMOONFLG)); /* ASTOK: INCORP2, MOONTHIS = LMOONFLG */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        rz_grp2pc(AGC_JOB_RENDEZVOUS_TRACKING_DATA_MEASURE_CONTINUE);
        mu.step = mu_next(mu.step);
    }

    return (AGC_STATUS_NO_CONVERGE); /* step bound exhausted */
}

/**********************************************************************
 * ASTOK after the crew accepted the noun 49 display (R22LEM7): INCORP2
 * of the waiting measurement, then the rest.
 * AGC: R22LEM7, ASTOK (P20-P25.agc pp.507, 586)
 *
 * FUNCTION NAME: lmnav_incorporate_accepted
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the INCORP2 status
 **********************************************************************/
agc_status_t lmnav_incorporate_accepted(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(mu.step != MU_END))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = rz_incorp2(&ip, &mu.inc, agc_flag_test(AGC_FLAG_LMOONFLG)); /* INCORP2, MOONTHIS = LMOONFLG */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_grp2pc(AGC_JOB_RENDEZVOUS_TRACKING_DATA_MEASURE_CONTINUE);
    mu.step = mu_next(mu.step);

    return (mu_run(&ip));
}

/**********************************************************************
 * LSR22.3 / LSR22.4: states to the mark time; W reinitialized if it was
 * invalidated (MARKTEST); then (RANGEBQ) R65 once unless on the surface,
 * and the measurements.
 * AGC: LSR22.3, LSR22.4, MARKTEST (P20-P25.agc pp.575, 583)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_incorporate
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_incorporate(void)
{
    rz_grp2pc(AGC_JOB_RENDEZVOUS_TRACKING_DATA_INCORPORATE); /* GRP2PC   */
    agc_interp_t ip;                                         /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st; /* integration status */

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        st = mu_integrate_surface(&ip, AGC_JOB_RENDEZVOUS_TRACKING_DATA_INCORPORATE);
    }
    else
    {
        rz_flag_put(AGC_FLAG_DMENFLG, true);
        st = mu_integrate(&ip, AGC_JOB_RENDEZVOUS_TRACKING_DATA_INCORPORATE);

        if (st == AGC_STATUS_OK && !agc_flag_test(AGC_FLAG_RENDWFLG))
        {
            mu_wlinit(); /* MARKTEST */
        }
    }

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return (agc_job_rendezvous_tracking_data_measure());
    }

    lmnav_state()->r65cntr = 0;

    return (lmnav_tracking_attitude(false, AGC_JOB_RENDEZVOUS_TRACKING_DATA_MEASURE));
} /* agc_job_rendezvous_tracking_data_incorporate */

/**********************************************************************
 * RANGEBQ1: SCALSHFT (0 moon, 2 earth), then the measurements.
 * AGC: RANGEBQ1 (P20-P25.agc p.576)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_measure
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_measure(void)
{
    mu.x2 = 2; /* earth */

    if (agc_flag_test(AGC_FLAG_LMOONFLG))
    {
        mu.x2 = 0; /* moon */
    }

    rz_grp2pc(AGC_JOB_RENDEZVOUS_TRACKING_DATA_MEASURE);
    mu.step = MU_RANGE;
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);

    return (mu_run(&ip));
}

/**********************************************************************
 * Restart point of the GRP2PCs inside the measurement sequence (the
 * AGC's return addresses in RANGEBQ .. TRUNBQ, R22LEM93 and ASTOK): the
 * sequence continues at the measurement it had reached (WHCHREAD /
 * mu.step), not from RANGEBQ, so an incorporated measurement is not
 * incorporated again.
 * AGC: GRP2PC return addresses (P20-P25.agc pp.575-586)
 *
 * FUNCTION NAME: agc_job_rendezvous_tracking_data_measure_continue
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, else the status of the step that failed
 **********************************************************************/
agc_status_t agc_job_rendezvous_tracking_data_measure_continue(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);

    return (mu_run(&ip));
}
