/**********************************************************************
 * FILE NAME: dps_burn_rasteer.c
 *
 * PURPOSE:
 *   RASTEER1, the Luminary aimpoint steering: for a Lambert-targeted
 *   burn (XDELVFLG clear) it regenerates the required velocity VR at RN for
 *   reaching RTARG at TPASS4 on the conic whose mu/a INITVEL last found,
 *   without iterating, and leaves DELVEET3 (VG).
 *
 * AGC SOURCE:
 *   Luminary099 P40-P47.agc pp. 776-780
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   RN            agc_vec_t  I    2^29 m       position
 *   PIPTIME       agc_dp_t   I    cs 2^28      time of RN
 *   UN            agc_vec_t  I    half unit    orbit normal (180MESS)
 *   GEOMSGN       agc_sp_t   I    -            sign of the transfer geometry
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_vec_unit          UNIT (interp.h)
 *   agc_dp_sqrt, _asin    SQRT, ASIN (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None raised here; AGC_STATUS_BAD_ARG when MUASTEER, GEOMSGN or a
 *   divisor is out of range.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only; runs in the SERVICER job (UPDATEVG). No loops.
 *
 * NOTES:
 *     s = (r1 + r2 + c)/2, c = |RTARG - RN|, IC = UNIT(RTARG - RN), IR = UNIT/R/
 *     A = GEOMSGN sqrt((s - r1)(2 mu - (s - c) mu/a) / (c r1))
 *     B = sgn(tm - t) sqrt((s - r2)(2 mu - s mu/a) / (c r1))
 *     VR = A UNIT(IC + IR) + B UNIT(IC - IR)
 *   where tm = (s^1.5 / sqrt(2 mu)) (pi/2 -+ (g - sin g cos g)), sin g =
 *   sqrt((s - c)/s), is the minimum-energy transfer time that selects the
 *   branch of B. Near 180 degrees (NORMSW) UNIT(IC - IR) or UNIT(IC + IR) is
 *   replaced by the perpendicular built from UN (180MESS / NEGPROD). Then
 *   DELVEET3 = VR - VN1 plus, in the earth sphere, the oblateness term
 *   -(mu/R^2)(T - GOBLTIME) UNITGOBL. The formulas were checked against an
 *   exact Keplerian orbit (docs/modules/guid-powered.md, Tests written).
 *
 * ALGORITHM:
 *   Scales: Rb = body position scale (2^29 m earth, 2^27 moon, RTX2 = 0 /
 *   2); MU/A 2^14 (earth) / 2^10 (moon) m^2/cs^2, MUASTEER mu 2^42 / 2^36
 *   m^3/cs^2, A and B 2^7 / 2^5 m/cs, VIPRIME and DELVEET3 2^7 m/cs.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/conic.h"
#include "agc/programs/rendezvous.h"

/* 2PI+3: 2DEC 3.141592653 B-2 (pi at 2^2).                                 */
static const agc_dp_t PI_B2 = 210828714; /* pi, 2^2 */

/* EARTHMU: 2DEC* -3.986032 E10 B-36* (m^3/cs^2).                            */
static const agc_dp_t EARTHMU = -155704375; /* EARTHMU, m^3/cs^2 2^36 */

/* The RASTEER1 working cells (the AGC's RMAG, IC, R1C, SS, MUSCALE are
 * kept in agc_guid_state()->lm as well; 30D/32D are locals here).        */
typedef struct rs_work
{
    agc_dp_t  c;        /* 30D  |RTARG - RN| at Rb; scale Rb */
    agc_dp_t  r1c;      /* R1C  c r1 2^(k1+k2) at Rb^2                   */
    agc_sp_t  muscale;  /* MUSCALE -(k1+k2)                              */
    agc_dp_t  ss;       /* SS   (r1 + r2 + c)/2 at Rb; scale Rb */
    agc_dp_t  rmag;     /* RMAG r1 at Rb; scale Rb */
    agc_vec_t ic;       /* IC   half unit; (2^1) */
    agc_dp_t  mu_a;     /* MU/A     mu over the semi-major axis (p.777); scale 2^14 (earth) or 2^10 (moon) m^2/cs^2 */
    agc_dp_t  muasteer; /* MUASTEER mu for the steering, scaled by MUSCALE   */
    agc_dp_t  rtmag;    /* RTMAG    |RTARG| at Rb; scale Rb */
    agc_sp_t  geomsgn;  /* GEOMSGN, sign of the geometry; unscaled */
} rs_work_t;

/**********************************************************************
 * SL* n: left for n > 0 (overflow to OVFIND), truncating right for n < 0.
 * AGC: none (helper for the indexed SL* shifts of RASTEER1, p.777-778).
 *
 * FUNCTION NAME: dp_shift
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   v         agc_dp_t        I    DP value
 *   n         int             I    shift count: > 0 left, < 0 right
 *
 * RETURNS: agc_dp_t -- v 2^n (OVFIND on a left-shift overflow), scale
 *          of v less n
 **********************************************************************/
static agc_dp_t dp_shift(agc_interp_t *ip, agc_dp_t v, int n)
{
    if (n >= 0)
    {
        return (agc_dp_shl(ip, v, n));
    }

    return (agc_dp_shr(v, -n));
}

/**********************************************************************
 * RASTEER1 .. SS (p.776-777): RMAG, IC, 30D, R1C with its normalization
 * MUSCALE, and SS. NORMUNX1 shifts a very short C left 13 (X1).
 *
 * FUNCTION NAME: rs_geometry
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   w         rs_work_t *     I/O  RASTEER1 working cells
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
static agc_status_t rs_geometry(agc_interp_t *ip, rs_work_t *w)
{
    const agc_rendz_t *rz   = agc_rendz_erasable(); /* targeting erasables */
    int                rtx2 = rz->body.rtx2;        /* RTX2: 0 earth, 2 moon */

    if (!AGC_ASSERT(rtx2 == 0 || rtx2 == 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t rn = agc_erase_rn(); /* RN, m at Rb; scale Rb */

    w->rmag       = dp_shift(ip, agc_vec_abval(ip, rn), rtx2); /* LXC,2 SL*      */
    int      n    = 0;                                         /* NORM shift count */
    agc_dp_t cabs = 0;                                         /* |C|, m at Rb; scale Rb */

    w->ic        = agc_rtb_normunit(ip, agc_vec_sub(ip, rz->rtarg, rn), &n, &cabs, NULL);
    w->c         = dp_shift(ip, cabs, rtx2 - n);                 /* XAD,2 SL* X1   */
    int      k1  = 0;                                            /* k1 shift count */
    int      k2  = 0;                                            /* k2 shift count */
    agc_dp_t cn  = agc_dp_norm(w->c, &k1);                       /* NORM X2; |C| normalised, scale Rb 2^-k1 */
    agc_tp_t r1c = agc_tp_norm(agc_dp_mul_tp(cn, w->rmag), &k2); /* DMP NORM X1; c r1 normalised, scale Rb^2 2^-(k1+k2) */

    w->r1c       = agc_tp_to_dp(r1c);
    w->muscale   = (agc_sp_t) (-(k1 + k2));                                         /* SXA,2 MUSCALE  */
    agc_dp_t sum = agc_dp_add(ip, agc_dp_shr(w->rtmag, 1), agc_dp_shr(w->rmag, 1)); /* (r1 + r2)/2, m at Rb; scale Rb */

    w->ss = agc_dp_add(ip, sum, agc_dp_shr(w->c, 1)); /* STORE SS       */

    if (!AGC_ASSERT(w->r1c != 0 && w->ss > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * One velocity component (p.777-778): with L = SS - 30D (for A) or SS
 * (for B) and rk = RMAG or RTMAG:
 *   sqrt( (SS - rk) (MUASTEER - L MU/A) / R1C ), rescaled by MUSCALE.
 *
 * FUNCTION NAME: rs_speed
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  --------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND, index
 *                                     registers)
 *   w         const rs_work_t *  I    RASTEER1 working cells
 *   len       agc_dp_t           I    chord length term, scale as in the
 *                                     listing
 *   rk        agc_dp_t           I    radius ratio term
 *
 * RETURNS: agc_dp_t -- the speed term, m/cs at the scale MUSCALE
 *          selects (p.778)
 **********************************************************************/
static agc_dp_t rs_speed(agc_interp_t *ip, const rs_work_t *w, agc_dp_t len, agc_dp_t rk)
{
    agc_dp_t energy = agc_dp_sub(ip, w->muasteer, agc_dp_mul_trunc(len, w->mu_a)); /* BDSU; MUASTEER - L MU/A, scale of MUASTEER */
    int      k      = 0;                                                           /* NORM shift count */
    agc_dp_t d      = agc_dp_norm(agc_dp_sub(ip, w->ss, rk), &k);                  /* NORM X1; SS - rk normalised, scale Rb 2^-k */

    d          = agc_dp_shr(d, 1);          /* SR1            */
    agc_dp_t q = agc_dp_div(ip, d, w->r1c); /* DDV R1C; (SS - rk)/R1C, scale per p.778 */
    agc_tp_t p = agc_dp_mul_tp(q, energy);  /* DMP (pushed); TP product, scale per p.778 */

    p = agc_tp_shift(ip, p, 1 - (int) w->muscale - k); /* XSU,2 SL* 1,2  */

    return (agc_tp_sqrt(p));
}

/**********************************************************************
 * UNIT(sign(GEOMSGN) (v x UN)) for 180MESS / NEGPROD.
 * AGC: none (shared tail of 180MESS and NEGPROD, p.779).
 *
 * FUNCTION NAME: rs_normal
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   v         agc_vec_t       I    vector to normalise
 *   geomsgn   agc_sp_t        I    GEOMSGN, sign of the geometry
 *
 * RETURNS: agc_vec_t -- the unit normal, half-unit (2^1)
 **********************************************************************/
static agc_vec_t rs_normal(agc_interp_t *ip, agc_vec_t v, agc_sp_t geomsgn)
{
    agc_vec_t n = agc_vec_cross(ip, v, agc_erase_un()); /* VXV UN; v x UN, scale 2^2 v */

    if (geomsgn < 0)
    {
        n = agc_vec_neg(n); /* SIGN GEOMSGN   */
    }

    return (agc_vec_unit(ip, n, NULL, NULL));
}

/**********************************************************************
 * VR (p.778-779): the main formula, or 180MESS (IC.IR >= 0: replace
 * UNIT(IC - IR) by UNIT(sign(-u+ x UN))) or NEGPROD (IC.IR < 0: replace
 * UNIT(IC + IR) by UNIT(sign(u- x UN))). GETVRVG2 rescales to 2^7.
 *
 * FUNCTION NAME: rs_vr
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  --------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND, index
 *                                     registers)
 *   w         const rs_work_t *  I    RASTEER1 working cells
 *   a         agc_dp_t           I    radial coefficient
 *   b         agc_dp_t           I    horizontal coefficient
 *
 * RETURNS: agc_vec_t -- VR, the required velocity, m/cs 2^7
 **********************************************************************/
static agc_vec_t rs_vr(agc_interp_t *ip, const rs_work_t *w, agc_dp_t a, agc_dp_t b)
{
    agc_vec_t ir = agc_guid_state()->avg.unitr; /* UNIT/R/; half unit (2^1) */
    agc_vec_t vr;                               /* VR, m/cs 2^7 */

    if (!agc_flag_test(AGC_FLAG_NORMSW))
    {
        agc_vec_t vb = agc_vec_scale(ip, agc_vec_unit(ip, agc_vec_sub(ip, w->ic, ir), NULL, NULL), b); /* B UNIT(IC - IR), m/cs at the A, B scale */
        agc_vec_t up = agc_vec_unit(ip, agc_vec_add(ip, w->ic, ir), NULL, NULL);                       /* UNIT(IC + IR), half unit; (2^1) */

        vr = agc_vec_add(ip, agc_vec_scale(ip, up, a), vb); /* GETVRVG1       */
    }
    else if (agc_vec_dot(ip, w->ic, ir) >= 0)
    { /* 180MESS        */
        agc_vec_t up = agc_vec_unit(ip, agc_vec_add(ip, agc_vec_shr(ir, 1), agc_vec_shr(w->ic, 1)), NULL, NULL); /* UNIT(IC + IR) (NEGPROD), half unit; (2^1) */
        agc_vec_t vb = agc_vec_scale(ip, rs_normal(ip, agc_vec_neg(up), w->geomsgn), b);                         /* B times the 180MESS normal, m/cs at the A, B scale */

        vr = agc_vec_add(ip, agc_vec_scale(ip, up, a), vb);
    }
    else
    { /* NEGPROD        */
        agc_vec_t um = agc_vec_unit(ip, agc_vec_sub(ip, agc_vec_shr(w->ic, 1), agc_vec_shr(ir, 1)), NULL, NULL); /* UNIT(IC - IR), half unit; (2^1) */
        agc_vec_t va = agc_vec_scale(ip, rs_normal(ip, um, w->geomsgn), a);                                      /* A times the NEGPROD normal, m/cs at the A, B scale */

        vr = agc_vec_add(ip, agc_vec_scale(ip, um, b), va);
    }

    int n = (int) agc_rendz_erasable()->body.rtx2 - 1; /* VSR* 0 -1,2    */

    return ((n < 0) ? agc_vec_shl(ip, vr, -n) : agc_vec_shr(vr, n)); /* SL* by -n or SR* by n */
}

/**********************************************************************
 * ASTREND .. NOGOBL (p.779-780): DELVEET3 = VR - VN1, plus in the earth
 * sphere (RTX2 = 0) G(OBL) = -(MU/R^2) UNITGOBL (PIPTIME - GOBLTIME).
 *
 * FUNCTION NAME: rs_vg
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   vr        agc_vec_t       I    required velocity VR, m/cs 2^7
 *
 * RETURNS: agc_vec_t -- DELVEET3 (VG), m/cs 2^7
 **********************************************************************/
static agc_vec_t rs_vg(agc_interp_t *ip, agc_vec_t vr)
{
    const agc_guid_t *g  = agc_guid_state();                /* guidance erasables */
    agc_vec_t         vg = agc_vec_sub(ip, vr, g->avg.vn1); /* VSU VN1; VG = VR - VN1, m/cs 2^7 */

    if (agc_rendz_erasable()->body.rtx2 != 0)
    {
        return (vg); /* NOGOBL         */
    }

    /* UNIT RN leaves |RN|^2 at 34D (LVSQUARE, the VSQ of RN).          */
    agc_dp_t rsq = agc_vec_sq(ip, agc_erase_rn()); /* RN^2, m^2 at Rb^2; scale Rb^2 */

    agc_dp_t dt = agc_dp_sub(ip, agc_erase_piptime(), g->lm.gobltime); /* PIPTIME - GOBLTIME, cs 2^28 */
    agc_dp_t k  = agc_dp_div(ip, agc_dp_mul_trunc(dt, EARTHMU), rsq);  /* 2^6     */

    return (agc_vec_add(ip, agc_vec_scale(ip, g->avg.gobl1_2, k), vg));
}

/**********************************************************************
 * RASTEER1 (Luminary p.776-780): VIPRIME and DELVEET3 for S40.8, which
 * continues at VGAIN* with DELVEET3 (agc_lm_s40_8).
 *
 * FUNCTION NAME: agc_lm_rasteer1
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
agc_status_t agc_lm_rasteer1(agc_interp_t *ip)
{
    agc_rendz_t   *rz = agc_rendz_erasable();  /* targeting erasables */
    agc_guid_lm_t *lm = &agc_guid_state()->lm; /* LM guidance erasables */
    rs_work_t      w  = {0};

    w.mu_a     = rz->mu_a;
    w.muasteer = rz->muasteer;
    w.rtmag    = rz->rtmag;
    w.geomsgn  = agc_erase_geomsgn();

    if (!AGC_ASSERT(w.muasteer > 0 && w.geomsgn != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = rs_geometry(ip, &w); /* status of rs_geometry */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t a = rs_speed(ip, &w, agc_dp_sub(ip, w.ss, w.c), w.rmag); /* A, m/cs at the A, B scale */

    a          = agc_dp_sign(a, agc_dp_from_sp(w.geomsgn)); /* STORE 32D      */
    agc_dp_t b = rs_speed(ip, &w, w.ss, w.rtmag);           /* -B (NO SIGN); |B|, m/cs at the A, B scale (2^7 or 2^5) */

    /* The minimum-energy transfer time less the time to go to the aimpoint
     * (p.778): tm - (TPASS4 - PIPTIME), cs 2^28. Its sign is that of B.       */
    const agc_rendz_t *rz_error = agc_rendz_erasable();                                         /* targeting erasables (TPASS4) */
    agc_dp_t           x        = agc_dp_sqrt(agc_dp_div(ip, agc_dp_sub(ip, w.ss, w.c), w.ss)); /* sin g, 2^0 */
    agc_dp_t           g        = agc_dp_asin(ip, agc_dp_shr(x, 1));                            /* SR1 ASIN, rev; g, rev 2^0 */
    agc_dp_t           g8       = agc_dp_mul_trunc(g, PI_B2);                                   /* gamma/8 (rad); scale per p.778 */
    agc_dp_t           y        = agc_dp_sqrt(agc_dp_div(ip, w.c, w.ss));                       /* sqrt(c/s), 2^0 */

    agc_interp_clear_ovf(ip);                                               /* BOV +1         */
    agc_dp_t t = agc_dp_sub(ip, g8, agc_dp_shr(agc_dp_mul_trunc(y, x), 3)); /* BDSU; g - sin g cos g, scale per p.778 */

    t          = agc_dp_sign(t, agc_dp_from_sp(w.geomsgn));
    agc_dp_t q = agc_dp_sub(ip, agc_dp_shr(PI_B2, 2), t); /* SR2 DSU; pi/2 -+ (g - sin g cos g), scale per p.778 */

    q            = agc_dp_mul_trunc(q, w.ss);                             /* DMP SS         */
    agc_dp_t num = agc_dp_mul_trunc(agc_dp_sqrt(agc_dp_shr(w.ss, 3)), q); /* s^1.5 term, scale per p.778 */
    agc_dp_t den = agc_dp_sqrt(agc_dp_shl(ip, w.muasteer, 3));            /* SL3 SQRT; sqrt(8 MUASTEER), scale per p.778 */

    if (!AGC_ASSERT(den > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t tm = agc_dp_div(ip, num, den); /* BDDV; tm, the minimum-energy time, cs 2^28 */

    b            = agc_dp_sign(b, agc_dp_add(ip, agc_dp_sub(ip, tm, rz_error->tpass4), agc_erase_piptime())); /* B WITH SIGN    */
    lm->rmag     = w.rmag;
    lm->ic       = w.ic;
    lm->r1c      = w.r1c;
    lm->ss       = w.ss;
    lm->muscale  = w.muscale;
    rz->viprime  = rs_vr(ip, &w, a, b);                                                                       /* STORE VIPRIME  */
    rz->delveet3 = rs_vg(ip, rz->viprime);                                                                    /* STORE DELVEET3 */

    return (AGC_STATUS_OK);
} /* agc_lm_rasteer1 */
