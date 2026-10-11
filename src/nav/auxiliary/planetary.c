/**********************************************************************
 * FILE NAME: planetary.c
 *
 * PURPOSE:
 *   PLANETARY INERTIAL ORIENTATION: conversion between the basic
 *   reference (inertial) frame and the earth-fixed or moon-fixed frame,
 *       R-TO-RP:  RP = M(T) (R - L x R)
 *       RP-TO-R:  R  = MT(T) (RP + LP x RP)
 *   with M from EARTHMX (rotation about Z by AZ) or MOONMX (lunar node,
 *   inclination and libration angles), and L the small-angle earth
 *   precession vector (EARTHL) or the lunar libration vector 504LM.
 *   NEWANGLE forms the time-varying angles.  Also the pad-load store.
 *
 * AGC SOURCE:
 *   Comanche055 PLANETARY_INERTIAL_ORIENTATION.agc pp. 1243-1251
 *     (RP-TO-R, R-TO-RP, MOONMX, NEWANGLE, EARTHMX, EARTHL)
 *   Luminary099 PLANETARY_INERTIAL_ORIENTATION.agc (identical apart from
 *     bank placement and LO6ZEROS for HI6ZEROS, both a zero vector)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type       I/O  Scale     Description
 *   ------------  ---------  ---  --------  --------------------------
 *   TEPHEM        agc_tp_t   I/O  2^42 cs   agc_erase_tephem / _set_
 *   RLS           agc_vec_t  I/O  2^27 m    agc_erase_rls / _set_
 *   AZO, AXO,     agc_dp_t   I/O  2^0       module static padload
 *   -AYO, 504LM   agc_vec_t                 (no core accessor yet)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dp_sin/cos        SIN, COS (interp.h)
 *   agc_mat_vec/vec_mat   MXV, VXM (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms.  AGC_STATUS_BAD_ARG on NULL or out-of-range inputs and
 *   for times before the TEPHEM epoch.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Job context, both vehicles.  Constants p. 1251 (LM CONTROLLED_
 *   CONSTANTS pp. 52-53).
 *
 * NOTES:
 *   NEWANGLE can leave an angle as a sign-disagreeing word pair; SIN/COS
 *   of it differ by 1 LSB from the joined value (agc_dp_t cannot carry
 *   the split): MOONMX <= 4 LSB, docs/modules/review-nav-aux.md.
 *
 * ALGORITHM:
 *   See PURPOSE; each function prolog gives its AGC formula.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* ---- constants (p. 1251; LM CONTROLLED_CONSTANTS p. 52-53) ------------ */
/* COSI 2DEC .99964173 B-1, oct 17775 02052: cos(I) scale 2^1, I = 1 deg
 * 32.1 min, mean lunar equator to ecliptic.                               */
static const agc_dp_t COSI = 134169642; /* cos I, scale 2^1      */

/* SINI 2DEC .02676579 B-1, oct 00333 10374: sin(I) scale 2^1.            */
static const agc_dp_t SINI = 3592444; /* sin I, scale 2^1      */

/* NODDOT 2DEC -.457335121 E-2, oct 77665 42175: rev/cs scale 2^-28
 * = -1.07047011E-8 rad/s, regression of the lunar node.                   */
static const agc_dp_t NODDOT = -1227650; /* rev/cs, 2^-28   */

/* FDOT 2DEC .570863327, oct 22211 00625: rev/cs scale 2^-27
 * = 2.67240410E-6 rad/s, mean motion of the moon's mean longitude.        */
static const agc_dp_t FDOT = 153239957; /* rev/cs, 2^-27   */

/* BDOT 2DEC -3.07500686 E-8, oct 77777 77767: rev/cs scale 2^-28
 * = -7.19757301E-14 rad/s (obliquity rate).                               */
static const agc_dp_t BDOT = -8; /* rev/cs, 2^-28   */

/* NODIO 2DEC .986209434, oct 37436 01613: revs 2^0 = 6.19653663041 rad.  */
static const agc_dp_t NODIO = 264733579; /* rev, 2^0 */

/* FSUBO 2DEC .829090536, oct 32417 32160: revs 2^0 = 5.20932947829 rad.  */
static const agc_dp_t FSUBO = 222557296; /* rev, 2^0 */

/* BSUBO 2DEC .0651201393, oct 02052 35552: revs 2^0 = 0.40916190299 rad. */
static const agc_dp_t BSUBO = 17480554; /* rev, 2^0 */

/* WEARTH 2DEC .973561595, oct 37116 32523: rev/cs scale 2^-23
 * = 7.29211494E-5 rad/s, earth rotation rate.                             */
static const agc_dp_t WEARTH = 261338451; /* rev/cs, 2^-23 */

/* NEWANGLE shift selectors (the AGC's X1 values).                         */
#define XDOT_SHIFT_WEARTH 0 /* WEARTH 2^-23: SL* 5,1 -> 5      */
#define XDOT_SHIFT_NODE   5 /* NODDOT, BDOT 2^-28: SL* 0       */
#define XDOT_SHIFT_F      4 /* FDOT 2^-27: SL* 1               */

/* Pad-loaded erasables (TEPHEM, AZO, -AYO, AXO, 504LM, RLS).             */
static agc_navaux_padload_t padload; /* AZO AXO -AYO 504LM, see type */

/**********************************************************************
 * AGC: none (pad load of TEPHEM, AZO, AXO, -AYO, 504LM, RLS)
 *   -- store the launch-time erasables
 *
 * FUNCTION NAME: agc_navaux_set_padload
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   pad       const agc_navaux_padload_t *  I    pad load (scales in
 *                                                the type, auxiliary.h)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL or
 *          out-of-range TEPHEM / 504LM.  TEPHEM and RLS go to their core
 *          erasables (their only store), the rest to the module static.
 **********************************************************************/
agc_status_t agc_navaux_set_padload(const agc_navaux_padload_t *pad)
{
    if (!AGC_ASSERT(agc_navaux_tp_ok(pad->tephem) && agc_navaux_vec_ok(pad->lm504)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    padload = *pad;
    agc_erase_set_tephem(pad->tephem); /* one TEPHEM: core erasable */
    agc_erase_set_rls(pad->rls);       /* one RLS: core erasable    */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (pad load) -- read the launch-time erasables back
 *
 * FUNCTION NAME: agc_navaux_get_padload
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ----------------------------
 *   out       agc_navaux_padload_t *  O    pad load; TEPHEM and RLS from
 *                                          the core erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL
 **********************************************************************/
agc_status_t agc_navaux_get_padload(agc_navaux_padload_t *out)
{
    if (!AGC_ASSERT(agc_navaux_vec_ok(padload.lm504)))
    {
        return (AGC_STATUS_BAD_ARG); /* store corrupted */
    }

    *out        = padload;
    out->tephem = agc_erase_tephem(); /* P11 / V70 may have moved it */
    out->rls    = agc_erase_rls();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * NEWANGLE (PLANETARY_INERTIAL_ORIENTATION.agc p. 1248, label NEWANGLE)
 *   -- X = X0 + XDOT (T + T0), revs scale 2^0, modulo one revolution
 *
 * FUNCTION NAME: agc_planet_newangle
 *
 * ARGUMENTS:
 *   Argument    Type            I/O  Description (scale)
 *   ----------  --------------  ---  ----------------------------------
 *   ip          agc_interp_t *  I/O  interpreter state (OVFIND)
 *   timsubo     agc_tp_t        I    T0 = TEPHEM, scale 2^42 cs
 *   t_cs        agc_dp_t        I    T, scale 2^28 cs, T + T0 >= 0
 *   x0          agc_dp_t        I    X0, revs scale 2^0
 *   xdot        agc_dp_t        I    XDOT, rev/cs at 2^-(23+xdot_shift)
 *   xdot_shift  int             I    XDOT_SHIFT_* (the AGC's X1), 0..5
 *   x           agc_dp_t *      O    X, revs scale 2^0
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL, a
 *          bad shift or T + T0 < 0
 *
 * T + T0 is formed in TP (cs 2^42) and multiplied in two parts, the low
 * 28 bits (TIMSUBM+1, cs 2^28) and the high word (TIMSUBM, SL 9 -> 2^33),
 * each product shifted to revolutions (SL* 5,1 and SL* 10D,1).  The DMP
 * products stay TP through the shift (MPAC+2), DAD adds to the top two
 * words (no rounding); SL and DAD overflow wraps mod one revolution, and
 * the final BOV clears OVFIND.  Bit-exact against Comanche055 (40 random
 * cases).  The AGC can leave X as a sign-disagreeing word pair, which
 * SIN/COS see (up to 1 LSB in MOONMX, docs/modules/review-nav-aux.md).
 **********************************************************************/
agc_status_t agc_planet_newangle(agc_interp_t *ip, agc_tp_t timsubo, agc_dp_t t_cs, agc_dp_t x0, agc_dp_t xdot, int xdot_shift, agc_dp_t *x)
{
    /* DLOAD SR 6D 14D / TAD TIMSUBO: T+T0, cs scale 2^42 (TIMSUBM).       */
    agc_tp_t tsum = agc_tp_shift(ip, agc_tp_from_dp(t_cs), -14); /* 2^42 cs */

    tsum = agc_tp_add(ip, tsum, timsubo);

    if (!AGC_ASSERT(tsum >= 0))
    {
        return (AGC_STATUS_BAD_ARG); /* times before the epoch: not used */
    }

    /* DLOAD TIMSUBM +1, DMP XDOT, SL* 5,1 (on the TP MPAC), DAD X0 (on its
     * top two words).                                                      */
    agc_dp_t t_low = agc_navaux_tp_low_dp(tsum);                                   /* TIMSUBM+1, 2^28 cs */
    agc_tp_t prod  = agc_tp_shift(ip, agc_dp_mul_tp(t_low, xdot), 5 - xdot_shift); /* XDOT T, 2^0 rev */
    agc_dp_t part  = agc_dp_add(ip, agc_tp_to_dp(prod), x0);                       /* 2^0 rev */

    /* SLOAD TIMSUBM, SL 9 (cs 2^33), DMP XDOT, SL* 10D,1, DAD partial.     */
    agc_sp_t t_high = agc_dp_high(agc_tp_to_dp(tsum));           /* TIMSUBM, 2^14 */
    agc_dp_t t_hi9  = agc_dp_shl(ip, agc_dp_from_sp(t_high), 9); /* 2^33 cs */

    prod = agc_tp_shift(ip, agc_dp_mul_tp(t_hi9, xdot), 10 - xdot_shift);
    *x   = agc_dp_add(ip, agc_tp_to_dp(prod), part);
    agc_interp_clear_ovf(ip); /* BOV +1: leave OVFIND clear      */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * EARTHMX (PLANETARY_INERTIAL_ORIENTATION.agc p. 1249, label EARTHMX)
 *   -- earth-fixed rotation matrix
 *       M = | cos AZ   sin AZ  0 |
 *           | -sin AZ  cos AZ  0 |  scale 2^1,
 *           | 0        0       1 |  AZ = AZO + WEARTH (T + TEPHEM)
 *
 * FUNCTION NAME: agc_planet_earthmx
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    TEPHEM, AZO
 *   t_cs      agc_dp_t                      I    T, scale 2^28 cs
 *   m         agc_mat_t *                   O    M, scale 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on NULL
 *          or a NEWANGLE failure
 **********************************************************************/
agc_status_t agc_planet_earthmx(agc_interp_t *ip, const agc_navaux_padload_t *pad, agc_dp_t t_cs, agc_mat_t *m)
{
    agc_dp_t     az = 0;                                                                                    /* 504AZ revs 2^0                  */
    agc_status_t st = agc_planet_newangle(ip, pad->tephem, t_cs, pad->azo, WEARTH, XDOT_SHIFT_WEARTH, &az); /* NEWANGLE status */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_dp_t c = agc_dp_cos(ip, az); /* cos AZ, scale 2^1               */
    agc_dp_t s = agc_dp_sin(ip, az); /* sin AZ, scale 2^1               */

    m->row[0] = agc_vec_make(c, s, 0);
    m->row[1] = agc_vec_make(agc_dp_neg(s), c, 0);
    m->row[2] = agc_vec_make(0, 0, AGC_DP_HALF); /* HIDPHALF        */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * EARTHL (PLANETARY_INERTIAL_ORIENTATION.agc p. 1250, label EARTHL)
 *   -- earth precession vector L = (-AXO, -AYO, 0)
 *
 * FUNCTION NAME: agc_planet_earthl
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   pad       const agc_navaux_padload_t *  I    AXO, -AYO, 2^0 rad
 *   l         agc_vec_t *                   O    L (504LPL), 2^0 rad
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL or
 *          out-of-range angles
 **********************************************************************/
agc_status_t agc_planet_earthl(const agc_navaux_padload_t *pad, agc_vec_t *l)
{
    *l = agc_vec_make(agc_dp_neg(pad->axo), pad->neg_ayo, 0); /* 504LPL  */

    return (AGC_STATUS_OK);
}

/* Moon angles B, F, NODI (MOONMX first half, p. 1246). */
typedef struct moon_angles
{
    agc_dp_t cob;  /* COB  cos(B)/2, scale 2^1                          */
    agc_dp_t sob;  /* SOB  sin(B)/2, scale 2^1                          */
    agc_dp_t f;    /* 504F revs 2^0                                     */
    agc_dp_t nodi; /* NODI revs 2^0                                     */
} moon_angles_t;

/**********************************************************************
 * AGC: MOONMX p. 1246 (DMP SL1R) -- product of two half-scaled values
 *
 * FUNCTION NAME: mul_half
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   a         agc_dp_t        I    factor, scale 2^1
 *   b         agc_dp_t        I    factor, scale 2^1
 *
 * RETURNS: agc_dp_t -- a b, scale 2^1: TP product shifted, rounded once
 **********************************************************************/
static agc_dp_t mul_half(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    return (agc_tp_round(ip, agc_tp_shift(ip, agc_dp_mul_tp(a, b), 1)));
}

/**********************************************************************
 * AGC: MOONMX pp. 1247-1248 (VXSC, VXSC, VSU/VAD, VSL1)
 *   -- u su -+ v sv, back to scale 2^1
 *
 * FUNCTION NAME: combine
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   u         agc_vec_t       I    first vector, scale 2^1
 *   su        agc_dp_t        I    its scalar, scale 2^1
 *   v         agc_vec_t       I    second vector, scale 2^1
 *   sv        agc_dp_t        I    its scalar, scale 2^1
 *   subtract  bool            I    true: VSU, false: VAD
 *
 * RETURNS: agc_vec_t -- (u su -+ v sv) VSL1, scale 2^1
 **********************************************************************/
static agc_vec_t combine(agc_interp_t *ip, agc_vec_t u, agc_dp_t su, agc_vec_t v, agc_dp_t sv, bool subtract)
{
    agc_vec_t a = agc_vec_scale(ip, u, su); /* scale 2^2           */
    agc_vec_t b = agc_vec_scale(ip, v, sv); /* scale 2^2           */
    agc_vec_t c = agc_vec_add(ip, a, b);    /* VAD, scale 2^2      */

    if (subtract)
    {
        c = agc_vec_sub(ip, a, b); /* VSU, scale 2^2 */
    }

    return (agc_vec_shl(ip, c, 1)); /* VSL1 -> scale 2^1   */
}

/**********************************************************************
 * MOONMX (PLANETARY_INERTIAL_ORIENTATION.agc pp. 1246-1248, label
 *   MOONMX) -- moon-fixed rotation matrix
 *     A = (cos N, sin N cos B, sin N sin B)
 *     B = (-sin N, cos N cos B, cos N sin B)
 *     C = (0, -sin B, cos B)
 *     D = B cos I - C sin I
 *     M0 = -(A cos F + D sin F), M1 = A sin F - D cos F,
 *     M2 = B sin I + C cos I;  all vectors and M scale 2^1
 *
 * FUNCTION NAME: agc_planet_moonmx
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    TEPHEM
 *   t_cs      agc_dp_t                      I    T, scale 2^28 cs
 *   m         agc_mat_t *                   O    M, scale 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on NULL
 *          or a NEWANGLE failure
 **********************************************************************/
agc_status_t agc_planet_moonmx(agc_interp_t *ip, const agc_navaux_padload_t *pad, agc_dp_t t_cs, agc_mat_t *m)
{
    moon_angles_t ang;                 /* COB SOB 504F NODI, see type */
    agc_tp_t      tsubo = pad->tephem; /* TEPHEM, scale 2^42 cs   */

    agc_dp_t     b         = 0;                                                                      /* 504B (obliquity), 2^0 rev */
    agc_status_t st_angles = agc_planet_newangle(ip, tsubo, t_cs, BSUBO, BDOT, XDOT_SHIFT_NODE, &b); /* NEWANGLE status */

    if (st_angles == AGC_STATUS_OK)
    {
        st_angles = agc_planet_newangle(ip, tsubo, t_cs, FSUBO, FDOT, XDOT_SHIFT_F, &ang.f);
    }

    if (st_angles == AGC_STATUS_OK)
    {
        st_angles = agc_planet_newangle(ip, tsubo, t_cs, NODIO, NODDOT, XDOT_SHIFT_NODE, &ang.nodi);
    }

    if (!AGC_ASSERT(st_angles == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    ang.cob = agc_dp_cos(ip, b);
    ang.sob = agc_dp_sin(ip, b);

    agc_dp_t  cosn   = agc_dp_cos(ip, ang.nodi);                      /* COSNODI, 2^1 */
    agc_dp_t  sinn   = agc_dp_sin(ip, ang.nodi);                      /* SINNODI, 2^1 */
    agc_dp_t  sncb   = mul_half(ip, sinn, ang.cob);                   /* sin N cos B, 2^1 */
    agc_dp_t  snsb   = mul_half(ip, sinn, ang.sob);                   /* sin N sin B, 2^1 */
    agc_dp_t  cncb   = mul_half(ip, cosn, ang.cob);                   /* cos N cos B, 2^1 */
    agc_dp_t  cnsb   = mul_half(ip, cosn, ang.sob);                   /* cos N sin B, 2^1 */
    agc_vec_t avectr = agc_vec_make(cosn, sncb, snsb);                /* AVECTR, 2^1 */
    agc_vec_t bvectr = agc_vec_make(agc_dp_neg(sinn), cncb, cnsb);    /* 2^1 */
    agc_vec_t cvectr = agc_vec_make(0, agc_dp_neg(ang.sob), ang.cob); /* 2^1 */

    m->row[2]        = combine(ip, cvectr, COSI, bvectr, SINI, false);
    agc_vec_t dvectr = combine(ip, bvectr, COSI, cvectr, SINI, true); /* 2^1 */
    agc_dp_t  cosf   = agc_dp_cos(ip, ang.f);                         /* COS 504F, 2^1 */
    agc_dp_t  sinf   = agc_dp_sin(ip, ang.f);                         /* SIN 504F, 2^1 */

    m->row[1] = combine(ip, avectr, sinf, dvectr, cosf, true);
    m->row[0] = agc_vec_neg(combine(ip, avectr, cosf, dvectr, sinf, false));

    return (AGC_STATUS_OK);
} /* agc_planet_moonmx */

/**********************************************************************
 * AGC: R-TO-RP / RP-TO-R pp. 1243-1245 (common part)
 *   -- M matrix and the libration/precession vector
 *
 * FUNCTION NAME: planet_m_and_l
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    pad load
 *   moon      bool                          I    true: MOONMX/504LM
 *   t_cs      agc_dp_t                      I    T, scale 2^28 cs
 *   want_lp   bool                          I    false: L (inertial, for
 *                                                R-TO-RP); true: LP
 *                                                (planetary, RP-TO-R)
 *   m         agc_mat_t *                   O    M, scale 2^1
 *   l         agc_vec_t *                   O    L or LP, scale 2^0 rad
 *
 * RETURNS: agc_status_t -- status of EARTHMX/MOONMX/EARTHL, or
 *          AGC_STATUS_BAD_ARG on NULL
 **********************************************************************/
static agc_status_t planet_m_and_l(agc_interp_t *ip, const agc_navaux_padload_t *pad, bool moon, agc_dp_t t_cs, bool want_lp, agc_mat_t *m, agc_vec_t *l)
{
    agc_status_t st = AGC_STATUS_OK; /* returned to the caller */

    if (moon)
    {
        st = agc_planet_moonmx(ip, pad, t_cs, m);
        *l = pad->lm504; /* LP = 504LM, moon-fixed */

        if (!want_lp)
        {
            *l = agc_vec_shl(ip, agc_vec_mat(ip, pad->lm504, m), 1); /* L */
        }
    }
    else
    {
        st = agc_planet_earthmx(ip, pad, t_cs, m);

        if (st == AGC_STATUS_OK)
        {
            st = agc_planet_earthl(pad, l); /* L, rad 2^0     */
        }

        if (want_lp)
        {
            *l = agc_vec_shl(ip, agc_mat_vec(ip, m, *l), 1); /* LP = M L   */
        }
    }

    return (st);
}

/**********************************************************************
 * R-TO-RP (PLANETARY_INERTIAL_ORIENTATION.agc p. 1245, label R-TO-RP)
 *   -- RP = M(T) (R - L x R)
 *
 * FUNCTION NAME: agc_planet_r_to_rp
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    pad load
 *   moon      bool                          I    true: moon-fixed frame
 *   r         agc_vec_t                     I    R, any scale 2^n
 *   t_cs      agc_dp_t                      I    T, scale 2^28 cs
 *   rp        agc_vec_t *                   O    RP, scale 2^n (of R)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG, or the
 *          status of the matrix routines
 **********************************************************************/
agc_status_t agc_planet_r_to_rp(agc_interp_t *ip, const agc_navaux_padload_t *pad, bool moon, agc_vec_t r, agc_dp_t t_cs, agc_vec_t *rp)
{
    if (!AGC_ASSERT(agc_navaux_vec_ok(r)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t    m;                                                       /* M(T), scale 2^1  */
    agc_vec_t    l;                                                       /* L, scale 2^0 rad */
    agc_status_t st = planet_m_and_l(ip, pad, moon, t_cs, false, &m, &l); /* returned to the caller */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t d = agc_vec_sub(ip, r, agc_vec_cross(ip, l, r)); /* 2^n */

    *rp = agc_vec_shl(ip, agc_mat_vec(ip, &m, d), 1); /* RPRPXXXX*/

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * RP-TO-R (PLANETARY_INERTIAL_ORIENTATION.agc p. 1243, label RP-TO-R)
 *   -- R = MT(T) (RP + LP x RP)
 *
 * FUNCTION NAME: agc_planet_rp_to_r
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    pad load
 *   moon      bool                          I    true: moon-fixed frame
 *   rp        agc_vec_t                     I    RP, any scale 2^n
 *   t_cs      agc_dp_t                      I    T, scale 2^28 cs
 *   r         agc_vec_t *                   O    R, scale 2^n (of RP)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG, or the
 *          status of the matrix routines
 **********************************************************************/
agc_status_t agc_planet_rp_to_r(agc_interp_t *ip, const agc_navaux_padload_t *pad, bool moon, agc_vec_t rp, agc_dp_t t_cs, agc_vec_t *r)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (r != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_vec_ok(rp)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t    m;                                                       /* M(T), scale 2^1   */
    agc_vec_t    lp;                                                      /* LP, scale 2^0 rad */
    agc_status_t st = planet_m_and_l(ip, pad, moon, t_cs, true, &m, &lp); /* returned to the caller */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t s = agc_vec_add(ip, agc_vec_cross(ip, lp, rp), rp); /* 2^n */

    *r = agc_vec_shl(ip, agc_vec_mat(ip, s, &m), 1); /* VXM MMATRIX, VSL1 */

    return (AGC_STATUS_OK);
}
