/*
 * planetary.c -- PLANETARY INERTIAL ORIENTATION
 * (Comanche055 PLANETARY_INERTIAL_ORIENTATION.agc pp. 1243-1251; the
 * Luminary099 code is identical apart from bank placement and LO6ZEROS in
 * place of HI6ZEROS, both a zero vector).
 *
 * Converts between the basic reference (inertial) frame and the earth-fixed
 * or moon-fixed planetary frame:
 *     R-TO-RP:  RP = M(T) (R - L x R)
 *     RP-TO-R:  R  = MT(T) (RP + LP x RP)
 * with M from EARTHMX (rotation about Z by AZ) or MOONMX (lunar node,
 * inclination and libration angles), and L the small-angle earth
 * precession vector (EARTHL) or the lunar libration vector 504LM.
 *
 * Also owns the launch-time pad load (TEPHEM, AZO, AXO, -AYO, 504LM, RLS),
 * which the AGC keeps in erasable; here module-private (requested as
 * shared accessors, see docs/modules/nav-aux.md).
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* ---- constants (p. 1251; LM CONTROLLED_CONSTANTS p. 52-53) ------------ */
/* COSI 2DEC .99964173 B-1, oct 17775 02052: cos(I) scale 2^1, I = 1 deg
 * 32.1 min, mean lunar equator to ecliptic.                               */
static const agc_dp_t COSI = 134169642;
/* SINI 2DEC .02676579 B-1, oct 00333 10374: sin(I) scale 2^1.            */
static const agc_dp_t SINI = 3592444;
/* NODDOT 2DEC -.457335121 E-2, oct 77665 42175: rev/cs scale 2^-28
 * = -1.07047011E-8 rad/s, regression of the lunar node.                   */
static const agc_dp_t NODDOT = -1227650;
/* FDOT 2DEC .570863327, oct 22211 00625: rev/cs scale 2^-27
 * = 2.67240410E-6 rad/s, mean motion of the moon's mean longitude.        */
static const agc_dp_t FDOT = 153239957;
/* BDOT 2DEC -3.07500686 E-8, oct 77777 77767: rev/cs scale 2^-28
 * = -7.19757301E-14 rad/s (obliquity rate).                               */
static const agc_dp_t BDOT = -8;
/* NODIO 2DEC .986209434, oct 37436 01613: revs 2^0 = 6.19653663041 rad.  */
static const agc_dp_t NODIO = 264733579;
/* FSUBO 2DEC .829090536, oct 32417 32160: revs 2^0 = 5.20932947829 rad.  */
static const agc_dp_t FSUBO = 222557296;
/* BSUBO 2DEC .0651201393, oct 02052 35552: revs 2^0 = 0.40916190299 rad. */
static const agc_dp_t BSUBO = 17480554;
/* WEARTH 2DEC .973561595, oct 37116 32523: rev/cs scale 2^-23
 * = 7.29211494E-5 rad/s, earth rotation rate.                             */
static const agc_dp_t WEARTH = 261338451;

/* NEWANGLE shift selectors (the AGC's X1 values).                         */
#define XDOT_SHIFT_WEARTH 0
#define XDOT_SHIFT_NODE   5
#define XDOT_SHIFT_F      4

/* Pad-loaded erasables (TEPHEM, AZO, -AYO, AXO, 504LM, RLS).             */
static agc_navaux_padload_t padload;

agc_status_t agc_navaux_set_padload(const agc_navaux_padload_t *pad)
{
    if (!AGC_ASSERT(pad != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_tp_ok(pad->tephem) &&
                    agc_navaux_vec_ok(pad->lm504)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    padload = *pad;

    return (AGC_STATUS_OK);
}

agc_status_t agc_navaux_get_padload(agc_navaux_padload_t *out)
{
    if (!AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *out = padload;

    return (AGC_STATUS_OK);
}

/*
 * NEWANGLE (PLANETARY_INERTIAL_ORIENTATION.agc p. 1248, label NEWANGLE)
 * X = X0 + XDOT (T + T0), revs scale 2^0, modulo one revolution.
 * T + T0 is formed in TP (cs 2^42) and multiplied in two parts, the low
 * 28 bits (TIMSUBM+1, cs 2^28) and the high word (TIMSUBM, SL 9 -> 2^33),
 * each product shifted to revolutions (SL* 5,1 and SL* 10D,1).
 * DEVIATION: the AGC lets these shifts and adds overflow (angle wraps mod
 * one revolution); modelled with agc_navaux_rev_*.  The DMP products are
 * carried in TP through the shift (MPAC+2) and rounded once.
 */
agc_status_t agc_planet_newangle(agc_interp_t *ip, agc_tp_t timsubo,
                                 agc_dp_t t_cs, agc_dp_t x0, agc_dp_t xdot,
                                 int xdot_shift, agc_dp_t *x)
{
    if (!AGC_ASSERT((ip != NULL) && (x != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((xdot_shift >= 0) && (xdot_shift <= 5)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* DLOAD SR 6D 14D / TAD TIMSUBO: T+T0, cs scale 2^42 (TIMSUBM).       */
    agc_tp_t tsum = agc_tp_add(ip, agc_navaux_tp_from_cs(t_cs), timsubo);

    if (!AGC_ASSERT(tsum >= 0))
    {
        return (AGC_STATUS_BAD_ARG); /* times before the epoch: not used */
    }

    /* DLOAD TIMSUBM +1, DMP XDOT, SL* 5,1, DAD X0.                        */
    agc_dp_t t_low = agc_navaux_tp_low_dp(tsum);
    agc_dp_t part  =
        agc_navaux_rev_tp_shl(agc_dp_mul_tp(t_low, xdot), 5 - xdot_shift);

    part = agc_navaux_rev_add(part, x0);
    /* SLOAD TIMSUBM, SL 9 (cs 2^33), DMP XDOT, SL* 10D,1, DAD partial.     */
    agc_sp_t t_high = agc_dp_high(agc_tp_to_dp(tsum)); /* TIMSUBM  */
    agc_dp_t t_hi9  = agc_dp_shl(ip, agc_dp_from_sp(t_high), 9);
    agc_dp_t part2  =
        agc_navaux_rev_tp_shl(agc_dp_mul_tp(t_hi9, xdot), 10 - xdot_shift);

    *x = agc_navaux_rev_add(part2, part);
    agc_interp_clear_ovf(ip); /* BOV +1: leave OVFIND clear      */

    return (AGC_STATUS_OK);
}

/*
 * EARTHMX (PLANETARY_INERTIAL_ORIENTATION.agc p. 1249, label EARTHMX)
 * M = | cos AZ   sin AZ  0 |
 *     | -sin AZ  cos AZ  0 |  scale 2^1,  AZ = AZO + WEARTH (T + TEPHEM).
 *     | 0        0       1 |
 */
agc_status_t agc_planet_earthmx(agc_interp_t *ip,
                                const agc_navaux_padload_t *pad, agc_dp_t t_cs,
                                agc_mat_t *m)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (m != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t     az = 0; /* 504AZ revs 2^0                  */
    agc_status_t st = agc_planet_newangle(ip, pad->tephem, t_cs, pad->azo,
                                          WEARTH, XDOT_SHIFT_WEARTH, &az);

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

/*
 * EARTHL (PLANETARY_INERTIAL_ORIENTATION.agc p. 1250, label EARTHL)
 * L = (-AXO, -AYO, 0), rad scale 2^0.
 */
agc_status_t agc_planet_earthl(const agc_navaux_padload_t *pad, agc_vec_t *l)
{
    if (!AGC_ASSERT((pad != NULL) && (l != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(pad->axo) &&
                    agc_navaux_dp_ok(pad->neg_ayo)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

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

static agc_status_t moonmx_angles(agc_interp_t *ip, agc_tp_t tsubo,
                                  agc_dp_t t_cs, moon_angles_t *a)
{
    if (!AGC_ASSERT((ip != NULL) && (a != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t     b  = 0;
    agc_status_t st =
        agc_planet_newangle(ip, tsubo, t_cs, BSUBO, BDOT, XDOT_SHIFT_NODE, &b);

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_newangle(ip, tsubo, t_cs, FSUBO, FDOT, XDOT_SHIFT_F,
                                 &a->f);
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_newangle(ip, tsubo, t_cs, NODIO, NODDOT,
                                 XDOT_SHIFT_NODE, &a->nodi);
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    a->cob = agc_dp_cos(ip, b);
    a->sob = agc_dp_sin(ip, b);

    return (AGC_STATUS_OK);
}

/* DMP SL1R: product of two half-scaled quantities, back to scale 2^1. */
static agc_dp_t mul_half(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0);
    }

    return (agc_dp_shl_round(ip, agc_dp_mul(ip, a, b), 1));
}

/* VXSC of two vectors by two scalars, VSU/VAD, VSL1 (MOONMX second half). */
static agc_vec_t combine(agc_interp_t *ip, agc_vec_t u, agc_dp_t su,
                         agc_vec_t v, agc_dp_t sv, bool subtract)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (agc_vec_zero());
    }

    agc_vec_t a = agc_vec_scale(ip, u, su); /* scale 2^2           */
    agc_vec_t b = agc_vec_scale(ip, v, sv);
    agc_vec_t c = subtract ? agc_vec_sub(ip, a, b) : agc_vec_add(ip, a, b);

    return (agc_vec_shl(ip, c, 1)); /* VSL1 -> scale 2^1   */
}

/*
 * MOONMX (PLANETARY_INERTIAL_ORIENTATION.agc pp. 1246-1248, label MOONMX)
 *   A = (cos N, sin N cos B, sin N sin B)     B = (-sin N, cos N cos B,
 *   C = (0, -sin B, cos B)                         cos N sin B)
 *   D = B cos I - C sin I
 *   M0 = -(A cos F + D sin F),  M1 = A sin F - D cos F,  M2 = B sin I + C cos I
 * All vectors and M scale 2^1.
 */
agc_status_t agc_planet_moonmx(agc_interp_t *ip,
                               const agc_navaux_padload_t *pad, agc_dp_t t_cs,
                               agc_mat_t *m)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (m != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    moon_angles_t ang;
    agc_status_t  st = moonmx_angles(ip, pad->tephem, t_cs, &ang);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_dp_t  cosn   = agc_dp_cos(ip, ang.nodi);
    agc_dp_t  sinn   = agc_dp_sin(ip, ang.nodi); /* SINNODI     */
    agc_vec_t avectr = agc_vec_make(cosn, mul_half(ip, sinn, ang.cob),
                                    mul_half(ip, sinn, ang.sob));
    agc_vec_t bvectr =
        agc_vec_make(agc_dp_neg(sinn), mul_half(ip, cosn, ang.cob),
                     mul_half(ip, cosn, ang.sob));
    agc_vec_t cvectr = agc_vec_make(0, agc_dp_neg(ang.sob), ang.cob);

    m->row[2] = combine(ip, cvectr, COSI, bvectr, SINI, false);
    agc_vec_t dvectr = combine(ip, bvectr, COSI, cvectr, SINI, true);
    agc_dp_t  cosf   = agc_dp_cos(ip, ang.f);
    agc_dp_t  sinf   = agc_dp_sin(ip, ang.f);

    m->row[1] = combine(ip, avectr, sinf, dvectr, cosf, true);
    m->row[0] = agc_vec_neg(combine(ip, avectr, cosf, dvectr, sinf, false));

    return (AGC_STATUS_OK);
}

/* M matrix and the libration/precession vector in the form each of
 * R-TO-RP (want_lp = false: L, inertial) and RP-TO-R (true: LP, planetary)
 * needs it.                                                               */
static agc_status_t planet_m_and_l(agc_interp_t *ip,
                                   const agc_navaux_padload_t *pad, bool moon,
                                   agc_dp_t t_cs, bool want_lp, agc_mat_t *m,
                                   agc_vec_t *l)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((m != NULL) && (l != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st;

    if (moon)
    {
        st = agc_planet_moonmx(ip, pad, t_cs, m);
        *l = want_lp ? pad->lm504 /* LP = 504LM     */
             :
             agc_vec_shl(ip, agc_vec_mat(ip, pad->lm504, m), 1);
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

/*
 * R-TO-RP (PLANETARY_INERTIAL_ORIENTATION.agc p. 1245, label R-TO-RP)
 * RP = M(T) (R - L x R); output at the scale of the input.
 */
agc_status_t agc_planet_r_to_rp(agc_interp_t *ip,
                                const agc_navaux_padload_t *pad, bool moon,
                                agc_vec_t r, agc_dp_t t_cs, agc_vec_t *rp)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (rp != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_vec_ok(r)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t    m;
    agc_vec_t    l;
    agc_status_t st = planet_m_and_l(ip, pad, moon, t_cs, false, &m, &l);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t d = agc_vec_sub(ip, r, agc_vec_cross(ip, l, r)); /* RTORPB  */

    *rp = agc_vec_shl(ip, agc_mat_vec(ip, &m, d), 1);  /* RPRPXXXX*/

    return (AGC_STATUS_OK);
}

/*
 * RP-TO-R (PLANETARY_INERTIAL_ORIENTATION.agc p. 1243, label RP-TO-R)
 * R = MT(T) (RP + LP x RP); output at the scale of the input.
 */
agc_status_t agc_planet_rp_to_r(agc_interp_t *ip,
                                const agc_navaux_padload_t *pad, bool moon,
                                agc_vec_t rp, agc_dp_t t_cs, agc_vec_t *r)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (r != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_vec_ok(rp)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t    m;
    agc_vec_t    lp;
    agc_status_t st = planet_m_and_l(ip, pad, moon, t_cs, true, &m, &lp);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t s = agc_vec_add(ip, agc_vec_cross(ip, lp, rp), rp); /* RPTORB */

    *r = agc_vec_shl(ip, agc_vec_mat(ip, s, &m), 1); /* VXM MMATRIX, VSL1 */

    return (AGC_STATUS_OK);
}
