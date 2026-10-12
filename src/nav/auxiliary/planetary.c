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
#include "agc/core/interp_words.h"
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
    *out        = padload;
    out->tephem = agc_erase_tephem(); /* P11 / V70 may have moved it */
    out->rls    = agc_erase_rls();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * NEWANGLE (PLANETARY_INERTIAL_ORIENTATION.agc p. 1248, label NEWANGLE)
 *   -- X = X0 + XDOT (T + T0) on the interpreter's words
 *
 * T + T0 is formed in TP (DLOAD SR 14D, TAD TIMSUBO: TIMSUBM, cs 2^42)
 * and multiplied in two parts, the low two words (TIMSUBM+1, cs 2^28)
 * and the high word (SLOAD TIMSUBM, SL 9 -> 2^33), each DMP product
 * shifted to revolutions on the TP MPAC (SL* 5,1 and SL* 10D,1). The
 * DADs add to the top two words and keep MPAC+2, so X is the TP MPAC
 * the AGC leaves (its words may disagree in sign; SIN and COS read them
 * as they are). SL and DAD overflow wraps mod one revolution; the final
 * BOV clears OVFIND.
 *
 * FUNCTION NAME: newangle_words
 *
 * ARGUMENTS:
 *   Argument    Type              I/O  Description (scale)
 *   ----------  ----------------  ---  --------------------------------
 *   ip          agc_interp_t *    I/O  interpreter state (OVFIND)
 *   timsubo     agc_tp_t          I    T0 = TEPHEM, scale 2^42 cs
 *   t_cs        agc_dp_t          I    T, scale 2^28 cs
 *   x0          agc_dp_t          I    X0, revs scale 2^0
 *   xdot        agc_dp_t          I    XDOT, rev/cs at 2^-(23+xdot_shift)
 *   xdot_shift  int               I    XDOT_SHIFT_* (the AGC's X1), 0..5
 *   x           agc_tp_words_t *  O    X, the TP MPAC, revs scale 2^0
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for
 *          T + T0 < 0 (before the epoch)
 **********************************************************************/
static agc_status_t newangle_words(agc_interp_t *ip, agc_tp_t timsubo, agc_dp_t t_cs, agc_dp_t x0, agc_dp_t xdot, int xdot_shift, agc_tp_words_t *x)
{
    agc_tp_words_t tsum = agc_mpac_shift(ip, agc_mpac_dload(agc_dp_to_words(t_cs)), -14); /* DLOAD SR 14D, 2^42 cs */

    tsum = agc_mpac_tad(ip, tsum, agc_tp_to_words(timsubo)); /* TAD TIMSUBO: TIMSUBM */

    if (!AGC_ASSERT(agc_tp_from_words(tsum) >= 0))
    {
        return (AGC_STATUS_BAD_ARG); /* times before the epoch: not used */
    }

    const agc_dp_words_t tlow  = {tsum.mid, tsum.lo};                                                         /* DLOAD TIMSUBM +1, 2^28 cs */
    const agc_dp_words_t thigh = {tsum.hi, 0};                                                                /* SLOAD TIMSUBM, 2^14 */
    const agc_dp_words_t xdw   = agc_dp_to_words(xdot);
    agc_tp_words_t       p     = agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(tlow), xdw), 5 - xdot_shift); /* DMP, SL* 5,1 */
    const agc_dp_words_t part  = agc_mpac_store(agc_mpac_dad(ip, p, agc_dp_to_words(x0)));                    /* DAD X0, PUSH */

    p  = agc_mpac_shift(ip, agc_mpac_dload(thigh), 9);              /* SL 9: 2^33 cs */
    p  = agc_mpac_shift(ip, agc_mpac_dmp(p, xdw), 10 - xdot_shift); /* DMP XDOT, SL* 10D,1 */
    *x = agc_mpac_dad(ip, p, part);                                 /* DAD partial */
    agc_interp_clear_ovf(ip);                                       /* BOV +1: leave OVFIND clear */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * NEWANGLE (PLANETARY_INERTIAL_ORIENTATION.agc p. 1248, label NEWANGLE)
 *   -- X = X0 + XDOT (T + T0), as stored (newangle_words)
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
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG for
 *          T + T0 < 0
 **********************************************************************/
agc_status_t agc_planet_newangle(agc_interp_t *ip, agc_tp_t timsubo, agc_dp_t t_cs, agc_dp_t x0, agc_dp_t xdot, int xdot_shift, agc_dp_t *x)
{
    agc_tp_words_t     m;                                                                /* X as the TP MPAC */
    const agc_status_t st = newangle_words(ip, timsubo, t_cs, x0, xdot, xdot_shift, &m); /* NEWANGLE status */

    if (st == AGC_STATUS_OK)
    {
        *x = agc_dp_from_words(agc_mpac_store(m));
    }

    return (st);
}

/**********************************************************************
 * EARTHMX (PLANETARY_INERTIAL_ORIENTATION.agc p. 1249, label EARTHMX)
 *   -- earth-fixed rotation matrix on the push-down words
 *       M = | cos AZ   sin AZ  0 |
 *           | -sin AZ  cos AZ  0 |  scale 2^1,
 *           | 0        0       1 |  AZ = AZO + WEARTH (T + TEPHEM)
 * The first COS takes NEWANGLE's TP MPAC, the others the pushed 504AZ.
 *
 * FUNCTION NAME: earthmx_words
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    TEPHEM, AZO
 *   t_cs      agc_dp_t                      I    T, scale 2^28 cs
 *   m         agc_vec_words_t [3]           O    MMATRIX rows as stored,
 *                                                scale 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the NEWANGLE status
 **********************************************************************/
static agc_status_t earthmx_words(agc_interp_t *ip, const agc_navaux_padload_t *pad, agc_dp_t t_cs, agc_vec_words_t m[3])
{
    agc_tp_words_t     azm;                                                                                   /* 504AZ as the TP MPAC */
    const agc_status_t st = newangle_words(ip, pad->tephem, t_cs, pad->azo, WEARTH, XDOT_SHIFT_WEARTH, &azm); /* NEWANGLE status */

    if (st == AGC_STATUS_OK)
    {
        const agc_dp_words_t az   = agc_mpac_store(azm);                              /* PUSH: 504AZ */
        const agc_dp_words_t zero = {0, 0};                                           /* HI6ZEROS */
        const agc_dp_words_t s    = agc_mpac_store(agc_mpac_sin(agc_mpac_dload(az))); /* SIN, 2^1 */

        m[0].c[0] = agc_mpac_store(agc_mpac_cos(azm));                                /* COS of the TP MPAC */
        m[0].c[1] = s;
        m[0].c[2] = zero;
        m[1].c[0] = agc_mpac_store(agc_mpac_dcomp(agc_mpac_sin(agc_mpac_dload(az)))); /* SIN DCOMP */
        m[1].c[1] = agc_mpac_store(agc_mpac_cos(agc_mpac_dload(az)));                 /* COS */
        m[1].c[2] = zero;
        m[2].c[0] = zero;
        m[2].c[1] = zero;
        m[2].c[2] = agc_dp_to_words(AGC_DP_HALF);                                     /* HIDPHALF */
    }

    return (st);
}

/**********************************************************************
 * EARTHMX (PLANETARY_INERTIAL_ORIENTATION.agc p. 1249, label EARTHMX)
 *   -- earth-fixed rotation matrix (earthmx_words), as values
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
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the NEWANGLE status
 **********************************************************************/
agc_status_t agc_planet_earthmx(agc_interp_t *ip, const agc_navaux_padload_t *pad, agc_dp_t t_cs, agc_mat_t *m)
{
    agc_vec_words_t    w[3];                                 /* MMATRIX words */
    const agc_status_t st = earthmx_words(ip, pad, t_cs, w); /* EARTHMX status */

    if (st == AGC_STATUS_OK)
    {
        for (int i = 0; i < 3; i++)
        {
            m->row[i] = agc_vec_from_words(w[i]);
        }
    }

    return (st);
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

/**********************************************************************
 * AGC: MOONMX p. 1247 (DMP SL1R) -- a product of two half-scaled words
 *
 * FUNCTION NAME: dmp_sl1r
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   a         agc_dp_words_t  I    MPAC as stored, scale 2^1
 *   b         agc_dp_words_t  I    operand as stored, scale 2^1
 *
 * RETURNS: agc_dp_words_t -- a b, scale 2^1 (SL1R of the TP product)
 **********************************************************************/
static agc_dp_words_t dmp_sl1r(agc_interp_t *ip, agc_dp_words_t a, agc_dp_words_t b)
{
    return (agc_mpac_store(agc_mpac_shift_round(ip, agc_mpac_dmp(agc_mpac_dload(a), b), 1)));
}

/**********************************************************************
 * AGC: MOONMX pp. 1247-1248 (VXSC, VXSC, VAD/VSU, VSL1) -- u su +- v sv
 * on the stored words, back to scale 2^1; MPAC holds v sv, the push-down
 * list u su (VAD: MPAC + PD, VSU: PD - MPAC)
 *
 * FUNCTION NAME: combine
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ------------------------------------
 *   ip        agc_interp_t *   I/O  interpreter state (overflow)
 *   u         agc_vec_words_t  I    pushed vector, scale 2^1
 *   su        agc_tp_words_t   I    its scalar, scale 2^1
 *   v         agc_vec_words_t  I    MPAC vector, scale 2^1
 *   sv        agc_tp_words_t   I    its scalar, scale 2^1
 *   subtract  bool             I    true: VSU (u su - v sv)
 *
 * RETURNS: agc_vec_words_t -- (u su +- v sv) VSL1, scale 2^1
 **********************************************************************/
static agc_vec_words_t combine(agc_interp_t *ip, agc_vec_words_t u, agc_tp_words_t su, agc_vec_words_t v, agc_tp_words_t sv, bool subtract)
{
    const agc_vec_words_t pd = agc_mpac_vxsc(ip, su, u); /* pushed, scale 2^2 */
    agc_vec_words_t       c  = agc_mpac_vxsc(ip, sv, v); /* MPAC, scale 2^2 */

    for (int i = 0; i < 3; i++)
    {
        agc_tp_words_t s = agc_mpac_dload(c.c[i]); /* component */

        s      = subtract ? agc_mpac_bdsu(ip, s, pd.c[i]) : agc_mpac_dad(ip, s, pd.c[i]); /* VSU / VAD */
        c.c[i] = agc_mpac_store(s);
    }

    return (agc_mpac_vshift(ip, c, 1)); /* VSL1 -> scale 2^1 */
}

/**********************************************************************
 * MOONMX (PLANETARY_INERTIAL_ORIENTATION.agc pp. 1246-1248, label
 *   MOONMX) -- moon-fixed rotation matrix on the push-down words
 *     A = (cos N, sin N cos B, sin N sin B)
 *     B = (-sin N, cos N cos B, cos N sin B)
 *     C = (0, -sin B, cos B)
 *     D = B cos I - C sin I
 *     M0 = -(A cos F + D sin F), M1 = A sin F - D cos F,
 *     M2 = B sin I + C cos I;  all vectors and M scale 2^1
 * COS of B and of NODI take NEWANGLE's TP MPAC; the other SIN/COS the
 * stored angle words, as the listing loads them.
 *
 * FUNCTION NAME: moonmx_words
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    TEPHEM
 *   t_cs      agc_dp_t                      I    T, scale 2^28 cs
 *   m         agc_vec_words_t [3]           O    MMATRIX rows as stored,
 *                                                scale 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or a NEWANGLE status
 **********************************************************************/
static agc_status_t moonmx_words(agc_interp_t *ip, const agc_navaux_padload_t *pad, agc_dp_t t_cs, agc_vec_words_t m[3])
{
    agc_tp_words_t bm;                                                                            /* 504B, TP MPAC */
    agc_tp_words_t fm;                                                                            /* 504F, TP MPAC */
    agc_tp_words_t nm;                                                                            /* NODI, TP MPAC */
    agc_status_t   st = newangle_words(ip, pad->tephem, t_cs, BSUBO, BDOT, XDOT_SHIFT_NODE, &bm); /* NEWANGLE status */

    if (st == AGC_STATUS_OK)
    {
        st = newangle_words(ip, pad->tephem, t_cs, FSUBO, FDOT, XDOT_SHIFT_F, &fm);
    }

    if (st == AGC_STATUS_OK)
    {
        st = newangle_words(ip, pad->tephem, t_cs, NODIO, NODDOT, XDOT_SHIFT_NODE, &nm);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_dp_words_t cob  = agc_mpac_store(agc_mpac_cos(bm));                                                          /* PUSH COS: COB, 2^1 */
    const agc_dp_words_t sob  = agc_mpac_store(agc_mpac_sin(agc_mpac_dload(agc_mpac_store(bm))));                          /* SIN: SOB, 2^1 */
    const agc_dp_words_t f    = agc_mpac_store(fm);                                                                        /* 504F, revs 2^0 */
    const agc_tp_words_t cnm  = agc_mpac_cos(nm);                                                                          /* COS of the TP MPAC */
    const agc_dp_words_t cosn = agc_mpac_store(cnm);                                                                       /* COS(NODI), 2^1 */
    const agc_tp_words_t snm  = agc_mpac_sin(agc_mpac_dload(agc_mpac_store(nm)));                                          /* SIN(NODI) */
    const agc_dp_words_t sinn = agc_mpac_store(snm);                                                                       /* SINNODI, 2^1 */
    agc_vec_words_t      av   = {{cosn, dmp_sl1r(ip, sinn, cob), dmp_sl1r(ip, sinn, sob)}};                                /* AVECTR */
    agc_vec_words_t      bv   = {{agc_mpac_store(agc_mpac_dcomp(snm)), dmp_sl1r(ip, cosn, cob), dmp_sl1r(ip, cosn, sob)}}; /* BVECTR */
    agc_vec_words_t      cv   = {{{0, 0}, agc_mpac_store(agc_mpac_dcomp(agc_mpac_dload(sob))), cob}};                      /* CVECTR */
    const agc_tp_words_t sini = agc_mpac_dload(agc_dp_to_words(SINI));                                                     /* SINI, 2^1 */
    const agc_tp_words_t cosi = agc_mpac_dload(agc_dp_to_words(COSI));                                                     /* COSI, 2^1 */

    m[2]                     = combine(ip, bv, sini, cv, cosi, false); /* M2 */
    const agc_vec_words_t dv = combine(ip, bv, cosi, cv, sini, true);  /* DVECTR */

    m[1] = combine(ip, av, agc_mpac_sin(agc_mpac_dload(f)), dv, agc_mpac_cos(agc_mpac_dload(f)), true);  /* M1 */
    m[0] = combine(ip, av, agc_mpac_cos(agc_mpac_dload(f)), dv, agc_mpac_sin(agc_mpac_dload(f)), false); /* M0 before VCOMP */

    for (int i = 0; i < 3; i++)
    {
        m[0].c[i] = agc_mpac_store(agc_mpac_dcomp(agc_mpac_dload(m[0].c[i]))); /* VCOMP */
    }

    return (AGC_STATUS_OK);
} /* moonmx_words */

/**********************************************************************
 * MOONMX (PLANETARY_INERTIAL_ORIENTATION.agc pp. 1246-1248) -- the
 * moon-fixed rotation matrix (moonmx_words), as values
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
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or a NEWANGLE status
 **********************************************************************/
agc_status_t agc_planet_moonmx(agc_interp_t *ip, const agc_navaux_padload_t *pad, agc_dp_t t_cs, agc_mat_t *m)
{
    agc_vec_words_t    w[3];                                /* MMATRIX words */
    const agc_status_t st = moonmx_words(ip, pad, t_cs, w); /* MOONMX status */

    if (st == AGC_STATUS_OK)
    {
        for (int i = 0; i < 3; i++)
        {
            m->row[i] = agc_vec_from_words(w[i]);
        }
    }

    return (st);
}

/**********************************************************************
 * AGC: R-TO-RP / RP-TO-R pp. 1243-1245 (common part) -- MMATRIX and the
 *   libration/precession vector on the words
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
 *   m         agc_vec_words_t [3]           O    MMATRIX words, 2^1
 *   l         agc_vec_words_t *             O    L or LP, scale 2^0 rad
 *
 * RETURNS: agc_status_t -- status of EARTHMX/MOONMX/EARTHL
 **********************************************************************/
static agc_status_t planet_m_and_l(agc_interp_t *ip, const agc_navaux_padload_t *pad, bool moon, agc_dp_t t_cs, bool want_lp, agc_vec_words_t m[3], agc_vec_words_t *l)
{
    agc_status_t st; /* returned to the caller */

    if (moon)
    {
        st = moonmx_words(ip, pad, t_cs, m);
        *l = agc_vec_to_words(pad->lm504); /* LP = 504LM, moon-fixed */

        if (st == AGC_STATUS_OK && !want_lp)
        {
            *l = agc_mpac_vshift(ip, agc_mpac_vxm(ip, *l, m), 1); /* VXM MMATRIX VSL1: L */
        }
    }
    else
    {
        agc_vec_t lv; /* 504LPL, rad 2^0 */

        st = earthmx_words(ip, pad, t_cs, m);

        if (st == AGC_STATUS_OK)
        {
            st = agc_planet_earthl(pad, &lv); /* L, rad 2^0     */
        }

        *l = agc_vec_to_words(lv);

        if (st == AGC_STATUS_OK && want_lp)
        {
            *l = agc_mpac_vshift(ip, agc_mpac_mxv(ip, m, *l), 1); /* MXV VSL1: LP = M L */
        }
    }

    return (st);
}

/**********************************************************************
 * R-TO-RP (PLANETARY_INERTIAL_ORIENTATION.agc p. 1245, label R-TO-RP)
 *   -- RP = M(T) (R - L x R) on the stored words of R
 *
 * FUNCTION NAME: agc_planet_r_to_rp_words
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    pad load
 *   moon      bool                          I    true: moon-fixed frame
 *   r         agc_vec_words_t               I    R as stored (504RPR),
 *                                                any scale 2^n
 *   t_cs      agc_dp_t                      I    T, scale 2^28 cs
 *   rp        agc_vec_words_t *             O    RP as stored, 2^n
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the matrix
 *          routines
 **********************************************************************/
agc_status_t agc_planet_r_to_rp_words(agc_interp_t *ip, const agc_navaux_padload_t *pad, bool moon, agc_vec_words_t r, agc_dp_t t_cs, agc_vec_words_t *rp)
{
    agc_vec_words_t m[3];                                                   /* MMATRIX words, 2^1 */
    agc_vec_words_t l;                                                      /* L, scale 2^0 rad */
    agc_status_t    st = planet_m_and_l(ip, pad, moon, t_cs, false, m, &l); /* returned to the caller */

    if (st == AGC_STATUS_OK)
    {
        agc_vec_words_t d = agc_mpac_vxv(ip, l, r); /* VXV 504RPR */

        for (int i = 0; i < 3; i++)
        {
            d.c[i] = agc_mpac_store(agc_mpac_bdsu(ip, agc_mpac_dload(d.c[i]), r.c[i])); /* BVSU 504RPR */
        }

        *rp = agc_mpac_vshift(ip, agc_mpac_mxv(ip, m, d), 1); /* MXV MMATRIX, RPRPXXXX VSL1 */
    }

    return (st);
}

/**********************************************************************
 * R-TO-RP (PLANETARY_INERTIAL_ORIENTATION.agc p. 1245, label R-TO-RP)
 *   -- RP = M(T) (R - L x R) for a stored value R
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
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the matrix
 *          routines
 **********************************************************************/
agc_status_t agc_planet_r_to_rp(agc_interp_t *ip, const agc_navaux_padload_t *pad, bool moon, agc_vec_t r, agc_dp_t t_cs, agc_vec_t *rp)
{
    agc_vec_words_t    w;                                                                           /* RP words */
    const agc_status_t st = agc_planet_r_to_rp_words(ip, pad, moon, agc_vec_to_words(r), t_cs, &w); /* R-TO-RP status */

    if (st == AGC_STATUS_OK)
    {
        *rp = agc_vec_from_words(w);
    }

    return (st);
}

/**********************************************************************
 * RP-TO-R (PLANETARY_INERTIAL_ORIENTATION.agc p. 1243, label RP-TO-R)
 *   -- R = MT(T) (RP + LP x RP) on the stored words of RP
 *
 * FUNCTION NAME: agc_planet_rp_to_r_words
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    pad load
 *   moon      bool                          I    true: moon-fixed frame
 *   rp        agc_vec_words_t               I    RP as stored (504RPR),
 *                                                any scale 2^n
 *   t_cs      agc_dp_t                      I    T, scale 2^28 cs
 *   r         agc_vec_words_t *             O    R as stored, 2^n
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the matrix
 *          routines
 **********************************************************************/
agc_status_t agc_planet_rp_to_r_words(agc_interp_t *ip, const agc_navaux_padload_t *pad, bool moon, agc_vec_words_t rp, agc_dp_t t_cs, agc_vec_words_t *r)
{
    agc_vec_words_t m[3];                                                   /* MMATRIX words, 2^1 */
    agc_vec_words_t lp;                                                     /* LP, scale 2^0 rad */
    agc_status_t    st = planet_m_and_l(ip, pad, moon, t_cs, true, m, &lp); /* returned to the caller */

    if (st == AGC_STATUS_OK)
    {
        const agc_vec_words_t s = agc_mpac_vad(ip, agc_mpac_vxv(ip, lp, rp), rp); /* VXV VAD 504RPR */

        *r = agc_mpac_vshift(ip, agc_mpac_vxm(ip, s, m), 1); /* VXM MMATRIX, RPRPXXXX VSL1 */
    }

    return (st);
}

/**********************************************************************
 * RP-TO-R (PLANETARY_INERTIAL_ORIENTATION.agc p. 1243, label RP-TO-R)
 *   -- R = MT(T) (RP + LP x RP) for a stored value RP
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
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the matrix
 *          routines
 **********************************************************************/
agc_status_t agc_planet_rp_to_r(agc_interp_t *ip, const agc_navaux_padload_t *pad, bool moon, agc_vec_t rp, agc_dp_t t_cs, agc_vec_t *r)
{
    agc_vec_words_t    w;                                                                            /* R words */
    const agc_status_t st = agc_planet_rp_to_r_words(ip, pad, moon, agc_vec_to_words(rp), t_cs, &w); /* RP-TO-R status */

    if (st == AGC_STATUS_OK)
    {
        *r = agc_vec_from_words(w);
    }

    return (st);
}
