/**********************************************************************
 * FILE NAME: latlong.c
 *
 * PURPOSE:
 *   LATITUDE LONGITUDE SUBROUTINES.  LAT-LONG converts a position vector
 *   at a given time to geodetic latitude, longitude and altitude over the
 *   earth (pad radius or Fischer ellipsoid) or the moon (504RM or the
 *   landing-site radius |RLS|); LALOTORV is the inverse.  Also SETGAMMA,
 *   GETERAD, SETRE, ARCTAN and the LAT/LONG/ALT erasable store.
 *
 * AGC SOURCE:
 *   Comanche055 LATITUDE_LONGITUDE_SUBROUTINES.agc pp. 1236-1242
 *     (LAT-LONG, LALOTORV, GETERAD, ARCTAN, SETGAMMA, SETRE)
 *   Luminary099 LATITUDE_LONGITUDE_SUBROUTINES.agc (identical; its ERAD
 *     and 504RM live in CONTROLLED_CONSTANTS.agc with the same values)
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type           I/O  Scale     Description
 *   ------------  -------------  ---  --------  ----------------------
 *   LAT,LONG,ALT  agc_latlong_t  I/O  2^0 rev,  noun 43 store, module
 *                                     2^29 m    static g_latlong
 *   RLS           agc_vec_t      I    2^27 m    via the pad argument
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_planet_r_to_rp    R-TO-RP: reference to planet-fixed frame
 *   agc_planet_rp_to_r    RP-TO-R: planet-fixed to reference frame
 *   agc_dp_asin, agc_tp_sqrt, agc_vec_unit  interpretive ops (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms.  AGC_STATUS_BAD_ARG on NULL or out-of-range inputs; the
 *   status of R-TO-RP / RP-TO-R is passed through.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called in job context by the programs (P21, P22, P23, P68, LLASRD).
 *   Constants computed by MIT with A = 6378166 m, B = 6356784 m (Fischer
 *   1960 ellipsoid).
 *
 * NOTES:
 *   ARCTAN's SR1 ASIN takes a stale MPAC+2 into ARCSIN's SQRTSUB; the C
 *   ASIN takes a DP, so earth LAT can differ by 1 LSB (see
 *   docs/modules/review-nav-aux.md finding 5).
 *
 * ALGORITHM:
 *   LAT-LONG: RP = R-TO-RP(R), unit RP; LAT = atan(z / (GAMMA |xy|)),
 *   LONG = atan(y / x), ALT = |R| - ERADM.  LALOTORV reverses the steps.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* B2XSC 2DEC .0179450689, oct 00446 00305: B^2, scale 2^51 m^2.          */
static const agc_dp_t B2XSC = 4817093; /* B^2, 2^51 m^2       */

/* B2/A2 2DEC .9933064884 B-1, oct 17711 05254: GAMMA = B^2/A^2, 2^1.     */
static const agc_dp_t B2_A2 = 133319340; /* GAMMA, scale 2^1    */

/* EE 2DEC 6.6935116 E-3, oct 00155 25250: 1 - B^2/A^2, scale 2^0.        */
static const agc_dp_t EE = 1796776; /* e^2, scale 2^0      */

/* ERAD 2DEC 6373338 B-29, oct 00302 17755: pad 37-B radius, 2^29 m.      */
static const agc_dp_t ERAD = 3186669; /* pad radius, 2^29 m  */

/* 504RM 2DEC 1738090 B-29, oct 00065 01265: moon radius, scale 2^29 m.   */
static const agc_dp_t RM504 = 869045; /* moon radius, 2^29 m */

/* LAT, LONG, ALT (both ropes, noun 43; the CM's LANDLAT/LANDLONG/LANDALT
 * EQUALS them): the erasables LAT-LONG writes and LALOTORV reads, kept
 * here for the programs that hand them to the display (P21, P22, P23,
 * LLASRD, P68). ERADM is not part of the store.  Scale 2^0 rev, 2^29 m. */
static agc_latlong_t g_latlong; /* LAT, LONG, ALT store */

/**********************************************************************
 * AGC: LAT-LONG / ARCTAN p. 1240 (DLOAD DSQ / PDDL DSQ / DAD)
 *   -- a^2 + b^2 as the AGC forms it before SQRT
 *
 * FUNCTION NAME: sum_of_squares
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   a         agc_dp_t        I    first term, scale 2^s
 *   b         agc_dp_t        I    second term, scale 2^s
 *
 * RETURNS: agc_tp_t -- a^2 + b^2, scale 2^2s.  The first square is pushed
 *          as DP (top two words), the second stays a TP in MPAC and DAD
 *          adds the DP to its top words, so SQRT sees MPAC+2 of b^2 only.
 **********************************************************************/
static agc_tp_t sum_of_squares(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(a) && agc_navaux_dp_ok(b)))
    {
        return (0);
    }

    agc_tp_t a2 = agc_tp_from_dp(agc_dp_sq(ip, a)); /* pushed DP, 2^2s */

    return (agc_tp_add(ip, agc_dp_sq_tp(b), a2));
}

/**********************************************************************
 * SETGAMMA (LATITUDE_LONGITUDE_SUBROUTINES.agc p. 1241, label SETGAMMA)
 *   -- flattening ratio for the earth or the moon
 *
 * FUNCTION NAME: agc_latlong_setgamma
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------
 *   lunar     bool  I    LUNAFLAG: true = moon, false = earth
 *
 * RETURNS: agc_dp_t -- GAMRP, scale 2^1: B^2/A^2 for the earth, 1 (1B1 =
 *          DP1/2) for the moon
 **********************************************************************/
agc_dp_t agc_latlong_setgamma(bool lunar)
{
    agc_dp_t gamrp = B2_A2; /* GAMRP, scale 2^1  */

    if (lunar)
    {
        gamrp = AGC_DP_HALF; /* 1B1: sphere */
    }

    return (gamrp);
}

/**********************************************************************
 * GETERAD (LATITUDE_LONGITUDE_SUBROUTINES.agc p. 1239, label GETERAD)
 *   -- Fischer ellipsoid radius R = B / sqrt(1 - EE cos^2 L)
 *
 * FUNCTION NAME: agc_latlong_geterad
 *
 * ARGUMENTS:
 *   Argument      Type            I/O  Description (scale)
 *   ------------  --------------  ---  ------------------------------
 *   ip            agc_interp_t *  I/O  interpreter state (overflow)
 *   half_sin_lat  agc_dp_t        I    1/2 sin L, L geocentric
 *                                      (ALPHAV +4), scale 2^1
 *
 * RETURNS: agc_dp_t -- ERADM, scale 2^29 m (ERAD on bad input)
 **********************************************************************/
agc_dp_t agc_latlong_geterad(agc_interp_t *ip, agc_dp_t half_sin_lat)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (ERAD);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(half_sin_lat)))
    {
        return (ERAD);
    }

    /* DSQ leaves the TP square in MPAC; SL1 shifts all three words and
     * BDSU then uses the top two (truncation after the shift).            */
    agc_dp_t sin2 = agc_tp_to_dp(agc_tp_shift(ip, agc_dp_sq_tp(half_sin_lat), 1)); /* sin^2/2, 2^0 */
    agc_dp_t cos2 = agc_dp_sub(ip, AGC_DP_HALF, sin2);                             /* BDSU, 2^0  */
    agc_dp_t den  = agc_dp_sub(ip, AGC_DP_HALF, agc_dp_mul(ip, cos2, EE));         /* scale 2^0 */
    agc_dp_t q    = agc_dp_div(ip, B2XSC, den);                                    /* BDDV, 2^51 m^2 */

    return (agc_dp_shr_round(agc_dp_sqrt(ip, q), 4)); /* SQRT, SR4R     */
}

/**********************************************************************
 * SETRE (LATITUDE_LONGITUDE_SUBROUTINES.agc p. 1242, label SETRE)
 *   -- choose the planet radius ERADM
 *
 * FUNCTION NAME: agc_latlong_setre
 *
 * ARGUMENTS:
 *   Argument      Type                          I/O  Description (scale)
 *   ------------  ----------------------------  ---  ------------------
 *   ip            agc_interp_t *                I/O  interpreter state
 *   pad           const agc_navaux_padload_t *  I    pad load (RLS 2^27 m)
 *   opt           agc_latlong_opt_t             I    LUNAFLAG, ERADFLAG
 *   half_sin_lat  agc_dp_t                      I    1/2 sin L, 2^1
 *   eradm         agc_dp_t *                    O    ERADM, 2^29 m
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL or
 *          out-of-range RLS.  Earth: ERADFLAG = 0 -> ERAD, 1 -> GETERAD.
 *          Moon: ERADFLAG = 1 -> 504RM, 0 -> |RLS| (TSTRLSRM, SR2R).
 **********************************************************************/
agc_status_t agc_latlong_setre(agc_interp_t *ip, const agc_navaux_padload_t *pad, agc_latlong_opt_t opt, agc_dp_t half_sin_lat, agc_dp_t *eradm)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (eradm != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_vec_ok(pad->rls)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (opt.lunar && opt.erad)
    {
        *eradm = RM504;
    }
    else if (opt.lunar)
    {
        *eradm = agc_dp_shr_round(agc_vec_abval(ip, pad->rls), 2); /* |RLS| */
    }
    else if (opt.erad)
    {
        *eradm = agc_latlong_geterad(ip, half_sin_lat);
    }
    else
    {
        *eradm = ERAD;
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ARCTAN (LATITUDE_LONGITUDE_SUBROUTINES.agc p. 1240, label ARCTAN)
 *   -- THETA = atan2(SINTH, COSTH)
 *
 * FUNCTION NAME: agc_latlong_arctan
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (overflow)
 *   sinth     agc_dp_t        I    SINTH, scale 2^1
 *   costh     agc_dp_t        I    COSTH, scale 2^1
 *
 * RETURNS: agc_dp_t -- THETA, revs scale 2^0 in [-1/2, +1/2]; 0/0 gives
 *          0; |sin/r| >= 1 (DDV overflow) gives +-1/4
 **********************************************************************/
agc_dp_t agc_latlong_arctan(agc_interp_t *ip, agc_dp_t sinth, agc_dp_t costh)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(sinth) && agc_navaux_dp_ok(costh)))
    {
        return (0);
    }

    agc_interp_clear_ovf(ip);                       /* BOV CLROVFLW   */
    agc_tp_t r2 = sum_of_squares(ip, sinth, costh); /* r^2, scale 2^2 */

    if (agc_tp_to_dp(r2) == 0)
    {
        return (0); /* ARCTANXX: 0/0  */
    }

    agc_dp_t q = agc_dp_div(ip, sinth, agc_tp_sqrt(ip, r2)); /* sin, 2^0  */

    if (agc_interp_test_ovf(ip))
    {
        return (agc_dp_sign(AGC_DP_QUARTER, sinth)); /* ATAN=90        */
    }

    agc_dp_t theta = agc_dp_asin(ip, agc_dp_shr(q, 1)); /* SR1 ASIN, 2^0 rev */

    if (costh >= 0)
    {
        return (theta); /* BMN NEGCOS not taken */
    }

    agc_dp_t neg = agc_dp_neg(theta); /* NEGCOS: DCOMP, 2^0 rev */

    if (neg >= 0)
    {
        return (agc_dp_sub(ip, neg, AGC_DP_HALF)); /* NEGOUT         */
    }

    return (agc_dp_add(ip, neg, AGC_DP_HALF));
} /* agc_latlong_arctan */

/**********************************************************************
 * LAT-LONG (LATITUDE_LONGITUDE_SUBROUTINES.agc pp. 1236-1237, label
 *   LAT-LONG) -- position vector to latitude, longitude, altitude
 *
 * FUNCTION NAME: agc_latlong_from_r
 *
 * ARGUMENTS:
 *   Argument  Type                          I/O  Description (scale)
 *   --------  ----------------------------  ---  ----------------------
 *   ip        agc_interp_t *                I/O  interpreter state
 *   pad       const agc_navaux_padload_t *  I    pad load
 *   opt       agc_latlong_opt_t             I    LUNAFLAG, ERADFLAG
 *   r         agc_vec_t                     I    R (ALPHAV), 2^29 m
 *   t_cs      agc_dp_t                      I    time, scale 2^28 cs
 *   out       agc_latlong_t *               O    LAT, LONG 2^0 rev;
 *                                                ALT, ERADM 2^29 m
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG, or the
 *          status of R-TO-RP / SETRE.  The geodetic latitude uses
 *          tan(LAT) = sin(Lc) / (GAMMA cos(Lc)).
 **********************************************************************/
agc_status_t agc_latlong_from_r(agc_interp_t *ip, const agc_navaux_padload_t *pad, agc_latlong_opt_t opt, agc_vec_t r, agc_dp_t t_cs, agc_latlong_t *out)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (out != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_vec_ok(r)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t     alpham = agc_vec_abval(ip, r);                                 /* ALPHAM 2^29 m  */
    agc_vec_t    rp;                                                            /* RP, 2^29 m     */
    agc_status_t st     = agc_planet_r_to_rp(ip, pad, opt.lunar, r, t_cs, &rp); /* returned */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t alphav = agc_vec_unit(ip, rp, NULL, NULL); /* unit RP 2^1    */
    agc_dp_t  gamrp  = agc_latlong_setgamma(opt.lunar);  /* GAMRP, 2^1 */

    st = agc_latlong_setre(ip, pad, opt, alphav.c[2], &out->eradm);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t costh = agc_tp_sqrt(ip, sum_of_squares(ip, alphav.c[0], alphav.c[1])); /* cos Lc, scale 2^1 */

    /* DMP SL1R: shift the TP product, then round once.                     */
    costh    = agc_tp_round(ip, agc_tp_shift(ip, agc_dp_mul_tp(costh, gamrp), 1));
    out->lat = agc_latlong_arctan(ip, alphav.c[2], costh);
    out->lon = agc_latlong_arctan(ip, alphav.c[1], alphav.c[0]);
    out->alt = agc_dp_sub(ip, alpham, out->eradm);

    return (AGC_STATUS_OK);
} /* agc_latlong_from_r */

/**********************************************************************
 * LALOTORV (LATITUDE_LONGITUDE_SUBROUTINES.agc pp. 1238-1239, label
 *   LALOTORV) -- latitude, longitude, altitude to a position vector
 *
 * FUNCTION NAME: agc_latlong_to_r
 *
 * ARGUMENTS:
 *   Argument   Type                          I/O  Description (scale)
 *   ---------  ----------------------------  ---  ---------------------
 *   ip         agc_interp_t *                I/O  interpreter state
 *   pad        const agc_navaux_padload_t *  I    pad load
 *   opt        agc_latlong_opt_t             I    LUNAFLAG, ERADFLAG
 *   in         const agc_latlong_t *         I    LAT, LONG 2^0 rev,
 *                                                 ALT 2^29 m
 *   t_cs       agc_dp_t                      I    time, scale 2^28 cs
 *   r          agc_vec_t *                   O    R, scale 2^29 m
 *   out_eradm  agc_dp_t *                    O    ERADM 2^29 m, or NULL
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG, or the
 *          status of SETRE / RP-TO-R.  Unit RP = UNIT(cos LONG cos LAT,
 *          sin LONG cos LAT, GAMMA sin LAT); R = (ERADM + ALT) * RP-TO-R.
 **********************************************************************/
agc_status_t agc_latlong_to_r(agc_interp_t *ip, const agc_navaux_padload_t *pad, agc_latlong_opt_t opt, const agc_latlong_t *in, agc_dp_t t_cs, agc_vec_t *r, agc_dp_t *out_eradm)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (in != NULL) && (r != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(in->lat) && agc_navaux_dp_ok(in->alt)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t     gamrp = agc_latlong_setgamma(opt.lunar); /* GAMRP 2^1 */
    agc_dp_t     clat  = agc_dp_cos(ip, in->lat);         /* scale 2^1 */
    agc_dp_t     slat  = agc_dp_sin(ip, in->lat);         /* scale 2^1 */
    agc_dp_t     z     = agc_dp_mul(ip, slat, gamrp);     /* scale 2^2 */
    agc_dp_t     y     = agc_dp_sin(ip, in->lon);         /* RP y, 2^2 after DMP */
    agc_dp_t     x     = agc_dp_cos(ip, in->lon);         /* RP x, 2^2 after DMP */
    agc_vec_t    urp   = agc_vec_zero();                  /* unit RP 2^1 */
    agc_dp_t     eradm = 0;                               /* 2^29 m    */
    agc_status_t st    = AGC_STATUS_OK;                   /* returned  */
    agc_vec_t    ur    = agc_vec_zero();                  /* 2^1       */

    y   = agc_dp_mul(ip, y, clat);
    x   = agc_dp_mul(ip, x, clat);
    urp = agc_vec_make(x, y, z);
    urp = agc_vec_unit(ip, urp, NULL, NULL); /* UNIT, scale 2^1 */
    st  = agc_latlong_setre(ip, pad, opt, urp.c[2], &eradm);

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(ip, pad, opt.lunar, urp, t_cs, &ur);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t rad = agc_dp_add(ip, eradm, in->alt); /* RE + ALT 2^29 m */

    *r = agc_vec_shl(ip, agc_vec_scale(ip, ur, rad), 1); /* VXSC VSL1      */

    if (out_eradm != NULL)
    {
        *out_eradm = eradm;
    }

    return (AGC_STATUS_OK);
} /* agc_latlong_to_r */

/**********************************************************************
 * AGC: none (store for LAT, LONG, ALT, noun 43) -- set the erasables
 *
 * FUNCTION NAME: agc_navaux_set_latlong
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ll        const agc_latlong_t *  I    LAT, LONG 2^0 rev, ALT 2^29 m
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL or
 *          out-of-range values
 **********************************************************************/
agc_status_t agc_navaux_set_latlong(const agc_latlong_t *ll)
{
    if (!AGC_ASSERT(ll != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(ll->lat) && agc_navaux_dp_ok(ll->lon) && agc_navaux_dp_ok(ll->alt)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    g_latlong.lat = ll->lat;
    g_latlong.lon = ll->lon;
    g_latlong.alt = ll->alt;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (store for LAT, LONG, ALT, noun 43) -- read the erasables
 *
 * FUNCTION NAME: agc_navaux_get_latlong
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ---------------------------------
 *   ll        agc_latlong_t *  O    LAT, LONG 2^0 rev, ALT 2^29 m
 *                                   (eradm copied as stored, 0)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on NULL
 **********************************************************************/
agc_status_t agc_navaux_get_latlong(agc_latlong_t *ll)
{
    if (!AGC_ASSERT(ll != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(g_latlong.lat)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *ll = g_latlong;

    return (AGC_STATUS_OK);
}
