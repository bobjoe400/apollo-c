/*
 * latlong.c -- LATITUDE LONGITUDE SUBROUTINES
 * (Comanche055 LATITUDE_LONGITUDE_SUBROUTINES.agc pp. 1236-1242; the
 * Luminary099 code is identical, its ERAD and 504RM constants live in
 * CONTROLLED_CONSTANTS.agc with the same values).
 *
 * LAT-LONG converts a position vector at a given time to geodetic
 * latitude, longitude and altitude over the earth (pad radius or Fischer
 * ellipsoid) or the moon (504RM or the landing-site radius |RLS|).
 * LALOTORV is the inverse.  Constants computed by MIT with
 * A = 6378166 m, B = 6356784 m (Fischer 1960 ellipsoid).
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* B2XSC 2DEC .0179450689, oct 00446 00305: B^2 scaled 2^51 m^2.          */
static const agc_dp_t B2XSC = 4817093;
/* B2/A2 2DEC .9933064884 B-1, oct 17711 05254: GAMMA = B^2/A^2, scale 2^1. */
static const agc_dp_t B2_A2 = 133319340;
/* EE 2DEC 6.6935116 E-3, oct 00155 25250: 1 - B^2/A^2, scale 2^0.         */
static const agc_dp_t EE = 1796776;
/* ERAD 2DEC 6373338 B-29, oct 00302 17755: pad radius (pad 37-B), 2^29 m. */
static const agc_dp_t ERAD = 3186669;
/* 504RM 2DEC 1738090 B-29, oct 00065 01265: moon radius, scale 2^29 m.    */
static const agc_dp_t RM504 = 869045;

/*
 * SETGAMMA (LATITUDE_LONGITUDE_SUBROUTINES.agc p. 1241, label SETGAMMA)
 * GAMRP = B^2/A^2 for the earth, 1 (1B1 = DP1/2 at scale 2^1) for the moon.
 */
agc_dp_t agc_latlong_setgamma(bool lunar)
{
    return (lunar ? AGC_DP_HALF : B2_A2);
}

/*
 * GETERAD (LATITUDE_LONGITUDE_SUBROUTINES.agc p. 1239, label GETERAD)
 * Fischer ellipsoid radius R = B / sqrt(1 - EE cos^2 L), L geocentric.
 * Input 1/2 sin L (ALPHAV +4); output ERADM scale 2^29 m.
 */
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

    agc_dp_t sin2 =
        agc_dp_shl(ip, agc_dp_sq(ip, half_sin_lat), 1); /* sin^2/2 */
    agc_dp_t cos2 = agc_dp_sub(ip, AGC_DP_HALF, sin2);  /* BDSU DP1/2     */
    agc_dp_t den  = agc_dp_sub(ip, AGC_DP_HALF, agc_dp_mul(ip, cos2, EE));
    agc_dp_t q    = agc_dp_div(ip, B2XSC, den);     /* BDDV B2XSC     */

    return (agc_dp_shr_round(agc_dp_sqrt(ip, q), 4)); /* SQRT, SR4R     */
}

/*
 * SETRE (LATITUDE_LONGITUDE_SUBROUTINES.agc p. 1242, label SETRE)
 * Earth: ERADFLAG = 0 -> ERAD, 1 -> GETERAD.  Moon: ERADFLAG = 1 -> 504RM,
 * 0 -> |RLS| (TSTRLSRM, RLS scale 2^27 m, SR2R to 2^29 m).
 */
agc_status_t agc_latlong_setre(agc_interp_t *ip,
                               const agc_navaux_padload_t *pad,
                               agc_latlong_opt_t opt, agc_dp_t half_sin_lat,
                               agc_dp_t *eradm)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (eradm != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_vec_ok(pad->rls)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (opt.lunar)
    {
        *eradm =
            opt.erad ? RM504 : agc_dp_shr_round(agc_vec_abval(ip, pad->rls), 2);
    }
    else
    {
        *eradm = opt.erad ? agc_latlong_geterad(ip, half_sin_lat) : ERAD;
    }

    return (AGC_STATUS_OK);
}

/*
 * ARCTAN (LATITUDE_LONGITUDE_SUBROUTINES.agc p. 1240, label ARCTAN)
 * THETA = atan2(SINTH, COSTH), inputs scale 2^1, result revs 2^0 in
 * [-1/2, +1/2].  0/0 gives 0; |sin/r| >= 1 (DDV overflow) gives +-1/4.
 */
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

    agc_interp_clear_ovf(ip); /* BOV CLROVFLW   */
    agc_dp_t r2 = agc_dp_add(ip, agc_dp_sq(ip, sinth), agc_dp_sq(ip, costh));

    if (r2 == 0)
    {
        return (0); /* ARCTANXX: 0/0  */
    }

    agc_dp_t q = agc_dp_div(ip, sinth, agc_dp_sqrt(ip, r2)); /* sin, 2^0  */

    if (agc_interp_test_ovf(ip))
    {
        return (agc_dp_sign(AGC_DP_QUARTER, sinth)); /* ATAN=90        */
    }

    agc_dp_t theta = agc_dp_asin(ip, agc_dp_shr(q, 1)); /* SR1 ASIN       */

    if (costh >= 0)
    {
        return (theta); /* BMN NEGCOS not taken */
    }

    agc_dp_t neg = agc_dp_neg(theta); /* NEGCOS: DCOMP  */

    if (neg >= 0)
    {
        return (agc_dp_sub(ip, neg, AGC_DP_HALF)); /* NEGOUT         */
    }

    return (agc_dp_add(ip, neg, AGC_DP_HALF));
}

/*
 * LAT-LONG (LATITUDE_LONGITUDE_SUBROUTINES.agc pp. 1236-1237, label LAT-LONG)
 * Input R (ALPHAV) scale 2^29 m, time scale 2^28 cs.  Output LAT, LONG
 * revs 2^0, ALT = |R| - ERADM scale 2^29 m.  The geodetic latitude uses
 * tan(LAT) = sin(Lc) / (GAMMA cos(Lc)).
 */
agc_status_t agc_latlong_from_r(agc_interp_t *ip,
                                const agc_navaux_padload_t *pad,
                                agc_latlong_opt_t opt, agc_vec_t r,
                                agc_dp_t t_cs, agc_latlong_t *out)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (out != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_vec_ok(r)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t     alpham = agc_vec_abval(ip, r); /* ALPHAM 2^29 m  */
    agc_vec_t    rp;
    agc_status_t st = agc_planet_r_to_rp(ip, pad, opt.lunar, r, t_cs, &rp);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t alphav = agc_vec_unit(ip, rp, NULL, NULL); /* unit RP 2^1    */
    agc_dp_t  gamrp  = agc_latlong_setgamma(opt.lunar);

    st = agc_latlong_setre(ip, pad, opt, alphav.c[2], &out->eradm);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t costh = agc_dp_sqrt(ip, agc_dp_add(ip, agc_dp_sq(ip, alphav.c[0]),
                                                agc_dp_sq(ip, alphav.c[1])));

    costh =
        agc_dp_shl_round(ip, agc_dp_mul(ip, costh, gamrp), 1); /* DMP SL1R */
    out->lat = agc_latlong_arctan(ip, alphav.c[2], costh);
    out->lon = agc_latlong_arctan(ip, alphav.c[1], alphav.c[0]);
    out->alt = agc_dp_sub(ip, alpham, out->eradm);

    return (AGC_STATUS_OK);
}

/*
 * LALOTORV (LATITUDE_LONGITUDE_SUBROUTINES.agc pp. 1238-1239, label LALOTORV)
 * Unit RP = UNIT(cos LONG cos LAT, sin LONG cos LAT, GAMMA sin LAT);
 * R = (ERADM + ALT) * RP-TO-R(unit RP), scale 2^29 m.
 */
agc_status_t agc_latlong_to_r(agc_interp_t *ip, const agc_navaux_padload_t *pad,
                              agc_latlong_opt_t opt, const agc_latlong_t *in,
                              agc_dp_t t_cs, agc_vec_t *r, agc_dp_t *out_eradm)
{
    if (!AGC_ASSERT((ip != NULL) && (pad != NULL) && (in != NULL) &&
                    (r != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(agc_navaux_dp_ok(in->lat) && agc_navaux_dp_ok(in->alt)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t     gamrp = agc_latlong_setgamma(opt.lunar);
    agc_dp_t     clat  = agc_dp_cos(ip, in->lat); /* scale 2^1      */
    agc_dp_t     z     = agc_dp_mul(ip, agc_dp_sin(ip, in->lat), gamrp); /* 2^2 */
    agc_dp_t     y     = agc_dp_mul(ip, agc_dp_sin(ip, in->lon), clat);
    agc_dp_t     x     = agc_dp_mul(ip, agc_dp_cos(ip, in->lon), clat);
    agc_vec_t    urp   = agc_vec_unit(ip, agc_vec_make(x, y, z), NULL, NULL);
    agc_dp_t     eradm = 0;
    agc_status_t st    = agc_latlong_setre(ip, pad, opt, urp.c[2], &eradm);
    agc_vec_t    ur    = agc_vec_zero();

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_rp_to_r(ip, pad, opt.lunar, urp, t_cs, &ur);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t rad = agc_dp_add(ip, eradm, in->alt);       /* RE + ALT 2^29  */

    *r = agc_vec_shl(ip, agc_vec_scale(ip, ur, rad), 1); /* VXSC VSL1      */

    if (out_eradm != NULL)
    {
        *out_eradm = eradm;
    }

    return (AGC_STATUS_OK);
}
