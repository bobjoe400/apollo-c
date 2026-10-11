/*
 * agc_navaux.h -- auxiliary navigation subroutines (agent nav-aux).
 *
 * Translated from (Comanche055 page numbers; Luminary099 code is identical
 * unless noted):
 *   LATITUDE_LONGITUDE_SUBROUTINES.agc        pp. 1236-1242  src/nav/auxiliary/latlong.c
 *   PLANETARY_INERTIAL_ORIENTATION.agc        pp. 1243-1251  src/nav/auxiliary/planetary.c
 *   MEASUREMENT_INCORPORATION.agc             pp. 1252-1261  src/nav/auxiliary/measurement.c
 *   TIME_OF_FREE_FALL.agc                     pp. 1373-1388  src/nav/auxiliary/free_fall.c
 *   LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES   CM pp. 785-788, LM pp. 984-987
 *                                                            src/nav/auxiliary/ephemerides.c
 *   STAR_TABLES.agc (LM: CONTROLLED_CONSTANTS) pp. 1389-1393 src/nav/auxiliary/star_tables.c
 *
 * Scale notation: "scale 2^N u" means value = fraction * 2^N in unit u
 * (the AGC comment "B-N").  cs = centiseconds, revs = revolutions.
 * Angles in revolutions use scale 2^0 (0.5 = pi rad).
 */
#ifndef AGC_NAVAUX_H
#define AGC_NAVAUX_H

#include <stdbool.h>

#include "agc/core/interp.h"
#include "agc/core/status.h"
#include "agc/core/word.h"

/* ======================================================================
 * Launch-time pad load used by planetary orientation, lat/long and the
 * ephemerides.  agc_navaux_set_padload stores TEPHEM and RLS in the core
 * erasables (agc_erase_tephem / agc_erase_rls, their only store) and AZO,
 * AXO, -AYO, 504LM in planetary.c; agc_navaux_get_padload reads them back
 * the same way.  Functions take the pad explicitly; agc_lsposition, whose
 * signature is fixed, reads agc_erase_tephem.
 * ==================================================================== */
typedef struct agc_navaux_padload
{
    agc_tp_t tephem;  /* TEPHEM (= TIMSUBO): cs from midnight 1 July
                           preceding launch to AGC clock zero, TP scale 2^42 cs */
    agc_dp_t  azo;    /* AZO: earth rotation angle at TEPHEM, scale 2^0 revs  */
    agc_dp_t  axo;    /* AXO: earth precession angle about X, scale 2^0 rad   */
    agc_dp_t  neg_ayo; /* -AYO: minus the angle about Y, scale 2^0 rad         */
    agc_vec_t lm504;  /* 504LM: lunar libration vector, moon-fixed, 2^0 rad   */
    agc_vec_t rls;    /* RLS: landing site, moon-fixed, scale 2^27 m          */
} agc_navaux_padload_t;

agc_status_t agc_navaux_set_padload(const agc_navaux_padload_t *pad);
agc_status_t agc_navaux_get_padload(agc_navaux_padload_t *out);

/* ======================================================================
 * Planetary inertial orientation (planetary.c)
 * Time arguments t_cs: centiseconds since AGC clock zero, scale 2^28 cs.
 * M matrices are rotation matrices scaled 2^1 (half-unit rows).
 * ==================================================================== */
/* NEWANGLE: X = X0 + XDOT (T + TIMSUBO) modulo one revolution.
 * xdot scale 2^-(23 + shift) rev/cs, where xdot_shift is the AGC X1:
 * 0 for WEARTH (2^-23), 5 for NODDOT/BDOT (2^-28), 4 for FDOT (2^-27).    */
agc_status_t agc_planet_newangle(agc_interp_t *ip, agc_tp_t timsubo,
                                 agc_dp_t t_cs, agc_dp_t x0, agc_dp_t xdot,
                                 int xdot_shift, agc_dp_t *x);
agc_status_t agc_planet_earthmx(agc_interp_t *ip,
                                const agc_navaux_padload_t *pad, agc_dp_t t_cs,
                                agc_mat_t *m); /* EARTHMX */
agc_status_t agc_planet_moonmx(agc_interp_t *ip,
                               const agc_navaux_padload_t *pad, agc_dp_t t_cs,
                               agc_mat_t *m); /* MOONMX  */
agc_status_t agc_planet_earthl(const agc_navaux_padload_t *pad,
                               agc_vec_t                  *l); /* EARTHL  */
/* R-TO-RP: RP = M(T) (R - L x R); RP-TO-R: R = MT(T) (RP + LP x RP).
 * Scale preserving: 2^29 m (earth) / 2^27 m (moon) in the AGC callers, or
 * a half-unit vector.  moon selects MOONMX/504LM versus EARTHMX/EARTHL.   */
agc_status_t agc_planet_r_to_rp(agc_interp_t *ip,
                                const agc_navaux_padload_t *pad, bool moon,
                                agc_vec_t r, agc_dp_t t_cs, agc_vec_t *rp);
agc_status_t agc_planet_rp_to_r(agc_interp_t *ip,
                                const agc_navaux_padload_t *pad, bool moon,
                                agc_vec_t rp, agc_dp_t t_cs, agc_vec_t *r);

/* ======================================================================
 * Latitude / longitude (latlong.c)
 * ==================================================================== */
typedef struct agc_latlong_opt
{
    bool lunar; /* LUNAFLAG: true = moon, false = earth                   */
    bool erad;  /* ERADFLAG: earth: true = Fischer ellipsoid (GETERAD),
                      false = fixed pad radius ERAD; moon: true = 504RM,
                      false = |RLS|                                          */
} agc_latlong_opt_t;

typedef struct agc_latlong
{
    agc_dp_t lat;   /* LAT, geodetic latitude, scale 2^0 revs (+-1/4)       */
    agc_dp_t lon;   /* LONG, longitude, scale 2^0 revs (-1/2 .. +1/2)       */
    agc_dp_t alt;   /* ALT, altitude above ERADM, scale 2^29 m              */
    agc_dp_t eradm; /* ERADM, planet radius used, scale 2^29 m              */
} agc_latlong_t;

/* LAT, LONG, ALT erasables (noun 43; CM LANDLAT/LANDLONG/LANDALT): set by
 * the programs after LAT-LONG, read for LALOTORV. eradm is not stored.    */
agc_status_t agc_navaux_set_latlong(const agc_latlong_t *ll);
agc_status_t agc_navaux_get_latlong(agc_latlong_t *ll);

/* LAT-LONG: r scale 2^29 m (earth or moon centred).                       */
agc_status_t agc_latlong_from_r(agc_interp_t *ip,
                                const agc_navaux_padload_t *pad,
                                agc_latlong_opt_t opt, agc_vec_t r,
                                agc_dp_t t_cs, agc_latlong_t *out);
/* LALOTORV: in->lat/lon/alt used; returns r scale 2^29 m and in->eradm
 * is not read (out_eradm, if non-NULL, receives ERADM).                   */
agc_status_t agc_latlong_to_r(agc_interp_t *ip, const agc_navaux_padload_t *pad,
                              agc_latlong_opt_t opt, const agc_latlong_t *in,
                              agc_dp_t t_cs, agc_vec_t *r, agc_dp_t *out_eradm);
/* GETERAD: half_sin_lat = sin(geocentric lat)/2; returns radius 2^29 m.    */
agc_dp_t agc_latlong_geterad(agc_interp_t *ip, agc_dp_t half_sin_lat);
/* ARCTAN: sinth, costh scale 2^1; returns theta revs 2^0 in [-1/2, 1/2].  */
agc_dp_t agc_latlong_arctan(agc_interp_t *ip, agc_dp_t sinth, agc_dp_t costh);
/* SETGAMMA: b^2/a^2 (earth) or 1 (moon), scale 2^1.                       */
agc_dp_t agc_latlong_setgamma(bool lunar);
/* SETRE: radius ERADM scale 2^29 m.                                        */
agc_status_t agc_latlong_setre(agc_interp_t *ip,
                               const agc_navaux_padload_t *pad,
                               agc_latlong_opt_t opt, agc_dp_t half_sin_lat,
                               agc_dp_t *eradm);

/* ======================================================================
 * Measurement incorporation (measurement.c)
 * ==================================================================== */
#define AGC_INCORP_ROWS 9 /* 9x9 W matrix (6x6 when DMENFLG is clear)    */
#define AGC_INCORP_BLKS 3 /* column triples: position, velocity, bias    */

/* W, the error transition matrix.  blk[b][m] = W(m, 3b..3b+2), the 3-vector
 * stored at AGC address W + 54b + 6m.  Scaled as kept by the W-matrix
 * integration (nav-integ); INCORP1/INCORP2 do not depend on the scale.    */
typedef struct agc_incorp_w
{
    agc_vec_t blk[AGC_INCORP_BLKS][AGC_INCORP_ROWS];
} agc_incorp_w_t;

typedef struct agc_incorp
{
    /* inputs to INCORP1 */
    bool      dim9;                     /* DMENFLG: true = 9-dim      */
    agc_vec_t bvector[AGC_INCORP_BLKS]; /* BVECTOR (6 or 9 dim)       */
    agc_dp_t  deltaq;                   /* DELTAQ measured deviation  */
    agc_tp_t  variance;                 /* VARIANCE, TP               */
    /* outputs of INCORP1 (inputs of INCORP2) */
    agc_vec_t zi[AGC_INCORP_BLKS];     /* ZI, normalized by NEWZCOMP */
    agc_vec_t omega[AGC_INCORP_BLKS];  /* OMEGA weighting vector     */
    agc_vec_t deltax[AGC_INCORP_BLKS]; /* DELTAX state deviations:
                                               [0] pos 2^29/2^27 m,
                                               [1] vel 2^7/2^5 m/cs,
                                               [2] bias                   */
    agc_dp_t gamma;                   /* GAMMA (normalized)         */
    int      normgam;                 /* NORMGAM, AGC index value   */
} agc_incorp_t;

/* Integration temporaries touched by INCORP2 phase 3 (FAZC..FAZAB5). */
typedef struct agc_incorp_sv
{
    bool      moon;    /* MOONTHIS: primary body of the updated vehicle  */
    agc_vec_t rcv;     /* RCV  conic position, 2^29 / 2^27 m              */
    agc_vec_t vcv;     /* VCV  conic velocity, 2^7 / 2^5 m/cs             */
    agc_vec_t tdeltav; /* TDELTAV position deviation, 2^22 / 2^18 m       */
    agc_vec_t tnuv;    /* TNUV velocity deviation, 2^3 / 2^-1 m/cs        */
    agc_vec_t x789;    /* X789 bias states (radar/landmark)               */
    bool      rectify; /* out: caller must CALL RECTIFY (PBODY = moon)    */
} agc_incorp_sv_t;

agc_status_t agc_incorp_state_deviation(agc_interp_t *ip, const agc_incorp_w_t *w,
                         agc_incorp_t *inc); /* INCORP1 */
agc_status_t agc_incorp_update_w(agc_interp_t *ip, const agc_incorp_t *inc,
                                 agc_incorp_w_t *w); /* INCORP2 FAZA-B */
agc_status_t agc_incorp_update_state(agc_interp_t *ip, const agc_incorp_t *inc,
                               agc_incorp_sv_t *sv); /* INCORP2 FAZC-  */

/* ======================================================================
 * Time of free fall (free_fall.c).  E = earth scale, M = moon scale.
 * The push-list quantities of TFFCONIC live in this struct; nr/na are the
 * positive normalization counts (the AGC keeps -NR in X1, -NA in X2).
 * ==================================================================== */
typedef struct agc_tff
{
    /* inputs */
    agc_vec_t rone; /* RONE  E 2^29 m,  M 2^27 m                       */
    agc_vec_t vone; /* VONE  E 2^7 m/cs, M 2^5 m/cs                    */
    agc_dp_t  rtmu; /* TFF/RTMU 1/sqrt(mu) E 2^-17, M 2^-14            */
    /* TFFCONIC outputs */
    agc_vec_t vone_p;   /* VONE' = VN/sqrt(mu)  E 2^-10, M 2^-9            */
    agc_dp_t  rmag1;    /* RMAG1 |RN|            E 2^29, M 2^27 m          */
    agc_dp_t  nrmag;    /* NRMAG |RN| normalized E 2^(29-NR) m             */
    int       nr;       /* NR (AGC X1 = -NR)                               */
    agc_dp_t  tffnp;    /* TFFNP semi latus rectum E 2^(38-2NR), M 2^(36-2NR) */
    agc_dp_t  tffvsq;   /* TFFVSQ -(v^2/mu) 1/m E 2^-20, M 2^-18           */
    agc_dp_t  tffalfa;  /* TFFALFA alfa 1/m E 2^-(26-NR), M 2^-(24-NR)     */
    agc_dp_t  tffrtalf; /* TFFRTALF sqrt|alfa| E 2^-(10+NA), M 2^-(9+NA)   */
    int       na;       /* NA (AGC X2 = -NA)                               */
    agc_dp_t  tff1_alf; /* TFF1/ALF 1/alfa m E 2^(22+2NA), M 2^(20+2NA)    */
    /* TFFRP/RA outputs */
    agc_dp_t rper; /* RPER perigee radius E 2^29, M 2^27 m            */
    agc_dp_t rapo; /* RAPO apogee radius (NEARONE if invalid)         */
    /* CALCTFF/CALCTPER debris left for the entry displays */
    agc_dp_t rterm;   /* RTERM  E 2^29, M 2^27 m                         */
    agc_dp_t nrterm;  /* NRTERM E 2^(29-NR) m                            */
    agc_dp_t tffq1;   /* TFFQ1 R.V/sqrt(mu) E 2^16, M 2^15               */
    agc_dp_t tffdelq; /* TFFDELQ Q2-Q1 E 2^16, M 2^15                    */
    agc_dp_t tffx;    /* TFFX  X (2^0)                                   */
    agc_dp_t tfftem;  /* TFFTEM P Z^2 sgn or P/alfa sgn E 2^(59-2NR)     */
} agc_tff_t;

agc_status_t agc_tff_conic(agc_interp_t *ip, agc_tff_t *tff); /* TFFCONIC */
agc_status_t agc_tff_rp_ra(agc_interp_t *ip, agc_tff_t *tff); /* TFFRP/RA */
/* Returned time: cs scale 2^28; NEARONE (AGC_TFF_NEARONE) = no free fall. */
agc_status_t agc_tff_calctff(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t rterm,
                             agc_dp_t *t_cs); /* CALCTFF  */
agc_status_t agc_tff_calctper(agc_interp_t *ip, agc_tff_t *tff, agc_dp_t rper,
                              agc_dp_t *t_cs);           /* CALCTPER */
agc_dp_t agc_tff_poly(agc_interp_t *ip, agc_dp_t x); /* T(X)     */

#define AGC_TFF_NEARONE \
    ((agc_dp_t) 268435455) /* NEARONE 2DEC .999999999,
                                                  oct 37777 37777          */

/* ======================================================================
 * Lunar and solar ephemerides (ephemerides.c)
 * ==================================================================== */
#define AGC_EPHEM_NCOEF 10 /* ninth-degree lunar polynomial (CM)          */

typedef struct agc_ephem_csm
{
    agc_tp_t  timemo; /* TIMEMO centre of validity, TP 2^42 cs */
    agc_vec_t vecoem[AGC_EPHEM_NCOEF]; /* VECOEM, descending: [0] = a9 ..
                                          [9] = a0; a_n m/cs^n at scale
                                          2^(31 - 26n) so that the Horner
                                          sum with t at 2^26 cs is 2^31 m  */
    agc_vec_t reso;    /* RESO sun position at TIMEMO, 2^38 m  */
    agc_vec_t veso;    /* VESO sun velocity at TIMEMO, 2^9 m/cs*/
    agc_dp_t  omegaes; /* OMEGAES rate of RESO, 2^-26 rev/cs   */
} agc_ephem_csm_t;

agc_status_t agc_ephem_set_csm_data(const agc_ephem_csm_t *eph);
agc_status_t agc_ephem_get_csm_data(agc_ephem_csm_t *out);

/* CM: moon position earth-centred 2^29 m; moon velocity 2^7 m/cs; sun
 * position earth-centred 2^38 m.                                          */
agc_status_t agc_ephem_csm_lunpos(agc_interp_t *ip, const agc_ephem_csm_t *eph,
                                  agc_tp_t tephem, agc_dp_t t_cs,
                                  agc_vec_t *moon);
agc_status_t agc_ephem_csm_lunvel(agc_interp_t *ip, const agc_ephem_csm_t *eph,
                                  agc_tp_t tephem, agc_dp_t t_cs,
                                  agc_vec_t *vmoon);
agc_status_t agc_ephem_csm_solpos(agc_interp_t *ip, const agc_ephem_csm_t *eph,
                                  agc_tp_t tephem, agc_dp_t t_cs,
                                  agc_vec_t *sun);
agc_status_t agc_ephem_csm_lspos(agc_interp_t *ip, const agc_ephem_csm_t *eph,
                                 agc_tp_t tephem, agc_dp_t t_cs,
                                 agc_vec_t *moon, agc_vec_t *sun);
/* LM: unit vectors (scale 2^1) VMOON and VSUN in the basic reference frame. */
agc_status_t agc_ephem_lm_lspos(agc_interp_t *ip, agc_tp_t tephem,
                                agc_dp_t t_cs, agc_vec_t *vmoon,
                                agc_vec_t *vsun);

/* LSPOS for the vehicle selected by agc_state_vehicle(), using the pad load
 * (agc_navaux_set_padload) and, for the CSM, agc_ephem_set_csm_data().
 * time_cs: ground elapsed time, cs scale 2^28.
 *   CSM: *moon_pos = moon position (earth-centred) scale 2^29 m
 *        *sun_pos  = sun position  (earth-centred) scale 2^38 m
 *   LM:  *moon_pos = VMOON unit vector scale 2^1
 *        *sun_pos  = VSUN  unit vector scale 2^1                          */
agc_status_t agc_lsposition(agc_interp_t *ip, agc_dp_t time_cs,
                            agc_vec_t *moon_pos, agc_vec_t *sun_pos);

/* ======================================================================
 * Star table (star_tables.c): 37 navigation stars, unit vectors in the
 * basic reference frame, scale 2^1 (2DEC ... B-1).
 * ==================================================================== */
#define AGC_STAR_COUNT  37
#define AGC_STAR_CATLOG 6970 /* CATLOG DEC 6970 (oct 15472)                */

agc_status_t agc_star_vector(int star, agc_vec_t *out); /* star 1..37      */

#endif /* AGC_NAVAUX_H */
