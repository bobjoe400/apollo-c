/*
 * align_locsam.c -- star selection and sighting arithmetic of P51-P53.agc:
 * S50/LOCSAM (sun, earth, moon lines of sight and occultation cones),
 * OCCULT, PICAPAR (R56 in Luminary: best star pair), PLANET (reference
 * vector of the sighted body), the CHKSDATA (R54) angle test, the R51
 * AXISGEN set-up and CHKSCODE.  (SVDWN1/SVDWN2 are in src/nav/svdwn.c.)
 *
 * Comanche and Luminary differ in LOCSAM (CSM uses conic state, the
 * ecliptic-pole aberration term and a 15 degree sun cone; LM uses the
 * precision state and a 60 degree sun cone), in PICAPAR (shaft axis,
 * separation limits, test order, final ordering) and in PLANET (CSM adds
 * the aberration vector VEL/C).  Both are kept, selected by
 * agc_state_vehicle().
 */
#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_flags_common.h"
#include "agc/agc_navaux.h"
#include "align_internal.h"

/* ---- constants (2DEC value -> DP integer = value * 2^28 / 2^B) --------- */
#define RSUBM      ((agc_dp_t)869045)     /* 1738090 B-29   moon radius m     */
#define RSUBE      ((agc_dp_t)3189083)    /* 6378166 B-29   earth radius m    */
#define RSUBEM     ((agc_dp_t)192201000)  /* 384402000 B-29 earth-moon m (LM) */
#define ROE        ((agc_dp_t)690215)     /* .00257125 R(EM)/R(ES) (LM)       */
#define DEG5_REVS  ((agc_dp_t)3728270)    /* 5DEGREES .013888889 revs         */
#define CSS5       ((agc_dp_t)66853179)   /* .2490475  (COS 5)/4              */
#define CSSUN_CSM  ((agc_dp_t)64821794)   /* .24148    (COS 15)/4             */
#define CSSUN_LM   ((agc_dp_t)33554432)   /* .125      (COS 60)/4             */
#define ONE_OVER_C ((agc_dp_t)5731)       /* 1/C .000042699 B-1 (cs/m)        */
#define ECLIPOL_Y  ((agc_dp_t)-10598)     /* ECLIPOL -.00007896 B-1           */
#define ECLIPOL_Z  ((agc_dp_t)24440)      /* ECLIPOL  .00018209 B-1           */
#define SIN33      ((agc_dp_t)144321135)  /* .5376381241                      */
#define COS33      ((agc_dp_t)226338251)  /* .8431756920                      */
#define HALFDP     AGC_DP_HALF            /* HALFDP .5                        */
#define CSS66      ((agc_dp_t)16235103)   /* .060480472 (COS76)/4             */
#define CSS6640    ((agc_dp_t)-41882876)  /* -.15602587 (COS76 - COS30)/4     */
#define CSS33_CSM  ((agc_dp_t)52882506)   /* .197002688 COS(38)/4             */
#define CSS40_LM   ((agc_dp_t)43137578)   /* .16070  COS 50 / 4 (CSS40=CSS33) */
#define ONE_SQRT3  ((agc_dp_t)154981267)  /* 1/SQR3 .57735021                 */
#define STAR6_SUN  228                    /* DEC227 + 1: codes 46,47,50 octal */
#define STAR6_MAX  240                    /* moon code 50 octal * 6           */
#define LOCSAM_EARTH_SHIFT 9              /* VSR8 + VSR1 (CSM moon sphere)    */

/*
 * OCCOS (P51-P53.agc, Comanche p. 750 / Luminary p. 936) -- occultation cone
 * cosine for a body of radius r at distance |RATT| (left in 36D by UNIT):
 *   COS(ASIN(r/|R|) + 5 deg)/4
 */
static agc_dp_t occos(agc_interp_t *ip, agc_dp_t radius, agc_dp_t rmag)
{
    (void)AGC_ASSERT(rmag > 0);
    (void)AGC_ASSERT(radius > 0);
    agc_dp_t a = agc_dp_shr(agc_dp_div(ip, radius, rmag), 1);
    a = agc_dp_add(ip, agc_dp_asin(ip, a), DEG5_REVS);
    return agc_dp_shr(agc_dp_cos(ip, a), 1);
}

/* LOCSAM earth/moon centred branches for the CSM (Comanche p. 749-750). */
static void locsam_csm(agc_interp_t *ip, const agc_state_vector_t *sv,
                       agc_vec_t moon, agc_vec_t sun, agc_align_locsam_t *o)
{
    agc_dp_t rmag = 0;
    (void)AGC_ASSERT(sv != NULL);
    (void)AGC_ASSERT(o != NULL);
    if (!sv->moon) {                                       /* EARTCNTR   */
        o->vmoon = agc_vec_unit(ip, agc_vec_sub(ip, moon, sv->r), NULL, NULL);
        o->vearth = agc_vec_neg(agc_vec_unit(ip, sv->r, &rmag, NULL));
        o->cearth = occos(ip, RSUBE, rmag);
        o->cmoon = CSS5;
        o->vsun = agc_vec_unit(ip, sun, NULL, NULL);
    } else {                                               /* MOONCNTR   */
        o->vsun = agc_vec_unit(ip, agc_vec_sub(ip, sun,
                               agc_vec_shr(moon, LOCSAM_EARTH_SHIFT)), NULL, NULL);
        o->vearth = agc_vec_neg(agc_vec_unit(ip, agc_vec_add(ip, moon, sv->r),
                                             NULL, NULL));
        o->vmoon = agc_vec_neg(agc_vec_unit(ip, sv->r, &rmag, NULL));
        o->cmoon = occos(ip, RSUBM, rmag);
        o->cearth = CSS5;
    }
    /* ENDSAM: VEL/C = VSUN x ECLIPOL + VATT (1/C)                         */
    const agc_vec_t eclipol = agc_vec_make(0, ECLIPOL_Y, ECLIPOL_Z);
    const agc_vec_t aber = agc_vec_cross(ip, o->vsun, eclipol);
    o->vel_c = agc_vec_add(ip, agc_vec_scale(ip, sv->v, ONE_OVER_C), aber);
    o->csun = CSSUN_CSM;
}

/* LOCSAM earth/moon centred branches for the LM (Luminary p. 935-936). */
static void locsam_lm(agc_interp_t *ip, const agc_state_vector_t *sv,
                      agc_vec_t moon, agc_vec_t sun, agc_align_locsam_t *o)
{
    agc_dp_t rmag = 0;
    (void)AGC_ASSERT(sv != NULL);
    (void)AGC_ASSERT(o != NULL);
    o->vsun = sun;
    if (!sv->moon) {                                       /* EARTCNTR   */
        const agc_vec_t rem = agc_vec_shl(ip, agc_vec_scale(ip, moon, RSUBEM), 1);
        o->vmoon = agc_vec_unit(ip, agc_vec_sub(ip, rem, sv->r), NULL, NULL);
        o->vearth = agc_vec_neg(agc_vec_unit(ip, sv->r, &rmag, NULL));
        o->cearth = occos(ip, RSUBE, rmag);
        o->cmoon = CSS5;
    } else {                                               /* MOONCNTR   */
        o->vsun = agc_vec_unit(ip, agc_vec_sub(ip, sun,
                               agc_vec_scale(ip, moon, ROE)), NULL, NULL);
        const agc_vec_t rem = agc_vec_scale(ip, moon, RSUBEM);
        o->vearth = agc_vec_neg(agc_vec_unit(ip, agc_vec_add(ip, rem, sv->r),
                                             NULL, NULL));
        o->vmoon = agc_vec_neg(agc_vec_unit(ip, sv->r, &rmag, NULL));
        o->cmoon = occos(ip, RSUBM, rmag);
        o->cearth = CSS5;
    }
    o->vel_c = agc_vec_zero();
    o->csun = CSSUN_LM;                                    /* ENDSAM     */
}

/*
 * LOCSAM tail (Comanche pp. 749-750, Luminary pp. 935-936): the lines of
 * sight and cones from the integrated state sv (RATT 2^29 m, VATT 2^7
 * m/cs) and the LSPOS output (CSM: moon 2^29 m, sun 2^38 m earth centred;
 * LM: unit vectors VMOON, VSUN scale 2).
 */
agc_status_t agc_align_locsam_from(agc_interp_t *ip, const agc_state_vector_t *sv,
                                   agc_vec_t moon, agc_vec_t sun,
                                   agc_align_locsam_t *out)
{
    if (!AGC_ASSERT((ip != NULL) && (sv != NULL)) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_state_vehicle() == AGC_VEHICLE_CSM) {
        locsam_csm(ip, sv, moon, sun, out);
    } else {
        locsam_lm(ip, sv, moon, sun, out);
    }
    return AGC_STATUS_OK;
}

/*
 * S50 alias LOCSAM (P51-P53.agc, Comanche pp. 748-751, Luminary pp.
 * 934-936) -- lines of sight to earth, sun and moon and their occultation
 * cone cosines at time time_cs (cs B28).  CSM: LSPOS + CSMCONIC; LM: LSPOS
 * + LEMPREC.  TSIGHT is set to the time.  BUSY: asleep in INTSTALL.
 */
agc_status_t agc_align_locsam(agc_interp_t *ip, agc_dp_t time_cs,
                              agc_job_id_t resume_job, agc_align_locsam_t *out)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t moon;
    agc_vec_t sun;
    agc_align_ram()->tsight = time_cs;
    agc_status_t st = agc_lsposition(ip, time_cs, &moon, &sun);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    agc_integ_out_t o;
    st = align_integ(ip, csm ? ALIGN_CSMCONIC : ALIGN_LEMPREC, time_cs,
                     resume_job, &o);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    agc_state_vector_t sv;
    sv.r = o.ratt;
    sv.v = o.vatt;
    sv.t = o.tat;
    sv.moon = (o.x2 != 0);                    /* X2: 0 earth, 2 moon       */
    return agc_align_locsam_from(ip, &sv, moon, sun, out);
}

/*
 * OCCULT (Comanche p. 754 / Luminary p. 939) -- test a half-unit star
 * vector against the three occultation cones.  With CULTRIX = (VEARTH,
 * VSUN, VMOON) and CSS = (CEARTH, CSUN, CMOON):  D = CSS - CULTRIX STAR.
 * Occulted (CULTFLAG set) when D.x is zero or negative or D.y or D.z is
 * negative (the BZE / BMN / SIGN sequence).  Returns the CULTFLAG value.
 */
bool agc_align_occult(agc_interp_t *ip, const agc_align_locsam_t *ls, agc_vec_t star)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(ls != NULL)) {
        return true;
    }
    agc_mat_t cultrix;
    cultrix.row[0] = ls->vearth;
    cultrix.row[1] = ls->vsun;
    cultrix.row[2] = ls->vmoon;
    const agc_vec_t css = agc_vec_make(ls->cearth, ls->csun, ls->cmoon);
    const agc_vec_t d = agc_vec_sub(ip, css, agc_mat_vec(ip, &cultrix, star));
    const bool cult = (d.c[0] <= 0) || (d.c[1] < 0) || (d.c[2] < 0);
    return cult;
}

/* Catalogue star by its index *6 (CATLOG,1 with X1 = 6 x star number). */
static agc_vec_t catlog(int star6)
{
    agc_vec_t v = agc_vec_zero();
    (void)AGC_ASSERT((star6 % 6) == 0);
    if (AGC_ASSERT((star6 >= 6) && (star6 <= 6 * AGC_ALIGN_NSTARS))) {
        if (agc_star_vector(star6 / 6, &v) != AGC_STATUS_OK) {
            v = agc_vec_zero();
        }
    }
    return v;
}

/* PICAPAR state for one search (the X1/X2 index registers and VFLAG). */
typedef struct picapar_search {
    agc_interp_t *ip;
    const agc_align_locsam_t *ls;
    agc_vec_t sax;
    bool vflag;            /* VFLAG: no pair found yet */
    int besti, bestj;      /* star numbers * 6         */
} picapar_search_t;

/* STRATGY / STRAT / NEWPAR (Comanche p. 754, Luminary p. 939): keep the
 * pair with the smaller dot product (larger separation). */
static void picapar_strategy(picapar_search_t *s, int i6, int j6)
{
    (void)AGC_ASSERT(s != NULL);
    (void)AGC_ASSERT((i6 > j6) && (j6 >= 6));
    if (s->vflag) {                                   /* BONCLR VFLAG    */
        s->vflag = false;
        s->besti = i6;                                /* NEWPAR          */
        s->bestj = j6;
        return;
    }
    const agc_dp_t best = agc_vec_dot(s->ip, catlog(s->besti), catlog(s->bestj));
    const agc_dp_t cur = agc_vec_dot(s->ip, catlog(i6), catlog(j6));
    if (agc_dp_sub(s->ip, cur, best) >= 0) {          /* BPL PIC3        */
        return;
    }
    s->besti = i6;                                    /* NEWPAR          */
    s->bestj = j6;
}

/* PIC1-PIC4 for the CSM (Comanche pp. 753-754). */
static void picapar_search_csm(picapar_search_t *s)
{
    (void)AGC_ASSERT(s != NULL);
    (void)AGC_ASSERT(s->ip != NULL);
    for (int i6 = 6 * AGC_ALIGN_NSTARS; i6 >= 6; i6 -= 6) {          /* PIC1 */
        const agc_vec_t si = catlog(i6);
        if (agc_align_occult(s->ip, s->ls, si)) {
            continue;                                                  /* PIC2 */
        }
        for (int j6 = i6 - 6; j6 >= 6; j6 -= 6) {                     /* PIC3 */
            const agc_vec_t sj = catlog(j6);
            if (agc_align_occult(s->ip, s->ls, sj)) {
                continue;                                              /* PIC4 */
            }
            agc_dp_t d = agc_dp_sub(s->ip, agc_vec_dot(s->ip, si, sj), CSS66);
            if (d < 0) {
                continue;            /* separation outside the upper limit */
            }
            if (agc_dp_add(s->ip, d, CSS6640) >= 0) {
                continue;            /* separation less than 30 degrees     */
            }
            if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, si, s->sax), CSS33_CSM) < 0) {
                break;               /* major star not in cone: PIC1        */
            }
            if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, sj, s->sax), CSS33_CSM) < 0) {
                continue;
            }
            picapar_strategy(s, i6, j6);                               /* STRATGY */
        }
    }
}

/* PIC1-PIC4 for the LM (Luminary pp. 938-939): cone tests first. */
static void picapar_search_lm(picapar_search_t *s)
{
    (void)AGC_ASSERT(s != NULL);
    (void)AGC_ASSERT(s->ip != NULL);
    for (int i6 = 6 * AGC_ALIGN_NSTARS; i6 >= 6; i6 -= 6) {          /* PIC1 */
        const agc_vec_t si = catlog(i6);
        if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, si, s->sax), CSS40_LM) < 0) {
            continue;                                                  /* PIC2 */
        }
        for (int j6 = i6 - 6; j6 >= 6; j6 -= 6) {                     /* PIC3 */
            const agc_vec_t sj = catlog(j6);
            if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, sj, s->sax), CSS40_LM) < 0) {
                continue;                                              /* PIC4 */
            }
            if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, si, sj), CSS40_LM) >= 0) {
                continue;            /* separation less than 50 degrees     */
            }
            if (agc_align_occult(s->ip, s->ls, si)) {
                break;               /* major star occulted: PIC1           */
            }
            if (agc_align_occult(s->ip, s->ls, sj)) {
                continue;
            }
            picapar_strategy(s, i6, j6);                               /* STRATGY */
        }
    }
}

/*
 * PICAPAR (Comanche pp. 752-755) / R56 alias PICAPAR (Luminary pp.
 * 937-940) -- choose the pair of catalogue stars with the largest
 * separation inside the separation window and the field of view about the
 * optics shaft axis SAX.  SAX (reference) = UNIT((XNB a + ZNB b) VXM
 * REFSMMAT) with (a,b) = (SIN33, COS33) for the CSM sextant and (.5, .5)
 * for the LM AOT.  Output: BESTI, BESTJ (star number * 6), *found (VFLAG
 * clear).  Luminary finally orders the pair so BESTJ is nearer SAX.
 */
agc_status_t agc_align_picapar(agc_interp_t *ip, const agc_align_locsam_t *ls,
                               const agc_mat_t *refsmmat, const uint16_t cdu[3],
                               agc_sp_t *besti, agc_sp_t *bestj, bool *found)
{
    if (!AGC_ASSERT((ip != NULL) && (ls != NULL) && (refsmmat != NULL)) ||
        !AGC_ASSERT((cdu != NULL) && (besti != NULL) && (bestj != NULL) &&
                    (found != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_cdutrig_t trig;
    agc_mat_t nb;
    agc_status_t st = agc_align_cdutrig(cdu, &trig);          /* CDUTRIG  */
    if (st == AGC_STATUS_OK) {
        st = agc_align_calcsmsc(ip, &trig, &nb);              /* CALCSMSC */
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    const agc_vec_t xs = agc_vec_scale(ip, nb.row[0], csm ? SIN33 : HALFDP);
    const agc_vec_t zs = agc_vec_scale(ip, nb.row[2], csm ? COS33 : HALFDP);
    picapar_search_t s = { .ip = ip, .ls = ls, .vflag = true, .besti = 0, .bestj = 0 };
    s.sax = agc_vec_unit(ip, agc_vec_mat(ip, agc_vec_add(ip, zs, xs), refsmmat),
                         NULL, NULL);
    agc_align_ram()->sax = s.sax;
    if (csm) {
        picapar_search_csm(&s);
    } else {
        picapar_search_lm(&s);
        if (!s.vflag) {                                     /* PICGXT */
            const agc_dp_t di = agc_vec_dot(ip, s.sax, catlog(s.besti));
            const agc_dp_t dj = agc_vec_dot(ip, s.sax, catlog(s.bestj));
            if (agc_dp_sub(ip, dj, di) < 0) {
                const int t = s.besti;
                s.besti = s.bestj;
                s.bestj = t;
            }
        }
    }
    *besti = (agc_sp_t)s.besti;
    *bestj = (agc_sp_t)s.bestj;
    *found = !s.vflag;
    return (agc_flag_set(AGC_FLAG_VFLAG) == AGC_STATUS_OK) && !s.vflag
               ? agc_flag_clear(AGC_FLAG_VFLAG) : AGC_STATUS_OK;
}

/*
 * PLANET (Comanche pp. 782-784 / Luminary pp. 961-962) -- reference
 * half-unit vector of the sighted body.  star6 = 6 x star code:
 *   0          planet keyed in on N88 (STARSAV3 * 1/SQR3 CSM, STARAD LM)
 *   6..222     catalogue star (CATLOG)
 *   228,234,240  sun, earth, moon from LOCSAM (VEARTH and VSUN exchanged
 *              first so that codes 46, 47, 50 octal index sun, earth, moon)
 * CSM: LOCSAM is always run at TSIGHT and the result is corrected for
 * aberration: UNIT(v + VEL/C) (CORPLAN).  BESTI+starind = star6.
 */
agc_status_t agc_align_planet_vector(agc_interp_t *ip, agc_sp_t star6,
                                     agc_sp_t starind, agc_dp_t time_cs,
                                     agc_job_id_t resume_job, agc_vec_t *los)
{
    if (!AGC_ASSERT((ip != NULL) && (los != NULL)) ||
        !AGC_ASSERT((star6 >= 0) && (star6 <= STAR6_MAX) && (starind >= 0) &&
                    (starind <= 1))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    ram->tsight = time_cs;
    agc_status_t st = AGC_STATUS_OK;
    if (csm || (star6 >= STAR6_SUN)) {
        st = agc_align_locsam(ip, time_cs, resume_job, &ram->locsam);
        if (st != AGC_STATUS_OK) {
            return st;
        }
    }
    if (starind == 0) {
        ram->besti = star6;
    } else {
        ram->bestj = star6;
    }
    agc_vec_t v;
    if (star6 == 0) {                                          /* planet  */
        v = csm ? agc_vec_unit(ip, agc_vec_scale(ip, ram->starsav3, ONE_SQRT3),
                               NULL, NULL)
                : agc_vec_unit(ip, ram->starad[0], NULL, NULL);
    } else if (star6 < STAR6_SUN) {
        v = catlog(star6);                                     /* NOTPLAN */
    } else if (star6 == STAR6_SUN) {
        v = ram->locsam.vsun;           /* STARAD+0 after the exchange    */
    } else if (star6 == STAR6_SUN + 6) {
        v = ram->locsam.vearth;
    } else {
        v = ram->locsam.vmoon;
    }
    *los = csm ? agc_vec_unit(ip, agc_vec_add(ip, v, ram->locsam.vel_c), NULL, NULL)
               : v;                                            /* CORPLAN */
    return st;
}

/*
 * CHKSDATA / R54 angle test (Comanche pp. 759-760, Luminary pp. 945-946):
 * first pass X1 = STARAD:  18D   = ACOS(STARAD . STARAD+6)   (catalogue)
 * second pass X1 = 6D:     THETA = ACOS(6D . 12D)            (observed)
 * NORMTEM1 = ABS(THETA - 18D), revs B0.  All vectors half-unit.
 */
agc_status_t agc_align_chksdata_angle(agc_interp_t *ip, const agc_vec_t cat[2],
                                      const agc_vec_t obs[2], agc_dp_t *diff)
{
    if (!AGC_ASSERT((ip != NULL) && (obs != NULL)) ||
        !AGC_ASSERT((cat != NULL) && (diff != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    ram->theta18 = agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, cat[0], cat[1]), 1));
    ram->theta = agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, obs[0], obs[1]), 1));
    *diff = agc_dp_abs(agc_dp_sub(ip, ram->theta, ram->theta18));
    ram->normtem1 = *diff;            /* RTB SGNAGREE: no-op in this model */
    return AGC_STATUS_OK;
}

/*
 * R51 fine-align solution (Comanche p. 757 / Luminary p. 942): AXISGEN of
 * the desired-SM catalogue pair a (STARAD, STARAD+6) against the observed
 * present-SM pair b (6D, 12D) gives XDC, the desired SM wrt the present SM.
 */
agc_status_t agc_align_fine_solve(agc_interp_t *ip, const agc_vec_t a[2],
                                 const agc_vec_t b[2], agc_mat_t *xdc)
{
    if (!AGC_ASSERT((ip != NULL) && (a != NULL)) ||
        !AGC_ASSERT((b != NULL) && (xdc != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_align_ram_t *ram = agc_align_ram();
    const agc_status_t st = agc_imu_axisgen(ip, a, b, xdc);   /* AXISGEN */
    if (st == AGC_STATUS_OK) {
        ram->xdc = *xdc;
    }
    return st;
}

/*
 * CHKSCODE (Comanche p. 773): star codes 0..50 octal are acceptable
 * (L+3 return), anything else is not (L+1).  STARCODE -0 counts as 0.
 */
bool agc_align_chkscode(agc_sp_t starcode)
{
    (void)AGC_ASSERT(starcode >= AGC_SP_MIN);
    (void)AGC_ASSERT(starcode <= AGC_SP_MAX);
    return (starcode >= 0) && (starcode <= AGC_ALIGN_STAR_CODE_MAX);
}
