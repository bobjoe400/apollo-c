/**********************************************************************
 * FILE NAME: locsam.c
 *
 * PURPOSE:
 *   Star selection and sighting arithmetic of P51-P53: S50/LOCSAM (sun,
 *   earth, moon lines of sight and occultation cones), OCCULT, PICAPAR
 *   (R56 in Luminary: the best star pair), the PLANET vector of the
 *   sighted body, the CHKSDATA (R54) angle test, the R51 AXISGEN set-up
 *   and CHKSCODE.  SVDWN1/SVDWN2 are in src/nav/integration/svdwn.c.
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 748-760, 773, 783 (LOCSAM .. PICAPAR, CHKSDATA, CHKSCODE, PLANET)
 *   Luminary099 P51-P53.agc pp. 934-946 (LOCSAM .. PICAPAR, CHKSDATA)
 *   STAR_TABLES.agc pp. 1389-1393 (CATLOG)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   TSIGHT, BESTI, BESTJ, SAX, LOCSAM block, THETA, 18D, NORMTEM1, XDC, STARSAV3, STARAD: agc_align_ram()
 *   VFLAG         flag      O                   no pair found
 *   REFSMMAT      agc_mat_t I    half-unit      agc_erase_refsmmat()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_imu_axisgen       IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_star_vector       star catalogue (auxiliary.h)
 *   agc_lsposition        LSPOS (auxiliary.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_mat_vec           matrix arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_flag_put          flag words (flags_common.h)
 *   agc_state_vehicle     vehicle state (state.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   Comanche and Luminary differ in LOCSAM (CSM: conic state, the
 *   ecliptic-pole aberration term, 15 degree sun cone; LM: precision
 *   state, 60 degree sun cone), in PICAPAR (shaft axis, separation
 *   limits, test order, final ordering) and in PLANET (CSM adds VEL/C).
 *   Both are kept, selected by agc_state_vehicle().  PICAPAR loads the 37
 *   catalogue stars once per search.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"

/* ---- constants (2DEC value -> DP integer = value * 2^28 / 2^B) --------- */
#define RSUBM              ((agc_dp_t) 869045)    /* 1738090 B-29   moon radius m     */
#define RSUBE              ((agc_dp_t) 3189083)   /* 6378166 B-29   earth radius m    */
#define RSUBEM             ((agc_dp_t) 192201000) /* 384402000 B-29 earth-moon m (LM) */
#define ROE                ((agc_dp_t) 690215)    /* .00257125 R(EM)/R(ES) (LM)       */
#define DEG5_REVS          ((agc_dp_t) 3728270)   /* 5DEGREES .013888889 revs         */
#define CSS5               ((agc_dp_t) 66853179)  /* .2490475  (COS 5)/4              */
#define CSSUN_CSM          ((agc_dp_t) 64821794)  /* .24148    (COS 15)/4             */
#define CSSUN_LM           ((agc_dp_t) 33554432)  /* .125      (COS 60)/4             */
#define ONE_OVER_C         ((agc_dp_t) 5731)      /* 1/C .000042699 B-1 (cs/m)        */
#define ECLIPOL_Y          ((agc_dp_t) -10598)    /* ECLIPOL -.00007896 B-1           */
#define ECLIPOL_Z          ((agc_dp_t) 24440)     /* ECLIPOL  .00018209 B-1           */
#define SIN33              ((agc_dp_t) 144321135) /* .5376381241                      */
#define COS33              ((agc_dp_t) 226338251) /* .8431756920                      */
#define HALFDP             AGC_DP_HALF            /* HALFDP .5                        */
#define CSS66              ((agc_dp_t) 16235103)  /* .060480472 (COS76)/4             */
#define CSS6640            ((agc_dp_t) -41882876) /* -.15602587 (COS76 - COS30)/4     */
#define CSS33_CSM          ((agc_dp_t) 52882506)  /* .197002688 COS(38)/4             */
#define CSS40_LM           ((agc_dp_t) 43137578)  /* .16070  COS 50 / 4 (CSS40=CSS33) */
#define ONE_SQRT3          ((agc_dp_t) 154981267) /* 1/SQR3 .57735021                 */
#define STAR6_SUN          228                    /* DEC227 + 1: codes 46,47,50 octal */
#define STAR6_MAX          240                    /* moon code 50 octal * 6           */
#define LOCSAM_EARTH_SHIFT 9                      /* VSR8 + VSR1 (CSM moon sphere)    */

/**********************************************************************
 * OCCOS (P51-P53.agc, Comanche p. 750 / Luminary p. 936) -- occultation
 * cone cosine for a body of radius r at distance |RATT| (left in 36D by
 * UNIT): COS(ASIN(r/|R|) + 5 deg)/4
 *
 * FUNCTION NAME: occos
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   radius    agc_dp_t        I    body radius (RSUBE / RSUBM), same scale
 *                                  as rmag
 *   rmag      agc_dp_t        I    distance to the body centre, m 2^29 (CSM
 *                                  earth) or as UNIT gives it
 *
 * RETURNS: agc_dp_t -- COS(ASIN(radius / rmag) + 5 deg) / 4, scale 2^2
 **********************************************************************/
static agc_dp_t occos(agc_interp_t *ip, agc_dp_t radius, agc_dp_t rmag)
{
    agc_dp_t a = agc_dp_shr(agc_dp_div(ip, radius, rmag), 1); /* radius / rmag / 2, then the cone half-angle, revs 2^0 */

    a = agc_dp_add(ip, agc_dp_asin(ip, a), DEG5_REVS);

    return (agc_dp_shr(agc_dp_cos(a), 1));
}

/**********************************************************************
 * LOCSAM tail (Comanche pp. 749-750, Luminary pp. 935-936): the lines of
 * sight and cones from the integrated state sv (RATT 2^29 m, VATT 2^7 m/cs)
 * and the LSPOS output (CSM: moon 2^29 m, sun 2^38 m earth centred; LM:
 * unit vectors VMOON, VSUN scale 2).
 *
 * FUNCTION NAME: agc_align_locsam_from
 *
 * ARGUMENTS:
 *   Argument  Type                        I/O  Description (scale)
 *   --------  --------------------------  ---  --------------------------
 *   ip        agc_interp_t *              IO   interpreter state (OVFIND,
 *                                              MPAC words)
 *   sv        const agc_state_vector_t *  I    integrated state: R m 2^29 /
 *                                              2^27, V m/cs 2^7 / 2^5, X2
 *                                              earth or moon
 *   moon      agc_vec_t                   I    LSPOS moon: CSM m 2^29
 *                                              (earth centred), LM unit
 *                                              vector scale 2^1
 *   sun       agc_vec_t                   I    LSPOS sun: CSM m 2^38 (earth
 *                                              centred), LM unit vector
 *                                              scale 2^1
 *   out       agc_align_locsam_t *        O    LOCSAM results: VEARTH,
 *                                              VSUN, VMOON half-unit;
 *                                              CEARTH, CSUN, CMOON scale
 *                                              2^2; VEL/C
 *
 * RETURNS: none
 **********************************************************************/
void agc_align_locsam_from(agc_interp_t *ip, const agc_state_vector_t *sv, agc_vec_t moon, agc_vec_t sun, agc_align_locsam_t *out)
{
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        /* LOCSAM earth/moon centred branches for the CSM (Comanche p. 749-750). */
        agc_dp_t rmag = 0; /* |R| from UNIT, m 2^29 (earth) or 2^27 (moon) */

        if (!sv->moon)
        { /* EARTCNTR   */
            out->vmoon  = agc_vec_unit(ip, agc_vec_sub(ip, moon, sv->r), NULL, NULL);
            out->vearth = agc_vec_neg(agc_vec_unit(ip, sv->r, &rmag, NULL));
            out->cearth = occos(ip, RSUBE, rmag);
            out->cmoon  = CSS5;
            out->vsun   = agc_vec_unit(ip, sun, NULL, NULL);
        }
        else
        { /* MOONCNTR   */
            out->vsun   = agc_vec_unit(ip, agc_vec_sub(ip, sun, agc_vec_shr(moon, LOCSAM_EARTH_SHIFT)), NULL, NULL);
            out->vearth = agc_vec_neg(agc_vec_unit(ip, agc_vec_add(ip, moon, sv->r), NULL, NULL));
            out->vmoon  = agc_vec_neg(agc_vec_unit(ip, sv->r, &rmag, NULL));
            out->cmoon  = occos(ip, RSUBM, rmag);
            out->cearth = CSS5;
        }

        /* ENDSAM: VEL/C = VSUN x ECLIPOL + VATT (1/C)                         */
        const agc_vec_t eclipol = agc_vec_make(0, ECLIPOL_Y, ECLIPOL_Z); /* ECLIPOL, the ecliptic pole, half-unit (2^1) */
        const agc_vec_t aber    = agc_vec_cross(ip, out->vsun, eclipol); /* VSUN x ECLIPOL, earth orbital velocity direction, scale 2^2 */

        out->vel_c = agc_vec_add(ip, agc_vec_scale(ip, sv->v, ONE_OVER_C), aber);
        out->csun  = CSSUN_CSM;
    }
    else
    {
        /* LOCSAM earth/moon centred branches for the LM (Luminary p. 935-936). */
        {
            agc_dp_t rmag = 0; /* |R| from UNIT, m 2^29 (earth) or 2^27 (moon) */

            out->vsun = sun;

            if (!sv->moon)
            { /* EARTCNTR   */
                const agc_vec_t rem = agc_vec_shl(ip, agc_vec_scale(ip, moon, RSUBEM), 1); /* moon position from earth, m 2^29 */

                out->vmoon  = agc_vec_unit(ip, agc_vec_sub(ip, rem, sv->r), NULL, NULL);
                out->vearth = agc_vec_neg(agc_vec_unit(ip, sv->r, &rmag, NULL));
                out->cearth = occos(ip, RSUBE, rmag);
                out->cmoon  = CSS5;
            }
            else
            { /* MOONCNTR   */
                out->vsun           = agc_vec_unit(ip, agc_vec_sub(ip, sun, agc_vec_scale(ip, moon, ROE)), NULL, NULL);
                const agc_vec_t rem = agc_vec_scale(ip, moon, RSUBEM); /* moon position from earth, m 2^29 */

                out->vearth = agc_vec_neg(agc_vec_unit(ip, agc_vec_add(ip, rem, sv->r), NULL, NULL));
                out->vmoon  = agc_vec_neg(agc_vec_unit(ip, sv->r, &rmag, NULL));
                out->cmoon  = occos(ip, RSUBM, rmag);
                out->cearth = CSS5;
            }

            out->vel_c = agc_vec_zero();
            out->csun  = CSSUN_LM; /* ENDSAM     */
        }
    }
} /* agc_align_locsam_from */

/**********************************************************************
 * S50 alias LOCSAM (P51-P53.agc, Comanche pp. 748-751, Luminary pp.
 * 934-936) -- lines of sight to earth, sun and moon and their occultation
 * cone cosines at time time_cs (cs B28).  CSM: LSPOS + CSMCONIC; LM: LSPOS
 * + LEMPREC.  TSIGHT is set to the time.  BUSY: asleep in INTSTALL.
 *
 * FUNCTION NAME: agc_align_locsam
 *
 * ARGUMENTS:
 *   Argument    Type                  I/O  Description (scale)
 *   ----------  --------------------  ---  ------------------------------
 *   ip          agc_interp_t *        IO   interpreter state (OVFIND, MPAC
 *                                          words)
 *   time_cs     agc_dp_t              I    time, cs 2^28
 *   resume_job  agc_job_id_t          I    job resumed after an integration
 *                                          wait (INTSTALL)
 *   out         agc_align_locsam_t *  O    LOCSAM results (lines of sight, cone cosines)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_align_locsam(agc_interp_t *ip, agc_dp_t time_cs, agc_job_id_t resume_job, agc_align_locsam_t *out)
{
    agc_vec_t moon; /* LSPOS moon (see agc_align_locsam_from), scale 2^29 m or 2^1 */
    agc_vec_t sun;  /* LSPOS sun (see agc_align_locsam_from), scale 2^38 m or 2^1 */

    agc_align_ram()->tsight = time_cs;
    agc_status_t st         = agc_lsposition(ip, time_cs, &moon, &sun); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const bool      csm = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* running on the CSM */
    agc_integ_out_t o;                                              /* integration output (RATT, VATT) */

    st = align_integ(ip, csm ? ALIGN_CSMCONIC : ALIGN_LEMPREC, time_cs, resume_job, &o); /* per vehicle */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_state_vector_t sv; /* the integrated state for the tail */

    sv.r    = o.ratt;
    sv.v    = o.vatt;
    sv.t    = o.tat;
    sv.moon = (o.x2 != 0); /* X2: 0 earth, 2 moon       */

    agc_align_locsam_from(ip, &sv, moon, sun, out);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * OCCULT (Comanche p. 754 / Luminary p. 939) -- test a half-unit star
 * vector against the three occultation cones.  With CULTRIX = (VEARTH,
 * VSUN, VMOON) and CSS = (CEARTH, CSUN, CMOON):  D = CSS - CULTRIX STAR.
 * Occulted (CULTFLAG set) when D.x is zero or negative or D.y or D.z is
 * negative (the BZE / BMN / SIGN sequence).  Returns the CULTFLAG value.
 *
 * FUNCTION NAME: agc_align_occult
 *
 * ARGUMENTS:
 *   Argument  Type                        I/O  Description (scale)
 *   --------  --------------------------  ---  --------------------------
 *   ip        agc_interp_t *              IO   interpreter state (OVFIND,
 *                                              MPAC words)
 *   ls        const agc_align_locsam_t *  I    LOCSAM results (sun, earth,
 *                                              moon vectors and cone
 *                                              cosines)
 *   star      agc_vec_t                   I    star line of sight,
 *                                              half-unit (2^1)
 *
 * RETURNS: bool -- CULTFLAG: true when the star is inside one of the cones
 **********************************************************************/
bool agc_align_occult(agc_interp_t *ip, const agc_align_locsam_t *ls, agc_vec_t star)
{
    agc_mat_t cultrix; /* CULTRIX: VEARTH, VSUN, VMOON rows, half-unit */

    cultrix.row[0]       = ls->vearth;
    cultrix.row[1]       = ls->vsun;
    cultrix.row[2]       = ls->vmoon;
    const agc_vec_t css  = agc_vec_make(ls->cearth, ls->csun, ls->cmoon);         /* CEARTH, CSUN, CMOON, scale 2^2 */
    const agc_vec_t d    = agc_vec_sub(ip, css, agc_mat_vec(ip, &cultrix, star)); /* CSS - CULTRIX STAR, scale 2^2 */
    const bool      cult = (d.c[0] <= 0) || (d.c[1] < 0) || (d.c[2] < 0);         /* the star is occulted */

    return (cult);
}

/* PICAPAR state for one search (the X1/X2 index registers and VFLAG). */
typedef struct picapar_search
{
    agc_interp_t             *ip;                        /* interpreter state                     */
    const agc_align_locsam_t *ls;                        /* LOCSAM results (cones)                */
    agc_vec_t                 sax;                       /* SAX, shaft axis (reference), half-unit (2^1) */
    bool                      vflag;                     /* VFLAG: no pair found yet              */
    int                       besti;                     /* BESTI: major star x 6                 */
    int                       bestj;                     /* BESTJ: minor star x 6                 */
    agc_vec_t                 cat[AGC_ALIGN_NSTARS + 1]; /* CATLOG by star number, half-unit (2^1) */
} picapar_search_t;

/**********************************************************************
 * AGC: CATLOG (Comanche055 STAR_TABLES.agc p. 1393; Luminary099
 *      CONTROLLED_CONSTANTS.agc p. 51)
 *
 * CATLOG,1 with X1 = 6 x star number (6 .. 222, from the search loops).
 *
 * FUNCTION NAME: catlog
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  ----------------------------
 *   s         const picapar_search_t *  IO   PICAPAR search state
 *   star6     int                       I    star code x 6 (CATLOG index; 0
 *                                            planet, 228/234/240 sun,
 *                                            earth, moon)
 *
 * RETURNS: agc_vec_t -- the catalogue star, half-unit (2^1)
 **********************************************************************/
static agc_vec_t catlog(const picapar_search_t *s, int star6)
{
    return (s->cat[star6 / 6]);
}

/**********************************************************************
 * AGC: none (VLOAD* CATLOG,1 for each star, STAR_TABLES.agc pp. 1389-1393)
 *
 * The catalogue stars 1 .. 37 into s->cat (entry 0 unused).
 *
 * FUNCTION NAME: catlog_load
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------------
 *   s         picapar_search_t *  IO   the search (cat filled)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t catlog_load(picapar_search_t *s)
{
    agc_status_t st = AGC_STATUS_OK; /* status of the call */

    s->cat[0] = agc_vec_zero();

    for (int k = 1; (k <= AGC_ALIGN_NSTARS) && (st == AGC_STATUS_OK); k++)
    {
        st = agc_star_vector(k, &s->cat[k]);
    }

    return (st);
}

/**********************************************************************
 * STRATGY / STRAT / NEWPAR (Comanche p. 754, Luminary p. 939): keep the
 * pair with the smaller dot product (larger separation).
 *
 * FUNCTION NAME: picapar_strategy
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------------
 *   s         picapar_search_t *  IO   PICAPAR search state
 *   i6        int                 I    major star x 6
 *   j6        int                 I    minor star x 6
 *
 * RETURNS: none
 **********************************************************************/
static void picapar_strategy(picapar_search_t *s, int i6, int j6)
{
    if (s->vflag)
    { /* BONCLR VFLAG    */
        s->vflag = false;
        s->besti = i6; /* NEWPAR          */
        s->bestj = j6;
    }
    else
    {
        const agc_dp_t best = agc_vec_dot(s->ip, catlog(s, s->besti), catlog(s, s->bestj)); /* dot product of the kept pair, scale 2^2 */
        const agc_dp_t cur  = agc_vec_dot(s->ip, catlog(s, i6), catlog(s, j6));             /* dot product of this pair, scale 2^2 */

        if (agc_dp_sub(s->ip, cur, best) < 0)
        {
            s->besti = i6; /* NEWPAR          */
            s->bestj = j6;
        }
    }
}

/**********************************************************************
 * PIC1-PIC4 for the CSM (Comanche pp. 753-754).
 *
 * FUNCTION NAME: picapar_search_csm
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------------
 *   s         picapar_search_t *  IO   PICAPAR search state
 *
 * RETURNS: none
 **********************************************************************/
static void picapar_search_csm(picapar_search_t *s)
{
    for (int i6 = 6 * AGC_ALIGN_NSTARS; i6 >= 6; i6 -= 6)
    { /* PIC1 */
        const agc_vec_t si = catlog(s, i6); /* major star, half-unit (2^1) */

        if (agc_align_occult(s->ip, s->ls, si))
        {
            continue; /* PIC2 */
        }

        for (int j6 = i6 - 6; j6 >= 6; j6 -= 6)
        { /* PIC3 */
            const agc_vec_t sj = catlog(s, j6); /* minor star, half-unit (2^1) */

            if (agc_align_occult(s->ip, s->ls, sj))
            {
                continue; /* PIC4 */
            }

            agc_dp_t d = agc_dp_sub(s->ip, agc_vec_dot(s->ip, si, sj), CSS66); /* SI . SJ - CSS66, scale 2^2 */

            if (d < 0)
            {
                continue; /* separation outside the upper limit */
            }

            if (agc_dp_add(s->ip, d, CSS6640) >= 0)
            {
                continue; /* separation less than 30 degrees     */
            }

            if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, si, s->sax), CSS33_CSM) < 0)
            {
                break; /* major star not in cone: PIC1        */
            }

            if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, sj, s->sax), CSS33_CSM) < 0)
            {
                continue;
            }

            picapar_strategy(s, i6, j6); /* STRATGY */
        }
    }
} /* picapar_search_csm */

/**********************************************************************
 * PIC1-PIC4 for the LM (Luminary pp. 938-939): cone tests first.
 *
 * FUNCTION NAME: picapar_search_lm
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------------
 *   s         picapar_search_t *  IO   PICAPAR search state
 *
 * RETURNS: none
 **********************************************************************/
static void picapar_search_lm(picapar_search_t *s)
{
    for (int i6 = 6 * AGC_ALIGN_NSTARS; i6 >= 6; i6 -= 6)
    { /* PIC1 */
        const agc_vec_t si = catlog(s, i6); /* major star, half-unit (2^1) */

        if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, si, s->sax), CSS40_LM) < 0)
        {
            continue; /* PIC2 */
        }

        for (int j6 = i6 - 6; j6 >= 6; j6 -= 6)
        { /* PIC3 */
            const agc_vec_t sj = catlog(s, j6); /* minor star, half-unit (2^1) */

            if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, sj, s->sax), CSS40_LM) < 0)
            {
                continue; /* PIC4 */
            }

            if (agc_dp_sub(s->ip, agc_vec_dot(s->ip, si, sj), CSS40_LM) >= 0)
            {
                continue; /* separation less than 50 degrees     */
            }

            if (agc_align_occult(s->ip, s->ls, si))
            {
                break; /* major star occulted: PIC1           */
            }

            if (agc_align_occult(s->ip, s->ls, sj))
            {
                continue;
            }

            picapar_strategy(s, i6, j6); /* STRATGY */
        }
    }
}

/**********************************************************************
 * PICAPAR (Comanche pp. 752-755) / R56 alias PICAPAR (Luminary pp. 937-940)
 * -- choose the pair of catalogue stars with the largest separation inside
 * the separation window and the field of view about the optics shaft axis
 * SAX.  SAX (reference) = UNIT((XNB a + ZNB b) VXM REFSMMAT) with (a,b) =
 * (SIN33, COS33) for the CSM sextant and (.5, .5) for the LM AOT.  Output:
 * BESTI, BESTJ (star number * 6), *found (VFLAG clear).  Luminary finally
 * orders the pair so BESTJ is nearer SAX.
 *
 * FUNCTION NAME: agc_align_picapar
 *
 * ARGUMENTS:
 *   Argument  Type                        I/O  Description (scale)
 *   --------  --------------------------  ---  --------------------------
 *   ip        agc_interp_t *              IO   interpreter state (OVFIND,
 *                                              MPAC words)
 *   ls        const agc_align_locsam_t *  I    LOCSAM results (sun, earth,
 *                                              moon vectors and cone
 *                                              cosines)
 *   refsmmat  const agc_mat_t *           I    REFSMMAT, half-unit rows
 *                                              (reference -> present SM)
 *   cdu       const uint16_t              I    CDU words X, Y, Z (2^15 = 1
 *                                              rev)
 *   besti     agc_sp_t *                  O    BESTI: first star code x 6
 *   bestj     agc_sp_t *                  O    BESTJ: second star code x 6
 *   found     bool *                      O    true when a star pair was
 *                                              found (VFLAG clear)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_align_picapar(agc_interp_t *ip, const agc_align_locsam_t *ls, const agc_mat_t *refsmmat, const uint16_t cdu[3], agc_sp_t *besti, agc_sp_t *bestj, bool *found)
{
    agc_cdutrig_t trig; /* CDUTRIG sines and cosines */
    agc_mat_t     nb;   /* XNB, YNB, ZNB, half-unit rows */

    /* Field assignments, not an initializer: Frama-C 25 (the proof tool)
     * fails internally on an initializer of this struct; cat is filled by
     * catlog_load and sax computed below before either is read.         */
    picapar_search_t s; /* PICAPAR search state */

    s.ip    = ip;
    s.ls    = ls;
    s.vflag = true;
    s.besti = 0;
    s.bestj = 0;

    agc_align_cdutrig(cdu, &trig);                        /* CDUTRIG  */
    agc_status_t st = agc_align_calcsmsc(ip, &trig, &nb); /* CALCSMSC */

    if (st == AGC_STATUS_OK)
    {
        st = catlog_load(&s);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const bool      csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);           /* running on the CSM */
    const agc_vec_t xs  = agc_vec_scale(ip, nb.row[0], csm ? SIN33 : HALFDP); /* XNB a (SIN33 or .5), scale 2^2; per vehicle */
    const agc_vec_t zs  = agc_vec_scale(ip, nb.row[2], csm ? COS33 : HALFDP); /* ZNB b (COS33 or .5), scale 2^2; per vehicle */

    s.sax                = agc_vec_unit(ip, agc_vec_mat(ip, agc_vec_add(ip, zs, xs), refsmmat), NULL, NULL);
    agc_align_ram()->sax = s.sax;

    if (csm)
    {
        picapar_search_csm(&s);
    }
    else
    {
        picapar_search_lm(&s);

        if (!s.vflag)
        { /* PICGXT */
            const agc_dp_t di = agc_vec_dot(ip, s.sax, catlog(&s, s.besti)); /* SAX . BESTI star, scale 2^2 */
            const agc_dp_t dj = agc_vec_dot(ip, s.sax, catlog(&s, s.bestj)); /* SAX . BESTJ star, scale 2^2 */

            if (agc_dp_sub(ip, dj, di) < 0)
            {
                const int t = s.besti; /* CDUTRIG sines and cosines */

                s.besti = s.bestj;
                s.bestj = t;
            }
        }
    }

    *besti = (agc_sp_t) s.besti;
    *bestj = (agc_sp_t) s.bestj;
    *found = !s.vflag;
    agc_flag_put(AGC_FLAG_VFLAG, s.vflag); /* SET VFLAG; CLEAR if a pair found */

    return (AGC_STATUS_OK);
} /* agc_align_picapar */

/**********************************************************************
 * PLANET (Comanche pp. 782-784 / Luminary pp. 961-962) -- reference
 * half-unit vector of the sighted body.  star6 = 6 x star code: 0
 * planet keyed in on N88 (STARSAV3 * 1/SQR3 CSM, STARAD LM) 6..222
 * catalogue star (CATLOG) 228,234,240  sun, earth, moon from LOCSAM (VEARTH
 * and VSUN exchanged first so that codes 46, 47, 50 octal index sun, earth,
 * moon) CSM: LOCSAM is always run at TSIGHT and the result is corrected for
 * aberration: UNIT(v + VEL/C) (CORPLAN).  BESTI+starind = star6.
 *
 * FUNCTION NAME: agc_align_planet_vector
 *
 * ARGUMENTS:
 *   Argument    Type            I/O  Description (scale)
 *   ----------  --------------  ---  ------------------------------------
 *   ip          agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   star6       agc_sp_t        I    star code x 6 (CATLOG index; 0 planet,
 *                                    228/234/240 sun, earth, moon)
 *   starind     agc_sp_t        I    STARIND: 0 = first star (BESTI), 1 =
 *                                    second (BESTJ)
 *   time_cs     agc_dp_t        I    time, cs 2^28
 *   resume_job  agc_job_id_t    I    job resumed after an integration wait
 *                                    (INTSTALL)
 *   los         agc_vec_t *     O    line of sight, half-unit (2^1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_align_planet_vector(agc_interp_t *ip, agc_sp_t star6, agc_sp_t starind, agc_dp_t time_cs, agc_job_id_t resume_job, agc_vec_t *los)
{
    if (!AGC_ASSERT((ip != NULL) && (los != NULL)) || !AGC_ASSERT((star6 >= 0) && (star6 <= STAR6_MAX)))
    { /* STARCODE is crew-keyed */
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_ram_t *ram = agc_align_ram();                          /* alignment erasables */
    const bool       csm = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* running on the CSM */

    ram->tsight     = time_cs;
    agc_status_t st = AGC_STATUS_OK; /* status of the call */

    if (csm || (star6 >= STAR6_SUN))
    {
        st = agc_align_locsam(ip, time_cs, resume_job, &ram->locsam);

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    if (starind == 0)
    {
        ram->besti = star6;
    }
    else
    {
        ram->bestj = star6;
    }

    agc_vec_t v = agc_vec_zero(); /* line of sight before CORPLAN, half-unit (2^1) */

    if (star6 == 0)
    { /* planet  */
        v = csm ? agc_vec_unit(ip, agc_vec_scale(ip, ram->starsav3, ONE_SQRT3), NULL, NULL) : agc_vec_unit(ip, ram->starad[0], NULL, NULL); /* per vehicle */
    }
    else if (star6 < STAR6_SUN)
    {
        st = agc_star_vector(star6 / 6, &v); /* NOTPLAN: CATLOG */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }
    else if (star6 == STAR6_SUN)
    {
        v = ram->locsam.vsun; /* STARAD+0 after the exchange    */
    }
    else if (star6 == STAR6_SUN + 6)
    {
        v = ram->locsam.vearth;
    }
    else
    {
        v = ram->locsam.vmoon;
    }

    *los = csm ? agc_vec_unit(ip, agc_vec_add(ip, v, ram->locsam.vel_c), NULL, NULL) : v; /* CORPLAN */

    return (st);
} /* agc_align_planet_vector */

/**********************************************************************
 * CHKSDATA / R54 angle test (Comanche pp. 759-760, Luminary pp. 945-946):
 * first pass X1 = STARAD:  18D   = ACOS(STARAD . STARAD+6)   (catalogue)
 * second pass X1 = 6D:     THETA = ACOS(6D . 12D)            (observed)
 * NORMTEM1 = ABS(THETA - 18D), revs B0.  All vectors half-unit.
 *
 * FUNCTION NAME: agc_align_chksdata_angle
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ip        agc_interp_t *   IO   interpreter state (OVFIND, MPAC words)
 *   cat       const agc_vec_t  I    STARAD, STARAD +6, half-unit
 *   obs       const agc_vec_t  I    6D, 12D, half-unit
 *
 * RETURNS: agc_dp_t -- NORMTEM1 = |THETA - 18D|, revs 2^0
 **********************************************************************/
agc_dp_t agc_align_chksdata_angle(agc_interp_t *ip, const agc_vec_t cat[2], const agc_vec_t obs[2])
{
    agc_align_ram_t *ram = agc_align_ram(); /* alignment erasables */

    /* DOT* SL1 ACOS: the SL1 acts on the TP dot product.               */
    ram->theta18  = agc_dp_acos(ip, align_dot_shl(ip, cat[0], cat[1], 1));
    ram->theta    = agc_dp_acos(ip, align_dot_shl(ip, obs[0], obs[1], 1));
    ram->normtem1 = agc_dp_abs(agc_dp_sub(ip, ram->theta, ram->theta18)); /* RTB SGNAGREE: no-op in this model */

    return (ram->normtem1);
}

/**********************************************************************
 * R51 fine-align solution (Comanche p. 757 / Luminary p. 942): AXISGEN of
 * the desired-SM catalogue pair a (STARAD, STARAD+6) against the observed
 * present-SM pair b (6D, 12D) gives XDC, the desired SM wrt the present SM.
 *
 * FUNCTION NAME: agc_align_fine_solve
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ip        agc_interp_t *   IO   interpreter state (OVFIND, MPAC words)
 *   a         const agc_vec_t  I    desired-SM catalogue pair (STARAD,
 *                                   STARAD +6), half-unit
 *   b         const agc_vec_t  I    observed present-SM pair (6D, 12D),
 *                                   half-unit
 *   xdc       agc_mat_t *      O    XDC, YDC, ZDC: desired SM wrt present
 *                                   SM, half-unit rows
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_align_fine_solve(agc_interp_t *ip, const agc_vec_t a[2], const agc_vec_t b[2], agc_mat_t *xdc)
{
    agc_align_ram_t   *ram = agc_align_ram();                /* alignment erasables */
    const agc_status_t st  = agc_imu_axisgen(ip, a, b, xdc); /* AXISGEN */

    if (st == AGC_STATUS_OK)
    {
        ram->xdc = *xdc;
    }

    return (st);
}

/**********************************************************************
 * CHKSCODE (Comanche p. 773): star codes 0..50 octal are acceptable (L+3
 * return), anything else is not (L+1).  STARCODE -0 counts as 0.
 *
 * FUNCTION NAME: agc_align_chkscode
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   starcode  agc_sp_t  I    STARCODE (bits 1-6 the code)
 *
 * RETURNS: bool -- true for an acceptable code (L+3 return)
 **********************************************************************/
bool agc_align_chkscode(agc_sp_t starcode)
{
    return ((starcode >= 0) && (starcode <= AGC_ALIGN_STAR_CODE_MAX));
}
