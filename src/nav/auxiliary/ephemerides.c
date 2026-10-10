/*
 * ephemerides.c -- LUNAR AND SOLAR EPHEMERIDES SUBROUTINES
 *
 * The two vehicles carry different programs under the same labels:
 *
 *  CM (Comanche055 pp. 785-788): the moon from a ninth-degree polynomial
 *     in time whose vector coefficients VECOEM are uplinked about a centre
 *     time TIMEMO; the sun by rotating the uplinked RESO about the normal
 *     to RESO x VESO at the rate OMEGAES.  Outputs are full position
 *     vectors (moon 2^29 m, sun 2^38 m) and the lunar velocity (2^7 m/cs).
 *
 *  LM (Luminary099 pp. 984-987, NEVILLE, 25 Oct 67): unit vectors only,
 *     from mean-element angles with one-term periodic corrections:
 *       LOS = LOSO + LOSR T - C sin(2 pi T/365 + CARG)
 *       LOM = LOMO + LOMR T - A sin(2 pi T/27.55 + AARG) - B sin(2 pi T/32 + BARG)
 *       LON = LONO + LONR T
 *       S = (cos LOS, cos OBL sin LOS, sin OBL sin LOS)
 *       M = (cos LOM, cos OBL sin LOM - sin OBL sin IM sin(LOM-LON),
 *            sin OBL sin LOM + cos OBL sin IM sin(LOM-LON))
 *     T in days since TEPHEM epoch (midnight 1 July before launch).
 *     LUNPOS EQUALS LSPOS; LUNVEL is a bare RVQ ("to fool integration").
 *
 * agc_lsposition selects between them with agc_state_vehicle().
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* ---------------- CM constants and uplinked data ----------------------- */
/* NINEB4 2DEC 9.0 B-4 (oct 22000 00000): 9 scale 2^4.                    */
static const agc_dp_t NINEB4 = 150994944;
/* ONEB4 2DEC 1.0 B-4 (oct 02000 00000): 1 scale 2^4.                     */
static const agc_dp_t ONEB4 = 16777216;
/* LSTIME's SL 16 takes the TP time (2^42 cs) to 2^26 cs: the CM
 * ephemeris is valid for |T + TEPHEM - TIMEMO| < 2^26 cs (7.77 days).    */
#define EPHEM_CSM_TSPAN ((agc_tp_t)1 << 26)

/* VECOEM, TIMEMO, RESO, VESO, OMEGAES: uplinked erasables (module-private
 * pending shared accessors, see docs/modules/nav-aux.md).                */
static agc_ephem_csm_t csm_data;

agc_status_t agc_ephem_set_csm_data(const agc_ephem_csm_t *eph)
{
    if (!AGC_ASSERT(eph != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_navaux_tp_ok(eph->timemo) && agc_navaux_vec_ok(eph->reso) &&
                    agc_navaux_vec_ok(eph->veso))) {
        return AGC_STATUS_BAD_ARG;
    }
    csm_data = *eph;
    return AGC_STATUS_OK;
}

agc_status_t agc_ephem_get_csm_data(agc_ephem_csm_t *out)
{
    if (!AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    *out = csm_data;
    return AGC_STATUS_OK;
}

/*
 * LSTIME (CM p. 786): t = T + TEPHEM - TIMEMO, cs scale 2^26.
 * SR 14, TAD TEPHEM, DCOMP, TAD TIMEMO, DCOMP, SL 16.
 */
static agc_status_t ephem_lstime(agc_interp_t *ip, const agc_ephem_csm_t *eph,
                                 agc_tp_t tephem, agc_dp_t t_cs, agc_dp_t *t26)
{
    if (!AGC_ASSERT((ip != NULL) && (eph != NULL) && (t26 != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_navaux_tp_ok(tephem) && agc_navaux_dp_ok(t_cs))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_tp_t t = agc_tp_add(ip, agc_navaux_tp_from_cs(t_cs), tephem);
    t = agc_tp_add(ip, eph->timemo, -t);                 /* DCOMP TAD      */
    t = -t;                                              /* DCOMP          */
    if (!AGC_ASSERT((t < EPHEM_CSM_TSPAN) && (t > -EPHEM_CSM_TSPAN))) {
        return AGC_STATUS_OVERFLOW;    /* outside the ephemeris fit span   */
    }
    /* SL 16: 2^42 cs -> 2^26 cs; done as TP -> DP (2^28 cs), SL 2.       */
    *t26 = agc_dp_shl(ip, agc_navaux_cs_from_tp(t), 2);
    return AGC_STATUS_OK;
}

/*
 * REM / REMA (CM p. 787): moon position by Horner's rule,
 * ((a9 t + a8) t + ... ) t + a0, then VSL2: 2^31 m -> 2^29 m.
 */
static agc_vec_t ephem_rem(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_dp_t t26)
{
    if (!AGC_ASSERT((ip != NULL) && (eph != NULL))) {
        return agc_vec_zero();
    }
    if (!AGC_ASSERT(agc_navaux_dp_ok(t26))) {
        return agc_vec_zero();
    }
    agc_vec_t r = eph->vecoem[0];                        /* PDVL VECOEM    */
    for (int i = 1; i < AGC_EPHEM_NCOEF; i++) {          /* TIX,1 REMA     */
        r = agc_vec_add(ip, agc_vec_scale(ip, r, t26), eph->vecoem[i]);
    }
    return agc_vec_shl(ip, r, 2);                        /* VSL2           */
}

/*
 * VEM / VEMA (CM p. 787): moon velocity, the derivative of the REM
 * polynomial: ((9 a9 t + 8 a8) t + ... ) t + 1 a1, multipliers n at
 * scale 2^4 (NINEB4 counting down by ONEB4), then VSL2 -> 2^7 m/cs.
 */
static agc_vec_t ephem_vem(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_dp_t t26)
{
    if (!AGC_ASSERT((ip != NULL) && (eph != NULL))) {
        return agc_vec_zero();
    }
    if (!AGC_ASSERT(agc_navaux_dp_ok(t26))) {
        return agc_vec_zero();
    }
    agc_dp_t n = NINEB4;
    agc_vec_t v = agc_vec_scale(ip, eph->vecoem[0], n);  /* VXSC VECOEM    */
    for (int i = 1; i < (AGC_EPHEM_NCOEF - 1); i++) {    /* TIX,1 VEMA     */
        agc_vec_t acc = agc_vec_scale(ip, v, t26);       /* VXSC 0D        */
        n = agc_dp_sub(ip, n, ONEB4);                    /* DSU ONEB4      */
        v = agc_vec_add(ip, agc_vec_scale(ip, eph->vecoem[i], n), acc);
    }
    return agc_vec_shl(ip, v, 2);                        /* VSL2           */
}

/*
 * RES (CM p. 786): sun position, RESO rotated by OMEGAES t about the
 * normal N = UNIT(UNIT(RESO) x VESO):
 *   RES = RESO cos(w t) + (N x RESO) sin(w t),  2^38 m.
 */
static agc_vec_t ephem_res(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_dp_t t26)
{
    if (!AGC_ASSERT((ip != NULL) && (eph != NULL))) {
        return agc_vec_zero();
    }
    if (!AGC_ASSERT(agc_navaux_dp_ok(t26))) {
        return agc_vec_zero();
    }
    agc_dp_t ang = agc_dp_mul(ip, t26, eph->omegaes);    /* revs 2^0       */
    agc_vec_t c = agc_vec_scale(ip, eph->reso, agc_dp_cos(ip, ang));  /* 2^39 */
    agc_dp_t s = agc_dp_sin(ip, ang);                    /* sin/2          */
    agc_vec_t u = agc_vec_unit(ip, eph->reso, NULL, NULL);
    agc_vec_t nrm = agc_vec_unit(ip, agc_vec_cross(ip, u, eph->veso), NULL, NULL);
    agc_vec_t ns = agc_vec_shl(ip, agc_vec_cross(ip, nrm, eph->reso), 1); /* VXV VSL1 */
    agc_vec_t r = agc_vec_add(ip, agc_vec_scale(ip, ns, s), c);          /* VXSC VAD */
    return agc_vec_shl(ip, r, 1);                        /* VSL1: 2^38 m   */
}

/* LUNPOS (CM p. 786): moon position, earth-centred, 2^29 m. */
agc_status_t agc_ephem_csm_lunpos(agc_interp_t *ip, const agc_ephem_csm_t *eph,
                                  agc_tp_t tephem, agc_dp_t t_cs, agc_vec_t *moon)
{
    if (!AGC_ASSERT((ip != NULL) && (eph != NULL) && (moon != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t t26 = 0;
    agc_status_t st = ephem_lstime(ip, eph, tephem, t_cs, &t26);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    *moon = ephem_rem(ip, eph, t26);
    return AGC_STATUS_OK;
}

/* LUNVEL (CM p. 786): moon velocity, earth-centred, 2^7 m/cs. */
agc_status_t agc_ephem_csm_lunvel(agc_interp_t *ip, const agc_ephem_csm_t *eph,
                                  agc_tp_t tephem, agc_dp_t t_cs, agc_vec_t *vmoon)
{
    if (!AGC_ASSERT((ip != NULL) && (eph != NULL) && (vmoon != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t t26 = 0;
    agc_status_t st = ephem_lstime(ip, eph, tephem, t_cs, &t26);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    *vmoon = ephem_vem(ip, eph, t26);
    return AGC_STATUS_OK;
}

/* SOLPOS (CM p. 786): sun position, earth-centred, 2^38 m. */
agc_status_t agc_ephem_csm_solpos(agc_interp_t *ip, const agc_ephem_csm_t *eph,
                                  agc_tp_t tephem, agc_dp_t t_cs, agc_vec_t *sun)
{
    if (!AGC_ASSERT((ip != NULL) && (eph != NULL) && (sun != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t t26 = 0;
    agc_status_t st = ephem_lstime(ip, eph, tephem, t_cs, &t26);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    *sun = ephem_res(ip, eph, t26);
    return AGC_STATUS_OK;
}

/* LSPOS (CM p. 786): sun (AGC: 2D of the VAC) then moon (AGC: MPAC). */
agc_status_t agc_ephem_csm_lspos(agc_interp_t *ip, const agc_ephem_csm_t *eph,
                                 agc_tp_t tephem, agc_dp_t t_cs,
                                 agc_vec_t *moon, agc_vec_t *sun)
{
    if (!AGC_ASSERT((ip != NULL) && (eph != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT((moon != NULL) && (sun != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t t26 = 0;
    agc_status_t st = ephem_lstime(ip, eph, tephem, t_cs, &t26);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    *sun = ephem_res(ip, eph, t26);                      /* RES, RESA      */
    *moon = ephem_rem(ip, eph, t26);                     /* REM            */
    return AGC_STATUS_OK;
}

/* ---------------- LM constants (CONTROLLED_CONSTANTS pp. 51-52) --------
 * "Tables contain constants for 1969 - 1970".  Angles in revs scale 2^0.  */
/* KONMAT rows (scale 2^1):  | 1  0   0  |
 *                           | 0  K1  K2 |
 *                           | 0  K3  K4 |                                  */
/* K1 2DEC .91745 B-1, oct 16533 30007: cos(OBL).                          */
#define LM_K1 ((agc_dp_t)123138055)
/* K2 2DEC -.03571 B-1, oct 77333 56654: -sin(OBL) sin(IM).                */
#define LM_K2 ((agc_dp_t)-4792915)
/* K3 2DEC .39784 B-1, oct 06273 03275: sin(OBL).                          */
#define LM_K3 ((agc_dp_t)53397181)
/* K4 2DEC .082354 B-1, oct 01242 24467: cos(OBL) sin(IM).                 */
#define LM_K4 ((agc_dp_t)11053367)
/* CSTODAY 2DEC 8640000 B-33, oct 00020 17260: cs per day, scale 2^33.     */
static const agc_dp_t CSTODAY = 270000;

/* RATESP / VAL67 rows for the three angles, in the AGC's X2 / X1 order. */
#define LM_NANGLE 3          /* LOM, LOS, LON (STMP, STMP+2, STMP+4)      */
#define LM_NTERM  3          /* A (moon), B (moon), C (sun) corrections   */
/* Rates, 2DEC x B+4: rev/day scale 2^-4.                                  */
static const agc_dp_t LM_RATE[LM_NANGLE] = {
    157200012,   /* LOMR .03660098 rev/day,   oct 22572 27214 */
    11758719,    /* LOSR .00273779 rev/day,   oct 01315 26177 */
    -632176,     /* LONR -.00014719 rev/day,  oct 77731 55217 */
};
/* Epoch values at TEPHEM, revs scale 2^0.                                 */
static const agc_dp_t LM_ANGLE0[LM_NANGLE] = {
    218850686,   /* LOMO .815282336 rev, oct 32055 22576 */
    73732485,    /* LOSO .274674910 rev, oct 10624 10605 */
    264733597,   /* LONO .986209499 rev, oct 37436 01635 */
};
/* Periodic terms: amplitude (rev, x B+1: scale 2^-1), phase (rev 2^0),
 * frequency (1/period in 1/day, x B+1: scale 2^-1).                       */
typedef struct lm_term {
    agc_dp_t mod;
    agc_dp_t arg;
    agc_dp_t freq;
} lm_term_t;
static const lm_term_t LM_TERM[LM_NTERM] = {
    /* AMOD .01726666666 B+1* (oct 01065 31323), AARG .530784445
     * (oct 20770 13725), 1/27 .036291712 B+1 (oct 02245 06475) = 1/27.5545 */
    { 9269971, 142481365, 19483965 },
    /* BMOD .003505277 B+1 (oct 00162 33431), BARG .585365625
     * (oct 22566 24130), 1/32 .03125 B+1 (oct 02000 00000)                 */
    { 1881881, 157132888, 16777216 },
    /* CMOD .005325277 B+1 (oct 00256 17752), CARG -.01106341036
     * (oct 77512 67453), 1/365 .002737925 B+1 (oct 00131 26730)            */
    { 2858986, -2969812, 1469912 },
};

/* POSITB (LM p. 985-986): one periodic correction,
 * MOD sin(2 pi (T FREQ) + ARG): DMP*, SL 8 (wrap), DAD* (wrap), SIN, DMP*.
 * T in days scale 2^9; result revs scale 2^0.
 * DEVIATION: the product is kept as exact TP through SL 8 and the SL/DAD
 * overflow is modelled as wrap mod 1 rev (agc_navaux_rev_*), as NEWANGLE. */
static agc_dp_t ephem_lm_term(agc_interp_t *ip, agc_dp_t timep, int k)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return 0;
    }
    if (!AGC_ASSERT((k >= 0) && (k < LM_NTERM))) {
        return 0;
    }
    agc_dp_t x = agc_navaux_rev_tp_shl(agc_dp_mul_tp(timep, LM_TERM[k].freq), 8);
    x = agc_navaux_rev_add(x, LM_TERM[k].arg);
    return agc_dp_mul(ip, agc_dp_sin(ip, x), LM_TERM[k].mod);
}

/* POSITD (LM p. 986): ANGLE0 + RATE T (SL 5, wrap) - GTMP.               */
static agc_dp_t ephem_lm_angle(agc_dp_t timep, int j, agc_dp_t gtmp)
{
    if (!AGC_ASSERT((j >= 0) && (j < LM_NANGLE))) {
        return 0;
    }
    if (!AGC_ASSERT(agc_navaux_dp_ok(gtmp))) {
        return 0;
    }
    agc_dp_t x = agc_navaux_rev_tp_shl(agc_dp_mul_tp(timep, LM_RATE[j]), 5);
    x = agc_navaux_rev_add(x, LM_ANGLE0[j]);             /* DAD* RATESP+6  */
    return agc_navaux_rev_add(x, agc_dp_neg(gtmp));      /* DSU GTMP       */
}

/* VDEF then MXV KONMAT, UNIT (LM p. 986): (x, y, z) half-sines/cosines
 * (scale 2^1) through the obliquity matrix, unit result scale 2^1.       */
static agc_vec_t ephem_lm_konmat(agc_interp_t *ip, agc_dp_t x, agc_dp_t y, agc_dp_t z)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return agc_vec_zero();
    }
    if (!AGC_ASSERT(agc_navaux_dp_ok(x) && agc_navaux_dp_ok(y) && agc_navaux_dp_ok(z))) {
        return agc_vec_zero();
    }
    agc_mat_t konmat;
    konmat.row[0] = agc_vec_make(AGC_DP_HALF, 0, 0);     /* 1.0 B-1, 0, 0  */
    konmat.row[1] = agc_vec_make(0, LM_K1, LM_K2);
    konmat.row[2] = agc_vec_make(0, LM_K3, LM_K4);
    agc_vec_t v = agc_mat_vec(ip, &konmat, agc_vec_make(x, y, z));
    return agc_vec_unit(ip, v, NULL, NULL);
}

/*
 * LSPOS (Luminary099 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 * pp. 985-987, labels LSPOS POSITA POSITB POSITD POSITE POSITF):
 * VMOON and VSUN, unit vectors scale 2^1, basic reference frame.
 * TIMEP = (T + TEPHEM) / CSTODAY, days scale 2^9 (DDV takes only the DP
 * part of the TP dividend, so T + TEPHEM is truncated to 2^14 cs).
 * The AGC's loop over FREEFLAG / X1 / X2 (moon: terms A and B; sun:
 * term C; node: no term) is written out as straight-line calls.
 */
agc_status_t agc_ephem_lm_lspos(agc_interp_t *ip, agc_tp_t tephem, agc_dp_t t_cs,
                                agc_vec_t *vmoon, agc_vec_t *vsun)
{
    if (!AGC_ASSERT((ip != NULL) && (vmoon != NULL) && (vsun != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_navaux_tp_ok(tephem) && agc_navaux_dp_ok(t_cs))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_tp_t t = agc_tp_add(ip, agc_navaux_tp_from_cs(t_cs), tephem);  /* SR TAD */
    agc_dp_t timep = agc_dp_div(ip, agc_tp_to_dp(t), CSTODAY);         /* DDV    */
    /* 1st pass (X2 = 0): LOM with the A and B terms (FREEFLAG loop).      */
    agc_dp_t g = ephem_lm_term(ip, timep, 0);            /* GTMP = 0 + A term */
    g = agc_dp_add(ip, ephem_lm_term(ip, timep, 1), g);  /* DAD GTMP: + B    */
    agc_dp_t lom = ephem_lm_angle(timep, 0, g);
    /* 2nd pass (X2 = -2): LOS with the C term.                            */
    agc_dp_t los = ephem_lm_angle(timep, 1, ephem_lm_term(ip, timep, 2));
    /* 3rd pass (X2 = -4, POSITE): LON, no periodic term.                 */
    agc_dp_t lon = ephem_lm_angle(timep, 2, 0);
    /* POSITF: moon then sun.                                              */
    agc_dp_t dl = agc_navaux_rev_add(lom, agc_dp_neg(lon));            /* DSU    */
    *vmoon = ephem_lm_konmat(ip, agc_dp_cos(ip, lom), agc_dp_sin(ip, lom),
                             agc_dp_sin(ip, dl));
    *vsun = ephem_lm_konmat(ip, agc_dp_cos(ip, los), agc_dp_sin(ip, los), 0);
    return AGC_STATUS_OK;
}

/*
 * LSPOS contract for nav-integ: dispatches on agc_state_vehicle().
 *   CSM: *moon_pos 2^29 m, *sun_pos 2^38 m (earth-centred, LUNPOS/SOLPOS)
 *   LM:  *moon_pos = VMOON, *sun_pos = VSUN, unit vectors scale 2^1
 * TEPHEM comes from the shared erasable (agc_erase_tephem, set by P11).
 * DEVIATION: the other nav-aux entry points take TEPHEM from
 * agc_navaux_padload_t; the two must be kept in step (see module note).
 */
agc_status_t agc_lsposition(agc_interp_t *ip, agc_dp_t time_cs,
                            agc_vec_t *moon_pos, agc_vec_t *sun_pos)
{
    if (!AGC_ASSERT((ip != NULL) && (moon_pos != NULL) && (sun_pos != NULL))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(agc_navaux_dp_ok(time_cs))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_tp_t tephem = agc_erase_tephem();
    if (agc_state_vehicle() == AGC_VEHICLE_LM) {
        return agc_ephem_lm_lspos(ip, tephem, time_cs, moon_pos, sun_pos);
    }
    return agc_ephem_csm_lspos(ip, &csm_data, tephem, time_cs, moon_pos, sun_pos);
}
