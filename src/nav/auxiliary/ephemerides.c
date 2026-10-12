/**********************************************************************
 * FILE NAME: ephemerides.c
 *
 * PURPOSE:
 *   LUNAR AND SOLAR EPHEMERIDES SUBROUTINES.  The two vehicles carry
 *   different programs under the same labels:
 *   CM: the moon from a ninth-degree polynomial in time whose vector
 *     coefficients VECOEM are uplinked about a centre time TIMEMO; the
 *     sun by rotating the uplinked RESO about the normal to RESO x VESO
 *     at the rate OMEGAES.  Outputs are full position vectors (moon
 *     2^29 m, sun 2^38 m) and the lunar velocity (2^7 m/cs).
 *   LM (NEVILLE, 25 Oct 67): unit vectors only, from mean-element angles
 *     with one-term periodic corrections:
 *       LOS = LOSO + LOSR T - C sin(2 pi T/365 + CARG)
 *       LOM = LOMO + LOMR T - A sin(2 pi T/27.55 + AARG)
 *                           - B sin(2 pi T/32 + BARG)
 *       LON = LONO + LONR T
 *       S = (cos LOS, cos OBL sin LOS, sin OBL sin LOS)
 *       M = (cos LOM, cos OBL sin LOM - sin OBL sin IM sin(LOM-LON),
 *            sin OBL sin LOM + cos OBL sin IM sin(LOM-LON))
 *     T in days since the TEPHEM epoch (midnight 1 July before launch).
 *     LUNPOS EQUALS LSPOS; LUNVEL is a bare RVQ ("to fool integration").
 *   agc_lsposition selects between them with agc_state_vehicle().
 *
 * AGC SOURCE:
 *   Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc pp. 785-788
 *     (LSPOS, LUNPOS, LUNVEL, SOLPOS, LSTIME, REM, VEM, RES)
 *   Luminary099 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc pp. 984-987
 *     (LSPOS, POSITA, POSITB, POSITD, POSITE, POSITF); constants in
 *     CONTROLLED_CONSTANTS.agc pp. 51-52
 *
 * EXTERNAL VARIABLES:  (AGC erasables)
 *   Name          Type             I/O  Scale     Description
 *   ------------  ---------------  ---  --------  --------------------
 *   TEPHEM        agc_tp_t         I    2^42 cs   agc_erase_tephem
 *   VECOEM ..     agc_ephem_csm_t  I/O  see type  CM uplink, module
 *   OMEGAES                                       static csm_data
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_state_vehicle     vehicle selection (state.h)
 *   agc_erase_tephem      TEPHEM erasable (state.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms.  AGC_STATUS_OVERFLOW when the CM time is outside the
 *   ephemeris fit span (|t| >= 2^26 cs); AGC_STATUS_BAD_ARG on NULL or
 *   out-of-range inputs.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called by the orbit integration (nav-integ) in job context.  LM
 *   tables "contain constants for 1969 - 1970".
 *
 * NOTES:
 *   LM LSPOS keeps GTMP, LOM, LOS, LON as the words DAD / DSU leave
 *   (signs may disagree); SIN, COS and MXV read those words, so VMOON
 *   and VSUN are bit-exact (review-nav-aux finding 8).
 *
 * ALGORITHM:
 *   See PURPOSE; each function prolog gives its AGC step.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"

/* ---------------- CM constants and uplinked data ----------------------- */
/* NINEB4 2DEC 9.0 B-4 (oct 22000 00000): 9 scale 2^4.                    */
static const agc_dp_t NINEB4 = 150994944; /* 9, scale 2^4 */

/* ONEB4 2DEC 1.0 B-4 (oct 02000 00000): 1 scale 2^4.                     */
static const agc_dp_t ONEB4 = 16777216; /* 1, scale 2^4 */

/* LSTIME's SL 16 takes the TP time (2^42 cs) to 2^26 cs: the CM
 * ephemeris is valid for |T + TEPHEM - TIMEMO| < 2^26 cs (7.77 days).    */
#define EPHEM_CSM_TSPAN ((agc_tp_t) 1 << 26) /* 2^26 cs, at 2^42 */

/* VECOEM, TIMEMO, RESO, VESO, OMEGAES: uplinked erasables (module-private
 * pending shared accessors, see docs/modules/nav-aux.md).                */
static agc_ephem_csm_t csm_data; /* CM uplink, scales in the type */

/**********************************************************************
 * AGC: none (uplink of VECOEM, TIMEMO, RESO, VESO, OMEGAES)
 *   -- store the CM ephemeris data
 *
 * FUNCTION NAME: agc_ephem_set_csm_data
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   eph       const agc_ephem_csm_t *  I    uplink (scales in the type)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (always)
 **********************************************************************/
agc_status_t agc_ephem_set_csm_data(const agc_ephem_csm_t *eph)
{
    csm_data = *eph;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (uplinked CM ephemeris data) -- read the store back
 *
 * FUNCTION NAME: agc_ephem_get_csm_data
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   out       agc_ephem_csm_t *  O    uplink (scales in the type)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (always)
 **********************************************************************/
agc_status_t agc_ephem_get_csm_data(agc_ephem_csm_t *out)
{
    *out = csm_data;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * LSTIME (Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 *   p. 786, label LSTIME) -- t = T + TEPHEM - TIMEMO
 *   (SR 14, TAD TEPHEM, DCOMP, TAD TIMEMO, DCOMP, SL 16)
 *
 * FUNCTION NAME: ephem_lstime
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state
 *   eph       const agc_ephem_csm_t *  I    TIMEMO, TP 2^42 cs
 *   tephem    agc_tp_t                 I    TEPHEM, TP 2^42 cs
 *   t_cs      agc_dp_t                 I    T, scale 2^28 cs
 *   t26       agc_dp_t *               O    t, cs scale 2^26
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_OVERFLOW when t lies
 *          outside the ephemeris fit span
 **********************************************************************/
static agc_status_t ephem_lstime(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_tp_t tephem, agc_dp_t t_cs, agc_dp_t *t26)
{
    agc_tp_t t = agc_tp_shift(ip, agc_tp_from_dp(t_cs), -14); /* 2^42 cs */

    t = agc_tp_add(ip, t, tephem); /* TAD TEPHEM     */

    t = agc_tp_add(ip, eph->timemo, -t); /* DCOMP TAD      */
    t = -t;                              /* DCOMP          */

    if (!AGC_ASSERT((t < EPHEM_CSM_TSPAN) && (t > -EPHEM_CSM_TSPAN)))
    {
        return (AGC_STATUS_OVERFLOW); /* outside the ephemeris fit span   */
    }

    *t26 = agc_tp_to_dp(agc_tp_shift(ip, t, 16)); /* SL 16: 2^26 cs */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * REM / REMA (Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 *   p. 787, labels REM, REMA) -- moon position by Horner's rule,
 *   ((a9 t + a8) t + ... ) t + a0, then VSL2: 2^31 m -> 2^29 m
 *
 * FUNCTION NAME: ephem_rem
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state
 *   eph       const agc_ephem_csm_t *  I    VECOEM (scales in the type)
 *   t26       agc_dp_t                 I    t, cs scale 2^26
 *
 * RETURNS: agc_vec_t -- moon position, earth-centred, scale 2^29 m
 **********************************************************************/
static agc_vec_t ephem_rem(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_dp_t t26)
{
    agc_vec_t r = eph->vecoem[0]; /* PDVL VECOEM, Horner sum 2^31 m */

    for (int i = 1; i < AGC_EPHEM_NCOEF; i++)
    { /* TIX,1 REMA     */
        r = agc_vec_add(ip, agc_vec_scale(ip, r, t26), eph->vecoem[i]);
    }

    return (agc_vec_shl(ip, r, 2)); /* VSL2           */
}

/**********************************************************************
 * RES (Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc p. 786,
 *   labels RES, RESA) -- sun position, RESO rotated by OMEGAES t about
 *   the normal N = UNIT(UNIT(RESO) x VESO):
 *   RES = RESO cos(w t) + (N x RESO) sin(w t)
 *
 * FUNCTION NAME: ephem_res
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state
 *   eph       const agc_ephem_csm_t *  I    RESO, VESO, OMEGAES
 *   t26       agc_dp_t                 I    t, cs scale 2^26
 *
 * RETURNS: agc_vec_t -- sun position, earth-centred, scale 2^38 m
 **********************************************************************/
static agc_vec_t ephem_res(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_dp_t t26)
{
    /* PUSH DMP OMEGAES / PUSH COS: the angle is used as its top words.  */
    agc_dp_t  ang = agc_dp_mul_trunc(t26, eph->omegaes);     /* revs 2^0    */
    agc_dp_t  cw  = agc_dp_cos(ang);                         /* scale 2^1  */
    agc_vec_t c   = agc_vec_scale(ip, eph->reso, cw);        /* 2^39 m     */
    agc_dp_t  s   = agc_dp_sin(ang);                         /* scale 2^1  */
    agc_vec_t u   = agc_vec_unit(ip, eph->reso, NULL, NULL); /* scale 2^1  */
    agc_vec_t nrm = agc_vec_cross(ip, u, eph->veso);         /* 2^10 m/cs */

    nrm          = agc_vec_unit(ip, nrm, NULL, NULL);                     /* N, scale 2^1 */
    agc_vec_t ns = agc_vec_shl(ip, agc_vec_cross(ip, nrm, eph->reso), 1); /* VXV VSL1: N x RESO, 2^38 m */
    agc_vec_t r  = agc_vec_add(ip, agc_vec_scale(ip, ns, s), c);          /* 2^39 m */

    return (agc_vec_shl(ip, r, 1)); /* VSL1: 2^38 m   */
}

/**********************************************************************
 * LUNPOS (Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc p. 786,
 *   label LUNPOS) -- moon position, earth-centred
 *
 * FUNCTION NAME: agc_ephem_csm_lunpos
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state
 *   eph       const agc_ephem_csm_t *  I    uplinked ephemeris data
 *   tephem    agc_tp_t                 I    TEPHEM, TP 2^42 cs
 *   t_cs      agc_dp_t                 I    T, scale 2^28 cs
 *   moon      agc_vec_t *              O    LUNPOS, 2^29 m
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the LSTIME status
 **********************************************************************/
agc_status_t agc_ephem_csm_lunpos(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_tp_t tephem, agc_dp_t t_cs, agc_vec_t *moon)
{
    agc_dp_t     t26 = 0;                                         /* 2^26 cs */
    agc_status_t st  = ephem_lstime(ip, eph, tephem, t_cs, &t26); /* LSTIME  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    *moon = ephem_rem(ip, eph, t26);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * LUNVEL (Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc p. 786,
 *   label LUNVEL) -- moon velocity, earth-centred
 *
 * FUNCTION NAME: agc_ephem_csm_lunvel
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state
 *   eph       const agc_ephem_csm_t *  I    uplinked ephemeris data
 *   tephem    agc_tp_t                 I    TEPHEM, TP 2^42 cs
 *   t_cs      agc_dp_t                 I    T, scale 2^28 cs
 *   vmoon     agc_vec_t *              O    LUNVEL, 2^7 m/cs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the LSTIME status
 **********************************************************************/
agc_status_t agc_ephem_csm_lunvel(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_tp_t tephem, agc_dp_t t_cs, agc_vec_t *vmoon)
{
    agc_dp_t     t26 = 0;                                         /* 2^26 cs */
    agc_status_t st  = ephem_lstime(ip, eph, tephem, t_cs, &t26); /* LSTIME  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* VEM / VEMA (Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
     *   p. 787, labels VEM, VEMA) -- moon velocity, the derivative of the
     *   REM polynomial: ((9 a9 t + 8 a8) t + ... ) t + 1 a1, multipliers n
     *   at scale 2^4 (NINEB4 counting down by ONEB4), then VSL2 */
    agc_dp_t  n = NINEB4;                               /* n, scale 2^4 */
    agc_vec_t v = agc_vec_scale(ip, eph->vecoem[0], n); /* sum, 2^9 m/cs */

    for (int i = 1; i < (AGC_EPHEM_NCOEF - 1); i++)
    { /* TIX,1 VEMA     */
        agc_vec_t acc = agc_vec_scale(ip, v, t26); /* VXSC 0D, scale of sum */

        n = agc_dp_sub(ip, n, ONEB4); /* DSU ONEB4      */
        v = agc_vec_add(ip, agc_vec_scale(ip, eph->vecoem[i], n), acc);
    }

    *vmoon = agc_vec_shl(ip, v, 2);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SOLPOS (Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc p. 786,
 *   label SOLPOS) -- sun position, earth-centred
 *
 * FUNCTION NAME: agc_ephem_csm_solpos
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state
 *   eph       const agc_ephem_csm_t *  I    uplinked ephemeris data
 *   tephem    agc_tp_t                 I    TEPHEM, TP 2^42 cs
 *   t_cs      agc_dp_t                 I    T, scale 2^28 cs
 *   sun       agc_vec_t *              O    SOLPOS, 2^38 m
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the LSTIME status
 **********************************************************************/
agc_status_t agc_ephem_csm_solpos(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_tp_t tephem, agc_dp_t t_cs, agc_vec_t *sun)
{
    if (!AGC_ASSERT((ip != NULL) && (eph != NULL) && (sun != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t     t26 = 0;                                         /* 2^26 cs */
    agc_status_t st  = ephem_lstime(ip, eph, tephem, t_cs, &t26); /* LSTIME  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    *sun = ephem_res(ip, eph, t26);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * LSPOS (Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc p. 786,
 *   label LSPOS) -- sun (AGC: 2D of the VAC) then moon (AGC: MPAC)
 *
 * FUNCTION NAME: agc_ephem_csm_lspos
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state
 *   eph       const agc_ephem_csm_t *  I    uplinked ephemeris data
 *   tephem    agc_tp_t                 I    TEPHEM, TP 2^42 cs
 *   t_cs      agc_dp_t                 I    T, scale 2^28 cs
 *   moon      agc_vec_t *              O    moon position, 2^29 m
 *   sun       agc_vec_t *              O    sun position, 2^38 m
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the LSTIME status
 **********************************************************************/
agc_status_t agc_ephem_csm_lspos(agc_interp_t *ip, const agc_ephem_csm_t *eph, agc_tp_t tephem, agc_dp_t t_cs, agc_vec_t *moon, agc_vec_t *sun)
{
    agc_dp_t     t26 = 0;                                         /* 2^26 cs */
    agc_status_t st  = ephem_lstime(ip, eph, tephem, t_cs, &t26); /* LSTIME  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    *sun  = ephem_res(ip, eph, t26); /* RES, RESA      */
    *moon = ephem_rem(ip, eph, t26); /* REM            */

    return (AGC_STATUS_OK);
}

/* ---------------- LM constants (CONTROLLED_CONSTANTS pp. 51-52) --------
 * "Tables contain constants for 1969 - 1970".  Angles in revs scale 2^0.  */
/* KONMAT rows (scale 2^1):  | 1  0   0  |
 *                           | 0  K1  K2 |
 *                           | 0  K3  K4 |                                  */
/* K1 2DEC .91745 B-1, oct 16533 30007: cos(OBL).                          */
#define LM_K1 ((agc_dp_t) 123138055) /* cos OBL, 2^1 */

/* K2 2DEC -.03571 B-1, oct 77333 56654: -sin(OBL) sin(IM).                */
#define LM_K2 ((agc_dp_t) -4792915) /* 2^1         */

/* K3 2DEC .39784 B-1, oct 06273 03275: sin(OBL).                          */
#define LM_K3 ((agc_dp_t) 53397181) /* sin OBL, 2^1 */

/* K4 2DEC .082354 B-1, oct 01242 24467: cos(OBL) sin(IM).                 */
#define LM_K4 ((agc_dp_t) 11053367) /* 2^1         */

/* CSTODAY 2DEC 8640000 B-33, oct 00020 17260: cs per day, scale 2^33.     */
static const agc_dp_t CSTODAY = 270000; /* cs/day, scale 2^33 */

/* RATESP / VAL67 rows for the three angles, in the AGC's X2 / X1 order. */
#define LM_NANGLE 3 /* LOM, LOS, LON (STMP, STMP+2, STMP+4)      */
#define LM_NTERM  3 /* A (moon), B (moon), C (sun) corrections   */

/* Rates, 2DEC x B+4: rev/day scale 2^-4.                                  */
static const agc_dp_t LM_RATE[LM_NANGLE] = { /* rev/day, scale 2^-4 */
    157200012, /* LOMR .03660098 rev/day,   oct 22572 27214 */
    11758719,  /* LOSR .00273779 rev/day,   oct 01315 26177 */
    -632176,   /* LONR -.00014719 rev/day,  oct 77731 55217 */
};

/* Epoch values at TEPHEM, revs scale 2^0.                                 */
static const agc_dp_t LM_ANGLE0[LM_NANGLE] = { /* rev, scale 2^0 */
    218850686, /* LOMO .815282336 rev, oct 32055 22576 */
    73732485,  /* LOSO .274674910 rev, oct 10624 10605 */
    264733597, /* LONO .986209499 rev, oct 37436 01635 */
};

/* Periodic terms: amplitude (rev, x B+1: scale 2^-1), phase (rev 2^0),
 * frequency (1/period in 1/day, x B+1: scale 2^-1).                       */
typedef struct lm_term
{
    agc_dp_t mod;  /* amplitude, rev scale 2^-1    */
    agc_dp_t arg;  /* phase, rev scale 2^0         */
    agc_dp_t freq; /* 1/period, 1/day scale 2^-1   */
} lm_term_t;

static const lm_term_t LM_TERM[LM_NTERM] = { /* A, B, C; scales above */
    /* AMOD .01726666666 B+1* (oct 01065 31323), AARG .530784445
     * (oct 20770 13725), 1/27 .036291712 B+1 (oct 02245 06475) = 1/27.5545 */
    {9269971, 142481365, 19483965},

    /* BMOD .003505277 B+1 (oct 00162 33431), BARG .585365625
     * (oct 22566 24130), 1/32 .03125 B+1 (oct 02000 00000)                 */
    {1881881, 157132888, 16777216},

    /* CMOD .005325277 B+1 (oct 00256 17752), CARG -.01106341036
     * (oct 77512 67453), 1/365 .002737925 B+1 (oct 00131 26730)            */
    {2858986, -2969812, 1469912},
};

/**********************************************************************
 * POSITB (Luminary099 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 *   pp. 985-986, label POSITB) -- one periodic correction added to GTMP:
 *   GTMP + MOD sin(2 pi (T FREQ) + ARG)
 *
 * FUNCTION NAME: ephem_lm_term
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state
 *   timep     agc_dp_words_t  I    TIMEP, days scale 2^9
 *   k         int             I    term: 0 = A, 1 = B, 2 = C
 *   gtmp      agc_dp_words_t  I    GTMP before the term, rev 2^0
 *
 * RETURNS: agc_dp_words_t -- GTMP as stored, revs scale 2^0.  DMP*, SL 8
 *          on the TP (wraps mod 1 rev), DAD* onto its top two words, SIN
 *          of those words, DMP*, DAD GTMP: the words may disagree in sign.
 **********************************************************************/
static agc_dp_words_t ephem_lm_term(agc_interp_t *ip, agc_dp_words_t timep, int k, agc_dp_words_t gtmp)
{
    agc_tp_words_t m = agc_mpac_dmp(agc_mpac_dload(timep), agc_dp_to_words(LM_TERM[k].freq)); /* rev, 2^8 */

    m = agc_mpac_dad(ip, agc_mpac_shift(ip, m, 8), agc_dp_to_words(LM_TERM[k].arg)); /* SL 8 DAD*, wraps */
    m = agc_mpac_dmp(agc_mpac_sin(m), agc_dp_to_words(LM_TERM[k].mod));              /* SIN DMP* */

    return (agc_mpac_store(agc_mpac_dad(ip, m, gtmp))); /* DAD GTMP, STORE GTMP */
}

/**********************************************************************
 * POSITD (Luminary099 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 *   p. 986, label POSITD) -- ANGLE0 + RATE T - GTMP
 *
 * FUNCTION NAME: ephem_lm_angle
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state
 *   timep     agc_dp_words_t  I    TIMEP, days scale 2^9
 *   j         int             I    angle: 0 = LOM, 1 = LOS, 2 = LON
 *   gtmp      agc_dp_words_t  I    GTMP periodic terms, rev 2^0
 *
 * RETURNS: agc_dp_words_t -- STMP as stored, revs scale 2^0: DMP*, SL 5
 *          on the TP, DAD* and DSU on its top two words (all wrap mod
 *          1 rev)
 **********************************************************************/
static agc_dp_words_t ephem_lm_angle(agc_interp_t *ip, agc_dp_words_t timep, int j, agc_dp_words_t gtmp)
{
    agc_tp_words_t m = agc_mpac_dmp(agc_mpac_dload(timep), agc_dp_to_words(LM_RATE[j])); /* rev, 2^5 */

    m = agc_mpac_dad(ip, agc_mpac_shift(ip, m, 5), agc_dp_to_words(LM_ANGLE0[j])); /* SL 5 DAD* */

    return (agc_mpac_store(agc_mpac_dsu(ip, m, gtmp))); /* DSU GTMP, STORE STMP */
}

/**********************************************************************
 * POSITF (Luminary099 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 *   p. 986, SIN/COS PDDL, VDEF MXV KONMAT UNIT) -- rotate by the
 *   obliquity matrix
 *     KONMAT rows (scale 2^1):  | 1  0   0  |
 *                               | 0  K1  K2 |
 *                               | 0  K3  K4 |
 *
 * FUNCTION NAME: ephem_lm_konmat
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state
 *   l         agc_dp_words_t  I    LOM or LOS as stored, rev 2^0
 *   z         agc_dp_words_t  I    sin(LOM-LON) or 0 as pushed, 2^1
 *
 * RETURNS: agc_vec_t -- unit vector, scale 2^1 (UNIT of the MXV words)
 **********************************************************************/
static agc_vec_t ephem_lm_konmat(agc_interp_t *ip, agc_dp_words_t l, agc_dp_words_t z)
{
    const agc_vec_words_t konmat[3] = {{{agc_dp_to_words(AGC_DP_HALF), agc_dp_to_words(0), agc_dp_to_words(0)}}, /* 1.0 B-1, 0, 0 */
                                       {{agc_dp_to_words(0), agc_dp_to_words(LM_K1), agc_dp_to_words(LM_K2)}},
                                       {{agc_dp_to_words(0), agc_dp_to_words(LM_K3), agc_dp_to_words(LM_K4)}}};
    agc_vec_words_t       v; /* VDEF: COS L, SIN L, z; 2^1 */

    v.c[0] = agc_mpac_store(agc_mpac_cos(agc_mpac_dload(l)));
    v.c[1] = agc_mpac_store(agc_mpac_sin(agc_mpac_dload(l)));
    v.c[2] = z;

    return (agc_vec_unit(ip, agc_vec_from_words(agc_mpac_mxv(ip, konmat, v)), NULL, NULL));
}

/**********************************************************************
 * LSPOS (Luminary099 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 *   pp. 985-987, labels LSPOS, POSITA, POSITB, POSITD, POSITE, POSITF)
 *   -- VMOON and VSUN in the basic reference frame
 *
 * FUNCTION NAME: agc_ephem_lm_lspos
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state
 *   tephem    agc_tp_t        I    TEPHEM, TP 2^42 cs
 *   t_cs      agc_dp_t        I    T, scale 2^28 cs
 *   vmoon     agc_vec_t *     O    VMOON, unit vector scale 2^1
 *   vsun      agc_vec_t *     O    VSUN, unit vector scale 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 *
 * TIMEP = (T + TEPHEM) / CSTODAY, days scale 2^9 (DDV takes only the DP
 * part of the TP dividend, so T + TEPHEM is truncated to 2^14 cs).  The
 * AGC's loop over FREEFLAG / X1 / X2 (moon: terms A and B; sun: term C;
 * node: no term) is written out as straight-line calls.  GTMP and STMP
 * are kept as the words the AGC stores (DAD / DSU leave them free to
 * disagree in sign), which SIN and COS then read.
 **********************************************************************/
agc_status_t agc_ephem_lm_lspos(agc_interp_t *ip, agc_tp_t tephem, agc_dp_t t_cs, agc_vec_t *vmoon, agc_vec_t *vsun)
{
    const agc_dp_words_t zero = agc_dp_to_words(0);                                             /* KONMAT +2: ZEROS */
    agc_tp_words_t       t    = agc_mpac_shift(ip, agc_mpac_dload(agc_dp_to_words(t_cs)), -14); /* SR 14D, 2^42 cs */

    t = agc_mpac_tad(ip, t, agc_tp_to_words(tephem)); /* TAD TEPHEM */

    const agc_dp_words_t timep = agc_mpac_store(agc_mpac_ddv(ip, t, agc_dp_to_words(CSTODAY))); /* DDV CSTODAY, 2^9 day */

    /* 1st pass (X2 = 0): LOM with the A and B terms (FREEFLAG loop).      */
    agc_dp_words_t       g   = ephem_lm_term(ip, timep, 1, ephem_lm_term(ip, timep, 0, zero)); /* GTMP, 2^0 rev */
    const agc_dp_words_t lom = ephem_lm_angle(ip, timep, 0, g);                                /* LOM, 2^0 rev */

    /* 2nd pass (X2 = -2): LOS with the C term.                            */
    g                        = ephem_lm_term(ip, timep, 2, zero);
    const agc_dp_words_t los = ephem_lm_angle(ip, timep, 1, g); /* LOS, 2^0 rev */

    /* 3rd pass (X2 = -4, POSITE): LON, no periodic term.                 */
    const agc_dp_words_t lon = ephem_lm_angle(ip, timep, 2, zero); /* LON, 2^0 rev */

    /* POSITF: DLOAD DSU SIN PDDL (sin(LOM - LON)), moon then sun.         */
    const agc_dp_words_t sdl = agc_mpac_store(agc_mpac_sin(agc_mpac_dsu(ip, agc_mpac_dload(lom), lon))); /* 2^1 */

    *vmoon = ephem_lm_konmat(ip, lom, sdl);
    *vsun  = ephem_lm_konmat(ip, los, zero);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * LSPOS (LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc, CM p. 786, LM
 *   pp. 985-987) -- the nav-integ contract, dispatching on
 *   agc_state_vehicle()
 *
 * FUNCTION NAME: agc_lsposition
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state
 *   time_cs   agc_dp_t        I    ground elapsed time, scale 2^28 cs
 *   moon_pos  agc_vec_t *     O    CSM: moon 2^29 m; LM: VMOON 2^1
 *   sun_pos   agc_vec_t *     O    CSM: sun 2^38 m; LM: VSUN 2^1
 *
 * RETURNS: agc_status_t -- status of the vehicle's LSPOS.  TEPHEM comes
 *          from the shared erasable (agc_erase_tephem, set by P11 or
 *          agc_navaux_set_padload).
 **********************************************************************/
agc_status_t agc_lsposition(agc_interp_t *ip, agc_dp_t time_cs, agc_vec_t *moon_pos, agc_vec_t *sun_pos)
{
    agc_tp_t tephem = agc_erase_tephem(); /* TEPHEM, scale 2^42 cs */

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return (agc_ephem_lm_lspos(ip, tephem, time_cs, moon_pos, sun_pos));
    }

    return (agc_ephem_csm_lspos(ip, &csm_data, tephem, time_cs, moon_pos, sun_pos));
}
