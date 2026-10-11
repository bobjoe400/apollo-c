/*
 * geometry.c -- CSM_GEOMETRY.agc (Comanche055 pp. 285-296): sextant
 * line of sight in navigation-base axes (SXTNB, SXTLOGIC), desired optics
 * angles for a star (CALCSXA, SXTANG, SXTANG1), the LOG subroutine and the
 * earth-rotation of a target vector (EARROT1, EARROT2, OVERADAY).
 * SVDWN1/SVDWN2 of the same file are in src/nav/integration/svdwn.c.
 *
 * Frames: NB = navigation base; NB1 = optics base ("NB2NB1": NB tilted
 * 32.5 degrees about Y; the sextant shaft axis lies along ZNB1).
 * Optics angles: SA = shaft, TA = trunnion.  The trunnion CDU counts
 * 4 TA - 19.775 deg x 4, hence the SXTLOGIC offset and scaling.
 */
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/programs/alignment.h"

/* NB2NB1 / NB1NB2 elements, 2DEC ... B-1 (cos 32.523 deg, sin 32.523 deg). */
#define NB_COS  ((agc_dp_t) 113169126) /* .8431756920 B-1                 */
#define NB_SIN  ((agc_dp_t) 72160568)  /* .5376381241 B-1                 */
#define NB_HALF AGC_DP_HALF            /* HALFNFLT .5                     */
/* 10DEGS- DEC 3600 (added to the MPAC high word): 79.1 deg = 4 x 19.775.  */
#define TEN_DEGS_MINUS ((agc_dp_t) (3600 * 16384))
/* 20DEG- DEC 03600, DEC 00000: the same offset as a DP.                    */
#define TWENTY_DEG_MINUS ((agc_dp_t) (3600 * 16384))
/* 270DEG OCT 60000: shaft 270 degrees, 2's complement CDU word.          */
#define SHAFT_270DEG ((uint16_t) 060000)
/* 20DEGS- DEC -07199, DEC -00000 (PAC high word), as a raw 1's complement
 * word: 077777 - 7199.                                                    */
#define PAC_20DEGS_MINUS ((uint16_t) (077777 - 7199))
/* LOG: NEARLY1 .999999999, CLOG2/32 .0216608494 = ln 2 / 32.              */
#define NEARLY1  ((agc_dp_t) 268435456 - 1)
#define CLOG2_32 ((agc_dp_t) 5814540)
/* POLY coefficients a0..a3 (2DEC 0, .031335467, .0130145859, .0215738898). */
#define LOG_A1 ((agc_dp_t) 8411550)
#define LOG_A2 ((agc_dp_t) 3493576)
#define LOG_A3 ((agc_dp_t) 5791197)
/* 1/WIE 2DEC 8616410: one sidereal day in cs (B28 cs, i.e. raw cs).      */
#define ONE_OVER_WIE ((agc_dp_t) 8616410)
#define OVERADAY_MAX 32 /* DTEAROT < 2^28 cs = 31.2 sidereal days        */

/* NB2NB1 (NB axes -> optics base NB1), elements B-1. */
static agc_mat_t nb2nb1(void)
{
    agc_mat_t m;

    m.row[0] = agc_vec_make(NB_COS, 0, -NB_SIN);
    m.row[1] = agc_vec_make(0, NB_HALF, 0);
    m.row[2] = agc_vec_make(NB_SIN, 0, NB_COS);

    if (!AGC_ASSERT(m.row[1].c[1] == AGC_DP_HALF))
    {
        return ((agc_mat_t){0}); /* no valid result */
    }

    _Static_assert(NB_COS > NB_SIN, "AGC constant");

    return (m);
}

/* NB1NB2 (optics base NB1 -> NB), the transpose of NB2NB1. */
static agc_mat_t nb1nb2(void)
{
    const agc_mat_t m = nb2nb1();

    if (!AGC_ASSERT(m.row[0].c[2] == -NB_SIN))
    {
        return ((agc_mat_t){0}); /* no valid result */
    }

    if (!AGC_ASSERT(m.row[2].c[0] == NB_SIN))
    {
        return ((agc_mat_t){0}); /* no valid result */
    }

    return (agc_mat_transpose(&m));
}

/*
 * SXTLOGIC (CSM_GEOMETRY.agc p. 285) -- the trunnion angle in revs from
 * the trunnion CDU: (CDULOGIC(TA) + 10DEGS-) x 1/4 (SHORTMP by QUARTER;
 * the third word is dropped by DANZIG, i.e. a truncating shift).
 */
agc_dp_t agc_csm_sxtlogic(uint16_t trunnion_cdu)
{
    if (!AGC_ASSERT(trunnion_cdu <= 077777u))
    {
        return (0); /* no valid result */
    }

    agc_ovf_t      ovf = 0;
    const agc_dp_t a   = agc_dp_add_raw(agc_align_cdulogic(trunnion_cdu),
                                        TEN_DEGS_MINUS, &ovf); /* ADS MPAC */

    if (!AGC_ASSERT(ovf == 0))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_shr(a, 2)); /* x QUARTER */
}

/*
 * SXTNB (CSM_GEOMETRY.agc p. 285) -- the sextant line of sight in NB axes
 * from the mark's trunnion (TA) and shaft (SA) CDU words:
 *   STARM = (cos SA sin TA, sin SA sin TA, cos TA)/2   (optics base)
 *   result = NB1NB2 STARM, VSL1  (half-unit, NB)       (32D)
 */
agc_status_t agc_csm_sxtnb(agc_interp_t *ip, uint16_t trunnion_cdu,
                           uint16_t shaft_cdu, agc_vec_t *starm_nb)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(starm_nb != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_dp_t ta     = agc_csm_sxtlogic(trunnion_cdu);        /* PD0 */
    const agc_dp_t sin_ta = agc_dp_shl(ip, agc_dp_sin(ip, ta), 1); /* PD2 */
    const agc_dp_t sa     = agc_align_cdulogic(shaft_cdu);         /* PD4 */
    agc_vec_t      starm;

    starm.c[0] = agc_dp_mul_trunc(agc_dp_cos(ip, sa), sin_ta); /* DMP */
    starm.c[1] = agc_dp_mul_trunc(agc_dp_sin(ip, sa), sin_ta);
    starm.c[2] = agc_dp_cos(ip, ta);
    const agc_mat_t m = nb1nb2();

    *starm_nb = agc_vec_shl(ip, agc_mat_vec(ip, &m, starm), 1); /* MXV VSL1 */

    return (AGC_STATUS_OK);
}

/* ZNB=S1 (p. 287): the star is along the shaft axis. */
static void znb_eq_s1(uint16_t *sac, uint16_t *pac, bool *cult)
{
    if (!AGC_ASSERT(sac != NULL))
    {
        return;
    }

    if (!AGC_ASSERT(pac != NULL))
    {
        return;
    }

    *sac  = SHAFT_270DEG;     /* DLOAD 270DEG STODL SAC */
    *pac  = PAC_20DEGS_MINUS; /* 20DEGS- STORE PAC      */
    *cult = false;            /* CLRGO CULTFLAG         */
}

/*
 * SXTANG1 (CSM_GEOMETRY.agc pp. 286-287) -- optics angles for STAR given
 * the optics-base axes XNB1, YNB1, ZNB1 (rows of nb1, half-unit, in the
 * frame of STAR):
 *   PDA = UNIT(ZNB1 x S);  SA = ARCTRIG(PDA . -XNB1, PDA . YNB1) -> SAC
 *   TA = ACOS(2 S . ZNB1); PAC = 1STO2S(4 TA - 20DEG-)
 * A negative trunnion or one over 90 degrees sets CULTFLAG (SXTALARM; the
 * alarm itself was removed from the routine); otherwise CULTFLAG is clear.
 */
static agc_status_t sxtang1(agc_interp_t *ip, agc_vec_t star,
                            const agc_mat_t *nb1, uint16_t *sac, uint16_t *pac,
                            bool *cult)
{
    if (!AGC_ASSERT(nb1 != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((sac != NULL) && (pac != NULL) && (cult != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t pda = agc_vec_cross(ip, nb1->row[2], star);

    align_clear_ovf(ip); /* BOV +1  */
    pda = agc_vec_unit(ip, pda, NULL, NULL);

    if (agc_interp_test_ovf(ip))
    { /* BOV     */
        znb_eq_s1(sac, pac, cult);

        return (AGC_STATUS_OK);
    }

    const agc_dp_t sinth = agc_dp_neg(agc_vec_dot(ip, pda, nb1->row[0]));
    const agc_dp_t costh = agc_vec_dot(ip, pda, nb1->row[1]);

    *sac = agc_align_ones_to_twos(agc_imu_arctrig(ip, sinth, costh));
    align_clear_ovf(ip); /* BOV +1  */
    const agc_dp_t ta =
        agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, star, nb1->row[2]), 1));

    if (ta < 0)
    {
        *cult = true; /* BMN SXTALARM: SETGO       */

        return (AGC_STATUS_OK);
    }

    const agc_dp_t ta4 = agc_dp_shl(ip, ta, 2); /* SL2     */

    if (agc_interp_test_ovf(ip))
    {
        *cult = true; /* BOV SXTALARM (> 90 deg)   */

        return (AGC_STATUS_OK);
    }

    *pac  = agc_align_ones_to_twos(agc_dp_sub(ip, ta4, TWENTY_DEG_MINUS));
    *cult = false; /* CLRGO CULTFLAG            */

    return (AGC_STATUS_OK);
}

/* Record CULTFLAG as the AGC routine leaves it. */
static agc_status_t store_cult(bool cult)
{
    agc_flag_put(AGC_FLAG_CULTFLAG, cult);

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_CULTFLAG) == cult))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/*
 * CALCSXA (CSM_GEOMETRY.agc p. 286) -- desired optics angles SAC, PAC for
 * the star direction STAR (half-unit, SM): *SMNB* with the CDUTRIG output
 * t, then NB2NB1, VSL1 into optics-base axes; XNB1..ZNB1 = unit axes;
 * SXTANG1.  STAR is left in optics-base axes (ram->star), as the AGC does.
 */
agc_status_t agc_csm_calcsxa(agc_interp_t *ip, agc_vec_t star_sm,
                             const agc_cdutrig_t *t, uint16_t *sac,
                             uint16_t *pac, bool *cult)
{
    if (!AGC_ASSERT((ip != NULL) && (t != NULL)) ||
        !AGC_ASSERT((sac != NULL) && (pac != NULL) && (cult != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t s = star_sm;

    agc_guid_smnb(t, &s); /* *SMNB*  */
    const agc_mat_t m = nb2nb1();

    s                     = agc_vec_shl(ip, agc_mat_vec(ip, &m, s), 1); /* MXV VSL1 */
    agc_align_ram()->star = s;                                          /* STOVL STAR */
    const agc_mat_t unit = agc_mat_identity_half(); /* HIUNITX.. */
    agc_status_t    st   = sxtang1(ip, s, &unit, sac, pac, cult);

    if (st == AGC_STATUS_OK)
    {
        st = store_cult(*cult);
    }

    return (st);
}

/*
 * SXTANG (CSM_GEOMETRY.agc pp. 286-287) -- optics angles for STAR given in
 * any frame in which nb holds the NB axes (XNB, YNB, ZNB rows, half-unit):
 * TRANSP1, XNB1.. = NB2NB1 (rows of the transpose), VSL1, TRANSP1/2, then
 * SXTANG1.
 */
agc_status_t agc_csm_sxtang(agc_interp_t *ip, agc_vec_t star,
                            const agc_mat_t *nb, uint16_t *sac, uint16_t *pac,
                            bool *cult)
{
    if (!AGC_ASSERT((ip != NULL) && (nb != NULL)) ||
        !AGC_ASSERT((sac != NULL) && (pac != NULL) && (cult != NULL)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_mat_t eref = agc_mat_transpose(nb); /* EREF WRT NB2     */
    const agc_mat_t m    = nb2nb1();
    agc_mat_t       t1;

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        t1.row[i] = agc_vec_shl(ip, agc_mat_vec(ip, &m, eref.row[i]), 1);
    }

    const agc_mat_t nb1 = agc_mat_transpose(&t1); /* TRANSP2          */
    agc_status_t    st  = sxtang1(ip, star, &nb1, sac, pac, cult);

    if (st == AGC_STATUS_OK)
    {
        st = store_cult(*cult);
    }

    return (st);
}

/*
 * LOG (CSM_GEOMETRY.agc p. 291) -- -ln(x)/32 for 0 < x < 1:
 * NORM x (n left shifts, xn in [.5, 1)), y = NEARLY1 - xn,
 * POLY a1 y + a2 y^2 + a3 y^3, plus n x CLOG2/32 (SHORTMP, DAS).
 */
agc_status_t agc_csm_log(agc_interp_t *ip, agc_dp_t x, agc_dp_t *neg_log)
{
    if (!AGC_ASSERT((ip != NULL) && (neg_log != NULL)) || !AGC_ASSERT(x > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int            n  = 0;
    const agc_dp_t xn = agc_dp_norm(x, &n); /* NORM     */

    if (!AGC_ASSERT((n >= 0) && (n < AGC_DP_BITS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_dp_t y       = agc_dp_sub(ip, NEARLY1, xn); /* BDSU     */
    const agc_dp_t coef[4] = {0, LOG_A1, LOG_A2, LOG_A3};
    const agc_dp_t p       = agc_dp_poly(y, coef, 3);            /* POLY     */
    const agc_dp_t k       = agc_dp_mul_int(CLOG2_32, (agc_sp_t) n); /* SHORTMP  */
    agc_ovf_t      ovf     = 0;

    *neg_log = agc_dp_add_raw(p, k, &ovf); /* DAS MPAC */

    if (!AGC_ASSERT(ovf == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/*
 * EARROT1 (CSM_GEOMETRY.agc p. 293) -- set up the earth-rotation frame
 * RTEAST = UNITW x RTINIT, RTNORM = RTEAST x UNITW, then EARROT2.
 */
agc_status_t agc_csm_earrot1(agc_interp_t *ip, agc_csm_earrot_t *e)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(e != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    e->rteast = agc_vec_cross(ip, e->unitw, e->rtinit); /* .5 UNIT  */
    e->rtnorm = agc_vec_cross(ip, e->rteast, e->unitw); /* .5 UNIT  */

    return (agc_csm_earrot2(ip, e));
}

/*
 * EARROT2 / OVERADAY (CSM_GEOMETRY.agc pp. 293-294) -- rotate RTINIT about
 * UNITW through the earth rotation angle WT = DTEAROT / (1/WIE):
 *   RT = UNIT(RTINIT + 2((cos WT - 1)/2 RTNORM + sin WT/2 RTEAST))
 * When the divide overflows (more than a day), OVERADAY takes one
 * sidereal day off DTEAROT and tries again (bounded: OVERADAY_MAX).
 */
agc_status_t agc_csm_earrot2(agc_interp_t *ip, agc_csm_earrot_t *e)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(e != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    align_clear_ovf(ip); /* BOVB     */
    agc_dp_t wt = agc_dp_div(ip, e->dtearot, ONE_OVER_WIE);

    for (int k = 0; (k < OVERADAY_MAX) && agc_interp_test_ovf(ip); k++)
    {
        const agc_dp_t day = agc_dp_sign(ONE_OVER_WIE, e->dtearot);

        e->dtearot = agc_dp_sub(ip, e->dtearot, day); /* OVERADAY */
        wt         = agc_dp_div(ip, e->dtearot, ONE_OVER_WIE);
    }

    const agc_vec_t a = agc_vec_scale(
        ip, e->rtnorm, agc_dp_sub(ip, agc_dp_cos(ip, wt), AGC_DP_HALF));
    const agc_vec_t b = agc_vec_scale(ip, e->rteast, agc_dp_sin(ip, wt));
    const agc_vec_t c = agc_vec_shl(ip, agc_vec_add(ip, b, a), 1); /* VSL1 */

    e->rt = agc_vec_unit(ip, agc_vec_add(ip, c, e->rtinit), NULL, NULL);

    return (AGC_STATUS_OK);
}
