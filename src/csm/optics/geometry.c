/**********************************************************************
 * FILE NAME: geometry.c
 *
 * PURPOSE:
 *   CSM_GEOMETRY: the sextant line of sight in navigation-base axes
 *   (SXTNB, SXTLOGIC), the desired optics angles for a star (CALCSXA,
 *   SXTANG, SXTANG1, ZNB=S1), the LOG subroutine and the earth rotation
 *   of a target vector (EARROT1, EARROT2, OVERADAY).  SVDWN1/SVDWN2 of
 *   the same file are in src/nav/integration/svdwn.c.
 *
 *   Frames: NB = navigation base; NB1 = optics base (NB2NB1: NB tilted
 *   32.5 degrees about Y; the sextant shaft axis lies along ZNB1).
 *   Optics angles: SA = shaft, TA = trunnion.  The trunnion CDU counts 4
 *   TA - 19.775 deg x 4, hence the SXTLOGIC offset and scaling.
 *
 * AGC SOURCE:
 *   Comanche055 CSM_GEOMETRY.agc pp. 285-296
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   STAR          agc_vec_t O    half-unit      CALCSXA (agc_align_ram())
 *   CULTFLAG      flag      O                   trunnion over 90 deg
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_guid_smnb         CDUTRIG, *NBSM* / *SMNB*, PIPASR (guidance.h)
 *   agc_mpac_*            interpretive instructions on MPAC words (interp_words.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_mat_*             matrix arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_tp_from_words     TP arithmetic (interp.h)
 *   agc_interp_test_ovf   interpreter state: OVFIND (interp.h)
 *   agc_flag_put          flag words (flags_common.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   SXTNB and SXTANG run on the MPAC words (word_forms.c,
 *   interp_words.h): bit exact on the oracle.  EARROT1/2 have no caller
 *   in the tree (entry has its own copy, entry_earrot).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp_words.h"
#include "agc/programs/alignment.h"

/* NB2NB1 / NB1NB2 elements, 2DEC ... B-1 (cos 32.523 deg, sin 32.523 deg). */
#define NB_COS  ((agc_dp_t) 113169126) /* .8431756920 B-1                 */
#define NB_SIN  ((agc_dp_t) 72160568)  /* .5376381241 B-1                 */
#define NB_HALF AGC_DP_HALF            /* HALFNFLT .5                     */

/* 10DEGS- DEC 3600 (added to the MPAC high word): 79.1 deg = 4 x 19.775.  */
#define TEN_DEGS_MINUS ((agc_dp_t) (3600 * 16384)) /* 10DEGS-, MPAC high word offset */

/* 20DEG- DEC 03600, DEC 00000: the same offset as a DP.                    */
#define TWENTY_DEG_MINUS ((agc_dp_t) (3600 * 16384)) /* 20DEG-, the same offset as a DP */

/* 270DEG OCT 60000: shaft 270 degrees, 2's complement CDU word.          */
#define SHAFT_270DEG ((uint16_t) 060000) /* 270DEG: 2s complement CDU word */

/* 20DEGS- DEC -07199, DEC -00000 (PAC high word), as a raw 1's complement
 * word: 077777 - 7199.                                                    */
#define PAC_20DEGS_MINUS ((uint16_t) (077777 - 7199)) /* 20DEGS- as a raw word */

/* LOG: NEARLY1 .999999999, CLOG2/32 .0216608494 = ln 2 / 32.              */
#define NEARLY1  ((agc_dp_t) 268435456 - 1) /* NEARLY1 .999999999 B0 */
#define CLOG2_32 ((agc_dp_t) 5814540)       /* CLOG2/32 = ln 2 / 32, B0 */

/* POLY coefficients a0..a3 (2DEC 0, .031335467, .0130145859, .0215738898). */
#define LOG_A1 ((agc_dp_t) 8411550) /* a1 .031335467 B0 */
#define LOG_A2 ((agc_dp_t) 3493576) /* a2 .0130145859 B0 */
#define LOG_A3 ((agc_dp_t) 5791197) /* a3 .0215738898 B0 */

/* 1/WIE 2DEC 8616410: one sidereal day in cs (B28 cs, i.e. raw cs).      */
#define ONE_OVER_WIE ((agc_dp_t) 8616410) /* 1/WIE: cs per sidereal day */
#define OVERADAY_MAX 32                   /* DTEAROT < 2^28 cs = 31.2 sidereal days        */

/**********************************************************************
 * AGC: NB2NB1 (Comanche055 CSM_GEOMETRY.agc p. 294)
 *
 * NB2NB1 (NB axes -> optics base NB1), elements B-1.
 *
 * FUNCTION NAME: nb2nb1
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_mat_t -- NB2NB1: optics base NB1 wrt NB2, half-unit rows
 *          (B-1)
 **********************************************************************/
static agc_mat_t nb2nb1(void)
{
    agc_mat_t m; /* the mark */

    m.row[0] = agc_vec_make(NB_COS, 0, -NB_SIN);
    m.row[1] = agc_vec_make(0, NB_HALF, 0);
    m.row[2] = agc_vec_make(NB_SIN, 0, NB_COS);

    _Static_assert(NB_COS > NB_SIN, "AGC constant");

    return (m);
}

/**********************************************************************
 * SXTLOGIC (CSM_GEOMETRY.agc p. 285) -- the trunnion angle in revs from the
 * trunnion CDU: (CDULOGIC(TA) + 10DEGS-) x 1/4 (SHORTMP by QUARTER; the
 * third word is dropped by DANZIG, i.e. a truncating shift).
 *
 * FUNCTION NAME: agc_csm_sxtlogic
 *
 * ARGUMENTS:
 *   Argument      Type      I/O  Description (scale)
 *   ------------  --------  ---  ----------------------------------------
 *   trunnion_cdu  uint16_t  I    trunnion CDU word (CDUT, 2^15 = 1 rev)
 *
 * RETURNS: agc_dp_t -- the trunnion angle TA, revs 2^0
 **********************************************************************/
agc_dp_t agc_csm_sxtlogic(uint16_t trunnion_cdu)
{
    agc_ovf_t      ovf = 0;                                                                      /* overflow of the ADS (dropped as the AGC does) */
    const agc_dp_t a   = agc_dp_add_raw(agc_align_cdulogic(trunnion_cdu), TEN_DEGS_MINUS, &ovf); /* CDULOGIC(TA) + 10DEGS-, revs 2^0 */

    return (agc_dp_shr(a, 2)); /* x QUARTER */
}

/**********************************************************************
 * SXTNB (CSM_GEOMETRY.agc p. 285) -- the sextant line of sight in NB axes
 * from the mark's trunnion (TA) and shaft (SA) CDU words: STARM = (cos SA
 * sin TA, sin SA sin TA, cos TA)/2   (optics base) result = NB1NB2 STARM,
 * VSL1  (half-unit, NB)       (32D)
 *
 * FUNCTION NAME: agc_csm_sxtnb
 *
 * ARGUMENTS:
 *   Argument      Type            I/O  Description (scale)
 *   ------------  --------------  ---  ----------------------------------
 *   ip            agc_interp_t *  IO   interpreter state (OVFIND, MPAC
 *                                      words)
 *   trunnion_cdu  uint16_t        I    trunnion CDU word (CDUT, 2^15 = 1
 *                                      rev)
 *   shaft_cdu     uint16_t        I    shaft CDU word (CDUS, 2^15 = 1 rev)
 *   starm_nb      agc_vec_t *     O    STARM in nav-base axes, half-unit
 *                                      (2^1)
 **********************************************************************/
void agc_csm_sxtnb(agc_interp_t *ip, uint16_t trunnion_cdu, uint16_t shaft_cdu, agc_vec_t *starm_nb)
{
    const agc_dp_t ta = agc_csm_sxtlogic(trunnion_cdu); /* TA, revs 2^0 (PD0) */
    const agc_dp_t sa = agc_align_cdulogic(shaft_cdu);  /* SA, revs 2^0 (PD4) */

    /* SIN SL1, PUSH: the SL1 shifts SIN's MPAC +2 into the stored words,
     * which the two DMPs then take as stored (word level).              */
    const agc_tp_words_t sin_ta = agc_mpac_shift(ip, agc_mpac_sin(agc_mpac_dload(agc_dp_to_words(ta))), 1); /* SIN TA SL1 as the TP MPAC, scale 2^0 */
    const agc_dp_words_t pd2    = agc_mpac_store(sin_ta);                                                   /* PD2 */
    const agc_tp_words_t cos_sa = agc_mpac_cos(agc_mpac_dload(agc_dp_to_words(sa)));                        /* COS SA as the TP MPAC, scale 2^1 */
    const agc_tp_words_t sin_sa = agc_mpac_sin(agc_mpac_dload(agc_dp_to_words(sa)));                        /* SIN SA as the TP MPAC, scale 2^1 */
    agc_vec_t            starm;                                                                             /* STARM, optics base, half-unit (2^1) */

    starm.c[0] = agc_dp_from_words(agc_mpac_store(agc_mpac_dmp(cos_sa, pd2))); /* COS DMP */
    starm.c[1] = agc_dp_from_words(agc_mpac_store(agc_mpac_dmp(sin_sa, pd2))); /* SIN DMP */
    starm.c[2] = agc_dp_cos(ta);

    /* NB1NB2 (optics base NB1 -> NB), the transpose of NB2NB1. */
    const agc_mat_t m_nb1nb2 = nb2nb1();                     /* NB2NB1 (transposed below) */
    const agc_mat_t m        = agc_mat_transpose(&m_nb1nb2); /* the mark */

    *starm_nb = agc_vec_shl(ip, agc_mat_vec(ip, &m, starm), 1); /* MXV VSL1 */
}

/**********************************************************************
 * SXTANG1 (CSM_GEOMETRY.agc pp. 286-287) -- optics angles for STAR given
 * the optics-base axes XNB1, YNB1, ZNB1 (rows of nb1, half-unit, in the
 * frame of STAR): PDA = UNIT(ZNB1 x S);  SA = ARCTRIG(PDA . -XNB1, PDA .
 * YNB1) -> SAC TA = ACOS(2 S . ZNB1); PAC = 1STO2S(4 TA - 20DEG-) A
 * negative trunnion or one over 90 degrees sets CULTFLAG (SXTALARM; the
 * alarm itself was removed from the routine); otherwise CULTFLAG is clear.
 *
 * FUNCTION NAME: sxtang1
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     IO   interpreter state (OVFIND, MPAC
 *                                     words)
 *   star      agc_vec_t          I    star line of sight, half-unit (2^1)
 *   nb1       const agc_mat_t *  I    NB1 axes in the frame of star,
 *                                     half-unit rows
 *   sac       uint16_t *         O    shaft angle command, CDU word
 *   pac       uint16_t *         O    trunnion angle command, CDU word
 *   cult      bool *             O    true when the trunnion is over 90 deg
 *                                     (CULTFLAG)
 *
 * RETURNS: none
 **********************************************************************/
static void sxtang1(agc_interp_t *ip, agc_vec_t star, const agc_mat_t *nb1, uint16_t *sac, uint16_t *pac, bool *cult)
{
    agc_vec_t pda = agc_vec_cross(ip, nb1->row[2], star); /* PDA = UNIT(ZNB1 x STAR), half-unit (2^1) */

    align_clear_ovf(ip); /* BOV +1  */
    pda = agc_vec_unit(ip, pda, NULL, NULL);

    if (agc_interp_test_ovf(ip))
    { /* BOV     */
        /* ZNB=S1 (p. 287): the star is along the shaft axis. */
        *sac  = SHAFT_270DEG;     /* DLOAD 270DEG STODL SAC */
        *pac  = PAC_20DEGS_MINUS; /* 20DEGS- STORE PAC      */
        *cult = false;            /* CLRGO CULTFLAG         */
        agc_flag_put(AGC_FLAG_CULTFLAG, false);

        return;
    }

    const agc_dp_t sinth = agc_dp_neg(agc_vec_dot(ip, pda, nb1->row[0])); /* -PDA . XNB1, scale 2^2 */
    const agc_dp_t costh = agc_vec_dot(ip, pda, nb1->row[1]);             /* PDA . YNB1, scale 2^2 */

    /* 1STO2S of ARCTRIG's words (word_forms.c), not of its value. */
    *sac = align_words_to_cdu(align_arctrig_words(ip, sinth, costh));
    align_clear_ovf(ip); /* BOV +1  */

    /* DOT SL1 ACOS, BMN, SL2, BOV, DSU 20DEG-, 1STO2S on the MPAC words. */
    agc_tp_words_t ta = agc_mpac_acos(ip, agc_mpac_shift(ip, agc_mpac_dot(ip, star, nb1->row[2]), 1)); /* ACOS(STAR . ZNB1 SL1) as MPAC words, revs 2^0 */

    if (agc_tp_from_words(ta) < 0)
    {
        *cult = true; /* BMN SXTALARM: SETGO       */
    }
    else
    {
        ta = agc_mpac_shift(ip, ta, 2); /* SL2       */

        if (agc_interp_test_ovf(ip))
        {
            *cult = true; /* BOV SXTALARM (> 90 deg)   */
        }
        else
        {
            *pac  = align_words_to_cdu(agc_mpac_store(agc_mpac_dsu(ip, ta, agc_dp_to_words(TWENTY_DEG_MINUS))));
            *cult = false; /* CLRGO CULTFLAG            */
        }
    }

    agc_flag_put(AGC_FLAG_CULTFLAG, *cult); /* SETGO / CLRGO CULTFLAG */
} /* sxtang1 */

/**********************************************************************
 * CALCSXA (CSM_GEOMETRY.agc p. 286) -- desired optics angles SAC, PAC for
 * the star direction STAR (half-unit, SM): *SMNB* with the CDUTRIG output
 * t, then NB2NB1, VSL1 into optics-base axes; XNB1..ZNB1 = unit axes;
 * SXTANG1.  STAR is left in optics-base axes (ram->star), as the AGC does.
 *
 * FUNCTION NAME: agc_csm_calcsxa
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------------
 *   ip        agc_interp_t *         IO   interpreter state (OVFIND, MPAC
 *                                         words)
 *   star_sm   agc_vec_t              O    star line of sight in SM axes,
 *                                         half-unit (2^1)
 *   t         const agc_cdutrig_t *  I    CDUTRIG of the present CDUs (scale 2^1)
 *   sac       uint16_t *             O    shaft angle command, CDU word
 *   pac       uint16_t *             O    trunnion angle command, CDU word
 *   cult      bool *                 O    true when the trunnion is over 90
 *                                         deg (CULTFLAG)
 *
 * RETURNS: none
 **********************************************************************/
void agc_csm_calcsxa(agc_interp_t *ip, agc_vec_t star_sm, const agc_cdutrig_t *t, uint16_t *sac, uint16_t *pac, bool *cult)
{
    agc_vec_t s = star_sm; /* STAR in NB, then NB1, half-unit (2^1) */

    agc_guid_smnb(t, &s);         /* *SMNB*  */
    const agc_mat_t m = nb2nb1(); /* the mark */

    s                     = agc_vec_shl(ip, agc_mat_vec(ip, &m, s), 1); /* MXV VSL1 */
    agc_align_ram()->star = s;                                          /* STOVL STAR */
    const agc_mat_t unit  = agc_mat_identity_half();                    /* HIUNITX.. */

    sxtang1(ip, s, &unit, sac, pac, cult);
}

/**********************************************************************
 * SXTANG (CSM_GEOMETRY.agc pp. 286-287) -- optics angles for STAR given in
 * any frame in which nb holds the NB axes (XNB, YNB, ZNB rows, half-unit):
 * TRANSP1, XNB1.. = NB2NB1 (rows of the transpose), VSL1, TRANSP1/2, then
 * SXTANG1.
 *
 * FUNCTION NAME: agc_csm_sxtang
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     IO   interpreter state (OVFIND, MPAC
 *                                     words)
 *   star      agc_vec_t          I    star line of sight, half-unit (2^1)
 *   nb        const agc_mat_t *  I    XNB, YNB, ZNB rows, half-unit
 *   sac       uint16_t *         O    shaft angle command, CDU word
 *   pac       uint16_t *         O    trunnion angle command, CDU word
 *   cult      bool *             O    true when the trunnion is over 90 deg
 *                                     (CULTFLAG)
 *
 * RETURNS: none
 **********************************************************************/
void agc_csm_sxtang(agc_interp_t *ip, agc_vec_t star, const agc_mat_t *nb, uint16_t *sac, uint16_t *pac, bool *cult)
{
    const agc_mat_t eref = agc_mat_transpose(nb); /* EREF WRT NB2     */
    const agc_mat_t m    = nb2nb1();              /* the mark */
    agc_mat_t       t1;                           /* NB2NB1 times the reference axes, half-unit rows */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        t1.row[i] = agc_vec_shl(ip, agc_mat_vec(ip, &m, eref.row[i]), 1);
    }

    const agc_mat_t nb1 = agc_mat_transpose(&t1); /* TRANSP2          */

    sxtang1(ip, star, &nb1, sac, pac, cult);
}

/**********************************************************************
 * LOG (CSM_GEOMETRY.agc p. 291) -- -ln(x)/32 for 0 < x < 1: NORM x (n left
 * shifts, xn in [.5, 1)), y = NEARLY1 - xn, POLY a1 y + a2 y^2 + a3 y^3,
 * plus n x CLOG2/32 (SHORTMP, DAS).
 *
 * FUNCTION NAME: agc_csm_log
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   x         agc_dp_t        I    argument, 0 < x < 1, scale 2^0
 *   neg_log   agc_dp_t *      O    -ln(x)/32, 2^5
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued
 *          its successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_csm_log(agc_interp_t *ip, agc_dp_t x, agc_dp_t *neg_log)
{
    if (!AGC_ASSERT((ip != NULL) && (neg_log != NULL)) || !AGC_ASSERT(x > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int            n  = 0;                  /* NORM shift count */
    const agc_dp_t xn = agc_dp_norm(x, &n); /* x normalised to [.5, 1), scale 2^0 */

    const agc_dp_t y       = agc_dp_sub(ip, NEARLY1, xn);            /* NEARLY1 - xn, scale 2^0 */
    const agc_dp_t coef[4] = {0, LOG_A1, LOG_A2, LOG_A3};
    const agc_dp_t p       = agc_dp_poly(y, coef, 3);                /* the POLY, -ln(xn)/32, scale 2^5 */
    const agc_dp_t k       = agc_dp_mul_int(CLOG2_32, (agc_sp_t) n); /* n ln 2 / 32 (SHORTMP), scale 2^5 */
    agc_ovf_t      ovf     = 0;                                      /* overflow of the DAS (dropped as the AGC does) */

    *neg_log = agc_dp_add_raw(p, k, &ovf); /* DAS MPAC */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * EARROT1 (CSM_GEOMETRY.agc p. 293) -- set up the earth-rotation frame
 * RTEAST = UNITW x RTINIT, RTNORM = RTEAST x UNITW, then EARROT2.
 *
 * FUNCTION NAME: agc_csm_earrot1
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------------
 *   ip        agc_interp_t *      IO   interpreter state (OVFIND, MPAC
 *                                      words)
 *   e         agc_csm_earrot_t *  IO   earth-rotation frame (RTINIT,
 *                                      RTEAST, RTNORM, UNITW)
 *
 **********************************************************************/
void agc_csm_earrot1(agc_interp_t *ip, agc_csm_earrot_t *e)
{
    e->rteast = agc_vec_cross(ip, e->unitw, e->rtinit); /* .5 UNIT  */
    e->rtnorm = agc_vec_cross(ip, e->rteast, e->unitw); /* .5 UNIT  */
    agc_csm_earrot2(ip, e);
}

/**********************************************************************
 * EARROT2 / OVERADAY (CSM_GEOMETRY.agc pp. 293-294) -- rotate RTINIT about
 * UNITW through the earth rotation angle WT = DTEAROT / (1/WIE): RT =
 * UNIT(RTINIT + 2((cos WT - 1)/2 RTNORM + sin WT/2 RTEAST)) When the divide
 * overflows (more than a day), OVERADAY takes one sidereal day off DTEAROT
 * and tries again (bounded: OVERADAY_MAX).
 *
 * FUNCTION NAME: agc_csm_earrot2
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------------
 *   ip        agc_interp_t *      IO   interpreter state (OVFIND, MPAC
 *                                      words)
 *   e         agc_csm_earrot_t *  IO   earth-rotation frame (RTINIT,
 *                                      RTEAST, RTNORM, UNITW)
 *
 * RETURNS: none
 **********************************************************************/
void agc_csm_earrot2(agc_interp_t *ip, agc_csm_earrot_t *e)
{
    align_clear_ovf(ip);                                    /* BOVB     */
    agc_dp_t wt = agc_dp_div(ip, e->dtearot, ONE_OVER_WIE); /* DTEAROT / (1/WIE): earth rotation angle, revs 2^0 */

    for (int k = 0; (k < OVERADAY_MAX) && agc_interp_test_ovf(ip); k++)
    {
        const agc_dp_t day = agc_dp_sign(ONE_OVER_WIE, e->dtearot); /* one sidereal day with the sign of DTEAROT, cs 2^28 */

        e->dtearot = agc_dp_sub(ip, e->dtearot, day); /* OVERADAY */
        wt         = agc_dp_div(ip, e->dtearot, ONE_OVER_WIE);
    }

    const agc_vec_t a = agc_vec_scale(ip, e->rtnorm, agc_dp_sub(ip, agc_dp_cos(wt), AGC_DP_HALF)); /* RTNORM (cos WT - 1/2), half-unit (2^1) */
    const agc_vec_t b = agc_vec_scale(ip, e->rteast, agc_dp_sin(wt));                              /* RTEAST sin WT, half-unit (2^1) */
    const agc_vec_t c = agc_vec_shl(ip, agc_vec_add(ip, b, a), 1);                                 /* (a + b) VSL1, half-unit (2^1) */

    e->rt = agc_vec_unit(ip, agc_vec_add(ip, c, e->rtinit), NULL, NULL);
}
