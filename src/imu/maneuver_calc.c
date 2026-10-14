/**********************************************************************
 * FILE NAME: maneuver_calc.c
 *
 * PURPOSE:
 *   KALCMANU set-up mathematics: direction cosine matrices from CDU
 *   angles and back (CDUTODCM, DCMTOCDU), the incremental rotation
 *   matrix DEL (DELCOMP), and the maneuver axis/angle computation of
 *   KALCMAN3 (SECAD, CHECKMAX, ALTCALC, COFMAXGO).
 *
 * AGC SOURCE:
 *   Comanche055 ANGLFIND.agc pp. 399-411
 *   Luminary099 ATTITUDE_MANEUVER_ROUTINE.agc pp. 351-363 (differs: see
 *   NOTES)
 *
 * EXTERNAL VARIABLES:
 *   none (inputs and outputs are arguments)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dp_mul_tp         DMP TP product; agc_tp_shift, agc_tp_add
 *   agc_dp_asin/acos      ARCSIN / ARCCOS; agc_dp_sqrt SQRT
 *   agc_mat_mul           MXM3, TRANSPOS / TRNSPSPD
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None here; LM TOOBADF is reported as AGC_KALC_TOOBAD for maneuver.c
 *   (ALARM 00401 there).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles; interpretive subroutines.
 *
 * NOTES:
 *   The mathematics is the same in both ropes; differences:
 *   * LM KALCMAN3 first refuses final attitudes with |CPSI| >= 70 deg
 *     (TOOBADF, alarm 00401).
 *   * MXM3/TRNSPSPD: CM forms M1 x (columns of M2) then transposes, LM
 *     forms (rows of M1) x M2; both give M1 M2 (agc_mat_mul).
 *   * LM calls DEL "KEL".
 *   Scaling: S/C-to-SM matrices (MIS, MFS) are scaled 2 (elements / 2);
 *   MFI = TMIS MFS is scaled 4; DEL is scaled 1; angles are revolutions
 *   (scale 2^0, "scaled 2 pi" in the AGC comments); COF is half-unit.
 *   Every product in these routines is DMP (truncating); oracle finding
 *   F5 residuals (<= 4 LSB) are word-split effects the agc_dp_t value
 *   type cannot carry (see docs/modules/review-imu.md).
 *
 * ALGORITHM:
 *   As the AGC listing; see each function prolog.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/interp.h"
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"

/* MINANG = 0.25 deg and MAXANG = 170 deg, in revolutions.  Luminary has
 * them as 2DEC.  Comanche writes single-word DEC constants, so DSU reads
 * MINANG with MAXANG as its low word and MAXANG with SD (DEC .433015):
 *   CM MINANG = 11, 7737  (.00069375, .472222)
 *   CM MAXANG = 7737, 7095 (.472222, .433015)                            */
#define MINANG_LM ((agc_dp_t) 186227)                  /* 2DEC 0.00069375               */
#define MAXANG_LM ((agc_dp_t) 126761187)               /* 2DEC 0.472222222              */
#define MINANG_CM ((agc_dp_t) ((11L << 14) + 7737L))   /* rev 2^0  */
#define MAXANG_CM ((agc_dp_t) ((7737L << 14) + 7095L)) /* rev 2^0  */

/* LOCKANGL DEC .388889 = 70 deg at half-revolution scale.  DSU reads two
 * words, so the low half is the next word in fixed memory: the READCDUK
 * instruction CA CDUZ (octal 30034).                                     */
#define LOCKANGL ((agc_dp_t) ((6372L << 14) + 030034L)) /* half-rev 2^0 */
#define CDU_MASK 077777u                                /* 15-bit CDU word */

/**********************************************************************
 * AGC: none (checker) -- every component a valid DP
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I    vector
 *
 * RETURNS: bool -- true when in range
 **********************************************************************/
static bool vec_ok(agc_vec_t v)
{
    bool ok = true; /* all components in range */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        ok = ok && v.c[i] >= AGC_DP_MIN && v.c[i] <= AGC_DP_MAX;
    }

    return (ok);
}

/**********************************************************************
 * BOVB SIGNMPAC (INTERPRETER.agc) -- on overflow, DPPOSMAX with the
 * sign
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state
 *   v   agc_dp_t        I    MPAC
 *
 * RETURNS: agc_dp_t -- v, or +-DPPOSMAX after an overflow
 **********************************************************************/
static agc_dp_t bovb_signmpac(agc_interp_t *ip, agc_dp_t v)
{
    if (agc_interp_test_ovf(ip))
    {
        v = agc_rtb_signmpac(v);
    }

    return (v);
}

/**********************************************************************
 * AGC: none (DLOAD x, DMP y, [DCOMP,] SL1, STORE) -- a CDUTODCM element
 * from two stored words
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *  I/O  interpreter state
 *   x     agc_dp_words_t  I    multiplicand as stored, 2^0
 *   y     agc_dp_words_t  I    multiplier as stored, 2^0
 *   comp  bool            I    DCOMP before the shift
 *
 * RETURNS: agc_dp_t -- the stored element, 2^0
 **********************************************************************/
static agc_dp_t kalc_dmp_sl1(agc_interp_t *ip, agc_dp_words_t x, agc_dp_words_t y, bool comp)
{
    agc_tp_words_t m = agc_mpac_dmp(agc_mpac_dload(x), y); /* DMP, TP MPAC */

    if (comp)
    {
        m = agc_mpac_dcomp(m); /* DCOMP: all three words */
    }

    return (agc_dp_from_words(agc_mpac_store(agc_mpac_shift(ip, m, 1))));
}

/**********************************************************************
 * CDUTODCM (ANGLFIND.agc pp.405-407) -- S/C to SM direction cosines
 *
 * From the CDU angles PHI = OGA (X), THETA = IGA (Y), PSI = MGA (Z);
 * scale 2. Every product is DMP on the push-down words as stored, and the
 * BDSU / DAD / DSU keep MPAC+2 for the SL1 after them (oracle F5).
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *      I/O  interpreter state
 *   cdu  const uint16_t [3]  I    CDU patterns X, Y, Z
 *   m    agc_mat_t *         O    MIS rows C0..C8, B1
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on bad input
 **********************************************************************/
agc_status_t agc_kalc_cdutodcm(agc_interp_t *ip, const uint16_t cdu[3], agc_mat_t *m)
{
    agc_dp_words_t s[3]; /* PD0, 4, 8: SIN / 2 of PHI, THETA, PSI as stored, 2^0 */
    agc_dp_words_t c[3]; /* PD2, 6, 10: COS / 2 as stored, 2^0 */

    for (int i = 0; i < 3; i++)
    { /* LOOPSIN: X, Y, Z    */
        if (!AGC_ASSERT(cdu[i] <= CDU_MASK))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        const agc_dp_words_t a = agc_dp_to_words(agc_rtb_cdulogic(cdu[i])); /* angle, rev 2^0 */

        s[i] = agc_mpac_store(agc_mpac_sin(agc_mpac_dload(a))); /* SIN PDDL */
        c[i] = agc_mpac_store(agc_mpac_cos(agc_mpac_dload(a))); /* COS PUSH */
    }

    /* PD12 = sin(th) sin(ph), PD14 = cos(ph) sin(th): DMP then PDDL keeps
     * the two words; scale 4.                                            */
    const agc_dp_words_t pd12 = agc_mpac_store(agc_mpac_dmp(agc_mpac_dload(s[1]), s[0])); /* DLOAD 4 DMP 0 */
    const agc_dp_words_t pd14 = agc_mpac_store(agc_mpac_dmp(agc_mpac_dload(c[0]), s[1])); /* DLOAD 2 DMP 4 */

    m->row[0].c[0] = kalc_dmp_sl1(ip, c[1], c[2], false); /* C0: DLOAD 6 DMP 10D SL1 */

    /* C1: cos(th) DMP sin(ps) DMP cos(ph) SL1, BDSU PD12, SL1            */
    agc_tp_words_t t = agc_mpac_dmp(agc_mpac_dmp(agc_mpac_dload(c[1]), s[2]), c[0]); /* TP MPAC, 2^0 */

    t              = agc_mpac_bdsu(ip, agc_mpac_shift(ip, t, 1), pd12);
    m->row[0].c[1] = agc_dp_from_words(agc_mpac_store(agc_mpac_shift(ip, t, 1)));

    /* C2: cos(th) DMP sin(ps) DMP sin(ph) SL1, DAD PD14, SL1             */
    t              = agc_mpac_dmp(agc_mpac_dmp(agc_mpac_dload(c[1]), s[2]), s[0]);
    t              = agc_mpac_dad(ip, agc_mpac_shift(ip, t, 1), pd14);
    m->row[0].c[2] = agc_dp_from_words(agc_mpac_store(agc_mpac_shift(ip, t, 1)));
    m->row[1].c[0] = agc_dp_from_words(s[2]);             /* C3: DLOAD 8D  */
    m->row[1].c[1] = kalc_dmp_sl1(ip, c[2], c[0], false); /* C4: DMP SL1   */
    m->row[1].c[2] = kalc_dmp_sl1(ip, c[2], s[0], true);  /* C5: DMP DCOMP SL1 */
    m->row[2].c[0] = kalc_dmp_sl1(ip, s[1], c[2], true);  /* C6: DMP DCOMP SL1 */

    /* C7: (PD14 DMP sin(ps) SL1) pushed; cos(th) DMP sin(ph), DAD, SL1   */
    agc_dp_words_t pd = agc_mpac_store(agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(pd14), s[2]), 1)); /* PDDL: PD14 */

    t              = agc_mpac_dad(ip, agc_mpac_dmp(agc_mpac_dload(c[1]), s[0]), pd);
    m->row[2].c[1] = agc_dp_from_words(agc_mpac_store(agc_mpac_shift(ip, t, 1)));

    /* C8: (PD12 DMP sin(ps) SL1) pushed; cos(th) DMP cos(ph), DSU, SL1   */
    pd             = agc_mpac_store(agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(pd12), s[2]), 1)); /* PDDL: PD12 */
    t              = agc_mpac_dsu(ip, agc_mpac_dmp(agc_mpac_dload(c[1]), c[0]), pd);
    m->row[2].c[2] = agc_dp_from_words(agc_mpac_store(agc_mpac_shift(ip, t, 1)));

    return (AGC_STATUS_OK);
} /* agc_kalc_cdutodcm */

/**********************************************************************
 * BOVB SIGNMPAC on the word-level MPAC (ANGLFIND.agc DELCOMP): on an
 * overflow the stored words are the signed maximum SIGNMPAC leaves.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND, cleared)
 *   m   agc_tp_words_t  I    MPAC
 *
 * RETURNS: agc_dp_words_t -- the stored words, 2^0
 **********************************************************************/
static agc_dp_words_t store_signmpac(agc_interp_t *ip, agc_tp_words_t m)
{
    agc_dp_words_t w = agc_mpac_store(m); /* MPAC, MPAC+1 as held */

    if (agc_interp_test_ovf(ip))
    {
        w = agc_dp_to_words(agc_rtb_signmpac(agc_dp_from_words(w))); /* SIGNMPAC */
    }

    return (w);
}

/**********************************************************************
 * SUHALFA / SUHALFAP (ANGLFIND.agc p.410) -- DCMTOCDU quadrant on the
 * pushed words
 *
 * When the cosine term is negative the angle is +-.5 - angle: DLOAD,
 * DCOMP, BPL (BRANCH: the sign of the first non-zero word), DSU or DAD
 * DPHALF, PUSH.
 *
 * ARGUMENTS:
 *   ip       agc_interp_t *  I/O  interpreter state
 *   ang      agc_dp_words_t  I    ARCSIN result as pushed, rev 2^0
 *   costerm  agc_dp_t        I    cosine element of the matrix
 *
 * RETURNS: agc_dp_words_t -- angle as stored, rev 2^0
 **********************************************************************/
static agc_dp_words_t dcm_quadrant(agc_interp_t *ip, agc_dp_words_t ang, agc_dp_t costerm)
{
    agc_dp_words_t r = ang; /* OKTHETA / OKPHI */

    if (costerm < 0)
    {
        const agc_dp_words_t half = agc_dp_to_words(AGC_DP_HALF);            /* DPHALF */
        agc_tp_words_t       n    = agc_mpac_dcomp(agc_mpac_dload(ang));     /* DLOAD DCOMP */
        const bool           pos  = (n.hi != 0) ? (n.hi > 0) : (n.mid >= 0); /* BPL */

        n = pos ? agc_mpac_dsu(ip, n, half) : agc_mpac_dad(ip, n, half); /* SUHALFA / DAD */
        r = agc_mpac_store(n);
    }

    return (r);
}

/**********************************************************************
 * DCMTOCDU (ANGLFIND.agc pp.409-410) -- CDU angles from an S/C to SM
 * matrix
 *
 * cos(MGA), scale 1, is left in *cos_mga (S1): ARCSIN, PUSH, COS of the
 * TP MPAC, SL1 with MPAC+2, BOVB SIGNMPAC. The angles stay the words the
 * push-down list holds until VDEF.
 *
 * ARGUMENTS:
 *   ip       agc_interp_t *     I/O  interpreter state
 *   m        const agc_mat_t *  I    matrix, B1
 *   angles   agc_vec_t *        O    PHI, THETA, PSI, rev 2^0
 *   cos_mga  agc_dp_t *         O    S1 = cos(MGA), 2^0
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on bad input
 **********************************************************************/
agc_status_t agc_kalc_dcmtocdu(agc_interp_t *ip, const agc_mat_t *m, agc_vec_t *angles, agc_dp_t *cos_mga)
{
    if (!AGC_ASSERT(vec_ok(m->row[1]) && vec_ok(m->row[2])))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_tp_words_t psim = agc_mpac_asin(ip, agc_mpac_dload(agc_dp_to_words(m->row[1].c[0])));                           /* DLOAD* ARCSIN C3 */
    const agc_dp_words_t psi  = agc_mpac_store(psim);                                                                         /* PUSH: PD +0 */
    const agc_dp_words_t s1   = store_signmpac(ip, agc_mpac_shift(ip, agc_mpac_cos(psim), 1));                                /* COS SL1 BOVB: S1 */
    agc_tp_words_t       q    = agc_mpac_dcomp(agc_mpac_dload(agc_dp_to_words(m->row[2].c[0])));                              /* -C6 */
    const agc_dp_words_t th   = dcm_quadrant(ip, agc_mpac_store(agc_mpac_asin(ip, agc_mpac_ddv(ip, q, s1))), m->row[0].c[0]); /* THETA */

    q                       = agc_mpac_dcomp(agc_mpac_dload(agc_dp_to_words(m->row[1].c[2])));                              /* -C5 */
    const agc_dp_words_t ph = dcm_quadrant(ip, agc_mpac_store(agc_mpac_asin(ip, agc_mpac_ddv(ip, q, s1))), m->row[1].c[1]); /* PHI */

    *angles  = agc_vec_make(agc_dp_from_words(ph), agc_dp_from_words(th), agc_dp_from_words(psi)); /* VECOFANG: VDEF */
    *cos_mga = agc_dp_from_words(s1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * DELCOMP diagonal (ANGLFIND.agc p.407) -- u u (1 - cos A) + cos A
 *
 * PDDL COF, DSQ, DMP 4, DAD 2, SL3, BOVB SIGNMPAC; scale 1. The DMP takes
 * PD4 = 1 - cos A as the words BDSU left (signs may disagree).
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *  I/O  interpreter state
 *   u     agc_dp_t        I    axis component, B1
 *   omc2  agc_dp_words_t  I    (1 - cos A) / 2 as stored (PD4), 2^0
 *   cos8  agc_dp_words_t  I    cos A / 8 as stored (PD2), 2^0
 *
 * RETURNS: agc_dp_t -- DEL element, 2^0
 **********************************************************************/
static agc_dp_t del_diag(agc_interp_t *ip, agc_dp_t u, agc_dp_words_t omc2, agc_dp_words_t cos8)
{
    agc_tp_words_t m = agc_mpac_dsq(agc_mpac_dload(agc_dp_to_words(u))); /* DSQ, 2^0 */

    m = agc_mpac_dad(ip, agc_mpac_dmp(m, omc2), cos8); /* DMP 4, DAD 2 */
    m = agc_mpac_shift(ip, m, 3);                      /* SL3          */

    return (agc_dp_from_words(store_signmpac(ip, m)));
}

/**********************************************************************
 * DELCOMP off-diagonal pair (ANGLFIND.agc pp.408-409)
 *
 * PD6 = ui uj (1-cos A) $4 (DMP, DMP 4, SL1), PD8 = uk sin A $4;
 * out[0] = (PD8 + PD6) SL2, out[1] = (PD6 - PD8) SL2, all on the words
 * the push-down list holds.
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *  I/O  interpreter state
 *   ui    agc_dp_t        I    axis component i, B1
 *   uj    agc_dp_t        I    axis component j, B1
 *   uk    agc_dp_t        I    axis component k, B1
 *   omc2  agc_dp_words_t  I    (1 - cos A) / 2 as stored (PD4), 2^0
 *   sin2  agc_dp_words_t  I    sin A / 2 as stored (PD0), 2^0
 *   out   agc_dp_t [2]    O    the two DEL elements, 2^0
 **********************************************************************/
static void del_pair(agc_interp_t *ip, agc_dp_t ui, agc_dp_t uj, agc_dp_t uk, agc_dp_words_t omc2, agc_dp_words_t sin2, agc_dp_t out[2])
{
    agc_tp_words_t m = agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(ui)), agc_dp_to_words(uj)); /* DLOAD DMP, 2^0 */

    m                         = agc_mpac_shift(ip, agc_mpac_dmp(m, omc2), 1);            /* DMP 4, SL1   */
    const agc_dp_words_t pd6  = agc_mpac_store(m);                                       /* PDDL: $4     */
    const agc_tp_words_t ksin = agc_mpac_dmp(agc_mpac_dload(agc_dp_to_words(uk)), sin2); /* DMP 0        */
    const agc_dp_words_t pd8  = agc_mpac_store(ksin);                                    /* PUSH: $4     */

    out[0] = agc_dp_from_words(store_signmpac(ip, agc_mpac_shift(ip, agc_mpac_dad(ip, ksin, pd6), 2)));                 /* DAD 6, SL2 */
    out[1] = agc_dp_from_words(store_signmpac(ip, agc_mpac_shift(ip, agc_mpac_bdsu(ip, agc_mpac_dload(pd8), pd6), 2))); /* DLOAD, BDSU, SL2 */
}

/**********************************************************************
 * DELCOMP (ANGLFIND.agc pp.407-409) -- the rotation matrix DEL
 *
 * DEL = I cos(A) + U U^T (1 - cos(A)) + U* sin(A), scale 1, U half-unit
 * along the rotation axis. The push-down cells are kept as the words the
 * AGC stores: BDSU leaves 1 - cos A as a sign-disagreeing word pair that
 * DMPSUB multiplies by its words (oracle finding F5).
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  I/O  interpreter state
 *   u    agc_vec_t       I    COF, B1
 *   a    agc_dp_t        I    angle A, rev 2^0
 *   del  agc_mat_t *     O    DEL, 2^0
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on bad input
 **********************************************************************/
agc_status_t agc_kalc_delcomp(agc_interp_t *ip, agc_vec_t u, agc_dp_t a, agc_mat_t *del)
{
    if (!AGC_ASSERT(vec_ok(u) && a >= AGC_DP_MIN && a <= AGC_DP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_dp_words_t aw   = agc_dp_to_words(a);                                                /* A, rev 2^0 */
    const agc_dp_words_t sin2 = agc_mpac_store(agc_mpac_sin(agc_mpac_dload(aw)));                  /* SIN PDDL: PD0 $2 */
    const agc_tp_words_t cosm = agc_mpac_cos(agc_mpac_dload(aw));                                  /* COS, TP MPAC $2 */
    const agc_dp_words_t cos2 = agc_mpac_store(cosm);                                              /* PUSH: PD2 */
    const agc_dp_words_t cos8 = agc_mpac_store(agc_mpac_shift(ip, cosm, -2));                      /* SR2 PDDL: PD2 $8 */
    const agc_dp_words_t half = agc_dp_to_words(AGC_DP_HALF);                                      /* DPHALF */
    const agc_dp_words_t omc2 = store_signmpac(ip, agc_mpac_bdsu(ip, agc_mpac_dload(cos2), half)); /* BDSU BOVB: PD4 $2 */

    del->row[0].c[0] = del_diag(ip, u.c[0], omc2, cos8);
    del->row[1].c[1] = del_diag(ip, u.c[1], omc2, cos8);
    del->row[2].c[2] = del_diag(ip, u.c[2], omc2, cos8);
    agc_dp_t pr[2]; /* off-diagonal pair, 2^0 */

    del_pair(ip, u.c[0], u.c[1], u.c[2], omc2, sin2, pr);
    del->row[1].c[0] = pr[0]; /* DEL +6   */
    del->row[0].c[1] = pr[1]; /* DEL +2   */
    del_pair(ip, u.c[0], u.c[2], u.c[1], omc2, sin2, pr);
    del->row[0].c[2] = pr[0]; /* DEL +4   */
    del->row[2].c[0] = pr[1]; /* DEL +12D */
    del_pair(ip, u.c[1], u.c[2], u.c[0], omc2, sin2, pr);
    del->row[2].c[1] = pr[0]; /* DEL +14D */
    del->row[1].c[2] = pr[1]; /* DEL +10D */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * COFMAXGO / METHOD1-3 (ANGLFIND.agc pp.401-403) -- signs of the
 * ALTCALC axis
 *
 * From the largest component: its sign from COFSKEW, the others from the
 * off-diagonal terms of MFISYM.
 *
 * ARGUMENTS:
 *   cof   agc_vec_t          I    COF from the square roots, B1
 *   skew  agc_vec_t          I    COFSKEW
 *   sym   const agc_mat_t *  I    MFISYM
 *
 * RETURNS: agc_vec_t -- COF with signs
 **********************************************************************/
static agc_vec_t cofmaxgo(agc_vec_t cof, agc_vec_t skew, const agc_mat_t *sym)
{
    int mx; /* index of the largest COF component */

    if (cof.c[0] - cof.c[1] < 0)
    { /* components are positive  */
        mx = (cof.c[1] - cof.c[2] < 0) ? 2 : 1; /* COMP12   */
    }
    else
    {
        mx = (cof.c[0] - cof.c[2] < 0) ? 2 : 0; /* METHOD3 / METHOD1 */
    }

    if (skew.c[mx] < 0)
    { /* UnPOS    */
        cof = agc_vec_neg(cof);
    }

    for (int j = 0; j < 3; j++)
    { /* OKUnm    */
        int r = (j < mx) ? j : mx; /* MFISYM row: lower index */
        int c = (j < mx) ? mx : j; /* MFISYM column             */

        if (j != mx && sym->row[r].c[c] < 0)
        {
            cof.c[j] = agc_dp_neg(cof.c[j]);
        }
    }

    return (cof);
}

/**********************************************************************
 * AGC: none (SLOAD) -- a CDU word read as a ones'-complement SP
 *
 * ARGUMENTS:
 *   p  uint16_t  I    CDU pattern
 *
 * RETURNS: agc_dp_t -- value as a DP high word
 **********************************************************************/
static agc_dp_t sload_cdu(uint16_t p)
{
    long int v = (p & 040000u) ? -(long int) (~p & 037777u) : (long int) p; /* 1:S */

    return ((agc_dp_t) (v * 16384L));
}

/**********************************************************************
 * KALCMAN3 .. SECAD .. CHECKMAX / ALTCALC (ANGLFIND.agc pp.399-403)
 *
 * From the initial CDUs bcdu and the final CPHI, CTHETA, CPSI: MIS, MFS,
 * MFI, the maneuver angle AM and axis COF; LM TOOBADF for |CPSI| >= 70
 * deg.
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *      I/O  interpreter state
 *   bcdu  const uint16_t [3]  I    initial CDU patterns (BCDU)
 *   cphi  const uint16_t [3]  I    CPHI, CTHETA, CPSI
 *   out   agc_kalc_setup_t *  O    the KALCMANU setup
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on bad input
 **********************************************************************/
agc_status_t agc_kalc_compute(agc_interp_t *ip, const uint16_t bcdu[3], const uint16_t cphi[3], agc_kalc_setup_t *out)
{
    out->kcase = AGC_KALC_NORMAL;
    out->cof   = agc_vec_zero();

    if (agc_state_vehicle() == AGC_VEHICLE_LM && agc_dp_sub(ip, agc_dp_abs(sload_cdu(cphi[2])), LOCKANGL) >= 0)
    {
        out->kcase = AGC_KALC_TOOBAD; /* TOOBADF  */

        return (AGC_STATUS_OK);
    }

    agc_status_t st = agc_kalc_cdutodcm(ip, bcdu, &out->mis); /* MIS    */

    if (st == AGC_STATUS_OK)
    {
        st = agc_kalc_cdutodcm(ip, cphi, &out->mfs); /* MFS    */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* MFI, its transpose, COFSKEW and CAM (SECAD, p.399-400).                */
    agc_mat_t tmis = agc_mat_transpose(&out->mis); /* TRANSPOS */

    out->mfi           = agc_mat_mul(ip, &tmis, &out->mfs); /* MXM3 $4  */
    const agc_mat_t *f = &out->mfi;                         /* MFI, 2^0 */

    out->cofskew = agc_vec_make(agc_dp_sub(ip, f->row[2].c[1], f->row[1].c[2]),
                                agc_dp_sub(ip, f->row[0].c[2], f->row[2].c[0]),
                                agc_dp_sub(ip, f->row[1].c[0], f->row[0].c[1]));
    agc_dp_t cam = agc_dp_add(ip, f->row[0].c[0], f->row[2].c[2]); /* CAM, 2^0 */

    cam      = agc_dp_sub(ip, cam, AGC_DP_QUARTER);
    out->cam = agc_dp_add(ip, cam, f->row[1].c[1]); /* (trace - 1)/4      */
    out->am  = agc_dp_acos(ip, out->cam);           /* revolutions        */

    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM); /* LM rope */

    if (agc_dp_sub(ip, out->am, lm ? MINANG_LM : MINANG_CM) < 0) /* MINANG */
    {
        out->kcase = AGC_KALC_SMALL; /* less than 0.25 deg       */

        return (AGC_STATUS_OK);
    }

    if (agc_dp_sub(ip, out->am, lm ? MAXANG_LM : MAXANG_CM) < 0) /* MAXANG */
    { /* CHECKMAX */
        out->cof = agc_vec_unit(ip, out->cofskew, NULL, NULL);

        return (AGC_STATUS_OK);
    }

    out->kcase = AGC_KALC_LARGE; /* ALTCALC  */
    agc_mat_t sym;               /* MFISYM,  2^0 */

    /* ALTCALC (p.400-401): for AM > 170 deg the axis comes from the
     * symmetric part MFISYM = (MFI + TMFI)/2 (scale 4):
     *   COFi = SQRT((MFISYMii - CAM/2) / (1/2 - CAM)), then UNIT.            */
    agc_mat_t t = agc_mat_transpose(&out->mfi); /* product term, 2^0 */

    for (int i = 0; i < 3; i++)
    {
        sym.row[i] = agc_vec_shr(agc_vec_add(ip, out->mfi.row[i], t.row[i]), 1);
    }

    agc_dp_t pd0 = agc_dp_shr(out->cam, 1);                                  /* CAM $4, 2^0 */
    agc_dp_t pd2 = bovb_signmpac(ip, agc_dp_sub(ip, AGC_DP_HALF, out->cam)); /* 1 - CAM,  2^0 */
    agc_dp_t cof_altcalc[3];                                                 /* COF before UNIT, 2^0 */

    for (int i = 2; i >= 0; i--)
    { /* Z, Y, X  */
        agc_dp_t r = agc_dp_div(ip, agc_dp_sub(ip, sym.row[i].c[i], pd0), pd2); /* quotient for SQRT, 2^0 */

        cof_altcalc[i] = agc_dp_sqrt(r); /* $ ROOT 2 */
    }

    if (!AGC_ASSERT(pd2 >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t cof = agc_vec_unit(ip, agc_vec_make(cof_altcalc[0], cof_altcalc[1], cof_altcalc[2]), NULL, NULL); /* COF, B1 */

    out->cof = cofmaxgo(cof, out->cofskew, &sym);

    return (AGC_STATUS_OK);
} /* agc_kalc_compute */
