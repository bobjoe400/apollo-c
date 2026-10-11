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

#include "agc/core/assert.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "internal.h"

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
 * AGC: none (DMP, DAD, SLn, STORE) -- the interpreter's TP sequence
 *
 * The TP product keeps MPAC+2, the DAD adds to MPAC, MPAC+1 only, the
 * shift moves MPAC+2's top bits up and the STORE truncates to two words.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  I/O  interpreter state
 *   a    agc_dp_t        I    multiplicand
 *   b    agc_dp_t        I    multiplier
 *   add  agc_dp_t        I    DAD operand (0 for none)
 *   n    int             I    left shift 0..3
 *
 * RETURNS: agc_dp_t -- the stored two words
 **********************************************************************/
static agc_dp_t dmp_dad_shl(agc_interp_t *ip, agc_dp_t a, agc_dp_t b, agc_dp_t add, int n)
{
    agc_tp_t m = agc_dp_mul_tp(a, b); /* DMP: TP MPAC, scales add */

    m = agc_tp_add(ip, m, agc_tp_from_dp(add)); /* DAD: MPAC+2 kept  */

    return (agc_tp_to_dp(agc_tp_shift(ip, m, n))); /* SLn, STORE        */
}

/**********************************************************************
 * CDUTODCM (ANGLFIND.agc pp.405-407) -- S/C to SM direction cosines
 *
 * From the CDU angles PHI = OGA (X), THETA = IGA (Y), PSI = MGA (Z);
 * scale 2.  Every product is DMP; C1 loses MPAC+2 at its BDSU
 * (DEVIATION, review-imu F5).
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
    agc_dp_t s[3]; /* SIN / 2 of PHI, THETA, PSI, 2^0 */
    agc_dp_t c[3]; /* COS / 2 of PHI, THETA, PSI, 2^0 */

    for (int i = 0; i < 3; i++)
    { /* LOOPSIN: X, Y, Z    */
        if (!AGC_ASSERT(cdu[i] <= CDU_MASK))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_dp_t a = agc_rtb_cdulogic(cdu[i]); /* angle, rev 2^0 */

        s[i] = agc_dp_sin(a); /* sin/2               */
        c[i] = agc_dp_cos(a); /* cos/2               */
    }

    const agc_dp_t sph = s[0]; /* PD0 SIN(PHI) / 2, 2^0               */
    const agc_dp_t cph = c[0]; /* PD2 COS(PHI) / 2, 2^0               */
    const agc_dp_t sth = s[1]; /* PD4 SIN(THETA) / 2, 2^0             */
    const agc_dp_t cth = c[1]; /* PD6 COS(THETA) / 2, 2^0             */
    const agc_dp_t sps = s[2]; /* PD8 SIN(PSI) / 2, 2^0               */
    const agc_dp_t cps = c[2]; /* PD10 COS(PSI) / 2, 2^0              */

    /* PD12 = sin(th)sin(ph), PD14 = cos(ph)sin(th): DMP then PDDL keeps
     * two words; scale 4.                                                */
    const agc_dp_t pd12 = agc_dp_mul_trunc(sth, sph); /* PD6 sin(th)sin(ph),  2^0 */
    const agc_dp_t pd14 = agc_dp_mul_trunc(cph, sth); /* PD7 cos(ph)sin(th),  2^0 */

    m->row[0].c[0] = dmp_dad_shl(ip, cth, cps, 0, 1); /* C0: DMP SL1    */

    /* C1: cos(th) DMP sin(ps) DMP cos(ph) SL1, BDSU PD12, SL1.  The BDSU
     * leaves MPAC+2 of the product; its top bit is lost here (DEVIATION:
     * see docs/modules/review-imu.md F5).                               */
    agc_dp_t t = dmp_dad_shl(ip, agc_dp_mul_trunc(cth, sps), cph, 0, 1); /* product term, 2^0 */

    m->row[0].c[1] = agc_dp_shl(ip, agc_dp_sub(ip, pd12, t), 1);

    /* C2: cos(th) DMP sin(ps) DMP sin(ph) SL1, DAD PD14, SL1            */
    agc_tp_t tp = agc_tp_shift(ip, agc_dp_mul_tp(agc_dp_mul_trunc(cth, sps), sph), 1); /* TP MPAC, 2^0 */

    tp             = agc_tp_add(ip, tp, agc_tp_from_dp(pd14));
    m->row[0].c[2] = agc_tp_to_dp(agc_tp_shift(ip, tp, 1));
    m->row[1].c[0] = sps;                                         /* C3: DLOAD 8D  */
    m->row[1].c[1] = dmp_dad_shl(ip, cps, cph, 0, 1);             /* C4: DMP SL1   */
    m->row[1].c[2] = agc_dp_neg(dmp_dad_shl(ip, cps, sph, 0, 1)); /* C5: DMP DCOMP SL1 */
    m->row[2].c[0] = agc_dp_neg(dmp_dad_shl(ip, sth, cps, 0, 1)); /* C6: DMP DCOMP SL1 */

    /* C7: (PD14 DMP sin(ps) SL1) pushed; cos(th) DMP sin(ph), DAD, SL1  */
    t              = dmp_dad_shl(ip, pd14, sps, 0, 1);
    m->row[2].c[1] = dmp_dad_shl(ip, cth, sph, t, 1);

    /* C8: (PD12 DMP sin(ps) SL1) pushed; cos(th) DMP cos(ph), DSU, SL1  */
    t              = dmp_dad_shl(ip, pd12, sps, 0, 1);
    m->row[2].c[2] = dmp_dad_shl(ip, cth, cph, agc_dp_neg(t), 1);

    return (AGC_STATUS_OK);
} /* agc_kalc_cdutodcm */

/**********************************************************************
 * SUHALFA / SUHALFAP (ANGLFIND.agc p.410) -- DCMTOCDU quadrant
 *
 * When the cosine term is negative the angle is +-.5 - angle.
 *
 * ARGUMENTS:
 *   ip       agc_interp_t *  I/O  interpreter state
 *   ang      agc_dp_t        I    ARCSIN result, rev 2^0
 *   costerm  agc_dp_t        I    cosine element of the matrix
 *
 * RETURNS: agc_dp_t -- angle, rev 2^0
 **********************************************************************/
static agc_dp_t dcm_quadrant(agc_interp_t *ip, agc_dp_t ang, agc_dp_t costerm)
{
    if (costerm >= 0)
    {
        return (ang); /* OKTHETA / OKPHI    */
    }

    agc_dp_t n = agc_dp_neg(ang);                                                            /* DCOMP, rev 2^0 */
    agc_dp_t r = (n >= 0) ? agc_dp_sub(ip, n, AGC_DP_HALF) : agc_dp_add(ip, n, AGC_DP_HALF); /* BPL SUHALFA, rev 2^0 */

    return (r);
}

/**********************************************************************
 * DCMTOCDU (ANGLFIND.agc pp.409-410) -- CDU angles from an S/C to SM
 * matrix
 *
 * cos(MGA), scale 1, is left in *cos_mga (S1).  The AGC's COS ... SL1
 * shifts the TP COS result; agc_dp_cos gives two words (F5 residual).
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

    agc_dp_t psi  = agc_dp_asin(ip, m->row[1].c[0]);        /* C3, rev 2^0 */
    agc_dp_t cpsi = agc_dp_shl(ip, agc_dp_cos(psi), 1); /* S1 = COS(MGA), 2^0 */

    cpsi           = bovb_signmpac(ip, cpsi);                              /* S1       */
    agc_dp_t q     = agc_dp_div(ip, agc_dp_neg(m->row[2].c[0]), cpsi);     /* -C6 / S1, 2^0 */
    agc_dp_t theta = dcm_quadrant(ip, agc_dp_asin(ip, q), m->row[0].c[0]); /* THETA, rev 2^0 */

    q            = agc_dp_div(ip, agc_dp_neg(m->row[1].c[2]), cpsi);     /* -C5 */
    agc_dp_t phi = dcm_quadrant(ip, agc_dp_asin(ip, q), m->row[1].c[1]); /* PHI, rev 2^0 */

    *angles  = agc_vec_make(phi, theta, psi); /* VECOFANG */
    *cos_mga = cpsi;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * DELCOMP diagonal (ANGLFIND.agc p.407) -- u u (1 - cos A) + cos A
 *
 * PDDL COF, DSQ, DMP 4, DAD 2, SL3, BOVB SIGNMPAC; scale 1.
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *  I/O  interpreter state
 *   u     agc_dp_t        I    axis component, B1
 *   omc2  agc_dp_t        I    (1 - cos A) / 2, 2^0
 *   cos8  agc_dp_t        I    cos A / 8, 2^0
 *
 * RETURNS: agc_dp_t -- DEL element, 2^0
 **********************************************************************/
static agc_dp_t del_diag(agc_interp_t *ip, agc_dp_t u, agc_dp_t omc2, agc_dp_t cos8)
{
    agc_dp_t uu = agc_tp_to_dp(agc_dp_sq_tp(u)); /* DSQ, then DMP reads 2 words, 2^0 */

    return (bovb_signmpac(ip, dmp_dad_shl(ip, uu, omc2, cos8, 3)));
}

/**********************************************************************
 * DELCOMP off-diagonal pair (ANGLFIND.agc pp.408-409)
 *
 * PD6 = ui uj (1-cos A) $4 (DMP, DMP 4, SL1), PD8 = uk sin A $4;
 * out[0] = (PD8 + PD6) SL2, out[1] = (PD6 - PD8) SL2.
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *  I/O  interpreter state
 *   ui    agc_dp_t        I    axis component i, B1
 *   uj    agc_dp_t        I    axis component j, B1
 *   uk    agc_dp_t        I    axis component k, B1
 *   omc2  agc_dp_t        I    (1 - cos A) / 2, 2^0
 *   sin2  agc_dp_t        I    sin A / 2, 2^0
 *   out   agc_dp_t [2]    O    the two DEL elements, 2^0
 **********************************************************************/
static void del_pair(agc_interp_t *ip, agc_dp_t ui, agc_dp_t uj, agc_dp_t uk, agc_dp_t omc2, agc_dp_t sin2, agc_dp_t out[2])
{
    agc_dp_t uij = agc_dp_mul_trunc(ui, uj);         /* DMP, next DMP reads 2 words, 2^0 */
    agc_dp_t pd6 = dmp_dad_shl(ip, uij, omc2, 0, 1); /* SL1, PDDL: $4, 2^0 */
    agc_dp_t pd8 = agc_dp_mul_trunc(uk, sin2);       /* PUSH: $4, 2^0  */

    out[0] = bovb_signmpac(ip, dmp_dad_shl(ip, uk, sin2, pd6, 2));
    out[1] = bovb_signmpac(ip, agc_dp_shl(ip, agc_dp_sub(ip, pd6, pd8), 2));
}

/**********************************************************************
 * DELCOMP (ANGLFIND.agc pp.407-409) -- the rotation matrix DEL
 *
 * DEL = I cos(A) + U U^T (1 - cos(A)) + U* sin(A), scale 1, U half-unit
 * along the rotation axis.  Residual vs the rope: BDSU leaves 1 - cos A
 * as a sign-disagreeing word pair that DMPSUB truncates by its words
 * (F5).
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

    agc_dp_t sin2 = agc_dp_sin(a);                                    /* PD0 $2, 2^0 */
    agc_dp_t cos2 = agc_dp_cos(a);                                    /* COS(A) / 2, 2^0 */
    agc_dp_t cos8 = agc_dp_shr(cos2, 2);                                  /* PD2 $8, 2^0 */
    agc_dp_t omc2 = bovb_signmpac(ip, agc_dp_sub(ip, AGC_DP_HALF, cos2)); /* PD4 (1-cos A)/2, $2 2^0 */

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

    if (!AGC_ASSERT(vec_ok(cof)))
    {
        return (agc_vec_zero()); /* no valid result */
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
    if (!AGC_ASSERT(p <= CDU_MASK))
    {
        return (0); /* no valid result */
    }

    long int v = (p & 040000u) ? -(long int) (~p & 037777u) : (long int) p; /* 1:S */

    if (!AGC_ASSERT(v >= -16383L && v <= 16383L))
    {
        return (0); /* no valid result */
    }

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

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
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
