/**********************************************************************
 * FILE NAME: cdu_trig.c
 *
 * PURPOSE:
 *   POWERED_FLIGHT_SUBROUTINES: CDUTRIG / CD*TR*GS / QUICTRIG compute the
 *   sines and cosines of three CDU angles; AX*SR*T rotates a vector
 *   between stable-member (SM) and navigation-base (NB) axes using them;
 *   SMCDURES (Comanche) converts SM increments to CDU increments;
 *   FLESHPOT (Luminary) builds the NB->SM matrix XNB. The erasables
 *   CDUSPOT, SINCDU, COSCDU they fill for the SERVICER, the landing and
 *   the TVC DAP are owned here (agc_imu_trig).
 *
 * AGC SOURCE:
 *   Comanche055 POWERED_FLIGHT_SUBROUTINES.agc pp. 1365-1372 (CDUTRIG,
 *     CD*TR*GS, QUICTRIG, AX*SR*T, *SMNB*, *NBSM*, SMCDURES)
 *   Luminary099 POWERED_FLIGHT_SUBROUTINES.agc pp. 1259-1267 (differs:
 *     FLESHPOT / CALCSMSC instead of SMCDURES)
 *   CDULOGIC: RTB_OP_CODES.agc, Comanche p.1488
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale        Description
 *   ------------  --------  ---  -----------  ------------------------
 *   CDUX/Y/Z      uint16_t  I    2^15/rev     CDU counters (CDUTRIG)
 *   CDUSPOT, SINCDU, COSCDU  agc_cdutrig_t  I/O  see imu.h  owned here
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dp_sin, _cos      SINE / COSINE (interp.h)
 *   agc_sp_sin, _cos      SPSIN / SPCOS (QUICTRIG)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None; AGC_STATUS_BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   AX*SR*T is safe only for vectors of magnitude below one (as in the
 *   AGC). QUICTRIG runs under INHINT.
 *
 * NOTES:
 *   The AGC keeps the triples in the order Y Z X (CDUSPOT, SINCDU,
 *   COSCDU) because AX*SR*T rotates about the inner, middle and outer
 *   gimbal axes in that order; the C arrays keep that order
 *   (AGC_TRIG_Y/Z/X). DEVIATION: CDULOGIC's 1's-complement logic on a
 *   2's-complement count (one count for negative angles), see cdu_as_sp.
 *   Filed with the IMU (it was src/guidance/powered_flight.c): the
 *   gimbal-angle transforms serve KALCMANU (BALLANGS), the alignment and
 *   radar routines, the SERVICER and the TVC DAP; imu is their lowest
 *   caller (layering lane).
 *
 * ALGORITHM:
 *   The basic-language routines AX*SR*T and FLESHPOT work on the stored
 *   words (agc_dp_words_t): DMPSUB keeps the high two words of the TP
 *   product, DAS and DDOUBL leave the words' signs as they fall, and the
 *   next pass multiplies those words (*NBSM* word form for callers that
 *   keep VBUF's words). SMCDURES is interpretive (DMP truncates, SL1 and
 *   DAD act on the TP MPAC).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/interp.h"
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"

#define CDU_SIGN_BIT   ((uint16_t) 040000) /* bit 15 of the 15-bit CDU word  */
#define CDU_MASK       ((uint16_t) 077777) /* the 15 bits of a CDU word */
#define CDU_FULL_SCALE 0100000L            /* 2^15 counts = one revolution    */

static agc_cdutrig_t g_trig; /* CDUSPOT, SINCDU, COSCDU (Y Z X) */

/**********************************************************************
 * AGC: none (accessor) -- the erasables CDUSPOT, SINCDU, COSCDU
 *
 * QUICTRIG / CDUTRIG results kept between routines: the SERVICER (READACCS
 * and the LM landing radar), P40 steering, the landing guidance and the
 * TVC DAP read them.
 *
 * RETURNS: agc_cdutrig_t * -- CDUSPOT, SINCDU, COSCDU (scales in imu.h)
 **********************************************************************/
agc_cdutrig_t *agc_imu_trig(void)
{
    return (&g_trig);
}

/* ---------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ---------------------------------------------------------------------- */

/**********************************************************************
 * The CDU word as a signed SP fraction scaled at a half revolution (pi),
 * which is how SPSIN/SPCOS and the AGC's 1's-complement arithmetic see it.
 * DEVIATION: the AGC applies 1's-complement logic to a 2's-complement
 * reading, an error of one count (40 arc-seconds) for negative angles;
 * QUICTRIG accepts that error and so do we (CDU 040000 maps to -037777).
 * AGC: none (helper; the AGC uses the CDU word directly, p.1367)
 *
 * FUNCTION NAME: cdu_as_sp
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   cdu       uint16_t    I    CDU angle, 15-bit 2s-complement counts
 *
 * RETURNS: agc_sp_t -- the angle, half revolutions (scale 2^0 = pi)
 **********************************************************************/
static agc_sp_t cdu_as_sp(uint16_t cdu)
{
    uint16_t w = (uint16_t) (cdu & CDU_MASK); /* 15-bit CDU word */

    if ((w & CDU_SIGN_BIT) != 0U)
    {
        return ((agc_sp_t) ((int) w - 077777));
    }

    return ((agc_sp_t) w);
}

/* ---------------------------------------------------------------------- */

/**********************************************************************
 * CDULOGIC (RTB_OP_CODES.agc, Comanche p.1488): convert the 15-bit
 * 2's-complement CDU angle (scaled in half revolutions) to a DP angle in
 * revolutions, range [-1/2, +1/2). 2^15 counts are one revolution, so the
 * DP value is the signed count times 2^13.
 *
 * FUNCTION NAME: agc_imu_cdulogic
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ---------------------------------------
 *   cdu       uint16_t    I    CDU angle, 15-bit 2s-complement counts
 *
 * RETURNS: agc_dp_t -- the angle, rev 2^0, in [-1/2, +1/2)
 **********************************************************************/
agc_dp_t agc_imu_cdulogic(uint16_t cdu)
{
    long int w = (long int) (cdu & CDU_MASK); /* signed counts, 2^15/rev */

    if (w >= (long int) CDU_SIGN_BIT)
    {
        w -= CDU_FULL_SCALE; /* 2's complement negative angle */
    }

    return ((agc_dp_t) (w * 8192L)); /* w / 2^15 rev at scale 2^0    */
}

/**********************************************************************
 * CD*TR*GS (POWERED_FLIGHT_SUBROUTINES, CM p.1366, LM p.1260): for the
 * three 2's-complement angles at CDUSPOT (order Y Z X) store the 1's
 * complement DP angles back in CDUSPOT and their half-sines and
 * half-cosines in SINCDU and COSCDU. The AGC loops X, Z, Y (index 4, 2, 0)
 * and leaves MPAC untouched; C locals make the latter automatic.
 *
 * FUNCTION NAME: agc_imu_cdtrigs
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   cdu_yzx   const uint16_t   I    CDU angles in Y Z X order, counts
 **********************************************************************/
void agc_imu_cdtrigs(agc_cdutrig_t *t, const uint16_t cdu_yzx[3])
{
    agc_interp_t ip; /* for SINE and COSINE */

    agc_interp_init(&ip);

    for (int k = 2; k >= 0; k--)
    { /* TRIGLOOP (TR*GL*P): index 4, 2, 0 */
        t->cduraw[k]  = cdu_as_sp(cdu_yzx[k]);
        t->cduspot[k] = agc_imu_cdulogic(cdu_yzx[k]); /* USPRCADR CDULOGIC */
        t->coscdu[k]  = agc_dp_cos(t->cduspot[k]);    /* COSINE, 2^1     */
        t->sincdu[k]  = agc_dp_sin(t->cduspot[k]);    /* SINE +1, 2^1    */
    }
}

/**********************************************************************
 * CDUTRIG / CDUTRIGS (CM p.1366, LM p.1260): as CD*TR*GS but from the
 * present CDU counters, loaded CDUY, CDUZ, CDUX into CDUSPOT. CDUTEMP is
 * not touched so it stays consistent with the last PIPA reading.
 *
 * FUNCTION NAME: agc_imu_cdutrig
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 **********************************************************************/
void agc_imu_cdutrig(agc_cdutrig_t *t)
{
    uint16_t cdu[3]; /* CDUY, CDUZ, CDUX counts */

    cdu[AGC_TRIG_X] = agc_erase_cdux(); /* CA CDUX  TS CDUSPOT +4       */
    cdu[AGC_TRIG_Y] = agc_erase_cduy(); /* CA CDUY  TS CDUSPOT          */
    cdu[AGC_TRIG_Z] = agc_erase_cduz(); /* CA CDUZ  TS CDUSPOT +2       */
    agc_imu_cdtrigs(t, cdu);
}

/**********************************************************************
 * QUICTRIG (CM p.1367, LM p.1261): fast single-precision version of
 * CD*TR*GS for the guidance cycle (4.1 ms vs 41 ms). SPSIN/SPCOS take the
 * 2's-complement angle scaled at pi and return sin, cos scaled at 1; the
 * MP BIT14 halves them to match the interpreter's sin/2 outputs. CDUSPOT
 * keeps the 2's-complement angles (no CDULOGIC). Runs under INHINT in the
 * AGC because the DAP shares the temporaries; C locals need no inhibit,
 * but the INHINT/RELINT pair is kept for the shared SINCDU/COSCDU.
 *
 * FUNCTION NAME: agc_imu_quictrig
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   cdu_yzx   const uint16_t   I    CDU angles in Y Z X order, counts
 **********************************************************************/
void agc_imu_quictrig(agc_cdutrig_t *t, const uint16_t cdu_yzx[3])
{
    agc_inhint();

    for (int k = 2; k >= 0; k--)
    { /* index 4, 2, 0 (X Z Y)       */
        agc_sp_t a = cdu_as_sp(cdu_yzx[k]); /* angle, scale 2^0 = pi */

        t->cduraw[k] = a;
        t->sincdu[k] = agc_sp_mul(agc_sp_sin(a), AGC_SP_HALF); /* MP BIT14 */
        t->coscdu[k] = agc_sp_mul(agc_sp_cos(a), AGC_SP_HALF);
    }

    agc_relint();
}

/**********************************************************************
 * DMPSUB as AX*SR*T calls it (INTERPRETER.agc p.1136): MPAC, MPAC+1
 * times the DP at ADDRWD; the following DXCH MPAC keeps the high two
 * words of the TP product, signs as the partial products leave them.
 * AGC: none (helper for TC DMPSUB / DXCH MPAC, p.1370)
 *
 * FUNCTION NAME: words_dmpsub
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -------------------------------------
 *   a         agc_dp_words_t  I    multiplicand words (MPAC, MPAC+1)
 *   b         agc_dp_words_t  I    multiplier words (at ADDRWD)
 *
 * RETURNS: agc_dp_words_t -- MPAC, MPAC+1 of a b, scale of a times b
 **********************************************************************/
static agc_dp_words_t words_dmpsub(agc_dp_words_t a, agc_dp_words_t b)
{
    return (agc_mpac_store(agc_mpac_dmp(agc_mpac_dload(a), b)));
}

/**********************************************************************
 * DCS: both words complemented (the AGC's DCS VBUF / DCS MPAC).
 * AGC: none (helper for EXTEND DCS, p.1370)
 *
 * FUNCTION NAME: words_dcs
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -------------------------------------
 *   a         agc_dp_words_t  I    DP words
 *
 * RETURNS: agc_dp_words_t -- -a word by word, same scale
 **********************************************************************/
static agc_dp_words_t words_dcs(agc_dp_words_t a)
{
    agc_dp_words_t r = {(agc_sp_t) -a.hi, (agc_sp_t) -a.lo}; /* DCS */

    return (r);
}

/**********************************************************************
 * DDOUBL (DAS A onto itself): both words doubled, the carry of the low
 * word into the high word as DAS makes it; callers keep the value below
 * one half, so the overflow left in A is never looked at.
 * AGC: none (helper for EXTEND DDOUBL, p.1370, p.1265)
 *
 * FUNCTION NAME: words_ddoubl
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -------------------------------------
 *   a         agc_dp_words_t  I    DP words
 *
 * RETURNS: agc_dp_words_t -- 2 a, scale of a / 2
 **********************************************************************/
static agc_dp_words_t words_ddoubl(agc_dp_words_t a)
{
    int ovf = 0; /* overflow left in A (not looked at) */

    return (agc_dp_words_das(a, a, &ovf));
}

/**********************************************************************
 * One component of an elementary rotation: DAS TERM1TMP onto the
 * component times the half cosine, then DDOUBL (DAS A onto itself),
 * which restores the scale lost to the half sine and cosine. The words
 * stay as DAS leaves them (signs may disagree). AX*SR*T's operands keep
 * the sum below one half, so neither DAS overflows; the overflow left
 * in A is not looked at by the AGC.
 * AGC: none (DMPSUB / DAS TERM1TMP / DDOUBL sequence, p.1370)
 *
 * FUNCTION NAME: rotate_component
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -------------------------------------
 *   term      agc_dp_words_t  I    TERM1TMP: the sine term, 2^-1 of v's
 *                                  scale
 *   comp      agc_dp_words_t  I    the component, v's scale
 *   c         agc_dp_words_t  I    half cosine, scale 2^1
 *
 * RETURNS: agc_dp_words_t -- 2 (term + comp c), v's scale
 **********************************************************************/
static agc_dp_words_t rotate_component(agc_dp_words_t term, agc_dp_words_t comp, agc_dp_words_t c)
{
    int            ovf = 0;                                                   /* overflow left in A (not looked at) */
    agc_dp_words_t sum = agc_dp_words_das(term, words_dmpsub(comp, c), &ovf); /* DAS TERM1TMP, 2^-1 v */

    return (words_ddoubl(sum)); /* DDOUBL */
}

/**********************************************************************
 * AX*SR*T (CM pp.1369-1371, LM pp.1262-1264) on the words of VBUF: A = +3
 * transforms NB to SM, A = -3 transforms SM to NB. The INDEXI table (4,
 * 2, 0, 4) picks the (DEX1, DEX2) pairs; DEXDEX counts 3, 2, 1 (or -3,
 * -2, -1). Each pass is basic code (DMPSUB, DAS, DDOUBL), so a component
 * can leave in words of disagreeing sign, and the next pass multiplies
 * those words as stored. Guaranteed safe only for vectors of magnitude
 * less than unity.
 *
 * FUNCTION NAME: agc_imu_axisrot_words
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   dir       agc_axisrot_dir_t      I    rotation direction (SM to NB or
 *                                         NB to SM)
 *   v         agc_vec_words_t *      I/O  VBUF, rotated in place, any
 *                                         scale |v| < 1
 **********************************************************************/
void agc_imu_axisrot_words(const agc_cdutrig_t *t, agc_axisrot_dir_t dir, agc_vec_words_t *v)
{
    /* INDEXI pairs in DP word offsets 4,2 / 2,0 / 0,4, halved to indices. */
    static const int indexi[4] = {2, 1, 0, 2};
    const bool       nbsm      = (dir == AGC_AXISROT_NBSM); /* A = +3 */

    for (int dexdex = 3; dexdex >= 1; dexdex--)
    { /* ROTLOOP, DIM DEXDEX */
        int k = nbsm ? (3 - dexdex) : (dexdex - 1); /* +3->0 ... -3->2      */

        /* One elementary rotation about the gimbal axis whose half-sine and
         * half-cosine sit at index d1, mixing components d2 and d1:
         *   NBSM: v[d2]' = 2(v[d2] c - v[d1] s),  v[d1]' = 2(v[d1] c + v[d2] s)
         *   SMNB: v[d2]' = 2(v[d2] c + v[d1] s),  v[d1]' = 2(v[d1] c - v[d2] s)
         * The AGC forms the sine term first (TERM1TMP), then DAS adds the
         * cosine term and DDOUBL restores the scale.                          */
        int d1 = indexi[k];     /* DEX1: gimbal axis, component */
        int d2 = indexi[k + 1]; /* DEX2: second component      */

        const agc_dp_words_t s    = agc_dp_to_words(t->sincdu[d1]); /* half sine, scale 2^1   */
        const agc_dp_words_t c    = agc_dp_to_words(t->coscdu[d1]); /* half cosine, scale 2^1 */
        const agc_dp_words_t old2 = v->c[d2];                       /* VBUF(D2) before, v's scale */

        /* first pass, BUF = 1: DCS VBUF(D1) times sine                       */
        agc_dp_words_t term = words_dmpsub(words_dcs(v->c[d1]), s); /* TERM1TMP, 2^-1 v */

        if (!nbsm)
        {
            term = words_dcs(term); /* SMNB case: DCS MPAC          */
        }

        v->c[d2] = rotate_component(term, v->c[d2], c);

        /* second pass, BUF = 0: old VBUF(D2) times sine                      */
        term = words_dmpsub(old2, s);

        if (!nbsm)
        {
            term = words_dcs(term);
        }

        v->c[d1] = rotate_component(term, v->c[d1], c);
    }
} /* agc_imu_axisrot_words */

/**********************************************************************
 * AX*SR*T (CM pp.1369-1371, LM pp.1262-1264) on a vector value: the
 * value enters VBUF as sign-agreeing words (MPACVBUF of a stored vector)
 * and leaves as the value of the words AX*SR*T leaves.
 *
 * FUNCTION NAME: agc_imu_axisrot
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   dir       agc_axisrot_dir_t      I    rotation direction (SM to NB or
 *                                         NB to SM)
 *   v         agc_vec_t *            I/O  vector rotated in place, any
 *                                         scale |v| < 1
 **********************************************************************/
void agc_imu_axisrot(const agc_cdutrig_t *t, agc_axisrot_dir_t dir, agc_vec_t *v)
{
    agc_vec_words_t w; /* VBUF as words */

    for (int i = 0; i < 3; i++)
    {
        w.c[i] = agc_dp_to_words(v->c[i]);
    }

    agc_imu_axisrot_words(t, dir, &w);

    for (int i = 0; i < 3; i++)
    {
        v->c[i] = agc_dp_from_words(w.c[i]);
    }
} /* agc_imu_axisrot */

/**********************************************************************
 * *SMNB* (CM p.1369, LM p.1263): SM -> NB with the present SINCDU/COSCDU.
 *
 * FUNCTION NAME: agc_imu_smnb
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   v         agc_vec_t *            I/O  vector rotated in place, any
 *                                         scale |v| < 1
 **********************************************************************/
void agc_imu_smnb(const agc_cdutrig_t *t, agc_vec_t *v)
{
    agc_imu_axisrot(t, AGC_AXISROT_SMNB, v); /* C*MM*N1: CS THREE */
}

/**********************************************************************
 * *NBSM* (CM p.1369, LM p.1263): NB -> SM with the present SINCDU/COSCDU.
 *
 * FUNCTION NAME: agc_imu_nbsm
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   v         agc_vec_t *            I/O  vector rotated in place, any
 *                                         scale |v| < 1
 **********************************************************************/
void agc_imu_nbsm(const agc_cdutrig_t *t, agc_vec_t *v)
{
    agc_imu_axisrot(t, AGC_AXISROT_NBSM, v); /* C*MM*N3: CA THREE */
}

/**********************************************************************
 * CDU*SMNB (CM p.1368, LM p.1262): CDUTRIGS then SM -> NB.
 *
 * FUNCTION NAME: agc_imu_cdu_smnb
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   v         agc_vec_t *      I/O  vector rotated in place, any scale |v|
 *                                   < 1
 **********************************************************************/
void agc_imu_cdu_smnb(agc_cdutrig_t *t, agc_vec_t *v)
{
    agc_imu_cdutrig(t);
    agc_imu_smnb(t, v);
}

/**********************************************************************
 * CDU*NBSM (CM p.1368, LM p.1262): CDUTRIGS then NB -> SM.
 *
 * FUNCTION NAME: agc_imu_cdu_nbsm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   v         agc_vec_t *      I/O  vector rotated in place, any scale |v|
 *                                   < 1
 **********************************************************************/
void agc_imu_cdu_nbsm(agc_cdutrig_t *t, agc_vec_t *v)
{
    agc_imu_cdutrig(t);
    agc_imu_nbsm(t, v);
}

/**********************************************************************
 * TRG*SMNB (CM p.1368, LM p.1262): CD*TR*GS on caller's angles (Y Z X),
 * then SM -> NB.
 *
 * FUNCTION NAME: agc_imu_trg_smnb
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   cdu_yzx   const uint16_t   I    CDU angles in Y Z X order, counts
 *   v         agc_vec_t *      I/O  vector rotated in place, any scale |v|
 *                                   < 1
 **********************************************************************/
void agc_imu_trg_smnb(agc_cdutrig_t *t, const uint16_t cdu_yzx[3], agc_vec_t *v)
{
    agc_imu_cdtrigs(t, cdu_yzx);
    agc_imu_smnb(t, v);
}

/**********************************************************************
 * TRG*NBSM (CM p.1368, LM p.1262): CD*TR*GS then NB -> SM. (Comanche's
 * NBSM interface, "the first to go", is not translated: no callers.)
 *
 * FUNCTION NAME: agc_imu_trg_nbsm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  ----------------------------------
 *   t         agc_cdutrig_t *  I/O  CDU trig block (CDUSPOT, SINCDU, COSCDU
 *                                   2^1)
 *   cdu_yzx   const uint16_t   I    CDU angles in Y Z X order, counts
 *   v         agc_vec_t *      I/O  vector rotated in place, any scale |v|
 *                                   < 1
 **********************************************************************/
void agc_imu_trg_nbsm(agc_cdutrig_t *t, const uint16_t cdu_yzx[3], agc_vec_t *v)
{
    agc_imu_cdtrigs(t, cdu_yzx);
    agc_imu_nbsm(t, v);
}

/**********************************************************************
 * TRG*NBSM (CM p.1368, LM p.1262) with VBUF's words: CD*TR*GS then
 * NB -> SM, the result as AX*SR*T leaves it in VBUF (a caller that
 * stores it and later shifts or divides it sees the words, e.g. AVEIT's
 * VSR3 after SURFSTAR).
 *
 * FUNCTION NAME: agc_imu_trg_nbsm_words
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  --------------------------------
 *   t         agc_cdutrig_t *    I/O  CDU trig block (CDUSPOT, SINCDU,
 *                                     COSCDU 2^1)
 *   cdu_yzx   const uint16_t     I    CDU angles in Y Z X order, counts
 *   v         agc_vec_words_t *  I/O  VBUF, rotated in place, any scale
 *                                     |v| < 1
 **********************************************************************/
void agc_imu_trg_nbsm_words(agc_cdutrig_t *t, const uint16_t cdu_yzx[3], agc_vec_words_t *v)
{
    agc_imu_cdtrigs(t, cdu_yzx);
    agc_imu_axisrot_words(t, AGC_AXISROT_NBSM, v); /* C*MM*N3: CA THREE */
}

/**********************************************************************
 * SMCDURES (Comanche p.1372): CDU increments from SM increments.
 *   DCDU(X) =  (dx cos IGA - dz sin IGA) / cos MGA
 *   DCDU(Y) =  dy - DCDU(X) sin MGA
 *   DCDU(Z) =  dx sin IGA + dz cos IGA
 * IGA = CDUY, MGA = CDUZ. DTHETASM and DCDU are in revolutions (2^0);
 * the sines and cosines are halves (2^1), hence the SL1s.
 *
 * FUNCTION NAME: agc_imu_smcdures
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND, index
 *                                         registers)
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   dthetasm  agc_vec_t              I    SM attitude increments, rev 2^0
 *   dcdu      agc_vec_t *            O    CDU increments, rev 2^0
 *
 * RETURNS: none
 **********************************************************************/
void agc_imu_smcdures(agc_interp_t *ip, const agc_cdutrig_t *t, agc_vec_t dthetasm, agc_vec_t *dcdu)
{
    const agc_dp_t sy = t->sincdu[AGC_TRIG_Y]; /* sin IGA / 2, scale 2^1 */
    const agc_dp_t cy = t->coscdu[AGC_TRIG_Y]; /* cos IGA / 2, scale 2^1 */
    const agc_dp_t sz = t->sincdu[AGC_TRIG_Z]; /* sin MGA / 2, scale 2^1 */
    const agc_dp_t cz = t->coscdu[AGC_TRIG_Z]; /* cos MGA / 2, scale 2^1 */

    /* DLOAD DMP DTHETASM COSCDUY ; PDDL DMP DTHETASM+4 SINCDUY ; BDSU ; DDV.
     * DMP keeps the third product word in MPAC+2: PDDL and DDV use the top
     * two (truncation), SL1 shifts all three, and DAD leaves MPAC+2 alone. */
    agc_dp_t pd0 = agc_dp_mul_trunc(dthetasm.c[0], cy);                      /* PDL 0, rev 2^1 */
    agc_dp_t a   = agc_dp_sub(ip, pd0, agc_dp_mul_trunc(dthetasm.c[2], sy)); /* DCDU(X) numerator, rev 2^1 */

    a          = agc_dp_div(ip, a, cz);
    dcdu->c[0] = a;                                                       /* STORE DCDU     */
    a          = agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(a, sz), 1)); /* DMP SL1 */
    dcdu->c[1] = agc_dp_sub(ip, dthetasm.c[1], a);                        /* BDSU DTHETASM+2 */

    /* DLOAD DTHETASM ; DMP PDDL SINCDUY DTHETASM+4 ; DMP DAD COSCDUY ; SL1 */
    pd0        = agc_dp_mul_trunc(dthetasm.c[0], sy);
    agc_tp_t z = agc_tp_add(ip, agc_dp_mul_tp(dthetasm.c[2], cy), agc_tp_from_dp(pd0)); /* DCDU(Z) / 2, rev 2^1 (TP) */

    dcdu->c[2] = agc_tp_to_dp(agc_tp_shift(ip, z, 1)); /* STORE DCDU +4  */
}

/**********************************************************************
 * FLESHPOT / CALCSMSC (Luminary pp.1265-1267): the NB->SM matrix (XNB or
 * XNBPIP, half-unit rows scaled 2^1) from SINCDU/COSCDU. Rows are the NB
 * axes in SM coordinates; row 1 = row 2 x row 0 (TCF VXV), doubled.
 *
 * FUNCTION NAME: agc_imu_fleshpot
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  ----------------------------
 *   t         const agc_cdutrig_t *  I    CDU trig block (SINCDU, COSCDU
 *                                         2^1)
 *   xnb       agc_mat_t *            O    NB-to-SM matrix, half-unit rows
 **********************************************************************/
void agc_imu_fleshpot(const agc_cdutrig_t *t, agc_mat_t *xnb)
{
    /* The half sines and cosines as stored (SINCDU, COSCDU), scale 2^1.    */
    const agc_dp_words_t sx = agc_dp_to_words(t->sincdu[AGC_TRIG_X]); /* sin OGA / 2 */
    const agc_dp_words_t cx = agc_dp_to_words(t->coscdu[AGC_TRIG_X]); /* cos OGA / 2 */
    const agc_dp_words_t sy = agc_dp_to_words(t->sincdu[AGC_TRIG_Y]); /* sin IGA / 2 */
    const agc_dp_words_t cy = agc_dp_to_words(t->coscdu[AGC_TRIG_Y]); /* cos IGA / 2 */
    const agc_dp_words_t sz = agc_dp_to_words(t->sincdu[AGC_TRIG_Z]); /* sin MGA / 2 */
    const agc_dp_words_t cz = agc_dp_to_words(t->coscdu[AGC_TRIG_Z]); /* cos MGA / 2 */

    /* Basic code: TC DMP / DMPSUB, DXCH MPAC, DDOUBL, DAS; each element is
     * kept as the words those leave until it is stored.                    */
    const agc_dp_words_t mxz = words_dmpsub(words_dcs(sx), sz);                              /* -SX SZ, 2^2       */
    const agc_dp_words_t t5  = words_ddoubl(words_ddoubl(words_dmpsub(mxz, sy)));            /* -SX SY SZ, 2^1    */
    const agc_dp_words_t buf = words_ddoubl(words_ddoubl(words_dmpsub(words_dcs(mxz), cy))); /* SX SZ CY, 2^1     */
    int                  ovf = 0;                                                            /* DAS overflow (operands < 1/2: none) */

    xnb->row[0].c[0] = agc_dp_from_words(words_ddoubl(words_dmpsub(cy, cz)));                             /* COSY COSZ  */
    xnb->row[0].c[1] = agc_dp_from_words(sz);                                                             /* SINZ       */
    xnb->row[0].c[2] = agc_dp_from_words(words_ddoubl(words_dmpsub(words_dcs(sy), cz)));                  /* -SINY COSZ */
    xnb->row[2].c[1] = agc_dp_from_words(words_ddoubl(words_dmpsub(words_dcs(sx), cz)));                  /* -SINX COSZ */
    xnb->row[2].c[2] = agc_dp_from_words(agc_dp_words_das(t5, words_ddoubl(words_dmpsub(cy, cx)), &ovf)); /* CX CY - SX SY SZ */

    /* The listing's comment reads "- SINY COSX + SINX SINZ COSY" but the
     * code loads +SINCDUY (DCA, not DCS): the element is +SY CX + SX SZ CY,
     * the correct Z_NB component. The code is translated, not the comment. */
    xnb->row[2].c[0] = agc_dp_from_words(agc_dp_words_das(buf, words_ddoubl(words_dmpsub(sy, cx)), &ovf));
    agc_interp_t ip; /* for VXV */

    agc_interp_init(&ip);
    agc_vec_t y = agc_vec_cross(&ip, xnb->row[2], xnb->row[0]); /* VXV, 2^2 */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        xnb->row[1].c[i] = agc_dp_from_words(words_ddoubl(agc_dp_to_words(y.c[i]))); /* DDOUBL */
    }
} /* agc_imu_fleshpot */
