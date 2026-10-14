/**********************************************************************
 * FILE NAME: word_forms.c
 *
 * PURPOSE:
 *   The alignment results whose low bits depend on the words the AGC
 *   leaves in MPAC, not only on their value: ARCTRIG's angle (ASIN or
 *   ACOS with SIGN, or HIDPHALF - ASIN) as words, 1STO2S on such words,
 *   and DOT / DMP followed by SLn on the TP MPAC.  The instructions run
 *   on include/agc/core/interp_words.h; 1TO2SUB reads the major word
 *   after DDOUBL, so the same value converts to counts differing by one
 *   when the words disagree in sign.  The yaAGC oracle
 *   (tests/oracle/alignment.c: RRANGLES, SXTANG, RRNB, OANB) shows the
 *   AGC result.
 *
 * AGC SOURCE:
 *   INFLIGHT_ALIGNMENT_ROUTINES.agc (Comanche055 p. 1349 / Luminary099
 *   p. 1349, ARCTRIG), RTB_OP_CODES.agc pp. 1397-1398 (1STO2S, 1TO2SUB),
 *   INTERPRETER.agc TSSL (SL on the TP MPAC).
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name              Description
 *   ----------------  ----------------------------------------------
 *   agc_mpac_*        interpretive instructions on words (interp_words.h)
 *   agc_tp_shift      SL n on the TP MPAC (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ARCTRIG's ASIN/ACOS latch ALARM 01301 for an argument out of range,
 *   as agc_mpac_asin / agc_mpac_acos do.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles.  -0 is not modelled: 1TO2SUB's CCS turns it into +0.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp_words.h"
#include "agc/programs/alignment.h"

#define WORD_MAX 16383 /* largest 14-bit magnitude                    */
#define CDU_MOD  32768 /* one revolution in CDU counts                 */

#define QTSN45 ((agc_dp_t) 47459389) /* QTSN45 2DEC .1768 (sin 45 deg / 4), rope 05520 26075 */

/**********************************************************************
 * ARCTRIG (INFLIGHT_ALIGNMENT_ROUTINES.agc p. 1349) on the MPAC words
 *
 *   |SINTH| >= sin 45 / 4:  ACOS(2 COSTH) SIGN SINTH
 *   else THETA = ASIN(2 SINTH); COSTH >= 0: THETA (DLOAD THETA);
 *        COSTH < 0 (TRIG2): HIDPHALF SIGN SINTH, DSU THETA
 *
 * FUNCTION NAME: align_arctrig_words
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter (OVFIND, ARCSIN alarm)
 *   sinth     agc_dp_t        I    SINTH, sin/4 (B2)
 *   costh     agc_dp_t        I    COSTH, cos/4 (B2)
 *
 * RETURNS: agc_dp_words_t -- the angle in revs (B0) as MPAC, MPAC +1
 *          hold it at the RVQ (signs may disagree)
 **********************************************************************/
agc_dp_words_t align_arctrig_words(agc_interp_t *ip, agc_dp_t sinth, agc_dp_t costh)
{
    const agc_dp_words_t sw = agc_dp_to_words(sinth);                                                      /* SINTH as MPAC words, revs 2^0 */
    const agc_tp_words_t d  = agc_mpac_dsu(ip, agc_mpac_abs(agc_mpac_dload(sw)), agc_dp_to_words(QTSN45)); /* |SINTH| - QTSN45 (TP words), revs 2^0 */
    agc_tp_words_t       m;                                                                                /* the mark */

    if (agc_tp_from_words(d) >= 0)
    {
        /* (45,135) or (-135,-45): DLOAD SL1 COSTH, ACOS, SIGN SINTH.     */
        m = agc_mpac_sign(agc_mpac_acos(ip, agc_mpac_shift(ip, agc_mpac_dload(agc_dp_to_words(costh)), 1)), sw);
    }
    else
    {
        /* TRIG1: DLOAD SL1 SINTH, ASIN, STODL THETA.                    */
        m = agc_mpac_asin(ip, agc_mpac_shift(ip, agc_mpac_dload(sw), 1));

        if (costh < 0)
        {
            /* TRIG2: DLOAD HIDPHALF, SIGN SINTH, DSU THETA.             */
            const agc_dp_words_t theta = agc_mpac_store(m); /* ASIN result stored as THETA, revs 2^0 */

            m = agc_mpac_dsu(ip, agc_mpac_sign(agc_mpac_dload(agc_dp_to_words(AGC_DP_HALF)), sw), theta);
        }
    }

    return (agc_mpac_store(m));
}

/**********************************************************************
 * 1STO2S / 1TO2SUB (RTB_OP_CODES.agc pp. 1397-1398) on the words
 *
 * FUNCTION NAME: align_words_to_cdu
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ------------------------------------
 *   w         agc_dp_words_t I    angle in revs (B0) as MPAC words
 *
 * RETURNS: uint16_t -- 2's complement CDU pattern (2^-15 rev): DDOUBL,
 *          the major word (with the minor word's carry); a negative
 *          major word converted by CCS/COM (same value; -0 gives +0);
 *          a doubling overflow wraps by one revolution (LIMITS)
 **********************************************************************/
uint16_t align_words_to_cdu(agc_dp_words_t w)
{
    int32_t a     = 2 * (int32_t) w.hi; /* DDOUBL major word, counts */
    int32_t carry = 0;                  /* from 2 x minor word       */

    if (2 * (int32_t) w.lo > WORD_MAX)
    {
        carry = 1;
    }
    else if (2 * (int32_t) w.lo < -WORD_MAX)
    {
        carry = -1;
    }
    else
    {
        /* no carry */
    }

    a += carry;

    if (a > WORD_MAX)
    {
        a -= CDU_MOD; /* LIMITS: one revolution */
    }
    else if (a < -WORD_MAX - 1)
    {
        a += CDU_MOD;
    }
    else
    {
        /* in range */
    }

    return ((uint16_t) ((a < 0) ? (a + CDU_MOD) : a)); /* negative counts as the 2s complement pattern */
}

/**********************************************************************
 * DOT then SLn (INTERPRETER.agc TSSL, p. 1154) -- the shift acts on the
 * triple-precision MPAC the dot product leaves, so MPAC +2 supplies the
 * low bits of the stored DP
 *
 * FUNCTION NAME: align_dot_shl
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter (OVFIND on overflow)
 *   u, v      agc_vec_t       I    the vectors (scales add)
 *   n         int             I    left shift, 1..4
 *
 * RETURNS: agc_dp_t -- (u . v) 2^n, truncated to DP (no rounding: SL)
 **********************************************************************/
agc_dp_t align_dot_shl(agc_interp_t *ip, agc_vec_t u, agc_vec_t v, int n)
{
    return (agc_tp_to_dp(agc_tp_shift(ip, agc_vec_dot_tp(ip, u, v), n)));
}

/**********************************************************************
 * DMP then SLn (INTERPRETER.agc TSSL on the DMPSUB product) -- as
 * align_dot_shl for a scalar product
 *
 * FUNCTION NAME: align_dmp_shl
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter (OVFIND on overflow)
 *   a, b      agc_dp_t        I    the factors (scales add)
 *   n         int             I    left shift, 1..4
 *
 * RETURNS: agc_dp_t -- a b 2^n, truncated to DP (no rounding: SL)
 **********************************************************************/
agc_dp_t align_dmp_shl(agc_interp_t *ip, agc_dp_t a, agc_dp_t b, int n)
{
    return (agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(a, b), n)));
}
