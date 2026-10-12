/**********************************************************************
 * FILE NAME: vector_words.c
 *
 * PURPOSE:
 *   The vector part of the word-level MPAC API (agc_vec_words_t,
 *   include/agc/core/interp_words.h): DOT, VXM, MXV, VXSC, VAD, VSL and
 *   VSR on vectors whose components are held as the AGC stores them,
 *   two words each with signs free to disagree. The value versions in
 *   vector.c split each component into sign-agreeing words first and
 *   lose the low bits a DOTSUB or VSR of disagreeing words keeps.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETER.agc pp.1131 (VAD), 1138 (DOTSUB), 1143 (DOT,
 *   VXM, MXV), 1146 (VXSC, DVXSC), 1153 (VSL, VSR), 1159 (VRIGHT2);
 *   Luminary099 INTERPRETER.agc (identical)
 *
 * EXTERNAL VARIABLES:
 *   none (OVFIND and DVSIGN live in the caller's agc_interp_t)
 *
 * EXTERNAL REFERENCES:
 *   Name                Description
 *   ------------------  ------------------------------------------------
 *   agc_w_dotsub_words  DOTSUB on word pairs (mpac.c)
 *   agc_w_vright        RIGHT / VRIGHT2 (shift.c)
 *   agc_w_das           DAS (mpac.c)
 *   agc_mpac_dmpr/dad   scalar word-level instructions (words.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; OVFIND is set where the AGC sets it, never cleared here.
 *
 * NOTES:
 *   VSL doubles each component by DAS (overflow wraps and sets OVFIND as
 *   OVERFLWY); VSR rounds per word step (SETROUND) and VSHRRND.
 **********************************************************************/
#include "../mpac.h"
#include "agc/core/interp_words.h"

/**********************************************************************
 * AGC: none (C only) -- a vector as sign-agreeing stored words, as UNIT,
 * a constant or a value store leaves it
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I  vector, any scale
 *
 * RETURNS: agc_vec_words_t -- the same vector as word pairs
 **********************************************************************/
agc_vec_words_t agc_vec_to_words(agc_vec_t v)
{
    agc_vec_words_t r; /* words of v */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        r.c[i] = agc_dp_to_words(v.c[i]);
    }

    return (r);
}

/**********************************************************************
 * AGC: none (C only) -- the value of a word-level vector
 *
 * ARGUMENTS:
 *   v  agc_vec_words_t  I  vector as word pairs, any scale
 *
 * RETURNS: agc_vec_t -- hi 2^14 + lo per component, same scale
 **********************************************************************/
agc_vec_t agc_vec_from_words(agc_vec_words_t v)
{
    agc_vec_t r; /* values of v */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        r.c[i] = agc_dp_from_words(v.c[i]);
    }

    return (r);
}

/**********************************************************************
 * DOT (INTERPRETER.agc p.1143) via PREDOT / DOTSUB on stored words --
 * the TP MPAC. PREDOT sets DOTINC (= DVSIGN) to 2.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *   IO  OVFIND (+-1 on overflow), DVSIGN positive
 *   u   agc_vec_words_t  I   MPAC vector as stored
 *   v   agc_vec_words_t  I   operand as stored; scales add
 *
 * RETURNS: agc_tp_words_t -- u . v (TP MPAC)
 **********************************************************************/
agc_tp_words_t agc_mpac_vdot(agc_interp_t *ip, agc_vec_words_t u, agc_vec_words_t v)
{
    agc_w2_t uw[AGC_VEC_N]; /* u as model words */
    agc_w2_t vw[AGC_VEC_N]; /* v as model words */

    for (int k = 0; k < AGC_VEC_N; k++)
    {
        uw[k] = agc_w2_from_words(u.c[k]);
        vw[k] = agc_w2_from_words(v.c[k]);
    }

    ip->dvsign_neg = false; /* PREDOT: DOTINC = 2 */

    return (agc_w3_to_words(agc_w_dotsub_words(uw, vw, &ip->ovfind)));
}

/**********************************************************************
 * VXM (INTERPRETER.agc p.1143) -- v^T M: component j is DOTSUB of the
 * stored vector with column j of the stored rows; MPAC, MPAC+1 kept as
 * held.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *           IO  OVFIND, DVSIGN
 *   v   agc_vec_words_t          I   MPAC vector as stored
 *   m   const agc_vec_words_t [3] I  matrix rows as stored; scales add
 *
 * RETURNS: agc_vec_words_t -- v^T M as words
 **********************************************************************/
agc_vec_words_t agc_mpac_vxm(agc_interp_t *ip, agc_vec_words_t v, const agc_vec_words_t m[3])
{
    agc_vec_words_t r; /* product words */

    for (int j = 0; j < AGC_VEC_N; j++)
    {
        agc_vec_words_t col; /* column j */

        for (int i = 0; i < AGC_VEC_N; i++)
        {
            col.c[i] = m[i].c[j];
        }

        r.c[j] = agc_mpac_store(agc_mpac_vdot(ip, v, col));
    }

    return (r);
}

/**********************************************************************
 * MXV (INTERPRETER.agc p.1143) -- M v: component i is DOTSUB of the
 * stored vector with row i; MPAC, MPAC+1 kept as held.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *           IO  OVFIND, DVSIGN
 *   m   const agc_vec_words_t [3] I  matrix rows as stored
 *   v   agc_vec_words_t          I   vector as stored; scales add
 *
 * RETURNS: agc_vec_words_t -- M v as words
 **********************************************************************/
agc_vec_words_t agc_mpac_mxv(agc_interp_t *ip, const agc_vec_words_t m[3], agc_vec_words_t v)
{
    agc_vec_words_t r; /* product words */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        r.c[i] = agc_mpac_store(agc_mpac_vdot(ip, v, m[i]));
    }

    return (r);
}

/**********************************************************************
 * VXV (INTERPRETER.agc pp.1147-1148) -- MPAC x X on stored words: each
 * component is the DAS (with OVERFLOW) of two truncated DMPSUB products,
 * so its words may disagree in sign.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *   IO  OVFIND, set to +-1 on overflow
 *   u   agc_vec_words_t  I   MPAC vector as stored
 *   v   agc_vec_words_t  I   operand as stored; scales add
 *
 * RETURNS: agc_vec_words_t -- u x v as the words the AGC leaves
 **********************************************************************/
agc_vec_words_t agc_mpac_vxv(agc_interp_t *ip, agc_vec_words_t u, agc_vec_words_t v)
{
    agc_w2_t uw[AGC_VEC_N]; /* u as model words */
    agc_w2_t vw[AGC_VEC_N]; /* v as model words */

    for (int k = 0; k < AGC_VEC_N; k++)
    {
        uw[k] = agc_w2_from_words(u.c[k]);
        vw[k] = agc_w2_from_words(v.c[k]);
    }

    agc_w2_t        z = agc_w3_top(agc_w_dmpsub(agc_w_dcs(uw[1]), vw[0])); /* -X1M2 in MPAC+5 */
    agc_w2_t        y = agc_w3_top(agc_w_dmpsub(uw[2], vw[0]));            /* X1M3 (VBUF +2)  */
    agc_w2_t        x = agc_w3_top(agc_w_dmpsub(uw[1], vw[2]));            /* X3M2            */
    agc_vec_words_t r;                                                     /* u x v words */

    z      = agc_w_dad(z, agc_w3_top(agc_w_dmpsub(uw[0], vw[1])), &ip->ovfind);            /* + X2M1 */
    y      = agc_w_dad(y, agc_w3_top(agc_w_dmpsub(agc_w_dcs(uw[0]), vw[2])), &ip->ovfind); /* - X3M1 */
    x      = agc_w_dad(x, agc_w3_top(agc_w_dmpsub(agc_w_dcs(uw[2]), vw[1])), &ip->ovfind); /* - X2M3 */
    r.c[0] = agc_w2_to_words(x);
    r.c[1] = agc_w2_to_words(y);
    r.c[2] = agc_w2_to_words(z);

    return (r);
}

/**********************************************************************
 * VXSC with the scalar in MPAC (DVXSC, INTERPRETER.agc p.1146) --
 * DMPSUB and VROUND per component on the words as held.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *   IO  OVFIND
 *   s   agc_tp_words_t   I   scalar MPAC as words (MPAC+2 unused), 2^A
 *   v   agc_vec_words_t  I   vector operand as stored, 2^B
 *
 * RETURNS: agc_vec_words_t -- the product as words, 2^(A+B)
 **********************************************************************/
agc_vec_words_t agc_mpac_vxsc(agc_interp_t *ip, agc_tp_words_t s, agc_vec_words_t v)
{
    agc_vec_words_t r; /* product words */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        r.c[i] = agc_mpac_store(agc_mpac_dmpr(ip, s, v.c[i]));
    }

    return (r);
}

/**********************************************************************
 * VAD (INTERPRETER.agc p.1131) -- DAS and OVERFLOW per component on the
 * stored words (overflow as DAD: wraps, OVFIND).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *   IO  OVFIND
 *   a   agc_vec_words_t  I   augend as stored, 2^S
 *   b   agc_vec_words_t  I   addend as stored, 2^S
 *
 * RETURNS: agc_vec_words_t -- a + b as words, 2^S
 **********************************************************************/
agc_vec_words_t agc_mpac_vad(agc_interp_t *ip, agc_vec_words_t a, agc_vec_words_t b)
{
    agc_vec_words_t r; /* sum words */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        r.c[i] = agc_mpac_store(agc_mpac_dad(ip, agc_mpac_dload(a.c[i]), b.c[i]));
    }

    return (r);
}

/**********************************************************************
 * VSL n / VSR n (VSSL / VSSR, INTERPRETER.agc pp.1153, 1159) on stored
 * words: a left shift doubles each component by DAS (wrapping, OVFIND as
 * OVERFLWY), a right shift is RIGHT / VRIGHT2 (rounded word steps, then
 * VSHRRND).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *   IO  OVFIND
 *   v   agc_vec_words_t  I   vector as stored, 2^S
 *   n   int              I   > 0 left (VSL n), < 0 right (VSR -n)
 *
 * RETURNS: agc_vec_words_t -- the shifted vector as words, 2^(S-n)
 **********************************************************************/
agc_vec_words_t agc_mpac_vshift(agc_interp_t *ip, agc_vec_words_t v, int n)
{
    agc_vec_words_t r = v; /* shifted components */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        agc_w2_t p = agc_w2_from_words(v.c[i]); /* component words */

        if (n < 0)
        {
            p = agc_w_vright(p, -n);
        }

        for (int k = 0; k < n; k++)
        {
            int ovf = 0; /* DAS overflow: +1, -1 or 0 */

            p = agc_w_das(p, p, &ovf);

            if (ovf != 0)
            {
                ip->ovfind = (agc_ovf_t) ovf; /* OVERFLWY */
            }
        }

        r.c[i] = agc_w2_to_words(p);
    }

    return (r);
}
