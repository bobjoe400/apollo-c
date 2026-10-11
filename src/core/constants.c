/**********************************************************************
 * FILE NAME: constants.c
 *
 * PURPOSE:
 *   The vector constants of the interpretive constant pool: the
 *   half-unit vectors UNITX/UNITY/UNITZ (= XUNIT/YUNIT/ZUNIT) and the
 *   zero vector ZEROVECS/ZEROVEC. Scalar constants are macros in
 *   constants.h.
 *
 * AGC SOURCE:
 *   Comanche055 INTERPRETIVE_CONSTANTS.agc pp.1205-1206 (UNITZ, UNITY,
 *   UNITX, ZEROVECS, XUNIT, YUNIT, ZUNIT, ZEROVEC)
 *   Luminary099 INTERPRETIVE_CONSTANT.agc pp.1100-1101 (identical)
 *
 * EXTERNAL VARIABLES:
 *   none
 *
 * EXTERNAL REFERENCES:
 *   Name          Description
 *   ------------  ------------------------------------------------
 *   agc_vec_make  VDEF of three DP components (interp.h)
 *   agc_vec_zero  the zero vector (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none
 *
 * NOTES:
 *   The AGC overlaps the vectors in one table of DP words 0, 0, .5, 0,
 *   0, 0 labelled UNITZ, UNITY, UNITX, ZEROVECS (each vector starts one
 *   DP word after the previous); the second copy ZUNIT/YUNIT/XUNIT/
 *   ZEROVEC holds the same words (checked against the assembled ropes).
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/constants.h"

/**********************************************************************
 * UNITX = XUNIT (INTERPRETIVE_CONSTANTS.agc p.1205) -- 2DEC .5, 2DEC 0,
 * 2DEC 0.
 *
 * RETURNS: agc_vec_t -- (.5, 0, 0): the x half-unit vector, scaled 2^1
 **********************************************************************/
agc_vec_t agc_k_unitx(void)
{
    agc_vec_t v = agc_vec_make(AGC_K_DPHALF, 0, 0); /* scaled 2^1 */

    if (!AGC_ASSERT(v.c[0] == AGC_DP_HALF))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT((v.c[1] == 0) && (v.c[2] == 0)))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (v);
}

/**********************************************************************
 * UNITY = YUNIT (INTERPRETIVE_CONSTANTS.agc p.1205) -- (0, .5, 0).
 *
 * RETURNS: agc_vec_t -- the y half-unit vector, scaled 2^1
 **********************************************************************/
agc_vec_t agc_k_unity(void)
{
    agc_vec_t v = agc_vec_make(0, AGC_K_DPHALF, 0); /* scaled 2^1 */

    if (!AGC_ASSERT(v.c[1] == AGC_DP_HALF))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT((v.c[0] == 0) && (v.c[2] == 0)))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (v);
}

/**********************************************************************
 * UNITZ = ZUNIT (INTERPRETIVE_CONSTANTS.agc p.1205) -- (0, 0, .5).
 *
 * RETURNS: agc_vec_t -- the z half-unit vector, scaled 2^1
 **********************************************************************/
agc_vec_t agc_k_unitz(void)
{
    agc_vec_t v = agc_vec_make(0, 0, AGC_K_DPHALF); /* scaled 2^1 */

    if (!AGC_ASSERT(v.c[2] == AGC_DP_HALF))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT((v.c[0] == 0) && (v.c[1] == 0)))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (v);
}

/**********************************************************************
 * ZEROVECS = ZEROVEC = ZERODP (INTERPRETIVE_CONSTANTS.agc p.1205) --
 * (0, 0, 0).
 *
 * RETURNS: agc_vec_t -- the zero vector (any scale)
 **********************************************************************/
agc_vec_t agc_k_zerovec(void)
{
    agc_vec_t v = agc_vec_zero(); /* any scale */

    if (!AGC_ASSERT(v.c[0] == 0))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT((v.c[1] == 0) && (v.c[2] == 0)))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (v);
}
