/*
 * agc_constants.c -- vector constants of INTERPRETIVE_CONSTANTS.agc
 * (Comanche055 pp.1205-1206): UNITX/UNITY/UNITZ, XUNIT/YUNIT/ZUNIT and
 * ZEROVECS/ZEROVEC. Scalar constants are macros in agc_constants.h.
 */
#include "agc/agc_constants.h"
#include "agc/agc_assert.h"

/* UNITX = XUNIT: 2DEC .5, 2DEC 0, 2DEC 0 (half-unit x, scale 2^1).        */
agc_vec_t agc_k_unitx(void)
{
    agc_vec_t v = agc_vec_make(AGC_K_DPHALF, 0, 0);
    (void)AGC_ASSERT(v.c[0] == AGC_DP_HALF);
    (void)AGC_ASSERT((v.c[1] == 0) && (v.c[2] == 0));
    return v;
}

/* UNITY = YUNIT: (0, .5, 0).                                               */
agc_vec_t agc_k_unity(void)
{
    agc_vec_t v = agc_vec_make(0, AGC_K_DPHALF, 0);
    (void)AGC_ASSERT(v.c[1] == AGC_DP_HALF);
    (void)AGC_ASSERT((v.c[0] == 0) && (v.c[2] == 0));
    return v;
}

/* UNITZ = ZUNIT: (0, 0, .5).                                               */
agc_vec_t agc_k_unitz(void)
{
    agc_vec_t v = agc_vec_make(0, 0, AGC_K_DPHALF);
    (void)AGC_ASSERT(v.c[2] == AGC_DP_HALF);
    (void)AGC_ASSERT((v.c[0] == 0) && (v.c[1] == 0));
    return v;
}

/* ZEROVECS = ZEROVEC = ZERODP: (0, 0, 0).                                  */
agc_vec_t agc_k_zerovec(void)
{
    agc_vec_t v = agc_vec_zero();
    (void)AGC_ASSERT(v.c[0] == 0);
    (void)AGC_ASSERT((v.c[1] == 0) && (v.c[2] == 0));
    return v;
}
