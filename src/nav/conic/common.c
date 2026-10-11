/**********************************************************************
 * FILE NAME: common.c
 *
 * PURPOSE:
 *   MUTABLE, ABTCONIC, the flag mirror and the interpreter-level helpers
 *   of the conic subroutines: the interpretive MPAC is triple precision
 *   (DMP leaves a TP product, and a following SL/SR/NORM/TAD/ROUND/SQRT
 *   operates on all three words); the agc_conic_m*_ helpers are one
 *   function per interpretive instruction on that model.
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1304-1305 (ABTCONIC, MUTABLE);
 *   INTERPRETER.agc pp. 1131-1183 (DAD, DSU, BDSU, DMP, DMPR, DSQ, DDV,
 *   BDDV, SQRT, SLR/SRR, ROUND, NORM, ABS, SIGN, BMN, BZE, POLY)
 *   Luminary099 CONTROLLED_CONSTANTS.agc (MUTABLE, same assembled words),
 *   CONIC_SUBROUTINES.agc p. 1200 (ABTCONIC)
 *
 * EXTERNAL VARIABLES:
 *   Name      Type  I/O  Scale  Description
 *   --------  ----  ---  -----  ------------------------------------
 *   SOLNSW    flag  O    -      through agc_conic_mirror_flag_
 *   INFINFLG  flag  O    -      through agc_conic_mirror_flag_
 *   COGAFLAG  flag  O    -      through agc_conic_mirror_flag_
 *   APSESW    flag  O    -      through agc_conic_mirror_flag_
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_tp_shift, agc_tp_norm, agc_tp_round, agc_tp_sqrt, agc_dp_sq_tp,
 *   agc_dp_mul_tp, agc_dp_poly, agc_dp_div  core TP/DP operations
 *   agc_poodoo            POODOO (exec.h)
 *   agc_flag_put          flagword write (flags_common.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ABTCONIC: POODOO 00607, returned as AGC_STATUS_POODOO (the AGC never
 *   returns). SQRTABRT (POODOO 01302) cannot be reached from a conic path
 *   and is only asserted.
 *
 * NOTES:
 *   DEVIATION: an agc_tp_t is one number, so sign-mixed MPAC words are not
 *   modelled (BDSU leaves MPAC+2 uncomplemented; a later shift or ROUND
 *   of such a value can differ from the AGC in the last bit).
 *********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

/* MUTABLE (CONIC_SUBROUTINES.agc p. 1305; Luminary: CONTROLLED_CONSTANTS),
 * DP entries. VLOAD* MUTABLE,1 reads six words from MUTABLE - X1: earth
 * X1 = -2 gives entries 1..3, moon X1 = -10D entries 5..7.               */
static const agc_dp_t mutable_tab[8] = {
    155704375, /* MUE        22437 16067  3.986032 E10 B-36  m^3/cs^2      */
    115696162, /* 1/MUE      15625 21042  .25087606 E-10 B+34              */
    204442107, /* SQRT(MUE)  30276 04773  1.99650495 E5 B-18               */
    176229826, /* 1/SQRT(MUE) 25004 06702 .50087529 E-5 B+17               */
    122569450, /* MUM        16471 01352  4.902778 E8 B-30                 */
    146972992, /* 1/MUM      21412 20500  .203966 E-8 B+28                 */
    181389047, /* SQRT(MUM)  25477 03367  2.21422176 E4 B-15               */
    198627193  /* 1/SQRT(MUM) 27533 07571 .45162595 E-4 B+14               */
};

/**********************************************************************
 * VLOAD* MUTABLE,1 / STORE 1/MU (CONIC_SUBROUTINES.agc p. 1305) -- clear
 * the conic context and load 1/MU, ROOTMU and 1/ROOTMU of the body
 *
 * FUNCTION NAME: agc_conic_ctx_init_
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ctx       agc_conic_ctx_t *  O    zeroed; 1/MU, ROOTMU, 1/ROOTMU
 *   body      agc_conic_body_t   I    AGC_CONIC_EARTH or AGC_CONIC_MOON
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL pointer or an unknown body
 *********************************************************************/
agc_status_t agc_conic_ctx_init_(agc_conic_ctx_t *ctx, agc_conic_body_t body)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(body == AGC_CONIC_EARTH || body == AGC_CONIC_MOON))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int             base = (body == AGC_CONIC_EARTH) ? 1 : 5; /* MUTABLE+2; or +10 words */
    agc_conic_ctx_t z    = {0};                               /* cleared context */

    *ctx             = z;
    ctx->body        = body;
    ctx->inv_mu      = mutable_tab[base];
    ctx->root_mu     = mutable_tab[base + 1];
    ctx->inv_root_mu = mutable_tab[base + 2];
    ctx->vvec        = agc_vec_zero();
    ctx->ur1         = agc_vec_zero();
    ctx->u2          = agc_vec_zero();
    ctx->un          = agc_vec_zero();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * MUEARTH,2 / MUM (ORBITAL_INTEGRATION.agc p. 1354) -- MU of the body as
 * KEPPREP loads it through PBODY (same assembled octal as MUTABLE)
 *
 * FUNCTION NAME: agc_conic_mu_
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ----------------------------------
 *   body      agc_conic_body_t  I    central body
 *
 * RETURNS: agc_dp_t -- MU, 2^36 (earth) / 2^30 (moon) m^3/cs^2
 *********************************************************************/
agc_dp_t agc_conic_mu_(agc_conic_body_t body)
{
    if (!AGC_ASSERT(body == AGC_CONIC_EARTH || body == AGC_CONIC_MOON))
    {
        return (mutable_tab[0]);
    }

    if (!AGC_ASSERT(mutable_tab[0] > mutable_tab[4])) /* MUE > MUM */
    {
        return (mutable_tab[0]);
    }

    return ((body == AGC_CONIC_EARTH) ? mutable_tab[0] : mutable_tab[4]); /* MUEARTH, else MUM */
}

/* ---- interpretive MPAC as a TP accumulator (see internal.h) ------- */

/**********************************************************************
 * AGC: none (TP model) -- MPAC+2 of a TP accumulator, the word a DP-mode
 * STORE drops
 *
 * FUNCTION NAME: mpac_low
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------------
 *   m         agc_tp_t  I    MPAC, MPAC+1, MPAC+2
 *
 * RETURNS: agc_tp_t -- MPAC+2 alone, |value| < 2^14 TP LSB
 *********************************************************************/
static agc_tp_t mpac_low(agc_tp_t m)
{
    agc_tp_t lo = m - agc_tp_from_dp(agc_tp_to_dp(m)); /* MPAC+2, 2^-28 of the MPAC scale */

    if (!AGC_ASSERT(lo > -((agc_tp_t) 1 << AGC_SP_BITS)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(lo < ((agc_tp_t) 1 << AGC_SP_BITS)))
    {
        return (0); /* no valid result */
    }

    return (lo);
}

/**********************************************************************
 * DAD X (INTERPRETER.agc p. 1131) -- DAS into MPAC,+1 (overflow-corrected
 * result, OVFIND); MPAC+2 kept
 *
 * FUNCTION NAME: agc_conic_mdad_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  OVFIND
 *   m         agc_tp_t        I    MPAC
 *   d         agc_dp_t        I    X, scale of MPAC
 *
 * RETURNS: agc_tp_t -- MPAC + X
 *********************************************************************/
agc_tp_t agc_conic_mdad_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX))
    {
        return (m);
    }

    return (agc_tp_from_dp(agc_dp_add(ip, agc_tp_to_dp(m), d)) + mpac_low(m));
}

/**********************************************************************
 * DSU X (INTERPRETER.agc p. 1133) -- as DAD with the complement of X
 *
 * FUNCTION NAME: agc_conic_mdsu_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  OVFIND
 *   m         agc_tp_t        I    MPAC
 *   d         agc_dp_t        I    X, scale of MPAC
 *
 * RETURNS: agc_tp_t -- MPAC - X
 *********************************************************************/
agc_tp_t agc_conic_mdsu_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX))
    {
        return (m);
    }

    return (agc_tp_from_dp(agc_dp_sub(ip, agc_tp_to_dp(m), d)) + mpac_low(m));
}

/**********************************************************************
 * BDSU X (INTERPRETER.agc p. 1134) -- MPAC,+1 = X - MPAC,+1. MPAC+2 is
 * NOT complemented by the AGC (DCOM acts on A,L only), so a leftover
 * product word keeps its sign; reproduced as is.
 *
 * FUNCTION NAME: agc_conic_mbdsu_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  OVFIND
 *   m         agc_tp_t        I    MPAC
 *   d         agc_dp_t        I    X, scale of MPAC
 *
 * RETURNS: agc_tp_t -- X - MPAC,+1 with the old MPAC+2
 *********************************************************************/
agc_tp_t agc_conic_mbdsu_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX))
    {
        return (m);
    }

    return (agc_tp_from_dp(agc_dp_sub(ip, d, agc_tp_to_dp(m))) + mpac_low(m));
}

/**********************************************************************
 * DMP X (DMPSUB, INTERPRETER.agc p. 1136) -- MPAC,+1 times X, TP product
 * in MPAC,+1,+2
 *
 * FUNCTION NAME: agc_conic_mdmp_
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------------
 *   m         agc_tp_t  I    MPAC (MPAC+2 ignored)
 *   d         agc_dp_t  I    X
 *
 * RETURNS: agc_tp_t -- TP product, scale = sum of the operand scales
 *********************************************************************/
agc_tp_t agc_conic_mdmp_(agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_mul_tp(agc_tp_to_dp(m), d));
}

/**********************************************************************
 * DMPR X (INTERPRETER.agc p. 1149) -- DMP then ROUNDSUB
 *
 * FUNCTION NAME: agc_conic_mdmpr_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  OVFIND (carry of the round)
 *   m         agc_tp_t        I    MPAC
 *   d         agc_dp_t        I    X
 *
 * RETURNS: agc_tp_t -- rounded DP product (MPAC+2 = 0)
 *********************************************************************/
agc_tp_t agc_conic_mdmpr_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX))
    {
        return (m);
    }

    return (agc_tp_from_dp(agc_dp_mul(ip, agc_tp_to_dp(m), d)));
}

/**********************************************************************
 * DSQ (DSQSUB, INTERPRETER.agc p. 1180) -- square of MPAC,+1, TP result
 *
 * FUNCTION NAME: agc_conic_mdsq_
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------------
 *   m         agc_tp_t  I    MPAC (MPAC+2 ignored)
 *
 * RETURNS: agc_tp_t -- MPAC^2, twice the scale of MPAC
 *********************************************************************/
agc_tp_t agc_conic_mdsq_(agc_tp_t m)
{
    agc_dp_t d = agc_tp_to_dp(m); /* MPAC,+1, scale of MPAC */

    if (!AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_sq_tp(d));
}

/**********************************************************************
 * DDV X (INTERPRETER.agc p. 1162) -- MPAC,+1 / X; DP quotient, OVFIND and
 * DPPOSMAX/NEGMAX on overflow
 *
 * FUNCTION NAME: agc_conic_mddv_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  OVFIND
 *   m         agc_tp_t        I    dividend MPAC,+1
 *   d         agc_dp_t        I    divisor X
 *
 * RETURNS: agc_tp_t -- quotient, scale of MPAC minus scale of X
 *********************************************************************/
agc_tp_t agc_conic_mddv_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX))
    {
        return (m);
    }

    return (agc_tp_from_dp(agc_dp_div(ip, agc_tp_to_dp(m), d)));
}

/**********************************************************************
 * BDDV X (INTERPRETER.agc p. 1149) -- X / MPAC,+1
 *
 * FUNCTION NAME: agc_conic_mbddv_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  OVFIND
 *   m         agc_tp_t        I    divisor MPAC,+1
 *   d         agc_dp_t        I    dividend X
 *
 * RETURNS: agc_tp_t -- quotient, scale of X minus scale of MPAC
 *********************************************************************/
agc_tp_t agc_conic_mbddv_(agc_interp_t *ip, agc_tp_t m, agc_dp_t d)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX))
    {
        return (m);
    }

    return (agc_tp_from_dp(agc_dp_div(ip, d, agc_tp_to_dp(m))));
}

/**********************************************************************
 * SQRT (SQRTSUB, INTERPRETER.agc p. 1181) -- square root of the TP MPAC
 * (MPAC+2 takes part when MPAC is zero). An argument below -2^-14 is the
 * AGC's SQRTABRT (POODOO 01302); no conic path can produce one (every
 * root is of a quantity just tested or built non-negative), so it is
 * only asserted.
 *
 * FUNCTION NAME: agc_conic_msqrt_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state
 *   m         agc_tp_t        I    MPAC, scale 2^(2k)
 *
 * RETURNS: agc_tp_t -- sqrt(MPAC), scale 2^k
 *********************************************************************/
agc_tp_t agc_conic_msqrt_(agc_interp_t *ip, agc_tp_t m)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(m > -((agc_tp_t) 1 << (AGC_TP_BITS - AGC_SP_BITS))))
    {
        return (m);
    }

    return (agc_tp_from_dp(agc_tp_sqrt(ip, m)));
}

/**********************************************************************
 * SLR n / SRR -n (INTERPRETER.agc pp. 1160-1161) -- shift, then ROUNDSUB
 *
 * FUNCTION NAME: agc_conic_mshiftr_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  OVFIND
 *   m         agc_tp_t        I    MPAC
 *   n         int             I    left shift count (negative: right)
 *
 * RETURNS: agc_tp_t -- MPAC 2^n, rounded to DP (scale of MPAC - n)
 *********************************************************************/
agc_tp_t agc_conic_mshiftr_(agc_interp_t *ip, agc_tp_t m, int n)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(n > -128 && n < 128))
    {
        return (m);
    }

    return (agc_conic_mround_(ip, agc_tp_shift(ip, m, n)));
}

/**********************************************************************
 * ROUND (ROUNDSUB, INTERPRETER.agc p. 1137) -- round MPAC+2 into MPAC,+1
 * and clear it; OVFIND on a carry out of MPAC
 *
 * FUNCTION NAME: agc_conic_mround_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  OVFIND
 *   m         agc_tp_t        I    MPAC
 *
 * RETURNS: agc_tp_t -- rounded MPAC (MPAC+2 = 0), same scale
 *********************************************************************/
agc_tp_t agc_conic_mround_(agc_interp_t *ip, agc_tp_t m)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX))
    {
        return (m);
    }

    return (agc_tp_from_dp(agc_tp_round(ip, m)));
}

/**********************************************************************
 * NORM X (TSLC, INTERPRETER.agc p. 1157) -- left-justify the TP MPAC
 * until |MPAC| >= 1/2; zero is left alone with a count of 0 (core
 * agc_tp_norm)
 *
 * FUNCTION NAME: agc_conic_mnorm_
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------------
 *   m         agc_tp_t  I    MPAC
 *   nshift    int *     O    number of left shifts (the AGC stores
 *                            -nshift in X)
 *
 * RETURNS: agc_tp_t -- normalized MPAC, scale of MPAC - nshift
 *********************************************************************/
agc_tp_t agc_conic_mnorm_(agc_tp_t m, int *nshift)
{
    if (!AGC_ASSERT(nshift != NULL))
    {
        return (m);
    }

    if (!AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX))
    {
        *nshift = 0;

        return (m);
    }

    return (agc_tp_norm(m, nshift));
}

/**********************************************************************
 * ABS (INTERPRETER.agc p. 1183) -- magnitude of MPAC
 *
 * FUNCTION NAME: agc_conic_mabs_
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------------
 *   m         agc_tp_t  I    MPAC
 *
 * RETURNS: agc_tp_t -- |MPAC|, same scale
 *********************************************************************/
agc_tp_t agc_conic_mabs_(agc_tp_t m)
{
    if (!AGC_ASSERT(m >= AGC_TP_MIN))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(m <= AGC_TP_MAX))
    {
        return (0); /* no valid result */
    }

    return ((m < 0) ? -m : m); /* complement if negative */
}

/**********************************************************************
 * SIGN X (INTERPRETER.agc p. 1151) with a DP word at X -- complement MPAC
 * if X is negative
 *
 * FUNCTION NAME: agc_conic_msign_
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------------
 *   m         agc_tp_t  I    MPAC
 *   s         agc_dp_t  I    X (only its sign is used)
 *
 * RETURNS: agc_tp_t -- MPAC with the sign of X, same scale
 *********************************************************************/
agc_tp_t agc_conic_msign_(agc_tp_t m, agc_dp_t s)
{
    if (!AGC_ASSERT(s >= AGC_DP_MIN && s <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX))
    {
        return (0); /* no valid result */
    }

    return ((s < 0) ? -m : m); /* complement if X < 0 */
}

/**********************************************************************
 * SIGN X (INTERPRETER.agc p. 1151) with an SP tag at X (GEOMSGN, SGNRDOT
 * +-.5): the AGC tests the tag word (non-zero), so the following word
 * never matters
 *
 * FUNCTION NAME: agc_conic_msign_sp_
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------------
 *   m         agc_tp_t  I    MPAC
 *   s         agc_sp_t  I    SP tag, non-zero
 *
 * RETURNS: agc_tp_t -- MPAC with the sign of the tag, same scale
 *********************************************************************/
agc_tp_t agc_conic_msign_sp_(agc_tp_t m, agc_sp_t s)
{
    if (!AGC_ASSERT(s != 0))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(m >= AGC_TP_MIN && m <= AGC_TP_MAX))
    {
        return (0); /* no valid result */
    }

    return ((s < 0) ? -m : m); /* complement if X < 0 */
}

/**********************************************************************
 * BMN (INTERPRETER.agc p. 1128) -- branch if MPAC,+1 is negative
 *
 * FUNCTION NAME: agc_conic_mbmn_
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------------
 *   m         agc_tp_t  I    MPAC (MPAC+2 not tested)
 *
 * RETURNS: bool -- true when the branch is taken
 *********************************************************************/
bool agc_conic_mbmn_(agc_tp_t m)
{
    if (!AGC_ASSERT(m >= AGC_TP_MIN))
    {
        return (false);
    }

    if (!AGC_ASSERT(m <= AGC_TP_MAX))
    {
        return (false);
    }

    return (agc_tp_to_dp(m) < 0);
}

/**********************************************************************
 * BZE (INTERPRETER.agc p. 1128) -- branch if MPAC,+1 is zero
 *
 * FUNCTION NAME: agc_conic_mbze_
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------------
 *   m         agc_tp_t  I    MPAC (MPAC+2 not tested)
 *
 * RETURNS: bool -- true when the branch is taken
 *********************************************************************/
bool agc_conic_mbze_(agc_tp_t m)
{
    if (!AGC_ASSERT(m >= AGC_TP_MIN))
    {
        return (false);
    }

    if (!AGC_ASSERT(m <= AGC_TP_MAX))
    {
        return (false);
    }

    return (agc_tp_to_dp(m) == 0);
}

/**********************************************************************
 * POLY (INTERPRETER.agc p. 1139, called through EXIT / TC POLY / DEC N)
 * -- coef[0] + coef[1] x + ... + coef[ncoef-1] x^(ncoef-1); "DEC N" in the
 * listing is ncoef - 2 (core agc_dp_poly, DMPSUB + DAS per term)
 *
 * FUNCTION NAME: agc_conic_poly_
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ----------------------------------
 *   x         agc_dp_t          I    argument (DELTIME XI 2^6, POLYCOEF
 *                                    R1A/W^2 2^0)
 *   coef      const agc_dp_t *  I    assembled 2DEC coefficients
 *   ncoef     int               I    number of coefficients, 2..16
 *
 * RETURNS: agc_dp_t -- polynomial value, scale of the coefficients
 *********************************************************************/
agc_dp_t agc_conic_poly_(agc_dp_t x, const agc_dp_t *coef, int ncoef)
{
    if (!AGC_ASSERT(coef != NULL))
    {
        return (0);
    }

    if (!AGC_ASSERT(ncoef >= 2 && ncoef <= 16))
    {
        return (0);
    }

    return (agc_dp_poly(x, coef, ncoef - 1));
}

/**********************************************************************
 * AGC: none (SET/CLEAR of SOLNSW, INFINFLG, COGAFLAG, APSESW, CONIC_
 * SUBROUTINES.agc pp. 1289-1302) -- write an output switch into its
 * flagword, where the AGC keeps it
 *
 * FUNCTION NAME: agc_conic_mirror_flag_
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ----------------------------------
 *   f         agc_conic_flag_t  I    which switch
 *   set       bool              I    new value
 *********************************************************************/
void agc_conic_mirror_flag_(agc_conic_flag_t f, bool set)
{
    agc_flag_id_t id; /* flagword bit of the switch */

    switch (f)
    {
        case AGC_CONIC_FLAG_SOLNSW:
            id = AGC_FLAG_SOLNSW;
            break;
        case AGC_CONIC_FLAG_INFINFLG:
            id = AGC_FLAG_INFINFLG;
            break;
        case AGC_CONIC_FLAG_COGAFLAG:
            id = AGC_FLAG_COGAFLAG;
            break;
        case AGC_CONIC_FLAG_APSESW:
            id = AGC_FLAG_APSESW;
            break;
        default:
            return; /* unreachable: cannot happen */
    }

    if (!AGC_ASSERT(id != 0))
    {
        return;
    }

    agc_flag_put(id, set);

    if (!AGC_ASSERT(agc_flag_test(id) == set))
    {
        return;
    }
}

/**********************************************************************
 * ABTCONIC (CONIC_SUBROUTINES.agc p. 1304) -- EXIT, TC POODOO, OCT 00607
 *
 * FUNCTION NAME: agc_conic_abtconic_
 *
 * ARGUMENTS:
 *   none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_POODOO (the AGC never returns),
 *          AGC_STATUS_BAILOUT (CSM with average G on: POODOO WHIMPERs),
 *          or AGC_STATUS_BAD_ARG if POODOO rejects the code
 *********************************************************************/
agc_status_t agc_conic_abtconic_(void)
{
    _Static_assert(AGC_CONIC_ALARM_ABTCONIC == 0607, "AGC constant");

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM || agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_poodoo(AGC_CONIC_ALARM_ABTCONIC); /* TC POODOO */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (st == AGC_STATUS_BAILOUT)
    { /* CSM POODOO with average G on: WHIMPER */
        return (st);
    }

    return (AGC_STATUS_POODOO);
}

/**********************************************************************
 * AGC: none (X1 = -2 / -10D, the MUTABLE index, as the targeting
 * programs keep it in RTX1 by SXA,1) -- conic body of an X1 value
 *
 * FUNCTION NAME: agc_conic_body_of_x1
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ----------------------------------------------
 *   x1        int   I    -2 (earth) or -10 (moon)
 *
 * RETURNS: agc_conic_body_t -- AGC_CONIC_EARTH or AGC_CONIC_MOON
 *********************************************************************/
agc_conic_body_t agc_conic_body_of_x1(int x1)
{
    if (!AGC_ASSERT(x1 == -2 || x1 == -10))
    {
        return (AGC_CONIC_EARTH);
    }

    agc_conic_body_t b = (x1 == -10) ? AGC_CONIC_MOON : AGC_CONIC_EARTH; /* MUTABLE index -10D: moon */

    if (!AGC_ASSERT(b == AGC_CONIC_EARTH || b == AGC_CONIC_MOON))
    {
        return (AGC_CONIC_EARTH);
    }

    return (b);
}
