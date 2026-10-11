/**********************************************************************
 * FILE NAME: common.c
 *
 * PURPOSE:
 *   MUTABLE, ABTCONIC, the flag mirror and the few interpreter-level
 *   helpers the word-level MPAC API (interp_words.h) does not provide:
 *   BMN/BZE on the MPAC words, SIGN with an SP tag, POLY and VXSC on
 *   stored words.
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1304-1305 (ABTCONIC, MUTABLE);
 *   INTERPRETER.agc pp. 1128 (BMN, BZE), 1139-1140 (POLY), 1145-1146
 *   (VXSC), 1151 (SIGN)
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
 *   agc_mpac_*            word-level interpretive ops (interp_words.h)
 *   agc_poodoo            POODOO (exec.h)
 *   agc_flag_put          flagword write (flags_common.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ABTCONIC: POODOO 00607, returned as AGC_STATUS_POODOO (the AGC never
 *   returns). SQRTABRT (POODOO 01302) cannot be reached from a conic path
 *   and is only asserted.
 *
 * NOTES:
 *   -0 is not modelled (interp_words.h): a word the AGC leaves as -0 is
 *   +0 here.
 *********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

/* MUTABLE (CONIC_SUBROUTINES.agc p. 1305; Luminary: CONTROLLED_CONSTANTS),
 * DP entries as assembled (major, minor; octal). VLOAD* MUTABLE,1 reads
 * six words from MUTABLE - X1: earth X1 = -2 gives entries 1..3, moon
 * X1 = -10D entries 5..7.                                                */
static const agc_dp_words_t mutable_tab[8] = {
    {022437, 016067}, /* MUE         3.986032 E10 B-36  m^3/cs^2 */
    {015625, 021042}, /* 1/MUE       .25087606 E-10 B+34         */
    {030276, 04773},  /* SQRT(MUE)   1.99650495 E5 B-18          */
    {025004, 06702},  /* 1/SQRT(MUE) .50087529 E-5 B+17          */
    {016471, 01352},  /* MUM         4.902778 E8 B-30            */
    {021412, 020500}, /* 1/MUM       .203966 E-8 B+28            */
    {025477, 03367},  /* SQRT(MUM)   2.21422176 E4 B-15          */
    {027533, 07571}   /* 1/SQRT(MUM) .45162595 E-4 B+14          */
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
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for an
 *          unknown body (the caller's input)
 *********************************************************************/
agc_status_t agc_conic_ctx_init_(agc_conic_ctx_t *ctx, agc_conic_body_t body)
{
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
 * RETURNS: agc_dp_words_t -- MU, 2^36 (earth) / 2^30 (moon) m^3/cs^2
 *********************************************************************/
agc_dp_words_t agc_conic_mu_(agc_conic_body_t body)
{
    return ((body == AGC_CONIC_EARTH) ? mutable_tab[0] : mutable_tab[4]); /* MUEARTH, else MUM */
}

/* ---- MPAC helpers beyond interp_words.h ---------------------------- */

/**********************************************************************
 * SIGN X (INTERPRETER.agc p. 1151) with an SP tag at X (GEOMSGN, SGNRDOT
 * +-.5): the AGC tests the tag word (non-zero), so the following word
 * never matters
 *
 * FUNCTION NAME: agc_conic_msign_sp_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   m         agc_tp_words_t  I    MPAC
 *   s         agc_sp_t        I    SP tag, non-zero
 *
 * RETURNS: agc_tp_words_t -- MPAC with the sign of the tag, same scale
 *********************************************************************/
agc_tp_words_t agc_conic_msign_sp_(agc_tp_words_t m, agc_sp_t s)
{
    agc_dp_words_t tag = {s, 0}; /* the tag word; its successor is not read */

    return (agc_mpac_sign(m, tag));
}

/**********************************************************************
 * BMN (INTERPRETER.agc p. 1128) -- branch if MPAC,+1 is negative (BRANCH
 * tests the major word, the minor word when the major is zero)
 *
 * FUNCTION NAME: agc_conic_mbmn_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   m         agc_tp_words_t  I    MPAC (MPAC+2 not tested)
 *
 * RETURNS: bool -- true when the branch is taken
 *********************************************************************/
bool agc_conic_mbmn_(agc_tp_words_t m)
{
    return ((m.hi < 0) || ((m.hi == 0) && (m.mid < 0)));
}

/**********************************************************************
 * BZE (INTERPRETER.agc p. 1128) -- branch if MPAC,+1 is zero
 *
 * FUNCTION NAME: agc_conic_mbze_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   m         agc_tp_words_t  I    MPAC (MPAC+2 not tested)
 *
 * RETURNS: bool -- true when the branch is taken
 *********************************************************************/
bool agc_conic_mbze_(agc_tp_words_t m)
{
    return ((m.hi == 0) && (m.mid == 0));
}

/**********************************************************************
 * TC POLY / DEC N (INTERPRETER.agc pp. 1139-1140) -- coef[0] + coef[1] x
 * + ... + coef[ncoef-1] x^(ncoef-1) on the words: x = MPAC,+1 as held is
 * moved to VBUF, then per term DMPSUB by VBUF and DAS of the next
 * coefficient into MPAC,+1 (no OVFIND: "USER'S RESPONSIBILITY TO ASSURE
 * NO OVFLOW"). "DEC N" in the listing is ncoef - 2.
 *
 * FUNCTION NAME: agc_conic_poly_
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  ----------------------------------
 *   m         agc_tp_words_t    I    MPAC = x (DELTIME XI 2^6, POLYCOEF
 *                                    R1A/W^2 2^0)
 *   coef      const agc_dp_t *  I    assembled 2DEC coefficients
 *   ncoef     int               I    number of coefficients, 2..16
 *
 * RETURNS: agc_tp_words_t -- MPAC = polynomial value, scale of the
 *          coefficients; MPAC+2 as the last DMPSUB left it
 *********************************************************************/
agc_tp_words_t agc_conic_poly_(agc_tp_words_t m, const agc_dp_t *coef, int ncoef)
{
    agc_interp_t         scratch;                                                 /* POLY's DAS leaves OVFIND alone */
    const agc_dp_words_t vbuf = agc_mpac_store(m);                                /* DXCH MPAC VBUF: x */
    agc_tp_words_t       acc  = agc_mpac_dload(agc_dp_to_words(coef[ncoef - 1])); /* A(N) */

    agc_interp_init(&scratch);

    for (int k = ncoef - 2; k >= 0; k--)
    { /* POLY2 / POLYLOOP */
        acc = agc_mpac_dmp(acc, vbuf);                               /* DMPSUB by VBUF */
        acc = agc_mpac_dad(&scratch, acc, agc_dp_to_words(coef[k])); /* DAS A(k)       */
    }

    return (acc);
}

/**********************************************************************
 * AGC: none (C boundary) -- the value of a stored DP pair, for the public
 * outputs (conic.h) and the vector instructions, which take values
 *
 * FUNCTION NAME: agc_conic_value_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   w         agc_dp_words_t  I    major, minor word
 *
 * RETURNS: agc_dp_t -- major 2^14 + minor, same scale
 *********************************************************************/
agc_dp_t agc_conic_value_(agc_dp_words_t w)
{
    return (agc_dp_from_words(w));
}

/**********************************************************************
 * VXSC with the scalar in MPAC (DVXSC, INTERPRETER.agc p. 1146) -- per
 * component: MPAC = the scalar's words, DMPSUB by the component, VROUND
 *
 * FUNCTION NAME: agc_conic_vxsc_
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  ------------------------------------
 *   ip        agc_interp_t *  I/O  OVFIND (carry of a round)
 *   m         agc_tp_words_t  I    MPAC: the scalar (MPAC+2 not read)
 *   v         agc_vec_t       I    vector operand at X
 *
 * RETURNS: agc_vec_t -- MPAC v, scales add
 *********************************************************************/
agc_vec_t agc_conic_vxsc_(agc_interp_t *ip, agc_tp_words_t m, agc_vec_t v)
{
    agc_vec_t r = v; /* result, scales of the operands added */
    int       k = 0; /* component index (unscaled) */

    for (k = 0; k < AGC_VEC_N; k++)
    {
        agc_tp_words_t c = agc_mpac_dmpr(ip, m, agc_dp_to_words(v.c[k])); /* DMPSUB, VROUND */

        r.c[k] = agc_dp_from_words(agc_mpac_store(c));
    }

    return (r);
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

    agc_flag_put(id, set);
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
    agc_conic_body_t b = (x1 == -10) ? AGC_CONIC_MOON : AGC_CONIC_EARTH; /* MUTABLE index -10D: moon */

    return (b);
}
