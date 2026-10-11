/**********************************************************************
 * FILE NAME: lambert.c
 *
 * PURPOSE:
 *   LAMBERT, the conic from R1VEC to R2VEC in TDESIRED, and ITERATOR, the
 *   bounded secant step it (and P37) uses. The independent variable is
 *   COGA, the cotangent of the flight-path angle at R1 (from the
 *   vertical).
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1266-1268 (description),
 *   pp. 1285-1286 (ITERATOR, SGNCHECK, MINCHECK, MODNGDEL, FIRSTIME,
 *   POSDEL, MAXCHECK, DELOK, NEWDEL, MODPSDEL), pp. 1296-1300 (LAMBERT,
 *   UPLIM, MAXCOGA, MINCOGA, LAMBLOOP, NEGP, HIENERGY, COMMONLM, BIGTIME,
 *   LOENERGY, SUFFCHEK, 360LAMB, NOGUESS, LOLIM, INITV, TARGETV)
 *   Luminary099 CONIC_SUBROUTINES.agc pp. 1162-1164, 1181-1182,
 *   1192-1196 (differs, see NOTES)
 *
 * EXTERNAL VARIABLES:
 *   Name      Type       I/O  Scale  Description
 *   --------  ---------  ---  -----  -------------------------------
 *   SOLNSW    flag       O    -      no acceptable solution
 *   ORDERSW   flag       I    -      ITERATOR second-order switch
 *   UN        agc_vec_t  O    2^1    accessor agc_erase_un()
 *   GEOMSGN   agc_sp_t   O    2^0    accessor agc_erase_geomsgn()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_conic_geom_       GEOM (geom.c)
 *   agc_conic_getx_       GETX (geom.c)
 *   agc_conic_deltime_    DELTIME (universal.c)
 *   agc_conic_lamenter_   LAMENTER for TARGETV (universal.c)
 *   agc_state_vehicle     CSM/LM differences
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; LAMBERT reports failure through SOLNSW (360LAMB, SUFFCHEK) and
 *   AGC_STATUS_NO_CONVERGE. AGC_STATUS_BAD_ARG for a NULL pointer,
 *   GEOMSGN = 0, an LM ITERCTR outside 1..20 or an unknown body.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the caller's job, both vehicles. LAMBLOOP is bounded by
 *   ITERCTR and by AGC_CONIC_LAMBERT_MAX_PASSES (passes through NEGP,
 *   HIENERGY and LOENERGY do not count ITERCTR in the AGC).
 *
 * NOTES:
 *   Comanche vs Luminary:
 *     entry     CSM: SSP ITERCTR 20D        LM: CLEAR SOLNSW (ITERCTR
 *                                           from SETITCTR: 20, or 5 if
 *                                           AVEGFLAG)
 *     SUFFCHEK  CSM: tolerance TDESIRED/4   LM: TDESIRED 2^-17
 *               CSM: fail -> RTNLAMB        LM: fail -> INITV (VVEC
 *                                           computed anyway)
 *     INITV     CSM: CLEAR SOLNSW           LM: (cleared at entry)
 *   DEVIATION: the CSM SUFFCHEK failure and 360LAMB leave VVEC and
 *   VTARGET unchanged in the AGC; here they are zero.
 *
 * ALGORITHM:
 *   For each COGA, LAMBLOOP forms P and R1A, gets x from GETX and the
 *   time from DELTIME, and steps COGA by ITERATOR (secant, bounded by
 *   COGAMIN/COGAMAX) until |TDESIRED - T| <= EPSILONL = TDESIRED 2^-19
 *   or ITERCTR runs out (then SUFFCHEK decides). INITV then gives VVEC
 *   and TARGETV (LAMENTER) the terminal velocity.
 *********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "internal.h"

/* LAMBERT erasables beyond the shared context. */
typedef struct lambert_state
{
    agc_dp_t tdesired;   /* TDESIRED 2^28 cs                               */
    agc_dp_t epsilonl;   /* EPSILONL 2^28 cs                               */
    agc_dp_t one_m_csth; /* 1-CSTH 2^2                                     */
    agc_dp_t csth_rho;   /* CSTH-RHO 2^7                                   */
    agc_dp_t r1_r2;      /* 0D: R1/R2 2^7                                  */
    agc_dp_t cogamax;    /* COGAMAX (14D) 2^5                              */
    agc_dp_t cogamin;    /* COGAMIN (8D) 2^5                               */
    agc_dp_t dcoga;      /* DCOGA = DELINDEP (12D) 2^5                     */
    agc_dp_t tprev;      /* TPREV = DEPREV 2^28                            */
    agc_dp_t terrlamb;   /* TERRLAMB = DELDEP 2^28                         */
    agc_sp_t tweekit; /* TWEEKIT (40D), SP 2^0 */
    agc_sp_t iterctr; /* ITERCTR (22D), count (unscaled) */
    bool     slopesw;    /* SLOPESW                                        */
} lambert_state_t;

/* Where a LAMBLOOP pass goes next. */
typedef enum lambert_next
{
    LAMB_LOOP = 0, /* back to LAMBLOOP with a new COGA              */
    LAMB_INITV,    /* converged                                      */
    LAMB_SUFFCHEK  /* out of iterations or DCOGA = 0                 */
} lambert_next_t;

/**********************************************************************
 * FIRSTIME (CONIC_SUBROUTINES.agc pp. 1285-1286) -- first step
 * (MAX - MIN) TWEEKIT with the sign of DELDEP. TWEEKIT is read as a DP
 * word: 40D = TWEEKIT and 41D = S2, which GETX has just set to 1 ("S2(41D)
 * SHOULDNT CONTAIN HI ORDER ONES").
 *
 * FUNCTION NAME: iterator_firstime
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  -----------------------------
 *   ip        agc_interp_t *          I/O  interpreter state (OVFIND)
 *   it        const agc_iterator_t *  I    MAX, MIN, TWEEKIT, DELDEP
 *
 * RETURNS: agc_tp_t -- trial DELINDEP, scale of INDEP
 *********************************************************************/
static agc_tp_t iterator_firstime(agc_interp_t *ip, const agc_iterator_t *it)
{
    if (!AGC_ASSERT(ip != NULL && it != NULL))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(it->tweekit >= 0))
    {
        return (0); /* no valid result */
    }

    const agc_dp_t tw = agc_dp_pack(it->tweekit,
                                    (agc_sp_t) 1); /* (TWEEKIT, S2), 2^0 */
    agc_tp_t m = agc_conic_mdmp_(agc_tp_from_dp(it->min),
                                 tw);     /* DMP: scale of INDEP */
    const agc_dp_t lo = agc_tp_to_dp(m); /* 0D MIN TWEEKIT, scale of INDEP */

    m = agc_conic_mdmp_(agc_tp_from_dp(it->max), tw); /* MAX TWEEKIT */

    m = agc_conic_mdsu_(ip, m, lo);

    return (agc_conic_msign_(m, it->deldep)); /* SIGN DELDEP    */
}

/**********************************************************************
 * ITERATOR (CONIC_SUBROUTINES.agc pp. 1285-1286, labels ITERATOR,
 * SGNCHECK, MINCHECK, MODNGDEL, FIRSTIME, POSDEL, MAXCHECK, DELOK, NEWDEL,
 * MODPSDEL) -- secant step DELINDEP = DELINDEP DELDEP / (DEP - DEPREV),
 * moving the bound behind INDEP in, and cutting a step that would pass
 * the other bound to 9/10 of the distance to it
 *
 * FUNCTION NAME: agc_conic_iterator
 *
 * ARGUMENTS:
 *   Argument  Type              I/O  Description (scale)
 *   --------  ----------------  ---  -----------------------------------
 *   ip        agc_interp_t *    I/O  interpreter state (OVFIND)
 *   it        agc_iterator_t *  I/O  INDEP, DEP, DEPREV, DELDEP, MAX,
 *                                    MIN, TWEEKIT, SLOPESW, ORDERSW in;
 *                                    DELINDEP, MAX/MIN, SLOPESW out
 *                                    (scales of the caller's variables)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL pointer or MAX < MIN
 *********************************************************************/
agc_status_t agc_conic_iterator(agc_interp_t *ip, agc_iterator_t *it)
{
    if (!AGC_ASSERT(ip != NULL && it != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(it->max >= it->min))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_tp_t m = 0; /* MPAC, scale of INDEP */

    if (it->slopesw)
    { /* BONCLR SLOPESW */
        it->slopesw = false;
        m           = iterator_firstime(ip, it);
    }
    else
    {
        int n1 = 0; /* NORM X1 count of DEP - DEPREV */
        int n2 = 0; /* NORM X2 count of DELINDEP */

        m = agc_conic_mnorm_(
            agc_conic_mdsu_(ip, agc_tp_from_dp(it->dep), it->deprev), &n1);
        const agc_dp_t ddep = agc_tp_to_dp(
            m); /* PDDL: DEP - DEPREV, scale of DEP */

        m = agc_conic_mnorm_(agc_tp_from_dp(it->delindep), &n2);
        m = agc_conic_mshiftr_(ip, agc_conic_mdmp_(m, it->deldep), 1 + n1 - n2);
        m = agc_tp_shift(ip, agc_conic_mddv_(ip, m, ddep), -1); /* SR1 */

        if (it->ordersw)
        { /* ABS SIGN DELDEP */
            m = agc_conic_msign_(agc_conic_mabs_(m), it->deldep);
        }
    }

    const agc_dp_t trial = agc_tp_to_dp(m); /* SGNCHECK PUSH, scale of INDEP */

    if (!agc_conic_mbmn_(m))
    { /* POSDEL         */
        if (!it->ordersw)
        {
            it->min = it->indep;
        }

        m = agc_conic_mbdsu_(ip, agc_tp_from_dp(it->indep), it->max);
        m = agc_conic_mdsu_(ip, m, trial); /* MAXCHECK       */

        if (agc_interp_test_ovf(ip) || agc_conic_mbmn_(m))
        { /* MODPSDEL */
            m = agc_conic_mdsu_(ip, agc_tp_from_dp(
                                    it->max), it->indep);
            it->delindep =
                agc_tp_to_dp(agc_conic_mdmp_(m, AGC_CONIC_DP9_10));

            return (AGC_STATUS_OK);
        }
    }
    else
    {
        if (!it->ordersw)
        {
            it->max = it->indep;
        }

        m = agc_conic_mbdsu_(ip, agc_tp_from_dp(it->indep), it->min);
        m = agc_conic_mdsu_(ip, m, trial); /* MINCHECK       */

        if (agc_interp_test_ovf(ip) || !agc_conic_mbmn_(m))
        { /* MODNGDEL */
            m = agc_conic_mdsu_(ip, agc_tp_from_dp(
                                    it->min), it->indep);
            it->delindep =
                agc_tp_to_dp(agc_conic_mdmp_(m, AGC_CONIC_DP9_10));

            return (AGC_STATUS_OK);
        }
    }

    it->delindep = trial; /* DELOK NEWDEL   */

    return (AGC_STATUS_OK);
} /* agc_conic_iterator */

/**********************************************************************
 * LAMBERT entry to MAXCOGA (CONIC_SUBROUTINES.agc pp. 1296-1297) --
 * geometry, R1/R2, 1-CSTH and the upper bound COGAMAX = SNTH/(1-CSTH) +
 * sqrt(2 R1/(R2 (1-CSTH))), limited to COGUPLIM
 *
 * FUNCTION NAME: lambert_geometry
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  --------------------------
 *   ip        agc_interp_t *            I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *         I/O  GEOM results, SNTH, CSTH 2^1
 *   in        const agc_lambert_in_t *  I    R1VEC, R2VEC
 *   ls        lambert_state_t *         O    R1/R2 2^7, 1-CSTH 2^2,
 *                                            COGAMAX 2^5
 *
 * RETURNS: bool -- false for 360LAMB (1-CSTH rounds to zero)
 *********************************************************************/
static bool lambert_geometry(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                             const agc_lambert_in_t *in, lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (false);
    }

    if (!AGC_ASSERT(in != NULL && ls != NULL))
    {
        return (false);
    }

    agc_dp_t     csth = 0; /* 0D CSTH, 2^1 */
    agc_status_t st   =
        agc_conic_geom_(
            ip, ctx, in->r1vec, in->r2vec, &ctx->snth, &csth); /* CALL GEOM */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (false);
    }

    int      n = 0;         /* MAGVEC2 NORM X1 */
    agc_tp_t m = agc_conic_mnorm_(agc_tp_from_dp(ctx->magvec2),
                                  &n);   /* NORM: 2^(29-n) */
    const agc_dp_t r2n = agc_tp_to_dp(m); /* PDDL: R2, 2^(29-n) */

    m = agc_tp_shift(ip, agc_tp_from_dp(ctx->r1), -1); /* R1 SR1, 2^30 */

    m = agc_tp_shift(ip, agc_conic_mddv_(ip, m, r2n),
                     n - 6);         /* SL* 0 -6,1 */
    ls->r1_r2 = agc_tp_to_dp(m);     /* 0D R1/R2 2^7   */
    ctx->csth = csth;                /* STORE CSTH     */
    m         =
        agc_conic_mbdsu_(ip, agc_tp_shift(ip, agc_tp_from_dp(csth), -1),
                         AGC_CONIC_D1_4);
    ls->one_m_csth = agc_tp_to_dp(m); /* 1-CSTH 2^2     */
    m              = agc_conic_mround_(ip, m);

    if (agc_conic_mbze_(m))
    { /* BZE 360LAMB    */
        return (false);
    }

    m = agc_conic_mnorm_(m, &n);
    const agc_dp_t omc_n = agc_tp_to_dp(m); /* 1-CSTH normalized, 2^(2-n) */

    m = agc_tp_shift(ip, agc_tp_from_dp(ls->r1_r2), -1);
    m = agc_tp_shift(ip, agc_conic_mddv_(ip, m, omc_n),
                     n - 3);      /* SL* 0 -3,1 */
    const agc_dp_t root =
        agc_tp_to_dp(agc_conic_msqrt_(ip, m)); /* 2D 2^5 */

    m = agc_tp_shift(ip, agc_tp_from_dp(ctx->snth), -6);
    m = agc_conic_mdad_(ip, agc_conic_mddv_(ip, m,
                                            ls->one_m_csth), root);
    ls->cogamax = agc_tp_to_dp(m); /* STORE COGAMAX  */

    if (agc_interp_test_ovf(ip))
    { /* BOV UPLIM      */
        ls->cogamax = AGC_CONIC_COGUPLIM;
    }
    else if (!agc_conic_mbmn_(m) &&
             !agc_conic_mbmn_(agc_conic_mdsu_(ip, m, AGC_CONIC_COGUPLIM)))
    {
        ls->cogamax = AGC_CONIC_COGUPLIM; /* UPLIM          */
    }

    return (true);
} /* lambert_geometry */

/**********************************************************************
 * MAXCOGA, LOLIM, MINCOGA, NOGUESS (CONIC_SUBROUTINES.agc pp. 1297-1300)
 * -- CSTH-RHO, the lower bound COGAMIN = (CSTH - R1/R2)/SNTH (COGLOLIM
 * beyond 180 degrees or on overflow), TWEEKIT and the first COGA (input
 * guess, or the midpoint of the bounds)
 *
 * FUNCTION NAME: lambert_bounds
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  --------------------------
 *   ip        agc_interp_t *            I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *         I/O  CSTH, SNTH in; COGA 2^5 out
 *   in        const agc_lambert_in_t *  I    GEOMSGN, GUESSW, COGA guess
 *   ls        lambert_state_t *         I/O  CSTH-RHO 2^7, COGAMIN 2^5,
 *                                            TWEEKIT, DCOGA 2^5
 *********************************************************************/
static void lambert_bounds(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                           const agc_lambert_in_t *in, lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return;
    }

    if (!AGC_ASSERT(in != NULL && ls != NULL))
    {
        return;
    }

    agc_tp_t m = agc_tp_shift(
        ip, agc_tp_from_dp(ctx->csth), -6); /* CSTH SR 6, 2^7 */

    ls->csth_rho = agc_tp_to_dp(agc_conic_mdsu_(ip, m, ls->r1_r2));
    ls->cogamin  = AGC_CONIC_COGLOLIM; /* LOLIM          */

    if (in->geomsgn >= 0)
    { /* BMN LOLIM      */
        m = agc_tp_shift(ip, agc_tp_from_dp(ls->csth_rho), 1);
        m = agc_conic_mddv_(ip, m, ctx->snth);

        if (!agc_interp_test_ovf(ip))
        {                                       /* BOV LOLIM      */
            ls->cogamin = agc_tp_to_dp(m); /* MINCOGA        */
        }
    }

    if (in->guessw)
    { /* NOGUESS        */
        ls->tweekit = AGC_CONIC_TWEEKIT_NOGUESS;
        const agc_dp_t half_min = agc_tp_to_dp(
            agc_tp_shift(
                ip, agc_tp_from_dp(ls->cogamin), -1)); /* COGAMIN SR1, 2^6 */

        m         = agc_tp_shift(ip, agc_tp_from_dp(ls->cogamax), -1);
        ctx->coga = agc_tp_to_dp(agc_conic_mdad_(ip, m, half_min));
        ls->dcoga = ctx->coga; /* STCALL DCOGA   */
    }
    else
    {
        ls->tweekit = AGC_CONIC_TWEEKIT_GUESS;
        ctx->coga   = in->coga;
    }
} /* lambert_bounds */

/**********************************************************************
 * LAMBLOOP to the GETX call (CONIC_SUBROUTINES.agc pp. 1297-1298) --
 * P = (1-CSTH)/(SNTH COGA - (CSTH-RHO)) and R1A = 2 - P (1 + COGA^2)
 *
 * FUNCTION NAME: lambert_p_r1a
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *        I/O  COGA, SNTH in; P 2^4, R1A 2^6
 *                                           out
 *   ls        const lambert_state_t *  I    1-CSTH, CSTH-RHO
 *
 * RETURNS: bool -- false for NEGP (P <= 0)
 *********************************************************************/
static bool lambert_p_r1a(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                          const lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (false);
    }

    if (!AGC_ASSERT(ls != NULL))
    {
        return (false);
    }

    int      n = 0; /* NORM X1 count */
    agc_tp_t m = agc_conic_mdmp_(
        agc_tp_from_dp(ctx->coga), ctx->snth); /* COGA SNTH, 2^6 */

    m = agc_conic_mdsu_(ip, agc_tp_shift(ip, m, -1), ls->csth_rho);
    const agc_dp_t den = agc_tp_to_dp(
        agc_conic_mnorm_(m, &n)); /* 0D, 2^(7-n) */

    m = agc_tp_shift(ip, agc_tp_from_dp(ls->one_m_csth),
                     n - 9);                       /* SL* 0 -9D,1 */
    m = agc_conic_mddv_(ip, m, den);

    if (agc_conic_mbmn_(m) || agc_conic_mbze_(m))
    { /* BMN BZE NEGP   */
        return (false);
    }

    ctx->p = agc_tp_to_dp(m); /* STODL P / COGA */
    m      = agc_conic_mdad_(ip, agc_conic_mdsq_(agc_tp_from_dp(ctx->coga)),
                             AGC_CONIC_D1_1024);
    m        = agc_conic_mdmp_(agc_conic_mnorm_(m, &n), ctx->p);
    m        = agc_tp_shift(ip, m, 8 - n); /* SR* 0 -8D,1    */
    ctx->r1a = agc_tp_to_dp(agc_conic_mbdsu_(ip, m, AGC_CONIC_D1_32));

    return (true);
}

/**********************************************************************
 * NEGP, HIENERGY, BIGTIME, LOENERGY, COMMONLM (CONIC_SUBROUTINES.agc
 * pp. 1298-1299) -- make COGA a bound (low energy: COGAMAX, high energy:
 * COGAMIN), halve DCOGA and step back by it
 *
 * FUNCTION NAME: lambert_retreat
 *
 * ARGUMENTS:
 *   Argument     Type               I/O  Description (scale)
 *   -----------  -----------------  ---  ------------------------------
 *   ip           agc_interp_t *     I/O  interpreter state (OVFIND)
 *   ctx          agc_conic_ctx_t *  I/O  COGA 2^5
 *   ls           lambert_state_t *  I/O  COGAMAX or COGAMIN, DCOGA 2^5
 *   high_energy  bool               I    true: HIENERGY (COGAMIN = COGA)
 *
 * RETURNS: lambert_next_t -- LAMB_LOOP, or LAMB_SUFFCHEK when DCOGA has
 *          halved to zero
 *********************************************************************/
static lambert_next_t lambert_retreat(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                      lambert_state_t *ls, bool high_energy)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (LAMB_SUFFCHEK);
    }

    if (!AGC_ASSERT(ls != NULL))
    {
        return (LAMB_SUFFCHEK);
    }

    if (high_energy)
    {
        ls->cogamin = ctx->coga; /* HIENERGY       */
    }
    else
    {
        ls->cogamax = ctx->coga; /* LOENERGY       */
    }

    ls->dcoga = agc_tp_to_dp(
        agc_tp_shift(ip, agc_tp_from_dp(ls->dcoga), -1));

    if (ls->dcoga == 0)
    { /* BZE SUFFCHEK   */
        return (LAMB_SUFFCHEK);
    }

    ctx->coga = agc_tp_to_dp(
        agc_conic_mbdsu_(ip, agc_tp_from_dp(ls->dcoga), ctx->coga));

    return (LAMB_LOOP);
}

/**********************************************************************
 * End of LAMBLOOP (CONIC_SUBROUTINES.agc p. 1298) -- CALL ITERATOR, and
 * COGA = COGA + DCOGA unless DCOGA is zero
 *
 * FUNCTION NAME: lambert_step
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *  I/O  COGA 2^5, T 2^28 cs
 *   ls        lambert_state_t *  I/O  TPREV, TERRLAMB, DCOGA, bounds,
 *                                     TWEEKIT, SLOPESW
 *
 * RETURNS: lambert_next_t -- LAMB_LOOP, or LAMB_SUFFCHEK when DCOGA = 0
 *********************************************************************/
static lambert_next_t lambert_step(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                   lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (LAMB_SUFFCHEK);
    }

    if (!AGC_ASSERT(ls != NULL))
    {
        return (LAMB_SUFFCHEK);
    }

    agc_iterator_t it; /* ITERATOR arguments: COGA 2^5, T 2^28 cs */

    it.indep    = ctx->coga;
    it.dep      = ctx->t;
    it.deprev   = ls->tprev;
    it.deldep   = ls->terrlamb;
    it.delindep = ls->dcoga;
    it.max      = ls->cogamax;
    it.min      = ls->cogamin;
    it.tweekit  = ls->tweekit;
    it.slopesw  = ls->slopesw;
    it.ordersw  = agc_flag_test(AGC_FLAG_ORDERSW); /* never set in Apollo 11 */
    agc_status_t st = agc_conic_iterator(ip, &it); /* CALL ITERATOR */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (LAMB_SUFFCHEK);
    }

    ls->dcoga   = it.delindep;
    ls->cogamax = it.max;
    ls->cogamin = it.min;
    ls->slopesw = it.slopesw;

    if (ls->dcoga == 0)
    { /* DLOAD MPAC BZE */
        return (LAMB_SUFFCHEK);
    }

    ctx->coga = agc_tp_to_dp(
        agc_conic_mdad_(ip, agc_tp_from_dp(ls->dcoga), ctx->coga));

    return (LAMB_LOOP);
} /* lambert_step */

/**********************************************************************
 * LAMBLOOP (CONIC_SUBROUTINES.agc pp. 1297-1298) -- one pass: P, R1A,
 * GETX, DELTIME, the convergence test, CHECKCTR and the ITERATOR step
 *
 * FUNCTION NAME: lambert_pass
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *  I/O  conic erasables
 *   ls        lambert_state_t *  I/O  LAMBERT erasables, ITERCTR
 *
 * RETURNS: lambert_next_t -- LAMB_LOOP, LAMB_INITV (converged) or
 *          LAMB_SUFFCHEK
 *********************************************************************/
static lambert_next_t lambert_pass(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                   lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (LAMB_SUFFCHEK);
    }

    if (!AGC_ASSERT(ls != NULL))
    {
        return (LAMB_SUFFCHEK);
    }

    if (!lambert_p_r1a(ip, ctx, ls))
    { /* NEGP           */
        return (lambert_retreat(ip, ctx, ls, ls->dcoga < 0));
    }

    if (agc_interp_test_ovf(ip))
    { /* BOV HIENERGY   */
        return (lambert_retreat(ip, ctx, ls, true));
    }

    agc_status_t st = agc_conic_getx_(ip, ctx); /* CALL GETX */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (LAMB_SUFFCHEK);
    }

    ls->tprev = ctx->t; /* DLOAD T STODL TPREV */

    if (ctx->infinflg)
    { /* BON INFINFLG NEGP */
        return (lambert_retreat(ip, ctx, ls, ls->dcoga < 0));
    }

    st = agc_conic_deltime_(ip, ctx);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (LAMB_SUFFCHEK);
    }

    if (agc_interp_test_ovf(ip))
    { /* BOV BIGTIME    */
        ctx->t = ls->tprev;

        return (lambert_retreat(ip, ctx, ls, false));
    }

    agc_tp_t m = agc_conic_mbdsu_(
        ip, agc_tp_from_dp(ctx->t), ls->tdesired); /* TDESIRED - T, 2^28 cs */

    ls->terrlamb = agc_tp_to_dp(m); /* TERRLAMB       */
    m            = agc_conic_mbdsu_(ip, agc_conic_mabs_(m), ls->epsilonl);

    if (!agc_conic_mbmn_(m))
    { /* BPL INITV      */
        return (LAMB_INITV);
    }

    ls->iterctr = (agc_sp_t) (ls->iterctr - 1); /* RTB CHECKCTR   */

    if (ls->iterctr == 0)
    { /* BHIZ SUFFCHEK  */
        return (LAMB_SUFFCHEK);
    }

    return (lambert_step(ip, ctx, ls));
} /* lambert_pass */

/**********************************************************************
 * SUFFCHEK (CONIC_SUBROUTINES.agc p. 1299) -- accept the last COGA if
 * |TERRLAMB| <= TDESIRED/4 (CSM) or TDESIRED 2^-17 (LM), plus ONEBIT
 *
 * FUNCTION NAME: lambert_suffchek
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ls        const lambert_state_t *  I    TERRLAMB, TDESIRED 2^28 cs
 *
 * RETURNS: bool -- true when the time error is acceptable (BPL INITV)
 *********************************************************************/
static bool lambert_suffchek(agc_interp_t *ip, const lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ls != NULL))
    {
        return (false);
    }

    const bool csm = (agc_state_vehicle() ==
                      AGC_VEHICLE_CSM); /* Comanche tolerance */
    agc_tp_t m = agc_conic_mabs_(
        agc_tp_from_dp(ls->terrlamb));  /* ABS TERRLAMB, 2^28 cs */
    const agc_dp_t err = agc_tp_to_dp(m); /* PDDL: |TERRLAMB| 2^28 cs */

    m = agc_conic_mdmp_(agc_tp_from_dp(ls->tdesired),
                        csm ? AGC_CONIC_D1_4 : /* CSM .25  */
                        AGC_CONIC_BEE17);      /* LM 2^-17 */

    m = agc_conic_mdsu_(ip, agc_conic_mdad_(ip, m, AGC_CONIC_ONEBIT), err);

    if (!AGC_ASSERT(ls->tdesired >= 0))
    {
        return (false);
    }

    return (!agc_conic_mbmn_(m)); /* BPL INITV      */
}

/**********************************************************************
 * INITV (CONIC_SUBROUTINES.agc p. 1300) -- VVEC = VTAN (UN x UR1) +
 * VTAN COGA UR1 with VTAN = sqrt(MU P / R1)
 *
 * FUNCTION NAME: lambert_initv
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       const agc_conic_ctx_t *  I    R1, P, COGA, ROOTMU, UR1, UN
 *
 * RETURNS: agc_vec_t -- VVEC, 2^7 / 2^5 m/cs
 *********************************************************************/
static agc_vec_t lambert_initv(agc_interp_t *ip, const agc_conic_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT(ctx->r1 > 0))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    int      n = 0;         /* NORM X1 count of R1 */
    agc_tp_t m = agc_conic_mnorm_(agc_tp_from_dp(ctx->r1),
                                  &n);   /* NORM: 2^(29-n) */
    const agc_dp_t r1n = agc_tp_to_dp(m); /* PDDL: R1, 2^(29-n) */

    m = agc_tp_shift(ip, agc_tp_from_dp(ctx->p), -1); /* P SR1, 2^5 */

    m = agc_tp_shift(ip, agc_conic_mddv_(ip, m, r1n),
                     n - 4);               /* SL* 0 -4,1 */
    m = agc_conic_mdmp_(agc_conic_msqrt_(ip, m), ctx->root_mu);
    m = agc_tp_shift(ip, m, 1);
    const agc_dp_t vtan = agc_tp_to_dp(m); /* 0D VTAN 2^7    */

    m = agc_tp_shift(ip, agc_conic_mdmp_(m, ctx->coga), 5);
    const agc_vec_t vrad = agc_vec_scale(
        ip, ctx->ur1, agc_tp_to_dp(m)); /* VTAN COGA UR1, 2^8 */
    agc_vec_t v = agc_vec_shl(
        ip, agc_vec_scale(ip, ctx->un, vtan), 1); /* VTAN UN VSL1, 2^7 */

    v = agc_vec_add(ip, agc_vec_cross(ip, v, ctx->ur1), vrad);

    return (agc_vec_shl(ip, v, 1)); /* VSL1           */
}

/**********************************************************************
 * LAMBERT entry (CONIC_SUBROUTINES.agc p. 1296) -- ITERCTR (CSM) or
 * CLEAR SOLNSW (LM), MUTABLE, EPSILONL = TDESIRED 2^-19, SET SLOPESW
 *
 * FUNCTION NAME: lambert_entry
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  --------------------------
 *   ip        agc_interp_t *            I/O  interpreter state (OVFIND
 *                                            cleared, BOV +1)
 *   ctx       agc_conic_ctx_t *         O    MUTABLE, NORMSW, UN, GEOMSGN
 *   in        const agc_lambert_in_t *  I    caller's inputs
 *   ls        lambert_state_t *         O    LAMBERT erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL pointer, GEOMSGN = 0, an LM ITERCTR outside 1..20 or an
 *          unknown body
 *********************************************************************/
static agc_status_t lambert_entry(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                  const agc_lambert_in_t *in,
                                  lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(in != NULL && ls != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(in->geomsgn != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_conic_ctx_init_(ctx, in->body); /* VLOAD* MUTABLE,1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_interp_clear_ovf(ip); /* SETPD 0D, BOV +1 */
    lambert_state_t z = {0};

    *ls = z;

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        ls->iterctr = AGC_CONIC_LAMBERT_ITERCTR; /* SSP ITERCTR 20D */
    }
    else
    {
        if (!AGC_ASSERT(in->iterctr >= 1 &&
                        in->iterctr <= AGC_CONIC_LAMBERT_ITERCTR))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        ls->iterctr = in->iterctr; /* set by SETITCTR */
        agc_conic_mirror_flag_(AGC_CONIC_FLAG_SOLNSW, false); /* mirror only */
    }

    ls->tdesired = in->tdesired;
    ls->epsilonl = agc_tp_to_dp(
        agc_conic_mdmpr_(ip, agc_tp_from_dp(in->tdesired), AGC_CONIC_BEE19));
    ls->slopesw = true; /* SET SLOPESW    */
    /* DEVIATION: TERRLAMB (an erasable) and DCOGA (push-down 12D) are not
     * set by LAMBERT before an early NEGP/SUFFCHEK can read them; the AGC
     * uses whatever they last held. Here DCOGA = 0 and TERRLAMB = TDESIRED,
     * so such an exit reports no solution.                               */
    ls->terrlamb = in->tdesired;
    ctx->normsw  = in->normsw;
    ctx->un      = in->un;
    ctx->geomsgn = in->geomsgn;

    return (AGC_STATUS_OK);
} /* lambert_entry */

/* UN and GEOMSGN as LAMBERT left them in erasable (LM RASTEER, P40-P42
 * Lambert steering, reads them after the last INITVEL / LAMBERT).        */
static agc_vec_t g_un;      /* UN, 2^1                                */
static agc_sp_t  g_geomsgn; /* GEOMSGN SP 2^0: +.5 short / -.5 long way */

/**********************************************************************
 * AGC: none (accessor of erasable UN, CONIC_SUBROUTINES.agc p. 1306) --
 * UN as LAMBERT left it (LM RASTEER reads it after INITVEL/LAMBERT)
 *
 * FUNCTION NAME: agc_erase_un
 *
 * ARGUMENTS:
 *   none
 *
 * RETURNS: agc_vec_t -- UN, unit normal, 2^1
 *********************************************************************/
agc_vec_t agc_erase_un(void)
{
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        if (!AGC_ASSERT(g_un.c[i] >= AGC_DP_MIN && g_un.c[i] <= AGC_DP_MAX))
        {
            return (agc_vec_zero());
        }
    }

    return (g_un);
}

/**********************************************************************
 * AGC: none (accessor of erasable GEOMSGN, CONIC_SUBROUTINES.agc p. 1306)
 * -- GEOMSGN as LAMBERT left it
 *
 * FUNCTION NAME: agc_erase_geomsgn
 *
 * ARGUMENTS:
 *   none
 *
 * RETURNS: agc_sp_t -- GEOMSGN, +.5 short way / -.5 long way (SP, 2^0)
 *********************************************************************/
agc_sp_t agc_erase_geomsgn(void)
{
    if (!AGC_ASSERT(g_geomsgn >= AGC_SP_MIN && g_geomsgn <= AGC_SP_MAX))
    {
        return (0);
    }

    return (g_geomsgn);
}

/**********************************************************************
 * AGC: none (C output record) -- copy the LAMBERT debris to *out and
 * keep UN, GEOMSGN for the erasable accessors
 *
 * FUNCTION NAME: lambert_output
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ctx       const agc_conic_ctx_t *  I    conic erasables
 *   ls        const lambert_state_t *  I    1-CSTH
 *   out       agc_lambert_out_t *      O    debris (see conic.h)
 *********************************************************************/
static void lambert_output(const agc_conic_ctx_t *ctx,
                           const lambert_state_t *ls, agc_lambert_out_t *out)
{
    if (!AGC_ASSERT(ctx != NULL && ls != NULL))
    {
        return;
    }

    if (!AGC_ASSERT(out != NULL))
    {
        return;
    }

    out->snth       = ctx->snth;
    out->csth       = ctx->csth;
    out->one_m_csth = ls->one_m_csth;
    out->coga       = ctx->coga;
    out->p          = ctx->p;
    out->r1a        = ctx->r1a;
    out->r1         = ctx->r1;
    out->ur1        = ctx->ur1;
    out->u2         = ctx->u2;
    out->un         = ctx->un;
    g_un            = ctx->un;      /* STOVL UN (erasable)    */
    g_geomsgn       = ctx->geomsgn; /* GEOMSGN (erasable)     */
}

/**********************************************************************
 * AGC: none (SET/CLEAR SOLNSW, CONIC_SUBROUTINES.agc pp. 1299-1300) --
 * set SOLNSW in *out and in the flagword
 *
 * FUNCTION NAME: lambert_solnsw
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  -------------------------------
 *   out       agc_lambert_out_t *  O    solnsw
 *   set       bool                 I    new SOLNSW value
 *********************************************************************/
static void lambert_solnsw(agc_lambert_out_t *out, bool set)
{
    if (!AGC_ASSERT(out != NULL))
    {
        return;
    }

    out->solnsw = set;
    agc_conic_mirror_flag_(AGC_CONIC_FLAG_SOLNSW, set);
}

/**********************************************************************
 * LAMBERT (CONIC_SUBROUTINES.agc pp. 1296-1300, label LAMBERT) -- the
 * velocity at R1VEC for a conic to R2VEC in TDESIRED
 *
 * FUNCTION NAME: agc_conic_lambert
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  --------------------------
 *   ip        agc_interp_t *            I/O  interpreter state (OVFIND)
 *   in        const agc_lambert_in_t *  I    R1VEC, R2VEC, TDESIRED,
 *                                            GEOMSGN, GUESSW, COGA, NORMSW,
 *                                            UN, VTARGTAG, ITERCTR (LM)
 *   out       agc_lambert_out_t *       O    VVEC, VTARGET 2^7 / 2^5 m/cs,
 *                                            SOLNSW and the debris
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_NO_CONVERGE when
 *          SOLNSW is set (360LAMB, SUFFCHEK failed); AGC_STATUS_BAD_ARG
 *          as for lambert_entry
 *********************************************************************/
agc_status_t agc_conic_lambert(agc_interp_t *ip, const agc_lambert_in_t *in,
                               agc_lambert_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_conic_ctx_t ctx; /* conic erasables */
    lambert_state_t ls; /* LAMBERT erasables */
    agc_status_t    st = lambert_entry(ip, &ctx, in, &ls); /* entry checks */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    out->vvec    = agc_vec_zero();
    out->vtarget = agc_vec_zero();

    if (!lambert_geometry(ip, &ctx, in, &ls))
    { /* 360LAMB        */
        lambert_output(&ctx, &ls, out);
        lambert_solnsw(out, true);

        return (AGC_STATUS_NO_CONVERGE);
    }

    lambert_bounds(ip, &ctx, in, &ls);
    lambert_next_t next = LAMB_SUFFCHEK; /* if passes run out */

    for (int pass = 0; pass < AGC_CONIC_LAMBERT_MAX_PASSES; pass++)
    {
        next = lambert_pass(ip, &ctx, &ls); /* LAMBLOOP       */

        if (next != LAMB_LOOP)
        {
            break;
        }
    }

    if (next == LAMB_LOOP)
    {
        next = LAMB_SUFFCHEK;
    } /* DEVIATION: bound */

    lambert_output(&ctx, &ls, out);
    const bool ok = (next == LAMB_INITV) ||
                    lambert_suffchek(ip, &ls); /* solution accepted */
    const bool csm = (agc_state_vehicle() ==
                      AGC_VEHICLE_CSM); /* CSM: RTNLAMB on failure */

    lambert_solnsw(out, !ok); /* SETGO SOLNSW / CLEAR */

    if (!ok && csm)
    { /* CSM: RTNLAMB   */
        return (AGC_STATUS_NO_CONVERGE);
    }

    out->vvec = lambert_initv(ip, &ctx); /* INITV: VVEC    */
    ctx.vvec  = out->vvec;

    if (in->vtargtag == 0)
    { /* TARGETV        */
        st = agc_conic_lamenter_(ip, &ctx, ctx.magvec2, &out->vtarget);

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (ok ? AGC_STATUS_OK : /* SOLNSW clear / set */
            AGC_STATUS_NO_CONVERGE);
} /* agc_conic_lambert */
