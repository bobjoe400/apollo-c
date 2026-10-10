/*
 * lambert.c -- LAMBERT (CONIC_SUBROUTINES.agc, Comanche 055
 * pp. 1266-1268 description, code pp. 1296-1300; Luminary 099
 * pp. 1162-1164, 1192-1196) and ITERATOR (p. 1285-1286).
 *
 * LAMBERT finds the initial velocity for a conic from R1VEC to R2VEC in
 * TDESIRED. The independent variable is COGA, the cotangent of the flight-
 * path angle at R1 (from the vertical); for each COGA it forms P and R1A,
 * gets x from GETX and the time from DELTIME, and steps COGA by ITERATOR
 * (secant, bounded by COGAMIN/COGAMAX) until |TDESIRED - T| <= EPSILONL =
 * TDESIRED 2^-19 or ITERCTR runs out (then SUFFCHEK decides).
 *
 * Comanche vs Luminary:
 *   entry     CSM: SSP ITERCTR 20D        LM: CLEAR SOLNSW (ITERCTR from
 *                                          SETITCTR: 20, or 5 if AVEGFLAG)
 *   SUFFCHEK  CSM: tolerance TDESIRED/4   LM: TDESIRED 2^-17
 *             CSM: fail -> RTNLAMB        LM: fail -> INITV (VVEC anyway)
 *   INITV     CSM: CLEAR SOLNSW           LM: (cleared at entry)
 */
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
    agc_sp_t tweekit;    /* TWEEKIT (40D)                                  */
    agc_sp_t iterctr;    /* ITERCTR (22D)                                  */
    bool     slopesw;    /* SLOPESW                                        */
} lambert_state_t;

/* Where a LAMBLOOP pass goes next. */
typedef enum lambert_next
{
    LAMB_LOOP = 0, /* back to LAMBLOOP with a new COGA              */
    LAMB_INITV,    /* converged                                      */
    LAMB_SUFFCHEK  /* out of iterations or DCOGA = 0                 */
} lambert_next_t;

/* ITERATOR FIRSTIME (p. 1285-1286): first step (MAX - MIN) TWEEKIT with
 * the sign of DELDEP. TWEEKIT is read as a DP word: 40D (S1) = TWEEKIT
 * and 41D (S2), which GETX has just set to 1 ("S2(41D) SHOULDNT CONTAIN
 * HI ORDER ONES").                                                       */
static agc_tp_t iterator_firstime(agc_interp_t *ip, const agc_iterator_t *it)
{
    if (!AGC_ASSERT(ip != NULL && it != NULL))
    {
        return 0; /* no valid result */
    }

    if (!AGC_ASSERT(it->tweekit >= 0))
    {
        return 0; /* no valid result */
    }

    const agc_dp_t tw = agc_dp_pack(it->tweekit, (agc_sp_t) 1);
    const agc_dp_t lo =
        agc_conic_mstore_(agc_conic_mdmp_(agc_conic_mload_(it->min), tw));
    agc_tp_t m = agc_conic_mdmp_(agc_conic_mload_(it->max), tw);

    m = agc_conic_mdsu_(ip, m, lo);

    return agc_conic_msign_(m, it->deldep); /* SIGN DELDEP    */
}

/*
 * ITERATOR (CONIC_SUBROUTINES.agc pp. 1285-1286, labels ITERATOR,
 * SGNCHECK, MINCHECK, MODNGDEL, FIRSTIME, POSDEL, MAXCHECK, DELOK, NEWDEL,
 * MODPSDEL). Secant step DELINDEP = DELINDEP DELDEP / (DEP - DEPREV),
 * moving the bound behind INDEP in, and cutting a step that would pass
 * the other bound to 9/10 of the distance to it.
 */
agc_status_t agc_conic_iterator(agc_interp_t *ip, agc_iterator_t *it)
{
    if (!AGC_ASSERT(ip != NULL && it != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(it->max >= it->min))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_tp_t m = 0;

    if (it->slopesw)
    { /* BONCLR SLOPESW */
        it->slopesw = false;
        m           = iterator_firstime(ip, it);
    }
    else
    {
        int n1 = 0;
        int n2 = 0;

        m = agc_conic_mnorm_(
            agc_conic_mdsu_(ip, agc_conic_mload_(it->dep), it->deprev), &n1);
        const agc_dp_t ddep = agc_conic_mstore_(m); /* PDDL           */

        m = agc_conic_mnorm_(agc_conic_mload_(it->delindep), &n2);
        m = agc_conic_mshiftr_(ip, agc_conic_mdmp_(m, it->deldep), 1 + n1 - n2);
        m = agc_conic_mshift_(ip, agc_conic_mddv_(ip, m, ddep), -1); /* SR1 */

        if (it->ordersw)
        { /* ABS SIGN DELDEP */
            m = agc_conic_msign_(agc_conic_mabs_(m), it->deldep);
        }
    }

    const agc_dp_t trial = agc_conic_mstore_(m); /* SGNCHECK PUSH  */

    if (!agc_conic_mbmn_(m))
    { /* POSDEL         */
        if (!it->ordersw)
        {
            it->min = it->indep;
        }

        m = agc_conic_mbdsu_(ip, agc_conic_mload_(it->indep), it->max);
        m = agc_conic_mdsu_(ip, m, trial); /* MAXCHECK       */

        if (agc_interp_test_ovf(ip) || agc_conic_mbmn_(m))
        { /* MODPSDEL */
            m = agc_conic_mdsu_(ip, agc_conic_mload_(
                                    it->max), it->indep);
            it->delindep =
                agc_conic_mstore_(agc_conic_mdmp_(m, AGC_CONIC_DP9_10));

            return AGC_STATUS_OK;
        }
    }
    else
    {
        if (!it->ordersw)
        {
            it->max = it->indep;
        }

        m = agc_conic_mbdsu_(ip, agc_conic_mload_(it->indep), it->min);
        m = agc_conic_mdsu_(ip, m, trial); /* MINCHECK       */

        if (agc_interp_test_ovf(ip) || !agc_conic_mbmn_(m))
        { /* MODNGDEL */
            m = agc_conic_mdsu_(ip, agc_conic_mload_(
                                    it->min), it->indep);
            it->delindep =
                agc_conic_mstore_(agc_conic_mdmp_(m, AGC_CONIC_DP9_10));

            return AGC_STATUS_OK;
        }
    }

    it->delindep = trial; /* DELOK NEWDEL   */

    return AGC_STATUS_OK;
}

/* LAMBERT entry to MAXCOGA (p. 1296-1297): geometry, R1/R2, 1-CSTH and the
 * upper bound COGAMAX = SNTH/(1-CSTH) + sqrt(2 R1/(R2 (1-CSTH))), limited
 * to COGUPLIM. Returns false for 360LAMB (1-CSTH rounds to zero).         */
static bool lambert_geometry(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                             const agc_lambert_in_t *in, lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return false;
    }

    if (!AGC_ASSERT(in != NULL && ls != NULL))
    {
        return false;
    }

    agc_dp_t     csth = 0;
    agc_status_t st   =
        agc_conic_geom_(ip, ctx, in->r1vec, in->r2vec, &ctx->snth, &csth);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return false;
    }

    int            n   = 0; /* MAGVEC2 NORM X1 */
    const agc_dp_t r2n =
        agc_conic_mstore_(agc_conic_mnorm_(agc_conic_mload_(ctx->magvec2), &n));
    agc_tp_t m = agc_conic_mshift_(ip, agc_conic_mload_(ctx->r1), -1);

    m = agc_conic_mshift_(ip, agc_conic_mddv_(ip, m, r2n),
                          n - 6);         /* SL* 0 -6,1 */
    ls->r1_r2 = agc_conic_mstore_(m);     /* 0D R1/R2 2^7   */
    ctx->csth = csth;                     /* STORE CSTH     */
    m         =
        agc_conic_mbdsu_(ip, agc_conic_mshift_(ip, agc_conic_mload_(csth), -1),
                         AGC_CONIC_D1_4);
    ls->one_m_csth = agc_conic_mstore_(m); /* 1-CSTH 2^2     */
    m              = agc_conic_mround_(ip, m);

    if (agc_conic_mbze_(m))
    { /* BZE 360LAMB    */
        return false;
    }

    m = agc_conic_mnorm_(m, &n);
    const agc_dp_t omc_n = agc_conic_mstore_(m);

    m = agc_conic_mshift_(ip, agc_conic_mload_(ls->r1_r2), -1);
    m = agc_conic_mshift_(ip, agc_conic_mddv_(ip, m, omc_n),
                          n - 3); /* SL* 0 -3,1 */
    const agc_dp_t root =
        agc_conic_mstore_(agc_conic_msqrt_(ip, m)); /* 2D 2^5 */

    m = agc_conic_mshift_(ip, agc_conic_mload_(ctx->snth), -6);
    m = agc_conic_mdad_(ip, agc_conic_mddv_(ip, m,
                                            ls->one_m_csth), root);
    ls->cogamax = agc_conic_mstore_(m); /* STORE COGAMAX  */

    if (agc_interp_test_ovf(ip))
    { /* BOV UPLIM      */
        ls->cogamax = AGC_CONIC_COGUPLIM;
    }
    else if (!agc_conic_mbmn_(m) &&
             !agc_conic_mbmn_(agc_conic_mdsu_(ip, m, AGC_CONIC_COGUPLIM)))
    {
        ls->cogamax = AGC_CONIC_COGUPLIM; /* UPLIM          */
    }

    return true;
}

/* MAXCOGA, LOLIM, MINCOGA, NOGUESS (p. 1297-1300): CSTH-RHO, the lower
 * bound COGAMIN = (CSTH - R1/R2)/SNTH (COGLOLIM for > 180 deg or on
 * overflow), TWEEKIT and the first COGA (input guess, or the midpoint).   */
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

    agc_tp_t m = agc_conic_mshift_(ip, agc_conic_mload_(ctx->csth), -6);

    ls->csth_rho = agc_conic_mstore_(agc_conic_mdsu_(ip, m, ls->r1_r2));
    ls->cogamin  = AGC_CONIC_COGLOLIM; /* LOLIM          */

    if (in->geomsgn >= 0)
    { /* BMN LOLIM      */
        m = agc_conic_mshift_(ip, agc_conic_mload_(ls->csth_rho), 1);
        m = agc_conic_mddv_(ip, m, ctx->snth);

        if (!agc_interp_test_ovf(ip))
        {                                       /* BOV LOLIM      */
            ls->cogamin = agc_conic_mstore_(m); /* MINCOGA        */
        }
    }

    if (in->guessw)
    { /* NOGUESS        */
        ls->tweekit = AGC_CONIC_TWEEKIT_NOGUESS;
        const agc_dp_t half_min = agc_conic_mstore_(
            agc_conic_mshift_(ip, agc_conic_mload_(ls->cogamin), -1));

        m         = agc_conic_mshift_(ip, agc_conic_mload_(ls->cogamax), -1);
        ctx->coga = agc_conic_mstore_(agc_conic_mdad_(ip, m, half_min));
        ls->dcoga = ctx->coga; /* STCALL DCOGA   */
    }
    else
    {
        ls->tweekit = AGC_CONIC_TWEEKIT_GUESS;
        ctx->coga   = in->coga;
    }
}

/* LAMBLOOP from the top to the GETX call (p. 1297-1298):
 * P = (1-CSTH)/(SNTH COGA - (CSTH-RHO)) (2^4), R1A = 2 - P (1 + COGA^2)
 * (2^6). Returns false for NEGP (P <= 0).                                */
static bool lambert_p_r1a(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                          const lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return false;
    }

    if (!AGC_ASSERT(ls != NULL))
    {
        return false;
    }

    int      n = 0;
    agc_tp_t m = agc_conic_mdmp_(agc_conic_mload_(ctx->coga), ctx->snth);

    m = agc_conic_mdsu_(ip, agc_conic_mshift_(ip, m, -1), ls->csth_rho);
    const agc_dp_t den = agc_conic_mstore_(agc_conic_mnorm_(m, &n)); /* 0D */

    m = agc_conic_mshift_(ip, agc_conic_mload_(ls->one_m_csth),
                          n - 9);                  /* SL* 0 -9D,1 */
    m = agc_conic_mddv_(ip, m, den);

    if (agc_conic_mbmn_(m) || agc_conic_mbze_(m))
    { /* BMN BZE NEGP   */
        return false;
    }

    ctx->p = agc_conic_mstore_(m); /* STODL P / COGA */
    m      = agc_conic_mdad_(ip, agc_conic_mdsq_(agc_conic_mload_(ctx->coga)),
                             AGC_CONIC_D1_1024);
    m        = agc_conic_mdmp_(agc_conic_mnorm_(m, &n), ctx->p);
    m        = agc_conic_mshift_(ip, m, 8 - n); /* SR* 0 -8D,1    */
    ctx->r1a = agc_conic_mstore_(agc_conic_mbdsu_(ip, m, AGC_CONIC_D1_32));

    return true;
}

/* NEGP / HIENERGY / BIGTIME / LOENERGY / COMMONLM (p. 1298-1299): make
 * COGA a bound (low energy: COGAMAX, high energy: COGAMIN), halve DCOGA
 * and step back by it.                                                   */
static lambert_next_t lambert_retreat(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                      lambert_state_t *ls, bool high_energy)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return LAMB_SUFFCHEK;
    }

    if (!AGC_ASSERT(ls != NULL))
    {
        return LAMB_SUFFCHEK;
    }

    if (high_energy)
    {
        ls->cogamin = ctx->coga; /* HIENERGY       */
    }
    else
    {
        ls->cogamax = ctx->coga; /* LOENERGY       */
    }

    ls->dcoga = agc_conic_mstore_(
        agc_conic_mshift_(ip, agc_conic_mload_(ls->dcoga), -1));

    if (ls->dcoga == 0)
    { /* BZE SUFFCHEK   */
        return LAMB_SUFFCHEK;
    }

    ctx->coga = agc_conic_mstore_(
        agc_conic_mbdsu_(ip, agc_conic_mload_(ls->dcoga), ctx->coga));

    return LAMB_LOOP;
}

/* ITERATOR call and COGA update at the end of LAMBLOOP (p. 1298). */
static lambert_next_t lambert_step(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                   lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return LAMB_SUFFCHEK;
    }

    if (!AGC_ASSERT(ls != NULL))
    {
        return LAMB_SUFFCHEK;
    }

    agc_iterator_t it;

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
    agc_status_t st = agc_conic_iterator(ip, &it);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return LAMB_SUFFCHEK;
    }

    ls->dcoga   = it.delindep;
    ls->cogamax = it.max;
    ls->cogamin = it.min;
    ls->slopesw = it.slopesw;

    if (ls->dcoga == 0)
    { /* DLOAD MPAC BZE */
        return LAMB_SUFFCHEK;
    }

    ctx->coga = agc_conic_mstore_(
        agc_conic_mdad_(ip, agc_conic_mload_(ls->dcoga), ctx->coga));

    return LAMB_LOOP;
}

/* One pass of LAMBLOOP (pp. 1297-1298). */
static lambert_next_t lambert_pass(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                   lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return LAMB_SUFFCHEK;
    }

    if (!AGC_ASSERT(ls != NULL))
    {
        return LAMB_SUFFCHEK;
    }

    if (!lambert_p_r1a(ip, ctx, ls))
    { /* NEGP           */
        return lambert_retreat(ip, ctx, ls, ls->dcoga < 0);
    }

    if (agc_interp_test_ovf(ip))
    { /* BOV HIENERGY   */
        return lambert_retreat(ip, ctx, ls, true);
    }

    agc_status_t st = agc_conic_getx_(ip, ctx);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return LAMB_SUFFCHEK;
    }

    ls->tprev = ctx->t; /* DLOAD T STODL TPREV */

    if (ctx->infinflg)
    { /* BON INFINFLG NEGP */
        return lambert_retreat(ip, ctx, ls, ls->dcoga < 0);
    }

    st = agc_conic_deltime_(ip, ctx);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return LAMB_SUFFCHEK;
    }

    if (agc_interp_test_ovf(ip))
    { /* BOV BIGTIME    */
        ctx->t = ls->tprev;

        return lambert_retreat(ip, ctx, ls, false);
    }

    agc_tp_t m = agc_conic_mbdsu_(ip, agc_conic_mload_(ctx->t), ls->tdesired);

    ls->terrlamb = agc_conic_mstore_(m); /* TERRLAMB       */
    m            = agc_conic_mbdsu_(ip, agc_conic_mabs_(m), ls->epsilonl);

    if (!agc_conic_mbmn_(m))
    { /* BPL INITV      */
        return LAMB_INITV;
    }

    ls->iterctr = (agc_sp_t) (ls->iterctr - 1); /* RTB CHECKCTR   */

    if (ls->iterctr == 0)
    { /* BHIZ SUFFCHEK  */
        return LAMB_SUFFCHEK;
    }

    return lambert_step(ip, ctx, ls);
}

/* SUFFCHEK (p. 1299): accept the last COGA if |TERRLAMB| <= TDESIRED/4
 * (CSM) or TDESIRED 2^-17 (LM), plus one bit.                            */
static bool lambert_suffchek(agc_interp_t *ip, const lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ls != NULL))
    {
        return false;
    }

    const bool     csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    const agc_dp_t err =
        agc_conic_mstore_(agc_conic_mabs_(agc_conic_mload_(ls->terrlamb)));
    agc_tp_t m = agc_conic_mdmp_(agc_conic_mload_(ls->tdesired),
                                 csm ? AGC_CONIC_D1_4 : AGC_CONIC_BEE17);

    m = agc_conic_mdsu_(ip, agc_conic_mdad_(ip, m, AGC_CONIC_ONEBIT), err);

    if (!AGC_ASSERT(ls->tdesired >= 0))
    {
        return false;
    }

    return !agc_conic_mbmn_(m); /* BPL INITV      */
}

/* INITV (p. 1300): VVEC = VTAN (UN x UR1) + VTAN COGA UR1 with
 * VTAN = sqrt(MU P / R1), 2^7 / 2^5.                                     */
static agc_vec_t lambert_initv(agc_interp_t *ip, const agc_conic_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return agc_vec_zero(); /* no valid result */
    }

    if (!AGC_ASSERT(ctx->r1 > 0))
    {
        return agc_vec_zero(); /* no valid result */
    }

    int            n   = 0;
    const agc_dp_t r1n =
        agc_conic_mstore_(agc_conic_mnorm_(agc_conic_mload_(ctx->r1), &n));
    agc_tp_t m = agc_conic_mshift_(ip, agc_conic_mload_(ctx->p), -1);

    m = agc_conic_mshift_(ip, agc_conic_mddv_(ip, m, r1n),
                          n - 4);          /* SL* 0 -4,1 */
    m = agc_conic_mdmp_(agc_conic_msqrt_(ip, m), ctx->root_mu);
    m = agc_conic_mshift_(ip, m, 1);
    const agc_dp_t vtan = agc_conic_mstore_(m); /* 0D VTAN 2^7    */

    m = agc_conic_mshift_(ip, agc_conic_mdmp_(m, ctx->coga), 5);
    const agc_vec_t vrad = agc_vec_scale(ip, ctx->ur1, agc_conic_mstore_(m));
    agc_vec_t       v    = agc_vec_shl(ip, agc_vec_scale(ip, ctx->un, vtan), 1);

    v = agc_vec_add(ip, agc_vec_cross(ip, v, ctx->ur1), vrad);

    return agc_vec_shl(ip, v, 1); /* VSL1           */
}

/* LAMBERT entry (p. 1296): ITERCTR, SOLNSW, MUTABLE, EPSILONL, SLOPESW. */
static agc_status_t lambert_entry(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                  const agc_lambert_in_t *in,
                                  lambert_state_t *ls)
{
    if (!AGC_ASSERT(ip != NULL && ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(in != NULL && ls != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(in->geomsgn != 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = agc_conic_ctx_init_(ctx, in->body); /* VLOAD* MUTABLE,1 */

    if (st != AGC_STATUS_OK)
    {
        return st;
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
            return AGC_STATUS_BAD_ARG;
        }

        ls->iterctr = in->iterctr; /* set by SETITCTR */
        agc_conic_mirror_flag_(AGC_CONIC_FLAG_SOLNSW, false); /* mirror only */
    }

    ls->tdesired = in->tdesired;
    ls->epsilonl = agc_conic_mstore_(
        agc_conic_mdmpr_(ip, agc_conic_mload_(in->tdesired), AGC_CONIC_BEE19));
    ls->slopesw = true; /* SET SLOPESW    */
    /* DEVIATION: TERRLAMB (an erasable) and DCOGA (push-down 12D) are not
     * set by LAMBERT before an early NEGP/SUFFCHEK can read them; the AGC
     * uses whatever they last held. Here DCOGA = 0 and TERRLAMB = TDESIRED,
     * so such an exit reports no solution.                               */
    ls->terrlamb = in->tdesired;
    ctx->normsw  = in->normsw;
    ctx->un      = in->un;
    ctx->geomsgn = in->geomsgn;

    return AGC_STATUS_OK;
}

/* Copy the LAMBERT debris to *out. */
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
}

/* Set SOLNSW (in *out and in the flagword). */
static void lambert_solnsw(agc_lambert_out_t *out, bool set)
{
    if (!AGC_ASSERT(out != NULL))
    {
        return;
    }

    out->solnsw = set;
    agc_conic_mirror_flag_(AGC_CONIC_FLAG_SOLNSW, set);
}

/*
 * LAMBERT (CONIC_SUBROUTINES.agc pp. 1296-1300, label LAMBERT).
 * See agc_nav.h for the status contract.
 */
agc_status_t agc_conic_lambert(agc_interp_t *ip, const agc_lambert_in_t *in,
                               agc_lambert_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_conic_ctx_t ctx;
    lambert_state_t ls;
    agc_status_t    st = lambert_entry(ip, &ctx, in, &ls);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    out->vvec    = agc_vec_zero();
    out->vtarget = agc_vec_zero();

    if (!lambert_geometry(ip, &ctx, in, &ls))
    { /* 360LAMB        */
        lambert_output(&ctx, &ls, out);
        lambert_solnsw(out, true);

        return AGC_STATUS_NO_CONVERGE;
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
    const bool ok  = (next == LAMB_INITV) || lambert_suffchek(ip, &ls);
    const bool csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);

    lambert_solnsw(out, !ok); /* SETGO SOLNSW / CLEAR */

    if (!ok && csm)
    { /* CSM: RTNLAMB   */
        return AGC_STATUS_NO_CONVERGE;
    }

    out->vvec = lambert_initv(ip, &ctx); /* INITV: VVEC    */
    ctx.vvec  = out->vvec;

    if (in->vtargtag == 0)
    { /* TARGETV        */
        st = agc_conic_lamenter_(ip, &ctx, ctx.magvec2, &out->vtarget);

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return AGC_STATUS_BAD_ARG;
        }
    }

    return ok ? AGC_STATUS_OK : AGC_STATUS_NO_CONVERGE;
}
