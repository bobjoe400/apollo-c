/**********************************************************************
 * FILE NAME: kepler.c
 *
 * PURPOSE:
 *   KEPLER (entry KEPLERN): the position and velocity after a transfer
 *   time TAU. on the conic through a state, and KEPPREP, the first guess
 *   of the universal variable x that the integration package gives it.
 *
 * AGC SOURCE:
 *   Comanche055 CONIC_SUBROUTINES.agc pp. 1263-1265 (description),
 *   pp. 1277-1282 (code: KEPLERN, STOREMAX, PERIODCH, MODDONE, BADX,
 *   STORBNDS, STOREMIN, DXCOMP, XDIFF, KEPLOOP, NDXCHNGE, POSDELX,
 *   NEWDELX, BRNCHCTR, PDXCHNGE, TIMEOVFL, CMNTOVFL, NEGTOVFL);
 *   ORBITAL_INTEGRATION.agc pp. 1334-1335 (KEPPREP)
 *   Luminary099 CONIC_SUBROUTINES.agc pp. 1173-1178 and
 *   ORBITAL_INTEGRATION.agc (identical code)
 *
 * EXTERNAL VARIABLES:
 *   none (the conic erasables are fields of agc_conic_ctx_t and of the
 *   file-local kepler_state_t)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_conic_deltime_    DELTIME (universal.c)
 *   agc_conic_kepconvg_   KEPCONVG (universal.c)
 *   agc_conic_m*_         one interpretive op on the TP MPAC (common.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_OVERFLOW when T overflows until DELX
 *   halves to zero (KEPRTN without a new state); AGC_STATUS_NO_CONVERGE
 *   when the 20 passes run out (the state is still computed, as in the
 *   AGC).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the caller's job, both vehicles. KEPLOOP is bounded by
 *   ITERCTR = 20; PERIODCH by AGC_CONIC_PERIOD_MAX_PASSES (DEVIATION:
 *   unbounded in the AGC).
 *
 * ALGORITHM:
 *   KEPLER solves sqrt(mu) TAU = f(x) for x by a secant iteration bounded
 *   by XMIN/XMAX (the bounds move in after every step; a step that would
 *   cross a bound is cut to 9/10 of the way to it), converging when
 *   |TAU - T| <= EPSILONT = |TAU| 2^-22. Elliptic TAU beyond one period
 *   is first reduced modulo the period.
 *********************************************************************/
#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "internal.h"

/* KEPLER erasables that are not shared with the other conic routines. */
typedef struct kepler_state
{
    agc_dp_words_t tau;      /* TAU.     reduced transfer time, 2^28 cs        */
    agc_dp_words_t alpha;    /* ALPHA (8D) 1/a, 2^-22 / 2^-20 1/m             */
    agc_dp_words_t xmax;     /* XMAX (10D) 2^17 / 2^16 sqrt(m)                 */
    agc_dp_words_t xmin;     /* XMIN (12D) 2^17 / 2^16                         */
    agc_dp_words_t delx;     /* DELX     2^17 / 2^16                           */
    agc_dp_words_t delt;     /* DELT     TAU - T, 2^28 cs                      */
    agc_dp_words_t epsilont; /* EPSILONT 2^28 cs                               */
    agc_dp_words_t tc;       /* TC       2^28 cs                               */
} kepler_state_t;

/* Outcome of one KEPLOOP pass. */
typedef enum kepler_step
{
    KEPLER_CONTINUE = 0, /* BRNCHCTR: count and iterate                   */
    KEPLER_CONVERGED,    /* KEPCONVG                                       */
    KEPLER_KEPRTN        /* CMNTOVFL with DELX = 0: return, no new state   */
} kepler_step_t;

/* KEPPREP push-down cells (ORBITAL_INTEGRATION.agc p. 1334). */
enum kepprep_cell
{
    KEPPREP_ROOTMU = 0, /* 0D  SQRT(MU), 2^18 / 2^15                     */
    KEPPREP_R,          /* 36D |R|, 2^29 / 2^27 m                        */
    KEPPREP_RN,         /* 2D  |R| normalized, 2^(29-n1) / 2^(27-n1)     */
    KEPPREP_S,          /* 4D  S = (TAU. - TC)/R, normalized             */
    KEPPREP_FS,         /* 6D  F S, 2^(6+n1-n2)                          */
    KEPPREP_MU,         /* MUEARTH / MUMOON, 2^36 / 2^30                 */
    KEPPREP_CELLS       /* number of cells                               */
};

/**********************************************************************
 * KEPLERN to STOREMAX (CONIC_SUBROUTINES.agc pp. 1277-1278) -- URRECT,
 * R1, KEPC1 = R.V/ROOTMU, KEPC2 = R V^2/MU - 1, ALPHA = (1 - KEPC2)/R1
 * and the x bound XMAX = 2 pi/sqrt(ALPHA) (ellipse) or sqrt(-50/ALPHA)
 * (hyperbola)
 *
 * FUNCTION NAME: kepler_setup
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *        I/O  out: UR1 (URRECT) 2^1, R1
 *                                           2^29 / 2^27 m, VVEC, KEPC1
 *                                           2^17 / 2^16, KEPC2 2^6
 *   in        const agc_kepler_in_t *  I    RRECT, VRECT
 *   ks        kepler_state_t *         O    ALPHA 2^-22 / 2^-20 1/m,
 *                                           XMAX 2^17 / 2^16 sqrt(m)
 *********************************************************************/
static void kepler_setup(agc_interp_t *ip, agc_conic_ctx_t *ctx, const agc_kepler_in_t *in, kepler_state_t *ks)
{
    agc_dp_t r1 = 0; /* R1 = |RRECT|, 2^29 / 2^27 m */

    ctx->ur1         = agc_vec_unit(ip, in->rrect, &r1, NULL); /* UNIT, URRECT   */
    ctx->r1          = agc_dp_to_words(r1);                    /* STOVL R1       */
    ctx->vvec        = in->vrect;
    agc_tp_words_t m = agc_mpac_dot(ip, in->rrect, in->vrect); /* DOT: TP MPAC, 2^36 / 2^32 */

    m          = agc_mpac_shift_round(ip, m, 1);                  /* DOT SL1R       */
    m          = agc_mpac_shift_round(ip, agc_mpac_dmp(m, ctx->inv_root_mu), 1);
    ctx->kepc1 = agc_mpac_store(m);                               /* KEPC1 2^17     */
    m          = agc_mpac_vsq(ip, in->vrect);                     /* VSQ DMPR 1/MU  */
    m          = agc_mpac_dmpr(ip, m, ctx->inv_mu);
    m          = agc_mpac_shift(ip, agc_mpac_dmp(m, ctx->r1), 3); /* DMP R1 SL3  */
    m          = agc_mpac_round(ip, agc_mpac_dsu(ip, m, AGC_CONIC_D1_64));
    ctx->kepc2 = agc_mpac_store(m);                               /* KEPC2 2^6      */
    m          = agc_mpac_shift_round(ip, agc_mpac_bdsu(ip, m, AGC_CONIC_D1_64), -1);
    m          = agc_mpac_ddv(ip, m, ctx->r1);                    /* DDV R1         */
    ks->alpha  = agc_mpac_store(m);                               /* ALPHA          */

    if (!agc_conic_mbmn_(m))
    { /* BPL 1REV       */
        m = agc_mpac_sqrt(m);
        m = agc_mpac_bddv(ip, m, AGC_CONIC_2PISC); /* BDDV 2PISC     */
        agc_interp_clear_ovf(ip);                  /* BOV STOREMAX   */
    }
    else
    {
        m = agc_mpac_ddv(ip, agc_mpac_dload(AGC_CONIC_M50SC), ks->alpha);

        if (!agc_interp_test_ovf(ip))
        { /* BOV STOREMAX   */
            m = agc_mpac_sqrt(m);
        }
    }

    ks->xmax = agc_mpac_store(m); /* STOREMAX       */
} /* kepler_setup */

/**********************************************************************
 * STOREMAX to MODDONE (CONIC_SUBROUTINES.agc p. 1278) -- PERIOD = XMAX /
 * (ROOTMU ALPHA) and PERIODCH, which removes whole periods from TAU.
 * keeping its sign; no reduction when the period overflows or is
 * negative (hyperbola)
 *
 * FUNCTION NAME: kepler_period
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       const agc_conic_ctx_t *  I    1/ROOTMU
 *   ks        kepler_state_t *         I/O  XMAX, ALPHA in; TAU. 2^28 cs
 *                                           reduced modulo the period
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_NO_CONVERGE when
 *          PERIODCH reaches AGC_CONIC_PERIOD_MAX_PASSES (no AGC bound)
 *********************************************************************/
static agc_status_t kepler_period(agc_interp_t *ip, const agc_conic_ctx_t *ctx, kepler_state_t *ks)
{
    agc_tp_words_t       m  = agc_mpac_dmp(agc_mpac_dload(ks->xmax), ctx->inv_root_mu); /* DMP 1/ROOTMU, 2^0 */
    const agc_dp_words_t p0 = agc_mpac_store(m);                                        /* 0D XMAX/ROOTMU, 2^0 / 2^2 */
    int                  n  = 0;                                                        /* ALPHA NORM X1 count       */

    m                            = agc_mpac_norm(agc_mpac_dload(ks->alpha), &n);
    const agc_dp_words_t alpha_n = agc_mpac_store(m); /* 2^(-22-n) / 2^(-20-n) */

    m = agc_mpac_shift(ip, agc_mpac_dload(p0), n - 6); /* SL* 0 -6,1 */

    m = agc_mpac_ddv(ip, m, alpha_n);

    if (!(agc_interp_test_ovf(ip) || agc_conic_mbmn_(m)))
    {
        const agc_dp_words_t period = agc_mpac_store(m); /* 0D = PERIOD, 2^28 cs */
        int                  pass   = 0;                 /* PERIODCH passes */

        for (pass = 0; pass < AGC_CONIC_PERIOD_MAX_PASSES; pass++)
        { /* PERIODCH */
            m = agc_mpac_abs(agc_mpac_dload(ks->tau));
            m = agc_mpac_dsu(ip, m, period);

            if (agc_conic_mbmn_(m))
            { /* BMN MODDONE    */
                break;
            }

            ks->tau = agc_mpac_store(agc_mpac_sign(m, ks->tau));
        }

        if (!AGC_ASSERT(pass < AGC_CONIC_PERIOD_MAX_PASSES))
        {
            return (AGC_STATUS_NO_CONVERGE); /* DEVIATION bound reached */
        }
    }

    return (AGC_STATUS_OK);
} /* kepler_period */

/**********************************************************************
 * MODDONE, BADX, STORBNDS, STOREMIN, DXCOMP, XDIFF (CONIC_SUBROUTINES.agc
 * pp. 1278-1279) -- first x (XKEPNEW, or XMAX/2 with the sign of TAU. if
 * the guess is zero, of the wrong sign or beyond XMAX), the bounds,
 * EPSILONT and DELX
 *
 * FUNCTION NAME: kepler_bounds
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *        O    X 2^17 / 2^16 sqrt(m)
 *   in        const agc_kepler_in_t *  I    XKEPNEW, XPREV
 *   ks        kepler_state_t *         I/O  XMIN, XMAX, EPSILONT 2^28 cs,
 *                                           DELX 2^17 / 2^16
 *********************************************************************/
static void kepler_bounds(agc_interp_t *ip, agc_conic_ctx_t *ctx, const agc_kepler_in_t *in, kepler_state_t *ks)
{
    ctx->x              = agc_dp_to_words(in->xkepnew);                   /* DLOAD XKEPNEW, STORE X */
    agc_tp_words_t m    = agc_mpac_sign(agc_mpac_dload(ctx->x), ks->tau); /* SIGN TAU.: X, 2^17 / 2^16 */
    bool           badx = agc_conic_mbze_(m) || agc_conic_mbmn_(m);       /* BZE BMN: BADX taken */

    if (!badx)
    { /* ABS DSU XMAX   */
        m    = agc_mpac_dsu(ip, agc_mpac_abs(m), ks->xmax);
        badx = !agc_conic_mbmn_(m); /* BPL BADX       */
    }

    if (badx)
    { /* BADX           */
        m      = agc_mpac_shift(ip, agc_mpac_dload(ks->xmax), -1);
        ctx->x = agc_mpac_store(agc_mpac_sign(m, ks->tau));
    }

    if (!agc_conic_mbmn_(agc_mpac_dload(ks->tau)))
    { /* STORBNDS: DLOAD TAU. BPL STOREMIN */
        ks->xmin = AGC_CONIC_KEPZERO;
    }
    else
    {
        ks->xmin = agc_mpac_store(agc_mpac_dcomp(agc_mpac_dload(ks->xmax))); /* DLOAD XMAX DCOMP */
        ks->xmax = AGC_CONIC_KEPZERO;
    }

    m            = agc_mpac_dmpr(ip, agc_mpac_dload(ks->tau), AGC_CONIC_BEE22);
    ks->epsilont = agc_mpac_store(agc_mpac_abs(m));                                                       /* DXCOMP     */
    ks->delx     = agc_mpac_store(agc_mpac_bdsu(ip, agc_mpac_dload(agc_dp_to_words(in->xprev)), ctx->x)); /* XDIFF */
} /* kepler_bounds */

/**********************************************************************
 * KEPLOOP secant step, NDXCHNGE, POSDELX, PDXCHNGE (CONIC_SUBROUTINES.agc
 * pp. 1279-1280) -- trial DELX = DELX DELT / (T - TC), cut to 9/10 of
 * the distance to the bound it would cross after moving the opposite
 * bound in to X
 *
 * FUNCTION NAME: kepler_newdelx
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   ctx       const agc_conic_ctx_t *  I    X, T
 *   ks        kepler_state_t *         I/O  TC, DELX, DELT in; XMIN or
 *                                           XMAX moved in to X
 *
 * RETURNS: agc_dp_words_t -- the new DELX, 2^17 / 2^16 sqrt(m)
 *********************************************************************/
static agc_dp_words_t kepler_newdelx(agc_interp_t *ip, const agc_conic_ctx_t *ctx, kepler_state_t *ks)
{
    int                  n1   = 0;                                                                    /* NORM X1 count of T - TC */
    int                  n2   = 0;                                                                    /* NORM X2 count of DELX */
    agc_tp_words_t       m    = agc_mpac_norm(agc_mpac_dsu(ip, agc_mpac_dload(ctx->t), ks->tc), &n1); /* T - TC, 2^(28-n1) */
    const agc_dp_words_t dt_n = agc_mpac_store(m);                                                    /* PDDL: 0D, 2^(28-n1) */

    m                          = agc_mpac_norm(agc_mpac_dload(ks->delx), &n2);      /* NORM X2    */
    m                          = agc_mpac_dmp(m, ks->delt);                         /* XSU,1 X2; DMP  */
    m                          = agc_mpac_shift_round(ip, m, 1 + n1 - n2);          /* SLR* 1,1       */
    m                          = agc_mpac_shift(ip, agc_mpac_ddv(ip, m, dt_n), -1); /* DDV SR1 */
    const agc_dp_words_t trial = agc_mpac_store(m);                                 /* PUSH 0D: trial DELX, 2^17 */

    if (!agc_conic_mbmn_(m))
    { /* POSDELX        */
        ks->xmin = ctx->x;
        m        = agc_mpac_bdsu(ip, agc_mpac_dload(ctx->x), ks->xmax);
        m        = agc_mpac_dsu(ip, m, trial);

        if (agc_interp_test_ovf(ip) || agc_conic_mbmn_(m))
        { /* PDXCHNGE */
            m = agc_mpac_dsu(ip, agc_mpac_dload(ks->xmax), ctx->x);

            return (agc_mpac_store(agc_mpac_dmpr(ip, m, AGC_CONIC_DP9_10)));
        }
    }
    else
    {
        ks->xmax = ctx->x; /* move max in    */
        m        = agc_mpac_bdsu(ip, agc_mpac_dload(ctx->x), ks->xmin);
        m        = agc_mpac_dsu(ip, m, trial);

        if (agc_interp_test_ovf(ip) || !agc_conic_mbmn_(m))
        { /* NDXCHNGE */
            m = agc_mpac_dsu(ip, agc_mpac_dload(ks->xmin), ctx->x);

            return (agc_mpac_store(agc_mpac_dmpr(ip, m, AGC_CONIC_DP9_10)));
        }
    }

    return (trial);
} /* kepler_newdelx */

/**********************************************************************
 * KEPLOOP to BRNCHCTR (CONIC_SUBROUTINES.agc pp. 1279-1280) -- one pass
 * of the Kepler iteration: XI, DELTIME, the convergence test and the
 * secant step
 *
 * FUNCTION NAME: kepler_pass
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  ---------------------------------
 *   ip        agc_interp_t *     I/O  interpreter state (OVFIND)
 *   ctx       agc_conic_ctx_t *  I/O  X, XSQ, XI, T and the DELTIME
 *                                     results
 *   ks        kepler_state_t *   I/O  bounds, DELX, DELT, TC
 *
 * RETURNS: kepler_step_t -- KEPLER_CONTINUE, KEPLER_CONVERGED (to
 *          KEPCONVG) or KEPLER_KEPRTN (no new state)
 *********************************************************************/
static kepler_step_t kepler_pass(agc_interp_t *ip, agc_conic_ctx_t *ctx, kepler_state_t *ks)
{
    int            n1 = 0;                                    /* DSQ NORM X1 */
    agc_tp_words_t m  = agc_mpac_dsq(agc_mpac_dload(ctx->x)); /* X^2, 2^34 */

    m = agc_mpac_norm(m, &n1); /* XSQ, 2^(34-n1) / 2^(32-n1) */

    ctx->xsq    = agc_mpac_store(m);                                            /* PUSH 0D        */
    ctx->xsq_n1 = n1;
    m           = agc_mpac_shift_round(ip, agc_mpac_dmp(m, ks->alpha), 6 - n1); /* SRR* 0 -6,1 */
    ctx->xi     = agc_mpac_store(m);                                            /* XI 2^6         */
    agc_conic_deltime_(ip, ctx);                                                /* CALL DELTIME */

    if (agc_interp_test_ovf(ip))
    { /* BOV TIMEOVFL   */
        /* TIMEOVFL, NEGTOVFL, CMNTOVFL (CONIC_SUBROUTINES.agc p. 1281) -- T
         * overflowed, so X was too big: make X a bound, halve DELX and back X
         * off by it */
        if (agc_conic_mbmn_(agc_mpac_dload(ctx->x)))
        { /* DLOAD X BMN NEGTOVFL */
            ks->xmin = ctx->x;
        }
        else
        {
            ks->xmax = ctx->x;
        }

        agc_tp_words_t m_timeovfl = agc_mpac_shift(ip, agc_mpac_dload(ks->delx), -1); /* CMNTOVFL: DLOAD DELX SR1 */

        ks->delx = agc_mpac_store(m_timeovfl);

        if (agc_conic_mbze_(m_timeovfl))
        { /* BZE KEPRTN     */
            return (KEPLER_KEPRTN);
        }

        ctx->x = agc_mpac_store(agc_mpac_bdsu(ip, m_timeovfl, ctx->x)); /* BDSU X, STODL X */
        ctx->t = ks->tc;                                                /* DLOAD TC STORE T */

        return (KEPLER_CONTINUE);
    }

    m        = agc_mpac_bdsu(ip, agc_mpac_dload(ctx->t), ks->tau); /* BDSU TAU. */
    ks->delt = agc_mpac_store(m);
    m        = agc_mpac_bdsu(ip, agc_mpac_abs(m), ks->epsilont);

    if (!agc_conic_mbmn_(m))
    { /* BPL KEPCONVG   */
        return (KEPLER_CONVERGED);
    }

    ks->delx = kepler_newdelx(ip, ctx, ks); /* NEWDELX: STORE DELX */
    m        = agc_mpac_dload(ks->delx);    /* MPAC = DELX (MPAC+2 zero) */

    if (agc_conic_mbze_(m))
    { /* BZE KEPCONVG   */
        return (KEPLER_CONVERGED);
    }

    ctx->x = agc_mpac_store(agc_mpac_dad(ip, m, ctx->x)); /* DAD X */
    ks->tc = ctx->t;                                      /* STODL X / T; STORE TC */

    return (KEPLER_CONTINUE);
} /* kepler_pass */

/**********************************************************************
 * KEPLER (CONIC_SUBROUTINES.agc pp. 1277-1282, label KEPLERN) -- state
 * after TAU. on the conic through RRECT, VRECT
 *
 * FUNCTION NAME: agc_conic_kepler
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   in        const agc_kepler_in_t *  I    RRECT, VRECT, TAU., XKEPNEW,
 *                                           TC, XPREV, body
 *   out       agc_kepler_out_t *       O    RCV, VCV, TC, XPREV and the
 *                                           debris (see conic.h)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_NO_CONVERGE after 20
 *          passes (state still computed) or when PERIODCH reaches its pass
 *          bound in kepler_period(); AGC_STATUS_OVERFLOW for KEPRTN from
 *          TIMEOVFL (RCV/VCV = RRECT/VRECT)
 *********************************************************************/
agc_status_t agc_conic_kepler(agc_interp_t *ip, const agc_kepler_in_t *in, agc_kepler_out_t *out)
{
    agc_conic_ctx_t ctx;                                      /* conic erasables */
    agc_status_t    st = agc_conic_ctx_init_(&ctx, in->body); /* VLOAD* MUTABLE,1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_interp_clear_ovf(ip); /* SETPD 0, BOV +1 */
    kepler_state_t ks = {0};  /* KEPLER erasables */

    ks.tau = agc_dp_to_words(in->tau);
    ks.tc  = agc_dp_to_words(in->tc);

    kepler_setup(ip, &ctx, in, &ks);
    st = kepler_period(ip, &ctx, &ks); /* STOREMAX, PERIODCH */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    kepler_bounds(ip, &ctx, in, &ks);
    kepler_step_t step = KEPLER_CONTINUE; /* outcome of the last pass */
    int           pass = 0;               /* KEPLOOP passes (ITERCTR) */

    while (pass < AGC_CONIC_KEPLER_ITERCTR && step == KEPLER_CONTINUE)
    {
        step = kepler_pass(ip, &ctx, &ks); /* KEPLOOP        */
        pass++;                            /* CHECKCTR       */
    }

    /* Copy the KEPLER results and debris to *out. */
    agc_ovf_t ovf = 0; /* overflow (cannot occur) */

    out->urrect     = ctx.ur1;
    out->r1         = agc_dp_from_words(ctx.r1);
    out->alpha      = agc_dp_from_words(ks.alpha);
    out->tau        = agc_dp_from_words(ks.tau);
    out->tmodulo    = agc_dp_sub_raw(in->tau, out->tau, &ovf);
    (void) ovf; /* |reduced TAU.| <= |TAU.|: cannot overflow */
    out->iterations = pass;

    if (step == KEPLER_KEPRTN)
    { /* RCV/VCV untouched */
        out->rcv   = in->rrect;
        out->vcv   = in->vrect;
        out->tc    = agc_dp_from_words(ks.tc);
        out->xprev = in->xprev;

        return (AGC_STATUS_OVERFLOW);
    }

    agc_conic_kepconvg_(ip, &ctx, &out->rcv, &out->vcv); /* KEPCONVG */
    out->tc    = agc_dp_from_words(ctx.t);               /* STODL TC       */
    out->xprev = agc_dp_from_words(ctx.x);               /* STCALL XPREV   */

    return ((step == KEPLER_CONVERGED) ? AGC_STATUS_OK : AGC_STATUS_NO_CONVERGE); /* KEPCONVG or ITERCTR out */
} /* agc_conic_kepler */

/**********************************************************************
 * KEPPREP from "DSQ PDDL" to "DAD XKEP" (ORBITAL_INTEGRATION.agc
 * pp. 1334-1335) -- the series term of the first guess,
 * S (-FS + 2 (FS)^2 - (2/3)(1/R - ALPHA) MU S^2 ...) sqrt(MU), plus XKEP
 *
 * FUNCTION NAME: kepprep_series
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND)
 *   in        const agc_kepler_in_t *  I    VRECT (= VCV), XPREV (= XKEP)
 *   cell      const agc_dp_words_t []  I    push-down cells, see enum
 *                                           kepprep_cell
 *   n1        int                      I    NORM X1 count of |R|
 *   n2        int                      I    NORM S1 count of TAU. - TC
 *
 * RETURNS: agc_dp_words_t -- 6D series + XKEP, 2^17 / 2^16 sqrt(m)
 *********************************************************************/
static agc_dp_words_t kepprep_series(agc_interp_t *ip, const agc_kepler_in_t *in, const agc_dp_words_t cell[KEPPREP_CELLS], int n1, int n2)
{
    agc_tp_words_t       m     = agc_mpac_dsq(agc_mpac_dload(cell[KEPPREP_FS])); /* DSQ (FS), 2^(12+2(n1-n2)) */
    const agc_dp_words_t fs_sq = agc_mpac_store(m);                              /* 8D (FS)^2, 2^(12+2(n1-n2)) */

    m                         = agc_mpac_dsq(agc_mpac_dload(cell[KEPPREP_S])); /* DSQ S */
    const agc_dp_words_t s_sq = agc_mpac_store(m);                             /* 10D S^2, 2^(2(n1-n2)-2) */

    m                         = agc_mpac_shift(ip, agc_mpac_dload(cell[KEPPREP_MU]), -3); /* SR3 */
    const agc_dp_words_t mu43 = agc_mpac_store(agc_mpac_shift(ip, m, -4));                /* 12D SR4: MU 2^43 / 2^37 */

    m = agc_mpac_vsq(ip, in->vrect);                     /* PDVL VSQ VCV */
    m = agc_mpac_dmp(m, cell[KEPPREP_R]);                /* DMP 36D      */
    m = agc_mpac_bdsu(ip, m, mu43);                      /* BDSU         */
    m = agc_mpac_ddv(ip, m, cell[KEPPREP_RN]);           /* DDV 2D       */
    m = agc_mpac_dmp(m, s_sq);                           /* DMP 10D      */
    m = agc_mpac_dmp(m, AGC_CONIC_DP2_3);                /* DMP DP2/3    */
    m = agc_mpac_shift(ip, m, n1 - 3);                   /* SL* 0 -3,1   */
    m = agc_mpac_dad(ip, m, fs_sq);                      /* XSU,1 S1 DAD */
    m = agc_mpac_shift(ip, m, 8 + n1 - n2);              /* SL* 8D,1     */
    m = agc_mpac_dsu(ip, m, cell[KEPPREP_FS]);           /* DSU 6D (FS)  */
    m = agc_mpac_dmp(m, cell[KEPPREP_ROOTMU]);           /* DMP 0D       */
    m = agc_mpac_dmp(m, cell[KEPPREP_S]);                /* DMP 4D       */
    m = agc_mpac_shift(ip, m, 8 + n1 - n2);              /* SL* 8D,1     */
    m = agc_mpac_shift(ip, m, n1 - n2);                  /* SL* 0,1      */
    m = agc_mpac_dad(ip, m, agc_dp_to_words(in->xprev)); /* DAD XKEP */

    return (agc_mpac_store(m));
} /* kepprep_series */

/**********************************************************************
 * KEPPREP (ORBITAL_INTEGRATION.agc pp. 1334-1335, label KEPPREP) -- first
 * guess of x for KEPLER by a series in the time step:
 * XKEPNEW = XKEP + sqrt(MU) S (1 - FS + 2 (FS)^2 + ...), S = (TAU-TC)/R.
 * The AGC then calls KEPLERN itself with X1 from MOONFLAG; here the caller
 * calls agc_conic_kepler (nav-integ agc_integ_kepprep does both).
 *
 * FUNCTION NAME: agc_conic_kepprep
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  ---------------------------
 *   ip        agc_interp_t *           I/O  interpreter state (OVFIND
 *                                           cleared, BOVB TCDANZIG)
 *   in        const agc_kepler_in_t *  I    RCV (rrect), VCV (vrect),
 *                                           TAU., TC, XKEP (xprev), body
 *   xkepnew   agc_dp_t *               O    XKEPNEW, 2^17 / 2^16 sqrt(m)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (always)
 *********************************************************************/
agc_status_t agc_conic_kepprep(agc_interp_t *ip, const agc_kepler_in_t *in, agc_dp_t *xkepnew)
{
    agc_dp_words_t cell[KEPPREP_CELLS]; /* push-down cells, 2^N per kepprep_cell */
    agc_dp_t       r = 0;               /* 36D |R| from UNIT, 2^29 / 2^27 m */

    cell[KEPPREP_MU]     = agc_conic_mu_(in->body);                                         /* DLOAD* MUEARTH,2 */
    cell[KEPPREP_ROOTMU] = agc_mpac_store(agc_mpac_sqrt(agc_mpac_dload(cell[KEPPREP_MU]))); /* SQRT */
    agc_vec_t ur         = agc_vec_unit(ip, in->rrect, &r, NULL);                           /* UNIT RCV, 2^1 */
    int       n1         = 0;                                                               /* NORM X1 count of |R| */
    int       n2         = 0;                                                               /* NORM S1 count of TAU.-TC */

    cell[KEPPREP_R]           = agc_dp_to_words(r);
    cell[KEPPREP_RN]          = agc_mpac_store(agc_mpac_norm(agc_mpac_dload(cell[KEPPREP_R]), &n1)); /* 2D */
    const agc_dp_words_t fsmu = agc_mpac_store(agc_mpac_dot(ip, ur, in->vrect));                     /* DOT VCV PDDL: 4D F sqrt(MU), 2^7 / 2^5 */
    agc_tp_words_t       m    = agc_mpac_dload(agc_dp_to_words(in->tau));                            /* TAU., 2^28 */

    m = agc_mpac_dsu(ip, m, agc_dp_to_words(in->tc)); /* DSU TC: TAU. - TC */

    m = agc_mpac_shift(ip, agc_mpac_norm(m, &n2), -1); /* NORM S1 SR1 */
    m = agc_mpac_ddv(ip, m, cell[KEPPREP_RN]);         /* DDV 2D      */

    cell[KEPPREP_S]          = agc_mpac_store(m);                    /* 4D S, 2^(n1-n2-1) */
    m                        = agc_mpac_dmp(agc_mpac_dload(fsmu), cell[KEPPREP_S]);
    cell[KEPPREP_FS]         = agc_mpac_store(m);                    /* 6D FS, 2^(6+n1-n2) */
    const agc_dp_words_t sum = kepprep_series(ip, in, cell, n1, n2); /* PDDL: series + XKEP, 2^17 */

    m        = agc_mpac_dmp(agc_mpac_dload(cell[KEPPREP_S]), cell[KEPPREP_ROOTMU]); /* DMP 0D   */
    m        = agc_mpac_shift(ip, m, 1 + n1 - n2);                                  /* SL* 1,1  */
    agc_interp_clear_ovf(ip);                                                       /* BOVB TCDANZIG */
    *xkepnew = agc_dp_from_words(agc_mpac_store(agc_mpac_dad(ip, m, sum)));         /* DAD STORE XKEPNEW */

    return (AGC_STATUS_OK);
} /* agc_conic_kepprep */
