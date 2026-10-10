/*
 * kepler.c -- KEPLER (entry KEPLERN), its first guess KEPPREP, and
 * the conic path of CSMCONIC / LEMCONIC (RVCON).
 *
 * KEPLERN: CONIC_SUBROUTINES.agc Comanche 055 pp. 1277-1282 (Luminary 099
 * pp. 1173-1178, identical code). KEPPREP: ORBITAL_INTEGRATION.agc
 * pp. 1334-1335 (both programs, identical). RVCON / CSMCONIC / LEMCONIC:
 * INTEGRATION_INITIALIZATION.agc pp. 1318-1322.
 *
 * KEPLER solves the universal Kepler equation sqrt(mu) TAU = f(x) for x by
 * a secant iteration bounded by XMIN/XMAX (the bounds move in after every
 * step; a step that would cross a bound is cut to 9/10 of the way to it),
 * converging when |TAU - T| <= EPSILONT = |TAU| 2^-22, and at most ITERCTR
 * = 20 passes. Elliptic TAU beyond one period is first reduced modulo the
 * period.
 */
#include "internal.h"
#include "agc/agc_assert.h"
#include "agc/agc_state.h"
#include "agc/agc_flags_common.h"

/* KEPLER erasables that are not shared with the other conic routines. */
typedef struct kepler_state {
    agc_dp_t tau;        /* TAU.     reduced transfer time, 2^28 cs        */
    agc_dp_t alpha;      /* ALPHA (8D) 1/a, 2^-22 / 2^-20 1/m             */
    agc_dp_t xmax;       /* XMAX (10D) 2^17 / 2^16 sqrt(m)                 */
    agc_dp_t xmin;       /* XMIN (12D) 2^17 / 2^16                         */
    agc_dp_t delx;       /* DELX     2^17 / 2^16                           */
    agc_dp_t delt;       /* DELT     TAU - T, 2^28 cs                      */
    agc_dp_t epsilont;   /* EPSILONT 2^28 cs                               */
    agc_dp_t tc;         /* TC       2^28 cs                               */
} kepler_state_t;

/* Outcome of one KEPLOOP pass. */
typedef enum kepler_step {
    KEPLER_CONTINUE = 0, /* BRNCHCTR: count and iterate                   */
    KEPLER_CONVERGED,    /* KEPCONVG                                       */
    KEPLER_KEPRTN        /* CMNTOVFL with DELX = 0: return, no new state   */
} kepler_step_t;

/* KEPLERN to STOREMAX (p. 1277-1278): URRECT, R1, KEPC1 = R.V/ROOTMU
 * (2^17), KEPC2 = R V^2/MU - 1 (2^6), ALPHA = (1 - KEPC2)/R1, XMAX =
 * 2 pi / sqrt(ALPHA) (ellipse) or sqrt(-50 / ALPHA) (hyperbola), 2^17.   */
static void kepler_setup(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                         const agc_kepler_in_t *in, kepler_state_t *ks)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(in != NULL && ks != NULL);
    agc_dp_t r1 = 0;
    ctx->ur1 = agc_vec_unit(ip, in->rrect, &r1, NULL); /* UNIT, URRECT   */
    ctx->r1 = r1;                                      /* STOVL R1       */
    ctx->vvec = in->vrect;
    agc_tp_t m = agc_vec_dot_tp(ip, in->rrect, in->vrect);  /* DOT: TP MPAC */
    m = agc_conic_mshiftr_(ip, m, 1);                  /* DOT SL1R       */
    m = agc_conic_mshiftr_(ip, agc_conic_mdmp_(m, ctx->inv_root_mu), 1);
    ctx->kepc1 = agc_conic_mstore_(m);                 /* KEPC1 2^17     */
    m = agc_conic_mload_(agc_vec_sq(ip, in->vrect));   /* VSQ DMPR 1/MU  */
    m = agc_conic_mdmpr_(ip, m, ctx->inv_mu);
    m = agc_conic_mshift_(ip, agc_conic_mdmp_(m, r1), 3); /* DMP R1 SL3  */
    m = agc_conic_mround_(ip, agc_conic_mdsu_(ip, m, AGC_CONIC_D1_64));
    ctx->kepc2 = agc_conic_mstore_(m);                 /* KEPC2 2^6      */
    m = agc_conic_mshiftr_(ip, agc_conic_mbdsu_(ip, m, AGC_CONIC_D1_64), -1);
    m = agc_conic_mddv_(ip, m, r1);                    /* DDV R1         */
    ks->alpha = agc_conic_mstore_(m);                  /* ALPHA          */
    if (!agc_conic_mbmn_(m)) {                         /* BPL 1REV       */
        m = agc_conic_msqrt_(ip, m);
        m = agc_conic_mbddv_(ip, m, AGC_CONIC_2PISC);  /* BDDV 2PISC     */
        (void)agc_interp_test_ovf(ip);                 /* BOV STOREMAX   */
    } else {
        m = agc_conic_mddv_(ip, agc_conic_mload_(AGC_CONIC_M50SC), ks->alpha);
        if (!agc_interp_test_ovf(ip)) {                /* BOV STOREMAX   */
            m = agc_conic_msqrt_(ip, m);
        }
    }
    ks->xmax = agc_conic_mstore_(m);                   /* STOREMAX       */
}

/* STOREMAX to MODDONE (p. 1278): PERIOD = XMAX / (ROOTMU ALPHA) (2^28)
 * and PERIODCH, which removes whole periods from TAU. keeping its sign.
 * No reduction when the period overflows or is negative (hyperbola).     */
static void kepler_period(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                          kepler_state_t *ks)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(ks != NULL);
    const agc_dp_t p0 = agc_conic_mstore_(
        agc_conic_mdmp_(agc_conic_mload_(ks->xmax), ctx->inv_root_mu));
    int n = 0;                                         /* ALPHA NORM X1  */
    const agc_dp_t alpha_n = agc_conic_mstore_(
        agc_conic_mnorm_(agc_conic_mload_(ks->alpha), &n));
    agc_tp_t m = agc_conic_mshift_(ip, agc_conic_mload_(p0), n - 6); /* SL* 0 -6,1 */
    m = agc_conic_mddv_(ip, m, alpha_n);
    if (agc_interp_test_ovf(ip) || agc_conic_mbmn_(m)) { /* BOV BMN MODDONE */
        return;
    }
    const agc_dp_t period = agc_conic_mstore_(m);      /* 0D = PERIOD    */
    int pass = 0;
    for (pass = 0; pass < AGC_CONIC_PERIOD_MAX_PASSES; pass++) { /* PERIODCH */
        m = agc_conic_mabs_(agc_conic_mload_(ks->tau));
        m = agc_conic_mdsu_(ip, m, period);
        if (agc_conic_mbmn_(m)) {                      /* BMN MODDONE    */
            break;
        }
        ks->tau = agc_conic_mstore_(agc_conic_msign_(m, ks->tau));
    }
    (void)AGC_ASSERT(pass < AGC_CONIC_PERIOD_MAX_PASSES);
}

/* MODDONE, BADX, STORBNDS, STOREMIN, DXCOMP, XDIFF (pp. 1278-1279):
 * X from XKEPNEW (or XMAX/2 with the sign of TAU. if the guess is zero,
 * of the wrong sign or beyond XMAX), the bounds, EPSILONT and DELX.      */
static void kepler_bounds(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                          const agc_kepler_in_t *in, kepler_state_t *ks)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(in != NULL && ks != NULL);
    ctx->x = in->xkepnew;                              /* STORE X        */
    agc_tp_t m = agc_conic_msign_(agc_conic_mload_(in->xkepnew), ks->tau);
    bool badx = agc_conic_mbze_(m) || agc_conic_mbmn_(m);
    if (!badx) {                                       /* ABS DSU XMAX   */
        m = agc_conic_mdsu_(ip, agc_conic_mabs_(m), ks->xmax);
        badx = !agc_conic_mbmn_(m);                    /* BPL BADX       */
    }
    if (badx) {                                        /* BADX           */
        m = agc_conic_mshift_(ip, agc_conic_mload_(ks->xmax), -1);
        ctx->x = agc_conic_mstore_(agc_conic_msign_(m, ks->tau));
    }
    if (ks->tau >= 0) {                                /* STOREMIN       */
        ks->xmin = AGC_CONIC_KEPZERO;
    } else {
        ks->xmin = agc_dp_neg(ks->xmax);
        ks->xmax = AGC_CONIC_KEPZERO;
    }
    m = agc_conic_mdmpr_(ip, agc_conic_mload_(ks->tau), AGC_CONIC_BEE22);
    ks->epsilont = agc_conic_mstore_(agc_conic_mabs_(m)); /* DXCOMP     */
    ks->delx = agc_conic_mstore_(                       /* XDIFF        */
        agc_conic_mbdsu_(ip, agc_conic_mload_(in->xprev), ctx->x));
}

/* Secant step of KEPLOOP (p. 1279): trial DELX = DELX DELT / (T - TC),
 * then cut to 9/10 of the distance to the bound it would cross
 * (NDXCHNGE / PDXCHNGE) after moving the opposite bound in to X.         */
static agc_dp_t kepler_newdelx(agc_interp_t *ip, const agc_conic_ctx_t *ctx,
                               kepler_state_t *ks)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(ks != NULL);
    int n1 = 0;
    int n2 = 0;
    agc_tp_t m = agc_conic_mnorm_(agc_conic_mdsu_(ip, agc_conic_mload_(ctx->t),
                                                  ks->tc), &n1);
    const agc_dp_t dt_n = agc_conic_mstore_(m);        /* PDDL: 0D       */
    m = agc_conic_mnorm_(agc_conic_mload_(ks->delx), &n2); /* NORM X2    */
    m = agc_conic_mdmp_(m, ks->delt);                  /* XSU,1 X2; DMP  */
    m = agc_conic_mshiftr_(ip, m, 1 + n1 - n2);        /* SLR* 1,1       */
    m = agc_conic_mshift_(ip, agc_conic_mddv_(ip, m, dt_n), -1); /* DDV SR1 */
    const agc_dp_t trial = agc_conic_mstore_(m);       /* PUSH 0D        */
    if (!agc_conic_mbmn_(m)) {                         /* POSDELX        */
        ks->xmin = ctx->x;
        m = agc_conic_mbdsu_(ip, agc_conic_mload_(ctx->x), ks->xmax);
        m = agc_conic_mdsu_(ip, m, trial);
        if (agc_interp_test_ovf(ip) || agc_conic_mbmn_(m)) {   /* PDXCHNGE */
            m = agc_conic_mdsu_(ip, agc_conic_mload_(ks->xmax), ctx->x);
            return agc_conic_mstore_(agc_conic_mdmpr_(ip, m, AGC_CONIC_DP9_10));
        }
        return trial;
    }
    ks->xmax = ctx->x;                                 /* move max in    */
    m = agc_conic_mbdsu_(ip, agc_conic_mload_(ctx->x), ks->xmin);
    m = agc_conic_mdsu_(ip, m, trial);
    if (agc_interp_test_ovf(ip) || !agc_conic_mbmn_(m)) {      /* NDXCHNGE */
        m = agc_conic_mdsu_(ip, agc_conic_mload_(ks->xmin), ctx->x);
        return agc_conic_mstore_(agc_conic_mdmpr_(ip, m, AGC_CONIC_DP9_10));
    }
    return trial;
}

/* TIMEOVFL / NEGTOVFL / CMNTOVFL (p. 1281): T overflowed, X was too big:
 * make X a bound, halve DELX and back X off by it.                       */
static kepler_step_t kepler_timeovfl(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                     kepler_state_t *ks)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(ks != NULL);
    if (ctx->x < 0) {                                  /* NEGTOVFL       */
        ks->xmin = ctx->x;
    } else {
        ks->xmax = ctx->x;
    }
    ks->delx = agc_conic_mstore_(agc_conic_mshift_(ip, agc_conic_mload_(ks->delx), -1));
    if (ks->delx == 0) {                               /* BZE KEPRTN     */
        return KEPLER_KEPRTN;
    }
    ctx->x = agc_conic_mstore_(agc_conic_mbdsu_(ip, agc_conic_mload_(ks->delx), ctx->x));
    ctx->t = ks->tc;                                   /* DLOAD TC STORE T */
    return KEPLER_CONTINUE;
}

/* One pass of KEPLOOP (pp. 1279-1280) up to BRNCHCTR. */
static kepler_step_t kepler_pass(agc_interp_t *ip, agc_conic_ctx_t *ctx,
                                 kepler_state_t *ks)
{
    (void)AGC_ASSERT(ip != NULL && ctx != NULL);
    (void)AGC_ASSERT(ks != NULL);
    int n1 = 0;                                        /* DSQ NORM X1    */
    agc_tp_t m = agc_conic_mnorm_(agc_conic_mdsq_(agc_conic_mload_(ctx->x)), &n1);
    ctx->xsq = agc_conic_mstore_(m);                   /* PUSH 0D        */
    ctx->xsq_n1 = n1;
    m = agc_conic_mshiftr_(ip, agc_conic_mdmp_(m, ks->alpha), 6 - n1); /* SRR* 0 -6,1 */
    ctx->xi = agc_conic_mstore_(m);                    /* XI 2^6         */
    agc_status_t st = agc_conic_deltime_(ip, ctx);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    if (agc_interp_test_ovf(ip)) {                     /* BOV TIMEOVFL   */
        return kepler_timeovfl(ip, ctx, ks);
    }
    m = agc_conic_mbdsu_(ip, agc_conic_mload_(ctx->t), ks->tau); /* BDSU TAU. */
    ks->delt = agc_conic_mstore_(m);
    m = agc_conic_mbdsu_(ip, agc_conic_mabs_(m), ks->epsilont);
    if (!agc_conic_mbmn_(m)) {                         /* BPL KEPCONVG   */
        return KEPLER_CONVERGED;
    }
    ks->delx = kepler_newdelx(ip, ctx, ks);            /* NEWDELX        */
    if (ks->delx == 0) {                               /* BZE KEPCONVG   */
        return KEPLER_CONVERGED;
    }
    ctx->x = agc_conic_mstore_(agc_conic_mdad_(ip, agc_conic_mload_(ks->delx), ctx->x));
    ks->tc = ctx->t;                                   /* STODL X / T; STORE TC */
    return KEPLER_CONTINUE;
}

/* Copy the KEPLER results and debris to *out. */
static void kepler_output(const agc_conic_ctx_t *ctx, const kepler_state_t *ks,
                          const agc_kepler_in_t *in, agc_kepler_out_t *out)
{
    (void)AGC_ASSERT(ctx != NULL && ks != NULL);
    (void)AGC_ASSERT(in != NULL && out != NULL);
    agc_ovf_t ovf = 0;
    out->urrect = ctx->ur1;
    out->r1 = ctx->r1;
    out->alpha = ks->alpha;
    out->tau = ks->tau;
    out->tmodulo = agc_dp_sub_raw(in->tau, ks->tau, &ovf);
    (void)ovf;   /* |reduced TAU.| <= |TAU.|: cannot overflow */
}

/*
 * KEPLER (CONIC_SUBROUTINES.agc pp. 1263-1265 description, code
 * pp. 1277-1282, label KEPLERN). See agc_nav.h for the status contract.
 */
agc_status_t agc_conic_kepler(agc_interp_t *ip, const agc_kepler_in_t *in,
                              agc_kepler_out_t *out)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(out != NULL)) { return AGC_STATUS_BAD_ARG; }
    agc_conic_ctx_t ctx;
    agc_status_t st = agc_conic_ctx_init_(&ctx, in->body); /* VLOAD* MUTABLE,1 */
    if (st != AGC_STATUS_OK) { return st; }
    (void)agc_interp_test_ovf(ip);                     /* SETPD 0, BOV +1 */
    kepler_state_t ks = { in->tau, 0, 0, 0, 0, 0, 0, in->tc };
    kepler_setup(ip, &ctx, in, &ks);
    kepler_period(ip, &ctx, &ks);
    kepler_bounds(ip, &ctx, in, &ks);
    kepler_step_t step = KEPLER_CONTINUE;
    int pass = 0;
    while (pass < AGC_CONIC_KEPLER_ITERCTR && step == KEPLER_CONTINUE) {
        step = kepler_pass(ip, &ctx, &ks);             /* KEPLOOP        */
        pass++;                                        /* CHECKCTR       */
    }
    kepler_output(&ctx, &ks, in, out);
    out->iterations = pass;
    if (step == KEPLER_KEPRTN) {                       /* RCV/VCV untouched */
        out->rcv = in->rrect;
        out->vcv = in->vrect;
        out->tc = ks.tc;
        out->xprev = in->xprev;
        return AGC_STATUS_OVERFLOW;
    }
    st = agc_conic_kepconvg_(ip, &ctx, &out->rcv, &out->vcv); /* KEPCONVG */
    out->tc = ctx.t;                                   /* STODL TC       */
    out->xprev = ctx.x;                                /* STCALL XPREV   */
    if (st != AGC_STATUS_OK) { return st; }
    return (step == KEPLER_CONVERGED) ? AGC_STATUS_OK : AGC_STATUS_NO_CONVERGE;
}

/* KEPPREP second-order term (ORBITAL_INTEGRATION.agc p. 1334, from
 * "DSQ PDDL* MUEARTH,2" to "SL* SL* 8D,1 0,1"):
 * S (-FS + 2 (FS)^2 - (2/3)(1/R - ALPHA) MU S^2 ...) sqrt(MU), 2^18.
 * n1 = NORM count of |R|, n2 = NORM count of TAU. - TC (S1).            */
static agc_tp_t kepprep_series(agc_interp_t *ip, const agc_kepler_in_t *in,
                               const agc_dp_t cell[6], int n1, int n2)
{
    (void)AGC_ASSERT(ip != NULL && in != NULL);
    (void)AGC_ASSERT(cell != NULL);
    /* cell[]: 0 SQRT MU, 1 |R| (36D), 2 R normalized, 3 S, 4 FS, 5 MU */
    const agc_dp_t fs_sq = agc_conic_mstore_(agc_conic_mdsq_(agc_conic_mload_(cell[4])));
    const agc_dp_t s_sq = agc_conic_mstore_(agc_conic_mdsq_(agc_conic_mload_(cell[3])));
    const agc_dp_t mu43 = agc_conic_mstore_(               /* SR3 SR4: 12D */
        agc_conic_mshift_(ip, agc_conic_mshift_(ip, agc_conic_mload_(cell[5]), -3), -4));
    agc_tp_t m = agc_conic_mload_(agc_vec_sq(ip, in->vrect)); /* VSQ VCV  */
    m = agc_conic_mbdsu_(ip, agc_conic_mdmp_(m, cell[1]), mu43); /* DMP 36D BDSU */
    m = agc_conic_mdmp_(agc_conic_mddv_(ip, m, cell[2]), s_sq); /* DDV 2D DMP */
    m = agc_conic_mdmp_(m, AGC_CONIC_DP2_3);           /* DMP DP2/3      */
    m = agc_conic_mshift_(ip, m, n1 - 3);              /* SL* 0 -3,1     */
    m = agc_conic_mdad_(ip, m, fs_sq);                 /* XSU,1 S1; DAD  */
    m = agc_conic_mshift_(ip, m, 8 + n1 - n2);         /* SL* 8D,1       */
    m = agc_conic_mdsu_(ip, m, cell[4]);               /* DSU (FS)       */
    m = agc_conic_mdmp_(agc_conic_mdmp_(m, cell[0]), cell[3]); /* DMP 0D, 4D */
    m = agc_conic_mshift_(ip, m, 8 + n1 - n2);         /* SL* 8D,1       */
    return agc_conic_mshift_(ip, m, n1 - n2);          /* SL* 0,1        */
}

/*
 * KEPPREP (ORBITAL_INTEGRATION.agc pp. 1334-1335, label KEPPREP)
 * First guess of x for KEPLER by a Taylor series in the time step:
 *   XKEPNEW = XKEP + sqrt(MU) S (1 - FS + 2 (FS)^2 + ...),  S = (TAU-TC)/R,
 * from RCV (in->rrect), VCV (in->vrect), TAU., TC and XKEP (in->xprev).
 * The AGC then calls KEPLERN itself with X1 from MOONFLAG; here the caller
 * calls agc_conic_kepler (see agc_conic_rvcon).
 */
agc_status_t agc_conic_kepprep(agc_interp_t *ip, const agc_kepler_in_t *in,
                               agc_dp_t *xkepnew)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(xkepnew != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(in->body == AGC_CONIC_EARTH || in->body == AGC_CONIC_MOON)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t cell[6];                                  /* see kepprep_series */
    cell[5] = agc_conic_mu_(in->body);                 /* DLOAD* MUEARTH,2 */
    cell[0] = agc_conic_mstore_(agc_conic_msqrt_(ip, agc_conic_mload_(cell[5])));
    agc_vec_t ur = agc_vec_unit(ip, in->rrect, &cell[1], NULL); /* UNIT RCV */
    int n1 = 0;
    int n2 = 0;
    cell[2] = agc_conic_mstore_(agc_conic_mnorm_(agc_conic_mload_(cell[1]), &n1));
    const agc_dp_t fsmu = agc_vec_dot(ip, ur, in->vrect); /* DOT VCV: 4D  */
    agc_tp_t m = agc_conic_mdsu_(ip, agc_conic_mload_(in->tau), in->tc);
    m = agc_conic_mshift_(ip, agc_conic_mnorm_(m, &n2), -1); /* NORM S1 SR1 */
    cell[3] = agc_conic_mstore_(agc_conic_mddv_(ip, m, cell[2])); /* S: DDV 2D */
    cell[4] = agc_conic_mstore_(agc_conic_mdmp_(agc_conic_mload_(fsmu), cell[3]));
    m = agc_conic_mdad_(ip, kepprep_series(ip, in, cell, n1, n2), in->xprev);
    const agc_dp_t sum = agc_conic_mstore_(m);         /* DAD XKEP, PDDL */
    m = agc_conic_mdmp_(agc_conic_mload_(cell[3]), cell[0]); /* DMP 0D   */
    m = agc_conic_mshift_(ip, m, 1 + n1 - n2);         /* SL* 1,1        */
    (void)agc_interp_test_ovf(ip);                     /* BOVB TCDANZIG  */
    *xkepnew = agc_conic_mstore_(agc_conic_mdad_(ip, m, sum)); /* XKEPNEW */
    return AGC_STATUS_OK;
}

/*
 * RVCON (INTEGRATION_INITIALIZATION.agc pp. 1321-1322, label RVCON) with
 * the RECTIFY and RECTOUT arithmetic: TAU. = TDEC - TET; RECTIFY (RRECT =
 * RCV, TC = XKEP = 0); KEPPREP; KEPLERN; TET = TC + TET.
 * DEVIATION: the state vector carries no TDELTAV/TNUV, so it is taken as
 * already rectified (RCV = RRECT); the RATT/VATT rescaling to earth scale
 * and the INTEXIT bookkeeping are left to nav-integ.
 */
agc_status_t agc_conic_rvcon(agc_interp_t *ip, const agc_state_vector_t *sv,
                             agc_dp_t tdec, agc_state_vector_t *out,
                             agc_kepler_out_t *kout)
{
    if (!AGC_ASSERT(ip != NULL && sv != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (!AGC_ASSERT(out != NULL)) { return AGC_STATUS_BAD_ARG; }
    agc_kepler_in_t kin;
    kin.rrect = sv->r;
    kin.vrect = sv->v;
    kin.tau = agc_dp_sub(ip, tdec, sv->t);             /* DLOAD TDEC DSU TET */
    kin.xkepnew = 0;
    kin.tc = 0;                                        /* RECTIFY        */
    kin.xprev = 0;                                     /* XKEP = XPREV   */
    kin.body = sv->moon ? AGC_CONIC_MOON : AGC_CONIC_EARTH;
    agc_status_t st = agc_conic_kepprep(ip, &kin, &kin.xkepnew);
    if (st != AGC_STATUS_OK) { return st; }
    agc_kepler_out_t ko;
    st = agc_conic_kepler(ip, &kin, &ko);
    if (st == AGC_STATUS_BAD_ARG) { return st; }
    out->r = ko.rcv;                                   /* RECTOUT: RRECT = RCV */
    out->v = ko.vcv;
    out->t = agc_dp_add(ip, ko.tc, sv->t);             /* DLOAD TC DAD TET */
    out->moon = sv->moon;
    if (kout != NULL) {
        *kout = ko;
    }
    return st;
}

/*
 * CSMCONIC (INTEGRATION_INITIALIZATION.agc p. 1318, labels CSMCONIC,
 * IFLAGC, INTEGRV1, ALOADED -> RVCON): conic extrapolation of the CSM
 * permanent state (RRECTCSM, VRECTCSM, TETCSM, CMOONFLG) to TDEC1.
 */
agc_status_t agc_conic_csmconic(agc_interp_t *ip, agc_dp_t tdec1,
                                agc_state_vector_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL)) { return AGC_STATUS_BAD_ARG; }
    agc_state_vector_t sv;                             /* PTOACSM        */
    sv.r = agc_erase_rrectcsm();
    sv.v = agc_erase_vrectcsm();
    sv.t = agc_erase_tetcsm();
    sv.moon = agc_flag_test(AGC_FLAG_CMOONFLG);
    (void)AGC_ASSERT(tdec1 >= AGC_DP_MIN && tdec1 <= AGC_DP_MAX);
    return agc_conic_rvcon(ip, &sv, tdec1, out, NULL);
}

/*
 * LEMCONIC (INTEGRATION_INITIALIZATION.agc pp. 1318-1319): as CSMCONIC for
 * the LM permanent state (RRECTLEM, VRECTLEM, TETLEM, LMOONFLG). With
 * SURFFLAG set PTOALEM goes to USEPIOS (landing-site state, nav-integ):
 * not available here, AGC_STATUS_UNIMPLEMENTED.
 */
agc_status_t agc_conic_lemconic(agc_interp_t *ip, agc_dp_t tdec1,
                                agc_state_vector_t *out)
{
    if (!AGC_ASSERT(ip != NULL && out != NULL)) { return AGC_STATUS_BAD_ARG; }
    if (agc_flag_test(AGC_FLAG_SURFFLAG)) {            /* BON SURFFLAG USEPIOS */
        return AGC_STATUS_UNIMPLEMENTED;
    }
    agc_state_vector_t sv;                             /* PTOALEM        */
    sv.r = agc_erase_rrectlem();
    sv.v = agc_erase_vrectlem();
    sv.t = agc_erase_tetlem();
    sv.moon = agc_flag_test(AGC_FLAG_LMOONFLG);
    (void)AGC_ASSERT(tdec1 >= AGC_DP_MIN && tdec1 <= AGC_DP_MAX);
    return agc_conic_rvcon(ip, &sv, tdec1, out, NULL);
}
