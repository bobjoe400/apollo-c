/*
 * orbital_integration.c -- ORBITAL INTEGRATION
 * (Comanche055 ORBITAL_INTEGRATION.agc pp. 1334-1354; Luminary099
 * pp. 1227-1248; the CM QUALITY1/QUALITY2 live in RT8_OP_CODES.agc p. 1516
 * and the LM constant pool in CONTROLLED_CONSTANTS.agc, same values).
 *
 * Encke integration of the deviation from a conic.  One time step:
 *   TIMESTEP  origin (sphere of influence) check, rectification test;
 *   INTGRATE  YV = TDELTAV, ZV = TNUV;
 *   three Nystrom stages at H = 0, DT/2, DT: ACCOMP computes the
 *   disturbing acceleration FV, DIFEQ+0/+1/+2 accumulate, DIFEQCOM
 *   predicts the deviation, FBR3 advances the conic by DT/2 (KEPLER);
 *   ENDSTATE stores TDELTAV, TNUV.  The W matrix columns are then
 *   integrated over the same step with DOW.. as the derivative.
 *
 * Index registers: X2 = PBODY = 0 (earth) / 2 (moon) selects the scale;
 * "VSL* 0 -k,2" is a right shift by k + PBODY.  NORM leaves -n in an
 * index register; here n is kept positive and the AGC index sums are
 * written out with their signs.
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "agc/agc_flags_common.h"
#include "agc/agc_integ.h"
#include "integ_internal.h"

/* ---- constant pool (CM pp. 1353-1354) --------------------------------- */
/* 3/4   2DEC 3.0 B-2, oct 30000 00000: 3 at scale 2^2 (also the rectify
 *       limit .75 for |TDELTAV| and |TNUV| in their own scales).         */
static const agc_dp_t K_3_4 = 201326592;
/* THREE/8 2DEC .375, oct 14000 00000.                                    */
static const agc_dp_t K_THREE_8 = 100663296;
/* DQUARTER = DP1/4 2DEC .25, oct 10000 00000.                            */
static const agc_dp_t K_DQUARTER = 67108864;
/* HALFDP .5 (XUNIT), oct 20000 00000.                                    */
static const agc_dp_t K_HALFDP = 134217728;
/* 3/32 2DEC 3 B-5, oct 03000 00000.                                      */
static const agc_dp_t K_3_32 = 25165824;
/* 15/16 2DEC 15. B-4, oct 36000 00000.                                   */
static const agc_dp_t K_15_16 = 251658240;
/* 3/64 2DEC 3 B-6, oct 01400 00000.                                      */
static const agc_dp_t K_3_64 = 12582912;
/* 7/12 2DEC .5833333333, oct 22525 12525.                                */
static const agc_dp_t K_7_12 = 156587349;
/* 2/3 = DP2/3 2DEC .6666666667, oct 25252 25253.                         */
static const agc_dp_t K_2_3 = 178956971;
/* 9/16 2DEC 9 B-4, oct 22000 00000.                                      */
static const agc_dp_t K_9_16 = 150994944;
/* 5/128 2DEC 5 B-7, oct 01200 00000.                                     */
static const agc_dp_t K_5_128 = 10485760;
/* 5/8 2DEC 5 B-3, oct 24000 00000.                                       */
static const agc_dp_t K_5_8 = 167772160;
/* D1/32 2DEC 1.0 B-5, oct 01000 00000 (CONIC_SUBROUTINES pool).          */
static const agc_dp_t K_D1_32 = 8388608;
/* RECRATIO 2DEC .01, oct 00243 32703: rectify when |dR| >= .01 |R|.      */
static const agc_dp_t K_RECRATIO = 2684355;
/* RSPHERE 2DEC 64373.76 E3 B-29, oct 03654 21000: lunar sphere of
 * influence radius 64373.76 km, scale 2^29 m.                            */
static const agc_dp_t K_RSPHERE = 32186880;

/* MU table addressed MUEARTH - X2 (2DEC*): X2 = 0 earth, 2 moon, 4 sun.
 *   MUEARTH 3.986032 E10 B-36  m^3/cs^2  oct 22437 16067
 *   MUM     4.9027780 E8 B-30            oct 16471 01352
 *   (sun)   1.32715445 E16 B-54          oct 27446 14620               */
static const agc_dp_t MU_TAB[3] = { 155704375, 122569450, 197761424 };

/* Oblateness tables addressed TABLE - X2 (index PBODY/2: earth, moon).
 *   RDE / RDM: 80467.20 km B-29 (oct 04627 25200) / 16093.44 km B-27
 *       (oct 03654 21000): oblateness only inside this radius.
 *   J4REQ/J3: .4991607391 E7 B-26 (oct 02302 24736) / 0.
 *   2J3RE/J2: -.1355426363 E5 B-27 (oct 77776 53032) / -176236.02 B-25
 *       (oct 77651 76237).
 *   J2REQSQ: 1.75501139 E21 B-72 (oct 13710 35320) / .3067493316 E18 B-60
 *       (oct 10407 05344).
 * (3J22R2MU 9.20479048 E16 B-58 is in the pool but unused; QUALITY2 uses
 *  the pad-loaded E3J22R2M instead.)                                     */
static const agc_dp_t RD_TAB[2]      = { 40233600, 32186880 };
static const agc_dp_t J4REQ_J3[2]    = { 19966430, 0 };
static const agc_dp_t J3RE_J2_2[2]   = { -27109, -1409888 };
static const agc_dp_t J2REQSQ_TAB[2] = { 99760848, 71420644 };

/* ASCALE,1 (p. 1353 "IT IS VITAL THAT THE FOLLOWING CONSTANTS NOT BE
 * SHUFFLED"): DLOAD* ASCALE,1 loads the two SP words at ASCALE - X1 into
 * S1, S2.  X1 = 0/2 primary (earth/moon), 4/6 secondary body, 8/10 sun.  */
#define ASCALE_N 6
static const int ASCALE_S1[ASCALE_N] = { -7, -9, 0, -2, -9, -11 };
static const int ASCALE_S2[ASCALE_N] = { -6, -4, -12, -2, -6, -2 };

/* Lunar potential pad loads (SP): see agc_integ_set_lunar_coeffs().      */
static agc_sp_t e3j22r2m;
static agc_sp_t e32c31rm;

#define INTEG_MAX_SHIFT 62
#define TP_HALF ((agc_tp_t)0x20000000000LL)   /* 2^41: TP .5             */

agc_status_t agc_integ_set_lunar_coeffs(agc_sp_t j22, agc_sp_t c31)
{
    if (!AGC_ASSERT(j22 >= AGC_SP_MIN) ||
        !AGC_ASSERT(c31 >= AGC_SP_MIN)) {
        return AGC_STATUS_BAD_ARG;
    }
    e3j22r2m = j22;
    e32c31rm = c31;
    return AGC_STATUS_OK;
}

/* MUEARTH - X2: the mu of earth (0), moon (2) or sun (4).               */
agc_dp_t agc_integ_mu(int x2)
{
    if (!AGC_ASSERT((x2 == 0) || (x2 == 2) || (x2 == 4))) {
        return MU_TAB[0];
    }
    return MU_TAB[x2 / 2];
}

/* Indexed shift (VSL* or VSR* with a computed count): n > 0 left, n < 0 right. */
agc_vec_t agc_integ_vshift(agc_interp_t *ip, agc_vec_t v, int n)
{
    int k = n;
    (void)AGC_ASSERT(ip != NULL);
    if (!AGC_ASSERT((k <= INTEG_MAX_SHIFT) && (k >= -INTEG_MAX_SHIFT))) {
        k = (k > 0) ? INTEG_MAX_SHIFT : -INTEG_MAX_SHIFT;
    }
    if (k >= 0) {
        return agc_vec_shl(ip, v, k);
    }
    return agc_vec_shr(v, -k);
}

agc_dp_t agc_integ_dshift(agc_interp_t *ip, agc_dp_t a, int n)
{
    int k = n;
    (void)AGC_ASSERT(ip != NULL);
    if (!AGC_ASSERT((k <= INTEG_MAX_SHIFT) && (k >= -INTEG_MAX_SHIFT))) {
        k = (k > 0) ? INTEG_MAX_SHIFT : -INTEG_MAX_SHIFT;
    }
    if (k >= 0) {
        return agc_dp_shl(ip, a, k);
    }
    return agc_dp_shr(a, -k);
}

/* NORM on the triple-precision MPAC (TSLC p. 1157): shift left until
 * |v| >= 1/2; *n = shifts. Used after DMP, which leaves a TP product.    */
static agc_tp_t tp_norm(agc_interp_t *ip, agc_tp_t v, int *n)
{
    agc_tp_t m = v;
    int k = 0;
    (void)AGC_ASSERT(n != NULL);
    (void)AGC_ASSERT((v >= AGC_TP_MIN) && (v <= AGC_TP_MAX));
    if (m != 0) {
        for (k = 0; k < AGC_TP_BITS; k++) {
            if ((m >= TP_HALF) || (m <= -TP_HALF)) {
                break;
            }
            m = agc_tp_add(ip, m, m);
        }
    }
    *n = k;
    return m;
}

/* SETMOON / CLRMOON (p. 1315): MOONFLAG and PBODY together.             */
agc_status_t agc_integ_setmoon(agc_integ_ctx_t *ctx, bool moon)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_integ_flag(AGC_FLAG_MOONFLAG, moon);
    ctx->pbody = moon ? INTEG_PBODY_MOON : INTEG_PBODY_EARTH;
    return AGC_STATUS_OK;
}

/* RECTIFY (p. 1347): establish a new conic through the actual state.
 *   RRECT = RCV = TDELTAV SR(7+PBODY) + RCV
 *   VRECT = VCV = TNUV SR(4+PBODY) + VCV, then MINIRECT.                 */
agc_status_t agc_integ_rectify(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL) ||
        !AGC_ASSERT(agc_integ_valid_ctx(ctx))) {
        return AGC_STATUS_BAD_ARG;
    }
    const int p = ctx->pbody;
    ctx->sv.rrect = agc_vec_add(ip, agc_vec_shr(ctx->sv.deltav, 7 + p),
                                ctx->sv.rcv);
    ctx->sv.rcv = ctx->sv.rrect;
    const agc_vec_t v = agc_vec_add(ip, agc_vec_shr(ctx->sv.nuv, 4 + p),
                                    ctx->sv.vcv);
    return agc_integ_minirect(ctx, v);
}

/* MINIRECT (p. 1347): VRECT = VCV = v, zero TDELTAV TNUV TC XKEP.        */
agc_status_t agc_integ_minirect(agc_integ_ctx_t *ctx, agc_vec_t v)
{
    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(agc_integ_valid_ctx(ctx))) {
        return AGC_STATUS_BAD_ARG;
    }
    ctx->sv.vrect = v;
    ctx->sv.vcv = v;
    ctx->sv.deltav = agc_vec_zero();
    ctx->sv.nuv = agc_vec_zero();
    ctx->sv.tc = 0;
    ctx->sv.xkep = 0;
    return AGC_STATUS_OK;
}

/*
 * KEPPREP (p. 1334-1335) and the KEPLERN call it ends in: first guess of
 * x from the current conic point (RCV, VCV at TC) by series in TAU.-TC,
 * then KEPLER from RRECT, VRECT to TAU.  Results RCV VCV TC XKEP.
 * The arithmetic is agent nav-conic's (agc_conic_kepprep/kepler); this
 * wrapper owns the erasables.  KEPPREP reads RCV and VCV, so they are
 * passed in the rrect/vrect fields of the KEPPREP call.
 */
agc_status_t agc_integ_kepprep(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL) ||
        !AGC_ASSERT(agc_integ_valid_ctx(ctx))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_kepler_in_t in;
    in.rrect = ctx->sv.rcv;
    in.vrect = ctx->sv.vcv;
    in.tau = ctx->tau;
    in.xkepnew = 0;
    in.tc = ctx->sv.tc;
    in.xprev = ctx->sv.xkep;
    in.body = agc_flag_test(AGC_FLAG_MOONFLAG) ? AGC_CONIC_MOON : AGC_CONIC_EARTH;
    agc_dp_t xnew = 0;
    agc_status_t st = agc_conic_kepprep(ip, &in, &xnew);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    in.rrect = ctx->sv.rrect;                 /* KEPLERN works from RRECT  */
    in.vrect = ctx->sv.vrect;
    in.xkepnew = xnew;
    agc_kepler_out_t kout;
    st = agc_conic_kepler(ip, &in, &kout);
    if ((st == AGC_STATUS_OK) || (st == AGC_STATUS_NO_CONVERGE)) {
        ctx->sv.rcv = kout.rcv;               /* ITERCTR exhausted counts  */
        ctx->sv.vcv = kout.vcv;               /* as converged (BHIZ)       */
    } else if (st != AGC_STATUS_OVERFLOW) {
        return st;
    } else {
        /* TIMEOVFL: RCV, VCV keep their previous contents.               */
    }
    ctx->sv.tc = kout.tc;
    ctx->sv.xkep = kout.xprev;                /* XPREV EQUALS XKEP         */
    return AGC_STATUS_OK;
}

/* FBR3 (p. 1336): advance the conic by DT/2.  TIX,1 skips the ROUND on
 * the mid-step entry (DIFEQCNT -12), so DT/2 SR 9 is truncated there and
 * rounded at the end of the step (DIFEQCNT -24).  DT/2 2^19 -> 2^28 cs. */
static agc_status_t fbr3(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT((ctx->stage == 1) || (ctx->stage == 2))) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ip != NULL);
    const agc_dp_t d = (ctx->stage == 1) ? agc_dp_shr(ctx->dt2, 9)
                                         : agc_dp_shr_round(ctx->dt2, 9);
    ctx->tau = agc_dp_add(ip, d, ctx->sv.tc);         /* TAU. = TC + DT/2 */
    ctx->sv.tet = agc_dp_add(ip, d, ctx->sv.tet);     /* TET += DT/2      */
    return agc_integ_kepprep(ip, ctx);
}

/* GOBAQUE / INT-ABRT (p. 1340): on overflow back the conic up by H to the
 * start of the step, rectify and try again; with no deviation left the
 * integration is hopeless: POODOO 00430.                                 */
static agc_integ_next_t gobaque(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    (void)AGC_ASSERT(ip != NULL);
    (void)AGC_ASSERT(agc_integ_valid_ctx(ctx));
    if (agc_vec_abval(ip, ctx->sv.deltav) == 0) {
        (void)agc_poodoo(AGC_INTEG_ALARM_INT_ABRT);   /* status in ABORT  */
        return INTEG_NEXT_ABORT;
    }
    const agc_dp_t h28 = agc_dp_shr(ctx->h, 9);       /* H 2^19 -> 2^28   */
    ctx->tau = agc_dp_sub(ip, ctx->sv.tc, h28);
    ctx->sv.tet = agc_dp_sub(ip, ctx->sv.tet, h28);
    if (agc_integ_kepprep(ip, ctx) != AGC_STATUS_OK) {
        return INTEG_NEXT_FAIL;
    }
    if (agc_integ_rectify(ip, ctx) != AGC_STATUS_OK) {
        return INTEG_NEXT_FAIL;
    }
    agc_integ_flag(AGC_FLAG_RPQFLAG, true);
    return INTEG_NEXT_TESTLOOP;
}

/* ======================================================================
 * Disturbing accelerations: ACCOMP, GAMCOMP, OBLATE, QUALITY1-3
 * ==================================================================== */

typedef enum acc_result {
    ACC_OK = 0,      /* FV complete, go to NBRANCH                          */
    ACC_OVF,         /* overflow: GOBAQUE                                   */
    ACC_FAIL         /* a called routine rejected its arguments            */
} acc_result_t;

/* VSQ keeping the triple-precision sum (VSQSUB leaves MPAC..MPAC+2).    */
static agc_tp_t vsq_tp(agc_interp_t *ip, agc_vec_t v)
{
    agc_tp_t s = agc_dp_mul_tp(v.c[0], v.c[0]);
    (void)AGC_ASSERT(ip != NULL);
    for (int i = 1; i < AGC_VEC_N; i++) {
        s = agc_tp_add(ip, s, agc_dp_mul_tp(v.c[i], v.c[i]));
    }
    (void)AGC_ASSERT((s >= 0) && (s <= AGC_TP_MAX));
    return s;
}

/* DMP x SL1R: TP product, shifted left one, rounded to DP.              */
static agc_dp_t dmp_sl1r(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    (void)AGC_ASSERT((a >= AGC_DP_MIN) && (a <= AGC_DP_MAX));
    (void)AGC_ASSERT((b >= AGC_DP_MIN) && (b <= AGC_DP_MAX));
    agc_tp_t t = agc_dp_mul_tp(a, b);
    t = agc_tp_add(ip, t, t);
    return agc_tp_round_dp(t);
}

/* GAMCOMP push-list work cells.                                         */
typedef struct gam_work {
    agc_dp_t bsq;    /* 0D  |BETAV/2|^2 normalised by n1                  */
    agc_dp_t rho_q;  /* 2D  (ALPHAM_n2 / 2) / BETAM_n3, SR1R              */
    agc_dp_t rho;    /* 4D  rho = ALPHAM / BETAM at the ASCALE scale       */
    int      n1;     /* -C(31D)                                           */
    int      xs;     /* X2 = S1 - n2 + n3 (ASCALE word 1 + norm counts)   */
} gam_work_t;

/* GAMCOMP first half (p. 1338-1339): normalised B^2, BETAV -> unit,
 * BETAM = |BETAV|, rho = ALPHAM / BETAM.                                */
static void gamcomp_rho(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x1,
                        gam_work_t *g)
{
    int n2 = 0;
    int n3 = 0;
    agc_tp_t b2 = tp_norm(ip, vsq_tp(ip, agc_vec_shr(ctx->betav, 1)), &g->n1);
    g->bsq = agc_tp_round_dp(b2);                       /* NORM ROUND 31D */
    agc_dp_t am = agc_dp_shr(agc_dp_norm(ctx->alpham, &n2), 1); /* NORM 32D SR1 */
    agc_dp_t bm = 0;
    ctx->betav = agc_vec_unit(ip, ctx->betav, &bm, NULL);  /* UNIT, 36D   */
    ctx->betam = bm;                                       /* BETAM       */
    bm = agc_dp_norm(bm, &n3);                             /* NORM 33D    */
    g->rho_q = agc_dp_shr_round(agc_dp_div(ip, am, bm), 1); /* BDDV SR1R  */
    g->xs = ASCALE_S1[x1 / 2] - n2 + n3;   /* XCHX,2 S1 XAD,2 32D XSU,2 33D */
    g->rho = agc_integ_dshift(ip, g->rho_q, 1 + g->xs);    /* SR* 0 -1,2  */
    (void)AGC_ASSERT((x1 >= 0) && (x1 < 2 * ASCALE_N));
    (void)AGC_ASSERT(ctx->betam >= 0);
}

/*
 * GAMCOMP (ORBITAL_INTEGRATION p. 1338-1340): add to FV the acceleration
 * difference -MU [f(q) B + A] / |B|^3 for a body at BETAV (Battin's f(q)
 * form; A = ALPHAV unit vector x ALPHAM).  x1 = ASCALE index (0/2 primary,
 * 4/6 secondary, 8/10 sun), x2 = MU index (0 earth, 2 moon, 4 sun).
 * Returns true when the final VAD overflows (BOV GOBAQUE).
 */
static bool gamcomp(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x1, int x2)
{
    gam_work_t g;
    if (!AGC_ASSERT((x1 >= 0) && (x1 < 2 * ASCALE_N) && ((x1 % 2) == 0)) ||
        !AGC_ASSERT((x2 == 0) || (x2 == 2) || (x2 == 4))) {
        return true;
    }
    gamcomp_rho(ip, ctx, x1, &g);
    agc_dp_t pd6 = agc_dp_shr_round(g.rho, 1);                    /* 6D   */
    const agc_dp_t dot = agc_dp_shl_round(ip, agc_vec_dot(ip, ctx->alphav,
                                                         ctx->betav), 1);
    pd6 = agc_dp_sub(ip, pd6, dot);                    /* q: BDSU, 6D      */
    const agc_dp_t pd8 = agc_dp_shl(ip, agc_dp_mul(ip, pd6, g.rho), 1);
    const agc_dp_t pd10 = agc_dp_add(ip, pd8, K_DQUARTER);
    const agc_dp_t pd12 = agc_dp_mul(ip, agc_dp_sqrt(ip, pd10), pd10);
    const agc_dp_t pd14 = agc_dp_add(ip, agc_dp_shl(ip, pd12, 1), K_DQUARTER);
    agc_dp_t t = agc_dp_add(ip, pd10, K_HALFDP);
    t = agc_dp_shl(ip, agc_dp_mul(ip, t, pd8), 1);
    t = agc_dp_div(ip, agc_dp_add(ip, t, K_THREE_8), pd14);
    t = agc_dp_mul(ip, t, pd6);                        /* G/2, DMPR 6      */
    agc_vec_t v = agc_vec_add(ip, agc_vec_shr(ctx->alphav, 3),
                              agc_vec_scale(ip, ctx->betav, t));  /* 16D */
    int n4 = 0;
    const agc_tp_t den = tp_norm(ip, agc_dp_mul_tp(g.bsq, pd12), &n4);
    agc_dp_t f = agc_dp_div(ip, g.rho_q, agc_tp_round_dp(den)); /* BDDV 2 */
    f = agc_dp_neg(agc_tp_to_dp(agc_dp_mul_tp(f, agc_integ_mu(x2))));
    v = agc_vec_scale(ip, v, f);                       /* DCOMP VXSC       */
    const int xsum = g.xs + ASCALE_S2[x1 / 2] + n4 + g.n1;  /* XAD XSU XSU */
    (void)agc_interp_test_ovf(ip);                     /* BOV +1 (reset)   */
    v = agc_integ_vshift(ip, v, 1 + xsum);             /* VSR* 0 -1,2      */
    ctx->fv = agc_vec_add(ip, v, ctx->fv);             /* VAD FV, STORE FV */
    return agc_interp_test_ovf(ip);                    /* BOV -> GOBAQUE   */
}

/* LUNPOS: CM ephemeris (earth-centred moon, 2^29 m); LM LUNPOS = LSPOS.  */
static agc_status_t lunpos(agc_interp_t *ip, agc_dp_t t, agc_vec_t *moon)
{
    (void)AGC_ASSERT(moon != NULL);
    (void)AGC_ASSERT(ip != NULL);
    if (agc_state_vehicle() == AGC_VEHICLE_LM) {
        agc_vec_t sun;
        return agc_lsposition(ip, t, moon, &sun);
    }
    agc_ephem_csm_t eph;
    const agc_status_t st = agc_ephem_get_csm_data(&eph);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_ephem_csm_lunpos(ip, &eph, agc_erase_tephem(), t, moon);
}

/* LUNVEL: CM moon velocity 2^7 m/cs. The LM LUNVEL is a bare RVQ "TO
 * FOOL INTEGRATION" (unreachable there); zero is returned.               */
static agc_status_t lunvel(agc_interp_t *ip, agc_dp_t t, agc_vec_t *vmoon)
{
    (void)AGC_ASSERT(vmoon != NULL);
    (void)AGC_ASSERT(ip != NULL);
    if (agc_state_vehicle() == AGC_VEHICLE_LM) {
        *vmoon = agc_vec_zero();      /* DEVIATION: MPAC left as is in AGC */
        return AGC_STATUS_OK;
    }
    agc_ephem_csm_t eph;
    const agc_status_t st = agc_ephem_get_csm_data(&eph);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    return agc_ephem_csm_lunvel(ip, &eph, agc_erase_tephem(), t, vmoon);
}

/* ACCOMP third-body part (p. 1337-1338, MIDFLAG set): LSPOS, RPQV, RPSV,
 * VECTAB +6 (and LM RQVV), GETRPSV, GAMCOMP for the secondary body and
 * for the sun.                                                           */
static acc_result_t third_body(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_vec_t moon;
    agc_vec_t sun;
    const int p = ctx->pbody;
    if (agc_lsposition(ip, ctx->sv.tet, &moon, &sun) != AGC_STATUS_OK) {
        return ACC_FAIL;
    }
    int x2 = 2;                                  /* secondary = moon       */
    if (agc_flag_test(AGC_FLAG_MOONFLAG)) {
        moon = agc_vec_neg(moon);                /* moon -> earth          */
        x2 = 0;                                  /* secondary = earth      */
    }
    ctx->betav = moon;
    ctx->rpqv = moon;
    ctx->rpsv = sun;
    const bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM);
    const bool is_cislunar_midcourse_navigation = lm && (agc_erase_modreg() == 23);    /* MODREG OCT27  */
    if (is_cislunar_midcourse_navigation || agc_flag_test(AGC_FLAG_DIM0FLAG)) {
        const agc_vec_t r = agc_vec_scale(ip, ctx->alphav, ctx->alpham);
        const agc_vec_t rq = agc_vec_sub(ip, agc_integ_vshift(ip, r, x2 - 1),
                                         ctx->betav);     /* VSR* 1,2 VSU */
        ctx->vectab[(2 * ctx->stage) + 1] = rq;
        if (lm) {
            ctx->rqvv = rq;                      /* LM: STORE RQVV         */
        }
    }
    agc_integ_flag(AGC_FLAG_RPQFLAG, false);     /* GETRPSV                */
    if (agc_flag_test(AGC_FLAG_MOONFLAG)) {
        ctx->rpsv = agc_vec_add(ip, agc_vec_shr(ctx->rpqv, 9), ctx->rpsv);
    }
    if (gamcomp(ip, ctx, p + 4, x2)) {
        return ACC_OVF;
    }
    ctx->betav = ctx->rpsv;
    return gamcomp(ip, ctx, p + 8, 4) ? ACC_OVF : ACC_OK;
}

/* OBLATE push-list cells shared with QUALITY1-3.                         */
typedef struct obl_work {
    agc_vec_t urpv;  /* 14D URPV unit R in moon-fixed axes, 2^1 (moon)     */
    agc_dp_t  cphi;  /* 18D COSPHI/2: z of the unit R in planet axes, 2^1  */
    agc_vec_t uz;    /* 20D UZ planet polar axis, 2^1                       */
    agc_dp_t  r4n;   /* 0D  |R|^4 normalised                               */
    int       x1;    /* X1 = -(4 N + M): N, M the two NORM counts          */
} obl_work_t;

/* Legendre terms (COMTERM, p. 1341): P2/64, P3/32, P4/128, P5/1024.     */
typedef struct legendre {
    agc_dp_t p2;     /* 0D */
    agc_dp_t p3;     /* 2D */
    agc_dp_t p4;     /* 4D */
    agc_dp_t p5;     /* MPAC */
} legendre_t;

/* OBLATE moon branch (p. 1341): URPV = R-TO-RP(ALPHAV), UZ = unit of
 * MT (ZUNIT + 504LM x ZUNIT) -- the lunar pole in reference axes.  The
 * AGC reuses the MMATRIX left in the push list by R-TO-RP; MOONMX is
 * called again here for the same matrix (same TET).                     */
static agc_status_t moon_axes(agc_interp_t *ip, const agc_integ_ctx_t *ctx,
                              obl_work_t *w)
{
    agc_navaux_padload_t pad;
    agc_mat_t m;
    const agc_vec_t zunit = agc_vec_make(0, 0, AGC_DP_HALF);
    agc_status_t st = agc_navaux_get_padload(&pad);
    if (st == AGC_STATUS_OK) {
        st = agc_planet_r_to_rp(ip, &pad, true, ctx->alphav, ctx->sv.tet,
                                &w->urpv);
    }
    if (st == AGC_STATUS_OK) {
        st = agc_planet_moonmx(ip, &pad, ctx->sv.tet, &m);
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    w->cphi = w->urpv.c[2];
    agc_vec_t ax = agc_vec_add(ip, agc_vec_cross(ip, pad.lm504, zunit), zunit);
    w->uz = agc_vec_unit(ip, agc_vec_mat(ip, ax, &m), NULL, NULL);
    (void)AGC_ASSERT(ip != NULL);
    return AGC_STATUS_OK;
}

/* COMTERM Legendre polynomials in COSPHI/2.                             */
static void legendre_terms(agc_interp_t *ip, agc_dp_t cphi, legendre_t *l)
{
    (void)AGC_ASSERT(l != NULL);
    (void)AGC_ASSERT((cphi >= AGC_DP_MIN) && (cphi <= AGC_DP_MAX));
    l->p2 = agc_dp_mul(ip, cphi, K_3_32);                     /* P2/64 0D */
    l->p3 = agc_dp_sub(ip, agc_dp_mul(ip, agc_dp_sq(ip, cphi), K_15_16),
                       K_3_64);                                /* P3/32 2D */
    agc_dp_t t = dmp_sl1r(ip, agc_dp_mul(ip, l->p3, cphi), K_7_12);
    l->p4 = agc_dp_sub(ip, t, agc_dp_mul(ip, l->p2, K_2_3));  /* P4/128 4D */
    t = agc_dp_mul(ip, agc_dp_mul(ip, l->p4, cphi), K_9_16);
    l->p5 = agc_dp_sub(ip, t, agc_dp_mul(ip, l->p3, K_5_128)); /* P5/1024 */
}

/* COMTERM (p. 1341-1342): TVEC = (((P5 J4/J3 / R + P4) 2J3RE/J2 / R + P3)
 * UR - (((P4 J4/J3 / R + P3) 2J3RE/J2 / R + P2) UZ, before the J2/R^4
 * factor.                                                               */
static agc_vec_t comterm_tvec(agc_interp_t *ip, const agc_integ_ctx_t *ctx,
                              const obl_work_t *w, const legendre_t *l)
{
    const int b = ctx->pbody / 2;
    (void)AGC_ASSERT((b == 0) || (b == 1));
    (void)AGC_ASSERT(ctx->alpham > 0);
    agc_dp_t t = agc_dp_mul(ip, l->p5, J4REQ_J3[b]);          /* DMP*     */
    t = agc_dp_add(ip, agc_dp_div(ip, t, ctx->alpham), l->p4);
    t = agc_dp_mul(ip, t, J3RE_J2_2[b]);                      /* DMPR*    */
    t = agc_dp_add(ip, agc_dp_div(ip, t, ctx->alpham), l->p3);
    const agc_vec_t tvec = agc_vec_scale(ip, ctx->alphav, t);  /* TVEC     */
    t = agc_dp_shr(agc_dp_mul(ip, l->p4, J4REQ_J3[b]), 1);    /* DMP* SR1 */
    t = agc_dp_add(ip, agc_dp_div(ip, t, ctx->alpham), l->p3);
    t = agc_dp_shr(agc_dp_mul(ip, t, J3RE_J2_2[b]), 3);       /* DMPR* SR3*/
    t = agc_dp_add(ip, agc_dp_div(ip, t, ctx->alpham), l->p2);
    const agc_vec_t u = agc_vec_shl(ip, agc_vec_scale(ip, w->uz, t), 1);
    return agc_vec_sub(ip, tvec, u);                           /* BVSU     */
}

/* QUALITY1 / QUALITY2 (CM RT8_OP_CODES p. 1516, LM p. 1248): the lunar
 * J22 term x R^4, B61:  E3J22R2M ( 5 (Y^2 - X^2) UR + (X, -Y, 0) ) / 2.  */
static agc_vec_t quality2(agc_interp_t *ip, const obl_work_t *w)
{
    const agc_vec_t u = w->urpv;
    (void)AGC_ASSERT(ip != NULL);
    (void)AGC_ASSERT(w->r4n >= 0);
    agc_dp_t t = agc_dp_sub(ip, agc_dp_sq(ip, u.c[1]), agc_dp_sq(ip, u.c[0]));
    t = agc_dp_mul(ip, t, K_5_8);                             /* DMP 5/8  */
    agc_vec_t q = agc_vec_shl(ip, agc_vec_scale(ip, u, t), 3); /* VSL3 2D */
    q.c[0] = agc_dp_add(ip, agc_dp_shr(u.c[0], 1), q.c[0]);   /* SR1 DAD  */
    q.c[1] = agc_dp_sub(ip, q.c[1], agc_dp_shr(u.c[1], 1));   /* SR1 BDSU */
    return agc_vec_scale(ip, q, agc_dp_from_sp(e3j22r2m));    /* SLOAD VXSC */
}

/* QUALITY3 (p. 1342-1343): the C31 term, added to the J22 term, scaled
 * by 1/R^4, rotated to reference axes (RP-TO-R) and added to FV.        */
static acc_result_t quality3(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                             const obl_work_t *w, agc_vec_t j22)
{
    const agc_vec_t u = w->urpv;
    const agc_dp_t c2 = agc_dp_sq(ip, w->cphi);                    /* DSQ  */
    const agc_dp_t c5 = agc_dp_mul(ip, c2, K_5_8);                 /* 8D   */
    agc_dp_t t = agc_dp_sub(ip, K_D1_32,
                            agc_dp_add(ip, agc_dp_shr(c2, 2), c5)); /* 1-7c2 */
    t = agc_dp_mul(ip, agc_dp_mul(ip, t, u.c[0]), K_5_8);
    agc_vec_t v = agc_vec_shl(ip, agc_vec_scale(ip, u, t), 5);     /* B5   */
    v.c[0] = agc_dp_add(ip, agc_dp_sub(ip, c5, K_D1_32), v.c[0]);
    t = agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_mul(ip, u.c[0], u.c[2]), K_5_8), 1);
    v.c[2] = agc_dp_add(ip, t, v.c[2]);
    int n = 0;
    const agc_dp_t rn = agc_dp_norm(ctx->alpham, &n);              /* X2=-n */
    t = agc_dp_div(ip, agc_dp_from_sp(e32c31rm), rn);
    v = agc_integ_vshift(ip, agc_vec_scale(ip, v, t), n - 3);      /* 0 -3,2 */
    v = agc_vec_add(ip, v, j22);
    v = agc_integ_vshift(ip, v, -27 - w->x1);                      /* 0 -27D,1 */
    v = agc_vec_div(ip, v, w->r4n);                                /* V/SC 0D */
    agc_navaux_padload_t pad;
    agc_vec_t r = agc_vec_zero();
    if ((agc_navaux_get_padload(&pad) != AGC_STATUS_OK) ||
        (agc_planet_rp_to_r(ip, &pad, true, v, ctx->sv.tet, &r) != AGC_STATUS_OK)) {
        return ACC_FAIL;
    }
    (void)agc_interp_test_ovf(ip);        /* "OVERFLOW INDICATOR RESET IN RP-TO-R" */
    ctx->fv = agc_vec_add(ip, r, ctx->fv);
    return agc_interp_test_ovf(ip) ? ACC_OVF : ACC_OK;
}

/*
 * OBLATE (p. 1341-1342): within RDE (80467 km) of the earth or RDM
 * (16093 km) of the moon add the zonal harmonics (earth J2 J3 J4, moon
 * J2 J3) and, for the moon, QUALITY1-3 (J22, C31).  ALPHAV = unit R,
 * ALPHAM = |R|.  NBRANCH follows in the caller.
 */
static acc_result_t oblate(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const int b = ctx->pbody / 2;
    obl_work_t w;
    legendre_t l;
    if (!AGC_ASSERT((b == 0) || (b == 1)) ||
        (agc_dp_sub(ip, ctx->alpham, RD_TAB[b]) >= 0)) {
        return ACC_OK;                                   /* BPL NBRANCH   */
    }
    const bool moon = agc_flag_test(AGC_FLAG_MOONFLAG);
    w.urpv = ctx->alphav;
    if (!moon) {                                         /* COSPHIE       */
        w.cphi = ctx->alphav.c[2];
        w.uz = agc_vec_make(0, 0, AGC_DP_HALF);          /* ZUNIT         */
    } else if (moon_axes(ip, ctx, &w) != AGC_STATUS_OK) {
        return ACC_FAIL;
    } else {
        /* URPV, COSPHI/2, UZ set */
    }
    legendre_terms(ip, w.cphi, &l);
    const agc_vec_t tvec = comterm_tvec(ip, ctx, &w, &l);
    int n = 0;
    int m = 0;
    const agc_dp_t r2 = agc_dp_sq(ip, agc_dp_norm(ctx->alpham, &n)); /* NORM X1 DSQ */
    w.r4n = agc_tp_to_dp(tp_norm(ip, agc_dp_mul_tp(r2, r2), &m));    /* DSQ NORM S1 */
    const agc_dp_t k = agc_dp_div(ip, J2REQSQ_TAB[b], w.r4n);       /* BDDV*      */
    agc_vec_t v = agc_vec_scale(ip, tvec, k);
    (void)agc_interp_test_ovf(ip);                       /* BOV +1 (reset) */
    w.x1 = -(4 * n) - m;                                 /* XAD,1 X1 X1 S1 */
    v = agc_integ_vshift(ip, v, -22 - w.x1);             /* VSL* 0 -22D,1  */
    ctx->fv = agc_vec_add(ip, v, ctx->fv);
    if (agc_interp_test_ovf(ip)) {
        return ACC_OVF;                                  /* BOV GOBAQUE    */
    }
    if (!moon) {
        return ACC_OK;                                   /* QUALITY1 BOF   */
    }
    return quality3(ip, ctx, &w, quality2(ip, &w));
}

/* ACCOMP (p. 1337): FV = disturbing acceleration at ALPHAV (deviation).
 * Primary body by GAMCOMP; with MIDFLAG the secondary body and the sun;
 * then OBLATE.                                                          */
static acc_result_t accomp(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const int p = ctx->pbody;
    if (!AGC_ASSERT((ctx->stage >= 0) && (ctx->stage <= 2)) ||
        !AGC_ASSERT((p == INTEG_PBODY_EARTH) || (p == INTEG_PBODY_MOON))) {
        return ACC_FAIL;
    }
    ctx->fv = agc_vec_zero();
    ctx->betav = agc_vec_add(ip, agc_vec_shr(ctx->alphav, 7 + p), ctx->sv.rcv);
    if (agc_flag_test(AGC_FLAG_DIM0FLAG)) {
        ctx->vectab[2 * ctx->stage] = ctx->betav;       /* VECTAB,2       */
    }
    agc_dp_t am = 0;
    ctx->alphav = agc_vec_unit(ip, ctx->alphav, &am, NULL);
    ctx->alpham = am;                                   /* 36D            */
    if (gamcomp(ip, ctx, p, p)) {
        return ACC_OVF;
    }
    ctx->alphav = ctx->betav;                           /* unit R         */
    ctx->alpham = ctx->betam;                           /* |R|            */
    if (agc_flag_test(AGC_FLAG_MIDFLAG)) {
        const acc_result_t r = third_body(ip, ctx);
        if (r != ACC_OK) {
            return r;
        }
    }
    return oblate(ip, ctx);
}

/* DOW..1 (p. 1352): gravity-gradient derivative of a W column,
 * BETAM [3 (A.UR) UR - A] / |R|^3 for R = VECTAB entry, A = ALPHAV.      */
static agc_vec_t dow1(agc_interp_t *ip, const agc_integ_ctx_t *ctx, agc_vec_t r)
{
    const agc_vec_t a16 = agc_vec_shr(ctx->alphav, 4);                /* 0D */
    agc_dp_t rmag = 0;
    const agc_vec_t u = agc_vec_unit(ip, r, &rmag, NULL);             /* 36D */
    agc_vec_t g = agc_vec_proj(ip, ctx->alphav, u);                   /* VPROJ */
    g = agc_vec_sub(ip, agc_vec_scale(ip, g, K_3_4), a16);
    int n = 0;
    int m = 0;
    const agc_dp_t rn = agc_dp_norm(rmag, &n);                        /* S2   */
    const agc_tp_t r3 = tp_norm(ip, agc_dp_mul_tp(agc_dp_sq(ip, rn), rn), &m);
    const agc_dp_t k = agc_dp_div(ip, agc_dp_shr(ctx->betam, 1), agc_tp_to_dp(r3));
    g = agc_vec_scale(ip, g, k);
    (void)AGC_ASSERT(rmag >= 0);
    (void)AGC_ASSERT(ctx->betam > 0);
    return agc_integ_vshift(ip, g, (3 * n) + m - 8);                  /* 0 -8D,2 */
}

/* DOW.. (p. 1352): FV for the W integration from the stored positions.   */
static void dow(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const int p = ctx->pbody;
    const int k = ctx->stage;
    (void)AGC_ASSERT((k >= 0) && (k <= 2));
    (void)AGC_ASSERT((p == INTEG_PBODY_EARTH) || (p == INTEG_PBODY_MOON));
    ctx->betam = agc_integ_mu(p);                          /* MUEARTH,2   */
    ctx->fv = dow1(ip, ctx, ctx->vectab[2 * k]);
    if (!agc_flag_test(AGC_FLAG_MIDFLAG)) {
        return;
    }
    ctx->betam = agc_integ_mu(2 - p);                      /* MUEARTH -2,2, X2 = -PBODY */
    agc_vec_t f2 = dow1(ip, ctx, ctx->vectab[(2 * k) + 1]);
    if (!agc_flag_test(AGC_FLAG_MOONFLAG)) {
        f2 = agc_vec_shr(f2, 6);                           /* VSR6        */
    }
    ctx->fv = agc_vec_add(ip, f2, ctx->fv);
}

/* ======================================================================
 * Nystrom step: DIFEQ0, DIFEQ+0/+1/+2, DIFEQCOM
 * ==================================================================== */

/* DIFEQ0 (p. 1345): ALPHAV = YV, DIFEQCNT = 0, H = 0.                    */
static void difeq0(agc_integ_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    ctx->alphav = ctx->yv;
    ctx->stage = 0;
    ctx->h = 0;
    (void)AGC_ASSERT(ctx->dt2 != 0);
}

/* DIFEQ+0 / DIFEQ+1 (p. 1348) and DIFEQCOM (p. 1350): accumulate PHIV,
 * PSIV, advance H by DT/2 and predict ALPHAV = YV + H (ZV + H FV / 2).   */
static void difeq01(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    (void)AGC_ASSERT((ctx->stage == 0) || (ctx->stage == 1));
    if (ctx->stage == 0) {
        ctx->phiv = agc_vec_shr(ctx->fv, 3);                      /* DIFEQ+0 */
    } else {
        const agc_vec_t f2 = agc_vec_shr(ctx->fv, 1);             /* DIFEQ+1 */
        ctx->psiv = agc_vec_add(ip, f2, ctx->phiv);
        ctx->phiv = agc_vec_add(ip, agc_vec_shr(f2, 1), ctx->phiv);
    }
    ctx->h = agc_dp_add(ip, ctx->dt2, ctx->h);                    /* DIFEQCOM */
    ctx->stage += 1;
    agc_vec_t a = agc_vec_shr(agc_vec_scale(ip, ctx->fv, ctx->h), 1);
    a = agc_vec_scale(ip, agc_vec_add(ip, a, ctx->zv), ctx->h);
    ctx->alphav = agc_vec_add(ip, a, ctx->yv);
    (void)AGC_ASSERT(ctx->stage <= 2);
}

/* DIFEQ+2 (p. 1348): end of the step, YV and ZV updated.                */
static void difeq2(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx->stage == 2);
    (void)AGC_ASSERT(ip != NULL);
    const agc_dp_t h23 = agc_dp_mul(ip, ctx->h, K_2_3);
    agc_vec_t y = agc_vec_shl(ip, agc_vec_scale(ip, ctx->phiv, h23), 1);
    y = agc_vec_scale(ip, agc_vec_add(ip, y, ctx->zv), ctx->h);
    ctx->yv = agc_vec_add(ip, y, ctx->yv);
    agc_vec_t z = agc_vec_add(ip, agc_vec_shr(ctx->fv, 3), ctx->psiv);
    z = agc_vec_shl(ip, agc_vec_scale(ip, z, h23), 1);
    ctx->zv = agc_vec_add(ip, z, ctx->zv);
}

/* INTGRATE .. ENDSTATE (p. 1345-1349) for the state vector: three ACCOMP
 * evaluations at H = 0, DT/2, DT with FBR3 conic advances in between.   */
static agc_integ_next_t state_step(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    (void)AGC_ASSERT(ip != NULL);
    difeq0(ctx);
    for (int k = 0; k < 3; k++) {                 /* NBRANCH stages        */
        const acc_result_t r = accomp(ip, ctx);
        if (r == ACC_OVF) {
            return gobaque(ip, ctx);
        }
        if (!AGC_ASSERT(r == ACC_OK)) {
            return INTEG_NEXT_FAIL;
        }
        if (k == 2) {
            break;
        }
        difeq01(ip, ctx);
        if (fbr3(ip, ctx) != AGC_STATUS_OK) {
            return INTEG_NEXT_FAIL;
        }
    }
    difeq2(ip, ctx);
    if (agc_interp_test_ovf(ip)) {                /* ENDSTATE BOV GOBAQUE  */
        return gobaque(ip, ctx);
    }
    ctx->sv.nuv = ctx->zv;
    ctx->sv.deltav = ctx->yv;
    if (agc_flag_test(AGC_FLAG_MIDAVFLG)) {
        return INTEG_NEXT_CKMID2;
    }
    if (!agc_flag_test(AGC_FLAG_DIM0FLAG)) {
        return INTEG_NEXT_TESTLOOP;
    }
    return INTEG_NEXT_WMATRIX;
}

/* WMATEND (p. 1350): W overflowed. Stop integrating and invalidating it;
 * pick up a permanent state vector update instead; ALARM 421.            */
static agc_integ_next_t wmatend(void)
{
    agc_integ_flag(AGC_FLAG_DIM0FLAG, false);
    agc_integ_flag(AGC_FLAG_ORBWFLAG, false);
    agc_integ_flag(AGC_FLAG_RENDWFLG, false);
    agc_integ_flag(AGC_FLAG_STATEFLG, true);
    const agc_status_t st = agc_alarm(AGC_INTEG_ALARM_WMATEND);
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    (void)AGC_ASSERT(!agc_flag_test(AGC_FLAG_DIM0FLAG));
    return INTEG_NEXT_TESTLOOP;
}

/* One W column over the step just taken: DIFEQ0, DOW.., DIFEQ+0/+1/+2. */
static void wcolumn_step(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_JSWITCH));
    (void)AGC_ASSERT((ctx->colreg >= 0) && (ctx->colreg < AGC_INTEG_W_COLUMNS));
    difeq0(ctx);
    for (int k = 0; k < 3; k++) {
        dow(ip, ctx);                             /* NBRANCH -> DIFEQ+k    */
        if (k < 2) {
            difeq01(ip, ctx);                     /* DIFEQCOM -> DOW..     */
        }
    }
    difeq2(ip, ctx);
}

/*
 * AMOVED, NEXTCOL, DIFEQ+2 (JSWITCH), RELOADSV (p. 1348-1350): integrate
 * the W columns COLREG = -30 (6x6, D6OR9FLG clear) or -48 (9x9) up to 0,
 * i.e. columns 5..0 or 8..0. Position part blk[0][m], velocity part
 * blk[1][m]; YV/ZV = W SR3.  Returns RELOAD (RELOADSV), or TESTLOOP after
 * WMATEND.
 */
agc_integ_next_t agc_integ_wcolumns(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL) ||
        !AGC_ASSERT(agc_integ_valid_ctx(ctx))) {
        return INTEG_NEXT_FAIL;
    }
    agc_integ_flag(AGC_FLAG_JSWITCH, true);                  /* AMOVED   */
    const int first = agc_flag_test(AGC_FLAG_D6OR9FLG) ? 8 : 5;
    for (int col = first; col >= 0; col--) {
        ctx->colreg = col;
        (void)agc_integ_grp2pc();                            /* NEXTCOL  */
        ctx->yv = agc_vec_shr(ctx->w.blk[0][col], 3);
        ctx->zv = agc_vec_shr(ctx->w.blk[1][col], 3);
        wcolumn_step(ip, ctx);
        (void)agc_integ_grp2pc();                            /* DIFEQ+2  */
        ctx->w.blk[1][col] = agc_vec_shl(ip, ctx->zv, 3);    /* W +54D,2 */
        const agc_vec_t y = agc_vec_shl(ip, ctx->yv, 3);
        if (agc_interp_test_ovf(ip)) {
            return wmatend();
        }
        ctx->w.blk[0][col] = y;                              /* W,2      */
        (void)agc_integ_grp2pc();
    }
    return INTEG_NEXT_RELOAD;                                /* RELOADSV */
}

/* RECTEST (p. 1344): rectify when |TDELTAV| or |TNUV| reaches 3/4 of its
 * scale (or overflows), or |TDELTAV| >= .01 |RCV| (10D).                */
static agc_status_t rectest(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const int p = ctx->pbody;
    (void)AGC_ASSERT(ctx->rc >= 0);
    const agc_dp_t d = agc_vec_abval(ip, ctx->sv.deltav);
    bool rect = agc_interp_test_ovf(ip);                       /* BOV      */
    if (!rect) {
        rect = (agc_dp_sub(ip, d, K_3_4) >= 0);                /* DSU BPL  */
    }
    if (!rect) {
        agc_dp_t q = agc_dp_shr(d, 7 + p);                     /* SL* 0 -7,2 */
        q = agc_dp_sub(ip, agc_dp_div(ip, q, ctx->rc), K_RECRATIO);
        rect = (q >= 0);
    }
    if (!rect) {
        const agc_dp_t e = agc_dp_sub(ip, agc_vec_abval(ip, ctx->sv.nuv), K_3_4);
        rect = agc_interp_test_ovf(ip) || (e >= 0);            /* BOV BMN  */
    }
    if (rect) {
        return agc_integ_rectify(ip, ctx);                     /* CALLRECT */
    }
    (void)AGC_ASSERT(!rect);
    return AGC_STATUS_OK;
}

/* LM TIMESTEP origin test (Luminary099 p. 1239-1240): only when MIDFLAG,
 * which Luminary099 never sets (RME = RMM = DPPOSMAX), and only when the
 * vehicle is receding, (R.V) DT >= 0.                                    */
static agc_status_t lm_sphere(agc_interp_t *ip, agc_integ_ctx_t *ctx, bool *sw)
{
    agc_vec_t sun;
    *sw = false;
    (void)AGC_ASSERT(ip != NULL);
    const agc_dp_t rv = agc_dp_mul(ip, agc_vec_dot(ip, ctx->sv.rcv, ctx->sv.vcv),
                                   ctx->dt2);
    if (rv < 0) {
        return AGC_STATUS_OK;                                  /* RECTEST  */
    }
    if (agc_flag_test(AGC_FLAG_MOONFLAG)) {                    /* LUNSPH   */
        if (agc_dp_sub(ip, agc_dp_shr(ctx->rc, 2), K_RSPHERE) < 0) {
            return AGC_STATUS_OK;
        }
        if (agc_flag_test(AGC_FLAG_RPQFLAG)) {
            agc_vec_t m;
            const agc_status_t st = lunpos(ip, ctx->sv.tet, &m);
            if (st != AGC_STATUS_OK) {
                return st;
            }
            ctx->rpqv = agc_vec_neg(m);
        }
        *sw = true;                                            /* DOSWITCH */
        return AGC_STATUS_OK;
    }
    if (agc_flag_test(AGC_FLAG_RPQFLAG)) {                     /* else EARSPH */
        const agc_status_t st = agc_lsposition(ip, ctx->sv.tet, &ctx->rpqv, &sun);
        if (st != AGC_STATUS_OK) {
            return st;
        }
    }
    const agc_dp_t d = agc_vec_abval(ip, agc_vec_sub(ip, ctx->sv.rcv, ctx->rpqv));
    *sw = (agc_dp_sub(ip, d, K_RSPHERE) < 0);                  /* INLUNCHK */
    (void)AGC_ASSERT(d >= 0);
    return AGC_STATUS_OK;
}

/*
 * TIMESTEP / RECTEST / CALLRECT / INTGRATE (p. 1344-1345) then the state
 * step through ENDSTATE.  With MIDFLAG the sphere-of-influence test
 * (CHKSWTCH, CM; R.V test, LM) may change origin (DOSWITCH) instead of
 * testing for rectification.
 */
agc_integ_next_t agc_integ_timestep(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (!AGC_ASSERT(ip != NULL) ||
        !AGC_ASSERT(agc_integ_valid_ctx(ctx))) {
        return INTEG_NEXT_FAIL;
    }
    bool sw = false;
    agc_status_t st = AGC_STATUS_OK;
    if (agc_flag_test(AGC_FLAG_MIDFLAG)) {
        if (agc_state_vehicle() == AGC_VEHICLE_CSM) {
            agc_dp_t test = 0;
            st = agc_integ_chkswtch(ip, ctx, &test);
            sw = (test < 0);                                   /* BMN      */
        } else {
            st = lm_sphere(ip, ctx, &sw);
        }
    }
    if (st == AGC_STATUS_OK) {
        st = sw ? agc_integ_origchng(ip, ctx) : rectest(ip, ctx);
    }
    if (st != AGC_STATUS_OK) {
        return INTEG_NEXT_FAIL;
    }
    ctx->zv = ctx->sv.nuv;                                     /* INTGRATE */
    ctx->yv = ctx->sv.deltav;
    agc_integ_flag(AGC_FLAG_JSWITCH, false);
    return state_step(ip, ctx);
}

/*
 * CHKSWTCH / RPQOK / EARSPH (CM p. 1345): *test >= 0 means stay.
 * Moon primary: RSPHERE - |R| (R = RCV + TDELTAV); earth primary:
 * |R - RPQ| - RSPHERE.  RPQV from LUNPOS when RPQFLAG is set.
 */
agc_status_t agc_integ_chkswtch(agc_interp_t *ip, agc_integ_ctx_t *ctx,
                                agc_dp_t *test)
{
    if (!AGC_ASSERT(ip != NULL) ||
        !AGC_ASSERT(agc_integ_valid_ctx(ctx)) ||
        !AGC_ASSERT(test != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const bool moon = agc_flag_test(AGC_FLAG_MOONFLAG);
    if (agc_flag_test(AGC_FLAG_RPQFLAG)) {
        agc_vec_t m;
        const agc_status_t st = lunpos(ip, ctx->sv.tet, &m);
        if (st != AGC_STATUS_OK) {
            return st;
        }
        ctx->rpqv = moon ? agc_vec_neg(m) : m;
    }
    const int p = ctx->pbody;
    const agc_vec_t r = agc_vec_add(ip, agc_vec_shr(ctx->sv.deltav, 7 + p),
                                    ctx->sv.rcv);
    if (moon) {
        *test = agc_dp_sub(ip, K_RSPHERE, agc_dp_shr(agc_vec_abval(ip, r), 2));
    } else {
        *test = agc_dp_sub(ip, agc_vec_abval(ip, agc_vec_sub(ip, r, ctx->rpqv)),
                           K_RSPHERE);
    }
    return AGC_STATUS_OK;
}

/*
 * ORIGCHNG (p. 1346): rectify, then move the origin to the other body:
 *   RRECT = RCV = (RCV SR PBODY - RPQV) SL (2 - PBODY)
 *   VRECT = VCV = (VCV SR PBODY - VMOON) SL (2 - PBODY), VMOON negated
 * when the moon was primary; MOONFLAG and PBODY toggle.
 */
agc_status_t agc_integ_origchng(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_status_t st = agc_integ_rectify(ip, ctx);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    const int p = ctx->pbody;
    const bool moon = agc_flag_test(AGC_FLAG_MOONFLAG);
    agc_vec_t r = agc_vec_sub(ip, agc_vec_shr(ctx->sv.rcv, p), ctx->rpqv);
    r = agc_integ_vshift(ip, r, 2 - p);                       /* VSL* 2,2 */
    ctx->sv.rrect = r;
    ctx->sv.rcv = r;
    agc_vec_t vm;
    st = lunvel(ip, ctx->sv.tet, &vm);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    if (moon) {
        vm = agc_vec_neg(vm);
    }
    agc_vec_t v = agc_vec_sub(ip, agc_vec_shr(ctx->sv.vcv, p), vm);
    v = agc_integ_vshift(ip, v, 2 - p);                       /* VSL* 0 +2,2 */
    ctx->sv.vrect = v;
    ctx->sv.vcv = v;
    return agc_integ_setmoon(ctx, !moon);                     /* CLR/SETMOON */
}
