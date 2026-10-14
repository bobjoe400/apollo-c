/**********************************************************************
 * FILE NAME: orbital_integration.c
 *
 * PURPOSE:
 *   ORBITAL INTEGRATION: Encke integration of the deviation from a conic
 *   (TIMESTEP, RECTEST, INTGRATE, ACCOMP, GAMCOMP, OBLATE, QUALITY1-3,
 *   DIFEQ, FBR3, ENDSTATE, GOBAQUE), the W-matrix columns (AMOVED,
 *   NEXTCOL, DOW..), RECTIFY / MINIRECT, KEPPREP, CHKSWTCH / ORIGCHNG.
 *
 * AGC SOURCE:
 *   Comanche055 ORBITAL_INTEGRATION.agc pp. 1334-1354; the CM QUALITY1 /
 *   QUALITY2 in RT8_OP_CODES.agc p. 1516
 *   Luminary099 ORBITAL_INTEGRATION.agc pp. 1227-1248 (differs: the
 *   TIMESTEP origin test, VECTAB/RQVV for MODREG 27, SETITCTR; the
 *   constant pool in CONTROLLED_CONSTANTS.agc, same values)
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale         Description
 *   ---------  ---------  ---  ------------  ----------------------------
 *   MOONFLAG   flag       I/O  -             moon is the primary body
 *   MIDFLAG    flag       I    -             third bodies (outside RME)
 *   RPQFLAG    flag       I/O  -             RPQV must be recomputed
 *   DIM0FLAG   flag       I/O  -             integrate W as well
 *   JSWITCH    flag       I/O  -             W integration in progress
 *   TEPHEM     agc_tp_t   I    2^42 cs       ephemeris epoch (LUNPOS)
 *   MODREG     agc_sp_t   I    -             LM: P23-type VECTAB use
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  -----------------------------------------
 *   agc_conic_kepprep      KEPPREP series (nav/conic.h)
 *   agc_conic_kepler       KEPLER (nav/conic.h)
 *   agc_lsposition         LSPOS (nav/auxiliary.h)
 *   agc_planet_r_to_rp     R-TO-RP / RP-TO-R / MOONMX (nav/auxiliary.h)
 *   agc_mpac_*             interpretive instructions on words
 *                          (core/interp_words.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   POODOO 00430 (INT-ABRT) through the driver when GOBAQUE finds no
 *   deviation left; ALARM 00421 (WMATEND) on W overflow; a called
 *   routine's failure is kept in ctx->fail (INTEG_NEXT_FAIL).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the job that holds INTSTALL; both vehicles.  Index
 *   registers: X2 = PBODY = 0 (earth) / 2 (moon) selects the scale;
 *   "VSL* 0 -k,2" is a right shift by k + PBODY.  NORM leaves -n in an
 *   index register; here n is kept positive and the AGC index sums are
 *   written out with their signs.
 *
 * NOTES:
 *   GAMCOMP, COMTERM, QUALITY1-3 and the OBLATE tail run on AGC words
 *   (agc_mpac_*, agc_vec_words_t): DSU/BDSU/DAD results whose words disagree
 *   in sign feed DMP, DMPR, VXSC and VSR exactly as the AGC holds them
 *   (review finding N8).
 *
 * ALGORITHM:
 *   One time step: TIMESTEP (origin / rectification test); INTGRATE
 *   YV = TDELTAV, ZV = TNUV; three Nystrom stages at H = 0, DT/2, DT:
 *   ACCOMP computes the disturbing acceleration FV, DIFEQ+0/+1/+2
 *   accumulate, DIFEQCOM predicts the deviation, FBR3 advances the conic
 *   by DT/2 (KEPLER); ENDSTATE stores TDELTAV, TNUV.  The W columns are
 *   then integrated over the same step with DOW.. as the derivative.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/flags_common.h"
#include "agc/core/interp_words.h"
#include "agc/nav/integration.h"
#include "internal.h"

/* ---- constant pool (CM pp. 1353-1354) --------------------------------- */
/* 3/4   2DEC 3.0 B-2, oct 30000 00000: 3 at scale 2^2 (also the rectify
 *       limit .75 for |TDELTAV| and |TNUV| in their own scales).         */
static const agc_dp_t K_3_4 = 201326592; /* 3/4 at 2^2 */

/* THREE/8 2DEC .375, oct 14000 00000.                                    */
static const agc_dp_t K_THREE_8 = 100663296; /* .375 at 2^0 */

/* DQUARTER = DP1/4 2DEC .25, oct 10000 00000.                            */
static const agc_dp_t K_DQUARTER = 67108864; /* .25 at 2^0 */

/* HALFDP .5 (XUNIT), oct 20000 00000.                                    */
static const agc_dp_t K_HALFDP = 134217728; /* .5 at 2^0 */

/* 3/32 2DEC 3 B-5, oct 03000 00000.                                      */
static const agc_dp_t K_3_32 = 25165824; /* 3/32 at 2^0 */

/* 15/16 2DEC 15. B-4, oct 36000 00000.                                   */
static const agc_dp_t K_15_16 = 251658240; /* 15/16 at 2^0 */

/* 3/64 2DEC 3 B-6, oct 01400 00000.                                      */
static const agc_dp_t K_3_64 = 12582912; /* 3/64 at 2^0 */

/* 7/12 2DEC .5833333333, oct 22525 12525.                                */
static const agc_dp_t K_7_12 = 156587349; /* 7/12 at 2^0 */

/* 2/3 = DP2/3 2DEC .6666666667, oct 25252 25253.                         */
static const agc_dp_t K_2_3 = 178956971; /* 2/3 at 2^0 */

/* 9/16 2DEC 9 B-4, oct 22000 00000.                                      */
static const agc_dp_t K_9_16 = 150994944; /* 9/16 at 2^0 */

/* 5/128 2DEC 5 B-7, oct 01200 00000.                                     */
static const agc_dp_t K_5_128 = 10485760; /* 5/128 at 2^0 */

/* 5/8 2DEC 5 B-3, oct 24000 00000.                                       */
static const agc_dp_t K_5_8 = 167772160; /* 5/8 at 2^0 */

/* D1/32 2DEC 1.0 B-5, oct 01000 00000 (CONIC_SUBROUTINES pool).          */
static const agc_dp_t K_D1_32 = 8388608; /* 1/32 at 2^0 */

/* RECRATIO 2DEC .01, oct 00243 32703: rectify when |dR| >= .01 |R|.      */
static const agc_dp_t K_RECRATIO = 2684355; /* .01 at 2^0 */

/* RSPHERE 2DEC 64373.76 E3 B-29, oct 03654 21000: lunar sphere of
 * influence radius 64373.76 km, scale 2^29 m.                            */
static const agc_dp_t K_RSPHERE = 32186880; /* 64373.76 km, m 2^29 */

/* MU table addressed MUEARTH - X2 (2DEC*): X2 = 0 earth, 2 moon, 4 sun.
 *   MUEARTH 3.986032 E10 B-36  m^3/cs^2  oct 22437 16067
 *   MUM     4.9027780 E8 B-30            oct 16471 01352
 *   (sun)   1.32715445 E16 B-54          oct 27446 14620               */
static const agc_dp_t MU_TAB[3] = {155704375, 122569450, 197761424};

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
static const agc_dp_t RD_TAB[2]      = {40233600, 32186880};
static const agc_dp_t J4REQ_J3[2]    = {19966430, 0};
static const agc_dp_t J3RE_J2_2[2]   = {-27109, -1409888};
static const agc_dp_t J2REQSQ_TAB[2] = {99760848, 71420644};

/* ASCALE,1 (p. 1353 "IT IS VITAL THAT THE FOLLOWING CONSTANTS NOT BE
 * SHUFFLED"): DLOAD* ASCALE,1 loads the two SP words at ASCALE - X1 into
 * S1, S2.  X1 = 0/2 primary (earth/moon), 4/6 secondary body, 8/10 sun.  */
#define ASCALE_N 6 /* ASCALE pairs: primary, secondary, sun x (earth, moon) */
static const int ASCALE_S1[ASCALE_N] = {-7, -9, 0, -2, -9, -11};
static const int ASCALE_S2[ASCALE_N] = {-6, -4, -12, -2, -6, -2};

/* Lunar potential pad loads (SP): see agc_integ_set_lunar_coeffs().      */
static agc_sp_t e3j22r2m; /* E3J22R2M, SP pad load, 2^58 */
static agc_sp_t e32c31rm; /* E32C31RM, SP pad load, 2^80 */

#define TP_HALF ((agc_tp_t) 0x20000000000LL) /* 2^41: TP .5             */

/**********************************************************************
 * E3J22R2M / E32C31RM (ERASABLE_ASSIGNMENTS.agc, pad loads) -- set the lunar
 * potential coefficients QUALITY2/3 use
 *
 * FUNCTION NAME: agc_integ_set_lunar_coeffs
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   j22       agc_sp_t    I    E3J22R2M, the J22 term coefficient (SP, B61 x
 *                              R^4 product)
 *   c31       agc_sp_t    I    E32C31RM, the C31 term coefficient (SP)
 *********************************************************************/
void agc_integ_set_lunar_coeffs(agc_sp_t j22, agc_sp_t c31)
{
    e3j22r2m = j22;
    e32c31rm = c31;
}

/**********************************************************************
 * E3J22R2M / E32C31RM (ERASABLE_ASSIGNMENTS.agc, pad loads) -- read the lunar
 * potential coefficients back (erasable access by address, src/core/ecadr.c)
 *
 * FUNCTION NAME: agc_integ_get_lunar_coeffs
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   j22       agc_sp_t *  O    E3J22R2M (SP)
 *   c31       agc_sp_t *  O    E32C31RM (SP)
 *********************************************************************/
void agc_integ_get_lunar_coeffs(agc_sp_t *j22, agc_sp_t *c31)
{
    *j22 = e3j22r2m;
    *c31 = e32c31rm;
}

/**********************************************************************
 * MUEARTH,2 (ORBITAL_INTEGRATION.agc p. 1353) -- the gravitational constant
 * indexed by X2
 *
 * FUNCTION NAME: agc_integ_mu
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------------
 *   x2        int         I    X2: 0 earth, 2 moon, 4 sun
 *
 * RETURNS: agc_dp_t -- agc_dp_t MU: earth 2^36, moon 2^30, sun 2^54 m^3/cs^2
 *********************************************************************/
agc_dp_t agc_integ_mu(int x2)
{
    return (MU_TAB[x2 / 2]);
}

/**********************************************************************
 * NORM (TSLC, INTERPRETER.agc p. 1157) -- NORM on the triple-precision MPAC:
 * shift left until |v| >= 1/2.  Used after DMP, which leaves a TP product.
 *
 * FUNCTION NAME: tp_norm
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  I/O  interpretive state (OVFIND)
 *   v         agc_tp_t        I    TP MPAC, any scale
 *   n         int *           O    left shifts done (the AGC stores the
 *                                  complement)
 *
 * RETURNS: agc_tp_t -- the normalised TP MPAC (scale - n)
 *********************************************************************/
static agc_tp_t tp_norm(agc_interp_t *ip, agc_tp_t v, int *n)
{
    agc_tp_t m = v; /* MPAC being normalised, TP at 2^N */
    int      k = 0; /* left shifts done */

    if (m != 0)
    {
        for (k = 0; k < AGC_TP_BITS; k++)
        {
            if ((m >= TP_HALF) || (m <= -TP_HALF))
            {
                break;
            }

            m = agc_tp_add(ip, m, m);
        }
    }

    *n = k;

    return (m);
}

/**********************************************************************
 * SETMOON / CLRMOON (INTEGRATION_INITIALIZATION.agc p. 1315) -- MOONFLAG and
 * PBODY together
 *
 * FUNCTION NAME: agc_integ_setmoon
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (PBODY)
 *   moon      bool               I    true: moon primary (PBODY 2)
 *********************************************************************/
void agc_integ_setmoon(agc_integ_ctx_t *ctx, bool moon)
{
    agc_flag_put(AGC_FLAG_MOONFLAG, moon);
    ctx->pbody = moon ? INTEG_PBODY_MOON : INTEG_PBODY_EARTH; /* PBODY 2 or 0 */
}

/**********************************************************************
 * RECTIFY (ORBITAL_INTEGRATION.agc p. 1347) -- establish a new conic through
 * the actual state:
 *   RRECT = RCV = TDELTAV SR(7+PBODY) + RCV
 *   VRECT = VCV = TNUV SR(4+PBODY) + VCV, then MINIRECT.
 *
 * FUNCTION NAME: agc_integ_rectify
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the state
 *                                     vector)
 *********************************************************************/
void agc_integ_rectify(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const int p = ctx->pbody; /* PBODY: 0 earth, 2 moon */

    ctx->sv.rrect     = agc_vec_add(ip, agc_vec_shr(ctx->sv.deltav, 7 + p), ctx->sv.rcv);
    ctx->sv.rcv       = ctx->sv.rrect;
    const agc_vec_t v = agc_vec_add(ip, agc_vec_shr(ctx->sv.nuv, 4 + p), ctx->sv.vcv); /* VRECT, m/cs 2^7 (moon 2^5) */

    agc_integ_minirect(ctx, v);
}

/**********************************************************************
 * MINIRECT (ORBITAL_INTEGRATION.agc p. 1347) -- VRECT = VCV = v, zero TDELTAV
 * TNUV TC XKEP
 *
 * FUNCTION NAME: agc_integ_minirect
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the state
 *                                     vector)
 *   v         agc_vec_t          I    the new VRECT, m/cs 2^7 (moon 2^5)
 *********************************************************************/
void agc_integ_minirect(agc_integ_ctx_t *ctx, agc_vec_t v)
{
    ctx->sv.vrect  = v;
    ctx->sv.vcv    = v;
    ctx->sv.deltav = agc_vec_zero();
    ctx->sv.nuv    = agc_vec_zero();
    ctx->sv.tc     = 0;
    ctx->sv.xkep   = 0;
}

/**********************************************************************
 * KEPPREP (ORBITAL_INTEGRATION.agc pp. 1334-1335) and the KEPLERN call it ends
 * in -- first guess of x from the current conic point (RCV, VCV at TC) by
 * series in TAU.-TC, then KEPLER from RRECT, VRECT to TAU.  Results RCV VCV TC
 * XKEP.
 * The arithmetic is agent nav-conic's (agc_conic_kepprep/kepler); this wrapper
 * owns the erasables.  KEPPREP reads RCV and VCV, so they are passed in the
 * rrect/vrect fields of the KEPPREP call.  ITERCTR running out counts as
 * converged (BHIZ); on TIMEOVFL RCV, VCV keep their contents.
 *
 * FUNCTION NAME: agc_integ_kepprep
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (TAU., the conic)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of KEPPREP /
 *          KEPLER other than NO_CONVERGE and OVERFLOW
 *********************************************************************/
agc_status_t agc_integ_kepprep(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_kepler_in_t in; /* KEPPREP / KEPLER inputs */

    in.rrect      = ctx->sv.rcv;
    in.vrect      = ctx->sv.vcv;
    in.tau        = ctx->tau;
    in.xkepnew    = 0;
    in.tc         = ctx->sv.tc;
    in.xprev      = ctx->sv.xkep;
    in.body       = agc_flag_test(AGC_FLAG_MOONFLAG) ? AGC_CONIC_MOON : AGC_CONIC_EARTH; /* MOONFLAG: moon or earth conic */
    agc_dp_t xnew = 0;                                                                   /* XKEPNEW, sqrt(m) 2^17 (moon 2^16) */
    agc_conic_kepprep(ip, &in, &xnew);

    in.rrect   = ctx->sv.rrect; /* KEPLERN works from RRECT  */
    in.vrect   = ctx->sv.vrect;
    in.xkepnew = xnew;
    agc_kepler_out_t kout;      /* KEPLER outputs */

    agc_status_t st = agc_conic_kepler(ip, &in, &kout); /* KEPLER status */

    if ((st == AGC_STATUS_OK) || (st == AGC_STATUS_NO_CONVERGE))
    {
        ctx->sv.rcv = kout.rcv; /* ITERCTR exhausted counts  */
        ctx->sv.vcv = kout.vcv; /* as converged (BHIZ)       */
    }
    else if (st != AGC_STATUS_OVERFLOW)
    {
        return (st);
    }
    else
    {
        /* TIMEOVFL: RCV, VCV keep their previous contents.               */
    }

    ctx->sv.tc   = kout.tc;
    ctx->sv.xkep = kout.xprev; /* XPREV EQUALS XKEP         */

    return (AGC_STATUS_OK);
} /* agc_integ_kepprep */

/**********************************************************************
 * FBR3 (ORBITAL_INTEGRATION.agc p. 1336) -- advance the conic by DT/2.  TIX,1
 * skips the ROUND on the mid-step entry (DIFEQCNT -12), so DT/2 SR 9 is
 * truncated there and rounded at the end of the step (DIFEQCNT -24).  DT/2
 * 2^19 -> 2^28 cs.
 *
 * FUNCTION NAME: advance_conic_half_step
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (stage 1 or 2,
 *                                     TAU., TET, the conic)
 *
 * RETURNS: agc_status_t -- the status of KEPPREP
 *********************************************************************/
static agc_status_t advance_conic_half_step(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const agc_dp_t d = (ctx->stage == 1) ? agc_dp_shr(ctx->dt2, 9) : agc_dp_shr_round(ctx->dt2, 9); /* DT/2 at 2^28 cs: SR 9 (mid step) or SR 9 ROUND (end) */

    ctx->tau    = agc_dp_add(ip, d, ctx->sv.tc);  /* TAU. = TC + DT/2 */
    ctx->sv.tet = agc_dp_add(ip, d, ctx->sv.tet); /* TET += DT/2      */

    return (agc_integ_kepprep(ip, ctx));
}

/**********************************************************************
 * GOBAQUE / INT-ABRT (ORBITAL_INTEGRATION.agc p. 1340) -- on overflow back the
 * conic up by H to the start of the step, rectify and try again; with no
 * deviation left the integration is hopeless: POODOO 00430.
 *
 * FUNCTION NAME: gobaque
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_TESTLOOP, INTEG_NEXT_ABORT
 *                              (INT-ABRT), or INTEG_NEXT_FAIL with ctx->fail
 *********************************************************************/
static agc_integ_next_t gobaque(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (agc_vec_abval(ip, ctx->sv.deltav) == 0)
    {
        return (INTEG_NEXT_ABORT); /* POODOO INT-ABRT, raised by the driver */
    }

    const agc_dp_t h28 = agc_dp_shr(ctx->h, 9); /* H 2^19 -> 2^28   */

    ctx->tau    = agc_dp_sub(ip, ctx->sv.tc, h28);
    ctx->sv.tet = agc_dp_sub(ip, ctx->sv.tet, h28);

    agc_status_t st = agc_integ_kepprep(ip, ctx); /* status of KEPPREP */

    if (st != AGC_STATUS_OK)
    {
        return (agc_integ_fail(ctx, st));
    }

    agc_integ_rectify(ip, ctx);
    agc_flag_put(AGC_FLAG_RPQFLAG, true);

    return (INTEG_NEXT_TESTLOOP);
}

/* ======================================================================
 * Disturbing accelerations: ACCOMP, GAMCOMP, OBLATE, QUALITY1-3
 * ==================================================================== */

typedef enum acc_result
{
    ACC_OK = 0, /* FV complete, go to NBRANCH                          */
    ACC_OVF,    /* overflow: GOBAQUE                                   */
    ACC_FAIL    /* a called routine failed: ctx->fail                 */
} acc_result_t;

/**********************************************************************
 * AGC: none (C only) -- a routine ACCOMP called failed: keep its status
 *
 * FUNCTION NAME: acc_fail
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (fail)
 *   st        agc_status_t       I    the status of the routine that failed
 *
 * RETURNS: acc_result_t -- ACC_FAIL
 *********************************************************************/
static acc_result_t acc_fail(agc_integ_ctx_t *ctx, agc_status_t st)
{
    ctx->fail = st;

    return (ACC_FAIL);
}

/**********************************************************************
 * VSQ (VSQSUB, INTERPRETER.agc p. 1139) -- VSQ keeping the triple-precision
 * sum MPAC..MPAC+2
 *
 * FUNCTION NAME: vsq_tp
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  I/O  interpretive state (OVFIND)
 *   v         agc_vec_t       I    vector, any scale 2^N
 *
 * RETURNS: agc_tp_t -- |v|^2 as the TP MPAC, scale 2^2N
 *********************************************************************/
static agc_tp_t vsq_tp(agc_interp_t *ip, agc_vec_t v)
{
    agc_tp_t s = agc_dp_mul_tp(v.c[0], v.c[0]); /* TP sum of squares, scale 2^2N */

    for (int i = 1; i < AGC_VEC_N; i++)
    {
        s = agc_tp_add(ip, s, agc_dp_mul_tp(v.c[i], v.c[i]));
    }

    return (s);
}

/* GAMCOMP push-list work cells, as the words stored there.             */
typedef struct gam_work
{
    agc_dp_words_t bsq;   /* 0D  |BETAV/2|^2 normalised by n1              */
    agc_dp_words_t rho_q; /* 2D  (ALPHAM_n2 / 2) / BETAM_n3, SR1R          */
    agc_dp_words_t rho;   /* 4D  rho = ALPHAM / BETAM at the ASCALE scale   */
    agc_dp_words_t rho4;  /* 6D  rho / 4 (SR1R), then q                     */
    int            n1;    /* -C(31D)                                       */
    int            xs;    /* X2 = S1 - n2 + n3 (ASCALE word 1 + norm counts) */
} gam_work_t;

/**********************************************************************
 * GAMCOMP second half (ORBITAL_INTEGRATION.agc pp. 1339-1340) -- q = rho/4 - 2
 * ALPHAV.BETAV, Battin's f(q) series and -MU [f(q) B + A] / |B|^3 before its
 * shift to the FV scale.  Word level: every DSU/DAD result feeds the next
 * DMPR, DDV or VXSC as the AGC holds it.
 *
 * FUNCTION NAME: gamcomp_term
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------------
 *   ip        agc_interp_t *           I/O  interpretive state (OVFIND)
 *   ctx       const agc_integ_ctx_t *  I    integration erasables (ALPHAV,
 *                                           BETAV unit vectors, 2^1)
 *   g         const gam_work_t *       I    GAMCOMP push-list cells (0D, 2D,
 *                                           4D, 6D)
 *   x2        int                      I    MU index: 0 earth, 2 moon, 4 sun
 *   n4        int *                    O    NORM count of |B|^2 f-denominator
 *                                           (30D)
 *
 * RETURNS: vec_words_t -- the acceleration term as word pairs, at the
 *                         push-list scale
 *********************************************************************/
static agc_vec_words_t gamcomp_term(agc_interp_t *ip, const agc_integ_ctx_t *ctx, const gam_work_t *g, int x2, int *n4)
{
    const agc_dp_words_t dq = agc_dp_to_words(K_DQUARTER);                                            /* DQUARTER words, .25 */
    agc_tp_words_t       m  = agc_mpac_shift_round(ip, agc_mpac_dot(ip, ctx->alphav, ctx->betav), 1); /* DOT SL1R */

    m                         = agc_mpac_bdsu(ip, m, g->rho4);                                                     /* BDSU 6D: q   */
    const agc_dp_words_t q    = agc_mpac_store(m);                                                                 /* PUSH 6D      */
    const agc_dp_words_t pd8  = agc_mpac_store(agc_mpac_shift(ip, agc_mpac_dmpr(ip, m, g->rho), 1));               /* DMPR 4 SL1 PUSH */
    const agc_dp_words_t pd10 = agc_mpac_store(agc_mpac_dad(ip, agc_mpac_dload(pd8), dq));                         /* DAD PUSH 10D */
    const agc_dp_words_t pd12 = agc_mpac_store(agc_mpac_dmpr(ip, agc_mpac_sqrt(agc_mpac_dload(pd10)), pd10));      /* SQRT DMPR PUSH */
    const agc_dp_words_t pd14 = agc_mpac_store(agc_mpac_dad(ip, agc_mpac_shift(ip, agc_mpac_dload(pd12), 1), dq)); /* SL1 DAD PDDL */

    m                        = agc_mpac_dad(ip, agc_mpac_dload(pd10), agc_dp_to_words(K_HALFDP));        /* DLOAD 10D DAD HALFDP */
    m                        = agc_mpac_shift(ip, agc_mpac_dmpr(ip, m, pd8), 1);                         /* DMPR 8D SL1          */
    m                        = agc_mpac_ddv(ip, agc_mpac_dad(ip, m, agc_dp_to_words(K_THREE_8)), pd14);  /* DAD THREE/8 DDV 14D  */
    m                        = agc_mpac_dmpr(ip, m, q);                                                  /* G/2: DMPR 6          */
    const agc_vec_words_t a8 = agc_mpac_vshift(ip,                                                       /* PDVL VSR3 ALPHAV */
                                               agc_vec_to_words(ctx->alphav),
                                               -3);
    const agc_vec_words_t v  = agc_mpac_vad(ip, a8, agc_mpac_vxsc(ip, m, agc_vec_to_words(ctx->betav))); /* VXSC BETAV, VAD 16D  */

    m = agc_mpac_round(ip, agc_mpac_norm(agc_mpac_dmp(agc_mpac_dload(g->bsq), pd12), n4)); /* DLOAD 0 DMP 12D NORM ROUND */
    m = agc_mpac_dmp(agc_mpac_bddv(ip, m, g->rho_q), agc_dp_to_words(agc_integ_mu(x2)));   /* BDDV 2 DMP* MUEARTH,2     */

    return (agc_mpac_vxsc(ip, agc_mpac_dcomp(m), v)); /* DCOMP VXSC */
}

/**********************************************************************
 * GAMCOMP (ORBITAL_INTEGRATION.agc pp. 1338-1340) -- add to FV the
 * acceleration difference -MU [f(q) B + A] / |B|^3 for a body at BETAV
 * (Battin's f(q) form; A = ALPHAV unit vector x ALPHAM).
 *
 * FUNCTION NAME: gamcomp
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (ALPHAV, ALPHAM,
 *                                     BETAV, BETAM, FV)
 *   x1        int                I    ASCALE index: 0/2 primary, 4/6
 *                                     secondary, 8/10 sun
 *   x2        int                I    MU index: 0 earth, 2 moon, 4 sun
 *
 * RETURNS: bool -- true when the final VAD overflows (BOV GOBAQUE)
 *********************************************************************/
static bool gamcomp(agc_interp_t *ip, agc_integ_ctx_t *ctx, int x1, int x2)
{
    gam_work_t g; /* GAMCOMP push-list cells */

    /* GAMCOMP first half (p. 1338-1339): normalised B^2, BETAV -> unit,
     * BETAM = |BETAV|, rho = ALPHAM / BETAM.                                */
    int      n2 = 0;                                                          /* NORM count of ALPHAM (32D) */
    int      n3 = 0;                                                          /* NORM count of BETAM (33D) */
    agc_tp_t b2 = tp_norm(ip, vsq_tp(ip, agc_vec_shr(ctx->betav, 1)), &g.n1); /* |BETAV/2|^2 normalised: TP 2^0 */

    g.bsq                   = agc_dp_to_words(agc_tp_round_dp(b2));                             /* NORM ROUND 31D */
    agc_tp_words_t       m  = agc_mpac_norm(agc_mpac_dload(agc_dp_to_words(ctx->alpham)), &n2); /* NORM 32D       */
    const agc_dp_words_t am = agc_mpac_store(agc_mpac_shift(ip, m, -1));                        /* SR1 PDVL       */
    agc_dp_t             bm = 0;                                                                /* BETAM = |BETAV|, scale of BETAV */

    ctx->betav = agc_vec_unit(ip, ctx->betav, &bm, NULL);                 /* UNIT, 36D: BETAM */
    ctx->betam = bm;
    m          = agc_mpac_norm(agc_mpac_dload(agc_dp_to_words(bm)), &n3); /* NORM 33D  */
    m          = agc_mpac_shift_round(ip, agc_mpac_bddv(ip, m, am), -1);  /* BDDV SR1R */
    g.rho_q    = agc_mpac_store(m);                                       /* PUSH 2D   */
    g.xs       = ASCALE_S1[x1 / 2] - n2 + n3;                             /* XCHX,2 S1 XAD,2 32D XSU,2 33D */

    /* SR* 0 -1,2 leaves the bits shifted out in MPAC+2; PUSH stores RHO
     * (4D) without them and SR1R rounds them in (6D).                       */
    m                          = agc_mpac_shift(ip, agc_mpac_dload(g.rho_q), 1 + g.xs); /* SR* 0 -1,2 */
    g.rho                      = agc_mpac_store(m);                                     /* PUSH 4D    */
    g.rho4                     = agc_mpac_store(agc_mpac_shift_round(ip, m, -1));       /* SR1R 6D    */
    int                   n4   = 0;                                                     /* NORM count of the f denominator (30D) */
    const agc_vec_words_t v    = gamcomp_term(ip, ctx, &g, x2, &n4);                    /* acceleration term words */
    const int             xsum = g.xs + ASCALE_S2[x1 / 2] + n4 + g.n1;                  /* XAD XSU XSU */

    agc_interp_clear_ovf(ip); /* BOV +1 (reset)   */
    ctx->fv = agc_vec_add(ip, /* VSR* 0 -1,2 VAD FV */
                          agc_vec_from_words(agc_mpac_vshift(ip, v, 1 + xsum)),
                          ctx->fv);

    return (agc_interp_test_ovf(ip)); /* BOV -> GOBAQUE   */
}

/**********************************************************************
 * LUNPOS (LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc C p. 786; LM =
 * LSPOS) -- the moon relative to the earth
 *
 * FUNCTION NAME: lunpos
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  I/O  interpretive state (OVFIND)
 *   t         agc_dp_t        I    time, cs 2^28
 *   moon      agc_vec_t *     O    earth-centred moon position, m 2^29
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the ephemeris / LSPOS status
 *********************************************************************/
static agc_status_t lunpos(agc_interp_t *ip, agc_dp_t t, agc_vec_t *moon)
{
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_vec_t sun; /* sun position (LSPOS, unused here), m 2^38 */

        return (agc_lsposition(ip, t, moon, &sun));
    }

    agc_ephem_csm_t eph; /* CM ephemeris pad load */
    agc_ephem_get_csm_data(&eph);

    return (agc_ephem_csm_lunpos(ip, &eph, agc_erase_tephem(), t, moon));
}

/**********************************************************************
 * LUNVEL (LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc C p. 787; LM: a
 * bare RVQ "TO FOOL INTEGRATION") -- the moon's velocity relative to the
 * earth; zero for the LM, which never reaches it
 *
 * FUNCTION NAME: lunvel
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  I/O  interpretive state (OVFIND)
 *   t         agc_dp_t        I    time, cs 2^28
 *   vmoon     agc_vec_t *     O    moon velocity, m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the ephemeris status
 *********************************************************************/
static agc_status_t lunvel(agc_interp_t *ip, agc_dp_t t, agc_vec_t *vmoon)
{
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        *vmoon = agc_vec_zero(); /* DEVIATION: MPAC left as is in AGC */

        return (AGC_STATUS_OK);
    }

    agc_ephem_csm_t eph; /* CM ephemeris pad load */
    agc_ephem_get_csm_data(&eph);

    return (agc_ephem_csm_lunvel(ip, &eph, agc_erase_tephem(), t, vmoon));
}

/**********************************************************************
 * ACCOMP third-body part (ORBITAL_INTEGRATION.agc pp. 1337-1338, MIDFLAG set)
 * -- LSPOS, RPQV, RPSV, VECTAB +6 (and LM RQVV), GETRPSV, GAMCOMP for the
 * secondary body and for the sun
 *
 * FUNCTION NAME: third_body
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (RPQV, RPSV,
 *                                     VECTAB, FV)
 *
 * RETURNS: acc_result_t -- ACC_OK, ACC_OVF (GOBAQUE) or ACC_FAIL with
 *                          ctx->fail
 *********************************************************************/
static acc_result_t third_body(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_vec_t moon;           /* moon position from LSPOS, m 2^29 */
    agc_vec_t sun;            /* sun position from LSPOS, m 2^38 */
    const int p = ctx->pbody; /* PBODY: 0 earth, 2 moon */

    const agc_status_t st = agc_lsposition(ip, ctx->sv.tet, &moon, &sun); /* status of LSPOS */

    if (st != AGC_STATUS_OK)
    {
        return (acc_fail(ctx, st));
    }

    int x2 = 2; /* secondary = moon       */

    if (agc_flag_test(AGC_FLAG_MOONFLAG))
    {
        moon = agc_vec_neg(moon); /* moon -> earth          */
        x2   = 0;                 /* secondary = earth      */
    }

    ctx->betav                                  = moon;
    ctx->rpqv                                   = moon;
    ctx->rpsv                                   = sun;
    const bool lm                               = (agc_state_vehicle() == AGC_VEHICLE_LM); /* LM: RQVV kept as well */
    const bool is_cislunar_midcourse_navigation = lm && (agc_erase_modreg() == 23);        /* MODREG OCT27  */

    if (is_cislunar_midcourse_navigation || agc_flag_test(AGC_FLAG_DIM0FLAG))
    {
        const agc_vec_t r  = agc_vec_scale(ip, ctx->alphav, ctx->alpham);             /* R = UNIT(R) |R|, m 2^29 (moon 2^27) */
        const agc_vec_t rq = agc_vec_sub(ip, agc_vec_shl(ip, r, x2 - 1), ctx->betav); /* VSR* 1,2 VSU; R - RPQ at 2^29 m */

        ctx->vectab[(2 * ctx->stage) + 1] = rq;

        if (lm)
        {
            ctx->rqvv = rq; /* LM: STORE RQVV         */
        }
    }

    agc_flag_put(AGC_FLAG_RPQFLAG, false); /* GETRPSV                */

    if (agc_flag_test(AGC_FLAG_MOONFLAG))
    {
        ctx->rpsv = agc_vec_add(ip, agc_vec_shr(ctx->rpqv, 9), ctx->rpsv);
    }

    if (gamcomp(ip, ctx, p + 4, x2))
    {
        return (ACC_OVF);
    }

    ctx->betav = ctx->rpsv;

    return (gamcomp(ip, ctx, p + 8, 4) ? ACC_OVF : ACC_OK); /* the sun's GAMCOMP overflowed: GOBAQUE */
} /* third_body */

/* OBLATE push-list cells shared with QUALITY1-3.                         */
typedef struct obl_work
{
    agc_vec_t urpv; /* 14D URPV unit R in moon-fixed axes, 2^1 (moon)     */
    agc_dp_t  cphi; /* 18D COSPHI/2: z of the unit R in planet axes, 2^1  */
    agc_vec_t uz;   /* 20D UZ planet polar axis, 2^1                       */
    agc_dp_t  r4n;  /* 0D  |R|^4 normalised, 2^0 after NORM */
    int       x1;   /* X1 = -(4 N + M): N, M the two NORM counts          */
} obl_work_t;

/* Legendre terms (COMTERM, p. 1341): P2/64, P3/32, P4/128, P5/1024 as
 * the words the DSU and BDSU leave (their signs may disagree, and the
 * DMP, DMPR and VXSC that use them see the words).                       */
typedef struct legendre
{
    agc_dp_words_t p2; /* 0D   */
    agc_dp_words_t p3; /* 2D   */
    agc_dp_words_t p4; /* 4D   */
    agc_dp_words_t p5; /* MPAC */
} legendre_t;

/**********************************************************************
 * OBLATE moon branch (ORBITAL_INTEGRATION.agc p. 1341) -- URPV =
 * R-TO-RP(ALPHAV), UZ = unit of MT (ZUNIT + 504LM x ZUNIT), the lunar pole in
 * reference axes.  The AGC reuses the MMATRIX left in the push list by
 * R-TO-RP; MOONMX is called again here for the same matrix (same TET).
 *
 * FUNCTION NAME: moon_axes
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------------
 *   ip        agc_interp_t *           I/O  interpretive state (OVFIND)
 *   ctx       const agc_integ_ctx_t *  I    integration erasables (ALPHAV,
 *                                           TET)
 *   w         obl_work_t *             O    OBLATE cells URPV, COSPHI/2, UZ
 *                                           (2^1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the pad-load / R-TO-RP / MOONMX
 *                          status
 *********************************************************************/
static agc_status_t moon_axes(agc_interp_t *ip, const agc_integ_ctx_t *ctx, obl_work_t *w)
{
    agc_navaux_padload_t pad;                                     /* lunar pad loads (504LM) */
    agc_mat_t            m;                                       /* MOONMX, 2^1 */
    const agc_vec_t      zunit = agc_vec_make(0, 0, AGC_DP_HALF); /* ZUNIT, half unit 2^1 */

    agc_navaux_get_padload(&pad);
    agc_status_t st = agc_planet_r_to_rp(ip, &pad, true, ctx->alphav, ctx->sv.tet, &w->urpv); /* status of R-TO-RP */

    if (st == AGC_STATUS_OK)
    {
        st = agc_planet_moonmx(ip, &pad, ctx->sv.tet, &m);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    w->cphi      = w->urpv.c[2];
    agc_vec_t ax = agc_vec_add(ip, agc_vec_cross(ip, pad.lm504, zunit), zunit); /* ZUNIT + 504LM x ZUNIT, 2^1 */

    w->uz = agc_vec_unit(ip, agc_vec_mat(ip, ax, &m), NULL, NULL);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * QUALITY3 (ORBITAL_INTEGRATION.agc pp. 1342-1343) -- the C31 term, added to
 * the J22 term, scaled by 1/R^4, rotated to reference axes (RP-TO-R) and added
 * to FV.  Word level: the DSU, BDSU and DAD results feed DMP and VXSC as held.
 *
 * FUNCTION NAME: lunar_c31_term
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------------
 *   ip        agc_interp_t *      I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *   I/O  integration erasables (ALPHAM, TET,
 *                                      FV)
 *   w         const obl_work_t *  I    OBLATE cells (URPV, COSPHI/2, R^4, X1)
 *   j22       vec_words_t         I    the J22 term x R^4 as words, B61
 *
 * RETURNS: acc_result_t -- ACC_OK, ACC_OVF (GOBAQUE) or ACC_FAIL with
 *                          ctx->fail
 *********************************************************************/
static acc_result_t lunar_c31_term(agc_interp_t *ip, agc_integ_ctx_t *ctx, const obl_work_t *w, agc_vec_words_t j22)
{
    const agc_vec_words_t u   = agc_vec_to_words(w->urpv);                              /* URPV words, 2^1 */
    const agc_dp_words_t  k58 = agc_dp_to_words(K_5_8);                                 /* 5/8 words */
    const agc_dp_words_t  one = agc_dp_to_words(K_D1_32);                               /* D1/32 words */
    agc_tp_words_t        m   = agc_mpac_dsq(agc_mpac_dload(agc_dp_to_words(w->cphi))); /* DSQ: COSPHI^2 B2 */
    const agc_dp_words_t  c2  = agc_mpac_store(m);                                      /* PUSH 8D          */
    const agc_dp_words_t  c5  = agc_mpac_store(agc_mpac_dmp(m, k58));                   /* DMP 5/8, PDDL 8D */

    m                 = agc_mpac_dad(ip, agc_mpac_shift(ip, agc_mpac_dload(c2), -2), c5); /* SR2 DAD 8D       */
    m                 = agc_mpac_bdsu(ip, m, one);                                        /* BDSU: 1-7c2, B5  */
    m                 = agc_mpac_dmp(agc_mpac_dmp(m, u.c[0]), k58);                       /* DMP DMP URPV 5/8 */
    agc_vec_words_t v = agc_mpac_vshift(ip, agc_mpac_vxsc(ip, m, u), 5);                  /* VXSC VSL5 URPV   */

    v.c[0]                   = agc_mpac_store(agc_mpac_dad(ip, agc_mpac_dsu(ip, agc_mpac_dload(c5), one), v.c[0])); /* DSU DAD 8D */
    m                        = agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dmp(agc_mpac_dload(u.c[0]), u.c[2]), k58), 1);
    v.c[2]                   = agc_mpac_store(agc_mpac_dad(ip, m, v.c[2]));                                         /* SL1 DAD 12D */
    int                  n   = 0;                                                                                   /* NORM count of ALPHAM (X2) */
    agc_tp_words_t       rn  = agc_mpac_norm(agc_mpac_dload(agc_dp_to_words(ctx->alpham)), &n);                     /* NORM X2, PUSH */
    const agc_dp_words_t c31 = {e32c31rm, 0};

    m                      = agc_mpac_ddv(ip, agc_mpac_dload(c31), agc_mpac_store(rn));  /* SLOAD E32C31RM DDV */
    v                      = agc_mpac_vshift(ip, agc_mpac_vxsc(ip, m, v), n - 3);        /* VXSC VSL* 0 -3,2   */
    v                      = agc_mpac_vshift(ip, agc_mpac_vad(ip, v, j22), -27 - w->x1); /* VAD VSL* 0 -27D,1  */
    const agc_vec_t      q = agc_vec_div(ip, agc_vec_from_words(v), w->r4n);             /* V/SC 0D; C31 + J22 terms / R^4, 2^-20 (moon-fixed axes) */
    agc_navaux_padload_t pad;                                                            /* lunar pad loads */
    agc_vec_t            r = agc_vec_zero();                                             /* the term in reference axes, 2^-20 */

    agc_navaux_get_padload(&pad);
    agc_status_t st = agc_planet_rp_to_r(ip, &pad, true, q, ctx->sv.tet, &r); /* RP-TO-R */

    if (st != AGC_STATUS_OK)
    {
        return (acc_fail(ctx, st));
    }

    agc_interp_clear_ovf(ip); /* "OVERFLOW INDICATOR RESET IN RP-TO-R" */
    ctx->fv = agc_vec_add(ip, r, ctx->fv);

    return (agc_interp_test_ovf(ip) ? ACC_OVF : ACC_OK); /* BOV GOBAQUE */
} /* lunar_c31_term */

/**********************************************************************
 * QUALITY1 / QUALITY2 (CM RT8_OP_CODES.agc p. 1516, LM ORBITAL_INTEGRATION.agc
 * p. 1248) -- the lunar J22 term x R^4, B61: E3J22R2M ( 5 (Y^2 - X^2) UR + (X,
 * -Y, 0) ) / 2, word level as QUALITY3
 *
 * FUNCTION NAME: lunar_j22_term
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  ----------------------------------
 *   ip        agc_interp_t *      I/O  interpretive state (OVFIND)
 *   w         const obl_work_t *  I    OBLATE cells (URPV, 2^1)
 *
 * RETURNS: vec_words_t -- the J22 term as words, B61
 *********************************************************************/
static agc_vec_words_t lunar_j22_term(agc_interp_t *ip, const obl_work_t *w)
{
    const agc_vec_words_t u  = agc_vec_to_words(w->urpv);                                  /* URPV words, 2^1 */
    const agc_dp_words_t  x2 = agc_mpac_store(agc_mpac_dsq(agc_mpac_dload(u.c[0])));       /* DLOAD DSQ, PDDL 2D */
    agc_tp_words_t        m  = agc_mpac_dsu(ip, agc_mpac_dsq(agc_mpac_dload(u.c[1])), x2); /* Y^2 - X^2 (DSQ DSU), 2^2 */

    m                 = agc_mpac_dmp(m, agc_dp_to_words(K_5_8));         /* DMP 5/8            */
    agc_vec_words_t q = agc_mpac_vshift(ip, agc_mpac_vxsc(ip, m, u), 3); /* VXSC URPV VSL3, 2D */

    q.c[0] = agc_mpac_store(agc_mpac_dad(ip, agc_mpac_shift(ip, agc_mpac_dload(u.c[0]), -1), q.c[0]));  /* SR1 DAD 2D  */
    q.c[1] = agc_mpac_store(agc_mpac_bdsu(ip, agc_mpac_shift(ip, agc_mpac_dload(u.c[1]), -1), q.c[1])); /* SR1 BDSU 4D */

    const agc_dp_words_t j22 = {e3j22r2m, 0};

    return (agc_mpac_vxsc(ip, agc_mpac_dload(j22), q)); /* SLOAD VXSC, PDDL 2D */
}

/**********************************************************************
 * COMTERM (ORBITAL_INTEGRATION.agc p. 1341) -- the Legendre polynomials P2/64,
 * P3/32, P4/128, P5/1024 in COSPHI/2
 *
 * FUNCTION NAME: legendre_terms
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  I/O  interpretive state (OVFIND)
 *   cphi      agc_dp_t        I    COSPHI/2, 2^1
 *
 * RETURNS: legendre_t -- P2..P5 as the words the DSU / BDSU leave
 *********************************************************************/
static legendre_t legendre_terms(agc_interp_t *ip, agc_dp_t cphi)
{
    const agc_dp_words_t c = agc_dp_to_words(cphi);                                         /* COSPHI/2 words, 2^1 */
    legendre_t           l;                                                                 /* P2..P5 words */
    agc_tp_words_t       m = agc_mpac_dmpr(ip, agc_mpac_dload(c), agc_dp_to_words(K_3_32)); /* DLOAD DMPR */

    l.p2 = agc_mpac_store(m); /* PDDL: P2/64 to 0D */
    m    = agc_mpac_dmpr(ip, agc_mpac_dsq(agc_mpac_dload(c)), agc_dp_to_words(K_15_16));
    m    = agc_mpac_dsu(ip, m, agc_dp_to_words(K_3_64));
    l.p3 = agc_mpac_store(m); /* PUSH: P3/32 to 2D */
    m    = agc_mpac_dmp(agc_mpac_dmpr(ip, m, c), agc_dp_to_words(K_7_12));

    const agc_dp_words_t t4 = agc_mpac_store(agc_mpac_shift_round(ip, m, 1)); /* SL1R PDDL 4D */

    m    = agc_mpac_bdsu(ip, agc_mpac_dmpr(ip, agc_mpac_dload(l.p2), agc_dp_to_words(K_2_3)), t4);
    l.p4 = agc_mpac_store(m); /* BDSU PUSH: P4/128 to 4D */
    m    = agc_mpac_dmpr(ip, agc_mpac_dmpr(ip, m, c), agc_dp_to_words(K_9_16));

    const agc_dp_words_t t6 = agc_mpac_store(m); /* PDDL 6D */

    m    = agc_mpac_bdsu(ip, agc_mpac_dmpr(ip, agc_mpac_dload(l.p3), agc_dp_to_words(K_5_128)), t6);
    l.p5 = agc_mpac_store(m); /* BDSU: P5/1024 */

    return (l);
}

/**********************************************************************
 * COMTERM (ORBITAL_INTEGRATION.agc pp. 1341-1342) -- TVEC = (((P5 J4/J3 / R +
 * P4) 2J3RE/J2 / R + P3) UR - (((P4 J4/J3 / R + P3) 2J3RE/J2 / R + P2) UZ,
 * before the J2/R^4 factor.  Word level throughout: the DAD results feed the
 * VXSCs as the AGC holds them.
 *
 * FUNCTION NAME: comterm_tvec
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------------
 *   ip        agc_interp_t *           I/O  interpretive state (OVFIND)
 *   ctx       const agc_integ_ctx_t *  I    integration erasables (PBODY,
 *                                           ALPHAV, ALPHAM)
 *   w         const obl_work_t *       I    OBLATE cells (COSPHI/2, UZ)
 *
 * RETURNS: vec_words_t -- TVEC as words
 *********************************************************************/
static agc_vec_words_t comterm_tvec(agc_interp_t *ip, const agc_integ_ctx_t *ctx, const obl_work_t *w)
{
    const int            b    = ctx->pbody / 2;                           /* PBODY / 2: 0 earth, 1 moon */
    const legendre_t     l    = legendre_terms(ip, w->cphi);              /* P2..P5 words */
    const agc_dp_words_t r_w  = agc_dp_to_words(ctx->alpham);             /* ALPHAM words, m 2^29 (moon 2^27) */
    const agc_dp_words_t j4_w = agc_dp_to_words(J4REQ_J3[b]);             /* J4REQ/J3 words */
    const agc_dp_words_t j3_w = agc_dp_to_words(J3RE_J2_2[b]);            /* 2J3RE/J2 words */
    agc_tp_words_t       m    = agc_mpac_dmp(agc_mpac_dload(l.p5), j4_w); /* DMP* J4REQ/J3,2 */

    m                        = agc_mpac_dad(ip, agc_mpac_ddv(ip, m, r_w), l.p4);    /* DDV DAD 4      */
    m                        = agc_mpac_dmpr(ip, m, j3_w);                          /* DMPR* 2J3RE/J2 */
    m                        = agc_mpac_dad(ip, agc_mpac_ddv(ip, m, r_w), l.p3);    /* DDV DAD 2      */
    const agc_vec_words_t ur = agc_mpac_vxsc(ip, m, agc_vec_to_words(ctx->alphav)); /* VXSC ALPHAV    */

    m                  = agc_mpac_shift(ip, agc_mpac_dmp(agc_mpac_dload(l.p4), j4_w), -1); /* STODL DMP* SR1 */
    m                  = agc_mpac_dad(ip, agc_mpac_ddv(ip, m, r_w), l.p3);                 /* DDV DAD        */
    m                  = agc_mpac_shift(ip, agc_mpac_dmpr(ip, m, j3_w), -3);               /* DMPR* SR3      */
    m                  = agc_mpac_dad(ip, agc_mpac_ddv(ip, m, r_w), l.p2);                 /* DDV DAD        */
    agc_vec_words_t uz = agc_mpac_vxsc(ip, m, agc_vec_to_words(w->uz));                    /* VXSC UZ        */
    agc_vec_words_t t;                                                                     /* TVEC words */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        int ovf = 0; /* |VXSC UZ| <= |MPAC| / 2: VSL1 cannot overflow */

        uz.c[i] = agc_dp_words_das(uz.c[i], uz.c[i], &ovf);                            /* VSL1: DAS      */
        t.c[i]  = agc_mpac_store(agc_mpac_bdsu(ip, agc_mpac_dload(uz.c[i]), ur.c[i])); /* BVSU TVEC      */
    }

    return (t);
}

/**********************************************************************
 * OBLATE (ORBITAL_INTEGRATION.agc pp. 1341-1342) -- within RDE (80467 km) of
 * the earth or RDM (16093 km) of the moon add the zonal harmonics (earth J2 J3
 * J4, moon J2 J3) and, for the moon, QUALITY1-3 (J22, C31).  ALPHAV = unit R,
 * ALPHAM = |R|.  NBRANCH follows in the caller.
 *
 * FUNCTION NAME: oblate
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (ALPHAV, ALPHAM,
 *                                     FV)
 *
 * RETURNS: acc_result_t -- ACC_OK, ACC_OVF (GOBAQUE) or ACC_FAIL with
 *                          ctx->fail
 *********************************************************************/
static acc_result_t oblate(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const int  b = ctx->pbody / 2; /* PBODY / 2: 0 earth, 1 moon */
    obl_work_t w;                  /* OBLATE push-list cells */

    if (agc_dp_sub(ip, ctx->alpham, RD_TAB[b]) >= 0) /* b: PBODY / 2, 0 or 1 */
    {
        return (ACC_OK); /* BPL NBRANCH   */
    }

    const bool moon = agc_flag_test(AGC_FLAG_MOONFLAG); /* moon primary */

    w.urpv = ctx->alphav;

    if (!moon)
    { /* COSPHIE       */
        w.cphi = ctx->alphav.c[2];
        w.uz   = agc_vec_make(0, 0, AGC_DP_HALF); /* ZUNIT         */
    }
    else
    { /* URPV, COSPHI/2, UZ */
        const agc_status_t st = moon_axes(ip, ctx, &w); /* status of the moon axes */

        if (st != AGC_STATUS_OK)
        {
            return (acc_fail(ctx, st));
        }
    }

    const agc_vec_words_t tvec = comterm_tvec(ip, ctx, &w); /* TVEC words, before the J2/R^4 factor */
    int                   n    = 0;                         /* NORM count of ALPHAM (X1) */
    int                   n4   = 0;                         /* NORM count of R^4 (S1) */

    /* NORM X1 DSQ DSQ NORM S1: the normalised R^4 (PUSH 0D); BDDV* keeps
     * its MPAC+2 under the dividend J2REQSQ.                                */
    agc_tp_words_t m = agc_mpac_dsq(agc_mpac_norm(agc_mpac_dload(agc_dp_to_words(ctx->alpham)), &n)); /* R normalised, squared (TP) */

    m                       = agc_mpac_norm(agc_mpac_dsq(m), &n4);
    w.r4n                   = agc_dp_from_words(agc_mpac_store(m));                  /* PUSH 0D     */
    m                       = agc_mpac_bddv(ip, m, agc_dp_to_words(J2REQSQ_TAB[b])); /* BDDV*       */
    const agc_vec_words_t v = agc_mpac_vxsc(ip, m, tvec);                            /* VXSC TVEC   */

    agc_interp_clear_ovf(ip); /* BOV +1 (reset) */
    w.x1    = -(4 * n) - n4;  /* XAD,1 X1 X1 S1 */
    ctx->fv = agc_vec_add(ip, /* VSL* 0 -22D,1 VAD */
                          agc_vec_from_words(agc_mpac_vshift(ip, v, -22 - w.x1)),
                          ctx->fv);

    if (agc_interp_test_ovf(ip))
    {
        return (ACC_OVF); /* BOV GOBAQUE    */
    }

    if (!moon)
    {
        return (ACC_OK); /* QUALITY1 BOF   */
    }

    return (lunar_c31_term(ip, ctx, &w, lunar_j22_term(ip, &w))); /* QUALITY1-3 */
} /* oblate */

/**********************************************************************
 * ACCOMP (ORBITAL_INTEGRATION.agc p. 1337) -- FV = disturbing acceleration at
 * ALPHAV (the deviation): primary body by GAMCOMP; with MIDFLAG the secondary
 * body and the sun; then OBLATE
 *
 * FUNCTION NAME: accomp
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (ALPHAV, RCV, FV,
 *                                     VECTAB)
 *
 * RETURNS: acc_result_t -- ACC_OK, ACC_OVF (GOBAQUE) or ACC_FAIL with
 *                          ctx->fail
 *********************************************************************/
static acc_result_t accomp(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const int p = ctx->pbody; /* PBODY: 0 earth, 2 moon */

    ctx->fv    = agc_vec_zero();
    ctx->betav = agc_vec_add(ip, agc_vec_shr(ctx->alphav, 7 + p), ctx->sv.rcv);

    if (agc_flag_test(AGC_FLAG_DIM0FLAG))
    {
        ctx->vectab[2 * ctx->stage] = ctx->betav; /* VECTAB,2       */
    }

    agc_dp_t am = 0; /* ALPHAM = |ALPHAV|, m 2^22 (moon 2^18) */

    ctx->alphav = agc_vec_unit(ip, ctx->alphav, &am, NULL);
    ctx->alpham = am; /* 36D            */

    if (gamcomp(ip, ctx, p, p))
    {
        return (ACC_OVF);
    }

    ctx->alphav = ctx->betav; /* unit R         */
    ctx->alpham = ctx->betam; /* |R|            */

    if (agc_flag_test(AGC_FLAG_MIDFLAG))
    {
        const acc_result_t r = third_body(ip, ctx); /* outcome of the third-body terms */

        if (r != ACC_OK)
        {
            return (r);
        }
    }

    return (oblate(ip, ctx));
}

/**********************************************************************
 * DOW..1 (ORBITAL_INTEGRATION.agc p. 1352) -- gravity-gradient derivative of a
 * W column, BETAM [3 (A.UR) UR - A] / |R|^3 for R = VECTAB entry, A = ALPHAV
 *
 * FUNCTION NAME: body_gravity_gradient
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------------
 *   ip        agc_interp_t *           I/O  interpretive state (OVFIND)
 *   ctx       const agc_integ_ctx_t *  I    integration erasables (ALPHAV,
 *                                           BETAM = MU)
 *   r         agc_vec_t                I    VECTAB entry, m 2^29 (moon 2^27)
 *
 * RETURNS: agc_vec_t -- the derivative at the FV scale of the W integration
 *********************************************************************/
static agc_vec_t body_gravity_gradient(agc_interp_t *ip, const agc_integ_ctx_t *ctx, agc_vec_t r)
{
    const agc_vec_t a16  = agc_vec_shr(ctx->alphav, 4);      /* 0D; ALPHAV / 16, 2^5 */
    agc_dp_t        rmag = 0;                                /* |R|, m 2^29 (moon 2^27) */
    const agc_vec_t u    = agc_vec_unit(ip, r, &rmag, NULL); /* 36D; unit R, 2^1 */
    agc_vec_t       g    = agc_vec_proj(ip, ctx->alphav, u); /* VPROJ; (A.UR) UR, 2^1 */

    g                 = agc_vec_sub(ip, agc_vec_scale(ip, g, K_3_4), a16);
    int            n  = 0;                                                           /* NORM count of |R| (S2) */
    int            m  = 0;                                                           /* NORM count of |R|^3 (34D) */
    const agc_dp_t rn = agc_dp_norm(rmag, &n);                                       /* S2; |R| normalised, 2^0 */
    const agc_tp_t r3 = tp_norm(ip, agc_dp_mul_tp(agc_dp_sq(rn), rn), &m);           /* |R|^3 normalised, TP 2^0 */
    const agc_dp_t k  = agc_dp_div(ip, agc_dp_shr(ctx->betam, 1), agc_tp_to_dp(r3)); /* BETAM / 2 / |R|^3 (SR1 DDV), normalised 2^0 */

    g = agc_vec_scale(ip, g, k);

    return (agc_vec_shl(ip, g, (3 * n) + m - 8)); /* 0 -8D,2 */
}

/**********************************************************************
 * DOW.. (ORBITAL_INTEGRATION.agc p. 1352) -- FV for the W integration from the
 * stored positions (primary body; with MIDFLAG also the secondary)
 *
 * FUNCTION NAME: dow
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (VECTAB, FV)
 *********************************************************************/
static void dow(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const int p = ctx->pbody; /* PBODY: 0 earth, 2 moon */
    const int k = ctx->stage; /* Nystrom stage (DIFEQCNT) */

    ctx->betam = agc_integ_mu(p); /* MUEARTH,2   */
    ctx->fv    = body_gravity_gradient(ip, ctx, ctx->vectab[2 * k]);

    if (agc_flag_test(AGC_FLAG_MIDFLAG))
    {
        ctx->betam   = agc_integ_mu(2 - p);                                      /* MUEARTH -2,2, X2 = -PBODY */
        agc_vec_t f2 = body_gravity_gradient(ip, ctx, ctx->vectab[(2 * k) + 1]); /* secondary body term, FV scale (earth primary: VSR6 below) */

        if (!agc_flag_test(AGC_FLAG_MOONFLAG))
        {
            f2 = agc_vec_shr(f2, 6); /* VSR6        */
        }

        ctx->fv = agc_vec_add(ip, f2, ctx->fv);
    }
}

/* ======================================================================
 * Nystrom step: DIFEQ0, DIFEQ+0/+1/+2, DIFEQCOM
 * ==================================================================== */

/**********************************************************************
 * DIFEQ0 (ORBITAL_INTEGRATION.agc p. 1345) -- ALPHAV = YV, DIFEQCNT = 0, H = 0
 *
 * FUNCTION NAME: nystrom_start
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables
 *********************************************************************/
static void nystrom_start(agc_integ_ctx_t *ctx)
{
    ctx->alphav = ctx->yv;
    ctx->stage  = 0;
    ctx->h      = 0;
}

/**********************************************************************
 * DIFEQ+0 / DIFEQ+1 (ORBITAL_INTEGRATION.agc p. 1348) and DIFEQCOM (p. 1350)
 * -- accumulate PHIV, PSIV, advance H by DT/2 and predict ALPHAV = YV + H (ZV
 * + H FV / 2)
 *
 * FUNCTION NAME: nystrom_stage
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables
 *********************************************************************/
static void nystrom_stage(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    if (ctx->stage == 0)
    {
        ctx->phiv = agc_vec_shr(ctx->fv, 3); /* DIFEQ+0 */
    }
    else
    {
        const agc_vec_t f2 = agc_vec_shr(ctx->fv, 1); /* DIFEQ+1; FV / 2, FV scale 2^1 */

        ctx->psiv = agc_vec_add(ip, f2, ctx->phiv);
        ctx->phiv = agc_vec_add(ip, agc_vec_shr(f2, 1), ctx->phiv);
    }

    ctx->h      = agc_dp_add(ip, ctx->dt2, ctx->h);                   /* DIFEQCOM */
    ctx->stage  += 1;
    agc_vec_t a = agc_vec_shr(agc_vec_scale(ip, ctx->fv, ctx->h), 1); /* H FV / 2, deviation velocity scale */

    a           = agc_vec_scale(ip, agc_vec_add(ip, a, ctx->zv), ctx->h);
    ctx->alphav = agc_vec_add(ip, a, ctx->yv);
}

/**********************************************************************
 * DIFEQ+2 (ORBITAL_INTEGRATION.agc p. 1348) -- end of the step: YV and ZV
 * updated
 *
 * FUNCTION NAME: nystrom_end
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables
 *********************************************************************/
static void nystrom_end(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const agc_dp_t h23 = agc_dp_mul(ip, ctx->h, K_2_3);                         /* H 2/3, cs 2^19 */
    agc_vec_t      y   = agc_vec_shl(ip, agc_vec_scale(ip, ctx->phiv, h23), 1); /* 2/3 H PHIV, deviation velocity scale */

    y           = agc_vec_scale(ip, agc_vec_add(ip, y, ctx->zv), ctx->h);
    ctx->yv     = agc_vec_add(ip, y, ctx->yv);
    agc_vec_t z = agc_vec_add(ip, agc_vec_shr(ctx->fv, 3), ctx->psiv); /* FV / 8 + PSIV, FV scale */

    z       = agc_vec_shl(ip, agc_vec_scale(ip, z, h23), 1);
    ctx->zv = agc_vec_add(ip, z, ctx->zv);
}

/**********************************************************************
 * AMOVED, NEXTCOL, DIFEQ+2 (JSWITCH), RELOADSV (ORBITAL_INTEGRATION.agc pp.
 * 1348-1350) -- integrate the W columns COLREG = -30 (6x6, D6OR9FLG clear) or
 * -48 (9x9) up to 0, i.e. columns 5..0 or 8..0.  Position part blk[0][m],
 * velocity part blk[1][m]; YV/ZV = W SR3.
 * DEVIATION (restart): the AGC restarts group 2 after the NEXTCOL / DIFEQ+2
 * GRP2PC and finishes the column in COLREG; C reruns the calling job segment
 * from its top, which integrates the state ATOPCSM/ATOPLEM already moved and
 * all W columns again from there, so columns not yet done when the restart
 * came lag one step.
 *
 * FUNCTION NAME: agc_integ_wcolumns
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (W, VECTAB)
 *
 * RETURNS: agc_integ_next_t -- INTEG_NEXT_RELOAD (RELOADSV), or
 *                              INTEG_NEXT_TESTLOOP after WMATEND (W overflow,
 *                              ALARM 421)
 *********************************************************************/
agc_integ_next_t agc_integ_wcolumns(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_flag_put(AGC_FLAG_JSWITCH, true);                       /* AMOVED   */
    const int first = agc_flag_test(AGC_FLAG_D6OR9FLG) ? 8 : 5; /* last column: 8 (9x9, D6OR9FLG) or 5 (6x6) */

    for (int col = first; col >= 0; col--)
    {
        ctx->colreg = col;
        agc_integ_grp2pc(); /* NEXTCOL  */
        ctx->yv     = agc_vec_shr(ctx->w.blk[0][col], 3);
        ctx->zv     = agc_vec_shr(ctx->w.blk[1][col], 3);

        /* One W column over the step just taken: DIFEQ0, DOW.., DIFEQ+0/+1/+2. */

        nystrom_start(ctx);

        for (int k = 0; k < 3; k++)
        {
            dow(ip, ctx); /* NBRANCH -> DIFEQ+k    */

            if (k < 2)
            {
                nystrom_stage(ip, ctx); /* DIFEQCOM -> DOW..     */
            }
        }

        nystrom_end(ip, ctx);
        agc_integ_grp2pc();                               /* DIFEQ+2  */
        ctx->w.blk[1][col] = agc_vec_shl(ip, ctx->zv, 3); /* W +54D,2 */
        const agc_vec_t y  = agc_vec_shl(ip, ctx->yv, 3); /* W position part, W scale */

        if (agc_interp_test_ovf(ip))
        {
            /* WMATEND (p. 1350): W overflowed. Stop integrating and invalidating it;
             * pick up a permanent state vector update instead; ALARM 421.            */
            agc_flag_put(AGC_FLAG_DIM0FLAG, false);
            agc_flag_put(AGC_FLAG_ORBWFLAG, false);
            agc_flag_put(AGC_FLAG_RENDWFLG, false);
            agc_flag_put(AGC_FLAG_STATEFLG, true);
            agc_alarm(AGC_INTEG_ALARM_WMATEND);

            return (INTEG_NEXT_TESTLOOP);
        }

        ctx->w.blk[0][col] = y; /* W,2      */
        agc_integ_grp2pc();
    }

    return (INTEG_NEXT_RELOAD); /* RELOADSV */
} /* agc_integ_wcolumns */

/**********************************************************************
 * RECTEST (ORBITAL_INTEGRATION.agc p. 1344) -- rectify when |TDELTAV| or
 * |TNUV| reaches 3/4 of its scale (or overflows), or |TDELTAV| >= .01 |RCV|
 * (10D)
 *
 * FUNCTION NAME: rectest
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the state
 *                                     vector, RC)
 *
 * RETURNS: none
 *********************************************************************/
static void rectest(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    const int      p    = ctx->pbody;                        /* PBODY: 0 earth, 2 moon */
    const agc_dp_t d    = agc_vec_abval(ip, ctx->sv.deltav); /* |TDELTAV|, m 2^22 (moon 2^18) */
    bool           rect = agc_interp_test_ovf(ip);           /* BOV      */

    if (!rect)
    {
        rect = (agc_dp_sub(ip, d, K_3_4) >= 0); /* DSU BPL  */
    }

    if (!rect)
    {
        agc_dp_t q = agc_dp_shr(d, 7 + p); /* SL* 0 -7,2; |TDELTAV| at the RCV scale, m 2^29 (moon 2^27) */

        q    = agc_dp_sub(ip, agc_dp_div(ip, q, ctx->rc), K_RECRATIO);
        rect = (q >= 0);
    }

    if (!rect)
    {
        const agc_dp_t e = agc_dp_sub(ip, agc_vec_abval(ip, ctx->sv.nuv), K_3_4); /* |TNUV| - 3/4, m/cs 2^3 (moon 2^-1) */

        rect = agc_interp_test_ovf(ip) || (e >= 0); /* BOV BMN  */
    }

    if (rect)
    {
        agc_integ_rectify(ip, ctx); /* CALLRECT */
    }
}

/**********************************************************************
 * TIMESTEP origin test (Luminary099 ORBITAL_INTEGRATION.agc pp. 1239-1240) --
 * only when MIDFLAG, which Luminary099 never sets (RME = RMM = DPPOSMAX), and
 * only when the vehicle is receding, (R.V) DT >= 0
 *
 * FUNCTION NAME: lm_sphere
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (RCV, VCV, RPQV)
 *   sw        bool *             O    true: change origin (DOSWITCH)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the LUNPOS / LSPOS status
 *********************************************************************/
static agc_status_t lm_sphere(agc_interp_t *ip, agc_integ_ctx_t *ctx, bool *sw)
{
    agc_vec_t sun; /* sun position (LSPOS, unused), m 2^38 */

    *sw = false;

    const agc_dp_t rv = agc_dp_mul_trunc(agc_vec_dot(ip, ctx->sv.rcv, ctx->sv.vcv), ctx->dt2); /* DOT DMP: sign only; R.V DT/2, 2^55 m^2/cs . cs (sign only) */

    if (rv < 0)
    {
        return (AGC_STATUS_OK); /* RECTEST  */
    }

    if (agc_flag_test(AGC_FLAG_MOONFLAG))
    { /* LUNSPH   */
        if (agc_dp_sub(ip, agc_dp_shr(ctx->rc, 2), K_RSPHERE) < 0)
        {
            return (AGC_STATUS_OK);
        }

        if (agc_flag_test(AGC_FLAG_RPQFLAG))
        {
            agc_vec_t          m;                                /* earth-centred moon, m 2^29 */
            const agc_status_t st = lunpos(ip, ctx->sv.tet, &m); /* status of LUNPOS */

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }

            ctx->rpqv = agc_vec_neg(m);
        }

        *sw = true; /* DOSWITCH */
    }
    else
    {
        if (agc_flag_test(AGC_FLAG_RPQFLAG))
        { /* else EARSPH */
            const agc_status_t st = agc_lsposition(ip, ctx->sv.tet, &ctx->rpqv, &sun); /* status of LSPOS */

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }
        }

        const agc_dp_t d = agc_vec_abval(ip, agc_vec_sub(ip, ctx->sv.rcv, ctx->rpqv)); /* |R - RPQ|, m 2^29 */

        *sw = (agc_dp_sub(ip, d, K_RSPHERE) < 0); /* INLUNCHK */
    }

    return (AGC_STATUS_OK);
} /* lm_sphere */

/**********************************************************************
 * INTGRATE .. ENDSTATE (ORBITAL_INTEGRATION.agc pp. 1345-1349) -- the state
 * step: three ACCOMP evaluations at H = 0, DT/2, DT with FBR3 conic advances
 * in between
 *
 * FUNCTION NAME: integrate_step
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables
 *
 * RETURNS: agc_integ_next_t -- the next label: TESTLOOP, CKMID2, WMATRIX,
 *                              GOBAQUE's, or INTEG_NEXT_FAIL with ctx->fail
 *********************************************************************/
static agc_integ_next_t integrate_step(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    ctx->zv = ctx->sv.nuv; /* INTGRATE */
    ctx->yv = ctx->sv.deltav;
    agc_flag_put(AGC_FLAG_JSWITCH, false);

    nystrom_start(ctx);

    for (int k = 0; k < 3; k++)
    { /* NBRANCH stages        */
        const acc_result_t r = accomp(ip, ctx); /* outcome of ACCOMP */

        if (r == ACC_OVF)
        {
            return (gobaque(ip, ctx));
        }

        if (r != ACC_OK)
        {
            return (INTEG_NEXT_FAIL); /* ACCOMP kept the status in ctx->fail */
        }

        if (k == 2)
        {
            break;
        }

        nystrom_stage(ip, ctx);

        const agc_status_t st = advance_conic_half_step(ip, ctx); /* status of FBR3 (KEPPREP) */

        if (st != AGC_STATUS_OK)
        {
            return (agc_integ_fail(ctx, st));
        }
    }

    nystrom_end(ip, ctx);

    if (agc_interp_test_ovf(ip))
    { /* ENDSTATE BOV GOBAQUE  */
        return (gobaque(ip, ctx));
    }

    ctx->sv.nuv    = ctx->zv;
    ctx->sv.deltav = ctx->yv;

    if (agc_flag_test(AGC_FLAG_MIDAVFLG))
    {
        return (INTEG_NEXT_MIDTOAV_CHECK);
    }

    if (!agc_flag_test(AGC_FLAG_DIM0FLAG))
    {
        return (INTEG_NEXT_TESTLOOP);
    }

    return (INTEG_NEXT_WMATRIX);
} /* integrate_step */

/**********************************************************************
 * TIMESTEP / RECTEST / CALLRECT (ORBITAL_INTEGRATION.agc pp. 1344-1345) --
 * then the state step through ENDSTATE.  With MIDFLAG the sphere-of-influence
 * test (CHKSWTCH, CM; R.V test, LM) may change origin (DOSWITCH) instead of
 * testing for rectification.
 *
 * FUNCTION NAME: agc_integ_timestep
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables
 *
 * RETURNS: agc_integ_next_t -- the next label, or INTEG_NEXT_FAIL with
 *                              ctx->fail
 *********************************************************************/
agc_integ_next_t agc_integ_timestep(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    bool         sw = false;         /* change origin (DOSWITCH) */
    agc_status_t st = AGC_STATUS_OK; /* status of the origin / rectification test */

    if (agc_flag_test(AGC_FLAG_MIDFLAG))
    {
        if (agc_state_vehicle() == AGC_VEHICLE_CSM)
        {
            agc_dp_t test = 0; /* CHKSWTCH test value, m 2^29 */

            st = agc_integ_chkswtch(ip, ctx, &test);
            sw = (test < 0); /* BMN      */
        }
        else
        {
            st = lm_sphere(ip, ctx, &sw);
        }
    }

    if (st == AGC_STATUS_OK && sw)
    {
        st = agc_integ_origchng(ip, ctx); /* DOSWITCH */
    }
    else if (st == AGC_STATUS_OK)
    {
        rectest(ip, ctx); /* RECTEST */
    }

    if (st != AGC_STATUS_OK)
    {
        return (agc_integ_fail(ctx, st));
    }

    return (integrate_step(ip, ctx));
}

/**********************************************************************
 * CHKSWTCH / RPQOK / EARSPH (ORBITAL_INTEGRATION.agc C p. 1345) -- the
 * sphere-of-influence test: moon primary: RSPHERE - |R| (R = RCV + TDELTAV);
 * earth primary: |R - RPQ| - RSPHERE.  RPQV from LUNPOS when RPQFLAG is set.
 *
 * FUNCTION NAME: agc_integ_chkswtch
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (RCV, TDELTAV,
 *                                     RPQV)
 *   test      agc_dp_t *         O    the test value, m 2^29: >= 0 stay, < 0
 *                                     change origin
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the LUNPOS status
 *********************************************************************/
agc_status_t agc_integ_chkswtch(agc_interp_t *ip, agc_integ_ctx_t *ctx, agc_dp_t *test)
{
    const bool moon = agc_flag_test(AGC_FLAG_MOONFLAG); /* moon primary */

    if (agc_flag_test(AGC_FLAG_RPQFLAG))
    {
        agc_vec_t          m;                                /* earth-centred moon, m 2^29 */
        const agc_status_t st = lunpos(ip, ctx->sv.tet, &m); /* status of LUNPOS */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        ctx->rpqv = moon ? agc_vec_neg(m) : m; /* RPQV: earth from the moon, or the moon */
    }

    const int       p = ctx->pbody;                                                       /* PBODY: 0 earth, 2 moon */
    const agc_vec_t r = agc_vec_add(ip, agc_vec_shr(ctx->sv.deltav, 7 + p), ctx->sv.rcv); /* R = RCV + TDELTAV, m 2^29 (moon 2^27) */

    if (moon)
    {
        *test = agc_dp_sub(ip, K_RSPHERE, agc_dp_shr(agc_vec_abval(ip, r), 2));
    }
    else
    {
        *test = agc_dp_sub(ip, agc_vec_abval(ip, agc_vec_sub(ip, r, ctx->rpqv)), K_RSPHERE);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ORIGCHNG (ORBITAL_INTEGRATION.agc p. 1346) -- rectify, then move the origin
 * to the other body:
 *   RRECT = RCV = (RCV SR PBODY - RPQV) SL (2 - PBODY)
 *   VRECT = VCV = (VCV SR PBODY - VMOON) SL (2 - PBODY), VMOON negated
 * when the moon was primary; MOONFLAG and PBODY toggle.
 *
 * FUNCTION NAME: agc_integ_origchng
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     I/O  interpretive state (OVFIND)
 *   ctx       agc_integ_ctx_t *  I/O  integration erasables (the state
 *                                     vector, RPQV, PBODY)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the RECTIFY / LUNVEL status
 *********************************************************************/
agc_status_t agc_integ_origchng(agc_interp_t *ip, agc_integ_ctx_t *ctx)
{
    agc_integ_rectify(ip, ctx);

    const int  p    = ctx->pbody;                                              /* PBODY before the change: 0 earth, 2 moon */
    const bool moon = agc_flag_test(AGC_FLAG_MOONFLAG);                        /* moon primary before the change */
    agc_vec_t  r    = agc_vec_sub(ip, agc_vec_shr(ctx->sv.rcv, p), ctx->rpqv); /* position about the new body, m 2^29 (moon 2^27) */

    r             = agc_vec_shl(ip, r, 2 - p); /* VSL* 2,2 */
    ctx->sv.rrect = r;
    ctx->sv.rcv   = r;
    agc_vec_t vm;                              /* moon velocity, m/cs 2^7 */

    const agc_status_t st = lunvel(ip, ctx->sv.tet, &vm); /* LUNVEL */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (moon)
    {
        vm = agc_vec_neg(vm);
    }

    agc_vec_t v = agc_vec_sub(ip, agc_vec_shr(ctx->sv.vcv, p), vm); /* velocity about the new body, m/cs 2^7 (moon 2^5) */

    v             = agc_vec_shl(ip, v, 2 - p); /* VSL* 0 +2,2 */
    ctx->sv.vrect = v;
    ctx->sv.vcv   = v;

    agc_integ_setmoon(ctx, !moon); /* CLR/SETMOON */

    return (AGC_STATUS_OK);
}
