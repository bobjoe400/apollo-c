/**********************************************************************
 * FILE NAME: ascent.c
 *
 * PURPOSE:
 *   P12.agc (Luminary 099, pp. 838-842) and ASCENT_GUIDANCE.agc
 *   (pp. 843-856): powered ascent from the lunar surface (P12), the ascent
 *   guidance equations used by P12, P70 and P71 (ATMAG, ASCENT), engine
 *   cut-off (ENGOFF..TERMASC), the cross-range YCOMP and the logarithm
 *   LOGSUB.
 *
 *   The ascent guidance is a linear-tangent-like law in the frame of
 *   UNIT/R/ (radial), LAXIS (cross range) and ZAXIS1 (downrange): the
 *   thrust acceleration components ATR = (A + B t)/TBUP - GEFF and
 *   ATY = (C + D t)/TBUP, ATP from the remaining magnitude.
 *
 *   Scales: R 2^24 m, V 2^7 m/cs, TGO TBUP TTO 2^17 cs, accelerations
 *   2^-9 m/cs^2 unless stated; the push-down list holds -L (0D), D12 (2D),
 *   D21 (4D) and E (6D) in the AGC and C locals here.
 *
 * AGC SOURCE:
 *   Luminary099 P12.agc, pp. 838-842; ASCENT_GUIDANCE.agc, pp. 843-856
 *
 * EXTERNAL VARIABLES:
 *   agc_lm_asc() (ascent erasables), agc_lm_ldg() (UNFC/2, UNWC/2,
 *   /LAND/), agc_lm_pad(); agc_guid_state() (R, V, UNIT/R/, GDT1/2,
 *   XNBPIP, MASS1, ABDVCONV)
 *
 * EXTERNAL REFERENCES:
 *   agc_lm_findcduw_from_unfc, agc_lm_burnbaby, agc_lm_enginof2,
 *   agc_lm_dap_zatteror/setmindb/restordb/pflitedb, agc_integ_lemprec,
 *   agc_lm_mungrav, agc_waitlist_call, agc_exec_findvac,
 *   agc_dsky_display
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none (no alarms; the job ends at ASCTERM or in P00 after TERMASC)
 **********************************************************************/
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

/* ---- constants (pp. 842, 836, 852, 855-856; CONTROLLED_CONSTANTS) ----- */
#define AS_THRESH2  308                    /* THRESH2: DVMON threshold after ignition (APS) */
#define AS_TGOA     ((agc_dp_t) 75776000)  /* (TGO)A 2DEC 37000 B-17       */
#define AS_1DVA     ((agc_dp_t) 31876710)  /* (1/DV)A 2DEC 15.20 B-7       */
#define AS_ATA      ((agc_dp_t) 45194051)  /* (AT)A 2DEC 3.2883 E-4 B9     */
#define AS_TBUPA    ((agc_dp_t) 188215296) /* (TBUP)A 2DEC 91902 B-17      */
#define AS_ATDECAY  ((agc_dp_t) -10)       /* ATDECAY 2DEC -10             */
#define AS_APSVEX   (-15514)               /* APSVEX DEC -3030 E-2 B-5     */
#define AS_HINJECT  ((agc_dp_t) 292608)    /* 2DEC 18288 B-24: 60,000 ft   */
#define AS_MOONRATE ((agc_dp_t) 3746009)   /* 2DEC* .26616994890062991 E-7 B+19 */
#define AS_49FPS    ((agc_dp_t) 626428)    /* 2DEC .149352 B-6             */
#define AS_VINJNOM  ((agc_dp_t) 35216215)  /* 2DEC 16.7924 B-7: 5509.5 fps */
#define AS_RDOTDNOM ((agc_dp_t) 124646)    /* 2DEC .059436 B-7: 19.5 fps   */
#define AS_ATRCS    ((agc_dp_t) 21577916)  /* AT/RCS 2DEC .0000785 B+10    */
#define AS_KT1      ((agc_dp_t) 134217728) /* KT1 2DEC 0.5000              */
#define AS_NEARONE  AGC_DP_MAX             /* NEARONE 2DEC .999999999      */
#define AS_4SEC17   ((agc_dp_t) 819200)    /* 2DEC 400 B-17                */
#define AS_T2A      ((agc_dp_t) 409600)    /* T2A = 2SEC(17)               */
#define AS_T3       ((agc_dp_t) 2048000)   /* T3 2DEC 1000 B-17            */
#define AS_100CS    ((agc_dp_t) 204800)    /* 100CS = 2SEC(18)             */
#define AS_2SEC18   ((agc_dp_t) 204800)    /* 2DEC 200 B-18                */
#define AS_2SEC9    ((agc_dp_t) 104857600) /* 2DEC 200 B-9                 */
#define AS_6SEC18   ((agc_dp_t) 614400)    /* 2DEC 600 B-18                */
#define AS_PRLIMIT  ((agc_dp_t) -17153026) /* 2DEC -.0639                  */
#define AS_MINABDV  ((agc_dp_t) 298634)    /* 2DEC .0356 B-5               */
#define AS_BIT3H    4                      /* BIT3H OCT 4                  */
#define AS_25KFT    ((agc_dp_t) 121920)    /* 2DEC 7620 B-24               */
#define AS_10SECS   ((agc_dp_t) 1000)      /* 2DEC 1000                    */
#define AS_SIN5DEG  ((agc_dp_t) 5849209)   /* 2DEC 0.08716 B-2             */
#define AS_40FPS    ((agc_dp_t) 255685)    /* 2DEC 0.12192 B-7             */
#define AS_CLOG2_32 ((agc_dp_t) 5814540)   /* 2DEC .0216608494             */
#define AS_PRIO17   017                    /* PRIO17: CUTOFF job priority                   */

/* LOGSUB polynomial (p. 856), -LOG via ln(1 - u)/32. */
static const agc_dp_t as_logcoef[8] = {2, -8388994, -4179185, -3019954, -497816, -7693572, 10350833, -11257160};

/* ======================================================================
 * Initialisation (P12.agc pp. 840-842)
 * ====================================================================== */

/**********************************************************************
 * GUIDINIT (pp. 841-842): WM = MOONRATE x (moon's pole in stable member
 * axes now), /LAND/ = |RLS| (2^24 m).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_guidinit(agc_interp_t *ip)
{
    agc_navaux_padload_t nav;                               /* pad loads of the auxiliary nav lane */
    agc_status_t         st = agc_navaux_get_padload(&nav); /* step status */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_vec_t pole = agc_vec_zero(); /* moon's pole in SM, half unit 2^1 */

    const agc_status_t st_pole = agc_planet_rp_to_r(ip, &nav, true, agc_vec_make(0, 0, AGC_DP_HALF), agc_rtb_loadtime(), &pole); /* step status */

    if (!AGC_ASSERT(st_pole == AGC_STATUS_OK))
    {
        return (st_pole);
    }

    agc_mat_t     m = agc_erase_refsmmat(); /* REFSMMAT, half unit */
    agc_lm_ldg_t *l = agc_lm_ldg();         /* landing erasables */

    l->wm       = agc_vec_scale(ip, agc_mat_vec(ip, &m, pole), AS_MOONRATE);
    l->land_mag = agc_dp_shl(ip, agc_vec_abval(ip, agc_erase_rls()), 3);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * COMMINIT (p. 841): targets for P12, P70 and P71 not following P70:
 * RCO = /LAND/ + 60,000 ft; QAXIS = UNIT(REFSMMAT (VCSM x RCSM)).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 **********************************************************************/
void agc_lm_comminit(agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc(); /* ascent erasables */

    a->rco      = agc_dp_add(ip, AS_HINJECT, agc_lm_ldg()->land_mag);
    a->txo      = 0;
    a->yco      = 0;
    a->rdotd    = 0;
    a->ydotd    = 0;
    agc_mat_t m = agc_erase_refsmmat();                                          /* REFSMMAT, half unit */
    agc_vec_t h = agc_vec_cross(ip, agc_erase_vrectcsm(), agc_erase_rrectcsm()); /* VCSM x RCSM: CSM orbit normal, 2^36 */

    a->qaxis = agc_vec_unit(ip, agc_mat_vec(ip, &m, h), NULL, NULL);
}

/**********************************************************************
 * P12INIT (pp. 840-841): APS engine data for P12 and P71; COMMINIT too
 * unless FLAP (P71 after P70 keeps the targets).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 **********************************************************************/
void agc_lm_powered_ascent_init(agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc(); /* ascent erasables */

    a->dv[3] = AS_1DVA;
    a->dv[2] = AS_1DVA;
    a->dv[1] = AS_1DVA;
    a->at    = AS_ATA;
    a->tbup  = AS_TBUPA;
    a->tto   = agc_dp_shl(ip, agc_dp_neg(AS_ATDECAY), 11);
    a->ve    = agc_dp_shr(agc_dp_from_sp((agc_sp_t) -AS_APSVEX), 2);

    if (!agc_flag_test(AGC_FLAG_FLAP)) /* RVQ          */
    {
        agc_lm_comminit(ip);
    }
}

/**********************************************************************
 * YCOMP (p. 854): cross range Y = (UNIT/R/ . QAXIS) RCO.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_dp_t -- Y, the cross range, 2^24 m
 **********************************************************************/
agc_dp_t agc_lm_ycomp(agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc(); /* ascent erasables */

    /* DOT SL2: the shift takes in MPAC+2 of the DOT's TP product.     */
    agc_dp_t d = agc_tp_to_dp(agc_tp_shift(ip, agc_vec_dot_tp(ip, agc_guid_state()->avg.unitr, a->qaxis), 2)); /* UNIT/R/ . QAXIS SL2, scale 2^0 */

    a->y = agc_dp_mul_trunc(d, a->rco);

    return (a->y);
}

/**********************************************************************
 * LOGSUB (p. 856): -LOG(X) for 0 < X < 1, result scaled 2^5:
 * NORM, ln(1 - u)/32 by POLY, and the shift count times ln(2)/32.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *   x   agc_dp_t        I    X, 0 < X < 1, scale 2^0
 *
 * RETURNS: agc_dp_t -- -LOG(X), scale 2^5
 **********************************************************************/
agc_dp_t agc_lm_logsub(agc_interp_t *ip, agc_dp_t x)
{
    int      n   = 0;                                                          /* NORM shift count (MPAC+6 = -n) */
    agc_dp_t xn  = agc_dp_norm(x, &n);                                         /* NORM: x left-justified, 2^0 */
    agc_dp_t p   = agc_dp_poly(agc_dp_sub(ip, AS_NEARONE, xn), as_logcoef, 7); /* ln(1 - u)/32 by POLY, scale 2^5 */
    agc_tp_t s   = agc_tp_mul_sp(agc_tp_from_dp(AS_CLOG2_32), (agc_sp_t) -n);  /* -n ln(2)/32 as a TP, 2^5 */
    agc_dp_t sum = agc_dp_add(ip, (agc_dp_t) s, p);                            /* DAS MPAC: ln(x)/32, scale 2^5 */

    return (agc_dp_neg(sum)); /* DCOMP        */
}

/* ======================================================================
 * Ascent guidance equations (ASCENT_GUIDANCE pp. 845-849)
 * ====================================================================== */

/**********************************************************************
 * RATES .. PROK (pp. 846-848): PRATE (B) and YRATE (D) from the radius
 * and cross-range errors at TGO; B > 0 is not permitted and B/TBUP is
 * limited to PRLIMIT.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  I/O  interpreter state (OVFIND)
 *   d12  agc_dp_t        I    D12 = TBUP + TGO/L (PDL 2D), 2^17 cs
 **********************************************************************/
static void as_rates(agc_interp_t *ip, agc_dp_t d12)
{
    agc_lm_asc_t *a = agc_lm_asc();     /* ascent erasables */
    agc_guid_t   *g = agc_guid_state(); /* guidance state   */

    agc_dp_t d21 = agc_dp_sub(ip, a->tgo, d12);                                       /* PDL 4D, 2^17 cs  */
    agc_dp_t e   = agc_dp_shl(ip, agc_dp_sub(ip, a->tgo, agc_dp_shl(ip, d21, 1)), 3); /* E = TGO - 2 D21, SL3 (PDL 6D), 2^14 cs */
    agc_dp_t mdr = agc_dp_mul_trunc(a->tgo, g->lm.hdotdisp);                          /* -DR = R + RDOT TGO - RCO, 2^24 m */

    mdr = agc_dp_sub(ip, agc_dp_add(ip, mdr, a->r_mag), a->rco);

    /* DMP, DAD, SL2: DAD leaves the DMP's MPAC+2, which SL2 takes in. */
    agc_tp_t tb = agc_tp_add(ip, agc_dp_mul_tp(a->drdot, d21), agc_tp_from_dp(mdr)); /* 2^24 m */
    agc_dp_t b  = agc_tp_to_dp(agc_tp_shift(ip, tb, 2));                             /* D21 DRDOT - DR, SL2, 2^22 m */

    a->prate = agc_dp_div(ip, agc_dp_div(ip, b, e), a->tgo);

    if (a->prate >= 0)
    {
        a->prate = 0; /* B > 0 not allowed */
    }
    else if (agc_dp_sub(ip, agc_dp_div(ip, agc_dp_shr(a->prate, 4), a->tbup), AS_PRLIMIT) < 0)
    { /* PRLIMIT TBUP, SL4 */
        a->prate = agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(AS_PRLIMIT, a->tbup), 4));
    }

    agc_dp_t mdy = agc_dp_add(ip, agc_dp_mul_trunc(a->tgo, a->ydot), a->y); /* -DY = Y + YDOT TGO - YCO, 2^24 m */

    mdy         = agc_dp_sub(ip, mdy, a->yco);
    agc_tp_t td = agc_tp_add(ip, agc_dp_mul_tp(a->dydot, d21), agc_tp_from_dp(mdy)); /* 2^24 m */
    agc_dp_t d  = agc_tp_to_dp(agc_tp_shift(ip, td, 2));                             /* D21 DYDOT - DY, SL2, 2^22 m */

    a->yrate = agc_dp_div(ip, agc_dp_div(ip, d, e), a->tgo);
} /* as_rates */

/**********************************************************************
 * T2TEST .. CONST (pp. 846-848): with more than 2 s to go, the steering
 * constants A (PCONS) and C (YCONS); B and D are updated only until
 * TGO reaches T3 (FLPC).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 **********************************************************************/
static void as_t2test(agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc(); /* ascent erasables */

    if (agc_dp_sub(ip, a->tgo, AS_T2A) < 0)
    {
        return; /* CMPONENT     */
    }

    agc_dp_t x       = agc_dp_div(ip, agc_dp_sub(ip, a->tbup, a->tgo), a->tbup); /* 1 - TGO/TBUP, scale 2^0 */
    agc_dp_t negl    = agc_dp_shl(ip, agc_lm_logsub(ip, x), 5);                  /* -L in PDL(0), scale 2^0 */
    agc_dp_t d12     = agc_dp_sub(ip, a->tbup, agc_dp_div(ip, a->tgo, negl));    /* D12 = TBUP + TGO/L in PDL(2), 2^17 cs */
    bool     norates = agc_flag_test(AGC_FLAG_FLPC);                             /* FLPC: no position control (B, D frozen) */

    if (!norates && agc_dp_sub(ip, a->tgo, AS_T3) < 0)
    {
        agc_flag_set(AGC_FLAG_FLPC);
        norates = true;
    }

    if (norates)
    {
        a->prate = 0; /* NORATES      */
        a->yrate = 0;
    }
    else
    {
        as_rates(ip, d12);
    }

    agc_dp_t bd = agc_dp_mul_trunc(a->prate, d12); /* CONST: B D12, scale 2^-9 */

    a->pcons    = agc_dp_sub(ip, agc_dp_shr(agc_dp_div(ip, a->drdot, negl), 2), bd);
    agc_dp_t dd = agc_dp_mul_trunc(a->yrate, d12); /* D D12, scale 2^-9 */

    a->ycons = agc_dp_sub(ip, agc_dp_shr(agc_dp_div(ip, a->dydot, negl), 2), dd);
} /* as_t2test */

/**********************************************************************
 * CMPONENT .. AIMER (pp. 848-849): thrust acceleration components one
 * second (100 cs) ahead; ATP takes what is left of AT, with the sign of
 * DZDOT; UNFC/2 = AH + ATP ZAXIS1.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 **********************************************************************/
static void as_cmponent(agc_interp_t *ip)
{
    agc_lm_asc_t   *a  = agc_lm_asc();                                                   /* ascent erasables */
    const agc_vec_t ur = agc_guid_state()->avg.unitr;                                    /* UNIT/R/, half unit 2^1 */
    agc_dp_t        t  = agc_dp_add(ip, agc_dp_mul_trunc(AS_100CS, a->prate), a->pcons); /* A + B (T - T0), scale 2^-9 */

    a->atr       = agc_dp_sub(ip, agc_dp_shl(ip, agc_dp_div(ip, t, a->tbup), 1), a->geff);
    t            = agc_dp_add(ip, agc_dp_mul_trunc(AS_100CS, a->yrate), a->ycons);
    a->aty       = agc_dp_shl(ip, agc_dp_div(ip, t, a->tbup), 1);
    agc_vec_t ah = agc_vec_add(ip, agc_vec_scale(ip, ur, a->atr), agc_vec_scale(ip, a->laxis, a->aty)); /* AH = ATR UNIT/R/ + ATY LAXIS, 2^8 */

    ah             = agc_vec_shl(ip, ah, 1);
    agc_dp_t ahsq  = 0;                     /* |AH|^2 (34D), scale 2^18 */
    agc_dp_t ahmag = agc_vec_abval(ip, ah); /* |AH| (PDL 6), scale 2^9 */

    ahsq          = agc_vec_sq(ip, ah);                     /* 34D */
    agc_dp_t atp2 = agc_dp_sub(ip, agc_dp_sq(a->at), ahsq); /* AT^2 - AH^2 = ATP^2, scale 2^18 */
    agc_dp_t atp  = 0;                                      /* ATP before SIGN, scale 2^-9 m/cs^2 */

    if (atp2 >= 0)
    {
        atp = agc_dp_sqrt(atp2);
    }
    else
    { /* NO-ATP */
        ah = agc_vec_scale(ip, ah, agc_dp_div(ip, a->at, ahmag));
    }

    a->atp      = agc_dp_sign(atp, a->dzdot);                               /* AIMER */
    agc_vec_t f = agc_vec_shl(ip, agc_vec_scale(ip, a->zaxis1, a->atp), 1); /* ATP ZAXIS1, scale 2^-9 m/cs^2 */

    agc_lm_ldg()->unfc_2 = agc_vec_add(ip, f, ah);
} /* as_cmponent */

/**********************************************************************
 * ASCTERM1 +1 (p. 850): N63 display unless RCS trimming or engine fail.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t as_ascterm_display(const agc_job_ctx_t *ctx)
{
    if (agc_flag_test(AGC_FLAG_FLRCS) || agc_flag_test(AGC_FLAG_FLUNDISP))
    {
        return (AGC_STATUS_OK); /* ASCTERM3     */
    }

    const agc_dsky_cont_t cont = {AGC_JOB_ENDOFJOB, AGC_JOB_ENDOFJOB, AGC_JOB_ENDOFJOB};

    return (agc_dsky_display(ctx, AGC_DSKY_GODSPR, agc_dsky_vn(6, 63), &cont));
}

/**********************************************************************
 * ASCTERM (p. 850): FINDCDUW unless trimming with RCS, then displays.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t as_ascterm(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (agc_flag_test(AGC_FLAG_FLRCS))
    {
        return (AGC_STATUS_OK);
    }

    agc_status_t st = agc_lm_findcduw_from_unfc(ip); /* step status */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (as_ascterm_display(ctx));
}

/**********************************************************************
 * KEEPVR1 (p. 852): stay vertical, UNFC/2 = UNIT/R/.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t as_keepvr_radial_thrust(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_ldg()->unfc_2 = agc_guid_state()->avg.unitr;

    return (as_ascterm(ctx, ip));
}

/**********************************************************************
 * MAINLINE .. ANG1CHEK .. CLRXFLAG (pp. 850-851): window toward -UNIT/R/;
 * until TXO the vertical-rise attitude checks.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t as_mainline(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc();     /* ascent erasables */
    agc_guid_t   *g = agc_guid_state(); /* guidance state */

    agc_lm_ldg()->unwc_2 = agc_vec_neg(g->avg.unitr);

    if (agc_dp_sub(ip, a->txo, agc_erase_piptime()) >= 0)
    {
        return (as_ascterm(ctx, ip));
    }

    if (agc_flag_test(AGC_FLAG_ROTFLAG))
    { /* ANG1CHEK */
        const agc_vec_t xnb = g->lm.xnbpip.row[0];                        /* XNBPIP, half unit 2^1 */
        agc_dp_t        c1  = agc_vec_dot(ip, agc_lm_ldg()->unfc_2, xnb); /* UNFC/2 . XNBPIP, scale 2^2 */

        if (agc_dp_sub(ip, c1, agc_lm_pad()->costhet1) < 0 && agc_dp_sub(ip, agc_vec_dot(ip, xnb, g->avg.unitr), agc_lm_pad()->costhet2) < 0)
        {
            return (as_keepvr_radial_thrust(ctx, ip));
        }

        agc_flag_clear(AGC_FLAG_ROTFLAG); /* OFFROT */
    }

    /* NOR29FLG clear: start R29 in the ascent phase */
    agc_flag_clear(AGC_FLAG_NO_POWERED_DESIGNATE);
    agc_flag_clear(AGC_FLAG_XOVINFLG); /* allow X-axis override  */

    return (as_ascterm(ctx, ip));
} /* as_mainline */

/**********************************************************************
 * EXITVR (p. 851): end of the vertical rise; with ROTFLAG clear the
 * rise continues another ten seconds (TXO).
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t as_exitvr(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_flag_clear(AGC_FLAG_FLVR);

    if (!agc_flag_test(AGC_FLAG_ROTFLAG))
    {
        agc_lm_asc()->txo = agc_dp_add(ip, agc_erase_piptime(), AS_10SECS);
    }

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_FLVR)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (as_mainline(ctx, ip));
}

/**********************************************************************
 * ENGOFF (pp. 852-853): engine-off task at TGO, restart protected in
 * group 4 with the variable delay ENGOFFDT; the delta-V monitor off.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- OK, or the WAITLIST status (BAILOUT 1203)
 **********************************************************************/
static agc_status_t as_engoff(agc_interp_t *ip)
{
    agc_lm_asc_t *a  = agc_lm_asc();                                                                               /* ascent erasables */
    agc_dp_t      dt = agc_dp_add(ip, agc_dp_sub(ip, agc_rtb_loadtime(), agc_erase_piptime()), agc_erase_ttogo()); /* DT to engine off, cs 2^28 */
    agc_sp_t      lo = agc_dp_low(agc_dp_neg(dt));                                                                 /* TPAGREE, CCS: low word, cs 2^14 */

    a->engoffdt     = (agc_sp_t) ((lo > 0) ? lo : 1);                                        /* at least 1 bit */
    agc_inhint();
    agc_status_t st = agc_waitlist_call((uint16_t) a->engoffdt, AGC_TASK_ASCENT_ENGINE_OFF); /* step status */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st); /* waitlist full: BAILOUT 1203 */
    }

    const agc_phase_var_t v = {AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_ASCENT_ENGINE_OFF, a->engoffdt, false, 0U};

    agc_phase_change_var(047014, &v);
    agc_relint();
    agc_flag_set(AGC_FLAG_IDLEFLAG); /* DVMON off     */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ASCENT (pp. 845-850). Ends the job (ASCTERM), or for P12 (FLPI) goes
 * on to P12RET and the ignition sequence.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_ascent(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc(); /* ascent erasables */

    /* ASCENT first part (pp. 845-846): the state in the ascent frame, GEFF,
     * the velocity to be gained (compensated for GEFF over TGO) and VGBODY
     * for N85. Returns |VGVECT|.                                        */
    agc_lm_asc_t   *a_vgvect = agc_lm_asc();     /* ascent erasables */
    agc_guid_t     *g        = agc_guid_state(); /* guidance state */
    const agc_vec_t ur       = g->avg.unitr;     /* UNIT/R/, half unit 2^1 */

    a_vgvect->r_mag = agc_vec_abval(ip, g->lm.r);

    /* DOT SL1 (twice below): the shift takes in MPAC+2 of the TP.    */
    a_vgvect->zdot  = agc_tp_to_dp(agc_tp_shift(ip, agc_vec_dot_tp(ip, a_vgvect->zaxis1, g->lm.v), 1));
    a_vgvect->laxis = agc_vec_shl(ip, agc_vec_cross(ip, a_vgvect->zaxis1, ur), 1);
    a_vgvect->ydot  = agc_tp_to_dp(agc_tp_shift(ip, agc_vec_dot_tp(ip, a_vgvect->laxis, g->lm.v), 1));
    a_vgvect->y     = agc_lm_ycomp(ip);                                                                /* YCOMP: Y */
    agc_dp_t gr     = agc_vec_dot(ip, agc_vec_div(ip, g->avg.gdt1_2, AS_2SEC18), ur);                  /* G . UR = GR, scale 2^9 m/cs^2 */
    agc_dp_t h2     = agc_dp_div(ip, agc_vec_sq(ip, agc_vec_cross(ip, ur, g->lm.v)), a_vgvect->r_mag); /* H^2/R^3, scale 2^9 */

    a_vgvect->geff  = agc_dp_add(ip, agc_dp_shl(ip, h2, 1), gr);
    a_vgvect->dzdot = agc_dp_sub(ip, a_vgvect->zdotd, a_vgvect->zdot);
    a_vgvect->dydot = agc_dp_sub(ip, a_vgvect->ydotd, a_vgvect->ydot);
    a_vgvect->drdot = agc_dp_sub(ip, a_vgvect->rdotd, g->lm.hdotdisp);                                                              /* RDOT */
    agc_vec_t vg    = agc_vec_add(ip, agc_vec_scale(ip, ur, a_vgvect->drdot), agc_vec_scale(ip, a_vgvect->laxis, a_vgvect->dydot)); /* DRDOT UR + DYDOT LAXIS, 2^7 m/cs */

    vg          = agc_vec_shl(ip, agc_vec_add(ip, vg, agc_vec_scale(ip, a_vgvect->zaxis1, a_vgvect->dzdot)), 1);
    agc_dp_t tg = agc_dp_mul_trunc(a_vgvect->tgo, a_vgvect->geff); /* TGO GEFF, 2^7 m/cs */

    a_vgvect->vgvect = agc_vec_sub(ip, vg, agc_vec_shl(ip, agc_vec_scale(ip, ur, tg), 1));
    a_vgvect->vgbody = agc_vec_shl(ip, agc_mat_vec(ip, &g->lm.xnbpip, a_vgvect->vgvect), 1);
    agc_dp_t vgmag   = agc_vec_abval(ip, a_vgvect->vgvect); /* |VGVECT|, 2^7 m/cs */

    if (agc_flag_test(AGC_FLAG_FLRCS))
    {
        a->tgo = agc_dp_div(ip, vgmag, AS_ATRCS); /* TGO = VG/AT   */

        return (AGC_STATUS_OK); /* ASCTERM2      */
    }

    /**********************************************************************
     * MAINENG (p. 846): TGO = TBUP (VG/VE)(1 - KT VG/VE) - TTO; TTOGO.
     * DEVIATION: the AGC's DSU TTO (and ATMAG's DSU 6SEC(18) for TBUP)
     * can leave the two words sign-disagreeing; SR 11D shifts each word
     * on its own and every later DDV by TGO or TBUP divides those words.
     * An agc_dp_t holds the value only, so TTOGO may differ by 1 cs and
     * AT, PRATE..YCONS, ATR..ATP, YAW, PITCH by a few LSB (oracle
     * ascent_a/b; the rope run on canonical words matches the C).
     *
     * ARGUMENTS:
     *   ip     agc_interp_t *  I/O  interpreter state (OVFIND)
     *   vgmag  agc_dp_t        I    |VGVECT|, 2^7 m/cs
     **********************************************************************/
    agc_lm_asc_t *a_maineng = agc_lm_asc();                                            /* ascent erasables */
    agc_dp_t      r         = agc_dp_div(ip, vgmag, a_maineng->ve);                    /* VG/VE, scale 2^0 */
    agc_dp_t      t         = agc_dp_sub(ip, AS_NEARONE, agc_dp_mul_trunc(r, AS_KT1)); /* 1 - KT VG/VE, scale 2^0 */

    t              = agc_dp_mul_trunc(agc_dp_mul_trunc(t, r), a_maineng->tbup);
    a_maineng->tgo = agc_dp_sub(ip, t, a_maineng->tto);
    agc_erase_set_ttogo(agc_dp_neg(agc_dp_shr(a_maineng->tgo, 11))); /* 2^28  */

    if (!agc_flag_test(AGC_FLAG_IDLEFLAG) && agc_dp_sub(ip, a->tgo, AS_4SEC17) < 0)
    {
        agc_status_t st_engoff = as_engoff(ip);

        if (st_engoff != AGC_STATUS_OK)
        {
            return (st_engoff);
        }
    }

    as_t2test(ip);
    as_cmponent(ip);

    if (agc_flag_test(AGC_FLAG_FLPI))
    {
        return (agc_lm_powered_ascent_return(ctx, ip));
    }

    if (agc_flag_test(AGC_FLAG_FLVR))
    {
        /**********************************************************************
         * CHECKALT / CHECKYAW / KEEPVR (pp. 850-852): below 25,000 ft keep the
         * vertical rise until the line of sight is within 5 degrees of the
         * YZ plane or RDOT exceeds 40 ft/s.
         *
         * ARGUMENTS:
         *   ctx  const agc_job_ctx_t *  I    executive job context
         *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
         *
         * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
         *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
         **********************************************************************/
        if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_FLVR))) /* in the vertical rise */
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_lm_asc_t *a_checkalt = agc_lm_asc();                                                                        /* ascent erasables */
        agc_guid_t   *g_checkalt = agc_guid_state();                                                                    /* guidance state */
        agc_dp_t      h          = agc_dp_sub(ip, agc_dp_sub(ip, a_checkalt->r_mag, agc_lm_ldg()->land_mag), AS_25KFT); /* /R/MAG - /LAND/ - 25KFT, 2^24 m */

        if (h >= 0)
        {
            return (as_exitvr(ctx, ip));
        }

        agc_flag_set(AGC_FLAG_XOVINFLG);                                                                                                                /* prohibit X override    */
        agc_vec_t los = agc_vec_add(ip, agc_vec_scale(ip, a_checkalt->laxis, a_checkalt->aty), agc_vec_scale(ip, a_checkalt->zaxis1, a_checkalt->atp)); /* ATY LAXIS + ATP ZAXIS1 (LOSVEC), 2^8 */

        los        = agc_vec_unit(ip, los, NULL, NULL);
        agc_dp_t s = agc_dp_abs(agc_vec_dot(ip, los, g_checkalt->lm.xnbpip.row[1])); /* |LOS . YNBPIP|, scale 2^2 */

        if (agc_dp_sub(ip, s, AS_SIN5DEG) < 0 && agc_dp_sub(ip, g_checkalt->lm.hdotdisp, AS_40FPS) >= 0)
        {
            agc_flag_clear(AGC_FLAG_ROTFLAG); /* EXITVR1 */

            return (as_exitvr(ctx, ip));
        }

        agc_lm_ldg()->unwc_2 = los; /* KEEPVR */

        return (as_keepvr_radial_thrust(ctx, ip));
    }

    return (as_mainline(ctx, ip));
} /* agc_lm_ascent */

/**********************************************************************
 * ATMAG (pp. 843-844), SERVICER's exit during ascent: the inverse
 * velocity increments 1/DV0..3 give TBUP and AT; then ASCENT. Below
 * 10% more than lunar gravity, no guidance this cycle (ASCTERM4).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_atmag(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_lm_phaschng(00035);
    agc_guid_lm_t *g = &agc_guid_state()->lm; /* LM guidance state */
    agc_lm_asc_t  *a = agc_lm_asc();          /* ascent erasables */

    if (!agc_flag_test(AGC_FLAG_FLRCS))
    {
        if (agc_dp_sub(&ip, g->abdvconv, AS_MINABDV) < 0)
        {
            agc_inhint();
            agc_lm_dap_zatteror(); /* ASCTERM4      */
            agc_relint();

            return (as_ascterm_display(ctx));
        }

        agc_flag_clear(AGC_FLAG_SURFFLAG);
        agc_flag_clear(AGC_FLAG_RENDWFLG);
        agc_dp_t n = agc_dp_div(&ip, agc_dp_from_sp(AS_BIT3H), g->abdvconv); /* BIT3H / ABDVCONV: new 1/DV, 2^7 cs/m */

        g->mass1   = a->dv[1];                                                                                  /* 1/DV0 = MASS1 */
        a->dv[1]   = a->dv[2];
        a->dv[2]   = a->dv[3];
        a->dv[3]   = n;
        agc_dp_t s = agc_dp_add(&ip, agc_dp_add(&ip, agc_dp_add(&ip, g->mass1, a->dv[1]), a->dv[2]), a->dv[3]); /* 1/DV0 + .. + 1/DV3, scale 2^9 */

        /* DMP DMP SL3: the second product's MPAC+2 enters the shift.    */
        s       = agc_tp_to_dp(agc_tp_shift(&ip, agc_dp_mul_tp(agc_dp_mul_trunc(s, a->ve), AS_2SEC9), 3));
        a->tbup = agc_dp_sub(&ip, agc_dp_add(&ip, agc_dp_shr(a->tbup, 1), s), AS_6SEC18);
        a->at   = agc_dp_div(&ip, agc_dp_shr(a->ve, 1), a->tbup);
    }

    return (agc_lm_ascent(ctx, &ip));
} /* agc_lm_job_atmag */

/* ======================================================================
 * Engine cut-off and termination (pp. 852-854)
 * ====================================================================== */

/**********************************************************************
 * ENGOFF1 (p. 853): shut off the engine; ascent post-burn job CUTOFF.
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_task_ascent_engine_off(void)
{
    agc_status_t st = agc_lm_enginof2(); /* step status */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    st = agc_exec_findvac(AS_PRIO17, AGC_JOB_CUTOFF);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_CUTOFF, AGC_TASK_NONE, AS_PRIO17, false, 0U};

    agc_phase_change_var(07024, &v);

    return (AGC_STATUS_OK); /* TASKOVER      */
}

/**********************************************************************
 * GOFLASH of ASCENT_GUIDANCE.agc (pp. 853-854): a V16 flash whose
 * terminate goes to TERMASC
 *
 * ARGUMENTS:
 *   ctx      const agc_job_ctx_t *  I    executive job context
 *   verb     uint8_t                I    verb (16)
 *   noun     uint8_t                I    noun (63 or 85)
 *   proceed  agc_job_id_t           I    job on PROCEED
 *   enter    agc_job_id_t           I    job on ENTER (recycle)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t as_flash(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t proceed, agc_job_id_t enter)
{
    const agc_dsky_cont_t cont = {AGC_JOB_TERMASC, proceed, enter};

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(verb, noun), &cont));
}

/**********************************************************************
 * CUTOFF (p. 853): RCS trim mode; V16N63 until PROCEED (CUTOFF1).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_cutoff(agc_job_ctx_t *ctx)
{
    agc_flag_set(AGC_FLAG_FLRCS);

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_FLRCS))) /* RCS trim mode */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_lm_job_cutoff_display(ctx));
}

/**********************************************************************
 * CUTOFF -5 (p. 853): the V16N63 flash itself (ENTER repeats it).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_cutoff_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_FLRCS)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (as_flash(ctx, 16, 63, AGC_JOB_CUTOFF_PROCEED, AGC_JOB_CUTOFF_DISPLAY));
}

/**********************************************************************
 * CUTOFF1 (p. 853): zero attitude errors before reducing the deadband.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_cutoff_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_FLRCS))) /* after CUTOFF */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_lm_dap_zatteror();
    agc_status_t st = agc_lm_dap_setmindb(); /* step status */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_relint();

    return (agc_lm_job_cutoff_trim_display(ctx));
}

/**********************************************************************
 * CUTOFF2 (p. 854): V16N85 (VGBODY) until PROCEED or terminate.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_cutoff_trim_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_FLRCS))) /* after CUTOFF */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_CUTOFF_TRIM_DISPLAY, AGC_TASK_NONE, 0, false, 0U };

    agc_phase_change_var(04024, &v);

    return (as_flash(ctx, 16, 85, AGC_JOB_TERMASC, AGC_JOB_CUTOFF_TRIM_DISPLAY));
}

/**********************************************************************
 * TERMASC (p. 854): restore the astronaut's deadband, disallow aborts,
 * and go to P00.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_termasc(void)
{
    const agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_TERMASC, AGC_TASK_NONE, 0, false, 0U };

    agc_phase_change_var(04024, &v);
    agc_inhint();
    agc_status_t st = agc_lm_dap_restordb(); /* step status */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_relint();
    agc_flag_clear(AGC_FLAG_LETABORT);

    return (agc_exec_gotopooh());
}

/* ======================================================================
 * P12: powered ascent (P12.agc pp. 838-840)
 * ====================================================================== */

/**********************************************************************
 * PHASCHNG OCT 04024 (P12.agc, ASCENT_GUIDANCE.agc pp. 838-854):
 * group 4 restarts at this point of this job
 *
 * ARGUMENTS:
 *   job  agc_job_id_t  I    job to restart at
 **********************************************************************/
static void as_restart_here(agc_job_id_t job)
{
    const agc_phase_var_t v = {AGC_RESTART_JOB, job, AGC_TASK_NONE, 0, false, 0U};

    agc_phase_change_var(04024, &v);
}

/**********************************************************************
 * GOFLASH of P12.agc (pp. 838-839): a V06 flash whose terminate goes
 * to GOTOPOOH
 *
 * ARGUMENTS:
 *   ctx      const agc_job_ctx_t *  I    executive job context
 *   noun     uint8_t                I    noun of the V06 flash
 *   proceed  agc_job_id_t           I    job on PROCEED
 *   enter    agc_job_id_t           I    job on ENTER (recycle)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
static agc_status_t as_flash_or_terminate(const agc_job_ctx_t *ctx,
                                          uint8_t              noun,
                                          agc_job_id_t         proceed,
                                          agc_job_id_t         enter)
{
    const agc_dsky_cont_t cont = {AGC_JOB_GOTOPOOH, proceed, enter};

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, noun), &cont));
}

/**********************************************************************
 * P12LM (p. 838): IMU check, flags, DVMON; flash TIG (V06N33).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_powered_ascent(agc_job_ctx_t *ctx)
{
    as_restart_here(AGC_JOB_POWERED_ASCENT);

    if (agc_imu_status_check() != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK);
    }

    agc_flag_set(AGC_FLAG_MUNFLAG);
    agc_flag_set(AGC_FLAG_ACC4_2FL); /* insure 4-jet translation    */

    /* R10FLAG set: no cross-pointer outputs */
    agc_flag_set(AGC_FLAG_ALTITUDE_METERS_ONLY);
    agc_rendz_clradmod();                     /* RADMODES for R29            */
    agc_flag_clear(AGC_FLAG_RNDVZFLG);
    agc_guid_lm_t *g = &agc_guid_state()->lm; /* LM guidance state */

    g->dvthrush = AS_THRESH2;
    g->dvcntr   = 4;
    agc_erase_set_trkmkcnt(0); /* R29 downlink not ready      */

    return (agc_lm_job_powered_ascent_flash(ctx));
}

/**********************************************************************
 * P12 (p. 838): the V06N33 flash (ENTER repeats it).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_powered_ascent_flash(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_MUNFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (as_flash_or_terminate(ctx, 33, AGC_JOB_POWERED_ASCENT_PROCEED, AGC_JOB_POWERED_ASCENT_FLASH));
}

/**********************************************************************
 * After PROCEED (pp. 838-839): WM, /LAND/, engine data, TGO, and the LM
 * state integrated to TIG (LEMPREC; may wait for the lock).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_powered_ascent_proceed(agc_job_ctx_t *ctx)
{
    as_restart_here(AGC_JOB_POWERED_ASCENT_PROCEED);
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_status_t st = agc_lm_guidinit(&ip); /* step status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_set(AGC_FLAG_FLPI);
    agc_lm_powered_ascent_init(&ip);
    agc_lm_asc()->tgo = AS_TGOA; /* initial nominal TGO         */

    return (agc_lm_job_powered_ascent_at_ignition(ctx));
}

/**********************************************************************
 * P12LMB (p. 839): V1S, R, gravity and UNIT/R/ at TIG; XRANGE for N76
 * and the nominal injection targets.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_powered_ascent_at_ignition(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_integ_ctx_t *ic = agc_integ_shared_ctx();                                                          /* shared integration context */
    agc_status_t     st = agc_integ_lemprec(&ip, ic, agc_erase_tig(), AGC_JOB_POWERED_ASCENT_AT_IGNITION); /* step status */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* asleep        */
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_guid_t   *g = agc_guid_state();     /* guidance state */
    agc_lm_asc_t *a = agc_lm_asc();         /* ascent erasables */
    agc_mat_t     m = agc_erase_refsmmat(); /* REFSMMAT, half unit */

    a->v1s        = agc_vec_shl(&ip, agc_mat_vec(&ip, &m, ic->out.vatt), 1);
    g->lm.r       = agc_vec_shl(&ip, agc_mat_vec(&ip, &m, ic->out.ratt), 6);
    g->avg.gdt1_2 = agc_lm_mungrav(&ip, g->lm.r);
    g->avg.unitr  = agc_vec_unit(&ip, g->lm.r, NULL, NULL);
    a->xrange     = agc_dp_neg(agc_dp_shr(agc_lm_ycomp(&ip), 5));
    a->zdotd      = AS_VINJNOM;
    a->rdotd      = AS_RDOTDNOM;
    as_restart_here(AGC_JOB_NEWLOAD);

    return (agc_lm_job_newload(ctx));
}

/**********************************************************************
 * NEWLOAD (p. 839): flash cross range and apolune values (V06N76).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_newload(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_FLPI)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (as_flash_or_terminate(ctx, 76, AGC_JOB_NEWLOAD_PROCEED, AGC_JOB_NEWLOAD));
}

/**********************************************************************
 * After PROCEED on N76 (pp. 839-840): YCO, the velocity at tip-over
 * (V(IGN) + 49 fps UNIT/R/), RDOT, ZAXIS1, then one ASCENT pass.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_newload_proceed(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state (OVFIND) */

    agc_interp_init(&ip);
    agc_guid_t   *g = agc_guid_state(); /* guidance state */
    agc_lm_asc_t *a = agc_lm_asc();     /* ascent erasables */

    g->lm.which        = AGC_LM_WHICH_POWERED_ASCENT;
    as_restart_here(AGC_JOB_NEWLOAD_PROCEED);
    a->yco             = agc_dp_add(&ip, agc_dp_shl(&ip, a->xrange, 5), a->y);
    const agc_vec_t ur = g->avg.unitr; /* UNIT/R/, half unit 2^1 */

    g->lm.v        = agc_vec_add(&ip, agc_vec_scale(&ip, ur, AS_49FPS), a->v1s);
    g->lm.hdotdisp = agc_tp_to_dp(agc_tp_shift(&ip, agc_vec_dot_tp(&ip, g->lm.v, ur), 1)); /* RDOT: DOT SL1 on the TP */
    a->zaxis1      = agc_vec_unit(&ip, agc_vec_cross(&ip, ur, a->qaxis), NULL, NULL);
    agc_flag_set(AGC_FLAG_FLVR);

    return (agc_lm_ascent(ctx, &ip));
}

/**********************************************************************
 * P12RET (p. 840): yaw and pitch of the thrust command for display,
 * one degree deadband, then the ignition sequence (P12BURN).
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    executive job context
 *   ip   agc_interp_t *         I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_powered_ascent_return(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc(); /* ascent erasables */

    /* ATY^2 by DSQ is a TP; DAD adds ATP^2 to its top words and SQRT
     * reads all three (MPAC+2 takes part).                            */
    agc_tp_t s = agc_tp_add(ip, agc_dp_sq_tp(a->aty), agc_tp_from_dp(agc_dp_sq(a->atp))); /* ATY^2 + ATP^2, scale 2^18 (m/cs^2)^2 */

    a->yaw = 0;

    if (!AGC_ASSERT(s >= 0 && s < AGC_TP_MAX / 8))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_tp_to_dp(s) != 0) /* BZE YAWDUN tests MPAC, MPAC+1 only */
    {
        /* SQRT SL1: SQRT leaves the bits of its unnormalising right
         * shift in MPAC+2 and SL1 takes the top one back. SQRT(4 s)
         * has the same normalised root and one shift fewer, so it is
         * the same two words (s < 1/8 keeps 4 s in range).          */
        agc_dp_t root2 = agc_tp_sqrt(agc_tp_shift(ip, s, 2)); /* SQRT(4 s) = 2 sqrt(s), scale 2^9 */
        agc_dp_t h     = agc_dp_div(ip, a->aty, root2);       /* ATY/|AT horizontal|, scale 2^1 */

        a->yaw = agc_dp_asin(ip, h); /* revolutions   */
    }

    agc_vec_t u = agc_vec_unit(ip, agc_lm_ldg()->unfc_2, NULL, NULL);                                    /* UNIT(UNFC/2), half unit 2^1 */
    agc_dp_t  c = agc_tp_to_dp(agc_tp_shift(ip, agc_vec_dot_tp(ip, u, agc_guid_state()->avg.unitr), 1)); /* UNIT(UNFC/2) . UNIT/R/ SL1, scale 2^1; DOT SL1 on the TP */

    a->pitch         = agc_dp_neg(agc_dp_acos(ip, c));
    as_restart_here(AGC_JOB_POWERED_ASCENT_BURN);
    agc_job_ctx_t c2 = *ctx; /* job context copy */

    return (agc_lm_job_powered_ascent_burn(&c2));
} /* agc_lm_powered_ascent_return */

/**********************************************************************
 * P12RET (p. 840) after its phase change: FLPI off, deadband, BURNBABY.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    executive job context
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_job_powered_ascent_burn(agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_FLPI);
    agc_inhint();
    agc_status_t st = agc_lm_dap_pflitedb(); /* step status */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    agc_relint();

    return (agc_lm_burnbaby(ctx));
}
