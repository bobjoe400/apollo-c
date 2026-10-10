/*
 * ascent.c -- P12.agc (Luminary 099, pp. 838-842) and ASCENT_GUIDANCE.agc
 * (pp. 843-856): powered ascent from the lunar surface (P12), the ascent
 * guidance equations used by P12, P70 and P71 (ATMAG, ASCENT), engine
 * cut-off (ENGOFF..TERMASC), the cross-range YCOMP and the logarithm
 * LOGSUB.
 *
 * The ascent guidance is a linear-tangent-like law in the frame of
 * UNIT/R/ (radial), LAXIS (cross range) and ZAXIS1 (downrange): the
 * thrust acceleration components ATR = (A + B t)/TBUP - GEFF and
 * ATY = (C + D t)/TBUP, ATP from the remaining magnitude.
 *
 * Scales: R 2^24 m, V 2^7 m/cs, TGO TBUP TTO 2^17 cs, accelerations
 * 2^-9 m/cs^2 unless stated; the push-down list holds -L (0D), D12 (2D),
 * D21 (4D) and E (6D) in the AGC and C locals here.
 */
#include "agc/agc_assert.h"
#include "agc/agc_channels.h"
#include "agc/agc_lm_landing.h"
#include "landing_pending.h"
#include "../io/sysmisc.h"

/* ---- constants (pp. 842, 836, 852, 855-856; CONTROLLED_CONSTANTS) ----- */
#define AS_THRESH2   308
#define AS_TGOA      ((agc_dp_t)75776000)  /* (TGO)A 2DEC 37000 B-17       */
#define AS_1DVA      ((agc_dp_t)31876710)  /* (1/DV)A 2DEC 15.20 B-7       */
#define AS_ATA       ((agc_dp_t)45194051)  /* (AT)A 2DEC 3.2883 E-4 B9     */
#define AS_TBUPA     ((agc_dp_t)188215296) /* (TBUP)A 2DEC 91902 B-17      */
#define AS_ATDECAY   ((agc_dp_t)-10)       /* ATDECAY 2DEC -10             */
#define AS_APSVEX    (-15514)              /* APSVEX DEC -3030 E-2 B-5     */
#define AS_HINJECT   ((agc_dp_t)292608)    /* 2DEC 18288 B-24: 60,000 ft   */
#define AS_MOONRATE  ((agc_dp_t)3746009)   /* 2DEC* .26616994890062991 E-7 B+19 */
#define AS_49FPS     ((agc_dp_t)626428)    /* 2DEC .149352 B-6             */
#define AS_VINJNOM   ((agc_dp_t)35216215)  /* 2DEC 16.7924 B-7: 5509.5 fps */
#define AS_RDOTDNOM  ((agc_dp_t)124646)    /* 2DEC .059436 B-7: 19.5 fps   */
#define AS_ATRCS     ((agc_dp_t)21577916)  /* AT/RCS 2DEC .0000785 B+10    */
#define AS_KT1       ((agc_dp_t)134217728) /* KT1 2DEC 0.5000              */
#define AS_NEARONE   AGC_DP_MAX            /* NEARONE 2DEC .999999999      */
#define AS_4SEC17    ((agc_dp_t)819200)    /* 2DEC 400 B-17                */
#define AS_T2A       ((agc_dp_t)409600)    /* T2A = 2SEC(17)               */
#define AS_T3        ((agc_dp_t)2048000)   /* T3 2DEC 1000 B-17            */
#define AS_100CS     ((agc_dp_t)204800)    /* 100CS = 2SEC(18)             */
#define AS_2SEC18    ((agc_dp_t)204800)    /* 2DEC 200 B-18                */
#define AS_2SEC9     ((agc_dp_t)104857600) /* 2DEC 200 B-9                 */
#define AS_6SEC18    ((agc_dp_t)614400)    /* 2DEC 600 B-18                */
#define AS_PRLIMIT   ((agc_dp_t)-17153026) /* 2DEC -.0639                  */
#define AS_MINABDV   ((agc_dp_t)298634)    /* 2DEC .0356 B-5               */
#define AS_BIT3H     4                     /* BIT3H OCT 4                  */
#define AS_25KFT     ((agc_dp_t)121920)    /* 2DEC 7620 B-24               */
#define AS_10SECS    ((agc_dp_t)1000)      /* 2DEC 1000                    */
#define AS_SIN5DEG   ((agc_dp_t)5849209)   /* 2DEC 0.08716 B-2             */
#define AS_40FPS     ((agc_dp_t)255685)    /* 2DEC 0.12192 B-7             */
#define AS_CLOG2_32  ((agc_dp_t)5814540)   /* 2DEC .0216608494             */
#define AS_PRIO17    017

/* LOGSUB polynomial (p. 856), -LOG via ln(1 - u)/32. */
static const agc_dp_t as_logcoef[8] = {
    2, -8388994, -4179185, -3019954, -497816, -7693572, 10350833, -11257160
};

/* ======================================================================
 * Initialisation (P12.agc pp. 840-842)
 * ====================================================================== */

/* GUIDINIT (pp. 841-842): WM = MOONRATE x (moon's pole in stable member
 * axes now), /LAND/ = |RLS| (2^24 m).                                */
agc_status_t agc_lm_guidinit(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_navaux_padload_t nav;
    agc_status_t st = agc_navaux_get_padload(&nav);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    agc_vec_t pole = agc_vec_zero();
    st = agc_planet_rp_to_r(ip, &nav, true, agc_vec_make(0, 0, AGC_DP_HALF),
                            agc_rtb_loadtime(), &pole);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    agc_mat_t m = agc_erase_refsmmat();
    agc_lm_ldg_t *l = agc_lm_ldg();
    l->wm = agc_vec_scale(ip, agc_mat_vec(ip, &m, pole), AS_MOONRATE);
    l->land_mag = agc_dp_shl(ip, agc_vec_abval(ip, agc_erase_rls()), 3);
    return AGC_STATUS_OK;
}

/* COMMINIT (p. 841): targets for P12, P70 and P71 not following P70:
 * RCO = /LAND/ + 60,000 ft; QAXIS = UNIT(REFSMMAT (VCSM x RCSM)).    */
agc_status_t agc_lm_comminit(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_lm_asc_t *a = agc_lm_asc();
    a->rco = agc_dp_add(ip, AS_HINJECT, agc_lm_ldg()->land_mag);
    a->txo = 0;
    a->yco = 0;
    a->rdotd = 0;
    a->ydotd = 0;
    agc_mat_t m = agc_erase_refsmmat();
    agc_vec_t h = agc_vec_cross(ip, agc_erase_vrectcsm(), agc_erase_rrectcsm());
    a->qaxis = agc_vec_unit(ip, agc_mat_vec(ip, &m, h), NULL, NULL);
    (void)AGC_ASSERT(a->rco > 0);
    return AGC_STATUS_OK;
}

/* P12INIT (pp. 840-841): APS engine data for P12 and P71; COMMINIT too
 * unless FLAP (P71 after P70 keeps the targets).                     */
agc_status_t agc_lm_powered_ascent_init(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_lm_asc_t *a = agc_lm_asc();
    a->dv[3] = AS_1DVA;
    a->dv[2] = AS_1DVA;
    a->dv[1] = AS_1DVA;
    a->at = AS_ATA;
    a->tbup = AS_TBUPA;
    a->tto = agc_dp_shl(ip, agc_dp_neg(AS_ATDECAY), 11);
    a->ve = agc_dp_shr(agc_dp_from_sp((agc_sp_t)-AS_APSVEX), 2);
    if (agc_flag_test(AGC_FLAG_FLAP)) {
        return AGC_STATUS_OK;                             /* RVQ          */
    }
    return agc_lm_comminit(ip);
}

/* YCOMP (p. 854): cross range Y = (UNIT/R/ . QAXIS) RCO. */
agc_dp_t agc_lm_ycomp(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return 0;
    }
    agc_lm_asc_t *a = agc_lm_asc();
    agc_dp_t d = agc_dp_shl(ip, agc_vec_dot(ip, agc_guid_state()->avg.unitr,
                                            a->qaxis), 2);
    a->y = agc_dp_mul_trunc(d, a->rco);
    (void)AGC_ASSERT(a->rco >= 0);
    return a->y;
}

/* LOGSUB (p. 856): -LOG(X) for 0 < X < 1, result scaled 2^5:
 * NORM, ln(1 - u)/32 by POLY, and the shift count times ln(2)/32.    */
agc_dp_t agc_lm_logsub(agc_interp_t *ip, agc_dp_t x)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(x > 0)) {
        return 0;
    }
    int n = 0;
    agc_dp_t xn = agc_dp_norm(x, &n);                    /* MPAC+6 = -n  */
    agc_dp_t p = agc_dp_poly(agc_dp_sub(ip, AS_NEARONE, xn), as_logcoef, 7);
    agc_tp_t s = agc_tp_mul_sp(agc_tp_from_dp(AS_CLOG2_32), (agc_sp_t)-n);
    agc_dp_t sum = agc_dp_add(ip, (agc_dp_t)s, p);       /* DAS MPAC     */
    (void)AGC_ASSERT(n >= 0 && n < 28);
    return agc_dp_neg(sum);                              /* DCOMP        */
}

/* ======================================================================
 * Ascent guidance equations (ASCENT_GUIDANCE pp. 845-849)
 * ====================================================================== */

/* ASCENT first part (pp. 845-846): the state in the ascent frame, GEFF,
 * the velocity to be gained (compensated for GEFF over TGO) and VGBODY
 * for N85. Returns |VGVECT|.                                        */
static agc_dp_t as_vgvect(agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc();
    agc_guid_t *g = agc_guid_state();
    const agc_vec_t ur = g->avg.unitr;
    a->r_mag = agc_vec_abval(ip, g->lm.r);
    a->zdot = agc_dp_shl(ip, agc_vec_dot(ip, a->zaxis1, g->lm.v), 1);
    a->laxis = agc_vec_shl(ip, agc_vec_cross(ip, a->zaxis1, ur), 1);
    a->ydot = agc_dp_shl(ip, agc_vec_dot(ip, a->laxis, g->lm.v), 1);
    (void)agc_lm_ycomp(ip);
    agc_dp_t gr = agc_vec_dot(ip, agc_vec_div(ip, g->avg.gdt1_2, AS_2SEC18), ur);
    agc_dp_t h2 = agc_dp_div(ip, agc_vec_sq(ip, agc_vec_cross(ip, ur, g->lm.v)),
                             a->r_mag);
    a->geff = agc_dp_add(ip, agc_dp_shl(ip, h2, 1), gr);
    a->dzdot = agc_dp_sub(ip, a->zdotd, a->zdot);
    a->dydot = agc_dp_sub(ip, a->ydotd, a->ydot);
    a->drdot = agc_dp_sub(ip, a->rdotd, g->lm.hdotdisp);       /* RDOT */
    agc_vec_t vg = agc_vec_add(ip, agc_vec_scale(ip, ur, a->drdot),
                               agc_vec_scale(ip, a->laxis, a->dydot));
    vg = agc_vec_shl(ip, agc_vec_add(ip, vg, agc_vec_scale(ip, a->zaxis1, a->dzdot)), 1);
    agc_dp_t tg = agc_dp_mul_trunc(a->tgo, a->geff);
    a->vgvect = agc_vec_sub(ip, vg, agc_vec_shl(ip, agc_vec_scale(ip, ur, tg), 1));
    a->vgbody = agc_vec_shl(ip, agc_mat_vec(ip, &g->lm.xnbpip, a->vgvect), 1);
    return agc_vec_abval(ip, a->vgvect);
}

/* MAINENG (p. 846): TGO = TBUP (VG/VE)(1 - KT VG/VE) - TTO; TTOGO. */
static void as_maineng(agc_interp_t *ip, agc_dp_t vgmag)
{
    agc_lm_asc_t *a = agc_lm_asc();
    agc_dp_t r = agc_dp_div(ip, vgmag, a->ve);
    agc_dp_t t = agc_dp_sub(ip, AS_NEARONE, agc_dp_mul_trunc(r, AS_KT1));
    t = agc_dp_mul_trunc(agc_dp_mul_trunc(t, r), a->tbup);
    a->tgo = agc_dp_sub(ip, t, a->tto);
    agc_erase_set_ttogo(agc_dp_neg(agc_dp_shr(a->tgo, 11)));    /* 2^28  */
    (void)AGC_ASSERT(a->ve != 0);
    (void)AGC_ASSERT(a->tbup >= 0);
}

/* RATES .. PROK (pp. 846-848): PRATE (B) and YRATE (D) from the radius
 * and cross-range errors at TGO; B > 0 is not permitted and B/TBUP is
 * limited to PRLIMIT. d12 = TBUP + TGO/L.                           */
static void as_rates(agc_interp_t *ip, agc_dp_t d12)
{
    agc_lm_asc_t *a = agc_lm_asc();
    agc_guid_t *g = agc_guid_state();
    agc_dp_t d21 = agc_dp_sub(ip, a->tgo, d12);                    /* 4D */
    agc_dp_t e = agc_dp_shl(ip, agc_dp_sub(ip, a->tgo, agc_dp_shl(ip, d21, 1)), 3);
    agc_dp_t mdr = agc_dp_mul_trunc(a->tgo, g->lm.hdotdisp);
    mdr = agc_dp_sub(ip, agc_dp_add(ip, mdr, a->r_mag), a->rco);   /* -DR */
    agc_dp_t b = agc_dp_shl(ip, agc_dp_add(ip, agc_dp_mul_trunc(a->drdot, d21),
                                           mdr), 2);
    a->prate = agc_dp_div(ip, agc_dp_div(ip, b, e), a->tgo);
    if (a->prate >= 0) {
        a->prate = 0;                                  /* B > 0 not allowed */
    } else if (agc_dp_sub(ip, agc_dp_div(ip, agc_dp_shr(a->prate, 4), a->tbup),
                          AS_PRLIMIT) < 0) {
        a->prate = agc_dp_shl(ip, agc_dp_mul_trunc(AS_PRLIMIT, a->tbup), 4);
    }
    agc_dp_t mdy = agc_dp_add(ip, agc_dp_mul_trunc(a->tgo, a->ydot), a->y);
    mdy = agc_dp_sub(ip, mdy, a->yco);                              /* -DY */
    agc_dp_t d = agc_dp_shl(ip, agc_dp_add(ip, agc_dp_mul_trunc(a->dydot, d21),
                                           mdy), 2);
    a->yrate = agc_dp_div(ip, agc_dp_div(ip, d, e), a->tgo);
    (void)AGC_ASSERT(a->tgo > 0);
}

/* T2TEST .. CONST (pp. 846-848): with more than 2 s to go, the steering
 * constants A (PCONS) and C (YCONS); B and D are updated only until
 * TGO reaches T3 (FLPC).                                            */
static void as_t2test(agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc();
    if (agc_dp_sub(ip, a->tgo, AS_T2A) < 0) {
        return;                                          /* CMPONENT     */
    }
    agc_dp_t x = agc_dp_div(ip, agc_dp_sub(ip, a->tbup, a->tgo), a->tbup);
    agc_dp_t negl = agc_dp_shl(ip, agc_lm_logsub(ip, x), 5);       /* 0D */
    agc_dp_t d12 = agc_dp_sub(ip, a->tbup, agc_dp_div(ip, a->tgo, negl));
    bool norates = agc_flag_test(AGC_FLAG_FLPC);
    if (!norates && agc_dp_sub(ip, a->tgo, AS_T3) < 0) {
        (void)agc_flag_set(AGC_FLAG_FLPC);
        norates = true;
    }
    if (norates) {
        a->prate = 0;                                    /* NORATES      */
        a->yrate = 0;
    } else {
        as_rates(ip, d12);
    }
    agc_dp_t bd = agc_dp_mul_trunc(a->prate, d12);                  /* CONST */
    a->pcons = agc_dp_sub(ip, agc_dp_shr(agc_dp_div(ip, a->drdot, negl), 2), bd);
    agc_dp_t dd = agc_dp_mul_trunc(a->yrate, d12);
    a->ycons = agc_dp_sub(ip, agc_dp_shr(agc_dp_div(ip, a->dydot, negl), 2), dd);
    (void)AGC_ASSERT(negl != 0);
}

/* CMPONENT .. AIMER (pp. 848-849): thrust acceleration components one
 * second (100 cs) ahead; ATP takes what is left of AT, with the sign of
 * DZDOT; UNFC/2 = AH + ATP ZAXIS1.                                  */
static void as_cmponent(agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc();
    const agc_vec_t ur = agc_guid_state()->avg.unitr;
    agc_dp_t t = agc_dp_add(ip, agc_dp_mul_trunc(AS_100CS, a->prate), a->pcons);
    a->atr = agc_dp_sub(ip, agc_dp_shl(ip, agc_dp_div(ip, t, a->tbup), 1), a->geff);
    t = agc_dp_add(ip, agc_dp_mul_trunc(AS_100CS, a->yrate), a->ycons);
    a->aty = agc_dp_shl(ip, agc_dp_div(ip, t, a->tbup), 1);
    agc_vec_t ah = agc_vec_add(ip, agc_vec_scale(ip, ur, a->atr),
                               agc_vec_scale(ip, a->laxis, a->aty));
    ah = agc_vec_shl(ip, ah, 1);
    agc_dp_t ahsq = 0;
    agc_dp_t ahmag = agc_vec_abval(ip, ah);
    ahsq = agc_vec_sq(ip, ah);                                     /* 34D */
    agc_dp_t atp2 = agc_dp_sub(ip, agc_dp_sq(ip, a->at), ahsq);
    agc_dp_t atp = 0;
    if (atp2 >= 0) {
        atp = agc_dp_sqrt(ip, atp2);
    } else {                                                       /* NO-ATP */
        ah = agc_vec_scale(ip, ah, agc_dp_div(ip, a->at, ahmag));
    }
    a->atp = agc_dp_sign(atp, a->dzdot);                           /* AIMER */
    agc_vec_t f = agc_vec_shl(ip, agc_vec_scale(ip, a->zaxis1, a->atp), 1);
    agc_lm_ldg()->unfc_2 = agc_vec_add(ip, f, ah);
    (void)AGC_ASSERT(a->tbup != 0);
}

/* ASCTERM1 +1 (p. 850): N63 display unless RCS trimming or engine fail. */
static agc_status_t as_ascterm1(const agc_job_ctx_t *ctx)
{
    if (agc_flag_test(AGC_FLAG_FLRCS) || agc_flag_test(AGC_FLAG_FLUNDISP)) {
        return AGC_STATUS_OK;                            /* ASCTERM3     */
    }
    const agc_dsky_cont_t cont = { AGC_JOB_ENDOFJOB, AGC_JOB_ENDOFJOB,
                                   AGC_JOB_ENDOFJOB };
    (void)AGC_ASSERT(ctx != NULL);
    return agc_dsky_display(ctx, AGC_DSKY_GODSPR, agc_dsky_vn(6, 63), &cont);
}

/* ASCTERM (p. 850): FINDCDUW unless trimming with RCS, then displays. */
static agc_status_t as_ascterm(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (agc_flag_test(AGC_FLAG_FLRCS)) {
        return AGC_STATUS_OK;
    }
    agc_status_t st = agc_lm_findcduw_m2(ip);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return as_ascterm1(ctx);
}

/* KEEPVR1 (p. 852): stay vertical, UNFC/2 = UNIT/R/. */
static agc_status_t as_keepvr1(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_ldg()->unfc_2 = agc_guid_state()->avg.unitr;
    (void)AGC_ASSERT(ip != NULL);
    return as_ascterm(ctx, ip);
}

/* MAINLINE .. ANG1CHEK .. CLRXFLAG (pp. 850-851): window toward -UNIT/R/;
 * until TXO the vertical-rise attitude checks.                       */
static agc_status_t as_mainline(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc();
    agc_guid_t *g = agc_guid_state();
    agc_lm_ldg()->unwc_2 = agc_vec_neg(g->avg.unitr);
    if (agc_dp_sub(ip, a->txo, agc_erase_piptime()) >= 0) {
        return as_ascterm(ctx, ip);
    }
    if (agc_flag_test(AGC_FLAG_ROTFLAG)) {                         /* ANG1CHEK */
        const agc_vec_t xnb = g->lm.xnbpip.row[0];
        agc_dp_t c1 = agc_vec_dot(ip, agc_lm_ldg()->unfc_2, xnb);
        if (agc_dp_sub(ip, c1, agc_lm_pad()->costhet1) < 0 &&
            agc_dp_sub(ip, agc_vec_dot(ip, xnb, g->avg.unitr),
                       agc_lm_pad()->costhet2) < 0) {
            return as_keepvr1(ctx, ip);
        }
        (void)agc_flag_clear(AGC_FLAG_ROTFLAG);                    /* OFFROT */
    }
    (void)agc_flag_clear(AGC_FLAG_NOR29FLG);    /* start R29 in ascent    */
    (void)agc_flag_clear(AGC_FLAG_XOVINFLG);    /* allow X-axis override  */
    return as_ascterm(ctx, ip);
}

/* EXITVR (p. 851): end of the vertical rise; with ROTFLAG clear the
 * rise continues another ten seconds (TXO).                         */
static agc_status_t as_exitvr(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    (void)agc_flag_clear(AGC_FLAG_FLVR);
    if (!agc_flag_test(AGC_FLAG_ROTFLAG)) {
        agc_lm_asc()->txo = agc_dp_add(ip, agc_erase_piptime(), AS_10SECS);
    }
    (void)AGC_ASSERT(!agc_flag_test(AGC_FLAG_FLVR));
    return as_mainline(ctx, ip);
}

/* CHECKALT / CHECKYAW / KEEPVR (pp. 850-852): below 25,000 ft keep the
 * vertical rise until the line of sight is within 5 degrees of the
 * YZ plane or RDOT exceeds 40 ft/s.                                 */
static agc_status_t as_checkalt(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc();
    agc_guid_t *g = agc_guid_state();
    agc_dp_t h = agc_dp_sub(ip, agc_dp_sub(ip, a->r_mag, agc_lm_ldg()->land_mag),
                            AS_25KFT);
    if (h >= 0) {
        return as_exitvr(ctx, ip);
    }
    (void)agc_flag_set(AGC_FLAG_XOVINFLG);      /* prohibit X override    */
    agc_vec_t los = agc_vec_add(ip, agc_vec_scale(ip, a->laxis, a->aty),
                                agc_vec_scale(ip, a->zaxis1, a->atp));
    los = agc_vec_unit(ip, los, NULL, NULL);
    agc_dp_t s = agc_dp_abs(agc_vec_dot(ip, los, g->lm.xnbpip.row[1]));
    if (agc_dp_sub(ip, s, AS_SIN5DEG) < 0 &&
        agc_dp_sub(ip, g->lm.hdotdisp, AS_40FPS) >= 0) {
        (void)agc_flag_clear(AGC_FLAG_ROTFLAG);                    /* EXITVR1 */
        return as_exitvr(ctx, ip);
    }
    agc_lm_ldg()->unwc_2 = los;                                    /* KEEPVR */
    return as_keepvr1(ctx, ip);
}

/* ENGOFF (pp. 852-853): engine-off task at TGO, restart protected in
 * group 4 with the variable delay ENGOFFDT; the delta-V monitor off.  */
static void as_engoff(agc_interp_t *ip)
{
    agc_lm_asc_t *a = agc_lm_asc();
    agc_dp_t dt = agc_dp_add(ip, agc_dp_sub(ip, agc_rtb_loadtime(),
                                            agc_erase_piptime()), agc_erase_ttogo());
    agc_sp_t lo = agc_dp_low(agc_dp_neg(dt));            /* TPAGREE, CCS  */
    a->engoffdt = (agc_sp_t)((lo > 0) ? lo : 1);         /* at least 1 bit */
    agc_inhint();
    agc_status_t st = agc_waitlist_call((uint16_t)a->engoffdt, AGC_TASK_ENGOFF1);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    const agc_phase_var_t v = { AGC_RESTART_TASK, AGC_JOB_NONE, AGC_TASK_ENGOFF1,
                                a->engoffdt, false, 0U };
    st = agc_phase_change_var(047014, &v);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_relint();
    (void)agc_flag_set(AGC_FLAG_IDLEFLAG);               /* DVMON off     */
}

/* ASCENT (pp. 845-850). Ends the job (ASCTERM), or for P12 (FLPI) goes
 * on to P12RET and the ignition sequence.                           */
agc_status_t agc_lm_ascent(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_lm_asc_t *a = agc_lm_asc();
    agc_dp_t vgmag = as_vgvect(ip);
    if (agc_flag_test(AGC_FLAG_FLRCS)) {
        a->tgo = agc_dp_div(ip, vgmag, AS_ATRCS);        /* TGO = VG/AT   */
        return AGC_STATUS_OK;                            /* ASCTERM2      */
    }
    as_maineng(ip, vgmag);
    if (!agc_flag_test(AGC_FLAG_IDLEFLAG) &&
        agc_dp_sub(ip, a->tgo, AS_4SEC17) < 0) {
        as_engoff(ip);
    }
    as_t2test(ip);
    as_cmponent(ip);
    if (agc_flag_test(AGC_FLAG_FLPI)) {
        return agc_lm_powered_ascent_ret(ctx, ip);
    }
    if (agc_flag_test(AGC_FLAG_FLVR)) {
        return as_checkalt(ctx, ip);
    }
    return as_mainline(ctx, ip);
}

/* ATMAG (pp. 843-844), SERVICER's exit during ascent: the inverse
 * velocity increments 1/DV0..3 give TBUP and AT; then ASCENT. Below
 * 10% more than lunar gravity, no guidance this cycle (ASCTERM4).   */
agc_status_t agc_lm_job_atmag(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    (void)agc_lm_phaschng(00035);
    agc_guid_lm_t *g = &agc_guid_state()->lm;
    agc_lm_asc_t *a = agc_lm_asc();
    if (!agc_flag_test(AGC_FLAG_FLRCS)) {
        if (agc_dp_sub(&ip, g->abdvconv, AS_MINABDV) < 0) {
            agc_inhint();
            (void)agc_lm_dap_zatteror();                 /* ASCTERM4      */
            agc_relint();
            return as_ascterm1(ctx);
        }
        (void)agc_flag_clear(AGC_FLAG_SURFFLAG);
        (void)agc_flag_clear(AGC_FLAG_RENDWFLG);
        agc_dp_t n = agc_dp_div(&ip, agc_dp_from_sp(AS_BIT3H), g->abdvconv);
        g->mass1 = a->dv[1];                             /* 1/DV0 = MASS1 */
        a->dv[1] = a->dv[2];
        a->dv[2] = a->dv[3];
        a->dv[3] = n;
        agc_dp_t s = agc_dp_add(&ip, agc_dp_add(&ip, agc_dp_add(&ip, g->mass1,
                                a->dv[1]), a->dv[2]), a->dv[3]);
        s = agc_dp_shl(&ip, agc_dp_mul_trunc(agc_dp_mul_trunc(s, a->ve), AS_2SEC9), 3);
        a->tbup = agc_dp_sub(&ip, agc_dp_add(&ip, agc_dp_shr(a->tbup, 1), s), AS_6SEC18);
        a->at = agc_dp_div(&ip, agc_dp_shr(a->ve, 1), a->tbup);
    }
    return agc_lm_ascent(ctx, &ip);
}

/* ======================================================================
 * Engine cut-off and termination (pp. 852-854)
 * ====================================================================== */

/* ENGOFF1 (p. 853): shut off the engine; ascent post-burn job CUTOFF. */
agc_status_t agc_lm_task_engoff1(void)
{
    agc_status_t st = agc_lm_enginof2();
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED);
    st = agc_exec_findvac(AS_PRIO17, AGC_JOB_CUTOFF);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    const agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_CUTOFF, AGC_TASK_NONE,
                                AS_PRIO17, false, 0U };
    st = agc_phase_change_var(07024, &v);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return AGC_STATUS_OK;                                /* TASKOVER      */
}

/* Flash with terminate -> TERMASC. */
static agc_status_t as_flash(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun,
                             agc_job_id_t proceed, agc_job_id_t enter)
{
    const agc_dsky_cont_t cont = { AGC_JOB_TERMASC, proceed, enter };
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(verb == 16);
    return agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(verb, noun), &cont);
}

/* CUTOFF (p. 853): RCS trim mode; V16N63 until PROCEED (CUTOFF1). */
agc_status_t agc_lm_job_cutoff(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_flag_set(AGC_FLAG_FLRCS);
    return agc_lm_job_cutoff5(ctx);
}

/* CUTOFF -5: the V16N63 flash itself (ENTER repeats it). */
agc_status_t agc_lm_job_cutoff5(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_FLRCS));
    return as_flash(ctx, 16, 63, AGC_JOB_CUTOFF1, AGC_JOB_CUTOFF5);
}

/* CUTOFF1 (p. 853): zero attitude errors before reducing the deadband. */
agc_status_t agc_lm_job_cutoff1(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    (void)agc_lm_dap_zatteror();
    (void)agc_lm_dap_setmindb();
    agc_relint();
    return agc_lm_job_cutoff2(ctx);
}

/* CUTOFF2 (p. 854): V16N85 (VGBODY) until PROCEED or terminate. */
agc_status_t agc_lm_job_cutoff2(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_CUTOFF2, AGC_TASK_NONE,
                                0, false, 0U };
    agc_status_t st = agc_phase_change_var(04024, &v);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return as_flash(ctx, 16, 85, AGC_JOB_TERMASC, AGC_JOB_CUTOFF2);
}

/* TERMASC (p. 854): restore the astronaut's deadband, disallow aborts,
 * and go to P00.                                                    */
agc_status_t agc_lm_job_termasc(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_TERMASC, AGC_TASK_NONE,
                                0, false, 0U };
    agc_status_t st = agc_phase_change_var(04024, &v);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_inhint();
    (void)agc_lm_dap_restordb();
    agc_relint();
    (void)agc_flag_clear(AGC_FLAG_LETABORT);
    return agc_exec_gotopooh();
}

/* ======================================================================
 * P12: powered ascent (P12.agc pp. 838-840)
 * ====================================================================== */

/* Group 4 restart at this job (PHASCHNG OCT 04024). */
static void as_restart_here(agc_job_id_t job)
{
    const agc_phase_var_t v = { AGC_RESTART_JOB, job, AGC_TASK_NONE, 0,
                                false, 0U };
    agc_status_t st = agc_phase_change_var(04024, &v);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(job != AGC_JOB_NONE);
}

/* Flash with terminate -> GOTOPOOH. */
static agc_status_t as_flash00(const agc_job_ctx_t *ctx, uint8_t noun,
                               agc_job_id_t proceed, agc_job_id_t enter)
{
    const agc_dsky_cont_t cont = { AGC_JOB_GOTOPOOH, proceed, enter };
    (void)AGC_ASSERT(ctx != NULL);
    (void)AGC_ASSERT(noun == 33 || noun == 76);
    return agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, noun), &cont);
}

/* P12LM (p. 838): IMU check, flags, DVMON; flash TIG (V06N33). */
agc_status_t agc_lm_job_powered_ascent_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    as_restart_here(AGC_JOB_POWERED_ASCENT_LM);
    if (agc_imu_status_check_both() != AGC_STATUS_OK) {
        return AGC_STATUS_OK;
    }
    (void)agc_flag_set(AGC_FLAG_MUNFLAG);
    (void)agc_flag_set(AGC_FLAG_ACC4_2FL);  /* insure 4-jet translation    */
    (void)agc_flag_set(AGC_FLAG_LANDING_ANALOG_DISPLAYS_FLAG);   /* no cross-pointer outputs    */
    (void)agc_rendz_clradmod();             /* RADMODES for R29            */
    (void)agc_flag_clear(AGC_FLAG_RNDVZFLG);
    agc_guid_lm_t *g = &agc_guid_state()->lm;
    g->dvthrush = AS_THRESH2;
    g->dvcntr = 4;
    agc_erase_set_trkmkcnt(0);              /* R29 downlink not ready      */
    return agc_lm_job_powered_ascent_flash(ctx);
}

/* V06N33 flash (ENTER repeats it). */
agc_status_t agc_lm_job_powered_ascent_flash(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_MUNFLAG));
    return as_flash00(ctx, 33, AGC_JOB_POWERED_ASCENT_LM2, AGC_JOB_POWERED_ASCENT_FLASH);
}

/* After PROCEED (pp. 838-839): WM, /LAND/, engine data, TGO, and the LM
 * state integrated to TIG (LEMPREC; may wait for the lock).          */
agc_status_t agc_lm_job_powered_ascent_lm2(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    as_restart_here(AGC_JOB_POWERED_ASCENT_LM2);
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = agc_lm_guidinit(&ip);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    (void)agc_flag_set(AGC_FLAG_FLPI);
    (void)agc_lm_powered_ascent_init(&ip);
    agc_lm_asc()->tgo = AS_TGOA;            /* initial nominal TGO         */
    return agc_lm_job_powered_ascent_lmb(ctx);
}

/* P12LMB (p. 839): V1S, R, gravity and UNIT/R/ at TIG; XRANGE for N76
 * and the nominal injection targets.                                 */
agc_status_t agc_lm_job_powered_ascent_lmb(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_integ_ctx_t *ic = agc_integ_shared_ctx();
    agc_status_t st = agc_integ_lemprec(&ip, ic, agc_erase_tig(), AGC_JOB_POWERED_ASCENT_LMB);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;                            /* asleep        */
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    agc_guid_t *g = agc_guid_state();
    agc_lm_asc_t *a = agc_lm_asc();
    agc_mat_t m = agc_erase_refsmmat();
    a->v1s = agc_vec_shl(&ip, agc_mat_vec(&ip, &m, ic->out.vatt), 1);
    g->lm.r = agc_vec_shl(&ip, agc_mat_vec(&ip, &m, ic->out.ratt), 6);
    g->avg.gdt1_2 = agc_lm_mungrav(&ip, g->lm.r);
    g->avg.unitr = agc_vec_unit(&ip, g->lm.r, NULL, NULL);
    a->xrange = agc_dp_neg(agc_dp_shr(agc_lm_ycomp(&ip), 5));
    a->zdotd = AS_VINJNOM;
    a->rdotd = AS_RDOTDNOM;
    as_restart_here(AGC_JOB_NEWLOAD);
    return agc_lm_job_newload(ctx);
}

/* NEWLOAD (p. 839): flash cross range and apolune values (V06N76). */
agc_status_t agc_lm_job_newload(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_flag_test(AGC_FLAG_FLPI));
    return as_flash00(ctx, 76, AGC_JOB_NEWLOAD2, AGC_JOB_NEWLOAD);
}

/* After PROCEED on N76 (pp. 839-840): YCO, the velocity at tip-over
 * (V(IGN) + 49 fps UNIT/R/), RDOT, ZAXIS1, then one ASCENT pass. */
agc_status_t agc_lm_job_newload2(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_guid_t *g = agc_guid_state();
    agc_lm_asc_t *a = agc_lm_asc();
    g->lm.which = AGC_LM_WHICH_POWERED_ASCENT;
    as_restart_here(AGC_JOB_NEWLOAD2);
    a->yco = agc_dp_add(&ip, agc_dp_shl(&ip, a->xrange, 5), a->y);
    const agc_vec_t ur = g->avg.unitr;
    g->lm.v = agc_vec_add(&ip, agc_vec_scale(&ip, ur, AS_49FPS), a->v1s);
    g->lm.hdotdisp = agc_dp_shl(&ip, agc_vec_dot(&ip, g->lm.v, ur), 1);   /* RDOT */
    a->zaxis1 = agc_vec_unit(&ip, agc_vec_cross(&ip, ur, a->qaxis), NULL, NULL);
    (void)agc_flag_set(AGC_FLAG_FLVR);
    return agc_lm_ascent(ctx, &ip);
}

/* P12RET (p. 840): yaw and pitch of the thrust command for display,
 * one degree deadband, then the ignition sequence (P12BURN).         */
agc_status_t agc_lm_powered_ascent_ret(const agc_job_ctx_t *ctx, agc_interp_t *ip)
{
    if (!AGC_ASSERT(ctx != NULL && ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_lm_asc_t *a = agc_lm_asc();
    agc_dp_t s = agc_dp_add(ip, agc_dp_sq(ip, a->atp), agc_dp_sq(ip, a->aty));
    a->yaw = 0;
    if (s != 0) {
        agc_dp_t h = agc_dp_div(ip, a->aty, agc_dp_shl(ip, agc_dp_sqrt(ip, s), 1));
        a->yaw = agc_dp_asin(ip, h);                     /* revolutions   */
    }
    agc_vec_t u = agc_vec_unit(ip, agc_lm_ldg()->unfc_2, NULL, NULL);
    agc_dp_t c = agc_dp_shl(ip, agc_vec_dot(ip, u, agc_guid_state()->avg.unitr), 1);
    a->pitch = agc_dp_neg(agc_dp_acos(ip, c));
    as_restart_here(AGC_JOB_POWERED_ASCENT_BURN);
    agc_job_ctx_t c2 = *ctx;
    return agc_lm_job_powered_ascent_burn(&c2);
}

/* P12RET after its phase change: FLPI off, deadband, BURNBABY. */
agc_status_t agc_lm_job_powered_ascent_burn(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)agc_flag_clear(AGC_FLAG_FLPI);
    agc_inhint();
    (void)agc_lm_dap_pflitedb();
    agc_relint();
    (void)AGC_ASSERT(agc_guid_state()->lm.which == AGC_LM_WHICH_POWERED_ASCENT);
    return agc_lm_burnbaby(ctx);
}
