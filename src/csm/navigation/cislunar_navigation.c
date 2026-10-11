/*
 * cislunar_navigation.c -- P23 CISLUNAR MIDCOURSE NAVIGATION and R57
 * OPTICS CALIBRATION ROUTINE, CSM (P20-P25.agc, Comanche055 pp.570-571,
 * 619-631): P23, P23.00 .. P23.85, V94ENTER, POINTAXS (R23.05 .. R23.5),
 * R23.55 .. R23.END, R23.10, HORIZ, DIVIDE, RECT.1, LDPLANET, R57.
 *
 * "Do midcourse navigation by incorporation of star/earth and star/moon
 * optical measurements." The crew names a star and an earth or moon
 * landmark or horizon (noun 70); the optics measure the angle between
 * them; the measured minus computed angle, corrected for aberration of
 * light, updates the CSM state. R57 "measures the effect of solar
 * radiation on the SXT trunnion angle and stores the measured trunnion
 * bias for P23".
 *
 * Scales: RZC, RCLL m 2^29 (earth centred at the measured body), VZC m/cs
 * 2^7, unit vectors 2^1, angles revolutions, DELTAQ m 2^29 (2^27 moon).
 */
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define CN_ONE_C   ((agc_dp_t) 187779866) /* ONE/C 2DEC* .333564049 E-6 B+21* */
#define CN_AEARTH  ((agc_dp_t) 3189083)   /* AEARTH 2DEC 6378166 B-29     */
#define CN_RADMOON ((agc_dp_t) 869045)    /* RADMOON 2DEC 1738090 B-29    */
#define CN_TRUN19  ((agc_dp_t) (01604 * 16384)) /* TRUN19 OCT 01604, 00000  */
#define CN_1_0B18  ((agc_dp_t) 1024) /* 1.0B18 2DEC 1.0 B-18         */
#define CN_VARSUBL3 \
    ((agc_dp_t) 13719616) /* VARSUBL3 2DEC* 3.4299040 E+6 B-26* */
#define CN_TRUNVAR    ((agc_dp_t) 175922)    /* TRUNVAR 2DEC 2.5 E-9 B+18    */
#define CN_PI_4       ((agc_dp_t) 210828714) /* PI/4.0                       */
#define CN_JC_X       ((agc_dp_t) 72160567)  /* JCAXIS 1/2 sin 32.523 deg    */
#define CN_JC_Z       ((agc_dp_t) 113169126) /* JCAXIS 1/2 cos 32.523 deg    */
#define CN_1_SQR3     ((agc_dp_t) 154981267) /* 1/SQR3 2DEC .57735021        */
#define CN_19_77DEG   061740u                /* 19.77DEG OCT 61740           */
#define CN_V50N25P    0202U                  /* V50N25P OCT 00202 (GOPERF1)  */
#define CN_BITS7_9    0700u
#define CN_BITS4_6    0070u
#define CN_PRIO14     ((agc_prio_t) 014)
#define CN_ALARM_BUSY ((agc_alarm_code_t) 01211) /* marking system busy    */

/* P23 erasables.                                                         */
typedef struct cn_state
{
    agc_sp_t     trunbias; /* TRUNBIAS: R57 trunnion bias, CDU units    */
    uint16_t     trunion;  /* TRUNION: trunnion CDU of the mark         */
    agc_dp_t     marktime; /* MARKTIME                                  */
    agc_vec_t    rzc, vzc; /* RZC m 2^29, VZC m/cs 2^7                  */
    agc_mat_t    ubar;     /* UBAR0 UBAR1 UBAR2: horizon frame          */
    agc_vec_t    rcll;     /* RCLL: CSM to landmark/horizon, m 2^29     */
    agc_incorp_t inc;
    agc_job_id_t calibration_ret; /* EGRESS of R57                             */
} cn_state_t;

static cn_state_t cn;

agc_sp_t agc_erase_trunbias(void)
{
    if (!AGC_ASSERT(cn.trunbias >= AGC_SP_MIN))
    {
        return (0); /* no valid result */
    }

    return (cn.trunbias);
}

/* ---- R57 (pp.570-571) -------------------------------------------------- */
/* R57: abort (BAILOUT 1211) when the marking system or an extended verb is
 * in use; else marking system in use, V59 (calibration mark, GOMARKFR).
 * The caller continues as ret after ENDR57.                             */
agc_status_t rz_optics_calibration(const agc_job_ctx_t *ctx, agc_job_id_t ret)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ret != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    cn.calibration_ret = ret;
    uint16_t x = agc_erase_extvbact();

    if ((x & 6U) != 0U)
    { /* P23ABRT  */
        agc_status_t st = agc_bailout(CN_ALARM_BUSY);

        return ((st == AGC_STATUS_OK) ? AGC_STATUS_ALARM : st);
    }

    agc_erase_set_extvbact((uint16_t) (x | 2U));

    return (rz_continue(AGC_JOB_OPTICS_CALIBRATION)); /* R57A     */
}

/* R57A: V59FLAG (calibration mark); V59 (GOMARKFR, R1-R3 blanked):
 * TERMINATE P00, PROCEED or ENTER end R57.                               */
agc_status_t agc_job_optics_calibration(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_flag_put(AGC_FLAG_CALIBRATION_MARKING, true); /* V59FLAG  */
    agc_status_t st = rz_flash(ctx, AGC_DSKY_GOMARKFR, 59U, 0U,
                               AGC_JOB_GOTOPOOH, AGC_JOB_OPTICS_CALIBRATION_END,
                               AGC_JOB_OPTICS_CALIBRATION_END);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(7U));
}

/* MARKDISP: after a calibration mark, V06N87 (the trunnion) GOMARKFR:
 * PROCEED keeps the bias (R57B), ENTER marks again (R57A).               */
agc_status_t agc_job_optics_calibration_mark_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st =
        rz_flash(ctx, AGC_DSKY_GOMARKFR, 6U, 87U, AGC_JOB_GOTOPOOH,
                 AGC_JOB_OPTICS_CALIBRATION_BIAS, AGC_JOB_OPTICS_CALIBRATION);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(5U));
}

/* R57B: TRUNBIAS = the marked trunnion (MRKBUF1 +5) MSU 19.77 degrees
 * (two's to ones' complement about the trunnion zero).                  */
agc_status_t agc_job_optics_calibration_bias(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int d = ((int) agc_erase_mrkbuf(1, 5) - (int) CN_19_77DEG) & 077777;

    d           = ((d & 040000) != 0) ? d - 0100000 : d;
    cn.trunbias = (agc_sp_t) ((d < AGC_SP_MIN) ? AGC_SP_MIN : d);

    return (agc_job_optics_calibration_end(ctx));
}

/* ENDR57: V59FLAG off; ENDMARK as a PRIO14 job (so that R53 can take
 * over the marking); back to the caller.                                 */
agc_status_t agc_job_optics_calibration_end(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_flag_put(AGC_FLAG_CALIBRATION_MARKING, false);
    agc_status_t st =
        agc_exec_novac(CN_PRIO14, AGC_JOB_OPTICS_CALIBRATION_END_MARK);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (rz_continue(cn.calibration_ret));
}

agc_status_t agc_job_optics_calibration_end_mark(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_endmark());
}

/* ---- RECT.1 (p.630): the CSM state from the permanent conic + deviation
 * at 2^29 / 2^7 (SR2 from the moon scale); ZMEASURE when the measured
 * body is not the primary body.                                          */
static agc_status_t cn_rect(agc_interp_t *ip)
{
    bool moon = agc_flag_test(AGC_FLAG_CMOONFLG);
    int  x2   = moon ? -2 : 0;

    rz_flag_put(AGC_FLAG_ZMEASURE, moon != agc_flag_test(AGC_FLAG_LUNAFLAG));
    agc_integ_sv_t sv;
    agc_status_t   st = agc_integ_perm_get(AGC_VEHICLE_CSM, &sv);
    agc_vec_t      r  =
        agc_vec_add(ip, rz_vshift(ip, agc_vec_shr(sv.deltav, 7), x2), sv.rcv);

    cn.rzc = rz_vshift(ip, r, x2); /* B29      */
    agc_vec_t v =
        agc_vec_add(ip, rz_vshift(ip, agc_vec_shr(sv.nuv, 4), x2), sv.vcv);

    cn.vzc = rz_vshift(ip, v, x2); /* B7       */

    return (st);
}

/* ---- HORIZ (pp.627-630) ------------------------------------------------- */
/* DIVIDE: b / a with both normalized (b also SR1): the quotient and its
 * index *x1 (C(X1) = -n(b) + n(a)); callers apply SR* k,1.              */
static agc_dp_t cn_divide(agc_interp_t *ip, agc_dp_t b, agc_dp_t a, int *x1)
{
    int      nb = 0;
    int      na = 0;
    agc_dp_t bn = agc_dp_shr(rz_norm(b, &nb), 1);
    agc_dp_t an = rz_norm(a, &na);

    *x1 = nb - na; /* XSU,1 S1 */

    if (!AGC_ASSERT(an != 0))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_div(ip, bn, an)); /* BDDV     */
}

/* SR* k,1 after DIVIDE.                                                  */
static agc_dp_t cn_div_sr(agc_interp_t *ip, agc_dp_t b, agc_dp_t a, int k)
{
    int      x1 = 0;
    agc_dp_t q  = cn_divide(ip, b, a, &x1);

    return (rz_sr_star(ip, q, k, x1));
}

/* The two tangent points of the horizon ellipse (semi-axes AH, BH) seen
 * from RH in the UBAR frame: XH/A +- ALPHA, YH/A -+ BETA, A = (XH/AH)^2 +
 * (YH/BH)^2.                                                             */
static void cn_tangents(agc_interp_t *ip, agc_vec_t rh, agc_dp_t ah,
                        agc_dp_t bh, agc_vec_t t[2])
{
    agc_dp_t xa = cn_div_sr(ip, rh.c[0], ah, 8); /* B9       */
    agc_dp_t yb = cn_div_sr(ip, rh.c[1], bh, 8);
    agc_dp_t a  =
        agc_dp_add(ip, agc_dp_mul_trunc(yb, yb), agc_dp_mul_trunc(xa, xa));
    agc_dp_t sq    = agc_dp_sqrt(ip, agc_dp_sub(ip, a, CN_1_0B18)); /* B9 */
    agc_dp_t xha   = cn_div_sr(ip, rh.c[0], a, 17); /* B29      */
    agc_dp_t yha   = cn_div_sr(ip, rh.c[1], a, 17);
    agc_dp_t s28   = cn_div_sr(ip, sq, a, 8); /* 28D      */
    agc_dp_t alpha = agc_dp_mul_trunc(
        agc_dp_mul_trunc(cn_div_sr(ip, ah, bh, 0), s28), rh.c[1]);

    alpha = agc_dp_shl(ip, alpha, 1);
    agc_dp_t beta = agc_dp_mul_trunc(
        agc_dp_mul_trunc(cn_div_sr(ip, bh, ah, 0), s28), rh.c[0]);

    beta = agc_dp_shl(ip, beta, 1);
    t[0] =
        agc_vec_make(agc_dp_add(ip, xha, alpha), agc_dp_sub(ip, yha, beta), 0);
    t[1] =
        agc_vec_make(agc_dp_sub(ip, xha, alpha), agc_dp_add(ip, yha, beta), 0);
}

/* HORIZ: the horizon point (near or far, NORFHOR) of the measured body in
 * the plane of the star (US) and RZC; earth: the Fischer ellipsoid plus
 * HORIZALT; moon: RADMOON. Result m 2^29 in reference axes.             */
static agc_status_t cn_horiz(agc_interp_t *ip, agc_vec_t *out)
{
    agc_navaux_padload_t pad;
    agc_status_t         st = agc_navaux_get_padload(&pad);
    agc_vec_t            us = agc_align_ram()->starsav2;
    agc_vec_t            uz = agc_vec_make(pad.neg_ayo, pad.axo, AGC_DP_MAX);

    cn.ubar.row[2] =
        agc_vec_unit(ip, agc_vec_cross(ip, us, cn.rzc), NULL, NULL);
    cn.ubar.row[0] =
        agc_vec_unit(ip, agc_vec_cross(ip, uz, cn.ubar.row[2]), NULL, NULL);
    cn.ubar.row[1] = agc_vec_unit(
        ip, agc_vec_cross(ip, cn.ubar.row[2], cn.ubar.row[0]), NULL, NULL);
    agc_dp_t bh = CN_RADMOON; /* HORIZ.6  */
    agc_dp_t ah = CN_RADMOON;

    if (!agc_flag_test(AGC_FLAG_LUNAFLAG))
    {
        agc_dp_t halt = agc_rendz_nav_pad()->horizalt;
        agc_dp_t s    = agc_vec_dot(ip, cn.ubar.row[1], uz); /* ALPHAV +4 */

        bh = agc_dp_add(ip, agc_latlong_geterad(ip, s), halt);
        ah = agc_dp_add(ip, CN_AEARTH, halt);
    }

    agc_vec_t rh =
        agc_vec_shl(ip, agc_mat_vec(ip, &cn.ubar, cn.rzc), 1); /* HORIZ.1 */
    agc_vec_t ush = agc_vec_shl(ip, agc_mat_vec(ip, &cn.ubar, us), 1);
    agc_vec_t t[2];

    cn_tangents(ip, rh, ah, bh, t);
    agc_dp_t a0 = agc_vec_dot(
        ip, agc_vec_unit(ip, agc_vec_sub(ip, t[0], rh), NULL, NULL), ush);
    agc_dp_t a1 = agc_vec_dot(
        ip, agc_vec_unit(ip, agc_vec_sub(ip, t[1], rh), NULL, NULL), ush);
    bool far    = agc_flag_test(AGC_FLAG_NORFHOR);
    bool use_t1 = (agc_dp_sub(ip, a0, a1) < 0) ? !far : far; /* HORIZ.3  */

    *out = agc_vec_shl(ip, agc_vec_mat(ip, use_t1 ? t[1] : t[0], &cn.ubar),
                       1); /* HORIZ.5 */

    return (st);
}

/* R23.3: a landmark: ERADFLAG, LALOTORV of LAT LONG ALT at MARKTIME.     */
static agc_status_t cn_landmark(agc_interp_t *ip, agc_vec_t *r)
{
    rz_flag_put(AGC_FLAG_ERADFLAG, true);
    agc_navaux_padload_t pad;
    agc_latlong_t        ll;
    agc_status_t         st = agc_navaux_get_padload(&pad);

    if (st == AGC_STATUS_OK)
    {
        st = agc_navaux_get_latlong(&ll);
    }

    agc_latlong_opt_t opt = {agc_flag_test(AGC_FLAG_LUNAFLAG), true};

    if (st == AGC_STATUS_OK)
    {
        st = agc_latlong_to_r(ip, &pad, opt, &ll, cn.marktime, r, NULL);
    }

    return (st);
}

/* ---- POINTAXS (R23.05 .. R23.5 pp.623-624) ------------------------------
 * W from WMIDPOS / WMIDVEL when invalid; the CSM (with W when valid) to
 * MARKTIME; RECT.1; RZC moved to the measured body (LUNPOS); RCLL = the
 * landmark (LALOTORV) or horizon point (HORIZ) - RZC. BUSY: INTSTALL. */
static agc_status_t cn_pointaxs(agc_interp_t *ip, agc_job_id_t resume)
{
    if (!agc_flag_test(AGC_FLAG_ORBWFLAG))
    { /* R23.05   */
        const agc_rendz_nav_pad_t *p = agc_rendz_nav_pad();
        agc_incorp_w_t            *w = &agc_integ_shared_ctx()->w;

        for (int m = 0; m < 6; m++)
        {
            w->blk[0][m] = agc_vec_zero();
            w->blk[1][m] = agc_vec_zero();
        }

        for (int k = 0; k < 3; k++)
        { /* INITIALW */
            w->blk[0][k].c[k]     = agc_dp_from_sp(p->wmidpos);
            w->blk[1][3 + k].c[k] = agc_dp_from_sp(p->wmidvel);
        }
    }

    rz_set_marktime(cn.marktime);
    agc_status_t st = rz_setintg(resume); /* R23.1    */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_DIM0FLAG, agc_flag_test(AGC_FLAG_ORBWFLAG));
    rz_flag_put(AGC_FLAG_ORBWFLAG, true); /* R23.2    */
    st = agc_integ_integrv(ip, agc_integ_shared_ctx(), cn.marktime);
    agc_phase_change(2, 04022);

    if (st == AGC_STATUS_OK)
    {
        st = cn_rect(ip);
    }

    if (st == AGC_STATUS_OK && agc_flag_test(AGC_FLAG_ZMEASURE))
    {
        /* ZMEASURE (p.624): RZC from the primary to the measured body: + the
         * moon (LUNPOS) with the moon primary, - the moon with the earth.        */
        agc_ephem_csm_t      eph;
        agc_navaux_padload_t pad;
        agc_status_t         st_shift = agc_ephem_get_csm_data(&eph);

        if (st_shift == AGC_STATUS_OK)
        {
            st_shift = agc_navaux_get_padload(&pad);
        }

        agc_vec_t moon = agc_vec_zero();

        if (st_shift == AGC_STATUS_OK)
        {
            st_shift = agc_ephem_csm_lunpos(ip, &eph, pad.tephem, cn.marktime,
                                            &moon);
        }

        if (!agc_flag_test(AGC_FLAG_CMOONFLG))
        {
            moon = agc_vec_neg(moon); /* VCOMP    */
        }

        cn.rzc = agc_vec_add(ip, moon, cn.rzc);
        st     = st_shift;
    }

    agc_vec_t r = agc_vec_zero();

    if (st == AGC_STATUS_OK)
    {
        st = (rz_word_of(agc_erase_landmark()) != 0U) ?
             cn_landmark(ip, &r) :
             cn_horiz(ip, &r);     /* R23.4 */
    }

    cn.rcll = agc_vec_sub(ip, r, cn.rzc); /* R23.5    */

    return (st);
}

/* ---- R23.55 .. R23.53 (pp.625-626) --------------------------------------
 * The star-landmark (horizon) angle corrected for aberration: UCLSTAR =
 * UNIT(UNIT(RCLL) + VZC/c), USSTAR = UNIT(US + (VZC - VESO)/c); B0 =
 * UNIT(USSTAR - cos(Q) UCLSTAR); DELTAQ = (trunnion - TRUNBIAS (SR3) +
 * 19.77 deg - Q) |RCLL| PI/4 SL3 (SL2 more in the moon's sphere);
 * VARIANCE = |RCLL|^2 TRUNVAR + VARSUBL3; INCORP1 (6x6) and the noun 49
 * values.                                                               */
static agc_status_t cn_measure(agc_interp_t *ip)
{
    agc_dp_t        r2 = 0;
    agc_dp_t        r1 = 0;
    agc_vec_t       u  = agc_vec_unit(ip, cn.rcll, &r1, &r2); /* 30D, 32D */
    agc_ephem_csm_t eph;
    agc_status_t    st  = agc_ephem_get_csm_data(&eph);
    agc_vec_t       a   = agc_vec_shr(agc_vec_scale(ip, cn.vzc, CN_ONE_C), 15);
    agc_vec_t       ucl = agc_vec_unit(ip, agc_vec_add(ip, a, u), NULL, NULL);
    agc_vec_t       dv  = agc_vec_sub(ip, agc_vec_shr(cn.vzc, 2), eph.veso);
    agc_vec_t       b   = agc_vec_shr(agc_vec_scale(ip, dv, CN_ONE_C), 13);
    agc_vec_t       uss = agc_vec_unit(
        ip, agc_vec_add(ip, b, agc_align_ram()->starsav2), NULL, NULL);
    agc_dp_t  cosq = agc_dp_shl(ip, agc_vec_dot(ip, uss, ucl), 1);
    agc_vec_t c    = agc_vec_shl(ip, agc_vec_neg(agc_vec_scale(ip, ucl,
                                                               cosq)), 1);

    cn.inc.bvector[0] = agc_vec_unit(ip, agc_vec_add(ip, c, uss), NULL, NULL);
    cn.inc.bvector[1] = agc_vec_zero();
    cn.inc.bvector[2] = agc_vec_zero();
    agc_dp_t q  = agc_dp_neg(agc_dp_acos(ip, cosq));
    int      tv = ((cn.trunion & 040000u) != 0U) ? (int) cn.trunion - 0100000 :
                  (int) cn.trunion;                                  /* MSU 0 */
    agc_dp_t t =
        agc_dp_from_sp((agc_sp_t) ((tv < AGC_SP_MIN) ? AGC_SP_MIN : tv));

    t = agc_dp_shr(agc_dp_sub(ip, t, agc_dp_from_sp(
                                  cn.trunbias)), 3);
    t = agc_dp_add(ip, agc_dp_add(ip, t, CN_TRUN19), q);
    t =
        agc_dp_shl(ip, agc_dp_mul_trunc(agc_dp_mul_trunc(t, r1), CN_PI_4), 3);
    cn.inc.deltaq =
        agc_flag_test(AGC_FLAG_CMOONFLG) ? agc_dp_shl(ip, t, 2) : t;
    cn.inc.variance = agc_dp_mul_tp(r2, CN_TRUNVAR) + (agc_tp_t) CN_VARSUBL3;
    rz_flag_put(AGC_FLAG_DMENFLG, false);
    cn.inc.dim9 = false;

    if (st == AGC_STATUS_OK)
    {
        st =
            agc_incorp_state_deviation(ip, &agc_integ_shared_ctx()->w, &cn.inc);
    }

    agc_phase_change(2, 04022);
    int k = agc_flag_test(AGC_FLAG_CMOONFLG) ? -2 : 0; /* SR2      */

    rz_set_csm_residuals(1,
                         rz_shift(ip, agc_vec_abval(ip, cn.inc.deltax[1]), k));
    rz_set_csm_residuals(0,
                         rz_shift(ip, agc_vec_abval(ip, cn.inc.deltax[0]), k));

    return (st);
}

/* ---- P23 program flow (pp.619-623, 626-627, 631) ----------------------- */
/* P23: rendezvous off; group 4 off, group 2 on; one mark; star and
 * landmark target flags off; STARIND, BESTI zero; R57 to be done; V94 not
 * allowed. Without REFSMMAT: R57 and R53 now (P23.00). DEVIATION:
 * PHSPRDT2 = PRIO13 (the group 2 restart priority) and TEMPFLSH = R60ADRS
 * (V94ENTER) are not kept; the restart tables of the exec lane own them. */
agc_status_t agc_job_cislunar_midcourse_navigation(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_flag_put(AGC_FLAG_RNDVZFLG, false);
    agc_phase_change(4, 00004); /* 2PHSCHNG: leave group 4 */
    agc_phase_change(2, 00012); /* enter group 2           */
    agc_align_ram_t *a = agc_align_ram();

    a->markindx = 1;
    rz_flag_put(AGC_FLAG_TARG2FLG, false);
    rz_flag_put(AGC_FLAG_TARG1FLG, false);
    a->starind = 0;
    a->besti   = 0;
    rz_flag_put(AGC_FLAG_TRUNNION_BIAS_KNOWN, false);   /* R57FLAG  */
    rz_flag_put(AGC_FLAG_ALTERNATE_LOS_ALLOWED, false); /* V94FLAG  */

    if (agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        rz_flag_put(AGC_FLAG_SAVECFLG, false); /* P23.05   */

        return (agc_job_cislunar_midcourse_navigation_ident(ctx));
    }

    return (rz_optics_calibration(ctx,
                                  AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_SIGHT));
}

/* P23.00 after R57: R53, then P23.60.                                    */
agc_status_t agc_job_cislunar_midcourse_navigation_sight(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_csm_sighting_mark(ctx,
                                  AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_MARKED));
}

/* P23.06: V05N70 (star, landmark, horizon codes).                        */
agc_status_t agc_job_cislunar_midcourse_navigation_ident(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 5U, 70U, AGC_JOB_GOTOPOOH,
                     AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_IDENT_CHECK,
                     AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_IDENT));
}

/* R23.10: operator error, the codes again.                               */
static agc_status_t cn_ident_error(agc_job_ctx_t *ctx)
{
    agc_dsky_falton();
    agc_status_t st = AGC_STATUS_OK;

    return ((st != AGC_STATUS_OK) ?
            st :
            agc_job_cislunar_midcourse_navigation_ident(ctx));
}

/* P23.15 .. P23.18: exactly one of landmark and horizon; its C digit 1
 * earth, 2 moon (LUNAFLAG); the star 0 (planet) .. 37 (STARSAV2 from the
 * catalogue, BESTI); a horizon's D digit 1 near, 2 far (NORFHOR); a
 * landmark's coordinates on noun 89; then P23.30.                        */
agc_status_t agc_job_cislunar_midcourse_navigation_ident_check(
    agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t lm = rz_word_of(agc_erase_landmark());

    if ((lm == 0U) == (agc_erase_horizon() == 0U))
    {
        return (cn_ident_error(ctx)); /* P23.151  */
    }

    uint16_t c =
        (uint16_t) (((lm != 0U) ? lm : agc_erase_horizon()) & CN_BITS7_9);

    if (c != 0100U && c != 0200U)
    {
        return (cn_ident_error(ctx));
    }

    rz_flag_put(AGC_FLAG_LUNAFLAG, c == 0200U); /* P23.16   */
    agc_align_ram_t *a    = agc_align_ram();
    agc_sp_t         star = a->starcode;

    if (star < 0 || star > 37)
    {
        return (cn_ident_error(ctx));
    }

    if (star != 0)
    { /* P23.17   */
        a->besti = (agc_sp_t) (6 * star);
        agc_status_t st = agc_star_vector(star, &a->starsav2); /* LOWMEMRY */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    if (agc_erase_horizon() == 0U)
    { /* P23.20   */
        return (rz_landing_site_display(
                    ctx, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_STAR));
    }

    uint16_t d = (uint16_t) (agc_erase_horizon() & CN_BITS4_6);

    if (d != 010U && d != 020U)
    {
        return (cn_ident_error(ctx));
    }

    rz_flag_put(AGC_FLAG_NORFHOR, d == 020U); /* P23.18   */

    return (agc_job_cislunar_midcourse_navigation_star(ctx));
}

/* P23.30: a planet (star code 0): its vector on noun 88 (LDPLANET); then
 * P23.31: the second pass incorporates (P23.85); else V50N25 checklist
 * 202: PROCEED V94 (CMC points the optics), ENTER R57 / R52 (P23.55).   */
agc_status_t agc_job_cislunar_midcourse_navigation_star(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_align_ram()->starcode == 0)
    { /* LDPLANET */
        return (rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 88U, AGC_JOB_GOTOPOOH,
                         AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_PLANET,
                         AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_STAR));
    }

    return (agc_job_cislunar_midcourse_navigation_checklist(ctx));
}

/* LDPLANET PROCEED: STARSAV2 = UNIT(STARSAV3 / SQRT 3).                  */
agc_status_t agc_job_cislunar_midcourse_navigation_planet(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_align_ram_t *a = agc_align_ram();

    a->starsav2 = agc_vec_unit(&ip, agc_vec_scale(&ip, a->starsav3, CN_1_SQR3),
                               NULL, NULL);

    return (agc_job_cislunar_midcourse_navigation_checklist(ctx));
}

agc_status_t agc_job_cislunar_midcourse_navigation_checklist(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_flag_test(AGC_FLAG_SAVECFLG))
    {
        return (agc_job_cislunar_midcourse_navigation_incorporate(
                    ctx)); /* P23.85 */
    }

    agc_dsky_cont_t cont;

    cont.terminate = AGC_JOB_GOTOPOOH;
    cont.proceed   = AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_POINT;
    cont.enter     = AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_CALIBRATE;

    return (agc_dsky_display(ctx, AGC_DSKY_GOPERF1, CN_V50N25P, &cont));
}

/* P23.55: ENTER on the checklist: R57, then R52 (P23.56).                */
agc_status_t agc_job_cislunar_midcourse_navigation_calibrate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_optics_calibration(ctx,
                                  AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_OPTICS));
}

/* V94ENTER (PROCEED on the checklist, or verb 94 during R52): MARKTIME
 * = now; then the attitude for the measurement.                          */
agc_status_t agc_job_cislunar_midcourse_navigation_point(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    cn.marktime = agc_rtb_loadtime();

    return (agc_job_cislunar_midcourse_navigation_point_axes(ctx));
}

/* V94ENTER cont.: POINTAXS; POINTVSM = UNIT(REFSMMAT RCLL); SCAXIS =
 * JCAXIS (the SXT line of sight at 32.523 degrees trunnion); 3AXISFLG
 * off (VECPOINT); R60CSM; resumes here after an integration wait.        */
agc_status_t agc_job_cislunar_midcourse_navigation_point_axes(
    agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_status_t st =
        cn_pointaxs(&ip, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_POINT_AXES);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    agc_mat_t refsmmat = agc_erase_refsmmat();

    agc_imu_set_pointvsm(
        agc_vec_unit(&ip, agc_mat_vec(&ip, &refsmmat, cn.rcll), NULL, NULL));
    st = AGC_STATUS_OK;
    agc_vec_t jc = agc_vec_zero();

    jc.c[0] = CN_JC_X;
    jc.c[2] = CN_JC_Z;
    st      = agc_imu_set_scaxis(jc);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_3AXISFLG, false);
    agc_phase_change(2, 00012);

    return (agc_imu_attitude_maneuver(
                ctx, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_MANEUVERED));
}

/* After R60: R57 unless already done (R57FLAG), then R52.                */
agc_status_t agc_job_cislunar_midcourse_navigation_maneuvered(
    agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(2, 04022);

    if (agc_flag_test(AGC_FLAG_TRUNNION_BIAS_KNOWN))
    {
        return (agc_job_cislunar_midcourse_navigation_optics(ctx)); /* P23.57 */
    }

    return (agc_job_cislunar_midcourse_navigation_calibrate(ctx)); /* P23.56 */
}

/* P23.57: V94 allowed, R57 done; R52 (automatic optics positioning).     */
agc_status_t agc_job_cislunar_midcourse_navigation_optics(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_flag_put(AGC_FLAG_ALTERNATE_LOS_ALLOWED, true); /* V94FLAG  */
    rz_flag_put(AGC_FLAG_TRUNNION_BIAS_KNOWN, true);   /* R57FLAG  */

    return (agc_csm_auto_optics_positioning(
                ctx, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_OPTICS_DONE));
}

agc_status_t agc_job_cislunar_midcourse_navigation_optics_done(
    agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_flag_put(AGC_FLAG_ALTERNATE_LOS_ALLOWED, false);
    rz_flag_put(AGC_FLAG_TRUNNION_BIAS_KNOWN, false);

    return (agc_job_cislunar_midcourse_navigation_marked(ctx));
}

/* P23.60: the mark (MARKSTAT VAC): MARKTIME, TRUNION = its CDUT, the mark
 * to MARKDOWN; V05N71.                                                  */
agc_status_t agc_job_cislunar_midcourse_navigation_marked(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_align_sxt_mark_t m;
    agc_status_t         st = agc_sxtmark_mark(0, &m);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    cn.marktime = m.t;
    cn.trunion  = m.cdut;
    agc_erase_set_markdown(0, rz_word_of(agc_dp_high(m.t)));
    agc_erase_set_markdown(1, rz_word_of(agc_dp_low(m.t)));
    agc_erase_set_markdown(2, m.cduy);
    agc_erase_set_markdown(3, m.cdus);
    agc_erase_set_markdown(4, m.cduz);
    agc_erase_set_markdown(5, m.cdut);
    agc_erase_set_markdown(6, m.cdux);

    return (agc_job_cislunar_midcourse_navigation_mark_display(ctx));
}

/* V05N71 GOFLASH: PROCEED stores the data (P23.65), ENTER redisplays.   */
agc_status_t agc_job_cislunar_midcourse_navigation_mark_display(
    agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 5U, 71U, AGC_JOB_GOTOPOOH,
                     AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_SAVE,
                     AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_MARK_DISPLAY));
}

/* P23.65: SAVECFLG; the codes are checked again (P23.15), then P23.85.   */
agc_status_t agc_job_cislunar_midcourse_navigation_save(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_flag_put(AGC_FLAG_SAVECFLG, true);

    return (agc_job_cislunar_midcourse_navigation_ident_check(ctx));
}

/* P23.85: W renewed by RENDWFLG off; POINTAXS at the mark time; R23.55;
 * R23.6: V06N49 (GOFLASHR, R3 blanked): PROCEED incorporates (R23.7),
 * TERMINATE / ENTER P00. Resumes here after an integration wait.        */
agc_status_t agc_job_cislunar_midcourse_navigation_incorporate(
    agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_flag_put(AGC_FLAG_RENDWFLG, false);
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_status_t st =
        cn_pointaxs(&ip, AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_INCORPORATE);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    st = cn_measure(&ip);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = rz_flash(ctx, AGC_DSKY_GOFLASHR, 6U, 49U, AGC_JOB_GOTOPOOH,
                  AGC_JOB_CISLUNAR_MIDCOURSE_NAVIGATION_UPDATE,
                  AGC_JOB_GOTOPOOH);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_dsky_blanket(4U); /* BIT3     */
    agc_phase_change(2, 00012);

    return (st);
}

/* R23.7: VEHUPFLG (the CSM); INCORP2; P00.                              */
agc_status_t agc_job_cislunar_midcourse_navigation_update(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_flag_put(AGC_FLAG_VEHUPFLG, true);
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_status_t st =
        rz_incorp2(&ip, &cn.inc, agc_flag_test(AGC_FLAG_CMOONFLG));

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_exec_gotopooh());
}
