/*
 * tpi_search.c -- P17/P77 TPI SEARCH, CSM only (TPI_SEARCH.agc,
 * Comanche055 pp.551-561).
 *
 * "The TPI search routine determines the minimum total velocity transfer
 * trajectory from a given TPI maneuver time within the constraint of a
 * safe pericenter. This velocity is the sum of the impulsive velocities
 * for the TPI and TPF maneuvers. The S17.1 routine extrapolates the state
 * vectors of both vehicles to the TPI time and computes the relative
 * phase angle between the vehicles, the altitude difference ... and
 * selects a search sector based on the sign of the altitude difference.
 * The S17.2 routine further defines the search sector by computing angular
 * limits and uses the time theta subroutine to compute the search start
 * and end times. The search is then made in an iterative loop using the
 * Lambert subroutine ..."
 *
 * Names: S17.1 = agc_rendz_tpi_search_phase, S17.2 =
 * agc_rendz_tpi_search_sector, TRANSANG = agc_rendz_transang.
 * Scales: r m 2^29, v m/cs 2^7, t cs 2^28, angles revolutions (THETZERO,
 * IT: half revolutions after PIINVERS), heights m 2^29.
 */
#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/imu/imu.h"

/* ---- constants (pp.551-552; "TEMPORARY") ------------------------------- */
#define TPS_HPE      ((agc_dp_t) 78710)     /* 157420.0 B-29: 85 NM       */
#define TPS_HPL      ((agc_dp_t) 5334)      /* 10668.0213 B-29: 35000 ft  */
#define TPS_CDSEC    ((agc_dp_t) 40000)     /* 2DEC 40000 cs              */
#define TPS_CLSEC    ((agc_dp_t) 15000)     /* 2DEC 15000 cs              */
#define TPS_PIINVERS ((agc_dp_t) 85445659)  /* .3183098862 = 1/pi         */
#define TPS_SEC1THET ((agc_dp_t) 52195783)  /* .1944444444: 70 deg        */
#define TPS_SEC2THET ((agc_dp_t) 246065835) /* .9166666667: 330 deg       */
#define TPS_MANYFEET ((agc_dp_t) -67108864) /* -1.0 B-2                   */
#define TPS_LIMVEL   ((agc_dp_t) 12784)     /* .6096 E-2 B-7: 2 fps       */
#define TPS_DFTMOON  ((agc_dp_t) 76)        /* .1524 E3 B-29: 500 ft      */
#define TPS_DP002    ((agc_dp_t) 536871)    /* 2DEC 0.002                 */
#define TPS_EPSFOUR  ((agc_dp_t) 11184811)  /* EPSFOUR .0416666666        */
/* TRANSANG (CONIC_SUBROUTINES MUTABLE, D1/32, 2PISC) */
#define TPS_SQRT_MUE ((agc_dp_t) 204442107) /* 1.99650495 E5 B-18*        */
#define TPS_SQRT_MUM ((agc_dp_t) 181389047) /* 2.21422176 E4 B-15*        */
#define TPS_INV_MUE  ((agc_dp_t) 115696162) /* .25087606 E-10 B+34*       */
#define TPS_INV_MUM  ((agc_dp_t) 146972992) /* .203966 E-8 B+28           */
#define TPS_D1_32    ((agc_dp_t) 8388608)   /* 1.0 B-5                    */
#define TPS_2PISC    ((agc_dp_t) 26353589)  /* 6.28318530 B-6             */

/* S17.2 work cells (TPI_SEARCH erasables). */
typedef struct tps_work
{
    agc_dp_t tfo, tfi;    /* search start / end time, cs 2^28          */
    agc_dp_t it;          /* IT: end angle, half revolutions            */
    agc_dp_t thetl;       /* THETL: transfer angle limit, revs          */
    agc_dp_t hpo, delveo; /* previous HP, DELVEE                        */
    agc_dp_t deltee;      /* DELTEE: time step, cs 2^28                 */
    int      xrs[2];      /* XRS: X1 X2 of the conic integration        */
} tps_work_t;

static tps_work_t tps_work;

/* KFLAG: off = passive higher (K positive), on = passive lower. */
static bool tps_kflag(void)
{
    return (agc_flag_test(AGC_FLAG_KFLAG));
}

/* LEMSTORE / CSMSTORE: an integration result as active (AVFLAG matches
 * the vehicle) or passive state.                                       */
static void tps_store(agc_rendz_t *rz, bool lm, const agc_integ_out_t *o)
{
    bool lm_active = agc_flag_test(AGC_FLAG_AVFLAG);

    if (lm == lm_active)
    {
        rz_store_active(rz, o->ratt, o->vatt);
    }
    else
    {
        rz_store_passive(rz, o->ratt, o->vatt);
    }
}

/* ---- S17.1 (pp.552-553) ------------------------------------------------ */
agc_status_t agc_rendz_tpi_search_phase(agc_interp_t *ip, agc_rendz_t *rz,
                                        agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_out_t o;
    agc_status_t    st =
        rz_integ(ip, RZ_LEMCONIC, agc_erase_ttpi(), resume, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    tps_store(rz, true, &o);
    st = rz_integ(ip, RZ_CSMCONIC, agc_erase_ttpi(), resume, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    tps_store(rz, false, &o);
    agc_dp_t  rp  = 0;
    agc_vec_t urp = agc_vec_unit(ip, rz->rpass3, &rp, NULL);     /* 0D */

    rz->delhite = agc_dp_sub(ip, rp, rz_abval(ip, rz->ract3)); /* RP - RA */
    rz_flag_put(AGC_FLAG_KFLAG, rz->delhite < 0);                /* ON = -  */
    rz->e2 = rz_unit(ip, agc_vec_cross(ip, urp, rz->vpass3));
    agc_vec_t p   = agc_vec_shl(ip, agc_vec_proj(ip, rz->ract3, rz->e2), 2);
    agc_vec_t rpa = rz_unit(ip, agc_vec_sub(ip, rz->ract3, p)); /* RPA  */
    agc_dp_t  a   =
        agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, rpa, urp), 1));
    agc_dp_t s = agc_vec_dot(ip, agc_vec_cross(ip, urp, rz->ract3), rz->e2);

    rz->thetzero    = agc_dp_sign(a, s); /* + ACTIVE AHEAD, - BEHIND  */
    tps_work.xrs[0] = o.x1;              /* XRS        */
    tps_work.xrs[1] = o.x2;

    return (agc_rendz_selectmu(rz, o.x2 != 0)); /* RTX1 RTX2 = XRS (JUNCT3) */
}

/* SNTH and CSTH = -|cos| of angle (half revolutions) for TIMETHET:
 * "SIN(180-ALPHA)=SIN(ALPHA) ETC" (COS BMN +2 DCOMP).                  */
static agc_status_t tps_timethet(agc_interp_t *ip, agc_rendz_t *rz,
                                 agc_dp_t angle, agc_dp_t *t)
{
    agc_dp_t snth = agc_dp_sin(ip, angle);
    agc_dp_t csth = agc_dp_cos(ip, angle);

    if (csth >= 0)
    {
        csth = agc_dp_neg(csth);
    }

    agc_conic_time_out_t o;
    agc_status_t         st =
        rz_timethet_rv(ip, rz, rz->rpass3, rz->vpass3, snth, csth, true, &o);

    *t = o.t;

    return (st);
}

/* S17.2 head (pp.553-554): sector limits PHI(0) = 2.5 X - THETZERO and
 * PHI(I) = IT = X - THETZERO, X = (SQRT|URA . E2| SL1 + .002)/PI (sign by
 * KFLAG); start time TF = TFO, end time TFI; DELVEE = DPPOSMAX, HP =
 * MANYFEET, THETL = 70 deg (KFLAG on) or 330 deg; CLEAR ITSWICH.       */
static agc_status_t tps_limits(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_dp_t x = agc_dp_abs(agc_vec_dot(ip, rz_unit(ip, rz->ract3), rz->e2));

    x = agc_dp_add(ip, agc_dp_shl(ip, agc_dp_sqrt(ip, x), 1), TPS_DP002);

    if (!tps_kflag())
    {
        x = agc_dp_neg(x); /* BON DCOMP KFLAG    */
    }

    x           = agc_dp_mul_trunc(x, TPS_PIINVERS); /* 0D     */
    tps_work.it = agc_dp_sub(ip, x, rz->thetzero);   /* PHI(I) */
    agc_dp_t phi0 = agc_dp_add(ip, agc_dp_add(ip, agc_dp_shr(x, 1), x),
                               tps_work.it);     /* PHI(0) */
    agc_dp_t     t  = 0;
    agc_status_t st = tps_timethet(ip, rz, phi0, &t);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->tf       = t;
    tps_work.tfo = t;
    st           = tps_timethet(ip, rz, tps_work.it, &t);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    tps_work.tfi = t;
    rz_flag_put(AGC_FLAG_ITSWICH, false);
    rz->delvee     = RZ_DPPOSMAX;
    rz->hp         = TPS_MANYFEET;
    tps_work.thetl = tps_kflag() ? TPS_SEC1THET : TPS_SEC2THET;

    return (AGC_STATUS_OK);
}

/* CONCAUL .. JUNCT3 .. PERIAPO (pp.554-556): passive to TTPI + TF (conic),
 * the target; INITVEL (conic, no offsets, 15 deg cone); RELDELV, MAGVTPI,
 * DELVEE; pericenter height HP of the transfer.                        */
static agc_status_t tps_trial(agc_interp_t *ip, agc_rendz_t *rz,
                              agc_job_id_t resume)
{
    rz_integ_kind_t k =
        agc_flag_test(AGC_FLAG_AVFLAG) ? RZ_CSMCONIC : RZ_LEMCONIC;
    agc_integ_out_t o;
    agc_status_t    st =
        rz_integ(ip, k, agc_dp_add(ip, agc_erase_ttpi(), rz->tf), resume, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->vpass4      = o.vatt; /* JUNCT3 */
    rz->rpass4      = o.ratt;
    rz->rtarg       = o.ratt;
    rz->dellt4      = rz->tf;
    tps_work.hpo    = rz->hp;
    tps_work.delveo = rz->delvee;
    rz->intime      = agc_erase_ttpi();
    rz->rinit       = rz->ract3;
    rz->vinit       = rz->vact3;
    st              = agc_rendz_initvel(ip, rz, 0, TPS_EPSFOUR, resume);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->reldelv = rz_abval(ip, agc_vec_sub(ip, rz->vtprime, rz->vpass4));
    rz->magvtpi = rz_abval(ip, rz->delveet3);
    rz->delvee  = agc_dp_add(ip, rz->magvtpi, rz->reldelv);
    agc_rendz_periapo_t pa;

    st     = agc_rendz_periapo1(ip, rz, rz->ract3, rz->viprime, &pa);
    rz->hp = agc_rendz_shiftr1(rz, pa.hperi); /* SL* 0,2 */

    return (st);
}

/* Branch targets of the S17.2 search loop. */
typedef enum tps_label
{
    TPS_L_CONCAUL = 0,
    TPS_L_BIS,
    TPS_L_ENDEN,
    TPS_L_RNGETEST,
    TPS_L_ALARUMS
} tps_label_t;

/* JUNCT1 / JUNCT2: DELTEE = step (minus when KFLAG is on); TF += DELTEE. */
static tps_label_t tps_step(agc_interp_t *ip, agc_rendz_t *rz, bool set,
                            agc_dp_t step)
{
    if (set)
    {
        tps_work.deltee = tps_kflag() ? agc_dp_neg(step) : step; /* JUNCT1 */
    }

    rz->tf = agc_dp_add(ip, tps_work.deltee, rz->tf); /* JUNCT2   */

    return (TPS_L_BIS);
}

/* INTERP (pp.556-557): SET ITSWICH; linear interpolation of the time step
 * to the safe height: DELTEE = DELTEE (HPERMIN + 500 ft - HP)/(HP - HPO),
 * with the AGC's NORM bookkeeping.                                     */
static tps_label_t tps_interp(agc_interp_t *ip, agc_rendz_t *rz)
{
    rz_flag_put(AGC_FLAG_ITSWICH, true);
    int      x1 = 0;
    agc_dp_t a  = rz_norm(agc_dp_sub(ip, rz->hp, tps_work.hpo), &x1);
    int      x2 = 0;
    agc_dp_t b  = agc_dp_add(ip, TPS_DFTMOON, rz->hpermin);

    b  = agc_dp_shr(rz_norm(agc_dp_sub(ip, b, rz->hp), &x2), 1); /* SR1   */
    x2 = x2 - x1;                                                /* XSU,2 X1 */
    agc_dp_t q = agc_dp_mul_trunc(agc_dp_div(ip, b, a), tps_work.deltee);

    tps_work.deltee = rz_sr_star(ip, q, -1, x2); /* SR* 0-1,2 */

    return (tps_step(ip, rz, false, 0));
}

/* After a trial (pp.556-557): forced acceptance (ITSWICH) -> ENDEN; safe
 * pericenter -> HALFSAFE (2 fps convergence, step halving, interpolation
 * when the previous one was unsafe); unsafe -> interpolate when the
 * previous one was safe, alarm when getting worse, else step 400 s.  */
static tps_label_t tps_decide(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (agc_flag_test(AGC_FLAG_ITSWICH))
    {
        return (TPS_L_ENDEN);
    }

    agc_dp_t hpo_margin = agc_dp_sub(ip, rz->hpermin, tps_work.hpo);

    if (agc_dp_sub(ip, rz->hpermin, rz->hp) < 0)
    { /* HALFSAFE */
        agc_dp_t dd = agc_dp_sub(ip, rz->delvee, tps_work.delveo);

        if (agc_dp_sub(ip, agc_dp_abs(dd), TPS_LIMVEL) < 0)
        {
            return (TPS_L_ENDEN);
        }

        if (dd < 0)
        { /* LRGRDVO  */
            return ((hpo_margin < 0) ? tps_step(ip, rz, false, 0) :
                    tps_step(ip, rz, true, TPS_CLSEC));
        }

        if (hpo_margin >= 0)
        {
            return (tps_interp(ip, rz));
        }

        tps_work.deltee = agc_dp_neg(agc_dp_shr(tps_work.deltee, 1));

        return (tps_step(ip, rz, false, 0));
    }

    if (hpo_margin < 0)
    {
        return (tps_interp(ip, rz)); /* SOLUTION AT HAND    */
    }

    if (agc_dp_sub(ip, rz->hp, tps_work.hpo) < 0)
    {
        return (TPS_L_ALARUMS); /* IT'S GETTING WORSE  */
    }

    return (tps_step(ip, rz, true, TPS_CDSEC));
}

/* BIS (p.554): the transfer angle THETA of the last Lambert solution
 * (ARCTRIG of SNTH/2, CSTH/2, 0..1 rev); the search has run out of time
 * when (TF - TFI) signed by (THETL - THETA) is negative.             */
static tps_label_t tps_bis(agc_interp_t *ip, const agc_rendz_t *rz)
{
    agc_dp_t th =
        agc_imu_arctrig(ip, agc_dp_shr(rz->snth, 1), agc_dp_shr(rz->csth, 1));

    if (th < 0)
    {
        th = agc_dp_add(ip, th, RZ_DPPOSMAX);
    }

    agc_dp_t lim = agc_dp_sub(ip, tps_work.thetl, th);
    agc_dp_t d   = agc_dp_sign(agc_dp_sub(ip, rz->tf, tps_work.tfi), lim);

    return ((d < 0) ? TPS_L_RNGETEST : TPS_L_CONCAUL);
}

/* RNGETEST (p.557): out of time; accept if the pericenter is safe, the
 * solution time being TF - DELTEE.                                     */
static tps_label_t tps_rngetest(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (agc_dp_sub(ip, rz->hp, rz->hpermin) < 0)
    {
        return (TPS_L_ALARUMS);
    }

    rz->tf = agc_dp_sub(ip, rz->tf, tps_work.deltee);

    return (TPS_L_ENDEN);
}

/* SRR* by a signed count (negative: shift left). */
static agc_dp_t tps_srr_star(agc_interp_t *ip, agc_dp_t a, int count)
{
    if (!AGC_ASSERT(count >= -60 && count <= 60))
    {
        return (0); /* no valid result */
    }

    return ((count >= 0) ? agc_dp_shr_round(a, count) : rz_shift(ip, a,
                                                                 -count));
}

/* ---- TRANSANG (XANG pp.558-559): central angle of the passive vehicle
 * during the transfer, CENTANG = SQRT(MU/A^3) TF / 2PI revolutions,
 * A = R/(2 - R V^2/MU) of the passive state at TPF (RPASS4 VPASS4).    */
agc_status_t agc_rendz_transang(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool      moon   = (tps_work.xrs[1] != 0);
    int       x2     = tps_work.xrs[1];
    agc_vec_t v      = rz_vshift(ip, rz->vpass4, x2);          /* VSR* 0,2 */
    agc_dp_t  sqrtmu = moon ? TPS_SQRT_MUM : TPS_SQRT_MUE;     /* 00D      */
    agc_dp_t  r      = rz_abval(ip, rz_vshift(ip, rz->rpass4, x2)); /* 02D      */
    agc_dp_t  invmu  = moon ? TPS_INV_MUM : TPS_INV_MUE;       /* 04D      */
    int       x1     = 0;
    agc_dp_t  q      = agc_dp_mul(ip, rz_norm(agc_vec_sq(ip, v), &x1), invmu);

    q = tps_srr_star(ip, agc_dp_mul_trunc(q, r), -3 - x1); /* RV2/MU   */
    q = rz_norm(agc_dp_sub(ip, TPS_D1_32, q), &x1);        /* 2-RV2/MU */
    agc_dp_t a = agc_dp_div(ip, agc_dp_shr_round(r, 1), q);

    a = rz_sl_star(ip, a, -5, x1); /* A  02D   */
    agc_dp_t a3 = agc_dp_mul_trunc(agc_dp_sqrt(ip, agc_dp_shr(a, 1)), a);

    a3 = rz_norm(a3, &x1); /* 00D      */
    agc_dp_t n = agc_dp_div(ip, agc_dp_shr(sqrtmu, 1), a3);

    n = rz_sl_star(ip, agc_dp_mul_trunc(n, rz->tf), 0, x1); /* x TF     */
    agc_dp_t twopi = rz_norm(TPS_2PISC, &x1);

    rz->centang = rz_sl_star(ip, agc_dp_div(ip, n, twopi), -3, x1);

    if (!AGC_ASSERT(rz->centang >= AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* ENDEN .. NEXUS (pp.557-558): RELDELV signed by VTPRIME . RPASS4;
 * TRANSANG; the pericenter code NN1 (1 or 2) from the signs of RELDELV,
 * RACT3 . VIPRIME and KFLAG; POSTTPI = HP.                             */
static agc_status_t tps_enden(agc_interp_t *ip, agc_rendz_t *rz)
{
    rz->reldelv =
        agc_dp_sign(rz->reldelv, agc_vec_dot(ip, rz->vtprime, rz->rpass4));
    agc_status_t st  = agc_rendz_transang(ip, rz);
    agc_dp_t     sg1 =
        agc_dp_sign(agc_vec_dot(ip, rz->ract3, rz->viprime), rz->reldelv);
    agc_sp_t nn1;

    if (sg1 >= 0)
    { /* USEKAY   */
        nn1 = tps_kflag() ? 2 : 1;
    }
    else
    {
        nn1 = (rz->reldelv < 0) ? 2 : 1;
    }

    rz->nn1     = nn1; /* NEXUS    */
    rz->posttpi = rz->hp;

    return (st);
}

/* ---- S17.2 (pp.553-558) ------------------------------------------------
 * OK: solution found (TF, DELVEET3, CENTANG, NN1, POSTTPI); ALARM: no
 * safe pericenter in this sector (ALARUMS); BUSY: an INTSTALL sleep, the
 * job resumes as `resume` and the search starts again.                */
agc_status_t agc_rendz_tpi_search_sector(agc_interp_t *ip, agc_rendz_t *rz,
                                         agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = tps_limits(ip, rz);
    tps_label_t  l  = TPS_L_CONCAUL;

    for (int i = 0; i < AGC_RENDZ_TPISRCH_MAXITER && st == AGC_STATUS_OK; i++)
    {
        switch (l)
        {
            case TPS_L_CONCAUL:
                st = tps_trial(ip, rz, resume);

                if (st == AGC_STATUS_OK)
                {
                    l = tps_decide(ip, rz);
                }

                break;
            case TPS_L_BIS:
                l = tps_bis(ip, rz);
                break;
            case TPS_L_RNGETEST:
                l = tps_rngetest(ip, rz);
                break;
            case TPS_L_ENDEN:
                return (tps_enden(ip, rz));
            case TPS_L_ALARUMS:
                return (AGC_STATUS_ALARM);
            default:
                return (AGC_STATUS_BAD_ARG);
        }
    }

    /* DEVIATION: the AGC search has no iteration bound. */
    return ((st != AGC_STATUS_OK) ? st : AGC_STATUS_NO_CONVERGE);
}

/* ============================ P17 / P77 (pp.559-561) ===================== */
/* VNCOMP17 (p.560): the display it repeats and where PROCEED goes. */
typedef struct tps_display
{
    uint8_t      verb, noun;
    agc_job_id_t proceed;
} tps_display_t;

static tps_display_t tps_display = {6U, 72U, AGC_JOB_TPI_SEARCH_COMPUTE};

/* VNCOMP17: flash; TERMINATE is illegal ("TC -3": the same display),
 * PROCEED -> proceed, V32 -> P17.1, other data loads -> the display. */
static agc_status_t tps_vncomp17(const agc_job_ctx_t *ctx, uint8_t verb,
                                 uint8_t noun, agc_job_id_t proceed)
{
    if (!AGC_ASSERT(proceed != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    tps_display.verb    = verb; /* TS VERBNOUN  */
    tps_display.noun    = noun;
    tps_display.proceed = proceed;

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, verb, noun,
                     AGC_JOB_TPI_SEARCH_DISPLAY, proceed,
                     AGC_JOB_TPI_SEARCH_RESPONSE));
}

/* The VNCOMP17 display again. */
agc_status_t agc_job_tpi_search_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (tps_vncomp17(ctx, tps_display.verb, tps_display.noun,
                         tps_display.proceed));
}

/* ENTER on VNCOMP17: CS MPAC, AD BIT6, BZF P17.1 -- V32 recycles to
 * P17.1, anything else shows the display again.                       */
agc_status_t agc_job_tpi_search_response(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint8_t      verb = 0U;
    uint8_t      noun = 0U;
    agc_status_t st   = agc_dsky_response(&verb, &noun);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (verb == 32U)
    {
        return (agc_job_tpi_search_input(ctx));
    }

    return (agc_job_tpi_search_display(ctx));
}

/* P17 TPI SEARCH (p.559, label P17): AVFLAGA. */
agc_status_t agc_job_tpi_search(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_avflag(true);

    return (agc_job_tpi_search_input(ctx));
}

/* P77 TPI SEARCH, OTHER VEHICLE ACTIVE (label P77): AVFLAGP. */
agc_status_t agc_job_tpi_search_other_vehicle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_avflag(false);

    return (agc_job_tpi_search_input(ctx));
}

/* P17.1: P20FLGON; V06N37 (TTPI). */
agc_status_t agc_job_tpi_search_input(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_tracking_flags_on();

    return (rz_vnpooh(ctx, 6U, 37U, AGC_JOB_TPI_SEARCH_START));
}

/* After V06N37: CLEAR UPDATFLG, S17.1, SET UPDATFLG; OPTION2 = 2 (K
 * positive, KFLAG off) or 1; V06N72 (phase angle, delta H, option). */
agc_status_t agc_job_tpi_search_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();

    agc_flag_clear(AGC_FLAG_UPDATFLG);
    agc_status_t st =
        agc_rendz_tpi_search_phase(&ip, rz, AGC_JOB_TPI_SEARCH_START);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_set(AGC_FLAG_UPDATFLG);
    st = rz_set_option(2, tps_kflag() ? 1U : 2U); /* SXA,1 OPTION2 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (tps_vncomp17(ctx, 6U, 72U, AGC_JOB_TPI_SEARCH_COMPUTE));
}

/* PROCEED on V06N72 (p.560): KFLAG from the option (1: on, 2: off);
 * HPERMIN = 35000 ft (moon) or 85 NM; S17.2; V06N58, then V06N55.
 * No solution: ALARUMS, alarm 00124 and V05N09 (PROCEED ends P17).    */
agc_status_t agc_job_tpi_search_compute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_rendz_t *rz = rz_state();

    agc_flag_clear(AGC_FLAG_UPDATFLG);
    rz_flag_put(AGC_FLAG_KFLAG, rz_option(2) == 1U);
    rz->hpermin = (tps_work.xrs[1] != 0) ? TPS_HPL : TPS_HPE;
    agc_status_t st =
        agc_rendz_tpi_search_sector(&ip, rz, AGC_JOB_TPI_SEARCH_COMPUTE);

    agc_flag_set(AGC_FLAG_UPDATFLG);

    if (st == AGC_STATUS_ALARM)
    { /* ALARUMS      */
        agc_alarm((agc_alarm_code_t) AGC_ALARM_00124);

        return (tps_vncomp17(ctx, 5U, 9U, AGC_JOB_GOTOPOOH));
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (tps_vncomp17(ctx, 6U, 58U, AGC_JOB_TPI_SEARCH_RESULTS)); /* P17.3 */
}

/* After V06N58: V06N55 (pericenter code, central angle) GOFLASHR, R2
 * blanked; TERMINATE, PROCEED -> P00, RECYCLE -> P17.1.               */
agc_status_t agc_job_tpi_search_results(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st =
        rz_flash(ctx, AGC_DSKY_GOFLASHR, 6U, 55U, AGC_JOB_GOTOPOOH,
                 AGC_JOB_GOTOPOOH, AGC_JOB_TPI_SEARCH_INPUT);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(2U)); /* BLANK R2     */
}
