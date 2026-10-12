/**********************************************************************
 * FILE NAME: tpi_search.c
 *
 * PURPOSE:
 *   P17/P77 TPI SEARCH, CSM only. "The TPI search routine determines the
 *   minimum total velocity transfer trajectory from a given TPI maneuver
 *   time within the constraint of a safe pericenter. This velocity is
 *   the sum of the impulsive velocities for the TPI and TPF maneuvers.
 *   The S17.1 routine extrapolates the state vectors of both vehicles to
 *   the TPI time and computes the relative phase angle between the
 *   vehicles, the altitude difference ... and selects a search sector
 *   based on the sign of the altitude difference. The S17.2 routine
 *   further defines the search sector by computing angular limits and
 *   uses the time theta subroutine to compute the search start and end
 *   times. The search is then made in an iterative loop using the
 *   Lambert subroutine ..."  Names: S17.1 = agc_rendz_tpi_search_phase,
 *   S17.2 = agc_rendz_tpi_search_sector, TRANSANG = agc_rendz_transang.
 *
 * AGC SOURCE:
 *   Comanche055 TPI_SEARCH.agc pp. 551-561 (S17.1, S17.2, CONCAUL,
 *   JUNCT1 .. JUNCT3, HALFSAFE, LRGRDVO, INTERP, BIS, RNGETEST, ENDEN,
 *   USEKAY, NEXUS, ALARUMS, TRANSANG, XANG, P17, P77, P17.1 .. P17.3,
 *   VNCOMP17); CSM only
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   TTPI        agc_dp_t   I    cs 2^28    TPI time (N37)
 *   OPTION2     word       I/O  -          K sign (V06N72)
 *   KFLAG ITSWICH AVFLAG UPDATFLG  flags  I/O  -
 *   THETZERO DELHITE E2 TF HP HPERMIN DELVEE MAGVTPI RELDELV NN1
 *   CENTANG POSTTPI (agc_rendz_t)  O  rev, m 2^29, cs 2^28, m/cs 2^7
 *   TFO TFI IT THETL HPO DELVEO DELTEE XRS (file static)  I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                          Description
 *   ----------------------------  ------------------------------------
 *   rz_integ                      LEMCONIC / CSMCONIC
 *   rz_timethet_rv                TIMETHET
 *   agc_rendz_initvel             INITVEL (conic Lambert)
 *   agc_rendz_periapo_body_scale  PERIAPO1
 *   agc_imu_arctrig               ARCTRIG
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   00124 (ALARUMS: no safe pericenter in the sector) with V05N09.
 *   AGC_STATUS_NO_CONVERGE after AGC_RENDZ_TPISRCH_MAXITER steps
 *   (DEVIATION: the AGC search has no bound). Integration statuses
 *   (INTSTALL sleep) are passed back; the search then starts again.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs of P17 / P77 (CSM).
 *
 * NOTES:
 *   Scales: r m 2^29, v m/cs 2^7, t cs 2^28, angles revolutions
 *   (THETZERO, IT: half revolutions after PIINVERS), heights m 2^29.
 **********************************************************************/
#include "../../programs/navigation/internal.h"
#include "agc/core/assertion.h"
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
    agc_dp_t tfo;    /* TFO: search start time, cs 2^28            */
    agc_dp_t tfi;    /* TFI: search end time, cs 2^28              */
    agc_dp_t it;     /* IT: end angle, half revolutions 2^0 */
    agc_dp_t thetl;  /* THETL: transfer angle limit, rev 2^0 */
    agc_dp_t hpo;    /* HPO: previous HP, m 2^29                   */
    agc_dp_t delveo; /* DELVEO: previous DELVEE, m/cs 2^7          */
    agc_dp_t deltee; /* DELTEE: time step, cs 2^28                 */
    int      xrs[2]; /* XRS: X1 X2 of the conic integration        */
} tps_work_t;

static tps_work_t tps_work; /* S17.2 work cells */

/* KFLAG (agc_flag_test(AGC_FLAG_KFLAG)): off = passive higher (K
 * positive), on = passive lower.                                       */

/**********************************************************************
 * LEMSTORE / CSMSTORE: an integration result as active (AVFLAG matches
 * the vehicle) or passive state.
 * AGC: LEMSTORE, CSMSTORE (P34-35_P74-75.agc p.498)
 *
 * FUNCTION NAME: tps_store
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------
 *   rz        agc_rendz_t *            I/O  rendezvous erasables
 *   lm        bool                     I    true: the LM state
 *   o         const agc_integ_out_t *  I    the integration result
 **********************************************************************/
static void tps_store(agc_rendz_t *rz, bool lm, const agc_integ_out_t *o)
{
    bool lm_active = agc_flag_test(AGC_FLAG_AVFLAG); /* AVFLAG */

    if (lm == lm_active)
    {
        rz_store_active(rz, o->ratt, o->vatt);
    }
    else
    {
        rz_store_passive(rz, o->ratt, o->vatt);
    }
}

/**********************************************************************
 * S17.1 (pp.552-553)
 * AGC: S17.1 (TPI_SEARCH.agc pp.552-553)
 *
 * FUNCTION NAME: agc_rendz_tpi_search_phase
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration or SELECTMU status
 **********************************************************************/
agc_status_t agc_rendz_tpi_search_phase(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_integ_out_t o;                                                            /* integration result */
    agc_status_t    st = rz_integ(ip, RZ_LEMCONIC, agc_erase_ttpi(), resume, &o); /* LM */

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
    agc_dp_t  rp  = 0;                                       /* |RP|, m 2^29 */
    agc_vec_t urp = agc_vec_unit(ip, rz->rpass3, &rp, NULL); /* 0D, 2^1 */

    rz->delhite = agc_dp_sub(ip, rp, rz_abval(ip, rz->ract3)); /* RP - RA */
    rz_flag_put(AGC_FLAG_KFLAG, rz->delhite < 0);              /* ON = -  */
    rz->e2      = rz_unit(ip, agc_vec_cross(ip, urp, rz->vpass3));
    agc_vec_t p;                                               /* m 2^29 */

    p             = agc_vec_shl(ip, agc_vec_proj(ip, rz->ract3, rz->e2), 2);
    agc_vec_t rpa = rz_unit(ip, agc_vec_sub(ip, rz->ract3, p)); /* RPA, 2^1 */
    agc_dp_t  a;                                                /* rev 2^0 */

    a = agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, rpa, urp), 1));
    agc_dp_t s; /* sign, 2^31 */

    s = agc_vec_dot(ip, agc_vec_cross(ip, urp, rz->ract3), rz->e2);

    rz->thetzero    = agc_dp_sign(a, s); /* + ACTIVE AHEAD, - BEHIND  */
    tps_work.xrs[0] = o.x1;              /* XRS        */
    tps_work.xrs[1] = o.x2;

    agc_rendz_selectmu(rz, o.x2 != 0); /* RTX1 RTX2 = XRS (JUNCT3) */

    return (AGC_STATUS_OK);
} /* agc_rendz_tpi_search_phase */

/**********************************************************************
 * SNTH and CSTH = -|cos| of angle (half revolutions) for TIMETHET:
 * "SIN(180-ALPHA)=SIN(ALPHA) ETC" (COS BMN +2 DCOMP).
 * AGC: S17.2 "COS BMN +2 DCOMP", TIMETHET (TPI_SEARCH.agc p.554)
 *
 * FUNCTION NAME: tps_timethet
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   angle     agc_dp_t        I    the angle, revolutions 2^0
 *   t         agc_dp_t *      O    the passive transfer time, cs 2^28
 *
 * RETURNS: agc_status_t -- the TIMETHET status
 **********************************************************************/
static agc_status_t tps_timethet(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t angle, agc_dp_t *t)
{
    agc_dp_t snth = agc_dp_sin(angle); /* SNTH, 2^1 */
    agc_dp_t csth = agc_dp_cos(angle); /* CSTH, 2^1 */

    if (csth >= 0)
    {
        csth = agc_dp_neg(csth);
    }

    agc_conic_time_out_t o;  /* conic result */
    agc_status_t         st; /* TIMETHET */

    st = rz_timethet_rv(ip, rz, rz->rpass3, rz->vpass3, snth, csth, true, &o);

    *t = o.t;

    return (st);
}

/**********************************************************************
 * S17.2 head (pp.553-554): sector limits PHI(0) = 2.5 X - THETZERO and
 * PHI(I) = IT = X - THETZERO, X = (SQRT|URA . E2| SL1 + .002)/PI (sign by
 * KFLAG); start time TF = TFO, end time TFI; DELVEE = DPPOSMAX, HP =
 * MANYFEET, THETL = 70 deg (KFLAG on) or 330 deg; CLEAR ITSWICH.
 * AGC: S17.2 head (TPI_SEARCH.agc pp.553-554)
 *
 * FUNCTION NAME: tps_limits
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the TIMETHET status
 **********************************************************************/
static agc_status_t tps_limits(agc_interp_t *ip, agc_rendz_t *rz)
{
    agc_dp_t x; /* 2^2 */

    x = agc_dp_abs(agc_vec_dot(ip, rz_unit(ip, rz->ract3), rz->e2));

    x = agc_dp_add(ip, agc_dp_shl(ip, agc_dp_sqrt(x), 1), TPS_DP002);

    if (!agc_flag_test(AGC_FLAG_KFLAG))
    {
        x = agc_dp_neg(x); /* BON DCOMP KFLAG    */
    }

    x                 = agc_dp_mul_trunc(x, TPS_PIINVERS);                                /* 0D     */
    tps_work.it       = agc_dp_sub(ip, x, rz->thetzero);                                  /* PHI(I) */
    agc_dp_t     phi0 = agc_dp_add(ip, agc_dp_add(ip, agc_dp_shr(x, 1), x), tps_work.it); /* PHI(0), rev 2^0 */
    agc_dp_t     t    = 0;                                                                /* time, cs 2^28 */
    agc_status_t st   = tps_timethet(ip, rz, phi0, &t);                                   /* TFO */

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

    tps_work.tfi   = t;
    rz_flag_put(AGC_FLAG_ITSWICH, false);
    rz->delvee     = RZ_DPPOSMAX;
    rz->hp         = TPS_MANYFEET;
    tps_work.thetl = TPS_SEC2THET; /* KFLAG off: 330 deg */

    if (agc_flag_test(AGC_FLAG_KFLAG))
    {
        tps_work.thetl = TPS_SEC1THET; /* KFLAG on: 70 deg */
    }

    return (AGC_STATUS_OK);
} /* tps_limits */

/**********************************************************************
 * CONCAUL .. JUNCT3 .. PERIAPO (pp.554-556): passive to TTPI + TF (conic),
 * the target; INITVEL (conic, no offsets, 15 deg cone); RELDELV, MAGVTPI,
 * DELVEE; pericenter height HP of the transfer.
 * AGC: CONCAUL .. JUNCT3 .. PERIAPO (TPI_SEARCH.agc pp.554-556)
 *
 * FUNCTION NAME: tps_trial
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration, INITVEL or
 *          PERIAPO1 status
 **********************************************************************/
static agc_status_t tps_trial(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume)
{
    rz_integ_kind_t k = RZ_LEMCONIC; /* the passive vehicle */

    if (agc_flag_test(AGC_FLAG_AVFLAG))
    {
        k = RZ_CSMCONIC; /* the LM is active */
    }

    agc_integ_out_t o;  /* integration result */
    agc_status_t    st; /* passive */

    st = rz_integ(ip, k, agc_dp_add(ip, agc_erase_ttpi(), rz->tf), resume, &o);

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

    if (st == AGC_STATUS_OK)
    {
        rz->reldelv = rz_abval(ip, agc_vec_sub(ip, rz->vtprime, rz->vpass4));
        rz->magvtpi = rz_abval(ip, rz->delveet3);
        rz->delvee  = agc_dp_add(ip, rz->magvtpi, rz->reldelv);
        agc_rendz_periapo_t pa; /* PERIAPO1 results */

        st     = agc_rendz_periapo_body_scale(ip, rz, rz->ract3, rz->viprime, &pa);
        rz->hp = agc_rendz_scalar_from_body(rz, pa.hperi); /* SL* 0,2 */
    }

    return (st);
} /* tps_trial */

/* Branch targets of the S17.2 search loop. */
typedef enum tps_label
{
    TPS_L_CONCAUL = 0,
    TPS_L_BIS,
    TPS_L_ENDEN,
    TPS_L_RNGETEST,
    TPS_L_ALARUMS
} tps_label_t;

/**********************************************************************
 * JUNCT1 / JUNCT2: DELTEE = step (minus when KFLAG is on); TF += DELTEE.
 * AGC: JUNCT1, JUNCT2 (TPI_SEARCH.agc p.556)
 *
 * FUNCTION NAME: tps_step
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   set       bool            I    true: DELTEE = step (JUNCT1), false:
 *                                  keep DELTEE
 *   step      agc_dp_t        I    the step, cs 2^28
 *
 * RETURNS: tps_label_t -- TPS_L_BIS
 **********************************************************************/
static tps_label_t tps_step(agc_interp_t *ip, agc_rendz_t *rz, bool set, agc_dp_t step)
{
    if (set)
    { /* JUNCT1 */
        tps_work.deltee = step;

        if (agc_flag_test(AGC_FLAG_KFLAG))
        {
            tps_work.deltee = agc_dp_neg(step); /* passive lower */
        }
    }

    rz->tf = agc_dp_add(ip, tps_work.deltee, rz->tf); /* JUNCT2   */

    return (TPS_L_BIS);
}

/**********************************************************************
 * INTERP (pp.556-557): SET ITSWICH; linear interpolation of the time step
 * to the safe height: DELTEE = DELTEE (HPERMIN + 500 ft - HP)/(HP - HPO),
 * with the AGC's NORM bookkeeping.
 * AGC: INTERP (TPI_SEARCH.agc pp.556-557)
 *
 * FUNCTION NAME: tps_interp
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: tps_label_t -- TPS_L_BIS
 **********************************************************************/
static tps_label_t tps_interp(agc_interp_t *ip, agc_rendz_t *rz)
{
    rz_flag_put(AGC_FLAG_ITSWICH, true);
    int      x1 = 0; /* NORM index, -(left shifts) */
    agc_dp_t a;      /* 2^(29+X1) */

    a           = rz_norm(agc_dp_sub(ip, rz->hp, tps_work.hpo), &x1);
    int      x2 = 0;                                        /* NORM index, -(left shifts) */
    agc_dp_t b  = agc_dp_add(ip, TPS_DFTMOON, rz->hpermin); /* m 2^29 */

    b  = agc_dp_shr(rz_norm(agc_dp_sub(ip, b, rz->hp), &x2), 1); /* SR1   */
    x2 = x2 - x1;                                                /* XSU,2 X1 */
    agc_dp_t q;                                                  /* cs, scale fixed by the SR* */

    q = agc_dp_mul_trunc(agc_dp_div(ip, b, a), tps_work.deltee);

    tps_work.deltee = rz_sr_star(ip, q, -1, x2); /* SR* 0-1,2 */

    return (tps_step(ip, rz, false, 0));
}

/**********************************************************************
 * After a trial (pp.556-557): forced acceptance (ITSWICH) -> ENDEN; safe
 * pericenter -> HALFSAFE (2 fps convergence, step halving, interpolation
 * when the previous one was unsafe); unsafe -> interpolate when the
 * previous one was safe, alarm when getting worse, else step 400 s.
 * AGC: HALFSAFE, LRGRDVO (TPI_SEARCH.agc pp.556-557)
 *
 * FUNCTION NAME: tps_decide
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: tps_label_t -- the next label (ENDEN, BIS or ALARUMS)
 **********************************************************************/
static tps_label_t tps_decide(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (agc_flag_test(AGC_FLAG_ITSWICH))
    {
        return (TPS_L_ENDEN);
    }

    agc_dp_t hpo_margin; /* m 2^29 */

    hpo_margin = agc_dp_sub(ip, rz->hpermin, tps_work.hpo);

    if (agc_dp_sub(ip, rz->hpermin, rz->hp) < 0)
    { /* HALFSAFE */
        agc_dp_t dd; /* m/cs 2^7 */

        dd = agc_dp_sub(ip, rz->delvee, tps_work.delveo);

        if (agc_dp_sub(ip, agc_dp_abs(dd), TPS_LIMVEL) < 0)
        {
            return (TPS_L_ENDEN);
        }

        if (dd < 0 && hpo_margin < 0)
        { /* LRGRDVO: the previous one unsafe */
            return (tps_step(ip, rz, false, 0));
        }

        if (dd < 0)
        { /* LRGRDVO: step 150 s */
            return (tps_step(ip, rz, true, TPS_CLSEC));
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
} /* tps_decide */

/**********************************************************************
 * BIS (p.554): the transfer angle THETA of the last Lambert solution
 * (ARCTRIG of SNTH/2, CSTH/2, 0..1 rev); the search has run out of time
 * when (TF - TFI) signed by (THETL - THETA) is negative.
 * AGC: BIS (TPI_SEARCH.agc p.554)
 *
 * FUNCTION NAME: tps_bis
 *
 * ARGUMENTS:
 *   Argument  Type                 I/O  Description (scale)
 *   --------  -------------------  ---  ---------------------------
 *   ip        agc_interp_t *       I/O  interpreter state (OVFIND)
 *   rz        const agc_rendz_t *  I    rendezvous erasables
 *
 * RETURNS: tps_label_t -- TPS_L_RNGETEST (out of time) or TPS_L_CONCAUL
 **********************************************************************/
static tps_label_t tps_bis(agc_interp_t *ip, const agc_rendz_t *rz)
{
    agc_dp_t th; /* rev 2^0 */

    th = agc_imu_arctrig(ip, agc_dp_shr(rz->snth, 1), agc_dp_shr(rz->csth, 1));

    if (th < 0)
    {
        th = agc_dp_add(ip, th, RZ_DPPOSMAX);
    }

    agc_dp_t lim = agc_dp_sub(ip, tps_work.thetl, th); /* rev 2^0 */
    agc_dp_t d;                                        /* cs 2^28 */

    d = agc_dp_sign(agc_dp_sub(ip, rz->tf, tps_work.tfi), lim);

    if (d < 0)
    {
        return (TPS_L_RNGETEST); /* out of time */
    }

    return (TPS_L_CONCAUL);
}

/**********************************************************************
 * RNGETEST (p.557): out of time; accept if the pericenter is safe, the
 * solution time being TF - DELTEE.
 * AGC: RNGETEST (TPI_SEARCH.agc p.557)
 *
 * FUNCTION NAME: tps_rngetest
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: tps_label_t -- TPS_L_ENDEN or TPS_L_ALARUMS
 **********************************************************************/
static tps_label_t tps_rngetest(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (agc_dp_sub(ip, rz->hp, rz->hpermin) < 0)
    {
        return (TPS_L_ALARUMS);
    }

    rz->tf = agc_dp_sub(ip, rz->tf, tps_work.deltee);

    return (TPS_L_ENDEN);
}

/**********************************************************************
 * TRANSANG (XANG pp.558-559): central angle of the passive vehicle
 * during the transfer, CENTANG = SQRT(MU/A^3) TF / 2PI revolutions,
 * A = R/(2 - R V^2/MU) of the passive state at TPF (RPASS4 VPASS4).
 * AGC: TRANSANG, XANG (TPI_SEARCH.agc pp.558-559)
 *
 * FUNCTION NAME: agc_rendz_transang
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG on a bad
 *          argument
 **********************************************************************/
agc_status_t agc_rendz_transang(agc_interp_t *ip, agc_rendz_t *rz)
{
    int       x2     = tps_work.xrs[1];               /* X2: 0 or 2 */
    agc_vec_t v      = rz_vshift(ip, rz->vpass4, x2); /* VSR* 0,2, m/cs 2^7 */
    agc_dp_t  sqrtmu = TPS_SQRT_MUE;                  /* 00D, B-18 / B-15 */
    agc_dp_t  invmu  = TPS_INV_MUE;                   /* 04D, 1/MU 2^34 / 2^28 */
    agc_dp_t  r;                                      /* 02D, m 2^29 */

    r = rz_abval(ip, rz_vshift(ip, rz->rpass4, x2));

    if (x2 != 0)
    { /* moon */
        sqrtmu = TPS_SQRT_MUM;
        invmu  = TPS_INV_MUM;
    }

    int      x1 = 0; /* NORM index, -(left shifts) */
    agc_dp_t q;      /* DMPR, 2^(6+X1) */

    q              = agc_dp_mul(ip, rz_norm(agc_vec_sq(ip, v), &x1), invmu);
    agc_dp_t rv    = agc_dp_mul_trunc(q, r); /* DMP 02D, 2^(35+X1) */
    int      count = -3 - x1;                /* SRR* 0-3,1 */

    if (!AGC_ASSERT(count >= -60 && count <= 60))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    q = rz_shift(ip, rv, -count); /* negative count: a left shift */

    if (count >= 0)
    {
        q = agc_dp_shr_round(rv, count); /* RV2/MU, rounded */
    }

    q          = rz_norm(agc_dp_sub(ip, TPS_D1_32, q), &x1); /* 2-RV2/MU */
    agc_dp_t a = agc_dp_div(ip, agc_dp_shr_round(r, 1), q);  /* A, 2^(24+X1) */

    a = rz_sl_star(ip, a, -5, x1); /* A  02D   */
    agc_dp_t a3;                   /* A^1.5, 2^43 / 2^40 */

    a3 = agc_dp_mul_trunc(agc_dp_sqrt(agc_dp_shr(a, 1)), a);

    a3         = rz_norm(a3, &x1);                          /* 00D      */
    agc_dp_t n = agc_dp_div(ip, agc_dp_shr(sqrtmu, 1), a3); /* n, scale by X1 */

    n              = rz_sl_star(ip, agc_dp_mul_trunc(n, rz->tf), 0, x1); /* x TF     */
    agc_dp_t twopi = rz_norm(TPS_2PISC, &x1);                            /* 2PI, B-6 normalized */

    rz->centang = rz_sl_star(ip, agc_dp_div(ip, n, twopi), -3, x1);

    return (AGC_STATUS_OK);
} /* agc_rendz_transang */

/**********************************************************************
 * ENDEN .. NEXUS (pp.557-558): RELDELV signed by VTPRIME . RPASS4;
 * TRANSANG; the pericenter code NN1 (1 or 2) from the signs of RELDELV,
 * RACT3 . VIPRIME and KFLAG; POSTTPI = HP.
 * AGC: ENDEN, USEKAY, NEXUS (TPI_SEARCH.agc pp.557-558)
 *
 * FUNCTION NAME: tps_enden
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- the TRANSANG status
 **********************************************************************/
static agc_status_t tps_enden(agc_interp_t *ip, agc_rendz_t *rz)
{
    rz->reldelv     = agc_dp_sign(rz->reldelv, agc_vec_dot(ip, rz->vtprime, rz->rpass4));
    agc_status_t st = agc_rendz_transang(ip, rz); /* TRANSANG */
    agc_dp_t     sg1;                             /* sign only, unscaled */

    sg1 = agc_dp_sign(agc_vec_dot(ip, rz->ract3, rz->viprime), rz->reldelv); /* sign only */
    agc_sp_t nn1;                                                            /* NN1, 1 or 2, unscaled */

    bool two = (rz->reldelv < 0); /* NN1 = 2 */

    if (sg1 >= 0)
    { /* USEKAY   */
        two = agc_flag_test(AGC_FLAG_KFLAG);
    }

    nn1 = 1;

    if (two)
    {
        nn1 = 2;
    }

    rz->nn1     = nn1; /* NEXUS    */
    rz->posttpi = rz->hp;

    return (st);
}

/**********************************************************************
 * S17.2 (pp.553-558)
 * OK: solution found (TF, DELVEET3, CENTANG, NN1, POSTTPI); ALARM: no
 * safe pericenter in this sector (ALARUMS); BUSY: an INTSTALL sleep, the
 * job resumes as `resume` and the search starts again.
 * AGC: S17.2 (TPI_SEARCH.agc pp.553-558)
 *
 * FUNCTION NAME: agc_rendz_tpi_search_sector
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (solution), AGC_STATUS_ALARM
 *          (ALARUMS), AGC_STATUS_NO_CONVERGE (bound), AGC_STATUS_BAD_ARG,
 *          else the integration status
 **********************************************************************/
agc_status_t agc_rendz_tpi_search_sector(agc_interp_t *ip, agc_rendz_t *rz, agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) || !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = tps_limits(ip, rz); /* limits */
    tps_label_t  l  = TPS_L_CONCAUL;      /* next label */

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

    if (st != AGC_STATUS_OK)
    {
        return (st); /* INTSTALL sleep or failure */
    }

    return (AGC_STATUS_NO_CONVERGE); /* DEVIATION: the AGC has no bound */
} /* agc_rendz_tpi_search_sector */

/* ============================ P17 / P77 (pp.559-561) ===================== */
/* VNCOMP17 (p.560): the display it repeats and where PROCEED goes. */
typedef struct tps_display
{
    uint8_t      verb;    /* verb of VERBNOUN          */
    uint8_t      noun;    /* noun of VERBNOUN          */
    agc_job_id_t proceed; /* PROCEED continuation      */
} tps_display_t;

static tps_display_t tps_display = {6U, 72U, AGC_JOB_TPI_SEARCH_COMPUTE};

/**********************************************************************
 * VNCOMP17: flash; TERMINATE is illegal ("TC -3": the same display),
 * PROCEED -> proceed, V32 -> P17.1, other data loads -> the display.
 * AGC: VNCOMP17 (TPI_SEARCH.agc p.560)
 *
 * FUNCTION NAME: tps_compute_display
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ctx       const agc_job_ctx_t *  I    job context of the calling job
 *   verb      uint8_t                I    verb of the display
 *   noun      uint8_t                I    noun of the display
 *   proceed   agc_job_id_t           I    job for PROCEED
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t tps_compute_display(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t proceed)
{
    if (!AGC_ASSERT(proceed != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    tps_display.verb    = verb; /* TS VERBNOUN  */
    tps_display.noun    = noun;
    tps_display.proceed = proceed;

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, verb, noun, AGC_JOB_TPI_SEARCH_DISPLAY, proceed, AGC_JOB_TPI_SEARCH_RESPONSE));
}

/**********************************************************************
 * The VNCOMP17 display again.
 * AGC: VNCOMP17 "TC -3" (TPI_SEARCH.agc p.560)
 *
 * FUNCTION NAME: agc_job_tpi_search_display
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_search_display(agc_job_ctx_t *ctx)
{
    return (tps_compute_display(ctx, tps_display.verb, tps_display.noun, tps_display.proceed));
}

/**********************************************************************
 * ENTER on VNCOMP17: CS MPAC, AD BIT6, BZF P17.1 -- V32 recycles to
 * P17.1, anything else shows the display again.
 * AGC: VNCOMP17 ENTER (TPI_SEARCH.agc p.560)
 *
 * FUNCTION NAME: agc_job_tpi_search_response
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_search_response(agc_job_ctx_t *ctx)
{
    uint8_t verb = 0U; /* verb keyed in */
    uint8_t noun = 0U; /* noun keyed in */

    agc_dsky_response(&verb, &noun);

    if (verb == 32U)
    {
        return (agc_job_tpi_search_input(ctx));
    }

    return (agc_job_tpi_search_display(ctx));
}

/**********************************************************************
 * P17 TPI SEARCH (p.559, label P17): AVFLAGA.
 * AGC: P17 (TPI_SEARCH.agc p.559)
 *
 * FUNCTION NAME: agc_job_tpi_search
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_search(agc_job_ctx_t *ctx)
{
    rz_avflag(true);

    return (agc_job_tpi_search_input(ctx));
}

/**********************************************************************
 * P77 TPI SEARCH, OTHER VEHICLE ACTIVE (label P77): AVFLAGP.
 * AGC: P77 (TPI_SEARCH.agc p.559)
 *
 * FUNCTION NAME: agc_job_tpi_search_other_vehicle
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_search_other_vehicle(agc_job_ctx_t *ctx)
{
    rz_avflag(false);

    return (agc_job_tpi_search_input(ctx));
}

/**********************************************************************
 * P17.1: P20FLGON; V06N37 (TTPI).
 * AGC: P17.1 (TPI_SEARCH.agc p.559)
 *
 * FUNCTION NAME: agc_job_tpi_search_input
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_search_input(agc_job_ctx_t *ctx)
{
    rz_tracking_flags_on();

    return (rz_vnpooh(ctx, 6U, 37U, AGC_JOB_TPI_SEARCH_START));
}

/**********************************************************************
 * After V06N37: CLEAR UPDATFLG, S17.1, SET UPDATFLG; OPTION2 = 2 (K
 * positive, KFLAG off) or 1; V06N72 (phase angle, delta H, option).
 * AGC: P17.1 after V06N37 (TPI_SEARCH.agc p.559)
 *
 * FUNCTION NAME: agc_job_tpi_search_start
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_search_start(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    agc_flag_clear(AGC_FLAG_UPDATFLG);
    agc_status_t st; /* S17.1 */

    st = agc_rendz_tpi_search_phase(&ip, rz, AGC_JOB_TPI_SEARCH_START);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_flag_set(AGC_FLAG_UPDATFLG);
    uint16_t opt = 2U; /* KFLAG off: K positive */

    if (agc_flag_test(AGC_FLAG_KFLAG))
    {
        opt = 1U; /* K negative */
    }

    st = rz_set_option(2, opt); /* SXA,1 OPTION2 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (tps_compute_display(ctx, 6U, 72U, AGC_JOB_TPI_SEARCH_COMPUTE));
}

/**********************************************************************
 * PROCEED on V06N72 (p.560): KFLAG from the option (1: on, 2: off);
 * HPERMIN = 35000 ft (moon) or 85 NM; S17.2; V06N58, then V06N55.
 * No solution: ALARUMS, alarm 00124 and V05N09 (PROCEED ends P17).
 * AGC: P17.2, ALARUMS (TPI_SEARCH.agc p.560)
 *
 * FUNCTION NAME: agc_job_tpi_search_compute
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_search_compute(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_rendz_t *rz = agc_rendz_erasable(); /* rendezvous erasables */

    agc_flag_clear(AGC_FLAG_UPDATFLG);
    rz_flag_put(AGC_FLAG_KFLAG, rz_option(2) == 1U);
    rz->hpermin = TPS_HPE; /* earth: 85 NM */

    if (tps_work.xrs[1] != 0)
    {
        rz->hpermin = TPS_HPL; /* moon: 35000 ft */
    }

    agc_status_t st; /* S17.2 */

    st = agc_rendz_tpi_search_sector(&ip, rz, AGC_JOB_TPI_SEARCH_COMPUTE);

    agc_flag_set(AGC_FLAG_UPDATFLG);

    if (st == AGC_STATUS_ALARM)
    { /* ALARUMS      */
        agc_alarm((agc_alarm_code_t) AGC_ALARM_00124);

        return (tps_compute_display(ctx, 5U, 9U, AGC_JOB_GOTOPOOH));
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* P17.3: V06N58 */
    return (tps_compute_display(ctx, 6U, 58U, AGC_JOB_TPI_SEARCH_RESULTS));
}

/**********************************************************************
 * After V06N58: V06N55 (pericenter code, central angle) GOFLASHR, R2
 * blanked; TERMINATE, PROCEED -> P00, RECYCLE -> P17.1.
 * AGC: P17.3 (TPI_SEARCH.agc p.561)
 *
 * FUNCTION NAME: agc_job_tpi_search_results
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ctx       agc_job_ctx_t *  I    job context (checked, not used)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_tpi_search_results(agc_job_ctx_t *ctx)
{
    agc_status_t st = rz_flash(ctx, AGC_DSKY_GOFLASHR, 6U, 55U, AGC_JOB_GOTOPOOH, AGC_JOB_GOTOPOOH, AGC_JOB_TPI_SEARCH_INPUT); /* V06N55 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_blanket(2U)); /* BLANK R2     */
}
