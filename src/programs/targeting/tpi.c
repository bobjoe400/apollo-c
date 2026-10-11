/**********************************************************************
 * FILE NAME: tpi.c
 *
 * PURPOSE:
 *   Transfer phase initiation and midcourse subroutines, both vehicles:
 *     S33/34.1   TPI time search for a given LOS elevation (pp.469-473)
 *     S34/35.2   rendezvous target and required velocity (pp.475-476)
 *     S3435.25   intercept velocity from a given target (pp.475-476)
 *     S34/35.3   crew overwrite of the delta V (p.477)
 *     S34/35.4   NOVRWRT, delta V in line-of-sight axes (pp.477-478)
 *     P34/P74E   the TPI computation proper (pp.464-465)
 *   The push-down list cells of the AGC are named locals; the cell is
 *   given in the comments.
 *
 * AGC SOURCE:
 *   Comanche055 P34-35_P74-75.agc pp. 464-480 (ELCALC, FIGTIME, OKMAX,
 *   ADTIME, GOINT, S34/35.2, S3435.25, S34/35.3, NOVRWRT, P34/P74E)
 *   Luminary099 P34-35_P74-75.agc (identical text)
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   TTPI        agc_dp_t   I/O  cs 2^28    TPI time (N37)
 *   TIG         agc_dp_t   O    cs 2^28    = TTPI at P34/P74E
 *   ETPIFLAG    flag       I    -          elevation not computed
 *   ITSWICH     flag       I    -          iterate TTPI for ELEV
 *   RACT3 VACT3 RPASS3 VPASS3 (agc_rendz_t) I  m 2^29, m/cs 2^7  states
 *   ELEV CENTANG NOMTPI TITER SECMAX DELEL DELELO DELTEEO (agc_rendz_t)
 *               I/O  search state (revolutions 2^0, cs 2^28)
 *   RTARG VPASS4 DELVEET3 DELVLVC DVLOS DELVTPI DELVTPF POSTTPI
 *               O    results (m 2^29, m/cs 2^7)
 *
 * EXTERNAL REFERENCES:
 *   Name                          Description
 *   ----------------------------  ------------------------------------
 *   agc_rendz_los_and_normal      S34/35.1: ULOS, UNRM
 *   rz_intint                     INTINT: conic or precision update
 *   agc_rendz_initvel             INITVEL: Lambert required velocity
 *   agc_rendz_lomat               LOMAT: local vertical matrix
 *   rz_timethet_rv                TIMETHET: transfer time for an angle
 *   agc_rendz_periapo_body_scale  PERIAPO1: post-TPI apsides
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   agc_rendz_tpi_time_search returns AGC_STATUS_ALARM for the AGC's
 *   non-zero MPAC exit (no solution: BMN NORMEX); the caller raises
 *   00611. AGC_STATUS_BAD_ARG for NULL pointers; integration statuses
 *   (INTSTALL sleep, restart) are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs inside the caller's job. The time search ends within
 *   AGC_RENDZ_TPI_MAXITER passes (TITER halved each pass).
 *
 * NOTES:
 *   Scales: r m 2^29, v m/cs 2^7, t cs 2^28, angles revolutions 2^0,
 *   unit vectors 2^1.
 **********************************************************************/
#include "../navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/flags_common.h"

/* ---- constants (P34-35 p.485) ------------------------------------------- */
#define TPI_MAX250  ((agc_dp_t) 25000)    /* MAX250 2DEC 25 E3 B-28: 250 s */
#define TPI_THIRD   ((agc_dp_t) 89478485) /* THIRD 2DEC .333333333         */
#define TPI_ELEPS   ((agc_dp_t) 74565)    /* ELEPS .27777777 E-3: .1 deg   */
#define TPI_EPSFOUR ((agc_dp_t) 11184811) /* EPSFOUR .0416666666: 15 deg   */
/* TITER start: OCT 40000, the SP word with only the sign bit (ones'
 * complement -37777); halved each pass, the search gives up when it
 * reaches zero (AGC_RENDZ_TPI_MAXITER passes).                           */
#define TPI_TITER0   AGC_SP_MIN /* TITER start, OCT 40000 */
#define TPI_TITER1   AGC_SP_MAX /* OCT 37777 after the 1st pass  */
#define TPI_MAX_LOOP (AGC_RENDZ_TPI_MAXITER + 2) /* TITER passes + 2 */

/**********************************************************************
 * FIGTIME .. OKMAX (pp.470-471): time step dt = angle 2pi / (WA - WP),
 * limited to SECMAX. Returns false when there is no solution.
 * AGC: FIGTIME .. OKMAX (P34-35_P74-75.agc pp.470-471)
 *
 * FUNCTION NAME: tpi_figtime
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   dt        agc_dp_t *      O    time step, cs 2^28, limited to SECMAX
 *
 * RETURNS: bool -- true with *dt set; false when the geometry has no
 *          solution (BMN NORMEX)
 * AGC: FIGTIME .. OKMAX (P34-35_P74-75.agc pp.470-471)
 *
 * FUNCTION NAME: tpi_figtime
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   dt        agc_dp_t *      O    time step, cs 2^28, limited to SECMAX
 *
 * RETURNS: bool -- true with *dt set; false when the geometry has no
 *          solution (BMN NORMEX)
 **********************************************************************/
static bool tpi_figtime(agc_interp_t *ip, agc_rendz_t *rz, agc_dp_t *dt)
{
    agc_dp_t  rp = 0;  /* |RP|, m 2^29 */
    agc_dp_t  ra = 0;  /* |RA|, m 2^29 */
    agc_vec_t urp; /* 12D UNIT RP, 2^1 */

    urp = agc_vec_unit(ip, rz->rpass3, &rp, NULL);
    agc_vec_t ura; /* 14D UNIT RA, 2^1 */

    ura = agc_vec_unit(ip, rz->ract3, &ra, NULL);

    if (!AGC_ASSERT(urp.c[0] >= AGC_DP_MIN && ura.c[0] >= AGC_DP_MIN))
    {
        return (false);
    }

    bool fail = false; /* no solution (NORMEX) */
    /* FIGTIME angle part (pp.470-471): the central angle the active vehicle
     * must gain on the passive one, revolutions. *fail when the geometry has
     * no solution (the AGC's BMN NORMEX with a negative MPAC).             */
    agc_dp_t d30 = agc_dp_sub(ip, rp, ra); /* 30D RP - RA, m 2^29 */
    agc_dp_t m; /* rev 2^0 */

    m = agc_dp_sign(agc_dp_sub(ip, RZ_DPHALF, rz->elev), d30);

    fail = (m < 0);
    agc_dp_t c28 = agc_dp_mul_trunc( /* 28D, 2^30 */
        agc_dp_cos(ip, agc_dp_sub(ip, RZ_DPHALF, rz->elev)), ra);

    c28 = agc_dp_neg(agc_dp_div(ip, c28, rp)); /* COS(180-A) = -COS A */

    if (agc_dp_sub(ip, RZ_DPHALF, agc_dp_abs(c28)) < 0)
    {
        fail = true;
    }

    agc_vec_t urp_angle = rz_unit(ip, rz->rpass3); /* 0D UNIT RP, 2^1 */
    agc_vec_t ura_angle = rz_unit(ip, rz->ract3); /* 6D UNIT RA, 2^1 */
    agc_dp_t  s; /* sign only, 2^3 */

    s = agc_vec_dot(ip, agc_vec_cross(ip, ura_angle,
                                      urp_angle), rz->unrm);
    agc_dp_t ang_angle =
        agc_dp_acos(ip, /* rev 2^0 */
                    agc_dp_shl(ip, agc_vec_dot(ip, urp_angle, ura_angle), 1));
    agc_dp_t a1 =
        agc_dp_add(ip, agc_dp_sub(ip, agc_dp_sign(ang_angle, s), RZ_DPHALF),
                   rz->elev); /* rev 2^0 */
    agc_dp_t a2 = /* rev 2^0 */
                  agc_dp_sign(agc_dp_sub(ip, RZ_DPHALF, agc_dp_acos(ip,
                                                                    c28)), d30);

    if (!AGC_ASSERT(a1 >= AGC_DP_MIN && a2 >= AGC_DP_MIN))
    {
        return (false);
    }

    agc_dp_t ang = agc_dp_add(ip, a2, a1); /* angle to gain, rev 2^0 */

    if (fail)
    {
        return (false);
    }

    int x1 = 0; /* NORM index, -(left shifts) */
    /* FIGTIME rate part (pp.470-471): WA - WP, both angular rates times
     * RA RP, normalized; *x1 = the NORM index.                             */
    agc_vec_t ura_rate = rz_unit(ip, rz->ract3); /* UNIT RA, 2^1 */
    agc_vec_t urp_rate = rz_unit(ip, rz->rpass3); /* UNIT RP, 2^1 */
    agc_vec_t ha; /* UN*RA, 2^1 */

    ha = rz_unit(ip, agc_vec_cross(ip, rz->unrm, ura_rate));
    agc_dp_t wa; /* WA RA RP, 2^37 */

    wa = agc_dp_mul_trunc(agc_vec_dot(ip, ha, rz->vact3), rp);
    agc_vec_t hp =
        rz_unit(ip, agc_vec_cross(ip, agc_vec_cross(ip, urp_rate, rz->vpass3),
                                  urp_rate)); /* (RP*VP)*RP, 2^1 */
    agc_dp_t wp; /* WP RA RP, 2^37 */

    wp = agc_dp_mul_trunc(agc_vec_dot(ip, hp, rz->vpass3), ra);
    agc_dp_t w = rz_norm(agc_dp_sub(ip, wa, wp), &x1); /* 2^(37+X1) */
    agc_dp_t t; /* to cs 2^28 below */

    t = agc_dp_div(ip, agc_dp_mul_trunc(ang, RZ_TWOPI_B4), w);

    t = rz_sl_star(ip, agc_dp_mul_trunc(t, ra), -3, x1); /* SL* 0-3,1 */
    t = agc_dp_mul_trunc(t, rp);

    if (agc_dp_sub(ip, agc_dp_abs(t), rz->secmax) >= 0)
    {
        t = agc_dp_sign(rz->secmax, t); /* max time, signed like dt */
    }

    *dt = t;

    return (true);
} /* tpi_figtime */

/**********************************************************************
 * One pass ELCALC .. ADTIME. *done: the search has exited (OK = the AGC
 * zero MPAC: converged, elevation computed, or out of iterations).
 * AGC: ELCALC .. ADTIME (P34-35_P74-75.agc pp.469-472)
 *
 * FUNCTION NAME: tpi_search_pass
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   done      bool *          O    true when the search has exited (TIMEX
 *                                  or NORMEX)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (zero MPAC exit or next pass),
 *          AGC_STATUS_ALARM for no solution, AGC_STATUS_BAD_ARG when an
 *          invariant fails
 * AGC: ELCALC .. ADTIME (P34-35_P74-75.agc pp.469-472)
 *
 * FUNCTION NAME: tpi_search_pass
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   done      bool *          O    true when the search has exited (TIMEX
 *                                  or NORMEX)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (zero MPAC exit or next pass),
 *          AGC_STATUS_ALARM for no solution, AGC_STATUS_BAD_ARG when an
 *          invariant fails
 **********************************************************************/
static agc_status_t tpi_search_pass(agc_interp_t *ip, agc_rendz_t *rz,
                                    bool *done)
{
    *done = true;
    /* ---- ELCALC (p.469): LOS elevation EA of the passive vehicle seen from
     * the active one, revolutions; > .5 rev when the LOS is below the local
     * horizontal (EA = DPPOSMAX - angle).                                   */
    agc_status_t st = agc_rendz_los_and_normal(ip, rz); /* S34/35.1 */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t h; /* 0D UNRM x RA, 2^30 */

    h = agc_vec_cross(ip, rz->unrm, rz->ract3);
    agc_vec_t ura  = rz_unit(ip, rz->ract3); /* UNIT RA, 2^1 */
    agc_vec_t proj = agc_vec_proj(ip, rz->ulos, ura); /* VPROJ, 2^3 */
    agc_vec_t up   = /* 2^1 */
                     rz_unit(ip,
                             agc_vec_sub(ip, rz->ulos,
                                         agc_vec_shl(ip, proj, 2)));
    agc_dp_t hup = agc_vec_dot(ip, h, up); /* 0D, 2^31 */
    agc_dp_t c   = agc_dp_sign(agc_vec_dot(ip, up, rz->ulos), hup); /* 2^2 */
    agc_dp_t ea  = agc_dp_acos(ip, agc_dp_shl(ip, c, 1)); /* EA, rev 2^0 */

    if (agc_vec_dot(ip, rz->ulos, rz->ract3) < 0)
    { /* BPL      */
        ea = agc_dp_sub(ip, RZ_DPPOSMAX, ea);
    }

    if (!AGC_ASSERT(ea >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!rz_vflag_test(RZ_VF_ITSWICH))
    { /* ELEX      */
        agc_erase_set_ttpi(agc_dp_add(ip, agc_erase_ttpi(), rz->nomtpi));

        if (!agc_flag_test(AGC_FLAG_ETPIFLAG))
        {
            rz->elev = ea;
        }

        return (AGC_STATUS_OK); /* TIMEX     */
    }

    rz->delelo = rz->delel; /* TESTY     */
    rz->delel  = agc_dp_sub(ip, ea, rz->elev);

    if (agc_dp_sub(ip, agc_dp_abs(rz->delel), TPI_ELEPS) < 0)
    {
        return (AGC_STATUS_OK); /* TIMEX     */
    }

    agc_dp_t titer; /* TITER SR1, 2^0 */

    titer = agc_dp_shr(agc_dp_from_sp(rz->titer), 1);

    if (agc_dp_high(titer) == 0)
    {
        return (AGC_STATUS_OK); /* too many iterations: MPAC 0 */
    }

    rz->titer = agc_dp_high(titer);
    agc_dp_t dt = 0; /* FIGTIME step, cs 2^28 */

    if (!tpi_figtime(ip, rz, &dt))
    {
        return (AGC_STATUS_ALARM); /* BMN NORMEX     */
    }

    /* OKMAX .. ADTIME (pp.471-472): choose the step from the history of the
     * elevation error (halve and reverse when the solution was crossed or
     * the error grew) and add it to NOMTPI.                                */
    agc_dp_t step; /* time step, cs 2^28 */

    if (rz->titer < 0)
    { /* first pass    */
        rz->titer   = TPI_TITER1;
        step        = dt;
        rz->delteeo = step; /* STORDELT      */
    }
    else if (agc_dp_mul_trunc(rz->delel, rz->delelo) < 0)
    {
        rz->secmax = agc_dp_mul_trunc(rz->secmax, TPI_THIRD); /* crossed */
        step       = agc_dp_sign(agc_dp_neg(agc_dp_shr(agc_dp_abs(dt), 1)),
                                 rz->delteeo);  /* RESIGN        */
        rz->delteeo = step;
    }
    else if (agc_dp_sub(ip, agc_dp_abs(rz->delelo), agc_dp_abs(rz->delel)) < 0)
    { /* REVERS        */
        agc_dp_t back = agc_dp_neg(rz->delteeo); /* cs 2^28 */

        rz->delteeo = agc_dp_shr(back, 1);
        step        = agc_dp_add(ip, rz->delteeo, back);
    }
    else
    { /* NEXTES        */
        step        = agc_dp_sign(agc_dp_abs(dt), rz->delteeo);
        rz->delteeo = step;
    }

    rz->nomtpi = agc_dp_add(ip, step, rz->nomtpi); /* ADTIME        */

    if (!AGC_ASSERT(rz->titer >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *done = false;

    return (AGC_STATUS_OK);
} /* tpi_search_pass */

/**********************************************************************
 * GOINT x 2 (p.472, 479): both saved states advanced by NOMTPI (conic
 * unless NOMTPI = 0), stored by ACTIVE and PASSIVE.
 *
 * FUNCTION NAME: tpi_goint
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
static agc_status_t tpi_goint(agc_interp_t *ip, agc_rendz_t *rz,
                              agc_job_id_t resume)
{
    bool         conic = (rz->nomtpi != 0); /* conic unless NOMTPI = 0 */
    agc_vec_t    r; /* position, m 2^29 */
    agc_vec_t    v; /* velocity, m/cs 2^7 */
    agc_status_t st = rz_intint(ip, rz, rz->raprec, rz->vaprec, 0, rz->nomtpi,
                                conic, resume, &r, &v); /* active */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_store_active(rz, r, v);
    st = rz_intint(ip, rz, rz->rpprec, rz->vpprec, 0, rz->nomtpi, conic, resume,
                   &r, &v);

    if (st == AGC_STATUS_OK)
    {
        rz_store_passive(rz, r, v);
    }

    return (st);
}

/**********************************************************************
 * S33/34.1 iteration: GOINT (both saved states to TTPI + NOMTPI) and one
 * ELCALC .. ADTIME pass, until a pass exits.
 * AGC: GOINT, ELCALC .. ADTIME (P34-35_P74-75.agc pp.469-472)
 *
 * FUNCTION NAME: tpi_search_loop
 *
 * ARGUMENTS:
 *   Argument     Type            I/O  Description (scale)
 *   -----------  --------------  ---  -----------------------------
 *   ip           agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz           agc_rendz_t *   I/O  rendezvous erasables
 *   goint_first  bool            I    true: advance the states before the
 *                                     first pass
 *   resume       agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- the status of the exiting pass (OK or
 *          AGC_STATUS_ALARM), else the integration status (INTSTALL sleep)
 **********************************************************************/
static agc_status_t tpi_search_loop(agc_interp_t *ip, agc_rendz_t *rz,
                                    bool goint_first, agc_job_id_t resume)
{
    for (int pass = 0; pass < TPI_MAX_LOOP; pass++)
    {
        if (goint_first || pass > 0)
        {
            agc_status_t st = tpi_goint(ip, rz, resume); /* GOINT status */

            if (st != AGC_STATUS_OK)
            {
                return (st);
            }
        }

        bool         done = false; /* search exited */
        agc_status_t st   = tpi_search_pass(ip, rz, &done); /* pass status */

        if (done || st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (AGC_STATUS_NO_CONVERGE); /* not reached: TITER bounds the loop */
}

/**********************************************************************
 * S33/34.1 (pp.469-473)
 * AGC: S33/34.1 (P34-35_P74-75.agc pp.469-473)
 *
 * FUNCTION NAME: agc_rendz_tpi_time_search
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (zero MPAC exit), AGC_STATUS_ALARM
 *          (no solution: caller raises 00611), AGC_STATUS_BAD_ARG, else the
 *          integration status
 * AGC: S33/34.1 (P34-35_P74-75.agc pp.469-473)
 *
 * FUNCTION NAME: agc_rendz_tpi_time_search
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (zero MPAC exit), AGC_STATUS_ALARM
 *          (no solution: caller raises 00611), AGC_STATUS_BAD_ARG, else the
 *          integration status
 **********************************************************************/
agc_status_t agc_rendz_tpi_time_search(agc_interp_t *ip, agc_rendz_t *rz,
                                       agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->titer  = TPI_TITER0; /* SSP TITER     */
    rz->secmax = TPI_MAX250;
    rz->raprec = rz->ract3;
    rz->vaprec = rz->vact3;
    rz->rpprec = rz->rpass3;
    rz->vpprec = rz->vpass3;

    return (tpi_search_loop(ip, rz, false, resume));
}

/**********************************************************************
 * S33/34.1 continued at the GOINT step after an INTSTALL sleep.
 * AGC: GOINT (P34-35_P74-75.agc p.472)
 *
 * FUNCTION NAME: agc_rendz_tpi_time_search_cont
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- as agc_rendz_tpi_time_search
 **********************************************************************/
agc_status_t agc_rendz_tpi_time_search_cont(agc_interp_t *ip, agc_rendz_t *rz,
                                            agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (tpi_search_loop(ip, rz, true, resume));
}

/**********************************************************************
 * S34/35.2 (pp.475-476)
 * Passive to TPASS4 (conic if NN = 0, else precision): RTARG, VPASS4,
 * then S3435.25 (agc_rendz_intercept_velocity).
 * AGC: S34/35.2 (P34-35_P74-75.agc pp.475-476)
 *
 * FUNCTION NAME: agc_rendz_tpi_required_velocity
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration or INITVEL status
 * AGC: S34/35.2 (P34-35_P74-75.agc pp.475-476)
 *
 * FUNCTION NAME: agc_rendz_tpi_required_velocity
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration or INITVEL status
 **********************************************************************/
agc_status_t agc_rendz_tpi_required_velocity(agc_interp_t *ip, agc_rendz_t *rz,
                                             agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t    r; /* position, m 2^29 */
    agc_vec_t    v; /* velocity, m/cs 2^7 */
    agc_status_t st; /* to TPASS4 */

    st = rz_intint(ip, rz, rz->rpass3, rz->vpass3, rz->intime,
                   rz->tpass4, rz->nn == 0, resume, &r, &v);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_rendz_intercept_velocity(ip, rz, r, v, resume));
}

/**********************************************************************
 * S3435.25 (p.475-476), also the P38/P39 MAINRTNE entry
 * RTARG = r, VPASS4 = v; ACTCENT = active central angle 0..1 rev;
 * DELLT4 = TPASS4 - INTIME; RINIT/VINIT = RACT3/VACT3; INITVEL with NN
 * offsets and the 15 degree cone; DELVLVC = LOMAT DELVEET3.
 * DEVIATION (open question): the AGC takes "UNIT RA . UNIT RP" from push
 * cell 0D, which holds UNIT RP when called from P34 but the S34/35.1
 * ULOS when called from P35; the documented UNIT RP is used here.
 * AGC: S3435.25 (P34-35_P74-75.agc pp.475-476)
 *
 * FUNCTION NAME: agc_rendz_intercept_velocity
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   rtarg     agc_vec_t       I    target position (passive at TPASS4), m
 *                                  2^29
 *   vpass4    agc_vec_t       I    passive velocity at TPASS4, m/cs 2^7
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the INITVEL or LOMAT status
 * AGC: S3435.25 (P34-35_P74-75.agc pp.475-476)
 *
 * FUNCTION NAME: agc_rendz_intercept_velocity
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   rtarg     agc_vec_t       I    target position (passive at TPASS4), m
 *                                  2^29
 *   vpass4    agc_vec_t       I    passive velocity at TPASS4, m/cs 2^7
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the INITVEL or LOMAT status
 **********************************************************************/
agc_status_t agc_rendz_intercept_velocity(agc_interp_t *ip, agc_rendz_t *rz,
                                          agc_vec_t rtarg, agc_vec_t vpass4,
                                          agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz->rtarg  = rtarg;    /* S3435.25     */
    rz->vpass4 = vpass4;
    agc_vec_t urp = rz_unit(ip, rz->rtarg); /* 0D UNIT RP, 2^1 */
    agc_vec_t ura = rz_unit(ip, rz->ract3); /* UNIT RA, 2^1 */
    agc_dp_t  s; /* sign only, 2^3 */

    s = agc_vec_dot(ip, agc_vec_cross(ip, ura, urp), rz->unrm);
    agc_dp_t c   = agc_dp_shl(ip, agc_vec_dot(ip, ura, urp), 1);  /* cos, 2^1 */
    agc_dp_t phi = agc_dp_sign(agc_dp_acos(ip, c), s); /* ACTCENT, rev 2^0 */

    if (phi < 0)
    {
        phi = agc_dp_add(ip, phi, RZ_DPPOSMAX); /* two pi       */
    }

    rz->actcent = phi; /* NOPIE        */
    rz->dellt4  = agc_dp_sub(ip, rz->tpass4, rz->intime);
    rz->rinit   = rz->ract3;
    rz->vinit   = rz->vact3;
    agc_status_t st; /* INITVEL */

    st = agc_rendz_initvel(ip, rz, rz->nn, TPI_EPSFOUR, resume);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_mat_t m; /* LOMAT / LOS rows, 2^1 */

    st          = agc_rendz_lomat(ip, rz, &m);
    rz->delvlvc = agc_vec_shl(ip, agc_mat_vec(ip, &m, rz->delveet3), 1);

    return (st);
} /* agc_rendz_intercept_velocity */

/**********************************************************************
 * S34/35.3 (p.477): the crew changed DELVLVC on V06N81
 * DELVEET3 = M^T DELVLVC; new target RTARG by conic integration of the
 * new velocity from TIG to TPASS4; NOVRWRT. Here NOVRWRT's MXV 0D finds
 * the LOMAT matrix (push list at 18D), so DVLOS is DELVLVC again; the
 * following S34/35.4 recomputes it in LOS axes.
 * AGC: S34/35.3 (P34-35_P74-75.agc p.477)
 *
 * FUNCTION NAME: agc_rendz_tpi_crew_overwrite
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the LOMAT or integration status
 * AGC: S34/35.3 (P34-35_P74-75.agc p.477)
 *
 * FUNCTION NAME: agc_rendz_tpi_crew_overwrite
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the LOMAT or integration status
 **********************************************************************/
agc_status_t agc_rendz_tpi_crew_overwrite(agc_interp_t *ip, agc_rendz_t *rz,
                                          agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t    m; /* LOMAT / LOS rows, 2^1 */
    agc_status_t st = agc_rendz_lomat(ip, rz, &m); /* LOMAT status */

    rz->delveet3 = agc_vec_shl(ip, agc_vec_mat(ip, rz->delvlvc, &m), 1);
    agc_vec_t vnew = agc_vec_add(ip, rz->delveet3, rz->vact3); /* m/cs 2^7 */
    agc_vec_t r; /* position, m 2^29 */
    agc_vec_t v; /* velocity, m/cs 2^7 */

    if (st == AGC_STATUS_OK)
    {
        st = rz_intint(ip, rz, rz->ract3, vnew, agc_erase_tig(), rz->tpass4,
                       true, resume, &r, &v);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->rtarg = r;
    rz->dvlos = agc_vec_shl(ip, agc_mat_vec(ip, &m, rz->delveet3), 1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * S34/35.4 / NOVRWRT (pp.477-478): DVLOS = delta V in the frame
 * (ULOS, (B x ULOS) VSL1, B), B = UNIT(UNRM x ULOS).
 * AGC: S34/35.4, NOVRWRT (P34-35_P74-75.agc pp.477-478)
 *
 * FUNCTION NAME: agc_rendz_tpi_los_delta_v
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a NULL
 *          pointer
 * AGC: S34/35.4, NOVRWRT (P34-35_P74-75.agc pp.477-478)
 *
 * FUNCTION NAME: agc_rendz_tpi_los_delta_v
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a NULL
 *          pointer
 **********************************************************************/
agc_status_t agc_rendz_tpi_los_delta_v(agc_interp_t *ip, agc_rendz_t *rz)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t m; /* LOMAT / LOS rows, 2^1 */

    m.row[0] = rz->ulos; /* 0D       */
    agc_vec_t b = /* 12D B, 2^1 */
                  rz_unit(ip,
                          agc_vec_neg(agc_vec_cross(ip, rz->ulos, rz->unrm)));

    m.row[1]  = agc_vec_shl(ip, agc_vec_cross(ip, b, rz->ulos), 1); /* 6D   */
    m.row[2]  = b; /* 12D      */
    rz->dvlos = agc_vec_shl(ip, agc_mat_vec(ip, &m, rz->delveet3), 1);

    if (!AGC_ASSERT(rz->dvlos.c[0] != AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * P34/P74E (pp.464-465)
 * Passive transfer time for CENTANG (TIMETHET, time only), TPASS4 =
 * TTPI + T, S34/35.2, DELVTPI, DELVTPF, POSTTPI (pericenter altitude
 * after TPI), TIG = TTPI.
 * AGC: P34/P74E (P34-35_P74-75.agc pp.464-465)
 *
 * FUNCTION NAME: agc_rendz_tpi_compute
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of TIMETHET, S34/35.2 or PERIAPO1
 * AGC: P34/P74E (P34-35_P74-75.agc pp.464-465)
 *
 * FUNCTION NAME: agc_rendz_tpi_compute
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rz        agc_rendz_t *   I/O  rendezvous erasables
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of TIMETHET, S34/35.2 or PERIAPO1
 **********************************************************************/
agc_status_t agc_rendz_tpi_compute(agc_interp_t *ip, agc_rendz_t *rz,
                                   agc_job_id_t resume)
{
    if (!AGC_ASSERT(ip != NULL && rz != NULL) ||
        !AGC_ASSERT(resume != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t             csth = agc_dp_cos(ip, rz->centang); /* CSTH, 2^1 */
    agc_dp_t             snth = agc_dp_sin(ip, rz->centang); /* SNTH, 2^1 */
    agc_conic_time_out_t o; /* TIMETHET result */
    agc_status_t         st = /* TIMETHET */
                              rz_timethet_rv(ip, rz, rz->rpass3, rz->vpass3,
                                             snth, csth, true, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t ttpi = agc_erase_ttpi(); /* TTPI, cs 2^28 */

    rz->intime = ttpi;                      /* FOR INITVEL  */
    rz->tpass4 = agc_dp_add(ip, ttpi, o.t); /* RENDEZVOUS T */
    st         = agc_rendz_tpi_required_velocity(ip, rz, resume);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz->delvtpi = rz_abval(ip, rz->delveet3);
    rz->delvtpf = rz_abval(ip, agc_vec_sub(ip, rz->vpass4, rz->vtprime));
    agc_rendz_periapo_t pa; /* PERIAPO1 results */

    st = agc_rendz_periapo_body_scale(ip, rz, rz->ract3, rz->viprime,
                                      &pa);
    rz->posttpi = agc_rendz_scalar_from_body(rz, pa.hperi);
    agc_erase_set_tig(ttpi);

    return (st);
}
