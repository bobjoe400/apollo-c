/**********************************************************************
 * FILE NAME: navigation.c
 *
 * PURPOSE:
 *   P20 RENDEZVOUS NAVIGATION, P22 LUNAR SURFACE NAVIGATION (PROG22 =
 *   PROG20) and P25 PREFERRED TRACKING ATTITUDE, LM only. "The purpose
 *   of this program is to control the rendezvous radar from startup
 *   through acquisition and lockon to the CSM and to update either the
 *   LM or CSM state vector (as specified by the astronaut by DSKY entry)
 *   on the basis of the RR tracking data." Here: the program flow PROG20
 *   .. P20LEMD2, the surface option of P22 (ORBCHGO .. ORBCHG3: the CSM
 *   orbit assumed to change before launch), UPPSV (permanent state
 *   vectors to now), LPS20.1 (LOS from the state vectors) and LPS20.2
 *   (400 NM check), P25, LUNSFCHK, and the LM rendezvous noun data.
 *   R21/R23/R24 are in lm/radar/acquisition.c, R22 in
 *   lm/navigation/tracking_data.c, R61/R65 in
 *   lm/navigation/tracking_attitude.c.
 *
 * AGC SOURCE:
 *   Luminary099 P20-P25.agc pp. 492-503 (PROG20, PROG22, LUNSFCHK,
 *   ORBCHGO .. ORBCHG3, PROG20A, P20LEM1, 526ALARM, P20LEMA .. P20LEMD2,
 *   P20LEMWT, UPPSV, PROG25, P25LEM1, P25LEM2), p. 507, pp. 564-567
 *   (LPS20.1, LPS20.2); LM only
 *
 * EXTERNAL VARIABLES:
 *   Name        Type       I/O  Scale      Description
 *   ----------  ---------  ---  ---------  ---------------------------
 *   REFSMMAT    agc_mat_t  I    2^1        reference to SM
 *   TIG TETCSM TETLEM  agc_dp_t  I  cs 2^28  launch / state times
 *   MODREG      agc_sp_t   I    -          P20 or P22
 *   CHAN33      channel    I    -          RR auto discrete
 *   TRKMKCNT    agc_sp_t   O    -          mark counter (zeroed)
 *   RRTARGET MLOSV LOSVEL (rr_state_t) O  2^1, m 2^20, m/cs 2^7
 *   LMPOS LMVEL LNCHTM R65CNTR (lmnav_state_t) I/O  m 2^29, m/cs 2^7
 *   RNDVZFLG TRACKFLG UPDATFLG SURFFLAG VEHUPFLG RENDWFLG LOSCMFLG
 *   SRCHOPTN ACMODFLG NORRMON R04FLAG P25FLAG RRNBSW ...  flags  I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ----------------------------------------------
 *   rz_integ              LEMCONIC / CSMCONIC (LPS20.1)
 *   agc_integ_integrv     INTEGRV (UPPSV, ORBCHG3)
 *   agc_conic_timethet    TIMETHET (ORBCHG3)
 *   agc_imu_status_check  R02BOTH
 *   lmnav_tracking_attitude, agc_job_rendezvous_sighting_mark  R61, R21
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   00526 (range over 400 NM), 00514 (RR out of auto outside P20/P22),
 *   the R02BOTH alarms. Display, waitlist and integration statuses are
 *   passed back (an INTSTALL sleep resumes the job from its start).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Jobs and tasks of P20, P22 and P25 (LM).
 *
 * NOTES:
 *   Scales: r m 2^29 (moon-centred states in the integration at 2^27),
 *   v m/cs 2^7, t cs 2^28, LOS half unit, MLOSV m 2^20.
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "../radar/radar.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "agc/io/nouns.h"
#include "agc/io/service.h"

#define NAV_FHNM            ((agc_dp_t) 189644800) /* FHNM: 400 NM, m 2^20 */
#define NAV_ONEB_2          ((agc_dp_t) 67108864) /* ONEB-2: 1.0 B-2 */
#define NAV_250DEC          250U                 /* 250DEC: 2.5 s             */
#define NAV_1500DEC         1500U                /* 1500DEC: 15 s             */
#define NAV_60SCNDS         6000U                /* 60SCNDS DEC 6000          */
#define NAV_MODE_RENDEZVOUS 20                   /* OCT24 = 20 decimal        */
#define NAV_MODE_SURFACE    22 /* P22 */
#define NAV_PRIO26          ((agc_prio_t) 026) /* PRIO26 */
#define NAV_PRIO14          ((agc_prio_t) 014) /* PRIO14 */
#define NAV_OPTION_CODE     012U /* OCT00012: option 12 (N06) */
#define NAV_CHECKLIST_AUTO  0201U /* OCT201: select RR auto    */
#define NAV_ALARM_RANGE     ((agc_alarm_code_t) 00526) /* range > 400 NM      */
#define NAV_ALARM_AUTO      ((agc_alarm_code_t) 00514) /* RR not in auto */
#define NAV_CH33_RR_AUTO    000002u /* chan 33 bit 2 (0 = auto)  */

static lmnav_state_t lmnav; /* LM navigation erasables */

/**********************************************************************
 * The LM rendezvous navigation erasables (P20-P25).
 * AGC: ERASABLE_ASSIGNMENTS.agc (P20-P25 erasables)
 *
 * FUNCTION NAME: lmnav_state
 *
 * ARGUMENTS: none
 *
 * RETURNS: lmnav_state_t * -- the LM navigation erasables (never NULL)
 **********************************************************************/
lmnav_state_t *lmnav_state(void)
{
    return (&lmnav);
}

/**********************************************************************
 * LPS20.1 (pp.564-565)
 * "1) Extrapolate the LEM and CSM vectors ... 2) computes the LOS vector
 * to the CSM, converts it to stable member coordinates and stores it in
 * RRTARGET. 3) computes the magnitude of the LOS vector and stores it in
 * MLOSV." In R21 on the lunar surface (LOSCMFLG and SURFFLAG) the LM is
 * not integrated (LMPOS from UPPSV). In P20 (RNDVZFLG) the LOS is shifted
 * up 9 places for precision; an overflow means more than 400 NM.
 * AGC: LPS20.1 (P20-P25.agc pp.564-565)
 *
 * FUNCTION NAME: lmnav_los
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   tdec1     agc_dp_t        I    TDEC1, time of the LOS, cs 2^28
 *   resume    agc_job_id_t    I    job that continues the caller
 *   too_far   bool *          O    true when the shifted LOS overflowed
 *                                  (over 400 NM)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the integration status (INTSTALL sleep)
 **********************************************************************/
agc_status_t lmnav_los(agc_interp_t *ip, agc_dp_t tdec1, agc_job_id_t resume,
                       bool *too_far)
{
    if (!AGC_ASSERT(ip != NULL && too_far != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *too_far = false;
    agc_integ_out_t o; /* integration result */
    agc_status_t    st; /* integration status */

    if (!agc_flag_test(AGC_FLAG_LOSCMFLG) || !agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        st = rz_integ(ip, RZ_LEMCONIC, tdec1, resume, &o); /* LMINT  */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        lmnav.lmpos = o.ratt;
        lmnav.lmvel = o.vatt;
        tdec1       = o.tat;
    }

    st = rz_integ(ip, RZ_CSMCONIC, tdec1, resume, &o); /* CSMINT */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_mat_t   refsmmat = agc_erase_refsmmat(); /* REFSMMAT, 2^1 */
    rr_state_t *s        = rr_state(); /* radar erasables */
    agc_vec_t   dv       = agc_vec_sub(ip, o.vatt, lmnav.lmvel); /* m/cs 2^7 */

    agc_inhint(); /* DODES uses these erasables */
    agc_waitlist_kill(AGC_TASK_RR_DESIGNATE_LOOP);
    agc_relint();
    s->losvel = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, dv), 1);
    agc_vec_t los = agc_vec_sub(ip, o.ratt, lmnav.lmpos); /* m 2^29 */

    if (agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        agc_interp_clear_ovf(ip); /* BOVB     */
        los = agc_vec_shl(ip, los, 9);
    }

    agc_vec_t u = agc_vec_unit(ip, los, &s->mlosv, NULL); /* NOTSHIFT, 2^1 */

    *too_far    = agc_interp_test_ovf(ip);
    s->rrtarget = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, u), 1);
    rz_flag_put(AGC_FLAG_RRNBSW, false);

    return (AGC_STATUS_OK);
} /* lmnav_los */

/**********************************************************************
 * LPS20.2 (p.566): the LOS is 400 NM or less.
 * AGC: LPS20.2 (P20-P25.agc p.566)
 *
 * FUNCTION NAME: lmnav_range_ok
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when MLOSV is under 400 NM
 **********************************************************************/
bool lmnav_range_ok(void)
{
    agc_dp_t m = rr_state()->mlosv; /* MLOSV, m 2^20 */

    if (!AGC_ASSERT(m >= 0))
    {
        return (false);
    }

    return (m < NAV_FHNM);
}

/**********************************************************************
 * UPPSV (pp.499-501): the permanent state vectors to TDEC1, the
 * updated vehicle with its W matrix (9x9, 6x6 on the surface) when it is
 * valid, the other vehicle to the same time without; on the surface
 * LMPOS / LMVEL from the LM conic state. INTEGRV releases INTSTALL.
 * AGC: UPPSV (P20-P25.agc pp.499-501)
 *
 * FUNCTION NAME: nav_uppsv_one
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   csm       bool            I    VINTFLAG: integrate the CSM
 *   with_w    bool            I    integrate the W matrix too
 *   tdec1     agc_dp_t        I    TDEC1, cs 2^28
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the INTSTALL / INTEGRV
 *          status
 **********************************************************************/
static agc_status_t nav_uppsv_one(agc_interp_t *ip, bool csm, bool with_w,
                                  agc_dp_t tdec1, agc_job_id_t resume)
{
    agc_status_t st = agc_integ_install(0, resume); /* INTSTALL */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_integ_setiflgs();

    if (with_w)
    {
        rz_flag_put(AGC_FLAG_DIM0FLAG, true);
        rz_flag_put(AGC_FLAG_D6OR9FLG, !agc_flag_test(AGC_FLAG_SURFFLAG));
    }

    rz_flag_put(AGC_FLAG_VINTFLAG, csm);
    st = agc_integ_integrv(ip, agc_integ_shared_ctx(), tdec1);
    agc_phase_change(2, 04022); /* GRP2PC   */

    return (st);
}

/**********************************************************************
 * UPPSV: the updated vehicle (with W when valid) to TDEC1, then the other
 * vehicle to the same time; on the surface LMPOS / LMVEL from the LM state.
 * AGC: UPPSV, UPPSV1, UPPSV4 (P20-P25.agc pp.499-501)
 *
 * FUNCTION NAME: nav_uppsv
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   tdec1     agc_dp_t        I    TDEC1, cs 2^28
 *   resume    agc_job_id_t    I    job that continues the caller
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration status
 *          (INTSTALL sleep)
 **********************************************************************/
static agc_status_t nav_uppsv(agc_interp_t *ip, agc_dp_t tdec1,
                              agc_job_id_t resume)
{
    bool         w     = agc_flag_test(AGC_FLAG_RENDWFLG); /* W valid */
    bool         csm   = agc_flag_test(AGC_FLAG_VEHUPFLG); /* CSM updated */
    bool         first = !w || csm;        /* UPPSV1: the CSM first */
    agc_status_t st    = nav_uppsv_one(ip, first, w, tdec1, resume); /* 1st */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t t2 =
        first ? agc_erase_tetcsm() : agc_erase_tetlem(); /* cs 2^28 */

    st = nav_uppsv_one(ip, !first, false, t2, resume); /* UPPSV4  */

    if (st != AGC_STATUS_OK || !agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return (st);
    }

    agc_integ_sv_t sv; /* LM permanent state */

    st          = agc_integ_perm_get(AGC_VEHICLE_LM, &sv);
    lmnav.lmpos = agc_vec_shr(sv.rcv, 2); /* VSR2     */
    lmnav.lmvel = agc_vec_shr(sv.vcv, 2);

    return (st);
}

/* ---- PROG20 / PROG22 (pp.492-495) ------------------------------------- */
/**********************************************************************
 * PROG20: group 4 off, restart at PRIORITY 26; on the surface (P22)
 * choose the CSM orbit option, else the LM state is to be updated.
 * AGC: PROG20, PROG22, LUNSFCHK (P20-P25.agc p.492)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_lm
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
agc_status_t agc_job_rendezvous_navigation_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_phase_job(04U, 05022U, AGC_JOB_RENDEZVOUS_NAVIGATION, 026);

    if (!agc_flag_test(AGC_FLAG_SURFFLAG))
    {                                          /* LUNSFCHK */
        rz_flag_put(AGC_FLAG_VEHUPFLG, false); /* LM state updated */

        return (agc_job_rendezvous_navigation_start(ctx));
    }

    rz_flag_put(AGC_FLAG_VEHUPFLG, true);   /* ORBCHGO: the CSM */
    agc_status_t st = rz_set_option(2, 1U); /* will not change   */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_rendezvous_navigation_orbit_option(ctx));
}

/**********************************************************************
 * GOPERF4 option 12 (assumed CSM orbit option in R2).
 * AGC: ORBCHGO (P20-P25.agc p.493)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_orbit_option
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
agc_status_t agc_job_rendezvous_navigation_orbit_option(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_cont_t cont; /* astronaut returns */

    cont.terminate = AGC_JOB_GOTOPOOH;
    cont.proceed   = AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHOSEN;
    cont.enter     = AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_OPTION;

    return (agc_dsky_display(ctx, AGC_DSKY_GOPERF4, NAV_OPTION_CODE, &cont));
}

/**********************************************************************
 * ORBCHG1: option 1 (orbit unchanged): PROG20A; else V06N33 flashing for
 * the estimated time of launch (TIG).
 * AGC: ORBCHG1 (P20-P25.agc p.493)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_orbit_chosen
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
agc_status_t agc_job_rendezvous_navigation_orbit_chosen(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (rz_option(2) == 1U)
    {
        return (agc_job_rendezvous_navigation_start(ctx));
    }

    return (agc_job_rendezvous_navigation_launch_time(ctx));
}

/**********************************************************************
 * V06N33 flashing (estimated launch time in TIG); PROCEED: the orbit
 * change.
 * AGC: ORBCHG1 V06N33 (P20-P25.agc p.493)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_launch_time
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
agc_status_t agc_job_rendezvous_navigation_launch_time(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_flash(ctx, AGC_DSKY_GOFLASH, 6U, 33U, AGC_JOB_GOTOPOOH,
                     AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHANGE,
                     AGC_JOB_RENDEZVOUS_NAVIGATION_LAUNCH_TIME));
}

/**********************************************************************
 * ORBCHG3 first part: LM (no W) and CSM (W when valid) to the launch
 * time by precision integration; R, V at the moon scale.
 * AGC: ORBCHG3 (P20-P25.agc p.493)
 *
 * FUNCTION NAME: nav_orbit_states
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   rsubl     agc_vec_t *     O    RSUBL: LM position at launch, moon scale
 *   rsubc     agc_vec_t *     O    RSUBC: CSM position at launch, moon
 *                                  scale
 *   vsubc     agc_vec_t *     O    VSUBC: CSM velocity at launch, moon
 *                                  scale
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration status
 *          (INTSTALL sleep)
 **********************************************************************/
static agc_status_t nav_orbit_states(agc_interp_t *ip, agc_vec_t *rsubl,
                                     agc_vec_t *rsubc, agc_vec_t *vsubc)
{
    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* integration erasables */
    agc_status_t     st; /* INTSTALL */

    st = agc_integ_install(0, AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHANGE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    lmnav.lnchtm = agc_erase_tig();
    rz_flag_put(AGC_FLAG_VINTFLAG, false); /* LM       */
    rz_flag_put(AGC_FLAG_INTYPFLG, false); /* Encke    */
    rz_flag_put(AGC_FLAG_DIM0FLAG, false);
    rz_flag_put(AGC_FLAG_D6OR9FLG, false);
    st = agc_integ_integrv(ip, ctx, lmnav.lnchtm); /* INTEGRV  */
    agc_phase_change(2, 04022);                            /* GRP2PC   */

    if (st == AGC_STATUS_OK)
    {
        *rsubl = ctx->out.ratt1; /* RSUBL    */
        st     =
            agc_integ_install(0, AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHANGE);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_dp_t tat = ctx->out.tat; /* TAT, cs 2^28 */
    bool     w   = agc_flag_test(AGC_FLAG_RENDWFLG); /* W valid */

    rz_flag_put(AGC_FLAG_VINTFLAG, true); /* CSM      */
    rz_flag_put(AGC_FLAG_INTYPFLG, false);
    rz_flag_put(AGC_FLAG_DIM0FLAG, w);

    if (w)
    {
        rz_flag_put(AGC_FLAG_D6OR9FLG, true);
    }

    st = agc_integ_integrv(ip, ctx, tat); /* NOWMATX  */
    agc_phase_change(2, 04022);
    *vsubc = ctx->out.vatt1;
    *rsubc = ctx->out.ratt1;

    return (st);
} /* nav_orbit_states */

/**********************************************************************
 * ORBCHG3 (pp.493-495): the CSM orbit plane is rotated to contain the LM
 * at launch. TIMETHET gives the CSM state through the angle between
 * R(CSM) and UCSM = UNIT(R(LM) x N1), N1 = UNIT(R(CSM) x V(CSM)); the
 * velocity is turned into the plane of R(LM) and the new position; the
 * CSM permanent state becomes that state at LNCHTM - TRANSTM.
 * AGC: ORBCHG3 (P20-P25.agc pp.493-495)
 *
 * FUNCTION NAME: nav_orbit_change
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, else the integration, TIMETHET or
 *          INTWAKE0 status
 **********************************************************************/
static agc_status_t nav_orbit_change(agc_interp_t *ip)
{
    agc_vec_t    rsubl; /* RSUBL, m 2^27 */
    agc_vec_t    rsubc; /* RSUBC, m 2^27 */
    agc_vec_t    vsubc; /* VSUBC, m/cs 2^5 */
    agc_status_t st = nav_orbit_states(ip, &rsubl, &rsubc, &vsubc); /* states */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t n1; /* N1, 2^1 */

    n1 = agc_vec_unit(ip, agc_vec_cross(ip, rsubc, vsubc), NULL, NULL);
    agc_vec_t ucsm = agc_vec_unit(ip, agc_vec_cross(ip, rsubl, /* UCSM, 2^1 */
                                                    n1), NULL, NULL);
    agc_timetheta_in_t in; /* TIMETHET inputs */

    in.csth = agc_dp_shl(
        ip, agc_vec_dot(ip, agc_vec_unit(ip, rsubc, NULL, NULL), ucsm), 1);
    in.snth =
        agc_dp_sqrt(ip, agc_dp_sub(ip, NAV_ONEB_2, agc_dp_sq(ip, in.csth)));
    in.rvec = rsubc;
    in.vvec = agc_vec_neg(vsubc);
    in.rvsw = false;
    in.body = AGC_CONIC_MOON;
    rz_flag_put(AGC_FLAG_RVSW, false);
    agc_conic_time_out_t o; /* conic result */

    st = agc_conic_timethet(ip, &in, &o);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t newvel = agc_vec_neg(o.vnew); /* m/cs 2^5 */
    agc_vec_t n2; /* N2, 2^1 */

    n2 = agc_vec_unit(ip, agc_vec_cross(ip, o.rnew, rsubl), NULL, NULL);
    agc_vec_t dir = agc_vec_unit(ip, agc_vec_cross(ip, n2, /* 2^1 */
                                                   o.rnew), NULL, NULL);
    agc_vec_t ncsmvel; /* m/cs 2^5 */

    ncsmvel = agc_vec_shl(ip, agc_vec_scale(ip, dir, agc_vec_abval(ip,
                                                                   newvel)), 1);

    agc_phase_change(2, 04022); /* GRP2PC   */
    st = agc_integ_install(0, AGC_JOB_RENDEZVOUS_NAVIGATION_ORBIT_CHANGE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* integration erasables */

    ctx->sv.tet   = agc_dp_sub(ip, lmnav.lnchtm, o.t);        /* TET      */
    ctx->sv.rcv   = o.rnew;
    ctx->sv.rrect = o.rnew;
    agc_integ_minirect(ctx, ncsmvel);    /* VRECT    */
    st = agc_integ_atopcsm(ip, ctx, 2);  /* X2 = 2   */
    agc_status_t sw = agc_integ_intwake0(); /* INTWAKE0 */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* the first failure */
    }

    return (sw);
} /* nav_orbit_change */

/**********************************************************************
 * ORBCHG2: the orbit change, then PROG20A. An INTSTALL sleep resumes this
 * job from its start.
 * AGC: ORBCHG2 (P20-P25.agc p.493)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_orbit_change
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
agc_status_t agc_job_rendezvous_navigation_orbit_change(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = nav_orbit_change(&ip); /* ORBCHG3 */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    return (agc_job_rendezvous_navigation_start(ctx));
}

/**********************************************************************
 * PROG20A (pp.495-496): R02BOTH; UPDATFLG, TRACKFLG, RNDVZFLG on; search,
 * manual acquisition, R04FLAG (alarm 521 wanted), NORRMON, LOSCMFLG off;
 * CLRADMOD; P20LEM1.
 * AGC: PROG20A (P20-P25.agc pp.495-496)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_start
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
agc_status_t agc_job_rendezvous_navigation_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_imu_status_check(); /* R02BOTH  */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* alarm and P00 (POODOO)     */
    }

    rz_flag_put(AGC_FLAG_UPDATFLG, true);
    rz_flag_put(AGC_FLAG_TRACKFLG, true);
    rz_flag_put(AGC_FLAG_RNDVZFLG, true);
    rz_flag_put(AGC_FLAG_SRCHOPTN, false);
    rz_flag_put(AGC_FLAG_ACMODFLG, false);
    rz_flag_put(AGC_FLAG_ALARM_521_SUPPRESSED, false);
    rz_flag_put(AGC_FLAG_NORRMON, false);
    rz_flag_put(AGC_FLAG_LOSCMFLG, false);
    agc_rendz_clradmod();

    return (agc_job_rendezvous_navigation_range_check(ctx));
}

/**********************************************************************
 * P20LEM1 (p.496): mark counter zero; LOS now (LPS20.1); more than 400 NM
 * (LPS20.2, or the LPS20.1 overflow): priority alarm 526.
 * AGC: P20LEM1 (P20-P25.agc p.496)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_range_check
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
agc_status_t agc_job_rendezvous_navigation_range_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(2, 04022);
    agc_erase_set_trkmkcnt(0); /* MARKCTR  */
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    bool         too_far = false; /* range > 400 NM */
    agc_status_t st      =
        lmnav_los(&ip, agc_rtb_loadtime(),
                  AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_CHECK,
                  &too_far); /* LPS20.1 */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    if (too_far || !lmnav_range_ok())
    {
        return (agc_job_rendezvous_navigation_range_alarm(ctx));
    }

    return (agc_job_rendezvous_navigation_attitude(ctx));
}

/**********************************************************************
 * 526ALARM: TERMINATE exits P20 through V56, PROCEED is illegal (the
 * alarm again), ENTER recycles to P20LEM1.
 * AGC: 526ALARM (P20-P25.agc p.496)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_range_alarm
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
agc_status_t agc_job_rendezvous_navigation_range_alarm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rz_prio_alarm(ctx, NAV_ALARM_RANGE,
                          AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                          AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_ALARM,
                          AGC_JOB_RENDEZVOUS_NAVIGATION_RANGE_CHECK));
}

/**********************************************************************
 * GOTOV56 as a job (the TERMINATE return of the P20 displays).
 * AGC: GOTOV56 (P20-P25.agc p.593)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_terminate
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
agc_status_t agc_job_rendezvous_navigation_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_rendz_goto_terminate_tracking());
}

/**********************************************************************
 * P20LEMA (p.496): preferred tracking attitude (R61LEM) unless on the
 * surface, then P20LEMB.
 * AGC: P20LEMA (P20-P25.agc p.496)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_attitude
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
agc_status_t agc_job_rendezvous_navigation_attitude(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(2, 04022);

    if (agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return (agc_job_rendezvous_navigation_acquire(ctx));
    }

    return (lmnav_tracking_attitude(ctx, true,
                                    AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE));
}

/* ---- P20LEMB .. P20LEMD2 (pp.496-499) ---------------------------------- */
/**********************************************************************
 * P20LEMB: restart at priority 10 (a V37-requested program runs first),
 * priority back to 26; wait for the track flag; with the RR in auto mode
 * go to acquisition; otherwise ask for auto mode in P20/P22 (GOPERF1
 * 201: ENTER = manual acquisition R23), elsewhere priority alarm 514.
 * AGC: P20LEMB, P20LEMB5, P20LEMB6 (P20-P25.agc pp.496-497)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_acquire
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
agc_status_t agc_job_rendezvous_navigation_acquire(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_phase_job(0U, 05022U, AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE, 010);
    agc_exec_prio_change(NAV_PRIO26); /* PRIOCHNG */

    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return (lmnav_wait(NAV_250DEC)); /* P20LEMWT */
    }

    if ((agc_chan_read(AGC_CHAN_CHAN33) & NAV_CH33_RR_AUTO) == 0U)
    {
        return (agc_job_rendezvous_navigation_cdu_zero_wait(ctx)); /* B3   */
    }

    agc_sp_t mm = agc_erase_modreg(); /* P20LEMB5: MODREG, unscaled */

    if (mm == NAV_MODE_RENDEZVOUS || mm == NAV_MODE_SURFACE)
    { /* P20LEMB6 */
        agc_dsky_cont_t cont; /* astronaut returns */

        cont.terminate = AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE;
        cont.proceed   = AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE;
        cont.enter     = AGC_JOB_RENDEZVOUS_NAVIGATION_MANUAL_REQUEST;

        return (agc_dsky_display(ctx, AGC_DSKY_GOPERF1, NAV_CHECKLIST_AUTO,
                                 &cont));
    }

    return (rz_prio_alarm(ctx, NAV_ALARM_AUTO,
                          AGC_JOB_RENDEZVOUS_NAVIGATION_TERMINATE,
                          AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE,
                          AGC_JOB_RENDEZVOUS_NAVIGATION_ACQUIRE));
}

/**********************************************************************
 * ENTER on the auto-mode request: manual acquisition (R23LEM), not
 * allowed on the surface (P20LEMB2: operator error light).
 * AGC: P20LEMB2 (P20-P25.agc p.497)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_manual_request
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
agc_status_t agc_job_rendezvous_navigation_manual_request(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!agc_flag_test(AGC_FLAG_SURFFLAG))
    {
        return (agc_job_manual_acquisition(ctx));
    }

    agc_dsky_falton(); /* FALTON   */

    return (agc_job_rendezvous_navigation_acquire(ctx));
}

/**********************************************************************
 * P20LEMB3: wait while the RR CDUs are zeroed (2.5 s DELAYJOB); after a
 * search or manual acquisition go back to auto and to the data read;
 * else update the state vectors and designate.
 * AGC: P20LEMB3, P20LEMB4 (P20-P25.agc p.497)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_cdu_zero_wait
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
agc_status_t agc_job_rendezvous_navigation_cdu_zero_wait(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (rr_radmodes_test(RR_RCDU0BIT))
    { /* P20LEMB4 */
        return (agc_svc_delayjob(NAV_250DEC,
                                 AGC_JOB_RENDEZVOUS_NAVIGATION_CDU_ZERO_WAIT));
    }

    if (agc_flag_test(AGC_FLAG_SRCHOPTN) || agc_flag_test(AGC_FLAG_ACMODFLG))
    {
        rz_flag_put(AGC_FLAG_SRCHOPTN, false);
        rz_flag_put(AGC_FLAG_ACMODFLG, false);

        return (lmnav_wait(NAV_250DEC));
    }

    return (agc_job_rendezvous_navigation_update_states(ctx));
}

/**********************************************************************
 * P20LEMC3: permanent state vectors to now (UPPSV), then P20LEMC.
 * AGC: P20LEMC3 (P20-P25.agc p.498)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_update_states
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
agc_status_t agc_job_rendezvous_navigation_update_states(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = nav_uppsv(&ip, agc_rtb_loadtime(), /* UPPSV */
                                AGC_JOB_RENDEZVOUS_NAVIGATION_UPDATE_STATES);

    if (st != AGC_STATUS_OK)
    {
        return (st); /* BUSY: asleep in INTSTALL  */
    }

    return (agc_job_rendezvous_navigation_designate(ctx));
}

/**********************************************************************
 * P20LEMC: P20 over (rendezvous flag off): end; no track flag: wait 15 s
 * (P20LEMD); else designate (R21LEM).
 * AGC: P20LEMC, P20LEMD, P20LEMF (P20-P25.agc p.498)
 *
 * FUNCTION NAME: agc_job_rendezvous_navigation_designate
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
agc_status_t agc_job_rendezvous_navigation_designate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(2, 04022);

    if (!agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        return (AGC_STATUS_OK); /* ENDOFJOB */
    }

    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    { /* P20LEMD  */
        return (agc_waitlist_call(NAV_1500DEC,
                                  AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_WAIT));
    }

    return (agc_job_rendezvous_sighting_mark(ctx)); /* P20LEMF  */
}

/**********************************************************************
 * P20LEMWT: in `delay_cs` look again (P20LEMC1); the job ends, with the
 * phase 2.7 restart point only while the track flag is on.
 * AGC: P20LEMWT, P20LMWT1 (P20-P25.agc p.498)
 *
 * FUNCTION NAME: lmnav_wait
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   delay_cs  uint16_t  I    the wait, cs
 *
 * RETURNS: agc_status_t -- the waitlist status
 **********************************************************************/
agc_status_t lmnav_wait(uint16_t delay_cs)
{
    if (!AGC_ASSERT(delay_cs > 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st; /* P20LEMC1 */

    st =
        agc_waitlist_call(delay_cs, AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_CHECK);

    if (st == AGC_STATUS_OK && agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        agc_phase_change(2, 040072); /* P20LMWT1 */
    }

    return (st); /* ENDOFJOB */
}

/**********************************************************************
 * P20LEMC1 (task): P20 over: stop; track flag on: the R22 data read job;
 * else look again in 15 s.
 * AGC: P20LEMC1 (P20-P25.agc p.498)
 *
 * FUNCTION NAME: agc_task_rendezvous_navigation_track_check
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_rendezvous_navigation_track_check(void)
{
    if (!agc_flag_test(AGC_FLAG_RNDVZFLG))
    {
        return (AGC_STATUS_OK); /* TASKOVER */
    }

    if (agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        return (agc_exec_findvac(NAV_PRIO26,
                                 AGC_JOB_RENDEZVOUS_TRACKING_DATA_NEXT));
    }

    return (agc_waitlist_fixdelay(NAV_1500DEC,
                                  AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_CHECK));
}

/**********************************************************************
 * P20LEMD1 (task): track flag on: P20LEMC3 (designate); else 15 s more.
 * AGC: P20LEMD1, P20LEMD2 (P20-P25.agc p.499)
 *
 * FUNCTION NAME: agc_task_rendezvous_navigation_track_wait
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_rendezvous_navigation_track_wait(void)
{
    if (agc_flag_test(AGC_FLAG_TRACKFLG))
    { /* P20LEMD2 */
        return (agc_exec_findvac(NAV_PRIO26,
                                 AGC_JOB_RENDEZVOUS_NAVIGATION_UPDATE_STATES));
    }

    return (agc_waitlist_fixdelay(NAV_1500DEC,
                                  AGC_TASK_RENDEZVOUS_NAVIGATION_TRACK_WAIT));
}

/**********************************************************************
 * P25 PREFERRED TRACKING ATTITUDE (pp.502-503)
 * "To compute the preferred tracking attitude of the LM to continuously
 * point the LM tracking beacon at the CSM and to perform the maneuver to
 * the preferred tracking attitude and continuously maintain this
 * attitude within prescribed limits."
 * AGC: PROG25 (P20-P25.agc p.502)
 *
 * FUNCTION NAME: agc_job_preferred_tracking_attitude
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
agc_status_t agc_job_preferred_tracking_attitude(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rz_phase_job(04U, 05022U, AGC_JOB_PREFERRED_TRACKING_ATTITUDE, 026);
    agc_status_t st = agc_imu_status_check(); /* R02BOTH  */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    rz_flag_put(AGC_FLAG_TRACKFLG, true);
    rz_flag_put(AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE,
                true); /* P25FLAG */

    return (agc_job_preferred_tracking_attitude_check(ctx));
}

/**********************************************************************
 * P25LEM1: P25 over: end; no track flag: look again in a minute; else
 * R65 (fine preferred tracking attitude, 7 passes), then here again.
 * AGC: P25LEM1, P25LMWT1 (P20-P25.agc p.502)
 *
 * FUNCTION NAME: agc_job_preferred_tracking_attitude_check
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
agc_status_t agc_job_preferred_tracking_attitude_check(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_phase_change(2, 04022);

    if (!agc_flag_test(AGC_FLAG_PREFERRED_TRACKING_ATTITUDE_ACTIVE))
    {
        return (AGC_STATUS_OK); /* ENDOFJOB */
    }

    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    { /* P25LMWT1 */
        return (agc_waitlist_call(NAV_60SCNDS,
                                  AGC_TASK_PREFERRED_TRACKING_ATTITUDE_WAIT));
    }

    lmnav.r65cntr = 7;

    return (lmnav_tracking_attitude(ctx, false,
                                    AGC_JOB_PREFERRED_TRACKING_ATTITUDE_CHECK));
}

/**********************************************************************
 * P25LEM2 (task): P25LEM1 as a PRIO14 job.
 * AGC: P25LEM2 (P20-P25.agc p.503)
 *
 * FUNCTION NAME: agc_task_preferred_tracking_attitude_wait
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_preferred_tracking_attitude_wait(void)
{
    if (!AGC_ASSERT(lmnav.r65cntr >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_findvac(NAV_PRIO14,
                             AGC_JOB_PREFERRED_TRACKING_ATTITUDE_CHECK));
}

/**********************************************************************
 * Noun data of the LM rendezvous navigation (R22 / P20 LM): DATAGOOD,
 * OMEGAD, the N49 radar residuals, WHCHREAD. NOT_FOUND: not one of them.
 * AGC: none (noun table access; nouns 49, 80)
 *
 * FUNCTION NAME: agc_lm_rendz_noun_read
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  ------------------------------------
 *   data_id   uint16_t    I    noun data id
 *   v         agc_dp_t *  O    the value at its noun scale
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_NOT_FOUND when not an
 *          LM navigation item, AGC_STATUS_BAD_ARG on a bad argument
 **********************************************************************/
agc_status_t agc_lm_rendz_noun_read(uint16_t data_id, agc_dp_t *v)
{
    if (!AGC_ASSERT(v != NULL) || !AGC_ASSERT(data_id < AGC_DATA_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const lmnav_state_t *n = lmnav_state(); /* LM navigation erasables */

    switch (data_id)
    {
        case AGC_DATA_DATAGOOD:
            *v = agc_dp_from_sp(n->datagood);

            return (AGC_STATUS_OK);

        case AGC_DATA_OMEGAD:
            *v = n->omegad;

            return (AGC_STATUS_OK);

        case AGC_DATA_RADAR_DELTA_R:
            *v = n->r22disp[0];

            return (AGC_STATUS_OK);

        case AGC_DATA_RADAR_DELTA_V:
            *v = n->r22disp[1];

            return (AGC_STATUS_OK);

        case AGC_DATA_WHCHREAD:
            *v = agc_dp_from_sp(n->whchread);

            return (AGC_STATUS_OK);

        default:
            break;
    }

    return (AGC_STATUS_NOT_FOUND);
}
