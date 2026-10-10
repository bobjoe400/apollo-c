/*
 * attitude_maneuver.c -- R60 attitude maneuver routine, R62 crew-defined maneuver,
 * VECPOINT, and (LM) BALLANGS, G+N,AUTO and ISITAUTO.
 *
 * Source: R60_62.agc, Comanche055 pp.390-398 and Luminary099 pp.342-363
 *         (diffed).  Differences kept:
 *   * LM converts the final gimbal angles to FDAI ball angles (BALLANGS)
 *     before each V06N18 display and has VECPNT1 (from given CDUs).
 *   * the auto-mode test: CM reads channel 31 bits 13-15 ("mode auto and
 *     control GNC"); LM G+N,AUTO reads channel 30 bit 10 then ISITAUTO
 *     channel 31 bit 14.
 *   * RELINUS (restart of a priority display): CM continues only with
 *     TRACKFLG, sets PDSPFLAG and TARG1FLG, OPTIND = 0, priority 14, else
 *     restart 1.11 (PIKUP20); LM priority 26, with TRACKFLG sets PDSPFLAG,
 *     else restart 2.7 (P20) or 2.11 (P25).  CHKLINUS restart point is
 *     CM 1.7, LM 2.13.
 *   * ROT180 / PICKX: CM HIUNITX, LM HIDPHALF (the same vector .5, 0, 0).
 *
 * The AGC's display calls return to the job at different points for the
 * crew's responses; here each response is its own job (agc_dsky_cont_t).
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/nav/auxiliary.h"
#include "internal.h"
#include "pending.h"

#define SINGIMLC ((agc_dp_t) 115047034)  /* 2DEC .4285836003 = sin(59)/2  */
#define SINVEC1  ((agc_dp_t) 101907670)  /* 2DEC .3796356537 = sin(49.4)/2 */
#define SINVEC2  ((agc_dp_t) 66091971)   /* 2DEC .2462117800 = sin(29.5)/2 */
#define VECANG1  ((agc_dp_t) 37282702)   /* 2DEC .1388888889 = 50 deg      */
#define VECANG2  ((agc_dp_t) 26097892)   /* 2DEC .09722222222 = 35 deg     */
#define DPB14    ((agc_dp_t) (1L << 14)) /* DPB-14: OCT 00001, OCT 00000   */
#define CDU_MASK 077777u
#define PRIO14   ((agc_prio_t) 014)
#define PRIO26   ((agc_prio_t) 026)
#define BIT7     0100u /* LINUS (BLANKET) code          */

static agc_vec_t scaxis;   /* SCAXIS: S/C axis to point, half-unit     */
static agc_vec_t pointvsm; /* POINTVSM: direction in SM axes, half-unit */
static uint16_t  fdai[3];  /* FDAIX, FDAIY, FDAIZ (LM), CDU patterns   */
static agc_job_id_t
    attitude_maneuver_return; /* TEMPR60: where R60 returns               */
static bool relinus_static;   /* last R60 display was the static V06N18   */

static bool vec_ok(agc_vec_t v)
{
    bool ok = true;
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        ok = ok && v.c[i] >= AGC_DP_MIN && v.c[i] <= AGC_DP_MAX;
    }
    return ok;
}

agc_status_t agc_imu_set_scaxis(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(v)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    scaxis = v;
    if (!AGC_ASSERT(vec_ok(scaxis)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

void agc_imu_set_pointvsm(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(v)))
    {
        return;
    }
    pointvsm = v;
    if (!AGC_ASSERT(vec_ok(pointvsm)))
    {
        return;
    }
}

uint16_t agc_erase_fdai(int axis)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return 0;
    }
    if (!AGC_ASSERT(fdai[axis] <= CDU_MASK))
    {
        return 0;
    }
    return fdai[axis];
}

/* ======================================================================= */
/* VECPOINT (p.393-397 CM / LM)                                            */
/* ======================================================================= */

/* FINDGIMB: commanded CDU angles from the S/C-to-SM matrix m.            */
static agc_status_t findgimb(agc_interp_t *ip, const agc_mat_t *m,
                             uint16_t gimbal[3])
{
    agc_vec_t    ang;
    agc_dp_t     cmga;
    agc_status_t st = agc_kalc_dcmtocdu(ip, m, &ang, &cmga);
    if (st == AGC_STATUS_OK)
    {
        st = agc_rtb_v1sto2s(ang, gimbal); /* 2:S COMPLEMENT     */
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* COMPMATX .. FINDGIMB: MFS = MIS DEL(COF, AM); if that is within 59 deg
 * of gimbal lock and the axis is not the thrust axis, rotate a further
 * 50 deg (AOT) or 35 deg about +-SCAXIS to get out of it.                */
static agc_status_t compmatx(agc_interp_t *ip, const agc_mat_t *mis,
                             agc_vec_t cofv, agc_dp_t am, uint16_t gimbal[3])
{
    agc_mat_t    del;
    agc_status_t st = agc_kalc_delcomp(ip, cofv, am, &del);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }
    agc_mat_t mfs = agc_mat_mul(ip, mis, &del); /* MXM3     */
    if (agc_dp_sub(ip, agc_dp_abs(mfs.row[1].c[0]), SINGIMLC) < 0 ||
        agc_dp_sub(ip, agc_dp_abs(scaxis.c[0]), SINVEC1) >= 0)
    {
        return findgimb(ip, &mfs, gimbal); /* no lock, or thrust axis */
    }
    agc_vec_t ig = mfs.row[1]; /* IG axis in final S/C    */
    if (ig.c[0] < 0)
    {
        ig = agc_vec_neg(ig);
    }
    agc_vec_t u   = (agc_vec_cross(ip, ig, scaxis).c[0] < 0) /* IGSAMEX  */
                        ?
                        scaxis :
                        agc_vec_neg(scaxis); /* U=SCAXIS */
    agc_dp_t  ang = (agc_dp_sub(ip, agc_dp_abs(scaxis.c[0]), SINVEC2) >= 0) ?
                        VECANG1 :
                        VECANG2;                        /* CHEKAXIS */
    st            = agc_kalc_delcomp(ip, u, ang, &del); /* COMPMFSN */
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }
    agc_mat_t m2 = agc_mat_mul(ip, &mfs, &del);
    return findgimb(ip, &m2, gimbal);
}

/* PICKAXIS / ROT180 / PICKX: VF x VI is (nearly) zero.  Parallel: keep
 * the present CDUs.  Antiparallel: rotate 180 deg about an axis in the
 * plane of X(I) and Y(SM) perpendicular to VI, or about X.               */
static agc_status_t pickaxis(agc_interp_t *ip, const agc_mat_t *mis,
                             agc_vec_t vf, const uint16_t cdu0[3],
                             uint16_t gimbal[3])
{
    if (agc_vec_dot(ip, vf, scaxis) >= 0)
    {
        for (int i = 0; i < 3; i++)
        {
            gimbal[i] = cdu0[i]; /* CDU desired = present   */
        }
        return AGC_STATUS_OK;
    }
    const agc_vec_t hiunitx = agc_vec_make(AGC_DP_HALF, 0, 0);
    agc_dp_t        mag     = 0;
    agc_vec_t       c =
        agc_vec_unit(ip, agc_vec_cross(ip, mis->row[1], hiunitx), NULL, NULL);
    c = agc_vec_unit(ip, agc_vec_cross(ip, c, scaxis), &mag, NULL);
    if (agc_interp_test_ovf(ip) || agc_dp_sub(ip, mag, DPB14) < 0)
    {
        c = hiunitx; /* PICKX    */
    }
    return compmatx(ip, mis, c, AGC_DP_HALF, gimbal); /* XROT     */
}

/* VECPOINT body from initial CDU angles cdu0 (VECLEAR / STORANG).        */
static agc_status_t vecpoint_from(agc_interp_t *ip, const uint16_t cdu0[3],
                                  uint16_t gimbal[3])
{
    agc_interp_clear_ovf(ip); /* BOV VECLEAR: clear OVFIND */
    agc_mat_t    mis;
    agc_status_t st = agc_kalc_cdutodcm(ip, cdu0, &mis);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }
    agc_vec_t vf =
        agc_vec_unit(ip, agc_vec_mat(ip, pointvsm, &mis), NULL, NULL);
    agc_dp_t  mag = 0;
    agc_vec_t c   = agc_vec_unit(ip, agc_vec_cross(ip, vf, scaxis), &mag, NULL);
    if (agc_interp_test_ovf(ip) || agc_dp_sub(ip, mag, DPB14) < 0)
    {
        return pickaxis(ip, &mis, vf, cdu0, gimbal);
    }
    agc_vec_t cofv = agc_vec_neg(c); /* VCOMP    */
    agc_dp_t  am =
        agc_dp_acos(ip, agc_dp_shl(ip, agc_vec_dot(ip, scaxis, vf), 1));
    return compmatx(ip, &mis, cofv, am, gimbal);
}

/* VECPOINT: point S/C axis scaxis_in along pointvsm_in from the present
 * CDU angles (READCDUK); gimbal[] = CPHI, CTHETA, CPSI patterns.          */
agc_status_t agc_imu_vecpoint(agc_interp_t *ip, agc_vec_t pointvsm_in,
                              agc_vec_t scaxis_in, uint16_t gimbal[3])
{
    if (!AGC_ASSERT(ip != NULL && gimbal != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_imu_set_pointvsm(pointvsm_in);
    if (agc_imu_set_scaxis(scaxis_in) != AGC_STATUS_OK)
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    const uint16_t cdu0[3] = {agc_erase_cdux(), agc_erase_cduy(),
                              agc_erase_cduz()};
    agc_relint();
    return vecpoint_from(ip, cdu0, gimbal);
}

/* VECPNT1 (LM): as VECPOINT but from the desired CDUs given in cdu0.     */
agc_status_t agc_imu_vecpnt1(agc_interp_t *ip, const uint16_t cdu0[3],
                             agc_vec_t pointvsm_in, agc_vec_t scaxis_in,
                             uint16_t gimbal[3])
{
    if (!AGC_ASSERT(ip != NULL && cdu0 != NULL && gimbal != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_imu_set_pointvsm(pointvsm_in);
    if (agc_imu_set_scaxis(scaxis_in) != AGC_STATUS_OK)
    {
        return AGC_STATUS_BAD_ARG;
    }
    return vecpoint_from(ip, cdu0, gimbal);
}

/* ======================================================================= */
/* LM: BALLANGS, G+N,AUTO, ISITAUTO (pp.346-348)                           */
/* ======================================================================= */

/* BALLANGS: FDAI ball angles from gimbal angles X, Y, Z:
 *   YAW = ARCSIN(-SX CZ),  ROLL = ARCTAN(SZ / CZ CX),
 *   PITCH = ARCTAN((CX SY + SX SZ CY) / (CX CY - SX SZ SY)).             */
agc_status_t agc_imu_ballangs(agc_interp_t *ip, const uint16_t gimbal[3],
                              uint16_t out[3])
{
    if (!AGC_ASSERT(ip != NULL && gimbal != NULL && out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const uint16_t yzx[3] = {gimbal[1], gimbal[2], gimbal[0]}; /* CDUSPOT */
    agc_cdutrig_t  t;
    agc_guid_cdtrigs(&t, yzx); /* CD*TR*G */
    agc_status_t st = AGC_STATUS_OK;
    const agc_dp_t sx = t.sincdu[AGC_TRIG_X], cx = t.coscdu[AGC_TRIG_X];
    const agc_dp_t sy = t.sincdu[AGC_TRIG_Y], cy = t.coscdu[AGC_TRIG_Y];
    const agc_dp_t sz = t.sincdu[AGC_TRIG_Z], cz = t.coscdu[AGC_TRIG_Z];
    agc_dp_t       yaw =
        agc_dp_asin(ip, agc_dp_neg(agc_dp_shl(ip, agc_dp_mul(ip, sx, cz), 1)));
    agc_dp_t roll =
        agc_latlong_arctan(ip, sz, agc_dp_shl(ip, agc_dp_mul(ip, cz, cx), 1));
    agc_dp_t sxsz = agc_dp_shl(ip, agc_dp_mul(ip, sz, sx), 2);
    agc_dp_t pd4  = agc_dp_mul(ip, sxsz, cy); /* SXSZCY   */
    agc_dp_t pd6  = agc_dp_mul(ip, sxsz, sy); /* SXSZSY   */
    agc_dp_t costh =
        agc_dp_sub(ip, agc_dp_shl(ip, agc_dp_mul(ip, cx, cy), 1), pd6);
    agc_dp_t sinth =
        agc_dp_add(ip, agc_dp_shl(ip, agc_dp_mul(ip, sy, cx), 1), pd4);
    agc_dp_t pitch = agc_latlong_arctan(ip, sinth, costh);
    st = agc_rtb_v1sto2s(agc_vec_make(roll, pitch, yaw), out); /* FDAIX..  */
    for (int i = 0; i < 3 && st == AGC_STATUS_OK; i++)
    {
        fdai[i] = out[i];
    }
    return st;
}

/* ISITAUTO: channel 31 bit 14; positive = not in auto, +0 = auto.       */
agc_sp_t agc_imu_isitauto(void)
{
    uint16_t v = agc_chan_read(AGC_CHAN_CHAN31) & 020000u;
    if (!AGC_ASSERT(v == 0u || v == 020000u))
    {
        return 0; /* no valid result */
    }
    return (agc_sp_t) v;
}

/* G+N,AUTO: channel 30 bit 10 (not in G+N: positive), then ISITAUTO.    */
agc_sp_t agc_imu_gn_auto(void)
{
    uint16_t v = agc_chan_read(AGC_CHAN_CHAN30) & 01000u;
    if (!AGC_ASSERT(v == 0u || v == 01000u))
    {
        return 0; /* no valid result */
    }
    return (v != 0u) ? (agc_sp_t) (v - 1u) : agc_imu_isitauto(); /* CCS A */
}

/* ======================================================================= */
/* R60 (pp.390-392 CM, pp.342-346 LM)                                      */
/* ======================================================================= */

/* VECPOINT from SCAXIS / POINTVSM into CPHI, CTHETA, CPSI, unless the
 * 3-axis flag says CPHI.. were loaded directly (REDOMANN / REDOMANC).     */
static agc_status_t final_angles(void)
{
    if (agc_flag_test(AGC_FLAG_3AXISFLG))
    {
        return AGC_STATUS_OK;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    uint16_t     g[3];
    agc_status_t st = agc_imu_vecpoint(&ip, pointvsm, scaxis, g);
    for (int i = 0; i < 3 && st == AGC_STATUS_OK; i++)
    {
        agc_erase_set_thetad(i, g[i]); /* STORE CPHI     */
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* LM: BALLANGS of CPHI, CTHETA, CPSI for noun 18.                        */
static void lm_ballangs(void)
{
    if (agc_state_vehicle() != AGC_VEHICLE_LM)
    {
        return;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    const uint16_t g[3] = {agc_erase_thetad(0), agc_erase_thetad(1),
                           agc_erase_thetad(2)};
    uint16_t       out[3];
    agc_status_t   st = agc_imu_ballangs(&ip, g, out);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }
}

/* CHKLINUS: if the priority display flag is set, make the display a
 * priority one (LINUS) and set the restart to RELINUS (CM 1.7, LM 2.13). */
static void chklinus(bool is_static)
{
    relinus_static = is_static;
    if (!agc_flag_test(AGC_FLAG_PDSPFLAG))
    {
        return; /* TC Q          */
    }
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_phase_change(2, 00132);
    }
    else
    {
        agc_phase_change(1, 00071);
    }
    agc_status_t st = agc_dsky_blanket(BIT7); /* LINUS         */
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }
}

/* TOBALL (CM) / TOBALLA (LM): flash V06N18 "please perform auto
 * maneuver"; V34 -> R61TEST, PROCEED -> REDOMANC, ENTER -> ENDMANU1.     */
static agc_status_t toballa(const agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t cont = {AGC_JOB_ATTITUDE_MANEUVER_TERMINATE,
                                  AGC_JOB_REDOMANC,
                                  AGC_JOB_ATTITUDE_MANEUVER_ENTER};
    agc_status_t          st =
        agc_dsky_display(ctx, AGC_DSKY_GOPERF2R, agc_dsky_vn(6, 18), &cont);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    chklinus(false);
    return AGC_STATUS_OK; /* ENDOFJOB      */
}

/* AUTOMANV / STARTMNV: static V06N18 during the maneuver, then GOMANUR;
 * ENDMANUV redisplays the request when the maneuver is done.             */
static agc_status_t automanv(const agc_job_ctx_t *ctx)
{
    agc_status_t st =
        agc_dsky_display(ctx, AGC_DSKY_GODSPR, agc_dsky_vn(6, 18), NULL);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    chklinus(true);
    return agc_imu_gomanur(ctx, AGC_JOB_ENDMANUV);
}

/* Auto-mode test before the maneuver: CM "mode auto and control GNC"
 * (channel 31 bits 13-15 = 1, 1, 0 as read); LM G+N,AUTO.               */
static bool in_auto(void)
{
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return agc_imu_gn_auto() == 0;
    }
    uint16_t v =
        (uint16_t) ((agc_chan_read(AGC_CHAN_CHAN31) ^ 030000u) & 070000u);
    if (!AGC_ASSERT(v <= 070000u))
    {
        return false;
    }
    return v == 0u; /* BZF +2        */
}

/* R60CSM / R60LEM: attitude maneuver to the VECPOINT (or loaded CPHI..)
 * attitude.  ret_job continues the caller when R60 ends (TEMPR60).       */
agc_status_t agc_imu_attitude_maneuver(const agc_job_ctx_t *ctx,
                                       agc_job_id_t         ret_job)
{
    if (!AGC_ASSERT(ctx != NULL && ret_job != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    attitude_maneuver_return = ret_job; /* MAKECADR      */
    agc_status_t st = final_angles(); /* REDOMANN      */
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    lm_ballangs();                      /* TOBALL (LM)   */
    return toballa(ctx);
}

/* REDOMANC (PROCEED): recompute, then maneuver if in auto, else ask again. */
agc_status_t agc_job_redomanc(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = final_angles();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    lm_ballangs(); /* TOBALLC (LM)  */
    if (!in_auto())
    {
        return toballa(ctx); /* not auto      */
    }
    return automanv(ctx);
}

/* ENDMANUV: maneuver finished; ask for the next one (TOBALL / TOBALLA).  */
agc_status_t agc_job_endmanuv(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(!agc_kalc_active()))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return toballa(ctx);
}

/* ENDMANU1: reset the 3-axis flag and return to the user (BANKJUMP).    */
static agc_status_t endmanu1(const agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_3AXISFLG);
    if (attitude_maneuver_return == AGC_JOB_NONE)
    {
        return AGC_STATUS_OK;
    }
    agc_job_ctx_t next = *ctx; /* same job      */
    next.id            = attitude_maneuver_return;
    return agc_dispatch_job(&next);
}

agc_status_t agc_job_attitude_maneuver_enter(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return endmanu1(ctx); /* ENTER         */
}

/* R61TEST (V34): in P00 (V49/V89) return to the user, who does ENDEXT;
 * in R61 (priority display flag) terminate P20 via V56; else GOTOPOOH.   */
agc_status_t agc_job_attitude_maneuver_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_erase_modreg() == 0)
    {
        return endmanu1(ctx);
    }
    if (!agc_flag_test(AGC_FLAG_PDSPFLAG))
    {
        return agc_exec_gotopooh();
    }
    return agc_rendz_goto_terminate_tracking();
}

/* RELINUS (restart 1.7 CM / 2.13 LM): resume a priority R60 display.    */
agc_status_t agc_job_relinus(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM);
    if (lm)
    {
        agc_exec_prio_change(PRIO26);
    }
    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        if (!lm)
        {
            agc_phase_change(1, 00111); /* PIKUP20 */
        }
        else if (agc_flag_test(AGC_FLAG_RNDVZFLG))
        {
            agc_phase_change(2, 040072);
        }
        else
        {
            agc_phase_change(2, 040112);
        }
        return AGC_STATUS_OK; /* ENDOFJOB      */
    }
    agc_flag_set(AGC_FLAG_PDSPFLAG);
    if (!lm)
    {
        imu_vflag_set(IMU_VF_TARG1FLG); /* for R52       */
        agc_erase_set_optind(0); /* OPTIND was set to -1 by V37    */
        agc_exec_prio_change(PRIO14);
    }
    return relinus_static ? automanv(ctx) : toballa(ctx); /* TC TBASE    */
}

/* ======================================================================= */
/* R62 (CM p.398, LM p.363)                                                */
/* ======================================================================= */

/* R62DISP (V49): flash V06N22 with CPHI, CTHETA, CPSI; the crew may load
 * new angles (ENTER redisplays); PROCEED -> GOMOVE; V34 -> ENDEXT.       */
agc_status_t agc_job_crew_defined_maneuver_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_dsky_cont_t cont = {AGC_JOB_CREW_DEFINED_MANEUVER_TERMINATE,
                                  AGC_JOB_CREW_DEFINED_MANEUVER_GOMOVE,
                                  AGC_JOB_CREW_DEFINED_MANEUVER_DISPLAY};
    agc_status_t          st =
        agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, 22), &cont);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* GOMOVE: set the 3-axis flag and do R60; ENDEXT afterwards.             */
agc_status_t agc_job_crew_defined_maneuver_gomove(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_flag_set(AGC_FLAG_3AXISFLG);
    return agc_imu_attitude_maneuver(ctx,
                                     AGC_JOB_CREW_DEFINED_MANEUVER_TERMINATE);
}

agc_status_t agc_job_crew_defined_maneuver_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_dsky_endmark(); /* ENDEXT        */
}
