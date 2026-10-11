/*
 * alignment.c -- inflight alignment routines.
 *
 * Source: INFLIGHT_ALIGNMENT_ROUTINES.agc, Comanche055 pp.1355-1363 and
 *         Luminary099 pp.1250-1258 (same text; the only difference is the
 *         CM CPHIFLAG test at the end of CALCGA, see agc_imu_calcga), and
 *         CALCSMSC (Comanche P51-P53.agc p.763; Luminary carries the same
 *         routine in POWERED_FLIGHT_SUBROUTINES.agc).
 *
 * Scaling: every direction vector is a "half-unit" vector (unit vector
 * scaled 2, i.e. each component / 2).  ARCTRIG takes sin/4 and cos/4 and
 * returns an angle in revolutions (scale 2^0) in [-0.5, +0.5).  The dot
 * and cross products of two half-unit vectors are therefore already the
 * /4 quantities ARCTRIG wants.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "internal.h"

/* QTSN45: 2DEC .1768 = sin(45 deg)/4.                                    */
#define QTSN45 ((agc_dp_t) 47459389) /* .1768 x 2^28                */
/* .166...: 2DEC .1666666667 = 60 degrees in revolutions.                 */
#define SIXTY_DEG_REV     ((agc_dp_t) 44739243) /* .1666666667 x 2^28         */
#define ALARM_GIMBAL_LOCK ((agc_alarm_code_t) 00401)

static bool vec_in_range(agc_vec_t v)
{
    bool ok = true;

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        ok = ok && v.c[i] >= AGC_DP_MIN && v.c[i] <= AGC_DP_MAX;
    }

    return (ok);
}

/* ARCTRIG (INFLIGHT_ALIGNMENT_ROUTINES.agc p.1357): angle from SINTH =
 * sin/4 and COSTH = cos/4.  Arcsin is used within 45 deg of 0 or 180 deg,
 * arccos otherwise, so neither function is used near its flat region.
 * Result THETA in revolutions, -.5 to +.5.                               */
agc_dp_t agc_imu_arctrig(agc_interp_t *ip, agc_dp_t sinth, agc_dp_t costh)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (0);
    }

    if (!AGC_ASSERT(sinth >= AGC_DP_MIN && sinth <= AGC_DP_MAX))
    {
        return (0);
    }

    if (!AGC_ASSERT(costh >= AGC_DP_MIN && costh <= AGC_DP_MAX))
    {
        return (0);
    }

    agc_dp_t theta;

    if (agc_dp_sub(ip, agc_dp_abs(sinth), QTSN45) >= 0)
    {
        /* (45,135) or (-135,-45): X = ARCCOS(COS) WITH SIGN(SIN).        */
        theta = agc_dp_acos(ip, agc_dp_shl(ip, costh, 1));

        return (agc_dp_sign(theta, sinth));
    }

    /* TRIG1: (-45,45) or (135,-135): X = ARCSIN(SIN).                    */
    theta = agc_dp_asin(ip, agc_dp_shl(ip, sinth, 1));

    if (costh >= 0)
    {
        return (theta); /* (-45,45)          */
    }

    /* TRIG2 (135,-135): X = .5 WITH SIGN(SIN) - ARCSIN(SIN).             */
    return (agc_dp_sub(ip, agc_dp_sign(AGC_DP_HALF, sinth), theta));
}

/* CALCGTA (INFLIGHT_ALIGNMENT_ROUTINES.agc pp.1355-1356): gyro torquing
 * angles that bring the stable member to the desired orientation.  dc rows
 * are XDC, YDC, ZDC: the desired SM axes referred to the present SM
 * (half-unit).  gyro[] = OGC, IGC, MGC (X, Y, Z gyros), revolutions; the
 * same three words are the OGC gyro buffer IMUPULSE reads.               */
void agc_imu_calcgta(agc_interp_t *ip, const agc_mat_t *dc,
                     agc_dp_t gyro[3])
{
    if (!AGC_ASSERT(ip != NULL && dc != NULL && gyro != NULL))
    {
        return;
    }

    if (!AGC_ASSERT(vec_in_range(dc->row[0]) && vec_in_range(dc->row[2])))
    {
        return;
    }

    agc_vec_t xdc = dc->row[0];
    /* ZP = UNIT(-XD3, 0, XD1).                                           */
    agc_vec_t zp = agc_vec_unit(
        ip, agc_vec_make(agc_dp_neg(xdc.c[2]), 0, xdc.c[0]), NULL, NULL);
    /* SIN(IGC) = ZP1, COS(IGC) = ZP3 (SR1: half-unit -> /4).             */
    agc_dp_t igc =
        agc_imu_arctrig(ip, agc_dp_shr(zp.c[0], 1), agc_dp_shr(zp.c[2], 1));
    /* SIN(MGC) = XD2;  COS(MGC) = (ZP3)(XD1) - (ZP1)(XD3).                */
    agc_dp_t pd00 = agc_dp_mul(ip, zp.c[0], xdc.c[2]);
    agc_dp_t cosm = agc_dp_sub(ip, agc_dp_mul(ip, zp.c[2], xdc.c[0]), pd00);
    agc_dp_t mgc  = agc_imu_arctrig(ip, agc_dp_shr(xdc.c[1], 1), cosm);
    /* COS(OGC) = ZP . ZDC,  SIN(OGC) = ZP . YDC.                          */
    agc_dp_t coso = agc_vec_dot(ip, zp, dc->row[2]);
    agc_dp_t sino = agc_vec_dot(ip, zp, dc->row[1]);
    agc_dp_t ogc  = agc_imu_arctrig(ip, sino, coso);

    gyro[0] = ogc;
    gyro[1] = igc;
    gyro[2] = mgc;
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_OGC, gyro);
}

/* GIMLOCK1 (p.1360): middle gimbal angle beyond 60 degrees.              */
static void gimlock1(void)
{
    agc_alarm(ALARM_GIMBAL_LOCK); /* ALARM 00401  */
    agc_status_t st = AGC_STATUS_ALARM;

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_ALARM))
    {
        return;
    }

    agc_flag_set(AGC_FLAG_GLOKFAIL); /* UPFLAG       */
}

/* CALCGA (INFLIGHT_ALIGNMENT_ROUTINES.agc pp.1359-1360): CDU driving
 * angles that bring the stable member to the desired orientation.  nb rows
 * XNB YNB ZNB and sm rows XSM YSM ZSM are half-unit in one common frame.
 * thetad[] = X (outer), Y (inner), Z (middle) CDU patterns.
 * When the middle gimbal angle reaches 60 deg the AGC raises alarm 00401,
 * sets GLOKFAIL and still completes; this returns AGC_STATUS_OK in that
 * case too, callers test GLOKFAIL as the AGC programs do.
 * CM only: if CPHIFLAG is set it is cleared and THETAD is not written (the
 * angles are left in MPAC for the caller, here in thetad[]).             */
agc_status_t agc_imu_calcga(agc_interp_t *ip, const agc_mat_t *nb,
                            const agc_mat_t *sm, uint16_t thetad[3])
{
    if (!AGC_ASSERT(ip != NULL && nb != NULL && sm != NULL && thetad != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(vec_in_range(nb->row[0]) && vec_in_range(sm->row[1])))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_vec_t xnb = nb->row[0];
    const agc_vec_t ysm = sm->row[1];
    /* MGA = UNIT(OGA x IGA), OGA = XNB, IGA = YSM.                        */
    agc_vec_t mga = agc_vec_unit(ip, agc_vec_cross(ip, xnb, ysm), NULL, NULL);
    agc_dp_t  ogc = agc_imu_arctrig(ip, agc_vec_dot(ip, mga, nb->row[1]),
                                    agc_vec_dot(ip, mga, nb->row[2]));
    /* COS(MG) = IGA . (MGA x OGA) (SL1),  SIN(MG) = IGA . OGA.            */
    agc_dp_t cosm =
        agc_dp_shl(ip, agc_vec_dot(ip, agc_vec_cross(ip, mga, xnb), ysm), 1);
    agc_dp_t mgc = agc_imu_arctrig(ip, agc_vec_dot(ip, ysm, xnb), cosm);

    if (agc_dp_sub(ip, agc_dp_abs(mgc), SIXTY_DEG_REV) >= 0)
    {
        gimlock1();
    }

    /* CALCGA1: COS(IG) = ZSM . MGA,  SIN(IG) = XSM . MGA.                 */
    agc_dp_t igc = agc_imu_arctrig(ip, agc_vec_dot(ip, sm->row[0], mga),
                                   agc_vec_dot(ip, sm->row[2], mga));
    agc_status_t st = agc_rtb_v1sto2s(agc_vec_make(ogc, igc, mgc), thetad);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_CSM &&
        imu_vflag_test(IMU_VF_CPHIFLAG))
    {
        imu_vflag_clear(IMU_VF_CPHIFLAG);

        return (AGC_STATUS_OK); /* BONCLR      */
    }

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_thetad(i, thetad[i]); /* index valid */
    }

    return (AGC_STATUS_OK);
}

/* AXISGEN1 (p.1361) for one frame: U = S1, V = UNIT(S1 x S2),
 * W = (U x V) SL1, all half-unit.                                        */
static void axisgen_triad(agc_interp_t *ip, agc_vec_t s1, agc_vec_t s2,
                          agc_vec_t t[3])
{
    if (!AGC_ASSERT(vec_in_range(s1) && vec_in_range(s2)))
    {
        return;
    }

    t[0] = s1;
    t[1] = agc_vec_unit(ip, agc_vec_cross(ip, s1, s2), NULL, NULL);
    t[2] = agc_vec_shl(ip, agc_vec_cross(ip, t[0], t[1]), 1);

    if (!AGC_ASSERT(vec_in_range(t[2])))
    {
        return;
    }
}

/* AXISGEN (INFLIGHT_ALIGNMENT_ROUTINES.agc pp.1361-1362): coordinates of
 * frame A referred to frame B from two stars known in both.
 * a[0], a[1] = star 1, 2 in A (the AGC's STARAD, STARAD +6); b[0], b[1]
 * = star 1, 2 in B (VAC 6D, 12D).  Output rows XDC, YDC, ZDC (half-unit):
 *     row i = UNIT( (UB)(UA_i) + (VB)(VA_i) + (WB)(WA_i) ) .            */
agc_status_t agc_imu_axisgen(agc_interp_t *ip, const agc_vec_t a[2],
                             const agc_vec_t b[2], agc_mat_t *a_wrt_b)
{
    if (!AGC_ASSERT(ip != NULL && a != NULL && b != NULL && a_wrt_b != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(vec_in_range(a[0]) && vec_in_range(b[0])))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t ta[3];
    agc_vec_t tb[3];

    axisgen_triad(ip, b[0], b[1], tb); /* first pass: VAC area       */
    axisgen_triad(ip, a[0], a[1], ta); /* second pass: STARAD        */

    for (int i = 0; i < 3; i++)
    { /* AXISGEN2: X2 = 6, 4, 2     */
        agc_vec_t j   = agc_vec_scale(ip, tb[0], ta[0].c[i]);
        agc_vec_t k   = agc_vec_scale(ip, tb[1], ta[1].c[i]);
        agc_vec_t l   = agc_vec_scale(ip, tb[2], ta[2].c[i]);
        agc_vec_t sum =
            agc_vec_shl(ip, agc_vec_add(ip, agc_vec_add(ip, l, j), k), 1);

        a_wrt_b->row[i] = agc_vec_unit(ip, sum, NULL, NULL);
    }

    return (AGC_STATUS_OK);
}

/* CALCSMSC (P51-P53.agc p.763): navigation base axes in stable-member
 * coordinates from the CDU angles.  sincdu/coscdu = sin/2, cos/2 of the
 * X, Y, Z CDU angles (as left by CDUTRIG).  With sX = sin CDUX etc.:
 *   XNB = ( cY cZ,               sZ,      -sY cZ )
 *   ZNB = ( cX sY + sX cY sZ,   -sX cZ,    cX cY - sX sY sZ )
 *   YNB = ZNB x XNB
 * each output half-unit.                                                 */
agc_status_t agc_imu_calcsmsc(agc_interp_t *ip, agc_vec_t sincdu,
                              agc_vec_t coscdu, agc_mat_t *nb)
{
    if (!AGC_ASSERT(ip != NULL && nb != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(vec_in_range(sincdu) && vec_in_range(coscdu)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_dp_t sx = sincdu.c[0], sy = sincdu.c[1], sz = sincdu.c[2];
    const agc_dp_t cx = coscdu.c[0], cy = coscdu.c[1], cz = coscdu.c[2];
    agc_vec_t      xnb = agc_vec_make(agc_dp_mul(ip, cy, cz), agc_dp_shr(sz, 1),
                                      agc_dp_neg(agc_dp_mul(ip, sy, cz)));

    nb->row[0] = agc_vec_shl(ip, xnb, 1);             /* VSL1     */
    agc_dp_t t26 = agc_dp_shl(ip, agc_dp_mul(ip, sx, sz), 1); /* 26D      */
    agc_dp_t z3  =
        agc_dp_sub(ip, agc_dp_mul(ip, cx, cy), agc_dp_mul(ip, t26, sy));
    agc_dp_t z2 = agc_dp_neg(agc_dp_mul(ip, sx, cz));
    agc_dp_t z1 =
        agc_dp_add(ip, agc_dp_mul(ip, cy, t26), agc_dp_mul(ip, cx, sy));

    nb->row[2] = agc_vec_shl(ip, agc_vec_make(z1, z2, z3), 1);
    nb->row[1] = agc_vec_shl(ip, agc_vec_cross(ip, nb->row[2], nb->row[0]), 1);

    return (AGC_STATUS_OK);
}
