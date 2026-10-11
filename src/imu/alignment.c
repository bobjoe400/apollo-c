/**********************************************************************
 * FILE NAME: alignment.c
 *
 * PURPOSE:
 *   Inflight alignment routines: CALCGTA (gyro torquing angles), ARCTRIG,
 *   CALCGA (CDU driving angles, GIMLOCK1), AXISGEN (one frame in
 *   another from two stars) and CALCSMSC (navigation base in SM axes).
 *
 * AGC SOURCE:
 *   Comanche055 INFLIGHT_ALIGNMENT_ROUTINES.agc pp. 1355-1363,
 *   P51-P53.agc p. 763 (CALCSMSC)
 *   Luminary099 INFLIGHT_ALIGNMENT_ROUTINES.agc pp. 1250-1258 (same text
 *   except the CM CPHIFLAG test at the end of CALCGA); CALCSMSC is in
 *   POWERED_FLIGHT_SUBROUTINES.agc
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   THETAD        uint16_t   O    CDU            agc_erase_set_thetad()
 *   OGC buffer    agc_dp_t   O    rev 2^0        agc_imu_gyro_buf_set()
 *   GLOKFAIL,     flags      I/O  --             agc_flag_*()
 *   CPHIFLAG
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_vec_unit, agc_vec_cross, agc_vec_dot_tp, agc_dp_asin/acos
 *   agc_rtb_v1sto2s       V1STO2S (THETAD patterns)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 00401 (gimbal lock, GLOKFAIL) from CALCGA; it still completes.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles; interpretive subroutines, no restart protection.
 *
 * NOTES:
 *   Every direction vector is a "half-unit" vector (unit vector scaled 2,
 *   i.e. each component / 2).  ARCTRIG takes sin/4 and cos/4 and returns
 *   an angle in revolutions (scale 2^0) in [-0.5, +0.5).  The dot and
 *   cross products of two half-unit vectors are therefore already the /4
 *   quantities ARCTRIG wants.  Every product is DMP (truncating); where
 *   the AGC shifts a product before storing it (DMP SL1, DOT SL1) the
 *   shift acts on the TP product.
 *
 * ALGORITHM:
 *   As the AGC listing; see each function prolog.
 **********************************************************************/
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
#define ALARM_GIMBAL_LOCK ((agc_alarm_code_t) 00401) /* GIMLOCK1 */

/**********************************************************************
 * AGC: none (checker) -- every component a valid DP
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I    vector
 *
 * RETURNS: bool -- true when all three components are in range
 **********************************************************************/
static bool vec_in_range(agc_vec_t v)
{
    bool ok = true; /* all components in range */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        ok = ok && v.c[i] >= AGC_DP_MIN && v.c[i] <= AGC_DP_MAX;
    }

    return (ok);
}

/**********************************************************************
 * ARCTRIG / TRIG1 / TRIG2 (INFLIGHT_ALIGNMENT_ROUTINES.agc p.1357) --
 * angle from its sine and cosine
 *
 * |sin| >= sin 45: ARCCOS(cos) with the sign of sin; else ARCSIN(sin),
 * and .5 with the sign of sin minus it when cos < 0.
 *
 * ARGUMENTS:
 *   ip     agc_interp_t *  I/O  interpreter state
 *   sinth  agc_dp_t        I    SINTH = sin/4, 2^0
 *   costh  agc_dp_t        I    COSTH = cos/4, 2^0
 *
 * RETURNS: agc_dp_t -- THETA, rev 2^0 in [-.5, .5)
 **********************************************************************/
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

    agc_dp_t theta; /* angle, rev 2^0 */

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

/**********************************************************************
 * CALCGTA (INFLIGHT_ALIGNMENT_ROUTINES.agc pp.1355-1356) -- gyro
 * torquing angles
 *
 * Angles that bring the stable member to the desired orientation.  dc
 * rows XDC, YDC, ZDC: the desired SM axes in the present SM, B1.  The
 * three words are also the OGC gyro buffer IMUPULSE reads.
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *     I/O  interpreter state
 *   dc    const agc_mat_t *  I    XDC, YDC, ZDC, B1
 *   gyro  agc_dp_t [3]       O    OGC, IGC, MGC, rev 2^0
 **********************************************************************/
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

    agc_vec_t xdc = dc->row[0]; /* XDC, B1 */
    /* ZP = UNIT(-XD3, 0, XD1).                                           */
    agc_vec_t zp = agc_vec_unit(
        ip, agc_vec_make(agc_dp_neg(xdc.c[2]), 0, xdc.c[0]), NULL, NULL); /* ZPRIME, B1 */
    /* SIN(IGC) = ZP1, COS(IGC) = ZP3 (SR1: half-unit -> /4).             */
    agc_dp_t igc =
        agc_imu_arctrig(ip, agc_dp_shr(zp.c[0], 1), agc_dp_shr(zp.c[2], 1)); /* IGC, rev 2^0 */
    /* SIN(MGC) = XD2;  COS(MGC) = (ZP3)(XD1) - (ZP1)(XD3).                */
    /* DMP PDDL / DMP DSU / STCALL: the DSU acts on MPAC, MPAC+1 and the
     * store drops MPAC+2, so both products are truncated (DMP).          */
    agc_dp_t pd00 = agc_dp_mul_trunc(zp.c[0], xdc.c[2]); /* ZP1 XD3, 2^-2 */
    agc_dp_t cosm = agc_dp_sub(ip, agc_dp_mul_trunc(zp.c[2], xdc.c[0]), pd00); /* COS(MGC)/4, 2^0 */
    agc_dp_t mgc  = agc_imu_arctrig(ip, agc_dp_shr(xdc.c[1], 1), cosm); /* MGC, rev 2^0 */
    /* COS(OGC) = ZP . ZDC,  SIN(OGC) = ZP . YDC.                          */
    agc_dp_t coso = agc_vec_dot(ip, zp, dc->row[2]); /* COS(OGC)/4, 2^0 */
    agc_dp_t sino = agc_vec_dot(ip, zp, dc->row[1]); /* SIN(OGC)/4, 2^0 */
    agc_dp_t ogc  = agc_imu_arctrig(ip, sino, coso); /* OGC, rev 2^0 */

    gyro[0] = ogc;
    gyro[1] = igc;
    gyro[2] = mgc;
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_OGC, gyro);
}

/**********************************************************************
 * GIMLOCK1 (INFLIGHT_ALIGNMENT_ROUTINES.agc p.1360) -- middle gimbal
 * beyond 60 deg
 *
 * ALARM 00401, UPFLAG GLOKFAIL, then CALCGA1 goes on.
 **********************************************************************/
static void gimbal_lock_alarm(void)
{
    agc_alarm(ALARM_GIMBAL_LOCK); /* ALARM 00401  */
    agc_status_t st = AGC_STATUS_ALARM; /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_ALARM))
    {
        return;
    }

    agc_flag_set(AGC_FLAG_GLOKFAIL); /* UPFLAG       */
}

/**********************************************************************
 * CALCGA / CALCGA1 (INFLIGHT_ALIGNMENT_ROUTINES.agc pp.1359-1360) --
 * CDU driving angles
 *
 * Angles that bring the stable member to the desired orientation.  nb
 * and sm rows are half-unit in one common frame.  At 60 deg middle gimbal
 * the AGC raises ALARM 00401, sets GLOKFAIL and still completes; this
 * returns OK then too and callers test GLOKFAIL as the AGC programs do.
 * CM only: CPHIFLAG set -> cleared, THETAD not written (BONCLR CPHIFLAG
 * S2).
 *
 * ARGUMENTS:
 *   ip      agc_interp_t *     I/O  interpreter state
 *   nb      const agc_mat_t *  I    XNB, YNB, ZNB, B1
 *   sm      const agc_mat_t *  I    XSM, YSM, ZSM, B1
 *   thetad  uint16_t [3]       O    X (outer), Y (inner), Z (middle)
 *                                   CDU patterns
 *
 * RETURNS: agc_status_t -- OK, or the V1STO2S status
 **********************************************************************/
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

    const agc_vec_t xnb = nb->row[0]; /* XNB = OGA, B1 */
    const agc_vec_t ysm = sm->row[1]; /* YSM = IGA, B1 */
    /* MGA = UNIT(OGA x IGA), OGA = XNB, IGA = YSM.                        */
    agc_vec_t mga = agc_vec_unit(ip, agc_vec_cross(ip, xnb, ysm), NULL, NULL); /* MGA, B1 */
    agc_dp_t  ogc = agc_imu_arctrig(ip, agc_vec_dot(ip, mga, nb->row[1]),
                                    agc_vec_dot(ip, mga, nb->row[2])); /* OGC, rev 2^0 */
    /* COS(MG) = IGA . (MGA x OGA) (SL1),  SIN(MG) = IGA . OGA.            */
    agc_dp_t cosm = agc_tp_to_dp(agc_tp_shift( /* DOT SL1 (TP), cos/4 2^0 */
                                     ip,
                                     agc_vec_dot_tp(ip,
                                                    agc_vec_cross(ip, mga, xnb),
                                                    ysm), 1));
    agc_dp_t mgc = agc_imu_arctrig(ip, agc_vec_dot(ip, ysm, xnb), cosm); /* MGC, rev 2^0 */

    if (agc_dp_sub(ip, agc_dp_abs(mgc), SIXTY_DEG_REV) >= 0)
    {
        gimbal_lock_alarm();
    }

    /* CALCGA1: COS(IG) = ZSM . MGA,  SIN(IG) = XSM . MGA.                 */
    agc_dp_t igc = agc_imu_arctrig(ip, agc_vec_dot(ip, sm->row[0], mga),
                                   agc_vec_dot(ip, sm->row[2], mga)); /* IGC, rev 2^0 */
    agc_status_t st = agc_rtb_v1sto2s(agc_vec_make(ogc, igc, mgc), thetad); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_CSM &&
        imu_vflag_test(IMU_VF_CPHIFLAG))
    {
        agc_flag_id_t id = imu_vflag(IMU_VF_CPHIFLAG); /* CPHIFLAG id for this vehicle */

        if (!AGC_ASSERT(id != 0U))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        if (!AGC_ASSERT(agc_flag_valid(id)))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_flag_clear(id);

        return (AGC_STATUS_OK); /* BONCLR      */
    }

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_thetad(i, thetad[i]); /* index valid */
    }

    return (AGC_STATUS_OK);
} /* agc_imu_calcga */

/**********************************************************************
 * AXISGEN1 (INFLIGHT_ALIGNMENT_ROUTINES.agc p.1361) -- the triad of one
 * frame
 *
 * U = S1, V = UNIT(S1 x S2), W = (U x V) VSL1, all half-unit.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state
 *   s1  agc_vec_t       I    star 1, B1
 *   s2  agc_vec_t       I    star 2, B1
 *   t   agc_vec_t [3]   O    U, V, W, B1
 **********************************************************************/
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

/**********************************************************************
 * AXISGEN / AXISGEN2 (INFLIGHT_ALIGNMENT_ROUTINES.agc pp.1361-1362) --
 * frame A referred to frame B
 *
 * Two stars known in both frames.  Row i = UNIT((UB)(UA i) + (VB)(VA i)
 * + (WB)(WA i)).  The AGC also copies XDC..ZDC into STARAD; the C
 * callers keep STARAD themselves (none reads it back).
 *
 * ARGUMENTS:
 *   ip       agc_interp_t *       I/O  interpreter state
 *   a        const agc_vec_t [2]  I    stars 1, 2 in A (STARAD, STARAD
 *                                      +6), B1
 *   b        const agc_vec_t [2]  I    stars 1, 2 in B (VAC 6D, 12D),
 *                                      B1
 *   a_wrt_b  agc_mat_t *          O    XDC, YDC, ZDC, B1
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a NULL argument
 **********************************************************************/
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

    agc_vec_t ta[3]; /* UA, VA, WA: frame A triad, B1 */
    agc_vec_t tb[3]; /* UB, VB, WB: frame B triad, B1 */

    axisgen_triad(ip, b[0], b[1], tb); /* first pass: VAC area       */
    axisgen_triad(ip, a[0], a[1], ta); /* second pass: STARAD        */

    for (int i = 0; i < 3; i++)
    { /* AXISGEN2: X2 = 6, 4, 2     */
        agc_vec_t j   = agc_vec_scale(ip, tb[0], ta[0].c[i]); /* (UB)(UA i), 2^-2 */
        agc_vec_t k   = agc_vec_scale(ip, tb[1], ta[1].c[i]); /* (VB)(VA i), 2^-2 */
        agc_vec_t l   = agc_vec_scale(ip, tb[2], ta[2].c[i]); /* (WB)(WA i), 2^-2 */
        agc_vec_t sum =
            agc_vec_shl(ip, agc_vec_add(ip, agc_vec_add(ip, l, j), k), 1); /* J + K + L VSL1, B1 */

        a_wrt_b->row[i] = agc_vec_unit(ip, sum, NULL, NULL);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CALCSMSC (P51-P53.agc p.763) -- navigation base axes in SM
 * coordinates
 *
 * XNB = (cY cZ, sZ, -sY cZ), ZNB = (cX sY + sX cY sZ, -sX cZ, cX cY -
 * sX sY sZ), YNB = ZNB x XNB; every product DMP (26D = sX sZ is DMP SL1
 * on the TP product).
 *
 * ARGUMENTS:
 *   ip      agc_interp_t *  I/O  interpreter state
 *   sincdu  agc_vec_t       I    sin/2 of CDUX, Y, Z, 2^0
 *   coscdu  agc_vec_t       I    cos/2 of CDUX, Y, Z, 2^0
 *   nb      agc_mat_t *     O    XNB, YNB, ZNB, B1
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on NULL or out of range
 **********************************************************************/
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

    const agc_dp_t sx  = sincdu.c[0]; /* SINCDUX / 2, 2^0               */
    const agc_dp_t sy  = sincdu.c[1]; /* SINCDUY / 2, 2^0               */
    const agc_dp_t sz  = sincdu.c[2]; /* SINCDUZ / 2, 2^0               */
    const agc_dp_t cx  = coscdu.c[0]; /* COSCDUX / 2, 2^0               */
    const agc_dp_t cy  = coscdu.c[1]; /* COSCDUY / 2, 2^0               */
    const agc_dp_t cz  = coscdu.c[2]; /* COSCDUZ / 2, 2^0               */
    agc_vec_t      xnb = agc_vec_make(agc_dp_mul_trunc(cy, cz), /* DMP, 2^-1 */
                                      agc_dp_shr(sz, 1),
                                      agc_dp_neg(agc_dp_mul_trunc(sy, cz)));

    nb->row[0] = agc_vec_shl(ip, xnb, 1);             /* VSL1     */
    /* 26D = sX sZ: DMP SL1 on the TP product, STORE truncates.          */
    agc_dp_t t26 =
        agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(sx, sz), 1)); /* 26D, 2^-1 */
    agc_dp_t z3 = agc_dp_sub(ip, agc_dp_mul_trunc(cx, cy), /* DSU, 2^-1 */
                             agc_dp_mul_trunc(t26, sy));
    agc_dp_t z2 = agc_dp_neg(agc_dp_mul_trunc(sx, cz)); /* DCOMP, 2^-1 */
    agc_dp_t z1 = agc_dp_add(ip, agc_dp_mul_trunc(cy, t26), /* DAD, 2^-1 */
                             agc_dp_mul_trunc(cx, sy));

    nb->row[2] = agc_vec_shl(ip, agc_vec_make(z1, z2, z3), 1);
    nb->row[1] = agc_vec_shl(ip, agc_vec_cross(ip, nb->row[2], nb->row[0]), 1);

    return (AGC_STATUS_OK);
}
