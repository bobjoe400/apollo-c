/**********************************************************************
 * FILE NAME: attitude_maneuver.c
 *
 * PURPOSE:
 *   R60 attitude maneuver routine, R62 crew-defined maneuver, VECPOINT,
 *   and (LM) BALLANGS, G+N,AUTO and ISITAUTO.
 *
 * AGC SOURCE:
 *   Comanche055 R60_62.agc pp. 390-398
 *   Luminary099 R60_62.agc pp. 342-363 (differs: see NOTES)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   CPHI..CPSI    uint16_t   O    CDU            agc_erase_set_thetad()
 *   3AXISFLG,     flags      I/O  --             agc_flag_*()
 *   PDSPFLAG,
 *   TRACKFLG
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_kalc_cdutodcm     CDUTODCM, DELCOMP, DCMTOCDU (maneuver_calc.c)
 *   agc_imu_gomanur       GOMANUR (maneuver.c)
 *   agc_dsky_display      GOPERF2R, GODSPR, GOFLASH
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   V34 on V06N18: GOTOPOOH, or GOTOV56 in R61.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles; R60 runs in the calling job; restart group 1 (CM) or
 *   2 (LM) for the priority display.
 *
 * NOTES:
 *   Differences kept:
 *   * LM converts the final gimbal angles to FDAI ball angles (BALLANGS)
 *     before each V06N18 display and has VECPNT1 (from given CDUs).
 *   * the auto-mode test: CM reads channel 31 bits 13-15 ("mode auto and
 *     control GNC"); LM G+N,AUTO reads channel 30 bit 10 then ISITAUTO
 *     channel 31 bit 14.
 *   * RELINUS (restart of a priority display): CM continues only with
 *     TRACKFLG, sets PDSPFLAG and TARG1FLG, OPTIND = 0, priority 14, else
 *     restart 1.11 (PIKUP20); LM priority 26, with TRACKFLG sets
 *     PDSPFLAG, else restart 2.7 (P20) or 2.11 (P25).  CHKLINUS restart
 *     point is CM 1.7, LM 2.13.
 *   * ROT180 / PICKX: CM HIUNITX, LM HIDPHALF (the same vector .5, 0, 0).
 *   The AGC's display calls return to the job at different points for
 *   the crew's responses; here each response is its own job
 *   (agc_dsky_cont_t).
 *
 * ALGORITHM:
 *   As the AGC listing; see each function prolog.
 **********************************************************************/
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
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/rendezvous.h"
#include "internal.h"

#define SINGIMLC ((agc_dp_t) 115047034)  /* 2DEC .4285836003 = sin(59)/2  */
#define SINVEC1  ((agc_dp_t) 101907670)  /* 2DEC .3796356537 = sin(49.4)/2 */
#define SINVEC2  ((agc_dp_t) 66091971)   /* 2DEC .2462117800 = sin(29.5)/2 */
#define VECANG1  ((agc_dp_t) 37282702)   /* 2DEC .1388888889 = 50 deg      */
#define VECANG2  ((agc_dp_t) 26097892)   /* 2DEC .09722222222 = 35 deg     */
#define DPB14    ((agc_dp_t) (1L << 14)) /* DPB-14: OCT 00001, OCT 00000   */
#define CDU_MASK 077777u                 /* 15-bit CDU word          */
#define PRIO14   ((agc_prio_t) 014)      /* CM RELINUS priority      */
#define PRIO26   ((agc_prio_t) 026)      /* LM RELINUS priority      */
#define BIT7     0100u                   /* LINUS (BLANKET) code          */

static agc_vec_t    scaxis;                   /* SCAXIS: S/C axis to point, B1           */
static agc_vec_t    pointvsm;                 /* POINTVSM: direction in SM axes, B1      */
static uint16_t     fdai[3];                  /* FDAIX, FDAIY, FDAIZ (LM), CDU patterns   */
static agc_job_id_t attitude_maneuver_return; /* TEMPR60: where R60 returns               */
static bool         relinus_static;           /* last R60 display was the static V06N18   */

/**********************************************************************
 * AGC: none (checker) -- every component a valid DP
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I    vector
 *
 * RETURNS: bool -- true when in range
 **********************************************************************/
static bool vec_ok(agc_vec_t v)
{
    bool ok = true; /* all components in range */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        ok = ok && v.c[i] >= AGC_DP_MIN && v.c[i] <= AGC_DP_MAX;
    }

    return (ok);
}

/**********************************************************************
 * AGC: none (accessor) -- SCAXIS, the S/C axis to point
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I    SCAXIS, B1
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG out of range
 **********************************************************************/
agc_status_t agc_imu_set_scaxis(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(v)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    scaxis = v;

    if (!AGC_ASSERT(vec_ok(scaxis)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (accessor) -- POINTVSM, the direction in SM axes
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I    POINTVSM, B1
 **********************************************************************/
void agc_imu_set_pointvsm(agc_vec_t v)
{
    pointvsm = v;
}

/**********************************************************************
 * AGC: none (accessor) -- FDAIX, FDAIY, FDAIZ (LM, noun 18)
 *
 * ARGUMENTS:
 *   axis  int  I    0 X, 1 Y, 2 Z
 *
 * RETURNS: uint16_t -- FDAI angle, CDU pattern
 **********************************************************************/
uint16_t agc_erase_fdai(int axis)
{
    return (fdai[axis]);
}

/* ======================================================================= */
/* VECPOINT (p.393-397 CM / LM)                                            */
/* ======================================================================= */

/**********************************************************************
 * FINDGIMB (R60_62.agc p.395 CM) -- commanded CDU angles from the
 * S/C-to-SM matrix
 *
 * DCMTOCDU, then RTB V1STO2S (2:S complement).
 *
 * ARGUMENTS:
 *   ip      agc_interp_t *     I/O  interpreter state
 *   m       const agc_mat_t *  I    MFS, B1
 *   gimbal  uint16_t [3]       O    CPHI, CTHETA, CPSI patterns
 *
 * RETURNS: agc_status_t -- OK or BAD_ARG
 **********************************************************************/
static agc_status_t findgimb(agc_interp_t *ip, const agc_mat_t *m, uint16_t gimbal[3])
{
    agc_vec_t    ang;                                        /* PHI, THETA, PSI, rev 2^0 */
    agc_dp_t     cmga;                                       /* COS(MGA), 2^0 */
    agc_status_t st = agc_kalc_dcmtocdu(ip, m, &ang, &cmga); /* status of the call */

    if (st == AGC_STATUS_OK)
    {
        agc_rtb_v1sto2s(ang, gimbal); /* 2:S COMPLEMENT     */
    }

    return (st);
}

/**********************************************************************
 * COMPMATX .. IGSAMEX / CHEKAXIS / COMPMFSN (R60_62.agc pp.394-395 CM)
 * -- final attitude
 *
 * MFS = MIS DEL(COF, AM).  Within 59 deg of gimbal lock, and not the
 * thrust axis, rotate a further 50 deg (AOT) or 35 deg about +-SCAXIS.
 *
 * ARGUMENTS:
 *   ip      agc_interp_t *     I/O  interpreter state
 *   mis     const agc_mat_t *  I    MIS, B1
 *   cofv    agc_vec_t          I    COF, rotation axis, B1
 *   am      agc_dp_t           I    AM, rev 2^0
 *   gimbal  uint16_t [3]       O    CPHI, CTHETA, CPSI patterns
 *
 * RETURNS: agc_status_t -- OK or BAD_ARG
 **********************************************************************/
static agc_status_t compmatx(agc_interp_t *ip, const agc_mat_t *mis, agc_vec_t cofv, agc_dp_t am, uint16_t gimbal[3])
{
    agc_mat_t    del;                                       /* DEL, 2^0 */
    agc_status_t st = agc_kalc_delcomp(ip, cofv, am, &del); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_mat_t mfs = agc_mat_mul(ip, mis, &del); /* MXM3     */

    if (agc_dp_sub(ip, agc_dp_abs(mfs.row[1].c[0]), SINGIMLC) < 0 || agc_dp_sub(ip, agc_dp_abs(scaxis.c[0]), SINVEC1) >= 0)
    {
        return (findgimb(ip, &mfs, gimbal)); /* no lock, or thrust axis */
    }

    agc_vec_t ig = mfs.row[1]; /* IG axis in final S/C, B1 */

    if (ig.c[0] < 0)
    {
        ig = agc_vec_neg(ig);
    }

    agc_vec_t u   = (agc_vec_cross(ip, ig, scaxis).c[0] < 0) ? scaxis : agc_vec_neg(scaxis);     /* IGSAMEX, B1; BMN U=SCAXIS; U=SCAXIS */
    agc_dp_t  ang = (agc_dp_sub(ip, agc_dp_abs(scaxis.c[0]), SINVEC2) >= 0) ? VECANG1 : VECANG2; /* CHEKAXIS: 50 or 35 deg, rev 2^0; BPL PICKANG1; CHEKAXIS */

    st = agc_kalc_delcomp(ip, u, ang, &del); /* COMPMFSN */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_mat_t m2 = agc_mat_mul(ip, &mfs, &del); /* MFS after COMPMFSN, B1 */

    return (findgimb(ip, &m2, gimbal));
} /* compmatx */

/**********************************************************************
 * AGC: none (DMP SLn, STORE) -- shift the TP product, then truncate
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state
 *   a   agc_dp_t        I    multiplicand
 *   b   agc_dp_t        I    multiplier
 *   n   int             I    left shift, 1 or 2
 *
 * RETURNS: agc_dp_t -- a b 2^n truncated to two words
 **********************************************************************/
static agc_dp_t dmp_shl(agc_interp_t *ip, agc_dp_t a, agc_dp_t b, int n)
{
    return (agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(a, b), n)));
}

/**********************************************************************
 * PICKAXIS / ROT180 / XROT / PICKX (R60_62.agc p.395 CM) -- VF x VI is
 * (nearly) zero
 *
 * Parallel: keep the present CDUs.  Antiparallel: rotate 180 deg about
 * an axis in the plane of X(I) and Y(SM) perpendicular to VI, or about X.
 *
 * ARGUMENTS:
 *   ip      agc_interp_t *      I/O  interpreter state
 *   mis     const agc_mat_t *   I    MIS, B1
 *   vf      agc_vec_t           I    VF, B1
 *   cdu0    const uint16_t [3]  I    present CDUs (PD25-27)
 *   gimbal  uint16_t [3]        O    CPHI, CTHETA, CPSI patterns
 *
 * RETURNS: agc_status_t -- OK or BAD_ARG
 **********************************************************************/
static agc_status_t pickaxis(agc_interp_t *ip, const agc_mat_t *mis, agc_vec_t vf, const uint16_t cdu0[3], uint16_t gimbal[3])
{
    if (agc_vec_dot(ip, vf, scaxis) >= 0)
    {
        for (int i = 0; i < 3; i++)
        {
            gimbal[i] = cdu0[i]; /* CDU desired = present   */
        }

        return (AGC_STATUS_OK);
    }

    const agc_vec_t hiunitx = agc_vec_make(AGC_DP_HALF, 0, 0);                                       /* HIUNITX, B1 */
    agc_dp_t        mag     = 0;                                                                     /* |cross product|, B1 */
    agc_vec_t       c       = agc_vec_unit(ip, agc_vec_cross(ip, mis->row[1], hiunitx), NULL, NULL); /* cross product axis, B1 */

    c = agc_vec_unit(ip, agc_vec_cross(ip, c, scaxis), &mag, NULL);

    if (agc_interp_test_ovf(ip) || agc_dp_sub(ip, mag, DPB14) < 0)
    {
        c = hiunitx; /* PICKX    */
    }

    return (compmatx(ip, mis, c, AGC_DP_HALF, gimbal)); /* XROT     */
}

/**********************************************************************
 * VECPOINT / VECLEAR (R60_62.agc pp.393-394 CM) -- the body from given
 * CDU angles
 *
 * ARGUMENTS:
 *   ip      agc_interp_t *      I/O  interpreter state
 *   cdu0    const uint16_t [3]  I    initial CDU patterns
 *   gimbal  uint16_t [3]        O    CPHI, CTHETA, CPSI patterns
 *
 * RETURNS: agc_status_t -- OK or BAD_ARG
 **********************************************************************/
static agc_status_t vecpoint_from(agc_interp_t *ip, const uint16_t cdu0[3], uint16_t gimbal[3])
{
    agc_interp_clear_ovf(ip);                            /* BOV VECLEAR: clear OVFIND */
    agc_mat_t    mis;                                    /* MIS, B1 */
    agc_status_t st = agc_kalc_cdutodcm(ip, cdu0, &mis); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_vec_t vf  = agc_vec_unit(ip, agc_vec_mat(ip, pointvsm, &mis), NULL, NULL); /* VF = UNIT(POINTVSM MIS), B1 */
    agc_dp_t  mag = 0;                                                             /* |cross product|, B1 */
    agc_vec_t c   = agc_vec_unit(ip, agc_vec_cross(ip, vf, scaxis), &mag, NULL);   /* cross product axis, B1 */

    if (agc_interp_test_ovf(ip) || agc_dp_sub(ip, mag, DPB14) < 0)
    {
        return (pickaxis(ip, &mis, vf, cdu0, gimbal));
    }

    if (!AGC_ASSERT(mag >= DPB14))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t cofv = agc_vec_neg(c);                                                                     /* VCOMP: COF, B1 */
    agc_dp_t  am   = agc_dp_acos(ip, agc_tp_to_dp(agc_tp_shift(ip, agc_vec_dot_tp(ip, scaxis, vf), 1))); /* DOT SL1 (TP), ARCCOS: rev 2^0 */

    return (compmatx(ip, &mis, cofv, am, gimbal));
}

/**********************************************************************
 * VECPOINT (R60_62.agc pp.393-397 CM) -- point SCAXIS along POINTVSM
 *
 * From the present CDU angles (READCDUK).
 *
 * ARGUMENTS:
 *   ip           agc_interp_t *  I/O  interpreter state
 *   pointvsm_in  agc_vec_t       I    POINTVSM, B1
 *   scaxis_in    agc_vec_t       I    SCAXIS, B1
 *   gimbal       uint16_t [3]    O    CPHI, CTHETA, CPSI patterns
 *
 * RETURNS: agc_status_t -- OK or BAD_ARG
 **********************************************************************/
agc_status_t agc_imu_vecpoint(agc_interp_t *ip, agc_vec_t pointvsm_in, agc_vec_t scaxis_in, uint16_t gimbal[3])
{
    agc_imu_set_pointvsm(pointvsm_in);

    if (agc_imu_set_scaxis(scaxis_in) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    const uint16_t cdu0[3] = {agc_erase_cdux(), agc_erase_cduy(), agc_erase_cduz()};

    agc_relint();

    return (vecpoint_from(ip, cdu0, gimbal));
}

/**********************************************************************
 * VECPNT1 (Luminary R60_62.agc p.353) -- VECPOINT from the given CDUs
 *
 * ARGUMENTS:
 *   ip           agc_interp_t *      I/O  interpreter state
 *   cdu0         const uint16_t [3]  I    initial CDU patterns
 *   pointvsm_in  agc_vec_t           I    POINTVSM, B1
 *   scaxis_in    agc_vec_t           I    SCAXIS, B1
 *   gimbal       uint16_t [3]        O    CPHI, CTHETA, CPSI patterns
 *
 * RETURNS: agc_status_t -- OK or BAD_ARG
 **********************************************************************/
agc_status_t agc_imu_vecpoint_from_cdus(agc_interp_t  *ip,
                                        const uint16_t cdu0[3],
                                        agc_vec_t      pointvsm_in,
                                        agc_vec_t      scaxis_in,
                                        uint16_t       gimbal[3])
{
    agc_imu_set_pointvsm(pointvsm_in);

    if (agc_imu_set_scaxis(scaxis_in) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (vecpoint_from(ip, cdu0, gimbal));
}

/* ======================================================================= */
/* LM: BALLANGS, G+N,AUTO, ISITAUTO (pp.346-348)                           */
/* ======================================================================= */

/**********************************************************************
 * BALLANGS (Luminary R60_62.agc pp.477-478) -- FDAI ball angles
 *
 * YAW = ARCSIN(-SX CZ), ROLL = ARCTAN(SZ / CZ CX), PITCH = ARCTAN((CX SY
 * + SX SZ CY) / (CX CY - SX SZ SY)); every product DMP.
 *
 * ARGUMENTS:
 *   ip      agc_interp_t *      I/O  interpreter state
 *   gimbal  const uint16_t [3]  I    CPHI, CTHETA, CPSI patterns
 *   out     uint16_t [3]        O    FDAIX, Y, Z patterns (roll, pitch,
 *                                    yaw)

 **********************************************************************/
void agc_imu_ballangs(agc_interp_t *ip, const uint16_t gimbal[3], uint16_t out[3])
{
    const uint16_t yzx[3] = {gimbal[1], gimbal[2], gimbal[0]}; /* CDUSPOT */
    agc_cdutrig_t  t;                                          /* sin/cos of the CDUs */

    agc_guid_cdtrigs(&t, yzx);                /* CD*TR*G */
    const agc_dp_t sx = t.sincdu[AGC_TRIG_X]; /* SINCDUX / 2, 2^0   */
    const agc_dp_t cx = t.coscdu[AGC_TRIG_X]; /* COSCDUX / 2, 2^0   */
    const agc_dp_t sy = t.sincdu[AGC_TRIG_Y]; /* SINCDUY / 2, 2^0   */
    const agc_dp_t cy = t.coscdu[AGC_TRIG_Y]; /* COSCDUY / 2, 2^0   */
    const agc_dp_t sz = t.sincdu[AGC_TRIG_Z]; /* SINCDUZ / 2, 2^0   */
    const agc_dp_t cz = t.coscdu[AGC_TRIG_Z]; /* COSCDUZ / 2, 2^0   */

    /* Every product is DMP; DMP SLn shifts the TP product before the
     * store truncates it (dmp_shl).                                     */
    agc_dp_t yaw   = agc_dp_asin(ip, agc_dp_neg(dmp_shl(ip, sx, cz, 1))); /* YAW, rev 2^0; SL1 DCOMP */
    agc_dp_t roll  = agc_latlong_arctan(ip, sz, dmp_shl(ip, cz, cx, 1));  /* ROLL, rev 2^0 */
    agc_dp_t sxsz  = dmp_shl(ip, sz, sx, 2);                              /* SL2 PUSH, 2^0 */
    agc_dp_t pd4   = agc_dp_mul_trunc(sxsz, cy);                          /* SXSZCY, 2^0   */
    agc_dp_t pd6   = agc_dp_mul_trunc(sxsz, sy);                          /* SXSZSY, 2^0   */
    agc_dp_t costh = agc_dp_sub(ip, dmp_shl(ip, cx, cy, 1), pd6);         /* DSU: cos/4, 2^0 */
    agc_dp_t sinth = agc_dp_add(ip, dmp_shl(ip, sy, cx, 1), pd4);         /* DAD: sin/4, 2^0 */
    agc_dp_t pitch = agc_latlong_arctan(ip, sinth, costh);                /* PITCH, rev 2^0 */

    agc_rtb_v1sto2s(agc_vec_make(roll, pitch, yaw), out); /* FDAIX..  */

    for (int i = 0; i < 3; i++)
    {
        fdai[i] = out[i];
    }
}

/**********************************************************************
 * ISITAUTO (Luminary R60_62.agc p.348) -- channel 31 bit 14
 *
 * RETURNS: agc_sp_t -- positive = not in auto, +0 = auto
 **********************************************************************/
agc_sp_t agc_imu_isitauto(void)
{
    uint16_t v = agc_chan_read(AGC_CHAN_CHAN31) & 020000u; /* channel bit */

    return ((agc_sp_t) v);
}

/**********************************************************************
 * G+N,AUTO (Luminary R60_62.agc p.348) -- channel 30 bit 10, then
 * ISITAUTO
 *
 * RETURNS: agc_sp_t -- positive when not in G+N or not auto, +0 = G+N
 *          auto
 **********************************************************************/
agc_sp_t agc_imu_gn_auto(void)
{
    uint16_t v = agc_chan_read(AGC_CHAN_CHAN30) & 01000u; /* channel bit */

    return ((v != 0u) ? (agc_sp_t) (v - 1u) : agc_imu_isitauto()); /* CCS A */
}

/* ======================================================================= */
/* R60 (pp.390-392 CM, pp.342-346 LM)                                      */
/* ======================================================================= */

/**********************************************************************
 * REDOMANN / REDOMANC (R60_62.agc p.390 CM) -- VECPOINT into CPHI,
 * CTHETA, CPSI
 *
 * Skipped when the 3-axis flag says CPHI.. were loaded directly.
 *
 * RETURNS: agc_status_t -- OK or BAD_ARG
 **********************************************************************/
static agc_status_t final_angles(void)
{
    if (agc_flag_test(AGC_FLAG_3AXISFLG))
    {
        return (AGC_STATUS_OK);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    uint16_t     g[3];                                            /* CPHI, CTHETA, CPSI */
    agc_status_t st = agc_imu_vecpoint(&ip, pointvsm, scaxis, g); /* status of the call */

    for (int i = 0; i < 3 && st == AGC_STATUS_OK; i++)
    {
        agc_erase_set_thetad(i, g[i]); /* STORE CPHI     */
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * TOBALL / TOBALLC (Luminary R60_62.agc p.343) -- BALLANGS of CPHI..
 * for noun 18
 **********************************************************************/
static void lm_ballangs(void)
{
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_interp_t ip; /* interpreter state */

        agc_interp_init(&ip);
        const uint16_t g[3] = {agc_erase_thetad(0), agc_erase_thetad(1), agc_erase_thetad(2)};
        uint16_t       out[3]; /* FDAIX..Z */

        agc_imu_ballangs(&ip, g, out);
    }
}

/**********************************************************************
 * CHKLINUS (R60_62.agc p.391 CM) -- make a priority display, restart to
 * RELINUS
 *
 * Only with the priority display flag; restart CM 1.7, LM 2.13.
 *
 * ARGUMENTS:
 *   is_static  bool  I    the display was the static V06N18 (TBASE1)
 *
 * RETURNS: agc_status_t -- OK, or the BLANKET status
 **********************************************************************/
static agc_status_t chklinus(bool is_static)
{
    relinus_static = is_static;

    if (!agc_flag_test(AGC_FLAG_PDSPFLAG))
    {
        return (AGC_STATUS_OK); /* TC Q          */
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_phase_change(00132);
    }
    else
    {
        agc_phase_change(00071);
    }

    return (agc_dsky_blanket(BIT7)); /* LINUS         */
}

/**********************************************************************
 * TOBALL (R60_62.agc p.390 CM) / TOBALLA (LM) -- flash V06N18
 *
 * "Please perform auto maneuver"; V34 -> R61TEST, PROCEED -> REDOMANC,
 * ENTER -> ENDMANU1.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    the R60 job
 *
 * RETURNS: agc_status_t -- OK (ENDOFJOB) or BAD_ARG
 **********************************************************************/
static agc_status_t toballa(const agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t cont = {AGC_JOB_ATTITUDE_MANEUVER_TERMINATE,
                                  AGC_JOB_REDOMANC,
                                  AGC_JOB_ATTITUDE_MANEUVER_ENTER};
    agc_status_t          st   = agc_dsky_display(ctx, AGC_DSKY_GOPERF2R, agc_dsky_vn(6, 18), &cont); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (chklinus(false)); /* ENDOFJOB      */
}

/**********************************************************************
 * TOBALLC +7 / STARTMNV (R60_62.agc p.391 CM) -- static V06N18, then
 * GOMANUR
 *
 * ENDMANUV redisplays the request when the maneuver is done.
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    the R60 job
 *
 * RETURNS: agc_status_t -- status of GOMANUR
 **********************************************************************/
static agc_status_t automanv(const agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_dsky_display(ctx, AGC_DSKY_GODSPR, agc_dsky_vn(6, 18), NULL); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    st = chklinus(true);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_imu_gomanur(ctx, AGC_JOB_ENDMANUV));
}

/**********************************************************************
 * TOBALLC (R60_62.agc p.390 CM) / G+N,AUTO (LM) -- auto-mode test
 *
 * CM: channel 31 bits 13-15 = 1, 1, 0 as read ("mode auto and control
 * GNC"); LM: G+N,AUTO.
 *
 * RETURNS: bool -- true when in auto
 **********************************************************************/
static bool in_auto(void)
{
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return (agc_imu_gn_auto() == 0);
    }

    uint16_t v = (uint16_t) ((agc_chan_read(AGC_CHAN_CHAN31) ^ 030000u) & 070000u); /* channel bit */

    return (v == 0u); /* BZF +2        */
}

/**********************************************************************
 * R60CSM (R60_62.agc p.390) / R60LEM -- attitude maneuver routine
 *
 * To the VECPOINT (or loaded CPHI..) attitude.
 *
 * ARGUMENTS:
 *   ctx      const agc_job_ctx_t *  I    the calling job
 *   ret_job  agc_job_id_t           I    job to continue the caller
 *                                        (TEMPR60)
 *
 * RETURNS: agc_status_t -- status of the display
 **********************************************************************/
agc_status_t agc_imu_attitude_maneuver(const agc_job_ctx_t *ctx, agc_job_id_t ret_job)
{
    if (!AGC_ASSERT(ctx != NULL && ret_job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    attitude_maneuver_return = ret_job;        /* MAKECADR      */
    agc_status_t st          = final_angles(); /* REDOMANN      */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    lm_ballangs(); /* TOBALL (LM)   */

    return (toballa(ctx));
}

/**********************************************************************
 * REDOMANC (R60_62.agc p.390 CM) -- PROCEED: recompute, maneuver if in
 * auto
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the R60 job
 *
 * RETURNS: agc_status_t -- status of the display or GOMANUR
 **********************************************************************/
agc_status_t agc_job_redomanc(agc_job_ctx_t *ctx)
{
    agc_status_t st = final_angles(); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    lm_ballangs(); /* TOBALLC (LM)  */

    if (!in_auto())
    {
        return (toballa(ctx)); /* not auto      */
    }

    return (automanv(ctx));
}

/**********************************************************************
 * ENDMANUV (R60_62.agc p.391 CM) -- maneuver finished, ask again
 * (TOBALL)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the R60 job
 *
 * RETURNS: agc_status_t -- status of the display
 **********************************************************************/
agc_status_t agc_job_endmanuv(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(!agc_kalc_active()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (toballa(ctx));
}

/**********************************************************************
 * ENDMANU1 (R60_62.agc p.391 CM) -- reset the 3-axis flag, return to
 * the user
 *
 * ARGUMENTS:
 *   ctx  const agc_job_ctx_t *  I    the R60 job
 *
 * RETURNS: agc_status_t -- status of the user's continuation
 **********************************************************************/
static agc_status_t maneuver_return(const agc_job_ctx_t *ctx)
{
    agc_flag_clear(AGC_FLAG_3AXISFLG);

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_3AXISFLG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (attitude_maneuver_return == AGC_JOB_NONE)
    {
        return (AGC_STATUS_OK);
    }

    agc_job_ctx_t next = *ctx; /* same job      */

    next.id = attitude_maneuver_return;

    return (agc_dispatch_job(&next));
}

/**********************************************************************
 * TOBALL ENTER (R60_62.agc p.390 CM) -- ENDMANU1
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the R60 job
 *
 * RETURNS: agc_status_t -- status of ENDMANU1
 **********************************************************************/
agc_status_t agc_job_attitude_maneuver_enter(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (maneuver_return(ctx)); /* ENTER         */
}

/**********************************************************************
 * R61TEST (R60_62.agc p.392 CM) -- V34 on V06N18
 *
 * In P00 (V49/V89) return to the user, who does ENDEXT; in R61 (priority
 * display flag) terminate P20 by GOTOV56; else GOTOPOOH.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the R60 job
 *
 * RETURNS: agc_status_t -- status of the exit taken
 **********************************************************************/
agc_status_t agc_job_attitude_maneuver_terminate(agc_job_ctx_t *ctx)
{
    if (agc_erase_modreg() == 0)
    {
        return (maneuver_return(ctx));
    }

    if (!agc_flag_test(AGC_FLAG_PDSPFLAG))
    {
        return (agc_exec_gotopooh());
    }

    return (agc_rendz_goto_terminate_tracking());
}

/**********************************************************************
 * RELINUS / GOREDO20 (R60_62.agc p.391 CM) -- restart of a priority R60
 * display
 *
 * CM continues only with TRACKFLG (PDSPFLAG, TARG1FLG, OPTIND 0, PRIO14)
 * else restart 1.11; LM PRIO26, TRACKFLG -> PDSPFLAG, else 2.7 or 2.11.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the restarted job
 *
 * RETURNS: agc_status_t -- status of the redisplay, or OK (ENDOFJOB)
 **********************************************************************/
agc_status_t agc_job_relinus(agc_job_ctx_t *ctx)
{
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM); /* vehicle is the LM */

    if (lm)
    {
        agc_exec_prio_change(PRIO26);
    }

    if (!agc_flag_test(AGC_FLAG_TRACKFLG))
    {
        if (!lm)
        {
            agc_phase_change(00111); /* PIKUP20 */
        }
        else if (agc_flag_test(AGC_FLAG_RNDVZFLG))
        {
            agc_phase_change(040072);
        }
        else
        {
            agc_phase_change(040112);
        }

        return (AGC_STATUS_OK); /* ENDOFJOB      */
    }

    agc_flag_set(AGC_FLAG_PDSPFLAG);

    if (!lm)
    {
        imu_vflag_set(IMU_VF_TARG1FLG); /* for R52       */
        agc_erase_set_optind(0);        /* OPTIND was set to -1 by V37    */
        agc_exec_prio_change(PRIO14);
    }

    return (relinus_static ? automanv(ctx) : toballa(ctx)); /* TC TBASE    */
} /* agc_job_relinus */

/* ======================================================================= */
/* R62 (CM p.398, LM p.363)                                                */
/* ======================================================================= */

/**********************************************************************
 * R62DISP (R60_62.agc p.398 CM, V49) -- flash V06N22
 *
 * CPHI, CTHETA, CPSI; the crew may load new angles (ENTER redisplays);
 * PROCEED -> GOMOVE; V34 -> ENDEXT.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the V49 job
 *
 * RETURNS: agc_status_t -- status of GOFLASH
 **********************************************************************/
agc_status_t agc_job_crew_defined_maneuver_display(agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t cont = {AGC_JOB_CREW_DEFINED_MANEUVER_TERMINATE,
                                  AGC_JOB_CREW_DEFINED_MANEUVER_GOMOVE,
                                  AGC_JOB_CREW_DEFINED_MANEUVER_DISPLAY};
    agc_status_t          st   = agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, 22), &cont); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * GOMOVE (R60_62.agc p.398 CM) -- 3-axis flag, then R60
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the V49 job
 *
 * RETURNS: agc_status_t -- status of R60
 **********************************************************************/
agc_status_t agc_job_crew_defined_maneuver_gomove(agc_job_ctx_t *ctx)
{
    agc_flag_set(AGC_FLAG_3AXISFLG);

    return (agc_imu_attitude_maneuver(ctx, AGC_JOB_CREW_DEFINED_MANEUVER_TERMINATE));
}

/**********************************************************************
 * R62 end (R60_62.agc p.398 CM) -- ENDEXT
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the V49 job
 *
 * RETURNS: agc_status_t -- status of ENDEXT
 **********************************************************************/
agc_status_t agc_job_crew_defined_maneuver_terminate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_endmark()); /* ENDEXT        */
}
