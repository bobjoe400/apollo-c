/**********************************************************************
 * FILE NAME: burn_steering.c
 *
 * PURPOSE:
 *   The steering mathematics of P40-P47.agc shared by both vehicles:
 *   S40.1 (initial thrust direction UT and VGTIG: CALCTHET for external
 *   delta-V, S40.1B for the Lambert aimpoint), CALCUT, S40.2,3 (preferred
 *   attitude), S41.1 (VG to control/body axes), the S40.8 time-to-go
 *   computations (TGOCALC, TGDCALC), the Comanche cross-product steering
 *   XPRODUCT and the Comanche S40.13 TIMEBURN. The flag-dependent
 *   sequencing is in src/csm/burns/sps/ and src/lm/burns/dps_burn.c.
 *
 * AGC SOURCE:
 *   Comanche055 P40-P47.agc pp. 709-728 (S40.1, CALCTHET, S40.1B,
 *     CALCUT, S40.2,3, TSTRXUT, S41.1, S40.8, S40.13)
 *   Luminary099 P40-P47.agc pp. 764-784 (S40.1, S40.1B, S40.2,3,
 *     S41.1, TGDCALC; differs: LEMPREC/INITVEL, no SPS bell axes)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale        Description
 *   ------------  ---------  ---  -----------  -----------------------
 *   VGTIG         agc_vec_t  I/O  m/cs 2^7     velocity to be gained
 *   TIG           agc_dp_t   I    cs 2^28      time of ignition
 *   REFSMMAT      agc_mat_t  I    2^1          reference to SM matrix
 *   TGO           agc_dp_t   O    cs 2^28      S40.13 burn time
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_guid_again        AGAIN: Lambert VGTIG at a time (targeting)
 *   agc_rendz_initvel     INITVEL (Luminary S40.1B)
 *   agc_integ_lemprec     LEMPREC (Luminary S40.1B)
 *   agc_imu_set_scaxis    SCAXIS / POINTVSM for R60 (imu)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   AGC_STATUS_ALARM from the S40.8 TGO routines when the thrust is more
 *   than 90 degrees from VG (the caller raises 01407); AGC_STATUS_BAD_ARG
 *   on a failed precondition; AGC_STATUS_NOT_FOUND when LEMPREC has no
 *   state; other statuses passed up from the targeting calls.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the burn program jobs; no loops beyond those of the callees.
 *
 * NOTES:
 *   Interpretive DMP truncates (agc_dp_mul_trunc) and leaves the third
 *   product word in MPAC+2, which a following SL, NORM or SR shifts into
 *   the result (dmp_sl1, TP paths); DMPR rounds (agc_dp_mul). See
 *   docs/modules/review-guid-powered.md R2, R4, R7.
 *
 * ALGORITHM:
 *   As in the listing. Scales: UT half unit (2^1); VGTIG, VG, DELVREF,
 *   BDT, DELVSIN m/cs 2^7; RTIG m 2^29; F M.newton 2^7; WEIGHT/G kg
 *   2^16; TGO cs 2^28; angles rev 2^0.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/rendezvous.h"

/* THETACON 2DEC .31830989 B-8 (1/pi).                                     */
static const agc_dp_t THETACON = 333772; /* 1/pi, scale 2^-8 */

/* TWODT 2DEC 200.0 B-28: 2 s.                                             */
static const agc_dp_t TWODT = 200; /* 2 s, cs 2^28 */

/* 200CS 2DEC 200 B-12.                                                    */
static const agc_dp_t CS200_B12 = 13107200; /* 200 cs, scale 2^12 */

/* EPS1 (10 deg) and EPS2 (35 deg) at 1 revolution (LM S40.1B).            */
static const agc_dp_t EPS1 = 7456540;  /* 10 deg, rev 2^0 */
static const agc_dp_t EPS2 = 26097892; /* 35 deg, rev 2^0 */

/* TRIMSCAL 2DEC 1.07975111 B-1: 85.41"/bit trims to rev; YBIAS, PBIAS:
 * yaw +0.95 deg and pitch -2.15 deg mechanical bias, thrust on.          */
static const agc_dp_t TRIMSCAL = 144921741; /* scale 2^1 */
static const agc_dp_t YBIAS    = 708371;    /* +0.95 deg, rev 2^0 */
static const agc_dp_t PBIAS    = -1603156;  /* -2.15 deg, rev 2^0 */

/* TENBNK14 2DEC 10. B-4; QUADROT: S/C to control axes, X rotation -7.25
 * degrees, elements times 0.1 (GIMBAL_LOCK_AVOIDANCE p.412-413).          */
static const agc_dp_t TENBNK14 = 167772160; /* 10, scale 2^4 */
static const agc_dp_t Q_P1     = 26843546;  /* .1, scale 2^0              */
static const agc_dp_t Q_COS    = 26628797;  /* (.1) COS 7.25, scale 2^0   */
static const agc_dp_t Q_SIN    = 3387655;   /* (.1) SIN 7.25, scale 2^0   */

/* -FOURDT 2DEC -800 B-18: -4 (200 cs), -4 for scaling.                    */
static const agc_dp_t MFOURDT = -819200; /* -800 cs, scale 2^18 */

/* S40.13 (CM): S40.135 2DEC 69.6005183 B-23 (7.96 s of 2-jet ullage),
 * 1SEC2D 100 B-14, 3.5SEC 350 B-13, 5SECOND 500 B-14.                     */
static const agc_dp_t TWO_JET_IMPULSE = 2227;     /* M.newton cs, scale 2^23 */
static const agc_dp_t SEC1_2D         = 1638400;  /* 100 cs, scale 2^14 */
static const agc_dp_t SEC3_5          = 11468800; /* 350 cs, scale 2^13 */
static const agc_dp_t SEC5_14         = 8192000;  /* 500 cs, scale 2^14 */

/**********************************************************************
 * GET.LVC / MIDGIM for the VGTIG display: DELVLVC (LV axes of RINIT,
 * VINIT). Comanche: SET AVFLAG, MIDGIM (P30-P37.agc); Luminary: GET.LVC
 * (P30_P37.agc). AGC: none (dispatch of the two).
 *
 * FUNCTION NAME: vg_to_lvc
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   veh       agc_vehicle_t   I    vehicle: AGC_VEHICLE_CSM or
 *                                  AGC_VEHICLE_LM
 *   vg        agc_vec_t       I    velocity to be gained, reference frame,
 *                                  m/cs 2^7
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
static agc_status_t vg_to_lvc(agc_interp_t *ip, agc_vehicle_t veh, agc_vec_t vg)
{
    agc_rendz_t *rz = agc_rendz_erasable(); /* RINIT VINIT */

    if (veh == AGC_VEHICLE_CSM)
    {
        agc_flag_set(AGC_FLAG_AVFLAG);

        return (agc_guid_midgim(ip, vg));
    }

    if (!AGC_ASSERT(rz != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_rendz_get_lvc(ip, rz->rinit, rz->vinit, vg, &rz->delvlvc));
}

/**********************************************************************
 * CALCTHET (CM p.709-710, LM p.765-766): the impulsive DELVSIN is spread
 * over the burn arc. theta/2 = (|RxV| / R^2) (1/pi) |DV| m / F; the
 * in-plane part DVP is rotated by theta/2 about UP:
 *   VGTIG = |DVP| (cos(theta/2) UNIT(DVP) + sin(theta/2) UNIT(DVP x UP))
 *           VSL2 + (DV . UP) UP;   UT = UNIT(VGTIG).
 *
 * FUNCTION NAME: calcthet
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   veh       agc_vehicle_t   I    vehicle: AGC_VEHICLE_CSM or
 *                                  AGC_VEHICLE_LM
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
static agc_status_t calcthet(agc_interp_t *ip, agc_vehicle_t veh)
{
    agc_rendz_t *rz = agc_rendz_erasable();    /* targeting */
    agc_burn_t  *b  = &agc_guid_state()->burn; /* UT, F, WEIGHT/G */

    rz->vinit  = rz->vtig;
    agc_dp_t h = 0;                                                                 /* |V x R| (36D), m^2/cs 2^36 */
    b->ut      = agc_vec_unit(ip, agc_vec_cross(ip, rz->vtig, rz->rtig), &h, NULL); /* UP */

    rz->rinit    = rz->rtig;
    agc_dp_t rsq = agc_vec_sq(ip, rz->rtig);                         /* VSQ PDDL 36D, m^2 2^58 */
    agc_dp_t th  = agc_dp_div(ip, agc_dp_mul(ip, h, THETACON), rsq); /* theta/2, rev 2^0 */
    th           = agc_dp_mul_trunc(agc_dp_mul_trunc(th, rz->delvsab), b->weight_g);
    th           = agc_dp_div(ip, th, b->f);                         /* 14D         */

    agc_vec_t dvup   = agc_vec_shl(ip, agc_vec_scale(ip, b->ut, agc_vec_dot(ip, rz->delvsin, b->ut)), 2);  /* PDL 0 (DV.UP) UP, m/cs 2^7 */
    agc_vec_t dvp    = agc_vec_sub(ip, rz->delvsin, dvup);                                                 /* PDL 6, m/cs 2^7 */
    agc_dp_t  sn     = agc_dp_sin(ip, th);                                                                 /* PDL 12, scale 2^1 */
    agc_vec_t vg     = agc_vec_scale(ip, agc_vec_unit(ip, agc_vec_cross(ip, dvp, b->ut), NULL, NULL), sn); /* VGTIG, m/cs 2^7 */
    agc_dp_t  dvpmag = 0;                                                                                  /* |DVP|, m/cs 2^7 */
    agc_vec_t udvp   = agc_vec_unit(ip, dvp, &dvpmag, NULL);                                               /* PDL 6, scale 2^1 */

    vg = agc_vec_add(ip, agc_vec_scale(ip, udvp, agc_dp_cos(ip, th)), vg);
    vg = agc_vec_add(ip, agc_vec_shl(ip, agc_vec_scale(ip, vg, dvpmag), 2), dvup);

    agc_erase_set_vgtig(vg);                  /* VG IGNITION */
    b->ut = agc_vec_unit(ip, vg, NULL, NULL); /* THRUST DIR. */

    if (!AGC_ASSERT(b->f > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (vg_to_lvc(ip, veh, vg));
}

/**********************************************************************
 * CALCUT (Comanche p.712-713): thrust direction for aimpoint steering.
 *   b  = (VIPRIME - VIPRIME(TIG-2)) / 2 s - g;   bc = CSTEER b
 *   q  = bc - (ug . bc) ug,   ug = UNIT(VGTIG)
 *   UT = UNIT(q + ug sqrt((F/m)^2 - q^2))
 *
 * FUNCTION NAME: agc_guid_calcut
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed); otherwise the status of the routine it
 *          continues in
 **********************************************************************/
agc_status_t agc_guid_calcut(agc_interp_t *ip)
{
    agc_guid_t  *g  = agc_guid_state();                      /* avg, burn */
    agc_rendz_t *rz = agc_rendz_erasable();                  /* RTIG VIPRIME */
    agc_status_t st = agc_guid_calcgrav(ip, AGC_VEHICLE_CSM, /* CALCGRAV at RTIG: GDT1/2 */
                                        rz->body.rtx2 != 0,
                                        rz->rtig,
                                        &g->avg);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_vec_t gv  = agc_vec_div(ip, agc_vec_shl(ip, g->avg.gdt1_2, 1), CS200_B12);                           /* g, m/cs^2 2^-5 */
    agc_vec_t dvr = agc_vec_div(ip, agc_vec_sub(ip, rz->viprime, g->burn.ut), CS200_B12);                    /* dV/dt, m/cs^2 2^-5 */
    agc_vec_t bc  = agc_vec_shl(ip, agc_vec_scale(ip, agc_vec_sub(ip, dvr, gv), g->burn.csteer), 2);         /* 12D B.C, scale 2^-15 */
    agc_vec_t ug  = agc_vec_unit(ip, agc_erase_vgtig(), NULL, NULL);                                         /* UG PDL 0, 2^1 */
    agc_vec_t q   = agc_vec_sub(ip, bc, agc_vec_shl(ip, agc_vec_scale(ip, ug, agc_vec_dot(ip, ug, bc)), 2)); /* Q, scale 2^-5 */
    agc_dp_t  fm  = agc_dp_div(ip, agc_dp_shr_round(g->burn.f, 4), g->burn.weight_g);                        /* F/m, scale 2^-5 */
    agc_dp_t  rad = agc_dp_sub(ip, agc_dp_sq(ip, fm), agc_vec_sq(ip, q));                                    /* (F/m)^2 - Q^2, 2^-10 */
    agc_vec_t t   = agc_vec_shl(ip, agc_vec_scale(ip, ug, agc_dp_sqrt(ip, rad)), 1);                         /* UG sqrt(), 2^-5 */

    g->burn.ut = agc_vec_unit(ip, agc_vec_add(ip, t, q), NULL, NULL);

    if (!AGC_ASSERT(rad >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* agc_guid_calcut */

/**********************************************************************
 * S40.1B Comanche (p.710-711): VIPRIME at TIG - 2 s (kept in UT), then
 * the Lambert solution at TIG: VGTIG = DELVEET3, DELVLVC, CALCUT.
 *
 * FUNCTION NAME: initial_vg_lambert_csm
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; otherwise the status of the
 *          routine it continues in
 **********************************************************************/
static agc_status_t initial_vg_lambert_csm(agc_interp_t *ip)
{
    agc_rendz_t *rz    = agc_rendz_erasable();                   /* TPASS4 */
    agc_dp_t     tdec1 = agc_dp_sub(ip, agc_erase_tig(), TWODT); /* cs 2^28 */

    rz->dellt4      = agc_dp_sub(ip, rz->tpass4, tdec1);
    agc_status_t st = agc_guid_again(ip, tdec1); /* AGAIN */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_guid_state()->burn.ut = rz->viprime; /* STODL UT    */
    rz->dellt4                = agc_dp_sub(ip, rz->tpass4, agc_erase_tig());
    st                        = agc_guid_again(ip, agc_erase_tig());

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_vgtig(rz->delveet3);
    st = vg_to_lvc(ip, AGC_VEHICLE_CSM, rz->delveet3);

    return ((st != AGC_STATUS_OK) ? st : agc_guid_calcut(ip)); /* MIDGIM failed */
}

/**********************************************************************
 * S40.1B Luminary (p.766-767): LEMPREC to TIG, INITVEL (NUMIT 0, EPSILON4
 * 10 degrees, or 45 with NORMSW), VGTIG = DELVEET3, UT, VGDISP, DELVLVC.
 *
 * FUNCTION NAME: initial_vg_lambert_lm
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- the status of the routine it continues in
 **********************************************************************/
static agc_status_t initial_vg_lambert_lm(agc_interp_t *ip)
{
    agc_rendz_t *rz = agc_rendz_erasable(); /* targeting */
    agc_guid_t  *g  = agc_guid_state();     /* avg, burn */

    rz->dellt4          = agc_dp_sub(ip, rz->tpass4, agc_erase_tig());                       /* TDEC1 = TIG */
    agc_integ_ctx_t *ic = agc_integ_shared_ctx();                                            /* LEMPREC */
    agc_status_t     st = agc_integ_lemprec(ip, ic, agc_erase_tig(), agc_guid_nav_resume()); /* LEMPREC */

    if (st == AGC_STATUS_OK && !ic->out.valid)
    {
        st = AGC_STATUS_NOT_FOUND; /* no RECTOUT */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_integ_out_t *p = &ic->out; /* RATT VATT */

    rz->rtig      = p->ratt;
    rz->rinit     = p->ratt;
    g->avg.unitr  = agc_vec_unit(ip, p->ratt, NULL, NULL);                              /* UNIT/R/     */
    rz->vtig      = p->vatt;
    rz->vinit     = p->vatt;
    rz->body.rtx1 = (agc_sp_t) p->x1;
    rz->body.rtx2 = (agc_sp_t) p->x2;
    agc_dp_t e4   = agc_flag_test(AGC_FLAG_NORMSW) ? agc_dp_add(ip, EPS1, EPS2) : EPS1; /* EPSILON4, rev 2^0; 45; 10 */

    st = agc_rendz_initvel(ip, rz, 0, e4, agc_guid_nav_resume());

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_erase_set_vgtig(rz->delveet3); /* VR - VN     */
    g->burn.ut = agc_vec_unit(ip, rz->delveet3, &g->burn.vgdisp, NULL);

    return (vg_to_lvc(ip, AGC_VEHICLE_LM, rz->delveet3));
} /* initial_vg_lambert_lm */

/**********************************************************************
 * S40.1 (CM p.709, LM p.764): initial thrust direction UT and VGTIG.
 * Comanche: BDT = 0 here; the caller sets FIRSTFLG (a Comanche-only flag).
 * Luminary: TIGSAVE = TIG.
 *
 * FUNCTION NAME: agc_guid_initial_velocity_to_gain
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   veh       agc_vehicle_t   I    vehicle: AGC_VEHICLE_CSM or
 *                                  AGC_VEHICLE_LM
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_guid_initial_velocity_to_gain(agc_interp_t *ip, agc_vehicle_t veh)
{
    agc_guid_t  *g  = agc_guid_state();     /* BDT, TIGSAVE */
    agc_rendz_t *rz = agc_rendz_erasable(); /* DELVSIN */

    if (veh == AGC_VEHICLE_CSM)
    {
        g->burn.bdt = agc_vec_zero(); /* LO6ZEROS    */
    }
    else
    {
        g->lm.tigsave = agc_erase_tig();
    }

    if (!agc_flag_test(AGC_FLAG_XDELVFLG))
    {
        return ((veh == AGC_VEHICLE_CSM) ? initial_vg_lambert_csm(ip) : initial_vg_lambert_lm(ip)); /* CM; LM */
    }

    if (veh == AGC_VEHICLE_CSM)
    {
        rz->delvsab = agc_vec_abval(ip, rz->delvsin); /* P30/P40 I/F */
    }

    return (calcthet(ip, veh));
}

/**********************************************************************
 * DMP then SL1 on the TP MPAC: the third word of the product shifts into
 * the DP result (AGC DMP leaves MPAC+2; SL shifts all three words).
 * AGC: none (helper for the DMP SL1 pairs of S40.2,3, p.715)
 *
 * FUNCTION NAME: dmp_sl1
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   a         agc_dp_t        I    multiplicand, DP fraction
 *   b         agc_dp_t        I    multiplier, DP fraction
 *
 * RETURNS: agc_dp_t -- the top two words of 2 a b (0 on overflow)
 **********************************************************************/
static agc_dp_t dmp_sl1(agc_interp_t *ip, agc_dp_t a, agc_dp_t b)
{
    agc_dp_t r = agc_tp_to_dp(agc_tp_shift(ip, agc_dp_mul_tp(a, b), 1)); /* 2 a b, scale of a b */

    if (!AGC_ASSERT(r == 0 || ((r < 0) == ((a < 0) != (b < 0)))))
    {
        return (0); /* SL1 overflowed: callers keep |a b| < 1/2      */
    }

    return (r);
}

/**********************************************************************
 * TSTRXUT / BADVCTOR (Comanche p.716): if UT x R was null (36D = 0) use
 * Y = -UNIT((UNIT(R) + UNIT(V)/8) x UT) instead.
 *
 * FUNCTION NAME: tstrxut
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   y         agc_vec_t       I    UNIT(UT x R) as computed, half unit
 *   mag       agc_dp_t        I    |UT x R| left by UNIT (36D); 0 = null
 *                                  vector
 *
 * RETURNS: agc_vec_t -- the Y axis to use, scale 2^1
 **********************************************************************/
static agc_vec_t tstrxut(agc_interp_t *ip, agc_vec_t y, agc_dp_t mag)
{
    agc_rendz_t *rz = agc_rendz_erasable(); /* RTIG, VTIG */

    if (mag != 0)
    {
        return (y);
    }

    agc_vec_t s = agc_vec_add(ip, agc_vec_shr(agc_vec_unit(ip, rz->vtig, NULL, NULL), 3), agc_vec_unit(ip, rz->rtig, NULL, NULL)); /* UNIT(R) + UNIT(V)/8, scale 2^1 */

    if (!AGC_ASSERT(mag == 0))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (agc_vec_neg(agc_vec_unit(ip, agc_vec_cross(ip, s, agc_guid_state()->burn.ut), NULL, NULL)));
}

/**********************************************************************
 * The SPS bell axes of S40.2,3 (Comanche p.715): pitch p = PACTOFF
 * TRIMSCAL SL1 + PBIAS, yaw y = YACTOFF TRIMSCAL SL1 + YBIAS (rev). VDEF
 * takes X from MPAC and Y, Z from the push-down list:
 *   XSC = (cp cy, -cp sy, sp)  ZSC = (-sp cy, sp sy, cp)  YSC = (sy, cy, 0)
 * (all halves, products DMP SL1); SCAXIS = (XSC.x, YSC.x, ZSC.x). The
 * axes are left in XSCREF..ZSCREF, still in spacecraft coordinates.
 *
 * FUNCTION NAME: sps_bell_axes
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND)
 *   pactoff   agc_sp_t        I    PACTOFF pitch trim, 85.41 arcsec/bit
 *   yactoff   agc_sp_t        I    YACTOFF yaw trim, 85.41 arcsec/bit
 *   b         agc_burn_t *    O    SCAXIS, XSCREF, YSCREF, ZSCREF
 *                                  (spacecraft axes, scale 2^1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (pitch
 *          beyond 90 degrees: cannot happen for 8-bit trims)
 **********************************************************************/
static agc_status_t sps_bell_axes(agc_interp_t *ip, agc_sp_t pactoff, agc_sp_t yactoff, agc_burn_t *b)
{
    if (!AGC_ASSERT(ip != NULL && b != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t yaw   = agc_dp_add(ip, dmp_sl1(ip, agc_dp_from_sp(yactoff), TRIMSCAL), YBIAS); /* yaw bell angle, rev 2^0 */
    agc_dp_t pitch = agc_dp_add(ip, dmp_sl1(ip, agc_dp_from_sp(pactoff), TRIMSCAL), PBIAS); /* pitch bell angle, rev 2^0 */
    agc_dp_t cy    = agc_dp_cos(ip, yaw);                                                   /* COS(Y+Y0) PDL 2, scale 2^1  */
    agc_dp_t sy    = agc_dp_sin(ip, yaw);                                                   /* SIN(Y+Y0) PDL 4, scale 2^1  */
    agc_dp_t cp    = agc_dp_cos(ip, pitch);                                                 /* COS(P+P0) PDL 6, scale 2^1  */
    agc_dp_t sp    = agc_dp_sin(ip, pitch);                                                 /* SIN(P+P0) PDL 8D, scale 2^1 */

    if (!AGC_ASSERT(cp > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    b->xscref = agc_vec_make(dmp_sl1(ip, cp, cy), agc_dp_neg(dmp_sl1(ip, cp, sy)), sp);
    b->zscref = agc_vec_make(agc_dp_neg(dmp_sl1(ip, sp, cy)), dmp_sl1(ip, sp, sy), cp);
    b->yscref = agc_vec_make(sy, cy, 0);
    b->scaxis = agc_vec_make(b->xscref.c[0], b->yscref.c[0], b->zscref.c[0]);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * S40.2,3 Comanche (p.714-716): POINTVSM = REFSMMAT UT VSL1; for the SPS
 * the bell axes give SCAXIS and, rotated into the frame X = UT, Y = UNIT(
 * UT x UNIT(R)), Z = UT x Y, the preferred orientation XSCREF..ZSCREF;
 * for the RCS (ENG2FLAG, S40.2,3B) SCAXIS = UNITX and the frame itself,
 * with Y = UNIT(UT x R). The SPS path crosses with UNIT(RTIG) (PDVL UNIT
 * RTIG / VXV VCOMP, p.716), the RCS path with RTIG as stored.
 *
 * FUNCTION NAME: agc_guid_preferred_thrust_attitude_csm
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   rcs       bool            I    ENG2FLAG: RCS burn (S40.2,3B) instead
 *                                  of SPS
 *   pactoff   agc_sp_t        I    PACTOFF pitch trim, 85.41 arcsec/bit
 *   yactoff   agc_sp_t        I    YACTOFF yaw trim, 85.41 arcsec/bit
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed), or the status of agc_imu_set_scaxis
 **********************************************************************/
agc_status_t agc_guid_preferred_thrust_attitude_csm(agc_interp_t *ip, bool rcs, agc_sp_t pactoff, agc_sp_t yactoff)
{
    agc_burn_t  *b        = &agc_guid_state()->burn; /* burn erasables */
    agc_rendz_t *rz       = agc_rendz_erasable();    /* RTIG, VTIG     */
    agc_mat_t    refsmmat = agc_erase_refsmmat();    /* half-unit rows */

    b->pointvsm    = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, b->ut), 1);        /* SM AT 2 */
    agc_dp_t  mag  = 0;                                                            /* |UT x R| (36D), scale 2^1 */
    agc_vec_t rdir = rcs ? rz->rtig : agc_vec_unit(ip, rz->rtig, NULL, NULL);      /* RTIG (RCS) or UNIT(RTIG) (SPS), m 2^29 or 2^1; S40.2,3B: RTIG; SPS: UNIT */
    agc_vec_t y    = agc_vec_unit(ip, agc_vec_cross(ip, b->ut, rdir), &mag, NULL); /* preferred Y axis, scale 2^1 */

    y = tstrxut(ip, y, mag);

    if (rcs)
    { /* S40.2,3B    */
        b->scaxis = agc_vec_make(AGC_DP_HALF, 0, 0);
        b->xscref = b->ut;
        b->yscref = y;
        b->zscref = agc_vec_shl(ip, agc_vec_cross(ip, b->xscref, b->yscref), 1);
    }
    else
    {
        agc_mat_t    m;                                           /* 2 RF/SC: rows UT, Y, Z (PDL 0, 6, 12D), 2^1 */
        agc_status_t st = sps_bell_axes(ip, pactoff, yactoff, b); /* status */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        m.row[0]  = b->ut;
        m.row[1]  = y;
        m.row[2]  = agc_vec_shl(ip, agc_vec_cross(ip, b->ut, y), 1);
        b->xscref = agc_vec_shl(ip, agc_vec_mat(ip, b->xscref, &m), 1);
        b->yscref = agc_vec_shl(ip, agc_vec_mat(ip, b->yscref, &m), 1);
        b->zscref = agc_vec_shl(ip, agc_vec_mat(ip, b->zscref, &m), 1);
    }

    agc_imu_set_pointvsm(b->pointvsm); /* FOR R60     */

    return (agc_imu_set_scaxis(b->scaxis));
} /* agc_guid_preferred_thrust_attitude_csm */

/**********************************************************************
 * S40.2,3 Luminary (p.768-769): POINTVSM, SCAXIS = +X (DPS, APS and RCS
 * alike), XSCREF = UT, YSCREF = UNIT(UT x RTIG) (or UNIT(UT x VTIG) when
 * that is null: FIXY), ZSCREF = -(YSCREF x XSCREF) VSL1; PFRATFLG set.
 *
 * FUNCTION NAME: agc_guid_preferred_thrust_attitude_lm
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_BAD_ARG (a precondition failed);
 *          otherwise the status of the routine it continues in
 **********************************************************************/
agc_status_t agc_guid_preferred_thrust_attitude_lm(agc_interp_t *ip)
{
    agc_burn_t  *b        = &agc_guid_state()->burn; /* burn erasables */
    agc_rendz_t *rz       = agc_rendz_erasable();    /* RTIG VTIG */
    agc_mat_t    refsmmat = agc_erase_refsmmat();    /* 2^1 */

    b->pointvsm  = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, b->ut), 1);
    b->scaxis    = agc_vec_make(AGC_DP_HALF, 0, 0); /* UNITX       */
    b->xscref    = b->ut;                           /* PLUSX       */
    agc_dp_t mag = 0;                               /* |UT x R|, scale 2^1 */

    b->yscref = agc_vec_unit(ip, agc_vec_cross(ip, b->ut, rz->rtig), &mag, NULL);

    if (mag == 0)
    { /* FIXY        */
        b->yscref = agc_vec_unit(ip, agc_vec_cross(ip, b->xscref, rz->vtig), NULL, NULL);
    }

    b->zscref = agc_vec_neg(agc_vec_shl(ip, agc_vec_cross(ip, b->yscref, b->xscref), 1));
    agc_flag_set(AGC_FLAG_PFRATFLG);
    agc_imu_set_pointvsm(b->pointvsm);

    return (agc_imu_set_scaxis(b->scaxis));
}

/**********************************************************************
 * S41.1 (CM p.717-718, LM p.783-784): VG (reference, m/cs 2^7) to the
 * Comanche control axes (CDUTRIG, REFSMMAT, *SMNB*, QUADROT, x10, VSL5)
 * or the Luminary body axes (REFSMMAT VSL1, CDU*SMNB).
 *
 * FUNCTION NAME: agc_guid_reference_to_body
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   veh       agc_vehicle_t   I    vehicle: AGC_VEHICLE_CSM or
 *                                  AGC_VEHICLE_LM
 *   vg_ref    agc_vec_t       I    VG in reference coordinates, m/cs 2^7
 *   vg_ctl    agc_vec_t *     O    VG in control (body) coordinates, m/cs
 *                                  2^7
 **********************************************************************/
void agc_guid_reference_to_body(agc_interp_t *ip, agc_vehicle_t veh, agc_vec_t vg_ref, agc_vec_t *vg_ctl)
{
    agc_guid_t *g        = agc_guid_state();                   /* trig */
    agc_mat_t   refsmmat = agc_erase_refsmmat();               /* 2^1 */
    agc_vec_t   v        = agc_mat_vec(ip, &refsmmat, vg_ref); /* VG SM, m/cs 2^7 */

    if (veh == AGC_VEHICLE_LM)
    {
        v       = agc_vec_shl(ip, v, 1);
        agc_guid_cdu_smnb(&g->trig, &v); /* CDU*SMNB    */
        *vg_ctl = v;

        return;
    }

    agc_guid_cdutrig(&g->trig);
    agc_guid_smnb(&g->trig, &v);
    agc_mat_t quadrot = {
        {{{Q_P1, 0, 0}}, {{0, Q_COS, -Q_SIN}}, {{0, Q_SIN, Q_COS}}}
    };

    v       = agc_vec_scale(ip, agc_mat_vec(ip, &quadrot, v), TENBNK14);
    *vg_ctl = agc_vec_shl(ip, v, 5); /* VSL5        */
}

/**********************************************************************
 * TGOCALC (Comanche S40.8 p.720): with d = UNIT(BDT - DELVREF) . VG (must
 * be negative: thrust within 90 degrees of VG, else alarm 1407 by the
 * caller -- AGC_STATUS_ALARM),
 *   TGO = -4(200 cs) d (1 + d/2Vex) / |BDT - DELVREF| - ETDECAY,
 * normalised by NORM X1 / NORM X2 and SL* (n2 - n1 - 9).
 *
 * FUNCTION NAME: agc_guid_steering_tgo_csm
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  -----------------------------
 *   ip        agc_interp_t *        I/O  interpreter state (OVFIND, index
 *                                        registers)
 *   in        const agc_tgo_in_t *  I    S40.8 TGO inputs (DELVG, VG, VEX,
 *                                        tail-off)
 *   tgo       agc_dp_t *            O    TGO, time to engine-off command,
 *                                        cs 2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed), AGC_STATUS_ALARM (an alarm was raised)
 **********************************************************************/
agc_status_t agc_guid_steering_tgo_csm(agc_interp_t *ip, const agc_tgo_in_t *in, agc_dp_t *tgo)
{
    agc_dp_t  dmag = 0;                                        /* |DELVG|, m/cs 2^7 */
    agc_vec_t u    = agc_vec_unit(ip, in->delvg, &dmag, NULL); /* 36D, scale 2^1 */
    agc_dp_t  d    = agc_vec_dot(ip, u, in->vg);               /* 2^8 */

    if (d >= 0)
    {
        return (AGC_STATUS_ALARM); /* INCRSVG     */
    }

    agc_dp_t m = agc_dp_add(ip, agc_dp_div(ip, d, in->vex), AGC_DP_HALF); /* d/2VEX + 1/2, scale per p.720; LODPHALF */

    /* DMP leaves the TP product; NORM and SR1 act on all three words and
     * PDDL pushes the top two.                                          */
    int      n1 = 0;                                     /* NORM X1 shift count */
    int      n2 = 0;                                     /* NORM X2 shift count */
    agc_tp_t mt = agc_tp_norm(agc_dp_mul_tp(m, d), &n1); /* NORM X1, scale 2^-n1 */

    m            = agc_tp_to_dp(agc_tp_shift(ip, mt, -1)); /* SR1, PDDL  */
    agc_dp_t den = agc_dp_norm(dmag, &n2);                 /* NORM X2, scale 2^-n2 */

    if (!AGC_ASSERT(den != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t t = agc_dp_shl(ip, agc_dp_div(ip, m, den), n2 - n1 - 9); /* TGO/(-4 2 s), scale per p.720; SL* 0-9D,2 */

    *tgo = agc_dp_sub(ip, agc_dp_mul_trunc(t, MFOURDT), in->tailoff); /* BDSU */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * TGDCALC (Luminary S40.8 p.771): with d = UNIT(-DELVREF) . VG < 0,
 *   TGO = -4(200 cs) d (1 + d/2VEX) SR 10 / |DELVREF| + TDECAY.
 *
 * FUNCTION NAME: agc_guid_steering_tgo_lm
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  -----------------------------
 *   ip        agc_interp_t *        I/O  interpreter state (OVFIND, index
 *                                        registers)
 *   in        const agc_tgo_in_t *  I    S40.8 TGO inputs (DELVG, VG, VEX,
 *                                        tail-off)
 *   tgo       agc_dp_t *            O    TGO, time to engine-off command,
 *                                        cs 2^28
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed), AGC_STATUS_ALARM (an alarm was raised)
 **********************************************************************/
agc_status_t agc_guid_steering_tgo_lm(agc_interp_t *ip, const agc_tgo_in_t *in, agc_dp_t *tgo)
{
    agc_dp_t  dmag = 0;                                        /* |DELVREF|, m/cs 2^7 */
    agc_vec_t u    = agc_vec_unit(ip, in->delvg, &dmag, NULL); /* VCOMP UNIT, 2^1 */
    agc_dp_t  d    = agc_vec_dot(ip, u, in->vg);               /* 2^8 */

    if (d >= 0)
    {
        return (AGC_STATUS_ALARM); /* ALARMIT     */
    }

    agc_tp_t mt = agc_dp_mul_tp(agc_dp_add(ip, agc_dp_div(ip, d, in->vex), AGC_DP_HALF), d); /* d (1/2 + d/VEX), scale per p.771; DMP */

    if (!AGC_ASSERT(dmag != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t t = agc_dp_div(ip, agc_tp_to_dp(agc_tp_shift(ip, mt, -10)), dmag); /* TGO/(-4 2 s), scale per p.771; SR 10D (TP), DDV 36D */

    *tgo = agc_dp_add(ip, agc_dp_mul_trunc(t, MFOURDT), in->tailoff);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * XPRODUCT (Comanche S40.8 p.720): cross-product steering,
 *   OMEGAC = KPRIMEDT * SMNB(REFSMMAT (UNIT(VG) x UNIT(c BDT - DELVREF))).
 * With the velocity-to-be-gained along the thrust (c BDT - DELVREF
 * parallel to VG) the commanded rate is zero.
 *
 * FUNCTION NAME: agc_guid_steering_cross_product
 *
 * ARGUMENTS:
 *   Argument  Type                    I/O  Description (scale)
 *   --------  ----------------------  ---  ---------------------------
 *   ip        agc_interp_t *          I/O  interpreter state (OVFIND, index
 *                                          registers)
 *   in        const agc_xprod_in_t *  I    cross-product steering inputs
 *                                          (VG, BDT, DELVREF, CSTEER)
 *   refsmmat  const agc_mat_t *       I    REFSMMAT, half-unit rows
 *   trig      const agc_cdutrig_t *   I    CDU sines and cosines (CDUTRIG),
 *                                          2^1
 *   omegac    agc_vec_t *             O    commanded rate OMEGAC, control
 *                                          axes
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
agc_status_t agc_guid_steering_cross_product(agc_interp_t         *ip,
                                             const agc_xprod_in_t *in,
                                             const agc_mat_t      *refsmmat,
                                             const agc_cdutrig_t  *trig,
                                             agc_vec_t            *omegac)
{
    agc_vec_t a = agc_vec_shl(ip, agc_vec_scale(ip, in->bdt, in->csteer), 2);    /* BDT DT, m/cs 2^7; VSL2 */
    agc_vec_t w = agc_vec_unit(ip, agc_vec_sub(ip, a, in->delvref), NULL, NULL); /* 2^1 */
    agc_vec_t x = agc_vec_cross(ip, agc_vec_unit(ip, in->vg, NULL, NULL), w);    /* UNIT(VG) x W, 2^2 */

    x       = agc_mat_vec(ip, refsmmat, x);       /* REFSMMAT/2  */
    agc_guid_smnb(trig, &x);
    *omegac = agc_vec_scale(ip, x, in->kprimedt); /* OMEGACLC    */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * The short-burn time of S40.13 (Comanche p.727): v is |VGTIG| (less the
 * ullage), m = WEIGHT/G. Below K1/m: TGO = (m v + K2)/K3; otherwise with
 * TEMP1 = v - K1/m and TEMP2 = 5 s F/(m - 3.5 s EMDOT) (SR2):
 * TGO = 1 s + 5 s TEMP1/TEMP2 if TEMP1 < TEMP2, else steer (-1).
 *
 * FUNCTION NAME: timeburn_csm
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -----------------------------------
 *   ip        agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                  registers)
 *   v         agc_dp_t        I    |VGTIG| less the ullage, m/cs 2^7
 *   m         agc_dp_t        I    WEIGHT/G, mass, kg 2^16
 *   emdot     agc_sp_t        I    EMDOT, SPS flow rate, kg/cs 2^3
 *   fang      agc_sp_t        I    FANG, SPS thrust, M.newton 2^-?
 *
 * RETURNS: agc_dp_t -- TODO
 **********************************************************************/
static agc_dp_t timeburn_csm(agc_interp_t *ip, agc_dp_t v, agc_dp_t m, agc_sp_t emdot, agc_sp_t fang)
{
    const agc_guid_csm_t *c     = &agc_guid_state()->csm;                          /* EK1VAL..EK3VAL */
    agc_dp_t              temp1 = agc_dp_sub(ip, v, agc_dp_div(ip, c->ek1val, m)); /* m/cs 2^7; 00D-K1/m */

    if (temp1 < 0)
    { /* S40.131     */
        return (agc_dp_div(ip,
                           agc_dp_add(ip, agc_dp_mul_trunc(m, v), c->ek2val),
                           c->ek3val));
    }

    agc_dp_t mass  = agc_dp_sub(ip, m, agc_dp_mul_trunc(agc_dp_from_sp(emdot), SEC3_5)); /* kg 2^16 */
    agc_dp_t f5    = agc_dp_shr(agc_dp_mul_trunc(agc_dp_from_sp(fang), SEC5_14), 2);     /* 5 s FANG SR2, scale per p.727 */
    agc_dp_t temp2 = agc_dp_div(ip, f5, mass);                                           /* TEMP2 04D, m/cs 2^7 */

    if (agc_dp_sub(ip, temp1, temp2) >= 0)
    {
        return (-1); /* S40.133     */
    }

    if (!AGC_ASSERT(temp2 > 0))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_add(ip,
                       agc_dp_mul_trunc(agc_dp_div(ip, temp1, temp2), SEC5_14),
                       SEC1_2D));
}

/**********************************************************************
 * S40.13 TIMEBURN (Comanche p.726-728): IMPULSW stays set (attitude hold,
 * engine off at TGO) for a burn short enough; TGO (cs 2^28) is then the
 * time from ignition to the engine-off command. With +X translation the
 * 7.96 s of 2-jet (doubled for 4-jet) ullage impulse is deducted.
 *
 * FUNCTION NAME: agc_guid_timeburn_csm
 *
 * ARGUMENTS:
 *   Argument   Type            I/O  Description (scale)
 *   ---------  --------------  ---  ----------------------------------
 *   ip         agc_interp_t *  I/O  interpreter state (OVFIND, index
 *                                   registers)
 *   ullage     bool            I    +X translation commanded (channel 31
 *                                   bit 7 low)
 *   four_jets  bool            I    NJETSFLG clear: 4-jet ullage
 *                                   (correction doubled)
 *   emdot      agc_sp_t        I    EMDOT, SPS flow rate, kg/cs 2^3
 *   fang       agc_sp_t        I    FANG, SPS thrust, M.newton 2^-?
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG (a
 *          precondition failed)
 **********************************************************************/
agc_status_t agc_guid_timeburn_csm(agc_interp_t *ip, bool ullage, bool four_jets, agc_sp_t emdot, agc_sp_t fang)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(emdot >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_burn_t *b = &agc_guid_state()->burn; /* WEIGHT/G */

    agc_flag_set(AGC_FLAG_IMPULSW);                    /* ASSUME NO STEERING */
    agc_dp_t v = agc_vec_abval(ip, agc_erase_vgtig()); /* |VGTIG| m/cs 2^7 */

    if (ullage)
    {
        agc_dp_t dvu = agc_dp_div(ip, TWO_JET_IMPULSE, b->weight_g); /* m/cs 2^7 */

        if (four_jets)
        {
            dvu = agc_dp_shl(ip, dvu, 1); /* SL1: 4 JETS */
        }

        v = agc_dp_sub(ip, v, dvu); /* S40.130 BDSU */
    }

    agc_dp_t t14 = timeburn_csm(ip, v, b->weight_g, emdot, fang); /* cs 2^14 */

    if (t14 < 0)
    {
        agc_flag_clear(AGC_FLAG_IMPULSW);

        return (AGC_STATUS_OK); /* WILL STEER  */
    }

    agc_erase_set_tgo((agc_dp_t) agc_dp_high(t14)); /* TGO AT +28  */

    return (AGC_STATUS_OK);
}
