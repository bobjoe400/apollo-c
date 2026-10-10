/*
 * p40.c -- the steering mathematics of P40-P47.agc shared by Comanche055
 * (pp. 709-728) and Luminary099 (pp. 764-784): S40.1 (initial thrust
 * direction UT and VGTIG; CALCTHET for external delta-V, S40.1B for the
 * Lambert aimpoint), AGAIN's companion CALCUT, S40.2,3 (preferred
 * attitude), S41.1 (VG to control/body axes), the S40.8 time-to-go
 * computations and the Comanche cross-product steering XPRODUCT, and the
 * Comanche S40.13 TIMEBURN. The flag-dependent sequencing is in
 * src/csm/burns/sps/burn.c and src/lm/burns/dps_burn.c.
 *
 * Scales: UT half unit (2^1); VGTIG, VG, DELVREF, BDT, DELVSIN m/cs 2^7;
 * RTIG m 2^29; F m.newton 2^7; WEIGHT/G kg 2^16; TGO cs 2^28; angles rev.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/guidance/guidance.h"
#include "agc/guidance/pending.h"

/* THETACON 2DEC .31830989 B-8 (1/pi).                                     */
static const agc_dp_t THETACON = 333772;
/* TWODT 2DEC 200.0 B-28: 2 s.                                             */
static const agc_dp_t TWODT = 200;
/* 200CS 2DEC 200 B-12.                                                    */
static const agc_dp_t CS200_B12 = 13107200;
/* EPS1 (10 deg) and EPS2 (35 deg) at 1 revolution (LM S40.1B).            */
static const agc_dp_t EPS1 = 7456540;
static const agc_dp_t EPS2 = 26097892;
/* TRIMSCAL 2DEC 1.07975111 B-1: 85.41"/bit trims to rev; YBIAS, PBIAS:
 * yaw +0.95 deg and pitch -2.15 deg mechanical bias, thrust on.          */
static const agc_dp_t TRIMSCAL = 144921741;
static const agc_dp_t YBIAS    = 708371;
static const agc_dp_t PBIAS    = -1603156;
/* TENBNK14 2DEC 10. B-4; QUADROT: S/C to control axes, X rotation -7.25
 * degrees, elements times 0.1 (GIMBAL_LOCK_AVOIDANCE p.412-413).          */
static const agc_dp_t TENBNK14 = 167772160;
static const agc_dp_t Q_P1     = 26843546; /* .1                         */
static const agc_dp_t Q_COS    = 26628797; /* .099200 = (.1) COS 7.25    */
static const agc_dp_t Q_SIN    = 3387655;  /* .012620 = (.1) SIN 7.25    */
/* -FOURDT 2DEC -800 B-18: -4 (200 cs), -4 for scaling.                    */
static const agc_dp_t MFOURDT = -819200;
/* S40.13 (CM): S40.135 2DEC 69.6005183 B-23 (7.96 s of 2-jet ullage),
 * 1SEC2D 100 B-14, 3.5SEC 350 B-13, 5SECOND 500 B-14.                     */
static const agc_dp_t TWO_JET_IMPULSE = 2227;
static const agc_dp_t SEC1_2D         = 1638400;
static const agc_dp_t SEC3_5          = 11468800;
static const agc_dp_t SEC5_14         = 8192000;

/* GET.LVC / MIDGIM for the VGTIG display: DELVLVC (LV axes of RINIT,
 * VINIT). Comanche: SET AVFLAG, MIDGIM; Luminary: GET.LVC.                 */
static agc_status_t vg_to_lvc(agc_interp_t *ip, agc_vehicle_t veh, agc_vec_t vg)
{
    agc_rendz_t *rz = agc_rendz_erasable();

    if (veh == AGC_VEHICLE_CSM)
    {
        agc_flag_set(AGC_FLAG_AVFLAG);

        return agc_guid_midgim(ip, vg);
    }

    if (!AGC_ASSERT(rz != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return agc_rendz_get_lvc(ip, rz->rinit, rz->vinit, vg, &rz->delvlvc);
}

/* CALCTHET (CM p.709-710, LM p.765-766): the impulsive DELVSIN is spread
 * over the burn arc. theta/2 = (|RxV| / R^2) (1/pi) |DV| m / F; the
 * in-plane part DVP is rotated by theta/2 about UP:
 *   VGTIG = |DVP| (cos(theta/2) UNIT(DVP) + sin(theta/2) UNIT(DVP x UP))
 *           VSL2 + (DV . UP) UP;   UT = UNIT(VGTIG).                       */
static agc_status_t calcthet(agc_interp_t *ip, agc_vehicle_t veh)
{
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_burn_t  *b  = &agc_guid_state()->burn;

    rz->vinit = rz->vtig;
    agc_dp_t h = 0;

    b->ut = agc_vec_unit(ip, agc_vec_cross(ip, rz->vtig, rz->rtig), &h,
                         NULL);        /* UP */
    rz->rinit = rz->rtig;
    agc_dp_t rsq = agc_vec_sq(ip, rz->rtig); /* VSQ PDDL 36D */
    agc_dp_t th  = agc_dp_div(ip, agc_dp_mul(ip, h, THETACON), rsq);

    th = agc_dp_mul(ip, agc_dp_mul(ip, th, rz->delvsab), b->weight_g);
    th = agc_dp_div(ip, th, b->f); /* 14D         */
    agc_vec_t dvup = agc_vec_shl(
        ip, agc_vec_scale(ip, b->ut, agc_vec_dot(ip, rz->delvsin, b->ut)),
        2);                                             /* PDL 0       */
    agc_vec_t dvp = agc_vec_sub(ip, rz->delvsin, dvup); /* PDL 6       */
    agc_dp_t  sn  = agc_dp_sin(ip, th);
    agc_vec_t vg  = agc_vec_scale(
        ip, agc_vec_unit(ip, agc_vec_cross(ip, dvp, b->ut), NULL, NULL), sn);
    agc_dp_t  dvpmag = 0;
    agc_vec_t udvp   = agc_vec_unit(ip, dvp, &dvpmag, NULL); /* 36D = |DVP| */

    vg = agc_vec_add(ip, agc_vec_scale(ip, udvp, agc_dp_cos(ip, th)), vg);
    vg = agc_vec_add(ip, agc_vec_shl(ip, agc_vec_scale(ip, vg, dvpmag), 2),
                     dvup);
    agc_erase_set_vgtig(vg);                  /* VG IGNITION */
    b->ut = agc_vec_unit(ip, vg, NULL, NULL); /* THRUST DIR. */

    if (!AGC_ASSERT(b->f > 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return vg_to_lvc(ip, veh, vg);
}

/* CALCUT (Comanche p.712-713): thrust direction for aimpoint steering.
 *   b  = (VIPRIME - VIPRIME(TIG-2)) / 2 s - g;   bc = CSTEER b
 *   q  = bc - (ug . bc) ug,   ug = UNIT(VGTIG)
 *   UT = UNIT(q + ug sqrt((F/m)^2 - q^2))                                  */
agc_status_t agc_guid_calcut(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_guid_t  *g  = agc_guid_state();
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_status_t st = agc_guid_calcgrav(ip, AGC_VEHICLE_CSM, rz->body.rtx2 != 0,
                                        rz->rtig, &g->avg);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    agc_vec_t gv =
        agc_vec_div(ip, agc_vec_shl(ip, g->avg.gdt1_2, 1), CS200_B12);
    agc_vec_t dvr =
        agc_vec_div(ip, agc_vec_sub(ip, rz->viprime, g->burn.ut), CS200_B12);
    agc_vec_t bc = agc_vec_shl(
        ip, agc_vec_scale(ip, agc_vec_sub(ip, dvr, gv), g->burn.csteer),
        2);                                                         /* 12D */
    agc_vec_t ug = agc_vec_unit(ip, agc_erase_vgtig(), NULL, NULL); /* PDL 0 */
    agc_vec_t q  = agc_vec_sub(
        ip, bc,
        agc_vec_shl(ip, agc_vec_scale(ip, ug, agc_vec_dot(ip, ug, bc)), 2));
    agc_dp_t fm =
        agc_dp_div(ip, agc_dp_shr_round(g->burn.f, 4), g->burn.weight_g);
    agc_dp_t  rad = agc_dp_sub(ip, agc_dp_sq(ip, fm), agc_vec_sq(ip, q));
    agc_vec_t t   =
        agc_vec_shl(ip, agc_vec_scale(ip, ug, agc_dp_sqrt(ip, rad)), 1);

    g->burn.ut = agc_vec_unit(ip, agc_vec_add(ip, t, q), NULL, NULL);

    if (!AGC_ASSERT(rad >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* S40.1B Comanche (p.710-711): VIPRIME at TIG - 2 s (kept in UT), then
 * the Lambert solution at TIG: VGTIG = DELVEET3, DELVLVC, CALCUT.         */
static agc_status_t initial_vg_lambert_csm(agc_interp_t *ip)
{
    agc_rendz_t *rz    = agc_rendz_erasable();
    agc_dp_t     tdec1 = agc_dp_sub(ip, agc_erase_tig(), TWODT);

    rz->dellt4 = agc_dp_sub(ip, rz->tpass4, tdec1);
    agc_status_t st = agc_guid_again(ip, tdec1);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    agc_guid_state()->burn.ut = rz->viprime; /* STODL UT    */
    rz->dellt4                = agc_dp_sub(ip, rz->tpass4, agc_erase_tig());
    st                        = agc_guid_again(ip, agc_erase_tig());

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    agc_erase_set_vgtig(rz->delveet3);
    st = vg_to_lvc(ip, AGC_VEHICLE_CSM, rz->delveet3);

    return (st != AGC_STATUS_OK) ? st : agc_guid_calcut(ip);
}

/* S40.1B Luminary (p.766-767): LEMPREC to TIG, INITVEL (NUMIT 0, EPSILON4
 * 10 degrees, or 45 with NORMSW), VGTIG = DELVEET3, UT, VGDISP, DELVLVC.  */
static agc_status_t initial_vg_lambert_lm(agc_interp_t *ip)
{
    agc_rendz_t *rz = agc_rendz_erasable();
    agc_guid_t  *g  = agc_guid_state();

    rz->dellt4 = agc_dp_sub(ip, rz->tpass4, agc_erase_tig()); /* TDEC1 = TIG */
    agc_guid_prec_t p;
    agc_status_t    st = agc_guid_prec_update(ip, AGC_VEHICLE_LM,
                                              agc_erase_tig(),
                                              AGC_JOB_NONE, &p); /* LEMPREC */

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    rz->rtig      = rz->rinit = p.ratt;
    g->avg.unitr  = agc_vec_unit(ip, p.ratt, NULL, NULL); /* UNIT/R/     */
    rz->vtig      = rz->vinit = p.vatt;
    rz->body.rtx1 = p.rtx1;
    rz->body.rtx2 = p.rtx2;
    agc_dp_t e4 =
        agc_flag_test(AGC_FLAG_NORMSW) ? agc_dp_add(ip, EPS1, EPS2) : EPS1;

    st = agc_rendz_initvel(ip, rz, 0, e4, agc_guid_nav_resume());

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    agc_erase_set_vgtig(rz->delveet3); /* VR - VN     */
    g->burn.ut = agc_vec_unit(ip, rz->delveet3, &g->burn.vgdisp, NULL);

    return vg_to_lvc(ip, AGC_VEHICLE_LM, rz->delveet3);
}

/* S40.1 (CM p.709, LM p.764): initial thrust direction UT and VGTIG.
 * Comanche: BDT = 0 here; the caller sets FIRSTFLG (a Comanche-only flag).
 * Luminary: TIGSAVE = TIG.                                                */
agc_status_t agc_guid_initial_velocity_to_gain(agc_interp_t *ip,
                                               agc_vehicle_t veh)
{
    if (!AGC_ASSERT(ip != NULL) ||
        !AGC_ASSERT(veh == AGC_VEHICLE_CSM || veh == AGC_VEHICLE_LM))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_guid_t  *g  = agc_guid_state();
    agc_rendz_t *rz = agc_rendz_erasable();

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
        return (veh == AGC_VEHICLE_CSM) ? initial_vg_lambert_csm(ip) :
               initial_vg_lambert_lm(ip);
    }

    if (veh == AGC_VEHICLE_CSM)
    {
        rz->delvsab = agc_vec_abval(ip, rz->delvsin); /* P30/P40 I/F */
    }

    return calcthet(ip, veh);
}

/* TSTRXUT / BADVCTOR (Comanche p.716): if UT x R was null (36D = 0) use
 * Y = -UNIT((UNIT(R) + UNIT(V)/8) x UT) instead.                          */
static agc_vec_t tstrxut(agc_interp_t *ip, agc_vec_t y, agc_dp_t mag)
{
    agc_rendz_t *rz = agc_rendz_erasable();

    if (mag != 0)
    {
        return y;
    }

    agc_vec_t s =
        agc_vec_add(ip, agc_vec_shr(agc_vec_unit(ip, rz->vtig, NULL, NULL), 3),
                    agc_vec_unit(ip, rz->rtig, NULL, NULL));

    if (!AGC_ASSERT(mag == 0))
    {
        return agc_vec_zero(); /* no valid result */
    }

    return agc_vec_neg(agc_vec_unit(
                           ip, agc_vec_cross(ip, s, agc_guid_state()->burn.ut),
                           NULL, NULL));
}

/* The SPS bell axes of S40.2,3 (Comanche p.715): pitch p = PACTOFF*TRIMSCAL
 * SL1 + PBIAS, yaw y = YACTOFF*TRIMSCAL SL1 + YBIAS (rev). VDEF takes X
 * from MPAC and Y, Z from the push-down list:
 *   XSC = (cp cy, -cp sy, sp)  ZSC = (-sp cy, sp sy, cp)  YSC = (sy, cy, 0)
 * (all halves, sin/cos SL1); SCAXIS = (XSC.x, YSC.x, ZSC.x).            */
static void bell_axes(agc_interp_t *ip, agc_sp_t pactoff, agc_sp_t yactoff,
                      agc_burn_t *b)
{
    agc_dp_t y = agc_dp_add(
        ip,
        agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_from_sp(yactoff), TRIMSCAL), 1),
        YBIAS);
    agc_dp_t p = agc_dp_add(
        ip,
        agc_dp_shl(ip, agc_dp_mul(ip, agc_dp_from_sp(pactoff), TRIMSCAL), 1),
        PBIAS);
    agc_dp_t cy = agc_dp_cos(ip, y), sy = agc_dp_sin(ip, y);
    agc_dp_t cp = agc_dp_cos(ip, p), sp = agc_dp_sin(ip, p);

    b->xscref =
        agc_vec_make(agc_dp_shl(ip, agc_dp_mul(ip, cp, cy), 1),
                     agc_dp_neg(agc_dp_shl(ip, agc_dp_mul(ip, cp, sy), 1)), sp);
    b->zscref =
        agc_vec_make(agc_dp_neg(agc_dp_shl(ip, agc_dp_mul(ip, sp, cy), 1)),
                     agc_dp_shl(ip, agc_dp_mul(ip, sp, sy), 1), cp);
    b->yscref = agc_vec_make(sy, cy, 0);
    b->scaxis = agc_vec_make(b->xscref.c[0], b->yscref.c[0], b->zscref.c[0]);

    if (!AGC_ASSERT(cp > 0))
    {
        return;
    }
}

/* S40.2,3 Comanche (p.714-716): POINTVSM = REFSMMAT UT VSL1; for the SPS
 * the bell axes give SCAXIS and, rotated into the frame X = UT, Y = UNIT(
 * UT x R), Z = UT x Y, the preferred orientation XSCREF..ZSCREF; for the
 * RCS (ENG2FLAG) SCAXIS = UNITX and the frame itself.                     */
agc_status_t agc_guid_preferred_thrust_attitude_csm(agc_interp_t *ip, bool rcs,
                                                    agc_sp_t pactoff,
                                                    agc_sp_t yactoff)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_burn_t  *b        = &agc_guid_state()->burn;
    agc_rendz_t *rz       = agc_rendz_erasable();
    agc_mat_t    refsmmat = agc_erase_refsmmat();

    b->pointvsm =
        agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, b->ut), 1); /* SM AT 2 */
    agc_dp_t  mag = 0;
    agc_vec_t y   =
        agc_vec_unit(ip, agc_vec_cross(ip, b->ut, rz->rtig), &mag, NULL);

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
        bell_axes(ip, pactoff, yactoff, b);
        agc_mat_t m;

        m.row[0]  = b->ut; /* 2 RF/SC     */
        m.row[1]  = y;
        m.row[2]  = agc_vec_shl(ip, agc_vec_cross(ip, b->ut, y), 1);
        b->xscref = agc_vec_shl(ip, agc_vec_mat(ip, b->xscref, &m), 1);
        b->yscref = agc_vec_shl(ip, agc_vec_mat(ip, b->yscref, &m), 1);
        b->zscref = agc_vec_shl(ip, agc_vec_mat(ip, b->zscref, &m), 1);
    }

    agc_imu_set_pointvsm(b->pointvsm); /* FOR R60     */

    return agc_imu_set_scaxis(b->scaxis);
}

/* S40.2,3 Luminary (p.768-769): POINTVSM, SCAXIS = +X (DPS, APS and RCS
 * alike), XSCREF = UT, YSCREF = UNIT(UT x RTIG) (or UNIT(UT x VTIG) when
 * that is null: FIXY), ZSCREF = -(YSCREF x XSCREF) VSL1; PFRATFLG set.    */
agc_status_t agc_guid_preferred_thrust_attitude_lm(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_burn_t  *b        = &agc_guid_state()->burn;
    agc_rendz_t *rz       = agc_rendz_erasable();
    agc_mat_t    refsmmat = agc_erase_refsmmat();

    b->pointvsm = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, b->ut), 1);
    b->scaxis   = agc_vec_make(AGC_DP_HALF, 0, 0);  /* UNITX       */
    b->xscref   = b->ut;                            /* PLUSX       */
    agc_dp_t mag = 0;

    b->yscref =
        agc_vec_unit(ip, agc_vec_cross(ip, b->ut, rz->rtig), &mag, NULL);

    if (mag == 0)
    { /* FIXY        */
        b->yscref = agc_vec_unit(ip, agc_vec_cross(ip, b->xscref, rz->vtig),
                                 NULL, NULL);
    }

    b->zscref = agc_vec_neg(
        agc_vec_shl(ip, agc_vec_cross(ip, b->yscref, b->xscref), 1));
    agc_flag_set(AGC_FLAG_PFRATFLG);
    agc_imu_set_pointvsm(b->pointvsm);

    return agc_imu_set_scaxis(b->scaxis);
}

/* S41.1 (CM p.717-718, LM p.783-784): VG (reference, m/cs 2^7) to the
 * Comanche control axes (CDUTRIG, REFSMMAT, *SMNB*, QUADROT, x10, VSL5)
 * or the Luminary body axes (REFSMMAT VSL1, CDU*SMNB).                    */
void agc_guid_reference_to_body(agc_interp_t *ip, agc_vehicle_t veh,
                                agc_vec_t vg_ref, agc_vec_t *vg_ctl)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(vg_ctl != NULL))
    {
        return;
    }

    agc_guid_t *g        = agc_guid_state();
    agc_mat_t   refsmmat = agc_erase_refsmmat();
    agc_vec_t   v        = agc_mat_vec(ip, &refsmmat, vg_ref);

    if (veh == AGC_VEHICLE_LM)
    {
        v = agc_vec_shl(ip, v, 1);
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

/* TGOCALC (Comanche S40.8 p.720): with d = UNIT(BDT - DELVREF) . VG (must
 * be negative: thrust within 90 degrees of VG, else alarm 1407 by the
 * caller -- AGC_STATUS_ALARM),
 *   TGO = -4(200 cs) d (1 + d/2Vex) / |BDT - DELVREF| - ETDECAY,
 * normalised by NORM X1 / NORM X2 and SL* (n2 - n1 - 9).                  */
agc_status_t agc_guid_steering_tgo_csm(agc_interp_t *ip, const agc_tgo_in_t *in,
                                       agc_dp_t *tgo)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(tgo != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_dp_t  dmag = 0;
    agc_vec_t u    = agc_vec_unit(ip, in->delvg, &dmag, NULL); /* 36D         */
    agc_dp_t  d    = agc_vec_dot(ip, u, in->vg);

    if (d >= 0)
    {
        return AGC_STATUS_ALARM; /* INCRSVG     */
    }

    agc_dp_t m =
        agc_dp_add(ip, agc_dp_div(ip, d, in->vex), AGC_DP_HALF); /* LODPHALF */

    m = agc_dp_mul(ip, m, d);
    int n1 = 0, n2 = 0;

    m = agc_dp_shr(agc_dp_norm(m, &n1), 1); /* NORM X1 SR1 */
    agc_dp_t den = agc_dp_norm(dmag, &n2);             /* NORM X2     */

    if (!AGC_ASSERT(den != 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_dp_t t =
        agc_dp_shl(ip, agc_dp_div(ip, m, den), n2 - n1 - 9); /* SL* 0-9D,2 */

    *tgo =
        agc_dp_sub(ip, agc_dp_mul(ip, t, MFOURDT), in->tailoff); /* BDSU     */

    return AGC_STATUS_OK;
}

/* TGDCALC (Luminary S40.8 p.771): with d = UNIT(-DELVREF) . VG < 0,
 *   TGO = -4(200 cs) d (1 + d/2VEX) SR 10 / |DELVREF| + TDECAY.           */
agc_status_t agc_guid_steering_tgo_lm(agc_interp_t *ip, const agc_tgo_in_t *in,
                                      agc_dp_t *tgo)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) || !AGC_ASSERT(tgo != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_dp_t  dmag = 0;
    agc_vec_t u    = agc_vec_unit(ip, in->delvg, &dmag, NULL); /* VCOMP UNIT  */
    agc_dp_t  d    = agc_vec_dot(ip, u, in->vg);

    if (d >= 0)
    {
        return AGC_STATUS_ALARM; /* ALARMIT     */
    }

    agc_dp_t m = agc_dp_mul(
        ip, agc_dp_add(ip, agc_dp_div(ip, d, in->vex), AGC_DP_HALF), d);

    if (!AGC_ASSERT(dmag != 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_dp_t t = agc_dp_div(ip, agc_dp_shr(m, 10), dmag); /* SR DDV 36D  */

    *tgo = agc_dp_add(ip, agc_dp_mul(ip, t, MFOURDT), in->tailoff);

    return AGC_STATUS_OK;
}

/* XPRODUCT (Comanche S40.8 p.720): cross-product steering,
 *   OMEGAC = KPRIMEDT * SMNB(REFSMMAT (UNIT(VG) x UNIT(c BDT - DELVREF))).
 * With the velocity-to-be-gained along the thrust (c BDT - DELVREF
 * parallel to VG) the commanded rate is zero.                             */
agc_status_t agc_guid_steering_cross_product(agc_interp_t         *ip,
                                             const agc_xprod_in_t *in,
                                             const agc_mat_t      *refsmmat,
                                             const agc_cdutrig_t  *trig,
                                             agc_vec_t            *omegac)
{
    if (!AGC_ASSERT(ip != NULL && in != NULL) ||
        !AGC_ASSERT(refsmmat != NULL && trig != NULL && omegac != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_vec_t a =
        agc_vec_shl(ip, agc_vec_scale(ip, in->bdt, in->csteer), 2); /* VSL2 */
    agc_vec_t w = agc_vec_unit(ip, agc_vec_sub(ip, a, in->delvref), NULL, NULL);
    agc_vec_t x = agc_vec_cross(ip, agc_vec_unit(ip, in->vg, NULL, NULL), w);

    x = agc_mat_vec(ip, refsmmat, x); /* REFSMMAT/2  */
    agc_guid_smnb(trig, &x);
    *omegac = agc_vec_scale(ip, x, in->kprimedt); /* OMEGACLC    */

    return AGC_STATUS_OK;
}

/* The short-burn time of S40.13 (Comanche p.727): v is |VGTIG| (less the
 * ullage), m = WEIGHT/G. Below K1/m: TGO = (m v + K2)/K3; otherwise with
 * TEMP1 = v - K1/m and TEMP2 = 5 s F/(m - 3.5 s EMDOT) (SR2):
 * TGO = 1 s + 5 s TEMP1/TEMP2 if TEMP1 < TEMP2, else steer (-1).          */
static agc_dp_t timeburn_csm(agc_interp_t *ip, agc_dp_t v, agc_dp_t m,
                             agc_sp_t emdot, agc_sp_t fang)
{
    const agc_guid_csm_t *c     = &agc_guid_state()->csm;
    agc_dp_t              temp1 =
        agc_dp_sub(ip, v, agc_dp_div(ip, c->ek1val, m)); /* 00D-K1/m */

    if (temp1 < 0)
    { /* S40.131     */
        return agc_dp_div(ip, agc_dp_add(ip, agc_dp_mul(ip, m, v), c->ek2val),
                          c->ek3val);
    }

    agc_dp_t mass =
        agc_dp_sub(ip, m, agc_dp_mul(ip, agc_dp_from_sp(emdot), SEC3_5));
    agc_dp_t f5 =
        agc_dp_shr(agc_dp_mul(ip, agc_dp_from_sp(fang), SEC5_14), 2);
    agc_dp_t temp2 = agc_dp_div(ip, f5, mass); /* 04D         */

    if (agc_dp_sub(ip, temp1, temp2) >= 0)
    {
        return -1; /* S40.133     */
    }

    if (!AGC_ASSERT(temp2 > 0))
    {
        return 0; /* no valid result */
    }

    return agc_dp_add(ip, agc_dp_mul(ip, agc_dp_div(ip, temp1, temp2), SEC5_14),
                      SEC1_2D);
}

/* S40.13 TIMEBURN (Comanche p.726-728): IMPULSW stays set (attitude hold,
 * engine off at TGO) for a burn short enough; TGO (cs 2^28) is then the
 * time from ignition to the engine-off command. With +X translation the
 * 7.96 s of 2-jet (doubled for 4-jet) ullage impulse is deducted.          */
agc_status_t agc_guid_timeburn_csm(agc_interp_t *ip, bool ullage,
                                   bool four_jets, agc_sp_t emdot,
                                   agc_sp_t fang)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(emdot >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_burn_t *b = &agc_guid_state()->burn;

    agc_flag_set(AGC_FLAG_IMPULSW); /* ASSUME NO STEERING */
    agc_dp_t v = agc_vec_abval(ip, agc_erase_vgtig());

    if (ullage)
    {
        agc_dp_t dvu = agc_dp_div(ip, TWO_JET_IMPULSE, b->weight_g);

        if (four_jets)
        {
            dvu = agc_dp_shl(ip, dvu, 1); /* SL1: 4 JETS */
        }

        v = agc_dp_sub(ip, v, dvu); /* S40.130 BDSU */
    }

    agc_dp_t t14 = timeburn_csm(ip, v, b->weight_g, emdot, fang);

    if (t14 < 0)
    {
        agc_flag_clear(AGC_FLAG_IMPULSW);

        return AGC_STATUS_OK; /* WILL STEER  */
    }

    agc_erase_set_tgo((agc_dp_t) agc_dp_high(t14)); /* TGO AT +28  */

    return AGC_STATUS_OK;
}
