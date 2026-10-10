/*
 * sband_lm.c -- S-BAND_ANTENNA_FOR_LM.agc (Luminary099 pp. 486-489):
 * R05, the steerable S-band antenna pitch and yaw angles (V64).
 *
 * The earth direction UE is taken to navigation-base axes (*SMNB*), then
 * rotated -45 degrees about the Z axis into antenna-base axes:
 *   x' = (x + y)/SQRT2,  y' = (y - x)/SQRT2.
 * PITCHANG = ASIN(SGN(X.UY)|X|) with X = UZ x URP, adjusted to 180 - pitch
 * when URP.UZ < 0; YAWANG = ASIN(SGN(YAW.Z)|Z|) with Z = UR x URP and
 * YAW = UX cos(pitch) - UZ sin(pitch).  Angles in revs B0 (noun 51).
 */
#include <stdbool.h>
#include <stddef.h>
#include "agc/agc_assert.h"
#include "../programs/align_internal.h"

#define REMDIST    ((agc_dp_t)192201000) /* mean earth-moon distance B-29 m */
#define ONEOVSQRT2 ((agc_dp_t)189812531) /* 1OVSQRT2 .7071067815            */
#define VN_ANTENNA_ANGLES    ALIGN_VN(6, 51)        /* VN 0651                         */
#define BLANK_R3   ((uint16_t)04)        /* BIT3                            */
#define EXTVB_BIT5 ((uint16_t)020)
#define PRIO5      ((agc_prio_t)05)
#define PRIO4      ((agc_prio_t)04)

static agc_dp_t pitchang;  /* PITCHANG (= ALPHASB) N51 R1, revs            */
static agc_dp_t yawang;    /* YAWANG   (= BETASB)  N51 R2, revs            */

agc_dp_t agc_sband_pitchang(void)
{
    (void)AGC_ASSERT(pitchang >= -AGC_DP_HALF);
    (void)AGC_ASSERT(pitchang <= AGC_DP_MAX);
    return pitchang;
}

agc_dp_t agc_sband_yawang(void)
{
    (void)AGC_ASSERT(yawang >= -AGC_DP_HALF);
    (void)AGC_ASSERT(yawang <= AGC_DP_HALF);
    return yawang;
}

/* ABVAL SIGN ASIN: ASIN(SGN(s) |v|). */
static agc_dp_t signed_asin(agc_interp_t *ip, agc_vec_t v, agc_dp_t s)
{
    (void)AGC_ASSERT(ip != NULL);
    const agc_dp_t m = agc_vec_abval(ip, v);
    (void)AGC_ASSERT(m >= 0);
    return agc_dp_asin(ip, agc_dp_sign(m, s));
}

/*
 * The angles for r_nb, the unit LM-to-earth vector in NB axes (half-unit,
 * the *SMNB* output stored in RLM).  Outputs revs.
 */
agc_status_t agc_sband_lm_angles(agc_interp_t *ip, agc_vec_t r_nb,
                                 agc_dp_t *pitch, agc_dp_t *yaw)
{
    if (!AGC_ASSERT((ip != NULL) && (pitch != NULL)) || !AGC_ASSERT(yaw != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_mat_t u = agc_mat_identity_half();          /* HIUNITX,Y,Z  */
    *pitch = 0;                                           /* HI6ZEROS     */
    *yaw = 0;
    agc_vec_t r = r_nb;
    const agc_dp_t y0 = r.c[1];
    r.c[1] = agc_dp_mul_trunc(agc_dp_sub(ip, y0, r.c[0]), ONEOVSQRT2); /* DMP */
    r.c[0] = agc_dp_mul_trunc(agc_dp_add(ip, y0, r.c[0]), ONEOVSQRT2);
    const agc_vec_t ur = agc_vec_unit(ip, r, NULL, NULL);            /* 0D */
    const agc_vec_t py = agc_vec_shl(ip, agc_vec_proj(ip, r, u.row[1]), 2);
    agc_vec_t urp = agc_vec_sub(ip, r, py);                /* BVSU RLM     */
    (void)agc_interp_test_ovf(ip);
    urp = agc_vec_unit(ip, urp, NULL, NULL);               /* COVCNV       */
    if (agc_interp_test_ovf(ip)) {
        return AGC_STATUS_OK;                              /* BOV SBANDEX  */
    }
    const agc_vec_t x = agc_vec_neg(agc_vec_shl(ip, agc_vec_cross(ip, urp, u.row[2]), 1));
    *pitch = signed_asin(ip, x, agc_vec_dot(ip, x, u.row[1]));
    if (agc_vec_dot(ip, urp, u.row[2]) < 0) {              /* BPL NOADJUST */
        *pitch = agc_dp_sub(ip, AGC_DP_HALF, *pitch);      /* HIDPHALF - p */
    }
    const agc_vec_t z = agc_vec_shl(ip, agc_vec_cross(ip, ur, urp), 1);
    const agc_vec_t a = agc_vec_scale(ip, u.row[2], agc_dp_sin(ip, *pitch));
    const agc_vec_t b = agc_vec_scale(ip, u.row[0], agc_dp_cos(ip, *pitch));
    const agc_dp_t yz = agc_vec_dot(ip, agc_vec_sub(ip, b, a), z);  /* YAW.Z */
    *yaw = signed_asin(ip, z, yz);
    return AGC_STATUS_OK;
}

/* UE = -UNIT(RATT) (earth sphere) or -UNIT(REMDIST VMOON SL1 + RATT). */
static agc_status_t earth_unit(agc_interp_t *ip, const agc_integ_out_t *o,
                               agc_vec_t *ue)
{
    (void)AGC_ASSERT(o != NULL);
    (void)AGC_ASSERT(ue != NULL);
    agc_vec_t r = o->ratt;                                 /* CONV4        */
    if (o->x2 != 0) {                                      /* CONV3 LUNPOS */
        agc_vec_t vmoon;
        agc_vec_t vsun;
        const agc_status_t st = agc_lsposition(ip, o->tat, &vmoon, &vsun);
        if (st != AGC_STATUS_OK) {
            return st;
        }
        r = agc_vec_add(ip, agc_vec_shl(ip, agc_vec_scale(ip, vmoon, REMDIST), 1),
                        o->ratt);
    }
    *ue = agc_vec_neg(agc_vec_unit(ip, r, NULL, NULL));     /* CONV5        */
    return AGC_STATUS_OK;
}

/* SBANDEX: display VN_ANTENNA_ANGLES at PRIO5 while EXTVBACT bit 5 is on. */
static agc_status_t sbandex(const agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    if ((agc_erase_extvbact() & EXTVB_BIT5) == 0u) {
        return agc_dsky_endext();
    }
    (void)agc_exec_prio_change(PRIO5);         /* PRIOCHNG (no pre-emption) */
    agc_status_t st = align_flash(ctx, AGC_DSKY_GOMARKFR, VN_ANTENNA_ANGLES,
                                  AGC_JOB_SBAND_B5OFF, AGC_JOB_SBAND_B5OFF,
                                  AGC_JOB_ENDOFJOB);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    st = agc_dsky_blanket(BLANK_R3);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_exec_prio_change(PRIO4);
    if (st == AGC_STATUS_BUSY) {
        return align_goto(AGC_JOB_SBANDANT_LM_RECYCLE);   /* CHANG1       */
    }
    return align_goto(AGC_JOB_SBANDANT_LM_RECYCLE);       /* TC SBANDANT+2 */
}

/* SBANDANT +2 (p. 486-488): LEMCONIC to now, UE, CDUTRIG, SM, *SMNB*. */
agc_status_t agc_job_sbandant_lm_recycle(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_integ_out_t o;
    agc_vec_t ue = agc_vec_zero();
    agc_cdutrig_t trig;
    agc_status_t st = align_integ(&ip, ALIGN_LEMCONIC, align_loadtime(),
                                  AGC_JOB_SBANDANT_LM_RECYCLE, &o);
    if (st == AGC_STATUS_OK) {
        st = earth_unit(&ip, &o, &ue);
    }
    if (st == AGC_STATUS_OK) {
        st = agc_guid_cdutrig(&trig);                       /* CDUTRIG      */
    }
    if (st == AGC_STATUS_OK) {
        const agc_mat_t refsmmat = agc_erase_refsmmat();
        ue = agc_vec_shl(&ip, agc_mat_vec(&ip, &refsmmat, ue), 1);
        st = agc_guid_smnb(&trig, &ue);                     /* *SMNB*       */
    }
    if (st == AGC_STATUS_OK) {
        st = agc_sband_lm_angles(&ip, ue, &pitchang, &yawang);
    }
    if (st != AGC_STATUS_OK) {
        return (st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st;
    }
    return sbandex(ctx);
}

/* SBANDANT (V64 E): R02BOTH, then SBANDANT +2. */
agc_status_t agc_job_sbandant_lm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_status_t st = agc_imu_r02both();
    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG) || (st != AGC_STATUS_OK)) {
        return AGC_STATUS_OK;
    }
    return agc_job_sbandant_lm_recycle(ctx);
}
