/*
 * align_s52.c -- orientation computations of P52/P54/P57 (P51-P53.agc):
 * S52.2 gimbal angles for a desired stable member, S52.3 nominal
 * orientation, the landing-site orientation (P52LS / LSORIENT), the
 * CAL53A coarse-align decision and the gyro-torque coarse-align matrix.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/programs/alignment_internal.h"

#define CDU_WORD_MASK 077777u
#define CDU_SIGN_BIT  040000u
#define CDU_ONES_NEG  077777 /* 1's complement negative bias           */
#define DEGREE1_X2    92     /* DEGREE1 = DEC 46, 1 deg at CDU/2; x2   */
#define DEG359_X2     32676  /* DEG359 = DEC 16338, 359 deg at CDU/2; x2 */

/*
 * S52.2 (P51-P53.agc, Comanche p. 774 / Luminary p. 953, label S52.2) --
 * compute the gimbal angles for the desired stable member XSMD.. and the
 * present vehicle attitude.  CDUTRIG + CALCSMSC give the nav base wrt the
 * present SM; each axis is taken to the reference frame (VXM REFSMMAT,
 * UNIT); XSM = XSMD (MATMOVE); CALCGA gives THETAD.
 *   smd      XSMD,YSMD,ZSMD   half-unit, reference coordinates
 *   cdu      CDUX,CDUY,CDUZ   cdu
 *   xsm      XSM.. output     half-unit, reference
 *   xnb      XNB.. output     half-unit, reference
 *   thetad   THETAD +0..+2    cdu (outer, inner, middle)
 */
agc_status_t agc_align_gimbal_angles(agc_interp_t *ip, const agc_mat_t *smd,
                                     const agc_mat_t *refsmmat,
                                     const uint16_t cdu[3], agc_mat_t *xsm,
                                     agc_mat_t *xnb, uint16_t thetad[3])
{
    if (!AGC_ASSERT((ip != NULL) && (smd != NULL) && (refsmmat != NULL)) ||
        !AGC_ASSERT((cdu != NULL) && (xsm != NULL) && (xnb != NULL) &&
                    (thetad != NULL)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_cdutrig_t trig;
    agc_status_t  st = agc_align_cdutrig(cdu, &trig); /* CDUTRIG        */
    if (st == AGC_STATUS_OK)
    {
        st = agc_align_calcsmsc(ip, &trig, xnb); /* CALCSMSC       */
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    /* S52.2A: X1 = 18, 12, 6 selects XNB, YNB, ZNB in turn (TIX,1 by 6). */
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        xnb->row[i] = agc_vec_unit(ip, agc_vec_mat(ip, xnb->row[i], refsmmat),
                                   NULL, NULL);
    }
    *xsm = *smd;                                 /* S52.2.1 MATMOVE        */
    return agc_imu_calcga(ip, xnb, xsm, thetad); /* CALCGA -> THETAD       */
}

/*
 * S52.3 axes (P51-P53.agc, Comanche p. 780 / Luminary p. 954) -- nominal
 * IMU orientation from the state (r, v at any common scales):
 *   CSM:  ZSMD = UNIT(-R), YSMD = UNIT(V x R), XSMD = UNIT(YSMD x ZSMD)
 *   LM:   XSMD = UNIT(R),  YSMD = UNIT(V x R), ZSMD = UNIT(XSMD x YSMD)
 */
agc_status_t agc_align_nominal_orientation_axes(agc_interp_t *ip, agc_vec_t r,
                                                agc_vec_t v, agc_mat_t *smd)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(smd != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        smd->row[2] = agc_vec_unit(ip, agc_vec_neg(r), NULL, NULL);
        smd->row[1] = agc_vec_unit(ip, agc_vec_cross(ip, v, r), NULL, NULL);
        smd->row[0] = agc_vec_unit(
            ip, agc_vec_cross(ip, smd->row[1], smd->row[2]), NULL, NULL);
    }
    else
    {
        smd->row[0] = agc_vec_unit(ip, r, NULL, NULL);
        smd->row[1] = agc_vec_unit(ip, agc_vec_cross(ip, v, r), NULL, NULL);
        smd->row[2] = agc_vec_unit(
            ip, agc_vec_cross(ip, smd->row[0], smd->row[1]), NULL, NULL);
    }
    return AGC_STATUS_OK;
}

/*
 * S52.3 (P51-P53.agc, Comanche p. 780 / Luminary p. 954, label S52.3) --
 * nominal IMU orientation at time TALIGN (cs B28): CSMPREC (CM) or
 * LEMCONIC (LM) to TALIGN, then the axes from RATT, VATT.
 */
agc_status_t agc_align_nominal_orientation(agc_interp_t *ip, agc_dp_t talign,
                                           agc_job_id_t resume_job,
                                           agc_mat_t   *smd)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(smd != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_integ_out_t    o;
    const bool         csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    const agc_status_t st  = align_integ(
         ip, csm ? ALIGN_CSMPREC : ALIGN_LEMCONIC, talign, resume_job, &o);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    return agc_align_nominal_orientation_axes(ip, o.ratt, o.vatt, smd);
}

/*
 * LSORIENT (P51-P53.agc, Luminary p. 983) and the tail of P52LS
 * (Comanche p. 743) -- landing-site orientation from the site vector XSMD
 * and the CSM state:  ZSMD = UNIT((R x V) x XSMD), YSMD = UNIT(ZSMD x XSMD).
 * The LM passes RRECTCSM/VRECTCSM, the CSM its CSMPREC RATT/VATT.
 */
agc_status_t agc_align_lsorient(agc_interp_t *ip, agc_vec_t xsmd,
                                agc_vec_t rcsm, agc_vec_t vcsm, agc_mat_t *smd)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(smd != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_vec_t h = agc_vec_cross(ip, rcsm, vcsm);
    smd->row[0]       = xsmd;
    smd->row[2] = agc_vec_unit(ip, agc_vec_cross(ip, h, xsmd), NULL, NULL);
    smd->row[1] =
        agc_vec_unit(ip, agc_vec_cross(ip, smd->row[2], xsmd), NULL, NULL);
    return AGC_STATUS_OK;
}

/* A CDU word read as the 1's complement SP the interpreter DLOAD sees. */
static int32_t ones_value(uint16_t w)
{
    const uint16_t m = (uint16_t) (w & CDU_WORD_MASK);
    if (!AGC_ASSERT(w <= CDU_WORD_MASK))
    {
        return 0; /* no valid result */
    }
    if ((m & CDU_SIGN_BIT) != 0u)
    {
        return (int32_t) m - CDU_ONES_NEG;
    }
    if (!AGC_ASSERT(m < CDU_SIGN_BIT))
    {
        return 0; /* no valid result */
    }
    return (int32_t) m;
}

/*
 * CALOOP (P51-P53.agc, Comanche p. 761 / Luminary p. 947, CAL53A) -- coarse
 * alignment is needed when any axis differs from THETAD by at least 1 deg
 * and by less than 359 deg (DEGREE1 = 46, DEG359 = 16338 at CDU/2 scale).
 * DEVIATION: the DLOAD* of two SP words picks up THETAD+n+1 / the next
 * VAC word as a low order part (< 1 count); it is ignored here.
 */
bool agc_align_coarse_needed(const uint16_t thetad[3], const uint16_t cdu[3])
{
    if (!AGC_ASSERT(thetad != NULL) || !AGC_ASSERT(cdu != NULL))
    {
        return true;
    }
    bool coarse = false;
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        int32_t diff = ones_value(cdu[i]) - ones_value(thetad[i]);
        if (diff < 0)
        {
            diff = -diff; /* ABS                     */
        }
        if ((diff >= DEGREE1_X2) && (diff < DEG359_X2))
        {
            coarse = true; /* COARFINE                */
        }
    }
    return coarse;
}

/*
 * GYCRS (Comanche p. 741) / GYCOARS (Luminary p. 931/944) -- desired SM
 * referred to the present SM for gyro-torque coarse alignment:
 *   XDC = UNIT(REFSMMAT XSMD), YDC = UNIT(REFSMMAT YSMD), ZDC = UNIT(REFSMMAT ZSMD)
 */
agc_status_t agc_align_gyro_coarse_dc(agc_interp_t *ip, const agc_mat_t *smd,
                                      const agc_mat_t *refsmmat, agc_mat_t *xdc)
{
    if (!AGC_ASSERT((ip != NULL) && (smd != NULL)) ||
        !AGC_ASSERT((refsmmat != NULL) && (xdc != NULL)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        xdc->row[i] = agc_vec_unit(ip, agc_mat_vec(ip, refsmmat, smd->row[i]),
                                   NULL, NULL);
    }
    return AGC_STATUS_OK;
}
