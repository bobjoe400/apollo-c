/**********************************************************************
 * FILE NAME: orientation.c
 *
 * PURPOSE:
 *   The orientation computations of P52, P54 and P57: S52.2 gimbal angles
 *   for a desired stable member, S52.3 nominal orientation, the
 *   landing-site orientation (P52LS / LSORIENT), the CAL53A coarse-align
 *   decision (CALOOP) and the gyro-torque coarse-align matrix (GYCRS /
 *   GYCOARS).
 *
 * AGC SOURCE:
 *   Comanche055 P51-P53.agc pp. 761-765 (S52.2, S52.3, CALOOP, GYCRS)
 *   Luminary099 P51-P53.agc pp. 931, 947-948, 983 (S52.2, S52.3, CALOOP, GYCOARS, LSORIENT)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   XSM, XNB      agc_mat_t O    half-unit      outputs (agc_align_ram())
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_integ           alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_*           alignment routines of this lane (alignment.h)
 *   agc_imu_calcga        IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_mat_vec           matrix arithmetic of the interpreter (interp.h)
 *   agc_state_vehicle     vehicle state (state.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   CALOOP compares the CDU words; the AGC compares SR1 DPs whose minor
 *   words are the next THETAD / CDU words (a difference of exactly 92
 *   counts may decide differently).
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/programs/alignment.h"
#include "internal.h"

#define CDU_WORD_MASK 077777u /* 15-bit CDU word */
#define CDU_SIGN_BIT  040000u /* sign bit of a 15-bit word */
#define CDU_ONES_NEG  077777  /* 1's complement negative bias           */
#define DEGREE1_X2    92      /* DEGREE1 = DEC 46, 1 deg at CDU/2; x2   */
#define DEG359_X2     32676   /* DEG359 = DEC 16338, 359 deg at CDU/2; x2 */

/**********************************************************************
 * S52.2 (P51-P53.agc, Comanche p. 774 / Luminary p. 953, label S52.2) --
 * compute the gimbal angles for the desired stable member XSMD.. and the
 * present vehicle attitude.  CDUTRIG + CALCSMSC give the nav base wrt the
 * present SM; each axis is taken to the reference frame (VXM REFSMMAT,
 * UNIT); XSM = XSMD (MATMOVE); CALCGA gives THETAD. smd      XSMD,YSMD,ZSMD
 * half-unit, reference coordinates cdu      CDUX,CDUY,CDUZ   cdu xsm
 * XSM.. output     half-unit, reference xnb      XNB.. output
 * half-unit, reference thetad   THETAD +0..+2    cdu (outer, inner, middle)
 *
 * FUNCTION NAME: agc_align_gimbal_angles
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     IO   interpreter state (OVFIND, MPAC
 *                                     words)
 *   smd       const agc_mat_t *  I    desired SM axes XSMD, YSMD, ZSMD wrt
 *                                     reference, half-unit rows
 *   refsmmat  const agc_mat_t *  I    REFSMMAT, half-unit rows (reference
 *                                     -> present SM)
 *   cdu       const uint16_t     I    CDU words X, Y, Z (2^15 = 1 rev)
 *   xsm       agc_mat_t *        O    XSM, YSM, ZSM: desired SM wrt present
 *                                     SM, half-unit rows
 *   xnb       agc_mat_t *        O    XNB, YNB, ZNB: nav base wrt present
 *                                     SM, half-unit rows
 *   thetad    uint16_t           O    gimbal angles X (outer), Y (inner), Z
 *                                     (middle), CDU words
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_align_gimbal_angles(agc_interp_t *ip, const agc_mat_t *smd, const agc_mat_t *refsmmat, const uint16_t cdu[3], agc_mat_t *xsm, agc_mat_t *xnb, uint16_t thetad[3])
{
    agc_cdutrig_t trig; /* CDUTRIG sines and cosines */

    const uint16_t yzx[3] = {cdu[1], cdu[2], cdu[0]}; /* CDUSPOT: CDUY, CDUZ, CDUX (AGC_TRIG_Y, _Z, _X) */

    agc_imu_cdtrigs(&trig, yzx);                              /* CDUTRIG (CD*TR*GS) */
    const agc_status_t st = agc_imu_calcsmsc(ip, &trig, xnb); /* CALCSMSC       */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* S52.2A: X1 = 18, 12, 6 selects XNB, YNB, ZNB in turn (TIX,1 by 6). */
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        xnb->row[i] = agc_vec_unit(ip, agc_vec_mat(ip, xnb->row[i], refsmmat), NULL, NULL);
    }

    *xsm = *smd; /* S52.2.1 MATMOVE        */

    return (agc_imu_calcga(ip, xnb, xsm, thetad)); /* CALCGA -> THETAD       */
}

/**********************************************************************
 * S52.3 axes (P51-P53.agc, Comanche p. 780 / Luminary p. 954) -- nominal
 * IMU orientation from the state (r, v at any common scales): CSM:  ZSMD =
 * UNIT(-R), YSMD = UNIT(V x R), XSMD = UNIT(YSMD x ZSMD) LM:   XSMD =
 * UNIT(R),  YSMD = UNIT(V x R), ZSMD = UNIT(XSMD x YSMD)
 *
 * FUNCTION NAME: agc_align_nominal_orientation_axes
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   r         agc_vec_t       I    vehicle position RATT, m 2^29 (earth) or
 *                                  2^27 (moon)
 *   v         agc_vec_t       I    vehicle velocity VATT, m/cs 2^7 (earth)
 *                                  or 2^5 (moon)
 *   smd       agc_mat_t *     I    desired SM axes XSMD, YSMD, ZSMD wrt
 *                                  reference, half-unit rows
 *
 * RETURNS: none
 **********************************************************************/
void agc_align_nominal_orientation_axes(agc_interp_t *ip, agc_vec_t r, agc_vec_t v, agc_mat_t *smd)
{
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        smd->row[2] = agc_vec_unit(ip, agc_vec_neg(r), NULL, NULL);
        smd->row[1] = agc_vec_unit(ip, agc_vec_cross(ip, v, r), NULL, NULL);
        smd->row[0] = agc_vec_unit(ip, agc_vec_cross(ip, smd->row[1], smd->row[2]), NULL, NULL);
    }
    else
    {
        smd->row[0] = agc_vec_unit(ip, r, NULL, NULL);
        smd->row[1] = agc_vec_unit(ip, agc_vec_cross(ip, v, r), NULL, NULL);
        smd->row[2] = agc_vec_unit(ip, agc_vec_cross(ip, smd->row[0], smd->row[1]), NULL, NULL);
    }
}

/**********************************************************************
 * S52.3 (P51-P53.agc, Comanche p. 780 / Luminary p. 954, label S52.3) --
 * nominal IMU orientation at time TALIGN (cs B28): CSMPREC (CM) or LEMCONIC
 * (LM) to TALIGN, then the axes from RATT, VATT.
 *
 * FUNCTION NAME: agc_align_nominal_orientation
 *
 * ARGUMENTS:
 *   Argument    Type            I/O  Description (scale)
 *   ----------  --------------  ---  ------------------------------------
 *   ip          agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   talign      agc_dp_t        I    alignment time (DSPTEM1 / TALIGN), cs
 *                                    2^28
 *   resume_job  agc_job_id_t    I    job resumed after an integration wait
 *                                    (INTSTALL)
 *   smd         agc_mat_t *     I    desired SM axes XSMD, YSMD, ZSMD wrt
 *                                    reference, half-unit rows
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_align_nominal_orientation(agc_interp_t *ip, agc_dp_t talign, agc_job_id_t resume_job, agc_mat_t *smd)
{
    agc_integ_out_t    o;                                                                                   /* integration output (RATT, VATT) */
    const bool         csm = (agc_state_vehicle() == AGC_VEHICLE_CSM);                                      /* running on the CSM */
    const agc_status_t st  = align_integ(ip, csm ? ALIGN_CSMPREC : ALIGN_LEMCONIC, talign, resume_job, &o); /* per vehicle */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_align_nominal_orientation_axes(ip, o.ratt, o.vatt, smd);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * LSORIENT (P51-P53.agc, Luminary p. 983) and the tail of P52LS (Comanche
 * p. 743) -- landing-site orientation from the site vector XSMD and the CSM
 * state:  ZSMD = UNIT((R x V) x XSMD), YSMD = UNIT(ZSMD x XSMD). The LM
 * passes RRECTCSM/VRECTCSM, the CSM its CSMPREC RATT/VATT.
 *
 * FUNCTION NAME: agc_align_lsorient
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   xsmd      agc_vec_t       I    XSMD = UNIT(landing site), half-unit
 *                                  (2^1)
 *   rcsm      agc_vec_t       I    CSM position (RRECTCSM / RATT), m 2^29
 *                                  or 2^27
 *   vcsm      agc_vec_t       I    CSM velocity (VRECTCSM / VATT), m/cs 2^7
 *                                  or 2^5
 *   smd       agc_mat_t *     I    desired SM axes XSMD, YSMD, ZSMD wrt
 *                                  reference, half-unit rows
 *
 * RETURNS: none
 **********************************************************************/
void agc_align_lsorient(agc_interp_t *ip, agc_vec_t xsmd, agc_vec_t rcsm, agc_vec_t vcsm, agc_mat_t *smd)
{
    const agc_vec_t h = agc_vec_cross(ip, rcsm, vcsm); /* RCSM x VCSM, orbital angular momentum direction, scale 2^36 (unnormalised) */

    smd->row[0] = xsmd;
    smd->row[2] = agc_vec_unit(ip, agc_vec_cross(ip, h, xsmd), NULL, NULL);
    smd->row[1] = agc_vec_unit(ip, agc_vec_cross(ip, smd->row[2], xsmd), NULL, NULL);
}

/**********************************************************************
 * AGC: DLOAD (Comanche055 INTERPRETER.agc p. 1107; Luminary099
 *      INTERPRETER.agc p. 1002)
 *
 * A CDU word read as the 1's complement SP the interpreter DLOAD sees.
 *
 * FUNCTION NAME: ones_value
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------------
 *   w         uint16_t  I    raw 15-bit 1s complement word
 *
 * RETURNS: int32_t -- the signed value of a 15-bit word (CDU counts, -0 as
 *          0)
 **********************************************************************/
static int32_t ones_value(uint16_t w)
{
    const uint16_t m = (uint16_t) (w & CDU_WORD_MASK); /* the mark */

    if ((m & CDU_SIGN_BIT) != 0u)
    {
        return ((int32_t) m - CDU_ONES_NEG);
    }

    return ((int32_t) m);
}

/**********************************************************************
 * CALOOP (P51-P53.agc, Comanche p. 761 / Luminary p. 947, CAL53A) -- coarse
 * alignment is needed when any axis differs from THETAD by at least 1 deg
 * and by less than 359 deg (DEGREE1 = 46, DEG359 = 16338 at CDU/2 scale).
 * DEVIATION: the DLOAD* of two SP words picks up THETAD+n+1 / the next VAC
 * word as a low order part (< 1 count); it is ignored here.
 *
 * FUNCTION NAME: agc_align_coarse_needed
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   thetad    const uint16_t  O    gimbal angles X (outer), Y (inner), Z
 *                                  (middle), CDU words
 *   cdu       const uint16_t  I    CDU words X, Y, Z (2^15 = 1 rev)
 *
 * RETURNS: bool -- true when some gimbal is 1 degree or more from THETAD
 *          (COARFINE)
 **********************************************************************/
bool agc_align_coarse_needed(const uint16_t thetad[3], const uint16_t cdu[3])
{
    bool coarse = false; /* a gimbal is out of the window */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        int32_t diff = ones_value(cdu[i]) - ones_value(thetad[i]); /* |CDU - THETAD|, CDU counts */

        if (diff < 0)
        {
            diff = -diff; /* ABS                     */
        }

        if ((diff >= DEGREE1_X2) && (diff < DEG359_X2))
        {
            coarse = true; /* COARFINE                */
        }
    }

    return (coarse);
}

/**********************************************************************
 * GYCRS (Comanche p. 741) / GYCOARS (Luminary p. 931/944) -- desired SM
 * referred to the present SM for gyro-torque coarse alignment: XDC =
 * UNIT(REFSMMAT XSMD), YDC = UNIT(REFSMMAT YSMD), ZDC = UNIT(REFSMMAT ZSMD)
 *
 * FUNCTION NAME: agc_align_gyro_coarse_dc
 *
 * ARGUMENTS:
 *   Argument  Type               I/O  Description (scale)
 *   --------  -----------------  ---  -----------------------------------
 *   ip        agc_interp_t *     IO   interpreter state (OVFIND, MPAC
 *                                     words)
 *   smd       const agc_mat_t *  I    desired SM axes XSMD, YSMD, ZSMD wrt
 *                                     reference, half-unit rows
 *   refsmmat  const agc_mat_t *  I    REFSMMAT, half-unit rows (reference
 *                                     -> present SM)
 *   xdc       agc_mat_t *        O    XDC, YDC, ZDC: desired SM wrt present
 *                                     SM, half-unit rows
 *
 * RETURNS: none
 **********************************************************************/
void agc_align_gyro_coarse_dc(agc_interp_t *ip, const agc_mat_t *smd, const agc_mat_t *refsmmat, agc_mat_t *xdc)
{
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        xdc->row[i] = agc_vec_unit(ip, agc_mat_vec(ip, refsmmat, smd->row[i]), NULL, NULL);
    }
}
