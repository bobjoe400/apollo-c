/**********************************************************************
 * FILE NAME: s_band_antenna.c
 *
 * PURPOSE:
 *   R05, the CM S-band high-gain antenna pointing angles (V64): "THE
 *   S-BAND ANTENNA ROUTINE COMPUTES AND DISPLAYS THE PITCH AND YAW ANGLES
 *   OF THE HIGH GAIN ANTENNA FOR POINTING AT THE EARTH."  Pitch RHOSB =
 *   ACOS(UR . UZ) - 90 deg, yaw GAMMASB = angle of the X-Y projection of
 *   UR from +X, 0..360 deg (revs B0, noun 51).
 *
 * AGC SOURCE:
 *   Comanche055 S-BAND_ANTENNA_FOR_CM.agc pp. 934-935 (SBANDANT .. NOADJUST)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   RHOSB, GAMMASB agc_dp_t O   revs 2^0       this file (N51)
 *   EXTVBACT      uint16_t  I    bits           bit 5: R05 display on
 *   REFSMMAT      agc_mat_t I    half-unit      agc_erase_refsmmat()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_*            DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_svc_*             service routines: DELAYJOB, flags, B5OFF (service.h)
 *   agc_imu_status_check  IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_guid_*            CDUTRIG, *NBSM* / *SMNB*, PIPASR (guidance.h)
 *   agc_navaux_get_padload  pad loads (auxiliary.h)
 *   agc_ephem_*           ephemeris, LUNPOS (auxiliary.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_mat_*             matrix arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_*          interpreter state: OVFIND (interp.h)
 *   agc_erase_*           erasable accessors (state.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>

#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define HIDP1_4           AGC_DP_QUARTER   /* HIDP1/4 .25 rev = 90 deg      */
#define VN_ANTENNA_ANGLES ALIGN_VN(6, 51)  /* VN 0651                         */
#define BLANK_R3          ((uint16_t) 04)  /* BIT3                           */
#define DELAY_1CS         ((uint16_t) 1)   /* BIT1: minimum DELAYJOB         */
#define EXTVB_BIT5        ((uint16_t) 020) /* EXTVBACT bit 5: R05 running    */

static agc_dp_t rhosb;   /* RHOSB (= RANGE): N51 pitch, revs 2^0 */
static agc_dp_t gammasb; /* GAMMASB (= RHOSB +2): N51 yaw, revs 2^0 */

/**********************************************************************
 * AGC: none (accessor of RHOSB)
 *
 * RHOSB, the CM S-band pitch angle for N51.
 *
 * FUNCTION NAME: agc_sband_rhosb
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- RHOSB, revs 2^0
 **********************************************************************/
agc_dp_t agc_sband_rhosb(void)
{
    return (rhosb);
}

/**********************************************************************
 * AGC: none (accessor of GAMMASB)
 *
 * GAMMASB, the CM S-band yaw angle for N51.
 *
 * FUNCTION NAME: agc_sband_gammasb
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- GAMMASB, revs 2^0
 **********************************************************************/
agc_dp_t agc_sband_gammasb(void)
{
    return (gammasb);
}

/**********************************************************************
 * EISOI +2 .. NOADJUST (p. 934-935) -- the angles for r_nb, the vector from
 * the CSM to the earth in navigation-base axes (2^29 m): UR = UNIT(R);  URP
 * = UNIT(R - (R.UZ)UZ) yaw = ACOS(URP.UX), 360 - yaw when URP.UY < 0 (0 if
 * URP is null) pitch = ACOS(UR.UZ) - 90 deg
 *
 * FUNCTION NAME: agc_sband_cm_angles
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   r_nb      agc_vec_t       I    the earth vector in nav-base axes (any
 *                                  scale; unitized)
 *   pitch     agc_dp_t *      O    antenna pitch, revs 2^0
 *   yaw       agc_dp_t *      O    antenna yaw, revs 2^0
 *
 * RETURNS: none
 **********************************************************************/
void agc_sband_cm_angles(agc_interp_t *ip, agc_vec_t r_nb, agc_dp_t *pitch, agc_dp_t *yaw)
{
    const agc_mat_t hiunit = agc_mat_identity_half(); /* HIUNITX,Y,Z  */

    *yaw                = 0;                                                         /* HI6ZEROS     */
    const agc_vec_t ur  = agc_vec_unit(ip, r_nb, NULL, NULL);                        /* UNIT(R), 8D, half-unit (2^1) */
    const agc_vec_t pz  = agc_vec_shl(ip, agc_vec_proj(ip, r_nb, hiunit.row[2]), 2); /* the Z projection of R VSL2, scale as R */
    agc_vec_t       urp = agc_vec_sub(ip, r_nb, pz);                                 /* R - its Z projection (BVSU R), then UNIT, half-unit (2^1) */

    align_clear_ovf(ip);                     /* BOV COVCNV   */
    urp = agc_vec_unit(ip, urp, NULL, NULL); /* COVCNV UNIT  */

    if (!agc_interp_test_ovf(ip))
    { /* BOV NOADJUST */
        *yaw              = agc_dp_acos(ip, align_dot_shl(ip, urp, hiunit.row[0], 1));
        const agc_dp_t uy = align_dot_shl(ip, urp, hiunit.row[1], 1); /* B0 */

        if (uy < 0)
        { /* BPL NOADJUST */
            *yaw = agc_dp_sub(ip, AGC_DP_MAX, *yaw); /* DPPOSMAX - yaw */
        }
    }

    const agc_dp_t uz = align_dot_shl(ip, ur, hiunit.row[2], 1); /* B0 */

    *pitch = agc_dp_sub(ip, agc_dp_acos(ip, uz), HIDP1_4); /* DSU HIDP1/4 */
}

/**********************************************************************
 * AGC: REM (Comanche055 LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc p. 787)
 *
 * The CSM-to-earth vector in SM: -RATT (earth sphere) or -(REM + RATT).
 *
 * FUNCTION NAME: earth_vector_sm
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------------
 *   ip        agc_interp_t *           IO   interpreter state (OVFIND, MPAC
 *                                           words)
 *   o         const agc_integ_out_t *  I    CSMCONIC output (RATT m 2^29 /
 *                                           2^27, TAT, X2)
 *   r_sm      agc_vec_t *              O    the CSM-to-earth vector in SM
 *                                           axes, VSL1 (m 2^28 earth
 *                                           sphere)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t earth_vector_sm(agc_interp_t *ip, const agc_integ_out_t *o, agc_vec_t *r_sm)
{
    agc_vec_t r = agc_vec_neg(o->ratt); /* -RCM (earth) or -(REM + RCM) (moon), m 2^29 / 2^27 */

    if (o->x2 != 0)
    { /* moon: LUNPOS */
        agc_navaux_padload_t pad; /* pad loads (ephemeris, RLS) */
        agc_ephem_csm_t      eph; /* the CSM ephemeris pad words */
        agc_vec_t            rem; /* LUNPOS: moon from earth, m 2^29 */

        agc_navaux_get_padload(&pad);
        agc_ephem_get_csm_data(&eph);
        agc_status_t st = agc_ephem_csm_lunpos(ip, &eph, pad.tephem, o->tat, &rem); /* status of the call */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        r = agc_vec_neg(agc_vec_add(ip, rem, o->ratt)); /* VAD VCOMP    */
    }

    const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */

    *r_sm = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, r), 1); /* MXV VSL1 */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SBANDANT +2 (p. 934): LOADTIME, CDUTRIG, CSMCONIC to now; the earth
 * vector in SM, *SMNB*, the angles; then the display loop.
 *
 * FUNCTION NAME: agc_job_sbandant_cm_recycle
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_sbandant_cm_recycle(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_cdutrig_t   trig;                   /* CDUTRIG sines and cosines */
    agc_integ_out_t o;                      /* integration output (RATT, VATT) */
    const agc_dp_t  now = align_loadtime(); /* TIME2/1, cs 2^28 */

    agc_guid_cdutrig(&trig);                                                                  /* CDUTRIG      */
    agc_status_t st = align_integ(&ip, ALIGN_CSMCONIC, now, AGC_JOB_SBANDANT_CM_RECYCLE, &o); /* status of the call */
    agc_vec_t    r;                                                                           /* the earth vector in SM, then NB, scale 2^28 m */

    if (st == AGC_STATUS_OK)
    {
        st = earth_vector_sm(&ip, &o, &r);
    }

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st); /* INTSTALL: the job sleeps */
    }

    agc_guid_smnb(&trig, &r); /* *SMNB*       */
    agc_sband_cm_angles(&ip, r, &rhosb, &gammasb);

    /* Display VN_ANTENNA_ANGLES while EXTVBACT bit 5 is on (p. 935). */
    if ((agc_erase_extvbact() & EXTVB_BIT5) == 0u)
    {
        return (agc_dsky_endmark()); /* BZF ENDEXT   */
    }

    st = align_flash(ctx, AGC_DSKY_GOMARKFR, VN_ANTENNA_ANGLES, AGC_JOB_SBAND_B5OFF, AGC_JOB_SBAND_B5OFF, AGC_JOB_ENDOFJOB);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_dsky_blanket(BLANK_R3); /* immediate ret*/

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_svc_delayjob(DELAY_1CS, AGC_JOB_SBANDANT_CM_RECYCLE));
} /* agc_job_sbandant_cm_recycle */

/**********************************************************************
 * AGC: SBANDANT (Comanche055 S-BAND_ANTENNA_FOR_CM.agc p. 934)
 *
 * SBANDANT (V64 E): R02BOTH, then SBANDANT +2.
 *
 * FUNCTION NAME: agc_job_sbandant_cm
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------------
 *   ctx       agc_job_ctx_t *  I    the running job (display continuations,
 *                                   priority)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
agc_status_t agc_job_sbandant_cm(agc_job_ctx_t *ctx)
{
    const agc_status_t st = agc_imu_status_check(); /* R02BOTH      */

    if (st == AGC_STATUS_POODOO)
    {
        return (AGC_STATUS_OK); /* alarm 220/210, P00 started */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_sbandant_cm_recycle(ctx));
}

/**********************************************************************
 * AGC: TC (Comanche055 ERASABLE_ASSIGNMENTS.agc p. 85)
 *
 * TC B5OFF (terminate / proceed of the N51 display), both vehicles.
 *
 * FUNCTION NAME: agc_job_sband_b5off
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_job_sband_b5off(void)
{
    agc_svc_b5off(); /* B5OFF */
}
