/**********************************************************************
 * FILE NAME: s_band_antenna.c
 *
 * PURPOSE:
 *   R05, the LM steerable S-band antenna pitch and yaw angles (V64).  The
 *   earth direction UE is taken to navigation-base axes (*SMNB*), then
 *   rotated -45 degrees about Z into antenna-base axes: x' = (x +
 *   y)/SQRT2, y' = (y - x)/SQRT2.  PITCHANG = ASIN(SGN(X.UY)|X|) with X =
 *   UZ x URP, adjusted to 180 - pitch when URP.UZ < 0; YAWANG =
 *   ASIN(SGN(YAW.Z)|Z|) with Z = UR x URP and YAW = UX cos(pitch) - UZ
 *   sin(pitch).  Angles in revs B0 (noun 51).
 *
 * AGC SOURCE:
 *   Luminary099 S-BAND_ANTENNA_FOR_LM.agc pp. 486-489 (SBANDANT .. SBANDEX)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   PITCHANG, YAWANG agc_dp_t O revs 2^0       this file (N51)
 *   EXTVBACT      uint16_t  I    bits           bit 5: R05 display on
 *   REFSMMAT      agc_mat_t I    half-unit      agc_erase_refsmmat()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_*            DSKY displays, astronaut returns (dsky.h)
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_exec_prio_change  executive: jobs, VAC areas, sleep / wake (exec.h)
 *   agc_imu_status_check  IMU mode switching, CALCGA / CALCGTA / AXISGEN, gyro torquing (imu.h)
 *   agc_guid_*            CDUTRIG, *NBSM* / *SMNB*, PIPASR (guidance.h)
 *   agc_lsposition        LSPOS (auxiliary.h)
 *   agc_mpac_*            interpretive instructions on MPAC words (interp_words.h)
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
 *
 * NOTES:
 *   ABVAL SIGN ASIN on the MPAC words (finding W3, fixed).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/interp_words.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define REMDIST           ((agc_dp_t) 192201000) /* mean earth-moon distance B-29 m */
#define ONEOVSQRT2        ((agc_dp_t) 189812531) /* 1OVSQRT2 .7071067815            */
#define VN_ANTENNA_ANGLES ALIGN_VN(6, 51)        /* VN 0651                         */
#define BLANK_R3          ((uint16_t) 04)        /* BIT3                            */
#define EXTVB_BIT5        ((uint16_t) 020)       /* EXTVBACT bit 5: R04 / S-band display in use */
#define PRIO5             ((agc_prio_t) 05)      /* PRIO5 */
#define PRIO4             ((agc_prio_t) 04)      /* PRIO4 */

static agc_dp_t pitchang; /* PITCHANG (= ALPHASB): N51 R1, revs 2^0 */
static agc_dp_t yawang;   /* YAWANG (= BETASB): N51 R2, revs 2^0 */

/**********************************************************************
 * AGC: none (accessor of PITCHANG)
 *
 * PITCHANG, the LM S-band pitch angle for N51.
 *
 * FUNCTION NAME: agc_sband_pitchang
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- PITCHANG, revs 2^0
 **********************************************************************/
agc_dp_t agc_sband_pitchang(void)
{
    return (pitchang);
}

/**********************************************************************
 * AGC: none (accessor of YAWANG)
 *
 * YAWANG, the LM S-band yaw angle for N51.
 *
 * FUNCTION NAME: agc_sband_yawang
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- YAWANG, revs 2^0
 **********************************************************************/
agc_dp_t agc_sband_yawang(void)
{
    return (yawang);
}

/**********************************************************************
 * AGC: ABVAL (Luminary099 INTERPRETER.agc p. 1075)
 *
 * ABVAL SIGN ASIN: ASIN(SGN(s) |v|).  ABVAL is VSQSUB falling into SQRT,
 * which leaves MPAC +2; SIGN complements all three words and ARCSIN's
 * SQRTSUB reads them, so the chain runs on the MPAC words.
 *
 * FUNCTION NAME: signed_asin
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   v         agc_vec_t       I    the vector whose length is taken,
 *                                  half-unit (2^1)
 *   s         agc_dp_t        I    the sign (SIGN operand), any scale
 *
 * RETURNS: agc_dp_t -- ASIN(SIGN(|v|, s)), revs 2^0
 **********************************************************************/
static agc_dp_t signed_asin(agc_interp_t *ip, agc_vec_t v, agc_dp_t s)
{
    const agc_tp_words_t m = agc_mpac_sign(agc_mpac_sqrt(agc_mpac_vsq(ip, v)), agc_dp_to_words(s)); /* the mark */

    return (agc_dp_from_words(agc_mpac_store(agc_mpac_asin(ip, m))));
}

/**********************************************************************
 * AGC: *SMNB* (Luminary099 POWERED_FLIGHT_SUBROUTINES.agc p. 1263)
 *
 * The angles for r_nb, the unit LM-to-earth vector in NB axes (half-unit,
 * the *SMNB* output stored in RLM).  Outputs revs.
 *
 * FUNCTION NAME: agc_sband_lm_angles
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   r_nb      agc_vec_t       I    the earth unit vector UE in nav-base
 *                                  axes, half-unit (2^1)
 *   pitch     agc_dp_t *      O    antenna pitch, revs 2^0
 *   yaw       agc_dp_t *      O    antenna yaw, revs 2^0
 *
 * RETURNS: none
 **********************************************************************/
void agc_sband_lm_angles(agc_interp_t *ip, agc_vec_t r_nb, agc_dp_t *pitch, agc_dp_t *yaw)
{
    const agc_mat_t u = agc_mat_identity_half(); /* HIUNITX,Y,Z  */

    *pitch            = 0;      /* HI6ZEROS     */
    *yaw              = 0;
    agc_vec_t      r  = r_nb;   /* UE in the antenna base (45 deg about Z), half-unit (2^1) */
    const agc_dp_t y0 = r.c[1]; /* UE.Y before the turn, scale 2^1 */

    r.c[1]              = agc_dp_mul_trunc(agc_dp_sub(ip, y0, r.c[0]), ONEOVSQRT2); /* DMP */
    r.c[0]              = agc_dp_mul_trunc(agc_dp_add(ip, y0, r.c[0]), ONEOVSQRT2);
    const agc_vec_t ur  = agc_vec_unit(ip, r, NULL, NULL);                          /* UNIT(R), 0D, half-unit (2^1) */
    const agc_vec_t py  = agc_vec_shl(ip, agc_vec_proj(ip, r, u.row[1]), 2);        /* the Y projection of R VSL2, half-unit (2^1) */
    agc_vec_t       urp = agc_vec_sub(ip, r, py);                                   /* R - its Y projection, then UNIT, half-unit (2^1) */

    align_clear_ovf(ip);
    urp = agc_vec_unit(ip, urp, NULL, NULL); /* COVCNV       */

    if (!agc_interp_test_ovf(ip)) /* BOV SBANDEX  */
    {
        const agc_vec_t x = agc_vec_neg(agc_vec_shl(ip, agc_vec_cross(ip, urp, u.row[2]), 1)); /* -(URP x UNITZ) VSL1, half-unit (2^1) */

        *pitch = signed_asin(ip, x, agc_vec_dot(ip, x, u.row[1]));

        if (agc_vec_dot(ip, urp, u.row[2]) < 0)
        { /* BPL NOADJUST */
            *pitch = agc_dp_sub(ip, AGC_DP_HALF, *pitch); /* HIDPHALF - p */
        }

        const agc_vec_t z  = agc_vec_shl(ip, agc_vec_cross(ip, ur, urp), 1);  /* (UR x URP) VSL1, half-unit (2^1) */
        const agc_vec_t a  = agc_vec_scale(ip, u.row[2], agc_dp_sin(*pitch)); /* UNITZ sin PITCH, scale 2^2 */
        const agc_vec_t b  = agc_vec_scale(ip, u.row[0], agc_dp_cos(*pitch)); /* UNITX cos PITCH, scale 2^2 */
        const agc_dp_t  yz = agc_vec_dot(ip, agc_vec_sub(ip, b, a), z);       /* YAW . Z, scale 2^2 */

        *yaw = signed_asin(ip, z, yz);
    }
}

/**********************************************************************
 * AGC: RATT (Luminary099 ORBITAL_INTEGRATION.agc p. 1247)
 *
 * UE = -UNIT(RATT) (earth sphere) or -UNIT(REMDIST VMOON SL1 + RATT).
 *
 * FUNCTION NAME: earth_unit
 *
 * ARGUMENTS:
 *   Argument  Type                     I/O  Description (scale)
 *   --------  -----------------------  ---  -----------------------------
 *   ip        agc_interp_t *           IO   interpreter state (OVFIND, MPAC
 *                                           words)
 *   o         const agc_integ_out_t *  I    LEMCONIC output (RATT m 2^29 /
 *                                           2^27, TAT, X2)
 *   ue        agc_vec_t *              O    UE = -UNIT(earth vector),
 *                                           half-unit (2^1)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor; otherwise the status of the call that failed
 *          (executive, display, IMU, integration or an input check)
 **********************************************************************/
static agc_status_t earth_unit(agc_interp_t *ip, const agc_integ_out_t *o, agc_vec_t *ue)
{
    agc_vec_t r = o->ratt; /* RATT (earth) or REMDIST VMOON SL1 + RATT (moon), m 2^29 / 2^27 */

    if (o->x2 != 0)
    { /* CONV3 LUNPOS */
        agc_vec_t          vmoon;                                          /* LSPOS moon unit vector, scale 2^1 */
        agc_vec_t          vsun;                                           /* LSPOS sun unit vector, scale 2^1 */
        const agc_status_t st = agc_lsposition(ip, o->tat, &vmoon, &vsun); /* status of the call */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        r = agc_vec_add(ip, agc_vec_shl(ip, agc_vec_scale(ip, vmoon, REMDIST), 1), o->ratt);
    }

    *ue = agc_vec_neg(agc_vec_unit(ip, r, NULL, NULL)); /* CONV5        */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SBANDANT +2 (p. 486-488): LEMCONIC to now, UE, CDUTRIG, SM, *SMNB*.
 *
 * FUNCTION NAME: agc_job_sbandant_lm_recycle
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
agc_status_t agc_job_sbandant_lm_recycle(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_integ_out_t o;                                                                                        /* integration output (RATT, VATT) */
    agc_vec_t       ue = agc_vec_zero();                                                                      /* UE in SM, then NB, half-unit (2^1) */
    agc_cdutrig_t   trig;                                                                                     /* CDUTRIG sines and cosines */
    agc_status_t    st = align_integ(&ip, ALIGN_LEMCONIC, align_loadtime(), AGC_JOB_SBANDANT_LM_RECYCLE, &o); /* status of the call */

    if (st == AGC_STATUS_OK)
    {
        st = earth_unit(&ip, &o, &ue);
    }

    if (st == AGC_STATUS_OK)
    {
        agc_guid_cdutrig(&trig); /* CDUTRIG      */
    }

    if (st == AGC_STATUS_OK)
    {
        const agc_mat_t refsmmat = agc_erase_refsmmat(); /* REFSMMAT, half-unit rows */

        ue = agc_vec_shl(&ip, agc_mat_vec(&ip, &refsmmat, ue), 1);
        agc_guid_smnb(&trig, &ue); /* *SMNB*       */
    }

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st); /* INTSTALL: the job sleeps */
    }

    agc_sband_lm_angles(&ip, ue, &pitchang, &yawang);

    /* SBANDEX: display VN_ANTENNA_ANGLES at PRIO5 while EXTVBACT bit 5 is on. */
    if ((agc_erase_extvbact() & EXTVB_BIT5) == 0u)
    {
        return (agc_dsky_endmark());
    }

    agc_exec_prio_change(PRIO5); /* PRIOCHNG       */
    st = align_flash(ctx, AGC_DSKY_GOMARKFR, VN_ANTENNA_ANGLES, AGC_JOB_SBAND_B5OFF, AGC_JOB_SBAND_B5OFF, AGC_JOB_ENDOFJOB);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = agc_dsky_blanket(BLANK_R3);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_exec_prio_change(PRIO4); /* PRIOCHNG: on to SBANDANT +2 */

    return (align_goto(AGC_JOB_SBANDANT_LM_RECYCLE)); /* TC SBANDANT+2 */
} /* agc_job_sbandant_lm_recycle */

/**********************************************************************
 * AGC: SBANDANT (Luminary099 S-BAND_ANTENNA_FOR_LM.agc p. 486)
 *
 * SBANDANT (V64 E): R02BOTH, then SBANDANT +2.
 *
 * FUNCTION NAME: agc_job_sbandant_lm
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
agc_status_t agc_job_sbandant_lm(agc_job_ctx_t *ctx)
{
    const agc_status_t st = agc_imu_status_check(); /* status of the call */

    if (st == AGC_STATUS_POODOO)
    {
        return (AGC_STATUS_OK); /* alarm 220/210, P00 started */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_sbandant_lm_recycle(ctx));
}
