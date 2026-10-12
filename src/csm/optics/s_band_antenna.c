/*
 * sband_cm.c -- S-BAND_ANTENNA_FOR_CM.agc (Comanche055 pp. 934-935):
 * R05, the S-band high-gain antenna pointing angles (V64).
 *
 * "THE S-BAND ANTENNA ROUTINE COMPUTES AND DISPLAYS THE PITCH AND YAW
 *  ANGLES OF THE HIGH GAIN ANTENNA FOR POINTING AT THE EARTH."
 * Pitch RHOSB = ACOS(UR . UZ) - 90 deg, yaw GAMMASB = angle of the X-Y
 * projection of UR from +X, 0..360 deg (revs B0, noun 51).
 */
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

#define HIDP1_4           AGC_DP_QUARTER   /* HIDP1/4 .25 rev = 90 deg      */
#define VN_ANTENNA_ANGLES ALIGN_VN(6, 51)  /* VN 0651                         */
#define BLANK_R3          ((uint16_t) 04)  /* BIT3                           */
#define DELAY_1CS         ((uint16_t) 1)   /* BIT1: minimum DELAYJOB         */
#define EXTVB_BIT5        ((uint16_t) 020) /* EXTVBACT bit 5: R05 running    */

static agc_dp_t rhosb;   /* RHOSB  (= RANGE) B(2)DSP N51 pitch, revs     */
static agc_dp_t gammasb; /* GAMMASB (= RHOSB +2) N51 yaw, revs           */

agc_dp_t agc_sband_rhosb(void)
{
    return (rhosb);
}

agc_dp_t agc_sband_gammasb(void)
{
    return (gammasb);
}

/*
 * EISOI +2 .. NOADJUST (p. 934-935) -- the angles for r_nb, the vector
 * from the CSM to the earth in navigation-base axes (2^29 m):
 *   UR = UNIT(R);  URP = UNIT(R - (R.UZ)UZ)
 *   yaw = ACOS(URP.UX), 360 - yaw when URP.UY < 0 (0 if URP is null)
 *   pitch = ACOS(UR.UZ) - 90 deg
 */
agc_status_t agc_sband_cm_angles(agc_interp_t *ip, agc_vec_t r_nb, agc_dp_t *pitch, agc_dp_t *yaw)
{
    const agc_mat_t hiunit = agc_mat_identity_half(); /* HIUNITX,Y,Z  */

    *yaw                = 0;                                  /* HI6ZEROS     */
    const agc_vec_t ur  = agc_vec_unit(ip, r_nb, NULL, NULL); /* 8D */
    const agc_vec_t pz  = agc_vec_shl(ip, agc_vec_proj(ip, r_nb, hiunit.row[2]), 2);
    agc_vec_t       urp = agc_vec_sub(ip, r_nb, pz);          /* BVSU R       */

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

    return (AGC_STATUS_OK);
}

/* The CSM-to-earth vector in SM: -RATT (earth sphere) or -(REM + RATT). */
static agc_status_t earth_vector_sm(agc_interp_t *ip, const agc_integ_out_t *o, agc_vec_t *r_sm)
{
    agc_vec_t r = agc_vec_neg(o->ratt); /* EISOI: -RCM  */

    if (o->x2 != 0)
    { /* moon: LUNPOS */
        agc_navaux_padload_t pad;
        agc_ephem_csm_t      eph;
        agc_vec_t            rem;
        agc_status_t         st = agc_navaux_get_padload(&pad);

        if (st == AGC_STATUS_OK)
        {
            st = agc_ephem_get_csm_data(&eph);
        }

        if (st == AGC_STATUS_OK)
        {
            st = agc_ephem_csm_lunpos(ip, &eph, pad.tephem, o->tat, &rem);
        }

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        r = agc_vec_neg(agc_vec_add(ip, rem, o->ratt)); /* VAD VCOMP    */
    }

    const agc_mat_t refsmmat = agc_erase_refsmmat();

    *r_sm = agc_vec_shl(ip, agc_mat_vec(ip, &refsmmat, r), 1); /* MXV VSL1 */

    return (AGC_STATUS_OK);
}

/*
 * SBANDANT +2 (p. 934): LOADTIME, CDUTRIG, CSMCONIC to now; the earth
 * vector in SM, *SMNB*, the angles; then the display loop.
 */
agc_status_t agc_job_sbandant_cm_recycle(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_cdutrig_t   trig;
    agc_integ_out_t o;
    const agc_dp_t  now = align_loadtime();

    agc_guid_cdutrig(&trig); /* CDUTRIG      */
    agc_status_t st = align_integ(&ip, ALIGN_CSMCONIC, now, AGC_JOB_SBANDANT_CM_RECYCLE, &o);
    agc_vec_t    r;

    if (st == AGC_STATUS_OK)
    {
        st = earth_vector_sm(&ip, &o, &r);
    }

    if (st == AGC_STATUS_OK)
    {
        agc_guid_smnb(&trig, &r); /* *SMNB*       */
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_sband_cm_angles(&ip, r, &rhosb, &gammasb);
    }

    if (st != AGC_STATUS_OK)
    {
        return ((st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st);
    }

    /* Display VN_ANTENNA_ANGLES while EXTVBACT bit 5 is on (p. 935). */
    if ((agc_erase_extvbact() & EXTVB_BIT5) == 0u)
    {
        return (agc_dsky_endmark()); /* BZF ENDEXT   */
    }

    agc_status_t st_angles = align_flash(ctx, AGC_DSKY_GOMARKFR, VN_ANTENNA_ANGLES, AGC_JOB_SBAND_B5OFF, AGC_JOB_SBAND_B5OFF, AGC_JOB_ENDOFJOB);

    if (st_angles != AGC_STATUS_OK)
    {
        return (st_angles);
    }

    st_angles = agc_dsky_blanket(BLANK_R3); /* immediate ret*/

    if (st_angles != AGC_STATUS_OK)
    {
        return (st_angles);
    }

    return (agc_svc_delayjob(DELAY_1CS, AGC_JOB_SBANDANT_CM_RECYCLE));
}

/* SBANDANT (V64 E): R02BOTH, then SBANDANT +2. */
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

/* TC B5OFF (terminate / proceed of the N51 display), both vehicles. */
agc_status_t agc_job_sband_b5off(void)
{
    return (agc_svc_b5off());
}
