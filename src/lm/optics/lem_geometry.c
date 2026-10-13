/**********************************************************************
 * FILE NAME: lem_geometry.c
 *
 * PURPOSE:
 *   LEM_GEOMETRY: the rendezvous radar gimbal angles for a line of sight
 *   (RRANGLES) and the radar line of sight in navigation-base axes from
 *   its CDUs (RRNB, RRNBMPAC).  SVDWN2 / SVDWN1 of the same file are in
 *   src/nav/integration/svdwn.c.
 *
 *   RR angles: T = trunnion, S = shaft.  Mode 1: T = ASIN(-Y), S =
 *   ARCTRIG of the X-Z projection; mode 2: T2 = 180 - T, S2 = S + 180.
 *
 * AGC SOURCE:
 *   Luminary099 LEM_GEOMETRY.agc pp. 320-325 (RRANGLES, RRNB, RRNBMPAC)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   none (the caller passes the LOS, the CDU words and the RR mode)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   align_*               alignment helpers: displays, stalls, phases, word forms (alignment.h)
 *   agc_align_cdulogic    alignment routines of this lane (alignment.h)
 *   agc_mpac_*            interpretive instructions on MPAC words (interp_words.h)
 *   agc_vec_*             vector arithmetic of the interpreter (interp.h)
 *   agc_dp_*              DP arithmetic of the interpreter (interp.h, word.h)
 *   agc_interp_test_ovf   interpreter state: OVFIND (interp.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A failing service call (executive, waitlist, display, IMU,
 *   integration) returns its agc_status_t to the job; ALARM-type ends
 *   (CURTAINS, BAILOUT done) end the job with AGC_STATUS_OK.
 *
 * NOTES:
 *   RRANGLES and RRNB run on the MPAC words (word_forms.c): bit exact on
 *   the oracle (findings A4, A5).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"

#define LODPHALF       AGC_DP_HALF /* LODPHALF .5 rev = 180 degrees            */
#define EXTVBACT_BIT5  000020u     /* EXTVBACT bit 5: extended verb active (V85 shown) */
#define RRLOSDSP_DELAY 100u        /* 1SEC: RRLOSDSP repeats once a second, cs */

/**********************************************************************
 * RRANGLES (LEM_GEOMETRY.agc pp. 322-323) -- RR trunnion and shaft for the
 * half-unit NB line of sight los_nb. T1 = ASIN(-Y);  T2 = .5 - T1 P =
 * UNIT(X, 0, Z);  S1 = ARCTRIG(P.x/2, P.z/2);  S2 = S1 + .5 MODEA = (T1,
 * S1), MODEB = (T2, S2) as 2's complement CDU words (2V1STO2S), exchanged
 * when the RR is in mode 2 (RADMODES ANTENBIT). If the projection cannot be
 * unitized the LOS is along Y: the AGC goes to LUNDESCH (manoeuvre request,
 * P20-P25); here *along_y is set and the outputs are left untouched.
 *
 * FUNCTION NAME: agc_lm_rrangles
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  --------------------------------------
 *   ip        agc_interp_t *  IO   interpreter state (OVFIND, MPAC words)
 *   los_nb    agc_vec_t       I    RR line of sight in nav-base axes,
 *                                  half-unit (2^1)
 *   rr_mode2  bool            I    RR in mode 2 (RADMODES ANTENBIT): MODEA
 *                                  and MODEB exchanged
 *   modea     uint16_t        O    MODEA: trunnion, shaft CDU words (2^15 =
 *                                  1 rev)
 *   modeb     uint16_t        O    MODEB: trunnion, shaft CDU words (2^15 =
 *                                  1 rev)
 *   along_y   bool *          O    true when the LOS is along Y (BOVB
 *                                  LUNDESCH); outputs untouched
 **********************************************************************/
void agc_lm_rrangles(agc_interp_t *ip, agc_vec_t los_nb, bool rr_mode2, uint16_t modea[2], uint16_t modeb[2], bool *along_y)
{
    /* The angles stay in the words ASIN, BDSU, ARCTRIG and DAD leave
     * (signs may disagree), which 2V1STO2S converts: see word_forms.c. */
    const agc_dp_words_t half = agc_dp_to_words(LODPHALF);                                                       /* LODPHALF as words, revs 2^0 */
    const agc_tp_words_t t1m  = agc_mpac_asin(ip, agc_mpac_dcomp(agc_mpac_dload(agc_dp_to_words(los_nb.c[1])))); /* DLOAD DCOMP 34D, ASIN */
    const agc_dp_words_t t1w  = agc_mpac_store(t1m);                                                             /* PUSH: PD0 */
    const agc_dp_words_t t2w  = agc_mpac_store(agc_mpac_bdsu(ip, t1m, half));                                    /* BDSU LODPHALF: PD4 */
    agc_vec_t            p    = los_nb;                                                                          /* the LOS with Y = 0, then UNIT, half-unit (2^1) */

    p.c[1]   = 0;                       /* LO6ZEROS -> 34D */
    p        = agc_vec_unit(ip, p, NULL, NULL);
    *along_y = agc_interp_test_ovf(ip); /* BOVB LUNDESCH   */

    if (!*along_y)
    {
        const agc_dp_t       sinth = agc_dp_shr(p.c[0], 1);                                       /* P.X SR1 -> SINTH, scale 2^2 */
        const agc_dp_t       costh = agc_dp_shr(p.c[2], 1);                                       /* P.Z SR1 -> COSTH, scale 2^2 */
        const agc_dp_words_t s1w   = align_arctrig_words(ip, sinth, costh);                       /* PD2 */
        const agc_dp_words_t s2w   = agc_mpac_store(agc_mpac_dad(ip, agc_mpac_dload(s1w), half)); /* DAD LODPHALF */
        uint16_t             a[2]  = {align_words_to_cdu(t1w), align_words_to_cdu(s1w)};
        uint16_t             b[2]  = {align_words_to_cdu(t2w), align_words_to_cdu(s2w)};

        for (int k = 0; k < 2; k++)
        {
            modea[k] = rr_mode2 ? b[k] : a[k]; /* DXCH swap       */
            modeb[k] = rr_mode2 ? a[k] : b[k]; /* DXCH swap in mode 2 */
        }
    }
}

/**********************************************************************
 * RRNB / RRNBMPAC (LEM_GEOMETRY.agc p. 324) -- the RR line of sight in NB
 * axes from the trunnion (TANGNB) and shaft (TANGNB +1) CDU words: ( sin S
 * cos T, -sin T, cos S cos T ) / 2       (half-unit, 32D) RRNBMPAC is the
 * same with the two angles taken from MPAC.
 *
 * FUNCTION NAME: agc_lm_rrnb
 *
 * ARGUMENTS:
 *   Argument      Type            I/O  Description (scale)
 *   ------------  --------------  ---  ----------------------------------
 *   ip            agc_interp_t *  IO   interpreter state (OVFIND, MPAC
 *                                      words)
 *   trunnion_cdu  uint16_t        I    trunnion CDU word (CDUT, 2^15 = 1
 *                                      rev)
 *   shaft_cdu     uint16_t        I    shaft CDU word (CDUS, 2^15 = 1 rev)
 *   los_nb        agc_vec_t *     O    the RR line of sight in nav-base
 *                                      axes, half-unit (2^1)
 **********************************************************************/
void agc_lm_rrnb(agc_interp_t *ip, uint16_t trunnion_cdu, uint16_t shaft_cdu, agc_vec_t *los_nb)
{
    const agc_dp_t t     = agc_align_cdulogic(trunnion_cdu); /* trunnion T, revs 2^0 */
    const agc_dp_t y     = agc_dp_neg(agc_dp_sin(t));        /* -SIN T (34D), scale 2^1 */
    const agc_dp_t cos_t = agc_dp_cos(t);                    /* COS T (PD0), scale 2^1 */
    const agc_dp_t s     = agc_align_cdulogic(shaft_cdu);    /* shaft S (RRNB1), revs 2^0 */

    /* DMP SL1 then STORE: the SL1 acts on the TP product, so MPAC+2's top
     * bit becomes the stored low word's LSB (no rounding).              */
    const agc_dp_t z = align_dmp_shl(ip, agc_dp_cos(s), cos_t, 1); /* COS S COS T SL1, scale 2^1 */
    const agc_dp_t x = align_dmp_shl(ip, agc_dp_sin(s), cos_t, 1); /* SIN S COS T SL1, scale 2^1 */

    *los_nb = agc_vec_make(x, y, z);
}

/**********************************************************************
 * AGC: none -- ARCTRIG, then BPL / DAD DPPOSMAX: an angle 0..1 rev (error 2^-28 rev).
 *
 * FUNCTION NAME: angle_0_360
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *  I   interpreter state (OVFIND)
 *   sin4  agc_dp_t        I   sin / 4 of the angle
 *   cos4  agc_dp_t        I   cos / 4 of the angle
 *
 * RETURNS: the angle, 0..1 revolution
 **********************************************************************/
static agc_dp_t angle_0_360(agc_interp_t *ip, agc_dp_t sin4, agc_dp_t cos4)
{
    agc_dp_t a = agc_imu_arctrig(ip, sin4, cos4); /* ARCTRIG, rev 2^0 */

    if (a < 0)
    {
        a = agc_dp_add(ip, a, AGC_DP_MAX);
    }

    return (a);
}

/**********************************************************************
 * RRLOSDSP (EXTENDED_VERBS.agc L pp.299-300): the V85 job, FINDVAC'd by
 * the verb (src/io/verbs/extended_verbs_lm_radar.c). LOS in NB axes from
 * CDUT, CDUS (RRNBMPAC); its X-Z projection P.  RR-ELEV = angle (P.X,
 * P.Z) about +Y; RR-AZ = angle of the LOS out of the X-Z plane (LOS.Y,
 * LOS.P).  Revolutions.  Again in 1 s.
 *
 * FUNCTION NAME: agc_job_rr_los_display
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the RRNBMPAC or DELAYJOB status
 **********************************************************************/
agc_status_t agc_job_rr_los_display(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_vec_t los = agc_vec_zero(); /* RR LOS in NB axes, half-unit, 2^1 */
    agc_lm_rrnb(&ip, agc_erase_cdut(), agc_erase_cdus(), &los);

    agc_vec_t xz = los; /* LOS projected on the X-Z plane, half-unit, 2^1 */

    xz.c[1]               = 0;                                                                          /* HI6ZEROS into 8D       */
    xz                    = agc_vec_unit(&ip, xz, NULL, NULL);
    const agc_vec_t unitx = agc_vec_make(AGC_DP_HALF, 0, 0);                                            /* UNITX, half-unit, 2^1 */
    const agc_vec_t unity = agc_vec_make(0, AGC_DP_HALF, 0);                                            /* UNITY, half-unit, 2^1 */
    const agc_vec_t unitz = agc_vec_make(0, 0, AGC_DP_HALF);                                            /* UNITZ, half-unit, 2^1 */
    agc_dp_t        elev  = angle_0_360(&ip, agc_vec_dot(&ip, unitx, xz), agc_vec_dot(&ip, xz, unitz)); /* RR-ELEV, rev 2^0 */
    agc_dp_t        az    = angle_0_360(&ip, agc_vec_dot(&ip, los, unity), agc_vec_dot(&ip, los, xz));  /* RR-AZ, rev 2^0 */

    agc_erase_set_rr_elev(elev);
    agc_erase_set_rr_az(az);

    return (agc_svc_delayjob(RRLOSDSP_DELAY, AGC_JOB_RR_LOS_DISPLAY_NEXT));
}

/**********************************************************************
 * AGC: none -- RRLOSDSP after DELAYJOB: again while EXTVBACT bit 5 is set, else ENDEXT.
 *
 * FUNCTION NAME: agc_job_rr_los_display_next
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the RRLOSDSP or ENDEXT status
 **********************************************************************/
agc_status_t agc_job_rr_los_display_next(void)
{
    if ((agc_erase_extvbact() & EXTVBACT_BIT5) != 0u)
    {
        return (agc_job_rr_los_display());
    }

    return (agc_dsky_endmark());
}
