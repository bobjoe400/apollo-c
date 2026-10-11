/*
 * lem_geometry.c -- LEM_GEOMETRY.agc (Luminary099 pp. 320-325): rendezvous
 * radar gimbal angles for a line of sight (RRANGLES) and the radar line of
 * sight in navigation-base axes from its CDUs (RRNB, RRNBMPAC).
 * SVDWN2/SVDWN1 of the same file are in src/nav/integration/svdwn.c.
 *
 * RR angles: T = trunnion, S = shaft.  Mode 1: T = ASIN(-Y), S = ARCTRIG
 * of the X-Z projection; mode 2: T2 = 180 - T, S2 = S + 180.
 */
#include <stdbool.h>
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/programs/alignment.h"

#define LODPHALF AGC_DP_HALF /* LODPHALF .5 rev = 180 degrees            */

/*
 * RRANGLES (LEM_GEOMETRY.agc pp. 322-323) -- RR trunnion and shaft for the
 * half-unit NB line of sight los_nb.
 *   T1 = ASIN(-Y);  T2 = .5 - T1
 *   P = UNIT(X, 0, Z);  S1 = ARCTRIG(P.x/2, P.z/2);  S2 = S1 + .5
 *   MODEA = (T1, S1), MODEB = (T2, S2) as 2's complement CDU words
 *   (2V1STO2S), exchanged when the RR is in mode 2 (RADMODES ANTENBIT).
 * If the projection cannot be unitized the LOS is along Y: the AGC goes to
 * LUNDESCH (manoeuvre request, P20-P25); here *along_y is set and the
 * outputs are left untouched.
 */
agc_status_t agc_lm_rrangles(agc_interp_t *ip, agc_vec_t los_nb, bool rr_mode2, uint16_t modea[2], uint16_t modeb[2], bool *along_y)
{
    const agc_dp_t t1 = agc_dp_asin(ip, agc_dp_neg(los_nb.c[1])); /* PD0 */
    agc_vec_t      p  = los_nb;

    p.c[1]   = 0;                       /* LO6ZEROS -> 34D */
    p        = agc_vec_unit(ip, p, NULL, NULL);
    *along_y = agc_interp_test_ovf(ip); /* BOVB LUNDESCH   */

    if (*along_y)
    {
        return (AGC_STATUS_OK);
    }

    const agc_dp_t sinth = agc_dp_shr(p.c[0], 1);             /* SR1 -> SINTH    */
    const agc_dp_t costh = agc_dp_shr(p.c[2], 1);             /* SR1 -> COSTH    */
    const agc_dp_t s1    = agc_imu_arctrig(ip, sinth, costh); /* PD2 */

    /* 2V1STO2S converts the words the angles were left in (ASIN's and
     * ARCTRIG's disagree in sign), not their values: see word_forms.c. */
    const align_words_t half = align_words_of(LODPHALF);
    const align_words_t t1w  = align_asin_words(t1);
    const align_words_t s1w  = align_arctrig_words(ip, sinth, costh, s1);
    const align_words_t t2w  = align_words_add(half, align_words_neg(t1w)); /* PD4 BDSU */
    const align_words_t s2w  = align_words_add(s1w, half);                  /* DAD     */
    uint16_t            a[2] = {align_words_to_cdu(t1w), align_words_to_cdu(s1w)};
    uint16_t            b[2] = {align_words_to_cdu(t2w), align_words_to_cdu(s2w)};

    for (int k = 0; k < 2; k++)
    {
        modea[k] = rr_mode2 ? b[k] : a[k]; /* DXCH swap       */
        modeb[k] = rr_mode2 ? a[k] : b[k];
    }

    return (AGC_STATUS_OK);
}

/*
 * RRNB / RRNBMPAC (LEM_GEOMETRY.agc p. 324) -- the RR line of sight in NB
 * axes from the trunnion (TANGNB) and shaft (TANGNB +1) CDU words:
 *   ( sin S cos T, -sin T, cos S cos T ) / 2       (half-unit, 32D)
 * RRNBMPAC is the same with the two angles taken from MPAC.
 */
agc_status_t agc_lm_rrnb(agc_interp_t *ip, uint16_t trunnion_cdu, uint16_t shaft_cdu, agc_vec_t *los_nb)
{
    const agc_dp_t t     = agc_align_cdulogic(trunnion_cdu);
    const agc_dp_t y     = agc_dp_neg(agc_dp_sin(t)); /* 34D      */
    const agc_dp_t cos_t = agc_dp_cos(t);             /* PD0      */
    const agc_dp_t s     = agc_align_cdulogic(shaft_cdu); /* RRNB1    */

    /* DMP SL1 then STORE: the SL1 acts on the TP product, so MPAC+2's top
     * bit becomes the stored low word's LSB (no rounding).              */
    const agc_dp_t z = align_dmp_shl(ip, agc_dp_cos(s), cos_t, 1);
    const agc_dp_t x = align_dmp_shl(ip, agc_dp_sin(s), cos_t, 1);

    *los_nb = agc_vec_make(x, y, z);

    return (AGC_STATUS_OK);
}
