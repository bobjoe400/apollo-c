/*
 * align_common.c -- storage for the alignment erasables and the small
 * word-format and trigonometric helpers shared by every alignment routine.
 *
 * Sources: P51-P53.agc (Comanche055 pp. 737-784, Luminary099 pp. 926-983),
 * RTB_OP_CODES.agc (CDULOGIC, 1STO2S) and POWERED_FLIGHT_SUBROUTINES.agc
 * (CDUTRIG) for the helpers this module needs locally.
 */
#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_align.h"
#include "align_pending.h"

#define CDU_WORD_MASK   077777u   /* 15-bit CDU register                     */
#define CDU_SIGN_BIT    040000u   /* 180 degrees                             */
#define CDU_MODULUS     0100000   /* one revolution in CDU counts            */
#define CDU_HALF_REV    16384     /* counts in half a revolution             */
#define CDU_TO_REVS_SHIFT 8192    /* 2^13: cdu count -> DP revs (2^28/2^15)  */

static agc_align_ram_t align_ram;

/* Alignment erasable block (ERASABLE_ASSIGNMENTS "ALIGNMENT STORAGE"). */
agc_align_ram_t *agc_align_ram(void)
{
    (void)AGC_ASSERT(align_ram.mkvac_index >= -1);
    (void)AGC_ASSERT(align_ram.mkvac_index < AGC_EXEC_MAX_VACS);
    return &align_ram;
}

/* Fresh start: clear the alignment erasables. */
void agc_align_ram_reset(void)
{
    const agc_align_ram_t zero = { .mkvac_index = -1 };
    align_ram = zero;
    align_ram.qmin = AGC_JOB_NONE;
    align_ram.qmaj = AGC_JOB_NONE;
    (void)AGC_ASSERT(align_ram.mkvac_index == -1);
    (void)AGC_ASSERT(align_ram.starind == 0);
}

/*
 * CDULOGIC (RTB_OP_CODES.agc, Luminary p. 1397) -- convert the single
 * precision 2's complement CDU angle (half-revolutions) to a DP angle scaled
 * in revolutions.  DEVIATION: local copy of the RTB op (owner sys-misc);
 * the 1's/2's complement juggling reduces to a sign extension here.
 */
agc_dp_t agc_align_cdulogic(uint16_t cdu)
{
    int32_t v = (int32_t)(cdu & CDU_WORD_MASK);
    if (!AGC_ASSERT(cdu <= CDU_WORD_MASK)) {
        v = (int32_t)(cdu & CDU_WORD_MASK);
    }
    if ((v & (int32_t)CDU_SIGN_BIT) != 0) {
        v -= CDU_MODULUS;
    }
    (void)AGC_ASSERT((v >= -CDU_HALF_REV) && (v < CDU_HALF_REV));
    return (agc_dp_t)(v * CDU_TO_REVS_SHIFT);   /* revs B0 */
}

/*
 * 1STO2S / 1TO2SUB (RTB_OP_CODES.agc, Luminary p. 1397-1398) -- convert a
 * DP angle in revolutions to a SP 2's complement angle in half revolutions.
 * The low word is dropped (truncation toward zero, as the 1's complement
 * high word is), and a doubling overflow wraps by one revolution (LIMITS).
 */
uint16_t agc_align_1sto2s(agc_dp_t revs)
{
    int32_t hv = revs / CDU_TO_REVS_SHIFT;
    if (!AGC_ASSERT((revs >= AGC_DP_MIN) && (revs <= AGC_DP_MAX))) {
        hv = 0;
    }
    if (hv >= CDU_HALF_REV) {
        hv -= CDU_MODULUS;
    } else if (hv < -CDU_HALF_REV) {
        hv += CDU_MODULUS;
    } else {
        /* no doubling overflow */
    }
    if (hv < 0) {
        hv += CDU_MODULUS;
    }
    (void)AGC_ASSERT((hv >= 0) && (hv <= (int32_t)CDU_WORD_MASK));
    return (uint16_t)hv;
}

/*
 * CDUTRIG / CD*TR*GS equivalent (POWERED_FLIGHT_SUBROUTINES.agc, Luminary
 * p. 1259-1260) -- sines and cosines of the three CDU angles.
 * DEVIATION: local re-statement so the alignment math can be called with
 * explicit angles (CDUSPOT); the owner lane (guid-powered) keeps the real
 * CDUTRIG.  Output scale 2 (sin/2), as the interpreter SIN/COS leave it.
 */
agc_status_t agc_align_cdutrig(agc_interp_t *ip, const uint16_t cdu[3],
                               agc_align_trig_t *out)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(cdu != NULL) ||
        !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    const agc_dp_t ax = agc_align_cdulogic(cdu[0]);
    const agc_dp_t ay = agc_align_cdulogic(cdu[1]);
    const agc_dp_t az = agc_align_cdulogic(cdu[2]);
    out->cosx = agc_dp_cos(ip, ax);
    out->sinx = agc_dp_sin(ip, ax);
    out->cosy = agc_dp_cos(ip, ay);
    out->siny = agc_dp_sin(ip, ay);
    out->cosz = agc_dp_cos(ip, az);
    out->sinz = agc_dp_sin(ip, az);
    out->cdu[0] = cdu[0];
    out->cdu[1] = cdu[1];
    out->cdu[2] = cdu[2];
    return AGC_STATUS_OK;
}

/*
 * CALCSMSC (P51-P53.agc, Comanche p. 763, label CALCSMSC) -- compute the
 * navigation base axes XNB, YNB, ZNB (half-unit, wrt the stable member)
 * from the sines and cosines of the CDU angles:
 *   XNB = ( cY cZ, sZ, -sY cZ )
 *   ZNB = ( cX sY + sX sZ cY, -sX cZ, cX cY - sX sY sZ )
 *   YNB = ZNB x XNB
 * The order of the DMP/DAD/DSU steps is the AGC's.
 */
agc_status_t agc_align_calcsmsc(agc_interp_t *ip, const agc_align_trig_t *t,
                                agc_mat_t *xnb)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(t != NULL) ||
        !AGC_ASSERT(xnb != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    /* XNB: z = -SINCDUY*COSCDUZ, y = SINCDUZ/2, x = COSCDUY*COSCDUZ; VSL1 */
    const agc_dp_t xz = agc_dp_neg(agc_dp_mul(ip, t->siny, t->cosz));
    const agc_dp_t xy = agc_dp_shr(t->sinz, 1);
    const agc_dp_t xx = agc_dp_mul(ip, t->cosy, t->cosz);
    xnb->row[0] = agc_vec_shl(ip, agc_vec_make(xx, xy, xz), 1);

    /* 26D = SL1(SINCDUX*SINCDUZ) */
    const agc_dp_t d26 = agc_dp_shl(ip, agc_dp_mul(ip, t->sinx, t->sinz), 1);
    /* z = COSCDUX*COSCDUY - 26D*SINCDUY */
    const agc_dp_t p0 = agc_dp_mul(ip, d26, t->siny);
    const agc_dp_t zz = agc_dp_sub(ip, agc_dp_mul(ip, t->cosx, t->cosy), p0);
    /* y = -SINCDUX*COSCDUZ */
    const agc_dp_t zy = agc_dp_neg(agc_dp_mul(ip, t->sinx, t->cosz));
    /* x = COSCDUY*26D + COSCDUX*SINCDUY */
    const agc_dp_t p4 = agc_dp_mul(ip, t->cosx, t->siny);
    const agc_dp_t zx = agc_dp_add(ip, agc_dp_mul(ip, t->cosy, d26), p4);
    xnb->row[2] = agc_vec_shl(ip, agc_vec_make(zx, zy, zz), 1);

    /* YNB = ZNB x XNB, VSL1 */
    xnb->row[1] = agc_vec_shl(ip, agc_vec_cross(ip, xnb->row[2], xnb->row[0]), 1);
    return AGC_STATUS_OK;
}
