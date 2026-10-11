/**********************************************************************
 * FILE NAME: rtb_ops.c
 *
 * PURPOSE:
 *   The RTB ("return to basic") op codes: basic-language routines that
 *   interpretive code calls with "RTB name" to work on MPAC.
 *
 * AGC SOURCE:
 *   Luminary099 RTB_OP_CODES.agc pp.1397-1402  (LOADTIME, CDULOGIC,
 *   SGNAGREE, 1STO2S, V1STO2S, 2V1STO2S, 1TO2SUB, CDUINC, PULSEIMU,
 *   SIGNMPAC, NORMUNIT, NORMUNX1, VECSGNAG)
 *   Comanche055 RT8_OP_CODES.agc  (the same plus READPIPS, INCRCDUS,
 *   TRANSP1/TRANSP2, QUALITY1/QUALITY2)
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors)
 *   Name         Type      I/O  Scale     Description
 *   -----------  --------  ---  --------  -------------------------------
 *   TIME2,TIME1  uint32_t  I    cs        agc_time_now_cs()
 *   PIPAX/Y/Z    agc_sp_t  I    pulses    agc_erase_pipax() ...
 *
 * EXTERNAL REFERENCES:
 *   Name           Description
 *   -------------  -------------------------------------------------
 *   agc_vec_unit   UNIT +1 (NORMUNIT ends there)
 *   agc_imu_pulse  IMUPULSE (PULSEIMU)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None in the AGC.  Out-of-range arguments fail an assertion and give
 *   0 / BAD_ARG.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles; called from interpretive code (job context).
 *
 * NOTES:
 *   In the AGC an interpretive program says "RTB  name" and the routine
 *   runs in basic language on MPAC, then returns through DANZIG.  In
 *   apollo-c each RTB op code is a plain C function that takes the MPAC
 *   contents as arguments and returns the new MPAC contents.  The
 *   interpreter "mode" (SP/DP/TP/vector) set by DANZIG, NEWMODE, TPMODE,
 *   VMODE and DPMODE becomes the C return type.
 *
 *   Number model (docs/TRANSLATION_GUIDE.md section 1): agc_dp_t holds a
 *   DP fraction as one two's-complement integer, so a DP value always
 *   has its two words in sign agreement.  The routines that exist only
 *   to force sign agreement (SGNAGREE/TPAGREE, VECSGNAG/VECAGREE)
 *   therefore operate on an explicit word-level MPAC image,
 *   agc_mpac_tp_t, where disagreement can be expressed.
 *
 *   CDU angles are kept by the hardware as 15-bit two's-complement
 *   patterns scaled in half-revolutions (bit 15 = -180 deg).  They are
 *   passed here as the raw uint16_t channel/counter pattern, never as
 *   agc_sp_t.
 *
 *   Not here: 1/PIPA (IMU_COMPENSATION_PACKAGE.agc, imu lane) and
 *   FORMDEG (not present in either Apollo 11 rope; listed in the
 *   assignment by mistake).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"

#define SP_RANGE        040000     /* 2^14: weight of one SP word          */
#define CDU_MODULUS     0100000    /* 2^15: two's-complement CDU modulus   */
#define CDU_MASK        077777     /* 15-bit pattern mask                  */
#define CDU_SIGN        040000     /* bit 15 of a CDU pattern              */
#define DP_TIME_MASK    0x0FFFFFFF /* TIME2,TIME1 are 28 bits              */
#define NORMUNIT_SHIFT  13         /* THIRTEEN: NORMUNIT left shift        */
#define NORMUNIT_FACTOR 8193       /* 2^13 + 1, see NORMUNIT below         */

/**********************************************************************
 * AGC: none (the CCS/AD ONE/COM conversions in CDUINC, RTB_OP_CODES.agc
 * p.1398) -- a 15-bit two's-complement CDU pattern as a signed count
 *
 * ARGUMENTS:
 *   pattern  uint16_t  I   CDU pattern, 15 bits, 2's complement, 2^0 pi
 *
 * RETURNS: the angle in units of 2^-14 half-revolution, -16384..+16383
 **********************************************************************/
static int32_t cdu_pattern_value(uint16_t pattern)
{
    uint16_t p = (uint16_t) (pattern & CDU_MASK); /* 15-bit pattern */
    int32_t  v = (int32_t) p;                     /* count, 2^-14 half-rev */

    if (!AGC_ASSERT((pattern & (uint16_t) ~CDU_MASK) == 0u)) /* 15-bit */
    {
        return (0); /* no valid result */
    }

    if ((p & CDU_SIGN) != 0u)
    {
        v -= (int32_t) CDU_MODULUS;
    }

    if (!AGC_ASSERT(v >= -(int32_t) SP_RANGE && v < (int32_t) SP_RANGE))
    {
        return (0); /* no valid result */
    }

    return (v);
}

/**********************************************************************
 * AGC: none (LIMITS correction in 1TO2SUB / CDUINC, RTB_OP_CODES.agc
 * p.1398) -- reduce a signed count to the 15-bit two's-complement pattern
 * (angle modulo 360 deg)
 *
 * ARGUMENTS:
 *   v  int32_t  I   angle in units of 2^-14 half-revolution, |v| < 2^17
 *
 * RETURNS: the 15-bit pattern
 **********************************************************************/
static uint16_t cdu_value_pattern(int32_t v)
{
    int32_t m = v % (int32_t) CDU_MODULUS; /* pattern value, 2^-14 half-rev */

    if (!AGC_ASSERT(v > -4 * (int32_t) CDU_MODULUS && v < 4 * (int32_t) CDU_MODULUS))
    {
        return (0); /* no valid result */
    }

    if (m < 0)
    {
        m += (int32_t) CDU_MODULUS;
    }

    if (!AGC_ASSERT(m >= 0 && m < (int32_t) CDU_MODULUS))
    {
        return (0); /* no valid result */
    }

    return ((uint16_t) m);
}

/**********************************************************************
 * LOADTIME (RTB_OP_CODES.agc p.1397): load TIME2,TIME1 into MPAC.
 * Result: centiseconds, scaled 2^28 cs (the raw DP integer is the count).
 *
 * RETURNS: TIME2,TIME1 in centiseconds (DP, 2^28 cs)
 **********************************************************************/
agc_dp_t agc_rtb_loadtime(void)
{
    uint32_t t = agc_time_now_cs(); /* TIME2,TIME1, cs */

    if (!AGC_ASSERT(t <= (uint32_t) DP_TIME_MASK))
    {
        return (0); /* no valid result */
    }

    t &= (uint32_t) DP_TIME_MASK;
    if (!AGC_ASSERT((agc_dp_t) t <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }


    return ((agc_dp_t) t);
}

/**********************************************************************
 * CDULOGIC (RTB_OP_CODES.agc p.1397): convert the SP two's-complement CDU
 * angle in MPAC (scaled in half-revolutions) to a DP ones'-complement angle
 * scaled in revolutions.
 *
 * The AGC multiplies the word by HALF and, for a negative pattern, adds
 * -HALF into MPAC+1; that -1/2 LSB correction is exactly the difference
 * between the ones'- and two's-complement readings of the same pattern, so
 * the result is the true two's-complement value / 2: dp = v * 2^13.
 *
 * ARGUMENTS:
 *   cdu_pattern  uint16_t  I   CDU angle, 15-bit 2's complement pattern,
 *                              half-revolutions
 *
 * RETURNS: the angle as a DP fraction of a revolution (2^0 rev)
 **********************************************************************/
agc_dp_t agc_rtb_cdulogic(uint16_t cdu_pattern)
{
    int32_t  v = cdu_pattern_value(cdu_pattern);            /* 2^-14 half-revs       */
    agc_dp_t r = (agc_dp_t) (v * (int32_t) (SP_RANGE / 2)); /* revolutions, 2^0 */

    if (!AGC_ASSERT(r >= -AGC_DP_HALF && r < AGC_DP_HALF))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT((cdu_pattern & (uint16_t) ~CDU_MASK) == 0u)) /* 15-bit */
    {
        return (0); /* no valid result */
    }

    return (r); /* revolutions, scale 2^0 */
}

/**********************************************************************
 * SGNAGREE / TPAGREE (RTB_OP_CODES.agc p.1397, INTERPRETER.agc TPAGREE):
 * force sign agreement among the three words of MPAC.  Each word is a signed
 * SP word (sign + 14 bits).  The value V = w0*2^28 + w1*2^14 + w2 (in units of
 * 2^-42) is preserved; afterwards every non-zero word has the sign of V.
 * TPAGREE also returns the signum of the input in A; that is the return
 * value here (+1, 0, -1).
 *
 * ARGUMENTS:
 *   mpac  agc_mpac_tp_t *  I/O word-level MPAC, MPAC+1, MPAC+2, each an
 *                              SP word
 **********************************************************************/
int agc_rtb_sgnagree(agc_mpac_tp_t *mpac)
{
    for (int i = 0; i < 3; i++)
    {
        if (!AGC_ASSERT(mpac->w[i] >= AGC_SP_MIN && mpac->w[i] <= AGC_SP_MAX))
        {
            return (0);
        }

    }

    int64_t v    = ((int64_t) mpac->w[0] * SP_RANGE * SP_RANGE) + ((int64_t) mpac->w[1] * SP_RANGE) + (int64_t) mpac->w[2]; /* V, units of 2^-42 */
    int     sign = 0;                                                                                                       /* signum of V, the TPAGREE result in A */
    int64_t mag  = (v < 0) ? -v : v;                                                                                        /* |V|, 2^-42 */

    /* DEVIATION: when |V| >= 1 the AGC leaves an overflowed high word; the
     * C version saturates the high word at POSMAX instead.                  */
    int64_t hi = mag / ((int64_t) SP_RANGE * SP_RANGE); /* |MPAC| */

    if (v > 0)
    {
        sign = 1;
    }
    else if (v < 0)
    {
        sign = -1;
    }

    if (hi > AGC_SP_MAX)
    {
        hi = AGC_SP_MAX;
    }

    int64_t mid = (mag / SP_RANGE) % SP_RANGE; /* MPAC+1 magnitude */
    int64_t lo  = mag % SP_RANGE;              /* MPAC+2 magnitude */

    mpac->w[0] = (agc_sp_t) (sign * hi);
    mpac->w[1] = (agc_sp_t) (sign * mid);
    mpac->w[2] = (agc_sp_t) (sign * lo);

    return (sign);
} /* agc_rtb_sgnagree */

/**********************************************************************
 * SGNAGREE on a DP pair (RTB_OP_CODES.agc p.1397, MPAC+2 = 0) -- the
 * agreed DP in agc_dp_t form
 *
 * ARGUMENTS:
 *   hi  agc_sp_t  I   MPAC, SP word
 *   lo  agc_sp_t  I   MPAC+1, SP word
 *
 * RETURNS: the DP value of the pair, units of 2^-28
 **********************************************************************/
agc_dp_t agc_rtb_sgnagree_dp(agc_sp_t hi, agc_sp_t lo)
{
    agc_mpac_tp_t m; /* word-level MPAC image */

    m.w[0] = hi;
    m.w[1] = lo;
    m.w[2] = 0;
    int sg = agc_rtb_sgnagree(&m); /* signum of the agreed pair           */
    if (!AGC_ASSERT(sg >= -1 && sg <= 1))
    {
        return (0); /* no valid result */
    }


    return ((agc_dp_t) ((int32_t) m.w[0] * (int32_t) SP_RANGE + (int32_t) m.w[1]));
}

/**********************************************************************
 * 1TO2SUB (RTB_OP_CODES.agc p.1398): double the DP angle (revolutions ->
 * half-revolutions), keep the high word (truncation toward zero, as the
 * ones'-complement DDOUBL/TS sequence does) and express it as a two's-
 * complement pattern; the LIMITS correction on overflow makes the result
 * the angle modulo one revolution.
 *
 * ARGUMENTS:
 *   angle_rev  agc_dp_t  I   DP angle, revolutions (2^0)
 *
 * RETURNS: the 15-bit 2's complement pattern, half-revolutions
 **********************************************************************/
static uint16_t one_to_two(agc_dp_t angle_rev)
{
    int64_t doubled = 2 * (int64_t) angle_rev;      /* DDOUBL            */
    int64_t hi      = doubled / (int64_t) SP_RANGE; /* truncates to 0    */
    if (!AGC_ASSERT(hi > -2 * (int64_t) SP_RANGE && hi < 2 * (int64_t) SP_RANGE))
    {
        return (0); /* no valid result */
    }


    return (cdu_value_pattern((int32_t) hi));
}

/**********************************************************************
 * 1STO2S (RTB_OP_CODES.agc p.1397): DP ones'-complement angle in revolutions
 * -> SP two's-complement angle in half-revolutions (MPAC+1 cleared).
 *
 * ARGUMENTS:
 *   angle_rev  agc_dp_t  I   DP angle, revolutions (2^0)
 *
 * RETURNS: the SP 2's complement angle, half-revolutions (MPAC)
 **********************************************************************/
uint16_t agc_rtb_1sto2s(agc_dp_t angle_rev)
{
    uint16_t p = one_to_two(angle_rev); /* 2's comp half-revs */

    if (!AGC_ASSERT(p <= CDU_MASK))
    {
        return (0); /* no valid result */
    }

    return (p);
}

/**********************************************************************
 * V1STO2S (RTB_OP_CODES.agc pp.1397-1398): 1STO2S on each component of a
 * vector of angles; result in MPAC, MPAC+1, MPAC+2 (TP mode).
 *
 * ARGUMENTS:
 *   angles_rev  agc_vec_t    I   vector of DP angles, revolutions
 *   out         uint16_t []  O   MPAC, MPAC+1, MPAC+2: 2's complement
 *                                half-revolutions
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG on a NULL out
 **********************************************************************/
agc_status_t agc_rtb_v1sto2s(agc_vec_t angles_rev, uint16_t out[3])
{
    for (int i = 0; i < AGC_VEC_N; i++)
    {
        out[i] = one_to_two(angles_rev.c[i]);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * 2V1STO2S (RTB_OP_CODES.agc p.1398): two-component version used by the RR;
 * results arrive in A (first component) and L (second).
 *
 * ARGUMENTS:
 *   a_rev  agc_dp_t     I   first DP angle, revolutions (to A)
 *   b_rev  agc_dp_t     I   second DP angle, revolutions (to L)
 *   out    uint16_t []  O   the two 2's complement patterns,
 *                           half-revolutions
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG on a NULL out
 **********************************************************************/
agc_status_t agc_rtb_2v1sto2s(agc_dp_t a_rev, agc_dp_t b_rev, uint16_t out[2])
{
    if (!AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    out[0] = one_to_two(a_rev);
    out[1] = one_to_two(b_rev);

    if (!AGC_ASSERT(out[0] <= CDU_MASK && out[1] <= CDU_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CDUINC (RTB_OP_CODES.agc pp.1398-1399): increment a two's-complement CDU
 * angle register by the ones'-complement SP quantity INC.  The AGC converts
 * the angle to ones' complement, adds, converts back and repairs a crossing
 * of +-180 deg by adding 37777 or 40000 (LIMITS); the net effect is
 * addition modulo 2^15 on the two's-complement pattern.
 *
 * ARGUMENTS:
 *   angle_pattern  uint16_t  I   CDU angle register, 2's complement,
 *                                half-revolutions
 *   inc            agc_sp_t  I   ones' complement SP increment, 2^-14
 *                                half-revolution
 *
 * RETURNS: the new 2's complement pattern
 **********************************************************************/
uint16_t agc_rtb_cduinc(uint16_t angle_pattern, agc_sp_t inc)
{
    int32_t a = cdu_pattern_value(angle_pattern); /* angle, 2^-14 half-rev */

    if (!AGC_ASSERT(inc >= AGC_SP_MIN && inc <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    uint16_t r = cdu_value_pattern(a + (int32_t) inc); /* new pattern */

    if (!AGC_ASSERT(r <= CDU_MASK))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * INCRCDUS (RT8_OP_CODES.agc, Comanche only): increment THETAD, +1, +2 by
 * the high words of the MPAC vector components using CDUINC.  The caller
 * passes the THETAD patterns; the vector components are SP increments held
 * in the high word of each DP component.
 *
 * ARGUMENTS:
 *   thetad  uint16_t []  I/O THETAD, +1, +2 as 2's complement patterns,
 *                            half-revolutions
 *   incr    agc_vec_t    I   MPAC vector; the high word of each component
 *                            is the SP increment
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG on a NULL pointer or an out-of-range
 *          component
 **********************************************************************/
agc_status_t agc_rtb_incrcdus(uint16_t thetad[3], agc_vec_t incr)
{
    if (!AGC_ASSERT(thetad != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        int32_t hi = incr.c[i] / (int32_t) SP_RANGE; /* MPAC, MPAC+3, +5  */

        if (!AGC_ASSERT(hi >= AGC_SP_MIN && hi <= AGC_SP_MAX))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        thetad[i] = agc_rtb_cduinc(thetad[i], (agc_sp_t) hi);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * READPIPS (RT8_OP_CODES.agc, Comanche only): read PIPAX, PIPAY, PIPAZ
 * under INHINT into the high words of a vector (low words zero), VMODE.
 * Result scale: PIPA pulses, 2^14 pulses per unit of the high word.
 *
 * RETURNS: the PIPA counts in the high words of a vector (VMODE), 2^14
 *          pulses
 **********************************************************************/
agc_vec_t agc_rtb_readpips(void)
{
    agc_inhint();
    agc_sp_t x = agc_erase_pipax(); /* PIPAX, pulses, 2^0 */
    agc_sp_t y = agc_erase_pipay(); /* PIPAY, pulses, 2^0 */
    agc_sp_t z = agc_erase_pipaz(); /* PIPAZ, pulses, 2^0 */

    agc_relint();

    if (!AGC_ASSERT(x >= AGC_SP_MIN && x <= AGC_SP_MAX))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT(y >= AGC_SP_MIN && z >= AGC_SP_MIN))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    agc_vec_t v; /* VMODE result, 2^14 pulses */

    v.c[0] = (agc_dp_t) x * (agc_dp_t) SP_RANGE;
    v.c[1] = (agc_dp_t) y * (agc_dp_t) SP_RANGE;
    v.c[2] = (agc_dp_t) z * (agc_dp_t) SP_RANGE;

    return (v);
}

/**********************************************************************
 * PULSEIMU (RTB_OP_CODES.agc p.1400): torque the gyros with the commands
 * whose address arrives in X1, i.e. call IMUPULSE without the IMUSTALL.
 *
 * ARGUMENTS:
 *   gyro_cmds  agc_vec_t  I   gyro torquing commands (the words X1
 *                             addresses), pulses
 *
 * RETURNS: the IMUPULSE status (OK, or BUSY when the gyros are in use)
 **********************************************************************/
agc_status_t agc_rtb_pulseimu(agc_vec_t gyro_cmds)
{
    for (int i = 0; i < AGC_VEC_N; i++)
    {
    }

    agc_status_t st = agc_imu_pulse(gyro_cmds.c); /* OGC buffer */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * TRANSP1 / TRANSP2 (RT8_OP_CODES.agc, Comanche only): transpose the 3x3
 * matrix XNB (TRANSP1) or XNB1 (TRANSP2) in place by three DXCH swaps.
 *
 * ARGUMENTS:
 *   m  agc_mat_t *  I/O XNB or XNB1, transposed in place
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG on a NULL matrix
 **********************************************************************/
agc_status_t agc_rtb_transp(agc_mat_t *m)
{
    if (!AGC_ASSERT(m != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    static const int swap_r[3] = {0, 0, 1};
    static const int swap_c[3] = {1, 2, 2};

    for (int k = 0; k < 3; k++)
    {
        int r = swap_r[k]; /* row of the swap */
        int c = swap_c[k]; /* column of the swap */

        if (!AGC_ASSERT(r < c && c < AGC_VEC_N))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_dp_t t = m->row[r].c[c]; /* DXCH temporary, scale of XNB */

        m->row[r].c[c] = m->row[c].c[r];
        m->row[c].c[r] = t;
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SIGNMPAC (RTB_OP_CODES.agc p.1401): set MPAC to DPOSMAX or -DPOSMAX by the
 * sign of the high word of MPAC (+0 -> POSMAX, -0 -> NEGMAX).  Used directly
 * as "RTB SIGNMPAC" and as the BOVB target that limits MPAC on overflow.
 * A tiny negative DP has a -0 high word, so "v < 0" is the faithful test.
 *
 * ARGUMENTS:
 *   mpac  agc_dp_t  I   MPAC, DP
 *
 * RETURNS: DPOSMAX or -DPOSMAX (2^0) by the sign of mpac
 **********************************************************************/
agc_dp_t agc_rtb_signmpac(agc_dp_t mpac)
{
    agc_dp_t r = (mpac < 0) ? -AGC_DP_MAX : AGC_DP_MAX; /* +-DPOSMAX, 2^0 */

    if (!AGC_ASSERT(r == AGC_DP_MAX || r == -AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * NORMUNIT / NORMUNX1 (RTB_OP_CODES.agc pp.1401-1402): like UNIT, but when
 * the high words of all three components are zero the vector is first
 * shifted left 13 so that UNIT does not blow up.  The shift count (13 or 0)
 * is left in 37D (NORMUNIT) or X1 (NORMUNX1): returned in *shift.  As the
 * AGC warns, after a shift the magnitude in 36D is too large by 2^13 and
 * the square in 34D by 2^26.
 *
 * The AGC forms the shifted component with "CA MPAC+1, MP BIT14, DAS MPAC":
 * DAS adds lo*2^13 to the pair (0, lo) still in MPAC, so the shifted value is
 * lo*(2^13 + 1) ("DAS GAINS A LITTLE ACCURACY").  That is kept here.
 *
 * ARGUMENTS:
 *   ip     agc_interp_t *  I   interpreter context (overflow, push-down
 *                              state)
 *   v      agc_vec_t       I   MPAC vector
 *   shift  int *           O   13 if the vector was shifted, else 0 (37D
 *                              / X1)
 *   abval  agc_dp_t *      O   |v| as UNIT leaves it in 36D (2^13 too
 *                              large after a shift)
 *   vsq    agc_dp_t *      O   |v|^2 as UNIT leaves it in 34D (2^26 too
 *                              large after a shift)
 *
 * RETURNS: the unit vector (half-unit, as UNIT returns it)
 **********************************************************************/
agc_vec_t agc_rtb_normunit(agc_interp_t *ip, agc_vec_t v, int *shift, agc_dp_t *abval, agc_dp_t *vsq)
{
    agc_vec_t w = v; /* vector given to UNIT, scale of v */
    int       n = 0; /* shift count (37D) */

    bool all_small = true; /* all high words zero */

    for (int i = 0; i < AGC_VEC_N; i++)
    {
        if (v.c[i] >= (agc_dp_t) SP_RANGE || v.c[i] <= -(agc_dp_t) SP_RANGE)
        {
            all_small = false;
        }
    }

    if (all_small)
    {
        for (int i = 0; i < AGC_VEC_N; i++)
        {
            w.c[i] = v.c[i] * (agc_dp_t) NORMUNIT_FACTOR;
        }

        n = NORMUNIT_SHIFT;
    }

    if (shift != NULL)
    {
        *shift = n;
    }

    return (agc_vec_unit(ip, w, abval, vsq)); /* UNIT +1               */
}

/**********************************************************************
 * VECSGNAG (RTB_OP_CODES.agc p.1402): force sign agreement in each component
 * of the vector in MPAC (VECAGREE).  agc_dp_t components are always in sign
 * agreement, so this is the identity; kept so callers mirror the AGC text.
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I   MPAC vector
 *
 * RETURNS: v unchanged
 **********************************************************************/
agc_vec_t agc_rtb_vecsgnag(agc_vec_t v)
{
    return (v);
}

/**********************************************************************
 * QUALITY2 (RT8_OP_CODES.agc, Comanche only, SETLOC MODCHG2; entered from
 * QUALITY1 with MPAC = URPV_x^2 scaled B2): the lunar J22 term of the
 * orbital-integration disturbing acceleration.
 *   v = 5 (y^2 - x^2) UR  (B6), VSL3 -> B3
 *   v.x += 2x, v.y -= 2y  (B3)    i.e. v += 2 (x UNITX - y UNITY)
 *   result = E3J22R2M * v         "J22 TERM X R**4", scaled B61
 * urpv: unit position vector, scaled B1.  e3j22r2m: SP coefficient
 * (padload erasable E3J22R2M), scaled B58.  On return the AGC loads
 * COSPHI/2 (= URPV z) into MPAC for QUALITY3; the caller does that.
 *
 * ARGUMENTS:
 *   ip        agc_interp_t *  I   interpreter context
 *   urpv      agc_vec_t       I   unit position vector, B1
 *   e3j22r2m  agc_sp_t        I   SP coefficient E3J22R2M, B58
 *
 * RETURNS: the J22 term x R^4, B61
 **********************************************************************/
agc_vec_t agc_rtb_quality2(agc_interp_t *ip, agc_vec_t urpv, agc_sp_t e3j22r2m)
{
    static const agc_dp_t five_eighths = 0x0A000000;               /* 5/8: 2DEC 5 B-3 */
    agc_dp_t              x2           = agc_dp_sq(ip, urpv.c[0]); /* B2              */
    agc_dp_t              y2           = agc_dp_sq(ip, urpv.c[1]); /* B2              */
    agc_dp_t              d            = agc_dp_sub(ip, y2, x2);   /* DSU, B2         */

    d           = agc_dp_mul(ip, d, five_eighths); /* DMP 5/8, B5     */
    agc_vec_t v = agc_vec_scale(ip, urpv, d);      /* VXSC, B6        */

    v      = agc_vec_shl(ip, v, 3);                            /* VSL3, B3        */
    v.c[0] = agc_dp_add(ip, agc_dp_shr(urpv.c[0], 1), v.c[0]); /* SR1 DAD  */
    v.c[1] = agc_dp_sub(ip, v.c[1], agc_dp_shr(urpv.c[1], 1)); /* SR1 BDSU */

    return (agc_vec_scale(ip, v, agc_dp_from_sp(e3j22r2m))); /* VXSC B61 */
}

/**********************************************************************
 * QUALITY1 (RT8_OP_CODES.agc, SETLOC MODCHG3): only when MOONFLAG is set is
 * the J22 term computed; otherwise control goes to NBRANCH in ORBITAL_
 * INTEGRATION (returned here as AGC_STATUS_NOT_FOUND, *out untouched).
 *
 * ARGUMENTS:
 *   ip        agc_interp_t *  I   interpreter context
 *   urpv      agc_vec_t       I   unit position vector, B1
 *   moonflag  bool            I   MOONFLAG (lunar primary body)
 *   e3j22r2m  agc_sp_t        I   SP coefficient E3J22R2M, B58
 *   out       agc_vec_t *     O   the J22 term, B61, when moonflag is set
 *
 * RETURNS: AGC_STATUS_OK, NOT_FOUND when MOONFLAG is clear (NBRANCH),
 *          BAD_ARG on a NULL pointer
 **********************************************************************/
agc_status_t agc_rtb_quality1(agc_interp_t *ip, agc_vec_t urpv, bool moonflag, agc_sp_t e3j22r2m, agc_vec_t *out)
{
    if (!AGC_ASSERT(ip != NULL) || !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!moonflag)
    {
        return (AGC_STATUS_NOT_FOUND); /* BOF -> NBRANCH  */
    }

    *out = agc_rtb_quality2(ip, urpv, e3j22r2m);

    return (AGC_STATUS_OK);
}
