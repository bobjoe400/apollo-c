/**********************************************************************
 * FILE NAME: compensation.c
 *
 * PURPOSE:
 *   IMU compensation package: PIPA scale-factor and bias compensation
 *   (1/PIPA), gyro compensation for acceleration-sensitive drift (ADIA,
 *   ADSRA) and bias drift (NBD) into GCOMP (IRIGCOMP, NBDONLY, LASTBIAS),
 *   and gyro torquing when any command exceeds two pulses (1/GYRO,
 *   GCOMP1).
 *
 * AGC SOURCE:
 *   Comanche055 IMU_COMPENSATION_PACKAGE.agc pp. 297-306
 *   Luminary099 IMU_COMPENSATION_PACKAGE.agc pp. 326-337 (differs: see
 *   NOTES)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   GCOMP         agc_dp_t   I/O  gyro pls 2^14  gyro buffer GCOMP
 *   DELV          agc_vec_t  I/O  PIPA pls 2^14  agc_erase_delv()
 *   PIPAX..Z      agc_sp_t   O    pulses B0      PIPUSE (CM LASTBIAS)
 *   PIPTIME       agc_dp_t   I    cs B28         agc_erase_piptime()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_imu_pulse_buf     IMUPULSE (gyro.c), agc_imu_stall
 *   agc_exec_novac        NOVAC 1/GYRO
 *   agc_pipasr_noscale    PIPASR +3 (LM surface)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None of its own; IMUPULSE returns ALARM when the IMU is caged.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles.  1/PIPA is called from SERVICER and the CM prelaunch
 *   filter; 1/GYRO, NBDONLY and LASTBIAS run as jobs.
 *
 * NOTES:
 *   All of this is basic (machine) code built from MP / DAS sequences,
 *   translated word for word with agc_sp_mul (MP) and the A and L halves
 *   of the product.  GCOMP and DELV are kept as AGC word pairs (DAS does
 *   not make the signs of the words agree, and DRFTSUB2 and every MP read
 *   one word): oracle finding F6, fixed by review-imu.
 *   Scales (from the AGC comments):
 *     DELVX..Z   PIPA pulses x 2^14 (DP; high word = whole pulses)
 *     PIPASCF    (ppm) x 2^-9
 *     PIPABIAS   (PIPA pulses)/cs x 2^-5 (LM) / 2^-8 (CM)
 *     1/PIPADT   cs x 2^8
 *     NBD        (gyro pulses)/cs x 2^-5
 *     ADIA ADSRA (gyro pulses)/(PIPA pulse) x 2^-6 (LM) / 2^-3 (CM)
 *     GCOMP      gyro pulses x 2^14 (DP), before 1/GYRO rescales it to
 *                revolutions for IMUPULSE (pulses x 2^7 raw)
 *   CM vs LM differences:
 *   * PIPA bias term: LM MP BIT4 (shift right 11), CM MP BIT1 (shift 14).
 *   * GCOMPSUB: LM MP BIT9 (shift right 6), CM MP BIT12 (shift right 3).
 *   * CM 1/PIPA runs under INHINT; LM does not.
 *   * NBDONLY/LASTBIAS: LM also compensates acceleration terms when
 *     SURFFLAG is set (LM on the lunar surface) and reads the PIPAs via
 *     PIPASR +3; CM computes the elapsed time with CCS instead of the
 *     HALF+HALF overflow trick and ends the job when it is -0.
 *   * LASTBIAS: CM BANKCALL PIPUSE (PIPAs zeroed), LM PIPUSE1.
 *
 * ALGORITHM:
 *   As the AGC listing; see each function prolog.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"

/* Powers of two used as MP multipliers (BITn = 2^(n-1)).                 */
#define BIT4  ((agc_sp_t) 00010) /* 2^3: MP shifts right 11 */
#define BIT6  ((agc_sp_t) 00040) /* 2^5: MP shifts right 9 */
#define BIT8  ((agc_sp_t) 00200) /* 2^7: MP shifts right 7 */
#define BIT9  ((agc_sp_t) 00400) /* 2^8: MP shifts right 6 */
#define BIT10 ((agc_sp_t) 01000) /* 2^9: MP shifts right 5 */
#define BIT12 ((agc_sp_t) 04000) /* 2^11: MP shifts right 3 */
#define BIT1  ((agc_sp_t) 00001) /* 2^0: MP shifts right 14 */

#define TIME1_MODULUS 040000              /* TIME1 is a 14-bit counter (cs)         */
#define PRIO21        ((agc_prio_t) 021)  /* priority greater than SERVICER         */
#define PRIO31_2SEC   ((agc_sp_t) 031000) /* 2 s scaled cs x 2^8 (PRIO31)     */

/* Pad-loaded compensation parameters (module-private erasables).         */
static agc_imu_comp_params_t comp;       /* pad-loaded PIPA and gyro coefficients */
static agc_sp_t              gcompsw;    /* GCOMPSW: <0 bypass, >0 torque, B0     */
static agc_sp_t              one_pipadt; /* 1/PIPADT cs x 2^8, or TIME1 snapshot  */

/* ---- small word-level helpers ------------------------------------------ */

/* An AGC double-precision word pair as the two words hold it.  A DAS
 * adds the words separately (the low-word overflow carries into the high
 * word) and never makes their signs agree, so GCOMP and DELV can be, e.g.,
 * (49, -14827): 48.09 pulses whose high word reads 49.  DRFTSUB2 tests
 * that high word, and every MP takes one word at a time, so the pair is
 * kept as such here (agc_dp_t would normalise it, see oracle finding F6). */
typedef struct word_pair
{
    agc_sp_t hi; /* major word: whole pulses x 2^0             */
    agc_sp_t lo; /* minor word: fractional pulses x 2^14       */
} word_pair_t;

/**********************************************************************
 * AGC: none (word pair) -- the value of a DP word pair, hi 2^14 + lo
 *
 * ARGUMENTS:
 *   w  word_pair_t  I    the two words
 *
 * RETURNS: agc_dp_t -- the value, same scale
 **********************************************************************/
static agc_dp_t pair_value(word_pair_t w)
{
    return (agc_dp_pack(w.hi, w.lo));
}

/**********************************************************************
 * AGC: none (word pair) -- the words of a DP whose signs agree
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I    value (MP product, loaded word pair)
 *
 * RETURNS: word_pair_t -- A and L words
 **********************************************************************/
static word_pair_t pair_of(agc_dp_t v)
{
    word_pair_t w = {agc_dp_high(v), agc_dp_low(v)}; /* A, L of v     */

    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        w.hi = 0;
        w.lo = 0;
    }

    if (!AGC_ASSERT(pair_value(w) == v || v < AGC_DP_MIN || v > AGC_DP_MAX))
    {
        return (w);
    }

    return (w);
}

/**********************************************************************
 * DAS (IMU_COMPENSATION_PACKAGE.agc pp.297-306) -- add a DP into a word
 * pair
 *
 * Word by word: a low-word overflow carries +-1 into the high word, whose
 * own overflow is not used by the compensation.  The signs of the words
 * are not made to agree.
 *
 * ARGUMENTS:
 *   acc  word_pair_t  I    accumulator words
 *   x    agc_dp_t     I    addend (sign-agreeing DP, e.g. an MP
 *                          product)
 *
 * RETURNS: word_pair_t -- the sum as the AGC words hold it
 **********************************************************************/
static word_pair_t das(word_pair_t acc, agc_dp_t x)
{
    const word_pair_t xw    = pair_of(x);     /* words of the addend    */
    int               lo    = acc.lo + xw.lo; /* low-word sum           */
    int               carry = 0;              /* overflow of the low sum */

    if (lo > AGC_SP_MAX)
    {
        lo    -= AGC_SP_MAX + 1;
        carry = 1;
    }
    else if (lo < AGC_SP_MIN)
    {
        lo    += AGC_SP_MAX + 1;
        carry = -1;
    }

    const int hi = acc.hi + xw.hi + carry; /* high-word sum          */

    if (!AGC_ASSERT(hi >= AGC_SP_MIN && hi <= AGC_SP_MAX))
    {
        return (acc); /* no valid result */
    }

    word_pair_t r = {(agc_sp_t) hi, (agc_sp_t) lo}; /* the sum      */

    return (r);
}

/**********************************************************************
 * GCOMPSUB / DRIFTSUB / FBIASSUB (IMU_COMPENSATION_PACKAGE.agc
 * pp.301-306) -- MP X; TS L; CAF ZERO; DAS acc: the high half of a b
 * into the low word
 *
 * ARGUMENTS:
 *   acc  word_pair_t  I    accumulator words
 *   a    agc_sp_t     I    multiplicand, B0
 *   b    agc_sp_t     I    multiplier, B0
 *
 * RETURNS: word_pair_t -- acc + (0, hi(a b))
 **********************************************************************/
static word_pair_t das_low_of_mp(word_pair_t acc, agc_sp_t a, agc_sp_t b)
{
    agc_sp_t hi = agc_dp_high(agc_sp_mul(a, b)); /* A of the product, B0 */

    return (das(acc, agc_dp_pack(0, hi)));
}

/**********************************************************************
 * AGC: none (CS) -- negate a single-precision word
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I    word, B0
 *
 * RETURNS: agc_sp_t -- -a
 **********************************************************************/
static agc_sp_t sp_neg(agc_sp_t a)
{
    agc_sp_t r = (agc_sp_t) (-a); /* -a (CS), B0 */

    return (r);
}

/**********************************************************************
 * AGC: none (checker) -- compensation coefficients within a word
 *
 * ARGUMENTS:
 *   p  const agc_imu_comp_params_t *  I    coefficients
 *
 * RETURNS: bool -- true when every word fits in 15 bits
 **********************************************************************/
static bool params_in_range(const agc_imu_comp_params_t *p)
{
    bool ok = true; /* all coefficients in range */

    for (int i = 0; i < 3; i++)
    {
        ok = ok && p->pipabias[i] >= AGC_SP_MIN && p->pipascf[i] >= AGC_SP_MIN && p->nbd[i] >= AGC_SP_MIN && p->adia[i] >= AGC_SP_MIN && p->adsra[i] >= AGC_SP_MIN;
    }

    if (!AGC_ASSERT(p != NULL) || !AGC_ASSERT(ok))
    {
        return (false);
    }

    return (ok);
}

/* ---- parameter access --------------------------------------------------- */

/**********************************************************************
 * AGC: none (pad load) -- PIPABIAS, PIPASCF, NBD, ADIA, ADSRA
 *
 * ARGUMENTS:
 *   p  const agc_imu_comp_params_t *  I    coefficients, scales as in
 *                                          imu.h
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on NULL or out of range
 **********************************************************************/
agc_status_t agc_imu_comp_load(const agc_imu_comp_params_t *p)
{
    if (!AGC_ASSERT(params_in_range(p)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    comp = *p;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (accessor) -- the compensation coefficients
 *
 * ARGUMENTS:
 *   p  agc_imu_comp_params_t *  O    coefficients
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on NULL
 **********************************************************************/
agc_status_t agc_imu_comp_get(agc_imu_comp_params_t *p)
{
    *p = comp;

    if (!AGC_ASSERT(params_in_range(p)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (accessor) -- GCOMPSW
 *
 * RETURNS: agc_sp_t -- GCOMPSW: < 0 bypass, > 0 torque needed, B0
 **********************************************************************/
agc_sp_t agc_imu_gcompsw(void)
{
    if (!AGC_ASSERT(gcompsw <= AGC_SP_MAX - 1)) /* MASK, -1: never POSMAX */
    {
        return (0); /* no valid result */
    }

    return (gcompsw);
}

/**********************************************************************
 * AGC: none (accessor) -- set GCOMPSW
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I    GCOMPSW, B0
 **********************************************************************/
void agc_imu_set_gcompsw(agc_sp_t v)
{
    if (AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        gcompsw = v;
    }

    if (!AGC_ASSERT(gcompsw == v || v < AGC_SP_MIN || v > AGC_SP_MAX))
    {
        return;
    }
}

/**********************************************************************
 * AGC: none (accessor) -- 1/PIPADT
 *
 * RETURNS: agc_sp_t -- 1/PIPADT, cs x 2^8 (or a TIME1 snapshot)
 **********************************************************************/
agc_sp_t agc_imu_pipa_interval(void)
{
    if (!AGC_ASSERT(one_pipadt >= AGC_SP_MIN && one_pipadt <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(one_pipadt >= 0)) /* cs x 2^8 or a TIME1 snapshot */
    {
        return (0); /* no valid result */
    }

    return (one_pipadt);
}

/**********************************************************************
 * AGC: none (accessor) -- set 1/PIPADT
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I    1/PIPADT, cs x 2^8
 **********************************************************************/
void agc_imu_set_pipa_interval(agc_sp_t v)
{
    if (AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        one_pipadt = v;
    }

    if (!AGC_ASSERT(one_pipadt == v || v < AGC_SP_MIN || v > AGC_SP_MAX))
    {
        return;
    }
}

/* ---- GCOMP access (the GCOMP gyro buffer lives in mode.c) ----------- */

/* GCOMP as its words.  The buffer holds the value; when something else
 * changed it (IMUPULSE, a load) the words restart from that value.      */
static word_pair_t gcomp_words[3]; /* GCOMP words, gyro pulses x 2^14 */

/**********************************************************************
 * AGC: none (GCOMP words) -- read GCOMP as word pairs
 *
 * The gyro buffer holds the value; when something else changed it
 * (IMUPULSE, a load) the words restart from that value.
 *
 * ARGUMENTS:
 *   g  word_pair_t [3]  O    GCOMP X, Y, Z words, gyro pulses x 2^14
 *
 * RETURNS: agc_status_t -- OK or BAD_ARG
 **********************************************************************/
static agc_status_t gcomp_read(word_pair_t g[3])
{
    agc_dp_t     v[3];                                             /* GCOMP buffer, gyro pulses x 2^14     */
    agc_status_t st = agc_imu_gyro_buf_get(AGC_IMU_GBUF_GCOMP, v); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK) || !AGC_ASSERT(g != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int k = 0; k < 3; k++)
    {
        if (pair_value(gcomp_words[k]) != v[k])
        {
            gcomp_words[k] = pair_of(v[k]);
        }

        g[k] = gcomp_words[k];
    }

    return (st);
}

/**********************************************************************
 * AGC: none (GCOMP words) -- store GCOMP words and their values
 *
 * ARGUMENTS:
 *   g  const word_pair_t [3]  I    GCOMP X, Y, Z words, gyro pulses x
 *                                  2^14
 **********************************************************************/
static void gcomp_write(const word_pair_t g[3])
{
    agc_dp_t v[3]; /* GCOMP buffer, gyro pulses x 2^14             */

    for (int k = 0; k < 3; k++)
    {
        gcomp_words[k] = g[k];
        v[k]           = pair_value(g[k]);
    }

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_GCOMP, v);

    if (!AGC_ASSERT(pair_value(gcomp_words[0]) == v[0]))
    {
        return;
    }
}

/**********************************************************************
 * 1/PIPA, NBDONLY, LASTBIAS (IMU_COMPENSATION_PACKAGE.agc pp.297, 304,
 * 306) -- CCS GCOMPSW
 *
 * The compensation is bypassed only when GCOMPSW is negative.
 *
 * RETURNS: bool -- true when GCOMPSW < 0
 **********************************************************************/
static bool gcomp_bypassed(void)
{
    bool bypass = (gcompsw < 0); /* CCS GCOMPSW negative */

    if (!AGC_ASSERT(bypass || gcompsw >= 0))
    {
        return (false);
    }

    return (bypass);
}

/* ---- 1/PIPA ------------------------------------------------------------- */

static agc_status_t comp_irig_words(const word_pair_t delv[3]);

/**********************************************************************
 * 1/PIPA / 1/PIPA1 (IMU_COMPENSATION_PACKAGE.agc pp.297-298) -- PIPA
 * compensation, then IRIGCOMP
 *
 * DELV = DELV + DELV SFE - BIAS DELTAT for Z, Y, X (the AGC's 4, 2, -0
 * index loop), PIPA pulses x 2^14; CM under INHINT.
 *
 * ARGUMENTS:
 *   delv  agc_vec_t *  I/O  DELVX..Z, PIPA pulses x 2^14
 *
 * RETURNS: agc_status_t -- IRIGCOMP status (OK or FULL)
 **********************************************************************/
agc_status_t agc_imu_one_over_pipa(agc_vec_t *delv)
{
    if (gcomp_bypassed())
    {
        return (AGC_STATUS_OK); /* TCF IRIG1      */
    }

    bool        cm = (agc_state_vehicle() == AGC_VEHICLE_CSM); /* CM: 1/PIPA under INHINT */
    word_pair_t dw[3];                                         /* DELVX..Z as words, PIPA pulses x 2^14  */

    if (cm)
    {
        agc_inhint(); /* CM: assure complete compensation of DELVs */
    }

    for (int axis = 2; axis >= 0; axis--)
    {
        /* 1/PIPA1 body for one axis (ICP p.326):
         *   DELV = DELV + DELV*SFE - BIAS*DELTAT, all in PIPA pulses x 2^14.    */
        agc_dp_t delv_axis = delv->c[axis]; /* DELVX+axis, PIPA pulses x 2^14 */

        agc_sp_t pulses = agc_dp_high(delv_axis);                 /* whole PIPA pulses, B0 */
        agc_dp_t p      = agc_sp_mul(comp.pipascf[axis], pulses); /* (PP) x 2^5    */

        /* Minor part, shifted right 9: fractional PIPA pulses x 2^14.        */
        agc_sp_t    frac = agc_dp_high(agc_sp_mul(agc_dp_low(p), BIT6)); /* SFE fraction, pulses x 2^14 */
        word_pair_t d    = {pulses, frac};                               /* TS DELVX +1    */

        d = das(d, agc_sp_mul(agc_dp_high(p), BIT6)); /* (PIPAI)(SFE)   */

        /* -(BIAS)(DELTAT): CS PIPABIAS, MP 1/PIPADT, MP BIT4 (LM) / BIT1 (CM). */
        agc_dp_t b     = agc_sp_mul(sp_neg(comp.pipabias[axis]), one_pipadt);   /* -(BIAS)(DELTAT), pulses x 2^8 */
        agc_sp_t shift = (agc_state_vehicle() == AGC_VEHICLE_LM) ? BIT4 : BIT1; /* MP BIT4 (LM) / BIT1 (CM), B0 */

        d = das(d, agc_sp_mul(agc_dp_high(b), shift));

        dw[axis]      = d;
        delv->c[axis] = pair_value(d);
    }

    agc_status_t st = comp_irig_words(dw); /* status of the call */

    if (cm)
    {
        agc_relint();
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
} /* agc_imu_one_over_pipa */

/* ---- gyro compensation terms ------------------------------------------- */

/**********************************************************************
 * GCOMPSUB (IMU_COMPENSATION_PACKAGE.agc p.301) -- GCOMP += coefficient
 * x PIPA counts
 *
 * The product is shifted right 6 (LM, MP BIT9) or 3 (CM, MP BIT12) into
 * gyro pulses x 2^14.
 *
 * ARGUMENTS:
 *   g     word_pair_t  I    GCOMP words of this gyro
 *   mpac  word_pair_t  I    -DELV words (DCS), PIPA pulses x 2^14
 *   coef  agc_sp_t     I    ADIA or ADSRA, gyro/PIPA pulse x 2^-3 (CM)
 *
 * RETURNS: word_pair_t -- new GCOMP words
 **********************************************************************/
static word_pair_t gcompsub(word_pair_t g, word_pair_t mpac, agc_sp_t coef)
{
    /* VBUF: gyro pulses x 2^11                                         */
    word_pair_t vbuf = pair_of(agc_sp_mul(coef, mpac.hi)); /* VBUF: gyro pulses x 2^11 */

    vbuf           = das_low_of_mp(vbuf, mpac.lo, coef);
    agc_sp_t shift = (agc_state_vehicle() == AGC_VEHICLE_LM) ? BIT9 : BIT12; /* MP BIT9 (LM) / BIT12 (CM), B0 */

    g = das(g, agc_sp_mul(vbuf.hi, shift)); /* HI(ADIA)(PIPA)         */
    g = das_low_of_mp(g, vbuf.lo, shift);   /* (ADIA)(PIPAI)          */

    return (g);
}

/**********************************************************************
 * DRFTSUB2 (IMU_COMPENSATION_PACKAGE.agc p.302) -- GCOMPSW > 0 when the
 * command is over 2 pulses
 *
 * CCS GCOMP (the major word as it stands), MASK NEGONE, CCS.
 *
 * ARGUMENTS:
 *   g  word_pair_t  I    GCOMP words of this gyro
 **********************************************************************/
static void torque_needed_check(word_pair_t g)
{
    int hi  = g.hi;                /* CCS GCOMP: the major word as it stands     */
    int mag = (hi < 0) ? -hi : hi; /* |GCOMP major|, pulses */

    if (mag == 0)
    {
        return; /* CCS +0: commands less than 1 pulse */
    }

    int masked = (mag - 1) & ~1; /* MASK NEGONE: drop bit 1 */

    if (masked > 0)
    {
        gcompsw = (agc_sp_t) (masked - 1); /* TS GCOMPSW: positive        */
    }
}

/**********************************************************************
 * DRIFTSUB (IMU_COMPENSATION_PACKAGE.agc p.302) -- GCOMP += NBD x
 * DELTAT
 *
 * ARGUMENTS:
 *   g    word_pair_t  I    GCOMP words of this gyro
 *   nbd  agc_sp_t     I    signed NBD, gyro pulses/cs x 2^-5
 *
 * RETURNS: word_pair_t -- new GCOMP words
 **********************************************************************/
static word_pair_t driftsub(word_pair_t g, agc_sp_t nbd)
{
    agc_dp_t p = agc_sp_mul(nbd, one_pipadt); /* gyro pulses x 2^3       */

    g = das(g, agc_sp_mul(agc_dp_high(p), BIT4)); /* shift right 11        */
    g = das_low_of_mp(g, agc_dp_low(p), BIT4);
    torque_needed_check(g);

    return (g);
}

/**********************************************************************
 * IRIGX / IRIGY / IRIGZ (IMU_COMPENSATION_PACKAGE.agc pp.299-300) --
 * acceleration-sensitive drift of one gyro
 *
 * The ADOA terms are commented out in the AGC.
 *
 * ARGUMENTS:
 *   g     word_pair_t            I    GCOMP words of this gyro
 *   axis  int                    I    0 X, 1 Y, 2 Z
 *   dv    const word_pair_t [3]  I    DELVX..Z words, PIPA pulses x
 *                                     2^14
 *
 * RETURNS: word_pair_t -- new GCOMP words
 **********************************************************************/
static word_pair_t irig_axis(word_pair_t g, int axis, const word_pair_t dv[3])
{
    /* DCS DELVX/Y/Z: both words complemented, PIPA pulses x 2^14       */
    const word_pair_t nx = {sp_neg(dv[0].hi), sp_neg(dv[0].lo)};
    const word_pair_t ny = {sp_neg(dv[1].hi), sp_neg(dv[1].lo)};
    const word_pair_t nz = {sp_neg(dv[2].hi), sp_neg(dv[2].lo)};

    if (axis == 0)
    {
        g = gcompsub(g, nx, comp.adia[0]);          /* -(ADIAX)(PIPAX)   */
        g = gcompsub(g, ny, sp_neg(comp.adsra[0])); /* +(ADSRAX)(PIPAY)  */
    }
    else if (axis == 1)
    {
        g = gcompsub(g, ny, comp.adia[1]);          /* -(ADIAY)(PIPAY)   */
        g = gcompsub(g, nz, sp_neg(comp.adsra[1])); /* +(ADSRAY)(PIPAZ)  */
    }
    else
    {
        g = gcompsub(g, ny, comp.adsra[2]); /* -(ADSRAZ)(PIPAY)  */
        g = gcompsub(g, nz, comp.adia[2]);  /* -(ADIAZ)(PIPAZ)   */
    }

    return (g);
}

/**********************************************************************
 * IRIGCOMP / NBD3 (IMU_COMPENSATION_PACKAGE.agc pp.299-300, 304) -- the
 * bias-drift sign per gyro
 *
 * ARGUMENTS:
 *   axis  int  I    0 X, 1 Y, 2 Z
 *
 * RETURNS: agc_sp_t -- -NBDX, -NBDY or +NBDZ, gyro pulses/cs x 2^-5
 **********************************************************************/
static agc_sp_t nbd_signed(int axis)
{
    agc_sp_t n = comp.nbd[axis];              /* NBD for this gyro, pulses/cs x 2^-5 */
    agc_sp_t r = (axis == 2) ? n : sp_neg(n); /* -NBDX, -NBDY, +NBDZ, 2^-5 */

    if (!AGC_ASSERT(r >= AGC_SP_MIN && r <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return (r);
}

/**********************************************************************
 * IRIGCOMP (IMU_COMPENSATION_PACKAGE.agc pp.299-301) -- gyro
 * compensation from DELV words
 *
 * GCOMPSW = 0; for X, Y, Z the acceleration terms then the bias drift;
 * any command over 2 pulses: NOVAC 1/GYRO at PRIO21.
 *
 * ARGUMENTS:
 *   delv  const word_pair_t [3]  I    DELVX..Z as 1/PIPA left them,
 *                                     PIPA pulses x 2^14
 *
 * RETURNS: agc_status_t -- OK, or the NOVAC status
 **********************************************************************/
static agc_status_t comp_irig_words(const word_pair_t delv[3])
{
    word_pair_t g[3]; /* GCOMP words, gyro pulses x 2^14           */

    if (!AGC_ASSERT(delv != NULL) || (gcomp_read(g) != AGC_STATUS_OK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    gcompsw = 0; /* indicate commands 2 pulses or less */

    for (int axis = 0; axis < 3; axis++)
    {
        g[axis] = irig_axis(g[axis], axis, delv);
        g[axis] = driftsub(g[axis], nbd_signed(axis));
    }

    gcomp_write(g);

    if (gcompsw <= 0)
    {
        return (AGC_STATUS_OK); /* TCF IRIG1      */
    }

    agc_status_t st = agc_exec_novac(PRIO21, AGC_JOB_GYRO_COMMAND_PULSE); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * IRIGCOMP (IMU_COMPENSATION_PACKAGE.agc p.299) -- on a DELV given as
 * values
 *
 * ARGUMENTS:
 *   delv  agc_vec_t  I    DELVX..Z, PIPA pulses x 2^14
 *
 * RETURNS: agc_status_t -- status of IRIGCOMP
 **********************************************************************/
agc_status_t agc_imu_comp_irig(agc_vec_t delv)
{
    const word_pair_t dw[3] = {pair_of(delv.c[0]), pair_of(delv.c[1]), pair_of(delv.c[2])}; /* PIPA pulses x 2^14 */

    if (!AGC_ASSERT(dw[0].hi >= AGC_SP_MIN && dw[2].hi <= AGC_SP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(pair_value(dw[1]) == delv.c[1]))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (comp_irig_words(dw));
}

/**********************************************************************
 * GCOMPZER (IMU_COMPENSATION_PACKAGE.agc p.337 LM) -- GCOMP and GCOMPSW
 * = 0
 *
 * RETURNS: agc_status_t -- OK
 **********************************************************************/
agc_status_t agc_imu_gcompzer(void)
{
    const word_pair_t zero[3] = {{0, 0}, {0, 0}, {0, 0}}; /* GCOMP = 0 */

    gcompsw = 0;
    gcomp_write(zero);

    return (AGC_STATUS_OK);
}

/* ---- 1/GYRO and GCOMP1 --------------------------------------------------- */

/**********************************************************************
 * 1/GYRO +n (IMU_COMPENSATION_PACKAGE.agc p.303) -- IMUPULSE on GCOMP,
 * IMUSTALL, GCOMP1
 *
 * RETURNS: agc_status_t -- OK (asleep or done), or ALARM when caged
 **********************************************************************/
static agc_status_t one_gyro_pulse(void)
{
    agc_status_t st = agc_imu_pulse_buf(AGC_IMU_GBUF_GCOMP, AGC_JOB_GYRO_COMMAND_RETRY); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        /* BUSY: sleeping until the gyros free up; ALARM: IMU caged.      */
        if (!AGC_ASSERT(st == AGC_STATUS_BUSY || st == AGC_STATUS_ALARM))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        return ((st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st); /* BUSY: asleep in GYROBUSY */
    }

    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING; /* IMUSTALL outcome */

    st = agc_imu_stall(AGC_JOB_GYRO_COMMAND_PULSED, &res);

    if (st != AGC_STATUS_OK || res != AGC_IMU_STALL_GOOD)
    {
        return (st); /* bad return: ENDOFJOB; or sleeping   */
    }

    if (!AGC_ASSERT(agc_imu_stall_outcome(AGC_IMU_DEV_IMU) == AGC_IMU_STALL_GOOD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_gyro_command_pulsed());
}

/**********************************************************************
 * 1/GYRO (IMU_COMPENSATION_PACKAGE.agc pp.302-303) -- job: scale GCOMP
 * for IMUPULSE and torque
 *
 * GCOMP (gyro pulses x 2^14) shifted right 7 into the IMUPULSE scale
 * (revolutions, 2^21 pulses per revolution).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- status of the torquing
 **********************************************************************/
agc_status_t agc_job_gyro_command_pulse(void)
{
    word_pair_t g[3]; /* GCOMP words, gyro pulses x 2^14           */

    if (gcomp_read(g) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int axis = 2; axis >= 0; axis--)
    { /* PIPAZ, Y, X        */
        /* 1/GYRO scaling (ICP p.333): GCOMP (gyro pulses x 2^14) shifted right 7
         * into the IMUPULSE scale (revolutions, 2^21 pulses per revolution).     */
        const word_pair_t gw = g[axis]; /* GCOMP +2axis, pulses x 2^14 */
        word_pair_t       r  = {0, agc_dp_high(agc_sp_mul(gw.lo, BIT8))};

        r = das(r, agc_sp_mul(gw.hi, BIT8)); /* XCH GCOMP, MP BIT8, DAS */

        g[axis] = r;
    }

    gcomp_write(g);

    return (one_gyro_pulse());
}

/**********************************************************************
 * 1/GYRO after GYROBUSY (IMU_COMPENSATION_PACKAGE.agc p.303) -- job:
 * IMUPULSE again
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the woken job
 *
 * RETURNS: agc_status_t -- status of the torquing
 **********************************************************************/
agc_status_t agc_job_gyro_command_retry(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id == AGC_JOB_GYRO_COMMAND_RETRY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (one_gyro_pulse());
}

/**********************************************************************
 * GCOMP1 (IMU_COMPENSATION_PACKAGE.agc p.303) -- job: good IMUSTALL
 * return, rescale GCOMP
 *
 * The fraction left in the minor word goes back to gyro pulses x 2^14
 * (shift left 7).  Bad return: ENDOFJOB.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK
 **********************************************************************/
agc_status_t agc_job_gyro_command_pulsed(void)
{
    if (agc_imu_stall_outcome(AGC_IMU_DEV_IMU) != AGC_IMU_STALL_GOOD)
    {
        return (AGC_STATUS_OK); /* L+2: TCF ENDOFJOB      */
    }

    word_pair_t g[3]; /* GCOMP words after IMUPULSE                */

    if (gcomp_read(g) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int axis = 2; axis >= 0; axis--)
    {
        /* Bits 8-14 of the minor part were zero, so L holds it all.      */
        g[axis].lo = agc_dp_low(agc_sp_mul(g[axis].lo, BIT8));
    }

    gcomp_write(g);

    return (AGC_STATUS_OK);
}

/* ---- NBDONLY / FBIASSUB / LASTBIAS -------------------------------------- */

/**********************************************************************
 * NBD2 (IMU_COMPENSATION_PACKAGE.agc p.304) -- elapsed TIME1 modulo the
 * 14-bit counter
 *
 * LM: AD HALF twice and drop the overflow; CM: CCS ... AD POSMAX.
 *
 * ARGUMENTS:
 *   now   int  I    TIME1 now, cs
 *   prev  int  I    TIME1 before, cs
 *
 * RETURNS: agc_sp_t -- DELTAT, cs B0
 **********************************************************************/
static agc_sp_t elapsed_cs(int now, int prev)
{
    int dt = (now - prev) % TIME1_MODULUS; /* elapsed TIME1, cs */

    if (dt < 0)
    {
        dt += TIME1_MODULUS;
    }

    return ((agc_sp_t) (dt & AGC_SP_MAX));
}

/**********************************************************************
 * NBD2, CM (IMU_COMPENSATION_PACKAGE.agc p.304) -- equal TIME1 readings
 * end the job
 *
 * COM prev, AD now, CCS: equal readings give -0 and the CM job ends with
 * no compensation (GCOMPSW untouched).  The LM's AD HALF, AD HALF
 * instead turns -0 into a zero DELTAT and goes on.
 *
 * ARGUMENTS:
 *   now   int  I    TIME1 now, cs
 *   prev  int  I    TIME1 before, cs
 *
 * RETURNS: bool -- true for CM and now == prev
 **********************************************************************/
static bool cm_no_time_elapsed(int now, int prev)
{
    return ((agc_state_vehicle() == AGC_VEHICLE_CSM) && (now == prev));
}

/**********************************************************************
 * FBIASSUB (IMU_COMPENSATION_PACKAGE.agc p.306) -- GCOMP += NBD x
 * DELTAT, then DRFTSUB2
 *
 * ARGUMENTS:
 *   g       word_pair_t  I    GCOMP words of this gyro
 *   nbd     agc_sp_t     I    signed NBD, gyro pulses/cs x 2^-5
 *   deltat  agc_dp_t     I    DELTAT, cs x 2^19
 *
 * RETURNS: word_pair_t -- new GCOMP words
 **********************************************************************/
static word_pair_t fbiassub(word_pair_t g, agc_sp_t nbd, agc_dp_t deltat)
{
    g = das(g, agc_sp_mul(nbd, agc_dp_high(deltat))); /* HI(NBD)(DELTAT) */
    g = das_low_of_mp(g, nbd, agc_dp_low(deltat));
    torque_needed_check(g);                           /* TCF DRFTSUB2            */

    return (g);
}

/**********************************************************************
 * NBD3 (IMU_COMPENSATION_PACKAGE.agc pp.304-305) -- bias drift over dt
 * cs, then 1/GYRO if needed
 *
 * LM on the surface also compensates the acceleration terms.
 *
 * ARGUMENTS:
 *   dt       agc_sp_t         I    elapsed time, cs B0
 *   surface  bool             I    LM on the lunar surface
 *
 * RETURNS: agc_status_t -- OK (ENDOFJOB) or the 1/GYRO status
 **********************************************************************/
static agc_status_t bias_drift_compensate(agc_sp_t dt, bool surface)
{
    agc_dp_t    deltat = agc_sp_mul(dt, BIT10); /* cs x 2^19 (SR 5)    */
    word_pair_t g[3];                           /* GCOMP words, gyro pulses x 2^14           */

    if (gcomp_read(g) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_vec_t   dv      = agc_erase_delv();                                       /* PIPA pulses x 2^14 */
    const word_pair_t delv[3] = {pair_of(dv.c[0]), pair_of(dv.c[1]), pair_of(dv.c[2])}; /* DELVX..Z words */

    gcompsw = 0; /* indicate commands 2 pulses or less */

    for (int axis = 0; axis < 3; axis++)
    {
        if (surface)
        {
            g[axis] = irig_axis(g[axis], axis, delv);
        }

        /* X uses +DELTAT with -NBDX; Y, Z use -DELTAT with +NBDY, -NBDZ:
         * the products are -NBDX dt, -NBDY dt, +NBDZ dt.                 */
        g[axis] = fbiassub(g[axis], nbd_signed(axis), deltat);
    }

    gcomp_write(g);

    if (gcompsw > 0)
    {
        return (agc_job_gyro_command_pulse()); /* TCF 1/GYRO              */
    }

    return (AGC_STATUS_OK); /* ENDOFJOB                */
} /* bias_drift_compensate */

/**********************************************************************
 * NBDONLY (IMU_COMPENSATION_PACKAGE.agc p.304) -- job: bias-drift
 * compensation when the PIPAs are not in use
 *
 * Only with DRIFTFLG set; TIME1 is snapshotted into 1/PIPADT.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the NBDONLY job
 *
 * RETURNS: agc_status_t -- OK or the 1/GYRO status
 **********************************************************************/
agc_status_t agc_job_nbdonly(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || gcomp_bypassed())
    {
        return (AGC_STATUS_OK);
    }

    agc_inhint();

    if (!agc_flag_test(AGC_FLAG_DRIFTFLG))
    { /* CCS FLAGWRD2: bit 15     */
        agc_relint();

        return (AGC_STATUS_OK); /* preread T3RUPT coincides */
    }

    bool surface = (agc_state_vehicle() == AGC_VEHICLE_LM) && agc_flag_test(AGC_FLAG_SURFFLAG); /* LM on the lunar surface */

    if (surface)
    {
        agc_pipasr_noscale(); /* PIPASR +3: no scaling    */
    }

    int now  = (int) (agc_time_now_cs() & (TIME1_MODULUS - 1)); /* TIME1   */
    int prev = one_pipadt;                                      /* 1/PIPADT: previous TIME1, cs */

    one_pipadt = (agc_sp_t) now; /* XCH 1/PIPADT             */
    agc_relint();

    if (!AGC_ASSERT(now >= 0 && now < TIME1_MODULUS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (cm_no_time_elapsed(now, prev))
    {
        return (AGC_STATUS_OK); /* NBD2: difference -0, ENDOFJOB */
    }

    return (bias_drift_compensate(elapsed_cs(now, prev), surface));
}

/**********************************************************************
 * LASTBIAS (IMU_COMPENSATION_PACKAGE.agc p.306) -- job: last bias
 * compensation as the PIPAs come into use
 *
 * CM BANKCALL PIPUSE (PIPAs zeroed), LM PIPUSE1; 1/PIPADT reset to 2 s.
 *
 * ARGUMENTS:
 *
 * RETURNS: agc_status_t -- OK or the 1/GYRO status
 **********************************************************************/
agc_status_t agc_job_lastbias(void)
{
    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        agc_status_t pu = agc_imu_pipuse(); /* CM: BANKCALL PIPUSE */

        if (!AGC_ASSERT(pu == AGC_STATUS_OK))
        {
            return (pu);
        }
    }
    else
    {
        agc_imu_pipa_fail_enable(); /* LM: BANKCALL PIPUSE1   */
    }

    if (gcomp_bypassed())
    {
        return (AGC_STATUS_OK);
    }

    bool surface = (agc_state_vehicle() == AGC_VEHICLE_LM) && agc_flag_test(AGC_FLAG_SURFFLAG); /* LM on the lunar surface */
    int  prev    = one_pipadt;                                                                  /* 1/PIPADT: previous TIME1, cs */

    one_pipadt = PRIO31_2SEC; /* 2 seconds, cs x 2^8      */

    /* PIPTIME +1 (LM) / PIPTIME1 +1 (CM): low word of the PIPA time.     */
    int now = agc_dp_low(agc_erase_piptime()); /* PIPTIME +1, cs */

    if (cm_no_time_elapsed(now, prev))
    {
        return (AGC_STATUS_OK); /* TCF NBD2: difference -0, ENDOFJOB */
    }

    return (bias_drift_compensate(elapsed_cs(now, prev), surface));
} /* agc_job_lastbias */
