/*
 * compensation.c -- IMU compensation package.
 *
 * Source: IMU_COMPENSATION_PACKAGE.agc, Comanche055 pp.297-306 and
 *         Luminary099 pp.326-337 (diffed; differences below).
 *
 * The package corrects the PIPA counts for scale-factor error and bias
 * (1/PIPA), computes the gyro compensation for acceleration-sensitive drift
 * (ADIA, ADSRA) and bias drift (NBD) into GCOMP (IRIGCOMP, NBDONLY), and
 * torques the gyros when any command exceeds two pulses (1/GYRO).
 *
 * All of this is basic (machine) code built from MP / DAS sequences.  It is
 * translated word for word with agc_sp_mul (MP), agc_dp_high/agc_dp_low
 * (the A and L halves of the product) and agc_dp_add_raw (DAS), so the
 * truncation of every partial product is the AGC's.
 *
 * Scales (from the AGC comments):
 *   DELVX..Z   PIPA pulses x 2^14 (DP; high word = whole pulses)
 *   PIPASCF    (ppm) x 2^-9
 *   PIPABIAS   (PIPA pulses)/cs x 2^-5 (LM) / 2^-8 (CM)
 *   1/PIPADT   cs x 2^8
 *   NBD        (gyro pulses)/cs x 2^-5
 *   ADIA ADSRA (gyro pulses)/(PIPA pulse) x 2^-6 (LM) / 2^-3 (CM)
 *   GCOMP      gyro pulses x 2^14 (DP), before 1/GYRO rescales it to
 *              revolutions for IMUPULSE (pulses x 2^7 raw)
 *
 * CM vs LM differences:
 *   * PIPA bias term: LM MP BIT4 (shift right 11), CM MP BIT1 (shift 14).
 *   * GCOMPSUB: LM MP BIT9 (shift right 6), CM MP BIT12 (shift right 3).
 *   * CM 1/PIPA runs under INHINT; LM does not.
 *   * NBDONLY/LASTBIAS: LM also compensates acceleration terms when
 *     SURFFLAG is set (LM on the lunar surface) and reads the PIPAs via
 *     PIPASR +3; CM computes the elapsed time with CCS instead of the
 *     HALF+HALF overflow trick (same result).
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "pending.h"

/* Powers of two used as MP multipliers (BITn = 2^(n-1)).                 */
#define BIT4  ((agc_sp_t) 00010)
#define BIT6  ((agc_sp_t) 00040)
#define BIT8  ((agc_sp_t) 00200)
#define BIT9  ((agc_sp_t) 00400)
#define BIT10 ((agc_sp_t) 01000)
#define BIT12 ((agc_sp_t) 04000)
#define BIT1  ((agc_sp_t) 00001)

#define TIME1_MODULUS 040000 /* TIME1 is a 14-bit counter (cs)         */
#define PRIO21        ((agc_prio_t) 021) /* priority greater than SERVICER         */
#define PRIO31_2SEC   ((agc_sp_t) 031000) /* 2 s scaled cs x 2^8 (PRIO31)     */

/* Pad-loaded compensation parameters (module-private erasables).         */
static agc_imu_comp_params_t comp;
static agc_sp_t gcompsw;    /* GCOMPSW: <0 bypass, >0 torque needed  */
static agc_sp_t one_pipadt; /* 1/PIPADT cs x 2^8, or TIME1 snapshot  */

/* ---- small word-level helpers ------------------------------------------ */

/* DAS: acc = acc + x (DP).  The overflow out of a DAS is not used by the
 * compensation code.                                                     */
static agc_dp_t das(agc_dp_t acc, agc_dp_t x)
{
    agc_ovf_t ovf = 0;
    agc_dp_t  r   = agc_dp_add_raw(acc, x, &ovf);
    if (!AGC_ASSERT(acc >= AGC_DP_MIN && acc <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    if (!AGC_ASSERT(ovf >= -1 && ovf <= 1))
    {
        return 0; /* no valid result */
    }
    return r;
}

/* "MP X; TS L; CAF ZERO; DAS acc": add the high half of a*b into the low
 * word of the accumulator.                                               */
static agc_dp_t das_low_of_mp(agc_dp_t acc, agc_sp_t a, agc_sp_t b)
{
    agc_sp_t hi = agc_dp_high(agc_sp_mul(a, b));
    if (!AGC_ASSERT(a >= AGC_SP_MIN && a <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    if (!AGC_ASSERT(b >= AGC_SP_MIN && b <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    return das(acc, agc_dp_pack(0, hi));
}

static agc_sp_t sp_neg(agc_sp_t a)
{
    if (!AGC_ASSERT(a >= AGC_SP_MIN && a <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    agc_sp_t r = (agc_sp_t) (-a);
    if (!AGC_ASSERT(r >= AGC_SP_MIN && r <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    return r;
}

static bool params_in_range(const agc_imu_comp_params_t *p)
{
    bool ok = true;
    for (int i = 0; i < 3; i++)
    {
        ok = ok && p->pipabias[i] >= AGC_SP_MIN &&
             p->pipascf[i] >= AGC_SP_MIN && p->nbd[i] >= AGC_SP_MIN &&
             p->adia[i] >= AGC_SP_MIN && p->adsra[i] >= AGC_SP_MIN;
    }
    if (!AGC_ASSERT(ok))
    {
        return false;
    }
    return ok;
}

/* ---- parameter access --------------------------------------------------- */

agc_status_t agc_imu_comp_load(const agc_imu_comp_params_t *p)
{
    if (!AGC_ASSERT(p != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(params_in_range(p)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    comp = *p;
    return AGC_STATUS_OK;
}

agc_status_t agc_imu_comp_get(agc_imu_comp_params_t *p)
{
    if (!AGC_ASSERT(p != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    *p = comp;
    if (!AGC_ASSERT(params_in_range(p)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

agc_sp_t agc_imu_gcompsw(void)
{
    if (!AGC_ASSERT(gcompsw >= AGC_SP_MIN && gcompsw <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    return gcompsw;
}

void agc_imu_set_gcompsw(agc_sp_t v)
{
    if (AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        gcompsw = v;
    }
}

agc_sp_t agc_imu_pipa_interval(void)
{
    if (!AGC_ASSERT(one_pipadt >= AGC_SP_MIN && one_pipadt <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    return one_pipadt;
}

void agc_imu_set_pipa_interval(agc_sp_t v)
{
    if (AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        one_pipadt = v;
    }
}

/* ---- GCOMP access (the GCOMP gyro buffer lives in mode.c) ----------- */

static agc_status_t gcomp_read(agc_dp_t g[3])
{
    agc_status_t st = agc_imu_gyro_buf_get(AGC_IMU_GBUF_GCOMP, g);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(g != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

static void gcomp_write(const agc_dp_t g[3])
{
    if (!AGC_ASSERT(g != NULL))
    {
        return;
    }
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_GCOMP, g);
}

/* GCOMPSW bypass test: "CCS GCOMPSW / TCF +3 / TCF +2 / TCF exit": the
 * compensation is bypassed only when GCOMPSW is negative.                */
static bool gcomp_bypassed(void)
{
    if (!AGC_ASSERT(gcompsw >= AGC_SP_MIN && gcompsw <= AGC_SP_MAX))
    {
        return false;
    }
    bool bypass = (gcompsw < 0);
    if (!AGC_ASSERT(bypass || gcompsw >= 0))
    {
        return false;
    }
    return bypass;
}

/* ---- 1/PIPA ------------------------------------------------------------- */

/* 1/PIPA1 body for one axis (ICP p.326):
 *   DELV = DELV + DELV*SFE - BIAS*DELTAT, all in PIPA pulses x 2^14.    */
static agc_dp_t pipa_axis(agc_dp_t delv, int axis)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return 0; /* no valid result */
    }
    agc_sp_t pulses = agc_dp_high(delv);
    agc_dp_t p = agc_sp_mul(comp.pipascf[axis], pulses); /* (PP) x 2^5    */
    /* Minor part, shifted right 9: fractional PIPA pulses x 2^14.        */
    agc_sp_t frac = agc_dp_high(agc_sp_mul(agc_dp_low(p), BIT6));
    agc_dp_t d    = agc_dp_pack(pulses, frac);    /* TS DELVX +1    */
    d = das(d, agc_sp_mul(agc_dp_high(p), BIT6)); /* (PIPAI)(SFE)   */
    /* -(BIAS)(DELTAT): CS PIPABIAS, MP 1/PIPADT, MP BIT4 (LM) / BIT1 (CM). */
    agc_dp_t b     = agc_sp_mul(sp_neg(comp.pipabias[axis]), one_pipadt);
    agc_sp_t shift = (agc_state_vehicle() == AGC_VEHICLE_LM) ? BIT4 : BIT1;
    d              = das(d, agc_sp_mul(agc_dp_high(b), shift));
    if (!AGC_ASSERT(d >= AGC_DP_MIN && d <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    return d;
}

/* 1/PIPA (ICP p.326, label 1/PIPA): compensate the PIPA counts in DELV for
 * scale factor and bias, then fall into IRIGCOMP.  Order Z, Y, X as the
 * AGC's 4, 2, -0 index loop.                                             */
agc_status_t agc_imu_one_over_pipa(agc_vec_t *delv)
{
    if (!AGC_ASSERT(delv != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (gcomp_bypassed())
    {
        return AGC_STATUS_OK; /* TCF IRIG1      */
    }
    bool cm = (agc_state_vehicle() == AGC_VEHICLE_CSM);
    if (cm)
    {
        agc_inhint(); /* CM: assure complete compensation of DELVs */
    }
    for (int axis = 2; axis >= 0; axis--)
    {
        delv->c[axis] = pipa_axis(delv->c[axis], axis);
    }
    agc_status_t st = agc_imu_comp_irig(*delv);
    if (cm)
    {
        agc_relint();
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* ---- gyro compensation terms ------------------------------------------- */

/* GCOMPSUB (ICP p.331): GCOMP[axis] += coef * mpac, where mpac is PIPA
 * pulses x 2^14 and coef the ADIA/ADSRA coefficient.  The product is
 * shifted right 6 (LM, MP BIT9) or 3 (CM, MP BIT12) into gyro pulses
 * x 2^14.                                                                */
static agc_dp_t gcompsub(agc_dp_t g, agc_dp_t mpac, agc_sp_t coef)
{
    if (!AGC_ASSERT(coef >= AGC_SP_MIN && coef <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    agc_dp_t vbuf  = agc_sp_mul(coef, agc_dp_high(mpac)); /* gyro pls x 2^8 */
    vbuf           = das_low_of_mp(vbuf, agc_dp_low(mpac), coef);
    agc_sp_t shift = (agc_state_vehicle() == AGC_VEHICLE_LM) ? BIT9 : BIT12;
    g = das(g, agc_sp_mul(agc_dp_high(vbuf), shift)); /* HI(ADIA)(PIPA) */
    g = das_low_of_mp(g, agc_dp_low(vbuf), shift);    /* (ADIA)(PIPAI)  */
    if (!AGC_ASSERT(g >= AGC_DP_MIN && g <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    return g;
}

/* DRFTSUB2 magnitude test (ICP p.332): set GCOMPSW positive when the high
 * word of GCOMP for this axis is 3 pulses or more ("MASK COMPCHK" drops
 * bit 1 of |GCOMP|-1).                                                   */
static void drftsub2(agc_dp_t g)
{
    int hi  = agc_dp_high(g);
    int mag = (hi < 0) ? -hi : hi;
    if (!AGC_ASSERT(mag <= AGC_SP_MAX))
    {
        return;
    }
    if (mag == 0)
    {
        return; /* CCS +0: commands less than 1 pulse */
    }
    int masked = (mag - 1) & ~1;
    if (masked > 0)
    {
        gcompsw = (agc_sp_t) (masked - 1); /* TS GCOMPSW: positive        */
    }
    if (!AGC_ASSERT(gcompsw >= AGC_SP_MIN))
    {
        return;
    }
}

/* DRIFTSUB (ICP p.332): GCOMP[axis] += NBD * DELTAT where DELTAT is
 * 1/PIPADT (cs x 2^8); NBD arrives with the sign wanted.                 */
static agc_dp_t driftsub(agc_dp_t g, agc_sp_t nbd)
{
    if (!AGC_ASSERT(nbd >= AGC_SP_MIN && nbd <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    agc_dp_t p = agc_sp_mul(nbd, one_pipadt);     /* gyro pulses x 2^3       */
    g = das(g, agc_sp_mul(agc_dp_high(p), BIT4)); /* shift right 11        */
    g = das_low_of_mp(g, agc_dp_low(p), BIT4);
    drftsub2(g);
    if (!AGC_ASSERT(g >= AGC_DP_MIN && g <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    return g;
}

/* IRIGX / IRIGY / IRIGZ (ICP p.329-330): acceleration-sensitive drift
 * terms for one gyro.  The ADOA terms are commented out in the AGC.      */
static agc_dp_t irig_axis(agc_dp_t g, int axis, agc_vec_t delv)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return 0; /* no valid result */
    }
    agc_dp_t nx = agc_dp_neg(delv.c[0]);
    agc_dp_t ny = agc_dp_neg(delv.c[1]);
    agc_dp_t nz = agc_dp_neg(delv.c[2]);
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
    if (!AGC_ASSERT(g >= AGC_DP_MIN && g <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    return g;
}

/* Bias-drift sign per axis in IRIGCOMP and NBDONLY: -NBDX, -NBDY, +NBDZ. */
static agc_sp_t nbd_signed(int axis)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return 0; /* no valid result */
    }
    agc_sp_t n = comp.nbd[axis];
    agc_sp_t r = (axis == 2) ? n : sp_neg(n);
    if (!AGC_ASSERT(r >= AGC_SP_MIN && r <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    return r;
}

/* IRIGCOMP (ICP p.328): GCOMPSW = 0; for X, Y, Z: acceleration terms then
 * bias drift; if any command exceeds 2 pulses, NOVAC 1/GYRO at PRIO21.   */
agc_status_t agc_imu_comp_irig(agc_vec_t delv)
{
    agc_dp_t g[3];
    if (gcomp_read(g) != AGC_STATUS_OK)
    {
        return AGC_STATUS_BAD_ARG;
    }
    gcompsw = 0; /* indicate commands 2 pulses or less */
    for (int axis = 0; axis < 3; axis++)
    {
        g[axis] = irig_axis(g[axis], axis, delv);
        g[axis] = driftsub(g[axis], nbd_signed(axis));
    }
    gcomp_write(g); /* buffer id is valid (rule 7)       */
    if (gcompsw <= 0)
    {
        return AGC_STATUS_OK; /* TCF IRIG1      */
    }
    agc_status_t st = agc_exec_novac(PRIO21, AGC_JOB_GYRO_COMMAND_PULSE);
    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* GCOMPZER (ICP p.337): zero GCOMP and GCOMPSW before the first 1/PIPA.  */
agc_status_t agc_imu_gcompzer(void)
{
    agc_dp_t zero[3] = {0, 0, 0};
    gcompsw          = 0;
    gcomp_write(zero);
    if (!AGC_ASSERT(gcompsw == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* ---- 1/GYRO and GCOMP1 --------------------------------------------------- */

/* 1/GYRO scaling (ICP p.333): GCOMP (gyro pulses x 2^14) shifted right 7
 * into the IMUPULSE scale (revolutions, 2^21 pulses per revolution).     */
static agc_dp_t gyro_rescale(agc_dp_t g)
{
    agc_sp_t lo = agc_dp_high(agc_sp_mul(agc_dp_low(g), BIT8)); /* SR 7   */
    agc_dp_t r  = agc_dp_pack(0, lo);
    r = das(r, agc_sp_mul(agc_dp_high(g), BIT8)); /* add whole pulses   */
    if (!AGC_ASSERT(g >= AGC_DP_MIN && g <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    if (!AGC_ASSERT(r >= AGC_DP_MIN && r <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    return r;
}

/* Common tail of 1/GYRO: IMUPULSE on GCOMP, then IMUSTALL.               */
static agc_status_t one_gyro_pulse(void)
{
    agc_status_t st =
        agc_imu_pulse_buf(AGC_IMU_GBUF_GCOMP, AGC_JOB_GYRO_COMMAND_RETRY);
    if (st != AGC_STATUS_OK)
    {
        /* BUSY: sleeping until the gyros free up; ALARM: IMU caged.      */
        if (!AGC_ASSERT(st == AGC_STATUS_BUSY || st == AGC_STATUS_ALARM))
        {
            return AGC_STATUS_BAD_ARG;
        }
        return (st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st;
    }
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;
    st                         = agc_imu_stall(AGC_JOB_GCOMP1, &res);
    if (st != AGC_STATUS_OK || res != AGC_IMU_STALL_GOOD)
    {
        return st; /* bad return: ENDOFJOB; or sleeping   */
    }
    agc_job_ctx_t self = {AGC_JOB_GCOMP1, PRIO21, false, -1, 0};
    return agc_job_gcomp1(&self);
}

/* 1/GYRO (ICP p.333): job started by IRIGCOMP or NBDONLY.                */
agc_status_t agc_job_gyro_command_pulse(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t g[3];
    if (gcomp_read(g) != AGC_STATUS_OK)
    {
        return AGC_STATUS_BAD_ARG;
    }
    for (int axis = 2; axis >= 0; axis--)
    { /* PIPAZ, Y, X        */
        g[axis] = gyro_rescale(g[axis]);
    }
    gcomp_write(g);
    return one_gyro_pulse();
}

/* Continuation of 1/GYRO after GYROBUSY sleep: retry IMUPULSE.           */
agc_status_t agc_job_gyro_command_retry(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(ctx->id == AGC_JOB_GYRO_COMMAND_RETRY))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return one_gyro_pulse();
}

/* GCOMP1 (ICP p.333): good return from IMUSTALL.  Rescale the fraction
 * left in the minor part back to gyro pulses x 2^14 (shift left 7).      */
agc_status_t agc_job_gcomp1(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_imu_stall_outcome(AGC_IMU_DEV_IMU) != AGC_IMU_STALL_GOOD)
    {
        return AGC_STATUS_OK; /* L+2: TCF ENDOFJOB      */
    }
    agc_dp_t g[3];
    if (gcomp_read(g) != AGC_STATUS_OK)
    {
        return AGC_STATUS_BAD_ARG;
    }
    for (int axis = 2; axis >= 0; axis--)
    {
        /* Bits 8-14 of the minor part were zero, so L holds it all.      */
        agc_sp_t lo = agc_dp_low(agc_sp_mul(agc_dp_low(g[axis]), BIT8));
        g[axis]     = agc_dp_pack(agc_dp_high(g[axis]), lo);
    }
    gcomp_write(g);
    return AGC_STATUS_OK;
}

/* ---- NBDONLY / FBIASSUB / LASTBIAS -------------------------------------- */

/* Elapsed TIME1 interval, modulo the 14-bit counter: NBD2 (LM adds HALF
 * twice and drops the overflow; CM uses CCS ... AD POSMAX).              */
static agc_sp_t elapsed_cs(int now, int prev)
{
    if (!AGC_ASSERT(now >= -TIME1_MODULUS && now < TIME1_MODULUS))
    {
        return 0; /* no valid result */
    }
    int dt = (now - prev) % TIME1_MODULUS;
    if (dt < 0)
    {
        dt += TIME1_MODULUS;
    }
    if (!AGC_ASSERT(dt >= 0 && dt < TIME1_MODULUS))
    {
        return 0; /* no valid result */
    }
    return (agc_sp_t) (dt & AGC_SP_MAX);
}

/* FBIASSUB (ICP p.336): GCOMP[axis] += NBD * DELTAT, DELTAT in cs x 2^19. */
static agc_dp_t fbiassub(agc_dp_t g, agc_sp_t nbd, agc_dp_t deltat)
{
    if (!AGC_ASSERT(nbd >= AGC_SP_MIN && nbd <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    g = das(g, agc_sp_mul(nbd, agc_dp_high(deltat))); /* HI(NBD)(DELTAT) */
    g = das_low_of_mp(g, nbd, agc_dp_low(deltat));
    drftsub2(g); /* TCF DRFTSUB2            */
    if (!AGC_ASSERT(g >= AGC_DP_MIN && g <= AGC_DP_MAX))
    {
        return 0; /* no valid result */
    }
    return g;
}

/* NBD3 onward (ICP p.334-335): bias drift over dt cs (and, LM on the
 * surface, the acceleration terms); 1/GYRO if any command > 2 pulses.    */
static agc_status_t nbd3(agc_sp_t dt, bool surface, agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(dt >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t deltat = agc_sp_mul(dt, BIT10); /* cs x 2^19 (shift right 5) */
    agc_dp_t g[3];
    if (gcomp_read(g) != AGC_STATUS_OK)
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t delv = agc_erase_delv();
    gcompsw        = 0; /* indicate commands 2 pulses or less */
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
        return agc_job_gyro_command_pulse(ctx); /* TCF 1/GYRO              */
    }
    if (!AGC_ASSERT(gcompsw == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK; /* ENDOFJOB                */
}

/* NBDONLY (ICP p.334): bias-drift-only compensation when the PIPAs are not
 * in use (DRIFTFLG set); TIME1 is snapshotted into 1/PIPADT.             */
agc_status_t agc_job_nbdonly(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || gcomp_bypassed())
    {
        return AGC_STATUS_OK;
    }
    agc_inhint();
    if (!agc_flag_test(AGC_FLAG_DRIFTFLG))
    { /* CCS FLAGWRD2: bit 15     */
        agc_relint();
        return AGC_STATUS_OK; /* preread T3RUPT coincides */
    }
    bool surface = (agc_state_vehicle() == AGC_VEHICLE_LM) &&
                   agc_flag_test(AGC_FLAG_SURFFLAG);
    if (surface)
    {
        agc_pfs_pipasr_noscale(); /* PIPASR +3: no scaling    */
    }
    int now    = (int) (agc_time_now_cs() & (TIME1_MODULUS - 1)); /* TIME1   */
    int prev   = one_pipadt;
    one_pipadt = (agc_sp_t) now; /* XCH 1/PIPADT             */
    agc_relint();
    return nbd3(elapsed_cs(now, prev), surface, ctx);
}

/* LASTBIAS (ICP p.336-337): final bias compensation when the PIPAs come
 * into use; 1/PIPADT is reset to 2 seconds.                              */
agc_status_t agc_job_lastbias(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_imu_pipuse1(); /* BANKCALL PIPUSE1         */
    if (gcomp_bypassed())
    {
        return AGC_STATUS_OK;
    }
    bool surface = (agc_state_vehicle() == AGC_VEHICLE_LM) &&
                   agc_flag_test(AGC_FLAG_SURFFLAG);
    int prev   = one_pipadt;
    one_pipadt = PRIO31_2SEC; /* 2 seconds, cs x 2^8      */
    /* PIPTIME +1 (LM) / PIPTIME1 +1 (CM): low word of the PIPA time.     */
    int now = agc_dp_low(agc_erase_piptime());
    if (!AGC_ASSERT(now >= AGC_SP_MIN))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return nbd3(elapsed_cs(now, prev), surface, ctx);
}
