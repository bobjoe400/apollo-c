/*
 * prelaunch_align.c -- Comanche prelaunch alignment and IMU test pieces.
 *
 * Source: Comanche055 IMU_CALIBRATION_AND_ALIGNMENT.agc pp.423-455 ("IMU
 * performance tests 2" and "optimum prelaunch alignment calibration", the
 * P01-P03 gyrocompass).  CM only; the Luminary P07 equivalents belong to
 * sys-misc (src/lm/imu_tests.c).
 *
 * Translated: FINETIME (p.431), ERTHRVSE / EARTHR / OVERFFIX (pp.428-430),
 * POSN17C (p.443).  The test sequencer, the gyrocompass filter and the
 * optical verification are stubs (see docs/modules/imu.md "Stubs").
 *
 * Scales: LATITUDE and azimuth in revolutions (2^0); ERVECTOR earth rate
 * per (centisecond x 2^-19) in revolutions (OMEG/MS); ERCOMP1 is the
 * ERCOMP gyro command buffer of imu_gyro.c, DP revolutions.
 */
#include <stddef.h>
#include "agc/agc_assert.h"
#include "agc/agc_status.h"
#include "agc/agc_word.h"
#include "agc/agc_interp.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"
#include "agc/agc_channels.h"
#include "agc/agc_imu.h"
#include "../imu/imu_pending.h"

#define OMEG_MS   ((agc_dp_t)65334634)  /* 2DEC .24339048 (OMEG/MS)        */
#define POSMAX_W  037777u               /* POSMAX                         */
#define FINETIME_TRIES 4                /* bound on the re-read loop       */
#define TIME_WRAP ((agc_dp_t)0x10000000L) /* DPPOSMAX + ONEDPP = 2^28 cs   */

static agc_vec_t ervector;    /* ERVECTOR: earth rate in local axes       */
static agc_dp_t  tmark;       /* TMARK: time of the last EARTHR, cs 2^28  */

agc_vec_t agc_csm_prelaunch_ervector(void)
{
    (void)AGC_ASSERT(tmark >= 0);
    (void)AGC_ASSERT(ervector.c[2] == 0);
    return ervector;
}

/* FINETIME (p.431): read LOSCALAR twice until two reads agree, retry
 * while it reads POSMAX (about to carry into HISCALAR), then read
 * HISCALAR.  Returns with interrupts inhibited.  DEVIATION: the AGC loops
 * without limit; here at most FINETIME_TRIES times.                      */
agc_status_t agc_csm_finetime(uint16_t *hiscalar, uint16_t *loscalar)
{
    if (!AGC_ASSERT(hiscalar != NULL && loscalar != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    uint16_t l = 0;
    for (int i = 0; i < FINETIME_TRIES; i++) {
        l = agc_chan_read(AGC_CHAN_LOSCALAR);
        if (agc_chan_read(AGC_CHAN_LOSCALAR) != l) {     /* RXOR, BZF   */
            l = agc_chan_read(AGC_CHAN_LOSCALAR);
        }
        if (l != POSMAX_W) {
            break;
        }
    }
    *loscalar = l;
    *hiscalar = agc_chan_read(AGC_CHAN_HISCALAR);
    (void)AGC_ASSERT(*loscalar <= 077777u);
    return AGC_STATUS_OK;
}

/* POSN17C (p.443): compass position, Z down, X downrange:
 *   XSM = (0, -cos AZ, sin AZ)/2, YSM = (0, sin AZ, cos AZ)/2,
 *   ZSM = (-1/2, 0, 0)  (the rest zeroed by ZEROING in SETUPGC).         */
agc_status_t agc_csm_prelaunch_posn17c(agc_interp_t *ip, agc_dp_t azimuth,
                                       agc_mat_t *xsm)
{
    if (!AGC_ASSERT(ip != NULL && xsm != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(azimuth >= AGC_DP_MIN && azimuth <= AGC_DP_MAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t s = agc_dp_sin(ip, azimuth);
    agc_dp_t c = agc_dp_cos(ip, azimuth);
    xsm->row[0] = agc_vec_make(0, agc_dp_neg(c), s);
    xsm->row[1] = agc_vec_make(0, s, c);
    xsm->row[2] = agc_vec_make(-AGC_DP_HALF, 0, 0);       /* CS HALF      */
    return AGC_STATUS_OK;
}

/* ERTHRVSE (p.430): ERVECTOR = (SIN LAT, -COS LAT, 0) OMEG/MS;
 * TMARK = present time; ERCOMP1 = 0.                                     */
agc_status_t agc_csm_prelaunch_erthrvse(agc_interp_t *ip, agc_dp_t latitude)
{
    if (!AGC_ASSERT(ip != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(latitude >= -AGC_DP_QUARTER && latitude <= AGC_DP_QUARTER)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t v = agc_vec_make(agc_dp_sin(ip, latitude),
                               agc_dp_neg(agc_dp_cos(ip, latitude)), 0);
    ervector = agc_vec_scale(ip, v, OMEG_MS);
    tmark = agc_rtb_loadtime();                            /* LOADTIME     */
    const agc_dp_t zero[3] = { 0, 0, 0 };
    return agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, zero); /* ERCOMP1 = 0 */
}

/* EARTHR (p.430): ERCOMP1 = ERCOMP1 + XSM (ERVECTOR (t - TMARK) 2^9),
 * TMARK = t, then PULSEIMU the ERCOMP1 buffer.  OVERFFIX adds 2^28 cs
 * when the time difference is negative (TIME2 wrapped).                  */
agc_status_t agc_csm_prelaunch_earthr(agc_interp_t *ip, const agc_mat_t *xsm)
{
    if (!AGC_ASSERT(ip != NULL && xsm != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t temptime = agc_rtb_loadtime();
    agc_dp_t dt = agc_dp_sub(ip, temptime, tmark);
    if (dt < 0) {
        dt = dt + TIME_WRAP;                               /* OVERFFIX     */
    }
    (void)AGC_ASSERT(dt >= 0 && dt <= AGC_DP_MAX);
    agc_vec_t inc = agc_vec_scale(ip, ervector, agc_dp_shl(ip, dt, 9));
    agc_dp_t buf[3];
    agc_status_t st = agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, buf);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    agc_vec_t er = agc_vec_add(ip, agc_mat_vec(ip, xsm, inc),
                               agc_vec_make(buf[0], buf[1], buf[2]));
    for (int i = 0; i < 3; i++) {
        buf[i] = er.c[i];
    }
    (void)agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, buf);
    tmark = temptime;
    return agc_imu_pulse_buf(AGC_IMU_GBUF_ERCOMP, AGC_JOB_NONE); /* PULSEIMU */
}

/* ---- stubs -------------------------------------------------------------- */

/* Shared body of the untranslated entry points.                          */
static agc_status_t prelaunch_stub(const agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM);
    return AGC_STATUS_UNIMPLEMENTED;
}

/* IMUTEST / GUESS / GEOIMUTT / IMUBACK (pp.423-428): IMU performance test
 * positioning and the test sequencer.                                    */
agc_status_t agc_csm_prelaunch_imutest(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* ESTIMS / RSTGTS1 / ALLOOP / ALFLT / NORMLOP / ALKCG ... (pp.434-442):
 * gyrocompass and drift-test filter loop.                                */
agc_status_t agc_csm_prelaunch_estims(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* ALWAYSG / COMPGS / ADDINDRF (pp.444-445): compass and erection job.    */
agc_status_t agc_csm_prelaunch_alwaysg(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* RESTAIER / RESTEST1 / RESTEST3 / ADERCOMP / SPITGYRO (pp.445-446).     */
agc_status_t agc_csm_prelaunch_restaier(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* CHKCOMED / GOBKCALB / PRELTERM (pp.446-447).                           */
agc_status_t agc_csm_prelaunch_chkcomed(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* TAR/EREF / TAR1 / CONTIN33 / NEXTBNKS (pp.449-451): target vectors.    */
agc_status_t agc_csm_prelaunch_tar_eref(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* GCOMPVER (p.448): optical verification routine.                        */
agc_status_t agc_csm_prelaunch_gcompver(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* OPTDATA (pp.448-449): target azimuth and elevation (V06N41).           */
agc_status_t agc_csm_prelaunch_optdata(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* LATAZCHK (p.452): azimuth and latitude load and check.                 */
agc_status_t agc_csm_prelaunch_latazchk(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* AZMTHCG1 (p.454): azimuth change (V78).                                */
agc_status_t agc_csm_prelaunch_azmthcg1(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}

/* TARGDRVE / RETARG / RETARG1 (pp.452-453): drive the optics to target. */
agc_status_t agc_csm_prelaunch_targdrve(agc_job_ctx_t *ctx)
{
    return prelaunch_stub(ctx);
}
