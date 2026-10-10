/*
 * gyro.c -- gyro torquing: IMUPULSE and its waitlist tasks.
 *
 * Source: IMU_MODE_SWITCHING_ROUTINES.agc, Comanche055 pp.1430-1437 and
 *         Luminary099 pp.1319-1326 (identical text).
 *
 * IMUPULSE torques the three IRIGs from double-precision commands in
 * revolutions (scale 2^0) held in a six-word buffer (X, Y, Z).  One gyro
 * pulse is 2^-21 revolution (0.618 arc-sec), i.e. 128 LSB of an agc_dp_t,
 * so the minor word of the AGC DP pair holds 1/128-pulse fractions.  The
 * gyros are torqued one at a time in the order Y, Z, X at 3200 pulses/s
 * (32 pulses per centisecond).  Commands of less than one pulse are not
 * sent; the unsent fraction of every command stays in the buffer.
 *
 * Long commands (8192 pulses or more) are sent as an initial train plus
 * N augments of 8192 pulses added to GYROCMD shortly before it runs out.
 * DEVIATION: the AGC keeps the augment count in the major word of the
 * command buffer while torquing; here it is a module variable and the
 * buffer holds only the final fraction throughout.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "internal.h"
#include "pending.h"

#define GYROFRAC    ((agc_dp_t) 28) /* 2DEC .215 B-21: .215 pulse, LSBs  */
#define GYROMIN_LSB 127             /* -GYROMIN OCT -176: |minor| > 127   */
#define PULSE_SHIFT 7               /* 128 DP LSB per pulse               */
#define AUG_PULSES  8192u           /* BIT14: one 8192-pulse augment      */
#define GDESELCT    01700u          /* select, sign and activity bits     */
#define GSEL_X      00100u          /* GSELECT words: OCT 00100 (X)       */
#define GSEL_Y      00200u          /*                OCT 00202 (Y)       */
#define GSEL_Z      00300u          /*                OCT 00302 (Z)       */
#define DT_PULSE    4u              /* CAF FOUR                           */
#define DT_GWAKE    1u              /* CAF ONE (after GWAKE)              */
#define GYROCMD_MAX 077777u
#define PHASE_DONE  3

/* LGYRO: zero when the gyros are free; else the buffer in use and the
 * phase counter (bits 13-14: 0 = Y, 1 = Z, 2 = X, 3 = done).             */
typedef struct lgyro_word
{
    bool               busy;
    agc_imu_gyro_buf_t buf;
    int                phase;
} lgyro_word_t;

static agc_dp_t     gbuf[AGC_IMU_GBUF_COUNT][3]; /* OGC.., GCOMP, ERCOMP */
static lgyro_word_t lgyro;
static uint16_t     gyrocmd;       /* GYROCMD hardware counter, pulses    */
static int          aug_left;      /* 8192 augments still to come         */
static agc_job_id_t gyro_waiter;   /* job sleeping in GYROBUSY            */
static bool         gwake_pending; /* GWAKE: waking job restarts torquing */

static bool buf_ok(agc_imu_gyro_buf_t b)
{
    bool ok = (int) b >= 0 && b < AGC_IMU_GBUF_COUNT;

    _Static_assert(AGC_IMU_GBUF_COUNT == 3, "AGC constant");

    if (!AGC_ASSERT(ok || !lgyro.busy || lgyro.buf != b))
    {
        return false;
    }

    return ok;
}

static void chan14(uint16_t bits, bool on)
{
    if (!AGC_ASSERT(bits <= GYROCMD_MAX))
    {
        return;
    }

    if (on)
    {
        agc_chan_or(AGC_CHAN_CHAN14, bits);
    }
    else
    {
        agc_chan_and(AGC_CHAN_CHAN14,
                     (uint16_t) (~(unsigned) bits & GYROCMD_MAX));
    }

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN14) & bits) ==
                    (on ? bits : 0u)))
    {
        return;
    }
}

static agc_status_t waitlist(uint16_t dt, agc_task_id_t task)
{
    if (!AGC_ASSERT(dt > 0u && dt <= AGC_WAITLIST_MAX_DELAY))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = agc_waitlist_call(dt, task);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* ---- buffers and counters ---------------------------------------------- */

agc_status_t agc_imu_gyro_buf_get(agc_imu_gyro_buf_t buf, agc_dp_t out[3])
{
    if (!AGC_ASSERT(buf_ok(buf) && out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    for (int i = 0; i < 3; i++)
    {
        out[i] = gbuf[buf][i];
    }

    if (!AGC_ASSERT(out[0] >= AGC_DP_MIN && out[0] <= AGC_DP_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

void agc_imu_gyro_buf_set(agc_imu_gyro_buf_t buf, const agc_dp_t in[3])
{
    if (!AGC_ASSERT(buf_ok(buf) && in != NULL))
    {
        return;
    }

    for (int i = 0; i < 3; i++)
    {
        if (!AGC_ASSERT(in[i] >= AGC_DP_MIN && in[i] <= AGC_DP_MAX))
        {
            return;
        }

        gbuf[buf][i] = in[i];
    }
}

bool agc_imu_gyros_busy(void)
{
    if (!AGC_ASSERT(lgyro.phase >= 0 && lgyro.phase <= PHASE_DONE))
    {
        return false;
    }

    if (!AGC_ASSERT(lgyro.busy || lgyro.phase == 0))
    {
        return false;
    }

    return lgyro.busy;
}

uint16_t agc_imu_gyrocmd(void)
{
    if (!AGC_ASSERT(gyrocmd <= GYROCMD_MAX))
    {
        return 0; /* no valid result */
    }

    if (!AGC_ASSERT(aug_left >= 0))
    {
        return 0; /* no valid result */
    }

    return gyrocmd;
}

void agc_imu_hw_set_gyrocmd(uint16_t pulses)
{
    if (!AGC_ASSERT(pulses <= GYROCMD_MAX))
    {
        return;
    }

    gyrocmd = pulses;

    if (!AGC_ASSERT(gyrocmd == pulses))
    {
        return;
    }
}

/* ---- IMUPULSE ------------------------------------------------------------ */

/* IMUPULSE / GYROBUSY / GWAKE (pp.1430-1431).  Reserves the gyros for
 * buffer buf and starts STRTGYRO in 40 ms (10 ms after a GWAKE).  If the
 * gyros are busy the calling job sleeps (GYROBUSY) as wake_as; that job
 * calls this again when woken.  ALARM when the IMU is being caged.       */
agc_status_t agc_imu_pulse_buf(agc_imu_gyro_buf_t buf, agc_job_id_t wake_as)
{
    if (!AGC_ASSERT(buf_ok(buf)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (imu_cagetstj() != AGC_STATUS_OK)
    { /* IMUCADR = -0     */
        return AGC_STATUS_ALARM;
    }

    if (lgyro.busy)
    { /* GYROBUSY / REGSLEEP */
        if (!AGC_ASSERT(wake_as != AGC_JOB_NONE))
        {
            return AGC_STATUS_BUSY;
        }

        gyro_waiter = wake_as;
        agc_exec_job_sleep(wake_as);

        return AGC_STATUS_BUSY;
    }

    uint16_t dt = DT_PULSE;

    if (gwake_pending)
    {
        dt            = DT_GWAKE; /* power supply left on by the waker   */
        gwake_pending = false;
    }
    else
    {
        chan14(AGC_CH014_GYROPWR, true); /* enable power     */
    }

    agc_status_t st = waitlist(dt, AGC_TASK_STRTGYRO); /* GWAKE2           */

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    lgyro.busy  = true;                     /* TS LGYRO         */
    lgyro.buf   = buf;
    lgyro.phase = 0;
    /* GYROAGRE: force sign agreement on the inputs; an agc_dp_t always
     * agrees in sign, so there is nothing to do.                         */
    imu_modeexit();

    return AGC_STATUS_OK;
}

agc_status_t agc_imu_pulse(const agc_dp_t gyro[3])
{
    if (!AGC_ASSERT(gyro != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_OGC, gyro);

    return agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_NONE);
}

/* ---- torquing tasks ------------------------------------------------------ */

/* LASTSEG / GYROEXIT (p.1435): last train; STRTGYRO when it is out.      */
static agc_status_t lastseg(void)
{
    if (!AGC_ASSERT(gyrocmd <= GYROCMD_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    uint16_t dt = (uint16_t) ((gyrocmd >> 5) + 3u); /* MP BIT10, AD THREE */

    if (!AGC_ASSERT(dt >= 3u && dt <= 03777u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = waitlist(dt, AGC_TASK_STRTGYRO);

    chan14(AGC_CH014_GYROACT, true); /* GYROEXIT         */

    return st;
}

/* AUG3 / GYROEXIT (p.1435): wake 8192AUG just before the train is out.  */
static agc_status_t aug3(void)
{
    if (!AGC_ASSERT(gyrocmd >= AUG_PULSES && gyrocmd <= GYROCMD_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    uint16_t dt = (uint16_t) ((gyrocmd >> 5) - 3u); /* AD NEG3          */

    if (!AGC_ASSERT(dt >= 253u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = waitlist(dt, AGC_TASK_8192AUG);

    chan14(AGC_CH014_GYROACT, true);

    return st;
}

/* MAJ+/MAJ-/GMERGE/LONGGYRO (pp.1434-1437): send one gyro command.  The
 * command is rounded by adding .215 pulse in its own direction; the
 * truncated fraction is left in the buffer.                              */
static agc_status_t torque(int axis, uint16_t select, agc_dp_t cmd)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return AGC_STATUS_BAD_ARG;
    }

    bool     neg   = (cmd < 0);
    agc_dp_t total = neg ? cmd - GYROFRAC : cmd + GYROFRAC;

    if (!AGC_ASSERT(total >= AGC_DP_MIN && total <= AGC_DP_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    chan14((uint16_t) (neg ? (select | AGC_CH014_GYRONEG) : select), true);
    uint32_t mag    = (uint32_t) (neg ? -total : total);
    agc_dp_t frac   = (agc_dp_t) (mag & 0177u); /* LOW7 of minor    */
    uint32_t pulses = mag >> PULSE_SHIFT;
    uint32_t naug   = pulses >> 13;                  /* major / 64       */
    uint16_t itemp2 = (uint16_t) (pulses & 017777u); /* pulses mod 8192  */

    gbuf[lgyro.buf][axis] = neg ? -frac : frac;

    if (naug >= 2u)
    { /* LONGGYRO         */
        aug_left = (int) naug - 2;
        gyrocmd  = (uint16_t) (AUG_PULSES + itemp2);

        return aug3();
    }

    /* N = 1: the AD MINUS1 gives -0 and the CCS lands on CAF BIT14.     */
    gyrocmd = (uint16_t) ((naug == 1u) ? AUG_PULSES + itemp2 : itemp2);

    return lastseg();
}

/* GSELECT (p.1433) for one gyro: true if a pulse train was started.     */
static bool gselect(int axis, uint16_t select, agc_status_t *st)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3 && st != NULL))
    {
        return false;
    }

    if (!AGC_ASSERT(lgyro.busy && buf_ok(lgyro.buf)))
    {
        return false;
    }

    agc_dp_t cmd   = gbuf[lgyro.buf][axis];
    agc_sp_t major = agc_dp_high(cmd);
    agc_sp_t minor = agc_dp_low(cmd);
    bool     send  = (major != 0) || /* MAJ+ / MAJ-      */
                     (minor > GYROMIN_LSB) || (minor < -GYROMIN_LSB); /* MIN+/- */

    if (!send)
    {
        return false; /* TCF STRTGYR2     */
    }

    *st = torque(axis, select, cmd);

    return true;
}

/* STRTGYR2 (p.1432): next gyro by phase; at the end free the gyros, wake
 * a job waiting in GYROBUSY and finish through IMUFINED (power left on). */
static agc_status_t strtgyr2(void)
{
    static const int      axis_of[PHASE_DONE] = {1, 2, 0}; /* Y, Z, X */
    static const uint16_t sel_of[PHASE_DONE]  = {GSEL_Y, GSEL_Z, GSEL_X};

    for (int guard = 0; guard <= PHASE_DONE; guard++)
    {
        int p = lgyro.phase;

        if (!AGC_ASSERT(p >= 0 && p <= PHASE_DONE))
        {
            return AGC_STATUS_BAD_ARG;
        }

        if (p == PHASE_DONE)
        {
            lgyro.busy  = false; /* TS LGYRO = 0     */
            lgyro.phase = 0;

            if (gyro_waiter != AGC_JOB_NONE)
            { /* JOBWAKE LGWAKE   */
                gwake_pending = true;
                agc_exec_job_wake_if_asleep(gyro_waiter);
                gyro_waiter = AGC_JOB_NONE;
            }

            return agc_task_imufined(); /* NORESET          */
        }

        if (!AGC_ASSERT(lgyro.busy))
        {
            return AGC_STATUS_BAD_ARG;
        }

        lgyro.phase = p + 1; /* AD BIT13 ADS LGYRO */
        agc_status_t st = AGC_STATUS_OK;

        if (gselect(axis_of[p], sel_of[p], &st))
        {
            return st;
        }
    }

    return AGC_STATUS_OK;
}

/* STRTGYRO (task, p.1432): de-select the last gyro, then STRTGYR2.       */
agc_status_t agc_task_strtgyro(void)
{
    chan14(GDESELCT, false);

    if (imu_caged())
    { /* CAGETEST         */
        return agc_imu_imubad();
    }

    if (!AGC_ASSERT(lgyro.busy))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN14) & GDESELCT) == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return strtgyr2();
}

/* 8192AUG (task, p.1435-1436): add 8192 pulses to GYROCMD.  Bad end if
 * the IMU has been caged or put in coarse align meanwhile.               */
agc_status_t agc_task_8192aug(void)
{
    if (imu_caged())
    {
        return agc_imu_imubad();
    }

    if ((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) != 0u)
    {
        return agc_imu_imubad();
    }

    if (!AGC_ASSERT(aug_left >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    uint32_t sum = (uint32_t) gyrocmd + AUG_PULSES;

    if (!AGC_ASSERT(sum <= GYROCMD_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    gyrocmd = (uint16_t) (sum & GYROCMD_MAX); /* ADS GYROCMD      */

    if (aug_left > 0)
    { /* AUG2             */
        aug_left--;

        return aug3();
    }

    return lastseg(); /* LASTSEG +1       */
}
