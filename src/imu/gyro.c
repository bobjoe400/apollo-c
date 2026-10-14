/**********************************************************************
 * FILE NAME: gyro.c
 *
 * PURPOSE:
 *   Gyro torquing: IMUPULSE (GYROBUSY, GWAKE) and its waitlist tasks
 *   (STRTGYRO, STRTGYR2, GSELECT, MAJ+/MAJ-/GMERGE, LONGGYRO, LASTSEG,
 *   AUG3, 8192AUG), plus the three gyro command buffers.
 *
 * AGC SOURCE:
 *   Comanche055 IMU_MODE_SWITCHING_ROUTINES.agc pp. 1430-1437
 *   Luminary099 IMU_MODE_SWITCHING_ROUTINES.agc pp. 1319-1326 (identical)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   channel 14    uint16_t   I/O  bits           gyro select, sign,
 *                                                activity, power
 *   IMODES30      uint16_t   I    bits           cage test
 *   OGC,IGC,MGC   agc_dp_t   I/O  rev 2^0        the OGC buffer (owned
 *                                                here: agc_imu_ogc)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_waitlist_call     WAITLIST
 *   agc_exec_job_sleep    JOBSLEEP (GYROBUSY), JOBWAKE (GWAKE)
 *   imu_cagetstj          CAGETSTJ (mode.c)
 *   agc_task_imufined     IMUFINED (mode.c), agc_imu_imubad
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   IMUBAD (BADEND on the IMU) when the IMU is caged or put in coarse
 *   align while torquing; AGC_STATUS_ALARM from IMUPULSE when caged.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles.  One user at a time (LGYRO); torquing runs as tasks.
 *
 * NOTES:
 *   IMUPULSE torques the three IRIGs from double-precision commands in
 *   revolutions (scale 2^0) held in a six-word buffer (X, Y, Z).  One
 *   gyro pulse is 2^-21 revolution (0.618 arc-sec), i.e. 128 LSB of an
 *   agc_dp_t, so the minor word of the AGC DP pair holds 1/128-pulse
 *   fractions.  The gyros are torqued one at a time in the order Y, Z, X
 *   at 3200 pulses/s (32 pulses per centisecond).  Commands of less than
 *   one pulse are not sent; the unsent fraction of every command stays
 *   in the buffer.
 *   The OGC buffer is the erasables OGC, IGC, MGC that CALCGTA, the
 *   alignment programs and the noun 93 load write (agc_imu_ogc): one
 *   erasable, as in the AGC (ECADR OGC in A at IMUPULSE), owned here
 *   with GCOMP and ERCOMP.
 *   DEVIATION: the AGC keeps the augment count in the major word of the
 *   command buffer while torquing; here it is a module variable and the
 *   buffer holds only the final fraction throughout.
 *
 * ALGORITHM:
 *   Long commands (8192 pulses or more) are sent as an initial train
 *   plus N augments of 8192 pulses added to GYROCMD shortly before it
 *   runs out (8192AUG).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "internal.h"

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
#define GYROCMD_MAX 077777u         /* 15-bit GYROCMD counter          */
#define PHASE_DONE  3               /* LGYRO phase 3: all gyros done   */

/* LGYRO: zero when the gyros are free; else the buffer in use and the
 * phase counter (bits 13-14: 0 = Y, 1 = Z, 2 = X, 3 = done).             */
typedef struct lgyro_word
{
    bool               busy;  /* LGYRO nonzero: gyros reserved     */
    agc_imu_gyro_buf_t buf;   /* command buffer (LGYRO low 8 bits) */
    int                phase; /* bits 13-14: 0 Y, 1 Z, 2 X, 3 done */
} lgyro_word_t;

static agc_dp_t     gbuf[AGC_IMU_GBUF_COUNT][3]; /* OGC, GCOMP, ERCOMP: rev 2^0 */
static lgyro_word_t lgyro;                       /* LGYRO                                  */
static uint16_t     gyrocmd;                     /* GYROCMD hardware counter, pulses    */
static int          aug_left;                    /* 8192 augments still to come         */
static agc_job_id_t gyro_waiter;                 /* job sleeping in GYROBUSY            */
static bool         gwake_pending;               /* GWAKE: waking job restarts torquing */

/**********************************************************************
 * AGC: none (EXTEND, WOR / WAND CHAN14) -- set or clear channel 14 bits
 *
 * ARGUMENTS:
 *   bits  uint16_t  I    channel 14 bits
 *   on    bool      I    true: WOR, false: WAND of the complement
 **********************************************************************/
static void chan14(uint16_t bits, bool on)
{
    if (on)
    {
        agc_chan_or(AGC_CHAN_CHAN14, bits);
    }
    else
    {
        agc_chan_and(AGC_CHAN_CHAN14, (uint16_t) (~(unsigned int) bits & GYROCMD_MAX));
    }
}

/**********************************************************************
 * AGC: none (TC WAITLIST, 2CADR task) -- schedule a gyro task
 *
 * ARGUMENTS:
 *   dt    uint16_t       I    delay, cs
 *   task  agc_task_id_t  I    task to start
 *
 * RETURNS: agc_status_t -- OK, BAILOUT (1203 list full), POODOO (1204
 *          zero delay) or BAD_ARG from WAITLIST
 **********************************************************************/
static agc_status_t waitlist(uint16_t dt, agc_task_id_t task)
{
    agc_status_t st = agc_waitlist_call(dt, task); /* status of the call */

    return (st);
}

/* ---- buffers and counters ---------------------------------------------- */

/**********************************************************************
 * AGC: none (ECADR in A at IMUPULSE) -- the three words of a gyro
 *      command buffer
 *
 * ARGUMENTS:
 *   buf  agc_imu_gyro_buf_t  I    buffer id
 *
 * RETURNS: agc_dp_t * -- X, Y, Z commands, rev 2^0
 **********************************************************************/
static agc_dp_t *gyro_words(agc_imu_gyro_buf_t buf)
{
    return (gbuf[buf]); /* ECADR OGC / GCOMP / ERCOMP */
}

/**********************************************************************
 * AGC: none (accessor) -- the erasables OGC, IGC, MGC
 *
 * The OGC gyro buffer: CALCGTA, the alignment programs (P52, P54, P57),
 * the systems test and the noun 93 load (V25N93, FINEK2) write it;
 * IMUPULSE on the OGC buffer torques by it and leaves the unsent
 * fractions in it.
 *
 * RETURNS: agc_dp_t * -- OGC, IGC, MGC (X, Y, Z), rev 2^0
 **********************************************************************/
agc_dp_t *agc_imu_ogc(void)
{
    return (gbuf[AGC_IMU_GBUF_OGC]);
}

/**********************************************************************
 * AGC: none (accessor) -- a six-word gyro command buffer
 *
 * OGC/IGC/MGC (CALCGTA, P52, P53), GCOMP (compensation) or ERCOMP (CM
 * prelaunch earth rate): X, Y, Z commands, revolutions 2^0.
 *
 * ARGUMENTS:
 *   buf  agc_imu_gyro_buf_t  I    buffer id
 *   out  agc_dp_t [3]        O    the three commands, rev 2^0
 *
 * RETURNS: none
 **********************************************************************/
void agc_imu_gyro_buf_get(agc_imu_gyro_buf_t buf, agc_dp_t out[3])
{
    for (int i = 0; i < 3; i++)
    {
        out[i] = gyro_words(buf)[i];
    }
}

/**********************************************************************
 * AGC: none (accessor) -- store a gyro command buffer
 *
 * ARGUMENTS:
 *   buf  agc_imu_gyro_buf_t  I    buffer id
 *   in   const agc_dp_t [3]  I    the three commands, rev 2^0
 **********************************************************************/
void agc_imu_gyro_buf_set(agc_imu_gyro_buf_t buf, const agc_dp_t in[3])
{
    for (int i = 0; i < 3; i++)
    {
        gyro_words(buf)[i] = in[i];
    }
}

/**********************************************************************
 * AGC: none (CCS LGYRO) -- the gyros are reserved
 *
 * RETURNS: bool -- true while LGYRO is nonzero
 **********************************************************************/
bool agc_imu_gyros_busy(void)
{
    return (lgyro.busy);
}

/**********************************************************************
 * AGC: none (accessor) -- GYROCMD, pulses still to send in this train
 *
 * RETURNS: uint16_t -- GYROCMD, pulses B0
 **********************************************************************/
uint16_t agc_imu_gyrocmd(void)
{
    return (gyrocmd);
}

/**********************************************************************
 * AGC: none (sim hook) -- GYROCMD as the hardware left it
 *
 * ARGUMENTS:
 *   pulses  uint16_t  I    pulses still to send, 0..077777
 **********************************************************************/
void agc_imu_hw_set_gyrocmd(uint16_t pulses)
{
    gyrocmd = pulses;
}

/* ---- IMUPULSE ------------------------------------------------------------ */

/**********************************************************************
 * IMUPULSE / GYROBUSY / GWAKE / GWAKE2 (IMU_MODE_SWITCHING_ROUTINES.agc
 * pp.1430-1431) -- torque the gyros
 *
 * Reserves the gyros for buffer buf and starts STRTGYRO in 40 ms (10 ms
 * after a GWAKE, power supply left on by the waker).  If the gyros are
 * busy the calling job sleeps (GYROBUSY) as wake_as and calls this again
 * when woken.  GYROAGRE (sign agreement) has nothing to do: an agc_dp_t
 * always agrees in sign.
 *
 * ARGUMENTS:
 *   buf      agc_imu_gyro_buf_t  I    buffer id (the ECADR arriving in
 *                                     A)
 *   wake_as  agc_job_id_t        I    job to sleep as when the gyros
 *                                     are busy
 *
 * RETURNS: agc_status_t -- OK, BUSY (asleep), ALARM when the IMU is being
 *          caged (IMUCADR -0), or the waitlist status; AGC_STATUS_BAD_ARG
 *          on a failed precondition
 **********************************************************************/
agc_status_t agc_imu_pulse_buf(agc_imu_gyro_buf_t buf, agc_job_id_t wake_as)
{
    if (imu_cagetstj() != AGC_STATUS_OK)
    { /* IMUCADR = -0     */
        return (AGC_STATUS_ALARM);
    }

    if (lgyro.busy)
    { /* GYROBUSY / REGSLEEP; a caller that cannot sleep (AGC_JOB_NONE) only learns BUSY */
        if (wake_as != AGC_JOB_NONE)
        {
            gyro_waiter = wake_as;
            agc_exec_job_sleep(wake_as);
        }

        return (AGC_STATUS_BUSY);
    }

    uint16_t dt = DT_PULSE; /* CAF FOUR, cs                       */

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
        return (st);
    }

    lgyro.busy  = true; /* TS LGYRO         */
    lgyro.buf   = buf;
    lgyro.phase = 0;

    /* GYROAGRE: force sign agreement on the inputs; an agc_dp_t always
     * agrees in sign, so there is nothing to do.                         */
    imu_modeexit();

    return (AGC_STATUS_OK);
} /* agc_imu_pulse_buf */

/**********************************************************************
 * IMUPULSE on OGC (IMU_MODE_SWITCHING_ROUTINES.agc p.1430) -- torque by
 * OGC, IGC, MGC
 *
 * ARGUMENTS:
 *   gyro  const agc_dp_t [3]  I    OGC, IGC, MGC, rev 2^0
 *
 * RETURNS: agc_status_t -- status of IMUPULSE; AGC_STATUS_BAD_ARG on a
 *          failed precondition
 **********************************************************************/
agc_status_t agc_imu_pulse(const agc_dp_t gyro[3])
{
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_OGC, gyro);

    return (agc_imu_pulse_buf(AGC_IMU_GBUF_OGC, AGC_JOB_NONE));
}

/* ---- torquing tasks ------------------------------------------------------ */

/**********************************************************************
 * LASTSEG / GYROEXIT (IMU_MODE_SWITCHING_ROUTINES.agc p.1435) -- the
 * last train; STRTGYRO when it is out
 *
 * DT = GYROCMD MP BIT10 (3200 pulses/s) + 3 cs for truncation and phase.
 *
 * RETURNS: agc_status_t -- waitlist status
 **********************************************************************/
static agc_status_t lastseg(void)
{
    uint16_t dt = (uint16_t) ((gyrocmd >> 5) + 3u); /* MP BIT10, AD THREE */

    agc_status_t st = waitlist(dt, AGC_TASK_STRTGYRO); /* status of the call */

    chan14(AGC_CH014_GYROACT, true); /* GYROEXIT         */

    return (st);
}

/**********************************************************************
 * AUG3 / GYROEXIT (IMU_MODE_SWITCHING_ROUTINES.agc p.1435) -- 8192AUG
 * just before the train is out
 *
 * RETURNS: agc_status_t -- waitlist status
 **********************************************************************/
static agc_status_t augment_schedule(void)
{
    uint16_t dt = (uint16_t) ((gyrocmd >> 5) - 3u); /* AD NEG3          */

    agc_status_t st = waitlist(dt, AGC_TASK_GYRO_AUGMENT); /* status of the call */

    chan14(AGC_CH014_GYROACT, true);

    return (st);
}

/**********************************************************************
 * MAJ+ / MAJ- / GMERGE / LONGGYRO (IMU_MODE_SWITCHING_ROUTINES.agc
 * pp.1434-1437) -- send one gyro command
 *
 * The command is rounded by .215 pulse (GYROFRAC) in its own direction;
 * the truncated fraction stays in the buffer.  N = command / 8192 pulses:
 * N = 0 or 1 one train (N = 1: the AD MINUS1 gives -0 and the CCS lands on
 * CAF BIT14), N >= 2 an initial train of 8192 + rest and N - 2 more
 * augments (LONGGYRO).
 * DEVIATION: the AGC keeps the augment count in the major word of the
 * buffer while torquing; here it is aug_left.
 *
 * ARGUMENTS:
 *   axis    int       I    0 X, 1 Y, 2 Z
 *   select  uint16_t  I    channel 14 gyro select bits
 *   cmd     agc_dp_t  I    command, rev 2^0
 *
 * RETURNS: agc_status_t -- status of LASTSEG or AUG3
 **********************************************************************/
static agc_status_t torque(int axis, uint16_t select, agc_dp_t cmd)
{
    bool     neg   = (cmd < 0);                             /* MAJ- / MIN- path             */
    agc_dp_t total = neg ? cmd - GYROFRAC : cmd + GYROFRAC; /* DAS +-GYROFRAC, rev 2^0; toward larger magnitude */

    chan14((uint16_t) (neg ? (select | AGC_CH014_GYRONEG) : select), true); /* AD BIT9 */
    uint32_t mag    = (uint32_t) (neg ? -total : total);                    /* |cmd|, 2^-28 rev */
    agc_dp_t frac   = (agc_dp_t) (mag & 0177u);                             /* LOW7, rev 2^0   */
    uint32_t pulses = mag >> PULSE_SHIFT;                                   /* whole pulses (2^-21 rev) */
    uint32_t naug   = pulses >> 13;                                         /* major / 64       */
    uint16_t itemp2 = (uint16_t) (pulses & 017777u);                        /* pulses mod 8192  */

    gyro_words(lgyro.buf)[axis] = neg ? -frac : frac; /* COM for MAJ- */

    if (naug >= 2u)
    { /* LONGGYRO         */
        aug_left = (int) naug - 2;
        gyrocmd  = (uint16_t) (AUG_PULSES + itemp2);

        return (augment_schedule());
    }

    /* N = 1: the AD MINUS1 gives -0 and the CCS lands on CAF BIT14.     */
    gyrocmd = (uint16_t) ((naug == 1u) ? AUG_PULSES + itemp2 : itemp2); /* ADS ITEMP2 */

    return (lastseg());
}

/**********************************************************************
 * GSELECT / MIN+ / MIN- (IMU_MODE_SWITCHING_ROUTINES.agc pp.1433-1434)
 * -- one gyro, if its command is big enough
 *
 * A nonzero major word, or a minor word beyond GYROMIN (127/128 pulse).
 *
 * ARGUMENTS:
 *   axis    int             I    0 X, 1 Y, 2 Z
 *   select  uint16_t        I    channel 14 gyro select bits
 *   st      agc_status_t *  O    status of the train started
 *
 * RETURNS: bool -- true if a pulse train was started
 **********************************************************************/
static bool gselect(int axis, uint16_t select, agc_status_t *st)
{
    agc_dp_t cmd   = gyro_words(lgyro.buf)[axis];                                     /* DCA 1400, rev 2^0 */
    agc_sp_t major = agc_dp_high(cmd);                                                /* RUPTREG1, 128 pulses B0 */
    agc_sp_t minor = agc_dp_low(cmd);                                                 /* RUPTREG2, pulse/128 B0  */
    bool     send  = (major != 0) || (minor > GYROMIN_LSB) || (minor < -GYROMIN_LSB); /* MAJ+ / MAJ-; MIN+/- */

    if (!send)
    {
        return (false); /* TCF STRTGYR2     */
    }

    *st = torque(axis, select, cmd);

    return (true);
}

/**********************************************************************
 * STRTGYR2 (IMU_MODE_SWITCHING_ROUTINES.agc p.1432) -- next gyro by the
 * LGYRO phase (Y, Z, X)
 *
 * At the end: LGYRO = 0, JOBWAKE GWAKE (a job waiting in GYROBUSY) and
 * NORESET: IMUFINED with the power supply left on.
 *
 * RETURNS: agc_status_t -- status of the train started or of IMUFINED
 **********************************************************************/
static agc_status_t gyro_next(void)
{
    static const int      axis_of[PHASE_DONE] = {1, 2, 0}; /* Y, Z, X */
    static const uint16_t sel_of[PHASE_DONE]  = {GSEL_Y, GSEL_Z, GSEL_X};

    for (int guard = 0; guard <= PHASE_DONE; guard++)
    {
        int p = lgyro.phase; /* LGYRO MP BIT4: phase      */

        if (p == PHASE_DONE)
        {
            lgyro.busy  = false; /* TS LGYRO = 0     */
            lgyro.phase = 0;

            if (gyro_waiter != AGC_JOB_NONE)
            { /* JOBWAKE LGWAKE   */
                gwake_pending   = true;
                agc_status_t st = agc_exec_job_wake_if_asleep(gyro_waiter); /* status of the JOBWAKE */

                gyro_waiter = AGC_JOB_NONE;

                if (st != AGC_STATUS_OK)
                {
                    return (st); /* CCSHOLE: POODOO */
                }
            }

            return (agc_task_imufined()); /* NORESET          */
        }

        lgyro.phase     = p + 1;         /* AD BIT13 ADS LGYRO */
        agc_status_t st = AGC_STATUS_OK; /* status of the call */

        if (gselect(axis_of[p], sel_of[p], &st))
        {
            return (st);
        }
    }

    return (AGC_STATUS_OK);
} /* gyro_next */

/**********************************************************************
 * STRTGYRO (IMU_MODE_SWITCHING_ROUTINES.agc p.1432) -- task: de-select
 * the last gyro, CAGETEST, STRTGYR2
 *
 * RETURNS: agc_status_t -- status of STRTGYR2, or IMUBAD when caged;
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_task_strtgyro(void)
{
    chan14(GDESELCT, false);

    if (imu_caged())
    { /* CAGETEST         */
        return (agc_imu_imubad());
    }

    if (!AGC_ASSERT(lgyro.busy))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (gyro_next());
}

/**********************************************************************
 * 8192AUG / AUG2 (IMU_MODE_SWITCHING_ROUTINES.agc pp.1435-1436) --
 * task: add 8192 pulses to GYROCMD
 *
 * Bad end if the IMU has been caged or put in coarse align meanwhile.
 *
 * RETURNS: agc_status_t -- status of AUG3 or LASTSEG, or IMUBAD;
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_task_gyro_augment(void)
{
    if (imu_caged())
    {
        return (agc_imu_imubad());
    }

    if ((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) != 0u)
    {
        return (agc_imu_imubad());
    }

    if (!AGC_ASSERT(aug_left >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint32_t sum = (uint32_t) gyrocmd + AUG_PULSES; /* ADS GYROCMD */

    if (!AGC_ASSERT(sum <= GYROCMD_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    gyrocmd = (uint16_t) (sum & GYROCMD_MAX); /* ADS GYROCMD      */

    if (aug_left > 0)
    { /* AUG2             */
        aug_left--;

        return (augment_schedule());
    }

    return (lastseg()); /* LASTSEG +1       */
}
