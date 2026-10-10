/*
 * mode.c -- IMU mode switching: zero, coarse align, fine align, PIPA
 * use, the generalized end-of-operation and stall routines, and R02.
 *
 * Source: IMU_MODE_SWITCHING_ROUTINES.agc, Comanche055 pp.1420-1448 and
 *         Luminary099 pp.1309-1337.  Gyro torquing (IMUPULSE and its
 *         tasks) is in gyro.c.
 *
 * CM vs LM differences (the files are otherwise identical):
 *   * GOODEND: when MODECADR is already negative (a completed operation
 *     nobody has stalled on yet) Luminary resets it to +0; Comanche would
 *     JOBWAKE the negative word.  Modelled the Luminary way for both
 *     (DEVIATION, see goodend_common).
 *   * STALL with two users of one device: CM POODOO 01210, LM BAILOUT
 *     01210 (BAILOUT1).
 *   * RNDREFDR / R02BOTH: LM uses the named masks TRACKBIT, DRFTBIT,
 *     REFSMBIT; same bits.
 *
 * MODECADR model.  Each device word holds +0 (free or operation in
 * progress), -1 (complete, good), -0 (complete, failed) or the CADR of a
 * stalled job.  Here: an enum plus the job id to wake.  The "advance LOC
 * if successful" of GOODEND becomes a per-device outcome the woken job
 * reads with agc_imu_stall_outcome().
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "internal.h"
#include "pending.h"

#define ALARM_ZERO_IN_GLOCK ((agc_alarm_code_t) 00206) /* zero in coarse+lock */
#define ALARM_IMU_NOT_OPER  ((agc_alarm_code_t) 00210) /* IMU not operating  */
#define ALARM_COARSE_ERR    ((agc_alarm_code_t) 00211) /* coarse align error  */
#define ALARM_PIP_FAIL      ((agc_alarm_code_t) 00212) /* PIPA fail at PIPFREE */
#define ALARM_REFSMMAT      ((agc_alarm_code_t) 00220) /* REFSMMAT not known  */
#define ALARM_TWO_USERS     ((agc_alarm_code_t) 01210) /* two users, 1 device */

#define DT_IMUZERO2 32u   /* BIT6: 320 ms for the AGS pulse train, cs  */
#define DT_IMUZERO3 1024u /* BIT11: 10.24 s for the CDUs to find gimbals */
#define DT_COARS    6u    /* CAF SIX                                  */
#define DT_COARS2   2u    /* "minimum of 4 ms wait"                   */
#define DT_600MS    60u   /* 600MS DEC 60                             */
#define DT_SETTLE   150u  /* FIXDELAY DEC 150                         */
#define DT_IFAILOK  512u  /* BIT10: 5.12 s                            */
#define DT_2SECS    200u  /* 2SECS                                    */
#define COMMAX      191   /* -COMMAX  DEC -191 (max pulses minus one)  */
#define COMMAX_     192   /* -COMMAX- DEC -192                         */
#define COARSTOL    182   /* DEC -.01111 at half-rev = 2 deg, counts   */
#define CDU_MASK    077777u

typedef enum modecadr_state
{
    MC_FREE = 0,  /* +0                                        */
    MC_DONE_GOOD, /* -1                                        */
    MC_DONE_BAD,  /* -0                                        */
    MC_STALLED    /* CADR of a sleeping job                    */
} modecadr_state_t;

typedef struct modecadr_word
{
    modecadr_state_t state;
    agc_job_id_t     job; /* MC_STALLED: job to wake                   */
} modecadr_word_t;

/* MODECADR words live in exec (agc_exec.h; STARTSB2 clears them, SVCT3
 * tests IMUCADR).  Encoding of the SP word: +0 free, -1 good, a positive
 * job id for a stalled job, and -2 standing in for -0 (failed).
 * DEVIATION: agc_sp_t has no -0.                                         */
#define MC_WORD_GOOD ((agc_sp_t) -1)
#define MC_WORD_BAD  ((agc_sp_t) -2)

static modecadr_word_t mc_get(int dev)
{
    if (!AGC_ASSERT(dev >= 0 && dev < AGC_MODECADR_N))
    {
        return (modecadr_word_t){0}; /* no valid result */
    }
    agc_sp_t        v = agc_erase_modecadr(dev);
    modecadr_word_t w = {MC_FREE, AGC_JOB_NONE};
    if (v == MC_WORD_GOOD)
    {
        w.state = MC_DONE_GOOD;
    }
    else if (v == MC_WORD_BAD)
    {
        w.state = MC_DONE_BAD;
    }
    else if (v > 0)
    {
        w.state = MC_STALLED;
        w.job   = (agc_job_id_t) v;
    }
    else
    {
        if (!AGC_ASSERT(v == 0))
        {
            return (modecadr_word_t){0}; /* no valid result */
        }
    }
    return w;
}

static void mc_set(int dev, modecadr_state_t s, agc_job_id_t job)
{
    if (!AGC_ASSERT((int) job >= 0 && (int) job <= AGC_SP_MAX))
    {
        return;
    }
    agc_sp_t     v  = (s == MC_DONE_GOOD) ? MC_WORD_GOOD :
                      (s == MC_DONE_BAD)  ? MC_WORD_BAD :
                      (s == MC_STALLED)   ? (agc_sp_t) job :
                                            0;
    agc_status_t st = agc_erase_set_modecadr(dev, v);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }
}

static agc_imu_stall_result_t outcome[AGC_IMU_DEV_COUNT];
static agc_sp_t command[3]; /* COMMAND: coarse-align pulses to go, 1:S   */
static agc_sp_t cducmd[3];  /* CDUXCMD..CDUZCMD hardware counters        */

/* ---- small helpers ------------------------------------------------------ */

static uint16_t notmask(uint16_t m)
{
    if (!AGC_ASSERT(m <= CDU_MASK))
    {
        return 0; /* no valid result */
    }
    uint16_t r = (uint16_t) (~(unsigned) m & CDU_MASK);
    if (!AGC_ASSERT((r & m) == 0u))
    {
        return 0; /* no valid result */
    }
    return r;
}

static void imodes30_or(uint16_t bits)
{
    if (!AGC_ASSERT(bits <= CDU_MASK))
    {
        return;
    }
    agc_erase_set_imodes30((uint16_t) (agc_erase_imodes30() | bits));
    if (!AGC_ASSERT((agc_erase_imodes30() & bits) == bits))
    {
        return;
    }
}

static void imodes30_clear(uint16_t bits)
{
    if (!AGC_ASSERT(bits <= CDU_MASK))
    {
        return;
    }
    agc_erase_set_imodes30((uint16_t) (agc_erase_imodes30() & notmask(bits)));
    if (!AGC_ASSERT((agc_erase_imodes30() & bits) == 0u))
    {
        return;
    }
}

static void imodes33_set(uint16_t bits, bool on)
{
    if (!AGC_ASSERT(bits <= CDU_MASK))
    {
        return;
    }
    uint16_t v = agc_erase_imodes33();
    v          = on ? (uint16_t) (v | bits) : (uint16_t) (v & notmask(bits));
    agc_erase_set_imodes33(v);
    if (!AGC_ASSERT(((agc_erase_imodes33() & bits) != 0u) == on))
    {
        return;
    }
}

static void dsp11_write(uint16_t v)
{
    if (!AGC_ASSERT(v <= CDU_MASK))
    {
        return;
    }
    agc_erase_set_dsptab(11, v);
}

static void chan_or(int chan, uint16_t bits)
{
    if (!AGC_ASSERT(chan == AGC_CHAN_CHAN12 || chan == AGC_CHAN_CHAN14))
    {
        return;
    }
    agc_chan_or(chan, bits);
}

static void chan_clear(int chan, uint16_t bits)
{
    if (!AGC_ASSERT(chan == AGC_CHAN_CHAN12 || chan == AGC_CHAN_CHAN14))
    {
        return;
    }
    agc_chan_and(chan, notmask(bits));
}

static void setissw(void)
{
    agc_status_t st = agc_t4_setissw(); /* IBNKCALL SETISSW      */
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }
    if (!AGC_ASSERT(agc_erase_imodes30() <= CDU_MASK))
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

/* Two's-complement 15-bit CDU difference a - b as a signed count
 * (MSU: result in ones' complement, range -16383..+16383).              */
static int cdu_msu(uint16_t a, uint16_t b)
{
    if (!AGC_ASSERT(a <= CDU_MASK && b <= CDU_MASK))
    {
        return 0; /* no valid result */
    }
    int d = (int) ((unsigned) (a - b) & CDU_MASK);
    if (d >= 040000)
    {
        d -= 0100000;
    }
    if (!AGC_ASSERT(d >= -040000 && d < 040000))
    {
        return 0; /* no valid result */
    }
    return d;
}

static uint16_t cdu_read(int axis)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return 0; /* no valid result */
    }
    uint16_t v = (axis == 0) ? agc_erase_cdux() :
                 (axis == 1) ? agc_erase_cduy() :
                               agc_erase_cduz();
    if (!AGC_ASSERT(v <= CDU_MASK))
    {
        return 0; /* no valid result */
    }
    return v;
}

/* ---- shared helpers (internal.h) ----------------------------------- */

bool imu_caged(void)
{
    uint16_t m = agc_erase_imodes30();
    if (!AGC_ASSERT(m <= CDU_MASK))
    {
        return false;
    }
    return (m & IM30_CAGE) != 0u;
}

void imu_modeexit(void)
{
    /* IMUPULSE reaches MODEEXIT without an INHINT; RELINT is harmless
     * there, but the C inhibit is counted, so only release a held one.  */
    if (agc_interrupts_inhibited())
    {
        agc_relint(); /* MODEEXIT: RELINT */
    }
    if (!AGC_ASSERT(agc_erase_modecadr(AGC_IMU_DEV_IMU) >= MC_WORD_BAD))
    {
        return;
    }
    return; /* TCF SWRETURN     */
}

/* CAGETSTJ (p.1438): during mode-switch initialization, if the IMU is
 * being caged set IMUCADR to -0 (complete but failed) and return.        */
agc_status_t imu_cagetstj(void)
{
    if (!imu_caged())
    {
        return AGC_STATUS_OK; /* TC Q             */
    }
    mc_set(AGC_IMU_DEV_IMU, MC_DONE_BAD,
           AGC_JOB_NONE); /* CS ZERO; TS IMUCADR */
    if (!AGC_ASSERT(mc_get(AGC_IMU_DEV_IMU).state == MC_DONE_BAD))
    {
        return AGC_STATUS_BAD_ARG;
    }
    imu_modeexit();
    return AGC_STATUS_BUSY;
}

/* CAGETEST (p.1438): in a task, terminate the mode switch (IMUBAD) if the
 * IMU has been caged.  Returns true when the task must end.              */
static bool cagetest(void)
{
    if (!imu_caged())
    {
        return false;
    }
    agc_status_t st = agc_imu_imubad();
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return false;
    }
    return true;
}

agc_status_t agc_imu_mode_init(void)
{
    for (int d = 0; d < AGC_IMU_DEV_COUNT; d++)
    {
        mc_set(d, MC_FREE, AGC_JOB_NONE);
        outcome[d] = AGC_IMU_STALL_SLEEPING;
    }
    for (int i = 0; i < 3; i++)
    {
        command[i] = 0;
        cducmd[i]  = 0;
    }
    if (!AGC_ASSERT(mc_get(0).state == MC_FREE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* ZEROICDU (p.1420): zero the ICDU counters.                             */
void agc_imu_zeroicdu(void)
{
    agc_erase_set_cdux(0);
    agc_erase_set_cduy(0);
    agc_erase_set_cduz(0);
    if (!AGC_ASSERT(agc_erase_cdux() == 0u && agc_erase_cduy() == 0u))
    {
        return;
    }
    if (!AGC_ASSERT(agc_erase_cduz() == 0u))
    {
        return;
    }
}

/* NOATTOFF (p.1428): turn off the NO ATT lamp (DSPTAB +11D bit 4), mark
 * the word changed (bit 15).                                             */
void agc_imu_noattoff(void)
{
    uint16_t v = agc_erase_dsptab(11);
    if (!AGC_ASSERT(v <= CDU_MASK))
    {
        return;
    }
    v = (uint16_t) ((v & notmask(DSP11_NOATT | DSP11_CHANGED)) | DSP11_CHANGED);
    dsp11_write(v);
}

agc_status_t agc_imu_clear_cmds(void)
{
    for (int i = 0; i < 3; i++)
    {
        cducmd[i] = 0; /* -0: no pulses outstanding    */
    }
    agc_imu_hw_set_gyrocmd(0);
    if (!AGC_ASSERT(cducmd[0] == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

agc_sp_t agc_imu_cducmd(int axis)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return 0;
    }
    if (!AGC_ASSERT(cducmd[axis] >= AGC_SP_MIN))
    {
        return 0;
    }
    return cducmd[axis];
}

agc_status_t agc_imu_hw_set_cducmd(int axis, agc_sp_t pulses)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(pulses >= -COMMAX_ && pulses <= COMMAX_))
    {
        return AGC_STATUS_BAD_ARG;
    }
    cducmd[axis] = pulses;
    return AGC_STATUS_OK;
}

/* ======================================================================= */
/* IMU zeroing (p.1421-1422)                                               */
/* ======================================================================= */

/* IMUZERO: zero the ICDUs.  Refused (alarm 00206, immediate failure) when
 * the IMU is both in coarse align and gimbal lock (runaway protection).  */
agc_status_t agc_imu_zero(void)
{
    agc_inhint();
    uint16_t lamps = agc_erase_dsptab(11);
    if (!AGC_ASSERT(lamps <= CDU_MASK))
    {
        agc_relint();
        return AGC_STATUS_BAD_ARG;
    }
    if ((notmask(lamps) & (DSP11_NOATT | DSP11_GLOCK)) == 0u)
    {
        agc_alarm(ALARM_ZERO_IN_GLOCK);
        mc_set(AGC_IMU_DEV_IMU, MC_DONE_BAD, AGC_JOB_NONE); /* CAGETSTJ +4 */
        imu_modeexit();
        return AGC_STATUS_OK;
    }
    if (imu_cagetstj() != AGC_STATUS_OK)
    { /* IMUZEROA       */
        return AGC_STATUS_OK;
    }
    imodes33_set(IM33_DAPDIS, true); /* disable DAP auto, hold */
    imodes30_or(IM30_ICDUFAILINH | IM30_IMUFAILINH);
    chan_clear(AGC_CHAN_CHAN12, AGC_CH012_COARSE | AGC_CH012_ENBIMUERR);
    agc_imu_noattoff();
    chan_or(AGC_CHAN_CHAN12, AGC_CH012_ZEROICDU);
    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) == 0u))
    {
        agc_relint();
        return AGC_STATUS_BAD_ARG;
    }
    agc_imu_zeroicdu();
    agc_status_t st = waitlist(DT_IMUZERO2, AGC_TASK_IMUZERO2);
    if (st != AGC_STATUS_OK)
    {
        agc_relint();
        return st;
    }
    if ((agc_erase_imodes30() & IM30_NOTOPER) != 0u)
    {
        agc_alarm(ALARM_IMU_NOT_OPER); /* IMU not operating */
    }
    imu_modeexit();
    return AGC_STATUS_OK;
}

/* IMUZERO2 (task): remove the zero discrete, wait 10 s for the counters. */
agc_status_t agc_task_imuzero2(void)
{
    if (cagetest())
    {
        return AGC_STATUS_OK;
    }
    agc_imu_zeroicdu();
    chan_clear(AGC_CHAN_CHAN12, AGC_CH012_ZEROICDU);
    agc_status_t st = agc_waitlist_vardelay(DT_IMUZERO3, AGC_TASK_IMUZERO3);
    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* IMUZERO3 (task): remove the fail inhibits, re-enable DAP, ENDIMU.     */
agc_status_t agc_task_imuzero3(void)
{
    if (cagetest())
    {
        return AGC_STATUS_OK;
    }
    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_ZEROICDU) == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    imodes30_clear(IM30_ICDUFAILINH | IM30_IMUFAILINH);
    imodes33_set(IM33_DAPDIS, false);
    setissw();
    if (!AGC_ASSERT(!imu_caged()))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_imu_endimu();
}

/* ======================================================================= */
/* Coarse align (p.1423-1426)                                              */
/* ======================================================================= */

/* IMUCOARS: coarse align to THETAD (CDU patterns X, Y, Z).               */
agc_status_t agc_imu_coarse(const uint16_t thetad[3])
{
    if (!AGC_ASSERT(thetad != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    for (int i = 0; i < 3; i++)
    {
        if (!AGC_ASSERT(thetad[i] <= CDU_MASK))
        {
            return AGC_STATUS_BAD_ARG;
        }
        agc_erase_set_thetad(i, thetad[i]);
    }
    agc_inhint();
    if (imu_cagetstj() != AGC_STATUS_OK)
    {
        return AGC_STATUS_OK;
    }
    agc_imu_setcoars();
    agc_status_t st = waitlist(DT_COARS, AGC_TASK_COARS);
    if (st != AGC_STATUS_OK)
    {
        agc_relint();
        return st;
    }
    imu_modeexit();
    return AGC_STATUS_OK;
}

/* COARS1 body: THETAD - THETAA, shifted right 2 and rounded (pulses).   */
static agc_sp_t coarse_command(int axis)
{
    int diff = cdu_msu(agc_erase_thetad(axis), cdu_read(axis));
    int mag  = (diff < 0) ? -diff : diff;
    int cmd  = (mag >> 2) + (((mag & 3) >= 2) ? 1 : 0); /* MP BIT13, round */
    cmd      = (diff < 0) ? -cmd : cmd;
    if (!AGC_ASSERT(cmd >= -4096 && cmd <= 4096))
    {
        return 0; /* no valid result */
    }
    return (agc_sp_t) cmd;
}

/* COARS (task): enable the ISS CDU error counters and compute COMMAND.   */
agc_status_t agc_task_coars(void)
{
    if (cagetest())
    {
        return AGC_STATUS_OK;
    }
    chan_or(AGC_CHAN_CHAN12, AGC_CH012_ENBIMUERR);
    for (int cduind = 2; cduind >= 0; cduind--)
    { /* CDUIND 2, 1, 0  */
        command[cduind] = coarse_command(cduind);
    }
    agc_status_t st = agc_waitlist_vardelay(DT_COARS2, AGC_TASK_COARS2);
    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* COMPOS / COMNEG / COMZERO for one CDU: the pulses to send this cycle
 * (at most 192) and the remainder left in COMMAND.                       */
static agc_sp_t coarse_slice(int axis)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return 0; /* no valid result */
    }
    int c   = command[axis];
    int mag = (c < 0) ? -c : c;
    if (mag - 1 - COMMAX <= 0)
    { /* COMZERO          */
        command[axis] = 0;
        return (agc_sp_t) c;
    }
    command[axis] = (agc_sp_t) ((c < 0) ? c + COMMAX_ : c - COMMAX_);
    if (!AGC_ASSERT(command[axis] != 0))
    {
        return 0; /* no valid result */
    }
    return (agc_sp_t) ((c < 0) ? -COMMAX_ : COMMAX_);
}

/* COARS2 (task): send up to 192 pulses per CDU every 600 ms until the
 * commands are exhausted, then let the gimbals settle 1.5 s.             */
agc_status_t agc_task_coars2(void)
{
    if (cagetest())
    {
        return AGC_STATUS_OK;
    }
    int itemp1 = 0; /* pulses to go?    */
    for (int cduind = 2; cduind >= 0; cduind--)
    {
        agc_sp_t slice = (command[cduind] == 0) ? 0 : coarse_slice(cduind);
        itemp1 += (slice != 0) ? 1 : 0; /* NEXTCDU INCR     */
        cducmd[cduind] = slice;         /* TS CDUXCMD       */
    }
    agc_status_t st;
    if (itemp1 > 0)
    { /* SENDPULS         */
        chan_or(AGC_CHAN_CHAN14, AGC_CH014_DRIVECDUX | AGC_CH014_DRIVECDUY |
                                     AGC_CH014_DRIVECDUZ);
        st = agc_waitlist_vardelay(DT_600MS, AGC_TASK_COARS2);
    }
    else
    {
        st = agc_waitlist_fixdelay(DT_SETTLE, AGC_TASK_CHKCORS);
    }
    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* CHKCORS (task): check each gimbal is within 2 degrees of THETAD.       */
agc_status_t agc_task_chkcors(void)
{
    if (!AGC_ASSERT(command[0] == 0 && command[1] == 0 && command[2] == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(cducmd[0] >= -COMMAX_ && cducmd[0] <= COMMAX_))
    {
        return AGC_STATUS_BAD_ARG;
    }
    for (int i = 2; i >= 0; i--)
    {
        int diff = cdu_msu(cdu_read(i), agc_erase_thetad(i));
        int mag  = (diff < 0) ? -diff : diff;
        if (mag != 0 && (mag - 1) - COARSTOL > 0)
        { /* COARSERR         */
            agc_alarm(ALARM_COARSE_ERR);
            return agc_imu_imubad();
        }
    }
    return agc_imu_endimu(); /* end of coarse    */
}

/* CA+ECE (task, p.1425): enable the three ISS CDU error counters.        */
agc_status_t agc_task_ca_ece(void)
{
    chan_or(AGC_CHAN_CHAN12, AGC_CH012_ENBIMUERR);
    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_ENBIMUERR) != 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK; /* TASKOVER         */
}

/* SETCOARS (p.1426): put the ISS in coarse align (bypassed if already). */
void agc_imu_setcoars(void)
{
    if ((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) != 0u)
    {
        return;
    }
    chan_clear(AGC_CHAN_CHAN12, AGC_CH012_ENBIMUERR); /* clear error ctrs */
    chan_clear(AGC_CHAN_CHAN14, AGC_CH014_GYROACT);   /* gyro activity   */
    agc_imu_hw_set_gyrocmd(0);                 /* CS ZERO: -0     */
    chan_or(AGC_CHAN_CHAN12, AGC_CH012_COARSE);
    dsp11_write(
        (uint16_t) (agc_erase_dsptab(11) | DSP11_NOATT | DSP11_CHANGED));
    imodes33_set(IM33_DAPDIS6, true);
    imodes30_or(IM30_IMUFAILINH);
    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) != 0u))
    {
        return;
    }
    if (!AGC_ASSERT((agc_erase_dsptab(11) & DSP11_NOATT) != 0u))
    {
        return;
    }
    agc_imu_rndrefdr();
    return;
}

/* RNDREFDR (p.1426): knock down TRACKFLG, DRIFTFLG and REFSMFLG.         */
void agc_imu_rndrefdr(void)
{
    agc_flag_clear(AGC_FLAG_TRACKFLG);
    agc_flag_clear(AGC_FLAG_DRIFTFLG);
    agc_flag_clear(AGC_FLAG_REFSMFLG);
}

/* ======================================================================= */
/* Fine align (p.1427-1428)                                                */
/* ======================================================================= */

/* IMUFINE / IMUFIN20: reset zero and coarse, IMU fail stays inhibited
 * for the first 5 s of fine align; ENDIMU after 2 s.                     */
agc_status_t agc_imu_fine(void)
{
    agc_inhint();
    if (imu_cagetstj() != AGC_STATUS_OK)
    {
        return AGC_STATUS_OK;
    }
    chan_clear(AGC_CHAN_CHAN12, AGC_CH012_COARSE | AGC_CH012_ZEROICDU);
    imodes33_set(IM33_DAPDIS6, false);
    agc_imu_noattoff();
    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) == 0u))
    {
        agc_relint();
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT((agc_erase_dsptab(11) & DSP11_NOATT) == 0u))
    {
        agc_relint();
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t sw = waitlist(DT_IFAILOK, AGC_TASK_IFAILOK);
    if (sw != AGC_STATUS_OK)
    {
        agc_relint();
        return sw;
    }
    agc_status_t st = waitlist(DT_2SECS, AGC_TASK_IMUFINED);
    if (st != AGC_STATUS_OK)
    {
        agc_relint();
        return st;
    }
    imu_modeexit();
    return AGC_STATUS_OK;
}

agc_status_t agc_imu_fin20(void)
{
    _Static_assert(AGC_TASK_IMUFINED != AGC_TASK_NONE, "AGC constant");
    agc_status_t st = agc_imu_fine(); /* IMUFIN20 = IMUFINE */
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* IMUFINED (task): see that no one has caged the IMU, then ENDIMU.       */
agc_status_t agc_task_imufined(void)
{
    if (cagetest())
    {
        return AGC_STATUS_OK;
    }
    if (!AGC_ASSERT(!imu_caged()))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(!agc_imu_gyros_busy()))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_imu_endimu();
}

/* PFAILOK2 tail shared by IFAILOK and PFAILOK.                           */
static agc_status_t pfailok2(uint16_t clear_bits)
{
    if (!AGC_ASSERT(clear_bits == IM30_IMUFAILINH || clear_bits == IM30_PIPFAILINH))
    {
        return AGC_STATUS_BAD_ARG;
    }
    imodes30_clear(clear_bits);
    if (!AGC_ASSERT(!imu_caged()))
    {
        return AGC_STATUS_BAD_ARG;
    }
    setissw();            /* the ISS warning may come on now               */
    return AGC_STATUS_OK; /* TASKOVER         */
}

/* IFAILOK (task): enable IMU fail unless caged or back in coarse align.  */
agc_status_t agc_task_ifailok(void)
{
    if (imu_caged())
    { /* CAGETSTQ         */
        return AGC_STATUS_OK;
    }
    if ((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) != 0u)
    {
        return AGC_STATUS_OK;
    }
    if (!AGC_ASSERT(!imu_caged()))
    {
        return AGC_STATUS_BAD_ARG;
    }
    imodes30_or(IM30_IMUOK); /* reset IMUFAIL    */
    return pfailok2(IM30_IMUFAILINH);
}

/* PFAILOK (task): enable the PIPA fail program alarm.                    */
agc_status_t agc_task_pfailok(void)
{
    if (imu_caged())
    {
        return AGC_STATUS_OK;
    }
    if (!AGC_ASSERT(!imu_caged()))
    {
        return AGC_STATUS_BAD_ARG;
    }
    imodes30_or(IM30_PIPOK); /* reset fail bits  */
    imodes33_set(IM33_PIPOK, true);
    return pfailok2(IM30_PIPFAILINH);
}

/* ======================================================================= */
/* PIPA use (p.1429)                                                       */
/* ======================================================================= */

/* PIPUSE: zero the PIPA counters, then PIPUSE1.                          */
agc_status_t agc_imu_pipuse(void)
{
    agc_erase_set_pipax(0); /* CS ZERO: -0      */
    agc_erase_set_pipay(0);
    agc_erase_set_pipaz(0);
    if (!AGC_ASSERT(agc_erase_pipax() == 0 && agc_erase_pipaz() == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_imu_pipuse1();
    return AGC_STATUS_OK;
}

/* PIPUSE1: from now until PIPFREE, a PIPA fail lights ISS warning.       */
void agc_imu_pipuse1(void)
{
    if (imu_caged())
    { /* do not enable PIPA fail     */
        return;
    }
    agc_inhint();
    imodes30_clear(IM30_PIPNOTUSED);
    if (!AGC_ASSERT(!imu_caged()))
    {
        agc_relint();
        return;
    }
    setissw(); /* PIPFREE2         */
    imu_modeexit();
    return;
}

/* PIPFREE: program done with the PIPAs.  If PIPA fail is on, alarm 00212
 * and reset the ISS warning.                                             */
void agc_imu_pipfree(void)
{
    agc_inhint();
    imodes30_or(IM30_PIPNOTUSED);
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        agc_relint();
        return;
    }
    if ((agc_erase_imodes30() & IM30_PIPOK) != 0u)
    {
        imu_modeexit();
        return;
    }
    agc_alarm(ALARM_PIP_FAIL);
    agc_inhint();
    setissw(); /* PIPFREE2         */
    imu_modeexit();
    return;
}

/* ======================================================================= */
/* End of operation and stall (p.1438-1441)                                */
/* ======================================================================= */

/* GOODEND / BADEND common part (p.1439).  good: C(RUPTREG3) = -1, else -0.
 * DEVIATION: Comanche would JOBWAKE a negative MODECADR word; Luminary
 * resets it to +0 ("BZF = TCF IF MODECADR = -0").  Both do the latter.  */
static agc_status_t goodend_common(agc_imu_device_t dev, bool good)
{
    if (!AGC_ASSERT((int) dev >= 0 && dev < AGC_IMU_DEV_COUNT))
    {
        return AGC_STATUS_BAD_ARG;
    }
    modecadr_word_t        w   = mc_get((int) dev);
    agc_imu_stall_result_t res = good ? AGC_IMU_STALL_GOOD : AGC_IMU_STALL_BAD;
    if (w.state == MC_FREE)
    { /* ENDMODE          */
        mc_set((int) dev, good ? MC_DONE_GOOD : MC_DONE_BAD, AGC_JOB_NONE);
        return AGC_STATUS_OK;
    }
    if (w.state != MC_STALLED)
    { /* already negative */
        mc_set((int) dev, MC_FREE, AGC_JOB_NONE);
        return AGC_STATUS_OK;
    }
    mc_set((int) dev, MC_FREE, AGC_JOB_NONE); /* XCH MODECADR     */
    outcome[dev] = res;                       /* ADS LOC if good  */
    if (!AGC_ASSERT(w.job != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return agc_exec_job_wake(w.job); /* JOBWAKE          */
}

agc_status_t agc_imu_goodend(agc_imu_device_t dev)
{
    if (!AGC_ASSERT((int) dev >= 0 && dev < AGC_IMU_DEV_COUNT))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = goodend_common(dev, true);
    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_NOT_FOUND))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

agc_status_t agc_imu_badend(agc_imu_device_t dev)
{
    if (!AGC_ASSERT((int) dev >= 0 && dev < AGC_IMU_DEV_COUNT))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = goodend_common(dev, false);
    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_NOT_FOUND))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

/* IMUBAD (p.1438): BADEND with C(A) = 0 (the IMU).                       */
agc_status_t agc_imu_imubad(void)
{
    agc_status_t st = agc_imu_badend(AGC_IMU_DEV_IMU);
    if (!AGC_ASSERT(mc_get(AGC_IMU_DEV_IMU).state != MC_STALLED))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
}

/* ENDIMU (p.1438): the mode is bad if the ISS warning (channel 11 bit 1)
 * is on, otherwise IMUGOOD -> GOODEND.                                   */
agc_status_t agc_imu_endimu(void)
{
    if ((agc_chan_read(AGC_CHAN_DSALMOUT) & AGC_CH011_ISSWARN) != 0u)
    {
        return agc_imu_imubad();
    }
    agc_status_t st = agc_imu_goodend(AGC_IMU_DEV_IMU);
    if (!AGC_ASSERT(mc_get(AGC_IMU_DEV_IMU).state != MC_STALLED))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
}

/* MODABORT (p.1441): two programs using the same device.                 */
static agc_status_t modabort(void)
{
    _Static_assert(ALARM_TWO_USERS == 01210, "AGC constant");
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return agc_bailout(ALARM_TWO_USERS); /* BAILOUT1 01210   */
    }
    return agc_poodoo(ALARM_TWO_USERS); /* POODOO 01210     */
}

/* STALL / IMUSTALL / OPTSTALL / AOTSTALL / RADSTALL (pp.1440-1441):
 *   +0  operation incomplete: MODECADR = caller, JOBSLEEP (SLEEPING);
 *   -1  complete and good: MODECADR = +0, return L+3 (GOOD);
 *   -0  complete but failed: MODECADR = +0, return L+2 (BAD);
 *   CADR: a second user -> MODABORT.                                     */
agc_status_t agc_imu_stall_dev(agc_imu_device_t dev, agc_job_id_t wake_as,
                               agc_imu_stall_result_t *result)
{
    if (!AGC_ASSERT((int) dev >= 0 && dev < AGC_IMU_DEV_COUNT && result != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    modecadr_word_t w = mc_get((int) dev);
    if (w.state == MC_STALLED)
    {
        agc_relint();
        return modabort();
    }
    if (w.state == MC_FREE)
    { /* MODESLP          */
        if (!AGC_ASSERT(wake_as != AGC_JOB_NONE))
        {
            agc_relint();
            return AGC_STATUS_BAD_ARG;
        }
        mc_set((int) dev, MC_STALLED, wake_as);
        outcome[dev] = AGC_IMU_STALL_SLEEPING;
        *result      = AGC_IMU_STALL_SLEEPING;
        agc_relint();
        agc_exec_job_sleep(wake_as);
        return AGC_STATUS_OK;
    }
    *result =
        (w.state == MC_DONE_GOOD) ? AGC_IMU_STALL_GOOD : AGC_IMU_STALL_BAD;
    outcome[dev] = *result;
    mc_set((int) dev, MC_FREE, AGC_JOB_NONE); /* MG2: reset to +0 */
    imu_modeexit();
    return AGC_STATUS_OK;
}

agc_status_t agc_imu_stall(agc_job_id_t wake_as, agc_imu_stall_result_t *result)
{
    if (!AGC_ASSERT(result != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_imu_stall_dev(AGC_IMU_DEV_IMU, wake_as, result);
    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return st;
}

agc_imu_stall_result_t agc_imu_stall_outcome(agc_imu_device_t dev)
{
    if (!AGC_ASSERT((int) dev >= 0 && dev < AGC_IMU_DEV_COUNT))
    {
        return AGC_IMU_STALL_BAD;
    }
    if (!AGC_ASSERT(outcome[dev] <= AGC_IMU_STALL_SLEEPING))
    {
        return AGC_IMU_STALL_BAD;
    }
    return outcome[dev];
}

/* ======================================================================= */
/* R02 IMU status check (p.1443-1444)                                      */
/* ======================================================================= */

/* R02BOTH: if REFSMMAT is known set IMUSE and return OK.  Otherwise alarm
 * 00220 (REFSMMAT unknown) or 00210 (IMU not operating) via VARALARM and
 * GOTOPOOH; returns AGC_STATUS_POODOO.                                   */
agc_status_t agc_imu_status_check(void)
{
    if (agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        return agc_imu_set_in_use();
    }
    agc_alarm_code_t code = ALARM_REFSMMAT; /* OCT220          */
    if ((agc_erase_imodes30() & IM30_NOTOPER) != 0u)
    {
        code = ALARM_IMU_NOT_OPER; /* 220 - BIT4      */
    }
    if (!AGC_ASSERT(code == 00220 || code == 00210))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_exec_varalarm(code);
    const agc_status_t gs = agc_exec_gotopooh();
    return (gs != AGC_STATUS_OK) ? gs : AGC_STATUS_POODOO;
}

/* R02ZERO: UPFLAG IMUSE.                                                 */
agc_status_t agc_imu_set_in_use(void)
{
    agc_flag_set(AGC_FLAG_IMUSE);
    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_IMUSE)))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* P06 / PRESTAND / SCALPREP (pp.1445-1447): GNCS to standby.  Needs the
 * scaler channels and FINETIME, GOPERF1, and a restart to POSTAND.       */
agc_status_t agc_imu_standby(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_UNIMPLEMENTED;
}

/* POSTAND (pp.1447-1448): recover TIME1/TIME2 after standby.             */
agc_status_t agc_imu_postand(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_UNIMPLEMENTED;
}
