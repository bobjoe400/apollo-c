/**********************************************************************
 * FILE NAME: mode.c
 *
 * PURPOSE:
 *   IMU mode switching: zero (IMUZERO), coarse align (IMUCOARS, COARS,
 *   COARS2, CHKCORS), fine align (IMUFINE), PIPA use (PIPUSE, PIPFREE),
 *   the generalized end-of-operation and stall routines (GOODEND,
 *   BADEND, STALL), R02 (R02BOTH) and P06 standby / POSTAND.  Gyro
 *   torquing (IMUPULSE and its tasks) is in gyro.c.
 *
 * AGC SOURCE:
 *   Comanche055 IMU_MODE_SWITCHING_ROUTINES.agc pp. 1420-1448
 *   Luminary099 IMU_MODE_SWITCHING_ROUTINES.agc pp. 1309-1337 (differs:
 *   see NOTES)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   MODECADR      agc_sp_t   I/O  B0             agc_erase_modecadr()
 *   IMODES30/33   uint16_t   I/O  bits           agc_erase_imodes30/33()
 *   DSPTAB +11D   uint16_t   I/O  bits           ISS lamp word
 *   THETAD        uint16_t   I/O  CDU            agc_erase_thetad()
 *   CDUX..Z       uint16_t   I/O  CDU            agc_erase_cdux() ..
 *   PIPAX..Z      agc_sp_t   O    pulses B0      agc_erase_pipax() ..
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_waitlist_call     WAITLIST / VARDELAY / FIXDELAY
 *   agc_exec_job_wake     JOBWAKE (GOODEND), JOBSLEEP (STALL)
 *   agc_t4_setissw        SETISSW (ISS warning)
 *   agc_alarm             ALARM, VARALARM; agc_poodoo, agc_bailout
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 00206 (zero in coarse align and gimbal lock), 00210 (IMU not
 *   operating), 00211 (coarse align error), 00212 (PIPA fail at
 *   PIPFREE), 00220 (REFSMMAT not known, R02); POODOO (CM) / BAILOUT
 *   (LM) 01210 for two users of one device (MODABORT).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles.  The mode switches run under INHINT and end at
 *   MODEEXIT; their tasks run from the waitlist.
 *
 * NOTES:
 *   CM vs LM differences (the files are otherwise identical):
 *   * GOODEND: when MODECADR is already negative (a completed operation
 *     nobody has stalled on yet) Luminary resets it to +0; Comanche would
 *     JOBWAKE the negative word.  Modelled the Luminary way for both
 *     (DEVIATION, see goodend_common).
 *   * STALL with two users of one device: CM POODOO 01210, LM BAILOUT
 *     01210 (BAILOUT1).
 *   * RNDREFDR / R02BOTH: LM uses the named masks TRACKBIT, DRFTBIT,
 *     REFSMBIT; same bits.
 *   MODECADR model.  Each device word holds +0 (free or operation in
 *   progress), -1 (complete, good), -0 (complete, failed) or the CADR of
 *   a stalled job.  Here: +0, -1, -2 (DEVIATION: -0 has no agc_sp_t) or
 *   the job id to wake.  The "advance LOC if successful" of GOODEND
 *   becomes a per-device outcome the woken job reads with
 *   agc_imu_stall_outcome().
 *
 * ALGORITHM:
 *   As the AGC listing; each mode switch sets the hardware discretes,
 *   schedules its tasks and returns; the last task ends the operation
 *   with GOODEND or BADEND, which wakes the job stalled on the device.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "internal.h"

#define ALARM_ZERO_IN_GLOCK ((agc_alarm_code_t) 00206) /* zero in coarse+lock */
#define ALARM_IMU_NOT_OPER  ((agc_alarm_code_t) 00210) /* IMU not operating  */
#define ALARM_COARSE_ERR    ((agc_alarm_code_t) 00211) /* coarse align error  */
#define ALARM_PIP_FAIL      ((agc_alarm_code_t) 00212) /* PIPA fail at PIPFREE */
#define ALARM_REFSMMAT      ((agc_alarm_code_t) 00220) /* REFSMMAT not known  */
#define ALARM_TWO_USERS     ((agc_alarm_code_t) 01210) /* two users, 1 device */

#define DT_ZERO_RELEASE 32u     /* BIT6: 320 ms for the AGS pulse train, cs  */
#define DT_ZERO_SETTLE  1024u   /* BIT11: 10.24 s for the CDUs to find gimbals */
#define DT_COARS        6u      /* CAF SIX                                  */
#define DT_PULSES_START 2u      /* "minimum of 4 ms wait"                   */
#define DT_600MS        60u     /* 600MS DEC 60                             */
#define DT_SETTLE       150u    /* FIXDELAY DEC 150                         */
#define DT_IFAILOK      512u    /* BIT10: 5.12 s                            */
#define DT_2SECS        200u    /* 2SECS                                    */
#define COMMAX          191     /* -COMMAX  DEC -191 (max pulses minus one)  */
#define COMMAX_         192     /* -COMMAX- DEC -192                         */
#define COARSTOL        182     /* DEC -.01111 at half-rev = 2 deg, counts   */
#define CDU_MASK        077777u /* 15-bit CDU / channel word            */

typedef enum modecadr_state
{
    MC_FREE = 0,  /* +0                                        */
    MC_DONE_GOOD, /* -1                                        */
    MC_DONE_BAD,  /* -0                                        */
    MC_STALLED    /* CADR of a sleeping job                    */
} modecadr_state_t;

typedef struct modecadr_word
{
    modecadr_state_t state; /* +0, -1, -0 or a stalled job         */
    agc_job_id_t     job;   /* MC_STALLED: job to wake                   */
} modecadr_word_t;

/* MODECADR words live in exec (agc_exec.h; STARTSB2 clears them, SVCT3
 * tests IMUCADR).  Encoding of the SP word: +0 free, -1 good, a positive
 * job id for a stalled job, and -2 standing in for -0 (failed).
 * DEVIATION: agc_sp_t has no -0 (see AGC_MODECADR_BAD in imu.h).         */
#define MC_WORD_GOOD AGC_MODECADR_GOOD /* -1: complete and good    */
#define MC_WORD_BAD  AGC_MODECADR_BAD  /* -2 for -0: complete, bad */

/**********************************************************************
 * AGC: none (MODECADR word decoder) -- the state of one MODECADR word
 *
 * +0 free or in progress, -1 good, -2 (for -0) bad, > 0 the job id of a
 * stalled job.
 *
 * ARGUMENTS:
 *   dev  int  I    device index 0 IMU, 1 optics, 2 radar
 *
 * RETURNS: modecadr_word_t -- the decoded word; MC_FREE on a bad index
 **********************************************************************/
static modecadr_word_t mc_get(int dev)
{
    agc_sp_t        v = agc_erase_modecadr(dev); /* MODECADR +dev, B0 */
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
    }

    return (w);
}

/**********************************************************************
 * CKMODCAD (EXTENDED_VERBS.agc p.243 CM), DESRETRN (LM p.541) -- the
 * MODECADR test by BZF
 *
 * True when the word is +0 or -0, i.e. free or complete-but-failed.  With
 * the -2 stand-in for -0 (DEVIATION above) a plain "== 0" would miss -0.
 *
 * ARGUMENTS:
 *   dev  agc_imu_device_t  I    IMUCADR, OPTCADR or RADCADR
 *
 * RETURNS: bool -- true if C(MODECADR + dev) is +0 or -0
 **********************************************************************/
bool agc_imu_modecadr_bzf(agc_imu_device_t dev)
{
    agc_sp_t v = agc_erase_modecadr((int) dev); /* MODECADR, B0 */

    return ((v == AGC_MODECADR_FREE) || (v == MC_WORD_BAD));
}

/**********************************************************************
 * AGC: none (MODECADR word encoder) -- TS MODECADR +dev
 *
 * ARGUMENTS:
 *   dev  int               I    device index 0..2
 *   s    modecadr_state_t  I    new state
 *   job  agc_job_id_t      I    job to wake (MC_STALLED only)
 **********************************************************************/
static void mc_set(int dev, modecadr_state_t s, agc_job_id_t job)
{
    agc_sp_t v = 0; /* MODECADR word: +0 not in use (ENDMODE/MG2), B0 */

    if (s == MC_DONE_GOOD)
    {
        v = MC_WORD_GOOD; /* -1 */
    }
    else if (s == MC_DONE_BAD)
    {
        v = MC_WORD_BAD; /* -0 */
    }
    else if (s == MC_STALLED)
    {
        v = (agc_sp_t) job; /* CADR of the sleeping job */
    }
    else
    {
        v = 0; /* +0 */
    }

    agc_erase_set_modecadr(dev, v); /* TS MODECADR */
}

static agc_imu_stall_result_t outcome[AGC_IMU_DEV_COUNT]; /* L+2/L+3 */
static agc_sp_t               command[3];                 /* COMMAND: pulses to go, B0  */
static agc_sp_t               cducmd[3];                  /* CDUXCMD..: pulses, B0       */

/* ---- small helpers ------------------------------------------------------ */

/**********************************************************************
 * AGC: none (CS/MASK helper) -- the 15-bit complement of a mask
 *
 * ARGUMENTS:
 *   m  uint16_t  I    mask, 15 bits
 *
 * RETURNS: uint16_t -- ~m within 15 bits
 **********************************************************************/
static uint16_t notmask(uint16_t m)
{
    uint16_t r = (uint16_t) (~(unsigned int) m & CDU_MASK); /* CS  */

    return (r);
}

/**********************************************************************
 * AGC: none (CS IMODES30, MASK, ADS IMODES30) -- set IMODES30 bits
 *
 * ARGUMENTS:
 *   bits  uint16_t  I    bits to set
 **********************************************************************/
static void imodes30_or(uint16_t bits)
{
    agc_erase_set_imodes30((uint16_t) (agc_erase_imodes30() | bits));
}

/**********************************************************************
 * AGC: none (CS bits, MASK IMODES30, TS) -- clear IMODES30 bits
 *
 * ARGUMENTS:
 *   bits  uint16_t  I    bits to clear
 **********************************************************************/
static void imodes30_clear(uint16_t bits)
{
    agc_erase_set_imodes30((uint16_t) (agc_erase_imodes30() & notmask(bits)));
}

/**********************************************************************
 * AGC: none (IMODES33 ADS / MASK) -- set or clear IMODES33 bits
 *
 * ARGUMENTS:
 *   bits  uint16_t  I    bits to change
 *   on    bool      I    true: set, false: clear
 **********************************************************************/
static void imodes33_set(uint16_t bits, bool on)
{
    uint16_t v = agc_erase_imodes33(); /* IMODES33 bits */

    v = on ? (uint16_t) (v | bits) : (uint16_t) (v & notmask(bits)); /* ADS / MASK */
    agc_erase_set_imodes33(v);
}

/**********************************************************************
 * AGC: none (TS DSPTAB +11D) -- store the ISS lamp word
 *
 * ARGUMENTS:
 *   v  uint16_t  I    new DSPTAB +11D word (bit 15 = changed)
 **********************************************************************/
static void iss_lamps_write(uint16_t v)
{
    agc_erase_set_dsptab(11, v);
}

/**********************************************************************
 * AGC: none (CS bits, EXTEND, WAND chan) -- clear channel bits
 *
 * ARGUMENTS:
 *   chan  int       I    channel 12, 13 or 14
 *   bits  uint16_t  I    bits to clear
 **********************************************************************/
static void chan_clear(int chan, uint16_t bits)
{
    agc_chan_and(chan, notmask(bits));
}

/**********************************************************************
 * SETISSW (T4RUPT_PROGRAM.agc, IBNKCALL) -- set or reset the ISS
 * warning from IMODES30
 **********************************************************************/
static void setissw(void)
{
    agc_t4_setissw(); /* IBNKCALL SETISSW      */
}

/**********************************************************************
 * AGC: none (TC WAITLIST, 2CADR task) -- schedule a mode-switch task
 *
 * ARGUMENTS:
 *   dt    uint16_t       I    delay, cs, 1..AGC_WAITLIST_MAX_DELAY
 *   task  agc_task_id_t  I    task to start
 *
 * RETURNS: agc_status_t -- OK, BAILOUT (1203) when the waitlist is full,
 *          POODOO (1204) or BAD_ARG on a bad delay
 **********************************************************************/
static agc_status_t waitlist(uint16_t dt, agc_task_id_t task)
{
    agc_status_t st = agc_waitlist_call(dt, task); /* status of the call */

    return (st);
}

/**********************************************************************
 * AGC: none (EXTEND, MSU) -- CDU difference a - b
 *
 * 15-bit two's-complement difference as a signed count (MSU leaves it in
 * ones' complement, range -16383..+16383).
 *
 * ARGUMENTS:
 *   a  uint16_t  I    minuend, CDU
 *   b  uint16_t  I    subtrahend, CDU
 *
 * RETURNS: int -- a - b in CDU counts
 **********************************************************************/
static int cdu_msu(uint16_t a, uint16_t b)
{
    int d = (int) ((unsigned int) (a - b) & CDU_MASK); /* MSU, CDU */

    if (d >= 040000)
    {
        d -= 0100000;
    }

    return (d);
}

/**********************************************************************
 * AGC: none (INDEX CDUIND, CA CDUX) -- one IMU CDU
 *
 * ARGUMENTS:
 *   axis  int  I    0 X, 1 Y, 2 Z
 *
 * RETURNS: uint16_t -- CDUX, CDUY or CDUZ, CDU
 **********************************************************************/
static uint16_t cdu_read(int axis)
{
    uint16_t v = agc_erase_cdux(); /* INDEX CDUIND: CDUX, Y, Z, CDU */

    if (axis == 1)
    {
        v = agc_erase_cduy();
    }
    else if (axis == 2)
    {
        v = agc_erase_cduz();
    }
    else
    {
        v = agc_erase_cdux();
    }

    return (v);
}

/* ---- shared helpers (internal.h) ----------------------------------- */

/**********************************************************************
 * CAGETEST (IMU_MODE_SWITCHING_ROUTINES.agc p.1438) -- CAF BIT6, MASK
 * IMODES30: IMU being caged
 *
 * RETURNS: bool -- true when IMODES30 bit 6 (cage) is set
 **********************************************************************/
bool imu_caged(void)
{
    uint16_t m = agc_erase_imodes30(); /* IMODES30 bits */

    return ((m & IM30_CAGE) != 0u);
}

/**********************************************************************
 * MODEEXIT (IMU_MODE_SWITCHING_ROUTINES.agc p.1422) -- RELINT, TCF
 * SWRETURN
 *
 * IMUPULSE reaches MODEEXIT without an INHINT; RELINT is harmless there,
 * but the C inhibit is counted, so only a held one is released.
 **********************************************************************/
void imu_modeexit(void)
{
    /* IMUPULSE reaches MODEEXIT without an INHINT; RELINT is harmless
     * there, but the C inhibit is counted, so only release a held one.  */
    if (agc_interrupts_inhibited())
    {
        agc_relint(); /* MODEEXIT: RELINT */
    }

    /* TCF SWRETURN     */
}

/**********************************************************************
 * CAGETSTJ (IMU_MODE_SWITCHING_ROUTINES.agc p.1438) -- caged during
 * mode-switch initialization
 *
 * If the IMU is being caged, IMUCADR = -0 (complete but failed) and
 * MODEEXIT.
 *
 * RETURNS: agc_status_t -- OK to go on, BUSY when the caller must return
 *          at once (MODEEXIT done); AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t imu_cagetstj(void)
{
    if (!imu_caged())
    {
        return (AGC_STATUS_OK); /* TC Q             */
    }

    mc_set(AGC_IMU_DEV_IMU, MC_DONE_BAD, AGC_JOB_NONE); /* CS ZERO; TS IMUCADR */

    if (!AGC_ASSERT(mc_get(AGC_IMU_DEV_IMU).state == MC_DONE_BAD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    imu_modeexit();

    if (!AGC_ASSERT(agc_imu_modecadr_bzf(AGC_IMU_DEV_IMU)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_BUSY);
}

/**********************************************************************
 * CAGETEST (IMU_MODE_SWITCHING_ROUTINES.agc p.1438) -- in a task,
 * IMUBAD if the IMU has been caged
 *
 * ARGUMENTS:
 *   caged  bool *  O    true when the task must end (IMUBAD done)
 *
 * RETURNS: agc_status_t -- OK, or the IMUBAD status
 **********************************************************************/
static agc_status_t cagetest(bool *caged)
{
    *caged = imu_caged();

    return (*caged ? agc_imu_imubad() : AGC_STATUS_OK); /* IMUBAD when caged, else OK */
}

/**********************************************************************
 * AGC: none (fresh start) -- MODECADR words +0, no pulses outstanding
 *
 * RETURNS: none
 **********************************************************************/
void agc_imu_mode_init(void)
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
}

/**********************************************************************
 * ZEROICDU (IMU_MODE_SWITCHING_ROUTINES.agc p.1420) -- zero the ICDU
 * counters
 **********************************************************************/
void agc_imu_zeroicdu(void)
{
    agc_erase_set_cdux(0);
    agc_erase_set_cduy(0);
    agc_erase_set_cduz(0);
}

/**********************************************************************
 * NOATTOFF (IMU_MODE_SWITCHING_ROUTINES.agc p.1428) -- NO ATT lamp off
 * (DSPTAB +11D bit 4), bit 15 set
 **********************************************************************/
void agc_imu_noattoff(void)
{
    uint16_t v = agc_erase_dsptab(11); /* DSPTAB +11D lamps */

    v = (uint16_t) ((v & notmask(ISS_LAMP_NO_ATT | ISS_LAMPS_CHANGED)) | ISS_LAMPS_CHANGED);
    iss_lamps_write(v);
}

/**********************************************************************
 * AGC: none (fresh start) -- CDUXCMD..CDUZCMD and GYROCMD = -0
 **********************************************************************/
void agc_imu_clear_cmds(void)
{
    for (int i = 0; i < 3; i++)
    {
        cducmd[i] = 0; /* -0: no pulses outstanding    */
    }

    agc_imu_hw_set_gyrocmd(0);
}

/**********************************************************************
 * AGC: none (accessor) -- CDUXCMD, CDUYCMD or CDUZCMD
 *
 * ARGUMENTS:
 *   axis  int  I    0 X, 1 Y, 2 Z
 *
 * RETURNS: agc_sp_t -- coarse-align pulses outstanding, B0
 **********************************************************************/
agc_sp_t agc_imu_cducmd(int axis)
{
    return (cducmd[axis]);
}

/**********************************************************************
 * AGC: none (sim hook) -- the counter as the CDU hardware left it, or
 * a program's write: the whole SP word (coarse align sends <= COMMAX =
 * 192 a time, but NEEDLES' ADS CDUXCMD and V41 write up to +-768 and
 * more: the counter is an erasable word).
 *
 * ARGUMENTS:
 *   axis    int       I    0 X, 1 Y, 2 Z
 *   pulses  agc_sp_t  I    pulses still to send, SP range, B0
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG for a bad axis or a value
 *          outside the SP word
 **********************************************************************/
agc_status_t agc_imu_hw_set_cducmd(int axis, agc_sp_t pulses)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3) || !AGC_ASSERT(pulses >= AGC_SP_MIN && pulses <= AGC_SP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    cducmd[axis] = pulses;

    return (AGC_STATUS_OK);
}

/* ======================================================================= */
/* IMU zeroing (p.1421-1422)                                               */
/* ======================================================================= */

/**********************************************************************
 * IMUZERO / IMUZEROA (IMU_MODE_SWITCHING_ROUTINES.agc pp.1421-1422) --
 * zero the ICDUs
 *
 * Refused (ALARM 00206, immediate failure) when the IMU is in coarse
 * align and gimbal lock (runaway protection).  Otherwise DAP auto and hold
 * off, IMU and ICDU fail inhibited, ZERO ICDU on, IMUZERO2 in 320 ms;
 * ALARM 00210 if the IMU is not operating.
 *
 * RETURNS: agc_status_t -- OK (the outcome is reported through IMUCADR),
 *          or the waitlist status; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_imu_zero(void)
{
    agc_inhint();
    uint16_t lamps = agc_erase_dsptab(11); /* DSPTAB +11D */

    if (!AGC_ASSERT(lamps <= CDU_MASK))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    if ((notmask(lamps) & (ISS_LAMP_NO_ATT | ISS_LAMP_GIMBAL_LOCK)) == 0u)
    {
        agc_alarm(ALARM_ZERO_IN_GLOCK);
        mc_set(AGC_IMU_DEV_IMU, MC_DONE_BAD, AGC_JOB_NONE); /* CAGETSTJ +4 */
        imu_modeexit();

        return (AGC_STATUS_OK);
    }

    if (imu_cagetstj() != AGC_STATUS_OK)
    { /* IMUZEROA       */
        return (AGC_STATUS_OK);
    }

    imodes33_set(IM33_DAPDIS, true); /* disable DAP auto, hold */
    imodes30_or(IM30_ICDUFAILINH | IM30_IMUFAILINH);
    chan_clear(AGC_CHAN_CHAN12, AGC_CH012_COARSE | AGC_CH012_ENBIMUERR);
    agc_imu_noattoff();
    agc_chan_or(AGC_CHAN_CHAN12, AGC_CH012_ZEROICDU);

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) == 0u))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    agc_imu_zeroicdu();
    agc_status_t st = waitlist(DT_ZERO_RELEASE, AGC_TASK_IMU_ZERO_RELEASE); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    if ((agc_erase_imodes30() & IM30_NOTOPER) != 0u)
    {
        agc_alarm(ALARM_IMU_NOT_OPER); /* IMU not operating */
    }

    imu_modeexit();

    return (AGC_STATUS_OK);
} /* agc_imu_zero */

/**********************************************************************
 * IMUZERO2 (IMU_MODE_SWITCHING_ROUTINES.agc p.1422) -- task: remove the
 * zero discrete
 *
 * Zero CDUX..Z again, drop channel 12 bit 5, VARDELAY 10.24 s for the
 * counters to find the gimbals.
 *
 * RETURNS: agc_status_t -- VARDELAY status; AGC_STATUS_BAD_ARG on a
 *          failed precondition
 **********************************************************************/
agc_status_t agc_task_imu_zero_release(void)
{
    bool         caged = false;            /* IMU caged: IMUBAD done */
    agc_status_t cst   = cagetest(&caged); /* IMUBAD status */

    if (caged)
    {
        return (cst);
    }

    agc_imu_zeroicdu();
    chan_clear(AGC_CHAN_CHAN12, AGC_CH012_ZEROICDU);

    if (!AGC_ASSERT(agc_erase_cdux() == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_waitlist_vardelay(DT_ZERO_SETTLE, AGC_TASK_IMU_ZERO_DONE); /* status of the call */

    return (st);
}

/**********************************************************************
 * IMUZERO3 (IMU_MODE_SWITCHING_ROUTINES.agc p.1422) -- task: remove the
 * fail inhibits, DAP on, ENDIMU
 *
 * RETURNS: agc_status_t -- ENDIMU status; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_imu_zero_done(void)
{
    bool         caged = false;            /* IMU caged: IMUBAD done */
    agc_status_t cst   = cagetest(&caged); /* IMUBAD status */

    if (caged)
    {
        return (cst);
    }

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_ZEROICDU) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    imodes30_clear(IM30_ICDUFAILINH | IM30_IMUFAILINH);
    imodes33_set(IM33_DAPDIS, false);
    setissw();

    if (!AGC_ASSERT(!imu_caged()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_ZEROICDU) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_imu_endimu());
}

/* ======================================================================= */
/* Coarse align (p.1423-1426)                                              */
/* ======================================================================= */

/**********************************************************************
 * IMUCOARS (IMU_MODE_SWITCHING_ROUTINES.agc p.1423) -- coarse align to
 * THETAD
 *
 * ARGUMENTS:
 *   thetad  const uint16_t [3]  I    THETAD X, Y, Z, CDU patterns
 *
 * RETURNS: agc_status_t -- OK (outcome through IMUCADR), or the waitlist
 *          status; AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_imu_coarse(const uint16_t thetad[3])
{
    for (int i = 0; i < 3; i++)
    {
        if (!AGC_ASSERT(thetad[i] <= CDU_MASK))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_erase_set_thetad(i, thetad[i]);
    }

    agc_inhint();

    if (imu_cagetstj() == AGC_STATUS_OK)
    {
        agc_imu_setcoars();
        agc_status_t st = waitlist(DT_COARS, AGC_TASK_COARSE_ALIGN_COMMANDS); /* status of the call */

        if (st != AGC_STATUS_OK)
        {
            agc_relint();

            return (st);
        }

        imu_modeexit();
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * COARS (IMU_MODE_SWITCHING_ROUTINES.agc p.1423) -- task: enable the
 * CDU error counters, COMMAND
 *
 * RETURNS: agc_status_t -- VARDELAY status; AGC_STATUS_BAD_ARG on a
 *          failed precondition
 **********************************************************************/
agc_status_t agc_task_coarse_align_commands(void)
{
    bool         caged = false;            /* IMU caged: IMUBAD done */
    agc_status_t cst   = cagetest(&caged); /* IMUBAD status */

    if (caged)
    {
        return (cst);
    }

    agc_chan_or(AGC_CHAN_CHAN12, AGC_CH012_ENBIMUERR);

    for (int cduind = 2; cduind >= 0; cduind--)
    { /* CDUIND 2, 1, 0  */
        /* COARS1 (IMU_MODE_SWITCHING_ROUTINES.agc p.1423) -- COMMAND for one
         * CDU
         * (THETAD - CDU) MP BIT13 (shift right 2) and rounded: pulses at 4 per
         * CDU count. */
        int diff = cdu_msu(agc_erase_thetad(cduind), cdu_read(cduind)); /* CDU */
        int mag  = (diff < 0) ? -diff : diff;                           /* |THETAD - CDU|, CDU    */
        int cmd  = (mag >> 2) + (((mag & 3) >= 2) ? 1 : 0);             /* MP BIT13, round */

        cmd             = (diff < 0) ? -cmd : cmd; /* sign of the difference */
        command[cduind] = (agc_sp_t) cmd;
    }

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_ENBIMUERR) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_waitlist_vardelay(DT_PULSES_START, AGC_TASK_COARSE_ALIGN_PULSES); /* status of the call */

    return (st);
}

/**********************************************************************
 * COMPOS / COMNEG / COMZERO (IMU_MODE_SWITCHING_ROUTINES.agc
 * pp.1424-1425) -- one 600 ms slice
 *
 * At most 192 pulses per CDU; the remainder stays in COMMAND.
 *
 * ARGUMENTS:
 *   axis  int  I    CDUIND 0..2
 *
 * RETURNS: agc_sp_t -- pulses to send now (CDUXCMD), B0
 **********************************************************************/
static agc_sp_t coarse_slice(int axis)
{
    int c   = command[axis];    /* COMMAND, pulses             */
    int mag = (c < 0) ? -c : c; /* CCS COMMAND magnitude       */

    if (mag - 1 - COMMAX <= 0)
    { /* COMZERO          */
        command[axis] = 0;

        return ((agc_sp_t) c);
    }

    command[axis] = (agc_sp_t) ((c < 0) ? c + COMMAX_ : c - COMMAX_); /* COMNEG/COMPOS */

    return ((agc_sp_t) ((c < 0) ? -COMMAX_ : COMMAX_)); /* -COMMAX- */
}

/**********************************************************************
 * COARS2 / NEXTCDU / SENDPULS (IMU_MODE_SWITCHING_ROUTINES.agc p.1424)
 * -- task: send the coarse-align pulses
 *
 * Every 600 ms until the commands are used up, then FIXDELAY 1.5 s for
 * the gimbals to settle and CHKCORS.
 *
 * RETURNS: agc_status_t -- VARDELAY / FIXDELAY status; AGC_STATUS_BAD_ARG
 *          on a failed precondition
 **********************************************************************/
agc_status_t agc_task_coarse_align_pulses(void)
{
    bool         caged = false;            /* IMU caged: IMUBAD done */
    agc_status_t cst   = cagetest(&caged); /* IMUBAD status */

    if (caged)
    {
        return (cst);
    }

    int itemp1 = 0; /* pulses to go?    */

    for (int cduind = 2; cduind >= 0; cduind--)
    {
        agc_sp_t slice = (command[cduind] == 0) ? 0 : coarse_slice(cduind); /* CCS COMMAND, pulses B0 */

        itemp1         += (slice != 0) ? 1 : 0; /* NEXTCDU INCR     */
        cducmd[cduind] = slice;                 /* TS CDUXCMD       */
    }

    if (!AGC_ASSERT(itemp1 >= 0 && itemp1 <= 3))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st; /* status of the call */

    if (itemp1 > 0)
    { /* SENDPULS         */
        agc_chan_or(AGC_CHAN_CHAN14, AGC_CH014_DRIVECDUX | AGC_CH014_DRIVECDUY | AGC_CH014_DRIVECDUZ);
        st = agc_waitlist_vardelay(DT_600MS, AGC_TASK_COARSE_ALIGN_PULSES);
    }
    else
    {
        st = agc_waitlist_fixdelay(DT_SETTLE, AGC_TASK_COARSE_ALIGN_CHECK);
    }

    return (st);
}

/**********************************************************************
 * CHKCORS / CORSCHK2 / COARSERR (IMU_MODE_SWITCHING_ROUTINES.agc
 * pp.1424-1425) -- task: gimbals within 2 deg
 *
 * ALARM 00211 and IMUBAD when a gimbal is more than COARSTOL from
 * THETAD, else ENDIMU.
 *
 * RETURNS: agc_status_t -- IMUBAD or ENDIMU status; AGC_STATUS_BAD_ARG on
 *          a failed precondition
 **********************************************************************/
agc_status_t agc_task_coarse_align_check(void)
{
    if (!AGC_ASSERT(command[0] == 0 && command[1] == 0 && command[2] == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(cducmd[0] >= -COMMAX_ && cducmd[0] <= COMMAX_))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 2; i >= 0; i--)
    {
        int diff = cdu_msu(cdu_read(i), agc_erase_thetad(i)); /* CDU   */
        int mag  = (diff < 0) ? -diff : diff;                 /* CCS magnitude, CDU  */

        if (mag != 0 && (mag - 1) - COARSTOL > 0)
        { /* COARSERR         */
            agc_alarm(ALARM_COARSE_ERR);

            return (agc_imu_imubad());
        }
    }

    return (agc_imu_endimu()); /* end of coarse    */
}

/**********************************************************************
 * CA+ECE (IMU_MODE_SWITCHING_ROUTINES.agc p.1425) -- task: enable the
 * three ISS CDU error counters
 *
 * RETURNS: agc_status_t -- OK (TASKOVER); AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_ca_ece(void)
{
    agc_chan_or(AGC_CHAN_CHAN12, AGC_CH012_ENBIMUERR);

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_ENBIMUERR) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* TASKOVER         */
}

/**********************************************************************
 * SETCOARS (IMU_MODE_SWITCHING_ROUTINES.agc p.1426) -- put the ISS in
 * coarse align (bypassed if already)
 *
 * Error counters off, gyro activity off, GYROCMD -0, coarse on, NO ATT
 * lamp on, DAP auto/hold off, IMU fail inhibited, then RNDREFDR.
 **********************************************************************/
void agc_imu_setcoars(void)
{
    if ((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) == 0u)
    {
        chan_clear(AGC_CHAN_CHAN12, AGC_CH012_ENBIMUERR); /* clear error ctrs */
        chan_clear(AGC_CHAN_CHAN14, AGC_CH014_GYROACT);   /* gyro activity   */
        agc_imu_hw_set_gyrocmd(0);                        /* CS ZERO: -0     */
        agc_chan_or(AGC_CHAN_CHAN12, AGC_CH012_COARSE);
        iss_lamps_write((uint16_t) (agc_erase_dsptab(11) | ISS_LAMP_NO_ATT | ISS_LAMPS_CHANGED));
        imodes33_set(IM33_DAPDIS6, true);
        imodes30_or(IM30_IMUFAILINH);

        agc_imu_rndrefdr();
    }
}

/**********************************************************************
 * RNDREFDR (IMU_MODE_SWITCHING_ROUTINES.agc p.1426) -- knock down
 * TRACKFLG, DRIFTFLG, REFSMFLG
 **********************************************************************/
void agc_imu_rndrefdr(void)
{
    agc_flag_clear(AGC_FLAG_TRACKFLG);
    agc_flag_clear(AGC_FLAG_DRIFTFLG);
    agc_flag_clear(AGC_FLAG_REFSMFLG);
}

/* ======================================================================= */
/* Fine align (p.1427-1428)                                                */
/* ======================================================================= */

/**********************************************************************
 * IMUFINE = IMUFIN20 (IMU_MODE_SWITCHING_ROUTINES.agc p.1427) -- fine
 * align mode switch
 *
 * Zero and coarse off, DAP auto/hold on, NO ATT off; IFAILOK in 5.12 s
 * (IMU fail stays inhibited), IMUFINED (ENDIMU) in 2 s.
 *
 * RETURNS: agc_status_t -- OK (outcome through IMUCADR), or the waitlist
 *          status; AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_imu_fine(void)
{
    agc_inhint();

    if (imu_cagetstj() != AGC_STATUS_OK)
    {
        return (AGC_STATUS_OK);
    }

    chan_clear(AGC_CHAN_CHAN12, AGC_CH012_COARSE | AGC_CH012_ZEROICDU);
    imodes33_set(IM33_DAPDIS6, false);
    agc_imu_noattoff();

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) == 0u))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT((agc_erase_dsptab(11) & ISS_LAMP_NO_ATT) == 0u))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t sw = waitlist(DT_IFAILOK, AGC_TASK_IFAILOK); /* status of the IFAILOK call */

    if (sw != AGC_STATUS_OK)
    {
        agc_relint();

        return (sw);
    }

    agc_status_t st = waitlist(DT_2SECS, AGC_TASK_IMUFINED); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        agc_relint();

        return (st);
    }

    imu_modeexit();

    return (AGC_STATUS_OK);
} /* agc_imu_fine */

/**********************************************************************
 * IMUFINED (IMU_MODE_SWITCHING_ROUTINES.agc p.1427) -- task: not caged,
 * then ENDIMU
 *
 * RETURNS: agc_status_t -- ENDIMU status; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_imufined(void)
{
    bool         caged = false;            /* IMU caged: IMUBAD done */
    agc_status_t cst   = cagetest(&caged); /* IMUBAD status */

    if (caged)
    {
        return (cst);
    }

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_ZEROICDU) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_imu_endimu());
}

/**********************************************************************
 * PFAILOK2 (IMU_MODE_SWITCHING_ROUTINES.agc p.1428) -- clear a fail
 * inhibit, SETISSW, TASKOVER
 *
 * ARGUMENTS:
 *   clear_bits  uint16_t  I    IMODES30 bit 4 (IMU) or bit 5 (PIPA)
 *
 * RETURNS: agc_status_t -- OK (TASKOVER); AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
static agc_status_t fail_inhibit_remove(uint16_t clear_bits)
{
    imodes30_clear(clear_bits);

    if (!AGC_ASSERT(!imu_caged()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    setissw(); /* the ISS warning may come on now               */

    return (AGC_STATUS_OK); /* TASKOVER         */
}

/**********************************************************************
 * IFAILOK (IMU_MODE_SWITCHING_ROUTINES.agc p.1428) -- task: enable IMU
 * fail
 *
 * Not when caged or when someone has gone into coarse align again.
 *
 * RETURNS: agc_status_t -- OK (TASKOVER); AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_ifailok(void)
{
    if (imu_caged())
    { /* CAGETSTQ         */
        return (AGC_STATUS_OK);
    }

    if ((agc_chan_read(AGC_CHAN_CHAN12) & AGC_CH012_COARSE) != 0u)
    {
        return (AGC_STATUS_OK);
    }

    if (!AGC_ASSERT(!imu_caged()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    imodes30_or(IM30_IMUOK); /* reset IMUFAIL    */

    if (!AGC_ASSERT((agc_erase_imodes30() & IM30_IMUOK) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (fail_inhibit_remove(IM30_IMUFAILINH));
}

/**********************************************************************
 * PFAILOK (IMU_MODE_SWITCHING_ROUTINES.agc p.1428) -- task: enable the
 * PIPA fail program alarm
 *
 * RETURNS: agc_status_t -- OK (TASKOVER); AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_task_pfailok(void)
{
    if (imu_caged())
    {
        return (AGC_STATUS_OK);
    }

    if (!AGC_ASSERT(!imu_caged()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    imodes30_or(IM30_PIPOK); /* reset fail bits  */
    imodes33_set(IM33_PIPOK, true);

    if (!AGC_ASSERT((agc_erase_imodes30() & IM30_PIPOK) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (fail_inhibit_remove(IM30_PIPFAILINH));
}

/* ======================================================================= */
/* PIPA use (p.1429)                                                       */
/* ======================================================================= */

/**********************************************************************
 * PIPUSE (IMU_MODE_SWITCHING_ROUTINES.agc p.1429) -- PIPAX..Z = -0,
 * then PIPUSE1
 *
 * RETURNS: none
 **********************************************************************/
void agc_imu_pipuse(void)
{
    agc_erase_set_pipax(0); /* CS ZERO: -0      */
    agc_erase_set_pipay(0);
    agc_erase_set_pipaz(0);

    agc_imu_pipa_fail_enable();
}

/**********************************************************************
 * PIPUSE1 / PIPFREE2 (IMU_MODE_SWITCHING_ROUTINES.agc p.1429) -- PIPA
 * fail lights the ISS warning from now on
 *
 * Until PIPFREE; not when the IMU is caged.
 **********************************************************************/
void agc_imu_pipa_fail_enable(void)
{
    if (!imu_caged())
    {
        agc_inhint();
        imodes30_clear(IM30_PIPNOTUSED);

        setissw(); /* PIPFREE2         */
        imu_modeexit();
    }
}

/**********************************************************************
 * PIPFREE (IMU_MODE_SWITCHING_ROUTINES.agc p.1429) -- program done with
 * the PIPAs
 *
 * Set "PIPAs not in use"; if PIPA fail is on, ALARM 00212 and SETISSW.
 **********************************************************************/
void agc_imu_pipfree(void)
{
    agc_inhint();
    imodes30_or(IM30_PIPNOTUSED);

    if ((agc_erase_imodes30() & IM30_PIPOK) != 0u)
    {
        imu_modeexit();
    }
    else
    {
        /* ALARM 212.  The AGC's ALARM ends with RELINT, so PIPFREE does
         * INHINT again; agc_alarm() leaves this routine's inhibit in place,
         * so a second agc_inhint() here would leave interrupts off.       */
        agc_alarm(ALARM_PIP_FAIL);
        setissw(); /* PIPFREE2         */
        imu_modeexit();
    }
}

/* ======================================================================= */
/* End of operation and stall (p.1438-1441)                                */
/* ======================================================================= */

/**********************************************************************
 * GOODEND / BADEND / ENDMODE (IMU_MODE_SWITCHING_ROUTINES.agc p.1439)
 * -- end of an I/O operation
 *
 * +0: nobody stalled yet, MODECADR = -1 (good) or -0 (bad).  A stalled
 * job: MODECADR = +0, its outcome kept, JOBWAKE.
 * DEVIATION: Comanche would JOBWAKE a negative MODECADR word; Luminary
 * resets it to +0 ("BZF = TCF IF MODECADR = -0").  Both do the latter.
 *
 * ARGUMENTS:
 *   dev   agc_imu_device_t  I    device index (C(A) on arrival)
 *   good  bool              I    true at GOODEND (RUPTREG3 = -1), false
 *                                at BADEND (-0)
 *
 * RETURNS: agc_status_t -- OK, or the JOBWAKE status; AGC_STATUS_BAD_ARG
 *          on a failed precondition
 **********************************************************************/
static agc_status_t goodend_common(agc_imu_device_t dev, bool good)
{
    modecadr_word_t        w   = mc_get((int) dev);                             /* MODECADR +dev */
    agc_imu_stall_result_t res = good ? AGC_IMU_STALL_GOOD : AGC_IMU_STALL_BAD; /* RUPTREG3 */

    if (w.state == MC_FREE)
    { /* ENDMODE          */
        mc_set((int) dev, good ? MC_DONE_GOOD : MC_DONE_BAD, AGC_JOB_NONE); /* -1 / -0 */

        return (AGC_STATUS_OK);
    }

    if (w.state != MC_STALLED)
    { /* already negative */
        mc_set((int) dev, MC_FREE, AGC_JOB_NONE);

        return (AGC_STATUS_OK);
    }

    mc_set((int) dev, MC_FREE, AGC_JOB_NONE); /* XCH MODECADR     */
    outcome[dev] = res;                       /* ADS LOC if good  */

    if (!AGC_ASSERT(w.job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_job_wake(w.job)); /* JOBWAKE          */
}

/**********************************************************************
 * GOODEND (IMU_MODE_SWITCHING_ROUTINES.agc p.1439) -- successful end of
 * an I/O operation
 *
 * ARGUMENTS:
 *   dev  agc_imu_device_t  I    device index
 *
 * RETURNS: agc_status_t -- OK or NOT_FOUND (no such job to wake)
 **********************************************************************/
agc_status_t agc_imu_goodend(agc_imu_device_t dev)
{
    agc_status_t st = goodend_common(dev, true); /* status of the call */

    return (st);
}

/**********************************************************************
 * BADEND (IMU_MODE_SWITCHING_ROUTINES.agc p.1439) -- failed end of an
 * I/O operation
 *
 * ARGUMENTS:
 *   dev  agc_imu_device_t  I    device index
 *
 * RETURNS: agc_status_t -- OK or NOT_FOUND (no such job to wake);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_imu_badend(agc_imu_device_t dev)
{
    if (!AGC_ASSERT((int) dev >= 0 && dev < AGC_IMU_DEV_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = goodend_common(dev, false); /* status of the call */

    return (st);
}

/**********************************************************************
 * IMUBAD (IMU_MODE_SWITCHING_ROUTINES.agc p.1438) -- BADEND with C(A) =
 * 0 (the IMU)
 *
 * RETURNS: agc_status_t -- OK, or BAD_ARG
 **********************************************************************/
agc_status_t agc_imu_imubad(void)
{
    agc_status_t st = agc_imu_badend(AGC_IMU_DEV_IMU); /* status of the call */

    if (!AGC_ASSERT(mc_get(AGC_IMU_DEV_IMU).state != MC_STALLED))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st); /* no job woken */
}

/**********************************************************************
 * ENDIMU / IMUGOOD (IMU_MODE_SWITCHING_ROUTINES.agc p.1438) -- end of
 * an IMU mode switch
 *
 * Bad (IMUBAD) if the ISS warning (channel 11 bit 1) is on, else GOODEND.
 *
 * RETURNS: agc_status_t -- OK, or BAD_ARG
 **********************************************************************/
agc_status_t agc_imu_endimu(void)
{
    if ((agc_chan_read(AGC_CHAN_DSALMOUT) & AGC_CH011_ISSWARN) != 0u)
    {
        return (agc_imu_imubad());
    }

    agc_status_t st = agc_imu_goodend(AGC_IMU_DEV_IMU); /* status of the call */

    return ((st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st); /* no job woken */
}

/**********************************************************************
 * MODABORT (IMU_MODE_SWITCHING_ROUTINES.agc p.1441) -- two programs
 * using the same device
 *
 * CM POODOO 01210, LM BAILOUT1 01210.
 *
 * ARGUMENTS:
 *   dev  agc_imu_device_t  I    device whose MODECADR holds a stalled job
 *
 * RETURNS: agc_status_t -- POODOO or BAILOUT status; AGC_STATUS_BAD_ARG
 *          on a failed precondition
 **********************************************************************/
static agc_status_t modabort(agc_imu_device_t dev)
{
    _Static_assert(ALARM_TWO_USERS == 01210, "AGC constant");
    const agc_vehicle_t v = agc_state_vehicle(); /* CM POODOO, LM BAILOUT */

    if (!AGC_ASSERT(mc_get((int) dev).state == MC_STALLED)) /* 2nd user */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (v == AGC_VEHICLE_LM)
    {
        return (agc_bailout(ALARM_TWO_USERS)); /* BAILOUT1 01210   */
    }

    return (agc_poodoo(ALARM_TWO_USERS)); /* POODOO 01210     */
}

/**********************************************************************
 * STALL / IMUSTALL / OPTSTALL / AOTSTALL / RADSTALL
 * (IMU_MODE_SWITCHING_ROUTINES.agc pp.1440-1441)
 *
 * +0 operation incomplete: MODECADR = caller, JOBSLEEP (SLEEPING);
 * -1 complete and good: MODECADR = +0, return L+3 (GOOD);
 * -0 complete but failed: MODECADR = +0, return L+2 (BAD);
 * CADR: a second user -> MODABORT.
 *
 * ARGUMENTS:
 *   dev      agc_imu_device_t          I    device index
 *   wake_as  agc_job_id_t              I    job to wake when the
 *                                           operation ends
 *   result   agc_imu_stall_result_t *  O    SLEEPING, GOOD or BAD
 *
 * RETURNS: agc_status_t -- OK, or MODABORT status
 **********************************************************************/
agc_status_t agc_imu_stall_dev(agc_imu_device_t dev, agc_job_id_t wake_as, agc_imu_stall_result_t *result)
{
    agc_inhint();
    modecadr_word_t w = mc_get((int) dev); /* INDEX A, CCS MODECADR */

    if (w.state == MC_STALLED)
    {
        agc_relint();

        return (modabort(dev));
    }

    if (w.state == MC_FREE)
    { /* MODESLP          */
        mc_set((int) dev, MC_STALLED, wake_as);
        outcome[dev] = AGC_IMU_STALL_SLEEPING;
        *result      = AGC_IMU_STALL_SLEEPING;
        agc_relint();
        agc_exec_job_sleep(wake_as);
    }
    else
    {
        *result      = (w.state == MC_DONE_GOOD) ? AGC_IMU_STALL_GOOD : AGC_IMU_STALL_BAD; /* -1 / -0 */
        outcome[dev] = *result;
        mc_set((int) dev, MC_FREE, AGC_JOB_NONE);                                          /* MG2: reset to +0 */
        imu_modeexit();
    }

    return (AGC_STATUS_OK);
} /* agc_imu_stall_dev */

/**********************************************************************
 * IMUSTALL (IMU_MODE_SWITCHING_ROUTINES.agc p.1441) -- STALL on the IMU
 *
 * ARGUMENTS:
 *   wake_as  agc_job_id_t              I    job to wake when the
 *                                           operation ends
 *   result   agc_imu_stall_result_t *  O    SLEEPING, GOOD or BAD
 *
 * RETURNS: agc_status_t -- OK, or MODABORT status; AGC_STATUS_BAD_ARG on
 *          a failed precondition
 **********************************************************************/
agc_status_t agc_imu_stall(agc_job_id_t wake_as, agc_imu_stall_result_t *result)
{
    agc_status_t st = agc_imu_stall_dev(AGC_IMU_DEV_IMU, wake_as, result); /* status of the call */

    return (st);
}

/**********************************************************************
 * AGC: none (BUF2 L+2 / L+3) -- the outcome seen by the woken job
 *
 * ARGUMENTS:
 *   dev  agc_imu_device_t  I    device index
 *
 * RETURNS: agc_imu_stall_result_t -- GOOD or BAD (SLEEPING before the
 *          end)
 **********************************************************************/
agc_imu_stall_result_t agc_imu_stall_outcome(agc_imu_device_t dev)
{
    return (outcome[dev]);
}

/* ======================================================================= */
/* R02 IMU status check (p.1443-1444)                                      */
/* ======================================================================= */

/**********************************************************************
 * R02BOTH (IMU_MODE_SWITCHING_ROUTINES.agc p.1443) -- IMU status check
 *
 * REFSMMAT known: R02ZERO.  Otherwise ALARM 00220 (REFSMMAT unknown) or
 * 00210 (IMU not operating) by VARALARM, then GOTOPOOH.
 *
 * RETURNS: agc_status_t -- R02ZERO status, or POODOO after GOTOPOOH;
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_imu_status_check(void)
{
    if (agc_flag_test(AGC_FLAG_REFSMFLG))
    {
        return (agc_imu_set_in_use());
    }

    agc_alarm_code_t code = ALARM_REFSMMAT; /* OCT220          */

    if ((agc_erase_imodes30() & IM30_NOTOPER) != 0u)
    {
        code = ALARM_IMU_NOT_OPER; /* 220 - BIT4      */
    }

    if (!AGC_ASSERT(code == 00220 || code == 00210))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_exec_varalarm(code);
    const agc_status_t gs = agc_exec_gotopooh(); /* TC GOTOPOOH */

    return ((gs != AGC_STATUS_OK) ? gs : AGC_STATUS_POODOO); /* P00 */
}

/**********************************************************************
 * R02ZERO (IMU_MODE_SWITCHING_ROUTINES.agc p.1444) -- UPFLAG IMUSE
 *
 * RETURNS: agc_status_t -- OK; AGC_STATUS_BAD_ARG on a failed
 *          precondition
 **********************************************************************/
agc_status_t agc_imu_set_in_use(void)
{
    agc_flag_set(AGC_FLAG_IMUSE);

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_IMUSE)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* ======================================================================= */
/* P06 standby and time recovery (pp.1445-1448 CM, pp.1334-1337 LM)        */
/* ======================================================================= */

#define STANDBY_CHECKLIST  062u        /* OCT62 (= .5SEC): GOPERF1 R1 code       */
#define STANDBY_PHASE      07024u      /* PHASCHNG type C, group 4, job + prio   */
#define STANDBY_PRIO       020         /* OCT 20000: POSTAND at PRIO20, FINDVAC  */
#define SCALER_5MS         020u        /* BIT5 of LOSCALAR: 5 ms (16/3200 s)     */
#define SCALER_LOW5        037u        /* LOW5: scaler bits below 10 ms          */
#define SCALER_MOD         0x10000000u /* 2^28: HISCALAR,LOSCALAR wrap      */
#define SCALER_TO_CS_SHIFT 5           /* SHORTMP by BIT10 = 2^-5: scaler to cs  */
#define SCALER_TRIES       4           /* bound on the SCALPREP re-reads         */
#define SCALER_POSMAX      037777u     /* POSMAX: LOSCALAR about to carry        */
#define FINETIME_TRIES     3           /* READ, re-READ, and the POSMAX retry   */

static uint32_t time2sav; /* TIME2SAV: TIME2,TIME1 at PRESTAND, cs B28     */
static uint32_t scalsave; /* SCALSAVE: scaler at PRESTAND, 1/3200 s B28    */

/**********************************************************************
 * FINETIME (IMU_CALIBRATION_AND_ALIGNMENT.agc C p.431 / IMU_PERFORMANCE_
 * TEST_2.agc L p.381): LOSCALAR read twice so a carry between the reads is
 * seen; if it is POSMAX (about to carry into HISCALAR) read again.  Returns
 * with interrupts inhibited, like the AGC.  *t = HISCALAR, LOSCALAR as a DP
 * count of the scalar (1/1600 s per LOSCALAR count, scale as channel 3/4).
 * BUSY when LOSCALAR still reads POSMAX: the caller tries again later.
 *
 * ARGUMENTS:
 *   t  agc_dp_t *  O   HISCALAR, LOSCALAR as one DP count
 *
 * RETURNS: AGC_STATUS_OK, BUSY while LOSCALAR reads POSMAX
 **********************************************************************/
agc_status_t agc_imu_finetime(agc_dp_t *t)
{
    agc_inhint();
    uint16_t lo = SCALER_POSMAX; /* LOSCALAR, scaler counts */

    for (int k = 0; k < FINETIME_TRIES && lo == SCALER_POSMAX; k++)
    {
        lo             = (uint16_t) (agc_chan_read((int) AGC_CHAN_LOSCALAR) & SCALER_POSMAX);
        uint16_t again = (uint16_t) (agc_chan_read((int) AGC_CHAN_LOSCALAR) & SCALER_POSMAX); /* second read: carry seen? */

        if (again != lo)
        { /* RXOR, BZF               */
            lo = again;
        }
    }

    uint16_t hi = (uint16_t) (agc_chan_read((int) AGC_CHAN_HISCALAR) & SCALER_POSMAX); /* HISCALAR, scaler counts x 2^14 */

    *t = agc_dp_pack((agc_sp_t) hi, (agc_sp_t) lo);

    return ((lo == SCALER_POSMAX) ? AGC_STATUS_BUSY : AGC_STATUS_OK); /* POSMAX: about to carry */
}

/**********************************************************************
 * SCALPREP (IMU_MODE_SWITCHING_ROUTINES.agc p.1447 CM / p.1336 LM) --
 * snapshot the scaler and check it against TIME1
 *
 * FINETIME +1 reads HISCALAR,LOSCALAR (1/3200 s per count).  The 10 ms
 * bit (bit 6) of the scaler counts 5 ms out of phase with TIME1, so 5 ms
 * (BIT5) is added and the low 5 bits are dropped.  If those bits are then
 * zero, the reading fell in the first 5 ms after a TIME1 increment and
 * does not agree with the TIME2,TIME1 reading (the AGC returns to L+1 to
 * read both again).  Ends with RELINT: the caller's INHINT and FINETIME's
 * are both released here.
 *
 * ARGUMENTS:
 *   scaler  uint32_t *  O  scaler + 5 ms, low 5 bits zeroed, units
 *                          1/3200 s, modulo 2^28 (MPAC,MPAC+1)
 *
 * RETURNS: AGC_STATUS_OK when the reading agrees with TIME1 (L+2),
 *          AGC_STATUS_BUSY when it must be read again (L+1);
 *          AGC_STATUS_BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t scalprep(uint32_t *scaler)
{
    agc_dp_t raw = 0; /* HISCALAR,LOSCALAR, 1/3200 s B28 */

    if (!AGC_ASSERT(scaler != NULL) || !AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_imu_finetime(&raw); /* FINETIME +1 */

    agc_relint(); /* RELINT (FINETIME's INHINT)                   */
    agc_relint(); /* RELINT (the caller's INHINT)                 */

    if (!AGC_ASSERT(raw >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* DAS MPAC with (0, BIT5): a carry out of HISCALAR is lost.        */
    uint32_t sum = ((uint32_t) raw + SCALER_5MS) % SCALER_MOD; /* +5 ms */

    *scaler = sum & ~SCALER_LOW5; /* CS LOW5, MASK MPAC +1      */

    if ((st != AGC_STATUS_OK) || ((sum & SCALER_LOW5) == 0u))
    {
        return (AGC_STATUS_BUSY); /* CCS A = +0: first interval */
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * P06 / PRESTAND (IMU_MODE_SWITCHING_ROUTINES.agc pp.1445-1446 CM,
 * pp.1334-1335 LM) -- GNCS from operate to standby
 *
 * Sets NODOFLAG (no V37), snapshots TIME2,TIME1 and the scaler until the
 * two agree, knocks down REFSMFLG/DRIFTFLG/TRACKFLG (RNDREFDR), IMUSE and
 * RNDVZFLG, sets the standby enable bit (channel 13 bit 11), makes
 * POSTAND the group 4 restart point, then asks the crew for standby
 * (V50N25 R1 = 00062) until the computer goes down.
 * DEVIATION: the AGC retries PRESTAND without limit.  The scaler does not
 * move while a C job runs, so after SCALER_TRIES the job yields and runs
 * P06 again (setting NODOFLAG twice is harmless).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P06 job (AGC_JOB_STANDBY)
 *
 * RETURNS: agc_status_t from the GOPERF1 request, or the yield status;
 *          AGC_STATUS_BAD_ARG on a failed precondition; a BUSY from
 *          SCALPREP becomes a yield, never the result
 **********************************************************************/
agc_status_t agc_job_standby(agc_job_ctx_t *ctx)
{
    uint32_t     scaler = 0;               /* scaler snapshot, 1/3200 s */
    agc_status_t st     = AGC_STATUS_BUSY; /* SCALPREP outcome          */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ctx->id == AGC_JOB_STANDBY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set(AGC_FLAG_NODOFLAG); /* UPFLAG NODOFLAG (no V37) */

    for (int k = 0; (k < SCALER_TRIES) && (st == AGC_STATUS_BUSY); k++)
    {
        agc_inhint();                 /* PRESTAND INHINT          */
        time2sav = agc_time_now_cs(); /* DCA TIME2, DXCH TIME2SAV */
        st       = scalprep(&scaler);
    }

    if (st == AGC_STATUS_BUSY)
    {
        return (agc_exec_job_yield(AGC_JOB_STANDBY)); /* TC PRESTAND */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    scalsave = scaler;                                  /* DXCH MPAC, DXCH SCALSAVE        */
    agc_imu_rndrefdr();
    agc_flag_clear(AGC_FLAG_IMUSE);
    agc_flag_clear(AGC_FLAG_RNDVZFLG);
    agc_chan_or(AGC_CHAN_CHAN13, AGC_CH013_STANDBYENB); /* WOR BIT11 */

    const agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_STANDBY_RECOVERY, AGC_TASK_NONE, STANDBY_PRIO, false, 0U};

    agc_phase_change_var(STANDBY_PHASE, &v); /* 2CADR POSTAND        */

    return (agc_job_standby_request(ctx));
}

/**********************************************************************
 * P06 (IMU_MODE_SWITCHING_ROUTINES.agc p.1446 CM / p.1335 LM) -- the
 * GOPERF1 loop: V50N25 with R1 = 00062 ("go to standby"); every answer
 * (V34, V33, ENTER) puts the request up again (TCF -3, -4, -5).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P06 job
 *
 * RETURNS: agc_status_t from agc_dsky_display; AGC_STATUS_BAD_ARG on a
 *          failed precondition
 **********************************************************************/
agc_status_t agc_job_standby_request(agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t again = {AGC_JOB_STANDBY_REQUEST, /* V34   */
                                   AGC_JOB_STANDBY_REQUEST, /* V33   */
                                   AGC_JOB_STANDBY_REQUEST}; /* ENTER */

    if (!AGC_ASSERT(agc_flag_test(AGC_FLAG_NODOFLAG)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_GOPERF1, STANDBY_CHECKLIST, &again));
}

/**********************************************************************
 * POSTAND / POSTCOM (IMU_MODE_SWITCHING_ROUTINES.agc pp.1447-1448 CM,
 * pp.1336-1337 LM) -- recover TIME2,TIME1 after standby
 *
 * Clears the standby enable bit and TIME2,TIME1, snapshots the scaler
 * (SCALPREP, retried until it agrees with the cleared clock), and adds
 * the scaler advance since PRESTAND plus TIME2SAV into TIME2,TIME1.  The
 * scaler difference (SHORTMP by BIT10 = 2^-5: 1/3200 s to cs) is made
 * positive by adding BIT10 to its high word, i.e. 2^23 cs, when the
 * scaler wrapped during standby; the modulo-2^28 difference here is the
 * same number.  Then NODOFLAG down and GOTOPOOH.
 * DEVIATION: bounded retries, then the job yields and runs POSTAND again
 * (the AGC's TC POSTAND +3 clears TIME2,TIME1 again the same way).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the restart job (group 4, PRIO20)
 *
 * RETURNS: agc_status_t from GOTOPOOH, or the yield status;
 *          AGC_STATUS_BAD_ARG on a failed precondition; a BUSY from
 *          SCALPREP becomes a yield, never the result
 **********************************************************************/
agc_status_t agc_job_standby_recovery(agc_job_ctx_t *ctx)
{
    uint32_t     scaler = 0;               /* scaler snapshot, 1/3200 s */
    agc_status_t st     = AGC_STATUS_BUSY; /* SCALPREP outcome          */

    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(scalsave < SCALER_MOD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    chan_clear(AGC_CHAN_CHAN13, AGC_CH013_STANDBYENB); /* WAND -BIT11 */

    for (int k = 0; (k < SCALER_TRIES) && (st == AGC_STATUS_BUSY); k++)
    {
        agc_inhint();       /* POSTAND +3 INHINT                */
        agc_time_set_cs(0); /* DXCH TIME2 with +0,+0            */
        st = scalprep(&scaler);
    }

    if (st == AGC_STATUS_BUSY)
    {
        return (agc_exec_job_yield(AGC_JOB_STANDBY_RECOVERY));
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* DCS SCALSAVE, DAS MPAC, SHORTMP BIT10; + BIT10 if negative.      */
    uint32_t dt_cs = ((scaler - scalsave) % SCALER_MOD) >> SCALER_TO_CS_SHIFT; /* scaler advance since PRESTAND, cs B28 */
    uint32_t now   = agc_time_now_cs();                                        /* counted since the clear, cs */

    if (!AGC_ASSERT(dt_cs < (SCALER_MOD >> SCALER_TO_CS_SHIFT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* POSTCOM: DCA TIME2SAV, DAS MPAC, DAS TIME2.                      */
    agc_time_set_cs((now + dt_cs + time2sav) % AGC_TIME_MODULUS_CS);
    agc_flag_clear(AGC_FLAG_NODOFLAG);

    return (agc_exec_gotopooh());
} /* agc_job_standby_recovery */
