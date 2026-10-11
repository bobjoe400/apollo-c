/**********************************************************************
 * FILE NAME: prelaunch_align.c
 *
 * PURPOSE:
 *   Comanche prelaunch alignment, part 1: P01 prelaunch initialization
 *   (GTSCPSS .. SETUPGC), the IMU positioning shared with the IMU
 *   performance test (GEOIMUTT, IMUBACK, NBPOSPL, COAALIGN, IMUSTLLG),
 *   the earth-rate compensation (ERTHRVSE, EARTHR, EARTHR*, OVERFFIX),
 *   POSN17C, the error exits (SOMERR2, ENDTEST1) and the P07 entry of
 *   the IMU performance test (IMUTEST, GUESS, LATAZCHK).
 *
 * AGC SOURCE:
 *   Comanche055 IMU_CALIBRATION_AND_ALIGNMENT.agc pp. 423-431, 442-443,
 *   452 (CM only; Luminary's P07 tests are sys-misc's)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   XSM..ZSM      agc_mat_t  I/O  half-unit      agc_align_ram()->xsm
 *   XNB..ZNB      agc_mat_t  O    half-unit      agc_align_ram()->xnb
 *   ERCOMP1       agc_dp_t   I/O  rev 2^0        gyro buffer ERCOMP
 *   AINLA overlay struct     I/O  see header     prelaunch_ram()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_imu_zero/coarse/  IMUZERO, IMUCOARS, IMUFINE, IMUSTALL (imu)
 *   fine/stall
 *   agc_imu_pulse_buf     IMUPULSE on the ERCOMP buffer (PULSEIMU)
 *   agc_imu_calcga        CALCGA (gimbal angles for the compass)
 *   agc_dsky_display      CLEANDSP, GOFLASH V06N41 (LATAZCHK)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   POODOO 01521 (P01 after P11, NOP01FLG set); ALARM 01601 + ENDTEST1
 *   when an IMU mode switch or torquing ends bad (SOMERR2); CALCGA's
 *   00401 (gimbal lock, IMU test then runs the PIPA test).
 *   AGC_STATUS_UNIMPLEMENTED from the IMU performance test's PIPA test
 *   (PIPACHK), see NOTES.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only.  Runs as jobs; every IMU stall is a separate job entry.
 *
 * NOTES:
 *   The IMU performance test drift computations (ALCGKK .. ONCEMORE,
 *   PIPACHK, TORQUE, VERTDRFT, SHOW, SETUPER1, TORQINCH) end in TC
 *   LAT(SPL) (p.441), an erasable-memory program not in the rope; they
 *   are stubbed (AGC_STATUS_UNIMPLEMENTED).  IMUTEST, GUESS, LATAZCHK
 *   and the positioning are translated.
 *   DEVIATION: LATITUDE, AZIMUTH and LAUNCHAZ are also kept in the P11
 *   pad load (src/csm/burns); see docs/modules/review-imu.md.
 *
 * ALGORITHM:
 *   P01: P11 already done -> 1521.  Else GEOCOMP1 = 1, 1/PIPADT = 50 cs,
 *   LENGTHOT = 128, 1SECXT1 = .5 s; azimuth = LAUNCHAZ; XSM = POSN17C;
 *   IMU zero, coarse align to CALCGA(XNB, XSM), fine align, then after
 *   PERFDLAY wake ESTIMS (prelaunch_gyrocompass.c).
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_csm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "prelaunch_internal.h"

#define OMEG_MS        ((agc_dp_t) 65334634) /* OMEG/MS 2DEC .24339048 (07623 26552) */
#define TIME_WRAP      ((agc_dp_t) 0x10000000L) /* DPPOSMAX + ONEDPP: 2^28 cs */
#define ALARM_NO_P01   ((agc_alarm_code_t) 01521) /* P01 after P11       */
#define COMPASS_PIPADT ((agc_sp_t) 06200) /* 1/PIPAGT: 50 cs, cs B8     */
#define TEST_PIPADT    ((agc_sp_t) 014400) /* OC14400: 100 cs, cs B8    */
#define COMPASS_PASSES ((agc_sp_t) 0200)  /* BIT8: 128 erection cycles  */
#define TEST_PASSES    ((agc_sp_t) 01602) /* TESTTIME: 898 passes       */
#define COMPASS_PERIOD ((agc_sp_t) 062)   /* 1/2SECX = .5SEC: 50 cs     */
#define TEST_PERIOD    ((agc_sp_t) 0144)  /* 1SECX = 1SEC: 100 cs       */
#define COMPASS_DELAY  ((agc_dp_t) 1)     /* PERFDLAY = 0,1: 1 cs B28   */
#define MODREG_NONE    ((agc_sp_t) -1)    /* -0 in MODREG (exec's code) */
#define VN_SITE        ((uint16_t) ((6 << 7) | 41)) /* VNG0641: V06N41 */
#define TEMPTIME_SHIFT 9                  /* ERTHR: SL 9D               */

static agc_vec_t       ervector; /* ERVECTOR: earth rate, rev/cs x 2^-19 B0  */
static agc_dp_t        tmark; /* TMARK: time of the last EARTHR, cs 2^28  */
static prelaunch_ram_t pl; /* AINLA overlay and pad words (see header) */

/**********************************************************************
 * AGC: none (accessor) -- the AINLA overlay of the prelaunch programs
 *
 * RETURNS: prelaunch_ram_t * -- the module's erasables, never NULL
 **********************************************************************/
prelaunch_ram_t *prelaunch_ram(void)
{
    if (!AGC_ASSERT(pl.lengthot >= 0 && pl.lengthot <= AGC_SP_MAX))
    {
        pl.lengthot = 0; /* recovery: no passes left                  */
    }

    if (!AGC_ASSERT(pl.premtrxc >= 0 && pl.premtrxc <= AGC_SP_MAX))
    {
        pl.premtrxc = 1; /* recovery: no azimuth slew pending         */
    }

    return (&pl);
}

/**********************************************************************
 * AGC: none (pad erasable load) -- LATITUDE, AZIMUTH, LAUNCHAZ
 *
 * The compass needs an erasable load (p.432): LAUNCHAZ (azimuth of X
 * SM from north), LATITUDE of the pad, AZIMUTH of ZNB.
 *
 * ARGUMENTS:
 *   latitude  agc_dp_t  I  LATITUDE, rev 2^0, |latitude| <= 1/4
 *   azimuth   agc_dp_t  I  AZIMUTH, rev 2^0
 *   launchaz  agc_dp_t  I  LAUNCHAZ, rev 2^0
 *
 * RETURNS: AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a latitude beyond
 *          the pole
 **********************************************************************/
agc_status_t agc_csm_prelaunch_set_pad(agc_dp_t latitude, agc_dp_t azimuth,
                                       agc_dp_t launchaz)
{
    if (!AGC_ASSERT(latitude >= -AGC_DP_QUARTER &&
                    latitude <= AGC_DP_QUARTER))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(azimuth >= AGC_DP_MIN && launchaz >= AGC_DP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pl.latitude = latitude;
    pl.azimuth  = azimuth;
    pl.launchaz = launchaz;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (accessors for tests and P11) -- ERVECTOR and THETAN1
 *
 * RETURNS: agc_vec_t -- ERVECTOR (rev/cs x 2^-19 B0)
 **********************************************************************/
agc_vec_t agc_csm_prelaunch_ervector(void)
{
    if (!AGC_ASSERT(tmark >= 0))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    if (!AGC_ASSERT(ervector.c[2] == 0))
    {
        return (agc_vec_zero()); /* no valid result */
    }

    return (ervector);
}

/**********************************************************************
 * AGC: none (accessor) -- THETAN1, the compass commands of this cycle
 *
 * RETURNS: agc_vec_t -- THETAN1, rev 2^0
 **********************************************************************/
agc_vec_t agc_csm_prelaunch_thetan(void)
{
    if (!AGC_ASSERT(pl.filt.thetan1.c[0] >= AGC_DP_MIN) ||
        !AGC_ASSERT(pl.filt.thetan1.c[2] >= AGC_DP_MIN))
    {
        return (agc_vec_zero()); /* no valid result                   */
    }

    return (pl.filt.thetan1);
}

/**********************************************************************
 * AGC: none (accessors) -- TMARK for the gyrocompass file
 *
 * ARGUMENTS:
 *   t  agc_dp_t  I  new TMARK, cs 2^28
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for a time out of range
 **********************************************************************/
agc_status_t prelaunch_set_tmark(agc_dp_t t)
{
    if (!AGC_ASSERT(t >= 0) || !AGC_ASSERT(t < TIME_WRAP))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    tmark = t;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (accessor) -- TMARK
 *
 * RETURNS: agc_dp_t -- TMARK, cs 2^28
 **********************************************************************/
agc_dp_t prelaunch_tmark(void)
{
    if (!AGC_ASSERT(tmark >= 0) || !AGC_ASSERT(tmark < TIME_WRAP))
    {
        return (0); /* no valid result                               */
    }

    return (tmark);
}

/**********************************************************************
 * POSN17C (IMU_CALIBRATION_AND_ALIGNMENT.agc p.443) -- compass position
 *
 * Z down, X downrange: XSM = (0, -cos AZ, sin AZ)/2, YSM = (0, sin AZ,
 * cos AZ)/2, ZSM = (-1/2, 0, 0).  SETUPGC has zeroed the other words.
 *
 * ARGUMENTS:
 *   ip       agc_interp_t *  I/O  interpreter state
 *   azimuth  agc_dp_t        I    NEWAZ1, rev 2^0 (clockwise from north)
 *   xsm      agc_mat_t *     O    rows XSM, YSM, ZSM, half-unit
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG on a NULL argument
 **********************************************************************/
agc_status_t agc_csm_prelaunch_posn17c(agc_interp_t *ip, agc_dp_t azimuth,
                                       agc_mat_t *xsm)
{
    if (!AGC_ASSERT(ip != NULL && xsm != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(azimuth >= AGC_DP_MIN && azimuth <= AGC_DP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t s = agc_dp_sin(ip, azimuth); /* sin AZ / 2, 2^0           */
    agc_dp_t c = agc_dp_cos(ip, azimuth); /* cos AZ / 2, 2^0           */

    xsm->row[0] = agc_vec_make(0, agc_dp_neg(c), s);
    xsm->row[1] = agc_vec_make(0, s, c);
    xsm->row[2] = agc_vec_make(-AGC_DP_HALF, 0, 0); /* CS HALF      */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * POSN17C (p.443) on NEWAZ1 into the alignment XSM
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state
 *
 * RETURNS: agc_status_t from agc_csm_prelaunch_posn17c
 **********************************************************************/
agc_status_t prelaunch_posn17c(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_csm_prelaunch_posn17c(ip, pl.newazmth,
                                                &agc_align_ram()->xsm); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * ERTHRVSE (IMU_CALIBRATION_AND_ALIGNMENT.agc p.430) -- earth rate vector
 *
 * ERVECTOR = (sin LAT, -cos LAT, 0) OMEG/MS; TMARK = present time;
 * ERCOMP1 = 0.
 *
 * ARGUMENTS:
 *   ip        agc_interp_t *  I/O  interpreter state
 *   latitude  agc_dp_t        I    LATITUDE, rev 2^0
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for a latitude beyond the pole
 **********************************************************************/
agc_status_t agc_csm_prelaunch_erthrvse(agc_interp_t *ip, agc_dp_t latitude)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(latitude >= -AGC_DP_QUARTER && latitude <= AGC_DP_QUARTER))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t v = agc_vec_make(agc_dp_sin(ip, latitude),
                               agc_dp_neg(agc_dp_cos(ip, latitude)), 0); /* earth rate, rev 2^0 */

    ervector = agc_vec_scale(ip, v, OMEG_MS);
    tmark    = agc_rtb_loadtime(); /* LOADTIME          */
    const agc_dp_t zero[3] = {0, 0, 0}; /* SCHZEROS, rev 2^0      */

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, zero); /* ERCOMP1 = 0 */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * EARTHR +3 / ERTHR (IMU_CALIBRATION_AND_ALIGNMENT.agc p.430) -- earth
 * rate since TMARK into ERCOMP1, then PULSEIMU
 *
 * ERCOMP1 = XSM (ERVECTOR (TEMPTIME - TMARK) 2^9) + ERCOMP1; TMARK =
 * TEMPTIME.  OVERFFIX adds 2^28 cs when the difference is negative
 * (TIME2 wrapped).
 *
 * ARGUMENTS:
 *   ip        agc_interp_t *  I/O  interpreter state
 *   temptime  agc_dp_t        I    TEMPTIME, cs 2^28
 *   wake_as   agc_job_id_t    I    job woken after GYROBUSY, or NONE
 *
 * RETURNS: agc_status_t from IMUPULSE (BUSY: the job sleeps as wake_as)
 **********************************************************************/
static agc_status_t earthr_at(agc_interp_t *ip, agc_dp_t temptime,
                              agc_job_id_t wake_as)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t dt = agc_dp_sub(ip, temptime, tmark); /* cs 2^28        */

    if (dt < 0)
    {
        dt = dt + TIME_WRAP; /* OVERFFIX          */
    }

    if (!AGC_ASSERT(dt >= 0 && dt <= AGC_DP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_vec_t inc =
        agc_vec_scale(ip, ervector, agc_dp_shl(ip, dt, TEMPTIME_SHIFT)); /* increment, rev 2^0 */
    agc_dp_t     buf[3];           /* ERCOMP1, rev 2^0              */
    agc_status_t st = agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, buf); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    agc_vec_t er = agc_vec_add(ip, agc_mat_vec(ip, &agc_align_ram()->xsm, inc),
                               agc_vec_make(buf[0], buf[1], buf[2])); /* earth-rate commands, rev 2^0 */

    for (int i = 0; i < 3; i++)
    {
        buf[i] = er.c[i];
    }

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, buf);
    tmark = temptime;

    return (agc_imu_pulse_buf(AGC_IMU_GBUF_ERCOMP, wake_as)); /* PULSEIMU */
}

/**********************************************************************
 * P11 tail at liftoff (P11.agc pp.536-537) -- ERCOMP = (XSM THETAN)
 * VSL1 + ERCOMP, then EARTHR +3 with TEMPTIME = TLIFTOFF
 *
 * ARGUMENTS:
 *   ip        agc_interp_t *  I/O  interpreter state
 *   tliftoff  agc_dp_t        I    TLIFTOFF, cs 2^28
 *
 * RETURNS: agc_status_t from IMUPULSE (not slept on: P11 goes on)
 **********************************************************************/
agc_status_t agc_csm_prelaunch_earthr_at_liftoff(agc_interp_t *ip,
                                                 agc_dp_t      tliftoff)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(tliftoff >= 0 && tliftoff <= AGC_DP_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t     buf[3]; /* ERCOMP, rev 2^0                              */
    agc_status_t st = agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, buf); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_mat_t *xsm = &agc_align_ram()->xsm; /* XSM..ZSM, B1 */
    agc_vec_t        er  = agc_vec_add(ip, agc_vec_shl(ip, agc_mat_vec(ip, xsm,
                                                                       pl.filt.
                                                                       thetan1),
                                                       1),
                                       agc_vec_make(buf[0], buf[1], buf[2])); /* earth-rate commands, rev 2^0 */

    for (int i = 0; i < 3; i++)
    {
        buf[i] = er.c[i]; /* STODL ERCOMP         */
    }

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, buf);

    return (earthr_at(ip, tliftoff, AGC_JOB_NONE)); /* EARTHR +3      */
}

/**********************************************************************
 * IMUSTLLG / COAALIGN +8 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.428) --
 * IMUSTALL; a bad end goes to SOMERR2
 *
 * ARGUMENTS:
 *   wake_as  agc_job_id_t  I  job that continues after the stall
 *   done     bool *        O  true: the mode switch was already over
 *                             and good, continue now
 *
 * RETURNS: agc_status_t -- OK (sleeping or done), or SOMERR2's status
 **********************************************************************/
agc_status_t prelaunch_imustall(agc_job_id_t wake_as, bool *done)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING; /* STALL return */

    if (!AGC_ASSERT(done != NULL) || !AGC_ASSERT(wake_as != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *done = false;
    agc_status_t st = agc_imu_stall(wake_as, &res); /* status of the call */

    if ((st != AGC_STATUS_OK) || (res == AGC_IMU_STALL_SLEEPING))
    {
        return (st);
    }

    if (res == AGC_IMU_STALL_BAD)
    {
        return (prelaunch_somerr2()); /* TC SOMERR2    */
    }

    *done = true; /* TC QPLACE     */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * IMUSTLLG return in the woken job (p.428): L+2 bad -> SOMERR2
 *
 * ARGUMENTS:
 *   good  bool *  O  true when the stalled mode switch ended good
 *
 * RETURNS: agc_status_t -- OK when good, else SOMERR2's status
 **********************************************************************/
agc_status_t prelaunch_stall_outcome(bool *good)
{
    if (!AGC_ASSERT(good != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *good = (agc_imu_stall_outcome(AGC_IMU_DEV_IMU) == AGC_IMU_STALL_GOOD);

    if (!*good)
    {
        return (prelaunch_somerr2());
    }

    if (!AGC_ASSERT(*good))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * EARTHR* / PROUT (IMU_CALIBRATION_AND_ALIGNMENT.agc p.430) -- EARTHR
 * at the present time, then IMUSTLLG
 *
 * ARGUMENTS:
 *   ip          agc_interp_t *  I/O  interpreter state
 *   busy_wake   agc_job_id_t    I    job run again when the gyros are
 *                                    busy (GYROBUSY sleeps the caller)
 *   stall_wake  agc_job_id_t    I    job woken when the torquing ends
 *   done        bool *          O    true: torquing over, continue now
 *
 * RETURNS: agc_status_t -- OK (sleeping or done), or SOMERR2's status
 **********************************************************************/
agc_status_t prelaunch_earthr_star(agc_interp_t *ip, agc_job_id_t busy_wake,
                                   agc_job_id_t stall_wake, bool *done)
{
    if (!AGC_ASSERT(ip != NULL && done != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(busy_wake != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *done = false;
    agc_status_t st = earthr_at(ip, agc_rtb_loadtime(), busy_wake); /* EARTHR */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* GYROBUSY: asleep as busy_wake      */
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_ALARM))
    {
        return (st); /* ALARM: caged, IMUCADR -0, the stall sees it  */
    }

    return (prelaunch_imustall(stall_wake, done)); /* PROUT: IMUSTLLG */
}

/**********************************************************************
 * ENDTEST1 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.427) -- end of the test
 * or compass: IMUSE down, MODREG -0, MKRELEAS, ENDEXT
 *
 * RETURNS: agc_status_t from ENDEXT
 **********************************************************************/
agc_status_t prelaunch_endtest1(void)
{
    agc_flag_clear(AGC_FLAG_IMUSE); /* DOWNFLAG IMUSE            */
    agc_status_t st = agc_phase_newmode(MODREG_NONE); /* NEWMODEX +3 */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    st = agc_sxtmark_mkreleas(); /* MKRELEAS                          */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (agc_dsky_endmark());
}

/**********************************************************************
 * SOMERR2 (IMU_CALIBRATION_AND_ALIGNMENT.agc p.442) -- bad IMU torque:
 * ALARM 1601, group 5 off (PHASCHNG 00005), ENDTEST1
 *
 * RETURNS: agc_status_t from ENDTEST1
 **********************************************************************/
agc_status_t prelaunch_somerr2(void)
{
    _Static_assert(PRELAUNCH_ALARM_TORQUE == 01601, "AGC constant");

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_alarm(PRELAUNCH_ALARM_TORQUE);
    agc_phase_change(5, 00005); /* group 5 inactive                     */

    return (prelaunch_endtest1());
}

/**********************************************************************
 * NBPOSPL (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.423-424) -- navigation
 * base in the compass position from AZIMUTH
 *
 * XNB = (1/2, 0, 0), YNB = (0, sin AZ, cos AZ)/2, ZNB = (0, -cos AZ,
 * sin AZ)/2 (ZEROING clears the 18 words first).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG on a NULL argument
 **********************************************************************/
agc_status_t prelaunch_nbpospl(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_mat_t *xnb = &agc_align_ram()->xnb; /* XNB..ZNB, B1 */
    agc_dp_t   s   = agc_dp_sin(ip, pl.azimuth); /* sin AZ / 2, 2^0   */
    agc_dp_t   c   = agc_dp_cos(ip, pl.azimuth); /* cos AZ / 2, 2^0   */

    xnb->row[0] = agc_vec_make(AGC_DP_HALF, 0, 0); /* CA HALF, TS XNB  */
    xnb->row[1] = agc_vec_make(0, s, c);
    xnb->row[2] = agc_vec_make(0, agc_dp_neg(c), s);

    if (!AGC_ASSERT(xnb->row[0].c[0] == AGC_DP_HALF))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GEOIMUTT (IMU_CALIBRATION_AND_ALIGNMENT.agc p.423) -- IMUZERO, then
 * IMUSTLLG (continues as AGC_JOB_PRELAUNCH_IMU_ZEROED)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P01 or P07 job
 *
 * RETURNS: agc_status_t of the stall or of the next step
 **********************************************************************/
static agc_status_t geoimutt(agc_job_ctx_t *ctx)
{
    bool done = false; /* IMUSTALL found the zero already over */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_imu_zero(); /* IMUZERO                       */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY))
    {
        return (st);
    }

    st = prelaunch_imustall(AGC_JOB_PRELAUNCH_IMU_ZEROED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return (st);
    }

    return (agc_job_prelaunch_imu_zeroed(ctx));
}

/**********************************************************************
 * P01 / GTSCPSS / GTSCPSSA / NXXTENN / SETUPGC (IMU_CALIBRATION_AND_
 * ALIGNMENT.agc pp.442-443) -- prelaunch initialization (V37 01)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the P01 job
 *
 * RETURNS: agc_status_t -- POODOO status when P11 has run (NOP01FLG),
 *          else that of GEOIMUTT
 **********************************************************************/
agc_status_t agc_job_prelaunch_initialization(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state for POSN17C        */

    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_flag_test(AGC_FLAG_NO_PRELAUNCH_INITIALIZATION))
    {
        return (agc_poodoo(ALARM_NO_P01)); /* P11 already done      */
    }

    pl.geocomp1 = 1; /* GTSCPSSA: lead-in for compass             */
    agc_imu_set_pipa_interval(COMPASS_PIPADT); /* 1/PIPADT = 1/PIPAGT */
    pl.lengthot  = COMPASS_PASSES; /* NXXTENN: CA BIT8              */
    pl.onesecxt1 = COMPASS_PERIOD; /* compass is a .5 s loop        */
    pl.premtrxc  = 1;
    pl.perfdlay  = COMPASS_DELAY;
    pl.newazmth  = pl.launchaz; /* DCA LUNCHAZ1, DXCH NEWAZ1         */
    pl.oldazmth  = pl.launchaz; /* DCA LUNCHAZ1, DXCH OLDAZMTH       */

    for (int r = 0; r < 3; r++)
    {
        agc_align_ram()->xsm.row[r] = agc_vec_zero(); /* SETUPGC: ZEROING */
    }

    agc_interp_init(&ip);
    agc_status_t st = prelaunch_posn17c(&ip); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (geoimutt(ctx)); /* GO TO IMU2 FOR FURTHER INITIALIZATION */
}

/**********************************************************************
 * IMUBACK / NBPOSPL (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.423-424) --
 * after IMUZERO: navigation base, CALCGA, IMUCOARS, gimbal-lock note
 *
 * In P03 (CHECKMM 03) NBPOSPL returns to SETNBPOS +1 (MKRELEAS, OPTDATA).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the woken job (AGC_JOB_PRELAUNCH_IMU_ZEROED)
 *
 * RETURNS: agc_status_t of the stall, of SOMERR2 or of the next step
 **********************************************************************/
agc_status_t agc_job_prelaunch_imu_zeroed(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;          /* interpreter state                    */
    uint16_t     thetad[3];   /* THETAD: CALCGA gimbal angles, CDU    */
    bool         ok = false;  /* stall outcome / IMUSTALL done        */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = prelaunch_stall_outcome(&ok); /* status of the call */

    if (!ok)
    {
        return (st);
    }

    pl.ndxctr = 0; /* IMUBACK                                          */
    agc_interp_init(&ip);
    st = prelaunch_nbpospl(&ip);

    if (agc_phase_checkmm(PRELAUNCH_MM_VERIFY))
    {
        return (prelaunch_verify_start(ctx)); /* TCF SETNBPOS +1      */
    }

    st = agc_imu_calcga(&ip, &agc_align_ram()->xnb, &agc_align_ram()->xsm,
                        thetad);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    for (int i = 0; i < 3; i++)
    {
        thetad[i] = agc_erase_thetad(i); /* IMUCOARS reads THETAD      */
    }

    st = agc_imu_coarse(thetad); /* IMUCOARS                           */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY))
    {
        return (st);
    }

    if (agc_flag_test(AGC_FLAG_GLOKFAIL))
    {
        pl.ndxctr = 1; /* +1 IF IN GIMBAL LOCK                         */
    }

    agc_flag_clear(AGC_FLAG_GLOKFAIL); /* DOWNFLAG GLOKFAIL            */
    st = prelaunch_imustall(AGC_JOB_PRELAUNCH_IMU_COARSE_ALIGNED, &ok);

    if ((st != AGC_STATUS_OK) || !ok)
    {
        return (st);
    }

    return (agc_job_prelaunch_imu_coarse_aligned(ctx));
} /* agc_job_prelaunch_imu_zeroed */

/**********************************************************************
 * PIPACHK (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.425-426) -- PIPA test of
 * the IMU performance test (taken when the coarse align ran into gimbal
 * lock).  Stub: the test's display and drift loop are not translated
 * (see the file prolog).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the positioning job
 *
 * RETURNS: AGC_STATUS_UNIMPLEMENTED
 **********************************************************************/
static agc_status_t pipachk(const agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(pl.ndxctr > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_UNIMPLEMENTED);
}

/**********************************************************************
 * IMUBACK after IMUCOARS (IMU_CALIBRATION_AND_ALIGNMENT.agc p.424) --
 * PIPA test if in gimbal lock, else IMUFINE and IMUSTLLG
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the woken job
 *
 * RETURNS: agc_status_t of the stall, of SOMERR2 or of the next step
 **********************************************************************/
agc_status_t agc_job_prelaunch_imu_coarse_aligned(agc_job_ctx_t *ctx)
{
    bool ok = false; /* stall outcome / IMUSTALL done             */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = prelaunch_stall_outcome(&ok); /* status of the call */

    if (!ok)
    {
        return (st);
    }

    if (pl.ndxctr > 0)
    {
        return (pipachk(ctx)); /* CCS NDXCTR: TC PIPACHK              */
    }

    st = agc_imu_fine(); /* IMUFINE                                    */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY))
    {
        return (st);
    }

    st = prelaunch_imustall(AGC_JOB_PRELAUNCH_IMU_FINE_ALIGNED, &ok);

    if ((st != AGC_STATUS_OK) || !ok)
    {
        return (st);
    }

    return (agc_job_prelaunch_imu_fine_aligned(ctx));
}

/**********************************************************************
 * IMUBACK after IMUFINE (IMU_CALIBRATION_AND_ALIGNMENT.agc p.424) --
 * LONGCALL GOESTIMS after PERFDLAY, JOBSLEEP until it wakes ESTIMS
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the woken job
 *
 * RETURNS: agc_status_t of LONGCALL, or SOMERR2's
 **********************************************************************/
agc_status_t agc_job_prelaunch_imu_fine_aligned(agc_job_ctx_t *ctx)
{
    bool ok = false; /* stall outcome                             */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = prelaunch_stall_outcome(&ok); /* status of the call */

    if (!ok)
    {
        return (st);
    }

    if (!AGC_ASSERT(pl.perfdlay > 0 && pl.perfdlay < TIME_WRAP))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    st = agc_waitlist_longcall((uint32_t) pl.perfdlay,
                               AGC_TASK_GYROCOMPASSING_WAKE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_exec_job_sleep(AGC_JOB_GYROCOMPASSING_START); /* CA ESTICADR  */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * GOESTIMS (IMU_CALIBRATION_AND_ALIGNMENT.agc p.424) -- JOBWAKE ESTIMS
 *
 * RETURNS: agc_status_t of JOBWAKE (TASKOVER)
 **********************************************************************/
agc_status_t agc_task_gyrocompassing_wake(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_exec_job_wake(AGC_JOB_GYROCOMPASSING_START); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_NOT_FOUND))
    {
        return (st);
    }

    return (AGC_STATUS_OK); /* TASKOVER                             */
}

/**********************************************************************
 * IMUTEST / COAALIGN (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.423, 428) --
 * IMU performance test entry (P07): zero the drift words, coarse align
 * to zero gimbal angles, IMUSTALL
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the systems-test job (MM 07)
 *
 * RETURNS: agc_status_t of the stall or of the next step
 **********************************************************************/
agc_status_t agc_csm_prelaunch_imutest(agc_job_ctx_t *ctx)
{
    const uint16_t zero[3] = {0, 0, 0}; /* THETAD = 0, CDU         */
    bool           done    = false;     /* IMUSTALL already over   */

    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pl.geocomp1 = 0; /* DRIFTT, GEOCOMP1 = 0                          */
    pl.lengthot = TEST_PASSES; /* CAF TESTTIME                       */
    agc_status_t st = agc_imu_coarse(zero); /* COAALIGN: IMUCOARS     */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY))
    {
        return (st);
    }

    st = prelaunch_imustall(AGC_JOB_IMU_TEST_COARSE_ALIGNED, &done);

    if ((st != AGC_STATUS_OK) || !done)
    {
        return (st);
    }

    return (agc_job_imu_test_coarse_aligned(ctx));
}

/**********************************************************************
 * IMUTEST after COAALIGN (IMU_CALIBRATION_AND_ALIGNMENT.agc p.423) --
 * 1SECXT1 = 1 s, 1/PIPADT = 100 cs, then GUESS (LATAZCHK)
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the woken job
 *
 * RETURNS: agc_status_t of the LATAZCHK display, or SOMERR2's
 **********************************************************************/
agc_status_t agc_job_imu_test_coarse_aligned(agc_job_ctx_t *ctx)
{
    bool ok = false; /* stall outcome                             */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = prelaunch_stall_outcome(&ok); /* status of the call */

    if (!ok)
    {
        return (st);
    }

    pl.onesecxt1 = TEST_PERIOD; /* CAF 1SECX, TS 1SECXT1              */
    agc_imu_set_pipa_interval(TEST_PIPADT); /* CA OC14400            */

    if (!AGC_ASSERT(agc_imu_pipa_interval() == TEST_PIPADT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_job_imu_test_site_display(ctx)); /* GUESS: LATAZCHK  */
}

/**********************************************************************
 * LATAZCHK (IMU_CALIBRATION_AND_ALIGNMENT.agc p.452) -- show AZIMUTH
 * (DSPTEM1, CDU form) and LATITUDE SL2 (DSPTEM1 +1), CLEANDSP
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the test job
 *
 * RETURNS: agc_status_t of CLEANDSP (the job goes on at the flash)
 **********************************************************************/
agc_status_t agc_job_imu_test_site_display(agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t next = {AGC_JOB_IMU_TEST_SITE_FLASH,
                                  AGC_JOB_IMU_TEST_SITE_FLASH,
                                  AGC_JOB_IMU_TEST_SITE_FLASH};
    agc_interp_t          ip; /* interpreter state                    */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_init(&ip);
    agc_dp_t lat = agc_dp_shl(&ip, pl.latitude, 2); /* SL2, rev 2^-2 */

    const uint16_t w[3] = {agc_rtb_1sto2s(pl.azimuth), /* 1STO2S */
                           align_sp_to_word(agc_dp_high(lat)),
                           align_sp_to_word(agc_dp_low(lat))};

    for (int i = 0; i < 3; i++)
    {
        agc_status_t st = agc_dsky_set_dsptem(i, w[i]); /* DSPTEM1 +i */

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return (st);
        }
    }

    return (agc_dsky_display(ctx, AGC_DSKY_CLEANDSP, 0, &next));
}

/**********************************************************************
 * LATAZCHK +7 (p.452) -- flash V06N41; V34 (not allowed) and ENTER
 * flash again, V33 loads
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the test job
 *
 * RETURNS: agc_status_t of GOFLASH
 **********************************************************************/
agc_status_t agc_job_imu_test_site_flash(agc_job_ctx_t *ctx)
{
    const agc_dsky_cont_t cont = {AGC_JOB_IMU_TEST_SITE_FLASH, /* TC +2 */
                                  AGC_JOB_IMU_TEST_SITE_LOADED,
                                  AGC_JOB_IMU_TEST_SITE_FLASH}; /* TC -5 */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(pl.geocomp1 == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, VN_SITE, &cont));
}

/**********************************************************************
 * LATAZCHK +12 / GUESS (IMU_CALIBRATION_AND_ALIGNMENT.agc pp.423, 452)
 * -- AZIMUTH = CDULOGIC(DSPTEM1), LATITUDE = DSPTEM1 +1 SR2; WANGI =
 * -cos LAT SL1, WANGO = sin LAT SL1; then GEOIMUTT
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I  the test job
 *
 * RETURNS: agc_status_t of GEOIMUTT
 **********************************************************************/
agc_status_t agc_job_imu_test_site_loaded(agc_job_ctx_t *ctx)
{
    agc_interp_t ip; /* interpreter state                            */

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_init(&ip);
    pl.azimuth = agc_rtb_cdulogic(agc_dsky_dsptem(0)); /* SLOAD, RTB */
    agc_sp_t lat = align_word_to_sp(agc_dsky_dsptem(1)); /* rev 2^-2  */

    pl.latitude = agc_dp_shr(agc_dp_pack(lat, 0), 2); /* SLOAD SR2, rev 2^0 */
    pl.wangi    = agc_dp_shl(&ip, agc_dp_neg(agc_dp_cos(&ip, pl.latitude)), 1);
    pl.wango    = agc_dp_shl(&ip, agc_dp_sin(&ip, pl.latitude), 1);

    if (!AGC_ASSERT(pl.latitude >= -AGC_DP_QUARTER &&
                    pl.latitude <= AGC_DP_QUARTER))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (geoimutt(ctx)); /* GEOIMUTT                              */
}
