/**********************************************************************
 * FILE NAME: imu_performance_test.c
 *
 * PURPOSE:
 *   IMU_PERFORMANCE_TEST_2: REDO's steps (GEOIMUTT / IMUBACK, LATAZCHK
 *   and the V06N41 site display, SHOW), POSGMBL coarse and fine align,
 *   the PIPA scale test (PIPACHK, CHECKG, PIPATASK, PIPJOBB), the
 *   vertical drift test (VERTDRFT), TORQUE, the earth-rate routines
 *   ERTHRVSE / EARTHR / EARTHR*, IMUSTALL outcomes and the subroutines
 *   ZEROING, OVERFFIX, ENDTEST1.
 *
 * AGC SOURCE:
 *   Luminary099 IMU_PERFORMANCE_TEST_2.agc pp.373-381
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors or by ECADR)
 *   Name          Type          I/O  Scale     Description
 *   ------------  ------------  ---  --------  ----------------------------
 *   W .. AINLA+99 uint16_t[]    I/O  words     the test erasables (st_words())
 *   XNB, XSM      agc_mat_t     I/O  2^1       navigation base, stable member
 *   THETAD        uint16_t[3]   O    CDU       coarse align angles
 *   DELV, PIPAs   agc_vec_t     I/O  pulses    PIPA counts
 *   DSPTEM1/2     uint16_t      O    words     V06N41, V06N98 registers
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_imu_zero/coarse/  IMUZERO, IMUCOARS, IMUFINE, IMUSTALL,
 *    fine/stall           IMUPULSE (imu lane)
 *   agc_imu_calcga        CALCGA
 *   agc_dsky_display      GOFLASH V06N41 / V06N98
 *   agc_waitlist_*        TWIDDLE, LONGCALL
 *   agc_exec_*            FINDVAC, JOBSLEEP, JOBWAKE, CHANG1
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   CCSHOLE when the first PIPA count is +0; the test ends with ENDTEST1
 *   (V34) or SOMERR2 (drift_filter.c).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only; the jobs and tasks of P07 (V92).
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "internal.h"

/**********************************************************************
 * ZEROING (IMU_PERFORMANCE_TEST_2 p.379): store +0 in n consecutive words
 * (the AGC takes the address in A and the count - 1 in ZERONDX1).
 *
 * ARGUMENTS:
 *   block  uint16_t *  I   the first word to clear
 *   n      int         I   number of words, 1..256
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for a count outside 1..256
 **********************************************************************/
agc_status_t agc_systems_test_zeroing(uint16_t *block, int n)
{
    if (!AGC_ASSERT(n >= 1 && n <= ZEROING_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < n; i++)
    {
        block[i] = 0;
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * OVERFFIX (p.378): DAD DPPOSMAX, DAD ONEDPP -- add 2^28 to a time
 * difference that went negative across TIME2 overflow (wraps like DAD).
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 *   a   agc_dp_t        I   the negative time difference, cs 2^28
 *
 * RETURNS: a + 2^28, cs 2^28
 **********************************************************************/
static agc_dp_t agc_systems_test_overffix(agc_interp_t *ip, agc_dp_t a)
{
    agc_dp_t s = agc_dp_add(ip, a, AGC_DP_MAX); /* running sum, cs 2^28 */

    s = agc_dp_add(ip, s, ONEDPP);

    return (s);
}

/**********************************************************************
 * ENDTEST1 (p.377): IMUSE down, MODREG -0 (NEWMODEA), ENDEXT.
 *
 * RETURNS: the NEWMODEA or ENDEXT status
 **********************************************************************/
agc_status_t endtest1(void)
{
    agc_flag_clear(AGC_FLAG_IMUSE);
    agc_status_t st = agc_phase_newmode(MODREG_NONE); /* NEWMODEA status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_dsky_endmark());
}

/**********************************************************************
 * ALIGNCOA / IMUSLLLG (p.378): IMUSTALL; bad -> SOMERR2.  *good: the mode
 * switch is over and good, go on now (else the job sleeps as wake_as).
 *
 * ARGUMENTS:
 *   wake_as  agc_job_id_t  I   the job that goes on after a stall that
 *                              sleeps
 *   good     bool *        O   true: the mode switch is over and good, go
 *                              on now
 *
 * RETURNS: AGC_STATUS_OK (asleep, good, or SOMERR2 done), or the IMUSTALL
 *          status
 **********************************************************************/
agc_status_t imu_stall(agc_job_id_t wake_as, bool *good)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;       /* STALL return */
    agc_status_t           st  = agc_imu_stall(wake_as, &res); /* IMUSTALL status */

    *good = false;

    if ((st != AGC_STATUS_OK) || (res == AGC_IMU_STALL_SLEEPING))
    {
        return (st);
    }

    if (res == AGC_IMU_STALL_BAD)
    {
        somerr2();

        return (AGC_STATUS_OK);
    }

    *good = true;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- The woken side of IMUSLLLG: bad -> SOMERR2.
 *
 * ARGUMENTS:
 *   good  bool *  O   true when the stalled mode switch ended good
 *
 * RETURNS: none
 **********************************************************************/
void imu_stall_outcome(bool *good)
{
    *good = (agc_imu_stall_outcome(AGC_IMU_DEV_IMU) == AGC_IMU_STALL_GOOD);

    if (!*good)
    {
        somerr2(); /* bad: SOMERR2 */
    }
}

/**********************************************************************
 * ERTHRVSE (p.380): ERVECTOR = (SIN LAT, -COS LAT, 0) OMEG/MS, TMARK =
 * LOADTIME, ERCOMP = 0.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND)
 **********************************************************************/
void erthrvse(agc_interp_t *ip)
{
    const agc_dp_t zero[3] = {0, 0, 0};
    agc_dp_t       lat     = w_dp(W_LATITUDE);                                              /* LATITUDE, rev 2^0 */
    agc_vec_t      er      = agc_vec_make(agc_dp_sin(lat), agc_dp_neg(agc_dp_cos(lat)), 0); /* (SIN, -COS, 0) LAT, then x OMEG/MS (rev/cs 2^-23) */

    er = agc_vec_scale(ip, er, OMEG_MS);

    for (int i = 0; i < 3; i++)
    {
        w_set_dp(W_ERVECTOR + 2 * i, er.c[i]);
    }

    w_set_dp(W_TMARK, agc_rtb_loadtime());
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, zero); /* STORE ERCOMP */
}

/**********************************************************************
 * EARTHR (p.380): ERCOMP += XSM (ERVECTOR (TIME - TMARK) 2^9), TMARK =
 * TIME, PULSEIMU ERCOMP (on GYROBUSY the job sleeps as busy_wake).
 *
 * ARGUMENTS:
 *   ip         agc_interp_t *  I   interpreter state (OVFIND)
 *   busy_wake  agc_job_id_t    I   the job run again when the gyros are
 *                                  busy
 *
 * RETURNS: the PULSEIMU status (BUSY: asleep; ALARM: IMU caged), or the
 *          gyro buffer read status
 **********************************************************************/
static agc_status_t earthr(agc_interp_t *ip, agc_job_id_t busy_wake)
{
    agc_dp_t  ercomp[3];                                                                             /* ERCOMP gyro commands, rev 2^0 */
    agc_dp_t  temptime = agc_rtb_loadtime();                                                         /* TEMPTIME = LOADTIME, cs 2^28 */
    agc_dp_t  d        = agc_dp_sub(ip, temptime, w_dp(W_TMARK));                                    /* TIME - TMARK, cs 2^28 */
    agc_vec_t er       = agc_vec_make(w_dp(W_ERVECTOR), w_dp(W_ERVECTOR + 2), w_dp(W_ERVECTOR + 4)); /* ERVECTOR, rev/cs (scale of OMEG/MS) */

    w_set_dp(W_TEMPTIME, temptime);

    if (d < 0)
    {
        d = agc_systems_test_overffix(ip, d); /* BPL ERTHR, CALL OVERFFIX */
    }

    agc_vec_t v = agc_vec_scale(ip, er, agc_dp_shl(ip, d, 9)); /* ERTHR: SL 9D, VXSC: rev 2^0 */

    v = agc_mat_vec(ip, &agc_align_ram()->xsm, v);
    agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, ercomp);

    for (int i = 0; i < 3; i++)
    {
        ercomp[i] = agc_dp_add(ip, v.c[i], ercomp[i]); /* VAD ERCOMP */
    }

    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, ercomp);
    w_set_dp(W_TMARK, temptime);

    return (agc_imu_pulse_buf(AGC_IMU_GBUF_ERCOMP, busy_wake)); /* PULSEIMU */
}

/**********************************************************************
 * EARTHR* (p.380) after PULSEIMU: IMUSLLLG, then TC QPLACES.
 *
 * ARGUMENTS:
 *   pulse  agc_status_t  I   the PULSEIMU status
 *
 * RETURNS: AGC_STATUS_OK (asleep, or the job goes on as the QPLACES
 *          return), or the failure
 **********************************************************************/
agc_status_t earthr_stall(agc_status_t pulse)
{
    bool good = false; /* torquing over and good */

    if (pulse == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* GYROBUSY: asleep          */
    }

    if ((pulse != AGC_STATUS_OK) && (pulse != AGC_STATUS_ALARM))
    {
        return (pulse); /* ALARM: caged, the stall sees it */
    }

    agc_status_t st = imu_stall(AGC_JOB_SYSTEMS_TEST_EARTH_RATE_DONE, &good); /* IMUSLLLG status */

    if (!good)
    {
        return (st);
    }

    /* TC QPLACES as the job's next run (see DEVIATION: keeps the call
     * graph free of the cycle through the step dispatch).                 */
    return (agc_exec_job_yield(AGC_JOB_SYSTEMS_TEST_EARTH_RATE_DONE));
}

/**********************************************************************
 * EARTHR* (p.380): QPLACES = next, EARTHR, IMUSLLLG, TC QPLACES.
 *
 * ARGUMENTS:
 *   next  st_step_t  I   QPLACES, the step after the torquing
 *
 * RETURNS: the EARTHR / IMUSLLLG status
 **********************************************************************/
agc_status_t earthr_star(st_step_t next)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    w_set_sp(W_QPLACES, (agc_sp_t) next);

    return (earthr_stall(earthr(&ip, AGC_JOB_SYSTEMS_TEST_EARTH_RATE_RETRY)));
}

/**********************************************************************
 * SHOW1 (p.381): DSPTEM2 +2 = POSITON, flash V06N98: V34 ENDTEST1, V33 TC
 * QPLACE, ENTER again.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t show1(agc_job_ctx_t *ctx)
{
    uint16_t       *w    = st_words(); /* W .. AINLA +99D, the test erasables */
    agc_dsky_cont_t cont = {AGC_JOB_SYSTEMS_TEST_END, AGC_JOB_SYSTEMS_TEST_RESUME, AGC_JOB_SYSTEMS_TEST_SHOW};

    agc_dsky_set_dsptem(5, w[W_POSITON]);

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, 98), &cont));
}

/**********************************************************************
 * SHOW (p.380): QPLACE = the step V33 goes on at, SHOW1.
 *
 * ARGUMENTS:
 *   next  st_step_t        I   QPLACE, the step V33 goes on at
 *   ctx   agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t show(st_step_t next, agc_job_ctx_t *ctx)
{
    w_set_sp(W_QPLACE, (agc_sp_t) next);

    return (show1(ctx));
}

/**********************************************************************
 * LATAZCHK (p.373): flash V06N41 (R1 azimuth, R2 latitude): V34 ENDTEST1,
 * V33 POSGMBL, ENTER (new data) again (TC -5).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t site_display(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_SYSTEMS_TEST_END, AGC_JOB_SYSTEMS_TEST_SITE_LOADED, AGC_JOB_SYSTEMS_TEST_SITE_DISPLAY};

    return (agc_dsky_display(ctx, AGC_DSKY_GOFLASH, agc_dsky_vn(6, 41), &cont));
}

/**********************************************************************
 * IMUBACK / NBPOSPL / GUESS / LATAZCHK (p.373): clear the indices, XNB =
 * (1/2, 0, 0), YNB = ZNB = 0, show LATITUDE SL2 and AZIMUTH as a CDU.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
agc_status_t imuback(agc_job_ctx_t *ctx)
{
    uint16_t    *w  = st_words();            /* W .. AINLA +99D, the test erasables */
    agc_interp_t ip;                         /* interpreter state */
    agc_mat_t   *nb = &agc_align_ram()->xnb; /* XNB, YNB, ZNB, half-unit */

    agc_interp_init(&ip);
    w[W_NDXCTR]      = 0;
    w[W_TORQNDX]     = 0;
    w[W_TORQNDX + 1] = 0;
    w[W_OVFLOWCK]    = 0;
    nb->row[0]       = agc_vec_make(AGC_DP_HALF, 0, 0);      /* ZEROING 18D, CA HALF */
    nb->row[1]       = agc_vec_zero();
    nb->row[2]       = agc_vec_zero();
    agc_dp_t lat2    = agc_dp_shl(&ip, w_dp(W_LATITUDE), 2); /* DLOAD SL2: rev 2^-2 */
    agc_dp_t az      = w_dp(W_AZIMUTH);                      /* AZIMUTH, rev 2^0 */

    agc_erase_set_dsptem1(1, agc_dp_high(lat2)); /* STODL DSPTEM1 +1 */
    agc_erase_set_dsptem1(2, agc_dp_low(lat2));
    agc_dsky_set_dsptem(0, agc_rtb_1sto2s(az));  /* RTB 1STO2S, TS DSPTEM1 */

    return (site_display(ctx));
}

/**********************************************************************
 * REDO / GEOIMUTT / IMUZERR (p.373, p.378): MM 07, IMUZERO, IMUSTALL.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the NEWMODEX, IMUZERO or IMUSTALL status, or that of IMUBACK
 **********************************************************************/
agc_status_t redo(agc_job_ctx_t *ctx)
{
    bool         good = false;                                           /* IMUZERO over and good */
    agc_status_t st   = agc_phase_newmode((agc_sp_t) SYSTEMS_TEST_MODE); /* NEWMODEX 07 */

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_zero(); /* GEOIMUTT: IMUZERR */
    }

    if ((st != AGC_STATUS_OK) && (st != AGC_STATUS_BUSY))
    {
        return (st);
    }

    st = imu_stall(AGC_JOB_SYSTEMS_TEST_IMU_ZEROED, &good);

    if (!good)
    {
        return (st);
    }

    return (imuback(ctx));
}

/**********************************************************************
 * FINIMUDD done (p.374): LONGCALL PERFDLAY GOESTIMS, JOBSLEEP ESTICADR.
 *
 * RETURNS: AGC_STATUS_OK (asleep until GOESTIMS), or the LONGCALL status
 *          (PERFDLAY out of range)
 **********************************************************************/
agc_status_t fine_aligned(void)
{
    agc_status_t st = agc_waitlist_longcall((uint32_t) w_dp(W_PERFDLAY), AGC_TASK_SYSTEMS_TEST_DRIFT_WAKE); /* LONGCALL status */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* PERFDLAY (ground loaded) out of LONGCALL range */
    }

    agc_exec_job_sleep(AGC_JOB_SYSTEMS_TEST_DRIFT_START); /* CA ESTICADR, TC JOBSLEEP */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * CHECKG (p.378): QPLACE = next, ZERONDX = -PIPA, then CHECKG1 (yield).
 *
 * ARGUMENTS:
 *   next  st_step_t  I   QPLACE, the step after the pulse is caught
 *
 * RETURNS: the CHANG1 status, or BAD_ARG for a bad PIPINDEX
 **********************************************************************/
static agc_status_t checkg(st_step_t next)
{
    int          axis = 0;                 /* PIPINDEX */
    agc_status_t st   = pipa_index(&axis); /* PIPINDEX check */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    w_set_sp(W_QPLACE, (agc_sp_t) next);
    w_set_sp(W_ZERONDX, (agc_sp_t) -pipa_get(axis)); /* CS PIPAX, TS ZERONDX */

    return (agc_exec_job_yield(AGC_JOB_SYSTEMS_TEST_PIP_CATCH));
}

/**********************************************************************
 * PIPACHK +2 (p.375): 17 counter overflows, 58 passes, sample 1, PIPA and
 * DATAPL zeroed, CHECKG, then PIPATASK in 20 ms.
 *
 * RETURNS: the CHECKG status
 **********************************************************************/
agc_status_t pipachk_arm(void)
{
    uint16_t    *w    = st_words();        /* W .. AINLA +99D, the test erasables */
    int          axis = 0;                 /* PIPINDEX */
    agc_status_t st   = pipa_index(&axis); /* PIPINDEX check */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    w_set_sp(W_DATAPL + 4, DEC17);
    w_set_sp(W_LENGTHOT, DEC58);
    w_set_sp(W_RESULTCT, 1);
    pipa_set(axis, 0);
    w[W_DATAPL] = 0;

    return (checkg(STEP_PIPATASK_START));
}

/**********************************************************************
 * PIPACHK (p.375): INDEX NDXCTR -- earth rate first unless in gimbal lock.
 *
 * RETURNS: the EARTHR* or CHECKG status
 **********************************************************************/
agc_status_t pipachk(void)
{
    uint16_t *w = st_words(); /* W .. AINLA +99D, the test erasables */

    if (w[W_NDXCTR] == 0u)
    {
        return (earthr_star(STEP_PIPACHK_ARM));
    }

    return (pipachk_arm());
}

/**********************************************************************
 * CCS NDXCTR (p.374): gimbal lock -> PIPA test only, else FINIMUDD.
 *
 * RETURNS: the status of PIPACHK, IMUFINE, the stall or FINIMUDD's
 *          LONGCALL
 **********************************************************************/
agc_status_t coarse_aligned(void)
{
    uint16_t *w    = st_words(); /* W .. AINLA +99D, the test erasables */
    bool      good = false;      /* IMUFINE over and good */

    if (ccs(w[W_NDXCTR]).branch == 0)
    {
        return (pipachk()); /* CCS NDXCTR: PIPA test only */
    }

    agc_status_t st = agc_imu_fine(); /* FINIMUDD: IMUFINE */

    if ((st != AGC_STATUS_OK) && (st != AGC_STATUS_BUSY))
    {
        return (st);
    }

    st = imu_stall(AGC_JOB_SYSTEMS_TEST_FINE_ALIGNED, &good);

    if (!good)
    {
        return (st);
    }

    return (fine_aligned());
}

/**********************************************************************
 * LATAZCHK +12 / POSGMBL (pp.373-374): AZIMUTH and LATITUDE from the
 * display, WANGI / WANGO, YNB, ZNB from AZIMUTH; CALCGA, IMUCOARS; count
 * a gimbal lock in NDXCTR; IMUSLLLG.
 *
 * RETURNS: the CALCGA, IMUCOARS or stall status, or that of what follows
 **********************************************************************/
agc_status_t posgmbl(void)
{
    uint16_t        *w    = st_words();                                                                                           /* W .. AINLA +99D, the test erasables */
    agc_interp_t     ip;                                                                                                          /* interpreter state */
    agc_align_ram_t *ar   = agc_align_ram();                                                                                      /* XNB, XSM, half-unit */
    uint16_t         thetad[3];                                                                                                   /* THETAD, CDU patterns */
    bool             good = false;                                                                                                /* IMUCOARS over and good */
    const uint16_t   r2   = agc_dsky_dsptem(1);                                                                                   /* DSPTEM1 +1 */
    const agc_dp_t   lat  = agc_dp_shr(agc_dp_from_sp((agc_sp_t) (((r2 & 040000u) != 0u) ? -(int) (WORD15 - r2) : (int) r2)), 2); /* SLOAD, SR2: rev 2^0 */
    const agc_dp_t   az   = agc_rtb_cdulogic(agc_dsky_dsptem(0));                                                                 /* SLOAD DSPTEM1, CDULOGIC: rev 2^0 */

    agc_interp_init(&ip);
    w_set_dp(W_AZIMUTH, az);
    w_set_dp(W_LATITUDE, lat);
    w_set_dp(W_WANGI, agc_dp_shl(&ip, agc_dp_neg(agc_dp_cos(lat)), 1));    /* COS DCOMP SL1 */
    w_set_dp(W_WANGO, agc_dp_shl(&ip, agc_dp_sin(lat), 1));                /* SIN SL1       */
    ar->xnb.row[1].c[1] = agc_dp_sin(az);                                  /* YNB +2        */
    ar->xnb.row[2].c[2] = agc_dp_sin(az);                                  /* ZNB +4        */
    ar->xnb.row[1].c[2] = agc_dp_cos(az);                                  /* YNB +4        */
    ar->xnb.row[2].c[1] = agc_dp_neg(agc_dp_cos(az));                      /* ZNB +2        */
    agc_status_t st     = agc_imu_calcga(&ip, &ar->xnb, &ar->xsm, thetad); /* CALCGA, IMUCOARS, stall status */

    if (st == AGC_STATUS_OK)
    {
        st = agc_imu_coarse(thetad); /* IMUCOARS */
    }

    if ((st != AGC_STATUS_OK) && (st != AGC_STATUS_BUSY))
    {
        return (st);
    }

    if (agc_flag_test(AGC_FLAG_GLOKFAIL))
    {
        w[W_NDXCTR] = (uint16_t) ((w[W_NDXCTR] + 1u) & WORD15); /* INCR NDXCTR: gimbal lock */
    }

    agc_flag_clear(AGC_FLAG_GLOKFAIL);
    st = imu_stall(AGC_JOB_SYSTEMS_TEST_COARSE_ALIGNED, &good);

    if (!good)
    {
        return (st);
    }

    return (coarse_aligned());
} /* posgmbl */

/**********************************************************************
 * PIPJOBB +3 (p.376): end unless the passes are over; sample 5, CHECKG.
 *
 * RETURNS: AGC_STATUS_OK (ENDOFJOB while passes are left) or the CHECKG
 *          status
 **********************************************************************/
agc_status_t pipjobb_catch(void)
{
    uint16_t *w = st_words(); /* W .. AINLA +99D, the test erasables */

    if ((w[W_LENGTHOT] & 040000u) == 0u && w[W_LENGTHOT] != 0u)
    {
        return (AGC_STATUS_OK); /* LENGTHOT > 0: ENDOFJOB  */
    }

    w_set_sp(W_RESULTCT, 5);

    return (checkg(STEP_PIPJOBB_RESULT));
}

/**********************************************************************
 * GUESS1 (p.377): only the south gyro gets earth rate; TORQNDX = POSMAX
 * (vertical drift), LOSVEC = CDUX, ESTIMS.
 *
 * RETURNS: the ESTIMS status
 **********************************************************************/
agc_status_t guess1(void)
{
    uint16_t *w = st_words(); /* W .. AINLA +99D, the test erasables */

    w[W_ERVECTOR]     = 0; /* ERVECTOR, +1: south gyro only */
    w[W_ERVECTOR + 1] = 0;
    w[W_TORQNDX]      = POSMAX;
    w[W_TORQNDX + 1]  = POSMAX;
    w[W_LOSVEC]       = agc_erase_cdux();

    return (estims());
}

/**********************************************************************
 * VERTDRFT / PON2 / PON4 / PON (pp.376-377): LENGTHOT = 3990 (about 1 h),
 * DRIFTT = -SOUTHDR(POSITON - 2), offset ERCOMP to miss the PIP dead
 * zones (X up: Y -16, Z +16; Z up in position 4: Y -16, X +16), EARTHR*.
 *
 * ARGUMENTS:
 *   lengthot  agc_sp_t  I   LENGTHOT, 3990 passes (or 0 from a -0 NDXCTR)
 *
 * RETURNS: the EARTHR* status, BAD_ARG for a bad POSITON, or the gyro
 *          buffer status
 **********************************************************************/
agc_status_t vertdrft(agc_sp_t lengthot)
{
    uint16_t *w = st_words();                            /* W .. AINLA +99D, the test erasables */
    int       k = W_SOUTHDR - 2 + (int) w_sp(W_POSITON); /* SOUTHDR -2 +POSITON, word offset */
    agc_dp_t  er[3];                                     /* ERCOMP gyro commands, rev 2^0 */

    if (!AGC_ASSERT(k >= 0 && k < W_WORDS))
    {
        return (AGC_STATUS_BAD_ARG); /* POSITON is a ground-loaded word */
    }

    w_set_sp(W_LENGTHOT, lengthot);
    w[W_DRIFTT] = (uint16_t) (WORD15 - w[k]); /* INDEX POSITON, CS SOUTHDR -2 */
    agc_imu_gyro_buf_get(AGC_IMU_GBUF_ERCOMP, er);

    /* CCS PIPINDEX: Z up in position 4 (PON4) or X up (PON2).            */
    int up = (w_sp(W_PIPINDEX) > 0) ? 0 : 2; /* ERCOMP component of the up axis */

    er[1]  = er[1] - (agc_dp_t) BIT5 * 040000;  /* CS BIT5, ADS ERCOMP +2 */
    er[up] = er[up] + (agc_dp_t) BIT5 * 040000; /* CA BIT5, ADS ERCOMP or +4 */
    agc_imu_gyro_buf_set(AGC_IMU_GBUF_ERCOMP, er);

    return (earthr_star(STEP_GUESS1)); /* PON */
}

/**********************************************************************
 * AINGOTN's rate (pp.376): DATAPL +4 = the pulse count since sample 1
 * (DCS DATAPL, DAS DATAPL +4: 17 counter overflows plus the difference),
 * time = DATAPL +6 - DATAPL +2 (OVERFFIX across a scaler overflow), rate
 * = count / time (DDV) x DEC585 (DMPR), word for word.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I   interpreter state (OVFIND, DVSIGN)
 *
 * RETURNS: MPAC: pulses per scaler count x DEC585, as DMPR leaves it
 **********************************************************************/
static agc_tp_words_t pip_rate(agc_interp_t *ip)
{
    static const agc_dp_words_t dec585   = {3200, 0};                                          /* OCT 06200, 0 */
    static const agc_dp_words_t dpposmax = {037777, 037777};                                   /* DPPOSMAX     */
    static const agc_dp_words_t onedpp   = {0, 1};                                             /* ONEDPP       */
    agc_dp_words_t              first    = w_words(W_DATAPL);                                  /* DATAPL, +1: sample 1 count */
    agc_dp_words_t              neg      = {(agc_sp_t) -first.hi, (agc_sp_t) -first.lo};
    int                         ovf      = 0;                                                  /* DAS overflow (counts are far below 1) */
    agc_dp_words_t              count    = agc_dp_words_das(w_words(W_DATAPL + 4), neg, &ovf); /* pulses since sample 1, 2^-28 per pulse */

    w_set_sp(W_DATAPL + 4, count.hi);
    w_set_sp(W_DATAPL + 5, count.lo);
    agc_tp_words_t t = agc_mpac_dsu(ip, agc_mpac_dload(w_words(W_DATAPL + 6)), w_words(W_DATAPL + 2)); /* scaler time, 2^-28 per count */

    if (agc_tp_from_words(t) < 0)
    {
        t = agc_mpac_dad(ip, agc_mpac_dad(ip, t, dpposmax), onedpp); /* CALL OVERFFIX */
    }

    agc_tp_words_t m = agc_mpac_ddv(ip, agc_mpac_dload(count), agc_mpac_store(t)); /* PDDL DDV */

    return (agc_mpac_dmpr(ip, m, dec585));
}

/**********************************************************************
 * AINGOTN (p.376): pulses / time x DEC585 into DSPTEM2; COAALIGN in gimbal
 * lock; SHOW, then VERTDRFT.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the CCSHOLE, IMUCOARS, stall or display status, or that of
 *          VERTDRFT
 **********************************************************************/
agc_status_t pipjobb_result(agc_job_ctx_t *ctx)
{
    uint16_t    *w    = st_words();           /* W .. AINLA +99D, the test erasables */
    agc_interp_t ip;                          /* interpreter state */
    st_ccs_t     c    = ccs(w[W_DATAPL + 1]); /* CCS DATAPL +1: first count */
    bool         good = false;                /* IMUCOARS over and good */

    if (c.branch == 1)
    {
        return (agc_ccshole()); /* TC CCSHOLE */
    }

    if (c.branch >= 2)
    {
        w[W_DATAPL + 4] = (c.branch == 2) ? (uint16_t) (WORD15 - w[W_DATAPL + 4]) : 0u; /* CS / TS DATAPL +4 */
    }

    agc_interp_init(&ip);
    agc_dp_words_t rate = agc_dp_to_words(agc_tp_to_dp(agc_tp_from_words(pip_rate(&ip)))); /* RTB SGNAGREE */

    agc_erase_set_dsptem1(3, rate.hi); /* STORE DSPTEM2 */
    agc_erase_set_dsptem1(4, rate.lo);
    st_ccs_t n = ccs(w[W_NDXCTR]);     /* CCS NDXCTR */

    if (n.branch >= 2)
    {
        return (vertdrft((n.branch == 2) ? DEC3990 : 0)); /* CCS NDXCTR falls into VERTDRFT */
    }

    if (n.branch == 1)
    {
        return (show(STEP_VERTDRFT, ctx));
    }

    /* COAALIGN (p.378): THETAD = 0, IMUCOARS, IMUSTALL; then SHOW.        */
    const uint16_t zero[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_thetad(i, 0);
    }

    agc_status_t st = agc_imu_coarse(zero); /* IMUCOARS, stall status */

    if ((st != AGC_STATUS_OK) && (st != AGC_STATUS_BUSY))
    {
        return (st);
    }

    st = imu_stall(AGC_JOB_SYSTEMS_TEST_GIMBALS_ZEROED, &good);

    return (good ? show(STEP_VERTDRFT, ctx) : st); /* asleep or SOMERR2: st */
} /* pipjobb_result */

/**********************************************************************
 * TORQUE (p.374): DSPTEM2 = 0, DRIFTI; SOUTHDR(POSITON - 1) = DRIFTI;
 * SHOW, then PIPACHK.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display status, BAD_ARG for a bad POSITON
 **********************************************************************/
agc_status_t torque(agc_job_ctx_t *ctx)
{
    uint16_t *w = st_words();                            /* W .. AINLA +99D, the test erasables */
    int       k = W_SOUTHDR - 1 + (int) w_sp(W_POSITON); /* SOUTHDR -1 +POSITON, word offset */

    if (!AGC_ASSERT(k >= 0 && k < W_WORDS))
    {
        return (AGC_STATUS_BAD_ARG); /* POSITON is a ground-loaded word */
    }

    agc_dsky_set_dsptem(3, 0);
    agc_dsky_set_dsptem(4, w[W_DRIFTI]);
    w[k] = w[W_DRIFTI]; /* INDEX POSITON, TS SOUTHDR -1 */

    return (show(STEP_PIPACHK, ctx));
}

/**********************************************************************
 * CHECKG1 (pp.378-379): no pulse yet -> sample again (CHANG1 between);
 * a pulse: DATAPL +RESULTCT = PIPA, +1, +2 = FINETIME; TC QPLACE.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the CHANG1 status (no pulse yet), BAD_ARG for a bad PIPINDEX
 *          or RESULTCT, or the step's status
 **********************************************************************/
agc_status_t pip_catch(agc_job_ctx_t *ctx)
{
    int          axis = 0;                                 /* PIPINDEX */
    agc_dp_t     t    = 0;                                 /* FINETIME HISCALAR, LOSCALAR, counts 2^-28 */
    agc_status_t st   = pipa_index(&axis);                 /* PIPINDEX / FINETIME status */
    int          r    = W_DATAPL + (int) w_sp(W_RESULTCT); /* INDEX RESULTCT */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_sp_t p = pipa_get(axis); /* PIPA count now, pulses 2^0 */

    if (p == (agc_sp_t) -w_sp(W_ZERONDX))
    {
        w_set_sp(W_ZERONDX, (agc_sp_t) -p); /* AD ZERONDX, BZF CHECKG1 */

        return (agc_exec_job_yield(AGC_JOB_SYSTEMS_TEST_PIP_CATCH));
    }

    st = agc_imu_finetime(&t); /* returns with INHINT */

    if (st == AGC_STATUS_BUSY || !AGC_ASSERT(r >= W_DATAPL && r + 2 < W_WORDS))
    {
        agc_relint();

        return ((st == AGC_STATUS_BUSY) ? agc_exec_job_yield(AGC_JOB_SYSTEMS_TEST_PIP_CATCH) : AGC_STATUS_BAD_ARG); /* RESULTCT overwritten */
    }

    w_set_sp(r, p);
    w_set_sp(r + 1, agc_dp_high(t));
    w_set_sp(r + 2, agc_dp_low(t));
    agc_relint();

    return (run_step((st_step_t) w_sp(W_QPLACE), ctx)); /* ENDCHKG: TC QPLACE */
}

/**********************************************************************
 * PIPATASK (p.375): DIM LENGTHOT; passes left -> again in 5.12 s, else
 * STARTPIP: FINDVAC PIPJOBB.
 *
 * RETURNS: the TWIDDLE or FINDVAC status
 **********************************************************************/
agc_status_t pip_timer(void)
{
    uint16_t *w = st_words();         /* W .. AINLA +99D, the test erasables */
    st_ccs_t  c = ccs(w[W_LENGTHOT]); /* CCS LENGTHOT */

    if (c.branch == 0)
    {
        w[W_LENGTHOT] = c.a; /* DIM: one count toward zero */
    }
    else if (c.branch == 2)
    {
        w[W_LENGTHOT] = (uint16_t) (WORD15 - c.a);
    }

    if (c.branch == 0 && c.a != 0u)
    {
        return (agc_waitlist_call(PIP_PERIOD_CS, AGC_TASK_SYSTEMS_TEST_PIP_TIMER)); /* TWIDDLE */
    }

    return (agc_exec_findvac(PRIO20, AGC_JOB_SYSTEMS_TEST_PIP_RESULT)); /* STARTPIP */
}
