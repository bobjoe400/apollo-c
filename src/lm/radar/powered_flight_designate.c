/**********************************************************************
 * FILE NAME: powered_flight_designate.c
 *
 * PURPOSE:
 *   R29 RENDEZVOUS RADAR DESIGNATE DURING POWERED FLIGHT, LM only:
 *   "Designates the rendezvous radar toward the computed LOS to the
 *   CSM, with the chief objective of obtaining range and range rate data
 *   at 2-second intervals for transmission to the ground. When the RR is
 *   within .5 degree of the computed LOS, tracking is enabled, and
 *   designation continues until the data-good discrete is received. At
 *   that point, designation ceases and a radar-reading routine takes
 *   over, preparing a consistent set of data for down telemetry."
 *   SERVICER calls R29 every 2 seconds (when NOR29FLG, READRFLG and the
 *   RR remode/zero/reposition bits are all clear and the RR is in LGC
 *   mode); R29 returns to NOR29NOW. The RR is first forced into mode 2
 *   (remode from mode 1, or a trunnion preposition to -180 degrees).
 *
 * AGC SOURCE:
 *   Luminary099 P20-P25.agc pp. 594-607 (R29, R29.LOS, SETUP29D,
 *   R29REM0J, PREPOS29, BEGDES29, R29DLOOP, R29DODES, R29DPAS2, DGOOD?,
 *   R29LOKON, END29DOD, R29READ, R29RDJOB, R29RANGE, R29RRR?, ENDRRD29);
 *   LM only
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  ----------------------------
 *   R V R(CSM) V(CSM) agc_vec_t I  2^24 m, m/cs  servicer states (SM)
 *   PIPTIME    agc_dp_t   I    cs 2^28    time of R, V
 *   PIPCTR     agc_sp_t   I    -          SERVICER phase
 *   RRTARGET (LOSSM) LOSVEL (LOSVDT/4) (rr_state_t) I/O  m 2^24
 *   TANG       agc_sp_t   O    pulses     gyro commands
 *   RADMODES   word       I/O  -          DESIGBIT REMODBIT REPOSBIT
 *   CHAN12 CHAN33  channels I/O -         tracker, error counters, DGOOD
 *   LOSCMFLG OLDESFLG READRFLG RNGSCFLG NOR29FLG  flags  I/O
 *   TRKMKCNT   agc_sp_t   O    -          downlink set consistent
 *   RM RDOTM MKTIME TANGNB AIG AMG AOG (agc_rr_downlink_t) O  downlink
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ----------------------------------------------
 *   agc_guid_axisrot      AX*SR*T: SM to NB
 *   agc_guid_quictrig     QUICTRIG of the IMU CDUs
 *   agc_radar_rrrdot      RRRDOT / RRRANGE readings
 *   agc_imu_stall_dev     RADSTALL
 *   rr_axis_start         the trunnion preposition
 *   rr_rrout              RROUT: send the gyro commands
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No alarms. A bad reading or the RR out of auto mode ends the
 *   reading (ENDRRD29) for a redesignate. Executive and waitlist
 *   statuses are passed back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called from SERVICER; the loop runs as .5 s waitlist tasks with
 *   PRIO21 jobs, the reading every 2 s as PRIO26 jobs.
 *
 * NOTES:
 *   Scales: LOSSM (R(CSM) - R + LOSVEL dt) m 2^24 in stable-member
 *   axes, LOSVDT/4 = .5 s of LOS velocity m 2^24, T - PIPTIME cs 2^17;
 *   the designate loop works in single precision (SPSIN, SPCOS, MP).
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "radar.h"

#define PFD_GAIN        (-8786)             /* RR29GAIN DEC -.53624   */
#define PFD_HALFSEC_B17 ((agc_dp_t) 102400) /* .5SECB17 2DEC 50 B-17  */
#define PFD_HALFSEC     50U                 /* .5 s, cs */
#define PFD_2SECS       200U                /* 2 s, cs */
#define PFD_SUPER110    96                  /* SUPER110 OCT 140       */
#define PFD_1SEC        100                 /* 1 s, cs */
#define PFD_PRIO21      ((agc_prio_t) 021)  /* PRIO21 */
#define PFD_PRIO26      ((agc_prio_t) 026)  /* PRIO26 */

/* The R29 downlink set (RM RDOTM MKTIME TANGNB AIG AMG AOG) and the
 * readings kept between the two RADSTALLs of R29RDJOB.                  */
static agc_rr_downlink_t pfd_down;   /* R29 downlink set */
static agc_dp_t          pfd_mktime; /* MPAC: time of the RR reading, cs 2^28 */
static uint16_t          pfd_cdu[5]; /* MPAC +2..+6: CDUT CDUS CDUY CDUZ CDUX */

/**********************************************************************
 * The R29 downlink set (RM RDOTM MKTIME TANGNB AIG AMG AOG).
 * AGC: R29RDJOB (P20-P25.agc p.607)
 *
 * FUNCTION NAME: agc_rendz_rr_downlink
 *
 * ARGUMENTS: none
 *
 * RETURNS: const agc_rr_downlink_t * -- the downlink set (never NULL)
 **********************************************************************/
const agc_rr_downlink_t *agc_rendz_rr_downlink(void)
{
    return (&pfd_down);
}

/**********************************************************************
 * Waitlist delay of the R29 tasks: 4 cs when PIPCTR is positive (the
 * task is offset a second from SERVICER), else 1 s.
 * AGC: "CCS PIPCTR" (P20-P25.agc p.598)
 *
 * FUNCTION NAME: pfd_offset
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint16_t -- the waitlist delay, cs (4 or 100)
 **********************************************************************/
static uint16_t pfd_offset(void)
{
    agc_sp_t c  = agc_guid_state()->avg.pipctr;             /* PIPCTR, unscaled */
    int      dt = ((c > 0) ? -PFD_SUPER110 : 0) + PFD_1SEC; /* CCS PIPCTR */

    return ((uint16_t) dt);
}

/**********************************************************************
 * R29 (p.596), called by SERVICER.
 * AGC: R29 (P20-P25.agc p.596)
 *
 * FUNCTION NAME: agc_radar_powered_flight_designate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG when R29 is
 *          not allowed, else the executive or waitlist status
 **********************************************************************/
agc_status_t agc_radar_powered_flight_designate(void)
{
    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_NO_POWERED_DESIGNATE)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!rr_radmodes_test(RR_DESIGBIT))
    {
        /* R29 first pass (p.596): designate on, tracker off, LOSCMFLG
         * and OLDESFLG off, error counters on; mode 2: preposition the
         * trunnion, mode 1: remode (in its own job, which can RADSTALL). */
        agc_inhint();
        rr_radmodes_set(RR_DESIGBIT);
        rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_TRACK);
        rz_flag_put(AGC_FLAG_LOSCMFLG, false);
        rz_flag_put(AGC_FLAG_OLDESFLG, false);
        rr_setrrecr();
        agc_status_t st; /* waitlist / NOVAC status */

        if (rr_mode2())
        { /* SETPRPOS */
            st = agc_waitlist_call(1U, AGC_TASK_POWERED_FLIGHT_PREPOSITION);
            rr_radmodes_set(RR_REPOSBIT);
        }
        else
        {
            st = agc_exec_novac(PFD_PRIO21, AGC_JOB_POWERED_FLIGHT_REMODE);
            rr_radmodes_clear(RR_DESIGBIT);
            rr_radmodes_set(RR_REMODBIT);
        }

        agc_relint(); /* NOR29NOW */

        return (st);
    }

    /**********************************************************************
     * R29.LOS / SETUP29D (pp.597-598): LOS = R(CSM) - R + (V(CSM) - V)(T -
     * PIPTIME), handed to the designate loop when it is ready for new data
     * (LOSCMFLG off); start the loop (BEGDES29) if it is not requested.
     * AGC: R29.LOS, SETUP29D (P20-P25.agc pp.597-598)
     *
     * FUNCTION NAME: pfd_los
     *
     * ARGUMENTS: none
     *
     * RETURNS: agc_status_t -- AGC_STATUS_OK, else the waitlist status
     **********************************************************************/
    {
        agc_guid_t *g  = agc_guid_state();                                                             /* servicer erasables */
        uint32_t    dt = (agc_time_now_cs() - (uint32_t) agc_erase_piptime()) & (uint32_t) AGC_DP_MAX; /* cs */
        agc_dp_t    dt17;                                                                              /* MP BIT12, cs 2^17 */

        dt17 = (agc_dp_t) ((dt & 037777u) << 11);
        agc_interp_t ip; /* interpreter state */

        agc_interp_init(&ip);
        agc_vec_t losvel = agc_vec_sub(&ip, g->lm.v_csm, g->lm.v);                                                     /* m/cs 2^7 */
        agc_vec_t los    = agc_vec_sub(&ip, agc_vec_add(&ip, agc_vec_scale(&ip, losvel, dt17), g->lm.r_csm), g->lm.r); /* m 2^24 */

        agc_inhint();

        if (agc_flag_test(AGC_FLAG_LOSCMFLG))
        { /* BOFSET: loop on  */
            agc_relint();

            return (AGC_STATUS_OK);
        }

        rz_flag_put(AGC_FLAG_LOSCMFLG, true);
        agc_relint();
        rr_state_t *s = rr_state(); /* radar erasables */

        s->rrtarget = los;                                         /* LOSSM    */
        s->losvel   = agc_vec_scale(&ip, losvel, PFD_HALFSEC_B17); /* LOSVDT/4 */
        rz_flag_put(AGC_FLAG_LOSCMFLG, false);

        if (agc_flag_test(AGC_FLAG_OLDESFLG))
        {
            return (AGC_STATUS_OK);
        }

        agc_inhint();
        rz_flag_put(AGC_FLAG_OLDESFLG, true);
        agc_status_t st; /* BEGDES29 */

        st = agc_waitlist_call(pfd_offset(), AGC_TASK_POWERED_FLIGHT_DESIGNATE);

        agc_relint();

        return (st);
    }
}

/**********************************************************************
 * R29REM0J: REMODE as a task, wait for its end in RADSTALL.
 * AGC: R29REM0J (P20-P25.agc p.597)
 *
 * FUNCTION NAME: agc_job_powered_flight_remode
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_powered_flight_remode(void)
{
    agc_status_t st = agc_waitlist_call(1U, AGC_TASK_RR_REMODE); /* REMODE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING; /* RADSTALL result */

    return (agc_imu_stall_dev(AGC_IMU_DEV_RADAR, AGC_JOB_ENDOFJOB, &res));
}

/**********************************************************************
 * PREPOS29: trunnion to -180 degrees, then repositioning off.
 * AGC: PREPOS29 (P20-P25.agc p.597)
 *
 * FUNCTION NAME: agc_task_powered_flight_preposition
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_powered_flight_preposition(void)
{
    if (!AGC_ASSERT(rr_radmodes_test(RR_REPOSBIT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (rr_axis_start(RR_AXIS_PREPOSITION_POWERED, 0, 0, 040000U));
}

/**********************************************************************
 * BEGDES29: the designate job, again in .5 s.
 * AGC: BEGDES29 (P20-P25.agc p.599)
 *
 * FUNCTION NAME: agc_task_powered_flight_designate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_powered_flight_designate(void)
{
    agc_status_t st; /* R29DODES */

    st = agc_exec_novac(PFD_PRIO21, AGC_JOB_POWERED_FLIGHT_DESIGNATE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_waitlist_vardelay(PFD_HALFSEC, AGC_TASK_POWERED_FLIGHT_DESIGNATE_LOOP));
}

/**********************************************************************
 * R29DLOOP after .5 s: stop when the designate is called off; wait a
 * centisecond while SETUP29D supplies new data (LOSCMFLG on); else mark
 * the loop on (LOSCMFLG) and go round again.
 * AGC: R29DLOOP (P20-P25.agc p.599)
 *
 * FUNCTION NAME: agc_task_powered_flight_designate_loop
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_powered_flight_designate_loop(void)
{
    if (!rr_radmodes_test(RR_DESIGBIT))
    {
        return (AGC_STATUS_OK); /* TASKOVER */
    }

    if (agc_flag_test(AGC_FLAG_LOSCMFLG))
    {
        return (agc_waitlist_vardelay(1U, AGC_TASK_POWERED_FLIGHT_DESIGNATE_LOOP));
    }

    rz_flag_put(AGC_FLAG_LOSCMFLG, true);

    return (agc_task_powered_flight_designate());
}

/* ---- R29DODES (pp.600-604) -------------------------------------------- */
/* SP sines and cosines of the RR CDU angles read on the first pass
 * (SAVECDUT, PUSHLOC, MODE).                                            */
typedef struct pfd_trig
{
    agc_sp_t sin_s; /* SPSIN shaft, 2^1    */
    agc_sp_t cos_s; /* SPCOS shaft, 2^1    */
    agc_sp_t sin_t; /* SPSIN trunnion, 2^1 */
    agc_sp_t cos_t; /* SPCOS trunnion, 2^1 */
} pfd_trig_t;

/**********************************************************************
 * UNIT(LOS) in NB axes (USPRCADR UNIT, MPACVBUF, AX*SR*T with A = -3):
 * the high words of ULOSNB (VBUF, half unit).
 * AGC: R29DODES "USPRCADR UNIT, AX*SR*T" (P20-P25.agc p.601)
 *
 * FUNCTION NAME: pfd_ulosnb
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------------
 *   ip        agc_interp_t *         I/O  interpreter state (OVFIND)
 *   los       agc_vec_t              I    LOS, SM axes, m 2^24
 *   trig      const agc_cdutrig_t *  I    CDUTRIG of the IMU CDUs
 *   u         agc_sp_t               O    ULOSNB high words, half unit 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
static agc_status_t pfd_ulosnb(agc_interp_t *ip, agc_vec_t los, const agc_cdutrig_t *trig, agc_sp_t u[3])
{
    agc_vec_t v = agc_vec_unit(ip, los, NULL, NULL); /* UNIT LOS, 2^1 */

    agc_guid_axisrot(trig, AGC_AXISROT_SMNB, &v);

    for (int i = 0; i < 3; i++)
    {
        u[i] = agc_dp_high(v.c[i]);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * MP chains: high word of a*b, then times c as a DP.
 * AGC: R29DODES "MP / MP" chains (P20-P25.agc p.602)
 *
 * FUNCTION NAME: pfd_mp_chain
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   a         agc_sp_t  I    SP factor, 2^1
 *   b         agc_sp_t  I    SP factor, 2^1
 *   c         agc_sp_t  I    SP factor, 2^1
 *
 * RETURNS: agc_dp_t -- high(a b) c as a DP, 2^3
 **********************************************************************/
static agc_dp_t pfd_mp_chain(agc_sp_t a, agc_sp_t b, agc_sp_t c)
{
    agc_dp_t ab = agc_sp_mul(a, b); /* 2^2 */

    return (agc_sp_mul(agc_dp_high(ab), c));
}

/**********************************************************************
 * First pass: RR CDUs, QUICTRIG of the IMU CDUs, and the self-track
 * enable when ULOSNB . (sin S cos T, -sin T, cos S cos T), doubled,
 * overflows (TESTCOS: within about .5 degree).
 * AGC: R29DODES first pass, TESTCOS (P20-P25.agc pp.600-602)
 *
 * FUNCTION NAME: pfd_first_pass
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------------------
 *   ip        agc_interp_t *   I/O  interpreter state (OVFIND)
 *   los       agc_vec_t        I    LOSSM, m 2^24
 *   trig      agc_cdutrig_t *  O    CDUTRIG of the IMU CDUs
 *   t         pfd_trig_t *     O    SP sines and cosines of the RR CDUs
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
static agc_status_t pfd_first_pass(agc_interp_t *ip, agc_vec_t los, agc_cdutrig_t *trig, pfd_trig_t *t)
{
    agc_inhint();
    uint16_t cdut   = agc_erase_cdut(); /* SAVECDUT */
    uint16_t cdus   = agc_erase_cdus(); /* shaft CDU */
    uint16_t yzx[3] = {agc_erase_cduy(), agc_erase_cduz(), agc_erase_cdux()};

    agc_relint();
    agc_guid_quictrig(trig, yzx);
    agc_status_t st   = AGC_STATUS_OK; /* status of the last step */
    agc_sp_t     u[3] = {0, 0, 0};

    st = pfd_ulosnb(ip, los, trig, u);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_sp_t tv = (agc_sp_t) rr_ones(cdut); /* trunnion, rev 2^0 */
    agc_sp_t sv = (agc_sp_t) rr_ones(cdus); /* shaft, rev 2^0 */

    t->cos_t  = agc_sp_cos(tv);                                                                                                                     /* PUSHLOC  */
    t->sin_t  = (agc_sp_t) -agc_sp_sin((agc_sp_t) -tv);                                                                                             /* -MODE    */
    t->sin_s  = agc_sp_sin(sv);
    t->cos_s  = agc_sp_cos(sv);
    int64_t c = (int64_t) agc_sp_mul((agc_sp_t) -t->sin_t, u[1]) + pfd_mp_chain(t->sin_s, t->cos_t, u[0]) + pfd_mp_chain(t->cos_s, t->cos_t, u[2]); /* cos, 2^2 */

    if (2 * c > (int64_t) AGC_DP_MAX)
    { /* TESTCOS  */
        agc_chan_or(AGC_CHAN_CHAN12, RR_CH12_TRACK);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * R29DODES: the LOS now (first pass) and LOSSM advanced .5 s, projected
 * 1 s ahead (second pass).
 * AGC: R29DODES (P20-P25.agc pp.600-604)
 *
 * FUNCTION NAME: agc_job_powered_flight_designate
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_powered_flight_designate(void)
{
    rr_state_t  *s = rr_state(); /* radar erasables */
    agc_interp_t ip;             /* interpreter state */

    agc_interp_init(&ip);
    agc_cdutrig_t trig; /* CDUTRIG sines / cosines */
    pfd_trig_t    t;    /* RR CDU sines / cosines */
    agc_status_t  st;   /* first pass */

    st = pfd_first_pass(&ip, s->rrtarget, &trig, &t);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    s->rrtarget   = agc_vec_add(&ip, s->rrtarget, s->losvel); /* DAS LOSSM */
    agc_sp_t u[3] = {0, 0, 0};

    st = pfd_ulosnb(&ip, agc_vec_add(&ip, s->rrtarget, s->losvel), &trig, u);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /**********************************************************************
     * R29DPAS2: commands to null half the error in .5 s:
     *   shaft    = ULOSNB' . (cos S, 0, -sin S)
     *   trunnion = ULOSNB' . (sin S sin T, cos T, cos S sin T), both x
     *   RR29GAIN.
     * AGC: R29DPAS2 (P20-P25.agc p.603)
     *
     * FUNCTION NAME: pfd_commands
     *
     * ARGUMENTS:
     *   Argument  Type                I/O  Description (scale)
     *   --------  ------------------  ---  ----------------------------
     *   t         const pfd_trig_t *  I    SP sines and cosines of the RR CDUs
     *   u         const agc_sp_t      I    ULOSNB of the advanced LOS, 2^1
     **********************************************************************/
    agc_dp_t shaft; /* 2^2 */

    shaft         = agc_sp_mul(t.cos_s, u[0]) + agc_sp_mul((agc_sp_t) -t.sin_s, u[2]);
    agc_dp_t trun = pfd_mp_chain(t.sin_t, t.sin_s, u[0]) + agc_sp_mul(t.cos_t, u[1]) + pfd_mp_chain(t.sin_t, t.cos_s, u[2]); /* 2^2 */

    agc_erase_set_tang(0, agc_dp_high(agc_sp_mul(agc_dp_high(trun), PFD_GAIN)));
    agc_erase_set_tang(1, agc_dp_high(agc_sp_mul(agc_dp_high(shaft), PFD_GAIN)));

    /**********************************************************************
     * DGOOD? / R29LOKON / END29DOD (p.604): on RR data good stop designating
     * and start the reading task; else send the commands.
     * AGC: DGOOD?, R29LOKON, END29DOD (P20-P25.agc p.604)
     *
     * FUNCTION NAME: pfd_lock_check
     *
     * ARGUMENTS: none
     *
     * RETURNS: agc_status_t -- AGC_STATUS_OK, else the waitlist status
     **********************************************************************/
    agc_status_t st_check = AGC_STATUS_OK; /* status of the last step */

    agc_inhint();

    if ((agc_chan_read(AGC_CHAN_CHAN33) & RR_CH33_RR_DGOOD) != 0U)
    {
        rr_rrout();
    }
    else
    { /* R29LOKON */
        rr_radmodes_clear(RR_DESIGBIT);
        rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_ENB_ECTR);
        rz_flag_put(AGC_FLAG_READRFLG, true);
        st_check = agc_waitlist_call(pfd_offset(), AGC_TASK_POWERED_FLIGHT_READ);
    }

    rz_flag_put(AGC_FLAG_LOSCMFLG, false); /* END29DOD */
    agc_relint();

    return (st_check);
}

/* ---- R29READ / R29RDJOB (pp.605-607) ---------------------------------- */
/**********************************************************************
 * R29READ: the reading job every 2 s while reading is allowed.
 * AGC: R29READ (P20-P25.agc p.605)
 *
 * FUNCTION NAME: agc_task_powered_flight_read
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_powered_flight_read(void)
{
    agc_status_t st; /* R29RDJOB */

    st = agc_exec_novac(PFD_PRIO26, AGC_JOB_POWERED_FLIGHT_READ);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_waitlist_vardelay(PFD_2SECS, AGC_TASK_POWERED_FLIGHT_READ_LOOP));
}

/**********************************************************************
 * R29READ after 2 s: again while READRFLG is set, else wait for a
 * redesignate.
 * AGC: R29READ (P20-P25.agc p.605)
 *
 * FUNCTION NAME: agc_task_powered_flight_read_loop
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_powered_flight_read_loop(void)
{
    if (!agc_flag_test(AGC_FLAG_READRFLG))
    {
        return (AGC_STATUS_OK); /* wait for a redesignate     */
    }

    return (agc_task_powered_flight_read());
}

/**********************************************************************
 * ENDRRD29: leave the reading mode, discredit the downlink.
 * AGC: ENDRRD29 (P20-P25.agc p.607)
 *
 * FUNCTION NAME: pfd_read_end
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          failed invariant
 **********************************************************************/
static agc_status_t pfd_read_end(void)
{
    agc_erase_set_trkmkcnt(0);
    rz_flag_put(AGC_FLAG_READRFLG, false);
    rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_TRACK);

    if (!AGC_ASSERT(agc_erase_trkmkcnt() == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* ENDOFJOB */
}

/**********************************************************************
 * RADSTALL for this job: it sleeps until the reading ends, or, when the
 * reading is already over, continues at once as wake_as.
 * AGC: RADSTALL (P20-P25.agc p.605)
 *
 * FUNCTION NAME: pfd_stall
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  ----------------------------------
 *   wake_as   agc_job_id_t  I    job that continues after the reading
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (asleep or continued), else the
 *          RADSTALL status
 **********************************************************************/
static agc_status_t pfd_stall(agc_job_id_t wake_as)
{
    agc_imu_stall_result_t res = AGC_IMU_STALL_SLEEPING;                              /* RADSTALL result */
    agc_status_t           st  = agc_imu_stall_dev(AGC_IMU_DEV_RADAR, wake_as, &res); /* RADSTALL */

    if (st != AGC_STATUS_OK || res == AGC_IMU_STALL_SLEEPING)
    {
        return (st);
    }

    return (rz_continue(wake_as));
}

/**********************************************************************
 * The reading in RADSTALL ended good (GOODEND).
 * AGC: RADSTALL return +2 (P20-P25.agc p.605)
 *
 * FUNCTION NAME: pfd_read_good
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true for GOODEND
 **********************************************************************/
static bool pfd_read_good(void)
{
    return (agc_imu_stall_outcome(AGC_IMU_DEV_RADAR) == AGC_IMU_STALL_GOOD);
}

/**********************************************************************
 * R29RDJOB: stop at once when R29 is over or the RR left auto mode;
 * read range rate.
 * AGC: R29RDJOB (P20-P25.agc p.605)
 *
 * FUNCTION NAME: agc_job_powered_flight_read
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_powered_flight_read(void)
{
    if (agc_flag_test(AGC_FLAG_NO_POWERED_DESIGNATE) || rr_radmodes_test(RR_AUTOMBIT))
    {
        return (pfd_read_end());
    }

    agc_status_t st = agc_radar_rrrdot(); /* RRRDOT */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (pfd_stall(AGC_JOB_POWERED_FLIGHT_READ_RANGE));
}

/**********************************************************************
 * After the range-rate read: a bad read redesignates; else keep the time
 * and the five CDUs of the reading and (R29RANGE) read range.
 * AGC: R29RDJOB, R29RANGE (P20-P25.agc p.606)
 *
 * FUNCTION NAME: agc_job_powered_flight_read_range
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_powered_flight_read_range(void)
{
    if (!pfd_read_good())
    {
        return (pfd_read_end());
    }

    pfd_mktime      = rr_timehold();
    agc_inhint();
    pfd_cdu[0]      = agc_erase_cdut();
    pfd_cdu[1]      = agc_erase_cdus();
    pfd_cdu[2]      = agc_erase_cduy();
    pfd_cdu[3]      = agc_erase_cduz();
    pfd_cdu[4]      = agc_erase_cdux();
    agc_relint();
    agc_status_t st = agc_radar_rrrange(); /* RRRANGE */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (pfd_stall(AGC_JOB_POWERED_FLIGHT_READ_DONE));
}

/**********************************************************************
 * After the range read: a scale change (RNGSCFLG) reads range again, a
 * bad read redesignates; else lay out the downlink set (COPYCYCLE) and
 * mark it consistent (TRKMKCNT = 1).
 * AGC: R29RRR?, COPYCYCLE (P20-P25.agc pp.606-607)
 *
 * FUNCTION NAME: agc_job_powered_flight_read_done
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the step ended or queued its
 *          successor, AGC_STATUS_BAD_ARG for a NULL ctx, else the status of
 *          the executive, display or navigation call that failed
 **********************************************************************/
agc_status_t agc_job_powered_flight_read_done(void)
{
    if (!pfd_read_good())
    { /* R29RRR?  */
        if (!agc_flag_test(AGC_FLAG_RNGSCFLG))
        {
            return (pfd_read_end());
        }

        rz_flag_put(AGC_FLAG_RNGSCFLG, false);
        agc_status_t st = agc_radar_rrrange(); /* R29RANGE */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        return (pfd_stall(AGC_JOB_POWERED_FLIGHT_READ_DONE));
    }

    agc_inhint();
    pfd_down.rm        = agc_erase_dnrrange(0); /* DNRRANGE */
    pfd_down.rdotm     = agc_erase_dnrrange(1); /* DNRRDOT  */
    pfd_down.mktime    = pfd_mktime;
    pfd_down.tangnb[0] = pfd_cdu[0];
    pfd_down.tangnb[1] = pfd_cdu[1];
    pfd_down.aig       = pfd_cdu[2];
    pfd_down.amg       = pfd_cdu[3];
    pfd_down.aog       = pfd_cdu[4];
    agc_erase_set_trkmkcnt(1);
    agc_relint();

    return (AGC_STATUS_OK); /* ENDOFJOB */
}
