/*
 * maneuver.c -- KALCMANU: automatic attitude maneuvers for the RCS DAP.
 *
 * Sources (CM = Comanche055, LM = Luminary099):
 *   GOMANUR            IMU_MODE_SWITCHING_ROUTINES.agc CM p.1442 / LM p.1331
 *   KALCMAN3, NOGO     ANGLFIND.agc CM pp.399-411 / ATTITUDE_MANEUVER_ROUTINE
 *                      .agc LM pp.351-363 (mathematics in maneuver_calc.c)
 *   WCALC (LOCSKIRT)   GIMBAL_LOCK_AVOIDANCE.agc CM p.412 / LM p.364
 *   NEWDELHI .. ENDMANU KALCMANU_STEERING.agc CM pp.414-419 / LM pp.365-369
 *
 * A maneuver: GOMANUR saves the user's return (ATTCADR) and priority and
 * runs KALCMAN3 in the user's job.  KALCMAN3 finds the rotation axis COF
 * and angle AM taking the present attitude to CPHI, CTHETA, CPSI, and DEL,
 * the rotation the attitude makes in one second at the selected rate.
 * Every second (UPDTCALL -> NEWDELHI) the desired attitude matrix MIS is
 * advanced by DEL and the DAP is given new desired CDU angles and the
 * per-0.1 s increments; in the last second MANUSTOP sets the final angles
 * and ENDMANU / GOODMANU restarts the user's job at its priority.
 *
 * CM vs LM differences, all kept (agc_state_vehicle()):
 *   * small maneuvers: CM sets HOLDFLAG -1 and LOADCDUD; LM loads CDUXD..
 *     from CPHI.. (TOOBADI).  LM also refuses |CPSI| >= 70 deg (TOOBADF).
 *   * rates: CM ARATE .05/.2/.5/2 deg/s, LM .2/.5/2/10 deg/s; CM rotates
 *     BRATE into control axes (QUADROT, -7.25 deg about X) and forms
 *     BIASTEMP; LM sets CALCMAN3 (NOGIMLOC).
 *   * DAP interface: CM writes WBODY/BIAS/DELCDUX (DP)/HOLDFLAG/RCSFLAGS
 *     in the CSM DAP; LM writes OMEGAPD/DELPEROR/DELCDUX (SP, negative
 *     increment)/CDUXD in the LM DAP.
 *   * NEWDELHI: CM aborts when HOLDFLAG >= 0 (astronaut took over), LM
 *     when ISITAUTO says the DAP is not in auto.
 *   * TIMECHK return layout (same timing result).
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/flags_common.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/core/status.h"
#include "agc/core/word.h"
#include "agc/csm/dap.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/lm/dap.h"
#include "internal.h"
#include "pending.h"

#define PRIO26          ((agc_prio_t) 026) /* NEWDELHI                   */
#define ALARM_GLOCK     ((agc_alarm_code_t) 00401)
#define ALARM_TWO_USERS ((agc_alarm_code_t) 01210)
#define ONESEC_CS       100         /* 1SEC / ONESEK +1           */
#define TIME1_MOD       040000      /* TIME1 is 14 bits           */
#define TIME_MOD        0x10000000L /* TIME2,TIME1 wrap at 2^28   */
#define DT_TENTH        1638        /* QUADROT / DT/TAU: DEC .1   */
#define CDU_MASK        077777u
#define RATE_COUNT      4

/* ARATE (GLA p.412 / p.364): maneuver rates at 22.5 deg/s, by RATEINDX/2. */
static const agc_dp_t arate_cm[RATE_COUNT] = {
    596523,  /* 2DEC .0022222222 =  .05 deg/s                           */
    2386093, /* 2DEC .0088888889 =  .2 deg/s                            */
    5965232, /* 2DEC .0222222222 =  .5 deg/s                            */
    23860929 /* 2DEC .0888888889 = 2.0 deg/s                            */
};
static const agc_dp_t arate_lm[RATE_COUNT] = {
    2386093,  /* 2DEC .0088888888 =  0.2 deg/s                           */
    5965232,  /* 2DEC .0222222222 =  0.5 deg/s                           */
    23860929, /* 2DEC .0888888888 =  2.0 deg/s                           */
    119304647 /* 2DEC .4444444444 = 10.0 deg/s                           */
};
#define ANGLTIME    ((agc_dp_t) 51200) /* 2DEC .000190735 = 100 B-19     */
#define BIASCALE    ((agc_dp_t) 68267) /* 2DEC .0002543132 (CM)          */
#define BIASCALE_LM ((agc_sp_t) -1024) /* OCT 75777 = -1/16 (LM)         */
/* QUADROT (CM): S/C to control axes, X rotation -7.25 deg, scaled .1.    */
static const agc_mat_t quadrot = {{
                                      {{26843546, 0, 0}}, /* .1,      0,       0           */
                                      {{0, 26628797, -3387655}}, /* 0,  .099200, -.012620         */
                                      {{0, 3387655, 26628797}} /* 0,  .012620,  .099200         */
                                  }};

/* KALCMANU erasables (module-private; EBANK BCDU).                       */
static uint16_t     bcdu[3];     /* BCDU: initial / previous desired CDUs   */
static uint16_t     ncdu[3];     /* NCDU: new desired CDU angles            */
static agc_mat_t    mis;         /* MIS: S/C -> SM now, scale 2             */
static agc_mat_t    del;         /* DEL (CM) / KEL (LM): 1-s rotation, 1    */
static agc_vec_t    cof;         /* COF: maneuver axis, half-unit           */
static agc_dp_t     am;          /* AM: maneuver angle, revolutions         */
static agc_vec_t    brate;       /* BRATE: maneuver rates, 450 / 45 deg/s   */
static agc_vec_t    biastemp;    /* BIASTEMP (CM): attitude error biases    */
static agc_dp_t     tm;          /* TM: maneuver time / end time, cs 2^28   */
static int          nextime;     /* NEXTIME: TIME1 of the next update, cs   */
static agc_sp_t     rateindx_lm; /* RATEINDX (LM): 0, 2, 4, 6               */
static agc_job_id_t attcadr;     /* ATTCADR: user's continuation, 0 = free  */
static agc_prio_t   attprio;     /* ATTPRIO: user's priority                */

static bool is_lm(void)
{
    agc_vehicle_t v = agc_state_vehicle();

    if (!AGC_ASSERT(v == AGC_VEHICLE_LM || v == AGC_VEHICLE_CSM))
    {
        return false;
    }

    return v == AGC_VEHICLE_LM;
}

/* Two's-complement CDU pattern as a signed count at 180 deg (CSM DAP).  */
static agc_sp_t cdu_signed(uint16_t p)
{
    if (!AGC_ASSERT(p <= CDU_MASK))
    {
        return 0; /* no valid result */
    }

    int v = (p & 040000u) ? (int) p - 0100000 : (int) p;

    if (!AGC_ASSERT(v >= -16384 && v <= 16383))
    {
        return 0; /* no valid result */
    }

    return (agc_sp_t) v;
}

/* "TS CDUXD" (CM): the high word of the DP desired angle.               */
static void csm_set_cdud(int axis, uint16_t p)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return;
    }

    agc_csm_rcs_t *rcs = agc_csm_dap_rcs();

    if (!AGC_ASSERT(rcs != NULL))
    {
        return;
    }

    rcs->cdud[axis] =
        (agc_dp_t) cdu_signed(p) * 16384 + agc_dp_low(rcs->cdud[axis]);
}

static void lm_set_cdud(int axis, uint16_t p)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3 && p <= CDU_MASK))
    {
        return;
    }

    if (axis == 0)
    {
        agc_erase_set_cduxd(p);
    }
    else if (axis == 1)
    {
        agc_erase_set_cduyd(p);
    }
    else
    {
        agc_erase_set_cduzd(p);
    }
}

static void set_cdud(int axis, uint16_t p)
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return;
    }

    if (!AGC_ASSERT(p <= CDU_MASK))
    {
        return;
    }

    if (is_lm())
    {
        lm_set_cdud(axis, p);
    }
    else
    {
        csm_set_cdud(axis, p);
    }
}

static uint16_t cphi(int axis) /* CPHI, CTHETA, CPSI       */
{
    if (!AGC_ASSERT(axis >= 0 && axis < 3))
    {
        return 0; /* no valid result */
    }

    uint16_t v = agc_erase_thetad(axis);

    if (!AGC_ASSERT(v <= CDU_MASK))
    {
        return 0; /* no valid result */
    }

    return v;
}

static int msu(uint16_t a, uint16_t b) /* MSU: a - b, 1:S counts    */
{
    if (!AGC_ASSERT(a <= CDU_MASK && b <= CDU_MASK))
    {
        return 0; /* no valid result */
    }

    int d = (int) ((unsigned) (a - b) & CDU_MASK);

    d = (d >= 040000) ? d - 0100000 : d;

    if (!AGC_ASSERT(d >= -040000 && d < 040000))
    {
        return 0; /* no valid result */
    }

    return d;
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

/* ---- RATEINDX ------------------------------------------------------------ */

agc_status_t agc_kalc_set_rateindx(agc_sp_t idx)
{
    if (!AGC_ASSERT(idx >= 0 && idx <= 6 && (idx & 1) == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (is_lm())
    {
        rateindx_lm = idx;

        return AGC_STATUS_OK;
    }

    agc_csm_rcs_t *rcs = agc_csm_dap_rcs();

    if (!AGC_ASSERT(rcs != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    rcs->ratindx = idx; /* CM: kept with the DAP (S41.2)    */

    return AGC_STATUS_OK;
}

agc_sp_t agc_kalc_rateindx(void)
{
    agc_sp_t idx = rateindx_lm;

    if (!is_lm())
    {
        const agc_csm_rcs_t *rcs = agc_csm_dap_rcs();

        idx = (rcs != NULL) ? rcs->ratindx : 0;
    }

    if (!AGC_ASSERT(idx >= 0 && idx <= 6))
    {
        idx = 0;
    }

    return idx;
}

bool agc_kalc_active(void)
{
    if (!AGC_ASSERT(attprio <= 037))
    {
        return false;
    }

    return attcadr != AGC_JOB_NONE;
}

agc_imu_stall_result_t agc_kalc_outcome(void)
{
    if (!AGC_ASSERT(attprio <= 037))
    {
        return AGC_IMU_STALL_BAD;
    }

    /* KALCMANU has no failure return: ENDMANU / GOODMANU always resume
     * the user (the AGC's "all returns are now made via GOODEND").       */
    return agc_kalc_active() ? AGC_IMU_STALL_SLEEPING : AGC_IMU_STALL_GOOD;
}

/* ======================================================================= */
/* DAP interface (KALCMANU_STEERING CM pp.418-419; LM p.369, DAP)          */
/* ======================================================================= */

/* STOPYZ (CM): zero pitch and yaw increments, rates and biases.          */
static void csm_stopyz(agc_csm_rcs_t *rcs)
{
    if (!AGC_ASSERT(rcs != NULL))
    {
        return;
    }

    for (int i = 1; i < 3; i++)
    {
        rcs->delcdu[i] = 0;
        rcs->wbody[i]  = 0;
        rcs->bias[i]   = 0;
    }

    if (!AGC_ASSERT(rcs->wbody[2] == 0))
    {
        return;
    }
}

/* STOPRATE: CM zeroes roll then falls into STOPYZ and resets the high-
 * rate flag (RCSFLAGS bit 15); LM calls the DAP's STOPRATE.             */
void agc_kalc_stoprate(void)
{
    if (is_lm())
    {
        agc_lm_dap_stoprate();

        return;
    }

    agc_csm_rcs_t    *rcs = agc_csm_dap_rcs();
    agc_csm_dapcom_t *com = agc_csm_dap_com();

    if (!AGC_ASSERT(rcs != NULL && com != NULL))
    {
        return;
    }

    rcs->delcdu[0] = 0;
    rcs->wbody[0]  = 0;
    rcs->bias[0]   = 0;

    if (!AGC_ASSERT(rcs->wbody[0] == 0))
    {
        return;
    }

    com->rcsflags = (uint16_t) (com->rcsflags &
                                (~(unsigned) AGC_CSM_RCSF_HIRATE & 077777u));
    csm_stopyz(rcs);
}

/* ZEROERROR (CM p.419): CDUXD.. = CDUX..; LM: ZATTEROR in the DAP.      */
agc_status_t agc_kalc_zeroerror(void)
{
    if (is_lm())
    {
        agc_lm_dap_zatteror();

        return AGC_STATUS_OK;
    }

    set_cdud(0, agc_erase_cdux());
    set_cdud(1, agc_erase_cduy());
    set_cdud(2, agc_erase_cduz());

    if (!AGC_ASSERT(agc_erase_cduz() <= CDU_MASK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(agc_erase_cdux() <= CDU_MASK))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* LOADCDUD / LOADYZ (CM p.419): desired angles = CPHI, CTHETA, CPSI.    */
static void loadcdud(int first_axis)
{
    if (!AGC_ASSERT(first_axis == 0 || first_axis == 1))
    {
        return;
    }

    for (int i = first_axis; i < 3; i++)
    {
        set_cdud(i, cphi(i));
    }

    if (!AGC_ASSERT(cphi(2) <= CDU_MASK))
    {
        return;
    }
}

/* ======================================================================= */
/* Termination: NOGO, MANUSTOP, ENDMANU / GOODMANU                         */
/* ======================================================================= */

/* NOGO (ANGLFIND p.411 / AMR p.363): stop any rate and end via ENDMANU
 * (CM) or GOODMANU (LM) in 20 ms.  Returns from the job (ENDOFJOB).      */
static agc_status_t nogo(void)
{
    if (is_lm())
    {
        agc_kalc_stoprate(); /* BANKCALL STOPRATE         */
        agc_inhint();
    }
    else
    {
        agc_inhint();
        agc_kalc_stoprate();
    }

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        agc_relint();

        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = waitlist(2u, AGC_TASK_ENDMANU);

    agc_relint();

    return st; /* ENDOFJOB        */
}

/* NOGO -2 / NOGOM2: the maneuver was interrupted: zero the attitude error
 * (ZEROEROR CM / ZATTEROR LM) and NOGO.                                  */
static agc_status_t nogom2(void)
{
    agc_inhint();
    agc_status_t st = agc_kalc_zeroerror();

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED))
    {
        agc_relint();

        return AGC_STATUS_BAD_ARG;
    }

    agc_relint();

    return nogo();
}

/* MANUSTOP (task): set the final angles and stop the rates.              */
agc_status_t agc_task_manustop(void)
{
    if (!is_lm())
    {
        agc_csm_rcs_t *rcs = agc_csm_dap_rcs();

        if (!AGC_ASSERT(rcs != NULL))
        {
            return AGC_STATUS_BAD_ARG;
        }

        csm_stopyz(rcs);      /* STOPYZ          */
        loadcdud(1);          /* LOADYZ          */
        set_cdud(0, cphi(0)); /* ENDROLL         */
        agc_kalc_stoprate();

        return agc_task_endmanu();
    }

    agc_erase_set_delcduz(0); /* DELDCDU2        */
    agc_erase_set_omegard(0);
    agc_erase_set_delreror(0);
    agc_erase_set_delcduy(0); /* DELDCDU1        */
    agc_erase_set_omegaqd(0);
    agc_erase_set_delqeror(0);
    loadcdud(1);          /* CDUZD, CDUYD    */
    set_cdud(0, cphi(0)); /* ENDROLL         */
    agc_erase_set_omegapd(0);
    agc_erase_set_delcdux(0); /* DELDCDU         */
    agc_erase_set_delperor(0);

    return agc_task_endmanu(); /* GOODMANU        */
}

/* ENDMANU (CM) / GOODMANU (LM) (task): restore the user's priority, free
 * KALCMANU (ATTCADR = 0) and restart the user's job (SPVAC).             */
agc_status_t agc_task_endmanu(void)
{
    agc_job_id_t job  = attcadr;
    agc_prio_t   prio = attprio; /* NEWPRIO         */

    attcadr = AGC_JOB_NONE;

    if (!AGC_ASSERT(job != AGC_JOB_NONE))
    {
        return AGC_STATUS_OK; /* nobody waiting  */
    }

    agc_status_t st = agc_exec_findvac(prio, job); /* SPVAC           */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* ======================================================================= */
/* Steering updates (KALCMANU_STEERING CM pp.414-417, LM pp.365-368)       */
/* ======================================================================= */

/* INCRDCDU (one axis): CM DELCDUX (DP) = (NCDU - BCDU) x .1;
 * LM DELDCDU (SP) = (BCDU - NCDU) x .1.  CDUXD = old BCDU, BCDU = NCDU. */
static void incrdcdu_axis(int i)
{
    if (!AGC_ASSERT(i >= 0 && i < 3))
    {
        return;
    }

    uint16_t old = bcdu[i];

    if (is_lm())
    {
        int d = msu(bcdu[i], ncdu[i]);

        d = (d < AGC_SP_MIN) ? AGC_SP_MIN : d;
        agc_sp_t inc = agc_dp_high(agc_sp_mul((agc_sp_t) d, DT_TENTH));

        /* CCS / AD ONE / COM: ones' to two's complement, same value.     */
        if (i == 0)
        {
            agc_erase_set_delcdux(inc);
        }
        else if (i == 1)
        {
            agc_erase_set_delcduy(inc);
        }
        else
        {
            agc_erase_set_delcduz(inc);
        }
    }
    else
    {
        int d = msu(ncdu[i], bcdu[i]);

        d = (d < AGC_SP_MIN) ? AGC_SP_MIN : d;
        agc_csm_rcs_t *rcs = agc_csm_dap_rcs();

        if (AGC_ASSERT(rcs != NULL))
        {
            rcs->delcdu[i] =
                agc_sp_mul((agc_sp_t) d, DT_TENTH); /* MP QUADROT */
        }
    }

    bcdu[i] = ncdu[i];
    set_cdud(i, old);
}

/* TIMECHK + MANUSTAL (p.415 / p.366): time left = TM - TIME2,TIME1 word
 * by word, as the DAS leaves it.  More than a second left: 0 (go on to
 * CONTMANU).  Otherwise the MANUSTOP delay in cs: the remainder of the
 * last second (MANUOFF), or 1 (2NDRETRN).                                */
static uint16_t timechk(void)
{
    long now = (long) agc_time_now_cs();

    if (!AGC_ASSERT(now >= 0 && now < TIME_MOD))
    {
        return 0; /* no valid result */
    }

    long hi = ((long) tm >> 14) - (now >> 14);
    long lo = ((long) tm & 037777L) - (now & 037777L);

    if (!AGC_ASSERT(lo > -040000L && lo < 040000L))
    {
        return 0; /* no valid result */
    }

    if (hi > 0 || (hi == 0 && lo > 0))
    {
        return 0u; /* TC Q: continue  */
    }

    if (hi < 0)
    {
        return 1u; /* 2NDRETRN        */
    }

    /* lo = +0: MANUOFF with A = 0.  lo < 0: CCS leaves |lo| - 1, COM
     * makes it -(|lo| - 1); then AD 1SEC and BZMF 2NDRETRN.             */
    long a = ONESEC_CS + ((lo == 0) ? 0 : lo + 1);

    return (a <= 0) ? 1u : (uint16_t) a;
}

/* CONTMANU / MANUCALL (p.416 / p.367): UPDTCALL at NEXTIME, then the next
 * update a second later.                                                 */
static agc_status_t contmanu(void)
{
    agc_inhint();

    if (!AGC_ASSERT(nextime >= 0 && nextime < TIME1_MOD))
    {
        agc_relint();

        return AGC_STATUS_BAD_ARG;
    }

    int diff = nextime - (int) agc_erase_time1();
    int dt   = (diff > 0) ? diff : (diff == 0) ? 1 : TIME1_MOD + diff;

    /* DEVIATION: NEXTIME == TIME1 gives the AGC a zero WAITLIST delay;
     * one centisecond is used instead.                                   */
    if (!AGC_ASSERT(dt > 0 && dt < TIME1_MOD))
    {
        agc_relint();

        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = waitlist((uint16_t) dt, AGC_TASK_UPDTCALL);

    agc_relint();
    nextime = (nextime + ONESEC_CS) & (TIME1_MOD - 1); /* ADS NEXTIME    */

    return st;                                         /* ENDOFJOB       */
}

/* INCRDCDU loop, RELINT, TMANUCHK (p.414-415 / p.365-366).               */
static agc_status_t incrdcdu_tmanuchk(void)
{
    for (int i = 2; i >= 0; i--)
    { /* CAF TWO: Z, Y, X */
        incrdcdu_axis(i);
    }

    if (!AGC_ASSERT(bcdu[0] == ncdu[0] && bcdu[1] == ncdu[1]))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(bcdu[2] == ncdu[2]))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (agc_interrupts_inhibited())
    {
        agc_relint();
    }

    uint16_t dt = timechk();

    if (dt == 0u)
    {
        return contmanu();
    }

    agc_inhint(); /* MANUSTAL         */
    agc_status_t st = waitlist(dt, AGC_TASK_MANUSTOP);

    agc_relint();

    return st; /* ENDOFJOB         */
}

/* MANUSTAT (CM p.416): completion time, enable auto maneuvers in the DAP
 * (HOLDFLAG -1), rates, biases and the high-rate flag.                   */
static void manustat_cm(void)
{
    agc_csm_rcs_t    *rcs = agc_csm_dap_rcs();
    agc_csm_dapcom_t *com = agc_csm_dap_com();

    if (!AGC_ASSERT(rcs != NULL && com != NULL))
    {
        return;
    }

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return;
    }

    rcs->holdflag = -1;

    if (6 - agc_kalc_rateindx() <= 0)
    { /* HIGHGAIN         */
        com->rcsflags = (uint16_t) (com->rcsflags | AGC_CSM_RCSF_HIRATE);
    }

    for (int i = 0; i < 3; i++)
    {
        rcs->wbody[i] = brate.c[i];                /* WBODY, 1, 2      */
        rcs->bias[i]  = agc_dp_low(biastemp.c[i]); /* BIASTEMP +1,3,5  */
    }
}

/* RATEBIAS (LM p.367): OMEGAxD = BRATE, DELxEROR = -|w| w / 16 / 1JACCx. */
static void manustat_lm(void)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return;
    }

    for (int i = 2; i >= 0; i--)
    {
        agc_sp_t w    = agc_dp_high(brate.c[i]);
        agc_sp_t jacc = (i == 0) ? agc_erase_1jacc() :
                        (i == 1) ? agc_erase_1jaccq() :
                        agc_erase_1jaccr();
        agc_sp_t a   = (agc_sp_t) ((w > 0) ? -w : w);           /* -|W|             */
        agc_sp_t q   = agc_dp_high(agc_sp_mul(a, BIASCALE_LM)); /* |W|/16   */
        agc_dp_t p   = agc_sp_mul(q, w);                        /* MP BRATE (A, L)  */
        agc_dp_t lag = 0;

        if (AGC_ASSERT(jacc != 0 && agc_dp_high(p) < (jacc < 0 ? -jacc : jacc)))
        {
            lag = p / jacc; /* DV 1JACC         */
        }

        if (i == 0)
        {
            agc_erase_set_omegapd(w);
            agc_erase_set_delperor((agc_sp_t) lag);
        }
        else if (i == 1)
        {
            agc_erase_set_omegaqd(w);
            agc_erase_set_delqeror((agc_sp_t) lag);
        }
        else
        {
            agc_erase_set_omegard(w);
            agc_erase_set_delreror((agc_sp_t) lag);
        }
    }
}

/* MANUSTAT: TM = TM + TIME - 1 s, vehicle part, NEXTIME = TIME1 + 1 s.   */
static void manustat(void)
{
    long t = ((long) tm + (long) agc_time_now_cs() - ONESEC_CS) % TIME_MOD;

    tm = (agc_dp_t) ((t < 0) ? t + TIME_MOD : t);
    agc_inhint();

    if (is_lm())
    {
        manustat_lm();
    }
    else
    {
        manustat_cm();
    }

    nextime = ((int) agc_erase_time1() + ONESEC_CS) & (TIME1_MOD - 1);

    if (!AGC_ASSERT(tm >= 0))
    {
        agc_relint();

        return;
    }

    if (!AGC_ASSERT(nextime >= 0 && nextime < TIME1_MOD))
    {
        agc_relint();

        return;
    }
}

/* NEWANGL +1 (p.414 / p.365): MIS = MIS DEL; NCDU from MIS; on the first
 * pass (CALCMAN2 set) MANUSTAT; then the DAP increments and time check.  */
static agc_status_t newangl1(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    mis = agc_mat_mul(ip, &mis, &del); /* MXM3             */
    agc_vec_t    ang;
    agc_dp_t     cmga;
    agc_status_t st = agc_kalc_dcmtocdu(ip, &mis, &ang, &cmga);

    if (st == AGC_STATUS_OK)
    {
        st = agc_rtb_v1sto2s(ang, ncdu); /* NCDU             */
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }

    if (agc_flag_test(AGC_FLAG_CALCMAN2))
    { /* BONCLR CALCMAN2  */
        agc_flag_clear(AGC_FLAG_CALCMAN2);
        manustat();
    }

    return incrdcdu_tmanuchk();
}

/* WCALC (LOCSKIRT; GLA p.412 / NOGIMLOC p.364): DEL for one second at the
 * selected rate, BRATE, maneuver time TM, (CM) BIASTEMP; start.          */
static agc_status_t wcalc(agc_interp_t *ip)
{
    int idx = agc_kalc_rateindx() / 2;

    if (!AGC_ASSERT(idx >= 0 && idx < RATE_COUNT))
    {
        return AGC_STATUS_BAD_ARG;
    }

    bool     lm = is_lm();
    agc_dp_t ar = lm ? arate_lm[idx] : arate_cm[idx];

    if (lm)
    {
        imu_vflag_set(IMU_VF_CALCMAN3); /* NOGIMLOC         */
    }

    agc_status_t st = agc_kalc_delcomp(ip, cof, agc_dp_shr(ar, 4), &del);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }

    agc_vec_t b = agc_vec_scale(ip, cof, ar);

    brate = lm ? b : agc_mat_vec(ip, &quadrot, b); /* MXV QUADROT (CM) */
    tm    = agc_dp_shr(agc_dp_div(ip, agc_dp_mul(ip, am, ANGLTIME), ar), 5);

    if (!lm)
    {
        biastemp = agc_vec_scale(ip, brate, BIASCALE);
    }

    agc_flag_set(AGC_FLAG_CALCMAN2); /* SETGO CALCMAN2   */

    return newangl1(ip);
}

/* ======================================================================= */
/* Entry points                                                            */
/* ======================================================================= */

/* Small maneuver (< .25 deg): go directly into attitude hold about the
 * commanded angles.  CM: HOLDFLAG = -1, LOADCDUD.  LM: CDUXD.. = CPHI..
 * (TLOAD CPHI, STCALL CDUXD, TOOBADI).                                  */
static agc_status_t small_maneuver(void)
{
    if (!is_lm())
    {
        agc_csm_rcs_t *rcs = agc_csm_dap_rcs();

        if (!AGC_ASSERT(rcs != NULL))
        {
            return AGC_STATUS_BAD_ARG;
        }

        agc_inhint();
        rcs->holdflag = -1;
        loadcdud(0);
        agc_relint();
    }
    else
    {
        loadcdud(0);
    }

    if (!AGC_ASSERT(!agc_interrupts_inhibited()))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return nogo();
}

/* KALCMAN3 (ANGLFIND p.399 / AMR p.351): set up and start the maneuver
 * to CPHI, CTHETA, CPSI from the present CDU angles (READCDUK).          */
static agc_status_t kalcman3(void)
{
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_inhint(); /* READCDUK         */
    bcdu[0] = agc_erase_cdux();
    bcdu[1] = agc_erase_cduy();
    bcdu[2] = agc_erase_cduz();
    agc_relint();
    const uint16_t   fin[3] = {cphi(0), cphi(1), cphi(2)};
    agc_kalc_setup_t k;
    agc_status_t     st = agc_kalc_compute(&ip, bcdu, fin, &k);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }

    mis = k.mis;
    cof = k.cof;
    am  = k.am;

    switch (k.kcase)
    {
        case AGC_KALC_TOOBAD: /* TOOBADF (LM)     */
            agc_alarm(ALARM_GLOCK);

            return nogo();
        case AGC_KALC_SMALL:
            return small_maneuver();
        case AGC_KALC_NORMAL:
        case AGC_KALC_LARGE:
            return wcalc(&ip); /* LOCSKIRT         */
        default:
            break;
    }

    return AGC_STATUS_BAD_ARG; /* unreachable: cannot happen */
}

agc_status_t agc_job_kalcman3(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return kalcman3();
}

/* GOMANUR (IMU_MODE_SWITCHING_ROUTINES p.1442 / p.1331): KALCMANU must be
 * free (else POODOO 01210).  Save the user's continuation and priority,
 * then run KALCMAN3 in the user's job.  The caller ends its job when this
 * returns; wake_as is started at its priority when the maneuver ends.    */
agc_status_t agc_imu_gomanur(const agc_job_ctx_t *ctx, agc_job_id_t wake_as)
{
    if (!AGC_ASSERT(ctx != NULL && wake_as != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (attcadr != AGC_JOB_NONE)
    {
        return agc_poodoo(ALARM_TWO_USERS); /* two users        */
    }

    attcadr = wake_as;   /* DXCH ATTCADR     */
    attprio = ctx->prio; /* ATTPRIO          */

    if (!AGC_ASSERT(attprio <= 037))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return kalcman3(); /* POSTJUMP KALCMAN3 */
}

/* UPDTCALL (task, p.417 / p.368): FINDVAC NEWDELHI at PRIO26.           */
agc_status_t agc_task_updtcall(void)
{
    if (!AGC_ASSERT(attcadr != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = agc_exec_findvac(PRIO26, AGC_JOB_NEWDELHI);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return st;
}

/* NEWDELHI (job, p.414 / p.365): stop if the astronaut has interrupted
 * (CM: HOLDFLAG >= 0; LM: not in auto stabilization), else NEWANGL.     */
agc_status_t agc_job_newdelhi(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    bool interrupted;

    if (is_lm())
    {
        interrupted = (agc_imu_isitauto() > 0); /* CCS A: TCF NOGO-2 */
    }
    else
    {
        const agc_csm_rcs_t *rcs = agc_csm_dap_rcs();

        interrupted = (rcs == NULL) || (rcs->holdflag >= 0); /* BZMF NOGO-2 */
    }

    if (interrupted)
    {
        return nogom2();
    }

    if (!AGC_ASSERT(attcadr != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_interp_t ip;

    agc_interp_init(&ip);

    return newangl1(&ip); /* NEWANGL          */
}
