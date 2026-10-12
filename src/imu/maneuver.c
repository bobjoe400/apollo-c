/**********************************************************************
 * FILE NAME: maneuver.c
 *
 * PURPOSE:
 *   KALCMANU: automatic attitude maneuvers for the RCS DAP (GOMANUR,
 *   KALCMAN3, WCALC, NEWDELHI .. MANUSTOP, ENDMANU).
 *
 * AGC SOURCE:
 *   GOMANUR            IMU_MODE_SWITCHING_ROUTINES.agc CM p.1442 / LM p.1331
 *   KALCMAN3, NOGO     ANGLFIND.agc CM pp.399-411 / ATTITUDE_MANEUVER_ROUTINE
 *                      .agc LM pp.351-363 (mathematics in maneuver_calc.c)
 *   WCALC (LOCSKIRT)   GIMBAL_LOCK_AVOIDANCE.agc CM p.412 / LM p.364
 *   NEWDELHI .. ENDMANU KALCMANU_STEERING.agc CM pp.414-419 / LM pp.365-369
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type       I/O  Scale          Description
 *   ------------  ---------  ---  -------------  ----------------------
 *   CPHI..CPSI    uint16_t   I    CDU            agc_erase_thetad()
 *   CDUXD.., DAP  various    O    see DAP        agc_csm_dap_rcs(), LM
 *                                                agc_erase_set_*()
 *   TIME2,TIME1   agc_dp_t   I    cs B28         agc_time_now_cs()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_kalc_compute      KALCMAN3 mathematics (maneuver_calc.c)
 *   agc_kalc_dcmtocdu     DCMTOCDU, agc_kalc_delcomp DELCOMP
 *   agc_exec_findvac      FINDVAC / SPVAC, agc_waitlist_call
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   POODOO 01210 (GOMANUR while KALCMANU is busy); LM ALARM 00401
 *   (TOOBADF, final attitude in gimbal lock).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles.  One maneuver at a time (ATTCADR).
 *
 * NOTES:
 *   A maneuver: GOMANUR saves the user's return (ATTCADR) and priority
 *   and runs KALCMAN3 in the user's job.  KALCMAN3 finds the rotation
 *   axis COF and angle AM taking the present attitude to CPHI, CTHETA,
 *   CPSI, and DEL, the rotation the attitude makes in one second at the
 *   selected rate.  Every second (UPDTCALL -> NEWDELHI) the desired
 *   attitude matrix MIS is advanced by DEL and the DAP is given new
 *   desired CDU angles and the per-0.1 s increments; in the last second
 *   MANUSTOP sets the final angles and ENDMANU / GOODMANU restarts the
 *   user's job at its priority.
 *   CM vs LM differences, all kept (agc_state_vehicle()):
 *   * small maneuvers: CM sets HOLDFLAG -1 and LOADCDUD; LM loads
 *     CDUXD.. from CPHI.. (TOOBADI).  LM also refuses |CPSI| >= 70 deg
 *     (TOOBADF).
 *   * rates: CM ARATE .05/.2/.5/2 deg/s, LM .2/.5/2/10 deg/s; CM rotates
 *     BRATE into control axes (QUADROT, -7.25 deg about X) and forms
 *     BIASTEMP; LM sets CALCMAN3 (NOGIMLOC).
 *   * DAP interface: CM writes WBODY/BIAS/DELCDUX (DP)/HOLDFLAG/RCSFLAGS
 *     in the CSM DAP; LM writes OMEGAPD/DELPEROR/DELCDUX (SP, negative
 *     increment)/CDUXD in the LM DAP.
 *   * NEWDELHI: CM aborts when HOLDFLAG >= 0 (astronaut took over), LM
 *     when ISITAUTO says the DAP is not in auto.
 *   * TIMECHK return layout (same timing result).
 *   TM is kept as the two words MANUSTAT's DAS leaves (review-imu): the
 *   minor word can be negative under a positive major word, and TIMECHK
 *   branches on the major word first.
 *
 * ALGORITHM:
 *   As the AGC listing; see each function prolog.
 **********************************************************************/
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
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "internal.h"

#define PRIO26          ((agc_prio_t) 026)         /* NEWDELHI                   */
#define ALARM_GLOCK     ((agc_alarm_code_t) 00401) /* TOOBADF (LM) */
#define ALARM_TWO_USERS ((agc_alarm_code_t) 01210) /* GOMANUR busy */
#define ONESEC_CS       100                        /* 1SEC / ONESEK +1           */
#define TIME1_MOD       040000                     /* TIME1 is 14 bits           */
#define TIME_MOD        0x10000000L                /* TIME2,TIME1 wrap at 2^28   */
#define DT_TENTH        1638                       /* QUADROT / DT/TAU: DEC .1   */
#define CDU_MASK        077777u                    /* 15-bit CDU word            */
#define RATE_COUNT      4                          /* ARATE entries (RATEINDX/2) */

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
                                      {{26843546, 0, 0}},        /* .1,      0,       0           */
                                      {{0, 26628797, -3387655}}, /* 0,  .099200, -.012620         */
                                      {{0, 3387655, 26628797}}   /* 0,  .012620,  .099200         */
                                  }};

/* KALCMANU erasables (module-private; EBANK BCDU).                       */
static uint16_t     bcdu[3];     /* BCDU: initial / previous desired CDUs   */
static uint16_t     ncdu[3];     /* NCDU: new desired CDU angles            */
static agc_mat_t    mis;         /* MIS: S/C -> SM now, scale 2             */
static agc_mat_t    del;         /* DEL (CM) / KEL (LM): 1-s rotation, 1    */
static agc_vec_t    cof;         /* COF: maneuver axis, B1                  */
static agc_dp_t     am;          /* AM: maneuver angle, rev 2^0             */
static agc_vec_t    brate;       /* BRATE: rates, 450 deg/s 2^0 (LM 45)     */
static agc_vec_t    biastemp;    /* BIASTEMP (CM): error biases, 2^0        */
static agc_dp_t     tm;          /* TM: maneuver time / end time, cs 2^28   */
static int          tm_hi;       /* TM after MANUSTAT, major word, cs 2^14  */
static int          tm_lo;       /* TM after MANUSTAT, minor word, cs B0    */
static int          nextime;     /* NEXTIME: TIME1 of the next update, cs   */
static agc_sp_t     rateindx_lm; /* RATEINDX (LM): 0, 2, 4, 6, B0           */
static agc_job_id_t attcadr;     /* ATTCADR: user's continuation, 0 = free  */
static agc_prio_t   attprio;     /* ATTPRIO: user's priority                */

/**********************************************************************
 * AGC: none (vehicle test) -- the LM rope
 *
 * RETURNS: bool -- true for Luminary
 **********************************************************************/
static bool is_lm(void)
{
    agc_vehicle_t v = agc_state_vehicle(); /* vehicle */

    return (v == AGC_VEHICLE_LM);
}

/**********************************************************************
 * AGC: none (CDU pattern) -- signed count at 180 deg (CSM DAP CDUXD)
 *
 * ARGUMENTS:
 *   p  uint16_t  I    CDU pattern
 *
 * RETURNS: agc_sp_t -- signed count, B0
 **********************************************************************/
static agc_sp_t cdu_signed(uint16_t p)
{
    int v = (p & 040000u) ? (int) p - 0100000 : (int) p; /* sign bit */

    return ((agc_sp_t) v);
}

/**********************************************************************
 * AGC: none (TS CDUXD) -- one desired CDU angle in the DAP
 *
 * LM: CDUXD.. words; CM: the high word of the DP desired angle.
 *
 * ARGUMENTS:
 *   axis  int       I    0 X, 1 Y, 2 Z
 *   p     uint16_t  I    desired CDU pattern
 **********************************************************************/
static void set_cdud(int axis, uint16_t p)
{
    if (is_lm())
    {
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
    else
    {
        /* "TS CDUXD" (CM): the high word of the DP desired angle.               */
        agc_csm_rcs_t *rcs = agc_csm_dap_rcs(); /* CSM RCS DAP erasables */

        rcs->cdud[axis] = (agc_dp_t) cdu_signed(p) * 16384 + agc_dp_low(rcs->cdud[axis]);
    }
} /* set_cdud */

/**********************************************************************
 * AGC: none (accessor) -- CPHI, CTHETA, CPSI
 *
 * ARGUMENTS:
 *   axis  int  I    0 X, 1 Y, 2 Z
 *
 * RETURNS: uint16_t -- final CDU pattern
 **********************************************************************/
static uint16_t cphi(int axis)
{
    uint16_t v = agc_erase_thetad(axis); /* vehicle */

    return (v);
}

/**********************************************************************
 * AGC: none (EXTEND, MSU) -- a - b of two CDU patterns
 *
 * ARGUMENTS:
 *   a  uint16_t  I    minuend, CDU
 *   b  uint16_t  I    subtrahend, CDU
 *
 * RETURNS: int -- a - b, CDU counts
 **********************************************************************/
static int msu(uint16_t a, uint16_t b)
{
    int d = (int) ((unsigned int) (a - b) & CDU_MASK); /* CDU difference, B0 */

    d = (d >= 040000) ? d - 0100000 : d; /* 15-bit sign */

    return (d);
}

/**********************************************************************
 * AGC: none (TC WAITLIST) -- schedule a KALCMANU task
 *
 * ARGUMENTS:
 *   dt    uint16_t       I    delay, cs
 *   task  agc_task_id_t  I    task to start
 *
 * RETURNS: agc_status_t -- OK, FULL or BAD_ARG
 **********************************************************************/
static agc_status_t waitlist(uint16_t dt, agc_task_id_t task)
{
    if (!AGC_ASSERT(dt > 0u && dt <= AGC_WAITLIST_MAX_DELAY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_waitlist_call(dt, task); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* ---- RATEINDX ------------------------------------------------------------ */

/**********************************************************************
 * AGC: none (accessor) -- RATEINDX (CM: kept with the DAP, S41.2)
 *
 * ARGUMENTS:
 *   idx  agc_sp_t  I    RATEINDX 0, 2, 4, 6
 *
 * RETURNS: agc_status_t -- OK or BAD_ARG
 **********************************************************************/
agc_status_t agc_kalc_set_rateindx(agc_sp_t idx)
{
    if (!AGC_ASSERT(idx >= 0 && idx <= 6 && (idx & 1) == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (is_lm())
    {
        rateindx_lm = idx;
    }
    else
    {
        agc_csm_rcs_t *rcs = agc_csm_dap_rcs(); /* CSM RCS DAP erasables */

        if (!AGC_ASSERT(rcs != NULL))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        rcs->ratindx = idx; /* CM: kept with the DAP (S41.2)    */
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (accessor) -- RATEINDX
 *
 * RETURNS: agc_sp_t -- RATEINDX 0, 2, 4, 6
 **********************************************************************/
agc_sp_t agc_kalc_rateindx(void)
{
    agc_sp_t idx = rateindx_lm; /* RATEINDX, B0 */

    if (!is_lm())
    {
        const agc_csm_rcs_t *rcs = agc_csm_dap_rcs(); /* CSM RCS DAP erasables */

        idx = (rcs != NULL) ? rcs->ratindx : 0; /* CM: DAP word */
    }

    if (!AGC_ASSERT(idx >= 0 && idx <= 6))
    {
        idx = 0;
    }

    return (idx);
}

/**********************************************************************
 * GOMANUR (IMU_MODE_SWITCHING_ROUTINES.agc p.1442) -- ATTCADR nonzero
 *
 * RETURNS: bool -- true while a maneuver is in progress
 **********************************************************************/
bool agc_kalc_active(void)
{
    return (attcadr != AGC_JOB_NONE);
}

/**********************************************************************
 * AGC: none (ATTSTALL view) -- KALCMANU has no failure return
 *
 * ENDMANU / GOODMANU always resume the user (the AGC's "all returns
 * are now made via GOODEND").
 *
 * RETURNS: agc_imu_stall_result_t -- SLEEPING while active, else GOOD
 **********************************************************************/
agc_imu_stall_result_t agc_kalc_outcome(void)
{
    /* KALCMANU has no failure return: ENDMANU / GOODMANU always resume
     * the user (the AGC's "all returns are now made via GOODEND").       */
    return (agc_kalc_active() ? AGC_IMU_STALL_SLEEPING : AGC_IMU_STALL_GOOD); /* ATTCADR */
}

/* ======================================================================= */
/* DAP interface (KALCMANU_STEERING CM pp.418-419; LM p.369, DAP)          */
/* ======================================================================= */

/**********************************************************************
 * STOPYZ (KALCMANU_STEERING.agc p.418 CM) -- zero pitch and yaw
 * increments, rates, biases
 *
 * ARGUMENTS:
 *   rcs  agc_csm_rcs_t *  I/O  CSM RCS DAP erasables
 **********************************************************************/
static void csm_stopyz(agc_csm_rcs_t *rcs)
{
    for (int i = 1; i < 3; i++)
    {
        rcs->delcdu[i] = 0;
        rcs->wbody[i]  = 0;
        rcs->bias[i]   = 0;
    }
}

/**********************************************************************
 * STOPRATE (KALCMANU_STEERING.agc p.418 CM; LM DAP) -- stop the
 * maneuver rates
 *
 * CM zeroes roll then falls into STOPYZ and resets the high-rate flag
 * (RCSFLAGS bit 15); LM calls the DAP's STOPRATE.
 **********************************************************************/
void agc_kalc_stoprate(void)
{
    if (is_lm())
    {
        agc_lm_dap_stoprate();
    }
    else
    {
        agc_csm_rcs_t    *rcs = agc_csm_dap_rcs(); /* CSM RCS DAP erasables */
        agc_csm_dapcom_t *com = agc_csm_dap_com(); /* CSM DAP common erasables */

        rcs->delcdu[0] = 0;
        rcs->wbody[0]  = 0;
        rcs->bias[0]   = 0;

        com->rcsflags = (uint16_t) (com->rcsflags & (~(unsigned int) AGC_CSM_RCSF_HIRATE & 077777u));
        csm_stopyz(rcs);
    }
}

/**********************************************************************
 * ZEROERROR (KALCMANU_STEERING.agc p.419 CM; LM ZATTEROR) -- CDUXD.. =
 * CDUX..
 *
 * RETURNS: agc_status_t -- OK
 **********************************************************************/
agc_status_t agc_kalc_zeroerror(void)
{
    if (is_lm())
    {
        agc_lm_dap_zatteror();
    }
    else
    {
        set_cdud(0, agc_erase_cdux());
        set_cdud(1, agc_erase_cduy());
        set_cdud(2, agc_erase_cduz());

        if (!AGC_ASSERT(agc_erase_cduz() <= CDU_MASK))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        if (!AGC_ASSERT(agc_erase_cdux() <= CDU_MASK))
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * LOADCDUD / LOADYZ (KALCMANU_STEERING.agc p.419 CM) -- desired angles
 * = CPHI..
 *
 * ARGUMENTS:
 *   first_axis  int  I    0 LOADCDUD (X, Y, Z), 1 LOADYZ (Y, Z)
 **********************************************************************/
static void loadcdud(int first_axis)
{
    for (int i = first_axis; i < 3; i++)
    {
        set_cdud(i, cphi(i));
    }
}

/* ======================================================================= */
/* Termination: NOGO, MANUSTOP, ENDMANU / GOODMANU                         */
/* ======================================================================= */

/**********************************************************************
 * NOGO (ANGLFIND.agc p.411 CM; LM p.363) -- stop any rate, ENDMANU in
 * 20 ms
 *
 * RETURNS: agc_status_t -- waitlist status (ENDOFJOB)
 **********************************************************************/
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

        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = waitlist(2u, AGC_TASK_ENDMANU); /* status of the call */

    agc_relint();

    return (st); /* ENDOFJOB        */
}

/**********************************************************************
 * NOGO -2 / NOGOM2 (ANGLFIND.agc p.411) -- interrupted: zero the error,
 * NOGO
 *
 * RETURNS: agc_status_t -- status of NOGO
 **********************************************************************/
static agc_status_t nogo_zero_error(void)
{
    agc_inhint();
    agc_status_t st = agc_kalc_zeroerror(); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    agc_relint();

    return (nogo());
}

/**********************************************************************
 * MANUSTOP / ENDROLL (KALCMANU_STEERING.agc p.417 CM; LM p.368) --
 * task: final angles, rates off
 *
 * RETURNS: agc_status_t -- status of ENDMANU
 **********************************************************************/
agc_status_t agc_task_manustop(void)
{
    if (!is_lm())
    {
        agc_csm_rcs_t *rcs = agc_csm_dap_rcs(); /* CSM RCS DAP erasables */

        if (!AGC_ASSERT(rcs != NULL))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        csm_stopyz(rcs);      /* STOPYZ          */
        loadcdud(1);          /* LOADYZ          */
        set_cdud(0, cphi(0)); /* ENDROLL         */
        agc_kalc_stoprate();

        return (agc_task_endmanu());
    }

    agc_erase_set_delcduz(0); /* DELDCDU2        */
    agc_erase_set_omegard(0);
    agc_erase_set_delreror(0);
    agc_erase_set_delcduy(0); /* DELDCDU1        */
    agc_erase_set_omegaqd(0);
    agc_erase_set_delqeror(0);
    loadcdud(1);              /* CDUZD, CDUYD    */
    set_cdud(0, cphi(0));     /* ENDROLL         */
    agc_erase_set_omegapd(0);
    agc_erase_set_delcdux(0); /* DELDCDU         */
    agc_erase_set_delperor(0);

    return (agc_task_endmanu()); /* GOODMANU        */
}

/**********************************************************************
 * ENDMANU (KALCMANU_STEERING.agc p.418 CM) / GOODMANU (LM) -- task:
 * resume the user
 *
 * Restore the user's priority, free KALCMANU (ATTCADR = 0), SPVAC.
 *
 * RETURNS: agc_status_t -- status of SPVAC
 **********************************************************************/
agc_status_t agc_task_endmanu(void)
{
    agc_job_id_t job  = attcadr; /* ATTCADR */
    agc_prio_t   prio = attprio; /* NEWPRIO         */

    attcadr = AGC_JOB_NONE;

    if (!AGC_ASSERT(job != AGC_JOB_NONE))
    {
        return (AGC_STATUS_OK); /* nobody waiting  */
    }

    agc_status_t st = agc_exec_findvac(prio, job); /* SPVAC           */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* ======================================================================= */
/* Steering updates (KALCMANU_STEERING CM pp.414-417, LM pp.365-368)       */
/* ======================================================================= */

/**********************************************************************
 * TIMECHK / MANUOFF / 2NDRETRN (KALCMANU_STEERING.agc p.415 CM; LM
 * p.366) -- time left
 *
 * TM - TIME2,TIME1 word by word as the DAS leaves it.  More than a
 * second left: 0 (CONTMANU).  Otherwise the MANUSTOP delay: the rest of
 * the last second (MANUOFF) or 1 cs (2NDRETRN).
 *
 * RETURNS: uint16_t -- 0, or the MANUSTOP delay in cs
 **********************************************************************/
static uint16_t timechk(void)
{
    long int now = (long int) agc_time_now_cs(); /* TIME2,TIME1, cs */

    /* DCS TIME2, DCA TM, DAS TTEMP: word by word from the TM words
     * MANUSTAT left (their signs may disagree), low-word carry included. */
    long int lo    = (long int) tm_lo - (now & 037777L); /* TTEMP +1, cs        */
    long int carry = 0;                                  /* DAS low-word carry  */

    if (lo < -037777L)
    {
        lo    += 040000L;
        carry = -1;
    }
    else if (lo > 037777L)
    {
        lo    -= 040000L;
        carry = 1;
    }

    long int hi = (long int) tm_hi - (now >> 14) + carry; /* TTEMP, cs 2^14     */

    if (hi > 0 || (hi == 0 && lo > 0))
    {
        return (0u); /* TC Q: continue  */
    }

    if (hi < 0)
    {
        return (1u); /* 2NDRETRN        */
    }

    /* lo = +0: MANUOFF with A = 0.  lo < 0: CCS leaves |lo| - 1, COM
     * makes it -(|lo| - 1); then AD 1SEC and BZMF 2NDRETRN.             */
    long int a = ONESEC_CS + ((lo == 0) ? 0 : lo + 1); /* MANUOFF, cs */

    return ((a <= 0) ? 1u : (uint16_t) a); /* BZMF 2NDRETRN */
} /* timechk */

/**********************************************************************
 * INCRDCDU / TMANUCHK / MANUSTAL (KALCMANU_STEERING.agc pp.414-415 CM;
 * LM pp.365-366)
 *
 * DAP increments per 0.1 s (CM DELCDUX DP = (NCDU - BCDU) .1, LM DELDCDU
 * SP = (BCDU - NCDU) .1), CDUXD = old BCDU, BCDU = NCDU; then the time
 * check: CONTMANU or MANUSTOP.
 *
 * RETURNS: agc_status_t -- status of CONTMANU or of the MANUSTOP call
 **********************************************************************/
static agc_status_t incrdcdu_tmanuchk(void)
{
    for (int i = 2; i >= 0; i--)
    { /* CAF TWO: Z, Y, X */
        /* INCRDCDU (one axis): CM DELCDUX (DP) = (NCDU - BCDU) x .1;
         * LM DELDCDU (SP) = (BCDU - NCDU) x .1.  CDUXD = old BCDU, BCDU = NCDU. */
        if (!AGC_ASSERT(i >= 0 && i < 3))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        uint16_t old = bcdu[i]; /* previous desired CDU (BCDU) */

        if (is_lm())
        {
            int d = msu(bcdu[i], ncdu[i]); /* CDU difference, B0 */

            d            = (d < AGC_SP_MIN) ? AGC_SP_MIN : d;               /* -16384 */
            agc_sp_t inc = agc_dp_high(agc_sp_mul((agc_sp_t) d, DT_TENTH)); /* DELDCDU, CDU B0 */

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
            int d = msu(ncdu[i], bcdu[i]); /* CDU difference, B0 */

            d                  = (d < AGC_SP_MIN) ? AGC_SP_MIN : d; /* -16384 */
            agc_csm_rcs_t *rcs = agc_csm_dap_rcs();                 /* CSM RCS DAP erasables */

            if (AGC_ASSERT(rcs != NULL))
            {
                rcs->delcdu[i] = agc_sp_mul((agc_sp_t) d, DT_TENTH); /* MP QUADROT */
            }
        }

        bcdu[i] = ncdu[i];
        set_cdud(i, old);
    } /* end for */

    if (!AGC_ASSERT(bcdu[0] == ncdu[0] && bcdu[1] == ncdu[1]))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(bcdu[2] == ncdu[2]))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_interrupts_inhibited())
    {
        agc_relint();
    }

    uint16_t dt = timechk(); /* delay, cs */

    if (dt == 0u)
    {
        /**********************************************************************
         * CONTMANU / MANUCALL (KALCMANU_STEERING.agc p.416 CM; LM p.367) --
         * UPDTCALL at NEXTIME
         *
         * DEVIATION: NEXTIME == TIME1 would give the AGC a zero WAITLIST delay;
         * 1 cs is used instead.
         *
         * RETURNS: agc_status_t -- waitlist status (ENDOFJOB)
         **********************************************************************/
        agc_inhint();

        int diff        = nextime - (int) agc_erase_time1(); /* NEXTIME - TIME1, cs */
        int dt_contmanu = diff;                              /* CCS A, AD ONE / AD NEGMAX, COM: cs to NEXTIME */

        if (diff == 0)
        {
            dt_contmanu = 1; /* see DEVIATION below */
        }
        else if (diff < 0)
        {
            dt_contmanu = TIME1_MOD + diff; /* TIME1 overflow */
        }
        else
        {
            dt_contmanu = diff;
        }

        /* DEVIATION: NEXTIME == TIME1 gives the AGC a zero WAITLIST delay;
         * one centisecond is used instead.                                   */
        if (!AGC_ASSERT(dt_contmanu > 0 && dt_contmanu < TIME1_MOD))
        {
            agc_relint();

            return (AGC_STATUS_BAD_ARG);
        }

        agc_status_t st_contmanu = waitlist((uint16_t) dt_contmanu, AGC_TASK_UPDTCALL); /* status of the call */

        agc_relint();
        nextime = (nextime + ONESEC_CS) & (TIME1_MOD - 1); /* ADS NEXTIME    */

        return (st_contmanu); /* ENDOFJOB       */
    }

    agc_inhint();                                      /* MANUSTAL         */
    agc_status_t st = waitlist(dt, AGC_TASK_MANUSTOP); /* status of the call */

    agc_relint();

    return (st); /* ENDOFJOB         */
} /* incrdcdu_tmanuchk */

/**********************************************************************
 * NEWANGL +1 (KALCMANU_STEERING.agc p.414 CM; LM p.365) -- MIS = MIS
 * DEL, NCDU, increments
 *
 * On the first pass (CALCMAN2 set) MANUSTAT.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state
 *
 * RETURNS: agc_status_t -- status of INCRDCDU / TMANUCHK
 **********************************************************************/
static agc_status_t steering_update(agc_interp_t *ip)
{
    mis             = agc_mat_mul(ip, &mis, &del);              /* MXM3             */
    agc_vec_t    ang;                                           /* PHI, THETA, PSI, rev 2^0 */
    agc_dp_t     cmga;                                          /* COS(MGA), 2^0 */
    agc_status_t st = agc_kalc_dcmtocdu(ip, &mis, &ang, &cmga); /* status of the call */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_rtb_v1sto2s(ang, ncdu); /* NCDU             */

    if (agc_flag_test(AGC_FLAG_CALCMAN2))
    { /* BONCLR CALCMAN2  */
        agc_flag_clear(AGC_FLAG_CALCMAN2);

        /**********************************************************************
         * MANUSTAT (KALCMANU_STEERING.agc p.416 CM; LM p.367) -- start the
         * maneuver
         *
         * TM = TM + TIME2,TIME1 then (-0, -1SEC) by DAS (the words are kept for
         * TIMECHK), the vehicle part, NEXTIME = TIME1 + 1 s.
         **********************************************************************/
        /* DCA TIME2, DAS TM; CS 1SEC, TS L, CS ZERO, DAS TM: the second DAS
         * adds -1SEC to the minor word only, so TM's words can disagree in
         * sign (e.g. 5, -30); TIMECHK reads them as they are.                */
        long int now   = (long int) agc_time_now_cs() % TIME_MOD;     /* TIME2,TIME1, cs */
        long int lo    = ((long int) tm & 037777L) + (now & 037777L); /* cs B0      */
        long int carry = (lo > 037777L) ? 1 : 0;                      /* low-word overflow           */

        lo    -= carry * 040000L;
        tm_hi = (int) ((((long int) tm >> 14) + (now >> 14) + carry) & 037777L);
        tm_lo = (int) (lo - ONESEC_CS);
        tm    = (agc_dp_t) (((long int) tm_hi << 14) + tm_lo); /* the value      */
        agc_inhint();

        if (is_lm())
        {
            /**********************************************************************
             * RATEBIAS (KALCMANU_STEERING.agc p.367 LM) -- OMEGAxD = BRATE,
             * DELxEROR = -|w| w / 16 / 1JACC
             **********************************************************************/
            {
                for (int i = 2; i >= 0; i--)
                {
                    agc_sp_t w    = agc_dp_high(brate.c[i]); /* BRATE high word, B0 */
                    agc_sp_t jacc = agc_erase_1jaccr();      /* 1/JACC for this axis, B0 */

                    if (i == 0)
                    {
                        jacc = agc_erase_1jacc();
                    }
                    else if (i == 1)
                    {
                        jacc = agc_erase_1jaccq();
                    }
                    else
                    {
                        jacc = agc_erase_1jaccr();
                    }

                    agc_sp_t a   = (agc_sp_t) ((w > 0) ? -w : w);           /* -|W|, B0          */
                    agc_sp_t q   = agc_dp_high(agc_sp_mul(a, BIASCALE_LM)); /* |W|/16, B0 */
                    agc_dp_t p   = agc_sp_mul(q, w);                        /* MP BRATE: W|W|/16, B14 */
                    agc_dp_t lag = 0;                                       /* DELxEROR, B0 */

                    if (AGC_ASSERT(jacc != 0 && agc_dp_high(p) < (jacc < 0 ? -jacc : jacc))) /* DV no overflow */
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
                } /* end for */
            }
        }
        else
        {
            /* MANUSTAT (CM p.416): completion time, enable auto maneuvers in the DAP
             * (HOLDFLAG -1), rates, biases and the high-rate flag.                   */
            agc_csm_rcs_t    *rcs = agc_csm_dap_rcs(); /* CSM RCS DAP erasables */
            agc_csm_dapcom_t *com = agc_csm_dap_com(); /* CSM DAP common erasables */

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

        nextime = ((int) agc_erase_time1() + ONESEC_CS) & (TIME1_MOD - 1);

        if (!AGC_ASSERT(tm_hi >= 0 && tm_lo >= -ONESEC_CS))
        {
            agc_relint();
        }
    }

    return (incrdcdu_tmanuchk());
}

/* ======================================================================= */
/* Entry points                                                            */
/* ======================================================================= */

/**********************************************************************
 * KALCMAN3 (ANGLFIND.agc p.399 CM; ATTITUDE_MANEUVER_ROUTINE.agc p.351
 * LM)
 *
 * Set up and start the maneuver to CPHI, CTHETA, CPSI from the present
 * CDUs (READCDUK).  Small (< .25 deg): attitude hold on the commanded
 * angles; LM |CPSI| >= 70 deg: ALARM 00401 (TOOBADF).
 *
 * RETURNS: agc_status_t -- status of NOGO or WCALC
 **********************************************************************/
static agc_status_t kalcmanu_start(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_inhint();                                                   /* READCDUK         */
    bcdu[0]                 = agc_erase_cdux();
    bcdu[1]                 = agc_erase_cduy();
    bcdu[2]                 = agc_erase_cduz();
    agc_relint();
    const uint16_t   fin[3] = {cphi(0), cphi(1), cphi(2)};
    agc_kalc_setup_t k;                                             /* KALCMAN3 results */
    agc_status_t     st     = agc_kalc_compute(&ip, bcdu, fin, &k); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    mis = k.mis;
    cof = k.cof;
    am  = k.am;

    switch (k.kcase)
    {
        case AGC_KALC_TOOBAD: /* TOOBADF (LM)     */
            agc_alarm(ALARM_GLOCK);

            return (nogo());
        case AGC_KALC_SMALL:
            /* Small maneuver (< .25 deg): go directly into attitude hold about the
             * commanded angles.  CM: HOLDFLAG = -1, LOADCDUD.  LM: CDUXD.. = CPHI..
             * (TLOAD CPHI, STCALL CDUXD, TOOBADI).                                  */
        {
            if (!is_lm())
            {
                agc_csm_rcs_t *rcs = agc_csm_dap_rcs(); /* CSM RCS DAP erasables */

                if (!AGC_ASSERT(rcs != NULL))
                {
                    return (AGC_STATUS_BAD_ARG);
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
                return (AGC_STATUS_BAD_ARG);
            }

            return (nogo());
        }
        case AGC_KALC_NORMAL:
        case AGC_KALC_LARGE:
            /* LOCSKIRT         */
            /**********************************************************************
             * WCALC = LOCSKIRT (GIMBAL_LOCK_AVOIDANCE.agc p.412 CM; LM NOGIMLOC
             * p.364)
             *
             * DEL for one second at the selected rate (ARATE SR4), BRATE, TM = AM
             * DMP ANGLTIME DDV ARATE SR 5, (CM) BIASTEMP; SETGO CALCMAN2 NEWANGL +1.
             *
             * ARGUMENTS:
             *   ip  agc_interp_t *  I/O  interpreter state
             *
             * RETURNS: agc_status_t -- status of NEWANGL
             **********************************************************************/
        {
            int idx = agc_kalc_rateindx() / 2; /* ARATE index */

            if (!AGC_ASSERT(idx >= 0 && idx < RATE_COUNT))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            bool     lm = is_lm();                            /* vehicle is the LM */
            agc_dp_t ar = lm ? arate_lm[idx] : arate_cm[idx]; /* ARATE,1: rev 2^0 */

            if (lm)
            {
                imu_vflag_set(IMU_VF_CALCMAN3); /* NOGIMLOC         */
            }

            agc_status_t st_wcalc = agc_kalc_delcomp(&ip, cof, agc_dp_shr(ar, 4), &del); /* status of the call */

            if (!AGC_ASSERT(st_wcalc == AGC_STATUS_OK))
            {
                return (st_wcalc);
            }

            agc_vec_t b = agc_vec_scale(&ip, cof, ar); /* BRATE in S/C axes, 2^0 */

            brate = lm ? b : agc_mat_vec(&ip, &quadrot, b);                             /* MXV QUADROT (CM) */
            tm    = agc_dp_shr(agc_dp_div(&ip, agc_dp_mul_trunc(am, ANGLTIME), ar), 5); /* AM DMP ANGLTIME (truncating), DDV* ARATE, SR 5 */

            if (!lm)
            {
                biastemp = agc_vec_scale(&ip, brate, BIASCALE);
            }

            agc_flag_set(AGC_FLAG_CALCMAN2); /* SETGO CALCMAN2   */

            return (steering_update(&ip));
        }
        default:
            break;
    } /* end switch */

    return (AGC_STATUS_BAD_ARG); /* unreachable: cannot happen */
} /* kalcmanu_start */

/**********************************************************************
 * KALCMAN3 (ANGLFIND.agc p.399) -- as a job
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I    the job
 *
 * RETURNS: agc_status_t -- status of KALCMAN3
 **********************************************************************/
agc_status_t agc_job_kalcmanu_start(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (kalcmanu_start());
}

/**********************************************************************
 * GOMANUR (IMU_MODE_SWITCHING_ROUTINES.agc p.1442 CM; p.1331 LM)
 *
 * KALCMANU must be free (else POODOO 01210).  Save the user's
 * continuation and priority, then KALCMAN3 in the user's job.  wake_as is
 * started at the user's priority when the maneuver ends.
 *
 * ARGUMENTS:
 *   ctx      const agc_job_ctx_t *  I    the user's job
 *   wake_as  agc_job_id_t           I    the user's continuation
 *                                        (ATTCADR)
 *
 * RETURNS: agc_status_t -- status of KALCMAN3, or POODOO
 **********************************************************************/
agc_status_t agc_imu_gomanur(const agc_job_ctx_t *ctx, agc_job_id_t wake_as)
{
    if (attcadr != AGC_JOB_NONE)
    {
        return (agc_poodoo(ALARM_TWO_USERS)); /* two users        */
    }

    attcadr = wake_as;   /* DXCH ATTCADR     */
    attprio = ctx->prio; /* ATTPRIO          */

    if (!AGC_ASSERT(attprio <= 037))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (kalcmanu_start()); /* POSTJUMP KALCMAN3 */
}

/**********************************************************************
 * UPDTCALL (KALCMANU_STEERING.agc p.417 CM; LM p.368) -- task: FINDVAC
 * NEWDELHI at PRIO26
 *
 * RETURNS: agc_status_t -- status of FINDVAC
 **********************************************************************/
agc_status_t agc_task_updtcall(void)
{
    if (!AGC_ASSERT(attcadr != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_exec_findvac(PRIO26, AGC_JOB_NEWDELHI); /* status of the call */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * NEWDELHI (KALCMANU_STEERING.agc p.414 CM; LM p.365) -- job: next
 * steering update
 *
 * Stop if the astronaut has interrupted (CM: HOLDFLAG >= 0; LM: not in
 * auto stabilization), else NEWANGL.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- status of NOGO -2 or NEWANGL
 **********************************************************************/
agc_status_t agc_job_newdelhi(void)
{
    bool stopped; /* astronaut took over */

    if (is_lm())
    {
        stopped = (agc_imu_isitauto() > 0); /* CCS A: TCF NOGO-2 */
    }
    else
    {
        const agc_csm_rcs_t *rcs = agc_csm_dap_rcs(); /* CSM RCS DAP erasables */

        stopped = (rcs == NULL) || (rcs->holdflag >= 0); /* BZMF NOGO-2 */
    }

    if (stopped)
    {
        return (nogo_zero_error());
    }

    if (!AGC_ASSERT(attcadr != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);

    return (steering_update(&ip)); /* NEWANGL          */
}
