/*
 * attitude.c -- phase 2 of the RCS-CSM autopilot: attitude hold,
 * automatic steering, minimum impulse, phase-plane switching logic and
 * jet on-time calculation.
 * AUTOMATIC_MANEUVERS.agc (Comanche055 pp. 1025-1036): AHFNOROT, AUTOCONT,
 * FREECONT, ATTHOLD, HOLDFUNC, GRABANG, ENDDAMP, JETS, JLOOP, INHOLD,
 * POSVEL, NEGVEL, J6., J7, J8, NJ22, NJ23, J18, J20, J21, J22, J23, J24,
 * JN, JP, JTIME, TAUNORM, DOJET, ZEROCMDS, T6PROG; T6PROGM (p. 1020).
 *
 * Attitude errors (p. 1027):
 *     AK = AMGB (CDU - THETAD) + BIAS
 * the biases (KALCMANU lead, 1-7 deg) are added only during automatic
 * maneuvers. Switching logic (p. 1029, FIG. 1): dead zone ADB, flat region
 * AF = .044 deg, switch lines WL+H = 0.5, WL = 0.4, WL-H = 0.3 deg/s,
 * hysteresis slope SLOPE2 0.8 deg/s/deg, switch-line SLOPE 0.6 deg/s/deg.
 * Jet on-time T = J/M (DELTA W), scaled 16384/1600 s (1 bit = 1/1600 s).
 */
#include "agc/agc_csm_dap_internal.h"

#define MANROT     ((uint16_t)000077)
#define DAMPBITS   ((uint16_t)016000) /* OCT16000                          */
#define AF         ((agc_sp_t)4)      /* FOUR: flat region .044 deg (180)  */
#define WLH_SLOP   ((agc_sp_t)76)     /* 00114 .00463 WL+H/SLOPE .83333 deg */
#define WLMH_SLP   ((agc_sp_t)45)     /* 00055 .00277 WL-H/SLOPE .5 deg     */
#define WLH        ((agc_dp_t)298262) /* 00022 06426 2DEC .0011111111 0.5 deg/s */
#define WLMH       ((agc_dp_t)178957) /* 00012 35415 2DEC .0006666666 0.3 deg/s */
#define WL         ((agc_dp_t)238609) /* 00016 22021 2DEC .0008888888 0.4 deg/s */
#define SLOPE2     ((agc_sp_t)5243)   /* 12173 .32 = 0.8 deg/s/deg, 450/180 */
#define BIT11      ((agc_sp_t)02000)  /* JTIME second SMALLMP              */

/* MINTAU (p. 1026): minimum impulse on-times, 1/1600 s.                  */
static const agc_sp_t mintau[4] = { 0, 23, -23, 0 };   /* 0, +-14 ms, 0 */

/* Forced firing masks (OCT01400, OCT00060, OCT00300) and rate damping
 * bits (BIT13, BIT12, BIT11) by SPNDX: roll, pitch, yaw.                */
static const uint16_t forcemask[AGC_CSM_AXES] = { 001400, 000060, 000300 };
static const uint16_t dampbit[AGC_CSM_AXES]   = { 010000, 004000, 002000 };

typedef enum pp_region {         /* outcome of the switching logic       */
    PP_J18 = 0, PP_J20, PP_J21, PP_J22, PP_J23, PP_J24
} pp_region_t;

typedef struct pp_point {        /* one axis in the phase plane          */
    agc_dp_t edot;               /* EDOT = ADOT - WBODY, 450 deg/s       */
    agc_dp_t edotvel;            /* EDOTVEL = |EDOT| by sign of EDOT     */
    agc_sp_t aerr;               /* AERR = ERROR, 180 deg                */
    agc_sp_t aerrvel;            /* AERRVEL = AERR by sign of EDOT       */
    agc_sp_t adbvel;             /* ADBVEL = +-(ADB+AF)                  */
    agc_sp_t adbaf;              /* T5TEMP = ADB+AF                      */
} pp_point_t;

static agc_status_t rcs_freecont(void);
static agc_status_t rcs_holdfunc(void);
static agc_status_t rcs_grabang(void);
static agc_status_t rcs_atthold(void);
static agc_status_t rcs_t6prog(void);
static pp_region_t  rcs_region(const pp_point_t *p);
static bool         rcs_delta_w(int i, const pp_point_t *p, agc_dp_t *kmpac);
static agc_sp_t     rcs_jtime(int i, agc_dp_t kmpac);

/* AHFNOROT, AUTOCONT (p. 1025).                                          */
agc_status_t csm_rcs_ahfnorot(void)
{
    const agc_csm_rcs_t *r = agc_csm_dap_rcs();
    if ((agc_chan_read(AGC_CHAN_CHAN31) & (uint16_t)AGC_CH031_FREE) == 0u) {
        return rcs_freecont();
    }
    if (csm_rcs_rate_init_pending()) {
        return csm_rcs_reinit();     /* recycle to initialise the filter  */
    }
    if ((agc_chan_read(AGC_CHAN_CHAN31) & (uint16_t)AGC_CH031_ATTHOLD) == 0u) {
        return rcs_holdfunc();
    }
    (void)AGC_ASSERT(r->holdflag >= AGC_SP_MIN);
    if (r->holdflag <= 0) {          /* AUTOCONT: HOLDFLAG +0, -0, -     */
        return rcs_atthold();
    }
    return rcs_grabang();
}

/* FREECONT (pp. 1025-1026): minimum impulse control from channel 32,
 * firing only on a new deflection (CHANTEMP holds the last image).       */
static agc_status_t rcs_freecont(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    r->holdflag = 1;                 /* inhibit automatic steering        */
    uint16_t ch32 = agc_chan_read(AGC_CHAN_CHAN32);
    uint16_t bits = (uint16_t)(~(unsigned)ch32 & MANROT & r->chantemp);
    r->chantemp = ch32;              /* LXCH CHANTEMP                     */
    agc_status_t st = csm_rcs_stickchk(bits);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    for (int i = 0; i < AGC_CSM_AXES; i++) {
        int m = (int)r->manndx[i];
        if (!AGC_ASSERT((m >= 0) && (m <= 3))) {
            return AGC_STATUS_BAD_ARG;
        }
        r->tau[i] = mintau[m];       /* TAU, TAU1, TAU2                   */
    }
    return csm_rcs_t6progm();
}

/* HOLDFUNC (p. 1028): +0 holds; anything else grabs new references.      */
static agc_status_t rcs_holdfunc(void)
{
    const agc_csm_rcs_t *r = agc_csm_dap_rcs();
    (void)AGC_ASSERT(r->holdflag >= AGC_SP_MIN);
    if (r->holdflag == 0) {
        return rcs_atthold();
    }
    return rcs_grabang();
}

/* GRABANG, ENDDAMP (p. 1028): zero WBODYs and biases; after rate damping
 * pick up the CDU angles as attitude hold references (HOLDFLAG +0).     */
static agc_status_t rcs_grabang(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    const agc_csm_dapcom_t *c = agc_csm_dap_com();
    for (int i = 0; i < AGC_CSM_AXES; i++) {
        r->wbody[i] = 0;
        r->bias[i] = 0;
    }
    if ((c->rcsflags & DAMPBITS) != 0u) {    /* rate damping not done   */
        for (int i = 0; i < AGC_CSM_AXES; i++) {
            r->error[i] = 0;
        }
        return csm_rcs_jets();
    }
    r->holdflag = 0;                          /* ENDDAMP                 */
    for (int i = 0; i < AGC_CSM_AXES; i++) {
        r->thetad[i] = csm_dap_cdu(i);        /* DCA CDUX; DXCH THETADX  */
    }
    (void)AGC_ASSERT(r->holdflag == 0);
    return rcs_atthold();
}

/* ATTHOLD (pp. 1027-1028): ERROR = AMGB (CDU - THETAD) [+ BIAS].         */
static agc_status_t rcs_atthold(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    agc_sp_t ex = csm_dap_msu(agc_erase_cdux(), r->thetad[0]);
    agc_sp_t t = csm_dap_msu(agc_erase_cduy(), r->thetad[1]);
    r->error[0] = csm_dap_ads(ex, csm_dap_mp_hi(t, r->amgb1));
    r->error[1] = csm_dap_mp_hi(t, r->amgb4);
    r->error[2] = csm_dap_mp_hi(t, r->amgb7);
    t = csm_dap_msu(agc_erase_cduz(), r->thetad[2]);
    r->error[1] = csm_dap_ads(r->error[1], csm_dap_mp_hi(t, r->amgb5));
    r->error[2] = csm_dap_ads(r->error[2], csm_dap_mp_hi(t, r->amgb8));
    if (r->holdflag < 0) {           /* add biases only if automatic      */
        for (int i = 0; i < AGC_CSM_AXES; i++) {
            r->error[i] = csm_dap_ads(r->error[i], r->bias[i]);
        }
    }
    (void)AGC_ASSERT(r->error[0] >= AGC_SP_MIN);
    return csm_rcs_jets();
}

/* JETS, JLOOP (p. 1030): for each axis locate (rate error, attitude
 * error) in the phase plane and compute the jet on-time TAU.             */
agc_status_t csm_rcs_jets(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    pp_point_t p;
    p.adbaf = csm_dap_ads(r->adb, AF);                      /* T5TEMP     */
    for (int i = AGC_CSM_AXES - 1; i >= 0; i--) {
        p.edot = r->adot[i];
        if (r->holdflag != 0) {      /* not INHOLD: rate error from WBODY */
            p.edot = csm_dap_das(p.edot, -r->wbody[i]);     /* ADOT-WBODY */
        }
        p.aerr = r->error[i];        /* AERR = BIAS + AK                  */
        bool pos = (p.edot >= 0);    /* POSVEL / NEGVEL (SIGNCK1)         */
        p.edotvel = pos ? p.edot : -p.edot;
        p.adbvel = pos ? p.adbaf : (agc_sp_t)-p.adbaf;
        p.aerrvel = pos ? p.aerr : (agc_sp_t)-p.aerr;
        agc_dp_t kmpac = 0;
        if (rcs_delta_w(i, &p, &kmpac)) {
            r->tau[i] = rcs_jtime(i, kmpac);                /* DOJET      */
        }
    }
    (void)AGC_ASSERT(p.adbaf > 0);
    return rcs_t6prog();
}

/* J6. .. NJ23 (pp. 1030-1032): which region of FIG. 1 the point is in.  */
static pp_region_t rcs_region(const pp_point_t *p)
{
    const agc_csm_rcs_t *r = agc_csm_dap_rcs();
    (void)AGC_ASSERT(p != NULL);
    int adb = (int)r->adb;
    if ((int)p->aerrvel - adb + WLH_SLOP > 0) {             /* J6.        */
        if ((int)p->aerrvel - (int)p->adbaf > 0) {          /* J7         */
            return ((-WLMH_SLP - (int)p->adbaf + (int)p->aerrvel) <= 0) ? PP_J20 : PP_J21;
        }
        int q = (int)csm_dap_dv(-p->edotvel, r->slope);
        return ((q - (int)p->aerrvel + adb) <= 0) ? PP_J18 : PP_J23;
    }
    if (csm_dap_das(-WLH, p->edotvel) >= 0) {               /* J8         */
        return PP_J22;
    }
    int a = (int)csm_dap_dv(p->edotvel, r->slope) + (int)p->adbaf + (int)p->aerrvel;
    if (a >= 0) {                                           /* NJ22       */
        return PP_J23;
    }
    if (csm_dap_das(-WLMH, p->edotvel) >= 0) {
        return PP_J23;
    }
    a = (int)p->aerrvel + (int)p->adbaf + WLMH_SLP;         /* NJ23       */
    (void)AGC_ASSERT(a >= -3 * AGC_SP_MAX);
    return (a >= 0) ? PP_J24 : PP_J22;
}

/* J18, J20, J21 (JP/JN), J22, J23, J24 (pp. 1032-1034): the desired change
 * in rate DELTA W into KMPAC (450 deg/s). False: J23 without a forced
 * firing, which leaves TAU as it was (DOJET +2).                         */
static bool rcs_delta_w(int i, const pp_point_t *p, agc_dp_t *kmpac)
{
    agc_csm_dapcom_t *c = agc_csm_dap_com();
    pp_region_t reg = rcs_region(p);
    agc_dp_t medot = -p->edot;
    bool fire = true;
    switch (reg) {
    case PP_J23:
        c->rcsflags = (uint16_t)(c->rcsflags & ~(unsigned)dampbit[i]); /* damping over */
        fire = (c->rcsflags & forcemask[i]) != 0u;          /* forced: J18 */
        *kmpac = medot;
        break;
    case PP_J18:
        *kmpac = medot;
        break;
    case PP_J20:   /* (ADBVEL - AERR)(SLOPE2) - EDOT                       */
        *kmpac = csm_dap_das(agc_sp_mul(csm_dap_ads((agc_sp_t)-p->aerr, p->adbvel), SLOPE2), medot);
        break;
    case PP_J24:   /* (-AERR - ADBVEL)(SLOPE2) - EDOT                      */
        *kmpac = csm_dap_das(agc_sp_mul(csm_dap_ads((agc_sp_t)-p->aerr, (agc_sp_t)-p->adbvel), SLOPE2), medot);
        break;
    case PP_J21:   /* EDOT >= 0: JP (-EDOT - WL), else JN (-EDOT + WL)     */
        *kmpac = csm_dap_das(medot, (p->edot >= 0) ? -WL : WL);
        break;
    case PP_J22:   /* EDOT >= 0: JN, else JP                               */
        *kmpac = csm_dap_das(medot, (p->edot >= 0) ? WL : -WL);
        break;
    default:
        (void)AGC_ASSERT(false);
        fire = false;
        break;
    }
    (void)AGC_ASSERT((i >= 0) && (i < AGC_CSM_AXES));
    return fire;
}

/* JTIME, TAUNORM, DOJET (p. 1035): TAU = J/M (DELTA W) for one jet; an
 * upper word left by the two SMALLMPs means the time overflowed: POSMAX
 * or NEGMAX.                                                             */
static agc_sp_t rcs_jtime(int i, agc_dp_t kmpac)
{
    const agc_csm_rcs_t *r = agc_csm_dap_rcs();
    if (!AGC_ASSERT((i >= 0) && (i < AGC_CSM_AXES))) {
        return 0;
    }
    agc_dp_t k = csm_dap_smallmp(kmpac, r->jm[i]);  /* J/M, J/M1, J/M2     */
    k = csm_dap_smallmp(k, BIT11);
    agc_sp_t hi = agc_dp_high(k);
    if (hi > 0) {
        return AGC_SP_MAX;
    }
    if (hi < 0) {
        return AGC_SP_MIN;
    }
    return agc_dp_low(k);                            /* TAUNORM            */
}

/* T6PROGM (p. 1020): zero the errors for manual rotations; T6PROG.       */
agc_status_t csm_rcs_t6progm(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs();
    for (int i = 0; i < AGC_CSM_AXES; i++) {
        r->error[i] = 0;
    }
    (void)AGC_ASSERT(r->error[2] == 0);
    return rcs_t6prog();
}

/* T6PROG (p. 1036): when the rotation commands (TAUs) have been
 * determined, reset T5LOC for phase 3 (JETSLECT). RESUME.                */
static agc_status_t rcs_t6prog(void)
{
    agc_status_t st = agc_rupt_set_t5_task(AGC_TASK_JETSLECT);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;
}
