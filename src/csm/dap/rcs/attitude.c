/**********************************************************************
 * FILE NAME: attitude.c
 *
 * PURPOSE:
 *   Phase 2 of the RCS-CSM autopilot: attitude hold, automatic steering,
 *   minimum impulse, the phase-plane switching logic and the jet on-time
 *   calculation. Labels: AHFNOROT, AUTOCONT, FREECONT, MINTAU, ATTHOLD,
 *   HOLDFUNC, GRABANG, ENDDAMP, JETS, JLOOP, INHOLD, SIGNCK1, POSVEL,
 *   NEGVEL, J6., J7, J8, SIGNCK2, NJ22, SIGNCK3, NJ23, J18, J20, J21,
 *   SIGNCK4, JN, JP, J22, SIGNCK5, J23, J24, JTIME, TAUNORM, DOJET,
 *   T6PROG; T6PROGM.
 *
 * AGC SOURCE:
 *   Comanche055 AUTOMATIC_MANEUVERS.agc pp. 1025-1036;
 *   RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1020 (T6PROGM)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type            I/O  Scale        Description
 *   ------------  --------------  ---  -----------  ------------------
 *   ADOT..ADOT2   agc_dp_words_t  I    450 deg/s    estimated rate
 *   WBODY..       agc_dp_t        I/O  450 deg/s    commanded rate
 *   ERRORX..Z     agc_sp_t        I/O  180 deg      attitude error
 *   BIAS..BIAS2   agc_sp_t        I/O  180 deg      KALCMANU lead
 *   THETADX..Z    uint16_t        I/O  CDU counts   hold references
 *   HOLDFLAG      agc_sp_t        I/O  -            + grab, +0 hold, - auto
 *   ADB, SLOPE    agc_sp_t        I    180 deg, -   deadband, switch line
 *   J/M..J/M2     agc_sp_t        I    (57.3/450)(B24/1600)  inertia/torque
 *   TAU..TAU2     agc_sp_t        O    1/1600 s     jet on-times
 *   CHANTEMP      uint16_t        I/O  bits         last channel 32
 *   RCSFLAGS      uint16_t        I/O  bits         damping, forced firing
 *   CHAN31, CHAN32  uint16_t      I    bits         mode switch, RHC
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   csm_dap_words_*       DAS, MP, SMALLMP on stored words (mysubs.c)
 *   csm_dap_msu, _ads,    basic-language word arithmetic (mysubs.c)
 *   _mp_hi, _dv
 *   agc_csm_dap_stickchk      STICKCHK (manual.c)
 *   csm_rcs_reinit        REINIT (t5rupt.c)
 *   agc_rupt_set_t5_task  T5LOC = JETSLECT (T6PROG)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG when an assertion fails.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs from the T5 interrupt 20 ms after phase 1, interrupts
 *   inhibited. Ones' complement -0 is not represented: where the AGC
 *   tests the sign of a word pair with CCS on both words (SIGNCK1..5) a
 *   pair of zeros counts as positive, as +0 does in the AGC.
 *
 * NOTES:
 *   Attitude errors (p. 1027): AK = AMGB (CDU - THETAD) + BIAS; the
 *   biases (KALCMANU lead, 1-7 deg) are added only during automatic
 *   maneuvers. Switching logic (p. 1029, FIG. 1): dead zone ADB, flat
 *   region AF = .044 deg, switch lines WL+H = 0.5, WL = 0.4, WL-H = 0.3
 *   deg/s, hysteresis slope SLOPE2 0.8 deg/s/deg, switch-line SLOPE 0.6
 *   deg/s/deg. Jet on-time T = J/M (DELTA W), 1 bit = 1/1600 s.
 *   EDOT, EDOTVEL, WTEMP and KMPAC are word pairs as the AGC holds them;
 *   TAU is the lower word of KMPAC after the two SMALLMPs when the upper
 *   word is zero (TAUNORM), so the split decides the on-time.
 **********************************************************************/
#include "../internal.h"

#define MANROT   ((uint16_t) 000077) /* MANROT: channel 32 bits 1-6       */
#define DAMPBITS ((uint16_t) 016000) /* OCT16000: rate damping bits 13-11 */
#define AF       ((agc_sp_t) 4)      /* FOUR: flat region .044 deg, 180 deg */
#define WLH_SLOP ((agc_sp_t) 76)     /* 00114 .00463 WL+H/SLOPE .83333 deg, 180 deg */
#define WLMH_SLP ((agc_sp_t) 45)     /* 00055 .00277 WL-H/SLOPE .5 deg, 180 deg     */
#define SLOPE2   ((agc_sp_t) 5243)   /* 12173 .32 = 0.8 deg/s/deg, 450/180 */
#define BIT11    ((agc_sp_t) 02000)  /* JTIME second SMALLMP: 2^-4         */

/* WL+H, WL-H, WL (2DEC, 450 deg/s) as their two words.                   */
static const agc_dp_words_t wlh  = {022, 06426};  /* .0011111111 0.5 deg/s */
static const agc_dp_words_t wlmh = {012, 035415}; /* .0006666666 0.3 deg/s */
static const agc_dp_words_t wl   = {016, 022021}; /* .0008888888 0.4 deg/s */

/* MINTAU (p. 1026): minimum impulse on-times, 1/1600 s.                  */
static const agc_sp_t mintau[4] = {0, 23, -23, 0}; /* 0, +-14 ms, 0       */

/* Forced firing masks (OCT01400, OCT00060, OCT00300) and rate damping
 * bits (BIT13, BIT12, BIT11) by SPNDX: roll, pitch, yaw.                */
static const uint16_t forcemask[AGC_CSM_AXES] = {001400, 000060, 000300};
static const uint16_t dampbit[AGC_CSM_AXES]   = {010000, 004000, 002000};

/* Outcome of the switching logic: the label the AGC branches to.        */
typedef enum pp_region
{
    PP_J18 = 0, /* fire to null the rate error                  */
    PP_J20,     /* fire onto the hysteresis line (+ side)       */
    PP_J21,     /* fire to the WL switch line                   */
    PP_J22,     /* fire to the WL switch line, other polarity   */
    PP_J23,     /* coast (unless a forced firing)               */
    PP_J24      /* fire onto the hysteresis line (- side)       */
} pp_region_t;

/* One axis in the phase plane (EDOT .. ADBVEL overlay, p. 1030).         */
typedef struct pp_point
{
    agc_dp_words_t edot;    /* EDOT = ADOT - WBODY, 450 deg/s          */
    agc_dp_words_t edotvel; /* EDOTVEL = EDOT by its sign, 450 deg/s   */
    agc_sp_t       aerr;    /* AERR = ERROR, scale 180 deg             */
    agc_sp_t       aerrvel; /* AERRVEL = AERR by the sign of EDOT, scale 180 deg */
    agc_sp_t       adbvel;  /* ADBVEL = +-(ADB+AF), scale 180 deg      */
    agc_sp_t       adbaf;   /* T5TEMP = ADB+AF, scale 180 deg          */
} pp_point_t;

static bool         words_nonneg(agc_dp_words_t w);
static agc_status_t rcs_grabang(void);
static agc_status_t rcs_atthold(void);
static agc_status_t rcs_t6prog(void);
static pp_region_t  rcs_region(const pp_point_t *p);
static bool         rcs_delta_w(int i, const pp_point_t *p, agc_dp_words_t *kmpac);
static agc_sp_t     rcs_jtime(int i, agc_dp_words_t kmpac);

/**********************************************************************
 * CCS X; ...; CCS X +1 (SIGNCK1 .. SIGNCK5, AUTOMATIC_MANEUVERS.agc
 * pp. 1030-1033) -- the sign of a word pair as the AGC tests it: the
 * upper word decides unless it is zero, then the lower word does.
 *
 * ARGUMENTS:
 *   w  agc_dp_words_t  I  the pair as stored, any scale
 *
 * RETURNS: bool -- true for + (or +0 in both words), false for -
 **********************************************************************/
static bool words_nonneg(agc_dp_words_t w)
{
    if (w.hi != 0)
    {
        return (w.hi > 0); /* CCS X                                    */
    }

    return (w.lo >= 0); /* CCS X +1                                    */
}

/**********************************************************************
 * AHFNOROT, AUTOCONT (AUTOMATIC_MANEUVERS.agc p. 1025) -- no manual
 * rotation commanded: free mode, filter initialisation, attitude hold or
 * automatic control by the mode switch (channel 31 bits 14, 13) and
 * HOLDFLAG.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the branch taken
 **********************************************************************/
agc_status_t csm_rcs_ahfnorot(void)
{
    const agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* RCS DAP erasables    */

    if ((agc_chan_read(AGC_CHAN_CHAN31) & (uint16_t) AGC_CH031_FREE) == 0u)
    {
        /* READ CHAN31; MASK BIT14; BZMF        */
        /**********************************************************************
         * FREECONT (AUTOMATIC_MANEUVERS.agc pp. 1025-1026) -- minimum impulse
         * control from channel 32, firing only on a new deflection (CHANTEMP
         * holds the last image): TAU = MINTAU by the STICKCHK indices.
         *
         * ARGUMENTS: none
         *
         * RETURNS: agc_status_t -- the STICKCHK or T6PROGM status
         **********************************************************************/
        agc_csm_rcs_t *r_freecont = agc_csm_dap_rcs(); /* RCS DAP erasables          */

        r_freecont->holdflag = 1;                                                                 /* CAF ONE; TS HOLDFLAG: inhibit steering        */
        uint16_t ch32        = agc_chan_read(AGC_CHAN_CHAN32);                                    /* READ CHAN32; TS L */
        uint16_t bits        = (uint16_t) (~(unsigned int) ch32 & MANROT & r_freecont->chantemp); /* COM; MASK MANROT; MASK CHANTEMP */

        r_freecont->chantemp = ch32;                       /* LXCH CHANTEMP                            */
        agc_status_t st      = agc_csm_dap_stickchk(bits); /* TC STICKCHK */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        for (int i = 0; i < AGC_CSM_AXES; i++)
        {
            int m = (int) r_freecont->manndx[i]; /* INDEX RMANNDX                    */

            if (!AGC_ASSERT((m >= 0) && (m <= 3)))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            r_freecont->tau[i] = mintau[m]; /* CA MINTAU; TS TAU, TAU1, TAU2         */
        }

        return (csm_rcs_t6prog_manual());
    }

    if (csm_rcs_rate_init_pending())
    {
        return (csm_rcs_reinit()); /* recycle to initialise the filter  */
    }

    if ((agc_chan_read(AGC_CHAN_CHAN31) & (uint16_t) AGC_CH031_ATTHOLD) == 0u)
    {
        /* MASK BIT13; BZMF HOLDFUNC            */
        /**********************************************************************
         * HOLDFUNC (AUTOMATIC_MANEUVERS.agc p. 1028) -- attitude hold: HOLDFLAG
         * +0 holds the references; anything else grabs new ones.
         *
         * ARGUMENTS: none
         *
         * RETURNS: agc_status_t -- the ATTHOLD or GRABANG status
         **********************************************************************/
        const agc_csm_rcs_t *r_holdfunc = agc_csm_dap_rcs(); /* RCS DAP erasables    */

        if (r_holdfunc->holdflag == 0)
        {
            return (rcs_atthold()); /* CCS HOLDFLAG: +0                      */
        }

        return (rcs_grabang());
    }

    if (r->holdflag <= 0)
    { /* AUTOCONT: HOLDFLAG +0, -0, - (BZMF) */
        return (rcs_atthold());
    }

    return (rcs_grabang());
}

/**********************************************************************
 * GRABANG, ENDDAMP (AUTOMATIC_MANEUVERS.agc p. 1028) -- zero the WBODYs
 * and biases; while rate damping is on, zero the errors and go to JETS;
 * after it, pick up the CDU angles as the hold references (HOLDFLAG
 * +0) and go to ATTHOLD.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the JETS or ATTHOLD status
 **********************************************************************/
static agc_status_t rcs_grabang(void)
{
    agc_csm_rcs_t          *r = agc_csm_dap_rcs(); /* RCS DAP erasables  */
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* RCSFLAGS           */

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        r->wbody[i] = 0; /* CAF ZERO; TS WBODY .. BIAS2                  */
        r->bias[i]  = 0;
    }

    if ((c->rcsflags & DAMPBITS) != 0u)
    { /* rate damping not completed */
        for (int i = 0; i < AGC_CSM_AXES; i++)
        {
            r->error[i] = 0;
        }

        return (csm_rcs_jets());
    }

    r->holdflag = 0; /* ENDDAMP: TS HOLDFLAG (+0)                       */

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        r->thetad[i] = csm_dap_cdu(i); /* DCA CDUX; DXCH THETADX; CDUZ  */
    }

    return (rcs_atthold());
}

/**********************************************************************
 * ATTHOLD (AUTOMATIC_MANEUVERS.agc pp. 1027-1028) -- the attitude
 * errors in control axes, ERROR = AMGB (CDU - THETAD), plus the biases
 * when HOLDFLAG is negative (automatic maneuver); then JETS.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the JETS status
 **********************************************************************/
static agc_status_t rcs_atthold(void)
{
    agc_csm_rcs_t *r  = agc_csm_dap_rcs();                           /* RCS DAP erasables       */
    agc_sp_t       ex = csm_dap_msu(agc_erase_cdux(), r->thetad[0]); /* CDUX - THETADX, scale 180 deg */
    agc_sp_t       t  = csm_dap_msu(agc_erase_cduy(), r->thetad[1]); /* T5TEMP, scale 180 deg   */

    r->error[0] = csm_dap_ads(ex, csm_dap_mp_hi(t, r->amgb1)); /* MP AMGB1; ADS ERRORX */
    r->error[1] = csm_dap_mp_hi(t, r->amgb4);
    r->error[2] = csm_dap_mp_hi(t, r->amgb7);
    t           = csm_dap_msu(agc_erase_cduz(), r->thetad[2]);
    r->error[1] = csm_dap_ads(r->error[1], csm_dap_mp_hi(t, r->amgb5));
    r->error[2] = csm_dap_ads(r->error[2], csm_dap_mp_hi(t, r->amgb8));

    if (r->holdflag < 0)
    { /* CS HOLDFLAG; BZMF JETS: biases only if automatic */
        for (int i = 0; i < AGC_CSM_AXES; i++)
        {
            r->error[i] = csm_dap_ads(r->error[i], r->bias[i]); /* CA BIAS; ADS ERRORX */
        }
    }

    return (csm_rcs_jets());
}

/**********************************************************************
 * JETS, JLOOP, INHOLD, SIGNCK1, POSVEL, NEGVEL, DOJET
 * (AUTOMATIC_MANEUVERS.agc pp. 1030, 1035) -- for each axis (Z, Y, X)
 * locate (rate error, attitude error) in the phase plane and compute
 * the jet on-time TAU; then T6PROG.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the T6PROG status
 **********************************************************************/
agc_status_t csm_rcs_jets(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* RCS DAP erasables          */
    pp_point_t     p;                     /* the axis' point            */

    p.adbaf = csm_dap_ads(r->adb, AF); /* CA ADB; AD FOUR; TS T5TEMP    */

    for (int i = AGC_CSM_AXES - 1; i >= 0; i--)
    { /* JLOOP */
        p.edot = r->adot[i]; /* DCA ADOT; DXCH EDOT                     */

        if (r->holdflag != 0)
        { /* CA HOLDFLAG; BZF INHOLD; DCS WBODY; DAS EDOT */
            csm_dap_words_das(&p.edot, csm_dap_words_neg(agc_dp_to_words(r->wbody[i])));
        }

        p.aerr   = r->error[i];          /* INHOLD: AERR = BIAS + AK   */
        bool pos = words_nonneg(p.edot); /* CCS EDOT; SIGNCK1          */

        p.edotvel            = pos ? p.edot : csm_dap_words_neg(p.edot); /* ?: POSVEL DCA / NEGVEL DCS EDOT */
        p.adbvel             = pos ? p.adbaf : (agc_sp_t) -p.adbaf;      /* ?: +-(ADB+AF)    */
        p.aerrvel            = pos ? p.aerr : (agc_sp_t) -p.aerr;        /* ?: +-AERR        */
        agc_dp_words_t kmpac = {0, 0};                                   /* DELTA W, 450 deg/s */

        if (rcs_delta_w(i, &p, &kmpac))
        {
            r->tau[i] = rcs_jtime(i, kmpac); /* JTIME; DOJET: TS TAU      */
        }
    }

    return (rcs_t6prog());
}

/**********************************************************************
 * J6., J7, J8, SIGNCK2, NJ22, SIGNCK3, NJ23 (AUTOMATIC_MANEUVERS.agc
 * pp. 1030-1032) -- the region of FIG. 1 the point is in.
 *
 * ARGUMENTS:
 *   p  const pp_point_t *  I  the axis' point (EDOTVEL, AERRVEL, ADB+AF)
 *
 * RETURNS: pp_region_t -- the label the AGC branches to
 **********************************************************************/
static pp_region_t rcs_region(const pp_point_t *p)
{
    const agc_csm_rcs_t *r   = agc_csm_dap_rcs(); /* ADB, SLOPE           */
    int                  adb = (int) r->adb;      /* ADB, 180 deg         */

    if ((int) p->aerrvel - adb + WLH_SLOP > 0)
    { /* J6.: SU ADB; AD WLH/SLOP; BZMF J8 */
        if ((int) p->aerrvel - (int) p->adbaf > 0)
        { /* CS T5TEMP; AD AERRVEL; BZMF +2; TCF J7 */
            /* J7: CS WL-H/SLP; SU T5TEMP; AD AERRVEL; BZMF J20; TCF J21  */
            return (((-WLMH_SLP - (int) p->adbaf + (int) p->aerrvel) <= 0) ? PP_J20 : PP_J21); /* ?: J20 / J21 */
        }

        /* DCS EDOTVEL; DV SLOPE; SU AERRVEL; AD ADB; BZMF J18; TCF J23  */
        int q = (int) csm_dap_dv(agc_dp_from_words(csm_dap_words_neg(p->edotvel)), r->slope); /* 180 deg */

        return (((q - (int) p->aerrvel + adb) <= 0) ? PP_J18 : PP_J23); /* ?: J18 / J23 */
    }

    /* J8: DCS WLH; DXCH WTEMP; DCA EDOTVEL; DAS WTEMP; CCS WTEMP          */
    agc_dp_words_t wtemp = csm_dap_words_neg(wlh); /* EDOTVEL - (WL+H), 450 deg/s */

    csm_dap_words_das(&wtemp, p->edotvel);

    if (words_nonneg(wtemp))
    {
        return (PP_J22);
    }

    /* NJ22: DCA EDOTVEL; DV SLOPE; AD T5TEMP; AD AERRVEL; CCS A          */
    int a = (int) csm_dap_dv(agc_dp_from_words(p->edotvel), r->slope) + (int) p->adbaf + (int) p->aerrvel; /* 180 deg */

    if (a >= 0)
    {
        return (PP_J23);
    }

    wtemp = csm_dap_words_neg(wlmh); /* DCS WLMH; DXCH WTEMP; DAS EDOTVEL  */
    csm_dap_words_das(&wtemp, p->edotvel);

    if (words_nonneg(wtemp))
    { /* CCS WTEMP; SIGNCK3 */
        return (PP_J23);
    }

    a = (int) p->aerrvel + (int) p->adbaf + WLMH_SLP; /* NJ23: CA AERRVEL; AD T5TEMP; AD WL-H/SLP */

    return ((a >= 0) ? PP_J24 : PP_J22); /* ?: CCS A: +, +0 J24; -, -0 J22 */
} /* rcs_region */

/**********************************************************************
 * J18, J20, J21, SIGNCK4, JN, JP, J22, SIGNCK5, J23, J24
 * (AUTOMATIC_MANEUVERS.agc pp. 1032-1034) -- the desired change in rate
 * DELTA W into KMPAC (450 deg/s). J23 resets the axis' rate damping
 * flag and, without a forced firing, leaves TAU as it was (DOJET +2).
 *
 * ARGUMENTS:
 *   i      int               I  axis: 0 roll, 1 pitch, 2 yaw (SPNDX)
 *   p      const pp_point_t *  I  the axis' point
 *   kmpac  agc_dp_words_t *  O  DELTA W, 450 deg/s, as the AGC holds it
 *
 * RETURNS: bool -- true when JTIME follows, false at J23 without a
 *          forced firing
 **********************************************************************/
static bool rcs_delta_w(int i, const pp_point_t *p, agc_dp_words_t *kmpac)
{
    agc_csm_dapcom_t *c     = agc_csm_dap_com();          /* RCSFLAGS          */
    pp_region_t       reg   = rcs_region(p);              /* switching logic   */
    agc_dp_words_t    medot = csm_dap_words_neg(p->edot); /* DCS EDOT, 450 deg/s */
    bool              epos  = words_nonneg(p->edot);      /* CCS EDOT; SIGNCK4/5 */
    bool              fire  = true;                       /* JTIME follows     */

    switch (reg)
    {
        case PP_J23:
            c->rcsflags = (uint16_t) (c->rcsflags & ~(unsigned int) dampbit[i]); /* CS BIT13; MASK RCSFLAGS */
            fire        = (c->rcsflags & forcemask[i]) != 0u;                    /* forced: TCF J18 */
            *kmpac      = medot;
            break;
        case PP_J18:
            *kmpac      = medot;                                                 /* DCS EDOT; DXCH KMPAC                    */
            break;
        case PP_J20:
            /* CS AERR; AD ADBVEL; MP SLOPE2; DXCH KMPAC; DCS EDOT; DAS   */
            *kmpac      = csm_dap_words_mp(csm_dap_ads((agc_sp_t) -p->aerr, p->adbvel), SLOPE2);
            csm_dap_words_das(kmpac, medot);
            break;
        case PP_J24:
            /* CS AERR; SU ADBVEL; MP SLOPE2; DXCH KMPAC; DCS EDOT; DAS   */
            *kmpac      = csm_dap_words_mp(csm_dap_ads((agc_sp_t) -p->aerr, (agc_sp_t) -p->adbvel), SLOPE2);
            csm_dap_words_das(kmpac, medot);
            break;
        case PP_J21:
            /* EDOT + : JP (DCS EDOT; DCS WL; DAS), - : JN (DCA WL; DAS)  */
            *kmpac      = medot;
            csm_dap_words_das(kmpac, epos ? csm_dap_words_neg(wl) : wl);         /* ?: JP / JN */
            break;
        case PP_J22:
            /* EDOT + : JN, - : JP                                        */
            *kmpac      = medot;
            csm_dap_words_das(kmpac, epos ? wl : csm_dap_words_neg(wl));         /* ?: JN / JP */
            break;
        default:
            fire        = false;                                                 /* unreachable: all regions listed        */
            break;
    }

    return (fire);
} /* rcs_delta_w */

/**********************************************************************
 * JTIME, TAUNORM, DOJET (AUTOMATIC_MANEUVERS.agc p. 1035) -- TAU = J/M
 * (DELTA W) for one jet: SMALLMP by J/M, then by BIT11; an upper word
 * left nonzero means the time overflowed (POSMAX or NEGMAX), otherwise
 * TAU is the lower word.
 *
 * ARGUMENTS:
 *   i      int             I  axis: 0 roll, 1 pitch, 2 yaw (SPNDX)
 *   kmpac  agc_dp_words_t  I  DELTA W, 450 deg/s, as the AGC holds it
 *
 * RETURNS: agc_sp_t -- TAU, 1/1600 s
 **********************************************************************/
static agc_sp_t rcs_jtime(int i, agc_dp_words_t kmpac)
{
    const agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* J/M                  */

    agc_dp_words_t k = csm_dap_words_smallmp(kmpac, r->jm[i]); /* CA J/M; TC SMALLMP, (57.3/450)(B24/1600) */

    k = csm_dap_words_smallmp(k, BIT11); /* CA BIT11; TC SMALLMP: 1/1600 s */

    if (k.hi > 0)
    {
        return (AGC_SP_MAX); /* CCS KMPAC: CA POSMAX                    */
    }

    if (k.hi < 0)
    {
        return (AGC_SP_MIN); /* CA NEGMAX                               */
    }

    return (k.lo); /* TAUNORM: CA KMPAC +1                            */
}

/**********************************************************************
 * T6PROGM (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1020) -- zero the errors
 * for manual rotations, then T6PROG.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the T6PROG status
 **********************************************************************/
agc_status_t csm_rcs_t6prog_manual(void)
{
    agc_csm_rcs_t *r = agc_csm_dap_rcs(); /* RCS DAP erasables          */

    for (int i = 0; i < AGC_CSM_AXES; i++)
    {
        r->error[i] = 0; /* CAF ZERO; TS ERRORX, ERRORY, ERRORZ         */
    }

    return (rcs_t6prog());
}

/**********************************************************************
 * T6PROG (AUTOMATIC_MANEUVERS.agc p. 1036) -- the rotation commands
 * (TAUs) are determined: T5LOC = JETSLECT for phase 3 (DCA JETADDR;
 * DXCH T5LOC); RESUME.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
static agc_status_t rcs_t6prog(void)
{
    agc_rupt_set_t5_task(AGC_TASK_JETSLECT);

    return (AGC_STATUS_OK);
}
