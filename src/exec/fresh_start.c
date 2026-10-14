/**********************************************************************
 * FILE NAME: fresh_start.c
 *
 * PURPOSE:
 *   FRESH START AND RESTART: SLAP1, DOFSTART, DOFSTRT1, GOPROG, ENEMA,
 *   GOPROG2, GOPROG3, LIGHTSET, STARTSUB, STARTSB1, STARTSB2, ENDRSTRT and
 *   GOTOPOOH.
 *
 * AGC SOURCE:
 *   Comanche055 FRESH_START_AND_RESTART.agc pp. 181-195
 *   Luminary099 FRESH_START_AND_RESTART.agc pp. 211-226 (differs: ENEMA
 *   calls STARTSB1, DOFSTRT1, GOPROG's LM part)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   FLAGWRD0-11   uint16_t  I/O  bits           SWINIT, STARTSB2 resets
 *   DSPTAB +11D   uint16_t  I/O  lamp bits      C/S lamp word
 *   FAILREG +0..2 uint16_t  O    octal code     cleared by SLAP1
 *   MODREG        agc_sp_t  I/O  B0 integer     -0 after a fresh start
 *   IMODES30/33   uint16_t  O    bits           IMU mode words
 *   channels 5,6,11-14,33   I/O  bits           outputs off, flip-flops
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   waitlist_clear_lists  STARTSB2: LST1, LST2 (waitlist.c)
 *   exec_clear_tables     STARTSB2: core sets, VAC areas (executive.c)
 *   agc_phase_restart_all GOPROG3 NXTRST loop (restarts.c)
 *   exec_up_startsb1      LM STARTSB1 DAP part (change_program/idle.c)
 *   exec_up_startsb2/dofstart  other modules' resets (change_program/idle.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   ALARM 1107 (phase tables disagree) and a fresh start; the BAILOUT
 *   1201-1203 of FINDVAC, NOVAC, WAITLIST during a restart.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Every entry runs with interrupts inhibited. Both vehicles.
 *
 * NOTES:
 *     SLAP1     man-initiated fresh start (V36): STARTSUB, clear FAILREG,
 *               REDOCTR, then DOFSTART.
 *     DOFSTART  machine-initiated fresh start: outputs off, all restart
 *               groups killed, MODREG = -0, IMU flags and flagwords to
 *               their fresh-start values, then idle (ENDRSTRT).
 *     GOPROG    hardware restart (GOJAM at 4000): count it in REDOCTR,
 *               STARTSUB, keep the lamps and engine state, then GOPROG3.
 *     ENEMA     software restart (WHIMPER, V37 via GOPROG2): STARTSB2
 *               (LM: STARTSB1), then GOPROG3.
 *     GOPROG3   check -PHASEn / PHASEn agreement (else alarm 1107 and a
 *               fresh start), display the major mode, restart every active
 *               group; with none active go to P00 unless MODREG = -0.
 *     STARTSUB  STARTSB1 (TIME3, TIME4, TIME5) + STARTSB2 (outbits, the
 *               waitlist, core sets, VAC areas, DSKY and mode-switch words).
 *
 *   Erasables owned by other lanes that these routines also reset are
 *   listed, not written, at the place the AGC writes them ("delegated");
 *   docs/modules/exec-sched.md asks the owners for reset entry points.
 *   The erasable self-test restore at GOPROG (ERESTORE / SKEEP4-7) is not
 *   modelled: ERASCHK does not exist in C, so ERESTORE is always +0.
 **********************************************************************/
#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define FS_BIT15            040000u            /* bit 15: lamp word "send me" / OSC FAIL */
#define FS_BIT14            020000u            /* bit 14: engine off / AGC WARNING */
#define FS_BIT13            010000u            /* bit 13: engine on */
#define FS_BIT12            004000u            /* bit 12: RUPT10 enable (channel 13) */
#define FS_BIT7             000100u            /* bit 7: CSM MARK REJECT (channel 16) */
#define FS_BIT6             000040u            /* bit 6: gyro enable, IMODES33 bit 6 */
#define FS_BIT5             000020u            /* bit 5: LM MARK REJECT (channel 16) */
#define FS_BIT4             000010u            /* bit 4: NO ATT lamp / throttle counter */
#define FS_BITS4_6          000050u            /* BITS4&6: NO ATT, GIMBAL LOCK lamps     */
#define FS_LAMPS_964        000450u            /* 9,6,4: PROG ALARM, GIMBAL LOCK, NO ATT */
#define FS_LAMPWORD         11                 /* DSPTAB +11D                            */
#define FS_DSPTAB_N         11                 /* DSPTAB +0 .. +10D (relay words)        */
#define FS_DSP_BLANK        073777u            /* CS BIT12: blank relay word             */
#define FS_IM30INIF         037411u            /* inhibit IMU fail 5 s and PIP ISSW      */
#define FS_IM30INIR         037000u            /* IM30INIR: restart value of IMODES30       */
#define FS_IFAILINH         000435u            /* failure inhibits kept on restart       */
#define FS_IM33INIT         016000u            /* = PRIO16: no PIP or TM fail signals    */
#define FS_ELR              022u               /* ERROR LIGHT RESET key code (-ELR)      */
#define FS_KEYCODE          037u               /* OCT37: key code bits of channel 16     */
#define FS_INTMASK          020100u            /* INTMASK / RSFLGBTS: FLGWRD10 bits 14,7 */
#define FS_F6_DAPBITS       060000u            /* FLAGWRD6 bits 15,14: autopilot in T5   */
#define FS_F6_RCS           020000u            /* 01: RCS DAP                            */
#define FS_F6_TVC           040000u            /* 10: TVC DAP                            */
#define FS_HALF_SEC         50                 /* .5SEC (cs)                             */
#define FS_CA_ECE_CS        6                  /* CAF SIX / TC WAITLIST CA+ECE           */
#define FS_PRIO30           ((agc_prio_t) 030) /* PRIO30: RESTREG after a fresh start */
#define FS_PHASE_4_1        014u               /* GOTOPOOH (CSM): PHASCHNG OCT 14        */
#define FS_PHASE_4_33       033u               /* GOTOPOOH (LM): 4.33SPOT, GOP00FIX      */
#define FS_FLAGWORDS_SWINIT 12                 /* SWINIT covers STATE .. STATE +11D      */

static uint16_t   redoctr; /* REDOCTR: restarts since fresh start     */
static agc_prio_t restreg; /* RESTREG: display restart priority       */

/**********************************************************************
 * AGC: none -- true for the Luminary rope (agc_state_vehicle)
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true for the LM
 **********************************************************************/
static bool is_lm(void)
{
    return (agc_state_vehicle() == AGC_VEHICLE_LM);
}

/**********************************************************************
 * AGC: none -- DOFSTART / STARTSB2 flagword reset (FRESH_START_AND_RESTART
 * C p.193 / L p.225 SWINIT): replace the bits of flagword `w` outside
 * `keep` by `init`.
 *
 * ARGUMENTS:
 *   w -- flagword index (FLAGWRDn)
 *   keep -- bits of the flagword kept
 *   init -- bits set from the SWINIT word
 **********************************************************************/
static void flagword_init(int w, uint16_t keep, uint16_t init)
{
    uint16_t v = (uint16_t) ((agc_flagword_get(w) & keep) | init); /* the flagword after the reset */

    agc_flagword_set(w, v);
}

/**********************************************************************
 * TS DSPTAB +11D (FRESH_START_AND_RESTART C p.183 / L p.211): the
 * C/S lamp word.
 *
 * ARGUMENTS:
 *   v -- the lamp word (DSPTAB +11D)
 **********************************************************************/
static void set_lampword(uint16_t v)
{
    agc_erase_set_dsptab(FS_LAMPWORD, v);
}

/**********************************************************************
 * STARTSB2, channel and flag part (FRESH_START_AND_RESTART C p.190 /
 * L p.220).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void startsb2_outbits(void)
{
    if (!is_lm())
    {
        agc_chan_and(AGC_CHAN_DSALMOUT, 077603u);                 /* UPLINK ACTY .. OPR ERR off */
        agc_chan_and(AGC_CHAN_CHAN13, 074777u);                   /* test alarms, standby enable */
        flagword_init(2, (uint16_t) ~025000u & 077777u, 001000u); /* R21MARK, P21FLAG off; SKIPVHF on */
    }
    else
    {
        agc_chan_and(AGC_CHAN_DSALMOUT, 030001u);            /* keep engine on/off, ISS warning */
        flagword_init(3, (uint16_t) ~000400u & 077777u, 0u); /* READRFLG off   */

        /* delegated: RADMODES turn-on/reposition/CDU zero/remode bits (prog-rendz) */
        agc_chan_and(AGC_CHAN_CHAN12, 027470u);              /* keep IMU flags, trims, RR lockon */
        flagword_init(5, (uint16_t) ~002010u & 077777u, 0u); /* NORRMON, R77FLAG off */
        agc_chan_and(AGC_CHAN_CHAN13, 074160u);              /* keep telemetry flags        */
        agc_chan_or(AGC_CHAN_CHAN13, FS_BIT12);              /* re-enable RUPT10            */
        agc_chan_and(AGC_CHAN_CHAN14, FS_BIT6);              /* keep gyro enable            */
    }
}

/**********************************************************************
 * STARTSB2 (C pp.190-192 / L pp.220-222): waitlist, executive, DSKY.
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void startsb2(void)
{
    startsb2_outbits();
    waitlist_clear_lists(); /* LST1 = NEG1/2, LST2 = ENDTASK */
    exec_clear_tables();    /* PRIORITY = -0, VACnUSE, NEWJOB */

    for (int i = 0; i < FS_DSPTAB_N; i++)
    { /* DSPOFF: blank R1-R3 etc. */
        agc_erase_set_dsptab(i, FS_DSP_BLANK);
    }

    /* io-pinball: DELAYLOC, R1SAVE, INLINK, DSPCNT, CADRSTOR, REQRET,
     * CLPASS, DSPLOCK, MONSAVE, MONSAVE1, VERBREG, NOUNREG, DSPLIST, NOUT,
     * DSPCOUNT; sys-misc DELAYLOC, DSRUPTSW, EXTVBACT (exec_up_startsb2).
     * Not reset here (no C storage of their own): SELFRET, ATTCADR, LGYRO,
     * SAMPLIM.                                                          */
    exec_up_startsb2(); /* other modules' STARTSB2 resets */

    agc_erase_set_markstat(0);
    exec_clear_modecadr();  /* IMUCADR OPTCADR RADCADR */
    agc_flagword_set(4, 0); /* kill interface displays */

    if (is_lm())
    { /* leave bit 6 unchanged   */
        agc_erase_set_imodes33((uint16_t) ((agc_erase_imodes33() & FS_BIT6) | FS_IM33INIT));
    }
}

/**********************************************************************
 * STARTSB1 (L p.220; the same code opens Comanche's STARTSUB C p.190):
 * TIME3 = 37777, TIME4 = 37775, TIME5 = 37774; LM: the DAP part (RCSFLAGS
 * bit 13 off so DAPIDLER reruns 1/ACCS, T6NEXT = POSMAX, WAND CHAN13
 * POSMAX, NXT6ADR, NEXTP, ACCSOKAY off, T5ADR = DAPIDLER); then STARTSB2.
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void startsb1(void)
{
    waitlist_reset_time3();
    rupt_reset_timers();

    if (is_lm())
    {
        exec_up_startsb1(); /* the DAP part (lm/dap) */
    }

    startsb2();
}

/**********************************************************************
 * STARTSUB (FRESH_START_AND_RESTART C p.190 / L p.219): DNTMGOTO =
 * DNPHASE1 and LM RADMODES (exec_up_startsub), then STARTSB1.
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void startsub(void)
{
    exec_up_startsub(); /* DNTMGOTO, LM RADMODES */

    startsb1(); /* TC STARTSB1 */
}

/**********************************************************************
 * DOFSTART tail common to both vehicles (FRESH_START_AND_RESTART C
 * pp.183-185 / L pp.211-213): groups, MODREG, RESTREG, IMU words, the
 * SWINIT flagwords.
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void dofstart_common(void)
{
    /* "The IMU was in coarse align in gimbal lock, so put it back"
     * (DOFSTART C p.184 / L p.212).                                       */
    uint16_t lamps = agc_erase_dsptab(FS_LAMPWORD); /* DSPTAB +11D before DOFSTART */

    if ((~lamps & FS_BITS4_6) == 0U)
    {
        agc_chan_or(AGC_CHAN_CHAN12, FS_BITS4_6);
    }

    agc_phase_mr_klean();                   /* TC MR.KLEAN          */
    agc_erase_set_modreg(EXEC_MODREG_NEG0); /* CS ZERO / TS MODREG  */
    restreg = FS_PRIO30;                    /* TS RESTREG           */
    agc_erase_set_imodes30(FS_IM30INIF);

    if (is_lm())
    {
        /* delegated (dap-lm): DB, RATEINDX, DAPBOOLS, STIKSENS, -RATEDB,
         * HIASCENT, DKTRAP, LMTRAP, DKKAOSN, LMKAOSN, LMOMEGAN, DKOMEGAN,
         * DKDB.                                                         */
        agc_erase_set_imodes33((uint16_t) (FS_IM33INIT + FS_BIT6));
    }
    else
    {
        /* delegated (imu/prog-align): OPTIND = -1, OPTMODES = OPTINITF. */
        agc_erase_set_imodes33(FS_IM33INIT);
        agc_rupt_set_t5_task(AGC_TASK_T5IDLOC); /* let T5 idle   */
    }

    /* DOFSTART flagword initialisation from SWINIT (C p.193 / L p.225).
     * Kept through a fresh start: CSM NODOP01, REFSMFLG, CMOONFLG,
     * LMOONFLG, SURFFLAG; LM REFSMFLG, SURFFLAG, CMOONFLG, LMOONFLG, APSFLAG.
     * SWINIT sets CSM FLAGWRD5 0200 (COMPUTER); LM FLAGWRD3 02000
     * (NOR29FLG), FLAGWRD7 0100, FLAGWRD11 40000 (LRBYPASS).              */
    uint16_t moons = (uint16_t) (agc_flag_mask(AGC_FLAG_CMOONFLG) | agc_flag_mask(AGC_FLAG_LMOONFLG) | agc_flag_mask(AGC_FLAG_SURFFLAG)); /* OCT6200: CMOONFLG, LMOONFLG, SURFFLAG kept */
    uint16_t refsm = agc_flag_mask(AGC_FLAG_REFSMFLG);                                                                                    /* REFSMFLG kept */

    for (int w = 0; w < FS_FLAGWORDS_SWINIT; w++)
    {
        uint16_t keep = 0; /* bits of the old flagword kept */
        uint16_t init = 0; /* bits set from SWINIT */

        if (w == 3)
        {
            keep = refsm;
            init = is_lm() ? 002000u : 0u; /* LM SWINIT: NOR29FLG */
        }
        else if (w == 8)
        {
            keep = moons; /* OCT6200            */
        }
        else if (w == 1 && !is_lm())
        {
            keep = 004000u; /* NOP01BIT (CSM)     */
        }
        else if (w == 5 && !is_lm())
        {
            init = 000200u; /* COMPUTER (CSM)     */
        }
        else if (w == 7 && is_lm())
        {
            init = 000100u;
        }
        else if (w == 10 && is_lm())
        {
            keep = 010000u; /* APSFLBIT (LM)      */
        }
        else if (w == 11 && is_lm())
        {
            init = 040000u; /* LRBYPASS (LM)      */
        }
        else
        {
            /* SWINIT word is 0 and nothing is kept */
        }

        flagword_init(w, keep, init);
    }

    /* ENDRSTRT: on to DUMMYJOB +2 (RELINT, NEWJOB untouched).          */
} /* dofstart_common */

/**********************************************************************
 * DOFSTART (FRESH_START_AND_RESTART C p.183): outputs off and fresh-start
 * initialisation.
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void dofstart_csm(void)
{
    /* delegated (sys-misc self-check / uplink): ERESTORE, SMODE, UPSVFLAG */
    agc_chan_write(AGC_CHAN_PYJETS, 0); /* turn off RCS jets    */
    agc_chan_write(AGC_CHAN_ROLLJETS, 0);
    agc_chan_write(AGC_CHAN_DSALMOUT, 0);
    agc_chan_write(AGC_CHAN_CHAN12, 0);
    agc_chan_write(AGC_CHAN_CHAN13, 0);
    agc_chan_write(AGC_CHAN_CHAN14, 0);

    /* delegated: WTOPTION, DNLSTCOD, NVSAVE, EBANKTEM, RATEINDX, TRKMKCNT,
     * VHFCNT, EXTVBACT.                                                */
    dofstart_common();
}

/**********************************************************************
 * DOFSTRT1 (FRESH_START_AND_RESTART L p.211): as DOFSTART but the engine
 * is not touched.
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void dofstrt1_lm(void)
{
    /* delegated (dap-lm, sys-misc): RCSFLAGS = 4, ABDELV, NVSAVE,
     * EBANKTEM, CH5MASK, CH6MASK, PVALVEST, ERESTORE, SMODE, DNLSTCOD,
     * AGSWORD, UPSVFLAG.                                               */
    restreg = FS_PRIO30;
    agc_chan_write(AGC_CHAN_PYJETS, 0);
    agc_chan_write(AGC_CHAN_ROLLJETS, 0);
    agc_chan_write(AGC_CHAN_CHAN12, 0);
    agc_chan_write(AGC_CHAN_CHAN13, 0);
    agc_chan_write(AGC_CHAN_CHAN14, 0);

    dofstart_common();
}

/**********************************************************************
 * DOFSTART (engine_off) or, LM only, DOFSTRT1 (!engine_off: PTBAD and a
 * bad ERESTORE leave the engine alone), then the other modules' resets
 * (FRESH_START_AND_RESTART C pp.183-185 / L pp.211-213).
 *
 * ARGUMENTS:
 *   slap1 -- true for the V36 SLAP1 fresh start (also the SLAP1-only resets)
 *   engine_off -- true: DOFSTART turns the engine off; LM false = DOFSTRT1
 *
 * RETURNS: none
 **********************************************************************/
static void dofstart(bool slap1, bool engine_off)
{
    if (is_lm())
    {
        if (engine_off)
        {
            agc_chan_write(AGC_CHAN_DSALMOUT, FS_BIT14); /* engine off  */
            agc_erase_set_thrust(0);                     /* CS ZERO / TS THRUST (-0)      */
        }

        dofstrt1_lm();
    }
    else
    {
        dofstart_csm();
    }

    exec_up_dofstart(slap1); /* other modules' erasables */
}

/**********************************************************************
 * LIGHTSET (C p.189 / L p.219): MARK REJECT together with ERROR LIGHT
 * RESET forces a fresh start; *done is true when it did one.
 *
 * ARGUMENTS:
 *   done -- set true when LIGHTSET did a fresh start
 *
 * RETURNS: none
 **********************************************************************/
static void lightset(bool *done)
{
    uint16_t nav         = agc_chan_read(AGC_CHAN_NAVKEYIN); /* channel 16, nav DSKY / MARK keys */
    uint16_t mark_reject = FS_BIT7;                          /* CSM: channel 16 bit 7 */

    if (is_lm())
    {
        mark_reject = FS_BIT5; /* LM: channel 16 bit 5 */
    }

    *done = false;

    if ((nav & mark_reject) != 0U)
    { /* else NONAVKEY: TC Q */
        bool elr = !is_lm() && (nav & FS_KEYCODE) == FS_ELR; /* nav DSKY  */

        if (!elr)
        {
            elr = agc_chan_read(AGC_CHAN_MNKEYIN) == FS_ELR; /* main DSKY */
        }

        if (elr)
        {
            *done = true;
            startsub();
            dofstart(false, true); /* DOFSTART */
        }
    }
}

/**********************************************************************
 * PCLOOP .. PTBAD (C p.188 / L p.218): every group's -PHASEn must be
 * the complement of PHASEn (*ok), else alarm 1107 and a fresh start,
 * whose status is returned.
 *
 * ARGUMENTS:
 *   ok_out -- set true when every -PHASEn is the complement of PHASEn
 *
 * RETURNS: none
 **********************************************************************/
static void phase_tables_ok(bool *ok_out)
{
    bool ok = true; /* -PHASEn = complement of PHASEn so far */

    for (int g = AGC_RESTART_GROUPS; g >= 1 && ok; g--)
    {
        ok = phase_group_agrees(g);
    }

    *ok_out = ok;

    if (!ok)
    {
        agc_alarm(AGC_ALARM_01107); /* PTBAD        */
        dofstart(false, false);     /* LM DOFSTRT1, CSM DOFSTART */
    }
}

/**********************************************************************
 * GOPROG3 (FRESH_START_AND_RESTART C pp.188-189 / L pp.218-219): check
 * the phase tables, display the major mode, restart every active group;
 * with none active go to P00 unless MODREG = -0.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the restart status; AGC_STATUS_ALARM after the 1107 fresh
 *          start; the MMDSPLAY BAILOUT
 **********************************************************************/
static agc_status_t goprog3(void)
{
    bool ok = false; /* phase tables agree */

    phase_tables_ok(&ok); /* PCLOOP; PTBAD's fresh start */

    if (!ok)
    {
        return (AGC_STATUS_ALARM);
    }

    agc_status_t st = fresh_mmdsplay(); /* TC MMDSPLAY          */

    if (st != AGC_STATUS_OK)
    {
        return (st); /* NOVAC BAILOUT 1202: ENEMA again */
    }

    if (is_lm())
    {
        flagword_init(1, (uint16_t) ~024000u & 077777u, 0u); /* DIDFLAG, RODFLAG */
        flagword_init(0, (uint16_t) ~002000u & 077777u, 0u); /* P21FLAG */
    }
    else if ((agc_flagword_get(6) & FS_F6_DAPBITS) == FS_F6_RCS)
    {
        exec_up_stoprate(); /* RCS DAP: STOPRATE    */
    }
    else
    {
        /* no RCS DAP running */
    }

    st = agc_phase_restart_all(); /* NXTRST loop          */

    if (st == AGC_STATUS_NOT_FOUND)
    { /* no group active      */
        st = AGC_STATUS_OK;

        if (agc_erase_modreg() >= 0)
        { /* MODREG not -0        */
            st = agc_exec_gotopooh();
        }
    }

    return (st); /* ENDRSTRT             */
} /* goprog3 */

/**********************************************************************
 * ELRSKIP .. NOCOARSE (FRESH_START_AND_RESTART C pp.187-188): the CSM
 * hardware-restart part.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the WAITLIST status of
 *          CA+ECE
 **********************************************************************/
static agc_status_t goprog_csm_tail(void)
{
    static const agc_task_id_t t5idler[4] = {
        /* T5IDLER 2CADRs       */
        AGC_TASK_T5IDLOC, AGC_TASK_REDORCS, AGC_TASK_REDOTVC, AGC_TASK_REDOSAT
    };
    unsigned int               sel        = (unsigned int) (agc_flagword_get(6) >> 13) & 3u; /* FLAGWRD6 bits 15,14: DAP in T5 */

    agc_rupt_set_t5_task(t5idler[sel]); /* T5LOC     */
    agc_status_t st = AGC_STATUS_OK;    /* CA+ECE outcome */

    agc_flag_clear(AGC_FLAG_INTFLAG); /* RASFLAG INTFLBIT     */
    exec_up_goprog_optics();          /* OPTMODES = (OPTMODES & OPTINITR) + BIT7 */

    agc_erase_set_imodes33((uint16_t) ((agc_erase_imodes33() & FS_BIT6) | FS_IM33INIT));
    uint16_t old = agc_erase_dsptab(FS_LAMPWORD); /* DSPTAB +11D at the restart */

    set_lampword((uint16_t) ((old & FS_LAMPS_964) | FS_BIT15));

    if ((old & FS_BIT4) != 0U)
    { /* NO ATT lamp was on   */
        exec_up_setcoars();
        st = agc_waitlist_call(FS_CA_ECE_CS, AGC_TASK_CA_ECE);

        if (st != AGC_STATUS_OK)
        {
            return (st); /* waitlist full: BAILOUT 1203 */
        }
    }

    agc_erase_set_imodes30((uint16_t) ((agc_erase_imodes30() & FS_IFAILINH) | FS_IM30INIR));

    if (agc_flag_test(AGC_FLAG_ENGONFLG))
    {
        agc_chan_or(AGC_CHAN_DSALMOUT, FS_BIT13); /* turn engine on       */
    }

    return (AGC_STATUS_OK);
} /* goprog_csm_tail */

/**********************************************************************
 * GOPROG (C pp.186-188 / L pp.216-217): hardware restart from GOJAM.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the restart status (see goprog3), or the status of the
 *          fresh start GOPROG chose
 **********************************************************************/
agc_status_t agc_restart_goprog(void)
{
    redoctr = (uint16_t) (redoctr + 1U); /* INCR REDOCTR         */
    agc_inhint();

    if (!is_lm())
    {
        exec_vac5stor();

        /* OSC FAIL (ch 33 bit 15) absent and AGC WARNING (bit 14) on:
         * a restart loop; do a fresh start. Channel 33 reads 0 when the
         * signal is present. DEVIATION: the WAND that resets the
         * channel 33 flip-flops is left to the sim harness.            */
        uint16_t ch33 = agc_chan_read(AGC_CHAN_CHAN33); /* channel 33: OSC FAIL bit 15, AGC WARNING bit 14 */

        if ((ch33 & FS_BIT15) != 0U && (ch33 & FS_BIT14) == 0U)
        {
            startsub();
            dofstart(false, true);
            agc_relint();

            return (AGC_STATUS_OK);
        }
    }
    else if ((agc_erase_dsptab(FS_LAMPWORD) & FS_BIT4) != 0U)
    {
        agc_chan_or(AGC_CHAN_CHAN12, (uint16_t) (FS_BIT4 | FS_BIT6)); /* coarse align */
    }
    else
    {
        /* LM, NO ATT lamp off */
    }

    bool         done = false;         /* LIGHTSET did a fresh start */
    agc_status_t st   = AGC_STATUS_OK; /* restart outcome */

    lightset(&done); /* BUTTONS              */

    if (!done)
    {
        startsub(); /* ERESTORE = +0        */

        if (is_lm())
        {
            /* SETINFL .. GOPROG3 (L p.217): the LM hardware-restart part.         */
            agc_flag_clear(AGC_FLAG_INTFLAG);
            uint16_t old = agc_erase_dsptab(FS_LAMPWORD); /* DSPTAB +11D at the restart */

            set_lampword((uint16_t) ((old & FS_LAMPS_964) | FS_BIT15));
            agc_erase_set_imodes30((uint16_t) ((agc_erase_imodes30() & FS_IFAILINH) | FS_IM30INIR));

            exec_up_goprog_downlist();             /* DNLSTCOD = AGSWORD   */
            agc_chan_or(AGC_CHAN_CHAN14, FS_BIT4); /* throttle counter on  */

            if (agc_flag_test(AGC_FLAG_ENGONFLG))
            {
                agc_chan_or(AGC_CHAN_DSALMOUT, FS_BIT13); /* turn engine on       */
            }
            else
            {
                agc_chan_or(AGC_CHAN_DSALMOUT, FS_BIT14); /* turn engine off      */
            }
        }
        else
        {
            st = goprog_csm_tail();
        }

        if (st == AGC_STATUS_OK)
        {
            st = goprog3();
        }
    }

    agc_relint();

    return (st);
} /* agc_restart_goprog */

/**********************************************************************
 * ENEMA / GOPROG2 (FRESH_START_AND_RESTART C p.188 / L p.217): software
 * restart: STARTSB2 (LM: STARTSB1), LIGHTSET, CSM TVC restart, GOPROG3.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the GOPROG3 status, or the first failing status before it
 **********************************************************************/
agc_status_t fresh_enema(void)
{
    bool         done = false;         /* LIGHTSET did a fresh start */
    agc_status_t st   = AGC_STATUS_OK; /* restart outcome */

    agc_inhint();

    if (is_lm())
    {
        startsb1();      /* TC STARTSB1          */
        lightset(&done); /* GOPROG2A             */
    }
    else
    {
        lightset(&done);

        if (!done)
        {
            startsb2();
        }
    }

    if (!done)
    {
        flagword_init(10, (uint16_t) ~FS_INTMASK & 077777u, 0u); /* RSFLGBTS / INTMASK */

        if (!is_lm() && (agc_flagword_get(6) & FS_F6_DAPBITS) == FS_F6_TVC)
        {
            st = agc_waitlist_call(FS_HALF_SEC, AGC_TASK_TVCEXEC); /* TVC on: TVCEXEC in .5 s */
        }

        if (st == AGC_STATUS_OK)
        {
            st = goprog3();
        }
    }

    agc_relint();

    return (st);
} /* fresh_enema */

/**********************************************************************
 * SLAP1 (C p.183 / L p.211): fresh start requested by V36.
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_fresh_start_slap1(void)
{
    agc_inhint();
    startsub(); /* STARTSUB */

    /* STARTSW: TCF SKIPSIM (the simulation start patch is not used).   */
    uint16_t lamps = agc_erase_dsptab(FS_LAMPWORD); /* DSPTAB +11D at the fresh start */

    set_lampword((uint16_t) ((lamps & FS_BITS4_6) | FS_BIT15));

    /* ERCOUNT, LM DUMPCNT = BIT12 and DSRUPTSW = -PRIO12: DOFSTART's
     * resets with slap1 (exec_up_dofstart).                          */

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_failreg(i, 0);
    }

    redoctr = 0;
    dofstart(true, true);
    agc_relint();
}

/**********************************************************************
 * DOFSTART entry (FRESH_START_AND_RESTART C p.183 / L p.211):
 * machine-initiated fresh start with interrupts inhibited.
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_fresh_dofstart(void)
{
    agc_inhint();
    dofstart(false, true);
    agc_relint();
}

/**********************************************************************
 * REDOCTR (FRESH_START_AND_RESTART C p.186 / L p.216): hardware
 * restarts since the last fresh start.
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint16_t -- C(REDOCTR)
 **********************************************************************/
uint16_t agc_restart_redoctr(void)
{
    return (redoctr);
}

/**********************************************************************
 * AGC: none -- REDOCTR, written by address (an image load, V21): the
 * restart count the next GOPROG increments.
 *
 * ARGUMENTS:
 *   v  uint16_t  I   C(REDOCTR), a count (any word: INCR counts it on)
 **********************************************************************/
void agc_restart_set_redoctr(uint16_t v)
{
    redoctr = v;
}

/**********************************************************************
 * GOTOPOOH (C p.194 / L p.223): protect the P00 request by restart
 * group 4 (CSM 4.1 = redisplay, LM 4.33 = GOP00FIX) and go to GOP00FIX.
 * Called from a job, the job continues as GOP00FIX (the caller returns
 * at once). Called from restart code (GOPROG3 with no group active), the
 * restart runs on as the job MMDSPLAY's NOVAC put in core set 0, and
 * GOP00FIX replaces DSPMMJOB there (exec_core0_continue_as): the PROG
 * digits stay as STARTSB2's DSPOFF left them (blank), as in the AGC.
 * When core set 0 is not the current job, GOP00FIX is started as a
 * NOVAC job at the display restart priority (RESTREG).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the CHANG1 (yield) or NOVAC status; OK when
 *          GOP00FIX took core set 0
 **********************************************************************/
agc_status_t agc_exec_gotopooh(void)
{
    agc_status_t st;               /* yield or NOVAC outcome */
    agc_prio_t   prio = FS_PRIO30; /* RESTREG; PRIO30 before the first fresh start set it */

    if (restreg > 0U)
    {
        prio = restreg;
    }

    if (is_lm())
    {
        agc_inhint();
        phase_set_fixed(4, FS_PHASE_4_33); /* DXCH -PHASE4         */
        agc_relint();
    }
    else
    {
        agc_phase_change(FS_PHASE_4_1); /* TC PHASCHNG OCT 14   */
    }

    if (exec_in_job())
    {
        st = agc_exec_job_yield(AGC_JOB_GO_IDLE_FIX);
    }
    else if (exec_core0_continue_as(AGC_JOB_GO_IDLE_FIX))
    {
        st = AGC_STATUS_OK; /* POSTJUMP GOP00FIX in core set 0 */
    }
    else
    {
        st = agc_exec_novac(prio, AGC_JOB_GO_IDLE_FIX);
    }

    return (st);
}

/**********************************************************************
 * GOTOPOOH as a job (FRESH_START_AND_RESTART C p.194 / L p.223).
 *
 * ARGUMENTS:
 *   ctx -- job context (core set 0); ctx->id is the job being run
 *
 * RETURNS: agc_status_t -- the GOTOPOOH status; BAD_ARG for a wrong job id
 **********************************************************************/
agc_status_t agc_job_gotopooh(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx->id == AGC_JOB_GOTOPOOH))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_gotopooh());
}
