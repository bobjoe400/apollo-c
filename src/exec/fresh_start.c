/*
 * fresh_start.c -- SLAP1, DOFSTART, DOFSTRT1, GOPROG, ENEMA, GOPROG2,
 *                  GOPROG3, LIGHTSET, STARTSUB, STARTSB1, STARTSB2,
 *                  ENDRSTRT, GOTOPOOH.
 *
 * Source: FRESH_START_AND_RESTART.agc, Comanche055 pp. 181-195,
 * Luminary099 pp. 211-226.
 *
 *   SLAP1     man-initiated fresh start (V36): STARTSUB, clear FAILREG,
 *             REDOCTR, then DOFSTART.
 *   DOFSTART  machine-initiated fresh start: outputs off, all restart
 *             groups killed, MODREG = -0, IMU flags and flagwords to
 *             their fresh-start values, then idle (ENDRSTRT).
 *   GOPROG    hardware restart (GOJAM at 4000): count it in REDOCTR,
 *             STARTSUB, keep the lamps and engine state, then GOPROG3.
 *   ENEMA     software restart (WHIMPER, V37 via GOPROG2): STARTSB2
 *             (LM: STARTSB1), then GOPROG3.
 *   GOPROG3   check -PHASEn / PHASEn agreement (else alarm 1107 and a
 *             fresh start), display the major mode, restart every active
 *             group; with none active go to P00 unless MODREG = -0.
 *   STARTSUB  STARTSB1 (TIME3, TIME4, TIME5) + STARTSB2 (outbits, the
 *             waitlist, core sets, VAC areas, DSKY and mode-switch words).
 *
 * Erasables owned by other lanes that these routines also reset are
 * listed, not written, at the place the AGC writes them ("delegated");
 * docs/modules/exec-sched.md asks the owners for reset entry points.
 * The erasable self-test restore at GOPROG (ERESTORE / SKEEP4-7) is not
 * modelled: ERASCHK does not exist in C, so ERESTORE is always +0.
 */
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define FS_BIT15            040000u
#define FS_BIT14            020000u
#define FS_BIT13            010000u
#define FS_BIT12            004000u
#define FS_BIT7             000100u
#define FS_BIT6             000040u
#define FS_BIT5             000020u
#define FS_BIT4             000010u
#define FS_BITS4_6          000050u /* BITS4&6: NO ATT, GIMBAL LOCK lamps     */
#define FS_LAMPS_964        000450u /* 9,6,4: PROG ALARM, GIMBAL LOCK, NO ATT */
#define FS_LAMPWORD         11      /* DSPTAB +11D                            */
#define FS_DSPTAB_N         11      /* DSPTAB +0 .. +10D (relay words)        */
#define FS_DSP_BLANK        073777u /* CS BIT12: blank relay word             */
#define FS_IM30INIF         037411u /* inhibit IMU fail 5 s and PIP ISSW      */
#define FS_IM30INIR         037000u
#define FS_IFAILINH         000435u /* failure inhibits kept on restart       */
#define FS_IM33INIT         016000u /* = PRIO16: no PIP or TM fail signals    */
#define FS_ELR              022u    /* ERROR LIGHT RESET key code (-ELR)      */
#define FS_KEYCODE          037u    /* OCT37: key code bits of channel 16     */
#define FS_INTMASK          020100u /* INTMASK / RSFLGBTS: FLGWRD10 bits 14,7 */
#define FS_F6_DAPBITS       060000u /* FLAGWRD6 bits 15,14: autopilot in T5   */
#define FS_F6_RCS           020000u /* 01: RCS DAP                            */
#define FS_F6_TVC           040000u /* 10: TVC DAP                            */
#define FS_HALF_SEC         50      /* .5SEC (cs)                             */
#define FS_CA_ECE_CS        6       /* CAF SIX / TC WAITLIST CA+ECE           */
#define FS_PRIO30           ((agc_prio_t) 030)
#define FS_PHASE_4_1        014u    /* GOTOPOOH (CSM): PHASCHNG OCT 14        */
#define FS_PHASE_4_33       033u    /* GOTOPOOH (LM): 4.33SPOT, GOP00FIX      */
#define FS_FLAGWORDS_SWINIT 12      /* SWINIT covers STATE .. STATE +11D      */

static uint16_t   redoctr; /* REDOCTR: restarts since fresh start     */
static agc_prio_t restreg; /* RESTREG: display restart priority       */
static uint16_t   ercount; /* ERCOUNT: self-check error counter       */

static bool is_lm(void)
{
    agc_vehicle_t v = agc_state_vehicle();

    if (!AGC_ASSERT(v == AGC_VEHICLE_CSM || v == AGC_VEHICLE_LM))
    {
        return (false);
    }

    return (agc_state_vehicle() == AGC_VEHICLE_LM);
}

/* Replace the bits of flagword `w` outside `keep` by `init`.          */
static void flagword_init(int w, uint16_t keep, uint16_t init)
{
    if (!AGC_ASSERT(w >= 0 && w < AGC_FLAGWORDS))
    {
        return;
    }

    if (!AGC_ASSERT((keep & init) == 0U))
    {
        return;
    }

    uint16_t v = (uint16_t) ((agc_flagword_get(w) & keep) | init);

    agc_flagword_set(w, v);
}

static void set_lampword(uint16_t v)
{
    if (!AGC_ASSERT(v <= 077777u))
    {
        return;
    }

    agc_erase_set_dsptab(FS_LAMPWORD, v);
}

/* STARTSB2, channel and flag part (C p.190 / L p.220).                */
static void startsb2_outbits(void)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return;
    }

    if (!is_lm())
    {
        agc_chan_and(AGC_CHAN_DSALMOUT, 077603u);                 /* UPLINK ACTY .. OPR ERR off */
        agc_chan_and(AGC_CHAN_CHAN13, 074777u);                   /* test alarms, standby enable */
        flagword_init(2, (uint16_t) ~025000u & 077777u, 001000u); /* R21MARK, P21FLAG off; SKIPVHF on */

        return;
    }

    agc_chan_and(AGC_CHAN_DSALMOUT, 030001u);            /* keep engine on/off, ISS warning */
    flagword_init(3, (uint16_t) ~000400u & 077777u, 0u); /* READRFLG off   */

    /* delegated: RADMODES turn-on/reposition/CDU zero/remode bits (prog-rendz) */
    agc_chan_and(AGC_CHAN_CHAN12, 027470u);              /* keep IMU flags, trims, RR lockon */
    flagword_init(5, (uint16_t) ~002010u & 077777u, 0u); /* NORRMON, R77FLAG off */
    agc_chan_and(AGC_CHAN_CHAN13, 074160u);              /* keep telemetry flags        */
    agc_chan_or(AGC_CHAN_CHAN13, FS_BIT12);              /* re-enable RUPT10            */
    agc_chan_and(AGC_CHAN_CHAN14, FS_BIT6);              /* keep gyro enable            */

    if (!AGC_ASSERT((agc_chan_read(AGC_CHAN_CHAN14) & ~FS_BIT6) == 0U))
    {
        return;
    }
}

/* STARTSB2 (C pp.190-192 / L pp.220-222): waitlist, executive, DSKY.  */
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
    agc_status_t su = exec_up_startsb2();

    if (!AGC_ASSERT(su == AGC_STATUS_OK))
    {
        return;
    }

    agc_erase_set_markstat(0);
    exec_clear_modecadr();  /* IMUCADR OPTCADR RADCADR */
    agc_flagword_set(4, 0); /* kill interface displays */

    if (is_lm())
    { /* leave bit 6 unchanged   */
        agc_erase_set_imodes33((uint16_t) ((agc_erase_imodes33() & FS_BIT6) | FS_IM33INIT));
    }

    if (!AGC_ASSERT(agc_flagword_get(4) == 0u))
    {
        return;
    }
}

/* STARTSB1 (L p.220; the same code opens Comanche's STARTSUB C p.190):
 * TIME3 = 37777, TIME4 = 37775, TIME5 = 37774, then STARTSB2.         */
static void startsb1(void)
{
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return;
    }

    waitlist_reset_time3();
    rupt_reset_timers();

    if (is_lm())
    {
        /* delegated (dap-lm): RCSFLAGS bit 13, T6NEXT, NXT6ADR, NEXTP,
         * DAPBOOLS ACCSOKAY.                                            */
        agc_chan_and(AGC_CHAN_CHAN13, 037777u); /* WAND POSMAX: T6 off   */
        agc_rupt_set_t5_task(AGC_TASK_DAPIDLER);
    }

    startsb2();
}

/* STARTSUB (C p.190 / L p.220).                                       */
static void startsub(void)
{
    /* delegated (sys-misc): DNTMGOTO = DNPHASE1, restart the downlist.
     * LM, delegated (prog-rendz): RADMODES = chan 33 bit 6 + RMODINIT.  */
    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return;
    }

    startsb1();

    if (!AGC_ASSERT(agc_exec_idle()))
    {
        return;
    }
}

/* DOFSTART tail common to both vehicles: groups, MODREG, IMU words.   */
static void dofstart_common(void)
{
    /* "The IMU was in coarse align in gimbal lock, so put it back"
     * (DOFSTART C p.184 / L p.212).                                       */
    uint16_t lamps = agc_erase_dsptab(FS_LAMPWORD);

    if (!AGC_ASSERT(lamps <= 077777u))
    {
        return;
    }

    if ((~lamps & FS_BITS4_6) == 0U)
    {
        agc_chan_or(AGC_CHAN_CHAN12, FS_BITS4_6);
    }

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return;
    }

    agc_status_t st = agc_phase_mr_klean(); /* TC MR.KLEAN          */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }

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
    uint16_t moons = (uint16_t) (agc_flag_mask(AGC_FLAG_CMOONFLG) | agc_flag_mask(AGC_FLAG_LMOONFLG) | agc_flag_mask(AGC_FLAG_SURFFLAG));
    uint16_t refsm = agc_flag_mask(AGC_FLAG_REFSMFLG);

    for (int w = 0; w < FS_FLAGWORDS_SWINIT; w++)
    {
        uint16_t keep = 0;
        uint16_t init = 0;

        if (w == 3)
        {
            keep = refsm;
            init = is_lm() ? 002000u : 0u;
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

    if (!AGC_ASSERT(refsm != 0U))
    {
        return;
    }

    if (!AGC_ASSERT(moons != 0U))
    {
        return;
    }

    /* ENDRSTRT: on to DUMMYJOB +2 (RELINT, NEWJOB untouched).          */
    if (!AGC_ASSERT(restreg == FS_PRIO30))
    {
        return;
    }
}

/* DOFSTART (C p.183): outputs off and fresh-start initialisation.     */
static void dofstart_csm(void)
{
    if (!AGC_ASSERT(!is_lm()))
    {
        return;
    }

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

    if (!AGC_ASSERT(agc_rupt_t5_task() == AGC_TASK_T5IDLOC))
    {
        return;
    }
}

/* DOFSTRT1 (L p.211): as DOFSTART but the engine is not touched.      */
static void dofstrt1_lm(void)
{
    if (!AGC_ASSERT(is_lm()))
    {
        return;
    }

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

    if (!AGC_ASSERT(agc_erase_modreg() == EXEC_MODREG_NEG0))
    {
        return;
    }
}

/* DOFSTART (engine_off) or, LM only, DOFSTRT1 (!engine_off: PTBAD and a
 * bad ERESTORE leave the engine alone), then the other modules' resets. */
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

    agc_status_t st = exec_up_dofstart(slap1); /* other modules' erasables */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }

    if (!AGC_ASSERT(restreg == FS_PRIO30))
    {
        return;
    }
}

/* LIGHTSET (C p.189 / L p.219): MARK REJECT together with ERROR LIGHT
 * RESET forces a fresh start. Returns true when it did one.           */
static bool lightset(void)
{
    uint16_t nav = agc_chan_read(AGC_CHAN_NAVKEYIN);

    if (!AGC_ASSERT(nav <= AGC_CHAN_MASK))
    {
        return (false);
    }

    uint16_t mark_reject = is_lm() ? FS_BIT5 : FS_BIT7;

    if ((nav & mark_reject) == 0U)
    {
        return (false); /* NONAVKEY: TC Q       */
    }

    bool elr = !is_lm() && (nav & FS_KEYCODE) == FS_ELR; /* nav DSKY  */

    if (!elr)
    {
        elr = agc_chan_read(AGC_CHAN_MNKEYIN) == FS_ELR; /* main DSKY */
    }

    if (!elr)
    {
        return (false);
    }

    startsub();
    dofstart(false, true);

    if (!AGC_ASSERT(agc_erase_modreg() == EXEC_MODREG_NEG0))
    {
        return (false);
    }

    return (true);
}

/* PCLOOP .. PTBAD (C p.188 / L p.218): every group's -PHASEn must be
 * the complement of PHASEn, else alarm 1107 and a fresh start.        */
static bool phase_tables_ok(void)
{
    bool ok = true;

    for (int g = AGC_RESTART_GROUPS; g >= 1 && ok; g--)
    {
        ok = phase_group_agrees(g);
    }

    if (!ok)
    {
        agc_alarm(AGC_ALARM_01107); /* PTBAD        */
        dofstart(false, false);     /* LM DOFSTRT1, CSM DOFSTART */
    }

    if (!AGC_ASSERT(agc_interrupts_inhibited()))
    {
        return (false);
    }

    return (ok);
}

/* GOPROG3 (C pp.188-189 / L pp.218-219).                              */
static agc_status_t goprog3(void)
{
    if (!phase_tables_ok())
    {
        return (AGC_STATUS_ALARM);
    }

    agc_status_t st = fresh_mmdsplay(); /* TC MMDSPLAY          */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BAILOUT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (is_lm())
    {
        flagword_init(1, (uint16_t) ~024000u & 077777u, 0u); /* DIDFLAG, RODFLAG */
        flagword_init(0, (uint16_t) ~002000u & 077777u, 0u); /* P21FLAG */
    }
    else if ((agc_flagword_get(6) & FS_F6_DAPBITS) == FS_F6_RCS)
    {
        st = exec_up_stoprate(); /* RCS DAP: STOPRATE    */

        if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
        {
            return (AGC_STATUS_BAD_ARG);
        }
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
}

/* ELRSKIP .. NOCOARSE (C pp.187-188): the CSM hardware-restart part. */
static void goprog_csm_tail(void)
{
    static const agc_task_id_t t5idler[4] = {
        /* T5IDLER 2CADRs       */
        AGC_TASK_T5IDLOC, AGC_TASK_REDORCS, AGC_TASK_REDOTVC, AGC_TASK_REDOSAT
    };
    unsigned                   sel        = (unsigned) (agc_flagword_get(6) >> 13) & 3u; /* bits 15,14 */

    agc_rupt_set_t5_task(t5idler[sel]); /* T5LOC     */
    agc_status_t st = AGC_STATUS_OK;

    agc_flag_clear(AGC_FLAG_INTFLAG); /* RASFLAG INTFLBIT     */
    st = exec_up_goprog_optics();     /* OPTMODES = (OPTMODES & OPTINITR) + BIT7 */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }

    agc_erase_set_imodes33((uint16_t) ((agc_erase_imodes33() & FS_BIT6) | FS_IM33INIT));
    uint16_t old = agc_erase_dsptab(FS_LAMPWORD);

    set_lampword((uint16_t) ((old & FS_LAMPS_964) | FS_BIT15));

    if ((old & FS_BIT4) != 0U)
    { /* NO ATT lamp was on   */
        st = exec_up_setcoars();

        if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
        {
            return;
        }

        const agc_status_t st_ca_ece = agc_waitlist_call(FS_CA_ECE_CS, AGC_TASK_CA_ECE);

        if (!AGC_ASSERT(st_ca_ece != AGC_STATUS_BAD_ARG))
        {
            return;
        }
    }

    agc_erase_set_imodes30((uint16_t) ((agc_erase_imodes30() & FS_IFAILINH) | FS_IM30INIR));

    if (agc_flag_test(AGC_FLAG_ENGONFLG))
    {
        agc_chan_or(AGC_CHAN_DSALMOUT, FS_BIT13); /* turn engine on       */
    }
}

/* SETINFL .. GOPROG3 (L p.217): the LM hardware-restart part.         */
static void goprog_lm_tail(void)
{
    if (!AGC_ASSERT(is_lm()))
    {
        return;
    }

    agc_flag_clear(AGC_FLAG_INTFLAG);
    uint16_t old = agc_erase_dsptab(FS_LAMPWORD);

    set_lampword((uint16_t) ((old & FS_LAMPS_964) | FS_BIT15));
    agc_erase_set_imodes30((uint16_t) ((agc_erase_imodes30() & FS_IFAILINH) | FS_IM30INIR));

    /* delegated (sys-misc): DNLSTCOD = AGSWORD                         */
    agc_chan_or(AGC_CHAN_CHAN14, FS_BIT4); /* throttle counter on  */

    if (agc_flag_test(AGC_FLAG_ENGONFLG))
    {
        agc_chan_or(AGC_CHAN_DSALMOUT, FS_BIT13); /* turn engine on       */
    }
    else
    {
        agc_chan_or(AGC_CHAN_DSALMOUT, FS_BIT14); /* turn engine off      */
    }

    if (!AGC_ASSERT(!agc_flag_test(AGC_FLAG_INTFLAG)))
    {
        return;
    }
}

/* GOPROG (C pp.186-188 / L pp.216-217): hardware restart from GOJAM.  */
agc_status_t agc_restart_goprog(void)
{
    if (!AGC_ASSERT(redoctr < UINT16_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    redoctr = (uint16_t) (redoctr + 1U); /* INCR REDOCTR         */
    agc_inhint();

    if (!is_lm())
    {
        exec_vac5stor();

        /* OSC FAIL (ch 33 bit 15) absent and AGC WARNING (bit 14) on:
         * a restart loop; do a fresh start. Channel 33 reads 0 when the
         * signal is present. DEVIATION: the WAND that resets the
         * channel 33 flip-flops is left to the sim harness.            */
        uint16_t ch33 = agc_chan_read(AGC_CHAN_CHAN33);

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

    agc_status_t st = AGC_STATUS_OK;

    if (!lightset())
    { /* BUTTONS              */
        startsub(); /* ERESTORE = +0        */

        if (is_lm())
        {
            goprog_lm_tail();
        }
        else
        {
            goprog_csm_tail();
        }

        st = goprog3();
    }

    agc_relint();

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* ENEMA / GOPROG2 (C p.188 / L p.217): software restart.              */
agc_status_t fresh_enema(void)
{
    agc_inhint();
    agc_status_t st = AGC_STATUS_OK;

    if (is_lm())
    {
        startsb1(); /* TC STARTSB1          */

        if (lightset())
        { /* GOPROG2A             */
            agc_relint();

            return (AGC_STATUS_OK);
        }

        flagword_init(10, (uint16_t) ~FS_INTMASK & 077777u, 0u); /* RSFLGBTS */
        st = goprog3();
        agc_relint();

        return (st);
    }

    if (lightset())
    {
        agc_relint();

        return (st);
    }

    startsb2();
    flagword_init(10, (uint16_t) ~FS_INTMASK & 077777u, 0u); /* INTMASK  */

    if ((agc_flagword_get(6) & FS_F6_DAPBITS) == FS_F6_TVC)
    {
        const agc_status_t st_tvc = agc_waitlist_call(FS_HALF_SEC, AGC_TASK_TVCEXEC);

        if (!AGC_ASSERT(st_tvc != AGC_STATUS_BAD_ARG)) /* TVC on */
        {
            return (AGC_STATUS_BAD_ARG);
        }
    }

    st = goprog3();
    agc_relint();

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* SLAP1 (C p.183 / L p.211): fresh start requested by V36.            */
agc_status_t agc_fresh_start_slap1(void)
{
    agc_inhint();
    startsub();

    /* STARTSW: TCF SKIPSIM (the simulation start patch is not used).   */
    uint16_t lamps = agc_erase_dsptab(FS_LAMPWORD);

    set_lampword((uint16_t) ((lamps & FS_BITS4_6) | FS_BIT15));

    /* delegated: LM DUMPCNT = BIT12 (sys-misc); DSRUPTSW = -PRIO12.    */
    ercount = 0;

    for (int i = 0; i < 3; i++)
    {
        agc_status_t st = agc_erase_set_failreg(i, 0);

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            agc_relint();

            return (AGC_STATUS_BAD_ARG);
        }
    }

    redoctr = 0;
    dofstart(true, true);
    agc_relint();

    if (!AGC_ASSERT(ercount == 0U))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* DOFSTART entry (machine-initiated fresh start).                     */
agc_status_t agc_fresh_dofstart(void)
{
    if (!AGC_ASSERT(redoctr < UINT16_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    dofstart(false, true);
    agc_relint();

    if (!AGC_ASSERT(agc_erase_modreg() == EXEC_MODREG_NEG0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

uint16_t agc_restart_redoctr(void)
{
    if (!AGC_ASSERT(restreg <= 037))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(ercount < UINT16_MAX))
    {
        return (0); /* no valid result */
    }

    return (redoctr);
}

/* GOTOPOOH (C p.194 / L p.223): protect the P00 request by restart
 * group 4 (CSM 4.1 = redisplay, LM 4.33 = GOP00FIX) and go to GOP00FIX.
 * Called from a job, the job continues as GOP00FIX (the caller returns
 * at once). Called from restart code (no job running), GOP00FIX is
 * started as a NOVAC job at the display restart priority (RESTREG).   */
agc_status_t agc_exec_gotopooh(void)
{
    if (!AGC_ASSERT(agc_erase_modreg() >= EXEC_MODREG_NEG0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st;

    if (is_lm())
    {
        agc_inhint();
        phase_set_fixed(4, FS_PHASE_4_33); /* DXCH -PHASE4         */
        agc_relint();
    }
    else
    {
        agc_phase_change(4, FS_PHASE_4_1); /* TC PHASCHNG OCT 14   */
    }

    if (exec_in_job())
    {
        st = agc_exec_job_yield(AGC_JOB_GO_IDLE_FIX);
    }
    else
    {
        st = agc_exec_novac(restreg > 0U ? restreg : FS_PRIO30, AGC_JOB_GO_IDLE_FIX);
    }

    if (!AGC_ASSERT(phase_group_agrees(4)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

agc_status_t agc_job_gotopooh(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx->id == AGC_JOB_GOTOPOOH))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_exec_gotopooh());
}
