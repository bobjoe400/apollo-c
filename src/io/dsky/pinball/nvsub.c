/**********************************************************************
 * FILE NAME: nvsub.c
 *
 * PURPOSE:
 *   PINBALL internal program interface: NVSUB and its relatives, ENDIDLE,
 *   the display-wait lists (CADRSTOR, DSPLIST), RECALTST, RELDSP.
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pp. 369-377, 380-383
 *     (NVSUB, NVMONOPT, NVSBCOM, NVSUBCOM, NVSUBEND, BLANKDSP, NVSUB1,
 *     NVSUB2, KILMONON, ENDIDLE, ISCADR+0, ISLIST+0, DSPABORT, JAMTERM,
 *     JAMPROC, BLANKSUB, BLNKSUB1, TESTBIT, DSPMM, RECALTST, RECAL1-3,
 *     DOTERM, DOPROC, PRENVBSY, NVSUBUSY, NVSUBSY1, NVSBWAIT, RELDSP,
 *     RELDSP1, RELDSP2)
 *   Luminary099 pp. 451-459, 462-465 (identical)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale  Description
 *   ------------  --------  ---  -----  ------------------------------
 *   channel 11    uint16_t  O    bits   key release lamp, V/N flash
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  --------------------------------------------
 *   agc_exec_job_sleep    ENDIDLE / NVSUBUSY (JOBSLEEP)
 *   agc_exec_job_wake     RELDSP wakes the DSPLIST sleeper
 *   agc_exec_job_wake_as  RECAL2: wake at terminate/proceed/data in
 *   agc_exec_novac        DSPMMJB, JAMTERM jobs
 *   agc_poodoo            DSPABORT 01206, DSPALARM 01501
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   POODOO 01206 when a second job would wait in CADRSTOR or DSPLIST;
 *   POODOO 01501 for an illegal internal verb/noun; AGC_STATUS_BUSY for
 *   "display system blocked" (L+1 return). BAD_ARG on a failed
 *   precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the calling program's job. ENDIDLE and NVSUBUSY put that job
 *   to sleep: it must return (ENDOFJOB in C) right after. Both vehicles.
 *
 * ALGORITHM:
 *   ENDIDLE and NVSUBUSY put the calling job to sleep exactly as the AGC
 *   does (agc_exec_job_sleep): the job keeps its core set. The AGC keeps
 *   the return CADR in CADRSTOR or DSPLIST and wakes the job there
 *   (RECALTST adds 0, 1 or 2 to its LOC for terminate, proceed, data in);
 *   here CADRSTOR keeps the three continuation job ids and RECAL2 wakes the
 *   job at the chosen one (agc_exec_job_wake_as).
 **********************************************************************/
#include <stddef.h>

#include "internal.h"

/* ---- interlocks ------------------------------------------------------- */

/**********************************************************************
 * AGC: NVSUB (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 370)
 *
 * MONSAVE1 external monitor bit or DSPLOCK: display system blocked.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when the display system is blocked
 **********************************************************************/
bool pin_dsp_blocked(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (false);
    }

    const pin_state_t *s = pin_st(); /* pinball erasables */

    return ((s->monsave1 & PIN_BIT14) != 0u || s->dsplock);
}

/**********************************************************************
 * AGC: ISCADR+0 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 373)
 *
 * CCS CADRSTOR: a job waits in ENDIDLE.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when a job waits in CADRSTOR (ENDIDLE)
 **********************************************************************/
bool pin_cadrstor_full(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (false);
    }

    return (pin_st()->cadrstor.used);
}

/**********************************************************************
 * AGC: ISLIST+0 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 373)
 *
 * CCS DSPLIST: a job waits for the display system.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when a job waits in DSPLIST
 **********************************************************************/
bool pin_dsplist_full(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (false);
    }

    return (pin_st()->dsplist.used);
}

/**********************************************************************
 * AGC: WITCHONE (DISPLAY_INTERFACE_ROUTINES.agc, CM p. 1478)
 *
 * XCH CADRSTOR / DSPLIST: empty the slot and return its sleeper.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   dsplist   bool  I    true for DSPLIST, false for CADRSTOR
 *
 * RETURNS: pin_sleeper_t -- the slot as it was before emptying
 **********************************************************************/
pin_sleeper_t pin_take_sleeper(bool dsplist)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return ((pin_sleeper_t){0}); /* no valid result */
    }

    pin_state_t   *s     = pin_st();                             /* pinball erasables */
    pin_sleeper_t *slot  = dsplist ? &s->dsplist : &s->cadrstor; /* the list */
    pin_sleeper_t  taken = *slot;                                /* its contents before */
    pin_sleeper_t  empty = {false, {AGC_JOB_NONE, AGC_JOB_NONE, AGC_JOB_NONE}};

    *slot = empty; /* XCH with 0      */

    if (!AGC_ASSERT(!slot->used))
    {
        return ((pin_sleeper_t){0}); /* no valid result */
    }

    return (taken);
}

/**********************************************************************
 * AGC: KILMONON (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 372)
 *
 * KILMONON: force bit 15 of MONSAVE1 to 1 (the kill monitor bit) and turn
 * off bit 14, the external monitor bit.
 *
 * ARGUMENTS: none
 **********************************************************************/
void pin_kilmonon(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return;
    }

    pin_st()->monsave1 = (uint16_t) PIN_BIT15;

    /* external monitor bit off: only DSPLOCK can still block the display */
    if (!AGC_ASSERT(pin_dsp_blocked() == pin_st()->dsplock))
    {
        return;
    }
}

/**********************************************************************
 * AGC: RELDSP2 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 383)
 *
 * RELDSP2: turn off key release light, DSPLOCK = +0.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void pin_reldsp2(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return;
    }

    agc_chan_and((int) AGC_CHAN_DSALMOUT, (uint16_t) (PIN_WORD ^ PIN_CH11_KEYREL));
    pin_st()->dsplock = false;

    if (!AGC_ASSERT((agc_chan_read((int) AGC_CHAN_DSALMOUT) & PIN_CH11_KEYREL) == 0u))
    {
        return; /* key release light still on */
    }
}

/**********************************************************************
 * AGC: RELDSP (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 382)
 *
 * RELDSP: set DSPLOCK to +0, turn RELDSP light off, search DSPLIST.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND; else the status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_releasedsp(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s  = pin_st();      /* pinball erasables */
    agc_status_t st = AGC_STATUS_OK; /* status so far */

    agc_inhint();
    s->monsave1 = (uint16_t) (s->monsave1 & ~PIN_BIT14); /* external monitor bit off */

    if (s->dsplist.used)
    {
        pin_sleeper_t sl = pin_take_sleeper(true); /* the DSPLIST sleeper */

        st = agc_exec_job_wake(sl.cont[0]);
        st = (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st; /* none: OK */
    }

    agc_relint();
    pin_reldsp2();

    if (!AGC_ASSERT(!s->dsplist.used))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * AGC: RELDSP1 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 383)
 *
 * RELDSP1: set DSPLOCK to +0, no DSPLIST search. Turn key release light
 * off if DSPLIST is empty; leave it alone if DSPLIST is not empty.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t pin_reldsp1(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!s->dsplist.used)
    {
        pin_reldsp2();
    }
    else
    {
        s->dsplock = false;
    }

    if (!AGC_ASSERT(!s->dsplock))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* ---- sleepers (pp. 372-373, 382) -------------------------------------- */

/**********************************************************************
 * AGC: ISCADR+0, ISLIST+0 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 373)
 *
 * ISCADR+0 / ISLIST+0: abort (code 01206) if CADRSTOR or DSPLIST is not
 * +0: only one job may wait in each.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_check_lists(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (pin_cadrstor_full() || pin_dsplist_full())
    {
        return (agc_poodoo(PIN_ALARM_DSPABORT)); /* DSPABORT        */
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: ENDIDLE (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 373)
 *
 * ENDIDLE: the job sleeps until the operator answers; it continues at t
 * (V34 terminate), p (V33 proceed) or e (data in, V32 recycle). DSPLOCK
 * is not set, so ENDIDLE can be used with an NVSUB-initiated monitor.
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------
 *   t         agc_job_id_t  I    continuation for V34 terminate
 *   p         agc_job_id_t  I    continuation for V33 proceed
 *   e         agc_job_id_t  I    continuation for data in / V32 recycle
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_endidle(agc_job_id_t t, agc_job_id_t p, agc_job_id_t e)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(t != AGC_JOB_NONE && p != AGC_JOB_NONE && e != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = pin_check_lists(); /* status so far */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    s->cadrstor.used    = true;
    s->cadrstor.cont[0] = t;
    s->cadrstor.cont[1] = p;
    s->cadrstor.cont[2] = e;
    agc_exec_job_sleep(t);

    if (!AGC_ASSERT(pin_cadrstor_full()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* TC JOBSLEEP     */
}

/**********************************************************************
 * AGC: NVSUBUSY, NVSUBSY1 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 382)
 *
 * NVSUBUSY / NVSUBSY1: put the job to sleep in DSPLIST, key release light
 * on; RELDSP wakes it at wake_as.
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------
 *   wake_as   agc_job_id_t  I    continuation the job resumes as when
 *                                woken
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition, BUSY;
 *          else the status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_nvsubusy(agc_job_id_t wake_as)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(wake_as != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = pin_check_lists(); /* status so far */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    pin_reldspon();
    s->dsplist.used    = true;
    s->dsplist.cont[0] = wake_as;
    agc_exec_job_sleep(wake_as); /* ENDNVBSY        */

    if (!AGC_ASSERT(pin_dsplist_full()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_BUSY); /* L+1: display was busy */
}

/**********************************************************************
 * RECALTST (pp. 376-377): after a completed load, proceed or terminate,
 * wake the job in ENDIDLE at the return LOADSTAT selects and leave it the
 * verb and noun of the response (MPAC, MPAC+1 in the AGC).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND; else the status of the routine it calls
 **********************************************************************/
agc_status_t pin_recaltst(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!s->cadrstor.used)
    {
        return (AGC_STATUS_OK); /* normal exit if keyboard initiated */
    }

    pin_sleeper_t sl = pin_take_sleeper(false); /* RECAL1          */

    agc_inhint();

    if (s->loadstat == PIN_LS_WAITING)
    {
        /* Pathological case exit: woken at its CADR, no RELDSP.          */
        agc_status_t st = agc_exec_job_wake(sl.cont[0]); /* status so far */

        agc_relint();

        return ((st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st); /* none: OK */
    }

    int which = 2; /* data in: CADR +3 */

    if (s->loadstat == PIN_LS_PROCEED)
    {
        which = 1; /* DOPROC */
    }
    else if (s->loadstat == PIN_LS_TERMINATE)
    {
        which = 0; /* DOTERM */
    }

    if (!AGC_ASSERT(which >= 0 && which <= 2))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_exec_job_wake_as(sl.cont[0], sl.cont[which]); /* RECAL2 */

    s->resp_verb = s->verbreg;
    s->resp_noun = s->nounreg;
    agc_relint();

    agc_status_t st2 = agc_dsky_releasedsp(); /* RECAL3          */

    return ((st == AGC_STATUS_OK || st == AGC_STATUS_NOT_FOUND) ? st2 : st); /* step 2 */
} /* pin_recaltst */

/* ---- NVSUB (pp. 370-372) ---------------------------------------------- */

/**********************************************************************
 * AGC: BLANKDSP (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 371)
 *
 * BLANKDSP: NVTEMP -4 blanks everything, -3 all but the mode lights, -2
 * also leaves the verb, -1 blanks the R's only; then the matching
 * registers are zeroed, numerical chars blocked, flash off (protect
 * against invisible flash) and REQRET zeroed.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   code      int   I    NVTEMP blank code -4..-1
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_blankdsp(int code)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(code >= -4 && code <= -1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_OK; /* status so far */

    agc_inhint();

    for (int i = 6 - code; i >= 0 && st == AGC_STATUS_OK; i--)
    { /* 7..10 down */
        st = pin_dsptab_store(i, (uint16_t) PIN_BIT12);
    }

    agc_relint();

    if (code <= -3)
    {
        s->verbreg = 0;
    }

    if (code <= -2)
    {
        s->nounreg = 0;
    }

    s->clpass     = 0;
    pin_block_dspcount(PIN_VD1);
    pin_flashoff();
    s->reqret     = PIN_REQ_PASS0; /* ENTSET -2       */
    s->reqret_neg = false;

    if (!AGC_ASSERT(s->dspcount == PIN_VD1 && s->dspcount_neg))
    {
        return (AGC_STATUS_BAD_ARG); /* DSPCOUNT not blocked at VD1 */
    }

    return (st); /* NVSUBEND        */
} /* pin_blankdsp */

/**********************************************************************
 * AGC: NVSUB1, NVSUB2 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 371)
 *
 * NVSUB1 / NVSUB2 for a positive VN: noun +0 just pastes the verb, verb +0
 * just shows the noun, otherwise both are shown and ENTPAS0 executes.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   nvtemp    uint16_t  I    NVTEMP: verb x 2^7 + noun (positive)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          PIN_STATUS_RECYCLE; else the status of the routine it calls
 **********************************************************************/
static agc_status_t pin_nvsub1(uint16_t nvtemp)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s    = pin_st();                             /* pinball erasables */
    uint8_t      noun = (uint8_t) (nvtemp & PIN_LOW7);        /* noun of NVTEMP */
    uint8_t      verb = (uint8_t) ((nvtemp >> 7) & PIN_LOW7); /* verb */

    s->entret = PIN_ENT_NVSUBEND; /* set return to NVSUBEND */

    if (nvtemp == 0u || !AGC_ASSERT(verb <= 99u && noun <= 99u))
    {
        return (pin_dspalarm());
    }

    if (noun == 0u)
    { /* display verb, then return */
        agc_status_t st = pin_updatvb(verb); /* status so far */

        /* zero REQRET so that pasted verbs can */
        s->reqret     = PIN_REQ_PASS0;
        s->reqret_neg = false; /* be executed by operator              */

        return (st);
    }

    if (verb == 0u)
    {
        return (pin_updatnn(noun)); /* display noun    */
    }

    agc_status_t st = pin_updatvb(verb); /* status so far */

    if (st == AGC_STATUS_OK)
    {
        st = pin_updatnn(noun);
    }

    s->loadstat   = PIN_LS_WAITING; /* waiting for data */
    s->clpass     = 0;
    s->reqret     = PIN_REQ_PASS0;  /* REQRET for pass 0 */
    s->reqret_neg = false;

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    st = pin_entpas0();

    return ((st == PIN_STATUS_RECYCLE) ? pin_enter() : st); /* ALMCYCLE  */
} /* pin_nvsub1 */

/**********************************************************************
 * AGC: NVMONOPT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 370)
 *
 * NVMONOPT: NVSUB with options (MONSAVE2): bits 8-14 a please verb to
 * paste after a monitor display, bits 1-3 registers to blank.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   nvtemp    uint16_t  I    NVTEMP: verb x 2^7 + noun
 *   options   uint16_t  I    MONSAVE2: please verb in bits 8-14, blank
 *                            bits 1-3
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition, BUSY; else
 *          the status of the routine it calls
 **********************************************************************/
agc_status_t pin_nvmonopt(uint16_t nvtemp, uint16_t options)
{
    if (!AGC_ASSERT(nvtemp <= PIN_POSMAX) || !AGC_ASSERT(options <= PIN_WORD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (pin_dsp_blocked())
    {
        return (AGC_STATUS_BUSY); /* DSP syst blocked: ret to 1+ calling loc */
    }

    pin_st()->monsave2 = options; /* store NVMONOPT options          */
    pin_kilmonon();               /* turn on kill monitor bit        */

    return (pin_nvsub1(nvtemp)); /* NVSUBEND: ret to 2+ calling loc */
}

/**********************************************************************
 * AGC: none (helper)
 *
 * VN word as the AGC codes it (CAF VxxNyy): verb x 2^7 + noun.
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   verb      uint8_t  I    verb 00..99
 *   noun      uint8_t  I    noun 00..99
 *
 * RETURNS: uint16_t -- VN = verb x 2^7 + noun
 **********************************************************************/
uint16_t agc_dsky_vn(uint8_t verb, uint8_t noun)
{
    if (!AGC_ASSERT(verb <= 99u))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(noun <= 99u))
    {
        return (0); /* no valid result */
    }

    return ((uint16_t) (((uint16_t) (verb & PIN_LOW7) << 7) | (noun & PIN_LOW7)));
}

/**********************************************************************
 * AGC: NVSUB (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 370)
 *
 * NVSUB: show verb and noun and execute the verb (L+2 OK, L+1 BUSY).
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   verb      uint8_t  I    verb number 00..99 (VERBREG)
 *   noun      uint8_t  I    noun number 00..99 (NOUNREG)
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_nvsub(uint8_t verb, uint8_t noun)
{
    if (!AGC_ASSERT(verb <= 99u) || !AGC_ASSERT(noun <= 99u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* LXCH 7: zero options */
    return (pin_nvmonopt(agc_dsky_vn(verb, noun), 0));
}

/**********************************************************************
 * AGC: NVSUB (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 370)
 *
 * NVSUB with the machine address to be specified in MPAC+2.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   verb      uint8_t   I    verb number 00..99 (VERBREG)
 *   noun      uint8_t   I    noun number 00..99 (NOUNREG)
 *   ecadr     uint16_t  I    machine address for MCTBS nouns (MPAC+2)
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition, BUSY; else
 *          the status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_nvsub_mctbs(uint8_t verb, uint8_t noun, uint16_t ecadr)
{
    if (!AGC_ASSERT(ecadr <= PIN_WORD) || !AGC_ASSERT(noun <= 99u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (pin_dsp_blocked())
    {
        return (AGC_STATUS_BUSY);
    }

    pin_st()->mpac2 = ecadr;

    return (agc_dsky_nvsub(verb, noun));
}

/**********************************************************************
 * AGC: NVMONOPT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 370)
 *
 * NVSUB with options (please verb, blank bits) for a monitor.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   verb      uint8_t   I    verb number 00..99 (VERBREG)
 *   noun      uint8_t   I    noun number 00..99 (NOUNREG)
 *   options   uint16_t  I    MONSAVE2: please verb in bits 8-14, blank
 *                            bits 1-3
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_nvmonopt(uint8_t verb, uint8_t noun, uint16_t options)
{
    if (!AGC_ASSERT(verb <= 99u) || !AGC_ASSERT(noun <= 99u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_nvmonopt(agc_dsky_vn(verb, noun), options));
}

/**********************************************************************
 * AGC: NVSUB (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 370)
 *
 * NVSUB with a negative NVTEMP: BLANKDSP.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   code      int   I    NVTEMP blank code -4..-1
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition, BUSY; else
 *          the status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_nvsub_blank(int code)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(code >= -4 && code <= -1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (pin_dsp_blocked())
    {
        return (AGC_STATUS_BUSY);
    }

    pin_st()->monsave2 = 0;
    pin_kilmonon();
    pin_st()->entret   = PIN_ENT_NVSUBEND;

    return (pin_blankdsp(code));
}

/**********************************************************************
 * AGC: NVSBWAIT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 382)
 *
 * NVSBWAIT: NVSUB if the display system is free; if busy, sleep in
 * DSPLIST (PRENVBSY) and wake as wake_as, which re-issues the call.
 *
 * ARGUMENTS:
 *   Argument  Type                   I/O  Description (scale)
 *   --------  ---------------------  ---  -------------------
 *   ctx       const agc_job_ctx_t *  I    calling job's context
 *   verb      uint8_t                I    verb number 00..99 (VERBREG)
 *   noun      uint8_t                I    noun number 00..99 (NOUNREG)
 *   wake_as   agc_job_id_t           I    continuation the job resumes as
 *                                         when woken
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_nvsbwait(const agc_job_ctx_t *ctx, uint8_t verb, uint8_t noun, agc_job_id_t wake_as)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(wake_as != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (pin_dsp_blocked())
    {
        return (agc_dsky_nvsubusy(wake_as)); /* NVSBWT1         */
    }

    return (agc_dsky_nvsub(verb, noun)); /* NVSBCOM         */
}

/* ---- BLANKSUB (pp. 374-375) ------------------------------------------- */

/**********************************************************************
 * AGC: BLANKSUB (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 374)
 *
 * BLANKSUB: blank R1 (bit 1), R2 (bit 2), R3 (bit 3) of the code, keeping
 * DSPCOUNT. BUSY (return to 1+ calling loc) if the display is blocked.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   code      uint16_t  I    blank bits: bit 1 R1, bit 2 R2, bit 3 R3
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition, BUSY;
 *          else the status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_blanksub(uint16_t code)
{
    if (!AGC_ASSERT(pin_invariants()) || !AGC_ASSERT(code <= PIN_WORD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    static const int16_t rnd1[3] = {PIN_R1D1, PIN_R2D1, PIN_R3D1};
    pin_state_t         *s       = pin_st();               /* pinball erasables */
    uint16_t             nvtemp  = (uint16_t) (code & 7u); /* MASK SEVEN      */

    if (pin_dsp_blocked())
    {
        return (AGC_STATUS_BUSY);
    }

    int16_t      save     = s->dspcount;     /* BLNKSUB1        */
    bool         save_neg = s->dspcount_neg; /* blocked flag */
    agc_status_t st       = AGC_STATUS_OK;   /* status so far */

    for (int r = 0; r < 3 && st == AGC_STATUS_OK; r++)
    {
        if ((nvtemp & (1u << r)) != 0u)
        { /* TESTBIT         */
            pin_set_dspcount(rnd1[r]);
            st = pin_5blank();
        }
    }

    s->dspcount     = save; /* restore DSPCOUNT to state it had */
    s->dspcount_neg = save_neg;

    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* ---- JAMTERM / JAMPROC (p. 373), DSPMM (p. 375) ----------------------- */

/**********************************************************************
 * AGC: JAMTERM, JAMPROC
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 373-374)
 *
 * JAMTERM / JAMPROC: V34 / V33 by program; leave ENTER set for ENTPAS0.
 *
 * ARGUMENTS:
 *   Argument  Type            I/O  Description (scale)
 *   --------  --------------  ---  -------------------
 *   ls        pin_loadstat_t  I    LOADSTAT for V34 (terminate) or V33
 *                                  (proceed)
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_jam(pin_loadstat_t ls)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    s->reqret     = PIN_REQ_PASS0; /* REQRET = 34 / 33 (positive)    */
    s->reqret_neg = false;
    pin_block_dspcount(PIN_VD1);

    if (!AGC_ASSERT(ls == PIN_LS_TERMINATE || ls == PIN_LS_PROCEED))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_vbproc_common(ls));
}

/**********************************************************************
 * AGC: JAMTERM (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 373)
 *
 * V34 by program: start the JAMTERM job (FINDVAC, PRIO32).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK; else the status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_jamterm(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_jam(PIN_LS_TERMINATE));
}

/**********************************************************************
 * AGC: JAMPROC (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 374)
 *
 * V33 by program, at once.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK; else the status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_jamproc(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_jam(PIN_LS_PROCEED));
}

/**********************************************************************
 * AGC: JAMTERM (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 373)
 *
 * Job entry of JAMTERM: terminate the display in ENDIDLE.
 *
 * ARGUMENTS:
 *   Argument  Type             I/O  Description (scale)
 *   --------  ---------------  ---  -------------------
 *   ctx       agc_job_ctx_t *  I/O  job context from the dispatcher (id,
 *                                   priority, VAC; MPAC word in arg)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition, POODOO;
 *          else the status of the routine it calls
 **********************************************************************/
agc_status_t agc_job_jamterm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = pin_jam(PIN_LS_TERMINATE); /* status so far */

    return ((st == AGC_STATUS_POODOO) ? st : AGC_STATUS_OK); /* abort only */
}

/**********************************************************************
 * AGC: DSPMM (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 375)
 *
 * DSPMM: display MODREG in the program lights through job DSPMMJB.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition, FULL;
 *          else the status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_dspmm(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_status_t st = agc_exec_novac(PIN_CHRPRIO, AGC_JOB_DSPMMJB); /* status */

    agc_relint();

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}
