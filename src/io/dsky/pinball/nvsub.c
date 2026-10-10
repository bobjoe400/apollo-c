/*
 * nvsub.c -- PINBALL internal program interface: NVSUB and its
 * relatives, ENDIDLE, the display-wait lists, RECALTST, RELDSP.
 *
 * PINBALL_GAME_BUTTONS_AND_LIGHTS.agc CM pp. 369-377, 380-383 (LM pp.
 * 451-459, 462-465): NVSUB, NVMONOPT, NVSBCOM, NVSUBCOM, NVSUBEND,
 * BLANKDSP, NVSUB1, NVSUB2, KILMONON, ENDIDLE, ISCADR+0, ISLIST+0,
 * DSPABORT, JAMTERM, JAMPROC, BLANKSUB, BLNKSUB1, TESTBIT, DSPMM,
 * RECALTST, RECAL1-3, DOTERM, DOPROC, PRENVBSY, NVSUBUSY, NVSUBSY1,
 * NVSBWAIT, RELDSP, RELDSP1, RELDSP2.
 *
 * Sleeping. ENDIDLE and NVSUBUSY put the calling job to sleep exactly as
 * the AGC does (agc_exec_job_sleep): the job keeps its core set and must
 * return (ENDOFJOB in C) right after. The AGC keeps the return CADR in
 * CADRSTOR or DSPLIST and wakes the job there (RECALTST adds 0, 1 or 2 to
 * its LOC for terminate, proceed, data in); here CADRSTOR keeps the three
 * continuation job ids and RECAL2 wakes the job at the chosen one
 * (agc_exec_job_wake_as, requested from exec-sched).
 */
#include <stddef.h>

#include "internal.h"

/* ---- interlocks ------------------------------------------------------- */

/* MONSAVE1 external monitor bit or DSPLOCK: display system blocked. */
bool pin_dsp_blocked(void)
{
    (void) AGC_ASSERT(pin_invariants());
    const pin_state_t *s = pin_st();
    return (s->monsave1 & PIN_BIT14) != 0u || s->dsplock;
}

bool pin_cadrstor_full(void)
{
    return pin_st()->cadrstor.used;
}

bool pin_dsplist_full(void)
{
    return pin_st()->dsplist.used;
}

pin_sleeper_t pin_take_sleeper(bool dsplist)
{
    (void) AGC_ASSERT(pin_invariants());
    pin_state_t   *s     = pin_st();
    pin_sleeper_t *slot  = dsplist ? &s->dsplist : &s->cadrstor;
    pin_sleeper_t  taken = *slot;
    pin_sleeper_t  empty = {false, {AGC_JOB_NONE, AGC_JOB_NONE, AGC_JOB_NONE}};
    *slot                = empty; /* XCH with 0      */
    (void) AGC_ASSERT(!slot->used);
    return taken;
}

/* KILMONON: force bit 15 of MONSAVE1 to 1 (the kill monitor bit) and turn
 * off bit 14, the external monitor bit.                                  */
void pin_kilmonon(void)
{
    pin_st()->monsave1 = (uint16_t) PIN_BIT15;
}

/* RELDSP2: turn off key release light, DSPLOCK = +0. */
static void pin_reldsp2(void)
{
    (void) AGC_ASSERT(pin_invariants());
    agc_chan_and((int)AGC_CHAN_DSALMOUT,
                 (uint16_t)(PIN_WORD ^ PIN_CH11_KEYREL));
    pin_st()->dsplock = false;
}

/* RELDSP: set DSPLOCK to +0, turn RELDSP light off, search DSPLIST. */
agc_status_t pin_reldsp(void)
{
    (void) AGC_ASSERT(pin_invariants());
    pin_state_t *s  = pin_st();
    agc_status_t st = AGC_STATUS_OK;
    agc_inhint();
    s->monsave1 =
        (uint16_t) (s->monsave1 & ~PIN_BIT14); /* external monitor bit off */
    if (s->dsplist.used)
    {
        pin_sleeper_t sl = pin_take_sleeper(true);
        st               = agc_exec_job_wake(sl.cont[0]);
        st               = (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
    }
    agc_relint();
    pin_reldsp2();
    (void) AGC_ASSERT(!s->dsplist.used);
    return st;
}

/* RELDSP1: set DSPLOCK to +0, no DSPLIST search. Turn key release light
 * off if DSPLIST is empty; leave it alone if DSPLIST is not empty.      */
agc_status_t pin_reldsp1(void)
{
    (void) AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    if (!s->dsplist.used)
    {
        pin_reldsp2();
    }
    else
    {
        s->dsplock = false;
    }
    (void) AGC_ASSERT(!s->dsplock);
    return AGC_STATUS_OK;
}

agc_status_t agc_dsky_releasedsp(void)
{
    return pin_reldsp();
}

/* ---- sleepers (pp. 372-373, 382) -------------------------------------- */

/* ISCADR+0 / ISLIST+0: abort (code 01206) if CADRSTOR or DSPLIST is not
 * +0: only one job may wait in each.                                     */
static agc_status_t pin_check_lists(void)
{
    (void) AGC_ASSERT(pin_invariants());
    if (pin_cadrstor_full() || pin_dsplist_full())
    {
        (void) agc_poodoo(PIN_ALARM_DSPABORT); /* DSPABORT        */
        return AGC_STATUS_POODOO;
    }
    return AGC_STATUS_OK;
}

/* ENDIDLE: the job sleeps until the operator answers; it continues at t
 * (V34 terminate), p (V33 proceed) or e (data in, V32 recycle). DSPLOCK
 * is not set, so ENDIDLE can be used with an NVSUB-initiated monitor.   */
agc_status_t pin_endidle(agc_job_id_t t, agc_job_id_t p, agc_job_id_t e)
{
    (void) AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    if (!AGC_ASSERT(t != AGC_JOB_NONE && p != AGC_JOB_NONE &&
                    e != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = pin_check_lists();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    s->cadrstor.used    = true;
    s->cadrstor.cont[0] = t;
    s->cadrstor.cont[1] = p;
    s->cadrstor.cont[2] = e;
    return agc_exec_job_sleep(t); /* TC JOBSLEEP     */
}

agc_status_t agc_dsky_endidle(const agc_job_ctx_t *ctx,
                              agc_job_id_t         on_terminate,
                              agc_job_id_t on_proceed, agc_job_id_t on_enter)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ctx->id != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return pin_endidle(on_terminate, on_proceed, on_enter);
}

/* NVSUBUSY / NVSUBSY1: put the job to sleep in DSPLIST, key release light
 * on; RELDSP wakes it at wake_as.                                        */
agc_status_t pin_nvsubusy(agc_job_id_t wake_as)
{
    (void) AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    if (!AGC_ASSERT(wake_as != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = pin_check_lists();
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    pin_reldspon();
    s->dsplist.used    = true;
    s->dsplist.cont[0] = wake_as;
    st                 = agc_exec_job_sleep(wake_as); /* ENDNVBSY        */
    return (st == AGC_STATUS_OK) ? AGC_STATUS_BUSY : st;
}

agc_status_t agc_dsky_nvsubusy(const agc_job_ctx_t *ctx, agc_job_id_t wake_as)
{
    (void) AGC_ASSERT(pin_invariants());
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return pin_nvsubusy(wake_as);
}

/* RECALTST (pp. 376-377): after a completed load, proceed or terminate,
 * wake the job in ENDIDLE at the return LOADSTAT selects and leave it the
 * verb and noun of the response (MPAC, MPAC+1 in the AGC).              */
agc_status_t pin_recaltst(void)
{
    (void) AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    if (!s->cadrstor.used)
    {
        return AGC_STATUS_OK; /* normal exit if keyboard initiated */
    }
    pin_sleeper_t sl = pin_take_sleeper(false); /* RECAL1          */
    agc_inhint();
    if (s->loadstat == PIN_LS_WAITING)
    {
        /* Pathological case exit: woken at its CADR, no RELDSP.          */
        agc_status_t st = agc_exec_job_wake(sl.cont[0]);
        agc_relint();
        return (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
    }
    int which =
        (s->loadstat == PIN_LS_PROCEED) ?
            1 /* DOPROC     */
            :
            ((s->loadstat == PIN_LS_TERMINATE) ? 0 : 2); /* DOTERM / data in */
    agc_status_t st =
        agc_exec_job_wake_as(sl.cont[0], sl.cont[which]); /* RECAL2 */
    s->resp_verb = s->verbreg;
    s->resp_noun = s->nounreg;
    agc_relint();
    (void) AGC_ASSERT(which >= 0 && which <= 2);
    agc_status_t st2 = pin_reldsp(); /* RECAL3          */
    return (st == AGC_STATUS_OK || st == AGC_STATUS_NOT_FOUND) ? st2 : st;
}

/* ---- NVSUB (pp. 370-372) ---------------------------------------------- */

/* BLANKDSP: NVTEMP -4 blanks everything, -3 all but the mode lights, -2
 * also leaves the verb, -1 blanks the R's only; then the matching
 * registers are zeroed, numerical chars blocked, flash off (protect
 * against invisible flash) and REQRET zeroed.                            */
static agc_status_t pin_blankdsp(int code)
{
    (void) AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    if (!AGC_ASSERT(code >= -4 && code <= -1))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = AGC_STATUS_OK;
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
    s->clpass = 0;
    pin_block_dspcount(PIN_VD1);
    pin_flashoff();
    s->reqret     = PIN_REQ_PASS0; /* ENTSET -2       */
    s->reqret_neg = false;
    return st; /* NVSUBEND        */
}

/* NVSUB1 / NVSUB2 for a positive VN: noun +0 just pastes the verb, verb +0
 * just shows the noun, otherwise both are shown and ENTPAS0 executes.  */
static agc_status_t pin_nvsub1(uint16_t nvtemp)
{
    (void) AGC_ASSERT(pin_invariants());
    pin_state_t *s    = pin_st();
    uint8_t      noun = (uint8_t) (nvtemp & PIN_LOW7);
    uint8_t      verb = (uint8_t) ((nvtemp >> 7) & PIN_LOW7);
    s->entret         = PIN_ENT_NVSUBEND; /* set return to NVSUBEND */
    if (nvtemp == 0u || !AGC_ASSERT(verb <= 99u && noun <= 99u))
    {
        return pin_dspalarm();
    }
    if (noun == 0u)
    { /* display verb, then return */
        agc_status_t st = pin_updatvb(verb);
        s->reqret = PIN_REQ_PASS0; /* zero REQRET so that pasted verbs can */
        s->reqret_neg = false;     /* be executed by operator              */
        return st;
    }
    if (verb == 0u)
    {
        return pin_updatnn(noun); /* display noun    */
    }
    agc_status_t st = pin_updatvb(verb);
    if (st == AGC_STATUS_OK)
    {
        st = pin_updatnn(noun);
    }
    s->loadstat   = PIN_LS_WAITING; /* waiting for data */
    s->clpass     = 0;
    s->reqret     = PIN_REQ_PASS0; /* REQRET for pass 0 */
    s->reqret_neg = false;
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    st = pin_entpas0();
    return (st == PIN_STATUS_RECYCLE) ? pin_enter() : st; /* ALMCYCLE  */
}

/* NVMONOPT: NVSUB with options (MONSAVE2): bits 8-14 a please verb to
 * paste after a monitor display, bits 1-3 registers to blank.            */
agc_status_t pin_nvmonopt(uint16_t nvtemp, uint16_t options)
{
    if (!AGC_ASSERT(nvtemp <= PIN_POSMAX) || !AGC_ASSERT(options <= PIN_WORD))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (pin_dsp_blocked())
    {
        return AGC_STATUS_BUSY; /* DSP syst blocked: ret to 1+ calling loc */
    }
    pin_st()->monsave2 = options; /* store NVMONOPT options          */
    pin_kilmonon();               /* turn on kill monitor bit        */
    return pin_nvsub1(nvtemp);    /* NVSUBEND: ret to 2+ calling loc */
}

uint16_t agc_dsky_vn(uint8_t verb, uint8_t noun)
{
    (void) AGC_ASSERT(verb <= 99u);
    (void) AGC_ASSERT(noun <= 99u);
    return (uint16_t) (((uint16_t) (verb & PIN_LOW7) << 7) | (noun & PIN_LOW7));
}

agc_status_t agc_dsky_nvsub(uint8_t verb, uint8_t noun)
{
    if (!AGC_ASSERT(verb <= 99u) || !AGC_ASSERT(noun <= 99u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return pin_nvmonopt(agc_dsky_vn(verb, noun), 0); /* LXCH 7: zero options */
}

/* NVSUB with the machine address to be specified in MPAC+2. */
agc_status_t agc_dsky_nvsub_mctbs(uint8_t verb, uint8_t noun, uint16_t ecadr)
{
    if (!AGC_ASSERT(ecadr <= PIN_WORD) || !AGC_ASSERT(noun <= 99u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (pin_dsp_blocked())
    {
        return AGC_STATUS_BUSY;
    }
    pin_st()->mpac2 = ecadr;
    return agc_dsky_nvsub(verb, noun);
}

agc_status_t agc_dsky_nvmonopt(uint8_t verb, uint8_t noun, uint16_t options)
{
    if (!AGC_ASSERT(verb <= 99u) || !AGC_ASSERT(noun <= 99u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return pin_nvmonopt(agc_dsky_vn(verb, noun), options);
}

/* NVSUB with a negative NVTEMP: BLANKDSP. */
agc_status_t agc_dsky_nvsub_blank(int code)
{
    (void) AGC_ASSERT(pin_invariants());
    if (!AGC_ASSERT(code >= -4 && code <= -1))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (pin_dsp_blocked())
    {
        return AGC_STATUS_BUSY;
    }
    pin_st()->monsave2 = 0;
    pin_kilmonon();
    pin_st()->entret = PIN_ENT_NVSUBEND;
    return pin_blankdsp(code);
}

/* NVSBWAIT: NVSUB if the display system is free; if busy, sleep in
 * DSPLIST (PRENVBSY) and wake as wake_as, which re-issues the call.     */
agc_status_t agc_dsky_nvsbwait(const agc_job_ctx_t *ctx, uint8_t verb,
                               uint8_t noun, agc_job_id_t wake_as)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(wake_as != AGC_JOB_NONE))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (pin_dsp_blocked())
    {
        return pin_nvsubusy(wake_as); /* NVSBWT1         */
    }
    return agc_dsky_nvsub(verb, noun); /* NVSBCOM         */
}

/* ---- BLANKSUB (pp. 374-375) ------------------------------------------- */

/* BLANKSUB: blank R1 (bit 1), R2 (bit 2), R3 (bit 3) of the code, keeping
 * DSPCOUNT. BUSY (return to 1+ calling loc) if the display is blocked. */
agc_status_t pin_blanksub(uint16_t code)
{
    (void) AGC_ASSERT(pin_invariants());
    static const int16_t rnd1[3] = {PIN_R1D1, PIN_R2D1, PIN_R3D1};
    pin_state_t         *s       = pin_st();
    uint16_t             nvtemp  = (uint16_t) (code & 7u); /* MASK SEVEN      */
    if (pin_dsp_blocked())
    {
        return AGC_STATUS_BUSY;
    }
    int16_t      save     = s->dspcount; /* BLNKSUB1        */
    bool         save_neg = s->dspcount_neg;
    agc_status_t st       = AGC_STATUS_OK;
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
    (void) AGC_ASSERT(nvtemp <= 7u);
    return st;
}

agc_status_t agc_dsky_blanksub(uint16_t code)
{
    (void) AGC_ASSERT(pin_invariants());
    (void) AGC_ASSERT(code <= PIN_WORD);
    return pin_blanksub(code);
}

/* ---- JAMTERM / JAMPROC (p. 373), DSPMM (p. 375) ----------------------- */

/* JAMTERM / JAMPROC: V34 / V33 by program; leave ENTER set for ENTPAS0. */
static agc_status_t pin_jam(pin_loadstat_t ls)
{
    (void) AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    s->reqret      = PIN_REQ_PASS0; /* REQRET = 34 / 33 (positive)    */
    s->reqret_neg  = false;
    pin_block_dspcount(PIN_VD1);
    (void) AGC_ASSERT(ls == PIN_LS_TERMINATE || ls == PIN_LS_PROCEED);
    return pin_vbproc_common(ls);
}

agc_status_t agc_dsky_jamterm(void)
{
    return pin_jam(PIN_LS_TERMINATE);
}

agc_status_t agc_dsky_jamproc(void)
{
    return pin_jam(PIN_LS_PROCEED);
}

agc_status_t agc_job_jamterm(agc_job_ctx_t *ctx)
{
    (void) AGC_ASSERT(pin_invariants());
    if (!AGC_ASSERT(ctx != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = pin_jam(PIN_LS_TERMINATE);
    return (st == AGC_STATUS_POODOO) ? st : AGC_STATUS_OK;
}

/* DSPMM: display MODREG in the program lights through job DSPMMJB. */
agc_status_t agc_dsky_dspmm(void)
{
    (void) AGC_ASSERT(pin_invariants());
    agc_inhint();
    agc_status_t st = agc_exec_novac(PIN_CHRPRIO, PIN_JOB_DSPMMJB);
    agc_relint();
    (void) AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL);
    return st;
}
