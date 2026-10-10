/*
 * pinball_monitor.c -- PINBALL monitor verbs, proceed/terminate/
 * resequence, request executive/waitlist, test lights, major mode change.
 *
 * PINBALL_GAME_BUTTONS_AND_LIGHTS.agc (CM pp. 355-358, 365-367, 385-387;
 * LM pp. 437-440, 447-449, 467-469): MONITOR, MONIT1, MONIT2, MONREQ,
 * KILLMON, MONDO, MONBUSY, PASTEVB, PASTEOPT, DSPFMEM, MMCHANG, REQMM,
 * VBRQEXEC, VBRQWAIT, REQUESTC, VBPROC, VBTERM, VBRESEQ, VBTSTLTS,
 * TSTLTS1, TSTLTS2, TSTLTS3.
 */
#include <stddef.h>
#include "pinball_internal.h"

#define PIN_FULLDSP   05675u          /* FULLDSP: display all 8's           */
#define PIN_FULLDSP1  07675u          /* FULLDSP1: all 8's and +            */
#define PIN_TSTCON1   00175u          /* uplink activity, temp, key rlse,
                                       * V/N flash, operator error          */
#define PIN_TSTCON2_CSM 040650u       /* DSPTAB+11D bits 4,6,8,9: no att,
                                       * gimbal lock, tracker, prog alm     */
#define PIN_TSTCON2_LM  040674u       /* bits 3,4,5,6,8,9: also LR lites    */
#define PIN_TSTCON3   00115u          /* chan 11 bits 1,3,4,7: uplink
                                       * activity, temp, operator error     */
#define PIN_13_11_1   016001u         /* 13-11,1                            */
#define PIN_PRIO16_BITS 016000u       /* PRIO16                             */
#define PIN_OCT55000  055000u
#define PIN_PRIO15_BITS 015000u       /* PRIO15                             */
#define PIN_RFAILS2   0330u           /* RFAILS2 (LM): radar CDU, data fail */
#define PIN_RCDUFBIT  0100u           /* RCDUFBIT (LM) = BIT7               */
#define PIN_MONREF    10              /* MONREF: -DEC10 starting in bit 8   */

/* ---- MONITOR (pp. 355-357) -------------------------------------------- */

/* MONITOR: verbs 11-17. MONSAVE gets the monitor verb and noun, MONSAVE1
 * the NOUNCADR with the kill bit (15) off and, for a keyed monitor, the
 * external monitor bit (14) on.                                          */
agc_status_t pin_monitor(void)
{
    pin_state_t *s = pin_st();
    if (!AGC_ASSERT(s->verbreg >= 11u && s->verbreg <= 17u)) {
        return AGC_STATUS_BAD_ARG;
    }
    uint16_t m1 = (uint16_t)(s->nouncadr & ~(PIN_BIT15 | PIN_BIT14));
    if (s->entret == PIN_ENT_ENDOFJOB) {
        m1 |= PIN_BIT14;                   /* externally initiated monitor  */
        s->monsave2 = 0;                   /* zero NVMONOPT options         */
    }
    uint16_t m = (uint16_t)(((uint16_t)(s->verbreg & PIN_LOW7) << 7) | s->nounreg);
    s->dsplock = false;                    /* +0 into DSPLOCK so monitor can run */
    agc_status_t st = AGC_STATUS_OK;
    /* Turn off KR lite if CADRSTOR and DSPLIST are both empty. (Lite comes
     * on if new monitor is keyed in over old monitor.)                   */
    if (!pin_cadrstor_full()) {
        st = pin_reldsp1();
    }
    agc_inhint();
    if (s->monsave == 0u && st == AGC_STATUS_OK) {
        st = agc_waitlist_call(1, PIN_TASK_MONREQ);   /* request MONREQ  */
    }
    s->monsave = m;
    s->monsave1 = m1;
    agc_relint();
    return (st == AGC_STATUS_OK) ? pin_entexit() : st;   /* TC ENTRET   */
}

/* MONREQ: called by waitlist every second while the monitor lives. */
agc_status_t agc_task_monreq(void)
{
    pin_state_t *s = pin_st();
    pin_lodsampt();                        /* time is snatched in rupt for N65 */
    if ((s->monsave1 & PIN_BIT15) != 0u) { /* killer bit = 1: KILLMON      */
        s->monsave = 0;
        s->monsave1 = 0;                   /* turn off kill and external bits */
        return AGC_STATUS_OK;
    }
    agc_status_t st = agc_waitlist_call(PIN_MONDEL_CS, PIN_TASK_MONREQ);
    agc_status_t st2 = agc_exec_novac(PIN_CHRPRIO, PIN_JOB_MONDO);
    (void)AGC_ASSERT(st2 == AGC_STATUS_OK || st2 == AGC_STATUS_FULL);
    return (st == AGC_STATUS_OK) ? st2 : st;
}

/* MONDO: called by exec. Change the monitor verb to the display verb
 * (V11-V17 -> V01-V07), set ENTRET to PASTEVB and run the noun test; the
 * ECADR (MCTBS) comes from MONSAVE1 through MPAC+2.                      */
agc_status_t agc_job_mondo(agc_job_ctx_t *ctx)
{
    pin_state_t *s = pin_st();
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if ((s->monsave1 & PIN_BIT15) != 0u) {
        return AGC_STATUS_OK;              /* terminate came since last MONREQ */
    }
    if (s->dsplock) {                      /* MONBUSY: NVSUB is busy        */
        pin_reldspon();
        return AGC_STATUS_OK;
    }
    agc_status_t st = pin_updatnn((uint8_t)(s->monsave & PIN_LOW7));
    int dverb = (int)((s->monsave & PIN_MID7) >> 7) - PIN_MONREF;
    if (st != AGC_STATUS_OK || !AGC_ASSERT(dverb >= 1 && dverb <= 7)) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    s->verbreg = (uint8_t)dverb;
    s->entret = PIN_ENT_PASTEVB;           /* return to PASTEVB after display */
    s->mpac2 = (uint16_t)(s->monsave1 & ~(PIN_BIT15 | PIN_BIT14));
    st = pin_testnn();
    return (st == AGC_STATUS_POODOO) ? st : AGC_STATUS_OK;
}

/* PASTEVB: put the monitor verb (or the NVMONOPT please verb) back into
 * VERBREG and the verb lights; zero REQRET so that pasted verbs can be
 * executed by the operator; process the NVMONOPT blank option.          */
agc_status_t pin_pastevb(void)
{
    pin_state_t *s = pin_st();
    uint16_t paste = (uint16_t)(s->monsave2 & PIN_MID7);    /* paste option */
    uint16_t verb = (paste != 0u) ? (uint16_t)(paste >> 7) : (uint16_t)(s->monsave >> 7);
    if (!AGC_ASSERT(verb <= 99u)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = pin_updatvb((uint8_t)verb);
    s->reqret = PIN_REQ_PASS0;
    s->reqret_neg = false;
    if (st == AGC_STATUS_OK) {
        st = pin_blanksub(s->monsave2);    /* either return goes on (TC +1) */
        st = (st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st;
    }
    (void)AGC_ASSERT(s->entret <= PIN_ENT_PASTEVB);
    return st;                             /* ENDOFJOB                       */
}

/* ---- V27 (p. 358) ----------------------------------------------------- */

/* DSPFMEM: fixed memory display (V27 N02 with the FCADR and superbank bits
 * loaded in noun 26). The fixed rope is not modelled in apollo-c.        */
agc_status_t pin_dspfmem(void)
{
    const pin_state_t *s = pin_st();
    (void)AGC_ASSERT(s->verbreg == 27u);
    (void)AGC_ASSERT(s->nouncadr <= PIN_WORD);
    return AGC_STATUS_UNIMPLEMENTED;
}

/* ---- V30 / V31 (pp. 365-366) ------------------------------------------ */

/* VBRQEXEC: request executive, from noun 26 in DSPTEM1: +0 priority word
 * (bit 1 = 1 for FINDVAC), +1 job, +2 BBCON.
 * DEVIATION: the job is an agc_job_id_t number in DSPTEM1 +1 (there are no
 * fixed-memory addresses); BBCON is ignored.                             */
agc_status_t pin_vbrqexec(void)
{
    pin_state_t *s = pin_st();
    uint16_t w = s->dsptem[0];
    agc_prio_t prio = (agc_prio_t)((w & ~PIN_BIT1) >> 9);   /* PRIO into MPAC+4 */
    agc_job_id_t job = (agc_job_id_t)s->dsptem[1];
    agc_status_t st = pin_reldsp();                          /* REQUESTC    */
    if (!AGC_ASSERT(prio >= 1u && prio <= 037u) || !AGC_ASSERT(job != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    if (st == AGC_STATUS_OK) {
        st = ((w & PIN_BIT1) != 0u) ? agc_exec_findvac(prio, job)  /* SETVAC */
                                    : agc_exec_novac(prio, job);
    }
    agc_relint();
    return st;                                               /* ENDOFJOB    */
}

/* VBRQWAIT: request waitlist, DSPTEM1 = time delay (cs), +1 the task.
 * DEVIATION: the task is an agc_task_id_t number in DSPTEM1 +1.          */
agc_status_t pin_vbrqwait(void)
{
    pin_state_t *s = pin_st();
    uint16_t delay = s->dsptem[0];
    agc_task_id_t task = (agc_task_id_t)s->dsptem[1];
    agc_status_t st = pin_reldsp();
    if (!AGC_ASSERT(delay >= 1u) || !AGC_ASSERT(task != AGC_TASK_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    if (st == AGC_STATUS_OK) {
        st = agc_waitlist_call(delay, task);
    }
    agc_relint();
    return st;
}

/* ---- V32 / V33 / V34 (pp. 366-367) ------------------------------------ */

/* VBPROC +1: LOADSTAT = +1 proceed without data (V33), -1 terminate
 * (V34), -0 data in or resequence (V32). Kill the monitor, release the
 * display, flash off and see if there is any recall from ENDIDLE.       */
agc_status_t pin_vbproc_common(pin_loadstat_t ls)
{
    pin_state_t *s = pin_st();
    if (!AGC_ASSERT(ls >= PIN_LS_PROCEED && ls <= PIN_LS_DATAIN)) {
        return AGC_STATUS_BAD_ARG;
    }
    s->loadstat = ls;
    pin_kilmonon();                        /* turn on kill monitor bit      */
    agc_status_t st = pin_reldsp();
    pin_flashoff();
    return (st == AGC_STATUS_OK) ? pin_recaltst() : st;
}

/* ---- V37 (p. 365) ----------------------------------------------------- */

/* REQMM: request two decimal characters for the major mode in the noun
 * lights (ENTPASHI assumes the TC REQMM at MMCHANG).                    */
agc_status_t pin_reqmm(void)
{
    pin_state_t *s = pin_st();
    s->reqret = PIN_REQ_MM;
    s->reqret_neg = true;
    pin_set_dspcount(PIN_ND1);
    s->nounreg = 0;
    agc_status_t st = pin_2blank();
    pin_flashon();
    s->decbrnch = 1u;                      /* set for dec                   */
    (void)AGC_ASSERT(s->dspcount == PIN_ND1);
    return (st == AGC_STATUS_OK) ? pin_entexit() : st;
}

/* MMCHANG +1: demand that 2 numerical characters were punched in
 * (DSPCOUNT = -ND2), blank the noun lights and go to V37 (MODROUTB).    */
agc_status_t pin_mmchang_resume(void)
{
    pin_state_t *s = pin_st();
    if (!s->dspcount_neg || s->dspcount != PIN_ND2) {
        return pin_almcycle();             /* DSPCOUNT not = -ND2           */
    }
    uint8_t mm = s->nounreg;
    s->nounreg = 0;
    pin_set_dspcount(PIN_ND1);
    agc_status_t st = pin_2blank();
    pin_block_dspcount(PIN_VD1);           /* block num char in             */
    if (!AGC_ASSERT(mm <= 99u) || st != AGC_STATUS_OK) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    return agc_v37_change_program(mm);
}

/* ---- V35 test lights (pp. 385-387) ------------------------------------ */

/* VBTSTLTS: all 8's and the + signs, every lamp on, test alarm outbit on;
 * TSTLTS2 runs 5 s later. DSPLOCK is left busy (from keyboard action)
 * until TSTLTS3 to insure that the lights test will be seen.            */
agc_status_t pin_vbtstlts(void)
{
    pin_state_t *s = pin_st();
    bool csm = agc_state_vehicle() == AGC_VEHICLE_CSM;
    if (csm && agc_erase_modreg() != 0) {          /* CHKPOOH (CM only)   */
        pin_falton();                              /* ALM/END             */
        return pin_pinbrnch();
    }
    agc_inhint();
    /* Set bit 1 of IMODES33 so IMUMON won't turn out any lamps.          */
    agc_erase_set_imodes33((uint16_t)(agc_erase_imodes33() | PIN_BIT1));
    (void)agc_chan_or((int)AGC_CHAN_DSALMOUT, PIN_TSTCON1);
    agc_status_t st = agc_erase_set_dsptab(11, csm ? PIN_TSTCON2_CSM : PIN_TSTCON2_LM);
    (void)agc_chan_or((int)AGC_CHAN_CHAN13, PIN_CH13_TESTALM);
    for (int i = PIN_DSPTAB_PANEL - 1; i >= 0 && st == AGC_STATUS_OK; i--) {   /* TSTLTS1 */
        st = agc_erase_set_dsptab(i, pin_ones_neg(PIN_FULLDSP));
    }
    static const int plus_rel[3] = { 1, 4, 6 };    /* turn on 3 plus signs */
    for (int i = 0; i < 3 && st == AGC_STATUS_OK; i++) {
        st = agc_erase_set_dsptab(plus_rel[i], pin_ones_neg(PIN_FULLDSP1));
    }
    s->nout = PIN_DSPTAB_PANEL;                    /* CAF ELEVEN TS NOUT  */
    if (st == AGC_STATUS_OK) {
        st = agc_waitlist_call(PIN_SHOLTS_CS, PIN_TASK_TSTLTS2);
    }
    agc_relint();
    return st;
}

/* TSTLTS2: called by waitlist. */
agc_status_t agc_task_tstlts2(void)
{
    agc_status_t st = agc_exec_novac(PIN_CHRPRIO, PIN_JOB_TSTLTS3);
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL);
    return st;
}

/* TSTLTS3, lamp part: turn off uplink activity, temp, operator error and
 * the test alarm outbit; make NO ATT follow bit 4 of channel 12 (no att
 * light on if in coarse align); set bits to indicate all lamps out.     */
static agc_status_t pin_tstlts3_lamps(void)
{
    agc_inhint();
    (void)agc_chan_and((int)AGC_CHAN_DSALMOUT, (uint16_t)(PIN_WORD ^ PIN_TSTCON3));
    (void)agc_chan_and((int)AGC_CHAN_CHAN13, (uint16_t)(PIN_WORD ^ PIN_CH13_TESTALM));
    uint16_t c = (uint16_t)((agc_chan_read((int)AGC_CHAN_CHAN12) & PIN_CH12_COARSE) | PIN_BIT15);
    agc_status_t st = agc_erase_set_dsptab(11, c);
    agc_erase_set_imodes33((uint16_t)((agc_erase_imodes33() & ~PIN_13_11_1) | PIN_PRIO16_BITS));
    agc_erase_set_imodes30((uint16_t)((agc_erase_imodes30() & ~PIN_OCT55000) | PIN_PRIO15_BITS));
    if (agc_state_vehicle() == AGC_VEHICLE_CSM) {
        agc_erase_set_optmodes((uint16_t)(agc_erase_optmodes() | PIN_BIT7));
    } else {
        agc_erase_set_radmodes((uint16_t)((agc_erase_radmodes() & ~PIN_RFAILS2) | PIN_RCDUFBIT));
    }
    agc_relint();
    (void)AGC_ASSERT(c >= PIN_BIT15);
    return st;
}

/* TSTLTS3: called by executive. Redisplay C(MODREG), kill monitor, flash
 * off, then TSTLTS4 (RELDSP, and PINBRNCH if ENDIDLE is awaiting operator
 * response).                                                            */
agc_status_t agc_job_tstlts3(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = pin_tstlts3_lamps();
    if (st == AGC_STATUS_OK) {
        st = agc_dsky_dspmm();
    }
    pin_kilmonon();
    pin_flashoff();
    agc_status_t st4 = pin_tstlts4();
    return (st == AGC_STATUS_OK) ? st4 : st;
}
