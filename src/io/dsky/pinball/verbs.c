/**********************************************************************
 * FILE NAME: verbs.c
 *
 * PURPOSE:
 *   PINBALL verb execution: ENTPAS0, TESTVB, TESTNN, REQADD, USEADD,
 *   INTMCTBS, VERBFAN, VBFANDIR (VERBTAB), the octal display verbs
 *   DSPA..DSPABC with their component tests, LODNNTAB on the agc_noun_t
 *   tables, and the alarm exits DSPALARM, PREDSPAL, CHARALRM, ALMCYCLE.
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pp. 324-333, 364
 *   Comanche055 PINBALL_NOUN_TABLES.agc p. 270 (LODNNTAB)
 *   Luminary099 pp. 406-415, 446 (identical)
 *
 * EXTERNAL VARIABLES:
 *   none (pinball state only)
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  -------------------------------------------
 *   agc_noun_lookup        LODNNTAB: the noun table row
 *   agc_extverb            GOEXTVB: verbs 40-99 (sys-misc)
 *   agc_fresh_start_slap1  V36 fresh start (SLAP1)
 *   agc_poodoo             abort 01501 (DSPALARM from an internal NVSUB)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   POODOO 01501 for an illegal verb/noun from an internal NVSUB; OPR ERR
 *   for a keyed one (CHARALRM); ALMCYCLE recycles (PIN_STATUS_RECYCLE).
 *   BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the CHARIN job (ENTER) or the NVSUB caller's job. V27
 *   (DSPFMEM) is a stub: see monitor.c. Both vehicles.
 **********************************************************************/
#include <stddef.h>

#include "internal.h"

/* ---- LODNNTAB (PINBALL_NOUN_TABLES p. 270) ---------------------------- */

/**********************************************************************
 * AGC: LODNNTAB, TESTNN
 *      (PINBALL_NOUN_TABLES.agc, CM p. 270;
 *      PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 324)
 *
 * Load the noun table row for NOUNREG and classify NNADTEM: +0 not in use,
 * - specify machine address (01-03) or channel (10), -0 increment machine
 * address (15), + normal.
 * DEVIATION: the C noun tables carry data ids, not ECADRs, so NOUNCADR is
 * only meaningful for nouns 01-03, 10, 15; it is zeroed for the others
 * (the AGC loaded the noun's ECADR, used only for the MCTBS cases).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND; else the status of the routine it calls
 **********************************************************************/
agc_status_t pin_noun_load(void)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    agc_status_t st = agc_noun_lookup(s->nounreg, &s->noun); /* status so far */

    if (st == AGC_STATUS_NOT_FOUND || s->noun.ncomp == 0u)
    {
        s->nntype = PIN_NNTYPE_SPARE;
        s->nouncadr = 0;

        return (AGC_STATUS_NOT_FOUND);
    }

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    switch (s->noun.data_id[0])
    {
        case AGC_DATA_SPECIFY_ADDR:
            s->nntype = PIN_NNTYPE_MCTBS;
            break;
        case AGC_DATA_SPECIFY_CHANNEL:
            s->nntype = PIN_NNTYPE_CHAN;
            break;
        case AGC_DATA_INCREMENT_ADDR:
            s->nntype = PIN_NNTYPE_AUGMENT;
            break;
        default:
            s->nntype = PIN_NNTYPE_NORMAL;
            s->nouncadr = 0;
            break;
    }

    return (AGC_STATUS_OK);
} /* pin_noun_load */

/* ---- alarm exits (p. 364) --------------------------------------------- */

/**********************************************************************
 * AGC: CHARALRM (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 364)
 *
 * CHARALRM: not NVSUB initiated, turn on OPR ERROR; ENDOFJOB.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t pin_charalrm(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_falton();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: DSPALARM (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 364)
 *
 * DSPALARM: an illegal verb/noun. From an internal NVSUB this is abort
 * 01501; from a monitor, the monitor is killed and its verb pasted back.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_dspalarm(void)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    if (s->entret == PIN_ENT_NVSUBEND)
    {
        return (agc_poodoo(PIN_ALARM_DSPALARM)); /* TC POODOO, OCT 01501  */
    }

    if (s->entret == PIN_ENT_PASTEVB)
    { /* if this is a monitor, kill it */
        pin_kilmonon();
        agc_dsky_falton();

        return (pin_pastevb()); /* put monitor verb back  */
    }

    return (pin_charalrm());
}

/**********************************************************************
 * AGC: PREDSPAL (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 364)
 *
 * PREDSPAL: alarm if decimal display with an octal-only noun.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_predspal(void)
{
    pin_block_dspcount(PIN_VD1);

    if (!AGC_ASSERT(pin_st()->dspcount_neg))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_dspalarm());
}

/**********************************************************************
 * AGC: ALMCYCLE (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 364)
 *
 * ALMCYCLE: turn on check fail light, put the original verb that was
 * executed back into VERBREG and the verb lights, and re-enter ENTER
 * (the caller's pin_enter loop does the POSTJUMP ENTER).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          PIN_STATUS_RECYCLE; else the status of the routine it calls
 **********************************************************************/
agc_status_t pin_almcycle(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(s->verbsave <= 99))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_falton();
    s->reqret = PIN_REQ_PASS0;            /* set for ENTPAS0        */
    s->reqret_neg = false;
    agc_status_t st = pin_updatvb(s->verbsave); /* status so far */

    return ((st == AGC_STATUS_OK) ? PIN_STATUS_RECYCLE : st); /* if OK */
}

/**********************************************************************
 * AGC: ENTEXIT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 323)
 *
 * TC ENTEXIT: the exit chosen by ENTRET.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_entexit(void)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    if (s->entret == PIN_ENT_PASTEVB)
    {
        return (pin_pastevb());
    }

    return (AGC_STATUS_OK); /* ENDOFJOB / NVSUBEND    */
}

/* ---- component tests (pp. 331-332) ------------------------------------ */

/**********************************************************************
 * AGC: COMPTEST (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 331)
 *
 * COMPTEST: noun comp G/E verb comp (comp numbers 0, 1, 2); else the
 * caller alarms (noun comp L/ verb comp).
 *
 * ARGUMENTS:
 *   Argument   Type  I/O  Description (scale)
 *   ---------  ----  ---  -------------------
 *   verb_comp  int   I    components the verb uses: 0, 1 or 2 (one to
 *                         three)
 *
 * RETURNS: bool -- true when the noun has enough components
 **********************************************************************/
bool pin_comptest(int verb_comp)
{
    const pin_state_t *s = pin_st(); /* pinball erasables */

    return ((int) s->noun.ncomp - 1 >= verb_comp);
}

/**********************************************************************
 * AGC: DECTEST, DCTSTCYC (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 332)
 *
 * DECTEST / DCTSTCYC: the decimal-only bit (bit 4 of the comp code) is 0;
 * else DECTEST alarms, DCTSTCYC (load verbs) alarms and recycles.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when the noun is not decimal only
 **********************************************************************/
bool pin_dectest(void)
{
    const pin_state_t *s = pin_st(); /* pinball erasables */

    return (!s->noun.dec_only);
}

/**********************************************************************
 * AGC: NOUNTEST (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 332)
 *
 * NOUNTEST: the noun can be loaded (no-load bit, bit 5 of the comp code).
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when the noun may be loaded
 **********************************************************************/
bool pin_nountest(void)
{
    const pin_state_t *s = pin_st(); /* pinball erasables */

    return (!s->noun.no_load);
}

/* ---- octal display verbs V01-V05 (pp. 330-332) ------------------------ */

/**********************************************************************
 * AGC: DSPA, DSPB, DSPC (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 331)
 *
 * DSPA, DSPB, DSPC, DSPAB, DSPABC with DSPCOM1/2/3. V01-V03 show one
 * component (1, 2 or 3) in R1; V04 components 1, 2 in R1, R2; V05 all
 * three. The tests run in the AGC's order before anything is shown.
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   verb      uint8_t  I    display verb 01..05
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_dsp_octal(uint8_t verb)
{
    static const int16_t rnd1[3] = {PIN_R1D1, PIN_R2D1, PIN_R3D1};

    int first = 0; /* comp shown in R1 */

    if (verb == 2u || verb == 3u)
    {
        first = (int) verb - 1; /* V02: comp 2, V03: comp 3 */
    }

    int count = (verb <= 3u) ? 1 : (int) verb - 2; /* registers used  */
    int need = first + count - 1;                 /* highest comp    */

    if (!pin_comptest(need) || !pin_dectest())
    {
        return (pin_dspalarm());
    }

    agc_status_t st = AGC_STATUS_OK; /* status so far */
    uint16_t buf[3] = {0, 0, 0};

    for (int i = 0; i < count && st == AGC_STATUS_OK; i++)
    {
        st = pin_comp_octal(first + i, &buf[i]); /* TSTFORDP, CHANDSP */
    }

    for (int i = count - 1; i >= 0 && st == AGC_STATUS_OK; i--)
    { /* DSPCOM3 */
        pin_set_dspcount(rnd1[i]);
        st = pin_dspoctwo(buf[i]);
    }

    return ((st == AGC_STATUS_OK) ? pin_entexit() : st); /* if OK */
} /* pin_dsp_octal */

/* ---- noun test and machine address (pp. 324-325) --------------------- */

/**********************************************************************
 * AGC: USEADD (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 324)
 *
 * USEADD: ECADR into NOUNCADR, reread the noun table, go to VERBFAN.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   ecadr     uint16_t  I    machine address (ECADR) supplied by the
 *                            caller
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_usead(uint16_t ecadr)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(ecadr <= PIN_WORD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t keep = ecadr;           /* ECADR to keep in NOUNCADR */
    agc_status_t st = pin_noun_load(); /* status so far */

    s->nouncadr = keep; /* SETNCADR               */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return (st);
    }

    return (pin_verbfan());
}

/**********************************************************************
 * AGC: INTMCTBS (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 325)
 *
 * INTMCTBS: internal machine address to be specified, left in MPAC+2 by
 * the NVSUB caller (or MONSAVE1 by MONDO). INTMCTBS +2 is also the noun
 * 15 (increment) entry. Don't display the CADR if verb = 05.
 *
 * ARGUMENTS:
 *   Argument    Type  I/O  Description (scale)
 *   ----------  ----  ---  -------------------
 *   from_mpac2  bool  I    true when the ECADR is taken from MPAC+2
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_intmctbs(bool from_mpac2)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    if (from_mpac2)
    {
        s->nouncadr = (uint16_t) (s->mpac2 & PIN_WORD); /* SETNCADR    */
    }

    if (s->verbreg != 5u)
    {
        pin_set_dspcount(PIN_R3D1);                  /* VB not = 05: display CADR */
        agc_status_t st = pin_dspoctwo(s->nouncadr); /* status so far */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }

    return (pin_verbfan());
}

/**********************************************************************
 * AGC: REQADD (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 324)
 *
 * REQADD: machine address (or channel) to be specified. External (keyed)
 * requests ask for it in R3; internal ones take it from MPAC+2.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_reqadd(void)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    s->clpass = PIN_CLPASS_NOBACK; /* set CLPASS for pass0 only */

    if (s->entret != PIN_ENT_ENDOFJOB)
    {
        return (pin_intmctbs(true));
    }

    return (pin_reqdat(PIN_INREL_Z, PIN_REQ_MCTBS)); /* REQDATZ   */
}

/**********************************************************************
 * AGC: REQADD, ENTER (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 323-324)
 *
 * REQADD continued after the operator's ENTER: alarm and recycle if
 * decimal was used for the MCTBS. An external MCTBS display will leave
 * flash on if ENDIDLE is not +0.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_reqadd_resume(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (s->decbrnch != 0u)
    {
        return (pin_almcycle());
    }

    pin_block_dspcount(PIN_VD1); /* block num char in      */

    if (pin_cadrstor_full())
    {
        pin_flashon();
    }

    if (!AGC_ASSERT(s->dspcount_neg))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_usead(s->xyz_oct[2])); /* XCH ZREG               */
}

/**********************************************************************
 * AGC: TESTNN (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 324)
 *
 * TESTNN: noun test for verbs below LOWVERB.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND; else the status of the routine it calls
 **********************************************************************/
agc_status_t pin_testnn(void)
{
    pin_state_t *s = pin_st();        /* pinball erasables */
    agc_status_t st = pin_noun_load(); /* status so far */

    if (st != AGC_STATUS_OK && st != AGC_STATUS_NOT_FOUND)
    {
        return (st);
    }

    switch (s->nntype)
    {
        case PIN_NNTYPE_NORMAL:
            return (pin_verbfan());
        case PIN_NNTYPE_MCTBS:
        case PIN_NNTYPE_CHAN:
            return (pin_reqadd());                                    /* specify machine CADR if - */
        case PIN_NNTYPE_AUGMENT:
            s->nouncadr = (uint16_t) ((s->nouncadr + 1u) & PIN_WORD); /* INCR NOUNCADR */

            return (pin_intmctbs(false));
        case PIN_NNTYPE_SPARE:
        default:

            return (pin_dspalarm()); /* not in use if +0       */
    }
}

/**********************************************************************
 * AGC: ENTPAS0 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 324)
 *
 * ENTPAS0: noun verb sub enters here. Block further numerical characters
 * so that stray chars do not get into verb or noun lights. TESTVB: if
 * the verb is G/E LOWVERB, skip the noun test.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_entpas0(void)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    s->decbrnch = 0;
    pin_block_dspcount(PIN_VD1);

    s->verbsave = s->verbreg; /* save verb for possible recycle */

    if (s->verbreg >= PIN_LOWVERB)
    {
        return (pin_verbfan());
    }

    return (pin_testnn());
}

/* ---- VERBFAN / VBFANDIR (pp. 325-326) --------------------------------- */

/**********************************************************************
 * AGC: VBFANDIR (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 325)
 *
 * VBFANDIR: INDEX VERBREG, CAF VERBTAB.
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   verb      uint8_t  I    verb 00..39
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_vbfandir(uint8_t verb)
{
    switch (verb)
    {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
            return (pin_dsp_octal(verb)); /* DSPA..DSPABC           */
        case 6:
            return (pin_decdsp());        /* decimal display        */
        case 7:
            return (pin_dspdpdec());      /* DP decimal display     */
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
            return (pin_monitor());
        default:
            break;
    }

    if (verb >= 21u)
    {
        /* VERBTAB VB21-VB39 part of the fan. */

        switch (verb)
        {
            case 21:
            case 22:
            case 23:
            case 24:
            case 25:
                return (pin_load_verb(verb));                 /* ALOAD..ABCLOAD         */
            case 27:
                return (pin_dspfmem());                       /* fixed memory display   */
            case 30:
                return (pin_vbrqexec());                      /* request executive      */
            case 31:
                return (pin_vbrqwait());                      /* request waitlist       */
            case 32:
                return (pin_vbproc_common(PIN_LS_DATAIN));    /* VBRESEQ        */
            case 33:
                return (pin_vbproc_common(PIN_LS_PROCEED));   /* VBPROC         */
            case 34:
                return (pin_vbproc_common(PIN_LS_TERMINATE)); /* VBTERM */
            case 35:
                return (pin_vbtstlts());                      /* test lights            */
            case 36:
                return (agc_fresh_start_slap1());             /* SLAP1 fresh start      */
            case 37:
                return (pin_reqmm());                         /* MMCHANG: TC REQMM      */
            default:
                return (pin_dspalarm());                      /* spare                  */
        }
    }

    return (pin_dspalarm()); /* VB00 illegal, spares   */
} /* pin_vbfandir */

/**********************************************************************
 * AGC: VERBFAN (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 325)
 *
 * VERBFAN: verbs 40 and up are extended verbs: release the display
 * system and go to GOEXTVB.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_verbfan(void)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    if (s->verbreg >= PIN_LST2CON)
    {
        agc_status_t st = agc_dsky_releasedsp(); /* release display syst   */

        return ((st == AGC_STATUS_OK) ? agc_extverb((int) s->verbreg, (int) s->nounreg) : st); /* if OK */
    }

    return (pin_vbfandir(s->verbreg));
}
