/**********************************************************************
 * FILE NAME: keys.c
 *
 * PURPOSE:
 *   PINBALL keyboard handling and the keyboard / uplink interrupts:
 *   CHARIN and the key routines (NUM, VERB, NOUN, signs, CLEAR, ENTER, KEY
 *   REL, ERROR RESET), PROCKEY, KEYRUPT1/2, UPRUPT, and the host key
 *   entry points used by the sim and the tests.
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pp. 315-324, 366-368,
 *     388-389 (CHARIN, NUM, 89TEST, DECTOBIN, ENDNMTST, DECEND, MORNUM,
 *     VERB, NOUN, NVCOM, POSGN, NEGSGN, SIGNTEST, CLEAR, CLPASHI, LEGALTST,
 *     ENTER, ENTPASHI, ACCEPTWD, VBRELDSP, UNSUSPEN, TSTLTS4, PROCKEY,
 *     ERROR)
 *   Comanche055 KEYRUPT_UPRUPT.agc pp. 1449-1451 (KEYRUPT1, KEYCOM,
 *     ACCEPTUP, UPRUPT, UPTEST, UPOK, TMFAIL2)
 *   Luminary099 pp. 398-406, 448-450, 470-471 and 1338-1340 (differs:
 *     ERROR resets RADMODES instead of OPTMODES)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale  Description
 *   ------------  --------  ---  -----  ------------------------------
 *   DSPTAB        uint16_t  I/O  bits   relay words (TSTAB, ERROR)
 *   IMODES30/33   uint16_t  I/O  bits   fail bits reset by ERROR
 *   OPTMODES      uint16_t  I/O  bits   CM: ERROR resets bit 7
 *   RADMODES      uint16_t  I/O  bits   LM: ERROR resets RFAILS, RCDUFBIT
 *   FAILREG,+1,+2 uint16_t  O    octal  zeroed by ERROR
 *   SFAIL         uint16_t  O    bits   zeroed by ERROR
 *   channels 11,13,15,16,INLINK         lamps, test alarm, key codes
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  --------------------------------------------
 *   agc_exec_novac        start CHARIN / PROCKEY jobs (CHRPRIO)
 *   agc_flag_set/clear    DSKYFLAG, UPLOCKFL
 *   agc_chan_*            lamps (channel 11), test alarm (channel 13)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   OPR ERR lamp (CHARALRM) on an illegal key; uplink lock-out (UPLOCKFL)
 *   on a bad uplink triple; AGC_STATUS_BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   KEYRUPT1/2 and UPRUPT run as interrupts and only NOVAC CHARIN; the
 *   key routines run in the CHARIN job. agc_dsky_key / agc_dsky_keys run
 *   CHARIN at once (sim convenience, see io-pinball.md). Both vehicles.
 *
 * NOTES:
 *   "THE LANGUAGE OF COMMUNICATION WITH THE PROGRAM IS A PAIR OF WORDS
 *    KNOWN AS VERB AND NOUN."
 *   ALMCYCLE's POSTJUMP ENTER is the bounded loop in pin_enter.
 **********************************************************************/
#include <stddef.h>

#include "internal.h"

#define PIN_ELRCODE  022u /* ELRCODE: error light reset key code */
#define PIN_KEYS_MAX 64   /* longest key string agc_dsky_keys takes */

/* DECON 2DEC 1 E-5 B14 = 2^14/10^5 = .16384 as a DP fraction (05174 13261). */
#define PIN_DECON       ((int64_t) 43980465) /* 2^14/10^5 at 2^-28 */
#define PIN_ERCON       0104u                /* ERCON: channel 11 bits 3, 7          */
#define PIN_GLNOATT     0050u                /* GL+NOATT: gimbal lock, no att lamps  */
#define PIN_RFAILS      0330u                /* RFAILS (LM): radar CDU and data fail */
#define PIN_RCDUFBIT    0100u                /* RCDUFBIT (LM) = BIT7                 */
#define PIN_PRIO16_BITS 016000u              /* PRIO16 OCT 16000                     */

/* CRITCON (p. 317): DSPCOUNT of the last character of VERB, NOUN, R1-R3. */
static const int16_t k_critcon[5] = {022, 020, 012, 005, 000};

/* ---- numerical characters (pp. 315-318) ------------------------------- */

/**********************************************************************
 * DECEND (p. 317): for R1-R3 convert the 5-digit integer N to N/10^5 by DMP
 * with DECON (2^14/10^5): (N x 2^-28)(DECON) = (N/10^5) x 2^-14 in MPAC,
 * +1, +2; MPAC+1, +2 go to XREG, XREGLP. A minus sign complements it
 * (DCS), so -00000 is ones' complement -0.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   inrel     int   I    INREL: 0 verb, 1 noun, 2..4 R1..R3
 **********************************************************************/
static void pin_decend(int inrel)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    int     k = inrel - PIN_INREL_X; /* component 0..2 (R1..R3) */
    int64_t n = s->xyz_dec[k];       /* decimal integer keyed so far */

    agc_dp_t frac  = (agc_dp_t) ((n * PIN_DECON) >> AGC_SP_BITS); /* 2^-28 */
    bool     minus = (s->decbrnch & 3u) == 2u;                    /* minus sign keyed */

    s->xyz_frac[k]    = minus ? -frac : frac; /* DCS     */
    s->xyz_negzero[k] = minus && frac == 0;
}

/**********************************************************************
 * ENDNMTST / ENDNUM / ENDALL / MORNUM (pp. 317-318).
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   inrel     int   I    INREL: 0 verb, 1 noun, 2..4 R1..R3
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
static agc_status_t pin_endnmtst(int inrel)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(inrel >= 0 && inrel <= PIN_INREL_Z) || !AGC_ASSERT(!s->dspcount_neg))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (s->dspcount != k_critcon[inrel])
    { /* MORNUM            */
        if (s->dspcount > 0)
        {
            s->dspcount--; /* DECREMENT DSPCOUNT */
        }
    }
    else
    {
        if ((s->decbrnch & 3u) != 0u && inrel >= PIN_INREL_X)
        {
            pin_decend(inrel); /* DECEND            */
        }

        /* ENDALL: block NUMIN by placing DSPCOUNT negatively.                 */
        pin_block_dspcount(s->dspcount);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * NUM (p. 316): numerical character in CHAR.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_num(void)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    if (s->dspcount_neg)
    {
        return (AGC_STATUS_OK); /* block data in if DSPCOUNT is - or -0 */
    }

    int inrel = pin_getinrel(); /* INREL of DSPCOUNT */

    if (!AGC_ASSERT(inrel >= 0) || !AGC_ASSERT(s->chr <= 9))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (s->clpass >= 0)
    {
        s->clpass = 0; /* if CLPASS is + or +0, make it +0 */
    }

    agc_status_t st; /* status so far */

    st = pin_dspin(s->dspcount, agc_dsky_relay_code(s->chr));

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* +0 octal, +1 + decimal, +2 - decimal.                                */
    /* Accumulate one character into VERBREG, NOUNREG or X/Y/ZREG: octal by
     * CYL x3 + CHAR, decimal by DECTOBIN (sum x 10 + CHAR).                 */
    bool decimal = (s->decbrnch & 3u) != 0u; /* a sign made the entry decimal */
    int  chr     = s->chr;                   /* the digit keyed (CHAR) */

    pin_state_t *s_accumulate = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(inrel >= 0 && inrel <= PIN_INREL_Z) || !AGC_ASSERT(chr >= 0 && chr <= 9))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (inrel == PIN_INREL_VERB || inrel == PIN_INREL_NOUN)
    {
        /* VERB/NOUN set DECBRNCH = 1, so these are always decimal.        */
        if (!AGC_ASSERT(decimal))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        uint8_t *reg = &s_accumulate->nounreg; /* NOUNREG ...      */

        if (inrel == PIN_INREL_VERB)
        {
            reg = &s_accumulate->verbreg; /* ... or VERBREG */
        }

        int v = (int) *reg * 10 + chr; /* new two-digit value */

        if (!AGC_ASSERT(v <= 99))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        *reg = (uint8_t) v;
    }
    else if (decimal)
    {
        int k = inrel - PIN_INREL_X; /* component 0..2 (R1..R3) */

        /* DECTOBIN */
        s_accumulate->xyz_dec[k] = s_accumulate->xyz_dec[k] * 10 + chr;
    }
    else
    {
        int k = inrel - PIN_INREL_X; /* component 0..2 (R1..R3) */

        s_accumulate->xyz_oct[k] = (uint16_t) (((uint32_t) s_accumulate->xyz_oct[k] << 3 | (uint32_t) chr) & PIN_WORD);
    }

    return (pin_endnmtst(inrel));
} /* pin_num */

/* ---- VERB, NOUN (p. 319) ---------------------------------------------- */

/**********************************************************************
 * AGC: NVCOM (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 319)
 *
 * NVCOM: blank the verb or noun lights and get ready for 2 decimal chars.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   verb      bool  I    true for the verb key, false for the noun key
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_nvcom(bool verb)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (verb)
    {
        s->verbreg = 0;
        pin_set_dspcount(PIN_VD1);
    }
    else
    {
        s->nounreg = 0;
        pin_set_dspcount(PIN_ND1); /* ND1, OCT 21 (DEC 17)              */
    }

    agc_status_t st = pin_2blank(); /* status so far */

    s->decbrnch   = 1u;            /* set for dec V/N code              */
    s->reqret     = PIN_REQ_PASS0; /* set for ENTPAS0                   */
    s->reqret_neg = false;

    /* If DSPALARM occurs before first ENTPAS0 or NVSUB, ENTRET must already
     * be set to TC ENDOFJOB.                                              */
    s->entret = PIN_ENT_ENDOFJOB;

    if (!AGC_ASSERT(s->dspcount == PIN_VD1 || s->dspcount == PIN_ND1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* ---- + and - (pp. 319-320) -------------------------------------------- */

/**********************************************************************
 * AGC: SIGNTEST (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 320)
 *
 * SIGNTEST: allows +,- only when DSPCOUNT = R1D1, R2D1 or R3D1, and only
 * the first of consecutive +/- characters.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when the sign key is accepted
 **********************************************************************/
static bool pin_signtest(void)
{
    const pin_state_t *s = pin_st(); /* pinball erasables */

    if ((s->decbrnch & 3u) != 0u)
    {
        return (false); /* sign for this word already in    */
    }

    if (s->dspcount_neg)
    {
        return (false);
    }

    return (s->dspcount == PIN_R1D1 || s->dspcount == PIN_R2D1 || s->dspcount == PIN_R3D1);
}

/**********************************************************************
 * AGC: POSGN, NEGSGN, BOTHSGN
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 319)
 *
 * POSGN / NEGSGN / BOTHSGN / FIXCLPAS.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   minus     bool  I    true for the - key, false for +
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_sign_key(bool minus)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!pin_signtest())
    {
        return (AGC_STATUS_OK); /* sign illegal: ENDOFJOB           */
    }

    int inrel = pin_getinrel(); /* INREL of DSPCOUNT */

    if (!AGC_ASSERT(inrel >= PIN_INREL_X) || !AGC_ASSERT(inrel <= PIN_INREL_Z))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = minus ? pin_minus_on() : pin_plus_on(); /* status */

    /* Set dec comp bit to 1: bit 5 for R1, bit 4 for R2, bit 3 for R3;
     * low bits 1 (+) or 2 (-).                                          */

    uint16_t sgn = minus ? 2u : 1u; /* DECBRNCH low bits: 2 -, 1 + */

    s->decbrnch = (uint16_t) (s->decbrnch + (PIN_BIT7 >> inrel) + sgn);

    if (s->clpass >= 0)
    {
        s->clpass = 0;
    }

    return (st);
}

/* ---- CLEAR (pp. 321-322) ---------------------------------------------- */

/**********************************************************************
 * AGC: LEGALTST (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 321)
 *
 * LEGALTST: INREL 0, 1 (verb, noun) cannot be cleared.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   inrel     int   I    INREL: 0 verb, 1 noun, 2..4 R1..R3
 *
 * RETURNS: bool -- true when the register may be cleared
 **********************************************************************/
static bool pin_legaltst(int inrel)
{
    return (inrel >= PIN_INREL_X && inrel <= PIN_INREL_Z);
}

/* ---- ENTER (pp. 323-324) ---------------------------------------------- */

/**********************************************************************
 * AGC: ENTER, ENTPASHI, ACCEPTWD
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 323)
 *
 * ENTER, ENTPASHI, ACCEPTWD: one pass.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_enter_once(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    s->clpass = 0;
    s->entret = PIN_ENT_ENDOFJOB;

    if (!s->reqret_neg)
    {
        return (pin_entpas0()); /* if +, pass 0           */
    }

    if (!AGC_ASSERT(s->reqret > PIN_REQ_PASS0 && s->reqret < PIN_REQ_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* If less than 2 chars in for the MM code, alarm and recycle (decided
     * at MMCHANG+1). If decimal, alarm if less than 5 chars in for data,
     * but leave REQRET - and flash on, so the operator can supply the
     * missing numerical characters and continue. Octal: any number of
     * chars OK.                                                          */
    if (s->reqret != PIN_REQ_MM && (s->decbrnch & 3u) != 0u && !s->dspcount_neg)
    {
        return (pin_dspalarm());
    }

    s->reqret_neg = false; /* ACCEPTWD: set REQRET + */
    pin_flashoff();

    return (pin_load_resume(s->reqret)); /* TC REQRET              */
}

/**********************************************************************
 * AGC: ENTER (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 323)
 *
 * ENTER. ALMCYCLE ends in POSTJUMP ENTER; that re-entry is this loop.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          PIN_STATUS_RECYCLE; else the status of the routine it calls
 **********************************************************************/
agc_status_t pin_enter(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = PIN_STATUS_RECYCLE; /* status so far */

    for (int pass = 0; pass < PIN_RECYCLE_MAX && st == PIN_STATUS_RECYCLE; pass++)
    {
        st = pin_enter_once();
    }

    if (!AGC_ASSERT(st != PIN_STATUS_RECYCLE))
    {
        return (AGC_STATUS_OK); /* bound hit: ENDOFJOB    */
    }

    return (st);
}

/* ---- KEY RELEASE (pp. 367-368) ---------------------------------------- */

/**********************************************************************
 * AGC: TSTLTS4 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 368)
 *
 * TSTLTS4: release display system and re-establish if CADRSTOR is full.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_tstlts4(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_dsky_releasedsp(); /* status so far */

    if (st != AGC_STATUS_OK || !pin_cadrstor_full())
    {
        return (st);
    }

    return (pin_pinbrnch());
}

/**********************************************************************
 * AGC: VBRELDSP (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 368)
 *
 * VBRELDSP. Key release: an external monitor suspended by the operator
 * (DSPLOCK was 1 and MONSAVE1 bit 14 set) is just unsuspended; otherwise
 * the display system is released.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_vbreldsp(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    agc_chan_and((int) AGC_CHAN_DSALMOUT, (uint16_t) (PIN_WORD ^ PIN_CH11_UPACT)); /* UPACT off */

    if (s->old_dsplock && (s->monsave1 & PIN_BIT14) != 0u)
    {
        s->dsplock = false; /* UNSUSPEN               */

        if (pin_cadrstor_full())
        {
            return (AGC_STATUS_OK);
        }

        return (pin_reldsp1());
    }

    return (pin_tstlts4());
}

/* ---- ERROR LIGHT RESET (pp. 388-389) ---------------------------------- */

/**********************************************************************
 * AGC: TSTAB (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 388)
 *
 * TSTAB: set BIT12 in every DSPTAB magnitude, keeping its sign (and so
 * its "to be sent" state). The AGC comment: "might want to reset CLPASS,
 * DECBRNCH, etc."
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_tstab(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_OK; /* status so far */

    for (int i = PIN_DSPTAB_PANEL - 1; i >= 0 && st == AGC_STATUS_OK; i--)
    {
        agc_inhint();
        uint16_t w   = agc_erase_dsptab(i);                      /* DSPTAB word */
        uint16_t mag = (uint16_t) (pin_ones_mag(w) | PIN_BIT12); /* magnitude */

        agc_erase_set_dsptab(i, pin_ones_negative(w) ? pin_ones_neg(mag) : mag); /* keep the sign */
        st = AGC_STATUS_OK;
        agc_relint();
    }

    return (st);
}

/**********************************************************************
 * AGC: ERROR (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 388)
 *
 * ERROR: restore the original C(DSPLOCK) -- thus error light reset leaves
 * DSPLOCK unchanged -- and reset the caution/alarm lamps.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_error_key(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    s->dsplock = s->old_dsplock;                            /* XCH 21/22REG           */
    agc_inhint();
    agc_chan_or((int) AGC_CHAN_DSALMOUT, PIN_CH11_CAUTRST); /* caution reset */

    /* Leave gimbal lock and no att intact, turning off auto, hold, free,
     * prog alarm and tracker.                                           */
    uint16_t c; /* new DSPTAB +11D lamp word */

    c = (uint16_t) ((agc_erase_dsptab(11) & PIN_GLNOATT) | PIN_BIT15);

    agc_erase_set_dsptab(11, c);
    agc_status_t st = AGC_STATUS_OK; /* status so far */

    /* Reset fail bits which generate prog alarm so that if the failure
     * still exists, the alarm will come back.                           */

    agc_erase_set_imodes33((uint16_t) (agc_erase_imodes33() | PIN_PRIO16_BITS));
    agc_erase_set_imodes30((uint16_t) (agc_erase_imodes30() | PIN_BIT10));

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        agc_erase_set_optmodes((uint16_t) (agc_erase_optmodes() | PIN_BIT7));
    }
    else
    {
        agc_erase_set_radmodes((uint16_t) ((agc_erase_radmodes() & ~PIN_RFAILS) | PIN_RCDUFBIT));
    }

    agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (PIN_WORD ^ PIN_CH13_TESTALM));
    agc_chan_and((int) AGC_CHAN_DSALMOUT, (uint16_t) (PIN_WORD ^ PIN_ERCON));
    agc_relint();
    st = pin_tstab();

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_failreg(i, 0); /* FAILREG, +1, +2        */
    }

    agc_erase_set_sfail(0);

    return (st);
} /* pin_error_key */

/* ---- CHARIN (p. 315) -------------------------------------------------- */

/**********************************************************************
 * AGC: CLEAR, CLPASHI, LEGALTST
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 321)
 *
 * CLEAR. Pass 0 blanks the register being loaded; a higher pass backs the
 * load up one register (CLPASHI: REQRET +3, verb - 1). DSPCOUNT is not
 * changed here because LEGALTST may still fail.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK (also for an illegal clear), BAD_ARG on a
 *          failed precondition; else the status of the routine it calls
 **********************************************************************/
static agc_status_t pin_clear_key(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s_key = pin_st();                      /* pinball erasables */
    int          inrel = pin_inrel_of(s_key->dspcount); /* INREL of DSPCOUNT */

    if (!AGC_ASSERT(inrel >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (s_key->clpass > 0)
    { /* CLPASHI      */
        inrel = (inrel > 0) ? inrel - 1 : 0; /* previous register */

        if (!pin_legaltst(inrel))
        {
            return (AGC_STATUS_OK);
        }

        /* backs data requests */
        s_key->reqret = pin_reqret_backup(s_key->reqret);

        if (s_key->verbreg > 0)
        {
            s_key->verbreg--; /* DIM VERBREG  */
        }

        agc_status_t st = pin_updatvb(s_key->verbreg); /* status */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }
    else if (!pin_legaltst(inrel))
    {
        return (AGC_STATUS_OK); /* illegal      */
    }

    agc_status_t st = pin_5blank_rel(inrel); /* CLEAR1, CLR5 */

    s_key->clpass++; /* set for higher pass */

    if (!AGC_ASSERT(s_key->clpass != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
} /* pin_clear_key */

/**********************************************************************
 * AGC: CHARIN2 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 315)
 *
 * CHARIN2 fan on the 5-bit input code.
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   code      uint8_t  I    5-bit key code
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_charin2(uint8_t code)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    s->chr = (code <= 7u) ? code : 0u; /* CHAR: digit keys only */

    switch (code)
    {
        case 001:
        case 002:
        case 003:
        case 004:
        case 005:
        case 006:
        case 007:
            return (pin_num());
        case 010:
        case 011:
            s->chr = (uint8_t) (code - 010u + 8u);

            /**********************************************************************
             * 89TEST (p. 316): 8 or 9 is accepted only in decimal.
             *
             * ARGUMENTS: none
             *
             * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
             *          status of the routine it calls
             **********************************************************************/
            if (!AGC_ASSERT(pin_invariants()))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            pin_state_t *s_89test = pin_st(); /* pinball erasables */

            if (!AGC_ASSERT(s_89test->chr == 8 || s_89test->chr == 9))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            if (s_89test->dspcount_neg)
            {
                return (AGC_STATUS_OK); /* block data in if DSPCOUNT is - or -0 */
            }

            if ((s_89test->decbrnch & 3u) == 0u)
            {
                return (pin_charalrm()); /* if DECBRNCH is +0, reject 8 or 9 */
            }

            return (pin_num());
        case 020:
            s->chr = 0u; /* NUM -2: CHAR = 0       */

            return (pin_num());
        case 021:
            return (pin_nvcom(true));     /* VERB                   */
        case 022:
            return (pin_error_key());     /* ERROR LIGHT RESET      */
        case 031:
            return (pin_vbreldsp());      /* KEY RELEASE            */
        case 032:
            return (pin_sign_key(false)); /* +                      */
        case 033:
            return (pin_sign_key(true));  /* -                      */
        case 034:
            return (pin_enter());         /* ENTER                  */
        case 036:
            return (pin_clear_key());     /* CLEAR */
        case 037:
            return (pin_nvcom(false));    /* NOUN                   */
        default:
            return (pin_charalrm());
    } /* end switch (code) */
} /* pin_charin2 */

/**********************************************************************
 * AGC: CHARIN (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 315)
 *
 * CHARIN: block display system, but save old C(DSPLOCK) for error light
 * reset. All keys except ER turn on KR lite if CADRSTOR is full; this
 * reminds the operator to re-establish a flashing display which he has
 * obscured with displays of his own.
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   code      uint8_t  I    5-bit key code
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition,
 *          PIN_STATUS_RECYCLE; else the status of the routine it calls
 **********************************************************************/
agc_status_t pin_charin(uint8_t code)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(code <= PIN_LOW5))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    s->old_dsplock = s->dsplock; /* TS 21/22REG            */
    s->dsplock     = true;       /* make DSP syst busy     */

    if (pin_cadrstor_full() && code != PIN_ELRCODE)
    {
        pin_reldspon();
    }

    agc_status_t st = pin_charin2(code); /* status so far */

    if (!AGC_ASSERT(st != PIN_STATUS_RECYCLE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * AGC: CHARIN (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 315)
 *
 * CHARIN as an executive job: the key code arrives in MPAC.
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
agc_status_t agc_job_charin(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(ctx->arg >= 0 && ctx->arg <= (agc_sp_t) PIN_LOW5))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = pin_charin((uint8_t) ctx->arg); /* status so far */

    return ((st == AGC_STATUS_POODOO) ? st : AGC_STATUS_OK); /* ENDOFJOB    */
}

/**********************************************************************
 * PROCKEY (p. 367): the PRO key (sampled in T4RUPT).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_job_prockey(void)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    s->reqret     = PIN_REQ_PASS0; /* set REQRET for ENTER pass 0 */
    s->reqret_neg = false;
    pin_block_dspcount(PIN_VD1);   /* block num chars, signs, clear */

    if (!AGC_ASSERT(s->dspcount_neg))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_vbproc_common(PIN_LS_PROCEED));
}

/**********************************************************************
 * AGC: PROCKEY (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 367)
 *
 * T4RUPT found the PRO key pressed: start the PROCKEY job (prio 30).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK; else the NOVAC status (BAILOUT 1202)
 **********************************************************************/
agc_status_t agc_dsky_prockey_request(void)
{
    agc_status_t st = agc_exec_novac(PIN_CHRPRIO, AGC_JOB_PROCKEY); /* status */

    return (st);
}

/* ---- KEYRUPT / UPRUPT (KEYRUPT_UPRUPT.agc pp. 1449-1451) -------------- */

/**********************************************************************
 * AGC: KEYCOM, ACCEPTUP (KEYRUPT_UPRUPT.agc, CM p. 1449)
 *
 * KEYCOM / ACCEPTUP: set DSKYFLAG and leave the 5-bit key code in MPAC of
 * a new CHARIN job.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   code      uint16_t  I    5-bit key code
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_keycom(uint16_t code)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(code <= PIN_LOW5))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_flag_set((agc_flag_id_t) AGC_FLAG_DSKYFLAG);
    agc_status_t st = AGC_STATUS_OK; /* status so far */

    st = agc_exec_novac(PIN_CHRPRIO, AGC_JOB_CHARIN);

    if (st == AGC_STATUS_OK)
    {
        st = agc_exec_set_new_job_arg((agc_sp_t) code); /* INDEX LOCCTR TS MPAC */
    }

    return (st);
}

/**********************************************************************
 * AGC: KEYRUPT1 (KEYRUPT_UPRUPT.agc, CM p. 1449)
 *
 * KEYRUPT1: main DSKY, key code in channel 15.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_keyrupt1(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_lodsampt(); /* time snatched for N65  */
    uint16_t code;  /* 5-bit key code */

    code = (uint16_t) (agc_chan_read((int) AGC_CHAN_MNKEYIN) & PIN_LOW5);

    if (!AGC_ASSERT(code <= PIN_LOW5))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_keycom(code));
}

/**********************************************************************
 * AGC: KEYCOM (KEYRUPT_UPRUPT.agc, CM p. 1449)
 *
 * KEYRUPT2 (CSM: via MARKRUPT): navigation DSKY, key code in channel 16.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_keyrupt2(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t code; /* 5-bit key code */

    code = (uint16_t) (agc_chan_read((int) AGC_CHAN_NAVKEYIN) & PIN_LOW5);

    if (!AGC_ASSERT(code <= PIN_LOW5))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_keycom(code));
}

/**********************************************************************
 * AGC: UPRUPT (KEYRUPT_UPRUPT.agc, CM p. 1450)
 *
 * UPRUPT: INLINK carries the character three times, C, NOT-C, C (bits
 * 1-5, 6-10, 11-15). A bad word locks out further uplink (UPLOCKFL) until
 * 'error reset' is sent via uplink.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   inlink    uint16_t  I    INLINK word: C, NOT-C, C in bits 1-5, 6-10,
 *                            11-15
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_uprupt(uint16_t inlink)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(inlink <= PIN_WORD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_lodsampt();
    agc_chan_or((int) AGC_CHAN_DSALMOUT, PIN_CH11_UPACT);   /* UPACT on */
    uint16_t low  = (uint16_t) (inlink & PIN_LOW5);         /* C, bits 1-5 */
    uint16_t mid  = (uint16_t) ((inlink >> 5) & PIN_LOW5);  /* NOT-C */
    uint16_t high = (uint16_t) ((inlink >> 10) & PIN_LOW5); /* C, bits 11-15 */

    if (mid != (uint16_t) (~low & PIN_LOW5) || high != low)
    { /* UPTEST */
        agc_flag_set((agc_flag_id_t) AGC_FLAG_UPLOCKFL);

        return (AGC_STATUS_OK); /* TMFAIL2 */
    }

    /* UPOK: code is good. 'Error reset' clears UPLOCKFL; any other code is
     * accepted only if UPLOCKFL is clear.                               */
    if (low == PIN_ELRCODE)
    {
        agc_flag_clear((agc_flag_id_t) AGC_FLAG_UPLOCKFL); /* CLUPLOCK */
        agc_status_t st = AGC_STATUS_OK;                   /* status so far */

        return ((st == AGC_STATUS_OK) ? pin_keycom(low) : st); /* if OK */
    }

    if (agc_flag_test((agc_flag_id_t) AGC_FLAG_UPLOCKFL))
    {
        return (AGC_STATUS_OK); /* TC RESUME              */
    }

    return (pin_keycom(low)); /* ACCEPTUP               */
}

/* ---- sim conveniences ------------------------------------------------- */

/**********************************************************************
 * AGC: KEYRUPT1, CHARIN
 *      (KEYRUPT_UPRUPT.agc, CM p. 1449;
 *      PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 315)
 *
 * One key: KEYRUPT1 bookkeeping, then CHARIN at once (no executive).
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   code      uint8_t  I    5-bit key code
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_key(uint8_t code)
{
    if (!AGC_ASSERT(code <= PIN_LOW5) || !AGC_ASSERT(code != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_lodsampt();
    agc_flag_set((agc_flag_id_t) AGC_FLAG_DSKYFLAG);
    agc_status_t st = AGC_STATUS_OK; /* status so far */

    return ((st == AGC_STATUS_OK) ? pin_charin(code) : st); /* if OK */
}

/**********************************************************************
 * AGC: none (host key string)
 *
 * Key code for one character of a key string; 0 for none.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   c         char  I    key character ('V', 'N', '0'..'9', '+', '-',
 *                        'E', 'C', 'K', 'R', 'P')
 *
 * RETURNS: uint8_t -- key code, 0 for an unknown character
 **********************************************************************/
static uint8_t pin_key_of_char(char c)
{
    static const uint8_t digit[10] = {020, 001, 002, 003, 004, 005, 006, 007, 010, 011};

    if (c >= '0' && c <= '9')
    {
        return (digit[c - '0']);
    }

    switch (c)
    {
        case 'V':
            return ((uint8_t) AGC_DSKY_KEY_VERB);
        case 'N':
            return ((uint8_t) AGC_DSKY_KEY_NOUN);
        case 'E':
            return ((uint8_t) AGC_DSKY_KEY_ENTR);
        case 'C':
            return ((uint8_t) AGC_DSKY_KEY_CLR);
        case 'R':
            return ((uint8_t) AGC_DSKY_KEY_RSET);
        case 'K':
            return ((uint8_t) AGC_DSKY_KEY_KEYREL);
        case '+':
            return ((uint8_t) AGC_DSKY_KEY_PLUS);
        case '-':
            return ((uint8_t) AGC_DSKY_KEY_MINUS);
        default:
            return (0u);
    }
}

/**********************************************************************
 * AGC: none (host entry)
 *
 * Feed a string of keys, each as KEYRUPT1 + CHARIN, PRO as PROCKEY.
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------
 *   seq       const char *  I    NUL-terminated key string (characters as
 *                                for one key)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_keys(const char *seq)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = AGC_STATUS_OK; /* status so far */

    for (int i = 0; i < PIN_KEYS_MAX && seq[i] != '\0' && st == AGC_STATUS_OK; i++)
    {
        if (seq[i] == 'P')
        {
            /* PRO for the sim: PROCKEY run at once. */
            if (!AGC_ASSERT(pin_invariants()))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            st = agc_job_prockey();
            continue;
        }

        uint8_t code = pin_key_of_char(seq[i]); /* 5-bit key code */

        st = (code != 0u) ? agc_dsky_key(code) : AGC_STATUS_BAD_ARG; /* unknown char */
    }

    return (st);
}
