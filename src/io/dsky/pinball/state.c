/**********************************************************************
 * FILE NAME: state.c
 *
 * PURPOSE:
 *   The PINBALL erasables (one static pin_state_t), DSPTAB maintenance
 *   (DSPIN, 11DSPIN, 2BLANK, 5BLANK), the lamp service routines, DSPOUT /
 *   DSPOUTSB for T4RUPT, the DSPTEM buffer views other lanes use, and the
 *   sim-side DSKY readout.
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pp. 318, 321-322,
 *     362-363, 379-380 (INRELTAB, GETINREL, 2BLANK, 5BLANK, DSPIN, DSPIN1,
 *     11DSPIN, FALTON, FLASHON, FLASHOFF, RELDSPON, LODSAMPT)
 *   Comanche055 T4RUPT_PROGRAM.agc pp. 133-135 (RELTAB, DSPOUT, DSPOUTSB)
 *   Luminary099 pp. 400-404, 444-445, 461-462 (identical)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale     Description
 *   ------------  --------  ---  --------  ---------------------------
 *   DSPTAB        uint16_t  I/O  bits      11 relay words (-: to send)
 *   DSPCNT        int       I/O  unscaled  DSPOUT scan position
 *   SAMPTIME      agc_dp_t  O    cs 2^28   LODSAMPT (noun 65)
 *   IMODES30      uint16_t  I    bits      LODSAMPT test
 *   channel 10    uint16_t  O    bits      relay word out (DSPOUT)
 *   channel 11    uint16_t  I/O  bits      DSKY lamps
 *
 * EXTERNAL REFERENCES:
 *   Name               Description
 *   -----------------  -----------------------------------------------
 *   agc_erase_dsptab   DSPTAB word access (core state)
 *   agc_chan_*         output channels 10 and 11
 *   agc_time_now_cs    TIME2,TIME1 for LODSAMPT
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A DSPTAB entry of +0 met by DSPIN is a CCSHOLE in the AGC: refused
 *   with AGC_STATUS_BAD_ARG (assertion). BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   DSPOUT runs in T4RUPT (interrupt); the rest in the CHARIN / NVSUB
 *   jobs. DSPOUTSB scans at most twice round DSPTAB. Both vehicles.
 *
 * NOTES:
 *   DSPTAB words are kept as raw 15-bit AGC words; a negative word (bit 15
 *   set) is still to be sent (NOUT counts them).
 **********************************************************************/
#include <stddef.h>

#include "internal.h"

/* All pinball erasables (PINBALL p. 310-312), module-private.             */
static pin_state_t g_pin; /* the pinball erasables */

/**********************************************************************
 * AGC: none (state)
 *
 * The pinball erasables (one static instance).
 *
 * ARGUMENTS: none
 *
 * RETURNS: pin_state_t * -- the pinball erasables
 **********************************************************************/
pin_state_t *pin_st(void)
{
    return (&g_pin);
}

/**********************************************************************
 * AGC: none (assertion helper)
 *
 * Invariants of the pinball erasables, asserted by the module's routines.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when every invariant holds
 **********************************************************************/
bool pin_invariants(void)
{
    const pin_state_t *s = &g_pin; /* pinball erasables */

    return (s->verbreg <= 99u && s->nounreg <= 99u && s->verbsave <= 99u &&
            s->reqret < PIN_REQ_COUNT && s->entret <= PIN_ENT_PASTEVB &&
            s->loadstat <= PIN_LS_DATAIN && s->nout >= 0 &&
            s->nout <= PIN_DSPTAB_LEN && (s->dspcount & 037) <= PIN_MD1 &&
            s->chr <= 9u && s->nntype <= PIN_NNTYPE_SPARE);
}

/* RELTAB (T4RUPT_PROGRAM p. 133), low 5 bits: relay code of digit 0-9.
 * Bits 12-15 of RELTAB give the relay word number = index + 1.           */
static const uint16_t k_relay_code[10] = {025, 003, 031, 033, 017,
                                          036, 034, 023, 035, 037};

/* INRELTAB (p. 318): DSPCOUNT (decimal 0..19) -> INREL. 15 is CCSHOLE.   */
static const int8_t k_inreltab[20] = {4, 4, 4, 4, 4, 3, 3, 3, 3, 3,
                                      2, 2, 2, 2, 2, -1, 1, 1, 0, 0};

/* DSMSK (p. 363): right char, left char, sign (bit 11), left char + sign. */
static const uint16_t k_dsmsk[4] = {037, 01740, 02000, 03740};

/* ---- relay codes ----------------------------------------------------- */

/**********************************************************************
 * AGC: RELTAB (T4RUPT_PROGRAM.agc, CM p. 133)
 *
 * RELTAB lookup for a decimal/octal digit.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   digit     int   I    digit 0..9, or AGC_DSKY_BLANK
 *
 * RETURNS: uint16_t -- 5-bit relay code (0 = blank)
 **********************************************************************/
uint16_t agc_dsky_relay_code(int digit)
{
    if (!AGC_ASSERT(digit >= 0) || !AGC_ASSERT(digit <= 9))
    {
        return (AGC_DSKY_RELAY_BLANK);
    }

    return (k_relay_code[digit]);
}

/**********************************************************************
 * AGC: none (sim readout, inverse of RELTAB)
 *
 * Inverse of RELTAB for the sim readout.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   code      uint16_t  I    5-bit relay code
 *
 * RETURNS: int -- digit 0..9, AGC_DSKY_BLANK, or -1 for no digit
 **********************************************************************/
int agc_dsky_relay_digit(uint16_t code)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_DSKY_BLANK);
    }

    if (!AGC_ASSERT(code <= PIN_LOW5))
    {
        return (AGC_DSKY_BLANK);
    }

    for (int d = 0; d < 10; d++)
    {
        if (k_relay_code[d] == code)
        {
            return (d);
        }
    }

    return (AGC_DSKY_BLANK);
}

/* ---- ones' complement word helpers ----------------------------------- */

/**********************************************************************
 * AGC: none (helper)
 *
 * Sign of a 15-bit ones' complement word (bit 15).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   w         uint16_t  I    15-bit ones' complement word
 *
 * RETURNS: bool -- true for a negative word (-0 included)
 **********************************************************************/
bool pin_ones_negative(uint16_t w)
{
    return ((w & PIN_BIT15) != 0u);
}

/**********************************************************************
 * AGC: none (helper: CCS; AD ONE)
 *
 * Magnitude of a 15-bit ones' complement word (CCS ...; AD ONE).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   w         uint16_t  I    15-bit ones' complement word
 *
 * RETURNS: uint16_t -- magnitude 0..037777
 **********************************************************************/
uint16_t pin_ones_mag(uint16_t w)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (0); /* no valid result */
    }

    uint16_t v = (uint16_t) (w & PIN_WORD); /* 15-bit word */

    return (pin_ones_negative(v) ? /* |word| */
            (uint16_t) ((~v) & PIN_POSMAX) : v);
}

/**********************************************************************
 * AGC: none (helper: CS)
 *
 * CS of a magnitude: the negative ones' complement word.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   mag       uint16_t  I    magnitude 0..037777
 *
 * RETURNS: uint16_t -- the ones' complement of mag
 **********************************************************************/
uint16_t pin_ones_neg(uint16_t mag)
{
    return ((uint16_t) ((~mag) & PIN_WORD));
}

/**********************************************************************
 * AGC: none (helper)
 *
 * DEVIATION: -0 (077777) maps to 0; agc_sp_t has no negative zero.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   w         uint16_t  I    15-bit ones' complement word
 *
 * RETURNS: agc_sp_t -- the signed value (-0 gives 0)
 **********************************************************************/
agc_sp_t pin_sp_from_ones(uint16_t w)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (0); /* no valid result */
    }

    uint16_t v = (uint16_t) (w & PIN_WORD); /* 15-bit word */

    if (pin_ones_negative(v))
    {
        return ((agc_sp_t) (-(int) pin_ones_mag(v)));
    }

    return ((agc_sp_t) v);
}

/**********************************************************************
 * AGC: none (helper)
 *
 * The 15-bit ones' complement word of an SP value.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   v         agc_sp_t  I    SP value
 *
 * RETURNS: uint16_t -- the 15-bit ones' complement word
 **********************************************************************/
uint16_t pin_ones_from_sp(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= AGC_SP_MIN) || !AGC_ASSERT(v <= AGC_SP_MAX))
    {
        return (0);
    }

    if (v < 0)
    {
        return (pin_ones_neg((uint16_t) (-v)));
    }

    return ((uint16_t) v);
}

/* ---- lamps (PINBALL p. 379-380) -------------------------------------- */

/**********************************************************************
 * AGC: FALTON (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 379)
 *
 * FALTON: turn on operator error light, bit 7 of channel 11.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_dsky_falton(void)
{
    agc_chan_or((int) AGC_CHAN_DSALMOUT, PIN_CH11_OPRERR); /* WOR cannot fail */
}

/**********************************************************************
 * AGC: FLASHON (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 380)
 *
 * FLASHON: turn on V/N flash, bit 6 of channel 11.
 *
 * ARGUMENTS: none
 **********************************************************************/
void pin_flashon(void)
{
    agc_chan_or((int) AGC_CHAN_DSALMOUT, PIN_CH11_FLASH);
}

/**********************************************************************
 * AGC: FLASHOFF (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 380)
 *
 * FLASHOFF: turn off V/N flash.
 *
 * ARGUMENTS: none
 **********************************************************************/
void pin_flashoff(void)
{
    agc_chan_and((int) AGC_CHAN_DSALMOUT,
                 (uint16_t) (PIN_WORD ^ PIN_CH11_FLASH));
}

/**********************************************************************
 * AGC: RELDSPON (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 379)
 *
 * RELDSPON: turn on key release light, bit 5 of channel 11.
 *
 * ARGUMENTS: none
 **********************************************************************/
void pin_reldspon(void)
{
    agc_chan_or((int) AGC_CHAN_DSALMOUT, PIN_CH11_KEYREL);
}

/**********************************************************************
 * LODSAMPT (p. 380): time is snatched in rupt for noun 65 (SAMPTIME).
 *
 * ARGUMENTS: none
 **********************************************************************/
void pin_lodsampt(void)
{
    agc_erase_set_samptime((agc_dp_t) agc_time_now_cs());
}

/* ---- DSPCOUNT -------------------------------------------------------- */

/**********************************************************************
 * AGC: none (TS DSPCOUNT)
 *
 * Set DSPCOUNT to a positive count (numerical characters allowed).
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   n         int16_t  I    DSPCOUNT value (0..025 octal)
 **********************************************************************/
void pin_set_dspcount(int16_t n)
{
    if (!AGC_ASSERT(n >= 0) || !AGC_ASSERT((n & 037) <= PIN_MD1))
    {
        return;
    }

    g_pin.dspcount     = n;
    g_pin.dspcount_neg = false;
}

/**********************************************************************
 * AGC: none (helper: CS n; TS DSPCOUNT)
 *
 * CS n; TS DSPCOUNT -- blocks numerical characters (DSPCOUNT negative).
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   n         int16_t  I    DSPCOUNT value (0..025 octal)
 **********************************************************************/
void pin_block_dspcount(int16_t n)
{
    if (!AGC_ASSERT(n >= 0) || !AGC_ASSERT(n <= PIN_MD1))
    {
        return;
    }

    g_pin.dspcount     = n;
    g_pin.dspcount_neg = true;
}

/**********************************************************************
 * AGC: INRELTAB (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 318)
 *
 * INRELTAB lookup; -1 for the CCSHOLE entry (DSPCOUNT 15 decimal).
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   dspcount  int16_t  I    DSPCOUNT 0..025 octal
 *
 * RETURNS: int -- INREL 0..4, or -1 for the CCSHOLE entry
 **********************************************************************/
int pin_inrel_of(int16_t dspcount)
{
    int idx = dspcount & 037; /* INRELTAB index */

    if (!AGC_ASSERT(idx < 20))
    {
        return (-1);
    }

    int inrel = k_inreltab[idx]; /* INREL of DSPCOUNT */

    if (!AGC_ASSERT(inrel >= 0))
    {
        return (-1); /* TC CCSHOLE                 */
    }

    return (inrel);
}

/**********************************************************************
 * GETINREL (p. 318): data register for the current DSPCOUNT.
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- INREL 0..4 for the current DSPCOUNT, or -1
 **********************************************************************/
int pin_getinrel(void)
{
    return (pin_inrel_of(g_pin.dspcount));
}

/* ---- DSPTAB ---------------------------------------------------------- */

/**********************************************************************
 * AGC: 2BLANK, DFRNT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 322, 363)
 *
 * Store -mag into DSPTAB+rel; INCR NOUT if the old contents were +NZ
 * (2BLANK / DFRNT: "IF OLD CONTENTS -, NOUT OK"; BZMF also skips +0).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   rel       int       I    DSPTAB index 0..10 (relay word)
 *   mag       uint16_t  I    new relay magnitude (stored negative: to be
 *                            sent)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t pin_dsptab_store(int rel, uint16_t mag)
{
    if (!AGC_ASSERT(rel >= 0) || !AGC_ASSERT(rel < PIN_DSPTAB_PANEL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    uint16_t old = agc_erase_dsptab(rel); /* previous DSPTAB word */

    agc_erase_set_dsptab(rel, pin_ones_neg((uint16_t) (mag & PIN_POSMAX)));

    if (!pin_ones_negative(old) && old != 0u)
    {
        g_pin.nout++;
    }

    agc_relint();

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * DSPIN / DSPIN1 (pp. 362-363). count = DSPCOUNT number (bit 14 set:
 * blank the sign with the left char), code = 5-bit relay code; or for
 * 11DSPIN count 2 and code 0 / BIT11 with rel given separately.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   rel       int       I    DSPTAB index 0..10 (relay word)
 *   sel       int       I    which relay field: 0/1 right or left char, 2
 *                            sign on, 3 sign off
 *   code      uint16_t  I    5-bit relay code (or BIT11 for a sign)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_dspin1(int rel, int sel, uint16_t code)
{
    if (!AGC_ASSERT(rel >= 0 && rel < PIN_DSPTAB_PANEL) ||
        !AGC_ASSERT(sel >= 0 && sel < 4))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t cur = agc_erase_dsptab(rel); /* current DSPTAB word */

    /* A DSPTAB entry of +0 is a CCSHOLE in the AGC.                     */
    if (!AGC_ASSERT(cur != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t mag = pin_ones_mag(cur); /* magnitude, unscaled */

    if ((uint16_t) (mag & k_dsmsk[sel]) == code)
    {
        return (AGC_STATUS_OK); /* DSLV: same                 */
    }

    uint16_t nmag; /* new relay magnitude */

    nmag = (uint16_t) ((mag & (uint16_t) ~k_dsmsk[sel]) + code);

    return (pin_dsptab_store(rel, nmag)); /* DFRNT                      */
}

/**********************************************************************
 * AGC: DSPIN (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 362)
 *
 * DSPIN: put the relay code of one character at DSPCOUNT count.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   count     int16_t   I    DSPCOUNT of the character (bit 14 set: also
 *                            blank the sign)
 *   code      uint16_t  I    5-bit relay code of the character
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_dspin(int16_t count, uint16_t code)
{
    if (!AGC_ASSERT(count >= 0) || !AGC_ASSERT(code <= PIN_LOW5))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int rel = (count & 037) >> 1; /* SR                         */

    if ((count & 1) == 0)
    {
        return (pin_dspin1(rel, 0, code)); /* right char                 */
    }

    int sel = ((uint16_t) count & PIN_BIT14) != 0u ? 3 : 1; /* left (+sign) */

    return (pin_dspin1(rel, sel, (uint16_t) (code << 5)));    /* SLEFT5       */
}

/**********************************************************************
 * 11DSPIN (p. 363): sign relay bit 11 of DSPTAB+rel on or off.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   rel       int   I    DSPTAB index 0..10 (relay word)
 *   on        bool  I    true to light the sign, false to turn it off
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_11dspin(int rel, bool on)
{
    if (!AGC_ASSERT(rel >= 0) || !AGC_ASSERT(rel < PIN_DSPTAB_PANEL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_dspin1(rel, 2, on ? /* sign bit on / off */
                       (uint16_t) PIN_BIT11 : 0u));
}

/**********************************************************************
 * 2BLANK (p. 322): blank the pair whose left char is DSPCOUNT.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_2blank(void)
{
    int rel = (g_pin.dspcount & 037) >> 1; /* DSPTAB index of the pair */

    if (!AGC_ASSERT(rel < PIN_DSPTAB_PANEL) || !AGC_ASSERT(g_pin.dspcount >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_dsptab_store(rel, (uint16_t) PIN_BIT12)); /* CS BLANKCON    */
}

/**********************************************************************
 * 5BLANK (pp. 321-322): blank R1, R2 or R3 for the current DSPCOUNT, zero
 * its X/Y/Z register and its decimal-component bit; leave DSPCOUNT at
 * the left-most digit of the register.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK; else the status of the routine it calls
 **********************************************************************/
agc_status_t pin_5blank(void)
{
    return (pin_5blank_rel(pin_getinrel()));
}

/**********************************************************************
 * AGC: 5BLANK, CLR5 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 321)
 *
 * 5BLANK +2 (CLR5 entry): the same with INREL already chosen.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   inrel     int   I    INREL: 0 verb, 1 noun, 2..4 R1..R3
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_5blank_rel(int inrel)
{
    static const int16_t sinblank[3] = {016, 005, 004}; /* SINBLANK     */
    static const int16_t doublk[3]   = {015, 011, 003}; /* DOUBLK       */
    static const int16_t rxd1[3]     = {PIN_R1D1, PIN_R2D1, PIN_R3D1};

    if (!AGC_ASSERT(inrel >= PIN_INREL_X) || !AGC_ASSERT(inrel <= PIN_INREL_Z))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int k = inrel - PIN_INREL_X; /* component 0..2 (R1..R3) */

    g_pin.xyz_oct[k]     = 0;
    g_pin.xyz_dec[k]     = 0;
    g_pin.xyz_frac[k]    = 0;
    g_pin.xyz_negzero[k] = false;
    /* zero pertinent dec comp bit (bit 5 R1, 4 R2, 3 R3) and low 2 bits */
    g_pin.decbrnch &= (uint16_t) ~((uint16_t) (020u >> k) | 3u);
    agc_status_t st = pin_dspin(sinblank[k], 0); /* status so far */

    pin_set_dspcount(doublk[k]);

    if (st == AGC_STATUS_OK)
    {
        st = pin_2blank();
    }

    pin_set_dspcount((int16_t) (doublk[k] - 2));

    if (st == AGC_STATUS_OK)
    {
        st = pin_2blank();
    }

    pin_set_dspcount(rxd1[k]);

    return (st);
}

/* ---- DSPOUT (T4RUPT p. 133) ------------------------------------------ */

/**********************************************************************
 * AGC: DSPOUTSB (T4RUPT_PROGRAM.agc, CM p. 135)
 *
 * DSPOUTSB: scan DSPTAB for a negative (pending) entry, starting below
 * DSPCNT and going round at most twice; send it to channel 10 and store
 * it back positively.
 *
 * ARGUMENTS:
 *   Argument    Type        I/O  Description (scale)
 *   ----------  ----------  ---  -------------------
 *   relay_word  uint16_t *  O    channel 10 word sent (0 when nothing
 *                                went out)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND
 **********************************************************************/
static agc_status_t pin_dspoutsb(uint16_t *relay_word)
{
    if (!AGC_ASSERT(relay_word != NULL) || !AGC_ASSERT(g_pin.nout > 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    g_pin.nout--; /* TS NOUT (CCS'd)        */

    for (int n = 0; n < 2 * PIN_DSPTAB_PANEL; n++)
    {
        g_pin.dspcnt = (int16_t) ((g_pin.dspcnt + PIN_DSPTAB_PANEL - 1) %
                                  PIN_DSPTAB_PANEL);
        uint16_t w = agc_erase_dsptab(g_pin.dspcnt); /* DSPTAB word */

        if (pin_ones_negative(w))
        {
            uint16_t mag = pin_ones_mag(w); /* magnitude, unscaled */

            agc_erase_set_dsptab(g_pin.dspcnt, mag); /* replace + */
            *relay_word = (uint16_t) (((uint16_t) (g_pin.dspcnt + 1) << 11) |
                                      (mag & PIN_LOW11));
            agc_chan_write((int) AGC_CHAN_OUT0, *relay_word);

            return (AGC_STATUS_OK);
        }
    }

    g_pin.nout = 0; /* 2nd pass: +0 into NOUT */

    return (AGC_STATUS_NOT_FOUND);
}

/**********************************************************************
 * AGC: DSPOUT (T4RUPT_PROGRAM.agc, CM p. 135)
 *
 * DSPOUT: display only when DSKYFLAG is on and NOUT is non-zero.
 *
 * ARGUMENTS:
 *   Argument    Type        I/O  Description (scale)
 *   ----------  ----------  ---  -------------------
 *   relay_word  uint16_t *  O    channel 10 word sent (0 when nothing
 *                                went out)
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition, NOT_FOUND;
 *          else the status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_dspout(uint16_t *relay_word)
{
    if (!AGC_ASSERT(relay_word != NULL) || !AGC_ASSERT(g_pin.nout >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *relay_word = 0;

    if (!agc_flag_test((agc_flag_id_t) AGC_FLAG_DSKYFLAG) || g_pin.nout == 0)
    {
        return (AGC_STATUS_NOT_FOUND); /* NODSPOUT               */
    }

    return (pin_dspoutsb(relay_word));
}

/**********************************************************************
 * AGC: none (sim readout)
 *
 * C(NOUT), the number of DSPTAB words still to send.
 *
 * ARGUMENTS: none
 *
 * RETURNS: int16_t -- NOUT, the number of DSPTAB words to send
 **********************************************************************/
int16_t agc_dsky_nout(void)
{
    return (g_pin.nout);
}

/* ---- sim readout ----------------------------------------------------- */

/**********************************************************************
 * AGC: none (sim readout)
 *
 * Character at DSPCOUNT position pos (0..025) decoded from DSPTAB.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   pos       int   I    DSPCOUNT position 0..025 octal
 *
 * RETURNS: int8_t -- digit 0..9 or AGC_DSKY_BLANK
 **********************************************************************/
static int8_t pin_digit_at(int pos)
{
    if (!AGC_ASSERT(pos >= 0) || !AGC_ASSERT(pos <= PIN_MD1))
    {
        return (AGC_DSKY_BLANK);
    }

    uint16_t mag  = pin_ones_mag(agc_erase_dsptab(pos >> 1)); /* magnitude */
    uint16_t code = ((pos & 1) != 0) ? /* left / right */
                    (uint16_t) ((mag >> 5) & PIN_LOW5) :
                    (uint16_t) (mag & PIN_LOW5); /* relay code of the char */

    return ((int8_t) agc_dsky_relay_digit(code));
}

/**********************************************************************
 * AGC: none (sim readout)
 *
 * Whether a sign relay bit is lit in DSPTAB+rel.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   rel       int   I    DSPTAB index of a sign word
 *
 * RETURNS: bool -- true when the sign relay bit (bit 11) is on
 **********************************************************************/
static bool pin_sign_lit(int rel)
{
    return ((pin_ones_mag(agc_erase_dsptab(rel)) & PIN_BIT11) != 0u);
}

/**********************************************************************
 * AGC: none (sim readout)
 *
 * The DSKY as the crew sees it, decoded from DSPTAB and channel 11.
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  -------------------
 *   out       agc_dsky_state_t *  O    lamps and digits as shown on the
 *                                      DSKY
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_dsky_get_state(agc_dsky_state_t *out)
{
    static const int base[3]      = {017, 012, 005}; /* RnD1 + 1          */
    static const int plus_rel[3]  = {6, 4, 1};       /* SGNTAB + 1        */
    static const int minus_rel[3] = {5, 3, 0};       /* SGNTAB            */

    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(g_pin.nout >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < 2; i++)
    {
        out->prog[i] = pin_digit_at(PIN_MD1 - i);
        out->verb[i] = pin_digit_at(PIN_VD1 - i);
        out->noun[i] = pin_digit_at(PIN_ND1 - i);
    }

    for (int r = 0; r < 3; r++)
    {
        for (int d = 0; d < AGC_DSKY_RDIGITS; d++)
        {
            out->r[r][d] = pin_digit_at(base[r] - 1 - d);
        }

        out->r_sign[r] = 0; /* no sign lit */

        if (pin_sign_lit(plus_rel[r]))
        {
            out->r_sign[r] = 1;
        }
        else if (pin_sign_lit(minus_rel[r]))
        {
            out->r_sign[r] = -1;
        }
    }

    out->lamps      = agc_chan_read((int) AGC_CHAN_DSALMOUT);
    out->flash      = (out->lamps & PIN_CH11_FLASH) != 0u;
    out->key_rel    = (out->lamps & PIN_CH11_KEYREL) != 0u;
    out->opr_err    = (out->lamps & PIN_CH11_OPRERR) != 0u;
    out->uplink_act = (out->lamps & PIN_CH11_UPACT) != 0u;
    out->verbreg    = g_pin.verbreg;
    out->nounreg    = g_pin.nounreg;

    return (AGC_STATUS_OK);
} /* agc_dsky_get_state */

/* ---- pinball erasables for other modules ----------------------------- */

/**********************************************************************
 * AGC: none (erasable read)
 *
 * DSPTEM1 +index (DSPTEM2 = +3) as stored.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   index     int   I    DSPTEM1 +index, 0..6
 *
 * RETURNS: uint16_t -- the word (0 on a bad index)
 **********************************************************************/
uint16_t agc_dsky_dsptem(int index)
{
    if (!AGC_ASSERT(index >= 0) || !AGC_ASSERT(index < 7))
    {
        return (0);
    }

    return (g_pin.dsptem[index]);
}

/**********************************************************************
 * AGC: none (erasable store)
 *
 * Store DSPTEM1 +index (e.g. NORMTEM1 of GOPERF1).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   index     int       I    DSPTEM1 +index, 0..6
 *   word      uint16_t  I    15-bit word to store
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_dsky_set_dsptem(int index, uint16_t word)
{
    if (!AGC_ASSERT(index >= 0 && index < 7) || !AGC_ASSERT(word <= PIN_WORD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    g_pin.dsptem[index] = word;

    return (AGC_STATUS_OK);
}

/* ---- program views of the DSPTEM buffer (erasable names of other lanes):
 * DSPTEM1 +i = index i (0..2); DSPTEMX = OPTIONX = DSPTEM2 +1 = index 4;
 * OPTIONX +i = index 4 + i (0..2). SP values travel as ones' complement
 * 15-bit words, DP DSPTEMX as the word pair (index 4, 5). ------------- */

/**********************************************************************
 * AGC: none (erasable store)
 *
 * Store DSPTEM1 +index as an SP value.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   index     int       I    DSPTEM1 +index, 0..2
 *   v         agc_sp_t  I    SP value (ones' complement word in the
 *                            buffer)
 **********************************************************************/
void agc_erase_set_dsptem1(int index, agc_sp_t v)
{
    if (!AGC_ASSERT(index >= 0 && index <= 2))
    {
        return;
    }

    agc_status_t st; /* status so far */

    st = agc_dsky_set_dsptem(index, pin_ones_from_sp(v));

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }
}

/**********************************************************************
 * AGC: none (erasable read)
 *
 * OPTIONX +index (= DSPTEM2 +1 +index).
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   index     int   I    OPTIONX +index, 0..2
 *
 * RETURNS: agc_sp_t -- OPTIONX +index as SP
 **********************************************************************/
agc_sp_t agc_erase_optionx(int index)
{
    if (!AGC_ASSERT(index >= 0 && index <= 2))
    {
        return (0);
    }

    return (pin_sp_from_ones(g_pin.dsptem[4 + index]));
}

/**********************************************************************
 * AGC: none (erasable store)
 *
 * Store OPTIONX +index.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   index     int       I    OPTIONX +index, 0..2
 *   v         agc_sp_t  I    SP value
 **********************************************************************/
void agc_erase_set_optionx(int index, agc_sp_t v)
{
    if (!AGC_ASSERT(index >= 0 && index <= 2))
    {
        return;
    }

    agc_status_t st; /* status so far */

    st = agc_dsky_set_dsptem(4 + index, pin_ones_from_sp(v));

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }
}

/**********************************************************************
 * AGC: none (erasable read)
 *
 * DSPTEMX,+1 (= DSPTEM2 +1, +2) as a DP.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- DSPTEMX,+1 as a DP
 **********************************************************************/
agc_dp_t agc_erase_dsptemx(void)
{
    agc_dp_t v = agc_dp_pack(pin_sp_from_ones(g_pin.dsptem[4]), /* scale */
                             pin_sp_from_ones(g_pin.dsptem[5])); /* of noun */

    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        return (0);
    }

    return (v);
}

/**********************************************************************
 * AGC: none (erasable store)
 *
 * Store DSPTEMX,+1 as a DP.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   v         agc_dp_t  I    DSPTEMX,+1 as a DP (centiseconds 2^28 for
 *                            HMS nouns)
 **********************************************************************/
void agc_erase_set_dsptemx(agc_dp_t v)
{
    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX))
    {
        return;
    }

    g_pin.dsptem[4] = pin_ones_from_sp(agc_dp_high(v));
    g_pin.dsptem[5] = pin_ones_from_sp(agc_dp_low(v));
}

/**********************************************************************
 * AGC: none (erasable read)
 *
 * XREG / YREG / ZREG as last loaded (octal word).
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   index     int   I    0 XREG, 1 YREG, 2 ZREG
 *
 * RETURNS: uint16_t -- the register's 15-bit word (0 on a bad index)
 **********************************************************************/
uint16_t agc_dsky_xyzreg(int index)
{
    if (!AGC_ASSERT(index >= 0) || !AGC_ASSERT(index < 3))
    {
        return (0);
    }

    return (g_pin.xyz_oct[index]);
}

/**********************************************************************
 * AGC: RECALTST (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 376)
 *
 * Verb and noun of the answer that woke the job in ENDIDLE.
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------
 *   verb      uint8_t *  O    verb of the response
 *   noun      uint8_t *  O    noun of the response
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_dsky_response(uint8_t *verb, uint8_t *noun)
{
    if (!AGC_ASSERT(verb != NULL) || !AGC_ASSERT(noun != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    *verb = g_pin.resp_verb;
    *noun = g_pin.resp_noun;

    return (AGC_STATUS_OK);
}

/* ---- STARTSUB part for pinball (FRESH_START_AND_RESTART p. 190-192) --- */

/**********************************************************************
 * AGC: none (fresh start)
 *
 * Zero the pinball erasables and blank the DSKY (fresh start).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition
 **********************************************************************/
agc_status_t agc_dsky_init(void)
{
    pin_state_t zero = {0};

    g_pin = zero;
    /* STARTSB2: turn off uplink acty, temp caution, KR, flash, op. error */
    agc_chan_and((int) AGC_CHAN_DSALMOUT, 077603u);

    for (int i = PIN_DSPTAB_PANEL - 1; i >= 0; i--)
    { /* DSPOFF         */
        agc_erase_set_dsptab(i, pin_ones_neg((uint16_t) PIN_BIT12));
    }

    g_pin.nout     = PIN_DSPTAB_PANEL; /* NOUTCON = ELEVEN       */
    g_pin.entret   = PIN_ENT_ENDOFJOB;
    g_pin.loadstat = PIN_LS_WAITING;
    pin_block_dspcount(PIN_VD1); /* CS VD1; TS DSPCOUNT    */
    agc_flagword_set(4, 0); /* kill interface displays */

    if (!AGC_ASSERT(g_pin.nout == 11))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}
