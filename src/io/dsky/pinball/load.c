/**********************************************************************
 * FILE NAME: load.c
 *
 * PURPOSE:
 *   PINBALL load verbs V21-V25 and the data request / store sequence.
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pp. 328, 343-348
 *     (REQDATX, REQDATY, REQDATZ, REQCOM, ABCLOAD, PUTXYZ with the noun 07
 *     flag-bit set/reset, ABLOAD, PUTXY, ALOAD, BLOAD, CLOAD, LOADLV,
 *     ALLDC/OC, PUTCOM, PUTNORM, PUTDPCOM, PUTCOM2, CHANLOAD)
 *   Luminary099 pp. 410, 425-430 (identical)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale     Description
 *   ------------  --------  ---  --------  ---------------------------
 *   ECADR words   uint16_t  I/O  unscaled  noun 07 flag bits, nouns 01-03
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  -------------------------------------------
 *   agc_erase_read_ecadr   noun 07 bit set/reset (state-flags)
 *   agc_erase_write_ecadr  noun 07 bit set/reset (state-flags)
 *   pin_comp_write         store a component (scale.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Mixed octal/decimal loads alarm and recycle (ALLDC/OC); scale
 *   overflow alarms and recycles (sfin.c). BAD_ARG on a failed
 *   precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the CHARIN job. Both vehicles.
 *
 * ALGORITHM:
 *   The AGC routine waits for each data word by leaving its return
 *   address, negated, in REQRET and ending the job (REQDATX..Z then TC
 *   ENTEXIT); the operator's ENTER does TC REQRET (ACCEPTWD). Here REQRET
 *   holds a pin_reqret_t naming the point after the REQDAT call and
 *   pin_load_resume() continues from it.
 **********************************************************************/
#include <stddef.h>

#include "internal.h"

#define PIN_VBSP1LD       21 /* VB21 = ALOAD                  */
#define PIN_VBSP2LD       22 /* VB22 = BLOAD                  */
#define PIN_VBSP3LD       23 /* VB23 = CLOAD                  */
#define PIN_SUPERBNK_CHAN 7u /* don't load chan 7 (superbank) */

/* ---- REQDATX / REQDATY / REQDATZ (p. 328) ----------------------------- */

/**********************************************************************
 * AGC: REQCOM (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 328)
 *
 * REQCOM: DSPCOUNT to the register's first digit, REQRET = -(return),
 * blank the register, flash on, and exit through ENTEXIT to wait.
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------
 *   inrel     int           I    register requested (INREL 2..4)
 *   step      pin_reqret_t  I    REQRET resume point of the load verb
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_reqdat(int inrel, pin_reqret_t step)
{
    static const int16_t rnd1[3] = {PIN_R1D1, PIN_R2D1, PIN_R3D1};
    pin_state_t         *s       = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(inrel >= PIN_INREL_X && inrel <= PIN_INREL_Z) ||
        !AGC_ASSERT(step > PIN_REQ_PASS0 && step < PIN_REQ_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_set_dspcount(rnd1[inrel - PIN_INREL_X]);
    s->reqret     = step;
    s->reqret_neg = true;
    agc_status_t st = pin_5blank(); /* status so far */

    pin_flashon();

    return ((st == AGC_STATUS_OK) ? pin_entexit() : st); /* ENDRQDAT    */
}

/**********************************************************************
 * AGC: CLPASHI (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 321)
 *
 * CLPASHI's "+3 to - number, backs data requests": the request for the
 * previous register of the same load verb.
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------
 *   step      pin_reqret_t  I    REQRET resume point of the load verb
 *
 * RETURNS: pin_reqret_t -- the resume point of the previous request
 **********************************************************************/
pin_reqret_t pin_reqret_backup(pin_reqret_t step)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (step); /* no backup */
    }

    switch (step)
    {
        case PIN_REQ_ABC_Y:
            return (PIN_REQ_ABC_X);
        case PIN_REQ_ABC_Z:
            return (PIN_REQ_ABC_Y);
        case PIN_REQ_AB_Y:
            return (PIN_REQ_AB_X);
        case PIN_REQ_PASS0:
        case PIN_REQ_ABC_X:
        case PIN_REQ_AB_X:
        case PIN_REQ_A_X:
        case PIN_REQ_B_Y:
        case PIN_REQ_C_Z:
        case PIN_REQ_MCTBS:
        case PIN_REQ_MM:
        case PIN_REQ_COUNT:
        default:
            return (step); /* unreachable: CLPASS keeps these out */
    }
}

/* ---- LOADLV (p. 346) -------------------------------------------------- */

/**********************************************************************
 * AGC: LOADLV (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 345)
 *
 * LOADLV: after a completed load, release the display for the priority
 * display problem, block numerical characters and clears, and go to
 * RECALTST to see if there is recall from ENDIDLE.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t agc_dsky_loadlv(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    s->decbrnch = 0;
    s->loadstat = PIN_LS_DATAIN; /* CS ZERO TS LOADSTAT      */
    agc_status_t st = agc_dsky_releasedsp(); /* status so far */

    pin_block_dspcount(PIN_VD1);

    if (!AGC_ASSERT(s->dspcount_neg))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_OK) ? pin_recaltst() : st); /* if OK */
}

/* ---- PUTCOM (pp. 347-348) --------------------------------------------- */

/**********************************************************************
 * AGC: PUTCOM (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 347)
 *
 * PUTCOM: store component k from XREG.. (octal) or XREG,XREGLP.. (decimal
 * via PUTDECSF). *finished when the load ended inside (alarm, HMSIN or
 * CHANLOAD going on to LOADLV).
 *
 * ARGUMENTS:
 *   Argument  Type    I/O  Description (scale)
 *   --------  ------  ---  -------------------
 *   k         int     I    noun component 0..2 (R1..R3)
 *   finished  bool *  O    true when the load ended inside (alarm, HMSIN,
 *                          CHANLOAD)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_putcom(int k, bool *finished)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t     *s = pin_st(); /* pinball erasables */
    agc_scale_info_t info; /* scale of the component */
    agc_status_t     st = pin_comp_scale(k, &info); /* status so far */

    if (!AGC_ASSERT(finished != NULL) || st != AGC_STATUS_OK)
    {
        return ((st != AGC_STATUS_OK) ? /* or BAD_ARG */
                st : AGC_STATUS_BAD_ARG);
    }

    *finished = false;

    if (s->decbrnch == 0u)
    {
        /* PUTCOM, octal: alarm and recycle if decimal only; for a DP scale
         * (DPTEST) +0 into the major part and the octal word into the minor
         * part; CHANLOAD for noun 10. */
        if (!AGC_ASSERT(pin_invariants()))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        pin_state_t *s_octal = pin_st(); /* pinball erasables */

        if (!pin_dectest())
        { /* DCTSTCYC  */
            *finished = true;

            return (pin_almcycle());
        }

        agc_status_t st_octal = AGC_STATUS_OK; /* status of the octal store */
        uint16_t     word     = s_octal->xyz_oct[k]; /* octal word */

        if (s_octal->nntype == PIN_NNTYPE_CHAN)
        { /* CHANLOAD  */
            *finished = true;

            if (s_octal->nouncadr != PIN_SUPERBNK_CHAN)
            {
                st_octal = pin_comp_write(k, (agc_dp_t) word);
            }

            return ((st_octal == AGC_STATUS_OK) ? /* if OK */
                    agc_dsky_loadlv() : st_octal);
        }

        agc_dp_t v_octal; /* octal word, unscaled */

        if (info.is_raw)
        {
            v_octal = (agc_dp_t) word;
        }
        else if (info.is_dp && pin_sf_is_dp(info.routine))
        {
            v_octal = agc_dp_pack(0, pin_sp_from_ones(word)); /* PUTDPCOM  */
        }
        else if (info.is_dp)
        {
            agc_dp_t old = 0; /* major part, unscaled */

            st_octal = pin_comp_read(k, &old);
            v_octal  = agc_dp_pack(pin_sp_from_ones(word), agc_dp_low(old));
        }
        else
        {
            v_octal = agc_dp_from_sp(pin_sp_from_ones(word));
        }

        if (!AGC_ASSERT(word <= PIN_WORD))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        return ((st_octal == AGC_STATUS_OK) ? /* store if octal ok */
                pin_comp_write(k, v_octal) : st_octal);
    } /* end if (s->decbrnch == 0u) */

    if (info.routine == (uint8_t) PIN_SF_HMS)
    {
        *finished = true;

        return (pin_hmsin()); /* stores all three, LOADLV */
    }

    agc_dp_t v = 0; /* value, scale of the comp */

    st = pin_putdecsf(k, s->xyz_frac[k], s->xyz_negzero[k], &v, finished);

    if (*finished)
    {
        return (st);
    }

    return (pin_comp_write(k, v));
} /* pin_putcom */

/**********************************************************************
 * AGC: ALLDC/OC (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 345)
 *
 * ALLDC/OC: tests that the data words loaded are either all decimal or
 * all octal; alarms (and recycles) if not.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   ncomp     int   I    number of components loaded 2..3
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_alldc_oc(int ncomp)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const pin_state_t *s = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(ncomp == 2 || ncomp == 3))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t dec = (uint16_t) ((s->decbrnch >> 2) & 7u); /* dec comp bits */
    uint16_t all = (ncomp == 3) ? 7u : 6u; /* DECBRNCH bits of all comps */

    if (dec == 0u || dec == all)
    {
        return (AGC_STATUS_OK);
    }

    return (pin_almcycle());
}

/**********************************************************************
 * AGC: PUTXYZ, PUTXY (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 343-344)
 *
 * PUTXYZ (ncomp 3) / PUTXY (ncomp 2) / single-component store.
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   first     int   I    first component 0..2
 *   ncomp     int   I    number of components to store 1..3
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_putn(int first, int ncomp)
{
    if (!AGC_ASSERT(first >= 0 && first < 3) ||
        !AGC_ASSERT(ncomp >= 1 && first + ncomp <= 3))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st; /* status so far */

    st = (ncomp > 1) ? pin_alldc_oc(ncomp) : AGC_STATUS_OK; /* ALLDC/OC */
    bool finished = false; /* load ended inside PUTCOM */

    for (int k = first; k < first + ncomp && st == AGC_STATUS_OK && !finished;
         k++)
    {
        st = pin_putcom(k, &finished);
    }

    if (st != AGC_STATUS_OK || finished)
    {
        return (st);
    }

    /* If noun 7 has just been loaded, set flag bits as specified.        */
    if (ncomp == 3 && pin_st()->nounreg == 7u)
    {
        /* PUTXYZ noun 07 part: XREG = ECADR of the flag word, YREG = bits to be
         * processed, ZREG zero to reset bits, non-zero to set bits. */
        if (!AGC_ASSERT(pin_invariants()))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        const pin_state_t *s     = pin_st(); /* pinball erasables */
        uint16_t           ecadr = s->xyz_oct[0]; /* flag word ECADR (XREG) */
        uint16_t           bits  = s->xyz_oct[1]; /* YREG bits */
        uint16_t           word  = 0; /* octal word */

        if (!AGC_ASSERT(s->nounreg == 7u))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_inhint();
        agc_status_t st_bits = agc_erase_read_ecadr(ecadr, &word); /* status */

        if (st_bits == AGC_STATUS_OK)
        {
            if (s->xyz_oct[2] != 0u && s->xyz_oct[2] != PIN_NEGZERO)
            {
                word = (uint16_t) (word | bits); /* set bits */
            }
            else
            {
                word = (uint16_t) (word & ~bits & PIN_WORD); /* BITSOFF */
            }

            st_bits = agc_erase_write_ecadr(ecadr, word);
        }

        agc_relint();
        st = st_bits;
    }

    return ((st == AGC_STATUS_OK) ? agc_dsky_loadlv() : st); /* if OK */
} /* pin_putn */

/* ---- the load verbs (pp. 343-346) ------------------------------------- */

/**********************************************************************
 * AGC: ALOAD, BLOAD, CLOAD
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 344-345)
 *
 * ALOAD (V21), BLOAD (V22), CLOAD (V23), ABLOAD (V24), ABCLOAD (V25):
 * component tests, then the first data request.
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   verb      uint8_t  I    load verb 21..25
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_load_verb(uint8_t verb)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if (!AGC_ASSERT(verb >= 21u && verb <= 25u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (verb == 21u)
    {
        return (pin_reqdat(PIN_INREL_X, PIN_REQ_A_X)); /* ALOAD     */
    }

    int comp = (verb == 22u || verb == 24u) ? 1 : 2; /* COMPTEST code */

    if (!pin_comptest(comp))
    {
        return (pin_dspalarm());
    }

    if (verb == 22u || verb == 23u)
    {
        s->clpass = PIN_CLPASS_NOBACK; /* set CLPASS for pass0 only */

        return ((verb == 22u) ? /* BLOAD / CLOAD */
                pin_reqdat(PIN_INREL_Y, PIN_REQ_B_Y) :
                pin_reqdat(PIN_INREL_Z, PIN_REQ_C_Z));
    }

    if (!pin_nountest())
    { /* test if noun can be loaded */
        return (pin_dspalarm());
    }

    agc_status_t st = pin_updatvb(PIN_VBSP1LD); /* status so far */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (pin_reqdat(PIN_INREL_X,
                       (verb == 24u) ? /* ABLOAD / ABCLOAD */
                       PIN_REQ_AB_X : PIN_REQ_ABC_X));
} /* pin_load_verb */

/**********************************************************************
 * AGC: ABLOAD, ABCLOAD
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 343-344)
 *
 * Show the next load verb and request the next register.
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------
 *   verb      uint8_t       I    load verb shown for the next request
 *   inrel     int           I    register requested next (INREL 2..4)
 *   step      pin_reqret_t  I    REQRET resume point of the load verb
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_next_request(uint8_t verb, int inrel, pin_reqret_t step)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(verb == PIN_VBSP2LD || verb == PIN_VBSP3LD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = pin_updatvb(verb); /* status so far */

    return ((st == AGC_STATUS_OK) ? pin_reqdat(inrel, step) : st); /* if OK */
}

/**********************************************************************
 * AGC: ACCEPTWD (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 323)
 *
 * TC REQRET from ACCEPTWD: continue the load verb after its data word.
 *
 * ARGUMENTS:
 *   Argument  Type          I/O  Description (scale)
 *   --------  ------------  ---  -------------------
 *   step      pin_reqret_t  I    REQRET resume point of the load verb
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_load_resume(pin_reqret_t step)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(step > PIN_REQ_PASS0 && step < PIN_REQ_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    switch (step)
    {
        case PIN_REQ_A_X:
            return (pin_putn(0, 1)); /* ALOAD  */
        case PIN_REQ_B_Y:
            return (pin_putn(1, 1)); /* BLOAD  */
        case PIN_REQ_C_Z:
            return (pin_putn(2, 1)); /* CLOAD  */
        case PIN_REQ_AB_X:
            return (pin_next_request(PIN_VBSP2LD, PIN_INREL_Y, PIN_REQ_AB_Y));
        case PIN_REQ_AB_Y:
            return (pin_putn(0, 2)); /* PUTXY  */
        case PIN_REQ_ABC_X:
            return (pin_next_request(PIN_VBSP2LD, PIN_INREL_Y, PIN_REQ_ABC_Y));
        case PIN_REQ_ABC_Y:
            return (pin_next_request(PIN_VBSP3LD, PIN_INREL_Z, PIN_REQ_ABC_Z));
        case PIN_REQ_ABC_Z:
            return (pin_putn(0, 3)); /* PUTXYZ */
        case PIN_REQ_MCTBS:
            return (pin_reqadd_resume());
        case PIN_REQ_MM:
            return (pin_mmchang_resume());
        case PIN_REQ_PASS0:
        case PIN_REQ_COUNT:
        default:
            return (AGC_STATUS_BAD_ARG);
    }
}
