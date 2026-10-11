/**********************************************************************
 * FILE NAME: scale.c
 *
 * PURPOSE:
 *   Noun component access (data ids, machine addresses, channels) and
 *   the decimal display scale routines of PINBALL (V06, V07).
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pp. 327, 333-342
 *     (MIXNOUN, DPTEST, TSTFORDP, CHANDSP, READLO, DECDSP, DSPDCGET,
 *     DSPDCPUT, DSPDCEND, SFOUTABR, DEGOUTSF, OPDEGOUT, FIXRANGE, SETAUG,
 *     DEGCOM, ARTOUTSF, AROUT1SF, DP1OUTSF, DP2OUTSF, DP3OUTSF, DPOUT,
 *     2INTOUT, DPFRACOT, HMSOUT, M/SOUT, SEPSEC, SEPSECNR, SEPMIN,
 *     DSPDPDEC)
 *   Luminary099 pp. 409, 415-424 (differs: LRPOSOUT for routine 6,
 *     360-CDUO for routine 13)
 *
 * EXTERNAL VARIABLES:
 *   Name          Type      I/O  Scale     Description
 *   ------------  --------  ---  --------  ---------------------------
 *   erasables     agc_dp_t  I/O  per noun  through agc_noun_read/write
 *   ECADR words   uint16_t  I/O  unscaled  nouns 01-03, 15 (ecadr access)
 *   channels      uint16_t  I/O  bits      noun 10; channel 33 (LRPOSOUT)
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  -------------------------------------------
 *   agc_noun_read/write    the erasable behind a data id
 *   agc_scale_info         SF routine and constants of a scale
 *   agc_erase_read_ecadr   machine-address nouns (state-flags)
 *   agc_chan_read/write    channel nouns (CHANDSP, CHANLOAD)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Decimal display of an octal-only noun alarms (PREDSPAL); mixed nouns
 *   alarm on V07; BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the CHARIN / NVSUB / MONDO job. Both vehicles.
 *
 * NOTES:
 *   OPDEGOUT: a negative word plus 20BIAS is a ones' complement AD, so
 *   -20BIAS gives -0 (DEGOUTSF shows +89.997 deg), oracle finding F15.
 *
 * ALGORITHM:
 *   The AGC scales a value for display into a DP fraction F whose 5
 *   decimal digits are shown (DSPDECWD: F x 10^5, rounded by 5 E-6). The
 *   multiplications are the AGC's: SHORTMP (SP x DP -> TP) and DMP (DP x DP
 *   -> TP) on sign and magnitude, truncating, with the products formed here
 *   exactly in 64-bit integers (units: SP 2^-14, DP 2^-28, TP 2^-42). This
 *   is display conversion, not flight arithmetic. HMSOUT and M/SOUT derive
 *   the same digits by integer division of the centisecond count.
 **********************************************************************/
#include <stddef.h>

#include "internal.h"

#define PIN_TP_DP_SHIFT 14                                 /* TP -> DP (drop MPAC+2) */
#define PIN_DP_MASK     (((int64_t) 1 << AGC_DP_BITS) - 1) /* 28 bits */
#define PIN_TP_MASK     (((int64_t) 1 << AGC_TP_BITS) - 1) /* 42 bits */

/* DEGTAB: .18 (05605 03656) and .45 (16314 31463) as DP fractions.        */
#define PIN_DEG18    ((int64_t) (05605 * 040000 + 03656))   /* .18 at 2^-28 */
#define PIN_DEG45    ((int64_t) (016314 * 040000 + 031463)) /* .45 at 2^-28 */
#define PIN_20BIAS   016040                                 /* 20BIAS: 19.775 deg     */
#define PIN_CS_HOUR  360000                                 /* HRCON                  */
#define PIN_CS_MIN   6000                                   /* MINCON                 */
#define PIN_MS_LIMIT 359850                                 /* 59M58.5S (M/SCON1,2)   */
#define PIN_MS_FORCE 359950                                 /* 59M59.5S (M/SCON3)     */
#define PIN_RNDCON   50                                     /* RNDCON .5 sec          */

static const int16_t k_rnd1[3] = {PIN_R1D1, PIN_R2D1, PIN_R3D1};

/* ---- pinball's own noun erasables ------------------------------------- */

/**********************************************************************
 * AGC: none (data layer)
 *
 * Storage word index in pin_state_t.dsptem[] (or 10+k for X/Y/ZREG) and
 * kind (0 SP raw, 1 SP, 2 DP) of a pinball-owned data id; -1 if not ours.
 * DSPTEMX = OPTIONX = DSPTEM2 +1; NORMTEM1 = DSPTEM1.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   id        uint16_t  I    agc_noun_data_t id
 *   kind      int *     O    0 SP raw, 1 SP, 2 DP
 *
 * RETURNS: int -- word index, or -1 for an id pinball does not own
 **********************************************************************/
static int pin_own_id(uint16_t id, int *kind)
{
    switch (id)
    {
        case AGC_DATA_DSPTEM1:
            *kind = 0;

            return (0);
        case AGC_DATA_DSPTEM1_P1:
            *kind = 0;

            return (1);
        case AGC_DATA_DSPTEM1_P2:
            *kind = 0;

            return (2);
        case AGC_DATA_DSPTEM1_DP:
            *kind = 2;

            return (0);
        case AGC_DATA_DSPTEM2:
            *kind = 1;

            return (3);
        case AGC_DATA_DSPTEM2_P1:
            *kind = 1;

            return (4);
        case AGC_DATA_DSPTEM2_P2:
            *kind = 1;

            return (5);
        case AGC_DATA_DSPTEM2_P1_DP:
            *kind = 2;

            return (4);
        case AGC_DATA_DSPTEMX:
            *kind = 1;

            return (4);
        case AGC_DATA_DSPTEMX_P1:
            *kind = 1;

            return (5);
        case AGC_DATA_DSPTEMX_P2:
            *kind = 1;

            return (6);
        case AGC_DATA_DSPTEMX_DP:
            *kind = 2;

            return (4);
        case AGC_DATA_OPTIONX:
            *kind = 0;

            return (4);
        case AGC_DATA_OPTIONX_P1:
            *kind = 0;

            return (5);
        case AGC_DATA_XREG:
            *kind = 0;

            return (10);
        case AGC_DATA_YREG:
            *kind = 0;

            return (11);
        case AGC_DATA_ZREG:
            *kind = 0;

            return (12);
        default:
            *kind = -1;

            return (-1);
    } /* end switch (id) */
} /* pin_own_id */

/**********************************************************************
 * AGC: none (helper)
 *
 * The pinball-owned word behind index idx (DSPTEM1.. or X/Y/ZREG).
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   idx       int   I    index from pin_own_id (0..6 DSPTEM, 10..12
 *                        X/Y/ZREG)
 *
 * RETURNS: uint16_t * -- the word in the pinball state
 **********************************************************************/
static uint16_t *pin_own_word(int idx)
{
    pin_state_t *s = pin_st(); /* pinball erasables */

    return ((idx >= 10) ? &s->xyz_oct[idx - 10] : &s->dsptem[idx]); /* X/Y/ZREG or DSPTEM */
}

/**********************************************************************
 * AGC: none (data layer)
 *
 * Read a data id pinball owns (DSPTEM1.., DSPTEMX.., OPTIONX.., X/Y/ZREG);
 * NOT_FOUND for every other id.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   data_id   uint16_t    I    agc_noun_data_t id of the erasable
 *                              (nouns.h)
 *   value     agc_dp_t *  I/O  value read or written (nouns.h value
 *                              convention)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND
 **********************************************************************/
agc_status_t agc_dsky_noun_read(uint16_t data_id, agc_dp_t *value)
{
    int kind = -1;                         /* id kind: 0 SP raw, 1 SP, 2 DP */
    int idx  = pin_own_id(data_id, &kind); /* word index or -1 */

    if (idx < 0)
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    if (!AGC_ASSERT(idx <= 12 && (idx < 6 || idx >= 10 || kind == 1)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t w = *pin_own_word(idx); /* 15-bit word */

    if (kind == 0)
    {
        *value = (agc_dp_t) w; /* raw word         */
    }
    else if (kind == 1)
    {
        *value = agc_dp_from_sp(pin_sp_from_ones(w));
    }
    else
    {
        *value = agc_dp_pack(pin_sp_from_ones(w), pin_sp_from_ones(*pin_own_word(idx + 1)));
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (data layer)
 *
 * Store into a data id pinball owns; NOT_FOUND for every other id.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   data_id   uint16_t  I    agc_noun_data_t id of the erasable (nouns.h)
 *   value     agc_dp_t  I    value to store (nouns.h value convention)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND
 **********************************************************************/
agc_status_t agc_dsky_noun_write(uint16_t data_id, agc_dp_t value)
{
    int kind = -1;                         /* id kind: 0 SP raw, 1 SP, 2 DP */
    int idx  = pin_own_id(data_id, &kind); /* word index or -1 */

    if (idx < 0)
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    if (!AGC_ASSERT(kind == 0 ? (value >= 0 && value <= (agc_dp_t) PIN_WORD) : (value >= AGC_DP_MIN && value <= AGC_DP_MAX))) /* raw word / SP */
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (kind == 0)
    {
        *pin_own_word(idx) = (uint16_t) value;
    }
    else if (kind == 1)
    {
        *pin_own_word(idx) = pin_ones_from_sp(agc_dp_high(value));
    }
    else
    {
        *pin_own_word(idx)     = pin_ones_from_sp(agc_dp_high(value));
        *pin_own_word(idx + 1) = pin_ones_from_sp(agc_dp_low(value));
    }

    return (AGC_STATUS_OK);
}

/* ---- component access ------------------------------------------------- */

/**********************************************************************
 * AGC: none (data layer)
 *
 * Whether the value of a data id is passed raw. agc_nouns.h: an id used
 * by both a raw and a signed scale is raw (DSPTEM1 +0..+2: WHOLE, OCTAL,
 * CDU_DEG; CSM OPTION2: OCTAL and WHOLE in noun 72).
 * INTEGRATION: replace by a per-id query once io-nouns offers one.
 *
 * ARGUMENTS:
 *   Argument   Type      I/O  Description (scale)
 *   ---------  --------  ---  -------------------
 *   id         uint16_t  I    agc_noun_data_t id
 *   scale_raw  bool      I    raw flag of the component scale
 *
 * RETURNS: bool -- true when the id's value is the raw 15-bit word
 **********************************************************************/
static bool pin_id_is_raw(uint16_t id, bool scale_raw)
{
    int kind = -1; /* id kind: 0 SP raw, 1 SP, 2 DP */

    if (pin_own_id(id, &kind) >= 0)
    {
        return (kind == 0);
    }

    return (scale_raw || id == (uint16_t) AGC_DATA_OPTION2);
}

/**********************************************************************
 * AGC: SFRUTNOR, SFRUTMIX (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 346)
 *
 * Scale information of component k, with is_raw describing the value
 * convention of the component's data id. Machine-address and channel
 * nouns (01-03, 10, 15) read and store the 15-bit word itself, so a -0
 * word keeps its sign through DSPSIGN, PRSHRTMP and V01 (SP scales).
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description (scale)
 *   --------  ------------------  ---  -------------------
 *   k         int                 I    noun component 0..2 (R1..R3)
 *   info      agc_scale_info_t *  O    scale of the component, is_raw per
 *                                      data id
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_comp_scale(int k, agc_scale_info_t *info)
{
    const pin_state_t *s = pin_st(); /* pinball erasables */

    agc_status_t st; /* status so far */

    st = agc_scale_info((agc_scale_t) s->noun.scale_type[k], info);

    if (st == AGC_STATUS_OK && s->nntype == PIN_NNTYPE_NORMAL)
    {
        info->is_raw = pin_id_is_raw(s->noun.data_id[k], info->is_raw);
    }
    else if (st == AGC_STATUS_OK)
    {
        if (!AGC_ASSERT(!info->is_dp))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        info->is_raw = true; /* MCTBS / AUGMENT / CHAN: the machine word */
    }

    return (st);
}

/**********************************************************************
 * AGC: SFRUTNOR, SFRUTMIX (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 346)
 *
 * SFRUTNOR / SFRUTMIX: SF routine number of component k (0 = octal).
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   k         int   I    noun component 0..2 (R1..R3)
 *
 * RETURNS: uint8_t -- SF routine number (0 = octal only)
 **********************************************************************/
uint8_t pin_comp_routine(int k)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (0); /* no valid result */
    }

    agc_scale_info_t info; /* scale of the component */

    if (pin_comp_scale(k, &info) != AGC_STATUS_OK)
    {
        return ((uint8_t) PIN_SF_OCTAL);
    }

    if (!AGC_ASSERT(info.routine < (uint8_t) PIN_SF_COUNT))
    {
        return (0); /* no valid result */
    }

    return (info.routine);
}

/**********************************************************************
 * AGC: DPTEST (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 327)
 *
 * DPTEST: SF routines whose data is DP for octal display and load
 * (DP1OUT, DP2OUT, DP3OUT, DP4OUT and, CM only, DPFRACOT).
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   rout      uint8_t  I    SF routine number (agc_sf_routine_t)
 *
 * RETURNS: bool -- true for a DP SF routine
 **********************************************************************/
bool pin_sf_is_dp(uint8_t rout)
{
    switch (rout)
    {
        case PIN_SF_DP1:
        case PIN_SF_DP2:
        case PIN_SF_DP3:
        case PIN_SF_DP4:
            return (true);
        case PIN_SF_DPFRAC:
            return (agc_state_vehicle() == AGC_VEHICLE_CSM); /* LM: 360-CDU */
        default:
            return (false);
    }
}

/**********************************************************************
 * AGC: none (data layer)
 *
 * Value <-> 15-bit machine word for a component of the given kind.
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  -------------------
 *   info      const agc_scale_info_t *  I    scale of the component (SF
 *                                            routine, constants, raw/DP
 *                                            flags)
 *   w         uint16_t                  I    15-bit machine word
 *
 * RETURNS: agc_dp_t -- value in the component's convention
 **********************************************************************/
agc_dp_t pin_value_of_word(const agc_scale_info_t *info, uint16_t w)
{
    if (info->is_raw)
    {
        return ((agc_dp_t) (w & PIN_WORD));
    }

    return (agc_dp_from_sp(pin_sp_from_ones(w)));
}

/**********************************************************************
 * AGC: none (helper)
 *
 * Inverse of pin_value_of_word: the 15-bit word for a value.
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  -------------------
 *   info      const agc_scale_info_t *  I    scale of the component (SF
 *                                            routine, constants, raw/DP
 *                                            flags)
 *   v         agc_dp_t                  I    value (convention of the
 *                                            component)
 *
 * RETURNS: uint16_t -- the 15-bit machine word
 **********************************************************************/
static uint16_t pin_word_of_value(const agc_scale_info_t *info, agc_dp_t v)
{
    uint16_t w = info->is_raw ? (uint16_t) ((uint32_t) v & PIN_WORD) : pin_ones_from_sp(agc_dp_high(v)); /* raw word, or the high SP word */

    return (w);
}

/**********************************************************************
 * AGC: CHANDSP, CHANLOAD
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 333, 348)
 *
 * Channel for nouns 10 (CHANDSP / CHANLOAD: NOUNCADR masked LOW9).
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  -------------------
 *   k         int   I    noun component 0..2 (R1..R3)
 *
 * RETURNS: int -- channel number (NOUNCADR + k, LOW9)
 **********************************************************************/
static int pin_chan_of(int k)
{
    int ch; /* channel number */

    ch = (int) ((pin_st()->nouncadr + (uint16_t) k) & PIN_LOW9);

    return ((ch < AGC_CHAN_COUNT) ? ch : -1); /* -1: no such chan */
}

/**********************************************************************
 * AGC: MIXNOUN, CHANDSP
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 327, 333)
 *
 * Read component k of NOUNREG. MCTBS nouns read the machine address in
 * NOUNCADR (+k); noun 10 reads the channel.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   k         int         I    noun component 0..2 (R1..R3)
 *   v         agc_dp_t *  O    value read (convention of the component)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND; else the status of the routine it calls
 **********************************************************************/
agc_status_t pin_comp_read(int k, agc_dp_t *v)
{
    const pin_state_t *s  = pin_st();                 /* pinball erasables */
    agc_scale_info_t   info;                          /* scale of the component */
    agc_status_t       st = pin_comp_scale(k, &info); /* status so far */

    if (!AGC_ASSERT(v != NULL) || st != AGC_STATUS_OK)
    {
        return ((st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG); /* or BAD_ARG */
    }

    uint16_t w = 0; /* 15-bit word */

    if (s->nntype == PIN_NNTYPE_CHAN)
    {
        int ch = pin_chan_of(k); /* channel number */

        w  = (ch >= 0) ? agc_chan_read(ch) : 0u; /* no channel: 0 */
        *v = pin_value_of_word(&info, w);

        return (AGC_STATUS_OK);
    }

    if (s->nntype == PIN_NNTYPE_MCTBS || s->nntype == PIN_NNTYPE_AUGMENT)
    {
        st = agc_erase_read_ecadr((uint16_t) ((s->nouncadr + (uint16_t) k) & PIN_WORD), &w);
        *v = pin_value_of_word(&info, w);

        return (st);
    }

    int      comp = (info.routine == (uint8_t) PIN_SF_HMS) ? 0 : k; /* HMS */
    uint16_t id   = s->noun.data_id[comp];                          /* data id of the component */

    st = agc_dsky_noun_read(id, v);

    return ((st == AGC_STATUS_NOT_FOUND) ? agc_noun_read(id, v) : st); /* data layer */
} /* pin_comp_read */

/**********************************************************************
 * AGC: PUTCOM (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 347)
 *
 * Write component k of NOUNREG (PUTCOM's INDEX NOUNADD, TS k).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   k         int       I    noun component 0..2 (R1..R3)
 *   v         agc_dp_t  I    value to store (convention of the component)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition,
 *          NOT_FOUND; else the status of the routine it calls
 **********************************************************************/
agc_status_t pin_comp_write(int k, agc_dp_t v)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const pin_state_t *s  = pin_st();                 /* pinball erasables */
    agc_scale_info_t   info;                          /* scale of the component */
    agc_status_t       st = pin_comp_scale(k, &info); /* status so far */

    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX) || st != AGC_STATUS_OK)
    {
        return ((st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG); /* or BAD_ARG */
    }

    if (s->nntype == PIN_NNTYPE_CHAN)
    {
        int ch = pin_chan_of(k); /* channel number */

        if (ch < 0)
        {
            return (AGC_STATUS_BAD_ARG);
        }

        agc_chan_write(ch, pin_word_of_value(&info, v));

        return (AGC_STATUS_OK);
    }

    if (s->nntype == PIN_NNTYPE_MCTBS || s->nntype == PIN_NNTYPE_AUGMENT)
    {
        uint16_t ecadr; /* machine address of the comp */

        ecadr = (uint16_t) ((s->nouncadr + (uint16_t) k) & PIN_WORD);

        return (agc_erase_write_ecadr(ecadr, pin_word_of_value(&info, v)));
    }

    int      comp = (info.routine == (uint8_t) PIN_SF_HMS) ? 0 : k; /* HMS */
    uint16_t id   = s->noun.data_id[comp];                          /* data id of the component */

    st = agc_dsky_noun_write(id, v);

    return ((st == AGC_STATUS_NOT_FOUND) ? agc_noun_write(id, v) : st); /* data layer */
} /* pin_comp_write */

/**********************************************************************
 * AGC: TSTFORDP, MIXNOUN, CHANDSP
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 327, 332, 333)
 *
 * The word V01-V05 show for component k. TSTFORDP / MIXNOUN: for a DP
 * scale only the minor part; CHANDSP: the channel.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   k         int         I    noun component 0..2 (R1..R3)
 *   word      uint16_t *  O    15-bit word shown in octal
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_comp_octal(int k, uint16_t *word)
{
    agc_scale_info_t info;                          /* scale of the component */
    agc_dp_t         v  = 0;                        /* datum, scale of the comp */
    agc_status_t     st = pin_comp_scale(k, &info); /* status so far */

    if (!AGC_ASSERT(word != NULL) || st != AGC_STATUS_OK)
    {
        return ((st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG); /* or BAD_ARG */
    }

    st = pin_comp_read(k, &v);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (info.is_raw)
    {
        *word = (uint16_t) ((uint32_t) v & PIN_WORD);
    }
    else if (info.is_dp && pin_sf_is_dp(info.routine))
    {
        *word = pin_ones_from_sp(agc_dp_low(v)); /* minor part only  */
    }
    else
    {
        *word = pin_ones_from_sp(agc_dp_high(v));
    }

    return (AGC_STATUS_OK);
}

/* ---- arithmetic helpers (sign and magnitude, truncating) -------------- */

/**********************************************************************
 * AGC: none (helper)
 *
 * Magnitude of a signed 64-bit integer.
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   a         int64_t  I    signed integer, unscaled
 *
 * RETURNS: int64_t -- |a|
 **********************************************************************/
static int64_t pin_mag(int64_t a)
{
    return ((a < 0) ? -a : a); /* magnitude */
}

/**********************************************************************
 * AGC: none (helper: SHORTMP / DMP magnitude)
 *
 * |a| x |b| >> shift with the sign of a (b is a positive constant).
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   a         int64_t  I    signed multiplicand
 *   b         int64_t  I    positive constant multiplier
 *   shift     int      I    left shift count after the multiply
 *
 * RETURNS: int64_t -- (|a| x |b|) >> shift, with the sign of a
 **********************************************************************/
static int64_t pin_mul_shift(int64_t a, int64_t b, int shift)
{
    if (!AGC_ASSERT(pin_mag(a) <= ((int64_t) 1 << AGC_DP_BITS)))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(b >= 0 && b <= ((int64_t) 1 << AGC_DP_BITS)))
    {
        return (0); /* no valid result */
    }

    int64_t p = (pin_mag(a) * b) >> shift; /* |a| x b >> shift */

    return ((a < 0) ? -p : p); /* sign */
}

/* ---- SF output routines (pp. 335-341) --------------------------------- */

/**********************************************************************
 * AGC: DEGOUTSF, DEGCOM
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 335-336)
 *
 * DEGOUTSF / DEGCOM: CDU angle word (bit 15 = 180 deg) x .18 (or x .45
 * for optics), FIXRANGE masking the sign bit and SETAUG adding the
 * augmenter for the upper half: F = degrees / 1000 (xxx.xx).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   w         uint16_t  I    CDU angle word (bit 15 = 180 deg)
 *   optics    bool      I    true for .45 (optics) instead of .18
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_degout(uint16_t w, bool optics)
{
    if (!AGC_ASSERT(w <= PIN_WORD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int64_t mult = optics ? PIN_DEG45 : PIN_DEG18; /* .18 or .45 constant */
    int64_t aug  = 0;                              /* SETAUG augmenter */

    if ((w & PIN_BIT15) != 0u)
    { /* FIXRANGE       */
        w   = (uint16_t) (w & PIN_POSMAX);
        aug = mult; /* SETAUG         */
    }

    agc_dp_t f = (agc_dp_t) (((int64_t) w * mult) >> PIN_TP_DP_SHIFT) + (agc_dp_t) aug; /* SHORTMP, DAS: F at 2^-28 */

    return (pin_dspdecwd(f, false));
}

/**********************************************************************
 * AGC: OPDEGOUT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 335)
 *
 * OPDEGOUT (CM): optics range is 90 deg. Add the 19.775 deg bias, the
 * ones' complement word taken as two's complement (-0 = -1), and fix up
 * an overflow by putting it into bit 15; then DEGOUTSF with .45.
 * NEGOPT: a negative word plus 20BIAS is a ones' complement AD, so
 * -20BIAS + 20BIAS gives -0 (never +0: its CCSHOLE is unreachable) and
 * COM leaves -0, which DEGOUTSF shows as +89.997 deg (oracle F15).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   w         uint16_t  I    optics CDU word
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_opdegout(uint16_t w)
{
    int32_t sum; /* word + 20BIAS (ones' AD) */

    if (!pin_ones_negative(w))
    {
        sum = (int32_t) w + PIN_20BIAS; /* pos or +0      */
    }
    else if (w == PIN_NEGZERO)
    {
        sum = PIN_20BIAS - 1; /* -0: subtract 1 */
    }
    else
    { /* NEGOPT         */
        sum = PIN_20BIAS - (int32_t) pin_ones_mag(w);

        if (sum == 0)
        {
            return (pin_degout(PIN_NEGZERO, true)); /* -0: COM keeps -0 */
        }

        if (sum > 0)
        {
            sum -= 1; /* if pos then subtract 1 because of 2s comp */
        }
    }

    /* BIASCOM: a positive overflow is stored with bit 15 set; a negative
     * sum stays a ones' complement word.                                 */
    uint16_t u; /* biased word, bit 15 fixed */

    u = (sum >= 0) ? (uint16_t) sum : pin_ones_neg((uint16_t) (-sum)); /* ones' word */

    if (!AGC_ASSERT(sum < 0100000))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_degout(u, true));
} /* pin_opdegout */

/**********************************************************************
 * ARTOUTSF: point at left of the DP SF constant. PRSHRTMP (p. 380):
 * SHORTMP of the constant by the SP datum, result MPAC, MPAC+1; a -0
 * datum forces a -0 result (SHORTMP alone would give +0), shown "-".
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   sp        agc_sp_t  I    SP datum (MPAC), scale of the component
 *   negzero   bool      I    true when the datum is -0
 *   sfout     agc_dp_t  I    SF output constant, binary point left of the
 *                            DP
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_artout(agc_sp_t sp, bool negzero, agc_dp_t sfout)
{
    if (!AGC_ASSERT(sp >= AGC_SP_MIN && sp <= AGC_SP_MAX) || !AGC_ASSERT(!negzero || sp == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(sfout >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t f; /* fraction to show, 2^-28 */

    f = (agc_dp_t) pin_mul_shift(sp, sfout, PIN_TP_DP_SHIFT);

    return (pin_dspdecwd(f, negzero));
}

/**********************************************************************
 * AGC: AROUT1SF (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 336)
 *
 * AROUT1SF: point between hi and lo parts of the SF constant; PRSHRTMP,
 * then the result is shifted left 14 by taking it from MPAC+1, MPAC+2.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   sp        agc_sp_t  I    SP datum (MPAC), scale of the component
 *   negzero   bool      I    true when the datum is -0
 *   sfout     agc_dp_t  I    SF output constant, binary point between the
 *                            words
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_arout1(agc_sp_t sp, bool negzero, agc_dp_t sfout)
{
    if (!AGC_ASSERT(sp >= AGC_SP_MIN && sp <= AGC_SP_MAX) || !AGC_ASSERT(!negzero || sp == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int64_t  tp = pin_mul_shift(sp, sfout, 0);                                               /* TP product, 2^-42 units */
    agc_dp_t f  = (agc_dp_t) ((tp < 0) ? -(pin_mag(tp) & PIN_DP_MASK) : (tp & PIN_DP_MASK)); /* sign; fraction to show, 2^-28 */

    return (pin_dspdecwd(f, negzero));
}

/**********************************************************************
 * AGC: DPOUT, DP1OUTSF, DP2OUTSF
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 336-337)
 *
 * DPOUT and DP1OUTSF / DP2OUTSF (also DP4) / DP3OUTSF: DMP of the fresh DP
 * datum by the DP SF constant (TP in MPAC..+2), then DP1 shifts left 14,
 * DP3 left 7, DP2 takes MPAC, MPAC+1.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   rout      uint8_t   I    DP1..DP4 SF routine number
 *   v         agc_dp_t  I    DP datum
 *   sfout     agc_dp_t  I    SF output constant from SFOUTAB (DP, scale
 *                            per routine)
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_dpout(uint8_t rout, agc_dp_t v, agc_dp_t sfout)
{
    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX) || !AGC_ASSERT(sfout >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int64_t tp = pin_mag(pin_mul_shift(v, sfout, PIN_TP_DP_SHIFT)); /* DMP  */
    int64_t f;                                                      /* fraction to show, 2^-28 */

    if (rout == (uint8_t) PIN_SF_DP1)
    {
        f = tp & PIN_DP_MASK; /* L14/OUT    */
    }
    else if (rout == (uint8_t) PIN_SF_DP3)
    {
        f = ((tp << 7) & PIN_TP_MASK) >> PIN_TP_DP_SHIFT; /* TPLEFTN 7  */
    }
    else
    {
        f = tp >> PIN_TP_DP_SHIFT; /* DP2, DP4   */
    }

    return (pin_dspdecwd((agc_dp_t) ((v < 0) ? -f : f), false)); /* sign */
}

/**********************************************************************
 * AGC: none (helper)
 *
 * Fraction F with exactly the 5 (or 2) digits n when shown truncated.
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   n         int64_t  I    digits as an integer, 0 <= n < scale
 *   scale     int64_t  I    10^5 (5 digits) or 10^2 (2 digits)
 *
 * RETURNS: agc_dp_t -- the fraction at 2^-28 (0 on a failed precondition)
 **********************************************************************/
static agc_dp_t pin_frac_of_digits(int64_t n, int64_t scale)
{
    if (!AGC_ASSERT(n >= 0 && n < scale))
    {
        return (0); /* no valid result */
    }

    return ((agc_dp_t) (((n << AGC_DP_BITS) + scale - 1) / scale));
}

/**********************************************************************
 * AGC: HMSOUT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 338)
 *
 * HMSOUT: hours in R1, minutes (mod 60) in R2, seconds with centiseconds
 * (mod 60) in R3, each with the sign of the time; no .5 sec rounding
 * (SEPSECNR). Displayed in the AGC's order R3, R2, R1.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   t         agc_dp_t  I    time, centiseconds at 2^28 (DP, signed)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_hmsout(agc_dp_t t)
{
    bool         neg     = t < 0;         /* sign */
    int64_t      mag     = pin_mag(t);    /* magnitude, unscaled */
    int64_t      part[3] = {mag / PIN_CS_HOUR, (mag / PIN_CS_MIN) % 60, mag % PIN_CS_MIN};
    agc_status_t st      = AGC_STATUS_OK; /* status so far */

    for (int r = 2; r >= 0 && st == AGC_STATUS_OK; r--)
    {
        agc_dp_t f = pin_frac_of_digits(part[r], 100000); /* F, 2^-28 */

        pin_set_dspcount(k_rnd1[r]);
        st = pin_dspdecwd(neg ? -f : f, neg); /* DSPDECWD       */
    }

    return ((st == AGC_STATUS_OK) ? pin_entexit() : st); /* if OK */
}

/**********************************************************************
 * AGC: M/SOUT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 339)
 *
 * M/SOUT: minutes in D1D2, blank D3, seconds in D4D5, limited to 59B59.
 * SEPSEC rounds by +-.5 sec unless the magnitude is at least 59M58.5S, in
 * which case it is forced to +-59M59.5S (M/SLIMIT) without rounding.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   k         int       I    noun component 0..2 (R1..R3)
 *   t         agc_dp_t  I    time, centiseconds at 2^28 (DP, signed)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_msout(int k, agc_dp_t t)
{
    bool    neg = t < 0;      /* sign */
    int64_t mag = pin_mag(t); /* magnitude, unscaled */

    mag          = (mag >= PIN_MS_LIMIT) ? PIN_MS_FORCE : mag + PIN_RNDCON; /* M/SLIMIT */
    int64_t  sec = (mag / 100) % 60;                                        /* whole seconds 0..59 */
    int64_t  min = mag / PIN_CS_MIN;                                        /* whole minutes */
    agc_dp_t fs  = pin_frac_of_digits(sec, 100);                            /* ss, 2^-28 */
    agc_dp_t fm  = pin_frac_of_digits(min, 100);                            /* mm, 2^-28 */

    pin_set_dspcount((int16_t) (k_rnd1[k] - 3));
    agc_dp_t     ss = neg ? -fs : fs;        /* signed ss, 2^-28 */
    agc_dp_t     mm = neg ? -fm : fm;        /* signed mm, 2^-28 */
    agc_status_t st = pin_dspdc2nr(ss, neg); /* sec in D4D5 */

    if (st == AGC_STATUS_OK)
    {
        st = pin_dspin((int16_t) (k_rnd1[k] - 2), 0); /* blank middle    */
    }

    pin_set_dspcount(k_rnd1[k]);

    return ((st == AGC_STATUS_OK) ? pin_dspdc2nr(mm, neg) : st); /* D1D2 */
}

/**********************************************************************
 * AGC: LRPOSOUT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, LM p. 418)
 *
 * LRPOSOUT (LM): landing radar position, channel 33 bits 7-6 to bits
 * 2-1, complemented; displayed as whole (ARTOUTSF).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   sfout     agc_dp_t  I    SF output constant (WHOLE)
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_lrposout(agc_dp_t sfout)
{
    uint16_t c33 = agc_chan_read((int) AGC_CHAN_CHAN33);     /* channel 33 image */
    uint16_t pos = (uint16_t) (~(uint16_t) (c33 >> 5) & 3u); /* MP BIT10, COM */

    return (pin_artout((agc_sp_t) pos, false, sfout));
}

/**********************************************************************
 * AGC: 360-CDU (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, LM p. 418)
 *
 * 360-CDU (LM): if the angle is not 0 or 180 degrees, 360 minus it.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   w         uint16_t  I    CDU angle word
 *
 * RETURNS: uint16_t -- 360 deg minus the angle, or the angle at 0 / 180
 **********************************************************************/
uint16_t pin_360_cdu(uint16_t w)
{
    if ((w & PIN_POSMAX) == 0u)
    {
        return (w);
    }

    return ((uint16_t) ((0100000u - (uint32_t) w) & PIN_WORD)); /* CS MPAC, AD ONE */
}

/**********************************************************************
 * 2INTOUT (p. 339): blank the register, + sign, the high word as two
 * decimal chars in D1D2 and the low word in D4D5 (as verb and noun).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   k         int       I    noun component 0..2 (R1..R3)
 *   v         agc_dp_t  I    two integers: high word XX, low word YY
 *                            (each < 100)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_2intout(int k, agc_dp_t v)
{
    agc_sp_t hi = agc_dp_high(v); /* first integer XX, unscaled */
    agc_sp_t lo = agc_dp_low(v);  /* second integer YY, unscaled */

    if (!AGC_ASSERT(hi >= 0 && lo >= 0) || !AGC_ASSERT(k >= 0 && k < 3))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = pin_5blank_rel(PIN_INREL_X + k); /* to blank RXD3   */

    if (st == AGC_STATUS_OK)
    {
        st = pin_plus_on(); /* turn on + sign  */
    }

    if (st == AGC_STATUS_OK)
    {
        st = pin_dspdecvn(hi); /* 1st integer     */
    }

    pin_set_dspcount((int16_t) (k_rnd1[k] - 3)); /* RXD4            */

    return ((st == AGC_STATUS_OK) ? pin_dspdecvn(lo) : st); /* 2nd integer */
}

/**********************************************************************
 * AGC: SFOUTABR, DSPDCEND, DSPDECWD
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 334, 359)
 *
 * SFOUTABR, the routines that end in DSPDCEND (DSPDECWD). The datum is
 * taken as the 15-bit word MPAC holds, so a raw -0 (machine-address nouns
 * 01-03) keeps its "-" sign in DSPSIGN and PRSHRTMP.
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  -------------------
 *   info      const agc_scale_info_t *  I    scale of the component (SF
 *                                            routine, constants, raw/DP
 *                                            flags)
 *   v         agc_dp_t                  I    datum (convention of the
 *                                            component)
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_sfout_dspdcend(const agc_scale_info_t *info, agc_dp_t v)
{
    bool     lm = agc_state_vehicle() == AGC_VEHICLE_LM;                                                  /* true on the LM */
    uint16_t w  = info->is_raw ? (uint16_t) ((uint32_t) v & PIN_WORD) : pin_ones_from_sp(agc_dp_high(v)); /* raw / SP; 15-bit word */
    agc_sp_t sp = pin_sp_from_ones(w);                                                                    /* SP, scale of the comp */
    bool     nz = (w == PIN_NEGZERO);                                                                     /* word is -0 */

    switch (info->routine)
    {
        case PIN_SF_FRACT:
            return (pin_dspdecwd(agc_dp_from_sp(sp), nz));
        case PIN_SF_DEG:
            return (pin_degout(w, false));
        case PIN_SF_ARITH:
            return (pin_artout(sp, nz, info->sf_out));
        case PIN_SF_ARITH1:
            return (pin_arout1(sp, nz, info->sf_out));
        case PIN_SF_OPDEG:
            return (lm ? pin_lrposout(info->sf_out) : pin_opdegout(w));               /* routine 6 */
        case PIN_SF_DPFRAC:
            return (lm ? pin_degout(pin_360_cdu(w), false) : pin_dspdecwd(v, false)); /* routine 13 */
        case PIN_SF_DP1:
        case PIN_SF_DP2:
        case PIN_SF_DP3:
        case PIN_SF_DP4:
            return (pin_dpout(info->routine, v, info->sf_out));
        default:
            return (pin_predspal());
    }
}

/**********************************************************************
 * AGC: DSPDCPUT (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 333)
 *
 * DSPDCPUT for component k: R(k+1), then the SF routine of SFOUTABR.
 * *exited is set when the routine left through ENTEXIT itself (HMSOUT)
 * or alarmed (octal-only noun).
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   k         int       I    noun component 0..2 (R1..R3)
 *   v         agc_dp_t  I    datum (convention of the component)
 *   exited    bool *    O    true when the routine left the verb itself
 *                            (alarm, ENTEXIT)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_dspdcput(int k, agc_dp_t v, bool *exited)
{
    agc_scale_info_t info;                          /* scale of the component */
    agc_status_t     st = pin_comp_scale(k, &info); /* status so far */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    pin_set_dspcount(k_rnd1[k]);
    *exited = (info.routine == (uint8_t) PIN_SF_OCTAL || info.routine == (uint8_t) PIN_SF_HMS);

    switch (info.routine)
    {
        case PIN_SF_OCTAL:
            return (pin_predspal()); /* dec disp with octal-only noun */
        case PIN_SF_HMS:
            return (pin_hmsout(v));
        case PIN_SF_MS:
            return (pin_msout(k, v));
        case PIN_SF_2INT:
            return (pin_2intout(k, v));
        default:
            return (pin_sfout_dspdcend(&info, v));
    }
}

/**********************************************************************
 * AGC: DECDSP (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 333)
 *
 * DECDSP (V06): pick up all components first (DSPDCGET), then display
 * from the last component to R1 (DSPDCPUT / DSPDCEND).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_decdsp(void)
{
    const pin_state_t *s = pin_st();            /* pinball erasables */
    int                n = (int) s->noun.ncomp; /* number of components */

    if (!AGC_ASSERT(n >= 1 && n <= 3))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dp_t     v[3] = {0, 0, 0};
    agc_status_t st   = AGC_STATUS_OK; /* status so far */

    for (int k = n - 1; k >= 0 && st == AGC_STATUS_OK; k--)
    { /* DSPDCGET */
        st = pin_comp_read(k, &v[k]);
    }

    bool exited = false; /* routine left the verb itself */

    for (int k = n - 1; k >= 0 && st == AGC_STATUS_OK && !exited; k--)
    {
        st = pin_dspdcput(k, v[k], &exited);
    }

    if (st != AGC_STATUS_OK || exited)
    {
        return (st);
    }

    return (pin_entexit());
}

/**********************************************************************
 * AGC: DSPDPDEC (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 341)
 *
 * DSPDPDEC (V07): a DP word of a normal noun as 10 decimal chars in R1,
 * R2 (no rounding); mixed nouns alarm.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_dspdpdec(void)
{
    const pin_state_t *s = pin_st(); /* pinball erasables */

    if (s->nounreg >= PIN_MIXCON)
    {
        return (pin_dspalarm());
    }

    agc_scale_info_t info;                          /* scale of the component */
    agc_dp_t         v  = 0;                        /* datum, scale of the comp */
    agc_status_t     st = pin_comp_scale(0, &info); /* status so far */

    if (st == AGC_STATUS_OK)
    {
        st = pin_comp_read(0, &v);
    }

    if (st == AGC_STATUS_OK && !info.is_dp)
    {
        /* C(NOUNADD), C(NOUNADD +1): two SP words of the normal noun.    */
        agc_dp_t lo = 0; /* low word, 2^-28 */

        if (s->noun.ncomp > 1u)
        {
            st = pin_comp_read(1, &lo);
        }

        v = agc_dp_pack(pin_sp_from_ones(pin_word_of_value(&info, v)), agc_dp_high(lo));
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    pin_set_dspcount(PIN_R1D1);
    st = pin_dsp2dec(v); /* TPAGREE, DSP2DEC */

    return ((st == AGC_STATUS_OK) ? pin_entexit() : st); /* if OK */
} /* pin_dspdpdec */
