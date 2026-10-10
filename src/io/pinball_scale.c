/*
 * pinball_scale.c -- noun component access and the decimal display scale
 * routines of PINBALL.
 *
 * PINBALL_GAME_BUTTONS_AND_LIGHTS.agc CM pp. 327, 333-342 (LM pp. 409,
 * 415-424): MIXNOUN, DPTEST, TSTFORDP, CHANDSP, READLO, DECDSP, DSPDCGET,
 * DSPDCPUT, DSPDCEND, SFOUTABR and the SF output routines DEGOUTSF,
 * OPDEGOUT (CM), LRPOSOUT (LM), 360-CDUO (LM), FIXRANGE, SETAUG, DEGCOM,
 * ARTOUTSF, AROUT1SF, DP1OUTSF, DP2OUTSF, DP3OUTSF, DPOUT, 2INTOUT,
 * DPFRACOT (CM), HMSOUT, M/SOUT, SEPSEC, SEPSECNR, SEPMIN, DSPDPDEC.
 *
 * Numbers. The AGC scales a value for display into a DP fraction F whose
 * 5 decimal digits are shown (DSPDECWD: F x 10^5, rounded by 5 E-6). The
 * multiplications are the AGC's: SHORTMP (SP x DP -> TP) and DMP (DP x DP
 * -> TP) on sign and magnitude, truncating, with the products formed here
 * exactly in 64-bit integers (units: SP 2^-14, DP 2^-28, TP 2^-42). This
 * is display conversion, not flight arithmetic. HMSOUT and M/SOUT derive
 * the same digits by integer division of the centisecond count.
 */
#include <stddef.h>
#include "pinball_internal.h"

#define PIN_TP_DP_SHIFT 14                         /* TP -> DP (drop MPAC+2) */
#define PIN_DP_MASK  (((int64_t)1 << AGC_DP_BITS) - 1)
#define PIN_TP_MASK  (((int64_t)1 << AGC_TP_BITS) - 1)
/* DEGTAB: .18 (05605 03656) and .45 (16314 31463) as DP fractions.        */
#define PIN_DEG18 ((int64_t)(05605 * 040000 + 03656))
#define PIN_DEG45 ((int64_t)(016314 * 040000 + 031463))
#define PIN_20BIAS 016040                          /* 20BIAS: 19.775 deg     */
#define PIN_CS_HOUR 360000                         /* HRCON                  */
#define PIN_CS_MIN  6000                           /* MINCON                 */
#define PIN_MS_LIMIT 359850                        /* 59M58.5S (M/SCON1,2)   */
#define PIN_MS_FORCE 359950                        /* 59M59.5S (M/SCON3)     */
#define PIN_RNDCON 50                              /* RNDCON .5 sec          */

static const int16_t k_rnd1[3] = { PIN_R1D1, PIN_R2D1, PIN_R3D1 };

/* ---- pinball's own noun erasables ------------------------------------- */

/* Storage word index in pin_state_t.dsptem[] (or 10+k for X/Y/ZREG) and
 * kind (0 SP raw, 1 SP, 2 DP) of a pinball-owned data id; -1 if not ours.
 * DSPTEMX = OPTIONX = DSPTEM2 +1; NORMTEM1 = DSPTEM1.                     */
static int pin_own_id(uint16_t id, int *kind)
{
    (void)AGC_ASSERT(pin_invariants());
    switch (id) {
    case AGC_DATA_DSPTEM1:       *kind = 0; return 0;
    case AGC_DATA_DSPTEM1_P1:    *kind = 0; return 1;
    case AGC_DATA_DSPTEM1_P2:    *kind = 0; return 2;
    case AGC_DATA_DSPTEM1_DP:    *kind = 2; return 0;
    case AGC_DATA_DSPTEM2:       *kind = 1; return 3;
    case AGC_DATA_DSPTEM2_P1:    *kind = 1; return 4;
    case AGC_DATA_DSPTEM2_P2:    *kind = 1; return 5;
    case AGC_DATA_DSPTEM2_P1_DP: *kind = 2; return 4;
    case AGC_DATA_DSPTEMX:       *kind = 1; return 4;
    case AGC_DATA_DSPTEMX_P1:    *kind = 1; return 5;
    case AGC_DATA_DSPTEMX_P2:    *kind = 1; return 6;
    case AGC_DATA_DSPTEMX_DP:    *kind = 2; return 4;
    case AGC_DATA_OPTIONX:       *kind = 0; return 4;
    case AGC_DATA_OPTIONX_P1:    *kind = 0; return 5;
    case AGC_DATA_XREG:          *kind = 0; return 10;
    case AGC_DATA_YREG:          *kind = 0; return 11;
    case AGC_DATA_ZREG:          *kind = 0; return 12;
    default:                     *kind = -1; return -1;
    }
}

static uint16_t *pin_own_word(int idx)
{
    (void)AGC_ASSERT(pin_invariants());
    pin_state_t *s = pin_st();
    return (idx >= 10) ? &s->xyz_oct[idx - 10] : &s->dsptem[idx];
}

agc_status_t agc_dsky_noun_read(uint16_t data_id, agc_dp_t *value)
{
    int kind = -1;
    int idx = pin_own_id(data_id, &kind);
    if (!AGC_ASSERT(value != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (idx < 0) {
        return AGC_STATUS_NOT_FOUND;
    }
    (void)AGC_ASSERT(idx <= 12 && (idx < 6 || idx >= 10 || kind == 1));
    uint16_t w = *pin_own_word(idx);
    if (kind == 0) {
        *value = (agc_dp_t)w;                           /* raw word         */
    } else if (kind == 1) {
        *value = agc_dp_from_sp(pin_sp_from_ones(w));
    } else {
        *value = agc_dp_pack(pin_sp_from_ones(w), pin_sp_from_ones(*pin_own_word(idx + 1)));
    }
    return AGC_STATUS_OK;
}

agc_status_t agc_dsky_noun_write(uint16_t data_id, agc_dp_t value)
{
    (void)AGC_ASSERT(pin_invariants());
    int kind = -1;
    int idx = pin_own_id(data_id, &kind);
    if (idx < 0) {
        return AGC_STATUS_NOT_FOUND;
    }
    if (!AGC_ASSERT(kind == 0 ? (value >= 0 && value <= (agc_dp_t)PIN_WORD)
                              : (value >= AGC_DP_MIN && value <= AGC_DP_MAX))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (kind == 0) {
        *pin_own_word(idx) = (uint16_t)value;
    } else if (kind == 1) {
        *pin_own_word(idx) = pin_ones_from_sp(agc_dp_high(value));
    } else {
        *pin_own_word(idx) = pin_ones_from_sp(agc_dp_high(value));
        *pin_own_word(idx + 1) = pin_ones_from_sp(agc_dp_low(value));
    }
    return AGC_STATUS_OK;
}

/* ---- component access ------------------------------------------------- */

/* Whether the value of a data id is passed raw. agc_nouns.h: an id used
 * by both a raw and a signed scale is raw (DSPTEM1 +0..+2: WHOLE, OCTAL,
 * CDU_DEG; CSM OPTION2: OCTAL and WHOLE in noun 72).
 * INTEGRATION: replace by a per-id query once io-nouns offers one.      */
static bool pin_id_is_raw(uint16_t id, bool scale_raw)
{
    (void)AGC_ASSERT(pin_invariants());
    int kind = -1;
    if (pin_own_id(id, &kind) >= 0) {
        return kind == 0;
    }
    return scale_raw || id == (uint16_t)AGC_DATA_OPTION2;
}

/* Scale information of component k, with is_raw describing the value
 * convention of the component's data id.                                */
agc_status_t pin_comp_scale(int k, agc_scale_info_t *info)
{
    const pin_state_t *s = pin_st();
    if (!AGC_ASSERT(k >= 0 && k < 3)
        || !AGC_ASSERT(info != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = agc_scale_info((agc_scale_t)s->noun.scale_type[k], info);
    if (st == AGC_STATUS_OK && s->nntype == PIN_NNTYPE_NORMAL) {
        info->is_raw = pin_id_is_raw(s->noun.data_id[k], info->is_raw);
    }
    return st;
}

/* SFRUTNOR / SFRUTMIX: SF routine number of component k (0 = octal). */
uint8_t pin_comp_routine(int k)
{
    (void)AGC_ASSERT(pin_invariants());
    agc_scale_info_t info;
    if (pin_comp_scale(k, &info) != AGC_STATUS_OK) {
        return (uint8_t)PIN_SF_OCTAL;
    }
    (void)AGC_ASSERT(info.routine < (uint8_t)PIN_SF_COUNT);
    return info.routine;
}

/* DPTEST: SF routines whose data is DP for octal display and load
 * (DP1OUT, DP2OUT, DP3OUT, DP4OUT and, CM only, DPFRACOT).               */
bool pin_sf_is_dp(uint8_t rout)
{
    (void)AGC_ASSERT(pin_invariants());
    (void)AGC_ASSERT(rout < (uint8_t)PIN_SF_COUNT);
    switch (rout) {
    case PIN_SF_DP1: case PIN_SF_DP2: case PIN_SF_DP3: case PIN_SF_DP4:
        return true;
    case PIN_SF_DPFRAC:
        return agc_state_vehicle() == AGC_VEHICLE_CSM;   /* LM: 360-CDU    */
    default:
        return false;
    }
}

/* Value <-> 15-bit machine word for a component of the given kind. */
agc_dp_t pin_value_of_word(const agc_scale_info_t *info, uint16_t w)
{
    (void)AGC_ASSERT(pin_invariants());
    if (info->is_raw) {
        return (agc_dp_t)(w & PIN_WORD);
    }
    return agc_dp_from_sp(pin_sp_from_ones(w));
}

static uint16_t pin_word_of_value(const agc_scale_info_t *info, agc_dp_t v)
{
    (void)AGC_ASSERT(pin_invariants());
    if (info->is_raw) {
        return (uint16_t)((uint32_t)v & PIN_WORD);
    }
    return pin_ones_from_sp(agc_dp_high(v));
}

/* Channel for nouns 10 (CHANDSP / CHANLOAD: NOUNCADR masked LOW9). */
static int pin_chan_of(int k)
{
    (void)AGC_ASSERT(pin_invariants());
    int ch = (int)((pin_st()->nouncadr + (uint16_t)k) & PIN_LOW9);
    return (ch < AGC_CHAN_COUNT) ? ch : -1;
}

/* Read component k of NOUNREG. MCTBS nouns read the machine address in
 * NOUNCADR (+k); noun 10 reads the channel.                             */
agc_status_t pin_comp_read(int k, agc_dp_t *v)
{
    (void)AGC_ASSERT(pin_invariants());
    const pin_state_t *s = pin_st();
    agc_scale_info_t info;
    agc_status_t st = pin_comp_scale(k, &info);
    if (!AGC_ASSERT(v != NULL) || st != AGC_STATUS_OK) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    uint16_t w = 0;
    if (s->nntype == PIN_NNTYPE_CHAN) {
        int ch = pin_chan_of(k);
        w = (ch >= 0) ? agc_chan_read(ch) : 0u;
        *v = pin_value_of_word(&info, w);
        return AGC_STATUS_OK;
    }
    if (s->nntype == PIN_NNTYPE_MCTBS || s->nntype == PIN_NNTYPE_AUGMENT) {
        st = agc_erase_read_ecadr((uint16_t)((s->nouncadr + (uint16_t)k) & PIN_WORD), &w);
        *v = pin_value_of_word(&info, w);
        return st;
    }
    uint16_t id = s->noun.data_id[(info.routine == (uint8_t)PIN_SF_HMS) ? 0 : k];
    st = agc_dsky_noun_read(id, v);
    return (st == AGC_STATUS_NOT_FOUND) ? agc_noun_read(id, v) : st;
}

/* Write component k of NOUNREG (PUTCOM's INDEX NOUNADD, TS k). */
agc_status_t pin_comp_write(int k, agc_dp_t v)
{
    (void)AGC_ASSERT(pin_invariants());
    const pin_state_t *s = pin_st();
    agc_scale_info_t info;
    agc_status_t st = pin_comp_scale(k, &info);
    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX) || st != AGC_STATUS_OK) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    if (s->nntype == PIN_NNTYPE_CHAN) {
        int ch = pin_chan_of(k);
        return (ch >= 0) ? agc_chan_write(ch, pin_word_of_value(&info, v)) : AGC_STATUS_BAD_ARG;
    }
    if (s->nntype == PIN_NNTYPE_MCTBS || s->nntype == PIN_NNTYPE_AUGMENT) {
        uint16_t ecadr = (uint16_t)((s->nouncadr + (uint16_t)k) & PIN_WORD);
        return agc_erase_write_ecadr(ecadr, pin_word_of_value(&info, v));
    }
    uint16_t id = s->noun.data_id[(info.routine == (uint8_t)PIN_SF_HMS) ? 0 : k];
    st = agc_dsky_noun_write(id, v);
    return (st == AGC_STATUS_NOT_FOUND) ? agc_noun_write(id, v) : st;
}

/* The word V01-V05 show for component k. TSTFORDP / MIXNOUN: for a DP
 * scale only the minor part; CHANDSP: the channel.                      */
agc_status_t pin_comp_octal(int k, uint16_t *word)
{
    (void)AGC_ASSERT(pin_invariants());
    agc_scale_info_t info;
    agc_dp_t v = 0;
    agc_status_t st = pin_comp_scale(k, &info);
    if (!AGC_ASSERT(word != NULL) || st != AGC_STATUS_OK) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    st = pin_comp_read(k, &v);
    if (st != AGC_STATUS_OK) {
        return st;
    }
    if (info.is_raw) {
        *word = (uint16_t)((uint32_t)v & PIN_WORD);
    } else if (info.is_dp && pin_sf_is_dp(info.routine)) {
        *word = pin_ones_from_sp(agc_dp_low(v));      /* minor part only  */
    } else {
        *word = pin_ones_from_sp(agc_dp_high(v));
    }
    return AGC_STATUS_OK;
}

/* ---- arithmetic helpers (sign and magnitude, truncating) -------------- */

static int64_t pin_mag(int64_t a)
{
    return (a < 0) ? -a : a;
}

/* |a| x |b| >> shift with the sign of a (b is a positive constant). */
static int64_t pin_mul_shift(int64_t a, int64_t b, int shift)
{
    (void)AGC_ASSERT(pin_mag(a) <= ((int64_t)1 << AGC_DP_BITS));
    (void)AGC_ASSERT(b >= 0 && b <= ((int64_t)1 << AGC_DP_BITS));
    int64_t p = (pin_mag(a) * b) >> shift;
    return (a < 0) ? -p : p;
}

/* ---- SF output routines (pp. 335-341) --------------------------------- */

/* DEGOUTSF / DEGCOM: CDU angle word (bit 15 = 180 deg) x .18 (or x .45
 * for optics), FIXRANGE masking the sign bit and SETAUG adding the
 * augmenter for the upper half: F = degrees / 1000 (xxx.xx).            */
static agc_status_t pin_degout(uint16_t w, bool optics)
{
    if (!AGC_ASSERT(w <= PIN_WORD)) {
        return AGC_STATUS_BAD_ARG;
    }
    int64_t mult = optics ? PIN_DEG45 : PIN_DEG18;
    int64_t aug = 0;
    if ((w & PIN_BIT15) != 0u) {                         /* FIXRANGE       */
        w = (uint16_t)(w & PIN_POSMAX);
        aug = mult;                                      /* SETAUG         */
    }
    agc_dp_t f = (agc_dp_t)(((int64_t)w * mult) >> PIN_TP_DP_SHIFT) + (agc_dp_t)aug; /* SHORTMP, DAS */
    (void)AGC_ASSERT(f >= 0 && f <= AGC_DP_MAX);
    return pin_dspdecwd(f, false);
}

/* OPDEGOUT (CM): optics range is 90 deg. Add the 19.775 deg bias, the
 * ones' complement word taken as two's complement (-0 = -1), and fix up
 * an overflow by putting it into bit 15; then DEGOUTSF with .45.        */
static agc_status_t pin_opdegout(uint16_t w)
{
    (void)AGC_ASSERT(pin_invariants());
    int32_t sum;
    if (!pin_ones_negative(w)) {
        sum = (int32_t)w + PIN_20BIAS;                   /* pos or +0      */
    } else if (w == PIN_NEGZERO) {
        sum = PIN_20BIAS - 1;                            /* -0: subtract 1 */
    } else {                                             /* NEGOPT         */
        sum = PIN_20BIAS - (int32_t)pin_ones_mag(w);
        if (sum == 0) {
            return agc_ccshole();
        }
        if (sum > 0) {
            sum -= 1;          /* if pos then subtract 1 because of 2s comp */
        }
    }
    /* BIASCOM: a positive overflow is stored with bit 15 set; a negative
     * sum stays a ones' complement word.                                 */
    uint16_t u = (sum >= 0) ? (uint16_t)sum : pin_ones_neg((uint16_t)(-sum));
    (void)AGC_ASSERT(sum < 0100000);
    return pin_degout(u, true);
}

/* ARTOUTSF: point at left of the DP SF constant. SHORTMP of the constant
 * by the SP datum, result MPAC, MPAC+1.                                  */
static agc_status_t pin_artout(agc_sp_t sp, agc_dp_t sfout)
{
    (void)AGC_ASSERT(pin_invariants());
    (void)AGC_ASSERT(sfout >= 0);
    agc_dp_t f = (agc_dp_t)pin_mul_shift(sp, sfout, PIN_TP_DP_SHIFT);
    return pin_dspdecwd(f, false);
}

/* AROUT1SF: point between hi and lo parts of the SF constant; the result
 * is shifted left 14 by taking it from MPAC+1, MPAC+2.                   */
static agc_status_t pin_arout1(agc_sp_t sp, agc_dp_t sfout)
{
    (void)AGC_ASSERT(pin_invariants());
    int64_t tp = pin_mul_shift(sp, sfout, 0);
    agc_dp_t f = (agc_dp_t)((tp < 0) ? -(pin_mag(tp) & PIN_DP_MASK) : (tp & PIN_DP_MASK));
    return pin_dspdecwd(f, false);
}

/* DPOUT and DP1OUTSF / DP2OUTSF (also DP4) / DP3OUTSF: DMP of the fresh DP
 * datum by the DP SF constant (TP in MPAC..+2), then DP1 shifts left 14,
 * DP3 left 7, DP2 takes MPAC, MPAC+1.                                    */
static agc_status_t pin_dpout(uint8_t rout, agc_dp_t v, agc_dp_t sfout)
{
    if (!AGC_ASSERT(v >= AGC_DP_MIN && v <= AGC_DP_MAX)
        || !AGC_ASSERT(sfout >= 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    int64_t tp = pin_mag(pin_mul_shift(v, sfout, PIN_TP_DP_SHIFT));   /* DMP  */
    int64_t f;
    if (rout == (uint8_t)PIN_SF_DP1) {
        f = tp & PIN_DP_MASK;                                  /* L14/OUT    */
    } else if (rout == (uint8_t)PIN_SF_DP3) {
        f = ((tp << 7) & PIN_TP_MASK) >> PIN_TP_DP_SHIFT;     /* TPLEFTN 7  */
    } else {
        f = tp >> PIN_TP_DP_SHIFT;                             /* DP2, DP4   */
    }
    return pin_dspdecwd((agc_dp_t)((v < 0) ? -f : f), false);
}

/* 2INTOUT: blank the register, + sign, the high word as 2 decimal chars in
 * D1D2 and the low word in D4D5 (like verb and noun).                    */
static agc_status_t pin_2intout(int k, agc_dp_t v)
{
    agc_sp_t hi = agc_dp_high(v);
    agc_sp_t lo = agc_dp_low(v);
    if (!AGC_ASSERT(hi >= 0 && lo >= 0)
        || !AGC_ASSERT(k >= 0 && k < 3)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_status_t st = pin_5blank_rel(PIN_INREL_X + k);     /* to blank RXD3   */
    if (st == AGC_STATUS_OK) {
        st = pin_plus_on();                                /* turn on + sign  */
    }
    if (st == AGC_STATUS_OK) {
        st = pin_dspdecvn(hi);                             /* 1st integer     */
    }
    pin_set_dspcount((int16_t)(k_rnd1[k] - 3));            /* RXD4            */
    return (st == AGC_STATUS_OK) ? pin_dspdecvn(lo) : st;  /* 2nd integer     */
}

/* Fraction F with exactly the 5 (or 2) digits n when shown truncated. */
static agc_dp_t pin_frac_of_digits(int64_t n, int64_t scale)
{
    (void)AGC_ASSERT(pin_invariants());
    (void)AGC_ASSERT(n >= 0 && n < scale);
    return (agc_dp_t)(((n << AGC_DP_BITS) + scale - 1) / scale);
}

/* HMSOUT: hours in R1, minutes (mod 60) in R2, seconds with centiseconds
 * (mod 60) in R3, each with the sign of the time; no .5 sec rounding
 * (SEPSECNR). Displayed in the AGC's order R3, R2, R1.                   */
static agc_status_t pin_hmsout(agc_dp_t t)
{
    (void)AGC_ASSERT(pin_invariants());
    if (!AGC_ASSERT(t >= AGC_DP_MIN && t <= AGC_DP_MAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    bool neg = t < 0;
    int64_t mag = pin_mag(t);
    int64_t part[3] = { mag / PIN_CS_HOUR, (mag / PIN_CS_MIN) % 60, mag % PIN_CS_MIN };
    agc_status_t st = AGC_STATUS_OK;
    for (int r = 2; r >= 0 && st == AGC_STATUS_OK; r--) {
        agc_dp_t f = pin_frac_of_digits(part[r], 100000);
        pin_set_dspcount(k_rnd1[r]);
        st = pin_dspdecwd(neg ? -f : f, neg);              /* DSPDECWD       */
    }
    return (st == AGC_STATUS_OK) ? pin_entexit() : st;
}

/* M/SOUT: minutes in D1D2, blank D3, seconds in D4D5, limited to 59B59.
 * SEPSEC rounds by +-.5 sec unless the magnitude is at least 59M58.5S, in
 * which case it is forced to +-59M59.5S (M/SLIMIT) without rounding.    */
static agc_status_t pin_msout(int k, agc_dp_t t)
{
    if (!AGC_ASSERT(k >= 0 && k < 3)
        || !AGC_ASSERT(t >= AGC_DP_MIN && t <= AGC_DP_MAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    bool neg = t < 0;
    int64_t mag = pin_mag(t);
    mag = (mag >= PIN_MS_LIMIT) ? PIN_MS_FORCE : mag + PIN_RNDCON;
    int64_t sec = (mag / 100) % 60;
    int64_t min = mag / PIN_CS_MIN;
    agc_dp_t fs = pin_frac_of_digits(sec, 100);
    agc_dp_t fm = pin_frac_of_digits(min, 100);
    pin_set_dspcount((int16_t)(k_rnd1[k] - 3));
    agc_status_t st = pin_dspdc2nr(neg ? -fs : fs, neg);   /* sec in D4D5     */
    if (st == AGC_STATUS_OK) {
        st = pin_dspin((int16_t)(k_rnd1[k] - 2), 0);       /* blank middle    */
    }
    pin_set_dspcount(k_rnd1[k]);
    return (st == AGC_STATUS_OK) ? pin_dspdc2nr(neg ? -fm : fm, neg) : st;  /* min D1D2 */
}

/* LRPOSOUT (LM): landing radar position, channel 33 bits 7-6 to bits
 * 2-1, complemented; displayed as whole (ARTOUTSF).                     */
static agc_status_t pin_lrposout(agc_dp_t sfout)
{
    (void)AGC_ASSERT(pin_invariants());
    uint16_t c33 = agc_chan_read((int)AGC_CHAN_CHAN33);
    uint16_t pos = (uint16_t)(~(uint16_t)(c33 >> 5) & 3u);   /* MP BIT10, COM */
    (void)AGC_ASSERT(pos <= 3u);
    return pin_artout((agc_sp_t)pos, sfout);
}

/* 360-CDU (LM): if the angle is not 0 or 180 degrees, 360 minus it. */
uint16_t pin_360_cdu(uint16_t w)
{
    (void)AGC_ASSERT(pin_invariants());
    (void)AGC_ASSERT(w <= PIN_WORD);
    if ((w & PIN_POSMAX) == 0u) {
        return w;
    }
    return (uint16_t)((0100000u - (uint32_t)w) & PIN_WORD);  /* CS MPAC, AD ONE */
}

/* SFOUTABR fan, part 2: the routines that end in DSPDCEND (DSPDECWD). */
static agc_status_t pin_sfout_word(uint8_t rout, const agc_scale_info_t *info, agc_dp_t v)
{
    (void)AGC_ASSERT(pin_invariants());
    bool lm = agc_state_vehicle() == AGC_VEHICLE_LM;
    uint16_t w = info->is_raw ? (uint16_t)((uint32_t)v & PIN_WORD) : pin_ones_from_sp(agc_dp_high(v));
    agc_sp_t sp = pin_sp_from_ones(w);
    switch (rout) {
    case PIN_SF_FRACT:  return pin_dspdecwd(agc_dp_from_sp(sp), false);
    case PIN_SF_DEG:    return pin_degout(w, false);
    case PIN_SF_ARITH:  return pin_artout(sp, info->sf_out);
    case PIN_SF_ARITH1: return pin_arout1(sp, info->sf_out);
    case PIN_SF_OPDEG:  return lm ? pin_lrposout(info->sf_out) : pin_opdegout(w);
    case PIN_SF_DPFRAC: return lm ? pin_degout(pin_360_cdu(w), false) : pin_dspdecwd(v, false);
    case PIN_SF_DP1: case PIN_SF_DP2: case PIN_SF_DP3: case PIN_SF_DP4:
        return pin_dpout(rout, v, info->sf_out);
    default:
        (void)AGC_ASSERT(rout < (uint8_t)PIN_SF_COUNT);
        return pin_predspal();
    }
}

/* DSPDCPUT for component k: R(k+1), then the SF routine. *exited is set
 * when the routine left through ENTEXIT itself (HMSOUT) or alarmed.     */
static agc_status_t pin_dspdcput(int k, agc_dp_t v, bool *exited)
{
    (void)AGC_ASSERT(pin_invariants());
    agc_scale_info_t info;
    agc_status_t st = pin_comp_scale(k, &info);
    if (!AGC_ASSERT(exited != NULL) || st != AGC_STATUS_OK) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    pin_set_dspcount(k_rnd1[k]);
    *exited = true;
    switch (info.routine) {
    case PIN_SF_OCTAL: return pin_predspal();  /* dec disp with octal-only noun */
    case PIN_SF_HMS:   return pin_hmsout(v);
    case PIN_SF_MS:    *exited = false; return pin_msout(k, v);
    case PIN_SF_2INT:  *exited = false; return pin_2intout(k, v);
    default:
        *exited = false;
        return pin_sfout_word(info.routine, &info, v);
    }
}

/* DECDSP (V06): pick up all components first (DSPDCGET), then display
 * from the last component to R1 (DSPDCPUT / DSPDCEND).                 */
agc_status_t pin_decdsp(void)
{
    (void)AGC_ASSERT(pin_invariants());
    const pin_state_t *s = pin_st();
    int n = (int)s->noun.ncomp;
    if (!AGC_ASSERT(n >= 1 && n <= 3)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t v[3] = { 0, 0, 0 };
    agc_status_t st = AGC_STATUS_OK;
    for (int k = n - 1; k >= 0 && st == AGC_STATUS_OK; k--) {   /* DSPDCGET */
        st = pin_comp_read(k, &v[k]);
    }
    bool exited = false;
    for (int k = n - 1; k >= 0 && st == AGC_STATUS_OK && !exited; k--) {
        st = pin_dspdcput(k, v[k], &exited);
    }
    if (st != AGC_STATUS_OK || exited) {
        return st;
    }
    return pin_entexit();
}

/* DSPDPDEC (V07): a DP word of a normal noun as 10 decimal chars in R1,
 * R2 (no rounding); mixed nouns alarm.                                   */
agc_status_t pin_dspdpdec(void)
{
    (void)AGC_ASSERT(pin_invariants());
    const pin_state_t *s = pin_st();
    if (s->nounreg >= PIN_MIXCON) {
        return pin_dspalarm();
    }
    agc_scale_info_t info;
    agc_dp_t v = 0;
    agc_status_t st = pin_comp_scale(0, &info);
    if (st == AGC_STATUS_OK) {
        st = pin_comp_read(0, &v);
    }
    if (st == AGC_STATUS_OK && !info.is_dp) {
        /* C(NOUNADD), C(NOUNADD +1): two SP words of the normal noun.    */
        agc_dp_t lo = 0;
        if (s->noun.ncomp > 1u) {
            st = pin_comp_read(1, &lo);
        }
        v = agc_dp_pack(pin_sp_from_ones(pin_word_of_value(&info, v)), agc_dp_high(lo));
    }
    if (st != AGC_STATUS_OK) {
        return st;
    }
    pin_set_dspcount(PIN_R1D1);
    st = pin_dsp2dec(v);                                   /* TPAGREE, DSP2DEC */
    return (st == AGC_STATUS_OK) ? pin_entexit() : st;
}
