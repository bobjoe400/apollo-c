/*
 * pinball_sfin.c -- PINBALL decimal load scale routines.
 *
 * PINBALL_GAME_BUTTONS_AND_LIGHTS.agc CM pp. 348-354 (LM pp. 430-436):
 * PUTDECSF, SFINTABR, DEGINSF, DEGINSF2, SIGNFIX, SGNTO1, NEG180, ENDSCALE
 * (LM: 360-CDU fix-up), ARTHINSF, BINROUND, ARTIN1SF, OPTDEGIN, OPDEGIN2
 * (CM), DPINSF, DPINSF2, DPINSF4, DPFRACIN (CM), TPLEFTN, 2ROUND,
 * TESTOFUF, HMSIN, RND/TST, MPACTST, SIZETST, ALL3DEC, FORCEV25.
 *
 * The decimal datum arrives as the DP fraction N/10^5 left by DECEND in
 * XREG, XREGLP (units 2^-28). DMP by the SFINTAB constant gives a TP
 * product (units 2^-42); here the products are formed exactly in 64-bit
 * integers on sign and magnitude, as the AGC multiplies. Overflow and
 * underflow (MPAC+6, TESTOFUF) alarm and recycle.
 */
#include <stddef.h>
#include "pinball_internal.h"

/* DEGCON1 2DEC 5.555555555 B-3 (26161 30707): 1/.18 for CDU degrees. */
#define PIN_DEGCON1 ((int64_t)186413511)
/* DEGCON2 2DEC 2.222222222 B-2 (21616 07071): 100/45 for optics (CM). */
#define PIN_DEGCON2 ((int64_t)149130809)
#define PIN_NEG_2  3240            /* NEG.2 OCT -06250: the 19.775 deg bias */
#define PIN_WHOLECON ((int64_t)100000)   /* (10EXP5/2EXP14)2EXP14          */
#define PIN_CS_HOUR ((int64_t)360000)    /* HRCON: 1 hour in centi-sec     */
#define PIN_CS_MIN  ((int64_t)6000)      /* MINCON: 1 minute in centi-sec  */
#define PIN_59MIN   59                   /* 59MIN                          */
#define PIN_5999CS  5999                 /* 59.99SEC: 5999 centi-sec       */
#define PIN_DEC345  034u                 /* DECBRNCH bits 3,4,5: R3,R2,R1  */
#define PIN_DP_LIM  ((int64_t)1 << AGC_DP_BITS)
#define PIN_SP_LIM  ((int64_t)1 << AGC_SP_BITS)

static int64_t pin_abs64(int64_t a)
{
    return (a < 0) ? -a : a;
}

/* |f| x c (units 2^-56) >> 14 = the TP magnitude in MPAC..MPAC+2. */
static int64_t pin_dmp_tp(agc_dp_t f, int64_t c)
{
    (void)AGC_ASSERT(f >= AGC_DP_MIN && f <= AGC_DP_MAX);
    (void)AGC_ASSERT(c >= 0 && c < PIN_DP_LIM);
    return (pin_abs64(f) * c) >> AGC_SP_BITS;
}

/* BINROUND / 2ROUND / TESTOFUF: round MPAC+1 into MPAC; the SP result in
 * the high word of *out. Overflow alarms and recycles.                   */
static agc_status_t pin_binround(int64_t dp, agc_dp_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(pin_abs64(dp) < PIN_DP_LIM)) {
        return AGC_STATUS_BAD_ARG;
    }
    int64_t mag = pin_abs64(dp);
    int64_t hi = (mag >> AGC_SP_BITS) + (((mag & 037777) >= 020000) ? 1 : 0);
    if (hi >= PIN_SP_LIM) {
        return pin_almcycle();                   /* OF: alarm and recycle  */
    }
    *out = agc_dp_from_sp((agc_sp_t)((dp < 0) ? -hi : hi));
    return AGC_STATUS_OK;
}

/* ARTHINSF: assumes point between hi and lo parts of SFCON; shifts the
 * result left by 14 (taking it from MPAC+1, MPAC+2). MPAC must be zero:
 * else too large a load, alarm and recycle.                              */
static agc_status_t pin_arthinsf(agc_dp_t f, agc_dp_t sfin, agc_dp_t *out)
{
    int64_t tp = pin_dmp_tp(f, sfin);
    if ((tp >> AGC_DP_BITS) != 0) {
        return pin_almcycle();
    }
    int64_t dp = tp & (PIN_DP_LIM - 1);
    return pin_binround((f < 0) ? -dp : dp, out);
}

/* ARTIN1SF: scales MPAC, +1 by the SF constant and rounds MPAC+1 into MPAC. */
static agc_status_t pin_artin1sf(agc_dp_t f, agc_dp_t sfin, agc_dp_t *out)
{
    int64_t dp = pin_dmp_tp(f, sfin) >> AGC_SP_BITS;
    (void)AGC_ASSERT(dp < PIN_DP_LIM);
    return pin_binround((f < 0) ? -dp : dp, out);
}

/* DPINSF (DP1, DP3), DPINSF2 (DP2: point between bits 7-8 of the high part
 * of the SF constant, shift left 7), DPINSF4 (DP4: bits 11-12, shift left
 * 3): round MPAC+2 into MPAC+1; the low part goes to E+1 (DPFRACIN).    */
static agc_status_t pin_dpinsf(agc_dp_t f, agc_dp_t sfin, int shift, agc_dp_t *out)
{
    if (!AGC_ASSERT(shift == 0 || shift == 3 || shift == 7) || !AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    int64_t tp = pin_dmp_tp(f, sfin) << shift;               /* TPLEFTN    */
    int64_t dp = (tp >> AGC_SP_BITS) + (((tp & 037777) >= 020000) ? 1 : 0);
    if ((tp >> AGC_TP_BITS) != 0 || dp >= PIN_DP_LIM) {
        return pin_almcycle();                               /* TESTOFUF   */
    }
    *out = (agc_dp_t)((f < 0) ? -dp : dp);
    return AGC_STATUS_OK;
}

/* Two's complement 15-bit angle word of +-m (SIGNFIX, SGNTO1, NEG180):
 * after the last shift an overflow forces the sign bit to 1, an underflow
 * to 0, and -0 with underflow is -180 degrees.                           */
static uint16_t pin_angle_word(bool neg, int64_t m)
{
    (void)AGC_ASSERT(m >= 0 && m < 0100000);
    int64_t w = neg ? (0100000 - m) : m;
    return (uint16_t)(w & PIN_WORD);
}

/* DEGINSF: SF routine for decimal degrees (xxx.xx). Multiply by 5.555 B-3,
 * round MPAC+1 by BIT11 away from zero before shifting left 3 -- this
 * causes 360.00 to overflow when shifted left and alarm -- and leave the
 * CDU angle (15 bits, two's complement) in MPAC.                        */
static agc_status_t pin_deginsf(agc_dp_t f, bool cdu360, agc_dp_t *out)
{
    if (!AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    int64_t d = pin_dmp_tp(f, PIN_DEGCON1) >> AGC_SP_BITS;   /* MPAC, MPAC+1 */
    if ((d & 037777) != 0) {
        d += 02000;                                          /* +-BIT11    */
    }
    if (d >= PIN_DP_LIM || (d >> 12) >= PIN_SP_LIM) {
        return pin_almcycle();               /* OF/UF after 2 shifts       */
    }
    uint16_t w = pin_angle_word(f < 0, d >> 11);             /* left 3     */
    if (cdu360) {
        w = pin_360_cdu(w);                  /* LM ENDSCALE: 360 - CDU deg */
    }
    *out = (agc_dp_t)w;
    return AGC_STATUS_OK;
}

/* OPTDEGIN (CM): optics scaling routine, range 90 degrees. Reject a
 * negative input; subtract the bias, multiply by 100/45 B-2, round as in
 * DEGINSF (by BIT12, two shifts in DEGINSF2).
 * DEVIATION: the rounding constant is added to the signed DP; when the
 * bias makes the words disagree in sign the AGC's per-word shifts may
 * differ from this by one LSB.                                           */
static agc_status_t pin_optdegin(agc_dp_t f, bool negzero, agc_dp_t *out)
{
    if (!AGC_ASSERT(out != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (f < 0 || negzero) {
        return pin_almcycle();                   /* reject - input         */
    }
    agc_dp_t fb = f - (agc_dp_t)(PIN_NEG_2 * PIN_SP_LIM);   /* ADS MPAC   */
    int64_t d = pin_dmp_tp(fb, PIN_DEGCON2) >> AGC_SP_BITS;
    int64_t v = ((fb < 0) ? -d : d) + 04000;                 /* + BIT12    */
    int64_t m = pin_abs64(v);
    if ((m >> 13) >= PIN_SP_LIM) {
        return pin_almcycle();
    }
    *out = (agc_dp_t)pin_angle_word(v < 0, m >> 12);
    return AGC_STATUS_OK;
}

/* SFINTABR fan, part 2 (routines that need the SF constant). */
static agc_status_t pin_sfin_const(uint8_t rout, const agc_scale_info_t *info,
                                   agc_dp_t f, agc_dp_t *out)
{
    switch (rout) {
    case PIN_SF_ARITH:  return pin_arthinsf(f, info->sf_in, out);
    case PIN_SF_ARITH1: return pin_artin1sf(f, info->sf_in, out);
    case PIN_SF_DP1:
    case PIN_SF_DP3:    return pin_dpinsf(f, info->sf_in, 0, out);   /* DPINSF  */
    case PIN_SF_DP2:    return pin_dpinsf(f, info->sf_in, 7, out);   /* DPINSF2 */
    case PIN_SF_DP4:    return pin_dpinsf(f, info->sf_in, 3, out);   /* DPINSF4 */
    default:
        (void)AGC_ASSERT(rout < (uint8_t)PIN_SF_COUNT);
        return AGC_STATUS_BAD_ARG;
    }
}

agc_status_t pin_putdecsf(int k, agc_dp_t frac, bool negzero, agc_dp_t *out, bool *exited)
{
    agc_scale_info_t info;
    agc_status_t st = pin_comp_scale(k, &info);
    if (!AGC_ASSERT(out != NULL && exited != NULL) || st != AGC_STATUS_OK) {
        return (st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG;
    }
    bool lm = agc_state_vehicle() == AGC_VEHICLE_LM;
    *exited = true;
    switch (info.routine) {
    case PIN_SF_OCTAL:
        return pin_almcycle();      /* alarm and recycle if dec load with octal-only noun */
    case PIN_SF_MS:                 /* min/sec can't be loaded            */
    case PIN_SF_2INT:               /* 2INTOUT can't be loaded            */
    case PIN_SF_HMS:                /* (HMSIN is entered by the caller)   */
        return pin_dspalarm();
    case PIN_SF_OPDEG:              /* LM: LRPOSOUT can't be loaded       */
        if (lm) {
            return pin_dspalarm();
        }
        st = pin_optdegin(frac, negzero, out);
        break;
    case PIN_SF_FRACT:
        st = pin_binround(frac, out);
        break;
    case PIN_SF_DEG:
        st = pin_deginsf(frac, false, out);
        break;
    case PIN_SF_DPFRAC:             /* CM: DPFRACIN.  LM: DEGINSF (360-CDU) */
        st = lm ? pin_deginsf(frac, true, out) : AGC_STATUS_OK;
        if (!lm) {
            *out = frac;
        }
        break;
    default:
        st = pin_sfin_const(info.routine, &info, frac, out);
        break;
    }
    *exited = (st != AGC_STATUS_OK);
    return st;
}

/* ---- HMSIN (pp. 353-354) ---------------------------------------------- */

/* RND/TST: whole number from the fraction N/10^5 (DMP by WHOLECON, round
 * MPAC+2 into MPAC+1); alarm if MPAC non-zero (G/ 16383).               */
static bool pin_whole(agc_dp_t f, int64_t *n)
{
    int64_t w = (pin_abs64(f) * PIN_WHOLECON + (PIN_DP_LIM >> 1)) >> AGC_DP_BITS;
    (void)AGC_ASSERT(n != NULL);
    *n = (f < 0) ? -w : w;
    return w < PIN_SP_LIM;
}

/* HMSIN: XREG = hours, YREG = minutes, ZREG = seconds (xxx.xx), all three
 * loaded in decimal; the sum in centiseconds goes to the noun's DP
 * erasable. G/ 745 hr, 39 min, 14.59 sec alarms (DAS overflow).         */
agc_status_t pin_hmsin(void)
{
    pin_state_t *s = pin_st();
    if ((s->decbrnch & PIN_DEC345) != PIN_DEC345) {          /* ALL3DEC    */
        s->verbsave = 25;          /* FORCEV25: force verb 25 by recycle */
        return pin_almcycle();
    }
    int64_t h = 0;
    int64_t m = 0;
    int64_t sec = 0;
    bool ok = pin_whole(s->xyz_frac[0], &h) && pin_whole(s->xyz_frac[1], &m)
              && pin_whole(s->xyz_frac[2], &sec);
    ok = ok && pin_abs64(h) * PIN_CS_HOUR < PIN_DP_LIM        /* MPACTST: G/ 745 */
            && pin_abs64(m) <= PIN_59MIN                       /* SIZETST         */
            && pin_abs64(sec) <= PIN_5999CS;
    int64_t t = h * PIN_CS_HOUR + m * PIN_CS_MIN;              /* DAS HITEMIN     */
    ok = ok && pin_abs64(t) < PIN_DP_LIM;
    t += sec;
    ok = ok && pin_abs64(t) < PIN_DP_LIM;
    if (!ok) {
        return pin_almcycle();
    }
    (void)AGC_ASSERT(pin_comp_routine(0) == (uint8_t)PIN_SF_HMS);
    agc_status_t st = pin_comp_write(0, (agc_dp_t)t);          /* TPAGREE, DXCH */
    return (st == AGC_STATUS_OK) ? pin_loadlv() : st;
}
