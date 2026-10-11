/**********************************************************************
 * FILE NAME: sfin.c
 *
 * PURPOSE:
 *   PINBALL decimal load scaling: the SFINTABR fan that turns the keyed
 *   decimal fraction into the word(s) to store for each SF routine.
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pp. 348-354 (PUTDECSF,
 *     SFINTABR, DEGINSF, DEGINSF2, SIGNFIX, SGNTO1, NEG180, ARTHINSF,
 *     BINROUND, ARTIN1SF, OPTDEGIN, OPDEGIN2, DPINSF, DPINSF2, DPINSF4,
 *     DPFRACIN, TPLEFTN, 2ROUND, TESTOFUF, HMSIN, RND/TST, MPACTST, SIZETST,
 *     ALL3DEC, FORCEV25)
 *   Luminary099 pp. 430-436 (differs: no OPTDEGIN / DPFRACIN; ENDSCALE
 *     360-CDU fix-up)
 *
 * EXTERNAL VARIABLES:
 *   none (works on the pinball state XREG.., the noun tables' constants)
 *
 * EXTERNAL REFERENCES:
 *   Name               Description
 *   -----------------  -----------------------------------------------
 *   agc_noun_write     store the scaled value (HMSIN, via pinball)
 *   agc_state_vehicle  CM / LM routine differences
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Overflow, underflow and out-of-range loads alarm and recycle
 *   (ALMCYCLE: PIN_STATUS_RECYCLE); HMSIN with fewer than three decimal
 *   words forces V25 (FORCEV25). BAD_ARG on a failed precondition.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Runs in the CHARIN job after ENTER. Both vehicles.
 *
 * NOTES:
 *   DEVIATION (OPTDEGIN, CM): the BIT12 rounding is added to the signed
 *   DP; with words of disagreeing sign the AGC's per-word shifts may differ
 *   by one LSB.
 *
 * ALGORITHM:
 *   The decimal datum arrives as the DP fraction N/10^5 left by DECEND in
 *   XREG, XREGLP (units 2^-28). DMP by the SFINTAB constant gives a TP
 *   product (units 2^-42); here the products are formed exactly in 64-bit
 *   integers on sign and magnitude, as the AGC multiplies. Overflow and
 *   underflow (MPAC+6, TESTOFUF) alarm and recycle.
 **********************************************************************/
#include <stddef.h>

#include "internal.h"

/* DEGCON1 2DEC 5.555555555 B-3 (26161 30707): 1/.18 for CDU degrees. */
#define PIN_DEGCON1 ((int64_t) 186413511)
/* DEGCON2 2DEC 2.222222222 B-2 (21616 07071): 100/45 for optics (CM). */
#define PIN_DEGCON2  ((int64_t) 149130809)
#define PIN_NEG_2    3240 /* NEG.2 OCT -06250: the 19.775 deg bias */
#define PIN_WHOLECON ((int64_t) 100000) /* (10EXP5/2EXP14)2EXP14          */
#define PIN_CS_HOUR  ((int64_t) 360000) /* HRCON: 1 hour in centi-sec     */
#define PIN_CS_MIN   ((int64_t) 6000)   /* MINCON: 1 minute in centi-sec  */
#define PIN_59MIN    59                 /* 59MIN                          */
#define PIN_5999CS   5999               /* 59.99SEC: 5999 centi-sec       */
#define PIN_DEC345   034u               /* DECBRNCH bits 3,4,5: R3,R2,R1  */
#define PIN_DP_LIM   ((int64_t) 1 << AGC_DP_BITS)
#define PIN_SP_LIM   ((int64_t) 1 << AGC_SP_BITS)

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
static int64_t pin_abs64(int64_t a)
{
    return ((a < 0) ? -a : a);
}

/**********************************************************************
 * AGC: none (helper: DMP)
 *
 * |f| x c (units 2^-56) >> 14 = the TP magnitude in MPAC..MPAC+2.
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  -------------------
 *   f         agc_dp_t  I    DP fraction at 2^-28
 *   c         int64_t   I    DP constant, magnitude at scale 2^-28
 *
 * RETURNS: int64_t -- TP magnitude at 2^-42 (0 on a failed precondition)
 **********************************************************************/
static int64_t pin_dmp_tp(agc_dp_t f, int64_t c)
{
    if (!AGC_ASSERT(f >= AGC_DP_MIN && f <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(c >= 0 && c < PIN_DP_LIM))
    {
        return (0); /* no valid result */
    }

    return ((pin_abs64(f) * c) >> AGC_SP_BITS);
}

/**********************************************************************
 * AGC: BINROUND, 2ROUND, TESTOFUF
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 350, 352)
 *
 * BINROUND / 2ROUND / TESTOFUF: round MPAC+1 into MPAC; the SP result in
 * the high word of *out. Overflow alarms and recycles.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   dp        int64_t     I    DP value at 2^-28 (MPAC, MPAC+1)
 *   out       agc_dp_t *  O    the rounded SP in the high word (SP scale
 *                              of the component)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_binround(int64_t dp, agc_dp_t *out)
{
    if (!AGC_ASSERT(out != NULL) || !AGC_ASSERT(pin_abs64(dp) < PIN_DP_LIM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int64_t mag = pin_abs64(dp); /* magnitude, unscaled */
    int64_t hi; /* MPAC after rounding MPAC+1 */

    hi = (mag >> AGC_SP_BITS) + (((mag & 037777) >= 020000) ? 1 : 0);

    if (hi >= PIN_SP_LIM)
    {
        return (pin_almcycle()); /* OF: alarm and recycle  */
    }

    *out = agc_dp_from_sp((agc_sp_t) ((dp < 0) ? -hi : hi));

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: ARTHINSF (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 350)
 *
 * ARTHINSF: assumes point between hi and lo parts of SFCON; shifts the
 * result left by 14 (taking it from MPAC+1, MPAC+2). MPAC must be zero:
 * else too large a load, alarm and recycle.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   f         agc_dp_t    I    decimal fraction N/10^5 at 2^-28 (DP)
 *   sfin      agc_dp_t    I    SF input constant from SFINTAB (DP, scale
 *                              per routine)
 *   out       agc_dp_t *  I/O  value to store, in the scale of the
 *                              component
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_arthinsf(agc_dp_t f, agc_dp_t sfin, agc_dp_t *out)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int64_t tp = pin_dmp_tp(f, sfin); /* TP product, 2^-42 units */

    if ((tp >> AGC_DP_BITS) != 0)
    {
        return (pin_almcycle());
    }

    int64_t dp = tp & (PIN_DP_LIM - 1); /* DP part, 2^-28 units */

    return (pin_binround((f < 0) ? -dp : dp, out));
}

/**********************************************************************
 * AGC: ARTIN1SF (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 350)
 *
 * ARTIN1SF: scales MPAC, +1 by the SF constant and rounds MPAC+1 into MPAC.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   f         agc_dp_t    I    decimal fraction N/10^5 at 2^-28 (DP)
 *   sfin      agc_dp_t    I    SF input constant from SFINTAB (DP, scale
 *                              per routine)
 *   out       agc_dp_t *  I/O  value to store, in the scale of the
 *                              component
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_artin1sf(agc_dp_t f, agc_dp_t sfin, agc_dp_t *out)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int64_t dp = pin_dmp_tp(f, sfin) >> AGC_SP_BITS; /* DP part, 2^-28 units */

    if (!AGC_ASSERT(dp < PIN_DP_LIM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (pin_binround((f < 0) ? -dp : dp, out));
}

/**********************************************************************
 * AGC: DPINSF, DPINSF2 (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 351)
 *
 * DPINSF (DP1, DP3), DPINSF2 (DP2: point between bits 7-8 of the high part
 * of the SF constant, shift left 7), DPINSF4 (DP4: bits 11-12, shift left
 * 3): round MPAC+2 into MPAC+1; the low part goes to E+1 (DPFRACIN).
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   f         agc_dp_t    I    decimal fraction N/10^5 at 2^-28 (DP)
 *   sfin      agc_dp_t    I    SF input constant from SFINTAB (DP, scale
 *                              per routine)
 *   shift     int         I    left shift count after the multiply
 *   out       agc_dp_t *  I/O  value to store, in the scale of the
 *                              component
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_dpinsf(agc_dp_t f, agc_dp_t sfin, int shift,
                               agc_dp_t *out)
{
    if (!AGC_ASSERT(shift == 0 || shift == 3 || shift == 7) ||
        !AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int64_t tp = pin_dmp_tp(f, sfin) << shift; /* TPLEFTN    */
    int64_t dp; /* DP part, 2^-28 units */

    dp = (tp >> AGC_SP_BITS) + (((tp & 037777) >= 020000) ? 1 : 0);

    if ((tp >> AGC_TP_BITS) != 0 || dp >= PIN_DP_LIM)
    {
        return (pin_almcycle()); /* TESTOFUF   */
    }

    *out = (agc_dp_t) ((f < 0) ? -dp : dp);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: SIGNFIX, SGNTO1, NEG180
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 349-350)
 *
 * Two's complement 15-bit angle word of +-m (SIGNFIX, SGNTO1, NEG180):
 * after the last shift an overflow forces the sign bit to 1, an underflow
 * to 0, and -0 with underflow is -180 degrees.
 *
 * ARGUMENTS:
 *   Argument  Type     I/O  Description (scale)
 *   --------  -------  ---  -------------------
 *   neg       bool     I    true for a negative angle
 *   m         int64_t  I    angle magnitude, 15-bit units (2^-15 rev)
 *
 * RETURNS: uint16_t -- two's complement angle word (0 on a failed
 *          precondition)
 **********************************************************************/
static uint16_t pin_angle_word(bool neg, int64_t m)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (0); /* no valid result */
    }

    if (!AGC_ASSERT(m >= 0 && m < 0100000))
    {
        return (0); /* no valid result */
    }

    int64_t w = neg ? (0100000 - m) : m; /* angle word, two's complement */

    return ((uint16_t) (w & PIN_WORD));
}

/**********************************************************************
 * AGC: DEGINSF (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 349)
 *
 * DEGINSF: SF routine for decimal degrees (xxx.xx). Multiply by 5.555 B-3,
 * round MPAC+1 by BIT11 away from zero before shifting left 3 -- this
 * causes 360.00 to overflow when shifted left and alarm -- and leave the
 * CDU angle (15 bits, two's complement) in MPAC.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   f         agc_dp_t    I    decimal fraction N/10^5 at 2^-28 (DP)
 *   cdu360    bool        I    true for the LM 360-CDU scale (ENDSCALE
 *                              fix-up)
 *   out       agc_dp_t *  O    CDU angle word (bit 15 = 180 deg)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_deginsf(agc_dp_t f, bool cdu360, agc_dp_t *out)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int64_t d = pin_dmp_tp(f, PIN_DEGCON1) >> AGC_SP_BITS; /* MPAC, MPAC+1 */

    if ((d & 037777) != 0)
    {
        d += 02000; /* +-BIT11    */
    }

    if (d >= PIN_DP_LIM || (d >> 12) >= PIN_SP_LIM)
    {
        return (pin_almcycle()); /* OF/UF after 2 shifts       */
    }

    uint16_t w = pin_angle_word(f < 0, d >> 11); /* left 3     */

    if (cdu360)
    {
        w = pin_360_cdu(w); /* LM ENDSCALE: 360 - CDU deg */
    }

    *out = (agc_dp_t) w;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: OPTDEGIN (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 350)
 *
 * OPTDEGIN (CM): optics scaling routine, range 90 degrees. Reject a
 * negative input; subtract the bias, multiply by 100/45 B-2, round as in
 * DEGINSF (by BIT12, two shifts in DEGINSF2).
 * DEVIATION: the rounding constant is added to the signed DP; when the
 * bias makes the words disagree in sign the AGC's per-word shifts may
 * differ from this by one LSB.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   f         agc_dp_t    I    decimal fraction N/10^5 at 2^-28 (DP)
 *   negzero   bool        I    true when -0 was keyed
 *   out       agc_dp_t *  O    optics angle word (bit 15 = 45 deg)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_optdegin(agc_dp_t f, bool negzero, agc_dp_t *out)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(out != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (f < 0 || negzero)
    {
        return (pin_almcycle()); /* reject - input         */
    }

    agc_dp_t fb = f - (agc_dp_t) (PIN_NEG_2 * PIN_SP_LIM); /* ADS MPAC   */
    int64_t d = pin_dmp_tp(fb, PIN_DEGCON2) >> AGC_SP_BITS; /* 2^-14 */
    int64_t v = ((fb < 0) ? -d : d) + 04000; /* + BIT12    */
    int64_t m = pin_abs64(v); /* magnitude */

    if ((m >> 13) >= PIN_SP_LIM)
    {
        return (pin_almcycle());
    }

    *out = (agc_dp_t) pin_angle_word(v < 0, m >> 12);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: SFINTABR (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 349)
 *
 * SFINTABR fan, part 2 (routines that need the SF constant).
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description (scale)
 *   --------  ------------------------  ---  -------------------
 *   rout      uint8_t                   I    SF routine number
 *                                            (agc_sf_routine_t)
 *   info      const agc_scale_info_t *  I    scale of the component (SF
 *                                            routine, constants, raw/DP
 *                                            flags)
 *   f         agc_dp_t                  I    decimal fraction N/10^5 at
 *                                            2^-28 (DP)
 *   out       agc_dp_t *                I/O  value to store, in the scale
 *                                            of the component
 *
 * RETURNS: agc_status_t -- BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
static agc_status_t pin_sfin_const(uint8_t rout, const agc_scale_info_t *info,
                                   agc_dp_t f, agc_dp_t *out)
{
    switch (rout)
    {
        case PIN_SF_ARITH:
            return (pin_arthinsf(f, info->sf_in, out));
        case PIN_SF_ARITH1:
            return (pin_artin1sf(f, info->sf_in, out));
        case PIN_SF_DP1:
        case PIN_SF_DP3:
            return (pin_dpinsf(f, info->sf_in, 0, out)); /* DPINSF  */
        case PIN_SF_DP2:
            return (pin_dpinsf(f, info->sf_in, 7, out)); /* DPINSF2 */
        case PIN_SF_DP4:
            return (pin_dpinsf(f, info->sf_in, 3, out)); /* DPINSF4 */
        default:

            if (!AGC_ASSERT(rout < (uint8_t) PIN_SF_COUNT))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            return (AGC_STATUS_BAD_ARG);
    }
}

/**********************************************************************
 * AGC: PUTDECSF, SFINTABR
 *      (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM pp. 348-349)
 *
 * PUTDECSF: the SFINTABR fan for component k; the decimal fraction becomes
 * the value to store.
 *
 * ARGUMENTS:
 *   Argument  Type        I/O  Description (scale)
 *   --------  ----------  ---  -------------------
 *   k         int         I    noun component 0..2 (R1..R3)
 *   frac      agc_dp_t    I    decimal fraction N/10^5 at 2^-28 (DP)
 *   negzero   bool        I    true when -00000 was keyed
 *   out       agc_dp_t *  O    value to store (convention of the
 *                              component)
 *   exited    bool *      O    true when the routine left the verb itself
 *                              (alarm, ENTEXIT)
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_putdecsf(int k, agc_dp_t frac, bool negzero, agc_dp_t *out,
                          bool *exited)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_scale_info_t info; /* scale of the component */
    agc_status_t     st = pin_comp_scale(k, &info); /* status so far */

    if (!AGC_ASSERT(out != NULL && exited != NULL) || st != AGC_STATUS_OK)
    {
        return ((st != AGC_STATUS_OK) ? st : AGC_STATUS_BAD_ARG);
    }

    bool lm = agc_state_vehicle() == AGC_VEHICLE_LM; /* true on the LM */

    *exited = true;

    switch (info.routine)
    {
        case PIN_SF_OCTAL:
            /* alarm and recycle if dec load with octal-only noun */
            return (pin_almcycle());
        case PIN_SF_MS:            /* min/sec can't be loaded            */
        case PIN_SF_2INT: /* 2INTOUT can't be loaded            */
        case PIN_SF_HMS:  /* (HMSIN is entered by the caller)   */
            return (pin_dspalarm());
        case PIN_SF_OPDEG: /* LM: LRPOSOUT can't be loaded       */

            if (lm)
            {
                return (pin_dspalarm());
            }

            st = pin_optdegin(frac, negzero, out);
            break;
        case PIN_SF_FRACT:
            st = pin_binround(frac, out);
            break;
        case PIN_SF_DEG:
            st = pin_deginsf(frac, false, out);
            break;
        case PIN_SF_DPFRAC: /* CM: DPFRACIN.  LM: DEGINSF (360-CDU) */
            st = lm ? pin_deginsf(frac, true, out) : AGC_STATUS_OK;

            if (!lm)
            {
                *out = frac;
            }

            break;
        default:
            st = pin_sfin_const(info.routine, &info, frac, out);
            break;
    }

    *exited = (st != AGC_STATUS_OK);

    if (st == AGC_STATUS_OK && !pin_sf_is_dp(info.routine))
    {
        /* the SP result (MPAC) in the value convention of the data id */
        bool angle = info.routine == (uint8_t) PIN_SF_DEG ||
                     info.routine == (uint8_t) PIN_SF_OPDEG ||
                     /* a CDU angle routine */
                     info.routine == (uint8_t) PIN_SF_DPFRAC;
        uint16_t w; /* angle word, two's complement */

        w = angle ? (uint16_t) *out : pin_ones_from_sp(agc_dp_high(*out));

        *out = (lm || info.routine != (uint8_t) PIN_SF_DPFRAC) ?
               pin_value_of_word(&info, w) :
               *out;
    }

    return (st);
}

/* ---- HMSIN (pp. 353-354) ---------------------------------------------- */

/**********************************************************************
 * AGC: RND/TST (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 353)
 *
 * RND/TST: whole number from the fraction N/10^5 (DMP by WHOLECON, round
 * MPAC+2 into MPAC+1); alarm if MPAC non-zero (G/ 16383).
 *
 * ARGUMENTS:
 *   Argument  Type       I/O  Description (scale)
 *   --------  ---------  ---  -------------------
 *   f         agc_dp_t   I    decimal fraction N/10^5 at 2^-28 (DP)
 *   n         int64_t *  O    the whole number
 *
 * RETURNS: bool -- true when the number fits (no overflow); false alarms
 **********************************************************************/
static bool pin_whole(agc_dp_t f, int64_t *n)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (false);
    }

    int64_t w; /* angle word, two's complement */

    w = (pin_abs64(f) * PIN_WHOLECON + (PIN_DP_LIM >> 1)) >> AGC_DP_BITS;

    if (!AGC_ASSERT(n != NULL))
    {
        return (false);
    }

    *n = (f < 0) ? -w : w;

    return (w < PIN_SP_LIM);
}

/**********************************************************************
 * AGC: HMSIN (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, CM p. 352)
 *
 * HMSIN: XREG = hours, YREG = minutes, ZREG = seconds (xxx.xx), all three
 * loaded in decimal; the sum in centiseconds goes to the noun's DP
 * erasable. G/ 745 hr, 39 min, 14.59 sec alarms (DAS overflow).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- OK, BAD_ARG on a failed precondition; else the
 *          status of the routine it calls
 **********************************************************************/
agc_status_t pin_hmsin(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    pin_state_t *s = pin_st(); /* pinball erasables */

    if ((s->decbrnch & PIN_DEC345) != PIN_DEC345)
    {                     /* ALL3DEC    */
        s->verbsave = 25; /* FORCEV25: force verb 25 by recycle */

        return (pin_almcycle());
    }

    int64_t h   = 0; /* hours keyed */
    int64_t m   = 0; /* minutes keyed */
    int64_t sec = 0; /* seconds keyed, cs */
    bool    ok  =
        pin_whole(s->xyz_frac[0], &h) && pin_whole(s->xyz_frac[1], &m) &&
        pin_whole(s->xyz_frac[2], &sec); /* all three fit */

    ok = ok && pin_abs64(h) * PIN_CS_HOUR < PIN_DP_LIM && /* MPACTST: G/ 745 */
         pin_abs64(m) <= PIN_59MIN &&                  /* SIZETST         */
         pin_abs64(sec) <= PIN_5999CS;
    int64_t t = h * PIN_CS_HOUR + m * PIN_CS_MIN; /* DAS HITEMIN     */

    ok = ok && pin_abs64(t) < PIN_DP_LIM;
    t += sec;
    ok = ok && pin_abs64(t) < PIN_DP_LIM;

    if (!ok)
    {
        return (pin_almcycle());
    }

    if (!AGC_ASSERT(pin_comp_routine(0) == (uint8_t) PIN_SF_HMS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = pin_comp_write(0, (agc_dp_t) t); /* TPAGREE, DXCH */

    return ((st == AGC_STATUS_OK) ? agc_dsky_loadlv() : st);
}
