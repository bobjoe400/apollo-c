/*
 * display.c -- PINBALL word display routines (CM pp. 359-363,
 * LM pp. 441-445): DSPSIGN, DSPRND, DSPDECWD, DSPDECNR, DSPDC2NR, DSP2DEC,
 * DSPDECVN, DSPOCTWO, DSP2BIT, +ON/-ON, UPDATVB, UPDATNN, DSPMMJB.
 *
 * The AGC converts the DP fraction in MPAC, MPAC+1 to decimal by repeated
 * SHORTMP by BINCON (10): the integer part that spills into MPAC is the next
 * digit and MPAC+1, MPAC+2 the remaining fraction. With the fraction held as
 * an agc_dp_t (units 2^-28) this is exactly digit = (f*10) >> 28,
 * f = (f*10) mod 2^28 -- display conversion, not flight arithmetic.
 */
#include <stddef.h>

#include "internal.h"

#define PIN_DP_ONE   ((int64_t) 1 << AGC_DP_BITS) /* 2^28                  */
#define PIN_DECROUND ((agc_dp_t) 02476)           /* 5 E-6, (0, 02476)      */
#define PIN_VNDSPCON 0244                         /* .01 rounded up, 2^-14  */
#define PIN_BINCON   10

/* SGNTAB (p. 320): DSPTAB rel of the minus relay of R1, R2, R3; the plus
 * relay is the next word up.                                              */
static const int k_sgntab[3] = {5, 3, 0};

/* +ON / -ON (pp. 319-320), common part SGNCOM. */
static agc_status_t pin_sign_on(bool plus)
{
    int inrel = pin_getinrel();

    if (!AGC_ASSERT(inrel >= PIN_INREL_X) || !AGC_ASSERT(inrel <= PIN_INREL_Z))
    {
        return AGC_STATUS_BAD_ARG;
    }

    int          minus_rel = k_sgntab[inrel - PIN_INREL_X];
    int          off       = plus ? minus_rel : minus_rel + 1; /* SGNOFF               */
    int          on        = plus ? minus_rel + 1 : minus_rel; /* SGNON                */
    agc_status_t st        = pin_11dspin(off, false);

    if (st == AGC_STATUS_OK)
    {
        st = pin_11dspin(on, true);
    }

    return st;
}

agc_status_t pin_plus_on(void)
{
    return pin_sign_on(true);
}

agc_status_t pin_minus_on(void)
{
    return pin_sign_on(false);
}

/* DSPSIGN (p. 359): + or +0 lights +, - or -0 lights -; leaves |MPAC|. */
static agc_status_t pin_dspsign(agc_dp_t *v, bool negzero)
{
    if (!AGC_ASSERT(v != NULL) ||
        !AGC_ASSERT(*v >= AGC_DP_MIN && *v <= AGC_DP_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (*v < 0 || (*v == 0 && negzero))
    {
        *v = -*v;

        return pin_minus_on();
    }

    return pin_plus_on();
}

/* DSPRND (p. 359): round by 5 E-6; on overflow force DPOSMAX. */
static agc_dp_t pin_dsprnd(agc_dp_t v)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return 0; /* no valid result */
    }

    if (!AGC_ASSERT(v >= 0))
    {
        return 0; /* no valid result */
    }

    int64_t s = (int64_t) v + PIN_DECROUND;

    if (s > (int64_t) AGC_DP_MAX)
    {
        return AGC_DP_MAX;
    }

    return (agc_dp_t) s;
}

/* DSPDCWD1 / TRACE1 (pp. 359-360): ndig decimal chars of the fraction v,
 * starting at DSPCOUNT and going down; then block DSPCOUNT (CS VD1).     */
static agc_status_t pin_dspdcwd1(agc_dp_t v, int ndig)
{
    if (!AGC_ASSERT(v >= 0) || !AGC_ASSERT(ndig >= 1 && ndig <= 10))
    {
        return AGC_STATUS_BAD_ARG;
    }

    pin_state_t *s  = pin_st();
    int64_t      f  = v;
    agc_status_t st = AGC_STATUS_OK;

    for (int i = 0; i < ndig && st == AGC_STATUS_OK; i++)
    {
        f *= PIN_BINCON; /* SHORTMP by BINCON    */
        int digit = (int) (f >> AGC_DP_BITS);

        f &= PIN_DP_ONE - 1;
        int16_t count = s->dspcount;

        s->dspcount = (count > 0) ? (int16_t) (count - 1) : count;
        st          = pin_dspin(count, agc_dsky_relay_code(digit));
    }

    pin_block_dspcount(PIN_VD1);

    return st;
}

/* DSPDECWD (p. 359): sign and 5 decimal chars, rounded by 5 E-6. */
agc_status_t pin_dspdecwd(agc_dp_t v, bool negzero)
{
    if (!AGC_ASSERT(v >= AGC_DP_MIN) || !AGC_ASSERT(v <= AGC_DP_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = pin_dspsign(&v, negzero);

    if (st != AGC_STATUS_OK)
    {
        return st;
    }

    return pin_dspdcwd1(pin_dsprnd(v), 5);
}

/* DSPDECNR (p. 360): as DSPDECWD without rounding. */
agc_status_t pin_dspdecnr(agc_dp_t v, bool negzero)
{
    if (!AGC_ASSERT(v >= AGC_DP_MIN) || !AGC_ASSERT(v <= AGC_DP_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = pin_dspsign(&v, negzero);

    return (st == AGC_STATUS_OK) ? pin_dspdcwd1(v, 5) : st;
}

/* DSPDC2NR (p. 360): sign and 2 decimal chars, no rounding. */
agc_status_t pin_dspdc2nr(agc_dp_t v, bool negzero)
{
    if (!AGC_ASSERT(v >= AGC_DP_MIN) || !AGC_ASSERT(v <= AGC_DP_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = pin_dspsign(&v, negzero);

    return (st == AGC_STATUS_OK) ? pin_dspdcwd1(v, 2) : st;
}

/* DSP2DEC (p. 360): sign in R1 and 10 decimal chars in R1, R2 (V07). */
agc_status_t pin_dsp2dec(agc_dp_t v)
{
    if (!AGC_ASSERT(v >= AGC_DP_MIN) || !AGC_ASSERT(v <= AGC_DP_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_status_t st = pin_11dspin(3, false); /* -R2 off              */

    if (st == AGC_STATUS_OK)
    {
        st = pin_11dspin(4, false); /* +R2 off              */
    }

    if (st == AGC_STATUS_OK)
    {
        st = pin_dspsign(&v, false);
    }

    return (st == AGC_STATUS_OK) ? pin_dspdcwd1(v, 10) : st;
}

/* DSPDECVN (p. 361): C(A) = N x 2^-14 shown as 2 decimal chars (verb,
 * noun, program): MP by .01 (VNDSPCON, rounded up) and take L.          */
agc_status_t pin_dspdecvn(int n)
{
    if (!AGC_ASSERT(n >= 0) || !AGC_ASSERT(n <= (int) AGC_SP_MAX))
    {
        return AGC_STATUS_BAD_ARG;
    }

    int32_t lword =
        (int32_t) (((int32_t) n * PIN_VNDSPCON) & (int32_t) PIN_POSMAX);
    agc_dp_t f = (agc_dp_t) (lword << AGC_SP_BITS); /* MPAC = L, MPAC+1 = 0 */

    return pin_dspdcwd1(f, 2);                      /* DSPDC2NR +3          */
}

/* DSPOCTWO (p. 361): C(A) as 5 octal chars from DSPCOUNT; BIT14 added to
 * DSPCOUNT so each left char also blanks its sign.                      */
agc_status_t pin_dspoctwo(uint16_t w)
{
    if (!AGC_ASSERT(w <= PIN_WORD) ||
        !AGC_ASSERT(pin_st()->dspcount <= PIN_MD1))
    {
        return AGC_STATUS_BAD_ARG;
    }

    pin_state_t *s = pin_st();

    s->dspcount = (int16_t) ((uint16_t) s->dspcount | PIN_BIT14);
    agc_status_t st = AGC_STATUS_OK;

    for (int i = 4; i >= 0 && st == AGC_STATUS_OK; i--)
    {                                                /* WDAGAIN      */
        int     digit = (int) ((w >> (3 * i)) & 7u); /* CYL x3, DSPMSK */
        int16_t count = s->dspcount;

        s->dspcount = ((count & 037) > 0) ? (int16_t) (count - 1) : count;
        st          = pin_dspin(count, agc_dsky_relay_code(digit));
    }

    pin_block_dspcount(PIN_VD1); /* DSPLV        */

    return st;
}

/* DSP2BIT (p. 362): C(A) as 2 octal chars (low 6 bits) from DSPCOUNT. */
agc_status_t pin_dsp2bit(uint16_t w)
{
    if (!AGC_ASSERT(w <= PIN_WORD) ||
        !AGC_ASSERT(pin_st()->dspcount <= PIN_MD1))
    {
        return AGC_STATUS_BAD_ARG;
    }

    pin_state_t *s  = pin_st();
    agc_status_t st = AGC_STATUS_OK;

    for (int i = 1; i >= 0 && st == AGC_STATUS_OK; i--)
    {
        int16_t count = s->dspcount;

        s->dspcount = (count > 0) ? (int16_t) (count - 1) : count;
        st          =
            pin_dspin(count, agc_dsky_relay_code((int) ((w >> (3 * i)) & 7u)));
    }

    pin_block_dspcount(PIN_VD1);

    return st;
}

/* UPDATVB -1 (pp. 328-329): verb into VERBREG and the verb lights. */
agc_status_t pin_updatvb(uint8_t verb)
{
    if (!AGC_ASSERT(verb <= 99) || !AGC_ASSERT(pin_st()->dspcount <= PIN_MD1))
    {
        return AGC_STATUS_BAD_ARG;
    }

    pin_st()->verbreg = verb;
    pin_set_dspcount(PIN_VD1);

    return pin_dspdecvn(verb); /* GOVNUPDT             */
}

/* UPDATNN -1 (p. 328): noun into NOUNREG, reread the noun tables (PUTADD)
 * and show it in the noun lights.                                        */
agc_status_t pin_updatnn(uint8_t noun)
{
    if (!AGC_ASSERT(noun <= 99) || !AGC_ASSERT(pin_st()->dspcount <= PIN_MD1))
    {
        return AGC_STATUS_BAD_ARG;
    }

    pin_st()->nounreg = noun;
    agc_status_t st = pin_noun_load();

    if (st != AGC_STATUS_OK && st != AGC_STATUS_NOT_FOUND)
    {
        return st;
    }

    pin_set_dspcount(PIN_ND1);

    return pin_dspdecvn(noun);
}

/* DSPMMJB (p. 376): MODREG into the program lights; DSPCOUNT preserved.
 * DEVIATION: the AGC blanks for MODREG = -0 and leaves the lights for
 * -NZ; agc_sp_t has no -0, so any negative MODREG blanks.               */
agc_status_t agc_job_dspmmjb(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(pin_st()->dspcount <= PIN_MD1))
    {
        return AGC_STATUS_BAD_ARG;
    }

    pin_state_t *s        = pin_st();
    int16_t      save     = s->dspcount; /* DSPMMTEM             */
    bool         save_neg = s->dspcount_neg;
    agc_sp_t     mm       = agc_erase_modreg();

    pin_set_dspcount(PIN_MD1);
    agc_status_t st = (mm >= 0) ? pin_dspdecvn(mm) : pin_2blank();

    s->dspcount     = save;
    s->dspcount_neg = save_neg;

    return st;
}
