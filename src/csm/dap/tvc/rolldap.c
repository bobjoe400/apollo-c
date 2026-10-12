/**********************************************************************
 * FILE NAME: rolldap.c
 *
 * PURPOSE:
 *   The TVC roll autopilot (an adaptation of the LM P-axis controller):
 *   keeps the outer gimbal angle OGA within a 5 deg deadband of OGAD and
 *   the OGA rate below the 0.1 deg/s limit-cycle rate with phase-plane
 *   switching logic (GSOP chapter 3), timing the roll jet firings with
 *   T6. Labels: ROLLDAP, DURATION, ROLLOGIC, DELOGAC, REG1TST .. REG5TST,
 *   REGION6, ROLLON, ONROLL, RATEDEN, RATENUM, MINLIMAP, DVOK, RATELIM,
 *   ROLLSET, MOREROLL, NEWROLL, MINTST, MAXTFIRE, NOMXFIRE, FIRELOOK,
 *   FIREPLUG, NOROLL, JETROLL, +TORQUE, -TORQUE, T6ENABL, NOROLL1.
 *
 * AGC SOURCE:
 *   Comanche055 TVCROLLDAP.agc pp. 984-998 (CSM only)
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   OGANOW,       uint16_t  I    2^15 = 1 rev   outer gimbal now and
 *   OGAPAST                                     1/2 s ago (raw CDUX)
 *   OGA (OGAERR)  agc_sp_t  I    B+0 rev        fly-from OGA error
 *   1/CONACC      agc_sp_t  I    B+9 s^2/rev    1 / roll acceleration
 *   TIME6         uint16_t  I/O  1/1600 s       T6 counter (raw word)
 *   CHAN6         uint16_t  O    bits           roll jets
 *   CHAN13        uint16_t  O    bit 15         T6RUPT enable
 *   TVC block     struct    I/O  per field      agc_csm_dap_tvc()
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   csm_dap_ads, _mp_hi,  word arithmetic in the AGC's manner
 *   _mp_lo, _dv, _msu     (mysubs.c)
 *   agc_rupt_set_t6_task  T6LOC = NOROLL1
 *   agc_rupt_set_time     TIME6
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none in the AGC. AGC_STATUS_BAD_ARG when an assertion on ROLLFIRE,
 *   1/CONACC or register I fails.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   ROLLDAP is a waitlist task from TVCEXEC every 1/2 s (3 cs delay);
 *   NOROLL1 is a T6 task. Firings: at most 2.5 s (2.56 s before the
 *   limit applies), at least 15 ms; the jet pairs alternate; a pair is
 *   off at least 1/2 s before a new one fires; a firing may be shortened
 *   but never extended on a later pass. CSM only.
 *
 * NOTES:
 *   OGARATE is B-4 rev/s; TEMREG holds the firing time at 625 us/bit (16
 *   bits/cs). The AGC builds CA or CS instruction words in register I
 *   and executes them with INDEX I; 0 X (a run-time sign selector); here
 *   I is +1 (CA), -1 (CS) or 0 (MOREROLL: keep the same jets).
 *   ROLLSET doubles the product twice in the 16-bit accumulator; a
 *   product of 1/2 or more wraps it and the overflow limit then takes
 *   the opposite sign. That is modelled (roll_fire_time), not corrected.
 *   NOROLL's -0 (ROLLFIRE, TEMREG, TIME6) is +0 here except for the raw
 *   TIME6 word, which is written as 77777 (an immediate T6RUPT).
 *
 * ALGORITHM:
 *   1. OGARATE from the 1/2-s OGA difference. If the jets are off but
 *      fired last pass, wait one more pass (TEMREG = 0).
 *   2. Locate (OGAERR, OGARATE) in the phase plane: coast (regions 2, 4,
 *      5), rate too high (region 3: -+MAXLIM), or fire towards the
 *      intersection of the tangent to the control parabola with the
 *      straight switch line (regions 1, 6).
 *   3. Firing time = (desired - present rate) T6SCALE 1/CONACC x 4.
 *   4. Same polarity as now: keep the jets, shorten only; otherwise a
 *      new firing of at least TMINFIRE on the alternate jet pair.
 **********************************************************************/
#include "../internal.h"

#define DB          ((agc_sp_t) 228)    /* 00344 DEC .01388889: 5 deg, B+0 rev */
#define NEG_SLOPE   ((agc_sp_t) 3277)   /* 06315 DEC 0.2: -switch slope, B+0/s */
#define LMCRATE     ((agc_sp_t) 73)     /* 00111 .00027778 B+4: 0.1 deg/s, B-4  */
#define INTERCEP    ((agc_sp_t) 328)    /* 00510 .0025 B+3: DB(-SLOPE)-LMCRATE  */
#define MINLIM      ((agc_sp_t) 728)    /* 01330 .00277778 B+4: 1 deg/s, B-4    */
#define MAXLIM      ((agc_sp_t) 3641)   /* 07071 .01388889 B+4: 5 deg/s, B-4    */
#define TMINFIRE    ((agc_sp_t) 24)     /* 00030 DEC 1.5 B+4: 15 ms, 16 bits/cs */
#define TMAXFIRE    ((agc_sp_t) 4000)   /* 07640 DEC 250 B+4: 2.5 s, 16 bits/cs */
#define RECIP_TMX   ((agc_sp_t) 4)      /* 1/TMXFIR = BIT3 (allows 2.56 s)     */
#define T6SCALE     ((agc_sp_t) 12800)  /* PRIO31 31000: (B+3)(16/cs)(100 cs/s) */
#define BIT5_SP     ((agc_sp_t) 16)     /* BIT5: OGA difference to B-4 rev/s  */
#define BIT11_SP    ((agc_sp_t) 1024)   /* BIT11, unscaled                    */
#define BIT14_SP    ((agc_sp_t) 8192)   /* BIT14: B-4 -> B-3 rev/s            */
#define A16_MAX     32767               /* 16-bit accumulator, 2^-14 units     */
#define A16_MOD     65535               /* its ones' complement modulus        */
#define PLUS_ROLL1  ((uint16_t) 000005) /* FIVE: jets 9 and 11               */
#define PLUS_ROLL2  ((uint16_t) 000120) /* OCT120: jets 13 and 15            */
#define MINUS_ROLL1 ((uint16_t) 000012) /* TEN: jets 12 and 10               */
#define MINUS_ROLL2 ((uint16_t) 000240) /* OCT 240: jets 16 and 14           */

/**********************************************************************
 * INDEX I; 0 X (TVCROLLDAP.agc pp. 991-994) -- the CA or CS instruction
 * that register I holds, applied to a word.
 *
 * ARGUMENTS:
 *   i  int       I  +1 (CA) or -1 (CS)
 *   v  agc_sp_t  I  the operand, any scale
 *
 * RETURNS: agc_sp_t -- v or -v (same scale)
 **********************************************************************/
static agc_sp_t sgn_mul(int i, agc_sp_t v)
{
    agc_sp_t r = v; /* CA: v unchanged, scale of v                      */

    if (i < 0)
    {
        r = (agc_sp_t) -v; /* CS                                       */
    }

    return (r);
}

/**********************************************************************
 * ONROLL, RATEDEN, RATENUM, MINLIMAP, DVOK (TVCROLLDAP.agc pp. 993-994)
 * -- the desired OGA rate at the intersection of the tangent to the
 * control parabola with the straight switch line, DOGADOT = NUM / DEN,
 * limited to +-POSMAX when NUM / DEN overflows. Leaves DEN in TEMREG
 * and 2(-SLOPE)(RATE^2)/CONACC + (-SLOPE)(I DELOGA - DB) in DELOGART.
 *
 * ARGUMENTS:
 *   deloga    agc_sp_t  I  OGA error of the operating point, B+0 rev
 *   delogart  agc_sp_t  I  OGA rate of the operating point, B-4 rev/s
 *   i         int       I  register I: +1 (CA) or -1 (CS)
 *
 * RETURNS: agc_sp_t -- desired OGA rate, B-4 rev/s
 **********************************************************************/
static agc_sp_t roll_desired_rate(agc_sp_t deloga, agc_sp_t delogart, int i)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    /* MP 1/CONACC; DOUBLE; MP -SLOPE: TEMREG = 2(-SLOPE)RATE/CONACC      */
    agc_sp_t a   = csm_dap_mp_hi(delogart, t->one_conacc);      /* scale B+5 rev^-1 s */
    agc_sp_t den = csm_dap_mp_hi(csm_dap_ads(a, a), NEG_SLOPE); /* TEMREG, scale B+6 rev^-1 s */
    agc_sp_t num = csm_dap_mp_hi(den, delogart);                /* 2(-S)(RATE^2)/CONACC, scale B+2 rev */

    den        = csm_dap_ads(den, sgn_mul(i, (agc_sp_t) -BIT11_SP)); /* RATEDEN */
    agc_sp_t e = csm_dap_ads(sgn_mul(i, deloga), (agc_sp_t) -DB);    /* scale B+0 rev */

    num         = csm_dap_ads(num, csm_dap_mp_hi(e, NEG_SLOPE)); /* ADS  */
    t->temreg   = den;
    t->delogart = num;
    num         = csm_dap_ads(csm_dap_mp_hi(LMCRATE, BIT11_SP), num);

    if (den == 0)
    {
        /* DV by a zero TEMREG: every quotient bit is set, so the first DV
         * overflows (MINLIMAP). DEVIATION: the AGC's TEMREG is then
         * usually -0 (ADS of opposites) and the overflow sign follows
         * the signs of L and TEMREG; -0 is not represented, so the sign
         * is that of the numerator.                                      */
        return ((num >= 0) ? AGC_SP_MAX : AGC_SP_MIN); /* ?: MINLIMAP     */
    }

    /* XCH L; CA ZERO; DV TEMREG: (0, NUM) / DEN is 0 unless NUM / DEN
     * overflows                                                          */
    int q = (int) num / (int) den; /* quotient in A, 2^-14 units      */

    if (q > 0)
    {
        return (AGC_SP_MAX); /* MINLIMAP: positive overflow             */
    }

    if (q < 0)
    {
        return (AGC_SP_MIN); /* MINLIMAP: negative overflow             */
    }

    return (csm_dap_dv(agc_dp_pack(num, 0), den)); /* DVOK, B-4 rev/s   */
} /* roll_desired_rate */

/**********************************************************************
 * ROLLSET (TVCROLLDAP.agc p. 994) -- the jet firing time from the
 * desired minus the present rate:
 *   SU OGARATE; TS (overflow: INDEX A; CS LIMITS);
 *   MP T6SCALE; MP 1/CONACC; DDOUBL; DDOUBL; TS (overflow: CS LIMITS).
 * The DDOUBLs work in the 16-bit accumulator: a product of 1/2 or more
 * wraps it (ones' complement, end-around carry) and the overflow sign
 * the TS then sees is the wrapped one, as in the AGC.
 *
 * ARGUMENTS:
 *   desired  agc_sp_t  I  desired OGA rate, B-4 rev/s
 *
 * RETURNS: agc_sp_t -- firing time, 625 us/bit (16 bits/cs); + is
 *          positive roll torque
 **********************************************************************/
static agc_sp_t roll_fire_time(agc_sp_t desired)
{
    const agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables    */

    agc_sp_t d = csm_dap_ad_sat(desired, (agc_sp_t) -t->ogarate); /* B-4 */
    agc_sp_t a = csm_dap_mp_hi(d, T6SCALE);                       /* MP T6SCALE              */
    agc_dp_t p = agc_sp_mul(a, t->one_conacc);                    /* MP 1/CONACC: A,L, scale B+5 s */

    /* The low word's carries keep the doubled pair sign-agreed, so the
     * accumulator after DDOUBL; DDOUBL is the upper part of 4 p.         */
    int32_t acc = (int32_t) ((4 * (int64_t) p) / 16384); /* A, 2^-14     */

    if (acc > A16_MAX)
    {
        acc -= A16_MOD; /* second DDOUBL wraps the 16-bit A           */
    }
    else if (acc < -A16_MAX)
    {
        acc += A16_MOD;
    }
    else
    {
        /* A holds the doubled value */
    }

    if (acc > AGC_SP_MAX)
    {
        return (AGC_SP_MAX); /* TS: + overflow; INDEX A; CS LIMITS      */
    }

    if (acc < AGC_SP_MIN)
    {
        return (AGC_SP_MIN); /* TS: - overflow                          */
    }

    return ((agc_sp_t) acc); /* TS TEMREG                               */
}

/**********************************************************************
 * REG2TST .. REG5TST (TVCROLLDAP.agc pp. 991-992) -- the coast regions
 * for an operating point with I x OGARATE negative: the coast side of
 * the straight switch line (2), of MINLIM (4) and of the deadband (5).
 *
 * ARGUMENTS:
 *   i  int  I  register I: +1 (CA) or -1 (CS)
 *
 * RETURNS: bool -- true when the point is in region 2, 4 or 5 (NOROLL)
 **********************************************************************/
static bool roll_coast_region(int i)
{
    const agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables    */

    /* (-SLOPE) OGA x 8 (B-3 rev/s) + OGARATE x BIT14 (B-4 -> B-3)        */
    agc_dp_t s     = csm_dap_ddoubl(csm_dap_ddoubl(csm_dap_ddoubl(agc_sp_mul(t->ogaerr, NEG_SLOPE)))); /* scale B-3 rev/s */
    agc_sp_t x     = csm_dap_ads(agc_dp_high(s), csm_dap_mp_hi(t->ogarate, BIT14_SP));                 /* scale B-3 rev/s */
    bool     coast = (csm_dap_ads(sgn_mul(i, x), (agc_sp_t) -INTERCEP) <= 0);                          /* REG2 */

    coast = coast || (csm_dap_ads(t->iogarate, MINLIM) <= 0);                   /* REG4 */
    coast = coast || (csm_dap_ads(sgn_mul(i, t->ogaerr), (agc_sp_t) -DB) <= 0); /* 5 */

    return (coast);
}

/**********************************************************************
 * ROLLOGIC .. REG5TST, REGION6, ROLLON, RATELIM (TVCROLLDAP.agc
 * pp. 990-994) -- the phase-plane region of the operating point and the
 * desired OGA rate.
 *
 * ARGUMENTS:
 *   desired  agc_sp_t *  O  desired OGA rate, B-4 rev/s (when true)
 *
 * RETURNS: bool -- false for a coast region (NOROLL), true to fire
 **********************************************************************/
static bool roll_regions(agc_sp_t *desired)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */
    agc_sp_t       r = 0;                 /* OGARATE, B-4 rev/s         */

    r = t->ogarate;

    /* TEMREG = DB - (1/2 CONACC) OGARATE^2, the 1/2 in the scaling       */
    agc_sp_t q = csm_dap_mp_hi(csm_dap_mp_hi((agc_sp_t) -r, t->one_conacc), r); /* scale B+0 rev */

    t->temreg = csm_dap_ads(q, DB);
    t->sgnrt  = (agc_sp_t) ((r > 0) ? 1 : -1); /* ?: sgn(0) negative     */

    /* DELOGA: distance from the switch parabola, + to its right          */
    t->deloga = csm_dap_ads(t->ogaerr, (agc_sp_t) -(t->sgnrt * t->temreg));
    int i     = (t->deloga > 0) ? 1 : -1; /* ?: CA (PRIO30) or CS (BIT15)  */

    t->i_sign = i;

    if (sgn_mul(i, t->sgnrt) > 0)
    { /* REG1TST: region 1, ROLLON */
        t->deloga   = csm_dap_ads(t->deloga, sgn_mul(i, DB));
        t->delogart = sgn_mul(i, (agc_sp_t) -LMCRATE); /* dummy point    */
        *desired    = roll_desired_rate(t->deloga, t->delogart, i);
    }
    else
    {
        t->iogarate = sgn_mul(i, r);

        if (csm_dap_ads(t->iogarate, MAXLIM) <= 0)
        { /* REG3TST: too fast */
            *desired = sgn_mul(i, (agc_sp_t) -MAXLIM); /* RATELIM           */
        }
        else
        {
            if (roll_coast_region(i))
            {
                return (false);
            }

            t->deloga   = t->ogaerr; /* REGION6: the actual state             */
            t->delogart = r;
            *desired    = roll_desired_rate(t->deloga, t->delogart, i);
        }
    }

    return (true);
} /* roll_regions */

/**********************************************************************
 * +TORQUE / -TORQUE (TVCROLLDAP.agc pp. 996-997) -- fire the jet pair
 * not used last time for this torque sign (ROLLWORD bit 1: +, bit 2: -).
 *
 * ARGUMENTS:
 *   plus  bool  I  true: +TORQUE (jets 9-11 / 13-15); false: -TORQUE
 *                  (jets 12-10 / 16-14)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          ROLLWORD
 **********************************************************************/
static agc_status_t roll_jets_on(bool plus)
{
    agc_csm_tvc_t *t    = agc_csm_dap_tvc(); /* TVC DAP erasables       */
    uint16_t       bit  = CSM_DAP_BIT2;      /* ROLLWORD bit of the sign */
    uint16_t       jets = MINUS_ROLL1;       /* channel 6 on-bits       */

    if (plus)
    {
        bit = CSM_DAP_BIT1;
    }

    if ((t->rollword & bit) == 0u)
    { /* NO.9-11, 12-10 */
        t->rollword = (uint16_t) (t->rollword | bit);  /* ADS ROLLWORD   */
        jets        = plus ? PLUS_ROLL1 : MINUS_ROLL1; /* ?: torque sign */
    }
    else
    { /* NO.13-15, NO.16-14 */
        t->rollword = (uint16_t) (t->rollword & ~(unsigned int) bit);
        jets        = plus ? PLUS_ROLL2 : MINUS_ROLL2; /* ?: torque sign */
    }

    if (!AGC_ASSERT(t->rollword <= 3u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_chan_write(AGC_CHAN_ROLLJETS, jets); /* WRITE CHAN6              */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * JETROLL, SAMEJETS, T6ENABL (TVCROLLDAP.agc pp. 996-997) -- NOROLL1 by
 * T6 in TEMREG counts; new jets unless MOREROLL (I = 0); T6RUPT enable.
 *
 * ARGUMENTS:
 *   time6  uint16_t  I  raw TIME6 word, 1/1600 s (77777 = -0 for NOROLL)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          register I or ROLLWORD
 **********************************************************************/
static agc_status_t roll_jetroll(uint16_t time6)
{
    agc_csm_tvc_t *t  = agc_csm_dap_tvc(); /* TVC DAP erasables         */
    agc_status_t   st = AGC_STATUS_OK;     /* jet selection status      */

    agc_rupt_set_t6_task(AGC_TASK_ROLL_JETS_OFF); /* T6LOC = NOROLL1    */
    agc_rupt_set_time(CSM_DAP_TIME6, time6);

    if (t->i_sign == 0)
    {
        return (AGC_STATUS_OK); /* SAMEJETS: keep the jets; TASKOVER   */
    }

    if (t->rollfire > 0)
    {
        st = roll_jets_on(true); /* +TORQUE                             */
    }
    else if (t->rollfire < 0)
    {
        st = roll_jets_on(false); /* -TORQUE                            */
    }
    else
    {
        /* +-0: no new jets, T6ENABL */
    }

    agc_chan_or(AGC_CHAN_CHAN13, (uint16_t) AGC_CH013_ENBT6RUPT); /* T6ENABL */

    return (st); /* RDAPEND: TASKOVER                                   */
}

/**********************************************************************
 * NOROLL (TVCROLLDAP.agc p. 996) -- coast: ROLLFIRE and TEMREG -0 (jets
 * may fire next pass), JETROLL with TIME6 = -0.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of JETROLL
 **********************************************************************/
static agc_status_t roll_noroll(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    t->rollfire = 0; /* CS ZERO: -0                                    */
    t->temreg   = 0; /* -0, 625 us/bit                                 */

    return (roll_jetroll(CSM_DAP_NEG0));
}

/**********************************************************************
 * ROLLDAP, DURATION, WAIT1/2 .. ROLLSET (TVCROLLDAP.agc pp. 989-994) --
 * waitlist task: OGARATE (the lower product word of (OGANOW - OGAPAST)
 * x BIT5, B-4 rev/s), the 1/2-s wait after a firing, the phase-plane
 * logic and the firing.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a DAP
 *          state assertion, or the status of the jet logic
 **********************************************************************/
agc_status_t agc_task_rolldap(void)
{
    agc_csm_tvc_t *t       = agc_csm_dap_tvc(); /* TVC DAP erasables    */
    agc_sp_t       desired = 0;                 /* desired rate, B-4    */

    if (!AGC_ASSERT(t->one_conacc >= 0) || !AGC_ASSERT((t->rollfire == 0) || (t->rollfire == AGC_SP_MAX) || (t->rollfire == AGC_SP_MIN)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* MSU OGAPAST; MP BIT5; LXCH A; TS OGARATE                          */
    t->ogarate = csm_dap_mp_lo(csm_dap_msu(t->oganow, t->ogapast), BIT5_SP);

    if ((t->rollfire == 0) && (t->temreg != 0))
    {
        t->temreg = 0; /* jets off less than 1/2 s: WAIT1/2           */

        return (AGC_STATUS_OK);
    }

    if (!roll_regions(&desired))
    {
        return (roll_noroll());
    }

    t->temreg   = desired;                 /* ROLLSET: TS TEMREG        */
    agc_sp_t tf = roll_fire_time(desired); /* unscaled 625 us/bit     */

    t->temreg = tf;

    if (tf == 0)
    {
        return (roll_noroll());
    }

    /* MOREROLL / NEWROLL .. MINTST, MAXTFIRE, NOMXFIRE, FIRELOOK, FIREPLUG
     * (TVCROLLDAP.agc pp. 994-996) -- fire for the signed firing time: keep
     * firing with the present polarity (shorten only), stop for a new sign,
     * or start a new firing of at least TMINFIRE, at most TMAXFIRE. */
    agc_csm_tvc_t *t_fire = agc_csm_dap_tvc(); /* TVC DAP erasables          */
    agc_sp_t       f      = tf;                /* TEMREG, unscaled 625 us/bit */

    if (!AGC_ASSERT(tf != 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_sp_t s = csm_dap_mp_hi(f, t_fire->rollfire); /* MP ROLLFIRE; CCS A, unscaled */

    if (s < 0)
    {
        return (roll_noroll()); /* terminate: new sign requested        */
    }

    if (s > 0)
    {
        t_fire->i_sign = 0; /* MOREROLL switch                              */
    }
    else if ((agc_sp_t) ((f < 0) ? -f : f) - TMINFIRE <= 0) /* ?: |T6FIRE| */
    {
        return (roll_noroll()); /* NEWROLL, MINTST: too short           */
    }
    else
    {
        /* NEWROLL: long enough */
    }

    if (csm_dap_mp_hi(f, RECIP_TMX) != 0)
    { /* MAXTST       */
        f = (agc_sp_t) ((f > 0) ? TMAXFIRE : -TMAXFIRE); /* ?: CCS sign  */
    }

    t_fire->rollfire = (f > 0) ? AGC_SP_MAX : AGC_SP_MIN; /* ?: NOMXFIRE    */
    t_fire->temreg   = (agc_sp_t) ((f < 0) ? -f : f);     /* ?: MAG(T6FIRE) */

    if (t_fire->i_sign == 0)
    { /* FIREPLUG: no extension of a firing       */
        agc_sp_t left = csm_dap_word_sp(agc_rupt_time(CSM_DAP_TIME6)); /* TIME6 left, unscaled 1/1600 s */

        if (left - t_fire->temreg <= 0)
        {
            return (AGC_STATUS_OK); /* EXTENTST: don't; TASKOVER        */
        }
    }

    return (roll_jetroll((uint16_t) t_fire->temreg));
}

/**********************************************************************
 * NOROLL1, KILLJETS (TVCROLLDAP.agc p. 997) -- T6 task called by
 * JETROLL: shut off all roll jets (ROLLFIRE = 0: jets now off).
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (NOQRSM); AGC_STATUS_BAD_ARG on
 *          a failed precondition
 **********************************************************************/
agc_status_t agc_task_roll_jets_off(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc(); /* TVC DAP erasables          */

    t->rollfire = 0;
    agc_chan_write(AGC_CHAN_ROLLJETS, 0u); /* WRITE CHAN6              */

    if (!AGC_ASSERT(agc_chan_read(AGC_CHAN_ROLLJETS) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}
