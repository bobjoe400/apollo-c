/*
 * tvc_rolldap.c -- the TVC roll autopilot (an adaptation of the LM P-axis
 * controller). TVCROLLDAP.agc (Comanche055 pp. 984-998): ROLLDAP,
 * DURATION, ROLLOGIC, DELOGAC, REG1TST .. REG5TST, REGION6, ROLLON,
 * ONROLL, RATEDEN, RATENUM, MINLIMAP, DVOK, RATELIM, ROLLSET, MOREROLL,
 * NEWROLL, MINTST, MAXTFIRE, NOMXFIRE, FIRELOOK, FIREPLUG, NOROLL,
 * JETROLL, +TORQUE, -TORQUE, T6ENABL, NOROLL1.
 *
 * Keeps OGA within a 5 deg deadband of OGAD (OGA at ignition, P40) and
 * the OGA rate below the 0.1 deg/s limit-cycle rate, with switching logic
 * in the phase plane (GSOP chapter 3): switching parabolas are control
 * parabolas through 1/CONACC; the firing time is that needed to reach
 * the desired OGA rate at the penetration of the coast region (regions
 * 1, 6 by a tangent-line approximation, region 3 at +-MAXLIM). T6 times
 * the firings: maximum 2.5 s (2.56 s before the limit applies), minimum
 * 15 ms, jet pairs alternate, at least 1/2 s off before a new pair fires,
 * and a firing may be shortened but never extended on a later pass.
 * Waitlist task from TVCEXEC every 1/2 s (3 cs delay).
 *
 * OGAERR (register OGA) is the fly-from error, rev scale B+0; OGARATE
 * rev/s B-4; TEMREG holds the firing time, 625 us/bit (16 bits/cs).
 * The AGC builds CA/CS instructions in I (INDEX I; 0 X); here I is +1
 * (CA), -1 (CS) or 0 (MOREROLL).
 */
#include "agc/agc_csm_dap_internal.h"

#define DB        ((agc_sp_t)228)    /* 00344 DEC .01388889: 5 deg, B+0 rev */
#define NEG_SLOPE ((agc_sp_t)3277)   /* 06315 DEC 0.2: -switch-line slope /s */
#define LMCRATE   ((agc_sp_t)73)     /* 00111 DEC .00027778 B+4: 0.1 deg/s   */
#define INTERCEP  ((agc_sp_t)328)    /* 00510 DEC .0025 B+3: DB(-SLOPE)-LMCRATE */
#define MINLIM    ((agc_sp_t)728)    /* 01330 DEC .00277778 B+4: 1 deg/s     */
#define MAXLIM    ((agc_sp_t)3641)   /* 07071 DEC .01388889 B+4: 5 deg/s     */
#define TMINFIRE  ((agc_sp_t)24)     /* DEC 1.5 B+4: 15 ms, 16 bits/cs       */
#define TMAXFIRE  ((agc_sp_t)4000)   /* DEC 250 B+4: 2.5 s                   */
#define RECIP_TMX ((agc_sp_t)4)      /* 1/TMXFIR = BIT3 (allows 2.56 s)      */
#define T6SCALE   ((agc_sp_t)12800)  /* PRIO31: (B+3)(16 bits/cs)(100 cs/s)  */
#define BIT5_SP   ((agc_sp_t)16)
#define BIT11_SP  ((agc_sp_t)1024)
#define BIT14_SP  ((agc_sp_t)8192)
#define PLUS_ROLL1  ((uint16_t)000005) /* FIVE: jets 9 and 11               */
#define PLUS_ROLL2  ((uint16_t)000120) /* OCT120: jets 13 and 15            */
#define MINUS_ROLL1 ((uint16_t)000012) /* TEN: jets 12 and 10               */
#define MINUS_ROLL2 ((uint16_t)000240) /* OCT 240: jets 16 and 14           */

static agc_sp_t sgn_mul(int i, agc_sp_t v)     /* INDEX I; 0 v: CA or CS v */
{
    (void)AGC_ASSERT((i == 1) || (i == -1));
    return (i < 0) ? (agc_sp_t)-v : v;
}

/* ROLLDAP (p. 989): OGA rate, first-order difference over 1/2 s; the low
 * product word of (OGANOW - OGAPAST) x BIT5 (LXCH A), B-4 rev/s.         */
static void roll_rate(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_sp_t d = csm_dap_msu(t->oganow, t->ogapast);
    t->ogarate = csm_dap_mp_lo(d, BIT5_SP);
    (void)AGC_ASSERT(t->ogarate >= AGC_SP_MIN);
    (void)AGC_ASSERT(t->oganow <= CSM_DAP_NEG0);
}

/* ROLLON / REGION6 .. ONROLL .. DVOK (pp. 992-994): desired OGA rate at
 * the intersection of the tangent to the control parabola with the
 * straight switch line: DOGADOT = NUM / DEN, limited to +-POSMAX.       */
static agc_sp_t roll_desired_rate(agc_sp_t deloga, agc_sp_t delogart, int i)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_sp_t a = csm_dap_mp_hi(delogart, t->one_conacc);
    agc_sp_t den = csm_dap_mp_hi(csm_dap_ads(a, a), NEG_SLOPE); /* 2(-S)R/CONACC */
    agc_sp_t num = csm_dap_mp_hi(den, delogart);  /* 2(-SLOPE)(RATESQ)/CONACC */
    den = csm_dap_ads(den, sgn_mul(i, (agc_sp_t)-BIT11_SP)); /* RATEDEN    */
    agc_sp_t e = csm_dap_ads(sgn_mul(i, deloga), (agc_sp_t)-DB);
    num = csm_dap_ads(num, csm_dap_mp_hi(e, NEG_SLOPE));
    t->temreg = den;
    t->delogart = num;
    num = csm_dap_ads(csm_dap_mp_hi(LMCRATE, BIT11_SP), num); /* RATENUM   */
    if (!AGC_ASSERT(den != 0)) {
        return 0;
    }
    int q = (int)num / (int)den;     /* DV (0, NUM): overflow check       */
    if (q != 0) {
        return (q > 0) ? AGC_SP_MAX : AGC_SP_MIN;  /* MINLIMAP            */
    }
    return csm_dap_dv(agc_dp_pack(num, 0), den);   /* DVOK, B-4 rev/s     */
}

/* ROLLSET (p. 994): jet firing time from the desired minus the present
 * rate: (dRATE) T6SCALE 1/CONACC x 4, overflow-limited, 625 us/bit.      */
static agc_sp_t roll_fire_time(agc_sp_t desired)
{
    const agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_sp_t d = csm_dap_ad_sat(desired, (agc_sp_t)-t->ogarate); /* SU     */
    agc_sp_t a = csm_dap_mp_hi(d, T6SCALE);
    int64_t v = (int64_t)agc_sp_mul(a, t->one_conacc) * 4; /* DDOUBL x2   */
    int64_t hi = v / 16384;                        /* upper word          */
    (void)AGC_ASSERT((hi > -2 * 16384) && (hi < 2 * 16384)); /* MAX < .60 */
    if (hi > AGC_SP_MAX) {
        return AGC_SP_MAX;                         /* INDEX A; CS LIMITS  */
    }
    if (hi < AGC_SP_MIN) {
        return AGC_SP_MIN;
    }
    return (agc_sp_t)hi;
}

/* ROLLOGIC .. REG5TST, REGION6, ROLLON, RATELIM (pp. 990-994): the
 * phase-plane region of the operating point; returns false for a coast
 * region (NOROLL), else the desired OGA rate in *desired.               */
static bool roll_regions(agc_sp_t *desired)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_sp_t r = t->ogarate;
    /* TEMREG = DB - (1/2 CONACC) OGARATE^2 (the 1/2 is in the scaling)  */
    agc_sp_t q = csm_dap_mp_hi(csm_dap_mp_hi((agc_sp_t)-r, t->one_conacc), r);
    t->temreg = csm_dap_ads(q, DB);
    t->sgnrt = (r > 0) ? (agc_sp_t)1 : (agc_sp_t)-1; /* sgn(0) negative   */
    /* DELOGA: distance from the switch parabola, + to its right         */
    t->deloga = csm_dap_ads(t->ogaerr, (agc_sp_t)-(t->sgnrt * t->temreg));
    int i = (t->deloga > 0) ? 1 : -1;              /* CA or CS            */
    t->i_sign = i;
    if (sgn_mul(i, t->sgnrt) > 0) {                /* REG1TST: region 1   */
        t->deloga = csm_dap_ads(t->deloga, sgn_mul(i, DB));      /* ROLLON */
        t->delogart = sgn_mul(i, (agc_sp_t)-LMCRATE); /* dummy point      */
        *desired = roll_desired_rate(t->deloga, t->delogart, i);
        return true;
    }
    t->iogarate = sgn_mul(i, r);
    if (csm_dap_ads(t->iogarate, MAXLIM) <= 0) {   /* REG3TST: too fast   */
        *desired = sgn_mul(i, (agc_sp_t)-MAXLIM);  /* RATELIM             */
        return true;
    }
    agc_dp_t s = csm_dap_ddoubl(csm_dap_ddoubl(csm_dap_ddoubl(
                     agc_sp_mul(t->ogaerr, NEG_SLOPE))));
    agc_sp_t x = csm_dap_ads(agc_dp_high(s), csm_dap_mp_hi(r, BIT14_SP));
    if ((csm_dap_ads(sgn_mul(i, x), (agc_sp_t)-INTERCEP) <= 0) ||  /* REG2 */
        (csm_dap_ads(t->iogarate, MINLIM) <= 0) ||                 /* REG4 */
        (csm_dap_ads(sgn_mul(i, t->ogaerr), (agc_sp_t)-DB) <= 0)) { /* REG5 */
        return false;
    }
    t->deloga = t->ogaerr;                         /* REGION6: actual state */
    t->delogart = r;
    *desired = roll_desired_rate(t->deloga, t->delogart, i);
    return true;
}

/* +TORQUE / -TORQUE (pp. 996-997): alternate the jet pairs.              */
static agc_status_t roll_jets_on(bool plus)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    uint16_t bit = plus ? CSM_DAP_BIT1 : CSM_DAP_BIT2;
    uint16_t jets;
    if ((t->rollword & bit) == 0u) {               /* NO.9-11 / NO.12-10  */
        t->rollword = (uint16_t)(t->rollword | bit);
        jets = plus ? PLUS_ROLL1 : MINUS_ROLL1;
    } else {                                       /* NO.13-15 / NO.16-14 */
        t->rollword = (uint16_t)(t->rollword & ~(unsigned)bit);
        jets = plus ? PLUS_ROLL2 : MINUS_ROLL2;
    }
    (void)AGC_ASSERT(t->rollword <= 3u);
    return agc_chan_write(AGC_CHAN_ROLLJETS, jets);
}

/* JETROLL, SAMEJETS, T6ENABL (pp. 996-997): NOROLL1 in TEMREG T6 counts;
 * time6 is the raw TIME6 word (NOROLL: -0, an immediate T6RUPT).         */
static agc_status_t roll_jetroll(uint16_t time6)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_status_t st = agc_rupt_set_t6_task(AGC_TASK_NOROLL1);
    if (st == AGC_STATUS_OK) {
        st = agc_rupt_set_time(CSM_DAP_TIME6, time6);
    }
    if ((st != AGC_STATUS_OK) || (t->i_sign == 0)) {
        return st;                                 /* keep the same jets  */
    }
    if (t->rollfire > 0) {
        st = roll_jets_on(true);
    } else if (t->rollfire < 0) {
        st = roll_jets_on(false);
    } else {
        /* coast: no new jets */
    }
    if (st == AGC_STATUS_OK) {
        st = agc_chan_or(AGC_CHAN_CHAN13, (uint16_t)AGC_CH013_ENBT6RUPT);
    }
    return st;                                     /* RDAPEND: TASKOVER   */
}

/* NOROLL (p. 996): coast; jets may fire next pass.                       */
static agc_status_t roll_noroll(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    t->rollfire = 0;                               /* -0                  */
    t->temreg = 0;                                 /* -0 for TIME6        */
    (void)AGC_ASSERT(t->i_sign != 0);
    return roll_jetroll(CSM_DAP_NEG0);
}

/* MOREROLL / NEWROLL .. NOMXFIRE .. FIREPLUG (pp. 995-996): fire for
 * TEMREG (non-zero, signed: + is positive roll torque).                  */
static agc_status_t roll_fire(agc_sp_t tf)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    agc_sp_t s = csm_dap_mp_hi(tf, t->rollfire);   /* same sign as now?   */
    if (s < 0) {
        return roll_noroll();                      /* new sign requested  */
    }
    if (s > 0) {
        t->i_sign = 0;                             /* MOREROLL switch     */
    } else {
        agc_sp_t mag = (tf < 0) ? (agc_sp_t)-tf : tf;  /* NEWROLL         */
        if (mag - TMINFIRE <= 0) {
            return roll_noroll();                  /* MINTST              */
        }
    }
    if (csm_dap_mp_hi(tf, RECIP_TMX) != 0) {       /* MAXTST              */
        tf = (tf > 0) ? TMAXFIRE : (agc_sp_t)-TMAXFIRE;
    }
    t->rollfire = (tf > 0) ? AGC_SP_MAX : AGC_SP_MIN; /* NOMXFIRE        */
    t->temreg = (tf < 0) ? (agc_sp_t)-tf : tf;     /* MAG(T6FIRE)         */
    if (t->i_sign == 0) {                          /* FIREPLUG: extension? */
        agc_sp_t left = csm_dap_word_sp(agc_rupt_time(CSM_DAP_TIME6));
        if (left - t->temreg <= 0) {
            return AGC_STATUS_OK;                  /* don't; TASKOVER     */
        }
    }
    return roll_jetroll((uint16_t)t->temreg);
}

/* ROLLDAP, DURATION, WAIT1/2 (p. 989): waitlist task.                    */
agc_status_t agc_task_rolldap(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    roll_rate();
    if ((t->rollfire == 0) && (t->temreg != 0)) {
        t->temreg = 0;           /* jets not off for 1/2 s yet: wait      */
        return AGC_STATUS_OK;
    }
    agc_sp_t desired = 0;
    if (!roll_regions(&desired)) {
        return roll_noroll();
    }
    t->temreg = desired;                           /* ROLLSET             */
    agc_sp_t tf = roll_fire_time(desired);
    t->temreg = tf;
    if (tf == 0) {
        return roll_noroll();
    }
    return roll_fire(tf);
}

/* NOROLL1 (p. 997): T6 task called by JETROLL; shut off all roll jets.   */
agc_status_t agc_task_noroll1(void)
{
    agc_csm_tvc_t *t = agc_csm_dap_tvc();
    t->rollfire = 0;                               /* jets now off        */
    agc_status_t st = agc_chan_write(AGC_CHAN_ROLLJETS, 0u); /* KILLJETS  */
    (void)AGC_ASSERT(t->rollfire == 0);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return st;                                     /* NOQRSM              */
}
