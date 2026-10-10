/*
 * findcduw.c -- FINDCDUW--GUIDAP_INTERFACE.agc (Luminary 099, pp. 908-925).
 * Program FINDCDUW, mod 1 (68-07-15), author Klumpp.
 *
 * FINDCDUW is the interface between the powered-flight guidance programs
 * and the digital autopilot. Its inputs are the thrust command vector
 * UNFC/2 and the window command vector UNWC/2; its outputs are the gimbal
 * angle increments DELCDUX/Y/Z, the commanded attitude rates OMEGAPD/QD/RD
 * and the attitude lag angles DELPEROR/QEROR/REROR (the angles by which the
 * body lags a ramp command because of the finite angular accelerations).
 * It aligns the estimated thrust vector from the thrust direction filter
 * with the thrust command and, when X-axis override is inhibited
 * (XOVINHIB), the +Z half of the LM ZX plane with the window command.
 * CPHI/CTHETA/CPSI (THETAD) receive the commanded gimbal angles for N22.
 *
 * Alarms: 00401 if the inputs call for an attitude in gimbal lock (CDUXD
 * and CDUYD are driven to the required values, CDUZD only to the 70 degree
 * cone); 00402 if UNFC/2 or UNWC/2 overflow in NORMUNIT (a STOPRATE is then
 * the only autopilot input).
 *
 * Number representation. C integers hold the mathematical value of a word.
 * A ones' complement value and a two's complement value of the same number
 * are therefore the same agc_sp_t; the representation only matters where
 * the AGC does modular (MSU) arithmetic on 15-bit words or reads a two's
 * complement word as ones' complement. Those places use the word helpers
 * below and are marked. Angles "2's, pi" are two's complement, scaled pi
 * radians (16384 = pi), range -16384 .. 16383.
 */
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/io/service_internal.h"
#include "agc/lm/landing.h"
#include "pending.h"

/* ---- constants (p. 925) ------------------------------------------------ */
static const agc_sp_t fc_gainfltr[2] = {
    3277, /* GAINFLTR DEC .2  gain of filter without CSM               */
    1638  /*          DEC .1  gain of filter with CSM docked           */
};
#define FC_DUNFVLIM 57   /* DEC .007 B-1: 7 mr max change in 2 s       */
#define FC_UNFVLIM  1057 /* DEC .129 B-1: 129 mr max thrust offset     */
/* DOTSWFMX DEC .93302 B-4 = cos^2(15 deg)/16, whose low word is the next
 * constant DAXMAX (DEC .11111111111): the DP limit on (UNWC.UNFC)^2.     */
#define FC_DOTSWFMX_HI 955
#define FC_DOTSWFMX_LO 1820
static const agc_sp_t fc_daxmax[2] = {
    /* DAXMAX = DAZMAX, 1's, pi     */
    1820, /* DEC .11111111111  20 deg in 2 s                          */
    182   /* DEC .0111111111   2 deg when CSM docked                  */
};
static const agc_sp_t fc_day2max[2] = {
    /* DAY/2MAX, 1's, pi            */
    910, /* DEC .05555555555                                         */
    91   /* DEC .0055555555                                          */
};
#define FC_CDUZDLIM  6372     /* DEC .3888888888: 70 deg MGA limit, 1's, pi */
#define FC_DT_DELT   819      /* DEC .05: 0.1 s DAP period / 2 s FINDCDUW   */
#define FC_DELERLIM  910      /* DELERLIM = DAY/2MAX: 10 deg lag limit      */
#define FC_HI5       (-1023)  /* HI5 OCT 76000: -(45 deg)^2 within 1 bit   */
#define FC_BIT11     1024     /* 1/16                                       */
#define FC_NEGMAX_2S (-16384) /* NEGMAX read as 2's complement: pi        */
#define FC_HALF      8192
/* SPARCSIN polynomial (Bob Crisp), argument scaled 1, angle scaled 180 deg */
#define FC_DPL1 10502
#define FC_DPL3 432
#define FC_DPL5 7300
#define FC_DPL7 (-11803)
#define FC_DPL9 8397

/* ---- erasables of the ECDUW block (ERASABLE_ASSIGNMENTS, "overlaying
 * KALCMANU storage"); module private.                                    */
static agc_vec_t fc_unfv_2;    /* UNFV/2 thrust direction in vehicle axes,
                                   half unit; only Y and Z are filtered   */
static agc_sp_t  fc_ndxcduw;   /* NDXCDUW 1 if CSM docked, else 0        */
static agc_sp_t  fc_flpautno;  /* FLPAUTNO non-zero = not PNGCS auto     */
static agc_sp_t  fc_flagoodw;  /* FLAGOODW non-zero = window good        */
static agc_sp_t  fc_delgmb[3]; /* -DELGMB X Y Z: -gimbal angle changes,
                                   1's, pi (later -DELATT, then limited)  */

/* ---- word-level helpers ------------------------------------------------ */

/* Wrap a difference of 15-bit two's complement words (MSU). */
static agc_sp_t fc_wrap15(int32_t d)
{
    int32_t w = d & 0x7FFF;
    if (w >= 0x4000)
    {
        w -= 0x8000;
    }
    (void) AGC_ASSERT(w >= -16384 && w <= 16383);
    return (agc_sp_t) w;
}

/* Value of a two's complement word when the AGC reads it as ones'. */
static agc_sp_t fc_ones_of_twos(agc_sp_t v)
{
    (void) AGC_ASSERT(v >= -16384);
    return (agc_sp_t) ((v < 0) ? v + 1 : v);
}

/* Value of a ones' complement word when the AGC reads it as two's. */
static agc_sp_t fc_twos_of_ones(agc_sp_t v)
{
    (void) AGC_ASSERT(v >= AGC_SP_MIN);
    return (agc_sp_t) ((v < 0) ? v - 1 : v);
}

/* 15-bit word of a ones' complement value (what MSU sees as two's). */
static int32_t fc_word1(agc_sp_t v)
{
    (void) AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX);
    int32_t w = (v < 0) ? 0x7FFF + (int32_t) v : (int32_t) v;
    (void) AGC_ASSERT(w >= 0 && w <= 0x7FFF);
    return w;
}

/* DELCDUX, DELCDUY, DELCDUZ by index (INDEX TEM2 / TS DELCDUX). */
static void fc_set_delcdu(int i, agc_sp_t v)
{
    (void) AGC_ASSERT(i >= 0 && i <= 2);
    (void) AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX);
    if (i == 0)
    {
        agc_erase_set_delcdux(v);
    }
    else if (i == 1)
    {
        agc_erase_set_delcduy(v);
    }
    else
    {
        agc_erase_set_delcduz(v);
    }
}

/* MP followed by DDOUBL: high word of twice the DP product. */
static agc_sp_t fc_mp_dbl(agc_sp_t a, agc_sp_t b)
{
    int32_t p  = 2 * (int32_t) a * (int32_t) b;
    int32_t hi = p / 16384;
    if (!AGC_ASSERT(hi <= AGC_SP_MAX && hi >= AGC_SP_MIN))
    {
        hi = (hi > 0) ? AGC_SP_MAX : AGC_SP_MIN;
    }
    return (agc_sp_t) hi;
}

/* ONESTO2S (p. 924): ones' complement SP -> two's complement 15-bit word.
 * (CCS A / AD ONE / CS A: the value is unchanged, only the bit pattern of
 * a negative number moves by one.)                                       */
uint16_t agc_lm_onesto2s(agc_sp_t a)
{
    (void) AGC_ASSERT(a >= AGC_SP_MIN && a <= AGC_SP_MAX);
    uint16_t w = (uint16_t) ((int32_t) a & 0x7FFF);
    (void) AGC_ASSERT(w <= 0x7FFFu);
    return w;
}

/* LIMITSUB (p. 924, "version courtesy Hugh Blair-Smith"): limit the
 * magnitude of x to the positive limit; DV by the limit gives a non-zero
 * quotient exactly when |x| >= limit.                                   */
agc_sp_t agc_lm_limitsub(agc_sp_t x, agc_sp_t limit)
{
    if (!AGC_ASSERT(limit > 0))
    {
        return 0;
    }
    (void) AGC_ASSERT(x >= AGC_SP_MIN && x <= AGC_SP_MAX);
    if (x >= limit)
    {
        return limit;
    }
    if (x <= (agc_sp_t) -limit)
    {
        return (agc_sp_t) -limit;
    }
    return x;
}

/* SPARCSIN (p. 923, Bob Crisp): argument scaled 1, angle scaled 180 deg,
 * max error +-5 bits in +-.94. SR receives x/2 (writing SR shifts it).  */
agc_sp_t agc_lm_sparcsin(agc_sp_t x)
{
    (void) AGC_ASSERT(x >= AGC_SP_MIN && x <= AGC_SP_MAX);
    agc_sp_t sr = (agc_sp_t) (x / 2); /* TS SR             */
    agc_sp_t x2 = agc_lm_mp_hi(x, x); /* MP A, TS TEM1     */
    agc_sp_t p  = agc_lm_mp_hi(x2, FC_DPL9);
    p           = agc_lm_mp_hi(agc_lm_ad(p, FC_DPL7), x2);
    p           = agc_lm_mp_hi(agc_lm_ad(p, FC_DPL5), x2);
    p           = agc_lm_mp_hi(agc_lm_ad(p, FC_DPL3), x2);
    agc_sp_t r  = agc_lm_mp_hi(agc_lm_ad(p, FC_DPL1), sr);
    (void) AGC_ASSERT(r >= -8192 && r <= 8192);
    return r;
}

/* SPARCSIN -1: argument scaled 2; DOUBLE, and on overflow TS SR skips and
 * INDEX A / CS LIMITS picks +-POSMAX (LIMITS-1 = POSMAX, LIMITS+1 = BIT15). */
agc_sp_t agc_lm_sparcsin_m1(agc_sp_t x2)
{
    int32_t d = 2 * (int32_t) x2;
    if (d > AGC_SP_MAX)
    {
        d = AGC_SP_MAX;
    }
    else if (d < AGC_SP_MIN)
    {
        d = AGC_SP_MIN;
    }
    (void) AGC_ASSERT(d >= AGC_SP_MIN && d <= AGC_SP_MAX);
    return agc_lm_sparcsin((agc_sp_t) d);
}

/* ARCTRGSP (p. 922): two's complement angle (pi) from sine in A and cosine
 * in L, both in units of 2; unambiguous anywhere in the circle, +-4 bits.
 * SPARCSIN is used only within +-45 degrees.                            */
agc_sp_t agc_lm_arctrgsp(agc_sp_t s, agc_sp_t c)
{
    (void) AGC_ASSERT(s >= AGC_SP_MIN && c >= AGC_SP_MIN);
    if (s == 0)
    { /* SINZERO           */
        return (c >= 0) ? 0 : FC_NEGMAX_2S;
    }
    int32_t as = (s < 0) ? -s : s;
    int32_t ac = (c < 0) ? -c : c;
    if (ac < as)
    { /* USECOS            */
        agc_sp_t a = agc_lm_ad(agc_lm_sparcsin_m1((agc_sp_t) -c), FC_HALF);
        return (s > 0) ? a : (agc_sp_t) -a; /* 1TO2&TCQ          */
    }
    agc_sp_t sn   = s;
    agc_sp_t base = 0;
    if (c < 0)
    { /* angle = pi - arcsin(sin)              */
        sn   = (agc_sp_t) -s;
        base = FC_NEGMAX_2S;
    }
    agc_sp_t a = agc_lm_sparcsin_m1(sn);
    agc_sp_t r = fc_wrap15((int32_t) a - base); /* MSU TEM3          */
    (void) AGC_ASSERT(r >= -16384 && r <= 16383);
    return r;
}

/* DVBYCOSM (p. 921): divide a matrix element (already containing
 * cos(MGA)/2) by cos(MGA) in TEM5 when that cannot overflow; otherwise
 * leave TEM5 - |high word| (gimbal angles then random multiples of pi/2). */
static agc_sp_t fc_dvbycosm(agc_dp_t op, agc_sp_t tem5)
{
    agc_sp_t hi   = agc_dp_high(op);
    int32_t  diff = (int32_t) tem5 - ((hi < 0) ? -hi : hi);
    (void) AGC_ASSERT(tem5 >= 0);
    if (diff <= 0)
    {
        return (agc_sp_t) diff; /* TSL&TCQ           */
    }
    agc_sp_t q = agc_lm_dv(op, tem5);
    (void) AGC_ASSERT(q >= AGC_SP_MIN && q <= AGC_SP_MAX);
    return q;
}

/* NB2CDUSP (p. 920): two's complement (pi) SP CDU angles X, Y, Z from the
 * matrix whose rows are the half-unit nav base X, Y, Z vectors in stable
 * member axes (push list 0, 6, 12).                                     */
static agc_status_t fc_nb2cdusp(agc_interp_t *ip, const agc_mat_t *nb,
                                agc_sp_t cdu[3])
{
    if (!AGC_ASSERT(ip != NULL && nb != NULL && cdu != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dp_t c2 =
        agc_dp_sub(ip, AGC_DP_QUARTER, agc_dp_sq(ip, nb->row[0].c[1]));
    if (c2 < 0)
    {
        c2 = 0; /* ZEROVECS: sin slightly > 1/2       */
    }
    agc_dp_t cos2 = agc_dp_sqrt(ip, c2); /* cos(CDUZ) in units of 2       */
    agc_sp_t tem5 = AGC_SP_MAX;          /* DDOUBL overflow -> POSMAX     */
    if (cos2 < AGC_DP_HALF)
    {
        tem5 = agc_dp_high(2 * cos2); /* cos(CDUZ) in units of 1      */
    }
    cdu[2] = agc_lm_arctrgsp(agc_dp_high(nb->row[0].c[1]), agc_dp_high(cos2));
    agc_sp_t cy = fc_dvbycosm(nb->row[0].c[0], tem5);
    agc_sp_t sy = fc_dvbycosm(nb->row[0].c[2], tem5);
    cdu[1]      = agc_lm_arctrgsp((agc_sp_t) -sy, cy); /* CDUY             */
    agc_sp_t cx = fc_dvbycosm(nb->row[1].c[1], tem5);
    agc_sp_t sx = fc_dvbycosm(nb->row[2].c[1], tem5);
    cdu[0]      = agc_lm_arctrgsp((agc_sp_t) -sx, cx); /* CDUX             */
    return AGC_STATUS_OK;
}

/* FLTRSUB (p. 919): one component of the thrust direction filter. The
 * offset moves toward the measurement by GAINFLTR, at most DUNFVLIM per
 * pass, and the total is limited to UNFVLIM.                            */
static agc_sp_t fc_fltrsub(agc_sp_t orig, agc_sp_t meas)
{
    if (!AGC_ASSERT(fc_ndxcduw == 0 || fc_ndxcduw == 1))
    {
        return orig;
    }
    agc_sp_t incr  = agc_lm_mp_hi(agc_lm_ad(meas, (agc_sp_t) -orig),
                                  fc_gainfltr[fc_ndxcduw]);
    incr           = agc_lm_limitsub(incr, FC_DUNFVLIM);
    agc_sp_t total = agc_lm_limitsub(agc_lm_ad(incr, orig), FC_UNFVLIM);
    (void) AGC_ASSERT(total >= -FC_UNFVLIM && total <= FC_UNFVLIM);
    return total;
}

/* UNWCTEST (p. 919): is the proposed window vector at least 15 degrees
 * from the thrust command? If not, the window-good flag is zeroed.      */
static bool fc_unwctest(agc_interp_t *ip, agc_vec_t unz, agc_vec_t unx)
{
    (void) AGC_ASSERT(ip != NULL);
    agc_dp_t d   = agc_dp_sq(ip, agc_vec_dot(ip, unz, unx));
    agc_dp_t lim = agc_dp_pack(FC_DOTSWFMX_HI, FC_DOTSWFMX_LO);
    if (agc_dp_sub(ip, d, lim) < 0)
    {
        return true; /* BMN DCMCL        */
    }
    fc_flagoodw = 0; /* SSP FLAGOODW 0   */
    (void) AGC_ASSERT(fc_flagoodw == 0);
    return false;
}

/* DCMCL (p. 913): the required direction cosine matrix. The filtered
 * thrust offsets (UNFVY/2, UNFVZ/2) tilt X so that the estimated thrust,
 * not the X axis, lies along the command.                               */
static agc_mat_t fc_dcmcl(agc_interp_t *ip, agc_vec_t unx, agc_vec_t unz)
{
    (void) AGC_ASSERT(ip != NULL);
    agc_vec_t uny1 = agc_vec_unit(ip, agc_vec_cross(ip, unz, unx), NULL, NULL);
    agc_vec_t munz = agc_vec_shl(ip, agc_vec_cross(ip, uny1, unx), 1);
    agc_vec_t a    = agc_vec_scale(ip, munz, fc_unfv_2.c[2]);
    agc_vec_t b    = agc_vec_scale(ip, uny1, fc_unfv_2.c[1]);
    agc_vec_t t    = agc_vec_shl(ip, agc_vec_sub(ip, a, b), 1); /* BVSU     */
    agc_mat_t m;
    m.row[0] = agc_vec_unit(ip, agc_vec_add(ip, t, unx), NULL, NULL);
    m.row[1] = agc_vec_shl(ip, agc_vec_cross(ip, m.row[0], munz), 1);
    m.row[2] =
        agc_vec_shl(ip, agc_vec_cross(ip, agc_vec_neg(m.row[1]), m.row[0]), 1);
    (void) AGC_ASSERT(m.row[0].c[0] <= AGC_DP_MAX);
    return m;
}

/* INITCDUW (p. 910): one call before each guided maneuver. */
agc_status_t agc_lm_initcduw(void)
{
    agc_vec_t unitx      = agc_vec_make(AGC_DP_HALF, 0, 0);
    fc_unfv_2            = unitx;
    agc_lm_ldg()->unwc_2 = unitx;
    (void) AGC_ASSERT(fc_unfv_2.c[0] == AGC_DP_HALF);
    (void) AGC_ASSERT(agc_lm_ldg()->unwc_2.c[1] == 0);
    return AGC_STATUS_OK;
}

/* Housekeeping and FETCH BASIC DATA (pp. 910-911): index, flags, and the
 * CDUs (or, in PNGCS auto, the desired CDUs) for the trig.             */
static void fc_fetch_basic(uint16_t yzx[3])
{
    fc_ndxcduw  = agc_flag_test(AGC_FLAG_CSMDKFLG) ? 1 : 0;
    bool xov    = agc_flag_test(AGC_FLAG_XOVINFLG);
    fc_flpautno = xov ? 1 : 0; /* set to PNZ flag "PNGCS auto not"  */
    fc_flagoodw = xov ? 1 : 0; /* PNZ if X override inhibited      */
    agc_inhint();              /* RELINT at PAUTNO                 */
    yzx[0] = agc_erase_cduy();
    yzx[1] = agc_erase_cduz();
    yzx[2] = agc_erase_cdux();
    if (agc_lm_chan_bit_present(AGC_CHAN_CHAN30, AGC_CH030_LMCONTROL) &&
        agc_lm_chan_bit_present(AGC_CHAN_CHAN31, AGC_CH031_AUTOSTAB))
    {
        fc_flpautno = 0; /* PNGCS auto: use CDUXD,YD,ZD      */
        yzx[0]      = agc_erase_cduyd();
        yzx[1]      = agc_erase_cduzd();
        yzx[2]      = agc_erase_cduxd();
    }
    agc_relint();
    (void) AGC_ASSERT(fc_ndxcduw == 0 || fc_ndxcduw == 1);
    (void) AGC_ASSERT(yzx[0] <= 0x7FFFu && yzx[2] <= 0x7FFFu);
}

/* NOATTCNT (p. 924): STOPRATE replaces the autopilot commands. */
static agc_status_t fc_noattcnt(bool alarm)
{
    if (alarm)
    {
        agc_alarm(AGC_ALARM_00402); /* alarm is recorded; go on  */
    }
    agc_inhint();
    agc_status_t st = agc_lm_dap_stoprate();
    agc_relint();
    (void) AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_UNIMPLEMENTED);
    return AGC_STATUS_OK;
}

/* Limit the MGA (alarm 00401), output CPHI.. for N22, and form the
 * unlimited -DELGMB = CDUxD - required angle (pp. 914-915, MGARET loop). */
static void fc_gimbal_changes(agc_sp_t cdu[3])
{
    agc_sp_t lim     = agc_lm_limitsub(fc_ones_of_twos(cdu[2]), FC_CDUZDLIM);
    agc_sp_t limited = fc_twos_of_ones(lim);
    if (fc_wrap15((int32_t) cdu[2] - limited) != 0)
    {
        agc_alarm(AGC_ALARM_00401); /* ALARMMGA         */
    }
    cdu[2]                 = limited;
    const uint16_t cdud[3] = {agc_erase_cduxd(), agc_erase_cduyd(),
                              agc_erase_cduzd()};
    agc_inhint(); /* RELINT after TCQCDUW         */
    agc_sp_t l = 0;
    for (int i = 2; i >= 0; i--)
    { /* DELGMBLP, bound 3            */
        if (agc_lm_ad(agc_lm_mp_hi(l, l), FC_HI5) > 0)
        {
            fc_flagoodw = 0; /* DELGMBZ or Y beyond 45 deg   */
        }
        (void) agc_erase_set_thetad(i, agc_lm_onesto2s(cdu[i]));
        agc_sp_t d   = fc_wrap15((int32_t) cdu[i] - (int32_t) cdud[i]);
        fc_delgmb[i] = (agc_sp_t) -d; /* MSU CDUXD, COM               */
        l            = fc_delgmb[i];
    }
    (void) AGC_ASSERT(fc_delgmb[0] >= AGC_SP_MIN);
}

/* Limit the attitude angle changes about X, Y', Z' (p. 916).            */
static void fc_limit_attitude(const agc_cdutrig_t *trig)
{
    agc_sp_t sinz = agc_dp_high(trig->sincdu[AGC_TRIG_Z]);
    agc_sp_t cosz = agc_dp_high(trig->coscdu[AGC_TRIG_Z]);
    int      n    = fc_ndxcduw;
    (void) AGC_ASSERT(n == 0 || n == 1);
    fc_delgmb[2] = agc_lm_limitsub(fc_delgmb[2], fc_daxmax[n]); /* DAZMAX */
    agc_sp_t y2  = agc_lm_mp_hi(fc_delgmb[1], cosz); /* -DELATTY'/2       */
    y2           = agc_lm_limitsub(y2, fc_day2max[n]);
    /* DEVIATION: DV takes (A,L); L after LIMITSUB is modelled as A (true
     * unless the value was limited negative, where L is a DV remainder). */
    agc_sp_t newy = AGC_SP_MAX;
    if (cosz != 0 && ((y2 < 0) ? -y2 : y2) < ((cosz < 0) ? -cosz : cosz))
    {
        newy = agc_lm_dv(agc_dp_pack(y2, y2), cosz);
    }
    agc_sp_t oldy = fc_delgmb[1];
    fc_delgmb[1]  = newy; /* -DELGMBY          */
    agc_sp_t t    = (agc_sp_t) -fc_mp_dbl(oldy, sinz);
    /* MSU on ones' complement operands: one bit error if signs mixed.   */
    agc_sp_t datx = fc_wrap15(fc_word1(t) - fc_word1(fc_delgmb[0]));
    datx          = agc_lm_limitsub(datx, fc_daxmax[n]); /* +DELATTX limited  */
    fc_delgmb[0]  = (fc_flagoodw != 0) ? (agc_sp_t) -datx : 0;
    agc_sp_t c    = fc_mp_dbl((agc_sp_t) -fc_delgmb[1], sinz);
    fc_delgmb[0]  = agc_lm_ad(fc_delgmb[0], c); /* ADS: -DELGMBX     */
}

/* Commanded attitude rates (p. 917), pi/4 rad/s, for a 2 s period. */
static void fc_rates(const agc_cdutrig_t *trig)
{
    agc_sp_t sinz = agc_dp_high(trig->sincdu[AGC_TRIG_Z]);
    agc_sp_t cosz = agc_dp_high(trig->coscdu[AGC_TRIG_Z]);
    agc_sp_t sinx = agc_dp_high(trig->sincdu[AGC_TRIG_X]);
    agc_sp_t cosx = agc_dp_high(trig->coscdu[AGC_TRIG_X]);
    agc_sp_t gx   = (agc_sp_t) -fc_delgmb[0];
    agc_sp_t gy   = (agc_sp_t) -fc_delgmb[1];
    agc_sp_t gz   = (agc_sp_t) -fc_delgmb[2];
    agc_sp_t p    = agc_lm_ad(gx, fc_mp_dbl(gy, sinz));
    p             = agc_lm_ad(p, p); /* two ADS          */
    agc_sp_t q    = agc_lm_mp_hi(fc_mp_dbl(gy, cosx), cosz);
    q             = agc_lm_ad(q, agc_lm_mp_hi(gz, sinx));
    q             = agc_lm_ad(q, q);
    q             = agc_lm_ad(q, q); /* three ADS        */
    agc_sp_t r    = agc_lm_mp_hi(fc_mp_dbl((agc_sp_t) -gy, sinx), cosz);
    r             = agc_lm_ad(r, agc_lm_mp_hi(gz, cosx));
    r             = agc_lm_ad(r, r);
    r             = agc_lm_ad(r, r);
    agc_erase_set_omegapd(p);
    agc_erase_set_omegaqd(q);
    agc_erase_set_omegard(r);
    (void) AGC_ASSERT(p >= AGC_SP_MIN && q >= AGC_SP_MIN && r >= AGC_SP_MIN);
}

/* CDUWXFR (p. 918): angle interface and lag angles
 * DELxEROR = OMEGA |OMEGA| / (2 * 1JACC), limited to DELERLIM.          */
static void fc_transfer(void)
{
    const agc_sp_t omega[3] = {agc_erase_omegapd(), agc_erase_omegaqd(),
                               agc_erase_omegard()};
    const agc_sp_t jacc[3]  = {agc_erase_1jacc(), agc_erase_1jaccq(),
                               agc_erase_1jaccr()};
    agc_sp_t       lag[3];
    for (int i = 2; i >= 0; i--)
    { /* bound 3          */
        agc_sp_t inc = agc_lm_mp_hi(fc_delgmb[i], FC_DT_DELT);
        fc_set_delcdu(i, inc); /* ONESTO2S         */
        agc_sp_t aw  = (agc_sp_t) ((omega[i] < 0) ? -omega[i] : omega[i]);
        agc_dp_t num = agc_sp_mul(agc_lm_mp_hi(aw, omega[i]), FC_BIT11);
        agc_sp_t j   = jacc[i];
        agc_sp_t q   = (num >= 0) ? FC_DELERLIM : (agc_sp_t) -FC_DELERLIM;
        if (j > 0 && ((num < 0) ? -num : num) < (agc_dp_t) j * 16384)
        {
            q = agc_lm_dv(num, j);
        }
        lag[i] = agc_lm_limitsub(q, FC_DELERLIM);
    }
    agc_erase_set_delperor(lag[0]);
    agc_erase_set_delqeror(lag[1]);
    agc_erase_set_delreror(lag[2]);
    (void) AGC_ASSERT(lag[0] >= -FC_DELERLIM && lag[0] <= FC_DELERLIM);
}

/* FETCH INPUTS and the thrust direction filter (pp. 912-913). Returns
 * NOT_FOUND when an entering command vector cannot be unitized.         */
static agc_status_t fc_inputs(agc_interp_t *ip, agc_vec_t unfc, agc_vec_t *unx,
                              agc_vec_t *unz, agc_cdutrig_t *trig)
{
    uint16_t yzx[3];
    fc_fetch_basic(yzx);
    int shift = 0;
    *unx      = agc_rtb_normunit(ip, unfc, &shift, NULL, NULL);
    *unz      = agc_rtb_normunit(ip, agc_lm_ldg()->unwc_2, &shift, NULL, NULL);
    agc_status_t st = agc_guid_quictrig(trig, yzx);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }
    if (agc_interp_test_ovf(ip))
    {
        return AGC_STATUS_NOT_FOUND; /* BOVB NOATTCNT    */
    }
    agc_vec_t u = agc_vec_unit(ip, agc_erase_delv(), NULL, NULL);
    if (agc_interp_test_ovf(ip))
    {
        return AGC_STATUS_OK; /* skip the filter  */
    }
    st = agc_guid_smnb(trig, &u); /* unit DELV, NB    */
    (void) AGC_ASSERT(st == AGC_STATUS_OK);
    fc_unfv_2.c[1] = agc_dp_from_sp(
        fc_fltrsub(agc_dp_high(fc_unfv_2.c[1]), agc_dp_high(u.c[1])));
    fc_unfv_2.c[2] = agc_dp_from_sp(
        fc_fltrsub(agc_dp_high(fc_unfv_2.c[2]), agc_dp_high(u.c[2])));
    return AGC_STATUS_OK;
}

/* AFTRFLTR (p. 913): a suitable window pointing vector. */
static agc_vec_t fc_window(agc_interp_t *ip, agc_vec_t unx, agc_vec_t unz)
{
    const agc_mat_t *xnb = &agc_guid_state()->lm.xnbpip;
    (void) AGC_ASSERT(ip != NULL);
    if (fc_flagoodw != 0 && fc_unwctest(ip, unz, unx))
    {
        return unz; /* window command   */
    }
    agc_vec_t znb = xnb->row[2]; /* FETCHZNB         */
    if (fc_unwctest(ip, znb, unx))
    {
        return znb;
    }
    agc_vec_t mx = agc_vec_neg(xnb->row[0]); /* Z and -X can't both be
                                                 parallel to UNFC/2     */
    (void) AGC_ASSERT(mx.c[0] >= AGC_DP_MIN);
    return mx;
}

/* FINDCDUW (pp. 910-918), entry with the thrust command in MPAC.
 * Normal interpretive return in all cases.                              */
agc_status_t agc_lm_findcduw(agc_interp_t *ip, agc_vec_t unfc_2)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    (void) agc_interp_test_ovf(ip);                /* BOV: clear       */
    agc_cdutrig_t *trig = &agc_guid_state()->trig; /* debris: SIN/COSCDU */
    agc_vec_t      unx;
    agc_vec_t      unz;
    agc_status_t   st = fc_inputs(ip, unfc_2, &unx, &unz, trig);
    if (st == AGC_STATUS_NOT_FOUND)
    {
        return fc_noattcnt(true);
    }
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    agc_mat_t m = fc_dcmcl(ip, unx, fc_window(ip, unx, unz));
    agc_sp_t  cdu[3];
    st = fc_nb2cdusp(ip, &m, cdu);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }
    fc_gimbal_changes(cdu);
    if (fc_flpautno != 0 || !agc_flag_test(AGC_FLAG_ENGONFLG))
    {
        agc_relint();
        return fc_noattcnt(false); /* not PNGCS auto / engine off  */
    }
    fc_limit_attitude(trig);
    fc_rates(trig);
    fc_transfer();
    agc_relint(); /* TC INTPRET after TCQCDUW     */
    return AGC_STATUS_OK;
}

/* FINDCDUW -2: the thrust command is already in UNFC/2. */
agc_status_t agc_lm_findcduw_m2(agc_interp_t *ip)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_vec_t unfc = agc_lm_ldg()->unfc_2; /* VLOAD UNFC/2     */
    (void) AGC_ASSERT(unfc.c[0] >= AGC_DP_MIN);
    return agc_lm_findcduw(ip, unfc);
}
