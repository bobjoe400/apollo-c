/**********************************************************************
 * FILE NAME: findcduw.c
 *
 * PURPOSE:
 *   FINDCDUW--GUIDAP_INTERFACE.agc (Luminary 099, pp. 908-925):
 *   program FINDCDUW, mod 1 (68-07-15), author Klumpp.
 *
 *   FINDCDUW is the interface between the powered-flight guidance programs
 *   and the digital autopilot. Its inputs are the thrust command vector
 *   UNFC/2 and the window command vector UNWC/2; its outputs are the gimbal
 *   angle increments DELCDUX/Y/Z, the commanded attitude rates OMEGAPD/QD/RD
 *   and the attitude lag angles DELPEROR/QEROR/REROR (the angles by which the
 *   body lags a ramp command because of the finite angular accelerations).
 *   It aligns the estimated thrust vector from the thrust direction filter
 *   with the thrust command and, when X-axis override is inhibited
 *   (XOVINHIB), the +Z half of the LM ZX plane with the window command.
 *   CPHI/CTHETA/CPSI (THETAD) receive the commanded gimbal angles for N22.
 *
 *   Alarms: 00401 if the inputs call for an attitude in gimbal lock (CDUXD
 *   and CDUYD are driven to the required values, CDUZD only to the 70 degree
 *   cone); 00402 if UNFC/2 or UNWC/2 overflow in NORMUNIT (a STOPRATE is then
 *   the only autopilot input).
 *
 *   Number representation. C integers hold the mathematical value of a word.
 *   A ones' complement value and a two's complement value of the same number
 *   are therefore the same agc_sp_t; the representation only matters where
 *   the AGC does modular (MSU) arithmetic on 15-bit words or reads a two's
 *   complement word as ones' complement. Those places use the word helpers
 *   below and are marked. Angles "2's, pi" are two's complement, scaled pi
 *   radians (16384 = pi), range -16384 .. 16383.
 *
 * AGC SOURCE:
 *   Luminary099 FINDCDUW--GUIDAP_INTERFACE.agc, pp. 908-925
 *
 * EXTERNAL VARIABLES:
 *   module statics fc_unfv_2, fc_ndxcduw, fc_flpautno, fc_flagoodw,
 *   fc_delgmb (the ECDUW block); agc_lm_ldg()->unwc_2, unfc_2;
 *   agc_guid_state()->trig, lm.xnbpip
 *
 * EXTERNAL REFERENCES:
 *   agc_rtb_normunit, agc_imu_quictrig, agc_imu_smnb,
 *   agc_lm_dap_stoprate, agc_erase_set_thetad,
 *   agc_erase_set_delcdux/y/z, agc_erase_set_omegapd/qd/rd,
 *   agc_erase_set_delperor/qeror/reror, agc_alarm
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   alarm 00401 (gimbal lock: CDUZ limited to 70 degrees); alarm 00402
 *   (UNFC/2 or UNWC/2 overflow in NORMUNIT, then STOPRATE)
 **********************************************************************/
#include <stdlib.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

/* ---- constants (p. 925) ------------------------------------------------ */
static const agc_sp_t fc_gainfltr[2] = {
    3277, /* GAINFLTR DEC .2  gain of filter without CSM               */
    1638  /*          DEC .1  gain of filter with CSM docked           */
};
#define FC_DUNFVLIM 57   /* DEC .007 B-1: 7 mr max change in 2 s       */
#define FC_UNFVLIM  1057 /* DEC .129 B-1: 129 mr max thrust offset     */

/* DOTSWFMX DEC .93302 B-4 = cos^2(15 deg)/16, whose low word is the next
 * constant DAXMAX (DEC .11111111111): the DP limit on (UNWC.UNFC)^2.     */
#define FC_DOTSWFMX_HI 955  /* DOTSWFMX high word                    */
#define FC_DOTSWFMX_LO 1820 /* its low word = DAXMAX's high word      */
static const agc_sp_t fc_daxmax[2]       = {
          /* DAXMAX = DAZMAX, 1's, pi     */
    1820, /* DEC .11111111111  20 deg in 2 s                          */
    182   /* DEC .0111111111   2 deg when CSM docked                  */
      };
static const agc_sp_t fc_day_half_max[2] = {
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
#define FC_HALF      8192     /* HALF: pi/2 as a 2's, pi angle            */

/* SPARCSIN polynomial (Bob Crisp), argument scaled 1, angle scaled 180 deg */
#define FC_DPL1 10502    /* SPARCSIN coefficient of x                */
#define FC_DPL3 432      /* of x^3                                   */
#define FC_DPL5 7300     /* of x^5                                   */
#define FC_DPL7 (-11803) /* of x^7                                  */
#define FC_DPL9 8397     /* of x^9                                   */

/* ---- erasables of the ECDUW block (ERASABLE_ASSIGNMENTS, "overlaying
 * KALCMANU storage"); module private.                                    */
/* UNFV/2: thrust direction in vehicle axes; only Y and Z are filtered. */
static agc_vec_t fc_unfv_2;   /* UNFV/2, half unit 2^1                   */
static agc_sp_t  fc_ndxcduw;  /* NDXCDUW 1 if CSM docked, else 0, 2^14 */
static agc_sp_t  fc_flpautno; /* FLPAUTNO non-zero = not PNGCS auto, 2^14 */
static agc_sp_t  fc_flagoodw; /* FLAGOODW non-zero = window good, 2^14 */

/* -DELGMB X Y Z: minus the gimbal angle changes (later -DELATT, then
 * limited).                                                             */
static agc_sp_t fc_delgmb[3]; /* -DELGMB, 1's, scale pi (2^0 pi)        */

/* ---- word-level helpers ------------------------------------------------ */

/**********************************************************************
 * AGC: none (MSU, FINDCDUW--GUIDAP_INTERFACE.agc pp. 914-922) -- wrap a
 * difference of 15-bit two's complement words as MSU does
 *
 * ARGUMENTS:
 *   d  int32_t  I    difference of two words, -2^15 < d < 2^15
 *
 * RETURNS: agc_sp_t -- the 15-bit difference, -16384..16383
 **********************************************************************/
static agc_sp_t fc_wrap15(int32_t d)
{
    int32_t w = d & 0x7FFF; /* 15-bit word, 0..077777 */

    if (w >= 0x4000)
    {
        w -= 0x8000;
    }

    return ((agc_sp_t) w);
}

/**********************************************************************
 * AGC: none (FINDCDUW--GUIDAP_INTERFACE.agc p. 916) -- the 15-bit word of
 * a ones' complement value, i.e. what MSU reads as two's complement
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I    ones' complement value, any scale 2^S
 *
 * RETURNS: int32_t -- the 15-bit word, 0..077777
 **********************************************************************/
static int32_t fc_ones_word(agc_sp_t v)
{
    int32_t w = (v < 0) ? 0x7FFF + (int32_t) v : (int32_t) v; /* 1's bits */

    return (w);
}

/**********************************************************************
 * AGC: none (MP then DDOUBL, FINDCDUW--GUIDAP_INTERFACE.agc pp. 916-917)
 * -- high word of twice the DP product, stored as the AGC stores an
 * overflowed A (overflow-corrected, i.e. wrapped)
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I    multiplicand, scale 2^A
 *   b  agc_sp_t  I    multiplier, scale 2^B
 *
 * RETURNS: agc_sp_t -- high word of 2 a b, scale 2^(A+B+1)
 **********************************************************************/
static agc_sp_t fc_mp_dbl(agc_sp_t a, agc_sp_t b)
{
    int32_t p  = 2 * (int32_t) a * (int32_t) b; /* 2 a b, 2^-28 units */
    int32_t hi = p / 16384;                     /* high word, 2^(A+B+1) */

    if (!AGC_ASSERT(hi <= AGC_SP_MAX && hi >= AGC_SP_MIN))
    { /* overflow-corrected store of the doubled A                  */
        hi %= 16384;
    }

    return ((agc_sp_t) hi);
}

/**********************************************************************
 * ONESTO2S (p. 924): ones' complement SP -> two's complement 15-bit word.
 * (CCS A / AD ONE / CS A: the value is unchanged, only the bit pattern of
 * a negative number moves by one.)
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I    ones' complement word, any scale 2^S
 *
 * RETURNS: uint16_t -- the two's complement 15-bit word
 **********************************************************************/
uint16_t agc_lm_onesto2s(agc_sp_t a)
{
    uint16_t w = (uint16_t) ((int32_t) a & 0x7FFF); /* 15-bit two's complement word */

    return (w);
}

/**********************************************************************
 * LIMITSUB (p. 924, "version courtesy Hugh Blair-Smith"): limit the
 * magnitude of x to the positive limit; DV by the limit gives a non-zero
 * quotient exactly when |x| >= limit.
 *
 * ARGUMENTS:
 *   x      agc_sp_t  I    value to limit, scale 2^S
 *   limit  agc_sp_t  I    positive limit, same scale 2^S
 *
 * RETURNS: agc_sp_t -- x limited to +-limit, scale 2^S
 **********************************************************************/
agc_sp_t agc_lm_limitsub(agc_sp_t x, agc_sp_t limit)
{
    if (x >= limit)
    {
        return (limit);
    }

    if (x <= (agc_sp_t) -limit)
    {
        return ((agc_sp_t) -limit);
    }

    return (x);
}

/**********************************************************************
 * SPARCSIN (p. 923, Bob Crisp): argument scaled 1, angle scaled 180 deg,
 * max error +-5 bits in +-.94. SR receives x/2 (writing SR shifts it).
 *
 * ARGUMENTS:
 *   x  agc_sp_t  I    sine, scale 2^0
 *
 * RETURNS: agc_sp_t -- arcsin(x), half-revolutions (180 deg) 2^0
 **********************************************************************/
agc_sp_t agc_lm_sparcsin(agc_sp_t x)
{
    agc_sp_t sr = (agc_sp_t) (x / 2);        /* TS SR: x/2, scale 2^1 */
    agc_sp_t x2 = agc_lm_mp_hi(x, x);        /* MP A, TS TEM1: x^2, scale 2^0 */
    agc_sp_t p  = agc_lm_mp_hi(x2, FC_DPL9); /* Horner sum, scale 2^0 */

    p          = agc_lm_mp_hi(agc_lm_ad(p, FC_DPL7), x2);
    p          = agc_lm_mp_hi(agc_lm_ad(p, FC_DPL5), x2);
    p          = agc_lm_mp_hi(agc_lm_ad(p, FC_DPL3), x2);
    agc_sp_t r = agc_lm_mp_hi(agc_lm_ad(p, FC_DPL1), sr); /* arcsin(x), half-revolutions 2^0 */

    return (r);
}

/**********************************************************************
 * SPARCSIN -1 (p. 923): argument scaled 2; DOUBLE, and on overflow TS SR
 * skips and INDEX A / CS LIMITS picks +-POSMAX (LIMITS-1 = POSMAX,
 * LIMITS+1 = BIT15).
 *
 * ARGUMENTS:
 *   x2  agc_sp_t  I    sine, scale 2^1
 *
 * RETURNS: agc_sp_t -- arcsin(2 x2), half-revolutions 2^0
 **********************************************************************/
agc_sp_t agc_lm_sparcsin_doubled(agc_sp_t x2)
{
    int32_t d = 2 * (int32_t) x2; /* 2 x2, scale 2^0 */

    if (d > AGC_SP_MAX)
    {
        d = AGC_SP_MAX;
    }
    else if (d < AGC_SP_MIN)
    {
        d = AGC_SP_MIN;
    }

    return (agc_lm_sparcsin((agc_sp_t) d));
}

/**********************************************************************
 * ARCTRGSP (p. 922): two's complement angle (pi) from sine in A and cosine
 * in L, both in units of 2; unambiguous anywhere in the circle, +-4 bits.
 * SPARCSIN is used only within +-45 degrees.
 *
 * ARGUMENTS:
 *   s  agc_sp_t  I    sine of the angle, scale 2^1
 *   c  agc_sp_t  I    cosine of the angle, scale 2^1
 *
 * RETURNS: agc_sp_t -- the angle, 2's complement, scale pi
 **********************************************************************/
agc_sp_t agc_lm_arctrgsp(agc_sp_t s, agc_sp_t c)
{
    if (s == 0)
    { /* SINZERO           */
        return ((c >= 0) ? 0 : FC_NEGMAX_2S); /* SINZERO: 0 or pi */
    }

    int32_t as = abs(s); /* |sin|, 2^1 */
    int32_t ac = abs(c); /* |cos|, 2^1 */

    if (ac < as)
    { /* USECOS            */
        agc_sp_t a = agc_lm_ad(agc_lm_sparcsin_doubled((agc_sp_t) -c), FC_HALF); /* pi/2 - arcsin(|cos|), 2's scale pi */

        return ((s > 0) ? a : (agc_sp_t) -a); /* 1TO2&TCQ          */
    }

    agc_sp_t sn   = s; /* sine used, 2^1 */
    agc_sp_t base = 0; /* pi when cos < 0, else 0, 2's scale pi */

    if (c < 0)
    { /* angle = pi - arcsin(sin)              */
        sn   = (agc_sp_t) -s;
        base = FC_NEGMAX_2S;
    }

    agc_sp_t a = agc_lm_sparcsin_doubled(sn);   /* arcsin(sn), 2's scale pi */
    agc_sp_t r = fc_wrap15((int32_t) a - base); /* MSU TEM3: angle, 2's, scale pi */

    return (r);
} /* agc_lm_arctrgsp */

/**********************************************************************
 * DVBYCOSM (p. 921): divide a matrix element (already containing
 * cos(MGA)/2) by cos(MGA) in TEM5 when that cannot overflow; otherwise
 * leave TEM5 - |high word| (gimbal angles then random multiples of pi/2).
 *
 * ARGUMENTS:
 *   op    agc_dp_t  I    matrix element times cos(MGA), 2^1
 *   tem5  agc_sp_t  I    cos(MGA), scale 2^0, >= 0
 *
 * RETURNS: agc_sp_t -- op / cos(MGA), scale 2^1, or TEM5 - |op|
 *          when the divide would overflow
 **********************************************************************/
static agc_sp_t fc_dvbycosm(agc_dp_t op, agc_sp_t tem5)
{
    agc_sp_t hi   = agc_dp_high(op);          /* high word of op, 2^1 */
    int32_t  diff = (int32_t) tem5 - abs(hi); /* TEM5 - |high word|, 2^0 */

    if (diff <= 0)
    {
        return ((agc_sp_t) diff); /* TSL&TCQ           */
    }

    agc_sp_t q = agc_lm_dv(op, tem5); /* op / cos(MGA), 2^1 */

    return (q);
}

/**********************************************************************
 * NB2CDUSP (p. 920): two's complement (pi) SP CDU angles X, Y, Z from the
 * matrix whose rows are the half-unit nav base X, Y, Z vectors in stable
 * member axes (push list 0, 6, 12).
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *     I/O  interpreter state (OVFIND)
 *   nb   const agc_mat_t *  I    rows: half-unit nav base X, Y, Z in
 *                                stable member axes, 2^1
 *   cdu  agc_sp_t 3         O    CDU angles X, Y, Z, 2's, scale pi
 *
 * RETURNS: none
 **********************************************************************/
static void fc_nb2cdusp(agc_interp_t *ip, const agc_mat_t *nb, agc_sp_t cdu[3])
{
    /* DSQ leaves a TP; BDSU DP1/4TH complements only MPAC, MPAC+1, so
     * MPAC+2 keeps the square's low word with its own sign, and SQRT
     * reads all three words: 1/4 - top(s^2) + low(s^2), at 2^2.      */
    agc_dp_t sq   = agc_dp_sq(nb->row[0].c[1]);                                                       /* top words of DSQ, 2^2 */
    agc_tp_t sqtp = agc_dp_sq_tp(nb->row[0].c[1]);                                                    /* DSQ as a TP, 2^2 */
    agc_tp_t c2   = agc_tp_from_dp(agc_dp_sub(ip, AGC_DP_QUARTER, sq)) + (sqtp - agc_tp_from_dp(sq)); /* as SQRT sees it, 2^2 */

    if (agc_tp_to_dp(c2) < 0)
    {
        c2 = 0; /* ZEROVECS: sin slightly > 1/2       */
    }

    agc_dp_t cos2 = agc_tp_sqrt(c2); /* cos(CDUZ) in units of 2, 2^1 */
    agc_sp_t tem5 = AGC_SP_MAX;      /* DDOUBL overflow -> POSMAX, cos 2^0 */

    if (cos2 < AGC_DP_HALF)
    {
        tem5 = agc_dp_high(2 * cos2); /* cos(CDUZ) in units of 1      */
    }

    cdu[2]      = agc_lm_arctrgsp(agc_dp_high(nb->row[0].c[1]), agc_dp_high(cos2));
    agc_sp_t cy = fc_dvbycosm(nb->row[0].c[0], tem5); /* cos(CDUY), 2^1 */
    agc_sp_t sy = fc_dvbycosm(nb->row[0].c[2], tem5); /* sin(CDUY), 2^1 */

    cdu[1]      = agc_lm_arctrgsp((agc_sp_t) -sy, cy); /* CDUY             */
    agc_sp_t cx = fc_dvbycosm(nb->row[1].c[1], tem5);  /* cos(CDUX), 2^1 */
    agc_sp_t sx = fc_dvbycosm(nb->row[2].c[1], tem5);  /* sin(CDUX), 2^1 */

    cdu[0] = agc_lm_arctrgsp((agc_sp_t) -sx, cx); /* CDUX             */
} /* fc_nb2cdusp */

/**********************************************************************
 * FLTRSUB (p. 919): one component of the thrust direction filter. The
 * offset moves toward the measurement by GAINFLTR, at most DUNFVLIM per
 * pass, and the total is limited to UNFVLIM.
 *
 * ARGUMENTS:
 *   orig  agc_sp_t  I    filtered offset UNFVY/2 or UNFVZ/2,
 *                        high word, scale 2^1
 *   meas  agc_sp_t  I    measured unit DELV component, NB,
 *                        high word, scale 2^1
 *
 * RETURNS: agc_sp_t -- new filtered offset, |.| <= UNFVLIM, 2^1
 **********************************************************************/
static agc_sp_t fc_fltrsub(agc_sp_t orig, agc_sp_t meas)
{
    agc_sp_t err  = agc_lm_ad(meas, (agc_sp_t) -orig);          /* error, 2^1    */
    agc_sp_t incr = agc_lm_mp_hi(err, fc_gainfltr[fc_ndxcduw]); /* 2^1 */

    incr           = agc_lm_limitsub(incr, FC_DUNFVLIM);
    agc_sp_t total = agc_lm_limitsub(agc_lm_ad(incr, orig), FC_UNFVLIM); /* new offset, 2^1 */

    return (total);
}

/**********************************************************************
 * UNWCTEST (p. 919): is the proposed window vector at least 15 degrees
 * from the thrust command? If not, the window-good flag is zeroed.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  I/O  interpreter state (OVFIND)
 *   unz  agc_vec_t       I    proposed window vector, half unit 2^1
 *   unx  agc_vec_t       I    thrust command, half unit 2^1
 *
 * RETURNS: bool -- true when the window vector is usable
 **********************************************************************/
static bool fc_unwctest(agc_interp_t *ip, agc_vec_t unz, agc_vec_t unx)
{
    agc_dp_t d   = agc_dp_sq(agc_vec_dot(ip, unz, unx));        /* (UNZ . UNX)^2, 2^2 */
    agc_dp_t lim = agc_dp_pack(FC_DOTSWFMX_HI, FC_DOTSWFMX_LO); /* DOTSWFMX: cos^2(15 deg)/16, 2^2 */

    if (agc_dp_sub(ip, d, lim) < 0)
    {
        return (true); /* BMN DCMCL        */
    }

    fc_flagoodw = 0; /* SSP FLAGOODW 0   */

    return (false);
}

/**********************************************************************
 * INITCDUW (p. 910): one call before each guided maneuver.
 **********************************************************************/
void agc_lm_initcduw(void)
{
    agc_vec_t unitx = agc_vec_make(AGC_DP_HALF, 0, 0); /* UNITX, half unit 2^1 */

    fc_unfv_2            = unitx;
    agc_lm_ldg()->unwc_2 = unitx;
}

/**********************************************************************
 * Housekeeping and FETCH BASIC DATA (pp. 910-911): index, flags, and the
 * CDUs (or, in PNGCS auto, the desired CDUs) for the trig.
 *
 * ARGUMENTS:
 *   yzx  uint16_t 3  O    CDU (or CDUxD) angles Y, Z, X, 2's,
 *                         scale pi
 **********************************************************************/
static void fc_fetch_basic(uint16_t yzx[3])
{
    fc_ndxcduw = agc_flag_test(AGC_FLAG_CSMDKFLG) ? 1 : 0; /* docked gains */
    bool xov   = agc_flag_test(AGC_FLAG_XOVINFLG);         /* X-axis override inhibited */

    fc_flpautno = xov ? 1 : 0; /* set to PNZ flag "PNGCS auto not"  */
    fc_flagoodw = xov ? 1 : 0; /* PNZ if X override inhibited      */
    agc_inhint();              /* RELINT at PAUTNO                 */
    yzx[0]      = agc_erase_cduy();
    yzx[1]      = agc_erase_cduz();
    yzx[2]      = agc_erase_cdux();

    if (agc_lm_chan_bit_present(AGC_CHAN_CHAN30, AGC_CH030_LMCONTROL) && agc_lm_chan_bit_present(AGC_CHAN_CHAN31, AGC_CH031_AUTOSTAB))
    {
        fc_flpautno = 0; /* PNGCS auto: use CDUXD,YD,ZD      */
        yzx[0]      = agc_erase_cduyd();
        yzx[1]      = agc_erase_cduzd();
        yzx[2]      = agc_erase_cduxd();
    }

    agc_relint();
}

/**********************************************************************
 * NOATTCNT (p. 924): STOPRATE replaces the autopilot commands.
 *
 * ARGUMENTS:
 *   alarm  bool  I    true: alarm 00402 (NORMUNIT overflow)
 *
 * RETURNS: none
 **********************************************************************/
static void fc_noattcnt(bool alarm)
{
    if (alarm)
    {
        agc_alarm(AGC_ALARM_00402); /* alarm is recorded; go on  */
    }

    agc_inhint();
    agc_lm_dap_stoprate();
    agc_relint();
}

/**********************************************************************
 * Limit the MGA (alarm 00401), output CPHI.. for N22, and form the
 * unlimited -DELGMB = CDUxD - required angle (pp. 914-915, MGARET loop).
 *
 * ARGUMENTS:
 *   cdu  agc_sp_t 3  I/O  required CDU angles X, Y, Z, 2's, pi;
 *                         CDUZ limited to the 70 degree cone
 **********************************************************************/
static void fc_gimbal_changes(agc_sp_t cdu[3])
{
    /* Value of a two's complement word when the AGC reads it as ones'. */

    agc_sp_t cduz_1s = cdu[2]; /* CDUZ read as ones' complement, scale pi */

    if (cdu[2] < 0)
    {
        cduz_1s = (agc_sp_t) (cdu[2] + 1);
    }

    agc_sp_t lim = agc_lm_limitsub(cduz_1s, FC_CDUZDLIM); /* 1's, scale pi */

    /* Value of a ones' complement word when the AGC reads it as two's. */

    agc_sp_t limited = lim; /* back to two's complement, scale pi */

    if (lim < 0)
    {
        limited = (agc_sp_t) (lim - 1);
    }

    if (fc_wrap15((int32_t) cdu[2] - limited) != 0)
    {
        agc_alarm(AGC_ALARM_00401); /* ALARMMGA         */
    }

    cdu[2]                 = limited;
    const uint16_t cdud[3] = {agc_erase_cduxd(), agc_erase_cduyd(), agc_erase_cduzd()};

    agc_inhint();   /* RELINT after TCQCDUW         */
    agc_sp_t l = 0; /* last -DELGMB, 1's, scale pi */

    for (int i = 2; i >= 0; i--)
    { /* DELGMBLP, bound 3            */
        if (agc_lm_ad(agc_lm_mp_hi(l, l), FC_HI5) > 0)
        {
            fc_flagoodw = 0; /* DELGMBZ or Y beyond 45 deg   */
        }

        agc_erase_set_thetad(i, agc_lm_onesto2s(cdu[i]));
        agc_sp_t d = fc_wrap15((int32_t) cdu[i] - (int32_t) cdud[i]); /* MSU CDUxD: required - desired, 2's, scale pi */

        fc_delgmb[i] = (agc_sp_t) -d; /* MSU CDUXD, COM               */
        l            = fc_delgmb[i];
    }
} /* fc_gimbal_changes */

/**********************************************************************
 * Commanded attitude rates (p. 917), pi/4 rad/s, for a 2 s period.
 *
 * ARGUMENTS:
 *   trig  const agc_cdutrig_t *  I    CDU sines and cosines (QUICTRIG), 2^1
 *
 * RETURNS: agc_status_t -- OK; BAD_ARG when the DAP refuses a rate
 **********************************************************************/
static agc_status_t fc_rates(const agc_cdutrig_t *trig)
{
    agc_sp_t sinz = agc_dp_high(trig->sincdu[AGC_TRIG_Z]); /* sin(CDUZ), 2^1 */
    agc_sp_t cosz = agc_dp_high(trig->coscdu[AGC_TRIG_Z]); /* cos(CDUZ), 2^1 */
    agc_sp_t sinx = agc_dp_high(trig->sincdu[AGC_TRIG_X]); /* sin(CDUX), 2^1 */
    agc_sp_t cosx = agc_dp_high(trig->coscdu[AGC_TRIG_X]); /* cos(CDUX), 2^1 */
    agc_sp_t gx   = (agc_sp_t) -fc_delgmb[0];              /* DELGMBX, 1's, scale pi */
    agc_sp_t gy   = (agc_sp_t) -fc_delgmb[1];              /* DELGMBY, 1's, scale pi */
    agc_sp_t gz   = (agc_sp_t) -fc_delgmb[2];              /* DELGMBZ, 1's, scale pi */
    agc_sp_t p    = agc_lm_ad(gx, fc_mp_dbl(gy, sinz));    /* OMEGAPD before doubling, scale pi/4 rad/s */

    p          = agc_lm_ad(p, p);                         /* two ADS          */
    agc_sp_t q = agc_lm_mp_hi(fc_mp_dbl(gy, cosx), cosz); /* OMEGAQD partial, scale pi/4 rad/s */

    q          = agc_lm_ad(q, agc_lm_mp_hi(gz, sinx));
    q          = agc_lm_ad(q, q);
    q          = agc_lm_ad(q, q);                                     /* three ADS        */
    agc_sp_t r = agc_lm_mp_hi(fc_mp_dbl((agc_sp_t) -gy, sinx), cosz); /* OMEGARD partial, scale pi/4 rad/s */

    r = agc_lm_ad(r, agc_lm_mp_hi(gz, cosx));
    r = agc_lm_ad(r, r);
    r = agc_lm_ad(r, r);

    agc_status_t st = agc_erase_set_omegapd(p); /* OMEGAPD.. store status */

    st = (st == AGC_STATUS_OK) ? agc_erase_set_omegaqd(q) : st; /* if OK */
    st = (st == AGC_STATUS_OK) ? agc_erase_set_omegard(r) : st; /* if OK */

    return (st);
}

/**********************************************************************
 * CDUWXFR (p. 918): angle interface and lag angles
 * DELxEROR = OMEGA |OMEGA| / (2 * 1JACC), limited to DELERLIM.
 *
 * RETURNS: agc_status_t -- OK; BAD_ARG when the DAP refuses a word
 **********************************************************************/
static agc_status_t fc_transfer(void)
{
    const agc_sp_t omega[3] = {agc_erase_omegapd(), agc_erase_omegaqd(), agc_erase_omegard()};
    const agc_sp_t jacc[3]  = {agc_erase_1jacc(), agc_erase_1jaccq(), agc_erase_1jaccr()};
    agc_sp_t       lag[3]   = {0, 0, 0};     /* DELPEROR..DELREROR, 1's, scale pi */
    agc_status_t   st       = AGC_STATUS_OK; /* DELCDUX.. store status */

    for (int i = 2; i >= 0 && st == AGC_STATUS_OK; i--)
    { /* bound 3          */
        agc_sp_t inc = agc_lm_mp_hi(fc_delgmb[i], FC_DT_DELT); /* DELCDUx for 0.1 s, 1's, scale pi */

        /* ONESTO2S         */
        /* DELCDUX, DELCDUY, DELCDUZ by index (INDEX TEM2 / TS DELCDUX). */

        if (i == 0)
        {
            st = agc_erase_set_delcdux(inc);
        }
        else if (i == 1)
        {
            st = agc_erase_set_delcduy(inc);
        }
        else
        {
            st = agc_erase_set_delcduz(inc);
        }

        agc_sp_t aw  = (agc_sp_t) abs(omega[i]);                         /* |OMEGA|, pi/4 rad/s 2^0 */
        agc_dp_t num = agc_sp_mul(agc_lm_mp_hi(aw, omega[i]), FC_BIT11); /* OMEGA |OMEGA| / 16, 2^-28 units */
        agc_sp_t j   = jacc[i];                                          /* 1/JACC for this axis, 2^14 */
        agc_sp_t q   = FC_DELERLIM;                                      /* DELERLIM, sign of num, scale pi */

        if (num < 0)
        {
            q = (agc_sp_t) -FC_DELERLIM;
        }

        if (j > 0 && abs(num) < (agc_dp_t) j * 16384) /* DV fits     */
        {
            q = agc_lm_dv(num, j);
        }

        lag[i] = agc_lm_limitsub(q, FC_DELERLIM);
    } /* for each axis, Z Y X */

    st = (st == AGC_STATUS_OK) ? agc_erase_set_delperor(lag[0]) : st; /* if OK */
    st = (st == AGC_STATUS_OK) ? agc_erase_set_delqeror(lag[1]) : st; /* if OK */
    st = (st == AGC_STATUS_OK) ? agc_erase_set_delreror(lag[2]) : st; /* if OK */

    return (st);
} /* fc_transfer */

/**********************************************************************
 * FETCH INPUTS and the thrust direction filter (pp. 912-913). Returns
 * NOT_FOUND when an entering command vector cannot be unitized.
 *
 * ARGUMENTS:
 *   ip    agc_interp_t *   I/O  interpreter state (OVFIND)
 *   unfc  agc_vec_t        I    thrust command UNFC/2, any length 2^1
 *   unx   agc_vec_t *      O    unit thrust command, half unit 2^1
 *   unz   agc_vec_t *      O    unit window command, half unit 2^1
 *   trig  agc_cdutrig_t *  O    CDU sines and cosines, 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK; AGC_STATUS_NOT_FOUND when NORMUNIT
 *          of a command vector overflows (BOVB NOATTCNT)
 **********************************************************************/
static agc_status_t fc_inputs(agc_interp_t *ip, agc_vec_t unfc, agc_vec_t *unx, agc_vec_t *unz, agc_cdutrig_t *trig)
{
    uint16_t yzx[3]; /* CDU angles Y, Z, X, 2's, pi */

    fc_fetch_basic(yzx);
    int shift = 0; /* NORMUNIT shift count (unused) */

    *unx = agc_rtb_normunit(ip, unfc, &shift, NULL, NULL);
    *unz = agc_rtb_normunit(ip, agc_lm_ldg()->unwc_2, &shift, NULL, NULL);
    agc_imu_quictrig(trig, yzx);

    if (agc_interp_test_ovf(ip))
    {
        return (AGC_STATUS_NOT_FOUND); /* BOVB NOATTCNT    */
    }

    agc_vec_t u = agc_vec_unit(ip, agc_erase_delv(), NULL, NULL); /* unit DELV, SM, half unit 2^1 */

    if (!agc_interp_test_ovf(ip)) /* skip the filter  */
    {
        agc_imu_smnb(trig, &u); /* unit DELV, NB    */
        fc_unfv_2.c[1] = agc_dp_from_sp(fc_fltrsub(agc_dp_high(fc_unfv_2.c[1]), agc_dp_high(u.c[1])));
        fc_unfv_2.c[2] = agc_dp_from_sp(fc_fltrsub(agc_dp_high(fc_unfv_2.c[2]), agc_dp_high(u.c[2])));
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AFTRFLTR (p. 913): a suitable window pointing vector.
 *
 * ARGUMENTS:
 *   ip   agc_interp_t *  I/O  interpreter state (OVFIND)
 *   unx  agc_vec_t       I    thrust command, half unit 2^1
 *   unz  agc_vec_t       I    window command, half unit 2^1
 *
 * RETURNS: agc_vec_t -- the window pointing vector, half unit 2^1
 **********************************************************************/
static agc_vec_t fc_window(agc_interp_t *ip, agc_vec_t unx, agc_vec_t unz)
{
    const agc_mat_t *xnb = &agc_guid_state()->lm.xnbpip; /* XNBPIP..ZNBPIP rows, 2^1 */

    if (fc_flagoodw != 0 && fc_unwctest(ip, unz, unx))
    {
        return (unz); /* window command   */
    }

    agc_vec_t znb = xnb->row[2]; /* FETCHZNB: +Z body axis, half unit 2^1 */

    if (fc_unwctest(ip, znb, unx))
    {
        return (znb);
    }

    /* Z and -X cannot both be parallel to UNFC/2. */
    agc_vec_t mx = agc_vec_neg(xnb->row[0]); /* -X body, half unit 2^1 */

    return (mx);
}

/**********************************************************************
 * FINDCDUW (pp. 910-918), entry with the thrust command in MPAC.
 * Normal interpretive return in all cases.
 *
 * ARGUMENTS:
 *   ip      agc_interp_t *  I/O  interpreter state (OVFIND)
 *   unfc_2  agc_vec_t       I    thrust command UNFC/2 (MPAC), 2^1
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_findcduw(agc_interp_t *ip, agc_vec_t unfc_2)
{
    agc_interp_clear_ovf(ip);                                      /* BOV: clear       */
    agc_cdutrig_t *trig = agc_imu_trig();                          /* debris: SIN/COSCDU */
    agc_vec_t      unx;                                            /* unit thrust command, half unit 2^1 */
    agc_vec_t      unz;                                            /* unit window command, half unit 2^1 */
    agc_status_t   st   = fc_inputs(ip, unfc_2, &unx, &unz, trig); /* step status */

    if (st == AGC_STATUS_NOT_FOUND)
    {
        fc_noattcnt(true);

        return (AGC_STATUS_OK);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    /* DCMCL (p. 913): the required direction cosine matrix. The filtered
     * thrust offsets (UNFVY/2, UNFVZ/2) tilt X so that the estimated thrust,
     * not the X axis, lies along the command. */
    agc_vec_t unz_dcmcl = fc_window(ip, unx, unz); /* UNZ for DCMCL, half unit 2^1 */

    agc_vec_t uny1    = agc_vec_unit(ip, agc_vec_cross(ip, unz_dcmcl, unx), NULL, NULL); /* UNIT(UNZ x UNX), half unit 2^1 */
    agc_vec_t munz    = agc_vec_shl(ip, agc_vec_cross(ip, uny1, unx), 1);                /* -UNZ, half unit 2^1 */
    agc_vec_t a       = agc_vec_scale(ip, munz, fc_unfv_2.c[2]);                         /* UNFVZ/2 (-UNZ), 2^1 */
    agc_vec_t b       = agc_vec_scale(ip, uny1, fc_unfv_2.c[1]);                         /* UNFVY/2 UNY1, 2^1 */
    agc_vec_t t_dcmcl = agc_vec_shl(ip, agc_vec_sub(ip, a, b), 1);                       /* BVSU: thrust offset vector, 2^1 */
    agc_mat_t m;                                                                         /* required nav base rows X, Y, Z, 2^1 */

    m.row[0] = agc_vec_unit(ip, agc_vec_add(ip, t_dcmcl, unx), NULL, NULL);
    m.row[1] = agc_vec_shl(ip, agc_vec_cross(ip, m.row[0], munz), 1);
    m.row[2] = agc_vec_shl(ip, agc_vec_cross(ip, agc_vec_neg(m.row[1]), m.row[0]), 1);

    /* required nav base rows, 2^1 */
    agc_sp_t cdu[3]; /* required CDU angles X, Y, Z, 2's, scale pi */

    fc_nb2cdusp(ip, &m, cdu);

    fc_gimbal_changes(cdu);

    if (fc_flpautno != 0 || !agc_flag_test(AGC_FLAG_ENGONFLG))
    {
        agc_relint();

        fc_noattcnt(false);

        return (AGC_STATUS_OK); /* not PNGCS auto / engine off  */
    }

    /* Limit the attitude angle changes about X, Y', Z' (p. 916).            */
    agc_sp_t sinz = agc_dp_high(trig->sincdu[AGC_TRIG_Z]); /* sin(CDUZ), 2^1 */
    agc_sp_t cosz = agc_dp_high(trig->coscdu[AGC_TRIG_Z]); /* cos(CDUZ), 2^1 */
    int      n    = fc_ndxcduw;                            /* NDXCDUW: 1 if CSM docked */

    if (!AGC_ASSERT(n == 0 || n == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    fc_delgmb[2] = agc_lm_limitsub(fc_delgmb[2], fc_daxmax[n]); /* DAZMAX */
    agc_sp_t y2  = agc_lm_mp_hi(fc_delgmb[1], cosz);            /* -DELATTY'/2, scale pi */

    y2 = agc_lm_limitsub(y2, fc_day_half_max[n]);

    /* DEVIATION: DV takes (A,L); L after LIMITSUB is modelled as A (true
     * unless the value was limited negative, where L is a DV remainder). */
    agc_sp_t newy = AGC_SP_MAX; /* new -DELGMBY, 1's, scale pi (POSMAX on DV overflow) */

    if (cosz != 0 && abs(y2) < abs(cosz)) /* DV cannot overflow     */
    {
        newy = agc_lm_dv(agc_dp_pack(y2, y2), cosz);
    }

    agc_sp_t oldy = fc_delgmb[1]; /* -DELGMBY before limiting, 1's, scale pi */

    fc_delgmb[1] = newy;                              /* -DELGMBY          */
    agc_sp_t t   = (agc_sp_t) -fc_mp_dbl(oldy, sinz); /* -DELATTX implied by Y, 1's, scale pi */

    /* MSU on ones' complement operands: one bit error if signs mixed.   */
    agc_sp_t datx = fc_wrap15(fc_ones_word(t) - fc_ones_word(fc_delgmb[0])); /* +DELATTX, 1's, scale pi (MSU on 1's words) */

    datx         = agc_lm_limitsub(datx, fc_daxmax[n]);       /* +DELATTX limited  */
    fc_delgmb[0] = (fc_flagoodw != 0) ? (agc_sp_t) -datx : 0; /* good window */
    agc_sp_t c   = fc_mp_dbl((agc_sp_t) -fc_delgmb[1], sinz); /* X change from the new Y, 1's, scale pi */

    fc_delgmb[0] = agc_lm_ad(fc_delgmb[0], c);                 /* ADS: -DELGMBX     */
    st           = fc_rates(trig);
    st           = (st == AGC_STATUS_OK) ? fc_transfer() : st; /* if OK */
    agc_relint();                                              /* TC INTPRET after TCQCDUW     */

    return (st);
} /* agc_lm_findcduw */

/**********************************************************************
 * FINDCDUW -2 (p. 910): the thrust command is already in UNFC/2.
 *
 * ARGUMENTS:
 *   ip  agc_interp_t *  I/O  interpreter state (OVFIND)
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or the status of the step that
 *          failed (AGC_STATUS_BAD_ARG on a failed precondition)
 **********************************************************************/
agc_status_t agc_lm_findcduw_from_unfc(agc_interp_t *ip)
{
    agc_vec_t unfc = agc_lm_ldg()->unfc_2; /* VLOAD UNFC/2, 2^1 */

    return (agc_lm_findcduw(ip, unfc));
}
