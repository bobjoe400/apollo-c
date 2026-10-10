/*
 * landing_displays.c -- LANDING_ANALOG_DISPLAYS.agc (Luminary 099,
 * pp. 898-907): R10, the landing analog displays. Four times a second
 * (from R10,R11) the altitude and altitude-rate meter (ALTM counter,
 * alternating rate and altitude) and, in the descent, the forward and
 * lateral velocity cross-pointers (CDUSCMD, CDUTCMD) are driven.
 *
 * Units: velocities in VVECT are m/cs at 2^5; ALTRATE and the cross-
 * pointer velocities are in display bit units (0.5 ft/s per bit for the
 * rate meter, 0.5571 ft/s per bit for the cross-pointers); ALTSAVE is the
 * altitude in bit units as a two-word count.
 *
 * Task structure: DISINDAT is a subroutine of ALTROUT/ALTOUT that pauses
 * 40 ms (VARDELAY) in SPEEDRUN; the AGC keeps the return in LADQSAVE.
 * Here the pending path is the static ld_altpath and the task after the
 * pause is SPEEDRN2 (agc_lm_task_speedrun2).
 */
#include "agc/agc_assert.h"
#include "agc/agc_channels.h"
#include "agc/agc_lm_landing.h"
#include "landing_pending.h"

#define LD_ARCONV    10498     /* OCT 24402: 656.1679798 B-10, rate to bits */
#define LD_ARTOA     873       /* DEC .1066098 B-1: .25/2.345, 4x/s cycle   */
#define LD_ARTOA2    8943      /* DEC .0021322 B8: .5/(2.345)(100)          */
#define LD_VELCONV   9422      /* OCT 22316: 588.914 B-10, vel to bits      */
#define LD_KPIP1_5   839       /* KPIP1(5) DEC .0512: DELV to m/cs 2^5      */
#define LD_MAXVBITS  359       /* OCT 00547: 199.9989 ft/s                  */
#define LD_1SEC      100
#define LD_BIT7      0100u     /* IMODES33: 0 do ALTRATE, 1 do ALTITUDE     */
#define LD_BIT8      0200u     /* IMODES33: inertial data flag              */
#define LD_BITS8_7   0300u     /* BITS8/7: inertial data, interleave flags  */
#define LD_BIT15W    040000u   /* sign bit of the ALTM word                 */
#define LD_BIT14     020000

typedef enum ld_path { LD_NONE = 0, LD_ALTROUT, LD_ALTOUT } ld_path_t;

/* LANDING ANALOGS DISPLAY STORAGE (module private). */
static ld_path_t ld_altpath;     /* LADQSAVE: ALTROUT +1 or ALTOUT +1        */
static agc_sp_t  ld_dt;          /* DT  time since PIPTIME, cs (2^14 wrap)   */
static agc_sp_t  ld_vvect[3];    /* VVECT velocity now, m/cs 2^5 (high words) */
static agc_sp_t  ld_vhy;         /* VHY cross-range velocity, m/cs 2^5       */
static agc_sp_t  ld_vhz;         /* VHZ down-range velocity, m/cs 2^5        */
static agc_sp_t  ld_altrate;     /* ALTRATE bit units                        */
static agc_dp_t  ld_altsave;     /* ALTSAVE, +1: altitude count, bit units   */
static agc_sp_t  ld_vel[2];      /* LATVEL, FORVEL bit units                 */
static agc_sp_t  ld_metr[2];     /* LATVMETR, FORVMETR meter positions       */
static agc_sp_t  ld_trak[2];     /* TRAKLATV, TRAKFWDV saturation tracking   */

static uint16_t ld_imodes33_bits(uint16_t set, uint16_t clear)
{
    uint16_t w = (uint16_t)((agc_erase_imodes33() | set) & ~(unsigned)clear);
    agc_erase_set_imodes33(w);
    (void)AGC_ASSERT((w & set) == set);
    return w;
}

/* SP sum kept in A and stored (saturating). */
static agc_sp_t ld_sat(int32_t x)
{
    int32_t r = x;
    if (r > AGC_SP_MAX) {
        r = AGC_SP_MAX;
    } else if (r < AGC_SP_MIN) {
        r = AGC_SP_MIN;
    }
    (void)AGC_ASSERT(r >= AGC_SP_MIN && r <= AGC_SP_MAX);
    return (agc_sp_t)r;
}

/* MP then DDOUBL (n times): high word of 2^n a b. */
static agc_sp_t ld_mp_dbl(agc_sp_t a, agc_sp_t b, int n)
{
    (void)AGC_ASSERT(n >= 0 && n <= 2);
    int32_t p = (int32_t)a * (int32_t)b * (1 << n);
    return ld_sat(p / 16384);
}

/* DCA x, DDOUBL, DDOUBL: the high word of 4x (DP doubling carries the
 * low word up). DEVIATION: an overflow of A is kept wrapped, as the
 * following MP / ADS would see the overflow-corrected word.          */
static agc_sp_t ld_ddoubl2(agc_dp_t x)
{
    agc_ovf_t ovf = 0;
    agc_dp_t d = agc_dp_shl_raw(x, 2, &ovf);
    (void)AGC_ASSERT(ovf >= -1 && ovf <= 1);
    agc_sp_t hi = agc_dp_high(d);
    (void)AGC_ASSERT(hi >= AGC_SP_MIN && hi <= AGC_SP_MAX);
    return hi;
}

/* DISPRSET / ABORTON (p. 907): remove the inertial data display (and in
 * the descent the RR error counter enable), reset the R10 flags.      */
static agc_status_t ld_disprset(void)
{
    if (!agc_flag_test(AGC_FLAG_LANDING_ANALOG_DISPLAYS_FLAG)) {
        uint16_t m = AGC_CH012_DISPINERT;
        if ((agc_erase_imodes33() & LD_BIT8) != 0u) {
            m = (uint16_t)(m | AGC_CH012_ENBRRCDUERR);
        }
        agc_status_t st = agc_chan_and(AGC_CHAN_CHAN12, (uint16_t)(077777u & ~(unsigned)m));
        (void)AGC_ASSERT(st == AGC_STATUS_OK);
    }
    (void)ld_imodes33_bits(0u, LD_BITS8_7);              /* ABORTON        */
    (void)agc_flag_clear(AGC_FLAG_DIDFLAG);
    ld_altpath = LD_NONE;
    return AGC_STATUS_OK;                                /* TASKOVER       */
}

/* DATAOUT (p. 899): the word to the ALTM counter; drive the meter. */
static agc_status_t ld_dataout(uint16_t word)
{
    agc_erase_set_altm(word);
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN14, AGC_CH014_ALTMETERACT);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(word <= 077777u);
    ld_altpath = LD_NONE;
    return st;                                           /* TASKOVER       */
}

/* ALTROUT +1 / ARCOMP (p. 898): altitude rate = RUNIT . VVECT in bit
 * units plus the DALTRATE compensation; positive rates go out with the
 * sign bit set (never +0).                                          */
static agc_status_t ld_altrout(void)
{
    (void)ld_imodes33_bits(LD_BIT7, 0u);   /* altitude next time         */
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN14, AGC_CH014_ALTRATESEL);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    const agc_guid_lm_t *g = &agc_guid_state()->lm;
    int32_t r = 0;
    for (int i = 0; i < 3; i++) {
        r += agc_lm_mp_hi(g->runit[i], ld_vvect[i]);     /* m/cs 2^6       */
    }
    agc_sp_t rate = ld_mp_dbl(LD_ARCONV, ld_sat(r), 2);  /* bit units      */
    ld_altrate = agc_lm_ad(agc_lm_mp_hi(g->daltrate, ld_dt), rate);
    uint16_t w = (ld_altrate < 0) ? (uint16_t)(-ld_altrate)
                                  : (uint16_t)((uint16_t)ld_altrate | LD_BIT15W);
    (void)AGC_ASSERT(w != 0u || ld_altrate < 0);         /* never +0       */
    return ld_dataout(w);
}

/* OLDDATA .. NEWDATA (pp. 899-900): extrapolate ALTSAVE with ALTRATE by
 * `factor` (ARTOA for half a second, or DT for new data); no negative
 * altitudes; a 15-bit unsigned output.                              */
static agc_status_t ld_extrapolate(agc_sp_t factor)
{
    int32_t inc = ld_mp_dbl(factor, ld_altrate, 1);
    int32_t total = ld_altsave + inc;                    /* AD, carry      */
    if (total < 0) {
        total = 0;                                       /* ZERODATA       */
    }
    ld_altsave = total;
    int32_t hi = total / 16384;
    int32_t lo = total % 16384;
    (void)AGC_ASSERT(hi >= 0);
    uint16_t w = (uint16_t)(((hi > 0) ? LD_BIT15W : 0u) + (uint16_t)lo);
    (void)AGC_ASSERT(lo >= 0);
    return ld_dataout(w);
}

/* ALTOUT +1 (p. 899): altitude. New data from SERVICER (ALTBITS not
 * negative) restarts the extrapolation over DT; otherwise the old value
 * is carried on by half a second of ALTRATE.                         */
static agc_status_t ld_altout(void)
{
    (void)ld_imodes33_bits(0u, LD_BIT7);   /* altitude rate next time    */
    agc_status_t st = agc_chan_and(AGC_CHAN_CHAN14,
                                   (uint16_t)(077777u & ~(unsigned)AGC_CH014_ALTRATESEL));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_guid_lm_t *g = &agc_guid_state()->lm;
    if (agc_dp_high(g->altbits) < 0) {
        return ld_extrapolate(LD_ARTOA);                 /* OLDDATA        */
    }
    ld_altsave = g->altbits;                             /* new data       */
    (void)AGC_ASSERT(ld_altsave >= 0);
    g->altbits = agc_dp_from_sp(-1);     /* -1: extrapolate next pass     */
    agc_sp_t dt9 = agc_lm_dv((agc_dp_t)ld_dt * 16384, 512);   /* DV BIT10 */
    return ld_extrapolate(agc_lm_mp_hi(dt9, LD_ARTOA2));
}

/* Continue at LADQSAVE: ALTROUT +1 or ALTOUT +1. */
static agc_status_t ld_continue(void)
{
    if (ld_altpath == LD_ALTROUT) {
        return ld_altrout();
    }
    if (ld_altpath == LD_ALTOUT) {
        return ld_altout();
    }
    (void)AGC_ASSERT(ld_altpath == LD_NONE);
    return AGC_STATUS_OK;
}

/* SPEEDRUN (pp. 901-902): VVECT = V + G (T - TN) + DELV now, m/cs 2^5;
 * then a 40 ms pause to let other rupts in.                         */
static agc_status_t ld_speedrun(void)
{
    const agc_guid_t *g = agc_guid_state();
    int32_t d = (int32_t)agc_lm_time1() - agc_dp_low(agc_erase_piptime());
    ld_dt = (agc_sp_t)(d & 0x3FFF);                      /* AD HALF, HALF  */
    const agc_sp_t pipa[3] = { agc_erase_pipax(), agc_erase_pipay(),
                               agc_erase_pipaz() };
    for (int i = 0; i < 3; i++) {
        agc_sp_t g4 = ld_ddoubl2(g->avg.gdt_2.c[i]);     /* DCA, DDOUBL x2 */
        agc_sp_t gt = agc_lm_dv(agc_sp_mul(g4, ld_dt), LD_1SEC);
        agc_sp_t v4 = ld_ddoubl2(g->lm.v.c[i]);          /* m/cs 2^5       */
        agc_sp_t dv = agc_lm_mp_hi(agc_lm_ad(pipa[i], g->avg.pipatmp[i]), LD_KPIP1_5);
        ld_vvect[i] = agc_lm_ad(agc_lm_ad(gt, v4), dv);
    }
    agc_status_t st = agc_waitlist_call(4, AGC_TASK_SPEEDRN2);  /* VARDELAY */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(ld_dt >= 0);
    return st;
}

/* DISINDAT (p. 900): does the astronaut wish the analog displays (mode
 * select switch in PGNCS)? First time: initialise (DIDFLAG) and enable
 * the display after 80 ms (INTLZE). Afterwards SPEEDRUN.             */
static agc_status_t ld_disindat(void)
{
    if (!agc_lm_chan_bit_present(AGC_CHAN_CHAN30, AGC_CH030_DISPINERT)) {
        return ld_disprset();                /* no inertial data requested */
    }
    if (agc_flag_test(AGC_FLAG_DIDFLAG)) {
        return ld_speedrun();
    }
    (void)agc_flag_set(AGC_FLAG_DIDFLAG);
    (void)ld_imodes33_bits(0u, LD_BIT7);     /* altrate first, then alt.   */
    ld_altpath = LD_NONE;
    if (agc_flag_test(AGC_FLAG_LANDING_ANALOG_DISPLAYS_FLAG)) {
        return AGC_STATUS_OK;                /* not in the descent         */
    }
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN12, AGC_CH012_DISPINERT);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    for (int i = 0; i < 2; i++) {
        ld_trak[i] = 0;
        ld_metr[i] = 0;
    }
    return agc_waitlist_call(8, AGC_TASK_INTLZE);
}

/* LANDISP (p. 898): R10 proper. */
agc_status_t agc_lm_landisp(void)
{
    agc_guid_state()->avg.pipctr = agc_lm_pipctr1();     /* PHSPRDT2       */
    /* DEVIATION: TBASE2 = TIME1 via NEWPHASE of the current group 2 phase */
    agc_status_t st = agc_phase_newphase(2, agc_phase_get(2), true);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    if (!agc_flag_test(AGC_FLAG_SWANDISP)) {
        return ld_disprset();
    }
    ld_altpath = ((agc_erase_imodes33() & LD_BIT7) != 0u) ? LD_ALTOUT : LD_ALTROUT;
    st = ld_disindat();
    if (st != AGC_STATUS_OK || ld_altpath == LD_NONE) {
        return st;
    }
    (void)AGC_ASSERT(ld_altpath != LD_NONE);
    return AGC_STATUS_OK;                    /* resumes at SPEEDRN2        */
}

/* INTLZE (pp. 900-901): enable the RR error counter; inertial data flag. */
agc_status_t agc_lm_task_intlze(void)
{
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN12, AGC_CH012_ENBRRCDUERR);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    uint16_t w = ld_imodes33_bits(LD_BIT8, 0u);
    (void)AGC_ASSERT((w & LD_BIT8) != 0u);
    return AGC_STATUS_OK;
}

/* VMONITOR (pp. 904-906) for one meter: the increment to send. A
 * saturated velocity drives the meter to the +-MAXVBITS stop and
 * remembers it (TRAK +-1); otherwise the meter follows the velocity,
 * moving at most MAXVBITS toward zero-crossing per pass.            */
static agc_sp_t ld_vmon_delta(agc_sp_t vel, agc_sp_t metr, agc_sp_t trak,
                              agc_sp_t *trak_new)
{
    const int32_t mx = LD_MAXVBITS;
    *trak_new = 0;
    if (vel >= mx || vel <= -mx) {                       /* CHKLASTY      */
        if ((metr > 0 && vel <= 0) || (trak > 0 && vel <= 0)) {
            return (agc_sp_t)-mx;                        /* LASTPOSY +5   */
        }
        if ((metr < 0 && vel >= 0) || (trak < 0 && vel > 0)) {
            return (agc_sp_t)mx;                         /* LASTNEGY +4   */
        }
        *trak_new = (vel <= 0) ? -1 : 1;                 /* NEG/POSVMAXY  */
        return (vel <= 0) ? (agc_sp_t)(-(metr + mx)) : (agc_sp_t)(mx - metr);
    }
    if (trak > 0) {                                      /* LATVPOS       */
        return (vel >= 0) ? (agc_sp_t)(vel - metr) : (agc_sp_t)-metr;
    }
    if (trak < 0) {                                      /* LATVNEG       */
        return (vel <= 0) ? (agc_sp_t)(vel - metr) : (agc_sp_t)-metr;
    }
    if (metr >= 0) {
        if (vel >= 0 || metr - vel - mx <= 0) {
            return (agc_sp_t)(vel - metr);               /* LVMINLM       */
        }
        return (agc_sp_t)-mx;
    }
    if (vel <= 0 || vel - mx - metr <= 0) {              /* NEGLMLV       */
        return (agc_sp_t)(vel - metr);
    }
    return (agc_sp_t)mx;
}

/* VMONITOR loop (pp. 904-907): forward, then lateral; ZEROLSTY sends the
 * increment to CDUSCMD / CDUTCMD and moves the meter; BITSET drives.  */
agc_status_t agc_lm_vmonitor(void)
{
    for (int i = 1; i >= 0; i--) {                       /* ITEMP5 1, 0   */
        agc_sp_t tn = 0;
        agc_sp_t d = ld_vmon_delta(ld_vel[i], ld_metr[i], ld_trak[i], &tn);
        ld_trak[i] = tn;
        /* DEVIATION: AD NEG0 turns +0 into -0 for the DINC hardware;
         * there is no -0 here.                                         */
        agc_erase_set_cdutcmd(i, d);
        ld_metr[i] = agc_lm_ad(ld_metr[i], d);
    }
    agc_status_t st = agc_chan_or(AGC_CHAN_CHAN14,
                                  (uint16_t)(AGC_CH014_DRIVECDUS | AGC_CH014_DRIVECDUT));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(ld_trak[0] >= -1 && ld_trak[0] <= 1);
    return st;
}

/* SPEEDRUN after the pause (pp. 902-904): in the descent, VHY and VHZ
 * along UHYP, UHZP; lateral and forward velocity through the outer
 * gimbal angle (M22 = sin AOG, M32 = cos AOG); the cross-pointers. */
agc_status_t agc_lm_task_speedrun2(void)
{
    if (agc_flag_test(AGC_FLAG_LANDING_ANALOG_DISPLAYS_FLAG)) {
        return ld_continue();                /* not in the descent         */
    }
    const agc_guid_lm_t *g = &agc_guid_state()->lm;
    (void)AGC_ASSERT(ld_altpath != LD_NONE);
    int32_t hy = 0;
    int32_t hz = 0;
    for (int i = 0; i < 3; i++) {
        agc_sp_t v = agc_lm_ad(agc_dp_high(g->delvs.c[i]), ld_vvect[i]);
        hy += agc_lm_mp_hi(v, agc_dp_high(g->uhyp.c[i]));
        hz += agc_lm_mp_hi(v, agc_dp_high(g->uhzp.c[i]));
    }
    ld_vhy = ld_sat(2 * (int32_t)ld_sat(hy));            /* DOUBLE        */
    ld_vhz = ld_sat(2 * (int32_t)ld_sat(hz));
    agc_sp_t s = agc_erase_m22();
    agc_sp_t c = agc_erase_m32();
    int32_t lat = (int32_t)agc_lm_mp_hi(c, ld_vhy) + agc_lm_mp_hi(s, ld_vhz);
    ld_vel[0] = ld_mp_dbl(LD_VELCONV, ld_sat(lat), 1);   /* LATVEL        */
    int32_t fwd = (int32_t)agc_lm_mp_hi(c, ld_vhz) - agc_lm_mp_hi(s, ld_vhy);
    ld_vel[1] = ld_mp_dbl(LD_VELCONV, ld_sat(fwd), 1);   /* FORVEL        */
    agc_status_t st = agc_lm_vmonitor();
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return ld_continue();
}
