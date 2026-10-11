/*
 * read.c -- LM radar reading (P20-P25.agc, Luminary099
 * pp.553-563, 612-613): the read lead-ins RRRANGE, RRRDOT, LRVELX/Y/Z,
 * LRALT, LRVEL and INITREAD; the RADAR RUPT reader RADAREAD with its
 * scale, data-good and bias handling; LRPOS2 (landing radar to position
 * 2); the landing radar lamps R12LITES / RADLITES.
 *
 * A program starts a reading with a lead-in and waits in RADSTALL; each
 * RADARUPT brings one sample (RNRAD) which RADAREAD checks and adds to
 * SAMPLSUM until N good samples are in (GOODEND) or the tries run out
 * (BADEND). Readings are raw counts: SAMPLSUM is a DP integer (the
 * 15-bit unsigned RNRAD with its top bit carried into the upper word).
 *
 * Radar select codes (channel 13 bits 1-4, bit 4 = activity):
 *   RRRANGE 11, RRRDOT 12, LRVELX 14, LRVELY 15, LRVELZ 16, LRALT 17.
 */
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/io/service.h"
#include "radar.h"

#define RD_RRRANGE          011u
#define RD_RRRDOT           012u
#define RD_LRVELX           014u
#define RD_LRVELY           015u
#define RD_LRVELZ           016u
#define RD_LRALT            017u
#define RD_ALLREAD          017u    /* ALLREAD OCT 17: all radar bits      */
#define RD_DGBITS           000230u /* DGBITS: chan 33 bits 4, 5, 8        */
#define RD_BITS5_8          000220u /* BITS5,8: LR data good bits          */
#define RD_ACTIVITY         000010u /* chan 13 bit 4: radar activity       */
#define RD_LR_SELECT        000004u /* chan 13 bit 3: landing radar        */
#define RD_POSMAX           037777u
#define RD_LVELBIAS         (-12288) /* LVELBIAS DEC -12288 (153.6 kc)      */
#define RD_RDOTBIAS         ((agc_dp_t) 17000) /* RDOTBIAS 2DEC 17000 counts     */
#define RD_HISCALIM         460  /* HISCALIM DEC 460: 2481.7 ft         */
#define RD_LR_POS2_WAIT     600U /* 6SECS DEC 600                       */
#define RD_2SECS            200U
#define RD_1SEC             100U
#define RD_POS_TRIES        14      /* FOURTEEN: 15 samples                */
#define RD_HLITE            000020u /* HLITE EQUALS BIT5                   */
#define RD_VLITE            000004u /* VLITE EQUALS BIT3                   */
#define RD_ALARM_UNEXPECTED ((agc_alarm_code_t) 00520) /* rupt not wanted  */
#define RD_ALARM_NO_DATA    ((agc_alarm_code_t) 00521) /* N samples failed */
#define RD_ALARM_LR_POS     ((agc_alarm_code_t) 00522) /* LR position bad  */

/* Radar read erasables (P20-P25, EBANK= LOSCOUNT).                       */
typedef struct rd_state
{
    agc_dp_t timehold;   /* TIMEHOLD: nominal mid time of the reading,
                          * cs 2^28                                       */
    agc_dp_t samplsum;   /* SAMPLSUM: sum of good samples, counts         */
    agc_dp_t sample;     /* ITEMP3, ITEMP4: this sample, counts           */
    agc_sp_t nsamp;      /* NSAMP: good samples still wanted, minus one   */
    agc_sp_t samplim;    /* SAMPLIM: tries left, -1 none expected (also
                          * the LRPOS2 scan counter)                      */
    uint16_t oldatagd;   /* OLDATAGD: data-good bits at the last sample   */
    uint16_t itemp1;     /* ITEMP1: this radar's data-good bit            */
    uint16_t dnindex;    /* DNINDEX: chan 13 select bits 1-3              */
    uint16_t dnradar[7]; /* DNRRANGE -1 +DNINDEX: last raw RNRAD per code */
} rd_state_t;

static rd_state_t rd = {0, 0, 0, 0, -1, 0U, 0U, 0U, {0U}};

agc_dp_t agc_radar_samplsum(void)
{
    if (!AGC_ASSERT(rd.samplim >= -1))
    {
        return (0); /* no valid result */
    }

    return (rd.samplsum);
}

agc_dp_t rr_timehold(void)
{
    if (!AGC_ASSERT(rd.timehold >= 0))
    {
        return (0); /* no valid result */
    }

    return (rd.timehold);
}

/* DNRRANGE .. (downlink): index 0..6 = select code 1..7.                 */
uint16_t agc_erase_dnrrange(int index)
{
    if (!AGC_ASSERT(index >= 0 && index < 7))
    {
        return (0U);
    }

    return (rd.dnradar[index]);
}

/* ---- INITREAD (p.554) ------------------------------------------------
 * N samples of radar `code`: TIMEHOLD = now + 4N cs (the midpoint of the
 * nominal sampling interval), N tries (the "DOUBLE" for 2N tries is not
 * assembled), data-good bits remembered, radar selected, SAMPLSUM = 0.
 * The caller then waits in RADSTALL.                                    */
static void rd_initread(uint16_t code, agc_sp_t n)
{
    if (!AGC_ASSERT(n >= 1 && n <= 5) ||
        !AGC_ASSERT(code >= RD_RRRANGE && code <= RD_LRALT))
    {
        return;
    }

    agc_inhint();
    rd.timehold = (agc_dp_t) (4 * n); /* MP BIT3  */
    rd.nsamp    = (agc_sp_t) (n - 1);
    rd.samplim  = n;
    rd.oldatagd = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & RD_DGBITS);
    rr_chan_clear(AGC_CHAN_CHAN13, RD_ALLREAD);
    agc_chan_or(AGC_CHAN_CHAN13, code);
    rd.timehold = (agc_dp_t) (((uint32_t) rd.timehold + agc_time_now_cs()) &
                              (uint32_t) AGC_DP_MAX);
    rd.samplsum = 0;
    agc_relint(); /* ROADBACK */
}

/* The lead-ins (p.553). One sample each, except LRVEL.                 */
agc_status_t agc_radar_rrrange(void)
{
    rd_initread(RD_RRRANGE, 1);

    return (AGC_STATUS_OK);
}

agc_status_t agc_radar_rrrdot(void)
{
    rd_initread(RD_RRRDOT, 1);

    return (AGC_STATUS_OK);
}

void agc_radar_lralt(void)
{
    rd_initread(RD_LRALT, 1);

    return;
}

/* LRVELX / LRVELY / LRVELZ as entered by the radar sampling test
 * (VARADAR: one sample).                                                */
void agc_radar_lr_velocity_beam(int beam, agc_sp_t nsamples)
{
    if (!AGC_ASSERT(beam >= 0 && beam <= 2))
    {
        return;
    }

    static const uint16_t codes[3] = {RD_LRVELZ, RD_LRVELY, RD_LRVELX};

    rd_initread(codes[beam], nsamples);

    return;
}

/* LRVEL: five samples of a velocity beam; beam2 = 0, 2, 4 for LRVELZ,
 * LRVELY, LRVELX.                                                      */
void agc_radar_lrvel(agc_sp_t beam2)
{
    if (!AGC_ASSERT(beam2 == 0 || beam2 == 2 || beam2 == 4))
    {
        return;
    }

    agc_radar_lr_velocity_beam(beam2 / 2, 5);

    return;
}

/* ---- lamps (R12LITES, RADLITES, HLIGHT, VLIGHT pp.612-613) -------------
 * The altitude (H) or velocity (V) lamp goes on when its data failed (H
 * also when the LR low scale discrete is missing, SCALBAD), which stops
 * the lamp flashing; else it goes off unless it is flashing. The
 * tracker fail lamp is updated after each (RRTRKF).                    */
static void rd_lr_lamp(bool velocity)
{
    uint16_t      lamp  = velocity ? RD_VLITE : RD_HLITE;
    agc_flag_id_t flash = velocity ? AGC_FLAG_VFLSHFLG : AGC_FLAG_HFLSHFLG;
    bool          fail  =
        rr_radmodes_test(velocity ? RR_LRVELBIT : RR_LRALTBIT);

    if (!velocity && agc_flag_test(AGC_FLAG_SCALBAD))
    {
        fail = true;
    }

    if (fail)
    { /* ONLITES  */
        rz_flag_put(flash, false);
        rr_lamp_put(lamp, true);
    }
    else if (!agc_flag_test(flash))
    {
        rr_lamp_put(lamp, false); /* LITIT    */
    }

    agc_status_t st = agc_rr_settrkf(); /* RRTRKF   */

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return;
    }
}

/* RADLITES: after a reading of LR velocity (ITEMP1 = BIT8), LR altitude
 * (BIT5) or RR (BIT4). The AGC tells BIT5 by CS BIT5 + BIT5 = -0.       */
void rr_radlites(uint16_t itemp1)
{
    if (!AGC_ASSERT(itemp1 != 0U))
    {
        return;
    }

    if ((agc_erase_imodes33() & 1U) != 0U)
    {
        return; /* lamp test */
    }

    if (itemp1 == RR_LRVELBIT)
    {
        rd_lr_lamp(true);
    }
    else if (itemp1 == RR_LRALTBIT)
    {
        rd_lr_lamp(false);
    }
    else
    {
        agc_status_t st = agc_rr_settrkf();

        if (!AGC_ASSERT(st == AGC_STATUS_OK))
        {
            return;
        }
    }
}

/* R12LITES: both landing radar lamps (from the R10/R12 lamp routine).  */
void agc_radar_landing_radar_lights(void)
{
    if ((agc_erase_imodes33() & 1U) != 0U)
    {
        return; /* ISWRETRN */
    }

    rd_lr_lamp(false);
    rd_lr_lamp(true);

    if (!AGC_ASSERT(agc_erase_radmodes() <= RR_WORD_MASK))
    {
        return;
    }
}

/* ---- RADAREAD (RADAR RUPT, pp.555-560) -------------------------------- */
/* BADRAD: no more tries; bad end for the waiting program.               */
static agc_status_t rd_badrad(void)
{
    rd.samplim = -1;

    return (rr_badend());
}

/* NOMORE: the tries ran out. Alarm 521 unless R12 is on (LRBYPASS
 * clear) or R04 is running (R04FLAG).                                   */
static agc_status_t rd_nomore(void)
{
    if (agc_flag_test(AGC_FLAG_LRBYPASS) &&
        !agc_flag_test(AGC_FLAG_ALARM_521_SUPPRESSED))
    {
        agc_alarm(RD_ALARM_NO_DATA);
    }

    return (rd_badrad());
}

/* RESAMPLE: another try if any are left (activity bit on again), else
 * DATAFAIL: the data-fail bit in RADMODES, the last sample only, lamps,
 * NOMORE.                                                               */
static agc_status_t rd_resample(void)
{
    if (rd.samplim > 0)
    {
        agc_chan_or(AGC_CHAN_CHAN13, RD_ACTIVITY);

        return (AGC_STATUS_OK); /* RESUME   */
    }

    rr_radmodes_set(rd.itemp1); /* DATAFAIL */
    rd.samplsum = rd.sample;
    rr_radlites(rd.itemp1);

    return (rd_nomore());
}

/* GOODRAD: enough good data; data-fail flag off, lamps, good end.       */
static agc_status_t rd_goodrad(void)
{
    rd.samplim = -1;
    rr_radmodes_clear(rd.itemp1);
    rr_radlites(rd.itemp1);

    return (agc_task_rr_goodend());
}

/* DGCHECK: the data-good bit must be present (0 in chan 33) before and
 * after the sample; then the sample goes into SAMPLSUM.                 */
static bool rd_dgcheck(void)
{
    uint16_t now    = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & rd.itemp1);
    uint16_t before = (uint16_t) (rd.oldatagd & rd.itemp1);

    rd.oldatagd = (uint16_t) ((rd.oldatagd & ~rd.itemp1) | now);

    if ((before | now) != 0U)
    {
        return (false);
    }

    agc_interp_t ip;

    agc_interp_init(&ip);
    rd.samplsum = agc_dp_add(&ip, rd.samplsum, rd.sample); /* DAS      */

    return (true);
}

/* RNRAD as the DP count (ITEMP3, ITEMP4): bit 15 in the upper word
 * (CAE RNRAD, DOUBLE, MASK BIT1), bits 1-14 plus `bias` in the lower.   */
static agc_dp_t rd_count(int bias)
{
    uint16_t w  = (uint16_t) (agc_erase_rnrad() & RR_WORD_MASK);
    int      hi = (int) ((w >> 14) & 1U);
    int      lo = (int) (w & RD_POSMAX) + bias;

    if (!AGC_ASSERT(lo >= AGC_SP_MIN && lo <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_pack((agc_sp_t) hi, (agc_sp_t) lo));
}

/* LRSCK (p.559): LR altitude on high scale: SCALBAD when the reading is
 * below 2481.7 ft (the low scale discrete should be present); the high
 * scale reading is 5 times the low scale count.                         */
static void rd_lr_scale(void)
{
    bool low =
        (agc_dp_high(rd.sample) <= 0) && (agc_dp_low(rd.sample) < RD_HISCALIM);

    rz_flag_put(AGC_FLAG_SCALBAD, low);
    agc_interp_t ip;

    agc_interp_init(&ip);
    agc_dp_t four = agc_dp_shl(&ip, rd.sample, 2);    /* DDOUBL x2 */

    rd.sample = agc_dp_add(&ip, rd.sample, four); /* DASAMPL  */
}

/* SCALECHK / RADIN / SCALADJ / DGCHECK2 for RR range, RR range rate and
 * LR altitude (pp.557-560). scale_bit: the chan 33 / RADMODES scale bit
 * (0 for range rate). A scale change since the last reading flips the
 * RADMODES bit, sets RNGSCFLG and rejects the reading.                 */
static agc_status_t rd_scaled(uint16_t scale_bit, bool lr)
{
    uint16_t now = 0U;

    if (scale_bit != 0U)
    { /* SCALECHK */
        now = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & scale_bit);
        uint16_t before = (uint16_t) (agc_erase_radmodes() & scale_bit);

        if (now != before)
        { /* SCALCHNG */
            agc_erase_set_radmodes(
                (uint16_t) (agc_erase_radmodes() ^ scale_bit));
            rd.oldatagd =
                (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & RD_DGBITS);
            rz_flag_put(AGC_FLAG_RNGSCFLG, true);

            return (rd_badrad());
        }
    }

    rd.sample = rd_count(0); /* RADIN    */

    if (scale_bit == 0U)
    { /* RR RDOT: no scale    */
        agc_interp_t ip;

        agc_interp_init(&ip);
        rd.sample = agc_dp_sub(&ip, rd.sample, RD_RDOTBIAS);
    }
    else if (now != 0U)
    { /* SCALADJ: high scale  */
        if (lr)
        {
            rd_lr_scale();
        }
        else
        {
            agc_interp_t ip;

            agc_interp_init(&ip);
            rd.sample = agc_dp_shl(&ip, rd.sample, 3); /* x 8      */
        }
    }

    return (rd_dgcheck() ? rd_goodrad() : rd_resample()); /* DGCHECK2 */
}

/* RADAREAD: the RADAR RUPT handler. The raw reading is kept for the
 * downlink; a rupt nobody asked for is alarm 520.                       */
agc_status_t agc_rendz_radar_read(void)
{
    rd.dnindex = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN13) & 7U);

    if (rd.dnindex != 0U)
    {
        rd.dnradar[rd.dnindex - 1U] =
            (uint16_t) (agc_erase_rnrad() & RR_WORD_MASK);
    }

    if (rd.samplim < 0)
    { /* TRYCOUNT */
        agc_alarm(RD_ALARM_UNEXPECTED);

        return (AGC_STATUS_OK); /* RESUME  */
    }

    if (rd.samplim == 0)
    {
        return (rd_nomore());
    }

    rd.samplim--; /* PLENTY   */

    if ((agc_chan_read(AGC_CHAN_CHAN13) & RD_LR_SELECT) == 0U)
    {
        /* Rendezvous radar sample (RENDRAD p.557): rejected during a reposition
         * or after an RR CDU fail.                                              */
        if (rr_radmodes_test(RR_REPOSBIT) || !rr_radmodes_test(RR_RCDUFBIT))
        {
            return (rd_badrad());
        }

        rd.itemp1 = RR_RRDATABT;
        bool range = (agc_chan_read(AGC_CHAN_CHAN13) & 1U) != 0U;

        return (rd_scaled(range ? RR_RRRSBIT : 0U, false));
    }

    /* Landing radar sample (LRPOSCHK, VELCHK, LRHEIGHT pp.556-557). R77
     * ignores data fails: it only copies the LR data-good bits.            */
    if (agc_flag_test(AGC_FLAG_RADAR_ALARMS_SUPPRESSED))
    { /* R77CHECK */
        uint16_t m = (uint16_t) (agc_erase_radmodes() & ~RD_BITS5_8);

        m = (uint16_t) (m | (agc_chan_read(AGC_CHAN_CHAN33) & RD_BITS5_8));
        agc_erase_set_radmodes((uint16_t) (m & RR_WORD_MASK));

        return (agc_task_rr_goodend());
    }

    if (((agc_erase_radmodes() ^ agc_chan_read(AGC_CHAN_CHAN33)) &
         RR_LRPOSBIT) != 0U)
    { /* LRPOSCHK */
        agc_alarm(RD_ALARM_LR_POS);

        return (rd_badrad());
    }

    if (((agc_chan_read(AGC_CHAN_CHAN13) ^ 3U) & 3U) == 0U)
    {
        rd.itemp1 = RR_LRALTBIT; /* LRHEIGHT */

        return (rd_scaled(RR_ALTSCBIT, true));
    }

    rd.sample = rd_count(RD_LVELBIAS); /* VELCHK   */
    rd.itemp1 = RR_LRVELBIT;

    if (!rd_dgcheck())
    {
        return (rd_resample());
    }

    if (rd.nsamp > 0)
    { /* NOEND    */
        rd.nsamp--;

        return (rd_resample());
    }

    return (rd_goodrad());
}

/* ---- LRPOS2 (pp.561-562): command the landing radar to position 2 and
 * check that it gets there: scanning for the inbit starts after 6 s,
 * once a second for 15 s; then 2 s for the antenna bounce to die out. */
agc_status_t agc_radar_lrpos2(void)
{
    agc_inhint();
    rr_radmodes_set(RR_LRPOSBIT);
    agc_status_t st;

    if ((agc_chan_read(AGC_CHAN_CHAN33) & RR_CH33_LR_POS2) == 0U)
    {
        st = rr_radnoop(); /* already  */
    }
    else
    {
        agc_chan_or(AGC_CHAN_CHAN12, RR_CH12_LRPOS2);
        st = agc_waitlist_call(RD_LR_POS2_WAIT, AGC_TASK_LR_POSITION_SCAN);
    }

    agc_relint();

    return (st);
}

/* LRPOSCAN: set up for 15 samples.                                      */
agc_status_t agc_task_lr_position_scan(void)
{
    rd.samplim = RD_POS_TRIES;
    agc_status_t st =
        agc_waitlist_fixdelay(RD_1SEC, AGC_TASK_LR_POSITION_CHECK);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* LRPOSNXT after the 1 s FIXDELAY.                                      */
agc_status_t agc_task_lr_position_check(void)
{
    agc_status_t st;

    if ((agc_chan_read(AGC_CHAN_CHAN33) & RR_CH33_LR_POS2) == 0U)
    {
        st = agc_waitlist_vardelay(RD_2SECS, AGC_TASK_LR_POSITION_SETTLED);
    }
    else if (rd.samplim > 0)
    {
        rd.samplim--;
        st = agc_waitlist_fixdelay(RD_1SEC, AGC_TASK_LR_POSITION_CHECK);
    }
    else
    {
        rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_LRPOS2); /* time up  */

        return (rr_badend());
    }

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* LASTLRDT after the 2 s wait: remove the command, good end.           */
agc_status_t agc_task_lr_position_settled(void)
{
    rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_LRPOS2);

    if (!AGC_ASSERT(rr_radmodes_test(RR_LRPOSBIT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_rr_goodend());
}
