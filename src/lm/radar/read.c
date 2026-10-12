/**********************************************************************
 * FILE NAME: read.c
 *
 * PURPOSE:
 *   LM radar reading: the read lead-ins RRRANGE, RRRDOT, LRVELX/Y/Z,
 *   LRALT, LRVEL and INITREAD; the RADAR RUPT reader RADAREAD with its
 *   scale, data-good and bias handling; LRPOS2 (landing radar to
 *   position 2); the landing radar lamps R12LITES / RADLITES. A program
 *   starts a reading with a lead-in and waits in RADSTALL; each
 *   RADARUPT brings one sample (RNRAD) which RADAREAD checks and adds
 *   to SAMPLSUM until N good samples are in (GOODEND) or the tries run
 *   out (BADEND).
 *
 * AGC SOURCE:
 *   Luminary099 P20-P25.agc pp. 553-563 (RRRANGE, RRRDOT, LRVELX ..
 *   LRALT, LRVEL, INITREAD, RADAREAD, TRYCOUNT, PLENTY, RENDRAD,
 *   LRPOSCHK, VELCHK, LRHEIGHT, SCALECHK, SCALCHNG, RADIN, SCALADJ,
 *   LRSCK, DGCHECK, DGCHECK2, NOEND, GOODRAD, RESAMPLE, DATAFAIL,
 *   NOMORE, BADRAD, LRPOS2, LRPOSCAN, LRPOSNXT, LASTLRDT) and
 *   pp. 612-613 (R12LITES, RADLITES, HLIGHT, VLIGHT); LM only
 *
 * EXTERNAL VARIABLES:
 *   Name       Type       I/O  Scale      Description
 *   ---------  ---------  ---  ---------  ----------------------------
 *   RNRAD      word       I    counts     the radar sample
 *   CHAN13     channel    I/O  -          radar select, activity bit
 *   CHAN33     channel    I    -          data good, scale, LR position
 *   CHAN12     channel    O    -          LR position 2 command
 *   RADMODES   word       I/O  -          data-fail, scale, position bits
 *   IMODES33   word       I    -          lamp test
 *   SCALBAD RNGSCFLG HFLSHFLG VFLSHFLG LRBYPASS R04FLAG R77FLAG  flags
 *   DNRRANGE .. word       O    counts     downlink copies of RNRAD
 *   TIMEHOLD SAMPLSUM NSAMP SAMPLIM OLDATAGD (file static) I/O
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ----------------------------------------------
 *   agc_task_rr_goodend   GOODEND: wake the program waiting in RADSTALL
 *   rr_badend             BADEND
 *   agc_rr_settrkf        RRTRKF: tracker fail lamp
 *   agc_waitlist_fixdelay FIXDELAY / VARDELAY (LRPOS2 scan)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   00520 (RADAR RUPT not expected), 00521 (N samples failed, unless
 *   R12 or R04 runs), 00522 (LR position wrong). Bad readings end in
 *   BADEND for the waiting program.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   RADAREAD runs in the RADAR RUPT; the lead-ins are called inhibited
 *   or inhibit themselves. Readings are raw counts: SAMPLSUM is a DP
 *   integer (the 15-bit unsigned RNRAD with its top bit carried into
 *   the upper word).
 *
 * NOTES:
 *   Radar select codes (channel 13 bits 1-4, bit 4 = activity):
 *   RRRANGE 11, RRRDOT 12, LRVELX 14, LRVELY 15, LRVELZ 16, LRALT 17.
 **********************************************************************/
#include <stddef.h>

#include "../../programs/navigation/internal.h"
#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/io/service.h"
#include "radar.h"

#define RD_RRRANGE          011u                       /* RR range */
#define RD_RRRDOT           012u                       /* RR range rate */
#define RD_LRVELX           014u                       /* LR velocity X */
#define RD_LRVELY           015u                       /* LR velocity Y */
#define RD_LRVELZ           016u                       /* LR velocity Z */
#define RD_LRALT            017u                       /* LR altitude */
#define RD_ALLREAD          017u                       /* ALLREAD OCT 17: all radar bits      */
#define RD_DGBITS           000230u                    /* DGBITS: chan 33 bits 4, 5, 8        */
#define RD_BITS5_8          000220u                    /* BITS5,8: LR data good bits          */
#define RD_ACTIVITY         000010u                    /* chan 13 bit 4: radar activity       */
#define RD_LR_SELECT        000004u                    /* chan 13 bit 3: landing radar        */
#define RD_POSMAX           037777u                    /* 14 bits */
#define RD_LVELBIAS         (-12288)                   /* LVELBIAS DEC -12288 (153.6 kc)      */
#define RD_RDOTBIAS         ((agc_dp_t) 17000)         /* RDOTBIAS, counts       */
#define RD_HISCALIM         460                        /* HISCALIM DEC 460: 2481.7 ft         */
#define RD_LR_POS2_WAIT     600U                       /* 6SECS DEC 600                       */
#define RD_2SECS            200U                       /* 2 s, cs */
#define RD_1SEC             100U                       /* 1 s, cs */
#define RD_POS_TRIES        14                         /* FOURTEEN: 15 samples                */
#define RD_HLITE            000020u                    /* HLITE EQUALS BIT5                   */
#define RD_VLITE            000004u                    /* VLITE EQUALS BIT3                   */
#define RD_ALARM_UNEXPECTED ((agc_alarm_code_t) 00520) /* rupt not wanted  */
#define RD_ALARM_NO_DATA    ((agc_alarm_code_t) 00521) /* N samples failed */
#define RD_ALARM_LR_POS     ((agc_alarm_code_t) 00522) /* LR position bad  */

/* Radar read erasables (P20-P25, EBANK= LOSCOUNT).                       */
typedef struct rd_state
{
    agc_dp_t timehold;   /* TIMEHOLD: reading mid time, cs 2^28      */
    agc_dp_t samplsum;   /* SAMPLSUM: sum of good samples, unscaled  */
    agc_dp_t sample;     /* ITEMP3, ITEMP4: this sample, unscaled    */
    agc_sp_t nsamp;      /* NSAMP: good samples wanted - 1, unscaled */
    agc_sp_t samplim;    /* SAMPLIM: tries left (-1 none expected; also the LRPOS2 scan counter), unscaled */
    uint16_t oldatagd;   /* OLDATAGD: data-good bits at the last sample   */
    uint16_t itemp1;     /* ITEMP1: this radar's data-good bit            */
    uint16_t dnindex;    /* DNINDEX: chan 13 select bits 1-3              */
    uint16_t dnradar[7]; /* DNRRANGE -1 +DNINDEX: last raw RNRAD per code */
} rd_state_t;

static rd_state_t rd = {0, 0, 0, 0, -1, 0U, 0U, 0U, {0U}};

/**********************************************************************
 * SAMPLSUM: the sum of the good samples of the last reading.
 * AGC: SAMPLSUM (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_radar_samplsum
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- SAMPLSUM, counts (DP integer)
 **********************************************************************/
agc_dp_t agc_radar_samplsum(void)
{
    return (rd.samplsum);
}

/**********************************************************************
 * TIMEHOLD: the nominal mid time of the last reading.
 * AGC: TIMEHOLD (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: rr_timehold
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_dp_t -- TIMEHOLD, cs 2^28
 **********************************************************************/
agc_dp_t rr_timehold(void)
{
    return (rd.timehold);
}

/**********************************************************************
 * DNRRANGE .. (downlink): index 0..6 = select code 1..7.
 * AGC: DNRRANGE (ERASABLE_ASSIGNMENTS.agc)
 *
 * FUNCTION NAME: agc_erase_dnrrange
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   index     int   I    0 .. 6 = select code 1 .. 7
 *
 * RETURNS: uint16_t -- the last raw RNRAD of that code (0 for a bad index)
 **********************************************************************/
uint16_t agc_erase_dnrrange(int index)
{
    return (rd.dnradar[index]);
}

/**********************************************************************
 * INITREAD (p.554)
 * N samples of radar `code`: TIMEHOLD = now + 4N cs (the midpoint of the
 * nominal sampling interval), N tries (the "DOUBLE" for 2N tries is not
 * assembled), data-good bits remembered, radar selected, SAMPLSUM = 0.
 * The caller then waits in RADSTALL.
 * AGC: INITREAD (P20-P25.agc p.554)
 *
 * FUNCTION NAME: rd_initread
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   code      uint16_t  I    radar select code (RRRANGE .. LRALT)
 *   n         agc_sp_t  I    samples wanted, 1 .. 5
 **********************************************************************/
static void rd_initread(uint16_t code, agc_sp_t n)
{
    agc_inhint();
    rd.timehold = (agc_dp_t) (4 * n); /* MP BIT3  */
    rd.nsamp    = (agc_sp_t) (n - 1);
    rd.samplim  = n;
    rd.oldatagd = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & RD_DGBITS);
    rr_chan_clear(AGC_CHAN_CHAN13, RD_ALLREAD);
    agc_chan_or(AGC_CHAN_CHAN13, code);
    rd.timehold = (agc_dp_t) (((uint32_t) rd.timehold + agc_time_now_cs()) & (uint32_t) AGC_DP_MAX);
    rd.samplsum = 0;
    agc_relint();                     /* ROADBACK */
}

/**********************************************************************
 * The lead-ins (p.553). One sample each, except LRVEL.
 * AGC: RRRANGE (P20-P25.agc p.553)
 *
 * FUNCTION NAME: agc_radar_rrrange
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_radar_rrrange(void)
{
    rd_initread(RD_RRRANGE, 1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * RRRDOT lead-in: one sample of RR range rate.
 * AGC: RRRDOT (P20-P25.agc p.553)
 *
 * FUNCTION NAME: agc_radar_rrrdot
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK
 **********************************************************************/
agc_status_t agc_radar_rrrdot(void)
{
    rd_initread(RD_RRRDOT, 1);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * LRALT lead-in: one sample of LR altitude.
 * AGC: LRALT (P20-P25.agc p.553)
 *
 * FUNCTION NAME: agc_radar_lralt
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_radar_lralt(void)
{
    rd_initread(RD_LRALT, 1);
}

/**********************************************************************
 * LRVELX / LRVELY / LRVELZ as entered by the radar sampling test
 * (VARADAR: one sample).
 * AGC: LRVELX, LRVELY, LRVELZ (P20-P25.agc p.553)
 *
 * FUNCTION NAME: agc_radar_lr_velocity_beam
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   beam      int       I    0 Z, 1 Y, 2 X
 *   nsamples  agc_sp_t  I    samples wanted, 1 .. 5
 **********************************************************************/
void agc_radar_lr_velocity_beam(int beam, agc_sp_t nsamples)
{
    static const uint16_t codes[3] = {RD_LRVELZ, RD_LRVELY, RD_LRVELX};

    rd_initread(codes[beam], nsamples);
}

/**********************************************************************
 * LRVEL: five samples of a velocity beam; beam2 = 0, 2, 4 for LRVELZ,
 * LRVELY, LRVELX.
 * AGC: LRVEL (P20-P25.agc p.553)
 *
 * FUNCTION NAME: agc_radar_lrvel
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   beam2     agc_sp_t  I    0, 2, 4 for LRVELZ, LRVELY, LRVELX
 **********************************************************************/
void agc_radar_lrvel(agc_sp_t beam2)
{
    agc_radar_lr_velocity_beam(beam2 / 2, 5);
}

/**********************************************************************
 * lamps (R12LITES, RADLITES, HLIGHT, VLIGHT pp.612-613)
 * The altitude (H) or velocity (V) lamp goes on when its data failed (H
 * also when the LR low scale discrete is missing, SCALBAD), which stops
 * the lamp flashing; else it goes off unless it is flashing. The
 * tracker fail lamp is updated after each (RRTRKF).
 * AGC: HLIGHT, VLIGHT (P20-P25.agc pp.612-613)
 *
 * FUNCTION NAME: rd_lr_lamp
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   velocity  bool  I    true: velocity lamp, false: altitude lamp
 **********************************************************************/
static void rd_lr_lamp(bool velocity)
{
    uint16_t      lamp  = velocity ? RD_VLITE : RD_HLITE;                         /* V or H lamp */
    agc_flag_id_t flash = velocity ? AGC_FLAG_VFLSHFLG : AGC_FLAG_HFLSHFLG;       /* flash flag */
    bool          fail  = rr_radmodes_test(velocity ? RR_LRVELBIT : RR_LRALTBIT); /* data fail */

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

    agc_rr_settrkf(); /* RRTRKF   */
}

/**********************************************************************
 * RADLITES: after a reading of LR velocity (ITEMP1 = BIT8), LR altitude
 * (BIT5) or RR (BIT4). The AGC tells BIT5 by CS BIT5 + BIT5 = -0.
 * AGC: RADLITES (P20-P25.agc p.613)
 *
 * FUNCTION NAME: rr_radlites
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  --------------------------------------
 *   itemp1    uint16_t  I    ITEMP1: the radar's data-good bit
 **********************************************************************/
void rr_radlites(uint16_t itemp1)
{
    if ((agc_erase_imodes33() & 1U) == 0U) /* lamp test */
    {
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
            agc_rr_settrkf(); /* RRTRKF */
        }
    }
}

/**********************************************************************
 * R12LITES: both landing radar lamps (from the R10/R12 lamp routine).
 * AGC: R12LITES (P20-P25.agc p.612)
 *
 * FUNCTION NAME: agc_radar_landing_radar_lights
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_radar_landing_radar_lights(void)
{
    if ((agc_erase_imodes33() & 1U) == 0U) /* ISWRETRN */
    {
        rd_lr_lamp(false);
        rd_lr_lamp(true);
    }
}

/* ---- RADAREAD (RADAR RUPT, pp.555-560) -------------------------------- */
/**********************************************************************
 * BADRAD: no more tries; bad end for the waiting program.
 * AGC: BADRAD (P20-P25.agc p.560)
 *
 * FUNCTION NAME: rd_badrad
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of BADEND
 **********************************************************************/
static agc_status_t rd_badrad(void)
{
    rd.samplim = -1;

    return (rr_badend());
}

/**********************************************************************
 * NOMORE: the tries ran out. Alarm 521 unless R12 is on (LRBYPASS
 * clear) or R04 is running (R04FLAG).
 * AGC: NOMORE (P20-P25.agc p.560)
 *
 * FUNCTION NAME: rd_nomore
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of BADEND
 **********************************************************************/
static agc_status_t rd_nomore(void)
{
    if (agc_flag_test(AGC_FLAG_LRBYPASS) && !agc_flag_test(AGC_FLAG_ALARM_521_SUPPRESSED))
    {
        agc_alarm(RD_ALARM_NO_DATA);
    }

    return (rd_badrad());
}

/**********************************************************************
 * RESAMPLE: another try if any are left (activity bit on again), else
 * DATAFAIL: the data-fail bit in RADMODES, the last sample only, lamps,
 * NOMORE.
 * AGC: RESAMPLE, DATAFAIL (P20-P25.agc p.560)
 *
 * FUNCTION NAME: rd_resample
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK (another try), else the status of
 *          BADEND
 **********************************************************************/
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

/**********************************************************************
 * GOODRAD: enough good data; data-fail flag off, lamps, good end.
 * AGC: GOODRAD (P20-P25.agc p.560)
 *
 * FUNCTION NAME: rd_goodrad
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of GOODEND
 **********************************************************************/
static agc_status_t rd_goodrad(void)
{
    rd.samplim = -1;
    rr_radmodes_clear(rd.itemp1);
    rr_radlites(rd.itemp1);

    return (agc_task_rr_goodend());
}

/**********************************************************************
 * DGCHECK: the data-good bit must be present (0 in chan 33) before and
 * after the sample; then the sample goes into SAMPLSUM.
 * AGC: DGCHECK (P20-P25.agc p.559)
 *
 * FUNCTION NAME: rd_dgcheck
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when the data-good bit was present before and after
 *          (sample added)
 **********************************************************************/
static bool rd_dgcheck(void)
{
    uint16_t now; /* after */

    now             = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & rd.itemp1);
    uint16_t before = (uint16_t) (rd.oldatagd & rd.itemp1); /* before */

    rd.oldatagd = (uint16_t) ((rd.oldatagd & ~rd.itemp1) | now);

    if ((before | now) != 0U)
    {
        return (false);
    }

    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    rd.samplsum = agc_dp_add(&ip, rd.samplsum, rd.sample); /* DAS      */

    return (true);
}

/**********************************************************************
 * RNRAD as the DP count (ITEMP3, ITEMP4): bit 15 in the upper word
 * (CAE RNRAD, DOUBLE, MASK BIT1), bits 1-14 plus `bias` in the lower.
 * AGC: "CAE RNRAD, DOUBLE, MASK BIT1" (P20-P25.agc p.558)
 *
 * FUNCTION NAME: rd_count
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   bias      int   I    count added to the lower word (LVELBIAS or 0)
 *
 * RETURNS: agc_dp_t -- the sample, counts (DP integer)
 **********************************************************************/
static agc_dp_t rd_count(int bias)
{
    uint16_t w  = (uint16_t) (agc_erase_rnrad() & RR_WORD_MASK); /* RNRAD */
    int      hi = (int) ((w >> 14) & 1U);                        /* bit 15 */
    int      lo = (int) (w & RD_POSMAX) + bias;                  /* bits 1-14 + bias */

    return (agc_dp_pack((agc_sp_t) hi, (agc_sp_t) lo));
}

/**********************************************************************
 * SCALECHK / RADIN / SCALADJ / DGCHECK2 for RR range, RR range rate and
 * LR altitude (pp.557-560). scale_bit: the chan 33 / RADMODES scale bit
 * (0 for range rate). A scale change since the last reading flips the
 * RADMODES bit, sets RNGSCFLG and rejects the reading.
 * AGC: SCALECHK, RADIN, SCALADJ, DGCHECK2 (P20-P25.agc pp.557-560)
 *
 * FUNCTION NAME: rd_scaled
 *
 * ARGUMENTS:
 *   Argument   Type      I/O  Description (scale)
 *   ---------  --------  ---  -------------------------------------
 *   scale_bit  uint16_t  I    chan 33 / RADMODES scale bit, 0 for range
 *                             rate
 *   lr         bool      I    true: LR altitude, false: RR range / rate
 *
 * RETURNS: agc_status_t -- the status of GOODEND, BADEND or AGC_STATUS_OK
 *          (another try)
 **********************************************************************/
static agc_status_t rd_scaled(uint16_t scale_bit, bool lr)
{
    uint16_t now = 0U; /* scale bit now */

    if (scale_bit != 0U)
    { /* SCALECHK */
        now = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & scale_bit);
        uint16_t before; /* last */

        before = (uint16_t) (agc_erase_radmodes() & scale_bit);

        if (now != before)
        { /* SCALCHNG */
            agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() ^ scale_bit));
            rd.oldatagd = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN33) & RD_DGBITS);
            rz_flag_put(AGC_FLAG_RNGSCFLG, true);

            return (rd_badrad());
        }
    }

    rd.sample = rd_count(0); /* RADIN    */

    if (scale_bit == 0U)
    { /* RR RDOT: no scale    */
        agc_interp_t ip; /* interpreter state */

        agc_interp_init(&ip);
        rd.sample = agc_dp_sub(&ip, rd.sample, RD_RDOTBIAS);
    }
    else if (now != 0U)
    { /* SCALADJ: high scale  */
        if (lr)
        {
            /**********************************************************************
             * LRSCK (p.559): LR altitude on high scale: SCALBAD when the reading is
             * below 2481.7 ft (the low scale discrete should be present); the high
             * scale reading is 5 times the low scale count.
             * AGC: LRSCK (P20-P25.agc p.559)
             *
             * FUNCTION NAME: rd_lr_scale
             *
             * ARGUMENTS: none
             **********************************************************************/
            {
                bool low; /* SCALBAD */

                low = (agc_dp_high(rd.sample) <= 0) && (agc_dp_low(rd.sample) < RD_HISCALIM);

                rz_flag_put(AGC_FLAG_SCALBAD, low);
                agc_interp_t ip; /* interpreter state */

                agc_interp_init(&ip);
                agc_dp_t four; /* DDOUBL x2, counts unscaled */

                four = agc_dp_shl(&ip, rd.sample, 2);

                rd.sample = agc_dp_add(&ip, rd.sample, four); /* DASAMPL  */
            }
        }
        else
        {
            agc_interp_t ip; /* interpreter state */

            agc_interp_init(&ip);
            rd.sample = agc_dp_shl(&ip, rd.sample, 3); /* x 8      */
        }
    }

    return (rd_dgcheck() ? rd_goodrad() : rd_resample()); /* DGCHECK2 */
} /* rd_scaled */

/**********************************************************************
 * RADAREAD: the RADAR RUPT handler. The raw reading is kept for the
 * downlink; a rupt nobody asked for is alarm 520.
 * AGC: RADAREAD (P20-P25.agc pp.555-560)
 *
 * FUNCTION NAME: agc_rendz_radar_read
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t agc_rendz_radar_read(void)
{
    rd.dnindex = (uint16_t) (agc_chan_read(AGC_CHAN_CHAN13) & 7U);

    if (rd.dnindex != 0U)
    {
        rd.dnradar[rd.dnindex - 1U] = (uint16_t) (agc_erase_rnrad() & RR_WORD_MASK);
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
        /* Rendezvous radar sample (RENDRAD p.557): rejected during a
         * reposition or after an RR CDU fail.                         */
        if (rr_radmodes_test(RR_REPOSBIT) || !rr_radmodes_test(RR_RCDUFBIT))
        {
            return (rd_badrad());
        }

        rd.itemp1  = RR_RRDATABT;
        bool range = (agc_chan_read(AGC_CHAN_CHAN13) & 1U) != 0U; /* RR range */

        uint16_t scale_bit = 0U; /* range rate: no scale bit */

        if (range)
        {
            scale_bit = RR_RRRSBIT; /* RR range scale */
        }

        return (rd_scaled(scale_bit, false));
    }

    /* Landing radar sample (LRPOSCHK, VELCHK, LRHEIGHT pp.556-557). R77
     * ignores data fails: it only copies the LR data-good bits.            */
    if (agc_flag_test(AGC_FLAG_RADAR_ALARMS_SUPPRESSED))
    { /* R77CHECK */
        uint16_t m; /* RADMODES */

        m = (uint16_t) (agc_erase_radmodes() & ~RD_BITS5_8);

        m = (uint16_t) (m | (agc_chan_read(AGC_CHAN_CHAN33) & RD_BITS5_8));
        agc_erase_set_radmodes((uint16_t) (m & RR_WORD_MASK));

        return (agc_task_rr_goodend());
    }

    if (((agc_erase_radmodes() ^ agc_chan_read(AGC_CHAN_CHAN33)) & RR_LRPOSBIT) != 0U)
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
} /* agc_rendz_radar_read */

/**********************************************************************
 * LRPOS2 (pp.561-562): command the landing radar to position 2 and
 * check that it gets there: scanning for the inbit starts after 6 s,
 * once a second for 15 s; then 2 s for the antenna bounce to die out.
 * AGC: LRPOS2 (P20-P25.agc p.561)
 *
 * FUNCTION NAME: agc_radar_lrpos2
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, AGC_STATUS_BAD_ARG on a bad
 *          argument, else the status of the call that failed
 **********************************************************************/
agc_status_t agc_radar_lrpos2(void)
{
    agc_inhint();
    rr_radmodes_set(RR_LRPOSBIT);
    agc_status_t st; /* waitlist status */

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

/**********************************************************************
 * LRPOSCAN: set up for 15 samples.
 * AGC: LRPOSCAN (P20-P25.agc p.561)
 *
 * FUNCTION NAME: agc_task_lr_position_scan
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_lr_position_scan(void)
{
    rd.samplim = RD_POS_TRIES;
    agc_status_t st; /* FIXDELAY */

    st = agc_waitlist_fixdelay(RD_1SEC, AGC_TASK_LR_POSITION_CHECK);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/**********************************************************************
 * LRPOSNXT after the 1 s FIXDELAY.
 * AGC: LRPOSNXT (P20-P25.agc p.562)
 *
 * FUNCTION NAME: agc_task_lr_position_check
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_lr_position_check(void)
{
    agc_status_t st; /* waitlist status */

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

/**********************************************************************
 * LASTLRDT after the 2 s wait: remove the command, good end.
 * AGC: LASTLRDT (P20-P25.agc p.562)
 *
 * FUNCTION NAME: agc_task_lr_position_settled
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK when the task ended or rescheduled
 *          itself, else the status of the waitlist or executive call that
 *          failed
 **********************************************************************/
agc_status_t agc_task_lr_position_settled(void)
{
    rr_chan_clear(AGC_CHAN_CHAN12, RR_CH12_LRPOS2);

    if (!AGC_ASSERT(rr_radmodes_test(RR_LRPOSBIT)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_task_rr_goodend());
}
