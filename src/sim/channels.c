/**********************************************************************
 * FILE NAME: channels.c
 *
 * PURPOSE:
 *   The I/O channels as Virtual AGC peripherals see them: the broadcast
 *   of output channels, the connect update, the fictitious outputs
 *   yaAGC computes (DSKY lamps 0163, IMU CDU drive 0174-0176, gyro
 *   torque 0177, optics / TVC drive 0171-0172), and the input path from
 *   peripheral packets into the input channels and interrupt requests.
 *
 * AGC SOURCE:
 *   No AGC routine: virtualagc yaAGC/SocketAPI.c (ChannelOutput,
 *   ChannelInput, UpdateAgcPeripheralConnect) and yaAGC/agc_engine.c
 *   (WriteIO, CpuWriteIO, UpdateDSKY, BurstOutput, the gyro and optics
 *   drives).
 *
 * EXTERNAL VARIABLES:
 *   Name            Type      I/O  Scale   Description
 *   --------------  --------  ---  ------  ---------------------------
 *   channels 5-035  uint16_t  I/O  bits    agc_chan_read / _set_input
 *   CDUXCMD-CDUZCMD agc_sp_t  I/O  pulses  agc_imu_cducmd
 *   GYROCMD         uint16_t  I/O  pulses  agc_imu_gyrocmd
 *   CDUTCMD, CDUSCMD agc_sp_t I/O  pulses  agc_erase_cdutcmd
 *   P/Q/R-RHCCTR    agc_sp_t  O    counts  LM RHC counters
 *   INLINK          uint16_t  O    word    uplink word
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   sim_socket_send       broadcast one packet (socket.c)
 *   sim_clock_standby     standby: STBY lamp, EL off, no drives
 *   sim_clock_arm_traps   channel 13 HANDRUPT trap bits (clock.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   sim_channels_tick returns the status of a refused CDU count write
 *   (agc_imu_hw_set_cducmd); sim_channels_init that of channel 30.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Called from the clock (clock.c) after every interrupt and job and
 *   once per centisecond, and from the socket server on input.
 *
 * NOTES:
 *   Outputs (yaAGC ChannelOutput): yaAGC broadcasts every program WRITE,
 *   WOR or WAND of a channel. The translated program writes the channel
 *   image in src/core/channels.c directly, so the harness samples the
 *   output channels after every interrupt and job and broadcasts what
 *   changed; 010 (DSKY relays) is sent after every T4RUPT, which writes
 *   it on every pass, and 034/035 (downlink) after every DOWNRUPT.
 *   DEVIATION (sampling): an output written twice with the same value
 *   between two samples, or changed and changed back inside one job, is
 *   sent once or not at all; only the relays and the downlink rely on
 *   repeats, and they are forced as above.
 *   Fictitious outputs (yaAGC agc_engine.c):
 *     0163       DSKY lamps not wired to a channel bit (UpdateDSKY): KEY
 *                REL, OPR ERR and V/N flash (1.28 s period, the first
 *                0.32 s dark; KEY REL and OPR ERR flash with it),
 *                RESTART (power-on GOJAM; reset by RSET on 015 or 016 or
 *                a write of channel 11 bit 10), STBY (light test, or
 *                standby with the EL-off bit), TEMP (channel 11 bit 4 or
 *                channel 30 bit 15). AGC WARNING is not modelled (no
 *                warning filter, clock.c).
 *     0174-0176  IMU CDU drive (BurstOutput): while channel 14 bits
 *                15-13 drive an axis, at most 24 pulses (192 /
 *                COARSE_SMOOTH 8) per axis every 75 ms, the first burst
 *                at once when no count was left over.
 *     0177       gyro torque pulses (GYRO_TIMING_SIMULATED off): when
 *                channel 14 bit 10 is set with GYROCMD non-zero, the
 *                train still pending is flushed in packets of at most
 *                03777, GYROCMD is taken whole, and it is sent in bursts
 *                of at most 1024 every 0.25 s, the first at once; word =
 *                ((channel 14 bits 6-9) << 6) | count.
 *     0171/0172  optics trunnion / shaft (CSM TVC yaw / pitch) drive: the
 *                whole count when channel 14 bit 12 / 11 is set.
 *   DEVIATION (CDU drive register): yaAGC empties CDUxCMD at each burst
 *   and keeps the rest in the drive; here the rest stays in CDUxCMD, as
 *   the hardware's diminishing counter leaves it (NEEDLES' ADS CDUXCMD
 *   sees the same total either way).
 *   DEVIATION (optics drive): yaAGC sends and clears 035/036 (the optics
 *   CDU angle counters, RegOPTY/RegOPTX); the drive counters the program
 *   writes are CDUTCMD/CDUSCMD (053/054), which are used here.
 *   DEVIATION (RHC): yaAGC loads the LM RHC counters from 0166-0170 at
 *   each write of channel 13 with bits 8 and 9 set; the sampled image
 *   gives no write events, so they are loaded once per centisecond while
 *   both bits are set (ZEROENBL sets them each DAP pass).
 *   Inputs (ChannelInput, WriteIO): see sim_input().
 *
 * ALGORITHM:
 *   none (state and packets)
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "internal.h"

#define CH_SLOTS          0200    /* channels 0..0177 (fictitious included)  */
#define CH_FIRST_UPDATE   011     /* connect update: channels 011..0177      */
#define CH_RELAY_ROWS     16      /* channel 010 relay rows (AAAA)           */
#define CH_RELAY_SHIFT    11      /* AAAA: channel 010 bits 12-15            */
#define CH_RELAY_ROW_MASK 017u    /* four row bits                           */
#define LAMP_TEMP         000010u /* 0163 bit 4: TEMP                        */
#define LAMP_KEY_REL      000020u /* 0163 bit 5: KEY REL                     */
#define LAMP_VN_FLASH     000040u /* 0163 bit 6: verb/noun dark (flash)      */
#define LAMP_OPER_ERR     000100u /* 0163 bit 7: OPR ERR                     */
#define LAMP_RESTART      000200u /* 0163 bit 8: RESTART                     */
#define LAMP_STBY         000400u /* 0163 bit 9: STBY                        */
#define LAMP_EL_OFF       001000u /* 0163 bit 10: display blanked (standby)  */
#define CH11_TEMP         000010u /* channel 11 bit 4: TEMP caution          */
#define CH11_KEY_REL      000020u /* channel 11 bit 5: KEY REL               */
#define CH11_VN_FLASH     000040u /* channel 11 bit 6: verb/noun flash       */
#define CH11_OPER_ERR     000100u /* channel 11 bit 7: OPR ERR               */
#define CH11_CAUT_RESET   001000u /* channel 11 bit 10: caution reset        */
#define CH13_LAMP_TEST    001000u /* channel 13 bit 10: DSKY light test      */
#define CH13_RHC_READ     000600u /* channel 13 bits 8, 9: RHC counters      */
#define CH14_GYRO_ACT     001000u /* channel 14 bit 10: gyro activity        */
#define CH14_GYRO_BITS    000740u /* channel 14 bits 6-9: gyro select, sign  */
#define CH14_GYRO_SHIFT   6       /* gyro bits -> 0177 bits 12-15            */
#define CH14_DRIVE_S      002000u /* channel 14 bit 11: drive CDU S          */
#define CH14_DRIVE_T      004000u /* channel 14 bit 12: drive CDU T          */
#define CH14_DRIVE_X      040000u /* channel 14 bit 15: drive CDU X          */
#define CH14_DRIVE_XYZ    070000u /* channel 14 bits 13-15: drive CDU X,Y,Z  */
#define CH30_POWER_UP     037777u /* yaAGC agc_engine_init: 030, TEMP in limits */
#define CH33_LATCHES      076000u /* channel 33 bits 11-15: internal latches */
#define FLASH_STEP_CS     32u     /* 0.32 s: yaAGC DSKY_OVERFLOW              */
#define FLASH_PERIOD      4u      /* 1.28 s: DSKY_FLASH_PERIOD quarters       */
#define SCALER_PER_CS     32u     /* 1/3200 s counts per centisecond          */
#define CDU_BURST         24      /* 192 / COARSE_SMOOTH pulses per burst     */
#define CDU_BURST_COUNTS  240u    /* IMUCDU_BURST_CYCLES: 75 ms in 1/3200 s   */
#define GYRO_FLUSH        03777u  /* packet limit when flushing a train       */
#define GYRO_FLUSH_MAX    17      /* 077777 / 03777 + 1 packets               */
#define GYRO_BURST        1024u   /* GYRO_BURST2: pulses per 0.25 s burst     */
#define GYRO_BURST_COUNTS 800u    /* GYRO_BURST: 0.25 s in 1/3200 s           */
#define KEY_RSET          022u    /* RSET key code                            */
#define SIGN15            040000u /* ones'-complement sign bit                */
#define MAG14             037777u /* ones'-complement magnitude bits          */
#define CH16_KEYS_MARKS   000177u /* nav DSKY key code + mark buttons         */

static uint16_t g_sent[CH_SLOTS];       /* last value broadcast             */
static uint16_t g_relay[CH_RELAY_ROWS]; /* last 010 word per relay row      */
static uint16_t g_rhc[3];               /* 0166-0170: pitch, yaw, roll      */
static uint16_t g_uplink;               /* 0173: last uplink word           */
static bool     g_restart_light;        /* yaAGC RestartLight               */
static uint32_t g_cdu_timer;            /* 1/3200 s since the last CDU burst */
static bool     g_cdu_left;             /* a CDU count was left (ImuChannel14) */
static uint16_t g_gyro_count;           /* GyroCount: pulses still to send   */
static uint16_t g_gyro_sel;             /* OldChannel14: gyro bits << 6      */
static uint32_t g_gyro_timer;           /* 1/3200 s toward the next burst    */

/* Output channels sampled for changes (010, 034, 035 are forced).        */
static const uint16_t k_outputs[] = {005, 006, 011, 012, 013, 014};

/**********************************************************************
 * AGC: none -- power-on state (yaAGC agc_engine_init): nothing sent yet,
 * channel 30 = 037777, RESTART lit by the first cycle.
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_status_t -- the status of the channel 30 store
 **********************************************************************/
agc_status_t sim_channels_init(void)
{
    for (int i = 0; i < CH_SLOTS; i++)
    {
        g_sent[i] = 0;
    }

    for (int i = 0; i < CH_RELAY_ROWS; i++)
    {
        g_relay[i] = 0;
    }

    for (int i = 0; i < 3; i++)
    {
        g_rhc[i] = 0;
    }

    g_uplink        = 0;
    g_restart_light = true;
    g_cdu_timer     = 0;
    g_cdu_left      = false;
    g_gyro_count    = 0;
    g_gyro_sel      = 0;
    g_gyro_timer    = 0;

    return (agc_chan_set_input((int) AGC_CHAN_CHAN30, CH30_POWER_UP));
}

/**********************************************************************
 * AGC: none -- broadcast a channel value when it differs from the last
 * one sent.
 *
 * ARGUMENTS:
 *   chan   uint16_t  I  channel, < 0200
 *   value  uint16_t  I  15-bit word
 **********************************************************************/
static void send_if_changed(uint16_t chan, uint16_t value)
{
    if (g_sent[chan] != value)
    {
        g_sent[chan] = value;
        sim_socket_send(chan, value);
    }
}

/**********************************************************************
 * AGC: none -- sample the output channels and broadcast the changes
 * (yaAGC ChannelOutput on CpuWriteIO); channel 11 bit 10 resets the
 * RESTART lamp (CpuWriteIO).
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_channels_sync(void)
{
    int n = (int) (sizeof (k_outputs) / sizeof (k_outputs[0])); /* channels sampled */

    sim_clock_arm_traps(); /* channel 13 bits 12-14 never read back */

    for (int i = 0; i < n; i++)
    {
        send_if_changed(k_outputs[i], agc_chan_read((int) k_outputs[i]));
    }

    if ((agc_chan_read((int) AGC_CHAN_DSALMOUT) & CH11_CAUT_RESET) != 0u)
    {
        g_restart_light = false;
    }
}

/**********************************************************************
 * AGC: none -- broadcast a channel whether or not it changed (channel
 * 010 rows are remembered for the connect update).
 *
 * ARGUMENTS:
 *   chan  uint16_t  I  channel 010, 034 or 035
 **********************************************************************/
void sim_channels_force(uint16_t chan)
{
    uint16_t v = agc_chan_read((int) chan); /* channel value */

    g_sent[chan] = v;
    sim_socket_send(chan, v);

    if (chan == (uint16_t) AGC_CHAN_OUT0)
    {
        g_relay[(v >> CH_RELAY_SHIFT) & CH_RELAY_ROW_MASK] = v;
    }
}

/**********************************************************************
 * AGC: none -- the value yaAGC's InputChannel[] holds for a channel of
 * the connect update: the channel image below 040, the stored RHC and
 * uplink inputs, and the 0163 lamps (DEVIATION: yaAGC keeps 0163
 * outside InputChannel[] and so never sends it on connect; a DSKY that
 * connects late would show no RESTART / STBY until the next change).
 *
 * ARGUMENTS:
 *   chan  uint16_t  I  channel, 011..0177
 *
 * RETURNS: uint16_t -- the value, 0 for a channel nothing stored
 **********************************************************************/
static uint16_t connect_value(uint16_t chan)
{
    uint16_t v = 0; /* nothing stored: not sent */

    if (chan < (uint16_t) AGC_CHAN_COUNT)
    {
        v = agc_chan_read((int) chan);
    }
    else if (chan >= SIM_CH_RHC_PITCH && chan <= SIM_CH_RHC_ROLL)
    {
        v = g_rhc[chan - SIM_CH_RHC_PITCH];
    }
    else if (chan == SIM_CH_UPLINK)
    {
        v = g_uplink;
    }
    else if (chan == SIM_CH_DSKY_LAMPS)
    {
        v = g_sent[chan];
    }
    else
    {
        /* pulse outputs (0171-0177) and unused channels hold nothing */
    }

    return (v);
}

/**********************************************************************
 * AGC: none -- yaAGC UpdateAgcPeripheralConnect: a new peripheral gets
 * every relay row and every channel 011-0177 that is not zero (zero is
 * the default and is not sent).
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_channels_send_all(void)
{
    for (int i = 0; i < CH_RELAY_ROWS; i++)
    {
        if (g_relay[i] != 0u)
        {
            sim_socket_send(AGC_CHAN_OUT0, g_relay[i]);
        }
    }

    for (uint16_t c = CH_FIRST_UPDATE; c < CH_SLOTS; c++)
    {
        uint16_t v = connect_value(c); /* stored value */

        if (v != 0u)
        {
            sim_socket_send(c, v);
        }
    }
}

/**********************************************************************
 * AGC: none -- yaAGC UpdateDSKY: the fictitious lamp channel 0163.
 *
 * ARGUMENTS:
 *   cs  uint32_t  I  centiseconds since power-on (the flash timer)
 **********************************************************************/
static void dsky_lamps(uint32_t cs)
{
    uint16_t c11 = agc_chan_read((int) AGC_CHAN_DSALMOUT); /* channel 11 */
    uint16_t c13 = agc_chan_read((int) AGC_CHAN_CHAN13);   /* channel 13 */
    uint16_t c30 = agc_chan_read((int) AGC_CHAN_CHAN30);   /* channel 30 */
    uint16_t v   = 0;                                      /* lamp word */

    v |= ((c13 & CH13_LAMP_TEST) != 0u) ? (LAMP_RESTART | LAMP_STBY) : 0u;   /* light test */
    v |= g_restart_light ? LAMP_RESTART : 0u;                                /* RESTART flip-flop */
    v |= sim_clock_standby() ? (LAMP_STBY | LAMP_EL_OFF) : 0u;               /* standby */
    v |= ((c11 & CH11_TEMP) != 0u || (c30 & SIGN15) != 0u) ? LAMP_TEMP : 0u; /* TEMP */
    v |= (uint16_t) (c11 & (CH11_KEY_REL | CH11_OPER_ERR));                  /* same bits in 0163 */

    if (((cs / FLASH_STEP_CS) % FLASH_PERIOD) == 0u && !sim_clock_standby())
    { /* the dark quarter of the 1.28 s flash cycle */
        v |= ((c11 & CH11_VN_FLASH) != 0u) ? LAMP_VN_FLASH : 0u; /* verb/noun off */
        v &= (uint16_t) ~(LAMP_KEY_REL | LAMP_OPER_ERR);
    }

    send_if_changed(SIM_CH_DSKY_LAMPS, v);
}

/**********************************************************************
 * AGC: none -- yaAGC BurstOutput: one coarse-align / error-counter burst
 * on the axes channel 14 drives, at most CDU_BURST pulses each.
 *
 * ARGUMENTS:
 *   c14  uint16_t  I  channel 14
 *
 * RETURNS: agc_status_t -- OK, or the first refused CDUxCMD write
 **********************************************************************/
static agc_status_t cdu_burst(uint16_t c14)
{
    agc_status_t st = AGC_STATUS_OK; /* first refused counter write */

    g_cdu_left = false;

    for (int axis = 0; axis < 3; axis++)
    {
        agc_sp_t cmd = agc_imu_cducmd(axis); /* CDUxCMD, B0 pulses */

        if ((c14 & (CH14_DRIVE_X >> axis)) != 0u && cmd != 0)
        {
            int32_t      mag  = (cmd < 0) ? -(int32_t) cmd : (int32_t) cmd;   /* |count| */
            int32_t      part = (mag > CDU_BURST) ? CDU_BURST : mag;          /* this burst */
            int32_t      left = (cmd < 0) ? -(mag - part) : (mag - part);     /* signed rest */
            agc_status_t s    = agc_imu_hw_set_cducmd(axis, (agc_sp_t) left); /* the rest stays */

            sim_socket_send((uint16_t) (SIM_CH_CDUX_DRV + (uint16_t) axis), (uint16_t) (((cmd < 0) ? SIGN15 : 0u) | (uint16_t) part)); /* sign | count */
            st         = (st == AGC_STATUS_OK) ? s : st;                                                                               /* keep the first failure */
            g_cdu_left = g_cdu_left || (left != 0);
        }
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- yaAGC IMU CDU drive pacing: a burst every 75 ms while
 * channel 14 bits 13-15 are set, at once when nothing was left over.
 *
 * ARGUMENTS:
 *   c14  uint16_t  I  channel 14
 *
 * RETURNS: agc_status_t -- OK, or the first refused CDUxCMD write
 **********************************************************************/
static agc_status_t cdu_drive(uint16_t c14)
{
    agc_status_t st = AGC_STATUS_OK; /* burst outcome */

    g_cdu_timer += SCALER_PER_CS;
    g_cdu_timer = (g_cdu_timer > CDU_BURST_COUNTS + SCALER_PER_CS) ? CDU_BURST_COUNTS + SCALER_PER_CS : g_cdu_timer; /* at most one burst due */

    if ((c14 & CH14_DRIVE_XYZ) != 0u && !g_cdu_left)
    {
        g_cdu_timer = 0; /* suddenly active: burst at once */
        st          = cdu_burst(c14);
    }
    else if ((c14 & CH14_DRIVE_XYZ) != 0u && g_cdu_timer >= CDU_BURST_COUNTS)
    {
        g_cdu_timer -= CDU_BURST_COUNTS;
        st          = cdu_burst(c14);
    }
    else
    {
        /* not driving, or the next burst is not due */
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- yaAGC gyro torquing (GYRO_TIMING_SIMULATED off): a new
 * GYROCMD flushes the pending train and starts a new one; bursts of at
 * most GYRO_BURST pulses every 0.25 s.
 *
 * ARGUMENTS:
 *   c14  uint16_t  I  channel 14
 **********************************************************************/
static void gyro_drive(uint16_t c14)
{
    uint16_t cmd = agc_imu_gyrocmd(); /* GYROCMD, pulses */

    if ((c14 & CH14_GYRO_ACT) != 0u && cmd != 0u)
    {
        for (int i = 0; i < GYRO_FLUSH_MAX && g_gyro_count > 0u; i++)
        {
            uint16_t part = (g_gyro_count > GYRO_FLUSH) ? (uint16_t) GYRO_FLUSH : g_gyro_count; /* one packet */

            sim_socket_send(SIM_CH_GYRO, (uint16_t) (g_gyro_sel | part));
            g_gyro_count = (uint16_t) (g_gyro_count - part);
        }

        g_gyro_count = cmd;
        g_gyro_sel   = (uint16_t) ((c14 & CH14_GYRO_BITS) << CH14_GYRO_SHIFT);
        g_gyro_timer = GYRO_BURST_COUNTS; /* first burst now */
        agc_imu_hw_set_gyrocmd(0);
    }
    else
    {
        g_gyro_timer += SCALER_PER_CS;
    }

    if (g_gyro_timer >= GYRO_BURST_COUNTS)
    {
        uint16_t part = (g_gyro_count > GYRO_BURST) ? (uint16_t) GYRO_BURST : g_gyro_count; /* this burst */

        g_gyro_timer -= GYRO_BURST_COUNTS;

        if (part != 0u)
        {
            sim_socket_send(SIM_CH_GYRO, (uint16_t) (g_gyro_sel | part));
            g_gyro_count = (uint16_t) (g_gyro_count - part);
        }
    }
}

/**********************************************************************
 * AGC: none -- optics (CSM) / RR (LM) / TVC (CSM SPS) drive: the whole
 * CDUTCMD / CDUSCMD count while channel 14 bit 12 / 11 is set.
 *
 * ARGUMENTS:
 *   c14  uint16_t  I  channel 14
 **********************************************************************/
static void optics_drive(uint16_t c14)
{
    static const uint16_t bit[2]  = {CH14_DRIVE_T, CH14_DRIVE_S};         /* drive enables */
    static const uint16_t chan[2] = {SIM_CH_OPT_T_DRV, SIM_CH_OPT_S_DRV}; /* 0171, 0172 */

    for (int which = 0; which < 2; which++)
    {
        agc_sp_t cmd = agc_erase_cdutcmd(which); /* CDUTCMD / CDUSCMD, B0 pulses */

        if ((c14 & bit[which]) != 0u && cmd != 0)
        {
            uint16_t word = (cmd < 0) ? (uint16_t) (~(uint16_t) (-(int32_t) cmd) & SIM_WORD_MASK) : (uint16_t) cmd; /* ones' complement, as yaAGC sends */

            sim_socket_send(chan[which], word);
            agc_erase_set_cdutcmd(which, 0);
        }
    }
}

/**********************************************************************
 * AGC: none -- a ones'-complement 15-bit word as a signed SP value.
 *
 * ARGUMENTS:
 *   w  uint16_t  I  15-bit word
 *
 * RETURNS: agc_sp_t -- the value (-0 -> 0)
 **********************************************************************/
static agc_sp_t sp_of_word(uint16_t w)
{
    agc_sp_t v = ((w & SIGN15) != 0u) ? (agc_sp_t) -(int32_t) ((uint16_t) ~w & MAG14) : (agc_sp_t) w; /* B0 count; negative: complement */

    return (v);
}

/**********************************************************************
 * AGC: none -- yaAGC ChannelOutput 013: with bits 8 and 9 set the RHC
 * angles from 0166-0170 go into the RHC counters (LM only; yaAGC
 * !CmOrLm). Q (pitch) at 042, P (yaw) at 043, R (roll) at 044.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void rhc_latch(void)
{
    uint16_t c13 = agc_chan_read((int) AGC_CHAN_CHAN13); /* channel 13 */

    if ((c13 & CH13_RHC_READ) == CH13_RHC_READ && agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_erase_set_q_rhcctr(sp_of_word(g_rhc[0]));
        agc_erase_set_p_rhcctr(sp_of_word(g_rhc[1]));
        agc_erase_set_r_rhcctr(sp_of_word(g_rhc[2]));
    }
}

/**********************************************************************
 * AGC: none -- once per centisecond: the 0163 lamps and, outside
 * standby (yaAGC returns before them), the drives and the RHC latch.
 *
 * ARGUMENTS:
 *   cs  uint32_t  I  centiseconds since power-on
 *
 * RETURNS: agc_status_t -- OK, or a refused CDUxCMD write
 **********************************************************************/
agc_status_t sim_channels_tick(uint32_t cs)
{
    agc_status_t st  = AGC_STATUS_OK;                        /* CDU drive outcome */
    uint16_t     c14 = agc_chan_read((int) AGC_CHAN_CHAN14); /* channel 14 */

    dsky_lamps(cs);

    if (!sim_clock_standby())
    {
        st = cdu_drive(c14);
        gyro_drive(c14);
        optics_drive(c14);
        rhc_latch();
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- yaAGC WriteIO of an input packet: the value through the
 * client's mask over the old value; channel 33 bits 11-15 are internal
 * latches a peripheral cannot write. DEVIATION: a packet for a channel
 * that is not an input (3, 4, 015, 016, 030-033) is ignored; yaAGC
 * would store it, so a peripheral could overwrite an output channel.
 *
 * ARGUMENTS:
 *   chan   uint16_t  I  channel, < 040
 *   value  uint16_t  I  15-bit word
 *   mask   uint16_t  I  the client's u-bit mask for chan
 *
 * RETURNS: bool -- true when stored
 **********************************************************************/
static bool input_channel(uint16_t chan, uint16_t value, uint16_t mask)
{
    uint16_t old = agc_chan_read((int) chan);                              /* current value */
    uint16_t v   = (uint16_t) ((value & mask) | (old & (uint16_t) ~mask)); /* merged */

    if (chan == (uint16_t) AGC_CHAN_CHAN33)
    {
        v = (uint16_t) ((old & CH33_LATCHES) | (v & (uint16_t) ~CH33_LATCHES));
    }

    return (agc_chan_set_input((int) chan, (uint16_t) (v & SIM_WORD_MASK)) == AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- one inbound packet that is not a mask or a counter
 * (yaAGC ChannelInput, WriteIO):
 *   015  main DSKY key code -> KEYRUPT1;
 *   016  nav DSKY key / mark buttons -> KEYRUPT2 (MARKRUPT) when a key
 *        or mark bit is set (DEVIATION: yaAGC raises no KEYRUPT2; the
 *        AGC hardware does); RSET (022) on 015 or 016 resets RESTART;
 *   0173 uplink word -> INLINK and UPRUPT;
 *   0166-0170 RHC angles, kept until channel 13 asks for them, echoed.
 *
 * ARGUMENTS:
 *   chan   uint16_t  I  channel, < 0200
 *   value  uint16_t  I  15-bit word
 *   mask   uint16_t  I  the client's u-bit mask for chan
 *
 * RETURNS: bool -- true when the packet requested an interrupt
 **********************************************************************/
bool sim_input(uint16_t chan, uint16_t value, uint16_t mask)
{
    bool     rupt = false;                     /* an interrupt was requested */
    uint16_t v    = (uint16_t) (value & mask); /* the masked value */

    if (chan == SIM_CH_UPLINK)
    {
        g_uplink = (uint16_t) (v | (g_uplink & (uint16_t) ~mask));
        agc_erase_set_inlink(g_uplink);
        sim_irq_request(SIM_IRQ_UPLINK);
        rupt     = true;
    }
    else if (chan >= SIM_CH_RHC_PITCH && chan <= SIM_CH_RHC_ROLL)
    {
        uint16_t *slot = &g_rhc[chan - SIM_CH_RHC_PITCH]; /* LastRhcPitch.. */

        *slot = (uint16_t) (v | (*slot & (uint16_t) ~mask));
        sim_socket_send(chan, *slot); /* yaAGC echoes it */
    }
    else if (chan < (uint16_t) AGC_CHAN_COUNT && input_channel(chan, value, mask))
    {
        uint16_t now = agc_chan_read((int) chan); /* stored value */

        g_restart_light = g_restart_light && !((chan == (uint16_t) AGC_CHAN_MNKEYIN || chan == (uint16_t) AGC_CHAN_NAVKEYIN) && now == KEY_RSET);

        if (chan == (uint16_t) AGC_CHAN_MNKEYIN)
        {
            sim_irq_request(SIM_IRQ_KEY1);
            rupt = true;
        }
        else if (chan == (uint16_t) AGC_CHAN_NAVKEYIN && (now & CH16_KEYS_MARKS) != 0u)
        {
            sim_irq_request(SIM_IRQ_KEY2);
            rupt = true;
        }
        else
        {
            /* a discrete or a key release: no interrupt */
        }
    }
    else
    {
        /* not an input channel: ignored (DEVIATION in input_channel) */
    } /* which channel */

    return (rupt);
} /* sim_input */
