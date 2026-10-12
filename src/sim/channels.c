/*
 * channels.c -- the I/O channels as Virtual AGC peripherals see them.
 *
 * Outputs (yaAGC SocketAPI.c ChannelOutput): yaAGC broadcasts every
 * program WRITE/WOR/WAND of a channel. The translated program writes the
 * channel image in src/core/channels.c directly, so the harness samples it
 * after every interrupt and every job and broadcasts what changed; channel
 * 010 (DSKY relays) is sent after every T4RUPT, which writes it on every
 * pass, and 034/035 (downlink) after every DOWNRUPT, so repeated words
 * reach the peripherals as they would from yaAGC.
 *
 * Fictitious outputs kept as yaAGC computes them (agc_engine.c):
 *   0163       DSKY lamps not wired to a channel bit: KEY REL, OPR ERR and
 *              V/N flash (1.28 s period, off for the first 0.32 s, KEY REL
 *              and OPR ERR flash with it), RESTART, STBY (light test),
 *              TEMP (channel 11 bit 4 or channel 30 bit 15); UpdateDSKY;
 *   0174-0176  IMU CDU coarse-align drive (CDUXCMD..CDUZCMD while channel
 *              14 bits 15-13 are set), at most 192 pulses per burst;
 *   0177       gyro torque pulses (GYROCMD while channel 14 bit 10 is set),
 *              ((channel 14 bits 6-9) << 6) | count;
 *   0171/0172  optics trunnion / shaft drive (CDUTCMD / CDUSCMD while
 *              channel 14 bit 12 / 11 is set).
 * Inputs (ChannelInput): see sim_input().
 *
 * DEVIATION: an output written twice with the same value between two
 * samples, or changed and changed back inside one job, is sent once or not
 * at all (yaAGC sends each write). Only the DSKY relays and the downlink
 * rely on repeats; they are forced as above.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "internal.h"

#define CH_SLOTS        0200    /* channels 0..0177 (fictitious included)  */
#define CH_RELAY_ROWS   16
#define LAMP_AGC_WARN   000001u
#define LAMP_TEMP       000010u
#define LAMP_KEY_REL    000020u
#define LAMP_VN_FLASH   000040u
#define LAMP_OPER_ERR   000100u
#define LAMP_RESTART    000200u
#define LAMP_STBY       000400u
#define CH11_TEMP       000010u
#define CH11_KEY_REL    000020u
#define CH11_VN_FLASH   000040u
#define CH11_OPER_ERR   000100u
#define CH11_CAUT_RESET 001000u
#define CH13_LAMP_TEST  001000u
#define CH13_RHC_READ   000600u /* RHC counter enable + start           */
#define CH14_GYRO_ACT   001000u
#define CH14_GYRO_BITS  000740u
#define CH14_DRIVE_S    002000u
#define CH14_DRIVE_T    004000u
#define CH14_DRIVE_X    040000u
#define CH30_TEMP_BAD   040000u
#define CH30_POWER_UP   037777u /* yaAGC: channel 30 with TEMP OK      */
#define FLASH_STEP_CS   32u     /* 0.32 s                               */
#define FLASH_PERIOD    4u      /* 1.28 s                               */
#define CDU_BURST       192
#define GYRO_BURST      03777u
#define GYRO_PACKETS    17      /* 077777 / 03777 + 1                  */
#define KEY_RSET        022u
#define SIGN15          040000u
#define CH16_KEYS_MARKS 000177u /* nav DSKY key code + mark buttons    */

static uint16_t g_sent[CH_SLOTS];       /* last value broadcast     */
static uint16_t g_relay[CH_RELAY_ROWS]; /* last 010 word per relay  */
static uint16_t g_rhc[3];               /* 0166-0170 pitch yaw roll */
static bool     g_restart_light;        /* yaAGC RestartLight       */

/* Output channels sampled for changes (010, 034, 035 are forced).        */
static const uint16_t k_outputs[] = {005, 006, 011, 012, 013, 014};

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

    g_rhc[0]        = 0;
    g_rhc[1]        = 0;
    g_rhc[2]        = 0;
    g_restart_light = true; /* lit by the power-on GOJAM */

    return (agc_chan_set_input((int) AGC_CHAN_CHAN30, CH30_POWER_UP));
}

static void send_if_changed(uint16_t chan, uint16_t value)
{
    if (g_sent[chan] != value)
    {
        g_sent[chan] = value;
        sim_socket_send(chan, value);
    }
}

void sim_channels_sync(void)
{
    int n = (int) (sizeof (k_outputs) / sizeof (k_outputs[0]));

    for (int i = 0; i < n; i++)
    {
        uint16_t c = k_outputs[i];

        send_if_changed(c, agc_chan_read((int) c));
    }

    if ((agc_chan_read((int) AGC_CHAN_DSALMOUT) & CH11_CAUT_RESET) != 0u)
    {
        g_restart_light = false; /* channel 11 bit 10 resets RESTART */
    }
}

void sim_channels_force(uint16_t chan)
{
    uint16_t v = agc_chan_read((int) chan);

    g_sent[chan] = v;
    sim_socket_send(chan, v);

    if (chan == (uint16_t) AGC_CHAN_OUT0)
    {
        g_relay[(v >> 11) & 017u] = v; /* AAAA: relay row               */
    }
}

/* yaAGC UpdateAgcPeripheralConnect: a new peripheral gets every output
 * that is not zero (zero is the default and is not sent).             */
void sim_channels_send_all(void)
{
    for (int i = 1; i < CH_RELAY_ROWS; i++)
    {
        if (g_relay[i] != 0u)
        {
            sim_socket_send(AGC_CHAN_OUT0, g_relay[i]);
        }
    }

    for (uint16_t c = 011; c < CH_SLOTS; c++)
    {
        uint16_t v = (c < AGC_CHAN_COUNT && c != (uint16_t) AGC_CHAN_OUT0) ? agc_chan_read((int) c) : g_sent[c];

        if (v != 0u && c != SIM_CH_STEP_ACK)
        {
            sim_socket_send(c, v);
        }
    }
}

/* yaAGC UpdateDSKY: the fictitious lamp channel 0163.                   */
static void dsky_lamps(uint32_t cs)
{
    uint16_t c11 = agc_chan_read((int) AGC_CHAN_DSALMOUT);
    uint16_t c13 = agc_chan_read((int) AGC_CHAN_CHAN13);
    uint16_t c30 = agc_chan_read((int) AGC_CHAN_CHAN30);
    uint16_t v   = 0;

    v |= ((c13 & CH13_LAMP_TEST) != 0u) ? (LAMP_RESTART | LAMP_STBY) : 0u;
    v |= g_restart_light ? LAMP_RESTART : 0u;
    v |= ((c11 & CH11_TEMP) != 0u || (c30 & CH30_TEMP_BAD) != 0u) ? LAMP_TEMP : 0u;
    v |= ((c11 & CH11_KEY_REL) != 0u) ? LAMP_KEY_REL : 0u;
    v |= ((c11 & CH11_OPER_ERR) != 0u) ? LAMP_OPER_ERR : 0u;

    if (((cs / FLASH_STEP_CS) % FLASH_PERIOD) == 0u)
    { /* the dark quarter of the 1.28 s flash cycle                    */
        v |= ((c11 & CH11_VN_FLASH) != 0u) ? LAMP_VN_FLASH : 0u;
        v &= (uint16_t) ~(LAMP_KEY_REL | LAMP_OPER_ERR);
    }

    send_if_changed(SIM_CH_DSKY_LAMPS, v);
}

/* yaAGC BurstOutput: CDUXCMD..CDUZCMD while channel 14 drives the axis. */
static agc_status_t cdu_drive(uint16_t c14)
{
    agc_status_t st = AGC_STATUS_OK; /* first refused counter write */

    for (int axis = 0; axis < 3; axis++)
    {
        agc_sp_t cmd = agc_imu_cducmd(axis);

        if ((c14 & (CH14_DRIVE_X >> axis)) == 0u || cmd == 0)
        {
            continue;
        }

        int32_t mag  = (cmd < 0) ? -(int32_t) cmd : (int32_t) cmd;
        int32_t part = (mag > CDU_BURST) ? CDU_BURST : mag;
        int32_t left = (cmd < 0) ? -(mag - part) : (mag - part);

        sim_socket_send((uint16_t) (SIM_CH_CDUX_DRV + (uint16_t) axis), (uint16_t) (((cmd < 0) ? SIGN15 : 0u) | (uint16_t) part));
        agc_status_t s = agc_imu_hw_set_cducmd(axis, (agc_sp_t) left);

        st = (st == AGC_STATUS_OK) ? s : st; /* keep the first failure */
    }

    return (st);
}

/* Gyro torquing: GYROCMD is emptied while channel 14 bit 10 is set.     */
static void gyro_drive(uint16_t c14)
{
    uint16_t left = agc_imu_gyrocmd();

    if (!((c14 & CH14_GYRO_ACT) == 0u || left == 0u))
    {
        uint16_t sel = (uint16_t) ((c14 & CH14_GYRO_BITS) << 6);

        for (int i = 0; i < GYRO_PACKETS && left > 0u; i++)
        {
            uint16_t part = (left > GYRO_BURST) ? (uint16_t) GYRO_BURST : left;

            sim_socket_send(SIM_CH_GYRO, (uint16_t) (sel | part));
            left = (uint16_t) (left - part);
        }

        agc_imu_hw_set_gyrocmd(0);
    }
}

/* Optics (CSM) / RR (LM) / TVC (CSM SPS) drive: CDUTCMD, CDUSCMD.        */
static void optics_drive(uint16_t c14)
{
    static const uint16_t bit[2]  = {CH14_DRIVE_T, CH14_DRIVE_S};
    static const uint16_t chan[2] = {SIM_CH_OPT_T_DRV, SIM_CH_OPT_S_DRV};

    for (int which = 0; which < 2; which++)
    {
        agc_sp_t cmd = agc_erase_cdutcmd(which);

        if ((c14 & bit[which]) == 0u || cmd == 0)
        {
            continue;
        }

        uint16_t word = (cmd < 0) ? (uint16_t) (~(uint16_t) (-(int32_t) cmd) & SIM_WORD_MASK) : (uint16_t) cmd; /* ones' complement, as yaAGC sends */

        sim_socket_send(chan[which], word);
        agc_erase_set_cdutcmd(which, 0);
    }
}

/* A ones'-complement 15-bit word as a signed SP value (-0 -> 0).       */
static agc_sp_t sp_of_word(uint16_t w)
{
    agc_sp_t v = ((w & SIGN15) != 0u) ? (agc_sp_t) -(int32_t) ((uint16_t) ~w & 037777u) : (agc_sp_t) w;

    return (v);
}

/* yaAGC ChannelOutput 013: with bits 8 and 9 set the RHC angles from
 * 0166-0170 go into the RHC counters (LM; the CSM has no RHC counters). */
static void rhc_latch(void)
{
    uint16_t c13 = agc_chan_read((int) AGC_CHAN_CHAN13);

    if (!((c13 & CH13_RHC_READ) != CH13_RHC_READ || agc_state_vehicle() != AGC_VEHICLE_LM))
    {
        agc_sp_t q = sp_of_word(g_rhc[0]);
        agc_sp_t p = sp_of_word(g_rhc[1]);
        agc_sp_t r = sp_of_word(g_rhc[2]);

        agc_erase_set_q_rhcctr(q); /* pitch */
        agc_erase_set_p_rhcctr(p); /* yaw, as yaAGC stores it at 043 */
        agc_erase_set_r_rhcctr(r);
    }
}

agc_status_t sim_channels_tick(uint32_t cs)
{
    uint16_t c14 = agc_chan_read((int) AGC_CHAN_CHAN14);

    dsky_lamps(cs);
    agc_status_t st = cdu_drive(c14);

    gyro_drive(c14);
    optics_drive(c14);
    rhc_latch();

    return (st);
}

/* Store an input channel through the client's mask (ChannelInput); false
 * when chan is not an input channel (the packet is ignored, as yaAGC does). */
static bool input_channel(uint16_t chan, uint16_t value, uint16_t mask)
{
    uint16_t old = agc_chan_read((int) chan);
    uint16_t v   = (uint16_t) ((value & mask) | (old & (uint16_t) ~mask));


    return (agc_chan_set_input((int) chan, (uint16_t) (v & SIM_WORD_MASK)) == AGC_STATUS_OK);
}

/* One inbound packet that is not a mask or a counter (yaAGC
 * ChannelInput). Returns true when it requested an interrupt:
 *   015  main DSKY key code -> KEYRUPT1 (RSET also resets RESTART);
 *   016  nav DSKY key / mark buttons -> KEYRUPT2 (MARKRUPT) when a key
 *        or mark bit is set (DEVIATION: yaAGC raises no KEYRUPT2; the
 *        AGC hardware does);
 *   0173 uplink word -> INLINK and UPRUPT;
 *   0166-0170 RHC angles, kept until channel 13 asks for them.         */
bool sim_input(uint16_t chan, uint16_t value, uint16_t mask)
{
    if (chan == SIM_CH_UPLINK)
    {
        agc_erase_set_inlink((uint16_t) (value & mask));
        sim_irq_request(SIM_IRQ_UPLINK);

        return (true);
    }

    if (chan >= SIM_CH_RHC_PITCH && chan <= SIM_CH_RHC_ROLL)
    {
        g_rhc[chan - SIM_CH_RHC_PITCH] = (uint16_t) (value & mask);
        sim_socket_send(chan, value); /* yaAGC echoes it */
    }
    else
    {
        if (chan < AGC_CHAN_COUNT)
        {
            if (input_channel(chan, value, mask)) /* not an input channel: ignored */
            {
                if (chan == (uint16_t) AGC_CHAN_MNKEYIN)
                {
                    g_restart_light = g_restart_light && ((value & 037u) != KEY_RSET);
                    sim_irq_request(SIM_IRQ_KEY1);

                    return (true);
                }

                if (chan == (uint16_t) AGC_CHAN_NAVKEYIN && (value & CH16_KEYS_MARKS) != 0u)
                {
                    sim_irq_request(SIM_IRQ_KEY2);

                    return (true);
                }
            }
        }
    }

    return (false);
}
