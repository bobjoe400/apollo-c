/**********************************************************************
 * FILE NAME: harness.c
 *
 * PURPOSE:
 *   Unit tests of the sim harness (src/sim/ without main.c) against
 *   yaAGC's behaviour: packet format, unprogrammed counter increments,
 *   the scaler-phase timing of TIME6 / T6RUPT, the radar cycle and
 *   RADARUPT, the HANDRUPT traps, the channel 33 latches, the CDU
 *   drive pacing, one packet per output-channel write, the CDU
 *   count FIFO, and the two LR reads of one SERVICER pass on the
 *   radar cycle (LRHJOB, then LRVJOB 5 cs later: no 01210).
 *
 * AGC SOURCE:
 *   No AGC routine: virtualagc yaAGC/agc_utilities.c (FormIoPacket,
 *   ParseIoPacket), yaAGC/agc_engine.c (Counter*, counter-timers,
 *   HANDRUPT, RADARUPT, BurstOutput, WriteIO).
 *
 * EXTERNAL VARIABLES:
 *   none (test counters are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   sim_packet_form/_parse, sim_counter_increment, sim_clock_*,
 *   sim_channels_*, sim_input   the harness under test (internal.h)
 *   agc_power_on, agc_rupt_go   power-on as main.c does it
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails.
 **********************************************************************/
#define _POSIX_C_SOURCE 200809L /* POSIX.1-2008: sockets, poll */

#include <stdio.h>

#include <netinet/in.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../../src/sim/internal.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"

#define CH13_RADAR   000010u /* channel 13 bit 4: radar activity */
#define CH13_T31A    004000u /* channel 13 bit 12: trap 31A      */
#define CH13_T6ENB   040000u /* channel 13 bit 15: TIME6 enable  */
#define CH14_DRIVE_X 040000u /* channel 14 bit 15: drive CDU X   */
#define NEG0         077777u /* ones'-complement -0              */

static int g_checks;   /* checks run */
static int g_failures; /* checks failed */

static void check(bool ok, int line, const char *text);
static void power_on(agc_vehicle_t v);
static void test_packets(void);
static void test_counters(void);
static void test_time6(void);
static void test_radar(void);
static void test_handrupt(void);
static void test_channel33(void);
static void test_cdu_drive(void);
static void test_output_writes(void);
static void test_cdu_fifo(void);
static void test_lr_reads(void);
static int  client_connect(int port);
static int  client_count(int fd, uint16_t chan);

/**********************************************************************
 * AGC: none -- run every test; print the totals.
 *
 * ARGUMENTS: none
 *
 * RETURNS: int -- 0 when every check passed, else 1
 **********************************************************************/
int main(void)
{
    test_packets();
    power_on(AGC_VEHICLE_CSM);
    test_counters();
    test_time6();
    test_channel33();
    test_cdu_drive();
    test_output_writes();
    test_cdu_fifo();
    power_on(AGC_VEHICLE_LM);
    test_radar();
    test_handrupt();
    test_lr_reads();
    (void) printf("test_harness: %d checks, %d failures\n", g_checks, g_failures); /* stdlib */

    return ((g_failures == 0) ? 0 : 1); /* exit status */
}

/**********************************************************************
 * AGC: none -- count one check, report a failure.
 *
 * ARGUMENTS:
 *   ok    bool          I  the check held
 *   line  int           I  source line
 *   text  const char *  I  the check as written
 **********************************************************************/
static void check(bool ok, int line, const char *text)
{
    g_checks++;

    if (!ok)
    {
        g_failures++;
        (void) printf("FAIL harness.c:%d: %s\n", line, text); /* stdlib */
    }
}

#define CHECK(c) check((c), __LINE__, #c) /* count and report a check */

/**********************************************************************
 * AGC: none -- power-on as main.c does it (no rope, no sockets); once
 * per vehicle: the host power-on is not repeatable in one process (the
 * display interface keeps its state, as erasable memory would).
 *
 * ARGUMENTS:
 *   v  agc_vehicle_t  I  vehicle
 **********************************************************************/
static void power_on(agc_vehicle_t v)
{
    agc_power_on(v);
    sim_clock_init();
    CHECK(agc_rupt_go() == AGC_STATUS_OK);
    CHECK(sim_channels_init() == AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- FormIoPacket / ParseIoPacket: the bytes of a DSKY RSET
 * packet, a round trip over every 8-bit channel, the u-bit, a bad
 * signature.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_packets(void)
{
    uint8_t  p[SIM_PACKET_BYTES]; /* one packet */
    uint16_t chan  = 0;           /* parsed channel */
    uint16_t value = 0;           /* parsed value */
    bool     ubit  = true;        /* parsed u-bit */
    bool     all   = true;        /* every round trip held */

    sim_packet_form(015, 022, p);
    CHECK(p[0] == 0x01u && p[1] == 0x68u && p[2] == 0x80u && p[3] == 0xD2u);

    for (uint16_t c = 0; c < 0400u; c++)
    {
        sim_packet_form(c, (uint16_t) (077777u - c), p);
        all = all && sim_packet_parse(p, &chan, &value, &ubit) && chan == c && value == (uint16_t) (077777u - c) && !ubit;
    }

    CHECK(all);
    sim_packet_form(032, 020000, p);
    p[0] |= 0x20u;
    CHECK(sim_packet_parse(p, &chan, &value, &ubit) && ubit && chan == 032u);
    p[2] = 0xFFu;
    CHECK(!sim_packet_parse(p, &chan, &value, &ubit));
}

/**********************************************************************
 * AGC: none -- UnprogrammedIncrement: PINC/MINC overflow to +-0, the
 * -0 -> +1 step, PCDU/MCDU wrap (CDUT: not paced), SHINC/SHANC in 14 bits, DINC toward
 * zero and holding at zero, refused counters and types.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_counters(void)
{
    agc_erase_set_pipax(16383);
    CHECK(sim_counter_increment(037, 0) == AGC_STATUS_OK && agc_erase_pipax() == 0);
    CHECK(sim_counter_increment(037, 2) == AGC_STATUS_OK && agc_erase_pipax() == -1);
    agc_erase_set_pipax(-16383);
    CHECK(sim_counter_increment(037, 2) == AGC_STATUS_OK && agc_erase_pipax() == 0);
    agc_erase_set_cdux(NEG0);
    CHECK(sim_counter_increment(032, 0) == AGC_STATUS_OK && agc_erase_cdux() == 1u);
    agc_erase_set_cdut(077777u);
    CHECK(sim_counter_increment(035, 1) == AGC_STATUS_OK && agc_erase_cdut() == 0u);
    CHECK(sim_counter_increment(035, 023) == AGC_STATUS_OK && agc_erase_cdut() == 077777u);
    agc_erase_set_rnrad(020001u);
    CHECK(sim_counter_increment(046, 5) == AGC_STATUS_OK && agc_erase_rnrad() == 2u);
    CHECK(sim_counter_increment(046, 6) == AGC_STATUS_OK && agc_erase_rnrad() == 5u);
    agc_erase_set_cdutcmd(0, 2);
    CHECK(sim_counter_increment(053, 4) == AGC_STATUS_OK && agc_erase_cdutcmd(0) == 1);
    CHECK(sim_counter_increment(053, 4) == AGC_STATUS_OK && agc_erase_cdutcmd(0) == 0);
    CHECK(sim_counter_increment(053, 4) == AGC_STATUS_OK && agc_erase_cdutcmd(0) == 0);
    CHECK(sim_counter_increment(053, 0) == AGC_STATUS_NOT_FOUND);
    CHECK(sim_counter_increment(024, 0) == AGC_STATUS_NOT_FOUND);
    CHECK(sim_counter_increment(037, 7) == AGC_STATUS_BAD_ARG);
}

/**********************************************************************
 * AGC: none -- TIME6 counts 16 DINCs a centisecond and requests T6RUPT
 * at the DINC after reaching -0 (ZOUT), resetting channel 13 bit 15.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_time6(void)
{
    sim_stats_t s; /* run statistics */

    agc_rupt_set_time(6, 16);
    agc_chan_or((int) AGC_CHAN_CHAN13, CH13_T6ENB);
    sim_clock_tick();
    sim_clock_stats(&s);
    CHECK(s.rupts[SIM_IRQ_T6] == 0u && agc_rupt_time(6) == NEG0);
    sim_clock_tick();
    sim_clock_stats(&s);
    CHECK(s.rupts[SIM_IRQ_T6] == 1u && (agc_chan_read((int) AGC_CHAN_CHAN13) & CH13_T6ENB) == 0u);
}

/**********************************************************************
 * AGC: none -- the radar gate counts TIME5 phases while channel 13 bit
 * 4 is set and keeps its count while it is clear; at 9 the cycle ends
 * at LOSCALAR phase 036 with RADARUPT and bit 4 reset.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_radar(void)
{
    sim_stats_t s; /* run statistics */

    agc_chan_or((int) AGC_CHAN_CHAN13, CH13_RADAR);

    for (int i = 0; i < 5; i++)
    {
        sim_clock_tick(); /* gates 1-5 */
    }

    agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) ~CH13_RADAR);
    sim_clock_tick();
    agc_chan_or((int) AGC_CHAN_CHAN13, CH13_RADAR);

    for (int i = 0; i < 4; i++)
    {
        sim_clock_tick(); /* gates 6-9 */
    }

    sim_clock_stats(&s);
    CHECK(s.rupts[SIM_IRQ_RADAR] == 0u);
    sim_clock_tick(); /* phase 036 */
    sim_clock_stats(&s);
    CHECK(s.rupts[SIM_IRQ_RADAR] == 1u);
}

/**********************************************************************
 * AGC: none -- channel 13 bit 12 arms trap 31A and reads back 0; the
 * trap fires once when channel 31 bits 1-6 leave all-ones.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_handrupt(void)
{
    sim_stats_t s; /* run statistics */

    agc_chan_or((int) AGC_CHAN_CHAN13, CH13_T31A);
    sim_channels_sync();
    CHECK((agc_chan_read((int) AGC_CHAN_CHAN13) & CH13_T31A) == 0u);
    sim_clock_tick();
    sim_clock_stats(&s);
    CHECK(s.rupts[SIM_IRQ_HAND] == 0u);
    CHECK(!sim_input(031, 077776u, 077777u));
    sim_clock_tick();
    sim_clock_tick();
    sim_clock_stats(&s);
    CHECK(s.rupts[SIM_IRQ_HAND] == 1u);
}

/**********************************************************************
 * AGC: none -- yaAGC WriteIO: channel 33 bits 11-15 are internal
 * latches a peripheral cannot write; bits 1-10 take the packet.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_channel33(void)
{
    CHECK(!sim_input(033, 0, 077777u));
    CHECK(agc_chan_read((int) AGC_CHAN_CHAN33) == 076000u);
    CHECK(sim_input(015, 022, 077777u)); /* RSET: KEYRUPT1 */
}

/**********************************************************************
 * AGC: none -- BurstOutput pacing: at most 24 pulses per axis, the
 * first burst at once, the next 75 ms later (8 cs, then 7: 240 counts
 * of 1/3200 s); the rest stays in CDUXCMD.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_cdu_drive(void)
{
    CHECK(agc_imu_hw_set_cducmd(0, 100) == AGC_STATUS_OK);
    agc_chan_or((int) AGC_CHAN_CHAN14, CH14_DRIVE_X);
    CHECK(sim_channels_tick(0) == AGC_STATUS_OK && agc_imu_cducmd(0) == 76);

    for (uint32_t cs = 1; cs < 8; cs++)
    {
        CHECK(sim_channels_tick(cs) == AGC_STATUS_OK && agc_imu_cducmd(0) == 76);
    }

    CHECK(sim_channels_tick(8) == AGC_STATUS_OK && agc_imu_cducmd(0) == 52);

    for (uint32_t cs = 9; cs < 15; cs++)
    {
        CHECK(sim_channels_tick(cs) == AGC_STATUS_OK && agc_imu_cducmd(0) == 52);
    }

    CHECK(sim_channels_tick(15) == AGC_STATUS_OK && agc_imu_cducmd(0) == 28);
}

/**********************************************************************
 * AGC: none -- a TCP client on 127.0.0.1:port (the peripheral side).
 *
 * ARGUMENTS:
 *   port  int  I  port to connect to
 *
 * RETURNS: int -- the socket, -1 when the connection failed
 **********************************************************************/
static int client_connect(int port)
{
    struct sockaddr_in a  = {0};                             /* server address */
    int                fd = socket(AF_INET, SOCK_STREAM, 0); /* client socket */

    a.sin_family      = AF_INET;
    a.sin_port        = htons((uint16_t) port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (fd >= 0 && connect(fd, (const struct sockaddr *) &a, sizeof (a)) != 0)
    {
        (void) close(fd); /* stdlib: connection refused */
        fd = -1;
    }

    return (fd);
}

/**********************************************************************
 * AGC: none -- read every packet that arrives within 200 ms of the last
 * one; count those for one channel.
 *
 * ARGUMENTS:
 *   fd    int       I  client socket
 *   chan  uint16_t  I  channel to count
 *
 * RETURNS: int -- packets for chan
 **********************************************************************/
static int client_count(int fd, uint16_t chan)
{
    int           n   = 0;               /* packets for chan */
    struct pollfd pfd = {fd, POLLIN, 0}; /* wait for input */

    for (int i = 0; i < 4096 && poll(&pfd, 1, 200) > 0; i++)
    {
        uint8_t  p[SIM_PACKET_BYTES]; /* one packet */
        uint16_t c = 0;               /* its channel */
        uint16_t v = 0;               /* its value */
        bool     u = false;           /* its u-bit */

        if (recv(fd, p, SIM_PACKET_BYTES, MSG_WAITALL) != (ssize_t) SIM_PACKET_BYTES)
        {
            break;
        }

        n += (sim_packet_parse(p, &c, &v, &u) && c == chan) ? 1 : 0; /* a packet for chan */
    }

    return (n);
}

/**********************************************************************
 * AGC: none -- yaAGC ChannelOutput on every CpuWriteIO: each write of
 * channel 5 sends a packet, also one that leaves the value unchanged and
 * two inside one job (the LM Simulator applies the jets of the packet
 * before the last one, so it needs every write the DAP makes;
 * closed-loop-fixes S-L1); a sample with no write sends nothing.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_output_writes(void)
{
    int port = 0; /* base port opened */

    for (int base = 27990; base < 28390 && port == 0; base += 100)
    {
        port = (sim_socket_open(base) == AGC_STATUS_OK) ? base : 0; /* opened: this base */
    }

    CHECK(port != 0);
    int fd = (port != 0) ? client_connect(port) : -1; /* the peripheral */

    CHECK(fd >= 0);
    sim_socket_poll();                              /* accept: connect update */
    sim_channels_sync();
    (void) client_count(fd, 0u);                    /* drain what is there */
    agc_chan_write((int) AGC_CHAN_PYJETS, 000041u);
    sim_channels_sync();
    agc_chan_write((int) AGC_CHAN_PYJETS, 000041u); /* same value again */
    agc_chan_or((int) AGC_CHAN_PYJETS, 000002u);    /* a second write in the same job */
    sim_channels_sync();
    sim_channels_sync();                            /* no write: nothing */
    CHECK(client_count(fd, (uint16_t) AGC_CHAN_PYJETS) == 3);

    if (fd >= 0)
    {
        (void) close(fd); /* stdlib */
    }

    sim_socket_close();
}

/**********************************************************************
 * AGC: none -- yaAGC PushCduFifo / ServiceCduFifo: PCDU / MCDU on
 * CDUX-CDUZ are applied at 400 counts a second (slow, 4 per cs) or 6400
 * (fast, 64 per cs), in arrival order (closed-loop-fixes S-L2).
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_cdu_fifo(void)
{
    bool ok = true; /* every increment accepted */

    agc_erase_set_cduy(0u);

    for (int i = 0; i < 10; i++)
    {
        ok = ok && sim_counter_increment(033, 1) == AGC_STATUS_OK; /* slow PCDU */
    }

    for (int i = 0; i < 100; i++)
    {
        ok = ok && sim_counter_increment(033, 023) == AGC_STATUS_OK; /* fast MCDU */
    }

    CHECK(ok && agc_erase_cduy() == 0u);
    CHECK(sim_channels_tick(100) == AGC_STATUS_OK && agc_erase_cduy() == 4u);
    CHECK(sim_channels_tick(101) == AGC_STATUS_OK && agc_erase_cduy() == 8u);
    CHECK(sim_channels_tick(102) == AGC_STATUS_OK && agc_erase_cduy() == (uint16_t) (0100000u - 22u)); /* 2 slow, 32 fast: 10 - 32 */
    CHECK(sim_channels_tick(103) == AGC_STATUS_OK && agc_erase_cduy() == (uint16_t) (0100000u - 86u)); /* 64 fast more  */
    CHECK(sim_channels_tick(104) == AGC_STATUS_OK && agc_erase_cduy() == (uint16_t) (0100000u - 90u)); /* the last 4    */
}

/**********************************************************************
 * AGC: none -- the two LR reads of one SERVICER pass on the radar cycle
 * (Luminary SERVICER.agc pp.872, 889, 892-893): LRHTASK starts LRHJOB
 * (RADSTALL for one altitude sample), and 5 cs later, where the C
 * SERVICER reaches VALTCHK, LRVJOB starts. LRVJOB must not RADSTALL
 * while LRHJOB holds RADCADR (BAILOUT 01210, NASSP PDI 2026-10-13); it
 * reads its beam after LRHJOB's sample.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void test_lr_reads(void)
{
    bool lrv_read  = false; /* LRVJOB reached RADSTALL */
    bool two_users = false; /* 01210 in FAILREG */

    for (int i = 0; i < 3; i++)
    {
        agc_erase_set_failreg(i, 0);
    }

    CHECK(agc_exec_novac(AGC_PRIO32, AGC_JOB_LRHJOB) == AGC_STATUS_OK); /* LRHTASK */

    for (int i = 0; i < 5; i++)
    {
        sim_clock_tick();
    }

    CHECK(agc_erase_modecadr(AGC_IMU_DEV_RADAR) == (agc_sp_t) AGC_JOB_LRHJOB_DONE);
    CHECK(agc_exec_novac(AGC_PRIO32, AGC_JOB_LRVJOB) == AGC_STATUS_OK); /* VALTCHK */

    for (int i = 0; i < 200; i++)
    {
        sim_clock_tick();
        lrv_read = lrv_read || agc_erase_modecadr(AGC_IMU_DEV_RADAR) == (agc_sp_t) AGC_JOB_LRVJOB_DONE;

        for (int k = 0; k < 3; k++)
        {
            two_users = two_users || (agc_erase_failreg(k) & 07777u) == 01210u;
        }
    }

    CHECK(!two_users);
    CHECK(lrv_read);

    /* P64 entry: HIGATASK (in SERVICER) starts HIGATJOB, which repositions
     * the antenna, 5 cs after LRHTASK started an altitude read.        */
    bool repositioning = false; /* HIGATJOB reached RADSTALL */

    for (int i = 0; i < 200; i++)
    {
        sim_clock_tick(); /* the reads above end */
    }

    CHECK(agc_exec_novac(AGC_PRIO32, AGC_JOB_LRHJOB) == AGC_STATUS_OK); /* LRHTASK */

    for (int i = 0; i < 5; i++)
    {
        sim_clock_tick();
    }

    CHECK(agc_exec_findvac(AGC_PRIO32, AGC_JOB_HIGATJOB) == AGC_STATUS_OK); /* HIGATASK */

    for (int i = 0; i < 100; i++)
    {
        sim_clock_tick();
        repositioning = repositioning || agc_erase_modecadr(AGC_IMU_DEV_RADAR) == (agc_sp_t) AGC_JOB_HIGATJOB_DONE;

        for (int k = 0; k < 3; k++)
        {
            two_users = two_users || (agc_erase_failreg(k) & 07777u) == 01210u;
        }
    }

    CHECK(!two_users);
    CHECK(repositioning);
} /* test_lr_reads */
