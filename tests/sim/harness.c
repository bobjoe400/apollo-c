/**********************************************************************
 * FILE NAME: harness.c
 *
 * PURPOSE:
 *   Unit tests of the sim harness (src/sim/ without main.c) against
 *   yaAGC's behaviour: packet format, unprogrammed counter increments,
 *   the scaler-phase timing of TIME6 / T6RUPT, the radar cycle and
 *   RADARUPT, the HANDRUPT traps, the channel 33 latches, and the CDU
 *   drive pacing.
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
#include <stdio.h>

#include <stdbool.h>
#include <stdint.h>

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
    power_on(AGC_VEHICLE_LM);
    test_radar();
    test_handrupt();
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
    CHECK(agc_power_on(v) == AGC_STATUS_OK);
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
 * -0 -> +1 step, PCDU/MCDU wrap, SHINC/SHANC in 14 bits, DINC toward
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
    agc_erase_set_cdux(077777u);
    CHECK(sim_counter_increment(032, 1) == AGC_STATUS_OK && agc_erase_cdux() == 0u);
    CHECK(sim_counter_increment(032, 023) == AGC_STATUS_OK && agc_erase_cdux() == 077777u);
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
