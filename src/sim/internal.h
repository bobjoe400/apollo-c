/**********************************************************************
 * FILE NAME: internal.h
 *
 * PURPOSE:
 *   Private declarations of the host harness (src/sim/): build/agc_sim
 *   runs the translated flight software with its own clock and interrupt
 *   driver (clock.c) and talks to Virtual AGC peripherals with yaAGC's
 *   socket protocol (socket.c, channels.c, counters.c): yaDSKY2,
 *   yaTelemetry, yaACA, ... connect to it as they would to yaAGC. Also
 *   used by tests/sim/harness.c.
 *
 * AGC SOURCE:
 *   none (infrastructure). Protocol reference: virtualagc
 *   yaAGC/agc_utilities.c (FormIoPacket, ParseIoPacket) and
 *   yaAGC/SocketAPI.c (ChannelOutput, ChannelInput). A packet is 4
 *   bytes 00uccccc 01cccvvv 10vvvvvv 11vvvvvv: c = channel (8 bits as
 *   parsed, 0200 set = counter increment), v = 15-bit value, u = the
 *   value is the client's mask for that channel.
 *
 * REQUIRES:
 *   none (self-contained; includes what it uses)
 *
 * GLOBAL VARIABLES:
 *   none (each file keeps its state static)
 **********************************************************************/
#ifndef AGC_SIM_INTERNAL_H
#define AGC_SIM_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/state.h"
#include "agc/core/status.h"

#define SIM_PACKET_BYTES 4       /* bytes per packet          */
#define SIM_WORD_MASK    077777u /* one AGC word, 15 bits     */

/* Fictitious channels (yaAGC conventions).                                */
#define SIM_CH_DSKY_LAMPS 0163u /* KEY REL, OPR ERR, V/N flash, etc. */
#define SIM_CH_RHC_PITCH  0166u /* in: RHC angles, latched into the  */
#define SIM_CH_RHC_YAW    0167u /*     RHC counters on ch13 b8+b9    */
#define SIM_CH_RHC_ROLL   0170u /*     (pitch, yaw, roll)     */
#define SIM_CH_OPT_T_DRV  0171u /* out: optics trunnion drive pulses */
#define SIM_CH_OPT_S_DRV  0172u /* out: optics shaft drive pulses    */
#define SIM_CH_UPLINK     0173u /* in: INLINK word + UPRUPT          */
#define SIM_CH_CDUX_DRV   0174u /* out: IMU CDU X coarse drive       */
#define SIM_CH_GYRO       0177u /* out: gyro torque pulses           */
#define SIM_CH_COUNTER    0200u /* in: 0200 | counter address        */

/* Sim-only lockstep channels (not used by any Virtual AGC peripheral).    */
#define SIM_CH_STEP     0150u /* in: run N centiseconds            */
#define SIM_CH_STEP_ACK 0151u /* out: N centiseconds done          */

/* Interrupt requests, in AGC priority order (INTERRUPT_LEAD_INS).         */
typedef enum sim_irq
{
    SIM_IRQ_T6 = 0,
    SIM_IRQ_T5,
    SIM_IRQ_T3,
    SIM_IRQ_T4,
    SIM_IRQ_KEY1,
    SIM_IRQ_KEY2,
    SIM_IRQ_UPLINK,
    SIM_IRQ_DOWNLINK,
    SIM_IRQ_RADAR,
    SIM_IRQ_HAND,
    SIM_IRQ_COUNT
} sim_irq_t;

/* ---- packets (socket.c) ----------------------------------------------- */
void sim_packet_form(uint16_t chan, uint16_t value, uint8_t pkt[SIM_PACKET_BYTES]);
bool sim_packet_parse(const uint8_t pkt[SIM_PACKET_BYTES], uint16_t *chan, uint16_t *value, bool *ubit);

/* ---- socket server (socket.c) ----------------------------------------- */
agc_status_t sim_socket_open(int base_port);
void         sim_socket_close(void);
void         sim_socket_poll(void);                          /* accept, read inbound packets  */
void         sim_socket_send(uint16_t chan, uint16_t value); /* broadcast */
int          sim_socket_clients(void);
bool         sim_socket_wait(int timeout_ms);                /* input or timeout     */
uint32_t     sim_socket_take_steps(void);                    /* lockstep credit       */
void         sim_socket_ack(uint32_t done);                  /* lockstep: N cs run    */
bool         sim_socket_lost_lockstep(void);                 /* stepping client gone  */

/* ---- host hooks: socket.c for agc_sim, src/nassp for NASSP -------------- */
bool sim_host_downrupt_external(void); /* DOWNRUPT requested by the host, not the clock */
void sim_host_radar_read(void);        /* radar cycle complete: host stores RNRAD       */

/* ---- channels and counters (channels.c, counters.c) ------------------- */
agc_status_t sim_channels_init(void);
void         sim_channels_sync(void);           /* broadcast changes     */
void         sim_channels_force(uint16_t chan); /* send even if same     */
void         sim_channels_send_all(void);       /* connect update        */
agc_status_t sim_channels_tick(uint32_t cs);    /* 0163, drives, RHC     */
bool         sim_input(uint16_t chan, uint16_t value, uint16_t mask);
agc_status_t sim_counter_increment(uint16_t addr, uint16_t type);

/* ---- clock and interrupts (clock.c) ----------------------------------- */
typedef struct sim_stats
{
    uint32_t cs;                   /* centiseconds run              */
    uint32_t jobs;                 /* agc_exec_run_one that ran one */
    uint32_t restarts;             /* software restarts serviced    */
    uint32_t busy_ticks;           /* exec bound hit in a tick      */
    uint32_t rupts[SIM_IRQ_COUNT]; /* interrupts taken              */
    uint32_t errors;               /* BAD_ARG from a handler / job  */
} sim_stats_t;

void     sim_clock_init(void);
void     sim_clock_tick(void);  /* one centisecond = 32 counts */
void     sim_clock_count(void); /* one scaler count, 1/3200 s  */
bool     sim_irq_pending(sim_irq_t irq);
void     sim_irq_request(sim_irq_t irq);
uint32_t sim_clock_cs(void);
bool     sim_clock_standby(void);   /* SBY: only the scaler runs */
void     sim_clock_stats(sim_stats_t *out);
void     sim_clock_arm_traps(void); /* channel 13 bits 12-14 */

/* ---- erasable pad load at power-on (pad.c) ---------------------------- */
agc_status_t sim_pad_load(const char *path);
bool         sim_pad_is_pad(const char *path);

/* ---- rope image as fixed memory (rope.c) ------------------------------ */
agc_status_t sim_rope_vehicle(const char *path, agc_vehicle_t *vehicle);
agc_status_t sim_rope_load(const char *path);

/* ---- timed input script (script.c) ------------------------------------ */
agc_status_t sim_script_open(const char *path);
void         sim_script_tick(uint32_t cs); /* apply the events due     */
void         sim_script_close(void);

#endif /* AGC_SIM_INTERNAL_H */
