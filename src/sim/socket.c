/*
 * socket.c -- yaAGC's peripheral socket interface, server side.
 *
 * Same conventions as yaAGC (SocketAPI.c, agc_utilities.c):
 *   - one listening TCP socket per client slot on ports base .. base+9
 *     (base 19697 for yaAGC; a peripheral picks its port), one client each;
 *   - a new client first receives every non-zero output (connect update);
 *   - packets are FormIoPacket / ParseIoPacket (internal.h); a client
 *     re-synchronises on the 00/01/10/11 byte signatures;
 *   - an inbound packet with the u-bit sets the client's mask for that
 *     channel; 0200 | addr is a counter increment; anything else is an
 *     input channel value merged through the mask (sim_input);
 *   - at most one interrupt-raising input per client per centisecond, so
 *     key presses sent back to back each get their KEYRUPT.
 * Lockstep (sim only): a packet on channel 0150 asks the harness to run
 * N centiseconds; the client then waits for 0151 = N. While stepping, no
 * further packet of that client is read.
 */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stddef.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "agc/core/assert.h"
#include "internal.h"

#define SOCK_SLOTS    10   /* yaAGC DEFAULT_MAX_CLIENTS              */
#define SOCK_CHANNELS 0400 /* 8-bit channel numbers incl. counters   */
#define SOCK_READ_MAX 4096 /* bytes read from one client per poll    */

typedef struct sock_client
{
    int      fd;                     /* -1: slot free                 */
    int      size;                   /* bytes of the packet so far    */
    uint8_t  pkt[SIM_PACKET_BYTES];
    uint16_t mask[SOCK_CHANNELS];    /* u-bit masks (yaAGC ChannelMasks) */
} sock_client_t;

static int           g_listen[SOCK_SLOTS];
static sock_client_t g_client[SOCK_SLOTS];
static bool          g_open;
static uint32_t      g_steps;         /* lockstep credit, centiseconds   */
static int           g_step_client;   /* slot that asked for it, or -1   */
static bool          g_step_lost;     /* that client disconnected        */

void sim_packet_form(uint16_t chan, uint16_t value,
                     uint8_t pkt[SIM_PACKET_BYTES])
{
    if (!AGC_ASSERT(chan <= 0777u) || !AGC_ASSERT(value <= SIM_WORD_MASK))
    {
        return;
    }

    pkt[0] = (uint8_t) (chan >> 3);
    pkt[1] =
        (uint8_t) (0x40u | ((chan << 3) & 0x38u) | ((value >> 12) & 0x07u));
    pkt[2] = (uint8_t) (0x80u | ((value >> 6) & 0x3Fu));
    pkt[3] = (uint8_t) (0xC0u | (value & 0x3Fu));
}

bool sim_packet_parse(const uint8_t pkt[SIM_PACKET_BYTES], uint16_t *chan,
                      uint16_t *value, bool *ubit)
{
    if (!AGC_ASSERT(chan != NULL && value != NULL) || !AGC_ASSERT(ubit != NULL))
    {
        return (false);
    }

    if ((pkt[0] & 0xC0u) != 0u || (pkt[1] & 0xC0u) != 0x40u ||
        (pkt[2] & 0xC0u) != 0x80u || (pkt[3] & 0xC0u) != 0xC0u)
    {
        return (false);
    }

    *chan  = (uint16_t) (((pkt[0] & 0x1Fu) << 3) | ((pkt[1] >> 3) & 7u));
    *value =
        (uint16_t) (((pkt[1] << 12) & 0x7000u) | ((pkt[2] << 6) & 0x0FC0u) |
                    (pkt[3] & 0x3Fu));
    *ubit = (pkt[0] & 0x20u) != 0u;

    return (true);
}

static void client_close(int i)
{
    if (!AGC_ASSERT(i >= 0 && i < SOCK_SLOTS) || g_client[i].fd < 0)
    {
        return;
    }

    if (close(g_client[i].fd) != 0)
    {
        g_client[i].fd = -1; /* already gone; nothing else to do */
    }

    g_client[i].fd   = -1;
    g_client[i].size = 0;

    if (i == g_step_client)
    {
        g_step_lost   = true;
        g_step_client = -1;
    }
}

static bool set_nonblocking(int fd)
{
    int fl = fcntl(fd, F_GETFL, 0);

    if (!AGC_ASSERT(fd >= 0) || fl < 0)
    {
        return (false);
    }

    return (fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0);
}

static int listen_on(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);

    if (fd < 0 || !AGC_ASSERT(port > 0 && port < 65536))
    {
        return (-1);
    }

    int                one = 1;
    struct sockaddr_in a;

    (void) memset(&a, 0, sizeof (a)); /* memset's result is its argument */
    a.sin_family      = AF_INET;
    a.sin_port        = htons((uint16_t) port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);

    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof (one)) != 0 ||
        bind(fd, (const struct sockaddr *) &a, sizeof (a)) != 0 ||
        listen(fd, 3) != 0 || !set_nonblocking(fd))
    {
        if (close(fd) != 0)
        {
            return (-1);
        }

        return (-1);
    }

    return (fd);
}

agc_status_t sim_socket_open(int base_port)
{
    if (!AGC_ASSERT(base_port > 0 && base_port + SOCK_SLOTS < 65536))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int opened = 0;

    for (int i = 0; i < SOCK_SLOTS; i++)
    {
        g_client[i].fd   = -1;
        g_client[i].size = 0;
        g_listen[i]      = listen_on(base_port + i);
        opened          += (g_listen[i] >= 0) ? 1 : 0;
    }

    g_open        = (opened > 0);
    g_steps       = 0;
    g_step_client = -1;
    g_step_lost   = false;

    if (!AGC_ASSERT(opened <= SOCK_SLOTS))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (g_open ? AGC_STATUS_OK : AGC_STATUS_NOT_FOUND);
}

void sim_socket_close(void)
{
    for (int i = 0; i < SOCK_SLOTS && g_open; i++)
    {
        client_close(i);

        if (g_listen[i] >= 0 && close(g_listen[i]) != 0)
        {
            g_listen[i] = -1;
        }

        g_listen[i] = -1;
    }

    g_open = false;

    if (!AGC_ASSERT(!g_open))
    {
        return;
    }
}

static void send_to(int i, const uint8_t pkt[SIM_PACKET_BYTES])
{
    if (!AGC_ASSERT(i >= 0 && i < SOCK_SLOTS) || g_client[i].fd < 0)
    {
        return;
    }

    ssize_t n = send(g_client[i].fd, pkt, SIM_PACKET_BYTES, MSG_NOSIGNAL);

    if (n != (ssize_t) SIM_PACKET_BYTES && !(n < 0 && errno == EAGAIN))
    {
        client_close(i); /* broken pipe or short write: drop the client */
    }
}

void sim_socket_send(uint16_t chan, uint16_t value)
{
    uint8_t pkt[SIM_PACKET_BYTES];

    if (!g_open || !AGC_ASSERT(value <= SIM_WORD_MASK))
    {
        return;
    }

    sim_packet_form(chan, value, pkt);

    for (int i = 0; i < SOCK_SLOTS; i++)
    {
        send_to(i, pkt);
    }
}

static void accept_clients(void)
{
    for (int i = 0; i < SOCK_SLOTS; i++)
    {
        if (g_listen[i] < 0 || g_client[i].fd >= 0)
        {
            continue;
        }

        int fd = accept(g_listen[i], NULL, NULL);

        if (fd < 0 || !set_nonblocking(fd))
        {
            continue;
        }

        g_client[i].fd   = fd;
        g_client[i].size = 0;

        for (int c = 0; c < SOCK_CHANNELS; c++)
        {
            g_client[i].mask[c] = SIM_WORD_MASK;
        }

        sim_channels_send_all(); /* yaAGC UpdateAgcPeripheralConnect */
    }
}

/* Act on one complete packet; true: stop reading this client for now.  */
static bool act_on_packet(int i)
{
    uint16_t chan  = 0;
    uint16_t value = 0;
    bool     ubit  = false;

    if (!sim_packet_parse(g_client[i].pkt, &chan, &value, &ubit))
    {
        return (false); /* e.g. the 0xFF keep-alive of other peripherals */
    }

    if (!AGC_ASSERT(chan < SOCK_CHANNELS))
    {
        return (false);
    }

    if (ubit)
    {
        g_client[i].mask[chan] = value;

        return (false);
    }

    if (chan == SIM_CH_STEP)
    {
        g_steps      += value;
        g_step_client = i;

        return (true);
    }

    if ((chan & SIM_CH_COUNTER) != 0u)
    {
        return (sim_counter_increment((uint16_t) (chan & 0177u), value) ==
                AGC_STATUS_BAD_ARG);
    }

    return (sim_input(chan, value, g_client[i].mask[chan]));
}

/* yaAGC packet framing: byte j of a packet carries signature j << 6.    */
static bool take_byte(int i, uint8_t c)
{
    sock_client_t *cl = &g_client[i];

    if ((c & 0xC0u) != (uint8_t) (cl->size << 6))
    {
        cl->size = 0;

        if ((c & 0xC0u) != 0u)
        {
            return (false); /* not a packet start: drop it */
        }
    }

    cl->pkt[cl->size] = c;
    cl->size++;

    if (cl->size < SIM_PACKET_BYTES)
    {
        return (false);
    }

    cl->size = 0;

    return (act_on_packet(i));
}

static void read_client(int i)
{
    for (int n = 0; n < SOCK_READ_MAX; n++)
    {
        uint8_t c = 0;
        ssize_t k = recv(g_client[i].fd, &c, 1, 0);

        if (k == 0 || (k < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
        {
            client_close(i); /* peer closed or error */

            return;
        }

        if (k < 0 || take_byte(i, c))
        {
            return; /* nothing more now, or an interrupt / step taken */
        }
    }
}

void sim_socket_poll(void)
{
    if (!g_open)
    {
        return;
    }

    accept_clients();

    for (int i = 0; i < SOCK_SLOTS; i++)
    {
        if (g_client[i].fd >= 0 && !(g_steps > 0u && i == g_step_client))
        {
            read_client(i);
        }
    }
}

int sim_socket_clients(void)
{
    int n = 0;

    for (int i = 0; i < SOCK_SLOTS; i++)
    {
        n += (g_client[i].fd >= 0) ? 1 : 0;
    }

    if (!AGC_ASSERT(n >= 0 && n <= SOCK_SLOTS))
    {
        return (0);
    }

    return (n);
}

/* Sleep until a client socket or a listening socket is readable.        */
bool sim_socket_wait(int timeout_ms)
{
    struct pollfd p[2 * SOCK_SLOTS];
    nfds_t        n = 0;

    if (!AGC_ASSERT(timeout_ms >= 0))
    {
        return (false);
    }

    for (int i = 0; i < SOCK_SLOTS && g_open; i++)
    {
        int fd = (g_client[i].fd >= 0) ? g_client[i].fd : g_listen[i];

        if (fd >= 0)
        {
            p[n].fd      = fd;
            p[n].events  = POLLIN;
            p[n].revents = 0;
            n++;
        }
    }

    return (poll(p, n, timeout_ms) > 0);
}

uint32_t sim_socket_take_steps(void)
{
    uint32_t s = g_steps;

    g_steps = 0;

    if (!AGC_ASSERT(g_steps == 0u))
    {
        return (0);
    }

    return (s);
}

/* Tell the stepping client that `done` centiseconds have been run.      */
void sim_socket_ack(uint32_t done)
{
    uint8_t pkt[SIM_PACKET_BYTES];

    if (g_step_client < 0 || !AGC_ASSERT(g_step_client < SOCK_SLOTS))
    {
        return;
    }

    sim_packet_form(SIM_CH_STEP_ACK, (uint16_t) (done & SIM_WORD_MASK), pkt);
    send_to(g_step_client, pkt);
}

bool sim_socket_lost_lockstep(void)
{
    if (!AGC_ASSERT(g_step_client < SOCK_SLOTS))
    {
        return (true);
    }

    return (g_step_lost);
}
