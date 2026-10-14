/**********************************************************************
 * FILE NAME: socket.c
 *
 * PURPOSE:
 *   yaAGC's peripheral socket interface, server side: the listening
 *   ports, the clients, packet framing in both directions, and the
 *   harness-only lockstep channels.
 *
 * AGC SOURCE:
 *   No AGC routine: virtualagc yaAGC/SocketAPI.c (ChannelOutput,
 *   ChannelInput, ChannelRoutineGeneric) and yaAGC/agc_utilities.c
 *   (FormIoPacket, ParseIoPacket, EstablishSocket).
 *
 * EXTERNAL VARIABLES:
 *   none (sockets and client state are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  -----------------------------------------
 *   sim_input              an input-channel packet (channels.c)
 *   sim_counter_increment  a counter packet (counters.c)
 *   sim_channels_send_all  the connect update (channels.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   sim_socket_open returns AGC_STATUS_BAD_ARG for a port range outside
 *   1..65535 and AGC_STATUS_NOT_FOUND when no port could be opened. A
 *   client whose socket breaks is dropped (as yaAGC does); a failed
 *   close() is reported on stderr.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   POSIX sockets, non-blocking, polled from the harness loop (main.c).
 *
 * NOTES:
 *   Conventions as yaAGC: one listening TCP socket per client slot on
 *   ports base .. base+9 (DEFAULT_MAX_CLIENTS; a peripheral picks its
 *   port), one client each, listen backlog 3; a new client gets every
 *   non-zero output first (UpdateAgcPeripheralConnect) and all-ones
 *   masks; a client re-synchronises on the 00/01/10/11 byte signatures
 *   (yaAGC's 0xFF keep-alive bytes are dropped that way); a packet with
 *   the u-bit sets the client's mask for that channel; 0200 | addr is a
 *   counter increment; anything else is an input channel value merged
 *   through the mask (sim_input).
 *   DEVIATION (pacing): yaAGC takes at most one packet per client per
 *   machine cycle, so two key packets sent back to back can overwrite
 *   channel 15 before the first KEYRUPT1; here reading a client stops
 *   at the first packet that requests an interrupt until the next
 *   poll, so each key gets its KEYRUPT. yaAGC's periodic keep-alive
 *   (four 0xFF bytes every 16 ChannelRoutine calls) is not sent: a dead
 *   client is found by the next broadcast instead.
 *   Lockstep (harness only, no Virtual AGC peripheral uses 0150/0151):
 *   a packet on channel 0150 asks the harness to run N centiseconds;
 *   the client then waits for 0151 = N. While stepping, no further
 *   packet of that client is read.
 *
 * ALGORITHM:
 *   none (byte framing and dispatch)
 **********************************************************************/
#define _POSIX_C_SOURCE 200809L /* POSIX.1-2008: sockets, poll */

#include <stdio.h>

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stddef.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "internal.h"

#define SOCK_SLOTS    10    /* yaAGC DEFAULT_MAX_CLIENTS              */
#define SOCK_BACKLOG  3     /* yaAGC EstablishSocket backlog          */
#define SOCK_CHANNELS 0400  /* 8-bit channel numbers incl. counters   */
#define SOCK_READ_MAX 4096  /* bytes read from one client per poll    */
#define SOCK_PORT_MAX 65535 /* highest TCP port                      */
#define SOCK_COUNTER  0177u /* counter address bits of 0200 | addr    */
#define SOCK_SEND_TRY 20    /* send attempts for one packet           */
#define SOCK_SEND_MS  10    /* wait for buffer space between attempts, ms */

typedef struct sock_client
{
    int      fd;                    /* -1: slot free                    */
    int      size;                  /* bytes of the packet so far       */
    uint8_t  pkt[SIM_PACKET_BYTES]; /* the packet being assembled       */
    uint16_t mask[SOCK_CHANNELS];   /* u-bit masks (yaAGC ChannelMasks) */
} sock_client_t;

static int           g_listen[SOCK_SLOTS]; /* listening sockets, -1 closed     */
static sock_client_t g_client[SOCK_SLOTS]; /* one client per port              */
static bool          g_open;               /* sim_socket_open succeeded        */
static uint32_t      g_steps;              /* lockstep credit, centiseconds    */
static int           g_step_client;        /* slot that asked for it, or -1    */
static bool          g_step_lost;          /* that client disconnected         */

/**********************************************************************
 * AGC: none -- yaAGC FormIoPacket.
 *
 * ARGUMENTS:
 *   chan   uint16_t  I  channel, 0..0777
 *   value  uint16_t  I  15-bit word
 *   pkt    uint8_t[] O  the four bytes
 **********************************************************************/
void sim_packet_form(uint16_t chan, uint16_t value, uint8_t pkt[SIM_PACKET_BYTES])
{
    pkt[0] = (uint8_t) (chan >> 3);
    pkt[1] = (uint8_t) (0x40u | ((chan << 3) & 0x38u) | ((value >> 12) & 0x07u));
    pkt[2] = (uint8_t) (0x80u | ((value >> 6) & 0x3Fu));
    pkt[3] = (uint8_t) (0xC0u | (value & 0x3Fu));
}

/**********************************************************************
 * AGC: none -- yaAGC ParseIoPacket.
 *
 * ARGUMENTS:
 *   pkt    const uint8_t[]  I  the four bytes
 *   chan   uint16_t *       O  channel, 0..0377
 *   value  uint16_t *       O  15-bit word
 *   ubit   bool *           O  the value is a mask
 *
 * RETURNS: bool -- false when the byte signatures do not match
 **********************************************************************/
bool sim_packet_parse(const uint8_t pkt[SIM_PACKET_BYTES], uint16_t *chan, uint16_t *value, bool *ubit)
{
    bool ok = (pkt[0] & 0xC0u) == 0u && (pkt[1] & 0xC0u) == 0x40u && (pkt[2] & 0xC0u) == 0x80u && (pkt[3] & 0xC0u) == 0xC0u; /* signatures */

    if (ok)
    {
        *chan  = (uint16_t) (((pkt[0] & 0x1Fu) << 3) | ((pkt[1] >> 3) & 7u));
        *value = (uint16_t) (((pkt[1] << 12) & 0x7000u) | ((pkt[2] << 6) & 0x0FC0u) | (pkt[3] & 0x3Fu));
        *ubit  = (pkt[0] & 0x20u) != 0u;
    }

    return (ok);
}

/**********************************************************************
 * AGC: none -- close a descriptor; a failure is reported (the
 * descriptor is released either way).
 *
 * ARGUMENTS:
 *   fd  int  I  descriptor, >= 0
 **********************************************************************/
static void close_fd(int fd)
{
    if (close(fd) != 0)
    {
        (void) fprintf(stderr, "agc_sim: close(%d) failed: %s\n", fd, strerror(errno)); /* stdlib diagnostic */
    }
}

/**********************************************************************
 * AGC: none -- drop a client (yaAGC "Removing socket"); the lockstep
 * client's departure ends a lockstep run.
 *
 * ARGUMENTS:
 *   i  int  I  slot, 0..SOCK_SLOTS-1
 **********************************************************************/
static void client_close(int i)
{
    if (g_client[i].fd >= 0)
    {
        close_fd(g_client[i].fd);
        g_client[i].fd   = -1;
        g_client[i].size = 0;

        if (i == g_step_client)
        {
            g_step_lost   = true;
            g_step_client = -1;
        }
    }
}

/**********************************************************************
 * AGC: none -- yaAGC UnblockSocket.
 *
 * ARGUMENTS:
 *   fd  int  I  descriptor
 *
 * RETURNS: bool -- true when the descriptor is now non-blocking
 **********************************************************************/
static bool set_nonblocking(int fd)
{
    int fl = fcntl(fd, F_GETFL, 0); /* file status flags */

    return (fl >= 0 && fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0);
}

/**********************************************************************
 * AGC: none -- yaAGC EstablishSocket: a non-blocking listening socket
 * on all interfaces.
 *
 * ARGUMENTS:
 *   port  int  I  TCP port, 1..65535
 *
 * RETURNS: int -- the socket, or -1 when it could not be opened (the
 *          slot stays closed, as yaAGC's)
 **********************************************************************/
static int listen_on(int port)
{
    int                fd  = socket(AF_INET, SOCK_STREAM, 0); /* listening socket */
    int                one = 1;                               /* SO_REUSEADDR on */
    struct sockaddr_in a;                                     /* the bound address */

    (void) memset(&a, 0, sizeof (a)); /* stdlib: returns &a */
    a.sin_family      = AF_INET;
    a.sin_port        = htons((uint16_t) port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);

    if (fd >= 0 && (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof (one)) != 0 || bind(fd, (const struct sockaddr *) &a, sizeof (a)) != 0 || listen(fd, SOCK_BACKLOG) != 0 || !set_nonblocking(fd)))
    {
        close_fd(fd);
        fd = -1;
    }

    return (fd);
}

/**********************************************************************
 * AGC: none -- open the listening sockets base_port .. base_port + 9.
 *
 * ARGUMENTS:
 *   base_port  int  I  first port
 *
 * RETURNS: agc_status_t -- OK when at least one port is open; NOT_FOUND
 *          when none is; BAD_ARG for a port range outside 1..65535
 **********************************************************************/
agc_status_t sim_socket_open(int base_port)
{
    int opened = 0; /* ports listening */

    if (base_port <= 0 || base_port + SOCK_SLOTS - 1 > SOCK_PORT_MAX)
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < SOCK_SLOTS; i++)
    {
        g_client[i].fd   = -1;
        g_client[i].size = 0;
        g_listen[i]      = listen_on(base_port + i);
        opened           += (g_listen[i] >= 0) ? 1 : 0; /* count the open ones */
    }

    g_open        = (opened > 0);
    g_steps       = 0;
    g_step_client = -1;
    g_step_lost   = false;

    return (g_open ? AGC_STATUS_OK : AGC_STATUS_NOT_FOUND); /* NOT_FOUND: no port */
}

/**********************************************************************
 * AGC: none -- close every client and listening socket.
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_socket_close(void)
{
    for (int i = 0; i < SOCK_SLOTS && g_open; i++)
    {
        client_close(i);

        if (g_listen[i] >= 0)
        {
            close_fd(g_listen[i]);
        }

        g_listen[i] = -1;
    }

    g_open = false;
}

/**********************************************************************
 * AGC: none -- send one packet to one client. A partial send or a full
 * buffer (EAGAIN) is retried: the rest of the packet is sent after up to
 * SOCK_SEND_MS for buffer space, at most SOCK_SEND_TRY times. A broken
 * pipe drops the client (yaAGC SOCKET_BROKEN); a client that still has
 * not drained loses the packet (none of it sent) or is dropped (part of
 * it sent, since half a packet would shift the stream); both reported.
 *
 * ARGUMENTS:
 *   i    int              I  slot
 *   pkt  const uint8_t[]  I  the packet
 **********************************************************************/
static void send_to(int i, const uint8_t pkt[SIM_PACKET_BYTES])
{
    size_t done = 0; /* bytes of the packet sent */

    for (int t = 0; t < SOCK_SEND_TRY && g_client[i].fd >= 0 && done < SIM_PACKET_BYTES; t++)
    {
        ssize_t n = send(g_client[i].fd, &pkt[done], SIM_PACKET_BYTES - done, MSG_NOSIGNAL); /* bytes sent, -1 error */

        if (n > 0)
        {
            done += (size_t) n; /* a partial send: the rest follows */
        }
        else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        {
            struct pollfd w = {g_client[i].fd, POLLOUT, 0}; /* wait for buffer space */

            if (poll(&w, 1, SOCK_SEND_MS) < 0 && errno != EINTR)
            {
                client_close(i);
            }
        }
        else
        {
            client_close(i); /* the peer has gone */
        }
    }

    if (g_client[i].fd >= 0 && done != 0u && done < SIM_PACKET_BYTES)
    {
        (void) fprintf(stderr, "agc_sim: client %d not draining: closed after a torn packet\n", i); /* stdlib diagnostic */
        client_close(i);                                                                            /* half a packet would shift every later one */
    }
    else if (g_client[i].fd >= 0 && done == 0u)
    {
        (void) fprintf(stderr, "agc_sim: client %d not draining: packet dropped\n", i); /* stdlib diagnostic */
    }
    else
    {
        /* sent, or no client */
    }
}

/**********************************************************************
 * AGC: none -- yaAGC ChannelOutput: broadcast one channel value.
 *
 * ARGUMENTS:
 *   chan   uint16_t  I  channel, 0..0777
 *   value  uint16_t  I  15-bit word
 **********************************************************************/
void sim_socket_send(uint16_t chan, uint16_t value)
{
    uint8_t pkt[SIM_PACKET_BYTES]; /* the packet */

    sim_packet_form(chan, (uint16_t) (value & SIM_WORD_MASK), pkt);

    for (int i = 0; i < SOCK_SLOTS && g_open; i++)
    {
        send_to(i, pkt);
    }
}

/**********************************************************************
 * AGC: none -- yaAGC ChannelRoutineGeneric: accept new clients, give
 * each all-ones masks and the connect update.
 *
 * ARGUMENTS: none
 **********************************************************************/
static void accept_clients(void)
{
    for (int i = 0; i < SOCK_SLOTS; i++)
    {
        int fd = (g_listen[i] >= 0 && g_client[i].fd < 0) ? accept(g_listen[i], NULL, NULL) : -1; /* new client */

        if (fd >= 0 && !set_nonblocking(fd))
        {
            close_fd(fd);
        }
        else if (fd >= 0)
        {
            g_client[i].fd   = fd;
            g_client[i].size = 0;

            for (int c = 0; c < SOCK_CHANNELS; c++)
            {
                g_client[i].mask[c] = SIM_WORD_MASK;
            }

            sim_channels_send_all();
        }
        else
        {
            /* slot busy, closed, or nobody waiting */
        }
    }
}

/**********************************************************************
 * AGC: none -- act on one complete packet of client i (yaAGC
 * ChannelInput). A counter the harness does not map is ignored, as
 * documented in counters.c; an unknown increment type is reported.
 *
 * ARGUMENTS:
 *   i  int  I  slot
 *
 * RETURNS: bool -- true: stop reading this client until the next poll
 *          (a lockstep request, or an input that requested an interrupt)
 **********************************************************************/
static bool act_on_packet(int i)
{
    uint16_t chan  = 0;     /* parsed channel */
    uint16_t value = 0;     /* parsed value */
    bool     ubit  = false; /* parsed u-bit */
    bool     stop  = false; /* stop reading this client */

    if (!sim_packet_parse(g_client[i].pkt, &chan, &value, &ubit))
    {
        /* framing guarantees the signatures: nothing to do */
    }
    else if (ubit)
    {
        g_client[i].mask[chan] = value;
    }
    else if (chan == SIM_CH_STEP)
    {
        g_steps       += value;
        g_step_client = i;
        stop          = true;
    }
    else if ((chan & SIM_CH_COUNTER) != 0u)
    {
        if (sim_counter_increment((uint16_t) (chan & SOCK_COUNTER), value) == AGC_STATUS_BAD_ARG)
        {
            (void) fprintf(stderr, "agc_sim: counter %03o: unknown increment type %o\n", (unsigned int) (chan & SOCK_COUNTER), (unsigned int) value); /* stdlib diagnostic */
        }
    }
    else
    {
        stop = sim_input(chan, value, g_client[i].mask[chan]);
    }

    return (stop);
}

/**********************************************************************
 * AGC: none -- yaAGC packet framing: byte j of a packet carries the
 * signature j << 6; a byte out of place restarts the packet (and is
 * dropped unless it can start one).
 *
 * ARGUMENTS:
 *   i  int      I  slot
 *   c  uint8_t  I  the byte received
 *
 * RETURNS: bool -- true: stop reading this client (act_on_packet)
 **********************************************************************/
static bool take_byte(int i, uint8_t c)
{
    sock_client_t *cl   = &g_client[i]; /* the client */
    bool           stop = false;        /* from act_on_packet */

    if ((c & 0xC0u) != (uint8_t) (cl->size << 6))
    {
        cl->size = 0;
    }

    if ((c & 0xC0u) == (uint8_t) (cl->size << 6))
    {
        cl->pkt[cl->size] = c;
        cl->size++;
    }

    if (cl->size == SIM_PACKET_BYTES)
    {
        cl->size = 0;
        stop     = act_on_packet(i);
    }

    return (stop);
}

/**********************************************************************
 * AGC: none -- read what client i has sent, up to SOCK_READ_MAX bytes,
 * stopping after a packet that asks for it; a closed peer or a socket
 * error drops the client.
 *
 * ARGUMENTS:
 *   i  int  I  slot with a client
 **********************************************************************/
static void read_client(int i)
{
    bool more = true; /* keep reading */

    for (int n = 0; n < SOCK_READ_MAX && more; n++)
    {
        uint8_t c = 0;                              /* the byte */
        ssize_t k = recv(g_client[i].fd, &c, 1, 0); /* bytes read, 0 closed, -1 error */

        if (k == 0 || (k < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
        {
            client_close(i);
            more = false;
        }
        else
        {
            more = k > 0 && !take_byte(i, c);
        }
    }
}

/**********************************************************************
 * AGC: none -- accept clients and read every client's input (not the
 * lockstep client while it has credit).
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_socket_poll(void)
{
    if (g_open)
    {
        accept_clients();

        for (int i = 0; i < SOCK_SLOTS; i++)
        {
            if (g_client[i].fd >= 0 && !(g_steps > 0u && i == g_step_client))
            {
                read_client(i);
            }
        }
    }
}

/**********************************************************************
 * AGC: none -- sleep until a client socket or a free slot's listening
 * socket is readable.
 *
 * ARGUMENTS:
 *   timeout_ms  int  I  longest wait, ms
 *
 * RETURNS: bool -- true when something is readable
 **********************************************************************/
bool sim_socket_wait(int timeout_ms)
{
    struct pollfd p[SOCK_SLOTS]; /* one descriptor per slot */
    nfds_t        n = 0;         /* descriptors used */

    for (int i = 0; i < SOCK_SLOTS && g_open; i++)
    {
        int fd = (g_client[i].fd >= 0) ? g_client[i].fd : g_listen[i]; /* client, else listener */

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

/**********************************************************************
 * AGC: none -- take the lockstep credit.
 *
 * ARGUMENTS: none
 *
 * RETURNS: uint32_t -- centiseconds asked for since the last call
 **********************************************************************/
uint32_t sim_socket_take_steps(void)
{
    uint32_t s = g_steps; /* credit, cs */

    g_steps = 0;

    return (s);
}

/**********************************************************************
 * AGC: none -- tell the stepping client that `done` centiseconds have
 * been run (channel 0151).
 *
 * ARGUMENTS:
 *   done  uint32_t  I  centiseconds run
 **********************************************************************/
void sim_socket_ack(uint32_t done)
{
    uint8_t pkt[SIM_PACKET_BYTES]; /* the packet */

    if (g_step_client >= 0)
    {
        sim_packet_form(SIM_CH_STEP_ACK, (uint16_t) (done & SIM_WORD_MASK), pkt);
        send_to(g_step_client, pkt);
    }
}

/**********************************************************************
 * AGC: none -- the lockstep client has disconnected.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true once it has gone
 **********************************************************************/
bool sim_socket_lost_lockstep(void)
{
    return (g_step_lost);
}

/**********************************************************************
 * AGC: none -- host hook (internal.h): under agc_sim the clock requests
 * DOWNRUPT itself every 20 ms (clock.c DEVIATION); the NASSP host
 * (src/nassp) answers true because its telemetry raises DOWNRUPT.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- false: the clock times DOWNRUPT
 **********************************************************************/
bool sim_host_downrupt_external(void)
{
    return (false);
}

/**********************************************************************
 * AGC: none -- host hook (internal.h): a radar cycle has completed.
 * Under agc_sim a peripheral has already stored RNRAD with a counter
 * packet (0200 | 046), so there is nothing to fetch.
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_host_radar_read(void)
{
    /* RNRAD arrives by packet before the cycle completes */
}
