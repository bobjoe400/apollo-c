/*
 * main.c -- build/agc_sim: the translated flight software on the host.
 *
 * Usage: agc_sim [--vehicle csm|lm] [--port N] [--max-cs N] [--script F]
 *                [--pad F] [--headless | --lockstep | --fast] [csm|lm] [cs]
 *
 *   (default)   peripheral server: listens on ports N..N+9 (default 19697
 *               for the CSM, 19797 for the LM: the Virtual AGC port
 *               conventions) and runs in real time, 1 cs of AGC time per
 *               10 ms; yaDSKY2, yaTelemetry, ... connect as to yaAGC.
 *   --fast      server without real-time pacing.
 *   --lockstep  server whose clock advances only when a client asks
 *               (channel 0150 = N cs, answered by 0151); used by
 *               tools/sim_client.py so scenarios are deterministic.
 *   --headless  no sockets: run --max-cs centiseconds (default 1000) and
 *               print a summary (the integration-wave driver).
 *   --script F  timed input packets (script.c), in any mode.
 *   --pad F     erasables set before GOJAM (pad.c): REFSMMAT, flagwords.
 * The bare words csm / lm and a bare number are --vehicle and --max-cs.
 *
 * Power-on: agc_power_on (host: vehicle, state, lane erasables, exec),
 * then DOFSTART, the machine fresh start (FRESH_START_AND_RESTART C
 * p.184 / L p.211).
 */
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define SIM_PORT_CSM   19697   /* Virtual AGC: CM peripherals 19697-19706 */
#define SIM_PORT_LM    19797   /* LM peripherals 19797-19806              */
#define SIM_DEFAULT_CS 1000ul  /* headless default run              */
#define SIM_MAX_RUN_CS 0x0FFFFFFFul
#define SIM_TICK_NS    10000000L   /* 10 ms                           */
#define SIM_WAIT_MS    20
#define SIM_LAG_TICKS  100   /* resynchronise after 1 s behind    */

typedef enum sim_mode
{
    SIM_MODE_REALTIME = 0,
    SIM_MODE_FAST,
    SIM_MODE_LOCKSTEP,
    SIM_MODE_HEADLESS
} sim_mode_t;

typedef struct sim_opts
{
    agc_vehicle_t vehicle;
    sim_mode_t    mode;
    int           port;
    unsigned long max_cs; /* 0: unbounded (server modes)          */
    const char   *script;
    const char   *pad;    /* --pad: erasable pad load            */
} sim_opts_t;

/* The value of option `name` given as `--name=V` or `--name V` (then
 * *used = 1); NULL when `arg` is not that option.                       */
static const char *opt_value(const char *arg, const char *name,
                             const char *next, int *used)
{
    size_t n = strlen(name);

    if (!AGC_ASSERT(arg != NULL && used != NULL) || strncmp(arg, name, n) != 0)
    {
        return (NULL);
    }

    if (arg[n] == '=')
    {
        return (&arg[n + 1]);
    }

    if (arg[n] == '\0' && next != NULL)
    {
        *used = 1;

        return (next);
    }

    return (NULL);
}

/* One argument (and maybe the next); false when it is not understood.  */
static bool parse_one(sim_opts_t *o, const char *arg, const char *next,
                      int *used)
{
    const char *v = NULL;

    if (!AGC_ASSERT(o != NULL && arg != NULL))
    {
        return (false);
    }

    if (strcmp(arg, "csm") == 0 || strcmp(arg, "lm") == 0)
    {
        o->vehicle = (arg[0] == 'l') ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM;
    }
    else if ((v = opt_value(arg, "--vehicle", next, used)) != NULL)
    {
        o->vehicle = (strcmp(v, "lm") == 0) ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM;
    }
    else if ((v = opt_value(arg, "--port", next, used)) != NULL)
    {
        o->port = (int) strtol(v, NULL, 10);
    }
    else if ((v = opt_value(arg, "--max-cs", next, used)) != NULL)
    {
        o->max_cs = strtoul(v, NULL, 10);
    }
    else if ((v = opt_value(arg, "--script", next, used)) != NULL)
    {
        o->script = v;
    }
    else if ((v = opt_value(arg, "--pad", next, used)) != NULL)
    {
        o->pad = v;
    }
    else if (arg[0] >= '0' && arg[0] <= '9')
    {
        o->max_cs = strtoul(arg, NULL, 10);
    }
    else
    {
        o->mode = (strcmp(arg, "--headless") == 0) ? SIM_MODE_HEADLESS :
                  (strcmp(arg, "--lockstep") == 0) ? SIM_MODE_LOCKSTEP :
                  (strcmp(arg, "--fast") == 0) ? SIM_MODE_FAST : o->mode;

        return (strcmp(arg,
                       "--headless") == 0 || strcmp(arg, "--lockstep") == 0 ||
                strcmp(arg, "--fast") == 0);
    }

    return (true);
}

static bool parse_args(int argc, char *argv[], sim_opts_t *o)
{
    if (!AGC_ASSERT(o != NULL) || !AGC_ASSERT(argc >= 1))
    {
        return (false);
    }

    o->vehicle = AGC_VEHICLE_CSM;
    o->mode    = SIM_MODE_REALTIME;
    o->port    = 0; /* by vehicle unless --port */
    o->max_cs  = 0;
    o->script  = NULL;
    o->pad     = NULL;

    for (int i = 1; i < argc; i++)
    {
        int used = 0;

        if (!parse_one(o, argv[i], (i + 1 < argc) ? argv[i + 1] : NULL, &used))
        {
            (void) fprintf(stderr, "agc_sim: unknown argument %s\n", argv[i]);

            return (false); /* fprintf result irrelevant: we stop here */
        }

        i += used;
    }

    if (o->port == 0)
    {
        o->port = (o->vehicle == AGC_VEHICLE_LM) ? SIM_PORT_LM : SIM_PORT_CSM;
    }

    if (o->mode == SIM_MODE_HEADLESS && o->max_cs == 0u)
    {
        o->max_cs = SIM_DEFAULT_CS;
    }

    return (o->port > 0 && o->port < 65526 && o->max_cs <= SIM_MAX_RUN_CS);
}

/* Real-time pacing: sleep until the next 10 ms boundary.              */
static void pace(struct timespec *next)
{
    struct timespec now;

    if (!AGC_ASSERT(next != NULL) || clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    {
        return;
    }

    next->tv_nsec += SIM_TICK_NS;

    if (next->tv_nsec >= 1000000000L)
    {
        next->tv_nsec -= 1000000000L;
        next->tv_sec++;
    }

    long ahead = (long) (next->tv_sec - now.tv_sec) * 1000000000L +
                 (next->tv_nsec - now.tv_nsec);

    if (ahead > 0)
    {
        struct timespec d = {ahead / 1000000000L, ahead % 1000000000L};

        (void) nanosleep(&d, NULL); /* an early wake-up only shortens the tick */
    }
    else if (ahead < -SIM_TICK_NS * SIM_LAG_TICKS)
    {
        *next = now; /* fell far behind (debugger, load): resynchronise */
    }
}

/* One lockstep step: wait for credit, run it, acknowledge.            */
static bool lockstep_step(void)
{
    uint32_t n = sim_socket_take_steps();

    if (n == 0u)
    {
        if (sim_socket_lost_lockstep())
        {
            return (false); /* the stepping client has gone: done */
        }

        (void) sim_socket_wait(SIM_WAIT_MS); /* timeout and input alike */
        sim_socket_poll();

        return (true);
    }

    for (uint32_t i = 0; i < n && i <= SIM_WORD_MASK; i++)
    {
        sim_script_tick(sim_clock_cs());
        sim_clock_tick();
        sim_socket_poll(); /* other peripherals keep talking */
    }

    sim_socket_ack(n);

    return (AGC_ASSERT(n <= SIM_WORD_MASK));
}

/* The harness loop. RULE 2 EXCEPTION (docs/CODING_STANDARD.md): in the
 * server modes without --max-cs this is the one loop with no bound: the
 * AGC runs until the process is stopped or the lockstep client leaves. */
static void run(const sim_opts_t *o)
{
    struct timespec next;
    bool            sockets = (o->mode != SIM_MODE_HEADLESS);

    if (!AGC_ASSERT(o != NULL) || clock_gettime(CLOCK_MONOTONIC, &next) != 0)
    {
        return;
    }

    for (;;)
    {
        if (o->max_cs != 0u && sim_clock_cs() >= o->max_cs)
        {
            return;
        }

        if (o->mode == SIM_MODE_LOCKSTEP)
        {
            if (!lockstep_step())
            {
                return;
            }

            continue;
        }

        sim_socket_poll();
        sim_script_tick(sim_clock_cs());
        sim_clock_tick();

        if (sockets && o->mode == SIM_MODE_REALTIME)
        {
            pace(&next);
        }
    }
}

static void print_summary(const sim_opts_t *o)
{
    sim_stats_t s;

    if (!AGC_ASSERT(o != NULL))
    {
        return;
    }

    sim_clock_stats(&s);
    /* printf results are irrelevant: the run is over (rule 7 note).    */
    (void) printf("agc_sim %s: %lu cs, %lu jobs, %lu restarts, %lu busy, "
                  "%lu errors, MODREG %d, FAILREG %05o %05o %05o\n",
                  (o->vehicle == AGC_VEHICLE_LM) ? "LM" : "CSM",
                  (unsigned long) s.cs, (unsigned long) s.jobs,
                  (unsigned long) s.restarts, (unsigned long) s.busy_ticks,
                  (unsigned long) s.errors, (int) agc_erase_modreg(),
                  (unsigned) agc_erase_failreg(0),
                  (unsigned) agc_erase_failreg(1),
                  (unsigned) agc_erase_failreg(2));
    (void) printf("agc_sim rupts: T6 %lu T5 %lu T3 %lu T4 %lu KEY1 %lu KEY2 %lu "
                  "UP %lu DOWN %lu RADAR %lu HAND %lu\n",
                  (unsigned long) s.rupts[0], (unsigned long) s.rupts[1],
                  (unsigned long) s.rupts[2], (unsigned long) s.rupts[3],
                  (unsigned long) s.rupts[4], (unsigned long) s.rupts[5],
                  (unsigned long) s.rupts[6], (unsigned long) s.rupts[7],
                  (unsigned long) s.rupts[8],
                  (unsigned long) s.rupts[9]);
}

static agc_status_t start(const sim_opts_t *o)
{
    if (!AGC_ASSERT(o != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_power_on(o->vehicle); /* state, lanes, exec   */

    if (st == AGC_STATUS_OK && o->pad != NULL)
    {
        st = sim_pad_load(o->pad); /* before GOJAM, like a core image */
    }

    st = (st == AGC_STATUS_OK) ? sim_clock_init() : st;
    st = (st == AGC_STATUS_OK) ? agc_rupt_go() : st; /* GOJAM -> GOPROG */
    sim_channels_init();

    if (st == AGC_STATUS_OK && o->script != NULL)
    {
        st = sim_script_open(o->script);
    }

    if (st == AGC_STATUS_OK && o->mode != SIM_MODE_HEADLESS)
    {
        st = sim_socket_open(o->port);
    }

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

int main(int argc, char *argv[])
{
    sim_opts_t o;

    if (!parse_args(argc, argv, &o))
    {
        (void) fprintf(stderr, "usage: agc_sim [--vehicle csm|lm] [--port N] "
                       "[--max-cs N] [--script F] [--pad F] "
                       "[--headless|--lockstep|--fast]\n");

        return (2); /* fprintf result irrelevant: exiting */
    }

    agc_status_t st = start(&o);

    if (st != AGC_STATUS_OK)
    {
        (void) fprintf(stderr, "agc_sim: start-up failed (status %d)\n",
                       (int) st);

        return (2); /* fprintf result irrelevant: exiting */
    }

    run(&o);
    sim_socket_close();
    sim_script_close();
    print_summary(&o);

    sim_stats_t s;

    sim_clock_stats(&s);

    return ((s.errors != 0u) ? 1 : 0);
}
