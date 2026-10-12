/*
 * main.c -- build/agc_sim: the translated flight software on the host.
 *
 * Usage: agc_sim [--vehicle csm|lm] [--port N] [--max-cs N] [--script F]
 *                [--pad F] [--core F] [--headless | --lockstep | --fast]
 *                [csm|lm] [cs] [exec-ropes-file]
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
 *   --pad F     erasables set before GOJAM (pad.c): REFSMMAT, flagwords,
 *               state vectors. --resume F does the same when F is a pad
 *               file (the launcher's "custom core file" start-up).
 *   --core F    (also --exec, --rope, or the first bare word) the rope
 *               image: its name picks the vehicle (Comanche055 -> CSM,
 *               Luminary099 -> LM; others refused) and it becomes the
 *               fixed memory ROPECHK / SHOW-BANKSUM sum (rope.c). Without
 *               it the vehicle's rope is taken from tools/oracle/ropes.
 * The bare words csm / lm and a bare number are --vehicle and --max-cs.
 *
 * yaAGC command line (the Virtual AGC launcher runs
 *   ../bin/yaAGC --core="source/Comanche055/Comanche055.bin" --port=19697
 *                --cfg=CM.ini [--resume=CM.core] [--debug ...]
 * from its Resources directory): options take one or two dashes, --cfg
 * names the vehicle when no rope does (CMSIM / LMSIM), --cd changes
 * directory, --quiet drops the banner; --resume=F loads F as a pad file
 * (pad.c) when it is one, else logs it (a yaAGC core dump cannot be
 * mapped onto the typed C erasable: fresh start); --debug, --symtab,
 * --directory and the other debugger / alarm / core-dump switches are
 * logged and ignored (no debugger).
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
#include <unistd.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define SIM_PORT_CSM   19697     /* Virtual AGC: CM peripherals 19697-19706 */
#define SIM_PORT_LM    19797     /* LM peripherals 19797-19806              */
#define SIM_DEFAULT_CS 1000ul    /* headless default run              */
#define SIM_MAX_RUN_CS 0x0FFFFFFFul
#define SIM_TICK_NS    10000000L /* 10 ms                           */
#define SIM_WAIT_MS    20
#define SIM_LAG_TICKS  100       /* resynchronise after 1 s behind    */
#define SIM_CFG_LINES  1000      /* bound on --cfg lines read          */

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
    bool          vehicle_set; /* --vehicle, csm/lm, the rope or --cfg */
    sim_mode_t    mode;
    int           port;
    unsigned long max_cs;      /* 0: unbounded (server modes)          */
    const char   *script;
    const char   *pad;         /* --pad: erasable pad load            */
    const char   *resume;      /* --resume: pad file, or a yaAGC core  */
    const char   *core;        /* --core / --exec / --rope / positional */
    const char   *cfg;         /* --cfg: yaDSKY2 ini (CMSIM / LMSIM)  */
    const char   *cd;          /* --cd: working directory             */
    bool          quiet;       /* --quiet: no start-up banner         */
} sim_opts_t;

/* yaAGC options that mean nothing here (no debugger, no core dumps, no
 * hardware alarms): accepted, logged, ignored.                         */
static const char *const k_ignored[] = {"-debug", "-nodebug", "-directory", "-symtab", "-symbols", "-fullname", "-interlace", "-dump-time", "-cdu-log", "-debug-dsky", "-debug-deda", "-deda-quiet", "-inhibit-alarms", "-show-alarms", "-initialize-sunburst-37", "-no-resume", "-command", "-interpreter", "-nx", "-args", "-version"};

/* The value of option `name` given as `-name=V` or `-name V` (then
 * *used = 1); NULL when `arg` is not that option. yaAGC accepts one or
 * two dashes; the caller has already dropped the first of two.         */
static const char *opt_value(const char *arg, const char *name, const char *next, int *used)
{
    size_t n = strlen(name);

    if (strncmp(arg, name, n) == 0)
    {
        if (arg[n] == '=')
        {
            return (&arg[n + 1]);
        }

        if (arg[n] == '\0' && next != NULL)
        {
            *used = 1;

            return (next);
        }
    }

    return (NULL);
}

/* An ignored yaAGC option (k_ignored, alone or with =VALUE): logged.   */
static bool ignored_option(const char *arg)
{
    int n = (int) (sizeof (k_ignored) / sizeof (k_ignored[0]));

    for (int i = 0; i < n; i++)
    {
        size_t len = strlen(k_ignored[i]);

        if (strncmp(arg, k_ignored[i], len) == 0 && (arg[len] == '\0' || arg[len] == '='))
        {
            (void) fprintf(stderr, "agc_sim: -%s ignored (no meaning for the C program)\n", arg); /* diagnostic */

            return (true);
        }
    }

    return (false);
}

/* The run-mode switches; false when `arg` is none of them.             */
static bool mode_option(sim_opts_t *o, const char *arg)
{
    bool hit = true;

    if (strcmp(arg, "-headless") == 0)
    {
        o->mode = SIM_MODE_HEADLESS;
    }
    else if (strcmp(arg, "-lockstep") == 0)
    {
        o->mode = SIM_MODE_LOCKSTEP;
    }
    else if (strcmp(arg, "-fast") == 0)
    {
        o->mode = SIM_MODE_FAST;
    }
    else if (strcmp(arg, "-quiet") == 0)
    {
        o->quiet = true;
    }
    else
    {
        hit = false;
    }

    return (hit);
}

/* Options with a value; false when `arg` is none of them.              */
static bool value_option(sim_opts_t *o, const char *arg, const char *next, int *used)
{
    const char *v = NULL;

    if ((v = opt_value(arg, "-vehicle", next, used)) != NULL)
    {
        o->vehicle     = (strcmp(v, "lm") == 0) ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM;
        o->vehicle_set = true;
    }
    else if ((v = opt_value(arg, "-port", next, used)) != NULL)
    {
        o->port = (int) strtol(v, NULL, 10);
    }
    else if ((v = opt_value(arg, "-max-cs", next, used)) != NULL)
    {
        o->max_cs = strtoul(v, NULL, 10);
    }
    else if ((v = opt_value(arg, "-script", next, used)) != NULL)
    {
        o->script = v;
    }
    else if ((v = opt_value(arg, "-pad", next, used)) != NULL)
    {
        o->pad = v;
    }
    else if ((v = opt_value(arg, "-resume", next, used)) != NULL)
    {
        o->resume = v;
    }
    else if ((v = opt_value(arg, "-core", next, used)) != NULL || (v = opt_value(arg, "-exec", next, used)) != NULL || (v = opt_value(arg, "-rope", next, used)) != NULL)
    {
        o->core = v;
    }
    else if ((v = opt_value(arg, "-cfg", next, used)) != NULL)
    {
        o->cfg = v;
    }
    else if ((v = opt_value(arg, "-cd", next, used)) != NULL)
    {
        o->cd = v;
    }
    else
    {
        return (false);
    }

    return (true);
}

/* One argument (and maybe the next); false when it is not understood.
 * Bare words: csm / lm (vehicle), a number (--max-cs), else yaAGC's
 * positional exec-ropes-file, then core-resume-file (--resume).       */
static bool parse_one(sim_opts_t *o, const char *arg, const char *next, int *used)
{
    const char *a  = (strncmp(arg, "--", 2) == 0) ? &arg[1] : arg; /* -- -> - as yaAGC */
    bool        ok = true;

    if (strcmp(a, "csm") == 0 || strcmp(a, "lm") == 0)
    {
        o->vehicle     = (a[0] == 'l') ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM;
        o->vehicle_set = true;
    }
    else if (a[0] >= '0' && a[0] <= '9')
    {
        o->max_cs = strtoul(a, NULL, 10);
    }
    else if (a[0] != '-')
    {
        if (o->core == NULL)
        {
            o->core = a;
        }
        else
        {
            o->resume = a; /* yaAGC's positional core-resume-file */
        }
    }
    else
    {
        ok = mode_option(o, a) || value_option(o, a, next, used) || ignored_option(a);
    }

    return (ok);
}

/* --cfg: the yaDSKY2 ini yaAGC is given names the vehicle (a line CMSIM
 * or LMSIM, yaAGC CliParseCfg). An unreadable file is an error there.  */
static bool cfg_vehicle(const char *path, agc_vehicle_t *vehicle, bool *found)
{
    FILE *f = fopen(path, "r");
    char  line[130];

    *found = false;

    if (f == NULL)
    {
        (void) fprintf(stderr, "agc_sim: cannot read configuration file %s\n", path); /* diagnostic */

        return (false);
    }

    for (int i = 0; i < SIM_CFG_LINES && fgets(line, (int) sizeof (line), f) != NULL; i++)
    {
        line[strcspn(line, "\r\n")] = '\0';

        if (strcmp(line, "LMSIM") == 0 || strcmp(line, "CMSIM") == 0)
        {
            *vehicle = (line[0] == 'L') ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM;
            *found   = true;
        }
    }

    (void) fclose(f); /* read-only file: nothing to lose */

    return (true);
}

/* The vehicle from the rope name (--core), else --cfg, unless --vehicle
 * gave it; a rope that is not Comanche055 / Luminary099, or one that
 * contradicts --vehicle, is refused.                                    */
static bool resolve_vehicle(sim_opts_t *o)
{
    agc_vehicle_t v     = o->vehicle;
    bool          found = false;
    bool          ok    = true;

    if (o->core != NULL)
    {
        ok = sim_rope_vehicle(o->core, &v) == AGC_STATUS_OK;

        if (!ok)
        {
            (void) fprintf(stderr, "agc_sim: %s: only the Comanche055 (CSM) and Luminary099 (LM) ropes are translated\n", o->core); /* diagnostic */
        }

        found = ok;
    }
    else if (o->cfg != NULL)
    {
        ok = cfg_vehicle(o->cfg, &v, &found);
    }
    else
    {
        /* --vehicle, csm / lm, or the CSM default */
    }

    if (ok && found && o->vehicle_set && v != o->vehicle)
    {
        (void) fprintf(stderr, "agc_sim: --vehicle contradicts the rope / --cfg\n"); /* diagnostic */
        ok = false;
    }

    o->vehicle = (ok && found) ? v : o->vehicle;

    return (ok);
}

static bool parse_args(int argc, char *argv[], sim_opts_t *o)
{
    (void) memset(o, 0, sizeof (*o)); /* returns o */
    o->vehicle = AGC_VEHICLE_CSM;
    o->mode    = SIM_MODE_REALTIME;

    for (int i = 1; i < argc; i++)
    {
        int used = 0;

        if (!parse_one(o, argv[i], (i + 1 < argc) ? argv[i + 1] : NULL, &used))
        {
            (void) fprintf(stderr, "agc_sim: unknown argument %s\n", argv[i]); /* diagnostic output only */

            return (false); /* fprintf result irrelevant: we stop here */
        }

        i += used;
    }

    if (o->cd != NULL && chdir(o->cd) != 0)
    {
        (void) fprintf(stderr, "agc_sim: cannot change directory to %s\n", o->cd); /* diagnostic */

        return (false);
    }

    if (!resolve_vehicle(o))
    {
        return (false);
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

    if (clock_gettime(CLOCK_MONOTONIC, &now) == 0)
    {
        next->tv_nsec += SIM_TICK_NS;

        if (next->tv_nsec >= 1000000000L)
        {
            next->tv_nsec -= 1000000000L;
            next->tv_sec++;
        }

        long ahead = (long) (next->tv_sec - now.tv_sec) * 1000000000L + (next->tv_nsec - now.tv_nsec);

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

        if (sim_socket_wait(SIM_WAIT_MS))
        {
            sim_socket_poll(); /* input (or a connection) is waiting */
        }

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

    if (clock_gettime(CLOCK_MONOTONIC, &next) == 0)
    {
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
}

static void print_summary(const sim_opts_t *o)
{
    sim_stats_t s;

    sim_clock_stats(&s);

    /* printf results are irrelevant: the run is over (rule 7 note).    */
    (void) printf("agc_sim %s: %lu cs, %lu jobs, %lu restarts, %lu busy, " "%lu errors, MODREG %d, FAILREG %05o %05o %05o\n", (o->vehicle == AGC_VEHICLE_LM) ? "LM" : "CSM", (unsigned long) s.cs, (unsigned long) s.jobs, (unsigned long) s.restarts, (unsigned long) s.busy_ticks, (unsigned long) s.errors, (int) agc_erase_modreg(), (unsigned) agc_erase_failreg(0), (unsigned) agc_erase_failreg(1), (unsigned) agc_erase_failreg(2)); /* diagnostic output only */
    (void) printf("agc_sim rupts: T6 %lu T5 %lu T3 %lu T4 %lu KEY1 %lu KEY2 %lu " "UP %lu DOWN %lu RADAR %lu HAND %lu\n", (unsigned long) s.rupts[0], (unsigned long) s.rupts[1], (unsigned long) s.rupts[2], (unsigned long) s.rupts[3], (unsigned long) s.rupts[4], (unsigned long) s.rupts[5], (unsigned long) s.rupts[6], (unsigned long) s.rupts[7], (unsigned long) s.rupts[8], (unsigned long) s.rupts[9]);                           /* diagnostic output only */
}

/* Fixed memory: the --core rope, else the vehicle's rope from
 * tools/oracle/ropes; without one ROPECHK passes every bank (logged).  */
static agc_status_t load_rope(const sim_opts_t *o, const char *argv0)
{
    const char  *name = (o->vehicle == AGC_VEHICLE_LM) ? "LM, Luminary099" : "CSM, Comanche055";
    agc_status_t st   = (o->core != NULL) ? sim_rope_load(o->core) : sim_rope_load_default(o->vehicle, argv0);

    if (st == AGC_STATUS_NOT_FOUND && o->core == NULL)
    {
        (void) fprintf(stderr, "agc_sim: no rope image found: self-check sums no fixed memory\n"); /* diagnostic */
        st = AGC_STATUS_OK;
    }
    else if (st != AGC_STATUS_OK)
    {
        (void) fprintf(stderr, "agc_sim: cannot load rope %s\n", o->core); /* diagnostic */
    }
    else if (!o->quiet)
    {
        (void) printf("agc_sim: %s, rope %s, port %d\n", name, (o->core != NULL) ? o->core : "tools/oracle/ropes", o->port); /* banner */
        (void) fflush(stdout);                                                                                               /* the launcher's log sees it at once */
    }
    else
    {
        /* quiet */
    }

    return (st);
}

/* --resume: a pad file is loaded like --pad; anything else (a yaAGC
 * core dump) is logged and the run is a fresh start.                   */
static agc_status_t load_resume(const char *path)
{
    agc_status_t st = AGC_STATUS_OK;

    if (sim_pad_is_pad(path))
    {
        st = sim_pad_load(path);
    }
    else
    {
        (void) fprintf(stderr, "agc_sim: resume file %s is not a pad file: fresh start\n", path); /* diagnostic */
    }

    return (st);
}

static agc_status_t start(const sim_opts_t *o, const char *argv0)
{
    agc_status_t st = agc_power_on(o->vehicle); /* state, lanes, exec   */

    st = (st == AGC_STATUS_OK) ? load_rope(o, argv0) : st;

    if (st == AGC_STATUS_OK && o->pad != NULL)
    {
        st = sim_pad_load(o->pad); /* before GOJAM, like a core image */
    }

    if (st == AGC_STATUS_OK && o->resume != NULL)
    {
        st = load_resume(o->resume);
    }

    st = (st == AGC_STATUS_OK) ? sim_clock_init() : st;
    st = (st == AGC_STATUS_OK) ? agc_rupt_go() : st; /* GOJAM -> GOPROG */
    st = (st == AGC_STATUS_OK) ? sim_channels_init() : st;

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
        (void) fprintf(stderr, "usage: agc_sim [--vehicle csm|lm] [--port N] " "[--max-cs N] [--script F] [--pad F] [--core ROPE] " "[--headless|--lockstep|--fast]\n"); /* diagnostic output only */

        return (2); /* fprintf result irrelevant: exiting */
    }

    agc_status_t st = start(&o, argv[0]);

    if (st != AGC_STATUS_OK)
    {
        (void) fprintf(stderr, "agc_sim: start-up failed (status %d)\n", (int) st); /* diagnostic output only */

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
