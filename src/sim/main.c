/**********************************************************************
 * FILE NAME: main.c
 *
 * PURPOSE:
 *   build/agc_sim: the translated flight software on the host, as a
 *   drop-in for yaAGC under the Virtual AGC launcher and peripherals.
 *   Command line, power-on, and the harness loop.
 *
 * AGC SOURCE:
 *   Power-on: DOFSTART, the machine fresh start through GOJAM -> GOPROG
 *   (FRESH_START_AND_RESTART C p.184 / L p.211), via agc_rupt_go. The
 *   command line is yaAGC's (virtualagc yaAGC/agc_cli.c, and
 *   VirtualAGC.cpp FormCommands for the launcher's use of it).
 *
 * EXTERNAL VARIABLES:
 *   none (options are a local struct)
 *
 * EXTERNAL REFERENCES:
 *   Name                Description
 *   ------------------  --------------------------------------------
 *   agc_power_on        host power-on: vehicle, state, lanes, exec
 *   agc_rupt_go         GOJAM -> GOPROG
 *   sim_clock_tick      one centisecond (clock.c)
 *   sim_socket_*        peripheral server (socket.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 2: an argument not understood, a rope other than
 *   Comanche055 / Luminary099, an unreadable --cfg, --rope, --pad or
 *   --script file, or no port to listen on (each reported on stderr).
 *   Exit status 1: the run counted errors (a handler or job returning
 *   AGC_STATUS_BAD_ARG, clock.c). Else 0.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   RULE 2 EXCEPTION (docs/CODING_STANDARD.md): run() is the one loop
 *   with no compile-time bound. In the server modes without --max-cs the
 *   AGC runs until the process is stopped or the lockstep client leaves.
 *
 * NOTES:
 *   Usage: agc_sim [--vehicle csm|lm] [--port N] [--max-cs N]
 *                  [--script F] [--pad F] [--core F]
 *                  [--headless | --lockstep | --fast] [csm|lm] [cs]
 *                  [exec-ropes-file [core-resume-file]]
 *     (default)   peripheral server: listens on ports N..N+9 (default
 *                 19697 for the CSM, 19797 for the LM: the Virtual AGC
 *                 port conventions) and runs in real time, 1 cs of AGC
 *                 time per 10 ms; yaDSKY2, yaTelemetry, ... connect as
 *                 to yaAGC.
 *     --fast      server without real-time pacing.
 *     --lockstep  server whose clock advances only when a client asks
 *                 (channel 0150 = N cs, answered by 0151); used by
 *                 tools/sim_client.py so scenarios are deterministic.
 *     --headless  no sockets: run --max-cs centiseconds (default 1000)
 *                 and print a summary.
 *     --script F  timed input packets (script.c), in any mode.
 *     --pad F     erasables set before GOJAM (pad.c). --resume F does
 *                 the same when F is a pad file (the launcher's "custom
 *                 core file" start-up); a yaAGC core dump cannot be
 *                 mapped onto the typed C erasable: logged, fresh start.
 *     --core F    (also --exec, --rope, or the first bare word) the rope
 *                 image: its name picks the vehicle (Comanche055 -> CSM,
 *                 Luminary099 -> LM; others refused) and it becomes the
 *                 fixed memory ROPECHK / SHOW-BANKSUM sum (rope.c).
 *                 Without it the vehicle's rope is taken from
 *                 tools/oracle/ropes.
 *   The bare words csm / lm and a bare number are --vehicle and
 *   --max-cs. yaAGC's command line (the launcher runs
 *     ../bin/yaAGC --core="source/Comanche055/Comanche055.bin"
 *                  --port=19697 --cfg=CM.ini [--resume=CM.core] [--debug]
 *   from its Resources directory): options take one or two dashes and
 *   `=VALUE` or a separate value; --cfg names the vehicle when no rope
 *   does (a CMSIM / LMSIM line, yaAGC CliParseCfg); --cd changes
 *   directory; --quiet drops the banner; the debugger, core-dump and
 *   alarm switches (k_ignored) are logged and ignored.
 *
 * ALGORITHM:
 *   parse; power on (agc_power_on, rope, pad, channels, GOJAM); run the
 *   clock in the chosen mode; print the summary.
 **********************************************************************/
#define _POSIX_C_SOURCE 200809L /* POSIX.1-2008: clock_gettime, chdir */

#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <time.h>
#include <unistd.h>

#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "internal.h"

#define SIM_PORT_CSM   19697        /* Virtual AGC: CM peripherals 19697-19706 */
#define SIM_PORT_LM    19797        /* LM peripherals 19797-19806              */
#define SIM_PORT_TOP   65526        /* highest base port (ten ports)           */
#define SIM_DEFAULT_CS 1000UL       /* headless default run, cs                */
#define SIM_MAX_RUN_CS 0x0FFFFFFFUL /* longest run, cs (2^28: TIME2/TIME1)     */
#define SIM_TICK_NS    10000000L    /* 10 ms                                   */
#define SIM_NS_PER_S   1000000000L  /* nanoseconds per second                  */
#define SIM_WAIT_MS    20           /* lockstep: longest wait for a packet     */
#define SIM_LAG_TICKS  100L         /* resynchronise after 1 s behind          */
#define SIM_CFG_LINES  1000         /* bound on --cfg lines read               */
#define SIM_CFG_LINE   130          /* longest --cfg line                      */
#define SIM_EXIT_USAGE 2            /* exit status: start-up refused           */

typedef enum sim_mode
{
    SIM_MODE_REALTIME = 0, /* server, paced to wall-clock time */
    SIM_MODE_FAST,         /* server, unpaced                  */
    SIM_MODE_LOCKSTEP,     /* server, stepped by a client      */
    SIM_MODE_HEADLESS      /* no sockets                       */
} sim_mode_t;

/* Options that take a value, in the order they are tried.               */
typedef enum sim_opt
{
    SIM_OPT_VEHICLE = 0, /* --vehicle csm|lm             */
    SIM_OPT_PORT,        /* --port N                     */
    SIM_OPT_MAX_CS,      /* --max-cs N                   */
    SIM_OPT_SCRIPT,      /* --script F                   */
    SIM_OPT_PAD,         /* --pad F                      */
    SIM_OPT_RESUME,      /* --resume F (yaAGC)           */
    SIM_OPT_CORE,        /* --core F (yaAGC)             */
    SIM_OPT_EXEC,        /* --exec F (yaAGC)             */
    SIM_OPT_ROPE,        /* --rope F                     */
    SIM_OPT_CFG,         /* --cfg F (yaAGC)              */
    SIM_OPT_CD,          /* --cd DIR (yaAGC)             */
    SIM_OPT_COUNT        /* number of value options      */
} sim_opt_t;

typedef struct sim_opts
{
    agc_vehicle_t     vehicle;     /* the vehicle to run                      */
    bool              vehicle_set; /* --vehicle or csm / lm given             */
    sim_mode_t        mode;        /* run mode                                */
    int               port;        /* base port; 0 = the vehicle's default    */
    unsigned long int max_cs;      /* 0: unbounded (server modes), cs         */
    const char       *script;      /* --script                                */
    const char       *pad;         /* --pad: erasable pad load                */
    const char       *resume;      /* --resume: pad file, or a yaAGC core     */
    const char       *core;        /* --core / --exec / --rope / positional   */
    const char       *cfg;         /* --cfg: yaDSKY2 ini (CMSIM / LMSIM)      */
    const char       *cd;          /* --cd: working directory                 */
    bool              quiet;       /* --quiet: no start-up banner             */
    bool              bad_value;   /* an option value was not understood      */
} sim_opts_t;

/* Names of the value options, indexed by sim_opt_t (one dash: a second
 * leading dash is dropped first).                                       */
static const char *const k_value_names[SIM_OPT_COUNT] = {"-vehicle", "-port", "-max-cs", "-script", "-pad", "-resume", "-core", "-exec", "-rope", "-cfg", "-cd"};

/* yaAGC options that mean nothing here (no debugger, no core dumps, no
 * hardware alarms): accepted, logged, ignored.                         */
static const char *const k_ignored[] = {"-debug", "-nodebug", "-directory", "-symtab", "-symbols", "-fullname", "-interlace", "-dump-time", "-cdu-log", "-debug-dsky", "-debug-deda", "-deda-quiet", "-inhibit-alarms", "-show-alarms", "-initialize-sunburst-37", "-no-resume", "-command", "-interpreter", "-nx", "-args"};

static bool         parse_args(int argc, char *argv[], sim_opts_t *o);
static agc_status_t start(const sim_opts_t *o, const char *argv0);
static void         run(const sim_opts_t *o);
static void         print_summary(const sim_opts_t *o);

/**********************************************************************
 * AGC: none -- the harness program.
 *
 * ARGUMENTS:
 *   argc  int     I  argument count
 *   argv  char *[] I  arguments (yaAGC's command line or ours)
 *
 * RETURNS: int -- 0 clean run, 1 errors counted, 2 start-up refused
 **********************************************************************/
int main(int argc, char *argv[])
{
    sim_opts_t  o;          /* the options */
    sim_stats_t s;          /* run statistics */
    int         status = 0; /* exit status */

    if (!parse_args(argc, argv, &o))
    {
        (void) fprintf(stderr, "usage: agc_sim [--vehicle csm|lm] [--port N] [--max-cs N] [--script F] [--pad F] [--resume F] [--core ROPE] [--cfg F] [--cd DIR] [--quiet] [--headless|--lockstep|--fast] [csm|lm] [cs] [rope [resume]]\n"); /* stdlib diagnostic */
        status = SIM_EXIT_USAGE;
    }
    else if (start(&o, argv[0]) != AGC_STATUS_OK)
    {
        (void) fprintf(stderr, "agc_sim: start-up failed\n"); /* stdlib diagnostic */
        status = SIM_EXIT_USAGE;
    }
    else
    {
        run(&o);
        sim_socket_close();
        sim_script_close();
        print_summary(&o);
        sim_clock_stats(&s);
        status = (s.errors != 0u) ? 1 : 0; /* 1: errors counted */
    }

    return (status);
}

/**********************************************************************
 * AGC: none -- a decimal number option value.
 *
 * ARGUMENTS:
 *   v    const char *         I  the text
 *   max  unsigned long int    I  largest value accepted
 *   out  unsigned long int *  O  the value
 *
 * RETURNS: bool -- false for text that is not a whole number <= max
 **********************************************************************/
static bool number_value(const char *v, unsigned long int max, unsigned long int *out)
{
    char *end = NULL; /* first character not converted */

    *out = strtoul(v, &end, 10);

    return (end != v && *end == '\0' && *out <= max);
}

/**********************************************************************
 * AGC: none -- the value of option `name` given as `-name=V` or
 * `-name V` (then *used = 1).
 *
 * ARGUMENTS:
 *   arg   const char *  I  the argument, one leading dash
 *   name  const char *  I  option name with its dash
 *   next  const char *  I  the following argument, or NULL
 *   used  int *         O  1 when next was taken
 *
 * RETURNS: const char * -- the value; NULL when arg is not that option
 **********************************************************************/
static const char *opt_value(const char *arg, const char *name, const char *next, int *used)
{
    size_t      n = strlen(name); /* name length */
    const char *v = NULL;         /* the value */

    if (strncmp(arg, name, n) == 0 && arg[n] == '=')
    {
        v = &arg[n + 1];
    }
    else if (strncmp(arg, name, n) == 0 && arg[n] == '\0' && next != NULL)
    {
        v     = next;
        *used = 1;
    }
    else
    {
        /* not this option */
    }

    return (v);
}

/**********************************************************************
 * AGC: none -- store one value option.
 *
 * ARGUMENTS:
 *   o   sim_opts_t *  I/O  options
 *   id  sim_opt_t     I    which option
 *   v   const char *  I    its value
 **********************************************************************/
static void store_option(sim_opts_t *o, sim_opt_t id, const char *v)
{
    unsigned long int n = 0; /* a number value */

    switch (id)
    {
        case SIM_OPT_VEHICLE:
            o->vehicle     = (strcmp(v, "lm") == 0) ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM; /* csm otherwise */
            o->vehicle_set = true;
            o->bad_value   = o->bad_value || (strcmp(v, "lm") != 0 && strcmp(v, "csm") != 0);
            break;
        case SIM_OPT_PORT:
            o->bad_value   = o->bad_value || !number_value(v, SIM_PORT_TOP, &n);
            o->port        = (int) n;
            break;
        case SIM_OPT_MAX_CS:
            o->bad_value   = o->bad_value || !number_value(v, SIM_MAX_RUN_CS, &o->max_cs);
            break;
        case SIM_OPT_SCRIPT:
            o->script      = v;
            break;
        case SIM_OPT_PAD:
            o->pad         = v;
            break;
        case SIM_OPT_RESUME:
            o->resume      = v;
            break;
        case SIM_OPT_CFG:
            o->cfg         = v;
            break;
        case SIM_OPT_CD:
            o->cd          = v;
            break;
        case SIM_OPT_CORE:
        case SIM_OPT_EXEC:
        case SIM_OPT_ROPE:
        case SIM_OPT_COUNT:
        default:
            o->core        = v;                                                         /* the rope image */
            break;
    }
}

/**********************************************************************
 * AGC: none -- a switch without a value (mode, --quiet) or an ignored
 * yaAGC option (logged).
 *
 * ARGUMENTS:
 *   o    sim_opts_t *  I/O  options
 *   arg  const char *  I    the argument, one leading dash
 *
 * RETURNS: bool -- false when arg is none of them
 **********************************************************************/
static bool switch_option(sim_opts_t *o, const char *arg)
{
    bool hit = true;                                               /* arg understood */
    int  n   = (int) (sizeof (k_ignored) / sizeof (k_ignored[0])); /* ignored options */

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

        for (int i = 0; i < n && !hit; i++)
        {
            size_t len = strlen(k_ignored[i]); /* option name length */

            hit = strncmp(arg, k_ignored[i], len) == 0 && (arg[len] == '\0' || arg[len] == '=');
        }

        if (hit)
        {
            (void) fprintf(stderr, "agc_sim: -%s ignored (no meaning for the C program)\n", arg); /* stdlib diagnostic */
        }
    }

    return (hit);
}

/**********************************************************************
 * AGC: none -- one argument (and maybe the next). Bare words: csm / lm
 * (vehicle), a number (--max-cs), else yaAGC's positional
 * exec-ropes-file, then core-resume-file.
 *
 * ARGUMENTS:
 *   o     sim_opts_t *  I/O  options
 *   arg   const char *  I    the argument
 *   next  const char *  I    the following argument, or NULL
 *   used  int *         O    1 when next was taken
 *
 * RETURNS: bool -- false when the argument is not understood
 **********************************************************************/
static bool parse_one(sim_opts_t *o, const char *arg, const char *next, int *used)
{
    const char *a  = (strncmp(arg, "--", 2) == 0) ? &arg[1] : arg; /* one dash, as yaAGC */
    const char *v  = NULL;                                         /* an option value */
    bool        ok = true;                                         /* understood */

    if (strcmp(a, "csm") == 0 || strcmp(a, "lm") == 0)
    {
        store_option(o, SIM_OPT_VEHICLE, a);
    }
    else if (a[0] >= '0' && a[0] <= '9')
    {
        store_option(o, SIM_OPT_MAX_CS, a);
    }
    else if (a[0] != '-')
    {
        store_option(o, (o->core == NULL) ? SIM_OPT_CORE : SIM_OPT_RESUME, a); /* rope first, then resume file */
    }
    else
    {
        for (int i = 0; i < (int) SIM_OPT_COUNT && v == NULL; i++)
        {
            v = opt_value(a, k_value_names[i], next, used);

            if (v != NULL)
            {
                store_option(o, (sim_opt_t) i, v);
            }
        }

        ok = (v != NULL) || switch_option(o, a);
    }

    return (ok);
}

/**********************************************************************
 * AGC: none -- --cfg: the yaDSKY2 ini yaAGC is given names the vehicle
 * (a line CMSIM or LMSIM, yaAGC CliParseCfg).
 *
 * ARGUMENTS:
 *   path     const char *     I  the ini file
 *   vehicle  agc_vehicle_t *  O  the vehicle, when found
 *   found    bool *           O  a CMSIM / LMSIM line was found
 *
 * RETURNS: bool -- false when the file cannot be read (an error in
 *          yaAGC too; reported)
 **********************************************************************/
static bool cfg_vehicle(const char *path, agc_vehicle_t *vehicle, bool *found)
{
    FILE *f = fopen(path, "r"); /* the ini file */
    char  line[SIM_CFG_LINE];   /* one line */

    *found = false;

    if (f == NULL)
    {
        (void) fprintf(stderr, "agc_sim: cannot read configuration file %s\n", path); /* stdlib diagnostic */
    }

    for (int i = 0; i < SIM_CFG_LINES && f != NULL && fgets(line, (int) sizeof (line), f) != NULL; i++)
    {
        line[strcspn(line, "\r\n")] = '\0';

        if (strcmp(line, "LMSIM") == 0 || strcmp(line, "CMSIM") == 0)
        {
            *vehicle = (line[0] == 'L') ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM; /* LMSIM / CMSIM */
            *found   = true;
        }
    }

    if (f != NULL)
    {
        (void) fclose(f); /* stdlib: read-only file, nothing to lose */
    }

    return (f != NULL);
}

/**********************************************************************
 * AGC: none -- the vehicle from the rope name (--core), else --cfg,
 * unless --vehicle gave it; a rope that is not Comanche055 /
 * Luminary099, or one that contradicts --vehicle, is refused.
 *
 * ARGUMENTS:
 *   o  sim_opts_t *  I/O  options; vehicle set
 *
 * RETURNS: bool -- false when refused (reported)
 **********************************************************************/
static bool resolve_vehicle(sim_opts_t *o)
{
    agc_vehicle_t v     = o->vehicle; /* vehicle found */
    bool          found = false;      /* the rope or --cfg named one */
    bool          ok    = true;       /* not refused */

    if (o->core != NULL)
    {
        ok    = sim_rope_vehicle(o->core, &v) == AGC_STATUS_OK;
        found = ok;

        if (!ok)
        {
            (void) fprintf(stderr, "agc_sim: %s: only the Comanche055 (CSM) and Luminary099 (LM) ropes are translated\n", o->core); /* stdlib diagnostic */
        }
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
        (void) fprintf(stderr, "agc_sim: --vehicle contradicts the rope / --cfg\n"); /* stdlib diagnostic */
        ok = false;
    }

    o->vehicle = (ok && found) ? v : o->vehicle; /* the rope / --cfg wins */

    return (ok);
}

/**********************************************************************
 * AGC: none -- the command line into *o, then --cd, the vehicle, the
 * default port and run length.
 *
 * ARGUMENTS:
 *   argc  int           I  argument count
 *   argv  char *[]      I  arguments
 *   o     sim_opts_t *  O  options
 *
 * RETURNS: bool -- false when an argument is not understood or refused
 **********************************************************************/
static bool parse_args(int argc, char *argv[], sim_opts_t *o)
{
    bool ok = true; /* every argument understood */

    (void) memset(o, 0, sizeof (*o)); /* stdlib: returns o */
    o->vehicle = AGC_VEHICLE_CSM;
    o->mode    = SIM_MODE_REALTIME;

    for (int i = 1; i < argc && ok; i++)
    {
        int used = 0; /* the next argument was the value */

        ok = parse_one(o, argv[i], (i + 1 < argc) ? argv[i + 1] : NULL, &used); /* NULL after the last */

        if (!ok || o->bad_value)
        {
            (void) fprintf(stderr, "agc_sim: argument %s not understood\n", argv[i]); /* stdlib diagnostic */
            ok = false;
        }

        i += used;
    }

    if (ok && o->cd != NULL && chdir(o->cd) != 0)
    {
        (void) fprintf(stderr, "agc_sim: cannot change directory to %s\n", o->cd); /* stdlib diagnostic */
        ok = false;
    }

    ok = ok && resolve_vehicle(o);

    if (o->port == 0)
    {
        o->port = (o->vehicle == AGC_VEHICLE_LM) ? SIM_PORT_LM : SIM_PORT_CSM; /* Virtual AGC defaults */
    }

    if (o->mode == SIM_MODE_HEADLESS && o->max_cs == 0u)
    {
        o->max_cs = SIM_DEFAULT_CS;
    }

    return (ok);
} /* parse_args */

/**********************************************************************
 * AGC: none -- fixed memory: the --core rope, else the vehicle's rope
 * from tools/oracle/ropes; without one ROPECHK passes every bank
 * (logged). Prints the banner.
 *
 * ARGUMENTS:
 *   o      const sim_opts_t *  I  options
 *   argv0  const char *        I  argv[0]
 *
 * RETURNS: agc_status_t -- OK; the load status for a --core rope that
 *          could not be loaded
 **********************************************************************/
static agc_status_t load_rope(const sim_opts_t *o, const char *argv0)
{
    const char  *name = (o->vehicle == AGC_VEHICLE_LM) ? "LM, Luminary099" : "CSM, Comanche055";               /* banner */
    agc_status_t st   = (o->core != NULL) ? sim_rope_load(o->core) : sim_rope_load_default(o->vehicle, argv0); /* given or default */

    if (st == AGC_STATUS_NOT_FOUND && o->core == NULL)
    {
        (void) fprintf(stderr, "agc_sim: no rope image found: self-check sums no fixed memory\n"); /* stdlib diagnostic */
        st = AGC_STATUS_OK;
    }
    else if (st != AGC_STATUS_OK)
    {
        (void) fprintf(stderr, "agc_sim: cannot load rope %s\n", (o->core != NULL) ? o->core : "(default)"); /* stdlib diagnostic */
    }
    else if (!o->quiet)
    {
        (void) printf("agc_sim: %s, rope %s, port %d\n", name, (o->core != NULL) ? o->core : "tools/oracle/ropes", o->port); /* stdlib banner */
        (void) fflush(stdout);                                                                                               /* stdlib: the launcher's log sees it at once */
    }
    else
    {
        /* quiet */
    }

    return (st);
}

/**********************************************************************
 * AGC: none -- power-on as yaAGC starts: agc_power_on, the rope, the
 * pad / --resume pad, the clock and the power-on channels (yaAGC
 * agc_engine_init), then GOJAM -> GOPROG; the script and the server.
 *
 * ARGUMENTS:
 *   o      const sim_opts_t *  I  options
 *   argv0  const char *        I  argv[0]
 *
 * RETURNS: agc_status_t -- OK, or the first step's failure
 **********************************************************************/
static agc_status_t start(const sim_opts_t *o, const char *argv0)
{
    agc_power_on(o->vehicle); /* state, lanes, exec */

    agc_status_t st = load_rope(o, argv0); /* fixed memory */

    st = (st == AGC_STATUS_OK && o->pad != NULL) ? sim_pad_load(o->pad) : st; /* before GOJAM, like a core image */

    if (st == AGC_STATUS_OK && o->resume != NULL && sim_pad_is_pad(o->resume))
    {
        st = sim_pad_load(o->resume);
    }
    else if (st == AGC_STATUS_OK && o->resume != NULL)
    {
        (void) fprintf(stderr, "agc_sim: resume file %s is not a pad file: fresh start\n", o->resume); /* stdlib diagnostic */
    }
    else
    {
        /* no --resume, or start-up already failed */
    }

    sim_clock_init();
    st = (st == AGC_STATUS_OK) ? sim_channels_init() : st;                                      /* channel 30 */
    st = (st == AGC_STATUS_OK) ? agc_rupt_go() : st;                                            /* GOJAM -> GOPROG */
    st = (st == AGC_STATUS_OK && o->script != NULL) ? sim_script_open(o->script) : st;          /* --script */
    st = (st == AGC_STATUS_OK && o->mode != SIM_MODE_HEADLESS) ? sim_socket_open(o->port) : st; /* the server */

    return (st);
}

/**********************************************************************
 * AGC: none -- real-time pacing: sleep until the next 10 ms boundary;
 * after falling more than 1 s behind (debugger, load) resynchronise.
 *
 * ARGUMENTS:
 *   next  struct timespec *  I/O  the boundary, advanced by 10 ms
 **********************************************************************/
static void pace(struct timespec *next)
{
    struct timespec now; /* monotonic time */

    if (clock_gettime(CLOCK_MONOTONIC, &now) == 0)
    {
        next->tv_nsec += SIM_TICK_NS;

        if (next->tv_nsec >= SIM_NS_PER_S)
        {
            next->tv_nsec -= SIM_NS_PER_S;
            next->tv_sec++;
        }

        long int ahead = (long int) (next->tv_sec - now.tv_sec) * SIM_NS_PER_S + (next->tv_nsec - now.tv_nsec); /* ns to sleep */

        if (ahead > 0)
        {
            struct timespec d = {ahead / SIM_NS_PER_S, ahead % SIM_NS_PER_S}; /* the sleep */

            (void) nanosleep(&d, NULL); /* stdlib: an early wake-up only shortens the tick */
        }
        else if (ahead < -SIM_TICK_NS * SIM_LAG_TICKS)
        {
            *next = now;
        }
        else
        {
            /* behind by less than a second: catch up */
        }
    }
}

/**********************************************************************
 * AGC: none -- one lockstep step: wait for credit, run it, acknowledge.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- false once the stepping client has gone
 **********************************************************************/
static bool lockstep_step(void)
{
    uint32_t n    = sim_socket_take_steps();     /* credit, cs */
    bool     more = !sim_socket_lost_lockstep(); /* the client is still there */

    if (n == 0u && more && sim_socket_wait(SIM_WAIT_MS))
    {
        sim_socket_poll(); /* input (or a connection) is waiting */
    }

    for (uint32_t i = 0; i < n && i < (uint32_t) SIM_MAX_RUN_CS; i++)
    {
        sim_script_tick(sim_clock_cs());
        sim_clock_tick();
        sim_socket_poll(); /* other peripherals keep talking */
    }

    if (n != 0u)
    {
        sim_socket_ack(n);
    }

    return (more || n != 0u);
}

/**********************************************************************
 * AGC: none -- the harness loop. RULE 2 EXCEPTION (file prolog): in the
 * server modes without --max-cs this loop has no bound; it ends when
 * the process is stopped or the lockstep client leaves.
 *
 * ARGUMENTS:
 *   o  const sim_opts_t *  I  options
 **********************************************************************/
static void run(const sim_opts_t *o)
{
    struct timespec next;                                               /* real-time boundary */
    bool            going = clock_gettime(CLOCK_MONOTONIC, &next) == 0; /* keep running */

    while (going)
    {
        if (o->max_cs != 0u && sim_clock_cs() >= o->max_cs)
        {
            going = false;
        }
        else if (o->mode == SIM_MODE_LOCKSTEP)
        {
            going = lockstep_step();
        }
        else
        {
            sim_socket_poll();
            sim_script_tick(sim_clock_cs());
            sim_clock_tick();

            if (o->mode == SIM_MODE_REALTIME)
            {
                pace(&next);
            }
        }
    }
}

/**********************************************************************
 * AGC: none -- the end-of-run summary on stdout.
 *
 * ARGUMENTS:
 *   o  const sim_opts_t *  I  options
 **********************************************************************/
static void print_summary(const sim_opts_t *o)
{
    sim_stats_t s; /* run statistics */

    sim_clock_stats(&s);
    (void) printf("agc_sim %s: %lu cs, %lu jobs, %lu restarts, %lu busy, %lu errors, MODREG %d, FAILREG %05o %05o %05o\n", (o->vehicle == AGC_VEHICLE_LM) ? "LM" : "CSM", (unsigned long int) s.cs, (unsigned long int) s.jobs, (unsigned long int) s.restarts, (unsigned long int) s.busy_ticks, (unsigned long int) s.errors, (int) agc_erase_modreg(), (unsigned int) agc_erase_failreg(0), (unsigned int) agc_erase_failreg(1), (unsigned int) agc_erase_failreg(2));                                                                                            /* stdlib summary; vehicle name */
    (void) printf("agc_sim rupts: T6 %lu T5 %lu T3 %lu T4 %lu KEY1 %lu KEY2 %lu UP %lu DOWN %lu RADAR %lu HAND %lu\n", (unsigned long int) s.rupts[SIM_IRQ_T6], (unsigned long int) s.rupts[SIM_IRQ_T5], (unsigned long int) s.rupts[SIM_IRQ_T3], (unsigned long int) s.rupts[SIM_IRQ_T4], (unsigned long int) s.rupts[SIM_IRQ_KEY1], (unsigned long int) s.rupts[SIM_IRQ_KEY2], (unsigned long int) s.rupts[SIM_IRQ_UPLINK], (unsigned long int) s.rupts[SIM_IRQ_DOWNLINK], (unsigned long int) s.rupts[SIM_IRQ_RADAR], (unsigned long int) s.rupts[SIM_IRQ_HAND]); /* stdlib summary */
}
