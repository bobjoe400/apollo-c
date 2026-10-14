/*
 * replay.c -- replay a NASSP scenario's AGC image on one engine (make nassp-replay).
 *
 * Built twice by sim/nassp/replay.py against NASSP's own src_sys/yaAGC/agc_engine.h
 * (the agc_t struct NASSP's C++ uses; GPL, downloaded at build time, never committed):
 *   ours:  this file + src/nassp/engine.c (the shim) + src/sim clock/channels/
 *          counters/rope + build/libagc.a
 *   stock: this file -DREPLAY_YAAGC + NASSP's yaAGC engine (agc_engine.c, ...)
 * and fed the same image. Both print the same transcript; the driver compares them
 * line for line. Host-only test tool, not part of libagc.
 *
 * Usage: replay IMAGE ROPE SECONDS UPLINK [gojam]
 *   IMAGE   a scenario's AGC_BEGIN .. AGC_END block (sim/nassp/images/NAME.agc)
 *   ROPE    Comanche055.bin / Luminary099.bin
 *   SECONDS AGC seconds of P00 after RSET, V37E00E (the integration steps)
 *   UPLINK  a DSKY key string sent through INLINK + UPRUPT the way NASSP's PCM
 *           telemetry sends MCC uplinks (csm_telecom.cpp), or "-" for none
 *   gojam   stock only: a GOJAM right after the load (yaAGC's ParityFail path), so the
 *           reference restarts as our shim does; the driver compares the downlist to
 *           this run (DAP, counters and channels after a restart, not a resumed image)
 *
 * Transcript ('#' lines are timing notes the driver does not compare):
 * for a Luminary image first "# radmodes-lrpos" (RADMODES bit 6 saved, and 5 s after the load),
 * then "# noun40" (the noun relay word 40 s after the load: the shim's load restart must resume
 * the running program from the saved phase table, PHASEn / TBASEn / LONGBASE);
 * "step csm TETCSM=hi,lo" / "step lem TETLEM=hi,lo" with that state-vector block (42
 * words, RRECT .. XKEP) whenever P00 integration moves TETCSM or TETLEM; then the
 * uplink's result (UPBUFF, UPSVFLAG, the blocks, DELVSLV and TIG, MODREG, DSPTAB);
 * "# dsky-latency" (V06N3, then the '6' key until the erasable shows noun 36); last (a
 * Luminary image: after "# noun40", the resumed program's list) the
 * downlist: DOWNRUPT 50 times a second as NASSP's PCM raises it, channels 34/35 from
 * ChannelOutput, one complete list ("#downlist ID", "#dn N hi lo wo" per word, wo the
 * word order code): '#' lines, compared by the driver word by word (DP tolerance, the
 * TIME2,TIME1 word and the words a load restart cannot reproduce left out).
 *
 * Timing: NASSP calls agc_engine once per memory cycle (11.71875 us); so does this.
 * Keys: channel 015 + KEYRUPT1, held 0.3 s, as NASSP's DSKY does.
 * Uplink words: 0.1 s apart and only while UPRUPT is not pending, as NASSP's PCM.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agc_engine.h"

#define MCT_S        0.00001171875 /* one memory cycle, s                     */
#define RUPT_KEY1    5             /* yaAGC interrupt numbers                 */
#define RUPT_UP      7
#define REG_INLINK   045
#define ADDR_UPBUFF  0304          /* COMPNUMB 0300 .. UPBUFF 0304-0327 (both ropes) */
#define ADDR_UPSV    01501         /* UPSVFLAG                                */
#define ADDR_SV_CSM  01554         /* RRECTCSM .. (42 words), TETCSM at +12   */
#define ADDR_SV_LEM  01626         /* RRECTLEM .. (42 words), TETLEM at +12   */
#define SV_WORDS     42
#define SV_TET       12
#define ADDR_DELVSLV 03404         /* DELVSLV (6) and TIG (2) at 03412, CSM  */
#define ADDR_MODREG  01011
#define ADDR_DSPTAB  01023
#define ADDR_RADMODE 0110          /* RADMODES = FLGWRD12 (Luminary)          */
#define RADM_LRPOS   040           /* RADMODES bit 6: LR position 2 desired   */
#define DSPTAB_NOUN  8             /* DSPTAB +8: the noun digits' relay word   */
#define NOUN_DIGITS  01777         /* its two digit codes (bits 10-6, 5-1)     */
#define NOUN36       01574         /* '3' 27, '6' 28                           */
#define CH11_OPRERR  0100          /* channel 011 bit 7: OPR ERR lamp          */
#ifdef REPLAY_YAAGC
#define ROPE_OK 5                  /* NASSP's yaAGC agc_load_binfile: 5 when loaded */
#else
#define ROPE_OK 0
#endif

#define DN_MAX    512  /* downlink word pairs kept                    */
#define DN_CYCLES 1707 /* 20 ms of memory cycles: NASSP's PCM DOWNRUPT */
#define DN_LIST   100  /* DP words of a downlist, the ID word included */
#define DN_SYNC   077340 /* LOWIDCOD, channel 35 of an ID word          */

static agc_t vagc;

/* The downlink as NASSP's PCM sees it: channel 34, then 35 (with channel 13 bit 7, the
 * word order code, at that write).                                                    */
static int      dn_on;
static long     dn_cycle;
static int      dn_n;
static unsigned dn_hi;
static unsigned dn_ch13;
static unsigned dn_word[DN_MAX][3];

/* NASSP callbacks the engines call: the downlink channels are kept. */
void ChannelOutput(agc_t *s, int ch, int v)
{
    (void) s;

#ifdef REPLAY_YAAGC
    if (ch == 07)
    {
        /* The superbank (FEB) is the host's to keep, as NASSP's ChannelOutput does
         * (apolloguidance.cpp): without it a Luminary task in banks 30-37 runs in 40-47. */
        vagc.InputChannel[07] = (int16_t) (v & 0160);
        vagc.OutputChannel7   = (int16_t) (v & 0160);
    }
#endif

    if (ch == 013)
    {
        dn_ch13 = (unsigned) v;
    }
    else if (ch == 034)
    {
        dn_hi = (unsigned) v & 077777u;
    }
    else if (ch == 035 && dn_on && dn_n < DN_MAX)
    {
        dn_word[dn_n][0] = dn_hi;
        dn_word[dn_n][1] = (unsigned) v & 077777u;
        dn_word[dn_n][2] = (dn_ch13 >> 6) & 1u;
        dn_n++;
    }
}

void RequestRadarData(agc_t *s)
{
    (void) s;
}

#ifdef REPLAY_YAAGC
int ChannelInput(agc_t *s)
{
    (void) s;

    return (0);
}

void ChannelRoutine(agc_t *s)
{
    (void) s;
}

void ShiftToDeda(agc_t *s, int d)
{
    (void) s;
    (void) d;
}

void UnblockSocket(int n)
{
    (void) n;
}

#endif

static unsigned word(int flat)
{
    return ((unsigned) (unsigned short) vagc.Erasable[flat / 0400][flat % 0400] & 077777u);
}

static void run(double s)
{
    long n = (long) (s / MCT_S);

    for (long i = 0; i < n; i++)
    {
        agc_engine(&vagc);

        if (dn_on && ++dn_cycle >= DN_CYCLES)
        {
            dn_cycle                   = 0;
            vagc.InterruptRequests[8] = 1; /* DOWNRUPT, as NASSP's PCM raises it */
        }
    }
}

/* The last complete downlist of a few seconds of downlink (an ID word, 99 words, the
 * next ID word).                                                                       */
static void downlist(void)
{
    int at = -1;

    dn_on = 1;
    run(5.0);
    dn_on = 0;

    for (int i = 0; i + DN_LIST < dn_n; i++)
    {
        if (dn_word[i][1] == DN_SYNC && dn_word[i + DN_LIST][1] == DN_SYNC && dn_word[i][2] == 0u)
        {
            at = i;
        }
    }

    if (at < 0)
    {
        printf("#downlist none (%d words)\n", dn_n);

        return;
    }

    printf("#downlist %05o\n", dn_word[at][0]);

    for (int k = 1; k < DN_LIST; k++)
    {
        const unsigned *w = dn_word[at + k];

        printf("#dn %02d %05o %05o %u\n", k, w[0], w[1], w[2]);
    }
}

static int key_code(char c)
{
    switch (c)
    {
        case 'V':
            return (021);
        case 'N':
            return (037);
        case 'E':
            return (034);
        case 'R':
            return (022);
        case 'C':
            return (036);
        case 'K':
            return (031);
        case 'P':
            return (032); /* not a DSKY key code: PRO is V33E below      */
        case '+':
            return (032);
        case '-':
            return (033);
        case '0':
            return (020);
        default:
            return ((c >= '1' && c <= '9') ? c - '0' : -1);
    }
}

static void key(char c)
{
    WriteIO(&vagc, 015, key_code(c));
    vagc.InterruptRequests[RUPT_KEY1] = 1;
    run(0.3);
}

static void keys(const char *s)
{
    for (; *s != '\0'; s++)
    {
        key(*s);
    }
}

/* MCC::pushAGCUplinkKey's two bytes for each key (mcc.cpp), as one INLINK word. */
static int uplink_word(char c)
{
    int k = key_code(c);

    return ((k < 0) ? -1 : (int) (0100000u | ((unsigned) k << 10) | ((~(unsigned) k & 037u) << 5) | (unsigned) k));
}

static void uplink(const char *s)
{
    for (; *s != '\0'; s++)
    {
        int w = uplink_word(*s);

        if (w < 0)
        {
            continue;
        }

        for (int i = 0; i < 100 && vagc.InterruptRequests[RUPT_UP] != 0; i++)
        {
            run(0.1);
        }

        vagc.Erasable[0][REG_INLINK]    = (int16_t) w;
        vagc.InterruptRequests[RUPT_UP] = 1;
        run(0.1);
    }
}

static void block(const char *tag, int base)
{
    printf("%s", tag);

    for (int i = 0; i < SV_WORDS; i++)
    {
        printf(" %05o", word(base + i));
    }

    printf("\n");
}

static void words(const char *tag, int base, int n)
{
    printf("%s", tag);

    for (int i = 0; i < n; i++)
    {
        printf(" %05o", word(base + i));
    }

    printf("\n");
}

#ifdef REPLAY_YAAGC

/* The engine state NASSP's ApolloGuidance::LoadState restores (apolloguidance.cpp,
 * AGCState bit order), so stock yaAGC continues the saved image as it does in NASSP. */
static int yaagc_state(const char *p)
{
    unsigned u;
    unsigned b = 2; /* Reset, isFirstTimestep */
    int      a;

    if (sscanf(p, "V10CHAN%d %u", &a, &u) == 2 && a >= 0 && a < 16)
    {
        vagc.OutputChannel10[a] = (int16_t) u;
    }
    else if (sscanf(p, "VINT%d %u", &a, &u) == 2 && a >= 0 && a <= NUM_INTERRUPT_TYPES)
    {
        vagc.InterruptRequests[a] = (int8_t) u;
    }
    else if (sscanf(p, "VOC7 %u", &u) == 1)
    {
        vagc.OutputChannel7 = (int16_t) u;
    }
    else if (sscanf(p, "IDXV %u", &u) == 1)
    {
        vagc.IndexValue = (int16_t) u;
    }
    else if (sscanf(p, "NEXTZ %u", &u) == 1)
    {
        vagc.NextZ = (int) u;
    }
    else if (sscanf(p, "SCALERCOUNTER %u", &u) == 1)
    {
        vagc.ScalerCounter = (int) u;
    }
    else if (sscanf(p, "CRCOUNT %u", &u) == 1)
    {
        vagc.ChannelRoutineCount = (int) u;
    }
    else if (sscanf(p, "STATE2 %u", &u) == 1)
    {
        vagc.RadarGateCounter = (int) (u & 017u);
    }
    else if (sscanf(p, "STATE %u", &u) == 1)
    {
        vagc.ExtraCode             = (u >> b++) & 1u;
        vagc.AllowInterrupt        = (u >> b++) & 1u;
        vagc.InIsr                 = (u >> b++) & 1u;
        vagc.SubstituteInstruction = (u >> b++) & 1u;
        vagc.PendFlag              = (u >> b++) & 1u;
        vagc.PendDelay             = (u >> b) & 7u;
        b                          += 3;
        vagc.ExtraDelay            = (u >> b) & 7u;
        b                          += 3;
        vagc.DownruptTimeValid     = (u >> b++) & 1u;
        b++; /* PadLoaded */
        vagc.NightWatchman         = (u >> b++) & 1u;
        vagc.RuptLock              = (u >> b++) & 1u;
        vagc.NoRupt                = (u >> b++) & 1u;
        vagc.TCTrap                = (u >> b++) & 1u;
        vagc.NoTC                  = (u >> b++) & 1u;
        vagc.Standby               = (u >> b++) & 1u;
        vagc.SbyPressed            = (u >> b++) & 1u;
        vagc.SbyStillPressed       = (u >> b++) & 1u;
        vagc.ParityFail            = (u >> b++) & 1u;
        b++; /* Spare1 */
        vagc.NightWatchmanTripped  = (u >> b++) & 1u;
        vagc.GeneratedWarning      = (u >> b++) & 1u;
        vagc.TookBZF               = (u >> b++) & 1u;
        vagc.TookBZMF              = (u >> b++) & 1u;
        vagc.Trap31A               = (u >> b++) & 1u;
        vagc.Trap31B               = (u >> b++) & 1u;
        vagc.Trap32                = (u >> b) & 1u;
    }
    else
    {
        return (0);
    }

    return (1);
}

#endif

static void load(const char *path)
{
    FILE *f  = fopen(path, "r");
    char  line[512];
    int   in = 0;

    if (f == NULL)
    {
        perror(path);
        exit(2);
    }

    while (fgets(line, sizeof line, f) != NULL)
    {
        char    *p = line;
        unsigned a;
        unsigned v;
        int      c;
        unsigned u;

        while (*p == ' ' || *p == '\t')
        {
            p++;
        }

        if (strncmp(p, "AGC_BEGIN", 9) == 0)
        {
            in = 1;
        }
        else if (!in)
        {
        }
        else if (strncmp(p, "AGC_END", 7) == 0)
        {
            break;
        }
        else if (sscanf(p, "EMEM%o %o", &a, &v) == 2 && a < 04000u)
        {
            vagc.Erasable[a / 0400u][a % 0400u] = (int16_t) v;
        }
        else if (sscanf(p, "VICHAN%d %u", &c, &u) == 2 && c >= 0 && c < NUM_CHANNELS)
        {
            vagc.InputChannel[c] = (int16_t) u;
        }

#ifdef REPLAY_YAAGC
        else if (yaagc_state(p))
        {
        }
#endif
        else
        {
            /* engine state our shim has no use for: it restarts (GOJAM) */
        }
    }

    (void) fclose(f);
}

int main(int argc, char **argv)
{
    if (argc != 5 && !(argc == 6 && strcmp(argv[5], "gojam") == 0))
    {
        fprintf(stderr, "usage: replay IMAGE ROPE SECONDS UPLINK [gojam]\n");

        return (2);
    }

    memset(&vagc, 0, sizeof vagc);
    agc_engine_init(&vagc, NULL, NULL, 0);

    if (agc_load_binfile(&vagc, argv[2]) != ROPE_OK)
    {
        fprintf(stderr, "replay: cannot load rope %s\n", argv[2]);

        return (2);
    }

    load(argv[1]);

#ifdef REPLAY_YAAGC
    if (argc == 6)
    {
        vagc.ParityFail = 1; /* the next cycle takes GOJAM (agc_engine.c: TriggeredAlarm || ParityFail) */
    }
#endif

    unsigned lrpos = word(ADDR_RADMODE) & RADM_LRPOS; /* as the scenario saved it */

    run(5.0);

    /* A load is no keystroke: the image's channel 015 still holds the last key code, and
     * NASSP's yaAGC takes keys only from its interrupt requests ('#': the driver checks the
     * OPR ERR lamp, channel 011 bit 7, 5 s after the load on ours). */
    printf("# oprerr-at-load %o\n", (unsigned) ReadIO(&vagc, 011) & CH11_OPRERR);

    /* LGC: the LR position the AGC wants (RADMODES bit 6). NASSP resumes the saved
     * image; our shim restarts it, and STARTSUB re-reads the bit from channel 33, where
     * an LR that is off reports no position: the shim must keep the saved bit. A '#'
     * line: the driver checks it in our transcript (stock resumes the image and does
     * not restart, so it is no reference for the restart's bit). */
    if (strstr(argv[2], "Luminary") != NULL)
    {
        printf("# radmodes-lrpos image %o after-load %o\n", lrpos, word(ADDR_RADMODE) & RADM_LRPOS);
        run(35.0);
        printf("# noun40 %o\n", word(ADDR_DSPTAB + DSPTAB_NOUN)); /* the noun relay word 40 s after the load */
        downlist();                                                  /* the list of the resumed program */
    }

    keys("R");
    keys("V37E00E");

    unsigned tc    = word(ADDR_SV_CSM + SV_TET) << 14 | word(ADDR_SV_CSM + SV_TET + 1);
    unsigned tl    = word(ADDR_SV_LEM + SV_TET) << 14 | word(ADDR_SV_LEM + SV_TET + 1);
    int      n     = 0;
    double   total = atof(argv[3]);

    printf("start TETCSM=%o,%o TETLEM=%o,%o\n", word(ADDR_SV_CSM + SV_TET), word(ADDR_SV_CSM + SV_TET + 1), word(ADDR_SV_LEM + SV_TET), word(ADDR_SV_LEM + SV_TET + 1));

    for (double t = 0.0; t < total; t += 1.0)
    {
        run(1.0);

        unsigned c = word(ADDR_SV_CSM + SV_TET) << 14 | word(ADDR_SV_CSM + SV_TET + 1);
        unsigned l = word(ADDR_SV_LEM + SV_TET) << 14 | word(ADDR_SV_LEM + SV_TET + 1);

        if (c != tc)
        {
            n++;
            printf("# csm step after %.0f s of P00\n", t + 1.0);
            printf("step csm TETCSM=%o,%o\n", c >> 14, c & 037777u);
            block("  csm", ADDR_SV_CSM);
            tc = c;
        }

        if (l != tl)
        {
            n++;
            printf("# lem step after %.0f s of P00\n", t + 1.0);
            printf("step lem TETLEM=%o,%o\n", l >> 14, l & 037777u);
            block("  lem", ADDR_SV_LEM);
            tl = l;
        }
    }

    printf("steps %d\n", n);

    if (strcmp(argv[4], "-") != 0)
    {
        uplink(argv[4]);
        run(10.0);
        printf("uplink %s\n", argv[4]);
        words("  upbuff", ADDR_UPBUFF, 024);
        words("  upsvflag", ADDR_UPSV, 1);
        block("  csm", ADDR_SV_CSM);
        block("  lem", ADDR_SV_LEM);
        words("  delvslv-tig", ADDR_DELVSLV, 10);
        words("  modreg", ADDR_MODREG, 1);
        words("  dsptab", ADDR_DSPTAB, 11);
    }

    /* How soon the erasable NASSP sees (the Lua hook, MFDs) shows a display: V06N3,
     * then the time from the '6' key press until DSPTAB's noun word reads 36. NASSP's
     * yaAGC works in that memory itself; the shim mirrors it ('#': the driver checks
     * ours against DSKY_LATENCY_S). */
    keys("V06N3");
    WriteIO(&vagc, 015, key_code('6'));
    vagc.InterruptRequests[RUPT_KEY1] = 1;

    double lat = 0.0; /* s from the key press */

    while (lat < 3.0 && (word(ADDR_DSPTAB + DSPTAB_NOUN) & NOUN_DIGITS) != NOUN36)
    {
        run(0.01);
        lat += 0.01;
    }

    printf("# dsky-latency %.2f\n", lat);

    if (strstr(argv[2], "Luminary") == NULL)
    {
        downlist(); /* after P00 and the uplink (the LGC's was sent above) */
    }

    return (0);
}
