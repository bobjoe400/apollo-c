/*
 * script.c -- timed inputs for runs without a peripheral (--script FILE).
 *
 * One event per line, `#` starts a comment:
 *     <cs> <channel> <value> [<repeat>]
 * cs decimal (centiseconds since power-on, non-decreasing), channel and
 * value octal, exactly as a peripheral packet would carry them: channel
 * 0200 | addr is a counter increment of type <value>, repeated <repeat>
 * times (e.g. `100 0237 0 500` = 500 PINC of PIPAX at 1 s); 015 is a main
 * DSKY key (KEYRUPT1); 032 bit 14 = 0 is PRO pressed; 0173 an uplink word.
 * Inputs that request an interrupt are taken one per centisecond, so two
 * key presses must be at least 1 cs apart.
 */
#include <stdio.h>
#include <stdlib.h>

#include "agc/core/assert.h"
#include "internal.h"

#define SCRIPT_LINE   256
#define SCRIPT_REPEAT 077777ul /* bound on one line's repeat count     */
#define SCRIPT_PER_CS 4096     /* bound on lines applied per cs         */

typedef struct script_event
{
    bool          valid;
    unsigned long cs;
    unsigned long chan;
    unsigned long value;
    unsigned long repeat;
} script_event_t;

static FILE          *g_file;
static script_event_t g_next;
static unsigned long  g_line;

/* Read the next event; false at end of file or on a malformed line.    */
static bool script_read(void)
{
    char buf[SCRIPT_LINE];

    g_next.valid = false;

    for (int i = 0; i < SCRIPT_PER_CS && g_file != NULL; i++)
    {
        if (fgets(buf, (int) sizeof (buf), g_file) == NULL)
        {
            return (false);
        }

        g_line++;
        g_next.repeat = 1;
        int n = sscanf(buf, "%lu %lo %lo %lu", &g_next.cs, &g_next.chan,
                       &g_next.value, &g_next.repeat);

        if (n >= 3)
        {
            g_next.valid = g_next.chan <= 0377ul &&
                           g_next.value <= SIM_WORD_MASK &&
                           g_next.repeat <= SCRIPT_REPEAT;

            if (!g_next.valid)
            {
                (void) fprintf(stderr, "agc_sim: script line %lu ignored\n",
                               g_line); /* diagnostic only */
            }

            return (g_next.valid);
        }
    }

    return (false);
}

agc_status_t sim_script_open(const char *path)
{
    if (!AGC_ASSERT(path != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    g_line = 0;
    g_file = fopen(path, "r");

    if (g_file == NULL)
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    bool any = script_read();

    if (!AGC_ASSERT(g_file != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (any ? AGC_STATUS_OK : AGC_STATUS_NOT_FOUND);
}

/* Apply one event; true when it requested an interrupt.                 */
static bool script_apply(const script_event_t *e)
{
    if (!AGC_ASSERT(e != NULL) || !AGC_ASSERT(e->valid))
    {
        return (false);
    }

    uint16_t chan  = (uint16_t) e->chan;
    uint16_t value = (uint16_t) e->value;

    if ((chan & SIM_CH_COUNTER) == 0u)
    {
        return (sim_input(chan, value, SIM_WORD_MASK));
    }

    for (unsigned long k = 0; k < e->repeat && k <= SCRIPT_REPEAT; k++)
    {
        if (sim_counter_increment((uint16_t) (chan & 0177u), value) !=
            AGC_STATUS_OK)
        {
            return (false);
        }
    }

    return (false);
}

void sim_script_tick(uint32_t cs)
{
    for (int i = 0; i < SCRIPT_PER_CS && g_next.valid; i++)
    {
        if (g_next.cs > (unsigned long) cs)
        {
            return;
        }

        script_event_t e = g_next;

        bool rupt = script_apply(&e);

        if (!script_read() || rupt)
        {
            return; /* end of script, or one interrupt input per cs */
        }
    }

    if (!AGC_ASSERT(g_line < 0x7FFFFFFFul))
    {
        return;
    }
}

void sim_script_close(void)
{
    if (g_file != NULL && fclose(g_file) != 0)
    {
        g_file = NULL;
    }

    g_file       = NULL;
    g_next.valid = false;

    if (!AGC_ASSERT(g_file == NULL))
    {
        return;
    }
}
