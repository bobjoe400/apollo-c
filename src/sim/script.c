/**********************************************************************
 * FILE NAME: script.c
 *
 * PURPOSE:
 *   Timed inputs for runs without a peripheral (--script FILE): each
 *   event is applied as the packet a peripheral would send.
 *
 * AGC SOURCE:
 *   none (harness input; packet semantics of yaAGC ChannelInput, see
 *   channels.c and counters.c)
 *
 * EXTERNAL VARIABLES:
 *   none (the open file and the next event are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  -----------------------------------------
 *   sim_input              an input-channel event (channels.c)
 *   sim_counter_increment  a counter event (counters.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   sim_script_open returns AGC_STATUS_NOT_FOUND when the file cannot
 *   be opened or holds no event. A malformed line ends the script with a
 *   diagnostic on stderr (the line number); a counter increment the
 *   harness refuses ends that line's repeats with a diagnostic.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Events are in non-decreasing time order. Inputs that request an
 *   interrupt are taken one per centisecond, so two key presses must be
 *   at least 1 cs apart.
 *
 * NOTES:
 *   One event per line, `#` starts a comment:
 *       <cs> <channel> <value> [<repeat>]
 *   cs decimal (centiseconds since power-on), channel and value octal,
 *   exactly as a packet would carry them: channel 0200 | addr is a
 *   counter increment of type <value>, repeated <repeat> times (e.g.
 *   `100 0237 0 500` = 500 PINC of PIPAX at 1 s); 015 is a main DSKY
 *   key (KEYRUPT1); 032 bit 14 = 0 is PRO pressed; 0173 an uplink word.
 *
 * ALGORITHM:
 *   none (read ahead one event; apply the events that are due)
 **********************************************************************/
#include <stdio.h>
#include <stdlib.h>

#include "internal.h"

#define SCRIPT_LINE      256      /* longest line read                    */
#define SCRIPT_REPEAT    077777UL /* bound on one line's repeat count     */
#define SCRIPT_PER_CS    4096     /* bound on lines read / applied per cs */
#define SCRIPT_CHAN_MAX  0377UL   /* 8-bit channel numbers                */
#define SCRIPT_ADDR_MASK 0177u    /* counter address bits of 0200 | addr  */

typedef struct script_event
{
    bool              valid;  /* an event is waiting                  */
    unsigned long int cs;     /* when, centiseconds since power-on    */
    unsigned long int chan;   /* channel, 0..0377                     */
    unsigned long int value;  /* 15-bit word or increment type        */
    unsigned long int repeat; /* counter increments, 1..077777        */
} script_event_t;

static FILE             *g_file; /* the script, NULL when closed */
static script_event_t    g_next; /* the next event               */
static unsigned long int g_line; /* line number read last        */

/**********************************************************************
 * AGC: none -- read the next event, skipping blank and comment lines.
 *
 * ARGUMENTS: none
 *
 * RETURNS: bool -- true when g_next holds an event; false at end of
 *          file or on a malformed line (reported)
 **********************************************************************/
static bool script_read(void)
{
    char buf[SCRIPT_LINE]; /* one line */
    bool done = false;     /* an event was found, or the script ended */

    g_next.valid = false;

    for (int i = 0; i < SCRIPT_PER_CS && g_file != NULL && !done; i++)
    {
        if (fgets(buf, (int) sizeof (buf), g_file) == NULL)
        {
            done = true;
        }
        else
        {
            g_line++;
            g_next.repeat = 1;
            int n         = sscanf(buf, "%lu %lo %lo %lu", &g_next.cs, &g_next.chan, &g_next.value, &g_next.repeat); /* fields read */

            done         = (n >= 3) || (n >= 1 && buf[0] != '#');
            g_next.valid = n >= 3 && g_next.chan <= SCRIPT_CHAN_MAX && g_next.value <= SIM_WORD_MASK && g_next.repeat >= 1UL && g_next.repeat <= SCRIPT_REPEAT;

            if (done && !g_next.valid)
            {
                (void) fprintf(stderr, "agc_sim: script line %lu not understood: script ends\n", g_line); /* stdlib diagnostic */
            }
        }
    }

    return (g_next.valid);
}

/**********************************************************************
 * AGC: none -- open the script and read its first event.
 *
 * ARGUMENTS:
 *   path  const char *  I  script file
 *
 * RETURNS: agc_status_t -- OK, or NOT_FOUND when the file cannot be
 *          opened or holds no event
 **********************************************************************/
agc_status_t sim_script_open(const char *path)
{
    g_line = 0;
    g_file = fopen(path, "r");

    return ((g_file != NULL && script_read()) ? AGC_STATUS_OK : AGC_STATUS_NOT_FOUND); /* no file or no event */
}

/**********************************************************************
 * AGC: none -- apply one event as its packet.
 *
 * ARGUMENTS:
 *   e  const script_event_t *  I  the event, valid
 *
 * RETURNS: bool -- true when it requested an interrupt
 **********************************************************************/
static bool script_apply(const script_event_t *e)
{
    uint16_t chan  = (uint16_t) e->chan;  /* channel */
    uint16_t value = (uint16_t) e->value; /* word or increment type */
    bool     rupt  = false;               /* an interrupt was requested */

    if ((chan & SIM_CH_COUNTER) == 0u)
    {
        rupt = sim_input(chan, value, SIM_WORD_MASK);
    }
    else
    {
        agc_status_t st = AGC_STATUS_OK; /* last increment's outcome */

        for (unsigned long int k = 0; k < e->repeat && st == AGC_STATUS_OK; k++)
        {
            st = sim_counter_increment((uint16_t) (chan & SCRIPT_ADDR_MASK), value);
        }

        if (st != AGC_STATUS_OK)
        {
            (void) fprintf(stderr, "agc_sim: script: counter %03o type %o refused (status %d)\n", (unsigned int) (chan & SCRIPT_ADDR_MASK), (unsigned int) value, (int) st); /* stdlib diagnostic */
        }
    }

    return (rupt);
}

/**********************************************************************
 * AGC: none -- apply the events due at cs, stopping after one that
 * requested an interrupt.
 *
 * ARGUMENTS:
 *   cs  uint32_t  I  centiseconds since power-on
 **********************************************************************/
void sim_script_tick(uint32_t cs)
{
    bool more = true; /* keep applying */

    for (int i = 0; i < SCRIPT_PER_CS && more && g_next.valid && g_next.cs <= (unsigned long int) cs; i++)
    {
        script_event_t e    = g_next;           /* the event due */
        bool           rupt = script_apply(&e); /* it requested an interrupt */

        more = script_read() && !rupt;
    }
}

/**********************************************************************
 * AGC: none -- close the script.
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_script_close(void)
{
    if (g_file != NULL)
    {
        (void) fclose(g_file); /* stdlib: read-only file, nothing to lose */
    }

    g_file       = NULL;
    g_next.valid = false;
}
