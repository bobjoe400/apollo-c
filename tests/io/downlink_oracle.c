/**********************************************************************
 * FILE NAME: downlink_oracle.c
 *
 * PURPOSE:
 *   Every downlist of both ropes word for word against yaAGC running the
 *   rope's own DODOWNTM (tools/oracle/cases/downlink_{csm,lm}.{spec,txt},
 *   gen/gen_downlink.py): one known erasable state (a word in every
 *   address the lists name) is loaded into the C through the erasable
 *   map, the channels the lists send are set as yaAGC had them, and 101
 *   DOWNRUPTs (ID word, 99 data words, the next ID) must give the same
 *   channel 34, channel 35 and word order code. The one word left out is
 *   TIME1 (TIME2,TIME1 low half): yaAGC's clock runs 10 ms a count while
 *   the list is sent, the unit test's does not.
 *
 * AGC SOURCE:
 *   Comanche055 DOWNLINK_LISTS.agc pp.170-180, DOWN-TELEMETRY_PROGRAM.agc
 *   pp.1093-1102; Luminary099 DOWNLINK_LISTS.agc pp.193-205,
 *   DOWN_TELEMETRY_PROGRAM.agc pp.988-997.
 *
 * EXTERNAL VARIABLES:
 *   none (test counters are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
 *   agc_erase_write_ecadr  the known state, by address
 *   agc_telem_*            the down-telemetry program under test
 *   agc_chan_*             channels 11-14, 30-33 in; 13, 34, 35 out
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails. SKIP (exit 0) when the case files
 *   are not found (run from the repository root).
 **********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/erasable_map/address_map.h"
#include "agc/io/service.h"

#define DL_WORDS 101   /* DP words per case: ID, 99 data words, next ID */
#define DL_LINE  128   /* longest dump line                             */
#define DL_BIT7  0100u /* channel 13 bit 7: the word order code */

static int g_checks;   /* checks run    */
static int g_failures; /* checks failed */

/* Prototypes: main() comes first (STYLE 4.6). */
static void check(bool ok, int line, const char *text);
static int  run_file(const char *path, agc_vehicle_t v);
static bool load_line(const char *line, int *code);
static void compare_case(FILE *f, const char *first, const char *name, int code);

#define CHECK(c) check((c), __LINE__, #c)

/**********************************************************************
 * AGC: none (test driver) -- every list of both ropes
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    int cases = run_file("tools/oracle/cases/downlink_csm.txt", AGC_VEHICLE_CSM); /* CSM lists compared */

    cases += run_file("tools/oracle/cases/downlink_lm.txt", AGC_VEHICLE_LM);

    if (cases == 0)
    {
        (void) printf("test_downlink: SKIP (tools/oracle/cases/downlink_*.txt not found)\n");
    }
    else
    {
        CHECK(cases == AGC_DNLIST_CSM_COUNT + AGC_DNLIST_LM_COUNT);
    }

    (void) printf("test_downlink: %d checks, %d failures\n", g_checks, g_failures);

    return ((g_failures == 0) ? 0 : 1); /* exit 1 on any failure */
}

/**********************************************************************
 * AGC: none -- a failed assertion inside the library is a test failure
 *
 * ARGUMENTS:
 *   file  const char *  I   source file of the assertion
 *   line  int           I   its line
 *   expr  const char *  I   its text
 *
 * RETURNS: false (the assertion's value)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("ASSERT %s:%d %s\n", file, line, expr);
    g_failures++;

    return (false);
}

/**********************************************************************
 * AGC: none (test helper) -- count a check, print a failure
 *
 * ARGUMENTS:
 *   ok    bool          I   the check's result
 *   line  int           I   test source line
 *   text  const char *  I   the checked expression
 **********************************************************************/
static void check(bool ok, int line, const char *text)
{
    g_checks++;

    if (!ok)
    {
        g_failures++;
        (void) printf("FAIL line %d: %s\n", line, text);
    }
}

/**********************************************************************
 * AGC: none -- every case of one dump file
 *
 * ARGUMENTS:
 *   path  const char *   I   the dump (downlink_csm.txt / _lm.txt)
 *   v     agc_vehicle_t  I   its rope
 *
 * RETURNS: int -- cases compared (0 when the file is not found)
 **********************************************************************/
static int run_file(const char *path, agc_vehicle_t v)
{
    FILE *f             = fopen(path, "r"); /* the dump */
    char  line[DL_LINE];                    /* a line   */
    char  name[DL_LINE] = "";               /* CASE name */
    int   cases         = 0;                /* compared */
    int   code          = -1;               /* DNLSTCOD of the case */

    while (f != NULL && fgets(line, sizeof (line), f) != NULL)
    {
        if (strncmp(line, "CASE ", 5) == 0)
        {
            agc_state_init(v);
            agc_telem_fresh_start();
            (void) snprintf(name, sizeof (name), "%s", line + 5);
            name[strcspn(name, "\n")] = '\0';
            code                      = -1;
        }
        else if (!load_line(line, &code))
        {
            compare_case(f, line, name, code);
            cases++;
        }
        else
        {
            /* LOAD / CHAN / header line taken */
        }
    }

    if (f != NULL)
    {
        (void) fclose(f);
    }

    return (cases);
} /* run_file */

/**********************************************************************
 * AGC: none -- take one header line of a case: LOAD (a word by ECADR;
 * DNLSTCOD's 0332 gives the list code), CHAN (a channel as yaAGC had
 * it), or the other header lines
 *
 * ARGUMENTS:
 *   line  const char *  I   the line
 *   code  int *         I/O DNLSTCOD of the case
 *
 * RETURNS: bool -- false at the first DNLINK line (the words follow)
 **********************************************************************/
static bool load_line(const char *line, int *code)
{
    unsigned int a = 0; /* ECADR or channel */
    unsigned int w = 0; /* word */

    if (sscanf(line, "LOAD %o %o", &a, &w) == 2)
    {
        CHECK(agc_erase_write_ecadr((uint16_t) a, (uint16_t) w) == AGC_STATUS_OK);
        *code = (a == 0332u) ? (int) w : *code; /* DNLSTCOD (unswitched 0332, both ropes) */
    }
    else if (sscanf(line, "CHAN %o %o", &a, &w) == 2)
    {
        agc_chan_write((int) a, (uint16_t) w);
    }

    return (strncmp(line, "DNLINK ", 7) != 0);
}

/**********************************************************************
 * AGC: none -- run the list and compare it word for word
 *
 * ARGUMENTS:
 *   f      FILE *        I   the dump, at word 1
 *   first  const char *  I   word 0's DNLINK line (the ID word)
 *   name   const char *  I   the case
 *   code   int           I   DNLSTCOD
 **********************************************************************/
static void compare_case(FILE *f, const char *first, const char *name, int code)
{
    int  equal   = 0;   /* words equal to yaAGC's */
    int  nonzero = 0;   /* data words the C sends non-zero */
    char line[DL_LINE]; /* a DNLINK line */

    CHECK(code >= 0);
    agc_telem_select_list(code);
    agc_telem_restart();
    (void) snprintf(line, sizeof (line), "%s", first);

    for (int i = 0; i < DL_WORDS; i++)
    {
        unsigned int n  = 0; /* word index         */
        unsigned int a  = 0; /* yaAGC's channel 34 */
        unsigned int l  = 0; /* yaAGC's channel 35 */
        unsigned int wo = 0; /* yaAGC's word order */

        if (i > 0 && fgets(line, sizeof (line), f) == NULL)
        {
            break;
        }

        CHECK(sscanf(line, "DNLINK %u %o %o %u", &n, &a, &l, &wo) == 4);
        CHECK(agc_telem_downrupt() == AGC_STATUS_OK);

        uint16_t     ca = agc_chan_read(AGC_CHAN_DNTM1);                                /* our channel 34 */
        uint16_t     cl = agc_chan_read(AGC_CHAN_DNTM2);                                /* our channel 35 */
        unsigned int cw = ((agc_chan_read(AGC_CHAN_CHAN13) & DL_BIT7) != 0u) ? 1u : 0u; /* our word order */
        bool         t1 = (i > 0 && i < DL_WORDS - 1 && wo == 0u);                      /* TIME2,TIME1: TIME1 not compared */

        if (i > 0 && i < DL_WORDS - 1 && (ca != 0u || cl != 0u))
        {
            nonzero++; /* a data word with a non-zero half */
        }

        if (ca == a && (cl == l || t1) && cw == wo)
        {
            equal++;
        }

        if (i > 0 && (ca != a || (cl != l && !t1) || cw != wo))
        {
            (void) printf("  %s word %u: yaAGC %05o %05o %u, C %05o %05o %u\n", name, n, a, l, wo, ca, cl, cw);
        }
    }

    (void) printf("test_downlink: %s: %d/%d words equal yaAGC, %d data words non-zero\n", name, equal, DL_WORDS, nonzero);
    CHECK(equal == DL_WORDS);
} /* compare_case */
