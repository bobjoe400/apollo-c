/**********************************************************************
 * FILE NAME: downlink_dump_oracle.c
 *
 * PURPOSE:
 *   The downlink erasable dump (DNDUMP, V74) of both ropes pair by pair
 *   against yaAGC running the rope's own DODOWNTM
 *   (tools/oracle/cases/dndump_{csm,lm}.{spec,txt}, gen/gen_dump.py): a
 *   known word in every erasable address is loaded into the C through
 *   the erasable map, DUMPCNT = BIT12 (one dump) by address, and the
 *   dump (per EBANK the ERASID word, DUMPLOC and TIME1, 128 pairs; then
 *   the downlist's ID) must give the same channel 34, channel 35 and word
 *   order code. Left out: the central and counter registers (0-060: the
 *   C has no A, L, Q, BBANK ... words), the run's own words (the main
 *   program at 0450, DNLSTCOD .. DNTMBUFF +21D), TIME1, and an address
 *   whose word the C keeps in another form (a -0, an enum, a DP whose
 *   words disagree in sign: read back unequal after the load, counted).
 *
 * AGC SOURCE:
 *   Comanche055 DOWN-TELEMETRY_PROGRAM.agc pp.1093-1102 (DNDUMP);
 *   Luminary099 DOWN_TELEMETRY_PROGRAM.agc pp.988-997.
 *
 * EXTERNAL VARIABLES:
 *   none (test counters are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
 *   agc_erase_write_ecadr  the known state, by address
 *   agc_telem_*            the down-telemetry program under test
 *   agc_chan_*             channels in; 13, 34, 35 out
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails. SKIP (exit 0) when the case files
 *   are not found (run from the repository root).
 **********************************************************************/
#include <stdio.h>
#include <string.h>

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/erasable_map/address_map.h"
#include "agc/io/service.h"

#define DD_ERASABLE   04000 /* erasable words, E0-E7                    */
#define DD_REGS       061   /* registers and counters: not compared     */
#define DD_LOOP       0450  /* the probe's main program (TC 0450)       */
#define DD_WORK_LO    0330  /* DNLSTCOD .. DNTMBUFF +21D: the run's own */
#define DD_WORK_HI    0366  /* the first address after them            */
#define DD_BANK       130   /* word pairs per EBANK: ID, DUMPLOC, 128    */
#define DD_PAIRS      1041  /* 8 EBANKs and the next downlist ID         */
#define DD_LINE       128   /* longest dump line                         */
#define DD_MIN_HALVES 1900  /* of 2082: all but the left-out words      */
#define DD_BIT7       0100u /* channel 13 bit 7: the word order code     */

static int      g_checks;            /* checks run                     */
static int      g_failures;          /* checks failed                  */
static bool     g_loading;           /* the known state is being loaded */
static int      g_refused;           /* assertions while loading       */
static uint16_t g_word[DD_ERASABLE]; /* the loaded words               */
static bool     g_same[DD_ERASABLE]; /* loaded and read back unchanged */

/* Prototypes: main() comes first (STYLE 4.6). */
static void check(bool ok, int line, const char *text);
static int  run_file(const char *path, agc_vehicle_t v);
static bool load_line(const char *line);
static bool compared(int a);
static void compare_dump(FILE *f, const char *first, const char *name);

#define CHECK(c) check((c), __LINE__, #c)

/**********************************************************************
 * AGC: none (test driver) -- the dump of both ropes
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    int cases = run_file("tools/oracle/cases/dndump_csm.txt", AGC_VEHICLE_CSM); /* dumps compared */

    cases += run_file("tools/oracle/cases/dndump_lm.txt", AGC_VEHICLE_LM);

    if (cases == 0)
    {
        (void) printf("test_dndump: SKIP (tools/oracle/cases/dndump_*.txt not found)\n");
    }
    else
    {
        CHECK(cases == 2);
    }

    (void) printf("test_dndump: %d checks, %d failures\n", g_checks, g_failures);

    return ((g_failures == 0) ? 0 : 1); /* exit 1 on any failure */
}

/**********************************************************************
 * AGC: none -- a library assertion: while the known state is loaded it
 * is a refused store (counted; the address is then left out), otherwise
 * a test failure
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
    if (g_loading)
    {
        g_refused++;
    }
    else
    {
        (void) printf("ASSERT %s:%d %s\n", file, line, expr);
        g_failures++;
    }

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
 * AGC: none -- the dump case of one file
 *
 * ARGUMENTS:
 *   path  const char *   I   the case (dndump_csm.txt / _lm.txt)
 *   v     agc_vehicle_t  I   its rope
 *
 * RETURNS: int -- cases compared (0 when the file is not found)
 **********************************************************************/
static int run_file(const char *path, agc_vehicle_t v)
{
    FILE *f             = fopen(path, "r"); /* the case  */
    char  line[DD_LINE];                    /* a line    */
    char  name[DD_LINE] = "";               /* CASE name */
    int   cases         = 0;                /* compared  */

    while (f != NULL && fgets(line, sizeof (line), f) != NULL)
    {
        if (strncmp(line, "CASE ", 5) == 0)
        {
            agc_state_init(v);
            agc_telem_fresh_start();
            (void) memset(g_same, 0, sizeof (g_same));
            g_refused                 = 0;
            g_loading                 = true;
            (void) snprintf(name, sizeof (name), "%s", line + 5);
            name[strcspn(name, "\n")] = '\0';
        }
        else if (!load_line(line))
        {
            g_loading = false;
            compare_dump(f, line, name);
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
 * AGC: none -- one header line: LOAD (a word by ECADR, kept with
 * whether the C reads it back unchanged), CHAN, or the other lines
 *
 * ARGUMENTS:
 *   line  const char *  I   the line
 *
 * RETURNS: bool -- false at the first DNLINK line (the pairs follow)
 **********************************************************************/
static bool load_line(const char *line)
{
    unsigned int a = 0; /* ECADR or channel */
    unsigned int w = 0; /* word */

    if (sscanf(line, "LOAD %o %o", &a, &w) == 2 && a < DD_ERASABLE)
    {
        int      before = g_refused;                                                          /* assertions so far */
        bool     ok     = agc_erase_write_ecadr((uint16_t) a, (uint16_t) w) == AGC_STATUS_OK; /* stored */
        uint16_t r      = 0;                                                                  /* read back */

        agc_erase_read_ecadr((uint16_t) a, &r);
        g_word[a] = (uint16_t) w;
        g_same[a] = ok && g_refused == before && r == (uint16_t) w;
    }
    else if (sscanf(line, "CHAN %o %o", &a, &w) == 2)
    {
        agc_chan_write((int) a, (uint16_t) w);
    }
    else
    {
        /* CASE / ROPE / ENTRY / DNLINK */
    }

    return (strncmp(line, "DNLINK ", 7) != 0);
}

/**********************************************************************
 * AGC: none -- whether the word at an address is compared: a loaded
 * word the C reads back unchanged, not a register, not the run's own
 *
 * ARGUMENTS:
 *   a  int  I   erasable address 0-3777
 *
 * RETURNS: bool -- compared
 **********************************************************************/
static bool compared(int a)
{
    bool work = (a >= DD_WORK_LO && a < DD_WORK_HI) || a == DD_LOOP; /* the run's own words */

    return (a >= DD_REGS && !work && g_same[a]);
}

/**********************************************************************
 * AGC: none -- run the dump and compare it pair by pair
 *
 * ARGUMENTS:
 *   f      FILE *        I   the case, at pair 1
 *   first  const char *  I   pair 0's DNLINK line (ERASID)
 *   name   const char *  I   the case
 **********************************************************************/
static void compare_dump(FILE *f, const char *first, const char *name)
{
    int  halves = 0;    /* halves compared  */
    int  equal  = 0;    /* of them equal    */
    int  other  = 0;    /* loaded words the C keeps in another form */
    char line[DD_LINE]; /* a DNLINK line    */

    for (int a = 0; a < DD_ERASABLE; a++)
    {
        uint16_t r = 0; /* the word after the whole load */

        agc_erase_read_ecadr((uint16_t) a, &r);
        g_same[a] = g_same[a] && r == g_word[a]; /* a later store to an overlay or pad-load neighbour re-forms a DP */

        if (a >= DD_REGS && !g_same[a])
        {
            other++;
        }
    }

    agc_telem_restart();
    agc_telem_start_dump(); /* DNTMGOTO = DNDUMPI */
    (void) snprintf(line, sizeof (line), "%s", first);

    for (int n = 0; n < DD_PAIRS; n++)
    {
        unsigned int k  = 0; /* pair index         */
        unsigned int a  = 0; /* yaAGC's channel 34 */
        unsigned int l  = 0; /* yaAGC's channel 35 */
        unsigned int wo = 0; /* yaAGC's word order */

        if (n > 0 && fgets(line, sizeof (line), f) == NULL)
        {
            break;
        }

        CHECK(sscanf(line, "DNLINK %u %o %o %u", &k, &a, &l, &wo) == 4);
        CHECK(agc_telem_downrupt() == AGC_STATUS_OK);

        uint16_t     ca = agc_chan_read(AGC_CHAN_DNTM1);                                /* our channel 34 */
        uint16_t     cl = agc_chan_read(AGC_CHAN_DNTM2);                                /* our channel 35 */
        unsigned int cw = ((agc_chan_read(AGC_CHAN_CHAN13) & DD_BIT7) != 0u) ? 1u : 0u; /* our word order */
        int          r  = n % DD_BANK;                                                  /* place in the EBANK */
        int          ad = (n / DD_BANK) * 0400 + 2 * (r - 2);                           /* the pair's first address */
        bool         c0 = n == DD_PAIRS - 1 || r < 2 || compared(ad);                   /* ch 34 compared */
        bool         c1 = n == DD_PAIRS - 1 || r == 0 || (r >= 2 && compared(ad + 1));  /* ch 35 compared (not TIME1) */
        bool         e0 = c0 && ca == a;                                                /* ch 34 equal */
        bool         e1 = c1 && cl == l;                                                /* ch 35 equal */

        CHECK(cw == wo);
        halves += (int) c0 + (int) c1;
        equal  += (int) e0 + (int) e1;

        if (c0 != e0 || c1 != e1)
        {
            (void) printf("  %s pair %u (address %04o): yaAGC %05o %05o, C %05o %05o\n", name, k, (unsigned int) ad, a, l, ca, cl);
        }
    }

    (void) printf("test_dndump: %s: %d/%d halves equal yaAGC (%d words in another form, %d stores refused)\n", name, equal, halves, other, g_refused);
    CHECK(equal == halves && halves > DD_MIN_HALVES);
} /* compare_dump */
