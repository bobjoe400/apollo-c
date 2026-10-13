/**********************************************************************
 * FILE NAME: sysmisc.c
 *
 * PURPOSE:
 *   Unit tests of the sys-misc lane, both vehicles, against the real
 *   library: the downlists and their table constraints, DOWNRUPT and the
 *   V74 erasable dump, LOADTIME, SGNAGREE, the self-check bank sums (also
 *   over the real Apollo 11 ropes), the service routines, LM T4RUPT's
 *   gimbal matrix, the radar verbs, the P27 V73 clock increment, R03's LM
 *   masses, and the LM P07 drift filter / PIPA rate word for word against
 *   yaAGC (tools/oracle/cases/systems_test_lm.{spec,txt}).
 *
 * AGC SOURCE:
 *   The lane's files: DOWNLINK_LISTS, DOWN-TELEMETRY_PROGRAM, RTB_OP_CODES,
 *   AGC_BLOCK_TWO_SELF-CHECK, SERVICE_ROUTINES, T4RUPT_PROGRAM,
 *   EXTENDED_VERBS, UPDATE_PROGRAM, IMU_PERFORMANCE_TEST_2 / _TESTS_4
 *   (pages in each test's comment)
 *
 * EXTERNAL VARIABLES:
 *   none (test counters and the rope image are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name            Description
 *   --------------  ------------------------------------------
 *   agc_telem_*,    the lane API under test (service.h)
 *    agc_dnlist_*,
 *    agc_selfcheck_*, agc_svc_*, agc_extverb_*, agc_uplink_*,
 *    agc_job_systems_test_*
 *   agc_state_init, executive / waitlist / DSKY set-up
 *    agc_exec_init
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails. The negative tests trip boundary
 *   assertions on purpose and are judged by the status they return; the
 *   assertion reports are not counted.
 **********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/guidance/guidance.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

static int g_checks;   /* checks run */
static int g_failures; /* checks failed */

static void     check(bool ok, int line, const char *text);
static void     test_downlist_word_counts(void);
static bool     sublist_ok(agc_vehicle_t v, int ref);
static void     test_downlist_tables(void);
static void     test_downrupt_sequence(void);
static void     test_downrupt_word_order_writes(void);
static void     test_erasable_dump(void);
static void     test_loadtime(void);
static void     test_sgnagree(void);
static void     run_selfcheck(int passes);
static void     test_banksum(void);
static void     test_showsum_display(void);
static void     test_ropechk_real(const char *name);
static void     test_service(void);
static void     run_t4rupts(int n);
static void     test_t4rupt_gimbal_matrix(void);
static void     test_uplink_clock_increment(void);
static void     test_dap_data_mass(void);
static int      oracle_ecadr(const char *name, int off);
static agc_sp_t oracle_sp(unsigned int w);
static void     oracle_set(char *line, uint16_t xsm[18], uint16_t delv[6]);
static void     oracle_fresh(void);
static void     oracle_run(const char *tag, const uint16_t xsm[18], const uint16_t delv[6]);
static int      oracle_compare(FILE *txt, const char *tag);
static void     test_systems_test_oracle(void);
static void     test_radar_verbs(void);

/**********************************************************************
 * AGC: none (test driver) -- run every test of the sys-misc lane
 *
 * RETURNS: 0 when every check passed, 1 otherwise
 **********************************************************************/
int main(void)
{
    test_downlist_word_counts();
    test_downlist_tables();
    test_downrupt_sequence();
    test_downrupt_word_order_writes();
    test_erasable_dump();
    test_loadtime();
    test_sgnagree();
    test_banksum();
    test_showsum_display();
    test_ropechk_real("Comanche055");
    test_ropechk_real("Luminary099");
    test_service();
    test_t4rupt_gimbal_matrix();
    test_radar_verbs();
    test_uplink_clock_increment();
    test_dap_data_mass();
    test_systems_test_oracle();
    (void) printf("test_sysmisc: %d checks, %d failures\n", g_checks, g_failures);

    return ((g_failures == 0) ? 0 : 1); /* exit status */
}

/**********************************************************************
 * AGC: none -- The negative tests (bad bank, no rope image, list code past DNTABLE)
 * trip boundary assertions on purpose; their reports are not printed.
 * Every such case is judged by the status it returns, never by a count of
 * reports.
 *
 * ARGUMENTS:
 *   file  const char *  I   source file of the failed assertion
 *   line  int           I   its line
 *   expr  const char *  I   its text
 *
 * RETURNS: false (the assertion's value)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) file;
    (void) line;
    (void) expr;

    return (false);
}

#define CHECK(c) check((c), __LINE__, #c)

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
 * DOWNLINK_LISTS.agc: "EACH LIST ... 100 DP WORDS" (2 s at 50 words/s).
 **********************************************************************/
static void test_downlist_word_counts(void)
{
    int words = 0; /* DP words of a list */

    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(agc_dnlist_word_count(0, &words) == AGC_STATUS_OK);
    CHECK(words == AGC_DNLIST_WORDS); /* CM coast and align  */

    for (int code = 1; code < AGC_DNLIST_CSM_COUNT; code++)
    {
        words = 0;
        CHECK(agc_dnlist_word_count(code, &words) == AGC_STATUS_OK);
        CHECK(words == AGC_DNLIST_WORDS);
    }

    agc_state_init(AGC_VEHICLE_LM);

    for (int code = 0; code < AGC_DNLIST_LM_COUNT; code++)
    {
        words = 0;
        CHECK(agc_dnlist_word_count(code, &words) == AGC_STATUS_OK);
        CHECK(words == AGC_DNLIST_WORDS);
    }

    /* out-of-range list code: refused (the library is built without
     * AGC_HOST_DEBUG, so its AGC_ASSERTs do not reach agc_assert_report) */
    CHECK(agc_dnlist_word_count(AGC_DNLIST_LM_COUNT, &words) != AGC_STATUS_OK);
}

/**********************************************************************
 * The constraints DOWN_TELEMETRY_PROGRAM.agc puts on the list tables
 * (DODOWNTM relies on them and does not check): entry ops 1-6, DNCHAN or
 * DNPTR; no DNCHAN 0; a sublist holds no DNPTR; a snapshot sublist holds
 * 1DNADRs only and all but its last entry fit DNTMBUFF (11 DP words).
 *
 * ARGUMENTS:
 *   v    agc_vehicle_t  I   the vehicle
 *   ref  int            I   the sublist id
 *
 * RETURNS: true when the sublist meets them
 **********************************************************************/
static bool sublist_ok(agc_vehicle_t v, int ref)
{
    agc_dn_list_t sub = {NULL, 0, false};
    agc_status_t  st  = (v == AGC_VEHICLE_LM) ? agc_dnlist_lm_sublist(ref, &sub) : agc_dnlist_csm_sublist(ref, &sub); /* lookup status */
    bool          ok  = st == AGC_STATUS_OK && sub.count >= 1;                                                        /* the constraints hold so far */

    for (int i = 0; ok && i < sub.count; i++)
    {
        agc_dn_entry_t e = sub.entry[i]; /* the entry */

        ok = e.op >= 1 && e.op <= AGC_DN_OP_CHAN && !(e.op == AGC_DN_OP_CHAN && e.ref == 0u) && (!sub.snapshot || e.op == 1);
    }

    return (ok && sub.snapshot == sub.entry[0].neg && (!sub.snapshot || 2 * (sub.count - 1) <= 22));
}

/**********************************************************************
 * AGC: none (test)
 **********************************************************************/
static void test_downlist_tables(void)
{
    for (int lm = 0; lm < 2; lm++)
    {
        agc_vehicle_t v = lm ? AGC_VEHICLE_LM : AGC_VEHICLE_CSM;           /* the vehicle */
        int           n = lm ? AGC_DNLIST_LM_COUNT : AGC_DNLIST_CSM_COUNT; /* DNTABLE size */

        agc_state_init(v);

        for (int code = 0; code < n; code++)
        {
            agc_dn_list_t list = {NULL, 0, false};

            /* DNTABLE of the vehicle */
            if (v == AGC_VEHICLE_LM)
            {
                CHECK(agc_dnlist_lm_control(code, &list) == AGC_STATUS_OK);
            }
            else
            {
                agc_dnlist_csm_control(code, &list);
            }

            for (int i = 0; i < list.count; i++)
            {
                agc_dn_entry_t e = list.entry[i]; /* the entry */

                CHECK(e.op >= 1 && e.op <= AGC_DN_OP_PTR);
                CHECK(!(e.op == AGC_DN_OP_CHAN && e.ref == 0u));
                CHECK(e.op != AGC_DN_OP_PTR || sublist_ok(v, (int) e.ref));
            }
        }
    }
}

/**********************************************************************
 * AGC: none -- DODOWNTM on the CM coast-and-align list (DNLSTCOD 0, ID 77777).
 **********************************************************************/
static void test_downrupt_sequence(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    agc_chan_reset();
    agc_time_set_cs(01234567u);
    agc_telem_fresh_start();
    agc_telem_restart();
    CHECK(agc_telem_downrupt() == AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_DNTM1) == 077777u); /* -DNLSTCOD         */
    CHECK(agc_chan_read(AGC_CHAN_DNTM2) == AGC_DN_LOWIDCOD);
    CHECK((agc_chan_read(AGC_CHAN_CHAN13) & 0100u) == 0u);
    int time_words = 0;                              /* TIME2,TIME1 words seen */

    for (int i = 1; i < AGC_DNLIST_WORDS; i++)
    {
        CHECK(agc_telem_downrupt() == AGC_STATUS_OK);

        if ((agc_chan_read(AGC_CHAN_CHAN13) & 0100u) == 0u)
        {
            time_words++; /* TIME2,TIME1 (WOZERO) */
            CHECK(agc_chan_read(AGC_CHAN_DNTM1) == (uint16_t) agc_erase_time2());
            CHECK(agc_chan_read(AGC_CHAN_DNTM2) == (uint16_t) agc_erase_time1());
        }
    }

    CHECK(time_words == 1);
    CHECK(agc_telem_downrupt() == AGC_STATUS_OK); /* word 101: next list */
    CHECK(agc_chan_read(AGC_CHAN_DNTM1) == 077777u);
    CHECK(agc_chan_read(AGC_CHAN_DNTM2) == AGC_DN_LOWIDCOD);
    agc_telem_select_list(1);                     /* UPDNLIST          */
    agc_telem_restart();
    CHECK(agc_telem_downrupt() == AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_DNTM1) == 077776u);
}

/**********************************************************************
 * AGC: none -- WOTEST (DOWN_TELEMETRY_PROGRAM.agc L p.997): DODOWNTM
 * writes channel 13 (WO1) only when the word order bit is 0, so a
 * DOWNRUPT in the middle of a list writes no channel 13 at all (each
 * write is a packet to the peripherals; closed-loop-fixes, ch13 at 50
 * packets a second against yaAGC's 2.5).
 **********************************************************************/
static void test_downrupt_word_order_writes(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    agc_chan_reset();
    agc_telem_fresh_start();
    agc_telem_restart();
    CHECK(agc_telem_downrupt() == AGC_STATUS_OK); /* ID word: WOZERO      */
    CHECK(agc_telem_downrupt() == AGC_STATUS_OK); /* WO1 sets bit 7       */
    uint16_t chan   = 0;                          /* channel written */
    uint16_t value  = 0;                          /* its word */
    int      writes = 0;                          /* channel 13 writes seen */

    for (int i = 0; i < AGC_CHAN_WLOG_MAX && agc_chan_next_write(&chan, &value); i++)
    {
        /* drain the log */
    }

    CHECK(agc_telem_downrupt() == AGC_STATUS_OK); /* bit 7 already 1      */

    for (int i = 0; i < AGC_CHAN_WLOG_MAX && agc_chan_next_write(&chan, &value); i++)
    {
        writes += (chan == AGC_CHAN_CHAN13) ? 1 : 0; /* a channel 13 write */
    }

    CHECK(writes == 0);
}

/**********************************************************************
 * AGC: none -- V74 erasable dump on the LM (SLAP1: DUMPCNT = BIT12, one dump): per
 * EBANK the ID 01776 + sync, DUMPLOC + TIME1, then 128 register pairs
 * read by address; after EBANK 7 the downlist resumes (DNPHASE1).
 **********************************************************************/
static void test_erasable_dump(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    agc_chan_reset();
    agc_telem_fresh_start();
    agc_telem_slap1();
    agc_telem_restart();
    CHECK(agc_erase_write_ecadr(02002u, 012345u) == AGC_STATUS_OK);
    CHECK(agc_erase_write_ecadr(02003u, 054321u) == AGC_STATUS_OK);
    agc_telem_start_dump();
    bool pair_ok = false; /* E4,1402 sent as written */

    for (int bank = 0; bank < 8; bank++)
    {
        CHECK(agc_telem_downrupt() == AGC_STATUS_OK);
        CHECK(agc_chan_read(AGC_CHAN_DNTM1) == 01776u); /* ERASID (LM)       */
        CHECK(agc_chan_read(AGC_CHAN_DNTM2) == AGC_DN_LOWIDCOD);
        CHECK(agc_telem_downrupt() == AGC_STATUS_OK);
        CHECK(agc_chan_read(AGC_CHAN_DNTM1) == (uint16_t) (bank * 0400));

        for (int pair = 0; pair < 128; pair++)
        {
            CHECK(agc_telem_downrupt() == AGC_STATUS_OK);

            if (bank == 4 && pair == 1)
            {
                pair_ok = agc_chan_read(AGC_CHAN_DNTM1) == 012345u && agc_chan_read(AGC_CHAN_DNTM2) == 054321u;
            }
        }
    }

    CHECK(pair_ok);
    CHECK(agc_telem_downrupt() == AGC_STATUS_OK); /* the downlist again */
    CHECK(agc_chan_read(AGC_CHAN_DNTM1) == 077777u);
    CHECK(agc_chan_read(AGC_CHAN_DNTM2) == AGC_DN_LOWIDCOD);
}

/**********************************************************************
 * AGC: none -- LOADTIME: MPAC = TIME2, TIME1 (cs, 2^28).
 **********************************************************************/
static void test_loadtime(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    agc_time_set_cs(0x0ABCDEFu);
    CHECK(agc_rtb_loadtime() == (agc_dp_t) 0x0ABCDEF);
    agc_time_advance_cs(5);
    CHECK(agc_rtb_loadtime() == (agc_dp_t) (0x0ABCDEF + 5));
}

/**********************************************************************
 * AGC: none -- SGNAGREE / TPAGREE: +1, -1 (= 2^14 - 1 low units) becomes +0, +37777.
 **********************************************************************/
static void test_sgnagree(void)
{
    agc_mpac_tp_t m = {{1, -1, 0}};

    CHECK(agc_rtb_sgnagree(&m) == 1);
    CHECK(m.w[0] == 0 && m.w[1] == AGC_SP_MAX && m.w[2] == 0);
    agc_mpac_tp_t n = {{-2, 5, -3}}; /* negative overall      */

    CHECK(agc_rtb_sgnagree(&n) == -1);
    CHECK(n.w[0] <= 0 && n.w[1] <= 0 && n.w[2] <= 0);
    CHECK(agc_rtb_sgnagree_dp(1, -1) == (agc_dp_t) AGC_SP_MAX);
    CHECK(agc_rtb_sgnagree_dp(-1, 1) == -(agc_dp_t) AGC_SP_MAX);
    CHECK(agc_rtb_sgnagree_dp(3, 4) == (agc_dp_t) (3 * 040000 + 4));
}

/* ROPECHK / SHOWSUM on a synthetic rope: bank 5 holds a sum word that
 * makes the bank sum +5, then two TC SELF words and a bugger word.       */
static uint16_t g_rope[AGC_ROPE_BANKS * AGC_ROPE_BANK_WORDS]; /* synthetic / real rope, 044 banks of 02000 words */

/**********************************************************************
 * AGC: none -- Run idle passes until the self-check is back at SELFCHK (at most one
 * pass per bank plus the SMODECHK passes).
 *
 * ARGUMENTS:
 *   passes  int  I   idle passes to run
 **********************************************************************/
static void run_selfcheck(int passes)
{
    for (int i = 0; i < passes; i++)
    {
        agc_selfcheck_idle_pass();
    }
}

/**********************************************************************
 * AGC: none (test) -- V91 SDISPLAY: MPAC+2 = ADRS1 (ECADR SKEEP1, 1371)
 * and V05N01, so the noun shows SKEEP1..3 from that address
 **********************************************************************/
static void test_showsum_display(void)
{
    uint16_t w[3]  = {0, 0, 0}; /* SKEEP1..3 */
    uint16_t at[3] = {1, 1, 1}; /* the words at 1371..1373 */

    agc_state_init(AGC_VEHICLE_CSM);
    agc_exec_init();
    agc_waitlist_init();
    agc_dsky_init();
    agc_selfcheck_set_rope(g_rope, 6);
    agc_erase_set_extvbact(0);
    CHECK(agc_dsky_keys("V91E") == AGC_STATUS_OK);

    for (int i = 0; i < 20 && agc_exec_run_one() != AGC_STATUS_NOT_FOUND; i++)
    {
        /* CHARIN, the extended verb, the display */
    }

    agc_selfcheck_showsum_words(w);

    for (int i = 0; i < 3; i++)
    {
        agc_erase_read_ecadr((uint16_t) (01371u + (uint16_t) i), &at[i]);
    }

    CHECK(at[0] == w[0] && at[1] == w[1] && at[2] == w[2]);
    agc_dsky_state_t s; /* DSKY readout */

    CHECK(agc_dsky_get_state(&s) == AGC_STATUS_OK);
    CHECK(s.verb[0] == 0 && s.verb[1] == 5 && s.noun[0] == 0 && s.noun[1] == 1);
}

/**********************************************************************
 * AGC: none (test) -- ROPECHK / SHOWSUM on a synthetic rope
 **********************************************************************/
static void test_banksum(void)
{
    uint16_t *b5 = &g_rope[5 * AGC_ROPE_BANK_WORDS]; /* bank 5 of the rope */

    agc_state_init(AGC_VEHICLE_CSM);
    b5[0]        = 012345u;
    b5[1]        = 070000u;                    /* negative: overflow paths */
    b5[2]        = 070000u;
    b5[10]       = (uint16_t) (002000u + 10u); /* TC SELF at 2012       */
    b5[11]       = (uint16_t) (002000u + 11u); /* TC SELF at 2013       */
    b5[12]       = 0;                          /* bugger word (set below) */
    b5[13]       = 077777u;                    /* not summed            */
    agc_selfcheck_set_rope(g_rope, 6);
    uint16_t sum = 0;                          /* bank sum */
    uint16_t bug = 0;                          /* bugger word */

    CHECK(agc_selfcheck_banksum(5, &sum, &bug) == AGC_STATUS_OK);

    /* choose the bugger word so the sum becomes +5 and check it           */
    uint16_t need = (uint16_t) ((5u + (077777u - sum)) % 077777u); /* bugger word for a sum of +5 */

    b5[12] = need;
    CHECK(agc_selfcheck_banksum(5, &sum, &bug) == AGC_STATUS_OK);
    CHECK(bug == need);
    CHECK(sum == 5u || sum == (uint16_t) (077777u - 5u));
    CHECK(agc_selfcheck_banksum(044, &sum, &bug) == AGC_STATUS_BAD_ARG);

    /* SHOW-BANKSUM: bank 0, then five NXTBNKs to bank 5.                   */
    uint16_t w[3] = {0, 0, 0};

    agc_selfcheck_showsum_start();
    agc_selfcheck_showsum_words(w);
    CHECK(w[0] == 0u && w[1] == 0u);

    for (int i = 0; i < 5; i++)
    {
        agc_selfcheck_showsum_next();
    }

    agc_selfcheck_showsum_words(w);
    CHECK(w[1] == 5u && w[2] == need && w[0] == sum);

    /* ROPECHK, + option: bank 0 (sum +0) passes, bank 1 (sum +0) fails:
     * ERCOUNT +1, alarm 01102, SMODE = +0 (idle loop).                    */
    agc_selfcheck_restart();
    agc_selfcheck_set_smode(5);
    uint16_t errs = agc_selfcheck_error_count(); /* ERCOUNT before */

    run_selfcheck(3);                                             /* SMODECHK, bank 0, bank 1 */
    CHECK(agc_selfcheck_error_count() == (uint16_t) (errs + 1u));
    CHECK(agc_erase_failreg(0) == 01102u);
    CHECK(agc_selfcheck_smode() == 0u);
    run_selfcheck(AGC_ROPE_BANKS + 2);
    CHECK(agc_selfcheck_error_count() == (uint16_t) (errs + 1u)); /* idle */

    /* -0: every bad bank counts (banks 1-4 and 6-43: all +0 sums).       */
    agc_selfcheck_restart();
    agc_selfcheck_set_smode(077777u);
    errs = agc_selfcheck_error_count();
    run_selfcheck(1 + AGC_ROPE_BANKS);
    CHECK(agc_selfcheck_error_count() == (uint16_t) (errs + AGC_ROPE_BANKS - 2u));
    CHECK(agc_selfcheck_smode() == 077777u);

    /* - option: restart at SELFCHK after each error, SMODE kept.          */
    agc_selfcheck_restart();
    agc_selfcheck_set_smode(077772u); /* -5 */
    errs = agc_selfcheck_error_count();
    run_selfcheck(6);                 /* (SMODECHK, bank 0, bank 1) twice */
    CHECK(agc_selfcheck_error_count() == (uint16_t) (errs + 2u));
    CHECK(agc_selfcheck_smode() == 077772u);

    /* option > 10 octal: BNKOPTN, the idle loop.                          */
    agc_selfcheck_restart();
    agc_selfcheck_set_smode(011u);
    run_selfcheck(1);
    CHECK(agc_selfcheck_smode() == 0u);

    /* no rope: the C program has no fixed memory to test, all pass.      */
    agc_selfcheck_set_rope(NULL, 0);
    agc_selfcheck_restart();
    agc_selfcheck_set_smode(5);
    errs = agc_selfcheck_error_count();
    run_selfcheck(1 + AGC_ROPE_BANKS);
    CHECK(agc_selfcheck_error_count() == errs && agc_selfcheck_smode() == 5u);
    agc_selfcheck_fresh_start(true);
    CHECK(agc_selfcheck_smode() == 0u && agc_selfcheck_error_count() == 0u);
} /* test_banksum */

/**********************************************************************
 * AGC: none -- ROPECHK over the real Apollo 11 ropes (tools/oracle/ropes/<rope>.bin,
 * yaYUL order: banks 2, 3, 0, 1, 4..43, each word shifted left by one):
 * every bank sums to +-its number, so ROPECHK runs a full cycle with no
 * 01102 alarm.  SKIP when the rope is not found (run from the root).
 *
 * ARGUMENTS:
 *   name  const char *  I   rope name (Comanche055, Luminary099)
 **********************************************************************/
static void test_ropechk_real(const char *name)
{
    char  path[128]; /* the rope file */
    FILE *f = NULL;

    for (int k = 0; k < 3 && f == NULL; k++)
    {
        static const char *const up[3] = {"", "../", "../../"}; /* run from the root or a build dir */

        (void) snprintf(path, sizeof (path), "%stools/oracle/ropes/%s.bin", up[k], name);
        f = fopen(path, "rb");
    }

    if (f == NULL)
    {
        (void) printf("test_sysmisc: SKIP %s rope (not found)\n", name);

        return;
    }

    bool ok = true; /* every bank read */

    for (int fb = 0; fb < AGC_ROPE_BANKS && ok; fb++)
    {
        int           bank = (fb < 4) ? (fb ^ 2) : fb; /* file order 2, 3, 0, 1, 4.. */
        unsigned char b[2 * AGC_ROPE_BANK_WORDS];      /* one bank as stored */

        ok = fread(b, 1, sizeof (b), f) == sizeof (b);

        for (int r = 0; r < AGC_ROPE_BANK_WORDS; r++)
        {
            g_rope[bank * AGC_ROPE_BANK_WORDS + r] = (uint16_t) ((((unsigned int) b[2 * r] << 8) | b[2 * r + 1]) >> 1);
        }
    }

    (void) fclose(f);
    CHECK(ok);
    agc_selfcheck_set_rope(g_rope, AGC_ROPE_BANKS);

    for (int bank = 0; bank < AGC_ROPE_BANKS; bank++)
    {
        uint16_t sum = 0; /* bank sum */
        uint16_t bug = 0; /* bugger word */

        CHECK(agc_selfcheck_banksum(bank, &sum, &bug) == AGC_STATUS_OK);
        CHECK(sum == (uint16_t) bank || sum == (uint16_t) (077777u - (unsigned int) bank));
    }

    agc_selfcheck_restart();
    agc_selfcheck_set_smode(5);
    uint16_t errs = agc_selfcheck_error_count(); /* ERCOUNT before */

    run_selfcheck(2 * (1 + AGC_ROPE_BANKS));
    CHECK(agc_selfcheck_error_count() == errs && agc_selfcheck_smode() == 5u);
    agc_selfcheck_set_smode(0);
    agc_selfcheck_set_rope(NULL, 0);
} /* test_ropechk_real */

/**********************************************************************
 * AGC: none -- UPFLAG / DOWNFLAG, GENTRAN, EXTVBACT.
 **********************************************************************/
static void test_service(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    uint16_t w = agc_svc_upflag(AGC_FLAG_QUITFLAG); /* the flagword UPFLAG leaves */

    CHECK((w & agc_flag_mask(AGC_FLAG_QUITFLAG)) != 0u);
    CHECK(agc_flag_test(AGC_FLAG_QUITFLAG));
    w                = agc_svc_downflag(AGC_FLAG_QUITFLAG);
    CHECK((w & agc_flag_mask(AGC_FLAG_QUITFLAG)) == 0u);
    uint16_t from[5] = {1, 2, 3, 4, 5};
    uint16_t to[5]   = {0, 0, 0, 0, 0};

    agc_svc_gentran(from, to, 5);
    CHECK(to[0] == 1u && to[4] == 5u);
    agc_erase_set_extvbact(024u);
    CHECK(agc_extverb_busy());
    agc_svc_b5off();
    CHECK(agc_erase_extvbact() == 004u); /* bit 5 cleared          */
    agc_erase_set_extvbact(0);
    CHECK(!agc_extverb_busy());
}

/**********************************************************************
 * AGC: none -- Run n T4RUPTs (the quick-display passes do not advance the phase).
 *
 * ARGUMENTS:
 *   n  int  I   T4RUPTs to run
 **********************************************************************/
static void run_t4rupts(int n)
{
    for (int pass = 0; pass < n; pass++)
    {
        uint16_t     next = 0;                 /* TIME4 delay, cs */
        agc_status_t st   = agc_t4rupt(&next); /* T4RUPT status */

        /* BAILOUT 1203: the waitlist is never serviced here, so the
         * ISSZERO WAITLIST calls fill it (the AGC would restart).      */
        CHECK(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY || st == AGC_STATUS_BAILOUT);
        CHECK(next > 0u);
    }
}

/**********************************************************************
 * AGC: none -- T4RUPT on the LM: within 0.96 s T4JUMP runs DAPT4S/GPMATRIX, so M11 =
 * SIN(MG) and M32 = COS(OG) from CDUZ, CDUX (scaled at 1); RCSMONIT runs
 * at phases 0 and 4 (every 480 ms, dap-lm).  The first passes see the ISS
 * come on (IMUMON zeroes the ICDUs), so the CDUs are set after them.
 **********************************************************************/
static void test_t4rupt_gimbal_matrix(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    agc_chan_reset();
    CHECK(agc_chan_set_input(AGC_CHAN_CHAN30, 077777u) == AGC_STATUS_OK);
    CHECK(agc_chan_set_input(AGC_CHAN_CHAN33, 077777u) == AGC_STATUS_OK);
    run_t4rupts(24);
    agc_erase_set_cduz(010000u);     /* MG = 45 degrees       */
    agc_erase_set_cdux(0u);          /* OG = 0                */
    run_t4rupts(16);
    int m11 = (int) agc_erase_m11(); /* M11, 2^14 = 1 */
    int m32 = (int) agc_erase_m32(); /* M32, 2^14 = 1 */

    CHECK(m11 > 11500 && m11 < 11700); /* .7071 x 2^14 = 11585  */
    CHECK(m32 > 16300);                /* COS 0 = 1 (POSMAX)    */
    CHECK(agc_erase_m22() == 0);       /* SIN 0                 */
}

/**********************************************************************
 * AGC: none -- V73 (UPEND73): the two words loaded into UPBUFF are the octal clock
 * increment (DCA UPBUFF, DXCH UPBUFF +8D, TIMEDIDL); V33 on the V21N02
 * adds it to TIME2,TIME1.  The V21N01 display carries the ECADR of the
 * word to load in MPAC+2 (UPBUFF = 0304 in Comanche); the ground's word
 * is stored there as pinball's internal machine-address load does, and
 * the load job takes it from that address.
 **********************************************************************/
static void test_uplink_clock_increment(void)
{
    agc_job_ctx_t ctx = {AGC_JOB_EXTENDED_VERB, 030, false, -1, 0};

    agc_state_init(AGC_VEHICLE_CSM);
    agc_exec_init();
    agc_waitlist_init();
    agc_dsky_init();
    agc_erase_set_extvbact(0);
    agc_time_set_cs(01000000u);
    CHECK(agc_uplink_update_start(&ctx, 73) == AGC_STATUS_OK);
    CHECK(agc_uplink_update_target() == 4);                   /* UPBUFF +0          */
    CHECK(agc_erase_write_ecadr(0304u, 1u) == AGC_STATUS_OK); /* V21N01 data at MPAC+2 */
    CHECK(agc_job_uplink_update_load(&ctx) == AGC_STATUS_OK);
    CHECK(agc_uplink_update_target() == 5);                   /* UPBUFF +1          */
    CHECK(agc_erase_write_ecadr(0305u, 2u) == AGC_STATUS_OK);
    CHECK(agc_job_uplink_update_load(&ctx) == AGC_STATUS_OK);
    CHECK(agc_uplink_update_target() == 24);                  /* UPTEMP (V21N02)    */
    CHECK(agc_uplink_update_dnword(4) == 1u && agc_uplink_update_dnword(5) == 2u);
    CHECK(agc_job_uplink_update_proceed(&ctx) == AGC_STATUS_OK);
    CHECK(agc_time_now_cs() == 01000000u + 040002u);          /* 1 x 2^14 + 2 cs */
}

/**********************************************************************
 * AGC: none -- DAPDAT2 (R03, N47 proceed): LEMMASS must exceed the empty LM, MINLMD +
 * MINMINLM = -2850 -2200 kg at 2^16 (rope words -713, -550).  An 8000 kg
 * LM (word 2000) is accepted and becomes MASS; 4000 kg (1000) is not.
 **********************************************************************/
static void test_dap_data_mass(void)
{
    agc_job_ctx_t ctx = {AGC_JOB_EXTENDED_VERB, 030, false, -1, 0};

    agc_state_init(AGC_VEHICLE_LM);
    agc_exec_init();
    agc_waitlist_init();
    agc_dsky_init();
    agc_erase_set_lemmass(1000);
    agc_guid_state()->lm.mass = 0;
    CHECK(agc_job_dap_data_load_mass(&ctx) == AGC_STATUS_OK);
    CHECK(agc_guid_state()->lm.mass == 0); /* asks for N47 again */
    agc_erase_set_lemmass(2000);
    CHECK(agc_job_dap_data_load_mass(&ctx) == AGC_STATUS_OK);
    CHECK(agc_guid_state()->lm.mass == agc_dp_from_sp(2000));
}

/* ---- P07 against yaAGC (tools/oracle/cases/systems_test_lm.*) -------- */

#define ORACLE_LINE 256 /* longest spec / dump line */

/**********************************************************************
 * AGC: none -- ECADR of a spec name: AINLA+k, W+k, E4,ooo (the test erasables are kept
 * at their AGC addresses, src/lm/systems_test.c).
 *
 * ARGUMENTS:
 *   name  const char *  I   AINLA, W or E4,ooo
 *   off   int           I   word offset
 *
 * RETURNS: the ECADR, -1 for another name
 **********************************************************************/
static int oracle_ecadr(const char *name, int off)
{
    int ec = -1; /* the ECADR */

    if (strncmp(name, "AINLA", 5) == 0)
    {
        ec = 02434 + off;
    }
    else if (strcmp(name, "W") == 0)
    {
        ec = 02400 + off;
    }
    else if (strncmp(name, "E4,", 3) == 0)
    {
        ec = 02000 + (int) strtol(name + 3, NULL, 8) - 01400 + off;
    }

    return (ec);
}

/**********************************************************************
 * AGC: none -- A 15-bit ones'-complement word as a signed value.
 *
 * ARGUMENTS:
 *   w  unsigned  I   a 15-bit word
 *
 * RETURNS: its ones' complement value
 **********************************************************************/
static agc_sp_t oracle_sp(unsigned int w)
{
    return ((agc_sp_t) (((w & 040000u) != 0u) ? -(int) (077777u - w) : (int) w)); /* negative: -(77777 - w) */
}

/**********************************************************************
 * AGC: none -- Apply one spec "set NAME[+k]=w,w,.." line (XSM and DELVX as values).
 *
 * ARGUMENTS:
 *   line  char *       I   the spec line (modified by strtok)
 *   xsm   uint16_t []  O   XSM words collected
 *   delv  uint16_t []  O   DELVX words collected
 **********************************************************************/
static void oracle_set(char *line, uint16_t xsm[18], uint16_t delv[6])
{
    char *eq  = strchr(line, '='); /* the '=' */
    char *add = NULL;              /* the '+' of NAME+k */
    int   off = 0;                 /* k */
    int   k   = 0;                 /* word index in the line */

    *eq = '\0';
    add = strchr(line + 4, '+');

    if (add != NULL)
    {
        *add = '\0';
        off  = atoi(add + 1);
    }

    for (char *t = strtok(eq + 1, ","); t != NULL; t = strtok(NULL, ","), k++)
    {
        unsigned int w  = (unsigned int) strtoul(t, NULL, 8); /* the word, octal */
        int          ec = oracle_ecadr(line + 4, off + k);    /* its ECADR */

        if (strcmp(line + 4, "XSM") == 0 && off + k < 18)
        {
            xsm[off + k] = (uint16_t) w;
        }
        else if (strcmp(line + 4, "DELVX") == 0 && off + k < 6)
        {
            delv[off + k] = (uint16_t) w;
        }
        else if (ec >= 0)
        {
            CHECK(agc_erase_write_ecadr((uint16_t) ec, (uint16_t) w) == AGC_STATUS_OK);
        }
    }
}

/**********************************************************************
 * AGC: none -- A fresh LM with an empty ECADR image for the next case.
 **********************************************************************/
static void oracle_fresh(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    agc_exec_init();
    agc_waitlist_init();
    agc_dsky_init();
    agc_erase_reset_ecadr();
}

/**********************************************************************
 * AGC: none -- Run the C side of one case: ALFLT (LENGTHOT 1: PERFERAS goes to SLEEPIE)
 * or PIPJOBB +10 through the QPLACE return.
 *
 * ARGUMENTS:
 *   tag   const char *       I   the case tag (alflt_*, pip_*)
 *   xsm   const uint16_t []  I   XSM words
 *   delv  const uint16_t []  I   DELVX words
 **********************************************************************/
static void oracle_run(const char *tag, const uint16_t xsm[18], const uint16_t delv[6])
{
    agc_job_ctx_t ctx = {AGC_JOB_SYSTEMS_TEST_FILTER, 020, false, -1, 0};
    agc_vec_t     d; /* DELV from the words, pulses x 2^14 */

    for (int i = 0; i < 9; i++)
    {
        agc_align_ram()->xsm.row[i / 3].c[i % 3] = agc_dp_pack(oracle_sp(xsm[2 * i]), oracle_sp(xsm[2 * i + 1]));
    }

    for (int i = 0; i < 3; i++)
    {
        d.c[i] = agc_dp_pack(oracle_sp(delv[2 * i]), oracle_sp(delv[2 * i + 1]));
    }

    agc_erase_set_delv(d);

    if (strncmp(tag, "alflt", 5) == 0)
    {
        CHECK(agc_erase_write_ecadr(02400 + 10, 1) == AGC_STATUS_OK); /* LENGTHOT */
        CHECK(agc_job_systems_test_filter(&ctx) == AGC_STATUS_OK);
    }
    else
    {
        CHECK(agc_erase_write_ecadr(02400 + 15, 5) == AGC_STATUS_OK); /* QPLACE: PIPJOBB result */
        ctx.id = AGC_JOB_SYSTEMS_TEST_RESUME;
        (void) agc_job_systems_test_resume(&ctx);                     /* ends in COAALIGN's stall */
    }
}

/**********************************************************************
 * AGC: none -- Compare the dump lines of the case that follows *p (-0 reads as +0: the
 * C does not keep -0 words).
 *
 * ARGUMENTS:
 *   txt  FILE *        I   the dump, positioned after the CASE line
 *   tag  const char *  I   the case tag
 *
 * RETURNS: the number of words compared
 **********************************************************************/
static int oracle_compare(FILE *txt, const char *tag)
{
    char line[ORACLE_LINE]; /* a dump line */
    int  n = 0;             /* words compared */

    while (fgets(line, sizeof (line), txt) != NULL && strncmp(line, "END", 3) != 0)
    {
        char         name[64]; /* the dumped name */
        unsigned int want = 0; /* the AGC's word */
        uint16_t     got  = 0; /* the C word */
        int          off  = 0; /* offset in NAME+k */

        bool named = sscanf(line, "%63s %o", name, &want) == 2 && (strncmp(name, "AINLA", 5) == 0 || strncmp(name, "W+", 2) == 0 || strncmp(name, "DSPTEM2", 7) == 0); /* a word line (not a header) */

        if (!named)
        {
            continue; /* CASE header lines */
        }

        if (strchr(name, '+') != NULL)
        {
            off                = atoi(strchr(name, '+') + 1);
            *strchr(name, '+') = '\0';
        }

        if (strcmp(name, "DSPTEM2") == 0)
        {
            got = agc_dsky_dsptem(3 + off);
        }
        else
        {
            agc_erase_read_ecadr((uint16_t) oracle_ecadr(name, off), &got);
        }

        want = (want == 077777u) ? 0u : want; /* -0 reads as +0 in C */
        n++;

        if (got != (uint16_t) want)
        {
            (void) printf("oracle %s %s+%d: C %05o AGC %05o\n", tag, name, off, got, want);
            CHECK(false);
        }
    }

    return (n);
} /* oracle_compare */

/**********************************************************************
 * AGC: none -- Every case of systems_test_lm: inputs from the spec, words from the dump.
 * SKIP when the case files are not found (run from the root).
 **********************************************************************/
static void test_systems_test_oracle(void)
{
    FILE    *spec     = fopen("tools/oracle/cases/systems_test_lm.spec", "r");
    FILE    *txt      = fopen("tools/oracle/cases/systems_test_lm.txt", "r");
    char     line[ORACLE_LINE]; /* a spec line */
    uint16_t xsm[18]  = {0};
    uint16_t delv[6]  = {0};
    int      compared = 0;      /* words compared */

    if (spec == NULL || txt == NULL)
    {
        (void) printf("test_sysmisc: SKIP systems_test_lm oracle cases (not found)\n");
    }

    oracle_fresh();

    while (spec != NULL && txt != NULL && fgets(line, sizeof (line), spec) != NULL)
    {
        line[strcspn(line, "\n")] = '\0';

        if (strncmp(line, "set ", 4) == 0 && strncmp(line, "set FIXLOC", 10) != 0 && strncmp(line, "set PUSHLOC", 11) != 0)
        {
            oracle_set(line, xsm, delv);
        }
        else if (strncmp(line, "run ", 4) == 0)
        {
            char want[ORACLE_LINE + 8]; /* the CASE line to find */

            oracle_run(line + 4, xsm, delv);
            (void) snprintf(want, sizeof (want), "CASE %s\n", line + 4);

            while (fgets(line, sizeof (line), txt) != NULL && strcmp(line, want) != 0)
            {
            }

            compared += oracle_compare(txt, want + 5);
            memset(xsm, 0, sizeof (xsm));
            memset(delv, 0, sizeof (delv));
            oracle_fresh();
        }
    }

    CHECK(spec == NULL || txt == NULL || compared == 4 * 100 + 4 * 4);

    if (spec != NULL)
    {
        (void) fclose(spec);
    }

    if (txt != NULL)
    {
        (void) fclose(txt);
    }
} /* test_systems_test_oracle */

/**********************************************************************
 * AGC: none -- RRDESEND (V44) outside continuous designate is just GOPIN: RADMODES and
 * NORRMON untouched.
 **********************************************************************/
static void test_radar_verbs(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    agc_erase_set_radmodes(01000u); /* designate, not continuous */
    agc_flag_set(AGC_FLAG_NORRMON);
    CHECK(agc_extverb_rr_designate_end() == AGC_STATUS_OK);
    CHECK(agc_erase_radmodes() == 01000u);
    CHECK(agc_flag_test(AGC_FLAG_NORRMON));

    /* after the 1 s delay of a real RRDESEND: R25 back on, GOPIN          */
    CHECK(agc_job_rr_designate_end_monitor() == AGC_STATUS_OK);
    CHECK(!agc_flag_test(AGC_FLAG_NORRMON));
    agc_erase_set_rsampdt(0);
    CHECK(agc_erase_rsampdt() == 0);
}
