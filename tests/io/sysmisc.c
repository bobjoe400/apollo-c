/*
 * sysmisc.c -- sys-misc lane unit tests (both vehicles).
 *
 * Covers: every downlist (CM 5, LM 6) is the documented 100 DP words; the
 * DOWNRUPT sequence of the CM coast-and-align list (ID word 77777 + sync
 * 77340 with word order 0, TIME2/TIME1 with word order 0, the next list
 * after 100 words); LOADTIME returns TIME2,TIME1; SGNAGREE on a sign-
 * disagreeing DP / TP fixes the signs and keeps the value; the self-check
 * bank sum (end-around carry, TC SELF words and the bugger word); service
 * routines UPFLAG/DOWNFLAG and GENTRAN; EXTVBACT; LM T4RUPT fills the
 * DAP gimbal matrix M11..M32; RRDESEND is a no-op outside continuous
 * designate.
 *
 * AGC_ASSERT failures are expected in the negative tests; this file counts
 * any that reach agc_assert_report (only asserts compiled with
 * AGC_HOST_DEBUG do) instead of printing.
 */
#include <stdio.h>

#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/programs/rendezvous.h"

static int g_checks;
static int g_failures;
static int g_assert_reports;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) file;
    (void) line;
    (void) expr;
    g_assert_reports++;

    return (false);
}

#define CHECK(c) check((c), __LINE__, #c)

static void check(bool ok, int line, const char *text)
{
    g_checks++;

    if (!ok)
    {
        g_failures++;
        (void) printf("FAIL line %d: %s\n", line, text);
    }
}

/* DOWNLINK_LISTS.agc: "EACH LIST ... 100 DP WORDS" (2 s at 50 words/s). */
static void test_downlist_word_counts(void)
{
    int words = 0;

    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(agc_dnlist_word_count(AGC_VEHICLE_CSM, 0, &words) == AGC_STATUS_OK);
    CHECK(words == AGC_DNLIST_WORDS); /* CM coast and align  */

    for (int code = 1; code < AGC_DNLIST_CSM_COUNT; code++)
    {
        words = 0;
        CHECK(agc_dnlist_word_count(AGC_VEHICLE_CSM, code, &words) ==
              AGC_STATUS_OK);
        CHECK(words == AGC_DNLIST_WORDS);
    }

    agc_state_init(AGC_VEHICLE_LM);

    for (int code = 0; code < AGC_DNLIST_LM_COUNT; code++)
    {
        words = 0;
        CHECK(agc_dnlist_word_count(AGC_VEHICLE_LM, code, &words) ==
              AGC_STATUS_OK);
        CHECK(words == AGC_DNLIST_WORDS);
    }

    /* out-of-range list code: refused (the library is built without
     * AGC_HOST_DEBUG, so its AGC_ASSERTs do not reach agc_assert_report) */
    CHECK(agc_dnlist_word_count(AGC_VEHICLE_LM, AGC_DNLIST_LM_COUNT, &words) !=
          AGC_STATUS_OK);
}

/* DODOWNTM on the CM coast-and-align list (DNLSTCOD 0, ID 77777).        */
static void test_downrupt_sequence(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    agc_chan_reset();
    agc_time_set_cs(01234567u);
    CHECK(agc_telem_fresh_start(0) == AGC_STATUS_OK);
    CHECK(agc_telem_downrupt() == AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_DNTM1) == 077777u); /* -DNLSTCOD         */
    CHECK(agc_chan_read(AGC_CHAN_DNTM2) == AGC_DN_LOWIDCOD);
    CHECK((agc_chan_read(AGC_CHAN_CHAN13) & 0100u) == 0u);
    int time_words = 0;

    for (int i = 1; i < AGC_DNLIST_WORDS; i++)
    {
        CHECK(agc_telem_downrupt() == AGC_STATUS_OK);

        if ((agc_chan_read(AGC_CHAN_CHAN13) & 0100u) == 0u)
        {
            time_words++; /* TIME2,TIME1 (WOZERO) */
            CHECK(agc_chan_read(AGC_CHAN_DNTM1) ==
                  (uint16_t) agc_erase_time2());
            CHECK(agc_chan_read(AGC_CHAN_DNTM2) ==
                  (uint16_t) agc_erase_time1());
        }
    }

    CHECK(time_words == 1);
    CHECK(agc_telem_downrupt() == AGC_STATUS_OK); /* word 101: next list */
    CHECK(agc_chan_read(AGC_CHAN_DNTM1) == 077777u);
    CHECK(agc_chan_read(AGC_CHAN_DNTM2) == AGC_DN_LOWIDCOD);
    agc_telem_select_list(1); /* UPDNLIST          */
    CHECK(agc_telem_restart() == AGC_STATUS_OK);
    CHECK(agc_telem_downrupt() == AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_DNTM1) == 077776u);
}

/* LOADTIME: MPAC = TIME2, TIME1 (cs, 2^28).                               */
static void test_loadtime(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    agc_time_set_cs(0x0ABCDEFu);
    CHECK(agc_rtb_loadtime() == (agc_dp_t) 0x0ABCDEF);
    agc_time_advance_cs(5);
    CHECK(agc_rtb_loadtime() == (agc_dp_t) (0x0ABCDEF + 5));
}

/* SGNAGREE / TPAGREE: +1, -1 (= 2^14 - 1 low units) becomes +0, +37777.  */
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
static uint16_t g_rope[6 * AGC_ROPE_BANK_WORDS];

static void test_banksum(void)
{
    uint16_t *b5 = &g_rope[5 * AGC_ROPE_BANK_WORDS];

    b5[0]  = 012345u;
    b5[1]  = 070000u;       /* negative: overflow paths */
    b5[2]  = 070000u;
    b5[10] = (uint16_t) (002000u + 10u); /* TC SELF at 2012       */
    b5[11] = (uint16_t) (002000u + 11u); /* TC SELF at 2013       */
    b5[12] = 0;                          /* bugger word (set below) */
    b5[13] = 077777u;                    /* not summed            */
    CHECK(agc_selfcheck_set_rope(g_rope, 6) == AGC_STATUS_OK);
    uint16_t sum = 0;
    uint16_t bug = 0;

    CHECK(agc_selfcheck_banksum(5, &sum, &bug) == AGC_STATUS_OK);
    /* choose the bugger word so the sum becomes +5 and check it           */
    uint16_t need = (uint16_t) ((5u + (077777u - sum)) % 077777u);

    b5[12] = need;
    CHECK(agc_selfcheck_banksum(5, &sum, &bug) == AGC_STATUS_OK);
    CHECK(bug == need);
    CHECK(sum == 5u || sum == (uint16_t) (077777u - 5u));
    CHECK(agc_selfcheck_bank_ok(5));
    CHECK(agc_selfcheck_bank_ok(0));  /* all +0: sum +0 = bank 0 */
    CHECK(!agc_selfcheck_bank_ok(4)); /* +0 sum, bank 4: error  */
    CHECK(agc_selfcheck_showsum_start() == AGC_STATUS_OK);
    CHECK(agc_selfcheck_skeep(2) == 0u);

    for (int i = 0; i < 5; i++)
    {
        CHECK(agc_selfcheck_showsum_next() == AGC_STATUS_OK);
    }

    CHECK(agc_selfcheck_skeep(2) == 5u && agc_selfcheck_skeep(3) == need);
    CHECK(agc_selfcheck_set_smode(5) == AGC_STATUS_OK); /* ROPECHK, + option */
    uint16_t errs = agc_selfcheck_error_count();

    CHECK(agc_selfcheck_idle_pass() == AGC_STATUS_ALARM); /* bank 1 fails  */
    CHECK(agc_selfcheck_error_count() == (uint16_t) (errs + 1u));
    CHECK(agc_selfcheck_smode() == 0u); /* + option: idle loop    */
    CHECK(agc_selfcheck_set_rope(NULL, 0) == AGC_STATUS_OK);
}

/* UPFLAG / DOWNFLAG, GENTRAN, EXTVBACT.                                   */
static void test_service(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    uint16_t w = agc_svc_upflag(AGC_FLAG_QUITFLAG);

    CHECK((w & agc_flag_mask(AGC_FLAG_QUITFLAG)) != 0u);
    CHECK(agc_flag_test(AGC_FLAG_QUITFLAG));
    w = agc_svc_downflag(AGC_FLAG_QUITFLAG);
    CHECK((w & agc_flag_mask(AGC_FLAG_QUITFLAG)) == 0u);
    uint16_t from[5] = {1, 2, 3, 4, 5};
    uint16_t to[5]   = {0, 0, 0, 0, 0};

    CHECK(agc_svc_gentran(from, to, 5) == AGC_STATUS_OK);
    CHECK(to[0] == 1u && to[4] == 5u);
    agc_erase_set_extvbact(024u);
    CHECK(agc_extverb_busy());
    CHECK(agc_svc_b5off() == AGC_STATUS_OK);
    CHECK(agc_erase_extvbact() == 004u); /* bit 5 cleared          */
    agc_svc_set_extvbact(0);
    CHECK(!agc_extverb_busy());
}

/* Run n T4RUPTs (the quick-display passes do not advance the phase).    */
static void run_t4rupts(int n)
{
    for (int pass = 0; pass < n; pass++)
    {
        uint16_t     next = 0;
        agc_status_t st   = agc_t4rupt(&next);

        CHECK(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY);
        CHECK(next > 0u);
    }
}

/* T4RUPT on the LM: within 0.96 s T4JUMP runs DAPT4S/GPMATRIX, so M11 =
 * SIN(MG) and M32 = COS(OG) from CDUZ, CDUX (scaled at 1); RCSMONIT runs
 * at phases 0 and 4 (every 480 ms, dap-lm).  The first passes see the ISS
 * come on (IMUMON zeroes the ICDUs), so the CDUs are set after them.     */
static void test_t4rupt_gimbal_matrix(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    agc_chan_reset();
    CHECK(agc_chan_set_input(AGC_CHAN_CHAN30, 077777u) == AGC_STATUS_OK);
    CHECK(agc_chan_set_input(AGC_CHAN_CHAN33, 077777u) == AGC_STATUS_OK);
    run_t4rupts(24);
    agc_erase_set_cduz(010000u); /* MG = 45 degrees       */
    agc_erase_set_cdux(0u);      /* OG = 0                */
    run_t4rupts(16);
    int m11 = (int) agc_erase_m11();
    int m32 = (int) agc_erase_m32();

    CHECK(m11 > 11500 && m11 < 11700); /* .7071 x 2^14 = 11585  */
    CHECK(m32 > 16300);                /* COS 0 = 1 (POSMAX)    */
    CHECK(agc_erase_m22() == 0);       /* SIN 0                 */
}

/* RRDESEND (V44) outside continuous designate is just GOPIN: RADMODES and
 * NORRMON untouched.                                                      */
static void test_radar_verbs(void)
{
    agc_job_ctx_t ctx = {AGC_JOB_EXTENDED_VERB, 030, false, -1, 0};

    agc_state_init(AGC_VEHICLE_LM);
    agc_erase_set_radmodes(01000u); /* designate, not continuous */
    agc_flag_set(AGC_FLAG_NORRMON);
    CHECK(agc_extverb_rr_designate_end(&ctx) == AGC_STATUS_OK);
    CHECK(agc_erase_radmodes() == 01000u);
    CHECK(agc_flag_test(AGC_FLAG_NORRMON));
    /* after the 1 s delay of a real RRDESEND: R25 back on, GOPIN          */
    CHECK(agc_job_rr_designate_end_monitor(&ctx) == AGC_STATUS_OK);
    CHECK(!agc_flag_test(AGC_FLAG_NORRMON));
    agc_erase_set_rsampdt(0);
    CHECK(agc_erase_rsampdt() == 0);
}

int main(void)
{
    test_downlist_word_counts();
    test_downrupt_sequence();
    test_loadtime();
    test_sgnagree();
    test_banksum();
    test_service();
    test_t4rupt_gimbal_matrix();
    test_radar_verbs();
    (void) printf(
        "test_sysmisc: %d checks, %d failures, %d expected assert reports\n",
        g_checks, g_failures, g_assert_reports);

    return ((g_failures == 0) ? 0 : 1);
}
