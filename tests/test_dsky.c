/*
 * test_dsky.c -- io-pinball unit tests: keyboard verb/noun entry, decimal
 * and octal loads, the relay words of DSPOUT, the alarm light, the
 * monitor, NVSUB interlocks and the uplink check.
 *
 * Links against the whole library (PINBALL needs the noun tables, the
 * executive/waitlist and the state accessors). The accessors pinball has
 * requested from other lanes are declared here as pinball declares them.
 */
#include <stdio.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_dsky.h"
#include "agc/agc_exec.h"
#include "agc/agc_state.h"
#include "agc/agc_channels.h"
#include "agc/agc_flags_common.h"

/* requested from state-flags (see docs/modules/io-pinball.md)            */
agc_status_t agc_erase_read_ecadr(uint16_t ecadr, uint16_t *word);

static int g_checks;
static int g_failures;
static int g_asserts;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    g_asserts++;
    (void)printf("assert %s:%d: %s\n", file, line, expr);
    return false;
}

#define CHECK(c) check((c), __LINE__, #c)

static void check(bool ok, int line, const char *text)
{
    g_checks++;
    if (!ok) {
        g_failures++;
        (void)printf("FAIL line %d: %s\n", line, text);
    }
}

static void fresh(agc_vehicle_t veh)
{
    agc_state_init(veh);
    CHECK(agc_exec_init() == AGC_STATUS_OK);
    CHECK(agc_waitlist_init() == AGC_STATUS_OK);
    CHECK(agc_dsky_init() == AGC_STATUS_OK);
}

static agc_dsky_state_t state(void)
{
    agc_dsky_state_t s;
    CHECK(agc_dsky_get_state(&s) == AGC_STATUS_OK);
    return s;
}

/* Register r (0..2) shows the 5 digits of n (decimal digits, sign sgn). */
static bool reg_is(const agc_dsky_state_t *s, int r, long n, int sgn)
{
    long v = n;
    for (int d = AGC_DSKY_RDIGITS - 1; d >= 0; d--) {
        if (s->r[r][d] != (int8_t)(v % 10)) {
            return false;
        }
        v /= 10;
    }
    return s->r_sign[r] == sgn;
}

static bool reg_blank(const agc_dsky_state_t *s, int r)
{
    for (int d = 0; d < AGC_DSKY_RDIGITS; d++) {
        if (s->r[r][d] != AGC_DSKY_BLANK) {
            return false;
        }
    }
    return true;
}

/* "V06N36E": verb 06 noun 36 shows the AGC clock (HMS) in R1-R3. */
static void test_display_clock_time(agc_vehicle_t veh)
{
    fresh(veh);
    CHECK(agc_time_set_cs(372345u) == AGC_STATUS_OK);       /* 1 h 2 min 3.45 s */
    CHECK(agc_dsky_keys("V06N36E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state();
    CHECK(s.verbreg == 6u && s.nounreg == 36u);
    CHECK(s.verb[0] == 0 && s.verb[1] == 6);
    CHECK(s.noun[0] == 3 && s.noun[1] == 6);
    CHECK(reg_is(&s, 0, 1, 1));
    CHECK(reg_is(&s, 1, 2, 1));
    CHECK(reg_is(&s, 2, 345, 1));
    CHECK(!s.flash && !s.opr_err);
}

/* The relay words DSPOUT sends: AAAA (DSPTAB index + 1), B, CCCCC, DDDDD. */
static void test_relay_words(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_relay_code(0) == 025u && agc_dsky_relay_code(6) == 034u);
    CHECK(agc_dsky_relay_code(8) == 035u && agc_dsky_relay_digit(037u) == 9);
    CHECK(agc_dsky_keys("V06N36E") == AGC_STATUS_OK);
    bool saw_decimal_display = false;
    bool noun_clock_time = false;
    int sent = 0;
    uint16_t w = 0;
    for (int i = 0; i < 40 && agc_dsky_dspout(&w) == AGC_STATUS_OK; i++) {
        sent++;
        saw_decimal_display = saw_decimal_display || w == (uint16_t)((10u << 11) | (025u << 5) | 034u);
        noun_clock_time = noun_clock_time || w == (uint16_t)((9u << 11) | (033u << 5) | 034u);
        CHECK(agc_chan_read((int)AGC_CHAN_OUT0) == w);
    }
    CHECK(saw_decimal_display && noun_clock_time);
    CHECK(sent >= 2 && agc_dsky_nout() == 0);
    CHECK(agc_dsky_dspout(&w) == AGC_STATUS_NOT_FOUND);
}

/* "V21N01E12345E": machine address requested in R3, then the load. */
static void test_load_component_one(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V21N01E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state();
    CHECK(s.flash && reg_blank(&s, 2));                      /* REQDATZ  */
    CHECK(agc_dsky_keys("12345E") == AGC_STATUS_OK);
    s = state();
    CHECK(reg_is(&s, 2, 12345, 0));                          /* address  */
    CHECK(s.flash && reg_blank(&s, 0));                      /* REQDATX  */
    CHECK(s.verb[0] == 2 && s.verb[1] == 1);
    CHECK(agc_dsky_keys("777E") == AGC_STATUS_OK);           /* octal    */
    uint16_t word = 0;
    CHECK(agc_erase_read_ecadr(012345u, &word) == AGC_STATUS_OK && word == 0777u);
    s = state();
    CHECK(!s.flash && !s.opr_err);
}

/* V25N22: decimal CDU degrees in, the same angles shown by V06N22. */
static void test_load_three_components(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V25N22E+18000E+09000E-04500E") == AGC_STATUS_OK);
    CHECK(agc_erase_thetad(0) == 040000u);                   /* 180 deg  */
    CHECK(agc_erase_thetad(1) == 020000u);                   /*  90 deg  */
    CHECK(agc_erase_thetad(2) == 070000u);                   /* -45 deg  */
    CHECK(agc_dsky_keys("V06N22E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state();
    CHECK(reg_is(&s, 0, 18000, 1));
    CHECK(reg_is(&s, 1, 9000, 1));
    CHECK(reg_is(&s, 2, 31500, 1));
    /* 360.00 overflows and alarms; the load is recycled (V25 again).    */
    CHECK(agc_dsky_keys("V25N22E+36000E+00000E+00000E") == AGC_STATUS_OK);
    s = state();
    CHECK(s.opr_err && s.flash && s.verbreg == 21u);
}

/* CLEAR: pass 0 blanks the register, the next pass backs the load up. */
static void test_clear(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V25N22E+18000E+1") == AGC_STATUS_OK);
    agc_dsky_state_t s = state();
    CHECK(s.verbreg == 22u && s.r[1][0] == 1);
    CHECK(agc_dsky_keys("C") == AGC_STATUS_OK);
    s = state();
    CHECK(reg_blank(&s, 1) && s.verbreg == 22u);
    CHECK(agc_dsky_keys("C") == AGC_STATUS_OK);              /* CLPASHI  */
    s = state();
    CHECK(reg_blank(&s, 0) && s.verbreg == 21u && s.flash);
    CHECK(agc_dsky_keys("+00001E") == AGC_STATUS_OK);        /* R1 again */
    s = state();
    CHECK(s.verbreg == 22u);
}

/* Operator error and its reset; a decimal entry of fewer than 5 digits. */
static void test_alarm(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V08E") == AGC_STATUS_OK);           /* spare    */
    CHECK(state().opr_err);
    CHECK(agc_dsky_keys("R") == AGC_STATUS_OK);              /* RSET     */
    CHECK(!state().opr_err);
    CHECK(agc_dsky_keys("V21N22E+123E") == AGC_STATUS_OK);   /* 3 digits */
    agc_dsky_state_t s = state();
    CHECK(s.opr_err && s.flash);                             /* REQRET kept */
    CHECK(agc_dsky_keys("45E") == AGC_STATUS_OK);
    CHECK(agc_erase_thetad(0) != 0u);
    CHECK(!state().flash);
    CHECK(agc_dsky_keys("8") == AGC_STATUS_OK);              /* blocked  */
}

/* Monitor V16N36: MONREQ (waitlist) and MONDO (job) refresh the display
 * and paste the monitor verb back.                                       */
static void test_monitor(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_time_set_cs(6000u) == AGC_STATUS_OK);          /* 1 minute */
    CHECK(agc_dsky_keys("V16N36E") == AGC_STATUS_OK);
    agc_inhint();
    CHECK(agc_task_monreq() == AGC_STATUS_OK);
    agc_relint();
    agc_job_ctx_t ctx = { AGC_JOB_CHARIN, 030, false, -1, 0 };
    CHECK(agc_job_mondo(&ctx) == AGC_STATUS_OK);
    agc_dsky_state_t s = state();
    CHECK(s.verb[0] == 1 && s.verb[1] == 6 && s.verbreg == 16u);
    CHECK(reg_is(&s, 1, 1, 1) && reg_is(&s, 0, 0, 1));
    /* A keyed monitor blocks NVSUB (external monitor bit) until KEY REL. */
    CHECK(agc_dsky_nvsub(6, 36) == AGC_STATUS_BUSY);
    CHECK(agc_dsky_keys("K") == AGC_STATUS_OK);
    CHECK(agc_dsky_nvsub(6, 36) == AGC_STATUS_OK);
}

/* NVSUB is refused while the operator uses the keyboard. */
static void test_nvsub(void)
{
    fresh(AGC_VEHICLE_LM);
    CHECK(agc_dsky_nvsub(6, 36) == AGC_STATUS_OK);
    agc_dsky_state_t s = state();
    CHECK(s.verbreg == 6u && s.nounreg == 36u && reg_is(&s, 0, 0, 1));
    CHECK(agc_dsky_keys("V") == AGC_STATUS_OK);              /* DSPLOCK  */
    CHECK(agc_dsky_nvsub(6, 36) == AGC_STATUS_BUSY);
    CHECK(agc_dsky_keys("K") == AGC_STATUS_OK);              /* KEY REL  */
    CHECK(agc_dsky_nvsub_blank(-1) == AGC_STATUS_OK);
    s = state();
    CHECK(reg_blank(&s, 0) && reg_blank(&s, 1) && reg_blank(&s, 2));
    CHECK(s.noun[0] == 3 && s.noun[1] == 6);
}

/* UPRUPT: C, NOT-C, C; a bad word locks out uplink until error reset. */
static void test_uprupt(void)
{
    fresh(AGC_VEHICLE_CSM);
    uint16_t c = 021u;                                       /* VERB     */
    uint16_t good = (uint16_t)(c | ((~c & 037u) << 5) | (c << 10));
    CHECK(agc_dsky_uprupt(good) == AGC_STATUS_OK);
    CHECK(!agc_flag_test((agc_flag_id_t)AGC_FLAG_UPLOCKFL));
    CHECK(state().uplink_act);
    CHECK(agc_dsky_uprupt((uint16_t)(good ^ 0100u)) == AGC_STATUS_OK);
    CHECK(agc_flag_test((agc_flag_id_t)AGC_FLAG_UPLOCKFL));
    uint16_t e = 022u;                                       /* ERROR RESET */
    CHECK(agc_dsky_uprupt((uint16_t)(e | ((~e & 037u) << 5) | (e << 10))) == AGC_STATUS_OK);
    CHECK(!agc_flag_test((agc_flag_id_t)AGC_FLAG_UPLOCKFL));
}

/* V25N36: hours, minutes, seconds loaded into the clock (HMSIN); a
 * minutes value over 59 alarms and recycles.                             */
static void test_hmsin(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V25N36E+00001E+00002E+00345E") == AGC_STATUS_OK);
    CHECK(agc_time_now_cs() == 372345u);
    CHECK(!state().opr_err);
    CHECK(agc_dsky_keys("V25N36E+00001E+00060E+00000E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state();
    CHECK(s.opr_err && s.flash && s.verbreg == 21u);
    CHECK(agc_time_now_cs() == 372345u);
    CHECK(agc_dsky_keys("V21N36E+00001E") == AGC_STATUS_OK);    /* not all 3 */
    CHECK(state().verbreg == 21u && state().flash);
}

/* V01N01: octal display of a machine address; V25N07: set flag bits. */
static void test_mctbs(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V21N01E1000E12345E") == AGC_STATUS_OK);
    CHECK(agc_dsky_keys("V01N01E1000E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state();
    CHECK(reg_is(&s, 0, 12345, 0));
    CHECK(s.r[2][0] == 1 && s.r[2][3] == 0 && s.r[2][4] == AGC_DSKY_BLANK); /* as keyed */
    CHECK(agc_dsky_keys("V25N07E1000E70E1E") == AGC_STATUS_OK);   /* set   */
    uint16_t w = 0;
    CHECK(agc_erase_read_ecadr(01000u, &w) == AGC_STATUS_OK && w == 012375u);
    CHECK(agc_dsky_keys("V25N07E1000E5E0E") == AGC_STATUS_OK);    /* reset */
    CHECK(agc_erase_read_ecadr(01000u, &w) == AGC_STATUS_OK && w == 012370u);
}

/* V35: all 8's and the + signs; TSTLTS3 (after 5 s) puts MODREG back. */
static void test_lights(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V35E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state();
    CHECK(reg_is(&s, 0, 88888, 1) && reg_is(&s, 1, 88888, 1) && reg_is(&s, 2, 88888, 1));
    CHECK(s.prog[0] == 8 && s.verb[1] == 8 && s.flash && s.opr_err && s.key_rel);
    agc_job_ctx_t ctx = { AGC_JOB_CHARIN, 030, false, -1, 0 };
    CHECK(agc_job_tstlts3(&ctx) == AGC_STATUS_OK);
    s = state();
    CHECK(!s.flash && !s.opr_err && !s.key_rel);
}

int main(void)
{
    test_display_clock_time(AGC_VEHICLE_CSM);
    test_display_clock_time(AGC_VEHICLE_LM);
    test_relay_words();
    test_load_component_one();
    test_load_three_components();
    test_clear();
    test_alarm();
    test_monitor();
    test_nvsub();
    test_uprupt();
    test_hmsin();
    test_mctbs();
    test_lights();
    (void)printf("test_dsky: %d checks, %d failures, %d assertion reports\n",
                 g_checks, g_failures, g_asserts);
    return (g_failures == 0) ? 0 : 1;
}
