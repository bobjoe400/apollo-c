/**********************************************************************
 * FILE NAME: dsky.c
 *
 * PURPOSE:
 *   io-pinball unit tests: keyboard verb/noun entry, decimal and octal
 *   loads, the relay words of DSPOUT, the alarm light, the monitor, NVSUB
 *   interlocks, the uplink check, HMSIN, machine-address nouns, V35,
 *   CLEANDSP without a continuation and V31 with a zero delay (POODOO
 *   1204 from WAITLIST). Links against the whole library
 *   (PINBALL needs the noun tables, the executive/waitlist and the state
 *   accessors).
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pp. 310-389,
 *   KEYRUPT_UPRUPT.agc pp. 150-153, T4RUPT_PROGRAM.agc pp. 133-135,
 *   DISPLAY_INTERFACE_ROUTINES.agc pp. 1474-1490; Luminary099 equivalents
 *
 * EXTERNAL VARIABLES:
 *   none (test counters are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_dsky_keys ...     the pinball keyboard/display API under test
 *   agc_exec_* / waitlist executive the jobs run under
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails. AGC_ASSERT failures are printed
 *   and counted (none are expected).
 **********************************************************************/
#include <stdio.h>

#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/erasable_map/address_map.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"

static int g_checks;   /* checks run                        */
static int g_failures; /* checks failed                     */
static int g_asserts;  /* AGC_ASSERT failures reported      */

/* Prototypes: main() comes first (STYLE 4.6). */
static void             check(bool ok, int line, const char *text);
static void             fresh(agc_vehicle_t veh);
static agc_dsky_state_t state(void);
static bool             reg_is(const agc_dsky_state_t *s, int r, long int n, int sgn);
static bool             reg_blank(const agc_dsky_state_t *s, int r);
static void             test_display_clock_time(agc_vehicle_t veh);
static void             test_relay_words(void);
static void             test_quikdsp_without_dskyflag(void);
static void             test_load_component_one(void);
static void             test_load_three_components(void);
static void             test_clear(void);
static void             test_alarm(void);
static void             test_monitor(void);
static void             test_nvsub(void);
static void             test_uprupt(void);
static void             test_hmsin(void);
static void             test_mctbs(void);
static void             test_lights(void);
static void             test_cleandsp_no_continuation(void);
static void             test_request_waitlist_zero_delay(void);
static void             test_display_machine_address(void);
static void             test_change_program_refused(void);

/**********************************************************************
 * AGC: none (test driver) -- run every test of this file
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    test_display_clock_time(AGC_VEHICLE_CSM);
    test_display_clock_time(AGC_VEHICLE_LM);
    test_relay_words();
    test_quikdsp_without_dskyflag();
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
    test_cleandsp_no_continuation();
    test_display_machine_address();
    test_request_waitlist_zero_delay();
    test_change_program_refused();
    (void) printf("test_dsky: %d checks, %d failures, %d assertion reports\n", g_checks, g_failures, g_asserts);

    return ((g_failures == 0) ? 0 : 1); /* exit 1 on any failure */
}

/**********************************************************************
 * AGC: none (test hook replacing core/assert.c) -- print and count
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description
 *   --------  -------------  ---  ---------------------------------
 *   file      const char *   I    source file of the failed assertion
 *   line      int            I    source line
 *   expr      const char *   I    assertion text
 *
 * RETURNS: bool -- false (the assertion stays failed)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    g_asserts++;
    (void) printf("assert %s:%d: %s\n", file, line, expr);

    return (false);
}

#define CHECK(c) check((c), __LINE__, #c)

/**********************************************************************
 * AGC: none (test helper) -- count one check, print a failure
 *
 * FUNCTION NAME: check
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description
 *   --------  -------------  ---  ---------------------------------
 *   ok        bool           I    check result
 *   line      int            I    source line of the check
 *   text      const char *   I    check expression text
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
 * AGC: none (test helper) -- fresh start of state, executive, pinball
 *
 * FUNCTION NAME: fresh
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description
 *   --------  -------------  ---  ---------------------------------
 *   veh       agc_vehicle_t  I    vehicle to start
 **********************************************************************/
static void fresh(agc_vehicle_t veh)
{
    agc_state_init(veh);
    agc_exec_init();
    agc_waitlist_init();
    agc_dsky_init();
}

/**********************************************************************
 * AGC: none (test helper) -- the DSKY as the crew sees it
 *
 * FUNCTION NAME: state
 *
 * RETURNS: agc_dsky_state_t -- lamps and digits
 **********************************************************************/
static agc_dsky_state_t state(void)
{
    agc_dsky_state_t s; /* DSKY readout */

    CHECK(agc_dsky_get_state(&s) == AGC_STATUS_OK);

    return (s);
}

/**********************************************************************
 * AGC: none (test helper) -- register r (0..2) shows the 5 digits of
 *      n (decimal digits, sign sgn)
 *
 * FUNCTION NAME: reg_is
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description
 *   --------  ------------------------  ---  ----------------------
 *   s         const agc_dsky_state_t *  I    DSKY readout
 *   r         int                       I    register 0..2 (R1..R3)
 *   n         long int                  I    value shown, 0..99999
 *   sgn       int                       I    sign: 1 +, -1 -, 0 none
 *
 * RETURNS: bool -- true when the register shows exactly that
 **********************************************************************/
static bool reg_is(const agc_dsky_state_t *s, int r, long int n, int sgn)
{
    long int v = n; /* digits still to compare */

    for (int d = AGC_DSKY_RDIGITS - 1; d >= 0; d--)
    {
        if (s->r[r][d] != (int8_t) (v % 10))
        {
            return (false);
        }

        v /= 10;
    }

    return (s->r_sign[r] == sgn);
}

/**********************************************************************
 * AGC: none (test helper) -- every digit of register r is blank
 *
 * FUNCTION NAME: reg_blank
 *
 * ARGUMENTS:
 *   Argument  Type                      I/O  Description
 *   --------  ------------------------  ---  ----------------------
 *   s         const agc_dsky_state_t *  I    DSKY readout
 *   r         int                       I    register 0..2 (R1..R3)
 *
 * RETURNS: bool -- true when all 5 digits are blank
 **********************************************************************/
static bool reg_blank(const agc_dsky_state_t *s, int r)
{
    for (int d = 0; d < AGC_DSKY_RDIGITS; d++)
    {
        if (s->r[r][d] != AGC_DSKY_BLANK)
        {
            return (false);
        }
    }

    return (true);
}

/**********************************************************************
 * AGC: none (test) -- "V06N36E": verb 06 noun 36 shows the AGC clock
 *      (HMS) in R1-R3.
 *
 * FUNCTION NAME: test_display_clock_time
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description
 *   --------  -------------  ---  ---------------------------------
 *   veh       agc_vehicle_t  I    vehicle under test
 **********************************************************************/
static void test_display_clock_time(agc_vehicle_t veh)
{
    fresh(veh);
    agc_time_set_cs(372345u);     /* 1 h 2 min 3.45 s */
    CHECK(agc_dsky_keys("V06N36E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state(); /* DSKY readout */

    CHECK(s.verbreg == 6u && s.nounreg == 36u);
    CHECK(s.verb[0] == 0 && s.verb[1] == 6);
    CHECK(s.noun[0] == 3 && s.noun[1] == 6);
    CHECK(reg_is(&s, 0, 1, 1));
    CHECK(reg_is(&s, 1, 2, 1));
    CHECK(reg_is(&s, 2, 345, 1));
    CHECK(!s.flash && !s.opr_err);
}

/**********************************************************************
 * AGC: none (test) -- The relay words DSPOUT sends: AAAA (DSPTAB index +
 *      1), B, CCCCC, DDDDD.
 *
 * FUNCTION NAME: test_relay_words
 **********************************************************************/
static void test_relay_words(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_relay_code(0) == 025u && agc_dsky_relay_code(6) == 034u);
    CHECK(agc_dsky_relay_code(8) == 035u && agc_dsky_relay_digit(037u) == 9);
    CHECK(agc_dsky_keys("V06N36E") == AGC_STATUS_OK);
    bool     saw_decimal_display = false; /* V06 relay word seen */
    bool     noun_clock_time     = false; /* N36 relay word seen */
    int      sent                = 0;     /* relay words sent */
    uint16_t w                   = 0;     /* channel 10 word */

    for (int i = 0; i < 40 && agc_dsky_dspout(&w) == AGC_STATUS_OK; i++)
    {
        sent++;
        saw_decimal_display = saw_decimal_display || w == (uint16_t) ((10u << 11) | (025u << 5) | 034u);
        noun_clock_time     = noun_clock_time || w == (uint16_t) ((9u << 11) | (033u << 5) | 034u);
        CHECK(agc_chan_read((int) AGC_CHAN_OUT0) == w);
    }

    CHECK(saw_decimal_display && noun_clock_time);
    CHECK(sent >= 2 && agc_dsky_nout() == 0);
    CHECK(agc_dsky_dspout(&w) == AGC_STATUS_NOT_FOUND);
}

/**********************************************************************
 * AGC: none (test) -- QUIKDSP (T4RUPT_PROGRAM C p. 137) sends pending
 *      DSPTAB words without DSKYFLAG; DSPOUT (p. 135) does not (sim B1)
 *
 * FUNCTION NAME: test_quikdsp_without_dskyflag
 **********************************************************************/
static void test_quikdsp_without_dskyflag(void)
{
    uint16_t w = 0; /* channel 10 word */

    fresh(AGC_VEHICLE_CSM);
    agc_flag_clear((agc_flag_id_t) AGC_FLAG_DSKYFLAG);
    CHECK(agc_dsky_nout() == 11); /* DSPOFF blanks pending */
    CHECK(agc_dsky_dspout(&w) == AGC_STATUS_NOT_FOUND && w == 0u);
    CHECK(agc_dsky_dspoutsb(&w) == AGC_STATUS_OK && w != 0u);
    CHECK(agc_dsky_nout() == 10);
    CHECK(agc_chan_read((int) AGC_CHAN_OUT0) == w);

    for (int i = 0; i < 20 && agc_dsky_dspoutsb(&w) == AGC_STATUS_OK; i++)
    {
        CHECK(w != 0u);
    }

    CHECK(agc_dsky_nout() == 0);
    CHECK(agc_dsky_dspoutsb(&w) == AGC_STATUS_NOT_FOUND && w == 0u);
}

/**********************************************************************
 * AGC: none (test) -- "V21N01E12345E": machine address requested in R3,
 *      then the load.
 *
 * FUNCTION NAME: test_load_component_one
 **********************************************************************/
static void test_load_component_one(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V21N01E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state(); /* DSKY readout */

    CHECK(s.flash && reg_blank(&s, 2));            /* REQDATZ  */
    CHECK(agc_dsky_keys("12345E") == AGC_STATUS_OK);
    s             = state();
    CHECK(reg_is(&s, 2, 12345, 0));                /* address  */
    CHECK(s.flash && reg_blank(&s, 0));            /* REQDATX  */
    CHECK(s.verb[0] == 2 && s.verb[1] == 1);
    CHECK(agc_dsky_keys("777E") == AGC_STATUS_OK); /* octal    */
    uint16_t word = 0;                             /* erasable read back */

    agc_erase_read_ecadr(012345u, &word);
    CHECK(word == 0777u);
    s = state();
    CHECK(!s.flash && !s.opr_err);
}

/**********************************************************************
 * AGC: none (test) -- V25N22: decimal CDU degrees in, the same angles
 *      shown by V06N22.
 *
 * FUNCTION NAME: test_load_three_components
 **********************************************************************/
static void test_load_three_components(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V25N22E+18000E+09000E-04500E") == AGC_STATUS_OK);
    CHECK(agc_erase_thetad(0) == 040000u); /* 180 deg  */
    CHECK(agc_erase_thetad(1) == 020000u); /*  90 deg  */
    CHECK(agc_erase_thetad(2) == 070000u); /* -45 deg  */
    CHECK(agc_dsky_keys("V06N22E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state();          /* DSKY readout */

    CHECK(reg_is(&s, 0, 18000, 1));
    CHECK(reg_is(&s, 1, 9000, 1));
    CHECK(reg_is(&s, 2, 31500, 1));

    /* 360.00 overflows and alarms; the load is recycled (V25 again).    */
    CHECK(agc_dsky_keys("V25N22E+36000E+00000E+00000E") == AGC_STATUS_OK);
    s = state();
    CHECK(s.opr_err && s.flash && s.verbreg == 21u);
}

/**********************************************************************
 * AGC: none (test) -- CLEAR: pass 0 blanks the register, the next pass
 *      backs the load up.
 *
 * FUNCTION NAME: test_clear
 **********************************************************************/
static void test_clear(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V25N22E+18000E+1") == AGC_STATUS_OK);
    agc_dsky_state_t s = state(); /* DSKY readout */

    CHECK(s.verbreg == 22u && s.r[1][0] == 1);
    CHECK(agc_dsky_keys("C") == AGC_STATUS_OK);
    s = state();
    CHECK(reg_blank(&s, 1) && s.verbreg == 22u);
    CHECK(agc_dsky_keys("C") == AGC_STATUS_OK);       /* CLPASHI  */
    s = state();
    CHECK(reg_blank(&s, 0) && s.verbreg == 21u && s.flash);
    CHECK(agc_dsky_keys("+00001E") == AGC_STATUS_OK); /* R1 again */
    s = state();
    CHECK(s.verbreg == 22u);
}

/**********************************************************************
 * AGC: none (test) -- Operator error and its reset; a decimal entry of
 *      fewer than 5 digits.
 *
 * FUNCTION NAME: test_alarm
 **********************************************************************/
static void test_alarm(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V08E") == AGC_STATUS_OK);         /* spare    */
    CHECK(state().opr_err);
    CHECK(agc_dsky_keys("R") == AGC_STATUS_OK);            /* RSET     */
    CHECK(!state().opr_err);
    CHECK(agc_dsky_keys("V21N22E+123E") == AGC_STATUS_OK); /* 3 digits */
    agc_dsky_state_t s = state();                          /* DSKY readout */

    CHECK(s.opr_err && s.flash);                /* REQRET kept */
    CHECK(agc_dsky_keys("45E") == AGC_STATUS_OK);
    CHECK(agc_erase_thetad(0) != 0u);
    CHECK(!state().flash);
    CHECK(agc_dsky_keys("8") == AGC_STATUS_OK); /* blocked  */
}

/**********************************************************************
 * AGC: none (test) -- Monitor V16N36: MONREQ (waitlist) and MONDO (job)
 *      refresh the display and paste the monitor verb back.
 *
 * FUNCTION NAME: test_monitor
 **********************************************************************/
static void test_monitor(void)
{
    fresh(AGC_VEHICLE_CSM);
    agc_time_set_cs(6000u); /* 1 minute */
    CHECK(agc_dsky_keys("V16N36E") == AGC_STATUS_OK);
    agc_inhint();
    CHECK(agc_task_monreq() == AGC_STATUS_OK);
    agc_relint();

    CHECK(agc_job_mondo() == AGC_STATUS_OK);
    agc_dsky_state_t s = state(); /* DSKY readout */

    CHECK(s.verb[0] == 1 && s.verb[1] == 6 && s.verbreg == 16u);
    CHECK(reg_is(&s, 1, 1, 1) && reg_is(&s, 0, 0, 1));

    /* A keyed monitor blocks NVSUB (external monitor bit) until KEY REL. */
    CHECK(agc_dsky_nvsub(6, 36) == AGC_STATUS_BUSY);
    CHECK(agc_dsky_keys("K") == AGC_STATUS_OK);
    CHECK(agc_dsky_nvsub(6, 36) == AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (test) -- NVSUB is refused while the operator uses the
 *      keyboard.
 *
 * FUNCTION NAME: test_nvsub
 **********************************************************************/
static void test_nvsub(void)
{
    fresh(AGC_VEHICLE_LM);
    CHECK(agc_dsky_nvsub(6, 36) == AGC_STATUS_OK);
    agc_dsky_state_t s = state(); /* DSKY readout */

    CHECK(s.verbreg == 6u && s.nounreg == 36u && reg_is(&s, 0, 0, 1));
    CHECK(agc_dsky_keys("V") == AGC_STATUS_OK); /* DSPLOCK  */
    CHECK(agc_dsky_nvsub(6, 36) == AGC_STATUS_BUSY);
    CHECK(agc_dsky_keys("K") == AGC_STATUS_OK); /* KEY REL  */
    CHECK(agc_dsky_nvsub_blank(-1) == AGC_STATUS_OK);
    s = state();
    CHECK(reg_blank(&s, 0) && reg_blank(&s, 1) && reg_blank(&s, 2));
    CHECK(s.noun[0] == 3 && s.noun[1] == 6);
}

/**********************************************************************
 * AGC: none (test) -- UPRUPT: C, NOT-C, C; a bad word locks out uplink
 *      until error reset.
 *
 * FUNCTION NAME: test_uprupt
 **********************************************************************/
static void test_uprupt(void)
{
    fresh(AGC_VEHICLE_CSM);
    uint16_t c    = 021u;                                            /* VERB key code */
    uint16_t good = (uint16_t) (c | ((~c & 037u) << 5) | (c << 10)); /* CNC */

    CHECK(agc_dsky_uprupt(good) == AGC_STATUS_OK);
    CHECK(!agc_flag_test((agc_flag_id_t) AGC_FLAG_UPLOCKFL));
    CHECK(state().uplink_act);
    CHECK(agc_dsky_uprupt((uint16_t) (good ^ 0100u)) == AGC_STATUS_OK);
    CHECK(agc_flag_test((agc_flag_id_t) AGC_FLAG_UPLOCKFL));
    uint16_t e = 022u; /* ERROR RESET */

    CHECK(agc_dsky_uprupt((uint16_t) (e | ((~e & 037u) << 5) | (e << 10))) == AGC_STATUS_OK);
    CHECK(!agc_flag_test((agc_flag_id_t) AGC_FLAG_UPLOCKFL));
}

/**********************************************************************
 * AGC: none (test) -- V25N36: hours, minutes, seconds loaded into the
 *      clock (HMSIN); a minutes value over 59 alarms and recycles.
 *
 * FUNCTION NAME: test_hmsin
 **********************************************************************/
static void test_hmsin(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V25N36E+00001E+00002E+00345E") == AGC_STATUS_OK);
    CHECK(agc_time_now_cs() == 372345u);
    CHECK(!state().opr_err);
    CHECK(agc_dsky_keys("V25N36E+00001E+00060E+00000E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state(); /* DSKY readout */

    CHECK(s.opr_err && s.flash && s.verbreg == 21u);
    CHECK(agc_time_now_cs() == 372345u);
    CHECK(agc_dsky_keys("V21N36E+00001E") == AGC_STATUS_OK); /* not all 3 */
    CHECK(state().verbreg == 21u && state().flash);
}

/**********************************************************************
 * AGC: none (test) -- V01N01: octal display of a machine address; V25N07:
 *      set flag bits.
 *
 * FUNCTION NAME: test_mctbs
 **********************************************************************/
static void test_mctbs(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V21N01E1000E12345E") == AGC_STATUS_OK);
    CHECK(agc_dsky_keys("V01N01E1000E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state(); /* DSKY readout */

    CHECK(reg_is(&s, 0, 12345, 0));
    CHECK(s.r[2][0] == 1 && s.r[2][3] == 0 && s.r[2][4] == AGC_DSKY_BLANK); /* as keyed */
    CHECK(agc_dsky_keys("V25N07E1000E70E1E") == AGC_STATUS_OK);             /* set   */
    uint16_t w = 0;                                                         /* flag word read back */

    agc_erase_read_ecadr(01000u, &w);
    CHECK(w == 012375u);
    CHECK(agc_dsky_keys("V25N07E1000E5E0E") == AGC_STATUS_OK); /* reset */
    agc_erase_read_ecadr(01000u, &w);
    CHECK(w == 012370u);
}

/**********************************************************************
 * AGC: none (test) -- V35: all 8's and the + signs; TSTLTS3 (after 5 s)
 *      puts MODREG back.
 *
 * FUNCTION NAME: test_lights
 **********************************************************************/
static void test_lights(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V35E") == AGC_STATUS_OK);
    agc_dsky_state_t s = state(); /* DSKY readout */

    CHECK(reg_is(&s, 0, 88888, 1) && reg_is(&s, 1, 88888, 1) && reg_is(&s, 2, 88888, 1));
    CHECK(s.prog[0] == 8 && s.verb[1] == 8 && s.flash && s.opr_err && s.key_rel);

    CHECK(agc_job_tstlts3() == AGC_STATUS_OK);
    s = state();
    CHECK(!s.flash && !s.opr_err && !s.key_rel);
}

/**********************************************************************
 * AGC: none (test) -- CLEANDSP with no continuation (P40BLNKR, BLANKDEX):
 *      the clean-out runs in a MAKEPLAY job of its own and the caller's
 *      job goes on (no sleep).
 *
 * FUNCTION NAME: test_cleandsp_no_continuation
 **********************************************************************/
static void test_cleandsp_no_continuation(void)
{
    fresh(AGC_VEHICLE_LM);
    agc_job_ctx_t ctx = {AGC_JOB_CHARIN, 020, false, -1, 0};

    CHECK(agc_dsky_display(&ctx, AGC_DSKY_CLEANDSP, 0u, NULL) == AGC_STATUS_OK);
    int makeplay = 0; /* MAKEPLAY jobs found */

    for (int c = 0; c < agc_exec_core_sets(); c++)
    {
        agc_exec_core_info_t ci; /* core set c */

        agc_exec_core_info(c, &ci);
        CHECK(ci.state != AGC_CORE_SLEEPING);
        makeplay += (ci.state == AGC_CORE_ACTIVE && ci.loc == AGC_JOB_MAKEPLAY) ? 1 : 0; /* count jobs */
    }

    CHECK(makeplay == 1);
}

/**********************************************************************
 * AGC: none (test) -- V25N26 then V31 with a zero delay: VBRQWAIT (CM p. 366)
 *      passes DSPTEM1 to WAITLIST, which gives POODOO 1204 for a zero
 *      delta time (WAITLIST C p. 1225).
 *
 * FUNCTION NAME: test_request_waitlist_zero_delay
 **********************************************************************/
static void test_request_waitlist_zero_delay(void)
{
    fresh(AGC_VEHICLE_CSM);
    CHECK(agc_dsky_keys("V25N26E0E1E0E") == AGC_STATUS_OK); /* DSPTEM1: delay 0, task 1, BBCON 0 */
    CHECK(agc_dsky_dsptem(0) == 0u && agc_dsky_dsptem(1) == 1u);
    CHECK(agc_dsky_keys("V31E") == AGC_STATUS_POODOO);
    CHECK(agc_erase_failreg(0) == 01204u);
}

/**********************************************************************
 * AGC: none (test) -- a program's V05N01 through the display interface
 *      with MPAC+2 = 0300: NVSUB's internal machine-address noun takes
 *      the ECADR from MPAC+2 (INTMCTBS, CM p. 366) and R1 shows the word
 *      there (the path P27's V21N01 and V91's V05N01 use).
 *
 * FUNCTION NAME: test_display_machine_address
 **********************************************************************/
static void test_display_machine_address(void)
{
    fresh(AGC_VEHICLE_CSM);
    agc_job_ctx_t ctx = {AGC_JOB_CHARIN, 020, false, -1, 0};

    CHECK(agc_erase_write_ecadr(0300u, 012345u) == AGC_STATUS_OK);
    CHECK(agc_dsky_display_ecadr(&ctx, AGC_DSKY_GODSPR, agc_dsky_vn(5, 1), NULL, 0300u) == AGC_STATUS_OK);

    for (int i = 0; i < 20 && agc_exec_run_one() != AGC_STATUS_NOT_FOUND; i++)
    {
        /* MAKEPLAY and what it starts */
    }

    agc_dsky_state_t s = state(); /* DSKY readout */

    CHECK(s.verb[0] == 0 && s.verb[1] == 5 && s.noun[0] == 0 && s.noun[1] == 1);
    CHECK(reg_is(&s, 0, 12345, 0));
}

/**********************************************************************
 * AGC: none (test) -- V37E00E while the IMU is being initialised
 *      (IMODES30 bit 6): CANTR00 / V37BAD (FRESH_START_AND_RESTART.agc
 *      L pp. 227-228) raise alarm 1520, release the display and end the
 *      job. The program alarm is not an operator error: ENTER does not
 *      run again (ALMCYCLE is reached only through pinball's own TCs),
 *      so OPR ERR stays off (closed-loop-fixes L1).
 *
 * FUNCTION NAME: test_change_program_refused
 **********************************************************************/
static void test_change_program_refused(void)
{
    fresh(AGC_VEHICLE_LM);
    agc_erase_set_imodes30(000040u);                     /* bit 6: IMU being initialised */
    CHECK(agc_dsky_keys("V37E00E") == AGC_STATUS_ALARM); /* V37BAD */
    agc_dsky_state_t s = state();                        /* DSKY readout */

    CHECK(agc_erase_failreg(0) == 01520u);
    CHECK(!s.opr_err);
    CHECK(!s.flash);
}
