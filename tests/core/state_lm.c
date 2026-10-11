/**********************************************************************
 * FILE NAME: state_lm.c
 *
 * PURPOSE:
 *   state-flags unit tests, LM build (Luminary099 flag names). The CSM
 *   and LM flag headers cannot share a translation unit, hence this
 *   second test binary; see tests/core/state.c for the conventions.
 *
 * AGC SOURCE:
 *   Luminary099 FLAGWORD_ASSIGNMENTS.agc pp. 61-88 (flag numbers nnnD,
 *   RADMODES = FLGWRD12, DAPBOOLS = FLGWRD13);
 *   INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc pp. 54-59
 *
 * EXTERNAL VARIABLES:
 *   none (test counters are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_state_init ...    the state, channel and erasable API under test
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails. AGC_ASSERT failures are expected
 *   in the negative tests; agc_assert_report() here ignores them.
 **********************************************************************/
#include <stdio.h>

#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/state.h"

static int g_checks;   /* checks run    */
static int g_failures; /* checks failed */

/* Prototypes: main() comes first (STYLE 4.6). */
static void check(bool ok, int line, const char *text);
static int  agc_dnum(agc_flag_id_t f);
static void test_lm_flag_ids(void);
static void test_lm_flag_ops(void);
static void test_lm_state(void);

/**********************************************************************
 * AGC: none (test driver) -- run every test of this file
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    test_lm_flag_ids();
    test_lm_flag_ops();
    test_lm_state();
    (void) printf("test_state_lm: %d checks, %d failures\n", g_checks, g_failures);

    return ((g_failures == 0) ? 0 : 1); /* exit 1 on any failure */
}

/**********************************************************************
 * AGC: none (test hook replacing core/assert.c)
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
    (void) file;
    (void) line;
    (void) expr;

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
 * AGC: none (test helper) -- AGC decimal flag number nnnD = 15 * flagword
 *      + (15 - bit)
 *
 * FUNCTION NAME: agc_dnum
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id
 *
 * RETURNS: int -- nnnD as printed in the AGC listing
 **********************************************************************/
static int agc_dnum(agc_flag_id_t f)
{
    return ((15 * agc_flag_word(f)) + (15 - agc_flag_bit(f)));
}

/**********************************************************************
 * AGC: none (test) -- LM flag ids against the decimal numbers
 *      (FLAGWORD_ASSIGNMENTS.agc L pp. 61-88)
 *
 * FUNCTION NAME: test_lm_flag_ids
 **********************************************************************/
static void test_lm_flag_ids(void)
{
    /* Numbers from Luminary099/FLAGWORD_ASSIGNMENTS.agc */
    agc_state_init(AGC_VEHICLE_LM);
    CHECK(agc_dnum(AGC_FLAG_MOONFLAG) == 3);
    CHECK(agc_dnum(AGC_FLAG_GROUND_TRACK_BASE_VECTORS) == 4); /* differs from CSM (033) */
    CHECK(agc_dnum(AGC_FLAG_NEEDLFLG) == 11);                 /* differs from CSM (006) */
    CHECK(agc_dnum(AGC_FLAG_AVEGFLAG) == 115);                /* differs from CSM (029) */
    CHECK(agc_dnum(AGC_FLAG_ITSWICH) == 105);                 /* differs from CSM (106) */
    CHECK(agc_dnum(AGC_FLAG_ORBIT_DISPLAY_MOON) == 118);      /* differs from CSM (137) */
    CHECK(agc_dnum(AGC_FLAG_IDLEFLAG) == 113);
    CHECK(agc_dnum(AGC_FLAG_APSFLAG) == 152);
    CHECK(agc_dnum(AGC_FLAG_LRBYPASS) == 165);
    CHECK(agc_dnum(AGC_FLAG_CDESFLAG) == 180);                /* FLGWRD12 (was RADMODES) */
    CHECK(agc_dnum(AGC_FLAG_PULSEFLG) == 195);                /* FLGWRD13 (was DAPBOOLS) */
    CHECK(agc_dnum(AGC_FLAG_AUTR1FLG) == 209);
    CHECK((int) AGC_FLAG_PULSEFLG == 0xDF);
    CHECK((int) AGC_FLAG_ETPIFLAG == (int) AGC_FLAG_OPTNSW);
    CHECK((int) AGC_FLAG_ALARM_521_SUPPRESSED == (int) AGC_FLAG_READRFLG);
    CHECK((int) AGC_FLAG_CMOONFLG == 0x8C);                   /* common header, FLAGWRD8 bit 12 */
}

/**********************************************************************
 * AGC: none (test) -- LM flagwords 12 and 13 (RADMODES, DAPBOOLS) exist
 *      only in the LM
 *
 * FUNCTION NAME: test_lm_flag_ops
 **********************************************************************/
static void test_lm_flag_ops(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    CHECK(agc_state_vehicle() == AGC_VEHICLE_LM);
    agc_flag_set(AGC_FLAG_PULSEFLG);                  /* word 13 exists on LM */
    CHECK(agc_flag_test(AGC_FLAG_PULSEFLG));
    CHECK(agc_flagword_get(13) == 0x4000u);
    agc_flag_invert(AGC_FLAG_PULSEFLG);
    CHECK(!agc_flag_test(AGC_FLAG_PULSEFLG));
    agc_flag_set(AGC_FLAG_TURNONFL);
    CHECK(agc_flagword_get(12) == 0x0001u);
    agc_flag_clear(AGC_FLAG_TURNONFL);
    agc_flagword_set(13, 077777u);
    CHECK(agc_flag_test(AGC_FLAG_AUTR1FLG));
    CHECK(agc_flagword_get(AGC_FLAGWORD_DAPBOOLS) == 077777u);
    agc_flagword_set(AGC_FLAGWORD_RADMODES, 000002u); /* one storage word */
    CHECK(agc_flagword_get(12) == 000002u);
    CHECK(agc_flag_valid(AGC_FLAG_AUTR1FLG));
    agc_state_init(AGC_VEHICLE_CSM);                  /* same id invalid on CSM */
    CHECK(!agc_flag_valid(AGC_FLAG_AUTR1FLG));
    agc_flag_set(AGC_FLAG_AUTR1FLG);                  /* rejected (assertion; flag word 12 has no AUTR1 on CSM) */
}

/**********************************************************************
 * AGC: none (test) -- LM OTHER state vector is the CSM set; LM channel
 *      names
 *
 * FUNCTION NAME: test_lm_state
 **********************************************************************/
static void test_lm_state(void)
{
    agc_state_vector_t in;  /* vector stored    */
    agc_state_vector_t out; /* vector read back */

    agc_state_init(AGC_VEHICLE_LM);
    in.r.c[0] = 11;
    in.r.c[1] = 22;
    in.r.c[2] = 33;
    in.v.c[0] = -1;
    in.v.c[1] = -2;
    in.v.c[2] = -3;
    in.t      = 4242;
    in.moon   = true;

    /* LM build: the OTHER vehicle is the CSM */
    CHECK(agc_sv_set(AGC_SV_OTHER, &in) == AGC_STATUS_OK);
    CHECK(agc_erase_rrectcsm().c[2] == 33);
    CHECK(agc_erase_vrectcsm().c[0] == -1);
    CHECK(agc_erase_tetcsm() == 4242);
    CHECK(agc_flag_test(AGC_FLAG_CMOONFLG));
    CHECK(!agc_flag_test(AGC_FLAG_LMOONFLG));
    CHECK(agc_sv_get(AGC_SV_OTHER, &out) == AGC_STATUS_OK);
    CHECK(out.moon && (out.t == 4242) && (out.r.c[1] == 22));
    CHECK(agc_sv_set(AGC_SV_THIS, &in) == AGC_STATUS_OK);
    CHECK(agc_erase_rn().c[0] == 11);
    CHECK(agc_flag_test(AGC_FLAG_MOONFLAG));

    /* LM channel bit names */
    CHECK(AGC_CH030_ABORTDPS == 000001);
    CHECK(AGC_CH030_ENGARMED == 000004);
    CHECK(AGC_CH031_ATTOUTDETENT == 040000);
    CHECK(AGC_CH033_RRDATAGOOD == 000010);
    CHECK(AGC_CH012_TRIMPITCHM == 000400);
    CHECK(agc_chan_set_signal(AGC_CHAN_CHAN30, AGC_CH030_ABORTDPS, true) == AGC_STATUS_OK);
    CHECK((agc_chan_read(AGC_CHAN_CHAN30) & AGC_CH030_ABORTDPS) == 0u);
} /* test_lm_state */
