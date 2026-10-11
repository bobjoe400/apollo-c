/**********************************************************************
 * FILE NAME: state.c
 *
 * PURPOSE:
 *   state-flags unit tests, CSM build (Comanche055 flag names): flag
 *   set/clear/invert/test, flag ids against the AGC decimal numbers,
 *   channel write/or/and/read and the inverted inputs 030-033, CSM
 *   channel bit names, TIME1/TIME2, state-vector round trips and the
 *   shared erasable accessors. The LM build is tested in state_lm.c.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc pp. 39-124 (flag numbers nnnD,
 *   channel names); T4RUPT_PROGRAM.agc p. 160 (OPTMON channel 33 bits)
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
 *   in the negative tests; agc_assert_report() here counts them.
 **********************************************************************/
#include <stdio.h>

#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_csm.h"
#include "agc/core/state.h"

static int g_checks;         /* checks run                        */
static int g_failures;       /* checks failed                     */
static int g_assert_reports; /* AGC_ASSERT failures reported      */

/* Prototypes: main() comes first (STYLE 4.6). */
static void      check(bool ok, int line, const char *text);
static int       agc_dnum(agc_flag_id_t f);
static void      test_flag_ids(void);
static void      test_flag_ops(void);
static void      test_flag_errors(void);
static void      test_time(void);
static void      test_channels(void);
static void      test_inverted_inputs(void);
static agc_vec_t vec3(agc_dp_t a, agc_dp_t b, agc_dp_t c);
static bool      vec_eq(agc_vec_t a, agc_vec_t b);
static void      test_state_vectors(void);
static void      test_erasables(void);

/**********************************************************************
 * AGC: none (test driver) -- run every test of this file
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    test_flag_ids();
    test_flag_ops();
    test_flag_errors();
    test_time();
    test_channels();
    test_inverted_inputs();
    test_state_vectors();
    test_erasables();
    (void) printf("test_state: %d checks, %d failures\n", g_checks, g_failures);

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
    g_assert_reports++;

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
 * AGC: none (test) -- flag ids against the AGC decimal numbers
 *      (ERASABLE_ASSIGNMENTS.agc C pp. 46-68)
 *
 * FUNCTION NAME: test_flag_ids
 **********************************************************************/
static void test_flag_ids(void)
{
    /* Numbers taken from the AGC listing (ERASABLE_ASSIGNMENTS.agc, pp. 46-66). */
    CHECK(agc_dnum(AGC_FLAG_CPHIFLAG) == 0);
    CHECK(agc_dnum(AGC_FLAG_MOONFLAG) == 3);
    CHECK(agc_dnum(AGC_FLAG_IMUSE) == 7);
    CHECK(agc_dnum(AGC_FLAG_AVEGFLAG) == 29);
    CHECK(agc_dnum(AGC_FLAG_REFSMFLG) == 47);
    CHECK(agc_dnum(AGC_FLAG_DSKYFLAG) == 75);
    CHECK(agc_dnum(AGC_FLAG_ENGONFLG) == 83);
    CHECK(agc_dnum(AGC_FLAG_SERVICER_RUNNING) == 114);
    CHECK(agc_dnum(AGC_FLAG_CMOONFLG) == 123);
    CHECK(agc_dnum(AGC_FLAG_LMOONFLG) == 124);
    CHECK(agc_dnum(AGC_FLAG_QUITFLAG) == 145);
    CHECK(agc_dnum(AGC_FLAG_INTFLAG) == 151);
    CHECK(agc_dnum(AGC_FLAG__05GSW) == 102);
    CHECK(agc_dnum(AGC_FLAG_STABLE_ORBIT_MIDCOURSE_ACTIVE) == 126);
    CHECK(agc_dnum(AGC_FLAG_CM_DSTBY) == 103);
    CHECK(agc_dnum(AGC_FLAG_S32_1F1) == 165);
    /* aliases share one bit */
    CHECK((int) AGC_FLAG_ETPIFLAG == (int) AGC_FLAG_FIRSTFLG);
    CHECK((int) AGC_FLAG_ETPIFLAG == (int) AGC_FLAG_OPTNSW);
    CHECK((int) AGC_FLAG_EGSW == (int) AGC_FLAG_TRUNNION_BIAS_KNOWN);
    CHECK((int) AGC_FLAG_TARG2FLG == (int) AGC_FLAG_BACKUP_MARKING);
    /* encoding (word << 4) | bit: MOONFLAG = FLAGWRD0 bit 12 */
    CHECK((int) AGC_FLAG_MOONFLAG == 0x0C);
    CHECK(agc_flag_mask(AGC_FLAG_MOONFLAG) == 0x0800u);
    CHECK(agc_flag_mask(AGC_FLAG_KFLAG) == 0x0001u);
    CHECK(agc_flag_mask(AGC_FLAG_CPHIFLAG) == 0x4000u);
}

/**********************************************************************
 * AGC: none (test) -- SET, CLEAR, INVERT, BON on flags and whole
 *      flagwords
 *
 * FUNCTION NAME: test_flag_ops
 **********************************************************************/
static void test_flag_ops(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(!agc_flag_test(AGC_FLAG_MOONFLAG));
    agc_flag_set(AGC_FLAG_MOONFLAG);
    CHECK(agc_flag_test(AGC_FLAG_MOONFLAG));
    CHECK(agc_flagword_get(0) == 0x0800u);
    CHECK(!agc_flag_test(AGC_FLAG_JSWITCH)); /* neighbours untouched */
    agc_flag_invert(AGC_FLAG_MOONFLAG);
    CHECK(!agc_flag_test(AGC_FLAG_MOONFLAG));
    agc_flag_invert(AGC_FLAG_MOONFLAG);
    CHECK(agc_flag_test(AGC_FLAG_MOONFLAG));
    agc_flag_clear(AGC_FLAG_MOONFLAG);
    CHECK(!agc_flag_test(AGC_FLAG_MOONFLAG));
    CHECK(agc_flagword_get(0) == 0u);

    /* highest CSM flagword, FLGWRD11 */
    agc_flag_set(AGC_FLAG_S32_1F3B);
    CHECK(agc_flagword_get(11) == 0x0800u);
    agc_flagword_set(11, 077777u);
    CHECK(agc_flag_test(AGC_FLAG_S32_1F1));
    agc_flagword_set(11, 0u);
    CHECK(!agc_flag_test(AGC_FLAG_S32_1F1));

    /* every flag set/cleared individually leaves exactly one bit */
    agc_flag_set(AGC_FLAG_REFSMFLG);
    CHECK(agc_flagword_get(3) == agc_flag_mask(AGC_FLAG_REFSMFLG));
    agc_flag_clear(AGC_FLAG_REFSMFLG);
    CHECK(agc_flagword_get(3) == 0u);
}

/**********************************************************************
 * AGC: none (test) -- invalid flag ids and flagword indices are rejected
 *
 * FUNCTION NAME: test_flag_errors
 **********************************************************************/
static void test_flag_errors(void)
{
    const int before = g_assert_reports; /* reports before this test */

    agc_state_init(AGC_VEHICLE_CSM);
    agc_flag_set((agc_flag_id_t) 0x00); /* rejected by AGC_ASSERT */ /* bit 0 */
    agc_flag_set((agc_flag_id_t) 0xC1); /* rejected by AGC_ASSERT */ /* word 12 is LM-only */
    agc_flag_clear((agc_flag_id_t) 0xFF); /* rejected by AGC_ASSERT */
    CHECK(!agc_flag_test((agc_flag_id_t) 0x10));
    agc_flagword_set(0, 0100000u); /* rejected by AGC_ASSERT */ /* > 15 bits */
    agc_flagword_set(12, 1u); /* rejected by AGC_ASSERT */
    agc_flagword_set(-1, 1u); /* rejected by AGC_ASSERT */
    CHECK(agc_flagword_get(0) == 0u);
    CHECK(!agc_flag_valid((agc_flag_id_t) 0x0F0));
    CHECK(agc_flag_valid(AGC_FLAG_CPHIFLAG));
    CHECK(g_assert_reports > before); /* the failures above were reported */
}

/**********************************************************************
 * AGC: none (test) -- TIME2/TIME1 split and the 2^28 cs wrap
 *      (ERASABLE_ASSIGNMENTS.agc C p. 39)
 *
 * FUNCTION NAME: test_time
 **********************************************************************/
static void test_time(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(agc_time_now_cs() == 0u);
    agc_time_advance_cs(100u);
    CHECK(agc_time_now_cs() == 100u);
    agc_time_set_cs(AGC_TIME_MODULUS_CS - 1u);
    agc_time_advance_cs(1u); /* TIME2 overflow */
    CHECK(agc_time_now_cs() == 0u);
    agc_time_set_cs(AGC_TIME_MODULUS_CS - 2u);
    agc_time_advance_cs(10u);
    CHECK(agc_time_now_cs() == 8u);
    agc_time_set_cs(AGC_TIME_MODULUS_CS); /* rejected by AGC_ASSERT */
    CHECK(agc_time_now_cs() == 8u);
    /* TIME2 = high 14 bits, TIME1 = low 14 bits of the 28-bit count */
    agc_time_set_cs((5u << 14) | 7u);
    CHECK(agc_erase_time2() == 5);
    CHECK(agc_erase_time1() == 7);
    agc_time_set_cs(0x0FFFFFFFu);
    CHECK(agc_erase_time1() == AGC_SP_MAX);
    CHECK(agc_erase_time2() == AGC_SP_MAX);
}

/**********************************************************************
 * AGC: none (test) -- channel WRITE/WOR/WAND/READ, bit constants,
 *      simulator inputs
 *
 * FUNCTION NAME: test_channels
 **********************************************************************/
static void test_channels(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(agc_chan_read(AGC_CHAN_DSALMOUT) == 0u);
    CHECK(agc_chan_read(AGC_CHAN_CHAN30) ==
          077777u); /* inverted: nothing present */
    CHECK(agc_chan_read(AGC_CHAN_CHAN33) == 077777u);

    agc_chan_write(AGC_CHAN_DSALMOUT, 0123u);
    CHECK(agc_chan_read(AGC_CHAN_DSALMOUT) == 0123u);
    agc_chan_or(AGC_CHAN_DSALMOUT, AGC_CH011_ENGINEON);
    CHECK(agc_chan_read(AGC_CHAN_DSALMOUT) == (0123u | 010000u));
    agc_chan_and(AGC_CHAN_DSALMOUT,
                 (uint16_t) ~AGC_CH011_ENGINEON & AGC_CHAN_MASK);
    CHECK(agc_chan_read(AGC_CHAN_DSALMOUT) == 0123u);

    /* bit constants: bit n has value 1 << (n - 1) */
    CHECK(AGC_CH011_ISSWARN == 000001);
    CHECK(AGC_CH011_ENGINEOFF == 020000);
    CHECK(AGC_CH030_IMUOPER == 000400);
    CHECK(AGC_CH030_IMUTEMPOK == 040000);
    CHECK(AGC_CH033_PIPAFAIL == 010000);
    CHECK(AGC_CH032_PROCEED == 020000);
    CHECK(AGC_CH012_COARSE == 000010);
    CHECK(AGC_CH012_TVCENABLE == 000200);
    CHECK(AGC_CH013_ENBT6RUPT == 040000);
    CHECK(AGC_CH014_DRIVECDUX == 040000);

    /* CSM bit names (CM channel list; OPTMON reads 033 bits 4, 5) */
    CHECK(AGC_CH033_ZEROOPTICS == 000010);
    CHECK(AGC_CH033_CMCOPTICS == 000020);
    CHECK(AGC_CH032_MINIMP_MASK == 000077);
    CHECK(AGC_CH032_MINIMPROLLM == 000040);
    CHECK(AGC_CH032_LMATTACHED == 002000);
    CHECK(AGC_CH030_ULLAGETHRUST == 000001);
    CHECK(AGC_CH030_GRR == 000040);
    CHECK(AGC_CH030_SIVBSEPABORT == 000010);
    CHECK(AGC_CH012_ZEROOPTICS == 001000);
    CHECK(AGC_CH012_SIVBINJECT == 010000);

    /* output channels reject the 16th bit; inputs and invalid channels are errors */
    agc_chan_write(AGC_CHAN_CHAN12, 0100000u); /* rejected by AGC_ASSERT */
    agc_chan_write(AGC_CHAN_CHAN30, 0u); /* rejected by AGC_ASSERT */
    agc_chan_write(AGC_CHAN_MNKEYIN, 1u); /* rejected by AGC_ASSERT */
    agc_chan_write(0, 1u); /* rejected by AGC_ASSERT */
    agc_chan_write(AGC_CHAN_COUNT, 1u); /* rejected by AGC_ASSERT */
    agc_chan_or(AGC_CHAN_HISCALAR, 1u); /* rejected by AGC_ASSERT */
    CHECK(agc_chan_read(-1) == 0u);

    /* simulator side */
    CHECK(agc_chan_set_input(AGC_CHAN_MNKEYIN, 031u) == AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_MNKEYIN) == 031u);
    CHECK(agc_chan_set_input(AGC_CHAN_DSALMOUT, 1u) == AGC_STATUS_BAD_ARG);
    CHECK(agc_chan_set_signal(AGC_CHAN_MNKEYIN, 004u, true) == AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_MNKEYIN) ==
          035u); /* normal: signal = bit set */
} /* test_channels */

/**********************************************************************
 * AGC: none (test) -- inverted inputs 030-033 and the channel 33
 *      flip-flops (T4RUPT_PROGRAM.agc C p. 145)
 *
 * FUNCTION NAME: test_inverted_inputs
 **********************************************************************/
static void test_inverted_inputs(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    /* signal present => bit reads 0 on channels 030-033 */
    CHECK(agc_chan_set_signal(AGC_CHAN_CHAN30, AGC_CH030_IMUOPER, true) ==
          AGC_STATUS_OK);
    CHECK((agc_chan_read(AGC_CHAN_CHAN30) & AGC_CH030_IMUOPER) == 0u);
    CHECK((agc_chan_read(AGC_CHAN_CHAN30) | AGC_CH030_IMUOPER) == 077777u);
    CHECK(agc_chan_set_signal(AGC_CHAN_CHAN30, AGC_CH030_IMUOPER, false) ==
          AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_CHAN30) == 077777u);
    CHECK(agc_chan_set_signal(AGC_CHAN_CHAN32, AGC_CH032_PROCEED, true) ==
          AGC_STATUS_OK);
    CHECK(agc_chan_read(AGC_CHAN_CHAN32) ==
          (077777u & ~(unsigned int) AGC_CH032_PROCEED));
    CHECK(agc_chan_set_input(AGC_CHAN_CHAN31, 0u) ==
          AGC_STATUS_OK); /* raw: all present */
    CHECK(agc_chan_read(AGC_CHAN_CHAN31) == 0u);

    /* channel 033 flip-flops (bits 11-15) reset by a write pulse, others kept */
    CHECK(agc_chan_set_signal(AGC_CHAN_CHAN33,
                              AGC_CH033_PIPAFAIL | AGC_CH033_VHFRANGEGOOD,
                              true) == AGC_STATUS_OK);
    CHECK((agc_chan_read(AGC_CHAN_CHAN33) & AGC_CH033_PIPAFAIL) == 0u);
    agc_chan_and(AGC_CHAN_CHAN33, AGC_CH033_FLIPFLOP_MASK); /* T4RUPT style WAND */
    CHECK((agc_chan_read(AGC_CHAN_CHAN33) & AGC_CH033_PIPAFAIL) !=
          0u); /* flip-flop reset */
    CHECK((agc_chan_read(AGC_CHAN_CHAN33) & AGC_CH033_VHFRANGEGOOD) ==
          0u); /* plain input untouched */

    agc_chan_reset();
    CHECK(agc_chan_read(AGC_CHAN_CHAN33) == 077777u);
    CHECK(agc_chan_read(AGC_CHAN_DSALMOUT) == 0u);
} /* test_inverted_inputs */

/**********************************************************************
 * AGC: none (test helper) -- build a vector
 *
 * FUNCTION NAME: vec3
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description
 *   --------  -------------  ---  ---------------------------------
 *   a         agc_dp_t       I    x component (any scale)
 *   b         agc_dp_t       I    y component
 *   c         agc_dp_t       I    z component
 *
 * RETURNS: agc_vec_t -- (a, b, c)
 **********************************************************************/
static agc_vec_t vec3(agc_dp_t a, agc_dp_t b, agc_dp_t c)
{
    agc_vec_t v; /* result, scale of the inputs */

    v.c[0] = a;
    v.c[1] = b;
    v.c[2] = c;

    return (v);
}

/**********************************************************************
 * AGC: none (test helper) -- exact vector equality
 *
 * FUNCTION NAME: vec_eq
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description
 *   --------  -------------  ---  ---------------------------------
 *   a         agc_vec_t      I    first vector
 *   b         agc_vec_t      I    second vector
 *
 * RETURNS: bool -- true when all components are equal
 **********************************************************************/
static bool vec_eq(agc_vec_t a, agc_vec_t b)
{
    return ((a.c[0] == b.c[0]) && (a.c[1] == b.c[1]) && (a.c[2] == b.c[2]));
}

/**********************************************************************
 * AGC: none (test) -- THIS/OTHER state vectors and their moon flags
 *
 * FUNCTION NAME: test_state_vectors
 **********************************************************************/
static void test_state_vectors(void)
{
    agc_state_vector_t in;  /* vector stored */
    agc_state_vector_t out; /* vector read back */

    agc_state_init(AGC_VEHICLE_CSM);

    in.r    = vec3(1000, -2000, 3000);
    in.v    = vec3(-1, 2, -3);
    in.t    = 123456;
    in.moon = true;
    CHECK(agc_sv_set(AGC_SV_THIS, &in) == AGC_STATUS_OK);
    CHECK(agc_sv_get(AGC_SV_THIS, &out) == AGC_STATUS_OK);
    CHECK(vec_eq(out.r, in.r) && vec_eq(out.v, in.v) && (out.t == in.t) &&
          out.moon);
    CHECK(agc_flag_test(AGC_FLAG_MOONFLAG));
    CHECK(vec_eq(agc_erase_rn(), in.r));
    CHECK(vec_eq(agc_erase_vn(), in.v));
    CHECK(agc_erase_piptime() == 123456);

    /* the other vehicle's set is independent; CSM build -> LM set */
    CHECK(agc_sv_get(AGC_SV_OTHER, &out) == AGC_STATUS_OK);
    CHECK(vec_eq(out.r, vec3(0, 0, 0)) && !out.moon);
    in.r    = vec3(7, 8, 9);
    in.v    = vec3(10, 11, 12);
    in.t    = 99;
    in.moon = false;
    CHECK(agc_sv_set(AGC_SV_OTHER, &in) == AGC_STATUS_OK);
    CHECK(vec_eq(agc_erase_rrectlem(), in.r));
    CHECK(vec_eq(agc_erase_vrectlem(), in.v));
    CHECK(agc_erase_tetlem() == 99);
    CHECK(vec_eq(agc_erase_rrectcsm(), vec3(0, 0, 0)));
    CHECK(agc_flag_test(AGC_FLAG_MOONFLAG)); /* THIS flag untouched */
    CHECK(!agc_flag_test(AGC_FLAG_LMOONFLG));
    in.moon = true;
    CHECK(agc_sv_set(AGC_SV_OTHER, &in) == AGC_STATUS_OK);
    CHECK(agc_flag_test(AGC_FLAG_LMOONFLG));
    CHECK(!agc_flag_test(AGC_FLAG_CMOONFLG));

    /* argument and range errors */
    CHECK(agc_sv_get(AGC_SV_THIS, NULL) == AGC_STATUS_BAD_ARG);
    CHECK(agc_sv_set(AGC_SV_THIS, NULL) == AGC_STATUS_BAD_ARG);
    CHECK(agc_sv_get((agc_vehicle_sv_t) 7, &out) == AGC_STATUS_BAD_ARG);
    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(vec_eq(agc_erase_rn(), vec3(0, 0, 0))); /* init clears everything */
} /* test_state_vectors */

/**********************************************************************
 * AGC: none (test) -- shared erasable accessors, range checks and
 *      fresh-start zeroing
 *
 * FUNCTION NAME: test_erasables
 **********************************************************************/
static void test_erasables(void)
{
    agc_mat_t m;                         /* REFSMMAT, scale 2^1       */
    const int before = g_assert_reports; /* reports before this test  */

    agc_state_init(AGC_VEHICLE_CSM);

    agc_erase_set_tig(0x0ABCDEF);
    CHECK(agc_erase_tig() == 0x0ABCDEF);
    agc_erase_set_ttogo(-5);
    CHECK(agc_erase_ttogo() == -5);
    agc_erase_set_tig(AGC_DP_MAX + 1); /* out of range: ignored */
    CHECK(agc_erase_tig() == 0x0ABCDEF);
    CHECK(g_assert_reports > before);

    agc_erase_set_modreg(11);
    CHECK(agc_erase_modreg() == 11);
    agc_erase_set_csmmass(12000);
    CHECK(agc_erase_csmmass() == 12000);
    agc_erase_set_cdux(077777u);
    CHECK(agc_erase_cdux() == 077777u);
    agc_erase_set_cduy(0100000u); /* 16 bits: ignored */
    CHECK(agc_erase_cduy() == 0u);
    agc_erase_set_markstat(0123u);
    CHECK(agc_erase_markstat() == 0123u);
    agc_erase_set_pipax(-7);
    CHECK(agc_erase_pipax() == -7);
    agc_erase_set_tephem((agc_tp_t) 1 << 40);
    CHECK(agc_erase_tephem() == ((agc_tp_t) 1 << 40));

    m = agc_erase_refsmmat();
    CHECK(m.row[1].c[1] == 0);
    m.row[0].c[0] = AGC_DP_HALF;
    m.row[1].c[1] = AGC_DP_HALF;
    m.row[2].c[2] = -AGC_DP_HALF;
    agc_erase_set_refsmmat(m);
    CHECK(agc_erase_refsmmat().row[2].c[2] == -AGC_DP_HALF);
    CHECK(agc_erase_refsmmat().row[0].c[1] == 0);

    agc_erase_set_dsptab(11, 040010u);
    CHECK(agc_erase_dsptab(11) == 040010u);
    agc_erase_set_dsptab(12, 1u); /* rejected by AGC_ASSERT */
    agc_erase_set_dsptab(-1, 1u); /* rejected by AGC_ASSERT */
    CHECK(agc_erase_set_failreg(2, 01201u) == AGC_STATUS_OK);
    CHECK(agc_erase_failreg(2) == 01201u);
    agc_erase_set_thetad(0, 0100000u); /* rejected by AGC_ASSERT */
    CHECK(agc_erase_thetad(3) == 0u);
    CHECK(agc_erase_set_failreg(3, 1u) == AGC_STATUS_BAD_ARG);
    CHECK(agc_erase_set_failreg(0, 0100000u) == AGC_STATUS_BAD_ARG);

    /* fresh-start zeroing reaches every storage class */
    agc_erase_set_rls(vec3(1, 2, 3));
    agc_erase_reset();
    CHECK(agc_erase_dsptab(11) == 0u);
    CHECK(agc_erase_failreg(2) == 0u);
    CHECK(agc_erase_tig() == 0);
    CHECK(agc_erase_refsmmat().row[2].c[2] == 0);
    CHECK(vec_eq(agc_erase_rls(), vec3(0, 0, 0)));
} /* test_erasables */
