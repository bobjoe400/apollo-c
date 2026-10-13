/**********************************************************************
 * FILE NAME: ecadr.c
 *
 * PURPOSE:
 *   Erasable access by address (src/core/ecadr.c): the address map onto
 *   the typed erasables, both ropes. A V71 state-vector / REFSMMAT /
 *   TEPHEM load (words stored from the last to the first, as STORLP71
 *   does, signs free to disagree) reaches the typed erasable; a read of a
 *   mapped address follows the typed value after the program changes it;
 *   flagwords and unmapped addresses behave as before.
 *
 * AGC SOURCE:
 *   Comanche055 / Luminary099 ERASABLE_ASSIGNMENTS (addresses from the
 *   assembly listings); UPDATE_PROGRAM.agc STORLP71 (C p.1496, L p.1384).
 *
 * EXTERNAL VARIABLES:
 *   none (test counters are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
 *   agc_erase_read_ecadr   read by address
 *   agc_erase_write_ecadr  store by address
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails.
 **********************************************************************/
#include <stdio.h>

#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"

static int g_checks;   /* checks run    */
static int g_failures; /* checks failed */

/* Prototypes: main() comes first (STYLE 4.6). */
static void check(bool ok, int line, const char *text);
static void store_block(uint16_t ecadr, const uint16_t *w, int n);
static void test_refsmmat(agc_vehicle_t v, uint16_t ecadr);
static void test_state_vector_uplink(agc_vehicle_t v);
static void test_tephem_and_follow(agc_vehicle_t v);
static void test_unmapped_and_flags(agc_vehicle_t v);
static void test_pad_loads(void);

#define CHECK(c) check((c), __LINE__, #c)

/**********************************************************************
 * AGC: none (test driver) -- run every test of this file
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    test_refsmmat(AGC_VEHICLE_CSM, 01735u);
    test_refsmmat(AGC_VEHICLE_LM, 01733u);
    test_state_vector_uplink(AGC_VEHICLE_CSM);
    test_state_vector_uplink(AGC_VEHICLE_LM);
    test_tephem_and_follow(AGC_VEHICLE_CSM);
    test_tephem_and_follow(AGC_VEHICLE_LM);
    test_unmapped_and_flags(AGC_VEHICLE_CSM);
    test_unmapped_and_flags(AGC_VEHICLE_LM);
    test_pad_loads();
    (void) printf("test_ecadr: %d checks, %d failures\n", g_checks, g_failures);

    return ((g_failures == 0) ? 0 : 1); /* exit 1 on any failure */
}

/**********************************************************************
 * AGC: none (test hook replacing core/assert.c)
 *
 * FUNCTION NAME: agc_assert_report
 *
 * ARGUMENTS:
 *   file  const char *  I  source file
 *   line  int           I  source line
 *   expr  const char *  I  the failed expression
 *
 * RETURNS: bool -- false (the assertion failed)
 **********************************************************************/
bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void) printf("assertion: %s (%s:%d)\n", expr, file, line);

    return (false);
}

/**********************************************************************
 * AGC: none (test helper) -- count one check, print it when it fails
 *
 * ARGUMENTS:
 *   ok    bool          I  the check passed
 *   line  int           I  source line
 *   text  const char *  I  the check's text
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
 * AGC: none (test helper) -- STORLP71: n words from the last to the first
 *
 * ARGUMENTS:
 *   ecadr  uint16_t          I  first address
 *   w      const uint16_t *  I  the words
 *   n      int               I  number of words
 **********************************************************************/
static void store_block(uint16_t ecadr, const uint16_t *w, int n)
{
    for (int i = n - 1; i >= 0; i--)
    {
        CHECK(agc_erase_write_ecadr((uint16_t) (ecadr + (uint16_t) i), w[i]) == AGC_STATUS_OK);
    }
}

/**********************************************************************
 * AGC: none (test) -- REFSMMAT uplink (RTCC MCCCRF / MCCLRF): identity
 * with one mixed-sign element; the typed matrix and the words read back.
 *
 * ARGUMENTS:
 *   v      agc_vehicle_t  I  rope
 *   ecadr  uint16_t       I  REFSMMAT address in that rope
 **********************************************************************/
static void test_refsmmat(agc_vehicle_t v, uint16_t ecadr)
{
    uint16_t w[18] = {0}; /* 9 DP elements, scale 2^1: 1.0 = 010000 00000 */
    uint16_t r     = 0;   /* a word read back */

    agc_state_init(v);
    w[0]  = 010000u; /* m11 = +1                                     */
    w[8]  = 010000u; /* m22 = +1                                     */
    w[16] = 010000u; /* m33 = +1                                     */
    w[2]  = 000001u; /* m12 = +1 x 2^14 - 3 (mixed signs, kept)       */
    w[3]  = 077774u; /* -3                                           */
    store_block(ecadr, w, 18);

    agc_mat_t m = agc_erase_refsmmat(); /* typed */

    CHECK(m.row[0].c[0] == (agc_dp_t) 010000 * 040000);
    CHECK(m.row[1].c[1] == (agc_dp_t) 010000 * 040000);
    CHECK(m.row[2].c[2] == (agc_dp_t) 010000 * 040000);
    CHECK(m.row[0].c[1] == 040000 - 3);
    agc_erase_read_ecadr((uint16_t) (ecadr + 3u), &r);
    CHECK(r == 077774u); /* the words as uplinked */
    agc_erase_read_ecadr((uint16_t) (ecadr + 2u), &r);
    CHECK(r == 000001u);
}

/**********************************************************************
 * AGC: none (test) -- the V71 state-vector load at 1501 (UPSVFLAG, RRECT,
 * VRECT, TET) lands in UPSVFLAG and the integration buffer that INTWAKEU
 * moves to the permanent vector; RRECTCSM written by the program reads
 * back at 1554 (RTCC "SV from AGC").
 *
 * ARGUMENTS:
 *   v  agc_vehicle_t  I  rope
 **********************************************************************/
static void test_state_vector_uplink(agc_vehicle_t v)
{
    uint16_t w[15] = {0}; /* UPSVFLAG, RRECT 6, VRECT 6, TET 2 */
    uint16_t r     = 0;   /* a word read back */

    agc_state_init(v);
    w[0]  = 1u;      /* CSM, earth          */
    w[1]  = 00123u;  /* RRECT x high         */
    w[2]  = 04567u;  /* RRECT x low          */
    w[7]  = 077654u; /* VRECT x high: -0123  */
    w[8]  = 073210u; /* VRECT x low:  -04567 */
    w[14] = 00042u;  /* TET low              */
    store_block(01501u, w, 15);

    agc_integ_ctx_t *ctx = agc_integ_shared_ctx(); /* integration erasables */

    CHECK(agc_erase_upsvflag() == 1);
    CHECK(ctx->sv.rrect.c[0] == (agc_dp_t) 00123 * 040000 + 04567);
    CHECK(ctx->sv.vrect.c[0] == -((agc_dp_t) 00123 * 040000 + 04567));
    CHECK(ctx->sv.tet == 042);

    agc_vec_t p = {{5 * 040000 + 7, -9, 0}}; /* RRECTCSM set by the program */

    agc_erase_set_rrectcsm(p);
    agc_erase_read_ecadr(01554u, &r);
    CHECK(r == 5u);
    agc_erase_read_ecadr(01555u, &r);
    CHECK(r == 7u);
    agc_erase_read_ecadr(01557u, &r);
    CHECK(r == (uint16_t) (077777u - 9u));
}

/**********************************************************************
 * AGC: none (test) -- TEPHEM (RTCC MCCTEP 01706) as three words, then a
 * change of the typed value is what the next read sees.
 *
 * ARGUMENTS:
 *   v  agc_vehicle_t  I  rope
 **********************************************************************/
static void test_tephem_and_follow(agc_vehicle_t v)
{
    uint16_t w[3] = {00001u, 00002u, 00003u}; /* TP words */
    uint16_t r    = 0;                        /* a word read back */

    agc_state_init(v);
    store_block(01706u, w, 3);
    CHECK(agc_erase_tephem() == ((agc_tp_t) 1 << 28) + (agc_tp_t) 2 * 040000 + 3);
    agc_erase_set_tephem((agc_tp_t) 7);
    agc_erase_read_ecadr(01706u, &r);
    CHECK(r == 0u);
    agc_erase_read_ecadr(01710u, &r);
    CHECK(r == 7u);
}

/**********************************************************************
 * AGC: none (test) -- the flagword map and the raw image are unchanged.
 *
 * ARGUMENTS:
 *   v  agc_vehicle_t  I  rope
 **********************************************************************/
static void test_unmapped_and_flags(agc_vehicle_t v)
{
    uint16_t r = 0; /* a word read back */

    agc_state_init(v);
    CHECK(agc_erase_write_ecadr(0104u, 012345u) == AGC_STATUS_OK);  /* FLAGWRD8 */
    CHECK(agc_flagword_get(8) == 012345u);
    CHECK(agc_erase_write_ecadr(02777u, 054321u) == AGC_STATUS_OK); /* not mapped */
    agc_erase_read_ecadr(02777u, &r);
    CHECK(r == 054321u);
}

/**********************************************************************
 * AGC: none (test) -- launch pad loads by address (Comanche055 CMPAD):
 * IMU compensation at 1452 (PBIASX PIPASCFX PBIASY ..), AZO at 1711 and
 * the lunar ephemeris (VECOEM 2036, OMEGAES 2146) reach their typed homes.
 **********************************************************************/
static void test_pad_loads(void)
{
    agc_imu_comp_params_t c;   /* compensation read back */
    agc_navaux_padload_t  pad; /* AZO .. */
    agc_ephem_csm_t       eph; /* VECOEM .. */

    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(agc_erase_write_ecadr(01452u, 077745u) == AGC_STATUS_OK); /* PBIASX = -26 */
    CHECK(agc_erase_write_ecadr(01453u, 000050u) == AGC_STATUS_OK); /* PIPASCFX = 40 */
    CHECK(agc_erase_write_ecadr(01470u, 000005u) == AGC_STATUS_OK); /* ADSRAZ = 5 */
    agc_imu_comp_get(&c);
    CHECK(c.pipabias[0] == -26);
    CHECK(c.pipascf[0] == 40);
    CHECK(c.adsra[2] == 5);
    CHECK(agc_erase_write_ecadr(01711u, 00003u) == AGC_STATUS_OK);  /* AZO high */
    CHECK(agc_erase_write_ecadr(01712u, 00007u) == AGC_STATUS_OK);  /* AZO low  */
    agc_navaux_get_padload(&pad);
    CHECK(pad.azo == 3 * 040000 + 7);
    CHECK(agc_erase_write_ecadr(02036u, 00001u) == AGC_STATUS_OK);  /* VECOEM [0].x high */
    CHECK(agc_erase_write_ecadr(02147u, 00011u) == AGC_STATUS_OK);  /* OMEGAES low */
    agc_ephem_get_csm_data(&eph);
    CHECK(eph.vecoem[0].c[0] == 040000);
    CHECK(eph.omegaes == 011);
}
