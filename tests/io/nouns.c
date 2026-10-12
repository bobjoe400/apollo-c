/**********************************************************************
 * FILE NAME: nouns.c
 *
 * PURPOSE:
 *   io-nouns unit tests: both noun tables complete and consistent (00-99),
 *   selected nouns against the AGC tables, out-of-range lookups, the
 *   scale-type table (SFINTAB / SFOUTAB constants) and the noun data-id
 *   read/write path against the core state.
 *
 * AGC SOURCE:
 *   Comanche055 PINBALL_NOUN_TABLES.agc pp. 263-289 (NNADTAB, NNTYPTAB,
 *   SFINTAB, SFOUTAB, IDADDTAB, RUTMXTAB); Luminary099 pp. 302-321
 *
 * EXTERNAL VARIABLES:
 *   none (test counters are file-static)
 *
 * EXTERNAL REFERENCES:
 *   Name                     Description
 *   -----------------------  ---------------------------------------
 *   agc_noun_lookup_vehicle  noun table row of either vehicle
 *   agc_scale_info_vehicle   scale-type row of either vehicle
 *   agc_noun_read / _write   noun data ids against the core state
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Exit status 1 when any check fails. AGC_ASSERT failures are expected
 *   in the negative tests; agc_assert_report() here swallows them.
 **********************************************************************/
#include <stdio.h>

#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/guidance/guidance.h"
#include "agc/io/nouns.h"
#include "agc/io/service.h"

static int g_checks;   /* checks run                        */
static int g_failures; /* checks failed                     */

/* Prototypes: main() comes first (STYLE 4.6). */
static void check(bool ok, int line, const char *text);
static bool is_spare(const agc_noun_t *n);
static void test_table_complete(agc_vehicle_t veh);
static void test_known_nouns(void);
static void test_unknown(void);
static void test_scales(void);
static void test_data(void);

/**********************************************************************
 * AGC: none (test driver) -- run every test of this file
 *
 * FUNCTION NAME: main
 *
 * RETURNS: int -- 0 when all checks pass, 1 otherwise
 **********************************************************************/
int main(void)
{
    test_table_complete(AGC_VEHICLE_CSM);
    test_table_complete(AGC_VEHICLE_LM);
    test_known_nouns();
    test_unknown();
    test_scales();
    test_data();
    (void) printf("test_nouns: %d checks, %d failures\n", g_checks, g_failures);

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
 * AGC: none (test helper) -- a spare noun has no components
 *
 * FUNCTION NAME: is_spare
 *
 * ARGUMENTS:
 *   Argument  Type                I/O  Description
 *   --------  ------------------  ---  ----------------------------
 *   n         const agc_noun_t *  I    noun table row
 *
 * RETURNS: bool -- true for a spare noun
 **********************************************************************/
static bool is_spare(const agc_noun_t *n)
{
    return (n->ncomp == 0);
}

/**********************************************************************
 * AGC: none (test) -- every noun 00-99 present, in order, internally
 *      consistent (NNADTAB, NNTYPTAB)
 *
 * FUNCTION NAME: test_table_complete
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description
 *   --------  -------------  ---  ---------------------------------
 *   veh       agc_vehicle_t  I    vehicle whose table is checked
 **********************************************************************/
static void test_table_complete(agc_vehicle_t veh)
{
    int spares = 0; /* spare rows seen */

    for (uint8_t n = 0; n < AGC_NOUN_COUNT; n++)
    {
        agc_noun_t         row;                                        /* noun table row */
        const agc_status_t st = agc_noun_lookup_vehicle(veh, n, &row); /* lookup status */

        CHECK(row.noun == n);
        CHECK(row.ncomp <= AGC_NOUN_MAX_COMP);
        CHECK(row.no_load ? row.dec_only : true); /* no-load: dec only */

        if (is_spare(&row))
        {
            spares++;
            CHECK(st == AGC_STATUS_NOT_FOUND);
            CHECK(row.data_id[0] == AGC_DATA_NONE && row.scale_type[0] == AGC_SCALE_NONE);
            continue;
        }

        CHECK(st == AGC_STATUS_OK);

        for (int k = 0; k < AGC_NOUN_MAX_COMP; k++)
        {
            CHECK(row.data_id[k] < AGC_DATA_COUNT);
            CHECK(row.scale_type[k] < AGC_SCALE_COUNT);

            if (k >= row.ncomp)
            {
                CHECK(row.scale_type[k] == AGC_SCALE_NONE && row.data_id[k] == AGC_DATA_NONE);
            }
            else
            {
                CHECK(row.scale_type[k] != AGC_SCALE_NONE);
                CHECK(row.data_id[k] != AGC_DATA_NONE || row.scale_type[0] == AGC_SCALE_HMS);
            }
        }
    }

    CHECK(spares == ((veh == AGC_VEHICLE_CSM) ? 9 : 16)); /* per vehicle */
} /* test_table_complete */

/**********************************************************************
 * AGC: none (test) -- noun 36 and a few mixed / vehicle-specific nouns
 *      (PINBALL_NOUN_TABLES.agc C pp. 263-289, L pp. 302-321)
 *
 * FUNCTION NAME: test_known_nouns
 **********************************************************************/
static void test_known_nouns(void)
{
    agc_noun_t n; /* noun table row */

    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 36, &n) == AGC_STATUS_OK);
    CHECK(n.ncomp == 3 && n.dec_only && !n.no_load);
    CHECK(n.scale_type[0] == AGC_SCALE_HMS && n.scale_type[1] == AGC_SCALE_HMS && n.scale_type[2] == AGC_SCALE_HMS);
    CHECK(n.data_id[0] == AGC_DATA_TIME2 && n.data_id[1] == AGC_DATA_NONE);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 36, &n) == AGC_STATUS_OK);
    CHECK(n.data_id[0] == AGC_DATA_TIME2);

    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 40, &n) == AGC_STATUS_OK); /* mixed */
    CHECK(n.ncomp == 3 && n.dec_only && n.no_load);
    CHECK(n.scale_type[0] == AGC_SCALE_MIN_SEC && n.scale_type[1] == AGC_SCALE_VEL3 && n.scale_type[2] == AGC_SCALE_VEL3);
    CHECK(n.data_id[0] == AGC_DATA_TTOGO && n.data_id[1] == AGC_DATA_VGDISP && n.data_id[2] == AGC_DATA_DVTOTAL);

    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 20, &n) == AGC_STATUS_OK); /* normal */
    CHECK(n.data_id[0] == AGC_DATA_CDUX && n.data_id[1] == AGC_DATA_CDUY && n.data_id[2] == AGC_DATA_CDUZ && n.scale_type[2] == AGC_SCALE_CDU_DEG);

    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 46, &n) == AGC_STATUS_OK && n.ncomp == 2);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 46, &n) == AGC_STATUS_OK && n.ncomp == 1);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 87, &n) == AGC_STATUS_OK && n.scale_type[1] == AGC_SCALE_Y_OPTICS_DEG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 87, &n) == AGC_STATUS_OK && n.scale_type[1] == AGC_SCALE_CDU_DEG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 66, &n) == AGC_STATUS_OK && n.scale_type[1] == AGC_SCALE_LR_POSITION && n.data_id[1] == AGC_DATA_CHANNEL33);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 10, &n) == AGC_STATUS_OK && n.data_id[0] == AGC_DATA_SPECIFY_CHANNEL);
}

/**********************************************************************
 * AGC: none (test) -- out-of-range and spare noun lookups
 *
 * FUNCTION NAME: test_unknown
 **********************************************************************/
static void test_unknown(void)
{
    agc_noun_t n; /* noun table row */

    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 100, &n) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 255, &n) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 36, NULL) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 4, &n) == AGC_STATUS_NOT_FOUND); /* spare */
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 0, &n) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 69, &n) == AGC_STATUS_NOT_FOUND);
}

/**********************************************************************
 * AGC: none (test) -- scale-type table and SF constants (SFINTAB,
 *      SFOUTAB)
 *
 * FUNCTION NAME: test_scales
 **********************************************************************/
static void test_scales(void)
{
    agc_scale_info_t si; /* scale-type row */

    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_HMS, &si) == AGC_STATUS_OK);
    CHECK(si.routine == AGC_SFR_HMS && si.sf_code == 0 && si.is_dp && !si.is_raw);
    CHECK(si.sf_in == 100000);            /* SFINTAB 00006 03240 */
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_POS9, &si) == AGC_STATUS_OK);
    CHECK(si.sf_code == 18);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_LM, AGC_SCALE_POS9, &si) == AGC_STATUS_OK);
    CHECK(si.sf_code == 26);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_LM, AGC_SCALE_LR_VELX, &si) == AGC_STATUS_OK);
    CHECK(si.sf_in < 0 && si.sf_out < 0); /* SFIN 77766 60557: negative */
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_LR_VELX, &si) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_LM, AGC_SCALE_Y_OPTICS_DEG, &si) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_CDU_DEG, &si) == AGC_STATUS_OK);
    CHECK(si.is_raw && !si.is_dp && si.routine == AGC_SFR_CDU_DEG && si.sf_in == 0);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_NONE, &si) == AGC_STATUS_BAD_ARG);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_COUNT, &si) == AGC_STATUS_BAD_ARG);

    agc_dp_t in  = 0; /* SFIN constant, unscaled */
    agc_dp_t out = 0; /* SFOUT constant, unscaled */

    /* CSM codes come from scale_defs (< 20 by construction; the range
     * guard went with the EVA pass, 2026-10-12): the last one resolves.   */
    CHECK(agc_scale_sfconst(AGC_VEHICLE_CSM, 19, &in, &out) == AGC_STATUS_OK);

    CHECK(agc_scale_sfconst(AGC_VEHICLE_LM, 28, &in, &out) == AGC_STATUS_OK);

    /* every scale type a noun uses resolves for that vehicle */
    for (int veh = 0; veh < 2; veh++)
    {
        const agc_vehicle_t v = (veh == 0) ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM; /* vehicle */

        for (uint8_t n = 0; n < AGC_NOUN_COUNT; n++)
        {
            agc_noun_t row; /* noun table row */

            (void) agc_noun_lookup_vehicle(v, n, &row);

            for (int k = 0; k < row.ncomp; k++)
            {
                CHECK(agc_scale_info_vehicle(v, (agc_scale_t) row.scale_type[k], &si) == AGC_STATUS_OK);
            }
        }
    }
} /* test_scales */

/**********************************************************************
 * AGC: none (test) -- agc_noun_lookup / read / write against the core
 *      state
 *
 * FUNCTION NAME: test_data
 **********************************************************************/
static void test_data(void)
{
    agc_dp_t   v = 0; /* datum read back, scale of the id */
    agc_noun_t n;     /* noun table row */

    agc_state_init(AGC_VEHICLE_LM);
    CHECK(agc_noun_lookup(36, &n) == AGC_STATUS_OK && n.data_id[0] == AGC_DATA_TIME2);
    CHECK(agc_noun_lookup(46, &n) == AGC_STATUS_OK && n.ncomp == 1); /* LM */
    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(agc_noun_lookup(46, &n) == AGC_STATUS_OK && n.ncomp == 2); /* CSM */

    CHECK(agc_noun_write(AGC_DATA_TIME2, 123456) == AGC_STATUS_OK);
    CHECK(agc_noun_read(AGC_DATA_TIME2, &v) == AGC_STATUS_OK && v == 123456);
    CHECK(agc_noun_write(AGC_DATA_TIG, 7000) == AGC_STATUS_OK);
    CHECK(agc_noun_read(AGC_DATA_TIG, &v) == AGC_STATUS_OK && v == 7000);
    CHECK(agc_noun_write(AGC_DATA_TIME2, -1) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_write(AGC_DATA_CDUY, 0x7FFF) == AGC_STATUS_OK); /* raw word */
    CHECK(agc_noun_read(AGC_DATA_CDUY, &v) == AGC_STATUS_OK && v == 0x7FFF);
    CHECK(agc_noun_write(AGC_DATA_CDUY, 0x8000) == AGC_STATUS_BAD_ARG);

    CHECK(agc_noun_read(AGC_DATA_SPECIFY_ADDR, &v) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_noun_read(AGC_DATA_NONE, &v) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_read(AGC_DATA_COUNT, &v) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_read(AGC_DATA_TIME2, NULL) == AGC_STATUS_BAD_ARG);

    /* HAPO lives in the guidance block: through agc_dispatch_noun.     */
    CHECK(agc_noun_write(AGC_DATA_HAPO, 123456789) == AGC_STATUS_OK);
    CHECK(agc_noun_read(AGC_DATA_HAPO, &v) == AGC_STATUS_OK && v == 123456789 && agc_guid_state()->burn.hapo == 123456789);

    /* N08: ALMCADR = the alarming job, +1 = +0 (no banks), +2 = ERCOUNT. */
    CHECK(agc_noun_read(AGC_DATA_ALMCADR, &v) == AGC_STATUS_OK);
    CHECK(agc_noun_read(AGC_DATA_ALMCADR_P1, &v) == AGC_STATUS_OK && v == 0);
    CHECK(agc_noun_read(AGC_DATA_ALMCADR_P2, &v) == AGC_STATUS_OK && v == (agc_dp_t) agc_selfcheck_error_count());
}
