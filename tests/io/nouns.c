/*
 * nouns.c -- io-nouns unit tests: noun tables, scale types, data ids.
 *
 * AGC_ASSERT failures are expected in the negative tests; this file supplies
 * its own agc_assert_report() that counts them instead of printing.
 */
#include <stdio.h>
#include <stdint.h>
#include "agc/core/assert.h"
#include "agc/io/nouns.h"
#include "agc/core/state.h"

static int g_checks;
static int g_failures;

bool agc_assert_report(const char *file, int line, const char *expr)
{
    (void)file;
    (void)line;
    (void)expr;
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

static bool is_spare(const agc_noun_t *n)
{
    return n->ncomp == 0;
}

/* Every noun 00-99 present, in order, internally consistent. */
static void test_table_complete(agc_vehicle_t veh)
{
    int spares = 0;
    for (uint8_t n = 0; n < AGC_NOUN_COUNT; n++) {
        agc_noun_t row;
        const agc_status_t st = agc_noun_lookup_vehicle(veh, n, &row);
        CHECK(row.noun == n);
        CHECK(row.ncomp <= AGC_NOUN_MAX_COMP);
        CHECK(row.no_load ? row.dec_only : true);
        if (is_spare(&row)) {
            spares++;
            CHECK(st == AGC_STATUS_NOT_FOUND);
            CHECK(row.data_id[0] == AGC_DATA_NONE && row.scale_type[0] == AGC_SCALE_NONE);
            continue;
        }
        CHECK(st == AGC_STATUS_OK);
        for (int k = 0; k < AGC_NOUN_MAX_COMP; k++) {
            CHECK(row.data_id[k] < AGC_DATA_COUNT);
            CHECK(row.scale_type[k] < AGC_SCALE_COUNT);
            if (k >= row.ncomp) {
                CHECK(row.scale_type[k] == AGC_SCALE_NONE && row.data_id[k] == AGC_DATA_NONE);
            } else {
                CHECK(row.scale_type[k] != AGC_SCALE_NONE);
                CHECK(row.data_id[k] != AGC_DATA_NONE || row.scale_type[0] == AGC_SCALE_HMS);
            }
        }
    }
    CHECK(spares == ((veh == AGC_VEHICLE_CSM) ? 9 : 16));
}

/* Noun 36 and a few mixed / vehicle-specific nouns. */
static void test_known_nouns(void)
{
    agc_noun_t n;
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 36, &n) == AGC_STATUS_OK);
    CHECK(n.ncomp == 3 && n.dec_only && !n.no_load);
    CHECK(n.scale_type[0] == AGC_SCALE_HMS && n.scale_type[1] == AGC_SCALE_HMS &&
          n.scale_type[2] == AGC_SCALE_HMS);
    CHECK(n.data_id[0] == AGC_DATA_TIME2 && n.data_id[1] == AGC_DATA_NONE);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 36, &n) == AGC_STATUS_OK);
    CHECK(n.data_id[0] == AGC_DATA_TIME2);

    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 40, &n) == AGC_STATUS_OK);   /* mixed */
    CHECK(n.ncomp == 3 && n.dec_only && n.no_load);
    CHECK(n.scale_type[0] == AGC_SCALE_MIN_SEC && n.scale_type[1] == AGC_SCALE_VEL3 &&
          n.scale_type[2] == AGC_SCALE_VEL3);
    CHECK(n.data_id[0] == AGC_DATA_TTOGO && n.data_id[1] == AGC_DATA_VGDISP &&
          n.data_id[2] == AGC_DATA_DVTOTAL);

    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 20, &n) == AGC_STATUS_OK);   /* normal */
    CHECK(n.data_id[0] == AGC_DATA_CDUX && n.data_id[1] == AGC_DATA_CDUY &&
          n.data_id[2] == AGC_DATA_CDUZ && n.scale_type[2] == AGC_SCALE_CDU_DEG);

    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 46, &n) == AGC_STATUS_OK && n.ncomp == 2);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 46, &n) == AGC_STATUS_OK && n.ncomp == 1);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 87, &n) == AGC_STATUS_OK &&
          n.scale_type[1] == AGC_SCALE_Y_OPTICS_DEG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 87, &n) == AGC_STATUS_OK &&
          n.scale_type[1] == AGC_SCALE_CDU_DEG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 66, &n) == AGC_STATUS_OK &&
          n.scale_type[1] == AGC_SCALE_LR_POSITION && n.data_id[1] == AGC_DATA_CHANNEL33);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 10, &n) == AGC_STATUS_OK &&
          n.data_id[0] == AGC_DATA_SPECIFY_CHANNEL);
}

static void test_unknown(void)
{
    agc_noun_t n;
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 100, &n) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 255, &n) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 36, NULL) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 4, &n) == AGC_STATUS_NOT_FOUND);   /* spare */
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_CSM, 0, &n) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_noun_lookup_vehicle(AGC_VEHICLE_LM, 69, &n) == AGC_STATUS_NOT_FOUND);
}

static void test_scales(void)
{
    agc_scale_info_t si;
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_HMS, &si) == AGC_STATUS_OK);
    CHECK(si.routine == AGC_SFR_HMS && si.sf_code == 0 && si.is_dp && !si.is_raw);
    CHECK(si.sf_in == 100000);                       /* SFINTAB 00006 03240 */
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_POS9, &si) == AGC_STATUS_OK);
    CHECK(si.sf_code == 18);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_LM, AGC_SCALE_POS9, &si) == AGC_STATUS_OK);
    CHECK(si.sf_code == 26);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_LM, AGC_SCALE_LR_VELX, &si) == AGC_STATUS_OK);
    CHECK(si.sf_in < 0 && si.sf_out < 0);            /* SFIN 77766 60557: negative */
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_LR_VELX, &si) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_LM, AGC_SCALE_Y_OPTICS_DEG, &si) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_CDU_DEG, &si) == AGC_STATUS_OK);
    CHECK(si.is_raw && !si.is_dp && si.routine == AGC_SFR_CDU_DEG && si.sf_in == 0);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_NONE, &si) == AGC_STATUS_BAD_ARG);
    CHECK(agc_scale_info_vehicle(AGC_VEHICLE_CSM, AGC_SCALE_COUNT, &si) == AGC_STATUS_BAD_ARG);

    agc_dp_t in = 0;
    agc_dp_t out = 0;
    CHECK(agc_scale_sfconst(AGC_VEHICLE_CSM, 20, &in, &out) == AGC_STATUS_BAD_ARG);
    CHECK(agc_scale_sfconst(AGC_VEHICLE_LM, 28, &in, &out) == AGC_STATUS_OK);
    /* every scale type a noun uses resolves for that vehicle */
    for (int veh = 0; veh < 2; veh++) {
        const agc_vehicle_t v = (veh == 0) ? AGC_VEHICLE_CSM : AGC_VEHICLE_LM;
        for (uint8_t n = 0; n < AGC_NOUN_COUNT; n++) {
            agc_noun_t row;
            (void)agc_noun_lookup_vehicle(v, n, &row);
            for (int k = 0; k < row.ncomp; k++) {
                CHECK(agc_scale_info_vehicle(v, (agc_scale_t)row.scale_type[k], &si) == AGC_STATUS_OK);
            }
        }
    }
}

/* agc_noun_lookup / read / write against the core state. */
static void test_data(void)
{
    agc_dp_t v = 0;
    agc_noun_t n;
    agc_state_init(AGC_VEHICLE_LM);
    CHECK(agc_noun_lookup(36, &n) == AGC_STATUS_OK && n.data_id[0] == AGC_DATA_TIME2);
    CHECK(agc_noun_lookup(46, &n) == AGC_STATUS_OK && n.ncomp == 1);       /* LM */
    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(agc_noun_lookup(46, &n) == AGC_STATUS_OK && n.ncomp == 2);       /* CSM */

    CHECK(agc_noun_write(AGC_DATA_TIME2, 123456) == AGC_STATUS_OK);
    CHECK(agc_noun_read(AGC_DATA_TIME2, &v) == AGC_STATUS_OK && v == 123456);
    CHECK(agc_noun_write(AGC_DATA_TIG, 7000) == AGC_STATUS_OK);
    CHECK(agc_noun_read(AGC_DATA_TIG, &v) == AGC_STATUS_OK && v == 7000);
    CHECK(agc_noun_write(AGC_DATA_TIME2, -1) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_write(AGC_DATA_CDUY, 0x7FFF) == AGC_STATUS_OK);         /* raw word */
    CHECK(agc_noun_read(AGC_DATA_CDUY, &v) == AGC_STATUS_OK && v == 0x7FFF);
    CHECK(agc_noun_write(AGC_DATA_CDUY, 0x8000) == AGC_STATUS_BAD_ARG);

    CHECK(agc_noun_read(AGC_DATA_SPECIFY_ADDR, &v) == AGC_STATUS_NOT_FOUND);
    CHECK(agc_noun_read(AGC_DATA_NONE, &v) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_read(AGC_DATA_COUNT, &v) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_read(AGC_DATA_TIME2, NULL) == AGC_STATUS_BAD_ARG);
    CHECK(agc_noun_read(AGC_DATA_HAPO, &v) == AGC_STATUS_UNIMPLEMENTED);   /* not in core yet */
    CHECK(agc_noun_write(AGC_DATA_HAPO, 0) == AGC_STATUS_UNIMPLEMENTED);
}

int main(void)
{
    test_table_complete(AGC_VEHICLE_CSM);
    test_table_complete(AGC_VEHICLE_LM);
    test_known_nouns();
    test_unknown();
    test_scales();
    test_data();
    (void)printf("test_nouns: %d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
