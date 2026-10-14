/**********************************************************************
 * FILE NAME: address_map.c
 *
 * PURPOSE:
 *   Erasable access by address (src/erasable_map/address_map.c): the address map onto
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
#include "agc/core/word.h"
#include "agc/csm/dap.h"
#include "agc/csm/entry.h"
#include "agc/erasable_map/address_map.h"
#include "agc/guidance/guidance.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/lm/landing.h"
#include "agc/nav/auxiliary.h"
#include "agc/nav/integration.h"
#include "agc/programs/alignment.h"
#include "agc/programs/rendezvous.h"

static int g_checks;   /* checks run    */
static int g_failures; /* checks failed */

/* Prototypes: main() comes first (STYLE 4.6). */
static void     check(bool ok, int line, const char *text);
static void     store_block(uint16_t ecadr, const uint16_t *w, int n);
static void     test_refsmmat(agc_vehicle_t v, uint16_t ecadr);
static void     test_state_vector_uplink(agc_vehicle_t v);
static void     test_tephem_and_follow(agc_vehicle_t v);
static void     test_unmapped_and_flags(agc_vehicle_t v);
static void     test_pad_loads(void);
static void     test_lm_landing_pad(void);
static void     test_lm_pad_more(void);
static void     test_phase_tables(agc_vehicle_t v, uint16_t longbase);
static void     test_csm_rcs_dap(void);
static void     test_vectors_stored(void);
static void     test_servicer_start(agc_vehicle_t v, uint16_t rn1, uint16_t gdt, uint16_t rtx);
static uint16_t rd(uint16_t ecadr);
static uint16_t sp_word(agc_sp_t v);
static bool     dp_at(uint16_t ecadr, agc_dp_t v);
static void     test_map_size(void);
static void     test_downlink_fields_csm(void);
static void     test_downlink_entry_dap_csm(void);
static void     test_downlink_fields_lm(void);

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
    test_lm_landing_pad();
    test_lm_pad_more();
    test_phase_tables(AGC_VEHICLE_CSM, 01135u);
    test_phase_tables(AGC_VEHICLE_LM, 01151u);
    test_csm_rcs_dap();
    test_vectors_stored();
    test_servicer_start(AGC_VEHICLE_LM, 03544u, 01236u, 02776u);
    test_servicer_start(AGC_VEHICLE_CSM, 01231u, 01206u, 03745u);
    test_map_size();
    test_downlink_fields_csm();
    test_downlink_entry_dap_csm();
    test_downlink_fields_lm();
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
 * the lunar ephemeris (VECOEM 2036, OMEGAES 2146) reach their typed homes;
 * FANG (1774) reaches both FANG and the low word of K3VAL (EK3VAL 1773).
 **********************************************************************/
static void test_pad_loads(void)
{
    agc_imu_comp_params_t c;   /* compensation read back */
    agc_navaux_padload_t  pad; /* AZO .. */
    agc_ephem_csm_t       eph; /* VECOEM .. */
    uint16_t              w;   /* FANG read back */

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
    CHECK(agc_erase_write_ecadr(01773u, 00552u) == AGC_STATUS_OK);  /* EK3VAL */
    CHECK(agc_erase_write_ecadr(01774u, 02200u) == AGC_STATUS_OK);  /* FANG   */
    CHECK(agc_guid_state()->csm.fang == 02200);
    CHECK(agc_guid_state()->csm.ek3val == agc_dp_pack(0552, 02200));
    agc_erase_read_ecadr(01774u, &w);
    CHECK(w == 02200u);
}

/**********************************************************************
 * AGC: none (test) -- the Luminary099 landing pad loads by address
 * (LMPAD): RBRFG at 2402, TCGIBRAK at 2435, RAPFG at 2436, COSTHET2 at
 * 2600 and TENDAPPR at 3424 reach agc_lm_pad().
 **********************************************************************/
static void test_lm_landing_pad(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    CHECK(agc_erase_write_ecadr(02402u, 00005u) == AGC_STATUS_OK); /* RBRFG x high */
    CHECK(agc_erase_write_ecadr(02435u, 00007u) == AGC_STATUS_OK); /* TCGIBRAK */
    CHECK(agc_erase_write_ecadr(02437u, 00011u) == AGC_STATUS_OK); /* RAPFG x low */
    CHECK(agc_erase_write_ecadr(02600u, 00002u) == AGC_STATUS_OK); /* COSTHET2 high */
    CHECK(agc_erase_write_ecadr(03424u, 00013u) == AGC_STATUS_OK); /* TENDAPPR */

    agc_lm_pad_t *pad = agc_lm_pad(); /* the landing pad loads */

    CHECK(pad->target[0].rfg.c[0] == 5 * 040000);
    CHECK(pad->target[0].tcgi == 7);
    CHECK(pad->target[1].rfg.c[0] == 011);
    CHECK(pad->costhet2 == 2 * 040000);
    CHECK(pad->tendappr == 013);
}

/**********************************************************************
 * AGC: none (test) -- the LM pad loads a scenario restore needs beyond the
 * landing pad (Luminary099 ERASABLE_ASSIGNMENTS): HIASCENT 3000, DKTRAP
 * 3003, IGNAOSR 3013, LRVF 2527, MASS 1244, X789 1700, WRENDVEL 2001,
 * ATIGINC 3400 reach their typed homes.
 *
 * FUNCTION NAME: test_lm_pad_more
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
static void test_lm_pad_more(void)
{
    agc_state_init(AGC_VEHICLE_LM);
    CHECK(agc_erase_write_ecadr(03000u, 02343u) == AGC_STATUS_OK);  /* HIASCENT */
    CHECK(agc_erase_write_ecadr(03003u, 077001u) == AGC_STATUS_OK); /* DKTRAP = -0776 */
    CHECK(agc_erase_write_ecadr(03013u, 00005u) == AGC_STATUS_OK);  /* IGNAOSR */
    CHECK(agc_erase_write_ecadr(02527u, 00012u) == AGC_STATUS_OK);  /* LRVF */
    CHECK(agc_erase_write_ecadr(01244u, 00003u) == AGC_STATUS_OK);  /* MASS high */
    CHECK(agc_erase_write_ecadr(01245u, 00004u) == AGC_STATUS_OK);  /* MASS low */
    CHECK(agc_erase_write_ecadr(01702u, 00006u) == AGC_STATUS_OK);  /* X789 y high */
    CHECK(agc_erase_write_ecadr(02001u, 00017u) == AGC_STATUS_OK);  /* WRENDVEL */
    CHECK(agc_erase_write_ecadr(03401u, 00021u) == AGC_STATUS_OK);  /* ATIGINC low */
    CHECK(*agc_lm_dap_padload(0) == 02343);
    CHECK(*agc_lm_dap_padload(1) == -0776);
    CHECK(agc_guid_state()->lm.ignaosr == 5);
    CHECK(agc_guid_state()->lm.lrvf == 012);
    CHECK(agc_erase_mass() == 3 * 040000 + 4);
    CHECK(agc_erase_x789().c[1] == 6 * 040000);
    CHECK(agc_rendz_nav_pad()->wrendvel == 017);
    CHECK(agc_rendz_erasable()->atiginc == 021);
}

/**********************************************************************
 * AGC: none (test) -- the restart phase table by address (a saved image's
 * -PHASEn PHASEn pairs at 0752, TBASEn at 01053 + 2(n-1), LONGBASE):
 * stored where GOPROG reads them, read back as stored.
 *
 * ARGUMENTS:
 *   v         agc_vehicle_t  I    vehicle
 *   longbase  uint16_t       I    LONGBASE ECADR (CSM 01135, LM 01151)
 **********************************************************************/
static void test_phase_tables(agc_vehicle_t v, uint16_t longbase)
{
    uint16_t w = 0; /* a word read back */

    agc_state_init(v);
    CHECK(agc_erase_write_ecadr(00760u, 077752u) == AGC_STATUS_OK); /* -PHASE4 */
    CHECK(agc_erase_write_ecadr(00761u, 00025u) == AGC_STATUS_OK);  /* PHASE4 = 25 */
    CHECK(agc_erase_write_ecadr(01061u, 01234u) == AGC_STATUS_OK);  /* TBASE4 */
    CHECK(agc_erase_write_ecadr(longbase, 05245u) == AGC_STATUS_OK);
    CHECK(agc_erase_write_ecadr((uint16_t) (longbase + 1u), 032373u) == AGC_STATUS_OK);
    CHECK(agc_phase_get(4) == 025u);
    CHECK(agc_phase_table_word(4, true) == 077752u);
    CHECK(agc_phase_tbase_word(4) == 01234u);
    CHECK(agc_phase_longbase() == ((uint32_t) 05245u << 14 | 032373u));
    agc_erase_read_ecadr(00761u, &w);
    CHECK(w == 025u);
    agc_erase_read_ecadr(01062u, &w);
    CHECK(w == 0u);                                                 /* PHSPRDT4: not kept */
}

/**********************************************************************
 * AGC: none (test) -- the CSM RCS DAP words a restart does not recompute
 * (flight bug 12): IXX, IAVG, IAVG/TLX (03070, MASSPROP), HOLDFLAG
 * (01332), THETADX..Z (03172), DELCDUX..Z (03175, DP) and CDUXD..ZD
 * (03246, DP). Values of the Apollo 11 entry scenario's image: a load
 * that drops them leaves J/M = 0 (no jet ever fires) and a hold
 * reference of 0, 0, 0.
 **********************************************************************/
static void test_csm_rcs_dap(void)
{
    uint16_t                w = 0;                 /* a word read back */
    const agc_csm_rcs_t    *r = agc_csm_dap_rcs(); /* RCS DAP erasables */
    const agc_csm_dapcom_t *c = agc_csm_dap_com(); /* IXX, IAVG */

    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(agc_erase_write_ecadr(03070u, 00504u) == AGC_STATUS_OK);  /* IXX      */
    CHECK(agc_erase_write_ecadr(03071u, 01730u) == AGC_STATUS_OK);  /* IAVG     */
    CHECK(agc_erase_write_ecadr(03072u, 01454u) == AGC_STATUS_OK);  /* IAVG/TLX */
    CHECK(agc_erase_write_ecadr(01332u, 077776u) == AGC_STATUS_OK); /* HOLDFLAG -1 */
    CHECK(agc_erase_write_ecadr(03172u, 00045u) == AGC_STATUS_OK);  /* THETADX  */
    CHECK(agc_erase_write_ecadr(03173u, 027015u) == AGC_STATUS_OK); /* THETADY  */
    CHECK(agc_erase_write_ecadr(03174u, 00001u) == AGC_STATUS_OK);  /* THETADZ  */
    CHECK(agc_erase_write_ecadr(03246u, 00045u) == AGC_STATUS_OK);  /* CDUXD    */
    CHECK(agc_erase_write_ecadr(03247u, 031301u) == AGC_STATUS_OK);
    CHECK(agc_erase_write_ecadr(03252u, 00001u) == AGC_STATUS_OK);  /* CDUZD    */
    CHECK(agc_erase_write_ecadr(03253u, 076025u) == AGC_STATUS_OK);
    CHECK(agc_erase_write_ecadr(03177u, 00002u) == AGC_STATUS_OK);  /* DELCDUY  */
    CHECK(c->ixx == 0504 && c->iavg == 01730 && c->iavg_tlx == 01454);
    CHECK(r->holdflag == -1);
    CHECK(r->thetad[0] == 045u && r->thetad[1] == 027015u && r->thetad[2] == 1u);
    CHECK(r->cdud[0] == (agc_dp_t) 045 * 040000 + 031301);
    CHECK(r->cdud[2] == (agc_dp_t) 040000 - 01752);                 /* 1, -1752: signs free to disagree */
    CHECK(r->delcdu[1] == (agc_dp_t) 2 * 040000);
    agc_erase_read_ecadr(01332u, &w);
    CHECK(w == 077776u);
    agc_erase_read_ecadr(03173u, &w);
    CHECK(w == 027015u);
    agc_erase_read_ecadr(03070u, &w);
    CHECK(w == 00504u);
}

/**********************************************************************
 * AGC: none (test helper) -- store a vector's six words (x hi lo, y, z)
 *
 * ARGUMENTS:
 *   ecadr  uint16_t  I  first address
 *   hi     uint16_t  I  the high word of each component (low words 0)
 **********************************************************************/
static void store_vec(uint16_t ecadr, uint16_t hi)
{
    for (uint16_t i = 0; i < 6u; i++)
    {
        CHECK(agc_erase_write_ecadr((uint16_t) (ecadr + i), ((i % 2u) == 0u) ? hi : 0u) == AGC_STATUS_OK); /* ?: high word, low word 0 */
    }
}

/**********************************************************************
 * AGC: none (test) -- vector erasables reach their own homes (flight
 * bug 13): the LM ascent's VGVECT (03645), LAXIS (02302), QAXIS = UHYP
 * (03717), ZAXIS1 = UHZP (03725) were stored into DELVLVC, and the CM's
 * UNITW (01713, Earth's polar axis; -AYO and AXO are its first four
 * words) was not mapped, so entry targeting (EARROT) lost the Earth's
 * rotation.
 **********************************************************************/
static void test_vectors_stored(void)
{
    const agc_dp_t one = (agc_dp_t) 01234 * 040000; /* high word 01234, low 0, 2^-28 units */

    agc_state_init(AGC_VEHICLE_LM);
    agc_rendz_erasable()->delvlvc = agc_vec_make(0, 0, 0);
    store_vec(03645u, 01234u);
    CHECK(agc_lm_asc()->vgvect.c[0] == one && agc_lm_asc()->vgvect.c[2] == one);
    store_vec(02302u, 01234u);
    CHECK(agc_lm_asc()->laxis.c[1] == one);
    store_vec(03717u, 01234u);
    CHECK(agc_guid_state()->lm.uhyp.c[0] == one);
    store_vec(03725u, 01234u);
    CHECK(agc_guid_state()->lm.uhzp.c[2] == one);
    CHECK(agc_rendz_erasable()->delvlvc.c[0] == 0);

    agc_state_init(AGC_VEHICLE_CSM);
    store_vec(01713u, 037777u);
    CHECK(agc_guid_state()->avg.unitw.c[0] == (agc_dp_t) 037777 * 040000);
    CHECK(agc_guid_state()->avg.unitw.c[2] == (agc_dp_t) 037777 * 040000);
    CHECK(agc_erase_write_ecadr(01717u, 040000u) == AGC_STATUS_OK); /* UNITW z high word -0 */
    uint16_t w = 0;                                                 /* a word read back */

    agc_erase_read_ecadr(01713u, &w);
    CHECK(w == 037777u);
}

/**********************************************************************
 * AGC: none (test) -- the average-G start state MIDTOAV leaves before
 * TIG-30 (flight bug 14): RN1, VN1 (6 words each), PIPTIME1, GDT1/2 and
 * GDT/2, RTX1/RTX2. A scenario saved between MIDTOAV and TIG-30 (lunar
 * liftoff at T-1m) resumes P12 at TIG-35, and PREREAD starts the
 * servicer from these words: unmapped, the LM's position was 0. LM also
 * R(CSM), V(CSM) (UHYP each cycle) and /LAND/ (the vertical rise).
 *
 * ARGUMENTS:
 *   v    agc_vehicle_t  I  vehicle
 *   rn1  uint16_t       I  RN1 ECADR (VN1 +6, PIPTIME1 +14, GDT1/2 +16)
 *   gdt  uint16_t       I  GDT/2 ECADR
 *   rtx  uint16_t       I  RTX1 ECADR (RTX2 +1)
 **********************************************************************/
static void test_servicer_start(agc_vehicle_t v, uint16_t rn1, uint16_t gdt, uint16_t rtx)
{
    const agc_dp_t     one = (agc_dp_t) 01234 * 040000; /* high word 01234, low 0, 2^-28 units */
    const agc_avg_g_t *a   = &agc_guid_state()->avg;    /* average-G block */

    agc_state_init(v);
    store_vec(rn1, 01234u);
    store_vec((uint16_t) (rn1 + 6u), 01234u);
    CHECK(agc_erase_write_ecadr((uint16_t) (rn1 + 12u), 01234u) == AGC_STATUS_OK); /* PIPTIME1 */
    CHECK(agc_erase_write_ecadr((uint16_t) (rn1 + 13u), 0u) == AGC_STATUS_OK);
    store_vec((uint16_t) (rn1 + 14u), 01234u);
    store_vec(gdt, 01234u);
    CHECK(agc_erase_write_ecadr(rtx, 077765u) == AGC_STATUS_OK);                   /* RTX1 -10 */
    CHECK(agc_erase_write_ecadr((uint16_t) (rtx + 1u), 2u) == AGC_STATUS_OK);      /* RTX2 2 */
    CHECK(a->rn1.c[0] == one && a->rn1.c[2] == one);
    CHECK(a->vn1.c[1] == one);
    CHECK(a->piptime1 == one);
    CHECK(a->gdt1_2.c[2] == one);
    CHECK(a->gdt_2.c[0] == one);
    CHECK(agc_rendz_erasable()->body.rtx1 == -10 && agc_rendz_erasable()->body.rtx2 == 2);
    CHECK(agc_rendz_erasable()->body.moon);

    if (v == AGC_VEHICLE_LM)
    {
        /* R(CSM), V(CSM) (01717, 01725): MUNRVG's CSM state, from which NORMLIZE
         * forms UHYP (= QAXIS) every average-G cycle with MUNFLAG set. */
        store_vec(01717u, 01234u);
        store_vec(01725u, 01234u);
        CHECK(agc_guid_state()->lm.r_csm.c[0] == one && agc_guid_state()->lm.v_csm.c[2] == one);

        /* /LAND/ (02332): the site radius CHECKALT ends the vertical rise against */
        CHECK(agc_erase_write_ecadr(02332u, 03237u) == AGC_STATUS_OK);
        CHECK(agc_erase_write_ecadr(02333u, 015100u) == AGC_STATUS_OK);
        CHECK(agc_lm_ldg()->land_mag == (agc_dp_t) 03237 * 040000 + 015100);
    }
}

/**********************************************************************
 * AGC: none (test helper) -- the word at an ECADR
 *
 * ARGUMENTS:
 *   ecadr  uint16_t  I  the address
 *
 * RETURNS: uint16_t -- the word
 **********************************************************************/
static uint16_t rd(uint16_t ecadr)
{
    uint16_t w = 0; /* the word */

    agc_erase_read_ecadr(ecadr, &w);

    return (w);
}

/**********************************************************************
 * AGC: none (test helper) -- a signed SP value as its ones' complement word
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I  the value
 *
 * RETURNS: uint16_t -- the word
 **********************************************************************/
static uint16_t sp_word(agc_sp_t v)
{
    return ((uint16_t) ((v < 0) ? (077777 + v) : v)); /* negative: ones' complement */
}

/**********************************************************************
 * AGC: none (test helper) -- the two words at ecadr are those of DP v
 *
 * ARGUMENTS:
 *   ecadr  uint16_t  I  the address of the high word
 *   v      agc_dp_t  I  the value
 *
 * RETURNS: bool -- true when they are (sign-agreeing words)
 **********************************************************************/
static bool dp_at(uint16_t ecadr, agc_dp_t v)
{
    agc_dp_words_t d = agc_dp_to_words(v); /* sign-agreeing pair */

    return (rd(ecadr) == sp_word(d.hi) && rd((uint16_t) (ecadr + 1u)) == sp_word(d.lo));
}

/**********************************************************************
 * AGC: none -- the map of each rope fits core's shadow store
 **********************************************************************/
static void test_map_size(void)
{
    agc_state_init(AGC_VEHICLE_CSM);
    CHECK(agc_erase_map_words() <= AGC_EC_SHADOW_WORDS);
    agc_state_init(AGC_VEHICLE_LM);
    CHECK(agc_erase_map_words() <= AGC_EC_SHADOW_WORDS);
}

/**********************************************************************
 * AGC: none -- the CSM words the downlists send reach their owners'
 * storage (Comanche055 addresses): a value the program stores shows at
 * the address; at an address two owners share (the AGC's overlays) the
 * one written last shows; a store by address reaches the owner.
 **********************************************************************/
static void test_downlink_fields_csm(void)
{
    agc_rendz_t      *rz  = agc_rendz_erasable(); /* rendezvous erasables */
    agc_csm_rcs_t    *rcs = agc_csm_dap_rcs();    /* RCS DAP */
    agc_csm_tvc_t    *tvc = agc_csm_dap_tvc();    /* TVC DAP */
    agc_csm_dapcom_t *dc  = agc_csm_dap_com();    /* AK, RCSFLAGS */

    agc_state_init(AGC_VEHICLE_CSM);
    rz->delveet1.c[1]      = 01234567;                             /* DELVEET1 +2 */
    CHECK(dp_at(02446u, rz->delveet1.c[1]));
    rz->tcsi               = 0777;                                 /* TCSI = TCDH +2 = DELVTPF (02636) */
    CHECK(dp_at(02636u, 0777));
    rz->delvtpf            = -0123456;                             /* the later writer shows */
    CHECK(dp_at(02636u, -0123456));
    rz->tcsi               = 0555;
    CHECK(dp_at(02636u, 0555));
    dc->ak[2]              = -5;                                   /* AK2 */
    dc->rcsflags           = 012345u;
    CHECK(rd(03100u) == 077772u && rd(03101u) == 012345u);
    rcs->wbody[0]          = 0700000;                              /* WBODY = OMEGAC (03125) */
    CHECK(dp_at(03125u, 0700000));
    tvc->omegac[0]         = -0300;
    CHECK(dp_at(03125u, -0300));
    rcs->adot[1].hi        = 5;                                    /* ADOT +2, +3 as stored */
    rcs->adot[1].lo        = -3;
    CHECK(rd(03135u) == 5u && rd(03136u) == 077774u);
    agc_align_ram()->besti = 0123;                                 /* BESTI = UPVERB (0302) */
    CHECK(rd(0302u) == 0123u);
    agc_time_set_cs(0x1234567u);                                   /* TIME2, TIME1 */
    CHECK(rd(024u) == ((0x1234567u >> 14) & 037777u) && rd(025u) == (0x1234567u & 037777u));
    CHECK(agc_erase_write_ecadr(025u, 5u) == AGC_STATUS_OK);
    CHECK((agc_time_now_cs() & 037777u) == 5u);
    agc_erase_set_cdux(012345u);                                   /* CDUX */
    CHECK(rd(032u) == 012345u);
    CHECK(agc_erase_write_ecadr(037u, 077770u) == AGC_STATUS_OK);  /* PIPAX -7 */
    CHECK(agc_erase_pipax() == -7);
    CHECK(agc_erase_write_ecadr(02751u, 3u) == AGC_STATUS_OK);     /* LANDMARK */
    CHECK(agc_erase_landmark() == 3);
    CHECK(agc_erase_write_ecadr(03414u, 00012u) == AGC_STATUS_OK); /* RTARG X high */
    CHECK(agc_dp_high(rz->rtarg.c[0]) == 012);
} /* test_downlink_fields_csm */

/**********************************************************************
 * AGC: none -- the CSM entry words of the entry downlist (Comanche055
 * addresses): VPRED, GAMMAEI, LATANG, RDOT, THETAH, CMDAPMOD's four values
 **********************************************************************/
static void test_downlink_entry_dap_csm(void)
{
    agc_entry_state_t *es = agc_entry_state(); /* entry erasables */

    agc_state_init(AGC_VEHICLE_CSM);
    es->vpred    = 0200000;
    es->gammaei  = -0400;
    CHECK(dp_at(03766u, 0200000) && dp_at(03770u, -0400));
    es->rdot     = 0333; /* LATANG +2 */
    es->thetah   = 0444;
    CHECK(dp_at(03677u, 0333) && dp_at(03701u, 0444));
    es->cmdapmod = AGC_ENTRY_DAPMOD_MINUS0;
    CHECK(rd(03300u) == 077777u);
    es->cmdapmod = AGC_ENTRY_DAPMOD_MINUS1;
    CHECK(rd(03300u) == 077776u);
    CHECK(agc_erase_write_ecadr(03300u, 1u) == AGC_STATUS_OK);
    CHECK(es->cmdapmod == AGC_ENTRY_DAPMOD_PLUS1);
    es->alfa_180 = -2;
    es->beta_180 = 3;
    CHECK(rd(03265u) == 077775u && rd(03266u) == 3u);
}

/**********************************************************************
 * AGC: none -- the LM words the downlists send reach their owners'
 * storage (Luminary099 addresses), with the overlays of the ascent
 * guidance (DELVEET1 = TTO..) and of the mark time (MKTIME)
 **********************************************************************/
static void test_downlink_fields_lm(void)
{
    agc_rendz_t     *rz  = agc_rendz_erasable();  /* rendezvous erasables */
    agc_lm_ldg_t    *ldg = agc_lm_ldg();          /* landing guidance */
    agc_guid_lm_t   *lm  = &agc_guid_state()->lm; /* LM guidance */
    agc_align_ram_t *ram = agc_align_ram();       /* alignment */

    agc_state_init(AGC_VEHICLE_LM);
    ldg->rgu.c[2]          = -01234567;                            /* RGU +4 */
    CHECK(dp_at(02642u, -01234567));
    ldg->ttf_8             = -0100000;
    CHECK(dp_at(03642u, -0100000));
    agc_lm_thr()->pseudo55 = 0321;
    CHECK(rd(03614u) == 0321u);
    ram->ynbsav.c[0]       = 0111;
    ram->znbsav.c[2]       = 0222;                                 /* YNBSAV +10D = ZNBSAV +4 */
    CHECK(dp_at(02236u, 0111) && dp_at(02250u, 0222));
    lm->lrcdudl[2]         = 054321u;                              /* LRZCDUDL */
    lm->hmeas              = 0777777;
    CHECK(rd(02336u) == 054321u && dp_at(03654u, 0777777));
    CHECK(agc_erase_write_ecadr(02266u, 00007u) == AGC_STATUS_OK); /* TTO high (ascent) = DELVEET1 */
    CHECK(rd(02266u) == 00007u);
    rz->delveet1.c[0]      = 0654321;                              /* the rendezvous program writes DELVEET1 */
    CHECK(dp_at(02266u, 0654321));
    lm->mktime             = 01000;                                /* MKTIME of the LR mark */
    CHECK(dp_at(03754u, 01000));
    CHECK(agc_erase_write_ecadr(03243u, 0100u) == AGC_STATUS_OK);  /* OMEGAPD */
    CHECK(agc_erase_omegapd() == 0100);
    CHECK(agc_erase_write_ecadr(02636u, 00003u) == AGC_STATUS_OK); /* RGU X high */
    CHECK(agc_dp_high(ldg->rgu.c[0]) == 3);
} /* test_downlink_fields_lm */
